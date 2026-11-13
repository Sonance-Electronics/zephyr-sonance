/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * CMSIS-DAP TCP backend.
 *
 * Every request and response is one CMSIS-DAP packet preceded by an 8-byte
 * little-endian header, as used by the OpenOCD cmsis-dap tcp backend:
 *
 *   uint32_t signature    "DAP\0" (0x00504144)
 *   uint16_t length       payload length, header excluded
 *   uint8_t  packet_type  1 = request, 2 = response
 *   uint8_t  reserved     0
 *
 * One client is served at a time; further connections are closed as soon
 * as they are accepted. The client owns the debug port from its DAP_Connect
 * until it disconnects, and the port is released when the client goes away.
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/dap/dap_link.h>

#include "cmsis_dap.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dap_tcp, CONFIG_DAP_LOG_LEVEL);

#define DAP_TCP_SIGNATURE     0x00504144U
#define DAP_TCP_TYPE_REQUEST  0x01U
#define DAP_TCP_TYPE_RESPONSE 0x02U
#define DAP_TCP_HDR_SIZE      8U

#define DAP_TCP_PKT_SIZE CONFIG_DAP_BACKEND_TCP_PKT_SIZE

/*
 * DAP_Transfer is not bounded by the packet size: its up to 255 entries
 * take at most 5 bytes of request and 4 bytes of response each. Size the
 * buffers so that a malformed request cannot run past them.
 */
#define DAP_TCP_XFER_MAX (3U + 255U * 5U)
#define DAP_TCP_BUF_SIZE MAX(DAP_TCP_PKT_SIZE, DAP_TCP_XFER_MAX)

#if defined(CONFIG_DAP_BACKEND_TCP_KEEPALIVE)
/* Drop a silent client after about 10 seconds */
#define DAP_TCP_KEEPIDLE_S  2
#define DAP_TCP_KEEPINTVL_S 2
#define DAP_TCP_KEEPCNT     4
#endif

static K_THREAD_STACK_DEFINE(dap_tcp_stack, CONFIG_DAP_BACKEND_TCP_STACK_SIZE);
static struct k_thread dap_tcp_thread;
static struct dap_link_context *dap_tcp_ctx;

static uint8_t dap_tcp_request[DAP_TCP_BUF_SIZE];
static uint8_t dap_tcp_response[DAP_TCP_HDR_SIZE + DAP_TCP_BUF_SIZE];

static int dap_tcp_recv_all(int fd, uint8_t *buf, size_t len)
{
	while (len > 0U) {
		ssize_t ret = zsock_recv(fd, buf, len, 0);

		if (ret == 0) {
			return -ENOTCONN;
		}

		if (ret < 0) {
			return -errno;
		}

		buf += ret;
		len -= (size_t)ret;
	}

	return 0;
}

static int dap_tcp_send_all(int fd, const uint8_t *buf, size_t len)
{
	while (len > 0U) {
		ssize_t ret = zsock_send(fd, buf, len, 0);

		if (ret < 0) {
			return -errno;
		}

		buf += ret;
		len -= (size_t)ret;
	}

	return 0;
}

static int dap_tcp_handle_packet(int fd)
{
	uint8_t hdr[DAP_TCP_HDR_SIZE];
	uint32_t signature;
	uint16_t length;
	uint32_t rsp_len;
	int ret;

	ret = dap_tcp_recv_all(fd, hdr, sizeof(hdr));
	if (ret != 0) {
		return ret;
	}

	signature = sys_get_le32(&hdr[0]);
	length = sys_get_le16(&hdr[4]);

	if (signature != DAP_TCP_SIGNATURE || hdr[6] != DAP_TCP_TYPE_REQUEST) {
		LOG_ERR("Invalid packet header");
		return -EPROTO;
	}

	if (length == 0U || length > DAP_TCP_PKT_SIZE) {
		LOG_ERR("Invalid packet length %u", length);
		return -EMSGSIZE;
	}

	ret = dap_tcp_recv_all(fd, dap_tcp_request, length);
	if (ret != 0) {
		return ret;
	}

	/* Commands may read past a short request; never let them see stale data */
	memset(&dap_tcp_request[length], 0, sizeof(dap_tcp_request) - length);

	rsp_len = dap_link_execute_cmd(dap_tcp_ctx, dap_tcp_request,
				       &dap_tcp_response[DAP_TCP_HDR_SIZE]);
	if (rsp_len > DAP_TCP_BUF_SIZE) {
		LOG_ERR("Response length %u exceeds buffer", rsp_len);
		return -EMSGSIZE;
	}

	sys_put_le32(DAP_TCP_SIGNATURE, &dap_tcp_response[0]);
	sys_put_le16((uint16_t)rsp_len, &dap_tcp_response[4]);
	dap_tcp_response[6] = DAP_TCP_TYPE_RESPONSE;
	dap_tcp_response[7] = 0U;

	return dap_tcp_send_all(fd, dap_tcp_response, DAP_TCP_HDR_SIZE + rsp_len);
}

static void dap_tcp_setup_client(int fd)
{
	struct zsock_timeval tv = {
		.tv_sec = CONFIG_DAP_BACKEND_TCP_SEND_TIMEOUT_MS / 1000,
		.tv_usec = (CONFIG_DAP_BACKEND_TCP_SEND_TIMEOUT_MS % 1000) * 1000,
	};
	int one = 1;

	if (zsock_setsockopt(fd, NET_IPPROTO_TCP, ZSOCK_TCP_NODELAY, &one, sizeof(one)) < 0) {
		LOG_WRN("Failed to set TCP_NODELAY (%d)", errno);
	}

	if (zsock_setsockopt(fd, ZSOCK_SOL_SOCKET, ZSOCK_SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
		LOG_WRN("Failed to set send timeout (%d)", errno);
	}

#if defined(CONFIG_DAP_BACKEND_TCP_KEEPALIVE)
	int idle = DAP_TCP_KEEPIDLE_S;
	int intvl = DAP_TCP_KEEPINTVL_S;
	int cnt = DAP_TCP_KEEPCNT;

	if (zsock_setsockopt(fd, ZSOCK_SOL_SOCKET, ZSOCK_SO_KEEPALIVE, &one, sizeof(one)) < 0 ||
	    zsock_setsockopt(fd, NET_IPPROTO_TCP, ZSOCK_TCP_KEEPIDLE, &idle, sizeof(idle)) < 0 ||
	    zsock_setsockopt(fd, NET_IPPROTO_TCP, ZSOCK_TCP_KEEPINTVL, &intvl, sizeof(intvl)) < 0 ||
	    zsock_setsockopt(fd, NET_IPPROTO_TCP, ZSOCK_TCP_KEEPCNT, &cnt, sizeof(cnt)) < 0) {
		LOG_WRN("Failed to enable keepalive (%d)", errno);
	}
#endif
}

static int dap_tcp_listen(void)
{
	struct net_sockaddr_in addr = {
		.sin_family = NET_AF_INET,
		.sin_addr.s_addr = net_htonl(NET_INADDR_ANY),
		.sin_port = net_htons(CONFIG_DAP_BACKEND_TCP_PORT),
	};
	int one = 1;
	int fd;

	fd = zsock_socket(NET_AF_INET, NET_SOCK_STREAM, NET_IPPROTO_TCP);
	if (fd < 0) {
		LOG_ERR("Failed to create socket (%d)", errno);
		return -errno;
	}

	if (zsock_setsockopt(fd, ZSOCK_SOL_SOCKET, ZSOCK_SO_REUSEADDR, &one, sizeof(one)) < 0) {
		LOG_WRN("Failed to set SO_REUSEADDR (%d)", errno);
	}

	if (zsock_bind(fd, (struct net_sockaddr *)&addr, sizeof(addr)) < 0 ||
	    zsock_listen(fd, 1) < 0) {
		LOG_ERR("Failed to listen on port %u (%d)", CONFIG_DAP_BACKEND_TCP_PORT, errno);
		(void)zsock_close(fd);
		return -errno;
	}

	return fd;
}

static int dap_tcp_accept(int listen_fd, const char *what)
{
	struct net_sockaddr_in peer;
	net_socklen_t len = sizeof(peer);
	char addr_str[NET_INET_ADDRSTRLEN];
	int fd;

	fd = zsock_accept(listen_fd, (struct net_sockaddr *)&peer, &len);
	if (fd < 0) {
		LOG_ERR("Failed to accept (%d)", errno);
		return -errno;
	}

	if (net_addr_ntop(NET_AF_INET, &peer.sin_addr, addr_str, sizeof(addr_str)) == NULL) {
		strcpy(addr_str, "?");
	}

	LOG_INF("%s %s:%u", what, addr_str, net_ntohs(peer.sin_port));

	return fd;
}

static void dap_tcp_serve(int listen_fd, int client_fd)
{
	struct zsock_pollfd fds[2] = {
		{.fd = client_fd, .events = ZSOCK_POLLIN},
		{.fd = listen_fd, .events = ZSOCK_POLLIN},
	};

	while (true) {
		int ret = zsock_poll(fds, ARRAY_SIZE(fds), -1);

		if (ret < 0) {
			LOG_ERR("Poll failed (%d)", errno);
			return;
		}

		if ((fds[1].revents & ZSOCK_POLLIN) != 0) {
			int extra = dap_tcp_accept(listen_fd, "Rejected client, busy:");

			if (extra >= 0) {
				(void)zsock_close(extra);
			}
		}

		if ((fds[0].revents & (ZSOCK_POLLERR | ZSOCK_POLLHUP | ZSOCK_POLLNVAL)) != 0) {
			return;
		}

		if ((fds[0].revents & ZSOCK_POLLIN) != 0) {
			ret = dap_tcp_handle_packet(client_fd);
			if (ret != 0) {
				if (ret != -ENOTCONN) {
					LOG_WRN("Closing client (%d)", ret);
				}

				return;
			}
		}
	}
}

static void dap_tcp_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int listen_fd;

	while (true) {
		listen_fd = dap_tcp_listen();
		if (listen_fd >= 0) {
			break;
		}

		k_sleep(K_SECONDS(1));
	}

	LOG_INF("Listening on port %u", CONFIG_DAP_BACKEND_TCP_PORT);

	while (true) {
		int client_fd = dap_tcp_accept(listen_fd, "Client connected from");

		if (client_fd < 0) {
			k_sleep(K_MSEC(100));
			continue;
		}

		dap_tcp_setup_client(client_fd);
		dap_tcp_serve(listen_fd, client_fd);

		(void)zsock_close(client_fd);
		(void)dap_link_release(dap_tcp_ctx);
		LOG_INF("Client disconnected");
	}
}

int dap_link_backend_tcp_init(struct dap_link_context *const dap_link_ctx)
{
	if (dap_tcp_ctx != NULL) {
		return -EALREADY;
	}

	dap_tcp_ctx = dap_link_ctx;
	dap_link_set_pkt_size(dap_link_ctx, DAP_TCP_PKT_SIZE);

	k_thread_create(&dap_tcp_thread, dap_tcp_stack, K_THREAD_STACK_SIZEOF(dap_tcp_stack),
			dap_tcp_thread_fn, NULL, NULL, NULL,
			K_PRIO_PREEMPT(CONFIG_DAP_BACKEND_TCP_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&dap_tcp_thread, "dap_tcp");

	return 0;
}
