/*
 * Copyright 2023-2024 NXP
 *
 * Based on a commit to drivers/ethernet/eth_mcux.c which was:
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_enet_ptp_clock

#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/ethernet/eth_nxp_enet.h>
#include <zephyr/logging/log.h>

#include <math.h>

#include <fsl_enet.h>

LOG_MODULE_REGISTER(ptp_clock_nxp_enet, LOG_LEVEL_INF);

struct ptp_clock_nxp_enet_config {
	const struct pinctrl_dev_config *pincfg;
	const struct device *module_dev;
	const struct device *port;
	const struct device *clock_dev;
	struct device *clock_subsys;
	void (*irq_config_func)(void);
};

struct ptp_clock_nxp_enet_data {
	ENET_Type *base;
	enet_handle_t *enet_handle;
	struct k_mutex ptp_mutex;
};

static int ptp_clock_nxp_enet_set(const struct device *dev,
				struct net_ptp_time *tm)
{
	struct ptp_clock_nxp_enet_data *data = dev->data;
	enet_ptp_time_t enet_time;

	enet_time.second = tm->second;
	enet_time.nanosecond = tm->nanosecond;

	ENET_Ptp1588SetTimer(data->base, data->enet_handle, &enet_time);

	return 0;
}

static int ptp_clock_nxp_enet_get(const struct device *dev,
				struct net_ptp_time *tm)
{
	struct ptp_clock_nxp_enet_data *data = dev->data;
	enet_ptp_time_t enet_time;

	ENET_Ptp1588GetTimer(data->base, data->enet_handle, &enet_time);

	tm->second = enet_time.second;
	tm->nanosecond = enet_time.nanosecond;

	return 0;
}

static int ptp_clock_nxp_enet_adjust(const struct device *dev,
					int increment)
{
	struct ptp_clock_nxp_enet_data *data = dev->data;
	int ret = 0;
	int key;

	if ((increment <= (int32_t)(-NSEC_PER_SEC)) ||
			(increment >= (int32_t)NSEC_PER_SEC)) {
		ret = -EINVAL;
	} else {
		key = irq_lock();
		if (data->base->ATPER != NSEC_PER_SEC) {
			ret = -EBUSY;
		} else {
			/* Seconds counter is handled by software. Change the
			 * period of one software second to adjust the clock.
			 */
			data->base->ATPER = NSEC_PER_SEC - increment;
			ret = 0;
		}
		irq_unlock(key);
	}

	return ret;

}

static uint32_t ptp_clock_nxp_enet_gcd(uint32_t a, uint32_t b)
{
	while (b != 0) {
		uint32_t t = b;

		b = a % b;
		a = t;
	}

	return a;
}

/*
 * Add corr ns instead of ATINC.INC once every period ticks.
 *
 * The hardware applies the correction every ATCOR.COR + 1 ticks, not every
 * COR, so the register takes period - 1; ENET_Ptp1588AdjustTimer() writes
 * its argument verbatim. The reference manual's "after how many timer clock
 * cycles the correction counter should be reset" reads as COR, but on an
 * RT1176 at 24 MHz, COR = 3 with INC = 41 and INC_CORR = 43 ran the timer
 * at (3 * 41 + 43) / 4 = 41.5 ns/tick, 4016 ppm slow: a crystal-derived SAI
 * bit clock measured over a PTP second read 4011-4014 ppm fast, and
 * rewriting COR to 2 live brought it to within 1 ppm.
 *
 * A period of 1 is not representable, since COR = 0 disables correction.
 */
static void ptp_clock_nxp_enet_set_correction(ENET_Type *base, uint32_t corr,
					      uint32_t period)
{
	__ASSERT_NO_MSG(period >= 2U);
	ENET_Ptp1588AdjustTimer(base, corr, period - 1U);
}

/*
 * ENET_Ptp1588StartTimer() sets ATINC.INC to floor(NSEC_PER_SEC / clk_hz),
 * truncating any fractional ns/tick (e.g. 24 MHz -> 41.667 ns/tick truncates
 * to 41, a static ~1.6% slow bias). The original upstream implementation of
 * this function only ever tried corr = hw_inc +/- 1 relative to that
 * truncated integer, so it could only reach +/-1/(2*hw_inc) (~1.22% here) --
 * smaller than the 1.6% truncation bias, so it either silently rejected
 * every correction (-EINVAL, discarded by the PTP servo caller) or, once the
 * offset was small enough to request a ratio inside that range, applied a
 * *worse* approximation (INC_CORR=42/ATCOR=2, average 41.33 ns/tick given
 * the COR + 1 period above) that
 * clobbered any better-informed correction already in the registers. Either
 * way the true ~1.6% bias was never actually cancelled: confirmed on
 * hardware -- the local/grandmaster 1PPS edges drifted at the same rate
 * whether the exact correction below was in place or the servo's coarse one
 * had overwritten it.
 *
 * Fix: always compute the correction against the *exact* target rate
 * (NSEC_PER_SEC / clk_hz as a real number, not its integer floor) with the
 * requested servo ratio applied on top, then search for the (corr, period)
 * pair -- bounded by the 7-bit INC_CORR field -- that best approximates it.
 * This bakes the truncation-bias cancellation into every call instead of a
 * separate one-time fix the servo can throw away.
 */
static int ptp_clock_nxp_enet_rate_adjust(const struct device *dev,
					double ratio)
{
	const struct ptp_clock_nxp_enet_config *config = dev->config;
	struct ptp_clock_nxp_enet_data *data = dev->data;
	uint32_t clk_hz;
	uint32_t hw_inc;
	uint32_t corr_max = ENET_ATINC_INC_CORR_MASK >> ENET_ATINC_INC_CORR_SHIFT;
	double target;
	double delta;
	double best_err = -1.0;
	uint32_t best_period = 0;
	int32_t best_corr = 0;
	uint32_t period;

	(void) clock_control_get_rate(config->clock_dev, config->clock_subsys,
				&clk_hz);

	hw_inc = NSEC_PER_SEC / clk_hz;
	target = (double)NSEC_PER_SEC / (double)clk_hz;

	/* No change needed. */
	if ((ratio > 1.0 && ratio - 1.0 < 0.00000001) ||
	   (ratio < 1.0 && 1.0 - ratio < 0.00000001)) {
		ratio = 1.0;
	}

	/* Extra ns/tick needed, on top of the truncated INC, to realize the
	 * exact target rate times the servo's requested ratio.
	 */
	delta = target * ratio - (double)hw_inc;

	/* From 2: see ptp_clock_nxp_enet_set_correction(). */
	for (period = 2; period <= corr_max; period++) {
		int32_t corr = (int32_t)hw_inc +
			(int32_t)floor(delta * period + 0.5);
		double err;

		if (corr < 0 || corr > (int32_t)corr_max) {
			continue;
		}

		err = fabs((double)(corr - (int32_t)hw_inc) / period - delta);

		if (best_period == 0 || err < best_err) {
			best_period = period;
			best_corr = corr;
			best_err = err;
		}
	}

	if (best_period == 0) {
		/* delta is always small (truncation remainder plus a tiny PI
		 * trim); this should not happen in practice.
		 */
		return -EINVAL;
	}

	k_mutex_lock(&data->ptp_mutex, K_FOREVER);

	ptp_clock_nxp_enet_set_correction(data->base, (uint32_t)best_corr, best_period);

	k_mutex_unlock(&data->ptp_mutex);

	return 0;
}

/* Seed the same exact correction at startup, before the PTP servo has ever
 * called ptp_clock_nxp_enet_rate_adjust() (ratio == 1.0 case above), so the
 * truncation bias is already cancelled during the pre-sync/pre-attach
 * window.
 */
static void ptp_clock_nxp_enet_correct_baseline_rate(ENET_Type *base, uint32_t clk_hz)
{
	uint32_t hw_inc = NSEC_PER_SEC / clk_hz;
	uint32_t remainder = NSEC_PER_SEC % clk_hz;
	uint32_t g;
	uint32_t delta;
	uint32_t period;
	uint32_t corr_max = ENET_ATINC_INC_CORR_MASK >> ENET_ATINC_INC_CORR_SHIFT;

	if (remainder == 0) {
		/* Exact divisor (e.g. 250 MHz -> 4 ns/tick); nothing to correct. */
		return;
	}

	g = ptp_clock_nxp_enet_gcd(remainder, clk_hz);
	delta = remainder / g;
	period = clk_hz / g;

	if (hw_inc + delta > corr_max) {
		LOG_WRN("Cannot fully correct %u Hz PTP clock source truncation "
			"(need INC_CORR=%u, max %u); PTP will not converge",
			clk_hz, hw_inc + delta, corr_max);
		return;
	}

	/* period >= 2: g divides remainder, which is less than clk_hz. */
	ptp_clock_nxp_enet_set_correction(base, hw_inc + delta, period);

	LOG_INF("PTP clock baseline: %u Hz source, INC=%u ns, "
		"INC_CORR=%u ns every %u ticks (exact avg %u.%06u ns/tick)",
		clk_hz, hw_inc, hw_inc + delta, period,
		hw_inc, (uint32_t)(((uint64_t)remainder * 1000000U) / clk_hz));
}

void nxp_enet_ptp_clock_callback(const struct device *dev,
			enum nxp_enet_callback_reason event,
			void *cb_data)
{
	const struct ptp_clock_nxp_enet_config *config = dev->config;
	struct ptp_clock_nxp_enet_data *data = dev->data;
	struct nxp_enet_ptp_data *ptp_data;

	__ASSERT(cb_data != NULL, "ptp data is NULL");

	ptp_data = (struct nxp_enet_ptp_data *)cb_data;

	if (event == NXP_ENET_MODULE_RESET) {
		enet_ptp_config_t ptp_config;
		uint32_t enet_ref_pll_rate;
		uint8_t ptp_multicast[6] = { 0x01, 0x1B, 0x19, 0x00, 0x00, 0x00 };
		uint8_t ptp_peer_multicast[6] = { 0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E };

		(void) clock_control_get_rate(config->clock_dev, config->clock_subsys,
					&enet_ref_pll_rate);

		ENET_AddMulticastGroup(data->base, ptp_multicast);
		ENET_AddMulticastGroup(data->base, ptp_peer_multicast);

		/* only for ERRATA_2579 */
		ptp_config.channel = kENET_PtpTimerChannel3;
		ptp_config.ptp1588ClockSrc_Hz = enet_ref_pll_rate;

		/* Share the mutex with mac driver */
		ptp_data->ptp_mutex = &data->ptp_mutex;
		/* Get enet handle from mac driver */
		data->enet_handle = ptp_data->enet;

		ENET_Ptp1588SetChannelMode(data->base, kENET_PtpTimerChannel3,
				kENET_PtpChannelPulseHighonCompare, true);
		ENET_Ptp1588StartTimer(data->base, ptp_config.ptp1588ClockSrc_Hz);
		ptp_clock_nxp_enet_correct_baseline_rate(data->base, ptp_config.ptp1588ClockSrc_Hz);
		ENET_EnableInterrupts(data->base, ENET_TS_INTERRUPT);
	}
}

static int ptp_clock_nxp_enet_init(const struct device *port)
{
	const struct ptp_clock_nxp_enet_config *config = port->config;
	struct ptp_clock_nxp_enet_data *data = port->data;
	int ret;

	data->base = (ENET_Type *)DEVICE_MMIO_GET(config->module_dev);

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		return ret;
	}

	k_mutex_init(&data->ptp_mutex);

	config->irq_config_func();

	return 0;
}

static void ptp_clock_nxp_enet_isr(const struct device *dev)
{
	struct ptp_clock_nxp_enet_data *data = dev->data;
	enet_ptp_timer_channel_t channel;

	unsigned int irq_lock_key = irq_lock();

	/* clear channel */
	for (channel = kENET_PtpTimerChannel1; channel <= kENET_PtpTimerChannel4; channel++) {
		if (ENET_Ptp1588GetChannelStatus(data->base, channel)) {
			ENET_Ptp1588ClearChannelStatus(data->base, channel);
		}
	}

	ENET_TimeStampIRQHandler(data->base, data->enet_handle);

	irq_unlock(irq_lock_key);
}

static DEVICE_API(ptp_clock, ptp_clock_nxp_enet_api) = {
	.set = ptp_clock_nxp_enet_set,
	.get = ptp_clock_nxp_enet_get,
	.adjust = ptp_clock_nxp_enet_adjust,
	.rate_adjust = ptp_clock_nxp_enet_rate_adjust,
};

#define PTP_CLOCK_NXP_ENET_INIT(n)						\
	static void nxp_enet_ptp_clock_##n##_irq_config_func(void)		\
	{									\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, 0, irq),			\
				DT_INST_IRQ_BY_IDX(n, 0, priority),		\
				ptp_clock_nxp_enet_isr,				\
				DEVICE_DT_INST_GET(n),				\
				0);						\
		irq_enable(DT_INST_IRQ_BY_IDX(n, 0, irq));			\
	}									\
										\
	PINCTRL_DT_INST_DEFINE(n);						\
										\
	static const struct ptp_clock_nxp_enet_config				\
		ptp_clock_nxp_enet_##n##_config = {				\
			.module_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),		\
			.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
			.port = DEVICE_DT_INST_GET(n),				\
			.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),	\
			.clock_subsys = (void *)				\
					DT_INST_CLOCKS_CELL_BY_IDX(n, 0, name),	\
			.irq_config_func =					\
				nxp_enet_ptp_clock_##n##_irq_config_func,	\
		};								\
										\
	static struct ptp_clock_nxp_enet_data ptp_clock_nxp_enet_##n##_data;	\
										\
	DEVICE_DT_INST_DEFINE(n, &ptp_clock_nxp_enet_init, NULL,		\
				&ptp_clock_nxp_enet_##n##_data,			\
				&ptp_clock_nxp_enet_##n##_config,		\
				POST_KERNEL, CONFIG_PTP_CLOCK_INIT_PRIORITY,	\
				&ptp_clock_nxp_enet_api);

DT_INST_FOREACH_STATUS_OKAY(PTP_CLOCK_NXP_ENET_INIT)
