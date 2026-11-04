/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Espressif I2S driver extensions
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_I2S_I2S_ESP32_H_
#define ZEPHYR_INCLUDE_DRIVERS_I2S_I2S_ESP32_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Called as each received block completes.
 *
 * With CONFIG_I2S_ESP32_RX_BLOCK_HOOK, the driver calls this from its DMA
 * completion interrupt when the last sample of @p mem_block has been
 * written, just before queueing the block for i2s_read(). The default does
 * nothing; an application overrides it, for example to timestamp the
 * block. It runs in interrupt context and must be placed in IRAM.
 *
 * @param dev I2S device.
 * @param mem_block The block i2s_read() will return.
 */
void i2s_esp32_rx_block_done(const struct device *dev, void *mem_block);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_I2S_I2S_ESP32_H_ */
