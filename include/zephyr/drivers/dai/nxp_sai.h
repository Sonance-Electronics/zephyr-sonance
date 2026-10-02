/*
 * Copyright 2026 Sonance
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief FIFO error reporting for the NXP SAI DAI driver.
 *
 * A FIFO error -- an RX overrun or a TX underrun -- loses words from the
 * middle of a frame. The SAI resumes at the start of the next frame, but the
 * partial frame already moved through DMA stays in the stream, so every
 * later word lands in the wrong slot. For a multichannel TDM stream that
 * means the channels are permanently swapped, and it cannot be undone from
 * inside the driver, since the consumer owns the DMA buffers holding the
 * partial frame.
 *
 * With CONFIG_DAI_NXP_SAI_STOP_ON_FIFO_ERROR, the driver halts the affected
 * direction on the first error and moves it to DAI_STATE_ERROR, so no
 * misaligned data follows. The consumer then stops its DMA, re-arms it from
 * the start of a buffer, and calls dai_nxp_sai_recover(), after which the
 * first word is slot 0 of a frame. The direction stays enabled throughout,
 * so a bit clock and frame sync it generates keep running: a codec clocked
 * from them never sees them stop, which many treat as a fault and answer by
 * muting and ramping back up. DAI_TRIGGER_STOP followed by DAI_TRIGGER_START
 * also recovers, but does stop the clocks.
 *
 * The DAI API has no error reporting of its own, so these calls let the
 * consumer find out.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DAI_NXP_SAI_H_
#define ZEPHYR_INCLUDE_DRIVERS_DAI_NXP_SAI_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dai.h>

#ifdef __cplusplus
extern "C" {
#endif

/** FIFO error status of one direction. */
struct dai_nxp_sai_status {
	/** Current state; DAI_STATE_ERROR after a FIFO error halted it. */
	enum dai_state state;
	/**
	 * FIFO errors since boot reported as for dai_nxp_sai_error_cb_t,
	 * whether or not they halted the direction.
	 */
	uint32_t fifo_errors;
};

/**
 * @brief Called from the SAI's ISR on a FIFO error.
 *
 * Reported for a direction that is running or paused. Not reported: an
 * error in the last frame of a stopped direction, whose data STOP discards
 * anyway; an error in a direction the hardware enabled without a START, as
 * the ASYNC direction is in synchronous mode; and a repeat on a direction
 * already halted, until it is recovered or restarted.
 *
 * @param dev SAI device
 * @param dir direction that saw the error
 * @param halted true if the direction was halted and is now in
 *               DAI_STATE_ERROR
 * @param user_data as passed to dai_nxp_sai_set_error_callback()
 */
typedef void (*dai_nxp_sai_error_cb_t)(const struct device *dev, enum dai_dir dir, bool halted,
				       void *user_data);

/**
 * @brief Read the FIFO error status of one direction.
 *
 * Safe to call from an ISR.
 *
 * @param dev SAI device
 * @param dir DAI_DIR_RX or DAI_DIR_TX
 * @param status filled in on success
 *
 * @retval 0 success
 * @retval -EINVAL bad direction or NULL @p status
 */
int dai_nxp_sai_get_status(const struct device *dev, enum dai_dir dir,
			   struct dai_nxp_sai_status *status);

/**
 * @brief Resume a direction halted by a FIFO error, without stopping it.
 *
 * Empties the FIFO and lets the direction resume at the start of the next
 * frame, keeping the transmitter/receiver -- and any clocks it generates --
 * running. The consumer must already have stopped its DMA and re-armed it
 * from the start of a buffer, since the partial frame that caused the error
 * may already be in memory. Safe to call from an ISR.
 *
 * @param dev SAI device
 * @param dir DAI_DIR_RX or DAI_DIR_TX
 *
 * @retval 0 the direction is RUNNING again
 * @retval -EPERM the direction is not in DAI_STATE_ERROR
 * @retval -EINVAL bad direction
 */
int dai_nxp_sai_recover(const struct device *dev, enum dai_dir dir);

/**
 * @brief Register a callback for FIFO errors.
 *
 * One callback per device, for both directions; NULL removes it. It runs in
 * the SAI's ISR, so it must not block, and a recovery that needs
 * dai_trigger() has to be handed to a thread.
 *
 * @param dev SAI device
 * @param cb callback, or NULL
 * @param user_data passed to @p cb
 *
 * @retval 0 success
 */
int dai_nxp_sai_set_error_callback(const struct device *dev, dai_nxp_sai_error_cb_t cb,
				   void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_DAI_NXP_SAI_H_ */
