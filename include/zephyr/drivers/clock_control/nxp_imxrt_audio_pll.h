/*
 * Copyright 2026 Sonance
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Runtime frequency trim for the i.MX RT audio PLL.
 *
 * Intended for disciplining an audio media clock to an external time
 * reference such as PTP. The audio PLL feeds every SAI on the SoC, and its
 * fractional numerator is the only fine control over that clock: everything
 * below it, the CCM root divider and the SAI's own bit clock divider, is
 * integer.
 *
 * This is deliberately not expressed through clock_control_set_rate(). A
 * servo wants to nudge a frequency by a fractional amount and needs to know
 * what it actually got; a target rate in Hz cannot express a sub-ppm
 * correction to a 393 MHz clock.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_NXP_IMXRT_AUDIO_PLL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_NXP_IMXRT_AUDIO_PLL_H_

#include <stdint.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nxp_imxrt_audio_pll NXP i.MX RT audio PLL
 * @ingroup clock_control_interface
 * @brief Sample rate family selection and fractional trim of the shared
 *        audio clock.
 * @{
 */

/**
 * @brief Take the audio PLL's analog-interface lock.
 *
 * The audio PLL's registers sit behind a shared analog interface: each
 * access is a sequence of register writes (direction, address, data,
 * toggle), so two accesses that interleave corrupt each other -- one
 * caller's data can land in a register the other addressed, or a read can
 * turn into a write. Every function in this file takes this lock. Code
 * outside this driver that reaches the audio PLL through the SDK, such as
 * a clock controller computing a SAI root rate with
 * CLOCK_GetRootClockFreq(), must hold it around that call too.
 *
 * A mutex: it may sleep, and it is a no-op before the kernel starts, when
 * there is only one context. It is also a no-op in an ISR, which cannot
 * take it, so an ISR must not access the audio PLL.
 *
 * @param dev audio PLL device
 */
void nxp_imxrt_audio_pll_lock(const struct device *dev);

/**
 * @brief Release the lock taken by nxp_imxrt_audio_pll_lock().
 *
 * @param dev audio PLL device
 */
void nxp_imxrt_audio_pll_unlock(const struct device *dev);

/**
 * @brief Largest offset this driver will apply, in parts per billion.
 *
 * A safety net, not a hardware limit. The 30-bit numerator could pull the
 * clock by several thousand ppm, which is far enough to put the media clock
 * somewhere useless if a servo ever runs away. Callers should clamp to
 * whatever their own loop needs, typically a few hundred ppm at most.
 */
#define NXP_IMXRT_AUDIO_PLL_MAX_OFFSET_PPB 1000000

/** @brief Audio sample rate families the PLL can be configured for. */
enum nxp_imxrt_audio_pll_family {
	/** 48 kHz and its multiples; the devicetree nominal, set up at init. */
	NXP_IMXRT_AUDIO_PLL_FAMILY_48K,
	/** 44.1 kHz and its multiples; needs the 44k1 devicetree properties. */
	NXP_IMXRT_AUDIO_PLL_FAMILY_44K1,
};

/**
 * @brief Switch the audio PLL between sample rate families.
 *
 * 44.1 kHz rates are not reachable by dividing a 48 kHz-family PLL, so the
 * PLL itself has to be reconfigured. Unlike the trim below, this bypasses
 * and re-locks the PLL, dropping the bit clock for as long as the lock
 * takes: stop audio first, and reconfigure the SAI and codecs afterwards.
 *
 * Any trim offset is discarded, and the PLL returns to the exact nominal
 * for the new family. A servo should re-apply its correction after the
 * switch rather than assuming it carried over.
 *
 * Calling this for the family already in effect still re-locks, so callers
 * that care should track the current family themselves.
 *
 * @param dev    audio PLL device
 * @param family rate family to configure for
 *
 * @retval 0 on success
 * @retval -EINVAL unknown family
 * @retval -ENOSYS the devicetree has no configuration for that family
 * @retval -ENOTSUP on a SoC without this PLL
 */
int nxp_imxrt_audio_pll_set_family(const struct device *dev,
				   enum nxp_imxrt_audio_pll_family family);

/**
 * @brief Offset the audio PLL from its devicetree nominal frequency.
 *
 * Writes only the fractional numerator, leaving the loop and post dividers
 * alone, so the PLL is never bypassed or re-locked and the SAI bit clock
 * stays running. Do not use CLOCK_InitAudioPll() for this: it powers the
 * PLL down and waits for re-lock, dropping the bit clock for hundreds of
 * microseconds and forcing attached codecs to re-acquire.
 *
 * The offset is absolute with respect to the devicetree configuration, not
 * cumulative, so a servo passes its current total correction each time.
 *
 * Resolution is one numerator step, which at the devicetree denominator is
 * reported by reading the offset back: the applied value is the nearest
 * representable one, so a caller that cares can compare.
 *
 * Safe to call while audio is streaming. Not an ISR API: it takes
 * nxp_imxrt_audio_pll_lock(), and the underlying analog-interface write
 * busy-waits on a completion handshake.
 *
 * @param dev audio PLL device, DEVICE_DT_GET(DT_NODELABEL(audio_pll))
 * @param ppb offset from nominal in parts per billion, positive is faster
 *
 * @retval 0 on success
 * @retval -ERANGE @p ppb exceeds NXP_IMXRT_AUDIO_PLL_MAX_OFFSET_PPB, or
 *                 would drive the numerator outside the fractional range
 * @retval -ENOTSUP on a SoC without this PLL
 */
int nxp_imxrt_audio_pll_set_offset_ppb(const struct device *dev, int32_t ppb);

/**
 * @brief Read back the offset the audio PLL is actually running at.
 *
 * Reads the hardware rather than a cached value, so it reflects the
 * quantisation applied by the last set call, and any change made behind
 * this driver's back.
 *
 * @param dev audio PLL device
 * @param ppb set to the realized offset from nominal, in parts per billion
 *
 * @retval 0 on success
 * @retval -EINVAL @p ppb is NULL
 * @retval -ENOTSUP on a SoC without this PLL
 */
int nxp_imxrt_audio_pll_get_offset_ppb(const struct device *dev, int32_t *ppb);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_NXP_IMXRT_AUDIO_PLL_H_ */
