/*
 * Copyright 2026 Sonance
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_NXP_IMXRT_AUDIO_PLL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_NXP_IMXRT_AUDIO_PLL_H_

/*
 * Packs a SAI instance's clock name (e.g. IMX_CCM_SAI1_CLK), root clock mux
 * selector and root clock post-divider into the single cell the
 * "nxp,imxrt-audio-pll" clock-controller expects, so one phandle specifier
 * fully identifies which SAI's root clock to configure and how.
 */
#define NXP_AUDIO_PLL_SAI_CLOCK(name, mux, div) \
	(((name) & 0xFFFFUL) | (((mux) & 0xFFUL) << 16) | (((div) & 0xFFUL) << 24))

#define NXP_AUDIO_PLL_CLOCK_NAME(id) ((id) & 0xFFFFUL)
#define NXP_AUDIO_PLL_CLOCK_MUX(id)  (((id) >> 16) & 0xFFUL)
#define NXP_AUDIO_PLL_CLOCK_DIV(id)  (((id) >> 24) & 0xFFUL)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_NXP_IMXRT_AUDIO_PLL_H_ */
