/*
 * Copyright 2026 Sonance
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_imxrt_audio_pll

#include <errno.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nxp_imxrt_audio_pll.h>
#include <zephyr/dt-bindings/clock/imx_ccm_rev2.h>
#include <zephyr/dt-bindings/clock/nxp_imxrt_audio_pll.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <fsl_clock.h>
#if defined(CONFIG_SOC_SERIES_IMXRT11XX)
#include <fsl_anatop_ai.h>
#endif

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_mcux_audio_pll);

struct mcux_audio_pll_config {
	clock_audio_pll_config_t pll_config;
	clock_audio_pll_config_t pll_config_44k1;
	bool has_44k1;
};

struct mcux_audio_pll_data {
	/*
	 * Serialises every access to the audio PLL's analog interface, and
	 * the nominal below, across all callers; see
	 * nxp_imxrt_audio_pll_lock(). An access is a sequence -- direction,
	 * address, data, toggle -- through one set of interface registers, so
	 * a caller preempted partway through resumes with whatever address and
	 * direction the preempting caller left: its data lands in the wrong
	 * register, or its read becomes a write. A mutex rather than a
	 * spinlock because a family switch holds it across the PLL re-lock.
	 */
	struct k_mutex lock;
	/*
	 * Nominal for the family currently configured. The trim below is
	 * relative to this, so both move when the family changes.
	 */
	uint32_t nominal_num;
	int64_t mult_scaled;
};

#define AUDIO_PLL_LOOP_DIV DT_INST_PROP(0, loop_divider)
#define AUDIO_PLL_NUM      DT_INST_PROP(0, numerator)
#define AUDIO_PLL_DENOM    DT_INST_PROP(0, denominator)

/*
 * The PLL multiplier is (loop_divider + numerator/denominator). Carrying it
 * pre-multiplied by the denominator keeps the ppb conversions in exact
 * integer arithmetic: a fractional offset p scales the whole multiplier, so
 * the numerator moves by (loop_divider * denominator + numerator) * p.
 *
 * This exceeds 32 bits for any useful denominator, hence the 64-bit type.
 */
#define AUDIO_PLL_MULT_SCALED(loop_div, num) ((int64_t)(loop_div) * AUDIO_PLL_DENOM + (num))

/*
 * The numerator field is signed 30-bit: a value at or above 2^29 is read by
 * the PLL as negative and the output drops well below the intended
 * frequency, while CLOCK_GetAvPllFreq() reads the same register back as
 * unsigned and still reports the intended one. Nothing in software notices,
 * so the ceiling is asserted here and enforced on every trim below.
 *
 * The trim adds to the numerator, so the nominal plus the full pull range
 * has to fit, not just the nominal.
 */
#define AUDIO_PLL_MFN_MAX 0x1FFFFFFFU

#define AUDIO_PLL_TRIM_HEADROOM(loop_div, num)                                                     \
	((int64_t)NXP_IMXRT_AUDIO_PLL_MAX_OFFSET_PPB * AUDIO_PLL_MULT_SCALED(loop_div, num) /      \
	 1000000000LL)

BUILD_ASSERT(AUDIO_PLL_NUM < AUDIO_PLL_DENOM, "audio PLL numerator must be below the denominator");
BUILD_ASSERT(AUDIO_PLL_DENOM <= 0x3FFFFFFF, "audio PLL denominator exceeds the 30-bit field");
BUILD_ASSERT(AUDIO_PLL_NUM + AUDIO_PLL_TRIM_HEADROOM(AUDIO_PLL_LOOP_DIV, AUDIO_PLL_NUM) <=
	     AUDIO_PLL_MFN_MAX,
	     "audio PLL numerator plus trim range reaches the field's sign bit; "
	     "lower the denominator");

/* The 44.1 kHz family is optional; boards running only one family omit it. */
#define AUDIO_PLL_HAS_44K1                                                                         \
	(DT_INST_NODE_HAS_PROP(0, loop_divider_44k1) && DT_INST_NODE_HAS_PROP(0, numerator_44k1))
#define AUDIO_PLL_LOOP_DIV_44K1 DT_INST_PROP_OR(0, loop_divider_44k1, 0)
#define AUDIO_PLL_NUM_44K1      DT_INST_PROP_OR(0, numerator_44k1, 0)

#if AUDIO_PLL_HAS_44K1
BUILD_ASSERT(AUDIO_PLL_NUM_44K1 < AUDIO_PLL_DENOM,
	     "44.1 kHz numerator must be below the denominator");
BUILD_ASSERT(AUDIO_PLL_NUM_44K1 +
	     AUDIO_PLL_TRIM_HEADROOM(AUDIO_PLL_LOOP_DIV_44K1, AUDIO_PLL_NUM_44K1) <=
	     AUDIO_PLL_MFN_MAX,
	     "44.1 kHz numerator plus trim range reaches the field's sign bit");
#endif

static int mcux_audio_pll_on(const struct device *dev, clock_control_subsys_t sub_system)
{
	uint32_t id = (uint32_t)(uintptr_t)sub_system;
	uint32_t clock_name = NXP_AUDIO_PLL_CLOCK_NAME(id);
	uint32_t mux = NXP_AUDIO_PLL_CLOCK_MUX(id);
	uint32_t div = NXP_AUDIO_PLL_CLOCK_DIV(id);

	ARG_UNUSED(dev);

#if defined(CONFIG_SOC_SERIES_IMXRT11XX)
	switch (clock_name) {
	case IMX_CCM_SAI1_CLK:
		CLOCK_SetRootClockMux(kCLOCK_Root_Sai1, mux);
		CLOCK_SetRootClockDiv(kCLOCK_Root_Sai1, div);
		break;
	case IMX_CCM_SAI2_CLK:
		CLOCK_SetRootClockMux(kCLOCK_Root_Sai2, mux);
		CLOCK_SetRootClockDiv(kCLOCK_Root_Sai2, div);
		break;
	case IMX_CCM_SAI3_CLK:
		CLOCK_SetRootClockMux(kCLOCK_Root_Sai3, mux);
		CLOCK_SetRootClockDiv(kCLOCK_Root_Sai3, div);
		break;
	case IMX_CCM_SAI4_CLK:
		CLOCK_SetRootClockMux(kCLOCK_Root_Sai4, mux);
		CLOCK_SetRootClockDiv(kCLOCK_Root_Sai4, div);
		break;
	default:
		LOG_ERR("unknown SAI clock name 0x%x", clock_name);
		return -EINVAL;
	}

	return 0;
#else
#error Initialize SOC Series-specific SAI root clock mux/div for this clock name
#endif /* CONFIG_SOC_SERIES */
}

void nxp_imxrt_audio_pll_lock(const struct device *dev)
{
	struct mcux_audio_pll_data *data = dev->data;

	/*
	 * Before the scheduler starts there is a single context, so nothing
	 * to exclude, and a mutex cannot be taken yet. An ISR cannot take it
	 * at all; no access here is an ISR API.
	 */
	if (k_is_pre_kernel() || k_is_in_isr()) {
		return;
	}
	(void)k_mutex_lock(&data->lock, K_FOREVER);
}

void nxp_imxrt_audio_pll_unlock(const struct device *dev)
{
	struct mcux_audio_pll_data *data = dev->data;

	if (k_is_pre_kernel() || k_is_in_isr()) {
		return;
	}
	(void)k_mutex_unlock(&data->lock);
}

static int mcux_audio_pll_off(const struct device *dev, clock_control_subsys_t sub_system)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sub_system);

	return 0;
}

/*
 * Adopt @pll as the running configuration and rebase the trim on it. The
 * caller holds the lock, so no trim sees the new PLL with the old nominal.
 */
static void mcux_audio_pll_apply(const struct device *dev, const clock_audio_pll_config_t *pll)
{
	struct mcux_audio_pll_data *data = dev->data;

	CLOCK_InitAudioPll(pll);

	data->nominal_num = pll->numerator;
	data->mult_scaled = AUDIO_PLL_MULT_SCALED(pll->loopDivider, pll->numerator);
}

int nxp_imxrt_audio_pll_set_family(const struct device *dev, enum nxp_imxrt_audio_pll_family family)
{
#if defined(CONFIG_SOC_SERIES_IMXRT11XX)
	const struct mcux_audio_pll_config *cfg = dev->config;
	const clock_audio_pll_config_t *pll;

	switch (family) {
	case NXP_IMXRT_AUDIO_PLL_FAMILY_48K:
		pll = &cfg->pll_config;
		break;
	case NXP_IMXRT_AUDIO_PLL_FAMILY_44K1:
		if (!cfg->has_44k1) {
			LOG_ERR("no 44.1 kHz family configuration in devicetree");
			return -ENOSYS;
		}
		pll = &cfg->pll_config_44k1;
		break;
	default:
		return -EINVAL;
	}

	nxp_imxrt_audio_pll_lock(dev);
	mcux_audio_pll_apply(dev, pll);
	nxp_imxrt_audio_pll_unlock(dev);

	return 0;
#else
	ARG_UNUSED(dev);
	ARG_UNUSED(family);
	return -ENOTSUP;
#endif
}

int nxp_imxrt_audio_pll_set_offset_ppb(const struct device *dev, int32_t ppb)
{
#if defined(CONFIG_SOC_SERIES_IMXRT11XX)
	struct mcux_audio_pll_data *data = dev->data;
	int64_t num;
	int ret = 0;

	if (ppb > NXP_IMXRT_AUDIO_PLL_MAX_OFFSET_PPB || ppb < -NXP_IMXRT_AUDIO_PLL_MAX_OFFSET_PPB) {
		return -ERANGE;
	}

	/* The nominal is read under the lock too: a family switch moves it. */
	nxp_imxrt_audio_pll_lock(dev);

	num = data->nominal_num + DIV_ROUND_CLOSEST((int64_t)ppb * data->mult_scaled, 1000000000LL);

	/*
	 * Above 2^29 the PLL reads the numerator as negative; see the note
	 * on AUDIO_PLL_MFN_MAX.
	 */
	if (num < 1 || num >= AUDIO_PLL_DENOM || num > AUDIO_PLL_MFN_MAX) {
		ret = -ERANGE;
	} else {
		/*
		 * One analog-interface transaction carries the whole 30-bit
		 * value, so the fractional divider never observes a partly
		 * updated numerator. Changing it on a running PLL is what the
		 * spread spectrum hardware does continuously, so the loop stays
		 * locked through the change.
		 */
		ANATOP_AI_Write(kAI_Itf_Audio, kAI_PLLAUDIO_CTRL2, (uint32_t)num);
	}

	nxp_imxrt_audio_pll_unlock(dev);

	return ret;
#else
	ARG_UNUSED(dev);
	ARG_UNUSED(ppb);
	return -ENOTSUP;
#endif
}

int nxp_imxrt_audio_pll_get_offset_ppb(const struct device *dev, int32_t *ppb)
{
#if defined(CONFIG_SOC_SERIES_IMXRT11XX)
	struct mcux_audio_pll_data *data = dev->data;
	int64_t num;

	if (ppb == NULL) {
		return -EINVAL;
	}

	nxp_imxrt_audio_pll_lock(dev);
	num = ANATOP_AI_Read(kAI_Itf_Audio, kAI_PLLAUDIO_CTRL2) & 0x3FFFFFFF;
	*ppb = (int32_t)DIV_ROUND_CLOSEST((num - data->nominal_num) * 1000000000LL,
					  data->mult_scaled);
	nxp_imxrt_audio_pll_unlock(dev);

	return 0;
#else
	ARG_UNUSED(dev);
	ARG_UNUSED(ppb);
	return -ENOTSUP;
#endif
}

static int mcux_audio_pll_init(const struct device *dev)
{
	const struct mcux_audio_pll_config *cfg = dev->config;
	struct mcux_audio_pll_data *data = dev->data;

	k_mutex_init(&data->lock);

	/*
	 * Shared by every SAI instance; configured exactly once here rather
	 * than redundantly by each SAI instance that ends up referencing
	 * this device. Boots on the 48 kHz family; a board needing 44.1 kHz
	 * switches at runtime.
	 */
	mcux_audio_pll_apply(dev, &cfg->pll_config);

	return 0;
}

static DEVICE_API(clock_control, mcux_audio_pll_driver_api) = {
	.on = mcux_audio_pll_on,
	.off = mcux_audio_pll_off,
};

static const struct mcux_audio_pll_config mcux_audio_pll_config_0 = {
	.pll_config = {
		.loopDivider = DT_INST_PROP(0, loop_divider),
		.postDivider = DT_INST_PROP(0, post_divider),
		.numerator = DT_INST_PROP(0, numerator),
		.denominator = DT_INST_PROP(0, denominator),
		.ssEnable = false,
	},
	.pll_config_44k1 = {
		.loopDivider = AUDIO_PLL_LOOP_DIV_44K1,
		.postDivider = DT_INST_PROP(0, post_divider),
		.numerator = AUDIO_PLL_NUM_44K1,
		.denominator = DT_INST_PROP(0, denominator),
		.ssEnable = false,
	},
	.has_44k1 = AUDIO_PLL_HAS_44K1,
};

static struct mcux_audio_pll_data mcux_audio_pll_data_0;

DEVICE_DT_INST_DEFINE(0, mcux_audio_pll_init, NULL, &mcux_audio_pll_data_0,
		      &mcux_audio_pll_config_0, PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &mcux_audio_pll_driver_api);
