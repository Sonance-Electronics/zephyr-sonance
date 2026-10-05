/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/dai.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>

/* must precede sai.h, whose inline helpers log */
LOG_MODULE_REGISTER(nxp_dai_sai, CONFIG_DAI_LOG_LEVEL);

#include "sai.h"

/* used for binding the driver */
#define DT_DRV_COMPAT nxp_dai_sai

/*
 * How long to wait for the transmitter/receiver to actually go down; see
 * sai_tx_rx_wait_disabled().
 *
 * The hardware only clears its enable bit at the end of the frame it is
 * in, so the wait has to cover a whole frame period. This was a flat
 * 50 us, which is already less than three frames at 44.1 kHz (22.7 us
 * each) and a fraction of one at the lower rates the hardware supports --
 * 125 us at 8 kHz -- so a stop could time out simply because the frame
 * had not finished.
 *
 * Allow four frames at the configured rate, with a floor for the fast
 * rates and for the case where no rate has been configured yet.
 */
#define SAI_TX_RX_HW_DISABLE_TIMEOUT_US(rate)\
	((rate) != 0U ? CLAMP((4U * 1000000U) / (rate), 100U, 20000U) : 20000U)

/* TODO list:
 *
 * 1) No busy waiting should be performed in any of the operations.
 * STOP, PAUSE and POST_STOP do not wait while the SAI provides the bit
 * clock, but START and config_set() do when they follow a STOP within the
 * frame it was issued in: see sai_tx_rx_wait_disabled(). Removing that
 * wait would mean returning -EBUSY instead, which every caller would have
 * to retry on. With an external bit clock STOP and PAUSE still wait; see
 * sai_tx_rx_disable(). (SOF)
 */

#ifdef CONFIG_SAI_HAS_MCLK_CONFIG_OPTION
/* note: i.MX8 boards don't seem to support the MICS field in the MCR
 * register. As such, the MCLK source field of sai_master_clock_t is
 * useless. I'm assuming the source is selected through xCR2's MSEL.
 *
 * TODO: for now, this function will set MCR's MSEL to the same value
 * as xCR2's MSEL or, rather, to the same MCLK as the one used for
 * generating BCLK. Is there a need to support different MCLKs in
 * xCR2 and MCR?
 */
static int sai_mclk_config(const struct device *dev,
			   sai_bclk_source_t bclk_source,
			   const struct sai_bespoke_config *bespoke)
{
	const struct sai_config *cfg;
	struct sai_data *data;
	sai_master_clock_t mclk_config;
	uint32_t msel, mclk_rate;
	int ret;

	cfg = dev->config;
	data = dev->data;

	mclk_config.mclkOutputEnable = cfg->mclk_is_output;

	ret = get_msel(bclk_source, &msel);
	if (ret < 0) {
		LOG_ERR("invalid MCLK source %d for MSEL", bclk_source);
		return ret;
	}

	/* get MCLK's rate */
	ret = get_mclk_rate(&cfg->clk_data, bclk_source, &mclk_rate);
	if (ret < 0) {
		LOG_ERR("failed to query MCLK's rate");
		return ret;
	}

	LOG_DBG("source MCLK is %u", mclk_rate);

	LOG_DBG("target MCLK is %u", bespoke->mclk_rate);

	/* source MCLK rate */
	mclk_config.mclkSourceClkHz = mclk_rate;

	/* target MCLK rate */
	mclk_config.mclkHz = bespoke->mclk_rate;

	/* commit configuration */
	SAI_SetMasterClockConfig(UINT_TO_I2S(data->regmap), &mclk_config);

	set_msel(data->regmap, msel);

	return 0;
}
#endif /* CONFIG_SAI_HAS_MCLK_CONFIG_OPTION */

/*
 * Minimum gap between FIFO error reports, per direction.
 *
 * The error flag can be raised on every frame, so one stall becomes a
 * flood: an RX ring left undrained for 9 ms produced 237 of these, which
 * is 1.25 s of output on a 115200 console -- the log becomes a far bigger
 * problem than the event it describes, and on a shared console it looks
 * like the system has hung. 100 ms collapses a burst into a single line
 * while still separating events a tenth of a second apart.
 *
 * The limit is kept per call site, so TX and RX are limited independently
 * but every SAI instance shares each one. The count of what was held back
 * prints as "Skipped N messages" just before the next report, so a burst
 * that ends mid-interval goes unreported until the next error.
 *
 * With CONFIG_LOG_RATELIMIT=n these warnings follow
 * CONFIG_LOG_RATELIMIT_FALLBACK, whose default drops them entirely.
 */
#define SAI_FIFO_ERROR_REPORT_MS 100U

/*
 * Handle a FIFO error on one direction. Returns true if the direction was
 * halted.
 *
 * A FIFO error loses words from the middle of a frame. FCONT is left at 0,
 * so once the error flag is cleared the SAI resumes at the start of the next
 * frame, but the partial frame already handed to DMA stays in the stream and
 * every word after it lands in the wrong slot: the channels are permanently
 * swapped. Only the consumer, which owns the DMA buffers, can resynchronise,
 * by stopping and restarting.
 *
 * With CONFIG_DAI_NXP_SAI_STOP_ON_FIFO_ERROR, halt the direction so nothing
 * misaligned follows: stop its DMA requests, leave the error flag set, which
 * with FCONT at 0 holds the FIFO from the next frame on, and mask the
 * interrupt so the held flag does not re-raise it. Disabling the
 * transmitter/receiver itself would mean waiting for the frame to end, which
 * cannot be done here, and in synchronous mode can take the other direction
 * down with it. The direction then sits in DAI_STATE_ERROR until the
 * consumer re-arms its DMA and calls dai_nxp_sai_recover(), which keeps it
 * enabled throughout, or until DAI_TRIGGER_STOP, after which
 * DAI_TRIGGER_START's software reset makes the first word slot 0 again.
 *
 * Only a RUNNING direction is halted. One already stopping, or paused, is
 * on its way down anyway and just has its flag cleared, as before.
 */
static bool sai_fifo_error(const struct device *dev, enum dai_dir dir)
{
	struct sai_data *data = dev->data;
	bool halted = false;

	if (dir == DAI_DIR_RX) {
		data->rx_fifo_errors++;
	} else {
		data->tx_fifo_errors++;
	}

	if (IS_ENABLED(CONFIG_DAI_NXP_SAI_STOP_ON_FIFO_ERROR) &&
	    sai_update_state(dir, data, DAI_STATE_ERROR) == 0) {
		sai_tx_rx_dma_enable(data, dir, false);
		sai_tx_rx_irq_enable(data, dir, kSAI_FIFOErrorInterruptEnable, false);
		halted = true;
	} else {
		sai_tx_rx_status_clear(data, dir, kSAI_FIFOErrorFlag);
	}

	if (data->error_cb != NULL) {
		data->error_cb(dev, dir, halted, data->error_cb_data);
	}

	return halted;
}

void sai_isr(const void *parameter)
{
	const struct device *dev;
	struct sai_data *data;
	bool halted;

	dev = parameter;
	data = dev->data;

	/* check for TX FIFO error */
	if (SAI_TX_RX_STATUS_IS_SET(DAI_DIR_TX, data->regmap, kSAI_FIFOErrorFlag)) {
		halted = sai_fifo_error(dev, DAI_DIR_TX);
		LOG_WRN_RATELIMIT_RATE(SAI_FIFO_ERROR_REPORT_MS, "FIFO underrun detected%s",
				       halted ? ", TX halted until restarted" : "");
	}

	/* check for RX FIFO error */
	if (SAI_TX_RX_STATUS_IS_SET(DAI_DIR_RX, data->regmap, kSAI_FIFOErrorFlag)) {
		halted = sai_fifo_error(dev, DAI_DIR_RX);
		LOG_WRN_RATELIMIT_RATE(SAI_FIFO_ERROR_REPORT_MS, "FIFO overrun detected%s",
				       halted ? ", RX halted until restarted" : "");
	}
}

static int sai_config_get(const struct device *dev,
			  struct dai_config *cfg,
			  enum dai_dir dir)
{
	struct sai_data *data = dev->data;

	/* dump content of the DAI configuration */
	memcpy(cfg, &data->cfg, sizeof(*cfg));

	return 0;
}

static const struct dai_properties
	*sai_get_properties(const struct device *dev, enum dai_dir dir, int stream_id)
{
	const struct sai_config *cfg = dev->config;

	switch (dir) {
	case DAI_DIR_RX:
		return cfg->rx_props;
	case DAI_DIR_TX:
		return cfg->tx_props;
	default:
		LOG_ERR("invalid direction: %d", dir);
		return NULL;
	}

	CODE_UNREACHABLE;
}

#ifdef CONFIG_SAI_IMX93_ERRATA_051421
/* notes:
 *	1) TX and RX operate in the same mode: master/slave. As such,
 *	there's no need to check the mode for both directions.
 *
 *	2) Only one of the directions can operate in SYNC mode at a
 *	time.
 *
 *	3) What this piece of code does is it makes the SYNC direction
 *	use the ASYNC direction's BCLK that comes from its input pad.
 *	Logically speaking, this would look like:
 *
 *                      +--------+     +--------+
 *                      |   TX   |     |   RX   |
 *                      | module |     | module |
 *                      +--------+     +--------+
 *                         |   ^            |
 *                         |   |            |
 *                 TX_BCLK |   |____________| RX_BCLK
 *                         |                |
 *                         V                V
 *                     +---------+    +---------+
 *                     | TX BCLK |    | RX BCLK |
 *                     |   pad   |    |   pad   |
 *                     +---------+    +---------+
 *                          |              |
 *                          | TX_BCLK      | RX_BCLK
 *                          V              V
 *
 *	Without BCI enabled, the TX module would use an RX_BCLK
 *	that's divided instead of the one that's obtained from
 *	bypassing the MCLK (i.e: TX_BCLK would have the value of
 *	MCLK / ((RX_DIV + 1) * 2)). If BCI is 1, then TX_BCLK will
 *	be the same as the RX_BCLK that's obtained from bypassing
 *	the MCLK on RX's side.
 *
 *	4) The check for BCLK == MCLK is there to see if the ASYNC
 *	direction will have the BYP bit toggled.
 *
 *	IMPORTANT1: in the above diagram and information, RX is SYNC
 *	with TX. The same applies if RX is SYNC with TX. Also, this
 *	applies to i.MX93. For other SoCs, things may be different
 *	so use this information with caution.
 *
 *	IMPORTANT2: for this to work, you also need to enable the
 *	pad's input path. For i.MX93, this can be achieved by setting
 *	the pad's SION bit.
 */
static void sai_config_set_err_051421(I2S_Type *base,
				      const struct sai_config *cfg,
				      const struct sai_bespoke_config *bespoke,
				      sai_transceiver_t *rx_config,
				      sai_transceiver_t *tx_config)
{
	if (tx_config->masterSlave == kSAI_Master &&
	    bespoke->mclk_rate == bespoke->bclk_rate) {
		if (cfg->tx_sync_mode == kSAI_ModeSync) {
			base->TCR2 |= I2S_TCR2_BCI(1);
		}

		if (cfg->rx_sync_mode == kSAI_ModeSync) {
			base->RCR2 |= I2S_RCR2_BCI(1);
		}
	}
}
#endif /* CONFIG_SAI_IMX93_ERRATA_051421 */

/* The directions an operation on @dir acts on, as a mask of BIT(dir).
 *
 *	1) The "rx_sync_mode" and "tx_sync_mode" properties force the user to pick from
 *	SYNC and ASYNC for each direction. As such, there are 4 possible combinations
 *	that need to be covered here:
 *		a) TX ASYNC, RX ASYNC
 *		b) TX SYNC, RX ASYNC
 *		c) TX ASYNC, RX SYNC
 *		d) TX SYNC, RX SYNC
 *
 *	Combination d) is not valid and is covered by a BUILD_ASSERT(). As such, there are 3 valid
 *	combinations that need to be supported. In combination a) an operation only acts on the
 *	target direction, so there's only combinations b) and c) to be covered.
 *
 *	2) We can distinguish between 3 types of directions:
 *		a) The target direction. This is the direction on which we want to perform the
 *		operation: a disable, a wait for one, or a software reset.
 *		b) The SYNC direction. This is, well, the direction that's in SYNC with the other
 *		direction.
 *		c) The ASYNC direction.
 *
 *	Of course, the target direction may differ from the SYNC or ASYNC directions, but it
 *	can't differ from both of them at the same time (i.e: TARGET != SYNC AND TARGET != ASYNC).
 *
 *	If the target direction is the same as the SYNC direction then we can safely act on the
 *	target direction as there's nothing depending on it. We also want to act on the ASYNC
 *	direction. We can only do this if the ASYNC direction wasn't software enabled (i.e:
 *	through an explicit trigger_start() call).
 *
 *	If the target direction is the same as the ASYNC direction then we can only act on it
 *	if the SYNC direction wasn't software enabled (i.e: through an explicit trigger_start()
 *	call).
 */
static uint32_t sai_tx_rx_dirs(struct sai_data *data, const struct sai_config *cfg,
			       enum dai_dir dir)
{
	enum dai_dir sync_dir, async_dir;

	if (cfg->tx_sync_mode == kSAI_ModeAsync && cfg->rx_sync_mode == kSAI_ModeAsync) {
		return BIT(dir);
	}

	sync_dir = SAI_TX_RX_GET_SYNC_DIR(cfg);
	async_dir = SAI_TX_RX_GET_ASYNC_DIR(cfg);

	if (dir == sync_dir) {
		return BIT(sync_dir) |
		       (SAI_TX_RX_DIR_IS_SW_ENABLED(async_dir, data) ? 0U : BIT(async_dir));
	}

	return SAI_TX_RX_DIR_IS_SW_ENABLED(sync_dir, data) ? 0U : BIT(async_dir);
}

/* Wait for disables requested by sai_tx_rx_disable() to take effect, on
 * the directions in @dirs.
 *
 * The hardware only clears TE/RE at the end of the current frame. This
 * costs nothing when a direction is already down, which it is unless the
 * caller follows a STOP within the frame it was issued in; it then waits
 * out the rest of that frame.
 *
 * The timeout starts here rather than at the disable request, so a frame
 * that could not finish because its clock was stopped in between, e.g.
 * while the clock source was being reprogrammed or runtime PM had gated
 * the SAI, is still given a full timeout once the clock runs again.
 *
 * START and config_set() carry on after a timeout, with a warning, as they
 * did before they waited at all: a frame that never ends means its bit
 * clock has stopped, typically an external one whose provider only starts
 * it again once the stream does, so failing would leave the DAI stuck.
 *
 * please note the difference between the transmitter/receiver's
 * hardware states and their software states. The software
 * states can be obtained by reading data->tx/rx_enabled, while
 * the hardware states can be obtained by reading TCSR/RCSR. The
 * hardware state can actually differ from the software state.
 * Here, we're interested in reading the hardware state which
 * indicates if the transmitter/receiver was actually disabled
 * or not.
 */
static int sai_tx_rx_wait_disabled(struct sai_data *data, uint32_t dirs)
{
	for (enum dai_dir dir = DAI_DIR_TX; dir <= DAI_DIR_RX; dir++) {
		if ((dirs & BIT(dir)) == 0U) {
			continue;
		}

		if (!WAIT_FOR(!SAI_TX_RX_IS_HW_ENABLED(dir, data->regmap),
			      SAI_TX_RX_HW_DISABLE_TIMEOUT_US(data->cfg.rate), k_busy_wait(1))) {
			LOG_WRN("dir %d still enabled after %u us", dir,
				SAI_TX_RX_HW_DISABLE_TIMEOUT_US(data->cfg.rate));
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int sai_pm_get(const struct device *dev)
{
	int ret;

	ret = pm_device_runtime_get(dev);
	if (ret < 0) {
		LOG_ERR("failed to get() SAI device: %d", ret);
	}

	return ret;
}

static int sai_config_set(const struct device *dev,
			  const struct dai_config *cfg,
			  const void *bespoke_data, size_t size)
{
	const struct sai_bespoke_config *bespoke;
	sai_transceiver_t *rx_config, *tx_config;
	struct sai_data *data;
	const struct sai_config *sai_cfg;
	int ret;

	if (cfg->type != DAI_IMX_SAI) {
		LOG_ERR("wrong DAI type: %d", cfg->type);
		return -EINVAL;
	}

	/*
	 * struct sai_bespoke_config has to match SOF's
	 * sof_ipc_dai_sai_params exactly, so a caller built against a
	 * different version of it would be silently misread field by field.
	 * Reject anything too short rather than reading past the end of it.
	 */
	if (size < sizeof(struct sai_bespoke_config)) {
		LOG_ERR("bespoke config too small: %zu, need %zu",
			size, sizeof(struct sai_bespoke_config));
		return -EINVAL;
	}

	bespoke = bespoke_data;
	data = dev->data;
	sai_cfg = dev->config;
	rx_config = &data->rx_config;
	tx_config = &data->tx_config;

	/* since this function configures the transmitter AND the receiver, that
	 * means both of them need to be stopped. As such, doing the state
	 * transition here will also result in a state check.
	 */
	ret = sai_update_state(DAI_DIR_TX, data, DAI_STATE_READY);
	if (ret < 0) {
		LOG_ERR("failed to update TX state. Reason: %d", ret);
		return ret;
	}

	ret = sai_update_state(DAI_DIR_RX, data, DAI_STATE_READY);
	if (ret < 0) {
		LOG_ERR("failed to update RX state. Reason: %d", ret);
		return ret;
	}

	/* condition: BCLK = FSYNC * TDM_SLOT_WIDTH * TDM_SLOTS */
	if (bespoke->bclk_rate !=
	    (bespoke->fsync_rate * bespoke->tdm_slot_width * bespoke->tdm_slots)) {
		LOG_ERR("bad BCLK value: %d", bespoke->bclk_rate);
		return -EINVAL;
	}

	/* TODO: this should be removed if we're to support sw channels != hw channels */
	if (count_leading_zeros(~bespoke->tx_slots) != bespoke->tdm_slots ||
	    count_leading_zeros(~bespoke->rx_slots) != bespoke->tdm_slots) {
		LOG_ERR("number of TX/RX slots doesn't match number of TDM slots");
		return -EINVAL;
	}

	/* get default configurations */
	get_bclk_default_config(&tx_config->bitClock);
	get_fsync_default_config(&tx_config->frameSync);
	get_serial_default_config(&tx_config->serialData);
	get_fifo_default_config(&tx_config->fifo);

	/* note1: this may be obvious but enabling multiple SAI
	 * channels (or data lines) may lead to FIFO starvation/
	 * overflow if data is not written/read from the respective
	 * TDR/RDR registers.
	 *
	 * note2: the SAI data line should be enabled based on
	 * the direction (TX/RX) we're enabling. Enabling the
	 * data line for the opposite direction will lead to FIFO
	 * overrun/underrun when working with a SYNC direction.
	 *
	 * note3: the TX/RX data line shall be enabled/disabled
	 * via the sai_trigger_() suite to avoid scenarios in
	 * which one configures both direction but only starts
	 * the SYNC direction which would lead to a FIFO underrun.
	 */
	tx_config->channelMask = 0x0;

	/* TODO: for now, only MCLK1 is supported */
	tx_config->bitClock.bclkSource = kSAI_BclkSourceMclkOption1;

	/* FSYNC is asserted for tdm_slot_width BCLKs */
	tx_config->frameSync.frameSyncWidth = bespoke->tdm_slot_width;

	/* serial data common configuration */
	tx_config->serialData.dataWord0Length = bespoke->tdm_slot_width;
	tx_config->serialData.dataWordNLength = bespoke->tdm_slot_width;
	tx_config->serialData.dataFirstBitShifted = bespoke->tdm_slot_width;
	tx_config->serialData.dataWordNum = bespoke->tdm_slots;

	/* clock provider configuration */
	switch (cfg->format & DAI_FORMAT_CLOCK_PROVIDER_MASK) {
	case DAI_CBP_CFP:
		tx_config->masterSlave = kSAI_Slave;
		break;
	case DAI_CBC_CFC:
		tx_config->masterSlave = kSAI_Master;
		break;
	case DAI_CBC_CFP:
	case DAI_CBP_CFC:
		LOG_ERR("unsupported provider configuration: %d",
			cfg->format & DAI_FORMAT_CLOCK_PROVIDER_MASK);
		return -ENOTSUP;
	default:
		LOG_ERR("invalid provider configuration: %d",
			cfg->format & DAI_FORMAT_CLOCK_PROVIDER_MASK);
		return -EINVAL;
	}

	LOG_DBG("SAI is in %d mode", tx_config->masterSlave);

	/* protocol configuration */
	switch (cfg->format & DAI_FORMAT_PROTOCOL_MASK) {
	case DAI_PROTO_I2S:
		/* BCLK is active LOW */
		tx_config->bitClock.bclkPolarity = kSAI_PolarityActiveLow;
		/* FSYNC is active LOW */
		tx_config->frameSync.frameSyncPolarity = kSAI_PolarityActiveLow;
		break;
	case DAI_PROTO_DSP_A:
		/* FSYNC is asserted for a single BCLK */
		tx_config->frameSync.frameSyncWidth = 1;
		/* BCLK is active LOW */
		tx_config->bitClock.bclkPolarity = kSAI_PolarityActiveLow;
		break;
	default:
		LOG_ERR("unsupported DAI protocol: %d",
			cfg->format & DAI_FORMAT_PROTOCOL_MASK);
		return -EINVAL;
	}

	LOG_DBG("SAI uses protocol: %d",
		cfg->format & DAI_FORMAT_PROTOCOL_MASK);

	/* clock inversion configuration */
	switch (cfg->format & DAI_FORMAT_CLOCK_INVERSION_MASK) {
	case DAI_INVERSION_IB_IF:
		SAI_INVERT_POLARITY(tx_config->bitClock.bclkPolarity);
		SAI_INVERT_POLARITY(tx_config->frameSync.frameSyncPolarity);
		break;
	case DAI_INVERSION_IB_NF:
		SAI_INVERT_POLARITY(tx_config->bitClock.bclkPolarity);
		break;
	case DAI_INVERSION_NB_IF:
		SAI_INVERT_POLARITY(tx_config->frameSync.frameSyncPolarity);
		break;
	case DAI_INVERSION_NB_NF:
		/* nothing to do here */
		break;
	default:
		LOG_ERR("invalid clock inversion configuration: %d",
			cfg->format & DAI_FORMAT_CLOCK_INVERSION_MASK);
		return -EINVAL;
	}

	LOG_DBG("FSYNC polarity: %d", tx_config->frameSync.frameSyncPolarity);
	LOG_DBG("BCLK polarity: %d", tx_config->bitClock.bclkPolarity);

	/* duplicate TX configuration */
	memcpy(rx_config, tx_config, sizeof(sai_transceiver_t));

	tx_config->serialData.dataMaskedWord = ~bespoke->tx_slots;
	rx_config->serialData.dataMaskedWord = ~bespoke->rx_slots;

	tx_config->fifo.fifoWatermark = sai_cfg->tx_fifo_watermark - 1;
	rx_config->fifo.fifoWatermark = sai_cfg->rx_fifo_watermark - 1;

	LOG_DBG("RX watermark: %d", sai_cfg->rx_fifo_watermark);
	LOG_DBG("TX watermark: %d", sai_cfg->tx_fifo_watermark);

	/* set the synchronization mode based on data passed from the DTS */
	tx_config->syncMode = sai_cfg->tx_sync_mode;
	rx_config->syncMode = sai_cfg->rx_sync_mode;

	ret = sai_pm_get(dev);
	if (ret < 0) {
		return ret;
	}

	/* the configuration cannot be changed while either direction is
	 * still enabled, and a STOP issued in the current frame may not have
	 * taken effect yet. This runs before data->cfg.rate changes, as the
	 * frame still finishing is at the old rate.
	 */
	if (sai_tx_rx_wait_disabled(data, BIT(DAI_DIR_TX) | BIT(DAI_DIR_RX)) < 0) {
		LOG_WRN("configuring anyway");
	}

	/* commit configuration */
	SAI_RxSetConfig(UINT_TO_I2S(data->regmap), rx_config);
	SAI_TxSetConfig(UINT_TO_I2S(data->regmap), tx_config);

	/* a few notes here:
	 *	1) TX and RX operate in the same mode: master or slave.
	 *	2) Setting BCLK's rate needs to be performed explicitly
	 *	since SetConfig() doesn't do it for us.
	 *	3) Setting BCLK's rate has to be performed after the
	 *	SetConfig() call as that resets the SAI registers.
	 */
	if (tx_config->masterSlave == kSAI_Master) {
		SAI_TxSetBitClockRate(UINT_TO_I2S(data->regmap), bespoke->mclk_rate,
				      bespoke->fsync_rate, bespoke->tdm_slot_width,
				      bespoke->tdm_slots);

		SAI_RxSetBitClockRate(UINT_TO_I2S(data->regmap), bespoke->mclk_rate,
				      bespoke->fsync_rate, bespoke->tdm_slot_width,
				      bespoke->tdm_slots);
	}

#ifdef CONFIG_SAI_HAS_MCLK_CONFIG_OPTION
	ret = sai_mclk_config(dev, tx_config->bitClock.bclkSource, bespoke);
	if (ret < 0) {
		LOG_ERR("failed to set MCLK configuration");
		pm_device_runtime_put(dev);
		return ret;
	}
#endif /* CONFIG_SAI_HAS_MCLK_CONFIG_OPTION */

#ifdef CONFIG_SAI_IMX93_ERRATA_051421
	sai_config_set_err_051421(UINT_TO_I2S(data->regmap),
				  sai_cfg, bespoke,
				  rx_config, tx_config);
#endif /* CONFIG_SAI_IMX93_ERRATA_051421 */

	/* this is needed so that rates different from FSYNC_RATE
	 * will not be allowed.
	 *
	 * this is because the hardware is configured to match
	 * the topology rates so attempting to play a file using
	 * a different rate from the one configured in the hardware
	 * doesn't work properly.
	 *
	 * if != 0, SOF will raise an error if the PCM rate is
	 * different than the hardware rate (a.k.a this one).
	 */
	data->cfg.rate = bespoke->fsync_rate;
	/* SOF note: we don't support a variable number of channels
	 * at the moment so leaving the number of channels as 0 is
	 * unnecessary and leads to issues (e.g: the mixer buffers
	 * use this value to set the number of channels so having
	 * a 0 as this value leads to mixer buffers having 0 channels,
	 * which, in turn, leads to the DAI ending up with 0 channels,
	 * thus resulting in an error)
	 */
	data->cfg.channels = bespoke->tdm_slots;

	sai_dump_register_data(data->regmap);

	return pm_device_runtime_put(dev);
}

/* True if the bit clock that @dir runs from comes from outside the SAI.
 *
 * This reads the bit clock direction (xCR2.BCD) the hardware is actually
 * configured with, from the direction that provides the clock: in
 * synchronous mode that is the ASYNC direction. The cached transceiver
 * configuration can disagree with the hardware after a config_set() that
 * failed part way through.
 */
static bool sai_tx_rx_bclk_is_external(struct sai_data *data, const struct sai_config *cfg,
				       enum dai_dir dir)
{
	I2S_Type *base = UINT_TO_I2S(data->regmap);

	if (cfg->tx_sync_mode != kSAI_ModeAsync || cfg->rx_sync_mode != kSAI_ModeAsync) {
		dir = SAI_TX_RX_GET_ASYNC_DIR(cfg);
	}

	return dir == DAI_DIR_RX ? (base->RCR2 & I2S_RCR2_BCD_MASK) == 0U
				 : (base->TCR2 & I2S_TCR2_BCD_MASK) == 0U;
}

/* Request that the directions in @dirs be disabled, by clearing @bits
 * (SAI_CSR_XE_MASK, SAI_CSR_BCE_MASK). Without @write only record them, for
 * a SAI suspended by runtime PM whose registers cannot be written;
 * sai_pm_action() writes them on resume.
 */
static void sai_tx_rx_request_disable(struct sai_data *data, uint32_t dirs, uint32_t bits,
				      bool write)
{
	for (enum dai_dir d = DAI_DIR_TX; d <= DAI_DIR_RX; d++) {
		if ((dirs & BIT(d)) == 0U) {
			continue;
		}

		if (write) {
			/* VERY IMPORTANT: DO NOT use SAI_TxEnable/SAI_RxEnable
			 * here as they do not disable the ASYNC direction.
			 * Since the software logic assures that the ASYNC direction
			 * is not disabled before the SYNC direction, we can force
			 * the disablement of the given direction.
			 */
			sai_tx_rx_force_disable(data, d, bits);
		} else {
			*sai_csr_off(data, d) |= bits;
		}
	}
}

/* Request that the directions an operation on @dir acts on be disabled.
 *
 * The hardware only clears TE/RE at the end of the current frame, and this
 * does not wait for it unless the bit clock comes from outside the SAI. So
 * STOP and PAUSE may return with the direction still running: a caller
 * must not take the hardware to be down by then. Anything that needs a
 * direction fully stopped -- a software reset, or a configuration change --
 * calls sai_tx_rx_wait_disabled() first. While a disable is pending,
 * sai_tx_rx_csr_update() keeps other register writes from cancelling it.
 *
 * An external bit clock is the exception. The frame can then only end
 * while the clock provider still runs, and the RM requires an external bit
 * clock to be "disabled after the SAI transmitter or receiver is disabled
 * and completes its current frames". A caller that stops a codec providing
 * the clock straight after STOP would otherwise leave TE/RE set, so with an
 * external bit clock, wait here as STOP and PAUSE always used to.
 *
 * The bit clock itself keeps running; see sai_tx_rx_force_disable() and
 * sai_trigger_post_stop().
 */
static int sai_tx_rx_disable(struct sai_data *data, const struct sai_config *cfg, enum dai_dir dir)
{
	uint32_t dirs = sai_tx_rx_dirs(data, cfg, dir);
	uint32_t external = 0U;

	sai_tx_rx_request_disable(data, dirs, SAI_CSR_XE_MASK, true);

	for (enum dai_dir d = DAI_DIR_TX; d <= DAI_DIR_RX; d++) {
		if ((dirs & BIT(d)) != 0U && sai_tx_rx_bclk_is_external(data, cfg, d)) {
			external |= BIT(d);
		}
	}

	return sai_tx_rx_wait_disabled(data, external);
}

static int sai_trigger_pause(const struct device *dev,
			     enum dai_dir dir)
{
	struct sai_data *data;
	const struct sai_config *cfg;
	int ret;

	data = dev->data;
	cfg = dev->config;

	if (dir != DAI_DIR_RX && dir != DAI_DIR_TX) {
		LOG_ERR("invalid direction: %d", dir);
		return -EINVAL;
	}

	/* attempt to change state */
	ret = sai_update_state(dir, data, DAI_STATE_PAUSED);
	if (ret < 0) {
		LOG_ERR("failed to transition to PAUSED from %d. Reason: %d",
			sai_get_state(dir, data), ret);
		return ret;
	}

	LOG_DBG("pause on direction %d", dir);

	/* with an external bit clock this waits, and a timeout leaves the
	 * direction PAUSED: that still permits both RUNNING and STOPPING
	 */
	ret = sai_tx_rx_disable(data, cfg, dir);
	if (ret < 0) {
		LOG_ERR("timed out disabling dir %d while pausing", dir);
	}

	/* disable TX/RX data line */
	sai_tx_rx_set_dline_mask(dir, data->regmap, 0x0);

	/* update the software state of TX/RX */
	sai_tx_rx_sw_enable_disable(dir, data, false);

	return ret;
}

static int sai_trigger_stop(const struct device *dev,
			    enum dai_dir dir)
{
	struct sai_data *data;
	const struct sai_config *cfg;
	int ret, pm_ret;
	uint32_t old_state;

	data = dev->data;
	cfg = dev->config;
	old_state = sai_get_state(dir, data);

	if (dir != DAI_DIR_RX && dir != DAI_DIR_TX) {
		LOG_ERR("invalid direction: %d", dir);
		return -EINVAL;
	}

	/* attempt to change state */
	ret = sai_update_state(dir, data, DAI_STATE_STOPPING);
	if (ret < 0) {
		LOG_ERR("failed to transition to STOPPING from %d. Reason: %d",
			sai_get_state(dir, data), ret);
		return ret;
	}

	LOG_DBG("stop on direction %d", dir);

	if (old_state != DAI_STATE_PAUSED) {
		/* update the software state of TX/RX */
		sai_tx_rx_sw_enable_disable(dir, data, false);

		/* disable TX/RX data line */
		sai_tx_rx_set_dline_mask(dir, data->regmap, 0x0);
	}

	/* disable DMA requests */
	sai_tx_rx_dma_enable(data, dir, false);

	/* disable error interrupt */
	sai_tx_rx_irq_enable(data, dir, kSAI_FIFOErrorInterruptEnable, false);

	irq_disable(cfg->irq);

	/* With the SAI providing the bit clock this does not wait. The DMA
	 * requests are already off, so whatever the direction receives in the
	 * rest of the frame stays in the FIFO, and START's software reset
	 * empties it: the data before STOP ends part way through a frame,
	 * where it used to end on a whole one. After a PAUSE this requests
	 * the disable again, which changes nothing.
	 *
	 * With an external bit clock it waits, and a timeout forces the
	 * direction to READY. There is no transition out of STOPPING except
	 * a completed stop, so a later STOP would be refused; START and
	 * config_set() carry on regardless.
	 */
	ret = sai_tx_rx_disable(data, cfg, dir);
	if (ret < 0) {
		LOG_ERR("timed out disabling dir %d, forcing it to READY", dir);
		sai_update_state(dir, data, DAI_STATE_READY);
	}

	/* with runtime PM this may gate the SAI's clocks before that frame
	 * has finished; it then finishes once START or config_set()
	 * resumes them, and sai_tx_rx_wait_disabled() allows for that
	 */
	pm_ret = pm_device_runtime_put(dev);

	return ret < 0 ? ret : pm_ret;
}

/* software reset the directions in @dirs, from sai_tx_rx_dirs() */
static void sai_tx_rx_sw_reset(struct sai_data *data, uint32_t dirs)
{
	for (enum dai_dir d = DAI_DIR_TX; d <= DAI_DIR_RX; d++) {
		if ((dirs & BIT(d)) != 0U) {
			sai_tx_rx_dir_sw_reset(data, d);
		}
	}
}

/*
 * Stop the bit clock that STOP leaves running (see
 * sai_tx_rx_force_disable()). The DAI API stops clocks here rather than at
 * STOP so that a downstream codec clocked from BCLK can be shut down first.
 *
 * Only a stopped direction is accepted. In synchronous mode this follows
 * the same rules as the STOP path, with one more: the other direction's
 * bit clock is only stopped if that direction is stopped too. Its software
 * enabled flag is not enough, as PAUSE clears it while a paused direction
 * still needs its clock.
 */
static int sai_trigger_post_stop(const struct device *dev, enum dai_dir dir)
{
	struct sai_data *data;
	const struct sai_config *cfg;
	enum pm_device_state pm_state;
	enum dai_state state;
	uint32_t dirs;
	int ret;

	data = dev->data;
	cfg = dev->config;

	if (dir != DAI_DIR_RX && dir != DAI_DIR_TX) {
		LOG_ERR("invalid direction: %d", dir);
		return -EINVAL;
	}

	/* a STOP leaves the direction in STOPPING, and config_set() in READY */
	state = sai_get_state(dir, data);
	if (state != DAI_STATE_STOPPING && state != DAI_STATE_READY) {
		LOG_ERR("POST_STOP on dir %d needs it stopped, state is %d", dir, state);
		return -EPERM;
	}

	/*
	 * A suspended SAI has its clocks gated, so its bit clock is already
	 * stopped, and its registers cannot be written. Resuming it just to
	 * clear BCE would restart BCLK until the clear took effect at the end
	 * of the frame. Record the bits instead: sai_pm_action() writes them
	 * as soon as anything resumes the SAI, so BCLK runs for at most a
	 * frame.
	 */
	dirs = sai_tx_rx_dirs(data, cfg, dir);
	for (enum dai_dir d = DAI_DIR_TX; d <= DAI_DIR_RX; d++) {
		state = sai_get_state(d, data);
		if (d != dir && (state == DAI_STATE_RUNNING || state == DAI_STATE_PAUSED ||
				 state == DAI_STATE_ERROR)) {
			dirs &= ~BIT(d);
		}
	}

	ret = pm_device_state_get(dev, &pm_state);
	if (ret == 0 && pm_state == PM_DEVICE_STATE_SUSPENDED) {
		sai_tx_rx_request_disable(data, dirs, SAI_CSR_BCE_MASK, false);
		return 0;
	}

	ret = sai_pm_get(dev);
	if (ret < 0) {
		return ret;
	}

	LOG_DBG("post-stop on direction %d", dir);

	/* this may follow STOP within its frame, with TE/RE not yet clear;
	 * BCE then clears at the end of that same frame
	 */
	sai_tx_rx_request_disable(data, dirs, SAI_CSR_BCE_MASK, true);

	return pm_device_runtime_put(dev);
}

static int sai_trigger_start(const struct device *dev,
			     enum dai_dir dir)
{
	struct sai_data *data;
	const struct sai_config *cfg;
	uint32_t old_state, dirs;
	bool reset;
	int ret, i;

	data = dev->data;
	cfg = dev->config;
	old_state = sai_get_state(dir, data);

	/* TX and RX should be triggered independently */
	if (dir != DAI_DIR_RX && dir != DAI_DIR_TX) {
		LOG_ERR("invalid direction: %d", dir);
		return -EINVAL;
	}

	/* a START from STOPPING or READY software resets the direction, which
	 * needs it fully disabled, so wait out a STOP issued in the current
	 * frame; see sai_tx_rx_wait_disabled() for why a timeout carries on.
	 *
	 * A resume from PAUSED does no reset and so does not wait.
	 */
	dirs = sai_tx_rx_dirs(data, cfg, dir);
	reset = old_state == DAI_STATE_STOPPING || old_state == DAI_STATE_READY;
	if (reset) {
		ret = sai_pm_get(dev);
		if (ret < 0) {
			return ret;
		}

		if (sai_tx_rx_wait_disabled(data, dirs) < 0) {
			LOG_WRN("starting dir %d anyway", dir);
		}
	}

	/* attempt to change state */
	ret = sai_update_state(dir, data, DAI_STATE_RUNNING);
	if (ret < 0) {
		LOG_ERR("failed to transition to RUNNING from %d. Reason: %d",
			sai_get_state(dir, data), ret);
		if (reset) {
			pm_device_runtime_put(dev);
		}
		return ret;
	}

	if (old_state == DAI_STATE_PAUSED) {
		/* if the SAI has been paused then there's no
		 * point in issuing a software reset. As such,
		 * skip this part and go directly to the TX/RX
		 * enablement.
		 */
		goto out_enable_dline;
	}

	LOG_DBG("start on direction %d", dir);

	sai_tx_rx_sw_reset(data, dirs);

	irq_enable(cfg->irq);

	/* a FIFO error that halted this direction leaves its flag set (see
	 * sai_fifo_error()); clear it so it neither holds the FIFO nor raises
	 * an interrupt the moment the error interrupt is enabled
	 */
	sai_tx_rx_status_clear(data, dir, kSAI_FIFOErrorFlag);

	/* enable error interrupt */
	sai_tx_rx_irq_enable(data, dir, kSAI_FIFOErrorInterruptEnable, true);

	/* avoid initial underrun by writing a frame's worth of 0s */
	if (dir == DAI_DIR_TX) {
		for (i = 0; i < data->cfg.channels; i++) {
			SAI_WriteData(UINT_TO_I2S(data->regmap), cfg->tx_dline, 0x0);
		}
	}

	/* TODO: for now, only DMA mode is supported */
	sai_tx_rx_dma_enable(data, dir, true);

out_enable_dline:
	/* enable TX/RX data line. This translates to TX_DLINE0/RX_DLINE0
	 * being enabled.
	 *
	 * TODO: for now we only support 1 data line per direction.
	 */
	sai_tx_rx_set_dline_mask(dir, data->regmap,
				 SAI_TX_RX_DLINE_MASK(dir, cfg));

	/* this will also enable the async side, and sets BCE with TE/RE, so
	 * no disable is pending on any direction it enables any more
	 */
	dirs |= BIT(dir);
	for (enum dai_dir d = DAI_DIR_TX; d <= DAI_DIR_RX; d++) {
		if ((dirs & BIT(d)) != 0U) {
			*sai_csr_off(data, d) = 0U;
		}
	}
	SAI_TX_RX_ENABLE_DISABLE(dir, data->regmap, true);

	/* update the software state of TX/RX */
	sai_tx_rx_sw_enable_disable(dir, data, true);

	return 0;
}

static int sai_trigger(const struct device *dev,
		       enum dai_dir dir,
		       enum dai_trigger_cmd cmd)
{
	switch (cmd) {
	case DAI_TRIGGER_START:
		return sai_trigger_start(dev, dir);
	case DAI_TRIGGER_PAUSE:
		return sai_trigger_pause(dev, dir);
	case DAI_TRIGGER_STOP:
		return sai_trigger_stop(dev, dir);
	case DAI_TRIGGER_POST_STOP:
		return sai_trigger_post_stop(dev, dir);
	case DAI_TRIGGER_PRE_START:
	case DAI_TRIGGER_COPY:
		/* COPY and PRE_START don't require the SAI
		 * driver to do anything at the moment so
		 * mark them as successful via a NULL return
		 *
		 * note: although the rest of the unhandled
		 * trigger commands may be valid, return
		 * an error code for them as they aren't
		 * implemented ATM (since they're not
		 * mandatory for the SAI driver to work).
		 */
		return 0;
	default:
		LOG_ERR("invalid trigger command: %d", cmd);
		return -EINVAL;
	}

	CODE_UNREACHABLE;
}

static int sai_probe(const struct device *dev)
{
	/* nothing to be done here but sadly mandatory to implement */
	return 0;
}

static int sai_remove(const struct device *dev)
{
	/* nothing to be done here but sadly mandatory to implement */
	return 0;
}

int dai_nxp_sai_get_status(const struct device *dev, enum dai_dir dir,
			   struct dai_nxp_sai_status *status)
{
	struct sai_data *data = dev->data;

	if (status == NULL || (dir != DAI_DIR_RX && dir != DAI_DIR_TX)) {
		return -EINVAL;
	}

	status->state = sai_get_state(dir, data);
	status->fifo_errors = dir == DAI_DIR_RX ? data->rx_fifo_errors : data->tx_fifo_errors;

	return 0;
}

int dai_nxp_sai_recover(const struct device *dev, enum dai_dir dir)
{
	const struct sai_config *cfg = dev->config;
	struct sai_data *data = dev->data;
	unsigned int key;
	int i;

	if (dir != DAI_DIR_RX && dir != DAI_DIR_TX) {
		return -EINVAL;
	}

	key = irq_lock();

	if (sai_get_state(dir, data) != DAI_STATE_ERROR) {
		irq_unlock(key);
		return -EPERM;
	}

	/*
	 * The reference manual's recovery for FCONT = 0: while FEF is set
	 * the direction discards data (RX) or sends zeros (TX), and it
	 * resumes at the start of the next frame once FEF clears. The FIFO
	 * must be emptied before that, and FEF being set is also what makes
	 * emptying it legal with the direction still enabled. The
	 * transmitter/receiver, and so the bit clock and frame sync it may
	 * be generating, never stop.
	 */
	sai_tx_rx_fifo_reset(data, dir);

	/* as in sai_trigger_start(): a frame of zeros so TX does not
	 * underrun again at the very next frame
	 */
	if (dir == DAI_DIR_TX) {
		for (i = 0; i < data->cfg.channels; i++) {
			SAI_WriteData(UINT_TO_I2S(data->regmap), cfg->tx_dline, 0x0);
		}
	}

	sai_tx_rx_dma_enable(data, dir, true);
	sai_tx_rx_status_clear(data, dir, kSAI_FIFOErrorFlag);
	sai_tx_rx_irq_enable(data, dir, kSAI_FIFOErrorInterruptEnable, true);

	/* not through sai_update_state(): ERROR -> RUNNING is legal only
	 * here, and allowing it there would let a START skip its stop
	 */
	if (dir == DAI_DIR_RX) {
		data->rx_state = DAI_STATE_RUNNING;
	} else {
		data->tx_state = DAI_STATE_RUNNING;
	}

	irq_unlock(key);

	return 0;
}

int dai_nxp_sai_set_error_callback(const struct device *dev, dai_nxp_sai_error_cb_t cb,
				   void *user_data)
{
	struct sai_data *data = dev->data;
	unsigned int key;

	/* the ISR reads the pair, so it must never see a new callback with
	 * the old user data
	 */
	key = irq_lock();
	data->error_cb = cb;
	data->error_cb_data = user_data;
	irq_unlock(key);

	return 0;
}

static DEVICE_API(dai, sai_api) = {
	.config_set = sai_config_set,
	.config_get = sai_config_get,
	.trigger = sai_trigger,
	.get_properties = sai_get_properties,
	.probe = sai_probe,
	.remove = sai_remove,
};

static int sai_clks_enable_disable(const struct device *dev, bool enable)
{
	int i, ret;
	const struct sai_config *cfg;
	const struct device *clk_dev;
	void *clk_id;

	cfg = dev->config;

	for (i = 0; i < cfg->clk_data.clock_num; i++) {
		clk_dev = cfg->clk_data.clocks[i].dev;
		clk_id = UINT_TO_POINTER(cfg->clk_data.clocks[i].id);

		if (enable) {
			ret = clock_control_on(clk_dev, clk_id);
		} else {
			ret = clock_control_off(clk_dev, clk_id);
		}

		if (ret < 0) {
			LOG_ERR("failed to gate/ungate clock %u: %d",
				cfg->clk_data.clocks[i].id, ret);
			return ret;
		}
	}

	return 0;
}

__maybe_unused static int sai_pm_action(const struct device *dev,
					enum pm_device_action action)
{
	bool enable = true;

	int ret;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		enable = false;
		break;
	case PM_DEVICE_ACTION_TURN_ON:
	case PM_DEVICE_ACTION_TURN_OFF:
		return 0;
	default:
		return -ENOTSUP;
	}

	ret = sai_clks_enable_disable(dev, enable);

	/* a POST_STOP issued while suspended could only record which bit
	 * clocks to stop (see sai_trigger_post_stop()); write them now,
	 * whatever resumed the SAI, before BCLK runs for more than a frame
	 */
	if (ret == 0 && enable) {
		sai_tx_rx_csr_reapply(dev->data, DAI_DIR_TX);
		sai_tx_rx_csr_reapply(dev->data, DAI_DIR_RX);
	}

	return ret;
}

static int sai_init(const struct device *dev)
{
	const struct sai_config *cfg;
	struct sai_data *data;
	int ret;

	cfg = dev->config;
	data = dev->data;

	device_map(&data->regmap, cfg->regmap_phys, cfg->regmap_size, K_MEM_CACHE_NONE);

	if (SAI_DLINE_COUNT(cfg->regmap_phys) == -1) {
		LOG_ERR("bad or unsupported SAI instance");
		return -EINVAL;
	}

	if (cfg->tx_dline >= SAI_DLINE_COUNT(cfg->regmap_phys)) {
		LOG_ERR("invalid TX data line index");
		return -EINVAL;
	}

	if (cfg->rx_dline >= SAI_DLINE_COUNT(cfg->regmap_phys)) {
		LOG_ERR("invalid RX data line index");
		return -EINVAL;
	}

#ifndef CONFIG_PM_DEVICE_RUNTIME
	ret = sai_clks_enable_disable(dev, true);
	if (ret < 0) {
		return ret;
	}
#endif /* CONFIG_PM_DEVICE_RUNTIME */

	/* note: optional operation so -ENOENT is allowed (i.e: we
	 * allow the default state to not be defined)
	 */
	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0 && ret != -ENOENT) {
		return ret;
	}

	/* set TX/RX default states */
	data->tx_state = DAI_STATE_NOT_READY;
	data->rx_state = DAI_STATE_NOT_READY;

	/* register ISR */
	cfg->irq_config();

	return pm_device_runtime_enable(dev);
}

#define SAI_INIT(inst)								\
										\
PINCTRL_DT_INST_DEFINE(inst);							\
										\
BUILD_ASSERT(SAI_FIFO_DEPTH(inst) > 0 &&					\
	     SAI_FIFO_DEPTH(inst) <= _SAI_FIFO_DEPTH(inst),			\
	     "invalid FIFO depth");						\
										\
BUILD_ASSERT(SAI_RX_FIFO_WATERMARK(inst) > 0 &&					\
	     SAI_RX_FIFO_WATERMARK(inst) <= _SAI_FIFO_DEPTH(inst),		\
	     "invalid RX FIFO watermark");					\
										\
BUILD_ASSERT(SAI_TX_FIFO_WATERMARK(inst) > 0 &&					\
	     SAI_TX_FIFO_WATERMARK(inst) <= _SAI_FIFO_DEPTH(inst),		\
	     "invalid TX FIFO watermark");					\
										\
BUILD_ASSERT(IS_ENABLED(CONFIG_SAI_HAS_MCLK_CONFIG_OPTION) ||			\
	     !DT_INST_PROP(inst, mclk_is_output),				\
	     "SAI doesn't support MCLK config but mclk_is_output is specified");\
										\
BUILD_ASSERT(SAI_TX_SYNC_MODE(inst) != SAI_RX_SYNC_MODE(inst) ||		\
	     SAI_TX_SYNC_MODE(inst) != kSAI_ModeSync,				\
	     "transmitter and receiver can't be both SYNC with each other");	\
										\
static const struct dai_properties sai_tx_props_##inst = {			\
	.fifo_address = SAI_TX_FIFO_BASE(inst, SAI_TX_DLINE_INDEX(inst)),	\
	.fifo_depth = SAI_FIFO_DEPTH(inst) * CONFIG_SAI_FIFO_WORD_SIZE,		\
	.dma_hs_id = SAI_TX_RX_DMA_HANDSHAKE(inst, tx),				\
};										\
										\
static const struct dai_properties sai_rx_props_##inst = {			\
	.fifo_address = SAI_RX_FIFO_BASE(inst, SAI_RX_DLINE_INDEX(inst)),	\
	.fifo_depth = SAI_FIFO_DEPTH(inst) * CONFIG_SAI_FIFO_WORD_SIZE,		\
	.dma_hs_id = SAI_TX_RX_DMA_HANDSHAKE(inst, rx),				\
};										\
										\
void irq_config_##inst(void)							\
{										\
	IRQ_CONNECT(DT_INST_IRQN(inst),						\
		    0,								\
		    sai_isr,							\
		    DEVICE_DT_INST_GET(inst),					\
		    0);								\
}										\
										\
static struct sai_config sai_config_##inst = {					\
	.regmap_phys = DT_INST_REG_ADDR(inst),					\
	.regmap_size = DT_INST_REG_SIZE(inst),					\
	.irq = DT_INST_IRQN(inst),						\
	.clk_data = SAI_CLOCK_DATA_DECLARE(inst),				\
	.rx_fifo_watermark = SAI_RX_FIFO_WATERMARK(inst),			\
	.tx_fifo_watermark = SAI_TX_FIFO_WATERMARK(inst),			\
	.mclk_is_output = DT_INST_PROP(inst, mclk_is_output),			\
	.tx_props = &sai_tx_props_##inst,					\
	.rx_props = &sai_rx_props_##inst,					\
	.irq_config = irq_config_##inst,					\
	.tx_sync_mode = SAI_TX_SYNC_MODE(inst),					\
	.rx_sync_mode = SAI_RX_SYNC_MODE(inst),					\
	.tx_dline = SAI_TX_DLINE_INDEX(inst),					\
	.rx_dline = SAI_RX_DLINE_INDEX(inst),					\
	.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),				\
};										\
										\
static struct sai_data sai_data_##inst = {					\
	.cfg.type = DAI_IMX_SAI,						\
	.cfg.dai_index = DT_INST_PROP_OR(inst, dai_index, 0),			\
};										\
										\
PM_DEVICE_DT_INST_DEFINE(inst, sai_pm_action);					\
										\
DEVICE_DT_INST_DEFINE(inst, &sai_init, PM_DEVICE_DT_INST_GET(inst),		\
		      &sai_data_##inst, &sai_config_##inst,			\
		      POST_KERNEL, CONFIG_DAI_INIT_PRIORITY,			\
		      &sai_api);						\

DT_INST_FOREACH_STATUS_OKAY(SAI_INIT);
