// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ALSA SoC Audio Layer - Rockchip SAI Controller driver
 *
 * Copyright (c) 2022 Rockchip Electronics Co. Ltd.
 */

#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>
#include <sound/tlv.h>

#include "rockchip_sai.h"
#include "rockchip_dlp_pcm.h"
#include "rockchip_utils.h"

#define DRV_NAME		"rockchip-sai"

#define CLK_SHIFT_RATE_HZ_MAX	5
#define FW_RATIO_MAX		8
#define FW_RATIO_MIN		1
#define MAXBURST_PER_FIFO	8

#define DEFAULT_FS		48000
#define TIMEOUT_US		1000
#define WAIT_TIME_MS_MAX	10000
#define QUIRK_ALWAYS_ON		BIT(0)

enum fpw_mode {
	FPW_ONE_BCLK_WIDTH,
	FPW_ONE_SLOT_WIDTH,
	FPW_HALF_FRAME_WIDTH,
};

struct rk_sai_dev {
	struct device *dev;
	struct clk *hclk;
	struct clk *mclk;
	struct regmap *regmap;
	struct reset_control *rst_h;
	struct reset_control *rst_m;
	struct snd_dmaengine_dai_dma_data capture_dma_data;
	struct snd_dmaengine_dai_dma_data playback_dma_data;
	struct snd_pcm_substream *substreams[SNDRV_PCM_STREAM_LAST + 1];
	unsigned int wait_time[SNDRV_PCM_STREAM_LAST + 1];
	unsigned int tx_lanes;
	unsigned int rx_lanes;
	unsigned int quirks;
	enum fpw_mode fpw;
	int  fw_ratio;
	bool has_capture;
	bool has_playback;
	bool is_master_mode;
	bool is_tdm;
	bool is_clk_auto;
};

static const struct sai_of_quirks {
	char *quirk;
	int id;
} of_quirks[] = {
	{
		.quirk = "rockchip,always-on",
		.id = QUIRK_ALWAYS_ON,
	},
};

static int rockchip_sai_runtime_suspend(struct device *dev)
{
	struct rk_sai_dev *sai = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	if (sai->is_master_mode)
		regmap_update_bits(sai->regmap, SAI_XFER,
				   SAI_XFER_CLK_MASK |
				   SAI_XFER_FSS_MASK,
				   SAI_XFER_CLK_DIS |
				   SAI_XFER_FSS_DIS);

	ret = regmap_read_poll_timeout_atomic(sai->regmap, SAI_XFER, val,
					      (val & SAI_XFER_FS_IDLE), 10, TIMEOUT_US);
	if (ret < 0)
		dev_warn(sai->dev, "Failed to idle FS\n");

	regcache_cache_only(sai->regmap, true);
	/*
	 * After FS idle, should wait at least 2 BCLK cycle to make sure
	 * the CLK gate operation done, and then disable mclk.
	 *
	 * Otherwise, the BCLK is still ungated. once the mclk is enabled,
	 * there maybe a risk that a few BCLK cycle leak. especially for
	 * low speed situation, such as 8k samplerate.
	 *
	 * The best way is to use delay per samplerate, but, the max time
	 * is quite a tiny value, so, let's make it simple to use the max
	 * time.
	 *
	 * The max BCLK cycle time is: 31us @ 8K-8Bit (64K BCLK)
	 */
	udelay(40);
	clk_disable_unprepare(sai->mclk);
	clk_disable_unprepare(sai->hclk);

	return 0;
}

static int rockchip_sai_runtime_resume(struct device *dev)
{
	struct rk_sai_dev *sai = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(sai->hclk);
	if (ret)
		goto err_hclk;

	ret = clk_prepare_enable(sai->mclk);
	if (ret)
		goto err_mclk;

	regcache_cache_only(sai->regmap, false);
	regcache_mark_dirty(sai->regmap);
	ret = regcache_sync(sai->regmap);
	if (ret)
		goto err_regmap;

	if (sai->quirks & QUIRK_ALWAYS_ON && sai->is_master_mode)
		regmap_update_bits(sai->regmap, SAI_XFER,
				   SAI_XFER_CLK_MASK |
				   SAI_XFER_FSS_MASK,
				   SAI_XFER_CLK_EN |
				   SAI_XFER_FSS_EN);

	return 0;

err_regmap:
	clk_disable_unprepare(sai->mclk);
err_mclk:
	clk_disable_unprepare(sai->hclk);
err_hclk:
	return ret;
}

static void rockchip_sai_fifo_xrun_detect(struct rk_sai_dev *sai,
					  int stream, bool en)
{
	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* clear irq status which was asserted before TXUIE enabled */
		regmap_update_bits(sai->regmap, SAI_INTCR,
				   SAI_INTCR_TXUIC, SAI_INTCR_TXUIC);
		regmap_update_bits(sai->regmap, SAI_INTCR,
				   SAI_INTCR_TXUIE_MASK,
				   SAI_INTCR_TXUIE(en));
	} else {
		/* clear irq status which was asserted before RXOIE enabled */
		regmap_update_bits(sai->regmap, SAI_INTCR,
				   SAI_INTCR_RXOIC, SAI_INTCR_RXOIC);
		regmap_update_bits(sai->regmap, SAI_INTCR,
				   SAI_INTCR_RXOIE_MASK,
				   SAI_INTCR_RXOIE(en));
	}
}

static void rockchip_sai_dma_ctrl(struct rk_sai_dev *sai,
				  int stream, bool en)
{
	if (!en)
		rockchip_sai_fifo_xrun_detect(sai, stream, 0);

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_update_bits(sai->regmap, SAI_DMACR,
				   SAI_DMACR_TDE_MASK,
				   SAI_DMACR_TDE(en));
	} else {
		regmap_update_bits(sai->regmap, SAI_DMACR,
				   SAI_DMACR_RDE_MASK,
				   SAI_DMACR_RDE(en));
	}

	if (en)
		rockchip_sai_fifo_xrun_detect(sai, stream, 1);
}

static void rockchip_sai_reset(struct rk_sai_dev *sai)
{
	/*
	 * Suggest to do reset hclk domain and then do mclk
	 * domain, especially for SLAVE mode without CLK in.
	 *
	 * It will be failed for mclk domain reset on SLAVE mode
	 * without CLK in, we workaround this by do hclk reset
	 * to bring controller back to master, and then do
	 * mclk domain reset, at last, recover regmap config.
	 */
	reset_control_assert(sai->rst_h);
	/* delay for reset assert done */
	udelay(10);
	reset_control_deassert(sai->rst_h);
	/* delay for reset assert done */
	udelay(10);
	reset_control_assert(sai->rst_m);
	/* delay for reset deassert done */
	udelay(10);
	reset_control_deassert(sai->rst_m);
	/* delay for reset deassert done */
	udelay(10);

	/* recover regmap config */
	regcache_mark_dirty(sai->regmap);
	regcache_sync(sai->regmap);
}

static int rockchip_sai_clear(struct rk_sai_dev *sai, unsigned int clr)
{
	unsigned int val = 0;
	int ret = 0;

	regmap_update_bits(sai->regmap, SAI_CLR, clr, clr);
	ret = regmap_read_poll_timeout_atomic(sai->regmap, SAI_CLR, val,
					      !(val & clr), 10, TIMEOUT_US);
	if (ret < 0) {
		dev_warn(sai->dev, "Failed to clear %u\n", clr);
		goto reset;
	}

	return 0;

reset:
	rockchip_sai_reset(sai);

	return 0;
}

static void rockchip_sai_xfer_start(struct rk_sai_dev *sai, int stream)
{
	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_update_bits(sai->regmap, SAI_XFER,
				   SAI_XFER_TXS_MASK,
				   SAI_XFER_TXS_EN);
	} else {
		regmap_update_bits(sai->regmap, SAI_XFER,
				   SAI_XFER_RXS_MASK,
				   SAI_XFER_RXS_EN);
	}
}

static void rockchip_sai_xfer_stop(struct rk_sai_dev *sai, int stream)
{
	unsigned int msk, val, clr, idle;
	int ret;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		msk = SAI_XFER_TXS_MASK;
		val = SAI_XFER_TXS_DIS;
		clr = SAI_CLR_TXC;
		idle = SAI_XFER_TX_IDLE;
	} else {
		msk = SAI_XFER_RXS_MASK;
		val = SAI_XFER_RXS_DIS;
		clr = SAI_CLR_RXC;
		idle = SAI_XFER_RX_IDLE;
	}

	regmap_update_bits(sai->regmap, SAI_XFER, msk, val);
	ret = regmap_read_poll_timeout_atomic(sai->regmap, SAI_XFER, val,
					      (val & idle), 10, TIMEOUT_US);
	if (ret < 0)
		dev_warn(sai->dev, "Failed to idle stream %d\n", stream);

	rockchip_sai_clear(sai, clr);
}

static void rockchip_sai_start(struct rk_sai_dev *sai, int stream)
{
	rockchip_sai_dma_ctrl(sai, stream, 1);
	rockchip_sai_xfer_start(sai, stream);
}

static void rockchip_sai_stop(struct rk_sai_dev *sai, int stream)
{
	rockchip_sai_dma_ctrl(sai, stream, 0);
	rockchip_sai_xfer_stop(sai, stream);
}

static void rockchip_sai_fmt_create(struct rk_sai_dev *sai, unsigned int fmt)
{
	unsigned int xcr_mask = 0, xcr_val = 0, xsft_mask = 0, xsft_val = 0;
	unsigned int fscr_mask = 0, fscr_val = 0;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_RIGHT_J:
		xcr_mask = SAI_XCR_VDJ_MASK | SAI_XCR_EDGE_SHIFT_MASK;
		xcr_val = SAI_XCR_VDJ_R | SAI_XCR_EDGE_SHIFT_0;
		xsft_mask = SAI_XSHIFT_SEL_MASK;
		xsft_val = SAI_XSHIFT_SEL(0);
		fscr_mask = SAI_FSCR_EDGE_MASK;
		fscr_val = SAI_FSCR_EDGE_DUAL;
		sai->fpw = FPW_HALF_FRAME_WIDTH;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		xcr_mask = SAI_XCR_VDJ_MASK | SAI_XCR_EDGE_SHIFT_MASK;
		xcr_val = SAI_XCR_VDJ_L | SAI_XCR_EDGE_SHIFT_0;
		xsft_mask = SAI_XSHIFT_SEL_MASK;
		xsft_val = SAI_XSHIFT_SEL(0);
		fscr_mask = SAI_FSCR_EDGE_MASK;
		fscr_val = SAI_FSCR_EDGE_DUAL;
		sai->fpw = FPW_HALF_FRAME_WIDTH;
		break;
	case SND_SOC_DAIFMT_I2S:
		xcr_mask = SAI_XCR_VDJ_MASK | SAI_XCR_EDGE_SHIFT_MASK;
		xcr_val = SAI_XCR_VDJ_L | SAI_XCR_EDGE_SHIFT_1;
		xsft_mask = SAI_XSHIFT_SEL_MASK;
		xsft_val = SAI_XSHIFT_SEL(2);
		fscr_mask = SAI_FSCR_EDGE_MASK;
		fscr_val = SAI_FSCR_EDGE_DUAL;
		sai->fpw = FPW_HALF_FRAME_WIDTH;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		xcr_mask = SAI_XCR_VDJ_MASK | SAI_XCR_EDGE_SHIFT_MASK;
		xcr_val = SAI_XCR_VDJ_L | SAI_XCR_EDGE_SHIFT_0;
		xsft_mask = SAI_XSHIFT_SEL_MASK;
		xsft_val = SAI_XSHIFT_SEL(2);
		fscr_mask = SAI_FSCR_EDGE_MASK;
		fscr_val = SAI_FSCR_EDGE_RISING;
		sai->fpw = FPW_ONE_BCLK_WIDTH;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		xcr_mask = SAI_XCR_VDJ_MASK | SAI_XCR_EDGE_SHIFT_MASK;
		xcr_val = SAI_XCR_VDJ_L | SAI_XCR_EDGE_SHIFT_0;
		xsft_mask = SAI_XSHIFT_SEL_MASK;
		xsft_val = SAI_XSHIFT_SEL(0);
		fscr_mask = SAI_FSCR_EDGE_MASK;
		fscr_val = SAI_FSCR_EDGE_RISING;
		sai->fpw = FPW_ONE_BCLK_WIDTH;
		break;
	default:
		dev_err(sai->dev, "Unsupported fmt %u\n", fmt);
		break;
	}

	regmap_update_bits(sai->regmap, SAI_TXCR, xcr_mask, xcr_val);
	regmap_update_bits(sai->regmap, SAI_RXCR, xcr_mask, xcr_val);
	regmap_update_bits(sai->regmap, SAI_TX_SHIFT, xsft_mask, xsft_val);
	regmap_update_bits(sai->regmap, SAI_RX_SHIFT, xsft_mask, xsft_val);
	regmap_update_bits(sai->regmap, SAI_FSCR, fscr_mask, fscr_val);
}

static int rockchip_sai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct rk_sai_dev *sai = snd_soc_dai_get_drvdata(dai);
	unsigned int mask = 0, val = 0;
	int ret = 0;

	pm_runtime_get_sync(dai->dev);
	mask = SAI_CKR_MSS_MASK;
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		val = SAI_CKR_MSS_MASTER;
		sai->is_master_mode = true;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		val = SAI_CKR_MSS_SLAVE;
		sai->is_master_mode = false;
		break;
	default:
		ret = -EINVAL;
		goto err_pm_put;
	}

	regmap_update_bits(sai->regmap, SAI_CKR, mask, val);

	mask = SAI_CKR_CKP_MASK | SAI_CKR_FSP_MASK;
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		val = SAI_CKR_CKP_NORMAL | SAI_CKR_FSP_NORMAL;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		val = SAI_CKR_CKP_NORMAL | SAI_CKR_FSP_INVERTED;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		val = SAI_CKR_CKP_INVERTED | SAI_CKR_FSP_NORMAL;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		val = SAI_CKR_CKP_INVERTED | SAI_CKR_FSP_INVERTED;
		break;
	default:
		ret = -EINVAL;
		goto err_pm_put;
	}

	regmap_update_bits(sai->regmap, SAI_CKR, mask, val);

	rockchip_sai_fmt_create(sai, fmt);

err_pm_put:
	pm_runtime_put(dai->dev);

	return ret;
}

static unsigned int rockchip_sai_lanes_auto(struct snd_pcm_hw_params *params,
					    struct snd_soc_dai *dai)
{
	struct rk_sai_dev *sai = snd_soc_dai_get_drvdata(dai);
	unsigned int lanes = 1;

	if (!sai->is_tdm)
		lanes = DIV_ROUND_UP(params_channels(params), 2);

	return lanes;
}

static int rockchip_sai_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct rk_sai_dev *sai = snd_soc_dai_get_drvdata(dai);
	struct snd_dmaengine_dai_dma_data *dma_data;
	unsigned int mclk_rate, mclk_req_rate, bclk_rate, div_bclk;
	unsigned int ch_per_lane, lanes, slot_width;
	unsigned int val, fscr, reg, fifo;

	dma_data = snd_soc_dai_get_dma_data(dai, substream);
	dma_data->maxburst = MAXBURST_PER_FIFO * params_channels(params) / 2;

	lanes = rockchip_sai_lanes_auto(params, dai);

	regmap_read(sai->regmap, SAI_DMACR, &val);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		reg = SAI_TXCR;
		if (sai->tx_lanes)
			lanes = sai->tx_lanes;
		fifo = SAI_DMACR_TDL_V(val) * lanes;
	} else {
		reg = SAI_RXCR;
		if (sai->rx_lanes)
			lanes = sai->rx_lanes;
		fifo = SAI_DMACR_TDL_V(val) * lanes;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
	case SNDRV_PCM_FORMAT_U8:
		val = SAI_XCR_VDW(8);
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		val = SAI_XCR_VDW(16);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		val = SAI_XCR_VDW(24);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
	case SNDRV_PCM_FORMAT_IEC958_SUBFRAME_LE:
		val = SAI_XCR_VDW(32);
		break;
	default:
		return -EINVAL;
	}

	val |= SAI_XCR_CSR(lanes);

	regmap_update_bits(sai->regmap, reg, SAI_XCR_VDW_MASK | SAI_XCR_CSR_MASK, val);

	regmap_read(sai->regmap, reg, &val);

	slot_width = SAI_XCR_SBW_V(val);
	ch_per_lane = params_channels(params) / lanes;

	regmap_update_bits(sai->regmap, reg, SAI_XCR_SNB_MASK,
			   SAI_XCR_SNB(ch_per_lane));

	fscr = SAI_FSCR_FW(sai->fw_ratio * slot_width * ch_per_lane);

	switch (sai->fpw) {
	case FPW_ONE_BCLK_WIDTH:
		fscr |= SAI_FSCR_FPW(1);
		break;
	case FPW_ONE_SLOT_WIDTH:
		fscr |= SAI_FSCR_FPW(slot_width);
		break;
	case FPW_HALF_FRAME_WIDTH:
		fscr |= SAI_FSCR_FPW(sai->fw_ratio * slot_width * ch_per_lane / 2);
		break;
	default:
		dev_err(sai->dev, "Invalid Frame Pulse Width %d\n", sai->fpw);
		return -EINVAL;
	}

	regmap_update_bits(sai->regmap, SAI_FSCR,
			   SAI_FSCR_FW_MASK | SAI_FSCR_FPW_MASK, fscr);

	if (sai->is_master_mode) {
		bclk_rate = sai->fw_ratio * slot_width * ch_per_lane * params_rate(params);
		if (sai->is_clk_auto)
			clk_set_rate(sai->mclk, bclk_rate);
		mclk_rate = clk_get_rate(sai->mclk);
		div_bclk = DIV_ROUND_CLOSEST(mclk_rate, bclk_rate);
		mclk_req_rate = bclk_rate * div_bclk;

		if (mclk_rate < mclk_req_rate - CLK_SHIFT_RATE_HZ_MAX ||
		    mclk_rate > mclk_req_rate + CLK_SHIFT_RATE_HZ_MAX) {
			dev_err(sai->dev, "Mismatch mclk: %u, expected %u (+/- %dHz)\n",
				mclk_rate, mclk_req_rate, CLK_SHIFT_RATE_HZ_MAX);
			return -EINVAL;
		}

		regmap_update_bits(sai->regmap, SAI_CKR, SAI_CKR_MDIV_MASK,
				   SAI_CKR_MDIV(div_bclk));
	}

	rockchip_utils_get_performance(substream, params, dai, fifo);

	return 0;
}

static int rockchip_sai_hw_free(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	rockchip_utils_put_performance(substream, dai);

	return 0;
}

static int rockchip_sai_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct rk_sai_dev *sai = snd_soc_dai_get_drvdata(dai);

	if (sai->is_master_mode) {
		/*
		 * Should wait for one BCLK ready after DIV and then ungate
		 * output clk to achieve the clean clk.
		 *
		 * The best way is to use delay per samplerate, but, the max time
		 * is quite a tiny value, so, let's make it simple to use the max
		 * time.
		 *
		 * The max BCLK cycle time is: 15.6us @ 8K-8Bit (64K BCLK)
		 */
		udelay(20);
		regmap_update_bits(sai->regmap, SAI_XFER,
				   SAI_XFER_CLK_MASK |
				   SAI_XFER_FSS_MASK,
				   SAI_XFER_CLK_EN |
				   SAI_XFER_FSS_EN);
	}

	return 0;
}

static int rockchip_sai_trigger(struct snd_pcm_substream *substream,
				int cmd, struct snd_soc_dai *dai)
{
	struct rk_sai_dev *sai = snd_soc_dai_get_drvdata(dai);
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		rockchip_sai_start(sai, substream->stream);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		rockchip_sai_stop(sai, substream->stream);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int rockchip_sai_set_sysclk(struct snd_soc_dai *dai, int clk_id,
				   unsigned int freq, int dir)
{
	struct rk_sai_dev *sai = snd_soc_dai_get_drvdata(dai);
	int ret;

	if (!freq || sai->is_clk_auto)
		return 0;

	ret = clk_set_rate(sai->mclk, freq);
	if (ret)
		dev_err(sai->dev, "Failed to set mclk %d\n", ret);

	return ret;
}

static int rockchip_sai_dai_probe(struct snd_soc_dai *dai)
{
	struct rk_sai_dev *sai = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai,
		sai->has_playback ? &sai->playback_dma_data : NULL,
		sai->has_capture  ? &sai->capture_dma_data  : NULL);

	return 0;
}

static int rockchip_sai_startup(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct rk_sai_dev *sai = snd_soc_dai_get_drvdata(dai);
	int stream = substream->stream;

	if (sai->substreams[stream])
		return -EBUSY;

	if (sai->wait_time[stream])
		substream->wait_time = msecs_to_jiffies(sai->wait_time[stream]);

	sai->substreams[stream] = substream;

	return 0;
}

static void rockchip_sai_shutdown(struct snd_pcm_substream *substream,
				      struct snd_soc_dai *dai)
{
	struct rk_sai_dev *sai = snd_soc_dai_get_drvdata(dai);

	sai->substreams[substream->stream] = NULL;
}

static int rockchip_sai_set_tdm_slot(struct snd_soc_dai *dai,
				     unsigned int tx_mask, unsigned int rx_mask,
				     int slots, int slot_width)
{
	struct rk_sai_dev *sai = snd_soc_dai_get_drvdata(dai);

	pm_runtime_get_sync(dai->dev);
	regmap_update_bits(sai->regmap, SAI_TXCR, SAI_XCR_SBW_MASK,
			   SAI_XCR_SBW(slot_width));
	regmap_update_bits(sai->regmap, SAI_RXCR, SAI_XCR_SBW_MASK,
			   SAI_XCR_SBW(slot_width));
	pm_runtime_put(dai->dev);

	sai->is_tdm = true;

	return 0;
}

static const struct snd_soc_dai_ops rockchip_sai_dai_ops = {
	.startup = rockchip_sai_startup,
	.shutdown = rockchip_sai_shutdown,
	.hw_params = rockchip_sai_hw_params,
	.hw_free = rockchip_sai_hw_free,
	.set_sysclk = rockchip_sai_set_sysclk,
	.set_fmt = rockchip_sai_set_fmt,
	.prepare = rockchip_sai_prepare,
	.trigger = rockchip_sai_trigger,
	.set_tdm_slot = rockchip_sai_set_tdm_slot,
};

static struct snd_soc_dai_driver rockchip_sai_dai = {
	.probe = rockchip_sai_dai_probe,
	.ops = &rockchip_sai_dai_ops,
	.symmetric_rates = 1,
};

static bool rockchip_sai_wr_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SAI_TXCR:
	case SAI_FSCR:
	case SAI_RXCR:
	case SAI_MONO_CR:
	case SAI_XFER:
	case SAI_CLR:
	case SAI_CKR:
	case SAI_DMACR:
	case SAI_INTCR:
	case SAI_TXDR:
	case SAI_PATH_SEL:
	case SAI_TX_SLOT_MASK0:
	case SAI_TX_SLOT_MASK1:
	case SAI_TX_SLOT_MASK2:
	case SAI_TX_SLOT_MASK3:
	case SAI_RX_SLOT_MASK0:
	case SAI_RX_SLOT_MASK1:
	case SAI_RX_SLOT_MASK2:
	case SAI_RX_SLOT_MASK3:
	case SAI_TX_SHIFT:
	case SAI_RX_SHIFT:
		return true;
	default:
		return false;
	}
}

static bool rockchip_sai_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SAI_TXCR:
	case SAI_FSCR:
	case SAI_RXCR:
	case SAI_MONO_CR:
	case SAI_XFER:
	case SAI_CLR:
	case SAI_CKR:
	case SAI_TXFIFOLR:
	case SAI_RXFIFOLR:
	case SAI_DMACR:
	case SAI_INTCR:
	case SAI_INTSR:
	case SAI_TXDR:
	case SAI_RXDR:
	case SAI_PATH_SEL:
	case SAI_TX_SLOT_MASK0:
	case SAI_TX_SLOT_MASK1:
	case SAI_TX_SLOT_MASK2:
	case SAI_TX_SLOT_MASK3:
	case SAI_RX_SLOT_MASK0:
	case SAI_RX_SLOT_MASK1:
	case SAI_RX_SLOT_MASK2:
	case SAI_RX_SLOT_MASK3:
	case SAI_TX_DATA_CNT:
	case SAI_RX_DATA_CNT:
	case SAI_TX_SHIFT:
	case SAI_RX_SHIFT:
	case SAI_VERSION:
		return true;
	default:
		return false;
	}
}

static bool rockchip_sai_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SAI_XFER:
	case SAI_INTCR:
	case SAI_INTSR:
	case SAI_CLR:
	case SAI_TXFIFOLR:
	case SAI_RXFIFOLR:
	case SAI_TXDR:
	case SAI_RXDR:
	case SAI_TX_DATA_CNT:
	case SAI_RX_DATA_CNT:
		return true;
	default:
		return false;
	}
}

static bool rockchip_sai_precious_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SAI_RXDR:
		return true;
	default:
		return false;
	}
}

static const struct reg_default rockchip_sai_reg_defaults[] = {
	{ SAI_TXCR, 0x00000bff },
	{ SAI_FSCR, 0x0001f03f },
	{ SAI_RXCR, 0x00000bff },
	{ SAI_PATH_SEL, 0x0000e4e4 },
};

static const struct regmap_config rockchip_sai_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = SAI_VERSION,
	.reg_defaults = rockchip_sai_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(rockchip_sai_reg_defaults),
	.writeable_reg = rockchip_sai_wr_reg,
	.readable_reg = rockchip_sai_rd_reg,
	.volatile_reg = rockchip_sai_volatile_reg,
	.precious_reg = rockchip_sai_precious_reg,
	.cache_type = REGCACHE_FLAT,
};

static const struct of_device_id rockchip_sai_match[] __maybe_unused = {
	{ .compatible = "rockchip,sai-v1", },
	{},
};

static int rockchip_sai_init_dai(struct rk_sai_dev *sai, struct resource *res,
				 struct snd_soc_dai_driver **dp)
{
	struct device_node *node = sai->dev->of_node;
	struct snd_soc_dai_driver *dai;
	struct property *dma_names;
	const char *dma_name;

	of_property_for_each_string(node, "dma-names", dma_names, dma_name) {
		if (!strcmp(dma_name, "tx"))
			sai->has_playback = true;
		if (!strcmp(dma_name, "rx"))
			sai->has_capture = true;
	}

	dai = devm_kmemdup(sai->dev, &rockchip_sai_dai,
			   sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	if (sai->has_playback) {
		dai->playback.stream_name = "Playback";
		dai->playback.channels_min = 1;
		dai->playback.channels_max = 512;
		dai->playback.rates = SNDRV_PCM_RATE_8000_384000;
		dai->playback.formats = SNDRV_PCM_FMTBIT_S8 |
					SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE |
					SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE;

		sai->playback_dma_data.addr = res->start + SAI_TXDR;
		sai->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		sai->playback_dma_data.maxburst = MAXBURST_PER_FIFO;
	}

	if (sai->has_capture) {
		dai->capture.stream_name = "Capture";
		dai->capture.channels_min = 1;
		dai->capture.channels_max = 512;
		dai->capture.rates = SNDRV_PCM_RATE_8000_384000;
		dai->capture.formats = SNDRV_PCM_FMTBIT_S8 |
				       SNDRV_PCM_FMTBIT_S16_LE |
				       SNDRV_PCM_FMTBIT_S24_LE |
				       SNDRV_PCM_FMTBIT_S32_LE |
				       SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE;

		sai->capture_dma_data.addr = res->start + SAI_RXDR;
		sai->capture_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		sai->capture_dma_data.maxburst = MAXBURST_PER_FIFO;
	}

	regmap_update_bits(sai->regmap, SAI_DMACR, SAI_DMACR_TDL_MASK,
			   SAI_DMACR_TDL(16));
	regmap_update_bits(sai->regmap, SAI_DMACR, SAI_DMACR_RDL_MASK,
			   SAI_DMACR_RDL(16));

	if (dp)
		*dp = dai;

	return 0;
}

static const char * const tx_lanes_text[] = { "Auto", "SDOx1", "SDOx2", "SDOx3", "SDOx4" };
static const char * const rx_lanes_text[] = { "Auto", "SDIx1", "SDIx2", "SDIx3", "SDIx4" };
static const char * const edge_text[] = { "Rising Edge", "Dual Edge" };
static const char * const edge_shift_text[] = { "Normal", "Shift 1 Edge" };

static const char * const fpw_text[] = {
	"One Bclk Width", "One Slot Width", "Half Frame Width" };
static const char * const fw_ratio_text[] = {
	"1", "2", "3", "4", "5", "6", "7", "8" };

static const char * const sjm_text[] = { "Right J", "Left J" };
static const char * const fbm_text[] = { "MSB", "LSB" };
static const char * const vdj_text[] = { "Right J", "Left J" };

static const char * const sbw_text[] = {
	 "0",  "0",  "0",  "0",  "0",  "0",  "0",  "8",
	 "9", "10", "11", "12", "13", "14", "15", "16",
	"17", "18", "19", "20", "21", "22", "23", "24",
	"25", "26", "27", "28", "29", "30", "31", "32", };

static const char * const mono_text[] = { "Disable", "Enable" };

static DECLARE_TLV_DB_SCALE(rmss_tlv, 0, 128, 0);

static const char * const mss_text[] = { "Slave", "Master" };

static const char * const ckp_text[] = { "Normal", "Inverted" };

static const char * const lpx_text[] = {
	"From SDO0", "From SDO1", "From SDO2", "From SDO3" };

static const char * const lps_text[] = { "Disable", "Enable" };
static const char * const sync_out_text[] = { "From CRU", "From IO" };
static const char * const sync_in_text[] = { "From IO", "From Sync Port" };

static const char * const rpaths_text[] = {
	"From SDI0", "From SDI1", "From SDI2", "From SDI3" };

static const char * const tpaths_text[] = {
	"From PATH0", "From PATH1", "From PATH2", "From PATH3" };

/* TXCR */
static SOC_ENUM_SINGLE_DECL(__maybe_unused tsft_enum, SAI_TXCR, 22, edge_shift_text);
static const struct soc_enum tx_lanes_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tx_lanes_text), tx_lanes_text);
static SOC_ENUM_SINGLE_DECL(__maybe_unused tsjm_enum, SAI_TXCR, 19, sjm_text);
static SOC_ENUM_SINGLE_DECL(__maybe_unused tfbm_enum, SAI_TXCR, 18, fbm_text);
static SOC_ENUM_SINGLE_DECL(__maybe_unused tvdj_enum, SAI_TXCR, 10, vdj_text);
static SOC_ENUM_SINGLE_DECL(__maybe_unused tsbw_enum, SAI_TXCR,  5, sbw_text);

/* FSCR */
static SOC_ENUM_SINGLE_DECL(__maybe_unused edge_enum, SAI_FSCR, 24, edge_text);
static const struct soc_enum __maybe_unused fpw_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(fpw_text), fpw_text);
static const struct soc_enum __maybe_unused fw_ratio_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(fw_ratio_text), fw_ratio_text);

/* RXCR */
static SOC_ENUM_SINGLE_DECL(__maybe_unused rsft_enum, SAI_RXCR, 22, edge_shift_text);
static const struct soc_enum rx_lanes_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(rx_lanes_text), rx_lanes_text);
static SOC_ENUM_SINGLE_DECL(__maybe_unused rsjm_enum, SAI_RXCR, 19, sjm_text);
static SOC_ENUM_SINGLE_DECL(__maybe_unused rfbm_enum, SAI_RXCR, 18, fbm_text);
static SOC_ENUM_SINGLE_DECL(__maybe_unused rvdj_enum, SAI_RXCR, 10, vdj_text);
static SOC_ENUM_SINGLE_DECL(__maybe_unused rsbw_enum, SAI_RXCR,  5, sbw_text);

/* MONO_CR */
static SOC_ENUM_SINGLE_DECL(rmono_switch, SAI_MONO_CR, 1, mono_text);
static SOC_ENUM_SINGLE_DECL(tmono_switch, SAI_MONO_CR, 0, mono_text);

/* CKR */
static const struct soc_enum __maybe_unused mss_switch =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mss_text), mss_text);
static SOC_ENUM_SINGLE_DECL(__maybe_unused sp_switch,  SAI_CKR, 1, ckp_text);
static SOC_ENUM_SINGLE_DECL(__maybe_unused fp_switch,  SAI_CKR, 0, ckp_text);

/* PATH_SEL */
static SOC_ENUM_SINGLE_DECL(lp3_enum, SAI_PATH_SEL, 28, lpx_text);
static SOC_ENUM_SINGLE_DECL(lp2_enum, SAI_PATH_SEL, 26, lpx_text);
static SOC_ENUM_SINGLE_DECL(lp1_enum, SAI_PATH_SEL, 24, lpx_text);
static SOC_ENUM_SINGLE_DECL(lp0_enum, SAI_PATH_SEL, 22, lpx_text);
static SOC_ENUM_SINGLE_DECL(lp3_switch, SAI_PATH_SEL, 21, lps_text);
static SOC_ENUM_SINGLE_DECL(lp2_switch, SAI_PATH_SEL, 20, lps_text);
static SOC_ENUM_SINGLE_DECL(lp1_switch, SAI_PATH_SEL, 19, lps_text);
static SOC_ENUM_SINGLE_DECL(lp0_switch, SAI_PATH_SEL, 18, lps_text);
static SOC_ENUM_SINGLE_DECL(sync_out_switch, SAI_PATH_SEL, 17, sync_out_text);
static SOC_ENUM_SINGLE_DECL(sync_in_switch, SAI_PATH_SEL, 16, sync_in_text);
static SOC_ENUM_SINGLE_DECL(rpath3_enum, SAI_PATH_SEL, 14, rpaths_text);
static SOC_ENUM_SINGLE_DECL(rpath2_enum, SAI_PATH_SEL, 12, rpaths_text);
static SOC_ENUM_SINGLE_DECL(rpath1_enum, SAI_PATH_SEL, 10, rpaths_text);
static SOC_ENUM_SINGLE_DECL(rpath0_enum, SAI_PATH_SEL, 8, rpaths_text);
static SOC_ENUM_SINGLE_DECL(tpath3_enum, SAI_PATH_SEL, 6, tpaths_text);
static SOC_ENUM_SINGLE_DECL(tpath2_enum, SAI_PATH_SEL, 4, tpaths_text);
static SOC_ENUM_SINGLE_DECL(tpath1_enum, SAI_PATH_SEL, 2, tpaths_text);
static SOC_ENUM_SINGLE_DECL(tpath0_enum, SAI_PATH_SEL, 0, tpaths_text);

static int __maybe_unused rockchip_sai_fpw_get(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_sai_dev *sai = snd_soc_component_get_drvdata(component);

	ucontrol->value.enumerated.item[0] = sai->fpw;

	return 0;
}

static int __maybe_unused rockchip_sai_fpw_put(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_sai_dev *sai = snd_soc_component_get_drvdata(component);
	int num;

	num = ucontrol->value.enumerated.item[0];
	if (num >= ARRAY_SIZE(fpw_text))
		return -EINVAL;

	sai->fpw = num;

	return 1;
}

static int __maybe_unused rockchip_sai_fw_ratio_get(struct snd_kcontrol *kcontrol,
						    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_sai_dev *sai = snd_soc_component_get_drvdata(component);

	ucontrol->value.enumerated.item[0] = sai->fw_ratio - 1;

	return 0;
}

static int __maybe_unused rockchip_sai_fw_ratio_put(struct snd_kcontrol *kcontrol,
						    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_sai_dev *sai = snd_soc_component_get_drvdata(component);
	int ratio = ucontrol->value.enumerated.item[0] + 1;

	if (ratio > FW_RATIO_MAX || ratio < FW_RATIO_MIN)
		return -EINVAL;

	sai->fw_ratio = ratio;

	return 1;
}

static int rockchip_sai_tx_lanes_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_sai_dev *sai = snd_soc_component_get_drvdata(component);

	ucontrol->value.enumerated.item[0] = sai->tx_lanes;

	return 0;
}

static int rockchip_sai_tx_lanes_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_sai_dev *sai = snd_soc_component_get_drvdata(component);
	int num;

	num = ucontrol->value.enumerated.item[0];
	if (num >= ARRAY_SIZE(tx_lanes_text))
		return -EINVAL;

	sai->tx_lanes = num;

	return 1;
}

static int rockchip_sai_rx_lanes_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_sai_dev *sai = snd_soc_component_get_drvdata(component);

	ucontrol->value.enumerated.item[0] = sai->rx_lanes;

	return 0;
}

static int rockchip_sai_rx_lanes_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_sai_dev *sai = snd_soc_component_get_drvdata(component);
	int num;

	num = ucontrol->value.enumerated.item[0];
	if (num >= ARRAY_SIZE(rx_lanes_text))
		return -EINVAL;

	sai->rx_lanes = num;

	return 1;
}

static int __maybe_unused rockchip_sai_mss_get(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_sai_dev *sai = snd_soc_component_get_drvdata(component);

	ucontrol->value.enumerated.item[0] = sai->is_master_mode;

	return 0;
}

static int __maybe_unused rockchip_sai_mss_put(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_sai_dev *sai = snd_soc_component_get_drvdata(component);
	bool mss;

	/* MUST: do not update mode while stream is running */
	if (snd_soc_component_active(component))
		return -EPERM;

	mss = !!ucontrol->value.enumerated.item[0];
	if (mss == sai->is_master_mode)
		return 0;

	sai->is_master_mode = mss;

	pm_runtime_get_sync(sai->dev);
	if (sai->is_master_mode) {
		/* Switch from Slave to Master */
		regmap_update_bits(sai->regmap, SAI_CKR,
				   SAI_CKR_MSS_MASK,
				   SAI_CKR_MSS_MASTER);
		regmap_update_bits(sai->regmap, SAI_XFER,
				   SAI_XFER_CLK_MASK |
				   SAI_XFER_FSS_MASK,
				   SAI_XFER_CLK_EN |
				   SAI_XFER_FSS_EN);
	} else {
		/* Switch from Master to Slave */
		regmap_update_bits(sai->regmap, SAI_CKR,
				   SAI_CKR_MSS_MASK,
				   SAI_CKR_MSS_SLAVE);
		regmap_update_bits(sai->regmap, SAI_XFER,
				   SAI_XFER_CLK_MASK |
				   SAI_XFER_FSS_MASK,
				   SAI_XFER_CLK_DIS |
				   SAI_XFER_FSS_DIS);
	}
	pm_runtime_put(sai->dev);

	return 1;
}

static int rockchip_sai_clk_auto_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_sai_dev *sai = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = sai->is_clk_auto;

	return 0;
}

static int rockchip_sai_clk_auto_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_sai_dev *sai = snd_soc_component_get_drvdata(component);
	bool clk_auto = ucontrol->value.integer.value[0];

	if (clk_auto == sai->is_clk_auto)
		return 0;

	sai->is_clk_auto = clk_auto;

	return 1;
}

static int rockchip_sai_wait_time_info(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = WAIT_TIME_MS_MAX;
	uinfo->value.integer.step = 1;

	return 0;
}

static int rockchip_sai_rd_wait_time_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_sai_dev *sai = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = sai->wait_time[SNDRV_PCM_STREAM_CAPTURE];

	return 0;
}

static int rockchip_sai_rd_wait_time_put(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_sai_dev *sai = snd_soc_component_get_drvdata(component);

	if (ucontrol->value.integer.value[0] > WAIT_TIME_MS_MAX)
		return -EINVAL;

	sai->wait_time[SNDRV_PCM_STREAM_CAPTURE] = ucontrol->value.integer.value[0];

	return 1;
}

static int rockchip_sai_wr_wait_time_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_sai_dev *sai = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = sai->wait_time[SNDRV_PCM_STREAM_PLAYBACK];

	return 0;
}

static int rockchip_sai_wr_wait_time_put(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_sai_dev *sai = snd_soc_component_get_drvdata(component);

	if (ucontrol->value.integer.value[0] > WAIT_TIME_MS_MAX)
		return -EINVAL;

	sai->wait_time[SNDRV_PCM_STREAM_PLAYBACK] = ucontrol->value.integer.value[0];

	return 1;
}

#define SAI_PCM_WAIT_TIME(xname, xhandler_get, xhandler_put)	\
{	.iface = SNDRV_CTL_ELEM_IFACE_PCM, .name = xname,	\
	.info = rockchip_sai_wait_time_info,			\
	.get = xhandler_get, .put = xhandler_put }

static __maybe_unused DECLARE_TLV_DB_SCALE(fs_shift_tlv, 0, 8192, 0);

static const struct snd_kcontrol_new rockchip_sai_controls[] = {
#ifdef CONFIG_SND_SOC_ROCKCHIP_SAI_VERBOSE
	SOC_ENUM("Transmit Edge Shift", tsft_enum),
	SOC_ENUM("Transmit Store Justified Mode", tsjm_enum),
	SOC_ENUM("Transmit First Bit Mode", tfbm_enum),
	SOC_ENUM("Transmit Valid Data Justified", tvdj_enum),
	SOC_ENUM("Transmit Slot Bit Width", tsbw_enum),

	SOC_ENUM("Receive Edge Shift", rsft_enum),
	SOC_ENUM("Receive Store Justified Mode", rsjm_enum),
	SOC_ENUM("Receive First Bit Mode", rfbm_enum),
	SOC_ENUM("Receive Valid Data Justified", rvdj_enum),
	SOC_ENUM("Receive Slot Bit Width", rsbw_enum),

	SOC_ENUM("Frame Edge Switch", edge_enum),
	SOC_ENUM_EXT("Frame Sync Pulse Width", fpw_enum,
		     rockchip_sai_fpw_get, rockchip_sai_fpw_put),
	SOC_ENUM_EXT("Frame Width Ratio", fw_ratio_enum,
		     rockchip_sai_fw_ratio_get, rockchip_sai_fw_ratio_put),

	SOC_ENUM_EXT("Master Slave Mode Select", mss_switch,
		     rockchip_sai_mss_get, rockchip_sai_mss_put),
	SOC_ENUM("Sclk Polarity", sp_switch),
	SOC_ENUM("Frame Sync Polarity", fp_switch),

	SOC_SINGLE_TLV("Transmit Frame Shift Select", SAI_TX_SHIFT,
		       0, 8192, 0, fs_shift_tlv),
	SOC_SINGLE_TLV("Receive Frame Shift Select", SAI_RX_SHIFT,
		       0, 8192, 0, fs_shift_tlv),
#endif
	SOC_ENUM_EXT("Transmit SDOx Select", tx_lanes_enum,
		     rockchip_sai_tx_lanes_get, rockchip_sai_tx_lanes_put),
	SOC_ENUM_EXT("Receive SDIx Select", rx_lanes_enum,
		     rockchip_sai_rx_lanes_get, rockchip_sai_rx_lanes_put),
	SOC_SINGLE_TLV("Receive Mono Slot Select", SAI_MONO_CR,
		       2, 128, 0, rmss_tlv),
	SOC_ENUM("Receive Mono Switch", rmono_switch),
	SOC_ENUM("Transmit Mono Switch", tmono_switch),

	SOC_ENUM("SDI3 Loopback Src Select", lp3_enum),
	SOC_ENUM("SDI2 Loopback Src Select", lp2_enum),
	SOC_ENUM("SDI1 Loopback Src Select", lp1_enum),
	SOC_ENUM("SDI0 Loopback Src Select", lp0_enum),
	SOC_ENUM("SDI3 Loopback Switch", lp3_switch),
	SOC_ENUM("SDI2 Loopback Switch", lp2_switch),
	SOC_ENUM("SDI1 Loopback Switch", lp1_switch),
	SOC_ENUM("SDI0 Loopback Switch", lp0_switch),
	SOC_ENUM("Sync Out Switch", sync_out_switch),
	SOC_ENUM("Sync In Switch", sync_in_switch),
	SOC_ENUM("Receive PATH3 Source Select", rpath3_enum),
	SOC_ENUM("Receive PATH2 Source Select", rpath2_enum),
	SOC_ENUM("Receive PATH1 Source Select", rpath1_enum),
	SOC_ENUM("Receive PATH0 Source Select", rpath0_enum),
	SOC_ENUM("Transmit SDO3 Source Select", tpath3_enum),
	SOC_ENUM("Transmit SDO2 Source Select", tpath2_enum),
	SOC_ENUM("Transmit SDO1 Source Select", tpath1_enum),
	SOC_ENUM("Transmit SDO0 Source Select", tpath0_enum),

	SOC_SINGLE_BOOL_EXT("Clk Auto Switch", 0,
			    rockchip_sai_clk_auto_get,
			    rockchip_sai_clk_auto_put),

	SAI_PCM_WAIT_TIME("PCM Read Wait Time MS",
			  rockchip_sai_rd_wait_time_get,
			  rockchip_sai_rd_wait_time_put),
	SAI_PCM_WAIT_TIME("PCM Write Wait Time MS",
			  rockchip_sai_wr_wait_time_get,
			  rockchip_sai_wr_wait_time_put),
};

static const struct snd_soc_component_driver rockchip_sai_component = {
	.name = DRV_NAME,
	.controls = rockchip_sai_controls,
	.num_controls = ARRAY_SIZE(rockchip_sai_controls),
};

static irqreturn_t rockchip_sai_isr(int irq, void *devid)
{
	struct rk_sai_dev *sai = (struct rk_sai_dev *)devid;
	struct snd_pcm_substream *substream;
	u32 val;

	regmap_read(sai->regmap, SAI_INTSR, &val);
	if (val & SAI_INTSR_TXUI_ACT) {
		dev_warn_ratelimited(sai->dev, "TX FIFO Underrun\n");
		regmap_update_bits(sai->regmap, SAI_INTCR,
				   SAI_INTCR_TXUIC, SAI_INTCR_TXUIC);
		regmap_update_bits(sai->regmap, SAI_INTCR,
				   SAI_INTCR_TXUIE_MASK,
				   SAI_INTCR_TXUIE(0));
		substream = sai->substreams[SNDRV_PCM_STREAM_PLAYBACK];
		if (substream)
			snd_pcm_stop_xrun(substream);
	}

	if (val & SAI_INTSR_RXOI_ACT) {
		dev_warn_ratelimited(sai->dev, "RX FIFO Overrun\n");
		regmap_update_bits(sai->regmap, SAI_INTCR,
				   SAI_INTCR_RXOIC, SAI_INTCR_RXOIC);
		regmap_update_bits(sai->regmap, SAI_INTCR,
				   SAI_INTCR_RXOIE_MASK,
				   SAI_INTCR_RXOIE(0));
		substream = sai->substreams[SNDRV_PCM_STREAM_CAPTURE];
		if (substream)
			snd_pcm_stop_xrun(substream);
	}

	return IRQ_HANDLED;
}

static int rockchip_sai_keep_clk_always_on(struct rk_sai_dev *sai)
{
	unsigned int mclk_rate, bclk_rate, div_bclk;

	sai->is_master_mode = true;

	/* init I2S fmt default */
	rockchip_sai_fmt_create(sai, SND_SOC_DAIFMT_I2S);

	regmap_update_bits(sai->regmap, SAI_FSCR,
			   SAI_FSCR_FW_MASK |
			   SAI_FSCR_FPW_MASK,
			   SAI_FSCR_FW(64) |
			   SAI_FSCR_FPW(32));

	mclk_rate = clk_get_rate(sai->mclk);
	bclk_rate = DEFAULT_FS * 64;
	div_bclk = DIV_ROUND_CLOSEST(mclk_rate, bclk_rate);

	regmap_update_bits(sai->regmap, SAI_CKR, SAI_CKR_MDIV_MASK,
			   SAI_CKR_MDIV(div_bclk));

	pm_runtime_forbid(sai->dev);

	dev_info(sai->dev, "CLK-ALWAYS-ON: mclk: %d, bclk: %d, fsync: %d\n",
		 mclk_rate, bclk_rate, DEFAULT_FS);

	return 0;
}

static int rockchip_sai_parse_quirks(struct rk_sai_dev *sai)
{
	int ret = 0, i = 0;

	for (i = 0; i < ARRAY_SIZE(of_quirks); i++)
		if (device_property_read_bool(sai->dev, of_quirks[i].quirk))
			sai->quirks |= of_quirks[i].id;

	if (sai->quirks & QUIRK_ALWAYS_ON)
		ret = rockchip_sai_keep_clk_always_on(sai);

	return ret;
}

static int rockchip_sai_get_fifo_count(struct device *dev,
				       struct snd_pcm_substream *substream)
{
	struct rk_sai_dev *sai = dev_get_drvdata(dev);
	int val = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		regmap_read(sai->regmap, SAI_TXFIFOLR, &val);
	else
		regmap_read(sai->regmap, SAI_RXFIFOLR, &val);

	val = ((val & SAI_FIFOLR_XFL3_MASK) >> SAI_FIFOLR_XFL3_SHIFT) +
	      ((val & SAI_FIFOLR_XFL2_MASK) >> SAI_FIFOLR_XFL2_SHIFT) +
	      ((val & SAI_FIFOLR_XFL1_MASK) >> SAI_FIFOLR_XFL1_SHIFT) +
	      ((val & SAI_FIFOLR_XFL0_MASK) >> SAI_FIFOLR_XFL0_SHIFT);

	return val;
}

static const struct snd_dlp_config dconfig = {
	.get_fifo_count = rockchip_sai_get_fifo_count,
};

static int rockchip_sai_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct rk_sai_dev *sai;
	struct snd_soc_dai_driver *dai;
	struct resource *res;
	void __iomem *regs;
	int ret, irq;

	sai = devm_kzalloc(&pdev->dev, sizeof(*sai), GFP_KERNEL);
	if (!sai)
		return -ENOMEM;

	sai->dev = &pdev->dev;
	sai->fw_ratio = 1;
	/* match to register default */
	sai->is_master_mode = true;
	dev_set_drvdata(&pdev->dev, sai);

	sai->rst_h = devm_reset_control_get_optional_exclusive(&pdev->dev, "h");
	if (IS_ERR(sai->rst_h))
		return PTR_ERR(sai->rst_h);

	sai->rst_m = devm_reset_control_get_optional_exclusive(&pdev->dev, "m");
	if (IS_ERR(sai->rst_m))
		return PTR_ERR(sai->rst_m);

	regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	sai->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					    &rockchip_sai_regmap_config);
	if (IS_ERR(sai->regmap))
		return PTR_ERR(sai->regmap);

	irq = platform_get_irq_optional(pdev, 0);
	if (irq > 0) {
		ret = devm_request_irq(&pdev->dev, irq, rockchip_sai_isr,
				       IRQF_SHARED, node->name, sai);
		if (ret) {
			dev_err(&pdev->dev, "Failed to request irq %d\n", irq);
			return ret;
		}
	}

	sai->mclk = devm_clk_get(&pdev->dev, "mclk");
	if (IS_ERR(sai->mclk)) {
		dev_err(&pdev->dev, "Failed to get mclk\n");
		return PTR_ERR(sai->mclk);
	}

	sai->hclk = devm_clk_get(&pdev->dev, "hclk");
	if (IS_ERR(sai->hclk)) {
		dev_err(&pdev->dev, "Failed to get hclk\n");
		return PTR_ERR(sai->hclk);
	}

	ret = rockchip_sai_parse_quirks(sai);
	if (ret)
		return ret;

	ret = rockchip_sai_init_dai(sai, res, &dai);
	if (ret)
		return ret;

	/*
	 * MUST: after pm_runtime_enable step, any register R/W
	 * should be wrapped with pm_runtime_get_sync/put.
	 *
	 * Another approach is to enable the regcache true to
	 * avoid access HW registers.
	 *
	 * Alternatively, performing the registers R/W before
	 * pm_runtime_enable is also a good option.
	 */
	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = rockchip_sai_runtime_resume(&pdev->dev);
		if (ret)
			goto err_runtime_disable;
	}

	ret = devm_snd_soc_register_component(&pdev->dev,
					      &rockchip_sai_component,
					      dai, 1);
	if (ret)
		goto err_runtime_suspend;

	if (device_property_read_bool(&pdev->dev, "rockchip,no-dmaengine")) {
		dev_info(&pdev->dev, "Used for Multi-DAI\n");
		return 0;
	}

	if (device_property_read_bool(&pdev->dev, "rockchip,digital-loopback"))
		ret = devm_snd_dmaengine_dlp_register(&pdev->dev, &dconfig);
	else
		ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);

	if (ret)
		goto err_runtime_suspend;

	return 0;

err_runtime_suspend:
	if (!pm_runtime_status_suspended(&pdev->dev))
		rockchip_sai_runtime_suspend(&pdev->dev);
err_runtime_disable:
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int rockchip_sai_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		rockchip_sai_runtime_suspend(&pdev->dev);

	return 0;
}

static const struct dev_pm_ops rockchip_sai_pm_ops = {
	SET_RUNTIME_PM_OPS(rockchip_sai_runtime_suspend, rockchip_sai_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

static struct platform_driver rockchip_sai_driver = {
	.probe = rockchip_sai_probe,
	.remove = rockchip_sai_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(rockchip_sai_match),
		.pm = &rockchip_sai_pm_ops,
	},
};
module_platform_driver(rockchip_sai_driver);

MODULE_DESCRIPTION("Rockchip SAI ASoC Interface");
MODULE_AUTHOR("Sugar Zhang <sugar.zhang@rock-chips.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, rockchip_sai_match);
