// SPDX-License-Identifier: GPL-2.0-only
/* sound/soc/rockchip/rockchip_i2s.c
 *
 * ALSA SoC Audio Layer - Rockchip I2S Controller driver
 *
 * Copyright (c) 2014 Rockchip Electronics Co. Ltd.
 * Author: Jianqun <jay.xu@rock-chips.com>
 */

#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/clk/rockchip.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>

#include "rockchip_i2s.h"
#include "rockchip_dlp_pcm.h"
#include "rockchip_utils.h"

#define DRV_NAME "rockchip-i2s"

#define CLK_PPM_MIN		(-1000)
#define CLK_PPM_MAX		(1000)

#define DEFAULT_MCLK_FS		256
#define DEFAULT_FS		48000

#define WAIT_TIME_MS_MAX	10000

#define QUIRK_ALWAYS_ON		BIT(0)

struct rk_i2s_pins {
	u32 reg_offset;
	u32 shift;
};

struct rk_i2s_dev {
	struct device *dev;

	struct clk *hclk;
	struct clk *mclk;
	struct clk *mclk_root;

	struct snd_dmaengine_dai_dma_data capture_dma_data;
	struct snd_dmaengine_dai_dma_data playback_dma_data;

	struct regmap *regmap;
	struct regmap *grf;

	struct snd_pcm_substream *substreams[SNDRV_PCM_STREAM_LAST + 1];
	unsigned int wait_time[SNDRV_PCM_STREAM_LAST + 1];

	bool has_capture;
	bool has_playback;

/*
 * Used to indicate the tx/rx status.
 * I2S controller hopes to start the tx and rx together,
 * also to stop them when they are both try to stop.
*/
	bool tx_start;
	bool rx_start;
	bool is_master_mode;
	const struct rk_i2s_pins *pins;
	unsigned int bclk_ratio;
	spinlock_t lock; /* tx/rx lock */
	unsigned int clk_trcm;

	unsigned int mclk_root_rate;
	unsigned int mclk_root_initial_rate;
	int clk_ppm;
	bool mclk_calibrate;

	unsigned int quirks;
};

static struct i2s_of_quirks {
	char *quirk;
	int id;
} of_quirks[] = {
	{
		.quirk = "rockchip,always-on",
		.id = QUIRK_ALWAYS_ON,
	},
};

static int i2s_runtime_suspend(struct device *dev)
{
	struct rk_i2s_dev *i2s = dev_get_drvdata(dev);

	regcache_cache_only(i2s->regmap, true);
	clk_disable_unprepare(i2s->mclk);

	return 0;
}

static int i2s_runtime_resume(struct device *dev)
{
	struct rk_i2s_dev *i2s = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(i2s->mclk);
	if (ret) {
		dev_err(i2s->dev, "clock enable failed %d\n", ret);
		return ret;
	}

	regcache_cache_only(i2s->regmap, false);
	regcache_mark_dirty(i2s->regmap);

	ret = regcache_sync(i2s->regmap);
	if (ret)
		clk_disable_unprepare(i2s->mclk);

	return ret;
}

static inline struct rk_i2s_dev *to_info(struct snd_soc_dai *dai)
{
	return snd_soc_dai_get_drvdata(dai);
}

static int rockchip_i2s_clear(struct rk_i2s_dev *i2s)
{
	unsigned int clr = I2S_CLR_TXC | I2S_CLR_RXC;
	unsigned int val = 0;
	int ret;

	/*
	 * Workaround for FIFO clear on SLAVE mode:
	 *
	 * A Suggest to do reset hclk domain and then do mclk
	 *   domain, especially for SLAVE mode without CLK in.
	 *   at last, recovery regmap config.
	 *
	 * B Suggest to switch to MASTER, and then do FIFO clr,
	 *   at last, bring back to SLAVE.
	 *
	 * Now we choose plan B here.
	 */
	if (!i2s->is_master_mode)
		regmap_update_bits(i2s->regmap, I2S_CKR,
				   I2S_CKR_MSS_MASK, I2S_CKR_MSS_MASTER);
	regmap_update_bits(i2s->regmap, I2S_CLR, clr, clr);

	ret = regmap_read_poll_timeout_atomic(i2s->regmap, I2S_CLR, val,
					      !(val & clr), 10, 100);
	if (!i2s->is_master_mode)
		regmap_update_bits(i2s->regmap, I2S_CKR,
				   I2S_CKR_MSS_MASK, I2S_CKR_MSS_SLAVE);
	if (ret < 0)
		dev_warn(i2s->dev, "failed to clear fifo on %s mode\n",
			 i2s->is_master_mode ? "master" : "slave");

	return ret;
}

static void rockchip_snd_txctrl(struct rk_i2s_dev *i2s, int on)
{
	spin_lock(&i2s->lock);
	if (on) {
		regmap_update_bits(i2s->regmap, I2S_DMACR,
				   I2S_DMACR_TDE_ENABLE, I2S_DMACR_TDE_ENABLE);

		regmap_update_bits(i2s->regmap, I2S_XFER,
				   I2S_XFER_TXS_START | I2S_XFER_RXS_START,
				   I2S_XFER_TXS_START | I2S_XFER_RXS_START);

		i2s->tx_start = true;
	} else {
		i2s->tx_start = false;

		regmap_update_bits(i2s->regmap, I2S_DMACR,
				   I2S_DMACR_TDE_ENABLE, I2S_DMACR_TDE_DISABLE);

		if (!i2s->rx_start && !(i2s->quirks & QUIRK_ALWAYS_ON)) {
			regmap_update_bits(i2s->regmap, I2S_XFER,
					   I2S_XFER_TXS_START |
					   I2S_XFER_RXS_START,
					   I2S_XFER_TXS_STOP |
					   I2S_XFER_RXS_STOP);

			udelay(150);
			rockchip_i2s_clear(i2s);
		}
	}
	spin_unlock(&i2s->lock);
}

static void rockchip_snd_rxctrl(struct rk_i2s_dev *i2s, int on)
{
	spin_lock(&i2s->lock);
	if (on) {
		regmap_update_bits(i2s->regmap, I2S_DMACR,
				   I2S_DMACR_RDE_ENABLE, I2S_DMACR_RDE_ENABLE);

		regmap_update_bits(i2s->regmap, I2S_XFER,
				   I2S_XFER_TXS_START | I2S_XFER_RXS_START,
				   I2S_XFER_TXS_START | I2S_XFER_RXS_START);

		i2s->rx_start = true;
	} else {
		i2s->rx_start = false;

		regmap_update_bits(i2s->regmap, I2S_DMACR,
				   I2S_DMACR_RDE_ENABLE, I2S_DMACR_RDE_DISABLE);

		if (!i2s->tx_start && !(i2s->quirks & QUIRK_ALWAYS_ON)) {
			regmap_update_bits(i2s->regmap, I2S_XFER,
					   I2S_XFER_TXS_START |
					   I2S_XFER_RXS_START,
					   I2S_XFER_TXS_STOP |
					   I2S_XFER_RXS_STOP);

			udelay(150);
			rockchip_i2s_clear(i2s);
		}
	}
	spin_unlock(&i2s->lock);
}

static int rockchip_i2s_set_fmt(struct snd_soc_dai *cpu_dai,
				unsigned int fmt)
{
	struct rk_i2s_dev *i2s = to_info(cpu_dai);
	unsigned int mask = 0, val = 0;
	int ret = 0;

	pm_runtime_get_sync(cpu_dai->dev);
	mask = I2S_CKR_MSS_MASK;
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* Set source clock in Master mode */
		val = I2S_CKR_MSS_MASTER;
		i2s->is_master_mode = true;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		val = I2S_CKR_MSS_SLAVE;
		i2s->is_master_mode = false;
		break;
	default:
		ret = -EINVAL;
		goto err_pm_put;
	}

	regmap_update_bits(i2s->regmap, I2S_CKR, mask, val);

	mask = I2S_CKR_CKP_MASK | I2S_CKR_TLP_MASK | I2S_CKR_RLP_MASK;
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		val = I2S_CKR_CKP_NORMAL |
		      I2S_CKR_TLP_NORMAL |
		      I2S_CKR_RLP_NORMAL;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		val = I2S_CKR_CKP_NORMAL |
		      I2S_CKR_TLP_INVERTED |
		      I2S_CKR_RLP_INVERTED;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		val = I2S_CKR_CKP_INVERTED |
		      I2S_CKR_TLP_NORMAL |
		      I2S_CKR_RLP_NORMAL;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		val = I2S_CKR_CKP_INVERTED |
		      I2S_CKR_TLP_INVERTED |
		      I2S_CKR_RLP_INVERTED;
		break;
	default:
		ret = -EINVAL;
		goto err_pm_put;
	}

	regmap_update_bits(i2s->regmap, I2S_CKR, mask, val);

	mask = I2S_TXCR_IBM_MASK | I2S_TXCR_TFS_MASK | I2S_TXCR_PBM_MASK;
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_RIGHT_J:
		val = I2S_TXCR_IBM_RSJM;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val = I2S_TXCR_IBM_LSJM;
		break;
	case SND_SOC_DAIFMT_I2S:
		val = I2S_TXCR_IBM_NORMAL;
		break;
	case SND_SOC_DAIFMT_DSP_A: /* PCM delay 1 bit mode */
		val = I2S_TXCR_TFS_PCM | I2S_TXCR_PBM_MODE(1);
		break;
	case SND_SOC_DAIFMT_DSP_B: /* PCM no delay mode */
		val = I2S_TXCR_TFS_PCM;
		break;
	default:
		ret = -EINVAL;
		goto err_pm_put;
	}

	regmap_update_bits(i2s->regmap, I2S_TXCR, mask, val);

	mask = I2S_RXCR_IBM_MASK | I2S_RXCR_TFS_MASK | I2S_RXCR_PBM_MASK;
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_RIGHT_J:
		val = I2S_RXCR_IBM_RSJM;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val = I2S_RXCR_IBM_LSJM;
		break;
	case SND_SOC_DAIFMT_I2S:
		val = I2S_RXCR_IBM_NORMAL;
		break;
	case SND_SOC_DAIFMT_DSP_A: /* PCM delay 1 bit mode */
		val = I2S_RXCR_TFS_PCM | I2S_RXCR_PBM_MODE(1);
		break;
	case SND_SOC_DAIFMT_DSP_B: /* PCM no delay mode */
		val = I2S_RXCR_TFS_PCM;
		break;
	default:
		ret = -EINVAL;
		goto err_pm_put;
	}

	regmap_update_bits(i2s->regmap, I2S_RXCR, mask, val);

err_pm_put:
	pm_runtime_put(cpu_dai->dev);

	return ret;
}

static void rockchip_i2s_get_performance(struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *params,
					 struct snd_soc_dai *dai,
					 unsigned int csr)
{
	struct rk_i2s_dev *i2s = to_info(dai);
	unsigned int tdl;
	int fifo;

	regmap_read(i2s->regmap, I2S_DMACR, &tdl);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		fifo = I2S_DMACR_TDL_V(tdl) * I2S_TXCR_CSR_V(csr);
	else
		fifo = I2S_DMACR_RDL_V(tdl) * I2S_RXCR_CSR_V(csr);

	rockchip_utils_get_performance(substream, params, dai, fifo);
}

static int rockchip_i2s_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct rk_i2s_dev *i2s = to_info(dai);
	unsigned int val = 0;
	unsigned int mclk_rate, bclk_rate, div_bclk, div_lrck;

	if (i2s->is_master_mode) {
		mclk_rate = clk_get_rate(i2s->mclk);
		bclk_rate = i2s->bclk_ratio * params_rate(params);
		if (!bclk_rate)
			return -EINVAL;

		div_bclk = DIV_ROUND_CLOSEST(mclk_rate, bclk_rate);
		div_lrck = bclk_rate / params_rate(params);
		regmap_update_bits(i2s->regmap, I2S_CKR,
				   I2S_CKR_MDIV_MASK,
				   I2S_CKR_MDIV(div_bclk));

		regmap_update_bits(i2s->regmap, I2S_CKR,
				   I2S_CKR_TSD_MASK |
				   I2S_CKR_RSD_MASK,
				   I2S_CKR_TSD(div_lrck) |
				   I2S_CKR_RSD(div_lrck));
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		val |= I2S_TXCR_VDW(8);
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		val |= I2S_TXCR_VDW(16);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		val |= I2S_TXCR_VDW(20);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		val |= I2S_TXCR_VDW(24);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
	case SNDRV_PCM_FORMAT_IEC958_SUBFRAME_LE:
		val |= I2S_TXCR_VDW(32);
		break;
	default:
		return -EINVAL;
	}

	switch (params_channels(params)) {
	case 8:
		val |= I2S_CHN_8;
		break;
	case 6:
		val |= I2S_CHN_6;
		break;
	case 4:
		val |= I2S_CHN_4;
		break;
	case 2:
		val |= I2S_CHN_2;
		break;
	default:
		dev_err(i2s->dev, "invalid channel: %d\n",
			params_channels(params));
		return -EINVAL;
	}

	rockchip_i2s_get_performance(substream, params, dai, val);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		regmap_update_bits(i2s->regmap, I2S_RXCR,
				   I2S_RXCR_VDW_MASK | I2S_RXCR_CSR_MASK,
				   val);
	else
		regmap_update_bits(i2s->regmap, I2S_TXCR,
				   I2S_TXCR_VDW_MASK | I2S_TXCR_CSR_MASK,
				   val);

	if (!IS_ERR(i2s->grf) && i2s->pins) {
		regmap_read(i2s->regmap, I2S_TXCR, &val);
		val &= I2S_TXCR_CSR_MASK;

		switch (val) {
		case I2S_CHN_4:
			val = I2S_IO_4CH_OUT_6CH_IN;
			break;
		case I2S_CHN_6:
			val = I2S_IO_6CH_OUT_4CH_IN;
			break;
		case I2S_CHN_8:
			val = I2S_IO_8CH_OUT_2CH_IN;
			break;
		default:
			val = I2S_IO_2CH_OUT_8CH_IN;
			break;
		}

		val <<= i2s->pins->shift;
		val |= (I2S_IO_DIRECTION_MASK << i2s->pins->shift) << 16;
		regmap_write(i2s->grf, i2s->pins->reg_offset, val);
	}

	regmap_update_bits(i2s->regmap, I2S_DMACR, I2S_DMACR_TDL_MASK,
			   I2S_DMACR_TDL(16));
	regmap_update_bits(i2s->regmap, I2S_DMACR, I2S_DMACR_RDL_MASK,
			   I2S_DMACR_RDL(16));

	return 0;
}

static int rockchip_i2s_hw_free(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	rockchip_utils_put_performance(substream, dai);

	return 0;
}

static int rockchip_i2s_trigger(struct snd_pcm_substream *substream,
				int cmd, struct snd_soc_dai *dai)
{
	struct rk_i2s_dev *i2s = to_info(dai);
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			rockchip_snd_rxctrl(i2s, 1);
		else
			rockchip_snd_txctrl(i2s, 1);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			rockchip_snd_rxctrl(i2s, 0);
		else
			rockchip_snd_txctrl(i2s, 0);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int rockchip_i2s_set_bclk_ratio(struct snd_soc_dai *dai,
				       unsigned int ratio)
{
	struct rk_i2s_dev *i2s = to_info(dai);

	i2s->bclk_ratio = ratio;

	return 0;
}

static int rockchip_i2s_clk_set_rate(struct rk_i2s_dev *i2s,
				     struct clk *clk, unsigned long rate,
				     int ppm)
{
	unsigned long rate_target;
	int delta, ret;

	if (ppm == i2s->clk_ppm)
		return 0;

	ret = rockchip_pll_clk_compensation(clk, ppm);
	if (ret != -ENOSYS)
		goto out;

	delta = (ppm < 0) ? -1 : 1;
	delta *= (int)div64_u64((uint64_t)rate * (uint64_t)abs(ppm) + 500000, 1000000);

	rate_target = rate + delta;

	if (!rate_target)
		return -EINVAL;

	ret = clk_set_rate(clk, rate_target);
	if (ret)
		return ret;
out:
	if (!ret)
		i2s->clk_ppm = ppm;

	return ret;
}

static int rockchip_i2s_set_sysclk(struct snd_soc_dai *cpu_dai, int clk_id,
				   unsigned int rate, int dir)
{
	struct rk_i2s_dev *i2s = to_info(cpu_dai);
	unsigned int root_rate, div, delta;
	uint64_t ppm;
	int ret;

	if (rate == 0)
		return 0;

	if (i2s->mclk_calibrate) {
		ret = rockchip_i2s_clk_set_rate(i2s, i2s->mclk_root,
						i2s->mclk_root_rate, 0);
		if (ret)
			return ret;

		root_rate = i2s->mclk_root_rate;
		delta = abs(root_rate % rate - rate);
		ppm = div64_u64((uint64_t)delta * 1000000, (uint64_t)root_rate);

		if (ppm) {
			div = DIV_ROUND_CLOSEST(i2s->mclk_root_initial_rate, rate);
			if (!div)
				return -EINVAL;

			root_rate = rate * round_up(div, 2);
			ret = clk_set_rate(i2s->mclk_root, root_rate);
			if (ret)
				return ret;

			i2s->mclk_root_rate = clk_get_rate(i2s->mclk_root);
		}
	}

	ret = clk_set_rate(i2s->mclk, rate);
	if (ret)
		dev_err(i2s->dev, "Fail to set mclk %d\n", ret);

	return ret;
}

static int rockchip_i2s_clk_compensation_info(struct snd_kcontrol *kcontrol,
					      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = CLK_PPM_MIN;
	uinfo->value.integer.max = CLK_PPM_MAX;
	uinfo->value.integer.step = 1;

	return 0;
}

static int rockchip_i2s_clk_compensation_get(struct snd_kcontrol *kcontrol,
					     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *compnt = snd_soc_kcontrol_component(kcontrol);
	struct rk_i2s_dev *i2s = snd_soc_component_get_drvdata(compnt);

	ucontrol->value.integer.value[0] = i2s->clk_ppm;

	return 0;
}

static int rockchip_i2s_clk_compensation_put(struct snd_kcontrol *kcontrol,
					     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *compnt = snd_soc_kcontrol_component(kcontrol);
	struct rk_i2s_dev *i2s = snd_soc_component_get_drvdata(compnt);
	int ppm = ucontrol->value.integer.value[0];

	if ((ucontrol->value.integer.value[0] < CLK_PPM_MIN) ||
	    (ucontrol->value.integer.value[0] > CLK_PPM_MAX))
		return -EINVAL;

	return rockchip_i2s_clk_set_rate(i2s, i2s->mclk_root, i2s->mclk_root_rate, ppm);
}

static struct snd_kcontrol_new rockchip_i2s_compensation_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_PCM,
	.name = "PCM Clk Compensation In PPM",
	.info = rockchip_i2s_clk_compensation_info,
	.get = rockchip_i2s_clk_compensation_get,
	.put = rockchip_i2s_clk_compensation_put,
};

static int rockchip_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct rk_i2s_dev *i2s = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai,
		i2s->has_playback ? &i2s->playback_dma_data : NULL,
		i2s->has_capture  ? &i2s->capture_dma_data  : NULL);

	if (i2s->mclk_calibrate)
		snd_soc_add_component_controls(dai->component,
					       &rockchip_i2s_compensation_control,
					       1);

	return 0;
}

static int rockchip_i2s_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct rk_i2s_dev *i2s = snd_soc_dai_get_drvdata(dai);
	int stream = substream->stream;

	if (i2s->substreams[stream])
		return -EBUSY;

	if (i2s->wait_time[stream])
		substream->wait_time = msecs_to_jiffies(i2s->wait_time[stream]);

	i2s->substreams[stream] = substream;

	return 0;
}

static void rockchip_i2s_shutdown(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct rk_i2s_dev *i2s = snd_soc_dai_get_drvdata(dai);

	i2s->substreams[substream->stream] = NULL;
}

static const struct snd_soc_dai_ops rockchip_i2s_dai_ops = {
	.startup = rockchip_i2s_startup,
	.shutdown = rockchip_i2s_shutdown,
	.hw_params = rockchip_i2s_hw_params,
	.hw_free = rockchip_i2s_hw_free,
	.set_bclk_ratio	= rockchip_i2s_set_bclk_ratio,
	.set_sysclk = rockchip_i2s_set_sysclk,
	.set_fmt = rockchip_i2s_set_fmt,
	.trigger = rockchip_i2s_trigger,
};

static struct snd_soc_dai_driver rockchip_i2s_dai = {
	.probe = rockchip_i2s_dai_probe,
	.ops = &rockchip_i2s_dai_ops,
};

static int rockchip_i2s_get_bclk_ratio(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *compnt = snd_soc_kcontrol_component(kcontrol);
	struct rk_i2s_dev *i2s = snd_soc_component_get_drvdata(compnt);

	ucontrol->value.integer.value[0] = i2s->bclk_ratio;

	return 0;
}

static int rockchip_i2s_put_bclk_ratio(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *compnt = snd_soc_kcontrol_component(kcontrol);
	struct rk_i2s_dev *i2s = snd_soc_component_get_drvdata(compnt);
	int value = ucontrol->value.integer.value[0];

	if (value == i2s->bclk_ratio)
		return 0;

	i2s->bclk_ratio = value;

	return 1;
}

static int rockchip_i2s_wait_time_info(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = WAIT_TIME_MS_MAX;
	uinfo->value.integer.step = 1;

	return 0;
}

static int rockchip_i2s_rd_wait_time_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_i2s_dev *i2s = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = i2s->wait_time[SNDRV_PCM_STREAM_CAPTURE];

	return 0;
}

static int rockchip_i2s_rd_wait_time_put(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_i2s_dev *i2s = snd_soc_component_get_drvdata(component);

	if (ucontrol->value.integer.value[0] > WAIT_TIME_MS_MAX)
		return -EINVAL;

	i2s->wait_time[SNDRV_PCM_STREAM_CAPTURE] = ucontrol->value.integer.value[0];

	return 1;
}

static int rockchip_i2s_wr_wait_time_get(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_i2s_dev *i2s = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = i2s->wait_time[SNDRV_PCM_STREAM_PLAYBACK];

	return 0;
}

static int rockchip_i2s_wr_wait_time_put(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct rk_i2s_dev *i2s = snd_soc_component_get_drvdata(component);

	if (ucontrol->value.integer.value[0] > WAIT_TIME_MS_MAX)
		return -EINVAL;

	i2s->wait_time[SNDRV_PCM_STREAM_PLAYBACK] = ucontrol->value.integer.value[0];

	return 1;
}

#define SAI_PCM_WAIT_TIME(xname, xhandler_get, xhandler_put)	\
{	.iface = SNDRV_CTL_ELEM_IFACE_PCM, .name = xname,	\
	.info = rockchip_i2s_wait_time_info,			\
	.get = xhandler_get, .put = xhandler_put }

static const struct snd_kcontrol_new rockchip_i2s_snd_controls[] = {
	SOC_SINGLE_EXT("BCLK Ratio", 0, 0, INT_MAX, 0,
		       rockchip_i2s_get_bclk_ratio,
		       rockchip_i2s_put_bclk_ratio),

	SAI_PCM_WAIT_TIME("PCM Read Wait Time MS",
			  rockchip_i2s_rd_wait_time_get,
			  rockchip_i2s_rd_wait_time_put),
	SAI_PCM_WAIT_TIME("PCM Write Wait Time MS",
			  rockchip_i2s_wr_wait_time_get,
			  rockchip_i2s_wr_wait_time_put),
};

static const struct snd_soc_component_driver rockchip_i2s_component = {
	.name = DRV_NAME,
	.controls = rockchip_i2s_snd_controls,
	.num_controls = ARRAY_SIZE(rockchip_i2s_snd_controls),
};

static bool rockchip_i2s_wr_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case I2S_TXCR:
	case I2S_RXCR:
	case I2S_CKR:
	case I2S_DMACR:
	case I2S_INTCR:
	case I2S_XFER:
	case I2S_CLR:
	case I2S_TXDR:
		return true;
	default:
		return false;
	}
}

static bool rockchip_i2s_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case I2S_TXCR:
	case I2S_RXCR:
	case I2S_CKR:
	case I2S_DMACR:
	case I2S_INTCR:
	case I2S_XFER:
	case I2S_CLR:
	case I2S_TXDR:
	case I2S_RXDR:
	case I2S_TXFIFOLR:
	case I2S_RXFIFOLR:
	case I2S_INTSR:
		return true;
	default:
		return false;
	}
}

static bool rockchip_i2s_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case I2S_INTSR:
	case I2S_CLR:
	case I2S_TXFIFOLR:
	case I2S_RXFIFOLR:
	case I2S_TXDR:
	case I2S_RXDR:
		return true;
	default:
		return false;
	}
}

static bool rockchip_i2s_precious_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case I2S_RXDR:
		return true;
	default:
		return false;
	}
}

static const struct reg_default rockchip_i2s_reg_defaults[] = {
	{0x00, 0x0000000f},
	{0x04, 0x0000000f},
	{0x08, 0x00071f1f},
	{0x10, 0x001f0000},
	{0x14, 0x01f00000},
};

static const struct regmap_config rockchip_i2s_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = I2S_RXDR,
	.reg_defaults = rockchip_i2s_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(rockchip_i2s_reg_defaults),
	.writeable_reg = rockchip_i2s_wr_reg,
	.readable_reg = rockchip_i2s_rd_reg,
	.volatile_reg = rockchip_i2s_volatile_reg,
	.precious_reg = rockchip_i2s_precious_reg,
	.cache_type = REGCACHE_FLAT,
};

static const struct rk_i2s_pins rk3399_i2s_pins = {
	.reg_offset = 0xe220,
	.shift = 11,
};

static const struct of_device_id rockchip_i2s_match[] __maybe_unused = {
#ifdef CONFIG_CPU_PX30
	{ .compatible = "rockchip,px30-i2s", },
#endif
#ifdef CONFIG_CPU_RK1808
	{ .compatible = "rockchip,rk1808-i2s", },
#endif
#ifdef CONFIG_CPU_RK3036
	{ .compatible = "rockchip,rk3036-i2s", },
#endif
	{ .compatible = "rockchip,rk3066-i2s", },
#ifdef CONFIG_CPU_RK312X
	{ .compatible = "rockchip,rk3128-i2s", },
#endif
#ifdef CONFIG_CPU_RK3188
	{ .compatible = "rockchip,rk3188-i2s", },
#endif
#ifdef CONFIG_CPU_RK322X
	{ .compatible = "rockchip,rk3228-i2s", },
#endif
#ifdef CONFIG_CPU_RK3288
	{ .compatible = "rockchip,rk3288-i2s", },
#endif
#ifdef CONFIG_CPU_RK3308
	{ .compatible = "rockchip,rk3308-i2s", },
#endif
#ifdef CONFIG_CPU_RK3328
	{ .compatible = "rockchip,rk3328-i2s", },
#endif
#ifdef CONFIG_CPU_RK3366
	{ .compatible = "rockchip,rk3366-i2s", },
#endif
#ifdef CONFIG_CPU_RK3368
	{ .compatible = "rockchip,rk3368-i2s", },
#endif
#ifdef CONFIG_CPU_RK3399
	{ .compatible = "rockchip,rk3399-i2s", .data = &rk3399_i2s_pins },
#endif
#ifdef CONFIG_CPU_RV1126
	{ .compatible = "rockchip,rv1126-i2s", },
#endif
	{},
};

static int rockchip_i2s_init_dai(struct rk_i2s_dev *i2s, struct resource *res,
				 struct snd_soc_dai_driver **dp)
{
	struct device_node *node = i2s->dev->of_node;
	struct snd_soc_dai_driver *dai;
	struct property *dma_names;
	const char *dma_name;
	unsigned int val;

	of_property_for_each_string(node, "dma-names", dma_names, dma_name) {
		if (!strcmp(dma_name, "tx"))
			i2s->has_playback = true;
		if (!strcmp(dma_name, "rx"))
			i2s->has_capture = true;
	}

	dai = devm_kmemdup(i2s->dev, &rockchip_i2s_dai,
			   sizeof(*dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;

	if (i2s->has_playback) {
		dai->playback.stream_name = "Playback";
		dai->playback.channels_min = 2;
		dai->playback.channels_max = 8;
		dai->playback.rates = SNDRV_PCM_RATE_8000_192000;
		dai->playback.formats = SNDRV_PCM_FMTBIT_S8 |
					SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S20_3LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE |
					SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE;

		i2s->playback_dma_data.addr = res->start + I2S_TXDR;
		i2s->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		i2s->playback_dma_data.maxburst = 8;

		if (!device_property_read_u32(i2s->dev, "rockchip,playback-channels", &val)) {
			if (val >= 2 && val <= 8)
				dai->playback.channels_max = val;
		}
	}

	if (i2s->has_capture) {
		dai->capture.stream_name = "Capture";
		dai->capture.channels_min = 2;
		dai->capture.channels_max = 8;
		dai->capture.rates = SNDRV_PCM_RATE_8000_192000;
		dai->capture.formats = SNDRV_PCM_FMTBIT_S8 |
				       SNDRV_PCM_FMTBIT_S16_LE |
				       SNDRV_PCM_FMTBIT_S20_3LE |
				       SNDRV_PCM_FMTBIT_S24_LE |
				       SNDRV_PCM_FMTBIT_S32_LE |
				       SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE;

		i2s->capture_dma_data.addr = res->start + I2S_RXDR;
		i2s->capture_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		i2s->capture_dma_data.maxburst = 8;

		if (!device_property_read_u32(i2s->dev, "rockchip,capture-channels", &val)) {
			if (val >= 2 && val <= 8)
				dai->capture.channels_max = val;
		}
	}

	i2s->clk_trcm = I2S_CKR_TRCM_TXRX;
	if (!device_property_read_u32(i2s->dev, "rockchip,clk-trcm", &val)) {
		if (val >= 0 && val <= 2) {
			i2s->clk_trcm = val << I2S_CKR_TRCM_SHIFT;
			if (i2s->clk_trcm)
				dai->symmetric_rates = 1;
		}
	}

	regmap_update_bits(i2s->regmap, I2S_CKR,
			   I2S_CKR_TRCM_MASK, i2s->clk_trcm);

	if (dp)
		*dp = dai;

	return 0;
}

static int rockchip_i2s_keep_clk_always_on(struct rk_i2s_dev *i2s)
{
	unsigned int mclk_rate = DEFAULT_FS * DEFAULT_MCLK_FS;
	unsigned int bclk_rate = i2s->bclk_ratio * DEFAULT_FS;
	unsigned int div_lrck = i2s->bclk_ratio;
	unsigned int div_bclk;

	div_bclk = DIV_ROUND_CLOSEST(mclk_rate, bclk_rate);

	/* assign generic freq */
	clk_set_rate(i2s->mclk, mclk_rate);

	regmap_update_bits(i2s->regmap, I2S_CKR,
			   I2S_CKR_MDIV_MASK,
			   I2S_CKR_MDIV(div_bclk));
	regmap_update_bits(i2s->regmap, I2S_CKR,
			   I2S_CKR_TSD_MASK |
			   I2S_CKR_RSD_MASK,
			   I2S_CKR_TSD(div_lrck) |
			   I2S_CKR_RSD(div_lrck));
	regmap_update_bits(i2s->regmap, I2S_XFER,
			   I2S_XFER_TXS_START | I2S_XFER_RXS_START,
			   I2S_XFER_TXS_START | I2S_XFER_RXS_START);

	pm_runtime_forbid(i2s->dev);

	dev_info(i2s->dev, "CLK-ALWAYS-ON: mclk: %d, bclk: %d, fsync: %d\n",
		 mclk_rate, bclk_rate, DEFAULT_FS);

	return 0;
}

static int rockchip_i2s_get_fifo_count(struct device *dev,
				       struct snd_pcm_substream *substream)
{
	struct rk_i2s_dev *i2s = dev_get_drvdata(dev);
	unsigned int tx, rx;
	int val = 0;

	regmap_read(i2s->regmap, I2S_TXFIFOLR, &tx);
	regmap_read(i2s->regmap, I2S_RXFIFOLR, &rx);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		val = I2S_FIFOLR_XFL3(tx) +
		      I2S_FIFOLR_XFL2(tx) +
		      I2S_FIFOLR_XFL1(tx) +
		      I2S_FIFOLR_XFL0(tx);
	else
		/* XFL4 is compatible for old version */
		val = I2S_FIFOLR_XFL4(tx) +
		      I2S_FIFOLR_XFL3(rx) +
		      I2S_FIFOLR_XFL2(rx) +
		      I2S_FIFOLR_XFL1(rx) +
		      I2S_FIFOLR_XFL0(rx);

	return val;
}

static const struct snd_dlp_config dconfig = {
	.get_fifo_count = rockchip_i2s_get_fifo_count,
};

static int rockchip_i2s_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	const struct of_device_id *of_id;
	struct rk_i2s_dev *i2s;
	struct snd_soc_dai_driver *dai;
	struct resource *res;
	void __iomem *regs;
	int ret, i, val;

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	spin_lock_init(&i2s->lock);
	i2s->dev = &pdev->dev;

	i2s->grf = syscon_regmap_lookup_by_phandle(node, "rockchip,grf");
	if (!IS_ERR(i2s->grf)) {
		of_id = of_match_device(rockchip_i2s_match, &pdev->dev);
		if (!of_id || !of_id->data)
			return -EINVAL;

		i2s->pins = of_id->data;
	}

	for (i = 0; i < ARRAY_SIZE(of_quirks); i++)
		if (device_property_read_bool(i2s->dev, of_quirks[i].quirk))
			i2s->quirks |= of_quirks[i].id;

	regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	i2s->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					    &rockchip_i2s_regmap_config);
	if (IS_ERR(i2s->regmap)) {
		dev_err(&pdev->dev,
			"Failed to initialise managed register map\n");
		return PTR_ERR(i2s->regmap);
	}

	i2s->bclk_ratio = 64;
	if (!device_property_read_u32(&pdev->dev, "rockchip,bclk-fs", &val)) {
		if ((val >= 32) && (val % 2 == 0))
			i2s->bclk_ratio = val;
	}

	dev_set_drvdata(&pdev->dev, i2s);

	i2s->mclk_calibrate =
		device_property_read_bool(&pdev->dev, "rockchip,mclk-calibrate");
	if (i2s->mclk_calibrate) {
		i2s->mclk_root = devm_clk_get(&pdev->dev, "i2s_clk_root");
		if (IS_ERR(i2s->mclk_root))
			return PTR_ERR(i2s->mclk_root);

		i2s->mclk_root_initial_rate = clk_get_rate(i2s->mclk_root);
		i2s->mclk_root_rate = i2s->mclk_root_initial_rate;
	}

	i2s->mclk = devm_clk_get(&pdev->dev, "i2s_clk");
	if (IS_ERR(i2s->mclk)) {
		dev_err(&pdev->dev, "Can't retrieve i2s master clock\n");
		return PTR_ERR(i2s->mclk);
	}

	/* try to prepare related clocks */
	i2s->hclk = devm_clk_get(&pdev->dev, "i2s_hclk");
	if (IS_ERR(i2s->hclk)) {
		dev_err(&pdev->dev, "Can't retrieve i2s bus clock\n");
		return PTR_ERR(i2s->hclk);
	}
	ret = clk_prepare_enable(i2s->hclk);
	if (ret) {
		dev_err(i2s->dev, "hclock enable failed %d\n", ret);
		return ret;
	}

	ret = rockchip_i2s_init_dai(i2s, res, &dai);
	if (ret)
		goto err_clk;

	/*
	 * CLK_ALWAYS_ON should be placed after all registers write done,
	 * because this situation will enable XFER bit which will make
	 * some registers(depend on XFER) write failed.
	 */
	if (i2s->quirks & QUIRK_ALWAYS_ON) {
		ret = rockchip_i2s_keep_clk_always_on(i2s);
		if (ret)
			goto err_clk;
	}

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
		ret = i2s_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	ret = devm_snd_soc_register_component(&pdev->dev,
					      &rockchip_i2s_component,
					      dai, 1);

	if (ret) {
		dev_err(&pdev->dev, "Could not register DAI\n");
		goto err_suspend;
	}

	if (device_property_read_bool(&pdev->dev, "rockchip,no-dmaengine")) {
		dev_info(&pdev->dev, "Used for Multi-DAI\n");
		return 0;
	}

	if (device_property_read_bool(&pdev->dev, "rockchip,digital-loopback"))
		ret = devm_snd_dmaengine_dlp_register(&pdev->dev, &dconfig);
	else
		ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);

	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM\n");
		goto err_suspend;
	}

	return 0;

err_suspend:
	if (!pm_runtime_status_suspended(&pdev->dev))
		i2s_runtime_suspend(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
err_clk:
	clk_disable_unprepare(i2s->hclk);

	return ret;
}

static int rockchip_i2s_remove(struct platform_device *pdev)
{
	struct rk_i2s_dev *i2s = dev_get_drvdata(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		i2s_runtime_suspend(&pdev->dev);

	clk_disable_unprepare(i2s->hclk);

	return 0;
}

static const struct dev_pm_ops rockchip_i2s_pm_ops = {
	SET_RUNTIME_PM_OPS(i2s_runtime_suspend, i2s_runtime_resume,
			   NULL)
};

static struct platform_driver rockchip_i2s_driver = {
	.probe = rockchip_i2s_probe,
	.remove = rockchip_i2s_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(rockchip_i2s_match),
		.pm = &rockchip_i2s_pm_ops,
	},
};
module_platform_driver(rockchip_i2s_driver);

MODULE_DESCRIPTION("ROCKCHIP IIS ASoC Interface");
MODULE_AUTHOR("jianqun <jay.xu@rock-chips.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, rockchip_i2s_match);
