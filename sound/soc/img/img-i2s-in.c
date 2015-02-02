/*
 * IMG I2S in controller driver
 *
 * Copyright (C) 2014 Imagination Technologies Ltd.
 *
 * Author: Damien Horsley <Damien.Horsley@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define IMG_I2S_IN_RX_FIFO			0

#define IMG_I2S_IN_CTL				0x4
#define IMG_I2S_IN_CTL_EXT_EN_MASK		BIT(25)
#define IMG_I2S_IN_CTL_DATA_EN_MASK		BIT(24)
#define IMG_I2S_IN_CTL_ACTIVE_CHAN_MASK		0xfffffffc
#define IMG_I2S_IN_CTL_ACTIVE_CH_SHIFT		2
#define IMG_I2S_IN_CTL_16PACK_MASK		BIT(1)
#define IMG_I2S_IN_CTL_ME_MASK			BIT(0)

#define IMG_I2S_IN_CH_CTL			0x4
#define IMG_I2S_IN_CH_CTL_CCDEL_MASK		0x38000
#define IMG_I2S_IN_CH_CTL_CCDEL_SHIFT		15
#define IMG_I2S_IN_CH_CTL_FEN_MASK		BIT(14)
#define IMG_I2S_IN_CH_CTL_FMODE_MASK		BIT(13)
#define IMG_I2S_IN_CH_CTL_16PACK_MASK		BIT(12)
#define IMG_I2S_IN_CH_CTL_JUST_MASK		BIT(10)
#define IMG_I2S_IN_CH_CTL_PACKH_MASK		BIT(9)
#define IMG_I2S_IN_CH_CTL_CLK_TRANS_MASK	BIT(8)
#define IMG_I2S_IN_CH_CTL_BLKP_MASK		BIT(7)
#define IMG_I2S_IN_CH_CTL_FIFO_FLUSH_MASK	BIT(6)
#define IMG_I2S_IN_CH_CTL_LSB_FIRST_MASK	BIT(5)
#define IMG_I2S_IN_CH_CTL_CET_MASK		BIT(4)
#define IMG_I2S_IN_CH_CTL_LRD_MASK		BIT(3)
#define IMG_I2S_IN_CH_CTL_FW_MASK		BIT(2)
#define IMG_I2S_IN_CH_CTL_SW_MASK		BIT(1)
#define IMG_I2S_IN_CH_CTL_ME_MASK		BIT(0)

struct img_i2s_in {
	spinlock_t lock;
	void __iomem *base;
	struct clk *clk_sys;
	struct snd_dmaengine_dai_dma_data dma_data;
	struct device *dev;
	unsigned int max_i2s_chan;
	void __iomem *channel_base;
	bool active;
	unsigned int active_channels;
};

static inline void img_i2s_in_writel(struct img_i2s_in *i2s, u32 val,
					u32 reg)
{
	writel(val, i2s->base + reg);
}

static inline u32 img_i2s_in_readl(struct img_i2s_in *i2s, u32 reg)
{
	return readl(i2s->base + reg);
}

static inline void img_i2s_in_ch_writel(struct img_i2s_in *i2s, u32 chan,
					u32 val, u32 reg)
{
	writel(val, i2s->channel_base + (chan * 0x20) + reg);
}

static inline u32 img_i2s_in_ch_readl(struct img_i2s_in *i2s, u32 chan,
					u32 reg)
{
	return readl(i2s->channel_base + (chan * 0x20) + reg);
}

static inline u32 img_i2s_in_ch_disable(struct img_i2s_in *i2s, u32 chan)
{
	u32 reg;

	reg = img_i2s_in_ch_readl(i2s, chan, IMG_I2S_IN_CH_CTL);
	reg &= ~IMG_I2S_IN_CH_CTL_ME_MASK;
	img_i2s_in_ch_writel(i2s, chan, reg, IMG_I2S_IN_CH_CTL);

	return reg;
}

static inline void img_i2s_in_ch_enable(struct img_i2s_in *i2s, u32 chan,
					u32 reg)
{
	reg |= IMG_I2S_IN_CH_CTL_ME_MASK;
	img_i2s_in_ch_writel(i2s, chan, reg, IMG_I2S_IN_CH_CTL);
}

static inline void img_i2s_in_flush(struct img_i2s_in *i2s)
{
	int i;
	u32 reg;

	for (i = 0; i < i2s->active_channels; i++) {
		reg = img_i2s_in_ch_disable(i2s, i);
		reg |= IMG_I2S_IN_CH_CTL_FIFO_FLUSH_MASK;
		img_i2s_in_ch_writel(i2s, i, reg, IMG_I2S_IN_CH_CTL);
		reg &= ~IMG_I2S_IN_CH_CTL_FIFO_FLUSH_MASK;
		img_i2s_in_ch_writel(i2s, i, reg, IMG_I2S_IN_CH_CTL);
		img_i2s_in_ch_enable(i2s, i, reg);
	}
}

static int img_i2s_in_check_rate(struct img_i2s_in *i2s,
		long max_sample_rate, unsigned int frame_size,
		unsigned int *bclk_filter_enable,
		unsigned int *bclk_filter_value)
{
	unsigned long temp_freq, cur_freq;

	temp_freq = max_sample_rate * frame_size;

	cur_freq = clk_get_rate(i2s->clk_sys);
	if (cur_freq >= temp_freq * 8) {
		*bclk_filter_enable = 1;
		*bclk_filter_value = 0;
	} else if (cur_freq >= temp_freq * 7) {
		*bclk_filter_enable = 1;
		*bclk_filter_value = 1;
	} else if (cur_freq >= temp_freq * 6) {
		*bclk_filter_enable = 0;
		*bclk_filter_value = 0;
	} else {
		return -EINVAL;
	}

	return 0;
}

static int img_i2s_in_trigger(struct snd_pcm_substream *substream, int cmd,
	struct snd_soc_dai *dai)
{
	struct img_i2s_in *i2s = snd_soc_dai_get_drvdata(dai);
	u32 reg;
	unsigned long flags;
	int ret = 0;

	dev_dbg(i2s->dev, "trigger cmd %d\n", cmd);

	reg = img_i2s_in_readl(i2s, IMG_I2S_IN_CTL);

	spin_lock_irqsave(&i2s->lock, flags);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		reg |= IMG_I2S_IN_CTL_ME_MASK;
		img_i2s_in_writel(i2s, reg, IMG_I2S_IN_CTL);
		i2s->active = true;
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		reg &= ~IMG_I2S_IN_CTL_ME_MASK;
		img_i2s_in_writel(i2s, reg, IMG_I2S_IN_CTL);

		if ((cmd == SNDRV_PCM_TRIGGER_STOP) ||
				((cmd == SNDRV_PCM_TRIGGER_SUSPEND) &&
				(!(substream->runtime->info &
				SNDRV_PCM_INFO_PAUSE))))
			img_i2s_in_flush(i2s);

		i2s->active = false;
		break;
	default:
		ret = -EINVAL;
	}
	spin_unlock_irqrestore(&i2s->lock, flags);

	return ret;
}

static int img_i2s_in_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct img_i2s_in *i2s = snd_soc_dai_get_drvdata(dai);
	unsigned int rate, channels, format, i2s_channels, frame_size;
	unsigned int bclk_filter_enable, bclk_filter_value;
	int i, ret = 0;
	u32 reg, control_reg, control_mask, chan_control_mask;
	u32 control_set = 0, chan_control_set = 0;
	unsigned long flags;

	rate = params_rate(params);
	format = params_format(params);
	channels = params_channels(params);
	i2s_channels = channels / 2;

	dev_dbg(i2s->dev, "hw_params rate %u channels %u format %u\n",
			rate, channels, format);

	switch (format) {
	case SNDRV_PCM_FORMAT_S24_LE:
		frame_size = 64;
		chan_control_set |= IMG_I2S_IN_CH_CTL_SW_MASK;
		chan_control_set |= IMG_I2S_IN_CH_CTL_FW_MASK;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		frame_size = 32;
		control_set |= IMG_I2S_IN_CTL_16PACK_MASK;
		chan_control_set |= IMG_I2S_IN_CH_CTL_16PACK_MASK;
		break;
	default:
		return -EINVAL;
	}

	if ((channels < 2) ||
			(channels > (i2s->max_i2s_chan * 2)) ||
			(channels % 2))
		return -EINVAL;

	control_set |= ((i2s_channels - 1) << IMG_I2S_IN_CTL_ACTIVE_CH_SHIFT);

	ret = img_i2s_in_check_rate(i2s, rate, frame_size,
			&bclk_filter_enable, &bclk_filter_value);
	if (ret < 0)
		return ret;

	if (bclk_filter_enable)
		chan_control_set |= IMG_I2S_IN_CH_CTL_FEN_MASK;

	if (bclk_filter_value)
		chan_control_set |= IMG_I2S_IN_CH_CTL_FMODE_MASK;

	control_mask = ~IMG_I2S_IN_CTL_16PACK_MASK &
			~IMG_I2S_IN_CTL_ACTIVE_CHAN_MASK;

	chan_control_mask = ~IMG_I2S_IN_CH_CTL_16PACK_MASK &
			~IMG_I2S_IN_CH_CTL_FEN_MASK &
			~IMG_I2S_IN_CH_CTL_FMODE_MASK &
			~IMG_I2S_IN_CH_CTL_SW_MASK &
			~IMG_I2S_IN_CH_CTL_FW_MASK;

	spin_lock_irqsave(&i2s->lock, flags);
	control_reg = img_i2s_in_readl(i2s, IMG_I2S_IN_CTL);
	control_reg = (control_reg & control_mask) | control_set;
	img_i2s_in_writel(i2s, control_reg, IMG_I2S_IN_CTL);

	for (i = 0; i < i2s_channels; i++) {
		reg = img_i2s_in_ch_disable(i2s, i);
		reg = (reg & chan_control_mask) | chan_control_set;
		img_i2s_in_ch_writel(i2s, i, reg, IMG_I2S_IN_CH_CTL);
		img_i2s_in_ch_enable(i2s, i, reg);
	}
	for (; i < i2s->max_i2s_chan; i++) {
		reg = img_i2s_in_ch_disable(i2s, i);
		reg = (reg & chan_control_mask) | chan_control_set;
		img_i2s_in_ch_writel(i2s, i, reg, IMG_I2S_IN_CH_CTL);
	}

	i2s->active_channels = i2s_channels;
	spin_unlock_irqrestore(&i2s->lock, flags);

	return 0;
}

static int img_i2s_in_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct img_i2s_in *i2s = snd_soc_dai_get_drvdata(dai);
	int i;
	unsigned long flags;
	u32 chan_control_mask, lrd_set = 0, blkp_set = 0, chan_control_set = 0;
	u32 reg;

	dev_dbg(i2s->dev, "set format %#x\n", fmt);

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		lrd_set |= IMG_I2S_IN_CH_CTL_LRD_MASK;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		lrd_set |= IMG_I2S_IN_CH_CTL_LRD_MASK;
		blkp_set |= IMG_I2S_IN_CH_CTL_BLKP_MASK;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		blkp_set |= IMG_I2S_IN_CH_CTL_BLKP_MASK;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		chan_control_set |= IMG_I2S_IN_CH_CTL_CLK_TRANS_MASK;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	default:
		return -EINVAL;
	}

	chan_control_mask = ~IMG_I2S_IN_CH_CTL_CLK_TRANS_MASK &
			~IMG_I2S_IN_CH_CTL_BLKP_MASK &
			~IMG_I2S_IN_CH_CTL_LRD_MASK;

	spin_lock_irqsave(&i2s->lock, flags);

	if (i2s->active) {
		spin_unlock_irqrestore(&i2s->lock, flags);
		return -EBUSY;
	}

	for (i = 0; i < i2s->active_channels; i++) {
		reg = img_i2s_in_ch_disable(i2s, i);
		reg = (reg & chan_control_mask) | chan_control_set;
		img_i2s_in_ch_writel(i2s, i, reg, IMG_I2S_IN_CH_CTL);
		reg |= IMG_I2S_IN_CH_CTL_ME_MASK;
		img_i2s_in_ch_writel(i2s, i, reg, IMG_I2S_IN_CH_CTL);
		reg = (reg & ~IMG_I2S_IN_CH_CTL_BLKP_MASK) | blkp_set;
		img_i2s_in_ch_writel(i2s, i, reg, IMG_I2S_IN_CH_CTL);
		reg = (reg & ~IMG_I2S_IN_CH_CTL_LRD_MASK) | lrd_set;
		img_i2s_in_ch_writel(i2s, i, reg, IMG_I2S_IN_CH_CTL);
	}
	for (; i < i2s->max_i2s_chan; i++) {
		reg = img_i2s_in_ch_readl(i2s, i, IMG_I2S_IN_CH_CTL);
		reg = (reg & chan_control_mask) | chan_control_set;
		img_i2s_in_ch_writel(i2s, i, reg, IMG_I2S_IN_CH_CTL);
		reg = (reg & ~IMG_I2S_IN_CH_CTL_BLKP_MASK) | blkp_set;
		img_i2s_in_ch_writel(i2s, i, reg, IMG_I2S_IN_CH_CTL);
		reg = (reg & ~IMG_I2S_IN_CH_CTL_LRD_MASK) | lrd_set;
		img_i2s_in_ch_writel(i2s, i, reg, IMG_I2S_IN_CH_CTL);
	}

	spin_unlock_irqrestore(&i2s->lock, flags);

	return 0;
}

static const struct snd_soc_dai_ops img_i2s_in_dai_ops = {
	.trigger = img_i2s_in_trigger,
	.hw_params = img_i2s_in_hw_params,
	.set_fmt = img_i2s_in_set_fmt
};

static int img_i2s_in_dai_probe(struct snd_soc_dai *dai)
{
	struct img_i2s_in *i2s = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, NULL, &i2s->dma_data);

	return 0;
}

static struct snd_soc_dai_driver img_i2s_in_dai = {
	.probe = img_i2s_in_dai_probe,
	.capture = {
		.channels_min = 2,
		.channels_max = 0, /* set during probe */
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S16_LE
	},
	.ops = &img_i2s_in_dai_ops
};

static const struct snd_soc_component_driver img_i2s_in_component = {
	.name = "img-i2s-in"
};

static int img_i2s_in_probe(struct platform_device *pdev)
{
	struct img_i2s_in *i2s;
	struct resource *res;
	void __iomem *base;
	int ret, i;
	struct reset_control *rst;
	u32 reg;
	unsigned int max_i2s_chan_pow_2;
	struct device *dev = &pdev->dev;

	i2s = devm_kzalloc(dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	platform_set_drvdata(pdev, i2s);

	i2s->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	i2s->base = base;

	if (of_property_read_u32(pdev->dev.of_node, "img,i2s-channels",
			&i2s->max_i2s_chan)) {
		dev_err(dev, "No img,i2s-channels property\n");
		return -EINVAL;
	}

	max_i2s_chan_pow_2 = 1 << get_count_order(i2s->max_i2s_chan);

	i2s->channel_base = base + (max_i2s_chan_pow_2 * 0x20);

	i2s->clk_sys = devm_clk_get(dev, "sys");
	if (IS_ERR(i2s->clk_sys))
		return PTR_ERR(i2s->clk_sys);

	ret = clk_prepare_enable(i2s->clk_sys);
	if (ret)
		return ret;

	i2s->dma_data.addr = res->start + IMG_I2S_IN_RX_FIFO;
	i2s->dma_data.addr_width = 4;
	i2s->dma_data.maxburst = 1;
	img_i2s_in_dai.capture.channels_max = i2s->max_i2s_chan * 2;

	rst = devm_reset_control_get(dev, "rst");
	if (IS_ERR(rst)) {
		dev_dbg(dev, "No top level reset found\n");

		reg = img_i2s_in_readl(i2s, IMG_I2S_IN_CTL);
		reg &= ~IMG_I2S_IN_CTL_ME_MASK;
		img_i2s_in_writel(i2s, reg, IMG_I2S_IN_CTL);

		for (i = 0; i < i2s->max_i2s_chan; i++) {
			reg = img_i2s_in_ch_disable(i2s, i);
			reg |= IMG_I2S_IN_CH_CTL_FIFO_FLUSH_MASK;
			img_i2s_in_ch_writel(i2s, i, reg, IMG_I2S_IN_CH_CTL);
			reg &= ~IMG_I2S_IN_CH_CTL_FIFO_FLUSH_MASK;
			img_i2s_in_ch_writel(i2s, i, reg, IMG_I2S_IN_CH_CTL);
		}
	} else {
		reset_control_assert(rst);
		reset_control_deassert(rst);
	}

	img_i2s_in_writel(i2s, 0, IMG_I2S_IN_CTL);

	for (i = 0; i < i2s->max_i2s_chan; i++)
		img_i2s_in_ch_writel(i2s, i,
			(4 << IMG_I2S_IN_CH_CTL_CCDEL_SHIFT) |
			IMG_I2S_IN_CH_CTL_JUST_MASK |
			IMG_I2S_IN_CH_CTL_FW_MASK, IMG_I2S_IN_CH_CTL);

	spin_lock_init(&i2s->lock);

	ret = devm_snd_soc_register_component(dev, &img_i2s_in_component,
					 &img_i2s_in_dai, 1);
	if (ret)
		goto err_clk_disable;

	ret = devm_snd_dmaengine_pcm_register(dev, NULL, 0);
	if (ret)
		goto err_clk_disable;

	dev_dbg(dev, "Probe successful\n");

	return 0;

err_clk_disable:
	clk_disable_unprepare(i2s->clk_sys);

	return ret;
}

static int img_i2s_in_dev_remove(struct platform_device *pdev)
{
	struct img_i2s_in *i2s = platform_get_drvdata(pdev);

	clk_disable_unprepare(i2s->clk_sys);

	return 0;
}

static const struct of_device_id img_i2s_in_of_match[] = {
	{ .compatible = "img,i2s-in" },
	{}
};
MODULE_DEVICE_TABLE(of, img_i2s_of_match);

static struct platform_driver img_i2s_in_driver = {
	.driver = {
		.name = "img-i2s-in",
		.of_match_table = img_i2s_in_of_match
	},
	.probe = img_i2s_in_probe,
	.remove = img_i2s_in_dev_remove
};
module_platform_driver(img_i2s_in_driver);

MODULE_AUTHOR("Damien Horsley <Damien.Horsley@imgtec.com>");
MODULE_DESCRIPTION("IMG I2S Input Driver");
MODULE_LICENSE("GPL v2");
