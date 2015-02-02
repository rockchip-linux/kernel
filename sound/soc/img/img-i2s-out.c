/*
 * IMG I2S out controller driver
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
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define IMG_I2S_OUT_TX_FIFO			0

#define IMG_I2S_OUT_CTL				0x4
#define IMG_I2S_OUT_CTL_DATA_EN_MASK		BIT(24)
#define IMG_I2S_OUT_CTL_ACTIVE_CHAN_MASK	0xffe000
#define IMG_I2S_OUT_CTL_ACTIVE_CHAN_SHIFT	13
#define IMG_I2S_OUT_CTL_FRM_SIZE_MASK		BIT(8)
#define IMG_I2S_OUT_CTL_MASTER_MASK		BIT(6)
#define IMG_I2S_OUT_CTL_CLK_MASK		BIT(5)
#define IMG_I2S_OUT_CTL_CLK_EN_MASK		BIT(4)
#define IMG_I2S_OUT_CTL_FRM_CLK_POL_MASK	BIT(3)
#define IMG_I2S_OUT_CTL_BCLK_POL_MASK		BIT(2)
#define IMG_I2S_OUT_CTL_ME_MASK			BIT(0)

#define IMG_I2S_OUT_SOFT_RESET			0x8
#define IMG_I2S_OUT_SOFT_RESET_MASK		BIT(0)

#define IMG_I2S_OUT_CH_CTL			0x4
#define IMG_I2S_OUT_CHAN_CTL_CH_MASK		BIT(11)
#define IMG_I2S_OUT_CHAN_CTL_LT_MASK		BIT(10)
#define IMG_I2S_OUT_CHAN_CTL_FMT_MASK		0xf0
#define IMG_I2S_OUT_CHAN_CTL_FMT_SHIFT		4
#define IMG_I2S_OUT_CHAN_CTL_JUST_MASK		BIT(3)
#define IMG_I2S_OUT_CHAN_CTL_FIFO_FLUSH_MASK	BIT(2)
#define IMG_I2S_OUT_CHAN_CTL_CLKT_MASK		BIT(1)
#define IMG_I2S_OUT_CHAN_CTL_ME_MASK		BIT(0)

struct img_i2s_out {
	spinlock_t lock;
	void __iomem *base;
	struct clk *clk_sys;
	struct clk *clk_ref;
	struct snd_dmaengine_dai_dma_data dma_data;
	struct device *dev;
	unsigned int max_i2s_chan;
	void __iomem *channel_base;
	bool force_clk_active;
	unsigned int active_channels;
	bool active;
};

static int img_i2s_out_suspend(struct device *dev)
{
	struct img_i2s_out *i2s = dev_get_drvdata(dev);

	clk_disable_unprepare(i2s->clk_ref);

	return 0;
}

static int img_i2s_out_resume(struct device *dev)
{
	struct img_i2s_out *i2s = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(i2s->clk_ref);
	if (ret) {
		dev_err(dev, "clk_enable failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static inline void img_i2s_out_writel(struct img_i2s_out *i2s, u32 val,
					u32 reg)
{
	writel(val, i2s->base + reg);
}

static inline u32 img_i2s_out_readl(struct img_i2s_out *i2s, u32 reg)
{
	return readl(i2s->base + reg);
}

static inline void img_i2s_out_ch_writel(struct img_i2s_out *i2s,
					u32 chan, u32 val, u32 reg)
{
	writel(val, i2s->channel_base + (chan * 0x20) + reg);
}

static inline u32 img_i2s_out_ch_readl(struct img_i2s_out *i2s, u32 chan,
					u32 reg)
{
	return readl(i2s->channel_base + (chan * 0x20) + reg);
}

static inline u32 img_i2s_out_ch_disable(struct img_i2s_out *i2s, u32 chan)
{
	u32 reg;

	reg = img_i2s_out_ch_readl(i2s, chan, IMG_I2S_OUT_CH_CTL);
	reg &= ~IMG_I2S_OUT_CHAN_CTL_ME_MASK;
	img_i2s_out_ch_writel(i2s, chan, reg, IMG_I2S_OUT_CH_CTL);

	return reg;
}

static inline void img_i2s_out_ch_enable(struct img_i2s_out *i2s, u32 chan,
					u32 reg)
{
	reg |= IMG_I2S_OUT_CHAN_CTL_ME_MASK;
	img_i2s_out_ch_writel(i2s, chan, reg, IMG_I2S_OUT_CH_CTL);
}

static inline u32 img_i2s_out_disable(struct img_i2s_out *i2s)
{
	u32 reg;

	reg = img_i2s_out_readl(i2s, IMG_I2S_OUT_CTL);
	reg &= ~IMG_I2S_OUT_CTL_ME_MASK;
	img_i2s_out_writel(i2s, reg, IMG_I2S_OUT_CTL);

	return reg;
}

static inline void img_i2s_out_enable(struct img_i2s_out *i2s, u32 reg)
{
	reg |= IMG_I2S_OUT_CTL_ME_MASK;
	img_i2s_out_writel(i2s, reg, IMG_I2S_OUT_CTL);
}

static inline void img_i2s_out_flush(struct img_i2s_out *i2s)
{
	int i;
	u32 reg;

	for (i = 0; i < i2s->active_channels; i++) {
		reg = img_i2s_out_ch_disable(i2s, i);
		reg |= IMG_I2S_OUT_CHAN_CTL_FIFO_FLUSH_MASK;
		img_i2s_out_ch_writel(i2s, i, reg, IMG_I2S_OUT_CH_CTL);
		reg &= ~IMG_I2S_OUT_CHAN_CTL_FIFO_FLUSH_MASK;
		img_i2s_out_ch_writel(i2s, i, reg, IMG_I2S_OUT_CH_CTL);
		img_i2s_out_ch_enable(i2s, i, reg);
	}
}

static int img_i2s_out_trigger(struct snd_pcm_substream *substream, int cmd,
	struct snd_soc_dai *dai)
{
	struct img_i2s_out *i2s = snd_soc_dai_get_drvdata(dai);
	u32 reg;
	unsigned long flags;
	int ret = 0;

	dev_dbg(i2s->dev, "trigger cmd %d\n", cmd);

	spin_lock_irqsave(&i2s->lock, flags);

	reg = img_i2s_out_readl(i2s, IMG_I2S_OUT_CTL);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (!i2s->force_clk_active)
			reg |= IMG_I2S_OUT_CTL_CLK_EN_MASK;
		reg |= IMG_I2S_OUT_CTL_DATA_EN_MASK;
		img_i2s_out_writel(i2s, reg, IMG_I2S_OUT_CTL);
		i2s->active = true;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (!i2s->force_clk_active)
			reg &= ~IMG_I2S_OUT_CTL_CLK_EN_MASK;
		reg &= ~IMG_I2S_OUT_CTL_DATA_EN_MASK;
		img_i2s_out_writel(i2s, reg, IMG_I2S_OUT_CTL);
		if ((cmd == SNDRV_PCM_TRIGGER_STOP) ||
			((cmd == SNDRV_PCM_TRIGGER_SUSPEND) &&
			(!(substream->runtime->info & SNDRV_PCM_INFO_PAUSE))))
				img_i2s_out_flush(i2s);
		i2s->active = false;
		break;
	default:
		ret = -EINVAL;
	}

	spin_unlock_irqrestore(&i2s->lock, flags);

	return ret;
}

static int img_i2s_out_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct img_i2s_out *i2s = snd_soc_dai_get_drvdata(dai);
	unsigned int channels, i2s_channels, format;
	long pre_div_a, pre_div_b, diff_a, diff_b, rate, clk_rate;
	unsigned long flags;
	int i;
	u32 reg, control_reg, control_mask, control_set = 0;

	rate = params_rate(params);
	format = params_format(params);
	channels = params_channels(params);
	i2s_channels = channels / 2;

	dev_dbg(i2s->dev, "hw_params rate %ld channels %u format %u\n",
			rate, channels, format);

	if (format != SNDRV_PCM_FORMAT_S32_LE)
		return -EINVAL;

	if ((channels < 2) ||
			(channels > (i2s->max_i2s_chan * 2)) ||
			(channels % 2))
		return -EINVAL;

	pre_div_a = clk_round_rate(i2s->clk_ref, rate * 256);
	if (pre_div_a < 0)
		return pre_div_a;
	pre_div_b = clk_round_rate(i2s->clk_ref, rate * 384);
	if (pre_div_b < 0)
		return pre_div_a;

	diff_a = abs((pre_div_a / 256) - rate);
	diff_b = abs((pre_div_b / 384) - rate);

	/* Use <= to favour lower clock rates */
	if (diff_a <= diff_b)
		clk_set_rate(i2s->clk_ref, pre_div_a);
	else
		clk_set_rate(i2s->clk_ref, pre_div_b);

	/*
	 * Another driver (eg machine driver) may have rejected the above
	 * change. Get the current rate and set the register bit according to
	 * the new min diff
	 */
	clk_rate = clk_get_rate(i2s->clk_ref);

	diff_a = abs((clk_rate / 256) - rate);
	diff_b = abs((clk_rate / 384) - rate);

	if (diff_a > diff_b)
		control_set |= IMG_I2S_OUT_CTL_CLK_MASK;

	control_set |= (((i2s_channels - 1) <<
			IMG_I2S_OUT_CTL_ACTIVE_CHAN_SHIFT) &
			IMG_I2S_OUT_CTL_ACTIVE_CHAN_MASK);

	control_mask = (~IMG_I2S_OUT_CTL_CLK_MASK &
			~IMG_I2S_OUT_CTL_ACTIVE_CHAN_MASK);

	spin_lock_irqsave(&i2s->lock, flags);

	control_reg = img_i2s_out_disable(i2s);
	control_reg = (control_reg & control_mask) | control_set;
	img_i2s_out_writel(i2s, control_reg, IMG_I2S_OUT_CTL);

	for (i = 0; i < i2s_channels; i++) {
		reg = img_i2s_out_ch_readl(i2s, i, IMG_I2S_OUT_CH_CTL);
		img_i2s_out_ch_enable(i2s, i, reg);
	}
	for (; i < i2s->max_i2s_chan; i++)
		reg = img_i2s_out_ch_disable(i2s, i);

	img_i2s_out_enable(i2s, control_reg);
	i2s->active_channels = i2s_channels;

	spin_unlock_irqrestore(&i2s->lock, flags);

	return 0;
}

static int img_i2s_out_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct img_i2s_out *i2s = snd_soc_dai_get_drvdata(dai);
	u32 reg;
	int i, ret = 0;
	bool force_clk_active;
	unsigned long flags;
	u32 chan_control_mask, control_mask, chan_control_set = 0;
	u32 control_reg, control_set = 0;

	dev_dbg(i2s->dev, "set format %#x\n", fmt);

	force_clk_active = ((fmt & SND_SOC_DAIFMT_CLOCK_MASK) ==
			SND_SOC_DAIFMT_CONT);

	if (force_clk_active)
		control_set |= IMG_I2S_OUT_CTL_CLK_EN_MASK;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		control_set |= IMG_I2S_OUT_CTL_MASTER_MASK;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		control_set |= IMG_I2S_OUT_CTL_BCLK_POL_MASK;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		control_set |= IMG_I2S_OUT_CTL_BCLK_POL_MASK;
		control_set |= IMG_I2S_OUT_CTL_FRM_CLK_POL_MASK;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		control_set |= IMG_I2S_OUT_CTL_FRM_CLK_POL_MASK;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		chan_control_set |= IMG_I2S_OUT_CHAN_CTL_CLKT_MASK;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		break;
	default:
		return -EINVAL;
	}

	control_mask = (~IMG_I2S_OUT_CTL_CLK_EN_MASK &
		~IMG_I2S_OUT_CTL_MASTER_MASK &
		~IMG_I2S_OUT_CTL_BCLK_POL_MASK &
		~IMG_I2S_OUT_CTL_FRM_CLK_POL_MASK);

	chan_control_mask = ~IMG_I2S_OUT_CHAN_CTL_CLKT_MASK;

	spin_lock_irqsave(&i2s->lock, flags);

	if (i2s->active) {
		spin_unlock_irqrestore(&i2s->lock, flags);
		return -EBUSY;
	}

	control_reg = img_i2s_out_disable(i2s);
	control_reg = (control_reg & control_mask) | control_set;
	img_i2s_out_writel(i2s, control_reg, IMG_I2S_OUT_CTL);

	for (i = 0; i < i2s->active_channels; i++) {
		reg = img_i2s_out_ch_disable(i2s, i);
		reg = (reg & chan_control_mask) | chan_control_set;
		img_i2s_out_ch_writel(i2s, i, reg, IMG_I2S_OUT_CH_CTL);
		img_i2s_out_ch_enable(i2s, i, reg);
	}
	for (; i < i2s->max_i2s_chan; i++) {
		reg = img_i2s_out_ch_readl(i2s, i, IMG_I2S_OUT_CH_CTL);
		reg = (reg & chan_control_mask) | chan_control_set;
		img_i2s_out_ch_writel(i2s, i, reg, IMG_I2S_OUT_CH_CTL);
	}

	img_i2s_out_enable(i2s, control_reg);

	i2s->force_clk_active = force_clk_active;

	spin_unlock_irqrestore(&i2s->lock, flags);

	return ret;
}

static const struct snd_soc_dai_ops img_i2s_out_dai_ops = {
	.trigger = img_i2s_out_trigger,
	.hw_params = img_i2s_out_hw_params,
	.set_fmt = img_i2s_out_set_fmt
};

static int img_i2s_out_dai_probe(struct snd_soc_dai *dai)
{
	struct img_i2s_out *i2s = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &i2s->dma_data, NULL);

	return 0;
}

static struct snd_soc_dai_driver img_i2s_out_dai = {
	.probe = img_i2s_out_dai_probe,
	.playback = {
		.channels_min = 2,
		.channels_max = 0, /* set during probe */
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE
	},
	.ops = &img_i2s_out_dai_ops
};

static const struct snd_soc_component_driver img_i2s_out_component = {
	.name = "img-i2s-out"
};

static int img_i2s_out_probe(struct platform_device *pdev)
{
	struct img_i2s_out *i2s;
	struct resource *res;
	void __iomem *base;
	int ret;
	struct reset_control *rst;
	unsigned int i, max_i2s_chan_pow_2;

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	platform_set_drvdata(pdev, i2s);

	i2s->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	i2s->base = base;

	if (of_property_read_u32(pdev->dev.of_node, "img,i2s-channels",
			&i2s->max_i2s_chan)) {
		dev_err(&pdev->dev, "No img,i2s-channels property\n");
		return -EINVAL;
	}

	max_i2s_chan_pow_2 = 1 << get_count_order(i2s->max_i2s_chan);

	i2s->channel_base = base + (max_i2s_chan_pow_2 * 0x20);

	i2s->clk_sys = devm_clk_get(&pdev->dev, "sys");
	if (IS_ERR(i2s->clk_sys))
		return PTR_ERR(i2s->clk_sys);

	i2s->clk_ref = devm_clk_get(&pdev->dev, "ref");
	if (IS_ERR(i2s->clk_ref))
		return PTR_ERR(i2s->clk_ref);

	ret = clk_prepare_enable(i2s->clk_sys);
	if (ret)
		return ret;

	rst = devm_reset_control_get(&pdev->dev, "rst");
	if (IS_ERR(rst)) {
		dev_dbg(&pdev->dev, "No top level reset found\n");
		img_i2s_out_writel(i2s, IMG_I2S_OUT_SOFT_RESET_MASK,
				IMG_I2S_OUT_SOFT_RESET);
		img_i2s_out_writel(i2s, 0, IMG_I2S_OUT_SOFT_RESET);
	} else {
		reset_control_assert(rst);
		reset_control_deassert(rst);
	}

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = img_i2s_out_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	spin_lock_init(&i2s->lock);

	i2s->dma_data.addr = res->start + IMG_I2S_OUT_TX_FIFO;
	i2s->dma_data.addr_width = 4;
	i2s->dma_data.maxburst = 4;
	img_i2s_out_dai.playback.channels_max = i2s->max_i2s_chan * 2;

	for (i = 0; i < i2s->max_i2s_chan; i++)
		img_i2s_out_ch_writel(i2s, i, IMG_I2S_OUT_CHAN_CTL_JUST_MASK |
				IMG_I2S_OUT_CHAN_CTL_LT_MASK |
				IMG_I2S_OUT_CHAN_CTL_CH_MASK |
				(8 << IMG_I2S_OUT_CHAN_CTL_FMT_SHIFT),
				IMG_I2S_OUT_CH_CTL);

	img_i2s_out_writel(i2s, IMG_I2S_OUT_CTL_FRM_SIZE_MASK,
			IMG_I2S_OUT_CTL);

	ret = devm_snd_soc_register_component(&pdev->dev,
			&img_i2s_out_component, &img_i2s_out_dai, 1);
	if (ret)
		goto err_suspend;

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
	if (ret)
		goto err_suspend;

	dev_dbg(&pdev->dev, "Probe successful\n");

	return 0;

err_suspend:
	if (!pm_runtime_status_suspended(&pdev->dev))
		img_i2s_out_suspend(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
	clk_disable_unprepare(i2s->clk_sys);

	return ret;
}

static int img_i2s_out_dev_remove(struct platform_device *pdev)
{
	struct img_i2s_out *i2s = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		img_i2s_out_suspend(&pdev->dev);

	clk_disable_unprepare(i2s->clk_sys);

	return 0;
}

static const struct of_device_id img_i2s_out_of_match[] = {
	{ .compatible = "img,i2s-out" },
	{}
};
MODULE_DEVICE_TABLE(of, img_i2s_of_match);

static const struct dev_pm_ops img_i2s_out_pm_ops = {
	SET_RUNTIME_PM_OPS(img_i2s_out_suspend,
			   img_i2s_out_resume, NULL)
};

static struct platform_driver img_i2s_out_driver = {
	.driver = {
		.name = "img-i2s-out",
		.of_match_table = img_i2s_out_of_match,
		.pm = &img_i2s_out_pm_ops
	},
	.probe = img_i2s_out_probe,
	.remove = img_i2s_out_dev_remove
};
module_platform_driver(img_i2s_out_driver);

MODULE_AUTHOR("Damien Horsley <Damien.Horsley@imgtec.com>");
MODULE_DESCRIPTION("IMG I2S Output Driver");
MODULE_LICENSE("GPL v2");
