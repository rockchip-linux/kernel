/*
 * IMG SPDIF out controller driver
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
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define IMG_SPDIF_OUT_TX_FIFO		0

#define IMG_SPDIF_OUT_CTL		0x4
#define IMG_SPDIF_OUT_CTL_FS_MASK	BIT(4)
#define IMG_SPDIF_OUT_CTL_CLK_MASK	BIT(2)
#define IMG_SPDIF_OUT_CTL_SRT_MASK	BIT(0)

#define IMG_SPDIF_OUT_CSL		0x14

#define IMG_SPDIF_OUT_CSH_UV		0x18
#define IMG_SPDIF_OUT_CSH_UV_CSH_SHIFT	0
#define IMG_SPDIF_OUT_CSH_UV_CSH_MASK	0xff

#define IMG_SPDIF_OUT_SOFT_RESET	0x1c
#define IMG_SPDIF_OUT_SOFT_RESET_MASK	BIT(0)

struct img_spdif_out {
	void __iomem *base;
	struct clk *clk_sys;
	struct clk *clk_ref;
	struct snd_dmaengine_dai_dma_data dma_data;
	struct device *dev;
};

static int img_spdif_out_suspend(struct device *dev)
{
	struct img_spdif_out *spdif = dev_get_drvdata(dev);

	clk_disable_unprepare(spdif->clk_ref);

	return 0;
}

static int img_spdif_out_resume(struct device *dev)
{
	struct img_spdif_out *spdif = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(spdif->clk_ref);
	if (ret) {
		dev_err(dev, "clk_enable failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static inline void img_spdif_out_writel(struct img_spdif_out *spdif, u32 val,
				u32 reg)
{
	writel(val, spdif->base + reg);
}

static inline u32 img_spdif_out_readl(struct img_spdif_out *spdif, u32 reg)
{
	return readl(spdif->base + reg);
}

static int img_spdif_out_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;

	return 0;
}

static int img_spdif_out_get_status_mask(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.iec958.status[0] = 0xff;
	ucontrol->value.iec958.status[1] = 0xff;
	ucontrol->value.iec958.status[2] = 0xff;
	ucontrol->value.iec958.status[3] = 0xff;
	ucontrol->value.iec958.status[4] = 0xff;

	return 0;
}

static int img_spdif_out_get_status(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct img_spdif_out *spdif = snd_soc_dai_get_drvdata(cpu_dai);
	u32 reg;

	reg = img_spdif_out_readl(spdif, IMG_SPDIF_OUT_CSL);
	ucontrol->value.iec958.status[0] = reg & 0xff;
	ucontrol->value.iec958.status[1] = (reg >> 8) & 0xff;
	ucontrol->value.iec958.status[2] = (reg >> 16) & 0xff;
	ucontrol->value.iec958.status[3] = (reg >> 24) & 0xff;
	reg = img_spdif_out_readl(spdif, IMG_SPDIF_OUT_CSH_UV);
	ucontrol->value.iec958.status[4] =
		(reg & IMG_SPDIF_OUT_CSH_UV_CSH_MASK) >>
		IMG_SPDIF_OUT_CSH_UV_CSH_SHIFT;

	return 0;
}

static int img_spdif_out_set_status(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct img_spdif_out *spdif = snd_soc_dai_get_drvdata(cpu_dai);
	u32 reg;

	reg = ((u32)ucontrol->value.iec958.status[3] << 24);
	reg |= ((u32)ucontrol->value.iec958.status[2] << 16);
	reg |= ((u32)ucontrol->value.iec958.status[1] << 8);
	reg |= (u32)ucontrol->value.iec958.status[0];

	img_spdif_out_writel(spdif, IMG_SPDIF_OUT_CSL, reg);

	reg = img_spdif_out_readl(spdif, IMG_SPDIF_OUT_CSH_UV);
	reg &= ~IMG_SPDIF_OUT_CSH_UV_CSH_MASK;
	reg |= (u32)ucontrol->value.iec958.status[4] <<
			IMG_SPDIF_OUT_CSH_UV_CSH_SHIFT;
	img_spdif_out_writel(spdif, reg, IMG_SPDIF_OUT_CSH_UV);

	return 0;
}

static struct snd_kcontrol_new img_spdif_out_controls[] = {
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = SNDRV_CTL_NAME_IEC958("", PLAYBACK, MASK),
		.info = img_spdif_out_info,
		.get = img_spdif_out_get_status_mask
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = SNDRV_CTL_NAME_IEC958("", PLAYBACK, DEFAULT),
		.info = img_spdif_out_info,
		.get = img_spdif_out_get_status,
		.put = img_spdif_out_set_status
	}
};

static int img_spdif_out_trigger(struct snd_pcm_substream *substream, int cmd,
			struct snd_soc_dai *dai)
{
	struct img_spdif_out *spdif = snd_soc_dai_get_drvdata(dai);
	int ret = 0;
	u32 reg;

	dev_dbg(spdif->dev, "trigger cmd %d\n", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		reg = img_spdif_out_readl(spdif, IMG_SPDIF_OUT_CTL);
		reg |= IMG_SPDIF_OUT_CTL_SRT_MASK;
		img_spdif_out_writel(spdif, reg, IMG_SPDIF_OUT_CTL);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		reg = img_spdif_out_readl(spdif, IMG_SPDIF_OUT_CTL);
		reg &= ~IMG_SPDIF_OUT_CTL_SRT_MASK;
		img_spdif_out_writel(spdif, reg, IMG_SPDIF_OUT_CTL);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int img_spdif_out_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct img_spdif_out *spdif = snd_soc_dai_get_drvdata(dai);
	unsigned int format, channels;
	long pre_div_a, pre_div_b, diff_a, diff_b, rate, clk_rate;
	u32 reg;

	rate = params_rate(params);
	format = params_format(params);
	channels = params_channels(params);

	dev_dbg(spdif->dev, "hw_params rate %ld channels %u format %u\n",
			rate, channels, format);

	if (format != SNDRV_PCM_FORMAT_S32_LE)
		return -EINVAL;

	if (channels != 2)
		return -EINVAL;

	pre_div_a = clk_round_rate(spdif->clk_ref, rate * 256);
	if (pre_div_a < 0)
		return pre_div_a;
	pre_div_b = clk_round_rate(spdif->clk_ref, rate * 384);
	if (pre_div_b < 0)
		return pre_div_a;

	diff_a = abs((pre_div_a / 256) - rate);
	diff_b = abs((pre_div_b / 384) - rate);

	/* Use <= to favour lower clock rates */
	if (diff_a <= diff_b)
		clk_set_rate(spdif->clk_ref, pre_div_a);
	else
		clk_set_rate(spdif->clk_ref, pre_div_b);

	/*
	 * Another driver (eg machine driver) may have rejected the above
	 * change. Get the current rate and set the register bit according to
	 * the new min diff
	 */
	clk_rate = clk_get_rate(spdif->clk_ref);

	diff_a = abs((clk_rate / 256) - rate);
	diff_b = abs((clk_rate / 384) - rate);

	reg = img_spdif_out_readl(spdif, IMG_SPDIF_OUT_CTL);
	if (diff_a <= diff_b)
		reg &= ~IMG_SPDIF_OUT_CTL_CLK_MASK;
	else
		reg |= IMG_SPDIF_OUT_CTL_CLK_MASK;
	img_spdif_out_writel(spdif, reg, IMG_SPDIF_OUT_CTL);

	return 0;
}

static const struct snd_soc_dai_ops img_spdif_out_dai_ops = {
	.trigger = img_spdif_out_trigger,
	.hw_params = img_spdif_out_hw_params
};

static int img_spdif_out_dai_probe(struct snd_soc_dai *dai)
{
	struct img_spdif_out *spdif = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &spdif->dma_data, NULL);

	snd_soc_add_dai_controls(dai, img_spdif_out_controls,
			ARRAY_SIZE(img_spdif_out_controls));

	return 0;
}

static struct snd_soc_dai_driver img_spdif_out_dai = {
	.probe = img_spdif_out_dai_probe,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE
	},
	.ops = &img_spdif_out_dai_ops
};

static const struct snd_soc_component_driver img_spdif_out_component = {
	.name = "img-spdif-out"
};

static int img_spdif_out_probe(struct platform_device *pdev)
{
	struct img_spdif_out *spdif;
	struct resource *res;
	void __iomem *base;
	int ret;
	struct reset_control *rst;

	spdif = devm_kzalloc(&pdev->dev, sizeof(*spdif), GFP_KERNEL);
	if (!spdif)
		return -ENOMEM;

	platform_set_drvdata(pdev, spdif);

	spdif->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	spdif->base = base;

	spdif->clk_sys = devm_clk_get(&pdev->dev, "sys");
	if (IS_ERR(spdif->clk_sys))
		return PTR_ERR(spdif->clk_sys);

	spdif->clk_ref = devm_clk_get(&pdev->dev, "ref");
	if (IS_ERR(spdif->clk_ref))
		return PTR_ERR(spdif->clk_ref);

	ret = clk_prepare_enable(spdif->clk_sys);
	if (ret)
		return ret;

	rst = devm_reset_control_get(&pdev->dev, "rst");
	if (IS_ERR(rst)) {
		dev_dbg(&pdev->dev, "No top level reset found\n");
		img_spdif_out_writel(spdif, IMG_SPDIF_OUT_SOFT_RESET_MASK,
				IMG_SPDIF_OUT_SOFT_RESET);
		img_spdif_out_writel(spdif, 0, IMG_SPDIF_OUT_SOFT_RESET);
	} else {
		reset_control_assert(rst);
		reset_control_deassert(rst);
	}

	img_spdif_out_writel(spdif, IMG_SPDIF_OUT_CTL_FS_MASK,
			IMG_SPDIF_OUT_CTL);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = img_spdif_out_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	spdif->dma_data.addr = res->start + IMG_SPDIF_OUT_TX_FIFO;
	spdif->dma_data.addr_width = 4;
	spdif->dma_data.maxburst = 4;

	ret = devm_snd_soc_register_component(&pdev->dev,
			&img_spdif_out_component,
			&img_spdif_out_dai, 1);
	if (ret)
		goto err_suspend;

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
	if (ret)
		goto err_suspend;

	dev_dbg(&pdev->dev, "Probe successful\n");

	return 0;

err_suspend:
	if (!pm_runtime_status_suspended(&pdev->dev))
		img_spdif_out_suspend(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
	clk_disable_unprepare(spdif->clk_sys);

	return ret;
}

static int img_spdif_out_dev_remove(struct platform_device *pdev)
{
	struct img_spdif_out *spdif = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		img_spdif_out_suspend(&pdev->dev);

	clk_disable_unprepare(spdif->clk_sys);

	return 0;
}

static const struct of_device_id img_spdif_out_of_match[] = {
	{ .compatible = "img,spdif-out" },
	{}
};
MODULE_DEVICE_TABLE(of, img_spdif_of_match);

static const struct dev_pm_ops img_spdif_out_pm_ops = {
	SET_RUNTIME_PM_OPS(img_spdif_out_suspend,
			   img_spdif_out_resume, NULL)
};

static struct platform_driver img_spdif_out_driver = {
	.driver = {
		.name = "img-spdif-out",
		.of_match_table = img_spdif_out_of_match,
		.pm = &img_spdif_out_pm_ops
	},
	.probe = img_spdif_out_probe,
	.remove = img_spdif_out_dev_remove
};
module_platform_driver(img_spdif_out_driver);

MODULE_AUTHOR("Damien Horsley <Damien.Horsley@imgtec.com>");
MODULE_DESCRIPTION("IMG SPDIF Output driver");
MODULE_LICENSE("GPL v2");
