/*
 * IMG parallel out controller driver
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

#define IMG_PRL_OUT_TX_FIFO		0

#define IMG_PRL_OUT_CTL			0x4
#define IMG_PRL_OUT_CTL_CH_MASK		BIT(4)
#define IMG_PRL_OUT_CTL_PACKH_MASK	BIT(3)
#define IMG_PRL_OUT_CTL_EDGE_MASK	BIT(2)
#define IMG_PRL_OUT_CTL_ME_MASK		BIT(1)
#define IMG_PRL_OUT_CTL_SRST_MASK	BIT(0)

struct img_prl_out {
	spinlock_t lock;
	void __iomem *base;
	struct clk *clk_sys;
	struct clk *clk_ref;
	struct snd_dmaengine_dai_dma_data dma_data;
	struct device *dev;
	bool active;
};

static int img_prl_out_suspend(struct device *dev)
{
	struct img_prl_out *prl = dev_get_drvdata(dev);

	clk_disable_unprepare(prl->clk_ref);

	return 0;
}

static int img_prl_out_resume(struct device *dev)
{
	struct img_prl_out *prl = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(prl->clk_ref);
	if (ret) {
		dev_err(dev, "clk_enable failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static inline void img_prl_out_writel(struct img_prl_out *prl,
				u32 val, u32 reg)
{
	writel(val, prl->base + reg);
}

static inline u32 img_prl_out_readl(struct img_prl_out *prl, u32 reg)
{
	return readl(prl->base + reg);
}

static int img_prl_out_get_edge(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct img_prl_out *prl = snd_soc_dai_get_drvdata(cpu_dai);
	u32 reg;

	reg = img_prl_out_readl(prl, IMG_PRL_OUT_CTL);
	ucontrol->value.integer.value[0] = !!(reg & IMG_PRL_OUT_CTL_EDGE_MASK);

	return 0;
}

static int img_prl_out_set_edge(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct img_prl_out *prl = snd_soc_dai_get_drvdata(cpu_dai);
	unsigned long flags;
	int ret = 0;
	u32 reg;

	spin_lock_irqsave(&prl->lock, flags);
	if (prl->active) {
		ret = -EBUSY;
	} else {
		reg = img_prl_out_readl(prl, IMG_PRL_OUT_CTL);
		if (ucontrol->value.integer.value[0])
			reg |= IMG_PRL_OUT_CTL_EDGE_MASK;
		else
			reg &= ~IMG_PRL_OUT_CTL_EDGE_MASK;
		img_prl_out_writel(prl, reg, IMG_PRL_OUT_CTL);
	}
	spin_unlock_irqrestore(&prl->lock, flags);

	return ret;
}

static int img_prl_out_get_ch_swap(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct img_prl_out *prl = snd_soc_dai_get_drvdata(cpu_dai);
	u32 reg;

	reg = img_prl_out_readl(prl, IMG_PRL_OUT_CTL);
	ucontrol->value.integer.value[0] = !!(reg & IMG_PRL_OUT_CTL_CH_MASK);

	return 0;
}

static int img_prl_out_set_ch_swap(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *cpu_dai = snd_kcontrol_chip(kcontrol);
	struct img_prl_out *prl = snd_soc_dai_get_drvdata(cpu_dai);
	unsigned long flags;
	int ret = 0;
	u32 reg;

	spin_lock_irqsave(&prl->lock, flags);
	if (prl->active) {
		ret = -EBUSY;
	} else {
		reg = img_prl_out_readl(prl, IMG_PRL_OUT_CTL);
		if (ucontrol->value.integer.value[0])
			reg |= IMG_PRL_OUT_CTL_CH_MASK;
		else
			reg &= ~IMG_PRL_OUT_CTL_CH_MASK;
		img_prl_out_writel(prl, reg, IMG_PRL_OUT_CTL);
	}
	spin_unlock_irqrestore(&prl->lock, flags);

	return ret;
}

static struct snd_kcontrol_new img_prl_out_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = "Parallel Out Edge Falling",
		.info = snd_ctl_boolean_mono_info,
		.get = img_prl_out_get_edge,
		.put = img_prl_out_set_edge
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = "Parallel Out Channel Swap",
		.info = snd_ctl_boolean_mono_info,
		.get = img_prl_out_get_ch_swap,
		.put = img_prl_out_set_ch_swap
	}
};

static int img_prl_out_trigger(struct snd_pcm_substream *substream, int cmd,
			struct snd_soc_dai *dai)
{
	struct img_prl_out *prl = snd_soc_dai_get_drvdata(dai);
	unsigned long flags;
	int ret = 0;
	u32 reg;

	dev_dbg(prl->dev, "trigger cmd %d\n", cmd);

	spin_lock_irqsave(&prl->lock, flags);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		reg = img_prl_out_readl(prl, IMG_PRL_OUT_CTL);
		reg |= IMG_PRL_OUT_CTL_ME_MASK;
		img_prl_out_writel(prl, reg, IMG_PRL_OUT_CTL);
		prl->active = true;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		reg = img_prl_out_readl(prl, IMG_PRL_OUT_CTL);
		reg &= ~IMG_PRL_OUT_CTL_ME_MASK;
		img_prl_out_writel(prl, reg, IMG_PRL_OUT_CTL);
		prl->active = false;
		break;
	default:
		ret = -EINVAL;
	}
	spin_unlock_irqrestore(&prl->lock, flags);

	return ret;
}

static int img_prl_out_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct img_prl_out *prl = snd_soc_dai_get_drvdata(dai);
	unsigned int rate, format, channels;
	u32 reg, reg_set = 0;
	unsigned long flags;

	rate = params_rate(params);
	format = params_format(params);
	channels = params_channels(params);

	dev_dbg(prl->dev, "hw_params rate %u channels %u format %u\n",
			rate, channels, format);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S32_LE:
		reg_set |= IMG_PRL_OUT_CTL_PACKH_MASK;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		break;
	default:
		return -EINVAL;
	}

	if (params_channels(params) != 2)
		return -EINVAL;

	clk_set_rate(prl->clk_ref, rate * 256);

	spin_lock_irqsave(&prl->lock, flags);
	reg = img_prl_out_readl(prl, IMG_PRL_OUT_CTL);
	reg = (reg & ~IMG_PRL_OUT_CTL_PACKH_MASK) | reg_set;
	img_prl_out_writel(prl, reg, IMG_PRL_OUT_CTL);
	spin_unlock_irqrestore(&prl->lock, flags);

	return 0;
}

static const struct snd_soc_dai_ops img_prl_out_dai_ops = {
	.trigger = img_prl_out_trigger,
	.hw_params = img_prl_out_hw_params
};

static int img_prl_out_dai_probe(struct snd_soc_dai *dai)
{
	struct img_prl_out *prl = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &prl->dma_data, NULL);

	snd_soc_add_dai_controls(dai, img_prl_out_controls,
			ARRAY_SIZE(img_prl_out_controls));

	return 0;
}

static struct snd_soc_dai_driver img_prl_out_dai = {
	.probe = img_prl_out_dai_probe,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S24_LE
	},
	.ops = &img_prl_out_dai_ops
};

static const struct snd_soc_component_driver img_prl_out_component = {
	.name = "img-prl-out"
};

static int img_prl_out_probe(struct platform_device *pdev)
{
	struct img_prl_out *prl;
	struct resource *res;
	void __iomem *base;
	int ret;
	struct reset_control *rst;

	prl = devm_kzalloc(&pdev->dev, sizeof(*prl), GFP_KERNEL);
	if (!prl)
		return -ENOMEM;

	platform_set_drvdata(pdev, prl);

	prl->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	prl->base = base;

	prl->clk_sys = devm_clk_get(&pdev->dev, "sys");
	if (IS_ERR(prl->clk_sys))
		return PTR_ERR(prl->clk_sys);

	prl->clk_ref = devm_clk_get(&pdev->dev, "ref");
	if (IS_ERR(prl->clk_ref))
		return PTR_ERR(prl->clk_ref);

	ret = clk_prepare_enable(prl->clk_sys);
	if (ret)
		return ret;

	rst = devm_reset_control_get(&pdev->dev, "rst");
	if (IS_ERR(rst)) {
		dev_dbg(&pdev->dev, "No top level reset found\n");
		img_prl_out_writel(prl, IMG_PRL_OUT_CTL_SRST_MASK,
				IMG_PRL_OUT_CTL);
		img_prl_out_writel(prl, 0, IMG_PRL_OUT_CTL);
	} else {
		reset_control_assert(rst);
		reset_control_deassert(rst);
	}

	img_prl_out_writel(prl, IMG_PRL_OUT_CTL_EDGE_MASK,
			IMG_PRL_OUT_CTL);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = img_prl_out_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	spin_lock_init(&prl->lock);

	prl->dma_data.addr = res->start + IMG_PRL_OUT_TX_FIFO;
	prl->dma_data.addr_width = 4;
	prl->dma_data.maxburst = 4;

	ret = devm_snd_soc_register_component(&pdev->dev,
			&img_prl_out_component,
			&img_prl_out_dai, 1);
	if (ret)
		goto err_suspend;

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
	if (ret)
		goto err_suspend;

	dev_dbg(&pdev->dev, "Probe successful\n");

	return 0;

err_suspend:
	if (!pm_runtime_status_suspended(&pdev->dev))
		img_prl_out_suspend(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
	clk_disable_unprepare(prl->clk_sys);

	return ret;
}

static int img_prl_out_dev_remove(struct platform_device *pdev)
{
	struct img_prl_out *prl = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		img_prl_out_suspend(&pdev->dev);

	clk_disable_unprepare(prl->clk_sys);

	return 0;
}

static const struct of_device_id img_prl_out_of_match[] = {
	{ .compatible = "img,parallel-out" },
	{}
};
MODULE_DEVICE_TABLE(of, img_prl_of_match);

static const struct dev_pm_ops img_prl_out_pm_ops = {
	SET_RUNTIME_PM_OPS(img_prl_out_suspend,
			   img_prl_out_resume, NULL)
};

static struct platform_driver img_prl_out_driver = {
	.driver = {
		.name = "img-parallel-out",
		.of_match_table = img_prl_out_of_match,
		.pm = &img_prl_out_pm_ops
	},
	.probe = img_prl_out_probe,
	.remove = img_prl_out_dev_remove
};
module_platform_driver(img_prl_out_driver);

MODULE_AUTHOR("Damien Horsley <Damien.Horsley@imgtec.com>");
MODULE_DESCRIPTION("IMG Parallel Output Driver");
MODULE_LICENSE("GPL v2");
