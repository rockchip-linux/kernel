/*
 * Copyright (c) 2010-2011,2013-2015 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include "lpass-lpaif-ipq806x.h"
#include "lpass.h"

#define DRV_NAME			"lpass-cpu"

static int lpass_cpu_daiops_set_sysclk(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq, int dir)
{
	struct lpass_data *drvdata = snd_soc_dai_get_drvdata(dai);

	drvdata->default_sysclk_freq = freq;

	return 0;
}

static int lpass_cpu_daiops_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct lpass_data *drvdata = snd_soc_dai_get_drvdata(dai);
	int ret;

	ret = clk_prepare_enable(drvdata->mi2s_osr_clk);
	if (ret) {
		dev_err(dai->dev, "%s() error in enabling mi2s osr clk: %d\n",
				__func__, ret);
		return ret;
	}

	ret = clk_prepare_enable(drvdata->mi2s_bit_clk);
	if (ret) {
		dev_err(dai->dev, "%s() error in enabling mi2s bit clk: %d\n",
				__func__, ret);
		clk_disable_unprepare(drvdata->mi2s_osr_clk);
		return ret;
	}

	return 0;
}

static void lpass_cpu_daiops_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct lpass_data *drvdata = snd_soc_dai_get_drvdata(dai);

	clk_disable_unprepare(drvdata->mi2s_bit_clk);
	clk_disable_unprepare(drvdata->mi2s_osr_clk);
}

static int lpass_cpu_daiops_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct lpass_data *drvdata = snd_soc_dai_get_drvdata(dai);
	snd_pcm_format_t format = params_format(params);
	unsigned int channels = params_channels(params);
	unsigned int rate = params_rate(params);
	unsigned long bclk_freq, oclk_freq;
	unsigned int regval;
	int bitwidth, ret;

	bitwidth = snd_pcm_format_width(format);
	if (bitwidth < 0) {
		dev_err(dai->dev, "%s() invalid bit width given\n", __func__);
		return bitwidth;
	}

	regval = 0;
	regval |= LPAIF_I2SCTL_LOOPBACK_DISABLE;
	regval |= LPAIF_I2SCTL_WSSRC_INTERNAL;

	switch (bitwidth) {
	case 16:
		regval |= LPAIF_I2SCTL_BITWIDTH_16;
		break;
	case 24:
		regval |= LPAIF_I2SCTL_BITWIDTH_24;
		break;
	case 32:
		regval |= LPAIF_I2SCTL_BITWIDTH_32;
		break;
	default:
		dev_err(dai->dev, "%s() invalid bitwidth given: %u\n",
				__func__, bitwidth);
		return -EINVAL;
	}

	switch (channels) {
	case 1:
		regval |= LPAIF_I2SCTL_SPKMODE_SD0;
		regval |= LPAIF_I2SCTL_SPKMONO_MONO;
		break;
	case 2:
		regval |= LPAIF_I2SCTL_SPKMODE_SD0;
		regval |= LPAIF_I2SCTL_SPKMONO_STEREO;
		break;
	case 4:
		regval |= LPAIF_I2SCTL_SPKMODE_QUAD01;
		regval |= LPAIF_I2SCTL_SPKMONO_STEREO;
		break;
	case 6:
		regval |= LPAIF_I2SCTL_SPKMODE_6CH;
		regval |= LPAIF_I2SCTL_SPKMONO_STEREO;
		break;
	case 8:
		regval |= LPAIF_I2SCTL_SPKMODE_8CH;
		regval |= LPAIF_I2SCTL_SPKMONO_STEREO;
		break;
	default:
		dev_err(dai->dev, "%s() invalid channels given: %u\n",
				__func__, channels);
		return -EINVAL;
	}

	ret = regmap_write(drvdata->lpaif_map,
			LPAIF_I2SCTL_REG(LPAIF_I2S_PORT_MI2S), regval);
	if (ret) {
		dev_err(dai->dev, "%s() error writing to i2sctl reg: %d\n",
				__func__, ret);
		return ret;
	}

	/*
	 * adjust the OSR clock so that the BIT clock can successfully be
	 * derived from it by the hardware
	 */
	bclk_freq = rate * bitwidth * 2;
	if (bitwidth == drvdata->alt_sysclk_enable_bitwidth &&
			drvdata->alt_sysclk_enable)
		oclk_freq = drvdata->alt_sysclk_freq;
	else if (!drvdata->sysclk_shift_enable)
		oclk_freq = drvdata->default_sysclk_freq;
	else if (bclk_freq >= drvdata->default_sysclk_freq >>
			drvdata->sysclk_shift_compare)
		oclk_freq = drvdata->default_sysclk_freq;
	else
		oclk_freq = drvdata->default_sysclk_freq >>
				drvdata->sysclk_shift_amount;

	ret = clk_set_rate(drvdata->mi2s_osr_clk, oclk_freq);
	if (ret) {
		dev_err(dai->dev, "%s() error setting mi2s osrclk to %lu: %d\n",
				__func__, oclk_freq, ret);
		return ret;
	}

	ret = clk_set_rate(drvdata->mi2s_bit_clk, bclk_freq);
	if (ret) {
		dev_err(dai->dev, "%s() error setting mi2s bitclk to %lu: %d\n",
				__func__, bclk_freq, ret);
		return ret;
	}

	return 0;
}

static int lpass_cpu_daiops_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct lpass_data *drvdata = snd_soc_dai_get_drvdata(dai);
	int ret;

	ret = regmap_write(drvdata->lpaif_map,
			LPAIF_I2SCTL_REG(LPAIF_I2S_PORT_MI2S), 0);
	if (ret)
		dev_err(dai->dev, "%s() error writing to i2sctl reg: %d\n",
				__func__, ret);

	return ret;
}

static int lpass_cpu_daiops_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct lpass_data *drvdata = snd_soc_dai_get_drvdata(dai);
	unsigned int reg, mask, val;
	int ret;

	reg = LPAIF_I2SCTL_REG(LPAIF_I2S_PORT_MI2S);
	mask = LPAIF_I2SCTL_SPKEN_MASK;
	val = LPAIF_I2SCTL_SPKEN_ENABLE;
	ret = regmap_update_bits(drvdata->lpaif_map, reg, mask, val);
	if (ret)
		dev_err(dai->dev, "%s() error writing to i2sctl reg: %d\n",
				__func__, ret);

	return ret;
}

static int lpass_cpu_daiops_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	struct lpass_data *drvdata = snd_soc_dai_get_drvdata(dai);
	unsigned int reg, mask, val;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		reg = LPAIF_I2SCTL_REG(LPAIF_I2S_PORT_MI2S);
		mask = LPAIF_I2SCTL_SPKEN_MASK;
		val = LPAIF_I2SCTL_SPKEN_ENABLE;
		ret = regmap_update_bits(drvdata->lpaif_map, reg, mask, val);
		if (ret)
			dev_err(dai->dev, "%s() error writing to i2sctl reg: %d\n",
					__func__, ret);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		reg = LPAIF_I2SCTL_REG(LPAIF_I2S_PORT_MI2S);
		mask = LPAIF_I2SCTL_SPKEN_MASK;
		val = LPAIF_I2SCTL_SPKEN_DISABLE;
		ret = regmap_update_bits(drvdata->lpaif_map, reg, mask, val);
		if (ret)
			dev_err(dai->dev, "%s() error writing to i2sctl reg: %d\n",
					__func__, ret);
		break;
	}

	return ret;
}

static struct snd_soc_dai_ops lpass_cpu_dai_ops = {
	.set_sysclk	= lpass_cpu_daiops_set_sysclk,
	.startup	= lpass_cpu_daiops_startup,
	.shutdown	= lpass_cpu_daiops_shutdown,
	.hw_params	= lpass_cpu_daiops_hw_params,
	.hw_free	= lpass_cpu_daiops_hw_free,
	.prepare	= lpass_cpu_daiops_prepare,
	.trigger	= lpass_cpu_daiops_trigger,
};

static int lpass_cpu_dai_probe(struct snd_soc_dai *dai)
{
	struct lpass_data *drvdata = snd_soc_dai_get_drvdata(dai);
	int ret;

	/* ensure audio hardware is disabled */
	ret = regmap_write(drvdata->lpaif_map,
			LPAIF_I2SCTL_REG(LPAIF_I2S_PORT_MI2S), 0);
	if (ret)
		dev_err(dai->dev, "%s() error writing to i2sctl reg: %d\n",
				__func__, ret);

	return ret;
}

static struct snd_soc_dai_driver lpass_cpu_dai_driver = {
	.playback = {
		.stream_name	= DRV_NAME "-playback",
		.formats	= SNDRV_PCM_FMTBIT_S16 |
					SNDRV_PCM_FMTBIT_S24 |
					SNDRV_PCM_FMTBIT_S32,
		.rates		= SNDRV_PCM_RATE_8000 |
					SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_32000 |
					SNDRV_PCM_RATE_48000 |
					SNDRV_PCM_RATE_96000,
		.rate_min	= 8000,
		.rate_max	= 96000,
		.channels_min	= 1,
		.channels_max	= 8,
	},
	.probe	= &lpass_cpu_dai_probe,
	.ops    = &lpass_cpu_dai_ops,
};

static const struct snd_soc_component_driver lpass_cpu_comp_driver = {
	.name = DRV_NAME,
};

static bool lpass_cpu_regmap_writeable(struct device *dev, unsigned int reg)
{
	int i;

	for (i = 0; i < LPAIF_I2S_PORT_NUM; ++i)
		if (reg == LPAIF_I2SCTL_REG(i))
			return true;

	for (i = 0; i < LPAIF_IRQ_PORT_NUM; ++i) {
		if (reg == LPAIF_IRQEN_REG(i))
			return true;
		if (reg == LPAIF_IRQCLEAR_REG(i))
			return true;
	}

	for (i = 0; i < LPAIF_RDMA_CHAN_NUM; ++i) {
		if (reg == LPAIF_RDMACTL_REG(i))
			return true;
		if (reg == LPAIF_RDMABASE_REG(i))
			return true;
		if (reg == LPAIF_RDMABUFF_REG(i))
			return true;
		if (reg == LPAIF_RDMAPER_REG(i))
			return true;
	}

	return false;
}

static bool lpass_cpu_regmap_readable(struct device *dev, unsigned int reg)
{
	int i;

	for (i = 0; i < LPAIF_I2S_PORT_NUM; ++i)
		if (reg == LPAIF_I2SCTL_REG(i))
			return true;

	for (i = 0; i < LPAIF_IRQ_PORT_NUM; ++i) {
		if (reg == LPAIF_IRQEN_REG(i))
			return true;
		if (reg == LPAIF_IRQSTAT_REG(i))
			return true;
	}

	for (i = 0; i < LPAIF_RDMA_CHAN_NUM; ++i) {
		if (reg == LPAIF_RDMACTL_REG(i))
			return true;
		if (reg == LPAIF_RDMABASE_REG(i))
			return true;
		if (reg == LPAIF_RDMABUFF_REG(i))
			return true;
		if (reg == LPAIF_RDMACURR_REG(i))
			return true;
		if (reg == LPAIF_RDMAPER_REG(i))
			return true;
	}

	return false;
}

static bool lpass_cpu_regmap_volatile(struct device *dev, unsigned int reg)
{
	int i;

	for (i = 0; i < LPAIF_IRQ_PORT_NUM; ++i)
		if (reg == LPAIF_IRQSTAT_REG(i))
			return true;

	for (i = 0; i < LPAIF_RDMA_CHAN_NUM; ++i)
		if (reg == LPAIF_RDMACURR_REG(i))
			return true;

	return false;
}

static const struct regmap_config lpass_cpu_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = LPAIF_RDMAPER_REG(LPAIF_RDMA_CHAN_MAX),
	.writeable_reg = lpass_cpu_regmap_writeable,
	.readable_reg = lpass_cpu_regmap_readable,
	.volatile_reg = lpass_cpu_regmap_volatile,
	.cache_type = REGCACHE_FLAT,
};

static int lpass_cpu_parse_of(struct device *dev, struct lpass_data *drvdata)
{
	struct device_node *np = dev->of_node;
	int ret;

	if (!np || !of_device_is_available(np)) {
		dev_err(dev, "%s() no available info for %s\n", __func__,
				DRV_NAME);
		return -EINVAL;
	}

	if (of_property_read_bool(np, "qcom,system-clock-shift")) {
		ret = of_property_read_u32(np,
				"qcom,system-clock-shift-compare",
				&drvdata->sysclk_shift_compare);
		if (ret) {
			dev_err(dev, "%s() qcom,system-clock-shift-compare not found: %d\n",
					__func__, ret);
			return -EINVAL;
		}
		ret = of_property_read_u32(np,
				"qcom,system-clock-shift-amount",
				&drvdata->sysclk_shift_amount);
		if (ret) {
			dev_err(dev, "%s() qcom,system-clock-shift-amount not found: %d\n",
					__func__, ret);
			return -EINVAL;
		}
		drvdata->sysclk_shift_enable = true;
	} else {
		drvdata->sysclk_shift_enable = false;
	}

	if (of_property_read_bool(np, "qcom,alternate-sysclk")) {
		ret = of_property_read_u32(np,
				"qcom,alternate-sysclk-bitwidth",
				&drvdata->alt_sysclk_enable_bitwidth);
		if (ret) {
			dev_err(dev, "%s() qcom,alternate-sysclk-bitwidth not found: %d\n",
					__func__, ret);
			return -EINVAL;
		}
		ret = of_property_read_u32(np,
				"qcom,alternate-sysclk-frequency",
				&drvdata->alt_sysclk_freq);
		if (ret) {
			dev_err(dev, "%s() qcom,alternate-sysclk-frequency not found: %d\n",
					__func__, ret);
			return -EINVAL;
		}
		drvdata->alt_sysclk_enable = true;
	} else {
		drvdata->alt_sysclk_enable = false;
	}

	return 0;
}

static int lpass_cpu_platform_probe(struct platform_device *pdev)
{
	struct lpass_data *drvdata;
	struct resource *res;
	int ret;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct lpass_data),
			GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;
	platform_set_drvdata(pdev, drvdata);

	ret = lpass_cpu_parse_of(&pdev->dev, drvdata);
	if (ret)
		return ret;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "lpass-lpaif");
	if (!res) {
		dev_err(&pdev->dev, "%s() error getting resource\n", __func__);
		return -ENODEV;
	}

	drvdata->lpaif = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR((void const __force *)drvdata->lpaif)) {
		dev_err(&pdev->dev, "%s() error mapping reg resource: %ld\n",
				__func__,
				PTR_ERR((void const __force *)drvdata->lpaif));
		return PTR_ERR((void const __force *)drvdata->lpaif);
	}

	drvdata->lpaif_map = devm_regmap_init_mmio(&pdev->dev, drvdata->lpaif,
			&lpass_cpu_regmap_config);
	if (IS_ERR(drvdata->lpaif_map)) {
		dev_err(&pdev->dev, "%s() error initializing regmap: %ld\n",
				__func__, PTR_ERR(drvdata->lpaif_map));
		return PTR_ERR(drvdata->lpaif_map);
	}

	drvdata->mi2s_osr_clk = devm_clk_get(&pdev->dev, "mi2s-osr-clk");
	if (IS_ERR(drvdata->mi2s_osr_clk)) {
		dev_err(&pdev->dev, "%s() error getting mi2s-osr-clk: %ld\n",
				__func__, PTR_ERR(drvdata->mi2s_osr_clk));
		return PTR_ERR(drvdata->mi2s_osr_clk);
	}

	drvdata->mi2s_bit_clk = devm_clk_get(&pdev->dev, "mi2s-bit-clk");
	if (IS_ERR(drvdata->mi2s_bit_clk)) {
		dev_err(&pdev->dev, "%s() error getting mi2s-bit-clk: %ld\n",
				__func__, PTR_ERR(drvdata->mi2s_bit_clk));
		return PTR_ERR(drvdata->mi2s_bit_clk);
	}

	drvdata->ahbix_clk = devm_clk_get(&pdev->dev, "ahbix-clk");
	if (IS_ERR(drvdata->ahbix_clk)) {
		dev_err(&pdev->dev, "%s() error getting ahbix-clk: %ld\n",
				__func__, PTR_ERR(drvdata->ahbix_clk));
		return PTR_ERR(drvdata->ahbix_clk);
	}

	ret = clk_set_rate(drvdata->ahbix_clk, LPASS_AHBIX_CLOCK_FREQUENCY);
	if (ret) {
		dev_err(&pdev->dev, "%s() error setting rate on ahbix_clk: %d\n",
				__func__, ret);
		return ret;
	}

	ret = clk_prepare_enable(drvdata->ahbix_clk);
	if (ret) {
		dev_err(&pdev->dev, "%s() Error enabling ahbix_clk: %d\n",
				__func__, ret);
		return ret;
	}

	ret = devm_snd_soc_register_component(&pdev->dev,
			&lpass_cpu_comp_driver, &lpass_cpu_dai_driver, 1);
	if (ret) {
		dev_err(&pdev->dev, "%s() error registering cpu driver: %d\n",
				__func__, ret);
		goto err_clk;
	}

	ret = asoc_qcom_lpass_platform_register(pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s() error registering platform driver: %d\n",
				__func__, ret);
		goto err_clk;
	}

	return 0;

err_clk:
	clk_disable_unprepare(drvdata->ahbix_clk);
	return ret;
}

static int lpass_cpu_platform_remove(struct platform_device *pdev)
{
	struct lpass_data *drvdata = platform_get_drvdata(pdev);

	clk_disable_unprepare(drvdata->ahbix_clk);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id lpass_cpu_device_id[] = {
	{.compatible = "qcom," DRV_NAME},
	{}
};
#endif

static struct platform_driver lpass_cpu_platform_driver = {
	.driver	= {
		.name		= DRV_NAME,
		.of_match_table	= of_match_ptr(lpass_cpu_device_id),
	},
	.probe	= lpass_cpu_platform_probe,
	.remove	= lpass_cpu_platform_remove,
};
module_platform_driver(lpass_cpu_platform_driver);

MODULE_DESCRIPTION("QTi LPASS CPU Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, lpass_cpu_device_id);
