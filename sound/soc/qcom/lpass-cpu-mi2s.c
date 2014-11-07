/*
 * Copyright (c) 2010-2011,2013-2014 The Linux Foundation. All rights reserved.
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
#include <linux/dma-mapping.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include "lpass-lpaif.h"
#include "lpass-cpu-mi2s.h"

#define DRV_NAME			"lpass-cpu-mi2s"

#define LPASS_OSR_TO_BIT_DIVIDER	4

static void lpass_lpaif_mi2s_playback(struct snd_soc_dai *dai, int enable)
{
	struct lpass_cpu_mi2s_data *pdata = snd_soc_dai_get_drvdata(dai);
	u32 cfg;

	cfg = readl(pdata->base + LPAIF_MI2S_CTL_OFFSET(LPAIF_I2S_PORT_MI2S));

	if (enable)
		cfg |= LPAIF_MI2SCTL_SPKEN;
	else
		cfg &= ~LPAIF_MI2SCTL_SPKEN;

	cfg &= ~LPAIF_MI2SCTL_WS;

	writel(cfg, pdata->base + LPAIF_MI2S_CTL_OFFSET(LPAIF_I2S_PORT_MI2S));
}

static int lpass_lpaif_mi2s_bitwidth(struct snd_soc_dai *dai, u32 bitwidth)
{
	struct lpass_cpu_mi2s_data *pdata = snd_soc_dai_get_drvdata(dai);
	u32 cfg;

	cfg = readl(pdata->base + LPAIF_MI2S_CTL_OFFSET(LPAIF_I2S_PORT_MI2S));

	cfg &= ~LPAIF_MI2SCTL_BITRATE_MASK;

	switch (bitwidth) {
	case SNDRV_PCM_FORMAT_S16:
		cfg |= LPAIF_MI2SCTL_BITRATE_16;
		break;
	case SNDRV_PCM_FORMAT_S24:
		cfg |= LPAIF_MI2SCTL_BITRATE_24;
		break;
	case SNDRV_PCM_FORMAT_S32:
		cfg |= LPAIF_MI2SCTL_BITRATE_32;
		break;
	default:
		dev_err(dai->dev, "%s: invalid bitwidth given: %u\n", __func__,
				bitwidth);
		return -EINVAL;
	}

	writel(cfg, pdata->base + LPAIF_MI2S_CTL_OFFSET(LPAIF_I2S_PORT_MI2S));

	return 0;
}

static int lpass_lpaif_mi2s_channels(struct snd_soc_dai *dai, u32 channels,
		u32 bitwidth)
{
	struct lpass_cpu_mi2s_data *pdata = snd_soc_dai_get_drvdata(dai);
	u32 cfg;

	cfg = readl(pdata->base + LPAIF_MI2S_CTL_OFFSET(LPAIF_I2S_PORT_MI2S));

	cfg &= ~LPAIF_MI2SCTL_SPKMODE_MASK;
	cfg &= ~LPAIF_MI2SCTL_SPKMONO_MASK;

	switch (channels) {
	case 1:
		cfg |= LPAIF_MI2SCTL_SPKMODE_SD0;
		cfg |= LPAIF_MI2SCTL_SPKMONO_MONO;
		break;
	case 2:
		cfg |= LPAIF_MI2SCTL_SPKMODE_SD0;
		cfg |= LPAIF_MI2SCTL_SPKMONO_STEREO;
		break;
	case 4:
		cfg |= LPAIF_MI2SCTL_SPKMODE_QUAD01;
		cfg |= LPAIF_MI2SCTL_SPKMONO_STEREO;
		break;
	case 6:
		cfg |= LPAIF_MI2SCTL_SPKMODE_6CH;
		cfg |= LPAIF_MI2SCTL_SPKMONO_STEREO;
		break;
	case 8:
		cfg |= LPAIF_MI2SCTL_SPKMODE_8CH;
		cfg |= LPAIF_MI2SCTL_SPKMONO_STEREO;
		break;
	default:
		dev_err(dai->dev, "%s: invalid channels given: %u\n", __func__,
				channels);
		return -EINVAL;
	}

	writel(cfg, pdata->base + LPAIF_MI2S_CTL_OFFSET(LPAIF_I2S_PORT_MI2S));

	return 0;
}

static int lpass_cpu_mi2s_set_pinctrl(struct snd_soc_dai *dai)
{
	struct lpass_cpu_mi2s_data *prtd = snd_soc_dai_get_drvdata(dai);
	struct mi2s_pinctrl *mi2s = &prtd->mi2s;
	int ret;

	dev_dbg(dai->dev, "%s: curr_state = %s\n", __func__,
			pin_states[mi2s->curr_state]);

	switch (mi2s->curr_state) {
	case STATE_DISABLED:
		ret = pinctrl_select_state(mi2s->pinctrl, mi2s->enabled);
		if (ret) {
			dev_err(dai->dev, "%s: pinctrl_select_state failed with %d\n",
					__func__, ret);
			return -EIO;
		}
		mi2s->curr_state = STATE_ENABLED;
		break;
	case STATE_ENABLED:
		dev_err(dai->dev, "%s: MI2S pins already enabled\n", __func__);
		break;
	default:
		dev_err(dai->dev, "%s: MI2S pin state is invalid: %d\n",
				__func__, mi2s->curr_state);
		return -EINVAL;
	}

	return 0;
}

static int lpass_cpu_mi2s_reset_pinctrl(struct snd_soc_dai *dai)
{
	struct lpass_cpu_mi2s_data *prtd = snd_soc_dai_get_drvdata(dai);
	struct mi2s_pinctrl *mi2s = &prtd->mi2s;
	int ret;

	dev_dbg(dai->dev, "%s: curr_state = %s\n", __func__,
			pin_states[mi2s->curr_state]);

	switch (mi2s->curr_state) {
	case STATE_ENABLED:
		ret = pinctrl_select_state(mi2s->pinctrl, mi2s->disabled);
		if (ret) {
			dev_err(dai->dev, "%s: pinctrl_select_state failed with %d\n",
					__func__, ret);
			return -EIO;
		}
		mi2s->curr_state = STATE_DISABLED;
		break;
	case STATE_DISABLED:
		dev_err(dai->dev, "%s: MI2S pins already disabled\n", __func__);
		break;
	default:
		dev_err(dai->dev, "%s: MI2S pin state is invalid: %d\n",
				__func__, mi2s->curr_state);
		return -EINVAL;
	}

	return 0;
}

static int lpass_cpu_mi2s_get_pinctrl(struct snd_soc_dai *dai)
{
	struct lpass_cpu_mi2s_data *prtd = snd_soc_dai_get_drvdata(dai);
	struct mi2s_pinctrl *mi2s = &prtd->mi2s;
	struct pinctrl *pinctrl;
	int ret;

	pinctrl = devm_pinctrl_get(dai->dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		dev_err(dai->dev, "%s: Unable to get pinctrl handle: %d\n",
				__func__, PTR_ERR_OR_ZERO(pinctrl));
		return -EINVAL;
	}
	mi2s->pinctrl = pinctrl;

	/* get all the states handles from Device Tree */
	mi2s->disabled = pinctrl_lookup_state(pinctrl, "mi2s-disabled");
	if (IS_ERR(mi2s->disabled)) {
		dev_err(dai->dev, "%s: could not get disabled pinstate: %d\n",
				__func__, PTR_ERR_OR_ZERO(mi2s->disabled));
		return -EINVAL;
	}

	mi2s->enabled = pinctrl_lookup_state(pinctrl, "mi2s-enabled");
	if (IS_ERR(mi2s->enabled)) {
		dev_err(dai->dev, "%s: could not get enabled pinstate: %d\n",
				__func__, PTR_ERR_OR_ZERO(mi2s->enabled));
		return -EINVAL;
	}

	/* Reset the MI2S pins to the disabled state */
	ret = pinctrl_select_state(mi2s->pinctrl, mi2s->disabled);
	if (ret) {
		dev_err(dai->dev, "%s: Disable MI2S pins failed with %d\n",
				__func__, ret);
		return -EIO;
	}
	mi2s->curr_state = STATE_DISABLED;

	return 0;
}

static int lpass_cpu_mi2s_daiops_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	int ret;

	ret = lpass_cpu_mi2s_set_pinctrl(dai);
	if (ret)
		dev_err(dai->dev, "%s: MI2S pinctrl set failed with %d\n",
				__func__, ret);

	return ret;
}

static void lpass_cpu_mi2s_daiops_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	int ret;

	ret = lpass_cpu_mi2s_reset_pinctrl(dai);
	if (ret)
		dev_err(dai->dev, "%s: MI2S pinctrl set failed with %d\n",
				__func__, ret);
}

static int lpass_cpu_mi2s_daiops_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	u32 ret;
	u32 bit_act;
	u32 bitwidth = params_format(params);
	u32 channels = params_channels(params);
	u32 rate = params_rate(params);
	struct lpass_cpu_mi2s_data *prtd = snd_soc_dai_get_drvdata(dai);

	bit_act = snd_pcm_format_width(bitwidth);
	if (bit_act < 0) {
		dev_err(dai->dev, "%s: Invalid bit width given\n", __func__);
		return bit_act;
	}

	ret = lpass_lpaif_mi2s_channels(dai, channels, bit_act);
	if (ret) {
		dev_err(dai->dev, "%s: Channel setting unsuccessful\n",
				__func__);
		return -EINVAL;
	}

	ret = lpass_lpaif_mi2s_bitwidth(dai, bitwidth);
	if (ret) {
		dev_err(dai->dev, "%s: Could not set bit width in HW\n",
				__func__);
		return -EINVAL;
	}

	ret = clk_set_rate(prtd->mi2s_osr_clk,
		(rate * bit_act * channels * LPASS_OSR_TO_BIT_DIVIDER));
	if (ret) {
		dev_err(dai->dev, "%s: error in setting mi2s osr clk: %d\n",
				__func__, ret);
		return ret;
	}

	ret = clk_prepare_enable(prtd->mi2s_osr_clk);
	if (ret) {
		dev_err(dai->dev, "%s: error in enabling mi2s osr clk: %d\n",
				__func__, ret);
		return ret;
	}

	ret = clk_set_rate(prtd->mi2s_bit_clk, rate * bit_act * channels);
	if (ret) {
		dev_err(dai->dev, "%s: error in setting mi2s bit clk: %d\n",
				__func__, ret);
		goto err;
	}

	ret = clk_prepare_enable(prtd->mi2s_bit_clk);
	if (ret) {
		dev_err(dai->dev, "%s: error in enabling mi2s bit clk: %d\n",
				__func__, ret);
		goto err;
	}

	prtd->mi2s_clocks_enabled = 1;

	return 0;

err:
	clk_disable_unprepare(prtd->mi2s_osr_clk);

	return ret;
}

static int lpass_cpu_mi2s_daiops_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct lpass_cpu_mi2s_data *prtd = snd_soc_dai_get_drvdata(dai);

	if (prtd->mi2s_clocks_enabled) {
		clk_disable_unprepare(prtd->mi2s_osr_clk);
		clk_disable_unprepare(prtd->mi2s_bit_clk);
	}
	prtd->mi2s_clocks_enabled = 0;

	return 0;
}

static int lpass_cpu_mi2s_daiops_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		lpass_lpaif_mi2s_playback(dai, 1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		lpass_lpaif_mi2s_playback(dai, 0);
		break;
	default:
		dev_err(dai->dev, "%s: Invalid trigger command given\n",
				__func__);
		ret = -EINVAL;
		break;
	}

	return ret;
}
static struct snd_soc_dai_ops lpass_cpu_mi2s_ops = {
	.startup	= lpass_cpu_mi2s_daiops_startup,
	.shutdown	= lpass_cpu_mi2s_daiops_shutdown,
	.hw_params	= lpass_cpu_mi2s_daiops_hw_params,
	.hw_free	= lpass_cpu_mi2s_daiops_hw_free,
	.trigger	= lpass_cpu_mi2s_daiops_trigger,
};

static int lpass_cpu_mi2s_dai_probe(struct snd_soc_dai *dai)
{
	int ret;
	struct lpass_cpu_mi2s_data *prtd = snd_soc_dai_get_drvdata(dai);

	ret = lpass_cpu_mi2s_get_pinctrl(dai);
	if (ret) {
		dev_err(dai->dev, "%s: Parsing MI2S pinctrl failed: %d\n",
				__func__, ret);
		return ret;
	}

	prtd->mi2s_osr_clk = devm_clk_get(dai->dev, "mi2s_osr_clk");
	if (IS_ERR(prtd->mi2s_osr_clk)) {
		dev_err(dai->dev, "%s: Error in getting mi2s_osr_clk\n",
				__func__);
		return PTR_ERR(prtd->mi2s_osr_clk);
	}

	prtd->mi2s_bit_clk = devm_clk_get(dai->dev, "mi2s_bit_clk");
	if (IS_ERR(prtd->mi2s_bit_clk)) {
		dev_err(dai->dev, "%s: Error in getting mi2s_bit_clk\n",
				__func__);
		return PTR_ERR(prtd->mi2s_bit_clk);
	}

	prtd->mi2s_clocks_enabled = 0;

	/* disable MI2S port */
	lpass_lpaif_mi2s_playback(dai, 0);

	return 0;
}

static struct snd_soc_dai_driver lpass_cpu_mi2s_dai_driver = {
	.name = "lpass-cpu-mi2s-dai",
	.playback = {
		.stream_name	= "lpass-cpu-mi2s-playback",
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
	.probe	= &lpass_cpu_mi2s_dai_probe,
	.ops    = &lpass_cpu_mi2s_ops,
};

static const struct snd_soc_component_driver lpass_cpu_mi2s_comp_driver = {
	.name = DRV_NAME,
};

static int lpass_cpu_mi2s_platform_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *lpass_res;
	struct lpass_cpu_mi2s_data *prtd;

	prtd = devm_kzalloc(&pdev->dev, sizeof(struct lpass_cpu_mi2s_data),
			GFP_KERNEL);
	if (!prtd)
		return -ENOMEM;
	platform_set_drvdata(pdev, prtd);

	prtd->ahbix_clk = devm_clk_get(&pdev->dev, "ahbix_clk");
	if (IS_ERR(prtd->ahbix_clk)) {
		dev_err(&pdev->dev, "%s: Error in getting ahbix_clk\n",
				__func__);
		return PTR_ERR(prtd->ahbix_clk);
	}

	clk_set_rate(prtd->ahbix_clk, 131072);
	ret = clk_prepare_enable(prtd->ahbix_clk);
	if (ret) {
		dev_err(&pdev->dev, "%s: Error in enabling ahbix_clk\n",
				__func__);
		return ret;
	}

	lpass_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			"lpass-lpaif-mem");
	if (!lpass_res) {
		dev_err(&pdev->dev, "%s: error getting resource\n", __func__);
		ret = -ENODEV;
		goto err_clk;
	}

	prtd->base = devm_ioremap_resource(&pdev->dev, lpass_res);
	if (IS_ERR(prtd->base)) {
		dev_err(&pdev->dev, "%s: error remapping resource\n",
				__func__);
		ret = PTR_ERR(prtd->base);
		goto err_clk;
	}

	prtd->irqnum = platform_get_irq_byname(pdev, "lpass-lpaif-irq");
	if (prtd->irqnum < 0) {
		dev_err(&pdev->dev, "%s: failed get irq res\n", __func__);
		ret = -ENODEV;
		goto err_clk;
	}

	prtd->irq_acquired = 0;

	ret = devm_snd_soc_register_component(&pdev->dev,
			&lpass_cpu_mi2s_comp_driver,
			&lpass_cpu_mi2s_dai_driver, 1);
	if (ret) {
		dev_err(&pdev->dev, "%s: error registering soc dai\n",
				__func__);
		goto err_clk;
	}

	ret = lpass_pcm_mi2s_platform_register(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "%s: error registering pcm device\n",
				__func__);
		goto err_clk;
	}

	return 0;

err_clk:
	clk_disable_unprepare(prtd->ahbix_clk);
	return ret;
}

static int lpass_cpu_mi2s_platform_remove(struct platform_device *pdev)
{
	struct lpass_cpu_mi2s_data *prtd = platform_get_drvdata(pdev);

	clk_disable_unprepare(prtd->ahbix_clk);

	return 0;
}

static const struct of_device_id lpass_cpu_mi2s_dt_match[] = {
	{.compatible = "qcom,lpass-cpu-mi2s"},
	{}
};

static struct platform_driver lpass_cpu_mi2s_platform_driver = {
	.driver	= {
		.name	= DRV_NAME,
		.of_match_table = lpass_cpu_mi2s_dt_match,
	},
	.probe	= lpass_cpu_mi2s_platform_probe,
	.remove	= lpass_cpu_mi2s_platform_remove,
};
module_platform_driver(lpass_cpu_mi2s_platform_driver);

MODULE_DESCRIPTION("QCOM LPASS MI2S CPU DRIVER");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, lpass_cpu_mi2s_dt_match);
