/*
 * rk_es8396.c  --  SoC audio for rockchip
 *
 * Driver for rockchip es8396 audio
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "../codecs/es8396.h"
#include "card_info.h"
#include "rk_pcm.h"
#include "rk_i2s.h"

static const struct snd_soc_dapm_widget rk_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Internal Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

static const struct snd_soc_dapm_route rk_audio_map[] = {
	{"Headphone", NULL, "HPL"},
	{"Headphone", NULL, "HPR"},
	{"Ext Spk", NULL, "SPKOUTL"},
	{"Ext Spk", NULL, "SPKOUTR"},
	/* AMIC */
	{"Headset Mic", NULL, "MIC Bias"},
	{"MONOINP", NULL, "Headset Mic"},
	{"MONOINN", NULL, "Headset Mic"},

	{"Internal Mic", NULL, "MIC Bias"},
	{"MIC", NULL, "Internal Mic"},
};

static const struct snd_kcontrol_new rk_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Internal Mic"),
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
};

static int es8396_aif1_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int pll_out = 0, dai_fmt = rtd->card->dai_link->dai_fmt;
	int ret;

	pr_debug("Enter::%s----%d\n", __func__, __LINE__);

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_fmt);
	if (ret < 0) {
		pr_err("%s():failed to set the format for codec side\n",
		       __func__);
		return ret;
	}

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_fmt);
	if (ret < 0) {
		pr_err("%s():failed to set the format for cpu side\n",
		       __func__);
		return ret;
	}

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
		pll_out = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
		pll_out = 11289600;
		break;
	default:
		pr_err("Enter:%s, %d, Error rate=%d\n",
		       __func__, __LINE__, params_rate(params));
		return -EINVAL;
	}
	pr_debug("Enter:%s, %d, rate=%d\n", __func__, __LINE__,
		 params_rate(params));

	if ((dai_fmt & SND_SOC_DAIFMT_MASTER_MASK) == SND_SOC_DAIFMT_CBS_CFS) {
		snd_soc_dai_set_sysclk(cpu_dai, 0, pll_out, 0);
		snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_BCLK,
				       (pll_out/4)/params_rate(params)-1);
		snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_MCLK, 3);
	}

	pr_debug("Enter:%s, %d, LRCK=%d\n", __func__, __LINE__,
		 (pll_out/4)/params_rate(params));
	return 0;
}

static int es8396_aif2_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int pll_out = 0, dai_fmt = rtd->card->dai_link->dai_fmt;
	int ret;

	pr_debug("Enter::%s----%d\n", __func__, __LINE__);

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_fmt);
	if (ret < 0) {
		pr_err("%s():failed to set the format for codec side\n",
		       __func__);
		return ret;
	}

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_fmt);
	if (ret < 0) {
		pr_err("%s():failed to set the format for cpu side\n",
		       __func__);
		return ret;
	}

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
		pll_out = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
		pll_out = 11289600;
		break;
	default:
		pr_err("Enter:%s, %d, Error rate=%d\n",
		       __func__, __LINE__, params_rate(params));
		return -EINVAL;
	}
	pr_debug("Enter:%s, %d, rate=%d\n", __func__, __LINE__,
		 params_rate(params));

	if ((dai_fmt & SND_SOC_DAIFMT_MASTER_MASK) == SND_SOC_DAIFMT_CBS_CFS) {
		snd_soc_dai_set_sysclk(cpu_dai, 0, pll_out, 0);
		snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_BCLK,
				       (pll_out/4)/params_rate(params)-1);
		snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_MCLK, 3);
	}

	pr_debug("Enter:%s, %d, LRCK=%d\n", __func__, __LINE__,
		 (pll_out/4)/params_rate(params));
	return 0;
}

static int rk_es8396_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	pr_debug("Enter::%s----%d\n", __func__, __LINE__);
	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
				     11289600, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err(KERN_ERR "Failed to set es8396 SYSCLK: %d\n", ret);
		return ret;
	}
	return 0;
}

static struct snd_soc_ops es8396_aif1_ops = {
	.hw_params = es8396_aif1_hw_params,
};

static struct snd_soc_ops es8396_aif2_ops = {
	.hw_params = es8396_aif2_hw_params,
};

static struct snd_soc_dai_link rk_dai[] = {
	{
		.name = "ES8396i I2S1",
		.stream_name = "ES8396 PCM",
		.codec_dai_name = "es8396-aif1",
		.init = rk_es8396_init,
		.ops = &es8396_aif1_ops,
	},
	{
		.name = "ES8396 I2S2",
		.stream_name = "ES8396 PCM",
		.codec_dai_name = "es8396-aif2",
		.ops = &es8396_aif2_ops,
	},
};

/* SoC card */
static struct snd_soc_card rockchip_es8396_snd_card = {
	.name = "RK_ES8396",
	.dai_link = rk_dai,
	.num_links = 2,
	.dapm_widgets = rk_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rk_dapm_widgets),
	.dapm_routes = rk_audio_map,
	.num_dapm_routes = ARRAY_SIZE(rk_audio_map),
	.controls = rk_mc_controls,
	.num_controls = ARRAY_SIZE(rk_mc_controls),
};

static int rockchip_es8396_audio_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card = &rockchip_es8396_snd_card;

	card->dev = &pdev->dev;

	ret = rockchip_of_get_sound_card_info(card);
	if (ret) {
		pr_err("%s() get sound card info failed:%d\n", __func__, ret);
		return ret;
	}

	ret = snd_soc_register_card(card);
	if (ret)
		pr_err("%s() register card failed:%d\n", __func__, ret);

	return ret;
}

static int rockchip_es8396_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rockchip_es8396_of_match[] = {
	{ .compatible = "rockchip-es8396", },
	{},
};
MODULE_DEVICE_TABLE(of, rockchip_es8396_of_match);
#endif /* CONFIG_OF */

static struct platform_driver rockchip_es8396_audio_driver = {
	.driver         = {
		.name   = "rockchip-es8396",
		.owner  = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(rockchip_es8396_of_match),
	},
	.probe          = rockchip_es8396_audio_probe,
	.remove         = rockchip_es8396_audio_remove,
};

module_platform_driver(rockchip_es8396_audio_driver);

/* Module information */
MODULE_AUTHOR("rockchip");
MODULE_DESCRIPTION("ROCKCHIP es8396 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:rk_es8396");
