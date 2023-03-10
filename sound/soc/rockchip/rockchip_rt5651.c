/*
 * rockchip_rt5651.c  --  RK3399 machine driver with RT5651 codecs
 *
 * Copyright (c) 2020 FriendlyElec Computer Tech. Co., Ltd.
 * (http://www.friendlyarm.com)
 *
 * Copyright (c) 2016, ROCKCHIP CORPORATION.  All rights reserved.
 * Author: Xiaotan Luo <lxt@rock-chips.com>
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
#include <linux/of_gpio.h>
#include <sound/jack.h>
#include <sound/soc.h>

#include "rockchip_i2s.h"
#include "../codecs/rt5651.h"

#define DRV_NAME "rockchip-rt5651"

struct rockchip_card_data {
	struct snd_soc_card *card;
	int gpio_hp_det;
	int gpio_hp_det_invert;
	bool codec_hp_det;
};

static const struct snd_soc_dapm_widget rockchip_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_SPK("Lineout", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_MIC("HDMIIN", NULL),
};

static const struct snd_soc_dapm_route rockchip_dapm_routes[] = {
	{"Headphones", NULL, "HPOL"},
	{"Headphones", NULL, "HPOR"},
	{"Lineout", NULL, "LOUTL"},
	{"Lineout", NULL, "LOUTR"},
	{"AIF2 Playback", NULL, "HDMIIN"},
};

static const struct snd_kcontrol_new rockchip_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphones"),
	SOC_DAPM_PIN_SWITCH("Lineout"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
};

static int rockchip_rt5651_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int mclk, ret;

	/* in bypass mode, the mclk has to be one of the frequencies below */
	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, mclk, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Can't set cpu clock out %d\n", ret);
		return ret;
	}

	snd_soc_dai_set_pll(codec_dai, 0, RT5651_PLL1_S_MCLK, mclk, mclk * 2);

	ret = snd_soc_dai_set_sysclk(codec_dai, RT5651_SCLK_S_PLL1, mclk * 2,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Can't set codec clock in %d\n", ret);
		return ret;
	}

	return 0;
}

static int rockchip_rt5651_voice_hw_params(struct snd_pcm_substream *substream,
					   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int mclk, ret;

	/* in bypass mode, the mclk has to be one of the frequencies below */
	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, mclk, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Can't set codec clock in %d\n", ret);
		return ret;
	}
	return 0;
}

static struct snd_soc_ops rockchip_sound_rt5651_hifi_ops = {
	.hw_params = rockchip_rt5651_hw_params,
};

static struct snd_soc_ops rockchip_sound_rt5651_voice_ops = {
	.hw_params = rockchip_rt5651_voice_hw_params,
};

static struct snd_soc_jack rockchip_hp_jack;
static struct snd_soc_jack_pin rockchip_hp_jack_pins[] = {
	{
		.pin = "Headphones",
		.mask = SND_JACK_HEADPHONE,
	},
};
static struct snd_soc_jack_gpio rockchip_hp_jack_gpio = {
	.name = "Headphone detection",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,
};

static int rockchip_card_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct rockchip_card_data *priv =
		snd_soc_card_get_drvdata(rtd->card);

	if (gpio_is_valid(priv->gpio_hp_det)) {
		snd_soc_card_jack_new(rtd->card, "Headphones",
				SND_JACK_HEADPHONE,
				&rockchip_hp_jack,
				rockchip_hp_jack_pins,
				ARRAY_SIZE(rockchip_hp_jack_pins));

		rockchip_hp_jack_gpio.gpio = priv->gpio_hp_det;
		rockchip_hp_jack_gpio.invert = priv->gpio_hp_det_invert;
		snd_soc_jack_add_gpios(&rockchip_hp_jack, 1,
				&rockchip_hp_jack_gpio);
	}

	return 0;
}

enum {
	DAILINK_RT5651_HIFI,
	DAILINK_RT5651_VOICE,
	DAILINK_RT5651_MAX
};

static struct snd_soc_dai_link rockchip_dailinks[] = {
	[DAILINK_RT5651_HIFI] = {
		.name = "RT5651 HIFI",
		.stream_name = "RT5651 PCM",
		.codec_dai_name = "rt5651-aif1",
		.ops = &rockchip_sound_rt5651_hifi_ops,
		.init = rockchip_card_dai_init,
		/* set rt5651 as slave */
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	},
	[DAILINK_RT5651_VOICE] = {
		.name = "RT5651 HDMIIN",
		.stream_name = "RT5651 PCM",
		.codec_dai_name = "rt5651-aif2",
		.ops = &rockchip_sound_rt5651_voice_ops,
		/* set rt5651 as slave */
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	},
};

static struct snd_soc_card rockchip_sound_card = {
	.name = "realtekrt5651codec_hdmiin",
	.owner = THIS_MODULE,
	.dai_link = rockchip_dailinks,
	.num_links =  ARRAY_SIZE(rockchip_dailinks),
	.dapm_widgets = rockchip_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rockchip_dapm_widgets),
	.dapm_routes = rockchip_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(rockchip_dapm_routes),
	.controls = rockchip_controls,
	.num_controls = ARRAY_SIZE(rockchip_controls),
};

static int rockchip_sound_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &rockchip_sound_card;
	struct rockchip_card_data *priv;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *cpu_node;
	enum of_gpio_flags flags;
	int i, ret;

	dev_info(&pdev->dev, "%s\n", __func__);

	cpu_node = of_parse_phandle(np, "rockchip,cpu", 0);
	if (!cpu_node) {
		dev_err(&pdev->dev,
			"Property 'rockchip,cpu' failed\n");
		return -EINVAL;
	}

	for (i = 0; i < DAILINK_RT5651_MAX; i++) {
		rockchip_dailinks[i].platform_of_node = cpu_node;
		rockchip_dailinks[i].cpu_of_node = cpu_node;

		rockchip_dailinks[i].codec_of_node =
			of_parse_phandle(np, "rockchip,codec", i);
		if (!rockchip_dailinks[i].codec_of_node) {
			dev_err(&pdev->dev,
				"Property[%d] 'rockchip,codec' failed\n", i);
			return -EINVAL;
		}
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->card = card;
	priv->codec_hp_det = false;
	priv->gpio_hp_det = of_get_named_gpio_flags(np,
			"rockchip,hp-det-gpio", 0, &flags);
	priv->gpio_hp_det_invert = !!(flags & OF_GPIO_ACTIVE_LOW);

	card->dev = &pdev->dev;
	snd_soc_card_set_drvdata(card, priv);

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(&pdev->dev, "%s register card failed %d\n",
			__func__, ret);

	dev_info(&pdev->dev, "snd_soc_register_card successful\n");
	return ret;
}

static const struct of_device_id rockchip_sound_of_match[] = {
	{ .compatible = "rockchip,rockchip-rt5651-sound", },
	{},
};

static struct platform_driver rockchip_sound_driver = {
	.probe = rockchip_sound_probe,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = rockchip_sound_of_match,
#ifdef CONFIG_PM
		.pm = &snd_soc_pm_ops,
#endif
	},
};

module_platform_driver(rockchip_sound_driver);

MODULE_AUTHOR("Xiaotan Luo <lxt@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip ASoC Machine Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, rockchip_sound_of_match);
