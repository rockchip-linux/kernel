/*
 * rockchip-hdmi-card.c
 *
 * ROCKCHIP ALSA SoC DAI driver for HDMI audio on rockchip processors.
 * Copyright (c) 2014, ROCKCHIP CORPORATION. All rights reserved.
 * Authors: Yakir Yang <ykk@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.*
 *
 */
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/core.h>
#include <sound/pcm_params.h>

#include "rockchip_i2s.h"

#define DRV_NAME "rockchip-hdmi-audio"

static int rockchip_hdmi_audio_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int dai_fmt = rtd->dai_link->dai_fmt;
	int mclk, ret;

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

	ret = snd_soc_dai_set_fmt(cpu_dai, dai_fmt);
	if (ret < 0) {
		dev_err(cpu_dai->dev, "failed to set cpu_dai fmt.\n");
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, mclk, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(cpu_dai->dev, "failed to set cpu_dai sysclk.\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops hdmi_audio_dai_ops = {
	.hw_params = rockchip_hdmi_audio_hw_params,
};

static struct snd_soc_dai_link hdmi_audio_dai = {
	.name = "RockchipHDMI",
	.stream_name = "RockchipHDMI",
	.codec_name = "dw-hdmi-audio",
	.codec_dai_name = "dw-hdmi-hifi",
	.ops = &hdmi_audio_dai_ops,
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		   SND_SOC_DAIFMT_CBS_CFS,
};

static struct snd_soc_card rockchip_hdmi_audio_card = {
	.name = "RockchipHDMI",
	.owner = THIS_MODULE,
	.dai_link = &hdmi_audio_dai,
	.num_links = 1,
};

static int rockchip_hdmi_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &rockchip_hdmi_audio_card;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);

	hdmi_audio_dai.cpu_of_node = of_parse_phandle(np, "i2s-controller", 0);
	if (!hdmi_audio_dai.cpu_of_node) {
		dev_err(&pdev->dev, "Property 'i2s-controller' missing !\n");
		goto free_priv_data;
	}

	hdmi_audio_dai.platform_of_node = hdmi_audio_dai.cpu_of_node;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "register card failed (%d)\n", ret);
		card->dev = NULL;
		goto free_cpu_of_node;
	}

	dev_info(&pdev->dev, "hdmi audio init success.\n");

	return 0;

free_cpu_of_node:
	hdmi_audio_dai.cpu_of_node = NULL;
	hdmi_audio_dai.platform_of_node = NULL;
free_priv_data:
	snd_soc_card_set_drvdata(card, NULL);
	platform_set_drvdata(pdev, NULL);
	card->dev = NULL;

	return ret;
}

static int rockchip_hdmi_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	snd_soc_card_set_drvdata(card, NULL);
	platform_set_drvdata(pdev, NULL);
	card->dev = NULL;

	return 0;
}

static const struct of_device_id rockchip_hdmi_audio_of_match[] = {
	{ .compatible = "rockchip,rk3288-hdmi-audio", },
	{},
};

static struct platform_driver rockchip_hdmi_audio_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = rockchip_hdmi_audio_of_match,
	},
	.probe = rockchip_hdmi_audio_probe,
	.remove = rockchip_hdmi_audio_remove,
};
module_platform_driver(rockchip_hdmi_audio_driver);

MODULE_AUTHOR("Yakir Yang <ykk@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip HDMI Audio ASoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, rockchip_hdmi_audio_of_match);
