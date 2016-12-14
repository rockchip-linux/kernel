/*
 * rk_i2s_dummy.c -- i2s dummy audio for rockchip
 *
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "card_info.h"
#include "rk_pcm.h"
#include "rk_i2s.h"

#define BCLK_DIV64	63

static int i2s_dummy_audio_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int pll_out = 0, dai_fmt = rtd->dai_link->dai_fmt;
	int div_bclk, div_mclk;
	int ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, dai_fmt);
	if (ret < 0)
		return ret;

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 96000:
		pll_out = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		pll_out = 11289600;
		break;
	case 176400:
		pll_out = 22579200;
		break;
	case 192000:
		pll_out = 24576000;
		break;
	default:
		return -EINVAL;
	}

	div_bclk = BCLK_DIV64;
	div_mclk = pll_out / (params_rate(params) * (div_bclk + 1)) - 1;

	snd_soc_dai_set_sysclk(cpu_dai, 0, pll_out, 0);
	snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_BCLK, div_bclk);
	snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_MCLK, div_mclk);

	return 0;
}

static struct snd_soc_ops dummy_audio_ops = {
	.hw_params = i2s_dummy_audio_hw_params,
};

static struct snd_soc_dai_link dummy_audio_dai = {
	.name = "Dummy Audio",
	.stream_name = "dummy pcm",
	.codec_dai_name = "dummy_codec",
	.ops = &dummy_audio_ops,
};

static struct snd_soc_card rockchip_dummy_snd_card = {
	.name = "RK-I2S-DUMMY-AUDIO",
	.dai_link = &dummy_audio_dai,
	.num_links = 1,
};

static int rockchip_dummy_audio_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card = &rockchip_dummy_snd_card;

	card->dev = &pdev->dev;

	ret = rockchip_of_get_sound_card_info(card);
	if (ret)
		return ret;

	return snd_soc_register_card(card);
}

static int rockchip_dummy_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id rockchip_dummy_audio_of_match[] = {
	{ .compatible = "rockchip,i2s-dummy-audio", },
	{},
};
MODULE_DEVICE_TABLE(of, rockchip_dummy_audio_of_match);

static struct platform_driver rockchip_dummy_audio_driver = {
	.driver = {
		.name = "dummy_audio",
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(rockchip_dummy_audio_of_match),
	},
	.probe = rockchip_dummy_audio_probe,
	.remove = rockchip_dummy_audio_remove,
};

module_platform_driver(rockchip_dummy_audio_driver);

MODULE_AUTHOR("Sugar <sugar.zhang@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip I2S Dummy Audio Card");
MODULE_LICENSE("GPL v2");
