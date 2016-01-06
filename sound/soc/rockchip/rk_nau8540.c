/*
 * Rockchip machine ASoC driver for boards using a nau8540 CODEC.
 *
 * Copyright (c) 2015, Fuzhou Rockchip Electronics Co., Ltd All rights reserved.
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
#include "rk_i2s.h"

static int nau8540_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int dai_fmt = rtd->dai_link->dai_fmt;
	unsigned int mclk = 0;
	int div_bclk = 0, div_mclk = 0;
	int ret = 0;

	ret = snd_soc_dai_set_fmt(codec_dai, dai_fmt);
	if (ret < 0) {
		dev_err(codec_dai->dev, "fail to set format for codec dai\n");
		return ret;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, dai_fmt);
	if (ret < 0) {
		dev_err(cpu_dai->dev, "fail to set format for cpu dai\n");
		return ret;
	}

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
		mclk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
		mclk = 11289600;
		break;
	default:
		return -EINVAL;
		break;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, mclk, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(cpu_dai->dev, "fail to set sysclk for cpu dai\n");
		return ret;
	}

	div_bclk = 64;
	div_mclk = mclk / (params_rate(params) * div_bclk) - 1;

	snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_BCLK, div_bclk - 1);
	snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_MCLK, div_mclk);

	return 0;
}

static struct snd_soc_ops nau8540_ops = {
	.hw_params = nau8540_hw_params,
};

static struct snd_soc_dai_link nau8540_dai_link[] = {
	{
		.name = "nau8540",
		.stream_name = "nau8540 codec pcm",
		.codec_dai_name = "nau8540",
		.ops = &nau8540_ops,
	}
};

static struct snd_soc_aux_dev nau_aux_dev[] = {
	{
		.name = "nau8540b",
		.codec_name = "nau8540.1-001d",
	},
};

static struct snd_soc_codec_conf nau_codec_conf[] = {
	{
		.dev_name = "nau8540.1-001d",
		.name_prefix = "b",
	},
};

static struct snd_soc_card snd_soc_card_nau8540 = {
	.name = "RK_NAU8540",
	.dai_link = nau8540_dai_link,
	.num_links = ARRAY_SIZE(nau8540_dai_link),
	.aux_dev = nau_aux_dev,
	.num_aux_devs = ARRAY_SIZE(nau_aux_dev),
	.codec_conf = nau_codec_conf,
	.num_configs = ARRAY_SIZE(nau_codec_conf),
};

static int nau8540_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_card_nau8540;
	int ret = 0;

	card->dev = &pdev->dev;

	ret = rockchip_of_get_sound_card_info(card);
	if (ret < 0)
		return ret;

	return snd_soc_register_card(card);
}

static int nau8540_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	return snd_soc_unregister_card(card);
}

#ifdef CONFIG_OF
static const struct of_device_id nau8540_audio_of_match[] = {
	{ .compatible = "rockchip,nau8540-audio", },
	{},
};
MODULE_DEVICE_TABLE(of, nau8540_audio_of_match);
#endif

static struct platform_driver nau8540_audio_driver = {
	.driver = {
		.name = "nau8540-audio",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(nau8540_audio_of_match),
	},
	.probe = nau8540_audio_probe,
	.remove = nau8540_audio_remove,
};
module_platform_driver(nau8540_audio_driver);

MODULE_AUTHOR("Sugar Zhang <sugar.zhang@rock-chips.com>");
MODULE_DESCRIPTION("ROCKCHIP nau8540 machine ASoC driver");
MODULE_LICENSE("GPL v2");
