/*
 * Rockchip machine ASoC driver for boards using a rk3228 CODEC.
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

static int rk3228_hw_params(struct snd_pcm_substream *substream,
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
		dev_err(codec_dai->dev, "failed to set format for codec side\n");
		return ret;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, dai_fmt);
	if (ret < 0) {
		dev_err(cpu_dai->dev, "failed to set format for cpu side\n");
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
		dev_err(cpu_dai->dev, "failed to set sysclk for cpu dai\n");
		return ret;
	}

	div_bclk = 64;
	div_mclk = mclk / (params_rate(params) * div_bclk) - 1;

	snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_BCLK, div_bclk - 1);
	snd_soc_dai_set_clkdiv(cpu_dai, ROCKCHIP_DIV_MCLK, div_mclk);

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, mclk, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(codec_dai->dev, "failed to set sysclk for codec dai\n");
		return ret;
	}

	return 0;
}

static int rk3228_asoc_init(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

static struct snd_soc_ops rk3228_ops = {
	.hw_params = rk3228_hw_params,
};

static struct snd_soc_dai_link rk3228_dai_link[] = {
	{
		.name = "rk3228",
		.stream_name = "rk3228 codec pcm",
		.codec_dai_name = "rk3228-hifi",
		.init = rk3228_asoc_init,
		.ops = &rk3228_ops,
	}
};

static struct snd_soc_card snd_soc_card_rk3228 = {
	.name = "RK_RK3228",
	.dai_link = rk3228_dai_link,
	.num_links = 1,
};

static int rk3228_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_card_rk3228;
	int ret = 0;

	card->dev = &pdev->dev;

	ret = rockchip_of_get_sound_card_info(card);
	if (ret < 0)
		return ret;

	return snd_soc_register_card(card);
}

static int rk3228_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	return snd_soc_unregister_card(card);
}

#ifdef CONFIG_OF
static const struct of_device_id rk3228_audio_of_match[] = {
	{ .compatible = "rockchip,rk3228-audio", },
	{},
};
MODULE_DEVICE_TABLE(of, rk3228_audio_of_match);
#endif

static struct platform_driver rk3228_audio_driver = {
	.driver = {
		.name = "rk3228-audio",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rk3228_audio_of_match),
	},
	.probe = rk3228_audio_probe,
	.remove = rk3228_audio_remove,
};
module_platform_driver(rk3228_audio_driver);

MODULE_AUTHOR("Sugar Zhang <sugar.zhang@rock-chips.com>");
MODULE_DESCRIPTION("ROCKCHIP rk3228 machine ASoC driver");
MODULE_LICENSE("GPL v2");
