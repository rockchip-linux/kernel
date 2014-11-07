/* Copyright (c) 2010-2011,2013-2014 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * max98357a.c -- MAX98357A ALSA SoC Codec driver
 */

#include <linux/module.h>
#include <linux/gpio.h>
#include <sound/soc.h>

#define DRV_NAME "max98357a-codec"

static int max98357a_codec_daiops_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct gpio_desc *sdmode = snd_soc_dai_get_drvdata(dai);

	gpiod_set_value(sdmode, 1);

	return 0;
}

static void max98357a_codec_daiops_shutdown(
		struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct gpio_desc *sdmode = snd_soc_dai_get_drvdata(dai);

	gpiod_set_value(sdmode, 0);
}

static struct snd_soc_dai_ops max98357a_codec_dai_ops = {
	.startup	= max98357a_codec_daiops_startup,
	.shutdown	= max98357a_codec_daiops_shutdown,
};

static int max98357a_codec_dai_probe(struct snd_soc_dai *dai)
{
	struct gpio_desc *sdmode;

	sdmode = devm_gpiod_get(dai->dev, "sdmode");
	if (IS_ERR(sdmode)) {
		dev_err(dai->dev, "unable to get SDMODE GPIO\n");
		return PTR_ERR(sdmode);
	}
	gpiod_direction_output(sdmode, 0);
	snd_soc_dai_set_drvdata(dai, sdmode);

	return 0;
}

static struct snd_soc_dai_driver max98357a_codec_dai_driver = {
	.name = "max98357a-codec-dai",
	.playback = {
		.stream_name	= "max98357a-codec-playback",
		.formats	= SNDRV_PCM_FMTBIT_S16 |
					SNDRV_PCM_FMTBIT_S24 |
					SNDRV_PCM_FMTBIT_S32,
		.rates		= SNDRV_PCM_RATE_8000 |
					SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_48000 |
					SNDRV_PCM_RATE_96000,
		.rate_min	= 8000,
		.rate_max	= 96000,
		.channels_min	= 1,
		.channels_max	= 2,
	},
	.probe = &max98357a_codec_dai_probe,
	.ops = &max98357a_codec_dai_ops,
};

static int max98357a_codec_platform_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_codec_driver *codec_driver;

	codec_driver = devm_kzalloc(&pdev->dev,
			sizeof(struct snd_soc_codec_driver), GFP_KERNEL);
	if (!codec_driver)
		return -ENOMEM;

	ret = snd_soc_register_codec(&pdev->dev, codec_driver,
			&max98357a_codec_dai_driver, 1);
	if (ret)
		dev_err(&pdev->dev, "%s: error registering codec dais\n",
				__func__);

	return ret;
}

static int max98357a_codec_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static const struct of_device_id max98357a_codec_dt_match[] = {
	{ .compatible = "maxim,max98357a-codec", },
	{}
};

static struct platform_driver max98357a_codec_platform_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = max98357a_codec_dt_match,
	},
	.probe	= max98357a_codec_platform_probe,
	.remove = max98357a_codec_platform_remove,
};
module_platform_driver(max98357a_codec_platform_driver);

MODULE_DESCRIPTION("Maxim MAX98357A Codec Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, max98357a_codec_dt_match);
