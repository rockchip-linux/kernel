/*
 * Rockchip machine ASoC driver for boards using PDM Micarray.
 *
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd All rights reserved.
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
#include "rk_pdm.h"

static int rkpdm_micarray_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int dai_fmt = rtd->dai_link->dai_fmt;

	return snd_soc_dai_set_fmt(cpu_dai, dai_fmt);
}

static struct snd_soc_ops rkpdm_micarray_ops = {
	.hw_params = rkpdm_micarray_hw_params,
};

static struct snd_soc_dai_link rkpdm_micarray_dai_link[] = {
	{
		.name = "rk-pdm-micarray",
		.stream_name = "rk-pdm-micarray",
		.codec_dai_name = "dummy_codec",
		.ops = &rkpdm_micarray_ops,
	}
};

static struct snd_soc_card snd_soc_card_rkpdm_micarray = {
	.name = "RK_PDM_MICARRAY",
	.dai_link = rkpdm_micarray_dai_link,
	.num_links = 1,
};

static int rkpdm_micarray_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_card_rkpdm_micarray;
	int ret = 0;

	card->dev = &pdev->dev;

	ret = rockchip_of_get_sound_card_info(card);
	if (ret < 0)
		return ret;

	return snd_soc_register_card(card);
}

static int rkpdm_micarray_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	return snd_soc_unregister_card(card);
}

static const struct of_device_id rkpdm_micarray_audio_of_match[] = {
	{ .compatible = "rockchip,pdm-micarray", },
	{},
};
MODULE_DEVICE_TABLE(of, rkpdm_micarray_audio_of_match);

static struct platform_driver rkpdm_micarray_audio_driver = {
	.driver = {
		.name = "rkpdm-micarray",
		.of_match_table = of_match_ptr(rkpdm_micarray_audio_of_match),
	},
	.probe = rkpdm_micarray_audio_probe,
	.remove = rkpdm_micarray_audio_remove,
};
module_platform_driver(rkpdm_micarray_audio_driver);

MODULE_AUTHOR("Sugar Zhang <sugar.zhang@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip PDM Micarray Machine ASoC Driver");
MODULE_LICENSE("GPL v2");
