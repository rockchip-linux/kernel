/*
 * Rockchip machine ASoC driver for boards using a MAX90809 CODEC.
 *
 * Copyright (c) 2014, ROCKCHIP CORPORATION.  All rights reserved.
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
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "rockchip_i2s.h"

#define DRV_NAME "rockchip-snd-max98090"

struct rk_mc_private {
	struct snd_soc_jack hp_jack;
	struct snd_soc_jack mic_jack;
};

static const struct snd_soc_dapm_widget rk_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

static const struct snd_soc_dapm_route rk_audio_map[] = {
	{"IN34", NULL, "Headset Mic"},
	{"IN34", NULL, "MICBIAS"},
	{"MICBIAS", NULL, "Headset Mic"},
	{"DMICL", NULL, "Int Mic"},
	{"Headphone", NULL, "HPL"},
	{"Headphone", NULL, "HPR"},
	{"Ext Spk", NULL, "SPKL"},
	{"Ext Spk", NULL, "SPKR"},
};

static const struct snd_kcontrol_new rk_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
};

static int rk_aif1_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int mclk;

	switch (params_rate(params)) {
	case 48000:
		mclk = 12288000;
		break;
	case 44100:
		mclk = 11289600;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, mclk,
				     SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Can't set codec clock %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Can't set codec clock %d\n", ret);
		return ret;
	}

	return ret;
}

static struct snd_soc_jack_pin hp_jack_pin = {
	.pin	= "Headphone",
	.mask	= SND_JACK_HEADPHONE,
};

static struct snd_soc_jack_pin mic_jack_pin = {
	.pin	= "Headset Mic",
	.mask	= SND_JACK_MICROPHONE,
};

static struct snd_soc_jack_gpio hp_jack_gpio = {
	.name			= "hp-gpio",
	.report			= SND_JACK_HEADPHONE,
	.debounce_time		= 200,
	.invert			= 0,
};

static struct snd_soc_jack_gpio mic_jack_gpio = {
	.name			= "mic-gpio",
	.report			= SND_JACK_MICROPHONE,
	.debounce_time		= 200,
	.invert			= 1,
};

static int rk_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret = 0;
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = runtime->card;
	struct rk_mc_private *drv = snd_soc_card_get_drvdata(card);
	struct snd_soc_jack *hp_jack = &drv->hp_jack;
	struct snd_soc_jack *mic_jack = &drv->mic_jack;

	card->dapm.idle_bias_off = true;

	snd_soc_dapm_enable_pin(dapm, "Headset Mic");
	snd_soc_dapm_enable_pin(dapm, "Headphone");
	snd_soc_dapm_enable_pin(dapm, "Ext Spk");
	snd_soc_dapm_enable_pin(dapm, "Int Mic");

	snd_soc_dapm_sync(dapm);

	/* Enable headphone jack detection */
	ret = snd_soc_jack_new(codec, "Headphone Jack", SND_JACK_HEADPHONE,
			       hp_jack);
	if (ret)
		return ret;

	ret = snd_soc_jack_add_pins(hp_jack, 1, &hp_jack_pin);
	if (ret)
		return ret;

	ret = snd_soc_jack_add_gpios(hp_jack, 1, &hp_jack_gpio);
	if (ret)
		return ret;

	/* Enable mic jack detection */
	ret = snd_soc_jack_new(codec, "Mic Jack", SND_JACK_MICROPHONE,
			       mic_jack);
	if (ret)
		return ret;

	ret = snd_soc_jack_add_pins(mic_jack, 1, &mic_jack_pin);
	if (ret)
		return ret;

	ret = snd_soc_jack_add_gpios(mic_jack, 1, &mic_jack_gpio);
	if (ret)
		return ret;

	return ret;
}

static struct snd_soc_ops rk_aif1_ops = {
	.hw_params = rk_aif1_hw_params,
};

static struct snd_soc_dai_link rk_dailink = {
	.name = "max98090",
	.stream_name = "Audio",
	.codec_dai_name = "HiFi",
	.init = rk_init,
	.ops = &rk_aif1_ops,
	/* set max98090 as slave */
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBS_CFS,
};

static struct snd_soc_card snd_soc_card_rk = {
	.name = "ROCKCHIP-I2S",
	.dai_link = &rk_dailink,
	.num_links = 1,
	.dapm_widgets = rk_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rk_dapm_widgets),
	.dapm_routes = rk_audio_map,
	.num_dapm_routes = ARRAY_SIZE(rk_audio_map),
	.controls = rk_mc_controls,
	.num_controls = ARRAY_SIZE(rk_mc_controls),
};

#ifdef CONFIG_PM_SLEEP
static int snd_rk_prepare(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct rk_mc_private *drv = snd_soc_card_get_drvdata(card);

	snd_soc_jack_free_gpios(&drv->hp_jack, 1, &hp_jack_gpio);
	snd_soc_jack_free_gpios(&drv->mic_jack, 1, &mic_jack_gpio);

	return snd_soc_suspend(dev);
}

static void snd_rk_complete(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct rk_mc_private *drv = snd_soc_card_get_drvdata(card);

	snd_soc_jack_add_gpios(&drv->hp_jack, 1, &hp_jack_gpio);
	snd_soc_jack_add_gpios(&drv->mic_jack, 1, &mic_jack_gpio);

	snd_soc_resume(dev);
}

static const struct dev_pm_ops rk_max98090_pm_ops = {
	.prepare = snd_rk_prepare,
	.complete = snd_rk_complete,
};

#define RK_MAX98090_PM_OPS	(&rk_max98090_pm_ops)
#else
#define RK_MAX98090_PM_OPS	NULL
#endif

static int snd_rk_mc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct rk_mc_private *drv;
	struct snd_soc_card *card = &snd_soc_card_rk;
	struct device_node *np = pdev->dev.of_node;

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	hp_jack_gpio.gpio = of_get_named_gpio(np, "rockchip,hp-det-gpios", 0);
	if (hp_jack_gpio.gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	mic_jack_gpio.gpio = of_get_named_gpio(np, "rockchip,mic-det-gpios", 0);
	if (mic_jack_gpio.gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	/* register the soc card */
	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, drv);

	rk_dailink.codec_of_node = of_parse_phandle(np,
			"rockchip,audio-codec", 0);
	if (!rk_dailink.codec_of_node) {
		dev_err(&pdev->dev,
			"Property 'rockchip,audio-codec' missing or invalid\n");
		return -EINVAL;
	}

	rk_dailink.cpu_of_node = of_parse_phandle(np,
			"rockchip,i2s-controller", 0);
	if (!rk_dailink.cpu_of_node) {
		dev_err(&pdev->dev,
			"Property 'rockchip,i2s-controller' missing or invalid\n");
		return -EINVAL;
	}

	rk_dailink.platform_of_node = rk_dailink.cpu_of_node;

	ret = snd_soc_register_card(card);
	if (ret) {
		pr_err("snd_soc_register_card failed %d\n", ret);
		return ret;
	}
	platform_set_drvdata(pdev, &card);

	ret = snd_soc_of_parse_card_name(card, "rockchip,model");
	if (ret)
		return ret;

	return ret;
}

static int snd_rk_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct rk_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	snd_soc_jack_free_gpios(&drv->hp_jack, 1, &hp_jack_gpio);
	snd_soc_jack_free_gpios(&drv->mic_jack, 1, &mic_jack_gpio);

	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id rockchip_max98090_of_match[] = {
	{ .compatible = "rockchip,rockchip-audio-max98090", },
	{},
};

static struct platform_driver snd_rk_mc_driver = {
	.probe = snd_rk_mc_probe,
	.remove = snd_rk_mc_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = RK_MAX98090_PM_OPS,
		.of_match_table = rockchip_max98090_of_match,
	},
};

module_platform_driver(snd_rk_mc_driver);
MODULE_AUTHOR("jianqun <jay.xu@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip max98090 machine ASoC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, rockchip_max98090_of_match);
