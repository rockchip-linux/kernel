/*
 * Intel Baytrail SST MAX98090 machine driver
 * Copyright (c) 2014, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../codecs/max98090.h"
#include "sst-dsp.h"
#include "sst-baytrail-ipc.h"

#define BYT_PLAT_CLK_3_HZ      25000000

struct byt_mc_private {
	struct snd_soc_jack hp_jack;
	struct snd_soc_jack mic_jack;
};

static const struct snd_soc_dapm_widget byt_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

static const struct snd_soc_dapm_route byt_audio_map[] = {
	{"IN34", NULL, "Headset Mic"},
	{"IN34", NULL, "MICBIAS"},
	{"MICBIAS", NULL, "Headset Mic"},
	{"DMICL", NULL, "Int Mic"},
	{"Headphone", NULL, "HPL"},
	{"Headphone", NULL, "HPR"},
	{"Ext Spk", NULL, "SPKL"},
	{"Ext Spk", NULL, "SPKR"},
};

static const struct snd_kcontrol_new byt_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
};

static int byt_aif1_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	pr_debug("Enter:%s", __func__);

	/*
	 * The particular clock id specified below does not matter since the
	 * max98090 driver ignores it.
	 */
	ret = snd_soc_dai_set_sysclk(codec_dai, M98090_REG_SYSTEM_CLOCK,
				     BYT_PLAT_CLK_3_HZ,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Can't set codec clock %d\n", ret);
		return ret;
	}

	return 0;
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
};

static struct snd_soc_jack_gpio mic_jack_gpio = {
	.name			= "mic-gpio",
	.report			= SND_JACK_MICROPHONE,
	.debounce_time		= 200,
	.invert			= 1,
};

/* enable the codec SHDN after LRCLK and BCLK have been activated by DSP FW */
static void byt_max98090_start(struct sst_dsp *dsp, void *data)
{
	struct snd_soc_pcm_runtime *runtime = data;
	struct snd_soc_codec *codec = runtime->codec;

	snd_soc_update_bits(codec, M98090_REG_DEVICE_SHUTDOWN,
				M98090_SHDNN_MASK, M98090_SHDNN_MASK);
}

static void byt_max98090_stop(struct sst_dsp *dsp, void *data)
{
	struct snd_soc_pcm_runtime *runtime = data;
	struct snd_soc_codec *codec = runtime->codec;

	snd_soc_update_bits(codec, M98090_REG_DEVICE_SHUTDOWN,
					M98090_SHDNN_MASK, 0);
}

static int byt_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_platform *platform = runtime->platform;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = runtime->card;
	struct byt_mc_private *drv = snd_soc_card_get_drvdata(card);
	struct sst_pdata *pdata = dev_get_platdata(platform->dev);
	struct snd_soc_jack *hp_jack = &drv->hp_jack;
	struct snd_soc_jack *mic_jack = &drv->mic_jack;

	pr_debug("Enter:%s", __func__);
	sst_byt_register_notifier(platform->dev, pdata,
		byt_max98090_start, byt_max98090_stop, runtime);

	card->dapm.idle_bias_off = true;

	ret = snd_soc_add_card_controls(card, byt_mc_controls,
					ARRAY_SIZE(byt_mc_controls));
	if (ret) {
		pr_err("unable to add card controls\n");
		return ret;
	}

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

	snd_soc_update_bits(codec, M98090_REG_INTERRUPT_S, M98090_IJDET_MASK,
				1 << M98090_IJDET_SHIFT);

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

	return ret;
}

static struct snd_soc_ops byt_aif1_ops = {
	.hw_params = byt_aif1_hw_params,
};

static struct snd_soc_dai_link byt_dailink[] = {
	{
		.name = "Baytrail Audio",
		.stream_name = "Audio",
		.cpu_dai_name = "Front-cpu-dai",
		.codec_dai_name = "HiFi",
		.codec_name = "max98090.1-0010",
		.platform_name = "baytrail-pcm-audio",
		.init = byt_init,
		.ops = &byt_aif1_ops,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	},
	{
		.name = "Baytrail Voice",
		.stream_name = "Voice",
		.cpu_dai_name = "Front-cpu-dai",
		.codec_dai_name = "HiFi",
		.codec_name = "max98090.1-0010",
		.platform_name = "baytrail-pcm-audio",
		.init = NULL,
		.ops = &byt_aif1_ops,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	},
};

static struct snd_soc_card snd_soc_card_byt = {
	.name = "byt-max98090",
	.dai_link = byt_dailink,
	.num_links = ARRAY_SIZE(byt_dailink),
	.dapm_widgets = byt_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(byt_dapm_widgets),
	.dapm_routes = byt_audio_map,
	.num_dapm_routes = ARRAY_SIZE(byt_audio_map),
	.controls = byt_mc_controls,
	.num_controls = ARRAY_SIZE(byt_mc_controls),
};

#ifdef CONFIG_PM_SLEEP
static int snd_byt_prepare(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct byt_mc_private *drv = snd_soc_card_get_drvdata(card);

	snd_soc_jack_free_gpios(&drv->hp_jack, 1, &hp_jack_gpio);
	snd_soc_jack_free_gpios(&drv->mic_jack, 1, &mic_jack_gpio);

	return snd_soc_suspend(dev);
}

static void snd_byt_complete(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct byt_mc_private *drv = snd_soc_card_get_drvdata(card);

	snd_soc_jack_add_gpios(&drv->hp_jack, 1, &hp_jack_gpio);
	snd_soc_jack_add_gpios(&drv->mic_jack, 1, &mic_jack_gpio);

	snd_soc_resume(dev);
}

static const struct dev_pm_ops byt_max98090_pm_ops = {
	.prepare = snd_byt_prepare,
	.complete = snd_byt_complete,
};

#define BYT_MAX98090_PM_OPS	(&byt_max98090_pm_ops)
#else
#define BYT_MAX98090_PM_OPS	NULL
#endif

static int snd_byt_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct byt_mc_private *drv;
	struct gpio_desc *mic_desc;
	struct gpio_desc *hp_desc;

	pr_debug("Entry %s\n", __func__);

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_ATOMIC);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}

	/*
	 * ASoC still uses legacy GPIOs so we look both GPIOs using
	 * descriptors here, convert them to numbers and release the
	 * acquired descriptors. Once ASoC switches over to GPIO descriptor
	 * APIs we can pass them directly.
	 */
	hp_desc = gpiod_get_index(pdev->dev.parent, NULL, 0);
	if (IS_ERR(hp_desc)) {
		pr_err("unable to get hp-gpio\n");
		return PTR_ERR(hp_desc);
	}
	mic_desc = gpiod_get_index(pdev->dev.parent, NULL, 1);
	if (IS_ERR(mic_desc)) {
		pr_err("unable to get mic-gpio\n");
		gpiod_put(hp_desc);
		return PTR_ERR(mic_desc);
	}

	hp_jack_gpio.gpio = desc_to_gpio(hp_desc);
	mic_jack_gpio.gpio = desc_to_gpio(mic_desc);

	gpiod_put(mic_desc);
	gpiod_put(hp_desc);

	/* register the soc card */
	snd_soc_card_byt.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_byt, drv);
	ret_val = snd_soc_register_card(&snd_soc_card_byt);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		return ret_val;
	}
	platform_set_drvdata(pdev, &snd_soc_card_byt);
	pr_info("%s successful\n", __func__);
	return ret_val;
}

static int snd_byt_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct byt_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	snd_soc_jack_free_gpios(&drv->hp_jack, 1, &hp_jack_gpio);
	snd_soc_jack_free_gpios(&drv->mic_jack, 1, &mic_jack_gpio);

	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver snd_byt_mc_driver = {
	.probe = snd_byt_mc_probe,
	.remove = snd_byt_mc_remove,
	.driver = {
		.name = "byt-max98090",
		.owner = THIS_MODULE,
		.pm = BYT_MAX98090_PM_OPS,
	},
};
module_platform_driver(snd_byt_mc_driver)

MODULE_DESCRIPTION("ASoC Intel(R) Baytrail Machine driver");
MODULE_AUTHOR("Omair Md Abdullah, Jarkko Nikula");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:byt-max98090");
