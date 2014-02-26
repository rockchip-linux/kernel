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
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../codecs/max98090.h"

struct byt_mc_private {
	struct snd_soc_jack jack;
};

static inline struct snd_soc_codec *byt_get_codec(struct snd_soc_card *card)
{
	bool found = false;
	struct snd_soc_codec *codec;

	list_for_each_entry(codec, &card->codec_dev_list, card_list) {
		if (!strstr(codec->name, "max98090.1-0010")) {
			pr_debug("codec was %s", codec->name);
			continue;
		} else {
			found = true;
			break;
		}
	}
	if (found == false) {
		pr_err("%s: cant find codec", __func__);
		return NULL;
	}
	return codec;
}

static int platform_clock_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_codec *codec;

	codec = byt_get_codec(card);
	if (!codec) {
		pr_err("Codec not found; Unable to set platform clock\n");
		return -EIO;
	}
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pr_debug("Platform clk turned ON\n");
		/*
		 * The particular clock id specified here does not really
		 * matter since the driver ignores this paramter.
		 */
		snd_soc_codec_set_sysclk(codec, M98090_REG_SYSTEM_CLOCK,
				0, 25000000, SND_SOC_CLOCK_IN);
	} else {
		/* Set codec clock source to internal clock before
		   turning off the platform clock. Codec needs clock
		   for Jack detection and button press */
		/*
		 * The particular clock id specified here does not really
		 * matter since the driver ignores this paramter.
		 */
		snd_soc_codec_set_sysclk(codec, M98090_REG_SYSTEM_CLOCK,
				0, 0, SND_SOC_CLOCK_IN);
		pr_debug("Platform clk turned OFF\n");
	}

	return 0;
}

static const struct snd_soc_dapm_widget byt_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			platform_clock_control, SND_SOC_DAPM_PRE_PMU|
			SND_SOC_DAPM_POST_PMD),
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

	{"Headphone", NULL, "Platform Clock"},
	{"Headset Mic", NULL, "Platform Clock"},
	{"Int Mic", NULL, "Platform Clock"},
	{"Ext Spk", NULL, "Platform Clock"},
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
	unsigned int fmt;
	int ret;

	pr_debug("Enter:%s", __func__);
	/* I2S Slave Mode`*/
	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	      SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	/*
	 * The particular clock id specified below does not matter since the
	 * max98090 driver ignores it.
	 */
	ret = snd_soc_dai_set_sysclk(codec_dai, M98090_REG_SYSTEM_CLOCK,
				     params_rate(params) * 256,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(codec_dai->dev, "Can't set codec clock %d\n", ret);
		return ret;
	}
	/*
	 * TODO:  The max98090 driver does not implement the set_pll op.  So I
	 * strongly doubt that we actually want call the following.  The core
	 * would return EINVAL.  For now the workaround is to print the error
	 * but ignore it instead of returning it.
	 */
	ret = snd_soc_dai_set_pll(codec_dai, 0, M98090_REG_SYSTEM_CLOCK,
				  params_rate(params) * 64,
				  params_rate(params) * 256);
	if (ret < 0) {
		pr_err("can't set codec pll: %d\n", ret);
		pr_err("Ignoring the error and proceeding anyway\n");
	}
	return 0;
}

static int byt_hp_jack_status_check(void)
{
	int spk_enable;
	int report;

	spk_enable = gpio_get_value_cansleep(CONFIG_SND_BYT_RAMBI_HPDET_GPIO);

	if (spk_enable) {
		pr_debug("Headphone Insert\n");
		report = SND_JACK_HEADPHONE;
	} else {
		pr_debug("Headphone Remove\n");
		report = SND_JACK_LINEOUT;
	}

	return report;
}

static int byt_mic_jack_status_check(void)
{
	int mic_enable;
	int report;

	mic_enable = !gpio_get_value_cansleep(CONFIG_SND_BYT_RAMBI_MICDET_GPIO);

	if (mic_enable) {
		pr_debug("Headset Mic Insert\n");
		report = SND_JACK_MICROPHONE;
	} else {
		pr_debug("Headset Mic Remove\n");
		report = SND_JACK_LINEIN;
	}

	return report;
}

static struct snd_soc_jack_pin hs_jack_pins[] = {
	{
		.pin	= "Headphone",
		.mask	= SND_JACK_HEADPHONE,
	},
	{
		.pin	= "Headset Mic",
		.mask	= SND_JACK_MICROPHONE,
	},
	{
		.pin	= "Ext Spk",
		.mask	= SND_JACK_LINEOUT,
	},
	{
		.pin	= "Int Mic",
		.mask	= SND_JACK_LINEIN,
	},
};

static struct snd_soc_jack_gpio hs_jack_gpios[] = {
	{
		.name			= "hp-gpio",
		.report			= SND_JACK_HEADPHONE | SND_JACK_LINEOUT,
		.debounce_time		= 200,
		.gpio			= CONFIG_SND_BYT_RAMBI_HPDET_GPIO,
		.jack_status_check	= &byt_hp_jack_status_check,
	},
	{
		.name			= "mic-gpio",
		.report			= SND_JACK_MICROPHONE | SND_JACK_LINEIN,
		.debounce_time		= 200,
		.gpio			= CONFIG_SND_BYT_RAMBI_MICDET_GPIO,
		.jack_status_check	= &byt_mic_jack_status_check,
	},
};

static int byt_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = runtime->card;
	struct byt_mc_private *drv = snd_soc_card_get_drvdata(card);
	struct snd_soc_jack *jack = &drv->jack;

	pr_debug("Enter:%s", __func__);
	card->dapm.idle_bias_off = true;

	ret = snd_soc_add_card_controls(card, byt_mc_controls,
					ARRAY_SIZE(byt_mc_controls));
	if (ret) {
		pr_err("unable to add card controls\n");
		return ret;
	}

	/* Keep the voice call paths active during
	suspend. Mark the end points ignore_suspend */
	/*TODO: CHECK this */
	snd_soc_dapm_ignore_suspend(dapm, "HPL");
	snd_soc_dapm_ignore_suspend(dapm, "HPR");

	snd_soc_dapm_ignore_suspend(dapm, "SPKL");
	snd_soc_dapm_ignore_suspend(dapm, "SPKR");

	snd_soc_dapm_enable_pin(dapm, "Headset Mic");
	snd_soc_dapm_enable_pin(dapm, "Headphone");
	snd_soc_dapm_enable_pin(dapm, "Ext Spk");
	snd_soc_dapm_enable_pin(dapm, "Int Mic");

	snd_soc_dapm_sync(dapm);


	/* Enable jack detection */
	ret = snd_soc_jack_new(codec, "Headphone", SND_JACK_HEADPHONE, jack);
	if (ret)
		return ret;

	ret = snd_soc_jack_add_pins(jack, ARRAY_SIZE(hs_jack_pins),
					hs_jack_pins);
	if (ret)
		return ret;

	snd_soc_update_bits(codec, M98090_REG_INTERRUPT_S, M98090_IJDET_MASK,
				1 << M98090_IJDET_SHIFT);

	snd_soc_jack_report(jack, SND_JACK_LINEOUT | SND_JACK_LINEIN,
				SND_JACK_HEADSET | SND_JACK_LINEOUT |
				SND_JACK_LINEIN);

	ret = snd_soc_jack_add_gpios(jack, ARRAY_SIZE(hs_jack_gpios),
					hs_jack_gpios);

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
		.ignore_suspend = 1,
		.ops = &byt_aif1_ops,
	},
	{
		.name = "Baytrail Voice",
		.stream_name = "Voice",
		.cpu_dai_name = "Mic1-cpu-dai",
		.codec_dai_name = "HiFi",
		.codec_name = "max98090.1-0010",
		.platform_name = "baytrail-pcm-audio",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &byt_aif1_ops,
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

static int snd_byt_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct byt_mc_private *drv;

	pr_debug("Entry %s\n", __func__);

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_ATOMIC);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}

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
	snd_soc_jack_free_gpios(&drv->jack, ARRAY_SIZE(hs_jack_gpios),
				hs_jack_gpios);

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
	},
};
module_platform_driver(snd_byt_mc_driver)

MODULE_DESCRIPTION("ASoC Intel(R) Baytrail Machine driver");
MODULE_AUTHOR("Omair Md Abdullah, Jarkko Nikula");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:byt-max98090");
