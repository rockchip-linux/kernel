/*
 * Rockchip machine ASoC driver for boards using a RT5640 CODEC.
 *
 * Copyright (c) 2015, ROCKCHIP CORPORATION.  All rights reserved.
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
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "rockchip_i2s.h"

#define DRV_NAME "rockchip-snd-rt5640"

static void rt5640_headphone_detect(void);
static void rt5640_mic_detect(void);

static struct gpio_desc *hpdet_gpiod;
static struct gpio_desc *micdet_gpiod;

static struct delayed_work hp_detect_work;
static struct delayed_work mic_detect_work;

static struct snd_soc_jack headphone_jack;
static struct snd_soc_jack mic_jack;

struct snd_soc_codec *g_mc_codec;

static const struct snd_soc_dapm_widget rk_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_SPK("Speakers", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
};

static const struct snd_soc_dapm_route rk_audio_map[] = {
	/* Input Lines */
	{"IN2P", NULL, "Int Mic"},
	{"IN2P", NULL, "Int Mic"},
	{"Int Mic", NULL, "MICBIAS1"},

	/* Output Lines */
	{"Headphones", NULL, "HPOL"},
	{"Headphones", NULL, "HPOR"},
	{"Speakers", NULL, "SPORP"},
	{"Speakers", NULL, "SPORN"},
	{"Speakers", NULL, "SPOLP"},
	{"Speakers", NULL, "SPOLN"},
};

static const struct snd_kcontrol_new rk_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphones"),
	SOC_DAPM_PIN_SWITCH("Speakers"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
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
	case 8000:
	case 16000:
	case 48000:
	case 96000:
		mclk = 12288000;
		break;
	case 44100:
		mclk = 11289600;
		break;
	default:
		return -EINVAL;
	}

	printk("%s -- line = %d, mclk = %d hz, params_rate = %d hz\n",
			__func__, __LINE__, mclk, params_rate(params));

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

static int rk_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_codec *codec = runtime->codec;
	int ret;

	/* Enable Headset and 4 Buttons Jack detection */
	ret = snd_soc_jack_new(codec, "Headphone Jack",
			       SND_JACK_HEADPHONE,
			       &headphone_jack);

	if (ret) {
		dev_err(codec->dev, "New Headphone Jack failed! (%d)\n", ret);
		return ret;
	}

	ret = snd_soc_jack_new(codec, "Mic Jack",
			       SND_JACK_MICROPHONE,
			       &mic_jack);

	if (ret) {
		dev_err(codec->dev, "New Mic Jack failed! (%d)\n", ret);
		return ret;
	}

	/* just enable speaker default */
	snd_soc_dapm_disable_pin(&codec->dapm, "Headphones");
	snd_soc_dapm_enable_pin(&codec->dapm, "Speakers");
	snd_soc_dapm_enable_pin(&codec->dapm, "Int Mic");
	snd_soc_dapm_sync(&codec->dapm);

	g_mc_codec = codec;

	/* detect headphone and mic startup */
	rt5640_headphone_detect();
	rt5640_mic_detect();

	return 0;
}

static struct snd_soc_ops rk_aif1_ops = {
	.hw_params = rk_aif1_hw_params,
};

static struct snd_soc_dai_link rk_dailink = {
	.name = "rt5640",
	.stream_name = "rt5640 PCM",
	.codec_dai_name = "rt5640-aif1",
	.init = rk_init,
	.ops = &rk_aif1_ops,
	/* set rt5640 as slave */
	.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBS_CFS,
};

static struct snd_soc_card snd_soc_card_rk = {
	// .name = "I2S-RT5640",
	.name = "DAISY-I2S",
	.owner = THIS_MODULE,
	.dai_link = &rk_dailink,
	.num_links = 1,
	.dapm_widgets = rk_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rk_dapm_widgets),
	.dapm_routes = rk_audio_map,
	.num_dapm_routes = ARRAY_SIZE(rk_audio_map),
	.controls = rk_mc_controls,
	.num_controls = ARRAY_SIZE(rk_mc_controls),
};

static void rt5640_headphone_detect(void)
{
	int level = gpiod_get_value(hpdet_gpiod);
	struct snd_soc_codec *codec = g_mc_codec;
	
	printk("%s -- line = %d, level = %d\n", __func__, __LINE__, level);

	if (level) {
		snd_soc_dapm_disable_pin(&codec->dapm, "Speakers");
		snd_soc_dapm_sync(&codec->dapm);
		snd_soc_dapm_enable_pin(&codec->dapm, "Headphones");
	} else {
		snd_soc_dapm_disable_pin(&codec->dapm, "Headphones");
		snd_soc_dapm_sync(&codec->dapm);
		snd_soc_dapm_enable_pin(&codec->dapm, "Speakers");
	}

	snd_soc_dapm_sync(&codec->dapm);

}

static void rt5640_mic_detect(void)
{
	int level;
	struct snd_soc_codec *codec = g_mc_codec;

	if (IS_ERR(micdet_gpiod))
		return;

	level = gpiod_get_value(micdet_gpiod);

	printk("%s -- line = %d, level = %d\n", __func__, __LINE__, level);

	if (level) {
		snd_soc_dapm_enable_pin(&codec->dapm, "Int Mic");
	} else {
		snd_soc_dapm_disable_pin(&codec->dapm, "Int Mic");
	}

	snd_soc_dapm_sync(&codec->dapm);
}

static void rt5640_headphone_detect_work(struct work_struct *work)
{
	rt5640_headphone_detect();
}

static void rt5640_mic_detect_work(struct work_struct *work)
{
	rt5640_mic_detect();
}

static irqreturn_t headphone_detect_irq(int irq, void *dev_id)
{
	queue_delayed_work(system_power_efficient_wq,
			   &hp_detect_work, msecs_to_jiffies(100));

	return IRQ_HANDLED;
}

static irqreturn_t mic_detect_irq(int irq, void *dev_id)
{
	queue_delayed_work(system_power_efficient_wq,
			   &mic_detect_work, msecs_to_jiffies(100));

	return IRQ_HANDLED;
}

static int snd_rk_mc_gpio_init(struct platform_device *pdev)
{
	int ret;

	hpdet_gpiod = devm_gpiod_get(&pdev->dev, "hpdet");

	if (IS_ERR(hpdet_gpiod)) {
		dev_err(&pdev->dev,
			"%s() Can not read property hpdet-gpios\n", __func__);
		goto do_err;
	} else {
		ret = gpiod_direction_input(hpdet_gpiod);
		if (ret < 0) {
			pr_err("%s() gpio_direction_input hpdet set ERROR\n", __func__);
			goto do_err;
		}

		printk("request_threaded_irq hpdet_gpiod ok\n");
	}

	micdet_gpiod = devm_gpiod_get(&pdev->dev, "micdet");

	if (IS_ERR(micdet_gpiod)) {
		dev_err(&pdev->dev,
			"%s() Can not read property micdet-gpios\n", __func__);
		goto do_err;
	} else {
		ret = gpiod_direction_input(micdet_gpiod);
		if (ret < 0) {
			pr_err("%s() gpio_direction_input micdet set ERROR\n", __func__);
			goto do_err;
		}

		printk("request_threaded_irq micdet_gpiod ok\n");
	}

	return 0;

do_err:
	return -EINVAL;
}

static int snd_rk_mc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct snd_soc_card *card = &snd_soc_card_rk;
	struct device_node *np = pdev->dev.of_node;

	/* register the soc card */
	card->dev = &pdev->dev;

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

	snd_rk_mc_gpio_init(pdev);

	ret = snd_soc_of_parse_card_name(card, "rockchip,model");
	if (ret) {
		dev_err(&pdev->dev,
			"Soc parse card name failed %d\n", ret);
		return ret;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err(&pdev->dev,
			"Soc register card failed %d\n", ret);
		return ret;
	}

	INIT_DELAYED_WORK(&hp_detect_work, rt5640_headphone_detect_work);
	INIT_DELAYED_WORK(&mic_detect_work, rt5640_mic_detect_work);

	ret = request_threaded_irq(gpiod_to_irq(hpdet_gpiod), NULL, headphone_detect_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
		| IRQF_ONESHOT, "hp_irq", NULL);
	if (ret) {
		pr_err("Failed to reguest IRQ: %d\n", ret);
		goto do_err;
	}

	if (!IS_ERR(micdet_gpiod)) {
		ret = request_threaded_irq(gpiod_to_irq(micdet_gpiod), NULL, mic_detect_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
			| IRQF_ONESHOT, "mic_irq", NULL);
		if (ret) {
			pr_err("Failed to reguest IRQ: %d\n", ret);
			goto do_err;
		}
	}
do_err:
	return ret;
}

static const struct of_device_id rockchip_rt5640_of_match[] = {
	{ .compatible = "rockchip,rockchip-audio-rt5640", },
	{},
};

MODULE_DEVICE_TABLE(of, rockchip_rt5640_of_match);

static struct platform_driver snd_rk_mc_driver = {
	.probe = snd_rk_mc_probe,
	.driver = {
		.name = DRV_NAME,
		.pm = &snd_soc_pm_ops,
		.of_match_table = rockchip_rt5640_of_match,
	},
};

module_platform_driver(snd_rk_mc_driver);

MODULE_AUTHOR("Xing Zheng <zhengxing@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip rt5640 machine ASoC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
