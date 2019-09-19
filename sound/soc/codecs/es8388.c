/* SPDX-License-Identifier: GPL-2.0 */
/*
 * es8388.c -- es8388 ALSA SoC audio driver
 * base on sound/soc/codecs/es8323.c
 *
 * Copyright (c) 2018 Rockchip Electronics Co. Ltd.
 *
 * Author: Mark Brown <will@everset-semi.com>
 * Author: Xiaotan Luo <lxt@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include "es8388.h"

#define INVALID_GPIO -1

#define ES8388_CODEC_SET_SPK	1
#define ES8388_CODEC_SET_HP	2

#define es8388_DEF_VOL	0x1b

static int es8388_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level);

/*
 * es8388 register cache
 * We can't read the es8388 register space when we
 * are using 2 wire for device control, so we cache them instead.
 */
static u16 es8388_reg[] = {
	0x06, 0x1C, 0xC3, 0xFC,	/*  0 */
	0xC0, 0x00, 0x00, 0x7C,	/*  4 */
	0x80, 0x00, 0x00, 0x06,	/*  8 */
	0x00, 0x06, 0x30, 0x30,	/* 12 */
	0xC0, 0xC0, 0x38, 0xB0,	/* 16 */
	0x32, 0x06, 0x00, 0x00,	/* 20 */
	0x06, 0x30, 0xC0, 0xC0,	/* 24 */
	0x08, 0x06, 0x1F, 0xF7,	/* 28 */
	0xFD, 0xFF, 0x1F, 0xF7,	/* 32 */
	0xFD, 0xFF, 0x00, 0x38,	/* 36 */
	0x38, 0x38, 0x38, 0x38,	/* 40 */
	0x38, 0x00, 0x00, 0x00,	/* 44 */
	0x00, 0x00, 0x00, 0x00,	/* 48 */
	0x00, 0x00, 0x00, 0x00,	/* 52 */
};

/* codec private data */
struct es8388_priv {
	unsigned int sysclk;
	struct clk *mclk;
	struct snd_pcm_hw_constraint_list *sysclk_constraints;

	int spk_ctl_gpio;
	int hp_det_gpio;

	bool muted;
	bool hp_inserted;
	bool spk_gpio_level;
	bool hp_det_level;
};

static struct es8388_priv *es8388_private;
static int es8388_set_gpio(int gpio, bool level)
{
	struct es8388_priv *es8388 = es8388_private;

	if (!es8388)
		return 0;

	if ((gpio & ES8388_CODEC_SET_SPK) && es8388 &&
	    es8388->spk_ctl_gpio != INVALID_GPIO) {
		gpio_set_value(es8388->spk_ctl_gpio, level);
	}

	return 0;
}

static irqreturn_t hp_det_irq_handler(int irq, void *dev_id)
{
	struct es8388_priv *es8388 = es8388_private;

	if (gpio_get_value(es8388->hp_det_gpio))
		es8388->hp_inserted = 0;
	else
		es8388->hp_inserted = 1;

	if (es8388->muted == 0) {
		if (es8388->hp_det_level != es8388->hp_inserted)
			es8388_set_gpio(ES8388_CODEC_SET_SPK,
					!es8388->spk_gpio_level);
		else
			es8388_set_gpio(ES8388_CODEC_SET_SPK,
					es8388->spk_gpio_level);
	}
	return IRQ_HANDLED;
}

static unsigned int es8388_read_reg_cache(struct snd_soc_codec *codec,
					  unsigned int reg)
{
	if (reg >= ARRAY_SIZE(es8388_reg))
		return -1;
	return es8388_reg[reg];
}

static int es8388_write(struct snd_soc_codec *codec, unsigned int reg,
			unsigned int value)
{
	u8 data[2];
	int ret;

	data[0] = reg;
	data[1] = value & 0x00ff;

	if (reg < ARRAY_SIZE(es8388_reg))
		es8388_reg[reg] = value;
	ret = codec->hw_write(codec->control_data, data, 2);
	if (ret == 2)
		return 0;
	if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int es8388_reset(struct snd_soc_codec *codec)
{
	snd_soc_write(codec, ES8388_CONTROL1, 0x80);
	return snd_soc_write(codec, ES8388_CONTROL1, 0x00);
}

struct _coeff_div {
	u32 mclk;
	u32 rate;
	u16 fs;
	u8 sr:4;
	u8 usb:1;
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 8k */
	{12288000, 8000, 1536, 0xa, 0x0},
	{11289600, 8000, 1408, 0x9, 0x0},
	{18432000, 8000, 2304, 0xc, 0x0},
	{16934400, 8000, 2112, 0xb, 0x0},
	{12000000, 8000, 1500, 0xb, 0x1},

	/* 11.025k */
	{11289600, 11025, 1024, 0x7, 0x0},
	{16934400, 11025, 1536, 0xa, 0x0},
	{12000000, 11025, 1088, 0x9, 0x1},

	/* 16k */
	{12288000, 16000, 768, 0x6, 0x0},
	{18432000, 16000, 1152, 0x8, 0x0},
	{12000000, 16000, 750, 0x7, 0x1},

	/* 22.05k */
	{11289600, 22050, 512, 0x4, 0x0},
	{16934400, 22050, 768, 0x6, 0x0},
	{12000000, 22050, 544, 0x6, 0x1},

	/* 32k */
	{12288000, 32000, 384, 0x3, 0x0},
	{18432000, 32000, 576, 0x5, 0x0},
	{12000000, 32000, 375, 0x4, 0x1},

	/* 44.1k */
	{11289600, 44100, 256, 0x2, 0x0},
	{16934400, 44100, 384, 0x3, 0x0},
	{12000000, 44100, 272, 0x3, 0x1},

	/* 48k */
	{12288000, 48000, 256, 0x2, 0x0},
	{18432000, 48000, 384, 0x3, 0x0},
	{12000000, 48000, 250, 0x2, 0x1},

	/* 88.2k */
	{11289600, 88200, 128, 0x0, 0x0},
	{16934400, 88200, 192, 0x1, 0x0},
	{12000000, 88200, 136, 0x1, 0x1},

	/* 96k */
	{12288000, 96000, 128, 0x0, 0x0},
	{18432000, 96000, 192, 0x1, 0x0},
	{12000000, 96000, 125, 0x0, 0x1},
};

static inline int get_coeff(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}

	return -EINVAL;
}

/* The set of rates we can generate from the above for each SYSCLK */
static unsigned int rates_12288[] = {
	8000, 12000, 16000, 24000, 24000, 32000, 48000, 96000,
};

static struct snd_pcm_hw_constraint_list constraints_12288 = {
	.count = ARRAY_SIZE(rates_12288),
	.list = rates_12288,
};

static unsigned int rates_112896[] = {
	8000, 11025, 22050, 44100,
};

static struct snd_pcm_hw_constraint_list constraints_112896 = {
	.count = ARRAY_SIZE(rates_112896),
	.list = rates_112896,
};

static unsigned int rates_12[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
	48000, 88235, 96000,
};

static struct snd_pcm_hw_constraint_list constraints_12 = {
	.count = ARRAY_SIZE(rates_12),
	.list = rates_12,
};

/*
 * Note that this should be called from init rather than from hw_params.
 */
static int es8388_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct es8388_priv *es8388 = snd_soc_codec_get_drvdata(codec);

	switch (freq) {
	case 11289600:
	case 18432000:
	case 22579200:
	case 36864000:
		es8388->sysclk_constraints = &constraints_112896;
		es8388->sysclk = freq;
		return 0;

	case 12288000:
	case 16934400:
	case 24576000:
	case 33868800:
		es8388->sysclk_constraints = &constraints_12288;
		es8388->sysclk = freq;
		return 0;

	case 12000000:
	case 24000000:
		es8388->sysclk_constraints = &constraints_12;
		es8388->sysclk = freq;
		return 0;
	}
	return -EINVAL;
}

static int es8388_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 iface = 0;
	u8 adciface = 0;
	u8 daciface = 0;

	iface = snd_soc_read(codec, ES8388_IFACE);
	adciface = snd_soc_read(codec, ES8388_ADC_IFACE);
	daciface = snd_soc_read(codec, ES8388_DAC_IFACE);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:	/* MASTER MODE */
		iface |= 0x80;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:	/* SLAVE MODE */
		iface &= 0x7F;
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		adciface &= 0xFC;
		daciface &= 0xF9;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		break;
	case SND_SOC_DAIFMT_DSP_A:
		break;
	case SND_SOC_DAIFMT_DSP_B:
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		iface &= 0xDF;
		adciface &= 0xDF;
		daciface &= 0xBF;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x20;
		adciface |= 0x20;
		daciface |= 0x40;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x20;
		adciface &= 0xDF;
		daciface &= 0xBF;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface &= 0xDF;
		adciface |= 0x20;
		daciface |= 0x40;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_write(codec, ES8388_IFACE, iface);
	snd_soc_write(codec, ES8388_ADC_IFACE, adciface);
	snd_soc_write(codec, ES8388_DAC_IFACE, daciface);

	return 0;
}

static int es8388_pcm_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct es8388_priv *es8388 = snd_soc_codec_get_drvdata(codec);

	/* The set of sample rates that can be supported depends on the
	 * MCLK supplied to the CODEC - enforce this.
	 */
	if (!es8388->sysclk) {
		dev_err(codec->dev,
			"No MCLK configured, call set_sysclk() on init\n");
		return -EINVAL;
	}

	snd_pcm_hw_constraint_list(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_RATE,
				   es8388->sysclk_constraints);

	return 0;
}

static int es8388_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct es8388_priv *es8388 = snd_soc_codec_get_drvdata(codec);
	u16 srate = snd_soc_read(codec, ES8388_IFACE) & 0x80;
	u16 adciface = snd_soc_read(codec, ES8388_ADC_IFACE) & 0xE3;
	u16 daciface = snd_soc_read(codec, ES8388_DAC_IFACE) & 0xC7;
	int coeff;

	coeff = get_coeff(es8388->sysclk, params_rate(params));
	if (coeff < 0) {
		coeff = get_coeff(es8388->sysclk / 2, params_rate(params));
		srate |= 0x40;
	}
	if (coeff < 0) {
		dev_err(codec->dev,
			"Unable to configure sample rate %dHz with %dHz MCLK\n",
			params_rate(params), es8388->sysclk);
		return coeff;
	}

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		adciface |= 0x000C;
		daciface |= 0x0018;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		adciface |= 0x0004;
		daciface |= 0x0008;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		adciface |= 0x0010;
		daciface |= 0x0020;
		break;
	}

	/* set iface & srate */
	snd_soc_write(codec, ES8388_DAC_IFACE, daciface);
	snd_soc_write(codec, ES8388_ADC_IFACE, adciface);

	if (coeff >= 0) {
		snd_soc_write(codec, ES8388_IFACE, srate);
		snd_soc_write(codec, ES8388_ADCCONTROL5,
			      coeff_div[coeff].sr |
			      (coeff_div[coeff].usb) << 4);
		snd_soc_write(codec, ES8388_DACCONTROL2,
			      coeff_div[coeff].sr |
			      (coeff_div[coeff].usb) << 4);
	}

	return 0;
}

static int es8388_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct es8388_priv *es8388 = snd_soc_codec_get_drvdata(codec);

	es8388->muted = mute;
	if (mute) {
		es8388_set_gpio(ES8388_CODEC_SET_SPK, !es8388->spk_gpio_level);
		usleep_range(18000, 20000);
		snd_soc_write(codec, ES8388_DACCONTROL3, 0x06);
	} else {
		snd_soc_write(codec, ES8388_DACCONTROL3, 0x02);
		snd_soc_write(codec, 0x30, es8388_DEF_VOL);
		snd_soc_write(codec, 0x31, es8388_DEF_VOL);
		msleep(50);
		if (!es8388->hp_inserted)
			es8388_set_gpio(ES8388_CODEC_SET_SPK,
					es8388->spk_gpio_level);
		usleep_range(18000, 20000);
	}
	return 0;
}

static int es8388_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	struct es8388_priv *es8388 = snd_soc_codec_get_drvdata(codec);
	int ret;

	switch (level) {
	case SND_SOC_BIAS_ON:
		dev_dbg(codec->dev, "%s on\n", __func__);
		break;
	case SND_SOC_BIAS_PREPARE:
		dev_dbg(codec->dev, "%s prepare\n", __func__);
		if (IS_ERR(es8388->mclk))
			break;
		if (snd_soc_codec_get_bias_level(codec) == SND_SOC_BIAS_ON) {
			clk_disable_unprepare(es8388->mclk);
		} else {
			ret = clk_prepare_enable(es8388->mclk);
			if (ret)
				return ret;
		}
		snd_soc_write(codec, ES8388_ANAVOLMANAG, 0x7C);
		snd_soc_write(codec, ES8388_CHIPLOPOW1, 0x00);
		snd_soc_write(codec, ES8388_CHIPLOPOW2, 0x00);
		snd_soc_write(codec, ES8388_CHIPPOWER, 0x00);
		snd_soc_write(codec, ES8388_ADCPOWER, 0x00);
		break;
	case SND_SOC_BIAS_STANDBY:
		dev_dbg(codec->dev, "%s standby\n", __func__);
		snd_soc_write(codec, ES8388_ANAVOLMANAG, 0x7C);
		snd_soc_write(codec, ES8388_CHIPLOPOW1, 0x00);
		snd_soc_write(codec, ES8388_CHIPLOPOW2, 0x00);
		snd_soc_write(codec, ES8388_CHIPPOWER, 0x00);
		snd_soc_write(codec, ES8388_ADCPOWER, 0x00);
		break;
	case SND_SOC_BIAS_OFF:
		if (es8388->mclk)
			clk_disable_unprepare(es8388->mclk);
		dev_dbg(codec->dev, "%s off\n", __func__);
		snd_soc_write(codec, ES8388_ADCPOWER, 0xFF);
		snd_soc_write(codec, ES8388_DACPOWER, 0xC0);
		snd_soc_write(codec, ES8388_CHIPLOPOW1, 0xFF);
		snd_soc_write(codec, ES8388_CHIPLOPOW2, 0xFF);
		snd_soc_write(codec, ES8388_CHIPPOWER, 0xFF);
		snd_soc_write(codec, ES8388_ANAVOLMANAG, 0x7B);
		break;
	}
	return 0;
}

#define es8388_RATES SNDRV_PCM_RATE_8000_96000

#define es8388_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops es8388_ops = {
	.startup = es8388_pcm_startup,
	.hw_params = es8388_pcm_hw_params,
	.set_fmt = es8388_set_dai_fmt,
	.set_sysclk = es8388_set_dai_sysclk,
	.digital_mute = es8388_mute,
};

static struct snd_soc_dai_driver es8388_dai = {
	.name = "ES8388 HiFi",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = es8388_RATES,
		     .formats = es8388_FORMATS,
		     },
	.capture = {
		    .stream_name = "Capture",
		    .channels_min = 2,
		    .channels_max = 8,
		    .rates = es8388_RATES,
		    .formats = es8388_FORMATS,
		    },
	.ops = &es8388_ops,
	.symmetric_rates = 1,
};

static int es8388_suspend(struct snd_soc_codec *codec)
{
	snd_soc_write(codec, 0x19, 0x06);
	snd_soc_write(codec, 0x30, 0x00);
	snd_soc_write(codec, 0x31, 0x00);
	snd_soc_write(codec, ES8388_ADCPOWER, 0xFF);
	snd_soc_write(codec, ES8388_DACPOWER, 0xc0);
	snd_soc_write(codec, ES8388_CHIPPOWER, 0xF3);
	snd_soc_write(codec, 0x00, 0x00);
	snd_soc_write(codec, 0x01, 0x58);
	snd_soc_write(codec, 0x2b, 0x9c);
	usleep_range(18000, 20000);
	return 0;
}

static int es8388_resume(struct snd_soc_codec *codec)
{
	snd_soc_write(codec, 0x2b, 0x80);
	snd_soc_write(codec, 0x01, 0x50);
	snd_soc_write(codec, 0x00, 0x32);
	snd_soc_write(codec, ES8388_CHIPPOWER, 0x00);
	snd_soc_write(codec, ES8388_DACPOWER, 0x0c);
	snd_soc_write(codec, ES8388_ADCPOWER, 0x00);
	snd_soc_write(codec, 0x31, es8388_DEF_VOL);
	snd_soc_write(codec, 0x30, es8388_DEF_VOL);
	snd_soc_write(codec, 0x19, 0x02);
	return 0;
}

static struct snd_soc_codec *es8388_codec;
static int es8388_probe(struct snd_soc_codec *codec)
{
	struct es8388_priv *es8388 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	if (!codec) {
		dev_err(codec->dev, "Codec device not registered\n");
		return -ENODEV;
	}

	es8388->mclk = devm_clk_get(codec->dev, "mclk");
	if (IS_ERR(es8388->mclk)) {
		dev_err(codec->dev, "%s mclk is missing or invalid\n",
			__func__);
		return PTR_ERR(es8388->mclk);
	}
	ret = clk_prepare_enable(es8388->mclk);
	if (ret)
		return ret;
	codec->hw_write = (hw_write_t)i2c_master_send;
	codec->control_data = container_of(codec->dev, struct i2c_client, dev);

	es8388_codec = codec;
	ret = es8388_reset(codec);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to issue reset\n");
		return ret;
	}
	usleep_range(18000, 20000);
	snd_soc_write(codec, 0x02, 0xf3);
	snd_soc_write(codec, 0x2B, 0x80);
	/* ES8388 salve */
	snd_soc_write(codec, 0x08, 0x00);
	snd_soc_write(codec, 0x00, 0x35);
	/* PLAYBACK & RECORD Mode,EnRefr=1 */
	snd_soc_write(codec, 0x01, 0x50);
	/* pdn_ana=0,ibiasgen_pdn=0 */
	snd_soc_write(codec, 0x03, 0x59);
	/* pdn_ana=0,ibiasgen_pdn=0 */
	snd_soc_write(codec, 0x05, 0x00);
	/* pdn_ana=0,ibiasgen_pdn=0 */
	snd_soc_write(codec, 0x06, 0x00);
	snd_soc_write(codec, 0x07, 0x7c);
	/* ADC L/R PGA =  0dB */
	snd_soc_write(codec, 0x09, 0x00);
	/* ADC INPUT=LIN2/RIN2 */
	snd_soc_write(codec, 0x0a, 0xf8);
	/* ADC INPUT=LIN2/RIN2 */
	snd_soc_write(codec, 0x0b, 0x82);
	/* I2S-16BIT */
	snd_soc_write(codec, 0x0C, 0x0c);
	/* MCLK/LRCK=256 */
	snd_soc_write(codec, 0x0d, 0x02);
	/*
	 * left channel polarity inverted and
	 * enable ADC right channel high pass filter
	 */
	snd_soc_write(codec, 0x0e, 0xb0);
	/* ADC Left Volume=0db */
	snd_soc_write(codec, 0x10, 0x00);
	/* ADC Right Volume=0db */
	snd_soc_write(codec, 0x11, 0x00);
	/* ALC off */
	snd_soc_write(codec, 0x12, 0x2a);
	snd_soc_write(codec, 0x13, 0xc0);
	snd_soc_write(codec, 0x14, 0x05);
	snd_soc_write(codec, 0x15, 0x06);
	snd_soc_write(codec, 0x16, 0x53);
	/* I2S-16BIT */
	snd_soc_write(codec, 0x17, 0x18);
	snd_soc_write(codec, 0x18, 0x02);
	/* DAC VOLUME=0DB */
	snd_soc_write(codec, 0x1A, 0x0A);
	snd_soc_write(codec, 0x1B, 0x0A);
	/* Left DAC TO Left IXER */
	snd_soc_write(codec, 0x26, 0x12);
	/* Left DAC TO Left MIXER */
	snd_soc_write(codec, 0x27, 0xb8);
	snd_soc_write(codec, 0x28, 0x38);
	snd_soc_write(codec, 0x29, 0x38);
	snd_soc_write(codec, 0x2A, 0xb8);
	/* START DLL and state-machine,START DSM */
	snd_soc_write(codec, 0x02, 0x00);
	/* SOFT RAMP RATE=32LRCKS/STEP,Enable ZERO-CROSS CHECK,DAC MUTE */
	snd_soc_write(codec, 0x19, 0x02);
	/* pdn_ana=0,ibiasgen_pdn=0 */
	snd_soc_write(codec, 0x04, 0x0c);
	usleep_range(18000, 20000);
	snd_soc_write(codec, 0x2e, 0x00);
	snd_soc_write(codec, 0x2f, 0x00);
	snd_soc_write(codec, 0x30, 0x08);
	snd_soc_write(codec, 0x31, 0x08);
	usleep_range(18000, 20000);
	snd_soc_write(codec, 0x30, 0x0f);
	snd_soc_write(codec, 0x31, 0x0f);
	usleep_range(18000, 20000);
	snd_soc_write(codec, 0x30, 0x18);
	snd_soc_write(codec, 0x31, 0x18);
	usleep_range(18000, 20000);
	/* pdn_ana=0,ibiasgen_pdn=0 */
	snd_soc_write(codec, 0x04, 0x2c);

	snd_soc_write(codec, ES8388_DACCONTROL3, 0x06);
	es8388_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}

static int es8388_remove(struct snd_soc_codec *codec)
{
	es8388_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_es8388 = {
	.probe = es8388_probe,
	.remove = es8388_remove,
	.suspend = es8388_suspend,
	.resume = es8388_resume,
	.set_bias_level = es8388_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(es8388_reg),
	.reg_word_size = sizeof(u16),
	.reg_cache_default = es8388_reg,
	.reg_cache_step = 1,
	.read = es8388_read_reg_cache,
	.write = es8388_write,

};

static int es8388_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct es8388_priv *es8388;
	int ret = -1;
	int hp_irq = 0;
	enum of_gpio_flags flags;
	struct i2c_adapter *adapter = to_i2c_adapter(i2c->dev.parent);
	char reg;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_I2C\n");
		return -EIO;
	}

	es8388 = devm_kzalloc(&i2c->dev, sizeof(struct es8388_priv),
			      GFP_KERNEL);
	if (!es8388)
		return -ENOMEM;

	i2c_set_clientdata(i2c, es8388);

	reg = ES8388_DACCONTROL18;
	ret = i2c_master_recv(i2c, &reg, 1);
	if (ret < 0) {
		dev_err(&i2c->dev, "i2c recv Failed\n");
		return ret;
	}

	es8388_private = es8388;
	es8388->spk_ctl_gpio = of_get_named_gpio_flags(i2c->dev.of_node,
						       "spk-con-gpio", 0,
						       &flags);
	if (es8388->spk_ctl_gpio < 0) {
		dev_info(&i2c->dev, "Can not read property spk_ctl_gpio\n");
		es8388->spk_ctl_gpio = INVALID_GPIO;
	} else {
		es8388->spk_gpio_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = devm_gpio_request_one(&i2c->dev, es8388->spk_ctl_gpio,
					    GPIOF_DIR_OUT, NULL);
		if (ret != 0) {
			dev_err(&i2c->dev, "Failed to request spk_ctl_gpio\n");
			return ret;
		}
		es8388_set_gpio(ES8388_CODEC_SET_SPK, !es8388->spk_gpio_level);
	}

	es8388->hp_det_gpio = of_get_named_gpio_flags(i2c->dev.of_node,
						      "hp-det-gpio", 0, &flags);
	if (es8388->hp_det_gpio < 0) {
		dev_info(&i2c->dev, "Can not read property hp_det_gpio\n");
		es8388->hp_det_gpio = INVALID_GPIO;
	} else {
		es8388->hp_det_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = devm_gpio_request_one(&i2c->dev, es8388->hp_det_gpio,
					    GPIOF_IN, NULL);
		if (ret != 0) {
			dev_err(&i2c->dev, "Failed to request hp_det_gpio\n");
			return ret;
		}
		hp_irq = gpio_to_irq(es8388->hp_det_gpio);

		if (hp_irq) {
			ret = devm_request_threaded_irq(&i2c->dev, hp_irq, NULL,
							hp_det_irq_handler,
							IRQ_TYPE_EDGE_BOTH |
							IRQF_ONESHOT, "ES8388",
							NULL);
			if (ret < 0) {
				dev_err(&i2c->dev, "request_irq failed: %d\n",
					ret);
				return ret;
			}
		}
	}
	ret = snd_soc_register_codec(&i2c->dev,
				     &soc_codec_dev_es8388,
				     &es8388_dai, 1);
	return ret;
}

static int es8388_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct i2c_device_id es8388_i2c_id[] = {
	{"es8388", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, es8388_i2c_id);

void es8388_i2c_shutdown(struct i2c_client *client)
{
	struct es8388_priv *es8388 = es8388_private;

	es8388_set_gpio(ES8388_CODEC_SET_SPK, !es8388->spk_gpio_level);
	mdelay(20);
	snd_soc_write(es8388_codec, ES8388_CONTROL2, 0x58);
	snd_soc_write(es8388_codec, ES8388_CONTROL1, 0x32);
	snd_soc_write(es8388_codec, ES8388_CHIPPOWER, 0xf3);
	snd_soc_write(es8388_codec, ES8388_DACPOWER, 0xc0);
	mdelay(50);
	snd_soc_write(es8388_codec, ES8388_DACCONTROL26, 0x00);
	snd_soc_write(es8388_codec, ES8388_DACCONTROL27, 0x00);
	mdelay(50);
	snd_soc_write(es8388_codec, ES8388_CONTROL1, 0x30);
	snd_soc_write(es8388_codec, ES8388_CONTROL1, 0x34);
}

static const struct of_device_id es8388_of_match[] = {
	{ .compatible = "everest,es8388", },
	{ }
};
MODULE_DEVICE_TABLE(of, es8388_of_match);

static struct i2c_driver es8388_i2c_driver = {
	.driver = {
		.name = "ES8388",
		.of_match_table = of_match_ptr(es8388_of_match),
		},
	.shutdown = es8388_i2c_shutdown,
	.probe = es8388_i2c_probe,
	.remove = es8388_i2c_remove,
	.id_table = es8388_i2c_id,
};
module_i2c_driver(es8388_i2c_driver);

MODULE_DESCRIPTION("ASoC es8388 driver");
MODULE_AUTHOR("Mark Brown <will@everset-semi.com>");
MODULE_LICENSE("GPL");
