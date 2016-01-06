/*
 * nau8540.c  --  NAU85L40 ALSA SoC Audio driver
 *
 * Copyright 2015 Nuvoton Technology Corp.
 *  Author: CFYang3 <CFYang3@nuvoton.com>
 *  Co-author: David Lin <ctlin0@nuvoton.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "nau8540.h"

static const struct reg_default nau8540_reg_defaults[] = {
	{0x01, 0x0000},
	{0x02, 0x0000},
	{0x03, 0x0000},
	{0x04, 0x0001},
	{0x05, 0x3126},
	{0x06, 0x0008},
	{0x07, 0x0010},
	{0x08, 0xC000},
	{0x09, 0x6000},
	{0x0A, 0xF13C},
	{0x10, 0x000B},
	{0x11, 0x0002},
	{0x12, 0x0000},
	{0x13, 0x0000},
	{0x14, 0x0000},
	{0x20, 0x0070},
	{0x21, 0x0000},
	{0x22, 0x0000},
	{0x23, 0x1010},
	{0x24, 0x1010},
	{0x30, 0x0000},
	{0x31, 0x0000},
	{0x32, 0x0000},
	{0x33, 0x0000},
	{0x34, 0x0000},
	{0x35, 0x0000},
	{0x36, 0x0000},
	{0x37, 0x0000},
	{0x38, 0x0000},
	{0x39, 0x0000},
	{0x3A, 0x0002},
	{0x40, 0x0400},
	{0x41, 0x1400},
	{0x42, 0x2400},
	{0x43, 0x0400},
	{0x44, 0x00E4},
	{0x50, 0x0000},
	{0x51, 0x0000},
	{0x52, 0xEFFF},
	{0x5A, 0x0000},
	{0x60, 0x0000},
	{0x61, 0x0000},
	{0x64, 0x0000},
	{0x65, 0x0020},
	{0x66, 0x0000},
	{0x67, 0x0004},
	{0x68, 0x0000},
	{0x69, 0x0000},
	{0x6A, 0x0000},
	{0x6B, 0x0101},
	{0x6C, 0x0101},
	{0x6D, 0x0000},
};

static u16 set_codec_reg_init[][2] = {
	{0x01, 0x000F},
	{0x02, 0x800A},
	{0x03, 0x8043},
	{0x04, 0x0001},
	{0x05, 0x3126},
	{0x06, 0x0010},
	{0x07, 0x0010},
	{0x08, 0xC000},
	{0x09, 0x6000},
	{0x0A, 0xF13C},
	{0x10, 0x0002},
	{0x11, 0x6012},
	{0x12, 0x4800},
	{0x13, 0x0000},
	{0x14, 0x0000},
	{0x20, 0x0070},
	{0x21, 0x0000},
	{0x22, 0x0000},
	{0x23, 0x1010},
	{0x24, 0x1010},
	{0x30, 0x0000},
	{0x31, 0x0000},
	{0x32, 0x0000},
	{0x33, 0x0000},
	{0x34, 0x0000},
	{0x35, 0x0000},
	{0x36, 0x0000},
	{0x37, 0x0000},
	{0x38, 0x0000},
	{0x39, 0x0000},
	{0x3A, 0x0002},
	{0x40, 0x04a1},
	{0x41, 0x04a1},
	{0x42, 0x04a1},
	{0x43, 0x04a1},
	{0x44, 0x00E4},
	{0x50, 0x0000},
	{0x51, 0x0000},
	{0x52, 0xEFFF},
	{0x60, 0x0060},
	{0x61, 0x0000},
	{0x64, 0x0000},
	{0x65, 0x0220},
	{0x66, 0x000F},
	{0x67, 0x0D04},
	{0x68, 0x7000},
	{0x69, 0x0000},
	{0x6A, 0x0000},
	{0x6B, 0x1010},
	{0x6C, 0x1010},
	{0x6D, 0xF000},
};

#define SET_CODEC_REG_INIT_NUM	ARRAY_SIZE(set_codec_reg_init)


static bool nau8540_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG0x01_POWER_MANAGEMENT ... REG0x0A_FLL_VCO_RSV:
	case REG0x10_PCM_CTRL0 ... REG0x14_PCM_CTRL4:
	case REG0x20_ALC_CONTROL_1 ... REG0x24_ALC_CONTROL_5:
	case REG0x2D_ALC_GAIN_CH12 ... REG0x3A_ADC_SAMPLE_RATE:
	case REG0x40_DIGITAL_GAIN_CH1 ... REG0x44_DIGITAL_MUX:
	case REG0x48_P2P_CH1 ... REG0x52_I2C_CTRL:
	case REG0x58_I2C_DEVICE_ID ... REG0x58_I2C_DEVICE_ID:
	case REG0x5A_RST ... REG0x5A_RST:
	case REG0x60_VMID_CTRL ... REG0x61_MUTE:
	case REG0x64_ANALOG_ADC1 ... REG0x6D_PWR:
		return true;
	default:
		return false;
	}
}

static bool nau8540_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG0x00_SW_RESET ... REG0x0A_FLL_VCO_RSV:
	case REG0x10_PCM_CTRL0 ... REG0x14_PCM_CTRL4:
	case REG0x20_ALC_CONTROL_1 ... REG0x24_ALC_CONTROL_5:
	case REG0x30_NOTCH_FIL1_CH1 ... REG0x3A_ADC_SAMPLE_RATE:
	case REG0x40_DIGITAL_GAIN_CH1 ... REG0x44_DIGITAL_MUX:
	case REG0x50_GPIO_CTRL ... REG0x52_I2C_CTRL:
	case REG0x5A_RST ... REG0x5A_RST:
	case REG0x60_VMID_CTRL ... REG0x61_MUTE:
	case REG0x64_ANALOG_ADC1 ... REG0x6D_PWR:
		return true;
	default:
		return false;
	}
}

static bool nau8540_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG0x00_SW_RESET:
	case REG0x2D_ALC_GAIN_CH12 ... REG0x2F_ALC_STATUS:
	case REG0x48_P2P_CH1 ... REG0x4F_PEAK_CH4:
	case REG0x58_I2C_DEVICE_ID:
	case REG0x5A_RST:
		return true;
	default:
		return false;
	}
}

static void nau8540_reset(struct regmap *regmap)
{
	regmap_write(regmap, REG0x00_SW_RESET, 0x00);
}

static const DECLARE_TLV_DB_MINMAX_MUTE(adc_vol_tlv, -12800, 3600);
static const DECLARE_TLV_DB_MINMAX(fepga_gain_tlv, -100, 3600);

static const struct snd_kcontrol_new nau8540_controls[] = {
	SOC_SINGLE_TLV("MIC1 Digital Volume", REG0x40_DIGITAL_GAIN_CH1,
		       0, 0x7ff, 0, adc_vol_tlv),
	SOC_SINGLE_TLV("MIC2 Digital Volume", REG0x41_DIGITAL_GAIN_CH2,
		       0, 0x7ff, 0, adc_vol_tlv),
	SOC_SINGLE_TLV("MIC3 Digital Volume", REG0x42_DIGITAL_GAIN_CH3,
		       0, 0x7ff, 0, adc_vol_tlv),
	SOC_SINGLE_TLV("MIC4 Digital Volume", REG0x43_DIGITAL_GAIN_CH4,
		       0, 0x7ff, 0, adc_vol_tlv),

	SOC_SINGLE_TLV("Frontend PGA1 Volume", REG0x6B_FEPGA3,
		       0, 0x3f, 0, fepga_gain_tlv),
	SOC_SINGLE_TLV("Frontend PGA2 Volume", REG0x6B_FEPGA3,
		       8, 0x3f, 0, fepga_gain_tlv),
	SOC_SINGLE_TLV("Frontend PGA3 Volume", REG0x6C_FEPGA4,
		       0, 0x3f, 0, fepga_gain_tlv),
	SOC_SINGLE_TLV("Frontend PGA4 Volume", REG0x6C_FEPGA4,
		       8, 0x3f, 0, fepga_gain_tlv),
};

static const struct snd_soc_dapm_widget nau8540_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),
	SND_SOC_DAPM_INPUT("MIC3"),
	SND_SOC_DAPM_INPUT("MIC4"),

	SND_SOC_DAPM_PGA("Frontend PGA1", REG0x6D_PWR, 12, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Frontend PGA2", REG0x6D_PWR, 13, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Frontend PGA3", REG0x6D_PWR, 14, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Frontend PGA4", REG0x6D_PWR, 15, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("MICBIAS1", REG0x67_MIC_BIAS, 10, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("MICBIAS2", REG0x67_MIC_BIAS, 11, 0, NULL, 0),

	SND_SOC_DAPM_ADC("ADC1", NULL, REG0x01_POWER_MANAGEMENT, 0, 0),
	SND_SOC_DAPM_ADC("ADC2", NULL, REG0x01_POWER_MANAGEMENT, 1, 0),
	SND_SOC_DAPM_ADC("ADC3", NULL, REG0x01_POWER_MANAGEMENT, 2, 0),
	SND_SOC_DAPM_ADC("ADC4", NULL, REG0x01_POWER_MANAGEMENT, 3, 0),

	SND_SOC_DAPM_AIF_OUT("AIFTX", "Capture", 0, REG0x11_PCM_CTRL1, 15, 1),
};

static const struct snd_soc_dapm_route nau8540_dapm_routes[] = {
	{"Frontend PGA1", NULL, "MIC1"},
	{"Frontend PGA2", NULL, "MIC2"},
	{"Frontend PGA3", NULL, "MIC3"},
	{"Frontend PGA4", NULL, "MIC4"},

	{"ADC1", NULL, "Frontend PGA1"},
	{"ADC2", NULL, "Frontend PGA2"},
	{"ADC3", NULL, "Frontend PGA3"},
	{"ADC4", NULL, "Frontend PGA4"},

	{"ADC1", NULL, "MICBIAS1"},
	{"ADC2", NULL, "MICBIAS1"},
	{"ADC3", NULL, "MICBIAS2"},
	{"ADC4", NULL, "MICBIAS2"},

	{"AIFTX", NULL, "ADC1"},
	{"AIFTX", NULL, "ADC2"},
	{"AIFTX", NULL, "ADC3"},
	{"AIFTX", NULL, "ADC4"},
};

static int nau8540_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct nau8540_priv *nau8540 = snd_soc_codec_get_drvdata(codec);

	u16 val_len = 0;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		val_len |= 0x4;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		val_len |= 0x8;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		val_len |= 0xc;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(nau8540->regmap, REG0x10_PCM_CTRL0, 0xc, val_len);

	return 0;
}

static int nau8540_set_dai_fmt(struct snd_soc_dai *codec_dai,
			       unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct nau8540_priv *nau8540 = snd_soc_codec_get_drvdata(codec);
	u16 reg_val_0 = 0, reg_val_1 = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		reg_val_0 |= 0x8;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		dev_alert(codec->dev, "Invalid DAI master/slave interface\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		reg_val_1 |= 0x2;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		reg_val_1 |= 0x1;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		reg_val_1 |= 0x3;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		reg_val_1 |= 0x3;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		reg_val_1 |= 0x80;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(nau8540->regmap, REG0x10_PCM_CTRL0,
			   0x83, reg_val_1);
	regmap_update_bits(nau8540->regmap, REG0x11_PCM_CTRL1,
			   0x8, reg_val_0);

	return 0;
}

static int nau8540_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	struct nau8540_priv *nau8540 = snd_soc_codec_get_drvdata(codec);
	int ret;

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		break;

	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			ret = regcache_sync(nau8540->regmap);
			if (ret) {
				dev_err(codec->dev,
					"Failed to sync cache: %d\n", ret);
				return ret;
			}
		}
		break;

	case SND_SOC_BIAS_OFF:
		break;
	}

	codec->dapm.bias_level = level;
	return 0;
}

#define NAU8540_RATES SNDRV_PCM_RATE_8000_48000

#define NAU8540_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops nau8540_dai_ops = {
	.hw_params	= nau8540_hw_params,
	.set_fmt	= nau8540_set_dai_fmt,
};

static struct snd_soc_dai_driver nau8540_dai = {
	.name = "nau8540",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,	/* Mono modes not yet supported */
		.channels_max = 8,
		.rates = NAU8540_RATES,
		.formats = NAU8540_FORMATS,
	},
	.ops = &nau8540_dai_ops,
};

#ifdef CONFIG_PM
static int nau8540_suspend(struct snd_soc_codec *codec)
{
	struct nau8540_priv *nau8540 = snd_soc_codec_get_drvdata(codec);

	nau8540_set_bias_level(codec, SND_SOC_BIAS_OFF);
	regcache_cache_only(nau8540->regmap, true);
	return 0;
}

static int nau8540_resume(struct snd_soc_codec *codec)
{
	struct nau8540_priv *nau8540 = snd_soc_codec_get_drvdata(codec);

	nau8540_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	regcache_cache_only(nau8540->regmap, false);
	return 0;
}
#else
#define nau8540_suspend NULL
#define nau8540_resume NULL
#endif

static int nau8540_probe(struct snd_soc_codec *codec)
{
	struct nau8540_priv *nau8540 = snd_soc_codec_get_drvdata(codec);
	int ret, i;

	codec->control_data = nau8540->regmap;

	ret = snd_soc_codec_set_cache_io(codec, 16, 16, SND_SOC_REGMAP);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	nau8540_reset(nau8540->regmap);

	for (i = 0; i < SET_CODEC_REG_INIT_NUM; i++) {
		ret = regmap_write(nau8540->regmap, set_codec_reg_init[i][0],
				   set_codec_reg_init[i][1]);
		if (ret)
			dev_err(codec->dev, "reg: 0x%x, ret: 0x%x\n",
				set_codec_reg_init[i][0], ret);
	}

	nau8540_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

static int nau8540_remove(struct snd_soc_codec *codec)
{
	nau8540_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_nau8540 = {
	.probe		= nau8540_probe,
	.remove		= nau8540_remove,
	.suspend	= nau8540_suspend,
	.resume		= nau8540_resume,
	.set_bias_level = nau8540_set_bias_level,
	.controls = nau8540_controls,
	.num_controls = ARRAY_SIZE(nau8540_controls),
	.dapm_widgets = nau8540_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(nau8540_dapm_widgets),
	.dapm_routes = nau8540_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(nau8540_dapm_routes),
};

static const struct regmap_config nau8540_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,

	.max_register = NAU8540_MAX_REGISTER,
	.readable_reg = nau8540_readable_reg,
	.writeable_reg = nau8540_writeable_reg,
	.volatile_reg = nau8540_volatile,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = nau8540_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(nau8540_reg_defaults),
};

static int nau8540_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct nau8540_priv *nau8540;
	int ret;

	nau8540 = devm_kzalloc(&i2c->dev, sizeof(*nau8540), GFP_KERNEL);
	if (!nau8540)
		return -ENOMEM;

	nau8540->regmap = devm_regmap_init_i2c(i2c, &nau8540_regmap_config);
	if (IS_ERR(nau8540->regmap)) {
		ret = PTR_ERR(nau8540->regmap);
		return ret;
	}

	i2c_set_clientdata(i2c, nau8540);

	ret = snd_soc_register_codec(&i2c->dev,
				     &soc_codec_dev_nau8540, &nau8540_dai, 1);
	return ret;
}

static int nau8540_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id nau8540_of_match[] = {
	{ .compatible = "nuvoton,nau8540", },
	{ }
};
MODULE_DEVICE_TABLE(of, nau8540_of_match);
#endif

static const struct i2c_device_id nau8540_i2c_id[] = {
	{ "nau8540", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nau8540_i2c_id);

static struct i2c_driver nau8540_i2c_driver = {
	.driver = {
		.name = "nau8540",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(nau8540_of_match),
	},
	.probe = nau8540_i2c_probe,
	.remove = nau8540_i2c_remove,
	.id_table = nau8540_i2c_id,
};
module_i2c_driver(nau8540_i2c_driver);

MODULE_DESCRIPTION("ASoC NAU85L40 driver");
MODULE_AUTHOR("CFYang3 <CFYang3@nuvoton.com>");
MODULE_LICENSE("GPL");
