// SPDX-License-Identifier: GPL-2.0
//
// es8323.c -- es8323 ALSA SoC audio driver
//
// Copyright (c) 2016 Rockchip Electronics Co. Ltd.
//
// Author: Mark Brown <will@everset-semi.com>
// Author: Jianqun Xu <jay.xu@rock-chips.com>
// Author: Nickey Yang <nickey.yang@rock-chips.com>

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include "es8323.h"

#define NR_SUPPORTED_MCLK_LRCK_RATIOS 5
static const unsigned int supported_mclk_lrck_ratios[NR_SUPPORTED_MCLK_LRCK_RATIOS] = {
	256, 384, 512, 768, 1024
};

#define es8323_DEF_VOL	0x1b

static int es8323_set_bias_level(struct snd_soc_component *component,
				 enum snd_soc_bias_level level);

static struct reg_default es8323_reg_defaults[] = {
	{ 0x00, 0x06 },
	{ 0x01, 0x1c },
	{ 0x02, 0xc3 },
	{ 0x03, 0xfc },
	{ 0x04, 0xc0 },
	{ 0x05, 0x00 },
	{ 0x06, 0x00 },
	{ 0x07, 0x7c },
	{ 0x08, 0x80 },
	{ 0x09, 0x00 },
	{ 0x0a, 0x00 },
	{ 0x0b, 0x06 },
	{ 0x0c, 0x00 },
	{ 0x0d, 0x06 },
	{ 0x0e, 0x30 },
	{ 0x0f, 0x30 },
	{ 0x10, 0xc0 },
	{ 0x11, 0xc0 },
	{ 0x12, 0x38 },
	{ 0x13, 0xb0 },
	{ 0x14, 0x32 },
	{ 0x15, 0x06 },
	{ 0x16, 0x00 },
	{ 0x17, 0x00 },
	{ 0x18, 0x06 },
	{ 0x19, 0x30 },
	{ 0x1a, 0xc0 },
	{ 0x1b, 0xc0 },
	{ 0x1c, 0x08 },
	{ 0x1d, 0x06 },
	{ 0x1e, 0x1f },
	{ 0x1f, 0xf7 },
	{ 0x20, 0xfd },
	{ 0x21, 0xff },
	{ 0x22, 0x1f },
	{ 0x23, 0xf7 },
	{ 0x24, 0xfd },
	{ 0x25, 0xff },
	{ 0x26, 0x00 },
	{ 0x27, 0x38 },
	{ 0x28, 0x38 },
	{ 0x29, 0x38 },
	{ 0x2a, 0x38 },
	{ 0x2b, 0x38 },
	{ 0x2c, 0x38 },
};

/* codec private data */
struct es8323_priv {
	unsigned int sysclk;
	unsigned int allowed_rates[NR_SUPPORTED_MCLK_LRCK_RATIOS];
	struct clk *mclk;
	struct snd_pcm_hw_constraint_list sysclk_constraints;
	struct snd_soc_component *component;
	struct regmap *regmap;
};

static int es8323_reset(struct snd_soc_component *component)
{
	snd_soc_component_write(component, ES8323_CONTROL1, 0x80);
	return snd_soc_component_write(component, ES8323_CONTROL1, 0x00);
}

static const char * const es8323_line_texts[] = {
	"Line 1", "Line 2", "PGA"
};

static const unsigned int es8323_line_values[] = {
	0, 1, 3
};

static const char * const es8323_pga_sell[] = {"Line 1L", "Line 2L", "DifferentialL"};
static const char * const es8323_pga_selr[] = {"Line 1R", "Line 2R", "DifferentialR"};
static const char * const es8323_lin_sell[] = {"Line 1L", "Line 2L", "NC", "MicL"};
static const char * const es8323_lin_selr[] = {"Line 1R", "Line 2R", "NC", "MicR"};

static const char * const stereo_3d_txt[] = {
	"No 3D  ", "Level 1", "Level 2",
	"Level 3", "Level 4", "Level 5",
	"Level 6", "Level 7"
};

static const char * const alc_func_txt[] = {
	"Off", "Right", "Left", "Stereo"
};

static const char * const ng_type_txt[] = {
	"Constant PGA Gain", "Mute ADC Output"
};

static const char * const deemph_txt[] = {
	"None", "32Khz", "44.1Khz", "48Khz"
};

static const char * const adcpol_txt[] = {
	"Normal", "L Invert", "R Invert", "L + R Invert"
};

static const char * const es8323_mono_mux[] = {
	"Stereo", "Mono (Left)", "Mono (Right)"
};

static const char * const es8323_diff_sel[] = {
	"Line 1", "Line 2"
};

SOC_VALUE_ENUM_SINGLE_DECL(es8323_left_dac_enum, ES8323_ADCCONTROL2, 6, 3, es8323_pga_sell, es8323_line_values);
SOC_VALUE_ENUM_SINGLE_DECL(es8323_right_dac_enum, ES8323_ADCCONTROL2, 4, 3, es8323_pga_selr, es8323_line_values);
static SOC_ENUM_SINGLE_DECL(es8323_diff_enum, ES8323_ADCCONTROL3, 7, es8323_diff_sel);
static SOC_ENUM_SINGLE_DECL(es8323_llin_enum, ES8323_DACCONTROL16, 3, es8323_lin_sell);
static SOC_ENUM_SINGLE_DECL(es8323_rlin_enum, ES8323_DACCONTROL16, 0, es8323_lin_selr);
static SOC_ENUM_SINGLE_DECL(es8323_mono_enum, ES8323_ADCCONTROL3, 3, es8323_mono_mux);

static const struct soc_enum es8323_enum[] = {
	SOC_VALUE_ENUM_SINGLE(ES8323_DACCONTROL16, 3, 7, ARRAY_SIZE(es8323_line_texts), es8323_line_texts, es8323_line_values),	/* LLINE */
	SOC_VALUE_ENUM_SINGLE(ES8323_DACCONTROL16, 0, 7, ARRAY_SIZE(es8323_line_texts), es8323_line_texts, es8323_line_values),	/* RLINE */
	SOC_VALUE_ENUM_SINGLE(ES8323_ADCCONTROL2, 6, 3, ARRAY_SIZE(es8323_pga_sell), es8323_line_texts, es8323_line_values),	/* Left PGA Mux */
	SOC_VALUE_ENUM_SINGLE(ES8323_ADCCONTROL2, 4, 3, ARRAY_SIZE(es8323_pga_sell), es8323_line_texts, es8323_line_values),	/* Right PGA Mux */
	SOC_ENUM_SINGLE(ES8323_DACCONTROL7, 2, 8, stereo_3d_txt),	/* stereo-3d */
	SOC_ENUM_SINGLE(ES8323_ADCCONTROL10, 6, 4, alc_func_txt),	/* alc func */
	SOC_ENUM_SINGLE(ES8323_ADCCONTROL14, 1, 2, ng_type_txt),	/* noise gate type */
	SOC_ENUM_SINGLE(ES8323_DACCONTROL6, 6, 4, deemph_txt),	/* Playback De-emphasis */
	SOC_ENUM_SINGLE(ES8323_ADCCONTROL6, 6, 4, adcpol_txt),
	SOC_ENUM_SINGLE(ES8323_ADCCONTROL3, 3, 3, es8323_mono_mux),
	SOC_ENUM_SINGLE(ES8323_ADCCONTROL3, 7, 2, es8323_diff_sel),
};

static const DECLARE_TLV_DB_SCALE(adc_tlv, -9600, 50, 1);
static const DECLARE_TLV_DB_SCALE(dac_tlv, -9600, 50, 1);
static const DECLARE_TLV_DB_SCALE(out_tlv, -4500, 150, 0);
static const DECLARE_TLV_DB_SCALE(bypass_tlv, 0, 300, 0);
static const DECLARE_TLV_DB_SCALE(bypass_tlv2, -15, 300, 0);

static const struct snd_kcontrol_new es8323_left_dac_mux_controls = SOC_DAPM_ENUM("Route", es8323_left_dac_enum);
static const struct snd_kcontrol_new es8323_right_dac_mux_controls = SOC_DAPM_ENUM("Route", es8323_right_dac_enum);
static const struct snd_kcontrol_new es8323_diffmux_controls = SOC_DAPM_ENUM("Route2", es8323_diff_enum);

static const struct snd_kcontrol_new es8323_snd_controls[] = {
	SOC_ENUM("3D Mode", es8323_enum[4]),
	SOC_SINGLE("ALC Capture Target Volume", ES8323_ADCCONTROL11, 4, 15, 0),
	SOC_SINGLE("ALC Capture Max PGA", ES8323_ADCCONTROL10, 3, 7, 0),
	SOC_SINGLE("ALC Capture Min PGA", ES8323_ADCCONTROL10, 0, 7, 0),
	SOC_ENUM("ALC Capture Function", es8323_enum[5]),
	SOC_SINGLE("ALC Capture ZC Switch", ES8323_ADCCONTROL13, 6, 1, 0),
	SOC_SINGLE("ALC Capture Hold Time", ES8323_ADCCONTROL11, 0, 15, 0),
	SOC_SINGLE("ALC Capture Decay Time", ES8323_ADCCONTROL12, 4, 15, 0),
	SOC_SINGLE("ALC Capture Attack Time", ES8323_ADCCONTROL12, 0, 15, 0),
	SOC_SINGLE("ALC Capture NG Threshold", ES8323_ADCCONTROL14, 3, 31, 0),
	SOC_ENUM("ALC Capture NG Type", es8323_enum[6]),
	SOC_SINGLE("ALC Capture NG Switch", ES8323_ADCCONTROL14, 0, 1, 0),
	SOC_SINGLE("ZC Timeout Switch", ES8323_ADCCONTROL13, 6, 1, 0),
	SOC_DOUBLE_R_TLV("Capture Digital Volume", ES8323_ADCCONTROL8,
			 ES8323_ADCCONTROL9, 0, 192, 1, adc_tlv),
	SOC_SINGLE("Capture Mute", ES8323_ADCCONTROL7, 2, 1, 0),
	SOC_SINGLE_TLV("Left Channel Capture Volume", ES8323_ADCCONTROL1, 4, 8,
		       0, bypass_tlv),
	SOC_SINGLE_TLV("Right Channel Capture Volume", ES8323_ADCCONTROL1, 0,
		       8, 0, bypass_tlv),
	SOC_ENUM("Playback De-emphasis", es8323_enum[7]),
	SOC_ENUM("Capture Polarity", es8323_enum[8]),
	SOC_DOUBLE_R_TLV("PCM Volume", ES8323_DACCONTROL4, ES8323_DACCONTROL5,
			 0, 192, 1, dac_tlv),
	SOC_SINGLE_TLV("Left Mixer Left Bypass Volume", ES8323_DACCONTROL17, 3,
		       7, 1, bypass_tlv2),
	SOC_SINGLE_TLV("Right Mixer Right Bypass Volume", ES8323_DACCONTROL20,
		       3, 7, 1, bypass_tlv2),
	SOC_DOUBLE_R_TLV("Output 1 Playback Volume", ES8323_DACCONTROL24,
			 ES8323_DACCONTROL25, 0, 33, 0, out_tlv),
	SOC_DOUBLE_R_TLV("Output 2 Playback Volume", ES8323_DACCONTROL26,
			 ES8323_DACCONTROL27, 0, 33, 0, out_tlv),
};

static const struct snd_kcontrol_new es8323_left_line_controls =
SOC_DAPM_ENUM("LLIN Mux", es8323_llin_enum);

static const struct snd_kcontrol_new es8323_right_line_controls =
SOC_DAPM_ENUM("RLIN Mux", es8323_rlin_enum);
/* Mono ADC Mux */
static const struct snd_kcontrol_new es8323_monomux_controls =
SOC_DAPM_ENUM("Mono Mux", es8323_mono_enum);

/* Left Mixer */
static const struct snd_kcontrol_new es8323_left_mixer_controls[] = {
	SOC_DAPM_SINGLE("Left Playback Switch", ES8323_DACCONTROL17, 7, 1, 0),
	SOC_DAPM_SINGLE("Left Bypass Switch", ES8323_DACCONTROL17, 6, 1, 0),
};

/* Right Mixer */
static const struct snd_kcontrol_new es8323_right_mixer_controls[] = {
	SOC_DAPM_SINGLE("Right Playback Switch", ES8323_DACCONTROL20, 7, 1, 0),
	SOC_DAPM_SINGLE("Right Bypass Switch", ES8323_DACCONTROL20, 6, 1, 0),
};

static const struct snd_soc_dapm_widget es8323_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("LINPUT1"),
	SND_SOC_DAPM_INPUT("LINPUT2"),
	SND_SOC_DAPM_INPUT("RINPUT1"),
	SND_SOC_DAPM_INPUT("RINPUT2"),
	SND_SOC_DAPM_MUX("Left PGA Mux", SND_SOC_NOPM, 0, 0,
			 &es8323_left_dac_mux_controls),
	SND_SOC_DAPM_MUX("Right PGA Mux", SND_SOC_NOPM, 0, 0,
			 &es8323_right_dac_mux_controls),
	SND_SOC_DAPM_MICBIAS("Mic Bias", ES8323_ADCPOWER, 3, 1),

	SND_SOC_DAPM_MUX("Differential Mux", SND_SOC_NOPM, 0, 0,
			 &es8323_diffmux_controls),

	SND_SOC_DAPM_MUX("Left ADC Mux", SND_SOC_NOPM, 0, 0,
			 &es8323_monomux_controls),
	SND_SOC_DAPM_MUX("Right ADC Mux", SND_SOC_NOPM, 0, 0,
			 &es8323_monomux_controls),

	SND_SOC_DAPM_MUX("Left Line Mux", SND_SOC_NOPM, 0, 0,
			 &es8323_left_line_controls),
	SND_SOC_DAPM_MUX("Right Line Mux", SND_SOC_NOPM, 0, 0,
			 &es8323_right_line_controls),

	SND_SOC_DAPM_ADC("Right ADC", "Right Capture", ES8323_ADCPOWER, 4, 1),
	SND_SOC_DAPM_ADC("Left ADC", "Left Capture", ES8323_ADCPOWER, 5, 1),

	/* gModify.Cmmt Implement when suspend/startup */
	SND_SOC_DAPM_DAC("Right DAC", "Right Playback", ES8323_DACPOWER, 6, 1),
	SND_SOC_DAPM_DAC("Left DAC", "Left Playback", ES8323_DACPOWER, 7, 1),

	SND_SOC_DAPM_AIF_OUT("I2S OUT", "Capture",  0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("I2S IN", "Playback", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MIXER("Left Mixer", SND_SOC_NOPM, 0, 0,
			   &es8323_left_mixer_controls[0],
			   ARRAY_SIZE(es8323_left_mixer_controls)),
	SND_SOC_DAPM_MIXER("Right Mixer", SND_SOC_NOPM, 0, 0,
			   &es8323_right_mixer_controls[0],
			   ARRAY_SIZE(es8323_right_mixer_controls)),
	SND_SOC_DAPM_PGA("Right ADC Power", ES8323_ADCPOWER, 6, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Left ADC Power", ES8323_ADCPOWER, 7, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Right Out 2", ES8323_DACPOWER, 2, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left Out 2", ES8323_DACPOWER, 3, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Out 1", ES8323_DACPOWER, 4, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left Out 1", ES8323_DACPOWER, 5, 0, NULL, 0),
	SND_SOC_DAPM_PGA("LAMP", ES8323_ADCCONTROL1, 4, 0, NULL, 0),
	SND_SOC_DAPM_PGA("RAMP", ES8323_ADCCONTROL1, 0, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("LOUT1"),
	SND_SOC_DAPM_OUTPUT("ROUT1"),
	SND_SOC_DAPM_OUTPUT("LOUT2"),
	SND_SOC_DAPM_OUTPUT("ROUT2"),
	SND_SOC_DAPM_OUTPUT("VREF"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"Left PGA Mux", "Line 1L", "LINPUT1"},
	{"Left PGA Mux", "Line 2L", "LINPUT2"},
	{"Left PGA Mux", "DifferentialL", "Differential Mux"},

	{"Right PGA Mux", "Line 1R", "RINPUT1"},
	{"Right PGA Mux", "Line 2R", "RINPUT2"},
	{"Right PGA Mux", "DifferentialR", "Differential Mux"},

	{"Differential Mux", "Line 1", "LINPUT1"},
	{"Differential Mux", "Line 1", "RINPUT1"},
	{"Differential Mux", "Line 2", "LINPUT2"},
	{"Differential Mux", "Line 2", "RINPUT2"},

	{"Left ADC Mux", "Stereo", "Right PGA Mux"},
	{"Left ADC Mux", "Stereo", "Left PGA Mux"},
	{"Left ADC Mux", "Mono (Left)", "Left PGA Mux"},

	{"Right ADC Mux", "Stereo", "Left PGA Mux"},
	{"Right ADC Mux", "Stereo", "Right PGA Mux"},
	{"Right ADC Mux", "Mono (Right)", "Right PGA Mux"},

	{"Left ADC Power", NULL, "Left ADC Mux"},
	{"Right ADC Power", NULL, "Right ADC Mux"},
	{"Left ADC", NULL, "Left ADC Power"},
	{"Right ADC", NULL, "Right ADC Power"},

	{"I2S OUT", NULL, "Left ADC"},
	{"I2S OUT", NULL, "Right ADC"},

	{"Left Line Mux", "Line 1L", "LINPUT1"},
	{"Left Line Mux", "Line 2L", "LINPUT2"},
	{"Left Line Mux", "MicL", "Left PGA Mux"},

	{"Right Line Mux", "Line 1R", "RINPUT1"},
	{"Right Line Mux", "Line 2R", "RINPUT2"},
	{"Right Line Mux", "MicR", "Right PGA Mux"},

	{"Right DAC", NULL, "I2S IN"},
	{"Left DAC", NULL, "I2S IN"},

	{"Left Mixer", "Left Playback Switch", "Left DAC"},
	{"Left Mixer", "Left Bypass Switch", "Left Line Mux"},

	{"Right Mixer", "Right Playback Switch", "Right DAC"},
	{"Right Mixer", "Right Bypass Switch", "Right Line Mux"},

	{"Left Out 1", NULL, "Left Mixer"},
	{"LOUT1", NULL, "Left Out 1"},
	{"Right Out 1", NULL, "Right Mixer"},
	{"ROUT1", NULL, "Right Out 1"},

	{"Left Out 2", NULL, "Left Mixer"},
	{"LOUT2", NULL, "Left Out 2"},
	{"Right Out 2", NULL, "Right Mixer"},
	{"ROUT2", NULL, "Right Out 2"},
};

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
	{2048000, 8000, 256, 0x2, 0x0},
	{4096000, 8000, 512, 0x4, 0x0},
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
	{4096000, 16000, 256, 0x2, 0x0},
	{8192000, 16000, 512, 0x4, 0x0},
	{12288000, 16000, 768, 0x6, 0x0},
	{18432000, 16000, 1152, 0x8, 0x0},
	{12000000, 16000, 750, 0x7, 0x1},

	/* 22.05k */
	{11289600, 22050, 512, 0x4, 0x0},
	{16934400, 22050, 768, 0x6, 0x0},
	{12000000, 22050, 544, 0x6, 0x1},

	/* 32k */
	{8192000, 32000, 256, 0x2, 0x0},
	{16384000, 32000, 512, 0x4, 0x0},
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

/*
 * Note that this should be called from init rather than from hw_params.
 */
static int es8323_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = codec_dai->component;
	struct es8323_priv *es8323 = snd_soc_component_get_drvdata(component);
	int i, ret;
	int count = 0;

	es8323->sysclk = freq;
	if (freq == 0) {
		es8323->sysclk_constraints.list = NULL;
		es8323->sysclk_constraints.count = 0;

		return 0;
	}

	ret = clk_set_rate(es8323->mclk, freq);
	if (ret)
		return ret;

	/* Limit supported sample rates to ones that can be autodetected
	 * by the codec running in slave mode.
	 */
	for (i = 0; i < NR_SUPPORTED_MCLK_LRCK_RATIOS; i++) {
		const unsigned int ratio = supported_mclk_lrck_ratios[i];

		if (freq % ratio == 0)
			es8323->allowed_rates[count++] = freq / ratio;
	}

	es8323->sysclk_constraints.list = es8323->allowed_rates;
	es8323->sysclk_constraints.count = count;

	return 0;
}

static int es8323_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	u8 iface = 0;
	u8 adciface = 0;
	u8 daciface = 0;

	iface = snd_soc_component_read(component, ES8323_IFACE);
	adciface = snd_soc_component_read(component, ES8323_ADC_IFACE);
	daciface = snd_soc_component_read(component, ES8323_DAC_IFACE);

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

	snd_soc_component_write(component, ES8323_IFACE, iface);
	snd_soc_component_write(component, ES8323_ADC_IFACE, adciface);
	snd_soc_component_write(component, ES8323_DAC_IFACE, daciface);

	return 0;
}

static int es8323_pcm_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	return 0;
}

static int es8323_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es8323_priv *es8323 = snd_soc_component_get_drvdata(component);
	u16 srate = snd_soc_component_read(component, ES8323_IFACE) & 0x80;
	u16 adciface = snd_soc_component_read(component, ES8323_ADC_IFACE) & 0xE3;
	u16 daciface = snd_soc_component_read(component, ES8323_DAC_IFACE) & 0xC7;
	int coeff;
	int i;

	/* Validate supported sample rates that are autodetected from MCLK */
	for (i = 0; i < NR_SUPPORTED_MCLK_LRCK_RATIOS; i++) {
		const unsigned int ratio = supported_mclk_lrck_ratios[i];

		if (es8323->sysclk % ratio != 0)
			continue;
		if (es8323->sysclk / ratio == params_rate(params))
			break;
	}
	if (i == NR_SUPPORTED_MCLK_LRCK_RATIOS) {
		dev_err(component->dev,
			"Unsupported sample rate %dHz with %dHz MCLK\n",
			params_rate(params), es8323->sysclk);
		return -EINVAL;
	}
	coeff = get_coeff(es8323->sysclk, params_rate(params));
	if (coeff < 0) {
		coeff = get_coeff(es8323->sysclk / 2, params_rate(params));
		srate |= 0x40;
	}
	if (coeff < 0) {
		dev_err(component->dev,
			"Unable to configure sample rate %dHz with %dHz MCLK\n",
			params_rate(params), es8323->sysclk);
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
	snd_soc_component_write(component, ES8323_DAC_IFACE, daciface);
	snd_soc_component_write(component, ES8323_ADC_IFACE, adciface);

	if (coeff >= 0) {
		snd_soc_component_write(component, ES8323_IFACE, srate);
		snd_soc_component_write(component, ES8323_ADCCONTROL5,
					coeff_div[coeff].sr |
					coeff_div[coeff].usb << 4);
		snd_soc_component_write(component, ES8323_DACCONTROL2,
					coeff_div[coeff].sr |
					coeff_div[coeff].usb << 4);
	}

	return 0;
}

static int es8323_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	return 0;
}

static int es8323_set_bias_level(struct snd_soc_component *component,
				 enum snd_soc_bias_level level)
{
	struct es8323_priv *es8323 = snd_soc_component_get_drvdata(component);
	int ret;

	switch (level) {
	case SND_SOC_BIAS_ON:
		dev_dbg(component->dev, "%s on\n", __func__);
		break;
	case SND_SOC_BIAS_PREPARE:
		dev_dbg(component->dev, "%s prepare\n", __func__);
		if (IS_ERR(es8323->mclk))
			break;
		if (snd_soc_component_get_bias_level(component) == SND_SOC_BIAS_ON) {
			clk_disable_unprepare(es8323->mclk);
		} else {
			ret = clk_prepare_enable(es8323->mclk);
			if (ret)
				return ret;
		}
		snd_soc_component_write(component, ES8323_ANAVOLMANAG, 0x7C);
		snd_soc_component_write(component, ES8323_CHIPLOPOW1, 0x00);
		snd_soc_component_write(component, ES8323_CHIPLOPOW2, 0x00);
		snd_soc_component_write(component, ES8323_CHIPPOWER, 0x00);
		snd_soc_component_write(component, ES8323_ADCPOWER, 0x59);
		break;
	case SND_SOC_BIAS_STANDBY:
		dev_dbg(component->dev, "%s standby\n", __func__);
		snd_soc_component_write(component, ES8323_ANAVOLMANAG, 0x7C);
		snd_soc_component_write(component, ES8323_CHIPLOPOW1, 0x00);
		snd_soc_component_write(component, ES8323_CHIPLOPOW2, 0x00);
		snd_soc_component_write(component, ES8323_CHIPPOWER, 0x00);
		snd_soc_component_write(component, ES8323_ADCPOWER, 0x59);
		break;
	case SND_SOC_BIAS_OFF:
		dev_dbg(component->dev, "%s off\n", __func__);
		snd_soc_component_write(component, ES8323_ADCPOWER, 0xFF);
		snd_soc_component_write(component, ES8323_DACPOWER, 0xC0);
		snd_soc_component_write(component, ES8323_CHIPLOPOW1, 0xFF);
		snd_soc_component_write(component, ES8323_CHIPLOPOW2, 0xFF);
		snd_soc_component_write(component, ES8323_CHIPPOWER, 0xFF);
		snd_soc_component_write(component, ES8323_ANAVOLMANAG, 0x7B);
		break;
	}
	return 0;
}

#define es8323_RATES SNDRV_PCM_RATE_8000_96000

#define es8323_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops es8323_ops = {
	.startup = es8323_pcm_startup,
	.hw_params = es8323_pcm_hw_params,
	.set_fmt = es8323_set_dai_fmt,
	.set_sysclk = es8323_set_dai_sysclk,
	.mute_stream = es8323_mute,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver es8323_dai = {
	.name = "ES8323 HiFi",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = es8323_RATES,
		     .formats = es8323_FORMATS,
		     },
	.capture = {
		    .stream_name = "Capture",
		    .channels_min = 1,
		    .channels_max = 2,
		    .rates = es8323_RATES,
		    .formats = es8323_FORMATS,
		    },
	.ops = &es8323_ops,
	.symmetric_rates = 1,
};

static int es8323_suspend(struct snd_soc_component *component)
{
	struct es8323_priv *es8323 = snd_soc_component_get_drvdata(component);

	snd_soc_component_write(component, 0x19, 0x06);
	snd_soc_component_write(component, 0x30, 0x00);
	snd_soc_component_write(component, 0x31, 0x00);
	snd_soc_component_write(component, ES8323_ADCPOWER, 0xFF);
	snd_soc_component_write(component, ES8323_DACPOWER, 0xc0);
	snd_soc_component_write(component, ES8323_CHIPPOWER, 0xF3);
	snd_soc_component_write(component, 0x00, 0x00);
	snd_soc_component_write(component, 0x01, 0x58);
	snd_soc_component_write(component, 0x2b, 0x9c);
	usleep_range(18000, 20000);
	regcache_cache_only(es8323->regmap, true);
	regcache_mark_dirty(es8323->regmap);
	return 0;
}

static int es8323_resume(struct snd_soc_component *component)
{
	struct es8323_priv *es8323 = snd_soc_component_get_drvdata(component);

	regcache_cache_only(es8323->regmap, false);
	snd_soc_component_cache_sync(component);
	snd_soc_component_write(component, 0x2b, 0x80);
	snd_soc_component_write(component, 0x01, 0x50);
	snd_soc_component_write(component, 0x00, 0x32);
	snd_soc_component_write(component, ES8323_CHIPPOWER, 0x00);
	snd_soc_component_write(component, ES8323_DACPOWER, 0x0c);
	snd_soc_component_write(component, ES8323_ADCPOWER, 0x59);
	snd_soc_component_write(component, 0x31, es8323_DEF_VOL);
	snd_soc_component_write(component, 0x30, es8323_DEF_VOL);
	snd_soc_component_write(component, 0x19, 0x02);
	return 0;
}

static int es8323_probe(struct snd_soc_component *component)
{
	struct es8323_priv *es8323 = snd_soc_component_get_drvdata(component);
	int ret = 0;

	es8323->mclk = devm_clk_get(component->dev, "mclk");
	if (IS_ERR(es8323->mclk)) {
		dev_err(component->dev, "%s mclk is missing or invalid\n", __func__);
		return PTR_ERR(es8323->mclk);
	}
	ret = clk_prepare_enable(es8323->mclk);
	if (ret)
		return ret;
	es8323->component = component;

	ret = es8323_reset(component);
	if (ret < 0) {
		dev_err(component->dev, "Failed to issue reset\n");
		clk_disable_unprepare(es8323->mclk);
		return ret;
	}

	snd_soc_component_write(component, 0x01, 0x60);
	snd_soc_component_write(component, 0x02, 0xF3);
	snd_soc_component_write(component, 0x02, 0xF0);
	snd_soc_component_write(component, 0x2B, 0x80);
	snd_soc_component_write(component, 0x00, 0x36);
	snd_soc_component_write(component, 0x08, 0x00);
	snd_soc_component_write(component, 0x04, 0x00);
	snd_soc_component_write(component, 0x06, 0xC3);
	snd_soc_component_write(component, 0x19, 0x02);
	snd_soc_component_write(component, 0x09, 0x00);
	snd_soc_component_write(component, 0x0A, 0x00);
	snd_soc_component_write(component, 0x0B, 0x02);
	snd_soc_component_write(component, 0x0C, 0x4C);
	snd_soc_component_write(component, 0x0D, 0x02);
	snd_soc_component_write(component, 0x10, 0x00);
	snd_soc_component_write(component, 0x11, 0x00);
	snd_soc_component_write(component, 0x12, 0xea);
	snd_soc_component_write(component, 0x13, 0xc0);
	snd_soc_component_write(component, 0x14, 0x05);
	snd_soc_component_write(component, 0x15, 0x06);
	snd_soc_component_write(component, 0x16, 0x53);

	snd_soc_component_write(component, 0x17, 0x18);
	snd_soc_component_write(component, 0x18, 0x02);
	snd_soc_component_write(component, 0x1A, 0x00);
	snd_soc_component_write(component, 0x1B, 0x00);
	snd_soc_component_write(component, 0x27, 0xB8);
	snd_soc_component_write(component, 0x2A, 0xB8);
	snd_soc_component_write(component, 0x35, 0xA0);
	usleep_range(18000, 20000);
	snd_soc_component_write(component, 0x2E, 0x1E);
	snd_soc_component_write(component, 0x2F, 0x1E);
	snd_soc_component_write(component, 0x30, 0x1E);
	snd_soc_component_write(component, 0x31, 0x1E);
	snd_soc_component_write(component, 0x03, 0x09);
	snd_soc_component_write(component, 0x02, 0x00);
	usleep_range(18000, 20000);
	snd_soc_component_write(component, 0x04, 0x3C);

	es8323_set_bias_level(component, SND_SOC_BIAS_STANDBY);
	return 0;
}

static void es8323_remove(struct snd_soc_component *component)
{
	es8323_set_bias_level(component, SND_SOC_BIAS_OFF);
}

static const struct snd_soc_component_driver soc_codec_dev_es8323 = {
	.probe = es8323_probe,
	.remove = es8323_remove,
	.suspend = es8323_suspend,
	.resume = es8323_resume,
	.set_bias_level = es8323_set_bias_level,
	.dapm_widgets		= es8323_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es8323_dapm_widgets),
	.dapm_routes		= audio_map,
	.num_dapm_routes	= ARRAY_SIZE(audio_map),
	.controls		= es8323_snd_controls,
	.num_controls		= ARRAY_SIZE(es8323_snd_controls),
};

static const struct regmap_config es8323_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= ES8323_DACCONTROL30,
	.cache_type	= REGCACHE_RBTREE,
	.reg_defaults	= es8323_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(es8323_reg_defaults),
	.use_single_read = true,
	.use_single_write = true,
};

static int es8323_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct es8323_priv *es8323;
	int ret = -1;
	struct i2c_adapter *adapter = to_i2c_adapter(i2c->dev.parent);
	char reg;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_I2C\n");
		return -EIO;
	}

	es8323 = devm_kzalloc(&i2c->dev, sizeof(struct es8323_priv), GFP_KERNEL);
	if (!es8323)
		return -ENOMEM;

	es8323->regmap = devm_regmap_init_i2c(i2c, &es8323_regmap_config);
	if (IS_ERR(es8323->regmap))
		return PTR_ERR(es8323->regmap);

	i2c_set_clientdata(i2c, es8323);

	reg = ES8323_DACCONTROL18;
	ret = i2c_master_recv(i2c, &reg, 1);
	if (ret < 0) {
		dev_err(&i2c->dev, "i2c recv Failed\n");
		return ret;
	}

	ret = devm_snd_soc_register_component(&i2c->dev,
					      &soc_codec_dev_es8323,
					      &es8323_dai, 1);
	return ret;
}

static int es8323_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_component(&client->dev);
	return 0;
}

static const struct i2c_device_id es8323_i2c_id[] = {
	{"es8323", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, es8323_i2c_id);

static void es8323_i2c_shutdown(struct i2c_client *client)
{
	struct es8323_priv *es8323 = i2c_get_clientdata(client);

	regmap_write(es8323->regmap, ES8323_CONTROL2, 0x58);
	regmap_write(es8323->regmap, ES8323_CONTROL1, 0x32);
	regmap_write(es8323->regmap, ES8323_CHIPPOWER, 0xf3);
	regmap_write(es8323->regmap, ES8323_DACPOWER, 0xc0);
	mdelay(50);
	regmap_write(es8323->regmap, ES8323_DACCONTROL26, 0x00);
	regmap_write(es8323->regmap, ES8323_DACCONTROL27, 0x00);
	mdelay(50);
	regmap_write(es8323->regmap, ES8323_CONTROL1, 0x30);
	regmap_write(es8323->regmap, ES8323_CONTROL1, 0x34);
}

static const struct of_device_id es8323_of_match[] = {
	{ .compatible = "everest,es8323", },
	{ }
};
MODULE_DEVICE_TABLE(of, es8323_of_match);

static struct i2c_driver es8323_i2c_driver = {
	.driver = {
		.name = "ES8323",
		.of_match_table = of_match_ptr(es8323_of_match),
		},
	.shutdown = es8323_i2c_shutdown,
	.probe = es8323_i2c_probe,
	.remove = es8323_i2c_remove,
	.id_table = es8323_i2c_id,
};
module_i2c_driver(es8323_i2c_driver);

MODULE_DESCRIPTION("ASoC es8323 driver");
MODULE_AUTHOR("Mark Brown <will@everset-semi.com>");
MODULE_LICENSE("GPL");
