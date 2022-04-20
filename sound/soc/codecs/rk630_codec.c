// SPDX-License-Identifier: GPL-2.0
//
// rk630_codec.c  --  rk630 ALSA Soc Audio driver
//
// Copyright (c) 2022 Rockchip Electronics Co. Ltd.
// Author: Xinhuang Li <buluess.li@rock-chips.com>
// Author: Yanchao Hu <yanchao.hu@rock-chips.com>
//
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mfd/rk630.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/rockchip/grf.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include "rk630_codec.h"

struct rk630_codec_priv {
	struct regmap *regmap;
	struct regmap *grf;
	struct regmap *cru;
	struct device *dev;
	unsigned int irq;
	struct snd_soc_component *component;
	unsigned int stereo_sysclk;
	unsigned int rate;
	bool mic_in_differential;
	struct gpio_desc *spk_ctl_gpio;
	struct clk *pclk;
	struct clk *mclk;
};

static int rk630_init(struct snd_soc_component *component)
{
	struct rk630_codec_priv *rk630 = snd_soc_component_get_drvdata(component);

	snd_soc_component_write(component, RK630_RESET, 0x00);
	mdelay(10);
	snd_soc_component_write(component, RK630_RESET, 0x43);
	mdelay(10);

	snd_soc_component_update_bits(component, RK630_BSTL_ALCL_CTL,
			RK630_BSTL_MODE_EN, RK630_BSTL_MODE_SINGLE);
	dev_dbg(component->dev, "%s  mic_in_differential = %d\n",
			__func__, rk630->mic_in_differential);
	if (rk630->mic_in_differential) {
		snd_soc_component_update_bits(component, RK630_BSTL_ALCL_CTL,
				RK630_BSTL_MODE_EN, RK630_BSTL_MODE_DIFF);
		snd_soc_component_update_bits(component, RK630_ALC_MUNIN_CTL,
				RK630_ALCR_MUTE_SHT, RK630_ALCR_DIS);
	}

	return 0;
}

static const struct reg_default rk630_reg_defaults[] = {
	{ RK630_RESET, 0x0003 },
	{ RK630_ADC_INT_CTL1, 0x0050 },
	{ RK630_ADC_INT_CTL2, 0x000e },
	{ RK630_DAC_INT_CTL1, 0x0050 },
	{ RK630_DAC_INT_CTL2, 0x000e },
	{ RK630_DAC_INT_CTL3, 0x22 },
	{ RK630_ADC_MIC_CTL, 0x0000 },
	{ RK630_BST_CTL, 0x000 },
	{ RK630_ALC_MUNIN_CTL, 0x0044 },
	{ RK630_BSTL_ALCL_CTL, 0x000c },
	{ RK630_ALCR_GAIN_CTL, 0x000C },
	{ RK630_ADC_ENABLE, 0x0000 },
	{ RK630_DAC_CTL, 0x0000 },
	{ RK630_DAC_ENABLE, 0x0000 },
	{ RK630_HPMIX_CTL, 0x0000 },
	{ RK630_HPMIX_S_SELECT, 0x0000 },
	{ RK630_HPOUT_CTL, 0x0000 },
	{ RK630_HPOUTL_GAIN, 0x0000 },
	{ RK630_HPOUTR_GAIN, 0x0000 },
	{ RK630_SELECT_CURRENT, 0x003e },
	{ RK630_PGAL_AGC_CTL1, 0x0000 },
	{ RK630_PGAL_AGC_CTL2, 0x0046 },
	{ RK630_PGAL_AGC_CTL3, 0x0041 },
	{ RK630_PGAL_AGC_CTL4, 0x000c },
	{ RK630_PGAL_ASR_CTL, 0x0000 },
	{ RK630_PGAL_AGC_MAX_H, 0x0026 },
	{ RK630_PGAL_AGC_MAX_L, 0x0040 },
	{ RK630_PGAL_AGC_MIN_H, 0x0036 },
	{ RK630_PGAL_AGC_MIN_L, 0x0020 },
	{ RK630_PGAL_AGC_CTL5, 0x0038 },
	{ RK630_PGAR_AGC_CTL1, 0x0000 },
	{ RK630_PGAR_AGC_CTL2, 0x0046 },
	{ RK630_PGAR_AGC_CTL3, 0x0041 },
	{ RK630_PGAR_AGC_CTL4, 0x000c },
	{ RK630_PGAR_ASR_CTL, 0x0000 },
	{ RK630_PGAR_AGC_MAX_H, 0x0026 },
	{ RK630_PGAR_AGC_MAX_L, 0x0040 },
	{ RK630_PGAR_AGC_MIN_H, 0x0036 },
	{ RK630_PGAR_AGC_MIN_L, 0x0020 },
	{ RK630_PGAR_AGC_CTL5, 0x0038 },
};

static bool rk630_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case RK630_RESET:
	case RK630_ADC_INT_CTL2:
		return true;
	default:
		return false;
	}
}

static bool rk630_codec_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case RK630_RESET:
	case RK630_ADC_INT_CTL1:
	case RK630_ADC_INT_CTL2:
	case RK630_DAC_INT_CTL1:
	case RK630_DAC_INT_CTL2:
	case RK630_DAC_INT_CTL3:
	case RK630_ADC_MIC_CTL:
	case RK630_BST_CTL:
	case RK630_ALC_MUNIN_CTL:
	case RK630_BSTL_ALCL_CTL:
	case RK630_ALCR_GAIN_CTL:
	case RK630_ADC_ENABLE:
	case RK630_DAC_CTL:
	case RK630_DAC_ENABLE:
	case RK630_HPMIX_CTL:
	case RK630_HPMIX_S_SELECT:
	case RK630_HPOUT_CTL:
	case RK630_HPOUTL_GAIN:
	case RK630_HPOUTR_GAIN:
	case RK630_SELECT_CURRENT:
	case RK630_PGAL_AGC_CTL1:
	case RK630_PGAL_AGC_CTL2:
	case RK630_PGAL_AGC_CTL3:
	case RK630_PGAL_AGC_CTL4:
	case RK630_PGAL_ASR_CTL:
	case RK630_PGAL_AGC_MAX_H:
	case RK630_PGAL_AGC_MAX_L:
	case RK630_PGAL_AGC_MIN_H:
	case RK630_PGAL_AGC_MIN_L:
	case RK630_PGAL_AGC_CTL5:
	case RK630_PGAR_AGC_CTL1:
	case RK630_PGAR_AGC_CTL2:
	case RK630_PGAR_AGC_CTL3:
	case RK630_PGAR_AGC_CTL4:
	case RK630_PGAR_ASR_CTL:
	case RK630_PGAR_AGC_MAX_H:
	case RK630_PGAR_AGC_MAX_L:
	case RK630_PGAR_AGC_MIN_H:
	case RK630_PGAR_AGC_MIN_L:
	case RK630_PGAR_AGC_CTL5:
	case RK630_ALC_CTL:
		return true;
	default:
		return false;
	}
}

static int rk630_set_bias_level(struct snd_soc_component *component,
				enum snd_soc_bias_level level)
{
	dev_dbg(component->dev, "%s  level=%d\n", __func__, level);
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		break;

	case SND_SOC_BIAS_STANDBY:
		if (snd_soc_component_get_bias_level(component) == SND_SOC_BIAS_OFF) {
			snd_soc_component_write(component, RK630_DAC_INT_CTL3, 0x32);
			snd_soc_component_update_bits(component, RK630_ADC_MIC_CTL,
						      RK630_ADC_CURRENT_ENABLE,
						      RK630_ADC_CURRENT_ENABLE);
			snd_soc_component_update_bits(component, RK630_DAC_CTL,
						      RK630_CURRENT_EN,
						      RK630_CURRENT_EN);
			/* set power */
			snd_soc_component_update_bits(component, RK630_ADC_ENABLE,
						      RK630_ADCL_REF_VOL_EN |
						      RK630_ADCR_REF_VOL_EN,
						      RK630_ADCL_REF_VOL_EN |
						      RK630_ADCR_REF_VOL_EN);

			snd_soc_component_update_bits(component, RK630_ADC_MIC_CTL,
						      RK630_ADCL_ZERO_DET_EN |
						      RK630_ADCR_ZERO_DET_EN,
						      RK630_ADCL_ZERO_DET_EN |
						      RK630_ADCR_ZERO_DET_EN);

			snd_soc_component_update_bits(component, RK630_DAC_CTL,
						      RK630_REF_VOL_DACL_EN |
						      RK630_REF_VOL_DACR_EN,
						      RK630_REF_VOL_DACL_EN |
						      RK630_REF_VOL_DACR_EN);

			snd_soc_component_update_bits(component, RK630_DAC_ENABLE,
						      RK630_DACL_REF_VOL_EN |
						      RK630_DACR_REF_VOL_EN,
						      RK630_DACL_REF_VOL_EN |
						      RK630_DACR_REF_VOL_EN);
		}
		break;

	case SND_SOC_BIAS_OFF:
		snd_soc_component_update_bits(component, RK630_DAC_ENABLE,
					      RK630_DACL_REF_VOL_EN |
					      RK630_DACR_REF_VOL_EN,
					      0);
		snd_soc_component_update_bits(component, RK630_DAC_CTL,
					      RK630_REF_VOL_DACL_EN |
					      RK630_REF_VOL_DACR_EN,
					      0);
		snd_soc_component_update_bits(component, RK630_ADC_MIC_CTL,
					      RK630_ADCL_ZERO_DET_EN |
					      RK630_ADCR_ZERO_DET_EN,
					      0);
		snd_soc_component_update_bits(component, RK630_ADC_ENABLE,
					      RK630_ADCL_REF_VOL_EN |
					      RK630_ADCR_REF_VOL_EN,
					      0);
		snd_soc_component_update_bits(component, RK630_ADC_MIC_CTL,
					      RK630_ADC_CURRENT_ENABLE,
					      0);
		snd_soc_component_update_bits(component, RK630_DAC_CTL,
					      RK630_CURRENT_EN,
					      0);
		snd_soc_component_write(component, RK630_DAC_INT_CTL3, 0x22);
		break;
	}
	return 0;
}

static const DECLARE_TLV_DB_SCALE(bst_vol_tlv, 0, 2000, 0);
static const DECLARE_TLV_DB_MINMAX(pga_vol_tlv, -1800, 2850);
static const DECLARE_TLV_DB_MINMAX(out_vol_tlv, -3900, 600);

static const struct snd_kcontrol_new rk630_snd_controls[] = {
	SOC_SINGLE_TLV("BSTL Capture Volume", RK630_BST_CTL,
			RK630_BSTL_GAIN_SHT, 1, 0, bst_vol_tlv),
	SOC_SINGLE_TLV("BSTR Capture Volume", RK630_BST_CTL,
			RK630_BSTR_GAIN_SHT, 1, 0, bst_vol_tlv),
	SOC_DOUBLE_R_RANGE_TLV("PGA AGC Volume", RK630_PGAL_AGC_CTL4,
		RK630_PGAR_AGC_CTL4, 0, 0x00, 0x1f, 0, pga_vol_tlv),
	SOC_DOUBLE_R_RANGE_TLV("Playback Volume", RK630_HPOUTL_GAIN,
		RK630_HPOUTR_GAIN, 0, 0x00, 0x1f, 0, out_vol_tlv),

	SOC_SINGLE("ALCL Switch", RK630_ALC_MUNIN_CTL,
		    RK630_ALCL_MUTE_SHT, 1, 0),
	SOC_SINGLE("ALCR Switch", RK630_ALC_MUNIN_CTL,
		    RK630_ALCR_MUTE_SHT, 1, 0),
};

static int rk630_dacl_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol,
			     int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component, RK630_DAC_ENABLE,
				    RK630_DACL_WORK, 0);
		snd_soc_component_update_bits(component, RK630_DAC_ENABLE,
				    RK630_DACL_EN | RK630_DACL_CLK_EN,
				    RK630_DACL_EN | RK630_DACL_CLK_EN);
		snd_soc_component_update_bits(component, RK630_DAC_ENABLE,
				    RK630_DACL_WORK, RK630_DACL_WORK);
		break;

	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, RK630_DAC_ENABLE,
				    RK630_DACL_EN
				    | RK630_DACL_CLK_EN, 0);
		snd_soc_component_update_bits(component, RK630_DAC_ENABLE,
				    RK630_DACL_WORK, 0);
		break;

	default:
		return 0;
	}

	return 0;
}

static int rk630_dacr_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol,
			     int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component, RK630_DAC_ENABLE,
				    RK630_DACR_WORK, 0);
		snd_soc_component_update_bits(component, RK630_DAC_ENABLE,
				    RK630_DACR_EN
				    | RK630_DACR_CLK_EN,
				    RK630_DACR_EN
				    | RK630_DACR_CLK_EN);
		snd_soc_component_update_bits(component, RK630_DAC_ENABLE,
				    RK630_DACR_WORK,
				    RK630_DACR_WORK);
		break;

	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, RK630_DAC_ENABLE,
				    RK630_DACR_EN
				    | RK630_DACR_CLK_EN, 0);
		snd_soc_component_update_bits(component, RK630_DAC_ENABLE,
				    RK630_DACR_WORK, 0);
		break;

	default:
		return 0;
	}

	return 0;
}

static int rk630_adcl_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component, RK630_ADC_ENABLE,
				    RK630_ADCL_CLK_EN
				    | RK630_ADCL_AMP_EN,
				    RK630_ADCL_CLK_EN
				    | RK630_ADCL_AMP_EN);
		break;

	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, RK630_ADC_ENABLE,
				    RK630_ADCL_CLK_EN
				    | RK630_ADCL_AMP_EN, 0);
		break;

	default:
		return 0;
	}

	return 0;
}

static int rk630_adcr_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component, RK630_ADC_ENABLE,
				    RK630_ADCR_CLK_EN
				    | RK630_ADCR_AMP_EN,
				    RK630_ADCR_CLK_EN
				    | RK630_ADCR_AMP_EN);
		break;

	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component, RK630_ADC_ENABLE,
				    RK630_ADCR_CLK_EN
				    | RK630_ADCR_AMP_EN, 0);
		break;

	default:
		return 0;
	}

	return 0;
}

/* HPmix */
static const struct snd_kcontrol_new rk630_hpmixl[] = {
	SOC_DAPM_SINGLE("ALCR Switch", RK630_HPMIX_S_SELECT,
			RK630_HPMIXL_SEL_ALCR_SFT, 1, 0),
	SOC_DAPM_SINGLE("ALCL Switch", RK630_HPMIX_S_SELECT,
			RK630_HPMIXL_SEL_ALCL_SFT, 1, 0),
	SOC_DAPM_SINGLE("DACL Switch", RK630_HPMIX_S_SELECT,
			RK630_HPMIXL_SEL_DACL_SFT, 1, 0),
};

static const struct snd_kcontrol_new rk630_hpmixr[] = {
	SOC_DAPM_SINGLE("ALCR Switch", RK630_HPMIX_S_SELECT,
			RK630_HPMIXR_SEL_ALCR_SFT, 1, 0),
	SOC_DAPM_SINGLE("ALCL Switch", RK630_HPMIX_S_SELECT,
			RK630_HPMIXR_SEL_ALCL_SFT, 1, 0),
	SOC_DAPM_SINGLE("DACR Switch", RK630_HPMIX_S_SELECT,
			RK630_HPMIXR_SEL_DACR_SFT, 1, 0),
};

static int rk630_hpmixl_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component, RK630_DAC_CTL,
				    RK630_ZO_DET_VOUTL_EN,
				    RK630_ZO_DET_VOUTL_EN);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(component, RK630_DAC_CTL,
				    RK630_ZO_DET_VOUTL_EN,
				    RK630_ZO_DET_VOUTL_DIS);
		break;

	default:
		return 0;
	}

	return 0;
}

static int rk630_hpmixr_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_component_update_bits(component, RK630_DAC_CTL,
				    RK630_ZO_DET_VOUTR_EN,
				    RK630_ZO_DET_VOUTR_EN);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_component_update_bits(component, RK630_DAC_CTL,
				    RK630_ZO_DET_VOUTR_EN,
				    RK630_ZO_DET_VOUTR_DIS);
		break;

	default:
		return 0;
	}

	return 0;
}

/* HP MUX */
static const char *const hpl_sel[] = {"HPMIXL", "DACL"};

static const struct soc_enum hpl_sel_enum =
	SOC_ENUM_SINGLE(RK630_HPMIX_S_SELECT, RK630_HPMIXL_BYPASS_SFT,
			ARRAY_SIZE(hpl_sel), hpl_sel);

static const struct snd_kcontrol_new hpl_sel_mux =
	SOC_DAPM_ENUM("HPL select Mux", hpl_sel_enum);

static const char *const hpr_sel[] = {"HPMIXR", "DACR"};

static const struct soc_enum hpr_sel_enum =
	SOC_ENUM_SINGLE(RK630_HPMIX_S_SELECT, RK630_HPMIXR_BYPASS_SFT,
			ARRAY_SIZE(hpr_sel), hpr_sel);

static const struct snd_kcontrol_new hpr_sel_mux =
	SOC_DAPM_ENUM("HPR select Mux", hpr_sel_enum);

/* IN_L MUX */
static const char *const lnl_sel[] = {"NO", "BSTL", "LINEL", "NOUSE"};

static const struct soc_enum lnl_sel_enum =
	SOC_ENUM_SINGLE(RK630_ALC_MUNIN_CTL, RK630_MUXINL_F_SHT,
			ARRAY_SIZE(lnl_sel), lnl_sel);

static const struct snd_kcontrol_new lnl_sel_mux =
	SOC_DAPM_ENUM("MUXIN_L select", lnl_sel_enum);

/* IN_R MUX */
static const char *const lnr_sel[] = {"NO", "BSTR", "LINER", "NOUSE"};

static const struct soc_enum lnr_sel_enum =
	SOC_ENUM_SINGLE(RK630_ALC_MUNIN_CTL, RK630_MUXINR_F_SHT,
			ARRAY_SIZE(lnr_sel), lnr_sel);

static const struct snd_kcontrol_new lnr_sel_mux =
	SOC_DAPM_ENUM("MUXIN_R select", lnr_sel_enum);

static const struct snd_soc_dapm_widget rk630_dapm_widgets[] = {
	/* microphone bias */
	SND_SOC_DAPM_SUPPLY("Mic Bias", RK630_ADC_MIC_CTL, RK630_MICBIAS_VOL_ENABLE, 0, NULL, 0),

	/* ADCs */
	SND_SOC_DAPM_ADC_E("ADCL", NULL, SND_SOC_NOPM,
			   0, 0, rk630_adcl_event,
			   SND_SOC_DAPM_POST_PMD
			   | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_ADC_E("ADCR", NULL, SND_SOC_NOPM,
			   0, 0, rk630_adcr_event,
			   SND_SOC_DAPM_POST_PMD
			   | SND_SOC_DAPM_POST_PMU),

	/* PGA */
	SND_SOC_DAPM_PGA("BSTL", RK630_BST_CTL, RK630_BSTL_PWRD_SFT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("BSTR", RK630_BST_CTL, RK630_BSTR_PWRD_SFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("BSTL Mute", RK630_BST_CTL, RK630_BSTL_MUTE_SHT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("BSTR Mute", RK630_BST_CTL, RK630_BSTR_MUTE_SHT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ALCL", RK630_ALC_MUNIN_CTL, RK630_ALCL_PWR_SHT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ALCR", RK630_ALC_MUNIN_CTL, RK630_ALCR_PWR_SHT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ALCL Mute", RK630_ALC_MUNIN_CTL, RK630_ALCL_MUTE_SHT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ALCR Mute", RK630_ALC_MUNIN_CTL, RK630_ALCR_MUTE_SHT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ALCL Ctl", RK630_ALC_CTL, 5, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ALCR Ctl", RK630_ALC_CTL, 4, 0, NULL, 0),

	/* DACs */
	SND_SOC_DAPM_DAC_E("DACL", NULL, SND_SOC_NOPM,
			   0, 0, rk630_dacl_event,
			   SND_SOC_DAPM_POST_PMD
			   | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_DAC_E("DACR", NULL, SND_SOC_NOPM,
			   0, 0, rk630_dacr_event,
			   SND_SOC_DAPM_POST_PMD
			   | SND_SOC_DAPM_POST_PMU),

	/* PGA */
	SND_SOC_DAPM_PGA("HPL", RK630_HPOUT_CTL,
			 RK630_HPOUTL_PWR_SHT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("HPR", RK630_HPOUT_CTL,
			 RK630_HPOUTR_PWR_SHT, 0, NULL, 0),

	/* MIXER */
	SND_SOC_DAPM_MIXER_E("HPMIXL", RK630_HPMIX_CTL,
			     RK630_HPMIXL_SFT, 0,
			     rk630_hpmixl,
			     ARRAY_SIZE(rk630_hpmixl),
			     rk630_hpmixl_event,
			     SND_SOC_DAPM_PRE_PMD
			     | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("HPMIXR", RK630_HPMIX_CTL,
			     RK630_HPMIXR_SFT, 0,
			     rk630_hpmixr,
			     ARRAY_SIZE(rk630_hpmixr),
			     rk630_hpmixr_event,
			     SND_SOC_DAPM_PRE_PMD
			     | SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_SUPPLY("HPR Init", RK630_HPOUT_CTL, 3, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("HPL Init", RK630_HPOUT_CTL, 6, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DACR Mute Off", RK630_HPOUT_CTL, 2, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("DACL Mute Off", RK630_HPOUT_CTL, 5, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("HPR Vref", RK630_HPOUT_CTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("HPL Vref", RK630_HPOUT_CTL, 1, 0, NULL, 0),

	/* MUX */
	SND_SOC_DAPM_MUX("IN_R Mux", SND_SOC_NOPM, 0, 0, &lnr_sel_mux),
	SND_SOC_DAPM_MUX("IN_L Mux", SND_SOC_NOPM, 0, 0, &lnl_sel_mux),
	SND_SOC_DAPM_MUX("HPL Mux", SND_SOC_NOPM, 0, 0, &hpl_sel_mux),
	SND_SOC_DAPM_MUX("HPR Mux", SND_SOC_NOPM, 0, 0, &hpr_sel_mux),

	/* Audio Interface */
	SND_SOC_DAPM_AIF_OUT("I2S DAC", "HiFi Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("I2S ADC", "HiFi Capture", 0, SND_SOC_NOPM, 0, 0),

	/* Input */
	SND_SOC_DAPM_INPUT("LINEL"),
	SND_SOC_DAPM_INPUT("LINER"),
	SND_SOC_DAPM_INPUT("MICP"),
	SND_SOC_DAPM_INPUT("MICN"),

	/* Output */
	SND_SOC_DAPM_OUTPUT("HPOUTL"),
	SND_SOC_DAPM_OUTPUT("HPOUTR"),
};

static const struct snd_soc_dapm_route rk630_dapm_routes[] = {
	/* Input */
	{"BSTR", NULL, "MICP"},
	{"BSTL", NULL, "MICP"},
	{"BSTL", NULL, "MICN"},

	{"IN_R Mux", "LINER", "LINER"},
	{"IN_R Mux", "BSTR", "BSTR"},
	{"IN_L Mux", "LINEL", "LINEL"},
	{"IN_L Mux", "BSTL", "BSTL"},

	{"ALCL", NULL, "IN_L Mux"},
	{"ALCR", NULL, "IN_R Mux"},

	{"ADCR", NULL, "ALCR"},
	{"ADCL", NULL, "ALCL"},

	{"I2S ADC", NULL, "BSTR Mute"},
	{"I2S ADC", NULL, "BSTL Mute"},
	{"I2S ADC", NULL, "ALCR Mute"},
	{"I2S ADC", NULL, "ALCL Mute"},
	{"I2S ADC", NULL, "ALCR Ctl"},
	{"I2S ADC", NULL, "ALCL Ctl"},

	{"I2S ADC", NULL, "Mic Bias"},

	{"I2S ADC", NULL, "ADCR"},
	{"I2S ADC", NULL, "ADCL"},

	/* Output */
	{"DACR", NULL, "I2S DAC"},
	{"DACL", NULL, "I2S DAC"},

	{"HPMIXR", "ALCR Switch", "ALCR"},
	{"HPMIXR", "ALCL Switch", "ALCL"},
	{"HPMIXR", "DACR Switch", "DACR"},

	{"HPMIXL", "ALCR Switch", "ALCR"},
	{"HPMIXL", "ALCL Switch", "ALCL"},
	{"HPMIXL", "DACL Switch", "DACL"},

	{"HPR", NULL, "HPR Vref"},
	{"HPL", NULL, "HPL Vref"},

	{"DACR", NULL, "DACR Mute Off"},
	{"DACL", NULL, "DACL Mute Off"},

	{"HPR Mux", "DACR", "DACR"},
	{"HPR Mux", "HPMIXR", "HPMIXR"},
	{"HPL Mux", "DACL", "DACL"},
	{"HPL Mux", "HPMIXL", "HPMIXL"},

	{"HPR", NULL, "HPR Mux"},
	{"HPL", NULL, "HPL Mux"},

	{"HPR", NULL, "HPR Init"},
	{"HPL", NULL, "HPL Init"},

	{"HPOUTR", NULL, "HPR"},
	{"HPOUTL", NULL, "HPL"},
};

static int rk630_codec_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *component = codec_dai->component;
	struct rk630_codec_priv *rk630 = snd_soc_component_get_drvdata(component);

	if (!rk630) {
		dev_warn(component->dev, "%s : rk630 is NULL\n", __func__);
		return -EINVAL;
	}

	rk630->stereo_sysclk = freq;

	return 0;
}

static int rk630_codec_set_dai_fmt(struct snd_soc_dai *codec_dai,
			     unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	struct rk630_codec_priv *rk630 = snd_soc_component_get_drvdata(component);
	unsigned int adc_aif1 = 0, adc_aif2 = 0, dac_aif1 = 0, dac_aif2 = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		adc_aif2 |= RK630_I2S_MODE_SLV;
		/*set tx_lrck->lrck,rx_lrck->lrck*/
		regmap_write(rk630->grf,
			     PLUMAGE_GRF_SOC_CON0,
			     ((CON_TX_LRCK_EN_MASK | CON_RX_LRCK_EN_MASK) <<
			     PLUMAGE_GRF_MASK_SHIFT) |
			     (CON_TX_LRCK_EN | CON_RX_LRCK_EN));
		/*sclk oe:1,lrck oe:1*/
		regmap_write(rk630->grf,
			     PLUMAGE_GRF_SOC_CON0,
			     ((CON_SCLK_OE_MASK | CON_I2S_LRCK_OE_MASK) <<
			     PLUMAGE_GRF_MASK_SHIFT) |
			     (CON_SCLK_OE_SLAVE | CON_I2S_LRCK_OE_SLAVE));
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		adc_aif2 |= RK630_I2S_MODE_MST;
		/*set tx_lrck->lrck*/
		regmap_write(rk630->grf,
			     PLUMAGE_GRF_SOC_CON0,
			     ((CON_TX_LRCK_EN_MASK | CON_RX_LRCK_EN_MASK) <<
			     PLUMAGE_GRF_MASK_SHIFT) |
			     (CON_TX_LRCK_EN | CON_RX_LRCK_DIS));
		/*sclk oe:0,lrck oe:0*/
		regmap_write(rk630->grf,
			     PLUMAGE_GRF_SOC_CON0,
			     ((CON_SCLK_OE_MASK | CON_I2S_LRCK_OE_MASK) <<
			     PLUMAGE_GRF_MASK_SHIFT) |
			     (CON_SCLK_OE_MASTER | CON_I2S_LRCK_OE_MASTER));
		break;
	default:
		dev_warn(component->dev, "%s : set master mask failed!\n", __func__);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		adc_aif1 |= RK630_ADC_DF_PCM;
		dac_aif1 |= RK630_DAC_DF_PCM;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		break;
	case SND_SOC_DAIFMT_I2S:
		adc_aif1 |= RK630_ADC_DF_I2S;
		dac_aif1 |= RK630_DAC_DF_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		adc_aif1 |= RK630_ADC_DF_RJ;
		dac_aif1 |= RK630_DAC_DF_RJ;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		adc_aif1 |= RK630_ADC_DF_LJ;
		dac_aif1 |= RK630_DAC_DF_LJ;
		break;
	default:
		dev_warn(component->dev, "%s : set format failed!\n", __func__);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		adc_aif1 |= RK630_ALRCK_POL_DIS;
		adc_aif2 |= RK630_ABCLK_POL_DIS;
		dac_aif1 |= RK630_DLRCK_POL_DIS;
		dac_aif2 |= RK630_DBCLK_POL_DIS;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		adc_aif1 |= RK630_ALRCK_POL_EN;
		adc_aif2 |= RK630_ABCLK_POL_EN;
		dac_aif1 |= RK630_DLRCK_POL_EN;
		dac_aif2 |= RK630_DBCLK_POL_EN;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		adc_aif1 |= RK630_ALRCK_POL_DIS;
		adc_aif2 |= RK630_ABCLK_POL_EN;
		dac_aif1 |= RK630_DLRCK_POL_DIS;
		dac_aif2 |= RK630_DBCLK_POL_EN;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		adc_aif1 |= RK630_ALRCK_POL_EN;
		adc_aif2 |= RK630_ABCLK_POL_DIS;
		dac_aif1 |= RK630_DLRCK_POL_EN;
		dac_aif2 |= RK630_DBCLK_POL_DIS;
		break;
	default:
		dev_warn(component->dev, "%s : set dai format failed!\n", __func__);
		return -EINVAL;
	}

	snd_soc_component_update_bits(component, RK630_ADC_INT_CTL1,
				      RK630_ALRCK_POL_MASK |
				      RK630_ADC_DF_MASK, adc_aif1);
	snd_soc_component_update_bits(component, RK630_ADC_INT_CTL2,
				      RK630_ABCLK_POL_MASK |
				      RK630_I2S_MODE_MASK, adc_aif2);
	snd_soc_component_update_bits(component, RK630_DAC_INT_CTL1,
				      RK630_DLRCK_POL_MASK |
				      RK630_DAC_DF_MASK, dac_aif1);
	snd_soc_component_update_bits(component, RK630_DAC_INT_CTL2,
				      RK630_DBCLK_POL_MASK, dac_aif2);

	return 0;
}

static int rk630_codec_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component = rtd->codec_dai->component;
	struct rk630_codec_priv *rk630 = snd_soc_component_get_drvdata(component);
	unsigned int rate = params_rate(params);
	unsigned int div, approximate_sample;
	unsigned int adc_aif1 = 0, adc_aif2  = 0, dac_aif1 = 0, dac_aif2  = 0;

	if (!rk630) {
		dev_warn(component->dev, "%s : rk630 is NULL\n", __func__);
		return -EINVAL;
	}

	/* bclk = codec_clk / 4 */
	/* lrck = bclk / (wl * 2) */
	div = (((rk630->stereo_sysclk / 4) / rate) / 2);
	if ((rk630->stereo_sysclk % (4 * rate * 2) > 0) ||
	    (div != 16 && div != 20 && div != 24 && div != 32)) {
		dev_warn(component->dev, "%s : need PLL\n", __func__);
		return -EINVAL;
	}

	switch (div) {
	case 16:
		adc_aif2 |= RK630_ADC_WL_16;
		dac_aif2 |= RK630_DAC_WL_16;
		break;
	case 20:
		adc_aif2 |= RK630_ADC_WL_20;
		dac_aif2 |= RK630_DAC_WL_20;
		break;
	case 24:
		adc_aif2 |= RK630_ADC_WL_24;
		dac_aif2 |= RK630_DAC_WL_24;
		break;
	case 32:
		adc_aif2 |= RK630_ADC_WL_32;
		dac_aif2 |= RK630_DAC_WL_32;
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(component->dev, "%s : MCLK = %dHz, sample rate = %dHz, div = %d\n",
	    __func__, rk630->stereo_sysclk, rate, div);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		adc_aif1 |= RK630_ADC_VWL_16;
		dac_aif1 |= RK630_DAC_VWL_16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		adc_aif1 |= RK630_ADC_VWL_20;
		dac_aif1 |= RK630_DAC_VWL_20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		adc_aif1 |= RK630_ADC_VWL_24;
		dac_aif1 |= RK630_DAC_VWL_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		adc_aif1 |= RK630_ADC_VWL_32;
		dac_aif1 |= RK630_DAC_VWL_32;
		break;
	default:
		return -EINVAL;
	}

	switch (params_channels(params)) {
	case RK630_MONO:
		adc_aif1 |= RK630_ADC_TYPE_MONO;
		break;
	case RK630_STEREO:
		adc_aif1 |= RK630_ADC_TYPE_STEREO;
		break;
	default:
		return -EINVAL;
	}

	adc_aif1 |= RK630_ADC_SWAP_DIS;
	adc_aif2 |= RK630_ADC_RST_DIS;
	dac_aif1 |= RK630_DAC_SWAP_DIS;
	dac_aif2 |= RK630_DAC_RST_DIS;

	rk630->rate = rate;
	switch (rk630->rate) {
	case 96000:
		approximate_sample = RK630_PGA_ASR_96KHz;
		break;
	case 48000:
		approximate_sample = RK630_PGA_ASR_48KHz;
		break;
	case 44100:
		approximate_sample = RK630_PGA_ASR_441KHz;
		break;
	case 32000:
		approximate_sample = RK630_PGA_ASR_32KHz;
		break;
	case 24000:
		approximate_sample = RK630_PGA_ASR_24KHz;
		break;
	case 16000:
		approximate_sample = RK630_PGA_ASR_16KHz;
		break;
	case 12000:
		approximate_sample = RK630_PGA_ASR_12KHz;
		break;
	case 8000:
		approximate_sample = RK630_PGA_ASR_8KHz;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_component_update_bits(component, RK630_PGAL_ASR_CTL,
				      RK630_PGA_ASR_MASK, approximate_sample);
	snd_soc_component_update_bits(component, RK630_PGAR_ASR_CTL,
				      RK630_PGA_ASR_MASK, approximate_sample);

	snd_soc_component_update_bits(component, RK630_ADC_INT_CTL1,
				      RK630_ADC_VWL_MASK |
				      RK630_ADC_SWAP_MASK |
				      RK630_ADC_TYPE_MASK,
				      adc_aif1);
	snd_soc_component_update_bits(component, RK630_ADC_INT_CTL2,
				      RK630_ADC_WL_MASK |
				      RK630_ADC_RST_MASK,
				      adc_aif2);
	snd_soc_component_update_bits(component, RK630_DAC_INT_CTL1,
				      RK630_DAC_VWL_MASK |
				      RK630_DAC_SWAP_MASK,
				      dac_aif1);
	snd_soc_component_update_bits(component, RK630_DAC_INT_CTL2,
				      RK630_DAC_WL_MASK |
				      RK630_DAC_RST_MASK,
				      dac_aif2);

	return 0;
}

static int rk630_codec_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_component *component = dai->component;
	struct rk630_codec_priv *rk630 = snd_soc_component_get_drvdata(component);

	if (mute) {
		if (rk630 && rk630->spk_ctl_gpio)
			gpiod_set_value(rk630->spk_ctl_gpio, 0);
	} else {
		if (rk630 && rk630->spk_ctl_gpio)
			gpiod_set_value(rk630->spk_ctl_gpio, 1);
	}

	return 0;
}

#define RK630_PLAYBACK_RATES (SNDRV_PCM_RATE_8000 |\
			      SNDRV_PCM_RATE_16000 |    \
			      SNDRV_PCM_RATE_32000 |    \
			      SNDRV_PCM_RATE_44100 |    \
			      SNDRV_PCM_RATE_48000 |    \
			      SNDRV_PCM_RATE_96000 |    \
			      SNDRV_PCM_RATE_192000)

#define RK630_CAPTURE_RATES (SNDRV_PCM_RATE_8000 |\
			     SNDRV_PCM_RATE_16000 |    \
			     SNDRV_PCM_RATE_32000 |    \
			     SNDRV_PCM_RATE_44100 |    \
			     SNDRV_PCM_RATE_48000 |    \
			     SNDRV_PCM_RATE_96000)

#define RK630_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
		       SNDRV_PCM_FMTBIT_S20_3LE |\
		       SNDRV_PCM_FMTBIT_S24_LE |\
		       SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops rk630_dai_ops = {
	.hw_params  = rk630_codec_hw_params,
	.set_fmt    = rk630_codec_set_dai_fmt,
	.set_sysclk = rk630_codec_set_dai_sysclk,
	.digital_mute   = rk630_codec_digital_mute,
};

static struct snd_soc_dai_driver rk630_dai[] = {
	{
		.name = "rk630-hifi",
		.playback = {
			.stream_name = "HiFi Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = RK630_PLAYBACK_RATES,
			.formats = RK630_FORMATS,
			},
		.capture = {
			.stream_name = "HiFi Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = RK630_CAPTURE_RATES,
			.formats = RK630_FORMATS,
			},
		.ops = &rk630_dai_ops,
	},
};

static int rk630_codec_probe(struct snd_soc_component *component)
{
	struct rk630_codec_priv *rk630 = snd_soc_component_get_drvdata(component);
	unsigned int val;
	int ret;

	snd_soc_component_init_regmap(component, rk630->regmap);
	rk630->component = component;
	snd_soc_component_force_bias_level(component, SND_SOC_BIAS_OFF);
	regmap_write(rk630->cru, CRU_GATE_CON0, GRU_ACODECPHY_EN);

	snd_soc_component_read(component, RK630_RESET, &val);
	if (val != 0x03) {
		dev_err(component->dev, "%s : codec register 0: %x is not a 0x00000003\n",
		__func__, val);
		ret = -ENODEV;
		goto err__;
	}

	rk630_init(component);

	return 0;
err__:
	dev_err(component->dev, "%s err ret=%d\n", __func__, ret);

	return ret;
}

/* power down chip */
static void rk630_codec_remove(struct snd_soc_component *component)
{
	struct rk630_codec_priv *rk630 = snd_soc_component_get_drvdata(component);

	if (!rk630) {
		dev_err(component->dev, "%s : rk630 is NULL\n", __func__);
		return;
	}

	if (rk630->spk_ctl_gpio)
		gpiod_set_value(rk630->spk_ctl_gpio, 0);

	snd_soc_component_write(component, RK630_RESET, 0xfc);
	snd_soc_component_write(component, RK630_RESET, 0x3);
}

static const struct snd_soc_component_driver soc_codec_dev_rk630 = {
	.probe = rk630_codec_probe,
	.remove = rk630_codec_remove,
	.set_bias_level = rk630_set_bias_level,
	.controls = rk630_snd_controls,
	.num_controls = ARRAY_SIZE(rk630_snd_controls),
	.dapm_routes = rk630_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(rk630_dapm_routes),
	.dapm_widgets = rk630_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rk630_dapm_widgets),
};

const struct regmap_config rk630_codec_regmap_config = {
	.name = "rk630-codec",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = RK630_MAX_REG,
	.cache_type = REGCACHE_NONE,
	.writeable_reg = rk630_codec_register,
	.readable_reg = rk630_codec_register,
	.volatile_reg = rk630_volatile_register,
	.reg_defaults = rk630_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(rk630_reg_defaults),
	.reg_format_endian = REGMAP_ENDIAN_NATIVE,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
};

static int rk630_codec_platform_probe(struct platform_device *pdev)
{
	struct device_node *rk630_np = pdev->dev.of_node;
	struct rk630 *rk630 = dev_get_drvdata(pdev->dev.parent);
	struct rk630_codec_priv *rk630_codec;
	int ret;

	rk630_codec = devm_kzalloc(&pdev->dev, sizeof(*rk630_codec), GFP_KERNEL);
	if (!rk630_codec)
		return -ENOMEM;

	rk630_codec->dev = &pdev->dev;

	platform_set_drvdata(pdev, rk630_codec);

	rk630_codec->grf = rk630->grf;
	if (IS_ERR(rk630_codec->grf)) {
		dev_err(&pdev->dev, "needs rk630 grf\n");
		ret = PTR_ERR(rk630_codec->grf);
		goto err__;
	}

	rk630_codec->cru = rk630->cru;
	if (IS_ERR(rk630_codec->cru)) {
		dev_err(&pdev->dev, "needs rk630 cru\n");
		ret = PTR_ERR(rk630_codec->cru);
		goto err__;
	}

	rk630_codec->regmap = rk630->codec;
	if (IS_ERR(rk630_codec->regmap)) {
		dev_err(&pdev->dev, "needs rk630 codec\n");
		ret = PTR_ERR(rk630_codec->regmap);
		goto err__;
	}

	rk630_codec->spk_ctl_gpio = devm_gpiod_get_optional(&pdev->dev, "spk-ctl",
							    GPIOD_OUT_LOW);
	if (!IS_ERR_OR_NULL(rk630_codec->spk_ctl_gpio))
		dev_dbg(&pdev->dev, "spk-ctl-gpio %d\n", desc_to_gpio(rk630_codec->spk_ctl_gpio));

	rk630_codec->mclk = devm_clk_get(&pdev->dev, "mclk");
	if (IS_ERR(rk630_codec->mclk)) {
		dev_err(&pdev->dev, "Unable to get mclk\n");
		ret = -ENXIO;
		goto err__;
	}

	ret = clk_prepare_enable(rk630_codec->mclk);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s() clock prepare error %d\n",
			__func__, ret);
		goto err__;
	}

	rk630_codec->mic_in_differential =
			of_property_read_bool(rk630_np, "mic-in-differential");

	return devm_snd_soc_register_component(&pdev->dev, &soc_codec_dev_rk630,
					       rk630_dai, ARRAY_SIZE(rk630_dai));
err__:
	platform_set_drvdata(pdev, NULL);

	return ret;
}

static int rk630_codec_platform_remove(struct platform_device *pdev)
{
	struct rk630_codec_priv *rk630_codec = platform_get_drvdata(pdev);

	clk_disable_unprepare(rk630_codec->mclk);

	return 0;
}

static void rk630_codec_platform_shutdown(struct platform_device *pdev)
{
	struct rk630_codec_priv *rk630 = dev_get_drvdata(&pdev->dev);

	if (!rk630 || !rk630->component)
		return;

	if (rk630->spk_ctl_gpio)
		gpiod_set_value(rk630->spk_ctl_gpio, 0);

	snd_soc_component_write(rk630->component, RK630_RESET, 0xfc);
	snd_soc_component_write(rk630->component, RK630_RESET, 0x03);
}

static const struct of_device_id rk630_codec_of_match[] = {
	{ .compatible = "rockchip,rk630-codec" },
	{},
};

static struct platform_driver rk630_codec_driver = {
	.driver = {
		.name = "rk630-codec",
		.of_match_table = of_match_ptr(rk630_codec_of_match),
	},
	.probe = rk630_codec_platform_probe,
	.remove = rk630_codec_platform_remove,
	.shutdown = rk630_codec_platform_shutdown,
};
module_platform_driver(rk630_codec_driver);

MODULE_DESCRIPTION("ASoC RK630 CODEC driver");
MODULE_AUTHOR("Xinhuang Li <buluess.li@rock-chips.com>");
MODULE_LICENSE("GPL");
