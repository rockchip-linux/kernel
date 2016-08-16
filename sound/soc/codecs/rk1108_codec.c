/*
 * rk1108_codec.c  --  rk1108 ALSA Soc Audio driver
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/mfd/syscon.h>
#include <linux/rockchip/grf.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <asm/dma.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include "rk1108_codec.h"


/*
 * playback vol
 * 00: -39dB
 * 26: 0dB
 * 31: 6dB
 * step: 1.5dB
 */
#define OUT_VOLUME	26

/*
 * capture vol
 * 0: -18db
 * 12: 0db
 * 31: 28.5db
 * step: 1.5db
 */
#define CAP_VOLUME	21

struct rk1108_codec_priv {
	struct regmap *regmap;
	struct clk *pclk;
	atomic_t refcount;
	int spk_ctl_gpio;
};

static void rk1108_analog_output(struct rk1108_codec_priv *rk1108, int mute)
{
	gpio_direction_output(rk1108->spk_ctl_gpio, mute);
}

static int rk1108_reset(struct snd_soc_codec *codec)
{
	struct rk1108_codec_priv *rk1108 = snd_soc_codec_get_drvdata(codec);

	regmap_write(rk1108->regmap, RK1108_RESET, 0x00);
	mdelay(10);
	regmap_write(rk1108->regmap, RK1108_RESET, 0x03);

	return 0;
}

static int rk1108_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		break;

	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
		break;
	}
	codec->dapm.bias_level = level;

	return 0;
}

static int rk1108_set_dai_fmt(struct snd_soc_dai *codec_dai,
			      unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct rk1108_codec_priv *rk1108 = snd_soc_codec_get_drvdata(codec);
	unsigned int adc_aif1 = 0, adc_aif2 = 0, dac_aif1 = 0, dac_aif2 = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		adc_aif2 |= RK1108_I2S_MODE_SLV;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		adc_aif2 |= RK1108_I2S_MODE_MST;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		adc_aif1 |= RK1108_ADC_DF_PCM;
		dac_aif1 |= RK1108_DAC_DF_PCM;
		break;
	case SND_SOC_DAIFMT_I2S:
		adc_aif1 |= RK1108_ADC_DF_I2S;
		dac_aif1 |= RK1108_DAC_DF_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		adc_aif1 |= RK1108_ADC_DF_RJ;
		dac_aif1 |= RK1108_DAC_DF_RJ;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		adc_aif1 |= RK1108_ADC_DF_LJ;
		dac_aif1 |= RK1108_DAC_DF_LJ;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		adc_aif1 |= RK1108_ALRCK_POL_DIS;
		adc_aif2 |= RK1108_ABCLK_POL_DIS;
		dac_aif1 |= RK1108_DLRCK_POL_DIS;
		dac_aif2 |= RK1108_DBCLK_POL_DIS;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		adc_aif1 |= RK1108_ALRCK_POL_EN;
		adc_aif2 |= RK1108_ABCLK_POL_EN;
		dac_aif1 |= RK1108_DLRCK_POL_EN;
		dac_aif2 |= RK1108_DBCLK_POL_EN;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		adc_aif1 |= RK1108_ALRCK_POL_DIS;
		adc_aif2 |= RK1108_ABCLK_POL_EN;
		dac_aif1 |= RK1108_DLRCK_POL_DIS;
		dac_aif2 |= RK1108_DBCLK_POL_EN;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		adc_aif1 |= RK1108_ALRCK_POL_EN;
		adc_aif2 |= RK1108_ABCLK_POL_DIS;
		dac_aif1 |= RK1108_DLRCK_POL_EN;
		dac_aif2 |= RK1108_DBCLK_POL_DIS;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(rk1108->regmap, RK1108_ADC_INT_CTL1,
			   RK1108_ALRCK_POL_MASK | RK1108_ADC_DF_MASK,
			   adc_aif1);
	regmap_update_bits(rk1108->regmap, RK1108_ADC_INT_CTL2,
			   RK1108_ABCLK_POL_MASK | RK1108_I2S_MODE_MASK,
			   adc_aif2);
	regmap_update_bits(rk1108->regmap, RK1108_DAC_INT_CTL1,
			   RK1108_DLRCK_POL_MASK | RK1108_DAC_DF_MASK,
			   dac_aif1);
	regmap_update_bits(rk1108->regmap, RK1108_DAC_INT_CTL2,
			   RK1108_DBCLK_POL_MASK, dac_aif2);

	return 0;
}

static int rk1108_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rk1108_codec_priv *rk1108 = snd_soc_codec_get_drvdata(codec);
	unsigned int adc_aif1 = 0, adc_aif2  = 0, dac_aif1 = 0, dac_aif2  = 0;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		adc_aif1 |= RK1108_ADC_VWL_16;
		dac_aif1 |= RK1108_DAC_VWL_16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		adc_aif1 |= RK1108_ADC_VWL_20;
		dac_aif1 |= RK1108_DAC_VWL_20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		adc_aif1 |= RK1108_ADC_VWL_24;
		dac_aif1 |= RK1108_DAC_VWL_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		adc_aif1 |= RK1108_ADC_VWL_32;
		dac_aif1 |= RK1108_DAC_VWL_32;
		break;
	default:
		return -EINVAL;
	}

	switch (params_channels(params)) {
	case 1:
		adc_aif1 |= RK1108_ADC_TYPE_MONO;
		break;
	case 2:
		adc_aif1 |= RK1108_ADC_TYPE_STEREO;
		break;
	default:
		return -EINVAL;
	}

	adc_aif1 |= RK1108_ADC_SWAP_DIS;
	adc_aif2 |= RK1108_ADC_RST_DIS;
	dac_aif1 |= RK1108_DAC_SWAP_DIS;
	dac_aif2 |= RK1108_DAC_RST_DIS;

	regmap_update_bits(rk1108->regmap, RK1108_ADC_INT_CTL1,
			   RK1108_ADC_VWL_MASK | RK1108_ADC_SWAP_MASK |
			   RK1108_ADC_TYPE_MASK, adc_aif1);
	regmap_update_bits(rk1108->regmap, RK1108_ADC_INT_CTL2,
			   RK1108_ADC_RST_MASK, adc_aif2);
	regmap_update_bits(rk1108->regmap, RK1108_DAC_INT_CTL1,
			   RK1108_DAC_VWL_MASK | RK1108_DAC_SWAP_MASK,
			   dac_aif1);
	regmap_update_bits(rk1108->regmap, RK1108_DAC_INT_CTL2,
			   RK1108_DAC_RST_MASK, dac_aif2);

	return 0;
}

static int rk1108_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rk1108_codec_priv *rk1108 = snd_soc_codec_get_drvdata(codec);
	unsigned int val = 0;

	if (mute)
		val = RK1108_LOUTL_MUTE | RK1108_LOUTR_MUTE;
	else
		val = RK1108_LOUTL_UNMUTE | RK1108_LOUTR_UNMUTE;

	regmap_update_bits(rk1108->regmap, RK1108_LOUT_CTL,
			   RK1108_LOUTL_MUTE_MSK | RK1108_LOUTR_MUTE_MSK, val);

	return 0;
}

static struct rk1108_reg_msk_val playback_open_list[] = {
	{ RK1108_DAC_CTL, RK1108_REF_VOL_DACL_MASK | RK1108_REF_VOL_DACR_MASK,
	  RK1108_REF_VOL_DACL_EN | RK1108_REF_VOL_DACR_EN },
	{ RK1108_LOUT_CTL, RK1108_LOUTL_MSK | RK1108_LOUTR_MSK,
	  RK1108_LOUTL_EN | RK1108_LOUTR_EN },
	{ RK1108_LOUT_CTL, RK1108_LOUTL_MUTE_MSK | RK1108_LOUTR_MUTE_MSK,
	  RK1108_LOUTL_UNMUTE | RK1108_LOUTR_UNMUTE },
	{ RK1108_DAC_ENABLE,
	  RK1108_DACL_REF_VOL_MASK | RK1108_DACR_REF_VOL_MASK,
	  RK1108_DACL_REF_VOL_EN | RK1108_DACR_REF_VOL_EN },
	{ RK1108_DAC_ENABLE, RK1108_DACL_CLK_MASK | RK1108_DACR_CLK_MASK,
	  RK1108_DACL_CLK_EN | RK1108_DACR_CLK_EN },
	{ RK1108_DAC_ENABLE, RK1108_DACL_EN_MASK | RK1108_DACR_EN_MASK,
	  RK1108_DACL_EN | RK1108_DACR_EN },
	{ RK1108_DAC_ENABLE, RK1108_DACL_INIT_MASK | RK1108_DACR_INIT_MASK,
	  RK1108_DACL_WORK | RK1108_DACR_WORK },
	{ RK1108_LOUTL_GAIN, RK1108_LOUT_GAIN_MASK, OUT_VOLUME },
	{ RK1108_LOUTR_GAIN, RK1108_LOUT_GAIN_MASK, OUT_VOLUME },
};
#define RK1108_PLAYBACK_OPEN_LIST_LEN ARRAY_SIZE(playback_open_list)

static struct rk1108_reg_msk_val playback_close_list[] = {
	{ RK1108_LOUTL_GAIN, RK1108_LOUT_GAIN_MASK, 0x00 },
	{ RK1108_LOUTR_GAIN, RK1108_LOUT_GAIN_MASK, 0x00 },
	{ RK1108_LOUT_CTL, RK1108_LOUTL_MUTE_MSK | RK1108_LOUTR_MUTE_MSK,
	  RK1108_LOUTL_MUTE | RK1108_LOUTR_MUTE },
	{ RK1108_LOUT_CTL, RK1108_LOUTL_MSK | RK1108_LOUTR_MSK,
	  RK1108_LOUTL_DIS | RK1108_LOUTR_DIS },
	{ RK1108_DAC_ENABLE, RK1108_DACL_EN_MASK | RK1108_DACR_EN_MASK,
	  RK1108_DACL_DIS | RK1108_DACR_DIS },
	{ RK1108_DAC_ENABLE, RK1108_DACL_CLK_MASK | RK1108_DACR_CLK_MASK,
	  RK1108_DACL_CLK_DIS | RK1108_DACR_CLK_DIS },
	{ RK1108_DAC_ENABLE,
	  RK1108_DACL_REF_VOL_MASK | RK1108_DACR_REF_VOL_MASK,
	  RK1108_DACL_REF_VOL_DIS | RK1108_DACR_REF_VOL_DIS },
	{ RK1108_DAC_CTL, RK1108_REF_VOL_DACL_MASK | RK1108_REF_VOL_DACR_MASK,
	  RK1108_REF_VOL_DACL_DIS | RK1108_REF_VOL_DACR_DIS },
	{ RK1108_DAC_ENABLE, RK1108_DACL_INIT_MASK | RK1108_DACR_INIT_MASK,
	  RK1108_DACL_INIT | RK1108_DACR_INIT },
};
#define RK1108_PLAYBACK_CLOSE_LIST_LEN ARRAY_SIZE(playback_close_list)

static struct rk1108_reg_msk_val capture_open_list[] = {
	{ RK1108_ADC_CTL, RK1108_ADC_CURRENT_MASK, RK1108_ADC_CURRENT_EN },
	{ RK1108_BIAS_CTL, RK1108_MICBIAS_VOL_EN_MASK, RK1108_MICBIAS_VOL_EN },
	{ RK1108_BIAS_CTL, RK1108_MICBIAS_VOL_MSK, RK1108_MICBIAS_VOL_MAX },
	{ RK1108_ADC_CTL,
	  RK1108_ADCL_REF_VOL_EN_MASK | RK1108_ADCR_REF_VOL_EN_MASK,
	  RK1108_ADCL_REF_VOL_EN | RK1108_ADCR_REF_VOL_EN },
	{ RK1108_BST_CTL, RK1108_BSTL_PWRD_MASK | RK1108_BSTR_PWRD_MASK,
	  RK1108_BSTL_EN | RK1108_BSTR_EN },
	{ RK1108_ALC_MUNIN_CTL, RK1108_ALCL_PWR_MASK | RK1108_ALCR_PWR_MASK,
	  RK1108_ALCL_EN | RK1108_ALCR_EN },
	{ RK1108_ADC_ENABLE, RK1108_ADCL_CLK_EN_MASK | RK1108_ADCR_CLK_EN_MASK,
	  RK1108_ADCL_CLK_EN | RK1108_ADCR_CLK_EN },
	{ RK1108_ADC_ENABLE, RK1108_ADCL_AMP_EN_MASK | RK1108_ADCR_AMP_EN_MASK,
	  RK1108_ADCL_AMP_EN | RK1108_ADCR_AMP_EN },
	{ RK1108_ALC_MUNIN_CTL, RK1108_ALCL_MUTE_MASK | RK1108_ALCR_MUTE_MASK,
	  RK1108_ALCL_UNMUTE | RK1108_ALCR_UNMUTE },
	{ RK1108_BST_CTL, RK1108_BSTL_MUTE_MASK | RK1108_BSTR_MUTE_MASK,
	  RK1108_BSTL_UNMUTE | RK1108_BSTR_UNMUTE },
	{ RK1108_BST_CTL, RK1108_BSTL_GAIN_MASK | RK1108_BSTR_GAIN_MASK,
	  RK1108_BSTL_GAIN_20 | RK1108_BSTL_GAIN_20 },
	{ RK1108_ALCL_GAIN_CTL, RK1108_ALCL_GAIN_MSK, CAP_VOLUME },
	{ RK1108_ALCR_GAIN_CTL, RK1108_ALCR_GAIN_MSK, CAP_VOLUME },
	{ RK1108_ADC_CTL,
	  RK1108_ADCL_ZERO_DET_EN_MASK | RK1108_ADCR_ZERO_DET_EN_MASK,
	  RK1108_ADCL_ZERO_DET_EN | RK1108_ADCR_ZERO_DET_EN },

};
#define RK1108_CAPTURE_OPEN_LIST_LEN ARRAY_SIZE(capture_open_list)

static struct rk1108_reg_msk_val capture_close_list[] = {
	{ RK1108_ALCL_GAIN_CTL, RK1108_ALCL_GAIN_MSK, 0x00 },
	{ RK1108_ALCR_GAIN_CTL, RK1108_ALCR_GAIN_MSK, 0x00 },
	{ RK1108_BST_CTL, RK1108_BSTL_GAIN_MASK | RK1108_BSTR_GAIN_MASK,
	  RK1108_BSTL_GAIN_0 | RK1108_BSTL_GAIN_0 },
	{ RK1108_ADC_ENABLE, RK1108_ADCL_AMP_EN_MASK | RK1108_ADCR_AMP_EN_MASK,
	  RK1108_ADCL_AMP_DIS | RK1108_ADCR_AMP_DIS },
	{ RK1108_ADC_ENABLE, RK1108_ADCL_CLK_EN_MASK | RK1108_ADCR_CLK_EN_MASK,
	  RK1108_ADCL_CLK_DIS | RK1108_ADCR_CLK_DIS },
	{ RK1108_ALC_MUNIN_CTL, RK1108_ALCL_PWR_MASK | RK1108_ALCR_PWR_MASK,
	  RK1108_ALCL_DIS | RK1108_ALCR_DIS },
	{ RK1108_ALC_MUNIN_CTL, RK1108_ALCL_MUTE_MASK | RK1108_ALCR_MUTE_MASK,
	  RK1108_ALCL_MUTE | RK1108_ALCR_MUTE },
	{ RK1108_BST_CTL, RK1108_BSTL_PWRD_MASK | RK1108_BSTR_PWRD_MASK,
	  RK1108_BSTL_DIS | RK1108_BSTR_DIS },
	{ RK1108_BST_CTL, RK1108_BSTL_MUTE_MASK | RK1108_BSTR_MUTE_MASK,
	  RK1108_BSTL_MUTE | RK1108_BSTR_MUTE},
	{ RK1108_ADC_CTL,
	  RK1108_ADCL_REF_VOL_EN_MASK | RK1108_ADCR_REF_VOL_EN_MASK,
	  RK1108_ADCL_REF_VOL_DIS | RK1108_ADCR_REF_VOL_DIS },
	{ RK1108_ADC_CTL, RK1108_ADC_CURRENT_MASK, RK1108_ADC_CURRENT_DIS},
	{ RK1108_ADC_CTL,
	  RK1108_ADCL_ZERO_DET_EN_MASK | RK1108_ADCR_ZERO_DET_EN_MASK,
	  RK1108_ADCL_ZERO_DET_DIS | RK1108_ADCR_ZERO_DET_DIS },

};
#define RK1108_CAPTURE_CLOSE_LIST_LEN ARRAY_SIZE(capture_close_list)

static int rk1108_codec_power_on(struct snd_soc_codec *codec)
{
	struct rk1108_codec_priv *rk1108 = snd_soc_codec_get_drvdata(codec);

	regmap_update_bits(rk1108->regmap, RK1108_SELECT_CURRENT,
			   RK1108_XCHARGE_MASK, RK1108_PRECHARGE_HPOUT);
	mdelay(1);
	regmap_update_bits(rk1108->regmap, RK1108_SELECT_CURRENT,
			   RK1108_CHARGE_CURRENT_ALL_MASK,
			   RK1108_CHARGE_CURRENT_ALL_ON);

	return 0;
}

static int rk1108_codec_power_off(struct snd_soc_codec *codec)
{
	struct rk1108_codec_priv *rk1108 = snd_soc_codec_get_drvdata(codec);

	regmap_update_bits(rk1108->regmap, RK1108_SELECT_CURRENT,
			   RK1108_XCHARGE_MASK, RK1108_DISCHARGE_HPOUT);
	regmap_update_bits(rk1108->regmap, RK1108_SELECT_CURRENT,
			   RK1108_CHARGE_CURRENT_ALL_MASK,
			   RK1108_CHARGE_CURRENT_ALL_ON);

	return 0;
}

static int rk1108_codec_open_capture(struct snd_soc_codec *codec)
{
	struct rk1108_codec_priv *rk1108 = snd_soc_codec_get_drvdata(codec);
	int i = 0;

	for (i = 0; i < RK1108_CAPTURE_OPEN_LIST_LEN; i++) {
		regmap_update_bits(rk1108->regmap, capture_open_list[i].reg,
				   capture_open_list[i].msk,
				   capture_open_list[i].val);
		mdelay(1);
	}

	return 0;
}

static int rk1108_codec_close_capture(struct snd_soc_codec *codec)
{
	struct rk1108_codec_priv *rk1108 = snd_soc_codec_get_drvdata(codec);
	int i = 0;

	for (i = 0; i < RK1108_CAPTURE_CLOSE_LIST_LEN; i++) {
		regmap_update_bits(rk1108->regmap, capture_close_list[i].reg,
				   capture_close_list[i].msk,
				   capture_close_list[i].val);
		mdelay(1);
	}

	return 0;
}

static int rk1108_codec_open_playback(struct snd_soc_codec *codec)
{
	struct rk1108_codec_priv *rk1108 = snd_soc_codec_get_drvdata(codec);
	int i = 0;

	for (i = 0; i < RK1108_PLAYBACK_OPEN_LIST_LEN; i++) {
		regmap_update_bits(rk1108->regmap, playback_open_list[i].reg,
				   playback_open_list[i].msk,
				   playback_open_list[i].val);
		mdelay(1);
	}

	rk1108_analog_output(rk1108, 1);

	return 0;
}

static int rk1108_codec_close_playback(struct snd_soc_codec *codec)
{
	struct rk1108_codec_priv *rk1108 = snd_soc_codec_get_drvdata(codec);
	int i = 0;

	rk1108_analog_output(rk1108, 0);

	for (i = 0; i < RK1108_PLAYBACK_CLOSE_LIST_LEN; i++) {
		regmap_update_bits(rk1108->regmap, playback_close_list[i].reg,
				   playback_close_list[i].msk,
				   playback_close_list[i].val);
		mdelay(1);
	}

	return 0;
}

static int rk1108_pcm_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rk1108_codec_priv *rk1108 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	if (!atomic_read(&rk1108->refcount)) {
		regmap_update_bits(rk1108->regmap, RK1108_DAC_CTL,
				   RK1108_CURRENT_MASK, RK1108_CURRENT_EN);
		mdelay(1);
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = rk1108_codec_open_playback(codec);
	else
		ret = rk1108_codec_open_capture(codec);

	atomic_inc(&rk1108->refcount);

	return ret;
}

static void rk1108_pcm_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rk1108_codec_priv *rk1108 = snd_soc_codec_get_drvdata(codec);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		rk1108_codec_close_playback(codec);
	else
		rk1108_codec_close_capture(codec);

	atomic_dec(&rk1108->refcount);
	if (!atomic_read(&rk1108->refcount))
		regmap_update_bits(rk1108->regmap, RK1108_DAC_CTL,
				   RK1108_CURRENT_MASK, RK1108_CURRENT_DIS);
}

static struct snd_soc_dai_ops rk1108_dai_ops = {
	.hw_params = rk1108_hw_params,
	.set_fmt = rk1108_set_dai_fmt,
	.digital_mute = rk1108_digital_mute,
	.startup = rk1108_pcm_startup,
	.shutdown = rk1108_pcm_shutdown,
};

static struct snd_soc_dai_driver rk1108_dai[] = {
	{
		.name = "rk1108-hifi",
		.id = RK1108_HIFI,
		.playback = {
			.stream_name = "HiFi Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S20_3LE |
				    SNDRV_PCM_FMTBIT_S24_LE |
				    SNDRV_PCM_FMTBIT_S32_LE),
		},
		.capture = {
			.stream_name = "HiFi Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S20_3LE |
				    SNDRV_PCM_FMTBIT_S24_LE |
				    SNDRV_PCM_FMTBIT_S32_LE),
		},
		.ops = &rk1108_dai_ops,
	},
};

static int rk1108_suspend(struct snd_soc_codec *codec)
{
	rk1108_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int rk1108_resume(struct snd_soc_codec *codec)
{
	rk1108_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

static int rk1108_probe(struct snd_soc_codec *codec)
{
	rk1108_reset(codec);
	rk1108_codec_power_on(codec);

	return 0;
}

static int rk1108_remove(struct snd_soc_codec *codec)
{
	struct rk1108_codec_priv *rk1108 = snd_soc_codec_get_drvdata(codec);

	rk1108_analog_output(rk1108, 0);
	rk1108_codec_power_off(codec);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_rk1108 = {
	.probe = rk1108_probe,
	.remove = rk1108_remove,
	.suspend = rk1108_suspend,
	.resume = rk1108_resume,
	.set_bias_level = rk1108_set_bias_level,
};

static const struct reg_default rk1108_codec_reg_defaults[] = {
	{ RK1108_RESET, 0x03 },
	{ RK1108_ADC_INT_CTL1, 0x50 },
	{ RK1108_ADC_INT_CTL2, 0x0e },
	{ RK1108_DAC_INT_CTL1, 0x50 },
	{ RK1108_DAC_INT_CTL2, 0x0e },
	{ RK1108_SELECT_CURRENT, 0x01 },
	{ RK1108_ALCL_GAIN_CTL, 0x0c },
	{ RK1108_ALCR_GAIN_CTL, 0x0c },
	{ RK1108_PGA_AGC_CTL2, 0x46 },
	{ RK1108_PGA_AGC_CTL3, 0x41 },
	{ RK1108_PGA_AGC_CTL4, 0x2c },
	{ RK1108_PGA_AGC_MAX_H, 0x26 },
	{ RK1108_PGA_AGC_MAX_L, 0x40 },
	{ RK1108_PGA_AGC_MIN_H, 0x36 },
	{ RK1108_PGA_AGC_MIN_L, 0x20 },
	{ RK1108_PGA_AGC_CTL5, 0x38 },
};

static bool rk1108_codec_write_read_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case RK1108_RESET:
	case RK1108_ADC_INT_CTL1:
	case RK1108_ADC_INT_CTL2:
	case RK1108_DAC_INT_CTL1:
	case RK1108_DAC_INT_CTL2:
	case RK1108_BIST_CTL:
	case RK1108_SELECT_CURRENT:
	case RK1108_BIAS_CTL:
	case RK1108_ADC_CTL:
	case RK1108_BST_CTL:
	case RK1108_ALC_MUNIN_CTL:
	case RK1108_ALCL_GAIN_CTL:
	case RK1108_ALCR_GAIN_CTL:
	case RK1108_ADC_ENABLE:
	case RK1108_DAC_CTL:
	case RK1108_DAC_ENABLE:
	case RK1108_LOUT_CTL:
	case RK1108_LOUTL_GAIN:
	case RK1108_LOUTR_GAIN:
	case RK1108_PGA_AGC_CTL1:
	case RK1108_PGA_AGC_CTL2:
	case RK1108_PGA_AGC_CTL3:
	case RK1108_PGA_AGC_CTL4:
	case RK1108_PGA_ASR_CTL:
	case RK1108_PGA_AGC_MAX_H:
	case RK1108_PGA_AGC_MAX_L:
	case RK1108_PGA_AGC_MIN_H:
	case RK1108_PGA_AGC_MIN_L:
	case RK1108_PGA_AGC_CTL5:
		return true;
	default:
		return false;
	}
}

static bool rk1108_codec_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case RK1108_RESET:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config rk1108_codec_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = RK1108_PGA_AGC_CTL5,
	.writeable_reg = rk1108_codec_write_read_reg,
	.readable_reg = rk1108_codec_write_read_reg,
	.volatile_reg = rk1108_codec_volatile_reg,
	.reg_defaults = rk1108_codec_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(rk1108_codec_reg_defaults),
	.cache_type = REGCACHE_FLAT,
};

static int rk1108_platform_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct rk1108_codec_priv *rk1108;
	struct resource *res;
	void __iomem *base;
	int ret = 0;
	struct regmap *grf;

	grf = syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(grf)) {
		dev_err(&pdev->dev,
			"missing 'rockchip,grf' property\n");
		return PTR_ERR(grf);
	}

	/* enable i2s->acodec path */
	ret = regmap_write(grf, RK1108_GRF_SOC_CON0, BIT(12) << 16 | BIT(12));
	if (ret != 0) {
		dev_err(&pdev->dev, "Could not write to GRF: %d\n", ret);
		return ret;
	}

	rk1108 = devm_kzalloc(&pdev->dev, sizeof(*rk1108), GFP_KERNEL);
	if (!rk1108)
		return -ENOMEM;

	rk1108->spk_ctl_gpio = of_get_named_gpio(np, "spk_ctl_io", 0);
	if (!gpio_is_valid(rk1108->spk_ctl_gpio)) {
		dev_err(&pdev->dev, "invalid spk ctl gpio\n");
		return -EINVAL;
	}
	ret = devm_gpio_request(&pdev->dev, rk1108->spk_ctl_gpio, "spk_ctl");
	if (ret < 0) {
		dev_err(&pdev->dev, "spk_ctl_gpio request fail\n");
		return ret;
	}

	rk1108->pclk = devm_clk_get(&pdev->dev, "g_pclk_acodec");
	if (IS_ERR(rk1108->pclk)) {
		dev_err(&pdev->dev, "can't get acodec pclk\n");
		return PTR_ERR(rk1108->pclk);
	}

	ret = clk_prepare_enable(rk1108->pclk);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable acodec pclk\n");
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	rk1108->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					       &rk1108_codec_regmap_config);
	if (IS_ERR(rk1108->regmap))
		return PTR_ERR(rk1108->regmap);

	atomic_set(&rk1108->refcount, 0);
	platform_set_drvdata(pdev, rk1108);

	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_rk1108,
				      rk1108_dai, ARRAY_SIZE(rk1108_dai));
}

static int rk1108_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static const struct of_device_id rk1108codec_of_match[] = {
	{ .compatible = "rockchip,rk1108-codec", },
	{},
};
MODULE_DEVICE_TABLE(of, rk1108codec_of_match);

static struct platform_driver rk1108_codec_driver = {
	.driver = {
		   .name = "rk1108-codec",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(rk1108codec_of_match),
	},
	.probe = rk1108_platform_probe,
	.remove = rk1108_platform_remove,
};
module_platform_driver(rk1108_codec_driver);

MODULE_AUTHOR("Sugar Zhang <sugar.zhang@rock-chips.com>");
MODULE_DESCRIPTION("ASoC rk1108 codec driver");
MODULE_LICENSE("GPL v2");
