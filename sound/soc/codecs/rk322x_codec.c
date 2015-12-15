/*
 * rk322x_codec.c  --  rk322x ALSA Soc Audio driver
 *
 * Copyright (c) 2015, Fuzhou Rockchip Electronics Co., Ltd All rights reserved.
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
#include "rk322x_codec.h"

/* volume setting
 *  0: -39dB
 *  26: 0dB
 *  31: 6dB
 *  Step: 1.5dB
*/
#define  OUT_VOLUME    (0x18)

struct rk322x_codec_priv {
	struct regmap *regmap;
	struct clk *pclk;
	unsigned int sclk;
	int spk_ctl_gpio;
	int spk_depop_time; /* msec */
};

static const struct reg_default rk322x_codec_reg_defaults[] = {
	{ CODEC_RESET, 0x03 },
	{ DAC_INIT_CTRL1, 0x00 },
	{ DAC_INIT_CTRL2, 0x50 },
	{ DAC_INIT_CTRL3, 0x0e },
	{ DAC_PRECHARGE_CTRL, 0x01 },
	{ DAC_PWR_CTRL, 0x00 },
	{ DAC_CLK_CTRL, 0x00 },
	{ HPMIX_CTRL, 0x00 },
	{ HPOUT_CTRL, 0x00 },
	{ HPOUTL_GAIN_CTRL, 0x00 },
	{ HPOUTR_GAIN_CTRL, 0x00 },
	{ HPOUT_POP_CTRL, 0x11 },
};

static int rk322x_codec_reset(struct snd_soc_codec *codec)
{
	struct rk322x_codec_priv *rk322x = snd_soc_codec_get_drvdata(codec);

	regmap_write(rk322x->regmap, CODEC_RESET, 0);
	mdelay(10);
	regmap_write(rk322x->regmap, CODEC_RESET, 0x03);

	return 0;
}

static int rk322x_set_dai_fmt(struct snd_soc_dai *codec_dai,
			      unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct rk322x_codec_priv *rk322x = snd_soc_codec_get_drvdata(codec);
	unsigned int val = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		val |= PIN_DIRECTION_IN | DAC_I2S_MODE_SLAVE;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		val |= PIN_DIRECTION_OUT | DAC_I2S_MODE_MASTER;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(rk322x->regmap, DAC_INIT_CTRL1,
			   PIN_DIRECTION_MASK | DAC_I2S_MODE_MASK, val);

	val = 0;
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		val |= DAC_MODE_PCM;
		break;
	case SND_SOC_DAIFMT_I2S:
		val |= DAC_MODE_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		val |= DAC_MODE_RJM;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val |= DAC_MODE_LJM;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(rk322x->regmap, DAC_INIT_CTRL2,
			   DAC_MODE_MASK, val);
	return 0;
}

static int rk322x_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct rk322x_codec_priv *rk322x = snd_soc_codec_get_drvdata(codec);

	rk322x->sclk = freq;
	return 0;
}

static void rk322x_analog_output(struct rk322x_codec_priv *rk322x, int mute)
{
	gpio_direction_output(rk322x->spk_ctl_gpio, mute);
}

static int rk322x_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rk322x_codec_priv *rk322x = snd_soc_codec_get_drvdata(codec);
	unsigned int val = 0;

	if (mute)
		val = HPOUTL_MUTE | HPOUTR_MUTE;
	else
		val = HPOUTL_UNMUTE | HPOUTR_UNMUTE;

	regmap_update_bits(rk322x->regmap, HPOUT_CTRL,
			   HPOUTL_MUTE_MASK | HPOUTR_MUTE_MASK, val);
	return 0;
}

static int rk322x_codec_power_on(struct snd_soc_codec *codec, int wait_ms)
{
	struct rk322x_codec_priv *rk322x = snd_soc_codec_get_drvdata(codec);

	regmap_update_bits(rk322x->regmap, DAC_PRECHARGE_CTRL,
			   DAC_CHARGE_XCHARGE_MASK, DAC_CHARGE_PRECHARGE);
	mdelay(10);
	regmap_update_bits(rk322x->regmap, DAC_PRECHARGE_CTRL,
			   DAC_CHARGE_CURRENT_ALL_MASK,
			   DAC_CHARGE_CURRENT_ALL_ON);

	/* TODO: select a small current to decrease power consumption */
	mdelay(wait_ms);

	return 0;
}

static int rk322x_codec_power_off(struct snd_soc_codec *codec, int wait_ms)
{
	struct rk322x_codec_priv *rk322x = snd_soc_codec_get_drvdata(codec);

	regmap_update_bits(rk322x->regmap, DAC_PRECHARGE_CTRL,
			   DAC_CHARGE_XCHARGE_MASK, DAC_CHARGE_DISCHARGE);
	mdelay(10);
	regmap_update_bits(rk322x->regmap, DAC_PRECHARGE_CTRL,
			   DAC_CHARGE_CURRENT_ALL_MASK,
			   DAC_CHARGE_CURRENT_ALL_ON);

	mdelay(wait_ms);

	return 0;
}

static struct rk322x_reg_msk_val playback_open_list[] = {
	{ DAC_PWR_CTRL, DAC_PWR_MASK, DAC_PWR_ON },
	{ DAC_PWR_CTRL, DACL_PATH_REFV_MASK | DACR_PATH_REFV_MASK,
	  DACL_PATH_REFV_ON | DACR_PATH_REFV_ON },
	{ DAC_PWR_CTRL, HPOUTL_ZERO_CROSSING_ON | HPOUTR_ZERO_CROSSING_ON,
	  HPOUTL_ZERO_CROSSING_ON | HPOUTR_ZERO_CROSSING_ON },
	{ HPOUT_POP_CTRL, HPOUTR_POP_MASK | HPOUTL_POP_MASK,
	  HPOUTR_POP_WORK | HPOUTL_POP_WORK },
	{ HPMIX_CTRL, HPMIXL_MASK | HPMIXR_MASK, HPMIXL_EN | HPMIXR_EN },
	{ HPMIX_CTRL, HPMIXL_INIT_MASK | HPMIXR_INIT_MASK,
	  HPMIXL_INIT_EN | HPMIXR_INIT_EN },
	{ HPOUT_CTRL, HPOUTL_MASK | HPOUTR_MASK, HPOUTL_EN | HPOUTR_EN },
	{ HPOUT_CTRL, HPOUTL_INIT_MASK | HPOUTR_INIT_MASK,
	  HPOUTL_INIT_EN | HPOUTR_INIT_EN },
	{ DAC_CLK_CTRL, DACL_REFV_MASK | DACR_REFV_MASK,
	  DACL_REFV_ON | DACR_REFV_ON },
	{ DAC_CLK_CTRL, DACL_CLK_MASK | DACR_CLK_MASK,
	  DACL_CLK_ON | DACR_CLK_ON },
	{ DAC_CLK_CTRL, DACL_MASK | DACR_MASK, DACL_ON | DACR_ON },
	{ DAC_CLK_CTRL, DACL_INIT_MASK | DACR_INIT_MASK,
	  DACL_INIT_ON | DACR_INIT_ON },
	{ DAC_SELECT, DACL_SELECT_MASK | DACR_SELECT_MASK,
	  DACL_SELECT | DACR_SELECT },
	{ HPMIX_CTRL, HPMIXL_INIT2_MASK | HPMIXR_INIT2_MASK,
	  HPMIXL_INIT2_EN | HPMIXR_INIT2_EN },
	{ HPOUT_CTRL, HPOUTL_MUTE_MASK | HPOUTR_MUTE_MASK,
	  HPOUTL_UNMUTE | HPOUTR_UNMUTE },
};

#define PLAYBACK_OPEN_LIST_LEN ARRAY_SIZE(playback_open_list)

static int rk322x_codec_open_playback(struct snd_soc_codec *codec)
{
	struct rk322x_codec_priv *rk322x = snd_soc_codec_get_drvdata(codec);
	int i = 0;

	regmap_update_bits(rk322x->regmap, DAC_PRECHARGE_CTRL,
			   DAC_CHARGE_CURRENT_ALL_MASK,
			   DAC_CHARGE_CURRENT_ALL_ON);

	for (i = 0; i < PLAYBACK_OPEN_LIST_LEN; i++) {
		regmap_update_bits(rk322x->regmap,
				   playback_open_list[i].reg,
				   playback_open_list[i].msk,
				   playback_open_list[i].val);
		mdelay(1);
	}

	msleep(rk322x->spk_depop_time);
	rk322x_analog_output(rk322x, 1);

	regmap_update_bits(rk322x->regmap, HPOUTL_GAIN_CTRL,
			   HPOUTL_GAIN_MASK, OUT_VOLUME);
	regmap_update_bits(rk322x->regmap, HPOUTR_GAIN_CTRL,
			   HPOUTR_GAIN_MASK, OUT_VOLUME);
	return 0;
}

static struct rk322x_reg_msk_val playback_close_list[] = {
	{ HPMIX_CTRL, HPMIXL_INIT2_MASK | HPMIXR_INIT2_MASK,
	  HPMIXL_INIT2_DIS | HPMIXR_INIT2_DIS },
	{ DAC_SELECT, DACL_SELECT_MASK | DACR_SELECT_MASK,
	  DACL_UNSELECT | DACR_UNSELECT },
	{ HPOUT_CTRL, HPOUTL_MUTE_MASK | HPOUTR_MUTE_MASK,
	  HPOUTL_MUTE | HPOUTR_MUTE },
	{ HPOUT_CTRL, HPOUTL_INIT_MASK | HPOUTR_INIT_MASK,
	  HPOUTL_INIT_DIS | HPOUTR_INIT_DIS },
	{ HPOUT_CTRL, HPOUTL_MASK | HPOUTR_MASK, HPOUTL_DIS | HPOUTR_DIS },
	{ HPMIX_CTRL, HPMIXL_MASK | HPMIXR_MASK, HPMIXL_DIS | HPMIXR_DIS },
	{ DAC_CLK_CTRL, DACL_MASK | DACR_MASK, DACL_OFF | DACR_OFF },
	{ DAC_CLK_CTRL, DACL_CLK_MASK | DACR_CLK_MASK,
	  DACL_CLK_OFF | DACR_CLK_OFF },
	{ DAC_CLK_CTRL, DACL_REFV_MASK | DACR_REFV_MASK,
	  DACL_REFV_OFF | DACR_REFV_OFF },
	{ HPOUT_POP_CTRL, HPOUTR_POP_MASK | HPOUTL_POP_MASK, 0 },
	{ DAC_PWR_CTRL, DACL_PATH_REFV_MASK | DACR_PATH_REFV_MASK,
	  DACL_PATH_REFV_OFF | DACR_PATH_REFV_OFF },
	{ DAC_PWR_CTRL, DAC_PWR_MASK, DAC_PWR_OFF },
	{ HPMIX_CTRL, HPMIXL_INIT_MASK | HPMIXR_INIT_MASK,
	  HPMIXL_INIT_DIS | HPMIXR_INIT_DIS },
	{ DAC_CLK_CTRL, DACL_INIT_MASK | DACR_INIT_MASK,
	  DACL_INIT_OFF | DACR_INIT_OFF },
};

#define PLAYBACK_CLOSE_LIST_LEN ARRAY_SIZE(playback_close_list)

static int rk322x_codec_close_playback(struct snd_soc_codec *codec)
{
	struct rk322x_codec_priv *rk322x = snd_soc_codec_get_drvdata(codec);
	int i = 0;

	rk322x_analog_output(rk322x, 0);

	regmap_update_bits(rk322x->regmap, HPOUTL_GAIN_CTRL,
			   HPOUTL_GAIN_MASK, 0);
	regmap_update_bits(rk322x->regmap, HPOUTR_GAIN_CTRL,
			   HPOUTR_GAIN_MASK, 0);

	for (i = 0; i < PLAYBACK_CLOSE_LIST_LEN; i++) {
		regmap_update_bits(rk322x->regmap,
				   playback_close_list[i].reg,
				   playback_close_list[i].msk,
				   playback_close_list[i].val);
		mdelay(1);
	}

	regmap_update_bits(rk322x->regmap, DAC_PRECHARGE_CTRL,
			   DAC_CHARGE_CURRENT_ALL_MASK,
			   DAC_CHARGE_CURRENT_I);
	return 0;
}

static int rk322x_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rk322x_codec_priv *rk322x = snd_soc_codec_get_drvdata(codec);
	unsigned int val = 0;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		val |= DAC_VDL_16BITS;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		val |= DAC_VDL_20BITS;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		val |= DAC_VDL_24BITS;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		val |= DAC_VDL_32BITS;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(rk322x->regmap, DAC_INIT_CTRL2, DAC_VDL_MASK, val);
	val = DAC_WL_32BITS | DAC_RST_DIS;
	regmap_update_bits(rk322x->regmap, DAC_INIT_CTRL3,
			   DAC_WL_MASK | DAC_RST_MASK, val);

	return 0;
}

static int rk322x_pcm_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	return rk322x_codec_open_playback(codec);
}

static void rk322x_pcm_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	rk322x_codec_close_playback(codec);
}

static struct snd_soc_dai_ops rk322x_dai_ops = {
	.hw_params = rk322x_hw_params,
	.set_fmt = rk322x_set_dai_fmt,
	.set_sysclk = rk322x_set_dai_sysclk,
	.digital_mute = rk322x_digital_mute,
	.startup = rk322x_pcm_startup,
	.shutdown = rk322x_pcm_shutdown,
};

static struct snd_soc_dai_driver rk322x_dai[] = {
	{
		.name = "rk322x-hifi",
		.id = RK322x_HIFI,
		.playback = {
			.stream_name = "HIFI Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S20_3LE |
				    SNDRV_PCM_FMTBIT_S24_LE |
				    SNDRV_PCM_FMTBIT_S32_LE),
		},
		.ops = &rk322x_dai_ops,
	},
};

static int rk322x_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		break;

	case SND_SOC_BIAS_STANDBY:
		break;

	case SND_SOC_BIAS_OFF:
		break;
	}
	codec->dapm.bias_level = level;

	return 0;
}

static int rk322x_codec_probe(struct snd_soc_codec *codec)
{
	rk322x_codec_reset(codec);
	rk322x_codec_power_on(codec, 0);

	return 0;
}

static int rk322x_codec_remove(struct snd_soc_codec *codec)
{
	rk322x_codec_close_playback(codec);
	rk322x_codec_power_off(codec, 0);

	return 0;
}

static int rk322x_suspend(struct snd_soc_codec *codec)
{
	rk322x_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int rk322x_resume(struct snd_soc_codec *codec)
{
	rk322x_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_rk322x = {
	.probe = rk322x_codec_probe,
	.remove = rk322x_codec_remove,
	.suspend = rk322x_suspend,
	.resume = rk322x_resume,
	.set_bias_level = rk322x_set_bias_level,
};

static bool rk322x_codec_write_read_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CODEC_RESET:
	case DAC_INIT_CTRL1:
	case DAC_INIT_CTRL2:
	case DAC_INIT_CTRL3:
	case DAC_PRECHARGE_CTRL:
	case DAC_PWR_CTRL:
	case DAC_CLK_CTRL:
	case HPMIX_CTRL:
	case DAC_SELECT:
	case HPOUT_CTRL:
	case HPOUTL_GAIN_CTRL:
	case HPOUTR_GAIN_CTRL:
	case HPOUT_POP_CTRL:
		return true;
	default:
		return false;
	}
}

static bool rk322x_codec_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CODEC_RESET:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config rk322x_codec_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = HPOUT_POP_CTRL,
	.writeable_reg = rk322x_codec_write_read_reg,
	.readable_reg = rk322x_codec_write_read_reg,
	.volatile_reg = rk322x_codec_volatile_reg,
	.reg_defaults = rk322x_codec_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(rk322x_codec_reg_defaults),
	.cache_type = REGCACHE_FLAT,
};

static int rk322x_platform_probe(struct platform_device *pdev)
{
	struct device_node *rk322x_np = pdev->dev.of_node;
	struct rk322x_codec_priv *rk322x;
	struct resource *res;
	void __iomem *base;
	int ret = 0;

	rk322x = devm_kzalloc(&pdev->dev, sizeof(*rk322x), GFP_KERNEL);
	if (!rk322x)
		return -ENOMEM;

	ret = of_property_read_u32(rk322x_np, "spk_depop_time",
				   &rk322x->spk_depop_time);
	if (ret < 0) {
		dev_info(&pdev->dev, "spk_depop_time undefined, use default value.\n");
		rk322x->spk_depop_time = 200;
	}

	rk322x->spk_ctl_gpio = of_get_named_gpio(rk322x_np, "spk_ctl_io", 0);
	if (!gpio_is_valid(rk322x->spk_ctl_gpio)) {
		dev_err(&pdev->dev, "invalid spk ctl gpio\n");
		return -EINVAL;
	}

	ret = devm_gpio_request(&pdev->dev, rk322x->spk_ctl_gpio, "spk_ctl");
	if (ret < 0) {
		dev_err(&pdev->dev, "spk_ctl_gpio request fail\n");
		return ret;
	}

	rk322x_analog_output(rk322x, 0);

	rk322x->pclk = devm_clk_get(&pdev->dev, "g_pclk_acodec");
	if (IS_ERR(rk322x->pclk)) {
		dev_err(&pdev->dev, "can't get acodec pclk\n");
		return PTR_ERR(rk322x->pclk);
	}

	ret = clk_prepare_enable(rk322x->pclk);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable acodec pclk\n");
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	rk322x->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					    &rk322x_codec_regmap_config);
	if (IS_ERR(rk322x->regmap))
		return PTR_ERR(rk322x->regmap);

	platform_set_drvdata(pdev, rk322x);

	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_rk322x,
				      rk322x_dai, ARRAY_SIZE(rk322x_dai));
}

static int rk322x_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rk322xcodec_of_match[] = {
		{ .compatible = "rockchip,rk322x-codec", },
		{},
};
MODULE_DEVICE_TABLE(of, rk322xcodec_of_match);
#endif

static struct platform_driver rk322x_codec_driver = {
	.driver = {
		   .name = "rk322x-codec",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(rk322xcodec_of_match),
	},
	.probe = rk322x_platform_probe,
	.remove = rk322x_platform_remove,
};
module_platform_driver(rk322x_codec_driver);

MODULE_AUTHOR("Sugar Zhang <sugar.zhang@rock-chips.com>");
MODULE_DESCRIPTION("ASoC rk322x codec driver");
MODULE_LICENSE("GPL v2");
