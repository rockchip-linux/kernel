/*
 * TAS571x amplifier audio driver
 *
 * Copyright (C) 2015 Google, Inc.
 * Copyright (c) 2013 Daniel Mack <zonque@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/stddef.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "tas571x.h"

#define TAS571X_NUM_SUPPLIES		2
static const char * const tas571x_supply_names[TAS571X_NUM_SUPPLIES] = {
	"VDD",
	"PVDD",
};

struct tas571x_private {
	struct regmap			*regmap;
	struct regulator_bulk_data	supplies[TAS571X_NUM_SUPPLIES];
	struct clk			*mclk;
	unsigned int			format;
	enum snd_soc_bias_level		bias_level;
	struct gpio_desc		*reset_gpio;
	struct gpio_desc		*pdn_gpio;
	unsigned int			dev_id;
	struct snd_soc_codec_driver	codec_driver;
};

static int tas571x_set_sysclk(struct snd_soc_dai *dai,
			      int clk_id, unsigned int freq, int dir)
{
	/*
	 * TAS5717 datasheet pg 21: "The DAP can autodetect and set the
	 * internal clock-control logic to the appropriate settings for all
	 * supported clock rates as defined in the clock control register."
	 */
	return 0;
}

static int tas571x_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct tas571x_private *priv = snd_soc_codec_get_drvdata(dai->codec);

	return regmap_update_bits(priv->regmap,
				  TAS571X_SOFT_MUTE_REG,
				  TAS571X_SOFT_MUTE_CH_MASK,
				  mute ? TAS571X_SOFT_MUTE_CH_MASK : 0);
}

static int tas571x_set_dai_fmt(struct snd_soc_dai *dai, unsigned int format)
{
	struct tas571x_private *priv = snd_soc_codec_get_drvdata(dai->codec);

	priv->format = format;

	return 0;
}

static int tas571x_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct tas571x_private *priv = snd_soc_codec_get_drvdata(dai->codec);
	u32 val;

	switch (priv->format & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_RIGHT_J:
		val = 0x00;
		break;
	case SND_SOC_DAIFMT_I2S:
		val = 0x03;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val = 0x06;
		break;
	default:
		return -EINVAL;
	}

	val += (clamp(params_width(params), 16, 24) >> 2) - 4;

	return regmap_update_bits(priv->regmap, TAS571X_SDI_REG,
				  TAS571X_SDI_FMT_MASK, val);
}

static int tas571x_set_shutdown(struct tas571x_private *priv, bool is_shutdown)
{
	return regmap_update_bits(priv->regmap, TAS571X_SYS_CTRL_2_REG,
		TAS571X_SYS_CTRL_2_SDN_MASK,
		is_shutdown ? TAS571X_SYS_CTRL_2_SDN_MASK : 0);
}

static int tas571x_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	struct tas571x_private *priv = snd_soc_codec_get_drvdata(codec);
	int ret, assert_pdn = 0;

	if (priv->bias_level == level)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_PREPARE:
		if (!IS_ERR(priv->mclk)) {
			ret = clk_prepare_enable(priv->mclk);
			if (ret) {
				dev_err(codec->dev,
					"Failed to enable master clock\n");
				return ret;
			}
		}

		ret = tas571x_set_shutdown(priv, false);
		if (ret)
			return ret;
		break;
	case SND_SOC_BIAS_STANDBY:
		ret = tas571x_set_shutdown(priv, true);
		if (ret)
			return ret;

		if (!IS_ERR(priv->mclk))
			clk_disable_unprepare(priv->mclk);
		break;
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_OFF:
		/* Note that this kills I2C accesses. */
		assert_pdn = 1;
		break;
	}

	if (!IS_ERR(priv->pdn_gpio))
		gpiod_set_value(priv->pdn_gpio, !assert_pdn);

	priv->bias_level = level;
	return 0;
}

static const struct snd_soc_dai_ops tas571x_dai_ops = {
	.set_sysclk	= tas571x_set_sysclk,
	.set_fmt	= tas571x_set_dai_fmt,
	.hw_params	= tas571x_hw_params,
	.digital_mute	= tas571x_digital_mute,
};

static const unsigned int tas5717_volume_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0, 0x3ff, TLV_DB_SCALE_ITEM(-10500, 125, 1),
};

static const struct snd_kcontrol_new tas5717_controls[] = {
	SOC_SINGLE_TLV("Master volume",
		       TAS571X_MVOL_REG, 0, 0x3ff, 1,
		       tas5717_volume_tlv),
	SOC_DOUBLE_R_TLV("Speaker volume",
			 TAS571X_CH1_VOL_REG, TAS571X_CH2_VOL_REG,
			 0, 0x3ff, 1, tas5717_volume_tlv),
};

static const struct snd_soc_dapm_widget tas5717_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DACL", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DACR", NULL, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_OUTPUT("OUT_A"),
	SND_SOC_DAPM_OUTPUT("OUT_B"),
	SND_SOC_DAPM_OUTPUT("OUT_C"),
	SND_SOC_DAPM_OUTPUT("OUT_D"),
};

static const struct snd_soc_dapm_route tas5717_dapm_routes[] = {
	{ "DACL",  NULL, "Playback" },
	{ "DACR",  NULL, "Playback" },

	{ "OUT_A", NULL, "DACL" },
	{ "OUT_B", NULL, "DACL" },
	{ "OUT_C", NULL, "DACR" },
	{ "OUT_D", NULL, "DACR" },
};

static const struct snd_soc_codec_driver tas571x_codec = {
	.set_bias_level = tas571x_set_bias_level,
	.suspend_bias_off = true,

	.dapm_widgets = tas5717_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tas5717_dapm_widgets),
	.dapm_routes = tas5717_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(tas5717_dapm_routes),
};

static int tas571x_register_size(struct tas571x_private *priv, unsigned int reg)
{
	switch (reg) {
	case TAS571X_MVOL_REG:
	case TAS571X_CH1_VOL_REG:
	case TAS571X_CH2_VOL_REG:
		return 2;
	default:
		return 1;
	}
}

static int tas571x_reg_write(void *context, unsigned int reg,
			     unsigned int value)
{
	struct i2c_client *client = context;
	struct tas571x_private *priv = i2c_get_clientdata(client);
	unsigned int i, size;
	uint8_t buf[5];
	int ret;

	size = tas571x_register_size(priv, reg);
	buf[0] = reg;

	for (i = size; i >= 1; --i) {
		buf[i] = value;
		value >>= 8;
	}

	ret = i2c_master_send(client, buf, size + 1);
	if (ret == size + 1)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int tas571x_reg_read(void *context, unsigned int reg,
			    unsigned int *value)
{
	struct i2c_client *client = context;
	struct tas571x_private *priv = i2c_get_clientdata(client);
	uint8_t send_buf, recv_buf[4];
	struct i2c_msg msgs[2];
	unsigned int size;
	unsigned int i;
	int ret;

	size = tas571x_register_size(priv, reg);
	send_buf = reg;

	msgs[0].addr = client->addr;
	msgs[0].len = sizeof(send_buf);
	msgs[0].buf = &send_buf;
	msgs[0].flags = 0;

	msgs[1].addr = client->addr;
	msgs[1].len = size;
	msgs[1].buf = recv_buf;
	msgs[1].flags = I2C_M_RD;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	else if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*value = 0;

	for (i = 0; i < size; i++) {
		*value <<= 8;
		*value |= recv_buf[i];
	}

	return 0;
}
static const struct regmap_config tas571x_regmap = {
	.reg_bits = 8,
	.val_bits = 32,
	.reg_read = tas571x_reg_read,
	.reg_write = tas571x_reg_write,
	.cache_type = REGCACHE_RBTREE,
};

static struct snd_soc_dai_driver tas571x_dai = {
	.name = "tas571x-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE |
			   SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &tas571x_dai_ops,
};

static int tas571x_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct tas571x_private *priv;
	struct device *dev = &client->dev;
	int i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	i2c_set_clientdata(client, priv);

	priv->mclk = devm_clk_get(dev, "mclk");
	if (PTR_ERR(priv->mclk) == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	for (i = 0; i < TAS571X_NUM_SUPPLIES; i++)
		priv->supplies[i].supply = tas571x_supply_names[i];

	/*
	 * This will fall back to the dummy regulator if nothing is specified
	 * in DT.
	 */
	if (devm_regulator_bulk_get(dev, TAS571X_NUM_SUPPLIES,
				    priv->supplies)) {
		dev_err(dev, "Failed to get supplies\n");
		return -EINVAL;
	}
	if (regulator_bulk_enable(TAS571X_NUM_SUPPLIES, priv->supplies)) {
		dev_err(dev, "Failed to enable supplies\n");
		return -EINVAL;
	}

	priv->regmap = devm_regmap_init(dev, NULL, client, &tas571x_regmap);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	priv->pdn_gpio = devm_gpiod_get(dev, "pdn");
	if (!IS_ERR(priv->pdn_gpio)) {
		gpiod_direction_output(priv->pdn_gpio, 1);
	} else if (PTR_ERR(priv->pdn_gpio) != -ENOENT &&
		   PTR_ERR(priv->pdn_gpio) != -ENOSYS) {
		dev_warn(dev, "error requesting pdn_gpio: %ld\n",
			 PTR_ERR(priv->pdn_gpio));
	}

	priv->reset_gpio = devm_gpiod_get(dev, "reset");
	if (!IS_ERR(priv->reset_gpio)) {
		gpiod_direction_output(priv->reset_gpio, 0);
		usleep_range(100, 200);
		gpiod_set_value(priv->reset_gpio, 1);
		usleep_range(12000, 20000);
	} else if (PTR_ERR(priv->reset_gpio) != -ENOENT &&
		   PTR_ERR(priv->reset_gpio) != -ENOSYS) {
		dev_warn(dev, "error requesting reset_gpio: %ld\n",
			 PTR_ERR(priv->reset_gpio));
	}

	priv->bias_level = SND_SOC_BIAS_STANDBY;

	if (regmap_write(priv->regmap, TAS571X_OSC_TRIM_REG, 0))
		return -EIO;

	if (tas571x_set_shutdown(priv, true))
		return -EIO;

	memcpy(&priv->codec_driver, &tas571x_codec, sizeof(priv->codec_driver));
	priv->dev_id = id->driver_data;

	switch (id->driver_data) {
	case TAS571X_ID_5717:
	case TAS571X_ID_5719:
		priv->codec_driver.controls = tas5717_controls;
		priv->codec_driver.num_controls = ARRAY_SIZE(tas5717_controls);
		break;
	}

	return snd_soc_register_codec(&client->dev, &priv->codec_driver,
				      &tas571x_dai, 1);
}

static int tas571x_i2c_remove(struct i2c_client *client)
{
	struct tas571x_private *priv = i2c_get_clientdata(client);

	snd_soc_unregister_codec(&client->dev);
	regulator_bulk_disable(TAS571X_NUM_SUPPLIES, priv->supplies);

	return 0;
}

static const struct of_device_id tas571x_of_match[] = {
	{ .compatible = "ti,tas5717", },
	{ .compatible = "ti,tas5719", },
	{ }
};
MODULE_DEVICE_TABLE(of, tas571x_of_match);

static const struct i2c_device_id tas571x_i2c_id[] = {
	{ "tas5717", TAS571X_ID_5717 },
	{ "tas5719", TAS571X_ID_5719 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas571x_i2c_id);

static struct i2c_driver tas571x_i2c_driver = {
	.driver = {
		.name = "tas571x",
		.of_match_table = of_match_ptr(tas571x_of_match),
	},
	.probe = tas571x_i2c_probe,
	.remove = tas571x_i2c_remove,
	.id_table = tas571x_i2c_id,
};
module_i2c_driver(tas571x_i2c_driver);

MODULE_DESCRIPTION("ASoC TAS571x driver");
MODULE_AUTHOR("Kevin Cernekee <cernekee@chromium.org>");
MODULE_LICENSE("GPL");
