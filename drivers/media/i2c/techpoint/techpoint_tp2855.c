// SPDX-License-Identifier: GPL-2.0
/*
 * techpoint techpoint lib
 *
 * Copyright (C) 2021 Rockchip Electronics Co., Ltd.
 */

#include "techpoint_tp2855.h"
#include "techpoint_dev.h"

static __maybe_unused const struct regval common_setting_297M_720p_regs[] = {
	{ 0x40, 0x08 },
	{ 0x01, 0xf0 },
	{ 0x02, 0x01 },
	{ 0x08, 0x0f },
	{ 0x20, 0x44 },
	{ 0x34, 0xe4 },
	{ 0x14, 0x44 },
	{ 0x15, 0x0d },
	{ 0x25, 0x04 },
	{ 0x26, 0x03 },
	{ 0x27, 0x09 },
	{ 0x29, 0x02 },
	{ 0x33, 0x07 },
	{ 0x33, 0x00 },
	{ 0x14, 0xc4 },
	{ 0x14, 0x44 },
	// {0x23, 0x02}, //vi test ok
	// {0x23, 0x00},
};

static __maybe_unused const struct regval common_setting_594M_1080p_regs[] = {
	{ 0x40, 0x08 },
	{ 0x01, 0xf0 },
	{ 0x02, 0x01 },
	{ 0x08, 0x0f },
	{ 0x20, 0x44 },
	{ 0x34, 0xe4 },
	{ 0x15, 0x0C },
	{ 0x25, 0x08 },
	{ 0x26, 0x06 },
	{ 0x27, 0x11 },
	{ 0x29, 0x0a },
	{ 0x33, 0x07 },
	{ 0x33, 0x00 },
	{ 0x14, 0x33 },
	{ 0x14, 0xb3 },
	{ 0x14, 0x33 },
	// {0x23, 0x02}, //vi test ok
	// {0x23, 0x00},
};

static struct techpoint_video_modes supported_modes[] = {
	{
	 .bus_fmt = MEDIA_BUS_FMT_UYVY8_2X8,
	 .width = 1920,
	 .height = 1080,
	 .max_fps = {
		     .numerator = 10000,
		     .denominator = 250000,
		     },
	 .link_freq_value = TP2855_LINK_FREQ_594M,
	 .common_reg_list = common_setting_594M_1080p_regs,
	 .common_reg_size = ARRAY_SIZE(common_setting_594M_1080p_regs),
	 .bpp = 8,
	 .lane = 4,
	 .vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	 .vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_1,
	 .vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_2,
	 .vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_3,
	  },
	{
	 .bus_fmt = MEDIA_BUS_FMT_UYVY8_2X8,
	 .width = 1280,
	 .height = 720,
	 .max_fps = {
		     .numerator = 10000,
		     .denominator = 250000,
		     },
	 .link_freq_value = TP2855_LINK_FREQ_297M,
	 .common_reg_list = common_setting_297M_720p_regs,
	 .common_reg_size = ARRAY_SIZE(common_setting_297M_720p_regs),
	 .bpp = 8,
	 .lane = 4,
	 .vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	 .vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_1,
	 .vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_2,
	 .vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_3,
	  },
};

int tp2855_initialize(struct techpoint *techpoint)
{
	int array_size = 0;
	struct i2c_client *client = techpoint->client;
	struct device *dev = &client->dev;

	techpoint->video_modes_num = ARRAY_SIZE(supported_modes);
	array_size =
	    sizeof(struct techpoint_video_modes) * techpoint->video_modes_num;
	techpoint->video_modes = devm_kzalloc(dev, array_size, GFP_KERNEL);
	memcpy(techpoint->video_modes, supported_modes, array_size);

	techpoint->cur_video_mode = &techpoint->video_modes[0];

	return 0;
}

int tp2855_get_channel_input_status(struct i2c_client *client, u8 ch)
{
	u8 val = 0;

	techpoint_write_reg(client, PAGE_REG, ch);
	techpoint_read_reg(client, INPUT_STATUS_REG, &val);
	dev_dbg(&client->dev, "input_status ch %d : %x\n", ch, val);

	return (val & INPUT_STATUS_MASK) ? 0 : 1;
}

int tp2855_get_all_input_status(struct i2c_client *client, u8 *detect_status)
{
	u8 val = 0, i;

	for (i = 0; i < PAD_MAX; i++) {
		techpoint_write_reg(client, PAGE_REG, i);
		techpoint_read_reg(client, INPUT_STATUS_REG, &val);
		detect_status[i] = tp2855_get_channel_input_status(client, i);
	}

	return 0;
}

int tp2855_set_channel_reso(struct i2c_client *client, int ch,
			    enum techpoint_support_reso reso)
{
	int val = reso;
	u8 tmp;
	const unsigned char SYS_MODE[5] = { 0x01, 0x02, 0x04, 0x08, 0x0f };

	techpoint_write_reg(client, 0x40, ch);

	switch (val) {
	case TECHPOINT_S_RESO_1080P_30:
		dev_err(&client->dev, "set channel %d 1080P_30, TBD\n", ch);
		techpoint_read_reg(client, 0xf5, &tmp);
		tmp &= ~SYS_MODE[ch];
		techpoint_write_reg(client, 0xf5, tmp);
		techpoint_write_reg(client, 0x02, 0x40);
		techpoint_write_reg(client, 0x07, 0xc0);
		techpoint_write_reg(client, 0x0b, 0xc0);
		techpoint_write_reg(client, 0x0c, 0x03);
		techpoint_write_reg(client, 0x0d, 0x50);
		techpoint_write_reg(client, 0x15, 0x03);
		techpoint_write_reg(client, 0x16, 0xd2);
		techpoint_write_reg(client, 0x17, 0x80);
		techpoint_write_reg(client, 0x18, 0x29);
		techpoint_write_reg(client, 0x19, 0x38);
		techpoint_write_reg(client, 0x1a, 0x47);
		techpoint_write_reg(client, 0x1c, 0x08);
		techpoint_write_reg(client, 0x1d, 0x98);
		techpoint_write_reg(client, 0x20, 0x30);
		techpoint_write_reg(client, 0x21, 0x84);
		techpoint_write_reg(client, 0x22, 0x36);
		techpoint_write_reg(client, 0x23, 0x3c);
		techpoint_write_reg(client, 0x2b, 0x60);
		techpoint_write_reg(client, 0x2c, 0x0a);
		techpoint_write_reg(client, 0x2d, 0x30);
		techpoint_write_reg(client, 0x2e, 0x70);
		techpoint_write_reg(client, 0x30, 0x48);
		techpoint_write_reg(client, 0x31, 0xbb);
		techpoint_write_reg(client, 0x32, 0x2e);
		techpoint_write_reg(client, 0x33, 0x90);
		techpoint_write_reg(client, 0x35, 0x05);
		techpoint_write_reg(client, 0x38, 0x00);
		techpoint_write_reg(client, 0x39, 0x1C);
		//def ahd config
		techpoint_write_reg(client, 0x02, 0x44);
		techpoint_write_reg(client, 0x0d, 0x72);
		techpoint_write_reg(client, 0x15, 0x01);
		techpoint_write_reg(client, 0x16, 0xf0);
		techpoint_write_reg(client, 0x20, 0x38);
		techpoint_write_reg(client, 0x21, 0x46);
		techpoint_write_reg(client, 0x25, 0xfe);
		techpoint_write_reg(client, 0x26, 0x0d);
		techpoint_write_reg(client, 0x2c, 0x3a);
		techpoint_write_reg(client, 0x2d, 0x54);
		techpoint_write_reg(client, 0x2e, 0x40);
		techpoint_write_reg(client, 0x30, 0xa5);
		techpoint_write_reg(client, 0x31, 0x95);
		techpoint_write_reg(client, 0x32, 0xe0);
		techpoint_write_reg(client, 0x33, 0x60);
		break;
	case TECHPOINT_S_RESO_1080P_25:
		dev_err(&client->dev, "set channel %d 1080P_25\n", ch);
		techpoint_read_reg(client, 0xf5, &tmp);
		tmp &= ~SYS_MODE[ch];
		techpoint_write_reg(client, 0xf5, tmp);
		techpoint_write_reg(client, 0x02, 0x40);
		techpoint_write_reg(client, 0x07, 0xc0);
		techpoint_write_reg(client, 0x0b, 0xc0);
		techpoint_write_reg(client, 0x0c, 0x03);
		techpoint_write_reg(client, 0x0d, 0x50);
		techpoint_write_reg(client, 0x15, 0x03);
		techpoint_write_reg(client, 0x16, 0xd2);
		techpoint_write_reg(client, 0x17, 0x80);
		techpoint_write_reg(client, 0x18, 0x29);
		techpoint_write_reg(client, 0x19, 0x38);
		techpoint_write_reg(client, 0x1a, 0x47);
		techpoint_write_reg(client, 0x1c, 0x0a);
		techpoint_write_reg(client, 0x1d, 0x50);
		techpoint_write_reg(client, 0x20, 0x30);
		techpoint_write_reg(client, 0x21, 0x84);
		techpoint_write_reg(client, 0x22, 0x36);
		techpoint_write_reg(client, 0x23, 0x3c);
		techpoint_write_reg(client, 0x2b, 0x60);
		techpoint_write_reg(client, 0x2c, 0x0a);
		techpoint_write_reg(client, 0x2d, 0x30);
		techpoint_write_reg(client, 0x2e, 0x70);
		techpoint_write_reg(client, 0x30, 0x48);
		techpoint_write_reg(client, 0x31, 0xbb);
		techpoint_write_reg(client, 0x32, 0x2e);
		techpoint_write_reg(client, 0x33, 0x90);
		techpoint_write_reg(client, 0x35, 0x05);
		techpoint_write_reg(client, 0x38, 0x00);
		techpoint_write_reg(client, 0x39, 0x1C);
		//def ahd config
		techpoint_write_reg(client, 0x02, 0x44);
		techpoint_write_reg(client, 0x0d, 0x73);
		techpoint_write_reg(client, 0x15, 0x01);
		techpoint_write_reg(client, 0x16, 0xf0);
		techpoint_write_reg(client, 0x20, 0x3c);
		techpoint_write_reg(client, 0x21, 0x46);
		techpoint_write_reg(client, 0x25, 0xfe);
		techpoint_write_reg(client, 0x26, 0x0d);
		techpoint_write_reg(client, 0x2c, 0x3a);
		techpoint_write_reg(client, 0x2d, 0x54);
		techpoint_write_reg(client, 0x2e, 0x40);
		techpoint_write_reg(client, 0x30, 0xa5);
		techpoint_write_reg(client, 0x31, 0x86);
		techpoint_write_reg(client, 0x32, 0xfb);
		techpoint_write_reg(client, 0x33, 0x60);
		break;
	case TECHPOINT_S_RESO_720P_30:
		dev_err(&client->dev, "set channel %d 720P_30\n", ch);
		techpoint_read_reg(client, 0xf5, &tmp);
		tmp |= SYS_MODE[ch];
		techpoint_write_reg(client, 0xf5, tmp);
		techpoint_write_reg(client, 0x02, 0x42);
		techpoint_write_reg(client, 0x07, 0xc0);
		techpoint_write_reg(client, 0x0b, 0xc0);
		techpoint_write_reg(client, 0x0c, 0x13);
		techpoint_write_reg(client, 0x0d, 0x50);
		techpoint_write_reg(client, 0x15, 0x13);
		techpoint_write_reg(client, 0x16, 0x15);
		techpoint_write_reg(client, 0x17, 0x00);
		techpoint_write_reg(client, 0x18, 0x19);
		techpoint_write_reg(client, 0x19, 0xd0);
		techpoint_write_reg(client, 0x1a, 0x25);
		techpoint_write_reg(client, 0x1c, 0x06);
		techpoint_write_reg(client, 0x1d, 0x72);
		techpoint_write_reg(client, 0x20, 0x30);
		techpoint_write_reg(client, 0x21, 0x84);
		techpoint_write_reg(client, 0x22, 0x36);
		techpoint_write_reg(client, 0x23, 0x3c);
		techpoint_write_reg(client, 0x2b, 0x60);
		techpoint_write_reg(client, 0x2c, 0x0a);
		techpoint_write_reg(client, 0x2d, 0x30);
		techpoint_write_reg(client, 0x2e, 0x70);
		techpoint_write_reg(client, 0x30, 0x48);
		techpoint_write_reg(client, 0x31, 0xbb);
		techpoint_write_reg(client, 0x32, 0x2e);
		techpoint_write_reg(client, 0x33, 0x90);
		techpoint_write_reg(client, 0x35, 0x25);
		techpoint_write_reg(client, 0x38, 0x00);
		techpoint_write_reg(client, 0x39, 0x18);
		//def ahd config
		techpoint_write_reg(client, 0x02, 0x46);
		techpoint_write_reg(client, 0x0d, 0x70);
		techpoint_write_reg(client, 0x20, 0x40);
		techpoint_write_reg(client, 0x21, 0x46);
		techpoint_write_reg(client, 0x25, 0xfe);
		techpoint_write_reg(client, 0x26, 0x01);
		techpoint_write_reg(client, 0x2c, 0x3a);
		techpoint_write_reg(client, 0x2d, 0x5a);
		techpoint_write_reg(client, 0x2e, 0x40);
		techpoint_write_reg(client, 0x30, 0x9d);
		techpoint_write_reg(client, 0x31, 0xca);
		techpoint_write_reg(client, 0x32, 0x01);
		techpoint_write_reg(client, 0x33, 0xd0);
		break;
	case TECHPOINT_S_RESO_720P_25:
		dev_err(&client->dev, "set channel %d 720P_25\n", ch);
		techpoint_read_reg(client, 0xf5, &tmp);
		tmp |= SYS_MODE[ch];
		techpoint_write_reg(client, 0xf5, tmp);
		techpoint_write_reg(client, 0x02, 0x42);
		techpoint_write_reg(client, 0x07, 0xc0);
		techpoint_write_reg(client, 0x0b, 0xc0);
		techpoint_write_reg(client, 0x0c, 0x13);
		techpoint_write_reg(client, 0x0d, 0x50);
		techpoint_write_reg(client, 0x15, 0x13);
		techpoint_write_reg(client, 0x16, 0x15);
		techpoint_write_reg(client, 0x17, 0x00);
		techpoint_write_reg(client, 0x18, 0x19);
		techpoint_write_reg(client, 0x19, 0xd0);
		techpoint_write_reg(client, 0x1a, 0x25);
		techpoint_write_reg(client, 0x1c, 0x07);
		techpoint_write_reg(client, 0x1d, 0xbc);
		techpoint_write_reg(client, 0x20, 0x30);
		techpoint_write_reg(client, 0x21, 0x84);
		techpoint_write_reg(client, 0x22, 0x36);
		techpoint_write_reg(client, 0x23, 0x3c);
		techpoint_write_reg(client, 0x2b, 0x60);
		techpoint_write_reg(client, 0x2c, 0x0a);
		techpoint_write_reg(client, 0x2d, 0x30);
		techpoint_write_reg(client, 0x2e, 0x70);
		techpoint_write_reg(client, 0x30, 0x48);
		techpoint_write_reg(client, 0x31, 0xbb);
		techpoint_write_reg(client, 0x32, 0x2e);
		techpoint_write_reg(client, 0x33, 0x90);
		techpoint_write_reg(client, 0x35, 0x25);
		techpoint_write_reg(client, 0x38, 0x00);
		techpoint_write_reg(client, 0x39, 0x18);
		//def ahd config
		techpoint_write_reg(client, 0x02, 0x46);
		techpoint_write_reg(client, 0x0d, 0x71);
		techpoint_write_reg(client, 0x20, 0x40);
		techpoint_write_reg(client, 0x21, 0x46);
		techpoint_write_reg(client, 0x25, 0xfe);
		techpoint_write_reg(client, 0x26, 0x01);
		techpoint_write_reg(client, 0x2c, 0x3a);
		techpoint_write_reg(client, 0x2d, 0x5a);
		techpoint_write_reg(client, 0x2e, 0x40);
		techpoint_write_reg(client, 0x30, 0x9e);
		techpoint_write_reg(client, 0x31, 0x20);
		techpoint_write_reg(client, 0x32, 0x10);
		techpoint_write_reg(client, 0x33, 0x90);
		break;
	default:
		dev_err(&client->dev,
			"set channel %d UNSUPPORT, default 1080P_25\n", ch);
		techpoint_read_reg(client, 0xf5, &tmp);
		tmp &= ~SYS_MODE[ch];
		techpoint_write_reg(client, 0xf5, tmp);
		techpoint_write_reg(client, 0x02, 0x40);
		techpoint_write_reg(client, 0x07, 0xc0);
		techpoint_write_reg(client, 0x0b, 0xc0);
		techpoint_write_reg(client, 0x0c, 0x03);
		techpoint_write_reg(client, 0x0d, 0x50);
		techpoint_write_reg(client, 0x15, 0x03);
		techpoint_write_reg(client, 0x16, 0xd2);
		techpoint_write_reg(client, 0x17, 0x80);
		techpoint_write_reg(client, 0x18, 0x29);
		techpoint_write_reg(client, 0x19, 0x38);
		techpoint_write_reg(client, 0x1a, 0x47);
		techpoint_write_reg(client, 0x1c, 0x0a);
		techpoint_write_reg(client, 0x1d, 0x50);
		techpoint_write_reg(client, 0x20, 0x30);
		techpoint_write_reg(client, 0x21, 0x84);
		techpoint_write_reg(client, 0x22, 0x36);
		techpoint_write_reg(client, 0x23, 0x3c);
		techpoint_write_reg(client, 0x2b, 0x60);
		techpoint_write_reg(client, 0x2c, 0x0a);
		techpoint_write_reg(client, 0x2d, 0x30);
		techpoint_write_reg(client, 0x2e, 0x70);
		techpoint_write_reg(client, 0x30, 0x48);
		techpoint_write_reg(client, 0x31, 0xbb);
		techpoint_write_reg(client, 0x32, 0x2e);
		techpoint_write_reg(client, 0x33, 0x90);
		techpoint_write_reg(client, 0x35, 0x05);
		techpoint_write_reg(client, 0x38, 0x00);
		techpoint_write_reg(client, 0x39, 0x1C);
		//def ahd config
		techpoint_write_reg(client, 0x02, 0x44);
		techpoint_write_reg(client, 0x0d, 0x73);
		techpoint_write_reg(client, 0x15, 0x01);
		techpoint_write_reg(client, 0x16, 0xf0);
		techpoint_write_reg(client, 0x20, 0x3c);
		techpoint_write_reg(client, 0x21, 0x46);
		techpoint_write_reg(client, 0x25, 0xfe);
		techpoint_write_reg(client, 0x26, 0x0d);
		techpoint_write_reg(client, 0x2c, 0x3a);
		techpoint_write_reg(client, 0x2d, 0x54);
		techpoint_write_reg(client, 0x2e, 0x40);
		techpoint_write_reg(client, 0x30, 0xa5);
		techpoint_write_reg(client, 0x31, 0x86);
		techpoint_write_reg(client, 0x32, 0xfb);
		techpoint_write_reg(client, 0x33, 0x60);
		break;
	}

#if TECHPOINT_TEST_PATTERN
	techpoint_write_reg(client, 0x2a, 0x3c);
#endif

	return 0;
}

int tp2855_get_channel_reso(struct i2c_client *client, int ch)
{
	u8 detect_fmt = 0xff;
	u8 reso = 0xff;

	techpoint_write_reg(client, 0x40, ch);
	techpoint_read_reg(client, 0x03, &detect_fmt);
	reso = detect_fmt & 0x7;

	switch (reso) {
	case TP2855_CVSTD_1080P_30:
		dev_err(&client->dev, "detect channel %d 1080P_30\n", ch);
		return TECHPOINT_S_RESO_1080P_30;
		break;
	case TP2855_CVSTD_1080P_25:
		dev_err(&client->dev, "detect channel %d 1080P_25\n", ch);
		return TECHPOINT_S_RESO_1080P_25;
		break;
	case TP2855_CVSTD_720P_30:
		dev_err(&client->dev, "detect channel %d 720P_30\n", ch);
		return TECHPOINT_S_RESO_720P_30;
		break;
	case TP2855_CVSTD_720P_25:
		dev_err(&client->dev, "detect channel %d 720P_25\n", ch);
		return TECHPOINT_S_RESO_720P_25;
		break;
	default:
		dev_err(&client->dev,
			"detect channel %d UNSUPPORT, default 1080P_25\n", ch);
		return TECHPOINT_S_RESO_1080P_25;
		break;
	}

	return reso;
}

int tp2855_set_quick_stream(struct i2c_client *client, u32 stream)
{
	if (stream) {
		techpoint_write_reg(client, 0x40, 0x8);
		techpoint_write_reg(client, 0x23, 0x0);
	} else {
		techpoint_write_reg(client, 0x40, 0x8);
		techpoint_write_reg(client, 0x23, 0x2);
	}

	return 0;
}
