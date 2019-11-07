// SPDX-License-Identifier: GPL-2.0
/*
 * imx214 driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 add poweron function.
 * V0.0X01.0X02 fix mclk issue when probe multiple camera.
 * OTP function need to do.
 * V0.0X01.0X03 add enum_frame_interval function.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_gpio.h>

#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include "imx214_eeprom_head.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x03)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define IMX214_LANES			2
#define IMX214_BITS_PER_SAMPLE		8
#define MIPI_FREQ_FULL		600000000
#define MIPI_FREQ_BINNING		600000000
/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define IMX214_PIXEL_RATE_FULL_SIZE	320000000
#define IMX214_PIXEL_RATE_BINNING	320000000
#define IMX214_XVCLK_FREQ		24000000

#define CHIP_ID				0x0214
#define IMX214_REG_CHIP_ID		0x0016

#define IMX214_REG_CTRL_MODE		0x0100
#define IMX214_MODE_SW_STANDBY		0x0
#define IMX214_MODE_STREAMING		BIT(0)

#define IMX214_REG_EXPOSURE		0x0202
#define	IMX214_EXPOSURE_MIN		4
#define	IMX214_EXPOSURE_STEP		1
#define IMX214_VTS_MAX			0xffff

#define IMX214_REG_GAIN_H		0x0204
#define IMX214_REG_GAIN_L		0x0205
#define IMX214_GAIN_MIN			0x200
#define IMX214_GAIN_MAX			0x1fff
#define IMX214_GAIN_STEP		0x200
#define IMX214_GAIN_DEFAULT		0x400

#define IMX214_REG_TEST_PATTERN		0x5e00//0x0600
#define	IMX214_TEST_PATTERN_ENABLE	0x80
#define	IMX214_TEST_PATTERN_DISABLE	0x0

#define IMX214_REG_VTS			0x0340

#define REG_NULL			0xFFFF

#define IMX214_REG_VALUE_08BIT		1
#define IMX214_REG_VALUE_16BIT		2
#define IMX214_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define IMX214_NAME			"imx214"
#define IMX214_MEDIA_BUS_FMT		MEDIA_BUS_FMT_SBGGR10_1X10

/* OTP MACRO */
#define MODULE_BKX 0X01
#define MODULE_TYPE MODULE_BKX
#if MODULE_TYPE == MODULE_BKX
#define  RG_Ratio_Typical_Default (0x026e)
#define  BG_Ratio_Typical_Default (0x0280)
#else
#define  RG_Ratio_Typical_Default (0x16f)
#define  BG_Ratio_Typical_Default (0x16f)
#endif

static const char * const imx214_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define IMX214_NUM_SUPPLIES ARRAY_SIZE(imx214_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct imx214_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct imx214 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[IMX214_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct imx214_mode *cur_mode;
	unsigned int		lane_num;
	unsigned int		cfg_num;
	unsigned int		pixel_rate;

	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate_ctrl;
	struct imx214_otp_info *otp;
	struct rkmodule_inf	module_inf;
	struct rkmodule_awb_cfg	awb_cfg;
	struct rkmodule_lsc_cfg	lsc_cfg;
};

#define to_imx214(sd) container_of(sd, struct imx214, subdev)

struct imx214_id_name {
	u32 id;
	char name[RKMODULE_NAME_LEN];
};

static const struct imx214_id_name imx214_module_info[] = {
	{0x36, "GuangDongLiteArray"},
	{0x0d, "CameraKing"},
	{0x00, "Unknown"}
};

static const struct imx214_id_name imx214_lens_info[] = {
	{0x47, "Sunny 3923C"},
	{0x07, "Largen 9611A6"},
	{0x00, "Unknown"}
};

/*
 * Xclk 24Mhz
 */
static const struct regval imx214_global_regs[] = {
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x0101, 0x03},
	{0x0105, 0x01},
	{0x0106, 0x01},
	{0x4550, 0x02},
	{0x4601, 0x00},
	{0x4642, 0x05},
	{0x6227, 0x11},
	{0x6276, 0x00},
	{0x900E, 0x06},
	{0xA802, 0x90},
	{0xA803, 0x11},
	{0xA804, 0x62},
	{0xA805, 0x77},
	{0xA806, 0xAE},
	{0xA807, 0x34},
	{0xA808, 0xAE},
	{0xA809, 0x35},
	{0xA80A, 0x62},
	{0xA80B, 0x83},
	{0xAE33, 0x00},
	{0x4174, 0x00},
	{0x4175, 0x11},
	{0x4612, 0x29},
	{0x461B, 0x12},
	{0x461F, 0x06},
	{0x4635, 0x07},
	{0x4637, 0x30},
	{0x463F, 0x18},
	{0x4641, 0x0D},
	{0x465B, 0x12},
	{0x465F, 0x11},
	{0x4663, 0x11},
	{0x4667, 0x0F},
	{0x466F, 0x0F},
	{0x470E, 0x09},
	{0x4909, 0xAB},
	{0x490B, 0x95},
	{0x4915, 0x5D},
	{0x4A5F, 0xFF},
	{0x4A61, 0xFF},
	{0x4A73, 0x62},
	{0x4A85, 0x00},
	{0x4A87, 0xFF},
	{0x583C, 0x04},
	{0x620E, 0x04},
	{0x6EB2, 0x01},
	{0x6EB3, 0x00},
	{0x9300, 0x02},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	{0x0216, 0x00},
	{0x0217, 0x00},
	{0x0100, 0x00},
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 600Mbps
 */
static const struct regval imx214_2104x1560_regs[] = {
	{0x0114, 0x01,},
	{0x0220, 0x00,},
	{0x0221, 0x11,},
	{0x0222, 0x01,},
	{0x0340, 0x06,},
	{0x0341, 0x40,},
	{0x0342, 0x13,},
	{0x0343, 0x90,},
	{0x0344, 0x00,},
	{0x0345, 0x00,},
	{0x0346, 0x00,},
	{0x0347, 0x00,},
	{0x0348, 0x10,},
	{0x0349, 0x6F,},
	{0x034A, 0x0C,},
	{0x034B, 0x2F,},
	{0x0381, 0x01,},
	{0x0383, 0x01,},
	{0x0385, 0x01,},
	{0x0387, 0x01,},
	{0x0900, 0x01,},
	{0x0901, 0x22,},
	{0x0902, 0x02,},
	{0x3000, 0x35,},
	{0x3054, 0x01,},
	{0x305C, 0x11,},
	{0x0112, 0x0A,},
	{0x0113, 0x0A,},
	{0x034C, 0x08,},
	{0x034D, 0x38,},
	{0x034E, 0x06,},
	{0x034F, 0x18,},
	{0x0401, 0x00,},
	{0x0404, 0x00,},
	{0x0405, 0x10,},
	{0x0408, 0x00,},
	{0x0409, 0x00,},
	{0x040A, 0x00,},
	{0x040B, 0x00,},
	{0x040C, 0x08,},
	{0x040D, 0x38,},
	{0x040E, 0x06,},
	{0x040F, 0x18,},
	{0x0301, 0x05,},
	{0x0303, 0x04,},
	{0x0305, 0x03,},
	{0x0306, 0x00,},
	{0x0307, 0x96,},
	{0x0309, 0x0A,},
	{0x030B, 0x01,},
	{0x0310, 0x00,},
	{0x0820, 0x09,},
	{0x0821, 0x60,},
	{0x0822, 0x00,},
	{0x0823, 0x00,},
	{0x3A03, 0x06,},
	{0x3A04, 0x68,},
	{0x3A05, 0x01,},
	{0x0B06, 0x01,},
	{0x30A2, 0x00,},
	{0x30B4, 0x00,},
	{0x3A02, 0xFF,},
	{0x3011, 0x00,},
	{0x3013, 0x01,},

	{0x4170, 0x00,},
	{0x4171, 0x10,},
	{0x4176, 0x00,},
	{0x4177, 0x3C,},
	{0xAE20, 0x04,},
	{0xAE21, 0x5C,},
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 15fps
 * mipi_datarate per lane 600Mbps
 */
static const struct regval imx214_4208x3120_regs[] = {
	{0x0114, 0x01},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0222, 0x01},
	{0x0340, 0x0C},
	{0x0341, 0x58},
	{0x0342, 0x13},
	{0x0343, 0x90},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x10},
	{0x0349, 0x6F},
	{0x034A, 0x0C},
	{0x034B, 0x2F},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x00},
	{0x0902, 0x00},
	{0x3000, 0x35},
	{0x3054, 0x01},
	{0x305C, 0x11},
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x034C, 0x10},
	{0x034D, 0x70},
	{0x034E, 0x0C},
	{0x034F, 0x30},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x10},
	{0x040D, 0x70},
	{0x040E, 0x0C},
	{0x040F, 0x30},
	{0x0301, 0x05},
	{0x0303, 0x04},
	{0x0305, 0x03},
	{0x0306, 0x00},
	{0x0307, 0x96},
	{0x0309, 0x0A},
	{0x030B, 0x01},
	{0x0310, 0x00},
	{0x0820, 0x09},
	{0x0821, 0x60},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x3A03, 0x08},
	{0x3A04, 0x70},
	{0x3A05, 0x02},
	{0x0B06, 0x01},
	{0x30A2, 0x00},
	{0x30B4, 0x00},
	{0x3A02, 0xFF},
	{0x3011, 0x00},
	{0x3013, 0x01},

	{0x4170, 0x00},
	{0x4171, 0x10},
	{0x4176, 0x00},
	{0x4177, 0x3C},
	{0xAE20, 0x04},
	{0xAE21, 0x5C},
	{0x0100, 0x00},
	{REG_NULL, 0x00},
};

static const struct imx214_mode supported_modes_2lane[] = {
	{
		.width = 4208,
		.height = 3120,
		.max_fps = {
			.numerator = 10000,
			.denominator = 150000,
		},
		.exp_def = 0x0c70,
		.hts_def = 0x1390,
		.vts_def = 0x0c7a,
		.reg_list = imx214_4208x3120_regs,
	},
	{
		.width = 2104,
		.height = 1560,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0630,
		.hts_def = 0x1390,
		.vts_def = 0x0640,
		.reg_list = imx214_2104x1560_regs,
	},
};

static const struct imx214_mode *supported_modes;

static const s64 link_freq_menu_items[] = {
	MIPI_FREQ_FULL,
	MIPI_FREQ_BINNING
};

static const char * const imx214_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int imx214_write_reg(struct i2c_client *client, u16 reg,
	int len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int imx214_write_array(struct i2c_client *client,
	const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = imx214_write_reg(client, regs[i].addr,
			IMX214_REG_VALUE_08BIT,
			regs[i].val);
	return ret;
}

/* Read registers up to 4 at a time */
static int imx214_read_reg(struct i2c_client *client, u16 reg,
	unsigned int len, u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int imx214_get_reso_dist(const struct imx214_mode *mode,
	struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct imx214_mode *
	imx214_find_best_fit(struct imx214 *imx214,
						struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < imx214->cfg_num; i++) {
		dist = imx214_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int imx214_set_fmt(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *fmt)
{
	struct imx214 *imx214 = to_imx214(sd);
	const struct imx214_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&imx214->mutex);

	mode = imx214_find_best_fit(imx214, fmt);
	fmt->format.code = IMX214_MEDIA_BUS_FMT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&imx214->mutex);
		return -ENOTTY;
#endif
	} else {
		imx214->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(imx214->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(imx214->vblank, vblank_def,
					 IMX214_VTS_MAX - mode->height,
					 1, vblank_def);
		if (mode->width == 2104 && mode->height == 1560) {
			__v4l2_ctrl_s_ctrl(imx214->link_freq,
				link_freq_menu_items[1]);
			__v4l2_ctrl_s_ctrl_int64(imx214->pixel_rate_ctrl,
				IMX214_PIXEL_RATE_BINNING);
		} else {
			__v4l2_ctrl_s_ctrl(imx214->link_freq,
				link_freq_menu_items[0]);
			__v4l2_ctrl_s_ctrl_int64(imx214->pixel_rate_ctrl,
				IMX214_PIXEL_RATE_FULL_SIZE);
		}
	}
	mutex_unlock(&imx214->mutex);

	return 0;
}

static int imx214_get_fmt(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *fmt)
{
	struct imx214 *imx214 = to_imx214(sd);
	const struct imx214_mode *mode = imx214->cur_mode;

	mutex_lock(&imx214->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&imx214->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = IMX214_MEDIA_BUS_FMT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&imx214->mutex);

	return 0;
}

static int imx214_enum_mbus_code(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = IMX214_MEDIA_BUS_FMT;

	return 0;
}

static int imx214_enum_frame_sizes(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx214 *imx214 = to_imx214(sd);

	if (fse->index >= imx214->cfg_num)
		return -EINVAL;

	if (fse->code != IMX214_MEDIA_BUS_FMT)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int imx214_enable_test_pattern(struct imx214 *imx214, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | IMX214_TEST_PATTERN_ENABLE;
	else
		val = IMX214_TEST_PATTERN_DISABLE;

	return imx214_write_reg(imx214->client,
			IMX214_REG_TEST_PATTERN,
			IMX214_REG_VALUE_08BIT,
			val);
}

static int imx214_g_frame_interval(struct v4l2_subdev *sd,
	struct v4l2_subdev_frame_interval *fi)
{
	struct imx214 *imx214 = to_imx214(sd);
	const struct imx214_mode *mode = imx214->cur_mode;

	mutex_lock(&imx214->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&imx214->mutex);

	return 0;
}

static void imx214_get_otp(struct imx214_otp_info *otp,
			       struct rkmodule_inf *inf)
{
	u32 i;

	/* fac */
	if (otp->flag & 0x80) {
		inf->fac.flag = 1;
		inf->fac.year = otp->year;
		inf->fac.month = otp->month;
		inf->fac.day = otp->day;
		for (i = 0; i < ARRAY_SIZE(imx214_module_info) - 1; i++) {
			if (imx214_module_info[i].id == otp->module_id)
				break;
		}
		strlcpy(inf->fac.module, imx214_module_info[i].name,
			sizeof(inf->fac.module));

		for (i = 0; i < ARRAY_SIZE(imx214_lens_info) - 1; i++) {
			if (imx214_lens_info[i].id == otp->lens_id)
				break;
		}
		strlcpy(inf->fac.lens, imx214_lens_info[i].name,
			sizeof(inf->fac.lens));
	}
	/* awb */
	if (otp->flag & 0x40) {
		inf->awb.flag = 1;
		inf->awb.r_value = otp->rg_ratio;
		inf->awb.b_value = otp->bg_ratio;
		inf->awb.gr_value = 0x400;
		inf->awb.gb_value = 0x400;

		inf->awb.golden_r_value = 0;
		inf->awb.golden_b_value = 0;
		inf->awb.golden_gr_value = 0;
		inf->awb.golden_gb_value = 0;
	}
	/* af */
	if (otp->flag & 0x20) {
		inf->af.flag = 1;
		inf->af.vcm_start = otp->vcm_start;
		inf->af.vcm_end = otp->vcm_end;
		inf->af.vcm_dir = otp->vcm_dir;
	}
	/* lsc */
	if (otp->flag & 0x10) {
		inf->lsc.flag = 1;
		inf->lsc.decimal_bits = 0;
		inf->lsc.lsc_w = 9;
		inf->lsc.lsc_h = 14;

		for (i = 0; i < 126; i++) {
			inf->lsc.lsc_r[i] = otp->lenc[i];
			inf->lsc.lsc_gr[i] = otp->lenc[i + 126];
			inf->lsc.lsc_gb[i] = otp->lenc[i + 252];
			inf->lsc.lsc_b[i] = otp->lenc[i + 378];
		}
	}
}

static void imx214_get_module_inf(struct imx214 *imx214,
	struct rkmodule_inf *inf)
{
	struct imx214_otp_info *otp = imx214->otp;

	strlcpy(inf->base.sensor, IMX214_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module,
		imx214->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, imx214->len_name, sizeof(inf->base.lens));
	if (otp)
		imx214_get_otp(otp, inf);
}

static void imx214_set_awb_cfg(struct imx214 *imx214,
			       struct rkmodule_awb_cfg *cfg)
{
	mutex_lock(&imx214->mutex);
	memcpy(&imx214->awb_cfg, cfg, sizeof(*cfg));
	mutex_unlock(&imx214->mutex);
}

static void imx214_set_lsc_cfg(struct imx214 *imx214,
			       struct rkmodule_lsc_cfg *cfg)
{
	mutex_lock(&imx214->mutex);
	memcpy(&imx214->lsc_cfg, cfg, sizeof(*cfg));
	mutex_unlock(&imx214->mutex);
}

static long imx214_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct imx214 *imx214 = to_imx214(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		imx214_get_module_inf(imx214, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_AWB_CFG:
		imx214_set_awb_cfg(imx214, (struct rkmodule_awb_cfg *)arg);
		break;
	case RKMODULE_LSC_CFG:
		imx214_set_lsc_cfg(imx214, (struct rkmodule_lsc_cfg *)arg);
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long imx214_compat_ioctl32(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *awb_cfg;
	struct rkmodule_lsc_cfg *lsc_cfg;
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx214_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		awb_cfg = kzalloc(sizeof(*awb_cfg), GFP_KERNEL);
		if (!awb_cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(awb_cfg, up, sizeof(*awb_cfg));
		if (!ret)
			ret = imx214_ioctl(sd, cmd, awb_cfg);
		kfree(awb_cfg);
		break;
	case RKMODULE_LSC_CFG:
		lsc_cfg = kzalloc(sizeof(*lsc_cfg), GFP_KERNEL);
		if (!lsc_cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(lsc_cfg, up, sizeof(*lsc_cfg));
		if (!ret)
			ret = imx214_ioctl(sd, cmd, lsc_cfg);
		kfree(lsc_cfg);
		break;
	default:
		ret = -ENOTTY;
		break;
	}
	return ret;
}
#endif

/*--------------------------------------------------------------------------*/
static int imx214_apply_otp(struct imx214 *imx214)
{
	int R_gain, G_gain, B_gain, base_gain;
	struct i2c_client *client = imx214->client;
	struct imx214_otp_info *otp_ptr = imx214->otp;
	struct rkmodule_awb_cfg *awb_cfg = &imx214->awb_cfg;
	struct rkmodule_lsc_cfg *lsc_cfg = &imx214->lsc_cfg;
	u32 golden_bg_ratio = 0;
	u32 golden_rg_ratio = 0;
	u32 golden_g_value = 0;
	u32 bg_ratio;
	u32 rg_ratio;
	//u32 g_value;
	u32 i;

	if (!otp_ptr)
		return 0;
	if (awb_cfg->enable) {
		golden_g_value = (awb_cfg->golden_gb_value +
			awb_cfg->golden_gr_value) / 2;
		if (golden_g_value != 0) {
			golden_rg_ratio = awb_cfg->golden_r_value * 0x400
				  / golden_g_value;
			golden_bg_ratio = awb_cfg->golden_b_value * 0x400
				  / golden_g_value;
		} else {
			golden_rg_ratio = RG_Ratio_Typical_Default;
			golden_bg_ratio = BG_Ratio_Typical_Default;
		}
	}
	/* apply OTP WB Calibration */
	if ((otp_ptr->flag & 0x40) && golden_bg_ratio && golden_rg_ratio) {
		rg_ratio = otp_ptr->rg_ratio;
		bg_ratio = otp_ptr->bg_ratio;
		dev_dbg(&client->dev, "rg:0x%x,bg:0x%x,gol rg:0x%x,bg:0x%x\n",
			rg_ratio, bg_ratio, golden_rg_ratio, golden_bg_ratio);
		/* calculate G gain */
		R_gain = golden_rg_ratio * 1000 / rg_ratio;
		B_gain = golden_bg_ratio * 1000 / bg_ratio;
		G_gain = 1000;
		if (R_gain < 1000 || B_gain < 1000) {
			if (R_gain < B_gain)
				base_gain = R_gain;
			else
				base_gain = B_gain;
		} else {
			base_gain = G_gain;
		}
		R_gain = 0x100 * R_gain / (base_gain);
		B_gain = 0x100 * B_gain / (base_gain);
		G_gain = 0x100 * G_gain / (base_gain);
		/* update sensor WB gain */
		if (R_gain > 0x100) {
			imx214_write_reg(client, 0x0210,
				IMX214_REG_VALUE_08BIT, R_gain >> 8);
			imx214_write_reg(client, 0x0211,
				IMX214_REG_VALUE_08BIT, R_gain & 0x00ff);
		}
		if (G_gain > 0x100) {
			imx214_write_reg(client, 0x020e,
				IMX214_REG_VALUE_08BIT, G_gain >> 8);
			imx214_write_reg(client, 0x020f,
				IMX214_REG_VALUE_08BIT, G_gain & 0x00ff);
			imx214_write_reg(client, 0x0214,
				IMX214_REG_VALUE_08BIT, G_gain >> 8);
			imx214_write_reg(client, 0x0215,
				IMX214_REG_VALUE_08BIT, G_gain & 0x00ff);
		}
		if (B_gain > 0x100) {
			imx214_write_reg(client, 0x0212,
				IMX214_REG_VALUE_08BIT, B_gain >> 8);
			imx214_write_reg(client, 0x0213,
				IMX214_REG_VALUE_08BIT, B_gain & 0x00ff);
		}
		dev_dbg(&client->dev, "apply awb gain: 0x%x, 0x%x, 0x%x\n",
			R_gain, G_gain, B_gain);
	}

	/* apply OTP Lenc Calibration */
	if ((otp_ptr->flag & 0x10) && lsc_cfg->enable) {
		for (i = 0; i < 504; i++) {
			imx214_write_reg(client, 0xA300 + i,
				IMX214_REG_VALUE_08BIT, otp_ptr->lenc[i]);
			dev_dbg(&client->dev, "apply lenc[%d]: 0x%x\n",
				i, otp_ptr->lenc[i]);
		}
		usleep_range(1000, 2000);
		//choose lsc table 1
		imx214_write_reg(client, 0x3021,
			IMX214_REG_VALUE_08BIT, 0x01);
		//enable lsc
		imx214_write_reg(client, 0x0B00,
			IMX214_REG_VALUE_08BIT, 0x01);
	}

	/* apply OTP SPC Calibration */
	if (otp_ptr->flag & 0x08) {
		for (i = 0; i < 63; i++) {
			imx214_write_reg(client, 0xD04C + i,
				IMX214_REG_VALUE_08BIT, otp_ptr->spc[i]);
			dev_dbg(&client->dev, "apply spc[%d]: 0x%x\n",
				i, otp_ptr->spc[i]);
			imx214_write_reg(client, 0xD08C + i,
				IMX214_REG_VALUE_08BIT, otp_ptr->spc[i + 63]);
			dev_dbg(&client->dev, "apply spc[%d]: 0x%x\n",
				i + 63, otp_ptr->spc[i + 63]);
		}
		//enable spc
		imx214_write_reg(client, 0x7BC8,
			IMX214_REG_VALUE_08BIT, 0x01);
	}
	return 0;
}

static int __imx214_start_stream(struct imx214 *imx214)
{
	int ret;

	ret = imx214_write_array(imx214->client, imx214->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&imx214->mutex);
	ret = v4l2_ctrl_handler_setup(&imx214->ctrl_handler);
	mutex_lock(&imx214->mutex);
	if (ret)
		return ret;

	ret = imx214_apply_otp(imx214);
	if (ret)
		return ret;

	return imx214_write_reg(imx214->client,
		IMX214_REG_CTRL_MODE,
		IMX214_REG_VALUE_08BIT,
		IMX214_MODE_STREAMING);
}

static int __imx214_stop_stream(struct imx214 *imx214)
{
	return imx214_write_reg(imx214->client,
		IMX214_REG_CTRL_MODE,
		IMX214_REG_VALUE_08BIT,
		IMX214_MODE_SW_STANDBY);
}

static int imx214_s_stream(struct v4l2_subdev *sd, int on)
{
	struct imx214 *imx214 = to_imx214(sd);
	struct i2c_client *client = imx214->client;
	int ret = 0;

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
		 imx214->cur_mode->width,
		 imx214->cur_mode->height,
		 DIV_ROUND_CLOSEST(imx214->cur_mode->max_fps.denominator,
			imx214->cur_mode->max_fps.numerator));

	mutex_lock(&imx214->mutex);
	on = !!on;
	if (on == imx214->streaming)
		goto unlock_and_return;

	if (on) {
		dev_info(&client->dev, "stream on!!!\n");
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __imx214_start_stream(imx214);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		dev_info(&client->dev, "stream off!!!\n");
		__imx214_stop_stream(imx214);
		pm_runtime_put(&client->dev);
	}

	imx214->streaming = on;

unlock_and_return:
	mutex_unlock(&imx214->mutex);

	return ret;
}

static int __imx214_power_on(struct imx214 *imx214);
static void __imx214_power_off(struct imx214 *imx214);

static int imx214_s_power(struct v4l2_subdev *sd, int on)
{
	struct imx214 *imx214 = to_imx214(sd);
	struct i2c_client *client = imx214->client;
	int ret = 0;

	dev_info(&client->dev, "%s(%d) on(%d)\n", __func__, __LINE__, on);
	mutex_lock(&imx214->mutex);

	/* If the power state is not modified - no work to do. */
	if (imx214->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		__imx214_power_on(imx214);
		ret = imx214_write_array(imx214->client, imx214_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		imx214->power_on = true;
		/* export gpio */
		if (!IS_ERR(imx214->reset_gpio))
			gpiod_export(imx214->reset_gpio, false);
		if (!IS_ERR(imx214->pwdn_gpio))
			gpiod_export(imx214->pwdn_gpio, false);

	} else {
		pm_runtime_put(&client->dev);
		__imx214_power_off(imx214);
		imx214->power_on = false;
		/* unexport gpio */
		if (!IS_ERR(imx214->reset_gpio))
			gpiod_unexport(imx214->reset_gpio);
		if (!IS_ERR(imx214->pwdn_gpio))
			gpiod_unexport(imx214->pwdn_gpio);
	}

unlock_and_return:
	mutex_unlock(&imx214->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 imx214_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, IMX214_XVCLK_FREQ / 1000 / 1000);
}

static int __imx214_power_on(struct imx214 *imx214)
{
	int ret;
	u32 delay_us;
	struct device *dev = &imx214->client->dev;

	dev_info(&imx214->client->dev, "%s(%d) enter!\n", __func__, __LINE__);

	if (!IS_ERR_OR_NULL(imx214->pins_default)) {
		ret = pinctrl_select_state(imx214->pinctrl,
			imx214->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(imx214->xvclk, IMX214_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(imx214->xvclk) != IMX214_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");

	ret = clk_prepare_enable(imx214->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(imx214->reset_gpio))
		gpiod_set_value_cansleep(imx214->reset_gpio, 1);

	ret = regulator_bulk_enable(IMX214_NUM_SUPPLIES, imx214->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(imx214->reset_gpio))
		gpiod_set_value_cansleep(imx214->reset_gpio, 0);

	usleep_range(500, 1000);
	if (!IS_ERR(imx214->pwdn_gpio))
		gpiod_set_value_cansleep(imx214->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = imx214_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(imx214->xvclk);

	return ret;
}

static void __imx214_power_off(struct imx214 *imx214)
{
	int ret;

	dev_info(&imx214->client->dev, "%s(%d) enter!\n", __func__, __LINE__);

	if (!IS_ERR(imx214->pwdn_gpio))
		gpiod_set_value_cansleep(imx214->pwdn_gpio, 0);
	clk_disable_unprepare(imx214->xvclk);
	if (!IS_ERR(imx214->reset_gpio))
		gpiod_set_value_cansleep(imx214->reset_gpio, 1);
	if (!IS_ERR_OR_NULL(imx214->pins_sleep)) {
		ret = pinctrl_select_state(imx214->pinctrl,
			imx214->pins_sleep);
		if (ret < 0)
			dev_dbg(&imx214->client->dev, "could not set pins\n");
	}
	regulator_bulk_disable(IMX214_NUM_SUPPLIES, imx214->supplies);
}

static int imx214_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx214 *imx214 = to_imx214(sd);

	return __imx214_power_on(imx214);
}

static int imx214_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx214 *imx214 = to_imx214(sd);

	__imx214_power_off(imx214);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int imx214_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx214 *imx214 = to_imx214(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct imx214_mode *def_mode = &supported_modes[0];

	mutex_lock(&imx214->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = IMX214_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&imx214->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int imx214_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct imx214 *imx214 = to_imx214(sd);

	if (fie->index >= imx214->cfg_num)
		return -EINVAL;

	if (fie->code != IMX214_MEDIA_BUS_FMT)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static const struct dev_pm_ops imx214_pm_ops = {
	SET_RUNTIME_PM_OPS(imx214_runtime_suspend,
		imx214_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops imx214_internal_ops = {
	.open = imx214_open,
};
#endif

static const struct v4l2_subdev_core_ops imx214_core_ops = {
	.s_power = imx214_s_power,
	.ioctl = imx214_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx214_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops imx214_video_ops = {
	.s_stream = imx214_s_stream,
	.g_frame_interval = imx214_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx214_pad_ops = {
	.enum_mbus_code = imx214_enum_mbus_code,
	.enum_frame_size = imx214_enum_frame_sizes,
	.enum_frame_interval = imx214_enum_frame_interval,
	.get_fmt = imx214_get_fmt,
	.set_fmt = imx214_set_fmt,
};

static const struct v4l2_subdev_ops imx214_subdev_ops = {
	.core	= &imx214_core_ops,
	.video	= &imx214_video_ops,
	.pad	= &imx214_pad_ops,
};

static int imx214_set_gain_reg(struct imx214 *imx214, u32 a_gain)
{
	int ret = 0;
	u32 gain_reg = 0;

	gain_reg = (512 - (512 * 512 / a_gain));
	if (gain_reg > 480)
		gain_reg = 480;

	ret = imx214_write_reg(imx214->client,
		IMX214_REG_GAIN_H,
		IMX214_REG_VALUE_08BIT,
		((gain_reg & 0x100) >> 8));
	ret |= imx214_write_reg(imx214->client,
		IMX214_REG_GAIN_L,
		IMX214_REG_VALUE_08BIT,
		(gain_reg & 0xff));
	return ret;
}

static int imx214_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx214 *imx214 = container_of(ctrl->handler,
					     struct imx214, ctrl_handler);
	struct i2c_client *client = imx214->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = imx214->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(imx214->exposure,
			imx214->exposure->minimum, max,
			imx214->exposure->step,
			imx214->exposure->default_value);
		break;
	}

	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = imx214_write_reg(imx214->client,
			IMX214_REG_EXPOSURE,
			IMX214_REG_VALUE_16BIT,
			ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx214_set_gain_reg(imx214, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = imx214_write_reg(imx214->client,
			IMX214_REG_VTS,
			IMX214_REG_VALUE_16BIT,
			ctrl->val + imx214->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx214_enable_test_pattern(imx214, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx214_ctrl_ops = {
	.s_ctrl = imx214_set_ctrl,
};

static int imx214_initialize_controls(struct imx214 *imx214)
{
	const struct imx214_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &imx214->ctrl_handler;
	mode = imx214->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &imx214->mutex;

	imx214->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
		V4L2_CID_LINK_FREQ, 1, 0,
		link_freq_menu_items);

	imx214->pixel_rate_ctrl = v4l2_ctrl_new_std(handler, NULL,
		V4L2_CID_PIXEL_RATE, 0, IMX214_PIXEL_RATE_FULL_SIZE,
		1, IMX214_PIXEL_RATE_FULL_SIZE);

	h_blank = mode->hts_def - mode->width;
	imx214->hblank = v4l2_ctrl_new_std(handler, NULL,
		V4L2_CID_HBLANK, h_blank, h_blank, 1, h_blank);
	if (imx214->hblank)
		imx214->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	imx214->vblank = v4l2_ctrl_new_std(handler, &imx214_ctrl_ops,
		V4L2_CID_VBLANK, vblank_def,
		IMX214_VTS_MAX - mode->height,
		1, vblank_def);

	exposure_max = mode->vts_def - 4;
	imx214->exposure = v4l2_ctrl_new_std(handler, &imx214_ctrl_ops,
		V4L2_CID_EXPOSURE, IMX214_EXPOSURE_MIN,
		exposure_max, IMX214_EXPOSURE_STEP,
		mode->exp_def);

	imx214->anal_gain = v4l2_ctrl_new_std(handler, &imx214_ctrl_ops,
		V4L2_CID_ANALOGUE_GAIN, IMX214_GAIN_MIN,
		IMX214_GAIN_MAX, IMX214_GAIN_STEP,
		IMX214_GAIN_DEFAULT);

	imx214->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
		&imx214_ctrl_ops, V4L2_CID_TEST_PATTERN,
		ARRAY_SIZE(imx214_test_pattern_menu) - 1,
		0, 0, imx214_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&imx214->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	imx214->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int imx214_check_sensor_id(struct imx214 *imx214,
				   struct i2c_client *client)
{
	struct device *dev = &imx214->client->dev;
	int ret = 0;
	u32 id = 0;

	ret = imx214_read_reg(client, IMX214_REG_CHIP_ID,
			       IMX214_REG_VALUE_16BIT, &id);

	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}
	dev_info(dev, "imx214 detected %04x sensor\n", id);
	return 0;
}

static int imx214_configure_regulators(struct imx214 *imx214)
{
	unsigned int i;

	for (i = 0; i < IMX214_NUM_SUPPLIES; i++)
		imx214->supplies[i].supply = imx214_supply_names[i];

	return devm_regulator_bulk_get(&imx214->client->dev,
		IMX214_NUM_SUPPLIES,
		imx214->supplies);
}

static int imx214_parse_of(struct imx214 *imx214)
{
	struct device *dev = &imx214->client->dev;
	struct device_node *endpoint;
	struct fwnode_handle *fwnode;
	int rval;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}
	fwnode = of_fwnode_handle(endpoint);
	rval = fwnode_property_read_u32_array(fwnode, "data-lanes", NULL, 0);
	if (rval <= 0) {
		dev_warn(dev, " Get mipi lane num failed!\n");
		return -1;
	}

	imx214->lane_num = rval;
	if (4 == imx214->lane_num) {
		dev_err(dev, "unsupported lane_num(%d)\n", imx214->lane_num);
	} else {
		imx214->cur_mode = &supported_modes_2lane[0];
		supported_modes = supported_modes_2lane;
		imx214->cfg_num = ARRAY_SIZE(supported_modes_2lane);

		/*pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
		imx214->pixel_rate = MIPI_FREQ_FULL * 2U *
						  imx214->lane_num / 8U;
		dev_info(dev, "lane_num(%d)  pixel_rate(%u)\n",
				 imx214->lane_num, imx214->pixel_rate);
	}
	return 0;
}

static void free_gpio(struct imx214 *imx214)
{
	dev_info(&imx214->client->dev, "%s(%d) enter!\n", __func__, __LINE__);

	if (!IS_ERR(imx214->pwdn_gpio))
		gpio_free(desc_to_gpio(imx214->pwdn_gpio));

	if (!IS_ERR(imx214->reset_gpio))
		gpio_free(desc_to_gpio(imx214->reset_gpio));
}

static int imx214_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct imx214 *imx214;
	struct v4l2_subdev *sd;
	char facing[2];
	struct device_node *eeprom_ctrl_node;
	struct i2c_client *eeprom_ctrl_client;
	struct v4l2_subdev *eeprom_ctrl;
	struct imx214_otp_info *otp_ptr;
	int ret;
	struct gpio_desc *pwdn_gpio, *reset_gpio;
	unsigned int pwdn = -1, reset = -1;
	enum of_gpio_flags flags;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	imx214 = devm_kzalloc(dev, sizeof(*imx214), GFP_KERNEL);
	if (!imx214)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
		&imx214->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
		&imx214->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
		&imx214->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
		&imx214->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	imx214->client = client;
	imx214->cur_mode = &supported_modes[0];

	imx214->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(imx214->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	imx214->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(imx214->reset_gpio)) {
		dev_info(dev, "Failed to get reset-gpios, maybe no use\n");
		reset = of_get_named_gpio_flags(node, "reset-gpios", 0, &flags);
		reset_gpio = gpio_to_desc(reset);
		if (IS_ERR(reset_gpio))
			dev_info(dev, "Failed to get reset-gpios again\n");
		else
			imx214->reset_gpio = reset_gpio;
	}

	imx214->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(imx214->pwdn_gpio)) {
		dev_info(dev, "Failed to get pwdn-gpios, maybe no use\n");
		pwdn = of_get_named_gpio_flags(node, "pwdn-gpios", 0, &flags);
		pwdn_gpio = gpio_to_desc(pwdn);
		if (IS_ERR(pwdn_gpio))
			dev_info(dev, "Failed to get pwdn-gpios again\n");
		else
			imx214->pwdn_gpio = pwdn_gpio;
	}

	ret = imx214_configure_regulators(imx214);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	ret = imx214_parse_of(imx214);
	if (ret != 0)
		return -EINVAL;

	imx214->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(imx214->pinctrl)) {
		imx214->pins_default =
			pinctrl_lookup_state(imx214->pinctrl,
				OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(imx214->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		imx214->pins_sleep =
			pinctrl_lookup_state(imx214->pinctrl,
				OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(imx214->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	mutex_init(&imx214->mutex);

	sd = &imx214->subdev;
	v4l2_i2c_subdev_init(sd, client, &imx214_subdev_ops);
	ret = imx214_initialize_controls(imx214);
	if (ret)
		goto err_destroy_mutex;

	ret = __imx214_power_on(imx214);
	if (ret)
		goto err_free_handler;

	ret = imx214_check_sensor_id(imx214, client);
	if (ret)
		goto err_power_off;

	eeprom_ctrl_node = of_parse_phandle(node, "eeprom-ctrl", 0);
	if (eeprom_ctrl_node) {
		eeprom_ctrl_client =
			of_find_i2c_device_by_node(eeprom_ctrl_node);
		of_node_put(eeprom_ctrl_node);
		if (IS_ERR_OR_NULL(eeprom_ctrl_client)) {
			dev_err(dev, "can not get node\n");
			goto continue_probe;
		}
		eeprom_ctrl = i2c_get_clientdata(eeprom_ctrl_client);
		if (IS_ERR_OR_NULL(eeprom_ctrl)) {
			dev_err(dev, "can not get eeprom i2c client\n");
		} else {
			otp_ptr = devm_kzalloc(dev, sizeof(*otp_ptr),
					  GFP_KERNEL);
			if (!otp_ptr)
				return -ENOMEM;
			ret = v4l2_subdev_call(eeprom_ctrl,
				core, ioctl, 0, otp_ptr);
			if (!ret) {
				imx214->otp = otp_ptr;
			} else {
				imx214->otp = NULL;
				devm_kfree(dev, otp_ptr);
			}
		}
	}

continue_probe:

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &imx214_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	imx214->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &imx214->pad, 0);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(imx214->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 imx214->module_index, facing,
		 IMX214_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__imx214_power_off(imx214);
	free_gpio(imx214);
err_free_handler:
	v4l2_ctrl_handler_free(&imx214->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&imx214->mutex);

	return ret;
}

static int imx214_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx214 *imx214 = to_imx214(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&imx214->ctrl_handler);
	mutex_destroy(&imx214->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__imx214_power_off(imx214);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id imx214_of_match[] = {
	{ .compatible = "sony,imx214" },
	{},
};
MODULE_DEVICE_TABLE(of, imx214_of_match);
#endif

static const struct i2c_device_id imx214_match_id[] = {
	{ "sony,imx214", 0 },
	{ },
};

static struct i2c_driver imx214_i2c_driver = {
	.driver = {
		.name = IMX214_NAME,
		.pm = &imx214_pm_ops,
		.of_match_table = of_match_ptr(imx214_of_match),
	},
	.probe		= &imx214_probe,
	.remove		= &imx214_remove,
	.id_table	= imx214_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&imx214_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&imx214_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Sony imx214 sensor driver");
MODULE_LICENSE("GPL v2");
