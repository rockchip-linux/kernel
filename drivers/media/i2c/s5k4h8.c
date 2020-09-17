// SPDX-License-Identifier: GPL-2.0
/*
 * s5k4h8 driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X0 first version,otp is not verified
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/slab.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x0)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define S5K4H8_LANES			4
#define S5K4H8_BITS_PER_SAMPLE		10
#define S5K4H8_LINK_FREQ_MHZ		280000000
/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define S5K4H8_PIXEL_RATE		224000000
#define S5K4H8_XVCLK_FREQ		24000000

#define CHIP_ID				0x4088
#define S5K4H8_REG_CHIP_ID_H		0x0000
#define S5K4H8_REG_CHIP_ID_L		0x0001

#define S5K4H8_REG_CTRL_MODE		0x0100
#define S5K4H8_MODE_SW_STANDBY		0x00
#define S5K4H8_MODE_STREAMING		0x01

#define S5K4H8_REG_EXPOSURE		0x0202

#define	S5K4H8_EXPOSURE_MIN		4
#define	S5K4H8_EXPOSURE_STEP		1
#define S5K4H8_VTS_MAX			0x1fff

#define S5K4H8_REG_AGAIN_H		0x0204
#define S5K4H8_REG_AGAIN_L		0x0205
#define S5K4H8_GAIN_MIN			32
#define S5K4H8_GAIN_MAX			1024
#define S5K4H8_GAIN_STEP		1
#define S5K4H8_GAIN_DEFAULT		32

#define S5K4H8_REG_VTS			0x0340

#define REG_NULL			0xFFFF
#define REG_DELAY			0xFFFE

#define S5K4H8_REG_VALUE_08BIT		1
#define S5K4H8_REG_VALUE_16BIT		2
#define S5K4H8_REG_VALUE_24BIT		3

#define GAIN_DEFAULT			0x0100

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define S5K4H8_NAME			"s5k4h8"

static const char * const s5k4h8_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define S5K4H8_NUM_SUPPLIES ARRAY_SIZE(s5k4h8_supply_names)

struct s5k4h8_otp_info {
	int flag; //bit[7]: info & awb
	u32 module_id;
	u32 year;
	u32 month;
	u32 day;
	u32 rg_ratio;
	u32 bg_ratio;
};

struct s5k4h8_id_name {
	u32 id;
	char name[RKMODULE_NAME_LEN];
};

struct regval {
	u16 addr;
	u16 val;
};

struct s5k4h8_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct s5k4h8 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[S5K4H8_NUM_SUPPLIES];
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
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct s5k4h8_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	struct s5k4h8_otp_info *otp;
	struct rkmodule_inf	module_inf;
	struct rkmodule_awb_cfg	awb_cfg;
};

#define to_s5k4h8(sd) container_of(sd, struct s5k4h8, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval s5k4h8_global_regs[] = {
	{0x6028, 0x2000},
	{0x602A, 0x1FD0},
	{0x6F12, 0x0448},
	{0x6F12, 0x0349},
	{0x6F12, 0x0160},
	{0x6F12, 0xC26A},
	{0x6F12, 0x511A},
	{0x6F12, 0x8180},
	{0x6F12, 0x00F0},
	{0x6F12, 0x60B8},
	{0x6F12, 0x2000},
	{0x6F12, 0x20E8},
	{0x6F12, 0x2000},
	{0x6F12, 0x13A0},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x38B5},
	{0x6F12, 0x0021},
	{0x6F12, 0x0446},
	{0x6F12, 0x8DF8},
	{0x6F12, 0x0010},
	{0x6F12, 0x00F5},
	{0x6F12, 0xB470},
	{0x6F12, 0x0122},
	{0x6F12, 0x6946},
	{0x6F12, 0x00F0},
	{0x6F12, 0x59F8},
	{0x6F12, 0x9DF8},
	{0x6F12, 0x0000},
	{0x6F12, 0xFF28},
	{0x6F12, 0x05D0},
	{0x6F12, 0x0020},
	{0x6F12, 0x08B1},
	{0x6F12, 0x04F2},
	{0x6F12, 0x6914},
	{0x6F12, 0x2046},
	{0x6F12, 0x38BD},
	{0x6F12, 0x0120},
	{0x6F12, 0xF8E7},
	{0x6F12, 0x10B5},
	{0x6F12, 0x92B0},
	{0x6F12, 0x0C46},
	{0x6F12, 0x4822},
	{0x6F12, 0x6946},
	{0x6F12, 0x00F0},
	{0x6F12, 0x46F8},
	{0x6F12, 0x0020},
	{0x6F12, 0x6946},
	{0x6F12, 0x04EB},
	{0x6F12, 0x4003},
	{0x6F12, 0x0A5C},
	{0x6F12, 0x02F0},
	{0x6F12, 0x0F02},
	{0x6F12, 0x04F8},
	{0x6F12, 0x1020},
	{0x6F12, 0x0A5C},
	{0x6F12, 0x401C},
	{0x6F12, 0x1209},
	{0x6F12, 0x5A70},
	{0x6F12, 0x4828},
	{0x6F12, 0xF2D3},
	{0x6F12, 0x12B0},
	{0x6F12, 0x10BD},
	{0x6F12, 0x2DE9},
	{0x6F12, 0xF041},
	{0x6F12, 0x164E},
	{0x6F12, 0x0F46},
	{0x6F12, 0x06F1},
	{0x6F12, 0x1105},
	{0x6F12, 0xA236},
	{0x6F12, 0xB0B1},
	{0x6F12, 0x1449},
	{0x6F12, 0x1248},
	{0x6F12, 0x0968},
	{0x6F12, 0x0078},
	{0x6F12, 0xB1F8},
	{0x6F12, 0x6A10},
	{0x6F12, 0xC007},
	{0x6F12, 0x0ED0},
	{0x6F12, 0x0846},
	{0x6F12, 0xFFF7},
	{0x6F12, 0xBEFF},
	{0x6F12, 0x84B2},
	{0x6F12, 0x2946},
	{0x6F12, 0x2046},
	{0x6F12, 0xFFF7},
	{0x6F12, 0xD0FF},
	{0x6F12, 0x4FF4},
	{0x6F12, 0x9072},
	{0x6F12, 0x3146},
	{0x6F12, 0x04F1},
	{0x6F12, 0x4800},
	{0x6F12, 0x00F0},
	{0x6F12, 0x16F8},
	{0x6F12, 0x002F},
	{0x6F12, 0x05D0},
	{0x6F12, 0x3146},
	{0x6F12, 0x2846},
	{0x6F12, 0xBDE8},
	{0x6F12, 0xF041},
	{0x6F12, 0x00F0},
	{0x6F12, 0x13B8},
	{0x6F12, 0xBDE8},
	{0x6F12, 0xF081},
	{0x6F12, 0x0022},
	{0x6F12, 0xAFF2},
	{0x6F12, 0x5501},
	{0x6F12, 0x0348},
	{0x6F12, 0x00F0},
	{0x6F12, 0x10B8},
	{0x6F12, 0x2000},
	{0x6F12, 0x0C40},
	{0x6F12, 0x2000},
	{0x6F12, 0x0560},
	{0x6F12, 0x0000},
	{0x6F12, 0x152D},
	{0x6F12, 0x48F6},
	{0x6F12, 0x296C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x41F2},
	{0x6F12, 0x950C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x49F2},
	{0x6F12, 0x514C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x4088},
	{0x6F12, 0x0166},
	{0x6F12, 0x0000},
	{0x6F12, 0x0002},
	{0x5360, 0x0004},
	{0x3078, 0x0059},
	{0x307C, 0x0025},
	{0x36D0, 0x00DD},
	{0x36D2, 0x0100},
	{0x306A, 0x00EF},
	{0x6028, 0x4000},
	{0x602A, 0x6214},
	{0x6F12, 0x7971},
	{0x602A, 0x6218},
	{0x6F12, 0x7150},
	{0x6028, 0x2000},
	{0x602A, 0x0EC6},
	{0x6F12, 0x0000},
	{0xFCFC, 0x4000},
	{0xF490, 0x0030},
	{0xF47A, 0x0012},
	{0xF428, 0x0200},
	{0xF48E, 0x0010},
	{0xF45C, 0x0004},
	{0x0B04, 0x0101},
	{0x0B00, 0x0080},
	{0x6028, 0x2000},
	{0x602A, 0x0C40},
	{0x6F12, 0x0140},
	{0xFCFC, 0x4000},
	{0x0202, 0x0465},
	{0x31AA, 0x0004},
	{0x1006, 0x0006},
	{0x31FA, 0x0000},
	{0x020E, 0x0100},
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 25fps
 * mipi_datarate per lane 560Mbps
 */
static const struct regval s5k4h8_3264x2448_regs[] = {
	{0x0200, 0x0618},
	{0x0344, 0x0008},
	{0x0348, 0x0CC7},
	{0x0346, 0x0008},
	{0x034A, 0x0997},
	{0x034C, 0x0CC0},
	{0x034E, 0x0990},
	{0x0342, 0x0EA0},
	{0x0340, 0x09BC},
	{0x0900, 0x0111},
	{0x0380, 0x0001},
	{0x0382, 0x0001},
	{0x0384, 0x0001},
	{0x0386, 0x0001},
	{0x0400, 0x0002},
	{0x0404, 0x0010},
	{0x0114, 0x0330},
	{0x0136, 0x1800},
	{0x0300, 0x0005},
	{0x0302, 0x0001},
	{0x0304, 0x0006},
	{0x0306, 0x008C},
	{0x030C, 0x0006},
	{0x030E, 0x008C},
	{0x3008, 0x0000},
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 560Mbps
 */
static const struct regval s5k4h8_1632x1224_regs[] = {
	{0x0200, 0x0800},
	{0x0344, 0x0008},
	{0x0348, 0x0CC7},
	{0x0346, 0x0008},
	{0x034A, 0x0997},
	{0x034C, 0x0660},
	{0x034E, 0x04C8},
	{0x0342, 0x0EA0},
	{0x0340, 0x04E0},
	{0x0900, 0x0212},
	{0x0380, 0x0001},
	{0x0382, 0x0001},
	{0x0384, 0x0001},
	{0x0386, 0x0003},
	{0x0400, 0x0002},
	{0x0404, 0x0020},
	{0x0114, 0x0330},
	{0x0136, 0x1800},
	{0x0300, 0x0005},
	{0x0302, 0x0002},
	{0x0304, 0x0004},
	{0x0306, 0x0075},
	{0x030C, 0x0006},
	{0x030E, 0x008C},
	{0x3008, 0x0000},
	{REG_NULL, 0x00},
};

static const struct s5k4h8_mode supported_modes[] = {
	{
		.width = 1632,
		.height = 1224,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x04b0,
		.hts_def = 0x0ea0,
		.vts_def = 0x04E0,
		.reg_list = s5k4h8_1632x1224_regs,
	},
	{
		.width = 3264,
		.height = 2448,
		.max_fps = {
			.numerator = 10000,
			.denominator = 250000,
		},
		.exp_def = 0x09B0,
		.hts_def = 0x0ea0,
		.vts_def = 0x09BC,
		.reg_list = s5k4h8_3264x2448_regs,
	},
};

static const s64 link_freq_menu_items[] = {
	S5K4H8_LINK_FREQ_MHZ
};

/* Write registers up to 4 at a time */
static int s5k4h8_write_reg(struct i2c_client *client, u16 reg,
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

static int s5k4h8_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		if (unlikely(regs[i].addr == REG_DELAY))
			usleep_range(regs[i].val, regs[i].val * 2);
		else
			ret = s5k4h8_write_reg(client, regs[i].addr,
					       S5K4H8_REG_VALUE_16BIT,
					       regs[i].val);
	}

	return ret;
}

/* Read registers up to 4 at a time */
static int s5k4h8_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
			   u32 *val)
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

static int s5k4h8_get_reso_dist(const struct s5k4h8_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
		abs(mode->height - framefmt->height);
}

static const struct s5k4h8_mode *
s5k4h8_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = s5k4h8_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int s5k4h8_set_fmt(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *fmt)
{
	struct s5k4h8 *s5k4h8 = to_s5k4h8(sd);
	const struct s5k4h8_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&s5k4h8->mutex);

	mode = s5k4h8_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&s5k4h8->mutex);
		return -ENOTTY;
#endif
	} else {
		s5k4h8->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(s5k4h8->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(s5k4h8->vblank, vblank_def,
					 S5K4H8_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&s5k4h8->mutex);

	return 0;
}

static int s5k4h8_get_fmt(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *fmt)
{
	struct s5k4h8 *s5k4h8 = to_s5k4h8(sd);
	const struct s5k4h8_mode *mode = s5k4h8->cur_mode;

	mutex_lock(&s5k4h8->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&s5k4h8->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&s5k4h8->mutex);

	return 0;
}

static int s5k4h8_enum_mbus_code(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;

	return 0;
}

static int s5k4h8_enum_frame_sizes(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int s5k4h8_g_frame_interval(struct v4l2_subdev *sd,
	struct v4l2_subdev_frame_interval *fi)
{
	struct s5k4h8 *s5k4h8 = to_s5k4h8(sd);
	const struct s5k4h8_mode *mode = s5k4h8->cur_mode;

	mutex_lock(&s5k4h8->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&s5k4h8->mutex);

	return 0;
}

static int s5k4h8_otp_read(struct s5k4h8 *s5k4h8)
{
	u32 otp_flag = 0;
	U32 temp = 0;
	struct s5k4h8_otp_info *otp_ptr;
	struct device *dev = &s5k4h8->client->dev;
	struct i2c_client *client = s5k4h8->client;

	otp_ptr = devm_kzalloc(dev, sizeof(*otp_ptr), GFP_KERNEL);
	if (!otp_ptr)
		return -ENOMEM;
	//stream on
	s5k4h8_write_reg(client, S5K4H8_REG_CTRL_MODE,
			S5K4H8_REG_VALUE_08BIT, S5K4H8_MODE_STREAMING);
	// set the page15 of OTP
	s5k4h8_write_reg(client, 0x0A02,
			S5K4H8_REG_VALUE_08BIT, 0x0f);
	//OTP enable and read start
	s5k4h8_write_reg(client, 0x0A00,
			S5K4H8_REG_VALUE_16BIT, 0x0100);

	s5k4h8_read_reg(client, 0x0A04, S5K4H8_REG_VALUE_08BIT,
			&otp_flag);
	if (otp_flag == 0x10) {
		otp_ptr->flag = 0x80;
		s5k4h8_read_reg(client, 0x0A05, S5K4H8_REG_VALUE_08BIT, &temp);
		otp_ptr->module_id = temp;
		s5k4h8_read_reg(client, 0x0A06, S5K4H8_REG_VALUE_08BIT, &temp);
		otp_ptr->year = temp;
		s5k4h8_read_reg(client, 0x0A07, S5K4H8_REG_VALUE_08BIT, &temp);
		otp_ptr->month = temp;
		s5k4h8_read_reg(client, 0x0A08, S5K4H8_REG_VALUE_08BIT, &temp);
		otp_ptr->day = temp;
		s5k4h8_read_reg(client, 0x0A09, S5K4H8_REG_VALUE_08BIT, &temp);
		otp_ptr->rg_ratio = (temp & 0x3) << 8;
		s5k4h8_read_reg(client, 0x0A0A, S5K4H8_REG_VALUE_08BIT, &temp);
		otp_ptr->rg_ratio |= temp & 0xff;
		s5k4h8_read_reg(client, 0x0A0B, S5K4H8_REG_VALUE_08BIT, &temp);
		otp_ptr->bg_ratio = (temp & 0x3) << 8;
		s5k4h8_read_reg(client, 0x0A0C, S5K4H8_REG_VALUE_08BIT, &temp);
		otp_ptr->bg_ratio |= temp & 0xff;
	} else if (otp_flag == 0xf1) {
		otp_ptr->flag = 0x80;
		s5k4h8_read_reg(client, 0x0A25, S5K4H8_REG_VALUE_08BIT, &temp);
		otp_ptr->module_id = temp;
		s5k4h8_read_reg(client, 0x0A26, S5K4H8_REG_VALUE_08BIT, &temp);
		otp_ptr->year = temp;
		s5k4h8_read_reg(client, 0x0A27, S5K4H8_REG_VALUE_08BIT, &temp);
		otp_ptr->month = temp;
		s5k4h8_read_reg(client, 0x0A28, S5K4H8_REG_VALUE_08BIT, &temp);
		otp_ptr->day = temp;
		s5k4h8_read_reg(client, 0x0A29, S5K4H8_REG_VALUE_08BIT, &temp);
		otp_ptr->rg_ratio = (temp & 0x3) << 8;
		s5k4h8_read_reg(client, 0x0A2A, S5K4H8_REG_VALUE_08BIT, &temp);
		otp_ptr->rg_ratio |= temp & 0xff;
		s5k4h8_read_reg(client, 0x0A2B, S5K4H8_REG_VALUE_08BIT, &temp);
		otp_ptr->bg_ratio = (temp & 0x3) << 8;
		s5k4h8_read_reg(client, 0x0A2C, S5K4H8_REG_VALUE_08BIT, &temp);
		otp_ptr->bg_ratio |= temp & 0xff;
	} else {
		otp_ptr->flag = 0;
		dev_err(dev, "otp read fail!\n");
	}
	s5k4h8_write_reg(client, 0x0A00,
			S5K4H8_REG_VALUE_16BIT, 0x0000);
	s5k4h8_write_reg(client, S5K4H8_REG_CTRL_MODE,
			S5K4H8_REG_VALUE_08BIT, S5K4H8_MODE_SW_STANDBY);
	if (otp_ptr->flag) {
		dev_err(dev, "%s id=0x%x, year=0x%x, month=0x%x, day=0x%x\n",
			__func__,
			otp_ptr->module_id,
			otp_ptr->year,
			otp_ptr->month,
			otp_ptr->day);
		dev_err(dev, "%s rg_ratio=0x%x, bg_ratio=0x%x\n",
			__func__,
			otp_ptr->rg_ratio,
			otp_ptr->bg_ratio);
		s5k4h8->otp = otp_ptr;
	} else {
		s5k4h8->otp = NULL;
		devm_kfree(dev, otp_ptr);
	}

	return 0;
}

static void s5k4h8_get_otp(struct s5k4h8_otp_info *otp,
	struct rkmodule_inf *inf)
{
	/* info & awb */
	if (otp->flag & 0x80) {
		inf->fac.flag = 1;
		inf->fac.year = otp->year;
		inf->fac.month = otp->month;
		inf->fac.day = otp->day;

		inf->awb.flag = 1;
		inf->awb.r_value = otp->rg_ratio;
		inf->awb.b_value = otp->bg_ratio;
		inf->awb.gr_value = 0;
		inf->awb.gb_value = 0;

		inf->awb.golden_r_value = 0;
		inf->awb.golden_b_value = 0;
		inf->awb.golden_gr_value = 0;
		inf->awb.golden_gb_value = 0;
	}
}

static void s5k4h8_get_module_inf(struct s5k4h8 *s5k4h8,
				struct rkmodule_inf *inf)
{
	struct s5k4h8_otp_info *otp = s5k4h8->otp;

	strlcpy(inf->base.sensor,
		S5K4H8_NAME,
		sizeof(inf->base.sensor));
	strlcpy(inf->base.module,
		s5k4h8->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens,
		s5k4h8->len_name,
		sizeof(inf->base.lens));
	if (otp)
		s5k4h8_get_otp(otp, inf);
}

static void s5k4h8_set_module_inf(struct s5k4h8 *s5k4h8,
				struct rkmodule_awb_cfg *cfg)
{
	mutex_lock(&s5k4h8->mutex);
	memcpy(&s5k4h8->awb_cfg, cfg, sizeof(*cfg));
	mutex_unlock(&s5k4h8->mutex);
}

static long s5k4h8_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct s5k4h8 *s5k4h8 = to_s5k4h8(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		s5k4h8_get_module_inf(s5k4h8, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_AWB_CFG:
		s5k4h8_set_module_inf(s5k4h8, (struct rkmodule_awb_cfg *)arg);
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long s5k4h8_compat_ioctl32(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = s5k4h8_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = s5k4h8_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}
#endif

/*--------------------------------------------------------------------------*/
static int s5k4h8_apply_otp(struct s5k4h8 *s5k4h8)
{
	int R_gain, G_gain, B_gain, GR_gain, GB_gain;
	int r_ratio, b_ratio;
	struct i2c_client *client = s5k4h8->client;
	struct s5k4h8_otp_info *otp_ptr = s5k4h8->otp;
	struct rkmodule_awb_cfg *awb_cfg = &s5k4h8->awb_cfg;
	u32 golden_bg_ratio;
	u32 golden_rg_ratio;
	u32 golden_g_value;

	if (!s5k4h8->awb_cfg.enable)
		return 0;

	golden_g_value = (awb_cfg->golden_gb_value +
		 awb_cfg->golden_gr_value) / 2;
	golden_bg_ratio = awb_cfg->golden_b_value * 512 / golden_g_value;
	golden_rg_ratio = awb_cfg->golden_r_value * 512 / golden_g_value;
	/* apply OTP WB Calibration */
	if ((otp_ptr->flag & 0x80) && golden_bg_ratio && golden_rg_ratio) {
		/* calculate G gain */
		r_ratio = golden_rg_ratio * 512 / otp_ptr->rg_ratio;
		b_ratio = golden_bg_ratio * 512 / otp_ptr->bg_ratio;
		if (r_ratio >= 512) {
			if (b_ratio >= 512) {
				R_gain = (u16)(GAIN_DEFAULT * r_ratio / 512);
				G_gain = GAIN_DEFAULT;
				B_gain = (u16)(GAIN_DEFAULT * b_ratio / 512);
			} else {
				R_gain = (u16)(GAIN_DEFAULT * r_ratio / b_ratio);
				G_gain = (u16)(GAIN_DEFAULT * 512 / b_ratio);
				B_gain = GAIN_DEFAULT;
			}
		} else {
			if (b_ratio >= 512) {
				R_gain = GAIN_DEFAULT;
				G_gain = (u16)(GAIN_DEFAULT * 512 / r_ratio);
				B_gain = (u16)(GAIN_DEFAULT *  b_ratio / r_ratio);
			} else {
				GR_gain = (u16)(GAIN_DEFAULT * 512 / r_ratio);
				GB_gain = (u16)(GAIN_DEFAULT * 512 / b_ratio);
				if (GR_gain >= GB_gain) {
					R_gain = GAIN_DEFAULT;
					G_gain = (u16)(GAIN_DEFAULT * 512 / r_ratio);
					B_gain = (u16)(GAIN_DEFAULT * b_ratio / r_ratio);
				} else {
					R_gain = (u16)(GAIN_DEFAULT * r_ratio / b_ratio);
					G_gain = (u16)(GAIN_DEFAULT * 512 / b_ratio);
					B_gain = GAIN_DEFAULT;
				}
			}
		}

		/* update sensor WB gain */
		s5k4h8_write_reg(client, 0x6028,
			S5K4H8_REG_VALUE_16BIT, 0x4000);
		s5k4h8_write_reg(client, 0x602a,
			S5K4H8_REG_VALUE_16BIT, 0x3058);
		s5k4h8_write_reg(client, 0x6f12,
			S5K4H8_REG_VALUE_08BIT, 0x01);

		s5k4h8_write_reg(client, 0x020e,
			S5K4H8_REG_VALUE_16BIT, G_gain);
		s5k4h8_write_reg(client, 0x0210,
			S5K4H8_REG_VALUE_16BIT, R_gain);
		s5k4h8_write_reg(client, 0x0212,
			S5K4H8_REG_VALUE_16BIT, B_gain);
		s5k4h8_write_reg(client, 0x0214,
			S5K4H8_REG_VALUE_16BIT, G_gain);
		dev_dbg(&client->dev, "apply awb gain: 0x%x, 0x%x, 0x%x\n",
			R_gain, G_gain, B_gain);
	}
	return 0;
}

static int __s5k4h8_start_stream(struct s5k4h8 *s5k4h8)
{
	int ret;

	ret = s5k4h8_write_array(s5k4h8->client, s5k4h8->cur_mode->reg_list);
	if (ret)
		return ret;

	if (s5k4h8->otp) {
		ret = s5k4h8_apply_otp(s5k4h8);
		if (ret)
			return ret;
	}

	/* In case these controls are set before streaming */
	mutex_unlock(&s5k4h8->mutex);
	ret = v4l2_ctrl_handler_setup(&s5k4h8->ctrl_handler);
	mutex_lock(&s5k4h8->mutex);
	return s5k4h8_write_reg(s5k4h8->client, S5K4H8_REG_CTRL_MODE,
			S5K4H8_REG_VALUE_08BIT, S5K4H8_MODE_STREAMING);
}

static int __s5k4h8_stop_stream(struct s5k4h8 *s5k4h8)
{
	return s5k4h8_write_reg(s5k4h8->client, S5K4H8_REG_CTRL_MODE,
			S5K4H8_REG_VALUE_08BIT, S5K4H8_MODE_SW_STANDBY);
}

static int s5k4h8_s_stream(struct v4l2_subdev *sd, int on)
{
	struct s5k4h8 *s5k4h8 = to_s5k4h8(sd);
	struct i2c_client *client = s5k4h8->client;
	int ret = 0;

	mutex_lock(&s5k4h8->mutex);
	on = !!on;
	if (on == s5k4h8->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __s5k4h8_start_stream(s5k4h8);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__s5k4h8_stop_stream(s5k4h8);
		pm_runtime_put(&client->dev);
	}

	s5k4h8->streaming = on;

unlock_and_return:
	mutex_unlock(&s5k4h8->mutex);

	return ret;
}

static int s5k4h8_s_power(struct v4l2_subdev *sd, int on)
{
	struct s5k4h8 *s5k4h8 = to_s5k4h8(sd);
	struct i2c_client *client = s5k4h8->client;
	int ret = 0;

	mutex_lock(&s5k4h8->mutex);

	/* If the power state is not modified - no work to do. */
	if (s5k4h8->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = s5k4h8_write_array(s5k4h8->client, s5k4h8_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		s5k4h8->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		s5k4h8->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&s5k4h8->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 s5k4h8_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, S5K4H8_XVCLK_FREQ / 1000 / 1000);
}

static int __s5k4h8_power_on(struct s5k4h8 *s5k4h8)
{
	int ret;
	u32 delay_us;
	struct device *dev = &s5k4h8->client->dev;

	if (!IS_ERR_OR_NULL(s5k4h8->pins_default)) {
		ret = pinctrl_select_state(s5k4h8->pinctrl,
					   s5k4h8->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(s5k4h8->xvclk, S5K4H8_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(s5k4h8->xvclk) != S5K4H8_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(s5k4h8->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(s5k4h8->reset_gpio))
		gpiod_set_value_cansleep(s5k4h8->reset_gpio, 1);

	ret = regulator_bulk_enable(S5K4H8_NUM_SUPPLIES, s5k4h8->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	usleep_range(1000, 1100);
	if (!IS_ERR(s5k4h8->reset_gpio))
		gpiod_set_value_cansleep(s5k4h8->reset_gpio, 0);

	usleep_range(500, 1000);
	if (!IS_ERR(s5k4h8->pwdn_gpio))
		gpiod_set_value_cansleep(s5k4h8->pwdn_gpio, 0);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = s5k4h8_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(s5k4h8->xvclk);

	return ret;
}

static void __s5k4h8_power_off(struct s5k4h8 *s5k4h8)
{
	int ret;

	if (!IS_ERR(s5k4h8->pwdn_gpio))
		gpiod_set_value_cansleep(s5k4h8->pwdn_gpio, 1);
	clk_disable_unprepare(s5k4h8->xvclk);
	if (!IS_ERR(s5k4h8->reset_gpio))
		gpiod_set_value_cansleep(s5k4h8->reset_gpio, 1);
	if (!IS_ERR_OR_NULL(s5k4h8->pins_sleep)) {
		ret = pinctrl_select_state(s5k4h8->pinctrl,
					   s5k4h8->pins_sleep);
		if (ret < 0)
			dev_dbg(&s5k4h8->client->dev, "could not set pins\n");
	}
	regulator_bulk_disable(S5K4H8_NUM_SUPPLIES, s5k4h8->supplies);
}

static int s5k4h8_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4h8 *s5k4h8 = to_s5k4h8(sd);

	return __s5k4h8_power_on(s5k4h8);
}

static int s5k4h8_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4h8 *s5k4h8 = to_s5k4h8(sd);

	__s5k4h8_power_off(s5k4h8);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int s5k4h8_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct s5k4h8 *s5k4h8 = to_s5k4h8(sd);
	struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct s5k4h8_mode *def_mode = &supported_modes[0];

	mutex_lock(&s5k4h8->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&s5k4h8->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int s5k4h8_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fie->code != MEDIA_BUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static const struct dev_pm_ops s5k4h8_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k4h8_runtime_suspend,
			s5k4h8_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops s5k4h8_internal_ops = {
	.open = s5k4h8_open,
};
#endif

static const struct v4l2_subdev_core_ops s5k4h8_core_ops = {
	.s_power = s5k4h8_s_power,
	.ioctl = s5k4h8_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = s5k4h8_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops s5k4h8_video_ops = {
	.s_stream = s5k4h8_s_stream,
	.g_frame_interval = s5k4h8_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops s5k4h8_pad_ops = {
	.enum_mbus_code = s5k4h8_enum_mbus_code,
	.enum_frame_size = s5k4h8_enum_frame_sizes,
	.enum_frame_interval = s5k4h8_enum_frame_interval,
	.get_fmt = s5k4h8_get_fmt,
	.set_fmt = s5k4h8_set_fmt,
};

static const struct v4l2_subdev_ops s5k4h8_subdev_ops = {
	.core	= &s5k4h8_core_ops,
	.video	= &s5k4h8_video_ops,
	.pad	= &s5k4h8_pad_ops,
};

static int s5k4h8_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k4h8 *s5k4h8 = container_of(ctrl->handler,
					struct s5k4h8, ctrl_handler);
	struct i2c_client *client = s5k4h8->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = s5k4h8->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(s5k4h8->exposure,
					 s5k4h8->exposure->minimum, max,
					 s5k4h8->exposure->step,
					 s5k4h8->exposure->default_value);
		break;
	}

	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = s5k4h8_write_reg(s5k4h8->client,
					S5K4H8_REG_EXPOSURE,
					S5K4H8_REG_VALUE_16BIT,
					ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = s5k4h8_write_reg(s5k4h8->client,
					S5K4H8_REG_AGAIN_H,
					S5K4H8_REG_VALUE_16BIT,
					ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = s5k4h8_write_reg(s5k4h8->client,
					S5K4H8_REG_VTS,
					S5K4H8_REG_VALUE_16BIT,
					ctrl->val + s5k4h8->cur_mode->height);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k4h8_ctrl_ops = {
	.s_ctrl = s5k4h8_set_ctrl,
};

static int s5k4h8_initialize_controls(struct s5k4h8 *s5k4h8)
{
	const struct s5k4h8_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &s5k4h8->ctrl_handler;
	mode = s5k4h8->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &s5k4h8->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			0, S5K4H8_PIXEL_RATE, 1, S5K4H8_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	s5k4h8->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (s5k4h8->hblank)
		s5k4h8->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	s5k4h8->vblank = v4l2_ctrl_new_std(handler, &s5k4h8_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				S5K4H8_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	s5k4h8->exposure = v4l2_ctrl_new_std(handler, &s5k4h8_ctrl_ops,
				V4L2_CID_EXPOSURE, S5K4H8_EXPOSURE_MIN,
				exposure_max, S5K4H8_EXPOSURE_STEP,
				mode->exp_def);

	s5k4h8->anal_gain = v4l2_ctrl_new_std(handler, &s5k4h8_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, S5K4H8_GAIN_MIN,
				S5K4H8_GAIN_MAX, S5K4H8_GAIN_STEP,
				S5K4H8_GAIN_DEFAULT);
	if (handler->error) {
		ret = handler->error;
		dev_err(&s5k4h8->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	s5k4h8->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int s5k4h8_check_sensor_id(struct s5k4h8 *s5k4h8,
				struct i2c_client *client)
{
	struct device *dev = &s5k4h8->client->dev;
	u16 id = 0;
	u32 reg_H = 0;
	u32 reg_L = 0;
	int ret;

	ret = s5k4h8_read_reg(client,
		S5K4H8_REG_CHIP_ID_H,
		S5K4H8_REG_VALUE_08BIT,
		&reg_H);
	ret |= s5k4h8_read_reg(client,
		S5K4H8_REG_CHIP_ID_L,
		S5K4H8_REG_VALUE_08BIT,
		&reg_L);
	id = ((reg_H << 8) & 0xff00) | (reg_L & 0xff);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	return ret;
}

static int s5k4h8_configure_regulators(struct s5k4h8 *s5k4h8)
{
	unsigned int i;

	for (i = 0; i < S5K4H8_NUM_SUPPLIES; i++)
		s5k4h8->supplies[i].supply = s5k4h8_supply_names[i];

	return devm_regulator_bulk_get(&s5k4h8->client->dev,
		S5K4H8_NUM_SUPPLIES,
		s5k4h8->supplies);
}

static int s5k4h8_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct s5k4h8 *s5k4h8;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	s5k4h8 = devm_kzalloc(dev, sizeof(*s5k4h8), GFP_KERNEL);
	if (!s5k4h8)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
		&s5k4h8->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
		&s5k4h8->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
		&s5k4h8->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
		&s5k4h8->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}
	s5k4h8->client = client;
	s5k4h8->cur_mode = &supported_modes[0];

	s5k4h8->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(s5k4h8->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	s5k4h8->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(s5k4h8->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	s5k4h8->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(s5k4h8->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = s5k4h8_configure_regulators(s5k4h8);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	s5k4h8->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(s5k4h8->pinctrl)) {
		s5k4h8->pins_default =
			pinctrl_lookup_state(s5k4h8->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(s5k4h8->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		s5k4h8->pins_sleep =
			pinctrl_lookup_state(s5k4h8->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(s5k4h8->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	mutex_init(&s5k4h8->mutex);

	sd = &s5k4h8->subdev;
	v4l2_i2c_subdev_init(sd, client, &s5k4h8_subdev_ops);
	ret = s5k4h8_initialize_controls(s5k4h8);
	if (ret)
		goto err_destroy_mutex;

	ret = __s5k4h8_power_on(s5k4h8);
	if (ret)
		goto err_free_handler;

	ret = s5k4h8_check_sensor_id(s5k4h8, client);
	if (ret)
		goto err_power_off;

	s5k4h8_otp_read(s5k4h8);

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &s5k4h8_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	s5k4h8->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &s5k4h8->pad, 0);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(s5k4h8->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 s5k4h8->module_index, facing,
		 S5K4H8_NAME, dev_name(sd->dev));
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
	__s5k4h8_power_off(s5k4h8);
err_free_handler:
	v4l2_ctrl_handler_free(&s5k4h8->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&s5k4h8->mutex);

	return ret;
}

static int s5k4h8_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4h8 *s5k4h8 = to_s5k4h8(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&s5k4h8->ctrl_handler);
	mutex_destroy(&s5k4h8->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__s5k4h8_power_off(s5k4h8);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id s5k4h8_of_match[] = {
	{ .compatible = "samsung,s5k4h8" },
	{},
};
MODULE_DEVICE_TABLE(of, s5k4h8_of_match);
#endif

static const struct i2c_device_id s5k4h8_match_id[] = {
	{ "samsung,s5k4h8", 0},
	{ },
};

static struct i2c_driver s5k4h8_i2c_driver = {
	.driver = {
		.name = S5K4H8_NAME,
		.pm = &s5k4h8_pm_ops,
		.of_match_table = of_match_ptr(s5k4h8_of_match),
	},
	.probe		= &s5k4h8_probe,
	.remove		= &s5k4h8_remove,
	.id_table	= s5k4h8_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&s5k4h8_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&s5k4h8_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Samsung s5k4h8 sensor driver");
MODULE_LICENSE("GPL v2");
