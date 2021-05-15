// SPDX-License-Identifier: GPL-2.0
/*
 * sc2335 driver
 *
 * Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version.
 * V0.0X01.0X01 add quick stream support.
 * V0.0X01.0X02 fix gain error.
 * V0.0X01.0X03 fix set flip/mirror error.
 */
//#define DEBUG
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/of_graph.h>
#include <linux/rk-camera-module.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <linux/pinctrl/consumer.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x03)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define SC2335_BITS_PER_SAMPLE		10
#define SC2335_ONE_LANE			1
#define SC2335_TWO_LANES			2
#define SC2335_LINK_FREQ_186M		185625000   // 371.25Mbps
#define SC2335_LINK_FREQ_371M		371250000	// 742.5Mbps

#define SC2335_PIXEL_RATE_ONE_LANE		(SC2335_LINK_FREQ_186M * 2 * \
					SC2335_ONE_LANE / SC2335_BITS_PER_SAMPLE)
#define SC2335_PIXEL_RATE_TWO_LANES		(SC2335_LINK_FREQ_371M * 2 * \
					SC2335_TWO_LANES / SC2335_BITS_PER_SAMPLE)

#define SC2335_XVCLK_FREQ		24000000

#define CHIP_ID				0xcb14
#define SC2335_REG_CHIP_ID		0x3107

#define SC2335_REG_CTRL_MODE		0x0100
#define SC2335_MODE_SW_STANDBY		0x0
#define SC2335_MODE_STREAMING		BIT(0)

#define SC2335_REG_EXPOSURE		0x3e00
#define	SC2335_EXPOSURE_MIN		1
#define	SC2335_EXPOSURE_STEP	1
#define SC2335_REG_VTS			0x320e
#define SC2335_VTS_MAX			0xffff

#define SC2335_REG_COARSE_DGAIN		0x3e06
#define SC2335_REG_FINE_DGAIN		0x3e07
#define SC2335_REG_COARSE_AGAIN		0x3e08
#define SC2335_REG_FINE_AGAIN		0x3e09
#define	ANALOG_GAIN_MIN			0x40
#define	ANALOG_GAIN_MAX			0x8000
#define	ANALOG_GAIN_STEP		1
#define	ANALOG_GAIN_DEFAULT		0x40

#define SC2335_REG_TEST_PATTERN		0x4501
#define SC2335_TEST_PATTERN_BIT_MASK	BIT(3)

#define SC2335_REG_FLIP_MIRROR		0x3221
#define SC2335_MIRROR_MASK			0x06
#define SC2335_FLIP_MASK			0x60

#define REG_NULL			0xFFFF
#define DELAY_MS			0xEEEE	/* Array delay token */

#define SC2335_REG_VALUE_08BIT		1
#define SC2335_REG_VALUE_16BIT		2
#define SC2335_REG_VALUE_24BIT		3

#define PIX_FORMAT MEDIA_BUS_FMT_SBGGR10_1X10
#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define SC2335_NAME			"sc2335"

static const char * const sc2335_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SC2335_NUM_SUPPLIES ARRAY_SIZE(sc2335_supply_names)

struct regval {
	u16 addr;
	u8 val;
};
enum sc2335_max_pad {
	PAD0, /* link to isp */
	PAD1, /* link to csi wr0 | hdr x2:L x3:M */
	PAD2, /* link to csi wr1 | hdr      x3:L */
	PAD3, /* link to csi wr2 | hdr x2:M x3:S */
	PAD_MAX,
};

struct sc2335_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
	u8 freq_idx;
	u8 lanes;
	u8 bpp;
};

struct sc2335 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct gpio_desc	*power_gpio;
	struct regulator_bulk_data supplies[SC2335_NUM_SUPPLIES];
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;
	struct v4l2_ctrl	*test_pattern;
	struct v4l2_ctrl	*h_flip;
	struct v4l2_ctrl	*v_flip;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct sc2335_mode *cur_mode;
	u32			module_index;
	u32			cur_vts;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	struct v4l2_fwnode_endpoint bus_cfg;
};

#define to_sc2335(sd) container_of(sd, struct sc2335, subdev)

static const struct regval sc2335_global_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 742.5Mbps, 1 lane
 */
static const struct regval sc2335_1920x1080_regs_1lane[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x36f9, 0x80},
	{0x3018, 0x12},
	{0x301f, 0x35},
	{0x3207, 0x3f},
	{0x3249, 0x0f},
	{0x3253, 0x08},
	{0x3271, 0x00},
	{0x3273, 0x03},
	{0x3301, 0x06},
	{0x3302, 0x09},
	{0x3304, 0x28},
	{0x3306, 0x30},
	{0x330b, 0x94},
	{0x330c, 0x08},
	{0x330d, 0x18},
	{0x330e, 0x14},
	{0x330f, 0x05},
	{0x3310, 0x06},
	{0x3314, 0x96},
	{0x3316, 0x00},
	{0x331e, 0x21},
	{0x332b, 0x08},
	{0x3333, 0x10},
	{0x3338, 0x80},
	{0x333a, 0x04},
	{0x334c, 0x04},
	{0x335f, 0x04},
	{0x3364, 0x17},
	{0x3366, 0x62},
	{0x337c, 0x05},
	{0x337d, 0x09},
	{0x337e, 0x00},
	{0x3390, 0x08},
	{0x3391, 0x18},
	{0x3392, 0x38},
	{0x3393, 0x09},
	{0x3394, 0x20},
	{0x3395, 0x20},
	{0x33a2, 0x07},
	{0x33ac, 0x04},
	{0x33ae, 0x14},
	{0x3614, 0x00},
	{0x3622, 0x16},
	{0x3630, 0x68},
	{0x3631, 0x84},
	{0x3637, 0x20},
	{0x363a, 0x1f},
	{0x3670, 0x0e},
	{0x3674, 0xa1},
	{0x3675, 0x9c},
	{0x3676, 0x9e},
	{0x3677, 0x84},
	{0x3678, 0x85},
	{0x3679, 0x87},
	{0x367c, 0x18},
	{0x367d, 0x38},
	{0x367e, 0x08},
	{0x367f, 0x18},
	{0x3690, 0x32},
	{0x3691, 0x32},
	{0x3692, 0x44},
	{0x369c, 0x08},
	{0x369d, 0x38},
	{0x36ea, 0xf5},
	{0x36ec, 0x0c},
	{0x36fa, 0xdf},
	{0x3908, 0x82},
	{0x391f, 0x18},
	{0x3e01, 0x8c},
	{0x3e02, 0x00},
	{0x3f00, 0x0d},
	{0x3f04, 0x02},
	{0x3f05, 0x0e},
	{0x3f09, 0x48},
	{0x4505, 0x0a},
	{0x4509, 0x20},
	{0x4800, 0x44},
	{0x481d, 0x0a},
	{0x4827, 0x03},
	{0x5787, 0x10},
	{0x5788, 0x06},
	{0x578a, 0x10},
	{0x578b, 0x06},
	{0x5790, 0x10},
	{0x5791, 0x10},
	{0x5792, 0x00},
	{0x5793, 0x10},
	{0x5794, 0x10},
	{0x5795, 0x00},
	{0x5799, 0x00},
	{0x57c7, 0x10},
	{0x57c8, 0x06},
	{0x57ca, 0x10},
	{0x57cb, 0x06},
	{0x57d1, 0x10},
	{0x57d4, 0x10},
	{0x57d9, 0x00},
	{0x36e9, 0x59},
	{0x36f9, 0x5b},
	{0x0100, 0x01},
	{0x363c, 0x04},
	{DELAY_MS, 0x0a},
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 371.25Mbps, 2 lane
 */
static const struct regval sc2335_1920x1080_regs_2lane[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x36f9, 0x80},
	{0x301f, 0x83},
	{0x3207, 0x3f},
	{0x3249, 0x0f},
	{0x3253, 0x08},
	{0x3271, 0x00},
	{0x3273, 0x03},
	{0x3301, 0x06},
	{0x3302, 0x09},
	{0x3304, 0x28},
	{0x3306, 0x30},
	{0x330b, 0x98},
	{0x330c, 0x08},
	{0x330d, 0x18},
	{0x330e, 0x14},
	{0x330f, 0x05},
	{0x3310, 0x06},
	{0x3314, 0x96},
	{0x3316, 0x00},
	{0x331e, 0x21},
	{0x332b, 0x08},
	{0x3333, 0x10},
	{0x3338, 0x80},
	{0x333a, 0x04},
	{0x334c, 0x04},
	{0x335f, 0x04},
	{0x3364, 0x17},
	{0x3366, 0x62},
	{0x337c, 0x05},
	{0x337d, 0x09},
	{0x337e, 0x00},
	{0x3390, 0x08},
	{0x3391, 0x18},
	{0x3392, 0x38},
	{0x3393, 0x09},
	{0x3394, 0x20},
	{0x3395, 0x20},
	{0x33a2, 0x07},
	{0x33ac, 0x04},
	{0x33ae, 0x14},
	{0x3614, 0x00},
	{0x3622, 0x16},
	{0x3630, 0x68},
	{0x3631, 0x84},
	{0x3637, 0x25},
	{0x363a, 0x1f},
	{0x363c, 0x0e},
	{0x3670, 0x0e},
	{0x3674, 0xb1},
	{0x3675, 0x41},
	{0x3676, 0x9c},
	{0x3677, 0x84},
	{0x3678, 0x85},
	{0x3679, 0x87},
	{0x367c, 0x18},
	{0x367d, 0x38},
	{0x367e, 0x08},
	{0x367f, 0x18},
	{0x3690, 0x32},
	{0x3691, 0x32},
	{0x3692, 0x44},
	{0x369c, 0x08},
	{0x369d, 0x38},
	{0x3908, 0x82},
	{0x391f, 0x18},
	{0x3e01, 0x8c},
	{0x3e02, 0x00},
	{0x3f00, 0x0d},
	{0x3f04, 0x02},
	{0x3f05, 0x0e},
	{0x3f09, 0x48},
	{0x4505, 0x0a},
	{0x4509, 0x20},
	{0x481d, 0x0a},
	{0x4827, 0x03},
	{0x5787, 0x10},
	{0x5788, 0x06},
	{0x578a, 0x10},
	{0x578b, 0x06},
	{0x5790, 0x10},
	{0x5791, 0x10},
	{0x5792, 0x00},
	{0x5793, 0x10},
	{0x5794, 0x10},
	{0x5795, 0x00},
	{0x5799, 0x00},
	{0x57c7, 0x10},
	{0x57c8, 0x06},
	{0x57ca, 0x10},
	{0x57cb, 0x06},
	{0x57d1, 0x10},
	{0x57d4, 0x10},
	{0x57d9, 0x00},
	{0x59e0, 0x60},
	{0x59e1, 0x08},
	{0x59e2, 0x3f},
	{0x59e3, 0x18},
	{0x59e4, 0x18},
	{0x59e5, 0x3f},
	{0x59e6, 0x06},
	{0x59e7, 0x02},
	{0x59e8, 0x38},
	{0x59e9, 0x10},
	{0x59ea, 0x0c},
	{0x59eb, 0x10},
	{0x59ec, 0x04},
	{0x59ed, 0x02},
	{0x59ee, 0xa0},
	{0x59ef, 0x08},
	{0x59f4, 0x18},
	{0x59f5, 0x10},
	{0x59f6, 0x0c},
	{0x59f7, 0x10},
	{0x59f8, 0x06},
	{0x59f9, 0x02},
	{0x59fa, 0x18},
	{0x59fb, 0x10},
	{0x59fc, 0x0c},
	{0x59fd, 0x10},
	{0x59fe, 0x04},
	{0x59ff, 0x02},
	{0x36e9, 0x20},
	{0x36f9, 0x27},
	{0x0100, 0x01},
	{DELAY_MS, 0x0a},
	{REG_NULL, 0x00},
};

static const struct sc2335_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0400,
		.hts_def = 0x44C * 2,
		.vts_def = 0x0465,
		.hdr_mode = NO_HDR,
		.reg_list = sc2335_1920x1080_regs_2lane,
		.freq_idx = 1,
		.lanes = 2,
		.bpp = 10,
	},
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0400,
		.hts_def = 0x44C * 2,
		.vts_def = 0x0465,
		.hdr_mode = NO_HDR,
		.reg_list = sc2335_1920x1080_regs_1lane,
		.freq_idx = 0,
		.lanes = 1,
		.bpp = 10,
	}
};

static const char * const sc2335_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

static const s64 link_freq_menu_items[] = {
	SC2335_LINK_FREQ_186M,
	SC2335_LINK_FREQ_371M
};

/* Write registers up to 4 at a time */
static int sc2335_write_reg(struct i2c_client *client,
	u16 reg, u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;
	u32 ret;

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

	ret = i2c_master_send(client, buf, len + 2);
	if (ret != len + 2)
		return -EIO;

	return 0;
}

static int sc2335_write_array(struct i2c_client *client,
	const struct regval *regs)
{
	u32 i;
	int delay_ms = 0, ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		if (regs[i].addr == DELAY_MS) {
			delay_ms = regs[i].val;
			dev_info(&client->dev, "delay(%d) ms !\n", delay_ms);
			usleep_range(1000 * delay_ms, 1000 * delay_ms + 100);
			continue;
		}
		ret = sc2335_write_reg(client, regs[i].addr,
				       SC2335_REG_VALUE_08BIT, regs[i].val);
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int sc2335_read_reg(struct i2c_client *client,
	u16 reg, unsigned int len, u32 *val)
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

static int sc2335_get_reso_dist(const struct sc2335_mode *mode,
	struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct sc2335_mode *
sc2335_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = sc2335_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int sc2335_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc2335 *sc2335 = to_sc2335(sd);
	const struct sc2335_mode *mode;
	s64 h_blank, vblank_def;
	s32 dst_link_freq = 0;
	s64 dst_pixel_rate = 0;

	mutex_lock(&sc2335->mutex);

	mode = sc2335_find_best_fit(fmt);
	fmt->format.code = PIX_FORMAT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sc2335->mutex);
		return -ENOTTY;
#endif
	} else {
		sc2335->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc2335->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc2335->vblank, vblank_def,
					 SC2335_VTS_MAX - mode->height,
					 1, vblank_def);
		dst_link_freq = mode->freq_idx;
		dst_pixel_rate = (u32)link_freq_menu_items[mode->freq_idx] /
						mode->bpp * 2 * mode->lanes;
		__v4l2_ctrl_s_ctrl_int64(sc2335->pixel_rate,
					 dst_pixel_rate);
		__v4l2_ctrl_s_ctrl(sc2335->link_freq,
				   dst_link_freq);
		sc2335->cur_vts = mode->vts_def;
	}

	mutex_unlock(&sc2335->mutex);

	return 0;
}

static int sc2335_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc2335 *sc2335 = to_sc2335(sd);
	const struct sc2335_mode *mode = sc2335->cur_mode;

	mutex_lock(&sc2335->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sc2335->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = PIX_FORMAT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&sc2335->mutex);

	return 0;
}

static int sc2335_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = PIX_FORMAT;

	return 0;
}

static int sc2335_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != PIX_FORMAT)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int sc2335_enable_test_pattern(struct sc2335 *sc2335, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = sc2335_read_reg(sc2335->client, SC2335_REG_TEST_PATTERN,
			       SC2335_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= SC2335_TEST_PATTERN_BIT_MASK;
	else
		val &= ~SC2335_TEST_PATTERN_BIT_MASK;

	ret |= sc2335_write_reg(sc2335->client, SC2335_REG_TEST_PATTERN,
				SC2335_REG_VALUE_08BIT, val);
	return ret;
}

static void sc2335_get_module_inf(struct sc2335 *sc2335,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, SC2335_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, sc2335->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, sc2335->len_name, sizeof(inf->base.lens));
}

static long sc2335_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc2335 *sc2335 = to_sc2335(sd);
	struct rkmodule_hdr_cfg *hdr;
	const struct sc2335_mode *mode;
	u32 i, h, w;
	long ret = 0;
	u32  stream;
	s32 dst_link_freq = 0;
	s64 dst_pixel_rate = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		sc2335_get_module_inf(sc2335, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = sc2335->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = sc2335->cur_mode->width;
		h = sc2335->cur_mode->height;
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				sc2335->cur_mode = &supported_modes[i];
				dev_dbg(&sc2335->client->dev, "cur mode idx=%d\n", i);
				break;
			}
		}
		if (i == ARRAY_SIZE(supported_modes)) {
			dev_err(&sc2335->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			mode = sc2335->cur_mode;
			w = mode->hts_def - mode->width;
			h = mode->vts_def - mode->height;
			__v4l2_ctrl_modify_range(sc2335->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(sc2335->vblank, h,
						 SC2335_VTS_MAX - mode->height, 1, h);
			dst_link_freq = mode->freq_idx;
			dst_pixel_rate = (u32)link_freq_menu_items[mode->freq_idx] /
							mode->bpp * 2 * mode->lanes;
			__v4l2_ctrl_s_ctrl_int64(sc2335->pixel_rate,
				dst_pixel_rate);
			__v4l2_ctrl_s_ctrl(sc2335->link_freq,
				dst_link_freq);
			sc2335->cur_vts = mode->vts_def;
		}
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);

		if (stream)
			ret = sc2335_write_reg(sc2335->client, SC2335_REG_CTRL_MODE,
				SC2335_REG_VALUE_08BIT, SC2335_MODE_STREAMING);
		else
			ret = sc2335_write_reg(sc2335->client, SC2335_REG_CTRL_MODE,
				SC2335_REG_VALUE_08BIT, SC2335_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sc2335_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_hdr_cfg *hdr;
	long ret;
	u32  stream;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc2335_ioctl(sd, cmd, inf);
		if (!ret) {
			ret = copy_to_user(up, inf, sizeof(*inf));
			if (ret)
				ret = -EFAULT;
		}
		kfree(inf);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc2335_ioctl(sd, cmd, hdr);
		if (!ret) {
			ret = copy_to_user(up, hdr, sizeof(*hdr));
			if (ret)
				ret = -EFAULT;
		}
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}
		if (copy_from_user(hdr, up, sizeof(*hdr)))
			return -EFAULT;

		ret = sc2335_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;

		ret = sc2335_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int sc2335_set_ctrl_gain(struct sc2335 *sc2335, u32 a_gain)
{
	struct i2c_client *client = sc2335->client;
	int ret = 0;
	u32 coarse_again = 0, fine_again = 0;
	u32 switch_value = 0, gain_ctrl = 0;
	u32 coarse_dgain = 0, fine_dgain = 0;

	dev_dbg(&client->dev, "set analog gain 0x%x\n", a_gain);
	if (a_gain < 0x80) { /*1x ~ 2x*/
		fine_again = a_gain;
		coarse_again = 0x03;
		fine_dgain = 0x80;
		coarse_dgain = 0x00;
		ret = sc2335_read_reg(client, 0x3040,
			SC2335_REG_VALUE_08BIT, &gain_ctrl);
		if (ret)
			return ret;
		if (gain_ctrl == 0x40)
			switch_value = 0x0e;
		else if (gain_ctrl == 0x41)
			switch_value = 0x0f;
		else
			switch_value = 0x07;
	} else if (a_gain < 0x100) { /*2x ~ 4x*/
		fine_again = a_gain >> 1;
		coarse_again = 0x7;
		switch_value = 0x07;
		fine_dgain = 0x80;
		coarse_dgain = 0x00;
	} else if (a_gain < 0x200) { /*4x ~ 8x*/
		fine_again = a_gain >> 2;
		coarse_again = 0xf;
		switch_value = 0x07;
		fine_dgain = 0x80;
		coarse_dgain = 0x00;
	} else if (a_gain < 0x400) { /*8x ~ 16x*/
		fine_again = a_gain >> 3;
		coarse_again = 0x1f;
		switch_value = 0x07;
		fine_dgain = 0x80;
		coarse_dgain = 0x00;
	} else if (a_gain < 0x800) { /*16 ~ 32x*/
		fine_again = 0x7f;
		coarse_again = 0x1f;
		switch_value = 0x07;
		fine_dgain = a_gain >> 3;
		coarse_dgain = 0x00;
	} else if (a_gain < 0x1000) { /*32 ~ 64*/
		fine_again = 0x7f;
		coarse_again = 0x1f;
		switch_value = 0x07;
		fine_dgain = a_gain >> 4;
		coarse_dgain = 0x01;
	} else if (a_gain < 0x2000) { /*64 ~ 128*/
		fine_again = 0x7f;
		coarse_again = 0x1f;
		switch_value = 0x07;
		fine_dgain = a_gain >> 5;
		coarse_dgain = 0x03;
	} else if (a_gain < 0x4000) { /*128 ~ 256*/
		fine_again = 0x7f;
		coarse_again = 0x1f;
		switch_value = 0x07;
		fine_dgain = a_gain >> 6;
		coarse_dgain = 0x07;
	} else if (a_gain < 0x8000) { /*256 ~ 504*/
		fine_again = 0x7f;
		coarse_again = 0x1f;
		switch_value = 0x07;
		fine_dgain = a_gain >> 7;
		coarse_dgain = 0x0f;
	}
	dev_dbg(&client->dev, "set fine_again = 0x%x, coarse_again = 0x%x, coarse_dgain=0x%x, fine_dgain=0x%x\n",
			fine_again, coarse_again, coarse_dgain, fine_dgain);
	ret = sc2335_write_reg(sc2335->client, 0x363c,
		SC2335_REG_VALUE_08BIT, switch_value);
	ret |= sc2335_write_reg(sc2335->client,
		SC2335_REG_COARSE_AGAIN,
		SC2335_REG_VALUE_08BIT,
		coarse_again);
	ret |= sc2335_write_reg(sc2335->client,
		SC2335_REG_FINE_AGAIN,
		SC2335_REG_VALUE_08BIT,
		fine_again);
	ret |= sc2335_write_reg(sc2335->client,
		SC2335_REG_COARSE_DGAIN,
		SC2335_REG_VALUE_08BIT,
		coarse_dgain);
	ret |= sc2335_write_reg(sc2335->client,
		SC2335_REG_FINE_DGAIN,
		SC2335_REG_VALUE_08BIT,
		fine_dgain);
	return ret;
}

static int __sc2335_start_stream(struct sc2335 *sc2335)
{
	int ret;

	ret = sc2335_write_array(sc2335->client, sc2335->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&sc2335->mutex);
	ret = v4l2_ctrl_handler_setup(&sc2335->ctrl_handler);
	mutex_lock(&sc2335->mutex);
	if (ret)
		return ret;

	return sc2335_write_reg(sc2335->client, SC2335_REG_CTRL_MODE,
			SC2335_REG_VALUE_08BIT, SC2335_MODE_STREAMING);
}

static int __sc2335_stop_stream(struct sc2335 *sc2335)
{
	return sc2335_write_reg(sc2335->client, SC2335_REG_CTRL_MODE,
			SC2335_REG_VALUE_08BIT, SC2335_MODE_SW_STANDBY);
}

static int sc2335_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sc2335 *sc2335 = to_sc2335(sd);
	struct i2c_client *client = sc2335->client;
	int ret = 0;

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
				sc2335->cur_mode->width,
				sc2335->cur_mode->height,
		DIV_ROUND_CLOSEST(sc2335->cur_mode->max_fps.denominator,
				  sc2335->cur_mode->max_fps.numerator));

	mutex_lock(&sc2335->mutex);
	on = !!on;
	if (on == sc2335->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __sc2335_start_stream(sc2335);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__sc2335_stop_stream(sc2335);
		pm_runtime_put(&client->dev);
	}

	sc2335->streaming = on;

unlock_and_return:
	mutex_unlock(&sc2335->mutex);

	return ret;
}

static int sc2335_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct sc2335 *sc2335 = to_sc2335(sd);
	const struct sc2335_mode *mode = sc2335->cur_mode;

	mutex_lock(&sc2335->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&sc2335->mutex);

	return 0;
}

static int sc2335_s_power(struct v4l2_subdev *sd, int on)
{
	struct sc2335 *sc2335 = to_sc2335(sd);
	struct i2c_client *client = sc2335->client;
	int ret = 0;

	mutex_lock(&sc2335->mutex);

	/* If the power state is not modified - no work to do. */
	if (sc2335->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		ret = sc2335_write_array(sc2335->client, sc2335_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		sc2335->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		sc2335->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&sc2335->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 sc2335_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, SC2335_XVCLK_FREQ / 1000 / 1000);
}

static int __sc2335_power_on(struct sc2335 *sc2335)
{
	int ret;
	u32 delay_us;
	struct device *dev = &sc2335->client->dev;

	if (!IS_ERR_OR_NULL(sc2335->pins_default)) {
		ret = pinctrl_select_state(sc2335->pinctrl,
					   sc2335->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	ret = clk_set_rate(sc2335->xvclk, SC2335_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(sc2335->xvclk) != SC2335_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(sc2335->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(sc2335->power_gpio))
		gpiod_set_value_cansleep(sc2335->power_gpio, 1);

	if (!IS_ERR(sc2335->reset_gpio))
		gpiod_set_value_cansleep(sc2335->reset_gpio, 0);

	ret = regulator_bulk_enable(SC2335_NUM_SUPPLIES, sc2335->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(sc2335->reset_gpio))
		gpiod_set_value_cansleep(sc2335->reset_gpio, 1);

	if (!IS_ERR(sc2335->pwdn_gpio))
		gpiod_set_value_cansleep(sc2335->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = sc2335_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	return 0;

disable_clk:
	clk_disable_unprepare(sc2335->xvclk);

	return ret;
}

static void __sc2335_power_off(struct sc2335 *sc2335)
{
	if (!IS_ERR(sc2335->pwdn_gpio))
		gpiod_set_value_cansleep(sc2335->pwdn_gpio, 0);
	clk_disable_unprepare(sc2335->xvclk);
	if (!IS_ERR(sc2335->reset_gpio))
		gpiod_set_value_cansleep(sc2335->reset_gpio, 0);

	if (!IS_ERR(sc2335->power_gpio))
		gpiod_set_value_cansleep(sc2335->power_gpio, 0);

	regulator_bulk_disable(SC2335_NUM_SUPPLIES, sc2335->supplies);
}

static int sc2335_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc2335 *sc2335 = to_sc2335(sd);

	return __sc2335_power_on(sc2335);
}

static int sc2335_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc2335 *sc2335 = to_sc2335(sd);

	__sc2335_power_off(sc2335);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sc2335_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc2335 *sc2335 = to_sc2335(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sc2335_mode *def_mode = &supported_modes[0];

	mutex_lock(&sc2335->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = PIX_FORMAT;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&sc2335->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int sc2335_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *config)
{
	struct sc2335 *sc2335 = to_sc2335(sd);
	u32 val = 0;

	val = 1 << (sc2335->cur_mode->lanes - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	config->type = V4L2_MBUS_CSI2;
	config->flags = val;

	return 0;
}

static int sc2335_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fie->code != PIX_FORMAT)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;

	return 0;
}

static const struct dev_pm_ops sc2335_pm_ops = {
	SET_RUNTIME_PM_OPS(sc2335_runtime_suspend,
			   sc2335_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sc2335_internal_ops = {
	.open = sc2335_open,
};
#endif

static const struct v4l2_subdev_core_ops sc2335_core_ops = {
	.s_power = sc2335_s_power,
	.ioctl = sc2335_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc2335_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc2335_video_ops = {
	.s_stream = sc2335_s_stream,
	.g_frame_interval = sc2335_g_frame_interval,
	.g_mbus_config = sc2335_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops sc2335_pad_ops = {
	.enum_mbus_code = sc2335_enum_mbus_code,
	.enum_frame_size = sc2335_enum_frame_sizes,
	.enum_frame_interval = sc2335_enum_frame_interval,
	.get_fmt = sc2335_get_fmt,
	.set_fmt = sc2335_set_fmt,
};

static const struct v4l2_subdev_ops sc2335_subdev_ops = {
	.core	= &sc2335_core_ops,
	.video	= &sc2335_video_ops,
	.pad	= &sc2335_pad_ops,
};

static int sc2335_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc2335 *sc2335 = container_of(ctrl->handler,
					     struct sc2335, ctrl_handler);
	struct i2c_client *client = sc2335->client;
	s64 max;
	int ret = 0;
	u32 val;

	dev_dbg(&client->dev, "ctrl->id(0x%x) val 0x%x\n",
		ctrl->id, ctrl->val);
	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sc2335->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(sc2335->exposure,
					 sc2335->exposure->minimum, max,
					 sc2335->exposure->step,
					 sc2335->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = sc2335_write_reg(sc2335->client, SC2335_REG_EXPOSURE,
				       SC2335_REG_VALUE_24BIT, ctrl->val << 5);
		dev_dbg(&client->dev, "set exposure 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = sc2335_set_ctrl_gain(sc2335, ctrl->val);
		dev_dbg(&client->dev, "set analog gain 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = sc2335_write_reg(sc2335->client, SC2335_REG_VTS,
				       SC2335_REG_VALUE_16BIT,
				       ctrl->val + sc2335->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = sc2335_enable_test_pattern(sc2335, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = sc2335_read_reg(sc2335->client, SC2335_REG_FLIP_MIRROR,
				       SC2335_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC2335_MIRROR_MASK;
		else
			val &= ~SC2335_MIRROR_MASK;
		ret |= sc2335_write_reg(sc2335->client, SC2335_REG_FLIP_MIRROR,
			SC2335_REG_VALUE_08BIT, val);
		break;
	case V4L2_CID_VFLIP:
		ret = sc2335_read_reg(sc2335->client, SC2335_REG_FLIP_MIRROR,
				       SC2335_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC2335_FLIP_MASK;
		else
			val &= ~SC2335_FLIP_MASK;
		ret |= sc2335_write_reg(sc2335->client, SC2335_REG_FLIP_MIRROR,
			SC2335_REG_VALUE_08BIT, val);
		break;

	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sc2335_ctrl_ops = {
	.s_ctrl = sc2335_set_ctrl,
};

static int sc2335_initialize_controls(struct sc2335 *sc2335)
{
	const struct sc2335_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	s32 dst_link_freq = 0;
	s64 dst_pixel_rate = 0;

	handler = &sc2335->ctrl_handler;
	mode = sc2335->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &sc2335->mutex;

	sc2335->link_freq = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);

	dst_link_freq = mode->freq_idx;
	dst_pixel_rate = (u32)link_freq_menu_items[mode->freq_idx] / mode->bpp * 2 * mode->lanes;
	__v4l2_ctrl_s_ctrl(sc2335->link_freq,
			   dst_link_freq);
	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, SC2335_PIXEL_RATE_TWO_LANES, 1, dst_pixel_rate);

	h_blank = mode->hts_def - mode->width;
	sc2335->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (sc2335->hblank)
		sc2335->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sc2335->cur_vts = mode->vts_def;
	vblank_def = mode->vts_def - mode->height;
	sc2335->vblank = v4l2_ctrl_new_std(handler, &sc2335_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				SC2335_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	sc2335->exposure = v4l2_ctrl_new_std(handler, &sc2335_ctrl_ops,
				V4L2_CID_EXPOSURE, SC2335_EXPOSURE_MIN,
				exposure_max, SC2335_EXPOSURE_STEP,
				mode->exp_def);

	sc2335->anal_gain = v4l2_ctrl_new_std(handler, &sc2335_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, ANALOG_GAIN_MIN,
				ANALOG_GAIN_MAX, ANALOG_GAIN_STEP,
				ANALOG_GAIN_DEFAULT);

	sc2335->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&sc2335_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(sc2335_test_pattern_menu) - 1,
				0, 0, sc2335_test_pattern_menu);
	sc2335->h_flip = v4l2_ctrl_new_std(handler, &sc2335_ctrl_ops,
					   V4L2_CID_HFLIP, 0, 1, 1, 0);

	sc2335->v_flip = v4l2_ctrl_new_std(handler, &sc2335_ctrl_ops,
					   V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&sc2335->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	sc2335->subdev.ctrl_handler = handler;
	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int sc2335_check_sensor_id(struct sc2335 *sc2335,
				  struct i2c_client *client)
{
	struct device *dev = &sc2335->client->dev;
	u32 id = 0;
	int ret;

	ret = sc2335_read_reg(client, SC2335_REG_CHIP_ID,
			      SC2335_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%04x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected SC2335 CHIP ID = 0x%04x sensor\n", CHIP_ID);

	return 0;
}

static int sc2335_configure_regulators(struct sc2335 *sc2335)
{
	unsigned int i;

	for (i = 0; i < SC2335_NUM_SUPPLIES; i++)
		sc2335->supplies[i].supply = sc2335_supply_names[i];

	return devm_regulator_bulk_get(&sc2335->client->dev,
				       SC2335_NUM_SUPPLIES,
				       sc2335->supplies);
}

static int sc2335_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct device_node *endpoint;
	struct sc2335 *sc2335;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	sc2335 = devm_kzalloc(dev, sizeof(*sc2335), GFP_KERNEL);
	if (!sc2335)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sc2335->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sc2335->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sc2335->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sc2335->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint),
		&sc2335->bus_cfg);
	if (ret) {
		dev_err(dev, "Failed to pasrse endpoint\n");
		return -EINVAL;
	}

	if (sc2335->bus_cfg.bus.mipi_csi1.data_lane == 1)
		sc2335->cur_mode = &supported_modes[1];
	else
		sc2335->cur_mode = &supported_modes[0];

	sc2335->client = client;

	sc2335->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sc2335->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}
	sc2335->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(sc2335->pinctrl)) {
		sc2335->pins_default =
			pinctrl_lookup_state(sc2335->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sc2335->pins_default))
			dev_err(dev, "could not get default pinstate\n");

	} else {
		dev_err(dev, "no pinctrl\n");
	}

	sc2335->power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(sc2335->power_gpio))
		dev_warn(dev, "Failed to get power-gpios\n");

	sc2335->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sc2335->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	sc2335->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(sc2335->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");
	ret = sc2335_configure_regulators(sc2335);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&sc2335->mutex);

	sd = &sc2335->subdev;
	v4l2_i2c_subdev_init(sd, client, &sc2335_subdev_ops);
	ret = sc2335_initialize_controls(sc2335);
	if (ret)
		goto err_destroy_mutex;

	ret = __sc2335_power_on(sc2335);
	if (ret)
		goto err_free_handler;

	ret = sc2335_check_sensor_id(sc2335, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sc2335_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif

#if defined(CONFIG_MEDIA_CONTROLLER)
	sc2335->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sc2335->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(sc2335->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sc2335->module_index, facing,
		 SC2335_NAME, dev_name(sd->dev));
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
	__sc2335_power_off(sc2335);
err_free_handler:
	v4l2_ctrl_handler_free(&sc2335->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sc2335->mutex);

	return ret;
}

static int sc2335_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc2335 *sc2335 = to_sc2335(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sc2335->ctrl_handler);
	mutex_destroy(&sc2335->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sc2335_power_off(sc2335);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sc2335_of_match[] = {
	{ .compatible = "smartsens,sc2335" },
	{},
};
MODULE_DEVICE_TABLE(of, sc2335_of_match);
#endif

static const struct i2c_device_id sc2335_match_id[] = {
	{ "smartsens,sc2335", 0 },
	{ },
};

static struct i2c_driver sc2335_i2c_driver = {
	.driver = {
		.name = SC2335_NAME,
		.pm = &sc2335_pm_ops,
		.of_match_table = of_match_ptr(sc2335_of_match),
	},
	.probe		= &sc2335_probe,
	.remove		= &sc2335_remove,
	.id_table	= sc2335_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sc2335_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sc2335_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Smartsens sc2335 sensor driver");
MODULE_AUTHOR("zack.zeng");
MODULE_LICENSE("GPL v2");
