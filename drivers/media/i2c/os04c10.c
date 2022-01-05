// SPDX-License-Identifier: GPL-2.0
/*
 * os04c10 driver
 *
 * Copyright (C) 2021 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version.
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
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/rk-preisp.h>
#include "../platform/rockchip/isp/rkisp_tb_helper.h"

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x00)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define OS04C10_LANES			4
#define MIPI_FREQ_384M			384000000
#define PIXEL_RATE_WITH_384M	(MIPI_FREQ_384M * 2 / 10 * OS04C10_LANES)

#define OS04C10_XVCLK_FREQ		24000000

#define OS04C10_CHIP_ID			0x530443
#define OS04C10_REG_CHIP_ID		0x300A

#define OS04C10_VTS_MAX			0xFFFF

#define OS04C10_REG_EXP_LONG_H		0x3501

#define OS04C10_REG_AGAIN_LONG_H	0x3508
#define OS04C10_REG_DGAIN_LONG_H	0x350A

#define OS04C10_REG_CTRL_MODE		0x0100
#define OS04C10_MODE_SW_STANDBY		0x0
#define OS04C10_MODE_STREAMING		BIT(0)

#define OS04C10_REG_SOFTWARE_RESET	0x0103
#define OS04C10_SOFTWARE_RESET_VAL	0x1

#define OS04C10_GAIN_MIN		0x0080
#define OS04C10_GAIN_MAX		0x7820
#define OS04C10_GAIN_STEP		1
#define OS04C10_GAIN_DEFAULT	0x0080

#define OS04C10_EXPOSURE_MIN		2
#define OS04C10_EXPOSURE_STEP		1

#define OS04C10_ANALOG_FLIP_REG	0x3716
#define ANALOG_FLIP_BIT_MASK	BIT(5)
#define OS04C10_FLIP_REG		0x3820
#define MIRROR_BIT_MASK			BIT(3)
#define FLIP_BIT_MASK			0x30

#define OS04C10_REG_VALUE_08BIT		1
#define OS04C10_REG_VALUE_16BIT		2
#define OS04C10_REG_VALUE_24BIT		3

#define OS04C10_NAME			"os04c10"

#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"
#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define OS04C10_REG_VTS			0x380E

#define REG_NULL			0xFFFF

static const char * const OS04C10_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define OS04C10_NUM_SUPPLIES ARRAY_SIZE(OS04C10_supply_names)

enum os04c10_max_pad {
	PAD0,
	PAD1,
	PAD2,
	PAD3,
	PAD_MAX,
};

struct regval {
	u16 addr;
	u8 val;
};

struct os04c10_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 vc[PAD_MAX];
};

struct os04c10 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[OS04C10_NUM_SUPPLIES];

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
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;
	struct v4l2_ctrl	*h_flip;
	struct v4l2_ctrl	*v_flip;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	bool			is_thunderboot;
	bool			is_thunderboot_ng;
	bool			is_first_streamoff;
	const struct os04c10_mode *cur_mode;
	u32			cfg_num;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	bool			has_init_exp;
	struct preisp_hdrae_exp_s init_hdrae_exp;
	u8			flip;
};

#define to_os04c10(sd) container_of(sd, struct os04c10, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval os04c10_linear10bit_2688x1520_regs[] = {
	{0x0100, 0x00},
	{0x0103, 0x01},
	{0x0301, 0x84},
	{0x0303, 0x01},
	{0x0305, 0x30},
	{0x0306, 0x01},
	{0x0307, 0x17},
	{0x0323, 0x04},
	{0x0324, 0x01},
	{0x0325, 0x62},
	{0x3012, 0x06},
	{0x3013, 0x02},
	{0x3016, 0x72},
	{0x3021, 0x03},
	{0x3106, 0x21},
	{0x3107, 0xa1},
	{0x3500, 0x00},
	{0x3501, 0x06},
	{0x3502, 0x1e},
	{0x3503, 0x88},
	{0x3508, 0x00},
	{0x3509, 0x80},
	{0x350a, 0x04},
	{0x350b, 0x00},
	{0x350c, 0x00},
	{0x350d, 0x80},
	{0x350e, 0x04},
	{0x350f, 0x00},
	{0x3510, 0x00},
	{0x3511, 0x00},
	{0x3512, 0x20},
	{0x3624, 0x02},
	{0x3625, 0x4c},
	{0x3660, 0x00},
	{0x3666, 0xa5},
	{0x3667, 0xa5},
	{0x366a, 0x64},
	{0x3673, 0x0d},
	{0x3672, 0x0d},
	{0x3671, 0x0d},
	{0x3670, 0x0d},
	{0x3685, 0x00},
	{0x3694, 0x0d},
	{0x3693, 0x0d},
	{0x3692, 0x0d},
	{0x3691, 0x0d},
	{0x3696, 0x4c},
	{0x3697, 0x4c},
	{0x3698, 0x40},
	{0x3699, 0x80},
	{0x369a, 0x18},
	{0x369b, 0x1f},
	{0x369c, 0x14},
	{0x369d, 0x80},
	{0x369e, 0x40},
	{0x369f, 0x21},
	{0x36a0, 0x12},
	{0x36a1, 0x5d},
	{0x36a2, 0x66},
	{0x370a, 0x00},
	{0x370e, 0x0c},
	{0x3710, 0x00},
	{0x3713, 0x00},
	{0x3725, 0x02},
	{0x372a, 0x03},
	{0x3738, 0xce},
	{0x3739, 0x10},
	{0x3748, 0x00},
	{0x374a, 0x00},
	{0x374c, 0x00},
	{0x374e, 0x00},
	{0x3756, 0x00},
	{0x3757, 0x0e},
	{0x3767, 0x00},
	{0x3771, 0x00},
	{0x377b, 0x20},
	{0x377c, 0x00},
	{0x377d, 0x0c},
	{0x3781, 0x03},
	{0x3782, 0x00},
	{0x3789, 0x14},
	{0x3795, 0x02},
	{0x379c, 0x00},
	{0x379d, 0x00},
	{0x37b8, 0x04},
	{0x37ba, 0x03},
	{0x37bb, 0x00},
	{0x37bc, 0x04},
	{0x37be, 0x08},
	{0x37c4, 0x11},
	{0x37c5, 0x80},
	{0x37c6, 0x14},
	{0x37c7, 0x08},
	{0x37da, 0x11},
	{0x381f, 0x08},
	{0x3829, 0x03},
	{0x3832, 0x00},
	{0x3881, 0x00},
	{0x3888, 0x04},
	{0x388b, 0x00},
	{0x3c80, 0x10},
	{0x3c86, 0x00},
	{0x3c9f, 0x01},
	{0x3d85, 0x1b},
	{0x3d8c, 0x71},
	{0x3d8d, 0xe2},
	{0x3f00, 0x0b},
	{0x3f06, 0x04},
	{0x400a, 0x01},
	{0x400b, 0x50},
	{0x400e, 0x08},
	{0x4040, 0x00},
	{0x4041, 0x07},
	{0x4043, 0x7e},
	{0x4045, 0x7e},
	{0x4047, 0x7e},
	{0x4049, 0x7e},
	{0x4090, 0x14},
	{0x40b0, 0x00},
	{0x40b1, 0x00},
	{0x40b2, 0x00},
	{0x40b3, 0x00},
	{0x40b4, 0x00},
	{0x40b5, 0x00},
	{0x40b7, 0x00},
	{0x40b8, 0x00},
	{0x40b9, 0x00},
	{0x40ba, 0x00},
	{0x4301, 0x00},
	{0x4303, 0x00},
	{0x4502, 0x04},
	{0x4503, 0x00},
	{0x4504, 0x06},
	{0x4506, 0x00},
	{0x4507, 0x64},
	{0x4803, 0x00},
	{0x480c, 0x32},
	{0x480e, 0x00},
	{0x4813, 0x00},
	{0x4819, 0x70},
	{0x481f, 0x30},
	{0x4823, 0x3f},
	{0x4825, 0x30},
	{0x4833, 0x10},
	{0x484b, 0x07},
	{0x488b, 0x00},
	{0x4d00, 0x04},
	{0x4d01, 0xad},
	{0x4d02, 0xbc},
	{0x4d03, 0xa1},
	{0x4d04, 0x1f},
	{0x4d05, 0x4c},
	{0x4d0b, 0x01},
	{0x4e00, 0x2a},
	{0x4e0d, 0x00},
	{0x5001, 0x09},
	{0x5004, 0x00},
	{0x5080, 0x04},
	{0x5036, 0x00},
	{0x5180, 0x70},
	{0x5181, 0x10},
	{0x520a, 0x03},
	{0x520b, 0x06},
	{0x520c, 0x0c},
	{0x580b, 0x0f},
	{0x580d, 0x00},
	{0x580f, 0x00},
	{0x5820, 0x00},
	{0x5821, 0x00},
	{0x301c, 0xf8},
	{0x301e, 0xb4},
	{0x301f, 0xd0},
	{0x3022, 0x01},
	{0x3109, 0xe7},
	{0x3600, 0x00},
	{0x3610, 0x65},
	{0x3611, 0x85},
	{0x3613, 0x3a},
	{0x3615, 0x60},
	{0x3621, 0x90},
	{0x3620, 0x0c},
	{0x3629, 0x00},
	{0x3661, 0x04},
	{0x3662, 0x10},
	{0x3664, 0x70},
	{0x3665, 0x00},
	{0x3681, 0xa6},
	{0x3682, 0x53},
	{0x3683, 0x2a},
	{0x3684, 0x15},
	{0x3700, 0x2a},
	{0x3701, 0x12},
	{0x3703, 0x28},
	{0x3704, 0x0e},
	{0x3706, 0x4a},
	{0x3709, 0x4a},
	{0x370b, 0xa2},
	{0x370c, 0x01},
	{0x370f, 0x04},
	{0x3714, 0x24},
	{0x3716, 0x24},
	{0x3719, 0x11},
	{0x371a, 0x1e},
	{0x3720, 0x00},
	{0x3724, 0x13},
	{0x373f, 0xb0},
	{0x3741, 0x4a},
	{0x3743, 0x4a},
	{0x3745, 0x4a},
	{0x3747, 0x4a},
	{0x3749, 0xa2},
	{0x374b, 0xa2},
	{0x374d, 0xa2},
	{0x374f, 0xa2},
	{0x3755, 0x10},
	{0x376c, 0x00},
	{0x378d, 0x30},
	{0x3790, 0x4a},
	{0x3791, 0xa2},
	{0x3798, 0x40},
	{0x379e, 0x00},
	{0x379f, 0x04},
	{0x37a1, 0x10},
	{0x37a2, 0x1e},
	{0x37a8, 0x10},
	{0x37a9, 0x1e},
	{0x37ac, 0xa0},
	{0x37b9, 0x01},
	{0x37bd, 0x01},
	{0x37bf, 0x26},
	{0x37c0, 0x11},
	{0x37c2, 0x04},
	{0x37cd, 0x19},
	{0x37d8, 0x02},
	{0x37d9, 0x08},
	{0x37e5, 0x02},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0a},
	{0x3805, 0x8f},
	{0x3806, 0x05},
	{0x3807, 0xff},
	{0x3808, 0x0a},
	{0x3809, 0x80},
	{0x380a, 0x05},
	{0x380b, 0xf0},
	{0x380c, 0x08},
	{0x380d, 0x5c},
	{0x380e, 0x06},
	{0x380f, 0x26},
	{0x3811, 0x08},
	{0x3813, 0x08},
	{0x3814, 0x01},
	{0x3815, 0x01},
	{0x3816, 0x01},
	{0x3817, 0x01},
	{0x3820, 0x80},
	{0x3821, 0x00},
	{0x3880, 0x25},
	{0x3882, 0x20},
	{0x3c91, 0x0b},
	{0x3c94, 0x45},
	{0x4000, 0xf3},
	{0x4001, 0x60},
	{0x4003, 0x40},
	{0x4008, 0x02},
	{0x4009, 0x0d},
	{0x4300, 0xff},
	{0x4302, 0x0f},
	{0x4305, 0x83},
	{0x4505, 0x84},
	{0x4809, 0x1e},
	{0x480a, 0x04},
	{0x4837, 0x15},
	{0x4c00, 0x08},
	{0x4c01, 0x08},
	{0x4c04, 0x00},
	{0x4c05, 0x00},
	{0x5000, 0xe9},
	{REG_NULL, 0x00},
};

/*
 * The width and height must be configured to be
 * the same as the current output resolution of the sensor.
 * The input width of the isp needs to be 16 aligned.
 * The input height of the isp needs to be 8 aligned.
 * If the width or height does not meet the alignment rules,
 * you can configure the cropping parameters with the following function to
 * crop out the appropriate resolution.
 * struct v4l2_subdev_pad_ops {
 *	.get_selection
 * }
 */
static const struct os04c10_mode supported_modes[] = {
	{
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 2688,
		.height = 1520,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x061e,
		.hts_def = 0x085c,
		.vts_def = 0x0626,
		.reg_list = os04c10_linear10bit_2688x1520_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const s64 link_freq_menu_items[] = {
	MIPI_FREQ_384M,
};

static int __os04c10_power_on(struct os04c10 *os04c10);

static int os04c10_check_sensor_id(struct os04c10 *os04c10,
				   struct i2c_client *client);

/* Write registers up to 4 at a time */
static int os04c10_write_reg(struct i2c_client *client, u16 reg,
			     u32 len, u32 val)
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

static int os04c10_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	int i, ret = 0;

	i = 0;
	while (regs[i].addr != REG_NULL) {
		ret = os04c10_write_reg(client, regs[i].addr, OS04C10_REG_VALUE_08BIT, regs[i].val);
		if (ret) {
			dev_err(&client->dev, "%s failed !\n", __func__);
			break;
		}
		i++;
	}

	return ret;
}

/* Read registers up to 4 at a time */
static int os04c10_read_reg(struct i2c_client *client,
			    u16 reg,
			    unsigned int len,
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

static int os04c10_get_reso_dist(const struct os04c10_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct os04c10_mode *
os04c10_find_best_fit(struct os04c10 *os04c10, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < os04c10->cfg_num; i++) {
		dist = os04c10_get_reso_dist(&supported_modes[i], framefmt);
		if ((cur_best_fit_dist == -1 || dist <= cur_best_fit_dist) &&
					(supported_modes[i].bus_fmt == framefmt->code)) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int os04c10_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct os04c10 *os04c10 = to_os04c10(sd);
	const struct os04c10_mode *mode;
	s64 h_blank, vblank_def;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;

	mutex_lock(&os04c10->mutex);

	mode = os04c10_find_best_fit(os04c10, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&os04c10->mutex);
		return -ENOTTY;
#endif
	} else {
		os04c10->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(os04c10->hblank, h_blank,
						h_blank, 1, h_blank);

		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(os04c10->vblank, vblank_def,
						OS04C10_VTS_MAX - mode->height,
						1, vblank_def);
		if (mode->hdr_mode == NO_HDR) {
			if (mode->bus_fmt == MEDIA_BUS_FMT_SBGGR10_1X10) {
				dst_link_freq = 0;
				dst_pixel_rate = PIXEL_RATE_WITH_384M;
			}
		}
		__v4l2_ctrl_s_ctrl_int64(os04c10->pixel_rate,
						dst_pixel_rate);
		__v4l2_ctrl_s_ctrl(os04c10->link_freq,
					dst_link_freq);
	}

	mutex_unlock(&os04c10->mutex);

	return 0;
}

static int os04c10_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct os04c10 *os04c10 = to_os04c10(sd);
	const struct os04c10_mode *mode = os04c10->cur_mode;

	mutex_lock(&os04c10->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&os04c10->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&os04c10->mutex);

	return 0;
}

static int os04c10_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct os04c10 *os04c10 = to_os04c10(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = os04c10->cur_mode->bus_fmt;

	return 0;
}

static int os04c10_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	struct os04c10 *os04c10 = to_os04c10(sd);

	if (fse->index >= os04c10->cfg_num)
		return -EINVAL;

	if (fse->code != supported_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int os04c10_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct os04c10 *os04c10 = to_os04c10(sd);
	const struct os04c10_mode *mode = os04c10->cur_mode;

	mutex_lock(&os04c10->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&os04c10->mutex);

	return 0;
}

static int os04c10_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *config)
{
	struct os04c10 *os04c10 = to_os04c10(sd);
	const struct os04c10_mode *mode = os04c10->cur_mode;
	u32 val = 0;

	if (mode->hdr_mode == NO_HDR)
		val = 1 << (OS04C10_LANES - 1) |
		      V4L2_MBUS_CSI2_CHANNEL_0 |
		      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	config->type = V4L2_MBUS_CSI2;
	config->flags = val;

	return 0;
}

static void os04c10_get_module_inf(struct os04c10 *os04c10,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, OS04C10_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, os04c10->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, os04c10->len_name, sizeof(inf->base.lens));
}

static long os04c10_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct os04c10 *os04c10 = to_os04c10(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case PREISP_CMD_SET_HDRAE_EXP:
		ret = -1;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		if (hdr_cfg->hdr_mode != 0)
			ret = -1;
		break;
	case RKMODULE_GET_MODULE_INFO:
		os04c10_get_module_inf(os04c10, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = os04c10->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);
		if (stream)
			ret = os04c10_write_reg(os04c10->client, OS04C10_REG_CTRL_MODE,
						OS04C10_REG_VALUE_08BIT, OS04C10_MODE_STREAMING);
		else
			ret = os04c10_write_reg(os04c10->client, OS04C10_REG_CTRL_MODE,
						OS04C10_REG_VALUE_08BIT, OS04C10_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long os04c10_compat_ioctl32(struct v4l2_subdev *sd, unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	long ret;
	u32 cg = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = os04c10_ioctl(sd, cmd, inf);
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
			ret = os04c10_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = os04c10_ioctl(sd, cmd, hdr);
		if (!ret)
			ret = copy_to_user(up, hdr, sizeof(*hdr));
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdr, up, sizeof(*hdr));
		if (!ret)
			ret = os04c10_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		hdrae = kzalloc(sizeof(*hdrae), GFP_KERNEL);
		if (!hdrae) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdrae, up, sizeof(*hdrae));
		if (!ret)
			ret = os04c10_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;
	case RKMODULE_SET_CONVERSION_GAIN:
		ret = copy_from_user(&cg, up, sizeof(cg));
		if (!ret)
			ret = os04c10_ioctl(sd, cmd, &cg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = os04c10_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __os04c10_start_stream(struct os04c10 *os04c10)
{
	int ret;

	if (!os04c10->is_thunderboot) {
		ret = os04c10_write_array(os04c10->client, os04c10->cur_mode->reg_list);
		if (ret)
			return ret;
	}

	/* In case these controls are set before streaming */
	ret = __v4l2_ctrl_handler_setup(&os04c10->ctrl_handler);
	if (ret)
		return ret;
	if (os04c10->has_init_exp && os04c10->cur_mode->hdr_mode != NO_HDR) {
		ret = os04c10_ioctl(&os04c10->subdev, PREISP_CMD_SET_HDRAE_EXP,
							&os04c10->init_hdrae_exp);
		if (ret) {
			dev_err(&os04c10->client->dev, "init exp fail in hdr mode\n");
			return ret;
		}
	}
	return os04c10_write_reg(os04c10->client, OS04C10_REG_CTRL_MODE,
			OS04C10_REG_VALUE_08BIT, OS04C10_MODE_STREAMING);
}

static int __os04c10_stop_stream(struct os04c10 *os04c10)
{
	os04c10->has_init_exp = false;
	if (os04c10->is_thunderboot)
		os04c10->is_first_streamoff = true;
	return os04c10_write_reg(os04c10->client, OS04C10_REG_CTRL_MODE,
		OS04C10_REG_VALUE_08BIT, OS04C10_MODE_SW_STANDBY);
}

static int os04c10_s_stream(struct v4l2_subdev *sd, int on)
{
	struct os04c10 *os04c10 = to_os04c10(sd);
	struct i2c_client *client = os04c10->client;
	int ret = 0;

	mutex_lock(&os04c10->mutex);
	on = !!on;
	if (on == os04c10->streaming)
		goto unlock_and_return;

	if (on) {
		if (os04c10->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			os04c10->is_thunderboot = false;
			__os04c10_power_on(os04c10);
		}
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __os04c10_start_stream(os04c10);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__os04c10_stop_stream(os04c10);
		pm_runtime_put(&client->dev);
	}

	os04c10->streaming = on;

unlock_and_return:
	mutex_unlock(&os04c10->mutex);

	return ret;
}

static int os04c10_s_power(struct v4l2_subdev *sd, int on)
{
	struct os04c10 *os04c10 = to_os04c10(sd);
	struct i2c_client *client = os04c10->client;
	int ret = 0;

	if (os04c10->is_thunderboot)
		return 0;

	mutex_lock(&os04c10->mutex);

	/* If the power state is not modified - no work to do. */
	if (os04c10->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret |= os04c10_write_reg(os04c10->client, OS04C10_REG_SOFTWARE_RESET,
					 OS04C10_REG_VALUE_08BIT, OS04C10_SOFTWARE_RESET_VAL);
		usleep_range(100, 200);

		os04c10->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		os04c10->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&os04c10->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 os04c10_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, OS04C10_XVCLK_FREQ / 1000 / 1000);
}

static int __os04c10_power_on(struct os04c10 *os04c10)
{
	int ret;
	u32 delay_us;
	struct device *dev = &os04c10->client->dev;

	if (os04c10->is_thunderboot)
		return 0;

	if (!IS_ERR_OR_NULL(os04c10->pins_default)) {
		ret = pinctrl_select_state(os04c10->pinctrl,
					   os04c10->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(os04c10->xvclk, OS04C10_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(os04c10->xvclk) != OS04C10_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(os04c10->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(os04c10->reset_gpio))
		gpiod_direction_output(os04c10->reset_gpio, 1);

	ret = regulator_bulk_enable(OS04C10_NUM_SUPPLIES, os04c10->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(os04c10->reset_gpio))
		gpiod_direction_output(os04c10->reset_gpio, 0);

	usleep_range(500, 1000);
	if (!IS_ERR(os04c10->pwdn_gpio))
		gpiod_direction_output(os04c10->pwdn_gpio, 1);
	/*
	 * There is no need to wait for the delay of RC circuit
	 * if the reset signal is directly controlled by GPIO.
	 */
	if (!IS_ERR(os04c10->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = os04c10_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(os04c10->xvclk);

	return ret;
}

static void __os04c10_power_off(struct os04c10 *os04c10)
{
	int ret;
	struct device *dev = &os04c10->client->dev;

	if (os04c10->is_thunderboot) {
		if (os04c10->is_first_streamoff) {
			os04c10->is_thunderboot = false;
			os04c10->is_first_streamoff = false;
		} else {
			return;
		}
	}

	// To avoid bad frames from MIPI
	os04c10_write_reg(os04c10->client, 0x3021, OS04C10_REG_VALUE_08BIT, 0x00);
	os04c10_write_reg(os04c10->client, 0x0100, OS04C10_REG_VALUE_08BIT, 0x00);

	if (!IS_ERR(os04c10->pwdn_gpio))
		gpiod_direction_output(os04c10->pwdn_gpio, 0);

	clk_disable_unprepare(os04c10->xvclk);

	if (!IS_ERR(os04c10->reset_gpio))
		gpiod_direction_output(os04c10->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(os04c10->pins_sleep)) {
		ret = pinctrl_select_state(os04c10->pinctrl,
									os04c10->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(OS04C10_NUM_SUPPLIES, os04c10->supplies);
}

static int os04c10_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct os04c10 *os04c10 = to_os04c10(sd);

	return __os04c10_power_on(os04c10);
}

static int os04c10_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct os04c10 *os04c10 = to_os04c10(sd);

	__os04c10_power_off(os04c10);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int os04c10_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct os04c10 *os04c10 = to_os04c10(sd);
	struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct os04c10_mode *def_mode = &supported_modes[0];

	mutex_lock(&os04c10->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&os04c10->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int os04c10_enum_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct os04c10 *os04c10 = to_os04c10(sd);

	if (fie->index >= os04c10->cfg_num)
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops os04c10_pm_ops = {
	SET_RUNTIME_PM_OPS(os04c10_runtime_suspend,
	os04c10_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops os04c10_internal_ops = {
	.open = os04c10_open,
};
#endif

static const struct v4l2_subdev_core_ops os04c10_core_ops = {
	.s_power = os04c10_s_power,
	.ioctl = os04c10_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = os04c10_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops os04c10_video_ops = {
	.s_stream = os04c10_s_stream,
	.g_frame_interval = os04c10_g_frame_interval,
	.g_mbus_config = os04c10_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops os04c10_pad_ops = {
	.enum_mbus_code = os04c10_enum_mbus_code,
	.enum_frame_size = os04c10_enum_frame_sizes,
	.enum_frame_interval = os04c10_enum_frame_interval,
	.get_fmt = os04c10_get_fmt,
	.set_fmt = os04c10_set_fmt,
};

static const struct v4l2_subdev_ops os04c10_subdev_ops = {
	.core	= &os04c10_core_ops,
	.video	= &os04c10_video_ops,
	.pad	= &os04c10_pad_ops,
};

static int os04c10_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct os04c10 *os04c10 = container_of(ctrl->handler, struct os04c10, ctrl_handler);
	struct i2c_client *client = os04c10->client;
	s64 max;
	int ret = 0;
	u32 again = 0, dgain = 0;
	u32 flip = 0, analog_flip = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = os04c10->cur_mode->height + ctrl->val - 8;
		__v4l2_ctrl_modify_range(os04c10->exposure, os04c10->exposure->minimum, max,
					 os04c10->exposure->step, os04c10->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = os04c10_write_reg(os04c10->client,
					OS04C10_REG_EXP_LONG_H,
					OS04C10_REG_VALUE_16BIT,
					ctrl->val);
		dev_dbg(&client->dev, "set exposure 0x%x\n", ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		if (ctrl->val > 1984) {
			dgain = ctrl->val * 1024 / 1984;
			again = 1984;
		} else {
			dgain = 1024;
			again = ctrl->val;
		}
		ret = os04c10_write_reg(os04c10->client,
					OS04C10_REG_AGAIN_LONG_H,
					OS04C10_REG_VALUE_16BIT,
					again & 0x1fff);
		ret |= os04c10_write_reg(os04c10->client,
					OS04C10_REG_DGAIN_LONG_H,
					OS04C10_REG_VALUE_16BIT,
					dgain & 0x3fff);
		dev_dbg(&client->dev, "set analog gain 0x%x\n", ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = os04c10_write_reg(os04c10->client, OS04C10_REG_VTS,
					OS04C10_REG_VALUE_16BIT,
					ctrl->val + os04c10->cur_mode->height);
		dev_dbg(&client->dev, "set vblank 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		break;
	case V4L2_CID_HFLIP:
		ret = os04c10_read_reg(os04c10->client, OS04C10_FLIP_REG,
					OS04C10_REG_VALUE_08BIT,
					&flip);
		if (ctrl->val)
			flip |= MIRROR_BIT_MASK;
		else
			flip &= ~MIRROR_BIT_MASK;
		ret |= os04c10_write_reg(os04c10->client, OS04C10_FLIP_REG,
					OS04C10_REG_VALUE_08BIT,
					flip);
		if (ret == 0)
			os04c10->flip = flip;
		break;
	case V4L2_CID_VFLIP:
		ret = os04c10_read_reg(os04c10->client, OS04C10_ANALOG_FLIP_REG,
					OS04C10_REG_VALUE_08BIT, &analog_flip);
		ret |= os04c10_read_reg(os04c10->client, OS04C10_FLIP_REG,
					OS04C10_REG_VALUE_08BIT, &flip);
		if (ctrl->val) {
			flip |= FLIP_BIT_MASK;
			analog_flip &= ~ANALOG_FLIP_BIT_MASK;
		} else {
			flip &= ~FLIP_BIT_MASK;
			analog_flip |= ANALOG_FLIP_BIT_MASK;
		}
		ret |= os04c10_write_reg(os04c10->client, OS04C10_FLIP_REG,
					OS04C10_REG_VALUE_08BIT, flip);
		ret |= os04c10_write_reg(os04c10->client, OS04C10_ANALOG_FLIP_REG,
					 OS04C10_REG_VALUE_08BIT, analog_flip);
		if (ret == 0)
			os04c10->flip = flip;
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}
	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops os04c10_ctrl_ops = {
	.s_ctrl = os04c10_set_ctrl,
};

static int os04c10_initialize_controls(struct os04c10 *os04c10)
{
	const struct os04c10_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 dst_link_freq = 0;
	u64 dst_pixel_rate = 0;

	handler = &os04c10->ctrl_handler;
	mode = os04c10->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &os04c10->mutex;

	os04c10->link_freq = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
						    1, 0, link_freq_menu_items);

	if (os04c10->cur_mode->bus_fmt == MEDIA_BUS_FMT_SBGGR10_1X10) {
		dst_link_freq = 0;
		dst_pixel_rate = PIXEL_RATE_WITH_384M;
	}
	/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
	os04c10->pixel_rate = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
						0, PIXEL_RATE_WITH_384M, 1, dst_pixel_rate);

	__v4l2_ctrl_s_ctrl(os04c10->link_freq, dst_link_freq);

	h_blank = mode->hts_def - mode->width;
	os04c10->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (os04c10->hblank)
		os04c10->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	os04c10->vblank = v4l2_ctrl_new_std(handler, &os04c10_ctrl_ops,
					    V4L2_CID_VBLANK, vblank_def,
					    OS04C10_VTS_MAX - mode->height,
					    1, vblank_def);

	exposure_max = mode->vts_def - 8;
	os04c10->exposure = v4l2_ctrl_new_std(handler, &os04c10_ctrl_ops,
					      V4L2_CID_EXPOSURE, OS04C10_EXPOSURE_MIN,
					      exposure_max, OS04C10_EXPOSURE_STEP,
					      mode->exp_def);

	os04c10->anal_gain = v4l2_ctrl_new_std(handler, &os04c10_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN, OS04C10_GAIN_MIN,
					       OS04C10_GAIN_MAX, OS04C10_GAIN_STEP,
					       OS04C10_GAIN_DEFAULT);

	os04c10->h_flip = v4l2_ctrl_new_std(handler, &os04c10_ctrl_ops,
					    V4L2_CID_HFLIP, 0, 1, 1, 0);

	os04c10->v_flip = v4l2_ctrl_new_std(handler, &os04c10_ctrl_ops,
					    V4L2_CID_VFLIP, 0, 1, 1, 0);
	os04c10->flip = 0;
	if (handler->error) {
		ret = handler->error;
		dev_err(&os04c10->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	os04c10->subdev.ctrl_handler = handler;
	os04c10->has_init_exp = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int os04c10_check_sensor_id(struct os04c10 *os04c10, struct i2c_client *client)
{
	struct device *dev = &os04c10->client->dev;
	u32 id = 0;
	int ret;

	if (os04c10->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = os04c10_read_reg(client, OS04C10_REG_CHIP_ID,
			       OS04C10_REG_VALUE_24BIT, &id);

	if (id != OS04C10_CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}
	dev_info(dev, "Detected OV%06x sensor\n", OS04C10_CHIP_ID);

	return 0;
}

static int os04c10_configure_regulators(struct os04c10 *os04c10)
{
	unsigned int i;

	for (i = 0; i < OS04C10_NUM_SUPPLIES; i++)
		os04c10->supplies[i].supply = OS04C10_supply_names[i];

	return devm_regulator_bulk_get(&os04c10->client->dev,
									OS04C10_NUM_SUPPLIES,
									os04c10->supplies);
}

static int os04c10_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct os04c10 *os04c10;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	u32 i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x",
				DRIVER_VERSION >> 16,
				(DRIVER_VERSION & 0xff00) >> 8,
				DRIVER_VERSION & 0x00ff);

	os04c10 = devm_kzalloc(dev, sizeof(*os04c10), GFP_KERNEL);
	if (!os04c10)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX, &os04c10->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
									&os04c10->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME, &os04c10->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME, &os04c10->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	if (ret) {
		hdr_mode = NO_HDR;
		dev_warn(dev, " Get hdr mode failed! no hdr default\n");
	}
	os04c10->cfg_num = ARRAY_SIZE(supported_modes);
	for (i = 0; i < os04c10->cfg_num; i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			os04c10->cur_mode = &supported_modes[i];
			break;
		}
	}
	os04c10->client = client;

	os04c10->is_thunderboot = IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP);

	os04c10->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(os04c10->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	os04c10->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(os04c10->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	os04c10->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_ASIS);
	if (IS_ERR(os04c10->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	os04c10->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(os04c10->pinctrl)) {
		os04c10->pins_default =
				pinctrl_lookup_state(os04c10->pinctrl,
						     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(os04c10->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		os04c10->pins_sleep =
				pinctrl_lookup_state(os04c10->pinctrl,
						     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(os04c10->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = os04c10_configure_regulators(os04c10);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&os04c10->mutex);

	sd = &os04c10->subdev;
	v4l2_i2c_subdev_init(sd, client, &os04c10_subdev_ops);
	ret = os04c10_initialize_controls(os04c10);
	if (ret)
		goto err_destroy_mutex;

	ret = __os04c10_power_on(os04c10);
	if (ret)
		goto err_free_handler;

	ret = os04c10_check_sensor_id(os04c10, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &os04c10_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	os04c10->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &os04c10->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(os04c10->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
				os04c10->module_index, facing,
				OS04C10_NAME, dev_name(sd->dev));
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
	__os04c10_power_off(os04c10);
err_free_handler:
	v4l2_ctrl_handler_free(&os04c10->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&os04c10->mutex);

	return ret;
}

static int os04c10_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct os04c10 *os04c10 = to_os04c10(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&os04c10->ctrl_handler);
	mutex_destroy(&os04c10->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__os04c10_power_off(os04c10);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id os04c10_of_match[] = {
	{ .compatible = "ovti,os04c10" },
	{},
};
MODULE_DEVICE_TABLE(of, os04c10_of_match);
#endif

static const struct i2c_device_id os04c10_match_id[] = {
	{ "ovti,os04c10", 0 },
	{ },
};

static struct i2c_driver os04c10_i2c_driver = {
	.driver = {
		.name = OS04C10_NAME,
		.pm = &os04c10_pm_ops,
		.of_match_table = of_match_ptr(os04c10_of_match),
	},
	.probe		= &os04c10_probe,
	.remove		= &os04c10_remove,
	.id_table	= os04c10_match_id,
};

#ifdef CONFIG_ROCKCHIP_THUNDER_BOOT
module_i2c_driver(os04c10_i2c_driver);
#else
static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&os04c10_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&os04c10_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);
#endif

MODULE_DESCRIPTION("OmniVision os04c10 sensor driver");
MODULE_LICENSE("GPL v2");
