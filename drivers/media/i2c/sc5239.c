// SPDX-License-Identifier: GPL-2.0
/*
 * sc5239 sensor driver
 *
 * Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version.
 */

//#define DEBUG
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <linux/rk-preisp.h>
#include <media/v4l2-async.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define DRIVER_VERSION		KERNEL_VERSION(0, 0x01, 0x00)

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define SC5239_NAME		"sc5239"
#define SC5239_MEDIA_BUS_FMT	MEDIA_BUS_FMT_SBGGR10_1X10
#define SC5239_LINK_FREQ_600M	300000000
#define SC5239_LINK_FREQ_828M	414000000

#define SC5239_XVCLK_FREQ	24000000

#define SC5239_REG_CHIP_ID_H	0x3107
#define SC5239_REG_CHIP_ID_L	0x3108

#define SC5239_REG_EXP_LONG_H   0x3e01
#define SC5239_REG_EXP_LONG_L	0x3e02
#define SC5239_REG_EXP_SHORT_H	0x3e04
#define SC5239_REG_EXP_SHORT_L	0x3e05

#define SC5239_REG_LONG_DGAIN		0x3e06
#define SC5239_REG_LONG_FINE_DGAIN	0x3e07
#define SC5239_REG_LONG_AGAIN		0x3e08
#define SC5239_REG_LONG_FINE_AGAIN	0x3e09

#define SC5239_REG_SHORT_DGAIN		0x3e10
#define SC5239_REG_SHORT_FINE_DGAIN	0x3e11
#define SC5239_REG_SHORT_AGAIN		0x3e12
#define SC5239_REG_SHORT_FINE_AGAIN	0x3e13

#define SC5239_FLIP_MIRROR_REG	0x3221
#define SC5239_MIRROR_MASK		0x06
#define SC5239_FLIP_MASK		0xe0

#define SC5239_REG_CTRL_MODE	0x0100
#define SC5239_MODE_SW_STANDBY	0x0
#define SC5239_MODE_STREAMING	BIT(0)

#define SC5239_CHIP_ID		0x5235

#define SC5239_REG_VTS_H	0x320e
#define SC5239_REG_VTS_L	0x320f

#define SC5239_VTS_MAX		0x3FFF
#define SC5239_HTS_MAX		0x0FFF

#define SC5239_EXPOSURE_NORMAL_MAX  0x118A
#define SC5239_EXPOSURE_NORMAL_MIN  3
#define SC5239_EXPOSURE_NORMAL_STEP 1

#define SC5239_GAIN_MIN		0x20
#define SC5239_GAIN_MAX		0x4000
#define SC5239_GAIN_STEP	1
#define SC5239_GAIN_DEFAULT	0x20

#define SC5239_LANES		2

static const char * const sc5239_supply_names[] = {
	"dovdd",    /* Digital I/O power */
	"avdd",     /* Analog power */
	"dvdd",     /* Digital power */
};

#define SC5239_NUM_SUPPLIES ARRAY_SIZE(sc5239_supply_names)

#define to_sc5239(sd) container_of(sd, struct sc5239, subdev)

enum {
	PAD0,
	PAD1,
	PAD2,
	PAD3,
	PAD_MAX,
};

enum {
	LINK_FREQ_INDEX,
	LINK_FREQ_15FPS_24MHz_HDR,
};

struct sc5239_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 link_freq_index;
	u32 bpp;
	const struct reg_sequence *reg_list;
	u32 reg_num;
	u32 hdr_mode;
	u32 vc[PAD_MAX];
};

struct sc5239 {
	struct i2c_client	*client;
	struct device	*dev;
	struct clk	*xvclk;
	struct regmap	*regmap;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;
	struct regulator_bulk_data supplies[SC5239_NUM_SUPPLIES];
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;
	struct v4l2_subdev  subdev;
	struct media_pad    pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl    *exposure;
	struct v4l2_ctrl    *anal_gain;
	struct v4l2_ctrl    *hblank;
	struct v4l2_ctrl    *vblank;
	struct v4l2_ctrl    *h_flip;
	struct v4l2_ctrl    *v_flip;
	struct v4l2_ctrl    *link_freq;
	struct v4l2_ctrl    *pixel_rate;
	struct mutex        lock;
	bool		    streaming;
	bool		    power_on;
	unsigned int        cfg_num;
	const struct sc5239_mode *cur_mode;
	u32		module_index;
	const char      *module_facing;
	const char      *module_name;
	const char      *len_name;
	bool			  has_init_exp;
	struct preisp_hdrae_exp_s init_hdrae_exp;
};

static const struct regmap_config sc5239_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xff00,
};

static const s64 link_freq_menu_items[] = {
	SC5239_LINK_FREQ_600M,
	SC5239_LINK_FREQ_828M,
};

/*
 * window size=2560*1920 mipi@2lane
 * mclk=24M mipi_clk=600Mbps
 * pixel_line_total=xxxx line_frame_total=2256
 * row_time=60us frame_rate=15fps
 */
static const struct reg_sequence sc5239_24M_2560_1920_liner_15fps_settings[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x3039, 0x80},
	{0x3029, 0x80},
	{0x302a, 0x36},
	{0x302b, 0x10},
	{0x302c, 0x00},
	{0x302d, 0x03},
	{0x3038, 0x44},
	{0x303a, 0x3b},
	{0x303d, 0x20},
	{0x3200, 0x00},
	{0x3201, 0x14},
	{0x3202, 0x00},
	{0x3203, 0x10},
	{0x3204, 0x0a},
	{0x3205, 0x1c},
	{0x3206, 0x07},
	{0x3207, 0x97},
	{0x3208, 0x0a},
	{0x3209, 0x00},
	{0x320a, 0x07},
	{0x320b, 0x80},
	{0x320c, 0x05},
	{0x320d, 0xdc},
	{0x320e, 0x0a},
	{0x320f, 0x6a},
	{0x3211, 0x05},
	{0x3213, 0x04},
	{0x3221, 0x00},
	{0x3235, 0x0f},
	{0x3236, 0x9e},
	{0x3301, 0x1c},
	{0x3303, 0x28},
	{0x3304, 0x10},
	{0x3306, 0x50},
	{0x3308, 0x10},
	{0x3309, 0x70},
	{0x330a, 0x00},
	{0x330b, 0xb8},
	{0x330e, 0x20},
	{0x3314, 0x14},
	{0x3315, 0x02},
	{0x331b, 0x83},
	{0x331e, 0x19},
	{0x331f, 0x61},
	{0x3320, 0x01},
	{0x3321, 0x04},
	{0x3326, 0x00},
	{0x3333, 0x20},
	{0x3334, 0x40},
	{0x3364, 0x05},
	{0x3366, 0x78},
	{0x3367, 0x08},
	{0x3368, 0x03},
	{0x3369, 0x00},
	{0x336a, 0x00},
	{0x336b, 0x00},
	{0x336c, 0x01},
	{0x336d, 0x40},
	{0x337f, 0x03},
	{0x338f, 0x40},
	{0x33b6, 0x07},
	{0x33b7, 0x17},
	{0x33b8, 0x20},
	{0x33b9, 0x20},
	{0x33ba, 0x44},
	{0x3620, 0x28},
	{0x3621, 0xac},
	{0x3622, 0xf6},
	{0x3623, 0x10},
	{0x3624, 0x47},
	{0x3625, 0x0b},
	{0x3630, 0x30},
	{0x3631, 0x88},
	{0x3632, 0x18},
	{0x3633, 0x23},
	{0x3634, 0x86},
	{0x3635, 0x4d},
	{0x3636, 0x21},
	{0x3637, 0x20},
	{0x3638, 0x18},
	{0x3639, 0x09},
	{0x363a, 0x83},
	{0x363b, 0x02},
	{0x363c, 0x07},
	{0x363d, 0x03},
	{0x3670, 0x00},
	{0x3677, 0x86},
	{0x3678, 0x86},
	{0x3679, 0xa8},
	{0x367e, 0x08},
	{0x367f, 0x18},
	{0x3802, 0x00},
	{0x3905, 0x98},
	{0x3907, 0x01},
	{0x3908, 0x11},
	{0x390a, 0x00},
	{0x391c, 0x9f},
	{0x391d, 0x00},
	{0x391e, 0x01},
	{0x391f, 0xc0},
	{0x3e00, 0x00},
	{0x3e01, 0xf9},
	{0x3e02, 0x80},
	{0x3e03, 0x0b},
	{0x3e06, 0x00},
	{0x3e07, 0x80},
	{0x3e08, 0x03},
	{0x3e09, 0x20},
	{0x3e1e, 0x30},
	{0x3e26, 0x20},
	{0x3f00, 0x0d},
	{0x3f04, 0x02},
	{0x3f05, 0xe6},
	{0x3f08, 0x04},
	{0x4500, 0x5d},
	{0x4509, 0x10},
	{0x4809, 0x01},
	{0x4837, 0x21},
	{0x5000, 0x06},
	{0x5002, 0x06},
	{0x5780, 0x7f},
	{0x5781, 0x06},
	{0x5782, 0x04},
	{0x5783, 0x00},
	{0x5784, 0x00},
	{0x5785, 0x16},
	{0x5786, 0x12},
	{0x5787, 0x08},
	{0x5788, 0x02},
	{0x578b, 0x07},
	{0x57a0, 0x00},
	{0x57a1, 0x72},
	{0x57a2, 0x01},
	{0x57a3, 0xf2},
	{0x6000, 0x20},
	{0x6002, 0x00},
	{0x3039, 0x24},
	{0x3029, 0x27},
};

/*
 * window size=2560*1920 mipi@2lane
 * mclk=24M mipi_clk=600Mbps
 * pixel_line_total= line_frame_total=2256
 * row_time=60us frame_rate=15fps
 */
static const struct reg_sequence sc5239_24M_2560_1920_hdr_15fps_settings[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x3039, 0x80},
	{0x3029, 0x80},
	{0x301f, 0x1b},
	{0x302a, 0x69},
	{0x302b, 0x01},
	{0x302c, 0x00},
	{0x302d, 0x03},
	{0x3037, 0x26},
	{0x3038, 0x66},
	{0x303a, 0x29},
	{0x303b, 0x0a},
	{0x303c, 0x0e},
	{0x303d, 0x03},
	{0x3200, 0x00},
	{0x3201, 0x10},
	{0x3202, 0x00},
	{0x3203, 0x0c},
	{0x3204, 0x0a},
	{0x3205, 0x1f},
	{0x3206, 0x07},
	{0x3207, 0x93},
	{0x3208, 0x0a},
	{0x3209, 0x00},
	{0x320a, 0x07},
	{0x320b, 0x80},
	{0x320c, 0x05},
	{0x320d, 0x64},
	{0x320e, 0x0f},
	{0x320f, 0xa0},
	{0x3210, 0x00},
	{0x3211, 0x08},
	{0x3212, 0x00},
	{0x3213, 0x04},
	{0x3220, 0x50},
	{0x3221, 0x00},
	{0x3235, 0x1f},
	{0x3236, 0x3e},
	{0x3301, 0x38},
	{0x3303, 0x20},
	{0x3304, 0x10},
	{0x3306, 0x58},
	{0x3308, 0x10},
	{0x3309, 0x60},
	{0x330a, 0x00},
	{0x330b, 0xb8},
	{0x330d, 0x30},
	{0x330e, 0x20},
	{0x3314, 0x14},
	{0x3315, 0x02},
	{0x331b, 0x83},
	{0x331e, 0x19},
	{0x331f, 0x59},
	{0x3320, 0x01},
	{0x3321, 0x04},
	{0x3326, 0x00},
	{0x3332, 0x22},
	{0x3333, 0x20},
	{0x3334, 0x40},
	{0x3350, 0x22},
	{0x3359, 0x22},
	{0x335c, 0x22},
	{0x3364, 0x05},
	{0x3366, 0xc8},
	{0x3367, 0x08},
	{0x3368, 0x03},
	{0x3369, 0x00},
	{0x336a, 0x00},
	{0x336b, 0x00},
	{0x336c, 0x01},
	{0x336d, 0x40},
	{0x337e, 0x88},
	{0x337f, 0x03},
	{0x338f, 0x40},
	{0x33ae, 0x22},
	{0x33af, 0x22},
	{0x33b0, 0x22},
	{0x33b4, 0x22},
	{0x33b6, 0x07},
	{0x33b7, 0x17},
	{0x33b8, 0x20},
	{0x33b9, 0x20},
	{0x33ba, 0x44},
	{0x3614, 0x00},
	{0x3620, 0x28},
	{0x3621, 0xac},
	{0x3622, 0xf6},
	{0x3623, 0x08},
	{0x3624, 0x47},
	{0x3625, 0x0b},
	{0x3630, 0x30},
	{0x3631, 0x88},
	{0x3632, 0x18},
	{0x3633, 0x34},
	{0x3634, 0x86},
	{0x3635, 0x4d},
	{0x3636, 0x21},
	{0x3637, 0x20},
	{0x3638, 0x18},
	{0x3639, 0x09},
	{0x363a, 0x83},
	{0x363b, 0x02},
	{0x363c, 0x07},
	{0x363d, 0x03},
	{0x3670, 0x00},
	{0x3677, 0x86},
	{0x3678, 0x86},
	{0x3679, 0xa8},
	{0x367e, 0x08},
	{0x367f, 0x18},
	{0x3905, 0x98},
	{0x3907, 0x01},
	{0x3908, 0x11},
	{0x390a, 0x00},
	{0x391c, 0x9f},
	{0x391d, 0x00},
	{0x391e, 0x01},
	{0x391f, 0xc0},
	{0x3988, 0x11},
	{0x3e00, 0x01},
	{0x3e01, 0xd4},
	{0x3e02, 0xe0},
	{0x3e03, 0x0b},
	{0x3e04, 0x1d},
	{0x3e05, 0xe0},
	{0x3e06, 0x00},
	{0x3e07, 0x80},
	{0x3e08, 0x03},
	{0x3e09, 0x20},
	{0x3e1e, 0x30},
	{0x3e23, 0x00},
	{0x3e24, 0xf2},
	{0x3e26, 0x20},
	{0x3f00, 0x0d},
	{0x3f02, 0x05},
	{0x3f04, 0x02},
	{0x3f05, 0xaa},
	{0x3f06, 0x21},
	{0x3f08, 0x04},
	{0x4500, 0x5d},
	{0x4502, 0x10},
	{0x4509, 0x10},
	{0x4602, 0x0f},
	{0x4809, 0x01},
	{0x4816, 0x51},
	{0x4837, 0x19},
	{0x5000, 0x20},
	{0x5002, 0x00},
	{0x6000, 0x26},
	{0x6002, 0x06},
	{0x3039, 0x23},
	{0x3029, 0x33},
};

static const struct sc5239_mode supported_modes[] = {
	{
		.width = 2560,
		.height = 1920,
		.max_fps = {
			.numerator = 10000,
			.denominator = 150000,
		},
		.exp_def = 0x08f9,
		.hts_def = 0x05dc,
		.vts_def = 0x0a6a,
		.link_freq_index = LINK_FREQ_INDEX,
		.bus_fmt = SC5239_MEDIA_BUS_FMT,
		.reg_list = sc5239_24M_2560_1920_liner_15fps_settings,
		.reg_num = ARRAY_SIZE(sc5239_24M_2560_1920_liner_15fps_settings),
		.hdr_mode = NO_HDR,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		.width = 2560,
		.height = 1920,
		.max_fps = {
			.numerator = 10000,
			.denominator = 150000,
		},
		.exp_def = 0x0ed4,
		.hts_def = 0x05dc,
		.vts_def = 0x0fa0,
		.link_freq_index = LINK_FREQ_15FPS_24MHz_HDR,
		.bus_fmt = SC5239_MEDIA_BUS_FMT,
		.reg_list = sc5239_24M_2560_1920_hdr_15fps_settings,
		.reg_num = ARRAY_SIZE(sc5239_24M_2560_1920_hdr_15fps_settings),
		.hdr_mode = HDR_X2,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_0,
		.vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_1,
	},
};

/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
static u64 to_pixel_rate(u32 index)
{
	u64 pixel_rate = link_freq_menu_items[index] * 2 * SC5239_LANES;

	do_div(pixel_rate, 10);
	return pixel_rate;
}

static inline int sc5239_read_reg(struct sc5239 *sc5239, u16 addr, u8 *value)
{
	unsigned int val;
	int ret;

	ret = regmap_read(sc5239->regmap, addr, &val);
	if (ret) {
		dev_err(sc5239->dev, "i2c read failed at addr: %x\n", addr);
		return ret;
	}
	*value = val & 0xff;
	return 0;
}

static inline int sc5239_write_reg(struct sc5239 *sc5239, u16 addr, u8 value)
{
	int ret;

	ret = regmap_write(sc5239->regmap, addr, value);
	if (ret) {
		dev_err(sc5239->dev, "i2c write failed at addr: %x\n", addr);
		return ret;
	}
	return ret;
}

static void sc5239_get_gain_reg(u32 total_gain, u32 *again, u32 *again_fine,
					 u32 *dgain, u32 *dgain_fine)
{
	if (total_gain < 0x40) {/* 1 ~ 2 gain */
		*again = 0x03;
		*again_fine = total_gain;
		*dgain = 0x00;
		*dgain_fine = 0x80;
	} else if (total_gain < 0x80) {/* 2 ~ 4 gain */
		*again = 0x07;
		*again_fine = total_gain >> 1;
		*dgain = 0x00;
		*dgain_fine = 0x80;
	} else if (total_gain < 0x100) {/* 4 ~ 8 gain */
		*again = 0x0F;
		*again_fine = total_gain >> 2;
		*dgain = 0x00;
		*dgain_fine = 0x80;
	} else if (total_gain < 0x200) {/* 8 ~ 16 gain */
		*again = 0x01F;
		*again_fine = total_gain >> 3;
		*dgain = 0x00;
		*dgain_fine = 0x80;
	} else if (total_gain < 0x400) {/* 16 ~ 32 gain */
		*again = 0x1F;
		*again_fine = 0x3F;
		*dgain = 0x00;
		*dgain_fine = total_gain >> 2;
	} else if (total_gain < 0x800) {/* 32 ~ 64 gain */
		*again = 0x1F;
		*again_fine = 0x3F;
		*dgain = 0x01;
		*dgain_fine = total_gain >> 3;
	} else if (total_gain < 0x1000) {/* 64 ~ 128 gain */
		*again = 0x1F;
		*dgain_fine = 0x3F;
		*dgain = 0x03;
		*dgain_fine = total_gain >> 4;
	} else if (total_gain < 0x2000) {/* 128 ~ 256 gain */
		*again = 0x1F;
		*again_fine = 0x3F;
		*dgain = 0x07;
		*dgain_fine = total_gain >> 5;
	} else if (total_gain < 0x4000) {/* 256 ~ 512 gain */
		*again = 0x1F;
		*again_fine = 0x3F;
		*dgain = 0x0F;
		*dgain_fine = total_gain >> 6;
	}
}

static int sc5239_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc5239 *sc5239 = container_of(ctrl->handler,
					     struct sc5239, ctrl_handler);
	s64 max;
	u32 again = 0, again_fine = 0, dgain = 0, dgain_fine = 0;
	int ret = 0;
	u32 vts = 0;
	u8 val = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = ((sc5239->cur_mode->height + ctrl->val) << 1) - 4;
		__v4l2_ctrl_modify_range(sc5239->exposure,
					 sc5239->exposure->minimum, max,
					 sc5239->exposure->step,
					 sc5239->exposure->default_value);
		break;
	}

	if (pm_runtime_get(sc5239->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		if (sc5239->cur_mode->hdr_mode != NO_HDR)
			return ret;
		dev_dbg(sc5239->dev, "set exposure 0x%x\n", ctrl->val);
		ret |= sc5239_write_reg(sc5239, SC5239_REG_EXP_LONG_H, ctrl->val >> 4);
		ret |= sc5239_write_reg(sc5239, SC5239_REG_EXP_LONG_L, ctrl->val << 4);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		if (sc5239->cur_mode->hdr_mode != NO_HDR)
			return ret;
		dev_dbg(sc5239->dev, "set again 0x%x\n", ctrl->val);
		sc5239_get_gain_reg(ctrl->val, &again, &again_fine, &dgain, &dgain_fine);
		ret |= sc5239_write_reg(sc5239, SC5239_REG_LONG_DGAIN, dgain);
		ret |= sc5239_write_reg(sc5239, SC5239_REG_LONG_FINE_DGAIN, dgain_fine);
		ret |= sc5239_write_reg(sc5239, SC5239_REG_LONG_AGAIN, again);
		ret |= sc5239_write_reg(sc5239, SC5239_REG_LONG_FINE_AGAIN, again_fine);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(sc5239->dev, "set vblank 0x%x\n", ctrl->val);
		vts = ctrl->val + sc5239->cur_mode->height;
		ret |= sc5239_write_reg(sc5239, SC5239_REG_VTS_H, (vts >> 8) & 0xff);
		ret |= sc5239_write_reg(sc5239, SC5239_REG_VTS_L, vts & 0xff);
		break;
	case V4L2_CID_HFLIP:
		ret |= sc5239_read_reg(sc5239, SC5239_FLIP_MIRROR_REG, &val);
		if (ret)
			break;

		if (ctrl->val)
			val |= SC5239_MIRROR_MASK;
		else
			val &= ~SC5239_MIRROR_MASK;
		ret |= sc5239_write_reg(sc5239, SC5239_FLIP_MIRROR_REG, val);
		break;
	case V4L2_CID_VFLIP:
		ret |= sc5239_read_reg(sc5239, SC5239_FLIP_MIRROR_REG, &val);
		if (ret)
			break;

		if (ctrl->val)
			val |= SC5239_FLIP_MASK;
		else
			val &= ~SC5239_FLIP_MASK;
		ret |= sc5239_write_reg(sc5239, SC5239_FLIP_MIRROR_REG, val);
		break;
	default:
		dev_warn(sc5239->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}
	pm_runtime_put(sc5239->dev);
	return ret;
}

static const struct v4l2_ctrl_ops sc5239_ctrl_ops = {
	.s_ctrl = sc5239_set_ctrl,
};

static int sc5239_get_regulators(struct sc5239 *sc5239)
{
	unsigned int i;

	for (i = 0; i < SC5239_NUM_SUPPLIES; i++)
		sc5239->supplies[i].supply = sc5239_supply_names[i];
	return devm_regulator_bulk_get(sc5239->dev,
				       SC5239_NUM_SUPPLIES,
				       sc5239->supplies);
}

static int sc5239_initialize_controls(struct sc5239 *sc5239)
{
	const struct sc5239_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &sc5239->ctrl_handler;
	mode = sc5239->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &sc5239->lock;
	sc5239->link_freq = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
						  ARRAY_SIZE(link_freq_menu_items) - 1, 0,
						  link_freq_menu_items);

	sc5239->pixel_rate = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
					      0, to_pixel_rate(LINK_FREQ_INDEX),
					      1, to_pixel_rate(LINK_FREQ_INDEX));
	h_blank = mode->hts_def - mode->width;

	sc5239->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					  h_blank, h_blank, 1, h_blank);
	if (sc5239->hblank)
		sc5239->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;

	sc5239->vblank = v4l2_ctrl_new_std(handler, &sc5239_ctrl_ops,
					  V4L2_CID_VBLANK, vblank_def,
					  SC5239_VTS_MAX - mode->height,
					  1, vblank_def);

	exposure_max = (2 * mode->vts_def - 8);

	sc5239->exposure = v4l2_ctrl_new_std(handler, &sc5239_ctrl_ops,
					    V4L2_CID_EXPOSURE, SC5239_EXPOSURE_NORMAL_MIN,
					    exposure_max, SC5239_EXPOSURE_NORMAL_STEP,
					    mode->exp_def);

	sc5239->anal_gain = v4l2_ctrl_new_std(handler, &sc5239_ctrl_ops,
					     V4L2_CID_ANALOGUE_GAIN, SC5239_GAIN_MIN,
					     SC5239_GAIN_MAX, SC5239_GAIN_STEP,
					     SC5239_GAIN_DEFAULT);

	v4l2_ctrl_new_std(handler, &sc5239_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std(handler, &sc5239_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (handler->error) {
		ret = handler->error;
		dev_err(sc5239->dev, "Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	sc5239->subdev.ctrl_handler = handler;
	sc5239->has_init_exp = false;

	return 0;
err_free_handler:
	v4l2_ctrl_handler_free(handler);
	return ret;
}

static int __sc5239_power_on(struct sc5239 *sc5239)
{
	int ret;
	struct device *dev = sc5239->dev;

	if (!IS_ERR_OR_NULL(sc5239->pins_default)) {
		ret = pinctrl_select_state(sc5239->pinctrl,
					   sc5239->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(sc5239->xvclk, SC5239_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate\n");
	if (clk_get_rate(sc5239->xvclk) != SC5239_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 27MHz\n");
	ret = clk_prepare_enable(sc5239->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	ret = regulator_bulk_enable(SC5239_NUM_SUPPLIES, sc5239->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}
	if (!IS_ERR(sc5239->reset_gpio))
		gpiod_set_value_cansleep(sc5239->reset_gpio, 1);
	usleep_range(1000, 2000);
	if (!IS_ERR(sc5239->pwdn_gpio))
		gpiod_set_value_cansleep(sc5239->pwdn_gpio, 1);
	if (!IS_ERR(sc5239->reset_gpio))
		gpiod_set_value_cansleep(sc5239->reset_gpio, 0);

	return 0;
disable_clk:
	clk_disable_unprepare(sc5239->xvclk);

	if (!IS_ERR_OR_NULL(sc5239->pins_sleep))
		pinctrl_select_state(sc5239->pinctrl, sc5239->pins_sleep);

	return ret;
}

static void __sc5239_power_off(struct sc5239 *sc5239)
{
	int ret;
	struct device *dev = sc5239->dev;

	if (!IS_ERR_OR_NULL(sc5239->pins_sleep)) {
		ret = pinctrl_select_state(sc5239->pinctrl,
					   sc5239->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	if (!IS_ERR(sc5239->reset_gpio))
		gpiod_set_value_cansleep(sc5239->reset_gpio, 1);
	if (!IS_ERR(sc5239->pwdn_gpio))
		gpiod_set_value_cansleep(sc5239->pwdn_gpio, 0);
	regulator_bulk_disable(SC5239_NUM_SUPPLIES, sc5239->supplies);
	clk_disable_unprepare(sc5239->xvclk);
}

static int sc5239_check_sensor_id(struct sc5239 *sc5239)
{
	u8 id_h = 0, id_l = 0;
	u16 id = 0;
	int ret = 0;

	ret = sc5239_read_reg(sc5239, SC5239_REG_CHIP_ID_H, &id_h);
	ret |= sc5239_read_reg(sc5239, SC5239_REG_CHIP_ID_L, &id_l);
	if (ret) {
		dev_err(sc5239->dev, "Failed to read sensor id, (%d)\n", ret);
		return ret;
	}
	id = id_h << 8 | id_l;
	if (id != SC5239_CHIP_ID) {
		dev_err(sc5239->dev, "sensor id: %04X mismatched\n", id);
		return -ENODEV;
	}
	dev_info(sc5239->dev, "Detected SC5239 sensor\n");

	return 0;
}

static void sc5239_get_module_inf(struct sc5239 *sc5239,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.lens, sc5239->len_name, sizeof(inf->base.lens));
	strscpy(inf->base.sensor, SC5239_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, sc5239->module_name, sizeof(inf->base.module));
}

static int sc5239_set_hdrae(struct sc5239 *sc5239,
			 struct preisp_hdrae_exp_s *ae)
{
	int ret = 0;
	u32 l_exp_time, m_exp_time, s_exp_time;
	u32 l_t_gain, m_t_gain, s_t_gain;
	u32 l_again = 0, l_again_fine = 0, l_dgain = 0, l_dgain_fine = 0;
	u32 s_again = 0, s_again_fine = 0, s_dgain = 0, s_dgain_fine = 0;

	if (!sc5239->has_init_exp && !sc5239->streaming) {
		sc5239->init_hdrae_exp = *ae;
		sc5239->has_init_exp = true;
		dev_dbg(&sc5239->client->dev, "sc5239 don't stream, record exp for hdr!\n");
		return ret;
	}

	l_exp_time = ae->long_exp_reg;
	m_exp_time = ae->middle_exp_reg;
	s_exp_time = ae->short_exp_reg;
	l_t_gain = ae->long_gain_reg;
	m_t_gain = ae->middle_gain_reg;
	s_t_gain = ae->short_gain_reg;

	dev_dbg(&sc5239->client->dev,
		"rev exp req: L_exp: 0x%x, M_exp: 0x%x, S_exp: 0x%x, L_tgain: 0x%x, M_tgain: 0x%x, S_tgain: 0x%x\n",
		l_exp_time, m_exp_time, s_exp_time,
		l_t_gain, m_t_gain, s_t_gain);

	if (sc5239->cur_mode->hdr_mode == HDR_X2) {
		//2 stagger
		l_t_gain = m_t_gain;
		l_exp_time = m_exp_time;
	}

	l_exp_time = l_exp_time << 1;
	s_exp_time = s_exp_time << 1;

	// set exposure reg
	ret |= sc5239_write_reg(sc5239, SC5239_REG_EXP_LONG_L, (l_exp_time << 4) & 0xff);
	ret |= sc5239_write_reg(sc5239, SC5239_REG_EXP_LONG_H, (l_exp_time >> 4));

	ret |= sc5239_write_reg(sc5239, SC5239_REG_EXP_SHORT_L, (s_exp_time << 4) & 0xff);
	ret |= sc5239_write_reg(sc5239, SC5239_REG_EXP_SHORT_H, (s_exp_time >> 4));

	// set gain reg
	sc5239_get_gain_reg(l_t_gain, &l_again, &l_again_fine, &l_dgain, &l_dgain_fine);
	sc5239_get_gain_reg(s_t_gain, &s_again, &s_again_fine, &s_dgain, &s_dgain_fine);

	ret |= sc5239_write_reg(sc5239, SC5239_REG_LONG_DGAIN, l_dgain);
	ret |= sc5239_write_reg(sc5239, SC5239_REG_LONG_FINE_DGAIN, l_dgain_fine);
	ret |= sc5239_write_reg(sc5239, SC5239_REG_LONG_AGAIN, l_again);
	ret |= sc5239_write_reg(sc5239, SC5239_REG_LONG_FINE_AGAIN, l_again_fine);

	ret |= sc5239_write_reg(sc5239, SC5239_REG_SHORT_DGAIN, s_dgain);
	ret |= sc5239_write_reg(sc5239, SC5239_REG_SHORT_FINE_DGAIN, s_dgain_fine);
	ret |= sc5239_write_reg(sc5239, SC5239_REG_SHORT_AGAIN, s_again);
	ret |= sc5239_write_reg(sc5239, SC5239_REG_SHORT_FINE_AGAIN, s_again_fine);

	return ret;
}

static long sc5239_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc5239 *sc5239 = to_sc5239(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;
	const struct sc5239_mode *mode;
	long ret = 0;
	u32 stream = 0;
	u32 i, h, w;
	u64 pixel_rate = 0;

	switch (cmd) {
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = sc5239->cur_mode->hdr_mode;
		break;
	case RKMODULE_GET_MODULE_INFO:
		sc5239_get_module_inf(sc5239, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		w = sc5239->cur_mode->width;
		h = sc5239->cur_mode->height;

		for (i = 0; i < sc5239->cfg_num; i++) {
			if (w == supported_modes[i].width &&
			h == supported_modes[i].height &&
			supported_modes[i].hdr_mode == hdr_cfg->hdr_mode) {
				sc5239->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == sc5239->cfg_num) {
			dev_err(sc5239->dev, "not find hdr mode:%d %dx%d config\n",
				hdr_cfg->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			mode = sc5239->cur_mode;
			w = mode->hts_def - mode->width;
			h = mode->vts_def - mode->height;
			__v4l2_ctrl_modify_range(sc5239->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(sc5239->vblank, h,
						 SC5239_VTS_MAX - mode->height, 1, h);

			__v4l2_ctrl_s_ctrl(sc5239->link_freq, mode->link_freq_index);
			pixel_rate = (u32)link_freq_menu_items[mode->link_freq_index]
							/ mode->bpp * 2 * SC5239_LANES;
			__v4l2_ctrl_s_ctrl_int64(sc5239->pixel_rate, pixel_rate);

			dev_info(sc5239->dev, "sensor mode: %d\n", mode->hdr_mode);
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		if (sc5239->cur_mode->hdr_mode == HDR_X2)
			ret = sc5239_set_hdrae(sc5239, arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);
		if (stream)
			ret = sc5239_write_reg(sc5239, SC5239_REG_CTRL_MODE,
				SC5239_MODE_STREAMING);
		else
			ret = sc5239_write_reg(sc5239, SC5239_REG_CTRL_MODE,
				SC5239_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

static int __sc5239_start_stream(struct sc5239 *sc5239)
{
	int ret;

	ret = regmap_multi_reg_write(sc5239->regmap,
				     sc5239->cur_mode->reg_list,
				     sc5239->cur_mode->reg_num);
	if (ret)
		return ret;
	__v4l2_ctrl_handler_setup(&sc5239->ctrl_handler);
	return sc5239_write_reg(sc5239, SC5239_REG_CTRL_MODE, SC5239_MODE_STREAMING);
}

static int __sc5239_stop_stream(struct sc5239 *sc5239)
{
	sc5239->has_init_exp = false;
	return sc5239_write_reg(sc5239, SC5239_REG_CTRL_MODE, SC5239_MODE_SW_STANDBY);
}

#ifdef CONFIG_COMPAT
static long sc5239_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	u32 stream = 0;
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}
		ret = sc5239_ioctl(sd, cmd, inf);
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
		ret = sc5239_ioctl(sd, cmd, hdr);
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

		if (copy_from_user(hdr, up, sizeof(*hdr))) {
			kfree(hdr);
			return -EFAULT;
		}

		ret = sc5239_ioctl(sd, cmd, hdr);

		kfree(hdr);
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		hdrae = kzalloc(sizeof(*hdrae), GFP_KERNEL);
		if (!hdrae) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(hdrae, up, sizeof(*hdrae))) {
			kfree(hdrae);
			return -EFAULT;
		}

		ret = sc5239_ioctl(sd, cmd, hdrae);

		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;

		ret = sc5239_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}
#endif

static int sc5239_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sc5239 *sc5239 = to_sc5239(sd);
	int ret = 0;

	mutex_lock(&sc5239->lock);
	on = !!on;
	if (on == sc5239->streaming)
		goto unlock_and_return;
	if (on) {
		ret = pm_runtime_get_sync(sc5239->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(sc5239->dev);
			goto unlock_and_return;
		}
		ret = __sc5239_start_stream(sc5239);
		if (ret) {
			dev_err(sc5239->dev, "Failed to start sc5239 stream\n");
			pm_runtime_put(sc5239->dev);
			goto unlock_and_return;
		}
	} else {
		__sc5239_stop_stream(sc5239);
		pm_runtime_put(sc5239->dev);
	}
	sc5239->streaming = on;
unlock_and_return:
	mutex_unlock(&sc5239->lock);
	return 0;
}

static int sc5239_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct sc5239 *sc5239 = to_sc5239(sd);
	const struct sc5239_mode *mode = sc5239->cur_mode;

	mutex_lock(&sc5239->lock);
	fi->interval = mode->max_fps;
	mutex_unlock(&sc5239->lock);
	return 0;
}

static int sc5239_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	struct sc5239 *sc5239 = to_sc5239(sd);

	u32 val = 1 << (SC5239_LANES - 1) | V4L2_MBUS_CSI2_CHANNEL_0 |
		  V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	config->type = V4L2_MBUS_CSI2;
	config->flags = (sc5239->cur_mode->hdr_mode == NO_HDR) ?
			val : (val | V4L2_MBUS_CSI2_CHANNEL_1);
	return 0;
}

static int sc5239_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = SC5239_MEDIA_BUS_FMT;
	return 0;
}

static int sc5239_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct sc5239 *sc5239 = to_sc5239(sd);

	if (fse->index >= sc5239->cfg_num)
		return -EINVAL;

	if (fse->code != SC5239_MEDIA_BUS_FMT)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;
	return 0;
}

static int sc5239_enum_frame_interval(struct v4l2_subdev *sd,
						  struct v4l2_subdev_pad_config *cfg,
						  struct v4l2_subdev_frame_interval_enum *fie)
{
	struct sc5239 *sc5239 = to_sc5239(sd);

	if (fie->index >= sc5239->cfg_num)
		return -EINVAL;
	fie->code = SC5239_MEDIA_BUS_FMT;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static int sc5239_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc5239 *sc5239 = to_sc5239(sd);
	const struct sc5239_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&sc5239->lock);
	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);
	fmt->format.code = SC5239_MEDIA_BUS_FMT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sc5239->lock);
		return -ENOTTY;
#endif
	} else {
		sc5239->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(sc5239->link_freq, mode->link_freq_index);
		__v4l2_ctrl_s_ctrl_int64(sc5239->pixel_rate,
					 to_pixel_rate(mode->link_freq_index));
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc5239->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc5239->vblank, vblank_def,
					 SC5239_VTS_MAX - mode->height,
					 1, vblank_def);
	}
	mutex_unlock(&sc5239->lock);
	return 0;
}

static int sc5239_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc5239 *sc5239 = to_sc5239(sd);
	const struct sc5239_mode *mode = sc5239->cur_mode;

	mutex_lock(&sc5239->lock);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sc5239->lock);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = SC5239_MEDIA_BUS_FMT;
		fmt->format.field = V4L2_FIELD_NONE;
		fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&sc5239->lock);
	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sc5239_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc5239 *sc5239 = to_sc5239(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sc5239_mode *def_mode = &supported_modes[0];

	mutex_lock(&sc5239->lock);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = SC5239_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;
	mutex_unlock(&sc5239->lock);
	return 0;
}
#endif
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sc5239_internal_ops = {
	.open = sc5239_open,
};
#endif

static int sc5239_s_power(struct v4l2_subdev *sd, int on)
{
	struct sc5239 *sc5239 = to_sc5239(sd);
	int ret = 0;

	mutex_lock(&sc5239->lock);
	if (sc5239->power_on == !!on)
		goto unlock_and_return;
	if (on) {
		ret = pm_runtime_get_sync(sc5239->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(sc5239->dev);
			goto unlock_and_return;
		}
		sc5239->power_on = true;
	} else {
		pm_runtime_put(sc5239->dev);
		sc5239->power_on = false;
	}
unlock_and_return:
	mutex_unlock(&sc5239->lock);
	return ret;
}

static const struct v4l2_subdev_core_ops sc5239_core_ops = {
	.s_power = sc5239_s_power,
	.ioctl = sc5239_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc5239_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc5239_video_ops = {
	.s_stream = sc5239_s_stream,
	.g_frame_interval = sc5239_g_frame_interval,
	.g_mbus_config = sc5239_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops sc5239_pad_ops = {
	.enum_mbus_code = sc5239_enum_mbus_code,
	.enum_frame_size = sc5239_enum_frame_sizes,
	.enum_frame_interval = sc5239_enum_frame_interval,
	.get_fmt = sc5239_get_fmt,
	.set_fmt = sc5239_set_fmt,
};

static const struct v4l2_subdev_ops sc5239_subdev_ops = {
	.core   = &sc5239_core_ops,
	.video  = &sc5239_video_ops,
	.pad    = &sc5239_pad_ops,
};

static int sc5239_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc5239 *sc5239 = to_sc5239(sd);

	__sc5239_power_on(sc5239);
	return 0;
}

static int sc5239_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc5239 *sc5239 = to_sc5239(sd);

	__sc5239_power_off(sc5239);
	return 0;
}

static const struct dev_pm_ops sc5239_pm_ops = {
	SET_RUNTIME_PM_OPS(sc5239_runtime_suspend,
			   sc5239_runtime_resume, NULL)
};

static int sc5239_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sc5239 *sc5239;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);
	sc5239 = devm_kzalloc(dev, sizeof(*sc5239), GFP_KERNEL);
	if (!sc5239)
		return -ENOMEM;
	sc5239->dev = dev;
	sc5239->regmap = devm_regmap_init_i2c(client, &sc5239_regmap_config);
	if (IS_ERR(sc5239->regmap)) {
		dev_err(dev, "Failed to initialize I2C\n");
		return -ENODEV;
	}
	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sc5239->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sc5239->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sc5239->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sc5239->len_name);
	if (ret) {
		dev_err(dev, "Failed to get module information\n");
		return -EINVAL;
	}
	sc5239->xvclk = devm_clk_get(sc5239->dev, "xvclk");
	if (IS_ERR(sc5239->xvclk)) {
		dev_err(sc5239->dev, "Failed to get xvclk\n");
		return -EINVAL;
	}
	sc5239->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sc5239->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");
	sc5239->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_HIGH);
	if (IS_ERR(sc5239->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");
	ret = sc5239_get_regulators(sc5239);
	if (ret) {
		dev_err(dev, "Failed to get regulators\n");
		return ret;
	}
	sc5239->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(sc5239->pinctrl)) {
		sc5239->pins_default =
			pinctrl_lookup_state(sc5239->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sc5239->pins_default))
			dev_info(dev, "could not get default pinstate\n");

		sc5239->pins_sleep =
			pinctrl_lookup_state(sc5239->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(sc5239->pins_sleep))
			dev_info(dev, "could not get sleep pinstate\n");
	} else {
		dev_info(dev, "no pinctrl\n");
	}
	mutex_init(&sc5239->lock);
	/* set default mode */
	sc5239->cur_mode = &supported_modes[0];
	sc5239->cfg_num = ARRAY_SIZE(supported_modes);
	sd = &sc5239->subdev;
	ret = __sc5239_power_on(sc5239);
	if (ret)
		goto err_free_handler;
	ret = sc5239_check_sensor_id(sc5239);
	if (ret)
		goto err_power_off;
	v4l2_i2c_subdev_init(sd, client, &sc5239_subdev_ops);
	ret = sc5239_initialize_controls(sc5239);
	if (ret)
		goto err_destroy_mutex;
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sc5239_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#ifdef CONFIG_MEDIA_CONTROLLER
	sc5239->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sc5239->pad);
	if (ret < 0)
		goto err_power_off;
#endif
	memset(facing, 0, sizeof(facing));
	if (strcmp(sc5239->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';
	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sc5239->module_index, facing,
		 SC5239_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "Failed to register v4l2 async subdev\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);
	return 0;
err_clean_entity:
#ifdef CONFIG_MEDIA_CONTROLLER
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__sc5239_power_off(sc5239);
err_free_handler:
	v4l2_ctrl_handler_free(&sc5239->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sc5239->lock);
	return ret;
}

static int sc5239_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc5239 *sc5239 = to_sc5239(sd);

	v4l2_async_unregister_subdev(sd);
#ifdef CONFIG_MEDIA_CONTROLLER
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sc5239->ctrl_handler);
	mutex_destroy(&sc5239->lock);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sc5239_power_off(sc5239);
	pm_runtime_set_suspended(&client->dev);
	return 0;
}

static const struct i2c_device_id sc5239_match_id[] = {
	{ "smartsens,sc5239", 0 },
	{ },
};

static const struct of_device_id sc5239_of_match[] = {
	{ .compatible = "smartsens,sc5239" },
	{},
};

MODULE_DEVICE_TABLE(of, sc5239_of_match);

static struct i2c_driver sc5239_i2c_driver = {
	.driver = {
		.name = SC5239_NAME,
		.pm = &sc5239_pm_ops,
		.of_match_table = of_match_ptr(sc5239_of_match),
	},
	.probe      = &sc5239_probe,
	.remove     = &sc5239_remove,
	.id_table   = sc5239_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sc5239_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sc5239_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Smartsens sc5239 Image Sensor driver");
MODULE_LICENSE("GPL v2");
