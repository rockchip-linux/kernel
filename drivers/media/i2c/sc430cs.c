// SPDX-License-Identifier: GPL-2.0
/*
 * sc430cs driver
 *
 * Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 add poweron function.
 * V0.0X01.0X02 fix mclk issue when probe multiple camera.
 * V0.0X01.0X03 fix gain range.
 * V0.0X01.0X04 add enum_frame_interval function.
 * V0.0X01.0X05 add quick stream on/off
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
#include <linux/rk-preisp.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x05)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define SC430CS_LANES			4
#define SC430CS_BITS_PER_SAMPLE		10
#define SC430CS_LINK_FREQ_315		157500000// 315Mbps

#define PIXEL_RATE_WITH_315M_10BIT		(SC430CS_LINK_FREQ_315 * 2 * \
					SC430CS_LANES / SC430CS_BITS_PER_SAMPLE)
#define SC430CS_XVCLK_FREQ		27000000

#define CHIP_ID				0xcd2e
#define SC430CS_REG_CHIP_ID		0x3107

#define SC430CS_REG_CTRL_MODE		0x0100
#define SC430CS_MODE_SW_STANDBY		0x0
#define SC430CS_MODE_STREAMING		BIT(0)

#define SC430CS_REG_EXPOSURE_H		0x3e00
#define SC430CS_REG_EXPOSURE_M		0x3e01
#define SC430CS_REG_EXPOSURE_L		0x3e02
#define	SC430CS_EXPOSURE_MIN		1
#define	SC430CS_EXPOSURE_STEP		1
#define SC430CS_VTS_MAX			0x7fff

#define SC430CS_REG_DIG_GAIN		0x3e06
#define SC430CS_REG_DIG_FINE_GAIN	0x3e07
#define SC430CS_REG_ANA_GAIN		0x3e08
#define SC430CS_REG_ANA_FINE_GAIN	0x3e09
#define SC430CS_GAIN_MIN		0x0040
#define SC430CS_GAIN_MAX		(24 * 32 * 64)    //23.32*31.75*64
#define SC430CS_GAIN_STEP		1
#define SC430CS_GAIN_DEFAULT		0x0800


#define SC430CS_REG_GROUP_HOLD		0x3812
#define SC430CS_GROUP_HOLD_START	0x00
#define SC430CS_GROUP_HOLD_END		0x30

#define SC430CS_REG_HIGH_TEMP_H		0x3974
#define SC430CS_REG_HIGH_TEMP_L		0x3975

#define SC430CS_REG_TEST_PATTERN	0x4501
#define SC430CS_TEST_PATTERN_BIT_MASK	BIT(3)

#define SC430CS_REG_VTS_H		0x320e
#define SC430CS_REG_VTS_L		0x320f

#define SC430CS_FLIP_MIRROR_REG		0x3221

#define SC430CS_FETCH_EXP_H(VAL)		(((VAL) >> 12) & 0xF)
#define SC430CS_FETCH_EXP_M(VAL)		(((VAL) >> 4) & 0xFF)
#define SC430CS_FETCH_EXP_L(VAL)		(((VAL) & 0xF) << 4)

#define SC430CS_FETCH_AGAIN_H(VAL)		(((VAL) >> 8) & 0x03)
#define SC430CS_FETCH_AGAIN_L(VAL)		((VAL) & 0xFF)

#define SC430CS_FETCH_MIRROR(VAL, ENABLE)	(ENABLE ? VAL | 0x06 : VAL & 0xf9)
#define SC430CS_FETCH_FLIP(VAL, ENABLE)		(ENABLE ? VAL | 0x60 : VAL & 0x9f)

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define SC430CS_REG_VALUE_08BIT		1
#define SC430CS_REG_VALUE_16BIT		2
#define SC430CS_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"
#define SC430CS_NAME			"sc430cs"

static const char * const sc430cs_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SC430CS_NUM_SUPPLIES ARRAY_SIZE(sc430cs_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct sc430cs_mode {
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

struct sc430cs {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[SC430CS_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;
	struct v4l2_fract	cur_fps;
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
	const struct sc430cs_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32			cur_vts;
	struct preisp_hdrae_exp_s init_hdrae_exp;
};

#define to_sc430cs(sd) container_of(sd, struct sc430cs, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval sc430cs_global_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 315Mbps, 4lane
 */
static const struct regval sc430cs_linear_10_2560x1440_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x36f9, 0x80},
	{0x301c, 0x78},
	{0x301f, 0x01},
	{0x3208, 0x0a},
	{0x3209, 0x00},
	{0x320a, 0x05},
	{0x320b, 0xa0},
	{0x320e, 0x05},
	{0x320f, 0xdc},
	{0x3214, 0x11},
	{0x3215, 0x11},
	{0x3223, 0x80},
	{0x3250, 0x00},
	{0x3253, 0x08},
	{0x3274, 0x01},
	{0x3301, 0x20},
	{0x3302, 0x18},
	{0x3303, 0x10},
	{0x3304, 0x50},
	{0x3306, 0x38},
	{0x3308, 0x18},
	{0x3309, 0x60},
	{0x330b, 0xc0},
	{0x330d, 0x10},
	{0x330e, 0x18},
	{0x330f, 0x04},
	{0x3310, 0x02},
	{0x331c, 0x04},
	{0x331e, 0x41},
	{0x331f, 0x51},
	{0x3320, 0x09},
	{0x3333, 0x10},
	{0x334c, 0x08},
	{0x3356, 0x09},
	{0x3364, 0x17},
	{0x338e, 0xfd},
	{0x3390, 0x08},
	{0x3391, 0x18},
	{0x3392, 0x38},
	{0x3393, 0x20},
	{0x3394, 0x20},
	{0x3395, 0x20},
	{0x3396, 0x08},
	{0x3397, 0x18},
	{0x3398, 0x38},
	{0x3399, 0x20},
	{0x339a, 0x20},
	{0x339b, 0x20},
	{0x339c, 0x20},
	{0x33ac, 0x10},
	{0x33ae, 0x18},
	{0x33af, 0x19},
	{0x360f, 0x01},
	{0x3620, 0x08},
	{0x3637, 0x25},
	{0x363a, 0x12},
	{0x3670, 0x0a},
	{0x3671, 0x07},
	{0x3672, 0x57},
	{0x3673, 0x5e},
	{0x3674, 0x84},
	{0x3675, 0x88},
	{0x3676, 0x8a},
	{0x367a, 0x58},
	{0x367b, 0x78},
	{0x367c, 0x58},
	{0x367d, 0x78},
	{0x3690, 0x33},
	{0x3691, 0x43},
	{0x3692, 0x34},
	{0x369c, 0x40},
	{0x369d, 0x78},
	{0x36ea, 0x39},
	{0x36eb, 0x0d},
	{0x36ec, 0x2c},
	{0x36ed, 0x24},
	{0x36fa, 0x39},
	{0x36fb, 0x33},
	{0x36fc, 0x10},
	{0x36fd, 0x14},
	{0x3908, 0x41},
	{0x396c, 0x0e},
	{0x3e00, 0x00},
	{0x3e01, 0xb6},
	{0x3e02, 0x00},
	{0x3e03, 0x0b},
	{0x3e08, 0x03},
	{0x3e09, 0x40},
	{0x3e1b, 0x2a},
	{0x4509, 0x30},
	{0x57a8, 0xd0},
	{0x36e9, 0x14},
	{0x36f9, 0x14},
	{REG_NULL, 0x00},
};

static const struct sc430cs_mode supported_modes[] = {
	{
		.width = 2560,
		.height = 1440,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0080,
		.hts_def = 0x0578 * 2,
		.vts_def = 0x05dc,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.reg_list = sc430cs_linear_10_2560x1440_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	}
};

static const s64 link_freq_menu_items[] = {
	SC430CS_LINK_FREQ_315
};

static const char * const sc430cs_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int sc430cs_write_reg(struct i2c_client *client, u16 reg,
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

static int sc430cs_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = sc430cs_write_reg(client, regs[i].addr,
					SC430CS_REG_VALUE_08BIT, regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int sc430cs_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
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

static int sc430cs_set_gain_reg(struct sc430cs *sc430cs, u32 gain)
{
	u8 Coarse_gain = 1, DIG_gain = 1;
	u32 Dcg_gainx100 = 1, ANA_Fine_gainx64 = 1;
	u8 Coarse_gain_reg = 0, DIG_gain_reg = 0;
	u8 ANA_Fine_gain_reg = 0x20, DIG_Fine_gain_reg = 0x80;
	int ret = 0;

	gain = gain * 16;
	if (gain <= 1024)
		gain = 1024;
	else if (gain > SC430CS_GAIN_MAX * 16)
		gain = SC430CS_GAIN_MAX * 16;

	if (gain < 1504) {               // start again
		Dcg_gainx100 = 100;
		Coarse_gain = 1;
		DIG_gain = 1;
		Coarse_gain_reg = 0x03;
		DIG_gain_reg = 0x0;
		DIG_Fine_gain_reg = 0x80;
	} else if (gain <= 3008) {
		Dcg_gainx100 = 147;
		Coarse_gain = 1;
		DIG_gain = 1;
		Coarse_gain_reg = 0x23;
		DIG_gain_reg = 0x0;
		DIG_Fine_gain_reg = 0x80;
	} else if (gain <= 6017) {
		Dcg_gainx100 = 147;
		Coarse_gain = 2;
		DIG_gain = 1;
		Coarse_gain_reg = 0x27;
		DIG_gain_reg = 0x0;
		DIG_Fine_gain_reg = 0x80;
	} else if (gain <= 12034) {
		Dcg_gainx100 = 147;
		Coarse_gain = 4;
		DIG_gain = 1;
		Coarse_gain_reg = 0x2f;
		DIG_gain_reg = 0x0;
		DIG_Fine_gain_reg = 0x80;
	} else if (gain <= 23879) {           // end again
		Dcg_gainx100 = 147;
		Coarse_gain = 8;
		DIG_gain = 1;
		Coarse_gain_reg = 0x3f;
		DIG_gain_reg = 0x0;
		DIG_Fine_gain_reg = 0x80;
	} else if (gain < 23879 * 2) {         // start dgain
		Dcg_gainx100 = 147;
		Coarse_gain = 8;
		DIG_gain = 1;
		ANA_Fine_gainx64 = 127;
		Coarse_gain_reg = 0x3f;
		DIG_gain_reg = 0x0;
		ANA_Fine_gain_reg = 0x7f;
	} else if (gain < 23879 * 4) {
		Dcg_gainx100 = 147;
		Coarse_gain = 8;
		DIG_gain = 2;
		ANA_Fine_gainx64 = 127;
		Coarse_gain_reg = 0x3f;
		DIG_gain_reg = 0x1;
		ANA_Fine_gain_reg = 0x7f;
	} else if (gain < 23879 * 8) {
		Dcg_gainx100 = 147;
		Coarse_gain = 8;
		DIG_gain = 4;
		ANA_Fine_gainx64 = 127;
		Coarse_gain_reg = 0x3f;
		DIG_gain_reg = 0x3;
		ANA_Fine_gain_reg = 0x7f;
	} else if (gain < 23879 * 16) {
		Dcg_gainx100 = 147;
		Coarse_gain = 8;
		DIG_gain = 8;
		ANA_Fine_gainx64 = 127;
		Coarse_gain_reg = 0x3f;
		DIG_gain_reg = 0x7;
		ANA_Fine_gain_reg = 0x7f;
	} else if (gain <= 1754822) {
		Dcg_gainx100 = 147;
		Coarse_gain = 8;
		DIG_gain = 16;
		ANA_Fine_gainx64 = 127;
		Coarse_gain_reg = 0x3f;
		DIG_gain_reg = 0xF;
		ANA_Fine_gain_reg = 0x7f;
	}

	if (gain < 1504)
		ANA_Fine_gain_reg = abs(100 * gain / (Dcg_gainx100 * Coarse_gain) / 16);
	else if (gain == 1504)
		ANA_Fine_gain_reg = 0x40;
	else if (gain < 23879)
		ANA_Fine_gain_reg = abs(100 * gain / (Dcg_gainx100 * Coarse_gain) / 16);
	else
		DIG_Fine_gain_reg = abs(800 * gain / (Dcg_gainx100 * Coarse_gain *
							DIG_gain) / ANA_Fine_gainx64);

	ret = sc430cs_write_reg(sc430cs->client,
				SC430CS_REG_DIG_GAIN,
				SC430CS_REG_VALUE_08BIT,
				DIG_gain_reg & 0xF);
	ret |= sc430cs_write_reg(sc430cs->client,
				 SC430CS_REG_DIG_FINE_GAIN,
				 SC430CS_REG_VALUE_08BIT,
				 DIG_Fine_gain_reg);
	ret |= sc430cs_write_reg(sc430cs->client,
				 SC430CS_REG_ANA_GAIN,
				 SC430CS_REG_VALUE_08BIT,
				 Coarse_gain_reg);
	ret |= sc430cs_write_reg(sc430cs->client,
				 SC430CS_REG_ANA_FINE_GAIN,
				 SC430CS_REG_VALUE_08BIT,
				 ANA_Fine_gain_reg);

	return ret;
}

static int sc430cs_get_reso_dist(const struct sc430cs_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct sc430cs_mode *
sc430cs_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = sc430cs_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int sc430cs_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct sc430cs *sc430cs = to_sc430cs(sd);
	const struct sc430cs_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&sc430cs->mutex);

	mode = sc430cs_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sc430cs->mutex);
		return -ENOTTY;
#endif
	} else {
		sc430cs->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc430cs->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc430cs->vblank, vblank_def,
					 SC430CS_VTS_MAX - mode->height,
					 1, vblank_def);
		sc430cs->cur_fps = mode->max_fps;
		sc430cs->cur_vts = (u32)mode->vts_def;
	}

	mutex_unlock(&sc430cs->mutex);

	return 0;
}

static int sc430cs_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct sc430cs *sc430cs = to_sc430cs(sd);
	const struct sc430cs_mode *mode = sc430cs->cur_mode;

	mutex_lock(&sc430cs->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sc430cs->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
		/* format info: width/height/data type/virctual channel */
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&sc430cs->mutex);

	return 0;
}

static int sc430cs_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct sc430cs *sc430cs = to_sc430cs(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = sc430cs->cur_mode->bus_fmt;

	return 0;
}

static int sc430cs_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != supported_modes[0].bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int sc430cs_enable_test_pattern(struct sc430cs *sc430cs, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = sc430cs_read_reg(sc430cs->client, SC430CS_REG_TEST_PATTERN,
			       SC430CS_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= SC430CS_TEST_PATTERN_BIT_MASK;
	else
		val &= ~SC430CS_TEST_PATTERN_BIT_MASK;

	ret |= sc430cs_write_reg(sc430cs->client, SC430CS_REG_TEST_PATTERN,
				 SC430CS_REG_VALUE_08BIT, val);
	return ret;
}

static int sc430cs_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct sc430cs *sc430cs = to_sc430cs(sd);
	const struct sc430cs_mode *mode = sc430cs->cur_mode;

	if (sc430cs->streaming)
		fi->interval = sc430cs->cur_fps;
	else
		fi->interval = mode->max_fps;

	return 0;
}

static int sc430cs_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				 struct v4l2_mbus_config *config)
{
	struct sc430cs *sc430cs = to_sc430cs(sd);
	const struct sc430cs_mode *mode = sc430cs->cur_mode;
	u32 val = 1 << (SC430CS_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	if (mode->hdr_mode != NO_HDR)
		val |= V4L2_MBUS_CSI2_CHANNEL_1;
	if (mode->hdr_mode == HDR_X3)
		val |= V4L2_MBUS_CSI2_CHANNEL_2;

	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;

	return 0;
}

static void sc430cs_get_module_inf(struct sc430cs *sc430cs,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, SC430CS_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, sc430cs->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, sc430cs->len_name, sizeof(inf->base.lens));
}

static long sc430cs_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc430cs *sc430cs = to_sc430cs(sd);
	struct rkmodule_hdr_cfg *hdr;
	u32 i, h, w;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		sc430cs_get_module_inf(sc430cs, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = sc430cs->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = sc430cs->cur_mode->width;
		h = sc430cs->cur_mode->height;
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				sc430cs->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == ARRAY_SIZE(supported_modes)) {
			dev_err(&sc430cs->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = sc430cs->cur_mode->hts_def - sc430cs->cur_mode->width;
			h = sc430cs->cur_mode->vts_def - sc430cs->cur_mode->height;
			__v4l2_ctrl_modify_range(sc430cs->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(sc430cs->vblank, h,
						 SC430CS_VTS_MAX - sc430cs->cur_mode->height, 1, h);
			sc430cs->cur_fps = sc430cs->cur_mode->max_fps;
			sc430cs->cur_vts = sc430cs->cur_mode->vts_def;
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = sc430cs_write_reg(sc430cs->client, SC430CS_REG_CTRL_MODE,
				 SC430CS_REG_VALUE_08BIT, SC430CS_MODE_STREAMING);
		else
			ret = sc430cs_write_reg(sc430cs->client, SC430CS_REG_CTRL_MODE,
				 SC430CS_REG_VALUE_08BIT, SC430CS_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sc430cs_compat_ioctl32(struct v4l2_subdev *sd,
				   unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc430cs_ioctl(sd, cmd, inf);
		if (!ret) {
			ret = copy_to_user(up, inf, sizeof(*inf));
			if (ret)
				ret = -EFAULT;
		}
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
			ret = sc430cs_ioctl(sd, cmd, cfg);
		else
			ret = -EFAULT;
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc430cs_ioctl(sd, cmd, hdr);
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

		ret = copy_from_user(hdr, up, sizeof(*hdr));
		if (!ret)
			ret = sc430cs_ioctl(sd, cmd, hdr);
		else
			ret = -EFAULT;
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
			ret = sc430cs_ioctl(sd, cmd, hdrae);
		else
			ret = -EFAULT;
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = sc430cs_ioctl(sd, cmd, &stream);
		else
			ret = -EFAULT;
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __sc430cs_start_stream(struct sc430cs *sc430cs)
{
	int ret;

	ret = sc430cs_write_array(sc430cs->client, sc430cs->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	ret = __v4l2_ctrl_handler_setup(&sc430cs->ctrl_handler);
	if (ret)
		return ret;

	return sc430cs_write_reg(sc430cs->client, SC430CS_REG_CTRL_MODE,
				 SC430CS_REG_VALUE_08BIT, SC430CS_MODE_STREAMING);
}

static int __sc430cs_stop_stream(struct sc430cs *sc430cs)
{
	return sc430cs_write_reg(sc430cs->client, SC430CS_REG_CTRL_MODE,
				 SC430CS_REG_VALUE_08BIT, SC430CS_MODE_SW_STANDBY);
}

static int sc430cs_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sc430cs *sc430cs = to_sc430cs(sd);
	struct i2c_client *client = sc430cs->client;
	int ret = 0;

	mutex_lock(&sc430cs->mutex);
	on = !!on;
	if (on == sc430cs->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __sc430cs_start_stream(sc430cs);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__sc430cs_stop_stream(sc430cs);
		pm_runtime_put(&client->dev);
	}

	sc430cs->streaming = on;

unlock_and_return:
	mutex_unlock(&sc430cs->mutex);

	return ret;
}

static int sc430cs_s_power(struct v4l2_subdev *sd, int on)
{
	struct sc430cs *sc430cs = to_sc430cs(sd);
	struct i2c_client *client = sc430cs->client;
	int ret = 0;

	mutex_lock(&sc430cs->mutex);

	/* If the power state is not modified - no work to do. */
	if (sc430cs->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = sc430cs_write_array(sc430cs->client, sc430cs_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		sc430cs->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		sc430cs->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&sc430cs->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 sc430cs_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, SC430CS_XVCLK_FREQ / 1000 / 1000);
}

static int __sc430cs_power_on(struct sc430cs *sc430cs)
{
	int ret;
	u32 delay_us;
	struct device *dev = &sc430cs->client->dev;

	if (!IS_ERR_OR_NULL(sc430cs->pins_default)) {
		ret = pinctrl_select_state(sc430cs->pinctrl,
					   sc430cs->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(sc430cs->xvclk, SC430CS_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(sc430cs->xvclk) != SC430CS_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(sc430cs->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(sc430cs->reset_gpio))
		gpiod_set_value_cansleep(sc430cs->reset_gpio, 0);

	ret = regulator_bulk_enable(SC430CS_NUM_SUPPLIES, sc430cs->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(sc430cs->reset_gpio))
		gpiod_set_value_cansleep(sc430cs->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(sc430cs->pwdn_gpio))
		gpiod_set_value_cansleep(sc430cs->pwdn_gpio, 1);

	if (!IS_ERR(sc430cs->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = sc430cs_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(sc430cs->xvclk);

	return ret;
}

static void __sc430cs_power_off(struct sc430cs *sc430cs)
{
	int ret;
	struct device *dev = &sc430cs->client->dev;

	if (!IS_ERR(sc430cs->pwdn_gpio))
		gpiod_set_value_cansleep(sc430cs->pwdn_gpio, 0);
	clk_disable_unprepare(sc430cs->xvclk);
	if (!IS_ERR(sc430cs->reset_gpio))
		gpiod_set_value_cansleep(sc430cs->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(sc430cs->pins_sleep)) {
		ret = pinctrl_select_state(sc430cs->pinctrl,
					   sc430cs->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(SC430CS_NUM_SUPPLIES, sc430cs->supplies);
}

static int sc430cs_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc430cs *sc430cs = to_sc430cs(sd);

	return __sc430cs_power_on(sc430cs);
}

static int sc430cs_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc430cs *sc430cs = to_sc430cs(sd);

	__sc430cs_power_off(sc430cs);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sc430cs_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc430cs *sc430cs = to_sc430cs(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sc430cs_mode *def_mode = &supported_modes[0];

	mutex_lock(&sc430cs->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&sc430cs->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int sc430cs_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops sc430cs_pm_ops = {
	SET_RUNTIME_PM_OPS(sc430cs_runtime_suspend,
			   sc430cs_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sc430cs_internal_ops = {
	.open = sc430cs_open,
};
#endif

static const struct v4l2_subdev_core_ops sc430cs_core_ops = {
	.s_power = sc430cs_s_power,
	.ioctl = sc430cs_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc430cs_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc430cs_video_ops = {
	.s_stream = sc430cs_s_stream,
	.g_frame_interval = sc430cs_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops sc430cs_pad_ops = {
	.enum_mbus_code = sc430cs_enum_mbus_code,
	.enum_frame_size = sc430cs_enum_frame_sizes,
	.enum_frame_interval = sc430cs_enum_frame_interval,
	.get_fmt = sc430cs_get_fmt,
	.set_fmt = sc430cs_set_fmt,
	.get_mbus_config = sc430cs_g_mbus_config,
};

static const struct v4l2_subdev_ops sc430cs_subdev_ops = {
	.core	= &sc430cs_core_ops,
	.video	= &sc430cs_video_ops,
	.pad	= &sc430cs_pad_ops,
};

static void sc430cs_modify_fps_info(struct sc430cs *sc430cs)
{
	const struct sc430cs_mode *mode = sc430cs->cur_mode;

	sc430cs->cur_fps.denominator = mode->max_fps.denominator * mode->vts_def /
				       sc430cs->cur_vts;
}

static int sc430cs_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc430cs *sc430cs = container_of(ctrl->handler,
					       struct sc430cs, ctrl_handler);
	struct i2c_client *client = sc430cs->client;
	s64 max;
	int ret = 0;
	u32 val = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sc430cs->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(sc430cs->exposure,
					 sc430cs->exposure->minimum, max,
					 sc430cs->exposure->step,
					 sc430cs->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		if (sc430cs->cur_mode->hdr_mode == NO_HDR) {
			val = ctrl->val << 1;
			/* 4 least significant bits of expsoure are fractional part */
			ret = sc430cs_write_reg(sc430cs->client,
						SC430CS_REG_EXPOSURE_H,
						SC430CS_REG_VALUE_08BIT,
						SC430CS_FETCH_EXP_H(val));
			ret |= sc430cs_write_reg(sc430cs->client,
						 SC430CS_REG_EXPOSURE_M,
						 SC430CS_REG_VALUE_08BIT,
						 SC430CS_FETCH_EXP_M(val));
			ret |= sc430cs_write_reg(sc430cs->client,
						 SC430CS_REG_EXPOSURE_L,
						 SC430CS_REG_VALUE_08BIT,
						 SC430CS_FETCH_EXP_L(val));
		}
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		if (sc430cs->cur_mode->hdr_mode == NO_HDR)
			ret = sc430cs_set_gain_reg(sc430cs, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = sc430cs_write_reg(sc430cs->client,
					SC430CS_REG_VTS_H,
					SC430CS_REG_VALUE_08BIT,
					(ctrl->val + sc430cs->cur_mode->height)
					>> 8);
		ret |= sc430cs_write_reg(sc430cs->client,
					 SC430CS_REG_VTS_L,
					 SC430CS_REG_VALUE_08BIT,
					 (ctrl->val + sc430cs->cur_mode->height)
					 & 0xff);
		if (!ret)
			sc430cs->cur_vts = ctrl->val + sc430cs->cur_mode->height;
		sc430cs_modify_fps_info(sc430cs);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = sc430cs_enable_test_pattern(sc430cs, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = sc430cs_read_reg(sc430cs->client, SC430CS_FLIP_MIRROR_REG,
				       SC430CS_REG_VALUE_08BIT, &val);
		ret |= sc430cs_write_reg(sc430cs->client, SC430CS_FLIP_MIRROR_REG,
					 SC430CS_REG_VALUE_08BIT,
					 SC430CS_FETCH_MIRROR(val, ctrl->val));
		break;
	case V4L2_CID_VFLIP:
		ret = sc430cs_read_reg(sc430cs->client, SC430CS_FLIP_MIRROR_REG,
				       SC430CS_REG_VALUE_08BIT, &val);
		ret |= sc430cs_write_reg(sc430cs->client, SC430CS_FLIP_MIRROR_REG,
					 SC430CS_REG_VALUE_08BIT,
					 SC430CS_FETCH_FLIP(val, ctrl->val));
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sc430cs_ctrl_ops = {
	.s_ctrl = sc430cs_set_ctrl,
};

static int sc430cs_initialize_controls(struct sc430cs *sc430cs)
{
	const struct sc430cs_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &sc430cs->ctrl_handler;
	mode = sc430cs->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &sc430cs->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, PIXEL_RATE_WITH_315M_10BIT, 1, PIXEL_RATE_WITH_315M_10BIT);

	h_blank = mode->hts_def - mode->width;
	sc430cs->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (sc430cs->hblank)
		sc430cs->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	sc430cs->vblank = v4l2_ctrl_new_std(handler, &sc430cs_ctrl_ops,
					    V4L2_CID_VBLANK, vblank_def,
					    SC430CS_VTS_MAX - mode->height,
					    1, vblank_def);
	exposure_max = mode->vts_def - 4;
	sc430cs->exposure = v4l2_ctrl_new_std(handler, &sc430cs_ctrl_ops,
					      V4L2_CID_EXPOSURE, SC430CS_EXPOSURE_MIN,
					      exposure_max, SC430CS_EXPOSURE_STEP,
					      mode->exp_def);
	sc430cs->anal_gain = v4l2_ctrl_new_std(handler, &sc430cs_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN, SC430CS_GAIN_MIN,
					       SC430CS_GAIN_MAX, SC430CS_GAIN_STEP,
					       SC430CS_GAIN_DEFAULT);
	sc430cs->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
							    &sc430cs_ctrl_ops,
					V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(sc430cs_test_pattern_menu) - 1,
					0, 0, sc430cs_test_pattern_menu);
	v4l2_ctrl_new_std(handler, &sc430cs_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &sc430cs_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(&sc430cs->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	sc430cs->subdev.ctrl_handler = handler;
	sc430cs->cur_fps = mode->max_fps;
	sc430cs->cur_vts = mode->vts_def;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int sc430cs_check_sensor_id(struct sc430cs *sc430cs,
				   struct i2c_client *client)
{
	struct device *dev = &sc430cs->client->dev;
	u32 id = 0;
	int ret;

	ret = sc430cs_read_reg(client, SC430CS_REG_CHIP_ID,
			       SC430CS_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected OV%06x sensor\n", CHIP_ID);

	return 0;
}

static int sc430cs_configure_regulators(struct sc430cs *sc430cs)
{
	unsigned int i;

	for (i = 0; i < SC430CS_NUM_SUPPLIES; i++)
		sc430cs->supplies[i].supply = sc430cs_supply_names[i];

	return devm_regulator_bulk_get(&sc430cs->client->dev,
				       SC430CS_NUM_SUPPLIES,
				       sc430cs->supplies);
}

static int sc430cs_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sc430cs *sc430cs;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	u32 i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	sc430cs = devm_kzalloc(dev, sizeof(*sc430cs), GFP_KERNEL);
	if (!sc430cs)
		return -ENOMEM;

	of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sc430cs->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sc430cs->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sc430cs->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sc430cs->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	sc430cs->client = client;
	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			sc430cs->cur_mode = &supported_modes[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(supported_modes))
		sc430cs->cur_mode = &supported_modes[0];

	sc430cs->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sc430cs->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	sc430cs->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sc430cs->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	sc430cs->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(sc430cs->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	sc430cs->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(sc430cs->pinctrl)) {
		sc430cs->pins_default =
			pinctrl_lookup_state(sc430cs->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sc430cs->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		sc430cs->pins_sleep =
			pinctrl_lookup_state(sc430cs->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(sc430cs->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = sc430cs_configure_regulators(sc430cs);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&sc430cs->mutex);

	sd = &sc430cs->subdev;
	v4l2_i2c_subdev_init(sd, client, &sc430cs_subdev_ops);
	ret = sc430cs_initialize_controls(sc430cs);
	if (ret)
		goto err_destroy_mutex;

	ret = __sc430cs_power_on(sc430cs);
	if (ret)
		goto err_free_handler;

	ret = sc430cs_check_sensor_id(sc430cs, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sc430cs_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	sc430cs->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sc430cs->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(sc430cs->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sc430cs->module_index, facing,
		 SC430CS_NAME, dev_name(sd->dev));
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
	__sc430cs_power_off(sc430cs);
err_free_handler:
	v4l2_ctrl_handler_free(&sc430cs->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sc430cs->mutex);

	return ret;
}

static int sc430cs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc430cs *sc430cs = to_sc430cs(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sc430cs->ctrl_handler);
	mutex_destroy(&sc430cs->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sc430cs_power_off(sc430cs);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sc430cs_of_match[] = {
	{ .compatible = "smartsens,sc430cs" },
	{},
};
MODULE_DEVICE_TABLE(of, sc430cs_of_match);
#endif

static const struct i2c_device_id sc430cs_match_id[] = {
	{ "smartsens,sc430cs", 0 },
	{ },
};

static struct i2c_driver sc430cs_i2c_driver = {
	.driver = {
		.name = SC430CS_NAME,
		.pm = &sc430cs_pm_ops,
		.of_match_table = of_match_ptr(sc430cs_of_match),
	},
	.probe		= &sc430cs_probe,
	.remove		= &sc430cs_remove,
	.id_table	= sc430cs_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sc430cs_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sc430cs_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("smartsens sc430cs sensor driver");
MODULE_LICENSE("GPL v2");
