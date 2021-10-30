// SPDX-License-Identifier: GPL-2.0
/*
 * sc1330 driver
 *
 * Copyright (C) 2021 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 add poweron function.
 * V0.0X01.0X02 fix mclk issue when probe multiple camera.
 * V0.0X01.0X03 add enum_frame_interval function.
 * V0.0X01.0X04 add quick stream on/off
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

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x04)

#define SC1330_NAME			"sc1330"
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"
#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define SC1330_CHIP_ID			0xca18
#define SC1330_REG_CHIP_ID		0x3107

#define DVP_FREQ_72M			72000000
#define SC1330_MAX_PIXEL_RATE		(DVP_FREQ_72M / 10 * 10) //(DVP_FREQ_72M * 2/ 10 * 10)

#define SC1330_REG_CTRL_MODE		0x0100
#define SC1330_MODE_SW_STANDBY		0x0
#define SC1330_MODE_STREAMING		BIT(0)

#define	SC1330_EXPOSURE_MIN		2// two lines long exp min
#define	SC1330_EXPOSURE_STEP		1
#define SC1330_VTS_MAX			0xffff

#define SC1330_XVCLK_FREQ		27000000
#define SC1330_DVP_BITS			10

#define SC1330_REG_VALUE_08BIT		1
#define SC1330_REG_VALUE_16BIT		2
#define SC1330_REG_VALUE_24BIT		3

//long exposure
#define SC1330_REG_EXP_LONG_H		0x3e00    //[3:0]
#define SC1330_REG_EXP_LONG_M		0x3e01    //[7:0]
#define SC1330_REG_EXP_LONG_L		0x3e02    //[7:4]

//short exposure
#define SC1330_REG_EXP_SF_H		0x3e04    //[7:0]
#define SC1330_REG_EXP_SF_L		0x3e05    //[7:4]

//long frame and normal gain reg
#define SC1330_REG_AGAIN		0x3e08
#define SC1330_REG_AGAIN_FINE		0x3e09

#define SC1330_REG_DGAIN		0x3e06
#define SC1330_REG_DGAIN_FINE		0x3e07

#define SC1330_GAIN_MIN			0x40
#define SC1330_GAIN_MAX			(44 * 32 * 64)
#define SC1330_GAIN_STEP		1
#define SC1330_GAIN_DEFAULT		0x40

//group hold
#define SC1330_GROUP_UPDATE_ADDRESS	0x3812
#define SC1330_GROUP_UPDATE_START_DATA	0x00
#define SC1330_GROUP_UPDATE_LAUNCH	0x30

#define SC1330_SOFTWARE_RESET_REG	0x0103
#define SC1330_REG_TEST_PATTERN		0x4501
#define SC1330_TEST_PATTERN_ENABLE	0x08

#define SC1330_REG_VTS			0x320e
#define SC1330_FLIP_REG			0x3221
#define SC1330_FLIP_MASK		0x60
#define SC1330_MIRROR_MASK		0x06
#define REG_NULL			0xFFFF

static const char * const sc1330_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SC1330_NUM_SUPPLIES ARRAY_SIZE(sc1330_supply_names)

#define to_sc1330(sd) container_of(sd, struct sc1330, subdev)

enum sc1330_max_pad {
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

struct sc1330_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 dvp_freq_idx;
	u32 bpp;
	u32 vc[PAD_MAX];
};

struct sc1330 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[SC1330_NUM_SUPPLIES];
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
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct sc1330_mode *cur_mode;
	u32			cfg_num;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	bool			has_init_exp;
	u32			cur_vts;
	struct preisp_hdrae_exp_s init_hdrae_exp;
};

static const struct regval sc1330_linear10bit_1280x960_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x36f9, 0x80},
	{0x3001, 0xff},
	{0x3002, 0xf0},
	{0x300a, 0x24},
	{0x3018, 0x6f},
	{0x301a, 0xf8},
	{0x301c, 0x94},
	{0x301f, 0x33},
	{0x303f, 0x81},
	{0x320c, 0x09},
	{0x320d, 0x60},
	{0x3211, 0x06},
	{0x3251, 0x98},
	{0x3253, 0x08},
	{0x325f, 0x0a},
	{0x3304, 0x40},
	{0x3306, 0x70},
	{0x3309, 0x70},
	{0x330a, 0x01},
	{0x330b, 0xf0},
	{0x330d, 0x28},
	{0x3310, 0x0e},
	{0x3314, 0x92},
	{0x331e, 0x31},
	{0x331f, 0x61},
	{0x335d, 0x60},
	{0x3364, 0x5e},
	{0x3396, 0x08},
	{0x3397, 0x18},
	{0x3398, 0x38},
	{0x3399, 0x0c},
	{0x339a, 0x10},
	{0x339b, 0x1e},
	{0x339c, 0x70},
	{0x33af, 0x38},
	{0x360f, 0x21},
	{0x3621, 0xe8},
	{0x3632, 0x68},
	{0x3633, 0x33},
	{0x3634, 0x23},
	{0x3635, 0x20},
	{0x3637, 0x19},
	{0x3638, 0x08},
	{0x363b, 0x04},
	{0x363c, 0x06},
	{0x3641, 0x01},
	{0x3670, 0x42},
	{0x3671, 0x05},
	{0x3672, 0x15},
	{0x3673, 0x15},
	{0x3674, 0xc0},
	{0x3675, 0x84},
	{0x3676, 0x88},
	{0x367a, 0x48},
	{0x367b, 0x58},
	{0x367c, 0x48},
	{0x367d, 0x58},
	{0x3699, 0x00},
	{0x369a, 0x00},
	{0x369b, 0x1f},
	{0x36a2, 0x48},
	{0x3000, 0x0f},
	{0x36a3, 0x58},
	{0x36a6, 0x48},
	{0x36a7, 0x58},
	{0x36ab, 0xc0},
	{0x36ac, 0x84},
	{0x36ad, 0x88},
	{0x36d0, 0x40},
	{0x36db, 0x04},
	{0x36dc, 0x14},
	{0x36dd, 0x14},
	{0x36de, 0x48},
	{0x36df, 0x58},
	{0x36ea, 0x30},
	{0x36eb, 0x07},
	{0x36ec, 0x17},
	{0x36ed, 0x14},
	{0x36fa, 0x30},
	{0x36fb, 0x00},
	{0x36fc, 0x10},
	{0x36fd, 0x14},
	{0x3e01, 0x3e},
	{0x450a, 0x71},
	{0x4603, 0x09},
	{0x578a, 0x18},
	{0x578b, 0x10},
	{0x5793, 0x18},
	{0x5794, 0x10},
	{0x5799, 0x00},
	{0x36e9, 0x20},
	{0x36f9, 0x20},
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
static const struct sc1330_mode supported_modes[] = {
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 1280,
		.height = 960,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x03e8/2,
		.hts_def = 0x0960,
		.vts_def = 0x03e8,
		.reg_list = sc1330_linear10bit_1280x960_regs,
		.hdr_mode = NO_HDR,
		.dvp_freq_idx = 0,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	}
};

static const s64 link_freq_items[] = {
	DVP_FREQ_72M,
};

/* Write registers up to 4 at a time */
static int sc1330_write_reg(struct i2c_client *client, u16 reg,
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

static int sc1330_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		ret |= sc1330_write_reg(client, regs[i].addr,
					SC1330_REG_VALUE_08BIT, regs[i].val);
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int sc1330_read_reg(struct i2c_client *client,
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

static int sc1330_enable_test_pattern(struct sc1330 *sc1330, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = sc1330_read_reg(sc1330->client, SC1330_REG_TEST_PATTERN,
			      SC1330_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= SC1330_TEST_PATTERN_ENABLE;
	else
		val &= ~SC1330_TEST_PATTERN_ENABLE;
	ret |= sc1330_write_reg(sc1330->client, SC1330_REG_TEST_PATTERN,
				SC1330_REG_VALUE_08BIT, val);
	return ret;
}

static void sc1330_get_module_inf(struct sc1330 *sc1330,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, SC1330_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, sc1330->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, sc1330->len_name, sizeof(inf->base.lens));
}

static void sc1330_change_mode(struct sc1330 *sc1330, const struct sc1330_mode *mode)
{
	sc1330->cur_mode = mode;
	sc1330->cur_vts = sc1330->cur_mode->vts_def;
	dev_info(&sc1330->client->dev, "set fmt: cur_mode: %dx%d, hdr: %d\n",
		mode->width, mode->height, mode->hdr_mode);
}

static int sc1330_get_reso_dist(const struct sc1330_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct sc1330_mode *
sc1330_find_best_fit(struct sc1330 *sc1330, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < sc1330->cfg_num; i++) {
		dist = sc1330_get_reso_dist(&supported_modes[i], framefmt);
		if ((cur_best_fit_dist == -1 || dist <= cur_best_fit_dist) &&
		    (supported_modes[i].bus_fmt == framefmt->code)) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int sc1330_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc1330 *sc1330 = to_sc1330(sd);
	const struct sc1330_mode *mode = sc1330->cur_mode;

	mutex_lock(&sc1330->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sc1330->mutex);
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
	mutex_unlock(&sc1330->mutex);

	return 0;
}

static int sc1330_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc1330 *sc1330 = to_sc1330(sd);
	const struct sc1330_mode *mode;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;

	mutex_lock(&sc1330->mutex);

	mode = sc1330_find_best_fit(sc1330, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sc1330->mutex);
		return -ENOTTY;
#endif
	} else {
		sc1330_change_mode(sc1330, mode);
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc1330->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc1330->vblank, vblank_def,
					 SC1330_VTS_MAX - mode->height,
					 1, vblank_def);
		__v4l2_ctrl_s_ctrl(sc1330->link_freq, mode->dvp_freq_idx);
		pixel_rate = (u32)link_freq_items[mode->dvp_freq_idx] /
			     mode->bpp * SC1330_DVP_BITS;
		__v4l2_ctrl_s_ctrl_int64(sc1330->pixel_rate, pixel_rate);
	}

	mutex_unlock(&sc1330->mutex);

	return 0;
}

static int sc1330_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct sc1330 *sc1330 = to_sc1330(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = sc1330->cur_mode->bus_fmt;

	return 0;
}

static int sc1330_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct sc1330 *sc1330 = to_sc1330(sd);

	if (fse->index >= sc1330->cfg_num)
		return -EINVAL;

	if (fse->code != supported_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int sc1330_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	struct sc1330 *sc1330 = to_sc1330(sd);

	if (fie->index >= sc1330->cfg_num)
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static int sc1330_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct sc1330 *sc1330 = to_sc1330(sd);
	const struct sc1330_mode *mode = sc1330->cur_mode;

	mutex_lock(&sc1330->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&sc1330->mutex);

	return 0;
}

static int sc1330_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	struct sc1330 *sc1330 = to_sc1330(sd);
	const struct sc1330_mode *mode = sc1330->cur_mode;
	u32 val = 0;

	if (mode->hdr_mode == NO_HDR)
		val = V4L2_MBUS_HSYNC_ACTIVE_HIGH |
		      V4L2_MBUS_VSYNC_ACTIVE_LOW |
		      V4L2_MBUS_PCLK_SAMPLE_RISING;

	config->type = V4L2_MBUS_PARALLEL;
	config->flags = val;

	return 0;
}

static void sc1330_get_gain_reg(u32 val, u32 *again_reg, u32 *again_fine_reg)
{
	if (val < 0x40) {
		val = 0x40;
		*again_reg = 0x03;
		*again_fine_reg = 0x40;
	}
	if (val < 0x80) {/* 1x ~ 2x gain */
		*again_reg = 0x03;
		*again_fine_reg = val;
	} else if (val < 0x100) {/* 2x ~ 4x gain */
		*again_reg = 0x07;
		*again_fine_reg = val >> 1;
	} else if (val < 0x158) {/* 4x ~ 5.375x gain */
		*again_reg = 0x0f;
		*again_fine_reg = val >> 2;
	} else if (val < 0x2b0) {/* 5.375x ~ 10.791x gain */
		*again_reg = 0x23;
		*again_fine_reg = (val * 0x40) / 0x158;
	} else if (val < 0x560) {/* 10.791x ~ 21.582x gain */
		*again_reg = 0x27;
		*again_fine_reg = ((val * 0x40) / 0x158) >> 1;
	} else if (val < 0xac0) {/* 21.582x ~ 43.164x gain */
		*again_reg = 0x2f;
		*again_fine_reg = ((val * 0x40) / 0x158) >> 2;
	} else if (val < 0x1580) {/* 43.164x ~ 86.328x gain */
		*again_reg = 0x3f;
		*again_fine_reg = ((val * 0x40) / 0x158) >> 3;
	}
}

static long sc1330_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc1330 *sc1330 = to_sc1330(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;

	long ret = 0;

	u32 stream;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		sc1330_get_module_inf(sc1330, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = sc1330->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);
		if (stream)
			ret = sc1330_write_reg(sc1330->client,
					       SC1330_REG_CTRL_MODE,
					       SC1330_REG_VALUE_08BIT,
					       SC1330_MODE_STREAMING);
		else
			ret = sc1330_write_reg(sc1330->client,
					       SC1330_REG_CTRL_MODE,
					       SC1330_REG_VALUE_08BIT,
					       SC1330_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sc1330_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	long ret = 0;
	u32 cg = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc1330_ioctl(sd, cmd, inf);
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

		ret = sc1330_ioctl(sd, cmd, hdr);
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

		sc1330_ioctl(sd, cmd, hdr);
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

		sc1330_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;
	case RKMODULE_SET_CONVERSION_GAIN:
		if (copy_from_user(&cg, up, sizeof(cg)))
			return -EFAULT;

		sc1330_ioctl(sd, cmd, &cg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;

		sc1330_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __sc1330_start_stream(struct sc1330 *sc1330)
{
	int ret;

	ret = sc1330_write_array(sc1330->client, sc1330->cur_mode->reg_list);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_handler_setup(&sc1330->ctrl_handler);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	if (sc1330->has_init_exp && sc1330->cur_mode->hdr_mode != NO_HDR) {
		ret = sc1330_ioctl(&sc1330->subdev, PREISP_CMD_SET_HDRAE_EXP,
				   &sc1330->init_hdrae_exp);
		if (ret) {
			dev_err(&sc1330->client->dev,
				"init exp fail in hdr mode\n");
			return ret;
		}
	}

	return sc1330_write_reg(sc1330->client,
				SC1330_REG_CTRL_MODE,
				SC1330_REG_VALUE_08BIT,
				SC1330_MODE_STREAMING);
}

static int __sc1330_stop_stream(struct sc1330 *sc1330)
{
	sc1330->has_init_exp = false;
	return sc1330_write_reg(sc1330->client,
				SC1330_REG_CTRL_MODE,
				SC1330_REG_VALUE_08BIT,
				SC1330_MODE_SW_STANDBY);
}

static int sc1330_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sc1330 *sc1330 = to_sc1330(sd);
	struct i2c_client *client = sc1330->client;
	int ret = 0;

	mutex_lock(&sc1330->mutex);
	on = !!on;
	if (on == sc1330->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __sc1330_start_stream(sc1330);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__sc1330_stop_stream(sc1330);
		pm_runtime_put(&client->dev);
	}

	sc1330->streaming = on;

unlock_and_return:
	mutex_unlock(&sc1330->mutex);

	return ret;
}

static int sc1330_s_power(struct v4l2_subdev *sd, int on)
{
	struct sc1330 *sc1330 = to_sc1330(sd);
	struct i2c_client *client = sc1330->client;
	int ret = 0;

	mutex_lock(&sc1330->mutex);

	/* If the power state is not modified - no work to do. */
	if (sc1330->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret |= sc1330_write_reg(sc1330->client,
					SC1330_SOFTWARE_RESET_REG,
					SC1330_REG_VALUE_08BIT,
					0x01);
		usleep_range(100, 200);

		sc1330->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		sc1330->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&sc1330->mutex);

	return ret;
}

static int __sc1330_power_on(struct sc1330 *sc1330)
{
	int ret;
	struct device *dev = &sc1330->client->dev;

	if (!IS_ERR_OR_NULL(sc1330->pins_default)) {
		ret = pinctrl_select_state(sc1330->pinctrl,
					   sc1330->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(sc1330->xvclk, SC1330_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(sc1330->xvclk) != SC1330_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(sc1330->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(sc1330->reset_gpio))
		gpiod_set_value_cansleep(sc1330->reset_gpio, 1);

	ret = regulator_bulk_enable(SC1330_NUM_SUPPLIES, sc1330->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(sc1330->pwdn_gpio))
		gpiod_set_value_cansleep(sc1330->pwdn_gpio, 0);

	usleep_range(500, 1000);
	if (!IS_ERR(sc1330->reset_gpio))
		gpiod_set_value_cansleep(sc1330->reset_gpio, 0);

	usleep_range(500, 1000);
	if (!IS_ERR(sc1330->pwdn_gpio))
		gpiod_set_value_cansleep(sc1330->pwdn_gpio, 0);
	usleep_range(4000, 8000);

	return 0;

disable_clk:
	clk_disable_unprepare(sc1330->xvclk);

	return ret;
}

static void __sc1330_power_off(struct sc1330 *sc1330)
{
	int ret;
	struct device *dev = &sc1330->client->dev;

	if (!IS_ERR(sc1330->pwdn_gpio))
		gpiod_set_value_cansleep(sc1330->pwdn_gpio, 1);
	clk_disable_unprepare(sc1330->xvclk);
	if (!IS_ERR(sc1330->reset_gpio))
		gpiod_set_value_cansleep(sc1330->reset_gpio, 1);
	if (!IS_ERR_OR_NULL(sc1330->pins_sleep)) {
		ret = pinctrl_select_state(sc1330->pinctrl,
					   sc1330->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(SC1330_NUM_SUPPLIES, sc1330->supplies);
}

static int sc1330_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc1330 *sc1330 = to_sc1330(sd);

	return __sc1330_power_on(sc1330);
}

static int sc1330_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc1330 *sc1330 = to_sc1330(sd);

	__sc1330_power_off(sc1330);

	return 0;
}

static const struct dev_pm_ops sc1330_pm_ops = {
	SET_RUNTIME_PM_OPS(sc1330_runtime_suspend,
			   sc1330_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sc1330_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc1330 *sc1330 = to_sc1330(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sc1330_mode *def_mode = &supported_modes[0];

	mutex_lock(&sc1330->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&sc1330->mutex);
	/* No crop or compose */

	return 0;
}
#endif

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sc1330_internal_ops = {
	.open = sc1330_open,
};
#endif

static const struct v4l2_subdev_core_ops sc1330_core_ops = {
	.s_power = sc1330_s_power,
	.ioctl = sc1330_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc1330_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc1330_video_ops = {
	.s_stream = sc1330_s_stream,
	.g_frame_interval = sc1330_g_frame_interval,
	.g_mbus_config = sc1330_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops sc1330_pad_ops = {
	.enum_mbus_code = sc1330_enum_mbus_code,
	.enum_frame_size = sc1330_enum_frame_sizes,
	.enum_frame_interval = sc1330_enum_frame_interval,
	.get_fmt = sc1330_get_fmt,
	.set_fmt = sc1330_set_fmt,
};

static const struct v4l2_subdev_ops sc1330_subdev_ops = {
	.core	= &sc1330_core_ops,   /* v4l2_subdev_core_ops sc1330_core_ops */
	.video	= &sc1330_video_ops,  /* */
	.pad	= &sc1330_pad_ops,    /* */
};

static int sc1330_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc1330 *sc1330 = container_of(ctrl->handler,
					     struct sc1330, ctrl_handler);
	struct i2c_client *client = sc1330->client;
	s64 max;
	u32 again = 0, again_fine = 0x80;
	int ret = 0;
	u32 val;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sc1330->cur_mode->height + ctrl->val - 3;
		__v4l2_ctrl_modify_range(sc1330->exposure,
					 sc1330->exposure->minimum, max,
					 sc1330->exposure->step,
					 sc1330->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev)) {
		dev_err(&client->dev, "pm_runtime_get_if_in_use return\n");
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		if (sc1330->cur_mode->hdr_mode != NO_HDR)
			return ret;
		val = ctrl->val << 1;
		ret = sc1330_write_reg(sc1330->client,
				       SC1330_REG_EXP_LONG_L,
				       SC1330_REG_VALUE_08BIT,
				       (val << 4 & 0XF0));
		ret |= sc1330_write_reg(sc1330->client,
					SC1330_REG_EXP_LONG_M,
					SC1330_REG_VALUE_08BIT,
					(val >> 4 & 0XFF));
		ret |= sc1330_write_reg(sc1330->client,
					SC1330_REG_EXP_LONG_H,
					SC1330_REG_VALUE_08BIT,
					(val >> 12 & 0X0F));
		dev_dbg(&client->dev, "set exposure 0x%x\n", val);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		if (sc1330->cur_mode->hdr_mode != NO_HDR)
			return ret;
		sc1330_get_gain_reg(ctrl->val, &again, &again_fine);
		dev_dbg(&client->dev,
			"recv:%d set again 0x%x, again_fine 0x%x\n",
			ctrl->val, again, again_fine);

		ret |= sc1330_write_reg(sc1330->client,
					SC1330_REG_AGAIN,
					SC1330_REG_VALUE_08BIT,
					again);
		ret |= sc1330_write_reg(sc1330->client,
					SC1330_REG_AGAIN_FINE,
					SC1330_REG_VALUE_08BIT,
					again_fine);
		ret |= sc1330_write_reg(sc1330->client,
					SC1330_REG_DGAIN,
					SC1330_REG_VALUE_08BIT,
					0x00);
		ret |= sc1330_write_reg(sc1330->client,
					SC1330_REG_DGAIN_FINE,
					SC1330_REG_VALUE_08BIT,
					0x80);
		break;
	case V4L2_CID_VBLANK:
		ret = sc1330_write_reg(sc1330->client, SC1330_REG_VTS,
				       SC1330_REG_VALUE_16BIT,
				       ctrl->val + sc1330->cur_mode->height);
		dev_dbg(&client->dev, "set vblank 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = sc1330_enable_test_pattern(sc1330, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = sc1330_read_reg(sc1330->client, SC1330_FLIP_REG,
				      SC1330_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC1330_MIRROR_MASK;
		else
			val &= ~SC1330_MIRROR_MASK;
		ret |= sc1330_write_reg(sc1330->client, SC1330_FLIP_REG,
					SC1330_REG_VALUE_08BIT, val);
		break;
	case V4L2_CID_VFLIP:
		ret = sc1330_read_reg(sc1330->client, SC1330_FLIP_REG,
				      SC1330_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC1330_FLIP_MASK;
		else
			val &= ~SC1330_FLIP_MASK;
		ret |= sc1330_write_reg(sc1330->client, SC1330_FLIP_REG,
					SC1330_REG_VALUE_08BIT, val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sc1330_ctrl_ops = {
	.s_ctrl = sc1330_set_ctrl,
};

static int sc1330_check_sensor_id(struct sc1330 *sc1330,
				  struct i2c_client *client)
{
	struct device *dev = &sc1330->client->dev;
	u32 id = 0;
	int ret;


	ret = sc1330_read_reg(client, SC1330_REG_CHIP_ID,
			      SC1330_REG_VALUE_16BIT, &id);
	if (id != SC1330_CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%04x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected SC%04x sensor\n", SC1330_CHIP_ID);

	return 0;
}

static int sc1330_initialize_controls(struct sc1330 *sc1330)
{
	const struct sc1330_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 pixel_rate = 0;

	handler = &sc1330->ctrl_handler;
	mode = sc1330->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &sc1330->mutex;

	sc1330->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
					V4L2_CID_LINK_FREQ,
					ARRAY_SIZE(link_freq_items) - 1, 0,
					link_freq_items);
	__v4l2_ctrl_s_ctrl(sc1330->link_freq, mode->dvp_freq_idx);

	/* pixel rate = link frequency * bits / BITS_PER_SAMPLE */
	pixel_rate = (u32)link_freq_items[mode->dvp_freq_idx] / mode->bpp *
		     SC1330_DVP_BITS;
	sc1330->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
					       V4L2_CID_PIXEL_RATE, 0,
					       SC1330_MAX_PIXEL_RATE,
					       1, pixel_rate);

	if (handler->error) {
		ret = handler->error;
		dev_err(&sc1330->client->dev,
			"Failed to V4L2_CID_PIXEL_RATE controls(%d)\n", ret);
	}

	h_blank = mode->hts_def - mode->width;
	sc1330->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (sc1330->hblank)
		sc1330->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (handler->error) {
		ret = handler->error;
		dev_err(&sc1330->client->dev,
			"Failed to V4L2_CID_HBLANK controls(%d)\n", ret);
	}

	vblank_def = mode->vts_def - mode->height;
	sc1330->vblank = v4l2_ctrl_new_std(handler, &sc1330_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   SC1330_VTS_MAX - mode->height,
					   1, vblank_def);

	if (handler->error) {
		ret = handler->error;
		dev_err(&sc1330->client->dev,
			"Failed to V4L2_CID_VBLANK controls(%d)\n", ret);
	}

	exposure_max = mode->vts_def - 3;
	sc1330->exposure = v4l2_ctrl_new_std(handler, &sc1330_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     SC1330_EXPOSURE_MIN,
					     exposure_max,
					     SC1330_EXPOSURE_STEP,
					     mode->exp_def);

	if (handler->error) {
		ret = handler->error;
		dev_err(&sc1330->client->dev,
			"Failed to V4L2_CID_EXPOSURE controls(%d)\n", ret);
	}

	sc1330->anal_gain = v4l2_ctrl_new_std(handler, &sc1330_ctrl_ops,
					      V4L2_CID_ANALOGUE_GAIN,
					      SC1330_GAIN_MIN,
					      SC1330_GAIN_MAX,
					      SC1330_GAIN_STEP,
					      SC1330_GAIN_DEFAULT);

	if (handler->error) {
		ret = handler->error;
		dev_err(&sc1330->client->dev,
			"Failed to V4L2_CID_ANALOGUE_GAIN controls(%d)\n", ret);
	}

	if (handler->error) {
		ret = handler->error;
		dev_err(&sc1330->client->dev,
			"Failed to V4L2_CID_TEST_PATTERN controls(%d)\n", ret);
	}

	v4l2_ctrl_new_std(handler,
			  &sc1330_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler,
			  &sc1330_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (handler->error) {
		ret = handler->error;
		dev_err(&sc1330->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	sc1330->subdev.ctrl_handler = handler;
	sc1330->has_init_exp = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int sc1330_configure_regulators(struct sc1330 *sc1330)
{
	unsigned int i;

	for (i = 0; i < SC1330_NUM_SUPPLIES; i++)
		sc1330->supplies[i].supply = sc1330_supply_names[i];

	return devm_regulator_bulk_get(&sc1330->client->dev,
				       SC1330_NUM_SUPPLIES,
				       sc1330->supplies);
}

static int sc1330_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sc1330 *sc1330;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	u32 hdr_mode = 0;

	dev_info(dev, " driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	sc1330 = devm_kzalloc(dev, sizeof(*sc1330), GFP_KERNEL);
	if (!sc1330)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sc1330->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sc1330->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sc1330->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sc1330->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, OF_CAMERA_HDR_MODE,
			&hdr_mode);

	if (ret) {
		hdr_mode = NO_HDR;
		dev_warn(dev, " Get hdr mode failed! no hdr default\n");
	}

	sc1330->cur_mode = &supported_modes[0];
	sc1330->client = client;

	sc1330->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sc1330->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	sc1330->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sc1330->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	sc1330->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(sc1330->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	sc1330->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(sc1330->pinctrl)) {
		sc1330->pins_default =
			pinctrl_lookup_state(sc1330->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sc1330->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		sc1330->pins_sleep =
			pinctrl_lookup_state(sc1330->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(sc1330->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = sc1330_configure_regulators(sc1330);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&sc1330->mutex);

	sd = &sc1330->subdev;
	v4l2_i2c_subdev_init(sd, client, &sc1330_subdev_ops);
	ret = sc1330_initialize_controls(sc1330);
	if (ret)
		goto err_destroy_mutex;

	ret = __sc1330_power_on(sc1330);
	if (ret)
		goto err_free_handler;

	ret = sc1330_check_sensor_id(sc1330, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sc1330_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	sc1330->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sc1330->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(sc1330->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sc1330->module_index, facing,
		 SC1330_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);
#ifdef USED_SYS_DEBUG
	add_sysfs_interfaces(dev);
#endif
	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__sc1330_power_off(sc1330);
err_free_handler:
	v4l2_ctrl_handler_free(&sc1330->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sc1330->mutex);

	return ret;
}

static int sc1330_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc1330 *sc1330 = to_sc1330(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sc1330->ctrl_handler);
	mutex_destroy(&sc1330->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sc1330_power_off(sc1330);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sc1330_of_match[] = {
	{ .compatible = "smartsens,sc1330" },
	{ },
};
MODULE_DEVICE_TABLE(of, sc1330_of_match);
#endif

static const struct i2c_device_id sc1330_match_id[] = {
	{ "smartsens,sc1330", 0 },
	{ },
};

static struct i2c_driver sc1330_i2c_driver = {
	.driver = {
		.name = SC1330_NAME,
		.pm = &sc1330_pm_ops,
		.of_match_table = of_match_ptr(sc1330_of_match),
	},
	.probe		= &sc1330_probe,
	.remove		= &sc1330_remove,
	.id_table	= sc1330_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sc1330_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sc1330_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Smartsens sc1330 sensor driver");
MODULE_LICENSE("GPL v2");
