// SPDX-License-Identifier: GPL-2.0
/*
 * sc035hgs driver
 *
 * Copyright (C) 2021 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version.
 * V0.0X01.0X01 fix time sequence error when streaming on.
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
#include <linux/rk-camera-module.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define CHIP_ID						0x00310b
#define SC035HGS_REG_CHIP_ID		0x3107

#define SC035HGS_LANES			1
#define SC035HGS_BITS_PER_SAMPLE		10
#define SC035HGS_LINK_FREQ_425MHZ	425000000 //850Mbps
#define SC035HGS_PIXEL_RATE		(SC035HGS_LINK_FREQ_425MHZ * 2 * \
					SC035HGS_LANES / SC035HGS_BITS_PER_SAMPLE)
#define SC035HGS_XVCLK_FREQ		24000000

#define SC035HGS_REG_CTRL_MODE		0x0100
#define SC035HGS_MODE_SW_STANDBY		0x0
#define SC035HGS_MODE_STREAMING		BIT(0)

#define SC035HGS_REG_EXPOSURE		0x3e01
#define	SC035HGS_EXPOSURE_MIN		6
#define	SC035HGS_EXPOSURE_STEP		1
#define SC035HGS_REG_VTS			0x320e
#define SC035HGS_VTS_MAX			0xffff

#define SC035HGS_REG_COARSE_DGAIN	0x3e06
#define SC035HGS_REG_FINE_DGAIN		0x3e07
#define SC035HGS_REG_COARSE_AGAIN	0x3e08
#define SC035HGS_REG_FINE_AGAIN		0x3e09
#define	ANALOG_GAIN_MIN			0x10
#define	ANALOG_GAIN_MAX			0x7c0
#define	ANALOG_GAIN_STEP		1
#define	ANALOG_GAIN_DEFAULT		0x10

#define SC035HGS_REG_TEST_PATTERN		0x4501
#define	SC035HGS_TEST_PATTERN_ENABLE	0xcc
#define	SC035HGS_TEST_PATTERN_DISABLE	0xc4

#define SC035HGS_REG_FLIP_MIRROR	0x3221
#define SC035HGS_MIRROR_MASK		0x06
#define SC035HGS_FLIP_MASK			0x60

#define SC035HGS_GROUP_HOLD			0x3812
#define SC035HGS_GROUP_HOLD_START	0X00
#define SC035HGS_GROUP_HOLD_LUNCH	0x30
#define REG_NULL			0xFFFF

#define SC035HGS_REG_VALUE_08BIT		1
#define SC035HGS_REG_VALUE_16BIT		2
#define SC035HGS_REG_VALUE_24BIT		3

#define SC035HGS_NAME			"sc035hgs"

static const char * const sc035hgs_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SC035HGS_NUM_SUPPLIES ARRAY_SIZE(sc035hgs_supply_names)

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

struct sc035hgs_mode {
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

struct sc035hgs {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[SC035HGS_NUM_SUPPLIES];
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
	const struct sc035hgs_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_sc035hgs(sd) container_of(sd, struct sc035hgs, subdev)

static const struct regval image_optimize_gain_1x_2x_regs[] = {
	{0x3314, 0x1e},
	{0x3317, 0x1b},
	{0x3631, 0x58},
	{0x3329, 0x3c},
	{0x332d, 0x3c},
	{0x332f, 0x40},
	{0x3335, 0x44},
	{0x3344, 0x44},
	{0x3316, 0x48},
	{0x3630, 0x4a},
	{REG_NULL, 0x00}
};

static const struct regval image_optimize_gain_2x_4x_regs[] = {
	{0x3314, 0x6f},
	{0x3317, 0x10},
	{0x3631, 0x48},
	{0x3329, 0x5c},
	{0x332d, 0x5c},
	{0x332f, 0x60},
	{0x3335, 0x64},
	{0x3344, 0x64},
	{0x3316, 0x68},
	{0x3630, 0x4c},
	{REG_NULL, 0x00}
};

static const struct regval image_optimize_regs[] = {
	{0x3314, 0x76},
	{0x3317, 0x15},
	{0x3631, 0x48},
	{0x3329, 0x5c},
	{0x332d, 0x5c},
	{0x332f, 0x60},
	{0x3335, 0x64},
	{0x3344, 0x64},
	{0x3316, 0x68},
	{0x3630, 0x4c},
	{REG_NULL, 0x00}
};


/*
 * Xclk 24Mhz
 * Pclk 45Mhz
 * linelength 683(0x2ab)
 * framelength 878(0x36e)
 * grabwindow_width 640
 * grabwindow_height 480
 * max_framerate 120fps
 * mipi_datarate per lane 850Mbps
 */
static const struct regval sc035hgs_640x480_120fps_1lane_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x36f9, 0x80},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x300f, 0x0f},
	{0x3018, 0x13},
	{0x3019, 0xfc},
	{0x301c, 0x78},
	{0x301f, 0x8b},
	{0x3031, 0x0a},
	{0x3037, 0x20},
	{0x303f, 0x01},
	{0x320c, 0x05},
	{0x320d, 0x54},
	{0x320e, 0x02},
	{0x320f, 0x10},
	{0x3217, 0x00},
	{0x3218, 0x00},
	{0x3220, 0x10},
	{0x3223, 0x48},
	{0x3226, 0x74},
	{0x3227, 0x07},
	{0x323b, 0x00},
	{0x3250, 0xf0},
	{0x3251, 0x02},
	{0x3252, 0x02},
	{0x3253, 0x08},
	{0x3254, 0x02},
	{0x3255, 0x07},
	{0x3304, 0x48},
	{0x3305, 0x00},
	{0x3306, 0x98},
	{0x3309, 0x50},
	{0x330a, 0x01},
	{0x330b, 0x18},
	{0x330c, 0x18},
	{0x330f, 0x40},
	{0x3310, 0x10},
	{0x3314, 0x1e},
	{0x3315, 0x30},
	{0x3316, 0x48},
	{0x3317, 0x1b},
	{0x3329, 0x3c},
	{0x332d, 0x3c},
	{0x332f, 0x40},
	{0x3335, 0x44},
	{0x3344, 0x44},
	{0x335b, 0x80},
	{0x335f, 0x80},
	{0x3366, 0x06},
	{0x3385, 0x31},
	{0x3387, 0x39},
	{0x3389, 0x01},
	{0x33b1, 0x03},
	{0x33b2, 0x06},
	{0x33bd, 0xe0},
	{0x33bf, 0x10},
	{0x3621, 0xa4},
	{0x3622, 0x05},
	{0x3624, 0x47},
	{0x3630, 0x4a},
	{0x3631, 0x68},
	{0x3633, 0x52},
	{0x3635, 0x03},
	{0x3636, 0x25},
	{0x3637, 0x8a},
	{0x3638, 0x0f},
	{0x3639, 0x08},
	{0x363a, 0x00},
	{0x363b, 0x48},
	{0x363c, 0x86},
	{0x363e, 0xf8},
	{0x3640, 0x00},
	{0x3641, 0x01},
	{0x36ea, 0x2f},
	{0x36eb, 0x0e},
	{0x36ec, 0x0e},
	{0x36ed, 0x20},
	{0x36fa, 0x2f},
	{0x36fb, 0x10},
	{0x36fc, 0x02},
	{0x36fd, 0x00},
	{0x3908, 0x91},
	{0x391b, 0x81},
	{0x3d08, 0x01},
	{0x3e01, 0x18},
	{0x3e02, 0xf0},
	{0x3e03, 0x2b},
	{0x3e06, 0x0c},
	{0x3f04, 0x03},
	{0x3f05, 0x80},
	{0x4500, 0x59},
	{0x4501, 0xc4},
	{0x4603, 0x00},
	{0x4800, 0x64},
	{0x4809, 0x01},
	{0x4810, 0x00},
	{0x4811, 0x01},
	{0x4837, 0x18},
	{0x5011, 0x00},
	{0x5988, 0x02},
	{0x598e, 0x05},
	{0x598f, 0x17},
	{0x36e9, 0x62},
	{0x36f9, 0x62},
	{REG_NULL, 0x00}
};

static const struct sc035hgs_mode supported_modes[] = {
	{
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 640,
		.height = 480,
		.max_fps = {
			.numerator = 10000,
			.denominator = 1200000,
		},
		.exp_def = 0x018f,
		.hts_def = 0x0554,
		.vts_def = 0x0210,
		.reg_list = sc035hgs_640x480_120fps_1lane_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const char * const sc035hgs_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

static const s64 link_freq_menu_items[] = {
	SC035HGS_LINK_FREQ_425MHZ
};

/* Write registers up to 4 at a time */
static int sc035hgs_write_reg(struct i2c_client *client,
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

static int sc035hgs_write_array(struct i2c_client *client,
	const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		ret = sc035hgs_write_reg(client, regs[i].addr,
				       SC035HGS_REG_VALUE_08BIT, regs[i].val);
		if (regs[i].addr == 0x0100 && regs[i].val == 0x01)
			usleep_range(10 * 1000, 20 * 1000);
	}

	return ret;
}

/* Read registers up to 4 at a time */
static int sc035hgs_read_reg(struct i2c_client *client,
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

static int sc035hgs_get_reso_dist(const struct sc035hgs_mode *mode,
	struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct sc035hgs_mode *
sc035hgs_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = sc035hgs_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int sc035hgs_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc035hgs *sc035hgs = to_sc035hgs(sd);
	const struct sc035hgs_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&sc035hgs->mutex);

	mode = sc035hgs_find_best_fit(fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sc035hgs->mutex);
		return -ENOTTY;
#endif
	} else {
		sc035hgs->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc035hgs->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc035hgs->vblank, vblank_def,
					 SC035HGS_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&sc035hgs->mutex);

	return 0;
}

static int sc035hgs_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc035hgs *sc035hgs = to_sc035hgs(sd);
	const struct sc035hgs_mode *mode = sc035hgs->cur_mode;

	mutex_lock(&sc035hgs->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sc035hgs->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&sc035hgs->mutex);

	return 0;
}

static int sc035hgs_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct sc035hgs *sc035hgs = to_sc035hgs(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = sc035hgs->cur_mode->bus_fmt;

	return 0;
}

static int sc035hgs_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct sc035hgs *sc035hgs = to_sc035hgs(sd);

	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != sc035hgs->cur_mode->bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int sc035hgs_enable_test_pattern(struct sc035hgs *sc035hgs, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | SC035HGS_TEST_PATTERN_ENABLE;
	else
		val = SC035HGS_TEST_PATTERN_DISABLE;

	return sc035hgs_write_reg(sc035hgs->client, SC035HGS_REG_TEST_PATTERN,
				  SC035HGS_REG_VALUE_08BIT, val);
}

static void sc035hgs_get_module_inf(struct sc035hgs *sc035hgs,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, SC035HGS_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, sc035hgs->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, sc035hgs->len_name, sizeof(inf->base.lens));
}

static long sc035hgs_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc035hgs *sc035hgs = to_sc035hgs(sd);
	struct rkmodule_hdr_cfg *hdr;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		sc035hgs_get_module_inf(sc035hgs, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = sc035hgs->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);
		if (stream)
			ret = sc035hgs_write_reg(sc035hgs->client, SC035HGS_REG_CTRL_MODE,
						 SC035HGS_REG_VALUE_08BIT,
						 SC035HGS_MODE_STREAMING);
		else
			ret = sc035hgs_write_reg(sc035hgs->client, SC035HGS_REG_CTRL_MODE,
						 SC035HGS_REG_VALUE_08BIT,
						 SC035HGS_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sc035hgs_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_hdr_cfg *hdr;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc035hgs_ioctl(sd, cmd, inf);
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

		ret = sc035hgs_ioctl(sd, cmd, hdr);
		if (!ret) {
			ret = copy_to_user(up, hdr, sizeof(*hdr));
			if (ret)
				ret = -EFAULT;
		}
		kfree(hdr);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;

		ret = sc035hgs_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int sc035hgs_set_ctrl_gain(struct sc035hgs *sc035hgs, u32 a_gain)
{
	struct device *dev = &sc035hgs->client->dev;
	u32 coarse_again = 0, fine_again = 0;
	u32 coarse_dgain = 0, fine_dgain = 0;
	int ret = 0;

	if (a_gain < 0x20) { /*1x ~ 2x*/
		fine_again = a_gain;
		coarse_again = 0x3;
		fine_dgain = 0x80;
		coarse_dgain = 0x0c;
	} else if (a_gain < 0x40) { /*2x ~ 4x*/
		fine_again = a_gain >> 1;
		coarse_again = 0x7;
		fine_dgain = 0x80;
		coarse_dgain = 0x0c;
	} else if (a_gain < 0x80) { /*4x ~ 8x*/
		fine_again = a_gain >> 2;
		coarse_again = 0xf;
		fine_dgain = 0x80;
		coarse_dgain = 0x0c;
	} else if (a_gain <= 0xf8) { /*8x ~ 15.5x*/
		fine_again = a_gain >> 3;
		coarse_again = 0x1f;
		fine_dgain = 0x80;
		coarse_dgain = 0x0c;
	} else if (a_gain < 0x200) { /*15.5x ~ 32x*/
		fine_again = 0x1f;
		coarse_again = 0x1f;
		fine_dgain = a_gain >> 1;
		coarse_dgain = 0x0c;
	} else if (a_gain < 0x400) { /*32x ~ 64x*/
		fine_again = 0x1f;
		coarse_again = 0x1f;
		fine_dgain = a_gain >> 2;
		coarse_dgain = 0x0d;
	} else if (a_gain <= 0x7C0) { /*64x ~ 124x*/
		fine_again = 0x1f;
		coarse_again = 0x1f;
		fine_dgain = a_gain >> 3;
		coarse_dgain = 0x0f;
	}

	dev_dbg(dev, "set fine_again = 0x%x, coarse_again = 0x%x, coarse_dgain=0x%x, fine_dgain=0x%x\n",
			fine_again, coarse_again, coarse_dgain, fine_dgain);

	if (a_gain < 0x20)
		ret = sc035hgs_write_array(sc035hgs->client,
					   image_optimize_gain_1x_2x_regs);
	else if (a_gain < 0x40)
		ret |= sc035hgs_write_array(sc035hgs->client,
					    image_optimize_gain_2x_4x_regs);
	else
		ret |= sc035hgs_write_array(sc035hgs->client,
					    image_optimize_regs);

	ret |= sc035hgs_write_reg(sc035hgs->client,
				  SC035HGS_GROUP_HOLD,
				  SC035HGS_REG_VALUE_08BIT,
				  SC035HGS_GROUP_HOLD_START);
	ret |= sc035hgs_write_reg(sc035hgs->client,
				  SC035HGS_REG_COARSE_AGAIN,
				  SC035HGS_REG_VALUE_08BIT,
				  coarse_again);
	ret |= sc035hgs_write_reg(sc035hgs->client,
				  SC035HGS_REG_FINE_AGAIN,
				  SC035HGS_REG_VALUE_08BIT,
				  fine_again);
	ret |= sc035hgs_write_reg(sc035hgs->client,
				  SC035HGS_REG_COARSE_DGAIN,
				  SC035HGS_REG_VALUE_08BIT,
				  coarse_dgain);
	ret |= sc035hgs_write_reg(sc035hgs->client,
				  SC035HGS_REG_FINE_DGAIN,
				  SC035HGS_REG_VALUE_08BIT,
				  fine_dgain);
	ret |= sc035hgs_write_reg(sc035hgs->client,
				  SC035HGS_GROUP_HOLD,
				  SC035HGS_REG_VALUE_08BIT,
				  SC035HGS_GROUP_HOLD_LUNCH);

	return ret;
}

static int __sc035hgs_start_stream(struct sc035hgs *sc035hgs)
{
	int ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&sc035hgs->mutex);
	ret = v4l2_ctrl_handler_setup(&sc035hgs->ctrl_handler);
	mutex_lock(&sc035hgs->mutex);
	if (ret)
		return ret;

	ret = sc035hgs_write_reg(sc035hgs->client, SC035HGS_REG_CTRL_MODE,
				 SC035HGS_REG_VALUE_08BIT, SC035HGS_MODE_STREAMING);

	usleep_range(10 * 1000, 20 * 1000);

	ret |= sc035hgs_write_reg(sc035hgs->client, 0x4418,
				  SC035HGS_REG_VALUE_08BIT, 0x08);
	ret |= sc035hgs_write_reg(sc035hgs->client, 0x363d,
				  SC035HGS_REG_VALUE_08BIT, 0x10);
	ret |= sc035hgs_write_reg(sc035hgs->client, 0x4419,
				  SC035HGS_REG_VALUE_08BIT, 0x80);
	if (ret)
		return ret;

	return ret;
}

static int __sc035hgs_stop_stream(struct sc035hgs *sc035hgs)
{
	return sc035hgs_write_reg(sc035hgs->client, SC035HGS_REG_CTRL_MODE,
				  SC035HGS_REG_VALUE_08BIT, SC035HGS_MODE_SW_STANDBY);
}

static int sc035hgs_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sc035hgs *sc035hgs = to_sc035hgs(sd);
	struct i2c_client *client = sc035hgs->client;
	int ret = 0;

	mutex_lock(&sc035hgs->mutex);
	on = !!on;
	if (on == sc035hgs->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __sc035hgs_start_stream(sc035hgs);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__sc035hgs_stop_stream(sc035hgs);
		pm_runtime_put(&client->dev);
	}

	sc035hgs->streaming = on;

unlock_and_return:
	mutex_unlock(&sc035hgs->mutex);

	return ret;
}

static int sc035hgs_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct sc035hgs *sc035hgs = to_sc035hgs(sd);
	const struct sc035hgs_mode *mode = sc035hgs->cur_mode;

	mutex_lock(&sc035hgs->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&sc035hgs->mutex);

	return 0;
}

static int sc035hgs_s_power(struct v4l2_subdev *sd, int on)
{
	struct sc035hgs *sc035hgs = to_sc035hgs(sd);
	struct i2c_client *client = sc035hgs->client;
	int ret = 0;

	mutex_lock(&sc035hgs->mutex);

	/* If the power state is not modified - no work to do. */
	if (sc035hgs->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = sc035hgs_write_array(sc035hgs->client, sc035hgs->cur_mode->reg_list);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		sc035hgs->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		sc035hgs->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&sc035hgs->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 sc035hgs_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, SC035HGS_XVCLK_FREQ / 1000 / 1000);
}

static int __sc035hgs_power_on(struct sc035hgs *sc035hgs)
{
	int ret;
	u32 delay_us;
	struct device *dev = &sc035hgs->client->dev;

	ret = clk_set_rate(sc035hgs->xvclk, SC035HGS_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(sc035hgs->xvclk) != SC035HGS_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(sc035hgs->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	ret = regulator_bulk_enable(SC035HGS_NUM_SUPPLIES, sc035hgs->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(sc035hgs->pwdn_gpio))
		gpiod_set_value_cansleep(sc035hgs->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = sc035hgs_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(sc035hgs->xvclk);

	return ret;
}

static void __sc035hgs_power_off(struct sc035hgs *sc035hgs)
{
	if (!IS_ERR(sc035hgs->pwdn_gpio))
		gpiod_set_value_cansleep(sc035hgs->pwdn_gpio, 0);
	clk_disable_unprepare(sc035hgs->xvclk);

	regulator_bulk_disable(SC035HGS_NUM_SUPPLIES, sc035hgs->supplies);
}

static int sc035hgs_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc035hgs *sc035hgs = to_sc035hgs(sd);

	return __sc035hgs_power_on(sc035hgs);
}

static int sc035hgs_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc035hgs *sc035hgs = to_sc035hgs(sd);

	__sc035hgs_power_off(sc035hgs);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sc035hgs_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc035hgs *sc035hgs = to_sc035hgs(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sc035hgs_mode *def_mode = &supported_modes[0];

	mutex_lock(&sc035hgs->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = sc035hgs->cur_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&sc035hgs->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int sc035hgs_g_mbus_config(struct v4l2_subdev *sd,
				  struct v4l2_mbus_config *config)
{
	u32 val = 0;

	val = 1 << (SC035HGS_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	config->type = V4L2_MBUS_CSI2;
	config->flags = val;

	return 0;
}

static int sc035hgs_enum_frame_interval(struct v4l2_subdev *sd,
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

static const struct dev_pm_ops sc035hgs_pm_ops = {
	SET_RUNTIME_PM_OPS(sc035hgs_runtime_suspend,
			   sc035hgs_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sc035hgs_internal_ops = {
	.open = sc035hgs_open,
};
#endif

static const struct v4l2_subdev_core_ops sc035hgs_core_ops = {
	.s_power = sc035hgs_s_power,
	.ioctl = sc035hgs_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc035hgs_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc035hgs_video_ops = {
	.s_stream = sc035hgs_s_stream,
	.g_frame_interval = sc035hgs_g_frame_interval,
	.g_mbus_config = sc035hgs_g_mbus_config,

};

static const struct v4l2_subdev_pad_ops sc035hgs_pad_ops = {
	.enum_mbus_code = sc035hgs_enum_mbus_code,
	.enum_frame_size = sc035hgs_enum_frame_sizes,
	.enum_frame_interval = sc035hgs_enum_frame_interval,
	.get_fmt = sc035hgs_get_fmt,
	.set_fmt = sc035hgs_set_fmt,
};

static const struct v4l2_subdev_ops sc035hgs_subdev_ops = {
	.core	= &sc035hgs_core_ops,
	.video	= &sc035hgs_video_ops,
	.pad	= &sc035hgs_pad_ops,
};

static int sc035hgs_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc035hgs *sc035hgs = container_of(ctrl->handler,
					     struct sc035hgs, ctrl_handler);
	struct i2c_client *client = sc035hgs->client;
	s64 max;
	u32 val = 0;
	u32 vts = 0;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sc035hgs->cur_mode->height + ctrl->val - 6;
		__v4l2_ctrl_modify_range(sc035hgs->exposure,
					 sc035hgs->exposure->minimum, max,
					 sc035hgs->exposure->step,
					 sc035hgs->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set exposure 0x%x\n", ctrl->val);
		ret = sc035hgs_write_reg(sc035hgs->client, SC035HGS_GROUP_HOLD,
					 SC035HGS_REG_VALUE_08BIT,
					 SC035HGS_GROUP_HOLD_START);
		ret |= sc035hgs_write_reg(sc035hgs->client, SC035HGS_REG_EXPOSURE,
					  SC035HGS_REG_VALUE_16BIT, ctrl->val << 4);
		ret |= sc035hgs_write_reg(sc035hgs->client, SC035HGS_GROUP_HOLD,
					  SC035HGS_REG_VALUE_08BIT,
					  SC035HGS_GROUP_HOLD_LUNCH);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set again 0x%x\n", ctrl->val);
		ret = sc035hgs_set_ctrl_gain(sc035hgs, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		vts = ctrl->val + sc035hgs->cur_mode->height;
		dev_dbg(&client->dev, "set vts 0x%x\n", vts);
		ret = sc035hgs_write_reg(sc035hgs->client, SC035HGS_REG_VTS,
					 SC035HGS_REG_VALUE_16BIT, vts);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = sc035hgs_enable_test_pattern(sc035hgs, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = sc035hgs_read_reg(sc035hgs->client, SC035HGS_REG_FLIP_MIRROR,
					SC035HGS_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC035HGS_MIRROR_MASK;
		else
			val &= ~SC035HGS_MIRROR_MASK;
		ret |= sc035hgs_write_reg(sc035hgs->client, SC035HGS_REG_FLIP_MIRROR,
					  SC035HGS_REG_VALUE_08BIT, val);
		break;
	case V4L2_CID_VFLIP:
		ret = sc035hgs_read_reg(sc035hgs->client, SC035HGS_REG_FLIP_MIRROR,
					SC035HGS_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC035HGS_FLIP_MASK;
		else
			val &= ~SC035HGS_FLIP_MASK;
		ret |= sc035hgs_write_reg(sc035hgs->client, SC035HGS_REG_FLIP_MIRROR,
					  SC035HGS_REG_VALUE_08BIT, val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sc035hgs_ctrl_ops = {
	.s_ctrl = sc035hgs_set_ctrl,
};

static int sc035hgs_initialize_controls(struct sc035hgs *sc035hgs)
{
	const struct sc035hgs_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &sc035hgs->ctrl_handler;
	mode = sc035hgs->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &sc035hgs->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, SC035HGS_PIXEL_RATE, 1, SC035HGS_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	sc035hgs->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					     h_blank, h_blank, 1, h_blank);
	if (sc035hgs->hblank)
		sc035hgs->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	sc035hgs->vblank = v4l2_ctrl_new_std(handler, &sc035hgs_ctrl_ops,
					     V4L2_CID_VBLANK, vblank_def,
					     SC035HGS_VTS_MAX - mode->height,
					     1, vblank_def);

	exposure_max = mode->vts_def - 6;
	sc035hgs->exposure = v4l2_ctrl_new_std(handler, &sc035hgs_ctrl_ops,
					       V4L2_CID_EXPOSURE, SC035HGS_EXPOSURE_MIN,
					       exposure_max, SC035HGS_EXPOSURE_STEP,
					       mode->exp_def);

	sc035hgs->anal_gain = v4l2_ctrl_new_std(handler, &sc035hgs_ctrl_ops,
						V4L2_CID_ANALOGUE_GAIN, ANALOG_GAIN_MIN,
						ANALOG_GAIN_MAX, ANALOG_GAIN_STEP,
						ANALOG_GAIN_DEFAULT);

	v4l2_ctrl_new_std_menu_items(handler, &sc035hgs_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(sc035hgs_test_pattern_menu) - 1,
				     0, 0, sc035hgs_test_pattern_menu);

	v4l2_ctrl_new_std(handler, &sc035hgs_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);

	v4l2_ctrl_new_std(handler, &sc035hgs_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (handler->error) {
		ret = handler->error;
		dev_err(&sc035hgs->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	sc035hgs->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int sc035hgs_check_sensor_id(struct sc035hgs *sc035hgs,
				  struct i2c_client *client)
{
	struct device *dev = &sc035hgs->client->dev;
	u32 id = 0;
	int ret;

	ret = sc035hgs_read_reg(client, SC035HGS_REG_CHIP_ID,
				SC035HGS_REG_VALUE_24BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%04x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected SC035HGS CHIP ID = 0x%04x sensor\n", CHIP_ID);

	return 0;
}

static int sc035hgs_configure_regulators(struct sc035hgs *sc035hgs)
{
	unsigned int i;

	for (i = 0; i < SC035HGS_NUM_SUPPLIES; i++)
		sc035hgs->supplies[i].supply = sc035hgs_supply_names[i];

	return devm_regulator_bulk_get(&sc035hgs->client->dev,
				       SC035HGS_NUM_SUPPLIES,
				       sc035hgs->supplies);
}

static int sc035hgs_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sc035hgs *sc035hgs;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	sc035hgs = devm_kzalloc(dev, sizeof(*sc035hgs), GFP_KERNEL);
	if (!sc035hgs)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sc035hgs->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sc035hgs->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sc035hgs->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sc035hgs->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	sc035hgs->client = client;
	sc035hgs->cur_mode = &supported_modes[0];

	sc035hgs->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sc035hgs->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	sc035hgs->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sc035hgs->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	sc035hgs->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(sc035hgs->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");
	ret = sc035hgs_configure_regulators(sc035hgs);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&sc035hgs->mutex);

	sd = &sc035hgs->subdev;
	v4l2_i2c_subdev_init(sd, client, &sc035hgs_subdev_ops);
	ret = sc035hgs_initialize_controls(sc035hgs);
	if (ret)
		goto err_destroy_mutex;

	ret = __sc035hgs_power_on(sc035hgs);
	if (ret)
		goto err_free_handler;

	ret = sc035hgs_check_sensor_id(sc035hgs, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sc035hgs_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	sc035hgs->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sc035hgs->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(sc035hgs->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sc035hgs->module_index, facing,
		 SC035HGS_NAME, dev_name(sd->dev));
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
	__sc035hgs_power_off(sc035hgs);
err_free_handler:
	v4l2_ctrl_handler_free(&sc035hgs->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sc035hgs->mutex);

	return ret;
}

static int sc035hgs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc035hgs *sc035hgs = to_sc035hgs(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sc035hgs->ctrl_handler);
	mutex_destroy(&sc035hgs->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sc035hgs_power_off(sc035hgs);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sc035hgs_of_match[] = {
	{ .compatible = "smartsens,sc035hgs" },
	{},
};
MODULE_DEVICE_TABLE(of, sc035hgs_of_match);
#endif

static const struct i2c_device_id sc035hgs_match_id[] = {
	{ "smartsens,sc035hgs", 0 },
	{ },
};

static struct i2c_driver sc035hgs_i2c_driver = {
	.driver = {
		.name = SC035HGS_NAME,
		.pm = &sc035hgs_pm_ops,
		.of_match_table = of_match_ptr(sc035hgs_of_match),
	},
	.probe		= &sc035hgs_probe,
	.remove		= &sc035hgs_remove,
	.id_table	= sc035hgs_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sc035hgs_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sc035hgs_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Smartsens sc035hgs sensor driver");
MODULE_AUTHOR("zack.zeng");
MODULE_LICENSE("GPL v2");
