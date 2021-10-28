// SPDX-License-Identifier: GPL-2.0
/*
 * gc1054 sensor driver
 *
 * Copyright (C) 2021 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version.
 * V0.0X01.0X01 add quick stream on/off
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/rk-preisp.h>

#define DRIVER_VERSION          KERNEL_VERSION(0, 0x01, 0x02)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define GC1054_NAME             "gc1054"
#define GC1054_MEDIA_BUS_FMT    MEDIA_BUS_FMT_SRGGB10_1X10

#define MIPI_FREQ_297M          297000000
#define GC1054_XVCLK_FREQ       27000000

#define GC1054_PIXEL_RATE			(47250000)
#define GC1054_XVCLK_FREQ			27000000

#define GC1054_PAGE_SELECT      0xFE

#define GC1054_REG_CHIP_ID_H    0xF0
#define GC1054_REG_CHIP_ID_L    0xF1

#define GC1054_REG_EXP_H        0x03
#define GC1054_REG_EXP_L        0x04

#define GC1054_REG_VTS_H        0x41
#define GC1054_REG_VTS_L        0x42

#define GC1054_REG_CTRL_MODE    0x3E
#define GC1054_MODE_SW_STANDBY  0x11
#define GC1054_MODE_STREAMING   0x91

#define REG_NULL                0xFF

#define GC1054_CHIP_ID          0x1054

#define GC1054_VTS_MAX          0x3FFF
#define GC1054_HTS_MAX          0xFFF

#define GC1054_EXPOSURE_MAX     0x3FFF
#define GC1054_EXPOSURE_MIN     1
#define GC1054_EXPOSURE_STEP    1

#define GC1054_GAIN_MIN         0x40
#define GC1054_GAIN_MAX         0x2000
#define GC1054_GAIN_STEP        1
#define GC1054_GAIN_DEFAULT     64

#define GC1054_LANES            2

#define GC1054_REG_VALUE_08BIT		1
#define GC1054_REG_VALUE_16BIT		2
#define GC1054_REG_VALUE_24BIT		3

#define SENSOR_ID(_msb, _lsb)   ((_msb) << 8 | (_lsb))

#define GC1054_FLIP_MIRROR_REG  0x17

#define GC_MIRROR_BIT_MASK      BIT(0)
#define GC_FLIP_BIT_MASK        BIT(1)

#define PIX_FORMAT MEDIA_BUS_FMT_SRGGB10_1X10

#define GC1054_NAME			"gc1054"

static const char * const gc1054_supply_names[] = {
	"vcc2v8_dvp",		/* Analog power */
	"vcc1v8_dvp",		/* Digital I/O power */
	"vdd1v5_dvp",		/* Digital core power */
};

#define GC1054_NUM_SUPPLIES ARRAY_SIZE(gc1054_supply_names)

#define to_gc1054(sd) container_of(sd, struct gc1054, subdev)

enum gc1054_max_pad {
	PAD0,
	PAD_MAX,
};

struct regval {
	u8 addr;
	u8 val;
};

struct gc1054_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 hdr_mode;
	const struct regval *reg_list;
};

struct gc1054 {
	struct i2c_client   *client;
	struct clk      *xvclk;
	struct gpio_desc    *reset_gpio;
	struct gpio_desc    *pwdn_gpio;

	struct regulator_bulk_data supplies[GC1054_NUM_SUPPLIES];

	struct v4l2_subdev  subdev;
	struct media_pad    pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl    *exposure;
	struct v4l2_ctrl    *anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;

	struct mutex        mutex;
	bool            streaming;
	bool			power_on;
	const struct gc1054_mode *cur_mode;

	u32         module_index;
	const char      *module_facing;
	const char      *module_name;
	const char      *len_name;

};

#define to_gc1054(sd) container_of(sd, struct gc1054, subdev)

/*
 * window_size=1920*1080 mipi@2lane
 * mclk=24mhz,mipi_clk=594Mbps
 * pixel_line_total=2200,line_frame_total=1125
 * row_time=29.629us,frame_rate=30fps
 */
static const struct regval gc1054_1280x720_regs_dvp[] = {
    //window_size=1920*1080
//mclk=27mhz,pclk=47.25mhz
//pixel_line_total=2200,line_frame_total=1125
//row_time=29.629us,frame_rate=30fps
/****system****/
	{0xf2, 0x00},
	{0xf6, 0x00},
	{0xfc, 0x04},
	{0xf7, 0x01},
	{0xf8, 0x0d},
	{0xf9, 0x00},
	{0xfa, 0x80},
	{0xfc, 0x0e},
	{0xfe, 0x00},
	{0x03, 0x02},
	{0x04, 0xa6},
	{0x05, 0x03},
	{0x06, 0x94},
	{0x07, 0x00},
	{0x08, 0x0a},
	{0x09, 0x00},
	{0x0a, 0x04},
	{0x0b, 0x00},
	{0x0c, 0x00},
	{0x0d, 0x02},
	{0x0e, 0xd4},
	{0x0f, 0x05},
	{0x10, 0x08},
	{0x17, 0xc0},
	{0x18, 0x02},
	{0x19, 0x08},
	{0x1a, 0x18},
	{0x1d, 0x12},
	{0x1e, 0x50},
	{0x1f, 0x80},
	{0x21, 0x30},
	{0x23, 0xf8},
	{0x25, 0x10},
	{0x28, 0x20},
	{0x34, 0x0a},
	{0x3c, 0x10},
	{0x3d, 0x0e},
	{0xcc, 0x8e},
	{0xcd, 0x9a},
	{0xcf, 0x70},
	{0xd0, 0xa9},
	{0xd1, 0xc5},
	{0xd2, 0xed},
	{0xd8, 0x3c},
	{0xd9, 0x7a},
	{0xda, 0x12},
	{0xdb, 0x50},
	{0xde, 0x0c},
	{0xe3, 0x60},
	{0xe4, 0x78},
	{0xfe, 0x01},
	{0xe3, 0x01},
	{0xe6, 0x10},
	{0xfe, 0x01},
	{0x80, 0x50},
	{0x88, 0x23},
	{0x89, 0x03},
	{0x90, 0x01},
	{0x92, 0x02},
	{0x94, 0x03},
	{0x95, 0x02},
	{0x96, 0xd0},
	{0x97, 0x05},
	{0x98, 0x00},
	{0xfe, 0x01},
	{0x40, 0x22},
	{0x43, 0x03},
	{0x4e, 0x3c},
	{0x4f, 0x00},
	{0x60, 0x00},
	{0x61, 0x80},
	{0xfe, 0x01},
	{0xb0, 0x48},
	{0xb1, 0x01},
	{0xb2, 0x00},
	{0xb6, 0x00},
	{0xfe, 0x02},
	{0x01, 0x00},
	{0x02, 0x01},
	{0x03, 0x02},
	{0x04, 0x03},
	{0x05, 0x04},
	{0x06, 0x05},
	{0x07, 0x06},
	{0x08, 0x0e},
	{0x09, 0x16},
	{0x0a, 0x1e},
	{0x0b, 0x36},
	{0x0c, 0x3e},
	{0x0d, 0x56},
	{0xfe, 0x02},
	{0xb0, 0x00},
	{0xb1, 0x00},
	{0xb2, 0x00},
	{0xb3, 0x11},
	{0xb4, 0x22},
	{0xb5, 0x54},
	{0xb6, 0xb8},
	{0xb7, 0x60},
	{0xb9, 0x00},
	{0xba, 0xc0},
	{0xc0, 0x20},
	{0xc1, 0x2d},
	{0xc2, 0x40},
	{0xc3, 0x5b},
	{0xc4, 0x80},
	{0xc5, 0xb5},
	{0xc6, 0x00},
	{0xc7, 0x6a},
	{0xc8, 0x00},
	{0xc9, 0xd4},
	{0xca, 0x00},
	{0xcb, 0xa8},
	{0xcc, 0x00},
	{0xcd, 0x50},
	{0xce, 0x00},
	{0xcf, 0xa1},
	{0xfe, 0x02},
	{0x54, 0xf7},
	{0x55, 0xf0},
	{0x56, 0x00},
	{0x57, 0x00},
	{0x58, 0x00},
	{0x5a, 0x04},
	{0xfe, 0x04},
	{0x81, 0x8a},
	{0xfe, 0x03},
	{0x01, 0x00},
	{0x02, 0x00},
	{0x03, 0x00},
	{0x10, 0x11},
	{0x15, 0x00},
	{0x40, 0x01},
	{0x41, 0x00},
	{0xfe, 0x00},
	{0xf2, 0x0f},
	{REG_NULL, 0x00},
};

static const struct gc1054_mode supported_modes[] = {
	{
		.width = 1280,
		.height = 720,
		.max_fps = {
			.numerator = 10000,
			.denominator = 250000,
		},
		.exp_def = 0x2D0,
		.hts_def = 0x834,
		.vts_def = 0x2ee,
		.reg_list = gc1054_1280x720_regs_dvp,
		.hdr_mode = NO_HDR,
	},
};

/* sensor register write */
static int gc1054_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	buf[0] = reg & 0xFF;
	buf[1] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	dev_err(&client->dev,
		"gc1054 write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

static int gc1054_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	int i, ret = 0;

	i = 0;
	while (regs[i].addr != REG_NULL) {
		ret = gc1054_write_reg(client, regs[i].addr, regs[i].val);
		if (ret) {
			dev_err(&client->dev, "%s failed !\n", __func__);
			break;
		}
		i++;
	}

	return ret;
}

/* sensor register read */
static int gc1054_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[1];
	int ret;

	buf[0] = reg & 0xFF;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret >= 0) {
		*val = buf[0];
		return 0;
	}

	dev_err(&client->dev,
		"gc1054 read reg(0x%x val:0x%x) failed !\n", reg, *val);

	return ret;
}

static int gc1054_get_reso_dist(const struct gc1054_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct gc1054_mode *
gc1054_find_best_fit(struct gc1054 *gc1054, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = gc1054_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist <= cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int gc1054_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc1054 *gc1054 = to_gc1054(sd);
	const struct gc1054_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&gc1054->mutex);

	mode = gc1054_find_best_fit(gc1054, fmt);
	fmt->format.code = PIX_FORMAT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&gc1054->mutex);
		return -ENOTTY;
#endif
	} else {
		gc1054->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(gc1054->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(gc1054->vblank, vblank_def,
					 GC1054_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&gc1054->mutex);
	return 0;
}

static int gc1054_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc1054 *gc1054 = to_gc1054(sd);
	const struct gc1054_mode *mode = gc1054->cur_mode;

	mutex_lock(&gc1054->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&gc1054->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = PIX_FORMAT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&gc1054->mutex);
	return 0;
}

#define CROP_START(SRC, DST) (((SRC) - (DST)) / 2 / 4 * 4)
#define DST_WIDTH 1280
#define DST_HEIGHT 720

static int gc1054_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct gc1054 *gc1054 = to_gc1054(sd);

	if (sel->target == V4L2_SEL_TGT_CROP_BOUNDS) {
		sel->r.left = CROP_START(gc1054->cur_mode->width, DST_WIDTH);
		sel->r.width = DST_WIDTH;
		sel->r.top = CROP_START(gc1054->cur_mode->height, DST_HEIGHT);
		sel->r.height = DST_HEIGHT;
		return 0;
	}
	return -EINVAL;
}

static int gc1054_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = PIX_FORMAT;

	return 0;
}
static int gc1054_enum_frame_sizes(struct v4l2_subdev *sd,
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


static void gc1054_get_module_inf(struct gc1054 *gc1054,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, GC1054_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, gc1054->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, gc1054->len_name, sizeof(inf->base.lens));
}

static long gc1054_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct gc1054 *gc1054 = to_gc1054(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case PREISP_CMD_SET_HDRAE_EXP:
		return -1;
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = gc1054->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		if (hdr_cfg->hdr_mode != 0)
			ret = -1;

		break;
	case RKMODULE_SET_CONVERSION_GAIN:
		break;
	case RKMODULE_GET_MODULE_INFO:
		gc1054_get_module_inf(gc1054, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);
		if (stream)
			ret = gc1054_write_reg(gc1054->client,
					       GC1054_REG_CTRL_MODE,
					       GC1054_MODE_STREAMING);
		else
			ret = gc1054_write_reg(gc1054->client,
					       GC1054_REG_CTRL_MODE,
					       GC1054_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long gc1054_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	long ret = 0;
	u32 stream = 0;
	u32 cg = 0;

	switch (cmd) {
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = gc1054_ioctl(sd, cmd, hdr);
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

		gc1054_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;
	case RKMODULE_SET_CONVERSION_GAIN:
		if (copy_from_user(&cg, up, sizeof(cg)))
			return -EFAULT;

		gc1054_ioctl(sd, cmd, &cg);
		break;
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = gc1054_ioctl(sd, cmd, inf);
		if (!ret) {
			ret = copy_to_user(up, inf, sizeof(*inf));
			if (ret)
				ret = -EFAULT;
		}
		kfree(inf);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;

		gc1054_ioctl(sd, cmd, &stream);
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

		gc1054_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}
#endif

static int __gc1054_start_stream(struct gc1054 *gc1054)
{
	int ret;

	ret = gc1054_write_array(gc1054->client, gc1054->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&gc1054->mutex);
	ret = v4l2_ctrl_handler_setup(&gc1054->ctrl_handler);
	mutex_lock(&gc1054->mutex);
	if (ret)
		return ret;

	return gc1054_write_reg(gc1054->client, GC1054_REG_CTRL_MODE,
				GC1054_MODE_STREAMING);
}

static int __gc1054_stop_stream(struct gc1054 *gc1054)
{
	return gc1054_write_reg(gc1054->client, GC1054_REG_CTRL_MODE,
				GC1054_MODE_SW_STANDBY);
}

static int gc1054_s_stream(struct v4l2_subdev *sd, int on)
{
	struct gc1054 *gc1054 = to_gc1054(sd);
	struct i2c_client *client = gc1054->client;
	int ret = 0;

	mutex_lock(&gc1054->mutex);
	on = !!on;
	if (on == gc1054->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __gc1054_start_stream(gc1054);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__gc1054_stop_stream(gc1054);
		pm_runtime_put(&client->dev);
	}

	gc1054->streaming = on;

unlock_and_return:
	mutex_unlock(&gc1054->mutex);
	return 0;
}

static int gc1054_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct gc1054 *gc1054 = to_gc1054(sd);
	const struct gc1054_mode *mode = gc1054->cur_mode;

	mutex_lock(&gc1054->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&gc1054->mutex);

	return 0;
}

static int gc1054_s_power(struct v4l2_subdev *sd, int on)
{
	struct gc1054 *gc1054 = to_gc1054(sd);
	struct i2c_client *client = gc1054->client;
	int ret = 0;

	mutex_lock(&gc1054->mutex);

	/* If the power state is not modified - no work to do. */
	if (gc1054->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		gc1054->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		gc1054->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&gc1054->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 gc1054_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, GC1054_XVCLK_FREQ / 1000 / 1000);
}

static int __gc1054_power_on(struct gc1054 *gc1054)
{
	int ret;
	u32 delay_us;
	struct device *dev = &gc1054->client->dev;

	ret = clk_set_rate(gc1054->xvclk, GC1054_XVCLK_FREQ);
	if (ret < 0) {
		dev_err(dev, "Failed to set xvclk rate (24MHz)\n");
		return ret;
	}
	if (clk_get_rate(gc1054->xvclk) != GC1054_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(gc1054->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(gc1054->reset_gpio))
		gpiod_set_value_cansleep(gc1054->reset_gpio, 1);

	ret = regulator_bulk_enable(GC1054_NUM_SUPPLIES, gc1054->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	/* According to datasheet, at least 10ms for reset duration */
	usleep_range(10 * 1000, 15 * 1000);

	if (!IS_ERR(gc1054->pwdn_gpio))
		gpiod_set_value_cansleep(gc1054->pwdn_gpio, 0);
	usleep_range(5 * 1000, 10 * 1000);

	if (!IS_ERR(gc1054->reset_gpio))
		gpiod_set_value_cansleep(gc1054->reset_gpio, 0);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = gc1054_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(gc1054->xvclk);

	return ret;
}

static void __gc1054_power_off(struct gc1054 *gc1054)
{
	if (!IS_ERR(gc1054->pwdn_gpio))
		gpiod_set_value_cansleep(gc1054->pwdn_gpio, 1);
	clk_disable_unprepare(gc1054->xvclk);
	if (!IS_ERR(gc1054->reset_gpio))
		gpiod_set_value_cansleep(gc1054->reset_gpio, 1);
	regulator_bulk_disable(GC1054_NUM_SUPPLIES, gc1054->supplies);
}

static int gc1054_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc1054 *gc1054 = to_gc1054(sd);

	return __gc1054_power_on(gc1054);
}

static int gc1054_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc1054 *gc1054 = to_gc1054(sd);

	__gc1054_power_off(gc1054);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int gc1054_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gc1054 *gc1054 = to_gc1054(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct gc1054_mode *def_mode = &supported_modes[0];

	mutex_lock(&gc1054->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = PIX_FORMAT;
	try_fmt->field = V4L2_FIELD_NONE;
	mutex_unlock(&gc1054->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int gc1054_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	config->type = V4L2_MBUS_PARALLEL;
	config->flags = V4L2_MBUS_HSYNC_ACTIVE_HIGH |
			V4L2_MBUS_VSYNC_ACTIVE_LOW |
			V4L2_MBUS_PCLK_SAMPLE_RISING;
	return 0;
}

static int gc1054_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fie->code != PIX_FORMAT)
		return -EINVAL;

	fie->code = GC1054_MEDIA_BUS_FMT;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}
static const struct dev_pm_ops gc1054_pm_ops = {
	SET_RUNTIME_PM_OPS(gc1054_runtime_suspend,
			   gc1054_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops gc1054_internal_ops = {
	.open = gc1054_open,
};
#endif
static const struct v4l2_subdev_core_ops gc1054_core_ops = {
	.s_power = gc1054_s_power,
	.ioctl = gc1054_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = gc1054_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops gc1054_video_ops = {
	.s_stream = gc1054_s_stream,
	.g_frame_interval = gc1054_g_frame_interval,
	.g_mbus_config = gc1054_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops gc1054_pad_ops = {
	.enum_mbus_code = gc1054_enum_mbus_code,
	.enum_frame_size = gc1054_enum_frame_sizes,
	.enum_frame_interval = gc1054_enum_frame_interval,
	.get_fmt = gc1054_get_fmt,
	.set_fmt = gc1054_set_fmt,
	.get_selection = gc1054_get_selection,
};

static const struct v4l2_subdev_ops gc1054_subdev_ops = {
	.core   = &gc1054_core_ops,
	.video  = &gc1054_video_ops,
	.pad    = &gc1054_pad_ops,
};

static  uint32_t gain_level_table[12] = {
								64,
								91,
								127,
								182,
								258,
								369,
								516,
								738,
								1032,
								1491,
								2084,
								0xffffffff,
};

static int gc1054_set_gain(struct gc1054 *gc1054, u32 gain)
{

	int Analog_Index;
	u32 tol_dig_gain = 0;
	static	int total;

	total = sizeof(gain_level_table) / sizeof(u32) - 1;

	for (Analog_Index = 0; Analog_Index < total; Analog_Index++) {
		if ((gain_level_table[Analog_Index] <= gain) &&
			(gain < gain_level_table[Analog_Index+1]))
			break;
	}

	tol_dig_gain = gain*64/gain_level_table[Analog_Index];
	gc1054_write_reg(gc1054->client, 0xfe, 0x01);
	gc1054_write_reg(gc1054->client, 0xb6, Analog_Index);
	gc1054_write_reg(gc1054->client, 0xb1, (tol_dig_gain>>6));
	gc1054_write_reg(gc1054->client, 0xb2, ((tol_dig_gain&0x3f)<<2));
	gc1054_write_reg(gc1054->client, 0xfe, 0x00);

	return 0;
}

static int gc1054_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc1054 *gc1054 = container_of(ctrl->handler,
					     struct gc1054, ctrl_handler);
	struct i2c_client *client = gc1054->client;
	s64 max;
	int ret = 0;
	u32 vts = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = gc1054->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(gc1054->exposure,
					 gc1054->exposure->minimum, max,
					 gc1054->exposure->step,
					 gc1054->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = gc1054_write_reg(gc1054->client, GC1054_REG_EXP_H,
				       (ctrl->val >> 8) & 0x3f);
		ret |= gc1054_write_reg(gc1054->client, GC1054_REG_EXP_L,
					ctrl->val & 0xff);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		gc1054_set_gain(gc1054, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		vts = ctrl->val + gc1054->cur_mode->height;
		ret = gc1054_write_reg(gc1054->client,
				       GC1054_REG_VTS_H,
				       (vts >> 8) & 0x3f);
		ret |= gc1054_write_reg(gc1054->client,
					GC1054_REG_VTS_L,
					vts & 0xff);
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			__func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);
	return ret;
}

static const struct v4l2_ctrl_ops gc1054_ctrl_ops = {
	.s_ctrl = gc1054_set_ctrl,
};

static int gc1054_initialize_controls(struct gc1054 *gc1054)
{
	const struct gc1054_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &gc1054->ctrl_handler;
	mode = gc1054->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);

	if (ret)
		return ret;
	handler->lock = &gc1054->mutex;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, GC1054_PIXEL_RATE, 1, GC1054_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	gc1054->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);

	if (gc1054->hblank)
		gc1054->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	gc1054->vblank = v4l2_ctrl_new_std(handler, &gc1054_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   GC1054_VTS_MAX - mode->height,
					   1, vblank_def);

	exposure_max = mode->vts_def;
	//exposure_max = mode->vts_def - 4;
	gc1054->exposure = v4l2_ctrl_new_std(handler, &gc1054_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     GC1054_EXPOSURE_MIN,
					     exposure_max,
					     GC1054_EXPOSURE_STEP,
					     mode->exp_def);

	gc1054->anal_gain = v4l2_ctrl_new_std(handler, &gc1054_ctrl_ops,
					      V4L2_CID_ANALOGUE_GAIN,
					      GC1054_GAIN_MIN,
					      GC1054_GAIN_MAX,
					      GC1054_GAIN_STEP,
					      GC1054_GAIN_DEFAULT);

	/* Digital gain */
	gc1054->digi_gain = v4l2_ctrl_new_std(handler, &gc1054_ctrl_ops,
					      V4L2_CID_DIGITAL_GAIN,
					      GC1054_GAIN_MIN,
					      GC1054_GAIN_MAX,
					      GC1054_GAIN_STEP,
					      GC1054_GAIN_DEFAULT);

	if (handler->error) {
		ret = handler->error;
		dev_err(&gc1054->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	gc1054->subdev.ctrl_handler = handler;
//	gc1054->digi_gain = GC1054_GAIN_DEFAULT;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int gc1054_check_sensor_id(struct gc1054 *gc1054,
				  struct i2c_client *client)
{
	struct device *dev = &gc1054->client->dev;
	u8 pid = 0, ver = 0;
	u16 id = 0;
	int ret = 0;

	/* Check sensor revision */
	ret = gc1054_read_reg(client, GC1054_REG_CHIP_ID_H, &pid);
	ret |= gc1054_read_reg(client, GC1054_REG_CHIP_ID_L, &ver);
	if (ret) {
		dev_err(&client->dev, "gc1054_read_reg failed (%d)\n", ret);
		return ret;
	}
	id = SENSOR_ID(pid, ver);
	if (id != GC1054_CHIP_ID) {
		dev_err(&client->dev,
			"Sensor detection failed (%04X,%d)\n",
			id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected GC%04x sensor\n", id);
	return 0;
}

static int gc1054_configure_regulators(struct gc1054 *gc1054)
{
	u32 i;

	for (i = 0; i < GC1054_NUM_SUPPLIES; i++)
		gc1054->supplies[i].supply = gc1054_supply_names[i];

	return devm_regulator_bulk_get(&gc1054->client->dev,
				       GC1054_NUM_SUPPLIES,
				       gc1054->supplies);
}
static int gc1054_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct gc1054 *gc1054;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	gc1054 = devm_kzalloc(dev, sizeof(*gc1054), GFP_KERNEL);
	if (!gc1054)
		return -ENOMEM;

	gc1054->client = client;
	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &gc1054->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &gc1054->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &gc1054->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &gc1054->len_name);
	if (ret) {
		dev_err(dev,
			"could not get module information!\n");
		return -EINVAL;
	}

	gc1054->client = client;
	gc1054->cur_mode = &supported_modes[0];
	gc1054->xvclk = devm_clk_get(&client->dev, "xvclk");
	if (IS_ERR(gc1054->xvclk)) {
		dev_err(&client->dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	gc1054->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gc1054->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	gc1054->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(gc1054->pwdn_gpio))
		dev_info(dev, "Failed to get pwdn-gpios, maybe no used\n");

	ret = gc1054_configure_regulators(gc1054);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&gc1054->mutex);

	sd = &gc1054->subdev;
	v4l2_i2c_subdev_init(sd, client, &gc1054_subdev_ops);
	ret = gc1054_initialize_controls(gc1054);
	if (ret)
		goto err_destroy_mutex;

	ret = __gc1054_power_on(gc1054);
	if (ret)
		goto err_free_handler;

	ret = gc1054_check_sensor_id(gc1054, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &gc1054_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	gc1054->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &gc1054->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(gc1054->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 gc1054->module_index, facing,
		 GC1054_NAME, dev_name(sd->dev));

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
	__gc1054_power_off(gc1054);
err_free_handler:
	v4l2_ctrl_handler_free(&gc1054->ctrl_handler);

err_destroy_mutex:
	mutex_destroy(&gc1054->mutex);
	return ret;
}

static int gc1054_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc1054 *gc1054 = to_gc1054(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&gc1054->ctrl_handler);
	mutex_destroy(&gc1054->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__gc1054_power_off(gc1054);
	pm_runtime_set_suspended(&client->dev);
	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id gc1054_of_match[] = {
	{ .compatible = "galaxycore,gc1054" },
	{},
};
MODULE_DEVICE_TABLE(of, gc1054_of_match);
#endif

static const struct i2c_device_id gc1054_match_id[] = {
	{ "galaxycore,gc1054", 0 },
	{ },
};

static struct i2c_driver gc1054_i2c_driver = {
	.driver = {
		.name = GC1054_NAME,
		.pm = &gc1054_pm_ops,
		.of_match_table = of_match_ptr(gc1054_of_match),
	},
	.probe      = &gc1054_probe,
	.remove     = &gc1054_remove,
	.id_table   = gc1054_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&gc1054_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&gc1054_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("GC1054 CMOS Image Sensor driver");
MODULE_LICENSE("GPL v2");
