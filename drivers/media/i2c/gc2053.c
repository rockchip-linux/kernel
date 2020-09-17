// SPDX-License-Identifier: GPL-2.0
/*
 * gc2053 driver
 *
 * Copyright (C) 2019 Fuzhou Rockchip Electronics Co., Ltd.
 * v0.0x01.0x0 first version,support mipi one lane
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

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x0)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define GC2053_LINK_FREQ_420MHZ		420000000
/* pixel rate = link frequency * 1 * lanes / BITS_PER_SAMPLE */
#define GC2053_PIXEL_RATE		(GC2053_LINK_FREQ_420MHZ * 2 * 1 / 10)
#define GC2053_XVCLK_FREQ		24000000

#define CHIP_ID				0x2053
#define GC2053_REG_CHIP_ID_H		0xf0
#define GC2053_REG_CHIP_ID_L		0xf1
#define SENSOR_ID(_msb, _lsb)		((_msb) << 8 | (_lsb))

#define GC2053_PAGE_SELECT		0xfe
#define GC2053_MODE_SELECT		0x10
#define GC2053_MODE_SW_STANDBY		0x00
#define GC2053_MODE_STREAMING		0x90

#define GC2053_REG_EXPOSURE_H		0x03
#define GC2053_REG_EXPOSURE_L		0x04
#define	GC2053_EXPOSURE_MIN		4
#define	GC2053_EXPOSURE_STEP		1
#define GC2053_VTS_MAX			0x7fff

#define GC2053_GAIN_MIN			0x40
#define GC2053_GAIN_MAX			0x1b7f
#define GC2053_GAIN_STEP		1
#define GC2053_GAIN_DEFAULT		0x40

#define GC2053_REG_VTS_H		0x41
#define GC2053_REG_VTS_L		0x42

#define REG_NULL			0xFFFF

#define GC2053_LANES			1
#define GC2053_BITS_PER_SAMPLE		10

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define GC2053_NAME			"gc2053"

static const char * const gc2053_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define GC2053_NUM_SUPPLIES ARRAY_SIZE(gc2053_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct gc2053_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct gc2053 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[GC2053_NUM_SUPPLIES];

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
	const struct gc2053_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_gc2053(sd) container_of(sd, struct gc2053, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval gc2053_global_regs[] = {
	{0xfe, 0x80},
	{0xfe, 0x80},
	{0xfe, 0x80},
	{0xfe, 0x00},
	{0xf2, 0x00},
	{0xf3, 0x00},
	{0xf4, 0x36},
	{0xf5, 0xc0},
	{0xf6, 0x81},
	{0xf7, 0x01},
	{0xf8, 0x22},
	{0xf9, 0x80},
	{0xfc, 0x8e},
	{0xfe, 0x00},
	{0x87, 0x18},
	{0xee, 0x30},
	{0xd0, 0xb7},
	{0x03, 0x04},
	{0x04, 0x10},
	{0x05, 0x04},
	{0x06, 0xB8},
	{0x07, 0x00},
	{0x08, 0x19},
	{0x09, 0x00},
	{0x0a, 0x02},
	{0x0b, 0x00},
	{0x0c, 0x02},
	{0x0d, 0x04},
	{0x0e, 0x40},
	{0x12, 0xe2},
	{0x13, 0x16},
	{0x19, 0x0a},
	{0x21, 0x1c},
	{0x28, 0x0a},
	{0x29, 0x24},
	{0x2b, 0x04},
	{0x32, 0xf8},
	{0x37, 0x03},
	{0x39, 0x15},
	{0x43, 0x07},
	{0x44, 0x40},
	{0x46, 0x0b},
	{0x4b, 0x20},
	{0x4e, 0x08},
	{0x55, 0x20},
	{0x66, 0x05},
	{0x67, 0x05},
	{0x77, 0x01},
	{0x78, 0x00},
	{0x7c, 0x93},
	{0x8c, 0x12},
	{0x8d, 0x92},
	{0x90, 0x01},
	{0x9d, 0x10},
	{0xce, 0x7c},
	{0xd2, 0x41},
	{0xd3, 0xdc},
	{0xda, 0x05},
	{0xdb, 0x00},
	{0xe6, 0x50},
	{0xb6, 0xc0},
	{0xb0, 0x70},
	{0xb1, 0x01},
	{0xb2, 0x00},
	{0xb3, 0x00},
	{0xb4, 0x00},
	{0xb8, 0x01},
	{0xb9, 0x00},
	{0x26, 0x30},
	{0xfe, 0x01},
	{0x40, 0x23},
	{0x55, 0x07},
	{0x60, 0x40},
	{0xfe, 0x04},
	{0x14, 0x78},
	{0x15, 0x78},
	{0x16, 0x78},
	{0x17, 0x78},
	{0xfe, 0x01},
	{0x92, 0x00},
	{0x94, 0x03},
	{0x95, 0x04},
	{0x96, 0x38},
	{0x97, 0x07},
	{0x98, 0x80},
	{0xfe, 0x01},
	{0x01, 0x05},
	{0x02, 0x89},
	{0x04, 0x01},
	{0x07, 0xa6},
	{0x08, 0xa9},
	{0x09, 0xa8},
	{0x0a, 0xa7},
	{0x0b, 0xff},
	{0x0c, 0xff},
	{0x0f, 0x00},
	{0x50, 0x1c},
	{0x89, 0x03},
	{0xfe, 0x04},
	{0x28, 0x86},
	{0x29, 0x86},
	{0x2a, 0x86},
	{0x2b, 0x68},
	{0x2c, 0x68},
	{0x2d, 0x68},
	{0x2e, 0x68},
	{0x2f, 0x68},
	{0x30, 0x4f},
	{0x31, 0x68},
	{0x32, 0x67},
	{0x33, 0x66},
	{0x34, 0x66},
	{0x35, 0x66},
	{0x36, 0x66},
	{0x37, 0x66},
	{0x38, 0x62},
	{0x39, 0x62},
	{0x3a, 0x62},
	{0x3b, 0x62},
	{0x3c, 0x62},
	{0x3d, 0x62},
	{0x3e, 0x62},
	{0x3f, 0x62},
	{0xfe, 0x01},
	{0x9a, 0x06},
	{0xfe, 0x00},
	{0x7b, 0x2a},
	{0x23, 0x2d},
	{0xfe, 0x03},
	{0x01, 0x27},
	{0x02, 0x56},
	{0x03, 0x8e},
	{0x12, 0x80},
	{0x13, 0x07},
	{0x15, 0x10},
	{0xfe, 0x00},
	{0x3e, 0x90},
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 1008Mbps
 */
static const struct regval gc2053_1920x1080_regs[] = {
	{REG_NULL, 0x00},
};

static const struct gc2053_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0410,
		.hts_def = 0x0970,
		.vts_def = 0x046d,
		.reg_list = gc2053_1920x1080_regs,
	},
};

static const s64 link_freq_menu_items[] = {
	GC2053_LINK_FREQ_420MHZ
};

/* sensor register write */
static int gc2053_write_reg(struct i2c_client *client, u8 reg, u8 val)
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
		"gc2053 write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

/* sensor register read */
static int gc2053_read_reg(struct i2c_client *client, u8 reg, u8 *val)
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
		"gc2053 read reg:0x%x failed !\n", reg);

	return ret;
}

static int gc2053_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = gc2053_write_reg(client, regs[i].addr, regs[i].val);

	return ret;
}

static int gc2053_get_reso_dist(const struct gc2053_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct gc2053_mode *
gc2053_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = gc2053_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int gc2053_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc2053 *gc2053 = to_gc2053(sd);
	const struct gc2053_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&gc2053->mutex);

	mode = gc2053_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&gc2053->mutex);
		return -ENOTTY;
#endif
	} else {
		gc2053->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(gc2053->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(gc2053->vblank, vblank_def,
					 GC2053_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&gc2053->mutex);

	return 0;
}

static int gc2053_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc2053 *gc2053 = to_gc2053(sd);
	const struct gc2053_mode *mode = gc2053->cur_mode;

	mutex_lock(&gc2053->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&gc2053->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&gc2053->mutex);

	return 0;
}

static int gc2053_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;

	return 0;
}

static int gc2053_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int gc2053_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct gc2053 *gc2053 = to_gc2053(sd);
	const struct gc2053_mode *mode = gc2053->cur_mode;

	mutex_lock(&gc2053->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&gc2053->mutex);

	return 0;
}

static void gc2053_get_module_inf(struct gc2053 *gc2053,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, GC2053_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, gc2053->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, gc2053->len_name, sizeof(inf->base.lens));
}

static long gc2053_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct gc2053 *gc2053 = to_gc2053(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		gc2053_get_module_inf(gc2053, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long gc2053_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	long ret;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = gc2053_ioctl(sd, cmd, inf);
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
			ret = gc2053_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __gc2053_start_stream(struct gc2053 *gc2053)
{
	int ret;

	ret = gc2053_write_array(gc2053->client, gc2053->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&gc2053->mutex);
	ret = v4l2_ctrl_handler_setup(&gc2053->ctrl_handler);
	mutex_lock(&gc2053->mutex);
	if (ret)
		return ret;

	ret = gc2053_write_reg(gc2053->client, GC2053_PAGE_SELECT, 0x03);
	ret |= gc2053_write_reg(gc2053->client, GC2053_MODE_SELECT,
				 GC2053_MODE_STREAMING);
	ret |= gc2053_write_reg(gc2053->client, GC2053_PAGE_SELECT, 0x00);
	return ret;
}

static int __gc2053_stop_stream(struct gc2053 *gc2053)
{
	int ret;

	ret = gc2053_write_reg(gc2053->client, GC2053_PAGE_SELECT, 0x03);
	ret |= gc2053_write_reg(gc2053->client, GC2053_MODE_SELECT,
				GC2053_MODE_SW_STANDBY);
	ret |= gc2053_write_reg(gc2053->client, GC2053_PAGE_SELECT, 0x00);
	return ret;
}

static int gc2053_s_stream(struct v4l2_subdev *sd, int on)
{
	struct gc2053 *gc2053 = to_gc2053(sd);
	struct i2c_client *client = gc2053->client;
	int ret = 0;

	mutex_lock(&gc2053->mutex);
	on = !!on;
	if (on == gc2053->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __gc2053_start_stream(gc2053);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__gc2053_stop_stream(gc2053);
		pm_runtime_put(&client->dev);
	}

	gc2053->streaming = on;

unlock_and_return:
	mutex_unlock(&gc2053->mutex);

	return ret;
}

static int gc2053_s_power(struct v4l2_subdev *sd, int on)
{
	struct gc2053 *gc2053 = to_gc2053(sd);
	struct i2c_client *client = gc2053->client;
	int ret = 0;

	mutex_lock(&gc2053->mutex);

	/* If the power state is not modified - no work to do. */
	if (gc2053->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = gc2053_write_array(gc2053->client, gc2053_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		gc2053->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		gc2053->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&gc2053->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 gc2053_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, GC2053_XVCLK_FREQ / 1000 / 1000);
}

static int __gc2053_power_on(struct gc2053 *gc2053)
{
	int ret;
	u32 delay_us;
	struct device *dev = &gc2053->client->dev;

	if (!IS_ERR_OR_NULL(gc2053->pins_default)) {
		ret = pinctrl_select_state(gc2053->pinctrl,
					   gc2053->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(gc2053->xvclk, GC2053_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(gc2053->xvclk) != GC2053_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(gc2053->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(gc2053->reset_gpio))
		gpiod_set_value_cansleep(gc2053->reset_gpio, 0);

	ret = regulator_bulk_enable(GC2053_NUM_SUPPLIES, gc2053->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(gc2053->reset_gpio))
		gpiod_set_value_cansleep(gc2053->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(gc2053->pwdn_gpio))
		gpiod_set_value_cansleep(gc2053->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = gc2053_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(gc2053->xvclk);

	return ret;
}

static void __gc2053_power_off(struct gc2053 *gc2053)
{
	int ret;
	struct device *dev = &gc2053->client->dev;

	if (!IS_ERR(gc2053->pwdn_gpio))
		gpiod_set_value_cansleep(gc2053->pwdn_gpio, 0);
	clk_disable_unprepare(gc2053->xvclk);
	if (!IS_ERR(gc2053->reset_gpio))
		gpiod_set_value_cansleep(gc2053->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(gc2053->pins_sleep)) {
		ret = pinctrl_select_state(gc2053->pinctrl,
					   gc2053->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(GC2053_NUM_SUPPLIES, gc2053->supplies);
}

static int gc2053_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2053 *gc2053 = to_gc2053(sd);

	return __gc2053_power_on(gc2053);
}

static int gc2053_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2053 *gc2053 = to_gc2053(sd);

	__gc2053_power_off(gc2053);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int gc2053_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gc2053 *gc2053 = to_gc2053(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct gc2053_mode *def_mode = &supported_modes[0];

	mutex_lock(&gc2053->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&gc2053->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int gc2053_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fie->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static u8 regValTable[29][4] = {
	/*0xb4 0xb3  0xb8  0xb9*/
	{0x00, 0x00, 0x01, 0x00},
	{0x00, 0x10, 0x01, 0x0c},
	{0x00, 0x20, 0x01, 0x1b},
	{0x00, 0x30, 0x01, 0x2c},
	{0x00, 0x40, 0x01, 0x3f},
	{0x00, 0x50, 0x02, 0x16},
	{0x00, 0x60, 0x02, 0x35},
	{0x00, 0x70, 0x03, 0x16},
	{0x00, 0x80, 0x04, 0x02},
	{0x00, 0x90, 0x04, 0x31},
	{0x00, 0xa0, 0x05, 0x32},
	{0x00, 0xb0, 0x06, 0x35},
	{0x00, 0xc0, 0x08, 0x04},
	{0x00, 0x5a, 0x09, 0x19},
	{0x00, 0x83, 0x0b, 0x0f},
	{0x00, 0x93, 0x0d, 0x12},
	{0x00, 0x84, 0x10, 0x00},
	{0x00, 0x94, 0x12, 0x3a},
	{0x01, 0x2c, 0x1a, 0x02},
	{0x01, 0x3c, 0x1b, 0x20},
	{0x00, 0x8c, 0x20, 0x0f},
	{0x00, 0x9c, 0x26, 0x07},
	{0x02, 0x64, 0x36, 0x21},
	{0x02, 0x74, 0x37, 0x3a},
	{0x00, 0xc6, 0x3d, 0x02},
	{0x00, 0xdc, 0x3f, 0x3f},
	{0x02, 0x85, 0x3f, 0x3f},
	{0x02, 0x95, 0x3f, 0x3f},
	{0x00, 0xce, 0x3f, 0x3f},
};

static u32 gainLevelTable[] = {
	64,
	74,
	89,
	102,
	127,
	147,
	177,
	203,
	260,
	300,
	361,
	415,
	504,
	581,
	722,
	832,
	1027,
	1182,
	1408,
	1621,
	1990,
	2291,
	2850,
	3282,
	4048,
	5180,
	5500,
	6744,
	7073,
};

static int gc2053_set_gain(struct i2c_client *client, u32 gain)
{
	u32 d_gain_code;
	int ret = 0;
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(gainLevelTable); i++) {
		if (gainLevelTable[i] <= gain && gain < gainLevelTable[i + 1])
			break;
	}
	d_gain_code = gain * 64 / gainLevelTable[i];
	ret = gc2053_write_reg(client, 0xfe, 0x00);
	ret |= gc2053_write_reg(client, 0xb4, regValTable[i][0]);
	ret |= gc2053_write_reg(client, 0xb3, regValTable[i][1]);
	ret |= gc2053_write_reg(client, 0xb8, regValTable[i][2]);
	ret |= gc2053_write_reg(client, 0xb9, regValTable[i][3]);
	ret |= gc2053_write_reg(client, 0xb1, d_gain_code >> 6);
	ret |= gc2053_write_reg(client, 0xb2, (d_gain_code << 2) & 0xFC);
	return ret;
}

static const struct dev_pm_ops gc2053_pm_ops = {
	SET_RUNTIME_PM_OPS(gc2053_runtime_suspend,
			   gc2053_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops gc2053_internal_ops = {
	.open = gc2053_open,
};
#endif

static const struct v4l2_subdev_core_ops gc2053_core_ops = {
	.s_power = gc2053_s_power,
	.ioctl = gc2053_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = gc2053_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops gc2053_video_ops = {
	.s_stream = gc2053_s_stream,
	.g_frame_interval = gc2053_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops gc2053_pad_ops = {
	.enum_mbus_code = gc2053_enum_mbus_code,
	.enum_frame_size = gc2053_enum_frame_sizes,
	.enum_frame_interval = gc2053_enum_frame_interval,
	.get_fmt = gc2053_get_fmt,
	.set_fmt = gc2053_set_fmt,
};

static const struct v4l2_subdev_ops gc2053_subdev_ops = {
	.core	= &gc2053_core_ops,
	.video	= &gc2053_video_ops,
	.pad	= &gc2053_pad_ops,
};

static int gc2053_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc2053 *gc2053 = container_of(ctrl->handler,
					     struct gc2053, ctrl_handler);
	struct i2c_client *client = gc2053->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = gc2053->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(gc2053->exposure,
					 gc2053->exposure->minimum, max,
					 gc2053->exposure->step,
					 gc2053->exposure->default_value);
		break;
	}

	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = gc2053_write_reg(client,
			GC2053_PAGE_SELECT,
			0x00);
		ret |= gc2053_write_reg(client,
			GC2053_REG_EXPOSURE_H,
			(ctrl->val >> 8) & 0x3f);
		ret |= gc2053_write_reg(client,
			GC2053_REG_EXPOSURE_L,
			ctrl->val & 0xff);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = gc2053_set_gain(client, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = gc2053_write_reg(client,
			 GC2053_PAGE_SELECT,
			 0x00);
		ret |= gc2053_write_reg(client,
			 GC2053_REG_VTS_H,
			 ((ctrl->val + gc2053->cur_mode->height) >> 8) & 0x3f);
		ret |= gc2053_write_reg(client,
			GC2053_REG_VTS_L,
			(ctrl->val + gc2053->cur_mode->height) & 0xff);
		break;

	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops gc2053_ctrl_ops = {
	.s_ctrl = gc2053_set_ctrl,
};

static int gc2053_initialize_controls(struct gc2053 *gc2053)
{
	const struct gc2053_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &gc2053->ctrl_handler;
	mode = gc2053->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &gc2053->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, GC2053_PIXEL_RATE, 1, GC2053_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	gc2053->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (gc2053->hblank)
		gc2053->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	gc2053->vblank = v4l2_ctrl_new_std(handler, &gc2053_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				GC2053_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	gc2053->exposure = v4l2_ctrl_new_std(handler, &gc2053_ctrl_ops,
				V4L2_CID_EXPOSURE, GC2053_EXPOSURE_MIN,
				exposure_max, GC2053_EXPOSURE_STEP,
				mode->exp_def);

	gc2053->anal_gain = v4l2_ctrl_new_std(handler, &gc2053_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, GC2053_GAIN_MIN,
				GC2053_GAIN_MAX, GC2053_GAIN_STEP,
				GC2053_GAIN_DEFAULT);

	if (handler->error) {
		ret = handler->error;
		dev_err(&gc2053->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	gc2053->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int gc2053_check_sensor_id(struct gc2053 *gc2053,
				  struct i2c_client *client)
{
	struct device *dev = &gc2053->client->dev;
	u8 pid = 0;
	u8 ver = 0;
	int ret;
	unsigned short id;

	ret = gc2053_read_reg(client, GC2053_REG_CHIP_ID_H, &pid);
	ret |= gc2053_read_reg(client, GC2053_REG_CHIP_ID_L, &ver);

	id = SENSOR_ID(pid, ver);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "detected gc%04x sensor\n", id);

	return ret;
}

static int gc2053_configure_regulators(struct gc2053 *gc2053)
{
	unsigned int i;

	for (i = 0; i < GC2053_NUM_SUPPLIES; i++)
		gc2053->supplies[i].supply = gc2053_supply_names[i];

	return devm_regulator_bulk_get(&gc2053->client->dev,
				       GC2053_NUM_SUPPLIES,
				       gc2053->supplies);
}

static int gc2053_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct gc2053 *gc2053;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "elvis driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	gc2053 = devm_kzalloc(dev, sizeof(*gc2053), GFP_KERNEL);
	if (!gc2053)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &gc2053->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &gc2053->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &gc2053->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &gc2053->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	gc2053->client = client;
	gc2053->cur_mode = &supported_modes[0];

	gc2053->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(gc2053->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}
	gc2053->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gc2053->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	gc2053->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(gc2053->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	gc2053->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(gc2053->pinctrl)) {
		gc2053->pins_default =
			pinctrl_lookup_state(gc2053->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(gc2053->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		gc2053->pins_sleep =
			pinctrl_lookup_state(gc2053->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(gc2053->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	ret = gc2053_configure_regulators(gc2053);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&gc2053->mutex);

	sd = &gc2053->subdev;
	v4l2_i2c_subdev_init(sd, client, &gc2053_subdev_ops);
	ret = gc2053_initialize_controls(gc2053);
	if (ret)
		goto err_destroy_mutex;

	ret = __gc2053_power_on(gc2053);
	if (ret)
		goto err_free_handler;

	ret = gc2053_check_sensor_id(gc2053, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &gc2053_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	gc2053->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &gc2053->pad, 0);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(gc2053->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 gc2053->module_index, facing,
		 GC2053_NAME, dev_name(sd->dev));
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
	__gc2053_power_off(gc2053);
err_free_handler:
	v4l2_ctrl_handler_free(&gc2053->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&gc2053->mutex);

	return ret;
}

static int gc2053_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2053 *gc2053 = to_gc2053(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&gc2053->ctrl_handler);
	mutex_destroy(&gc2053->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__gc2053_power_off(gc2053);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id gc2053_of_match[] = {
	{ .compatible = "galaxycore,gc2053" },
	{},
};
MODULE_DEVICE_TABLE(of, gc2053_of_match);
#endif

static const struct i2c_device_id gc2053_match_id[] = {
	{ "galaxycore,gc2053", 0 },
	{ },
};

static struct i2c_driver gc2053_i2c_driver = {
	.driver = {
		.name = GC2053_NAME,
		.pm = &gc2053_pm_ops,
		.of_match_table = of_match_ptr(gc2053_of_match),
	},
	.probe		= &gc2053_probe,
	.remove		= &gc2053_remove,
	.id_table	= gc2053_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&gc2053_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&gc2053_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("GC2053 CMOS Image Sensor driver");
MODULE_LICENSE("GPL v2");
