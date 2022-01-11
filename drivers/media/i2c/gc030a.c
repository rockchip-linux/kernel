// SPDX-License-Identifier: GPL-2.0
/*
 * gc030a driver
 *
 * Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 init version.
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

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define GC030A_LANES			1
#define GC030A_BITS_PER_SAMPLE		10
#define GC030A_LINK_FREQ_MHZ		328000000
/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define GC030A_PIXEL_RATE		(GC030A_LINK_FREQ_MHZ * 2 * 1 / 10)
#define GC030A_XVCLK_FREQ		24000000
#define CHIP_ID				0x030A
#define GC030A_REG_CHIP_ID_H		0xf0
#define GC030A_REG_CHIP_ID_L		0xf1
#define GC030A_REG_SET_PAGE		0xfe
#define GC030A_SET_PAGE_ONE		0x00
#define GC030A_REG_CTRL_MODE		0x8b
#define GC030A_MODE_SW_STANDBY		0x20
#define GC030A_MODE_STREAMING		0x30
#define GC030A_REG_EXPOSURE_H		0x03
#define GC030A_REG_EXPOSURE_L		0x04
#define GC030A_FETCH_HIGH_BYTE_EXP(VAL) (((VAL) >> 8) & 0x0F)	/* 4 Bits */
#define GC030A_FETCH_LOW_BYTE_EXP(VAL) ((VAL) & 0xFF)	/* 8 Bits */
#define	GC030A_EXPOSURE_MIN		4
#define	GC030A_EXPOSURE_STEP		1
#define GC030A_VTS_MAX			0x3fff
#define GC030A_REG_AGAIN		0xb6
#define GC030A_REG_DGAIN_INT		0xb1
#define GC030A_REG_DGAIN_FRAC		0xb2
#define GC030A_GAIN_MIN			0x40
#define GC030A_GAIN_MAX			0x300
#define GC030A_GAIN_STEP		1
#define GC030A_GAIN_DEFAULT		0x40
#define GC030A_SENSOR_DGAIN_BASE	0x40
#define GC030A_REG_VTS_H			0x41
#define GC030A_REG_VTS_L			0x42
#define REG_NULL			0xFF
#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define GC030A_NAME			"gc030a"
#define GC030A_MEDIA_BUS_FMT		MEDIA_BUS_FMT_SBGGR10_1X10
#define IMAGE_HV_MIRROR

#ifdef IMAGE_NORMAL
#define MIRROR 0x54
#define STARTY 0x01
#define STARTX 0x01
#endif

#ifdef IMAGE_H_MIRROR
#define MIRROR 0x55
#define STARTY 0x01
#define STARTX 0x02
#endif

#ifdef IMAGE_V_MIRROR
#define MIRROR 0x56
#define STARTY 0x02
#define STARTX 0x01
#endif

#ifdef IMAGE_HV_MIRROR
#define MIRROR 0x57
#define STARTY 0x02
#define STARTX 0x02
#endif

static const char * const gc030a_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};
#define GC030A_NUM_SUPPLIES ARRAY_SIZE(gc030a_supply_names)

struct regval {
	u8 addr;
	u8 val;
};

struct gc030a_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct gc030a {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[GC030A_NUM_SUPPLIES];
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
	const struct gc030a_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};
#define to_gc030a(sd) container_of(sd, struct gc030a, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval gc030a_global_regs[] = {
	/*SYS*/
	{0xfe, 0x80},
	{0xfe, 0x80},
	{0xfe, 0x80},
	{0xf7, 0x01},
	{0xf8, 0x05},
	{0xf9, 0x0f},
	{0xfa, 0x00},
	{0xfc, 0x0f},
	{0xfe, 0x00},
	/*ANALOG & CISCTL*/
	{0x03, 0x01},
	{0x04, 0xc8},
	{0x05, 0x03},
	{0x06, 0x7b},
	{0x07, 0x00},
	{0x08, 0x06},
	{0x0a, 0x00},
	{0x0c, 0x08},
	{0x0d, 0x01},
	{0x0e, 0xe8},
	{0x0f, 0x02},
	{0x10, 0x88},
	{0x12, 0x28}, // 23 add 20170110
	{0x17, MIRROR}, // Don't Change Here!!!
	{0x18, 0x12},
	{0x19, 0x07},
	{0x1a, 0x1b},
	{0x1d, 0x48}, // 40 travis20160318
	{0x1e, 0x50},
	{0x1f, 0x80},
	{0x23, 0x01},
	{0x24, 0xc8},
	{0x27, 0xaf},
	{0x28, 0x24},
	{0x29, 0x1a},
	{0x2f, 0x14},
	{0x30, 0x00},
	{0x31, 0x04},
	{0x32, 0x08},
	{0x33, 0x0c},
	{0x34, 0x0d},
	{0x35, 0x0e},
	{0x36, 0x0f},
	{0x72, 0x98},
	{0x73, 0x9a},
	{0x74, 0x47},
	{0x76, 0x82},
	{0x7a, 0xcb},
	{0xc2, 0x0c},
	{0xce, 0x03},
	{0xcf, 0x48},
	{0xd0, 0x10},
	{0xdc, 0x75},
	{0xeb, 0x78},
	/*ISP*/
	{0x90, 0x01},
	{0x92, STARTY}, // Don't Change Here!!!
	{0x94, STARTX}, // Don't Change Here!!!
	{0x95, 0x01},
	{0x96, 0xe0},
	{0x97, 0x02},
	{0x98, 0x80},
	/*Gain*/
	{0xb0, 0x46},
	{0xb1, 0x01},
	{0xb2, 0x03},
	{0xb3, 0x40},
	{0xb4, 0x40},
	{0xb5, 0x40},
	{0xb6, 0x03},
	/*BLK*/
	{0x40, 0x26},
	{0x4e, 0x00},
	{0x4f, 0x3c},
	/*Dark Sun*/
	{0xe0, 0x9f},
	{0xe1, 0x90},
	{0xe4, 0x0f},
	{0xe5, 0xff},
	/*MIPI*/
	{0xfe, 0x03},
	{0x10, 0x00},
	{0x01, 0x03},
	{0x02, 0x33},
	{0x03, 0x96},
	{0x04, 0x01},
	{0x05, 0x00},
	{0x06, 0x80},
	{0x11, 0x2b},
	{0x12, 0x20},
	{0x13, 0x03},
	{0x15, 0x00},
	{0x21, 0x10},
	{0x22, 0x00},
	{0x23, 0x30},
	{0x24, 0x02},
	{0x25, 0x12},
	{0x26, 0x02},
	{0x29, 0x01},
	{0x2a, 0x0a},
	{0x2b, 0x03},
	{0xfe, 0x00},
	{0xf9, 0x0e},
	{0xfc, 0x0e},
	{0xfe, 0x00},
	{0x25, 0xa2},
	{0x3f, 0x1a},
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 656Mbps
 */
static const struct regval gc030a_640x480_regs[] = {
	{REG_NULL, 0x00},
};

static const struct gc030a_mode supported_modes[] = {
	{
		.width		= 640,
		.height		= 480,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x01f0,//1152 //0x012c
		.hts_def = 0x062B,//2192
		.vts_def = 0x01fa,//1914
		.reg_list = gc030a_640x480_regs,
	},
};

static const s64 link_freq_menu_items[] = {
	GC030A_LINK_FREQ_MHZ
};

/* Write registers up to 4 at a time */
static int gc030a_write_reg(struct i2c_client *client, u8 reg, u8 val)
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
		"gc030a write reg(0x%x val:0x%x) failed !\n", reg, val);
	return ret;
}

static int gc030a_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i = 0;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = gc030a_write_reg(client, regs[i].addr, regs[i].val);
	return ret;
}

/* Read registers up to 4 at a time */
static int gc030a_read_reg(struct i2c_client *client, u8 reg, u8 *val)
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
		"gc030a read reg:0x%x failed !\n", reg);
	return ret;
}

static int gc030a_get_reso_dist(const struct gc030a_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct gc030a_mode *
gc030a_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = gc030a_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}
	return &supported_modes[cur_best_fit];
}

static int gc030a_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc030a *gc030a = to_gc030a(sd);
	const struct gc030a_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&gc030a->mutex);
	mode = gc030a_find_best_fit(fmt);
	fmt->format.code = GC030A_MEDIA_BUS_FMT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&gc030a->mutex);
		return -ENOTTY;
#endif
	} else {
		gc030a->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(gc030a->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(gc030a->vblank, vblank_def,
					 GC030A_VTS_MAX - mode->height,
					 1, vblank_def);
	}
	mutex_unlock(&gc030a->mutex);
	return 0;
}

static int gc030a_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct gc030a *gc030a = to_gc030a(sd);
	const struct gc030a_mode *mode = gc030a->cur_mode;

	mutex_lock(&gc030a->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&gc030a->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = GC030A_MEDIA_BUS_FMT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&gc030a->mutex);
	return 0;
}

static int gc030a_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = GC030A_MEDIA_BUS_FMT;
	return 0;
}

static int gc030a_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;
	if (fse->code != GC030A_MEDIA_BUS_FMT)
		return -EINVAL;
	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;
	return 0;
}

static int gc030a_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct gc030a *gc030a = to_gc030a(sd);
	const struct gc030a_mode *mode = gc030a->cur_mode;

	mutex_lock(&gc030a->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&gc030a->mutex);
	return 0;
}

static void gc030a_get_module_inf(struct gc030a *gc030a,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, GC030A_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, gc030a->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, gc030a->len_name, sizeof(inf->base.lens));
}

static long gc030a_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct gc030a *gc030a = to_gc030a(sd);
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		gc030a_get_module_inf(gc030a, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);
		if (stream) {
			ret = gc030a_write_reg(gc030a->client,
				 GC030A_REG_SET_PAGE,
				 GC030A_SET_PAGE_ONE);
			ret |= gc030a_write_reg(gc030a->client,
				 GC030A_REG_CTRL_MODE,
				 GC030A_MODE_STREAMING);
		} else {
			ret = gc030a_write_reg(gc030a->client,
				 GC030A_REG_SET_PAGE, GC030A_SET_PAGE_ONE);
			ret |= gc030a_write_reg(gc030a->client,
				 GC030A_REG_CTRL_MODE, GC030A_MODE_SW_STANDBY);
		}
		break;
	default:
		ret = -ENOTTY;
		break;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long gc030a_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}
		ret = gc030a_ioctl(sd, cmd, inf);
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
		if (copy_from_user(cfg, up, sizeof(*cfg))) {
			kfree(cfg);
			return -ENOMEM;
		}
		ret = gc030a_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;
		ret = gc030a_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}
#endif

static int __gc030a_start_stream(struct gc030a *gc030a)
{
	int ret;

	ret = gc030a_write_array(gc030a->client, gc030a->cur_mode->reg_list);
	msleep(100);
	if (ret)
		return ret;
	gc030a_write_reg(gc030a->client, 0xfe, 0x00);
	gc030a_write_reg(gc030a->client, 0x25, 0xe2);
	gc030a_write_reg(gc030a->client, 0xfe, 0x03);
	gc030a_write_reg(gc030a->client, 0x10, 0x90);
	gc030a_write_reg(gc030a->client, 0xfe, 0x00);
	msleep(20);

	return ret;
}

static int __gc030a_stop_stream(struct gc030a *gc030a)
{
	int ret = 0;

	msleep(20);
	gc030a_write_reg(gc030a->client, 0xfe, 0x03);
	gc030a_write_reg(gc030a->client, 0x10, 0x00);
	gc030a_write_reg(gc030a->client, 0xfe, 0x00);
	msleep(100);

	return ret;
}

static int gc030a_s_stream(struct v4l2_subdev *sd, int on)
{
	struct gc030a *gc030a = to_gc030a(sd);
	struct i2c_client *client = gc030a->client;
	int ret = 0;

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
				gc030a->cur_mode->width,
				gc030a->cur_mode->height,
				gc030a->cur_mode->max_fps.denominator);
	mutex_lock(&gc030a->mutex);
	on = !!on;
	if (on == gc030a->streaming)
		goto unlock_and_return;
	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		ret = __gc030a_start_stream(gc030a);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__gc030a_stop_stream(gc030a);
		pm_runtime_put(&client->dev);
	}
	gc030a->streaming = on;
unlock_and_return:
	mutex_unlock(&gc030a->mutex);
	return ret;
}

static int gc030a_s_power(struct v4l2_subdev *sd, int on)
{
	struct gc030a *gc030a = to_gc030a(sd);
	struct i2c_client *client = gc030a->client;
	int ret = 0;

	mutex_lock(&gc030a->mutex);
	/* If the power state is not modified - no work to do. */
	if (gc030a->power_on == !!on)
		goto unlock_and_return;
	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		ret = gc030a_write_array(gc030a->client, gc030a_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		gc030a->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		gc030a->power_on = false;
	}
unlock_and_return:
	mutex_unlock(&gc030a->mutex);
	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 gc030a_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, GC030A_XVCLK_FREQ / 1000 / 1000);
}

static int __gc030a_power_on(struct gc030a *gc030a)
{
	int ret;
	u32 delay_us;
	struct device *dev = &gc030a->client->dev;

	if (!IS_ERR_OR_NULL(gc030a->pins_default)) {
		ret = pinctrl_select_state(gc030a->pinctrl,
					   gc030a->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(gc030a->xvclk, GC030A_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(gc030a->xvclk) != GC030A_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(gc030a->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(gc030a->reset_gpio))
		gpiod_set_value_cansleep(gc030a->reset_gpio, 1);
	ret = regulator_bulk_enable(GC030A_NUM_SUPPLIES, gc030a->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}
	usleep_range(1000, 1100);
	if (!IS_ERR(gc030a->reset_gpio))
		gpiod_set_value_cansleep(gc030a->reset_gpio, 0);
	usleep_range(500, 1000);
	if (!IS_ERR(gc030a->pwdn_gpio))
		gpiod_set_value_cansleep(gc030a->pwdn_gpio, 1);
	/* 8192 cycles prior to first SCCB transaction */
	delay_us = gc030a_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	return 0;
disable_clk:
	clk_disable_unprepare(gc030a->xvclk);
	return ret;
}

static void __gc030a_power_off(struct gc030a *gc030a)
{
	int ret = 0;

	if (!IS_ERR(gc030a->pwdn_gpio))
		gpiod_set_value_cansleep(gc030a->pwdn_gpio, 0);
	clk_disable_unprepare(gc030a->xvclk);
	if (!IS_ERR(gc030a->reset_gpio))
		gpiod_set_value_cansleep(gc030a->reset_gpio, 1);
	if (!IS_ERR_OR_NULL(gc030a->pins_sleep)) {
		ret = pinctrl_select_state(gc030a->pinctrl,
					   gc030a->pins_sleep);
		if (ret < 0)
			dev_dbg(&gc030a->client->dev, "could not set pins\n");
	}
	regulator_bulk_disable(GC030A_NUM_SUPPLIES, gc030a->supplies);
}

static int gc030a_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc030a *gc030a = to_gc030a(sd);

	return __gc030a_power_on(gc030a);
}

static int gc030a_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc030a *gc030a = to_gc030a(sd);

	__gc030a_power_off(gc030a);
	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int gc030a_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gc030a *gc030a = to_gc030a(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct gc030a_mode *def_mode = &supported_modes[0];

	mutex_lock(&gc030a->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = GC030A_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;
	mutex_unlock(&gc030a->mutex);
	/* No crop or compose */
	return 0;
}
#endif

static int gc030a_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;
	if (fie->code != GC030A_MEDIA_BUS_FMT)
		return -EINVAL;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static const struct dev_pm_ops gc030a_pm_ops = {
	SET_RUNTIME_PM_OPS(gc030a_runtime_suspend,
			   gc030a_runtime_resume, NULL)
};
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API

static const struct v4l2_subdev_internal_ops gc030a_internal_ops = {
	.open = gc030a_open,
};
#endif

static const struct v4l2_subdev_core_ops gc030a_core_ops = {
	.s_power = gc030a_s_power,
	.ioctl = gc030a_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = gc030a_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops gc030a_video_ops = {
	.s_stream = gc030a_s_stream,
	.g_frame_interval = gc030a_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops gc030a_pad_ops = {
	.enum_mbus_code = gc030a_enum_mbus_code,
	.enum_frame_size = gc030a_enum_frame_sizes,
	.enum_frame_interval = gc030a_enum_frame_interval,
	.get_fmt = gc030a_get_fmt,
	.set_fmt = gc030a_set_fmt,
};

static const struct v4l2_subdev_ops gc030a_subdev_ops = {
	.core	= &gc030a_core_ops,
	.video	= &gc030a_video_ops,
	.pad	= &gc030a_pad_ops,
};

static u32 GC030A_AGC_Param[5][2] = {
	{64,  0},
	{91,  1},
	{126,  2},
	{178,  3},
	{242,  4},
};

#define ANALOG_GAIN_1 64 // 1.00x
#define ANALOG_GAIN_2 91 // 1.421875
#define ANALOG_GAIN_3 126 // 1.96875
#define ANALOG_GAIN_4 178 // 2.78125
#define ANALOG_GAIN_5 242 // 3.78125

static int gc030a_set_gain_reg(struct gc030a *gc030a, u32 a_gain)
{
	struct device *dev = &gc030a->client->dev;
	int ret = 0, gain_index = 0;
	u32 temp_gain = 0;

	dev_info(dev, "%s(%d) a_gain(0x%08x)!\n", __func__, __LINE__, a_gain);
	if (a_gain < 0x40)
		a_gain = 0x40;
	for (gain_index = 4; gain_index >= 0; gain_index--) {
		if (a_gain >= GC030A_AGC_Param[gain_index][0])
			break;
	}
	ret |= gc030a_write_reg(gc030a->client,
		GC030A_REG_AGAIN, GC030A_AGC_Param[gain_index][1]);
	temp_gain = a_gain * GC030A_SENSOR_DGAIN_BASE /
			    GC030A_AGC_Param[gain_index][0];
	dev_info(dev, "AGC_Param[%d][0](%d) temp_gain is(0x%08x)!\n",
				gain_index, GC030A_AGC_Param[gain_index][0], temp_gain);
	ret |= gc030a_write_reg(gc030a->client,
		GC030A_REG_DGAIN_INT,
		(temp_gain >> 6));
	ret |= gc030a_write_reg(gc030a->client,
		GC030A_REG_DGAIN_FRAC,
		(temp_gain << 2) & 0xfc);
	return ret;
}

static int gc030a_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc030a *gc030a = container_of(ctrl->handler,
					     struct gc030a, ctrl_handler);
	struct i2c_client *client = gc030a->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = gc030a->cur_mode->height + ctrl->val - 16;
		__v4l2_ctrl_modify_range(gc030a->exposure,
					 gc030a->exposure->minimum, max,
					 gc030a->exposure->step,
					 gc030a->exposure->default_value);
		break;
	}
	if (pm_runtime_get(&client->dev) <= 0)
		return 0;
	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = gc030a_write_reg(gc030a->client,
					GC030A_REG_SET_PAGE,
					GC030A_SET_PAGE_ONE);
		ret |= gc030a_write_reg(gc030a->client,
					GC030A_REG_EXPOSURE_H,
					GC030A_FETCH_HIGH_BYTE_EXP(ctrl->val));
		ret |= gc030a_write_reg(gc030a->client,
					GC030A_REG_EXPOSURE_L,
					GC030A_FETCH_LOW_BYTE_EXP(ctrl->val));
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = gc030a_set_gain_reg(gc030a, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = gc030a_write_reg(gc030a->client,
					GC030A_REG_SET_PAGE,
					GC030A_SET_PAGE_ONE);
		ret |= gc030a_write_reg(gc030a->client,
					GC030A_REG_VTS_H,
					((gc030a->cur_mode->height + ctrl->val) >> 8) & 0x3f);
		ret |= gc030a_write_reg(gc030a->client,
					GC030A_REG_VTS_L,
					(gc030a->cur_mode->height + ctrl->val) & 0xff);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}
	pm_runtime_put(&client->dev);
	return ret;
}

static const struct v4l2_ctrl_ops gc030a_ctrl_ops = {
	.s_ctrl = gc030a_set_ctrl,
};

static int gc030a_initialize_controls(struct gc030a *gc030a)
{
	const struct gc030a_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &gc030a->ctrl_handler;
	mode = gc030a->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &gc030a->mutex;
	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, GC030A_PIXEL_RATE, 1, GC030A_PIXEL_RATE);
	h_blank = mode->hts_def - mode->width;
	gc030a->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (gc030a->hblank)
		gc030a->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	gc030a->vblank = v4l2_ctrl_new_std(handler, &gc030a_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				GC030A_VTS_MAX - mode->height,
				1, vblank_def);
	exposure_max = mode->vts_def - 4;
	gc030a->exposure = v4l2_ctrl_new_std(handler, &gc030a_ctrl_ops,
				V4L2_CID_EXPOSURE, GC030A_EXPOSURE_MIN,
				exposure_max, GC030A_EXPOSURE_STEP,
				mode->exp_def);
	gc030a->anal_gain = v4l2_ctrl_new_std(handler, &gc030a_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, GC030A_GAIN_MIN,
				GC030A_GAIN_MAX, GC030A_GAIN_STEP,
				GC030A_GAIN_DEFAULT);
	if (handler->error) {
		ret = handler->error;
		dev_err(&gc030a->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}
	gc030a->subdev.ctrl_handler = handler;
	return 0;
err_free_handler:
	v4l2_ctrl_handler_free(handler);
	return ret;
}

static int gc030a_check_sensor_id(struct gc030a *gc030a,
				   struct i2c_client *client)
{
	struct device *dev = &gc030a->client->dev;
	u16 id = 0;
	u8 reg_H = 0;
	u8 reg_L = 0;
	int ret;

	ret = gc030a_write_reg(gc030a->client,
					GC030A_REG_SET_PAGE,
					GC030A_SET_PAGE_ONE);
	ret |= gc030a_read_reg(client, GC030A_REG_CHIP_ID_H, &reg_H);
	ret |= gc030a_read_reg(client, GC030A_REG_CHIP_ID_L, &reg_L);
	id = ((reg_H << 8) & 0xff00) | (reg_L & 0xff);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}
	dev_info(dev, "detected gc%04x sensor\n", id);
	return ret;
}

static int gc030a_configure_regulators(struct gc030a *gc030a)
{
	unsigned int i;

	for (i = 0; i < GC030A_NUM_SUPPLIES; i++)
		gc030a->supplies[i].supply = gc030a_supply_names[i];
	return devm_regulator_bulk_get(&gc030a->client->dev,
				       GC030A_NUM_SUPPLIES,
				       gc030a->supplies);
}

static int gc030a_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct gc030a *gc030a;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);
	gc030a = devm_kzalloc(dev, sizeof(*gc030a), GFP_KERNEL);
	if (!gc030a)
		return -ENOMEM;
	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &gc030a->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &gc030a->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &gc030a->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &gc030a->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}
	gc030a->client = client;
	gc030a->cur_mode = &supported_modes[0];
	gc030a->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(gc030a->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}
	gc030a->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gc030a->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");
	gc030a->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(gc030a->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");
	ret = gc030a_configure_regulators(gc030a);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}
	gc030a->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(gc030a->pinctrl)) {
		gc030a->pins_default =
			pinctrl_lookup_state(gc030a->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(gc030a->pins_default))
			dev_err(dev, "could not get default pinstate\n");
		gc030a->pins_sleep =
			pinctrl_lookup_state(gc030a->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(gc030a->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}
	mutex_init(&gc030a->mutex);
	sd = &gc030a->subdev;
	v4l2_i2c_subdev_init(sd, client, &gc030a_subdev_ops);
	ret = gc030a_initialize_controls(gc030a);
	if (ret)
		goto err_destroy_mutex;
	ret = __gc030a_power_on(gc030a);
	if (ret)
		goto err_free_handler;
	ret = gc030a_check_sensor_id(gc030a, client);
	if (ret)
		goto err_power_off;
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &gc030a_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	gc030a->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &gc030a->pad);
	if (ret < 0)
		goto err_power_off;
#endif
	memset(facing, 0, sizeof(facing));
	if (strcmp(gc030a->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';
	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 gc030a->module_index, facing,
		 GC030A_NAME, dev_name(sd->dev));
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
	__gc030a_power_off(gc030a);
err_free_handler:
	v4l2_ctrl_handler_free(&gc030a->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&gc030a->mutex);
	return ret;
}

static int gc030a_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc030a *gc030a = to_gc030a(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&gc030a->ctrl_handler);
	mutex_destroy(&gc030a->mutex);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__gc030a_power_off(gc030a);
	pm_runtime_set_suspended(&client->dev);
	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id gc030a_of_match[] = {
	{ .compatible = "galaxycore,gc030a" },
	{},
};
MODULE_DEVICE_TABLE(of, gc030a_of_match);
#endif

static const struct i2c_device_id gc030a_match_id[] = {
	{ "galaxycore,gc030a", 0 },
	{ },
};

static struct i2c_driver gc030a_i2c_driver = {
	.driver = {
		.name = GC030A_NAME,
		.pm = &gc030a_pm_ops,
		.of_match_table = of_match_ptr(gc030a_of_match),
	},
	.probe		= &gc030a_probe,
	.remove		= &gc030a_remove,
	.id_table	= gc030a_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&gc030a_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&gc030a_i2c_driver);
}

device_initcall_sync(sensor_mod_init);

module_exit(sensor_mod_exit);
MODULE_DESCRIPTION("GalaxyCore gc030a sensor driver");
MODULE_LICENSE("GPL v2");
