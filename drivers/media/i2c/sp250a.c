// SPDX-License-Identifier: GPL-2.0
/*
 * sp250a driver
 *
 * Copyright (C) 2021 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 init version
 * V0.0X01.0X01
 * 1. adjust power on/off sequence
 * 2. add some debug info
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

#define SP250A_LANES			1
#define SP250A_BITS_PER_SAMPLE		10
#define SP250A_LINK_FREQ_MHZ		420000000
/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define SP250A_PIXEL_RATE		(SP250A_LINK_FREQ_MHZ * 2 * 1 / 10)
#define SP250A_XVCLK_FREQ		24000000

#define CHIP_ID				0x250a
#define SP250A_REG_CHIP_ID_H		0x02
#define SP250A_REG_CHIP_ID_L		0x03

#define SP250A_REG_SET_PAGE		0xfd
#define SP250A_SET_PAGE_ZERO		0x00
#define SP250A_SET_PAGE_ONE		0x01

#define SP250A_REG_CTRL_MODE		0xac
#define SP250A_MODE_SW_STANDBY		0x00
#define SP250A_MODE_STREAMING		0x01

#define SP250A_REG_EXPOSURE_H		0x03
#define SP250A_REG_EXPOSURE_L		0x04
#define SP250A_FETCH_HIGH_BYTE_EXP(VAL) (((VAL) >> 8) & 0xFF)	/* 8 Bits */
#define SP250A_FETCH_LOW_BYTE_EXP(VAL) ((VAL) & 0xFF)	/* 8 Bits */
#define	SP250A_EXPOSURE_MIN		4
#define	SP250A_EXPOSURE_STEP		1
#define SP250A_VTS_MAX			0x1fff

#define SP250A_REG_AGAIN		0x24
#define SP250A_GAIN_MIN			0x10
#define SP250A_GAIN_MAX			0xff
#define SP250A_GAIN_STEP		1
#define SP250A_GAIN_DEFAULT		0x20

#define SP250A_REG_VTS_H		0x05
#define SP250A_REG_VTS_L		0x06

#define REG_NULL			0xFF

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define SP250A_NAME			"sp250a"
#define SP250A_MEDIA_BUS_FMT		MEDIA_BUS_FMT_SBGGR10_1X10

static const char * const sp250a_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SP250A_NUM_SUPPLIES ARRAY_SIZE(sp250a_supply_names)

struct regval {
	u8 addr;
	u8 val;
};

struct sp250a_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct sp250a {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[SP250A_NUM_SUPPLIES];
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
	const struct sp250a_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_sp250a(sd) container_of(sd, struct sp250a, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval sp250a_global_regs[] = {
	{0xfd, 0x01},
	//{0x03, 0x01}, //1base
	//{0x04, 0x73},
	{0x24, 0xff},
	{0x01, 0x01},
	{0x11, 0x30}, //rst_num1
	{0x33, 0x50}, //rst_num2
	{0x1c, 0x0c}, //[0]double shutter disable
	{0x1e, 0x80}, //rcn_dds_8lsb
	{0x29, 0x80}, //scnt_dds_8lsb
	{0x2a, 0xda}, //adc range 835mv,rgcol 1.5v,rgcnt 0.9v  //ea
	{0x2c, 0x60}, //high 8bit, pldo 2.57v
	{0x21, 0x26}, //pcp rst 3.3v, pcp tx 3.9v
	{0x25, 0x13}, //[4]bl_en, ipix 1.336uA
	{0x27, 0x01}, //two dds mode enable
	{0x55, 0x10},
	{0x66, 0x36},
	{0x68, 0x28},
	{0x72, 0x50},
	{0x58, 0x2a},
	{0x75, 0x60},
	{0x76, 0x05},
	{0x51, 0x30}, //pd reset restg
	{0x52, 0x2a}, //pd reset tg

	{0x3f, 0x00}, //mirror & flip
	{0x27, 0x00}, //wy171205
	{0x9d, 0x96},

	{0xd0, 0x03},
	{0xd1, 0x01},
	{0xd2, 0xd0},
	{0xd3, 0x02},
	{0xd4, 0x40},

	{0xfb, 0x7b},
	{0xf0, 0x00},
	{0xf1, 0x00},
	{0xf2, 0x00},
	{0xf3, 0x00},

	{0xa1, 0x04},
	//{0xac, 0x01},
	{0xb1, 0x01},

	{0xfd, 0x02},
	{0x1d, 0x01},
	{0x8a, 0x0f},

	{0xfd, 0x01},

	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 656Mbps
 */
static const struct regval sp250a_1600x1200_regs[] = {
	{REG_NULL, 0x00},
};

static const struct sp250a_mode supported_modes[] = {
	{
		.width = 1600,
		.height = 1200,
		.max_fps = {
			.numerator = 10000,
			.denominator = 302262,
		},
		.exp_def = 0x0480,
		.hts_def = 0x10DC,
		.vts_def = 0x04E6,
		.reg_list = sp250a_1600x1200_regs,
	},
};

static const s64 link_freq_menu_items[] = {
	SP250A_LINK_FREQ_MHZ
};

/* Write registers up to 4 at a time */
static int sp250a_write_reg(struct i2c_client *client, u8 reg, u8 val)
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
		"sp250a write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

static int sp250a_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i = 0;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = sp250a_write_reg(client, regs[i].addr, regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int sp250a_read_reg(struct i2c_client *client, u8 reg, u8 *val)
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
		"sp250a read reg:0x%x failed !\n", reg);

	return ret;
}

static int sp250a_get_reso_dist(const struct sp250a_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct sp250a_mode *
sp250a_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = sp250a_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int sp250a_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sp250a *sp250a = to_sp250a(sd);
	const struct sp250a_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&sp250a->mutex);

	mode = sp250a_find_best_fit(fmt);
	fmt->format.code = SP250A_MEDIA_BUS_FMT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sp250a->mutex);
		return -ENOTTY;
#endif
	} else {
		sp250a->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sp250a->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sp250a->vblank, vblank_def,
					 SP250A_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&sp250a->mutex);

	return 0;
}

static int sp250a_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct sp250a *sp250a = to_sp250a(sd);
	const struct sp250a_mode *mode = sp250a->cur_mode;

	mutex_lock(&sp250a->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sp250a->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = SP250A_MEDIA_BUS_FMT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&sp250a->mutex);

	return 0;
}

static int sp250a_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = SP250A_MEDIA_BUS_FMT;

	return 0;
}

static int sp250a_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != SP250A_MEDIA_BUS_FMT)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int sp250a_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct sp250a *sp250a = to_sp250a(sd);
	const struct sp250a_mode *mode = sp250a->cur_mode;

	mutex_lock(&sp250a->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&sp250a->mutex);

	return 0;
}

static void sp250a_get_module_inf(struct sp250a *sp250a,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, SP250A_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, sp250a->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, sp250a->len_name, sizeof(inf->base.lens));
}

static long sp250a_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sp250a *sp250a = to_sp250a(sd);
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		sp250a_get_module_inf(sp250a, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream) {
			ret = sp250a_write_reg(sp250a->client,
				 SP250A_REG_SET_PAGE,
				 SP250A_SET_PAGE_ONE);
			ret |= sp250a_write_reg(sp250a->client,
				 SP250A_REG_CTRL_MODE,
				 SP250A_MODE_STREAMING);
		} else {
			ret = sp250a_write_reg(sp250a->client,
				 SP250A_REG_SET_PAGE, SP250A_SET_PAGE_ONE);
			ret |= sp250a_write_reg(sp250a->client,
				 SP250A_REG_CTRL_MODE, SP250A_MODE_SW_STANDBY);
		}
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sp250a_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = sp250a_ioctl(sd, cmd, inf);
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
			ret = sp250a_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = sp250a_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __sp250a_start_stream(struct sp250a *sp250a)
{
	int ret;

	ret = sp250a_write_array(sp250a->client, sp250a->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&sp250a->mutex);
	ret = v4l2_ctrl_handler_setup(&sp250a->ctrl_handler);
	mutex_lock(&sp250a->mutex);
	if (ret)
		return ret;
	ret = sp250a_write_reg(sp250a->client,
				 SP250A_REG_SET_PAGE,
				 SP250A_SET_PAGE_ONE);
	ret |= sp250a_write_reg(sp250a->client,
				 SP250A_REG_CTRL_MODE,
				 SP250A_MODE_STREAMING);
	return ret;
}

static int __sp250a_stop_stream(struct sp250a *sp250a)
{
	int ret;

	ret = sp250a_write_reg(sp250a->client,
		SP250A_REG_SET_PAGE, SP250A_SET_PAGE_ONE);
	ret |= sp250a_write_reg(sp250a->client,
		SP250A_REG_CTRL_MODE, SP250A_MODE_SW_STANDBY);
	return ret;
}

static int sp250a_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sp250a *sp250a = to_sp250a(sd);
	struct i2c_client *client = sp250a->client;
	int ret = 0;

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
				sp250a->cur_mode->width,
				sp250a->cur_mode->height,
		DIV_ROUND_CLOSEST(sp250a->cur_mode->max_fps.denominator,
		sp250a->cur_mode->max_fps.numerator));

	mutex_lock(&sp250a->mutex);
	on = !!on;
	if (on == sp250a->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __sp250a_start_stream(sp250a);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__sp250a_stop_stream(sp250a);
		pm_runtime_put(&client->dev);
	}

	sp250a->streaming = on;

unlock_and_return:
	mutex_unlock(&sp250a->mutex);

	return ret;
}

static int sp250a_s_power(struct v4l2_subdev *sd, int on)
{
	struct sp250a *sp250a = to_sp250a(sd);
	struct i2c_client *client = sp250a->client;
	int ret = 0;

	mutex_lock(&sp250a->mutex);

	/* If the power state is not modified - no work to do. */
	if (sp250a->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = sp250a_write_array(sp250a->client, sp250a_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		sp250a->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		sp250a->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&sp250a->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 sp250a_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, SP250A_XVCLK_FREQ / 1000 / 1000);
}

static int __sp250a_power_on(struct sp250a *sp250a)
{
	int ret;
	u32 delay_us;
	struct device *dev = &sp250a->client->dev;

	if (!IS_ERR_OR_NULL(sp250a->pins_default)) {
		ret = pinctrl_select_state(sp250a->pinctrl,
					   sp250a->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(sp250a->xvclk, SP250A_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(sp250a->xvclk) != SP250A_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(sp250a->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(sp250a->reset_gpio))
		gpiod_set_value_cansleep(sp250a->reset_gpio, 0);

	ret = regulator_bulk_enable(SP250A_NUM_SUPPLIES, sp250a->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	usleep_range(500, 1000);
	if (!IS_ERR(sp250a->pwdn_gpio))
		gpiod_set_value_cansleep(sp250a->pwdn_gpio, 0);

	usleep_range(1000, 1100);
	if (!IS_ERR(sp250a->reset_gpio))
		gpiod_set_value_cansleep(sp250a->reset_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = sp250a_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(sp250a->xvclk);

	return ret;
}

static void __sp250a_power_off(struct sp250a *sp250a)
{
	int ret = 0;

	clk_disable_unprepare(sp250a->xvclk);
	if (!IS_ERR(sp250a->reset_gpio))
		gpiod_set_value_cansleep(sp250a->reset_gpio, 0);
	if (!IS_ERR(sp250a->pwdn_gpio))
		gpiod_set_value_cansleep(sp250a->pwdn_gpio, 1);
	if (!IS_ERR_OR_NULL(sp250a->pins_sleep)) {
		ret = pinctrl_select_state(sp250a->pinctrl,
					   sp250a->pins_sleep);
		if (ret < 0)
			dev_dbg(&sp250a->client->dev, "could not set pins\n");
	}
	regulator_bulk_disable(SP250A_NUM_SUPPLIES, sp250a->supplies);
}

static int sp250a_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sp250a *sp250a = to_sp250a(sd);

	return __sp250a_power_on(sp250a);
}

static int sp250a_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sp250a *sp250a = to_sp250a(sd);

	__sp250a_power_off(sp250a);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sp250a_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sp250a *sp250a = to_sp250a(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sp250a_mode *def_mode = &supported_modes[0];

	mutex_lock(&sp250a->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = SP250A_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&sp250a->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int sp250a_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fie->code != SP250A_MEDIA_BUS_FMT)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static int sp250a_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	u32 val = 0;

	val = 1 << (SP250A_LANES - 1) |
	      V4L2_MBUS_CSI2_CHANNEL_0 |
	      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	config->type = V4L2_MBUS_CSI2;
	config->flags = val;

	return 0;
}

static const struct dev_pm_ops sp250a_pm_ops = {
	SET_RUNTIME_PM_OPS(sp250a_runtime_suspend,
			   sp250a_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sp250a_internal_ops = {
	.open = sp250a_open,
};
#endif

static const struct v4l2_subdev_core_ops sp250a_core_ops = {
	.s_power = sp250a_s_power,
	.ioctl = sp250a_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sp250a_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sp250a_video_ops = {
	.s_stream = sp250a_s_stream,
	.g_frame_interval = sp250a_g_frame_interval,
	.g_mbus_config = sp250a_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops sp250a_pad_ops = {
	.enum_mbus_code = sp250a_enum_mbus_code,
	.enum_frame_size = sp250a_enum_frame_sizes,
	.enum_frame_interval = sp250a_enum_frame_interval,
	.get_fmt = sp250a_get_fmt,
	.set_fmt = sp250a_set_fmt,
};

static const struct v4l2_subdev_ops sp250a_subdev_ops = {
	.core	= &sp250a_core_ops,
	.video	= &sp250a_video_ops,
	.pad	= &sp250a_pad_ops,
};

static int sp250a_set_gain_reg(struct sp250a *sp250a, u32 a_gain)
{
	int ret = 0;

	ret = sp250a_write_reg(sp250a->client,
		SP250A_REG_SET_PAGE, SP250A_SET_PAGE_ONE);
	ret |= sp250a_write_reg(sp250a->client,
		SP250A_REG_AGAIN, a_gain & 0xff);
	ret |= sp250a_write_reg(sp250a->client,
				0x01,
				0x01);
	return ret;
}

static int sp250a_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sp250a *sp250a = container_of(ctrl->handler,
					     struct sp250a, ctrl_handler);
	struct i2c_client *client = sp250a->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sp250a->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(sp250a->exposure,
					 sp250a->exposure->minimum, max,
					 sp250a->exposure->step,
					 sp250a->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set exposure value 0x%x\n", ctrl->val);
		/* 4 least significant bits of expsoure are fractional part */
		ret = sp250a_write_reg(sp250a->client,
					SP250A_REG_SET_PAGE,
					SP250A_SET_PAGE_ONE);
		ret |= sp250a_write_reg(sp250a->client,
					SP250A_REG_EXPOSURE_H,
					SP250A_FETCH_HIGH_BYTE_EXP(ctrl->val));
		ret |= sp250a_write_reg(sp250a->client,
					SP250A_REG_EXPOSURE_L,
					SP250A_FETCH_LOW_BYTE_EXP(ctrl->val));
		ret |= sp250a_write_reg(sp250a->client,
					0x01,
					0x01);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set analog gain value 0x%x\n", ctrl->val);
		ret = sp250a_set_gain_reg(sp250a, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vb value 0x%x\n", ctrl->val);
		ret = sp250a_write_reg(sp250a->client,
					SP250A_REG_SET_PAGE,
					SP250A_SET_PAGE_ONE);
		ret |= sp250a_write_reg(sp250a->client,
					SP250A_REG_VTS_H,
					((ctrl->val - 32) >> 8) & 0xff);
		ret |= sp250a_write_reg(sp250a->client,
					SP250A_REG_VTS_L,
					(ctrl->val - 32) & 0xff);
		ret |= sp250a_write_reg(sp250a->client,
					0x01,
					0x01);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sp250a_ctrl_ops = {
	.s_ctrl = sp250a_set_ctrl,
};

static int sp250a_initialize_controls(struct sp250a *sp250a)
{
	const struct sp250a_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &sp250a->ctrl_handler;
	mode = sp250a->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &sp250a->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, SP250A_PIXEL_RATE, 1, SP250A_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	sp250a->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (sp250a->hblank)
		sp250a->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	sp250a->vblank = v4l2_ctrl_new_std(handler, &sp250a_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				SP250A_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	sp250a->exposure = v4l2_ctrl_new_std(handler, &sp250a_ctrl_ops,
				V4L2_CID_EXPOSURE, SP250A_EXPOSURE_MIN,
				exposure_max, SP250A_EXPOSURE_STEP,
				mode->exp_def);

	sp250a->anal_gain = v4l2_ctrl_new_std(handler, &sp250a_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, SP250A_GAIN_MIN,
				SP250A_GAIN_MAX, SP250A_GAIN_STEP,
				SP250A_GAIN_DEFAULT);
	if (handler->error) {
		ret = handler->error;
		dev_err(&sp250a->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	sp250a->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int sp250a_check_sensor_id(struct sp250a *sp250a,
				   struct i2c_client *client)
{
	struct device *dev = &sp250a->client->dev;
	u16 id = 0;
	u8 reg_H = 0;
	u8 reg_L = 0;
	int ret;

	ret = sp250a_write_reg(sp250a->client,
					SP250A_REG_SET_PAGE,
					SP250A_SET_PAGE_ZERO);
	ret |= sp250a_read_reg(client, SP250A_REG_CHIP_ID_H, &reg_H);
	ret |= sp250a_read_reg(client, SP250A_REG_CHIP_ID_L, &reg_L);
	id = ((reg_H << 8) & 0xff00) | (reg_L & 0xff);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected SP%06x sensor\n", CHIP_ID);
	return ret;
}

static int sp250a_configure_regulators(struct sp250a *sp250a)
{
	unsigned int i;

	for (i = 0; i < SP250A_NUM_SUPPLIES; i++)
		sp250a->supplies[i].supply = sp250a_supply_names[i];

	return devm_regulator_bulk_get(&sp250a->client->dev,
				       SP250A_NUM_SUPPLIES,
				       sp250a->supplies);
}

static int sp250a_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sp250a *sp250a;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	sp250a = devm_kzalloc(dev, sizeof(*sp250a), GFP_KERNEL);
	if (!sp250a)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sp250a->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sp250a->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sp250a->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sp250a->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}
	sp250a->client = client;
	sp250a->cur_mode = &supported_modes[0];

	sp250a->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sp250a->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	sp250a->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sp250a->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	sp250a->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(sp250a->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = sp250a_configure_regulators(sp250a);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	sp250a->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(sp250a->pinctrl)) {
		sp250a->pins_default =
			pinctrl_lookup_state(sp250a->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sp250a->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		sp250a->pins_sleep =
			pinctrl_lookup_state(sp250a->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(sp250a->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	mutex_init(&sp250a->mutex);

	sd = &sp250a->subdev;
	v4l2_i2c_subdev_init(sd, client, &sp250a_subdev_ops);
	ret = sp250a_initialize_controls(sp250a);
	if (ret)
		goto err_destroy_mutex;

	ret = __sp250a_power_on(sp250a);
	if (ret)
		goto err_free_handler;

	ret = sp250a_check_sensor_id(sp250a, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sp250a_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	sp250a->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sp250a->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(sp250a->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sp250a->module_index, facing,
		 SP250A_NAME, dev_name(sd->dev));
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
	__sp250a_power_off(sp250a);
err_free_handler:
	v4l2_ctrl_handler_free(&sp250a->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sp250a->mutex);

	return ret;
}

static int sp250a_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sp250a *sp250a = to_sp250a(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sp250a->ctrl_handler);
	mutex_destroy(&sp250a->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sp250a_power_off(sp250a);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sp250a_of_match[] = {
	{ .compatible = "superpix,sp250a" },
	{},
};
MODULE_DEVICE_TABLE(of, sp250a_of_match);
#endif

static const struct i2c_device_id sp250a_match_id[] = {
	{ "superpix,sp250a", 0 },
	{ },
};

static struct i2c_driver sp250a_i2c_driver = {
	.driver = {
		.name = SP250A_NAME,
		.pm = &sp250a_pm_ops,
		.of_match_table = of_match_ptr(sp250a_of_match),
	},
	.probe		= &sp250a_probe,
	.remove		= &sp250a_remove,
	.id_table	= sp250a_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sp250a_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sp250a_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("SuperPix sp250a sensor driver");
MODULE_LICENSE("GPL v2");
