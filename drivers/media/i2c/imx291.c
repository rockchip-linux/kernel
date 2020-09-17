// SPDX-License-Identifier: GPL-2.0
/*
 * imx291 driver
 *
 * Copyright (C) 2019 Fuzhou Rockchip Electronics Co., Ltd.
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

#define IMX291_DATA_FORMAT MEDIA_BUS_FMT_SRGGB12_1X12

#define IMX291_LINK_FREQ		222750000
#define IMX291_LANES			2
#define IMX291_BITS_PER_SAMPLE		12

/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define IMX291_PIXEL_RATE		(IMX291_LINK_FREQ * 2 * IMX291_LANES / IMX291_BITS_PER_SAMPLE)

#define IMX291_XVCLK_FREQ		37125000

#define CHIP_ID				0xb2
#define IMX291_REG_CHIP_ID		0x301e

#define IMX291_REG_CTRL_MODE		0x3000
#define IMX291_MODE_SW_STANDBY		0x1
#define IMX291_MODE_STREAMING		0x0

#define IMX291_REG_EXPOSURE_H		0x3022
#define IMX291_REG_EXPOSURE_M		0x3021
#define IMX291_REG_EXPOSURE_L		0x3020

#define IMX291_FETCH_HIGH_BYTE_EXP(VAL)	(((VAL) >> 16) & 0x03)
#define IMX291_FETCH_MID_BYTE_EXP(VAL)	(((VAL) >> 8) & 0xFF)
#define IMX291_FETCH_LOW_BYTE_EXP(VAL)	((VAL) & 0xFF)

#define	IMX291_EXPOSURE_MIN		2
#define	IMX291_EXPOSURE_STEP		1
#define IMX291_VTS_MAX			0x7fff

#define IMX291_REG_GAIN			0x3014
#define IMX291_GAIN_MIN			0x00
#define IMX291_GAIN_MAX			0xee
#define IMX291_GAIN_STEP		1
#define IMX291_GAIN_DEFAULT		0x00

#ifdef USED_TEST_PATTERN
#define IMX291_REG_TEST_PATTERN		0x5e00
#define	IMX291_TEST_PATTERN_ENABLE	0x80
#define	IMX291_TEST_PATTERN_DISABLE	0x0
#endif

#define IMX291_REG_VTS_H		0x301a
#define IMX291_REG_VTS_M		0x3019
#define IMX291_REG_VTS_L		0x3018
#define IMX291_FETCH_HIGH_BYTE_VTS(VAL)	(((VAL) >> 16) & 0x03)
#define IMX291_FETCH_MID_BYTE_VTS(VAL)	(((VAL) >> 8) & 0xFF)
#define IMX291_FETCH_LOW_BYTE_VTS(VAL)	((VAL) & 0xFF)

#define REG_NULL			0xFFFF
#define REG_DELAY			0xFFFE

#define IMX291_REG_VALUE_08BIT		1
#define IMX291_REG_VALUE_16BIT		2
#define IMX291_REG_VALUE_24BIT		3

#define IMX291_NAME			"imx291"

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

static const char * const imx291_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define IMX291_NUM_SUPPLIES ARRAY_SIZE(imx291_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct imx291_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct imx291 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[IMX291_NUM_SUPPLIES];

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
#ifdef USED_TEST_PATTERN
	struct v4l2_ctrl	*test_pattern;
#endif
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct imx291_mode *support_modes;
	u32			support_modes_num;
	const struct imx291_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_imx291(sd) container_of(sd, struct imx291, subdev)

/*
 * Xclk 37.125Mhz
 */
static const struct regval imx291_global_regs[] = {
	{0x3003, 0x01},
	{REG_DELAY, 10},
	{0x3000, 0x01},
	{0x3002, 0x00},
	{0x3005, 0x01},
	{0x3007, 0x00},
	{0x3009, 0x02},
	{0x300a, 0xf0},
	{0x300f, 0x00},
	{0x3010, 0x21},
	{0x3012, 0x64},
	{0x3016, 0x09},
	{0x3019, 0x04},
	{0x3018, 0x65},
	{0x301d, 0x11},
	{0x301c, 0x30},
	{0x3046, 0x01},
	{0x304b, 0x0a},
	{0x305c, 0x18},
	{0x305d, 0x03},
	{0x305e, 0x20},
	{0x305f, 0x01},
	{0x3070, 0x02},
	{0x3071, 0x11},
	{0x309b, 0x10},
	{0x309c, 0x22},
	{0x30a2, 0x02},
	{0x30a6, 0x20},
	{0x30a8, 0x20},
	{0x30aa, 0x20},
	{0x30ac, 0x20},
	{0x30b0, 0x43},
	{0x3119, 0x9e},
	{0x311c, 0x1e},
	{0x311e, 0x08},
	{0x3128, 0x05},
	{0x3129, 0x00},
	{0x313d, 0x83},
	{0x3150, 0x03},
	{0x315e, 0x1a},
	{0x3164, 0x1a},
	{0x317c, 0x00},
	{0x317e, 0x00},
	{0x31ec, 0x0e},
	{0x32b8, 0x50},
	{0x32b9, 0x10},
	{0x32ba, 0x00},
	{0x32bb, 0x04},
	{0x32c8, 0x50},
	{0x32c9, 0x10},
	{0x32ca, 0x00},
	{0x32cb, 0x04},
	{0x332c, 0xd3},
	{0x332d, 0x10},
	{0x332e, 0x0d},
	{0x3358, 0x06},
	{0x3359, 0xe1},
	{0x335a, 0x11},
	{0x3360, 0x1e},
	{0x3361, 0x61},
	{0x3362, 0x10},
	{0x33b0, 0x50},
	{0x33b2, 0x1a},
	{0x33b3, 0x04},
	{0x3405, 0x10},
	{0x3407, 0x01},
	{0x3414, 0x0a},
	{0x3418, 0x49},
	{0x3419, 0x04},
	{0x3441, 0x0c},
	{0x3442, 0x0c},
	{0x3443, 0x01},
	{0x3444, 0x20},
	{0x3445, 0x25},
	{0x3446, 0x57},
	{0x3447, 0x00},
	{0x3448, 0x55},
	{0x3449, 0x00},
	{0x344a, 0x1f},
	{0x344b, 0x00},
	{0x344c, 0x1f},
	{0x344d, 0x00},
	{0x344e, 0x1f},
	{0x344f, 0x00},
	{0x3450, 0x77},
	{0x3451, 0x00},
	{0x3452, 0x1f},
	{0x3453, 0x00},
	{0x3454, 0x17},
	{0x3455, 0x00},
	{0x3472, 0x9c},
	{0x3473, 0x07},
	{0x3480, 0x49},
	{REG_NULL, 0x00},
};

/*
 * Xclk 37.125Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 445.5Mbps
 */
static const struct regval imx291_1920x1080_regs[] = {
	{REG_NULL, 0x00},
};

static const struct imx291_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x03fe,
		.hts_def = 0x1130,
		.vts_def = 0x0465,
		.reg_list = imx291_1920x1080_regs,
	},
};

static const s64 link_freq_menu_items[] = {
	IMX291_LINK_FREQ
};

#ifdef USED_TEST_PATTERN
static const char * const imx291_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};
#endif

/* Write registers up to 4 at a time */
static int imx291_write_reg(struct i2c_client *client, u16 reg,
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

static int imx291_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		if (unlikely(regs[i].addr == REG_DELAY))
			usleep_range(regs[i].val * 1000, regs[i].val * 2000);
		else
			ret = imx291_write_reg(client, regs[i].addr,
						IMX291_REG_VALUE_08BIT,
						regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int imx291_read_reg(struct i2c_client *client, u16 reg,
			   unsigned int len, u32 *val)
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

static int imx291_get_reso_dist(const struct imx291_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct imx291_mode *
imx291_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = imx291_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}
	return &supported_modes[cur_best_fit];
}

static int imx291_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct imx291 *imx291 = to_imx291(sd);
	const struct imx291_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&imx291->mutex);

	mode = imx291_find_best_fit(fmt);
	fmt->format.code = IMX291_DATA_FORMAT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&imx291->mutex);
		return -ENOTTY;
#endif
	} else {
		imx291->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(imx291->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(imx291->vblank, vblank_def,
					 IMX291_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&imx291->mutex);

	return 0;
}

static int imx291_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct imx291 *imx291 = to_imx291(sd);
	const struct imx291_mode *mode = imx291->cur_mode;

	mutex_lock(&imx291->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&imx291->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = IMX291_DATA_FORMAT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&imx291->mutex);
	return 0;
}

static int imx291_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = IMX291_DATA_FORMAT;

	return 0;
}

static int imx291_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx291 *imx291 = to_imx291(sd);

	if (fse->index >= imx291->support_modes_num)
		return -EINVAL;

	if (fse->code != IMX291_DATA_FORMAT)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

#ifdef USED_TEST_PATTERN
static int imx291_enable_test_pattern(struct imx291 *imx291, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | IMX291_TEST_PATTERN_ENABLE;
	else
		val = IMX291_TEST_PATTERN_DISABLE;

	return imx291_write_reg(imx291->client,
				IMX291_REG_TEST_PATTERN,
				IMX291_REG_VALUE_08BIT,
				val);
}
#endif

static int imx291_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct imx291 *imx291 = to_imx291(sd);
	const struct imx291_mode *mode = imx291->cur_mode;

	mutex_lock(&imx291->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&imx291->mutex);

	return 0;
}

static void imx291_get_module_inf(struct imx291 *imx291,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, IMX291_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, imx291->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, imx291->len_name, sizeof(inf->base.lens));
}

static long imx291_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct imx291 *imx291 = to_imx291(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		imx291_get_module_inf(imx291, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long imx291_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = imx291_ioctl(sd, cmd, inf);
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
			ret = imx291_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif
static int __imx291_start_stream(struct imx291 *imx291)
{
	int ret;

	ret = imx291_write_array(imx291->client, imx291->cur_mode->reg_list);
	if (ret)
		return ret;
	/* In case these controls are set before streaming */
	mutex_unlock(&imx291->mutex);
	ret = v4l2_ctrl_handler_setup(&imx291->ctrl_handler);
	mutex_lock(&imx291->mutex);
	if (ret)
		return ret;

	ret = imx291_write_reg(imx291->client,
			       IMX291_REG_CTRL_MODE,
			       IMX291_REG_VALUE_08BIT,
			       0);
	return ret;
}

static int __imx291_stop_stream(struct imx291 *imx291)
{
	return imx291_write_reg(imx291->client,
				IMX291_REG_CTRL_MODE,
				IMX291_REG_VALUE_08BIT,
				1);
}

static int imx291_s_stream(struct v4l2_subdev *sd, int on)
{
	struct imx291 *imx291 = to_imx291(sd);
	struct i2c_client *client = imx291->client;
	int ret = 0;

	mutex_lock(&imx291->mutex);
	on = !!on;
	if (on == imx291->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __imx291_start_stream(imx291);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__imx291_stop_stream(imx291);
		pm_runtime_put(&client->dev);
	}

	imx291->streaming = on;

unlock_and_return:
	mutex_unlock(&imx291->mutex);

	return ret;
}

static int imx291_s_power(struct v4l2_subdev *sd, int on)
{
	struct imx291 *imx291 = to_imx291(sd);
	struct i2c_client *client = imx291->client;
	int ret = 0;

	mutex_lock(&imx291->mutex);

	/* If the power state is not modified - no work to do. */
	if (imx291->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = imx291_write_array(imx291->client, imx291_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		imx291->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		imx291->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&imx291->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 imx291_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, IMX291_XVCLK_FREQ / 1000 / 1000);
}

static int __imx291_power_on(struct imx291 *imx291)
{
	int ret;
	u32 delay_us;
	struct device *dev = &imx291->client->dev;

	if (!IS_ERR_OR_NULL(imx291->pins_default)) {
		ret = pinctrl_select_state(imx291->pinctrl,
					   imx291->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	ret = clk_set_rate(imx291->xvclk, IMX291_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (37.125M Hz)\n");

	if (clk_get_rate(imx291->xvclk) != IMX291_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched,based on 24M Hz\n");

	ret = clk_prepare_enable(imx291->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(imx291->reset_gpio))
		gpiod_set_value_cansleep(imx291->reset_gpio, 0);

	ret = regulator_bulk_enable(IMX291_NUM_SUPPLIES, imx291->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(imx291->reset_gpio))
		gpiod_set_value_cansleep(imx291->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(imx291->pwdn_gpio))
		gpiod_set_value_cansleep(imx291->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = imx291_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(imx291->xvclk);

	return ret;
}

static void __imx291_power_off(struct imx291 *imx291)
{
	int ret;
	struct device *dev = &imx291->client->dev;

	if (!IS_ERR(imx291->pwdn_gpio))
		gpiod_set_value_cansleep(imx291->pwdn_gpio, 0);
	clk_disable_unprepare(imx291->xvclk);
	if (!IS_ERR(imx291->reset_gpio))
		gpiod_set_value_cansleep(imx291->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(imx291->pins_sleep)) {
		ret = pinctrl_select_state(imx291->pinctrl,
					   imx291->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(IMX291_NUM_SUPPLIES, imx291->supplies);
}

static int imx291_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx291 *imx291 = to_imx291(sd);

	return __imx291_power_on(imx291);
}

static int imx291_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx291 *imx291 = to_imx291(sd);

	__imx291_power_off(imx291);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int imx291_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx291 *imx291 = to_imx291(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct imx291_mode *def_mode = &supported_modes[0];

	mutex_lock(&imx291->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = IMX291_DATA_FORMAT;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&imx291->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int imx291_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct imx291 *imx291 = to_imx291(sd);

	if (fie->index >= imx291->support_modes_num)
		return -EINVAL;

	if (fie->code != IMX291_DATA_FORMAT)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static const struct dev_pm_ops imx291_pm_ops = {
	SET_RUNTIME_PM_OPS(imx291_runtime_suspend,
			   imx291_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops imx291_internal_ops = {
	.open = imx291_open,
};
#endif

static const struct v4l2_subdev_core_ops imx291_core_ops = {
	.s_power = imx291_s_power,
	.ioctl = imx291_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx291_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops imx291_video_ops = {
	.s_stream = imx291_s_stream,
	.g_frame_interval = imx291_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx291_pad_ops = {
	.enum_mbus_code = imx291_enum_mbus_code,
	.enum_frame_size = imx291_enum_frame_sizes,
	.enum_frame_interval = imx291_enum_frame_interval,
	.get_fmt = imx291_get_fmt,
	.set_fmt = imx291_set_fmt,
};

static const struct v4l2_subdev_ops imx291_subdev_ops = {
	.core	= &imx291_core_ops,
	.video	= &imx291_video_ops,
	.pad	= &imx291_pad_ops,
};

static int imx291_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx291 *imx291 = container_of(ctrl->handler,
					     struct imx291, ctrl_handler);
	struct i2c_client *client = imx291->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = imx291->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(imx291->exposure,
					 imx291->exposure->minimum, max,
					 imx291->exposure->step,
					 imx291->exposure->default_value);
		break;
	}

	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = imx291_write_reg(imx291->client,
					IMX291_REG_EXPOSURE_H,
					IMX291_REG_VALUE_08BIT,
					IMX291_FETCH_HIGH_BYTE_EXP(ctrl->val));
		ret |= imx291_write_reg(imx291->client,
					IMX291_REG_EXPOSURE_M,
					IMX291_REG_VALUE_08BIT,
					IMX291_FETCH_MID_BYTE_EXP(ctrl->val));
		ret |= imx291_write_reg(imx291->client,
					IMX291_REG_EXPOSURE_L,
					IMX291_REG_VALUE_08BIT,
					IMX291_FETCH_LOW_BYTE_EXP(ctrl->val));
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx291_write_reg(imx291->client,
					IMX291_REG_GAIN,
					IMX291_REG_VALUE_08BIT,
					ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = imx291_write_reg(imx291->client,
					IMX291_REG_VTS_H,
					IMX291_REG_VALUE_08BIT,
					IMX291_FETCH_HIGH_BYTE_VTS(ctrl->val +
					imx291->cur_mode->height));
		ret |= imx291_write_reg(imx291->client,
					IMX291_REG_VTS_M,
					IMX291_REG_VALUE_08BIT,
					IMX291_FETCH_MID_BYTE_VTS(ctrl->val +
					imx291->cur_mode->height));
		ret |= imx291_write_reg(imx291->client,
					IMX291_REG_VTS_L,
					IMX291_REG_VALUE_08BIT,
					IMX291_FETCH_LOW_BYTE_VTS(ctrl->val +
					imx291->cur_mode->height));
		break;
	case V4L2_CID_TEST_PATTERN:
#ifdef USED_TEST_PATTERN
		ret = imx291_enable_test_pattern(imx291, ctrl->val);
#endif
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx291_ctrl_ops = {
	.s_ctrl = imx291_set_ctrl,
};

static int imx291_initialize_controls(struct imx291 *imx291)
{
	const struct imx291_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &imx291->ctrl_handler;
	mode = imx291->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &imx291->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, IMX291_PIXEL_RATE, 1, IMX291_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;

	imx291->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (imx291->hblank)
		imx291->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;

	imx291->vblank = v4l2_ctrl_new_std(handler, &imx291_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				IMX291_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;

	imx291->exposure = v4l2_ctrl_new_std(handler, &imx291_ctrl_ops,
				V4L2_CID_EXPOSURE, IMX291_EXPOSURE_MIN,
				exposure_max, IMX291_EXPOSURE_STEP,
				mode->exp_def);

	imx291->anal_gain = v4l2_ctrl_new_std(handler, &imx291_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, IMX291_GAIN_MIN,
				IMX291_GAIN_MAX, IMX291_GAIN_STEP,
				IMX291_GAIN_DEFAULT);

#ifdef USED_TEST_PATTERN
	imx291->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&imx291_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(imx291_test_pattern_menu) - 1,
				0, 0, imx291_test_pattern_menu);
#endif
	if (handler->error) {
		ret = handler->error;
		dev_err(&imx291->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	imx291->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int imx291_check_sensor_id(struct imx291 *imx291,
				  struct i2c_client *client)
{
	struct device *dev = &imx291->client->dev;
	u32 id = 0;
	int ret;

	ret = imx291_read_reg(client, IMX291_REG_CHIP_ID,
			      IMX291_REG_VALUE_08BIT, &id);

	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}
	return ret;
}

static int imx291_configure_regulators(struct imx291 *imx291)
{
	unsigned int i;

	for (i = 0; i < IMX291_NUM_SUPPLIES; i++)
		imx291->supplies[i].supply = imx291_supply_names[i];

	return devm_regulator_bulk_get(&imx291->client->dev,
				       IMX291_NUM_SUPPLIES,
				       imx291->supplies);
}

static int imx291_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct imx291 *imx291;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	imx291 = devm_kzalloc(dev, sizeof(*imx291), GFP_KERNEL);
	if (!imx291)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &imx291->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &imx291->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &imx291->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &imx291->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}
	imx291->client = client;
	imx291->support_modes = supported_modes;
	imx291->support_modes_num = ARRAY_SIZE(supported_modes);
	imx291->cur_mode = &supported_modes[0];

	imx291->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(imx291->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	imx291->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(imx291->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	imx291->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(imx291->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = imx291_configure_regulators(imx291);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	imx291->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(imx291->pinctrl)) {
		imx291->pins_default =
			pinctrl_lookup_state(imx291->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(imx291->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		imx291->pins_sleep =
			pinctrl_lookup_state(imx291->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(imx291->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	mutex_init(&imx291->mutex);

	sd = &imx291->subdev;
	v4l2_i2c_subdev_init(sd, client, &imx291_subdev_ops);
	ret = imx291_initialize_controls(imx291);
	if (ret)
		goto err_destroy_mutex;

	ret = __imx291_power_on(imx291);
	if (ret)
		goto err_free_handler;

	ret = imx291_check_sensor_id(imx291, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	dev_err(dev, "set the video v4l2 subdev api\n");
	sd->internal_ops = &imx291_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	dev_err(dev, "set the media controller\n");
	imx291->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &imx291->pad, 0);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(imx291->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 imx291->module_index, facing,
		 IMX291_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);
	dev_err(dev, "v4l2 async register subdev success\n");
	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__imx291_power_off(imx291);
err_free_handler:
	v4l2_ctrl_handler_free(&imx291->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&imx291->mutex);

	return ret;
}

static int imx291_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx291 *imx291 = to_imx291(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&imx291->ctrl_handler);
	mutex_destroy(&imx291->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__imx291_power_off(imx291);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id imx291_of_match[] = {
	{ .compatible = "sony,imx291" },
	{},
};
MODULE_DEVICE_TABLE(of, imx291_of_match);
#endif

static const struct i2c_device_id imx291_match_id[] = {
	{ "sony,imx291", 0 },
	{ },
};

static struct i2c_driver imx291_i2c_driver = {
	.driver = {
		.name = IMX291_NAME,
		.pm = &imx291_pm_ops,
		.of_match_table = of_match_ptr(imx291_of_match),
	},
	.probe		= &imx291_probe,
	.remove		= &imx291_remove,
	.id_table	= imx291_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&imx291_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&imx291_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Sony imx291 sensor driver");
MODULE_LICENSE("GPL v2");
