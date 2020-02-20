// SPDX-License-Identifier: GPL-2.0
/*
 * ps5268 driver
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

#define PS5268_DATA_FORMAT MEDIA_BUS_FMT_SBGGR10_1X10

#define PS5268_LINK_FREQ		200000000
#define PS5268_LANES			2
#define PS5268_BITS_PER_SAMPLE		10

/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define PS5268_PIXEL_RATE		(PS5268_LINK_FREQ * 2 * PS5268_LANES / PS5268_BITS_PER_SAMPLE)

#define PS5268_XVCLK_FREQ		24000000

#define CHIP_ID				0x5268
#define PS5268_REG_CHIP_ID_H		0x0100
#define PS5268_REG_CHIP_ID_L		0x0101

#define PS5268_REG_CTRL_MODE		0x010f
#define PS5268_MODE_SW_STANDBY		0x0
#define PS5268_MODE_STREAMING		0x1

#define PS5268_REG_EXPOSURE_H		0x0118
#define PS5268_REG_EXPOSURE_L		0x0119

#define PS5268_FETCH_HIGH_BYTE_EXP(VAL)	(((VAL) >> 8) & 0xFF)
#define PS5268_FETCH_LOW_BYTE_EXP(VAL)	((VAL) & 0xFF)

#define	PS5268_EXPOSURE_MIN		2
#define	PS5268_EXPOSURE_STEP		1
#define PS5268_VTS_MAX			0x7fff

#define PS5268_REG_CONVERSION_GAIN_SW	0X0128
#define PS5268_REG_GAIN			0x012B
#define PS5268_GAIN_MIN			0x10
#define PS5268_GAIN_MAX			0x400
#define PS5268_GAIN_STEP		1
#define PS5268_GAIN_DEFAULT		0x10

#ifdef USED_TEST_PATTERN
#define PS5268_REG_TEST_PATTERN		0x5e00
#define	PS5268_TEST_PATTERN_ENABLE	0x80
#define	PS5268_TEST_PATTERN_DISABLE	0x0
#endif

#define PS5268_REG_VTS_H		0x0116
#define PS5268_REG_VTS_L		0x0117
#define PS5268_FETCH_HIGH_BYTE_VTS(VAL)	(((VAL) >> 8) & 0xFF)
#define PS5268_FETCH_LOW_BYTE_VTS(VAL)	((VAL) & 0xFF)

#define PS5268_REG_UPDATA		0X0111

#define REG_NULL			0xFFFF
#define REG_DELAY			0xFFFE

#define PS5268_REG_VALUE_08BIT		1
#define PS5268_REG_VALUE_16BIT		2
#define PS5268_REG_VALUE_24BIT		3

#define PS5268_NAME			"ps5268"

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

static const char * const ps5268_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define PS5268_NUM_SUPPLIES ARRAY_SIZE(ps5268_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct ps5268_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct ps5268 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[PS5268_NUM_SUPPLIES];

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
	const struct ps5268_mode *support_modes;
	u32			support_modes_num;
	const struct ps5268_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_ps5268(sd) container_of(sd, struct ps5268, subdev)

/*
 * Xclk 24Mhz
 * mipi 2 lane
 */
static const struct regval ps5268_global_regs[] = {
	{0x010B, 0x07},
	{0x0114, 0x11},
	{0x0115, 0x98},
	{0x0116, 0x05},
	{0x0117, 0x46},
	{0x0140, 0x01}, /* mirror on */
	{0x0178, 0xC0},
	{0x0179, 0x1A},
	{0x022F, 0x24},
	{0x022D, 0x01},
	{0x0226, 0xB1},
	{0x0227, 0x39},
	{0x021C, 0x00},
	{0x0233, 0x70},
	{0x024B, 0x05},
	{0x024D, 0x11},
	{0x0252, 0x16},
	{0x0253, 0x26},
	{0x0254, 0x61},
	{0x0255, 0x11},
	{0x0664, 0x02},
	{0x0665, 0xAD},
	{0x0B02, 0x02},
	{0x0B0A, 0xFF},
	{0x0B0C, 0x00},
	{0x1200, 0x00},
	{0x1201, 0x01},
	{0x1300, 0x00},
	{0x1301, 0x01},
	{0x1303, 0x02},
	{0x1306, 0x99},
	{0x1307, 0x89},
	{0x1308, 0x36},
	{0x1309, 0x48},
	{0x130A, 0x23},
	{0x130B, 0x11},
	{0x130C, 0x40},
	{0x130D, 0x50},
	{0x130E, 0x60},
	{0x130F, 0x80},
	{0x1310, 0x80},
	{0x1311, 0x80},
	{0x1409, 0x1A},
	{0x140A, 0x15},
	{0x1411, 0x01},
	{0x1415, 0x04},
	{0x1417, 0x03},
	{0x1418, 0x02},
	{0x1406, 0x04},
	{0x1410, 0x02},
	{0x140F, 0x01},
	{0x0111, 0x01},
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 400Mbps
 */
static const struct regval ps5268_1920x1080_regs[] = {
	{REG_NULL, 0x00},
};

static const struct ps5268_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 250000,
		},
		.exp_def = 0x03f4,
		.hts_def = 0x1198,
		.vts_def = 0x0546,
		.reg_list = ps5268_1920x1080_regs,
	},
};

static const s64 link_freq_menu_items[] = {
	PS5268_LINK_FREQ
};

#ifdef USED_TEST_PATTERN
static const char * const ps5268_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};
#endif

/* Write registers up to 4 at a time */
static int ps5268_write_reg(struct i2c_client *client, u16 reg,
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

static int ps5268_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		if (unlikely(regs[i].addr == REG_DELAY))
			usleep_range(regs[i].val * 1000, regs[i].val * 2000);
		else
			ret = ps5268_write_reg(client, regs[i].addr,
					       PS5268_REG_VALUE_08BIT,
					       regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int ps5268_read_reg(struct i2c_client *client, u16 reg,
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

static int ps5268_get_reso_dist(const struct ps5268_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct ps5268_mode *
ps5268_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = ps5268_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}
	return &supported_modes[cur_best_fit];
}

static int ps5268_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ps5268 *ps5268 = to_ps5268(sd);
	const struct ps5268_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&ps5268->mutex);

	mode = ps5268_find_best_fit(fmt);
	fmt->format.code = PS5268_DATA_FORMAT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&ps5268->mutex);
		return -ENOTTY;
#endif
	} else {
		ps5268->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(ps5268->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height - 1;
		__v4l2_ctrl_modify_range(ps5268->vblank, vblank_def,
					 PS5268_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&ps5268->mutex);

	return 0;
}

static int ps5268_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ps5268 *ps5268 = to_ps5268(sd);
	const struct ps5268_mode *mode = ps5268->cur_mode;

	mutex_lock(&ps5268->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&ps5268->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = PS5268_DATA_FORMAT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&ps5268->mutex);
	return 0;
}

static int ps5268_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = PS5268_DATA_FORMAT;

	return 0;
}

static int ps5268_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct ps5268 *ps5268 = to_ps5268(sd);

	if (fse->index >= ps5268->support_modes_num)
		return -EINVAL;

	if (fse->code != PS5268_DATA_FORMAT)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

#ifdef USED_TEST_PATTERN
static int ps5268_enable_test_pattern(struct ps5268 *ps5268, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | PS5268_TEST_PATTERN_ENABLE;
	else
		val = PS5268_TEST_PATTERN_DISABLE;

	return ps5268_write_reg(ps5268->client,
				PS5268_REG_TEST_PATTERN,
				PS5268_REG_VALUE_08BIT,
				val);
}
#endif

static int ps5268_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ps5268 *ps5268 = to_ps5268(sd);
	const struct ps5268_mode *mode = ps5268->cur_mode;

	mutex_lock(&ps5268->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&ps5268->mutex);

	return 0;
}

static void ps5268_get_module_inf(struct ps5268 *ps5268,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, PS5268_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, ps5268->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, ps5268->len_name, sizeof(inf->base.lens));
}

static long ps5268_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ps5268 *ps5268 = to_ps5268(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		ps5268_get_module_inf(ps5268, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ps5268_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = ps5268_ioctl(sd, cmd, inf);
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
			ret = ps5268_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif
static int __ps5268_start_stream(struct ps5268 *ps5268)
{
	int ret;

	ret = ps5268_write_array(ps5268->client, ps5268->cur_mode->reg_list);
	if (ret)
		return ret;
	/* In case these controls are set before streaming */
	mutex_unlock(&ps5268->mutex);
	ret = v4l2_ctrl_handler_setup(&ps5268->ctrl_handler);
	mutex_lock(&ps5268->mutex);
	if (ret)
		return ret;

	ret = ps5268_write_reg(ps5268->client,
			       PS5268_REG_CTRL_MODE,
			       PS5268_REG_VALUE_08BIT,
			       PS5268_MODE_STREAMING);
	return ret;
}

static int __ps5268_stop_stream(struct ps5268 *ps5268)
{
	return ps5268_write_reg(ps5268->client,
				PS5268_REG_CTRL_MODE,
				PS5268_REG_VALUE_08BIT,
				PS5268_MODE_SW_STANDBY);
}

static int ps5268_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ps5268 *ps5268 = to_ps5268(sd);
	struct i2c_client *client = ps5268->client;
	int ret = 0;

	mutex_lock(&ps5268->mutex);
	on = !!on;
	if (on == ps5268->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __ps5268_start_stream(ps5268);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put_autosuspend(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__ps5268_stop_stream(ps5268);
		pm_runtime_put_autosuspend(&client->dev);
	}

	ps5268->streaming = on;

unlock_and_return:
	mutex_unlock(&ps5268->mutex);

	return ret;
}

static int ps5268_s_power(struct v4l2_subdev *sd, int on)
{
	struct ps5268 *ps5268 = to_ps5268(sd);
	struct i2c_client *client = ps5268->client;
	int ret = 0;

	mutex_lock(&ps5268->mutex);

	/* If the power state is not modified - no work to do. */
	if (ps5268->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = ps5268_write_array(ps5268->client, ps5268_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_autosuspend(&client->dev);
			goto unlock_and_return;
		}

		ps5268->power_on = true;
	} else {
		pm_runtime_mark_last_busy(&client->dev);
		pm_runtime_put_autosuspend(&client->dev);
		ps5268->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&ps5268->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 ps5268_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, PS5268_XVCLK_FREQ / 1000 / 1000);
}

static int __ps5268_power_on(struct ps5268 *ps5268)
{
	int ret;
	u32 delay_us;
	struct device *dev = &ps5268->client->dev;

	if (!IS_ERR_OR_NULL(ps5268->pins_default)) {
		ret = pinctrl_select_state(ps5268->pinctrl,
					   ps5268->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	ret = clk_set_rate(ps5268->xvclk, PS5268_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24M Hz)\n");

	if (clk_get_rate(ps5268->xvclk) != PS5268_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched,based on 24M Hz\n");

	ret = clk_prepare_enable(ps5268->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(ps5268->reset_gpio))
		gpiod_set_value_cansleep(ps5268->reset_gpio, 0);

	ret = regulator_bulk_enable(PS5268_NUM_SUPPLIES, ps5268->supplies);
	usleep_range(1000, 2000);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(ps5268->reset_gpio))
		gpiod_set_value_cansleep(ps5268->reset_gpio, 1);

	if (!IS_ERR(ps5268->pwdn_gpio))
		gpiod_set_value_cansleep(ps5268->pwdn_gpio, 1);
	msleep(65);
	/* 8192 cycles prior to first SCCB transaction */
	delay_us = ps5268_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(ps5268->xvclk);

	return ret;
}

static void __ps5268_power_off(struct ps5268 *ps5268)
{
	int ret;
	struct device *dev = &ps5268->client->dev;

	if (!IS_ERR(ps5268->pwdn_gpio))
		gpiod_set_value_cansleep(ps5268->pwdn_gpio, 0);
	clk_disable_unprepare(ps5268->xvclk);
	if (!IS_ERR(ps5268->reset_gpio))
		gpiod_set_value_cansleep(ps5268->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(ps5268->pins_sleep)) {
		ret = pinctrl_select_state(ps5268->pinctrl,
					   ps5268->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(PS5268_NUM_SUPPLIES, ps5268->supplies);
}

static int ps5268_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ps5268 *ps5268 = to_ps5268(sd);

	return __ps5268_power_on(ps5268);
}

static int ps5268_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ps5268 *ps5268 = to_ps5268(sd);

	__ps5268_power_off(ps5268);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ps5268_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ps5268 *ps5268 = to_ps5268(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct ps5268_mode *def_mode = &supported_modes[0];

	mutex_lock(&ps5268->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = PS5268_DATA_FORMAT;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&ps5268->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int ps5268_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct ps5268 *ps5268 = to_ps5268(sd);

	if (fie->index >= ps5268->support_modes_num)
		return -EINVAL;

	if (fie->code != PS5268_DATA_FORMAT)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static const struct dev_pm_ops ps5268_pm_ops = {
	SET_RUNTIME_PM_OPS(ps5268_runtime_suspend,
			   ps5268_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops ps5268_internal_ops = {
	.open = ps5268_open,
};
#endif

static const struct v4l2_subdev_core_ops ps5268_core_ops = {
	.s_power = ps5268_s_power,
	.ioctl = ps5268_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ps5268_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops ps5268_video_ops = {
	.s_stream = ps5268_s_stream,
	.g_frame_interval = ps5268_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ps5268_pad_ops = {
	.enum_mbus_code = ps5268_enum_mbus_code,
	.enum_frame_size = ps5268_enum_frame_sizes,
	.enum_frame_interval = ps5268_enum_frame_interval,
	.get_fmt = ps5268_get_fmt,
	.set_fmt = ps5268_set_fmt,
};

static const struct v4l2_subdev_ops ps5268_subdev_ops = {
	.core	= &ps5268_core_ops,
	.video	= &ps5268_video_ops,
	.pad	= &ps5268_pad_ops,
};

static int ps5268_set_gain(struct i2c_client *client, u32 gain)
{
	static u32 sghd = 1;
	u32 tal_gain = 0;
	u32 tmp = 0;
	u32 data = 0;
	int ret = 0;

	tal_gain = gain < 0x10 ? 0x10 : (gain > 0x400 ? 0x400 : gain);
	sghd = tal_gain > 0x240 ? 0 : (tal_gain < 0x180 ? 1 : sghd);
	if (sghd == 0)
		tal_gain >>= 1;
	tal_gain = tal_gain < 0x10 ? 0x10 : (tal_gain > 0x200 ? 0x200 : tal_gain);
	data = (tal_gain >> 4);

	while (data >>= 1)
		tmp++;

	if (tmp > 4)
		tmp = 4;
	tmp = (tmp << 4) + (tal_gain >> tmp) - 0x10;
	ret = ps5268_write_reg(client,
				PS5268_REG_CONVERSION_GAIN_SW,
				PS5268_REG_VALUE_08BIT,
				sghd);
	ret |= ps5268_write_reg(client,
				PS5268_REG_GAIN,
				PS5268_REG_VALUE_08BIT,
				tmp);
	ret |= ps5268_write_reg(client,
				PS5268_REG_UPDATA,
				PS5268_REG_VALUE_08BIT,
				0x01);
	return ret;
}

static int ps5268_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ps5268 *ps5268 = container_of(ctrl->handler,
					     struct ps5268, ctrl_handler);
	struct i2c_client *client = ps5268->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = ps5268->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(ps5268->exposure,
					 ps5268->exposure->minimum, max,
					 ps5268->exposure->step,
					 ps5268->exposure->default_value);
		break;
	}

	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = ps5268_write_reg(ps5268->client,
					PS5268_REG_EXPOSURE_H,
					PS5268_REG_VALUE_08BIT,
					PS5268_FETCH_HIGH_BYTE_EXP(ctrl->val));
		ret |= ps5268_write_reg(ps5268->client,
					PS5268_REG_EXPOSURE_L,
					PS5268_REG_VALUE_08BIT,
					PS5268_FETCH_LOW_BYTE_EXP(ctrl->val));
		ret |= ps5268_write_reg(ps5268->client,
					PS5268_REG_UPDATA,
					PS5268_REG_VALUE_08BIT,
					0x01);
		dev_dbg(&client->dev, "%s set exposure, val:0x%x\n",
			 __func__, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ps5268_set_gain(client, ctrl->val);
		dev_dbg(&client->dev, "%s set gain, val:0x%x\n",
			 __func__, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = ps5268_write_reg(ps5268->client,
					PS5268_REG_VTS_H,
					PS5268_REG_VALUE_08BIT,
					PS5268_FETCH_HIGH_BYTE_VTS(ctrl->val +
					ps5268->cur_mode->height));
		ret |= ps5268_write_reg(ps5268->client,
					PS5268_REG_VTS_L,
					PS5268_REG_VALUE_08BIT,
					PS5268_FETCH_LOW_BYTE_VTS(ctrl->val +
					ps5268->cur_mode->height));
		ret |= ps5268_write_reg(ps5268->client,
					PS5268_REG_UPDATA,
					PS5268_REG_VALUE_08BIT,
					0x01);
		break;
	case V4L2_CID_TEST_PATTERN:
#ifdef USED_TEST_PATTERN
		ret = ps5268_enable_test_pattern(ps5268, ctrl->val);
#endif
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ps5268_ctrl_ops = {
	.s_ctrl = ps5268_set_ctrl,
};

static int ps5268_initialize_controls(struct ps5268 *ps5268)
{
	const struct ps5268_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &ps5268->ctrl_handler;
	mode = ps5268->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &ps5268->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, PS5268_PIXEL_RATE, 1, PS5268_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;

	ps5268->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (ps5268->hblank)
		ps5268->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;

	ps5268->vblank = v4l2_ctrl_new_std(handler, &ps5268_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				PS5268_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;

	ps5268->exposure = v4l2_ctrl_new_std(handler, &ps5268_ctrl_ops,
				V4L2_CID_EXPOSURE, PS5268_EXPOSURE_MIN,
				exposure_max, PS5268_EXPOSURE_STEP,
				mode->exp_def);

	ps5268->anal_gain = v4l2_ctrl_new_std(handler, &ps5268_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, PS5268_GAIN_MIN,
				PS5268_GAIN_MAX, PS5268_GAIN_STEP,
				PS5268_GAIN_DEFAULT);

#ifdef USED_TEST_PATTERN
	ps5268->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&ps5268_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(ps5268_test_pattern_menu) - 1,
				0, 0, ps5268_test_pattern_menu);
#endif
	if (handler->error) {
		ret = handler->error;
		dev_err(&ps5268->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	ps5268->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ps5268_check_sensor_id(struct ps5268 *ps5268,
				  struct i2c_client *client)
{
	struct device *dev = &ps5268->client->dev;
	u32 id_h = 0;
	u32 id_l = 0;
	u32 id = 0;
	int ret;

	ret = ps5268_read_reg(client, PS5268_REG_CHIP_ID_H,
			      PS5268_REG_VALUE_08BIT, &id_h);
	ret |= ps5268_read_reg(client, PS5268_REG_CHIP_ID_L,
			       PS5268_REG_VALUE_08BIT, &id_l);
	id = (id_h << 8) | id_l;
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -EINVAL;
	}
	dev_info(dev, "sensor id(%06x), ret(%d)\n", id, ret);
	return 0;
}

static int ps5268_configure_regulators(struct ps5268 *ps5268)
{
	unsigned int i;

	for (i = 0; i < PS5268_NUM_SUPPLIES; i++)
		ps5268->supplies[i].supply = ps5268_supply_names[i];

	return devm_regulator_bulk_get(&ps5268->client->dev,
				       PS5268_NUM_SUPPLIES,
				       ps5268->supplies);
}

static int ps5268_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct ps5268 *ps5268;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	ps5268 = devm_kzalloc(dev, sizeof(*ps5268), GFP_KERNEL);
	if (!ps5268)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &ps5268->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &ps5268->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &ps5268->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &ps5268->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}
	ps5268->client = client;
	ps5268->support_modes = supported_modes;
	ps5268->support_modes_num = ARRAY_SIZE(supported_modes);
	ps5268->cur_mode = &supported_modes[0];

	ps5268->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(ps5268->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	ps5268->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ps5268->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	ps5268->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(ps5268->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = ps5268_configure_regulators(ps5268);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	ps5268->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(ps5268->pinctrl)) {
		ps5268->pins_default =
			pinctrl_lookup_state(ps5268->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(ps5268->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		ps5268->pins_sleep =
			pinctrl_lookup_state(ps5268->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(ps5268->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	mutex_init(&ps5268->mutex);

	sd = &ps5268->subdev;
	v4l2_i2c_subdev_init(sd, client, &ps5268_subdev_ops);
	ret = ps5268_initialize_controls(ps5268);
	if (ret)
		goto err_destroy_mutex;

	pm_runtime_set_autosuspend_delay(dev, 10 * 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	ret = pm_runtime_get_sync(dev);
	if (ret) {
		pm_runtime_put_noidle(dev);
		goto err_pm_disable;
	}

	ret = ps5268_check_sensor_id(ps5268, client);
	if (ret)
		goto err_pm_put;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	dev_err(dev, "set the video v4l2 subdev api\n");
	sd->internal_ops = &ps5268_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	dev_err(dev, "set the media controller\n");
	ps5268->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &ps5268->pad, 0);
	if (ret < 0)
		goto err_pm_put;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(ps5268->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 ps5268->module_index, facing,
		 PS5268_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_put_autosuspend(dev);

	dev_err(dev, "v4l2 async register subdev success\n");
	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_pm_put:
	pm_runtime_put(dev);
err_pm_disable:
	pm_runtime_disable(dev);
	v4l2_ctrl_handler_free(&ps5268->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ps5268->mutex);

	return ret;
}

static int ps5268_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ps5268 *ps5268 = to_ps5268(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&ps5268->ctrl_handler);
	mutex_destroy(&ps5268->mutex);

	pm_runtime_dont_use_autosuspend(&client->dev);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__ps5268_power_off(ps5268);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ps5268_of_match[] = {
	{ .compatible = "primesensor,ps5268" },
	{},
};
MODULE_DEVICE_TABLE(of, ps5268_of_match);
#endif

static const struct i2c_device_id ps5268_match_id[] = {
	{ "primesensor,ps5268", 0 },
	{ },
};

static struct i2c_driver ps5268_i2c_driver = {
	.driver = {
		.name = PS5268_NAME,
		.pm = &ps5268_pm_ops,
		.of_match_table = of_match_ptr(ps5268_of_match),
	},
	.probe		= &ps5268_probe,
	.remove		= &ps5268_remove,
	.id_table	= ps5268_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&ps5268_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&ps5268_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Sony ps5268 sensor driver");
MODULE_LICENSE("GPL v2");
