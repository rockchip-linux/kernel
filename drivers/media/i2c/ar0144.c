// SPDX-License-Identifier: GPL-2.0
/*
 * ar0144 driver
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

#define AR0144_DATA_FORMAT MEDIA_BUS_FMT_SGRBG12_1X12

#define AR0144_LANES			1
#define AR0144_BITS_PER_SAMPLE		12
#define AR0144_LINK_FREQ_378MHZ		378000000
/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define AR0144_PIXEL_RATE		(AR0144_LINK_FREQ_378MHZ * 2 * AR0144_LANES / AR0144_BITS_PER_SAMPLE)
#define AR0144_XVCLK_FREQ		24000000

#define CHIP_ID				0x0356
#define AR0144_REG_CHIP_ID		0x3000

#define AR0144_REG_CTRL_MODE		0x301a
#define AR0144_MODE_SW_STANDBY		0x0058
#define AR0144_MODE_STREAMING		0x005c

#define AR0144_REG_EXPOSURE		0x3012
#define	AR0144_EXPOSURE_MIN		4
#define	AR0144_EXPOSURE_STEP		1
#define AR0144_VTS_MAX			0x7fff

#define AR0144_REG_AGAIN		0x3060
#define AR0144_REG_DGAIN		0x305E
#define AR0144_FETCH_AGAIN(val)		((val) & 0x007f)
#define AR0144_FETCH_DGAIN(val)		((val) & 0x07FF)
#define AR0144_GAIN_MIN			0x80
#define AR0144_GAIN_MAX			0x800
#define AR0144_GAIN_STEP		1
#define AR0144_GAIN_DEFAULT		0x80

#define AR0144_REG_TEST_PATTERN		0x3070
#define	AR0144_TEST_PATTERN_ENABLE	0x0
#define	AR0144_TEST_PATTERN_DISABLE	0x0

#define AR0144_REG_VTS			0x300a

#define REG_NULL			0xFFFF

#define AR0144_REG_VALUE_08BIT		1
#define AR0144_REG_VALUE_16BIT		2
#define AR0144_REG_VALUE_24BIT		3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define AR0144_NAME			"ar0144"

static const char * const ar0144_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define AR0144_NUM_SUPPLIES ARRAY_SIZE(ar0144_supply_names)

struct regval {
	u16 addr;
	u16 val;
};

struct ar0144_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct ar0144 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[AR0144_NUM_SUPPLIES];

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
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct ar0144_mode *support_modes;
	u32			support_modes_num;
	const struct ar0144_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_ar0144(sd) container_of(sd, struct ar0144, subdev)

/*
 * Xclk 24Mhz
 * max_framerate 25fps
 * mipi 1 lane
 * mipi_datarate per lane 378Mbps
 */
static const struct regval ar0144_global_regs[] = {
	{0x3F4C, 0x4B3F},
	{0x3F4C, 0x003F},
	{0x3F4E, 0x5718},
	{0x3F4E, 0x0018},
	{0x3F50, 0x401F},
	{0x3F50, 0x17DF},
	{0x30B0, 0x0028},
	{0x3ED6, 0x3CB5},
	{0x3ED8, 0x8765},
	{0x3EDA, 0x8888},
	{0x3EDC, 0x97FF},
	{0x3EF8, 0x6522},
	{0x3EFA, 0x2222},
	{0x3EFC, 0x6666},
	{0x3F00, 0xAA05},
	{0x3EE2, 0x180E},
	{0x3EE4, 0x0808},
	{0x3EEA, 0x2A09},
	{0x3060, 0x000D},
	{0x3092, 0x00CF},
	{0x3268, 0x0030},
	{0x3786, 0x0006},
	{0x3F4A, 0x0F70},
	{0x306E, 0x4810},
	{0x3064, 0x1802},
	{0x3EF6, 0x804D},
	{0x3180, 0xC08F},
	{0x30BA, 0x7623},
	{0x3176, 0x0480},
	{0x3178, 0x0480},
	{0x317A, 0x0480},
	{0x317C, 0x0480},
	{0x302A, 0x0006},
	{0x302C, 0x0002},
	{0x302E, 0x0009},
	{0x3030, 0x008E},
	{0x3036, 0x000C},
	{0x30B0, 0x0028},
	{0x3038, 0x0001},
	{0x31AE, 0x0201},
	{0x31AC, 0x0C0C},
	{0x31B0, 0x002D},
	{0x31B2, 0x001B},
	{0x31B4, 0x1565},
	{0x31B6, 0x110D},
	{0x31B8, 0x2047},
	{0x31BA, 0x0105},
	{0x31BC, 0x0004},
	{0x3002, 0x0000},
	{0x3004, 0x0004},
	{0x3006, 0x031F},
	{0x3008, 0x0503},
	{0x300A, 0x03B5},
	{0x300C, 0x05D0},
	{0x3012, 0x03B4},
	{0x306E, 0x9010},
	{0x30A2, 0x0001},
	{0x30A6, 0x0001},
	{0x3040, 0x0000},
	{0x31D0, 0x0001},
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 */
static const struct regval ar0144_1280x800_regs[] = {
	{REG_NULL, 0x00},
};

static const struct ar0144_mode supported_modes[] = {
	{
		.width = 1280,
		.height = 800,
		.max_fps = {
			.numerator = 10000,
			.denominator = 250000,
		},
		.exp_def = 0x03b0,
		.hts_def = 0x05D0,
		.vts_def = 0x03b5,
		.reg_list = ar0144_1280x800_regs,
	},
};

static const s64 link_freq_menu_items[] = {
	AR0144_LINK_FREQ_378MHZ
};

static const char * const ar0144_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int ar0144_write_reg(struct i2c_client *client, u16 reg,
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

static int ar0144_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = ar0144_write_reg(client, regs[i].addr,
					AR0144_REG_VALUE_16BIT,
					regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int ar0144_read_reg(struct i2c_client *client, u16 reg,
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

static int ar0144_get_reso_dist(const struct ar0144_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct ar0144_mode *
ar0144_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = ar0144_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int ar0144_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ar0144 *ar0144 = to_ar0144(sd);
	const struct ar0144_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&ar0144->mutex);

	mode = ar0144_find_best_fit(fmt);
	fmt->format.code = AR0144_DATA_FORMAT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&ar0144->mutex);
		return -ENOTTY;
#endif
	} else {
		ar0144->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(ar0144->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ar0144->vblank, vblank_def,
					 AR0144_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&ar0144->mutex);

	return 0;
}

static int ar0144_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ar0144 *ar0144 = to_ar0144(sd);
	const struct ar0144_mode *mode = ar0144->cur_mode;

	mutex_lock(&ar0144->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&ar0144->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = AR0144_DATA_FORMAT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&ar0144->mutex);

	return 0;
}

static int ar0144_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = AR0144_DATA_FORMAT;

	return 0;
}

static int ar0144_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != AR0144_DATA_FORMAT)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int ar0144_enable_test_pattern(struct ar0144 *ar0144, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | AR0144_TEST_PATTERN_ENABLE;
	else
		val = AR0144_TEST_PATTERN_DISABLE;

	return ar0144_write_reg(ar0144->client,
				AR0144_REG_TEST_PATTERN,
				AR0144_REG_VALUE_16BIT,
				val);
}

static int ar0144_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct ar0144 *ar0144 = to_ar0144(sd);
	const struct ar0144_mode *mode = ar0144->cur_mode;

	mutex_lock(&ar0144->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&ar0144->mutex);

	return 0;
}

static void ar0144_get_module_inf(struct ar0144 *ar0144,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, AR0144_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, ar0144->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, ar0144->len_name, sizeof(inf->base.lens));
}

static long ar0144_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ar0144 *ar0144 = to_ar0144(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		ar0144_get_module_inf(ar0144, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ar0144_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	long ret;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ar0144_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __ar0144_start_stream(struct ar0144 *ar0144)
{
	int ret;

	ret = ar0144_write_array(ar0144->client, ar0144->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&ar0144->mutex);
	ret = v4l2_ctrl_handler_setup(&ar0144->ctrl_handler);
	mutex_lock(&ar0144->mutex);
	if (ret)
		return ret;

	return ar0144_write_reg(ar0144->client,
				AR0144_REG_CTRL_MODE,
				AR0144_REG_VALUE_16BIT,
				AR0144_MODE_STREAMING);
}

static int __ar0144_stop_stream(struct ar0144 *ar0144)
{
	return ar0144_write_reg(ar0144->client,
				AR0144_REG_CTRL_MODE,
				AR0144_REG_VALUE_16BIT,
				AR0144_MODE_SW_STANDBY);
}

static int ar0144_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ar0144 *ar0144 = to_ar0144(sd);
	struct i2c_client *client = ar0144->client;
	int ret = 0;

	mutex_lock(&ar0144->mutex);
	on = !!on;
	if (on == ar0144->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __ar0144_start_stream(ar0144);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__ar0144_stop_stream(ar0144);
		pm_runtime_put(&client->dev);
	}

	ar0144->streaming = on;

unlock_and_return:
	mutex_unlock(&ar0144->mutex);

	return ret;
}

static int ar0144_s_power(struct v4l2_subdev *sd, int on)
{
	struct ar0144 *ar0144 = to_ar0144(sd);
	struct i2c_client *client = ar0144->client;
	int ret = 0;

	mutex_lock(&ar0144->mutex);

	/* If the power state is not modified - no work to do. */
	if (ar0144->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = ar0144_write_array(ar0144->client, ar0144_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ar0144->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		ar0144->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&ar0144->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 ar0144_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, AR0144_XVCLK_FREQ / 1000 / 1000);
}

static int __ar0144_power_on(struct ar0144 *ar0144)
{
	int ret;
	u32 delay_us;
	struct device *dev = &ar0144->client->dev;

	if (!IS_ERR_OR_NULL(ar0144->pins_default)) {
		ret = pinctrl_select_state(ar0144->pinctrl,
					   ar0144->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	ret = clk_set_rate(ar0144->xvclk, AR0144_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(ar0144->xvclk) != AR0144_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(ar0144->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(ar0144->reset_gpio))
		gpiod_set_value_cansleep(ar0144->reset_gpio, 1);

	ret = regulator_bulk_enable(AR0144_NUM_SUPPLIES, ar0144->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	usleep_range(1000, 1100);
	if (!IS_ERR(ar0144->reset_gpio))
		gpiod_set_value_cansleep(ar0144->reset_gpio, 0);

	usleep_range(500, 1000);
	if (!IS_ERR(ar0144->pwdn_gpio))
		gpiod_set_value_cansleep(ar0144->pwdn_gpio, 0);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = ar0144_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(ar0144->xvclk);

	return ret;
}

static void __ar0144_power_off(struct ar0144 *ar0144)
{
	int ret;
	struct device *dev = &ar0144->client->dev;

	if (!IS_ERR(ar0144->pwdn_gpio))
		gpiod_set_value_cansleep(ar0144->pwdn_gpio, 1);
	clk_disable_unprepare(ar0144->xvclk);
	if (!IS_ERR(ar0144->reset_gpio))
		gpiod_set_value_cansleep(ar0144->reset_gpio, 1);
	if (!IS_ERR_OR_NULL(ar0144->pins_sleep)) {
		ret = pinctrl_select_state(ar0144->pinctrl,
					   ar0144->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(AR0144_NUM_SUPPLIES, ar0144->supplies);
}

static int ar0144_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0144 *ar0144 = to_ar0144(sd);

	return __ar0144_power_on(ar0144);
}

static int ar0144_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0144 *ar0144 = to_ar0144(sd);

	__ar0144_power_off(ar0144);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ar0144_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ar0144 *ar0144 = to_ar0144(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct ar0144_mode *def_mode = &supported_modes[0];

	mutex_lock(&ar0144->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = AR0144_DATA_FORMAT;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&ar0144->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int ar0144_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	struct ar0144 *ar0144 = to_ar0144(sd);

	if (fie->index >= ar0144->support_modes_num)
		return -EINVAL;

	if (fie->code != AR0144_DATA_FORMAT)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static const struct dev_pm_ops ar0144_pm_ops = {
	SET_RUNTIME_PM_OPS(ar0144_runtime_suspend,
			   ar0144_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops ar0144_internal_ops = {
	.open = ar0144_open,
};
#endif

static const struct v4l2_subdev_core_ops ar0144_core_ops = {
	.s_power = ar0144_s_power,
	.ioctl = ar0144_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ar0144_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops ar0144_video_ops = {
	.s_stream = ar0144_s_stream,
	.g_frame_interval = ar0144_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ar0144_pad_ops = {
	.enum_mbus_code = ar0144_enum_mbus_code,
	.enum_frame_size = ar0144_enum_frame_sizes,
	.enum_frame_interval = ar0144_enum_frame_interval,
	.get_fmt = ar0144_get_fmt,
	.set_fmt = ar0144_set_fmt,
};

static const struct v4l2_subdev_ops ar0144_subdev_ops = {
	.core	= &ar0144_core_ops,
	.video	= &ar0144_video_ops,
	.pad	= &ar0144_pad_ops,
};

static void ar0144_get_gain_reg(u32 gain, u32 *again_reg, u32 *dgain_reg)
{
	if (gain < 256) {
		/* 1x~2x */
		*again_reg = 0x00;
		*dgain_reg = gain;
	} else if (gain < 512) {
		/* 2x~4x */
		*again_reg = 0x10;
		*dgain_reg = gain >> 1;
	} else if (gain < 1024) {
		/* 4x~8x*/
		*again_reg = 0x20;
		*dgain_reg = gain >> 2;
	} else {
		/* 8x~ */
		*again_reg = 0x30;
		*dgain_reg = gain >> 3;
	}
}

static int ar0144_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0144 *ar0144 = container_of(ctrl->handler,
					     struct ar0144, ctrl_handler);
	struct i2c_client *client = ar0144->client;
	s64 max;
	int ret = 0;
	u32 again, dgain;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = ar0144->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(ar0144->exposure,
					 ar0144->exposure->minimum, max,
					 ar0144->exposure->step,
					 ar0144->exposure->default_value);
		break;
	}

	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = ar0144_write_reg(ar0144->client,
					AR0144_REG_EXPOSURE,
					AR0144_REG_VALUE_16BIT,
					ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ar0144_get_gain_reg(ctrl->val, &again, &dgain);
		ret = ar0144_write_reg(ar0144->client,
					AR0144_REG_AGAIN,
					AR0144_REG_VALUE_16BIT,
					AR0144_FETCH_AGAIN(again));
		ret = ar0144_write_reg(ar0144->client,
					AR0144_REG_DGAIN,
					AR0144_REG_VALUE_16BIT,
					AR0144_FETCH_DGAIN(dgain));
		break;
	case V4L2_CID_VBLANK:
		ret = ar0144_write_reg(ar0144->client,
					AR0144_REG_VTS,
					AR0144_REG_VALUE_16BIT,
					ctrl->val + ar0144->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ar0144_enable_test_pattern(ar0144, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ar0144_ctrl_ops = {
	.s_ctrl = ar0144_set_ctrl,
};

static int ar0144_initialize_controls(struct ar0144 *ar0144)
{
	const struct ar0144_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &ar0144->ctrl_handler;
	mode = ar0144->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &ar0144->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, AR0144_PIXEL_RATE, 1, AR0144_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	ar0144->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (ar0144->hblank)
		ar0144->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	ar0144->vblank = v4l2_ctrl_new_std(handler, &ar0144_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				AR0144_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	ar0144->exposure = v4l2_ctrl_new_std(handler, &ar0144_ctrl_ops,
				V4L2_CID_EXPOSURE, AR0144_EXPOSURE_MIN,
				exposure_max, AR0144_EXPOSURE_STEP,
				mode->exp_def);

	ar0144->anal_gain = v4l2_ctrl_new_std(handler, &ar0144_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, AR0144_GAIN_MIN,
				AR0144_GAIN_MAX, AR0144_GAIN_STEP,
				AR0144_GAIN_DEFAULT);

	ar0144->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&ar0144_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(ar0144_test_pattern_menu) - 1,
				0, 0, ar0144_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&ar0144->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	ar0144->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ar0144_check_sensor_id(struct ar0144 *ar0144,
				  struct i2c_client *client)
{
	struct device *dev = &ar0144->client->dev;
	u32 id = 0;
	int ret;

	ret = ar0144_read_reg(client, AR0144_REG_CHIP_ID,
			      AR0144_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return ret;
	}
	return 0;
}

static int ar0144_configure_regulators(struct ar0144 *ar0144)
{
	unsigned int i;

	for (i = 0; i < AR0144_NUM_SUPPLIES; i++)
		ar0144->supplies[i].supply = ar0144_supply_names[i];

	return devm_regulator_bulk_get(&ar0144->client->dev,
				       AR0144_NUM_SUPPLIES,
				       ar0144->supplies);
}

static int ar0144_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct ar0144 *ar0144;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	ar0144 = devm_kzalloc(dev, sizeof(*ar0144), GFP_KERNEL);
	if (!ar0144)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &ar0144->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &ar0144->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &ar0144->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &ar0144->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}
	ar0144->client = client;
	ar0144->support_modes = supported_modes;
	ar0144->support_modes_num = ARRAY_SIZE(supported_modes);
	ar0144->cur_mode = &supported_modes[0];

	ar0144->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(ar0144->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	ar0144->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ar0144->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	ar0144->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(ar0144->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = ar0144_configure_regulators(ar0144);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	ar0144->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(ar0144->pinctrl)) {
		ar0144->pins_default =
			pinctrl_lookup_state(ar0144->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(ar0144->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		ar0144->pins_sleep =
			pinctrl_lookup_state(ar0144->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(ar0144->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	mutex_init(&ar0144->mutex);

	sd = &ar0144->subdev;
	v4l2_i2c_subdev_init(sd, client, &ar0144_subdev_ops);
	ret = ar0144_initialize_controls(ar0144);
	if (ret)
		goto err_destroy_mutex;

	ret = __ar0144_power_on(ar0144);
	if (ret)
		goto err_free_handler;

	ret = ar0144_check_sensor_id(ar0144, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &ar0144_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	ar0144->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &ar0144->pad, 0);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(ar0144->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 ar0144->module_index, facing,
		 AR0144_NAME, dev_name(sd->dev));
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
	__ar0144_power_off(ar0144);
err_free_handler:
	v4l2_ctrl_handler_free(&ar0144->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ar0144->mutex);

	return ret;
}

static int ar0144_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0144 *ar0144 = to_ar0144(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&ar0144->ctrl_handler);
	mutex_destroy(&ar0144->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__ar0144_power_off(ar0144);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ar0144_of_match[] = {
	{ .compatible = "aptina,ar0144" },
	{},
};
MODULE_DEVICE_TABLE(of, ar0144_of_match);
#endif

static const struct i2c_device_id ar0144_match_id[] = {
	{ "aptina,ar0144", 0 },
	{ },
};

static struct i2c_driver ar0144_i2c_driver = {
	.driver = {
		.name = AR0144_NAME,
		.pm = &ar0144_pm_ops,
		.of_match_table = of_match_ptr(ar0144_of_match),
	},
	.probe		= &ar0144_probe,
	.remove		= &ar0144_remove,
	.id_table	= ar0144_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&ar0144_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&ar0144_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Aptina ar0144 sensor driver");
MODULE_LICENSE("GPL v2");
