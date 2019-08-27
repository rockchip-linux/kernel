// SPDX-License-Identifier: GPL-2.0
/*
 * imx327 driver LVDS
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
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
#include <linux/of_graph.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x0)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define IMX327_DATA_FORMAT MEDIA_BUS_FMT_SRGGB10_1X10

#define IMX327_LINK_FREQ		222750000
#define IMX327_LANES			4
#define IMX327_BITS_PER_SAMPLE		10

/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define IMX327_PIXEL_RATE \
(IMX327_LINK_FREQ * 2 * IMX327_LANES / IMX327_BITS_PER_SAMPLE)

#define IMX327_XVCLK_FREQ		37125000

#define CHIP_ID				0xa0
#define IMX327_REG_CHIP_ID		0x0208

#define IMX327_MODE_SW_STANDBY		0x1
#define IMX327_MODE_STREAMING		0x0

#define IMX327_REG_EXPOSURE_H		0x0222
#define IMX327_REG_EXPOSURE_M		0x0221
#define IMX327_REG_EXPOSURE_L		0x0220

#define IMX327_FETCH_HIGH_BYTE_EXP(VAL) (((VAL) >> 16) & 0x03)
#define IMX327_FETCH_MID_BYTE_EXP(VAL) (((VAL) >> 8) & 0xFF)
#define IMX327_FETCH_LOW_BYTE_EXP(VAL) ((VAL) & 0xFF)

#define	IMX327_EXPOSURE_MIN		4
#define	IMX327_EXPOSURE_STEP		1
#define IMX327_VTS_MAX			0x7fff

#define IMX327_REG_GAIN			0x0214
#define IMX327_GAIN_MIN			0x00
#define IMX327_GAIN_MAX			0xee
#define IMX327_GAIN_STEP		1
#define IMX327_GAIN_DEFAULT		0x00

#define IMX327_REG_VTS_H		0x021a
#define IMX327_REG_VTS_M		0x0219
#define IMX327_REG_VTS_L		0x0218
#define IMX327_FETCH_HIGH_BYTE_VTS(VAL) (((VAL) >> 16) & 0x03)
#define IMX327_FETCH_MID_BYTE_VTS(VAL) (((VAL) >> 8) & 0xFF)
#define IMX327_FETCH_LOW_BYTE_VTS(VAL) ((VAL) & 0xFF)

#define REG_NULL			0xFF

#define IMX327_NAME			"imx327"

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

static const char * const imx327_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define IMX327_NUM_SUPPLIES ARRAY_SIZE(imx327_supply_names)

struct regval {
	u8 id;
	u8 addr;
	u8 val;
};

struct imx327_mode {
	u32 width;
	u32 height;
	u32 max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct imx327 {
	struct spi_device	*spi;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[IMX327_NUM_SUPPLIES];
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
	const struct imx327_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_imx327(sd) container_of(sd, struct imx327, subdev)

/*
 * Xclk 37.125Mhz
 */
static const struct regval imx327_global_regs[] = {
	{0x02, 0x03, 0x01},
	{0x02, 0x00, 0x01},
	{0x02, 0x01, 0x00},
	{0x02, 0x02, 0x00},
	{0x02, 0x05, 0x01},
	{0x02, 0x07, 0x00},
	{0x02, 0x09, 0x02},
	{0x02, 0x0a, 0xf0},
	{0x02, 0x11, 0x02},
	{0x02, 0x18, 0x65},/* vMAX L */
	{0x02, 0x19, 0x04},/* VMAX M */
	{0x02, 0x1a, 0x00},/* VMAX H */
	{0x02, 0x1c, 0x30},/* HMAX L */
	{0x02, 0x1d, 0x11},/* HMAX H */
	{0x02, 0x46, 0xe1},
	{0x02, 0x4b, 0x0a},
	{0x02, 0x5c, 0x18},
	{0x02, 0x5d, 0x00},
	{0x02, 0x5e, 0x20},
	{0x02, 0x5f, 0x01},
	{0x02, 0x9e, 0x4a},
	{0x02, 0x9f, 0x4a},
	{0x02, 0xd2, 0x19},
	{0x02, 0xd7, 0x03},
	{0x03, 0x29, 0x00},
	{0x03, 0x3b, 0x61},
	{0x03, 0x5e, 0x1a},
	{0x03, 0x64, 0x1a},
	{0x03, 0x7c, 0x00},
	{0x03, 0xec, 0x0e},
	{0x06, 0x18, 0x49},
	{0x06, 0x19, 0x04},
	{0x06, 0x72, 0x9c},
	{0x06, 0x73, 0x07},
	{0x06, 0x80, 0x49},
	{0x00, REG_NULL, 0x00},
};

/*
 * Xclk 37.125Mhz
 * max_framerate 30fps
 * LVDS_datarate per lane 222.75Mbps
 */
static const struct regval imx327_1920x1080_regs[] = {
	{0x00, REG_NULL, 0x00},
};

static const struct imx327_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.max_fps = 30,
		.exp_def = 0x0300,
		.hts_def = 0x1130,
		.vts_def = 0x0465,
		.reg_list = imx327_1920x1080_regs,
	},
};

static const s64 link_freq_menu_items[] = {
	IMX327_LINK_FREQ
};

/* Write registers up to 4 at a time */
static int imx327_write_reg(struct spi_device *spi, u8 id, u8 reg, u8 val)
{
	int ret = 0;
	u8 buf_id = id;
	u8 buf_reg = reg;
	u8 buf_val = val;

	struct spi_message msg;
	struct spi_transfer tx[] = {
		{
			.tx_buf = &buf_id,
			.len = 1,
			.delay_usecs = 1,
		}, {
			.tx_buf = &buf_reg,
			.len = 1,
			.delay_usecs = 1,
		}, {
			.tx_buf = &buf_val,
			.len = 1,
			.delay_usecs = 1,
		},
	};
	spi_message_init(&msg);
	spi_message_add_tail(&tx[0], &msg);
	spi_message_add_tail(&tx[1], &msg);
	spi_message_add_tail(&tx[2], &msg);
	ret = spi_sync(spi, &msg);
	return ret;
}

static int imx327_write_array(struct spi_device *spi,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].id != 0; i++)
		ret = imx327_write_reg(spi, regs[i].id, regs[i].addr,
					regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int imx327_read_reg(struct spi_device *spi, u8 id, u8 reg, u8 *val)
{
	int ret = 0;
	u8 buf_id = id | 0x80;
	u8 buf_reg = reg;
	u8 buf_val = 0;

	struct spi_message msg;
	struct spi_transfer tx[] = {
		{
			.tx_buf = &buf_id,
			.len = 1,
			.delay_usecs = 1,
		}, {
			.tx_buf = &buf_reg,
			.len = 1,
			.delay_usecs = 1,
		}, {
			.rx_buf = &buf_val,
			.len = 1,
			.delay_usecs = 1,
		},
	};
	spi_message_init(&msg);
	spi_message_add_tail(&tx[0], &msg);
	spi_message_add_tail(&tx[1], &msg);
	spi_message_add_tail(&tx[2], &msg);
	ret = spi_sync(spi, &msg);
	*val = buf_val;

	return ret;
}

static int imx327_get_reso_dist(const struct imx327_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct imx327_mode *
imx327_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = imx327_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int imx327_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct imx327 *imx327 = to_imx327(sd);
	const struct imx327_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&imx327->mutex);

	mode = imx327_find_best_fit(fmt);
	fmt->format.code = IMX327_DATA_FORMAT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&imx327->mutex);
		return -ENOTTY;
#endif
	} else {
		imx327->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(imx327->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(imx327->vblank, vblank_def,
					 IMX327_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&imx327->mutex);

	return 0;
}

static int imx327_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct imx327 *imx327 = to_imx327(sd);
	const struct imx327_mode *mode = imx327->cur_mode;

	mutex_lock(&imx327->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&imx327->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = IMX327_DATA_FORMAT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&imx327->mutex);
	return 0;
}

static int imx327_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = IMX327_DATA_FORMAT;

	return 0;
}

static int imx327_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != IMX327_DATA_FORMAT)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int imx327_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct imx327 *imx327 = to_imx327(sd);
	const struct imx327_mode *mode = imx327->cur_mode;

	mutex_lock(&imx327->mutex);
	fi->interval.numerator = 10000;
	fi->interval.denominator = mode->max_fps * 10000;
	mutex_unlock(&imx327->mutex);

	return 0;
}

static void imx327_get_module_inf(struct imx327 *imx327,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, IMX327_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, imx327->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, imx327->len_name, sizeof(inf->base.lens));
}

static long imx327_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct imx327 *imx327 = to_imx327(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		imx327_get_module_inf(imx327, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long imx327_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = imx327_ioctl(sd, cmd, inf);
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
			ret = imx327_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __imx327_start_stream(struct imx327 *imx327)
{
	int ret;

	ret = imx327_write_array(imx327->spi, imx327->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&imx327->mutex);
	ret = v4l2_ctrl_handler_setup(&imx327->ctrl_handler);
	mutex_lock(&imx327->mutex);
	if (ret)
		return ret;

	return imx327_write_reg(imx327->spi,
				0x02, 0x00, 0x00);
}

static int __imx327_stop_stream(struct imx327 *imx327)
{
	return imx327_write_reg(imx327->spi,
				0x02, 0x00, 0x01);
}

static int imx327_s_stream(struct v4l2_subdev *sd, int on)
{
	struct imx327 *imx327 = to_imx327(sd);
	struct spi_device *spi = imx327->spi;
	int ret = 0;

	mutex_lock(&imx327->mutex);
	on = !!on;
	if (on == imx327->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&spi->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&spi->dev);
			goto unlock_and_return;
		}

		ret = __imx327_start_stream(imx327);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&spi->dev);
			goto unlock_and_return;
		}
	} else {
		__imx327_stop_stream(imx327);
		pm_runtime_put(&spi->dev);
	}

	imx327->streaming = on;

unlock_and_return:
	mutex_unlock(&imx327->mutex);

	return ret;
}

static int imx327_s_power(struct v4l2_subdev *sd, int on)
{
	struct imx327 *imx327 = to_imx327(sd);
	struct spi_device *spi = imx327->spi;
	int ret = 0;

	mutex_lock(&imx327->mutex);

	/* If the power state is not modified - no work to do. */
	if (imx327->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&spi->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&spi->dev);
			goto unlock_and_return;
		}

		ret = imx327_write_array(imx327->spi, imx327_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&spi->dev);
			goto unlock_and_return;
		}

		imx327->power_on = true;
	} else {
		pm_runtime_put(&spi->dev);
		imx327->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&imx327->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 imx327_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, IMX327_XVCLK_FREQ / 1000 / 1000);
}

static int __imx327_power_on(struct imx327 *imx327)
{
	int ret;
	u32 delay_us;
	struct device *dev = &imx327->spi->dev;

	if (!IS_ERR_OR_NULL(imx327->pins_default)) {
		ret = pinctrl_select_state(imx327->pinctrl,
					   imx327->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(imx327->xvclk, IMX327_XVCLK_FREQ);
	if (ret < 0) {
		dev_err(dev, "Failed to set xvclk rate (24MHz)\n");
		return ret;
	}
	if (clk_get_rate(imx327->xvclk) != IMX327_XVCLK_FREQ)
		dev_err(dev, "xvclk mismatched, modes are based on 37.125MHz\n");
	ret = clk_prepare_enable(imx327->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(imx327->reset_gpio))
		gpiod_set_value_cansleep(imx327->reset_gpio, 0);

	ret = regulator_bulk_enable(IMX327_NUM_SUPPLIES, imx327->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(imx327->reset_gpio))
		gpiod_set_value_cansleep(imx327->reset_gpio, 1);

	usleep_range(500, 1000);
	if (!IS_ERR(imx327->pwdn_gpio))
		gpiod_set_value_cansleep(imx327->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = imx327_cal_delay(81920);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(imx327->xvclk);

	return ret;
}

static void __imx327_power_off(struct imx327 *imx327)
{
	int ret = 0;

	if (!IS_ERR(imx327->pwdn_gpio))
		gpiod_set_value_cansleep(imx327->pwdn_gpio, 0);
	clk_disable_unprepare(imx327->xvclk);
	if (!IS_ERR(imx327->reset_gpio))
		gpiod_set_value_cansleep(imx327->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(imx327->pins_sleep)) {
		ret = pinctrl_select_state(imx327->pinctrl,
				   imx327->pins_sleep);
		if (ret < 0)
			dev_dbg(&imx327->spi->dev, "could not set pins\n");
	}
	regulator_bulk_disable(IMX327_NUM_SUPPLIES, imx327->supplies);
}

static int imx327_runtime_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct v4l2_subdev *sd = spi_get_drvdata(spi);
	struct imx327 *imx327 = to_imx327(sd);

	return __imx327_power_on(imx327);
}

static int imx327_runtime_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct v4l2_subdev *sd = spi_get_drvdata(spi);
	struct imx327 *imx327 = to_imx327(sd);

	__imx327_power_off(imx327);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int imx327_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx327 *imx327 = to_imx327(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct imx327_mode *def_mode = &supported_modes[0];

	mutex_lock(&imx327->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = IMX327_DATA_FORMAT;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&imx327->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static const struct dev_pm_ops imx327_pm_ops = {
	SET_RUNTIME_PM_OPS(imx327_runtime_suspend,
			   imx327_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops imx327_internal_ops = {
	.open = imx327_open,
};
#endif

static const struct v4l2_subdev_core_ops imx327_core_ops = {
	.s_power = imx327_s_power,
	.ioctl = imx327_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx327_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops imx327_video_ops = {
	.s_stream = imx327_s_stream,
	.g_frame_interval = imx327_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx327_pad_ops = {
	.enum_mbus_code = imx327_enum_mbus_code,
	.enum_frame_size = imx327_enum_frame_sizes,
	.get_fmt = imx327_get_fmt,
	.set_fmt = imx327_set_fmt,
};

static const struct v4l2_subdev_ops imx327_subdev_ops = {
	.core	= &imx327_core_ops,
	.video	= &imx327_video_ops,
	.pad	= &imx327_pad_ops,
};

static int imx327_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx327 *imx327 = container_of(ctrl->handler,
					     struct imx327, ctrl_handler);
	struct spi_device *spi = imx327->spi;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = imx327->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(imx327->exposure,
			imx327->exposure->minimum, max,
			imx327->exposure->step,
			imx327->exposure->default_value);
		break;
	}

	if (pm_runtime_get(&spi->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = imx327_write_reg(imx327->spi,
			(IMX327_REG_EXPOSURE_H >> 8) & 0xff,
			IMX327_REG_EXPOSURE_H & 0xff,
			IMX327_FETCH_HIGH_BYTE_EXP(ctrl->val));
		ret |= imx327_write_reg(imx327->spi,
			(IMX327_REG_EXPOSURE_M >> 8) & 0xff,
			IMX327_REG_EXPOSURE_M & 0xff,
			IMX327_FETCH_MID_BYTE_EXP(ctrl->val));
		ret |= imx327_write_reg(imx327->spi,
			(IMX327_REG_EXPOSURE_L >> 8) & 0xff,
			IMX327_REG_EXPOSURE_L & 0xff,
			IMX327_FETCH_LOW_BYTE_EXP(ctrl->val));
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx327_write_reg(imx327->spi,
			(IMX327_REG_GAIN >> 8) & 0xff,
			IMX327_REG_GAIN & 0xff,
			ctrl->val & 0xff);
		break;
	case V4L2_CID_VBLANK:
		ret = imx327_write_reg(imx327->spi,
			(IMX327_REG_VTS_H >> 8) & 0xff,
			IMX327_REG_VTS_H & 0xff,
			IMX327_FETCH_HIGH_BYTE_VTS(ctrl->val +
			imx327->cur_mode->height));
		ret |= imx327_write_reg(imx327->spi,
			(IMX327_REG_VTS_M >> 8) & 0xff,
			IMX327_REG_VTS_M & 0xff,
			IMX327_FETCH_MID_BYTE_VTS(ctrl->val +
			imx327->cur_mode->height));
		ret |= imx327_write_reg(imx327->spi,
			(IMX327_REG_VTS_L >> 8) & 0xff,
			IMX327_REG_VTS_L & 0xff,
			IMX327_FETCH_LOW_BYTE_VTS(ctrl->val +
			imx327->cur_mode->height));
		break;
	default:
		dev_warn(&spi->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&spi->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx327_ctrl_ops = {
	.s_ctrl = imx327_set_ctrl,
};

static int imx327_initialize_controls(struct imx327 *imx327)
{
	const struct imx327_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &imx327->ctrl_handler;
	mode = imx327->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &imx327->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, IMX327_PIXEL_RATE, 1, IMX327_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	imx327->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (imx327->hblank)
		imx327->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	imx327->vblank = v4l2_ctrl_new_std(handler, &imx327_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				IMX327_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	imx327->exposure = v4l2_ctrl_new_std(handler, &imx327_ctrl_ops,
				V4L2_CID_EXPOSURE, IMX327_EXPOSURE_MIN,
				exposure_max, IMX327_EXPOSURE_STEP,
				mode->exp_def);

	imx327->anal_gain = v4l2_ctrl_new_std(handler, &imx327_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, IMX327_GAIN_MIN,
				IMX327_GAIN_MAX, IMX327_GAIN_STEP,
				IMX327_GAIN_DEFAULT);

	if (handler->error) {
		ret = handler->error;
		dev_err(&imx327->spi->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	imx327->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int imx327_check_sensor_id(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	int ret = 0;
	u8 id = 0;

	ret = imx327_read_reg(spi, (IMX327_REG_CHIP_ID >> 8) & 0xff,
		IMX327_REG_CHIP_ID & 0xff, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%02x), ret(%d)\n", id, ret);
		return -ENODEV;
	}
	dev_info(dev, "Detected imx327 id:%02x\n", id);
	return 0;
}

static int imx327_configure_regulators(struct imx327 *imx327)
{
	unsigned int i;

	for (i = 0; i < IMX327_NUM_SUPPLIES; i++)
		imx327->supplies[i].supply = imx327_supply_names[i];

	return devm_regulator_bulk_get(&imx327->spi->dev,
				       IMX327_NUM_SUPPLIES,
				       imx327->supplies);
}

static int imx327_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct device_node *node = dev->of_node;
	struct imx327 *imx327;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	imx327 = devm_kzalloc(dev, sizeof(struct imx327), GFP_KERNEL);
	if (!imx327)
		return -ENOMEM;

	spi->mode = SPI_MODE_3 | SPI_LSB_FIRST;
	spi->irq = -1;
	spi->max_speed_hz = 400000;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(dev, "could not setup spi!\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &imx327->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &imx327->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &imx327->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &imx327->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	imx327->spi = spi;
	imx327->cur_mode = &supported_modes[0];

	imx327->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(imx327->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	imx327->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(imx327->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	imx327->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(imx327->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = imx327_configure_regulators(imx327);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	imx327->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(imx327->pinctrl)) {
		imx327->pins_default =
			pinctrl_lookup_state(imx327->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(imx327->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		imx327->pins_sleep =
			pinctrl_lookup_state(imx327->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(imx327->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}
	mutex_init(&imx327->mutex);

	sd = &imx327->subdev;
	v4l2_spi_subdev_init(sd, spi, &imx327_subdev_ops);
	ret = imx327_initialize_controls(imx327);
	if (ret)
		goto err_destroy_mutex;

	ret = __imx327_power_on(imx327);
	if (ret)
		goto err_free_handler;

	ret = imx327_check_sensor_id(spi);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &imx327_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif

#if defined(CONFIG_MEDIA_CONTROLLER)
	imx327->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &imx327->pad, 0);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(imx327->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 imx327->module_index, facing,
		 IMX327_NAME, dev_name(sd->dev));
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
	__imx327_power_off(imx327);
err_free_handler:
	v4l2_ctrl_handler_free(&imx327->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&imx327->mutex);

	return ret;
}

static int imx327_remove(struct spi_device *spi)
{
	struct v4l2_subdev *sd = spi_get_drvdata(spi);
	struct imx327 *imx327 = to_imx327(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&imx327->ctrl_handler);
	mutex_destroy(&imx327->mutex);

	pm_runtime_disable(&spi->dev);
	if (!pm_runtime_status_suspended(&spi->dev))
		__imx327_power_off(imx327);
	pm_runtime_set_suspended(&spi->dev);

	return 0;
}

static const struct spi_device_id imx327_match_id[] = {
	{ "sony,imx327", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, imx327_match_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id imx327_of_match[] = {
	{ .compatible = "sony,imx327" },
	{ },
};
MODULE_DEVICE_TABLE(of, imx327_of_match);
#endif

static struct spi_driver imx327_driver = {
	.driver = {
		.name = IMX327_NAME,
		.pm = &imx327_pm_ops,
		.of_match_table = of_match_ptr(imx327_of_match),
	},
	.probe		= &imx327_probe,
	.remove		= &imx327_remove,
	.id_table	= imx327_match_id,
};

static int __init imx327_mod_init(void)
{
	return spi_register_driver(&imx327_driver);
}

static void __exit imx327_mod_exit(void)
{
	spi_unregister_driver(&imx327_driver);
}

late_initcall(imx327_mod_init);
module_exit(imx327_mod_exit);

MODULE_DESCRIPTION("Sony imx327 sensor driver");
MODULE_LICENSE("GPL v2");
