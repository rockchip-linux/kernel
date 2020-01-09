// SPDX-License-Identifier: GPL-2.0
/*
 * ar0330 driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 * V0.0X01.0X01 add enum_frame_interval function.
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

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

/* 74.25Mhz */
#define AR0330_PIXEL_RATE		(196000000U)
#define AR0330_XVCLK_FREQ		24000000
#define AR0330_LINK_FREQ		268000000U

#define CHIP_ID				0x2604
#define AR0330_REG_CHIP_ID		0x3000

#define AR0330_REG_CTRL_MODE		0x301A
#define AR0330_MODE_SW_STANDBY		0x0058
#define AR0330_MODE_STREAMING		0x005C

#define AR0330_REG_EXPOSURE		0x3012
#define AR0330_EXPOSURE_MIN		4
#define AR0330_EXPOSURE_STEP		1
#define AR0330_VTS_MAX			0x0fff

#define AR0330_REG_ANALOG_GAIN		0x3060
#define ANALOG_GAIN_MIN			0x64
#define ANALOG_GAIN_MAX			0x320
#define ANALOG_GAIN_STEP		1
#define ANALOG_GAIN_DEFAULT		0xC8

#define AR0330_REG_VTS			0x300a

#define AR0330_REG_ORIENTATION		0x3040
#define AR0330_ORIENTATION_H		bit(14)
#define AR0330_ORIENTATION_V		bit(15)

#define REG_NULL			0xFFFF
#define REG_DELAY			0xFFFE

#define AR0330_REG_VALUE_08BIT		1
#define AR0330_REG_VALUE_16BIT		2
#define AR0330_REG_VALUE_24BIT		3

#define USE_HDR_MODE

/* h_offs 35 v_offs 14 */
#define PIX_FORMAT MEDIA_BUS_FMT_SGRBG12_1X12

#define AR0330_NAME			"ar0330"

static const char * const ar0330_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define AR0330_NUM_SUPPLIES ARRAY_SIZE(ar0330_supply_names)

struct regval {
	u16 addr;
	u16 val;
};

struct gain_range {
	u16 range_h;
	u16 val;
};

struct ar0330_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct ar0330 {
	struct i2c_client	*client;
	struct clk			*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[AR0330_NUM_SUPPLIES];

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
	bool				streaming;
	bool				power_on;
	const struct ar0330_mode *cur_mode;
	u32					module_index;
	const char			*module_facing;
	const char			*module_name;
	const char			*len_name;
};

#define to_ar0330(sd) container_of(sd, struct ar0330, subdev)

/*
 * Xclk 24Mhz
 * Pclk 98Mhz
 * linelength 0x9c0
 * framelength 0x51c
 * grabwindow_width 1920
 * grabwindow_height 1080
 * max_framerate 30fps
 * mipi rate 588Mbps 12it
 */

static const struct regval ar0330_global_regs[] = {
	//PLL_settings
	{0x301a, 0x0059}, //Reset Sensor
	{REG_DELAY, 0x0064},
	{0x31AE, 0x0202}, //MIPI 2lane
	{0x301a, 0x0058}, //Disable Streaming
	{REG_DELAY, 0x0032},
	{0x3064, 0x1802},
	{0x3078, 0x0001},
	{0x31e0, 0x0003},

	//Toggle Flash on Each Frame
	{0x3046, 0x4038}, // Enable Flash Pin
	{0x3048, 0x8480}, // Flash Pulse Length
	{0x31E0, 0x0203}, //OTPM V5
	{0x3ED2, 0x0146},
	{0x3EDA, 0x88BC},
	{0x3EDC, 0xAA63},
	{0x305E, 0x00A0},

	//PLL_settings 588Mbps 98Mhz
	//STATE = Master Clock,98000000
	{0x302A, 0x0006}, //VT_PIX_CLK_DIV = 6
	{0x302C, 0x0002}, //VT_SYS_CLK_DIV = 2
	{0x302E, 0x0002}, //PRE_PLL_CLK_DIV = 2
	{0x3030, 0x0031}, //PLL_MULTIPLIER = 49
	{0x3036, 0x000C}, //OP_PIX_CLK_DIV = 12
	{0x3038, 0x0001}, //OP_SYS_CLK_DIV = 1
	{0x31AC, 0x0C0C}, //DATA_FORMAT_BITS

	//MIPI Port Timing continuous mode
	{0x31B0, 0x002d},
	{0x31B2, 0x0012},
	{0x31B4, 0x3b44},
	{0x31B6, 0x314d},
	{0x31B8, 0x2089},
	{0x31BA, 0x0206},
	{0x31BC, 0x8005},
	{0x31BE, 0x2003},

	//Timing_settings
	{0x3004, 0x00C6}, //X_ADDR_START = 6
	{0x3008, 0x0845}, //X_ADDR_END = 2309
	{0x3002, 0x0084}, //Y_ADDR_START = 120
	{0x3006, 0x057B}, //Y_ADDR_END = 1415

	{0x300A, 0x051c}, //FRAME_LENGTH_LINES = 1308
	{0x300C, 0x04E0}, //LINE_LENGTH_PCK = 1248
	{0x3012, 0x051b}, //COARSE_INTEGRATION_TIME = 1307
	{0x3014, 0x0000}, //FINE_INTEGRATION_TIME = 0
	{0x30A2, 0x0001}, //X_ODD_INC = 1
	{0x30A6, 0x0001}, //Y_ODD_INC = 1

	{0x3040, 0x0000}, //READ_MODE = 0
	{0x3042, 0x0000}, //EXTRA_DELAY = 0
	{0x30BA, 0x002C}, //DIGITAL_CTRL = 44
	{0x3070, 0x0000},

	{0x30FE, 0x0080}, // RESERVED_MFR_30FE
	{0x31E0, 0x0703}, // RESERVED_MFR_31E0
	{0x3ECE, 0x08FF}, // RESERVED_MFR_3ECE
	{0x3ED0, 0xE4F6}, // RESERVED_MFR_3ED0
	{0x3ED2, 0x0146}, // RESERVED_MFR_3ED2
	{0x3ED4, 0x8F6C}, // RESERVED_MFR_3ED4
	{0x3ED6, 0x66CC}, // RESERVED_MFR_3ED6
	{0x3ED8, 0x8C42}, // RESERVED_MFR_3ED8
	{0x3EDA, 0x889B}, // RESERVED_MFR_3EDA
	{0x3EDC, 0x8863}, // RESERVED_MFR_3EDC
	{0x3EDE, 0xAA04}, // RESERVED_MFR_3EDE
	{0x3EE0, 0x15F0}, // RESERVED_MFR_3EE0
	{0x3EE6, 0x008C}, // RESERVED_MFR_3EE6
	{0x3EE8, 0x2024}, // RESERVED_MFR_3EE8
	{0x3EEA, 0xFF1F}, // RESERVED_MFR_3EEA
	{0x3F06, 0x046A}, // RESERVED_MFR_3F06
	{0x3EDA, 0x88BC}, // RESERVED_MFR_3EDA
	{0x3EDC, 0xAA63}, // RESERVED_MFR_3EDC
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * Pclk 98Mhz
 * linelength 0x9c0
 * framelength 0x51c
 * grabwindow_width 1920
 * grabwindow_height 1080
 * max_framerate 30fps
 * mipi_datarate per lane 588Mbps
 */
static const struct regval ar0330_2304x1296_regs[] = {
	{REG_NULL, 0x00}
};

#define HTS_DEF 2496
#define VTS_DEF 1308
#define MAX_FPS 30
static const struct ar0330_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x018c,
		.hts_def = HTS_DEF,
		.vts_def = VTS_DEF,
		.reg_list = ar0330_2304x1296_regs,
	}
};

static const s64 link_freq_menu_items[] = {
	AR0330_LINK_FREQ
};

static const char * const ar0330_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* sensor register write */
static int ar0330_write(struct i2c_client *client, u16 reg, u16 val)
{
	struct i2c_msg msg;
	u8 buf[4];
	int ret;

	dev_dbg(&client->dev, "write reg(0x%x val:0x%x)!\n", reg, val);

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = val >> 8 & 0xff;
	buf[3] = val & 0xff;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	dev_err(&client->dev,
		"ov5640 write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

static int ar0330_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	int i, delay_ms, ret = 0;

	i = 0;
	while (regs[i].addr != REG_NULL) {
		if (regs[i].addr == REG_DELAY) {
			delay_ms = regs[i].val;
			dev_info(&client->dev, "delay(%d) ms !\n", delay_ms);
			usleep_range(1000 * delay_ms, 1000 * delay_ms + 100);
			i++;
			continue;
		}
		ret = ar0330_write(client, regs[i].addr, regs[i].val);
		if (ret) {
			dev_err(&client->dev, "%s failed !\n", __func__);
			break;
		}
		i++;
	}

	return ret;
}

/* Read registers up to 4 at a time */
/* sensor register read */
static int ar0330_read_reg(struct i2c_client *client, u16 reg, u16 *val)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = (reg >> 8) & 0xff;
	buf[1] = reg & 0xFF;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret >= 0) {
		*val = (buf[0] << 8) | buf[1];
		return 0;
	}

	dev_err(&client->dev,
		"ar0330 read reg:0x%x failed !\n", reg);

	return ret;
}

static int ar0330_get_reso_dist(const struct ar0330_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct ar0330_mode *
ar0330_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	u32 i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = ar0330_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int ar0330_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ar0330 *ar0330 = to_ar0330(sd);
	const struct ar0330_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&ar0330->mutex);

	mode = ar0330_find_best_fit(fmt);
	fmt->format.code = PIX_FORMAT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&ar0330->mutex);
		return -ENOTTY;
#endif
	} else {
		ar0330->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(ar0330->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ar0330->vblank, vblank_def,
					 AR0330_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&ar0330->mutex);

	return 0;
}

static int ar0330_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ar0330 *ar0330 = to_ar0330(sd);
	const struct ar0330_mode *mode = ar0330->cur_mode;

	mutex_lock(&ar0330->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&ar0330->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = PIX_FORMAT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&ar0330->mutex);

	return 0;
}

static int ar0330_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = PIX_FORMAT;

	return 0;
}

static int ar0330_enum_frame_sizes(struct v4l2_subdev *sd,
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

static int ar0330_enable_test_pattern(struct ar0330 *ar0330, u32 pattern)
{
	return 0;
}

static void ar0330_get_module_inf(struct ar0330 *ar0330,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, AR0330_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, ar0330->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, ar0330->len_name, sizeof(inf->base.lens));
}

static long ar0330_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ar0330 *ar0330 = to_ar0330(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		ar0330_get_module_inf(ar0330, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ar0330_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = ar0330_ioctl(sd, cmd, inf);
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

static int __ar0330_start_stream(struct ar0330 *ar0330)
{
	int ret;

	ret = ar0330_write_array(ar0330->client, ar0330->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&ar0330->mutex);
	ret = v4l2_ctrl_handler_setup(&ar0330->ctrl_handler);
	mutex_lock(&ar0330->mutex);
	if (ret)
		return ret;

	return ar0330_write(ar0330->client, AR0330_REG_CTRL_MODE,
						AR0330_MODE_STREAMING);
}

static int __ar0330_stop_stream(struct ar0330 *ar0330)
{
	return ar0330_write(ar0330->client, AR0330_REG_CTRL_MODE,
						AR0330_MODE_SW_STANDBY);
}

static int ar0330_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ar0330 *ar0330 = to_ar0330(sd);
	struct i2c_client *client = ar0330->client;
	int ret = 0;

	mutex_lock(&ar0330->mutex);
	on = !!on;
	if (on == ar0330->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __ar0330_start_stream(ar0330);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__ar0330_stop_stream(ar0330);
		pm_runtime_put(&client->dev);
	}

	ar0330->streaming = on;
unlock_and_return:
	mutex_unlock(&ar0330->mutex);

	return ret;
}

static int ar0330_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ar0330 *ar0330 = to_ar0330(sd);
	const struct ar0330_mode *mode = ar0330->cur_mode;

	mutex_lock(&ar0330->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&ar0330->mutex);

	return 0;
}

static int ar0330_s_power(struct v4l2_subdev *sd, int on)
{
	struct ar0330 *ar0330 = to_ar0330(sd);
	struct i2c_client *client = ar0330->client;
	int ret = 0;

	mutex_lock(&ar0330->mutex);

	/* If the power state is not modified - no work to do. */
	if (ar0330->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = ar0330_write_array(ar0330->client, ar0330_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		ar0330->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		ar0330->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&ar0330->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 ar0330_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, AR0330_XVCLK_FREQ / 1000 / 1000);
}

static int __ar0330_power_on(struct ar0330 *ar0330)
{
	int ret;
	u32 delay_us;
	struct device *dev = &ar0330->client->dev;

	dev_info(dev, "%s(%d) enter\n", __func__, __LINE__);
	ret = clk_set_rate(ar0330->xvclk, AR0330_XVCLK_FREQ);
	if (ret < 0) {
		dev_err(dev, "Failed to set xvclk rate (%d)\n",
			AR0330_XVCLK_FREQ);
		return ret;
	}
	if (clk_get_rate(ar0330->xvclk) != AR0330_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on %d\n",
			AR0330_XVCLK_FREQ);

	ret = clk_prepare_enable(ar0330->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(ar0330->reset_gpio))
		gpiod_set_value_cansleep(ar0330->reset_gpio, 0);

	ret = regulator_bulk_enable(AR0330_NUM_SUPPLIES, ar0330->supplies);
	usleep_range(20000, 50000);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(ar0330->reset_gpio))
		gpiod_set_value_cansleep(ar0330->reset_gpio, 1);

	if (!IS_ERR(ar0330->pwdn_gpio))
		gpiod_set_value_cansleep(ar0330->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = ar0330_cal_delay(92000);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(ar0330->xvclk);

	return ret;
}

static void __ar0330_power_off(struct ar0330 *ar0330)
{
	if (!IS_ERR(ar0330->pwdn_gpio))
		gpiod_set_value_cansleep(ar0330->pwdn_gpio, 0);
	clk_disable_unprepare(ar0330->xvclk);
	if (!IS_ERR(ar0330->reset_gpio))
		gpiod_set_value_cansleep(ar0330->reset_gpio, 1);
	regulator_bulk_disable(AR0330_NUM_SUPPLIES, ar0330->supplies);
}

static int ar0330_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0330 *ar0330 = to_ar0330(sd);

	return __ar0330_power_on(ar0330);
}

static int ar0330_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0330 *ar0330 = to_ar0330(sd);

	__ar0330_power_off(ar0330);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ar0330_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ar0330 *ar0330 = to_ar0330(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct ar0330_mode *def_mode = &supported_modes[0];

	mutex_lock(&ar0330->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = PIX_FORMAT;
	try_fmt->field = V4L2_FIELD_NONE;
	mutex_unlock(&ar0330->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int ar0330_enum_frame_interval(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fie->code != PIX_FORMAT)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static const struct dev_pm_ops ar0330_pm_ops = {
	SET_RUNTIME_PM_OPS(ar0330_runtime_suspend,
			   ar0330_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops ar0330_internal_ops = {
	.open = ar0330_open,
};
#endif

static const struct v4l2_subdev_core_ops ar0330_core_ops = {
	.s_power = ar0330_s_power,
	.ioctl = ar0330_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ar0330_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops ar0330_video_ops = {
	.s_stream = ar0330_s_stream,
	.g_frame_interval = ar0330_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ar0330_pad_ops = {
	.enum_mbus_code = ar0330_enum_mbus_code,
	.enum_frame_size = ar0330_enum_frame_sizes,
	.enum_frame_interval = ar0330_enum_frame_interval,
	.get_fmt = ar0330_get_fmt,
	.set_fmt = ar0330_set_fmt,
};

static const struct v4l2_subdev_ops ar0330_subdev_ops = {
	.core	= &ar0330_core_ops,
	.video	= &ar0330_video_ops,
	.pad	= &ar0330_pad_ops,
};

struct gain_range gain_level[] = {
	{101, 0x00}, {104, 0x01}, {108, 0x02}, {111, 0x03}, {115, 0x04},
	{120, 0x05}, {124, 0x06}, {129, 0x07}, {134, 0x08}, {140, 0x09},
	{146, 0x0a}, {153, 0x0b}, {161, 0x0c}, {169, 0x0d}, {179, 0x0e},
	{189, 0x0f}, {201, 0x10}, {214, 0x12}, {230, 0x14}, {247, 0x16},
	{268, 0x18}, {292, 0x1a}, {321, 0x1c}, {357, 0x1e}, {401, 0x20},
	{457, 0x24}, {533, 0x28}, {641, 0x2c}, {801, 0x30}
};

static int ar0330_set_gain(struct ar0330 *ar0330, int gain)
{
	int ret = 0, i = 0;
	int num_gains = ARRAY_SIZE(gain_level);
	u16 again = 0;

	for (i = 0; i < num_gains; i++) {
		if (gain_level[i].range_h >= gain) {
			again = gain_level[i].val;
			break;
		}
	}

	ret = ar0330_write(ar0330->client, AR0330_REG_ANALOG_GAIN, again);
	return ret;
}

static int ar0330_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0330 *ar0330 = container_of(ctrl->handler,
					     struct ar0330, ctrl_handler);
	struct i2c_client *client = ar0330->client;
	int ret = 0;

	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = ar0330_write(ar0330->client,
						AR0330_REG_EXPOSURE, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ar0330_set_gain(ar0330, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = ar0330_write(ar0330->client, AR0330_REG_VTS,
				       ctrl->val + ar0330->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ar0330_enable_test_pattern(ar0330, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ar0330_ctrl_ops = {
	.s_ctrl = ar0330_set_ctrl,
};

static int ar0330_initialize_controls(struct ar0330 *ar0330)
{
	const struct ar0330_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &ar0330->ctrl_handler;
	mode = ar0330->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 7);
	if (ret)
		return ret;
	handler->lock = &ar0330->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, AR0330_PIXEL_RATE, 1, AR0330_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	ar0330->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (ar0330->hblank)
		ar0330->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	ar0330->vblank = v4l2_ctrl_new_std(handler, &ar0330_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				AR0330_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	ar0330->exposure = v4l2_ctrl_new_std(handler, &ar0330_ctrl_ops,
				V4L2_CID_EXPOSURE, AR0330_EXPOSURE_MIN,
				exposure_max, AR0330_EXPOSURE_STEP,
				mode->exp_def);

	ar0330->anal_gain = v4l2_ctrl_new_std(handler, &ar0330_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, ANALOG_GAIN_MIN,
				ANALOG_GAIN_MAX, ANALOG_GAIN_STEP,
				ANALOG_GAIN_DEFAULT);

	ar0330->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&ar0330_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(ar0330_test_pattern_menu) - 1,
				0, 0, ar0330_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&ar0330->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	ar0330->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ar0330_check_sensor_id(struct ar0330 *ar0330,
				  struct i2c_client *client)
{
	struct device *dev = &ar0330->client->dev;
	u16 id = 0;
	int ret;

	ret = ar0330_read_reg(client, AR0330_REG_CHIP_ID, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected AR0330 sensor\n");

	return 0;
}

static int ar0330_configure_regulators(struct ar0330 *ar0330)
{
	u32 i;

	for (i = 0; i < AR0330_NUM_SUPPLIES; i++)
		ar0330->supplies[i].supply = ar0330_supply_names[i];

	return devm_regulator_bulk_get(&ar0330->client->dev,
				       AR0330_NUM_SUPPLIES,
				       ar0330->supplies);
}

static int ar0330_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct ar0330 *ar0330;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "wpzz driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	ar0330 = devm_kzalloc(dev, sizeof(*ar0330), GFP_KERNEL);
	if (!ar0330)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &ar0330->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &ar0330->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &ar0330->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &ar0330->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ar0330->client = client;
	ar0330->cur_mode = &supported_modes[0];

	ar0330->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(ar0330->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	ar0330->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ar0330->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	ar0330->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(ar0330->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = ar0330_configure_regulators(ar0330);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&ar0330->mutex);

	sd = &ar0330->subdev;
	v4l2_i2c_subdev_init(sd, client, &ar0330_subdev_ops);
	ret = ar0330_initialize_controls(ar0330);
	if (ret)
		goto err_destroy_mutex;

	ret = __ar0330_power_on(ar0330);
	if (ret)
		goto err_free_handler;

	usleep_range(50000, 100000);
	ret = ar0330_check_sensor_id(ar0330, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &ar0330_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	ar0330->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &ar0330->pad, 0);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(ar0330->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 ar0330->module_index, facing,
		 AR0330_NAME, dev_name(sd->dev));
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
	__ar0330_power_off(ar0330);
err_free_handler:
	v4l2_ctrl_handler_free(&ar0330->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ar0330->mutex);

	return ret;
}

static int ar0330_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0330 *ar0330 = to_ar0330(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&ar0330->ctrl_handler);
	mutex_destroy(&ar0330->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__ar0330_power_off(ar0330);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ar0330_of_match[] = {
	{ .compatible = "Aptina,ar0330" },
	{},
};
MODULE_DEVICE_TABLE(of, ar0330_of_match);
#endif

static const struct i2c_device_id ar0330_match_id[] = {
	{ "Aptina,ar0330", 0 },
	{ },
};

static struct i2c_driver ar0330_i2c_driver = {
	.driver = {
		.name = AR0330_NAME,
		.pm = &ar0330_pm_ops,
		.of_match_table = of_match_ptr(ar0330_of_match),
	},
	.probe		= &ar0330_probe,
	.remove		= &ar0330_remove,
	.id_table	= ar0330_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&ar0330_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&ar0330_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Aptina ar0330 sensor driver");
MODULE_LICENSE("GPL v2");
