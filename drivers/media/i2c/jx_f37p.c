// SPDX-License-Identifier: GPL-2.0
/*
 * jx_f37pp driver
 *
 * Copyright (C) 2021 Rockchip Electronics Co., Ltd.
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
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/pinctrl/consumer.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define JX_F37P_PIXEL_RATE			(36000000)
#define JX_F37P_XVCLK_FREQ		24000000

#define CHIP_ID_H			0x08
#define CHIP_ID_L			0x41

#define JX_F37P_PIDH_ADDR     0x0a
#define JX_F37P_PIDL_ADDR     0x0b

#define JX_F37P_REG_CTRL_MODE		0x12
#define JX_F37P_MODE_SW_STANDBY		0x40
#define JX_F37P_MODE_STREAMING		0x00

#define JX_F37P_AEC_PK_LONG_EXPO_HIGH_REG 0x02	/* Exposure Bits 8-15 */
#define JX_F37P_AEC_PK_LONG_EXPO_LOW_REG 0x01	/* Exposure Bits 0-7 */
#define JX_F37P_FETCH_HIGH_BYTE_EXP(VAL) (((VAL) >> 8) & 0xFF)	/* 8-15 Bits */
#define JX_F37P_FETCH_LOW_BYTE_EXP(VAL) ((VAL) & 0xFF)	/* 0-7 Bits */
#define	JX_F37P_EXPOSURE_MIN		4
#define	JX_F37P_EXPOSURE_STEP		1
#define JX_F37P_VTS_MAX			0xffff

#define JX_F37P_AEC_PK_LONG_GAIN_REG	0x00	/* Bits 0 -7 */
#define	ANALOG_GAIN_MIN			0x00
#define	ANALOG_GAIN_MAX			0xf8	/* 15.5 */
#define	ANALOG_GAIN_STEP		1
#define	ANALOG_GAIN_DEFAULT		0x10

#define JX_F37P_DIGI_GAIN_L_MASK		0x3f
#define JX_F37P_DIGI_GAIN_H_SHIFT	6
#define JX_F37P_DIGI_GAIN_MIN		0
#define JX_F37P_DIGI_GAIN_MAX		(0x4000 - 1)
#define JX_F37P_DIGI_GAIN_STEP		1
#define JX_F37P_DIGI_GAIN_DEFAULT	1024

#define JX_F37P_REG_TEST_PATTERN		0x0c
#define	JX_F37P_TEST_PATTERN_ENABLE	0x80
#define	JX_F37P_TEST_PATTERN_DISABLE	0x0

#define JX_F37P_REG_HIGH_VTS			0x23
#define JX_F37P_REG_LOW_VTS			0X22
#define JX_F37P_FETCH_HIGH_BYTE_VTS(VAL) (((VAL) >> 8) & 0xFF)	/* 8-15 Bits */
#define JX_F37P_FETCH_LOW_BYTE_VTS(VAL) ((VAL) & 0xFF)	/* 0-7 Bits */

#define REG_NULL			0xFF
#define REG_DELAY			0xFE

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define JX_F37P_NAME			"jx_f37p"
#define JX_F37P_MEDIA_BUS_FMT		MEDIA_BUS_FMT_SBGGR10_1X10

static const char * const jx_f37p_supply_names[] = {
	"vcc2v8_dvp",		/* Analog power */
	"vcc1v8_dvp",		/* Digital I/O power */
};

#define JX_F37P_NUM_SUPPLIES ARRAY_SIZE(jx_f37p_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct jx_f37p_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
};

struct jx_f37p {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[JX_F37P_NUM_SUPPLIES];
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
	const struct jx_f37p_mode *cur_mode;
	unsigned int	lane_num;
	unsigned int	cfg_num;
	unsigned int	pixel_rate;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u32		old_gain;
};

#define to_jx_f37p(sd) container_of(sd, struct jx_f37p, subdev)

/*
 * Xclk 24Mhz
 * Pclk 36Mhz
 * linelength 750(0x2ee)
 * framelength 1920(0x780)
 * grabwindow_width 1280
 * grabwindow_height 720
 * max_framerate 25fps
 */
static const struct regval jx_f37p_1080p_dvp_linear_1lane_30fps[] = {
	{0x12, 0x40},
	{0x48, 0x85},
	{0x48, 0x05},
	{0x0E, 0x1D},
	{0x0F, 0x04},
	{0x10, 0x48},
	{0x11, 0x80},
	{0x46, 0x01},
	{0x47, 0x62},
	{0x0D, 0xAE},
	{0x57, 0x6A},
	{0x58, 0x22},
	{0x5F, 0x41},
	{0x60, 0x28},
	{0xA5, 0xC0},
	{0x20, 0x00},
	{0x21, 0x05},
	{0x22, 0x65},
	{0x23, 0x04},
	{0x24, 0xC0},
	{0x25, 0x38},
	{0x26, 0x43},
	{0x27, 0xC6},
	{0x28, 0x15},
	{0x29, 0x04},
	{0x2A, 0xBB},
	{0x2B, 0x14},
	{0x2C, 0x02},
	{0x2D, 0x00},
	{0x2E, 0x14},
	{0x2F, 0x04},
	{0x41, 0xC5},
	{0x42, 0x33},
	{0x47, 0x42},
	{0x76, 0x60},
	{0x77, 0x09},
	{0x80, 0x01},
	{0xAF, 0x22},
	{0xAB, 0x00},
	{0x1D, 0xFF},
	{0x1E, 0x9F},
	{0x6C, 0xC0},
	{0x9E, 0xF8},
	{0x31, 0x10},
	{0x32, 0x18},
	{0x33, 0xE8},
	{0x34, 0x5E},
	{0x35, 0x5E},
	{0x3A, 0xAF},
	{0x3B, 0x00},
	{0x3C, 0xFF},
	{0x3D, 0xFF},
	{0x3E, 0xFF},
	{0x3F, 0xBB},
	{0x40, 0xFF},
	{0x56, 0x92},
	{0x59, 0xAF},
	{0x5A, 0x47},
	{0x61, 0x18},
	{0x6F, 0x04},
	{0x85, 0x5F},
	{0x8A, 0x44},
	{0x91, 0x13},
	{0x94, 0xA0},
	{0x9B, 0x83},
	{0x9C, 0xE1},
	{0xA4, 0x80},
	{0xA6, 0x22},
	{0xA9, 0x1C},
	{0x5B, 0xE7},
	{0x5C, 0x28},
	{0x5D, 0x67},
	{0x5E, 0x11},
	{0x62, 0x21},
	{0x63, 0x0F},
	{0x64, 0xD0},
	{0x65, 0x02},
	{0x67, 0x49},
	{0x66, 0x00},
	{0x68, 0x04},
	{0x69, 0x72},
	{0x6A, 0x12},
	{0x7A, 0x00},
	{0x82, 0x20},
	{0x8D, 0x47},
	{0x8F, 0x90},
	{0x45, 0x01},
	{0x97, 0x20},
	{0x13, 0x81},
	{0x96, 0x84},
	{0x4A, 0x01},
	{0xB1, 0x00},
	{0xA1, 0x0F},
	{0xBE, 0x00},
	{0x7E, 0x48},
	{0xB5, 0xC0},
	{0x50, 0x02},
	{0x49, 0x10},
	{0x7F, 0x57},
	{0x90, 0x00},
	{0x7B, 0x4A},
	{0x7C, 0x0C},
	{0x8C, 0xFF},
	{0x8E, 0x00},
	{0x8B, 0x01},
	{0x0C, 0x00},
	{0xBC, 0x11},
	{0x19, 0x20},
	{0x1B, 0x4F},
	{0x12, 0x00},
	{0x00, 0x10},
	{REG_NULL, 0x00},
};

static const struct jx_f37p_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x00ff,
		.hts_def = 0x0500,
		.vts_def = 0x0465,
		.reg_list = jx_f37p_1080p_dvp_linear_1lane_30fps,
		.hdr_mode = NO_HDR,
	}
};

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 jx_f37p_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, JX_F37P_XVCLK_FREQ / 1000 / 1000);
}

static int jx_f37p_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	buf[0] = reg & 0xFF;
	buf[1] = val;

	msg.addr =  client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		return 0;

	dev_err(&client->dev,
		"jx_f37p write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

static int jx_f37p_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i, delay_us;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		if (regs[i].addr == REG_DELAY) {
			delay_us = jx_f37p_cal_delay(500 * 1000);
			usleep_range(delay_us, delay_us * 2);
		} else {
			ret = jx_f37p_write_reg(client,
						regs[i].addr, regs[i].val);
		}
	}

	return ret;
}

static int jx_f37p_read_reg(struct i2c_client *client, u8 reg, u8 *val)
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
		"jx_f37p read reg:0x%x failed !\n", reg);

	return ret;
}

static int jx_f37p_get_reso_dist(const struct jx_f37p_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct jx_f37p_mode *
jx_f37p_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = jx_f37p_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int jx_f37p_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct jx_f37p *jx_f37p = to_jx_f37p(sd);
	const struct jx_f37p_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&jx_f37p->mutex);

	mode = jx_f37p_find_best_fit(fmt);
	fmt->format.code = JX_F37P_MEDIA_BUS_FMT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&jx_f37p->mutex);
		return -ENOTTY;
#endif
	} else {
		jx_f37p->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(jx_f37p->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(jx_f37p->vblank, vblank_def,
					 JX_F37P_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&jx_f37p->mutex);

	return 0;
}

static int jx_f37p_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct jx_f37p *jx_f37p = to_jx_f37p(sd);
	const struct jx_f37p_mode *mode = jx_f37p->cur_mode;

	mutex_lock(&jx_f37p->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&jx_f37p->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = JX_F37P_MEDIA_BUS_FMT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&jx_f37p->mutex);

	return 0;
}

static int jx_f37p_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = JX_F37P_MEDIA_BUS_FMT;

	return 0;
}

static int jx_f37p_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != JX_F37P_MEDIA_BUS_FMT)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int jx_f37p_enable_test_pattern(struct jx_f37p *jx_f37p, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | JX_F37P_TEST_PATTERN_ENABLE;
	else
		val = JX_F37P_TEST_PATTERN_DISABLE;

	return jx_f37p_write_reg(jx_f37p->client, JX_F37P_REG_TEST_PATTERN, val);
}

static void jx_f37p_get_module_inf(struct jx_f37p *jx_f37p,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, JX_F37P_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, jx_f37p->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, jx_f37p->len_name, sizeof(inf->base.lens));
}

static long jx_f37p_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct jx_f37p *jx_f37p = to_jx_f37p(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = jx_f37p->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		if (hdr_cfg->hdr_mode != 0)
			ret = -1;
		break;
	case RKMODULE_GET_MODULE_INFO:
		jx_f37p_get_module_inf(jx_f37p, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = jx_f37p_write_reg(jx_f37p->client,
						JX_F37P_REG_CTRL_MODE,
						JX_F37P_MODE_STREAMING);
		else
			ret = jx_f37p_write_reg(jx_f37p->client,
						JX_F37P_REG_CTRL_MODE,
						JX_F37P_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long jx_f37p_compat_ioctl32(struct v4l2_subdev *sd,
				   unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_hdr_cfg *hdr;
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = jx_f37p_ioctl(sd, cmd, inf);
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

		ret = jx_f37p_ioctl(sd, cmd, hdr);
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

		jx_f37p_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;

	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;

		jx_f37p_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int jx_f37p_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct jx_f37p *jx_f37p = to_jx_f37p(sd);
	const struct jx_f37p_mode *mode = jx_f37p->cur_mode;

	mutex_lock(&jx_f37p->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&jx_f37p->mutex);

	return 0;
}

static int __jx_f37p_start_stream(struct jx_f37p *jx_f37p)
{
	return jx_f37p_write_reg(jx_f37p->client, JX_F37P_REG_CTRL_MODE,
				 JX_F37P_MODE_STREAMING);
}

static int __jx_f37p_stop_stream(struct jx_f37p *jx_f37p)
{
	return jx_f37p_write_reg(jx_f37p->client, JX_F37P_REG_CTRL_MODE,
				 JX_F37P_MODE_SW_STANDBY);
}

static int jx_f37p_s_stream(struct v4l2_subdev *sd, int on)
{
	struct jx_f37p *jx_f37p = to_jx_f37p(sd);
	struct i2c_client *client = jx_f37p->client;
	int ret = 0;

	mutex_lock(&jx_f37p->mutex);
	on = !!on;
	if (on == jx_f37p->streaming)
		goto unlock_and_return;

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
		 jx_f37p->cur_mode->width,
		 jx_f37p->cur_mode->height,
		 DIV_ROUND_CLOSEST(jx_f37p->cur_mode->max_fps.denominator,
		 jx_f37p->cur_mode->max_fps.numerator));

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __jx_f37p_start_stream(jx_f37p);
		if (ret) {
			v4l2_err(sd, " jx_f37p start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__jx_f37p_stop_stream(jx_f37p);
		pm_runtime_put(&client->dev);
	}

	jx_f37p->streaming = on;

unlock_and_return:
	mutex_unlock(&jx_f37p->mutex);

	return ret;
}

static int jx_f37p_s_power(struct v4l2_subdev *sd, int on)
{
	struct jx_f37p *jx_f37p = to_jx_f37p(sd);
	struct i2c_client *client = jx_f37p->client;
	int ret = 0;

	mutex_lock(&jx_f37p->mutex);

	/* If the power state is not modified - no work to do. */
	if (jx_f37p->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = jx_f37p_write_array(jx_f37p->client,
					  jx_f37p->cur_mode->reg_list);
		if (ret)
			goto unlock_and_return;

		/*
		 * Enter sleep state to make sure not mipi output
		 * during rkisp init.
		 */
		__jx_f37p_stop_stream(jx_f37p);

		mutex_unlock(&jx_f37p->mutex);
		/* In case these controls are set before streaming */
		ret = v4l2_ctrl_handler_setup(&jx_f37p->ctrl_handler);
		if (ret)
			return ret;
		mutex_lock(&jx_f37p->mutex);

		jx_f37p->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		jx_f37p->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&jx_f37p->mutex);

	return ret;
}


static int __jx_f37p_power_on(struct jx_f37p *jx_f37p)
{
	int ret;
	u32 delay_us;
	struct device *dev = &jx_f37p->client->dev;

	ret = clk_set_rate(jx_f37p->xvclk, JX_F37P_XVCLK_FREQ);
	if (ret < 0) {
		dev_err(dev, "Failed to set xvclk rate (24MHz)\n");
		return ret;
	}
	if (clk_get_rate(jx_f37p->xvclk) != JX_F37P_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(jx_f37p->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(jx_f37p->reset_gpio))
		gpiod_set_value_cansleep(jx_f37p->reset_gpio, 1);

	ret = regulator_bulk_enable(JX_F37P_NUM_SUPPLIES, jx_f37p->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	/* According to datasheet, at least 10ms for reset duration */
	usleep_range(10 * 1000, 15 * 1000);

	if (!IS_ERR(jx_f37p->reset_gpio))
		gpiod_set_value_cansleep(jx_f37p->reset_gpio, 0);

	if (!IS_ERR(jx_f37p->pwdn_gpio))
		gpiod_set_value_cansleep(jx_f37p->pwdn_gpio, 0);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = jx_f37p_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(jx_f37p->xvclk);

	return ret;
}

static void __jx_f37p_power_off(struct jx_f37p *jx_f37p)
{
	if (!IS_ERR(jx_f37p->pwdn_gpio))
		gpiod_set_value_cansleep(jx_f37p->pwdn_gpio, 1);
	clk_disable_unprepare(jx_f37p->xvclk);
	if (!IS_ERR(jx_f37p->reset_gpio))
		gpiod_set_value_cansleep(jx_f37p->reset_gpio, 1);
	regulator_bulk_disable(JX_F37P_NUM_SUPPLIES, jx_f37p->supplies);
}

static int jx_f37p_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct jx_f37p *jx_f37p = to_jx_f37p(sd);

	return __jx_f37p_power_on(jx_f37p);
}

static int jx_f37p_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct jx_f37p *jx_f37p = to_jx_f37p(sd);

	__jx_f37p_power_off(jx_f37p);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int jx_f37p_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct jx_f37p *jx_f37p = to_jx_f37p(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct jx_f37p_mode *def_mode = &supported_modes[0];

	mutex_lock(&jx_f37p->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = JX_F37P_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&jx_f37p->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int jx_f37p_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *config)
{
	config->type = V4L2_MBUS_PARALLEL;
	config->flags = V4L2_MBUS_HSYNC_ACTIVE_HIGH |
			V4L2_MBUS_VSYNC_ACTIVE_LOW |
			V4L2_MBUS_PCLK_SAMPLE_RISING;
	return 0;
}
static int jx_f37p_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fie->code != JX_F37P_MEDIA_BUS_FMT)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static const struct dev_pm_ops jx_f37p_pm_ops = {
	SET_RUNTIME_PM_OPS(jx_f37p_runtime_suspend,
			   jx_f37p_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops jx_f37p_internal_ops = {
	.open = jx_f37p_open,
};
#endif

static const struct v4l2_subdev_core_ops jx_f37p_core_ops = {
	.s_power = jx_f37p_s_power,
	.ioctl = jx_f37p_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = jx_f37p_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops jx_f37p_video_ops = {
	.s_stream = jx_f37p_s_stream,
	.g_mbus_config = jx_f37p_g_mbus_config,
	.g_frame_interval = jx_f37p_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops jx_f37p_pad_ops = {
	.enum_mbus_code = jx_f37p_enum_mbus_code,
	.enum_frame_size = jx_f37p_enum_frame_sizes,
	.enum_frame_interval = jx_f37p_enum_frame_interval,
	.get_fmt = jx_f37p_get_fmt,
	.set_fmt = jx_f37p_set_fmt,
};

static const struct v4l2_subdev_ops jx_f37p_subdev_ops = {
	.core	= &jx_f37p_core_ops,
	.video	= &jx_f37p_video_ops,
	.pad	= &jx_f37p_pad_ops,
};

static int jx_f37p_set_ctrl_gain(struct jx_f37p *jx_f37p, u32 a_gain)
{
	int ret = 0;
	u32 coarse_again, fine_again;

	/* Total gain = 2^PGA[5:4]*(1+PGA[3:0]/16) */
	if (a_gain != jx_f37p->old_gain) {
		if (a_gain < 0x10) { /*1x ~ 2x*/
			fine_again = a_gain;
			coarse_again = 0x00 << 4;
		} else if (a_gain < 0x40) { /*2x ~ 4x*/
			fine_again = (a_gain >> 1) - 16;
			coarse_again = 0x01 << 4;
		} else if (a_gain < 0x80) { /*4x ~ 8x*/
			fine_again = (a_gain >> 2) - 16;
			coarse_again = 0x2;
		} else { /*8x ~ 15.5x*/
			fine_again = (a_gain >> 3) - 16;
			coarse_again = 0x03 << 4;
		}
		ret = jx_f37p_write_reg(jx_f37p->client,
			JX_F37P_AEC_PK_LONG_GAIN_REG, coarse_again | fine_again);
		jx_f37p->old_gain = a_gain;
	}
	return ret;
}

static int jx_f37p_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct jx_f37p *jx_f37p = container_of(ctrl->handler,
					       struct jx_f37p, ctrl_handler);
	struct i2c_client *client = jx_f37p->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = jx_f37p->cur_mode->height + ctrl->val;
		__v4l2_ctrl_modify_range(jx_f37p->exposure,
					 jx_f37p->exposure->minimum, max,
					 jx_f37p->exposure->step,
					 jx_f37p->exposure->default_value);
		break;
	}

	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set expo: val: %d\n", ctrl->val);
		/* 4 least significant bits of expsoure are fractional part */
		ret = jx_f37p_write_reg(jx_f37p->client,
				JX_F37P_AEC_PK_LONG_EXPO_HIGH_REG,
				JX_F37P_FETCH_HIGH_BYTE_EXP(ctrl->val));
		ret |= jx_f37p_write_reg(jx_f37p->client,
				JX_F37P_AEC_PK_LONG_EXPO_LOW_REG,
				JX_F37P_FETCH_LOW_BYTE_EXP(ctrl->val));
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set a-gain: val: %d\n", ctrl->val);
		ret = jx_f37p_set_ctrl_gain(jx_f37p, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vblank: val: %d\n", ctrl->val);
		ret |= jx_f37p_write_reg(jx_f37p->client, JX_F37P_REG_HIGH_VTS,
					 JX_F37P_FETCH_HIGH_BYTE_VTS((ctrl->val
					 + jx_f37p->cur_mode->height)));
		ret |= jx_f37p_write_reg(jx_f37p->client, JX_F37P_REG_LOW_VTS,
					 JX_F37P_FETCH_LOW_BYTE_VTS((ctrl->val
					 + jx_f37p->cur_mode->height)));
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = jx_f37p_enable_test_pattern(jx_f37p, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops jx_f37p_ctrl_ops = {
	.s_ctrl = jx_f37p_set_ctrl,
};

static int jx_f37p_initialize_controls(struct jx_f37p *jx_f37p)
{
	const struct jx_f37p_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &jx_f37p->ctrl_handler;
	mode = jx_f37p->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);

	if (ret)
		return ret;
	handler->lock = &jx_f37p->mutex;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, JX_F37P_PIXEL_RATE, 1, JX_F37P_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	jx_f37p->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);

	if (jx_f37p->hblank)
		jx_f37p->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	jx_f37p->vblank = v4l2_ctrl_new_std(handler, &jx_f37p_ctrl_ops,
					    V4L2_CID_VBLANK, vblank_def,
					    JX_F37P_VTS_MAX - mode->height,
					    1, vblank_def);

	exposure_max = mode->vts_def;
	//exposure_max = mode->vts_def - 4;
	jx_f37p->exposure = v4l2_ctrl_new_std(handler, &jx_f37p_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      JX_F37P_EXPOSURE_MIN,
					      exposure_max,
					      JX_F37P_EXPOSURE_STEP,
					      mode->exp_def);

	jx_f37p->anal_gain = v4l2_ctrl_new_std(handler, &jx_f37p_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN,
					       ANALOG_GAIN_MIN,
					       ANALOG_GAIN_MAX,
					       ANALOG_GAIN_STEP,
					       ANALOG_GAIN_DEFAULT);

	/* Digital gain */
	jx_f37p->digi_gain = v4l2_ctrl_new_std(handler, &jx_f37p_ctrl_ops,
					       V4L2_CID_DIGITAL_GAIN,
					       JX_F37P_DIGI_GAIN_MIN,
					       JX_F37P_DIGI_GAIN_MAX,
					       JX_F37P_DIGI_GAIN_STEP,
					       JX_F37P_DIGI_GAIN_DEFAULT);

	if (handler->error) {
		ret = handler->error;
		dev_err(&jx_f37p->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	jx_f37p->subdev.ctrl_handler = handler;
	jx_f37p->old_gain = ANALOG_GAIN_DEFAULT;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int jx_f37p_check_sensor_id(struct jx_f37p *jx_f37p,
				   struct i2c_client *client)
{
	struct device *dev = &jx_f37p->client->dev;
	u8 id_h = 0;
	u8 id_l = 0;
	int ret;

	ret = jx_f37p_read_reg(client, JX_F37P_PIDH_ADDR, &id_h);
	ret |= jx_f37p_read_reg(client, JX_F37P_PIDL_ADDR, &id_l);
	if (id_h != CHIP_ID_H && id_l != CHIP_ID_L) {
		dev_err(dev, "Wrong camera sensor id(0x%02x%02x)\n",
			id_h, id_l);
		return -EINVAL;
	}

	dev_info(dev, "Detected jx_f37p (0x%02x%02x) sensor\n",
		id_h, id_l);

	return ret;
}

static int jx_f37p_configure_regulators(struct jx_f37p *jx_f37p)
{
	unsigned int i;

	for (i = 0; i < JX_F37P_NUM_SUPPLIES; i++)
		jx_f37p->supplies[i].supply = jx_f37p_supply_names[i];

	return devm_regulator_bulk_get(&jx_f37p->client->dev,
				       JX_F37P_NUM_SUPPLIES,
				       jx_f37p->supplies);
}

static int jx_f37p_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct jx_f37p *jx_f37p;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	jx_f37p = devm_kzalloc(dev, sizeof(*jx_f37p), GFP_KERNEL);
	if (!jx_f37p)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &jx_f37p->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &jx_f37p->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &jx_f37p->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &jx_f37p->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	jx_f37p->client = client;
	jx_f37p->cur_mode = &supported_modes[0];

	jx_f37p->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(jx_f37p->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	jx_f37p->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(jx_f37p->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	jx_f37p->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(jx_f37p->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = jx_f37p_configure_regulators(jx_f37p);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&jx_f37p->mutex);

	sd = &jx_f37p->subdev;
	v4l2_i2c_subdev_init(sd, client, &jx_f37p_subdev_ops);
	ret = jx_f37p_initialize_controls(jx_f37p);
	if (ret)
		goto err_destroy_mutex;

	ret = __jx_f37p_power_on(jx_f37p);
	if (ret)
		goto err_free_handler;

	ret = jx_f37p_check_sensor_id(jx_f37p, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &jx_f37p_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	jx_f37p->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &jx_f37p->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(jx_f37p->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 jx_f37p->module_index, facing,
		 JX_F37P_NAME, dev_name(sd->dev));

	ret = v4l2_async_register_subdev(sd);
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
	__jx_f37p_power_off(jx_f37p);
err_free_handler:
	v4l2_ctrl_handler_free(&jx_f37p->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&jx_f37p->mutex);

	return ret;
}

static int jx_f37p_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct jx_f37p *jx_f37p = to_jx_f37p(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&jx_f37p->ctrl_handler);
	mutex_destroy(&jx_f37p->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__jx_f37p_power_off(jx_f37p);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id jx_f37p_of_match[] = {
	{ .compatible = "soi,jx_f37p" },
	{},
};
MODULE_DEVICE_TABLE(of, jx_f37p_of_match);
#endif

static const struct i2c_device_id jx_f37p_match_id[] = {
	{ "soi,jx_f37p", 0 },
	{ },
};

static struct i2c_driver jx_f37p_i2c_driver = {
	.driver = {
		.name = JX_F37P_NAME,
		.pm = &jx_f37p_pm_ops,
		.of_match_table = of_match_ptr(jx_f37p_of_match),
	},
	.probe		= &jx_f37p_probe,
	.remove		= &jx_f37p_remove,
	.id_table	= jx_f37p_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&jx_f37p_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&jx_f37p_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("SOI jx_f37p sensor driver by steven.ou");
MODULE_LICENSE("GPL v2");
