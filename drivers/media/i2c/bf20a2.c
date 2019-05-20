// SPDX-License-Identifier: GPL-2.0
/*
 * BF20A2 CMOS Image Sensor driver
 * Copyright (C) 2019 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * v0.0X01.0X00 create file.
 * v0.0X01.0X01 add poweron function.
 * v0.0X01.0X02 fix mclk issue when probe multiple camera.
 * v0.0X01.0X03 add enum_frame_interval function.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_gpio.h>

#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>
#include <linux/rk-camera-module.h>
#include <linux/version.h>
#include <media/media-entity.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x03)
#define DRIVER_NAME "bf20a2"
#define BF20A2_PIXEL_RATE		(96 * 1000 * 1000)

/*
 * BF20A2 register definitions
 */
#define REG_SOFTWARE_STANDBY		0xd5

#define REG_SC_CHIP_ID_H		0xfc
#define REG_SC_CHIP_ID_L		0xfd

#define BF20A2_REG_TEST_PATTERN		0xb7
#define	BF20A2_TEST_PATTERN_ENABLE	0x80
#define	BF20A2_TEST_PATTERN_DISABLE	0x00

#define REG_NULL			0xFFFF	/* Array end token */

#define SENSOR_ID(_msb, _lsb)		((_msb) << 8 | (_lsb))
#define BF20A2_ID			0x20a2

struct sensor_register {
	u16 addr;
	u8 value;
};

struct bf20a2_framesize {
	u16 width;
	u16 height;
	u16 max_exp_lines;
	struct v4l2_fract max_fps;
	const struct sensor_register *regs;
};

struct bf20a2_pll_ctrl {
	u8 ctrl1;
	u8 ctrl2;
	u8 ctrl3;
};

struct bf20a2_pixfmt {
	u32 code;
	/* Output format Register Value (REG_FORMAT_CTRL00) */
	struct sensor_register *format_ctrl_regs;
};

struct pll_ctrl_reg {
	unsigned int div;
	unsigned char reg;
};

static const char * const bf20a2_supply_names[] = {
	"dovdd",	/* Digital I/O power */
	"avdd",		/* Analog power */
	"dvdd",		/* Digital core power */
};

#define BF20A2_NUM_SUPPLIES ARRAY_SIZE(bf20a2_supply_names)

struct bf20a2 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	unsigned int xvclk_frequency;
	struct clk *xvclk;
	struct gpio_desc *power_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;
	struct gpio_desc *pwdn2_gpio;
	struct regulator_bulk_data supplies[BF20A2_NUM_SUPPLIES];
	struct mutex lock;
	struct i2c_client *client;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_frequency;
	const struct bf20a2_framesize *frame_size;
	int streaming;
	bool power_on;
	u32 module_index;
	const char *module_facing;
	const char *module_name;
	const char *len_name;
};

static const struct sensor_register bf20a2_vga_regs[] = {
	{0xca, 0x82},//PLL 0x82:1M;0x12:2M;0x02:4M;0xc2:1/2M;0x72:1/4M
	{0xcd, 0x84},
	{0x12, 0x20},//DVP
	{0x39, 0x00},
	{0x3a, 0xe0},
	{0xbd, 0x3f},
	{0x15, 0x00},//Bit[4]: VCLK reverse
	{0xcb, 0x09},//Bit[3:2]: LDO sel. 00:1.5V,01:1.6V,10:1.7V,11:1.8V.
	{0xcc, 0xde},
	{0xce, 0x3F},
	{0xcf, 0x50},
	{0xd1, 0x30},
	{0xd5, 0x10},
	{0xd6, 0x0b},//drive capability
	{0xd7, 0x01},
	{0x4a, 0x40},//Bit[1]: Mirror,Bit[0]: Vertical Flip
	{0x32, 0x02},//Dummy line insert before active line low 8 bits
	{0x33, 0x03},//Bit[3:0]: line length high 4 bits
	{0x2a, 0x20},//Bit[7:0]: line length low 8 bits
	{0x1f, 0x18},
	{0x22, 0x18},
	{0x28, 0x60},//manual BLC
	{0x00, 0x3b},
	{0x0d, 0x3b},
	{0x28, 0x63},
	{0xa0, 0x44},//manual AWB
	{0x01, 0x19},
	{0x02, 0x18},
	{0x13, 0x0a},//manual AE
	{0x87, 0x20},//glb_gain
	{0x8c, 0x06},
	{0x8d, 0x72},//8c,8d: int_time
	{0x34, 0x30},//lens shading gain of R
	{0x35, 0x26},//lens shading gain of G
	{0x36, 0x24},//lens shading gain of B
	{0x40, 0x36},//gamma
	{0x41, 0x28},
	{0x42, 0x20},
	{0x43, 0x1a},
	{0x44, 0x18},
	{0x45, 0x14},
	{0x46, 0x12},
	{0x47, 0x10},
	{0x48, 0x0f},
	{0x49, 0x0e},
	{0x4B, 0x0d},
	{0x4C, 0x0c},
	{0x4E, 0x0b},
	{0x4F, 0x0a},
	{0x50, 0x09},
	{0x70, 0x0f},
	{0x72, 0x37}, //0x27
	{0x73, 0x68},
	{0x74, 0x82},
	{0x75, 0x89},
	{0x76, 0x8a},
	{0x78, 0x34},//edge enhancement
	{0x79, 0x00},
	{0x7a, 0x83},
	{0x13, 0x07},
	{0x24, 0x45},//AE target
	{0x80, 0x92},
	{0x82, 0x14},//GLB_MIN1
	{0x83, 0x28},
	{0x84, 0x38},
	{0x85, 0x60},
	{0x86, 0x70},//GLB_MAX3
	{0x8a, 0x96},//50hz banding
	{0x8b, 0x7d},//60hz banding
	{0x8f, 0x82},//bit[6:0]:INT_MIN
	{0x96, 0xb5},
	{0x9a, 0x50},
	{0xf0, 0x83},//8e Bit[5:0]:INT_MAX, the MAX steps of integral time
	{0xf7, 0x8a},
	{0xa0, 0x45},
	{0xa2, 0x08},
	{0xa3, 0x32},
	{0xa4, 0x01},
	{0xa5, 0x2c},
	{0xa7, 0x80},//blue target
	{0xa8, 0x7d},//red target
	{0xa9, 0x18},
	{0xaa, 0x15},
	{0xab, 0x18},
	{0xae, 0x80},//bigger the value, less  bias the pure color
	{0xc8, 0x0d},
	{0xc9, 0x14},
	{0xd3, 0x09},
	{0xd4, 0x24},
	{0x51, 0x0b},//color
	{0x52, 0x0e},
	{0x53, 0x6e},
	{0x54, 0x62},
	{0x57, 0x86},
	{0x58, 0x2d},
	{0x5a, 0x16},
	{0x5C, 0x20},
	{0xb0, 0x78},
	{0xb1, 0xd0},//e0 Cb Saturatison
	{0xb2, 0xc0},//d0 Cr Saturation
	{0xb3, 0x30},//threshold for dark scene
	{0xEe, 0x3C},//scenes register
	{0x56, 0x94},//88 Contrast
	{0x55, 0x8c},//8c Bit[7]:low light scene add luminance control;
	{REG_NULL, 0x00},
};

static const struct bf20a2_framesize bf20a2_framesizes[] = {
	{ /* VGA */
		.width		= 640,
		.height		= 480,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.regs		= bf20a2_vga_regs,
		.max_exp_lines	= 488,
	}
};

static const struct bf20a2_pixfmt bf20a2_formats[] = {
	{
		.code = MEDIA_BUS_FMT_YUYV8_2X8,
	}
};

static inline struct bf20a2 *to_bf20a2(struct v4l2_subdev *sd)
{
	return container_of(sd, struct bf20a2, sd);
}

/* sensor register write */
static int bf20a2_write(struct i2c_client *client, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	dev_dbg(&client->dev, "write reg(0x%x val:0x%x)!\n", reg, val);
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
		"bf20a2 write reg(0x%x val:0x%x) failed !\n", reg, val);

	return ret;
}

/* sensor register read */
static int bf20a2_read(struct i2c_client *client, u8 reg, u8 *val)
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
		"bf20a2 read reg:0x%x failed !\n", reg);

	return ret;
}

static int bf20a2_write_array(struct i2c_client *client,
			      const struct sensor_register *regs)
{
	int i, ret = 0;

	i = 0;
	while (regs[i].addr != REG_NULL) {
		ret = bf20a2_write(client, regs[i].addr, regs[i].value);
		if (ret) {
			dev_err(&client->dev, "%s failed !\n", __func__);
			break;
		}

		i++;
	}

	return ret;
}

static void bf20a2_get_default_format(struct v4l2_mbus_framefmt *format)
{
	format->width = bf20a2_framesizes[0].width;
	format->height = bf20a2_framesizes[0].height;
	format->colorspace = V4L2_COLORSPACE_SRGB;
	format->code = bf20a2_formats[0].code;
	format->field = V4L2_FIELD_NONE;
}

static void bf20a2_set_streaming(struct bf20a2 *bf20a2, int on)
{
	struct i2c_client *client = bf20a2->client;
	int ret;

	dev_dbg(&client->dev, "%s: on: %d\n", __func__, on);

	ret = bf20a2_write(client, REG_SOFTWARE_STANDBY, on);
	if (ret)
		dev_err(&client->dev, "bf20a2 soft standby failed\n");
}

/*
 * V4L2 subdev video and pad level operations
 */

static int bf20a2_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	if (code->index >= ARRAY_SIZE(bf20a2_formats))
		return -EINVAL;

	code->code = bf20a2_formats[code->index].code;

	return 0;
}

static int bf20a2_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int i = ARRAY_SIZE(bf20a2_formats);

	dev_dbg(&client->dev, "%s:\n", __func__);

	if (fse->index >= ARRAY_SIZE(bf20a2_framesizes))
		return -EINVAL;

	while (--i)
		if (fse->code == bf20a2_formats[i].code)
			break;

	fse->code = bf20a2_formats[i].code;

	fse->min_width  = bf20a2_framesizes[fse->index].width;
	fse->max_width  = fse->min_width;
	fse->max_height = bf20a2_framesizes[fse->index].height;
	fse->min_height = fse->max_height;

	return 0;
}

static int bf20a2_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct bf20a2 *bf20a2 = to_bf20a2(sd);

	dev_dbg(&client->dev, "%s enter\n", __func__);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		struct v4l2_mbus_framefmt *mf;

		mf = v4l2_subdev_get_try_format(sd, cfg, 0);
		mutex_lock(&bf20a2->lock);
		fmt->format = *mf;
		mutex_unlock(&bf20a2->lock);
		return 0;
#else
	return -ENOTTY;
#endif
	}

	mutex_lock(&bf20a2->lock);
	fmt->format = bf20a2->format;
	mutex_unlock(&bf20a2->lock);

	dev_dbg(&client->dev, "%s: %x %dx%d\n", __func__,
		bf20a2->format.code, bf20a2->format.width,
		bf20a2->format.height);

	return 0;
}

static void __bf20a2_try_frame_size(struct v4l2_mbus_framefmt *mf,
				    const struct bf20a2_framesize **size)
{
	const struct bf20a2_framesize *fsize = &bf20a2_framesizes[0];
	const struct bf20a2_framesize *match = NULL;
	int i = ARRAY_SIZE(bf20a2_framesizes);
	unsigned int min_err = UINT_MAX;

	while (i--) {
		unsigned int err = abs(fsize->width - mf->width)
				+ abs(fsize->height - mf->height);
		if (err < min_err && fsize->regs[0].addr) {
			min_err = err;
			match = fsize;
		}
		fsize++;
	}

	if (!match)
		match = &bf20a2_framesizes[0];

	mf->width  = match->width;
	mf->height = match->height;

	if (size)
		*size = match;
}

static int bf20a2_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int index = ARRAY_SIZE(bf20a2_formats);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	const struct bf20a2_framesize *size = NULL;
	struct bf20a2 *bf20a2 = to_bf20a2(sd);
	int ret = 0;

	dev_dbg(&client->dev, "%s enter\n", __func__);

	__bf20a2_try_frame_size(mf, &size);

	while (--index >= 0)
		if (bf20a2_formats[index].code == mf->code)
			break;

	if (index < 0)
		return -EINVAL;

	mf->colorspace = V4L2_COLORSPACE_SRGB;
	mf->code = bf20a2_formats[index].code;
	mf->field = V4L2_FIELD_NONE;

	mutex_lock(&bf20a2->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		*mf = fmt->format;
#else
		return -ENOTTY;
#endif
	} else {
		if (bf20a2->streaming) {
			mutex_unlock(&bf20a2->lock);
			return -EBUSY;
		}

		bf20a2->frame_size = size;
		bf20a2->format = fmt->format;
	}

	mutex_unlock(&bf20a2->lock);
	return ret;
}

static void bf20a2_get_module_inf(struct bf20a2 *bf20a2,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, DRIVER_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, bf20a2->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, bf20a2->len_name, sizeof(inf->base.lens));
}

static long bf20a2_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct bf20a2 *bf20a2 = to_bf20a2(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		bf20a2_get_module_inf(bf20a2, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long bf20a2_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = bf20a2_ioctl(sd, cmd, inf);
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
			ret = bf20a2_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int bf20a2_s_stream(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct bf20a2 *bf20a2 = to_bf20a2(sd);
	int ret = 0;

	dev_info(&client->dev, "%s: on: %d, %dx%d\n", __func__, on,
				bf20a2->frame_size->width,
				bf20a2->frame_size->height);

	mutex_lock(&bf20a2->lock);

	on = !!on;

	if (bf20a2->streaming == on)
		goto unlock;

	if (!on) {
		/* Stop Streaming Sequence */
		bf20a2_set_streaming(bf20a2, 0x11);
		bf20a2->streaming = on;
		goto unlock;
	}
	bf20a2_set_streaming(bf20a2, 0x10);
	ret = bf20a2_write_array(client, bf20a2->frame_size->regs);
	bf20a2->streaming = on;

unlock:
	mutex_unlock(&bf20a2->lock);
	return 0;
}

static int bf20a2_set_test_pattern(struct bf20a2 *bf20a2, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | BF20A2_TEST_PATTERN_ENABLE;
	else
		val = BF20A2_TEST_PATTERN_DISABLE;
	return bf20a2_write(bf20a2->client, BF20A2_REG_TEST_PATTERN, val);
}

static int bf20a2_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct bf20a2 *bf20a2 =
			container_of(ctrl->handler, struct bf20a2, ctrls);

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		return bf20a2_set_test_pattern(bf20a2, ctrl->val);
	}

	return 0;
}

static const struct v4l2_ctrl_ops bf20a2_ctrl_ops = {
	.s_ctrl = bf20a2_s_ctrl,
};

static const char * const bf20a2_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bars",
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev internal operations
 */

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int bf20a2_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);

	dev_dbg(&client->dev, "%s:\n", __func__);

	bf20a2_get_default_format(format);

	return 0;
}
#endif

static int bf20a2_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	config->type = V4L2_MBUS_PARALLEL;
	config->flags = V4L2_MBUS_HSYNC_ACTIVE_HIGH |
			V4L2_MBUS_VSYNC_ACTIVE_HIGH |
			V4L2_MBUS_PCLK_SAMPLE_RISING;

	return 0;
}

static int bf20a2_power(struct v4l2_subdev *sd, int on)
{
	int ret = 0;
	struct bf20a2 *bf20a2 = to_bf20a2(sd);
	struct i2c_client *client = bf20a2->client;
	struct device *dev = &bf20a2->client->dev;

	dev_info(&client->dev, "%s(%d) on(%d)\n", __func__, __LINE__, on);
	mutex_lock(&bf20a2->lock);

	/* If the power state is not modified - no work to do. */
	if (bf20a2->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		if (!IS_ERR(bf20a2->pwdn_gpio)) {
			gpiod_set_value_cansleep(bf20a2->pwdn_gpio, 0);
			usleep_range(2000, 5000);
		}
		ret = bf20a2_write_array(client, bf20a2->frame_size->regs);
		if (ret)
			dev_err(dev, "init error\n");
		bf20a2->power_on = true;
	} else {
		if (!IS_ERR(bf20a2->pwdn_gpio)) {
			gpiod_set_value_cansleep(bf20a2->pwdn_gpio, 1);
			usleep_range(2000, 5000);
		}
		bf20a2->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&bf20a2->lock);
	return 0;
}

static int bf20a2_enum_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(bf20a2_framesizes))
		return -EINVAL;

	if (fie->code != MEDIA_BUS_FMT_YUYV8_2X8)
		return -EINVAL;

	fie->width = bf20a2_framesizes[fie->index].width;
	fie->height = bf20a2_framesizes[fie->index].height;
	fie->interval = bf20a2_framesizes[fie->index].max_fps;
	return 0;
}

static const struct v4l2_subdev_core_ops bf20a2_subdev_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.ioctl = bf20a2_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = bf20a2_compat_ioctl32,
#endif
	.s_power = bf20a2_power,
};

static const struct v4l2_subdev_video_ops bf20a2_subdev_video_ops = {
	.s_stream = bf20a2_s_stream,
	.g_mbus_config = bf20a2_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops bf20a2_subdev_pad_ops = {
	.enum_mbus_code = bf20a2_enum_mbus_code,
	.enum_frame_size = bf20a2_enum_frame_sizes,
	.enum_frame_interval = bf20a2_enum_frame_interval,
	.get_fmt = bf20a2_get_fmt,
	.set_fmt = bf20a2_set_fmt,
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_ops bf20a2_subdev_ops = {
	.core  = &bf20a2_subdev_core_ops,
	.video = &bf20a2_subdev_video_ops,
	.pad   = &bf20a2_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops bf20a2_subdev_internal_ops = {
	.open = bf20a2_open,
};
#endif

static int bf20a2_detect(struct bf20a2 *bf20a2)
{
	struct i2c_client *client = bf20a2->client;
	u8 pid, ver;
	int ret;

	dev_dbg(&client->dev, "%s:\n", __func__);

	/* Check sensor revision */
	ret = bf20a2_read(client, REG_SC_CHIP_ID_H, &pid);
	if (!ret)
		ret = bf20a2_read(client, REG_SC_CHIP_ID_L, &ver);

	if (!ret) {
		unsigned short id;

		id = SENSOR_ID(pid, ver);
		if (id != BF20A2_ID) {
			ret = -1;
			dev_err(&client->dev,
				"Sensor detection failed (%04X, %d)\n",
				id, ret);
		} else {
			dev_info(&client->dev, "Found BF%04X sensor\n", id);
			if (!IS_ERR(bf20a2->pwdn_gpio))
				gpiod_set_value_cansleep(bf20a2->pwdn_gpio, 1);
		}
	}

	return ret;
}

static int __bf20a2_power_on(struct bf20a2 *bf20a2)
{
	int ret;
	struct device *dev = &bf20a2->client->dev;

	dev_info(dev, "%s(%d)\n", __func__, __LINE__);
	if (!IS_ERR(bf20a2->xvclk)) {
		ret = clk_set_rate(bf20a2->xvclk, 24000000);
		if (ret < 0)
			dev_info(dev, "Failed to set xvclk rate (24MHz)\n");
	}

	if (!IS_ERR(bf20a2->pwdn_gpio)) {
		gpiod_set_value_cansleep(bf20a2->pwdn_gpio, 1);
		usleep_range(2000, 5000);
	}

	if (!IS_ERR(bf20a2->supplies)) {
		ret = regulator_bulk_enable(BF20A2_NUM_SUPPLIES,
			bf20a2->supplies);
		if (ret < 0)
			dev_info(dev, "Failed to enable regulators\n");

		usleep_range(2000, 5000);
	}

	if (!IS_ERR(bf20a2->pwdn2_gpio)) {
		gpiod_set_value_cansleep(bf20a2->pwdn2_gpio, 1);
		usleep_range(2000, 5000);
	}

	if (!IS_ERR(bf20a2->pwdn_gpio)) {
		gpiod_set_value_cansleep(bf20a2->pwdn_gpio, 0);
		usleep_range(2000, 5000);
	}

	if (!IS_ERR(bf20a2->xvclk)) {
		ret = clk_prepare_enable(bf20a2->xvclk);
		if (ret < 0)
			dev_info(dev, "Failed to enable xvclk\n");
	}

	usleep_range(7000, 10000);

	return 0;
}

static void __bf20a2_power_off(struct bf20a2 *bf20a2)
{
	dev_info(&bf20a2->client->dev, "%s(%d)\n", __func__, __LINE__);
	if (!IS_ERR(bf20a2->xvclk))
		clk_disable_unprepare(bf20a2->xvclk);
	if (!IS_ERR(bf20a2->supplies))
		regulator_bulk_disable(BF20A2_NUM_SUPPLIES, bf20a2->supplies);
	if (!IS_ERR(bf20a2->pwdn_gpio))
		gpiod_set_value_cansleep(bf20a2->pwdn_gpio, 1);
}

static int bf20a2_configure_regulators(struct bf20a2 *bf20a2)
{
	unsigned int i;

	for (i = 0; i < BF20A2_NUM_SUPPLIES; i++)
		bf20a2->supplies[i].supply = bf20a2_supply_names[i];

	return devm_regulator_bulk_get(&bf20a2->client->dev,
				       BF20A2_NUM_SUPPLIES,
				       bf20a2->supplies);
}

static int bf20a2_parse_of(struct bf20a2 *bf20a2)
{
	struct device *dev = &bf20a2->client->dev;
	struct device_node *node = dev->of_node;
	struct gpio_desc *pwdn_gpio;
	unsigned int pwdn = -1;
	enum of_gpio_flags flags;
	int ret;

	bf20a2->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(bf20a2->pwdn_gpio)) {
		dev_info(dev, "Failed to get pwdn-gpios, maybe no used\n");
		pwdn = of_get_named_gpio_flags(node, "pwdn-gpios", 0, &flags);
		pwdn_gpio = gpio_to_desc(pwdn);
		if (IS_ERR(pwdn_gpio))
			dev_info(dev, "Failed to get pwdn-gpios again\n");
		else
			bf20a2->pwdn_gpio = pwdn_gpio;
	}

	bf20a2->pwdn2_gpio = devm_gpiod_get(dev, "pwdn2", GPIOD_OUT_LOW);
	if (IS_ERR(bf20a2->pwdn2_gpio)) {
		dev_info(dev, "Failed to get pwdn2-gpios, maybe no use\n");
		pwdn = of_get_named_gpio_flags(node, "pwdn2-gpios", 0, &flags);
		pwdn_gpio = gpio_to_desc(pwdn);
		if (IS_ERR(pwdn_gpio))
			dev_info(dev, "Failed to get pwdn2-gpios again\n");
		else
			bf20a2->pwdn2_gpio = pwdn_gpio;
	}

	ret = bf20a2_configure_regulators(bf20a2);
	if (ret)
		dev_info(dev, "Failed to get power regulators\n");

	return __bf20a2_power_on(bf20a2);
}

static int bf20a2_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct v4l2_subdev *sd;
	struct bf20a2 *bf20a2;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	bf20a2 = devm_kzalloc(&client->dev, sizeof(*bf20a2), GFP_KERNEL);
	if (!bf20a2)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &bf20a2->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &bf20a2->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &bf20a2->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &bf20a2->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	bf20a2->client = client;
	bf20a2->xvclk = devm_clk_get(&client->dev, "xvclk");
	if (IS_ERR(bf20a2->xvclk)) {
		dev_err(&client->dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	bf20a2_parse_of(bf20a2);

	bf20a2->xvclk_frequency = clk_get_rate(bf20a2->xvclk);
	if (bf20a2->xvclk_frequency < 6000000 ||
	    bf20a2->xvclk_frequency > 27000000)
		return -EINVAL;

	v4l2_ctrl_handler_init(&bf20a2->ctrls, 2);
	bf20a2->link_frequency =
			v4l2_ctrl_new_std(&bf20a2->ctrls, &bf20a2_ctrl_ops,
					  V4L2_CID_PIXEL_RATE, 0,
					  BF20A2_PIXEL_RATE, 1,
					  BF20A2_PIXEL_RATE);

	v4l2_ctrl_new_std_menu_items(&bf20a2->ctrls, &bf20a2_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(bf20a2_test_pattern_menu) - 1,
				     0, 0, bf20a2_test_pattern_menu);
	bf20a2->sd.ctrl_handler = &bf20a2->ctrls;

	if (bf20a2->ctrls.error) {
		dev_err(&client->dev, "%s: control initialization error %d\n",
			__func__, bf20a2->ctrls.error);
		return  bf20a2->ctrls.error;
	}

	sd = &bf20a2->sd;
	client->flags |= I2C_CLIENT_SCCB;
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	v4l2_i2c_subdev_init(sd, client, &bf20a2_subdev_ops);

	sd->internal_ops = &bf20a2_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif

#if defined(CONFIG_MEDIA_CONTROLLER)
	bf20a2->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &bf20a2->pad, 0);
	if (ret < 0) {
		v4l2_ctrl_handler_free(&bf20a2->ctrls);
		return ret;
	}
#endif

	mutex_init(&bf20a2->lock);

	bf20a2_get_default_format(&bf20a2->format);
	bf20a2->frame_size = &bf20a2_framesizes[0];

	ret = bf20a2_detect(bf20a2);
	if (ret < 0)
		goto error;

	memset(facing, 0, sizeof(facing));
	if (strcmp(bf20a2->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 bf20a2->module_index, facing,
		 DRIVER_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret)
		goto error;

	dev_info(&client->dev, "%s sensor driver registered !!\n", sd->name);

	return 0;

error:
	v4l2_ctrl_handler_free(&bf20a2->ctrls);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	mutex_destroy(&bf20a2->lock);
	__bf20a2_power_off(bf20a2);
	return ret;
}

static int bf20a2_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct bf20a2 *bf20a2 = to_bf20a2(sd);

	v4l2_ctrl_handler_free(&bf20a2->ctrls);
	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	mutex_destroy(&bf20a2->lock);

	__bf20a2_power_off(bf20a2);

	return 0;
}

static const struct i2c_device_id bf20a2_id[] = {
	{ "bf20a2", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, bf20a2_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id bf20a2_of_match[] = {
	{ .compatible = "byd,bf20a2", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, bf20a2_of_match);
#endif

static struct i2c_driver bf20a2_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(bf20a2_of_match),
	},
	.probe		= bf20a2_probe,
	.remove		= bf20a2_remove,
	.id_table	= bf20a2_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&bf20a2_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&bf20a2_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_AUTHOR("Benoit Parrot <bparrot@ti.com>");
MODULE_DESCRIPTION("BF20A2 CMOS Image Sensor driver");
MODULE_LICENSE("GPL v2");
