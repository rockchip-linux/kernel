// SPDX-License-Identifier: GPL-2.0
/*
 * s5k4h7yx driver
 *
 * Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X01 add poweron function.
 * V0.0X01.0X02 fix mclk issue when probe multiple camera.
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
#include <linux/rk-preisp.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x02)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define S5K4H7YX_LANES			4
#define S5K4H7YX_BITS_PER_SAMPLE	10

#define S5K4H7YX_LINK_FREQ		500000000 //1080P is 200M

/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define S5K4H7YX_PIXEL_RATE		(S5K4H7YX_LINK_FREQ / 10 * 2 * 4)

#define S5K4H7YX_XVCLK_FREQ		 24000000

#define CHIP_ID				0x487B
#define S5K4H7YX_REG_CHIP_ID		0x0000	//read only reg

#define S5K4H7YX_REG_CTRL_MODE		0x0100
#define S5K4H7YX_MODE_SW_STANDBY	0x00
#define S5K4H7YX_MODE_STREAMING		0x01

/*
 * mirror&flip
 * 0: No mirror&flip
 * 1: horizontal mirror
 * 2: Vertical flip
 * 3: Horizontal mirror & Vertical flip
 */
#define S5K4H7YX_REG_ORIENTATION_MODE	0x0101
#define MIRROR_BIT_MASK			BIT(0)
#define FLIP_BIT_MASK			BIT(1)


#define S5K4H7YX_REG_EXPOSURE		0x0202
#define	S5K4H7YX_EXPOSURE_MIN		2
#define	S5K4H7YX_EXPOSURE_STEP		1
#define S5K4H7YX_VTS_MAX		0xfffc

#define S5K4H7YX_REG_GAIN		0x0204
#define S5K4H7YX_GAIN_MIN		0x0020
#define S5K4H7YX_GAIN_MAX		0x0800
#define S5K4H7YX_GAIN_STEP		1
#define S5K4H7YX_GAIN_DEFAULT	32

// #define S5K4H7YX_REG_DGAIN	0x020d
#define S5K4H7YX_REG_DGAINGR	0x020e
#define S5K4H7YX_REG_DGAINR    0x0210
#define S5K4H7YX_REG_DGAINB    0x0212
#define S5K4H7YX_REG_DGAINGB   0x0214
// #define S5K4H7YX_DGAIN_MODE	1


#define S5K4H7YX_REG_TEST_PATTERN	0x0601
#define S5K4H7YX_TEST_PATTERN_ENABLE	0x1
#define S5K4H7YX_TEST_PATTERN_DISABLE	0x0

#define S5K4H7YX_REG_VTS		0x0340

#define REG_NULL			0xFFFF
#define REG_DELAY			0xFFFE

#define S5K4H7YX_REG_VALUE_08BIT	1
#define S5K4H7YX_REG_VALUE_16BIT	2
#define S5K4H7YX_REG_VALUE_24BIT	3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"
#define S5K4H7YX_NAME			"s5k4h7yx"

static const char * const s5k4h7yx_supply_names[] = {
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
	"avdd",		/* Analog power */
};

#define S5K4H7YX_NUM_SUPPLIES ARRAY_SIZE(s5k4h7yx_supply_names)

enum s5k4h7yx_max_pad {
	PAD0, /* link to isp */
	PAD1, /* link to csi wr0 | hdr x2:L x3:M */
	PAD2, /* link to csi wr1 | hdr      x3:L */
	PAD3, /* link to csi wr2 | hdr x2:M x3:S */
	PAD_MAX,
};

struct regval {
	u16 addr;
	u16 val;
};

struct s5k4h7yx_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 vc[PAD_MAX];
};

struct s5k4h7yx {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct gpio_desc	*pwren_gpio;
	struct gpio_desc	*iovdd_gpio;
	struct gpio_desc	*avdd_gpio;
	struct gpio_desc	*dvdd_gpio;
	struct regulator_bulk_data supplies[S5K4H7YX_NUM_SUPPLIES];

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*h_flip;
	struct v4l2_ctrl	*v_flip;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct s5k4h7yx_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	u8			flip;
};

#define to_s5k4h7yx(sd) container_of(sd, struct s5k4h7yx, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval s5k4h7yx_global_regs[] = {
	{0x0100, 0x00},
	{0x0000, 0x12},
	{0x0000, 0x48},
	{0x0A02, 0x15},
	{0x0100, 0x00},
	{0x0B05, 0x01},
	{0x3074, 0x06},
	{0x3075, 0x2F},
	{0x308A, 0x20},
	{0x308B, 0x08},
	{0x308C, 0x0B},
	{0x3081, 0x07},
	{0x307B, 0x85},
	{0x307A, 0x0A},
	{0x3079, 0x0A},
	{0x306E, 0x71},
	{0x306F, 0x28},
	{0x301F, 0x20},
	{0x306B, 0x9A},
	{0x3091, 0x1F},
	{0x30C4, 0x06},
	{0x3200, 0x09},
	{0x306A, 0x79},
	{0x30B0, 0xFF},
	{0x306D, 0x08},
	{0x3080, 0x00},
	{0x3929, 0x3F},
	{0x3084, 0x16},
	{0x3070, 0x0F},
	{0x3B45, 0x01},
	{0x30C2, 0x05},
	{0x3069, 0x87},
	{0x3924, 0x7F},
	{0x3925, 0xFD},
	{0x3C08, 0xFF},
	{0x3C09, 0xFF},
	{0x3C31, 0xFF},
	{0x3C32, 0xFF},
	{0x300A, 0x52},
	{0x3012, 0x52},
	{0x3013, 0x36},
	{0x3019, 0x5F},
	{0x301A, 0x57},
	{0x3024, 0x10},
	{0x3025, 0x4E},
	{0x3026, 0x9A},
	{0x302D, 0x0B},
	{0x302E, 0x09},
	{0x3C03, 0x3F},
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 500Mbps, 4lane
 */
static const struct regval s5k4h7yx_linear_3264x2448_regs[] = {
	{0x0100, 0x00},
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x0305, 0x06},
	{0x0306, 0x00},
	{0x0307, 0x88},
	{0x030D, 0x06},
	{0x030E, 0x00},
	{0x030F, 0xB7},
	{0x3C1F, 0x00},
	{0x3C17, 0x00},
	{0x3C1C, 0x05},
	{0x3C1D, 0x15},
	{0x0301, 0x04},
	{0x0820, 0x02},
	{0x0821, 0xDC},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0114, 0x03},
	{0x3906, 0x00},
	{0x0344, 0x00},
	{0x0345, 0x08},
	{0x0346, 0x00},
	{0x0347, 0x08},
	{0x0348, 0x0C},
	{0x0349, 0xC7},
	{0x034A, 0x09},
	{0x034B, 0x97},
	{0x034C, 0x0C},
	{0x034D, 0xC0},
	{0x034E, 0x09},
	{0x034F, 0x90},
	{0x0900, 0x00},
	{0x0901, 0x00},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0101, 0x00},
	{0x0340, 0x09},
	{0x0341, 0xE2},
	{0x0342, 0x0D},
	{0x0343, 0xFC},
	{0x0200, 0x0D},
	{0x0201, 0x6C},
	{0x0202, 0x00},
	{0x0203, 0x02},
	{0x3400, 0x01},
	{0x0100, 0x01},
	{REG_NULL, 0x00},
};

static const struct regval s5k4h7yx_linear_1920x1080_regs[] = {
	{0x0100, 0x00},
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x0305, 0x06},
	{0x0306, 0x00},
	{0x0307, 0x8C},
	{0x030D, 0x06},
	{0x030E, 0x00},
	{0x030F, 0x64},
	{0x3C1F, 0x00},
	{0x3C17, 0x00},
	{0x3C1C, 0x05},
	{0x3C1D, 0x15},
	{0x0301, 0x04},
	{0x0820, 0x01},
	{0x0821, 0x90},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0114, 0x03},
	{0x3906, 0x00},
	{0x0344, 0x02},
	{0x0345, 0xA8},
	{0x0346, 0x02},
	{0x0347, 0xB4},
	{0x0348, 0x0A},
	{0x0349, 0x27},
	{0x034A, 0x06},
	{0x034B, 0xEB},
	{0x034C, 0x07},
	{0x034D, 0x80},
	{0x034E, 0x04},
	{0x034F, 0x38},
	{0x0900, 0x00},
	{0x0901, 0x00},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0101, 0x00},
	{0x0340, 0x09},
	{0x0341, 0xE2},
	{0x0342, 0x0E},
	{0x0343, 0x68},
	{0x0200, 0x0D},
	{0x0201, 0xD8},
	{0x0202, 0x00},
	{0x0203, 0x02},
	{0x3400, 0x01},
	{0x0100, 0x01},
	{REG_NULL, 0x00},
};

static const struct s5k4h7yx_mode supported_modes[] = {
	{
		.width = 3264,
		.height = 2448,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0990,
		.hts_def = 0x0DFC,
		.vts_def = 0x09E2,
		.reg_list = s5k4h7yx_linear_3264x2448_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0438,
		.hts_def = 0x0E68,
		.vts_def = 0x09E2,
		.reg_list = s5k4h7yx_linear_1920x1080_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const s64 link_freq_menu_items[] = {
	S5K4H7YX_LINK_FREQ
};

static const char * const s5k4h7yx_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int s5k4h7yx_write_reg(struct i2c_client *client, u16 reg,
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

static int s5k4h7yx_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		if (unlikely(regs[i].addr == REG_DELAY))
			usleep_range(regs[i].val, regs[i].val * 2);
		else
			ret = s5k4h7yx_write_reg(client, regs[i].addr,
					       S5K4H7YX_REG_VALUE_08BIT,
					       regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int s5k4h7yx_read_reg(struct i2c_client *client, u16 reg,
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

static int s5k4h7yx_get_reso_dist(const struct s5k4h7yx_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
			abs(mode->height - framefmt->height);
}

static const struct s5k4h7yx_mode *
s5k4h7yx_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = s5k4h7yx_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}
	return &supported_modes[cur_best_fit];
}

static int s5k4h7yx_get_fmtcode(int orientationmode)
{
	int fmtcode = 0;

	switch (orientationmode) {
	case 0:
		fmtcode = MEDIA_BUS_FMT_SGRBG10_1X10;
		break;
	case 1:
		fmtcode = MEDIA_BUS_FMT_SRGGB10_1X10;
		break;
	case 2:
		fmtcode = MEDIA_BUS_FMT_SBGGR10_1X10;
		break;
	case 3:
		fmtcode = MEDIA_BUS_FMT_SGBRG10_1X10;
		break;
	default:
		fmtcode = MEDIA_BUS_FMT_SGRBG10_1X10;
		break;
	}
	return fmtcode;
}

static int s5k4h7yx_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);
	const struct s5k4h7yx_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&s5k4h7yx->mutex);
	mode = s5k4h7yx_find_best_fit(fmt);
	fmt->format.code = s5k4h7yx_get_fmtcode(s5k4h7yx->flip);
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&s5k4h7yx->mutex);
		return -ENOTTY;
#endif
	} else {
		s5k4h7yx->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(s5k4h7yx->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(s5k4h7yx->vblank, vblank_def,
					 S5K4H7YX_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&s5k4h7yx->mutex);

	return 0;
}

static int s5k4h7yx_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);
	const struct s5k4h7yx_mode *mode = s5k4h7yx->cur_mode;

	mutex_lock(&s5k4h7yx->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&s5k4h7yx->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = s5k4h7yx_get_fmtcode(s5k4h7yx->flip);
		fmt->format.field = V4L2_FIELD_NONE;
		/* format info: width/height/data type/virctual channel */
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&s5k4h7yx->mutex);

	return 0;
}

static int s5k4h7yx_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = s5k4h7yx_get_fmtcode(s5k4h7yx->flip);

	return 0;
}

static int s5k4h7yx_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);

	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != s5k4h7yx_get_fmtcode(s5k4h7yx->flip))
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int s5k4h7yx_enable_test_pattern(struct s5k4h7yx *s5k4h7yx, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | S5K4H7YX_TEST_PATTERN_ENABLE;
	else
		val = S5K4H7YX_TEST_PATTERN_DISABLE;

	return s5k4h7yx_write_reg(s5k4h7yx->client, S5K4H7YX_REG_TEST_PATTERN,
				S5K4H7YX_REG_VALUE_08BIT, val);
}

static int s5k4h7yx_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);
	const struct s5k4h7yx_mode *mode = s5k4h7yx->cur_mode;

	mutex_lock(&s5k4h7yx->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&s5k4h7yx->mutex);

	return 0;
}

static int s5k4h7yx_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);
	const struct s5k4h7yx_mode *mode = s5k4h7yx->cur_mode;
	u32 val = 1 << (S5K4H7YX_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	if (mode->hdr_mode != NO_HDR)
		val |= V4L2_MBUS_CSI2_CHANNEL_1;
	if (mode->hdr_mode == HDR_X3)
		val |= V4L2_MBUS_CSI2_CHANNEL_2;

	config->type = V4L2_MBUS_CSI2;
	config->flags = val;

	return 0;
}

static void s5k4h7yx_get_module_inf(struct s5k4h7yx *s5k4h7yx,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, S5K4H7YX_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, s5k4h7yx->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, s5k4h7yx->len_name, sizeof(inf->base.lens));
}

static long s5k4h7yx_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);
	struct rkmodule_hdr_cfg *hdr;
	u32 i, h, w;
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		s5k4h7yx_get_module_inf(s5k4h7yx, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = s5k4h7yx->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = s5k4h7yx->cur_mode->width;
		h = s5k4h7yx->cur_mode->height;
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				s5k4h7yx->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == ARRAY_SIZE(supported_modes)) {
			dev_err(&s5k4h7yx->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = s5k4h7yx->cur_mode->hts_def -
			    s5k4h7yx->cur_mode->width;
			h = s5k4h7yx->cur_mode->vts_def -
			    s5k4h7yx->cur_mode->height;
			__v4l2_ctrl_modify_range(s5k4h7yx->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(s5k4h7yx->vblank, h,
						 S5K4H7YX_VTS_MAX -
						 s5k4h7yx->cur_mode->height,
						 1, h);
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long s5k4h7yx_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	long ret;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = s5k4h7yx_ioctl(sd, cmd, inf);
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
			ret = s5k4h7yx_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = s5k4h7yx_ioctl(sd, cmd, hdr);
		if (!ret)
			ret = copy_to_user(up, hdr, sizeof(*hdr));
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdr, up, sizeof(*hdr));
		if (!ret)
			ret = s5k4h7yx_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		hdrae = kzalloc(sizeof(*hdrae), GFP_KERNEL);
		if (!hdrae) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdrae, up, sizeof(*hdrae));
		if (!ret)
			ret = s5k4h7yx_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __s5k4h7yx_start_stream(struct s5k4h7yx *s5k4h7yx)
{
	int ret;

	ret = s5k4h7yx_write_array(s5k4h7yx->client, s5k4h7yx->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&s5k4h7yx->mutex);
	ret = v4l2_ctrl_handler_setup(&s5k4h7yx->ctrl_handler);
	mutex_lock(&s5k4h7yx->mutex);
	if (ret)
		return ret;
	return s5k4h7yx_write_reg(s5k4h7yx->client, S5K4H7YX_REG_CTRL_MODE,
				S5K4H7YX_REG_VALUE_08BIT, S5K4H7YX_MODE_STREAMING);
}

static int __s5k4h7yx_stop_stream(struct s5k4h7yx *s5k4h7yx)
{
	return s5k4h7yx_write_reg(s5k4h7yx->client, S5K4H7YX_REG_CTRL_MODE,
				S5K4H7YX_REG_VALUE_08BIT, S5K4H7YX_MODE_SW_STANDBY);
}

static int s5k4h7yx_s_stream(struct v4l2_subdev *sd, int on)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);
	struct i2c_client *client = s5k4h7yx->client;
	int ret = 0;

	mutex_lock(&s5k4h7yx->mutex);
	on = !!on;
	if (on == s5k4h7yx->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __s5k4h7yx_start_stream(s5k4h7yx);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__s5k4h7yx_stop_stream(s5k4h7yx);
		pm_runtime_put(&client->dev);
	}

	s5k4h7yx->streaming = on;

unlock_and_return:
	mutex_unlock(&s5k4h7yx->mutex);

	return ret;
}

static int s5k4h7yx_s_power(struct v4l2_subdev *sd, int on)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);
	struct i2c_client *client = s5k4h7yx->client;
	int ret = 0;

	mutex_lock(&s5k4h7yx->mutex);

	/* If the power state is not modified - no work to do. */
	if (s5k4h7yx->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = s5k4h7yx_write_array(s5k4h7yx->client, s5k4h7yx_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		s5k4h7yx->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		s5k4h7yx->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&s5k4h7yx->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 s5k4h7yx_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, S5K4H7YX_XVCLK_FREQ / 1000 / 1000);
}

static int __s5k4h7yx_power_on(struct s5k4h7yx *s5k4h7yx)
{
	int ret;
	u32 delay_us;
	struct device *dev = &s5k4h7yx->client->dev;

	if (!IS_ERR_OR_NULL(s5k4h7yx->pins_default)) {
		ret = pinctrl_select_state(s5k4h7yx->pinctrl,
					   s5k4h7yx->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(s5k4h7yx->xvclk, S5K4H7YX_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(s5k4h7yx->xvclk) != S5K4H7YX_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(s5k4h7yx->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(s5k4h7yx->dvdd_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->dvdd_gpio, 0);
	if (!IS_ERR(s5k4h7yx->avdd_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->avdd_gpio, 0);

	if (!IS_ERR(s5k4h7yx->iovdd_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->iovdd_gpio, 0);
	if (!IS_ERR(s5k4h7yx->reset_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->reset_gpio, 0);

	if (!IS_ERR(s5k4h7yx->pwdn_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->pwdn_gpio, 0);

	usleep_range(500, 1000);
	ret = regulator_bulk_enable(S5K4H7YX_NUM_SUPPLIES, s5k4h7yx->supplies);

	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}
	if (!IS_ERR(s5k4h7yx->dvdd_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->dvdd_gpio, 1);
	if (!IS_ERR(s5k4h7yx->avdd_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->avdd_gpio, 1);

	if (!IS_ERR(s5k4h7yx->iovdd_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->iovdd_gpio, 1);

	if (!IS_ERR(s5k4h7yx->pwren_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->pwren_gpio, 1);

	usleep_range(1000, 1100);
	if (!IS_ERR(s5k4h7yx->pwdn_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->pwdn_gpio, 1);
	usleep_range(100, 150);
	if (!IS_ERR(s5k4h7yx->reset_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->reset_gpio, 1);

	usleep_range(12000, 16000);
	/* 8192 cycles prior to first SCCB transaction */
	delay_us = s5k4h7yx_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(s5k4h7yx->xvclk);

	return ret;
}

static void __s5k4h7yx_power_off(struct s5k4h7yx *s5k4h7yx)
{
	int ret;
	struct device *dev = &s5k4h7yx->client->dev;

	if (!IS_ERR(s5k4h7yx->dvdd_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->dvdd_gpio, 0);

	if (!IS_ERR(s5k4h7yx->avdd_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->avdd_gpio, 0);

	if (!IS_ERR(s5k4h7yx->iovdd_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->iovdd_gpio, 0);

	if (!IS_ERR(s5k4h7yx->pwdn_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->pwdn_gpio, 0);
	clk_disable_unprepare(s5k4h7yx->xvclk);
	if (!IS_ERR(s5k4h7yx->reset_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(s5k4h7yx->pins_sleep)) {
		ret = pinctrl_select_state(s5k4h7yx->pinctrl,
					   s5k4h7yx->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(S5K4H7YX_NUM_SUPPLIES, s5k4h7yx->supplies);
	if (!IS_ERR(s5k4h7yx->pwren_gpio))
		gpiod_set_value_cansleep(s5k4h7yx->pwren_gpio, 0);
}

static int s5k4h7yx_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);

	return __s5k4h7yx_power_on(s5k4h7yx);
}

static int s5k4h7yx_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);

	__s5k4h7yx_power_off(s5k4h7yx);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int s5k4h7yx_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct s5k4h7yx_mode *def_mode = &supported_modes[0];

	mutex_lock(&s5k4h7yx->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = s5k4h7yx_get_fmtcode(s5k4h7yx->flip);
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&s5k4h7yx->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int s5k4h7yx_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_frame_interval_enum *fie)
{
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);

	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fie->code = s5k4h7yx_get_fmtcode(s5k4h7yx->flip);

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops s5k4h7yx_pm_ops = {
	SET_RUNTIME_PM_OPS(s5k4h7yx_runtime_suspend,
			   s5k4h7yx_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops s5k4h7yx_internal_ops = {
	.open = s5k4h7yx_open,
};
#endif

static const struct v4l2_subdev_core_ops s5k4h7yx_core_ops = {
	.s_power = s5k4h7yx_s_power,
	.ioctl = s5k4h7yx_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = s5k4h7yx_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops s5k4h7yx_video_ops = {
	.s_stream = s5k4h7yx_s_stream,
	.g_frame_interval = s5k4h7yx_g_frame_interval,
	.g_mbus_config = s5k4h7yx_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops s5k4h7yx_pad_ops = {
	.enum_mbus_code = s5k4h7yx_enum_mbus_code,
	.enum_frame_size = s5k4h7yx_enum_frame_sizes,
	.enum_frame_interval = s5k4h7yx_enum_frame_interval,
	.get_fmt = s5k4h7yx_get_fmt,
	.set_fmt = s5k4h7yx_set_fmt,
};

static const struct v4l2_subdev_ops s5k4h7yx_subdev_ops = {
	.core	= &s5k4h7yx_core_ops,
	.video	= &s5k4h7yx_video_ops,
	.pad	= &s5k4h7yx_pad_ops,
};

static int s5k4h7yx_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k4h7yx *s5k4h7yx = container_of(ctrl->handler,
					     struct s5k4h7yx, ctrl_handler);
	struct i2c_client *client = s5k4h7yx->client;
	s64 max;
	u32 again = 0;
	u32 dgain = 0;
	int ret = 0;
	u32 val = 0;

	/*Propagate change of current control to all related controls*/
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/*Update max exposure while meeting expected vblanking*/
		max = s5k4h7yx->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(s5k4h7yx->exposure,
					 s5k4h7yx->exposure->minimum,
					 max,
					 s5k4h7yx->exposure->step,
					 s5k4h7yx->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = s5k4h7yx_write_reg(s5k4h7yx->client, S5K4H7YX_REG_EXPOSURE,
					 S5K4H7YX_REG_VALUE_16BIT,
					 ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		again = ctrl->val > 512 ? 512 : ctrl->val;
		dgain = ctrl->val > 512 ? ctrl->val - 512 : 0;
		ret = s5k4h7yx_write_reg(s5k4h7yx->client, S5K4H7YX_REG_GAIN,
				       S5K4H7YX_REG_VALUE_16BIT,
				       again);
		if (dgain > 0) {
			ret |= s5k4h7yx_write_reg(s5k4h7yx->client,
						S5K4H7YX_REG_DGAINGR,
						S5K4H7YX_REG_VALUE_16BIT,
						dgain);
			ret |= s5k4h7yx_write_reg(s5k4h7yx->client,
						S5K4H7YX_REG_DGAINR,
						S5K4H7YX_REG_VALUE_16BIT,
						dgain);
			ret |= s5k4h7yx_write_reg(s5k4h7yx->client,
						S5K4H7YX_REG_DGAINB,
						S5K4H7YX_REG_VALUE_16BIT,
						dgain);
			ret |= s5k4h7yx_write_reg(s5k4h7yx->client,
						S5K4H7YX_REG_DGAINGB,
						S5K4H7YX_REG_VALUE_16BIT,
						dgain);
		}
		break;
	case V4L2_CID_VBLANK:
		ret = s5k4h7yx_write_reg(s5k4h7yx->client, S5K4H7YX_REG_VTS,
					 S5K4H7YX_REG_VALUE_16BIT,
					 ctrl->val + s5k4h7yx->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = s5k4h7yx_enable_test_pattern(s5k4h7yx, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = s5k4h7yx_read_reg(client,
					S5K4H7YX_REG_ORIENTATION_MODE,
					S5K4H7YX_REG_VALUE_08BIT,
					&val);
		if (ctrl->val)
			val |= MIRROR_BIT_MASK;
		else
			val &= ~MIRROR_BIT_MASK;
		ret |= s5k4h7yx_write_reg(client,
					S5K4H7YX_REG_ORIENTATION_MODE,
					S5K4H7YX_REG_VALUE_08BIT,
					val);
		if (ret == 0)
			s5k4h7yx->flip = val;
		break;
	case V4L2_CID_VFLIP:
		ret = s5k4h7yx_read_reg(client,
					S5K4H7YX_REG_ORIENTATION_MODE,
					S5K4H7YX_REG_VALUE_08BIT,
					&val);
		if (ctrl->val)
			val |= FLIP_BIT_MASK;
		else
			val &= ~FLIP_BIT_MASK;
		ret |= s5k4h7yx_write_reg(client,
					S5K4H7YX_REG_ORIENTATION_MODE,
					S5K4H7YX_REG_VALUE_08BIT,
					val);
		if (ret == 0)
			s5k4h7yx->flip = val;
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k4h7yx_ctrl_ops = {
	.s_ctrl = s5k4h7yx_set_ctrl,
};

static int s5k4h7yx_initialize_controls(struct s5k4h7yx *s5k4h7yx)
{
	const struct s5k4h7yx_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &s5k4h7yx->ctrl_handler;
	mode = s5k4h7yx->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &s5k4h7yx->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, S5K4H7YX_PIXEL_RATE, 1, S5K4H7YX_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	s5k4h7yx->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (s5k4h7yx->hblank)
		s5k4h7yx->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	s5k4h7yx->vblank = v4l2_ctrl_new_std(handler, &s5k4h7yx_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   S5K4H7YX_VTS_MAX - mode->height,
					    1, vblank_def);

	exposure_max = mode->vts_def - 4;
	s5k4h7yx->exposure = v4l2_ctrl_new_std(handler, &s5k4h7yx_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5K4H7YX_EXPOSURE_MIN,
					     exposure_max,
					     S5K4H7YX_EXPOSURE_STEP,
					     mode->exp_def);

	s5k4h7yx->anal_gain = v4l2_ctrl_new_std(handler, &s5k4h7yx_ctrl_ops,
					      V4L2_CID_ANALOGUE_GAIN,
					      S5K4H7YX_GAIN_MIN,
					      S5K4H7YX_GAIN_MAX,
					      S5K4H7YX_GAIN_STEP,
					      S5K4H7YX_GAIN_DEFAULT);

	s5k4h7yx->test_pattern =
		v4l2_ctrl_new_std_menu_items(handler,
					     &s5k4h7yx_ctrl_ops,
				V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(s5k4h7yx_test_pattern_menu) - 1,
				0, 0, s5k4h7yx_test_pattern_menu);

	s5k4h7yx->h_flip = v4l2_ctrl_new_std(handler, &s5k4h7yx_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);

	s5k4h7yx->v_flip = v4l2_ctrl_new_std(handler, &s5k4h7yx_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	s5k4h7yx->flip = 0;

	if (handler->error) {
		ret = handler->error;
		dev_err(&s5k4h7yx->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	s5k4h7yx->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int s5k4h7yx_check_sensor_id(struct s5k4h7yx *s5k4h7yx,
				  struct i2c_client *client)
{
	struct device *dev = &s5k4h7yx->client->dev;
	u32 reg = 0;
	int ret;

	ret = s5k4h7yx_read_reg(client, S5K4H7YX_REG_CHIP_ID,
			      S5K4H7YX_REG_VALUE_16BIT, &reg);
	if (reg != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", reg, ret);
		return -ENODEV;
	}
	dev_info(dev, "detected s5kgm1%04x sensor\n", reg);
	return 0;
}

static int s5k4h7yx_configure_regulators(struct s5k4h7yx *s5k4h7yx)
{
	unsigned int i;

	for (i = 0; i < S5K4H7YX_NUM_SUPPLIES; i++)
		s5k4h7yx->supplies[i].supply = s5k4h7yx_supply_names[i];

	return devm_regulator_bulk_get(&s5k4h7yx->client->dev,
				       S5K4H7YX_NUM_SUPPLIES,
				       s5k4h7yx->supplies);
}

static int s5k4h7yx_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct s5k4h7yx *s5k4h7yx;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	u32 i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	s5k4h7yx = devm_kzalloc(dev, sizeof(*s5k4h7yx), GFP_KERNEL);
	if (!s5k4h7yx)
		return -ENOMEM;

	of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &s5k4h7yx->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &s5k4h7yx->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &s5k4h7yx->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &s5k4h7yx->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	s5k4h7yx->client = client;
	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			s5k4h7yx->cur_mode = &supported_modes[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(supported_modes))
		s5k4h7yx->cur_mode = &supported_modes[0];

	s5k4h7yx->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(s5k4h7yx->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	s5k4h7yx->pwren_gpio = devm_gpiod_get(dev, "pwren", GPIOD_OUT_LOW);
	if (IS_ERR(s5k4h7yx->pwren_gpio))
		dev_warn(dev, "Failed to get pwren-gpios\n");

	s5k4h7yx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(s5k4h7yx->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	s5k4h7yx->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(s5k4h7yx->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	s5k4h7yx->iovdd_gpio = devm_gpiod_get(dev, "iovdd", GPIOD_OUT_LOW);
	if (IS_ERR(s5k4h7yx->iovdd_gpio))
		dev_warn(dev, "Failed to get iovdd-gpios\n");

		s5k4h7yx->avdd_gpio = devm_gpiod_get(dev, "avdd", GPIOD_OUT_LOW);
	if (IS_ERR(s5k4h7yx->avdd_gpio))
		dev_warn(dev, "Failed to get avdd-gpios\n");

		s5k4h7yx->dvdd_gpio = devm_gpiod_get(dev, "dvdd", GPIOD_OUT_LOW);
	if (IS_ERR(s5k4h7yx->dvdd_gpio))
		dev_warn(dev, "Failed to get dvdd-gpios\n");

	s5k4h7yx->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(s5k4h7yx->pinctrl)) {
		s5k4h7yx->pins_default =
			pinctrl_lookup_state(s5k4h7yx->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(s5k4h7yx->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		s5k4h7yx->pins_sleep =
			pinctrl_lookup_state(s5k4h7yx->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(s5k4h7yx->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = s5k4h7yx_configure_regulators(s5k4h7yx);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&s5k4h7yx->mutex);

	sd = &s5k4h7yx->subdev;
	v4l2_i2c_subdev_init(sd, client, &s5k4h7yx_subdev_ops);
	ret = s5k4h7yx_initialize_controls(s5k4h7yx);
	if (ret)
		goto err_destroy_mutex;

	ret = __s5k4h7yx_power_on(s5k4h7yx);
	if (ret)
		goto err_free_handler;

	ret = s5k4h7yx_check_sensor_id(s5k4h7yx, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &s5k4h7yx_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	s5k4h7yx->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &s5k4h7yx->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(s5k4h7yx->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 s5k4h7yx->module_index, facing,
		 S5K4H7YX_NAME, dev_name(sd->dev));
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
	__s5k4h7yx_power_off(s5k4h7yx);
err_free_handler:
	v4l2_ctrl_handler_free(&s5k4h7yx->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&s5k4h7yx->mutex);

	return ret;
}

static int s5k4h7yx_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4h7yx *s5k4h7yx = to_s5k4h7yx(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&s5k4h7yx->ctrl_handler);
	mutex_destroy(&s5k4h7yx->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__s5k4h7yx_power_off(s5k4h7yx);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id s5k4h7yx_of_match[] = {
	{ .compatible = "samsung,s5k4h7yx" },
	{},
};
MODULE_DEVICE_TABLE(of, s5k4h7yx_of_match);
#endif

static const struct i2c_device_id s5k4h7yx_match_id[] = {
	{ "samsung,s5k4h7yx", 0 },
	{ },
};

static struct i2c_driver s5k4h7yx_i2c_driver = {
	.driver = {
		.name = S5K4H7YX_NAME,
		.pm = &s5k4h7yx_pm_ops,
		.of_match_table = of_match_ptr(s5k4h7yx_of_match),
	},
	.probe		= &s5k4h7yx_probe,
	.remove		= &s5k4h7yx_remove,
	.id_table	= s5k4h7yx_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&s5k4h7yx_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&s5k4h7yx_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("samsung s5k4h7yx sensor driver");
MODULE_LICENSE("GPL v2");
