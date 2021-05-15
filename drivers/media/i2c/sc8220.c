// SPDX-License-Identifier: GPL-2.0
/*
 * sc8220 sensor driver
 *
 * Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version.
 */

//#define DEBUG
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <linux/rk-preisp.h>
#include <media/v4l2-async.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define DRIVER_VERSION		KERNEL_VERSION(0, 0x01, 0x00)

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define SC8220_NAME				"sc8220"
#define BITS_PER_SAMPLE			10
#define SC8220_MEDIA_BUS_FMT	MEDIA_BUS_FMT_SBGGR10_1X10
#define SC8220_LINK_FREQ_371M		371250000	// 742.5Mbps

#define SC8220_PIXEL_RATE		(SC8220_LINK_FREQ_371M / BITS_PER_SAMPLE * \
					2 * SC8220_LANES)

#define SC8220_XVCLK_FREQ		27000000

#define SC8220_REG_CHIP_ID_H	0x3107
#define SC8220_REG_CHIP_ID_L	0x3108
#define SC8220_CHIP_ID			0xb10a

#define SC8220_REG_EXP_LONG_H	0x3e00
#define SC8220_REG_EXP_LONG_M	0x3e01
#define SC8220_REG_EXP_LONG_L	0x3e02

#define SC8220_REG_AGAIN_H		0x3e08
#define SC8220_REG_AGAIN_L		0x3e09
#define SC8220_REG_DGAIN_H		0x3e06
#define SC8220_REG_DGAIN_L		0x3e07

#define SC8220_REG_MIRROR_FLIP	0x3221
#define SC8220_MIRROR_MASK		0x06
#define SC8220_FLIP_MASK		0x60

#define SC8220_REG_CTRL_MODE	0x0100
#define SC8220_MODE_SW_STANDBY	0x0
#define SC8220_MODE_STREAMING	BIT(0)

#define SC8220_REG_VTS_H	0x320e
#define SC8220_REG_VTS_L	0x320f

#define SC8220_VTS_MAX		0x3FFF
#define SC8220_HTS_MAX		0xFFF

#define SC8220_EXPOSURE_NORMAL_MIN  2
#define SC8220_EXPOSURE_NORMAL_STEP 1

#define SC8220_GROUP_HOLD		0x3812
#define SC8220_GROUP_HOLD_START	0x00
#define SC8220_GROUP_HOLD_LUNCH	0x30

#define SC8220_GAIN_MIN		0x40
#define SC8220_GAIN_MAX		0x8000
#define SC8220_GAIN_STEP	1
#define SC8220_GAIN_DEFAULT	0x40

#define SC8220_LANES		4

static const char * const sc8220_supply_names[] = {
	"dovdd",    /* Digital I/O power */
	"avdd",     /* Analog power */
	"dvdd",     /* Digital power */
};

#define SC8220_NUM_SUPPLIES ARRAY_SIZE(sc8220_supply_names)

#define to_sc8220(sd) container_of(sd, struct sc8220, subdev)

enum {
	PAD0,
	PAD1,
	PAD2,
	PAD3,
	PAD_MAX,
};

enum {
	LINK_FREQ_INDEX,
};

struct sc8220_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 link_freq_index;
	const struct reg_sequence *reg_list;
	u32 reg_num;
	u32 hdr_mode;
	u32 vc[PAD_MAX];
};

struct sc8220 {
	struct device	*dev;
	struct clk	*xvclk;
	struct regmap	*regmap;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;
	struct regulator_bulk_data supplies[SC8220_NUM_SUPPLIES];
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;
	struct v4l2_subdev  subdev;
	struct media_pad    pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl    *exposure;
	struct v4l2_ctrl    *anal_gain;
	struct v4l2_ctrl    *hblank;
	struct v4l2_ctrl    *vblank;
	struct v4l2_ctrl    *h_flip;
	struct v4l2_ctrl    *v_flip;
	struct v4l2_ctrl    *link_freq;
	struct v4l2_ctrl    *pixel_rate;
	struct mutex        lock;
	bool		    streaming;
	bool		    power_on;
	unsigned int        cfg_num;
	const struct sc8220_mode *cur_mode;
	u32		module_index;
	const char      *module_facing;
	const char      *module_name;
	const char      *len_name;
	bool			  has_init_exp;
};

static const struct regmap_config sc8220_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x6f00,
};

static const s64 link_freq_menu_items[] = {
	SC8220_LINK_FREQ_371M,
};

/*
 * window size=3840*2160 mipi@4lane
 * mclk=27M mipi_clk=708.75Mbps
 * pixel_line_total=xxxx line_frame_total=2256
 * row_time=29.62us frame_rate=30fps
 */
static const struct reg_sequence sc8220_3840_2160_liner_30fps_settings[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x36f9, 0x80},
	{0x301f, 0x06},
	{0x3253, 0x08},
	{0x3302, 0x0a},
	{0x3303, 0x04},
	{0x3304, 0x30},
	{0x3306, 0x34},
	{0x3308, 0x0c},
	{0x3309, 0x50},
	{0x330b, 0x90},
	{0x330d, 0x20},
	{0x330e, 0x24},
	{0x330f, 0x04},
	{0x3310, 0x01},
	{0x3314, 0x12},
	{0x331e, 0x29},
	{0x331f, 0x49},
	{0x3340, 0x04},
	{0x3352, 0x03},
	{0x3353, 0x03},
	{0x3356, 0x08},
	{0x335e, 0x01},
	{0x335f, 0x03},
	{0x3367, 0x06},
	{0x3368, 0x06},
	{0x3369, 0xc0},
	{0x336a, 0x00},
	{0x336b, 0x08},
	{0x336d, 0x03},
	{0x337c, 0x04},
	{0x337d, 0x08},
	{0x33a2, 0x06},
	{0x33ac, 0x04},
	{0x33ae, 0x0a},
	{0x33af, 0x10},
	{0x3625, 0x0a},
	{0x3632, 0x88},
	{0x3637, 0x24},
	{0x3638, 0x08},
	{0x36ea, 0x35},
	{0x36eb, 0x0d},
	{0x36ec, 0x0b},
	{0x36ed, 0x24},
	{0x36fa, 0x35},
	{0x36fb, 0x25},
	{0x36fc, 0x10},
	{0x36fd, 0x34},
	{0x3907, 0x01},
	{0x3908, 0x11},
	{0x3e01, 0x8c},
	{0x3e02, 0x60},
	{0x4800, 0x24},
	{0x4837, 0x2b},
	{0x4853, 0xff},
	{0x5000, 0x06},
	{0x550f, 0x9c},
	{0x36e9, 0x20},
	{0x36f9, 0x50},
};

static const struct sc8220_mode supported_modes[] = {
	{
		.width = 3840,
		.height = 2160,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x8c6,
		.hts_def = 0x41a * SC8220_LANES * 2,
		.vts_def = 0x8ca,
		.link_freq_index = LINK_FREQ_INDEX,
		.reg_list = sc8220_3840_2160_liner_30fps_settings,
		.reg_num = ARRAY_SIZE(sc8220_3840_2160_liner_30fps_settings),
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static inline int sc8220_read_reg(struct sc8220 *sc8220, u16 addr, u8 *value)
{
	unsigned int val;
	int ret;

	ret = regmap_read(sc8220->regmap, addr, &val);
	if (ret) {
		dev_err(sc8220->dev, "i2c read failed at addr: %x\n", addr);
		return ret;
	}
	*value = val & 0xff;
	return 0;
}

static inline int sc8220_write_reg(struct sc8220 *sc8220, u16 addr, u8 value)
{
	int ret;

	ret = regmap_write(sc8220->regmap, addr, value);
	if (ret) {
		dev_err(sc8220->dev, "i2c write failed at addr: %x\n", addr);
		return ret;
	}
	return ret;
}

static int sc8220_set_gain(struct sc8220 *sc8220, u32 a_gain)
{
	int ret = 0;
	u32 coarse_again = 0, fine_again = 0;
	u32 coarse_dgain = 0, fine_dgain = 0;

	dev_dbg(sc8220->dev, "set analog gain 0x%x\n", a_gain);
	if (a_gain < 0x80) { /*1x ~ 2x*/
		fine_again = a_gain;
		coarse_again = 0x03;
		fine_dgain = 0x80;
		coarse_dgain = 0x00;
		ret |= sc8220_write_reg(sc8220, 0x3301, 0x0a);
		ret |= sc8220_write_reg(sc8220, 0x3633, 0x42);
		ret |= sc8220_write_reg(sc8220, 0x3622, 0xe7);
		ret |= sc8220_write_reg(sc8220, 0x3630, 0x80);
		ret |= sc8220_write_reg(sc8220, 0x3628, 0x40);
		ret |= sc8220_write_reg(sc8220, 0x363a, 0x00);
	} else if (a_gain < 0x100) { /*2x ~ 4x*/
		fine_again = a_gain >> 1;
		coarse_again = 0x7;
		fine_dgain = 0x80;
		coarse_dgain = 0x00;
		ret |= sc8220_write_reg(sc8220, 0x3301, 0x0E);
		ret |= sc8220_write_reg(sc8220, 0x3633, 0x52);
		ret |= sc8220_write_reg(sc8220, 0x3622, 0xe7);
		ret |= sc8220_write_reg(sc8220, 0x3630, 0x50);
		ret |= sc8220_write_reg(sc8220, 0x3628, 0x40);
		ret |= sc8220_write_reg(sc8220, 0x363a, 0x00);
	} else if (a_gain < 0x200) { /*4x ~ 8x*/
		fine_again = a_gain >> 2;
		coarse_again = 0xf;
		fine_dgain = 0x80;
		coarse_dgain = 0x00;
		ret |= sc8220_write_reg(sc8220, 0x3301, 0x40);
		ret |= sc8220_write_reg(sc8220, 0x3633, 0x53);
		ret |= sc8220_write_reg(sc8220, 0x3622, 0xA7);
		ret |= sc8220_write_reg(sc8220, 0x3630, 0x50);
		ret |= sc8220_write_reg(sc8220, 0x3628, 0x40);
		ret |= sc8220_write_reg(sc8220, 0x363a, 0x1f);
	} else if (a_gain <= 0x3f8) { /*8x ~ 16x*/
		fine_again = a_gain >> 3;
		coarse_again = 0x1f;
		fine_dgain = 0x80;
		coarse_dgain = 0x00;
	} else if (a_gain < 0x800) { /*16 ~ 32x*/
		fine_again = 0x7f;
		coarse_again = 0x1f;
		fine_dgain = a_gain >> 3;
		coarse_dgain = 0x00;
	} else if (a_gain < 0x1000) { /*32 ~ 64*/
		fine_again = 0x7f;
		coarse_again = 0x1f;
		fine_dgain = a_gain >> 4;
		coarse_dgain = 0x01;
	} else if (a_gain < 0x2000) { /*64 ~ 128*/
		fine_again = 0x7f;
		coarse_again = 0x1f;
		fine_dgain = a_gain >> 5;
		coarse_dgain = 0x03;
	} else if (a_gain < 0x4000) { /*128 ~ 256*/
		fine_again = 0x7f;
		coarse_again = 0x1f;
		fine_dgain = a_gain >> 6;
		coarse_dgain = 0x07;
	} else if (a_gain < 0x8000) { /*256 ~ 504*/
		fine_again = 0x7f;
		coarse_again = 0x1f;
		fine_dgain = a_gain >> 7;
		coarse_dgain = 0x0f;
	}

	dev_dbg(sc8220->dev, ">>>set fine_again = 0x%x, coarse_again = 0x%x, coarse_dgain=0x%x, fine_dgain=0x%x\n",
			fine_again, coarse_again, coarse_dgain, fine_dgain);

	if (a_gain > 0x200) {
		ret |= sc8220_write_reg(sc8220, 0x3301, 0x40);
		ret |= sc8220_write_reg(sc8220, 0x3633, 0x53);
		ret |= sc8220_write_reg(sc8220, 0x3622, 0x87);
		ret |= sc8220_write_reg(sc8220, 0x3630, 0x50);
		ret |= sc8220_write_reg(sc8220, 0x3628, 0x40);
		ret |= sc8220_write_reg(sc8220, 0x363a, 0x1f);
	}
	ret |= sc8220_write_reg(sc8220, SC8220_GROUP_HOLD, SC8220_GROUP_HOLD_START);
	ret |= sc8220_write_reg(sc8220, SC8220_REG_DGAIN_H, coarse_dgain);
	ret |= sc8220_write_reg(sc8220, SC8220_REG_DGAIN_L, fine_dgain);
	ret |= sc8220_write_reg(sc8220, SC8220_REG_AGAIN_H, coarse_again);
	ret |= sc8220_write_reg(sc8220, SC8220_REG_AGAIN_L, fine_again);
	ret |= sc8220_write_reg(sc8220, SC8220_GROUP_HOLD, SC8220_GROUP_HOLD_LUNCH);

	return ret;
}

static int sc8220_set_exp(struct sc8220 *sc8220, u32 exp)
{
	int ret;

	dev_dbg(sc8220->dev, "%s: exp : %d\n", __func__, exp);
	ret  = sc8220_write_reg(sc8220, SC8220_REG_EXP_LONG_H,
					(exp >> 12) & 0xf);
	ret |= sc8220_write_reg(sc8220, SC8220_REG_EXP_LONG_M,
					(exp >> 4) & 0xff);
	ret |= sc8220_write_reg(sc8220, SC8220_REG_EXP_LONG_L,
					(exp & 0xf) << 4);
	return ret;
}

static int sc8220_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc8220 *sc8220 = container_of(ctrl->handler,
					     struct sc8220, ctrl_handler);
	s64 max;
	int ret = 0;
	u8 val = 0;
	u32 vts = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sc8220->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(sc8220->exposure,
					 sc8220->exposure->minimum, max,
					 sc8220->exposure->step,
					 sc8220->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(sc8220->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = sc8220_set_exp(sc8220, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = sc8220_set_gain(sc8220, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(sc8220->dev, "set vblank 0x%x\n", ctrl->val);
		vts = ctrl->val + sc8220->cur_mode->height;
		ret  = sc8220_write_reg(sc8220, SC8220_REG_VTS_H, vts >> 8);
		ret |= sc8220_write_reg(sc8220, SC8220_REG_VTS_L, vts & 0xff);
		break;
	case V4L2_CID_HFLIP:
		ret = sc8220_read_reg(sc8220, SC8220_REG_MIRROR_FLIP, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC8220_MIRROR_MASK;
		else
			val &= ~SC8220_MIRROR_MASK;
		ret |= sc8220_write_reg(sc8220, SC8220_REG_MIRROR_FLIP, val);
		break;
	case V4L2_CID_VFLIP:
		ret = sc8220_read_reg(sc8220, SC8220_REG_MIRROR_FLIP, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC8220_FLIP_MASK;
		else
			val &= ~SC8220_FLIP_MASK;
		ret |= sc8220_write_reg(sc8220, SC8220_REG_MIRROR_FLIP, val);
		break;
	default:
		dev_warn(sc8220->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}
	pm_runtime_put(sc8220->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sc8220_ctrl_ops = {
	.s_ctrl = sc8220_set_ctrl,
};

static int sc8220_get_regulators(struct sc8220 *sc8220)
{
	unsigned int i;

	for (i = 0; i < SC8220_NUM_SUPPLIES; i++)
		sc8220->supplies[i].supply = sc8220_supply_names[i];
	return devm_regulator_bulk_get(sc8220->dev,
				SC8220_NUM_SUPPLIES,
				sc8220->supplies);
}

static int sc8220_initialize_controls(struct sc8220 *sc8220)
{
	const struct sc8220_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	u64 pixel_rate;
	int ret;

	handler = &sc8220->ctrl_handler;
	mode = sc8220->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &sc8220->lock;
	sc8220->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
			V4L2_CID_LINK_FREQ,
			ARRAY_SIZE(link_freq_menu_items) - 1, 0,
			link_freq_menu_items);

	pixel_rate = SC8220_PIXEL_RATE;
	sc8220->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
			V4L2_CID_PIXEL_RATE,
			0, pixel_rate,
			1, pixel_rate);

	h_blank = mode->hts_def - mode->width;
	sc8220->hblank = v4l2_ctrl_new_std(handler, NULL,
			V4L2_CID_HBLANK,
			h_blank, h_blank, 1, h_blank);
	if (sc8220->hblank)
		sc8220->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;

	sc8220->vblank = v4l2_ctrl_new_std(handler, &sc8220_ctrl_ops,
			V4L2_CID_VBLANK, vblank_def,
			SC8220_VTS_MAX - mode->height,
			1, vblank_def);

	exposure_max = mode->vts_def - 4;
	sc8220->exposure = v4l2_ctrl_new_std(handler, &sc8220_ctrl_ops,
			V4L2_CID_EXPOSURE, SC8220_EXPOSURE_NORMAL_MIN,
			exposure_max, SC8220_EXPOSURE_NORMAL_STEP,
			mode->exp_def);

	sc8220->anal_gain = v4l2_ctrl_new_std(handler, &sc8220_ctrl_ops,
			V4L2_CID_ANALOGUE_GAIN, SC8220_GAIN_MIN,
			SC8220_GAIN_MAX, SC8220_GAIN_STEP,
			SC8220_GAIN_DEFAULT);

	sc8220->h_flip = v4l2_ctrl_new_std(handler, &sc8220_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	sc8220->v_flip = v4l2_ctrl_new_std(handler, &sc8220_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (handler->error) {
		ret = handler->error;
		dev_err(sc8220->dev, "Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}
	sc8220->subdev.ctrl_handler = handler;
	sc8220->has_init_exp = false;
	return 0;
err_free_handler:
	v4l2_ctrl_handler_free(handler);
	return ret;
}

static int __sc8220_power_on(struct sc8220 *sc8220)
{
	int ret;
	struct device *dev = sc8220->dev;

	if (!IS_ERR_OR_NULL(sc8220->pins_default)) {
		ret = pinctrl_select_state(sc8220->pinctrl,
					   sc8220->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(sc8220->xvclk, SC8220_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate\n");
	if (clk_get_rate(sc8220->xvclk) != SC8220_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 27MHz\n");
	ret = clk_prepare_enable(sc8220->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	ret = regulator_bulk_enable(SC8220_NUM_SUPPLIES, sc8220->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}
	if (!IS_ERR(sc8220->reset_gpio))
		gpiod_set_value_cansleep(sc8220->reset_gpio, 1);
	usleep_range(1000, 2000);
	if (!IS_ERR(sc8220->pwdn_gpio))
		gpiod_set_value_cansleep(sc8220->pwdn_gpio, 1);
	if (!IS_ERR(sc8220->reset_gpio))
		gpiod_set_value_cansleep(sc8220->reset_gpio, 0);
	usleep_range(10000, 20000);
	return 0;
disable_clk:
	clk_disable_unprepare(sc8220->xvclk);

	if (!IS_ERR_OR_NULL(sc8220->pins_sleep))
		pinctrl_select_state(sc8220->pinctrl, sc8220->pins_sleep);

	return ret;
}

static void __sc8220_power_off(struct sc8220 *sc8220)
{
	int ret;
	struct device *dev = sc8220->dev;

	if (!IS_ERR_OR_NULL(sc8220->pins_sleep)) {
		ret = pinctrl_select_state(sc8220->pinctrl,
					   sc8220->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	if (!IS_ERR(sc8220->reset_gpio))
		gpiod_set_value_cansleep(sc8220->reset_gpio, 1);
	if (!IS_ERR(sc8220->pwdn_gpio))
		gpiod_set_value_cansleep(sc8220->pwdn_gpio, 0);
	regulator_bulk_disable(SC8220_NUM_SUPPLIES, sc8220->supplies);
	clk_disable_unprepare(sc8220->xvclk);
}

static int sc8220_check_sensor_id(struct sc8220 *sc8220)
{
	u8 id_h = 0, id_l = 0;
	u16 id = 0;
	int ret = 0;

	ret = sc8220_read_reg(sc8220, SC8220_REG_CHIP_ID_H, &id_h);
	ret |= sc8220_read_reg(sc8220, SC8220_REG_CHIP_ID_L, &id_l);
	if (ret) {
		dev_err(sc8220->dev, "Failed to read sensor id, (%d)\n", ret);
		return ret;
	}
	id = id_h << 8 | id_l;
	if (id != SC8220_CHIP_ID) {
		dev_err(sc8220->dev, "sensor id: %04X mismatched\n", id);
		return -ENODEV;
	}
	dev_info(sc8220->dev, "Detected SC8220 sensor\n");
	return 0;
}

static void sc8220_get_module_inf(struct sc8220 *sc8220,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.lens, sc8220->len_name, sizeof(inf->base.lens));
	strscpy(inf->base.sensor, SC8220_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, sc8220->module_name, sizeof(inf->base.module));
}

static long sc8220_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc8220 *sc8220 = to_sc8220(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = sc8220->cur_mode->hdr_mode;
		break;
	case RKMODULE_GET_MODULE_INFO:
		sc8220_get_module_inf(sc8220, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

static int __sc8220_start_stream(struct sc8220 *sc8220)
{
	int ret;

	ret = regmap_multi_reg_write(sc8220->regmap,
				     sc8220->cur_mode->reg_list,
				     sc8220->cur_mode->reg_num);
	if (ret)
		return ret;
	__v4l2_ctrl_handler_setup(&sc8220->ctrl_handler);
	return sc8220_write_reg(sc8220, SC8220_REG_CTRL_MODE, SC8220_MODE_STREAMING);
}

static int __sc8220_stop_stream(struct sc8220 *sc8220)
{
	sc8220->has_init_exp = false;
	return sc8220_write_reg(sc8220, SC8220_REG_CTRL_MODE, SC8220_MODE_SW_STANDBY);
}

#ifdef CONFIG_COMPAT
static long sc8220_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_hdr_cfg *hdr;
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}
		ret = sc8220_ioctl(sd, cmd, inf);
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
		ret = sc8220_ioctl(sd, cmd, hdr);
		if (!ret) {
			ret = copy_to_user(up, hdr, sizeof(*hdr));
			if (ret)
				ret = -EFAULT;
		}
		kfree(hdr);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}
#endif

static int sc8220_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sc8220 *sc8220 = to_sc8220(sd);
	int ret = 0;

	mutex_lock(&sc8220->lock);
	on = !!on;
	if (on == sc8220->streaming)
		goto unlock_and_return;
	if (on) {
		ret = pm_runtime_get_sync(sc8220->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(sc8220->dev);
			goto unlock_and_return;
		}
		ret = __sc8220_start_stream(sc8220);
		if (ret) {
			dev_err(sc8220->dev, "Failed to start sc8220 stream\n");
			pm_runtime_put(sc8220->dev);
			goto unlock_and_return;
		}
	} else {
		__sc8220_stop_stream(sc8220);
		pm_runtime_put(sc8220->dev);
	}
	sc8220->streaming = on;

unlock_and_return:
	mutex_unlock(&sc8220->lock);
	return 0;
}

static int sc8220_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct sc8220 *sc8220 = to_sc8220(sd);
	const struct sc8220_mode *mode = sc8220->cur_mode;

	mutex_lock(&sc8220->lock);
	fi->interval = mode->max_fps;
	mutex_unlock(&sc8220->lock);
	return 0;
}

static int sc8220_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	struct sc8220 *sc8220 = to_sc8220(sd);

	u32 val = 1 << (SC8220_LANES - 1) | V4L2_MBUS_CSI2_CHANNEL_0 |
		  V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	config->type = V4L2_MBUS_CSI2;
	config->flags = (sc8220->cur_mode->hdr_mode == NO_HDR) ?
			val : (val | V4L2_MBUS_CSI2_CHANNEL_1);
	return 0;
}

static int sc8220_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = SC8220_MEDIA_BUS_FMT;
	return 0;
}

static int sc8220_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct sc8220 *sc8220 = to_sc8220(sd);

	if (fse->index >= sc8220->cfg_num)
		return -EINVAL;
	if (fse->code != SC8220_MEDIA_BUS_FMT)
		return -EINVAL;
	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;
	return 0;
}

static int sc8220_enum_frame_interval(struct v4l2_subdev *sd,
						  struct v4l2_subdev_pad_config *cfg,
						  struct v4l2_subdev_frame_interval_enum *fie)
{
	struct sc8220 *sc8220 = to_sc8220(sd);

	if (fie->index >= sc8220->cfg_num)
		return -EINVAL;
	fie->code = SC8220_MEDIA_BUS_FMT;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static int sc8220_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc8220 *sc8220 = to_sc8220(sd);
	const struct sc8220_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&sc8220->lock);
	mode = v4l2_find_nearest_size(supported_modes,
		ARRAY_SIZE(supported_modes),
		width, height,
		fmt->format.width, fmt->format.height);
	fmt->format.code = SC8220_MEDIA_BUS_FMT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sc8220->lock);
		return -ENOTTY;
#endif
	} else {
		sc8220->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(sc8220->link_freq, mode->link_freq_index);
		__v4l2_ctrl_s_ctrl_int64(sc8220->pixel_rate,
					 SC8220_PIXEL_RATE);
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc8220->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc8220->vblank, vblank_def,
					 SC8220_VTS_MAX - mode->height,
					 1, vblank_def);
	}
	mutex_unlock(&sc8220->lock);
	return 0;
}

#define CROP_START(SRC, DST) (((SRC) - (DST)) / 2 / 4 * 4)
#define DST_WIDTH	3840
#define DST_HEIGHT	2160

static int sc8220_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct sc8220 *sc8220 = to_sc8220(sd);

	if (sel->target == V4L2_SEL_TGT_CROP_BOUNDS) {
		sel->r.left = CROP_START(sc8220->cur_mode->width, DST_WIDTH);
		sel->r.width = DST_WIDTH;
		sel->r.top = CROP_START(sc8220->cur_mode->height, DST_HEIGHT);
		sel->r.height = DST_HEIGHT;
		return 0;
	}
	return -EINVAL;
}

static int sc8220_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc8220 *sc8220 = to_sc8220(sd);
	const struct sc8220_mode *mode = sc8220->cur_mode;

	mutex_lock(&sc8220->lock);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sc8220->lock);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = SC8220_MEDIA_BUS_FMT;
		fmt->format.field = V4L2_FIELD_NONE;
		fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&sc8220->lock);
	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sc8220_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc8220 *sc8220 = to_sc8220(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sc8220_mode *def_mode = &supported_modes[0];

	mutex_lock(&sc8220->lock);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = SC8220_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;
	mutex_unlock(&sc8220->lock);
	return 0;
}
#endif

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sc8220_internal_ops = {
	.open = sc8220_open,
};
#endif

static int sc8220_s_power(struct v4l2_subdev *sd, int on)
{
	struct sc8220 *sc8220 = to_sc8220(sd);
	int ret = 0;

	mutex_lock(&sc8220->lock);
	if (sc8220->power_on == !!on)
		goto unlock_and_return;
	if (on) {
		ret = pm_runtime_get_sync(sc8220->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(sc8220->dev);
			goto unlock_and_return;
		}
		sc8220->power_on = true;
	} else {
		pm_runtime_put(sc8220->dev);
		sc8220->power_on = false;
	}
unlock_and_return:
	mutex_unlock(&sc8220->lock);
	return ret;
}

static const struct v4l2_subdev_core_ops sc8220_core_ops = {
	.s_power = sc8220_s_power,
	.ioctl = sc8220_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc8220_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc8220_video_ops = {
	.s_stream = sc8220_s_stream,
	.g_frame_interval = sc8220_g_frame_interval,
	.g_mbus_config = sc8220_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops sc8220_pad_ops = {
	.enum_mbus_code = sc8220_enum_mbus_code,
	.enum_frame_size = sc8220_enum_frame_sizes,
	.enum_frame_interval = sc8220_enum_frame_interval,
	.get_fmt = sc8220_get_fmt,
	.set_fmt = sc8220_set_fmt,
	.get_selection = sc8220_get_selection,
};

static const struct v4l2_subdev_ops sc8220_subdev_ops = {
	.core   = &sc8220_core_ops,
	.video  = &sc8220_video_ops,
	.pad    = &sc8220_pad_ops,
};

static int sc8220_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc8220 *sc8220 = to_sc8220(sd);

	__sc8220_power_on(sc8220);
	return 0;
}

static int sc8220_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc8220 *sc8220 = to_sc8220(sd);

	__sc8220_power_off(sc8220);
	return 0;
}

static const struct dev_pm_ops sc8220_pm_ops = {
	SET_RUNTIME_PM_OPS(sc8220_runtime_suspend,
			   sc8220_runtime_resume, NULL)
};

static int sc8220_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sc8220 *sc8220;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);
	sc8220 = devm_kzalloc(dev, sizeof(*sc8220), GFP_KERNEL);
	if (!sc8220)
		return -ENOMEM;
	sc8220->dev = dev;
	sc8220->regmap = devm_regmap_init_i2c(client, &sc8220_regmap_config);
	if (IS_ERR(sc8220->regmap)) {
		dev_err(dev, "Failed to initialize I2C\n");
		return -ENODEV;
	}
	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sc8220->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sc8220->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sc8220->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sc8220->len_name);
	if (ret) {
		dev_err(dev, "Failed to get module information\n");
		return -EINVAL;
	}
	sc8220->xvclk = devm_clk_get(sc8220->dev, "xvclk");
	if (IS_ERR(sc8220->xvclk)) {
		dev_err(sc8220->dev, "Failed to get xvclk\n");
		return -EINVAL;
	}
	sc8220->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sc8220->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");
	sc8220->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_HIGH);
	if (IS_ERR(sc8220->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");
	ret = sc8220_get_regulators(sc8220);
	if (ret) {
		dev_err(dev, "Failed to get regulators\n");
		return ret;
	}
	sc8220->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(sc8220->pinctrl)) {
		sc8220->pins_default =
			pinctrl_lookup_state(sc8220->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sc8220->pins_default))
			dev_info(dev, "could not get default pinstate\n");

		sc8220->pins_sleep =
			pinctrl_lookup_state(sc8220->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(sc8220->pins_sleep))
			dev_info(dev, "could not get sleep pinstate\n");
	} else {
		dev_info(dev, "no pinctrl\n");
	}
	mutex_init(&sc8220->lock);
	/* set default mode */
	sc8220->cur_mode = &supported_modes[0];
	sc8220->cfg_num = ARRAY_SIZE(supported_modes);
	sd = &sc8220->subdev;
	ret = __sc8220_power_on(sc8220);
	if (ret)
		goto err_free_handler;
	ret = sc8220_check_sensor_id(sc8220);
	if (ret)
		goto err_power_off;
	v4l2_i2c_subdev_init(sd, client, &sc8220_subdev_ops);
	ret = sc8220_initialize_controls(sc8220);
	if (ret)
		goto err_destroy_mutex;
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sc8220_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#ifdef CONFIG_MEDIA_CONTROLLER
	sc8220->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sc8220->pad);
	if (ret < 0)
		goto err_power_off;
#endif
	memset(facing, 0, sizeof(facing));
	if (strcmp(sc8220->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';
	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sc8220->module_index, facing,
		 SC8220_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "Failed to register v4l2 async subdev\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;
err_clean_entity:
#ifdef CONFIG_MEDIA_CONTROLLER
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__sc8220_power_off(sc8220);
err_free_handler:
	v4l2_ctrl_handler_free(&sc8220->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sc8220->lock);
	return ret;
}

static int sc8220_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc8220 *sc8220 = to_sc8220(sd);

	v4l2_async_unregister_subdev(sd);
#ifdef CONFIG_MEDIA_CONTROLLER
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sc8220->ctrl_handler);
	mutex_destroy(&sc8220->lock);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sc8220_power_off(sc8220);
	pm_runtime_set_suspended(&client->dev);
	return 0;
}

static const struct i2c_device_id sc8220_match_id[] = {
	{ "smartsens,sc8220", 0 },
	{ },
};

static const struct of_device_id sc8220_of_match[] = {
	{ .compatible = "smartsens,sc8220" },
	{},
};

MODULE_DEVICE_TABLE(of, sc8220_of_match);

static struct i2c_driver sc8220_i2c_driver = {
	.driver = {
		.name = SC8220_NAME,
		.pm = &sc8220_pm_ops,
		.of_match_table = of_match_ptr(sc8220_of_match),
	},
	.probe      = &sc8220_probe,
	.remove     = &sc8220_remove,
	.id_table   = sc8220_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sc8220_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sc8220_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Smartsens sc8220 Image Sensor driver");
MODULE_LICENSE("GPL v2");
