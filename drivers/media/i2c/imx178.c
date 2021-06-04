// SPDX-License-Identifier: GPL-2.0
/*
 * imx178 sensor driver
 *
 * Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version.
 * V0.0X01.0X01 fix some errors for exposure and gain.
 * 1.fix vts_def/hts_def wrong value;
 * 2.fix gain wrong value;
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

#define DRIVER_VERSION		KERNEL_VERSION(0, 0x01, 0x01)

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define IMX178_NAME				"imx178"
#define IMX178_MEDIA_BUS_FMT	MEDIA_BUS_FMT_SRGGB10_1X10
#define MIPI_FREQ				594000000
#define IMX178_XVCLK_FREQ		37125000
#define BITS_PER_SAMPLE			10

#define IMX178_REG_CHIP_ID		0x33be
#define IMX178_CHIP_ID			0xb10a

#define IMX178_REG_SHS_H		0x3036
#define IMX178_REG_SHS_M		0x3035
#define IMX178_REG_SHS_L		0x3034

#define IMX178_GAIN_MIN			0x00
#define IMX178_GAIN_MAX			0x1e0

#define IMX178_GAIN_REG_H		0x3020
#define IMX178_GAIN_REG_L		0x301f

#define IMX178_REG_MIRROR_FLIP	0x300f
#define IMX178_FETCH_MIRROR(VAL, ENABLE)	(ENABLE ? VAL | 0x02 : VAL & 0xfd)
#define IMX178_FETCH_FLIP(VAL, ENABLE)		(ENABLE ? VAL | 0x01 : VAL & 0xfe)

#define IMX178_REG_CTRL_MODE	0x3000
#define IMX178_REG_START_MODE	0x3008
#define IMX178_MODE_SW_STANDBY	0x0
#define IMX178_MODE_STREAMING	1

#define IMX178_REG_HOLD		0x3007
#define IMX178_REG_HOLD_START	0x01
#define IMX178_REG_HOLD_LUNCH	0X00

#define IMX178_REG_VMAX_H	0x302e
#define IMX178_REG_VMAX_M	0x302d
#define IMX178_REG_VMAX_L	0x302c

#define IMX178_VTS_MAX		0x3FFF
#define IMX178_HTS_MAX		0x0FFF

#define IMX178_EXPOSURE_NORMAL_MAX  0x118A
#define IMX178_EXPOSURE_NORMAL_MIN  3
#define IMX178_EXPOSURE_NORMAL_STEP 1

#define IMX178_GAIN_STEP	1
#define IMX178_GAIN_DEFAULT	0x40

#define IMX178_LANES		4

static const char * const imx178_supply_names[] = {
	"dovdd",    /* Digital I/O power */
	"avdd",     /* Analog power */
	"dvdd",     /* Digital power */
};

#define IMX178_NUM_SUPPLIES ARRAY_SIZE(imx178_supply_names)

#define to_imx178(sd) container_of(sd, struct imx178, subdev)

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

struct imx178_mode {
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
	struct rkmodule_lvds_cfg lvds_cfg;
};

struct imx178 {
	struct device	*dev;
	struct clk	*xvclk;
	struct regmap	*regmap;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;
	struct regulator_bulk_data supplies[IMX178_NUM_SUPPLIES];
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
	struct v4l2_ctrl    *link_freq;
	struct v4l2_ctrl    *pixel_rate;
	struct mutex        lock;
	bool		    streaming;
	bool		    power_on;
	unsigned int        cfg_num;
	const struct imx178_mode *cur_mode;
	u32		module_index;
	const char      *module_facing;
	const char      *module_name;
	const char      *len_name;
	bool		has_init_exp;
	u32		cur_vts;
};

static const struct regmap_config imx178_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0x6f00,
};

static const s64 link_freq_menu_items[] = {
	MIPI_FREQ,
};

/*
 * window size=3072*1728 mipi@4lane
 * mclk=37.125M mipi_clk=708.75Mbps
 * pixel_line_total=999 line_frame_total=2500
 * row_time=29.62us frame_rate=30fps
 */
static const struct reg_sequence imx178_3072_1728_liner_30fps_settings[] = {
	{0x3000, 0x07},
	{0x300E, 0x00},
	{0x300F, 0x03},
	{0x3010, 0x00},
	{0x3066, 0x06},
	{0x302C, 0xC4},
	{0x302D, 0x09},
	{0x302E, 0x00},
	{0x302F, 0xDE},
	{0x3030, 0x03},
	{0x300D, 0x04},
	{0x3059, 0x30},
	{0x3004, 0x03},
	{0x3101, 0x30},
	{0x310C, 0x00},
	{0x33BE, 0x21},
	{0x33BF, 0x21},
	{0x33C0, 0x2C},
	{0x33C1, 0x2C},
	{0x33C2, 0x21},
	{0x33C3, 0x2C},
	{0x33C4, 0x2C},
	{0x33C5, 0x00},
	{0x311C, 0x34},
	{0x311D, 0x28},
	{0x311E, 0xAB},
	{0x311F, 0x00},
	{0x3120, 0x95},
	{0x3121, 0x00},
	{0x3122, 0xB4},
	{0x3123, 0x00},
	{0x3124, 0x8c},
	{0x3125, 0x02},
	{0x312D, 0x03},
	{0x312E, 0x0C},
	{0x312F, 0x28},
	{0x3131, 0x2D},
	{0x3132, 0x00},
	{0x3133, 0xB4},
	{0x3134, 0x00},
	{0x3137, 0x50},
	{0x3138, 0x08},
	{0x3139, 0x00},
	{0x313A, 0x07},
	{0x313D, 0x05},
	{0x3140, 0x06},
	{0x3220, 0x8B},
	{0x3221, 0x00},
	{0x3222, 0x74},
	{0x3223, 0x00},
	{0x3226, 0xC2},
	{0x3227, 0x00},
	{0x32A9, 0x1B},
	{0x32AA, 0x00},
	{0x32B3, 0x0E},
	{0x32B4, 0x00},
	{0x33D6, 0x16},
	{0x33D7, 0x15},
	{0x33D8, 0x14},
	{0x33D9, 0x10},
	{0x33DA, 0x08},
	{0x3011, 0x00},
	{0x301B, 0x00},
	{0x3037, 0x08},
	{0x3038, 0x00},
	{0x3039, 0x00},
	{0x30AD, 0x49},
	{0x30AF, 0x54},
	{0x30B0, 0x33},
	{0x30B3, 0x0A},
	{0x30C4, 0x30},
	{0x3103, 0x03},
	{0x3104, 0x08},
	{0x3107, 0x10},
	{0x310F, 0x01},
	{0x32E5, 0x06},
	{0x32E6, 0x00},
	{0x32E7, 0x1F},
	{0x32E8, 0x00},
	{0x32E9, 0x00},
	{0x32EA, 0x00},
	{0x32EB, 0x00},
	{0x32EC, 0x00},
	{0x32EE, 0x00},
	{0x32F2, 0x02},
	{0x32F4, 0x00},
	{0x32F5, 0x00},
	{0x32F6, 0x00},
	{0x32F7, 0x00},
	{0x32F8, 0x00},
	{0x32FC, 0x02},
	{0x3310, 0x11},
	{0x3338, 0x81},
	{0x333D, 0x00},
	{0x3362, 0x00},
	{0x336B, 0x02},
	{0x336E, 0x11},
	{0x33B4, 0xFE},
	{0x33B5, 0x06},
	{0x33B9, 0x00},
	{0x3034, 0x08},
	{0x3035, 0x00},
	{0x301F, 0xA0},
	{0x3020, 0x00},
	{0x305E, 0x0A},
	{0x3015, 0x32},
};

static const struct imx178_mode supported_modes[] = {
	{
		.width = 3096,
		.height = 1774,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0008,
		.hts_def = 0x03de * 4,
		.vts_def = 0x09c4,
		.link_freq_index = LINK_FREQ_INDEX,
		.reg_list = imx178_3072_1728_liner_30fps_settings,
		.reg_num = ARRAY_SIZE(imx178_3072_1728_liner_30fps_settings),
		.hdr_mode = NO_HDR,
		.lvds_cfg = {
			.mode = LS_FIRST,
			.frm_sync_code[LVDS_CODE_GRP_LINEAR] = {
					.odd_sync_code = {
						.act = {
							.sav = 0x200,
							.eav = 0x274,
						},
						.blk = {
							.sav = 0x2ac,
							.eav = 0x2d8,
						},
					},
				},
			},
	}
};

/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
static u64 to_pixel_rate(u32 index)
{
	u64 pixel_rate = link_freq_menu_items[index] * 2 * IMX178_LANES / BITS_PER_SAMPLE;

	do_div(pixel_rate, 10);
	return pixel_rate;
}

static inline int imx178_read_reg(struct imx178 *imx178, u16 addr, u8 *value)
{
	unsigned int val;
	int ret;

	ret = regmap_read(imx178->regmap, addr, &val);
	if (ret) {
		dev_err(imx178->dev, "i2c read failed at addr: %x\n", addr);
		return ret;
	}
	*value = val & 0xff;
	return 0;
}

static inline int imx178_write_reg(struct imx178 *imx178, u16 addr, u8 value)
{
	int ret;

	ret = regmap_write(imx178->regmap, addr, value);
	if (ret) {
		dev_err(imx178->dev, "i2c write failed at addr: %x\n", addr);
		return ret;
	}
	return ret;
}

static int imx178_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx178 *imx178 = container_of(ctrl->handler,
					     struct imx178, ctrl_handler);
	const struct imx178_mode *mode = imx178->cur_mode;
	s64 max;
	int ret = 0;
	u8 val = 0;
	u32 vts = 0, shr0 = 0, again = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = mode->height + ctrl->val;
		__v4l2_ctrl_modify_range(imx178->exposure,
					 imx178->exposure->minimum, max,
					 imx178->exposure->step,
					 imx178->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(imx178->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_info(imx178->dev, "set exposure 0x%x", ctrl->val);
		shr0 = imx178->cur_vts - ctrl->val;
		ret  = imx178_write_reg(imx178, IMX178_REG_HOLD, IMX178_REG_HOLD_START);
		ret |= imx178_write_reg(imx178, IMX178_REG_SHS_H, (shr0 >> 16) & 0x1);
		ret |= imx178_write_reg(imx178, IMX178_REG_SHS_M, (shr0 >> 8) & 0xff);
		ret |= imx178_write_reg(imx178, IMX178_REG_SHS_L, (shr0 & 0xff));
		ret |= imx178_write_reg(imx178, IMX178_REG_HOLD, IMX178_REG_HOLD_LUNCH);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_info(imx178->dev, "set anal_gain 0x%x", ctrl->val);
		again = ctrl->val * 3;
		ret  = imx178_write_reg(imx178, IMX178_REG_HOLD, IMX178_REG_HOLD_START);
		ret |= imx178_write_reg(imx178, IMX178_GAIN_REG_H, (again >> 8) & 0x01);
		ret |= imx178_write_reg(imx178, IMX178_GAIN_REG_L, again & 0xff);
		ret |= imx178_write_reg(imx178, IMX178_REG_HOLD, IMX178_REG_HOLD_LUNCH);
		break;
	case V4L2_CID_VBLANK:
		dev_info(imx178->dev, "set vblank 0x%x", ctrl->val);
		vts = ctrl->val + mode->height;
		ret = imx178_write_reg(imx178, IMX178_REG_VMAX_H, vts >> 16);
		ret |= imx178_write_reg(imx178, IMX178_REG_VMAX_M, vts >> 8);
		ret |= imx178_write_reg(imx178, IMX178_REG_VMAX_L, vts & 0xff);
		break;
	case V4L2_CID_HFLIP:
		ret = imx178_read_reg(imx178, IMX178_REG_MIRROR_FLIP, &val);
		ret |= imx178_write_reg(imx178, IMX178_REG_MIRROR_FLIP,
					IMX178_FETCH_MIRROR(val, ctrl->val));
		break;
	case V4L2_CID_VFLIP:
		ret = imx178_read_reg(imx178, IMX178_REG_MIRROR_FLIP, &val);
		ret |= imx178_write_reg(imx178, IMX178_REG_MIRROR_FLIP,
					IMX178_FETCH_FLIP(val, ctrl->val));
		break;
	default:
		dev_warn(imx178->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}
	pm_runtime_put(imx178->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx178_ctrl_ops = {
	.s_ctrl = imx178_set_ctrl,
};

static int imx178_get_regulators(struct imx178 *imx178)
{
	unsigned int i;

	for (i = 0; i < IMX178_NUM_SUPPLIES; i++)
		imx178->supplies[i].supply = imx178_supply_names[i];

	return devm_regulator_bulk_get(imx178->dev, IMX178_NUM_SUPPLIES, imx178->supplies);
}

static int imx178_initialize_controls(struct imx178 *imx178)
{
	const struct imx178_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &imx178->ctrl_handler;
	mode = imx178->cur_mode;

	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;

	handler->lock = &imx178->lock;
	imx178->link_freq = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(link_freq_menu_items) - 1, 0,
						   link_freq_menu_items);
	imx178->pixel_rate = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
					       0, to_pixel_rate(LINK_FREQ_INDEX),
					       1, to_pixel_rate(LINK_FREQ_INDEX));
	h_blank = mode->hts_def - mode->width;
	imx178->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (imx178->hblank)
		imx178->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	vblank_def = mode->vts_def - mode->height;
	imx178->vblank = v4l2_ctrl_new_std(handler, &imx178_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   IMX178_VTS_MAX - mode->height,
					   1, vblank_def);
	imx178->cur_vts = mode->vts_def;
	exposure_max =  mode->vts_def - 1;
	imx178->exposure = v4l2_ctrl_new_std(handler, &imx178_ctrl_ops,
					     V4L2_CID_EXPOSURE, IMX178_EXPOSURE_NORMAL_MIN,
					     exposure_max, IMX178_EXPOSURE_NORMAL_STEP,
					     mode->exp_def);
	imx178->anal_gain = v4l2_ctrl_new_std(handler, &imx178_ctrl_ops,
					      V4L2_CID_ANALOGUE_GAIN, IMX178_GAIN_MIN,
					      IMX178_GAIN_MAX, IMX178_GAIN_STEP,
					      IMX178_GAIN_DEFAULT);
	v4l2_ctrl_new_std(handler, &imx178_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &imx178_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		dev_err(imx178->dev, "Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}
	imx178->subdev.ctrl_handler = handler;
	imx178->has_init_exp = false;
	return 0;
err_free_handler:
	v4l2_ctrl_handler_free(handler);
	return ret;
}

static int __imx178_power_on(struct imx178 *imx178)
{
	int ret;
	struct device *dev = imx178->dev;

	if (!IS_ERR_OR_NULL(imx178->pins_default)) {
		ret = pinctrl_select_state(imx178->pinctrl, imx178->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(imx178->xvclk, IMX178_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate\n");
	if (clk_get_rate(imx178->xvclk) != IMX178_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 27MHz\n");
	ret = clk_prepare_enable(imx178->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	ret = regulator_bulk_enable(IMX178_NUM_SUPPLIES, imx178->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}
	if (!IS_ERR(imx178->reset_gpio))
		gpiod_set_value_cansleep(imx178->reset_gpio, 1);
	usleep_range(1000, 2000);
	if (!IS_ERR(imx178->pwdn_gpio))
		gpiod_set_value_cansleep(imx178->pwdn_gpio, 1);
	if (!IS_ERR(imx178->reset_gpio))
		gpiod_set_value_cansleep(imx178->reset_gpio, 0);
	usleep_range(10000, 20000);
	return 0;

disable_clk:
	clk_disable_unprepare(imx178->xvclk);

	if (!IS_ERR_OR_NULL(imx178->pins_sleep))
		pinctrl_select_state(imx178->pinctrl, imx178->pins_sleep);

	return ret;
}

static void __imx178_power_off(struct imx178 *imx178)
{
	int ret;
	struct device *dev = imx178->dev;

	if (!IS_ERR_OR_NULL(imx178->pins_sleep)) {
		ret = pinctrl_select_state(imx178->pinctrl,
					   imx178->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	if (!IS_ERR(imx178->reset_gpio))
		gpiod_set_value_cansleep(imx178->reset_gpio, 1);
	if (!IS_ERR(imx178->pwdn_gpio))
		gpiod_set_value_cansleep(imx178->pwdn_gpio, 0);
	regulator_bulk_disable(IMX178_NUM_SUPPLIES, imx178->supplies);
	clk_disable_unprepare(imx178->xvclk);
}

static int imx178_check_sensor_id(struct imx178 *imx178)
{
	u8 id_h = 0;
	int ret = 0;

	ret = imx178_read_reg(imx178, IMX178_REG_CHIP_ID, &id_h);
	if (ret) {
		dev_err(imx178->dev, "Failed to read sensor id, (%d)\n", ret);
		return ret;
	}
	dev_info(imx178->dev, "Detected imx178 sensor\n");

	return 0;
}

static void imx178_get_module_inf(struct imx178 *imx178,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.lens, imx178->len_name, sizeof(inf->base.lens));
	strscpy(inf->base.sensor, IMX178_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, imx178->module_name, sizeof(inf->base.module));
}

static long imx178_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct imx178 *imx178 = to_imx178(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;
	struct rkmodule_lvds_cfg *lvds_cfg;
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = imx178->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		if (hdr_cfg->hdr_mode != 0)
			ret = -1;
		break;
	case RKMODULE_GET_MODULE_INFO:
		imx178_get_module_inf(imx178, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_LVDS_CFG:
		lvds_cfg = (struct rkmodule_lvds_cfg *)arg;
		memcpy(lvds_cfg, &imx178->cur_mode->lvds_cfg, sizeof(struct rkmodule_lvds_cfg));
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

static int __imx178_start_stream(struct imx178 *imx178)
{
	int ret;
	u8 mode = 1;

	ret = regmap_multi_reg_write(imx178->regmap,
				     imx178->cur_mode->reg_list,
				     imx178->cur_mode->reg_num);
	if (ret)
		return ret;
	__v4l2_ctrl_handler_setup(&imx178->ctrl_handler);

	imx178_read_reg(imx178, IMX178_REG_CTRL_MODE, &mode);

	usleep_range(1000, 2000);
	imx178_write_reg(imx178, IMX178_REG_CTRL_MODE, IMX178_MODE_SW_STANDBY);

	imx178_read_reg(imx178, IMX178_REG_CTRL_MODE, &mode);

	usleep_range(1000, 2000);
	imx178_write_reg(imx178, IMX178_REG_START_MODE, 0x0);

	return 0;
}

static int __imx178_stop_stream(struct imx178 *imx178)
{
	imx178->has_init_exp = false;
	return imx178_write_reg(imx178, IMX178_REG_CTRL_MODE, 0x7);
}

#ifdef CONFIG_COMPAT
static long imx178_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_hdr_cfg *hdr;
	struct rkmodule_lvds_cfg *lvds_cfg;
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx178_ioctl(sd, cmd, inf);
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

		ret = imx178_ioctl(sd, cmd, hdr);
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

		if (copy_from_user(hdr, up, sizeof(*hdr)))
			return -EFAULT;

		ret = imx178_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;
	case RKMODULE_GET_LVDS_CFG:
		lvds_cfg = kzalloc(sizeof(*lvds_cfg), GFP_KERNEL);
		if (!lvds_cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx178_ioctl(sd, cmd, lvds_cfg);
		if (!ret) {
			ret = copy_to_user(up, lvds_cfg, sizeof(*lvds_cfg));
			if (ret)
				ret = -EFAULT;
		}
		kfree(lvds_cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}
#endif

static int imx178_s_stream(struct v4l2_subdev *sd, int on)
{
	struct imx178 *imx178 = to_imx178(sd);
	int ret = 0;

	mutex_lock(&imx178->lock);
	on = !!on;
	if (on == imx178->streaming)
		goto unlock_and_return;
	if (on) {
		ret = pm_runtime_get_sync(imx178->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(imx178->dev);
			goto unlock_and_return;
		}
		ret = __imx178_start_stream(imx178);
		if (ret) {
			dev_err(imx178->dev, "Failed to start imx178 stream\n");
			pm_runtime_put(imx178->dev);
			goto unlock_and_return;
		}
	} else {
		__imx178_stop_stream(imx178);
		pm_runtime_put(imx178->dev);
	}
	imx178->streaming = on;

unlock_and_return:
	mutex_unlock(&imx178->lock);
	return 0;
}

static int imx178_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct imx178 *imx178 = to_imx178(sd);
	const struct imx178_mode *mode = imx178->cur_mode;

	mutex_lock(&imx178->lock);
	fi->interval = mode->max_fps;
	mutex_unlock(&imx178->lock);
	return 0;
}

static int imx178_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	struct imx178 *imx178 = to_imx178(sd);

	u32 val = 1 << (IMX178_LANES - 1) | V4L2_MBUS_CSI2_CHANNEL_0 |
		  V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	config->type = V4L2_MBUS_CCP2;
	config->flags = (imx178->cur_mode->hdr_mode == NO_HDR) ?
			val : (val | V4L2_MBUS_CSI2_CHANNEL_1);
	return 0;
}

static int imx178_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = IMX178_MEDIA_BUS_FMT;
	return 0;
}

static int imx178_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx178 *imx178 = to_imx178(sd);

	if (fse->index >= imx178->cfg_num)
		return -EINVAL;
	if (fse->code != IMX178_MEDIA_BUS_FMT)
		return -EINVAL;
	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;
	return 0;
}

static int imx178_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	struct imx178 *imx178 = to_imx178(sd);

	if (fie->index >= imx178->cfg_num)
		return -EINVAL;
	fie->code = IMX178_MEDIA_BUS_FMT;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static int imx178_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct imx178 *imx178 = to_imx178(sd);
	const struct imx178_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&imx178->lock);
	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);
	fmt->format.code = IMX178_MEDIA_BUS_FMT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&imx178->lock);
		return -ENOTTY;
#endif
	} else {
		imx178->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(imx178->link_freq, mode->link_freq_index);
		__v4l2_ctrl_s_ctrl_int64(imx178->pixel_rate, to_pixel_rate(mode->link_freq_index));
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(imx178->hblank, h_blank, h_blank, 1, h_blank);
		imx178->cur_vts = mode->vts_def;
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(imx178->vblank, vblank_def,
					 IMX178_VTS_MAX - mode->height, 1, vblank_def);
	}
	mutex_unlock(&imx178->lock);
	return 0;
}

#define CROP_START(SRC, DST) (((SRC) - (DST)) / 2 / 4 * 4)
#define DST_WIDTH	3072
#define DST_HEIGHT	1728

static int imx178_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct imx178 *imx178 = to_imx178(sd);

	if (sel->target == V4L2_SEL_TGT_CROP_BOUNDS) {
		sel->r.left = CROP_START(imx178->cur_mode->width, DST_WIDTH);
		sel->r.width = DST_WIDTH;
		sel->r.top = 21;
		sel->r.top = CROP_START(imx178->cur_mode->height, DST_HEIGHT);
		sel->r.height = DST_HEIGHT;
		return 0;
	}
	return -EINVAL;
}

static int imx178_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct imx178 *imx178 = to_imx178(sd);
	const struct imx178_mode *mode = imx178->cur_mode;

	mutex_lock(&imx178->lock);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&imx178->lock);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = IMX178_MEDIA_BUS_FMT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&imx178->lock);
	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int imx178_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx178 *imx178 = to_imx178(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct imx178_mode *def_mode = &supported_modes[0];

	mutex_lock(&imx178->lock);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = IMX178_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;
	mutex_unlock(&imx178->lock);
	return 0;
}
#endif

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops imx178_internal_ops = {
	.open = imx178_open,
};
#endif

static int imx178_s_power(struct v4l2_subdev *sd, int on)
{
	struct imx178 *imx178 = to_imx178(sd);
	int ret = 0;

	mutex_lock(&imx178->lock);
	if (imx178->power_on == !!on)
		goto unlock_and_return;
	if (on) {
		ret = pm_runtime_get_sync(imx178->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(imx178->dev);
			goto unlock_and_return;
		}
		imx178->power_on = true;
	} else {
		pm_runtime_put(imx178->dev);
		imx178->power_on = false;
	}
unlock_and_return:
	mutex_unlock(&imx178->lock);
	return ret;
}

static const struct v4l2_subdev_core_ops imx178_core_ops = {
	.s_power = imx178_s_power,
	.ioctl = imx178_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx178_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops imx178_video_ops = {
	.s_stream = imx178_s_stream,
	.g_frame_interval = imx178_g_frame_interval,
	.g_mbus_config = imx178_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops imx178_pad_ops = {
	.enum_mbus_code = imx178_enum_mbus_code,
	.enum_frame_size = imx178_enum_frame_sizes,
	.enum_frame_interval = imx178_enum_frame_interval,
	.get_fmt = imx178_get_fmt,
	.set_fmt = imx178_set_fmt,
	.get_selection = imx178_get_selection,
};

static const struct v4l2_subdev_ops imx178_subdev_ops = {
	.core   = &imx178_core_ops,
	.video  = &imx178_video_ops,
	.pad    = &imx178_pad_ops,
};

static int imx178_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx178 *imx178 = to_imx178(sd);

	__imx178_power_on(imx178);
	return 0;
}

static int imx178_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx178 *imx178 = to_imx178(sd);

	__imx178_power_off(imx178);
	return 0;
}

static const struct dev_pm_ops imx178_pm_ops = {
	SET_RUNTIME_PM_OPS(imx178_runtime_suspend,
			   imx178_runtime_resume, NULL)
};

static int imx178_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct imx178 *imx178;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);
	imx178 = devm_kzalloc(dev, sizeof(*imx178), GFP_KERNEL);
	if (!imx178)
		return -ENOMEM;
	imx178->dev = dev;
	imx178->regmap = devm_regmap_init_i2c(client, &imx178_regmap_config);
	if (IS_ERR(imx178->regmap)) {
		dev_err(dev, "Failed to initialize I2C\n");
		return -ENODEV;
	}
	ret  = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
			&imx178->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
			&imx178->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
			&imx178->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
			&imx178->len_name);
	if (ret) {
		dev_err(dev, "Failed to get module information\n");
		return -EINVAL;
	}
	imx178->xvclk = devm_clk_get(imx178->dev, "xvclk");
	if (IS_ERR(imx178->xvclk)) {
		dev_err(imx178->dev, "Failed to get xvclk\n");
		return -EINVAL;
	}
	imx178->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(imx178->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");
	imx178->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_HIGH);
	if (IS_ERR(imx178->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");
	ret = imx178_get_regulators(imx178);
	if (ret) {
		dev_err(dev, "Failed to get regulators\n");
		return ret;
	}

	imx178->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(imx178->pinctrl)) {
		imx178->pins_default = pinctrl_lookup_state(imx178->pinctrl,
				OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(imx178->pins_default))
			dev_info(dev, "could not get default pinstate\n");

		imx178->pins_sleep = pinctrl_lookup_state(imx178->pinctrl,
				OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(imx178->pins_sleep))
			dev_info(dev, "could not get sleep pinstate\n");
	} else {
		dev_info(dev, "no pinctrl\n");
	}

	mutex_init(&imx178->lock);
	/* set default mode */
	imx178->cur_mode = &supported_modes[0];
	imx178->cfg_num = ARRAY_SIZE(supported_modes);
	sd = &imx178->subdev;

	ret = __imx178_power_on(imx178);
	if (ret)
		goto err_free_handler;

	ret = imx178_check_sensor_id(imx178);
	if (ret)
		goto err_power_off;

	v4l2_i2c_subdev_init(sd, client, &imx178_subdev_ops);
	ret = imx178_initialize_controls(imx178);
	if (ret)
		goto err_destroy_mutex;
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &imx178_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#ifdef CONFIG_MEDIA_CONTROLLER
	imx178->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &imx178->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(imx178->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';
	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 imx178->module_index, facing, IMX178_NAME, dev_name(sd->dev));
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
	__imx178_power_off(imx178);
err_free_handler:
	v4l2_ctrl_handler_free(&imx178->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&imx178->lock);
	return ret;
}

static int imx178_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx178 *imx178 = to_imx178(sd);

	v4l2_async_unregister_subdev(sd);
#ifdef CONFIG_MEDIA_CONTROLLER
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&imx178->ctrl_handler);
	mutex_destroy(&imx178->lock);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__imx178_power_off(imx178);
	pm_runtime_set_suspended(&client->dev);
	return 0;
}

static const struct i2c_device_id imx178_match_id[] = {
	{ "sony,imx178", 0 },
	{ },
};

static const struct of_device_id imx178_of_match[] = {
	{ .compatible = "sony,imx178" },
	{},
};

MODULE_DEVICE_TABLE(of, imx178_of_match);

static struct i2c_driver imx178_i2c_driver = {
	.driver = {
		.name = IMX178_NAME,
		.pm = &imx178_pm_ops,
		.of_match_table = of_match_ptr(imx178_of_match),
	},
	.probe      = &imx178_probe,
	.remove     = &imx178_remove,
	.id_table   = imx178_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&imx178_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&imx178_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Sony imx178 Image Sensor Driver");
MODULE_LICENSE("GPL v2");
