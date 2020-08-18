// SPDX-License-Identifier: GPL-2.0
/*
 * s5kgm1sp driver
 *
 * Copyright (C) 2017 Rockchip Electronics Co., Ltd.
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

#define S5KGM1SP_LANES			4
#define S5KGM1SP_BITS_PER_SAMPLE	10

#define S5KGM1SP_LINK_FREQ		900000000   //4000*3000

/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define S5KGM1SP_PIXEL_RATE		(S5KGM1SP_LINK_FREQ / 10 * 2 * 4)

#define S5KGM1SP_XVCLK_FREQ		 24000000

#define CHIP_ID				0x5FB0
#define S5KGM1SP_REG_CHIP_ID		0x0042	//read only reg

#define S5KGM1SP_REG_CTRL_MODE		0x0100
#define S5KGM1SP_MODE_SW_STANDBY	0x00
#define S5KGM1SP_MODE_STREAMING		0x0100

#define S5KGM1SP_REG_EXPOSURE		0x0202
#define	S5KGM1SP_EXPOSURE_MIN		2
#define	S5KGM1SP_EXPOSURE_STEP		1
#define S5KGM1SP_VTS_MAX		0xfffc

#define S5KGM1SP_REG_GAIN		0x0204
#define S5KGM1SP_GAIN_MIN		0x0020
#define S5KGM1SP_GAIN_MAX		0xffff
#define S5KGM1SP_GAIN_STEP		1
#define S5KGM1SP_GAIN_DEFAULT		500

#define S5KGM1SP_REG_TEST_PATTERN	0x0600
#define S5KGM1SP_TEST_PATTERN_ENABLE	0x1
#define S5KGM1SP_TEST_PATTERN_DISABLE	0x0

#define S5KGM1SP_REG_VTS		0x0340

#define REG_NULL			0xFFFF
#define REG_DELAY			0xFFFE

#define S5KGM1SP_REG_VALUE_08BIT	1
#define S5KGM1SP_REG_VALUE_16BIT	2
#define S5KGM1SP_REG_VALUE_24BIT	3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"
#define S5KGM1SP_NAME			"s5kgm1sp"

static const char * const s5kgm1sp_supply_names[] = {
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
	"avdd",		/* Analog power */
};

#define S5KGM1SP_NUM_SUPPLIES ARRAY_SIZE(s5kgm1sp_supply_names)

enum s5kgm1sp_max_pad {
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

struct s5kgm1sp_mode {
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

struct s5kgm1sp {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct gpio_desc	*pwren_gpio;
	struct regulator_bulk_data supplies[S5KGM1SP_NUM_SUPPLIES];

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
	const struct s5kgm1sp_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_s5kgm1sp(sd) container_of(sd, struct s5kgm1sp, subdev)

/*
 * Xclk 24Mhz
 */
static const struct regval s5kgm1sp_global_regs[] = {
	{REG_NULL, 0x00},
};

static const struct regval s5kgm1sp_linear_3840x2160_regs[] = {
	{0x6028, 0x4000},
	{0x0000, 0x0002},
	{0x0000, 0xF8D1},
	{0x6010, 0x0001},
	{0x6214, 0x7971},
	{0x6218, 0x7150},
	{0x0A02, 0x0074},
	{0x6028, 0x2000},
	{0x602A, 0x3F5C},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x00F0},
	{0x6F12, 0x9EBB},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x2DE9},
	{0x6F12, 0xFF5F},
	{0x6F12, 0xFF48},
	{0x6F12, 0x8B46},
	{0x6F12, 0x1746},
	{0x6F12, 0x0068},
	{0x6F12, 0x9A46},
	{0x6F12, 0x4FEA},
	{0x6F12, 0x1049},
	{0x6F12, 0x80B2},
	{0x6F12, 0x8046},
	{0x6F12, 0x0146},
	{0x6F12, 0x0022},
	{0x6F12, 0x4846},
	{0x6F12, 0x00F0},
	{0x6F12, 0x02FB},
	{0x6F12, 0xF94D},
	{0x6F12, 0x95F8},
	{0x6F12, 0x6D00},
	{0x6F12, 0x0228},
	{0x6F12, 0x35D0},
	{0x6F12, 0x0224},
	{0x6F12, 0xF74E},
	{0x6F12, 0x5346},
	{0x6F12, 0xB6F8},
	{0x6F12, 0xB802},
	{0x6F12, 0xB0FB},
	{0x6F12, 0xF4F0},
	{0x6F12, 0xA6F8},
	{0x6F12, 0xB802},
	{0x6F12, 0xD5F8},
	{0x6F12, 0x1411},
	{0x6F12, 0x06F5},
	{0x6F12, 0x2E76},
	{0x6F12, 0x6143},
	{0x6F12, 0xC5F8},
	{0x6F12, 0x1411},
	{0x6F12, 0xB5F8},
	{0x6F12, 0x8C11},
	{0x6F12, 0x411A},
	{0x6F12, 0x89B2},
	{0x6F12, 0x25F8},
	{0x6F12, 0x981B},
	{0x6F12, 0x35F8},
	{0x6F12, 0x142C},
	{0x6F12, 0x6243},
	{0x6F12, 0x521E},
	{0x6F12, 0x00FB},
	{0x6F12, 0x0210},
	{0x6F12, 0xB5F8},
	{0x6F12, 0xF210},
	{0x6F12, 0x07FB},
	{0x6F12, 0x04F2},
	{0x6F12, 0x0844},
	{0x6F12, 0xC5F8},
	{0x6F12, 0xF800},
	{0x6F12, 0x5946},
	{0x6F12, 0x0098},
	{0x6F12, 0x00F0},
	{0x6F12, 0xDBFA},
	{0x6F12, 0x3088},
	{0x6F12, 0x4146},
	{0x6F12, 0x6043},
	{0x6F12, 0x3080},
	{0x6F12, 0xE86F},
	{0x6F12, 0x0122},
	{0x6F12, 0xB0FB},
	{0x6F12, 0xF4F0},
	{0x6F12, 0xE867},
	{0x6F12, 0x04B0},
	{0x6F12, 0x4846},
	{0x6F12, 0xBDE8},
	{0x6F12, 0xF05F},
	{0x6F12, 0x00F0},
	{0x6F12, 0xC7BA},
	{0x6F12, 0x0124},
	{0x6F12, 0xC8E7},
	{0x6F12, 0x2DE9},
	{0x6F12, 0xF041},
	{0x6F12, 0x8046},
	{0x6F12, 0xD848},
	{0x6F12, 0x0022},
	{0x6F12, 0x4168},
	{0x6F12, 0x0D0C},
	{0x6F12, 0x8EB2},
	{0x6F12, 0x3146},
	{0x6F12, 0x2846},
	{0x6F12, 0x00F0},
	{0x6F12, 0xB9FA},
	{0x6F12, 0xD74C},
	{0x6F12, 0xD54F},
	{0x6F12, 0x2078},
	{0x6F12, 0x97F8},
	{0x6F12, 0x8B12},
	{0x6F12, 0x10FB},
	{0x6F12, 0x01F0},
	{0x6F12, 0x2070},
	{0x6F12, 0x4046},
	{0x6F12, 0x00F0},
	{0x6F12, 0xB8FA},
	{0x6F12, 0x2078},
	{0x6F12, 0x97F8},
	{0x6F12, 0x8B12},
	{0x6F12, 0x0122},
	{0x6F12, 0xB0FB},
	{0x6F12, 0xF1F0},
	{0x6F12, 0x2070},
	{0x6F12, 0x3146},
	{0x6F12, 0x2846},
	{0x6F12, 0xBDE8},
	{0x6F12, 0xF041},
	{0x6F12, 0x00F0},
	{0x6F12, 0xA1BA},
	{0x6F12, 0x2DE9},
	{0x6F12, 0xFF47},
	{0x6F12, 0x8146},
	{0x6F12, 0xC648},
	{0x6F12, 0x1746},
	{0x6F12, 0x8846},
	{0x6F12, 0x8068},
	{0x6F12, 0x1C46},
	{0x6F12, 0x85B2},
	{0x6F12, 0x060C},
	{0x6F12, 0x0022},
	{0x6F12, 0x2946},
	{0x6F12, 0x3046},
	{0x6F12, 0x00F0},
	{0x6F12, 0x92FA},
	{0x6F12, 0x2346},
	{0x6F12, 0x3A46},
	{0x6F12, 0x4146},
	{0x6F12, 0x4846},
	{0x6F12, 0x00F0},
	{0x6F12, 0x9BFA},
	{0x6F12, 0xC14A},
	{0x6F12, 0x9088},
	{0x6F12, 0xF0B3},
	{0x6F12, 0xBE48},
	{0x6F12, 0x90F8},
	{0x6F12, 0xBA10},
	{0x6F12, 0xD1B3},
	{0x6F12, 0xD0F8},
	{0x6F12, 0x2801},
	{0x6F12, 0x1168},
	{0x6F12, 0x8842},
	{0x6F12, 0x00D3},
	{0x6F12, 0x0846},
	{0x6F12, 0x010A},
	{0x6F12, 0xB1FA},
	{0x6F12, 0x81F0},
	{0x6F12, 0xC0F1},
	{0x6F12, 0x1700},
	{0x6F12, 0xC140},
	{0x6F12, 0x02EB},
	{0x6F12, 0x4000},
	{0x6F12, 0xC9B2},
	{0x6F12, 0x0389},
	{0x6F12, 0xC288},
	{0x6F12, 0x9B1A},
	{0x6F12, 0x4B43},
	{0x6F12, 0x8033},
	{0x6F12, 0x02EB},
	{0x6F12, 0x2322},
	{0x6F12, 0x0092},
	{0x6F12, 0x438A},
	{0x6F12, 0x028A},
	{0x6F12, 0x9B1A},
	{0x6F12, 0x4B43},
	{0x6F12, 0x8033},
	{0x6F12, 0x02EB},
	{0x6F12, 0x2322},
	{0x6F12, 0x0192},
	{0x6F12, 0x838B},
	{0x6F12, 0x428B},
	{0x6F12, 0x9B1A},
	{0x6F12, 0x4B43},
	{0x6F12, 0x8033},
	{0x6F12, 0x02EB},
	{0x6F12, 0x2322},
	{0x6F12, 0x0292},
	{0x6F12, 0xC28C},
	{0x6F12, 0x808C},
	{0x6F12, 0x121A},
	{0x6F12, 0x4A43},
	{0x6F12, 0x8032},
	{0x6F12, 0x00EB},
	{0x6F12, 0x2220},
	{0x6F12, 0x0390},
	{0x6F12, 0x0022},
	{0x6F12, 0x6846},
	{0x6F12, 0x54F8},
	{0x6F12, 0x2210},
	{0x6F12, 0x50F8},
	{0x6F12, 0x2230},
	{0x6F12, 0x5943},
	{0x6F12, 0x090B},
	{0x6F12, 0x44F8},
	{0x6F12, 0x2210},
	{0x6F12, 0x521C},
	{0x6F12, 0x00E0},
	{0x6F12, 0x01E0},
	{0x6F12, 0x042A},
	{0x6F12, 0xF2D3},
	{0x6F12, 0x04B0},
	{0x6F12, 0x2946},
	{0x6F12, 0x3046},
	{0x6F12, 0xBDE8},
	{0x6F12, 0xF047},
	{0x6F12, 0x0122},
	{0x6F12, 0x00F0},
	{0x6F12, 0x3FBA},
	{0x6F12, 0x2DE9},
	{0x6F12, 0xF041},
	{0x6F12, 0x4FF4},
	{0x6F12, 0x7A71},
	{0x6F12, 0xB0FB},
	{0x6F12, 0xF1F2},
	{0x6F12, 0xB0FB},
	{0x6F12, 0xF1F5},
	{0x6F12, 0x01FB},
	{0x6F12, 0x1207},
	{0x6F12, 0x0024},
	{0x6F12, 0x934E},
	{0x6F12, 0x06E0},
	{0x6F12, 0x48F2},
	{0x6F12, 0xE801},
	{0x6F12, 0x4843},
	{0x6F12, 0x400B},
	{0x6F12, 0x00F0},
	{0x6F12, 0x40FA},
	{0x6F12, 0x641C},
	{0x6F12, 0x706B},
	{0x6F12, 0xAC42},
	{0x6F12, 0x4FEA},
	{0x6F12, 0x9000},
	{0x6F12, 0xF3D3},
	{0x6F12, 0x7843},
	{0x6F12, 0xBDE8},
	{0x6F12, 0xF041},
	{0x6F12, 0x00EB},
	{0x6F12, 0x4010},
	{0x6F12, 0x400B},
	{0x6F12, 0x00F0},
	{0x6F12, 0x32BA},
	{0x6F12, 0x70B5},
	{0x6F12, 0x0024},
	{0x6F12, 0x8A4D},
	{0x6F12, 0x0CE0},
	{0x6F12, 0xA0F5},
	{0x6F12, 0x7F42},
	{0x6F12, 0xFE3A},
	{0x6F12, 0x13D0},
	{0x6F12, 0x521E},
	{0x6F12, 0x14D0},
	{0x6F12, 0x91F8},
	{0x6F12, 0x0E11},
	{0x6F12, 0x00F0},
	{0x6F12, 0x29FA},
	{0x6F12, 0x641C},
	{0x6F12, 0x142C},
	{0x6F12, 0x05D2},
	{0x6F12, 0x05EB},
	{0x6F12, 0x8401},
	{0x6F12, 0xB1F8},
	{0x6F12, 0x0C01},
	{0x6F12, 0x0028},
	{0x6F12, 0xECD1},
	{0x6F12, 0x7D49},
	{0x6F12, 0x0420},
	{0x6F12, 0xA1F8},
	{0x6F12, 0xCA06},
	{0x6F12, 0x70BD},
	{0x6F12, 0x91F8},
	{0x6F12, 0x0E01},
	{0x6F12, 0x05E0},
	{0x6F12, 0x91F8},
	{0x6F12, 0x0E01},
	{0x6F12, 0x4FF4},
	{0x6F12, 0x7A71},
	{0x6F12, 0x10FB},
	{0x6F12, 0x01F0},
	{0x6F12, 0x00F0},
	{0x6F12, 0x15FA},
	{0x6F12, 0xE5E7},
	{0x6F12, 0x70B5},
	{0x6F12, 0x0024},
	{0x6F12, 0x764D},
	{0x6F12, 0x0CE0},
	{0x6F12, 0xA0F5},
	{0x6F12, 0x7F42},
	{0x6F12, 0xFE3A},
	{0x6F12, 0x13D0},
	{0x6F12, 0x521E},
	{0x6F12, 0x14D0},
	{0x6F12, 0x91F8},
	{0x6F12, 0x5E11},
	{0x6F12, 0x00F0},
	{0x6F12, 0x01FA},
	{0x6F12, 0x641C},
	{0x6F12, 0x142C},
	{0x6F12, 0x05D2},
	{0x6F12, 0x05EB},
	{0x6F12, 0x8401},
	{0x6F12, 0xB1F8},
	{0x6F12, 0x5C01},
	{0x6F12, 0x0028},
	{0x6F12, 0xECD1},
	{0x6F12, 0x6949},
	{0x6F12, 0x0220},
	{0x6F12, 0xA1F8},
	{0x6F12, 0xCA06},
	{0x6F12, 0x70BD},
	{0x6F12, 0x91F8},
	{0x6F12, 0x5E01},
	{0x6F12, 0x05E0},
	{0x6F12, 0x91F8},
	{0x6F12, 0x5E01},
	{0x6F12, 0x4FF4},
	{0x6F12, 0x7A71},
	{0x6F12, 0x10FB},
	{0x6F12, 0x01F0},
	{0x6F12, 0x00F0},
	{0x6F12, 0xEDF9},
	{0x6F12, 0xE5E7},
	{0x6F12, 0xF8B5},
	{0x6F12, 0x604D},
	{0x6F12, 0xB5F8},
	{0x6F12, 0xCA06},
	{0x6F12, 0x0328},
	{0x6F12, 0x5BD1},
	{0x6F12, 0x5F4E},
	{0x6F12, 0x96F8},
	{0x6F12, 0x3800},
	{0x6F12, 0x90B1},
	{0x6F12, 0x96F8},
	{0x6F12, 0x3900},
	{0x6F12, 0x0A28},
	{0x6F12, 0x0ED8},
	{0x6F12, 0x0024},
	{0x6F12, 0x08E0},
	{0x6F12, 0x06EB},
	{0x6F12, 0x4400},
	{0x6F12, 0x3219},
	{0x6F12, 0x408F},
	{0x6F12, 0x0121},
	{0x6F12, 0x4E32},
	{0x6F12, 0x00F0},
	{0x6F12, 0xD9F9},
	{0x6F12, 0x641C},
	{0x6F12, 0x96F8},
	{0x6F12, 0x3900},
	{0x6F12, 0xA042},
	{0x6F12, 0xF2D8},
	{0x6F12, 0xD5F8},
	{0x6F12, 0xDC06},
	{0x6F12, 0x8047},
	{0x6F12, 0x96F8},
	{0x6F12, 0x2E00},
	{0x6F12, 0x4FF4},
	{0x6F12, 0x7A71},
	{0x6F12, 0x10FB},
	{0x6F12, 0x01F0},
	{0x6F12, 0x00F0},
	{0x6F12, 0xC4F9},
	{0x6F12, 0x0220},
	{0x6F12, 0x00F0},
	{0x6F12, 0xCBF9},
	{0x6F12, 0x0120},
	{0x6F12, 0x00F0},
	{0x6F12, 0xCDF9},
	{0x6F12, 0x4D49},
	{0x6F12, 0x0020},
	{0x6F12, 0x4883},
	{0x6F12, 0x4B4C},
	{0x6F12, 0x94F8},
	{0x6F12, 0xFB20},
	{0x6F12, 0x4A83},
	{0x6F12, 0x95F8},
	{0x6F12, 0xAC10},
	{0x6F12, 0xB1B1},
	{0x6F12, 0x85F8},
	{0x6F12, 0x4807},
	{0x6F12, 0x00F0},
	{0x6F12, 0xC4F9},
	{0x6F12, 0x0646},
	{0x6F12, 0x3046},
	{0x6F12, 0x00F0},
	{0x6F12, 0xC5F9},
	{0x6F12, 0xD4F8},
	{0x6F12, 0x6412},
	{0x6F12, 0x8142},
	{0x6F12, 0x01D2},
	{0x6F12, 0x0121},
	{0x6F12, 0x00E0},
	{0x6F12, 0x0021},
	{0x6F12, 0x95F8},
	{0x6F12, 0x4807},
	{0x6F12, 0x8DF8},
	{0x6F12, 0x0000},
	{0x6F12, 0x9DF8},
	{0x6F12, 0x0000},
	{0x6F12, 0x0843},
	{0x6F12, 0xEDD0},
	{0x6F12, 0x95F8},
	{0x6F12, 0x9806},
	{0x6F12, 0x0028},
	{0x6F12, 0x0ED0},
	{0x6F12, 0x0122},
	{0x6F12, 0x1402},
	{0x6F12, 0x48F6},
	{0x6F12, 0xF825},
	{0x6F12, 0x2146},
	{0x6F12, 0x2846},
	{0x6F12, 0x00F0},
	{0x6F12, 0xAFF9},
	{0x6F12, 0x2146},
	{0x6F12, 0x2846},
	{0x6F12, 0xBDE8},
	{0x6F12, 0xF840},
	{0x6F12, 0x0022},
	{0x6F12, 0x00F0},
	{0x6F12, 0xA8B9},
	{0x6F12, 0xF8BD},
	{0x6F12, 0x2DE9},
	{0x6F12, 0xF041},
	{0x6F12, 0x344C},
	{0x6F12, 0x3249},
	{0x6F12, 0x0646},
	{0x6F12, 0x94F8},
	{0x6F12, 0x6970},
	{0x6F12, 0x8988},
	{0x6F12, 0x94F8},
	{0x6F12, 0x8120},
	{0x6F12, 0x0020},
	{0x6F12, 0xC1B1},
	{0x6F12, 0x2146},
	{0x6F12, 0xD1F8},
	{0x6F12, 0x9410},
	{0x6F12, 0x72B1},
	{0x6F12, 0x8FB1},
	{0x6F12, 0x0846},
	{0x6F12, 0x00F0},
	{0x6F12, 0x98F9},
	{0x6F12, 0x0546},
	{0x6F12, 0xE06F},
	{0x6F12, 0x00F0},
	{0x6F12, 0x94F9},
	{0x6F12, 0x8542},
	{0x6F12, 0x02D2},
	{0x6F12, 0xD4F8},
	{0x6F12, 0x9400},
	{0x6F12, 0x26E0},
	{0x6F12, 0xE06F},
	{0x6F12, 0x24E0},
	{0x6F12, 0x002F},
	{0x6F12, 0xFBD1},
	{0x6F12, 0x002A},
	{0x6F12, 0x24D0},
	{0x6F12, 0x0846},
	{0x6F12, 0x1EE0},
	{0x6F12, 0x1E49},
	{0x6F12, 0x0D8E},
	{0x6F12, 0x496B},
	{0x6F12, 0x4B42},
	{0x6F12, 0x77B1},
	{0x6F12, 0x2048},
	{0x6F12, 0x806F},
	{0x6F12, 0x10E0},
	{0x6F12, 0x4242},
	{0x6F12, 0x00E0},
	{0x6F12, 0x0246},
	{0x6F12, 0x0029},
	{0x6F12, 0x0FDB},
	{0x6F12, 0x8A42},
	{0x6F12, 0x0FDD},
	{0x6F12, 0x3046},
	{0x6F12, 0xBDE8},
	{0x6F12, 0xF041},
	{0x6F12, 0x00F0},
	{0x6F12, 0x78B9},
	{0x6F12, 0x002A},
	{0x6F12, 0x0CD0},
	{0x6F12, 0x1748},
	{0x6F12, 0xD0F8},
	{0x6F12, 0x8C00},
	{0x6F12, 0x25B1},
	{0x6F12, 0x0028},
	{0x6F12, 0xEDDA},
	{0x6F12, 0xEAE7},
	{0x6F12, 0x1946},
	{0x6F12, 0xEDE7},
	{0x6F12, 0x00F0},
	{0x6F12, 0x70F9},
	{0x6F12, 0xE060},
	{0x6F12, 0x0120},
	{0x6F12, 0xBDE8},
	{0x6F12, 0xF081},
	{0x6F12, 0x2DE9},
	{0x6F12, 0xF35F},
	{0x6F12, 0xDFF8},
	{0x6F12, 0x24A0},
	{0x6F12, 0x0C46},
	{0x6F12, 0xBAF8},
	{0x6F12, 0xBE04},
	{0x6F12, 0x08B1},
	{0x6F12, 0x00F0},
	{0x6F12, 0x67F9},
	{0x6F12, 0x0B4E},
	{0x6F12, 0x3088},
	{0x6F12, 0x0128},
	{0x6F12, 0x19D1},
	{0x6F12, 0x002C},
	{0x6F12, 0x17D1},
	{0x6F12, 0x11E0},
	{0x6F12, 0x2000},
	{0x6F12, 0x4690},
	{0x6F12, 0x2000},
	{0x6F12, 0x2C30},
	{0x6F12, 0x2000},
	{0x6F12, 0x2E30},
	{0x6F12, 0x2000},
	{0x6F12, 0x2580},
	{0x6F12, 0x2000},
	{0x6F12, 0x6000},
	{0x6F12, 0x2000},
	{0x6F12, 0x0DE0},
	{0x6F12, 0x4000},
	{0x6F12, 0x7000},
	{0x6F12, 0x2000},
	{0x6F12, 0x2BA0},
	{0x6F12, 0x2000},
	{0x6F12, 0x3600},
	{0x6F12, 0x6F4D},
	{0x6F12, 0x2889},
	{0x6F12, 0x18B1},
	{0x6F12, 0x401E},
	{0x6F12, 0x2881},
	{0x6F12, 0xBDE8},
	{0x6F12, 0xFC9F},
	{0x6F12, 0xDFF8},
	{0x6F12, 0xB491},
	{0x6F12, 0xD9F8},
	{0x6F12, 0x0000},
	{0x6F12, 0xB0F8},
	{0x6F12, 0xD602},
	{0x6F12, 0x38B1},
	{0x6F12, 0x3089},
	{0x6F12, 0x401C},
	{0x6F12, 0x80B2},
	{0x6F12, 0x3081},
	{0x6F12, 0xFF28},
	{0x6F12, 0x01D9},
	{0x6F12, 0xE889},
	{0x6F12, 0x3081},
	{0x6F12, 0x6648},
	{0x6F12, 0x4FF0},
	{0x6F12, 0x0008},
	{0x6F12, 0xC6F8},
	{0x6F12, 0x0C80},
	{0x6F12, 0xB0F8},
	{0x6F12, 0x5EB0},
	{0x6F12, 0x40F2},
	{0x6F12, 0xFF31},
	{0x6F12, 0x0B20},
	{0x6F12, 0x00F0},
	{0x6F12, 0x31F9},
	{0x6F12, 0xD9F8},
	{0x6F12, 0x0000},
	{0x6F12, 0x0027},
	{0x6F12, 0x3C46},
	{0x6F12, 0xB0F8},
	{0x6F12, 0xD412},
	{0x6F12, 0x21B1},
	{0x6F12, 0x0098},
	{0x6F12, 0x00F0},
	{0x6F12, 0x18F9},
	{0x6F12, 0x0746},
	{0x6F12, 0x0BE0},
	{0x6F12, 0xB0F8},
	{0x6F12, 0xD602},
	{0x6F12, 0x40B1},
	{0x6F12, 0x3089},
	{0x6F12, 0xE989},
	{0x6F12, 0x8842},
	{0x6F12, 0x04D3},
	{0x6F12, 0x0098},
	{0x6F12, 0xFFF7},
	{0x6F12, 0x5BFF},
	{0x6F12, 0x0746},
	{0x6F12, 0x0124},
	{0x6F12, 0x3846},
	{0x6F12, 0x00F0},
	{0x6F12, 0x1BF9},
	{0x6F12, 0xD9F8},
	{0x6F12, 0x0000},
	{0x6F12, 0xB0F8},
	{0x6F12, 0xD602},
	{0x6F12, 0x08B9},
	{0x6F12, 0xA6F8},
	{0x6F12, 0x0280},
	{0x6F12, 0xC7B3},
	{0x6F12, 0x4746},
	{0x6F12, 0xA6F8},
	{0x6F12, 0x0880},
	{0x6F12, 0x00F0},
	{0x6F12, 0x13F9},
	{0x6F12, 0xF068},
	{0x6F12, 0x3061},
	{0x6F12, 0x688D},
	{0x6F12, 0x50B3},
	{0x6F12, 0xA88D},
	{0x6F12, 0x50BB},
	{0x6F12, 0x00F0},
	{0x6F12, 0x10F9},
	{0x6F12, 0xA889},
	{0x6F12, 0x20B3},
	{0x6F12, 0x1CB3},
	{0x6F12, 0x706B},
	{0x6F12, 0xAA88},
	{0x6F12, 0xDAF8},
	{0x6F12, 0x0815},
	{0x6F12, 0xCAB1},
	{0x6F12, 0x8842},
	{0x6F12, 0x0CDB},
	{0x6F12, 0x90FB},
	{0x6F12, 0xF1F3},
	{0x6F12, 0x90FB},
	{0x6F12, 0xF1F2},
	{0x6F12, 0x01FB},
	{0x6F12, 0x1303},
	{0x6F12, 0xB3EB},
	{0x6F12, 0x610F},
	{0x6F12, 0x00DD},
	{0x6F12, 0x521C},
	{0x6F12, 0x01FB},
	{0x6F12, 0x1200},
	{0x6F12, 0x0BE0},
	{0x6F12, 0x91FB},
	{0x6F12, 0xF0F3},
	{0x6F12, 0x91FB},
	{0x6F12, 0xF0F2},
	{0x6F12, 0x00FB},
	{0x6F12, 0x1313},
	{0x6F12, 0xB3EB},
	{0x6F12, 0x600F},
	{0x6F12, 0x00DD},
	{0x6F12, 0x521C},
	{0x6F12, 0x5043},
	{0x6F12, 0x401A},
	{0x6F12, 0xF168},
	{0x6F12, 0x01EB},
	{0x6F12, 0x4000},
	{0x6F12, 0xF060},
	{0x6F12, 0xA88D},
	{0x6F12, 0x10B1},
	{0x6F12, 0xF089},
	{0x6F12, 0x3087},
	{0x6F12, 0xAF85},
	{0x6F12, 0x5846},
	{0x6F12, 0xBDE8},
	{0x6F12, 0xFC5F},
	{0x6F12, 0x00F0},
	{0x6F12, 0xE4B8},
	{0x6F12, 0x70B5},
	{0x6F12, 0x3049},
	{0x6F12, 0x0446},
	{0x6F12, 0x0020},
	{0x6F12, 0xC1F8},
	{0x6F12, 0x3005},
	{0x6F12, 0x2F48},
	{0x6F12, 0x0022},
	{0x6F12, 0xC168},
	{0x6F12, 0x0D0C},
	{0x6F12, 0x8EB2},
	{0x6F12, 0x3146},
	{0x6F12, 0x2846},
	{0x6F12, 0x00F0},
	{0x6F12, 0x6CF8},
	{0x6F12, 0x2046},
	{0x6F12, 0x00F0},
	{0x6F12, 0xD7F8},
	{0x6F12, 0x3146},
	{0x6F12, 0x2846},
	{0x6F12, 0xBDE8},
	{0x6F12, 0x7040},
	{0x6F12, 0x0122},
	{0x6F12, 0x00F0},
	{0x6F12, 0x62B8},
	{0x6F12, 0x10B5},
	{0x6F12, 0x0022},
	{0x6F12, 0xAFF2},
	{0x6F12, 0x6751},
	{0x6F12, 0x2448},
	{0x6F12, 0x00F0},
	{0x6F12, 0xCEF8},
	{0x6F12, 0x224C},
	{0x6F12, 0x0122},
	{0x6F12, 0xAFF2},
	{0x6F12, 0xD941},
	{0x6F12, 0x2060},
	{0x6F12, 0x2148},
	{0x6F12, 0x00F0},
	{0x6F12, 0xC6F8},
	{0x6F12, 0x0022},
	{0x6F12, 0xAFF2},
	{0x6F12, 0xA141},
	{0x6F12, 0x6060},
	{0x6F12, 0x1F48},
	{0x6F12, 0x00F0},
	{0x6F12, 0xBFF8},
	{0x6F12, 0x0022},
	{0x6F12, 0xAFF2},
	{0x6F12, 0xE931},
	{0x6F12, 0xA060},
	{0x6F12, 0x1C48},
	{0x6F12, 0x00F0},
	{0x6F12, 0xB8F8},
	{0x6F12, 0x0022},
	{0x6F12, 0xAFF2},
	{0x6F12, 0xB731},
	{0x6F12, 0x1A48},
	{0x6F12, 0x00F0},
	{0x6F12, 0xB2F8},
	{0x6F12, 0x0022},
	{0x6F12, 0xAFF2},
	{0x6F12, 0x7331},
	{0x6F12, 0x1848},
	{0x6F12, 0x00F0},
	{0x6F12, 0xACF8},
	{0x6F12, 0x0022},
	{0x6F12, 0xAFF2},
	{0x6F12, 0x2F31},
	{0x6F12, 0x1648},
	{0x6F12, 0x00F0},
	{0x6F12, 0xA6F8},
	{0x6F12, 0x0022},
	{0x6F12, 0xAFF2},
	{0x6F12, 0x7521},
	{0x6F12, 0x1448},
	{0x6F12, 0x00F0},
	{0x6F12, 0xA0F8},
	{0x6F12, 0x0022},
	{0x6F12, 0xAFF2},
	{0x6F12, 0xED11},
	{0x6F12, 0x1248},
	{0x6F12, 0x00F0},
	{0x6F12, 0x9AF8},
	{0x6F12, 0x0022},
	{0x6F12, 0xAFF2},
	{0x6F12, 0xAD01},
	{0x6F12, 0x1048},
	{0x6F12, 0x00F0},
	{0x6F12, 0x94F8},
	{0x6F12, 0xE060},
	{0x6F12, 0x10BD},
	{0x6F12, 0x0000},
	{0x6F12, 0x2000},
	{0x6F12, 0x2BA0},
	{0x6F12, 0x2000},
	{0x6F12, 0x0890},
	{0x6F12, 0x4000},
	{0x6F12, 0x7000},
	{0x6F12, 0x2000},
	{0x6F12, 0x2E30},
	{0x6F12, 0x2000},
	{0x6F12, 0x4690},
	{0x6F12, 0x0000},
	{0x6F12, 0x24A7},
	{0x6F12, 0x0001},
	{0x6F12, 0x1AF3},
	{0x6F12, 0x0001},
	{0x6F12, 0x09BD},
	{0x6F12, 0x0000},
	{0x6F12, 0xA943},
	{0x6F12, 0x0000},
	{0x6F12, 0x71F1},
	{0x6F12, 0x0000},
	{0x6F12, 0x7239},
	{0x6F12, 0x0000},
	{0x6F12, 0x5D87},
	{0x6F12, 0x0000},
	{0x6F12, 0x576B},
	{0x6F12, 0x0000},
	{0x6F12, 0x57ED},
	{0x6F12, 0x0000},
	{0x6F12, 0xBF8D},
	{0x6F12, 0x4AF6},
	{0x6F12, 0x293C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x42F2},
	{0x6F12, 0xA74C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x41F6},
	{0x6F12, 0xF32C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x010C},
	{0x6F12, 0x6047},
	{0x6F12, 0x40F6},
	{0x6F12, 0xBD1C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x010C},
	{0x6F12, 0x6047},
	{0x6F12, 0x4AF6},
	{0x6F12, 0x2D1C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x48F2},
	{0x6F12, 0x0B3C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x4AF6},
	{0x6F12, 0x431C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x48F2},
	{0x6F12, 0x6F2C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x47F6},
	{0x6F12, 0xA57C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x45F6},
	{0x6F12, 0x815C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x4AF6},
	{0x6F12, 0xE70C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x4AF6},
	{0x6F12, 0x171C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x4AF6},
	{0x6F12, 0x453C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x4AF6},
	{0x6F12, 0x532C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x45F2},
	{0x6F12, 0x377C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x45F2},
	{0x6F12, 0xD56C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x45F2},
	{0x6F12, 0xC91C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x40F2},
	{0x6F12, 0xAB2C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x44F6},
	{0x6F12, 0x897C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x45F2},
	{0x6F12, 0xA56C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x45F2},
	{0x6F12, 0xEF6C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x40F2},
	{0x6F12, 0x6D7C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x4BF6},
	{0x6F12, 0x8D7C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x6F12, 0x4BF2},
	{0x6F12, 0xAB4C},
	{0x6F12, 0xC0F2},
	{0x6F12, 0x000C},
	{0x6F12, 0x6047},
	{0x602A, 0x46A0},
	{0x6F12, 0x0549},
	{0x6F12, 0x0448},
	{0x6F12, 0x054A},
	{0x6F12, 0xC1F8},
	{0x6F12, 0x5005},
	{0x6F12, 0x101A},
	{0x6F12, 0xA1F8},
	{0x6F12, 0x5405},
	{0x6F12, 0xFFF7},
	{0x6F12, 0x0EBF},
	{0x6F12, 0x2000},
	{0x6F12, 0x46D8},
	{0x6F12, 0x2000},
	{0x6F12, 0x2E30},
	{0x6F12, 0x2000},
	{0x6F12, 0x6000},
	{0x6F12, 0x7047},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x0000},
	{0x6F12, 0x08D1},
	{0x6F12, 0x00A6},
	{0x6F12, 0x0000},
	{0x6F12, 0x00FF},
	{0x6028, 0x4000},
	{0x6214, 0x7971},
	{0x6218, 0x7150},
	{0x0344, 0x0058},
	{0x0346, 0x01AC},
	{0x0348, 0x0F57},
	{0x034A, 0x0A1B},
	{0x034C, 0x0F00},
	{0x034E, 0x0870},
	{0x0350, 0x0000},
	{0x0352, 0x0000},
	{0x0340, 0x0C7A},
	{0x0342, 0x13A0},
	{0x0900, 0x0111},
	{0x0380, 0x0001},
	{0x0382, 0x0001},
	{0x0384, 0x0001},
	{0x0386, 0x0001},
	{0x0404, 0x1000},
	{0x0402, 0x1010},
	{0x0136, 0x1800},
	{0x0304, 0x0006},
	{0x030C, 0x0000},
	{0x0306, 0x00F1},
	{0x0302, 0x0001},
	{0x0300, 0x0008},
	{0x030E, 0x0003},
	{0x0312, 0x0001},
	{0x0310, 0x0090},
	{0x6028, 0x2000},
	{0x602A, 0x1492},
	{0x6F12, 0x0078},
	{0x602A, 0x0E4E},
	{0x6F12, 0x007A},
	{0x6028, 0x4000},
	{0x0118, 0x0004},
	{0x021E, 0x0000},
	{0x6028, 0x2000},
	{0x602A, 0x2126},
	{0x6F12, 0x0100},
	{0x602A, 0x1168},
	{0x6F12, 0x0020},
	{0x602A, 0x2DB6},
	{0x6F12, 0x0001},
	{0x602A, 0x1668},
	{0x6F12, 0xF0F0},
	{0x602A, 0x166A},
	{0x6F12, 0xF0F0},
	{0x602A, 0x118A},
	{0x6F12, 0x0802},
	{0x602A, 0x151E},
	{0x6F12, 0x0001},
	{0x602A, 0x217E},
	{0x6F12, 0x0001},
	{0x602A, 0x1520},
	{0x6F12, 0x0008},
	{0x602A, 0x2522},
	{0x6F12, 0x0804},
	{0x602A, 0x2524},
	{0x6F12, 0x0400},
	{0x602A, 0x2568},
	{0x6F12, 0x5500},
	{0x602A, 0x2588},
	{0x6F12, 0x1111},
	{0x602A, 0x258C},
	{0x6F12, 0x1111},
	{0x602A, 0x25A6},
	{0x6F12, 0x0000},
	{0x602A, 0x252C},
	{0x6F12, 0x0601},
	{0x602A, 0x252E},
	{0x6F12, 0x0605},
	{0x602A, 0x25A8},
	{0x6F12, 0x1100},
	{0x602A, 0x25AC},
	{0x6F12, 0x0011},
	{0x602A, 0x25B0},
	{0x6F12, 0x1100},
	{0x602A, 0x25B4},
	{0x6F12, 0x0011},
	{0x602A, 0x15A4},
	{0x6F12, 0x0141},
	{0x602A, 0x15A6},
	{0x6F12, 0x0545},
	{0x602A, 0x15A8},
	{0x6F12, 0x0649},
	{0x602A, 0x15AA},
	{0x6F12, 0x024D},
	{0x602A, 0x15AC},
	{0x6F12, 0x0151},
	{0x602A, 0x15AE},
	{0x6F12, 0x0555},
	{0x602A, 0x15B0},
	{0x6F12, 0x0659},
	{0x602A, 0x15B2},
	{0x6F12, 0x025D},
	{0x602A, 0x15B4},
	{0x6F12, 0x0161},
	{0x602A, 0x15B6},
	{0x6F12, 0x0565},
	{0x602A, 0x15B8},
	{0x6F12, 0x0669},
	{0x602A, 0x15BA},
	{0x6F12, 0x026D},
	{0x602A, 0x15BC},
	{0x6F12, 0x0171},
	{0x602A, 0x15BE},
	{0x6F12, 0x0575},
	{0x602A, 0x15C0},
	{0x6F12, 0x0679},
	{0x602A, 0x15C2},
	{0x6F12, 0x027D},
	{0x602A, 0x15C4},
	{0x6F12, 0x0141},
	{0x602A, 0x15C6},
	{0x6F12, 0x0545},
	{0x602A, 0x15C8},
	{0x6F12, 0x0649},
	{0x602A, 0x15CA},
	{0x6F12, 0x024D},
	{0x602A, 0x15CC},
	{0x6F12, 0x0151},
	{0x602A, 0x15CE},
	{0x6F12, 0x0555},
	{0x602A, 0x15D0},
	{0x6F12, 0x0659},
	{0x602A, 0x15D2},
	{0x6F12, 0x025D},
	{0x602A, 0x15D4},
	{0x6F12, 0x0161},
	{0x602A, 0x15D6},
	{0x6F12, 0x0565},
	{0x602A, 0x15D8},
	{0x6F12, 0x0669},
	{0x602A, 0x15DA},
	{0x6F12, 0x026D},
	{0x602A, 0x15DC},
	{0x6F12, 0x0171},
	{0x602A, 0x15DE},
	{0x6F12, 0x0575},
	{0x602A, 0x15E0},
	{0x6F12, 0x0679},
	{0x602A, 0x15E2},
	{0x6F12, 0x027D},
	{0x602A, 0x1A50},
	{0x6F12, 0x0001},
	{0x602A, 0x1A54},
	{0x6F12, 0x0100},
	{0x6028, 0x4000},
	{0x0D00, 0x0101},
	{0x0D02, 0x0101},
	{0x0114, 0x0301},
	{0xF486, 0x0000},
	{0xF488, 0x0000},
	{0xF48A, 0x0000},
	{0xF48C, 0x0000},
	{0xF48E, 0x0000},
	{0xF490, 0x0000},
	{0xF492, 0x0000},
	{0xF494, 0x0000},
	{0xF496, 0x0000},
	{0xF498, 0x0000},
	{0xF49A, 0x0000},
	{0xF49C, 0x0000},
	{0xF49E, 0x0000},
	{0xF4A0, 0x0000},
	{0xF4A2, 0x0000},
	{0xF4A4, 0x0000},
	{0xF4A6, 0x0000},
	{0xF4A8, 0x0000},
	{0xF4AA, 0x0000},
	{0xF4AC, 0x0000},
	{0xF4AE, 0x0000},
	{0xF4B0, 0x0000},
	{0xF4B2, 0x0000},
	{0xF4B4, 0x0000},
	{0xF4B6, 0x0000},
	{0xF4B8, 0x0000},
	{0xF4BA, 0x0000},
	{0xF4BC, 0x0000},
	{0xF4BE, 0x0000},
	{0xF4C0, 0x0000},
	{0xF4C2, 0x0000},
	{0xF4C4, 0x0000},
	{0x0202, 0x0010},
	{0x0226, 0x0010},
	{0x0204, 0x0020},
	{0xF45A, 0x0015},
	{0x0B06, 0x0101},
	{0x6028, 0x2000},
	{0x602A, 0x107A},
	{0x6F12, 0x1D00},
	{0x602A, 0x1074},
	{0x6F12, 0x1D00},
	{0x602A, 0x0E7C},
	{0x6F12, 0x0000},
	{0x602A, 0x1120},
	{0x6F12, 0x0200},
	{0x602A, 0x1122},
	{0x6F12, 0x0078},
	{0x602A, 0x1128},
	{0x6F12, 0x0604},
	{0x602A, 0x1AC0},
	{0x6F12, 0x0200},
	{0x602A, 0x1AC2},
	{0x6F12, 0x0002},
	{0x602A, 0x1494},
	{0x6F12, 0x3D68},
	{0x602A, 0x1498},
	{0x6F12, 0xF10D},
	{0x602A, 0x1488},
	{0x6F12, 0x0F04},
	{0x602A, 0x148A},
	{0x6F12, 0x170B},
	{0x602A, 0x150E},
	{0x6F12, 0x40C2},
	{0x602A, 0x1510},
	{0x6F12, 0x80AF},
	{0x602A, 0x1512},
	{0x6F12, 0x00A0},
	{0x602A, 0x1486},
	{0x6F12, 0x1430},
	{0x602A, 0x1490},
	{0x6F12, 0x5009},
	{0x602A, 0x149E},
	{0x6F12, 0x01C4},
	{0x602A, 0x11CC},
	{0x6F12, 0x0008},
	{0x602A, 0x11CE},
	{0x6F12, 0x000B},
	{0x602A, 0x11D0},
	{0x6F12, 0x0006},
	{0x602A, 0x11DA},
	{0x6F12, 0x0012},
	{0x602A, 0x11E6},
	{0x6F12, 0x002A},
	{0x602A, 0x125E},
	{0x6F12, 0x0048},
	{0x602A, 0x11F4},
	{0x6F12, 0x0000},
	{0x602A, 0x11F8},
	{0x6F12, 0x0016},
	{0x6028, 0x4000},
	{0xF444, 0x05BF},
	{0xF44A, 0x0016},
	{0xF44C, 0x1414},
	{0xF44E, 0x0014},
	{0xF458, 0x0008},
	{0xF46E, 0xC040},
	{0xF470, 0x0008},
	{0x6028, 0x2000},
	{0x602A, 0x1CAA},
	{0x6F12, 0x0000},
	{0x602A, 0x1CAC},
	{0x6F12, 0x0000},
	{0x602A, 0x1CAE},
	{0x6F12, 0x0000},
	{0x602A, 0x1CB0},
	{0x6F12, 0x0000},
	{0x602A, 0x1CB2},
	{0x6F12, 0x0000},
	{0x602A, 0x1CB4},
	{0x6F12, 0x0000},
	{0x602A, 0x1CB6},
	{0x6F12, 0x0000},
	{0x602A, 0x1CB8},
	{0x6F12, 0x0000},
	{0x602A, 0x1CBA},
	{0x6F12, 0x0000},
	{0x602A, 0x1CBC},
	{0x6F12, 0x0000},
	{0x602A, 0x1CBE},
	{0x6F12, 0x0000},
	{0x602A, 0x1CC0},
	{0x6F12, 0x0000},
	{0x602A, 0x1CC2},
	{0x6F12, 0x0000},
	{0x602A, 0x1CC4},
	{0x6F12, 0x0000},
	{0x602A, 0x1CC6},
	{0x6F12, 0x0000},
	{0x602A, 0x1CC8},
	{0x6F12, 0x0000},
	{0x602A, 0x6000},
	{0x6F12, 0x000F},
	{0x602A, 0x6002},
	{0x6F12, 0xFFFF},
	{0x602A, 0x6004},
	{0x6F12, 0x0000},
	{0x602A, 0x6006},
	{0x6F12, 0x1000},
	{0x602A, 0x6008},
	{0x6F12, 0x1000},
	{0x602A, 0x600A},
	{0x6F12, 0x1000},
	{0x602A, 0x600C},
	{0x6F12, 0x1000},
	{0x602A, 0x600E},
	{0x6F12, 0x1000},
	{0x602A, 0x6010},
	{0x6F12, 0x1000},
	{0x602A, 0x6012},
	{0x6F12, 0x1000},
	{0x602A, 0x6014},
	{0x6F12, 0x1000},
	{0x602A, 0x6016},
	{0x6F12, 0x1000},
	{0x602A, 0x6018},
	{0x6F12, 0x1000},
	{0x602A, 0x601A},
	{0x6F12, 0x1000},
	{0x602A, 0x601C},
	{0x6F12, 0x1000},
	{0x602A, 0x601E},
	{0x6F12, 0x1000},
	{0x602A, 0x6020},
	{0x6F12, 0x1000},
	{0x602A, 0x6022},
	{0x6F12, 0x1000},
	{0x602A, 0x6024},
	{0x6F12, 0x1000},
	{0x602A, 0x6026},
	{0x6F12, 0x1000},
	{0x602A, 0x6028},
	{0x6F12, 0x1000},
	{0x602A, 0x602A},
	{0x6F12, 0x1000},
	{0x602A, 0x602C},
	{0x6F12, 0x1000},
	{0x602A, 0x1144},
	{0x6F12, 0x0100},
	{0x602A, 0x1146},
	{0x6F12, 0x1B00},
	{0x602A, 0x1080},
	{0x6F12, 0x0100},
	{0x602A, 0x1084},
	{0x6F12, 0x00C0},
	{0x602A, 0x108A},
	{0x6F12, 0x00C0},
	{0x602A, 0x1090},
	{0x6F12, 0x0001},
	{0x602A, 0x1092},
	{0x6F12, 0x0000},
	{0x602A, 0x1094},
	{0x6F12, 0xA32E},
	{0x602A, 0x602E},
	{0x6F12, 0x0000},
	{0x602A, 0x6038},
	{0x6F12, 0x0003},
	{0x602A, 0x603A},
	{0x6F12, 0x005F},
	{0x602A, 0x603C},
	{0x6F12, 0x0060},
	{0x602A, 0x603E},
	{0x6F12, 0x0061},
	{0x602A, 0x25D4},
	{0x6F12, 0x0000},
	{0x602A, 0x25D6},
	{0x6F12, 0x0000},
	{0x6028, 0x4000},
	{0x0100, 0x0100},
	{REG_NULL, 0x00},
};
static const struct s5kgm1sp_mode supported_modes[] = {
	{
		.width = 3840,
		.height = 2160,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x0BB8,
		.hts_def = 0x13A0,
		.vts_def = 0x0C7A,
		.reg_list = s5kgm1sp_linear_3840x2160_regs,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const s64 link_freq_menu_items[] = {
	S5KGM1SP_LINK_FREQ
};

static const char * const s5kgm1sp_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int s5kgm1sp_write_reg(struct i2c_client *client, u16 reg,
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

static int s5kgm1sp_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		if (unlikely(regs[i].addr == REG_DELAY))
			usleep_range(regs[i].val, regs[i].val * 2);
		else
			ret = s5kgm1sp_write_reg(client, regs[i].addr,
					       S5KGM1SP_REG_VALUE_16BIT,
					       regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
static int s5kgm1sp_read_reg(struct i2c_client *client, u16 reg,
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

static int s5kgm1sp_get_reso_dist(const struct s5kgm1sp_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
			abs(mode->height - framefmt->height);
}

static const struct s5kgm1sp_mode *
s5kgm1sp_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = s5kgm1sp_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int s5kgm1sp_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct s5kgm1sp *s5kgm1sp = to_s5kgm1sp(sd);
	const struct s5kgm1sp_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&s5kgm1sp->mutex);

	mode = s5kgm1sp_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&s5kgm1sp->mutex);
		return -ENOTTY;
#endif
	} else {
		s5kgm1sp->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(s5kgm1sp->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(s5kgm1sp->vblank, vblank_def,
					 S5KGM1SP_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&s5kgm1sp->mutex);

	return 0;
}

static int s5kgm1sp_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct s5kgm1sp *s5kgm1sp = to_s5kgm1sp(sd);
	const struct s5kgm1sp_mode *mode = s5kgm1sp->cur_mode;

	mutex_lock(&s5kgm1sp->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&s5kgm1sp->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
		fmt->format.field = V4L2_FIELD_NONE;
		/* format info: width/height/data type/virctual channel */
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&s5kgm1sp->mutex);

	return 0;
}

static int s5kgm1sp_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;

	return 0;
}

static int s5kgm1sp_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int s5kgm1sp_enable_test_pattern(struct s5kgm1sp *s5kgm1sp, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | S5KGM1SP_TEST_PATTERN_ENABLE;
	else
		val = S5KGM1SP_TEST_PATTERN_DISABLE;

	return s5kgm1sp_write_reg(s5kgm1sp->client, S5KGM1SP_REG_TEST_PATTERN,
				S5KGM1SP_REG_VALUE_16BIT, val);
}

static int s5kgm1sp_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct s5kgm1sp *s5kgm1sp = to_s5kgm1sp(sd);
	const struct s5kgm1sp_mode *mode = s5kgm1sp->cur_mode;

	mutex_lock(&s5kgm1sp->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&s5kgm1sp->mutex);

	return 0;
}

static int s5kgm1sp_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	struct s5kgm1sp *s5kgm1sp = to_s5kgm1sp(sd);
	const struct s5kgm1sp_mode *mode = s5kgm1sp->cur_mode;
	u32 val = 1 << (S5KGM1SP_LANES - 1) |
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

static void s5kgm1sp_get_module_inf(struct s5kgm1sp *s5kgm1sp,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, S5KGM1SP_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, s5kgm1sp->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, s5kgm1sp->len_name, sizeof(inf->base.lens));
}

static long s5kgm1sp_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct s5kgm1sp *s5kgm1sp = to_s5kgm1sp(sd);
	struct rkmodule_hdr_cfg *hdr;
	u32 i, h, w;
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		s5kgm1sp_get_module_inf(s5kgm1sp, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = s5kgm1sp->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = s5kgm1sp->cur_mode->width;
		h = s5kgm1sp->cur_mode->height;
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				s5kgm1sp->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == ARRAY_SIZE(supported_modes)) {
			dev_err(&s5kgm1sp->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = s5kgm1sp->cur_mode->hts_def -
			    s5kgm1sp->cur_mode->width;
			h = s5kgm1sp->cur_mode->vts_def -
			    s5kgm1sp->cur_mode->height;
			__v4l2_ctrl_modify_range(s5kgm1sp->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(s5kgm1sp->vblank, h,
						 S5KGM1SP_VTS_MAX -
						 s5kgm1sp->cur_mode->height,
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
static long s5kgm1sp_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = s5kgm1sp_ioctl(sd, cmd, inf);
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
			ret = s5kgm1sp_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = s5kgm1sp_ioctl(sd, cmd, hdr);
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
			ret = s5kgm1sp_ioctl(sd, cmd, hdr);
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
			ret = s5kgm1sp_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __s5kgm1sp_start_stream(struct s5kgm1sp *s5kgm1sp)
{
	int ret;

	ret = s5kgm1sp_write_array(s5kgm1sp->client, s5kgm1sp->cur_mode->reg_list);
	if (ret)
		return ret;
	/* In case these controls are set before streaming */
	mutex_unlock(&s5kgm1sp->mutex);
	ret = v4l2_ctrl_handler_setup(&s5kgm1sp->ctrl_handler);
	mutex_lock(&s5kgm1sp->mutex);
	if (ret)
		return ret;
	return s5kgm1sp_write_reg(s5kgm1sp->client, S5KGM1SP_REG_CTRL_MODE,
				S5KGM1SP_REG_VALUE_16BIT, S5KGM1SP_MODE_STREAMING);
}

static int __s5kgm1sp_stop_stream(struct s5kgm1sp *s5kgm1sp)
{
	return s5kgm1sp_write_reg(s5kgm1sp->client, S5KGM1SP_REG_CTRL_MODE,
				S5KGM1SP_REG_VALUE_16BIT, S5KGM1SP_MODE_SW_STANDBY);
}

static int s5kgm1sp_s_stream(struct v4l2_subdev *sd, int on)
{
	struct s5kgm1sp *s5kgm1sp = to_s5kgm1sp(sd);
	struct i2c_client *client = s5kgm1sp->client;
	int ret = 0;

	mutex_lock(&s5kgm1sp->mutex);
	on = !!on;
	if (on == s5kgm1sp->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __s5kgm1sp_start_stream(s5kgm1sp);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__s5kgm1sp_stop_stream(s5kgm1sp);
		pm_runtime_put(&client->dev);
	}

	s5kgm1sp->streaming = on;

unlock_and_return:
	mutex_unlock(&s5kgm1sp->mutex);

	return ret;
}

static int s5kgm1sp_s_power(struct v4l2_subdev *sd, int on)
{
	struct s5kgm1sp *s5kgm1sp = to_s5kgm1sp(sd);
	struct i2c_client *client = s5kgm1sp->client;
	int ret = 0;

	mutex_lock(&s5kgm1sp->mutex);

	/* If the power state is not modified - no work to do. */
	if (s5kgm1sp->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = s5kgm1sp_write_array(s5kgm1sp->client, s5kgm1sp_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		s5kgm1sp->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		s5kgm1sp->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&s5kgm1sp->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 s5kgm1sp_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, S5KGM1SP_XVCLK_FREQ / 1000 / 1000);
}

static int __s5kgm1sp_power_on(struct s5kgm1sp *s5kgm1sp)
{
	int ret;
	u32 delay_us;
	struct device *dev = &s5kgm1sp->client->dev;

	if (!IS_ERR_OR_NULL(s5kgm1sp->pins_default)) {
		ret = pinctrl_select_state(s5kgm1sp->pinctrl,
					   s5kgm1sp->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(s5kgm1sp->xvclk, S5KGM1SP_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(s5kgm1sp->xvclk) != S5KGM1SP_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(s5kgm1sp->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(s5kgm1sp->reset_gpio))
		gpiod_set_value_cansleep(s5kgm1sp->reset_gpio, 0);

	if (!IS_ERR(s5kgm1sp->pwdn_gpio))
		gpiod_set_value_cansleep(s5kgm1sp->pwdn_gpio, 0);

	usleep_range(500, 1000);
	ret = regulator_bulk_enable(S5KGM1SP_NUM_SUPPLIES, s5kgm1sp->supplies);

	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(s5kgm1sp->pwren_gpio))
		gpiod_set_value_cansleep(s5kgm1sp->pwren_gpio, 1);

	usleep_range(1000, 1100);
	if (!IS_ERR(s5kgm1sp->pwdn_gpio))
		gpiod_set_value_cansleep(s5kgm1sp->pwdn_gpio, 1);

	if (!IS_ERR(s5kgm1sp->reset_gpio))
		usleep_range(6000, 8000);
	else
		usleep_range(12000, 16000);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = s5kgm1sp_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(s5kgm1sp->xvclk);

	return ret;
}

static void __s5kgm1sp_power_off(struct s5kgm1sp *s5kgm1sp)
{
	int ret;
	struct device *dev = &s5kgm1sp->client->dev;

	if (!IS_ERR(s5kgm1sp->pwdn_gpio))
		gpiod_set_value_cansleep(s5kgm1sp->pwdn_gpio, 0);
	clk_disable_unprepare(s5kgm1sp->xvclk);
	if (!IS_ERR(s5kgm1sp->reset_gpio))
		gpiod_set_value_cansleep(s5kgm1sp->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(s5kgm1sp->pins_sleep)) {
		ret = pinctrl_select_state(s5kgm1sp->pinctrl,
					   s5kgm1sp->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(S5KGM1SP_NUM_SUPPLIES, s5kgm1sp->supplies);
	if (!IS_ERR(s5kgm1sp->pwren_gpio))
		gpiod_set_value_cansleep(s5kgm1sp->pwren_gpio, 0);
}

static int s5kgm1sp_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5kgm1sp *s5kgm1sp = to_s5kgm1sp(sd);

	return __s5kgm1sp_power_on(s5kgm1sp);
}

static int s5kgm1sp_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5kgm1sp *s5kgm1sp = to_s5kgm1sp(sd);

	__s5kgm1sp_power_off(s5kgm1sp);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int s5kgm1sp_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct s5kgm1sp *s5kgm1sp = to_s5kgm1sp(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct s5kgm1sp_mode *def_mode = &supported_modes[0];

	mutex_lock(&s5kgm1sp->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&s5kgm1sp->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int s5kgm1sp_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fie->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops s5kgm1sp_pm_ops = {
	SET_RUNTIME_PM_OPS(s5kgm1sp_runtime_suspend,
			   s5kgm1sp_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops s5kgm1sp_internal_ops = {
	.open = s5kgm1sp_open,
};
#endif

static const struct v4l2_subdev_core_ops s5kgm1sp_core_ops = {
	.s_power = s5kgm1sp_s_power,
	.ioctl = s5kgm1sp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = s5kgm1sp_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops s5kgm1sp_video_ops = {
	.s_stream = s5kgm1sp_s_stream,
	.g_frame_interval = s5kgm1sp_g_frame_interval,
	.g_mbus_config = s5kgm1sp_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops s5kgm1sp_pad_ops = {
	.enum_mbus_code = s5kgm1sp_enum_mbus_code,
	.enum_frame_size = s5kgm1sp_enum_frame_sizes,
	.enum_frame_interval = s5kgm1sp_enum_frame_interval,
	.get_fmt = s5kgm1sp_get_fmt,
	.set_fmt = s5kgm1sp_set_fmt,
};

static const struct v4l2_subdev_ops s5kgm1sp_subdev_ops = {
	.core	= &s5kgm1sp_core_ops,
	.video	= &s5kgm1sp_video_ops,
	.pad	= &s5kgm1sp_pad_ops,
};

static int s5kgm1sp_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5kgm1sp *s5kgm1sp = container_of(ctrl->handler,
					     struct s5kgm1sp, ctrl_handler);
	struct i2c_client *client = s5kgm1sp->client;
	s64 max;
	int ret = 0;

	/*Propagate change of current control to all related controls*/
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/*Update max exposure while meeting expected vblanking*/
		max = s5kgm1sp->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(s5kgm1sp->exposure,
					 s5kgm1sp->exposure->minimum,
					 max,
					 s5kgm1sp->exposure->step,
					 s5kgm1sp->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		ret = s5kgm1sp_write_reg(s5kgm1sp->client, S5KGM1SP_REG_EXPOSURE,
					 S5KGM1SP_REG_VALUE_16BIT,
					 ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = s5kgm1sp_write_reg(s5kgm1sp->client, S5KGM1SP_REG_GAIN,
					 S5KGM1SP_REG_VALUE_16BIT,
					 ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = s5kgm1sp_write_reg(s5kgm1sp->client, S5KGM1SP_REG_VTS,
					 S5KGM1SP_REG_VALUE_16BIT,
					 ctrl->val + s5kgm1sp->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = s5kgm1sp_enable_test_pattern(s5kgm1sp, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5kgm1sp_ctrl_ops = {
	.s_ctrl = s5kgm1sp_set_ctrl,
};

static int s5kgm1sp_initialize_controls(struct s5kgm1sp *s5kgm1sp)
{
	const struct s5kgm1sp_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &s5kgm1sp->ctrl_handler;
	mode = s5kgm1sp->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &s5kgm1sp->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, S5KGM1SP_PIXEL_RATE, 1, S5KGM1SP_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	s5kgm1sp->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					   h_blank, h_blank, 1, h_blank);
	if (s5kgm1sp->hblank)
		s5kgm1sp->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	s5kgm1sp->vblank = v4l2_ctrl_new_std(handler, &s5kgm1sp_ctrl_ops,
					   V4L2_CID_VBLANK, vblank_def,
					   S5KGM1SP_VTS_MAX - mode->height,
					    1, vblank_def);

	exposure_max = mode->vts_def - 4;
	s5kgm1sp->exposure = v4l2_ctrl_new_std(handler, &s5kgm1sp_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     S5KGM1SP_EXPOSURE_MIN,
					     exposure_max,
					     S5KGM1SP_EXPOSURE_STEP,
					     mode->exp_def);

	s5kgm1sp->anal_gain = v4l2_ctrl_new_std(handler, &s5kgm1sp_ctrl_ops,
					      V4L2_CID_ANALOGUE_GAIN,
					      S5KGM1SP_GAIN_MIN,
					      S5KGM1SP_GAIN_MAX,
					      S5KGM1SP_GAIN_STEP,
					      S5KGM1SP_GAIN_DEFAULT);

	s5kgm1sp->test_pattern =
		v4l2_ctrl_new_std_menu_items(handler,
					     &s5kgm1sp_ctrl_ops,
				V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(s5kgm1sp_test_pattern_menu) - 1,
				0, 0, s5kgm1sp_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&s5kgm1sp->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	s5kgm1sp->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int s5kgm1sp_check_sensor_id(struct s5kgm1sp *s5kgm1sp,
				  struct i2c_client *client)
{
	struct device *dev = &s5kgm1sp->client->dev;
	u32 reg = 0;
	int ret;

	ret = s5kgm1sp_read_reg(client, S5KGM1SP_REG_CHIP_ID,
			      S5KGM1SP_REG_VALUE_16BIT, &reg);
	if (reg != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", reg, ret);
		return -ENODEV;
	}
	dev_info(dev, "detected s5kgm1%04x sensor\n", reg);
	return 0;
}

static int s5kgm1sp_configure_regulators(struct s5kgm1sp *s5kgm1sp)
{
	unsigned int i;

	for (i = 0; i < S5KGM1SP_NUM_SUPPLIES; i++)
		s5kgm1sp->supplies[i].supply = s5kgm1sp_supply_names[i];

	return devm_regulator_bulk_get(&s5kgm1sp->client->dev,
				       S5KGM1SP_NUM_SUPPLIES,
				       s5kgm1sp->supplies);
}

static int s5kgm1sp_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct s5kgm1sp *s5kgm1sp;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;
	u32 i, hdr_mode = 0;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	s5kgm1sp = devm_kzalloc(dev, sizeof(*s5kgm1sp), GFP_KERNEL);
	if (!s5kgm1sp)
		return -ENOMEM;

	of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &s5kgm1sp->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &s5kgm1sp->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &s5kgm1sp->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &s5kgm1sp->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	s5kgm1sp->client = client;
	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if (hdr_mode == supported_modes[i].hdr_mode) {
			s5kgm1sp->cur_mode = &supported_modes[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(supported_modes))
		s5kgm1sp->cur_mode = &supported_modes[0];

	s5kgm1sp->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(s5kgm1sp->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	s5kgm1sp->pwren_gpio = devm_gpiod_get(dev, "pwren", GPIOD_OUT_LOW);
	if (IS_ERR(s5kgm1sp->pwren_gpio))
		dev_warn(dev, "Failed to get pwren-gpios\n");

	s5kgm1sp->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(s5kgm1sp->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	s5kgm1sp->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(s5kgm1sp->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	s5kgm1sp->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(s5kgm1sp->pinctrl)) {
		s5kgm1sp->pins_default =
			pinctrl_lookup_state(s5kgm1sp->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(s5kgm1sp->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		s5kgm1sp->pins_sleep =
			pinctrl_lookup_state(s5kgm1sp->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(s5kgm1sp->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = s5kgm1sp_configure_regulators(s5kgm1sp);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&s5kgm1sp->mutex);

	sd = &s5kgm1sp->subdev;
	v4l2_i2c_subdev_init(sd, client, &s5kgm1sp_subdev_ops);
	ret = s5kgm1sp_initialize_controls(s5kgm1sp);
	if (ret)
		goto err_destroy_mutex;

	ret = __s5kgm1sp_power_on(s5kgm1sp);
	if (ret)
		goto err_free_handler;

	ret = s5kgm1sp_check_sensor_id(s5kgm1sp, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &s5kgm1sp_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	s5kgm1sp->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &s5kgm1sp->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(s5kgm1sp->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 s5kgm1sp->module_index, facing,
		 S5KGM1SP_NAME, dev_name(sd->dev));
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
	__s5kgm1sp_power_off(s5kgm1sp);
err_free_handler:
	v4l2_ctrl_handler_free(&s5kgm1sp->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&s5kgm1sp->mutex);

	return ret;
}

static int s5kgm1sp_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5kgm1sp *s5kgm1sp = to_s5kgm1sp(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&s5kgm1sp->ctrl_handler);
	mutex_destroy(&s5kgm1sp->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__s5kgm1sp_power_off(s5kgm1sp);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id s5kgm1sp_of_match[] = {
	{ .compatible = "samsung,s5kgm1sp" },
	{},
};
MODULE_DEVICE_TABLE(of, s5kgm1sp_of_match);
#endif

static const struct i2c_device_id s5kgm1sp_match_id[] = {
	{ "samsung,s5kgm1sp", 0 },
	{ },
};

static struct i2c_driver s5kgm1sp_i2c_driver = {
	.driver = {
		.name = S5KGM1SP_NAME,
		.pm = &s5kgm1sp_pm_ops,
		.of_match_table = of_match_ptr(s5kgm1sp_of_match),
	},
	.probe		= &s5kgm1sp_probe,
	.remove		= &s5kgm1sp_remove,
	.id_table	= s5kgm1sp_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&s5kgm1sp_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&s5kgm1sp_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("samsung s5kgm1sp sensor driver");
MODULE_LICENSE("GPL v2");
