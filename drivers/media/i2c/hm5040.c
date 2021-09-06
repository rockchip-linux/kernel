// SPDX-License-Identifier: GPL-2.0
/*
 * hm5040 driver
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
#include <linux/version.h>
#include <media/v4l2-async.h>
#include <media/media-entity.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#include <linux/rk-camera-module.h>

/* verify default register values */
#define CHECK_REG_VALUE

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define MIPI_FREQ	210000000U
#define HM5040_PIXEL_RATE		(420000000LL * 2LL * 2LL / 10)
#define HM5040_XVCLK_FREQ		24000000

#define CHIP_ID				0x03bb
#define HM5040_REG_CHIP_ID		0x2016

#define HM5040_REG_CTRL_MODE		0x0100
#define HM5040_MODE_SW_STANDBY		0x00
#define HM5040_MODE_STREAMING		0x01

#define HM5040_REG_EXPOSURE		0x0202
#define HM5040_REG_EXPOSURE_L		0x0203
#define	HM5040_EXPOSURE_MIN		4
#define	HM5040_EXPOSURE_STEP		1
#define HM5040_VTS_MAX			0x7fff

#define HM5040_A_GAIN_H			0x0204
#define HM5040_A_GAIN_L			0x0205
#define HM5040_D_GAIN_GREENR_H		0x020E
#define HM5040_D_GAIN_GREENR_L		0x020F
#define HM5040_D_GAIN_RED_H		0x0210
#define HM5040_D_GAIN_RED_L		0x0211
#define HM5040_D_GAIN_BLUE_H		0x0212
#define HM5040_D_GAIN_BLUE_L		0x0213
#define HM5040_D_GAIN_GREENB_H		0x0214
#define HM5040_D_GAIN_GREENB_L		0x0215

#define HM5040_GAIN_L_MASK		0xff
#define HM5040_GAIN_H_MASK		0x1f
#define HM5040_GAIN_H_SHIFT	8
#define	ANALOG_GAIN_MIN			0x64 /* 1x */
#define	ANALOG_GAIN_MAX			0x63F /* 15.9x */
#define	ANALOG_GAIN_STEP		8
#define	ANALOG_GAIN_DEFAULT		0xC8 /* 2x */

#define HM5040_REG_GROUP	0x0104

#define HM5040_REG_TEST_PATTERN		0x0601
#define	HM5040_TEST_PATTERN_ENABLE	0x08
#define	HM5040_TEST_PATTERN_DISABLE	0x0

#define HM5040_REG_VTS			0x0340

#define REG_NULL			0xFFFF
#define DELAY_MS			0xEEEE	/* Array delay token */

#define HM5040_REG_VALUE_08BIT		1
#define HM5040_REG_VALUE_16BIT		2
#define HM5040_REG_VALUE_24BIT		3

#define HM5040_LANES			2
#define HM5040_BITS_PER_SAMPLE		10

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define HM5040_NAME			"hm5040"

static const char * const hm5040_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define HM5040_NUM_SUPPLIES ARRAY_SIZE(hm5040_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct hm5040_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct hm5040 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*power_gpio;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[HM5040_NUM_SUPPLIES];

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
	const struct hm5040_mode *cur_mode;
	unsigned int lane_num;
	unsigned int cfg_num;
	unsigned int pixel_rate;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	struct rkmodule_awb_cfg	awb_cfg;
};

#define to_hm5040(sd) container_of(sd, struct hm5040, subdev)

struct gain_table_t {
	u16 reg_gain;
	u16 gain_value;
};

/*
 * Xclk 24Mhz
 * Pclk 168Mhz
 * linelength 2600(0xa28)
 * framelength 1952(0x7a0)
 * grabwindow_width 2592
 * grabwindow_height 1944
 * max_framerate 30fps
 * mipi_datarate per lane 840Mbps
 */
static const struct regval hm5040_global_regs[] = {
	{0x0103, 0x01},
	{DELAY_MS, 5},
	{0x0100, 0x00},
	{0x3002, 0x32},
	{0x3016, 0x46},
	{0x3017, 0x29},
	{0x3003, 0x03},
	{0x3045, 0x03},
	{0xFBD7, 0x47},
	{0xFBD8, 0x89},
	{0xFBD9, 0x44},
	{0xFBDA, 0xed},
	{0xFBDB, 0x4c},
	{0xFBDC, 0x6c},
	{0xFBDD, 0x44},
	{0xFBDE, 0xda},
	{0xFBDF, 0x4c},
	{0xFBE0, 0x5f},
	{0xFBE1, 0x44},
	{0xFBE2, 0xe0},
	{0xFBE3, 0x4c},
	{0xFBE4, 0x59},
	{0xFBE5, 0x44},
	{0xFBE6, 0xe1},
	{0xFBE7, 0x4c},
	{0xFBE8, 0x4f},
	{0xFBE9, 0x44},
	{0xFBEA, 0xe6},
	{0xFBEB, 0x4E},
	{0xFBEC, 0xC5},
	{0xFBED, 0x43},
	{0xFBEE, 0xDA},
	{0xFBEF, 0x4E},
	{0xFBF0, 0xB0},
	{0xFBF1, 0x43},
	{0xFBF2, 0x8E},
	{0xFBF3, 0x4E},
	{0xFBF4, 0xAF},
	{0xFBF5, 0x43},
	{0xFBF6, 0x76},
	{0xFBF7, 0x4E},
	{0xFBF8, 0xB7},
	{0xFBF9, 0x43},
	{0xFBFA, 0x80},
	{0xFBFB, 0x4E},
	{0xFBFC, 0xC4},
	{0xFBFD, 0x43},
	{0xFBFE, 0x87},
	{0xFB00, 0x51},
	{0xF800, 0xc0},
	{0xF801, 0x24},
	{0xF802, 0x7c},
	{0xF803, 0xfb},
	{0xF804, 0x7d},
	{0xF805, 0xc7},
	{0xF806, 0x7b},
	{0xF807, 0x10},
	{0xF808, 0x7f},
	{0xF809, 0x72},
	{0xF80A, 0x7e},
	{0xF80B, 0x30},
	{0xF80C, 0x12},
	{0xF80D, 0x09},
	{0xF80E, 0x47},
	{0xF80F, 0xd0},
	{0xF810, 0x24},
	{0xF811, 0x90},
	{0xF812, 0x02},
	{0xF813, 0x05},
	{0xF814, 0xe0},
	{0xF815, 0xf5},
	{0xF816, 0x77},
	{0xF817, 0xe5},
	{0xF818, 0x77},
	{0xF819, 0xc3},
	{0xF81A, 0x94},
	{0xF81B, 0x80},
	{0xF81C, 0x50},
	{0xF81D, 0x08},
	{0xF81E, 0x75},
	{0xF81F, 0x7a},
	{0xF820, 0xfb},
	{0xF821, 0x75},
	{0xF822, 0x7b},
	{0xF823, 0xd7},
	{0xF824, 0x80},
	{0xF825, 0x33},
	{0xF826, 0xe5},
	{0xF827, 0x77},
	{0xF828, 0xc3},
	{0xF829, 0x94},
	{0xF82A, 0xc0},
	{0xF82B, 0x50},
	{0xF82C, 0x08},
	{0xF82D, 0x75},
	{0xF82E, 0x7a},
	{0xF82F, 0xfb},
	{0xF830, 0x75},
	{0xF831, 0x7b},
	{0xF832, 0xdb},
	{0xF833, 0x80},
	{0xF834, 0x24},
	{0xF835, 0xe5},
	{0xF836, 0x77},
	{0xF837, 0xc3},
	{0xF838, 0x94},
	{0xF839, 0xe0},
	{0xF83A, 0x50},
	{0xF83B, 0x08},
	{0xF83C, 0x75},
	{0xF83D, 0x7a},
	{0xF83E, 0xfb},
	{0xF83F, 0x75},
	{0xF840, 0x7b},
	{0xF841, 0xdf},
	{0xF842, 0x80},
	{0xF843, 0x15},
	{0xF844, 0xe5},
	{0xF845, 0x77},
	{0xF846, 0xc3},
	{0xF847, 0x94},
	{0xF848, 0xf0},
	{0xF849, 0x50},
	{0xF84A, 0x08},
	{0xF84B, 0x75},
	{0xF84C, 0x7a},
	{0xF84D, 0xfb},
	{0xF84E, 0x75},
	{0xF84F, 0x7b},
	{0xF850, 0xe3},
	{0xF851, 0x80},
	{0xF852, 0x06},
	{0xF853, 0x75},
	{0xF854, 0x7a},
	{0xF855, 0xfb},
	{0xF856, 0x75},
	{0xF857, 0x7b},
	{0xF858, 0xe7},
	{0xF859, 0xe5},
	{0xF85A, 0x55},
	{0xF85B, 0x7f},
	{0xF85C, 0x00},
	{0xF85D, 0xb4},
	{0xF85E, 0x22},
	{0xF85F, 0x02},
	{0xF860, 0x7f},
	{0xF861, 0x01},
	{0xF862, 0xe5},
	{0xF863, 0x53},
	{0xF864, 0x5f},
	{0xF865, 0x60},
	{0xF866, 0x05},
	{0xF867, 0x74},
	{0xF868, 0x14},
	{0xF869, 0x12},
	{0xF86A, 0xfa},
	{0xF86B, 0x4c},
	{0xF86C, 0x75},
	{0xF86D, 0x7c},
	{0xF86E, 0xfb},
	{0xF86F, 0x75},
	{0xF870, 0x7d},
	{0xF871, 0xc7},
	{0xF872, 0x75},
	{0xF873, 0x7e},
	{0xF874, 0x30},
	{0xF875, 0x75},
	{0xF876, 0x7f},
	{0xF877, 0x62},
	{0xF878, 0xe4},
	{0xF879, 0xf5},
	{0xF87A, 0x77},
	{0xF87B, 0xe5},
	{0xF87C, 0x77},
	{0xF87D, 0xc3},
	{0xF87E, 0x94},
	{0xF87F, 0x08},
	{0xF880, 0x40},
	{0xF881, 0x03},
	{0xF882, 0x02},
	{0xF883, 0xf9},
	{0xF884, 0x0e},
	{0xF885, 0x85},
	{0xF886, 0x7d},
	{0xF887, 0x82},
	{0xF888, 0x85},
	{0xF889, 0x7c},
	{0xF88A, 0x83},
	{0xF88B, 0xe0},
	{0xF88C, 0xfe},
	{0xF88D, 0xa3},
	{0xF88E, 0xe0},
	{0xF88F, 0xff},
	{0xF890, 0x12},
	{0xF891, 0x21},
	{0xF892, 0x22},
	{0xF893, 0x8e},
	{0xF894, 0x78},
	{0xF895, 0x8f},
	{0xF896, 0x79},
	{0xF897, 0x12},
	{0xF898, 0xfa},
	{0xF899, 0x40},
	{0xF89A, 0x12},
	{0xF89B, 0x22},
	{0xF89C, 0x93},
	{0xF89D, 0x50},
	{0xF89E, 0x07},
	{0xF89F, 0xe4},
	{0xF8A0, 0xf5},
	{0xF8A1, 0x78},
	{0xF8A2, 0xf5},
	{0xF8A3, 0x79},
	{0xF8A4, 0x80},
	{0xF8A5, 0x33},
	{0xF8A6, 0x12},
	{0xF8A7, 0xfa},
	{0xF8A8, 0x40},
	{0xF8A9, 0x7b},
	{0xF8AA, 0x01},
	{0xF8AB, 0xaf},
	{0xF8AC, 0x79},
	{0xF8AD, 0xae},
	{0xF8AE, 0x78},
	{0xF8AF, 0x12},
	{0xF8B0, 0x22},
	{0xF8B1, 0x4f},
	{0xF8B2, 0x74},
	{0xF8B3, 0x02},
	{0xF8B4, 0x12},
	{0xF8B5, 0xfa},
	{0xF8B6, 0x4c},
	{0xF8B7, 0x85},
	{0xF8B8, 0x7b},
	{0xF8B9, 0x82},
	{0xF8BA, 0xf5},
	{0xF8BB, 0x83},
	{0xF8BC, 0xe0},
	{0xF8BD, 0xfe},
	{0xF8BE, 0xa3},
	{0xF8BF, 0xe0},
	{0xF8C0, 0xff},
	{0xF8C1, 0x7d},
	{0xF8C2, 0x03},
	{0xF8C3, 0x12},
	{0xF8C4, 0x17},
	{0xF8C5, 0xd8},
	{0xF8C6, 0x12},
	{0xF8C7, 0x1b},
	{0xF8C8, 0x9b},
	{0xF8C9, 0x8e},
	{0xF8CA, 0x78},
	{0xF8CB, 0x8f},
	{0xF8CC, 0x79},
	{0xF8CD, 0x74},
	{0xF8CE, 0xfe},
	{0xF8CF, 0x25},
	{0xF8D0, 0x7b},
	{0xF8D1, 0xf5},
	{0xF8D2, 0x7b},
	{0xF8D3, 0x74},
	{0xF8D4, 0xff},
	{0xF8D5, 0x35},
	{0xF8D6, 0x7a},
	{0xF8D7, 0xf5},
	{0xF8D8, 0x7a},
	{0xF8D9, 0x78},
	{0xF8DA, 0x24},
	{0xF8DB, 0xe6},
	{0xF8DC, 0xff},
	{0xF8DD, 0xc3},
	{0xF8DE, 0x74},
	{0xF8DF, 0x20},
	{0xF8E0, 0x9f},
	{0xF8E1, 0x7e},
	{0xF8E2, 0x00},
	{0xF8E3, 0x25},
	{0xF8E4, 0x79},
	{0xF8E5, 0xff},
	{0xF8E6, 0xee},
	{0xF8E7, 0x35},
	{0xF8E8, 0x78},
	{0xF8E9, 0x85},
	{0xF8EA, 0x7f},
	{0xF8EB, 0x82},
	{0xF8EC, 0x85},
	{0xF8ED, 0x7e},
	{0xF8EE, 0x83},
	{0xF8EF, 0xf0},
	{0xF8F0, 0xa3},
	{0xF8F1, 0xef},
	{0xF8F2, 0xf0},
	{0xF8F3, 0x05},
	{0xF8F4, 0x77},
	{0xF8F5, 0x74},
	{0xF8F6, 0x02},
	{0xF8F7, 0x25},
	{0xF8F8, 0x7d},
	{0xF8F9, 0xf5},
	{0xF8FA, 0x7d},
	{0xF8FB, 0xe4},
	{0xF8FC, 0x35},
	{0xF8FD, 0x7c},
	{0xF8FE, 0xf5},
	{0xF8FF, 0x7c},
	{0xF900, 0x74},
	{0xF901, 0x02},
	{0xF902, 0x25},
	{0xF903, 0x7f},
	{0xF904, 0xf5},
	{0xF905, 0x7f},
	{0xF906, 0xe4},
	{0xF907, 0x35},
	{0xF908, 0x7e},
	{0xF909, 0xf5},
	{0xF90A, 0x7e},
	{0xF90B, 0x02},
	{0xF90C, 0xf8},
	{0xF90D, 0x7b},
	{0xF90E, 0x22},
	{0xF90F, 0x90},
	{0xF910, 0x30},
	{0xF911, 0x47},
	{0xF912, 0x74},
	{0xF913, 0x98},
	{0xF914, 0xf0},
	{0xF915, 0x90},
	{0xF916, 0x30},
	{0xF917, 0x36},
	{0xF918, 0x74},
	{0xF919, 0x1e},
	{0xF91A, 0xf0},
	{0xF91B, 0x90},
	{0xF91C, 0x30},
	{0xF91D, 0x42},
	{0xF91E, 0x74},
	{0xF91F, 0x24},
	{0xF920, 0xf0},
	{0xF921, 0xe5},
	{0xF922, 0x53},
	{0xF923, 0x60},
	{0xF924, 0x42},
	{0xF925, 0x78},
	{0xF926, 0x2b},
	{0xF927, 0x76},
	{0xF928, 0x01},
	{0xF929, 0xe5},
	{0xF92A, 0x55},
	{0xF92B, 0xb4},
	{0xF92C, 0x22},
	{0xF92D, 0x17},
	{0xF92E, 0x90},
	{0xF92F, 0x30},
	{0xF930, 0x36},
	{0xF931, 0x74},
	{0xF932, 0x46},
	{0xF933, 0xf0},
	{0xF934, 0x78},
	{0xF935, 0x28},
	{0xF936, 0x76},
	{0xF937, 0x31},
	{0xF938, 0x90},
	{0xF939, 0x30},
	{0xF93A, 0x0e},
	{0xF93B, 0xe0},
	{0xF93C, 0xc3},
	{0xF93D, 0x13},
	{0xF93E, 0x30},
	{0xF93F, 0xe0},
	{0xF940, 0x04},
	{0xF941, 0x78},
	{0xF942, 0x26},
	{0xF943, 0x76},
	{0xF944, 0x40},
	{0xF945, 0xe5},
	{0xF946, 0x55},
	{0xF947, 0xb4},
	{0xF948, 0x44},
	{0xF949, 0x21},
	{0xF94A, 0x90},
	{0xF94B, 0x30},
	{0xF94C, 0x47},
	{0xF94D, 0x74},
	{0xF94E, 0x9a},
	{0xF94F, 0xf0},
	{0xF950, 0x90},
	{0xF951, 0x30},
	{0xF952, 0x42},
	{0xF953, 0x74},
	{0xF954, 0x64},
	{0xF955, 0xf0},
	{0xF956, 0x90},
	{0xF957, 0x30},
	{0xF958, 0x0e},
	{0xF959, 0xe0},
	{0xF95A, 0x13},
	{0xF95B, 0x13},
	{0xF95C, 0x54},
	{0xF95D, 0x3f},
	{0xF95E, 0x30},
	{0xF95F, 0xe0},
	{0xF960, 0x0a},
	{0xF961, 0x78},
	{0xF962, 0x24},
	{0xF963, 0xe4},
	{0xF964, 0xf6},
	{0xF965, 0x80},
	{0xF966, 0x04},
	{0xF967, 0x78},
	{0xF968, 0x2b},
	{0xF969, 0xe4},
	{0xF96A, 0xf6},
	{0xF96B, 0x90},
	{0xF96C, 0x30},
	{0xF96D, 0x88},
	{0xF96E, 0x02},
	{0xF96F, 0x1d},
	{0xF970, 0x4f},
	{0xF971, 0x22},
	{0xF972, 0x90},
	{0xF973, 0x0c},
	{0xF974, 0x1a},
	{0xF975, 0xe0},
	{0xF976, 0x30},
	{0xF977, 0xe2},
	{0xF978, 0x18},
	{0xF979, 0x90},
	{0xF97A, 0x33},
	{0xF97B, 0x68},
	{0xF97C, 0xe0},
	{0xF97D, 0x64},
	{0xF97E, 0x05},
	{0xF97F, 0x70},
	{0xF980, 0x2f},
	{0xF981, 0x90},
	{0xF982, 0x30},
	{0xF983, 0x38},
	{0xF984, 0xe0},
	{0xF985, 0x70},
	{0xF986, 0x02},
	{0xF987, 0xa3},
	{0xF988, 0xe0},
	{0xF989, 0xc3},
	{0xF98A, 0x70},
	{0xF98B, 0x01},
	{0xF98C, 0xd3},
	{0xF98D, 0x40},
	{0xF98E, 0x21},
	{0xF98F, 0x80},
	{0xF990, 0x1b},
	{0xF991, 0x90},
	{0xF992, 0x33},
	{0xF993, 0x68},
	{0xF994, 0xe0},
	{0xF995, 0xb4},
	{0xF996, 0x05},
	{0xF997, 0x18},
	{0xF998, 0xc3},
	{0xF999, 0x90},
	{0xF99A, 0x30},
	{0xF99B, 0x3b},
	{0xF99C, 0xe0},
	{0xF99D, 0x94},
	{0xF99E, 0x0d},
	{0xF99F, 0x90},
	{0xF9A0, 0x30},
	{0xF9A1, 0x3a},
	{0xF9A2, 0xe0},
	{0xF9A3, 0x94},
	{0xF9A4, 0x00},
	{0xF9A5, 0x50},
	{0xF9A6, 0x02},
	{0xF9A7, 0x80},
	{0xF9A8, 0x01},
	{0xF9A9, 0xc3},
	{0xF9AA, 0x40},
	{0xF9AB, 0x04},
	{0xF9AC, 0x75},
	{0xF9AD, 0x10},
	{0xF9AE, 0x01},
	{0xF9AF, 0x22},
	{0xF9B0, 0x02},
	{0xF9B1, 0x16},
	{0xF9B2, 0xe1},
	{0xF9B3, 0x22},
	{0xF9B4, 0x90},
	{0xF9B5, 0xff},
	{0xF9B6, 0x33},
	{0xF9B7, 0xe0},
	{0xF9B8, 0x90},
	{0xF9B9, 0xff},
	{0xF9BA, 0x34},
	{0xF9BB, 0xe0},
	{0xF9BC, 0x60},
	{0xF9BD, 0x0d},
	{0xF9BE, 0x7c},
	{0xF9BF, 0xfb},
	{0xF9C0, 0x7d},
	{0xF9C1, 0xd7},
	{0xF9C2, 0x7b},
	{0xF9C3, 0x28},
	{0xF9C4, 0x7f},
	{0xF9C5, 0x34},
	{0xF9C6, 0x7e},
	{0xF9C7, 0xff},
	{0xF9C8, 0x12},
	{0xF9C9, 0x09},
	{0xF9CA, 0x47},
	{0xF9CB, 0x7f},
	{0xF9CC, 0x20},
	{0xF9CD, 0x7e},
	{0xF9CE, 0x01},
	{0xF9CF, 0x7d},
	{0xF9D0, 0x00},
	{0xF9D1, 0x7c},
	{0xF9D2, 0x00},
	{0xF9D3, 0x12},
	{0xF9D4, 0x12},
	{0xF9D5, 0xa4},
	{0xF9D6, 0xe4},
	{0xF9D7, 0x90},
	{0xF9D8, 0x3e},
	{0xF9D9, 0x44},
	{0xF9DA, 0xf0},
	{0xF9DB, 0x02},
	{0xF9DC, 0x16},
	{0xF9DD, 0x7e},
	{0xF9DE, 0x22},
	{0xF9DF, 0xe5},
	{0xF9E0, 0x44},
	{0xF9E1, 0x60},
	{0xF9E2, 0x10},
	{0xF9E3, 0x90},
	{0xF9E4, 0xf6},
	{0xF9E5, 0x2c},
	{0xF9E6, 0x74},
	{0xF9E7, 0x04},
	{0xF9E8, 0xf0},
	{0xF9E9, 0x90},
	{0xF9EA, 0xf6},
	{0xF9EB, 0x34},
	{0xF9EC, 0xf0},
	{0xF9ED, 0x90},
	{0xF9EE, 0xf6},
	{0xF9EF, 0x3c},
	{0xF9F0, 0xf0},
	{0xF9F1, 0x80},
	{0xF9F2, 0x0e},
	{0xF9F3, 0x90},
	{0xF9F4, 0xf5},
	{0xF9F5, 0xc0},
	{0xF9F6, 0x74},
	{0xF9F7, 0x04},
	{0xF9F8, 0xf0},
	{0xF9F9, 0x90},
	{0xF9FA, 0xf5},
	{0xF9FB, 0xc8},
	{0xF9FC, 0xf0},
	{0xF9FD, 0x90},
	{0xF9FE, 0xf5},
	{0xF9FF, 0xd0},
	{0xFA00, 0xf0},
	{0xFA01, 0x90},
	{0xFA02, 0xfb},
	{0xFA03, 0x7f},
	{0xFA04, 0x02},
	{0xFA05, 0x19},
	{0xFA06, 0x0b},
	{0xFA07, 0x22},
	{0xFA08, 0x90},
	{0xFA09, 0x0c},
	{0xFA0A, 0x1a},
	{0xFA0B, 0xe0},
	{0xFA0C, 0x20},
	{0xFA0D, 0xe2},
	{0xFA0E, 0x15},
	{0xFA0F, 0xe4},
	{0xFA10, 0x90},
	{0xFA11, 0x30},
	{0xFA12, 0xf8},
	{0xFA13, 0xf0},
	{0xFA14, 0xa3},
	{0xFA15, 0xf0},
	{0xFA16, 0x90},
	{0xFA17, 0x30},
	{0xFA18, 0xf1},
	{0xFA19, 0xe0},
	{0xFA1A, 0x44},
	{0xFA1B, 0x08},
	{0xFA1C, 0xf0},
	{0xFA1D, 0x90},
	{0xFA1E, 0x30},
	{0xFA1F, 0xf0},
	{0xFA20, 0xe0},
	{0xFA21, 0x44},
	{0xFA22, 0x08},
	{0xFA23, 0xf0},
	{0xFA24, 0x02},
	{0xFA25, 0x03},
	{0xFA26, 0xde},
	{0xFA27, 0x22},
	{0xFA28, 0x90},
	{0xFA29, 0x0c},
	{0xFA2A, 0x1a},
	{0xFA2B, 0xe0},
	{0xFA2C, 0x30},
	{0xFA2D, 0xe2},
	{0xFA2E, 0x0d},
	{0xFA2F, 0xe0},
	{0xFA30, 0x20},
	{0xFA31, 0xe0},
	{0xFA32, 0x06},
	{0xFA33, 0x90},
	{0xFA34, 0xfb},
	{0xFA35, 0x85},
	{0xFA36, 0x74},
	{0xFA37, 0x00},
	{0xFA38, 0xa5},
	{0xFA39, 0x12},
	{0xFA3A, 0x16},
	{0xFA3B, 0xa0},
	{0xFA3C, 0x02},
	{0xFA3D, 0x18},
	{0xFA3E, 0xac},
	{0xFA3F, 0x22},
	{0xFA40, 0x85},
	{0xFA41, 0x7b},
	{0xFA42, 0x82},
	{0xFA43, 0x85},
	{0xFA44, 0x7a},
	{0xFA45, 0x83},
	{0xFA46, 0xe0},
	{0xFA47, 0xfc},
	{0xFA48, 0xa3},
	{0xFA49, 0xe0},
	{0xFA4A, 0xfd},
	{0xFA4B, 0x22},
	{0xFA4C, 0x25},
	{0xFA4D, 0x7b},
	{0xFA4E, 0xf5},
	{0xFA4F, 0x7b},
	{0xFA50, 0xe4},
	{0xFA51, 0x35},
	{0xFA52, 0x7a},
	{0xFA53, 0xf5},
	{0xFA54, 0x7a},
	{0xFA55, 0x22},
	{0xFA56, 0xc0},
	{0xFA57, 0xd0},
	{0xFA58, 0x90},
	{0xFA59, 0x35},
	{0xFA5A, 0xb5},
	{0xFA5B, 0xe0},
	{0xFA5C, 0x54},
	{0xFA5D, 0xfc},
	{0xFA5E, 0x44},
	{0xFA5F, 0x01},
	{0xFA60, 0xf0},
	{0xFA61, 0x12},
	{0xFA62, 0x1f},
	{0xFA63, 0x5f},
	{0xFA64, 0xd0},
	{0xFA65, 0xd0},
	{0xFA66, 0x02},
	{0xFA67, 0x0a},
	{0xFA68, 0x16},
	{0xFA69, 0x22},
	{0xFA6A, 0x90},
	{0xFA6B, 0x0c},
	{0xFA6C, 0x1a},
	{0xFA6D, 0xe0},
	{0xFA6E, 0x20},
	{0xFA6F, 0xe0},
	{0xFA70, 0x06},
	{0xFA71, 0x90},
	{0xFA72, 0xfb},
	{0xFA73, 0x85},
	{0xFA74, 0x74},
	{0xFA75, 0x00},
	{0xFA76, 0xa5},
	{0xFA77, 0xe5},
	{0xFA78, 0x10},
	{0xFA79, 0x02},
	{0xFA7A, 0x1e},
	{0xFA7B, 0x8f},
	{0xFA7C, 0x22},
	{0xFA7D, 0x90},
	{0xFA7E, 0xfb},
	{0xFA7F, 0x85},
	{0xFA80, 0x74},
	{0xFA81, 0x00},
	{0xFA82, 0xa5},
	{0xFA83, 0xe5},
	{0xFA84, 0x1a},
	{0xFA85, 0x60},
	{0xFA86, 0x03},
	{0xFA87, 0x02},
	{0xFA88, 0x17},
	{0xFA89, 0x47},
	{0xFA8A, 0x22},
	{0xFA8B, 0x90},
	{0xFA8C, 0xfb},
	{0xFA8D, 0x84},
	{0xFA8E, 0x02},
	{0xFA8F, 0x18},
	{0xFA90, 0xd9},
	{0xFA91, 0x22},
	{0xFA92, 0x02},
	{0xFA93, 0x1f},
	{0xFA94, 0xb1},
	{0xFA95, 0x22},
	{0x35D8, 0x01},
	{0x35D9, 0x0F},
	{0x35DA, 0x01},
	{0x35DB, 0x72},
	{0x35DC, 0x01},
	{0x35DD, 0xB4},
	{0x35DE, 0x01},
	{0x35DF, 0xDF},
	{0x35E0, 0x02},
	{0x35E1, 0x08},
	{0x35E2, 0x02},
	{0x35E3, 0x28},
	{0x35E4, 0x02},
	{0x35E5, 0x56},
	{0x35E6, 0x02},
	{0x35E7, 0x6A},
	{0x35E8, 0x02},
	{0x35E9, 0x7D},
	{0x35EA, 0x02},
	{0x35EB, 0x8B},
	{0x35EC, 0x02},
	{0x35ED, 0x92},
	{0x35EF, 0x22},
	{0x35F1, 0x23},
	{0x35F3, 0x22},
	{0x35F6, 0x19},
	{0x35F7, 0x55},
	{0x35F8, 0x1D},
	{0x35F9, 0x4C},
	{0x35FA, 0x16},
	{0x35FB, 0xC7},
	{0x35FC, 0x1A},
	{0x35FD, 0xA0},
	{0x35FE, 0x18},
	{0x35FF, 0xD6},
	{0x3600, 0x03},
	{0x3601, 0xD4},
	{0x3602, 0x18},
	{0x3603, 0x8A},
	{0x3604, 0x0A},
	{0x3605, 0x0D},
	{0x3606, 0x1E},
	{0x3607, 0x8D},
	{0x3608, 0x17},
	{0x3609, 0x43},
	{0x360A, 0x19},
	{0x360B, 0x16},
	{0x360C, 0x1F},
	{0x360D, 0xAD},
	{0x360E, 0x19},
	{0x360F, 0x08},
	{0x3610, 0x14},
	{0x3611, 0x26},
	{0x3612, 0x1A},
	{0x3613, 0xB3},
	{0x35D2, 0x7F},
	{0x35D3, 0xFF},
	{0x35D4, 0x70},
	{0x35D0, 0x01},
	{0x3E44, 0x01},
	{0x0111, 0x02},
	{0x0114, 0x01},
	{0x2136, 0x0C},
	{0x2137, 0x00},
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x3016, 0x46},
	{0x3017, 0x29},
	{0x3003, 0x03},
	{0x3045, 0x03},
	{0x3047, 0x98},
	{0x0305, 0x04},
	{0x0306, 0x00},
	{0x0307, 0x64},
	{0x0301, 0x0A},
	{0x0309, 0x0A},
	{0x0340, 0x08},
	{0x0341, 0x24},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0A},
	{0x0349, 0x27},
	{0x034A, 0x07},
	{0x034B, 0x9F},
	{0x034C, 0x0A},
	{0x034D, 0x28},
	{0x034E, 0x07},
	{0x034F, 0xA0},
	{0x0383, 0x01},
	{0x0387, 0x01},
	{0x0202, 0x07},
	{0x0203, 0xad},
	{0x0205, 0xC0},
	{0x0900, 0x00},
	{0x0901, 0x00},
	{0x0902, 0x00},
	{0x040C, 0x0A},
	{0x040D, 0x28},
	{0x040E, 0x07},
	{0x040F, 0xA0},
	{0x0101, 0x00},
	//{0x0100, 0x01},

	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * Pclk 168Mhz
 * linelength 2600(0xa28)
 * framelength 1952(0x7a0)
 * grabwindow_width 2592
 * grabwindow_height 1944
 * max_framerate 30fps
 * mipi_datarate per lane 840Mbps
 */
static const struct regval hm5040_2592x1944_regs_2lane[] = {
	// 2592x1944 30fps 2 lane MIPI 840Mbps/lane
	{0x0100, 0x00},
	{0x0900, 0x00},
	{0x0901, 0x00},
	{0x0300, 0x00},
	{0x0302, 0x00},
	{0x0303, 0x01},
	{0x0304, 0x00},
	{0x0308, 0x00},
	{0x030a, 0x00},
	{0x030b, 0x01},
	{0x0340, 0x07},
	{0x0341, 0xc4},
	{0x034C, 0x0A},
	{0x034D, 0x28},
	{0x034E, 0x07},
	{0x034F, 0xA0},
	{0x040C, 0x0A},
	{0x040D, 0x28},
	{0x040E, 0x07},
	{0x040F, 0xA0},
	{0x0202, 0x0e},
	{0x0203, 0x50},
	{0x0205, 0xC0},
	//{0x0100, 0x01},
	{REG_NULL, 0x00},
};

static const struct hm5040_mode supported_modes_2lane[] = {
	{
		.width = 2592,
		.height = 1944,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x07a0,
		.hts_def = 0x0abe,
		.vts_def = 0x07c4,
		.reg_list = hm5040_2592x1944_regs_2lane,
	},
};

static const struct hm5040_mode *supported_modes;

static const s64 link_freq_menu_items[] = {
	MIPI_FREQ
};

static const char * const hm5040_test_pattern_menu[] = {
	"Disabled",
	"Solid color",
	"100% color bars",
	"Fade to gray color bars",
	"PN9",
};

/* Write registers up to 4 at a time */
static int hm5040_write_reg(struct i2c_client *client, u16 reg,
			    u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	dev_dbg(&client->dev, "%s(%d) enter!\n", __func__, __LINE__);
	dev_info(&client->dev, "write reg(0x%x val:0x%x)!\n", reg, val);

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

	if (i2c_master_send(client, buf, len + 2) != len + 2) {
		dev_err(&client->dev,
			   "write reg(0x%x val:0x%x)failed !\n", reg, val);
		return -EIO;
	}
	return 0;
}

static int hm5040_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	int i, delay_ms, ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		if (regs[i].addr == DELAY_MS) {
			delay_ms = regs[i].val;
			dev_info(&client->dev, "delay(%d) ms !\n", delay_ms);
			usleep_range(1000 * delay_ms, 1000 * delay_ms + 100);
			continue;
		}
		ret = hm5040_write_reg(client, regs[i].addr,
				       HM5040_REG_VALUE_08BIT, regs[i].val);
		if (ret)
			dev_err(&client->dev, "%s failed !\n", __func__);
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int hm5040_read_reg(struct i2c_client *client, u16 reg,
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

/* Check Register value */
#ifdef CHECK_REG_VALUE
static int hm5040_reg_verify(struct i2c_client *client,
				const struct regval *regs)
{
	u32 i;
	int ret = 0;
	u32 value;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		ret = hm5040_read_reg(client, regs[i].addr,
			  HM5040_REG_VALUE_08BIT, &value);
		if (value != regs[i].val) {
			dev_info(&client->dev, "%s: 0x%04x is 0x%x instead of 0x%x\n",
				  __func__, regs[i].addr, value, regs[i].val);
		}
	}
	return ret;
}
#endif

static int hm5040_get_reso_dist(const struct hm5040_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct hm5040_mode *
hm5040_find_best_fit(struct hm5040 *hm5040,
			struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < hm5040->cfg_num; i++) {
		dist = hm5040_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int hm5040_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct hm5040 *hm5040 = to_hm5040(sd);
	const struct hm5040_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&hm5040->mutex);

	mode = hm5040_find_best_fit(hm5040, fmt);
	fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&hm5040->mutex);
		return -ENOTTY;
#endif
	} else {
		hm5040->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(hm5040->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(hm5040->vblank, vblank_def,
					 HM5040_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&hm5040->mutex);

	return 0;
}

static int hm5040_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct hm5040 *hm5040 = to_hm5040(sd);
	const struct hm5040_mode *mode = hm5040->cur_mode;

	mutex_lock(&hm5040->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&hm5040->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&hm5040->mutex);

	return 0;
}

static int hm5040_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;

	return 0;
}

static int hm5040_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct hm5040 *hm5040 = to_hm5040(sd);

	if (fse->index >= hm5040->cfg_num)
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int hm5040_enable_test_pattern(struct hm5040 *hm5040, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | HM5040_TEST_PATTERN_ENABLE;
	else
		val = HM5040_TEST_PATTERN_DISABLE;

	return hm5040_write_reg(hm5040->client, HM5040_REG_TEST_PATTERN,
				HM5040_REG_VALUE_08BIT, val);
}

static int hm5040_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct hm5040 *hm5040 = to_hm5040(sd);
	const struct hm5040_mode *mode = hm5040->cur_mode;

	mutex_lock(&hm5040->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&hm5040->mutex);

	return 0;
}

static void hm5040_get_module_inf(struct hm5040 *hm5040,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, HM5040_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, hm5040->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, hm5040->len_name, sizeof(inf->base.lens));
}

static void hm5040_set_awb_cfg(struct hm5040 *hm5040,
				 struct rkmodule_awb_cfg *cfg)
{
	mutex_lock(&hm5040->mutex);
	memcpy(&hm5040->awb_cfg, cfg, sizeof(*cfg));
	mutex_unlock(&hm5040->mutex);
}

static long hm5040_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct hm5040 *hm5040 = to_hm5040(sd);
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		hm5040_get_module_inf(hm5040, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_AWB_CFG:
		hm5040_set_awb_cfg(hm5040, (struct rkmodule_awb_cfg *)arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = hm5040_write_reg(hm5040->client, HM5040_REG_CTRL_MODE,
				HM5040_REG_VALUE_08BIT, HM5040_MODE_STREAMING);
		else
			ret = hm5040_write_reg(hm5040->client, HM5040_REG_CTRL_MODE,
				HM5040_REG_VALUE_08BIT, HM5040_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long hm5040_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *awb_cfg;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = hm5040_ioctl(sd, cmd, inf);
		if (!ret) {
			ret = copy_to_user(up, inf, sizeof(*inf));
			if (ret)
				ret = -EFAULT;
		}
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		awb_cfg = kzalloc(sizeof(*awb_cfg), GFP_KERNEL);
		if (!awb_cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(awb_cfg, up, sizeof(*awb_cfg));
		if (!ret)
			ret = hm5040_ioctl(sd, cmd, awb_cfg);
		else
			ret = -EFAULT;
		kfree(awb_cfg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = hm5040_ioctl(sd, cmd, &stream);
		else
			ret = -EFAULT;
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __hm5040_start_stream(struct hm5040 *hm5040)
{
	int ret;

	ret = hm5040_write_array(hm5040->client, hm5040->cur_mode->reg_list);
	if (ret)
		return ret;

#ifdef CHECK_REG_VALUE
	usleep_range(10000, 20000);
	/*  verify default values to make sure everything has */
	/*  been written correctly as expected */
	dev_info(&hm5040->client->dev, "%s:Check register value!\n",
				__func__);
	ret = hm5040_reg_verify(hm5040->client, hm5040_global_regs);
	if (ret)
		return ret;

	ret = hm5040_reg_verify(hm5040->client, hm5040->cur_mode->reg_list);
	if (ret)
		return ret;
#endif

	/* In case these controls are set before streaming */
	mutex_unlock(&hm5040->mutex);
	ret = v4l2_ctrl_handler_setup(&hm5040->ctrl_handler);
	mutex_lock(&hm5040->mutex);
	if (ret)
		return ret;

	ret = hm5040_write_reg(hm5040->client, HM5040_REG_CTRL_MODE,
				HM5040_REG_VALUE_08BIT, HM5040_MODE_STREAMING);
	return ret;
}

static int __hm5040_stop_stream(struct hm5040 *hm5040)
{
	return hm5040_write_reg(hm5040->client, HM5040_REG_CTRL_MODE,
				HM5040_REG_VALUE_08BIT, HM5040_MODE_SW_STANDBY);
}

static int hm5040_s_stream(struct v4l2_subdev *sd, int on)
{
	struct hm5040 *hm5040 = to_hm5040(sd);
	struct i2c_client *client = hm5040->client;
	int ret = 0;

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
				hm5040->cur_mode->width,
				hm5040->cur_mode->height,
		DIV_ROUND_CLOSEST(hm5040->cur_mode->max_fps.denominator,
		hm5040->cur_mode->max_fps.numerator));

	mutex_lock(&hm5040->mutex);
	on = !!on;
	if (on == hm5040->streaming)
		goto unlock_and_return;

	if (on) {
		dev_info(&client->dev, "stream on!!!\n");
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __hm5040_start_stream(hm5040);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		dev_info(&client->dev, "stream off!!!\n");
		__hm5040_stop_stream(hm5040);
		pm_runtime_put(&client->dev);
	}

	hm5040->streaming = on;

unlock_and_return:
	mutex_unlock(&hm5040->mutex);

	return ret;
}

static int hm5040_s_power(struct v4l2_subdev *sd, int on)
{
	struct hm5040 *hm5040 = to_hm5040(sd);
	struct i2c_client *client = hm5040->client;
	int ret = 0;

	dev_info(&client->dev, "%s(%d) on(%d)\n", __func__, __LINE__, on);
	mutex_lock(&hm5040->mutex);

	/* If the power state is not modified - no work to do. */
	if (hm5040->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = hm5040_write_array(hm5040->client, hm5040_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		hm5040->power_on = true;
		/* export gpio */
		if (!IS_ERR(hm5040->reset_gpio))
			gpiod_export(hm5040->reset_gpio, false);
		if (!IS_ERR(hm5040->pwdn_gpio))
			gpiod_export(hm5040->pwdn_gpio, false);
	} else {
		pm_runtime_put(&client->dev);
		hm5040->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&hm5040->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 hm5040_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, HM5040_XVCLK_FREQ / 1000 / 1000);
}

static int __hm5040_power_on(struct hm5040 *hm5040)
{
	int ret;
	u32 delay_us;
	struct device *dev = &hm5040->client->dev;

	if (!IS_ERR(hm5040->power_gpio))
		gpiod_set_value_cansleep(hm5040->power_gpio, 1);

	usleep_range(1000, 2000);

	if (!IS_ERR_OR_NULL(hm5040->pins_default)) {
		ret = pinctrl_select_state(hm5040->pinctrl,
					   hm5040->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(hm5040->xvclk, HM5040_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(hm5040->xvclk) != HM5040_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(hm5040->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	ret = regulator_bulk_enable(HM5040_NUM_SUPPLIES, hm5040->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(hm5040->reset_gpio))
		gpiod_set_value_cansleep(hm5040->reset_gpio, 1);

	usleep_range(1000, 2000);
	if (!IS_ERR(hm5040->pwdn_gpio))
		gpiod_set_value_cansleep(hm5040->pwdn_gpio, 1);

	/* export gpio */
	if (!IS_ERR(hm5040->reset_gpio))
		gpiod_export(hm5040->reset_gpio, false);
	if (!IS_ERR(hm5040->pwdn_gpio))
		gpiod_export(hm5040->pwdn_gpio, false);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = hm5040_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	usleep_range(10000, 20000);
	return 0;

disable_clk:
	clk_disable_unprepare(hm5040->xvclk);

	return ret;
}

static void __hm5040_power_off(struct hm5040 *hm5040)
{
	int ret;
	struct device *dev = &hm5040->client->dev;

	if (!IS_ERR(hm5040->pwdn_gpio))
		gpiod_set_value_cansleep(hm5040->pwdn_gpio, 0);
	clk_disable_unprepare(hm5040->xvclk);
	if (!IS_ERR(hm5040->reset_gpio))
		gpiod_set_value_cansleep(hm5040->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(hm5040->pins_sleep)) {
		ret = pinctrl_select_state(hm5040->pinctrl,
					   hm5040->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	if (!IS_ERR(hm5040->power_gpio))
		gpiod_set_value_cansleep(hm5040->power_gpio, 0);

	regulator_bulk_disable(HM5040_NUM_SUPPLIES, hm5040->supplies);
}

static int hm5040_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hm5040 *hm5040 = to_hm5040(sd);

	return __hm5040_power_on(hm5040);
}

static int hm5040_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hm5040 *hm5040 = to_hm5040(sd);

	__hm5040_power_off(hm5040);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int hm5040_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct hm5040 *hm5040 = to_hm5040(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct hm5040_mode *def_mode = &supported_modes[0];

	mutex_lock(&hm5040->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&hm5040->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int hm5040_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct hm5040 *hm5040 = to_hm5040(sd);

	if (fie->index >= hm5040->cfg_num)
		return -EINVAL;

	if (fie->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static int hm5040_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	u32 val = 0;

	val = 1 << (HM5040_LANES - 1) |
	      V4L2_MBUS_CSI2_CHANNEL_0 |
	      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	config->type = V4L2_MBUS_CSI2;
	config->flags = val;

	return 0;
}

static const struct dev_pm_ops hm5040_pm_ops = {
	SET_RUNTIME_PM_OPS(hm5040_runtime_suspend,
			   hm5040_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops hm5040_internal_ops = {
	.open = hm5040_open,
};
#endif

static const struct v4l2_subdev_core_ops hm5040_core_ops = {
	.s_power = hm5040_s_power,
	.ioctl = hm5040_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = hm5040_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops hm5040_video_ops = {
	.s_stream = hm5040_s_stream,
	.g_frame_interval = hm5040_g_frame_interval,
	.g_mbus_config = hm5040_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops hm5040_pad_ops = {
	.enum_mbus_code = hm5040_enum_mbus_code,
	.enum_frame_size = hm5040_enum_frame_sizes,
	.enum_frame_interval = hm5040_enum_frame_interval,
	.get_fmt = hm5040_get_fmt,
	.set_fmt = hm5040_set_fmt,
};

static const struct v4l2_subdev_ops hm5040_subdev_ops = {
	.core	= &hm5040_core_ops,
	.video	= &hm5040_video_ops,
	.pad	= &hm5040_pad_ops,
};

/* real analog gain * 100 values*/
const struct gain_table_t hm5040_again_table[] = {
//0x0204/0x0205
	{0x00, 100},
	{0x10, 107},
	{0x20, 110},
	{0x30, 120},
	{0x40, 130},
	{0x50, 150},
	{0x60, 160},
	{0x70, 180},
	{0x80, 200},
	{0x90, 230},
	{0xA0, 270},
	{0xB0, 320},
	{0xC0, 400},
	{0xD0, 533},
	{0xE0, 800},
	{0xE4, 910},
	{0xE8, 1070},
	{0xEC, 1280},
	{0xF0, 1600},
};

const struct gain_table_t hm5040_dgain_table[] = {
	{0x00, 100},
	{0x08, 103},
	{0x10, 106},
	{0x18, 109},
	{0x20, 113},
	{0x28, 116},
	{0x30, 119},
	{0x38, 122},
	{0x40, 125},
	{0x48, 128},
	{0x50, 131},
	{0x58, 134},
	{0x60, 138},
	{0x68, 141},
	{0x70, 144},
	{0x78, 147},
	{0x80, 150},
	{0x88, 153},
	{0x90, 156},
	{0x98, 159},
	{0xA0, 163},
	{0xA8, 166},
	{0xB0, 169},
	{0xB8, 172},
	{0xC0, 175},
	{0xC8, 178},
	{0xD0, 181},
	{0xD8, 184},
	{0xE0, 188},
	{0xE8, 191},
	{0xF0, 194},
	{0xF8, 197},
};

static int hm5040_set_gain_reg(struct hm5040 *hm5040, u32 a_gain)
{
	struct i2c_client *client = hm5040->client;
	int ret = 0, i = 0;
	u32 temp_gain = 0, reg_again = 0, reg_dgain = 0;

	//To find the approximate coarse gain
	for (i = ARRAY_SIZE(hm5040_again_table) - 1; i > 0 ; i--) {
		if (a_gain >= hm5040_again_table[i].gain_value)
			break;
	}

	reg_again = hm5040_again_table[i].reg_gain;

	temp_gain = (a_gain * 100) / reg_again;
	dev_dbg(&client->dev, "temp_d_gain 0x%x\n", temp_gain);

	//use the fine gain to fix
	for (i = ARRAY_SIZE(hm5040_dgain_table) - 1; i > 0; i--) {
		if (temp_gain >= hm5040_dgain_table[i].gain_value)
			break;
	}

	reg_dgain = hm5040_dgain_table[i].reg_gain;

	ret = hm5040_write_reg(hm5040->client, HM5040_A_GAIN_L,
			       HM5040_REG_VALUE_08BIT, reg_again);
	ret |= hm5040_write_reg(hm5040->client, HM5040_D_GAIN_GREENR_L,
				HM5040_REG_VALUE_08BIT, reg_dgain);
	ret |= hm5040_write_reg(hm5040->client, HM5040_D_GAIN_RED_L,
				HM5040_REG_VALUE_08BIT, reg_dgain);
	ret |= hm5040_write_reg(hm5040->client, HM5040_D_GAIN_BLUE_L,
				HM5040_REG_VALUE_08BIT, reg_dgain);
	ret |= hm5040_write_reg(hm5040->client, HM5040_D_GAIN_GREENB_L,
				HM5040_REG_VALUE_08BIT, reg_dgain);

	dev_dbg(&client->dev, "reg_again:0x%x, reg_dgain:0x%x\n",
		 reg_again, reg_dgain);

	return ret;
}

static int hm5040_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hm5040 *hm5040 = container_of(ctrl->handler,
					     struct hm5040, ctrl_handler);
	struct i2c_client *client = hm5040->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = hm5040->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(hm5040->exposure,
					 hm5040->exposure->minimum, max,
					 hm5040->exposure->step,
					 hm5040->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
		/*group */
		dev_dbg(&client->dev, "set exposure value 0x%x\n", ctrl->val);
		ret = hm5040_write_reg(hm5040->client, HM5040_REG_GROUP,
				       HM5040_REG_VALUE_08BIT, 0x01);
		ret |= hm5040_write_reg(hm5040->client, HM5040_REG_EXPOSURE,
					HM5040_REG_VALUE_16BIT, ctrl->val);
		ret |= hm5040_write_reg(hm5040->client, HM5040_REG_GROUP,
					HM5040_REG_VALUE_08BIT, 0x00);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		/*group */
		dev_dbg(&client->dev, "set analog gain value 0x%x\n", ctrl->val);
		ret = hm5040_write_reg(hm5040->client, HM5040_REG_GROUP,
				       HM5040_REG_VALUE_08BIT, 0x01);
		ret |= hm5040_set_gain_reg(hm5040, ctrl->val);
		ret |= hm5040_write_reg(hm5040->client, HM5040_REG_GROUP,
					HM5040_REG_VALUE_08BIT, 0x00);
		break;

	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vb value 0x%x\n", ctrl->val);
		ret = hm5040_write_reg(hm5040->client, HM5040_REG_VTS,
				       HM5040_REG_VALUE_16BIT,
				       ctrl->val + hm5040->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = hm5040_enable_test_pattern(hm5040, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops hm5040_ctrl_ops = {
	.s_ctrl = hm5040_set_ctrl,
};

static int hm5040_initialize_controls(struct hm5040 *hm5040)
{
	const struct hm5040_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &hm5040->ctrl_handler;
	mode = hm5040->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &hm5040->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, hm5040->pixel_rate, 1, hm5040->pixel_rate);

	h_blank = mode->hts_def - mode->width;
	hm5040->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (hm5040->hblank)
		hm5040->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	hm5040->vblank = v4l2_ctrl_new_std(handler, &hm5040_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				HM5040_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	hm5040->exposure = v4l2_ctrl_new_std(handler, &hm5040_ctrl_ops,
				V4L2_CID_EXPOSURE, HM5040_EXPOSURE_MIN,
				exposure_max, HM5040_EXPOSURE_STEP,
				mode->exp_def);

	hm5040->anal_gain = v4l2_ctrl_new_std(handler, &hm5040_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, ANALOG_GAIN_MIN,
				ANALOG_GAIN_MAX, ANALOG_GAIN_STEP,
				ANALOG_GAIN_DEFAULT);

	hm5040->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&hm5040_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(hm5040_test_pattern_menu) - 1,
				0, 0, hm5040_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&hm5040->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	hm5040->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int hm5040_check_sensor_id(struct hm5040 *hm5040,
				  struct i2c_client *client)
{
	struct device *dev = &hm5040->client->dev;
	u32 id = 0;
	int ret;

	ret = hm5040_read_reg(client, HM5040_REG_CHIP_ID,
			      HM5040_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected OV%06x sensor\n", CHIP_ID);

	return 0;
}

static int hm5040_configure_regulators(struct hm5040 *hm5040)
{
	unsigned int i;

	for (i = 0; i < HM5040_NUM_SUPPLIES; i++)
		hm5040->supplies[i].supply = hm5040_supply_names[i];

	return devm_regulator_bulk_get(&hm5040->client->dev,
				       HM5040_NUM_SUPPLIES,
				       hm5040->supplies);
}

static int hm5040_parse_of(struct hm5040 *hm5040)
{
	struct device *dev = &hm5040->client->dev;
	struct device_node *endpoint;
	struct fwnode_handle *fwnode;
	int rval;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}
	fwnode = of_fwnode_handle(endpoint);
	rval = fwnode_property_read_u32_array(fwnode, "data-lanes", NULL, 0);
	if (rval <= 0) {
		dev_warn(dev, " Get mipi lane num failed!\n");
		return -1;
	}

	hm5040->lane_num = rval;
	if (2 == hm5040->lane_num) {
		hm5040->cur_mode = &supported_modes_2lane[0];
		supported_modes = supported_modes_2lane;
		hm5040->cfg_num = ARRAY_SIZE(supported_modes_2lane);

		/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
		hm5040->pixel_rate = MIPI_FREQ * 2U * hm5040->lane_num * 2U / 8U;
		dev_info(dev, "lane_num(%d)  pixel_rate(%u)\n",
				 hm5040->lane_num, hm5040->pixel_rate);
	} else {
		dev_err(dev, "unsupported lane_num(%d)\n", hm5040->lane_num);
		return -1;
	}

	return 0;
}

static int hm5040_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct hm5040 *hm5040;
	struct v4l2_subdev *sd;
	char facing[2] = "b";
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	hm5040 = devm_kzalloc(dev, sizeof(*hm5040), GFP_KERNEL);
	if (!hm5040)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &hm5040->module_index);
	if (ret) {
		dev_warn(dev, "could not get module index!\n");
		hm5040->module_index = 0;
	}
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &hm5040->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &hm5040->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &hm5040->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	hm5040->client = client;

	hm5040->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(hm5040->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	hm5040->power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(hm5040->power_gpio))
		dev_warn(dev, "Failed to get power-gpios, maybe no use\n");

	hm5040->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(hm5040->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios, maybe no use\n");

	hm5040->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(hm5040->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = hm5040_configure_regulators(hm5040);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}
	ret = hm5040_parse_of(hm5040);
	if (ret != 0)
		return -EINVAL;

	hm5040->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(hm5040->pinctrl)) {
		hm5040->pins_default =
			pinctrl_lookup_state(hm5040->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(hm5040->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		hm5040->pins_sleep =
			pinctrl_lookup_state(hm5040->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(hm5040->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	mutex_init(&hm5040->mutex);

	sd = &hm5040->subdev;
	v4l2_i2c_subdev_init(sd, client, &hm5040_subdev_ops);
	ret = hm5040_initialize_controls(hm5040);
	if (ret)
		goto err_destroy_mutex;

	ret = __hm5040_power_on(hm5040);
	if (ret)
		goto err_free_handler;

	ret = hm5040_check_sensor_id(hm5040, client);
	if (ret < 0) {
		dev_info(&client->dev, "%s(%d) Check id  failed\n"
				  "check following information:\n"
				  "Power/PowerDown/Reset/Mclk/I2cBus !!\n",
				  __func__, __LINE__);
		goto err_power_off;
	}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &hm5040_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	hm5040->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &hm5040->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(hm5040->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 hm5040->module_index, facing,
		 HM5040_NAME, dev_name(sd->dev));

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
	__hm5040_power_off(hm5040);
err_free_handler:
	v4l2_ctrl_handler_free(&hm5040->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&hm5040->mutex);

	return ret;
}

static int hm5040_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hm5040 *hm5040 = to_hm5040(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&hm5040->ctrl_handler);
	mutex_destroy(&hm5040->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__hm5040_power_off(hm5040);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id hm5040_of_match[] = {
	{ .compatible = "himax,hm5040" },
	{},
};
MODULE_DEVICE_TABLE(of, hm5040_of_match);
#endif

static const struct i2c_device_id hm5040_match_id[] = {
	{ "himax,hm5040", 0 },
	{ },
};

static struct i2c_driver hm5040_i2c_driver = {
	.driver = {
		.name = HM5040_NAME,
		.pm = &hm5040_pm_ops,
		.of_match_table = of_match_ptr(hm5040_of_match),
	},
	.probe		= &hm5040_probe,
	.remove		= &hm5040_remove,
	.id_table	= hm5040_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&hm5040_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&hm5040_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("HiMax hm5040 sensor driver");
MODULE_LICENSE("GPL v2");
