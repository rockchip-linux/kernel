// SPDX-License-Identifier: GPL-2.0
/*
 * hi846 driver
 *
 * Copyright (C) 2021 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 init version
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
//#define CHECK_REG_VALUE

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x00)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
#define MIPI_FREQ_360MHZ		360000000LL
#define MIPI_FREQ_720MHZ		720000000LL
#define HI846_XVCLK_FREQ		24000000

#define CHIP_ID				0x0846
#define HI846_REG_CHIP_ID_H		0x0f17
#define HI846_REG_CHIP_ID_L		0x0f16

#define HI846_REG_CTRL_MODE		0x0A00
#define HI846_MODE_SW_STANDBY		0x00
#define HI846_MODE_STREAMING		0x01

#define HI846_REG_EXPOSURE_H		0x0073
#define HI846_REG_EXPOSURE_M		0x0074
#define HI846_REG_EXPOSURE_L		0x0075
#define HI846_FETCH_HIGH_BYTE_EXP(VAL)	(((VAL) >> 16) & 0xF)	/* 4 Bits */
#define HI846_FETCH_MIDDLE_BYTE_EXP(VAL) (((VAL) >> 8) & 0xFF)	/* 8 Bits */
#define HI846_FETCH_LOW_BYTE_EXP(VAL)	((VAL) & 0xFF)	/* 8 Bits */

#define	HI846_EXPOSURE_MIN		4
#define	HI846_EXPOSURE_STEP		1
#define HI846_VTS_MAX			0x7fff

#define HI846_REG_GAIN			0x0077
#define HI846_GAIN_MASK			0xff

#define	ANALOG_GAIN_MIN			0x00
#define	ANALOG_GAIN_MAX			0xF0
#define	ANALOG_GAIN_STEP		1
#define	ANALOG_GAIN_DEFAULT		0x10

#define HI846_REG_GROUP	0x0046

#define HI846_REG_TEST_PATTERN		0x0A05
#define	HI846_TEST_PATTERN_ENABLE	0x01
#define	HI846_TEST_PATTERN_DISABLE	0x0
#define HI846_REG_TEST_PATTERN_SELECT	0x020a

#define HI846_REG_VTS			0x0006

#define REG_NULL			0xFFFF
#define DELAY_MS			0xEEEE	/* Array delay token */

#define HI846_REG_VALUE_08BIT		1
#define HI846_REG_VALUE_16BIT		2
#define HI846_REG_VALUE_24BIT		3

#define HI846_LANES			2
#define HI846_BITS_PER_SAMPLE		10

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define HI846_NAME			"hi846"

#define HI846_MEDIA_BUS_FMT		MEDIA_BUS_FMT_SGRBG10_1X10

static const struct regval *hi846_global_regs;

struct hi846_otp_info {
	int flag; // bit[7]: info, bit[6]:wb
	int module_id;
	int lens_id;
	int year;
	int month;
	int day;
	int rg_ratio;
	int bg_ratio;
};

static const char * const hi846_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define HI846_NUM_SUPPLIES ARRAY_SIZE(hi846_supply_names)

struct regval {
	u16 addr;
	u16 val;
};

struct hi846_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
};

struct hi846 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*power_gpio;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[HI846_NUM_SUPPLIES];

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
	const struct hi846_mode *cur_mode;
	unsigned int lane_num;
	unsigned int cfg_num;
	unsigned int pixel_rate;
	u32			module_index;
	struct hi846_otp_info *otp;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	struct rkmodule_awb_cfg	awb_cfg;
};

#define to_hi846(sd) container_of(sd, struct hi846, subdev)

/*
 * Xclk 24Mhz
 * Pclk 288Mhz
 * linelength 3800(0xED8)
 * framelength 2526(0x9DE)
 * grabwindow_width 3264
 * grabwindow_height 2448
 * max_framerate 30fps
 * MIPI speed(Mbps) : 1440Mbps x 2Lane
 */
static const struct regval hi846_global_regs_2lane[] = {
	{0x2000, 0x100A},
	{0x2002, 0x00FF},
	{0x2004, 0x0007},
	{0x2006, 0x3FFF},
	{0x2008, 0x3FFF},
	{0x200A, 0xC216},
	{0x200C, 0x1292},
	{0x200E, 0xC01A},
	{0x2010, 0x403D},
	{0x2012, 0x000E},
	{0x2014, 0x403E},
	{0x2016, 0x0B80},
	{0x2018, 0x403F},
	{0x201A, 0x82AE},
	{0x201C, 0x1292},
	{0x201E, 0xC00C},
	{0x2020, 0x4130},
	{0x2022, 0x43E2},
	{0x2024, 0x0180},
	{0x2026, 0x4130},
	{0x2028, 0x7400},
	{0x202A, 0x5000},
	{0x202C, 0x0253},
	{0x202E, 0x0AD1},
	{0x2030, 0x2360},
	{0x2032, 0x0009},
	{0x2034, 0x5020},
	{0x2036, 0x000B},
	{0x2038, 0x0002},
	{0x203A, 0x0044},
	{0x203C, 0x0016},
	{0x203E, 0x1792},
	{0x2040, 0x7002},
	{0x2042, 0x154F},
	{0x2044, 0x00D5},
	{0x2046, 0x000B},
	{0x2048, 0x0019},
	{0x204A, 0x1698},
	{0x204C, 0x000E},
	{0x204E, 0x099A},
	{0x2050, 0x0058},
	{0x2052, 0x7000},
	{0x2054, 0x1799},
	{0x2056, 0x0310},
	{0x2058, 0x03C3},
	{0x205A, 0x004C},
	{0x205C, 0x064A},
	{0x205E, 0x0001},
	{0x2060, 0x0007},
	{0x2062, 0x0BC7},
	{0x2064, 0x0055},
	{0x2066, 0x7000},
	{0x2068, 0x1550},
	{0x206A, 0x158A},
	{0x206C, 0x0004},
	{0x206E, 0x1488},
	{0x2070, 0x7010},
	{0x2072, 0x1508},
	{0x2074, 0x0004},
	{0x2076, 0x0016},
	{0x2078, 0x03D5},
	{0x207A, 0x0055},
	{0x207C, 0x08CA},
	{0x207E, 0x2019},
	{0x2080, 0x0007},
	{0x2082, 0x7057},
	{0x2084, 0x0FC7},
	{0x2086, 0x5041},
	{0x2088, 0x12C8},
	{0x208A, 0x5060},
	{0x208C, 0x5080},
	{0x208E, 0x2084},
	{0x2090, 0x12C8},
	{0x2092, 0x7800},
	{0x2094, 0x0802},
	{0x2096, 0x040F},
	{0x2098, 0x1007},
	{0x209A, 0x0803},
	{0x209C, 0x080B},
	{0x209E, 0x3803},
	{0x20A0, 0x0807},
	{0x20A2, 0x0404},
	{0x20A4, 0x0400},
	{0x20A6, 0xFFFF},
	{0x20A8, 0xF0B2},
	{0x20AA, 0xFFEF},
	{0x20AC, 0x0A84},
	{0x20AE, 0x1292},
	{0x20B0, 0xC02E},
	{0x20B2, 0x4130},
	{0x23FE, 0xC056},
	{0x3232, 0xFC0C},
	{0x3236, 0xFC22},
	{0x3248, 0xFCA8},
	{0x326A, 0x8302},
	{0x326C, 0x830A},
	{0x326E, 0x0000},
	{0x32CA, 0xFC28},
	{0x32CC, 0xC3BC},
	{0x32CE, 0xC34C},
	{0x32D0, 0xC35A},
	{0x32D2, 0xC368},
	{0x32D4, 0xC376},
	{0x32D6, 0xC3C2},
	{0x32D8, 0xC3E6},
	{0x32DA, 0x0003},
	{0x32DC, 0x0003},
	{0x32DE, 0x00C7},
	{0x32E0, 0x0031},
	{0x32E2, 0x0031},
	{0x32E4, 0x0031},
	{0x32E6, 0xFC28},
	{0x32E8, 0xC3BC},
	{0x32EA, 0xC384},
	{0x32EC, 0xC392},
	{0x32EE, 0xC3A0},
	{0x32F0, 0xC3AE},
	{0x32F2, 0xC3C4},
	{0x32F4, 0xC3E6},
	{0x32F6, 0x0003},
	{0x32F8, 0x0003},
	{0x32FA, 0x00C7},
	{0x32FC, 0x0031},
	{0x32FE, 0x0031},
	{0x3300, 0x0031},
	{0x3302, 0x82CA},
	{0x3304, 0xC164},
	{0x3306, 0x82E6},
	{0x3308, 0xC19C},
	{0x330A, 0x001F},
	{0x330C, 0x001A},
	{0x330E, 0x0034},
	{0x3310, 0x0000},
	{0x3312, 0x0000},
	{0x3314, 0xFC94},
	{0x3316, 0xC3D8},

	{0x0A00, 0x0000},
	{0x0E04, 0x0012},
	{0x002E, 0x1111},
	{0x0032, 0x1111},
	{0x0022, 0x0008},
	{0x0026, 0x0040},
	{0x0028, 0x0017},
	{0x002C, 0x09CF},
	{0x005C, 0x2101},
	{0x0006, 0x09BC},
	{0x0008, 0x0ED8},
	{0x000E, 0x0200},
	{0x000C, 0x0022},
	{0x0A22, 0x0000},
	{0x0A24, 0x0000},
	{0x0804, 0x0000},
	{0x0A12, 0x0CC0},
	{0x0A14, 0x0990},
	{0x0074, 0x09B6},
	{0x0076, 0x0000},
	{0x051E, 0x0000},
	{0x0200, 0x0400},
	{0x0A1A, 0x0C00},
	{0x0A0C, 0x0010},
	{0x0A1E, 0x0CCF},
	{0x0402, 0x0110},
	{0x0404, 0x00F4},
	{0x0408, 0x0000},
	{0x0410, 0x008D},
	{0x0412, 0x011A},
	{0x0414, 0x864C},
	{0x021C, 0x0001},
	{0x0C00, 0x9150},
	{0x0C06, 0x0021},
	{0x0C10, 0x0040},
	{0x0C12, 0x0040},
	{0x0C14, 0x0040},
	{0x0C16, 0x0040},
	{0x0A02, 0x0100},
	{0x0A04, 0x014A},
	{0x0418, 0x0000},
	{0x012A, 0x03B4},
	{0x0120, 0x0046},
	{0x0122, 0x0376},
	{0x0B02, 0xE04D},
	{0x0B10, 0x6821},
	{0x0B12, 0x0120},
	{0x0B14, 0x0001},
	{0x2008, 0x38FD},
	{0x326E, 0x0000},
	{0x0900, 0x0320},
	{0x0902, 0xC31A},
	{0x0914, 0xC109},
	{0x0916, 0x061A},
	{0x0918, 0x0306},
	{0x091A, 0x0B09},
	{0x091C, 0x0C07},
	{0x091E, 0x0A00},
	{0x090C, 0x042A},
	{0x090E, 0x006B},
	{0x0954, 0x0089},
	{0x0956, 0x0000},
	{0x0958, 0xCA00},
	{0x095A, 0x9240},
	{0x0F08, 0x2F04},
	{0x0F30, 0x001F},
	{0x0F36, 0x001F},
	{0x0F04, 0x3A00},
	{0x0F32, 0x025A},
	{0x0F38, 0x025A},
	{0x0F2A, 0x0024},
	{0x006A, 0x0100},
	{0x004C, 0x0100},
	//{0x0A00, 0x0100},
	{0x003c, 0x0101}, //fix framerate

	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * Pclk 288Mhz
 * linelength 3800(0xED8)
 * framelength 2526(0x9DE)
 * grabwindow_width 3264
 * grabwindow_height 2448
 * max_framerate 30fps
 * MIPI speed(Mbps) : 1440Mbps x 2Lane
 */
static const struct regval hi846_3264x2448_regs_2lane[] = {
	{0x0A00, 0x0000},
	{0x002E, 0x1111},
	{0x0032, 0x1111},
	{0x0026, 0x0040},
	{0x002C, 0x09CF},
	{0x005C, 0x2101},
	{0x0006, 0x09BC},
	{0x0008, 0x0ED8},
	{0x000C, 0x0022},
	{0x0A22, 0x0000},
	{0x0A24, 0x0000},
	{0x0804, 0x0000},
	{0x0A12, 0x0CC0},
	{0x0A14, 0x0990},
	{0x0074, 0x09B6},
	{0x0A04, 0x014A},
	{0x0418, 0x0000},
	{0x0B02, 0xE04D},
	{0x0B10, 0x6821},
	{0x0B12, 0x0120},
	{0x0B14, 0x0001},
	{0x2008, 0x38FD},
	{0x326E, 0x0000},
	//=============================================//
	//		MIPI 2lane 1440Mbps
	//=============================================//
	{0x0900, 0x0320},
	{0x0902, 0xC31A},
	{0x0914, 0xC109},
	{0x0916, 0x061A},
	{0x0918, 0x0306},
	{0x091A, 0x0B09},
	{0x091C, 0x0C07},
	{0x091E, 0x0A00},
	{0x090C, 0x042A},
	{0x090E, 0x006B},
	{0x0954, 0x0089},
	{0x0956, 0x0000},
	{0x0958, 0xCA00},
	{0x095A, 0x9240},
	{0x0F2A, 0x0024},
	{0x004C, 0x0100},
	//{0x0A00, 0x0100},

	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * Pclk 288Mhz
 * linelength 3800(0xED8)
 * framelength 2526(0x9DE)
 * grabwindow_width 3264
 * grabwindow_height 2448
 * max_framerate 30fps
 * MIPI speed(Mbps) : 720Mbps x 4Lane
 */
static const struct regval hi846_global_regs_4lane[] = {
	{0x2000, 0x987a},
	{0x2002, 0x00ff},
	{0x2004, 0x0047},
	{0x2006, 0x3fff},
	{0x2008, 0x3fff},
	{0x200a, 0xc216},
	{0x200c, 0x1292},
	{0x200e, 0xc01a},
	{0x2010, 0x403d},
	{0x2012, 0x000e},
	{0x2014, 0x403e},
	{0x2016, 0x0b80},
	{0x2018, 0x403f},
	{0x201a, 0x82ae},
	{0x201c, 0x1292},
	{0x201e, 0xc00c},
	{0x2020, 0x4130},
	{0x2022, 0x43e2},
	{0x2024, 0x0180},
	{0x2026, 0x4130},
	{0x2028, 0x7400},
	{0x202a, 0x5000},
	{0x202c, 0x0253},
	{0x202e, 0x0ad1},
	{0x2030, 0x2360},
	{0x2032, 0x0009},
	{0x2034, 0x5020},
	{0x2036, 0x000b},
	{0x2038, 0x0002},
	{0x203a, 0x0044},
	{0x203c, 0x0016},
	{0x203e, 0x1792},
	{0x2040, 0x7002},
	{0x2042, 0x154f},
	{0x2044, 0x00d5},
	{0x2046, 0x000b},
	{0x2048, 0x0019},
	{0x204a, 0x1698},
	{0x204c, 0x000e},
	{0x204e, 0x099a},
	{0x2050, 0x0058},
	{0x2052, 0x7000},
	{0x2054, 0x1799},
	{0x2056, 0x0310},
	{0x2058, 0x03c3},
	{0x205a, 0x004c},
	{0x205c, 0x064a},
	{0x205e, 0x0001},
	{0x2060, 0x0007},
	{0x2062, 0x0bc7},
	{0x2064, 0x0055},
	{0x2066, 0x7000},
	{0x2068, 0x1550},
	{0x206a, 0x158a},
	{0x206c, 0x0004},
	{0x206e, 0x1488},
	{0x2070, 0x7010},
	{0x2072, 0x1508},
	{0x2074, 0x0004},
	{0x2076, 0x0016},
	{0x2078, 0x03d5},
	{0x207a, 0x0055},
	{0x207c, 0x08ca},
	{0x207e, 0x2019},
	{0x2080, 0x0007},
	{0x2082, 0x7057},
	{0x2084, 0x0fc7},
	{0x2086, 0x5041},
	{0x2088, 0x12c8},
	{0x208a, 0x5060},
	{0x208c, 0x5080},
	{0x208e, 0x2084},
	{0x2090, 0x12c8},
	{0x2092, 0x7800},
	{0x2094, 0x0802},
	{0x2096, 0x040f},
	{0x2098, 0x1007},
	{0x209a, 0x0803},
	{0x209c, 0x080b},
	{0x209e, 0x3803},
	{0x20a0, 0x0807},
	{0x20a2, 0x0404},
	{0x20a4, 0x0400},
	{0x20a6, 0xffff},
	{0x20a8, 0xf0b2},
	{0x20aa, 0xffef},
	{0x20ac, 0x0a84},
	{0x20ae, 0x1292},
	{0x20b0, 0xc02e},
	{0x20b2, 0x4130},
	{0x20b4, 0xf0b2},
	{0x20b6, 0xffbf},
	{0x20b8, 0x2004},
	{0x20ba, 0x403f},
	{0x20bc, 0x00c3},
	{0x20be, 0x4fe2},
	{0x20c0, 0x8318},
	{0x20c2, 0x43cf},
	{0x20c4, 0x0000},
	{0x20c6, 0x9382},
	{0x20c8, 0xc314},
	{0x20ca, 0x2003},
	{0x20cc, 0x12b0},
	{0x20ce, 0xcab0},
	{0x20d0, 0x4130},
	{0x20d2, 0x12b0},
	{0x20d4, 0xc90a},
	{0x20d6, 0x4130},
	{0x20d8, 0x42d2},
	{0x20da, 0x8318},
	{0x20dc, 0x00c3},
	{0x20de, 0x9382},
	{0x20e0, 0xc314},
	{0x20e2, 0x2009},
	{0x20e4, 0x120b},
	{0x20e6, 0x120a},
	{0x20e8, 0x1209},
	{0x20ea, 0x1208},
	{0x20ec, 0x1207},
	{0x20ee, 0x1206},
	{0x20f0, 0x4030},
	{0x20f2, 0xc15e},
	{0x20f4, 0x4130},
	{0x20f6, 0x1292},
	{0x20f8, 0xc008},
	{0x20fa, 0x4130},
	{0x20fc, 0x42d2},
	{0x20fe, 0x82a1},
	{0x2100, 0x00c2},
	{0x2102, 0x1292},
	{0x2104, 0xc040},
	{0x2106, 0x4130},
	{0x2108, 0x1292},
	{0x210a, 0xc006},
	{0x210c, 0x42a2},
	{0x210e, 0x7324},
	{0x2110, 0x9382},
	{0x2112, 0xc314},
	{0x2114, 0x2011},
	{0x2116, 0x425f},
	{0x2118, 0x82a1},
	{0x211a, 0xf25f},
	{0x211c, 0x00c1},
	{0x211e, 0xf35f},
	{0x2120, 0x2406},
	{0x2122, 0x425f},
	{0x2124, 0x00c0},
	{0x2126, 0xf37f},
	{0x2128, 0x522f},
	{0x212a, 0x4f82},
	{0x212c, 0x7324},
	{0x212e, 0x425f},
	{0x2130, 0x82d4},
	{0x2132, 0xf35f},
	{0x2134, 0x4fc2},
	{0x2136, 0x01b3},
	{0x2138, 0x93c2},
	{0x213a, 0x829f},
	{0x213c, 0x2421},
	{0x213e, 0x403e},
	{0x2140, 0xfffe},
	{0x2142, 0x40b2},
	{0x2144, 0xec78},
	{0x2146, 0x831c},
	{0x2148, 0x40b2},
	{0x214a, 0xec78},
	{0x214c, 0x831e},
	{0x214e, 0x40b2},
	{0x2150, 0xec78},
	{0x2152, 0x8320},
	{0x2154, 0xb3d2},
	{0x2156, 0x008c},
	{0x2158, 0x2405},
	{0x215a, 0x4e0f},
	{0x215c, 0x503f},
	{0x215e, 0xffd8},
	{0x2160, 0x4f82},
	{0x2162, 0x831c},
	{0x2164, 0x90f2},
	{0x2166, 0x0003},
	{0x2168, 0x008c},
	{0x216a, 0x2401},
	{0x216c, 0x4130},
	{0x216e, 0x421f},
	{0x2170, 0x831c},
	{0x2172, 0x5e0f},
	{0x2174, 0x4f82},
	{0x2176, 0x831e},
	{0x2178, 0x5e0f},
	{0x217a, 0x4f82},
	{0x217c, 0x8320},
	{0x217e, 0x3ff6},
	{0x2180, 0x432e},
	{0x2182, 0x3fdf},
	{0x2184, 0x421f},
	{0x2186, 0x7100},
	{0x2188, 0x4f0e},
	{0x218a, 0x503e},
	{0x218c, 0xffd8},
	{0x218e, 0x4e82},
	{0x2190, 0x7a04},
	{0x2192, 0x421e},
	{0x2194, 0x831c},
	{0x2196, 0x5f0e},
	{0x2198, 0x4e82},
	{0x219a, 0x7a06},
	{0x219c, 0x0b00},
	{0x219e, 0x7304},
	{0x21a0, 0x0050},
	{0x21a2, 0x40b2},
	{0x21a4, 0xd081},
	{0x21a6, 0x0b88},
	{0x21a8, 0x421e},
	{0x21aa, 0x831e},
	{0x21ac, 0x5f0e},
	{0x21ae, 0x4e82},
	{0x21b0, 0x7a0e},
	{0x21b2, 0x521f},
	{0x21b4, 0x8320},
	{0x21b6, 0x4f82},
	{0x21b8, 0x7a10},
	{0x21ba, 0x0b00},
	{0x21bc, 0x7304},
	{0x21be, 0x007a},
	{0x21c0, 0x40b2},
	{0x21c2, 0x0081},
	{0x21c4, 0x0b88},
	{0x21c6, 0x4392},
	{0x21c8, 0x7a0a},
	{0x21ca, 0x0800},
	{0x21cc, 0x7a0c},
	{0x21ce, 0x0b00},
	{0x21d0, 0x7304},
	{0x21d2, 0x022b},
	{0x21d4, 0x40b2},
	{0x21d6, 0xd081},
	{0x21d8, 0x0b88},
	{0x21da, 0x0b00},
	{0x21dc, 0x7304},
	{0x21de, 0x0255},
	{0x21e0, 0x40b2},
	{0x21e2, 0x0081},
	{0x21e4, 0x0b88},
	{0x21e6, 0x4130},
	{0x23fe, 0xc056},
	{0x3232, 0xfc0c},
	{0x3236, 0xfc22},
	{0x3238, 0xfcfc},
	{0x323a, 0xfd84},
	{0x323c, 0xfd08},
	{0x3246, 0xfcd8},
	{0x3248, 0xfca8},
	{0x324e, 0xfcb4},
	{0x326a, 0x8302},
	{0x326c, 0x830a},
	{0x326e, 0x0000},
	{0x32ca, 0xfc28},
	{0x32cc, 0xc3bc},
	{0x32ce, 0xc34c},
	{0x32d0, 0xc35a},
	{0x32d2, 0xc368},
	{0x32d4, 0xc376},
	{0x32d6, 0xc3c2},
	{0x32d8, 0xc3e6},
	{0x32da, 0x0003},
	{0x32dc, 0x0003},
	{0x32de, 0x00c7},
	{0x32e0, 0x0031},
	{0x32e2, 0x0031},
	{0x32e4, 0x0031},
	{0x32e6, 0xfc28},
	{0x32e8, 0xc3bc},
	{0x32ea, 0xc384},
	{0x32ec, 0xc392},
	{0x32ee, 0xc3a0},
	{0x32f0, 0xc3ae},
	{0x32f2, 0xc3c4},
	{0x32f4, 0xc3e6},
	{0x32f6, 0x0003},
	{0x32f8, 0x0003},
	{0x32fa, 0x00c7},
	{0x32fc, 0x0031},
	{0x32fe, 0x0031},
	{0x3300, 0x0031},
	{0x3302, 0x82ca},
	{0x3304, 0xc164},
	{0x3306, 0x82e6},
	{0x3308, 0xc19c},
	{0x330a, 0x001f},
	{0x330c, 0x001a},
	{0x330e, 0x0034},
	{0x3310, 0x0000},
	{0x3312, 0x0000},
	{0x3314, 0xfc94},
	{0x3316, 0xc3d8},

	{0x0a00, 0x0000},
	{0x0e04, 0x0012},
	{0x002e, 0x1111},
	{0x0032, 0x1111},
	{0x0022, 0x0008},
	{0x0026, 0x0040},
	{0x0028, 0x0017},
	{0x002c, 0x09cf},
	{0x005c, 0x2101},
	{0x0006, 0x09de},
	{0x0008, 0x0ed8},
	{0x000e, 0x0200},
	{0x000c, 0x0022},
	{0x0a22, 0x0000},
	{0x0a24, 0x0000},
	{0x0804, 0x0000},
	{0x0a12, 0x0cc0},
	{0x0a14, 0x0990},
	{0x0074, 0x09d8},
	{0x0076, 0x0000},
	{0x051e, 0x0000},
	{0x0200, 0x0400},
	{0x0a1a, 0x0c00},
	{0x0a0c, 0x0010},
	{0x0a1e, 0x0ccf},
	{0x0402, 0x0110},
	{0x0404, 0x00f4},
	{0x0408, 0x0000},
	{0x0410, 0x008d},
	{0x0412, 0x011a},
	{0x0414, 0x864c},
	{0x021c, 0x0001},
	{0x0c00, 0x9950},
	{0x0c06, 0x0021},
	{0x0c10, 0x0040},
	{0x0c12, 0x0040},
	{0x0c14, 0x0040},
	{0x0c16, 0x0040},
	{0x0a02, 0x0100},
	{0x0a04, 0x014a},
	{0x0418, 0x0000},
	{0x012a, 0xffff},
	{0x0120, 0x0046},
	{0x0122, 0x0376},
	{0x0746, 0x0050},
	{0x0748, 0x01d5},
	{0x074a, 0x022b},
	{0x074c, 0x03b0},
	{0x0756, 0x043f},
	{0x0758, 0x3f1d},
	{0x0b02, 0xe04d},
	{0x0b10, 0x6821},
	{0x0b12, 0x0120},
	{0x0b14, 0x0001},
	{0x2008, 0x38fd},
	{0x326e, 0x0000},
	{0x0900, 0x0300},
	{0x0902, 0xc319},
	{0x0914, 0xc109},
	{0x0916, 0x061a},
	{0x0918, 0x0407},
	{0x091a, 0x0a0b},
	{0x091c, 0x0e08},
	{0x091e, 0x0a00},
	{0x090c, 0x0427},
	{0x090e, 0x0069},
	{0x0954, 0x0089},
	{0x0956, 0x0000},
	{0x0958, 0xca80},
	{0x095a, 0x9240},
	{0x0f08, 0x2f04},
	{0x0f30, 0x001f},
	{0x0f36, 0x001f},
	{0x0f04, 0x3a00},
	{0x0f32, 0x025a},
	{0x0f38, 0x025a},
	{0x0f2a, 0x4124},
	{0x006a, 0x0100},
	{0x004c, 0x0100},
	{0x003c, 0x0101}, //fix framerate
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * Pclk 288Mhz
 * linelength 3800(0xED8)
 * framelength 2526(0x9DE)
 * grabwindow_width 3264
 * grabwindow_height 2448
 * max_framerate 30fps
 * MIPI speed(Mbps) : 720Mbps x 4Lane
 */
static const struct regval hi846_3264x2448_regs_4lane[] = {
	{0x0a00, 0x0000},
	{0x002e, 0x1111},
	{0x0032, 0x1111},
	{0x0026, 0x0040},
	{0x002c, 0x09cf},
	{0x005c, 0x2101},
	{0x0006, 0x09de},
	{0x0008, 0x0ed8},
	{0x000c, 0x0022},
	{0x0a22, 0x0000},
	{0x0a24, 0x0000},
	{0x0804, 0x0000},
	{0x0a12, 0x0cc0},
	{0x0a14, 0x0990},
	{0x0074, 0x09d8},
	{0x0a04, 0x014a},
	{0x0418, 0x0000},
	{0x0b02, 0xe04d},
	{0x0b10, 0x6821},
	{0x0b12, 0x0120},
	{0x0b14, 0x0001},
	{0x2008, 0x38fd},
	{0x326e, 0x0000},
	//=============================================//
	//      MIPI 4lane 720Mbps
	//=============================================//
	{0x0900, 0x0300},
	{0x0902, 0xc319},
	{0x0914, 0xc109},
	{0x0916, 0x061a},
	{0x0918, 0x0407},
	{0x091a, 0x0a0b},
	{0x091c, 0x0e08},
	{0x091e, 0x0a00},
	{0x090c, 0x0427},
	{0x090e, 0x0069},
	{0x0954, 0x0089},
	{0x0956, 0x0000},
	{0x0958, 0xca80},
	{0x095a, 0x9240},
	{0x0f2a, 0x4124},
	{0x004c, 0x0100},
	//{0x0a00, 0x0100},

	{REG_NULL, 0x00},
};

static const struct hi846_mode supported_modes_2lane[] = {
	{
		.width = 3264,
		.height = 2448,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x09B0,
		.hts_def = 0x0ED8,
		.vts_def = 0x09DE,
		.reg_list = hi846_3264x2448_regs_2lane,
		.hdr_mode = NO_HDR,
	},
};

static const struct hi846_mode supported_modes_4lane[] = {
	{
		.width = 3264,
		.height = 2448,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x09B0,
		.hts_def = 0x0ED8,
		.vts_def = 0x09DE,
		.reg_list = hi846_3264x2448_regs_4lane,
		.hdr_mode = NO_HDR,
	}
};

static const struct hi846_mode *supported_modes;

static const s64 link_freq_menu_items[] = {
	MIPI_FREQ_360MHZ,
	MIPI_FREQ_720MHZ
};

static const char * const hi846_test_pattern_menu[] = {
	"Disabled",
	"Solid color bar",
	"100% color bars",
	"Fade to gray color bars",
	"PN9",
	"Horizental/Vertical gradient",
	"Check board",
	"Slant",
	"Resolution",
};

/* Write registers up to 4 at a time */
static int hi846_write_reg(struct i2c_client *client, u16 reg,
			    u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	dev_dbg(&client->dev, "%s(%d) enter!\n", __func__, __LINE__);
	dev_dbg(&client->dev, "write reg(0x%x val:0x%x)!\n", reg, val);

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

static int hi846_write_array(struct i2c_client *client,
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
		ret = hi846_write_reg(client, regs[i].addr,
				       HI846_REG_VALUE_16BIT, regs[i].val);
		if (ret)
			dev_err(&client->dev, "%s failed !\n", __func__);
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int hi846_read_reg(struct i2c_client *client, u16 reg,
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
static int hi846_reg_verify(struct i2c_client *client,
				const struct regval *regs)
{
	u32 i;
	int ret = 0;
	u32 value;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		ret = hi846_read_reg(client, regs[i].addr,
			  HI846_REG_VALUE_16BIT, &value);
		if (value != regs[i].val) {
			dev_info(&client->dev, "%s: 0x%04x is 0x%x instead of 0x%x\n",
				  __func__, regs[i].addr, value, regs[i].val);
		}
	}
	return ret;
}
#endif

static int hi846_get_reso_dist(const struct hi846_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct hi846_mode *
hi846_find_best_fit(struct hi846 *hi846,
			struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < hi846->cfg_num; i++) {
		dist = hi846_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int hi846_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct hi846 *hi846 = to_hi846(sd);
	const struct hi846_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&hi846->mutex);

	mode = hi846_find_best_fit(hi846, fmt);
	fmt->format.code = HI846_MEDIA_BUS_FMT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&hi846->mutex);
		return -ENOTTY;
#endif
	} else {
		hi846->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(hi846->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(hi846->vblank, vblank_def,
					 HI846_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&hi846->mutex);

	return 0;
}

static int hi846_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct hi846 *hi846 = to_hi846(sd);
	const struct hi846_mode *mode = hi846->cur_mode;

	mutex_lock(&hi846->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&hi846->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = HI846_MEDIA_BUS_FMT;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&hi846->mutex);

	return 0;
}

static int hi846_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = HI846_MEDIA_BUS_FMT;

	return 0;
}

static int hi846_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct hi846 *hi846 = to_hi846(sd);

	if (fse->index >= hi846->cfg_num)
		return -EINVAL;

	if (fse->code != HI846_MEDIA_BUS_FMT)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int hi846_enable_test_pattern(struct hi846 *hi846, u32 pattern)
{

	if (pattern) {
		hi846_write_reg(hi846->client, HI846_REG_TEST_PATTERN,
						HI846_REG_VALUE_08BIT, HI846_TEST_PATTERN_ENABLE);
		hi846_write_reg(hi846->client, HI846_REG_TEST_PATTERN_SELECT,
						HI846_REG_VALUE_08BIT, pattern - 1);
	} else {
		hi846_write_reg(hi846->client, HI846_REG_TEST_PATTERN,
						HI846_REG_VALUE_08BIT, HI846_TEST_PATTERN_DISABLE);
	}
	return 0;
}

static int hi846_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct hi846 *hi846 = to_hi846(sd);
	const struct hi846_mode *mode = hi846->cur_mode;

	mutex_lock(&hi846->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&hi846->mutex);

	return 0;
}

static void hi846_get_module_inf(struct hi846 *hi846,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, HI846_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, hi846->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, hi846->len_name, sizeof(inf->base.lens));

}

static void hi846_set_awb_cfg(struct hi846 *hi846,
				 struct rkmodule_awb_cfg *cfg)
{
	mutex_lock(&hi846->mutex);
	memcpy(&hi846->awb_cfg, cfg, sizeof(*cfg));
	mutex_unlock(&hi846->mutex);
}

static long hi846_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct hi846 *hi846 = to_hi846(sd);
	long ret = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		hi846_get_module_inf(hi846, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_AWB_CFG:
		hi846_set_awb_cfg(hi846, (struct rkmodule_awb_cfg *)arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = hi846_write_reg(hi846->client, HI846_REG_CTRL_MODE,
				HI846_REG_VALUE_08BIT, HI846_MODE_STREAMING);
		else
			ret = hi846_write_reg(hi846->client, HI846_REG_CTRL_MODE,
				HI846_REG_VALUE_08BIT, HI846_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long hi846_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = hi846_ioctl(sd, cmd, inf);
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

		if (copy_from_user(awb_cfg, up, sizeof(*awb_cfg))) {
			kfree(awb_cfg);
			return -EFAULT;
		}
		ret = hi846_ioctl(sd, cmd, awb_cfg);
		kfree(awb_cfg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;
		ret = hi846_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __hi846_start_stream(struct hi846 *hi846)
{
	int ret;

	ret = hi846_write_array(hi846->client, hi846->cur_mode->reg_list);
	if (ret)
		return ret;

#ifdef CHECK_REG_VALUE
	usleep_range(10000, 20000);
	/*  verify default values to make sure everything has */
	/*  been written correctly as expected */
	dev_info(&hi846->client->dev, "%s:Check register value!\n",
				__func__);
	ret = hi846_reg_verify(hi846->client, hi846_global_regs);
	if (ret)
		return ret;

	ret = hi846_reg_verify(hi846->client, hi846->cur_mode->reg_list);
	if (ret)
		return ret;
#endif

	/* In case these controls are set before streaming */
	mutex_unlock(&hi846->mutex);
	ret = v4l2_ctrl_handler_setup(&hi846->ctrl_handler);
	mutex_lock(&hi846->mutex);
	if (ret)
		return ret;

	if (ret)
		dev_info(&hi846->client->dev, "APPly otp failed!\n");

	ret = hi846_write_reg(hi846->client, HI846_REG_CTRL_MODE,
				HI846_REG_VALUE_08BIT, HI846_MODE_STREAMING);
	return ret;
}

static int __hi846_stop_stream(struct hi846 *hi846)
{
	return hi846_write_reg(hi846->client, HI846_REG_CTRL_MODE,
				HI846_REG_VALUE_08BIT, HI846_MODE_SW_STANDBY);
}

static int hi846_s_stream(struct v4l2_subdev *sd, int on)
{
	struct hi846 *hi846 = to_hi846(sd);
	struct i2c_client *client = hi846->client;
	int ret = 0;

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
				hi846->cur_mode->width,
				hi846->cur_mode->height,
		DIV_ROUND_CLOSEST(hi846->cur_mode->max_fps.denominator,
		hi846->cur_mode->max_fps.numerator));

	mutex_lock(&hi846->mutex);
	on = !!on;
	if (on == hi846->streaming)
		goto unlock_and_return;

	if (on) {
		dev_info(&client->dev, "stream on!!!\n");
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __hi846_start_stream(hi846);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		dev_info(&client->dev, "stream off!!!\n");
		__hi846_stop_stream(hi846);
		pm_runtime_put(&client->dev);
	}

	hi846->streaming = on;

unlock_and_return:
	mutex_unlock(&hi846->mutex);

	return ret;
}

static int hi846_s_power(struct v4l2_subdev *sd, int on)
{
	struct hi846 *hi846 = to_hi846(sd);
	struct i2c_client *client = hi846->client;
	int ret = 0;

	dev_info(&client->dev, "%s(%d) on(%d)\n", __func__, __LINE__, on);
	mutex_lock(&hi846->mutex);

	/* If the power state is not modified - no work to do. */
	if (hi846->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = hi846_write_array(hi846->client, hi846_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		hi846->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		hi846->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&hi846->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 hi846_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, HI846_XVCLK_FREQ / 1000 / 1000);
}

static int __hi846_power_on(struct hi846 *hi846)
{
	int ret;
	u32 delay_us;
	struct device *dev = &hi846->client->dev;

	if (!IS_ERR(hi846->power_gpio))
		gpiod_set_value_cansleep(hi846->power_gpio, 1);

	usleep_range(1000, 2000);

	if (!IS_ERR_OR_NULL(hi846->pins_default)) {
		ret = pinctrl_select_state(hi846->pinctrl,
					   hi846->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(hi846->xvclk, HI846_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(hi846->xvclk) != HI846_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(hi846->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	ret = regulator_bulk_enable(HI846_NUM_SUPPLIES, hi846->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(hi846->reset_gpio))
		gpiod_set_value_cansleep(hi846->reset_gpio, 1);

	if (!IS_ERR(hi846->pwdn_gpio))
		gpiod_set_value_cansleep(hi846->pwdn_gpio, 1);

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = hi846_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	usleep_range(10000, 20000);
	return 0;

disable_clk:
	clk_disable_unprepare(hi846->xvclk);

	return ret;
}

static void __hi846_power_off(struct hi846 *hi846)
{
	int ret;
	struct device *dev = &hi846->client->dev;

	if (!IS_ERR(hi846->pwdn_gpio))
		gpiod_set_value_cansleep(hi846->pwdn_gpio, 0);
	clk_disable_unprepare(hi846->xvclk);
	if (!IS_ERR(hi846->reset_gpio))
		gpiod_set_value_cansleep(hi846->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(hi846->pins_sleep)) {
		ret = pinctrl_select_state(hi846->pinctrl,
					   hi846->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	if (!IS_ERR(hi846->power_gpio))
		gpiod_set_value_cansleep(hi846->power_gpio, 0);

	regulator_bulk_disable(HI846_NUM_SUPPLIES, hi846->supplies);
}

static int hi846_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hi846 *hi846 = to_hi846(sd);

	return __hi846_power_on(hi846);
}

static int hi846_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hi846 *hi846 = to_hi846(sd);

	__hi846_power_off(hi846);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int hi846_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct hi846 *hi846 = to_hi846(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct hi846_mode *def_mode = &supported_modes[0];

	mutex_lock(&hi846->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = HI846_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&hi846->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int hi846_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct hi846 *hi846 = to_hi846(sd);

	if (fie->index >= hi846->cfg_num)
		return -EINVAL;

	if (fie->code != HI846_MEDIA_BUS_FMT)
		return -EINVAL;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static int hi846_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	struct hi846 *sensor = to_hi846(sd);
	struct device *dev = &sensor->client->dev;

	dev_info(dev, "%s(%d) enter!\n", __func__, __LINE__);

	if (2 == sensor->lane_num) {
		config->type = V4L2_MBUS_CSI2;
		config->flags = V4L2_MBUS_CSI2_2_LANE |
				V4L2_MBUS_CSI2_CHANNEL_0 |
				V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	} else if (4 == sensor->lane_num) {
		config->type = V4L2_MBUS_CSI2;
		config->flags = V4L2_MBUS_CSI2_4_LANE |
				V4L2_MBUS_CSI2_CHANNEL_0 |
				V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	} else {
		dev_err(&sensor->client->dev,
			"unsupported lane_num(%d)\n", sensor->lane_num);
	}
	return 0;
}

static const struct dev_pm_ops hi846_pm_ops = {
	SET_RUNTIME_PM_OPS(hi846_runtime_suspend,
			   hi846_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops hi846_internal_ops = {
	.open = hi846_open,
};
#endif

static const struct v4l2_subdev_core_ops hi846_core_ops = {
	.s_power = hi846_s_power,
	.ioctl = hi846_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = hi846_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops hi846_video_ops = {
	.s_stream = hi846_s_stream,
	.g_frame_interval = hi846_g_frame_interval,
	.g_mbus_config = hi846_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops hi846_pad_ops = {
	.enum_mbus_code = hi846_enum_mbus_code,
	.enum_frame_size = hi846_enum_frame_sizes,
	.enum_frame_interval = hi846_enum_frame_interval,
	.get_fmt = hi846_get_fmt,
	.set_fmt = hi846_set_fmt,
};

static const struct v4l2_subdev_ops hi846_subdev_ops = {
	.core	= &hi846_core_ops,
	.video	= &hi846_video_ops,
	.pad	= &hi846_pad_ops,
};

static int hi846_set_exposure_reg(struct hi846 *hi846, u32 exposure)
{
	int ret = 0;
	u32 cal_shutter = 0;

	cal_shutter = exposure >> 1;
	cal_shutter = cal_shutter << 1;

	ret = hi846_write_reg(hi846->client, HI846_REG_GROUP,
			      HI846_REG_VALUE_08BIT, 0x01);
	ret |= hi846_write_reg(hi846->client,
			       HI846_REG_EXPOSURE_H,
			       HI846_REG_VALUE_08BIT,
			       HI846_FETCH_HIGH_BYTE_EXP(cal_shutter));
	ret |= hi846_write_reg(hi846->client,
			       HI846_REG_EXPOSURE_M,
			       HI846_REG_VALUE_08BIT,
			       HI846_FETCH_MIDDLE_BYTE_EXP(cal_shutter));
	ret |= hi846_write_reg(hi846->client,
			       HI846_REG_EXPOSURE_L,
			       HI846_REG_VALUE_08BIT,
			       HI846_FETCH_LOW_BYTE_EXP(cal_shutter));
	ret |= hi846_write_reg(hi846->client, HI846_REG_GROUP,
			       HI846_REG_VALUE_08BIT, 0x00);

	return ret;
}

static int hi846_set_gain_reg(struct hi846 *hi846, u32 a_gain)
{
	int ret = 0;

	ret = hi846_write_reg(hi846->client, HI846_REG_GROUP,
			      HI846_REG_VALUE_08BIT, 0x01);
	ret |= hi846_write_reg(hi846->client, HI846_REG_GAIN,
			       HI846_REG_VALUE_08BIT, a_gain);
	ret |= hi846_write_reg(hi846->client, HI846_REG_GROUP,
			       HI846_REG_VALUE_08BIT, 0x00);

	return ret;
}

static int hi846_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hi846 *hi846 = container_of(ctrl->handler,
					     struct hi846, ctrl_handler);
	struct i2c_client *client = hi846->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = hi846->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(hi846->exposure,
					 hi846->exposure->minimum, max,
					 hi846->exposure->step,
					 hi846->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set exposure value 0x%x\n", ctrl->val);
		/* 4 least significant bits of expsoure are fractional part */
		ret = hi846_set_exposure_reg(hi846, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		dev_dbg(&client->dev, "set analog gain value 0x%x\n", ctrl->val);
		ret = hi846_set_gain_reg(hi846, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vb value 0x%x\n", ctrl->val);
		ret = hi846_write_reg(hi846->client, HI846_REG_VTS,
				       HI846_REG_VALUE_16BIT,
				       ctrl->val + hi846->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = hi846_enable_test_pattern(hi846, ctrl->val);
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops hi846_ctrl_ops = {
	.s_ctrl = hi846_set_ctrl,
};

static int hi846_initialize_controls(struct hi846 *hi846)
{
	const struct hi846_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &hi846->ctrl_handler;
	mode = hi846->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &hi846->mutex;

	if (2 == hi846->lane_num) {
		ctrl = v4l2_ctrl_new_int_menu(handler, NULL,
					      V4L2_CID_LINK_FREQ, 1, 1,
					      link_freq_menu_items);
	} else {
		ctrl = v4l2_ctrl_new_int_menu(handler, NULL,
					      V4L2_CID_LINK_FREQ, 1, 0,
					      link_freq_menu_items);
	}
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, hi846->pixel_rate, 1, hi846->pixel_rate);

	h_blank = mode->hts_def - mode->width;
	hi846->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (hi846->hblank)
		hi846->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	hi846->vblank = v4l2_ctrl_new_std(handler, &hi846_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				HI846_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	hi846->exposure = v4l2_ctrl_new_std(handler, &hi846_ctrl_ops,
				V4L2_CID_EXPOSURE, HI846_EXPOSURE_MIN,
				exposure_max, HI846_EXPOSURE_STEP,
				mode->exp_def);

	hi846->anal_gain = v4l2_ctrl_new_std(handler, &hi846_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, ANALOG_GAIN_MIN,
				ANALOG_GAIN_MAX, ANALOG_GAIN_STEP,
				ANALOG_GAIN_DEFAULT);

	hi846->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&hi846_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(hi846_test_pattern_menu) - 1,
				0, 0, hi846_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&hi846->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	hi846->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int hi846_check_sensor_id(struct hi846 *hi846,
				  struct i2c_client *client)
{
	struct device *dev = &hi846->client->dev;
	u32 id = 0;
	u32 reg_H = 0;
	u32 reg_L = 0;
	int ret;

	ret = hi846_read_reg(client, HI846_REG_CHIP_ID_H,
			      HI846_REG_VALUE_08BIT, &reg_H);
	ret |= hi846_read_reg(client, HI846_REG_CHIP_ID_L,
			      HI846_REG_VALUE_08BIT, &reg_L);
	id = ((reg_H << 8) & 0xff00) | (reg_L & 0xff);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%04x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected Hi%04x sensor\n", CHIP_ID);

	return 0;
}

static int hi846_configure_regulators(struct hi846 *hi846)
{
	unsigned int i;

	for (i = 0; i < HI846_NUM_SUPPLIES; i++)
		hi846->supplies[i].supply = hi846_supply_names[i];

	return devm_regulator_bulk_get(&hi846->client->dev,
				       HI846_NUM_SUPPLIES,
				       hi846->supplies);
}

static int hi846_parse_of(struct hi846 *hi846)
{
	struct device *dev = &hi846->client->dev;
	struct device_node *endpoint;
	struct fwnode_handle *fwnode;
	int rval;
	unsigned int fps;

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

	hi846->lane_num = rval;
	if (2 == hi846->lane_num) {
		hi846->cur_mode = &supported_modes_2lane[0];
		supported_modes = supported_modes_2lane;
		hi846->cfg_num = ARRAY_SIZE(supported_modes_2lane);
		hi846_global_regs = hi846_global_regs_2lane;

		/* pixel rate = HTS * VTS * FPS */
		fps = DIV_ROUND_CLOSEST(hi846->cur_mode->max_fps.denominator,
					hi846->cur_mode->max_fps.numerator);
		hi846->pixel_rate = hi846->cur_mode->vts_def *
				     hi846->cur_mode->hts_def * fps;

		dev_info(dev, "lane_num(%d)  pixel_rate(%u)\n",
				 hi846->lane_num, hi846->pixel_rate);
	} else if (4 == hi846->lane_num) {
		hi846->cur_mode = &supported_modes_4lane[0];
		supported_modes = supported_modes_4lane;
		hi846->cfg_num = ARRAY_SIZE(supported_modes_4lane);
		hi846_global_regs = hi846_global_regs_4lane;

		/* pixel rate = HTS * VTS * FPS */
		fps = DIV_ROUND_CLOSEST(hi846->cur_mode->max_fps.denominator,
					hi846->cur_mode->max_fps.numerator);
		hi846->pixel_rate = hi846->cur_mode->vts_def *
				     hi846->cur_mode->hts_def * fps;

		dev_info(dev, "lane_num(%d)  pixel_rate(%u)\n",
				 hi846->lane_num, hi846->pixel_rate);
	} else {
		dev_err(dev, "unsupported lane_num(%d)\n", hi846->lane_num);
		return -1;
	}

	return 0;
}

static int hi846_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct hi846 *hi846;
	struct v4l2_subdev *sd;
	char facing[2] = "b";
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	hi846 = devm_kzalloc(dev, sizeof(*hi846), GFP_KERNEL);
	if (!hi846)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &hi846->module_index);
	if (ret) {
		dev_warn(dev, "could not get module index!\n");
		hi846->module_index = 0;
	}
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &hi846->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &hi846->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &hi846->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	hi846->client = client;

	hi846->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(hi846->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	hi846->power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(hi846->power_gpio))
		dev_warn(dev, "Failed to get power-gpios, maybe no use\n");

	hi846->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(hi846->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios, maybe no use\n");

	hi846->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(hi846->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ret = hi846_configure_regulators(hi846);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}
	ret = hi846_parse_of(hi846);
	if (ret != 0)
		return -EINVAL;

	hi846->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(hi846->pinctrl)) {
		hi846->pins_default =
			pinctrl_lookup_state(hi846->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(hi846->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		hi846->pins_sleep =
			pinctrl_lookup_state(hi846->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(hi846->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	mutex_init(&hi846->mutex);

	sd = &hi846->subdev;
	v4l2_i2c_subdev_init(sd, client, &hi846_subdev_ops);
	ret = hi846_initialize_controls(hi846);
	if (ret)
		goto err_destroy_mutex;

	ret = __hi846_power_on(hi846);
	if (ret)
		goto err_free_handler;

	ret = hi846_check_sensor_id(hi846, client);
	if (ret < 0) {
		dev_info(&client->dev, "%s(%d) Check id  failed\n"
				  "check following information:\n"
				  "Power/PowerDown/Reset/Mclk/I2cBus !!\n",
				  __func__, __LINE__);
		goto err_power_off;
	}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &hi846_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	hi846->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &hi846->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(hi846->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 hi846->module_index, facing,
		 HI846_NAME, dev_name(sd->dev));

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
	__hi846_power_off(hi846);
err_free_handler:
	v4l2_ctrl_handler_free(&hi846->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&hi846->mutex);

	return ret;
}

static int hi846_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hi846 *hi846 = to_hi846(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&hi846->ctrl_handler);
	mutex_destroy(&hi846->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__hi846_power_off(hi846);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id hi846_of_match[] = {
	{ .compatible = "hynix,hi846" },
	{},
};
MODULE_DEVICE_TABLE(of, hi846_of_match);
#endif

static const struct i2c_device_id hi846_match_id[] = {
	{ "hynix,hi846", 0 },
	{ },
};

static struct i2c_driver hi846_i2c_driver = {
	.driver = {
		.name = HI846_NAME,
		.pm = &hi846_pm_ops,
		.of_match_table = of_match_ptr(hi846_of_match),
	},
	.probe		= &hi846_probe,
	.remove		= &hi846_remove,
	.id_table	= hi846_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&hi846_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&hi846_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("OmniVision hi846 sensor driver");
MODULE_LICENSE("GPL v2");
