// SPDX-License-Identifier: GPL-2.0
/*
 * HM2056 CMOS Image Sensor driver
 *
 * Copyright (C) 2018 Fuzhou Rockchip Electronics Co., Ltd.
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
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x0)
#define DRIVER_NAME "hm2056"
#define HM2056_PIXEL_RATE		(72 * 1000 * 1000)

/*
 * HM2056 register definitions
 */
#define REG_SC_CHIP_ID_H		0x0001
#define REG_SC_CHIP_ID_L		0x0002

#define HM2056_REG_MODE			0x0005
#define HM2056_MODE_SW_STANDBY		0x0
#define HM2056_MODE_STREAMING		0x1

#define REG_NULL			0xFFFF
#define REG_DELAY			0xFFFE

#define HM2056_REG_VALUE_08BIT		1
#define HM2056_REG_VALUE_16BIT		2
#define HM2056_REG_VALUE_24BIT		3

#define HM2056_ID			0x2056

struct sensor_register {
	u16 addr;
	u32 value;
};

struct hm2056_framesize {
	u16 width;
	u16 height;
	struct v4l2_fract max_fps;
	u16 max_exp_lines;
	const struct sensor_register *regs;
};

struct hm2056_pixfmt {
	u32 code;
	/* Output format Register Value (REG_FORMAT_CTRL00) */
	struct sensor_register *format_ctrl_regs;
};

static const char * const hm2056_supply_names[] = {
	"dovdd",	/* Digital I/O power */
	"avdd",		/* Analog power */
	"dvdd",		/* Digital core power */
};

#define HM2056_NUM_SUPPLIES ARRAY_SIZE(hm2056_supply_names)

struct hm2056 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	unsigned int fps;
	unsigned int xvclk_frequency;
	struct clk *xvclk;
	struct gpio_desc *pwdn_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *avdd_en_gpio;
	struct gpio_desc *dvdd_en_gpio;
	struct gpio_desc *dovdd_en_gpio;
	struct regulator_bulk_data supplies[HM2056_NUM_SUPPLIES];
	struct mutex lock;
	struct i2c_client *client;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_frequency;
	struct v4l2_fwnode_endpoint bus_cfg;
	const struct hm2056_framesize *frame_size;
	const struct hm2056_framesize *framesize_cfg;
	unsigned int cfg_num;
	int streaming;
	u32 module_index;
	const char *module_facing;
	const char *module_name;
	const char *len_name;
};

static const struct sensor_register hm2056_dvp_init_regs[] = {
	{0x0022, 0x00},
	{0x0020, 0x0000},
	{0x0025, 0x0000},
	{0x0026, 0x0087},
	{0x0027, 0x0030},
	{0x0028, 0x00C0},
	{0x002A, 0x001F},
	{0x002C, 0x000A},
	{0x0004, 0x0010},
	{0x0006, 0x0000},
	{0x000D, 0x0011},
	{0x000E, 0x0011},
	{0x000F, 0x0000},
	{0x0011, 0x0002},
	{0x0012, 0x001C},
	{0x0013, 0x0001},
	{0x0015, 0x0002},
	{0x0016, 0x0080},
	{0x0018, 0x0000},
	{0x001D, 0x0040},
	{0x0040, 0x0020},
	{0x0053, 0x000A},
	{0x0044, 0x0006},
	{0x0046, 0x00D8},
	{0x004A, 0x000A},
	{0x004B, 0x0072},
	{0x0075, 0x0001},
	{0x0070, 0x005F},
	{0x0071, 0x00FF},
	{0x0072, 0x0055},
	{0x0073, 0x0050},
	{0x0077, 0x0004},
	{0x0080, 0x00C8},
	{0x0082, 0x00A2},
	{0x0083, 0x00F0},
	{0x0085, 0x0011},
	{0x0086, 0x0002},
	{0x0087, 0x0080},
	{0x0088, 0x006C},
	{0x0089, 0x002E},
	{0x008A, 0x006D},
	{0x008D, 0x0020},
	{0x0090, 0x0000},
	{0x0091, 0x0010},
	{0x0092, 0x0011},
	{0x0093, 0x0012},
	{0x0094, 0x0016},
	{0x0095, 0x0008},
	{0x0096, 0x0000},
	{0x0097, 0x0010},
	{0x0098, 0x0011},
	{0x0099, 0x0012},
	{0x009A, 0x0016},
	{0x009B, 0x0034},
	{0x00A0, 0x0000},
	{0x00A1, 0x0004},
	{0x011F, 0x00F7},
	{0x0120, 0x0036},
	{0x0121, 0x0083},
	{0x0122, 0x007B},
	{0x0123, 0x00C2},
	{0x0124, 0x00CE},//ccm off
	{0x0125, 0x00FF},
	{0x0126, 0x0070},
	{0x0128, 0x001F},
	{0x0132, 0x0010},
	{0x0136, 0x000A},
	{0x0131, 0x00BD},
	{0x0140, 0x0014},
	{0x0141, 0x000A},
	{0x0142, 0x0014},
	{0x0143, 0x000A},
	{0x0144, 0x0006},
	{0x0145, 0x0000},
	{0x0146, 0x0020},
	{0x0147, 0x000A},
	{0x0148, 0x0010},
	{0x0149, 0x000C},
	{0x014A, 0x0080},
	{0x014B, 0x0080},
	{0x014C, 0x002E},
	{0x014D, 0x002E},
	{0x014E, 0x0005},
	{0x014F, 0x0005},
	{0x0150, 0x000D},
	{0x0155, 0x0000},
	{0x0156, 0x0010},
	{0x0157, 0x000A},
	{0x0158, 0x000A},
	{0x0159, 0x000A},
	{0x015A, 0x0005},
	{0x015B, 0x0005},
	{0x015C, 0x0005},
	{0x015D, 0x0005},
	{0x015E, 0x0008},
	{0x015F, 0x00FF},
	{0x0160, 0x0050},
	{0x0161, 0x0020},
	{0x0162, 0x0014},
	{0x0163, 0x000A},
	{0x0164, 0x0010},
	{0x0165, 0x0008},
	{0x0166, 0x000A},
	{0x018C, 0x0024},
	{0x018D, 0x0004},
	{0x018E, 0x0000},
	{0x018F, 0x0011},
	{0x0190, 0x0080},
	{0x0191, 0x0047},
	{0x0192, 0x0048},
	{0x0193, 0x0064},
	{0x0194, 0x0032},
	{0x0195, 0x00c8},
	{0x0196, 0x0096},
	{0x0197, 0x0064},
	{0x0198, 0x0032},
	{0x0199, 0x0014},
	{0x019A, 0x0020},
	{0x019B, 0x0014},
	{0x01BA, 0x0010},
	{0x01BB, 0x0004},
	{0x01D8, 0x0040},
	{0x01DE, 0x0060},
	{0x01E4, 0x0010},
	{0x01E5, 0x0010},
	{0x01F2, 0x000C},
	{0x01F3, 0x0014},
	{0x01F8, 0x0004},
	{0x01F9, 0x000C},
	{0x01FE, 0x0002},
	{0x01FF, 0x0004},
	{0x0220, 0x0000},
	{0x0221, 0x00B0},
	{0x0222, 0x0000},
	{0x0223, 0x0080},
	{0x0224, 0x008E},
	{0x0225, 0x0000},
	{0x0226, 0x0088},
	{0x022A, 0x0088},
	{0x022B, 0x0000},
	{0x022C, 0x0088},
	{0x022D, 0x0013},
	{0x022E, 0x000B},
	{0x022F, 0x0013},
	{0x0230, 0x000B},
	{0x0233, 0x0013},
	{0x0234, 0x000B},
	{0x0235, 0x0028},
	{0x0236, 0x0003},
	{0x0237, 0x0028},
	{0x0238, 0x0003},
	{0x023B, 0x0028},
	{0x023C, 0x0003},
	{0x023D, 0x005C},
	{0x023E, 0x0002},
	{0x023F, 0x005C},
	{0x0240, 0x0002},
	{0x0243, 0x005C},
	{0x0244, 0x0002},
	{0x0251, 0x000E},
	{0x0252, 0x0000},
	{0x0280, 0x000A},
	{0x0282, 0x0014},
	{0x0284, 0x002A},
	{0x0286, 0x0050},
	{0x0288, 0x0060},
	{0x028A, 0x006D},
	{0x028C, 0x0079},
	{0x028E, 0x0082},
	{0x0290, 0x008A},
	{0x0292, 0x0091},
	{0x0294, 0x009C},
	{0x0296, 0x00A7},
	{0x0298, 0x00BA},
	{0x029A, 0x00CD},
	{0x029C, 0x00E0},
	{0x029E, 0x002D},
	{0x02A0, 0x0000},//06
	{0x02E0, 0x0004},
	{0x02C0, 0x00B1},
	{0x02C1, 0x0001},
	{0x02C2, 0x007D},
	{0x02C3, 0x0007},
	{0x02C4, 0x00D2},
	{0x02C5, 0x0007},
	{0x02C6, 0x00C4},
	{0x02C7, 0x0007},
	{0x02C8, 0x0079},
	{0x02C9, 0x0001},
	{0x02CA, 0x00C4},
	{0x02CB, 0x0007},
	{0x02CC, 0x00F7},
	{0x02CD, 0x0007},
	{0x02CE, 0x003B},
	{0x02CF, 0x0007},
	{0x02D0, 0x00CF},
	{0x02D1, 0x0001},
	{0x0302, 0x0000},
	{0x0303, 0x0000},
	{0x0304, 0x0000},
	{0x02F0, 0x005E},
	{0x02F1, 0x0007},
	{0x02F2, 0x00A0},
	{0x02F3, 0x0000},
	{0x02F4, 0x0002},
	{0x02F5, 0x0000},
	{0x02F6, 0x00C4},
	{0x02F7, 0x0007},
	{0x02F8, 0x0011},
	{0x02F9, 0x0000},
	{0x02FA, 0x002A},
	{0x02FB, 0x0000},
	{0x02FC, 0x00A1},
	{0x02FD, 0x0007},
	{0x02FE, 0x00B8},
	{0x02FF, 0x0007},
	{0x0300, 0x00A7},
	{0x0301, 0x0000},
	{0x0305, 0x0000},
	{0x0306, 0x0000},
	{0x0307, 0x007A},
	{0x032D, 0x0000},
	{0x032E, 0x0001},
	{0x032F, 0x0000},
	{0x0330, 0x0001},
	{0x0331, 0x0000},
	{0x0332, 0x0001},
	{0x0333, 0x0082},
	{0x0334, 0x0000},
	{0x0335, 0x0084},
	{0x0336, 0x0000},
	{0x0337, 0x0001},
	{0x0338, 0x0000},
	{0x0339, 0x0001},
	{0x033A, 0x0000},
	{0x033B, 0x0001},
	{0x0340, 0x0030},
	{0x0341, 0x0044},
	{0x0342, 0x004A},
	{0x0343, 0x0042},
	{0x0344, 0x0074},
	{0x0345, 0x004F},
	{0x0346, 0x0067},
	{0x0347, 0x005C},
	{0x0348, 0x0059},
	{0x0349, 0x0067},
	{0x034A, 0x004D},
	{0x034B, 0x006E},
	{0x034C, 0x0044},
	{0x0350, 0x0080},
	{0x0351, 0x0080},
	{0x0352, 0x0018},
	{0x0353, 0x0018},
	{0x0354, 0x006E},
	{0x0355, 0x004A},
	{0x0356, 0x007A},
	{0x0357, 0x00C6},
	{0x0358, 0x0006},
	{0x035A, 0x0006},
	{0x035B, 0x00A0},
	{0x035C, 0x0073},
	{0x035D, 0x005A},
	{0x035E, 0x00C6},
	{0x035F, 0x00A0},
	{0x0360, 0x0002},
	{0x0361, 0x0018},
	{0x0362, 0x0080},
	{0x0363, 0x006C},
	{0x0364, 0x0000},
	{0x0365, 0x00F0},
	{0x0366, 0x0020},
	{0x0367, 0x000C},
	{0x0369, 0x0000},
	{0x036A, 0x0010},
	{0x036B, 0x0010},
	{0x036E, 0x0020},
	{0x036F, 0x0000},
	{0x0370, 0x0010},
	{0x0371, 0x0018},
	{0x0372, 0x000C},
	{0x0373, 0x0038},
	{0x0374, 0x003A},
	{0x0375, 0x0013},
	{0x0376, 0x0022},
	{0x0380, 0x00FD},//awb off
	{0x0381, 0x004A},
	{0x0382, 0x0020},
	{0x038A, 0x0040},
	{0x038B, 0x0008},
	{0x038C, 0x00C1},
	{0x038E, 0x0038},
	{0x038F, 0x0005},
	{0x0390, 0x00E0},
	{0x0391, 0x0005},
	{0x0393, 0x0080},
	{0x0395, 0x0021},
	{0x0398, 0x0002},
	{0x0399, 0x0074},
	{0x039A, 0x0003},
	{0x039B, 0x0011},
	{0x039C, 0x0003},
	{0x039D, 0x00AE},
	{0x039E, 0x0004},
	{0x039F, 0x00E8},
	{0x03A0, 0x0006},
	{0x03A1, 0x0022},
	{0x03A2, 0x0007},
	{0x03A3, 0x005C},
	{0x03A4, 0x0009},
	{0x03A5, 0x00D0},
	{0x03A6, 0x000C},
	{0x03A7, 0x000E},
	{0x03A8, 0x0010},
	{0x03A9, 0x0018},
	{0x03AA, 0x0020},
	{0x03AB, 0x0028},
	{0x03AC, 0x001E},
	{0x03AD, 0x001A},
	{0x03AE, 0x0013},
	{0x03AF, 0x000C},
	{0x03B0, 0x000B},
	{0x03B1, 0x0009},
	{0x03B3, 0x0010},
	{0x03B4, 0x0000},
	{0x03B5, 0x0010},
	{0x03B6, 0x0000},
	{0x03B7, 0x00EA},
	{0x03B8, 0x0000},
	{0x03B9, 0x003A},
	{0x03BA, 0x0001},
	{0x03D0, 0x00F8},
	{0x03BB, 0x0000},
	{0x03BC, 0x0087},
	{0x03BD, 0x00c3},
	{0x03BE, 0x0001},
	{0x03BF, 0x0000},
	{0x03E0, 0x0044},
	{0x03E1, 0x0044},
	{0x03E2, 0x0004},
	{0x03E4, 0x0004},
	{0x03E5, 0x0002},
	{0x03E6, 0x0004},
	{0x03E8, 0x0014},
	{0x03E9, 0x0013},
	{0x03EA, 0x0004},
	{0x03EC, 0x0004},
	{0x03ED, 0x0002},
	{0x03EE, 0x0004},
	{0x03F0, 0x0024},
	{0x03F1, 0x0022},
	{0x03F2, 0x0004},
	{0x0420, 0x0084},
	{0x0421, 0x0000},
	{0x0422, 0x0000},
	{0x0423, 0x0083},
	{0x0430, 0x0008},
	{0x0431, 0x0028},
	{0x0432, 0x0010},
	{0x0433, 0x0008},
	{0x0435, 0x000C},
	{0x0450, 0x00FF},
	{0x0451, 0x00FF},
	{0x0452, 0x00FF},
	{0x0453, 0x00FF},
	{0x0454, 0x00FF},
	{0x0455, 0x00FF},
	{0x0456, 0x00FF},
	{0x0457, 0x00FF},
	{0x0458, 0x0098},
	{0x0459, 0x0003},
	{0x045A, 0x0000},
	{0x045B, 0x0028},
	{0x045C, 0x0000},
	{0x045D, 0x0068},
	{0x0466, 0x0014},
	{0x047A, 0x0000},
	{0x047B, 0x0000},
	{0x0480, 0x0000},//saturation
	{0x0481, 0x0006},
	{0x0482, 0x000C},
	{0x04B0, 0x0040},
	{0x04B6, 0x0030},
	{0x04B9, 0x0010},
	{0x04B3, 0x0010},
	{0x04B1, 0x008E},
	{0x04B4, 0x0020},
	{0x04c0, 0x0000},
	{0x0540, 0x0000},
	{0x0541, 0x009D},
	{0x0542, 0x0000},
	{0x0543, 0x00BC},
	{0x0580, 0x0001},
	{0x0581, 0x000F},
	{0x0582, 0x0004},
	{0x0594, 0x0000},
	{0x0595, 0x0004},
	{0x05A9, 0x0003},
	{0x05AA, 0x0040},
	{0x05AB, 0x0080},
	{0x05AC, 0x000A},
	{0x05AD, 0x0010},
	{0x05AE, 0x000C},
	{0x05AF, 0x000C},
	{0x05B0, 0x0003},
	{0x05B1, 0x0003},
	{0x05B2, 0x001C},
	{0x05B3, 0x0002},
	{0x05B4, 0x0000},
	{0x05B5, 0x000C},
	{0x05B8, 0x0080},
	{0x05B9, 0x0032},
	{0x05BA, 0x0000},
	{0x05BB, 0x0080},
	{0x05BC, 0x0003},
	{0x05BD, 0x0000},
	{0x05BF, 0x0005},
	{0x05C0, 0x0010},
	{0x05C3, 0x0000},
	{0x05C4, 0x000C},
	{0x05C5, 0x0020},
	{0x05C7, 0x0001},
	{0x05C8, 0x0014},
	{0x05C9, 0x0054},
	{0x05CA, 0x0014},
	{0x05CB, 0x00E0},
	{0x05CC, 0x0020},
	{0x05CD, 0x0000},
	{0x05CE, 0x0008},
	{0x05CF, 0x0060},
	{0x05D0, 0x0010},
	{0x05D1, 0x0005},
	{0x05D2, 0x0003},
	{0x05D4, 0x0000},
	{0x05D5, 0x0005},
	{0x05D6, 0x0005},
	{0x05D7, 0x0005},
	{0x05D8, 0x0008},
	{0x05DC, 0x000C},
	{0x05D9, 0x0000},
	{0x05DB, 0x0000},
	{0x05DD, 0x000F},
	{0x05DE, 0x0000},
	{0x05DF, 0x000A},
	{0x05E0, 0x00A0},
	{0x05E1, 0x0000},
	{0x05E2, 0x00A0},
	{0x05E3, 0x0000},
	{0x05E4, 0x0004},
	{0x05E5, 0x0000},
	{0x05E6, 0x0083},
	{0x05E7, 0x0002},
	{0x05E8, 0x0006},
	{0x05E9, 0x0000},
	{0x05EA, 0x00E5},
	{0x05EB, 0x0001},
	{0x0660, 0x0004},
	{0x0661, 0x0016},
	{0x0662, 0x0004},
	{0x0663, 0x0028},
	{0x0664, 0x0004},
	{0x0665, 0x0018},
	{0x0666, 0x0004},
	{0x0667, 0x0021},
	{0x0668, 0x0004},
	{0x0669, 0x000C},
	{0x066A, 0x0004},
	{0x066B, 0x0025},
	{0x066C, 0x0000},
	{0x066D, 0x0012},
	{0x066E, 0x0000},
	{0x066F, 0x0080},
	{0x0670, 0x0000},
	{0x0671, 0x000A},
	{0x0672, 0x0004},
	{0x0673, 0x001D},
	{0x0674, 0x0004},
	{0x0675, 0x001D},
	{0x0676, 0x0000},
	{0x0677, 0x007E},
	{0x0678, 0x0001},
	{0x0679, 0x0047},
	{0x067A, 0x0000},
	{0x067B, 0x0073},
	{0x067C, 0x0004},
	{0x067D, 0x0014},
	{0x067E, 0x0004},
	{0x067F, 0x0028},
	{0x0680, 0x0000},
	{0x0681, 0x0022},
	{0x0682, 0x0000},
	{0x0683, 0x00A5},
	{0x0684, 0x0000},
	{0x0685, 0x001E},
	{0x0686, 0x0004},
	{0x0687, 0x001D},
	{0x0688, 0x0004},
	{0x0689, 0x0019},
	{0x068A, 0x0004},
	{0x068B, 0x0021},
	{0x068C, 0x0004},
	{0x068D, 0x000A},
	{0x068E, 0x0004},
	{0x068F, 0x0025},
	{0x0690, 0x0004},
	{0x0691, 0x0015},
	{0x0698, 0x0020},
	{0x0699, 0x0020},
	{0x069A, 0x0001},
	{0x069C, 0x0030},
	{0x069D, 0x0010},
	{0x069E, 0x0018},
	{0x069F, 0x0008},
	{0x0000, 0x0001},
	{0x0100, 0x0001},
	{0x0101, 0x0001},
	{REG_NULL, 0x00},
};

/* Senor full resolution setting */
static const struct sensor_register hm2056_dvp_1600x1200_15fps[] = {
	{0x0005, 0x00},
	{0x0006, 0x00},
	{0x000D, 0x00},
	{0x000E, 0x00},
	{0x0010, 0x00},
	{0x0011, 0x02},
	{0x0012, 0x04},
	{0x0013, 0x00},
	{0x002A, 0x2F},
	{0x0070, 0x5F},
	{0x0071, 0xAB},
	{0x0072, 0x55},
	{0x0073, 0x50},
	{0x0082, 0xE2},
	{0x011F, 0xFF},
	{0x0125, 0xDF},
	{0x0126, 0x70},
	{0x0131, 0xBC},
	{0x0144, 0x04},
	{0x0190, 0x87},
	{0x0192, 0x50},
#if 0
	{0x038F, 0x04},
	{0x0390, 0xDB},
#else
	//change for 850nm pass
	{0x038F, 0x07},
	{0x0390, 0x44},
	{0x0392, 0x04},
	{0x0393, 0x80},
#endif
	{0x03B7, 0xEA},
	{0x03B8, 0x00},
	{0x03B9, 0x3A},
	{0x03BA, 0x01},
	{0x0541, 0x9B},
	{0x0543, 0xBA},
	{0x05E4, 0x0A},
	{0x05E5, 0x00},
	{0x05E6, 0x49},
	{0x05E7, 0x06},
	{0x05E8, 0x0A},
	{0x05E9, 0x00},
	{0x05EA, 0xB9},
	{0x05EB, 0x04},
	{0x0000, 0x01},
	{0x0100, 0x01},
	{0x0101, 0x01},
	{0x0005, 0x01},
	{REG_NULL, 0x00},
};

/* Preview resolution setting*/
static const struct sensor_register hm2056_dvp_1280x720_21fps[] = {
	{0x0005, 0x00},
	{0x0006, 0x08},
	{0x000D, 0x00},
	{0x000E, 0x00},
	{0x0010, 0x00},
	{0x0011, 0x02},
	{0x0012, 0x08},
	{0x0013, 0x00},
	{0x002A, 0x2F},
	{0x0070, 0x5F},
	{0x0071, 0x99},
	{0x0072, 0x55},
	{0x0073, 0x50},
	{0x0082, 0xE2},
	{0x011F, 0xFF},
	{0x0125, 0xDF},
	{0x0126, 0x70},
	{0x0131, 0xBC},
	{0x0144, 0x04},
	{0x0190, 0x87},
	{0x0192, 0x50},
#if 0
	{0x038F, 0x02},
	{0x0390, 0xF0},
#else
	//change for 850nm pass
	{0x038F, 0x05},//15-30fps
	{0x0390, 0xe0},
	{0x0392, 0x03}, //04
	{0x0393, 0x80},
#endif
	{0x03B7, 0x8D},
	{0x03B8, 0x00},
	{0x03B9, 0xFF},
	{0x03BA, 0x00},
	{0x0540, 0x00},
	{0x0541, 0xBC},
	{0x0542, 0x00},
	{0x0543, 0xE1},
	{0x05E4, 0x08},
	{0x05E5, 0x00},
	{0x05E6, 0x07},
	{0x05E7, 0x05},
	{0x05E8, 0x06},
	{0x05E9, 0x00},
	{0x05EA, 0xD5},
	{0x05EB, 0x02},
	{0x0000, 0x01},
	{0x0100, 0x01},
	{0x0101, 0x01},
	{0x0005, 0x01},
	{REG_NULL, 0x00},
};

/* Preview resolution setting*/
static const struct sensor_register hm2056_dvp_640x480_30fps[] = {
	{0x0005, 0x00},
	{0x0006, 0x00},
	{0x000D, 0x11},
	{0x000E, 0x11},
	{0x0010, 0x00},
	{0x0011, 0x02},
	{0x0012, 0x1C},
	{0x0013, 0x01},
	{0x002A, 0x1F},
	{0x0070, 0x5F},
	{0x0071, 0xFF},
	{0x0072, 0x55},
	{0x0073, 0x50},
	{0x0082, 0xA2},
	{0x011F, 0xF7},
	{0x0125, 0xFF},
	{0x0126, 0x70},
	{0x0131, 0xBD},
	{0x0144, 0x06},
	{0x0190, 0x80},
	{0x0192, 0x48},
#if 0
	{0x038F, 0x02},
	{0x0390, 0xF0},
#else
	//change for 850nm pass
	{0x038F, 0x03},
	{0x0390, 0x11},
	{0x0392, 0x04},
	{0x0393, 0x80},
#endif
	{0x03B7, 0xEA},
	{0x03B8, 0x00},
	{0x03B9, 0x3A},
	{0x03BA, 0x01},
	{0x0540, 0x00},
	{0x0541, 0x9D},
	{0x0542, 0x00},
	{0x0543, 0xBD},
	{0x05E0, 0xA0},
	{0x05E1, 0x00},
	{0x05E2, 0xA0},
	{0x05E3, 0x00},
	{0x05E4, 0x04},
	{0x05E5, 0x00},
	{0x05E6, 0x83},
	{0x05E7, 0x02},
	{0x05E8, 0x06},
	{0x05E9, 0x00},
	{0x05EA, 0xE5},
	{0x05EB, 0x01},
	{0x0000, 0x01},
	{0x0100, 0x01},
	{0x0101, 0x01},
	{0x0005, 0x01},
	{REG_NULL, 0x00},
};

static const struct hm2056_framesize hm2056_dvp_framesizes[] = {
	{ /* 480p */
		.width		= 640,
		.height		= 480,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.regs		= hm2056_dvp_640x480_30fps,
	}, { /* 720p */
		.width		= 1280,
		.height		= 720,
		.max_fps = {
			.numerator = 10000,
			.denominator = 210000,
		},
		.regs		= hm2056_dvp_1280x720_21fps,
	}, { /* FULL */
		.width		= 1600,
		.height		= 1200,
		.max_fps = {
			.numerator = 10000,
			.denominator = 150000,
		},
		.regs		= hm2056_dvp_1600x1200_15fps,
	}
};

static const s64 link_freq_menu_items[] = {
	240000000
};

static const struct hm2056_pixfmt hm2056_formats[] = {
	{
		.code = MEDIA_BUS_FMT_YUYV8_2X8,
	}
};

static inline struct hm2056 *to_hm2056(struct v4l2_subdev *sd)
{
	return container_of(sd, struct hm2056, sd);
}

/* Write registers up to 4 at a time */
static int hm2056_write_reg(struct i2c_client *client, u16 reg,
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

static int hm2056_write_array(struct i2c_client *client,
			      const struct sensor_register *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		if (unlikely(regs[i].addr == REG_DELAY))
			usleep_range(regs[i].value * 1000, regs[i].value * 2000);
		else
			ret = hm2056_write_reg(client, regs[i].addr,
						HM2056_REG_VALUE_08BIT,
						regs[i].value);

	return ret;
}

/* Read registers up to 4 at a time */
static int hm2056_read_reg(struct i2c_client *client, u16 reg,
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

static void hm2056_get_default_format(struct hm2056 *hm2056,
				      struct v4l2_mbus_framefmt *format)
{
	format->width = hm2056->framesize_cfg[0].width;
	format->height = hm2056->framesize_cfg[0].height;
	format->colorspace = V4L2_COLORSPACE_SRGB;
	format->code = hm2056_formats[0].code;
	format->field = V4L2_FIELD_NONE;
}

static void hm2056_set_streaming(struct hm2056 *hm2056, int on)
{
	struct i2c_client *client = hm2056->client;
	int ret = 0;
	u8 val;

	dev_dbg(&client->dev, "%s: on: %d\n", __func__, on);

	val = on ? HM2056_MODE_STREAMING : HM2056_MODE_SW_STANDBY;
	ret = hm2056_write_reg(client,
		HM2056_REG_MODE,
		HM2056_REG_VALUE_08BIT,
		val);
	if (ret)
		dev_err(&client->dev, "hm2056 set stream failed\n");
}

/*
 * V4L2 subdev video and pad level operations
 */

static int hm2056_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	if (code->index >= ARRAY_SIZE(hm2056_formats))
		return -EINVAL;

	code->code = hm2056_formats[code->index].code;

	return 0;
}

static int hm2056_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct hm2056 *hm2056 = to_hm2056(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int i = ARRAY_SIZE(hm2056_formats);

	dev_dbg(&client->dev, "%s:\n", __func__);

	if (fse->index >= hm2056->cfg_num)
		return -EINVAL;

	while (--i)
		if (fse->code == hm2056_formats[i].code)
			break;

	fse->code = hm2056_formats[i].code;

	fse->min_width  = hm2056->framesize_cfg[fse->index].width;
	fse->max_width  = fse->min_width;
	fse->max_height = hm2056->framesize_cfg[fse->index].height;
	fse->min_height = fse->max_height;

	return 0;
}

static int hm2056_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hm2056 *hm2056 = to_hm2056(sd);

	dev_dbg(&client->dev, "%s enter\n", __func__);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		struct v4l2_mbus_framefmt *mf;

		mf = v4l2_subdev_get_try_format(sd, cfg, 0);
		mutex_lock(&hm2056->lock);
		fmt->format = *mf;
		mutex_unlock(&hm2056->lock);
		return 0;
#else
	return -ENOTTY;
#endif
	}

	mutex_lock(&hm2056->lock);
	fmt->format = hm2056->format;
	mutex_unlock(&hm2056->lock);

	dev_dbg(&client->dev, "%s: %x %dx%d\n", __func__,
		hm2056->format.code, hm2056->format.width,
		hm2056->format.height);

	return 0;
}

static void __hm2056_try_frame_size_fps(struct hm2056 *hm2056,
					struct v4l2_mbus_framefmt *mf,
					const struct hm2056_framesize **size,
					unsigned int fps)
{
	const struct hm2056_framesize *fsize = &hm2056->framesize_cfg[0];
	const struct hm2056_framesize *match = NULL;
	unsigned int i = hm2056->cfg_num;
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

	if (!match) {
		match = &hm2056->framesize_cfg[0];
	} else {
		fsize = &hm2056->framesize_cfg[0];
		for (i = 0; i < hm2056->cfg_num; i++) {
			if (fsize->width == match->width &&
			    fsize->height == match->height &&
			    fps >= DIV_ROUND_CLOSEST(fsize->max_fps.denominator,
				fsize->max_fps.numerator))
				match = fsize;

			fsize++;
		}
	}

	mf->width  = match->width;
	mf->height = match->height;

	if (size)
		*size = match;
}

static int hm2056_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int index = ARRAY_SIZE(hm2056_formats);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	const struct hm2056_framesize *size = NULL;
	struct hm2056 *hm2056 = to_hm2056(sd);
	int ret = 0;

	dev_dbg(&client->dev, "%s enter\n", __func__);

	__hm2056_try_frame_size_fps(hm2056, mf, &size, hm2056->fps);

	while (--index >= 0)
		if (hm2056_formats[index].code == mf->code)
			break;

	if (index < 0)
		return -EINVAL;

	mf->colorspace = V4L2_COLORSPACE_SRGB;
	mf->code = hm2056_formats[index].code;
	mf->field = V4L2_FIELD_NONE;

	mutex_lock(&hm2056->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		*mf = fmt->format;
#else
		return -ENOTTY;
#endif
	} else {
		if (hm2056->streaming) {
			mutex_unlock(&hm2056->lock);
			return -EBUSY;
		}

		hm2056->frame_size = size;
		hm2056->format = fmt->format;
	}

	mutex_unlock(&hm2056->lock);
	return ret;
}

static int hm2056_s_stream(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hm2056 *hm2056 = to_hm2056(sd);
	int ret = 0;

	dev_dbg(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
		hm2056->frame_size->width,
		hm2056->frame_size->height,
		DIV_ROUND_CLOSEST(hm2056->frame_size->max_fps.denominator,
				  hm2056->frame_size->max_fps.numerator));

	mutex_lock(&hm2056->lock);

	on = !!on;

	if (hm2056->streaming == on)
		goto unlock;

	if (!on) {
		/* Stop Streaming Sequence */
		hm2056_set_streaming(hm2056, on);
		hm2056->streaming = on;
		goto unlock;
	}

	ret = hm2056_write_array(client, hm2056->frame_size->regs);
	if (ret)
		goto unlock;

	hm2056_set_streaming(hm2056, on);
	hm2056->streaming = on;

unlock:
	mutex_unlock(&hm2056->lock);
	return ret;
}

static int hm2056_set_test_pattern(struct hm2056 *hm2056, int value)
{
	return 0;
}

static int hm2056_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hm2056 *hm2056 =
			container_of(ctrl->handler, struct hm2056, ctrls);

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		return hm2056_set_test_pattern(hm2056, ctrl->val);
	}

	return 0;
}

static const struct v4l2_ctrl_ops hm2056_ctrl_ops = {
	.s_ctrl = hm2056_s_ctrl,
};

static const char * const hm2056_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bars",
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev internal operations
 */

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int hm2056_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct hm2056 *hm2056 = to_hm2056(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);

	dev_dbg(&client->dev, "%s:\n", __func__);

	hm2056_get_default_format(hm2056, format);

	return 0;
}
#endif

static int hm2056_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	struct hm2056 *hm2056 = to_hm2056(sd);

	if (hm2056->bus_cfg.bus_type == V4L2_MBUS_PARALLEL) {
		config->type = V4L2_MBUS_PARALLEL;
		config->flags = V4L2_MBUS_HSYNC_ACTIVE_HIGH |
				V4L2_MBUS_VSYNC_ACTIVE_LOW |
				V4L2_MBUS_PCLK_SAMPLE_RISING;
	}

	return 0;
}

static int hm2056_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct hm2056 *hm2056 = to_hm2056(sd);

	mutex_lock(&hm2056->lock);
	fi->interval = hm2056->frame_size->max_fps;
	mutex_unlock(&hm2056->lock);

	return 0;
}

static int hm2056_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hm2056 *hm2056 = to_hm2056(sd);
	const struct hm2056_framesize *size = NULL;
	struct v4l2_mbus_framefmt mf;
	unsigned int fps;
	int ret = 0;

	dev_dbg(&client->dev, "Setting %d/%d frame interval\n",
		fi->interval.numerator, fi->interval.denominator);

	mutex_lock(&hm2056->lock);

	fps = DIV_ROUND_CLOSEST(fi->interval.denominator,
				fi->interval.numerator);
	mf = hm2056->format;
	__hm2056_try_frame_size_fps(hm2056, &mf, &size, fps);

	if (hm2056->frame_size != size) {
		dev_info(&client->dev, "%s match wxh@FPS is %dx%d@%d\n",
			__func__, size->width, size->height,
			DIV_ROUND_CLOSEST(size->max_fps.denominator,
				size->max_fps.numerator));
		ret = hm2056_write_array(client, size->regs);
		if (ret)
			goto unlock;
		hm2056->frame_size = size;
		hm2056->fps = fps;
	}
unlock:
	mutex_unlock(&hm2056->lock);

	return ret;
}

static void hm2056_get_module_inf(struct hm2056 *hm2056,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, DRIVER_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, hm2056->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, hm2056->len_name, sizeof(inf->base.lens));
}

static long hm2056_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct hm2056 *hm2056 = to_hm2056(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		hm2056_get_module_inf(hm2056, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long hm2056_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = hm2056_ioctl(sd, cmd, inf);
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

static int hm2056_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct hm2056 *hm2056 = to_hm2056(sd);
	struct i2c_client *client = hm2056->client;

	dev_dbg(&client->dev, "%s(%d)\n", __func__, __LINE__);

	if (hm2056->bus_cfg.bus_type == V4L2_MBUS_PARALLEL)
		ret = hm2056_write_array(client, hm2056_dvp_init_regs);
	return ret;
}

static int hm2056_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	struct hm2056 *hm2056 = to_hm2056(sd);
	struct i2c_client *client = hm2056->client;
	struct device *dev = &hm2056->client->dev;

	dev_dbg(&client->dev, "%s(%d) on(%d)\n", __func__, __LINE__, on);
	if (on) {
		if (!IS_ERR(hm2056->pwdn_gpio)) {
			gpiod_set_value_cansleep(hm2056->pwdn_gpio, 0);
			usleep_range(2000, 5000);
		}
		ret = hm2056_init(sd, 0);
		if (ret)
			dev_err(dev, "init error\n");
	} else {
		if (!IS_ERR(hm2056->pwdn_gpio)) {
			gpiod_set_value_cansleep(hm2056->pwdn_gpio, 1);
			usleep_range(2000, 5000);
		}
	}
	return 0;
}

static int hm2056_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	struct hm2056 *hm2056 = to_hm2056(sd);

	if (fie->index >= hm2056->cfg_num)
		return -EINVAL;

	if (fie->code != MEDIA_BUS_FMT_YUYV8_2X8)
		return -EINVAL;

	fie->width = hm2056->framesize_cfg[fie->index].width;
	fie->height = hm2056->framesize_cfg[fie->index].height;
	fie->interval = hm2056->framesize_cfg[fie->index].max_fps;
	return 0;
}

static const struct v4l2_subdev_core_ops hm2056_subdev_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.ioctl = hm2056_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = hm2056_compat_ioctl32,
#endif
	.s_power = hm2056_power,
};

static const struct v4l2_subdev_video_ops hm2056_subdev_video_ops = {
	.s_stream = hm2056_s_stream,
	.g_mbus_config = hm2056_g_mbus_config,
	.g_frame_interval = hm2056_g_frame_interval,
	.s_frame_interval = hm2056_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops hm2056_subdev_pad_ops = {
	.enum_mbus_code = hm2056_enum_mbus_code,
	.enum_frame_size = hm2056_enum_frame_sizes,
	.enum_frame_interval = hm2056_enum_frame_interval,
	.get_fmt = hm2056_get_fmt,
	.set_fmt = hm2056_set_fmt,
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_ops hm2056_subdev_ops = {
	.core  = &hm2056_subdev_core_ops,
	.video = &hm2056_subdev_video_ops,
	.pad   = &hm2056_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops hm2056_subdev_internal_ops = {
	.open = hm2056_open,
};
#endif

static int hm2056_detect(struct hm2056 *hm2056)
{
	struct i2c_client *client = hm2056->client;
	u32 pid_h = 0;
	u32 pid_l = 0;
	u32 id = 0;
	int ret;

	dev_dbg(&client->dev, "%s:\n", __func__);

	/* Check sensor revision */
	ret = hm2056_read_reg(client,
		REG_SC_CHIP_ID_H,
		HM2056_REG_VALUE_08BIT,
		&pid_h);
	ret |= hm2056_read_reg(client,
		REG_SC_CHIP_ID_L,
		HM2056_REG_VALUE_08BIT,
		&pid_l);
	id = pid_h << 8 | pid_l;
	if (!ret) {
		if (id != HM2056_ID) {
			ret = -1;
			dev_err(&client->dev,
				"Sensor detection failed (%04X, %d)\n",
				id, ret);
		} else {
			dev_info(&client->dev, "Found HM%04X sensor\n", id);
		}
	}

	return ret;
}

static int __hm2056_power_on(struct hm2056 *hm2056)
{
	int ret;
	struct device *dev = &hm2056->client->dev;

	dev_dbg(dev, "%s(%d)\n", __func__, __LINE__);
	if (!IS_ERR(hm2056->reset_gpio)) {
		gpiod_set_value_cansleep(hm2056->reset_gpio, 0);
		usleep_range(2000, 5000);
		gpiod_set_value_cansleep(hm2056->reset_gpio, 1);
		usleep_range(2000, 5000);
	}

	if (!IS_ERR(hm2056->xvclk)) {
		ret = clk_set_rate(hm2056->xvclk, 24000000);
		if (ret < 0)
			dev_info(dev, "Failed to set xvclk rate (24MHz)\n");
	}

	if (!IS_ERR(hm2056->avdd_en_gpio))
		gpiod_set_value_cansleep(hm2056->avdd_en_gpio, 1);
	if (!IS_ERR(hm2056->dovdd_en_gpio)) {
		gpiod_set_value_cansleep(hm2056->dovdd_en_gpio, 1);
		usleep_range(100, 200);
	}
	if (!IS_ERR(hm2056->dvdd_en_gpio)) {
		gpiod_set_value_cansleep(hm2056->dvdd_en_gpio, 1);
		usleep_range(1000, 2000);
	}

	if (!IS_ERR(hm2056->pwdn_gpio)) {
		gpiod_set_value_cansleep(hm2056->pwdn_gpio, 1);
		usleep_range(2000, 5000);
	}

	if (!IS_ERR(hm2056->supplies)) {
		ret = regulator_bulk_enable(HM2056_NUM_SUPPLIES,
					    hm2056->supplies);
		if (ret < 0)
			dev_info(dev, "Failed to enable regulators\n");

		usleep_range(2000, 4000);
	}

	if (!IS_ERR(hm2056->pwdn_gpio)) {
		gpiod_set_value_cansleep(hm2056->pwdn_gpio, 0);
		usleep_range(1000, 2000);
	}

	if (!IS_ERR(hm2056->reset_gpio)) {
		gpiod_set_value_cansleep(hm2056->reset_gpio, 0);
		usleep_range(1000, 2000);
	}

	if (!IS_ERR(hm2056->xvclk)) {
		ret = clk_prepare_enable(hm2056->xvclk);
		if (ret < 0)
			dev_info(dev, "Failed to enable xvclk\n");
	}

	return 0;
}

static void __hm2056_power_off(struct hm2056 *hm2056)
{
	dev_dbg(&hm2056->client->dev, "%s(%d)\n", __func__, __LINE__);
	if (!IS_ERR(hm2056->xvclk))
		clk_disable_unprepare(hm2056->xvclk);
	if (!IS_ERR(hm2056->supplies))
		regulator_bulk_disable(HM2056_NUM_SUPPLIES, hm2056->supplies);
	if (!IS_ERR(hm2056->avdd_en_gpio))
		gpiod_set_value_cansleep(hm2056->avdd_en_gpio, 0);

	if (!IS_ERR(hm2056->dovdd_en_gpio)) {
		gpiod_set_value_cansleep(hm2056->dovdd_en_gpio, 0);
		usleep_range(100, 200);
	}
	if (!IS_ERR(hm2056->dvdd_en_gpio)) {
		gpiod_set_value_cansleep(hm2056->dvdd_en_gpio, 0);
		usleep_range(1000, 2000);
	}
	if (!IS_ERR(hm2056->pwdn_gpio))
		gpiod_set_value_cansleep(hm2056->pwdn_gpio, 1);
	if (!IS_ERR(hm2056->reset_gpio))
		gpiod_set_value_cansleep(hm2056->reset_gpio, 0);
}

static int hm2056_configure_regulators(struct hm2056 *hm2056)
{
	unsigned int i;

	for (i = 0; i < HM2056_NUM_SUPPLIES; i++)
		hm2056->supplies[i].supply = hm2056_supply_names[i];

	return devm_regulator_bulk_get(&hm2056->client->dev,
				       HM2056_NUM_SUPPLIES,
				       hm2056->supplies);
}

static int hm2056_parse_of(struct hm2056 *hm2056)
{
	struct device *dev = &hm2056->client->dev;
	struct device_node *endpoint;
	int ret;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint),
					 &hm2056->bus_cfg);
	if (ret) {
		dev_err(dev, "Failed to parse endpoint\n");
		of_node_put(endpoint);
		return ret;
	}
	if (hm2056->bus_cfg.bus_type == V4L2_MBUS_PARALLEL) {
		hm2056->framesize_cfg = hm2056_dvp_framesizes;
		hm2056->cfg_num = ARRAY_SIZE(hm2056_dvp_framesizes);
	}
	hm2056->avdd_en_gpio = devm_gpiod_get(dev, "avdd_en", GPIOD_OUT_LOW);
	if (IS_ERR(hm2056->avdd_en_gpio))
		dev_info(dev, "Failed to get avdd_en-gpios, maybe no use\n");
	hm2056->dvdd_en_gpio = devm_gpiod_get(dev, "dvdd_en", GPIOD_OUT_LOW);
	if (IS_ERR(hm2056->dvdd_en_gpio))
		dev_info(dev, "Failed to get dvdd_en-gpios, maybe no use\n");
	hm2056->dovdd_en_gpio = devm_gpiod_get(dev, "dovdd_en", GPIOD_OUT_LOW);
	if (IS_ERR(hm2056->dovdd_en_gpio))
		dev_info(dev, "Failed to get dovdd_en-gpios, maybe no use\n");

	hm2056->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(hm2056->pwdn_gpio))
		dev_info(dev, "Failed to get pwdn-gpios, maybe no use\n");
	hm2056->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(hm2056->reset_gpio))
		dev_info(dev, "Failed to get reset-gpios, maybe no use\n");

	ret = hm2056_configure_regulators(hm2056);
	if (ret)
		dev_info(dev, "Failed to get power regulators\n");

	return __hm2056_power_on(hm2056);
}

static int hm2056_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct v4l2_subdev *sd;
	struct hm2056 *hm2056;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	hm2056 = devm_kzalloc(&client->dev, sizeof(*hm2056), GFP_KERNEL);
	if (!hm2056)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &hm2056->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &hm2056->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &hm2056->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &hm2056->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	hm2056->client = client;
	hm2056->xvclk = devm_clk_get(&client->dev, "xvclk");
	if (IS_ERR(hm2056->xvclk)) {
		dev_err(&client->dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	ret = hm2056_parse_of(hm2056);
	if (ret != 0)
		return -EINVAL;

	hm2056->xvclk_frequency = clk_get_rate(hm2056->xvclk);
	if (hm2056->xvclk_frequency < 6000000 ||
	    hm2056->xvclk_frequency > 27000000)
		return -EINVAL;

	v4l2_ctrl_handler_init(&hm2056->ctrls, 3);
	hm2056->link_frequency =
			v4l2_ctrl_new_std(&hm2056->ctrls, &hm2056_ctrl_ops,
					  V4L2_CID_PIXEL_RATE, 0,
					  HM2056_PIXEL_RATE, 1,
					  HM2056_PIXEL_RATE);

	v4l2_ctrl_new_int_menu(&hm2056->ctrls, NULL, V4L2_CID_LINK_FREQ,
			       0, 0, link_freq_menu_items);
	v4l2_ctrl_new_std_menu_items(&hm2056->ctrls, &hm2056_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(hm2056_test_pattern_menu) - 1,
				     0, 0, hm2056_test_pattern_menu);
	hm2056->sd.ctrl_handler = &hm2056->ctrls;

	if (hm2056->ctrls.error) {
		dev_err(&client->dev, "%s: control initialization error %d\n",
			__func__, hm2056->ctrls.error);
		return  hm2056->ctrls.error;
	}

	sd = &hm2056->sd;
	client->flags |= I2C_CLIENT_SCCB;
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	v4l2_i2c_subdev_init(sd, client, &hm2056_subdev_ops);

	sd->internal_ops = &hm2056_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif

#if defined(CONFIG_MEDIA_CONTROLLER)
	hm2056->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &hm2056->pad, 0);
	if (ret < 0) {
		v4l2_ctrl_handler_free(&hm2056->ctrls);
		return ret;
	}
#endif

	mutex_init(&hm2056->lock);

	hm2056_get_default_format(hm2056, &hm2056->format);
	hm2056->frame_size = &hm2056->framesize_cfg[0];
	hm2056->format.width = hm2056->framesize_cfg[0].width;
	hm2056->format.height = hm2056->framesize_cfg[0].height;
	hm2056->fps = DIV_ROUND_CLOSEST(hm2056->framesize_cfg[0].max_fps.denominator,
			hm2056->framesize_cfg[0].max_fps.numerator);

	ret = __hm2056_power_on(hm2056);
	ret = hm2056_detect(hm2056);
	if (ret < 0)
		goto error;

	memset(facing, 0, sizeof(facing));
	if (strcmp(hm2056->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 hm2056->module_index, facing,
		 DRIVER_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret)
		goto error;

	dev_info(&client->dev, "%s sensor driver registered !!\n", sd->name);

	return 0;

error:
	v4l2_ctrl_handler_free(&hm2056->ctrls);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	mutex_destroy(&hm2056->lock);
	__hm2056_power_off(hm2056);
	return ret;
}

static int hm2056_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hm2056 *hm2056 = to_hm2056(sd);

	v4l2_ctrl_handler_free(&hm2056->ctrls);
	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	mutex_destroy(&hm2056->lock);

	__hm2056_power_off(hm2056);

	return 0;
}

static const struct i2c_device_id hm2056_id[] = {
	{ "hm2056", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, hm2056_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id hm2056_of_match[] = {
	{ .compatible = "himax,hm2056", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, hm2056_of_match);
#endif

static struct i2c_driver hm2056_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(hm2056_of_match),
	},
	.probe		= hm2056_probe,
	.remove		= hm2056_remove,
	.id_table	= hm2056_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&hm2056_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&hm2056_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("HM2056 CMOS Image Sensor driver");
MODULE_LICENSE("GPL v2");
