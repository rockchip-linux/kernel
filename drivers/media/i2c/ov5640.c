// SPDX-License-Identifier: GPL-2.0
/*
 * ov5640 driver
 * Copyright (C) 2019 Fuzhou Rockchip Electronics Co., Ltd.
 * v0.1.0x00 : 1. create file.
 * V0.0X01.0X02 fix mclk issue when probe multiple camera.
 * V0.0X01.0X03 add enum_frame_interval function.
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

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x03)
#define DRIVER_NAME "ov5640"
#define OV5640_PIXEL_RATE_FULL_SIZE		(192 * 1000 * 1000)
#define OV5640_PIXEL_RATE_VGA_SIZE		(96 * 1000 * 1000)
#define OV5640_XVCLK_FREQ		24000000

#define MIPI_FREQ	336000000LL
#define LINK_FREQ_168MHZ	112000000LL

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

/*
 * OV5640 register definitions
 */
#define REG_SC_CHIP_ID_H		0x300a
#define REG_SC_CHIP_ID_L		0x300b

#define REG_NULL			0xFFFF	/* Array end token */
#define DELAY_MS			0xEEEE	/* Array delay token */

#define SENSOR_ID(_msb, _lsb)		((_msb) << 8 | (_lsb))
#define OV5640_ID			0x5640

struct sensor_register {
	u16 addr;
	u8 value;
};

struct ov5640_framesize {
	u16 width;
	u16 height;
	u16 fps;
	struct v4l2_fract max_fps;
	u16 max_exp_lines;
	const struct sensor_register *regs;
};

struct ov5640_pll_ctrl {
	u8 ctrl1;
	u8 ctrl2;
	u8 ctrl3;
};

struct ov5640_pixfmt {
	u32 code;
	/* Output format Register Value (REG_FORMAT_CTRL00) */
	struct sensor_register *format_ctrl_regs;
};

struct pll_ctrl_reg {
	unsigned int div;
	unsigned char reg;
};

static const char * const ov5640_supply_names[] = {
	"dovdd",	/* Digital I/O power */
	"avdd",		/* Analog power */
	"dvdd",		/* Digital core power */
};

#define OV5640_NUM_SUPPLIES ARRAY_SIZE(ov5640_supply_names)

struct ov5640 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	unsigned int fps;
	unsigned int xvclk_frequency;
	struct clk *xvclk;
	struct gpio_desc *power_gpio;
	struct gpio_desc *pwdn_gpio;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[OV5640_NUM_SUPPLIES];
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct mutex lock;
	struct i2c_client *client;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_freq;
	struct v4l2_fwnode_endpoint bus_cfg;
	const struct ov5640_framesize *frame_size;
	const struct ov5640_framesize *framesize_cfg;
	unsigned int lane_num;
	unsigned int cfg_num;
	unsigned int pixel_rate;
	int streaming;
	u32 module_index;
	const char *module_facing;
	const char *module_name;
	const char *len_name;
};

static const struct sensor_register ov5640_dvp_init_regs[] = {
	{0x3103, 0x11},
	{0x3008, 0x82},
	{0x3008, 0x42},
	{0x3103, 0x03},
	{0x3017, 0xff},
	{0x3018, 0xff},
	{0x3034, 0x1a},
	{0x3035, 0x21},
	{0x3036, 0x46},
	{0x3037, 0x13},
	{0x3108, 0x01},
	{0x3630, 0x36},
	{0x3631, 0x0e},
	{0x3632, 0xe2},
	{0x3633, 0x12},
	{0x3621, 0xe0},
	{0x3704, 0xa0},
	{0x3703, 0x5a},
	{0x3715, 0x78},
	{0x3717, 0x01},
	{0x370b, 0x60},
	{0x3705, 0x1a},
	{0x3905, 0x02},
	{0x3906, 0x10},
	{0x3901, 0x0a},
	{0x3731, 0x12},
	{0x3600, 0x08},
	{0x3601, 0x33},
	{0x302d, 0x60},
	{0x3620, 0x52},
	{0x371b, 0x20},
	{0x471c, 0x50},
	{0x3a13, 0x43},
	{0x3a18, 0x00},
	{0x3a19, 0x78},
	{0x3635, 0x13},
	{0x3636, 0x03},
	{0x3634, 0x40},
	{0x3622, 0x01},
	{0x3c01, 0x34},
	{0x3c04, 0x28},
	{0x3c05, 0x98},
	{0x3c06, 0x00},
	{0x3c07, 0x08},
	{0x3c08, 0x00},
	{0x3c09, 0x1c},
	{0x3c0a, 0x9c},
	{0x3c0b, 0x40},
	{0x3820, 0x41},
	{0x3821, 0x07},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0x9b},
	{0x3808, 0x03},
	{0x3809, 0x20},
	{0x380a, 0x02},
	{0x380b, 0x58},
	{0x380c, 0x07},
	{0x380d, 0x68},
	{0x380e, 0x03},
	{0x380f, 0xd8},
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x06},
	{0x3618, 0x00},
	{0x3612, 0x29},
	{0x3708, 0x64},
	{0x3709, 0x52},
	{0x370c, 0x03},
	{0x3a02, 0x03},
	{0x3a03, 0xd8},
	{0x3a08, 0x01},
	{0x3a09, 0x27},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0e, 0x03},
	{0x3a0d, 0x04},
	{0x3a14, 0x03},
	{0x3a15, 0xd8},
	{0x4001, 0x02},
	{0x4004, 0x02},
	{0x3000, 0x00},
	{0x3002, 0x1c},
	{0x3004, 0xff},
	{0x3006, 0xc3},
	{0x300e, 0x58},
	{0x302e, 0x00},
	{0x4740, 0x20},
	{0x4300, 0x32},//uyvy:0x32,yuyv:0x30
	{0x501f, 0x00},
	{0x4713, 0x03},
	{0x4407, 0x04},
	{0x440e, 0x00},
	{0x460b, 0x35},
	{0x460c, 0x20},
	{0x4837, 0x22},
	{0x3824, 0x02},
	{0x5000, 0xa7},
	{0x5001, 0xa3},
	{0x5180, 0xff},
	{0x5181, 0xf2},
	{0x5182, 0x00},
	{0x5183, 0x14},
	{0x5184, 0x25},
	{0x5185, 0x24},
	{0x5186, 0x0f},
	{0x5187, 0x0f},
	{0x5188, 0x0f},
	{0x5189, 0x80},
	{0x518a, 0x5d},
	{0x518b, 0xe3},
	{0x518c, 0xa7},
	{0x518d, 0x40},
	{0x518e, 0x33},
	{0x518f, 0x5e},
	{0x5190, 0x4e},
	{0x5191, 0xf8},
	{0x5192, 0x04},
	{0x5193, 0x70},
	{0x5194, 0xf0},
	{0x5195, 0xf0},
	{0x5196, 0x03},
	{0x5197, 0x01},
	{0x5198, 0x06},
	{0x5199, 0xd0},
	{0x519a, 0x04},
	{0x519b, 0x00},
	{0x519c, 0x04},
	{0x519d, 0x87},
	{0x519e, 0x38},
	{0x5381, 0x27},
	{0x5382, 0x41},
	{0x5383, 0x18},
	{0x5384, 0x0d},
	{0x5385, 0x59},
	{0x5386, 0x66},
	{0x5387, 0x63},
	{0x5388, 0x55},
	{0x5389, 0x0f},
	{0x538a, 0x01},
	{0x538b, 0x98},
	{0x5300, 0x08},
	{0x5301, 0x30},
	{0x5302, 0x10},
	{0x5303, 0x00},
	{0x5304, 0x08},
	{0x5305, 0x30},
	{0x5306, 0x08},
	{0x5307, 0x16},
	{0x5309, 0x08},
	{0x530a, 0x30},
	{0x530b, 0x04},
	{0x530c, 0x06},
	{0x5480, 0x01},
	{0x5481, 0x08},
	{0x5482, 0x14},
	{0x5483, 0x28},
	{0x5484, 0x51},
	{0x5485, 0x65},
	{0x5486, 0x71},
	{0x5487, 0x7d},
	{0x5488, 0x87},
	{0x5489, 0x91},
	{0x548a, 0x9a},
	{0x548b, 0xaa},
	{0x548c, 0xb8},
	{0x548d, 0xcd},
	{0x548e, 0xdd},
	{0x548f, 0xea},
	{0x5490, 0x1d},

	{0x5580, 0x02},
	{0x5583, 0x40},
	{0x5584, 0x10},
	{0x5589, 0x10},
	{0x558a, 0x00},
	{0x558b, 0xf8},
	{0x5800, 0x23},
	{0x5801, 0x14},
	{0x5802, 0x0f},
	{0x5803, 0x0f},
	{0x5804, 0x12},
	{0x5805, 0x26},
	{0x5806, 0x0c},
	{0x5807, 0x08},
	{0x5808, 0x05},
	{0x5809, 0x05},
	{0x580a, 0x08},
	{0x580b, 0x0d},
	{0x580c, 0x08},
	{0x580d, 0x03},
	{0x580e, 0x00},
	{0x580f, 0x00},
	{0x5810, 0x03},
	{0x5811, 0x09},
	{0x5812, 0x07},
	{0x5813, 0x03},
	{0x5814, 0x00},
	{0x5815, 0x01},
	{0x5816, 0x03},
	{0x5817, 0x08},
	{0x5818, 0x0d},
	{0x5819, 0x08},
	{0x581a, 0x05},
	{0x581b, 0x06},
	{0x581c, 0x08},
	{0x581d, 0x0e},
	{0x581e, 0x29},
	{0x581f, 0x17},
	{0x5820, 0x11},
	{0x5821, 0x11},
	{0x5822, 0x15},
	{0x5823, 0x28},
	{0x5824, 0x46},
	{0x5825, 0x26},
	{0x5826, 0x08},
	{0x5827, 0x26},
	{0x5828, 0x64},
	{0x5829, 0x26},
	{0x582a, 0x24},
	{0x582b, 0x22},
	{0x582c, 0x24},
	{0x582d, 0x24},
	{0x582e, 0x06},
	{0x582f, 0x22},
	{0x5830, 0x40},
	{0x5831, 0x42},
	{0x5832, 0x24},
	{0x5833, 0x26},
	{0x5834, 0x24},
	{0x5835, 0x22},
	{0x5836, 0x22},
	{0x5837, 0x26},
	{0x5838, 0x44},
	{0x5839, 0x24},
	{0x583a, 0x26},
	{0x583b, 0x28},
	{0x583c, 0x42},
	{0x583d, 0xce},
	{0x5025, 0x00},
	{0x3a0f, 0x30},
	{0x3a10, 0x28},
	{0x3a1b, 0x30},
	{0x3a1e, 0x26},
	{0x3a11, 0x60},
	{0x3a1f, 0x14},
	{0x3008, 0x02},

	{0x3503, 0x00},
	{0x3c07, 0x08},
	{0x3820, 0x41},
	{0x3821, 0x07},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3803, 0x04},
	{0x3806, 0x07},
	{0x3807, 0x9b},
	{0x3808, 0x03},
	{0x3809, 0x20},
	{0x380a, 0x02},
	{0x380b, 0x58},
	{0x380c, 0x07},
	{0x380d, 0x68},
	{0x380e, 0x04},//0x03
	{0x380f, 0x0a},//d8
	{0x3813, 0x06},
	{0x3618, 0x00},
	{0x3612, 0x29},
	{0x3709, 0x52},
	{0x370c, 0x03},
	{0x3a02, 0x03},
	{0x3a03, 0xd8},
	{0x3a08, 0x00},//01
	{0x3a09, 0x94},//27
	{0x3a0a, 0x00},
	{0x3a0b, 0x7b},//f6
	{0x3a0e, 0x06},//03
	{0x3a0d, 0x07},//04
	{0x3a14, 0x03},
	{0x3a15, 0xd8},
	{0x4004, 0x02},
	{0x3002, 0x1c},
	{0x4713, 0x03},
	{0x3035, 0x21},
	{0x3036, 0x46},
	{0x4837, 0x22},
	{0x3824, 0x02},
	{0x5001, 0xa3},

	{0x4005, 0x1a},
	{REG_NULL, 0x00},
};

/* Senor full resolution setting */
static const struct sensor_register ov5640_dvp_full[] = {
	{0x3a00, 0x78},
	{0x3820, 0x40},
	{0x3821, 0x06},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3803, 0x00},
	{0x3807, 0x9f},
	{0x3808, 0x0a},
	{0x3809, 0x20},
	{0x380a, 0x07},
	{0x380b, 0x98},
	{0x380c, 0x0b},
	{0x380d, 0x1c},
	{0x380e, 0x07},
	{0x380f, 0xb0},
	{0x3811, 0x10}, //
	{0x3813, 0x04},
	{0x3618, 0x04},
	{0x3612, 0x2b}, //4b
	{0x3708, 0x64},
	{0x3709, 0x12},
	{0x370c, 0x00},
	{0x3a02, 0x07},
	{0x3a03, 0xb0},
	{0x3a0e, 0x06},
	{0x3a0d, 0x08},
	{0x3a14, 0x07},
	{0x3a15, 0xb0},
	{0x4004, 0x06},
	{0x5000, 0xa7},
	{0x5001, 0x83},
	{0x519e, 0x38},
	{0x5381, 0x1e},
	{0x5382, 0x5b},
	{0x5383, 0x08},
	{0x460b, 0x37},
	{0x460c, 0x20},
	{0x3824, 0x01},
	{0x4005, 0x1A},
	{REG_NULL, 0x00},
};

/* Preview resolution setting*/
static const struct sensor_register ov5640_dvp_svga_30fps[] = {
	{0x3503, 0x00},
	{0x3a00, 0x78},
	{0x3c07, 0x08},
	{0x3820, 0x41},
	{0x3821, 0x07},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3803, 0x04},
	{0x3806, 0x07},///
	{0x3807, 0x9b},
	{0x3808, 0x03},
	{0x3809, 0x20},
	{0x380a, 0x02},
	{0x380b, 0x58},
	{0x380c, 0x07},
	{0x380d, 0x68},
	{0x380e, 0x04},//0x03
	{0x380f, 0x0a},//d8
	{0x3813, 0x06},
	{0x3618, 0x00},
	{0x3612, 0x29},
	{0x3709, 0x52},
	{0x370c, 0x03},
	{0x3a02, 0x08},
	{0x3a03, 0x14},
	{0x3a08, 0x00},//01
	{0x3a09, 0x94},//27
	{0x3a0a, 0x00},
	{0x3a0b, 0x7b},//f6
	{0x3a0e, 0x06},//03
	{0x3a0d, 0x07},//04
	{0x3a14, 0x08},
	{0x3a15, 0x14},
	{0x4004, 0x02},
	{0x3002, 0x1c},////
	{0x4713, 0x03},////
	{0x3035, 0x21},
	{0x3036, 0x46},
	{0x4837, 0x22},
	{0x3824, 0x02},////
	{0x5001, 0xa3},
	{REG_NULL, 0x00},
};

static const struct sensor_register ov5640_mipi_init_regs[] = {
	{0x3103, 0x11},
	{0x3008, 0x82},
	{0x3008, 0x42},//stop stream
	{0x3103, 0x03},
	{0x3017, 0x00},
	{0x3018, 0x00},
	{0x3034, 0x18},
	{0x3035, 0x11},
	{0x3036, 0x54},
	{0x3037, 0x13},
	{0x3108, 0x01},
	{0x3630, 0x36},
	{0x3631, 0x0e},
	{0x3632, 0xe2},
	{0x3633, 0x12},
	{0x3621, 0xe0},
	{0x3704, 0xa0},
	{0x3703, 0x5a},
	{0x3715, 0x78},
	{0x3717, 0x01},
	{0x370b, 0x60},
	{0x3705, 0x1a},
	{0x3905, 0x02},
	{0x3906, 0x10},
	{0x3901, 0x0a},
	{0x3731, 0x12},
	{0x3600, 0x08},
	{0x3601, 0x33},
	{0x302d, 0x60},
	{0x3620, 0x52},
	{0x371b, 0x20},
	{0x471c, 0x50},
	{0x3a13, 0x43},
	{0x3a18, 0x00},
	{0x3a19, 0xf8},
	{0x3635, 0x13},
	{0x3636, 0x03},
	{0x3634, 0x40},
	{0x3622, 0x01},
	{0x3c01, 0x34},
	{0x3c04, 0x28},
	{0x3c05, 0x98},
	{0x3c06, 0x00},
	{0x3c07, 0x07},
	{0x3c08, 0x00},
	{0x3c09, 0x1c},
	{0x3c0a, 0x9c},
	{0x3c0b, 0x40},
	{0x3820, 0x40},
	{0x3821, 0x06},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0x9f},
	{0x3808, 0x0a},//2592
	{0x3809, 0x20},
	{0x380a, 0x07},//1944
	{0x380b, 0x98},

	{0x380c, 0x0b},
	{0x380d, 0x1c},

	{0x380e, 0x07},//15fps
	{0x380f, 0xb4},

	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3618, 0x04},
	{0x3612, 0x2b},
	{0x3708, 0x63},
	{0x3709, 0x12},
	{0x370c, 0x00},
	{0x3a08, 0x01},
	{0x3a09, 0x27},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0e, 0x06},
	{0x3a0d, 0x08},

	{0x4001, 0x02},
	{0x4004, 0x06},
	{0x4050, 0x6e},
	{0x4051, 0x8f},
	{0x3000, 0x00},
	{0x3002, 0x1c},
	{0x3004, 0xff},
	{0x3006, 0xc3},
	{0x300e, 0x45},
	{0x302e, 0x08},
	{0x4300, 0x32},//uyvy:0x32,yuyv:0x30
	{0x4837, 0x0a},
	{0x501f, 0x00},
	{0x5684, 0x0a},
	{0x5685, 0x20},
	{0x5686, 0x07},
	{0x5687, 0x98},
	{0x440e, 0x00},
	{0x5000, 0xa7},
	{0x5001, 0xa3},

	//OV AWB
	{0x5180, 0xff},
	{0x5181, 0xf2},
	{0x5182, 0x00},
	{0x5183, 0x14},
	{0x5184, 0x25},
	{0x5185, 0x24},
	{0x5186, 0x09},
	{0x5187, 0x09},
	{0x5188, 0x0a},
	{0x5189, 0x75},
	{0x518a, 0x60},
	{0x518b, 0xd0},
	{0x518c, 0xb0},
	{0x518d, 0x42},
	{0x518e, 0x3d},
	{0x518f, 0x56},
	{0x5190, 0x49},
	{0x5191, 0xf8},
	{0x5192, 0x04},
	{0x5193, 0x70},
	{0x5194, 0xf0},
	{0x5195, 0xf0},
	{0x5196, 0x03},
	{0x5197, 0x01},
	{0x5198, 0x05},
	{0x5199, 0x5d},
	{0x519a, 0x04},
	{0x519b, 0x00},
	{0x519c, 0x07},
	{0x519d, 0x58},
	{0x519e, 0x38},
	//OV AWB

	{0x5381, 0x1e},
	{0x5382, 0x5b},
	{0x5383, 0x08},
	{0x5384, 0x0a},
	{0x5385, 0x7e},
	{0x5386, 0x88},
	{0x5387, 0x7c},
	{0x5388, 0x6c},
	{0x5389, 0x10},
	{0x538a, 0x01},
	{0x538b, 0x98},
	{0x5300, 0x08},
	{0x5301, 0x30},
	{0x5302, 0x10},
	{0x5303, 0x00},
	{0x5304, 0x08},
	{0x5305, 0x30},
	{0x5306, 0x08},
	{0x5307, 0x16},
	{0x5309, 0x08},
	{0x530a, 0x30},
	{0x530b, 0x04},
	{0x530c, 0x06},
	{0x5480, 0x01},
	{0x5481, 0x08},
	{0x5482, 0x14},
	{0x5483, 0x28},
	{0x5484, 0x51},
	{0x5485, 0x65},
	{0x5486, 0x71},
	{0x5487, 0x7d},
	{0x5488, 0x87},
	{0x5489, 0x91},
	{0x548a, 0x9a},
	{0x548b, 0xaa},
	{0x548c, 0xb8},
	{0x548d, 0xcd},
	{0x548e, 0xdd},
	{0x548f, 0xea},
	{0x5490, 0x1d},
	{0x5580, 0x02},
	{0x5583, 0x40},
	{0x5584, 0x10},
	{0x5589, 0x10},
	{0x558a, 0x00},
	{0x558b, 0xf8},
	{0x5800, 0x23},
	{0x5801, 0x14},
	{0x5802, 0x0f},
	{0x5803, 0x0f},
	{0x5804, 0x12},
	{0x5805, 0x26},
	{0x5806, 0x0c},
	{0x5807, 0x08},
	{0x5808, 0x05},
	{0x5809, 0x05},
	{0x580a, 0x08},
	{0x580b, 0x0d},
	{0x580c, 0x08},
	{0x580d, 0x03},
	{0x580e, 0x00},
	{0x580f, 0x00},
	{0x5810, 0x03},
	{0x5811, 0x09},
	{0x5812, 0x07},
	{0x5813, 0x03},
	{0x5814, 0x00},
	{0x5815, 0x01},
	{0x5816, 0x03},
	{0x5817, 0x08},
	{0x5818, 0x0d},
	{0x5819, 0x08},
	{0x581a, 0x05},
	{0x581b, 0x06},
	{0x581c, 0x08},
	{0x581d, 0x0e},
	{0x581e, 0x29},
	{0x581f, 0x17},
	{0x5820, 0x11},
	{0x5821, 0x11},
	{0x5822, 0x15},
	{0x5823, 0x28},
	{0x5824, 0x46},
	{0x5825, 0x26},
	{0x5826, 0x08},
	{0x5827, 0x26},
	{0x5828, 0x64},
	{0x5829, 0x26},
	{0x582a, 0x24},
	{0x582b, 0x22},
	{0x582c, 0x24},
	{0x582d, 0x24},
	{0x582e, 0x06},
	{0x582f, 0x22},
	{0x5830, 0x40},
	{0x5831, 0x42},
	{0x5832, 0x24},
	{0x5833, 0x26},
	{0x5834, 0x24},
	{0x5835, 0x22},
	{0x5836, 0x22},
	{0x5837, 0x26},
	{0x5838, 0x44},
	{0x5839, 0x24},
	{0x583a, 0x26},
	{0x583b, 0x28},
	{0x583c, 0x42},
	{0x583d, 0xce},
	{0x5025, 0x00},
	{0x3a0f, 0x30},
	{0x3a10, 0x28},
	{0x3a1b, 0x30},
	{0x3a1e, 0x26},
	{0x3a11, 0x60},
	{0x3a1f, 0x14},
	//{0x3008, 0x02},//start stream

	//EV 0
	{0x3a0f, 0x38},
	{0x3a10, 0x30},
	{0x3a11, 0x61},
	{0x3a1b, 0x38},
	{0x3a1e, 0x30},
	{0x3a1f, 0x10},

	//AEC/AGC detect window
	//03,02,01,00
	//13,12,11,10
	//23,22,21,20
	//33,32,31,30
	//bit 7~4 , 3~0
	{0x5688, 0x00}, //Window 01 , 00
	{0x5689, 0x00}, //Window 03 , 02
	{0x568a, 0xf0}, //Window 11 , 10
	{0x568b, 0x0f}, //Window 13 , 12
	{0x568c, 0xf0}, //Window 21 , 20
	{0x568d, 0x0f}, //Window 23 , 22
	{0x568e, 0x00}, //Window 31 , 30
	{0x568f, 0x00}, //Window 33 , 32

	//auto night mode for 60/50Hz light environment,
	//24Mhz clock input,24Mzh pclk
	{0x3a00, 0x7c},

	// ~ 10fps
	{0x3a02, 0x0b}, //60HZ max exposure limit MSB
	{0x3a03, 0x8e}, //60HZ max exposure limit LSB
	{0x3a14, 0x0b}, //50HZ max exposure limit MSB
	{0x3a15, 0x8e}, //50HZ max exposure limit LSB

	//Saturation -1
	{0x5001, 0xff},
	{0x5583, 0x30},
	{0x5584, 0x30},
	{0x5580, 0x02},
	{0x5588, 0x41},

	{0x3008, 0x42},//stop stream
	{REG_NULL, 0x00},
};

static const struct sensor_register ov5640_mipi_640x480_30fps[] = {
	{0x3035, 0x14},
	{0x3036, 0x38},
	{0x3c07, 0x08},
	{0x3c09, 0x1c},
	{0x3c0a, 0x9c},
	{0x3c0b, 0x40},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0x9b},

	{0x3808, 0x02},
	{0x3809, 0x80},
	{0x380a, 0x01},
	{0x380b, 0xe0},
	{0x380c, 0x07},
	{0x380d, 0x68},

	{0x380e, 0x03},//30fps
	{0x380f, 0xd8},

	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x06},
	{0x3618, 0x00},
	{0x3612, 0x29},
	{0x3708, 0x64},
	{0x3709, 0x52},
	{0x370c, 0x03},
	{0x3a02, 0x03},
	{0x3a03, 0xd8},
	{0x3a08, 0x01},
	{0x3a09, 0x0e},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0e, 0x03},
	{0x3a0d, 0x04},
	{0x3a14, 0x03},
	{0x3a15, 0xd8},
	{0x4001, 0x02},
	{0x4004, 0x02},
	{0x4713, 0x03},
	{0x4407, 0x04},
	{0x460b, 0x35},
	{0x460c, 0x22},
	{0x3824, 0x02},
	{0x5001, 0xa3},
	{0x3503, 0x00},

	{0x3008, 0x42}, //stop stream
	//{0x3c00, 0x04},
	{REG_NULL, 0x00},
};

static const struct sensor_register ov5640_mipi_1080P_30fps[] = {
	{0x3035, 0x14},
	{0x3036, 0x38},
	{0x3c07, 0x08},
	{0x3c09, 0x1c},
	{0x3c0a, 0x9c},
	{0x3c0b, 0x40},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0x9b},

	{0x3808, 0x07},
	{0x3809, 0x80},
	{0x380a, 0x04},
	{0x380b, 0x38},
	{0x380c, 0x09},
	{0x380d, 0xc4},

	{0x380e, 0x04},//30fps
	{0x380f, 0x60},

	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x06},
	{0x3618, 0x00},
	{0x3612, 0x29},
	{0x3708, 0x64},
	{0x3709, 0x52},
	{0x370c, 0x03},
	{0x3a02, 0x03},
	{0x3a03, 0xd8},
	{0x3a08, 0x01},
	{0x3a09, 0x0e},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0e, 0x03},
	{0x3a0d, 0x04},
	{0x3a14, 0x03},
	{0x3a15, 0xd8},
	{0x4001, 0x02},
	{0x4004, 0x02},
	{0x4713, 0x03},
	{0x4407, 0x04},
	{0x460b, 0x35},
	{0x460c, 0x22},
	{0x3824, 0x02},
	{0x5001, 0xa3},
	{0x3503, 0x00},

	{0x3008, 0x42}, //stop stream
	//{0x3c00, 0x04},
	{REG_NULL, 0x00},
};

static const struct sensor_register ov5640_mipi_full[] = {
//MIPI_2lane_5M(YUV) 15fps
	{0x3035, 0x21},
	{0x3036, 0x54},
	{0x3c07, 0x08},
	{0x3c09, 0x1c},
	{0x3c0a, 0x9c},
	{0x3c0b, 0x40},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x0a},
	{0x3805, 0x3f},
	{0x3806, 0x07},
	{0x3807, 0x9f},

	{0x3808, 0x0a},//2592
	{0x3809, 0x20},
	{0x380a, 0x07},//1944
	{0x380b, 0x98},

	{0x380c, 0x0b},
	{0x380d, 0x1c},

	{0x380e, 0x07},//15fps
	{0x380f, 0xb4},

	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3618, 0x04},
	{0x3612, 0x29},
	{0x3708, 0x21},
	{0x3709, 0x12},
	{0x370c, 0x00},
	{0x3a02, 0x03},
	{0x3a03, 0xd8},
	{0x3a08, 0x01},
	{0x3a09, 0x27},
	{0x3a0a, 0x00},
	{0x3a0b, 0xf6},
	{0x3a0e, 0x03},
	{0x3a0d, 0x04},
	{0x3a14, 0x03},
	{0x3a15, 0xd8},
	{0x4001, 0x02},
	{0x4004, 0x06},
	{0x4713, 0x03},
	{0x4407, 0x04},
	{0x460b, 0x35},
	{0x460c, 0x22},
	{0x3824, 0x02},
	{0x5001, 0x83},
	//{DELAY_MS, 70},
	// ~ 10fps
	{0x3a02, 0x0b}, //60HZ max exposure limit MSB
	{0x3a03, 0x8e}, //60HZ max exposure limit LSB
	{0x3a14, 0x0b}, //50HZ max exposure limit MSB
	{0x3a15, 0x8e}, //50HZ max exposure limit LSB
	{REG_NULL, 0x00},
};

static const struct ov5640_framesize ov5640_dvp_framesizes[] = {
	{ /* SVGA */
		.width		= 800,
		.height		= 600,
		.fps		= 30,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.regs		= ov5640_dvp_svga_30fps,
	}, { /* FULL */
		.width		= 2592,
		.height		= 1944,
		.fps		= 15,
		.max_fps = {
			.numerator = 10000,
			.denominator = 150000,
		},
		.regs		= ov5640_dvp_full,
	}
};

static const struct ov5640_framesize ov5640_mipi_framesizes[] = {
	{ /* 1080P */
		.width		= 1920,
		.height		= 1080,
		.fps		= 30,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.regs		= ov5640_mipi_1080P_30fps,
	},
	{ /* VGA */
		.width		= 640,
		.height		= 480,
		.fps		= 30,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.regs		= ov5640_mipi_640x480_30fps,
	}, { /* FULL */
		.width		= 2592,
		.height		= 1944,
		.fps		= 15,
		.max_fps = {
			.numerator = 10000,
			.denominator = 150000,
		},
		.regs		= ov5640_mipi_full,
	}
};

static const s64 link_freq_menu_items[] = {
	MIPI_FREQ,
	LINK_FREQ_168MHZ
};

static const struct ov5640_pixfmt ov5640_formats[] = {
	{
		.code = MEDIA_BUS_FMT_UYVY8_2X8,
	}
};

static inline struct ov5640 *to_ov5640(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov5640, sd);
}

/* sensor register write */
static int ov5640_write(struct i2c_client *client, u16 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	dev_dbg(&client->dev, "write reg(0x%x val:0x%x)!\n", reg, val);

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = val;

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

/* sensor register read */
static int ov5640_read(struct i2c_client *client, u16 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;

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
		"ov5640 read reg:0x%x failed !\n", reg);

	return ret;
}

static int ov5640_write_array(struct i2c_client *client,
			      const struct sensor_register *regs)
{
	int i, delay_ms, ret = 0;

	i = 0;
	while (regs[i].addr != REG_NULL) {
		if (regs[i].addr == DELAY_MS) {
			delay_ms = regs[i].value;
			dev_info(&client->dev, "delay(%d) ms !\n", delay_ms);
			usleep_range(1000 * delay_ms, 1000 * delay_ms + 100);
			i++;
			continue;
		}
		ret = ov5640_write(client, regs[i].addr, regs[i].value);
		if (ret) {
			dev_err(&client->dev, "%s failed !\n", __func__);
			break;
		}
		i++;
	}

	return ret;
}

static void ov5640_get_default_format(struct ov5640 *ov5640,
				      struct v4l2_mbus_framefmt *format)
{
	format->width = ov5640->framesize_cfg[0].width;
	format->height = ov5640->framesize_cfg[0].height;
	format->colorspace = V4L2_COLORSPACE_SRGB;
	format->code = ov5640_formats[0].code;
	format->field = V4L2_FIELD_NONE;
}

static void ov5640_set_streaming(struct ov5640 *ov5640, int on)
{
	struct i2c_client *client = ov5640->client;
	int ret = 0;
	u8 val;

	dev_info(&client->dev, "%s: on: %d\n", __func__, on);
	val = on ? 0x02 : 0x42;
	ret = ov5640_write(client, 0x3008, val);
	if (ret)
		dev_err(&client->dev, "ov5640 soft standby failed\n");
}

/*
 * V4L2 subdev video and pad level operations
 */

static int ov5640_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	if (code->index >= ARRAY_SIZE(ov5640_formats))
		return -EINVAL;

	code->code = ov5640_formats[code->index].code;

	return 0;
}

static int ov5640_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct ov5640 *ov5640 = to_ov5640(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int i = ARRAY_SIZE(ov5640_formats);

	dev_dbg(&client->dev, "%s:\n", __func__);

	if (fse->index >= ov5640->cfg_num)
		return -EINVAL;

	while (--i)
		if (fse->code == ov5640_formats[i].code)
			break;

	fse->code = ov5640_formats[i].code;

	fse->min_width  = ov5640->framesize_cfg[fse->index].width;
	fse->max_width  = fse->min_width;
	fse->max_height = ov5640->framesize_cfg[fse->index].height;
	fse->min_height = fse->max_height;

	return 0;
}

static int ov5640_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *ov5640 = to_ov5640(sd);

	dev_dbg(&client->dev, "%s enter\n", __func__);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		struct v4l2_mbus_framefmt *mf;

		mf = v4l2_subdev_get_try_format(sd, cfg, 0);
		mutex_lock(&ov5640->lock);
		fmt->format = *mf;
		mutex_unlock(&ov5640->lock);
		return 0;
#else
	return -ENOTTY;
#endif
	}

	mutex_lock(&ov5640->lock);
	fmt->format = ov5640->format;
	mutex_unlock(&ov5640->lock);

	dev_dbg(&client->dev, "%s: %x %dx%d\n", __func__,
		ov5640->format.code, ov5640->format.width,
		ov5640->format.height);

	return 0;
}

static void __ov5640_try_frame_size_fps(struct ov5640 *ov5640,
					struct v4l2_mbus_framefmt *mf,
					const struct ov5640_framesize **size,
					unsigned int fps)
{
	const struct ov5640_framesize *fsize = &ov5640->framesize_cfg[0];
	const struct ov5640_framesize *match = NULL;
	unsigned int i = ov5640->cfg_num;
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
		match = &ov5640->framesize_cfg[0];
	} else {
		fsize = &ov5640->framesize_cfg[0];
		for (i = 0; i < ov5640->cfg_num; i++) {
			if (fsize->width == match->width &&
			    fsize->height == match->height &&
			    fps >= fsize->fps)
				match = fsize;

			fsize++;
		}
	}

	mf->width  = match->width;
	mf->height = match->height;

	if (size)
		*size = match;
}

static int ov5640_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int index = ARRAY_SIZE(ov5640_formats);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	const struct ov5640_framesize *size = NULL;
	struct ov5640 *ov5640 = to_ov5640(sd);
	int ret = 0;

	dev_dbg(&client->dev, "%s enter\n", __func__);

	__ov5640_try_frame_size_fps(ov5640, mf, &size, ov5640->fps);

	while (--index >= 0)
		if (ov5640_formats[index].code == mf->code)
			break;

	if (index < 0)
		return -EINVAL;

	mf->colorspace = V4L2_COLORSPACE_SRGB;
	mf->code = ov5640_formats[index].code;
	mf->field = V4L2_FIELD_NONE;

	mutex_lock(&ov5640->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		*mf = fmt->format;
#else
		return -ENOTTY;
#endif
	} else {
		if (ov5640->streaming) {
			mutex_unlock(&ov5640->lock);
			return -EBUSY;
		}

		ov5640->frame_size = size;
		ov5640->format = fmt->format;
		if (mf->width == 2592 || mf->width == 1920) {
			__v4l2_ctrl_s_ctrl(ov5640->link_freq,
				link_freq_menu_items[0]);
			__v4l2_ctrl_s_ctrl_int64(ov5640->pixel_freq,
				OV5640_PIXEL_RATE_FULL_SIZE);
		} else {
			__v4l2_ctrl_s_ctrl(ov5640->link_freq,
				link_freq_menu_items[1]);
			__v4l2_ctrl_s_ctrl_int64(ov5640->pixel_freq,
				OV5640_PIXEL_RATE_VGA_SIZE);
		}
	}

	mutex_unlock(&ov5640->lock);
	return ret;
}

static int ov5640_s_stream(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *ov5640 = to_ov5640(sd);
	int ret = 0;

	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
		 ov5640->frame_size->width,
		 ov5640->frame_size->height,
		 ov5640->frame_size->fps);

	mutex_lock(&ov5640->lock);

	on = !!on;

	if (ov5640->streaming == on)
		goto unlock;

	if (!on) {
		/* Stop Streaming Sequence */
		ov5640_set_streaming(ov5640, on);
		ov5640->streaming = on;
		goto unlock;
	}

	ret = ov5640_write_array(client, ov5640->frame_size->regs);
	if (ret)
		goto unlock;

	ov5640_set_streaming(ov5640, on);
	ov5640->streaming = on;

unlock:
	mutex_unlock(&ov5640->lock);
	return ret;
}

static int ov5640_set_test_pattern(struct ov5640 *ov5640, int value)
{
	return 0;
}

static int ov5640_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov5640 *ov5640 =
			container_of(ctrl->handler, struct ov5640, ctrls);

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		return ov5640_set_test_pattern(ov5640, ctrl->val);
	}

	return 0;
}

static const struct v4l2_ctrl_ops ov5640_ctrl_ops = {
	.s_ctrl = ov5640_s_ctrl,
};

static const char * const ov5640_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bars",
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev internal operations
 */

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ov5640_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ov5640 *ov5640 = to_ov5640(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);

	dev_dbg(&client->dev, "%s:\n", __func__);

	ov5640_get_default_format(ov5640, format);

	return 0;
}
#endif

static int ov5640_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	struct ov5640 *ov5640 = to_ov5640(sd);

	if (ov5640->bus_cfg.bus_type == V4L2_MBUS_CSI2) {
		config->type = V4L2_MBUS_CSI2;
		config->flags = V4L2_MBUS_CSI2_2_LANE |
						V4L2_MBUS_CSI2_CHANNEL_0 |
						V4L2_MBUS_CSI2_CHANNEL_1 |
						V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	} else {
		config->type = V4L2_MBUS_PARALLEL;
		config->flags = V4L2_MBUS_HSYNC_ACTIVE_HIGH |
				V4L2_MBUS_VSYNC_ACTIVE_LOW |
				V4L2_MBUS_PCLK_SAMPLE_RISING;
	}

	return 0;
}

static int ov5640_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ov5640 *ov5640 = to_ov5640(sd);

	mutex_lock(&ov5640->lock);
	fi->interval = ov5640->frame_size->max_fps;
	mutex_unlock(&ov5640->lock);

	return 0;
}

static int ov5640_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *ov5640 = to_ov5640(sd);
	const struct ov5640_framesize *size = NULL;
	struct v4l2_mbus_framefmt mf;
	unsigned int fps;
	int ret = 0;

	dev_dbg(&client->dev, "Setting %d/%d frame interval\n",
		fi->interval.numerator, fi->interval.denominator);

	mutex_lock(&ov5640->lock);

	if (ov5640->format.width == 1600)
		goto unlock;

	fps = DIV_ROUND_CLOSEST(fi->interval.denominator,
				fi->interval.numerator);
	mf = ov5640->format;
	__ov5640_try_frame_size_fps(ov5640, &mf, &size, fps);

	if (ov5640->frame_size != size) {
		dev_info(&client->dev, "%s match wxh@FPS is %dx%d@%d\n",
			 __func__, size->width, size->height, size->fps);
		ret = ov5640_write_array(client, size->regs);
		if (ret)
			goto unlock;
		ov5640->frame_size = size;
		ov5640->fps = fps;
	}
unlock:
	mutex_unlock(&ov5640->lock);

	return ret;
}

static void ov5640_get_module_inf(struct ov5640 *ov5640,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, DRIVER_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, ov5640->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, ov5640->len_name, sizeof(inf->base.lens));
}

static long ov5640_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ov5640 *ov5640 = to_ov5640(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		ov5640_get_module_inf(ov5640, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ov5640_compat_ioctl32(struct v4l2_subdev *sd,
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

		ret = ov5640_ioctl(sd, cmd, inf);
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
			ret = ov5640_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int ov5640_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct ov5640 *ov5640 = to_ov5640(sd);
	struct i2c_client *client = ov5640->client;

	dev_info(&client->dev, "%s(%d)\n", __func__, __LINE__);

	if (ov5640->bus_cfg.bus_type == V4L2_MBUS_CSI2)
		ret = ov5640_write_array(client, ov5640_mipi_init_regs);
	else
		ret = ov5640_write_array(client, ov5640_dvp_init_regs);

	return ret;
}

static int ov5640_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	struct ov5640 *ov5640 = to_ov5640(sd);
	struct i2c_client *client = ov5640->client;
	struct device *dev = &ov5640->client->dev;

	dev_info(&client->dev, "%s(%d) on(%d)\n", __func__, __LINE__, on);
	if (on) {
		if (!IS_ERR(ov5640->pwdn_gpio)) {
			gpiod_set_value_cansleep(ov5640->pwdn_gpio, 0);
			usleep_range(2000, 5000);
		}
		ret = ov5640_init(sd, 0);
		usleep_range(10000, 20000);
		if (ret)
			dev_err(dev, "init error\n");
	} else {
		if (!IS_ERR(ov5640->pwdn_gpio)) {
			gpiod_set_value_cansleep(ov5640->pwdn_gpio, 1);
			usleep_range(2000, 5000);
		}
	}
	return 0;
}

static int ov5640_enum_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_interval_enum *fie)
{
	struct ov5640 *ov5640 = to_ov5640(sd);

	if (fie->index >= ov5640->cfg_num)
		return -EINVAL;

	if (fie->code != MEDIA_BUS_FMT_UYVY8_2X8)
		return -EINVAL;

	fie->width = ov5640->framesize_cfg[fie->index].width;
	fie->height = ov5640->framesize_cfg[fie->index].height;
	fie->interval = ov5640->framesize_cfg[fie->index].max_fps;
	return 0;
}

static const struct v4l2_subdev_core_ops ov5640_subdev_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.ioctl = ov5640_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ov5640_compat_ioctl32,
#endif
	.s_power = ov5640_power,
};

static const struct v4l2_subdev_video_ops ov5640_subdev_video_ops = {
	.s_stream = ov5640_s_stream,
	.g_mbus_config = ov5640_g_mbus_config,
	.g_frame_interval = ov5640_g_frame_interval,
	.s_frame_interval = ov5640_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops ov5640_subdev_pad_ops = {
	.enum_mbus_code = ov5640_enum_mbus_code,
	.enum_frame_size = ov5640_enum_frame_sizes,
	.enum_frame_interval = ov5640_enum_frame_interval,
	.get_fmt = ov5640_get_fmt,
	.set_fmt = ov5640_set_fmt,
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_ops ov5640_subdev_ops = {
	.core  = &ov5640_subdev_core_ops,
	.video = &ov5640_subdev_video_ops,
	.pad   = &ov5640_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops ov5640_subdev_internal_ops = {
	.open = ov5640_open,
};
#endif

static int ov5640_detect(struct ov5640 *ov5640)
{
	struct i2c_client *client = ov5640->client;
	u8 pid, ver;
	int ret;

	dev_dbg(&client->dev, "%s:\n", __func__);

	/* Check sensor revision */
	ret = ov5640_read(client, REG_SC_CHIP_ID_H, &pid);
	if (!ret)
		ret = ov5640_read(client, REG_SC_CHIP_ID_L, &ver);

	if (!ret) {
		unsigned short id;

		id = SENSOR_ID(pid, ver);
		if (id != OV5640_ID) {
			ret = -1;
			dev_err(&client->dev,
				"Sensor detection failed (%04X, %d)\n",
				id, ret);
		} else {
			dev_info(&client->dev, "Found OV%04X sensor\n", id);
			if (!IS_ERR(ov5640->pwdn_gpio))
				gpiod_set_value_cansleep(ov5640->pwdn_gpio, 1);
		}
	}

	return ret;
}

static int __ov5640_power_on(struct ov5640 *ov5640)
{
	int ret;
	struct device *dev = &ov5640->client->dev;

	dev_info(dev, "%s(%d)\n", __func__, __LINE__);
	if (!IS_ERR(ov5640->power_gpio)) {
		gpiod_set_value_cansleep(ov5640->power_gpio, 1);
		usleep_range(2000, 2500);
	}
	if (!IS_ERR_OR_NULL(ov5640->pins_default)) {
		ret = pinctrl_select_state(ov5640->pinctrl,
					   ov5640->pins_default);
		if (ret < 0)
			dev_info(dev, "could not set pins\n");
	}

	if (!IS_ERR(ov5640->reset_gpio)) {
		gpiod_set_value_cansleep(ov5640->reset_gpio, 0);
		usleep_range(2000, 5000);
		gpiod_set_value_cansleep(ov5640->reset_gpio, 1);
		usleep_range(2000, 5000);
	}

	if (!IS_ERR(ov5640->xvclk)) {
		ret = clk_set_rate(ov5640->xvclk, OV5640_XVCLK_FREQ);
		if (ret < 0)
			dev_info(dev, "Failed to set xvclk rate (24MHz)\n");
	}
	if (clk_get_rate(ov5640->xvclk) != OV5640_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");

	if (!IS_ERR(ov5640->pwdn_gpio)) {
		gpiod_set_value_cansleep(ov5640->pwdn_gpio, 1);
		usleep_range(2000, 5000);
	}

	if (!IS_ERR(ov5640->supplies)) {
		ret = regulator_bulk_enable(OV5640_NUM_SUPPLIES,
					    ov5640->supplies);
		if (ret < 0)
			dev_info(dev, "Failed to enable regulators\n");
		usleep_range(2000, 5000);
	}

	if (!IS_ERR(ov5640->pwdn_gpio)) {
		gpiod_set_value_cansleep(ov5640->pwdn_gpio, 0);
		usleep_range(2000, 5000);
	}

	if (!IS_ERR(ov5640->xvclk)) {
		ret = clk_prepare_enable(ov5640->xvclk);
		if (ret < 0)
			dev_info(dev, "Failed to enable xvclk\n");
	}
	usleep_range(2000, 5000);

	return 0;
}

static void __ov5640_power_off(struct ov5640 *ov5640)
{
	int ret;
	struct device *dev = &ov5640->client->dev;

	dev_info(&ov5640->client->dev, "%s(%d)\n", __func__, __LINE__);
	if (!IS_ERR(ov5640->xvclk))
		clk_disable_unprepare(ov5640->xvclk);
	if (!IS_ERR(ov5640->supplies))
		regulator_bulk_disable(OV5640_NUM_SUPPLIES, ov5640->supplies);
	if (!IS_ERR(ov5640->pwdn_gpio))
		gpiod_set_value_cansleep(ov5640->pwdn_gpio, 1);
	if (!IS_ERR(ov5640->reset_gpio))
		gpiod_set_value_cansleep(ov5640->reset_gpio, 0);
	if (!IS_ERR_OR_NULL(ov5640->pins_sleep)) {
		ret = pinctrl_select_state(ov5640->pinctrl,
					   ov5640->pins_sleep);
		if (ret < 0)
			dev_info(dev, "could not set pins\n");
	}

	if (!IS_ERR(ov5640->power_gpio))
		gpiod_set_value_cansleep(ov5640->power_gpio, 0);
}

static int ov5640_configure_regulators(struct ov5640 *ov5640)
{
	unsigned int i;

	for (i = 0; i < OV5640_NUM_SUPPLIES; i++)
		ov5640->supplies[i].supply = ov5640_supply_names[i];

	return devm_regulator_bulk_get(&ov5640->client->dev,
				       OV5640_NUM_SUPPLIES,
				       ov5640->supplies);
}

static int ov5640_parse_of(struct ov5640 *ov5640)
{
	struct device *dev = &ov5640->client->dev;
	struct device_node *endpoint;
	struct fwnode_handle *fwnode;
	int ret;
	int rval;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint),
					 &ov5640->bus_cfg);
	if (ret) {
		dev_err(dev, "Failed to parse endpoint\n");
		of_node_put(endpoint);
		return ret;
	}

	if (ov5640->bus_cfg.bus_type == V4L2_MBUS_CSI2) {
		ov5640->framesize_cfg = ov5640_mipi_framesizes;
		ov5640->cfg_num = ARRAY_SIZE(ov5640_mipi_framesizes);
	} else {
		ov5640->framesize_cfg = ov5640_dvp_framesizes;
		ov5640->cfg_num = ARRAY_SIZE(ov5640_dvp_framesizes);
	}
	ov5640->power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(ov5640->power_gpio))
		dev_info(dev, "Failed to get power-gpios, maybe no use\n");

	ov5640->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(ov5640->pwdn_gpio))
		dev_info(dev, "Failed to get pwdn-gpios, maybe no use\n");
	ov5640->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ov5640->reset_gpio))
		dev_info(dev, "Failed to get reset-gpios, maybe no use\n");

	ret = ov5640_configure_regulators(ov5640);
	if (ret)
		dev_info(dev, "Failed to get power regulators\n");

	ov5640->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(ov5640->pinctrl)) {
		ov5640->pins_default =
			pinctrl_lookup_state(ov5640->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(ov5640->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		ov5640->pins_sleep =
			pinctrl_lookup_state(ov5640->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(ov5640->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	}

	if (ov5640->bus_cfg.bus_type == V4L2_MBUS_CSI2) {
		/* get data lanes*/
		fwnode = of_fwnode_handle(endpoint);
		rval = fwnode_property_read_u32_array(fwnode,
						 "data-lanes", NULL, 0);
		if (rval <= 0) {
			dev_warn(dev, " Get mipi lane num failed!\n");
			return -1;
		}

		ov5640->lane_num = rval;
		if (2 == ov5640->lane_num) {
			/* pixel rate = link frequency * 2 * lanes / 8 */
			ov5640->pixel_rate = MIPI_FREQ * 2U *
							 ov5640->lane_num / 8U;
			dev_info(dev, "lane_num(%d)  pixel_rate(%u)\n",
					 ov5640->lane_num, ov5640->pixel_rate);
		} else {
			dev_err(dev, "unsupported lane_num(%d)\n",
					 ov5640->lane_num);
			return -1;
		}
	}
	return __ov5640_power_on(ov5640);
}

static int ov5640_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct v4l2_subdev *sd;
	struct ov5640 *ov5640;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0x00ff);

	ov5640 = devm_kzalloc(&client->dev, sizeof(*ov5640), GFP_KERNEL);
	if (!ov5640)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &ov5640->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &ov5640->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &ov5640->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &ov5640->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ov5640->client = client;
	ov5640->xvclk = devm_clk_get(&client->dev, "xvclk");
	if (IS_ERR(ov5640->xvclk)) {
		dev_err(&client->dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	ret = ov5640_parse_of(ov5640);
	if (ret != 0)
		return -EINVAL;

	ov5640->xvclk_frequency = clk_get_rate(ov5640->xvclk);
	if (ov5640->xvclk_frequency < 6000000 ||
	    ov5640->xvclk_frequency > 27000000)
		return -EINVAL;

	v4l2_ctrl_handler_init(&ov5640->ctrls, 3);

	ov5640->pixel_freq = v4l2_ctrl_new_std(&ov5640->ctrls, NULL,
					  V4L2_CID_PIXEL_RATE, 0,
					  OV5640_PIXEL_RATE_FULL_SIZE, 1,
					  OV5640_PIXEL_RATE_FULL_SIZE);
	ov5640->link_freq = v4l2_ctrl_new_int_menu(&ov5640->ctrls, NULL,
					V4L2_CID_LINK_FREQ, 1, 0,
					link_freq_menu_items);
	v4l2_ctrl_new_std_menu_items(&ov5640->ctrls, &ov5640_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ov5640_test_pattern_menu) - 1,
				     0, 0, ov5640_test_pattern_menu);
	ov5640->sd.ctrl_handler = &ov5640->ctrls;

	if (ov5640->ctrls.error) {
		dev_err(&client->dev, "%s: control initialization error %d\n",
			__func__, ov5640->ctrls.error);
		return  ov5640->ctrls.error;
	}

	sd = &ov5640->sd;
	client->flags |= I2C_CLIENT_SCCB;
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	v4l2_i2c_subdev_init(sd, client, &ov5640_subdev_ops);

	sd->internal_ops = &ov5640_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif

#if defined(CONFIG_MEDIA_CONTROLLER)
	ov5640->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &ov5640->pad, 0);
	if (ret < 0) {
		v4l2_ctrl_handler_free(&ov5640->ctrls);
		return ret;
	}
#endif

	mutex_init(&ov5640->lock);

	ov5640_get_default_format(ov5640, &ov5640->format);
	ov5640->frame_size = &ov5640->framesize_cfg[0];
	ov5640->format.width = ov5640->framesize_cfg[0].width;
	ov5640->format.height = ov5640->framesize_cfg[0].height;
	ov5640->fps = ov5640->framesize_cfg[0].fps;

	ret = ov5640_detect(ov5640);
	if (ret < 0) {
		dev_info(&client->dev, "Check id  failed\n"
				  "check following information:\n"
				  "Power/PowerDown/Reset/Mclk/I2cBus !!\n");
		goto error;
	}

	memset(facing, 0, sizeof(facing));
	if (strcmp(ov5640->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 ov5640->module_index, facing,
		 DRIVER_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret)
		goto error;

	dev_info(&client->dev, "%s sensor driver registered !!\n", sd->name);

	return 0;

error:
	v4l2_ctrl_handler_free(&ov5640->ctrls);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	mutex_destroy(&ov5640->lock);
	__ov5640_power_off(ov5640);
	return ret;
}

static int ov5640_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5640 *ov5640 = to_ov5640(sd);

	v4l2_ctrl_handler_free(&ov5640->ctrls);
	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	mutex_destroy(&ov5640->lock);

	__ov5640_power_off(ov5640);

	return 0;
}

static const struct i2c_device_id ov5640_id[] = {
	{ "ovti,ov5640", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, ov5640_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov5640_of_match[] = {
	{ .compatible = "ovti,ov5640" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ov5640_of_match);
#endif

static struct i2c_driver ov5640_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(ov5640_of_match),
	},
	.probe		= ov5640_probe,
	.remove		= ov5640_remove,
	.id_table	= ov5640_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&ov5640_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&ov5640_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_AUTHOR("randy wang <randy.wang@rockchips.com>");
MODULE_DESCRIPTION("OV5640 CMOS Image Sensor driver");
MODULE_LICENSE("GPL v2");
