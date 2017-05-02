/*
 * xc9080 isp + hm2131 sensor driver
 *
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * Copyright (C) 2012-2014 Intel Mobile Communications GmbH
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Note:
 *
 *v0.1.0:
 *1. Initialize version;
 */

#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-core.h>
#include <linux/slab.h>
#include "xc9080_camera_module.h"

#define HM2131_DRIVER_NAME "xc9080_hm2131"
#define EXTERNAL_ISP "xc9080"

#define xc9080_AEC_PK_GAIN_REG 0x0205

#define xc9080_AEC_PK_EXPO_HIGH_REG 0x0202
#define xc9080_AEC_PK_EXPO_LOW_REG 0x0203
#define xc9080_FETCH_HIGH_BYTE_EXP(VAL) (((VAL) >> 8) & 0xFF)
#define xc9080_FETCH_LOW_BYTE_EXP(VAL) ((VAL) & 0xFF)

#define XC9080_PIDH_ADDR 0xfffb
#define XC9080_PIDL_ADDR 0xfffc
#define XC9080_PIDH_MAGIC 0x71
#define XC9080_PIDL_MAGIC 0x60

#define HM2131_PIDH_ADDR 0x0000
#define HM2131_PIDL_ADDR 0x0001
#define HM2131_PIDH_MAGIC 0x21
#define HM2131_PIDL_MAGIC 0x31

#define HM2131_TIMING_VTS_HIGH_REG 0x0340
#define HM2131_TIMING_VTS_LOW_REG 0x0341
#define HM2131_TIMING_HTS_HIGH_REG 0x0342
#define HM2131_TIMING_HTS_LOW_REG 0x0343

#define HM2131_INTEGRATION_TIME_MARGIN 8
#define HM2131_FINE_INTG_TIME_MIN 0
#define HM2131_FINE_INTG_TIME_MAX_MARGIN 0
#define HM2131_COARSE_INTG_TIME_MIN 16
#define HM2131_COARSE_INTG_TIME_MAX_MARGIN 4

#define HM2131_HORIZONTAL_START_HIGH_REG 0x0344
#define HM2131_HORIZONTAL_START_LOW_REG	0x0345
#define HM2131_VERTICAL_START_HIGH_REG 0x0346
#define HM2131_VERTICAL_START_LOW_REG 0x0347
#define HM2131_HORIZONTAL_END_HIGH_REG 0x0348
#define HM2131_HORIZONTAL_END_LOW_REG 0x0349
#define HM2131_VERTICAL_END_HIGH_REG 0x034a
#define HM2131_VERTICAL_END_LOW_REG 0x034b
#define HM2131_HORIZONTAL_OUTPUT_SIZE_HIGH_REG 0x034c
#define HM2131_HORIZONTAL_OUTPUT_SIZE_LOW_REG 0x034d
#define HM2131_VERTICAL_OUTPUT_SIZE_HIGH_REG 0x034e
#define HM2131_VERTICAL_OUTPUT_SIZE_LOW_REG 0x034f

#define HM2131_ORIENTATION 0x0101

#define HM2131_EXT_CLK 24000000

static struct xc9080_camera_module hm2131;
static struct xc9080_camera_module_custom_config hm2131_custom_config;

/* ======================================================================== */
/* Base sensor configs */
/* ======================================================================== */

/* MCLK:24MHz 2160x1080 30fps mipi 4lane 576Mbps/lane */
static struct xc9080_camera_module_reg xc9080_init_tab_2160_1080_30fps[] = {
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xfffd, 0x80},
	/* RESET&PLL */
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xfffe, 0x50},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x001c, 0xff},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x001d, 0xff},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x001e, 0xff},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x001f, 0xff},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0018, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0019, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x001a, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x001b, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0030, 0x44},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0031, 0x98},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0032, 0x33},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0033, 0x31},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0020, 0x03},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0021, 0x0d},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0022, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0023, 0x86},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0024, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0025, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0026, 0x02},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0027, 0x06},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0028, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0029, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x002a, 0x02},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x002b, 0x05},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xfffe, 0x50},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0050, 0x0f},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0054, 0x0f},
	/* gpio control */
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0058, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0058, 0x05},
	/* PAD&DATASEL */
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xfffe, 0x50},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x00bc, 0x19},
	/* image:0x38 colorbar:0x3a */
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0090, 0x38},
	/* image:0x09 colorbar:0x0a */
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x00a8, 0x09},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0200, 0x0f},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0201, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0202, 0x80},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0203, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0214, 0x0f},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0215, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0216, 0x80},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0217, 0x00},
	/* COLORBAR&CROP */
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xfffe, 0x26},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8001, 0x88},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8002, 0x07},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8003, 0x40},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8004, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8005, 0x40},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8006, 0x40},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8007, 0x10},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8008, 0xf0},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x800b, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8000, 0x0d},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8041, 0x88},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8042, 0x07},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8043, 0x40},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8044, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8045, 0x03},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8046, 0x10},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8047, 0x10},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8048, 0xff},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x804b, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8040, 0x0d},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8010, 0x09},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8012, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8013, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8014, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8015, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8016, 0xa5},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8017, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8018, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8019, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8050, 0x05},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8052, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8053, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8054, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8055, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8056, 0xa5},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8057, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8058, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x8059, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xfffe, 0x2e},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0026, 0xc0},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0101, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0102, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0103, 0x05},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0104, 0x07},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0105, 0x88},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0106, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0107, 0x40},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0108, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0200, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0201, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0202, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0203, 0x05},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0204, 0x07},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0205, 0x88},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0206, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0207, 0x40},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0208, 0x01},
	/* ISPdatapath */
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xfffe, 0x30},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0000, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0001, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0002, 0x10},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0003, 0x20},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0004, 0x10},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0019, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0050, 0x20},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x005e, 0x37},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x005f, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0060, 0x37},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0061, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0064, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0065, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0066, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0067, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0006, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0007, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0008, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0009, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x000a, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x000b, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x000c, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x000d, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xfffe, 0x31},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0000, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0001, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0002, 0x10},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0003, 0x20},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0004, 0x10},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0019, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0050, 0x20},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x005e, 0x37},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x005f, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0060, 0x37},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0061, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0064, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0065, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0066, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0067, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0006, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0007, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0008, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0009, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x000a, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x000b, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x000c, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x000d, 0x38},
	/* MIPI&FIFO STIICH */
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xfffe, 0x26},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0000, 0x20},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0009, 0xc4},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x1000, 0x20},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x1009, 0xc4},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x2019, 0x08},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x201a, 0x70},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x201b, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x201c, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x201d, 0x08},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x201e, 0x70},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x201f, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x2020, 0x38},
	/* mipi no continue mode */
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x2015, 0x81},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x2017, 0x1e},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x2018, 0x1e},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x2023, 0x0f},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xfffe, 0x2c},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0000, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0001, 0x08},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0002, 0x70},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0004, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0005, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0008, 0x10},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0044, 0x08},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0045, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0048, 0x10},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0049, 0xd0},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0084, 0x08},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0085, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0088, 0x10},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0089, 0xd0},
	/* RETIMING&MERGE */
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xfffe, 0x2e},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0000, 0x42},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0001, 0xee},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0003, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0004, 0xa0},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0006, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0007, 0xa0},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x000a, 0x13},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x000b, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x000c, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x000d, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x1000, 0x0a},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x1001, 0x70},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x1002, 0x08},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x1003, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x1004, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x1005, 0x70},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x1006, 0x08},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x1007, 0x38},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x1008, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x1009, 0x70},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x100a, 0x08},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x100b, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x100e, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x100f, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xfffd, 0x80},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xfffe, 0x14},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xfffd, 0x80},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xfffe, 0x30}
};

static struct xc9080_camera_module_reg hm2131_init_tab_1928_1088_30fps[] = {
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0103, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0304, 0x2a},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0305, 0x0c},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0307, 0x55},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0303, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0309, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x030a, 0x0a},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x030d, 0x02},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x030f, 0x14},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5268, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5264, 0x24},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5265, 0x92},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5266, 0x23},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5267, 0x07},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5269, 0x02},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x02},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0111, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0112, 0x0a},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0113, 0x0a},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b20, 0x8e},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b18, 0x12},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b02, 0x05},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b43, 0x07},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b05, 0x1c},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b0e, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b0f, 0x0d},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b06, 0x06},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b39, 0x0b},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b42, 0x07},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b03, 0x0c},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b04, 0x07},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b3a, 0x0b},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b51, 0x80},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b52, 0x09},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xffff, 0x05},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b52, 0xc9},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b57, 0x07},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b68, 0x6b},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0350, 0x37},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5030, 0x10},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5032, 0x02},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5033, 0xd1},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5034, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5035, 0x67},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5229, 0x90},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5061, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5062, 0x94},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x50f5, 0x06},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5230, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x526c, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x520b, 0x41},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5254, 0x08},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x522b, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4144, 0x08},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4148, 0x03},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4024, 0x40},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b66, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b31, 0x06},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0202, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0203, 0x50},
	/* VTS */
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0340, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0341, 0x52},
	/* HTS */
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0342, 0x05},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0343, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x034c, 0x07},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x034d, 0x88},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x034e, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x034f, 0x40},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0101, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4020, 0x10},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x50dd, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0350, 0x37},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4131, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4132, 0x20},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5011, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5015, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x501d, 0x1c},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x501e, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x501f, 0x20},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x50d5, 0xf0},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x50d7, 0x12},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x50bb, 0x14},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5040, 0x07},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x50b7, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x50b8, 0x10},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x50b9, 0xff},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x50ba, 0xff},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5200, 0x26},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5201, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5202, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5203, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5217, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5219, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5234, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x526b, 0x03},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4c00, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0310, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b31, 0x06},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b3b, 0x02},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b44, 0x0c},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x4b45, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x50a1, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x50aa, 0x2e},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x50ac, 0x44},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x50ab, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x50a0, 0xb0},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x50a2, 0x1b},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x50af, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5208, 0x55},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5209, 0x03},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x520d, 0x40},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5214, 0x18},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5215, 0x03},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5216, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x521a, 0x10},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x521b, 0x24},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5232, 0x04},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5233, 0x03},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5106, 0xf0},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x510e, 0xc1},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5166, 0xf0},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x516e, 0xc1},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5196, 0xf0},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x519e, 0xc1},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51c0, 0x80},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51c4, 0x80},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51c8, 0x80},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51cc, 0x80},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51d0, 0x80},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51d4, 0x80},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51d8, 0x80},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51dc, 0x80},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51c1, 0x03},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51c5, 0x13},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51c9, 0x17},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51cd, 0x27},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51d1, 0x27},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51d5, 0x2b},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51d9, 0x2b},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51dd, 0x2b},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51c2, 0x4b},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51c6, 0x4b},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51ca, 0x4b},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51ce, 0x49},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51d2, 0x49},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51d6, 0x49},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51da, 0x49},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51de, 0x49},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51c3, 0x10},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51c7, 0x18},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51cb, 0x10},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51cf, 0x08},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51d3, 0x08},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51d7, 0x08},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51db, 0x08},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51df, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51e0, 0x94},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51e2, 0x94},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51e4, 0x94},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51e6, 0x94},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51e1, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51e3, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51e5, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x51e7, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5264, 0x23},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5265, 0x07},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5266, 0x23},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5267, 0x92},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5268, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xbaa2, 0xc0},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xbaa2, 0x40},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xba90, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0xba93, 0x02},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x3110, 0x0b},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x373e, 0x8a},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x373f, 0x8a},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x3701, 0x0a},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x3709, 0x0a},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x3703, 0x08},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x370b, 0x08},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x3705, 0x0f},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x370d, 0x0f},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x3713, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x3717, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5043, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5040, 0x05},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x5044, 0x07},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6000, 0x0f},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6001, 0xff},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6002, 0x1f},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6003, 0xff},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6004, 0xc2},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6005, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6006, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6007, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6008, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6009, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x600a, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x600b, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x600c, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x600d, 0x20},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x600e, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x600f, 0xa1},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6010, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6011, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6012, 0x06},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6013, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6014, 0x0b},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6015, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6016, 0x14},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6017, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6018, 0x25},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x6019, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x601a, 0x43},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x601b, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x601c, 0x82},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0000, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0104, 0x01},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0104, 0x00},
	{XC9080_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x03}
};

/* ======================================================================== */

static struct xc9080_camera_module_config hm2131_configs[] = {
	{
		.name = "2160x1080_30fps",
		.frm_fmt = {
			.width = 2160,
			.height = 1080,
			.code = V4L2_MBUS_FMT_YUYV8_2X8
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 30
			}
		},
		.auto_exp_enabled = false,
		.auto_gain_enabled = false,
		.auto_wb_enabled = false,
		.reg_table = (void *)xc9080_init_tab_2160_1080_30fps,
		.reg_table_num_entries =
			ARRAY_SIZE(xc9080_init_tab_2160_1080_30fps),
		.reg_sub_table = (void *)hm2131_init_tab_1928_1088_30fps,
		.reg_sub_table_num_entries =
			ARRAY_SIZE(hm2131_init_tab_1928_1088_30fps),
		.v_blanking_time_us = 5000,
		PLTFRM_CAM_ITF_MIPI_CFG(0, 4, 576, HM2131_EXT_CLK)
	}
};

/*--------------------------------------------------------------------------*/

static int hm2131_g_vts(struct xc9080_camera_module *cam_mod, u32 *vts)
{
	*vts = 1080;
	return 0;
}

/*--------------------------------------------------------------------------*/

static int hm2131_auto_adjust_fps(struct xc9080_camera_module *cam_mod,
	u32 exp_time)
{
	return 0;
}

/*--------------------------------------------------------------------------*/

static int hm2131_write_aec(struct xc9080_camera_module *cam_mod)
{
	return 0;
}

/*--------------------------------------------------------------------------*/

static int hm2131_g_ctrl(struct xc9080_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	xc9080_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_FLASH_LED_MODE:
		/* nothing to be done here */
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (IS_ERR_VALUE(ret))
		xc9080_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int hm2131_filltimings(struct xc9080_camera_module_custom_config *custom)
{
	int i, j;
	struct xc9080_camera_module_config *config;
	struct xc9080_camera_module_timings *timings;
	struct xc9080_camera_module_reg *reg_table;
	int reg_table_num_entries;

	for (i = 0; i < custom->num_configs; i++) {
		config = &custom->configs[i];
		reg_table = config->reg_sub_table;
		reg_table_num_entries = config->reg_sub_table_num_entries;
		timings = &config->timings;

		for (j = 0; j < reg_table_num_entries; j++) {
			switch (reg_table[j].reg) {
			case HM2131_TIMING_VTS_HIGH_REG:
				timings->frame_length_lines =
					reg_table[j].val << 8;
				break;
			case HM2131_TIMING_VTS_LOW_REG:
				timings->frame_length_lines |=
					reg_table[j].val;
				break;
			case HM2131_TIMING_HTS_HIGH_REG:
				timings->line_length_pck =
					(reg_table[j].val << 8);
				break;
			case HM2131_TIMING_HTS_LOW_REG:
				timings->line_length_pck |=
					reg_table[j].val;
				break;
			case HM2131_HORIZONTAL_START_HIGH_REG:
				timings->crop_horizontal_start =
					reg_table[j].val << 8;
				break;
			case HM2131_HORIZONTAL_START_LOW_REG:
				timings->crop_horizontal_start |=
					reg_table[j].val;
				break;
			case HM2131_VERTICAL_START_HIGH_REG:
				timings->crop_vertical_start =
					reg_table[j].val << 8;
				break;
			case HM2131_VERTICAL_START_LOW_REG:
				timings->crop_vertical_start |=
					reg_table[j].val;
				break;
			case HM2131_HORIZONTAL_END_HIGH_REG:
				timings->crop_horizontal_end =
					reg_table[j].val << 8;
				break;
			case HM2131_HORIZONTAL_END_LOW_REG:
				timings->crop_horizontal_end |=
					reg_table[j].val;
				break;
			case HM2131_VERTICAL_END_HIGH_REG:
				timings->crop_vertical_end =
					reg_table[j].val << 8;
				break;
			case HM2131_VERTICAL_END_LOW_REG:
				timings->crop_vertical_end |=
					reg_table[j].val;
				break;
			case HM2131_HORIZONTAL_OUTPUT_SIZE_HIGH_REG:
				timings->sensor_output_width =
					reg_table[j].val << 8;
				break;
			case HM2131_HORIZONTAL_OUTPUT_SIZE_LOW_REG:
				timings->sensor_output_width |=
					reg_table[j].val;
				break;
			case HM2131_VERTICAL_OUTPUT_SIZE_HIGH_REG:
				timings->sensor_output_height =
					reg_table[j].val << 8;
				break;
			case HM2131_VERTICAL_OUTPUT_SIZE_LOW_REG:
				timings->sensor_output_height |=
					reg_table[j].val;
				break;
			}
		}

		timings->vt_pix_clk_freq_hz =
					config->frm_intrvl.interval.denominator
					* timings->frame_length_lines
					* timings->line_length_pck;

		timings->coarse_integration_time_min =
			HM2131_COARSE_INTG_TIME_MIN;
		timings->coarse_integration_time_max_margin =
			HM2131_COARSE_INTG_TIME_MAX_MARGIN;

		timings->fine_integration_time_min = HM2131_FINE_INTG_TIME_MIN;
		timings->fine_integration_time_max_margin =
				HM2131_FINE_INTG_TIME_MAX_MARGIN;
	}

	return 0;
}

static int hm2131_g_timings(struct xc9080_camera_module *cam_mod,
	struct xc9080_camera_module_timings *timings)
{
	int ret = 0;
	unsigned int vts;

	if (IS_ERR_OR_NULL(cam_mod->active_config))
		goto err;

	*timings = cam_mod->active_config->timings;

	vts = (!cam_mod->vts_cur) ?
		timings->frame_length_lines :
		cam_mod->vts_cur;

	if (cam_mod->frm_intrvl_valid)
		timings->vt_pix_clk_freq_hz =
			cam_mod->frm_intrvl.interval.denominator
			* timings->frame_length_lines
			* timings->line_length_pck;
	else
		timings->vt_pix_clk_freq_hz =
			cam_mod->active_config->frm_intrvl.interval.denominator
			* timings->frame_length_lines
			* timings->line_length_pck;

	timings->frame_length_lines = vts;

	return ret;
err:
	xc9080_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int hm2131_set_flip(
	struct xc9080_camera_module *cam_mod,
	struct pltfrm_camera_module_reg reglist[],
	int len)
{
	return 0;
}

/*--------------------------------------------------------------------------*/

static int hm2131_s_ctrl(struct xc9080_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	xc9080_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
		ret = hm2131_write_aec(cam_mod);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (IS_ERR_VALUE(ret))
		xc9080_camera_module_pr_debug(cam_mod,
			"failed with error (%d) 0x%x\n", ret, ctrl_id);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int hm2131_s_ext_ctrls(struct xc9080_camera_module *cam_mod,
	struct xc9080_camera_module_ext_ctrls *ctrls)
{
	int ret = 0;

	/* Handles only exposure and gain together special case. */
	if ((ctrls->ctrls[0].id == V4L2_CID_GAIN ||
		ctrls->ctrls[0].id == V4L2_CID_EXPOSURE))
		ret = hm2131_write_aec(cam_mod);
	else
		ret = -EINVAL;

	if (IS_ERR_VALUE(ret))
		xc9080_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int hm2131_start_streaming(struct xc9080_camera_module *cam_mod)
{
	int ret = 0;

	xc9080_camera_module_pr_info(cam_mod,
		"active config=%s\n", cam_mod->active_config->name);

	ret = hm2131_g_vts(cam_mod, &cam_mod->vts_min);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = xc9080_camera_module_write_reg(
		v4l2_get_subdevdata(&cam_mod->sd), 0xfffe, 0x26);
	ret |= xc9080_camera_module_write_reg(
		v4l2_get_subdevdata(&cam_mod->sd), 0x8010, 0x0d);

	if (IS_ERR_VALUE(ret))
		goto err;

	msleep(25);

	return 0;
err:
	xc9080_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int hm2131_stop_streaming(struct xc9080_camera_module *cam_mod)
{
	int ret = 0;

	xc9080_camera_module_pr_debug(cam_mod, "\n");

	ret = xc9080_camera_module_write_reg(
		v4l2_get_subdevdata(&cam_mod->sd), 0xfffe, 0x26);
	ret |= xc9080_camera_module_write_reg(
		v4l2_get_subdevdata(&cam_mod->sd), 0x8010, 0x09);

	if (IS_ERR_VALUE(ret))
		goto err;

	msleep(25);

	return 0;

err:
	xc9080_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int hm2131_check_camera_id(struct xc9080_camera_module *cam_mod)
{
	u32 pidh, pidl;
	int i, ret = 0;

	xc9080_camera_module_pr_debug(cam_mod, "\n");

	ret |= xc9080_camera_module_read_reg(
		v4l2_get_subdevdata(&cam_mod->sd), 1, XC9080_PIDH_ADDR, &pidh);
	ret |= xc9080_camera_module_read_reg(
		v4l2_get_subdevdata(&cam_mod->sd), 1, XC9080_PIDL_ADDR, &pidl);
	if (IS_ERR_VALUE(ret)) {
		xc9080_camera_module_pr_err(cam_mod,
			"register read failed, xc9080 powered off?\n");
		goto err;
	}

	if ((pidh == XC9080_PIDH_MAGIC) && (pidl == XC9080_PIDL_MAGIC)) {
		xc9080_camera_module_pr_info(cam_mod,
			"successfully detected xc9080 ID 0x%02x%02x\n",
			pidh, pidl);
		xc9080_camera_module_write_reglist(
			v4l2_get_subdevdata(&cam_mod->sd),
			hm2131_configs[0].reg_table,
			hm2131_configs[0].reg_table_num_entries);
	} else {
		xc9080_camera_module_pr_err(cam_mod,
			"wrong xc9080 ID, expected 0x%02x%02x, detected 0x%02x%02x\n",
			XC9080_PIDH_MAGIC, XC9080_PIDL_MAGIC, pidh, pidl);
		ret = -EINVAL;
		goto err;
	}

	for (i = 0; i < ARRAY_SIZE(hm2131_configs[0].sub_client); i++) {
		if (!IS_ERR_OR_NULL(hm2131_configs[0].sub_client[i])) {
			pidh = 0;
			pidl = 0;
			xc9080_camera_module_i2c_bypass(cam_mod, i + 1);
			ret = xc9080_camera_module_read_reg(
				hm2131_configs[0].sub_client[i],
				1, HM2131_PIDH_ADDR, &pidh);
			ret = xc9080_camera_module_read_reg(
				hm2131_configs[0].sub_client[i],
				1, HM2131_PIDL_ADDR, &pidl);

			if (IS_ERR_VALUE(ret)) {
				xc9080_camera_module_pr_err(cam_mod,
					"register read failed, camera module powered off?\n");
				xc9080_camera_module_i2c_bypass(cam_mod,
					XC9080_SUB_I2C_BYPASS_OFF);
				goto err;
			}

			if ((pidh == HM2131_PIDH_MAGIC) &&
				(pidl == HM2131_PIDL_MAGIC)) {
				xc9080_camera_module_pr_info(cam_mod,
					"successfully detected hm2131(0x%x) ID 0x%02x%02x\n",
					hm2131_configs[0].sub_client[i]->addr,
					pidh, pidl);
			} else {
				xc9080_camera_module_pr_err(cam_mod,
					"wrong hm2131(0x%x) ID, expected 0x%02x%02x, detected 0x%02x%02x\n",
					hm2131_configs[0].sub_client[i]->addr,
					HM2131_PIDH_MAGIC, HM2131_PIDL_MAGIC,
					pidh, pidl);
				ret = -EINVAL;
				xc9080_camera_module_i2c_bypass(cam_mod,
					XC9080_SUB_I2C_BYPASS_OFF);
				goto err;
			}
		}
	}

	xc9080_camera_module_i2c_bypass(cam_mod, XC9080_SUB_I2C_BYPASS_OFF);

	return 0;

err:
	xc9080_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/* ======================================================================== */
/* This part is platform dependent */
/* ======================================================================== */

static struct v4l2_subdev_core_ops hm2131_camera_module_core_ops = {
	.g_ctrl = xc9080_camera_module_g_ctrl,
	.s_ctrl = xc9080_camera_module_s_ctrl,
	.s_ext_ctrls = xc9080_camera_module_s_ext_ctrls,
	.s_power = xc9080_camera_module_s_power,
	.ioctl = xc9080_camera_module_ioctl
};

static struct v4l2_subdev_video_ops hm2131_camera_module_video_ops = {
	.enum_frameintervals = xc9080_camera_module_enum_frameintervals,
	.s_mbus_fmt = xc9080_camera_module_s_fmt,
	.g_mbus_fmt = xc9080_camera_module_g_fmt,
	.try_mbus_fmt = xc9080_camera_module_try_fmt,
	.s_frame_interval = xc9080_camera_module_s_frame_interval,
	.g_frame_interval = xc9080_camera_module_g_frame_interval,
	.s_stream = xc9080_camera_module_s_stream
};

static struct v4l2_subdev_ops hm2131_camera_module_ops = {
	.core = &hm2131_camera_module_core_ops,
	.video = &hm2131_camera_module_video_ops,
};

static struct xc9080_camera_module_custom_config hm2131_custom_config = {
	.start_streaming = hm2131_start_streaming,
	.stop_streaming = hm2131_stop_streaming,
	.s_ctrl = hm2131_s_ctrl,
	.s_ext_ctrls = hm2131_s_ext_ctrls,
	.g_ctrl = hm2131_g_ctrl,
	.g_timings = hm2131_g_timings,
	.check_camera_id = hm2131_check_camera_id,
	.set_flip = hm2131_set_flip,
	.s_vts = hm2131_auto_adjust_fps,
	.configs = hm2131_configs,
	.num_configs = ARRAY_SIZE(hm2131_configs),
	.power_up_delays_ms = {5, 20, 0},
	/*
	 * 0: Exposure time valid fileds;
	 * 1: Exposure gain valid fileds;
	 * (2 fileds == 1 frames)
	 */
	.exposure_valid_frame = {4, 4}
};

static int hm2131_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int i;
	static int index;

	dev_info(&client->dev, "probing...\n");

	if (strcmp(client->name, EXTERNAL_ISP) == 0) {
		hm2131_filltimings(&hm2131_custom_config);
		v4l2_i2c_subdev_init(&hm2131.sd,
			client, &hm2131_camera_module_ops);
		hm2131.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
		hm2131.custom = hm2131_custom_config;
	} else {
		for (i = 0; i < ARRAY_SIZE(hm2131_configs); i++)
			hm2131_configs[i].sub_client[index] = client;
		index++;
	}

	dev_info(&client->dev, "probing successful\n");
	return 0;
}

static int hm2131_remove(struct i2c_client *client)
{
	struct xc9080_camera_module *cam_mod = i2c_get_clientdata(client);

	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	xc9080_camera_module_release(cam_mod);

	dev_info(&client->dev, "removed\n");

	return 0;
}

static const struct i2c_device_id hm2131_id[] = {
	{ HM2131_DRIVER_NAME, 0 },
	{ }
};

static const struct of_device_id hm2131_of_match[] = {
	{.compatible = "x-chip,xc9080"},
	{.compatible = "xc9080_hm2131"},
	{},
};

MODULE_DEVICE_TABLE(i2c, hm2131_id);

static struct i2c_driver hm2131_i2c_driver = {
	.driver = {
		.name = HM2131_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = hm2131_of_match
	},
	.probe = hm2131_probe,
	.remove = hm2131_remove,
	.id_table = hm2131_id,
};

module_i2c_driver(hm2131_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for hm2131");
MODULE_AUTHOR("Cain.cai");
MODULE_LICENSE("GPL");
