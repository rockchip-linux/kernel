/*
 * drivers/media/i2c/soc_camera/xgold/ov7251.c
 *
 * ov7251 sensor driver
 *
 * Copyright (C) 2016-2017 Fuzhou Rockchip Electronics Co., Ltd.
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
#include <media/v4l2-controls_rockchip.h>
#include "ov_camera_module.h"

#define OV7251_DRIVER_NAME "ov7251"

#define OV7251_FETCH_LSB_GAIN(VAL) ((VAL) & 0x00FF)     /* gain[7:0] */
#define OV7251_FETCH_MSB_GAIN(VAL) (((VAL) >> 8) & 0x3) /* gain[9:8] */
#define OV7251_AEC_PK_LONG_GAIN_HIGH_REG 0x350a	/* Bit 8 - 9 */
#define OV7251_AEC_PK_LONG_GAIN_LOW_REG	 0x350b	/* Bits 0 -7 */

#define OV7251_AEC_PK_LONG_EXPO_3RD_REG 0x3500	/* Exposure Bits 16-19 */
#define OV7251_AEC_PK_LONG_EXPO_2ND_REG 0x3501	/* Exposure Bits 8-15 */
#define OV7251_AEC_PK_LONG_EXPO_1ST_REG 0x3502	/* Exposure Bits 0-7 */

#define OV7251_AEC_GROUP_UPDATE_ADDRESS 0x3208
#define OV7251_AEC_GROUP_UPDATE_START_DATA 0x00
#define OV7251_AEC_GROUP_UPDATE_END_DATA 0x10
#define OV7251_AEC_GROUP_UPDATE_END_LAUNCH 0xA0

#define OV7251_FETCH_3RD_BYTE_EXP(VAL) (((VAL) >> 12) & 0xF)	/* 4 Bits */
#define OV7251_FETCH_2ND_BYTE_EXP(VAL) (((VAL) >> 4) & 0xFF)	/* 8 Bits */
#define OV7251_FETCH_1ST_BYTE_EXP(VAL) (((VAL) & 0x0F) << 4)	/* 4 Bits */

#define OV7251_PIDH_ADDR     0x300A
#define OV7251_PIDL_ADDR     0x300B

/* High byte of product ID */
#define OV7251_PIDH_MAGIC 0x77
/* Low byte of product ID  */
#define OV7251_PIDL_MAGIC 0x50

#define OV7251_EXT_CLK 24000000
#define OV7251_PLL_PREDIV0_REG 0x3088
#define OV7251_PLL_PREDIV_REG  0x3080
#define OV7251_PLL_MUL_HIGH_REG 0x3081
#define OV7251_PLL_MUL_LOW_REG 0x3082
#define OV7251_PLL_SPDIV_REG 0x3086
#define OV7251_PLL_DIVSYS_REG 0x3084
#define OV7251_TIMING_VTS_HIGH_REG 0x380e
#define OV7251_TIMING_VTS_LOW_REG 0x380f
#define OV7251_TIMING_HTS_HIGH_REG 0x380c
#define OV7251_TIMING_HTS_LOW_REG 0x380d
#define OV7251_FINE_INTG_TIME_MIN 0
#define OV7251_FINE_INTG_TIME_MAX_MARGIN 0
#define OV7251_COARSE_INTG_TIME_MIN 1
#define OV7251_COARSE_INTG_TIME_MAX_MARGIN 20
#define OV7251_TIMING_X_INC		0x3814
#define OV7251_TIMING_Y_INC		0x3815
#define OV7251_HORIZONTAL_START_HIGH_REG 0x3800
#define OV7251_HORIZONTAL_START_LOW_REG 0x3801
#define OV7251_VERTICAL_START_HIGH_REG 0x3802
#define OV7251_VERTICAL_START_LOW_REG 0x3803
#define OV7251_HORIZONTAL_END_HIGH_REG 0x3804
#define OV7251_HORIZONTAL_END_LOW_REG 0x3805
#define OV7251_VERTICAL_END_HIGH_REG 0x3806
#define OV7251_VERTICAL_END_LOW_REG 0x3807
#define OV7251_HORIZONTAL_OUTPUT_SIZE_HIGH_REG 0x3808
#define OV7251_HORIZONTAL_OUTPUT_SIZE_LOW_REG 0x3809
#define OV7251_VERTICAL_OUTPUT_SIZE_HIGH_REG 0x380a
#define OV7251_VERTICAL_OUTPUT_SIZE_LOW_REG 0x380b
#define OV7251_H_WIN_OFF_HIGH_REG 0x3810
#define OV7251_H_WIN_OFF_LOW_REG 0x3811
#define OV7251_V_WIN_OFF_HIGH_REG 0x3812
#define OV7251_V_WIN_OFF_LOW_REG 0x3813

#define OV7251_ANA_ARRAR01 0x3621
#define OV7251_TIMING_CONCTROL_VH 0x3803
#define OV7251_TIMING_CONCTROL18 0x3818

/* ======================================================================== */
/* Base sensor configs */
/* ======================================================================== */
/* MCLK:24MHz  1920x1080  30fps   mipi 1lane   800Mbps/lane */
static struct ov_camera_module_reg OV7251_init_tab_640_480_100fps[] = {
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x00},/* software sleep */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0103, 0x01},/* software reset */
	{OV_CAMERA_MODULE_REG_TYPE_TIMEOUT, 0x0000, 10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3001, 0x62},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3005, 0x00},/* SDA output, vsync output, pwm output, strobe output */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3012, 0xc0},/* mipi_ictl */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3013, 0xd2},/* drive strength = 0x01, bypass latch of hs_enable */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3014, 0x04},/* MIPI enable */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3016, 0x10},/* sclk control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3017, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3018, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x301a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x301b, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x301c, 0x20},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3023, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3037, 0xf0},/* sclk control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3098, 0x04},/* PLL */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3099, 0x28},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x309a, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x309b, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30b0, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30b1, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30b3, 0x64},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30b4, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x30b5, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3106, 0xda},/* PLL */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3500, 0x00},/* exposure H */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3501, 0x1f},/* exposure M */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3502, 0x80},/* exposure L */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3503, 0x07},/* Manual AGC, Manual AEC */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3509, 0x10},/* choose linear gain */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x350b, 0x10},/* gain L */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3600, 0x1c},/* analog control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3602, 0x62},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3620, 0xb7},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3622, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3626, 0x21},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3627, 0x30},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3630, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3631, 0x35},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3634, 0x60},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3636, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3662, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3663, 0x70},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3664, 0xf0},/* analog control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3666, 0x0a},/* FSIN/VSYNC */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3669, 0x1a},/* analog control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x366b, 0x50},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3673, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3674, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3675, 0x03},/* analog control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3705, 0xc1},/* sensor control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3709, 0x40},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x373c, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3742, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3757, 0xb3},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3788, 0x00},/* sensor control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37a8, 0x01},/* fifo control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x37a9, 0xc0},/* fifo control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3800, 0x00},/* x start H */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3801, 0x04},/* x start L */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3802, 0x00},/* y start H */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3803, 0x04},/* y start L */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3804, 0x02},/* x end H */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3805, 0x8b},/* x end L */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3806, 0x01},/* y end H */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3807, 0xeb},/* y end L */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3808, 0x02},/* isp x output size H */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3809, 0x80},/* isp x output size L */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380a, 0x01},/* isp y output size H */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380b, 0xe0},/* isp y output size L */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380c, 0x03},/* HTS H */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380d, 0xa0},/* HTS L */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e, 0x02},/* VTS H */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f, 0x04},/* VTS L */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3810, 0x00},/* ISP x win offset H */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x02},/* ISP x win offset L  */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3812, 0x00},/* ISP y win offset H */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x02},/* ISP y win offset L */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3814, 0x11},/* x inc */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3815, 0x11},/* y inc */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0x40},/* vflip_blc */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x00},/* hbin disable */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x382f, 0x0e},/* timing control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3832, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3833, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3834, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3835, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3837, 0x00},/* timing control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b80, 0x00},/* PWM and strobe control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b81, 0xa5},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b82, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b83, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b84, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b85, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b86, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b87, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b88, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b89, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b8a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b8b, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b8c, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b8d, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b8e, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b8f, 0x1a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b94, 0x05},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b95, 0xf2},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b96, 0x40},/* PWM and strobe control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c00, 0x89},/* low power mode control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c01, 0x63},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c02, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c03, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c04, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c05, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c06, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c07, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c0c, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c0d, 0xd0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c0e, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c0f, 0x04},/* low power mode control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4001, 0x42},/* BLC control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4004, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4005, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x404e, 0x01},/* BLC control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4241, 0x00},/* frame on number */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4242, 0x00},/* frame off number */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4300, 0xff},/* data max H */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4301, 0x00},/* data min H */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4501, 0x48},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4600, 0x00},/* vfifo read start H */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4601, 0x4e},/* vfifo read start L */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4801, 0x0f},/* MIPI control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4806, 0x0f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4819, 0xaa},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4823, 0x3e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4837, 0x19},/* MIPI control */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4a0d, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4a47, 0x7f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4a49, 0xf0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4a4b, 0x30},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5000, 0x85},/* bc_en, blc_en */
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5001, 0x80} /* latch_en */
};

/* ======================================================================== */
static struct ov_camera_module_config OV7251_configs[] = {
	{
		.name = "640x480_100fps",
		.frm_fmt = {
			.width = 640,
			.height = 480,
			.code = V4L2_MBUS_FMT_Y10_1X10
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 100
			}
		},
		.auto_exp_enabled = false,
		.auto_gain_enabled = false,
		.auto_wb_enabled = false,
		.reg_table = (void *)OV7251_init_tab_640_480_100fps,
		.reg_table_num_entries = ARRAY_SIZE(OV7251_init_tab_640_480_100fps),
		.v_blanking_time_us = 3078,
		.ignore_measurement_check = 1,
		PLTFRM_CAM_ITF_MIPI_CFG(0, 1, 640, 24000000)
	}
};

/*--------------------------------------------------------------------------*/
static int OV7251_set_flip(
	struct ov_camera_module *cam_mod,
	struct pltfrm_camera_module_reg reglist[],
	int len)
{
	int i, mode = 0;
	u16 match_reg[3];

	mode = ov_camera_module_get_flip_mirror(cam_mod);

	if (mode == -1) {
		ov_camera_module_pr_debug(
			cam_mod,
			"dts don't set flip, return!\n");
		return 0;
	}

	if (mode == OV_FLIP_BIT_MASK) {
		match_reg[0] = 0x04;
		match_reg[1] = 0x09;
		match_reg[2] = 0xa0;
	} else if (mode == OV_MIRROR_BIT_MASK) {
		match_reg[0] = 0x14;
		match_reg[1] = 0x0a;
		match_reg[2] = 0xc0;
	} else if (mode == (OV_MIRROR_BIT_MASK |
		OV_FLIP_BIT_MASK)) {
		match_reg[0] = 0x14;
		match_reg[1] = 0x09;
		match_reg[2] = 0xe0;
	} else {
		match_reg[0] = 0x04;
		match_reg[1] = 0x0a;
		match_reg[2] = 0x80;
	}

	for (i = len; i > 0; i--) {
		if (reglist[i].reg == OV7251_ANA_ARRAR01)
			reglist[i].val = match_reg[0];
		else if (reglist[i].reg == OV7251_TIMING_CONCTROL_VH)
			reglist[i].val = match_reg[1];
		else if (reglist[i].reg == OV7251_TIMING_CONCTROL18)
			reglist[i].val = match_reg[2];
	}

	return 0;
}

/*--------------------------------------------------------------------------*/
static int OV7251_g_VTS(struct ov_camera_module *cam_mod, u32 *vts)
{
	u32 msb, lsb;
	int ret;

	ret = ov_camera_module_read_reg_table(
		cam_mod,
		OV7251_TIMING_VTS_HIGH_REG,
		&msb);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = ov_camera_module_read_reg_table(
		cam_mod,
		OV7251_TIMING_VTS_LOW_REG,
		&lsb);
	if (IS_ERR_VALUE(ret))
		goto err;

	*vts = (msb << 8) | lsb;

	return 0;
err:
	ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

static int OV7251_auto_adjust_fps(struct ov_camera_module *cam_mod,
	u32 exp_time)
{
	int ret;
	u32 vts;

	if ((exp_time + OV7251_COARSE_INTG_TIME_MAX_MARGIN)
		> cam_mod->vts_min)
		vts = exp_time + OV7251_COARSE_INTG_TIME_MAX_MARGIN;
	else
		vts = cam_mod->vts_min;

	if (vts > 0xffff)
		vts = 0xffff;
	else
		vts = vts;  /*VTS value is 0x380e[3:0]/380f[7:0]*/

	ret = ov_camera_module_write_reg(cam_mod,
		OV7251_TIMING_VTS_LOW_REG,
		vts & 0xFF);
	ret |= ov_camera_module_write_reg(cam_mod,
		OV7251_TIMING_VTS_HIGH_REG,
		(vts >> 8) & 0xFF);

	if (IS_ERR_VALUE(ret)) {
		ov_camera_module_pr_err(cam_mod,
				"failed with error (%d)\n", ret);
	} else {
		ov_camera_module_pr_debug(cam_mod,
					  "updated vts = 0x%x,vts_min=0x%x\n", vts, cam_mod->vts_min);
		cam_mod->vts_cur = vts;
	}

	return ret;
}

static int OV7251_write_aec(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod,
		"exp_time = %d lines, gain = %d, flash_mode = %d\n",
		cam_mod->exp_config.exp_time,
		cam_mod->exp_config.gain,
		cam_mod->exp_config.flash_mode);

	/*
	 * if the sensor is already streaming, write to shadow registers,
	 * if the sensor is in SW standby, write to active registers,
	 * if the sensor is off/registers are not writeable, do nothing
	 */
	if ((cam_mod->state == OV_CAMERA_MODULE_SW_STANDBY) ||
		(cam_mod->state == OV_CAMERA_MODULE_STREAMING)) {
		u32 a_gain = cam_mod->exp_config.gain;
		u32 exp_time = cam_mod->exp_config.exp_time;
		a_gain = a_gain * cam_mod->exp_config.gain_percent / 100;

		mutex_lock(&cam_mod->lock);
		ret = ov_camera_module_write_reg(cam_mod,
			OV7251_AEC_GROUP_UPDATE_ADDRESS,
			OV7251_AEC_GROUP_UPDATE_START_DATA);

		if (!IS_ERR_VALUE(ret) && cam_mod->auto_adjust_fps)
			ret |= OV7251_auto_adjust_fps(cam_mod,
					cam_mod->exp_config.exp_time);
		ret |= ov_camera_module_write_reg(cam_mod,
			OV7251_AEC_PK_LONG_GAIN_HIGH_REG,
			OV7251_FETCH_MSB_GAIN(a_gain));
		ret |= ov_camera_module_write_reg(cam_mod,
			OV7251_AEC_PK_LONG_GAIN_LOW_REG,
			OV7251_FETCH_LSB_GAIN(a_gain));
		ret |= ov_camera_module_write_reg(cam_mod,
			OV7251_AEC_PK_LONG_EXPO_3RD_REG,
			OV7251_FETCH_3RD_BYTE_EXP(exp_time));
		ret |= ov_camera_module_write_reg(cam_mod,
			OV7251_AEC_PK_LONG_EXPO_2ND_REG,
			OV7251_FETCH_2ND_BYTE_EXP(exp_time));
		ret |= ov_camera_module_write_reg(cam_mod,
			OV7251_AEC_PK_LONG_EXPO_1ST_REG,
			OV7251_FETCH_1ST_BYTE_EXP(exp_time));

		ret |= ov_camera_module_write_reg(cam_mod,
			OV7251_AEC_GROUP_UPDATE_ADDRESS,
			OV7251_AEC_GROUP_UPDATE_END_DATA);
		ret |= ov_camera_module_write_reg(cam_mod,
			OV7251_AEC_GROUP_UPDATE_ADDRESS,
			OV7251_AEC_GROUP_UPDATE_END_LAUNCH);
		mutex_unlock(&cam_mod->lock);
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

static int OV7251_g_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

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
		ov_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int OV7251_filltimings(struct ov_camera_module_custom_config *custom)
{
	int i, j;
	u32 win_h_off = 0, win_v_off = 0;
	struct ov_camera_module_config *config;
	struct ov_camera_module_timings *timings;
	struct ov_camera_module_reg *reg_table;
	int reg_table_num_entries;

	for (i = 0; i < custom->num_configs; i++) {
		config = &custom->configs[i];
		reg_table = config->reg_table;
		reg_table_num_entries = config->reg_table_num_entries;
		timings = &config->timings;

		memset(timings, 0x00, sizeof(*timings));
		for (j = 0; j < reg_table_num_entries; j++) {
			switch (reg_table[j].reg) {
			case OV7251_TIMING_VTS_HIGH_REG:
				timings->frame_length_lines =
					((reg_table[j].val << 8) |
					(timings->frame_length_lines & 0xff));
				break;
			case OV7251_TIMING_VTS_LOW_REG:
				timings->frame_length_lines =
					(reg_table[j].val |
					(timings->frame_length_lines & 0xff00));
				break;
			case OV7251_TIMING_HTS_HIGH_REG:
				timings->line_length_pck =
					((reg_table[j].val << 8) |
					timings->line_length_pck);
				break;
			case OV7251_TIMING_HTS_LOW_REG:
				timings->line_length_pck =
					(reg_table[j].val |
					(timings->line_length_pck & 0xff00));
				break;
			case OV7251_TIMING_X_INC:
				timings->binning_factor_x =
					((reg_table[j].val >> 4) + 1) / 2;
				if (timings->binning_factor_x == 0)
					timings->binning_factor_x = 1;
				break;
			case OV7251_TIMING_Y_INC:
				timings->binning_factor_y =
					((reg_table[j].val >> 4) + 1) / 2;
				if (timings->binning_factor_y == 0)
					timings->binning_factor_y = 1;
				break;
			case OV7251_HORIZONTAL_START_HIGH_REG:
				timings->crop_horizontal_start =
					((reg_table[j].val << 8) |
					(timings->crop_horizontal_start & 0xff));
				break;
			case OV7251_HORIZONTAL_START_LOW_REG:
				timings->crop_horizontal_start =
					(reg_table[j].val |
					(timings->crop_horizontal_start & 0xff00));
				break;
			case OV7251_VERTICAL_START_HIGH_REG:
				timings->crop_vertical_start =
					((reg_table[j].val << 8) |
					(timings->crop_vertical_start & 0xff));
				break;
			case OV7251_VERTICAL_START_LOW_REG:
				timings->crop_vertical_start =
					((reg_table[j].val) |
					(timings->crop_vertical_start & 0xff00));
				break;
			case OV7251_HORIZONTAL_END_HIGH_REG:
				timings->crop_horizontal_end =
					((reg_table[j].val << 8) |
					(timings->crop_horizontal_end & 0xff));
				break;
			case OV7251_HORIZONTAL_END_LOW_REG:
				timings->crop_horizontal_end =
					(reg_table[j].val |
					(timings->crop_horizontal_end & 0xff00));
				break;
			case OV7251_VERTICAL_END_HIGH_REG:
				timings->crop_vertical_end =
					((reg_table[j].val << 8) |
					(timings->crop_vertical_end & 0xff));
				break;
			case OV7251_VERTICAL_END_LOW_REG:
				timings->crop_vertical_end =
					(reg_table[j].val |
					(timings->crop_vertical_end & 0xff00));
				break;
			case OV7251_H_WIN_OFF_HIGH_REG:
				win_h_off = (reg_table[j].val & 0xff) << 8;
				break;
			case OV7251_H_WIN_OFF_LOW_REG:
				win_h_off |= (reg_table[j].val & 0xff);
				break;
			case OV7251_V_WIN_OFF_HIGH_REG:
				win_v_off = (reg_table[j].val & 0xff) << 8;
				break;
			case OV7251_V_WIN_OFF_LOW_REG:
				win_v_off |= (reg_table[j].val & 0xff);
				break;
			case OV7251_HORIZONTAL_OUTPUT_SIZE_HIGH_REG:
				timings->sensor_output_width =
					((reg_table[j].val << 8) |
					(timings->sensor_output_width & 0xff));
				break;
			case OV7251_HORIZONTAL_OUTPUT_SIZE_LOW_REG:
				timings->sensor_output_width =
					(reg_table[j].val |
					(timings->sensor_output_width & 0xff00));
				break;
			case OV7251_VERTICAL_OUTPUT_SIZE_HIGH_REG:
				timings->sensor_output_height =
					((reg_table[j].val << 8) |
					(timings->sensor_output_height & 0xff));
				break;
			case OV7251_VERTICAL_OUTPUT_SIZE_LOW_REG:
				timings->sensor_output_height =
					(reg_table[j].val |
					(timings->sensor_output_height & 0xff00));
				break;
			}
		}

		timings->crop_horizontal_start += win_h_off;
		timings->crop_horizontal_end -= win_h_off;
		timings->crop_vertical_start += win_v_off;
		timings->crop_vertical_end -= win_v_off;

		timings->exp_time >>= 4;
		timings->vt_pix_clk_freq_hz =
			config->frm_intrvl.interval.denominator
			* timings->frame_length_lines
			* timings->line_length_pck;

		timings->coarse_integration_time_min =
			OV7251_COARSE_INTG_TIME_MIN;
		timings->coarse_integration_time_max_margin =
			OV7251_COARSE_INTG_TIME_MAX_MARGIN;

		/* OV Sensor do not use fine integration time. */
		timings->fine_integration_time_min =
			OV7251_FINE_INTG_TIME_MIN;
		timings->fine_integration_time_max_margin =
			OV7251_FINE_INTG_TIME_MAX_MARGIN;
	}

	return 0;
}

/*--------------------------------------------------------------------------*/

static int OV7251_g_timings(struct ov_camera_module *cam_mod,
			    struct ov_camera_module_timings *timings)
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
			* vts
			* timings->line_length_pck;
	else
		timings->vt_pix_clk_freq_hz =
			cam_mod->active_config->frm_intrvl.interval.denominator
			* vts
			* timings->line_length_pck;

	return ret;
err:
	ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int OV7251_s_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
		ret = OV7251_write_aec(cam_mod);
		break;
	case V4L2_CID_FLASH_LED_MODE:
		/* nothing to be done here */
		break;
	case V4L2_CID_FOCUS_ABSOLUTE:
		/* todo */
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int OV7251_s_ext_ctrls(struct ov_camera_module *cam_mod,
				 struct ov_camera_module_ext_ctrls *ctrls)
{
	int ret = 0;

	if ((ctrls->ctrls[0].id == V4L2_CID_GAIN ||
		ctrls->ctrls[0].id == V4L2_CID_EXPOSURE))
		ret = OV7251_write_aec(cam_mod);
	else
		ret = -EINVAL;

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);

	return ret;
}

/*--------------------------------------------------------------------------*/

static int OV7251_start_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "active config=%s\n", cam_mod->active_config->name);

	ret = OV7251_g_VTS(cam_mod, &cam_mod->vts_min);
	if (IS_ERR_VALUE(ret))
		goto err;

	mutex_lock(&cam_mod->lock);
	ret = ov_camera_module_write_reg(cam_mod, 0x0100, 0x01);
	ret |= ov_camera_module_write_reg(cam_mod, 0x4241, 0x00);
	ret |= ov_camera_module_write_reg(cam_mod, 0x4242, 0x00);
	mutex_unlock(&cam_mod->lock);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n",
		ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int OV7251_stop_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");
	mutex_lock(&cam_mod->lock);
	ret = ov_camera_module_write_reg(cam_mod, 0x0100, 0x00);
	ret |= ov_camera_module_write_reg(cam_mod, 0x4241, 0x00);
	ret |= ov_camera_module_write_reg(cam_mod, 0x4242, 0x0f);
	mutex_unlock(&cam_mod->lock);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int OV7251_check_camera_id(struct ov_camera_module *cam_mod)
{
	u32 pidh, pidl;
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret |= ov_camera_module_read_reg(cam_mod, 1, OV7251_PIDH_ADDR, &pidh);
	ret |= ov_camera_module_read_reg(cam_mod, 1, OV7251_PIDL_ADDR, &pidl);
	if (IS_ERR_VALUE(ret)) {
		ov_camera_module_pr_err(cam_mod,
			"register read failed, camera module powered off?\n");
		goto err;
	}

	if ((pidh == OV7251_PIDH_MAGIC) && (pidl == OV7251_PIDL_MAGIC))
		ov_camera_module_pr_err(cam_mod,
			"successfully detected camera ID 0x%02x%02x\n",
			pidh, pidl);
	else {
		ov_camera_module_pr_err(cam_mod,
			"wrong camera ID, expected 0x%02x%02x, detected 0x%02x%02x\n",
			OV7251_PIDH_MAGIC, OV7251_PIDL_MAGIC, pidh, pidl);
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/* ======================================================================== */
int ov_camera_7251_module_s_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	return 0;
}

/* ======================================================================== */

int ov_camera_7251_module_s_ext_ctrls(
	struct v4l2_subdev *sd,
	struct v4l2_ext_controls *ctrls)
{
	return 0;
}

long ov_camera_7251_module_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd,
	void *arg)
{
	return 0;
}

/* ======================================================================== */
/* This part is platform dependent */
/* ======================================================================== */

static struct v4l2_subdev_core_ops OV7251_camera_module_core_ops = {
	.g_ctrl = ov_camera_module_g_ctrl,
	.s_ctrl = ov_camera_module_s_ctrl,
	.s_ext_ctrls = ov_camera_module_s_ext_ctrls,
	.s_power = ov_camera_module_s_power,
	.ioctl = ov_camera_module_ioctl
};

static struct v4l2_subdev_video_ops OV7251_camera_module_video_ops = {
	.enum_frameintervals = ov_camera_module_enum_frameintervals,
	.s_mbus_fmt = ov_camera_module_s_fmt,
	.g_mbus_fmt = ov_camera_module_g_fmt,
	.try_mbus_fmt = ov_camera_module_try_fmt,
	.s_frame_interval = ov_camera_module_s_frame_interval,
	.s_stream = ov_camera_module_s_stream
};

static struct v4l2_subdev_ops OV7251_camera_module_ops = {
	.core = &OV7251_camera_module_core_ops,
	.video = &OV7251_camera_module_video_ops
};

static struct ov_camera_module OV7251;

static struct ov_camera_module_custom_config OV7251_custom_config = {
	.start_streaming = OV7251_start_streaming,
	.stop_streaming = OV7251_stop_streaming,
	.s_ctrl = OV7251_s_ctrl,
	.g_ctrl = OV7251_g_ctrl,
	.s_ext_ctrls = OV7251_s_ext_ctrls,
	.g_timings = OV7251_g_timings,
	.set_flip = OV7251_set_flip,
	.check_camera_id = OV7251_check_camera_id,
	.s_vts = OV7251_auto_adjust_fps,
	.configs = OV7251_configs,
	.num_configs = ARRAY_SIZE(OV7251_configs),
	.power_up_delays_ms = {5, 30, 30},
	/*
	 * 0: Exposure time valid fileds;
	 * 1: Exposure gain valid fileds;
	 * (2 fileds == 1 frames)
	 */
	.exposure_valid_frame = {4, 2}
};

static int OV7251_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	dev_info(&client->dev, "probing...\n");

	OV7251_filltimings(&OV7251_custom_config);

	v4l2_i2c_subdev_init(&OV7251.sd, client,
				&OV7251_camera_module_ops);
	OV7251.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	OV7251.custom = OV7251_custom_config;

	mutex_init(&OV7251.lock);
	dev_info(&client->dev, "probing successful\n");
	return 0;
}

/* ======================================================================== */

static int OV7251_remove(
	struct i2c_client *client)
{
	struct ov_camera_module *cam_mod = i2c_get_clientdata(client);

	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	mutex_destroy(&cam_mod->lock);
	ov_camera_module_release(cam_mod);

	dev_info(&client->dev, "removed\n");
	return 0;
}

static const struct i2c_device_id OV7251_id[] = {
	{ OV7251_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id OV7251_of_match[] = {
	{.compatible = "omnivision,ov7251-bw-v4l2-i2c-subdev"},
	{},
};

MODULE_DEVICE_TABLE(i2c, OV7251_id);

static struct i2c_driver OV7251_i2c_driver = {
	.driver = {
		.name = OV7251_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = OV7251_of_match
	},
	.probe = OV7251_probe,
	.remove = OV7251_remove,
	.id_table = OV7251_id,
};

module_i2c_driver(OV7251_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for OV7251");
MODULE_AUTHOR("Eike Grimpe");
MODULE_LICENSE("GPL");

