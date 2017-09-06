/*
 * drivers/media/i2c/soc_camera/xgold/gc2023.c
 *
 * gc2023 sensor driver
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
 *
 */

#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-core.h>
#include <linux/slab.h>
#include <media/v4l2-controls_rockchip.h>
#include "gc_camera_module.h"
#include <linux/gpio.h>

#define GC2023_DRIVER_NAME "gc2023"

#define GC2023_FETCH_GAIN(VAL) (VAL & 0xFF)       /* gain[7:0] */
#define GC2023_AEC_ANALOG_GAIN_REG	 0xb6	/* Bits 0 -7 */
#define GC2023_AEC_DIGITAL_INT_GAIN_REG	 0xb1	/* Bits 0 -3 */
#define GC2023_AEC_DIGITAL_FRAC_GAIN_REG 0xb2	/* Bits 2 -7 */

#define GC2023_AEC_EXPO_HIGH_REG 0x03	/* Exposure Bits 8-12 */
#define GC2023_AEC_EXPO_LOW_REG 0x04	/* Exposure Bits 0-7 */

#define GC2023_FETCH_HIGH_BYTE_EXP(VAL) ((VAL >> 8) & 0x1F)	/* 5 Bits */
#define GC2023_FETCH_LOW_BYTE_EXP(VAL) (VAL & 0xFF)	/* 8 Bits */

#define GC2023_PIDH_ADDR     0xF0
#define GC2023_PIDL_ADDR     0xF1

/* High byte of product ID */
#define GC2023_PIDH_MAGIC 0x20
/* Low byte of product ID  */
#define GC2023_PIDL_MAGIC 0x23

#define GC2023_EXT_CLK 24000000
#define GC2023_TIMING_VB_HIGH_REG 0x07
#define GC2023_TIMING_VB_LOW_REG 0x08
#define GC2023_TIMING_HB_HIGH_REG 0x09
#define GC2023_TIMING_HB_LOW_REG 0x0a
#define GC2023_TIMING_WINV_HIGH_REG 0x0d
#define GC2023_TIMING_WINV_LOW_REG 0x0e
#define GC2023_TIMING_WINH_HIGH_REG 0x0f
#define GC2023_TIMING_WINH_LOW_REG 0x10
#define GC2023_FINE_INTG_TIME_MIN 0
#define GC2023_FINE_INTG_TIME_MAX_MARGIN 0
#define GC2023_COARSE_INTG_TIME_MIN 1
#define GC2023_COARSE_INTG_TIME_MAX_MARGIN 4
#define GC2023_HORIZONTAL_OUTPUT_SIZE_HIGH_REG 0x97
#define GC2023_HORIZONTAL_OUTPUT_SIZE_LOW_REG 0x98
#define GC2023_VERTICAL_OUTPUT_SIZE_HIGH_REG 0x95
#define GC2023_VERTICAL_OUTPUT_SIZE_LOW_REG 0x96
#define GC2023_H_WIN_OFF_HIGH_REG 0x93
#define GC2023_H_WIN_OFF_LOW_REG 0x94
#define GC2023_V_WIN_OFF_HIGH_REG 0x91
#define GC2023_V_WIN_OFF_LOW_REG 0x92

#define GC2023_UPDOWN_MIRROR 0x17

static u32 GC2023_AGAIN_TBL[][2] = {
	{0xFF, 0},
	{0x00, 100}, /* 1x */
	{0x01, 142}, /* 1.42x */
	{0x02, 199}, /* 1.99x */
	{0x03, 286}, /* 2.86x */
	{0x04, 405}, /* 4.05x */
	{0x05, 578}, /* 5.78x */
	{0x06, 823}, /* 8.23x */
	{0x07, 1172}, /* 11.72x */
	{0x08, 1655}, /* 16.55x */
	{0x09, 2328}, /* 23.28x */
	{0x0a, 3327}, /* 33.27x */
};

/* ======================================================================== */
/* Base sensor configs */
/* ======================================================================== */
/* Actual_window_size=1920*1080 */
/* mclk=24Mhz,mipi_clock=768mhz,frame_rate=30fps */
/* pixel_line_total=2840,line_frame_total=1125,row_time=29.58us */
static struct gc_camera_module_reg gc2023_init_tab_1920_1080_30fps_1lane[] = {
	/*  SYS */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xfe, 0x80},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xfe, 0x80},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xfe, 0x80},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xf2, 0x00},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xf6, 0x00},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xfc, 0x06},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xf7, 0x01},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xf8, 0x0f},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xf9, 0x0e}, /* [0] pll enable */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xfa, 0x00},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xfc, 0x1e},

	/*  ANALOG & CISCTL */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xfe, 0x00},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x03, 0x03},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x04, 0xf6},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x05, 0x02}, /* HB */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x06, 0xc6},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x07, 0x00}, /* VB */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x08, 0x10},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x09, 0x00},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x0a, 0x00}, /* row start */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x0b, 0x00},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x0c, 0x00}, /* col start */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x0d, 0x04},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x0e, 0x40}, /* height 1088 */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x0f, 0x07},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x10, 0x88}, /* width 1928 */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x17, 0x54},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x18, 0x02},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x19, 0x0d},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x1a, 0x18},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x20, 0x54},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x23, 0xf0},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x24, 0xc1},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x25, 0x18},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x26, 0x64},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x28, 0xf4},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x29, 0x08},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x2a, 0x08},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x2f, 0x40},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x30, 0x99},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x34, 0x00},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x38, 0x80},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x3b, 0x12},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x3d, 0xb0},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xcc, 0x8a},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xcd, 0x99},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xcf, 0x70},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xd0, 0x9a},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xd2, 0xc1},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xd8, 0x80},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xda, 0x28},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xdc, 0x24},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xe1, 0x14},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xe3, 0xf0},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xe4, 0xfa},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xe6, 0x1f},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xe8, 0x02},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xe9, 0x02},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xea, 0x03},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xeb, 0x03},

	/*  ISP */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xfe, 0x00},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x80, 0x5c},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x88, 0x73},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x89, 0x03},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x90, 0x01},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x92, 0x00}, /* crop win y */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x94, 0x00}, /* crop win x */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x95, 0x04}, /* crop win height */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x96, 0x38},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x97, 0x07}, /* crop win width 1920 */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x98, 0x80},

	/*  BLK */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xfe, 0x00},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x40, 0x22},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x4e, 0x3c},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x4f, 0x00},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x60, 0x00},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x61, 0x80},

	/*  GAIN */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xfe, 0x00},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xb0, 0x48},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xb1, 0x01},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xb2, 0x00},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xb6, 0x00},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xfe, 0x01},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x01, 0x00},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x02, 0x01},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x03, 0x02},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x04, 0x03},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x05, 0x04},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x06, 0x05},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x07, 0x06},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x08, 0x0e},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x09, 0x16},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x0a, 0x1e},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x0b, 0x36},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x0c, 0x3e},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x0d, 0x56},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x0e, 0x5e},

	/*  DNDD */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xfe, 0x02},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x81, 0x05},

	/* dark sun */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xfe, 0x01},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x54, 0x77},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x58, 0x00},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x5a, 0x05},

	/* MIPI */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xfe, 0x03},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x01, 0x5b},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x02, 0x10},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x03, 0xaa},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x10, 0x90},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x11, 0x2b},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x12, 0x60}, /* lwc 1920*5/4 */
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x13, 0x09},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x15, 0x06},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x36, 0x88},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x21, 0x10},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x22, 0x05},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x23, 0x30},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x24, 0x02},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x25, 0x12},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x26, 0x08},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x29, 0x06},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x2a, 0x0a},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0x2b, 0x08},
	{GC_CAMERA_MODULE_REG_SINGLE_ONE_BYTE_DATA, 0xfe, 0x00},
};

/* ======================================================================== */
static struct gc_camera_module_config gc2023_configs[] = {
	{
		.name = "1920x1080_30fps",
		.frm_fmt = {
			.width = 1920,
			.height = 1080,
			.code = V4L2_MBUS_FMT_SRGGB10_1X10
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
		.reg_table = (void *)gc2023_init_tab_1920_1080_30fps_1lane,
		.reg_table_num_entries =
			sizeof(gc2023_init_tab_1920_1080_30fps_1lane) /
			sizeof(gc2023_init_tab_1920_1080_30fps_1lane[0]),
		.v_blanking_time_us = 5000,
		.ignore_measurement_check = 1,
		PLTFRM_CAM_ITF_MIPI_CFG(0, 1, 768, 24000000)
	}
};

/*--------------------------------------------------------------------------*/
static int gc2023_g_VTS(struct gc_camera_module *cam_mod, u32 *vts)
{
	u32 msb, lsb;
	u32 vb, winv;
	int ret;

	ret = gc_camera_module_read_reg_table(
		cam_mod,
		PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1,
		GC2023_TIMING_VB_HIGH_REG,
		&msb);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = gc_camera_module_read_reg_table(
		cam_mod,
		PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1,
		GC2023_TIMING_VB_LOW_REG,
		&lsb);
	if (IS_ERR_VALUE(ret))
		goto err;

	vb = (msb << 8) | lsb;

	ret = gc_camera_module_read_reg_table(
		cam_mod,
		PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1,
		GC2023_TIMING_WINH_HIGH_REG,
		&msb);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = gc_camera_module_read_reg_table(
		cam_mod,
		PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1,
		GC2023_TIMING_WINH_LOW_REG,
		&lsb);
	if (IS_ERR_VALUE(ret))
		goto err;

	winv = (msb << 8) | lsb;

	*vts = winv + vb + 16;

	return 0;
err:
	gc_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

static int gc2023_auto_adjust_fps(struct gc_camera_module *cam_mod,
	u32 exp_time)
{
	int ret;
	u32 vts;
	u32 vb, winv;
	u32 msb, lsb;

	if ((exp_time + GC2023_COARSE_INTG_TIME_MAX_MARGIN)
		> cam_mod->vts_min)
		vts = exp_time + GC2023_COARSE_INTG_TIME_MAX_MARGIN;
	else
		vts = cam_mod->vts_min;

	if (vts > 0xfff)
		vts = 0xfff;
	else
		vts = vts;

	ret = gc_camera_module_read_reg_table(
		cam_mod,
		PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1,
		GC2023_TIMING_WINH_HIGH_REG,
		&msb);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = gc_camera_module_read_reg_table(
		cam_mod,
		PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1,
		GC2023_TIMING_WINH_LOW_REG,
		&lsb);
	if (IS_ERR_VALUE(ret))
		goto err;

	winv = (msb << 8) | lsb;
	vb = vts - winv - 16;

	ret = gc_camera_module_write_reg(cam_mod,
		PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1,
		GC2023_TIMING_VB_LOW_REG,
		vb & 0xFF);
	ret |= gc_camera_module_write_reg(cam_mod,
		PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1,
		GC2023_TIMING_VB_HIGH_REG,
		(vb >> 8) & 0x0F);

	if (IS_ERR_VALUE(ret)) {
		gc_camera_module_pr_err(cam_mod,
				"failed with error (%d)\n", ret);
	} else {
		gc_camera_module_pr_info(cam_mod,
				"updated vts = 0x%x,vts_min=0x%x\n", vts, cam_mod->vts_min);
		cam_mod->vts_cur = vts;
	}
err:
	return ret;
}

static int gc2023_write_aec(struct gc_camera_module *cam_mod)
{
	int i;
	int ret = 0;

	gc_camera_module_pr_info(cam_mod,
		"exp_time = %d lines, gain = %d, flash_mode = %d\n",
		cam_mod->exp_config.exp_time,
		cam_mod->exp_config.gain,
		cam_mod->exp_config.flash_mode);

	if ((cam_mod->state == GC_CAMERA_MODULE_SW_STANDBY) ||
		(cam_mod->state == GC_CAMERA_MODULE_STREAMING)) {
		u32 a_gain_code, d_gain_code;
		u32 gain = cam_mod->exp_config.gain;
		u32 exp_time = cam_mod->exp_config.exp_time;

		gain = gain * cam_mod->exp_config.gain_percent / 100;

		if (!IS_ERR_VALUE(ret) && cam_mod->auto_adjust_fps)
			ret |= gc2023_auto_adjust_fps(cam_mod,
				cam_mod->exp_config.exp_time);

		for (i = 11; i >= 0; i--) {
			if (GC2023_AGAIN_TBL[i][1] < gain)
				break;
		}

		if (i == 0)
			i = 1;
		if (i > 11)
			i = 11;

		a_gain_code = GC2023_AGAIN_TBL[i][0];
		d_gain_code = gain * 64 / GC2023_AGAIN_TBL[i][1];

		ret |= gc_camera_module_write_reg(cam_mod,
			PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1,
			GC2023_AEC_ANALOG_GAIN_REG, a_gain_code);
		ret |= gc_camera_module_write_reg(cam_mod,
			PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1,
			GC2023_AEC_DIGITAL_INT_GAIN_REG, d_gain_code >> 6);
		ret |= gc_camera_module_write_reg(cam_mod,
			PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1,
			GC2023_AEC_DIGITAL_FRAC_GAIN_REG, (d_gain_code << 2) & 0xFC);

		gc_camera_module_pr_info(cam_mod,
			"gain = 0x%x, a_gain_code = 0x%x, d_gain_code = 0x%x\n",
			gain, a_gain_code, d_gain_code);

		if (exp_time < 1)
			exp_time = 1;
		if (exp_time > 8191)
			exp_time = 8191;

		ret |= gc_camera_module_write_reg(cam_mod,
			PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1,
			GC2023_AEC_EXPO_HIGH_REG,
			GC2023_FETCH_HIGH_BYTE_EXP(exp_time));
		ret |= gc_camera_module_write_reg(cam_mod,
			PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1,
			GC2023_AEC_EXPO_LOW_REG,
			GC2023_FETCH_LOW_BYTE_EXP(exp_time));
	}

	if (IS_ERR_VALUE(ret))
		gc_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);

	return ret;
}

static int gc2023_g_ctrl(struct gc_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	gc_camera_module_pr_debug(cam_mod, "\n");

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
		gc_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int gc2023_filltimings(struct gc_camera_module_custom_config *custom)
{
	int i, j;
	u32 win_h_off = 0, win_v_off = 0;
	u32 vb, hb, winv, winh;
	struct gc_camera_module_config *config;
	struct gc_camera_module_timings *timings;
	struct gc_camera_module_reg *reg_table;
	int reg_table_num_entries;

	for (i = 0; i < custom->num_configs; i++) {
		config = &custom->configs[i];
		reg_table = config->reg_table;
		reg_table_num_entries = config->reg_table_num_entries;
		timings = &config->timings;

		memset(timings, 0x00, sizeof(*timings));
		for (j = 0; j < reg_table_num_entries; j++) {
			switch (reg_table[j].reg) {
			case GC2023_TIMING_VB_HIGH_REG:
				vb = ((reg_table[j].val << 8) |
					 (vb & 0xff));
				break;
			case GC2023_TIMING_VB_LOW_REG:
				vb = (reg_table[j].val |
					 (vb & 0xff00));
				break;
			case GC2023_TIMING_HB_HIGH_REG:
				hb = ((reg_table[j].val << 8) |
					 hb);
				break;
			case GC2023_TIMING_HB_LOW_REG:
				hb = (reg_table[j].val |
					 (hb & 0xff00));
				break;
			case GC2023_TIMING_WINV_HIGH_REG:
				winv = ((reg_table[j].val << 8) |
					   (winv & 0xff));
				break;
			case GC2023_TIMING_WINV_LOW_REG:
				winv = (reg_table[j].val |
					   (winv & 0xff00));
				break;
			case GC2023_TIMING_WINH_HIGH_REG:
				winh = ((reg_table[j].val << 8) |
					   winh);
				break;
			case GC2023_TIMING_WINH_LOW_REG:
				winh = (reg_table[j].val |
					   (winh & 0xff00));
				break;
			case GC2023_H_WIN_OFF_HIGH_REG:
				win_h_off = (reg_table[j].val & 0xf) << 8;
				break;
			case GC2023_H_WIN_OFF_LOW_REG:
				win_h_off |= (reg_table[j].val & 0xff);
				break;
			case GC2023_V_WIN_OFF_HIGH_REG:
				win_v_off = (reg_table[j].val & 0xf) << 8;
				break;
			case GC2023_V_WIN_OFF_LOW_REG:
				win_v_off |= (reg_table[j].val & 0xff);
				break;
			case GC2023_HORIZONTAL_OUTPUT_SIZE_HIGH_REG:
				timings->sensor_output_width =
					((reg_table[j].val << 8) |
					(timings->sensor_output_width & 0xff));
				break;
			case GC2023_HORIZONTAL_OUTPUT_SIZE_LOW_REG:
				timings->sensor_output_width =
					(reg_table[j].val |
					(timings->sensor_output_width & 0xff00));
				break;
			case GC2023_VERTICAL_OUTPUT_SIZE_HIGH_REG:
				timings->sensor_output_height =
					((reg_table[j].val << 8) |
					(timings->sensor_output_height & 0xff));
				break;
			case GC2023_VERTICAL_OUTPUT_SIZE_LOW_REG:
				timings->sensor_output_height =
					(reg_table[j].val |
					(timings->sensor_output_height & 0xff00));
				break;
			}
		}

		/* timings->frame_length_lines = winv + 16 + vb;
		timings->line_length_pck = winh + 202 + hb; */
		timings->frame_length_lines = 1120;
		timings->line_length_pck = 2840;

		timings->crop_horizontal_start = win_h_off;
		timings->crop_horizontal_end = win_h_off + timings->sensor_output_width;
		timings->crop_vertical_start = win_v_off;
		timings->crop_vertical_end = win_v_off + timings->sensor_output_height;

		timings->exp_time >>= 4;
		timings->vt_pix_clk_freq_hz =
			config->frm_intrvl.interval.denominator
			* timings->frame_length_lines
			* timings->line_length_pck;

		timings->coarse_integration_time_min =
			GC2023_COARSE_INTG_TIME_MIN;
		timings->coarse_integration_time_max_margin =
			GC2023_COARSE_INTG_TIME_MAX_MARGIN;

		/* OV Sensor do not use fine integration time. */
		timings->fine_integration_time_min =
			GC2023_FINE_INTG_TIME_MIN;
		timings->fine_integration_time_max_margin =
			GC2023_FINE_INTG_TIME_MAX_MARGIN;

		timings->binning_factor_x = 1;
		timings->binning_factor_y = 1;
	}

	return 0;
}

/*--------------------------------------------------------------------------*/

static int gc2023_g_timings(struct gc_camera_module *cam_mod,
			    struct gc_camera_module_timings *timings)
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

	timings->frame_length_lines = vts;

	return ret;
err:
	gc_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int gc2023_s_ctrl(struct gc_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	gc_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
		ret = gc2023_write_aec(cam_mod);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (IS_ERR_VALUE(ret))
		gc_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int gc2023_s_ext_ctrls(struct gc_camera_module *cam_mod,
				 struct gc_camera_module_ext_ctrls *ctrls)
{
	int ret = 0;

	if ((ctrls->ctrls[0].id == V4L2_CID_GAIN ||
		ctrls->ctrls[0].id == V4L2_CID_EXPOSURE))
		ret = gc2023_write_aec(cam_mod);
	else
		ret = -EINVAL;

	if (IS_ERR_VALUE(ret))
		gc_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);

	return ret;
}

/*--------------------------------------------------------------------------*/
static int gc2023_start_streaming(struct gc_camera_module *cam_mod)
{
	int ret = 0;

	gc_camera_module_pr_info(cam_mod, "active config=%s\n",
		cam_mod->active_config->name);

	ret = gc2023_g_VTS(cam_mod, &cam_mod->vts_min);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = gc_camera_module_write_reg(cam_mod,
		PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1, 0xfe, 0x03);
	ret |= gc_camera_module_write_reg(cam_mod,
		PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1, 0x10, 0x90);
	ret |= gc_camera_module_write_reg(cam_mod,
		PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1, 0xfe, 0x00);

	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	gc_camera_module_pr_err(cam_mod, "failed with error (%d)\n",
		ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int gc2023_stop_streaming(struct gc_camera_module *cam_mod)
{
	int ret = 0;

	gc_camera_module_pr_info(cam_mod, "\n");
	ret = gc_camera_module_write_reg(cam_mod,
		PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1, 0xfe, 0x03);
	ret |= gc_camera_module_write_reg(cam_mod,
		PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1, 0x10, 0x00);
	ret |= gc_camera_module_write_reg(cam_mod,
		PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1, 0xfe, 0x00);

	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	gc_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int gc2023_check_camera_id(struct gc_camera_module *cam_mod)
{
	u32 pidh, pidl;
	int ret = 0;

	gc_camera_module_pr_err(cam_mod, "\n");

	ret |= gc_camera_module_read_reg(cam_mod, 1,
		PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1, GC2023_PIDH_ADDR, &pidh);
	ret |= gc_camera_module_read_reg(cam_mod, 1,
		PLTFRM_CAMERA_MODULE_REG1_TYPE_DATA1, GC2023_PIDL_ADDR, &pidl);
	if (IS_ERR_VALUE(ret)) {
		gc_camera_module_pr_err(cam_mod,
			"register read failed, camera module powered off?\n");
		goto err;
	}

	if ((pidh == GC2023_PIDH_MAGIC) && (pidl == GC2023_PIDL_MAGIC)) {
		gc_camera_module_pr_info(cam_mod,
			"successfully detected camera ID 0x%02x%02x\n",
			pidh, pidl);
	} else {
		gc_camera_module_pr_err(cam_mod,
			"wrong camera ID, expected 0x%02x%02x, detected 0x%02x%02x\n",
			GC2023_PIDH_MAGIC, GC2023_PIDL_MAGIC, pidh, pidl);
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	gc_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

int gc_camera_gc2023_module_s_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	return 0;
}

int gc_camera_gc2023_module_s_ext_ctrls(
	struct v4l2_subdev *sd,
	struct v4l2_ext_controls *ctrls)
{
	return 0;
}

long gc_camera_gc2023_module_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd,
	void *arg)
{
	return 0;
}


/* ======================================================================== */
/* This part is platform dependent */
/* ======================================================================== */
static struct v4l2_subdev_core_ops gc2023_camera_module_core_ops = {
	.g_ctrl = gc_camera_module_g_ctrl,
	.s_ctrl = gc_camera_module_s_ctrl,
	.s_ext_ctrls = gc_camera_module_s_ext_ctrls,
	.s_power = gc_camera_module_s_power,
	.ioctl = gc_camera_module_ioctl
};

static struct v4l2_subdev_video_ops gc2023_camera_module_video_ops = {
	.enum_frameintervals = gc_camera_module_enum_frameintervals,
	.s_mbus_fmt = gc_camera_module_s_fmt,
	.g_mbus_fmt = gc_camera_module_g_fmt,
	.try_mbus_fmt = gc_camera_module_try_fmt,
	.s_frame_interval = gc_camera_module_s_frame_interval,
	.g_frame_interval = gc_camera_module_g_frame_interval,
	.s_stream = gc_camera_module_s_stream
};

static struct v4l2_subdev_ops gc2023_camera_module_ops = {
	.core = &gc2023_camera_module_core_ops,
	.video = &gc2023_camera_module_video_ops
};

static struct gc_camera_module gc2023;

static struct gc_camera_module_custom_config gc2023_custom_config = {
	.start_streaming = gc2023_start_streaming,
	.stop_streaming = gc2023_stop_streaming,
	.s_ctrl = gc2023_s_ctrl,
	.g_ctrl = gc2023_g_ctrl,
	.s_ext_ctrls = gc2023_s_ext_ctrls,
	.g_timings = gc2023_g_timings,
	.check_camera_id = gc2023_check_camera_id,
	.s_vts = gc2023_auto_adjust_fps,
	.configs = gc2023_configs,
	.num_configs = sizeof(gc2023_configs) / sizeof(gc2023_configs[0]),
	.power_up_delays_ms = {5, 30, 30},
	.exposure_valid_frame = {4, 4}
};

static int gc2023_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	dev_info(&client->dev, "probing...\n");

	gc2023_filltimings(&gc2023_custom_config);

	v4l2_i2c_subdev_init(&gc2023.sd, client,
				&gc2023_camera_module_ops);
	gc2023.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	gc2023.custom = gc2023_custom_config;

	dev_info(&client->dev, "probing successful\n");
	return 0;
}

/* ======================================================================== */
static int gc2023_remove(
	struct i2c_client *client)
{
	struct gc_camera_module *cam_mod = i2c_get_clientdata(client);

	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	gc_camera_module_release(cam_mod);

	dev_info(&client->dev, "removed\n");
	return 0;
}

static const struct i2c_device_id gc2023_id[] = {
	{ GC2023_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id gc2023_of_match[] = {
	{.compatible = "galaxycore,gc2023-v4l2-i2c-subdev"},
	{},
};

MODULE_DEVICE_TABLE(i2c, gc2023_id);

static struct i2c_driver gc2023_i2c_driver = {
	.driver = {
		.name = GC2023_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gc2023_of_match
	},
	.probe = gc2023_probe,
	.remove = gc2023_remove,
	.id_table = gc2023_id,
};

module_i2c_driver(gc2023_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for gc2023");
MODULE_AUTHOR("Eike Grimpe");
MODULE_LICENSE("GPL");

