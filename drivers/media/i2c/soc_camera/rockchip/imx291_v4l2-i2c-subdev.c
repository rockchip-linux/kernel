/*
 * IMX291 sensor driver
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
#include "imx_camera_module.h"

#define IMX291_DRIVER_NAME "imx291"

#define IMX291_AEC_PK_GAIN_REG 0x3014

#define IMX291_AEC_PK_EXPO_HIGH_REG 0x3022
#define IMX291_AEC_PK_EXPO_MID_REG 0x3021
#define IMX291_AEC_PK_EXPO_LOW_REG  0x3020

#define IMX291_FETCH_HIGH_BYTE_EXP(VAL) (((VAL) >> 16) & 0x03)
#define IMX291_FETCH_MID_BYTE_EXP(VAL) (((VAL) >> 8) & 0xFF)
#define IMX291_FETCH_LOW_BYTE_EXP(VAL) ((VAL) & 0xFF)

#define IMX291_PID_ADDR 0x3008
#define IMX291_PID_MAGIC 0xA0

#define IMX291_MIRROR_FLIP_REG 0x3007
#define IMX291_ORIENTATION_V 0x1
#define IMX291_ORIENTATION_H 0x2

#define IMX291_TIMING_VTS_HIGH_REG 0x3019
#define IMX291_TIMING_VTS_LOW_REG 0x3018
#define IMX291_TIMING_HTS_HIGH_REG 0x301D
#define IMX291_TIMING_HTS_LOW_REG 0x301C

#define IMX291_INTEGRATION_TIME_MARGIN 8
#define IMX291_FINE_INTG_TIME_MIN 0
#define IMX291_FINE_INTG_TIME_MAX_MARGIN 0
#define IMX291_COARSE_INTG_TIME_MIN 16
#define IMX291_COARSE_INTG_TIME_MAX_MARGIN 4

#define IMX291_EXT_CLK 37125000

static struct imx_camera_module imx291;
static struct imx_camera_module_custom_config imx291_custom_config;

/* ======================================================================== */
/* Base sensor configs */
/* ======================================================================== */

/* MCLK:37.125MHz  1920x1080  30fps   2Lane   445.5Mbps/Lane */
static struct imx_camera_module_reg imx291_init_tab_1920_1080_30fps[] = {
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3003, 0x01},
	{IMX_CAMERA_MODULE_REG_TYPE_TIMEOUT, 0x0000, 0x10},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3000, 0x01},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3002, 0x00},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3005, 0x01},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3007, 0x00},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3009, 0x02},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x300a, 0xf0},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x300f, 0x00},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3010, 0x21},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3012, 0x64},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3016, 0x09},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3019, 0x04},/* VMAX H */
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3018, 0x65},/* VMAX L */
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x301d, 0x11},/* HMAX H */
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x301c, 0x30},/* HMAX L */
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3046, 0x01},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x304b, 0x0a},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x305c, 0x18},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x305d, 0x03},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x305e, 0x20},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x305f, 0x01},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3070, 0x02},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3071, 0x11},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x309b, 0x10},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x309c, 0x22},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x30a2, 0x02},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x30a6, 0x20},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x30a8, 0x20},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x30aa, 0x20},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x30ac, 0x20},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x30b0, 0x43},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3119, 0x9e},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x311c, 0x1e},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x311e, 0x08},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3128, 0x05},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3129, 0x00},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x313d, 0x83},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3150, 0x03},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x315e, 0x1a},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3164, 0x1a},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x317c, 0x00},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x317e, 0x00},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x31ec, 0x0e},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x32b8, 0x50},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x32b9, 0x10},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x32ba, 0x00},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x32bb, 0x04},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x32c8, 0x50},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x32c9, 0x10},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x32ca, 0x00},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x32cb, 0x04},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x332c, 0xd3},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x332d, 0x10},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x332e, 0x0d},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3358, 0x06},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3359, 0xe1},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x335a, 0x11},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3360, 0x1e},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3361, 0x61},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3362, 0x10},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x33b0, 0x50},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x33b2, 0x1a},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x33b3, 0x04},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3405, 0x10},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3407, 0x01},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3414, 0x0a},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3418, 0x49},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3419, 0x04},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3441, 0x0c},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3442, 0x0c},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3443, 0x01},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3444, 0x20},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3445, 0x25},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3446, 0x57},/* TCLKPOST */
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3447, 0x00},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3448, 0x55},/* THSZERO 0x37 */
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3449, 0x00},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x344a, 0x1f},/* THSPREPARE */
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x344b, 0x00},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x344c, 0x1f},/* TCLKRAIL */
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x344d, 0x00},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x344e, 0x1f},/* THSTRAIL */
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x344f, 0x00},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3450, 0x77},/* TCLKZERO */
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3451, 0x00},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3452, 0x1f},/* TCLKPREPARE */
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3453, 0x00},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3454, 0x17},/* TLPX */
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3455, 0x00},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3472, 0x9c},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3473, 0x07},
	{IMX_CAMERA_MODULE_REG_TYPE_DATA_SINGLE, 0x3480, 0x49}
};

/* ======================================================================== */

static struct imx_camera_module_config imx291_configs[] = {
	{
		.name = "1920x1080_30fps",
		.frm_fmt = {
			.width = 1920,
			.height = 1080,
			.code = V4L2_MBUS_FMT_SRGGB12_1X12
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
		.reg_table = (void *)imx291_init_tab_1920_1080_30fps,
		.reg_table_num_entries =
			ARRAY_SIZE(imx291_init_tab_1920_1080_30fps),
		.v_blanking_time_us = 5000,
		PLTFRM_CAM_ITF_MIPI_CFG(0, 2, 446, IMX291_EXT_CLK)
	}
};

/*--------------------------------------------------------------------------*/

static int imx291_g_vts(struct imx_camera_module *cam_mod, u32 *vts)
{
	u32 msb, lsb;
	int ret;

	ret = imx_camera_module_read_reg_table(cam_mod,
		IMX291_TIMING_VTS_HIGH_REG, &msb);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = imx_camera_module_read_reg_table(cam_mod,
		IMX291_TIMING_VTS_LOW_REG, &lsb);
	if (IS_ERR_VALUE(ret))
		goto err;

	*vts = (msb << 8) | lsb;

	return 0;
err:
	imx_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int imx291_auto_adjust_fps(struct imx_camera_module *cam_mod,
	u32 exp_time)
{
	int ret;
	u32 vts;

	if ((exp_time + IMX291_COARSE_INTG_TIME_MAX_MARGIN)
		> cam_mod->vts_min)
		vts = exp_time + IMX291_COARSE_INTG_TIME_MAX_MARGIN;
	else
		vts = cam_mod->vts_min;
	ret = imx_camera_module_write_reg(cam_mod,
		IMX291_TIMING_VTS_LOW_REG, vts & 0xFF);
	ret |= imx_camera_module_write_reg(cam_mod,
		IMX291_TIMING_VTS_HIGH_REG, (vts >> 8) & 0xFF);

	if (IS_ERR_VALUE(ret)) {
		imx_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	} else {
		imx_camera_module_pr_debug(cam_mod,
			"updated vts = %d,vts_min=%d\n", vts, cam_mod->vts_min);
		cam_mod->vts_cur = vts;
	}

	return ret;
}

/*--------------------------------------------------------------------------*/

static int imx291_write_aec(struct imx_camera_module *cam_mod)
{
	int ret = 0;

	imx_camera_module_pr_debug(cam_mod,
		"exp_time = %d, gain = %d, flash_mode = %d\n",
		cam_mod->exp_config.exp_time,
		cam_mod->exp_config.gain,
		cam_mod->exp_config.flash_mode);

	/*
	 * if the sensor is already streaming, write to shadow registers,
	 * if the sensor is in SW standby, write to active registers,
	 * if the sensor is off/registers are not writeable, do nothing
	 */
	if ((cam_mod->state == IMX_CAMERA_MODULE_SW_STANDBY) ||
		(cam_mod->state == IMX_CAMERA_MODULE_STREAMING)) {
		u32 a_gain = cam_mod->exp_config.gain;
		u32 exp_time = cam_mod->exp_config.exp_time;
		a_gain = a_gain * cam_mod->exp_config.gain_percent / 100;

		if (exp_time < 1)
			exp_time = 1;

		if (!IS_ERR_VALUE(ret) && cam_mod->auto_adjust_fps)
			ret = imx291_auto_adjust_fps(cam_mod,
				cam_mod->exp_config.exp_time);

		/* Gain */
		ret = imx_camera_module_write_reg(cam_mod,
			IMX291_AEC_PK_GAIN_REG, a_gain);

		/* Integration Time */
		ret = imx_camera_module_write_reg(cam_mod,
			IMX291_AEC_PK_EXPO_HIGH_REG,
			IMX291_FETCH_HIGH_BYTE_EXP(exp_time));
		ret |= imx_camera_module_write_reg(cam_mod,
			IMX291_AEC_PK_EXPO_MID_REG,
			IMX291_FETCH_MID_BYTE_EXP(exp_time));
		ret |= imx_camera_module_write_reg(cam_mod,
			IMX291_AEC_PK_EXPO_LOW_REG,
			IMX291_FETCH_LOW_BYTE_EXP(exp_time));
	}

	if (IS_ERR_VALUE(ret))
		imx_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int imx291_g_ctrl(struct imx_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	imx_camera_module_pr_debug(cam_mod, "\n");

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
		imx_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int imx291_filltimings(struct imx_camera_module_custom_config *custom)
{
	int i, j;
	struct imx_camera_module_config *config;
	struct imx_camera_module_timings *timings;
	struct imx_camera_module_reg *reg_table;
	int reg_table_num_entries;

	for (i = 0; i < custom->num_configs; i++) {
		config = &custom->configs[i];
		reg_table = config->reg_table;
		reg_table_num_entries = config->reg_table_num_entries;
		timings = &config->timings;

		for (j = 0; j < reg_table_num_entries; j++) {
			switch (reg_table[j].reg) {
			case IMX291_TIMING_VTS_HIGH_REG:
				timings->frame_length_lines = reg_table[j].val << 8;
				break;
			case IMX291_TIMING_VTS_LOW_REG:
				timings->frame_length_lines |= reg_table[j].val;
				break;
			case IMX291_TIMING_HTS_HIGH_REG:
				timings->line_length_pck = (reg_table[j].val << 8);
				break;
			case IMX291_TIMING_HTS_LOW_REG:
				timings->line_length_pck |= reg_table[j].val;
				break;
			}
		}

		timings->line_length_pck = timings->line_length_pck >> 1;
		timings->vt_pix_clk_freq_hz = config->frm_intrvl.interval.denominator
			* timings->frame_length_lines
			* timings->line_length_pck;

		timings->coarse_integration_time_min = IMX291_COARSE_INTG_TIME_MIN;
		timings->coarse_integration_time_max_margin = IMX291_COARSE_INTG_TIME_MAX_MARGIN;

		/* IMX Sensor do not use fine integration time. */
		timings->fine_integration_time_min = IMX291_FINE_INTG_TIME_MIN;
		timings->fine_integration_time_max_margin = IMX291_FINE_INTG_TIME_MAX_MARGIN;
	}

	return 0;
}

static int imx291_g_timings(struct imx_camera_module *cam_mod,
	struct imx_camera_module_timings *timings)
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
	imx_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int imx291_set_flip(
	struct imx_camera_module *cam_mod,
	struct pltfrm_camera_module_reg reglist[],
	int len)
{
	int i, mode = 0;
	u32 orientation = 0;

	mode = imx_camera_module_get_flip_mirror(cam_mod);
	if (mode == -1) {
		imx_camera_module_pr_info(cam_mod,
			"dts don't set flip, return!\n");
		return 0;
	}

	if (!IS_ERR_OR_NULL(cam_mod->active_config)) {
		imx_camera_module_read_reg_table(cam_mod,
			IMX291_MIRROR_FLIP_REG, &orientation);
		orientation &= 0xfc;
		if (PLTFRM_CAMERA_MODULE_IS_MIRROR(mode))
			orientation |= IMX291_ORIENTATION_V;
		if (PLTFRM_CAMERA_MODULE_IS_FLIP(mode))
			orientation |= IMX291_ORIENTATION_H;
		for (i = 0; i < len; i++) {
			if (reglist[i].reg == IMX291_MIRROR_FLIP_REG)
				reglist[i].val = orientation;
		}
	}

	return 0;
}

/*--------------------------------------------------------------------------*/

static int imx291_s_ctrl(struct imx_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	imx_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
		ret = imx291_write_aec(cam_mod);
		break;
	case V4L2_CID_FLASH_LED_MODE:
		/* nothing to be done here */
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (IS_ERR_VALUE(ret))
		imx_camera_module_pr_debug(cam_mod,
			"failed with error (%d) 0x%x\n", ret, ctrl_id);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int imx291_s_ext_ctrls(struct imx_camera_module *cam_mod,
	struct imx_camera_module_ext_ctrls *ctrls)
{
	int ret = 0;

	/* Handles only exposure and gain together special case. */
	if ((ctrls->ctrls[0].id == V4L2_CID_GAIN ||
		ctrls->ctrls[0].id == V4L2_CID_EXPOSURE))
		ret = imx291_write_aec(cam_mod);
	else
		ret = -EINVAL;

	if (IS_ERR_VALUE(ret))
		imx_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);

	return ret;
}

/*--------------------------------------------------------------------------*/

static int imx291_start_streaming(struct imx_camera_module *cam_mod)
{
	int ret = 0;

	imx_camera_module_pr_info(cam_mod,
		"active config=%s\n", cam_mod->active_config->name);

	ret = imx291_g_vts(cam_mod, &cam_mod->vts_min);
	if (IS_ERR_VALUE(ret))
		goto err;

	if (IS_ERR_VALUE(imx_camera_module_write_reg(cam_mod, 0x3000, 0)))
		goto err;

	msleep(25);

	return 0;
err:
	imx_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int imx291_stop_streaming(struct imx_camera_module *cam_mod)
{
	int ret = 0;

	imx_camera_module_pr_info(cam_mod, "\n");

	ret = imx_camera_module_write_reg(cam_mod, 0x3000, 1);
	if (IS_ERR_VALUE(ret))
		goto err;

	msleep(25);

	return 0;
err:
	imx_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int imx291_check_camera_id(struct imx_camera_module *cam_mod)
{
	u32 pid;
	int ret = 0;

	ret |= imx_camera_module_read_reg(cam_mod, 1, IMX291_PID_ADDR, &pid);

	if (IS_ERR_VALUE(ret)) {
		imx_camera_module_pr_err(cam_mod,
			"register read failed, camera module powered off?\n");
		goto err;
	}

	if (pid == IMX291_PID_MAGIC) {
		imx_camera_module_pr_info(cam_mod,
			"successfully detected camera ID 0x%02x\n",
			pid);
	} else {
		imx_camera_module_pr_err(cam_mod,
			"wrong camera ID, expected 0x%02x, detected 0x%02x\n",
			IMX291_PID_MAGIC, pid);
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	imx_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/* ======================================================================== */
/* This part is platform dependent */
/* ======================================================================== */

static struct v4l2_subdev_core_ops imx291_camera_module_core_ops = {
	.g_ctrl = imx_camera_module_g_ctrl,
	.s_ctrl = imx_camera_module_s_ctrl,
	.s_ext_ctrls = imx_camera_module_s_ext_ctrls,
	.s_power = imx_camera_module_s_power,
	.ioctl = imx_camera_module_ioctl
};

static struct v4l2_subdev_video_ops imx291_camera_module_video_ops = {
	.enum_frameintervals = imx_camera_module_enum_frameintervals,
	.s_mbus_fmt = imx_camera_module_s_fmt,
	.g_mbus_fmt = imx_camera_module_g_fmt,
	.try_mbus_fmt = imx_camera_module_try_fmt,
	.s_frame_interval = imx_camera_module_s_frame_interval,
	.g_frame_interval = imx_camera_module_g_frame_interval,
	.s_stream = imx_camera_module_s_stream
};

static struct v4l2_subdev_ops imx291_camera_module_ops = {
	.core = &imx291_camera_module_core_ops,
	.video = &imx291_camera_module_video_ops
};

static struct imx_camera_module_custom_config imx291_custom_config = {
	.start_streaming = imx291_start_streaming,
	.stop_streaming = imx291_stop_streaming,
	.s_ctrl = imx291_s_ctrl,
	.s_ext_ctrls = imx291_s_ext_ctrls,
	.g_ctrl = imx291_g_ctrl,
	.g_timings = imx291_g_timings,
	.check_camera_id = imx291_check_camera_id,
	.set_flip = imx291_set_flip,
	.s_vts = imx291_auto_adjust_fps,
	.configs = imx291_configs,
	.num_configs = ARRAY_SIZE(imx291_configs),
	.power_up_delays_ms = {5, 20, 0},
	/*
	 * 0: Exposure time valid fileds;
	 * 1: Exposure gain valid fileds;
	 * (2 fileds == 1 frames)
	 */
	.exposure_valid_frame = {4, 4}
};

static int imx291_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	dev_info(&client->dev, "probing...\n");

	imx291_filltimings(&imx291_custom_config);
	v4l2_i2c_subdev_init(&imx291.sd, client, &imx291_camera_module_ops);
	imx291.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx291.custom = imx291_custom_config;

	dev_info(&client->dev, "probing successful\n");
	return 0;
}

static int imx291_remove(struct i2c_client *client)
{
	struct imx_camera_module *cam_mod = i2c_get_clientdata(client);

	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;

	imx_camera_module_release(cam_mod);

	dev_info(&client->dev, "removed\n");
	return 0;
}

static const struct i2c_device_id imx291_id[] = {
	{ IMX291_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id imx291_of_match[] = {
	{.compatible = "sony,imx291-v4l2-i2c-subdev"},
	{},
};

MODULE_DEVICE_TABLE(i2c, imx291_id);

static struct i2c_driver imx291_i2c_driver = {
	.driver = {
		.name = IMX291_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = imx291_of_match
	},
	.probe = imx291_probe,
	.remove = imx291_remove,
	.id_table = imx291_id,
};

module_i2c_driver(imx291_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for IMX291");
MODULE_AUTHOR("CAIN.CAI");
MODULE_LICENSE("GPL");
