/*
 * drivers/media/i2c/soc_camera/rockchip/ov7750_v4l2-i2c-subdev.c
 *
 * ov7750 sensor driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * Copyright (C) 2012-2014 Intel Mobile Communications GmbH
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-core.h>
#include <linux/slab.h>
#include "ov_camera_module.h"

/*****************************************************************************
 * DEFINES
 *****************************************************************************/
#define ov7750_DRIVER_NAME "ov7750"

#define ov7750_FETCH_LSB_GAIN(VAL) ((VAL) & 0x00ff)
#define ov7750_FETCH_MSB_GAIN(VAL) (((VAL) >> 8) & 0xff)
#define ov7750_AEC_PK_LONG_GAIN_HIGH_REG 0x3508	/* Bit 6-13 */
#define ov7750_AEC_PK_LONG_GAIN_LOW_REG	 0x3509	/* Bits 0 -5 */

#define ov7750_AEC_PK_LONG_EXPO_3RD_REG 0x3500	/* Exposure Bits 16-19 */
#define ov7750_AEC_PK_LONG_EXPO_2ND_REG 0x3501	/* Exposure Bits 8-15 */
#define ov7750_AEC_PK_LONG_EXPO_1ST_REG 0x3502	/* Exposure Bits 0-7 */

#define ov7750_AEC_GROUP_UPDATE_ADDRESS 0x3208
#define ov7750_AEC_GROUP_UPDATE_START_DATA 0x00
#define ov7750_AEC_GROUP_UPDATE_END_DATA 0x10
#define ov7750_AEC_GROUP_UPDATE_END_LAUNCH 0xA0

#define ov7750_FETCH_3RD_BYTE_EXP(VAL) (((VAL) >> 16) & 0xF)	/* 4 Bits */
#define ov7750_FETCH_2ND_BYTE_EXP(VAL) (((VAL) >> 8) & 0xFF)	/* 8 Bits */
#define ov7750_FETCH_1ST_BYTE_EXP(VAL) ((VAL) & 0xFF)	/* 8 Bits */

#define ov7750_PIDH_ADDR     0x300A
#define ov7750_PIDL_ADDR     0x300B

#define ov7750_TIMING_VTS_HIGH_REG 0x380e
#define ov7750_TIMING_VTS_LOW_REG 0x380f
#define ov7750_TIMING_HTS_HIGH_REG 0x380c
#define ov7750_TIMING_HTS_LOW_REG 0x380d
#define ov7750_INTEGRATION_TIME_MARGIN 8
#define ov7750_FINE_INTG_TIME_MIN 0
#define ov7750_FINE_INTG_TIME_MAX_MARGIN 0
#define ov7750_COARSE_INTG_TIME_MIN 16
#define ov7750_COARSE_INTG_TIME_MAX_MARGIN 4
#define ov7750_TIMING_X_INC		0x3814
#define ov7750_TIMING_Y_INC		0x3815
#define ov7750_HORIZONTAL_START_HIGH_REG 0x3800
#define ov7750_HORIZONTAL_START_LOW_REG 0x3801
#define ov7750_VERTICAL_START_HIGH_REG 0x3802
#define ov7750_VERTICAL_START_LOW_REG 0x3803
#define ov7750_HORIZONTAL_END_HIGH_REG 0x3804
#define ov7750_HORIZONTAL_END_LOW_REG 0x3805
#define ov7750_VERTICAL_END_HIGH_REG 0x3806
#define ov7750_VERTICAL_END_LOW_REG 0x3807
#define ov7750_HORIZONTAL_OUTPUT_SIZE_HIGH_REG 0x3808
#define ov7750_HORIZONTAL_OUTPUT_SIZE_LOW_REG 0x3809
#define ov7750_VERTICAL_OUTPUT_SIZE_HIGH_REG 0x380a
#define ov7750_VERTICAL_OUTPUT_SIZE_LOW_REG 0x380b
#define ov7750_FLIP_REG                      0x3820
#define ov7750_MIRROR_REG                      0x3821

#define ov7750_EXT_CLK 26000000
#define ov7750_FULL_SIZE_RESOLUTION_WIDTH 3264
#define ov7750_BINING_SIZE_RESOLUTION_WIDTH 1632
#define ov7750_VIDEO_SIZE_RESOLUTION_WIDTH 3200

#define ov7750_EXP_VALID_FRAMES		4
/* High byte of product ID */
#define ov7750_PIDH_MAGIC 0x56
/* Low byte of product ID  */
#define ov7750_PIDL_MAGIC 0x47

#define BG_RATIO_TYPICAL  0x129
#define RG_RATIO_TYPICAL  0x11f

static struct ov_camera_module ov7750;
static struct ov_camera_module_custom_config ov7750_custom_config;

/*****************************************************************************
 * GLOBALS
 *****************************************************************************/

// Image sensor register settings default values taken from
// data sheet OV7750A_DS_1.1_SiliconImage.pdf.

// The settings may be altered by the code in IsiSetupSensor.
static struct ov_camera_module_reg ov7750_init_tab_640_480_60fps[] = {
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3000, 0x0f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3001, 0xff},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3002, 0xe4},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0100, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x0103, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3034, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3035, 0x21},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3036, 0x46},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x303c, 0x11},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3106, 0xf5},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3821, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3820, 0x41},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3827, 0xec},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370c, 0x0f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3612, 0x59},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3618, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5000, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5001, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5002, 0x41},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5003, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x5a00, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3000, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3001, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3002, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3016, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3017, 0xe0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3018, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x301c, 0xf8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x301d, 0xf0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a18, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a19, 0xf8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3c01, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3b07, 0x0c},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380c, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380d, 0x68},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380e, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380f, 0xd8},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3814, 0x31},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3815, 0x31},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3708, 0x64},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3709, 0x52},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3808, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3809, 0x80},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380a, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x380b, 0xE0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3801, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3802, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3803, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3804, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3805, 0x3f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3806, 0x07},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3807, 0xa1},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3811, 0x08},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3813, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3630, 0x2e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3632, 0xe2},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3633, 0x23},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3634, 0x44},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3636, 0x06},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3620, 0x64},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3621, 0xe0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3600, 0x37},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3704, 0xa0},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3703, 0x5a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3715, 0x78},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3717, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3731, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x370b, 0x60},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3705, 0x1a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3f05, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3f06, 0x10},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3f01, 0x0a},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a08, 0x01},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a09, 0x27},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0a, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0b, 0xf6},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0d, 0x04},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0e, 0x03},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a0f, 0x58},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a10, 0x50},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a1b, 0x58},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a1e, 0x50},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a11, 0x60},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x3a1f, 0x28},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4001, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4004, 0x02},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4000, 0x09},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4837, 0x24},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4050, 0x6e},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4051, 0x8f},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4202, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x300D, 0x00},
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 0x4800, 0x25},
};

static struct ov_camera_module_config ov7750_configs[] = {
	{OV_CAMERA_MODULE_REG_TYPE_DATA, 
		.name = "640x480_60fps",
		.frm_fmt = {
			.width = 640,
			.height = 480,
			.code = MEDIA_BUS_FMT_SBGGR8_1X8
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 60
			}
		},
		.auto_exp_enabled = false,
		.auto_gain_enabled = false,
		.auto_wb_enabled = false,
		.reg_table = (void *)ov7750_init_tab_640_480_60fps,
		.reg_table_num_entries =
			sizeof(ov7750_init_tab_640_480_60fps) /
			sizeof(ov7750_init_tab_640_480_60fps[0]),
		.reg_diff_table = NULL,
		.reg_diff_table_num_entries = 0,
		.v_blanking_time_us = 7251,
		PLTFRM_CAM_ITF_MIPI_CFG(0, 2, 800, 24000000)
	}
};

static int ov7750_g_VTS(struct ov_camera_module *cam_mod, u32 *vts)
{
	u32 msb, lsb;
	int ret;

	ret = ov_camera_module_read_reg_table(
		cam_mod,
		ov7750_TIMING_VTS_HIGH_REG,
		&msb);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = ov_camera_module_read_reg_table(
		cam_mod,
		ov7750_TIMING_VTS_LOW_REG,
		&lsb);
	if (IS_ERR_VALUE(ret))
		goto err;

	*vts = (msb << 8) | lsb;
	cam_mod->vts_cur = *vts;

	return 0;
err:
	ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

static int ov7750_g_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
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

static int ov7750_filltimings(struct ov_camera_module_custom_config *custom)
{
	int i, j;
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
			case ov7750_TIMING_VTS_HIGH_REG:
				timings->frame_length_lines =
					((reg_table[j].val << 8) |
					(timings->frame_length_lines & 0xff));
				break;
			case ov7750_TIMING_VTS_LOW_REG:
				timings->frame_length_lines =
					(reg_table[j].val |
					(timings->frame_length_lines & 0xff00));
				break;
			case ov7750_TIMING_HTS_HIGH_REG:
				timings->line_length_pck =
					((reg_table[j].val << 8) |
					timings->line_length_pck);
				break;
			case ov7750_TIMING_HTS_LOW_REG:
				timings->line_length_pck =
					(reg_table[j].val |
					(timings->line_length_pck & 0xff00));
				break;
			case ov7750_TIMING_X_INC:
				timings->binning_factor_x =
				((reg_table[j].val >> 4) + 1) / 2;
				if (timings->binning_factor_x == 0)
					timings->binning_factor_x = 1;
				break;
			case ov7750_TIMING_Y_INC:
				timings->binning_factor_y =
				((reg_table[j].val >> 4) + 1) / 2;
				if (timings->binning_factor_y == 0)
					timings->binning_factor_y = 1;
				break;
			case ov7750_HORIZONTAL_START_HIGH_REG:
				timings->crop_horizontal_start =
					((reg_table[j].val << 8) |
					(timings->crop_horizontal_start &
					0xff));
				break;
			case ov7750_HORIZONTAL_START_LOW_REG:
				timings->crop_horizontal_start =
					(reg_table[j].val |
					(timings->crop_horizontal_start &
					0xff00));
				break;
			case ov7750_VERTICAL_START_HIGH_REG:
				timings->crop_vertical_start =
					((reg_table[j].val << 8) |
					(timings->crop_vertical_start & 0xff));
				break;
			case ov7750_VERTICAL_START_LOW_REG:
				timings->crop_vertical_start =
					((reg_table[j].val) |
					(timings->crop_vertical_start &
					0xff00));
				break;
			case ov7750_HORIZONTAL_END_HIGH_REG:
				timings->crop_horizontal_end =
					((reg_table[j].val << 8) |
					(timings->crop_horizontal_end & 0xff));
				break;
			case ov7750_HORIZONTAL_END_LOW_REG:
				timings->crop_horizontal_end =
					(reg_table[j].val |
					(timings->crop_horizontal_end &
					0xff00));
				break;
			case ov7750_VERTICAL_END_HIGH_REG:
				timings->crop_vertical_end =
					((reg_table[j].val << 8) |
					(timings->crop_vertical_end & 0xff));
				break;
			case ov7750_VERTICAL_END_LOW_REG:
				timings->crop_vertical_end =
					(reg_table[j].val |
					(timings->crop_vertical_end & 0xff00));
				break;
			case ov7750_HORIZONTAL_OUTPUT_SIZE_HIGH_REG:
				timings->sensor_output_width =
					((reg_table[j].val << 8) |
					(timings->sensor_output_width & 0xff));
				break;
			case ov7750_HORIZONTAL_OUTPUT_SIZE_LOW_REG:
				timings->sensor_output_width =
					(reg_table[j].val |
					(timings->sensor_output_width &
					0xff00));
				break;
			case ov7750_VERTICAL_OUTPUT_SIZE_HIGH_REG:
				timings->sensor_output_height =
					((reg_table[j].val << 8) |
					(timings->sensor_output_height & 0xff));
				break;
			case ov7750_VERTICAL_OUTPUT_SIZE_LOW_REG:
				timings->sensor_output_height =
					(reg_table[j].val |
					(timings->sensor_output_height &
					0xff00));
				break;
			case ov7750_AEC_PK_LONG_EXPO_1ST_REG:
				timings->exp_time =
					((reg_table[j].val) |
					(timings->exp_time & 0xffff00));
				break;
			case ov7750_AEC_PK_LONG_EXPO_2ND_REG:
				timings->exp_time =
					((reg_table[j].val << 8) |
					(timings->exp_time & 0x00ff00));
				break;
			case ov7750_AEC_PK_LONG_EXPO_3RD_REG:
				timings->exp_time =
					(((reg_table[j].val & 0x0f) << 16) |
					(timings->exp_time & 0xff0000));
				break;
			case ov7750_AEC_PK_LONG_GAIN_LOW_REG:
				timings->gain =
					(reg_table[j].val |
					(timings->gain & 0x0700));
				break;
			case ov7750_AEC_PK_LONG_GAIN_HIGH_REG:
				timings->gain =
					(((reg_table[j].val & 0x07) << 8) |
					(timings->gain & 0xff));
				break;
			}
		}

		timings->exp_time >>= 4;
		timings->vt_pix_clk_freq_hz =
			config->frm_intrvl.interval.denominator
			* timings->frame_length_lines
			* timings->line_length_pck;

		timings->coarse_integration_time_min =
			ov7750_COARSE_INTG_TIME_MIN;
		timings->coarse_integration_time_max_margin =
			ov7750_COARSE_INTG_TIME_MAX_MARGIN;

		/* OV Sensor do not use fine integration time. */
		timings->fine_integration_time_min =
			ov7750_FINE_INTG_TIME_MIN;
		timings->fine_integration_time_max_margin =
			ov7750_FINE_INTG_TIME_MAX_MARGIN;
	}

	return 0;
}

static int ov7750_g_timings(struct ov_camera_module *cam_mod,
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
			cam_mod->frm_intrvl.interval.denominator *
			vts * timings->line_length_pck;
	else
		timings->vt_pix_clk_freq_hz =
		cam_mod->active_config->frm_intrvl.interval.denominator *
		vts * timings->line_length_pck;

	return ret;
err:
	ov_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

static int ov7750_s_ctrl(struct ov_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_FLASH_LED_MODE:
		/* nothing to be done here */
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_debug(cam_mod,
			"failed with error (%d) 0x%x\n", ret, ctrl_id);
	return ret;
}

static int ov7750_s_ext_ctrls(struct ov_camera_module *cam_mod,
				 struct ov_camera_module_ext_ctrls *ctrls)
{
	int ret = 0;

	if (IS_ERR_VALUE(ret))
		ov_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

static int ov7750_start_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod,
		"active config=%s\n", cam_mod->active_config->name);

	ret = ov7750_g_VTS(cam_mod, &cam_mod->vts_min);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = ov_camera_module_write_reg(cam_mod, 0x4800, 0x04);
	if (IS_ERR_VALUE(ret))
		goto err;


	if (IS_ERR_VALUE(ov_camera_module_write_reg(cam_mod, 0x0100, 1)))
		goto err;

	msleep(25);

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n",
		ret);
	return ret;
}

static int ov7750_stop_streaming(struct ov_camera_module *cam_mod)
{
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret = ov_camera_module_write_reg(cam_mod, 0x4800, 0x25);
	if (IS_ERR_VALUE(ret))
		goto err;
	

	ret = ov_camera_module_write_reg(cam_mod, 0x0100, 0);
	if (IS_ERR_VALUE(ret))
		goto err;

	msleep(25);

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

static int ov7750_check_camera_id(struct ov_camera_module *cam_mod)
{
	u32 pidh, pidl;
	int ret = 0;

	ov_camera_module_pr_debug(cam_mod, "\n");

	ret |= ov_camera_module_read_reg(cam_mod, 1, ov7750_PIDH_ADDR, &pidh);
	ret |= ov_camera_module_read_reg(cam_mod, 1, ov7750_PIDL_ADDR, &pidl);
	if (IS_ERR_VALUE(ret)) {
		ov_camera_module_pr_err(cam_mod,
			"register read failed, camera module powered off?\n");
		goto err;
	}

	if ((pidh == ov7750_PIDH_MAGIC) && (pidl == ov7750_PIDL_MAGIC)) {
		ov_camera_module_pr_err(cam_mod,
			"successfully detected camera ID 0x%02x%02x\n",
			pidh, pidl);
	} else {
		ov_camera_module_pr_err(cam_mod,
			"wrong camera ID, expected 0x%02x%02x, detected 0x%02x%02x\n",
			ov7750_PIDH_MAGIC, ov7750_PIDL_MAGIC, pidh, pidl);
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	ov_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/* ======================================================================== */
/* This part is platform dependent */
/* ======================================================================== */

static struct v4l2_subdev_core_ops ov7750_camera_module_core_ops = {
	.g_ctrl = ov_camera_module_g_ctrl,
	.s_ctrl = ov_camera_module_s_ctrl,
	.s_ext_ctrls = ov_camera_module_s_ext_ctrls,
	.s_power = ov_camera_module_s_power,
	.ioctl = ov_camera_module_ioctl
};

static struct v4l2_subdev_video_ops ov7750_camera_module_video_ops = {
	.s_frame_interval = ov_camera_module_s_frame_interval,
	.s_stream = ov_camera_module_s_stream
};

static struct v4l2_subdev_pad_ops ov7750_camera_module_pad_ops = {
	.enum_frame_interval = ov_camera_module_enum_frameintervals,
	.get_fmt = ov_camera_module_g_fmt,
	.set_fmt = ov_camera_module_s_fmt,
};

static struct v4l2_subdev_ops ov7750_camera_module_ops = {
	.core = &ov7750_camera_module_core_ops,
	.video = &ov7750_camera_module_video_ops,
	.pad = &ov7750_camera_module_pad_ops
};

static struct ov_camera_module_custom_config ov7750_custom_config = {
	.start_streaming = ov7750_start_streaming,
	.stop_streaming = ov7750_stop_streaming,
	.s_ctrl = ov7750_s_ctrl,
	.s_ext_ctrls = ov7750_s_ext_ctrls,
	.g_ctrl = ov7750_g_ctrl,
	.g_timings = ov7750_g_timings,
	.check_camera_id = ov7750_check_camera_id,
	.configs = ov7750_configs,
	.num_configs = ARRAY_SIZE(ov7750_configs),
	.power_up_delays_ms = {5, 20, 0}
};

static int ov7750_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	dev_info(&client->dev, "probing...\n");

	ov7750_filltimings(&ov7750_custom_config);
	v4l2_i2c_subdev_init(&ov7750.sd, client, &ov7750_camera_module_ops);

	ov7750.custom = ov7750_custom_config;

	dev_info(&client->dev, "probing successful\n");
	return 0;
}

static int ov7750_remove(struct i2c_client *client)
{
	struct ov_camera_module *cam_mod = i2c_get_clientdata(client);

	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	ov_camera_module_release(cam_mod);

	dev_info(&client->dev, "removed\n");
	return 0;
}

static const struct i2c_device_id ov7750_id[] = {
	{ov7750_DRIVER_NAME, 0 },
	{}
};

static const struct of_device_id ov7750_of_match[] = {
	{.compatible = "ovti,ov7750-v4l2-i2c-subdev"},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov7750_id);

static struct i2c_driver ov7750_i2c_driver = {
	.driver = {
		.name = ov7750_DRIVER_NAME,
		.of_match_table = ov7750_of_match
	},
	.probe = ov7750_probe,
	.remove = ov7750_remove,
	.id_table = ov7750_id,
};

module_i2c_driver(ov7750_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for ov7750");
MODULE_AUTHOR("Jacob");
MODULE_LICENSE("GPL");
