/*
 * AR0330CS sensor driver
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
 */

#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-core.h>
#include <linux/slab.h>
#include "aptina_camera_module.h"

#define AR0330CS_DRIVER_NAME "ar0330cs"

#define AR0330CS_PID_ADDR 0x31fc
#define AR0330CS_PID_MAGIC 0x3020

#define AR0330CS_EXT_CLK 24000000

#define AR0330CS_AEC_PK_AGAIN_REG 0x3060 /* gain Bit 0-5 */
#define AR0330CS_FETCH_AGAIN(VAL) (VAL & 0x0030)

#define AR0330CS_AEC_PK_DGAIN_REG 0x305e
#define AR0330CS_FETCH_DGAIN(VAL) (VAL & 0x03ff)

#define AR0330CS_AEC_PK_LONG_EXPO_REG 0x3012 /* Exposure Bits 0-15 */
#define AR0330CS_FETCH_BYTE_EXP(VAL) (VAL & 0xFFFF) /* 16Bits */
#define AR0330CS_AFPS_ENABLE_REG 0x30ce
#define AR0330CS_AFPS_ENABLE_VAL 0x0020

#define AR0330CS_TIMING_VTS_REG 0x300a
#define AR0330CS_TIMING_HTS_REG 0x300c
#define AR0330CS_INTEGRATION_TIME_MARGIN 1
#define AR0330CS_FINE_INTG_TIME_MIN 0
#define AR0330CS_FINE_INTG_TIME_MAX_MARGIN 0
#define AR0330CS_COARSE_INTG_TIME_MIN 1
#define AR0330CS_COARSE_INTG_TIME_MAX_MARGIN 0
#define AR0330CS_HORIZONTAL_START_REG 0x3004
#define AR0330CS_HORIZONTAL_END_REG 0x3008
#define AR0330CS_VERTICAL_START_REG 0x3002
#define AR0330CS_VERTICAL_END_REG 0x3006

#define AR0330CS_READOUT_FLIP_MIRROR_REG 0x3040

static struct aptina_camera_module ar0330cs;
static struct aptina_camera_module_custom_config ar0330cs_custom_config;

/* ======================================================================== */

static struct aptina_camera_module_reg ar0330cs_init_tab_2304_1296_30fps[] = {
/*STATE = Master Clock, 49200000*/
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x301A, 0x0058},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x302A, 0x0005},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x302c, 0x0002},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x302e, 0x0002},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3030, 0x003b},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3036, 0x000a},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3038, 0x0001},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x31ac, 0x0a0a},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x31ae, 0x0202},

{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x31b0, 0x005e},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x31b2, 0x0038},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x31b4, 0x4f66},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x31b6, 0x4215},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x31b8, 0x308b},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x31ba, 0x028a},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x31bc, 0x0008},

{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3002, 0x007e},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3004, 0x0006},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3006, 0x058d},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3008, 0x0905},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x300a, 0x0802},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x300c, 0x04e0},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3012, 0x0801},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3014, 0x0000},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x30a2, 0x0001},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x30a6, 0x0001},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x308c, 0x007e},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x308a, 0x0006},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3090, 0x058d},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x308e, 0x0905},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x30aa, 0x0802},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x303e, 0x04e0},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3016, 0x0801},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3018, 0x0000},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x30ae, 0x0001},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x30a8, 0x0001},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3040, 0x0000},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3042, 0x0339},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x30ba, 0x002c},

{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x31e0, 0x0303},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3064, 0x1802},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3ed2, 0x0146},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3ed4, 0x8f6c},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3ed6, 0x66cc},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3ed8, 0x8c42},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3eda, 0x88bc},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x3edc, 0xaa63},
{APTINA_CAMERA_MODULE_REG_TYPE_DATA, 0x305e, 0x00a0},
};

/* ======================================================================== */

static struct aptina_camera_module_config ar0330cs_configs[] = {
	{
		.name = "2304_1296_30fps",
		.frm_fmt = {
			.width = 2304,
			.height = 1296,
			.code = V4L2_MBUS_FMT_SGRBG10_1X10
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
		.reg_table = (void *)ar0330cs_init_tab_2304_1296_30fps,
		.reg_table_num_entries =
			ARRAY_SIZE(ar0330cs_init_tab_2304_1296_30fps),
		.v_blanking_time_us = 5000,
		PLTFRM_CAM_ITF_MIPI_CFG(0, 2, 767, AR0330CS_EXT_CLK)
	},
};

/*--------------------------------------------------------------------------*/

static int ar0330cs_g_vts(struct aptina_camera_module *cam_mod, u32 *vts)
{
	u32 value;
	int ret;

	ret = aptina_camera_module_read_reg_table(
		cam_mod,
		AR0330CS_TIMING_VTS_REG,
		&value);
	if (IS_ERR_VALUE(ret))
		goto err;

	*vts = value;

	return 0;
err:
	aptina_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ar0330cs_auto_adjust_fps(struct aptina_camera_module *cam_mod,
	u32 exp_time)
{
	int ret;
	u32 vts;

	if ((cam_mod->exp_config.exp_time +
		AR0330CS_COARSE_INTG_TIME_MAX_MARGIN)
		> cam_mod->vts_min)
		vts = cam_mod->exp_config.exp_time+
			AR0330CS_COARSE_INTG_TIME_MAX_MARGIN;
	else
		vts = cam_mod->vts_min;

	ret = aptina_camera_module_write_reg(cam_mod,
		AR0330CS_TIMING_VTS_REG,
		vts & 0xFFFF);

	if (IS_ERR_VALUE(ret)) {
		aptina_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	} else {
		aptina_camera_module_pr_debug(cam_mod,
			"updated vts = %d,vts_min=%d\n",
			vts, cam_mod->vts_min);
		cam_mod->vts_cur = vts;
	}

	return ret;
}

/*--------------------------------------------------------------------------*/

static int ar0330cs_write_aec(struct aptina_camera_module *cam_mod)
{
	int ret = 0;

	aptina_camera_module_pr_debug(cam_mod,
		"exp_time = %d, gain = %d, flash_mode = %d\n",
		cam_mod->exp_config.exp_time,
		cam_mod->exp_config.gain,
		cam_mod->exp_config.flash_mode);
	/*
	 * if the sensor is already streaming, write to shadow registers,
	 * if the sensor is in SW standby, write to active registers,
	 * if the sensor is off/registers are not writeable, do nothing
	 */
	if ((cam_mod->state == APTINA_CAMERA_MODULE_SW_STANDBY) ||
		(cam_mod->state == APTINA_CAMERA_MODULE_STREAMING)) {
		u32 tgain = cam_mod->exp_config.gain;
		u32 exp_time = cam_mod->exp_config.exp_time;
		u32 again, dgain;

		tgain = tgain * cam_mod->exp_config.gain_percent / 100;
		if (tgain < 256) { /* 1x~2x */
			again = 0x00;
			dgain = tgain;
		} else if (tgain < 512) { /* 2x~4x */
			again = 0x10;
			dgain = tgain >> 1;
		} else if (tgain < 1024) { /* 4x~8x*/
			again = 0x20;
			dgain = tgain >> 2;
		} else { /* 8x~ */
			again = 0x30;
			dgain = tgain >> 3;
		}

		if (!IS_ERR_VALUE(ret) && cam_mod->auto_adjust_fps)
			ret = ar0330cs_auto_adjust_fps(cam_mod,
				cam_mod->exp_config.exp_time);

		/* GROUPED_PARAMETER_HOLD*/
		ret |= aptina_camera_module_write_reg(cam_mod,
			0x0104, 0x01);

		ret |= aptina_camera_module_write_reg(cam_mod,
			AR0330CS_AEC_PK_AGAIN_REG,
			AR0330CS_FETCH_AGAIN(again));
		ret |= aptina_camera_module_write_reg(cam_mod,
			AR0330CS_AEC_PK_DGAIN_REG,
			AR0330CS_FETCH_DGAIN(dgain));
		ret |= aptina_camera_module_write_reg(cam_mod,
			AR0330CS_AEC_PK_LONG_EXPO_REG,
			AR0330CS_FETCH_BYTE_EXP(exp_time));

		/* GROUPED_PARAMETER_HOLD*/
		ret |= aptina_camera_module_write_reg(cam_mod,
			0x0104, 0x00);
	}

	if (IS_ERR_VALUE(ret))
		aptina_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);

	return ret;
}

/*--------------------------------------------------------------------------*/

static int ar0330cs_g_ctrl(struct aptina_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	aptina_camera_module_pr_debug(cam_mod, "\n");

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
		aptina_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ar0330cs_filltimings(
	struct aptina_camera_module_custom_config *custom)
{
	int i, j;
	struct aptina_camera_module_config *config;
	struct aptina_camera_module_timings *timings;
	struct aptina_camera_module_reg *reg_table;
	int reg_table_num_entries;

	for (i = 0; i < custom->num_configs; i++) {
		config = &custom->configs[i];
		reg_table = config->reg_table;
		reg_table_num_entries = config->reg_table_num_entries;
		timings = &config->timings;

		for (j = 0; j < reg_table_num_entries; j++) {
			switch (reg_table[j].reg) {
			case AR0330CS_TIMING_VTS_REG:
				timings->frame_length_lines = reg_table[j].val;
				break;
			case AR0330CS_TIMING_HTS_REG:
				timings->line_length_pck = reg_table[j].val;
				break;
			case AR0330CS_HORIZONTAL_START_REG:
				timings->crop_horizontal_start =
					reg_table[j].val;
				break;
			case AR0330CS_VERTICAL_START_REG:
				timings->crop_vertical_start = reg_table[j].val;
				break;
			case AR0330CS_HORIZONTAL_END_REG:
				timings->crop_horizontal_end = reg_table[j].val;
				break;
			case AR0330CS_VERTICAL_END_REG:
				timings->crop_vertical_end = reg_table[j].val;
				break;
			}
		}

		timings->vt_pix_clk_freq_hz =
			config->frm_intrvl.interval.denominator
			* timings->frame_length_lines
			* timings->line_length_pck;
		timings->coarse_integration_time_min =
			AR0330CS_COARSE_INTG_TIME_MIN;
		timings->coarse_integration_time_max_margin =
			AR0330CS_COARSE_INTG_TIME_MAX_MARGIN;

		/* aptina Sensor do not use fine integration time. */
		timings->fine_integration_time_min =
			AR0330CS_FINE_INTG_TIME_MIN;
		timings->fine_integration_time_max_margin =
			AR0330CS_FINE_INTG_TIME_MAX_MARGIN;
	}

	return 0;
}

/*--------------------------------------------------------------------------*/

static int ar0330cs_g_timings(struct aptina_camera_module *cam_mod,
	struct aptina_camera_module_timings *timings)
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
	aptina_camera_module_pr_err(cam_mod,
		"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ar0330cs_set_flip(
	struct aptina_camera_module *cam_mod,
	struct pltfrm_camera_module_reg reglist[],
	int len)
{
	int i, value = 0, mode = 0;

	mode = aptina_camera_module_get_flip_mirror(cam_mod);
	aptina_camera_module_pr_debug(cam_mod,
		"mode %d\n", mode);

	if (mode == -1) {
		aptina_camera_module_pr_err(
			cam_mod,
			"dts don't set flip, return!\n");
		return 0;
	}

	if (mode == APTINA_FLIP_BIT_MASK)
		value = 0x8000;
	else if (mode == APTINA_MIRROR_BIT_MASK)
		value = 0x4000;
	else if (mode == (APTINA_FLIP_BIT_MASK |
		APTINA_MIRROR_BIT_MASK))
		value = 0xc000;
	else
		value = 0x0000;

	for (i = 0; i < len; i++)
		if (reglist[i].reg == AR0330CS_READOUT_FLIP_MIRROR_REG)
			reglist[i].val = value;

	return 0;
}

/*--------------------------------------------------------------------------*/

static int ar0330cs_s_ctrl(struct aptina_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	aptina_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
		ret = ar0330cs_write_aec(cam_mod);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (IS_ERR_VALUE(ret))
		aptina_camera_module_pr_debug(cam_mod,
			"failed with error (%d) 0x%x\n", ret, ctrl_id);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ar0330cs_s_ext_ctrls(struct aptina_camera_module *cam_mod,
	struct aptina_camera_module_ext_ctrls *ctrls)
{
	int ret = 0;

	/* Handles only exposure and gain together special case. */
	if ((ctrls->ctrls[0].id == V4L2_CID_GAIN ||
		ctrls->ctrls[0].id == V4L2_CID_EXPOSURE))
		ret = ar0330cs_write_aec(cam_mod);
	else
		ret = -EINVAL;

	if (IS_ERR_VALUE(ret))
		aptina_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);

	return ret;
}

static int ar0330cs_start_streaming(struct aptina_camera_module *cam_mod)
{
	int ret = 0;

	aptina_camera_module_pr_info(cam_mod,
		"active config=%s\n",
		cam_mod->active_config->name);

	ret = ar0330cs_g_vts(cam_mod, &cam_mod->vts_min);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = aptina_camera_module_write_reg(cam_mod, 0x301a, 0x5c);
	if (IS_ERR_VALUE(ret))
		goto err;

	msleep(25);

	return 0;
err:
	aptina_camera_module_pr_err(cam_mod,
		"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int ar0330cs_stop_streaming(struct aptina_camera_module *cam_mod)
{
	int ret = 0;

	aptina_camera_module_pr_info(cam_mod, "\n");

	ret = aptina_camera_module_write_reg(cam_mod, 0x301a, 0x58);
	if (IS_ERR_VALUE(ret))
		goto err;

	msleep(25);

	return 0;
err:
	aptina_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}


/*--------------------------------------------------------------------------*/

static int ar0330cs_check_camera_id(struct aptina_camera_module *cam_mod)
{
	u32 pid;
	int ret = 0;

	ret = aptina_camera_module_read_reg(cam_mod,
		2, AR0330CS_PID_ADDR, &pid);
	if (IS_ERR_VALUE(ret)) {
		aptina_camera_module_pr_err(cam_mod,
			"register read failed, camera module powered off?\n");
		goto err;
	}

	if (pid == AR0330CS_PID_MAGIC) {
		aptina_camera_module_pr_info(cam_mod,
			"successfully detected camera ID 0x%04x\n", pid);
	} else {
		aptina_camera_module_pr_err(cam_mod,
			"wrong camera ID, expected 0x%04x, detected 0x%04x\n",
			AR0330CS_PID_MAGIC, pid);
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	aptina_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}

/* ======================================================================== */
/* This part is platform dependent */
/* ======================================================================== */

static struct v4l2_subdev_core_ops ar0330cs_camera_module_core_ops = {
	.g_ctrl = aptina_camera_module_g_ctrl,
	.s_ctrl = aptina_camera_module_s_ctrl,
	.s_ext_ctrls = aptina_camera_module_s_ext_ctrls,
	.s_power = aptina_camera_module_s_power,
	.ioctl = aptina_camera_module_ioctl
};

static struct v4l2_subdev_video_ops ar0330cs_camera_module_video_ops = {
	.enum_frameintervals = aptina_camera_module_enum_frameintervals,
	.s_mbus_fmt = aptina_camera_module_s_fmt,
	.g_mbus_fmt = aptina_camera_module_g_fmt,
	.try_mbus_fmt = aptina_camera_module_try_fmt,
	.s_frame_interval = aptina_camera_module_s_frame_interval,
	.g_frame_interval = aptina_camera_module_g_frame_interval,
	.s_stream = aptina_camera_module_s_stream
};

static struct v4l2_subdev_ops ar0330cs_camera_module_ops = {
	.core = &ar0330cs_camera_module_core_ops,
	.video = &ar0330cs_camera_module_video_ops,
};

static struct aptina_camera_module_custom_config ar0330cs_custom_config = {
	.start_streaming = ar0330cs_start_streaming,
	.stop_streaming = ar0330cs_stop_streaming,
	.s_ctrl = ar0330cs_s_ctrl,
	.s_ext_ctrls = ar0330cs_s_ext_ctrls,
	.g_ctrl = ar0330cs_g_ctrl,
	.g_timings = ar0330cs_g_timings,
	.check_camera_id = ar0330cs_check_camera_id,
	.set_flip = ar0330cs_set_flip,
	.s_vts = ar0330cs_auto_adjust_fps,
	.configs = ar0330cs_configs,
	.num_configs = ARRAY_SIZE(ar0330cs_configs),
	.power_up_delays_ms = {5, 20, 0},
	/*
	 * 0: Exposure time valid fileds;
	 * 1: Exposure gain valid fileds;
	 * (2 fileds == 1 frames)
	 */
	.exposure_valid_frame = {4, 4}
};

static int ar0330cs_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	dev_info(&client->dev, "probing...\n");

	ar0330cs_filltimings(&ar0330cs_custom_config);
	v4l2_i2c_subdev_init(&ar0330cs.sd,
		client, &ar0330cs_camera_module_ops);
	ar0330cs.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ar0330cs.custom = ar0330cs_custom_config;

	dev_info(&client->dev, "probing successful\n");

	return 0;
}

static int ar0330cs_remove(struct i2c_client *client)
{
	struct aptina_camera_module *cam_mod = i2c_get_clientdata(client);

	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;

	aptina_camera_module_release(cam_mod);

	dev_info(&client->dev, "removed\n");
	return 0;
}

static const struct i2c_device_id ar0330cs_id[] = {
	{ AR0330CS_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id ar0330cs_of_match[] = {
	{.compatible = "aptina,ar0330cs-v4l2-i2c-subdev"},
	{},
};

MODULE_DEVICE_TABLE(i2c, ar0330cs_id);

static struct i2c_driver ar0330cs_i2c_driver = {
	.driver = {
		.name = AR0330CS_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ar0330cs_of_match
	},
	.probe = ar0330cs_probe,
	.remove = ar0330cs_remove,
	.id_table = ar0330cs_id,
};

module_i2c_driver(ar0330cs_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for ar0330cs");
MODULE_AUTHOR("Cain.cai");
MODULE_LICENSE("GPL");

