/*
 * IMX219 sensor driver
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
#include "imx_camera_module.h"

#define IMX219_DRIVER_NAME "imx219"

#define IMX219_AEC_PK_GAIN_REG	 0x0157

#define IMX219_AEC_PK_EXPO_HIGH_REG 0x015a
#define IMX219_AEC_PK_EXPO_LOW_REG 0x015b

#define IMX219_FETCH_HIGH_BYTE_EXP(VAL) ((VAL >> 8) & 0xFF)
#define IMX219_FETCH_LOW_BYTE_EXP(VAL) (VAL & 0xFF)

#define IMX219_PIDH_ADDR    0x0000
#define IMX219_PIDL_ADDR    0x0001
#define IMX219_PIDH_MAGIC   0x02
#define IMX219_PIDL_MAGIC   0x19

#define IMX219_TIMING_VTS_HIGH_REG 0x0160
#define IMX219_TIMING_VTS_LOW_REG 0x0161
#define IMX219_TIMING_HTS_HIGH_REG 0x0162
#define IMX219_TIMING_HTS_LOW_REG 0x0163

#define IMX219_INTEGRATION_TIME_MARGIN 8
#define IMX219_FINE_INTG_TIME_MIN 0
#define IMX219_FINE_INTG_TIME_MAX_MARGIN 0
#define IMX219_COARSE_INTG_TIME_MIN 1
#define IMX219_COARSE_INTG_TIME_MAX_MARGIN 4
#define IMX219_TIMING_X_INC		0x0170
#define IMX219_TIMING_Y_INC		0x0171
#define IMX219_HORIZONTAL_START_HIGH_REG 0x0164
#define IMX219_HORIZONTAL_START_LOW_REG 0x0165
#define IMX219_VERTICAL_START_HIGH_REG 0x0168
#define IMX219_VERTICAL_START_LOW_REG 0x0169
#define IMX219_HORIZONTAL_END_HIGH_REG 0x0166
#define IMX219_HORIZONTAL_END_LOW_REG 0x0167
#define IMX219_VERTICAL_END_HIGH_REG 0x016a
#define IMX219_VERTICAL_END_LOW_REG 0x016b
#define IMX219_HORIZONTAL_OUTPUT_SIZE_HIGH_REG 0x016c
#define IMX219_HORIZONTAL_OUTPUT_SIZE_LOW_REG 0x016d
#define IMX219_VERTICAL_OUTPUT_SIZE_HIGH_REG 0x016e
#define IMX219_VERTICAL_OUTPUT_SIZE_LOW_REG 0x016f

#define IMX219_ORIENTATION_REG	0x0172
#define IMX219_ORIENTATION_H 0x1
#define IMX219_ORIENTATION_V 0x2

#define IMX219_EXT_CLK 24000000

static struct imx_camera_module imx219;
static struct imx_camera_module_custom_config imx219_custom_config;

/* ======================================================================== */
/* Base sensor configs */
/* ======================================================================== */

/* MCLK:24MHz  3280x2464  21.2fps   MIPI LANE2 */
static struct imx_camera_module_reg imx219_init_tab_3280_2464_21fps[] = {
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x30eb, 0x05},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x30eb, 0x0c},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x300a, 0xff},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x300b, 0xff},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x30eb, 0x05},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x30eb, 0x09},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0114, 0x01},//CSI_LANE_MODE
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0128, 0x00},//DPHY_CNTRL
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x012a, 0x18},//EXCK_FREQ H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x012b, 0x00},//EXCK_FREQ L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0157, 0x48},//ANALOG GAIN
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x015a, 0x01},//INTEG TIME H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x015b, 0xf4},//INTEG TIME L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0160, 0x09},//FRM_LENGTH H 2500
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0161, 0xc4},//FRM_LENGTH L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0162, 0x0d},//LINE LENGTH H 3448
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0163, 0x78},//LINE LENGTG L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0164, 0x00},//X_ADD_STA H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0165, 0x00},//X_ADD_STA L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0166, 0x0c},//X_ADD_END H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0167, 0xcf},//X_ADD_END L 
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0168, 0x00},//Y_ADD_STA H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0169, 0x00},//Y_ADD_STA L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016a, 0x09},//Y_ADD_END H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016b, 0x9f},//Y_ADD_END L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016c, 0x0c},//x_OUTPUT_SIZE H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016d, 0xd0},//x_OUTPUT_SIZE L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016e, 0x09},//y_OUTPUT_SIZE H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016f, 0xa0},//y_OUTPUT_SIZE L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0170, 0x01},//X_ODD_INC
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0171, 0x01},//Y_ODD_INC
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0174, 0x00},//BINNING_MODE_H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0175, 0x00},//BINNING_MODE_V
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x018c, 0x0a},//CSI_DATA_FORMAT H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x018d, 0x0a},//CSI_DATA_FORMAT L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0301, 0x05},//VTPXCK_DIV
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0303, 0x01},//VTSYCK_DIV
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0304, 0x03},//PREPLLCK_VT_DIV
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0305, 0x03},//PREPLLCK_OP_DIV
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0306, 0x00},//PLL_VT_MPY H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0307, 0x39},//PLL_VT_MPY L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0309, 0x0a},//OPPXCK_DIV H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x030b, 0x01},//OPSYCK_DIV
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x030c, 0x00},//PLL_OP_MPY H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x030d, 0x72},//PLL_OP_MPY L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x4767, 0x0f},//CIS Tuning
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x4750, 0x14},//CIS Tuning
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x47b4, 0x14}//CIS Tuning
};
static struct imx_camera_module_reg imx219_init_tab_1920_1080_48fps[] = {
/* MCLK:24MHz  1920x1080  47.5fps   MIPI LANE2 */
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x30eb, 0x05},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x30eb, 0x0c},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x300a, 0xff},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x300b, 0xff},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x30eb, 0x05},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x30eb, 0x09},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0114, 0x01},//CSI_LANE_MODE
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0128, 0x00},//DPHY_CNTRL
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x012a, 0x18},//EXCK_FREQ H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x012b, 0x00},//EXCK_FREQ L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0157, 0x90},//ANALOG GAIN
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x015a, 0x03},//INTEG TIME H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x015b, 0xe8},//INTEG TIME L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0160, 0x04},//FRM_LENGTH H 1113
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0161, 0x59},//FRM_LENGTH L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0162, 0x0d},//LINE LENGTH H 3448
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0163, 0x78},//LINE LENGTG L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0164, 0x02},//X_ADD_STA H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0165, 0xa8},//X_ADD_STA L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0166, 0x0a},//X_ADD_END H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0167, 0x27},//X_ADD_END L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0168, 0x02},//Y_ADD_STA H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0169, 0xb4},//Y_ADD_STA L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016a, 0x06},//Y_ADD_END H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016b, 0xeb},//Y_ADD_END L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016c, 0x07},//x_OUTPUT_SIZE H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016d, 0x80},//x_OUTPUT_SIZE L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016e, 0x04},//y_OUTPUT_SIZE H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016f, 0x38},//y_OUTPUT_SIZE L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0170, 0x01},//X_ODD_INC
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0171, 0x01},//X_ODD_INC
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0174, 0x00},//BINNING_MODE_H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0175, 0x00},//BINNING_MODE_V
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x018c, 0x0a},//CSI_DATA_FORMAT H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x018d, 0x0a},//CSI_DATA_FORMAT L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0301, 0x05},//VTPXCK_DIV
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0303, 0x01},//VTSYCK_DIV
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0304, 0x03},//PREPLLCK_VT_DIV
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0305, 0x03},//PREPLLCK_OP_DIV
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0306, 0x00},//PLL_VT_MPY H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0307, 0x39},//PLL_VT_MPY L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0309, 0x0a},//OPPXCK_DIV H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x030b, 0x01},//OPSYCK_DIV
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x030c, 0x00},//PLL_OP_MPY H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x030d, 0x72},//PLL_OP_MPY L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x455e, 0x00},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x471e, 0x4b},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x4767, 0x0f},//CIS Tuning
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x4750, 0x14},//CIS Tuning
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x4540, 0x00},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x47b4, 0x14},//CIS Tuning
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x4713, 0x30},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x478b, 0x10},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x478f, 0x10},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x4793, 0x10},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x4797, 0x0e},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x479b, 0x0e}
};
#if 0
static struct imx_camera_module_reg imx219_init_tab_1640_922_30fps[] = {
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x30eb, 0x05},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x30eb, 0x0c},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x300a, 0xff},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x300b, 0xff},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x30eb, 0x05},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x30eb, 0x09},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0114, 0x01},//CSI_LANE_MODE
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0128, 0x00},//DPHY_CNTRL
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x012a, 0x18},//EXCK_FREQ H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x012b, 0x00},//EXCK_FREQ L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0157, 0x90},//ANALOG GAIN
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x015a, 0x03},//INTEG TIME H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x015b, 0xe8},//INTEG TIME L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0160, 0x06},//FRM_LENGTH H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0161, 0xe3},//FRM_LENGTH L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0162, 0x0d},//LINE LENGTH H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0163, 0x78},//LINE LENGTG L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0164, 0x00},//X_ADD_STA H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0165, 0x00},//X_ADD_STA L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0166, 0x0c},//X_ADD_END H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0167, 0xcf},//X_ADD_END L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0168, 0x01},//Y_ADD_STA H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0169, 0x36},//Y_ADD_STA L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016a, 0x08},//Y_ADD_END H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016b, 0x69},//Y_ADD_END L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016c, 0x06},//x_OUTPUT_SIZE H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016d, 0x68},//x_OUTPUT_SIZE L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016e, 0x03},//y_OUTPUT_SIZE H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x016f, 0x9a},//y_OUTPUT_SIZE L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0170, 0x01},//X_ODD_INC
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0171, 0x01},//X_ODD_INC
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0174, 0x01},//BINNING_MODE_H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0175, 0x01},//BINNING_MODE_V
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x018c, 0x0a},//CSI_DATA_FORMAT H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x018d, 0x0a},//CSI_DATA_FORMAT L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0301, 0x05},//VTPXCK_DIV
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0303, 0x01},//VTSYCK_DIV
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0304, 0x03},//PREPLLCK_VT_DIV
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0305, 0x03},//PREPLLCK_OP_DIV
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0306, 0x00},//PLL_VT_MPY H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0307, 0x39},//PLL_VT_MPY L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x0309, 0x0a},//OPPXCK_DIV H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x030b, 0x01},//OPSYCK_DIV
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x030c, 0x00},//PLL_OP_MPY H
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x030d, 0x72},//PLL_OP_MPY L
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x455e, 0x00},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x471e, 0x4b},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x4767, 0x0f},//CIS Tuning
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x4750, 0x14},//CIS Tuning
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x4540, 0x00},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x47b4, 0x14},//CIS Tuning
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x4713, 0x30},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x478b, 0x10},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x478f, 0x10},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x4793, 0x10},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x4797, 0x0e},
{IMX_CAMERA_MODULE_REG_TYPE_DATA, 0x479b, 0x0e}
};
#endif
/* ======================================================================== */

static struct imx_camera_module_config imx219_configs[] = {
	{
		.name = "3280x2464_21fps",
		.frm_fmt = {
			.width = 3280,
			.height = 2464,
			.code = MEDIA_BUS_FMT_SRGGB10_1X10
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 21
			}
		},
		.auto_exp_enabled = false,
		.auto_gain_enabled = false,
		.auto_wb_enabled = false,
		.reg_table = (void *)imx219_init_tab_3280_2464_21fps,
		.reg_table_num_entries =
		sizeof(imx219_init_tab_3280_2464_21fps)
		/
		sizeof(imx219_init_tab_3280_2464_21fps[0]),
		.v_blanking_time_us = 5000,
		PLTFRM_CAM_ITF_MIPI_CFG(0, 2, 912, IMX219_EXT_CLK)
	},
	{
		.name = "1920x1080_48fps",
		.frm_fmt = {
			.width = 1920,
			.height = 1080,
			.code = MEDIA_BUS_FMT_SRGGB10_1X10
		},
		.frm_intrvl = {
			.interval = {
				.numerator = 1,
				.denominator = 48
			}
		},
		.auto_exp_enabled = false,
		.auto_gain_enabled = false,
		.auto_wb_enabled = false,
		.reg_table = (void *)imx219_init_tab_1920_1080_48fps,
		.reg_table_num_entries =
		sizeof(imx219_init_tab_1920_1080_48fps)
		/
		sizeof(imx219_init_tab_1920_1080_48fps[0]),
		.v_blanking_time_us = 5000,
		PLTFRM_CAM_ITF_MIPI_CFG(0, 2, 912, IMX219_EXT_CLK)
	}
#if 0
	{
		.name = "1640x922_30fps",
		.frm_fmt = {
			.width = 1640,
			.height = 922,
			.code = MEDIA_BUS_FMT_SRGGB10_1X10
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
		.reg_table = (void *)imx219_init_tab_1640_922_30fps,
		.reg_table_num_entries =
		sizeof(imx219_init_tab_1640_922_30fps)
		/
		sizeof(imx219_init_tab_1640_922_30fps[0]),
		.v_blanking_time_us = 5000,
		PLTFRM_CAM_ITF_MIPI_CFG(0, 2, 912, IMX219_EXT_CLK)
	}
#endif
};

/*--------------------------------------------------------------------------*/

static int imx219_g_VTS(struct imx_camera_module *cam_mod, u32 *vts)
{
	u32 msb, lsb;
	int ret;

	ret = imx_camera_module_read_reg_table(
		cam_mod,
		IMX219_TIMING_VTS_HIGH_REG,
		&msb);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = imx_camera_module_read_reg_table(
		cam_mod,
		IMX219_TIMING_VTS_LOW_REG,
		&lsb);
	if (IS_ERR_VALUE(ret))
		goto err;

	*vts = (msb << 8) | lsb;
	cam_mod->vts_cur = *vts;
	return 0;
err:
	imx_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int imx219_auto_adjust_fps(struct imx_camera_module *cam_mod,
	u32 exp_time)
{
	int ret;
	u32 vts;

	if ((cam_mod->exp_config.exp_time + IMX219_COARSE_INTG_TIME_MAX_MARGIN)
		> cam_mod->vts_min)
		vts = cam_mod->exp_config.exp_time + IMX219_COARSE_INTG_TIME_MAX_MARGIN;
	else
		vts = cam_mod->vts_min;
	ret = imx_camera_module_write_reg(cam_mod, IMX219_TIMING_VTS_LOW_REG, vts & 0xFF);
	ret |= imx_camera_module_write_reg(cam_mod, IMX219_TIMING_VTS_HIGH_REG, (vts >> 8) & 0xFF);

	if (IS_ERR_VALUE(ret))
		imx_camera_module_pr_err(cam_mod,
				"failed with error (%d)\n", ret);
	else{
		imx_camera_module_pr_debug(cam_mod,
					  "updated vts = %d,vts_min=%d\n", vts, cam_mod->vts_min);
		cam_mod->vts_cur = vts;
	}
	return ret;
}

/*--------------------------------------------------------------------------*/

static int imx219_write_aec(struct imx_camera_module *cam_mod)
{
	int ret = 0;

	imx_camera_module_pr_debug(cam_mod,
				  "exp_time = %d, gain = %d, flash_mode = %d\n",
				  cam_mod->exp_config.exp_time,
				  cam_mod->exp_config.gain,
				  cam_mod->exp_config.flash_mode);

	/* if the sensor is already streaming, write to shadow registers,
	   if the sensor is in SW standby, write to active registers,
	   if the sensor is off/registers are not writeable, do nothing */
	if ((cam_mod->state == IMX_CAMERA_MODULE_SW_STANDBY) || (cam_mod->state == IMX_CAMERA_MODULE_STREAMING)) {
		u32 a_gain = cam_mod->exp_config.gain;
		u32 exp_time = cam_mod->exp_config.exp_time;
		a_gain = a_gain * cam_mod->exp_config.gain_percent / 100;

		if (!IS_ERR_VALUE(ret) && cam_mod->auto_adjust_fps)
			ret = imx219_auto_adjust_fps(cam_mod, cam_mod->exp_config.exp_time);

		// Gain
		ret = imx_camera_module_write_reg(cam_mod, IMX219_AEC_PK_GAIN_REG, a_gain);

		// Integration Time
		ret = imx_camera_module_write_reg(cam_mod, IMX219_AEC_PK_EXPO_HIGH_REG, IMX219_FETCH_HIGH_BYTE_EXP(exp_time));
		ret |= imx_camera_module_write_reg(cam_mod, IMX219_AEC_PK_EXPO_LOW_REG, IMX219_FETCH_LOW_BYTE_EXP(exp_time));

	}

	if (IS_ERR_VALUE(ret))
		imx_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int imx219_g_ctrl(struct imx_camera_module *cam_mod, u32 ctrl_id)
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
static int imx219_filltimings(struct imx_camera_module_custom_config *custom)
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

		memset(timings, 0x00, sizeof(*timings));
		for (j = 0; j < reg_table_num_entries; j++) {
			switch (reg_table[j].reg) {
			case IMX219_TIMING_VTS_HIGH_REG:
				timings->frame_length_lines =
					((reg_table[j].val << 8) |
					(timings->frame_length_lines & 0xff));
				break;
			case IMX219_TIMING_VTS_LOW_REG:
				timings->frame_length_lines =
					(reg_table[j].val |
					(timings->frame_length_lines & 0xff00));
				break;
			case IMX219_TIMING_HTS_HIGH_REG:
				timings->line_length_pck =
					((reg_table[j].val << 8) |
					(timings->line_length_pck & 0xff));
				break;
			case IMX219_TIMING_HTS_LOW_REG:
				timings->line_length_pck =
					(reg_table[j].val |
					(timings->line_length_pck & 0xff00));
				break;
			case IMX219_TIMING_X_INC:
				timings->binning_factor_x = ((reg_table[j].val >> 4) + 1) / 2;
				if (timings->binning_factor_x == 0)
					timings->binning_factor_x = 1;
				break;
			case IMX219_TIMING_Y_INC:
				timings->binning_factor_y = ((reg_table[j].val >> 4) + 1) / 2;
				if (timings->binning_factor_y == 0)
					timings->binning_factor_y = 1;
				break;
			case IMX219_HORIZONTAL_START_HIGH_REG:
				timings->crop_horizontal_start =
					((reg_table[j].val << 8) |
					(timings->crop_horizontal_start & 0xff));
				break;
			case IMX219_HORIZONTAL_START_LOW_REG:
				timings->crop_horizontal_start =
					(reg_table[j].val |
					(timings->crop_horizontal_start & 0xff00));
				break;
			case IMX219_VERTICAL_START_HIGH_REG:
				timings->crop_vertical_start =
					((reg_table[j].val << 8) |
					(timings->crop_vertical_start & 0xff));
				break;
			case IMX219_VERTICAL_START_LOW_REG:
				timings->crop_vertical_start =
					((reg_table[j].val) |
					(timings->crop_vertical_start & 0xff00));
				break;
			case IMX219_HORIZONTAL_END_HIGH_REG:
				timings->crop_horizontal_end =
					((reg_table[j].val << 8) |
					(timings->crop_horizontal_end & 0xff));
				break;
			case IMX219_HORIZONTAL_END_LOW_REG:
				timings->crop_horizontal_end =
					(reg_table[j].val |
					(timings->crop_horizontal_end & 0xff00));
				break;
			case IMX219_VERTICAL_END_HIGH_REG:
				timings->crop_vertical_end =
					((reg_table[j].val << 8) |
					(timings->crop_vertical_end & 0xff));
				break;
			case IMX219_VERTICAL_END_LOW_REG:
				timings->crop_vertical_end =
					(reg_table[j].val |
					(timings->crop_vertical_end & 0xff00));
				break;
			case IMX219_HORIZONTAL_OUTPUT_SIZE_HIGH_REG:
				timings->sensor_output_width =
					((reg_table[j].val << 8) |
					(timings->sensor_output_width & 0xff));
				break;
			case IMX219_HORIZONTAL_OUTPUT_SIZE_LOW_REG:
				timings->sensor_output_width =
					(reg_table[j].val |
					(timings->sensor_output_width & 0xff00));
				break;
			case IMX219_VERTICAL_OUTPUT_SIZE_HIGH_REG:
				timings->sensor_output_height =
					((reg_table[j].val << 8) |
					(timings->sensor_output_height & 0xff));
				break;
			case IMX219_VERTICAL_OUTPUT_SIZE_LOW_REG:
				timings->sensor_output_height =
					(reg_table[j].val |
					(timings->sensor_output_height & 0xff00));
				break;
			case IMX219_AEC_PK_EXPO_HIGH_REG:
				timings->exp_time =
					((reg_table[j].val << 8) |
					(timings->exp_time & 0xff));
				break;
			case IMX219_AEC_PK_EXPO_LOW_REG:
				timings->exp_time =
					(reg_table[j].val |
					(timings->exp_time & 0xff00));
				break;
			case IMX219_AEC_PK_GAIN_REG:
				timings->gain =
					(reg_table[j].val |
					(timings->gain & 0x00));
				break;
			}
		}

		timings->vt_pix_clk_freq_hz = config->frm_intrvl.interval.denominator
					* timings->frame_length_lines
					* timings->line_length_pck;
		timings->coarse_integration_time_min = IMX219_COARSE_INTG_TIME_MIN;
		timings->coarse_integration_time_max_margin = IMX219_COARSE_INTG_TIME_MAX_MARGIN;

		/* IMX Sensor do not use fine integration time. */
		timings->fine_integration_time_min = IMX219_FINE_INTG_TIME_MIN;
		timings->fine_integration_time_max_margin = IMX219_FINE_INTG_TIME_MAX_MARGIN;
	}

	return 0;
}
static int imx219_g_timings(struct imx_camera_module *cam_mod,
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
	imx_camera_module_pr_err(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int imx219_set_flip(struct imx_camera_module *cam_mod)
{
	int mode = 0;
	u16 orientation = 0;

	mode = imx_camera_module_get_flip_mirror(cam_mod);
	if (mode == -1) {
		imx_camera_module_pr_info(
			cam_mod,
			"dts don't set flip, return!\n");
		return 0;
	}

	if (!IS_ERR_OR_NULL(cam_mod->active_config)) {
		if (mode == IMX_FLIP_BIT_MASK) {
			orientation = IMX219_ORIENTATION_V;
		}
		else if (mode == IMX_MIRROR_BIT_MASK) {
			orientation = IMX219_ORIENTATION_H;
		}
		else if (mode == (IMX_MIRROR_BIT_MASK | IMX_FLIP_BIT_MASK)) {
			orientation = IMX219_ORIENTATION_H | IMX219_ORIENTATION_V;
		}
		else {
			orientation = 0;
		}
	}

	return 0;
}

/*--------------------------------------------------------------------------*/

static int imx219_s_ctrl(struct imx_camera_module *cam_mod, u32 ctrl_id)
{
	int ret = 0;

	imx_camera_module_pr_debug(cam_mod, "\n");

	switch (ctrl_id) {
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
		ret = imx219_write_aec(cam_mod);
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

static int imx219_s_ext_ctrls(struct imx_camera_module *cam_mod,
				 struct imx_camera_module_ext_ctrls *ctrls)
{
	int ret = 0;

	/* Handles only exposure and gain together special case. */
	if (ctrls->count == 1)
		ret = imx219_s_ctrl(cam_mod, ctrls->ctrls[0].id);
	else if ((ctrls->count == 3) &&
		 ((ctrls->ctrls[0].id == V4L2_CID_GAIN &&
		   ctrls->ctrls[1].id == V4L2_CID_EXPOSURE) ||
		  (ctrls->ctrls[1].id == V4L2_CID_GAIN &&
		   ctrls->ctrls[0].id == V4L2_CID_EXPOSURE)))
		ret = imx219_write_aec(cam_mod);
	else
		ret = -EINVAL;

	if (IS_ERR_VALUE(ret))
		imx_camera_module_pr_debug(cam_mod,
			"failed with error (%d)\n", ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int imx219_start_streaming(struct imx_camera_module *cam_mod)
{
	int ret = 0;
	
	imx_camera_module_pr_debug(cam_mod, "active config=%s\n", cam_mod->active_config->name);

	ret = imx219_g_VTS(cam_mod, &cam_mod->vts_min);
	if (IS_ERR_VALUE(ret))
		goto err;

	if (IS_ERR_VALUE(imx_camera_module_write_reg(cam_mod, 0x0100, 1)))
		goto err;

	msleep(25);

	return 0;
err:
	imx_camera_module_pr_err(cam_mod, "failed with error (%d)\n",
		ret);
	return ret;
}

/*--------------------------------------------------------------------------*/

static int imx219_stop_streaming(struct imx_camera_module *cam_mod)
{
	int ret = 0;

	imx_camera_module_pr_debug(cam_mod, "\n");

	ret = imx_camera_module_write_reg(cam_mod, 0x0100, 0);
	if (IS_ERR_VALUE(ret))
		goto err;

	msleep(25);

	return 0;
err:
	imx_camera_module_pr_err(cam_mod, "failed with error (%d)\n", ret);
	return ret;
}
/*--------------------------------------------------------------------------*/
static int imx219_check_camera_id(struct imx_camera_module *cam_mod)
{
	u32 pidh, pidl;
	int ret = 0;

	imx_camera_module_pr_debug(cam_mod, "\n");

	ret |= imx_camera_module_read_reg(cam_mod, 1, IMX219_PIDH_ADDR, &pidh);
        ret |= imx_camera_module_read_reg(cam_mod, 1, IMX219_PIDL_ADDR, &pidl);
	if (IS_ERR_VALUE(ret)) {
		imx_camera_module_pr_err(cam_mod,
			"register read failed, camera module powered off?\n");
		goto err;
	}

	if ((pidh == IMX219_PIDH_MAGIC) && (pidl == IMX219_PIDL_MAGIC))
		imx_camera_module_pr_debug(cam_mod,
			"successfully detected camera ID 0x%02x 0x%02x\n", pidh, pidl);
	else {
		imx_camera_module_pr_err(cam_mod,
			"wrong camera ID, expected 0x%02x 0x%02x, detected 0x%02x 0x%02x\n",
			IMX219_PIDH_MAGIC, IMX219_PIDL_MAGIC, pidh, pidl);
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

static struct v4l2_subdev_core_ops imx219_camera_module_core_ops = {
	.g_ctrl = imx_camera_module_g_ctrl,
	.s_ctrl = imx_camera_module_s_ctrl,
	.s_ext_ctrls = imx_camera_module_s_ext_ctrls,
	.s_power = imx_camera_module_s_power,
	.ioctl = imx_camera_module_ioctl
};

static struct v4l2_subdev_video_ops imx219_camera_module_video_ops = {
	.s_frame_interval = imx_camera_module_s_frame_interval,
	.s_stream = imx_camera_module_s_stream
};
static struct v4l2_subdev_pad_ops imx219_camera_module_pad_ops = {
	.enum_frame_interval = imx_camera_module_enum_frameintervals,
	.get_fmt = imx_camera_module_g_fmt,
	.set_fmt = imx_camera_module_s_fmt,
};

static struct v4l2_subdev_ops imx219_camera_module_ops = {
	.core = &imx219_camera_module_core_ops,
	.video = &imx219_camera_module_video_ops,
	.pad = &imx219_camera_module_pad_ops
};

static struct imx_camera_module_custom_config imx219_custom_config = {
	.start_streaming = imx219_start_streaming,
	.stop_streaming = imx219_stop_streaming,
	.s_ctrl = imx219_s_ctrl,
	.s_ext_ctrls = imx219_s_ext_ctrls,
	.g_ctrl = imx219_g_ctrl,
	.g_timings = imx219_g_timings,
	.check_camera_id = imx219_check_camera_id,
	.set_flip = imx219_set_flip,
	.configs = imx219_configs,
	.num_configs = ARRAY_SIZE(imx219_configs),
	.power_up_delays_ms = {5, 20, 0}
};

static int imx219_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	dev_info(&client->dev, "probing...\n");

	imx219_filltimings(&imx219_custom_config);
	v4l2_i2c_subdev_init(&imx219.sd, client, &imx219_camera_module_ops);
	imx219.custom = imx219_custom_config;

	dev_info(&client->dev, "probing successful\n");
	return 0;
}

static int imx219_remove(struct i2c_client *client)
{
	struct imx_camera_module *cam_mod = i2c_get_clientdata(client);

	dev_info(&client->dev, "removing device...\n");

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	imx_camera_module_release(cam_mod);

	dev_info(&client->dev, "removed\n");
	return 0;
}

static const struct i2c_device_id imx219_id[] = {
	{ IMX219_DRIVER_NAME, 0 },
	{ }
};

static struct of_device_id imx219_of_match[] = {
	{.compatible = "sony,imx219-v4l2-i2c-subdev"},
	{},
};

MODULE_DEVICE_TABLE(i2c, imx219_id);

static struct i2c_driver imx219_i2c_driver = {
	.driver = {
		.name = IMX219_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = imx219_of_match
	},
	.probe = imx219_probe,
	.remove = imx219_remove,
	.id_table = imx219_id,
};

module_i2c_driver(imx219_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for IMX219");
MODULE_AUTHOR("George");
MODULE_LICENSE("GPL");