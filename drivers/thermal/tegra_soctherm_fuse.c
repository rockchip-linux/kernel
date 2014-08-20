/*
 * drivers/thermal/tegra_soctherm.c
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *	Mikko Perttunen <mperttunen@nvidia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/thermal.h>
#include <soc/tegra/fuse.h>
#include <soc/tegra/pmc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <dt-bindings/thermal/tegra124-soctherm.h>

#include "tegra_soctherm.h"

#define SENSOR_CONFIG0				0
#define		SENSOR_CONFIG0_STOP		BIT(0)
#define		SENSOR_CONFIG0_TALL_SHIFT	8
#define		SENSOR_CONFIG0_TCALC_OVER	BIT(4)
#define		SENSOR_CONFIG0_OVER		BIT(3)
#define		SENSOR_CONFIG0_CPTR_OVER	BIT(2)
#define SENSOR_CONFIG1				4
#define		SENSOR_CONFIG1_TSAMPLE_SHIFT	0
#define		SENSOR_CONFIG1_TIDDQ_EN_SHIFT	15
#define		SENSOR_CONFIG1_TEN_COUNT_SHIFT	24
#define		SENSOR_CONFIG1_TEMP_ENABLE	BIT(31)
#define SENSOR_CONFIG2				8
#define		SENSOR_CONFIG2_THERMA_SHIFT	16
#define		SENSOR_CONFIG2_THERMB_SHIFT	0


#define SENSOR_PDIV				0x1c0
#define		SENSOR_PDIV_T124		0x8888
#define SENSOR_HOTSPOT_OFF			0x1c4
#define		SENSOR_HOTSPOT_OFF_T124		0x00060600
#define SENSOR_TEMP1				0x1c8
#define		SENSOR_TEMP1_CPU_TEMP_MASK	(0xffff << 16)
#define		SENSOR_TEMP1_GPU_TEMP_MASK	0xffff
#define SENSOR_TEMP2				0x1cc
#define		SENSOR_TEMP2_MEM_TEMP_MASK	(0xffff << 16)
#define		SENSOR_TEMP2_PLLX_TEMP_MASK	0xffff

#define FUSE_TSENSOR8_CALIB			0x180
#define FUSE_SPARE_REALIGNMENT_REG_0		0x1fc

#define FUSE_TSENSOR_CALIB_CP_TS_BASE_MASK	0x1fff
#define FUSE_TSENSOR_CALIB_FT_TS_BASE_MASK	(0x1fff << 13)
#define FUSE_TSENSOR_CALIB_FT_TS_BASE_SHIFT	13

#define FUSE_TSENSOR8_CALIB_CP_TS_BASE_MASK	0x3ff
#define FUSE_TSENSOR8_CALIB_FT_TS_BASE_MASK	(0x7ff << 10)
#define FUSE_TSENSOR8_CALIB_FT_TS_BASE_SHIFT	10

#define FUSE_SPARE_REALIGNMENT_REG_SHIFT_CP_MASK	0x3f
#define FUSE_SPARE_REALIGNMENT_REG_SHIFT_FT_MASK	(0x1f << 21)
#define FUSE_SPARE_REALIGNMENT_REG_SHIFT_FT_SHIFT	21

static s64 div64_s64_precise(s64 a, s64 b)
{
	s64 r, al;

	/* Scale up for increased precision division */
	al = a << 16;

	r = div64_s64(al * 2 + 1, 2 * b);
	return r >> 16;
}

int tegra_soctherm_calculate_shared_calibration(
				struct tsensor_shared_calibration *r,
				u8 nominal_calib_cp,
				u8 nominal_calib_ft)
{
	u32 val;
	u32 shifted_cp, shifted_ft;
	int err;

	err = tegra_fuse_readl(FUSE_TSENSOR8_CALIB, &val);
	if (err)
		return err;
	r->base_cp = val & FUSE_TSENSOR8_CALIB_CP_TS_BASE_MASK;
	r->base_ft = (val & FUSE_TSENSOR8_CALIB_FT_TS_BASE_MASK)
		>> FUSE_TSENSOR8_CALIB_FT_TS_BASE_SHIFT;
	val = ((val & FUSE_SPARE_REALIGNMENT_REG_SHIFT_FT_MASK)
		>> FUSE_SPARE_REALIGNMENT_REG_SHIFT_FT_SHIFT);
	shifted_ft = sign_extend32(val, 4);

	err = tegra_fuse_readl(FUSE_SPARE_REALIGNMENT_REG_0, &val);
	if (err)
		return err;
	shifted_cp = sign_extend32(val, 5);

	r->actual_temp_cp = 2 * nominal_calib_cp + shifted_cp;
	r->actual_temp_ft = 2 * nominal_calib_ft + shifted_ft;

	return 0;
}

int tegra_soctherm_calculate_tsensor_calibration(
				struct tegra_tsensor *sensor,
				struct tsensor_shared_calibration *shared)
{
	struct tegra_tsensor_group *sensor_group;
	u32 val, calib;
	s32 actual_tsensor_ft, actual_tsensor_cp;
	s32 delta_sens, delta_temp;
	s32 mult, div;
	s16 therma, thermb;
	int err;

	sensor_group = sensor->group;

	err = tegra_fuse_readl(sensor->calib_fuse_offset, &val);
	if (err)
		return err;

	actual_tsensor_cp = (shared->base_cp * 64) + sign_extend32(val, 12);
	val = (val & FUSE_TSENSOR_CALIB_FT_TS_BASE_MASK)
		>> FUSE_TSENSOR_CALIB_FT_TS_BASE_SHIFT;
	actual_tsensor_ft = (shared->base_ft * 32) + sign_extend32(val, 12);

	delta_sens = actual_tsensor_ft - actual_tsensor_cp;
	delta_temp = shared->actual_temp_ft - shared->actual_temp_cp;

	mult = sensor_group->pdiv * sensor->config->tsample_ate;
	div = sensor->config->tsample * sensor_group->pdiv_ate;

	therma = div64_s64_precise((s64)delta_temp * (1LL << 13) * mult,
			(s64)delta_sens * div);
	thermb = div64_s64_precise(
			((s64)actual_tsensor_ft * shared->actual_temp_cp) -
			((s64)actual_tsensor_cp * shared->actual_temp_ft),
			(s64)delta_sens);

	therma = div64_s64_precise((s64)therma * sensor->fuse_corr_alpha,
			(s64)1000000LL);
	thermb = div64_s64_precise((s64)thermb * sensor->fuse_corr_alpha +
			sensor->fuse_corr_beta,
			(s64)1000000LL);

	calib = ((u16)therma << SENSOR_CONFIG2_THERMA_SHIFT) |
		 ((u16)thermb << SENSOR_CONFIG2_THERMB_SHIFT);

	sensor->calib = calib;

	return 0;
}

MODULE_AUTHOR("Mikko Perttunen <mperttunen@nvidia.com>");
MODULE_DESCRIPTION("Tegra SOCTHERM fuse management");
MODULE_LICENSE("GPL v2");
