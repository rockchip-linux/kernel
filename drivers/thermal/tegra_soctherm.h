/*
 * XXX
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

#ifndef __DRIVERS_THERMAL_TEGRA_SOCTHERM_H
#define __DRIVERS_THERMAL_TEGRA_SOCTHERM_H

#include <linux/module.h>

#define THERMCTL_LEVEL0_GROUP_CPU		0x0
#define THERMCTL_LEVEL0_GROUP_GPU		0x4
#define THERMCTL_LEVEL0_GROUP_MEM		0x8
#define THERMCTL_LEVEL0_GROUP_TSENSE		0xc

#define THERMTRIP				0x80
#define		THERMTRIP_ANY_EN_SHIFT		28
#define		THERMTRIP_ANY_EN_MASK		0x1
#define		THERMTRIP_MEM_EN_SHIFT		27
#define		THERMTRIP_MEM_EN_MASK		0x1
#define		THERMTRIP_GPU_EN_SHIFT		26
#define		THERMTRIP_GPU_EN_MASK		0x1
#define		THERMTRIP_CPU_EN_SHIFT		25
#define		THERMTRIP_CPU_EN_MASK		0x1
#define		THERMTRIP_TSENSE_EN_SHIFT	24
#define		THERMTRIP_TSENSE_EN_MASK	0x1
#define		THERMTRIP_GPUMEM_THRESH_SHIFT	16
#define		THERMTRIP_GPUMEM_THRESH_MASK	0xff
#define		THERMTRIP_CPU_THRESH_SHIFT	8
#define		THERMTRIP_CPU_THRESH_MASK	0xff
#define		THERMTRIP_TSENSE_THRESH_SHIFT	0
#define		THERMTRIP_TSENSE_THRESH_MASK	0xff

#define SENSOR_PDIV				0x1c0
#define SENSOR_HOTSPOT_OFF			0x1c4

#define SENSOR_TEMP1				0x1c8
#define		SENSOR_TEMP1_CPU_TEMP_MASK	(0xffff << 16)
#define		SENSOR_TEMP1_GPU_TEMP_MASK	0xffff
#define SENSOR_TEMP2				0x1cc
#define		SENSOR_TEMP2_MEM_TEMP_MASK	(0xffff << 16)
#define		SENSOR_TEMP2_PLLX_TEMP_MASK	0xffff

#define NOMINAL_CALIB_FT_T124			105

/*
 * struct tegra_tsensor_group.flags meanings
 */
#define SKIP_THERMAL_FW_REGISTRATION		BIT(0)
#define SKIP_THERMTRIP_REGISTRATION		BIT(1)


/**
 * struct tegra_tsensor_group - SOC_THERM sensor group data
 * @name: short name of the temperature sensor group
 * @id: numeric ID of the temperature sensor group
 * @thermctl_isr_shift: bit shift for interrupt status/enable register
 * @thermctl_level0_offset: offset of the THERMCTL_LEVEL0_GROUP_* reg
 * @thermtrip_enable_shift: register bit shift to enable the THERMTRIP feature
 * @thermtrip_threshold_mask: register mask to program the THERMTRIP threshold
 * @sensor_temp_offset: offset of the SENSOR_TEMP* register
 * @sensor_temp_mask: bit mask for this sensor group in SENSOR_TEMP* register
 * @pllx_hotspot_diff: hotspot offset from the PLLX sensor
 * @pllx_hotspot_mask: register bitfield mask for the HOTSPOT field
 * @pdiv: the sensor count post-divider to use during runtime
 * @pdiv_ate: the sensor count post-divider used during automated test
 * @pdiv_mask: register bitfield mask for the PDIV field for this sensor
 * @flags: any per-tsensor-group flags
 *
 * @pllx_hotspot_diff must be 0 for the PLLX sensor group.
 * The GPU and MEM sensor groups share a thermtrip temperature threshold.
 */
struct tegra_tsensor_group {
	const char	*name;
	u8		id;
	u8		thermctl_isr_shift;
	u8		thermtrip_enable_shift;
	u8		pdiv;
	u8		pdiv_ate;
	u8		flags;
	u16		thermctl_level0_offset;
	u16		sensor_temp_offset;
	u32		sensor_temp_mask;
	u32		thermtrip_threshold_mask;
	u32		pdiv_mask;
	u32		pllx_hotspot_mask;
	int		pllx_hotspot_diff;
};

struct tegra_tsensor_configuration {
	u32 tall, tsample, tiddq_en, ten_count, tsample_ate;
};

struct tegra_tsensor {
	const char *name;
	u32 base;
	struct tegra_tsensor_configuration *config;
	u32 calib_fuse_offset;
	s32 fuse_corr_alpha, fuse_corr_beta;
	u32 calib;
	struct tegra_tsensor_group *group;
};

struct tsensor_shared_calibration {
	u32 base_cp, base_ft;
	u32 actual_temp_cp, actual_temp_ft;
};

int tegra_soctherm_calculate_shared_calibration(
				struct tsensor_shared_calibration *r,
				u8 nominal_calib_cp,
				u8 nominal_calib_ft);
int tegra_soctherm_calculate_tsensor_calibration(
				struct tegra_tsensor *sensor,
				struct tsensor_shared_calibration *shared);
int tegra_soctherm_probe(
		struct platform_device *pdev,
		struct tegra_tsensor *tsensors,
		struct tegra_tsensor_group **tegra_tsensor_groups,
		u8 nominal_calib_cp, u8 nominal_calib_ft,
		bool is_ccroc);
int tegra_soctherm_remove(struct platform_device *pdev);

#endif
