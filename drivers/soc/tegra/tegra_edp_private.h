/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __TEGRA_EDP_PRIVATE_H
#define __TEGRA_EDP_PRIVATE_H

#include <linux/debugfs.h>

#define MAX_CPU_CORES	4

struct tegra_edp_limits {
	int temperature;
	unsigned int freq_limits[MAX_CPU_CORES];
};

struct tegra_edp_voltage_temp_constraint {
	int temperature;
	unsigned int voltage_limit_mv;
};

struct tegra_edp_cpu_leakage_params {
	int cpu_speedo_id;

	unsigned int temp_scaled;

	unsigned int dyn_scaled;
	int dyn_consts_n[MAX_CPU_CORES];     /* pre-multiplied by 'scaled */

	unsigned int consts_scaled;
	int leakage_consts_n[MAX_CPU_CORES]; /* pre-multiplied by 'scaled */

	unsigned int ijk_scaled;
	const int (*leakage_consts_ijk)[4][4]; /* pre-multiplied by 'scaled */
	unsigned int leakage_min;        /* minimum leakage current */

	unsigned int safety_cap[MAX_CPU_CORES];
	struct tegra_edp_voltage_temp_constraint volt_temp_cap;
};

struct tegra_edp_freq_vol_table {
	unsigned int freq;
	int voltage_mv;
};

struct tegra_edp {
	struct platform_device *pdev;

	struct tegra_edp_limits *def_edp_limits;
	int def_edp_limits_size;
	const int *temps;
	int temps_size;
	const struct tegra_edp_cpu_leakage_params *params;
	int params_size;
	unsigned int cpu_speedo_idx;

	struct tegra_edp_limits *edp_limits;
	int edp_limits_size;

	unsigned int reg_edp_ma;

	unsigned int max_cpu_freq;

	int edp_thermal_index;
	unsigned int edp_freq_limit;
	cpumask_t edp_cpumask;

	bool suspended;

#ifdef CONFIG_DEBUG_FS
	struct dentry *dir;
#endif

	bool edp_init_done;
};

struct tegra_cpu_edp_cooling_data {
	const struct tegra_edp_limits *cpu_edp_limits;
	int cpu_edp_limits_size;

	int edp_thermal_index;
	cpumask_t edp_cpumask;
	unsigned int edp_limit;
};

#endif	/* __TEGRA_EDP_PRIVATE_H */
