/*
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _TEGRA_DFLL_H_
#define _TEGRA_DFLL_H_

#include <linux/kernel.h>
#include <linux/platform_device.h>

enum tegra_dfll_thermal_type {
	TEGRA_DFLL_THERMAL_FLOOR = 0,
	TEGRA_DFLL_THERMAL_CAP,
};

extern int tegra_dfll_update_thermal_index(struct platform_device *pdev,
			enum tegra_dfll_thermal_type type,
			unsigned long new_index);
extern int tegra_dfll_get_thermal_index(struct platform_device *pdev,
			enum tegra_dfll_thermal_type type);
extern int tegra_dfll_count_thermal_states(struct platform_device *pdev,
			enum tegra_dfll_thermal_type type);
#endif
