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

#ifndef __TEGRA_EDP_H
#define __TEGRA_EDP_H

#include <linux/thermal.h>

extern int tegra_cpu_edp_get_thermal_index(struct platform_device *pdev);
extern int tegra_cpu_edp_count_therm_floors(struct platform_device *pdev);
extern int tegra_cpu_edp_update_thermal_index(struct platform_device *pdev,
					      unsigned long new_idx);

#ifdef CONFIG_TEGRA_CPU_EDP_LIMITS
extern bool tegra_cpu_edp_ready(void);
#else
static inline bool tegra_cpu_edp_ready(void)
{ return false; }
#endif

#endif	/* __TEGRA_EDP_H */
