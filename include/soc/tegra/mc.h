/*
 * Copyright (c) 2010 Google, Inc
 * Copyright (c) 2014 NVIDIA Corporation
 *
 * Author:
 *	Colin Cross <ccross@google.com>
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

#ifndef __SOC_TEGRA_MC_H__
#define __SOC_TEGRA_MC_H__

#ifdef CONFIG_TEGRA124_MC
u32 tegra124_mc_readl(u32 offs);
void tegra124_mc_writel(u32 val, u32 offs);
#else
static inline u32 tegra124_mc_readl(u32 offs) { return -ENODEV; }
static inline void tegra124_mc_writel(u32 val, u32 offs) {}
#endif

#endif /* __SOC_TEGRA_MC_H__ */
