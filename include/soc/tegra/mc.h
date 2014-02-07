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

#include <linux/of.h>

#ifdef CONFIG_TEGRA124_MC
u32 tegra124_mc_readl(u32 offs);
void tegra124_mc_writel(u32 val, u32 offs);
bool tegra124_mc_is_ready(void);
#else
static inline u32 tegra124_mc_readl(u32 offs) { return -ENODEV; }
static inline void tegra124_mc_writel(u32 val, u32 offs) {}
static inline bool tegra124_mc_is_ready(void) { return false; }
#endif

#define TEGRA_MC_CLIENT_AFI		0
#define TEGRA_MC_CLIENT_DC		2
#define TEGRA_MC_CLIENT_DCB		3
#define TEGRA_MC_CLIENT_EPP		4
#define TEGRA_MC_CLIENT_G2		5
#define TEGRA_MC_CLIENT_ISP		8
#define TEGRA_MC_CLIENT_MSENC		11
#define TEGRA_MC_CLIENT_MPE		11
#define TEGRA_MC_CLIENT_NV		12
#define TEGRA_MC_CLIENT_SATA		15
#define TEGRA_MC_CLIENT_VDE		16
#define TEGRA_MC_CLIENT_VI		17
#define TEGRA_MC_CLIENT_VIC		18
#define TEGRA_MC_CLIENT_XUSB_HOST	19
#define TEGRA_MC_CLIENT_XUSB_DEV	20
#define TEGRA_MC_CLIENT_TSEC		22
#define TEGRA_MC_CLIENT_ISPB		33
#define TEGRA_MC_CLIENT_GPU		34

#ifdef CONFIG_TEGRA124_MC
int tegra_mc_flush(int id);
int tegra_mc_flush_done(int id);
#else
static inline int tegra_mc_flush(int id)
{ return 0; }
static inline int tegra_mc_flush_done(int id)
{ return 0; }
#endif

static __maybe_unused inline int tegra_mc_get_effective_bytes_width(void)
{
	if (of_machine_is_compatible("nvidia,tegra124"))
		return 8;
	else
		return 4;
}
#endif /* __SOC_TEGRA_MC_H__ */
