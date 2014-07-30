/*
 * Copyright (C) 2011 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@android.com>
 *	Olof Johansson <olof@lixom.net>
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

#ifndef __TEGRA_EMC_H_
#define __TEGRA_EMC_H_

#include <soc/tegra/mc.h>

#define TEGRA_EMC_NUM_REGS 46

struct tegra_emc_table {
	unsigned long rate;
	u32 regs[TEGRA_EMC_NUM_REGS];
};

struct tegra_emc_pdata {
	int num_tables;
	struct tegra_emc_table *tables;
};

struct emc_clk_ops {
	long		(*emc_round_rate)(unsigned long);
	int		(*emc_set_rate)(unsigned long);
	unsigned long	(*emc_get_rate)(void);
	struct clk *	(*emc_predict_parent)(unsigned long, unsigned long *);
	void		(*emc_get_backup_parent)(struct clk **,
							unsigned long *);
};

#ifdef CONFIG_TEGRA124_EMC
void tegra124_emc_timing_invalidate(void);
bool tegra124_emc_is_ready(void);
unsigned long tegra124_predict_emc_rate(int millivolts);
const struct emc_clk_ops *tegra124_emc_get_ops(void);
#else
static inline void tegra124_emc_timing_invalidate(void) { return; };
static inline bool tegra124_emc_is_ready(void) { return true; };
static inline unsigned long tegra124_predict_emc_rate(int millivolts)
{ return -ENODEV; }
static inline const struct emc_clk_ops *tegra124_emc_get_ops(void)
{ return NULL; }
#endif

static __maybe_unused unsigned int
tegra_emc_bw_to_freq_req(unsigned int bw_kbps)
{
	unsigned int freq;
	unsigned int bytes_per_emc_clk;

	bytes_per_emc_clk = tegra_mc_get_effective_bytes_width() * 2;
	/* EMC_TO_DDR_CLOCK is 1 */
	freq = (bw_kbps + bytes_per_emc_clk - 1) / bytes_per_emc_clk;
	return freq;
}

static __maybe_unused unsigned int
tegra_emc_freq_req_to_bw(unsigned int freq_khz)
{
	unsigned int bw;
	unsigned int bytes_per_emc_clk;

	bytes_per_emc_clk = tegra_mc_get_effective_bytes_width() * 2;
	/* EMC_TO_DDR_CLOCK is 1 */
	bw = freq_khz * bytes_per_emc_clk;
	return bw;
}

#endif
