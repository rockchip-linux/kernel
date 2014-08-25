/*
 * Copyright (C) 2014 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SOC_TEGRA_PM_H__
#define __SOC_TEGRA_PM_H__

enum tegra_suspend_mode {
	TEGRA_SUSPEND_NONE = 0,
	TEGRA_SUSPEND_LP2, /* CPU voltage off */
	TEGRA_SUSPEND_LP1, /* CPU voltage off, DRAM self-refresh */
	TEGRA_SUSPEND_LP0, /* CPU + core voltage off, DRAM self-refresh */
	TEGRA_MAX_SUSPEND_MODE,
};

struct tegra_lp1_iram {
	void	*start_addr;
	void	*end_addr;
};

extern struct tegra_lp1_iram tegra_lp1_iram;
extern void (*tegra_sleep_core_finish)(unsigned long v2p);

void tegra20_lp1_iram_hook(void);
void tegra20_sleep_core_init(void);
void tegra30_lp1_iram_hook(void);
void tegra30_sleep_core_init(void);

extern unsigned long l2x0_saved_regs_addr;

void tegra_clear_cpu_in_lp2(void);
bool tegra_set_cpu_in_lp2(void);

void tegra_idle_lp2_last(void);
extern void (*tegra_tear_down_cpu)(void);

#ifdef CONFIG_PM_SLEEP
void tegra_smp_clear_cpu_init_mask(void);
enum tegra_suspend_mode
tegra_pm_validate_suspend_mode(enum tegra_suspend_mode mode);

/* low-level resume entry point */
void tegra_resume(void);
#else
static inline enum tegra_suspend_mode
tegra_pm_validate_suspend_mode(enum tegra_suspend_mode mode)
{
	return TEGRA_SUSPEND_NONE;
}

static inline void tegra_resume(void)
{
}
#endif /* CONFIG_PM_SLEEP */

#endif /* __SOC_TEGRA_PM_H__ */
