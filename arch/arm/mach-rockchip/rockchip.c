/*
 * Device Tree support for Rockchip SoCs
 *
 * Copyright (c) 2013 MundoReader S.L.
 * Author: Heiko Stuebner <heiko@sntech.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/irqchip.h>
#include <linux/memblock.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/cache-l2x0.h>
#include "core.h"
#include "pm.h"

static void __iomem *rockchip_cpu_debug[4];
static struct clk *pclk_dbg;
static struct clk *pclk_core_niu;

#define RK3288_DEBUG_PA_CPU(x)		(0xffbb0000 + (x * 0x2000))
#define CPU_DBGPCSR			0xa0

int rockchip_panic_notify(struct notifier_block *nb, unsigned long event,
			 void *p)
{
	unsigned long dbgpcsr;
	int i;
	void *pc = NULL;

	clk_enable(pclk_dbg);
	clk_enable(pclk_core_niu);

	for_each_online_cpu(i) {
		dbgpcsr = readl_relaxed(rockchip_cpu_debug[i] + CPU_DBGPCSR);

		/* NOTE: no offset on A17; see DBGDEVID1.PCSROffset */
		pc = (void *)(dbgpcsr & ~1);

		pr_err("CPU%d PC: <%p> %pF\n", i, pc, pc);
	}
	return NOTIFY_OK;
}

struct notifier_block rockchip_panic_nb = {
	.notifier_call = rockchip_panic_notify,
	.priority = INT_MAX,
};

static void __init rockchip_panic_init(void)
{
	int i;

	/* These two clocks appear to be needed to access regs */
	pclk_dbg = clk_get(NULL, "pclk_dbg");
	if (WARN_ON(IS_ERR(pclk_dbg)))
		return;
	clk_prepare(pclk_dbg);

	pclk_core_niu = clk_get(NULL, "pclk_core_niu");
	if (WARN_ON(IS_ERR(pclk_core_niu)))
		return;
	clk_prepare(pclk_core_niu);

	for (i = 0; i < 4; i++)
		rockchip_cpu_debug[i] = ioremap(RK3288_DEBUG_PA_CPU(i), SZ_4K);

	atomic_notifier_chain_register(&panic_notifier_list,
				       &rockchip_panic_nb);
}

static void __init rockchip_dt_init(void)
{
	l2x0_of_init(0, ~0UL);
	rockchip_suspend_init();
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	platform_device_register_simple("cpufreq-cpu0", 0, NULL, 0);

	/* HACKY (and rk3288-specific) panic notifier */
	rockchip_panic_init();
}

static void __init rockchip_memory_init(void)
{
	memblock_reserve(0xfe000000, 0x1000000);
}

static const char * const rockchip_board_dt_compat[] = {
	"rockchip,rk2928",
	"rockchip,rk3066a",
	"rockchip,rk3066b",
	"rockchip,rk3188",
	"rockchip,rk3288",
	NULL,
};

DT_MACHINE_START(ROCKCHIP_DT, "Rockchip (Device Tree)")
	.init_machine	= rockchip_dt_init,
	.dt_compat	= rockchip_board_dt_compat,
	.reserve        = rockchip_memory_init,
MACHINE_END
