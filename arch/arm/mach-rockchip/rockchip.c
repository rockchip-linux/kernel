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
#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/irqchip.h>
#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/cache-l2x0.h>
#include "core.h"
#include "pm.h"

#define RK3036_TIMER_PHYS 0x20044000

#define RK3288_GRF_SOC_CON0 0x244
#define RK3288_TIMER6_7_PHYS 0xff810000

static void rockchip_init_arch_timer_supply(resource_size_t phys, int offs)
{
	void __iomem *reg_base = ioremap(phys, SZ_16K);

	/*
	 * Most/all uboot versions for Rockchip SoCs don't enable
	 * timer which is needed for the architected timer to work.
	 * So make sure it is running during early boot.
	 */
	if (reg_base) {
		writel(0, reg_base + offs + 0x10);
		writel(0xffffffff, reg_base + offs);
		writel(0xffffffff, reg_base + offs + 0x04);
		writel(1, reg_base + offs + 0x10);
		dsb();
		iounmap(reg_base);
	} else {
		pr_err("rockchip: could not map timer registers\n");
	}
}

static void __init rockchip_timer_init(void)
{
	if (of_machine_is_compatible("rockchip,rk3288")) {
		struct regmap *grf;

		rockchip_init_arch_timer_supply(RK3288_TIMER6_7_PHYS, 0x20);

		/*
		 * Disable auto jtag/sdmmc switching that causes issues
		 * with the mmc controllers making them unreliable
		 */
		grf = syscon_regmap_lookup_by_compatible("rockchip,rk3288-grf");
		if (!IS_ERR(grf))
			regmap_write(grf, RK3288_GRF_SOC_CON0, 0x10000000);
		else
			pr_err("rockchip: could not get grf syscon\n");
	} else if (of_machine_is_compatible("rockchip,rk3036")) {
		rockchip_init_arch_timer_supply(RK3036_TIMER_PHYS, 0xa0);
	}

	of_clk_init(NULL);
	clocksource_probe();
}

static void __init rockchip_dt_init(void)
{
	rockchip_suspend_init();
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	platform_device_register_simple("cpufreq-dt", 0, NULL, 0);
}

static const char * const rockchip_board_dt_compat[] = {
	"rockchip,rk2928",
	"rockchip,rk3036",
	"rockchip,rk3066a",
	"rockchip,rk3066b",
	"rockchip,rk3188",
	"rockchip,rk3288",
	NULL,
};

DT_MACHINE_START(ROCKCHIP_DT, "Rockchip (Device Tree)")
	.l2c_aux_val	= 0,
	.l2c_aux_mask	= ~0,
	.init_time	= rockchip_timer_init,
	.dt_compat	= rockchip_board_dt_compat,
	.init_machine	= rockchip_dt_init,
MACHINE_END
