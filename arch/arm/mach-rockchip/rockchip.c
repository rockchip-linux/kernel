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
#include <linux/of_fdt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/irqchip.h>
#include <linux/memblock.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/cache-l2x0.h>
#include "core.h"
#include "pm.h"

static void __init rockchip_dt_init(void)
{
	l2x0_of_init(0, ~0UL);
	rockchip_suspend_init();
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	platform_device_register_simple("cpufreq-cpu0", 0, NULL, 0);
}

extern struct ion_platform_data ion_pdata;
extern void __init ion_reserve(struct ion_platform_data *data);
extern int __init rockchip_ion_find_heap(unsigned long node,
				const char *uname, int depth, void *data);
void __init rockchip_ion_reserve(void)
{
#ifdef CONFIG_ION_ROCKCHIP
	printk("%s\n", __func__);
	of_scan_flat_dt(rockchip_ion_find_heap, (void*)&ion_pdata);
	ion_reserve(&ion_pdata);
#endif
}
static void __init rockchip_memory_init(void)
{
	memblock_reserve(0xfe000000, 0x1000000);
	/* reserve memory for ION */
	rockchip_ion_reserve();
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
