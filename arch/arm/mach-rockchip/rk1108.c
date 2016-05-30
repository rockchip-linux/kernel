/*
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd
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

#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/cpuidle.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/irqchip.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/rockchip/common.h>
#include <linux/rockchip/cpu.h>
#include <linux/rockchip/cpu_axi.h>
#include <linux/rockchip/cru.h>
#include <linux/rockchip/dvfs.h>
#include <linux/rockchip/grf.h>
#include <linux/rockchip/iomap.h>
#include <linux/rockchip/pmu.h>
#include <asm/cputype.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include "loader.h"
#define CPU 1108
#include "sram.h"
#include <linux/rockchip/cpu.h>
#include "pm.h"

#define RK1108_DEVICE(name) \
	{ \
		.virtual	= (unsigned long) RK_##name##_VIRT, \
		.pfn		= __phys_to_pfn(RK1108_##name##_PHYS), \
		.length		= RK1108_##name##_SIZE, \
		.type		= MT_DEVICE, \
	}

static struct map_desc rk1108_io_desc[] __initdata = {
	RK1108_DEVICE(CRU),
	RK1108_DEVICE(GRF),
	RK1108_DEVICE(TIMER),
	RK1108_DEVICE(EFUSE),
	RK1108_DEVICE(CPU_AXI_BUS),
	RK_DEVICE(RK_DEBUG_UART_VIRT, RK1108_UART2_PHYS, RK1108_UART_SIZE),
	RK_DEVICE(RK_DDR_VIRT, RK1108_DDR_PCTL_PHYS, RK1108_DDR_PCTL_SIZE),
	RK_DEVICE(RK_DDR_VIRT + RK1108_DDR_PCTL_SIZE, RK1108_DDR_PHY_PHYS,
		  RK1108_DDR_PHY_SIZE),
	RK_DEVICE(RK_GPIO_VIRT(0), RK1108_GPIO0_PHYS, RK1108_GPIO_SIZE),
	RK_DEVICE(RK_GPIO_VIRT(1), RK1108_GPIO1_PHYS, RK1108_GPIO_SIZE),
	RK_DEVICE(RK_GPIO_VIRT(2), RK1108_GPIO2_PHYS, RK1108_GPIO_SIZE),
	RK_DEVICE(RK_GPIO_VIRT(3), RK1108_GPIO3_PHYS, RK1108_GPIO_SIZE),
	RK_DEVICE(RK_GIC_VIRT, RK1108_GIC_DIST_PHYS, RK1108_GIC_DIST_SIZE),
	RK_DEVICE(RK_GIC_VIRT + RK1108_GIC_DIST_SIZE, RK1108_GIC_CPU_PHYS,
		  RK1108_GIC_CPU_SIZE),
	RK_DEVICE(RK_PWM_VIRT, RK1108_PWM_PHYS, RK1108_PWM_SIZE),
};

static void __init rk1108_boot_mode_init(void)
{
	u32 flag = readl_relaxed(RK_GRF_VIRT + RK1108_GRF_OS_REG0);
	u32 mode = readl_relaxed(RK_GRF_VIRT + RK1108_GRF_OS_REG1);
	u32 rst_st = readl_relaxed(RK_CRU_VIRT + RK1108_CRU_GLB_RST_ST);

	if (flag == (SYS_KERNRL_REBOOT_FLAG | BOOT_RECOVER))
		mode = BOOT_MODE_RECOVERY;
	if (rst_st & ((1 << 2) | (1 << 3)))
		mode = BOOT_MODE_WATCHDOG;

	rockchip_boot_mode_init(flag, mode);
}

static void __init rk1108_dt_map_io(void)
{
	rockchip_soc_id = ROCKCHIP_SOC_RK1108;

	iotable_init(rk1108_io_desc, ARRAY_SIZE(rk1108_io_desc));
	debug_ll_io_init();

	rk1108_boot_mode_init();
	rockchip_efuse_init();
}

static void __init rk1108_dt_init_timer(void)
{
	of_clk_init(NULL);
	clocksource_of_init();
	of_dvfs_init();
}

static void __init rk1108_reserve(void)
{
	/* reserve memory for ION */
	rockchip_ion_reserve();
}

static void __init rk1108_suspend_init(void)
{
	struct device_node *parent;
	u32 pm_ctrbits = 0;

	parent = of_find_node_by_name(NULL, "rockchip_suspend");
	if (IS_ERR_OR_NULL(parent)) {
		PM_ERR("%s dev node err\n", __func__);
		return;
	}

	if (of_property_read_u32_array(parent, "rockchip,ctrbits",
				       &pm_ctrbits, 1)) {
		PM_ERR("%s: read rockchip ctrbits error\n", __func__);
		return;
	}

	/* TODO some suspend code should be done */
	PM_LOG("%s: pm_ctrbits = 0x%x\n", __func__, pm_ctrbits);
}

static void __init rk1108_init_late(void)
{
	if (rockchip_jtag_enabled)
		clk_prepare_enable(clk_get_sys(NULL, "clk_jtag"));

	rk1108_suspend_init();
	rockchip_suspend_init();
}

static void rk1108_restart(char mode, const char *cmd)
{
	u32 boot_flag, boot_mode;

	rockchip_restart_get_boot_mode(cmd, &boot_flag, &boot_mode);

	/* for loader */
	writel_relaxed(boot_flag, RK_GRF_VIRT + RK1108_GRF_OS_REG0);
	/* for linux */
	writel_relaxed(boot_mode, RK_GRF_VIRT + RK1108_GRF_OS_REG1);

	dsb();

	/* pll enter slow mode */
	writel_relaxed(0x01000000, RK_CRU_VIRT + RK1108_CRU_APLL_CON3);
	writel_relaxed(0x01000000, RK_CRU_VIRT + RK1108_CRU_GPLL_CON3);
	dsb();
	writel_relaxed(0xfdb9, RK_CRU_VIRT + RK1108_CRU_GLB_SRST_FST_VALUE);
	dsb();
}

static const char * const rk1108_dt_compat[] __initconst = {
	"rockchip,rk1108",
	NULL,
};

DT_MACHINE_START(RK1108_DT, "Rockchip RK1108")
	.smp		= smp_ops(rockchip_smp_ops),
	.map_io		= rk1108_dt_map_io,
	.init_time	= rk1108_dt_init_timer,
	.dt_compat	= rk1108_dt_compat,
	.init_late	= rk1108_init_late,
	.reserve	= rk1108_reserve,
	.restart	= rk1108_restart,
MACHINE_END
