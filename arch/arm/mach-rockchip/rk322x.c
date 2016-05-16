/*
 * Copyright (C) 2015 ROCKCHIP, Inc.
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
#include <linux/rockchip/psci_ddr.h>
#include <asm/cputype.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include "loader.h"
#define CPU 322x
#include "sram.h"
#include <linux/rockchip/cpu.h>
#include <linux/rockchip/psci.h>
#include <linux/wakeup_reason.h>
#include "pm.h"

#define RK322X_DEVICE(name) \
	{ \
		.virtual	= (unsigned long) RK_##name##_VIRT, \
		.pfn		= __phys_to_pfn(RK322X_##name##_PHYS), \
		.length		= RK322X_##name##_SIZE, \
		.type		= MT_DEVICE, \
	}

static const char * const rk3228a_dt_compat[] __initconst = {
	"rockchip,rk3228a",
	NULL,
};

static const char * const rk3228b_dt_compat[] __initconst = {
	"rockchip,rk3228b",
	NULL,
};

static const char * const rk3229_dt_compat[] __initconst = {
	"rockchip,rk3229",
	NULL,
};

static struct map_desc rk322x_io_desc[] __initdata = {
	RK322X_DEVICE(CRU),
	RK322X_DEVICE(GRF),
	RK322X_DEVICE(TIMER),
	RK322X_DEVICE(EFUSE),
	RK322X_DEVICE(CPU_AXI_BUS),
	RK_DEVICE(RK_DEBUG_UART_VIRT, RK322X_UART2_PHYS, RK322X_UART_SIZE),
	RK_DEVICE(RK_DDR_VIRT, RK322X_DDR_PCTL_PHYS, RK322X_DDR_PCTL_SIZE),
	RK_DEVICE(RK_DDR_VIRT + RK322X_DDR_PCTL_SIZE, RK322X_DDR_PHY_PHYS,
		  RK322X_DDR_PHY_SIZE),
	RK_DEVICE(RK_GPIO_VIRT(0), RK322X_GPIO0_PHYS, RK322X_GPIO_SIZE),
	RK_DEVICE(RK_GPIO_VIRT(1), RK322X_GPIO1_PHYS, RK322X_GPIO_SIZE),
	RK_DEVICE(RK_GPIO_VIRT(2), RK322X_GPIO2_PHYS, RK322X_GPIO_SIZE),
	RK_DEVICE(RK_GPIO_VIRT(3), RK322X_GPIO3_PHYS, RK322X_GPIO_SIZE),
	RK_DEVICE(RK_GIC_VIRT, RK322X_GIC_DIST_PHYS, RK322X_GIC_DIST_SIZE),
	RK_DEVICE(RK_GIC_VIRT + RK322X_GIC_DIST_SIZE, RK322X_GIC_CPU_PHYS,
		  RK322X_GIC_CPU_SIZE),
	RK_DEVICE(RK_PWM_VIRT, RK322X_PWM_PHYS, RK322X_PWM_SIZE),
};

static void __init rk322x_boot_mode_init(void)
{
	u32 flag = readl_relaxed(RK_GRF_VIRT + RK322X_GRF_OS_REG0);
	u32 mode = readl_relaxed(RK_GRF_VIRT + RK322X_GRF_OS_REG1);
	u32 rst_st = readl_relaxed(RK_CRU_VIRT + RK322X_CRU_GLB_RST_ST);

	if (flag == (SYS_KERNRL_REBOOT_FLAG | BOOT_RECOVER))
		mode = BOOT_MODE_RECOVERY;
	if (rst_st & ((1 << 2) | (1 << 3)))
		mode = BOOT_MODE_WATCHDOG;

	rockchip_boot_mode_init(flag, mode);
}

static void __init rk322x_dt_map_io(void)
{
	iotable_init(rk322x_io_desc, ARRAY_SIZE(rk322x_io_desc));
	debug_ll_io_init();

	rk322x_boot_mode_init();
	rockchip_efuse_init();
}

static void __init rk3228a_dt_map_io(void)
{
	rockchip_soc_id = ROCKCHIP_SOC_RK3228A;
	rk322x_dt_map_io();
}

static void __init rk3228b_dt_map_io(void)
{
	rockchip_soc_id = ROCKCHIP_SOC_RK3228B;
	rk322x_dt_map_io();
}

static void __init rk3229_dt_map_io(void)
{
	rockchip_soc_id = ROCKCHIP_SOC_RK3229;
	rk322x_dt_map_io();
}

static void __init rk322x_dt_init_timer(void)
{
	of_clk_init(NULL);
	clocksource_of_init();
	of_dvfs_init();
}

static void __init rk322x_reserve(void)
{
	/* reserve memory for uboot */
	rockchip_uboot_mem_reserve();

	/* reserve memory for ION */
	rockchip_ion_reserve();
}

static void __init rk322x_suspend_init(void)
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

	rockchip_psci_smc_write(PSCI_SIP_SUSPEND_WR_CTRBITS,
				pm_ctrbits, 0, SEC_REG_WR);
	PM_LOG("%s: pm_ctrbits = 0x%x\n", __func__, pm_ctrbits);
}

static void __init rk322x_init_ddrfreq_func(void)
{
	ddr_change_freq = (void *)psci_ddr_change_freq;
	ddr_set_auto_self_refresh = (void *)psci_ddr_set_auto_self_refresh;
}

#define GICD_ISPEND		0x200
#define GPIO_NUMS		4
#define GPIO_INTEN		0x30
#define GPIO_INT_STATUS		0x40

#define DUMP_GPIO_INTEN(ID) \
	do { \
		u32 en = readl_relaxed(RK_GPIO_VIRT(ID) + GPIO_INTEN); \
		if (en) { \
			printk("GPIO%d_INTEN: %08x\n", ID, en); \
		} \
	} while (0)

static void rk322x_irq_prepare(void)
{
	DUMP_GPIO_INTEN(0);
	DUMP_GPIO_INTEN(1);
	DUMP_GPIO_INTEN(2);
	DUMP_GPIO_INTEN(3);
}

static void rk322x_irq_finish(void)
{
	u32 irq_gpio, irq[4], val, i;

	/* gpio0~3, irq: 83, 84, 85, 86 */
	val = readl_relaxed(RK_GIC_VIRT + GICD_ISPEND + 8);
	irq_gpio = (val >> 19) & 0x0f;

	for (i = 0; i < ARRAY_SIZE(irq); i++)
		irq[i] = readl_relaxed(RK_GIC_VIRT + GICD_ISPEND + (1 + i) * 4);

	for (i = 0; i < ARRAY_SIZE(irq); i++) {
		if (irq[i])
			log_wakeup_reason(32 * (i + 1) + fls(irq[i]) - 1);
	}

	printk("wakeup irq: %08x %08x %08x %08x\n",
	       irq[3], irq[2], irq[1], irq[0]);

	for (i = 0; i < GPIO_NUMS; i++) {
		if (irq_gpio & (1 << i))
			printk("wakeup gpio%d: %08x\n", i,
			       readl_relaxed(RK_GPIO_VIRT(i) + GPIO_INT_STATUS));
	}
}

static void __init rk322x_init_late(void)
{
	if (rockchip_jtag_enabled)
		clk_prepare_enable(clk_get_sys(NULL, "clk_jtag"));

	rk322x_suspend_init();
	rockchip_suspend_init();
	rk322x_init_ddrfreq_func();
	rkpm_set_ops_prepare_finish(rk322x_irq_prepare, rk322x_irq_finish);
}

static void rk322x_restart(char mode, const char *cmd)
{
	u32 boot_flag, boot_mode;

	rockchip_restart_get_boot_mode(cmd, &boot_flag, &boot_mode);

	/* for loader */
	writel_relaxed(boot_flag, RK_GRF_VIRT + RK322X_GRF_OS_REG0);
	/* for linux */
	writel_relaxed(boot_mode, RK_GRF_VIRT + RK322X_GRF_OS_REG1);

	dsb();

	/* pll enter slow mode */
	writel_relaxed(0x11010000, RK_CRU_VIRT + RK322X_CRU_MODE_CON);
	dsb();
	writel_relaxed(0xeca8, RK_CRU_VIRT + RK322X_CRU_GLB_SRST_SND_VALUE);
	dsb();
}

DT_MACHINE_START(RK3228A_DT, "Rockchip RK3228A")
	.smp		= smp_ops(rockchip_smp_ops),
	.map_io		= rk3228a_dt_map_io,
	.init_time	= rk322x_dt_init_timer,
	.dt_compat	= rk3228a_dt_compat,
	.init_late	= rk322x_init_late,
	.reserve	= rk322x_reserve,
	.restart	= rk322x_restart,
MACHINE_END

DT_MACHINE_START(RK3228B_DT, "Rockchip RK3228B")
	.smp		= smp_ops(rockchip_smp_ops),
	.map_io		= rk3228b_dt_map_io,
	.init_time	= rk322x_dt_init_timer,
	.dt_compat	= rk3228b_dt_compat,
	.init_late	= rk322x_init_late,
	.reserve	= rk322x_reserve,
	.restart	= rk322x_restart,
MACHINE_END

DT_MACHINE_START(RK3229_DT, "Rockchip RK3229")
	.smp		= smp_ops(rockchip_smp_ops),
	.map_io		= rk3229_dt_map_io,
	.init_time	= rk322x_dt_init_timer,
	.dt_compat	= rk3229_dt_compat,
	.init_late	= rk322x_init_late,
	.reserve	= rk322x_reserve,
	.restart	= rk322x_restart,
MACHINE_END
