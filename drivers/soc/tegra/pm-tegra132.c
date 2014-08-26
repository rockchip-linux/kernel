/*
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk/tegra.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/smp.h>

#include <soc/tegra/common.h>
#include <soc/tegra/flowctrl.h>
#include <soc/tegra/fuse.h>
#include <soc/tegra/iomap.h>
#include <soc/tegra/pm.h>
#include <soc/tegra/pmc.h>

#include <asm/smp_spin_table.h>
#include <asm/suspend.h>

#define NS_RST_VEC_WR_DIS	0x2
#define TEGRA132_PM_CORE_C7	0x3
#define TEGRA132_PM_SYSTEM_LP0	0xd

#define HALT_REG_CORE0 (\
	FLOW_CTRL_WAIT_FOR_INTERRUPT | \
	FLOW_CTRL_HALT_LIC_IRQ | \
	FLOW_CTRL_HALT_LIC_FIQ)

#define HALT_REG_CORE1 FLOW_CTRL_WAITEVENT

static void tegra132_enter_sleep(unsigned long pmstate)
{
	u32 reg;
	int cpu = smp_processor_id();

	reg = cpu ? HALT_REG_CORE1 : HALT_REG_CORE0;
	flowctrl_cpu_suspend_enter(cpu);
	flowctrl_write_cpu_halt(cpu, reg);

	do {
		asm volatile(
		"       isb\n"
		"       msr actlr_el1, %0\n"
		"       wfi\n"
		:
		: "r" (pmstate));
	} while (0);
}

#ifdef CONFIG_PM_SLEEP
static void tegra132_sleep_core_finish(unsigned long arg)
{
	tegra132_enter_sleep(TEGRA132_PM_SYSTEM_LP0);
}

static int tegra132_cpu_suspend(unsigned long arg)
{
	if (arg != TEGRA_SUSPEND_LP0)
		return -EOPNOTSUPP;
	return __cpu_suspend(arg, &tegra_sleep_core);
}

void tegra132_sleep_core_init(void)
{
	tegra_sleep_core_finish = tegra132_sleep_core_finish;
}
#endif

#ifdef CONFIG_HOTPLUG_CPU
static int tegra132_cpu_off(unsigned long cpu)
{
	tegra132_enter_sleep(TEGRA132_PM_CORE_C7);

	return 0;
}

static int tegra132_cpu_on(unsigned long cpu)
{
	int ret;

	if (!tegra_pmc_cpu_is_powered(cpu)) {
		ret = tegra_pmc_cpu_power_on(cpu);
		if (ret)
			return ret;
	}
	flowctrl_write_cpu_halt(cpu, 0);
	tegra_cpu_out_of_reset(cpu);

	return 0;
}
#endif

static struct spin_table_soc_ops tegra_soc_ops = {
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_off = tegra132_cpu_off,
	.cpu_on = tegra132_cpu_on,
#endif
#ifdef CONFIG_PM_SLEEP
	.cpu_suspend = tegra132_cpu_suspend,
#endif
};

static void tegra132_cpu_pm_init(void)
{
	smp_spin_table_set_soc_ops(&tegra_soc_ops);
}

static int __init tegra132_pm_init(void)
{
	extern void *__aarch64_tramp;
	void __iomem *evp_cpu_reset;
	void __iomem *sb_ctrl;
	unsigned long reg;

	if (!soc_is_tegra() || tegra_get_chip_id() != TEGRA132)
		goto out;

	evp_cpu_reset = ioremap(TEGRA_EXCEPTION_VECTORS_BASE + 0x100,
				sizeof(u32));
	sb_ctrl = ioremap(TEGRA_SB_BASE, sizeof(u32));

	writel(virt_to_phys(&__aarch64_tramp), evp_cpu_reset);
	wmb();
	reg = readl(evp_cpu_reset);

	/* Prevent further modifications to the physical reset vector. */
	reg = readl(sb_ctrl);
	reg |= NS_RST_VEC_WR_DIS;
	writel(reg, sb_ctrl);
	wmb();

	iounmap(sb_ctrl);
	iounmap(evp_cpu_reset);

	tegra132_cpu_pm_init();
out:
	return 0;
}
early_initcall(tegra132_pm_init);
