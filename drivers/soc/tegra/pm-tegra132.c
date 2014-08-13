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

#include <soc/tegra/common.h>
#include <soc/tegra/flowctrl.h>
#include <soc/tegra/fuse.h>
#include <soc/tegra/iomap.h>
#include <soc/tegra/pmc.h>

#include <asm/smp_spin_table.h>

#define NS_RST_VEC_WR_DIS	0x2
#define TEGRA132_PM_CORE_C7	0x3

#ifdef CONFIG_HOTPLUG_CPU
static int tegra132_cpu_off(unsigned long cpu)
{
	unsigned long pmstate = TEGRA132_PM_CORE_C7;

	flowctrl_cpu_suspend_enter(cpu);
	flowctrl_write_cpu_halt(cpu, FLOW_CTRL_WAITEVENT);

	do {
		asm volatile(
		"       isb\n"
		"       msr actlr_el1, %0\n"
		"       wfi\n"
		:
		: "r" (pmstate));
	} while (0);

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

static struct spin_table_soc_ops tegra_soc_ops = {
	.cpu_off = tegra132_cpu_off,
	.cpu_on = tegra132_cpu_on,
};

static void tegra132_cpu_hotplug_init(void)
{
	smp_spin_table_set_soc_ops(&tegra_soc_ops);
}
#endif

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

#ifdef CONFIG_HOTPLUG_CPU
	tegra132_cpu_hotplug_init();
#endif

out:
	return 0;
}
early_initcall(tegra132_pm_init);
