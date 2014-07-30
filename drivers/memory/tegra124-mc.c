/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of_address.h>

#include <soc/tegra/mc.h>

#define DRV_NAME "tegra124-mc"

static void __iomem *tegra_mc_base;
static bool tegra_mc_init_done;

#define MC_CLIENT_HOTRESET_CTRL		0x200
#define MC_CLIENT_HOTRESET_STAT		0x204
#define MC_CLIENT_HOTRESET_CTRL_1	0x970
#define MC_CLIENT_HOTRESET_STAT_1	0x974

static DEFINE_SPINLOCK(tegra_mc_lock);

u32 tegra124_mc_readl(u32 offs)
{
	return readl(tegra_mc_base + offs);
}
EXPORT_SYMBOL(tegra124_mc_readl);

void tegra124_mc_writel(u32 val, u32 offs)
{
	writel(val, tegra_mc_base + offs);
}
EXPORT_SYMBOL(tegra124_mc_writel);

bool tegra124_mc_is_ready(void)
{
	return tegra_mc_init_done;
}
EXPORT_SYMBOL(tegra124_mc_is_ready);

#define HOTRESET_READ_COUNT	5
static bool tegra_stable_hotreset_check(u32 stat_reg, u32 *stat)
{
	int i;
	u32 cur_stat;
	u32 prv_stat;
	unsigned long flags;

	spin_lock_irqsave(&tegra_mc_lock, flags);
	prv_stat = tegra124_mc_readl(stat_reg);
	for (i = 0; i < HOTRESET_READ_COUNT; i++) {
		cur_stat = tegra124_mc_readl(stat_reg);
		if (cur_stat != prv_stat) {
			spin_unlock_irqrestore(&tegra_mc_lock, flags);
			return false;
		}
	}
	*stat = cur_stat;
	spin_unlock_irqrestore(&tegra_mc_lock, flags);
	return true;
}

int tegra_mc_flush(int id)
{
	u32 rst_ctrl, rst_stat;
	u32 rst_ctrl_reg, rst_stat_reg;
	unsigned long flags;
	bool ret;

	if (id < 32) {
		rst_ctrl_reg = MC_CLIENT_HOTRESET_CTRL;
		rst_stat_reg = MC_CLIENT_HOTRESET_STAT;
	} else {
		id %= 32;
		rst_ctrl_reg = MC_CLIENT_HOTRESET_CTRL_1;
		rst_stat_reg = MC_CLIENT_HOTRESET_STAT_1;
	}

	spin_lock_irqsave(&tegra_mc_lock, flags);

	rst_ctrl = tegra124_mc_readl(rst_ctrl_reg);
	rst_ctrl |= (1 << id);
	tegra124_mc_writel(rst_ctrl, rst_ctrl_reg);

	spin_unlock_irqrestore(&tegra_mc_lock, flags);

	do {
		udelay(10);
		rst_stat = 0;
		ret = tegra_stable_hotreset_check(rst_stat_reg, &rst_stat);
		if (!ret)
			continue;
	} while (!(rst_stat & (1 << id)));

	return 0;
}
EXPORT_SYMBOL(tegra_mc_flush);

int tegra_mc_flush_done(int id)
{
	u32 rst_ctrl;
	u32 rst_ctrl_reg, rst_stat_reg;
	unsigned long flags;

	if (id < 32) {
		rst_ctrl_reg = MC_CLIENT_HOTRESET_CTRL;
		rst_stat_reg = MC_CLIENT_HOTRESET_STAT;
	} else {
		id %= 32;
		rst_ctrl_reg = MC_CLIENT_HOTRESET_CTRL_1;
		rst_stat_reg = MC_CLIENT_HOTRESET_STAT_1;
	}

	spin_lock_irqsave(&tegra_mc_lock, flags);

	rst_ctrl = tegra124_mc_readl(rst_ctrl_reg);
	rst_ctrl &= ~(1 << id);
	tegra124_mc_writel(rst_ctrl, rst_ctrl_reg);

	spin_unlock_irqrestore(&tegra_mc_lock, flags);

	return 0;
}
EXPORT_SYMBOL(tegra_mc_flush_done);

static int tegra124_mc_probe(struct platform_device *pdev)
{
	tegra_mc_base = of_iomap(pdev->dev.of_node, 0);

	if (tegra_mc_base)
		tegra_mc_init_done = true;

	return 0;
}

static const struct of_device_id tegra124_mc_of_match[] = {
	{ .compatible = "nvidia,tegra124-mc", },
	{},
};

static struct platform_driver tegra124_mc_driver = {
	.probe = tegra124_mc_probe,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra124_mc_of_match,
	},
};

static int __init tegra124_mc_init(void)
{
	return platform_driver_register(&tegra124_mc_driver);
}
arch_initcall(tegra124_mc_init);

static void __exit tegra124_mc_exit(void)
{
	platform_driver_unregister(&tegra124_mc_driver);
}
module_exit(tegra124_mc_exit);

MODULE_DESCRIPTION("Tegra124 MC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
