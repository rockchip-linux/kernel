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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/soc/tegra/mc.h>

#define DRV_NAME "tegra124-mc"

static void __iomem *tegra_mc_base;

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

static int tegra124_mc_probe(struct platform_device *pdev)
{
	tegra_mc_base = of_iomap(pdev->dev.of_node, 0);

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
