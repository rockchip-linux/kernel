/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>


static int uboot_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct clk *reserved_clk;
	int i = 0;

	/* reserve clock */
	while(1) {
		
		reserved_clk = of_clk_get(np, i++);
		if (IS_ERR(reserved_clk))
			break;

		if (clk_prepare_enable(reserved_clk)) {
			pr_err("Failed to enable clock for uboot\n");
			break;
		}
	}

	/* setup irq */

	return 0;
}

static int uboot_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id uboot_dt_ids[] = {
	{ .compatible = "rockchip,uboot" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rk_iommu_dt_ids);

static struct platform_driver uboot_driver = {
	.probe = uboot_probe,
	.remove = uboot_remove,
	.driver = {
		   .name = "uboot",
		   .of_match_table = uboot_dt_ids,
	},
};

static int __init uboot_init(void)
{
	return platform_driver_register(&uboot_driver);
}
static void __exit uboot_exit(void)
{
	platform_driver_unregister(&uboot_driver);
}

subsys_initcall(uboot_init);
module_exit(uboot_exit);

MODULE_DESCRIPTION("Uboot Helper for Rockchip");
MODULE_AUTHOR("Jacob Chen <jacob2.chen@rock-chips.com>");
MODULE_ALIAS("platform:rockchip-uboot");
MODULE_LICENSE("GPL v2");
