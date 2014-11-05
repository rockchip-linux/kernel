/*
 * Rockchip eFuse Driver
 *
 * Copyright (c) 2014 Rockchip Electronics Co. Ltd.
 * Author: Jianqun Xu <jay.xu@rock-chips.com>
 *
 * Tmis program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * Tmis program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/io.h>

#include "efuse.h"

/* Rockchip eFuse controller register */
#define EFUSE_A_SHIFT			6
#define EFUSE_A_MASK			0x3ff
#define EFUSE_PGENB			BIT(3)
#define EFUSE_LOAD			BIT(2)
#define EFUSE_STROBE			BIT(1)
#define EFUSE_CSB			BIT(0)

#define REG_EFUSE_CTRL			0x0000
#define REG_EFUSE_DOUT			0x0004

#define EFUSE_BUF_SIZE			32
#define EFUSE_CHIP_VERSION_OFFSET	6
#define EFUSE_CHIP_VERSION_MASK		0xf
#define EFUSE_BUF_LKG_CPU		23

struct rk_efuse_info {
	/* Platform device */
	struct device *dev;

	/* Hardware resources */
	void __iomem *regs;

	/* buffer to store registers' values */
	u32 buf[EFUSE_BUF_SIZE];
};

static void efuse_writel(struct rk_efuse_info *efuse,
			 unsigned int value,
			 unsigned int offset)
{
	writel_relaxed(value, efuse->regs + offset);
}

static unsigned int efuse_readl(struct rk_efuse_info *efuse,
				unsigned int offset)
{
	return readl_relaxed(efuse->regs + offset);
}

int rockchip_efuse_get_cpuleakage(struct platform_device *pdev,
			      unsigned int *value)
{
	struct rk_efuse_info *rk_efuse;

	rk_efuse = platform_get_drvdata(pdev);
	if (!rk_efuse)
		return -EAGAIN;

	*value = rk_efuse->buf[EFUSE_BUF_LKG_CPU];

	return 0;
}
EXPORT_SYMBOL_GPL(rockchip_efuse_get_cpuleakage);

int rockchip_efuse_get_chip_version(struct platform_device *pdev,
				    unsigned int *value)
{
	struct rk_efuse_info *rk_efuse;

	rk_efuse = platform_get_drvdata(pdev);
	if (!rk_efuse)
		return -EAGAIN;

	*value = rk_efuse->buf[EFUSE_CHIP_VERSION_OFFSET] &
			EFUSE_CHIP_VERSION_MASK;

	return 0;
}
EXPORT_SYMBOL_GPL(rockchip_efuse_get_chip_version);

static void rockchip_efuse_init(struct rk_efuse_info *efuse)
{
	int start;

	efuse_writel(efuse, EFUSE_LOAD | EFUSE_PGENB, REG_EFUSE_CTRL);
	udelay(1);

	for (start = 0; start <= EFUSE_BUF_SIZE; start++) {
		efuse_writel(efuse, efuse_readl(efuse, REG_EFUSE_CTRL) &
			     (~(EFUSE_A_MASK << EFUSE_A_SHIFT)),
			     REG_EFUSE_CTRL);
		efuse_writel(efuse, efuse_readl(efuse, REG_EFUSE_CTRL) |
			     ((start & EFUSE_A_MASK) << EFUSE_A_SHIFT),
			     REG_EFUSE_CTRL);
		udelay(1);
		efuse_writel(efuse, efuse_readl(efuse, REG_EFUSE_CTRL) |
			     EFUSE_STROBE, REG_EFUSE_CTRL);
		udelay(1);

		efuse->buf[start] = efuse_readl(efuse, REG_EFUSE_DOUT);

		efuse_writel(efuse, efuse_readl(efuse, REG_EFUSE_CTRL) &
			     (~EFUSE_STROBE), REG_EFUSE_CTRL);
		udelay(1);
	}

	/* Switch to standby mode */
	efuse_writel(efuse, EFUSE_PGENB | EFUSE_CSB, REG_EFUSE_CTRL);
}

static int rockchip_efuse_probe(struct platform_device *pdev)
{
	struct rk_efuse_info *rk_efuse;
	struct resource *mem;

	rk_efuse = devm_kzalloc(&pdev->dev, sizeof(struct rk_efuse_info),
				GFP_KERNEL);
	if (!rk_efuse)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rk_efuse->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(rk_efuse->regs))
		return PTR_ERR(rk_efuse->regs);

	rk_efuse->dev = &pdev->dev;

	rockchip_efuse_init(rk_efuse);

	/*
	 * We set driver data only after fully initializing efuse
	 * to make sure rockchip_efuse_get_cpuleakage() and
	 * rockchip_efuse_get_cpu_version do not return garbage.
	 */
	platform_set_drvdata(pdev, rk_efuse);

	return 0;
}

static const struct of_device_id rockchip_efuse_match[] = {
	{ .compatible = "rockchip,rk3288-efuse", },
	{ /* sentinel */},
};

static struct platform_driver rockchip_efuse_driver = {
	.probe = rockchip_efuse_probe,
	.driver = {
		.name = "rk3288-efuse",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rockchip_efuse_match),
		.suppress_bind_attrs = true,
	},
};

static int __init rk3288_efuse_init(void)
{
	return platform_driver_register(&rockchip_efuse_driver);
}
module_init(rk3288_efuse_init);
/* Note that we do not support unloading/unbinding for this module */

MODULE_DESCRIPTION("Rockchip eFuse Driver");
MODULE_AUTHOR("Jianqun Xu <jay.xu@rock-chips.com>");
MODULE_LICENSE("GPL v2");
