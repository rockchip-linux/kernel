/*
 * Copyright (C) 2015 Rockchip Electronics Co., Ltd.
 *      Huang Tao <huang.tao@rock-chips.com>
 *
 * Copyright (C) 2015 Rockchip Electronics Co., Ltd.
 *      Jeffy Chen <jeffy.chen@chromium.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/cpuidle.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <asm/cpuidle.h>

static struct cpuidle_driver rk3288_cpuidle_driver = {
	.name = "rk3288_cpuidle",
	.owner = THIS_MODULE,
	.states[0] = ARM_CPUIDLE_WFI_STATE,
	.state_count = 1,
};

static int rockchip_cpuidle_probe(struct platform_device *pdev)
{
	int ret;

	ret = cpuidle_register(&rk3288_cpuidle_driver, NULL);

	if (ret)
		pr_err("failed to register cpuidle driver: %d\n", ret);

	return ret;
}

static int rockchip_cpuidle_remove(struct platform_device *pdev)
{
	cpuidle_unregister(&rk3288_cpuidle_driver);

	return 0;
}

static const struct of_device_id rockchip_cpuidle_match[] = {
	{ .compatible = "rockchip,rk3288-cpuidle", },
	{ /* sentinel */},
};
MODULE_DEVICE_TABLE(of, rockchip_cpuidle_match);

static struct platform_driver rockchip_cpuidle_driver = {
	.probe  = rockchip_cpuidle_probe,
	.remove = rockchip_cpuidle_remove,
	.driver = {
		.name = "rk3288-cpuidle",
		.owner = THIS_MODULE,
		.of_match_table = rockchip_cpuidle_match,
	},
};

module_platform_driver(rockchip_cpuidle_driver);

MODULE_AUTHOR("Jeffy Chen <jeffy.chen@rock-chips.com>");
MODULE_DESCRIPTION("rockchip cpu idle driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:rockchip-cpuidle");
