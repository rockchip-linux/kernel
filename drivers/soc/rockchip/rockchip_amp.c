// SPDX-License-Identifier: GPL-2.0-only
/*
 * Rockchip AMP support.
 *
 * Copyright (c) 2021 Rockchip Electronics Co. Ltd.
 * Author: Tony Xie <tony.xie@rock-chips.com>
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

static const struct of_device_id rockchip_amp_match[] = {
	{
		.compatible = "rockchip,rk3568-amp",
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, rockchip_amp_match);

static int rockchip_amp_probe(struct platform_device *pdev)
{
	struct clk_bulk_data *clks;
	int num_clks;
	int ret;

	num_clks = devm_clk_bulk_get_all(&pdev->dev, &clks);
	if (num_clks < 1)
		return -ENODEV;
	ret = clk_bulk_prepare_enable(num_clks, clks);
	if (ret) {
		dev_err(&pdev->dev, "failed to prepare enable clks: %d\n", ret);
		return ret;
	}
	return 0;
}

static struct platform_driver rockchip_amp_driver = {
	.probe = rockchip_amp_probe,
	.driver = {
		.name  = "rockchip-amp",
		.of_match_table = rockchip_amp_match,
	},
};
module_platform_driver(rockchip_amp_driver);

MODULE_DESCRIPTION("Rockchip AMP driver");
MODULE_AUTHOR("Tony xie<tony.xie@rock-chips.com>");
MODULE_LICENSE("GPL v2");
