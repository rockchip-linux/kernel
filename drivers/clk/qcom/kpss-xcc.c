/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clk/msm-clk-generic.h>

static int kpss_xcc_set_mux_sel(struct mux_clk *clk, int sel)
{
	writel_relaxed(sel, clk->base + clk->offset);
	return 0;
}

static int kpss_xcc_get_mux_sel(struct mux_clk *clk)
{
	return readl_relaxed(clk->base + clk->offset);
}

static const struct clk_mux_ops kpss_xcc_ops = {
	.set_mux_sel = kpss_xcc_set_mux_sel,
	.get_mux_sel = kpss_xcc_get_mux_sel,
};

static const char *aux_parents[] = {
	"pll8_vote",
	"pxo",
};

static u8 aux_parent_map[] = {
	3,
	0,
};

static const struct of_device_id kpss_xcc_match_table[] = {
	{ .compatible = "qcom,kpss-acc-v1", .data = (void *)1UL },
	{ .compatible = "qcom,kpss-gcc" },
	{}
};
MODULE_DEVICE_TABLE(of, kpss_xcc_match_table);

static int kpss_xcc_driver_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct clk *clk;
	struct resource *res;
	void __iomem *base;
	struct mux_clk *mux_clk;
	struct clk_init_data init = {
		.parent_names = aux_parents,
		.num_parents = 2,
		.ops = &clk_ops_gen_mux,
	};

	id = of_match_device(kpss_xcc_match_table, &pdev->dev);
	if (!id)
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	mux_clk = devm_kzalloc(&pdev->dev, sizeof(*mux_clk), GFP_KERNEL);
	if (!mux_clk)
		return -ENOMEM;

	mux_clk->mask = 0x3;
	mux_clk->parent_map = aux_parent_map;
	mux_clk->ops = &kpss_xcc_ops;
	mux_clk->base = base;
	mux_clk->hw.init = &init;

	if (id->data) {
		if (of_property_read_string_index(pdev->dev.of_node,
					"clock-output-names", 0, &init.name))
			return -ENODEV;
		mux_clk->offset = 0x14;
	} else {
		init.name = "acpu_l2_aux";
		mux_clk->offset = 0x28;
	}

	clk = devm_clk_register(&pdev->dev, &mux_clk->hw);

	return PTR_ERR_OR_ZERO(clk);
}

static struct platform_driver kpss_xcc_driver = {
	.probe = kpss_xcc_driver_probe,
	.driver = {
		.name = "kpss-xcc",
		.of_match_table = kpss_xcc_match_table,
		.owner = THIS_MODULE,
	},
};
module_platform_driver(kpss_xcc_driver);

MODULE_LICENSE("GPL v2");
