/*
**************************************************************************
 * Rockchip driver for CIF CIF 1.0
 * (Based on Intel driver for sofiaxxx)
 *
 * Copyright (C) 2015 Intel Mobile Communications GmbH
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
**************************************************************************
 */

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/videobuf-core.h>
#include <linux/slab.h>
#include <linux/platform_data/rk_cif10_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/clk.h>
#include <linux/reset.h>

#define RST_DISABLE     (0x0)
#define RST_ENABLE      (0x1)

#define GRF_SOC_CON4 (0x0410)
#define GRF_SOC_CON9 (0x0424)
#define WRITE_MASK  (0xFFFF << 16)
#define ENABLE_MASK (0x8000 << 16)
#define CIF1TO4_EN  (0x1 << 15)
#define CIF1TO4_DIS (0x0 << 15)
#define CH_NUM_1    (0x0 << 12)
#define CH_NUM_2    (0x1 << 12)
#define CH_NUM_4    (0x2 << 12)

#define CHATCIF0    (0x0 << 10)
#define CHBTCIF0    (0x1 << 10)
#define CHCTCIF0    (0x2 << 10)
#define CHDTCIF0    (0x3 << 10)

#define CHATCIF1    (0x0 << 8)
#define CHBTCIF1    (0x1 << 8)
#define CHCTCIF1    (0x2 << 8)
#define CHDTCIF1    (0x3 << 8)

#define CHATCIF2    (0x0 << 6)
#define CHBTCIF2    (0x1 << 6)
#define CHCTCIF2    (0x2 << 6)
#define CHDTCIF2    (0x3 << 6)

#define CHATCIF3    (0x0 << 4)
#define CHBTCIF3    (0x1 << 4)
#define CHCTCIF3    (0x2 << 4)
#define CHDTCIF3    (0x3 << 4)

struct cif1to4_module {
	struct regmap *regmap_grf;
	struct clk *pclk;
	struct clk *pclkin_vip;
	struct clk *pclkin_vip_inv;

	struct reset_control *prst;
};
static struct cif1to4_module *cif1to4;

#define write_grf_reg(addr, val)	\
	regmap_write(cif1to4->regmap_grf, addr, val)
#define read_grf_reg(addr, val)	        \
	regmap_read(cif1to4->regmap_grf, addr, val)

static const struct of_device_id cif1to4_of_match[] = {
	{.compatible = "rockchip,cif1to4"},
	{},
};

static int cif1to4_drv_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;


	cif1to4 = (struct cif1to4_module *)devm_kzalloc(
				&pdev->dev,
				sizeof(struct cif1to4_module),
				GFP_KERNEL);
	if (!cif1to4)
		return -ENOMEM;

	cif1to4->regmap_grf =
		syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(cif1to4->regmap_grf)) {
		dev_err(&pdev->dev,
			"Can't regmap cif1to4 rv1108 grf\n");
		return PTR_ERR(cif1to4->regmap_grf);
	}

	/* get clks resouce */
	cif1to4->pclk =
		devm_clk_get(&pdev->dev, "cif1to4_pclk");
	if (IS_ERR_OR_NULL(cif1to4->pclk)) {
		dev_err(&pdev->dev,
			"Get cif1to4_pclk clock resouce failed !\n");
		return -EINVAL;
	}
	cif1to4->pclkin_vip =
		devm_clk_get(&pdev->dev, "pclkin_vip");
	if (IS_ERR_OR_NULL(cif1to4->pclkin_vip)) {
		dev_err(&pdev->dev,
			"Get pclkin_vip clock resouce failed !\n");
		return -EINVAL;
	}
	cif1to4->pclkin_vip_inv =
		devm_clk_get(&pdev->dev, "pclkin_vip_inv");
	if (IS_ERR_OR_NULL(cif1to4->pclkin_vip_inv)) {
		dev_err(&pdev->dev,
			"Get pclkin_vip_inv clock resouce failed !\n");
		return -EINVAL;
	}

	/* get rsts resouce */
	cif1to4->prst =
		devm_reset_control_get(&pdev->dev, "cif1to4_rst");
	if (IS_ERR_OR_NULL(cif1to4->prst)) {
		dev_err(&pdev->dev,
			"Get cif1to4_rst resouce failed !\n");
		return -EINVAL;
	}

	/* config clk */
	clk_set_parent(cif1to4->pclkin_vip,
		       cif1to4->pclkin_vip_inv);
	clk_prepare_enable(cif1to4->pclk);
	mdelay(1);
	write_grf_reg(GRF_SOC_CON9,
		      WRITE_MASK | CIF1TO4_DIS | CH_NUM_4 | CHATCIF0 |
		      CHBTCIF1 | CHCTCIF2 | CHDTCIF3);
	pr_info("cif1to4 probe success\n");

	return 0;
}

static struct platform_driver cif1to4_plat_drv = {
	.driver = {
		.name = "cif1to4",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cif1to4_of_match),
	},
	.probe = cif1to4_drv_probe,
};

module_platform_driver(cif1to4_plat_drv);

MODULE_DESCRIPTION("For rk cif1to4 driver");
MODULE_AUTHOR("George");
MODULE_LICENSE("GPL");
