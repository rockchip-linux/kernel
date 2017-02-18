/*
**************************************************************************
 * Rockchip driver for CIF CVBSIN 1.0
 *
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
**************************************************************************
 */
#ifdef CONFIG_ARM
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include "cif_cvbsin.h"

#define RV1108_GRF_SOC_CON10 (0x0428)
#define RV1108_GRF_SOC_CON11 (0x042c)
#define VADC_PD_PRE          (0x1 << 6)
#define VADC_PD_CLMP         (0x1 << 5)
#define VADC_PD_BG           (0x1 << 4)
#define VADC_PD_ADC          (0x1 << 3)
#define VADC_LPF_BW_BYPASS   (0x3 << 12)
#define VADC_ICLMP_CTL_400MA (0x8 << 8)
#define VADC_GAIN            (0xa << 4)

#define write_grf_reg(addr, val)	\
	regmap_write(rv1108_cvbs.regmap_grf, addr, val)
#define read_grf_reg(addr, val)	        \
	regmap_read(rv1108_cvbs.regmap_grf, addr, val)

struct cif_cvbsin_rv1108 {
	struct regmap *regmap_grf;

	struct clk *cvbs_hclk;
	struct clk *cvbs_pclk;
	struct clk *cvbs_clk;
	struct clk *cvbs_clk_parent;

	struct reset_control *cvbs_prst;
	struct reset_control *cvbs_hrst;
	struct reset_control *cvbs_clk_rst;
};

static struct cif_cvbsin_rv1108 rv1108_cvbs = {0};

static void soc_cvbsin_reset(void)
{
	pr_info("soc cvbsin reset\n");
	reset_control_assert(rv1108_cvbs.cvbs_prst);
	reset_control_assert(rv1108_cvbs.cvbs_hrst);
	reset_control_assert(rv1108_cvbs.cvbs_clk_rst);
	udelay(5);
	reset_control_deassert(rv1108_cvbs.cvbs_prst);
	reset_control_deassert(rv1108_cvbs.cvbs_hrst);
	reset_control_deassert(rv1108_cvbs.cvbs_clk_rst);
	udelay(5);
}

static int soc_cvbsin_poweron(void)
{
	clk_prepare_enable(rv1108_cvbs.cvbs_pclk);
	clk_prepare_enable(rv1108_cvbs.cvbs_hclk);
	usleep_range(1000, 10 * 1000);
	write_grf_reg(RV1108_GRF_SOC_CON11,
		      0xFFFF0000 |
		      VADC_PD_CLMP);
	usleep_range(1000, 10 * 1000);
	soc_cvbsin_reset();
	return 0;
}

static int soc_cvbsin_poweroff(void)
{
	write_grf_reg(RV1108_GRF_SOC_CON11,
		      0xFFFF0000 |
		      VADC_PD_PRE |
		      VADC_PD_CLMP |
		      VADC_PD_BG |
		      VADC_PD_ADC);
	clk_disable_unprepare(rv1108_cvbs.cvbs_pclk);
	clk_disable_unprepare(rv1108_cvbs.cvbs_hclk);
	return 0;
}

static int soc_cvbsin_init(struct pltfrm_cvbsin_init_para *init)
{
	struct platform_device *pdev = init->pdev;
	struct device_node *np = pdev->dev.of_node;

	rv1108_cvbs.regmap_grf =
		syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(rv1108_cvbs.regmap_grf)) {
		dev_err(&pdev->dev,
			"Can't regmap cvbsin rv1108 grf\n");
		return PTR_ERR(rv1108_cvbs.regmap_grf);
	}

	/* get clks resouce */
	rv1108_cvbs.cvbs_hclk =
		devm_clk_get(&pdev->dev, "hclk_cvbs");
	if (IS_ERR_OR_NULL(rv1108_cvbs.cvbs_hclk)) {
		dev_err(&pdev->dev,
			"Get hclk_cvbs clock resouce failed !\n");
		return -EINVAL;
	}
	rv1108_cvbs.cvbs_pclk =
		devm_clk_get(&pdev->dev, "pclk_cvbs");
	if (IS_ERR_OR_NULL(rv1108_cvbs.cvbs_pclk)) {
		dev_err(&pdev->dev,
			"Get pclk_cvbs clock resouce failed !\n");
		return -EINVAL;
	}
	rv1108_cvbs.cvbs_clk =
		devm_clk_get(&pdev->dev, "clk_cvbs");
	if (IS_ERR_OR_NULL(rv1108_cvbs.cvbs_clk)) {
		dev_err(&pdev->dev,
			"Get clk_cvbs clock resouce failed !\n");
		return -EINVAL;
	}
	rv1108_cvbs.cvbs_clk_parent =
		devm_clk_get(&pdev->dev, "clk_cvbs_parent");
	if (IS_ERR_OR_NULL(rv1108_cvbs.cvbs_clk_parent)) {
		dev_err(&pdev->dev,
			"Get clk_cvbsin_parent clock resouce failed !\n");
		return -EINVAL;
	}

	/* get rsts resouce */
	rv1108_cvbs.cvbs_prst =
		devm_reset_control_get(&pdev->dev, "cvbs_prst");
	if (IS_ERR_OR_NULL(rv1108_cvbs.cvbs_prst)) {
		dev_err(&pdev->dev,
			"Get cvbsin prst resouce failed !\n");
		return -EINVAL;
	}
	rv1108_cvbs.cvbs_hrst =
		devm_reset_control_get(&pdev->dev, "cvbs_hrst");
	if (IS_ERR_OR_NULL(rv1108_cvbs.cvbs_hrst)) {
		dev_err(&pdev->dev,
			"Get cvbsin hrst resouce failed !\n");
		return -EINVAL;
	}
	rv1108_cvbs.cvbs_clk_rst =
		devm_reset_control_get(&pdev->dev, "cvbs_clk_rst");
	if (IS_ERR_OR_NULL(rv1108_cvbs.cvbs_clk_rst)) {
		dev_err(&pdev->dev,
			"Get cvbsin clk rst resouce failed !\n");
		return -EINVAL;
	}

	/* config clk */
	clk_set_parent(rv1108_cvbs.cvbs_clk,
		       rv1108_cvbs.cvbs_clk_parent);
	clk_set_rate(rv1108_cvbs.cvbs_pclk, 54000000);

	/* config adc */
	write_grf_reg(RV1108_GRF_SOC_CON10,
		      0xFFFF0000 |
		      VADC_ICLMP_CTL_400MA |
		      VADC_GAIN);
	write_grf_reg(RV1108_GRF_SOC_CON11,
		      0xFFFF0000 |
		      VADC_PD_PRE |
		      VADC_PD_CLMP |
		      VADC_PD_BG |
		      VADC_PD_ADC);
	dev_info(&pdev->dev,
		 "cvbsin init pltfrm success\n");
	return 0;
}

int pltfrm_rv1108_cvbsin_cfg(
	struct pltfrm_cvbsin_cfg_para *cfg)
{
	switch (cfg->cmd) {
	case PLTFRM_CVBSIN_POWERON:
		soc_cvbsin_poweron();
		break;

	case PLTFRM_CVBSIN_POWEROFF:
		soc_cvbsin_poweroff();
		break;

	case PLTFRM_CVBSIN_RST:
		soc_cvbsin_reset();
		break;

	case PLTFRM_CVBSIN_INIT:
		soc_cvbsin_init(
			(struct pltfrm_cvbsin_init_para *)cfg->cfg_para);
		break;

	default:
		break;
	}

	return 0;
}
#endif /* CONFIG_ARM */

