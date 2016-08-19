/*
**************************************************************************
 * Rockchip driver for CIF ISP 1.0
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
#include <linux/platform_data/rk_cif10_platform.h>
#include "cif_cif10_regs.h"
#include "cif_cif10.h"


/* read/write registers */
#define write_cif_reg(addr, val)	\
	__raw_writel(val, addr + rk1108.cif_base)
#define read_cif_reg(addr)			\
	__raw_readl(addr + rk1108.cif_base)
#define write_grf_reg(addr, val)	\
	regmap_write(rk1108.regmap_grf, addr, val)
#define read_grf_reg(addr)	        \
	regmap_read(rk1108.regmap_grf, addr)

struct cif_cif10_clk_rst_rk1108 {
	void __iomem *cif_base;

	struct clk	*aclk_cif;
	struct clk	*hclk_cif;
	struct clk	*pclk_cif;

	struct reset_control *cif_arst;
	struct reset_control *cif_hrst;
	struct reset_control *cif_prst;
};


struct cif_cif10_rk1108 {
	struct regmap *regmap_grf;
	void __iomem *cif_base;
	struct cif_cif10_clk_rst_rk1108 clk_rst[4];
};

static struct cif_cif10_rk1108 rk1108 = {0};

static int soc_clk_enable(struct pltfrm_soc_init_para *init)
{
	struct cif_cif10_clk_rst_rk1108 *clk_rst =
				&rk1108.clk_rst[init->pdev->id];

	clk_prepare_enable(clk_rst->aclk_cif);
	clk_prepare_enable(clk_rst->hclk_cif);
	clk_prepare_enable(clk_rst->pclk_cif);

	return 0;
}

static int soc_clk_disable(struct pltfrm_soc_init_para *init)
{
	struct cif_cif10_clk_rst_rk1108 *clk_rst =
					&rk1108.clk_rst[init->pdev->id];

	clk_disable_unprepare(clk_rst->aclk_cif);
	clk_disable_unprepare(clk_rst->hclk_cif);
	clk_disable_unprepare(clk_rst->pclk_cif);

	return 0;
}

static void soc_cif_reset(struct pltfrm_soc_init_para *init)
{
	int ctrl_reg, inten_reg, crop_reg, scl_reg;
	int set_size_reg, for_reg, vir_line_width_reg;
	int y_reg, uv_reg, y_reg1, uv_reg1;
	struct cif_cif10_clk_rst_rk1108 *clk_rst;
	struct platform_device *pdev = init->pdev;

	if (!pdev || rk1108.clk_rst[pdev->id].cif_base == NULL)
		return;

	rk1108.cif_base = rk1108.clk_rst[pdev->id].cif_base;
	clk_rst = &rk1108.clk_rst[pdev->id];
	pr_info("reset cif%d\n", pdev->id);
	ctrl_reg = read_cif_reg(CIF_CIF_CTRL);
	if (ctrl_reg & ENABLE_CAPTURE)
		write_cif_reg(CIF_CIF_CTRL, ctrl_reg&~ENABLE_CAPTURE);

	crop_reg = read_cif_reg(CIF_CIF_CROP);
	set_size_reg = read_cif_reg(CIF_CIF_SET_SIZE);
	inten_reg = read_cif_reg(CIF_CIF_INTEN);
	for_reg = read_cif_reg(CIF_CIF_FOR);
	vir_line_width_reg = read_cif_reg(CIF_CIF_VIR_LINE_WIDTH);
	scl_reg = read_cif_reg(CIF_CIF_SCL_CTRL);
	y_reg = read_cif_reg(CIF_CIF_FRM0_ADDR_Y);
	uv_reg = read_cif_reg(CIF_CIF_FRM0_ADDR_UV);
	y_reg1 = read_cif_reg(CIF_CIF_FRM1_ADDR_Y);
	uv_reg1 = read_cif_reg(CIF_CIF_FRM1_ADDR_UV);

	/*cru soft reset*/
	if (IS_ERR_OR_NULL(clk_rst->cif_arst)) {
		pr_err("cif%d  areset source error\n", pdev->id);
		return;
	}
	if (IS_ERR_OR_NULL(clk_rst->cif_hrst)) {
		pr_err("cif%d  hreset source error\n", pdev->id);
		return;
	}
	if (IS_ERR_OR_NULL(clk_rst->cif_prst)) {
		pr_err("cif%d  preset source error\n", pdev->id);
		return;
	}
	reset_control_assert(clk_rst->cif_arst);
	reset_control_assert(clk_rst->cif_hrst);
	reset_control_assert(clk_rst->cif_prst);
	udelay(5);
	reset_control_deassert(clk_rst->cif_arst);
	reset_control_deassert(clk_rst->cif_hrst);
	reset_control_deassert(clk_rst->cif_prst);
	udelay(5);

	write_cif_reg(CIF_CIF_CTRL, ctrl_reg & ~ENABLE_CAPTURE);
	write_cif_reg(CIF_CIF_FOR, for_reg);
	write_cif_reg(CIF_CIF_FRM0_ADDR_Y, y_reg);
	write_cif_reg(CIF_CIF_FRM0_ADDR_UV, uv_reg);
	write_cif_reg(CIF_CIF_FRM1_ADDR_Y, y_reg1);
	write_cif_reg(CIF_CIF_FRM1_ADDR_UV, uv_reg1);
	write_cif_reg(CIF_CIF_INTSTAT, 0xFFFFFFFF);
	write_cif_reg(CIF_CIF_CROP, crop_reg);
	write_cif_reg(CIF_CIF_SET_SIZE, set_size_reg);
	write_cif_reg(CIF_CIF_VIR_LINE_WIDTH, vir_line_width_reg);
	write_cif_reg(CIF_CIF_FRAME_STATUS, 0x00000000);
	write_cif_reg(CIF_CIF_SCL_CTRL, scl_reg);
	write_cif_reg(CIF_CIF_INTEN, inten_reg);
}

static int soc_init(struct pltfrm_soc_init_para *init)
{
	struct cif_cif10_clk_rst_rk1108 *clk_rst;
	struct platform_device *pdev = init->pdev;
	struct device_node *np = pdev->dev.of_node;

	rk1108.regmap_grf =
		syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(rk1108.regmap_grf)) {
		dev_err(&pdev->dev, "Can't regmap rk1108 grf\n");
		return PTR_ERR(rk1108.regmap_grf);
	}

	pr_info("init cif%d clk&rst\n", pdev->id);

	clk_rst = &rk1108.clk_rst[pdev->id];

	clk_rst->aclk_cif =
		devm_clk_get(&pdev->dev, "aclk_cif");
	if (IS_ERR_OR_NULL(clk_rst->aclk_cif)) {
		dev_err(&pdev->dev,
			"Get rk1108 cif%d aclk resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}
	clk_rst->hclk_cif =
		devm_clk_get(&pdev->dev, "hclk_cif");
	if (IS_ERR_OR_NULL(clk_rst->hclk_cif)) {
		dev_err(&pdev->dev,
			"Get rk1108 cif%d hclk resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}
	clk_rst->pclk_cif =
		devm_clk_get(&pdev->dev, "pclkin_cif");
	if (IS_ERR_OR_NULL(clk_rst->pclk_cif)) {
		dev_err(&pdev->dev,
			"Get rk1108 cif%d pclk resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}

	clk_rst->cif_arst =
		devm_reset_control_get(&pdev->dev, "cif_arst");
	if (IS_ERR_OR_NULL(clk_rst->cif_arst)) {
		dev_err(&pdev->dev,
			"Get rk1108 cif%d arst resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}
	clk_rst->cif_hrst =
		devm_reset_control_get(&pdev->dev, "cif_hrst");
	if (IS_ERR_OR_NULL(clk_rst->cif_hrst)) {
		dev_err(&pdev->dev,
			"Get rk1108 cif%d hrst resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}
	clk_rst->cif_prst =
		devm_reset_control_get(&pdev->dev, "cif_prst");
	if (IS_ERR_OR_NULL(clk_rst->cif_prst)) {
		dev_err(&pdev->dev,
			"Get rk1108 cif%d prst resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}

	clk_rst->cif_base = init->cif_base;
	rk1108.cif_base = init->cif_base;

	return 0;
}

int pltfrm_rk1108_cif_cfg(
		struct pltfrm_soc_cfg_para *cfg)
{
	switch (cfg->cmd) {
	case PLTFRM_CLKEN:
		soc_clk_enable((struct pltfrm_soc_init_para *)cfg->cfg_para);
		break;

	case PLTFRM_CLKDIS:
		soc_clk_disable((struct pltfrm_soc_init_para *)cfg->cfg_para);
		break;

	case PLTFRM_CLKRST:
		soc_cif_reset((struct pltfrm_soc_init_para *)cfg->cfg_para);
		break;

	case PLTFRM_SOC_INIT:
		soc_init((struct pltfrm_soc_init_para *)cfg->cfg_para);
		break;

	default:
		break;
	}

	return 0;
}
#endif /* CONFIG_ARM */

