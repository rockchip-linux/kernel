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

#define RV1108_GRF_SOC_CON1	0x0404
#define APIO5_33V		(0 << 14 | 1 << 30)
#define APIO5_18V		(1 << 14 | 1 << 30)

#define RV1108_GRF_SOC_CON4	(0x0410)
#define CIF0_DATASEL_CVBS	(0x3 << 16 | 0x0)
#define CIF0_DATASEL_CAM	(0x3 << 16 | 0x2)
#define CIF0_DATASEL_CIF1T4	(0x3 << 16 | 0x3)
#define RV1108_GRF_SOC_CON9	(0x0424)
#define ENABLE_MASK		(0x8000 << 16)
#define CIF1TO4_EN		(0x1 << 15)
#define CIF1TO4_DIS		(0x0 << 15)
#define NUM_MASK		(0x3000 << 16)
#define CH_NUM_1		(0x0 << 12)
#define CH_NUM_2		(0x1 << 12)
#define CH_NUM_4		(0x2 << 12)

/* read/write registers */
#define write_cif_reg(addr, val)	\
	__raw_writel(val, addr + rv1108.cif_base)
#define read_cif_reg(addr)			\
	__raw_readl(addr + rv1108.cif_base)
#define write_grf_reg(addr, val)	\
	regmap_write(rv1108.regmap_grf, addr, val)
#define read_grf_reg(addr, val)	        \
	regmap_read(rv1108.regmap_grf, addr, val)

struct cif_cif10_clk_rst_rv1108 {
	void __iomem *cif_base;

	struct clk	*aclk_cif;
	struct clk	*hclk_cif;
	struct clk	*pclk_cif;

	struct clk      *vip_clk;
	struct clk      *pclkin_cvbs2cif;
	struct clk      *pclkin_vip;
	struct clk      *pclk_cif1t4_out;
	struct clk      *pclkin_ninv;
	struct clk      *pclkin_inv;

	struct reset_control *cif_arst;
	struct reset_control *cif_hrst;
	struct reset_control *cif_prst;
};


struct cif_cif10_rv1108 {
	struct regmap *regmap_grf;
	void __iomem *cif_base;
	struct cif_cif10_clk_rst_rv1108 clk_rst[4];
};

static struct cif_cif10_rv1108 rv1108 = {0};

static void soc_clk_rst(struct cif_cif10_clk_rst_rv1108 *clk_rst)
{
	/*cru soft reset*/
	if (IS_ERR_OR_NULL(clk_rst)) {
		pr_err("clk rst error or null\n");
		return;
	}
	if (IS_ERR_OR_NULL(clk_rst->cif_arst)) {
		pr_err("areset source error\n");
		return;
	}
	if (IS_ERR_OR_NULL(clk_rst->cif_hrst)) {
		pr_err("hreset source error\n");
		return;
	}
	if (IS_ERR_OR_NULL(clk_rst->cif_prst)) {
		pr_err("preset source error\n");
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
}

static void soc_cif0_data_sel(struct pltfrm_soc_init_para *init)
{
	if (PLTFRM_CAM_ITF_IS_BT601_FIELD(init->cam_itf.type))
		write_grf_reg(RV1108_GRF_SOC_CON4,
			      CIF0_DATASEL_CVBS);
	else if (PLTFRM_CAM_ITF_IS_BT656_MIX(init->cam_itf.type))
		write_grf_reg(RV1108_GRF_SOC_CON4,
			      CIF0_DATASEL_CIF1T4);
	else
		write_grf_reg(RV1108_GRF_SOC_CON4,
			      CIF0_DATASEL_CAM);
}

static void soc_clk_init(struct pltfrm_soc_init_para *init)
{
	struct cif_cif10_clk_rst_rv1108 *clk_rst =
				&rv1108.clk_rst[init->pdev->id];

	dev_info(&init->pdev->dev,
		 "soc_clk_init id %d type %#x\n",
		 init->pdev->id,
		 init->cam_itf.type);

	soc_clk_rst(&rv1108.clk_rst[init->pdev->id]);

	if (init->pdev->id == 0)
		soc_cif0_data_sel(init);

	if (PLTFRM_CAM_ITF_IS_DVP(init->cam_itf.type)) {
		/* bit14 : APIO5  0:3.3V  1:1.8V */
		if (init->cam_itf.cfg.dvp.io_vol ==
		    PLTFRM_CAM_IO_1800)
			write_grf_reg(RV1108_GRF_SOC_CON1, APIO5_18V);
		else if (init->cam_itf.cfg.dvp.io_vol ==
			 PLTFRM_CAM_IO_3300)
			write_grf_reg(RV1108_GRF_SOC_CON1, APIO5_33V);
	}

	if (PLTFRM_CAM_ITF_IS_BT656_MIX(init->cam_itf.type)) {
		write_grf_reg(RV1108_GRF_SOC_CON9,
			      ENABLE_MASK | CIF1TO4_EN);
		if (PLTFRM_CAM_ITF_DVP_CHS(init->cam_itf.type) == 1)
			write_grf_reg(RV1108_GRF_SOC_CON9,
				      NUM_MASK | CH_NUM_1);
		else if (PLTFRM_CAM_ITF_DVP_CHS(init->cam_itf.type) == 2)
			write_grf_reg(RV1108_GRF_SOC_CON9,
				      NUM_MASK | CH_NUM_2);
		else if (PLTFRM_CAM_ITF_DVP_CHS(init->cam_itf.type) == 4)
			write_grf_reg(RV1108_GRF_SOC_CON9,
				      NUM_MASK | CH_NUM_4);
		else
			dev_err(&init->pdev->dev,
				"sensors channels %d error\n",
				PLTFRM_CAM_ITF_DVP_CHS(init->cam_itf.type));
	}

	switch (init->pdev->id) {
	case 0:
		if (PLTFRM_CAM_ITF_IS_BT601_FIELD(init->cam_itf.type)) {
			clk_set_parent(clk_rst->vip_clk,
				       clk_rst->pclkin_cvbs2cif);
		} else {
			if (PLTFRM_CAM_ITF_DVP_CHS(init->cam_itf.type) == 2 ||
			    PLTFRM_CAM_ITF_DVP_CHS(init->cam_itf.type) == 4) {
				clk_set_parent(clk_rst->vip_clk,
					       clk_rst->pclk_cif1t4_out);
			} else {
				if (init->cam_itf.cfg.dvp.pclk ==
				    PLTFRM_CAM_SDR_POS_EDG)
					clk_set_parent(clk_rst->pclkin_vip,
						       clk_rst->pclkin_ninv);
				else
					clk_set_parent(clk_rst->pclkin_vip,
						       clk_rst->pclkin_inv);
				clk_set_parent(clk_rst->vip_clk,
					       clk_rst->pclkin_vip);
			}
		}
		break;
	case 1:
		if (PLTFRM_CAM_ITF_IS_BT601_FIELD(init->cam_itf.type))
			clk_set_parent(clk_rst->vip_clk,
				       clk_rst->pclkin_cvbs2cif);
		else
			clk_set_parent(clk_rst->vip_clk,
				       clk_rst->pclk_cif1t4_out);
		break;
	case 2:
		if (PLTFRM_CAM_ITF_IS_BT601_FIELD(init->cam_itf.type))
			clk_set_parent(clk_rst->vip_clk,
				       clk_rst->pclkin_cvbs2cif);
		else
			clk_set_parent(clk_rst->vip_clk,
				       clk_rst->pclk_cif1t4_out);
		break;
	case 3:
		if (PLTFRM_CAM_ITF_IS_BT601_FIELD(init->cam_itf.type))
			clk_set_parent(clk_rst->vip_clk,
				       clk_rst->pclkin_cvbs2cif);
		else
			clk_set_parent(clk_rst->vip_clk,
				       clk_rst->pclk_cif1t4_out);
		break;
	default:
		dev_err(&init->pdev->dev,
			"invalid cif id %d\n",
			init->pdev->id);
		break;
	}
}

static int soc_clk_enable(struct pltfrm_soc_init_para *init)
{
	struct cif_cif10_clk_rst_rv1108 *clk_rst =
				&rv1108.clk_rst[init->pdev->id];

	clk_prepare_enable(clk_rst->aclk_cif);
	clk_prepare_enable(clk_rst->hclk_cif);
	clk_prepare_enable(clk_rst->pclk_cif);

	return 0;
}

static int soc_clk_disable(struct pltfrm_soc_init_para *init)
{
	struct cif_cif10_clk_rst_rv1108 *clk_rst =
					&rv1108.clk_rst[init->pdev->id];

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
	struct platform_device *pdev = init->pdev;

	if (!pdev || rv1108.clk_rst[pdev->id].cif_base == NULL)
		return;

	rv1108.cif_base = rv1108.clk_rst[pdev->id].cif_base;
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

	soc_clk_rst(&rv1108.clk_rst[init->pdev->id]);

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
	struct cif_cif10_clk_rst_rv1108 *clk_rst;
	struct platform_device *pdev = init->pdev;
	struct device_node *np = pdev->dev.of_node;

	rv1108.regmap_grf =
		syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(rv1108.regmap_grf)) {
		dev_err(&pdev->dev, "Can't regmap rv1108 grf\n");
		return PTR_ERR(rv1108.regmap_grf);
	}

	dev_info(&pdev->dev,
		 "init cif%d clk&rst\n",
		 pdev->id);

	clk_rst = &rv1108.clk_rst[pdev->id];

	clk_rst->aclk_cif =
		devm_clk_get(&pdev->dev, "aclk_cif");
	if (IS_ERR_OR_NULL(clk_rst->aclk_cif)) {
		dev_err(&pdev->dev,
			"Get rv1108 cif%d aclk resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}
	clk_rst->hclk_cif =
		devm_clk_get(&pdev->dev, "hclk_cif");
	if (IS_ERR_OR_NULL(clk_rst->hclk_cif)) {
		dev_err(&pdev->dev,
			"Get rv1108 cif%d hclk resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}
	clk_rst->pclk_cif =
		devm_clk_get(&pdev->dev, "pclkin_cif");
	if (IS_ERR_OR_NULL(clk_rst->pclk_cif)) {
		dev_err(&pdev->dev,
			"Get rv1108 cif%d pclk resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}
	clk_rst->vip_clk =
		devm_clk_get(&pdev->dev, "vip_clk");
	if (IS_ERR_OR_NULL(clk_rst->vip_clk)) {
		dev_err(&pdev->dev,
			"Get rv1108 cif%d vip_clk resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}
	clk_rst->pclkin_cvbs2cif =
		devm_clk_get(&pdev->dev, "pclkin_cvbs2cif");
	if (IS_ERR_OR_NULL(clk_rst->pclkin_cvbs2cif)) {
		dev_err(&pdev->dev,
			"Get rv1108 cif%d pclkin_cvbs2cif resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}
	clk_rst->pclkin_vip =
		devm_clk_get(&pdev->dev, "pclkin_vip");
	if (IS_ERR_OR_NULL(clk_rst->pclkin_vip)) {
		dev_err(&pdev->dev,
			"Get rv1108 cif%d pclkin_vip resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}
	clk_rst->pclk_cif1t4_out =
		devm_clk_get(&pdev->dev, "pclk_cif1t4_out");
	if (IS_ERR_OR_NULL(clk_rst->pclk_cif1t4_out)) {
		dev_err(&pdev->dev,
			"Get rv1108 cif%d pclk_cif1t4_out resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}
	clk_rst->pclkin_ninv =
		devm_clk_get(&pdev->dev, "pclkin_vip_ninv");
	if (IS_ERR_OR_NULL(clk_rst->pclkin_ninv)) {
		dev_err(&pdev->dev,
			"Get rv1108 cif%d pclkin_ninv resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}
	clk_rst->pclkin_inv =
		devm_clk_get(&pdev->dev, "pclkin_vip_inv");
	if (IS_ERR_OR_NULL(clk_rst->pclkin_inv)) {
		dev_err(&pdev->dev,
			"Get rv1108 cif%d pclkin_inv resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}

	clk_rst->cif_arst =
		devm_reset_control_get(&pdev->dev, "cif_arst");
	if (IS_ERR_OR_NULL(clk_rst->cif_arst)) {
		dev_err(&pdev->dev,
			"Get rv1108 cif%d arst resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}
	clk_rst->cif_hrst =
		devm_reset_control_get(&pdev->dev, "cif_hrst");
	if (IS_ERR_OR_NULL(clk_rst->cif_hrst)) {
		dev_err(&pdev->dev,
			"Get rv1108 cif%d hrst resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}
	clk_rst->cif_prst =
		devm_reset_control_get(&pdev->dev, "cif_prst");
	if (IS_ERR_OR_NULL(clk_rst->cif_prst)) {
		dev_err(&pdev->dev,
			"Get rv1108 cif%d prst resouce failed !\n",
			pdev->id);
		return -EINVAL;
	}

	clk_rst->cif_base = init->cif_base;
	rv1108.cif_base = init->cif_base;

	return 0;
}

int pltfrm_rv1108_cif_cfg(
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

	case PLTFRM_CLKINIT:
		soc_clk_init((struct pltfrm_soc_init_para *)cfg->cfg_para);
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

