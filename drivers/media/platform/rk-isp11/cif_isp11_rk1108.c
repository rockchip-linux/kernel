/*
**************************************************************************
 * Rockchip driver for CIF ISP 1.1
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
#include <linux/platform_data/rk_isp11_platform.h>
#include "cif_isp11_pltfrm.h"

/*MARVIN REGISTER*/
#define MRV_MIPI_BASE                           0x1C00
#define MRV_MIPI_CTRL                           0x00

#define GRF_SOC_CON5_OFFSET    (0x0414)

#define DPHY_RX_FORCERXMODE_MASK		(0xf<<16)
#define DPHY_RX_FORCERXMODE_BIT			(0x0)

/*
*GRF_IO_VSEL
*/
#define GRF_IO_VSEL_OFFSET		(0x0380)
#define DVP_V18SEL			((1<<1) | (1<<17))
#define DVP_V33SEL			((0<<1) | (1<<17))

/*
*GRF_IO_VSEL
*/
#define GRF_GPIO2B_E_OFFSET		(0x0380)
#define CIF_CLKOUT_STRENGTH(a)	(((a&0x03)<<3) | (0x03<<19))

/*
*CSI HOST
*/

#define CSIHOST_PHY_TEST_CTRL0            (0x30)
#define CSIHOST_PHY_TEST_CTRL1            (0x34)
#define CSIHOST_PHY_SHUTDOWNZ             (0x08)
#define CSIHOST_DPHY_RSTZ                 (0x0c)
#define CSIHOST_N_LANES                   (0x04)
#define CSIHOST_CSI2_RESETN               (0x10)
#define CSIHOST_PHY_STATE                 (0x14)
#define CSIHOST_DATA_IDS1                 (0x18)
#define CSIHOST_DATA_IDS2                 (0x1C)
#define CSIHOST_ERR1                      (0x20)
#define CSIHOST_ERR2                      (0x24)


#define write_cifisp_reg(addr, val)		__raw_writel(val, addr+rk1108->isp_base)
#define read_cifisp_reg(addr)		__raw_readl(addr+rk1108->isp_base)
#define write_cifcru_reg(addr, val)		__raw_writel(val, addr+cru_base_addr)
#define read_cifcru_reg(addr)		__raw_readl(addr+cru_base_addr)

#define write_grf_reg(addr, val)	regmap_write(rk1108->regmap_grf, addr, val)
#define read_grf_reg(addr, val)	regmap_read(rk1108->regmap_grf, addr, val)

#define write_csiphy_reg(addr, val)       __raw_writel(val, addr+rk1108->csiphy_base)
#define read_csiphy_reg(addr)             __raw_readl(addr+rk1108->csiphy_base)

struct cif_isp11_clk_rst_rk1108 {
	struct clk	*aclk_isp;
	struct clk	*hclk_isp;
	struct clk	*sclk_isp;
	struct clk	*sclk_isp_jpe;
	struct clk *sclk_mipidsi_24m;
	struct clk *pclk_mipi_csi;
	struct clk *pclk_isp_in;	
	struct reset_control *isp_rst;
};


struct cif_isp11_rk1108 {
	struct regmap *regmap_grf;
	void __iomem *csiphy_base;
	void __iomem *isp_base;
	struct cif_isp11_clk_rst_rk1108 clk_rst;
	struct cif_isp11_device *cif_isp11;
};

struct mipi_dphy_hsfreqrange {
	unsigned int range_l;
	unsigned int range_h;
	unsigned char cfg_bit;
};

static struct mipi_dphy_hsfreqrange mipi_dphy_hsfreq_range[] = {
#ifdef FPGA_TEST
	{80, 90, 0x00},
	{90, 100, 0x10},
	{100, 110, 0x20},
	{110, 130, 0x01},
	{130, 140, 0x11},
	{140, 150, 0x21},
	{150, 170, 0x02},
	{170, 180, 0x12},
	{180, 200, 0x22},
	{200, 220, 0x03},
	{220, 240, 0x13},
	{240, 250, 0x23},
	{250, 270, 0x4},
	{270, 300, 0x14},
	{300, 330, 0x5},
	{330, 360, 0x15},
	{360, 400, 0x25},
	{400, 450, 0x06},
	{450, 500, 0x16},
	{500, 550, 0x07},
	{550, 600, 0x17},
	{600, 650, 0x08},
	{650, 700, 0x18},
	{700, 750, 0x09},
	{750, 800, 0x19},
	{800, 850, 0x29},
	{850, 900, 0x39},
	{900, 950, 0x0a},
	{950, 1000, 0x1a}
#else
	{80, 110, 0x00},
	{110, 150, 0x01},
	{150, 200, 0x02},
	{200, 250, 0x03},
	{250, 300, 0x04},
	{300, 400, 0x05},
	{400, 500, 0x06},
	{500, 600, 0x07},
	{600, 700, 0x08},
	{700, 800, 0x09},
	{800, 1000, 0x10},
	{1000, 1200, 0x11},
	{1200, 1400, 0x12},
	{1400, 1600, 0x13},
	{1600, 1800, 0x14}
#endif
};

static struct cif_isp11_rk1108 *rk1108;
#ifdef FPGA_TEST

#define LANE_NUMBER 1
#define CRU_BASE_USER (0x800)

void MIPI_DPHY_WriteReg(unsigned char addr, unsigned char data)
{
	write_cifcru_reg(CRU_BASE_USER+0x20, 0x02ff0200|addr);
	write_cifcru_reg(CRU_BASE_USER+0x20, 0x01000000);
	write_cifcru_reg(CRU_BASE_USER+0x20, 0x02ff0000|data);
	write_cifcru_reg(CRU_BASE_USER+0x20, 0x01000100);
}

unsigned char MIPI_DPHY_ReadReg(unsigned char addr)
{
    unsigned char data;
	write_cifcru_reg(CRU_BASE_USER+0x20, 0x02ff0200|addr);
	write_cifcru_reg(CRU_BASE_USER+0x20, 0x01000000);
    data = (read_cifcru_reg(CRU_BASE_USER+0x18) & 0xff);
	write_cifcru_reg(CRU_BASE_USER+0x20, 0x03000100);

    return data;
}
#endif
static int mipi_dphy_cfg (struct pltfrm_cam_mipi_config *para)
{    
	unsigned char hsfreqrange=0xff, i;
	struct mipi_dphy_hsfreqrange *hsfreqrange_p;
	unsigned char datalane_en, input_sel;

	hsfreqrange_p = mipi_dphy_hsfreq_range;
	for (i = 0;
		i < (sizeof(mipi_dphy_hsfreq_range)/
		sizeof(struct mipi_dphy_hsfreqrange));
		i++) {
		if ((para->bit_rate > hsfreqrange_p->range_l) &&
			 (para->bit_rate <= hsfreqrange_p->range_h)) {
			hsfreqrange = hsfreqrange_p->cfg_bit;
			break;
		}
		hsfreqrange_p++;
	}

	if (hsfreqrange == 0xff) {
		hsfreqrange = 0x00;
	}
	/*hsfreqrange <<= 1;*/
	input_sel = para->dphy_index;
	datalane_en = 0;
	for (i=0; i<para->nb_lanes; i++)
		datalane_en |= (1<<i);

	if (input_sel == 0) {
#ifndef FPGA_TEST
	write_grf_reg(GRF_SOC_CON5_OFFSET,
	DPHY_RX_FORCERXMODE_MASK | DPHY_RX_FORCERXMODE_BIT); 

	/*set clock lane */
	/*write_csiphy_reg(0x34,0x00); */	
    write_csiphy_reg((0x100),hsfreqrange|
    				(read_csiphy_reg(0x100)&(~0xf)));
	if(para->nb_lanes > 0x00){/*lane0*/
		write_csiphy_reg((0x180),hsfreqrange|
						(read_csiphy_reg(0x180)&(~0xf)));
	}
	if(para->nb_lanes > 0x02){/*lane1*/
		write_csiphy_reg(0x200,hsfreqrange|
						(read_csiphy_reg(0x200)&(~0xf)));
	}
	if(para->nb_lanes > 0x04){/*lane4*/
		write_csiphy_reg(0x280,hsfreqrange|
						(read_csiphy_reg(0x280)&(~0xf)));
		write_csiphy_reg(0x300,hsfreqrange|
						(read_csiphy_reg(0x300)&(~0xf)));
	}

	/*set data lane num and enable clock lane */
	write_csiphy_reg(0x00, ((para->nb_lanes << 2)|(0x1<<6)|0x1));

    write_cifisp_reg((MRV_MIPI_BASE+MRV_MIPI_CTRL),
		read_cifisp_reg(MRV_MIPI_BASE+MRV_MIPI_CTRL) & (~(0x0f<<8)));
#else
	write_cifcru_reg(CRU_BASE_USER+0x20, 0x04000000);/*TESTCLR=0*/
	write_cifcru_reg(CRU_BASE_USER+0x20, 0x08000000);/*RSTZCAL=1*/	
	udelay(100);
	write_cifcru_reg(CRU_BASE_USER+0x20, 0x01000100);/*TESTCLK=1*/
	write_cifcru_reg(CRU_BASE_USER+0x20, 0x80008000);/*TESTCLR=1*/
	write_cifcru_reg(CRU_BASE_USER+0x20, 0x80000000);/*TESTCLR=0*/
	write_cifcru_reg(CRU_BASE_USER+0x20, 0x40004000);/*RSTZCAL=1*/	
#if (LANE_NUMBER == 1) 
		MIPI_DPHY_WriteReg(0xAC, 0x00); /* 1 lane */
#elif (LANE_NUMBER == 2)
		MIPI_DPHY_WriteReg(0xAC, 0x01); /* 2 lane */
#elif (LANE_NUMBER == 4)  
		MIPI_DPHY_WriteReg(0xAC, 0x03); /* 4 lane */
#endif
#if (LANE_NUMBER == 1)
	/*600-650M*/
		MIPI_DPHY_WriteReg(0x34, 0x0a);
	/*	MIPI_DPHY_WriteReg(0x44,0x10); */
#elif (LANE_NUMBER == 2)
	/*300-330M*/
		MIPI_DPHY_WriteReg(0x34, 0x0a);
		MIPI_DPHY_WriteReg(0x44, 0x0a);
#elif (LANE_NUMBER == 4)
	/*200-220M*/
		MIPI_DPHY_WriteReg(0x34, 0x06);
		MIPI_DPHY_WriteReg(0x44, 0x06);
#endif

		MIPI_DPHY_ReadReg(0x00);/*Normal operation*/
		/*TESTEN   =0,TESETCLK=1*/
		write_cifcru_reg(CRU_BASE_USER+0x20, 0x03000100);
#if (LANE_NUMBER == 1)
		/*ENABLE_0 =0,ENABLE_1=0*/
		write_cifcru_reg(CRU_BASE_USER+0x20, 0x30000000);
#elif (LANE_NUMBER == 2)
		/*ENABLE_0 =1,ENABLE_1=0*/
		write_cifcru_reg(CRU_BASE_USER+0x20, 0x30002000);
#elif (LANE_NUMBER == 4)
		/*ENABLE_0 =1,ENABLE_1=1*/
		write_cifcru_reg(CRU_BASE_USER+0x20, 0x30003000);
#endif
		/*SHUTDOWNZ=1*/
		write_cifcru_reg(CRU_BASE_USER+0x20, 0x08000800);
		/*RSTZ	 =1*/
		write_cifcru_reg(CRU_BASE_USER+0x20, 0x04000400);
#endif
	} else if (input_sel == 1){

	} else {
		goto fail;
	}

	return 0;
fail:
	return -1;
}

static int soc_clk_enable(void)
{
#ifndef FPGA_TEST
	struct cif_isp11_clk_rst_rk1108 *clk_rst = &rk1108->clk_rst;

	clk_prepare_enable(clk_rst->hclk_isp);
	clk_prepare_enable(clk_rst->aclk_isp);
	clk_prepare_enable(clk_rst->sclk_isp);
	clk_prepare_enable(clk_rst->sclk_isp_jpe);
	clk_prepare_enable(clk_rst->sclk_mipidsi_24m);
	clk_prepare_enable(clk_rst->pclk_isp_in);
	clk_prepare_enable(clk_rst->pclk_mipi_csi);
#endif
	return 0;
}

static int soc_clk_disable(void)
{
#ifndef FPGA_TEST
	struct cif_isp11_clk_rst_rk1108 *clk_rst = &rk1108->clk_rst;

	clk_disable_unprepare(clk_rst->hclk_isp);
	clk_disable_unprepare(clk_rst->aclk_isp);
	clk_disable_unprepare(clk_rst->sclk_isp);
	clk_disable_unprepare(clk_rst->sclk_isp_jpe);
	clk_disable_unprepare(clk_rst->sclk_mipidsi_24m);
	clk_disable_unprepare(clk_rst->pclk_isp_in);
	clk_disable_unprepare(clk_rst->pclk_mipi_csi);
#endif
	return 0;
}

static int soc_init(struct pltfrm_soc_init_para *init)
{
#ifndef FPGA_TEST
	struct cif_isp11_clk_rst_rk1108 *clk_rst;
	struct resource *res;
#endif
	struct platform_device *pdev = init->pdev;
	struct device_node *np = pdev->dev.of_node, *node;
	int err;

	rk1108 = (struct cif_isp11_rk1108 *)devm_kzalloc(
				&pdev->dev,
				sizeof(struct cif_isp11_rk1108),
				GFP_KERNEL);
	if (!rk1108) {
		dev_err(&pdev->dev, "Can't allocate cif_isp11_rk1108 \n");
		err = -ENOMEM;
		goto alloc_failed;
	}

	node = of_parse_phandle(np, "rockchip,grf", 0);
	if (node) {
		rk1108->regmap_grf = syscon_node_to_regmap(node);
		if (IS_ERR(rk1108->regmap_grf)) {
			dev_err(&pdev->dev, "Can't allocate cif_isp11_rk1108 \n");
			err = -ENODEV;
			goto regmap_failed;
		}
	}
#ifndef FPGA_TEST
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "csiphy_base");
	if (res == NULL) {
		dev_err(&pdev->dev,
			"platform_get_resource_byname csiphy_base failed\n");
		err = -ENODEV;
		goto regmap_failed;
	}
	rk1108->csiphy_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(rk1108->csiphy_base)) {
		dev_err(&pdev->dev, "devm_ioremap_resource failed\n");
		if (IS_ERR(rk1108->csiphy_base))
			err = PTR_ERR(rk1108->csiphy_base);
		else
			err = -ENODEV;
		goto regmap_failed;
	}

	clk_rst = &rk1108->clk_rst;
	clk_rst->aclk_isp = devm_clk_get(&pdev->dev, "aclk_isp");
	clk_rst->hclk_isp = devm_clk_get(&pdev->dev, "hclk_isp");
	clk_rst->sclk_isp = devm_clk_get(&pdev->dev, "sclk_isp");
	clk_rst->sclk_isp_jpe = devm_clk_get(&pdev->dev, "sclk_isp_jpe");	
	clk_rst->sclk_mipidsi_24m = devm_clk_get(&pdev->dev, "sclk_mipidsi_24m");	
	clk_rst->pclk_mipi_csi = devm_clk_get(&pdev->dev, "pclk_mipi_csi");
	clk_rst->isp_rst = devm_reset_control_get(&pdev->dev, "rst_isp");
	clk_rst->pclk_isp_in = devm_clk_get(&pdev->dev, "pclk_isp_in");

	if (IS_ERR_OR_NULL(clk_rst->aclk_isp) ||
		IS_ERR_OR_NULL(clk_rst->hclk_isp) ||
		IS_ERR_OR_NULL(clk_rst->sclk_isp) ||
		IS_ERR_OR_NULL(clk_rst->sclk_isp_jpe) ||
		IS_ERR_OR_NULL(clk_rst->pclk_mipi_csi) ||
		IS_ERR_OR_NULL(clk_rst->isp_rst) ||
		IS_ERR_OR_NULL(clk_rst->pclk_isp_in) ||
		IS_ERR_OR_NULL(clk_rst->sclk_mipidsi_24m)) {
		dev_err(&pdev->dev,"Get rk1108 cif isp11 clock resouce failed !\n");
		err = -EINVAL;
		goto clk_failed;
	}

	clk_set_rate(clk_rst->sclk_isp, 400000000);
	clk_set_rate(clk_rst->sclk_isp_jpe, 400000000);
	reset_control_deassert(clk_rst->isp_rst);
#endif
	rk1108->isp_base = init->isp_base;
	return 0;

#ifndef FPGA_TEST
clk_failed:
	if (!IS_ERR_OR_NULL(clk_rst->aclk_isp)) {
		devm_clk_put(&pdev->dev, clk_rst->aclk_isp);
	}
	if (!IS_ERR_OR_NULL(clk_rst->hclk_isp)) {
		devm_clk_put(&pdev->dev, clk_rst->hclk_isp);
	}
	if (!IS_ERR_OR_NULL(clk_rst->sclk_isp)) {
		devm_clk_put(&pdev->dev, clk_rst->sclk_isp);
	}
	if (!IS_ERR_OR_NULL(clk_rst->sclk_isp_jpe)) {
		devm_clk_put(&pdev->dev, clk_rst->sclk_isp_jpe);
	}
	if (!IS_ERR_OR_NULL(clk_rst->pclk_mipi_csi)) {
		devm_clk_put(&pdev->dev, clk_rst->pclk_mipi_csi);
	}
	if (!IS_ERR_OR_NULL(clk_rst->pclk_isp_in)) {
		devm_clk_put(&pdev->dev, clk_rst->pclk_isp_in);
	}
	if (!IS_ERR_OR_NULL(clk_rst->sclk_mipidsi_24m)) {
		devm_clk_put(&pdev->dev, clk_rst->sclk_mipidsi_24m);
	}	
	
	if (!IS_ERR_OR_NULL(clk_rst->isp_rst)) {
		reset_control_put(clk_rst->isp_rst);
	}
#endif
regmap_failed:
	

alloc_failed:
	
	return err;

}

int pltfrm_rk1108_cfg (
		struct pltfrm_soc_cfg_para *cfg)
{
	switch (cfg->cmd) {
	case PLTFRM_MCLK_CFG: {
		#if 0
		struct pltfrm_soc_mclk_para *mclk_para;
		mclk_para = (struct pltfrm_soc_mclk_para *)cfg->cfg_para;

		if (mclk_para->io_voltage == PLTFRM_IO_1V8)
			write_grf_reg(GRF_IO_VSEL_OFFSET, DVP_V18SEL);
		else
			write_grf_reg(GRF_IO_VSEL_OFFSET, DVP_V33SEL);
		
		write_grf_reg(GRF_GPIO2B_E_OFFSET,
			CIF_CLKOUT_STRENGTH(mclk_para->drv_strength));
		#endif
		break;
	}
	case PLTFRM_MIPI_DPHY_CFG:
		mipi_dphy_cfg((struct pltfrm_cam_mipi_config *)cfg->cfg_para);
		break;

	case PLTFRM_CLKEN:
		soc_clk_enable();
		break;

	case PLTFRM_CLKDIS:
		soc_clk_disable();
		break;

	case PLTFRM_CLKRST: 
		#ifndef FPGA_TEST
		reset_control_assert(rk1108->clk_rst.isp_rst);
		udelay(10);
		reset_control_deassert(rk1108->clk_rst.isp_rst);
		#endif
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

