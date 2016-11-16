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
#include "cif_isp11_regs.h"

/*
*GRF Registers
*/
#define GRF_SOC_CON5_OFFSET    (0x0414)

#define DPHY_RX_FORCERXMODE_MASK		(0xf<<16)
#define DPHY_RX_FORCERXMODE_BIT			(0x0)

/*
*MIPI PHY Registers
*/
#define DPHY_CTRL_BASE	0x00
#define DPHY_CTRL_LANE_ENABLE	(DPHY_CTRL_BASE + 0)
#define DPHY_CTRL_PWRCTL		(DPHY_CTRL_BASE + 4)
#define DPHY_CTRL_DIG_RST		(DPHY_CTRL_BASE + 0x80)
#define DPHY_CTRL_SIG_INV		(DPHY_CTRL_BASE + 0x84)

#define DPHY_CLOCK_LANE_BASE	0x100
#define DPHY_CLOCK_THS_SETTLE	 	(DPHY_CLOCK_LANE_BASE + 0x00)

#define DPHY_DATA_LANE0_BASE 0x180
#define DPHY_DATA_LANE0_THS_SETTLE (DPHY_DATA_LANE0_BASE + 0x00)

#define DPHY_DATA_LANE1_BASE 0x200
#define DPHY_DATA_LANE1_THS_SETTLE (DPHY_DATA_LANE0_BASE + 0x00)

#define DPHY_DATA_LANE2_BASE 0x280
#define DPHY_DATA_LANE2_THS_SETTLE (DPHY_DATA_LANE0_BASE + 0x00)

#define DPHY_DATA_LANE3_BASE 0x300
#define DPHY_DATA_LANE3_THS_SETTLE (DPHY_DATA_LANE0_BASE + 0x00)

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
	struct reset_control *isp_niu_arst;
	struct reset_control *isp_niu_hrst;
	struct reset_control *isp_hrst;
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
	{800, 1000, 0x0a}
};

static struct cif_isp11_rk1108 *rk1108;

static int mipi_dphy_cfg (struct pltfrm_cam_mipi_config *para)
{
	unsigned char hsfreqrange = 0xff, i;
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
		if (para->bit_rate > 1000) {
			hsfreqrange = 0x0a;
			printk(KERN_WARNING
				"%s: hsfreqrange(%d Mbps) isn't support,"
				"so use 1Gbps setting!\n",
				__func__,
				para->bit_rate);
		} else if (para->bit_rate <= 80) {
			hsfreqrange = 0x00;
			printk(KERN_WARNING
				"%s: hsfreqrange(%d Mbps) isn't support,"
				"so use 80Mbps setting!\n",
				__func__,
				para->bit_rate);
		}
	}

	/*hsfreqrange <<= 1;*/
	input_sel = para->dphy_index;
	datalane_en = 0;
	for (i = 0; i < para->nb_lanes; i++)
		datalane_en |= (1<<i);

	if (input_sel == 0) {
		write_csiphy_reg(DPHY_CTRL_PWRCTL, 0xe4);

		/*set data lane num and enable clock lane */
		write_csiphy_reg(DPHY_CTRL_LANE_ENABLE,
			((datalane_en << 2)|(0x1<<6)|0x1));
		/*Reset dphy analog part*/
		write_csiphy_reg(DPHY_CTRL_PWRCTL, 0xe0);
		msleep(1);
		/*Reset dphy digital part*/
		write_csiphy_reg(DPHY_CTRL_DIG_RST, 0x1e);
		write_csiphy_reg(DPHY_CTRL_DIG_RST, 0x1f);

		write_grf_reg(GRF_SOC_CON5_OFFSET,
			DPHY_RX_FORCERXMODE_MASK |
			DPHY_RX_FORCERXMODE_BIT);

		/*set clock lane */
		write_csiphy_reg(DPHY_CLOCK_THS_SETTLE,
			hsfreqrange |
			(read_csiphy_reg(DPHY_CLOCK_THS_SETTLE) & (~0xf)));
		if (para->nb_lanes >= 0x00) {/*lane0*/
			write_csiphy_reg(DPHY_DATA_LANE0_THS_SETTLE,
				hsfreqrange |
				(read_csiphy_reg(DPHY_DATA_LANE0_THS_SETTLE)&(~0xf)));
		}
		if (para->nb_lanes >= 0x02) {/*lane1*/
			write_csiphy_reg(DPHY_DATA_LANE1_THS_SETTLE,
				hsfreqrange |
				(read_csiphy_reg(DPHY_DATA_LANE1_THS_SETTLE)&(~0xf)));;
		}
		if (para->nb_lanes >= 0x04) {/*lane4*/
			write_csiphy_reg(DPHY_DATA_LANE2_THS_SETTLE,
				hsfreqrange |
				(read_csiphy_reg(DPHY_DATA_LANE2_THS_SETTLE)&(~0xf)));
			write_csiphy_reg(DPHY_DATA_LANE3_THS_SETTLE,
				hsfreqrange |
				(read_csiphy_reg(DPHY_DATA_LANE3_THS_SETTLE)&(~0xf)));
		}
		/*
		*MIPI CTRL bit8:11 SHUTDOWN_LANE are invert
		*connect to dphy pin_enable_x
		*/
		write_cifisp_reg(CIF_MIPI_CTRL,
			read_cifisp_reg(CIF_MIPI_CTRL) & (~(0x0f<<8)));
	} else {
		printk(KERN_ERR
			"MIPI DPHY %d isn't support for rk1108\n",
			input_sel);
		goto fail;
	}

	return 0;
fail:
	return -1;
}

static int soc_clk_enable(void)
{
	struct cif_isp11_clk_rst_rk1108 *clk_rst = &rk1108->clk_rst;

	clk_prepare_enable(clk_rst->hclk_isp);
	clk_prepare_enable(clk_rst->aclk_isp);
	clk_prepare_enable(clk_rst->sclk_isp);
	clk_prepare_enable(clk_rst->sclk_isp_jpe);
	clk_prepare_enable(clk_rst->sclk_mipidsi_24m);
	clk_prepare_enable(clk_rst->pclk_isp_in);
	clk_prepare_enable(clk_rst->pclk_mipi_csi);

	reset_control_assert(clk_rst->isp_rst);
	reset_control_assert(clk_rst->isp_niu_arst);
	reset_control_assert(clk_rst->isp_niu_hrst);
	reset_control_assert(clk_rst->isp_hrst);
	udelay(10);
	reset_control_deassert(clk_rst->isp_rst);
	reset_control_deassert(clk_rst->isp_niu_arst);
	reset_control_deassert(clk_rst->isp_niu_hrst);
	reset_control_deassert(clk_rst->isp_hrst);

	return 0;
}

static int soc_clk_disable(void)
{
	struct cif_isp11_clk_rst_rk1108 *clk_rst = &rk1108->clk_rst;

	clk_disable_unprepare(clk_rst->hclk_isp);
	clk_disable_unprepare(clk_rst->aclk_isp);
	clk_disable_unprepare(clk_rst->sclk_isp);
	clk_disable_unprepare(clk_rst->sclk_isp_jpe);
	clk_disable_unprepare(clk_rst->sclk_mipidsi_24m);
	clk_disable_unprepare(clk_rst->pclk_isp_in);
	clk_disable_unprepare(clk_rst->pclk_mipi_csi);
	return 0;
}

static int soc_init(struct pltfrm_soc_init_para *init)
{
	struct cif_isp11_clk_rst_rk1108 *clk_rst;
	struct resource *res;
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
	clk_rst->pclk_isp_in = devm_clk_get(&pdev->dev, "pclk_isp_in");

	clk_rst->isp_rst = devm_reset_control_get(&pdev->dev, "rst_isp");
	clk_rst->isp_niu_arst = devm_reset_control_get(&pdev->dev, "rst_isp_niu_a");
	clk_rst->isp_niu_hrst = devm_reset_control_get(&pdev->dev, "rst_isp_niu_h");
	clk_rst->isp_hrst = devm_reset_control_get(&pdev->dev, "rst_isp_h");

	if (IS_ERR_OR_NULL(clk_rst->aclk_isp) ||
		IS_ERR_OR_NULL(clk_rst->hclk_isp) ||
		IS_ERR_OR_NULL(clk_rst->sclk_isp) ||
		IS_ERR_OR_NULL(clk_rst->sclk_isp_jpe) ||
		IS_ERR_OR_NULL(clk_rst->pclk_mipi_csi) ||
		IS_ERR_OR_NULL(clk_rst->isp_rst) ||
		IS_ERR_OR_NULL(clk_rst->pclk_isp_in) ||
		IS_ERR_OR_NULL(clk_rst->sclk_mipidsi_24m) ||
		IS_ERR_OR_NULL(clk_rst->isp_niu_arst) ||
		IS_ERR_OR_NULL(clk_rst->isp_niu_hrst) ||
		IS_ERR_OR_NULL(clk_rst->isp_hrst)) {
		dev_err(&pdev->dev, "Get rk1108 cif isp11 clock resouce failed !\n");
		err = -EINVAL;
		goto clk_failed;
	}

	clk_set_rate(clk_rst->sclk_isp, 400000000);
	clk_set_rate(clk_rst->sclk_isp_jpe, 400000000);
	reset_control_deassert(clk_rst->isp_rst);

	rk1108->isp_base = init->isp_base;
	return 0;

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
regmap_failed:


alloc_failed:

	return err;

}

int pltfrm_rk1108_cfg (
		struct pltfrm_soc_cfg_para *cfg)
{
	switch (cfg->cmd) {
	case PLTFRM_MCLK_CFG:
		break;

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
		reset_control_assert(rk1108->clk_rst.isp_rst);
		usleep_range(5, 30);
		reset_control_deassert(rk1108->clk_rst.isp_rst);
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

