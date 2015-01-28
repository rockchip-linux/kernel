/*
 * IMG Pistachio DWMAC glue layer
 *
 * Copyright (C) 2015 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/stmmac.h>

struct pistachio_dwmac_priv_data {
	struct clk *sys_clk;
};

static void *pistachio_dwmac_setup(struct platform_device *pdev)
{
	struct pistachio_dwmac_priv_data *pdata;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->sys_clk = devm_clk_get(&pdev->dev, "sys");
	if (IS_ERR(pdata->sys_clk)) {
		dev_err(&pdev->dev, "Failed to get sys clock: %ld\n",
			PTR_ERR(pdata->sys_clk));
		return pdata->sys_clk;
	}

	return pdata;
}

static int pistachio_dwmac_init(struct platform_device *pdev, void *priv)
{
	struct pistachio_dwmac_priv_data *pdata = priv;

	return clk_prepare_enable(pdata->sys_clk);
}

static void pistachio_dwmac_exit(struct platform_device *pdev, void *priv)
{
	struct pistachio_dwmac_priv_data *pdata = priv;

	clk_disable_unprepare(pdata->sys_clk);
}

const struct stmmac_of_data pistachio_dwmac_data = {
	.setup = pistachio_dwmac_setup,
	.init = pistachio_dwmac_init,
	.exit = pistachio_dwmac_exit,
};
