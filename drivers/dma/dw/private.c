/*
 * Private driver for the Synopsys DesignWare DMA Controller
 *
 * Copyright (C) 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>

#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include "internal.h"

static struct dw_dma_platform_data dw_priv_pdata = {
	.is_private = 1,
	.chan_allocation_order = CHAN_ALLOCATION_ASCENDING,
	.chan_priority = CHAN_PRIORITY_ASCENDING,
};

static int dw_register_dummy_clk(struct device *dev, struct dw_dma_platform_data *pdata)
{
        struct clk *hclk;
        struct clk_init_data *clk_init;

        clk_init = devm_kzalloc(dev, sizeof(struct clk_init_data), GFP_KERNEL);
        if (clk_init == NULL) {
                dev_err(dev, "DMA: can't kzalloc memory for clk_init\n");
		return -ENOMEM;
        }

        clk_init->name  = "hclk";
        clk_init->ops   = &clk_fixed_rate_ops;
        clk_init->flags = CLK_IS_ROOT;

        pdata->hw_clk = devm_kzalloc(dev, sizeof(struct clk_hw), GFP_KERNEL);
        if (pdata->hw_clk == NULL) {
                dev_err(dev, "DMA: can't kzalloc memory for hw_clk\n");
		return -ENOMEM;
        }

        pdata->hw_clk->init = clk_init;

        hclk = devm_clk_register(dev, pdata->hw_clk);
        if (hclk == NULL) {
                dev_err(dev, "DMA: hclk not registered\n");
		return -ENODEV;
        }

        pdata->lookup_clk = clkdev_alloc(hclk, pdata->hw_clk->init->name, NULL);
        if (pdata->lookup_clk == NULL) {
                dev_err(dev, "DMA: can't kzalloc memory for lookup_clk\n");
		return -ENOMEM;
        }

        clkdev_add(pdata->lookup_clk);

        return 0;
}

static void dw_unregister_dummy_clk(struct dw_dma_platform_data *pdata)
{
        if (pdata->lookup_clk) {
                clkdev_drop(pdata->lookup_clk);
                pdata->lookup_clk = NULL;
        }
}

void *dw_priv_probe(struct device *dev, struct resource *mem, int irq, u64 mask)
{
	struct dw_dma_chip *chip;
	int err;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return ERR_PTR(-ENOMEM);

	chip->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(chip->regs))
		return ERR_CAST(chip->regs);

	if (!mask)
		mask = DMA_BIT_MASK(32);

	err = dma_coerce_mask_and_coherent(dev, mask);
	if (err)
		return ERR_PTR(err);

	chip->irq = irq;
	chip->dev = dev;

	err = dw_register_dummy_clk(dev, &dw_priv_pdata);
	if (err)
		return ERR_PTR(err);

	err = dw_dma_probe(chip, &dw_priv_pdata);
	if (err)
		return ERR_PTR(err);

	return chip;
}
EXPORT_SYMBOL_GPL(dw_priv_probe);

int dw_priv_remove(void *dma)
{
	struct dw_dma_chip *chip = dma;

	dw_unregister_dummy_clk(&dw_priv_pdata);

	return dw_dma_remove(chip);
}
EXPORT_SYMBOL_GPL(dw_priv_remove);
