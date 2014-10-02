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

#include "internal.h"

static struct dw_dma_platform_data dw_priv_pdata = {
	.is_private = 1,
	.chan_allocation_order = CHAN_ALLOCATION_ASCENDING,
	.chan_priority = CHAN_PRIORITY_ASCENDING,
};

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

	err = dw_dma_probe(chip, &dw_priv_pdata);
	if (err)
		return ERR_PTR(err);

	return chip;
}
EXPORT_SYMBOL_GPL(dw_priv_probe);

int dw_priv_remove(void *dma)
{
	struct dw_dma_chip *chip = dma;

	return dw_dma_remove(chip);
}
EXPORT_SYMBOL_GPL(dw_priv_remove);
