/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/bootmem.h>
#include <asm/cacheflush.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/sched.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#endif
#include "rk_sfc_blk.h"
#include "rk_sfc_api.h"

#define RKNAND_VERSION_AND_DATE  "rksfcbase v1.1 2016-01-08"

struct rk_sfc_info {
	void __iomem	*reg_base;
	int	irq;
	int	clk_rate;
	struct clk	*clk;		/* sfc clk*/
	struct clk	*ahb_clk;	/* ahb clk gate*/
};

static struct rk_sfc_info g_sfc_info;
struct device *g_sfc_device;
static struct completion sfc_irq_complete;

unsigned long rksfc_dma_map_single(unsigned long ptr, int size, int dir)
{
#ifdef CONFIG_ARM64
	__dma_map_area((void *)ptr, size, dir);
	return ((unsigned long)virt_to_phys((void *)ptr));
#else
	return dma_map_single(NULL, (void *)ptr, size
		, dir ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
#endif
}

void rksfc_dma_unmap_single(unsigned long ptr, int size, int dir)
{
#ifdef CONFIG_ARM64
	__dma_unmap_area(phys_to_virt(ptr), size, dir);
#else
	dma_unmap_single(NULL, (dma_addr_t)ptr, size
		, dir ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
#endif
}

int rksfc_get_reg_addr(unsigned long *p_sfc_addr)
{
	*p_sfc_addr = (unsigned long)g_sfc_info.reg_base;
	return 0;
}

static irqreturn_t rk_sfc_interrupt(int irq, void *dev_id)
{
	sfc_clean_irq();
	complete(&sfc_irq_complete);
	return IRQ_HANDLED;
}

void rk_sfc_irq_flag_init(void)
{
	init_completion(&sfc_irq_complete);
}

void wait_for_sfc_irq_completed(void)
{
	wait_for_completion_timeout(&sfc_irq_complete, msecs_to_jiffies(10));
}

static int rk_sfc_irq_config(int mode, void *pfun)
{
	int ret = 0;
	int irq = g_sfc_info.irq;

	if (mode)
		ret = request_irq(irq, pfun, 0, "sfc", g_sfc_info.reg_base);
	else
		free_irq(irq,  NULL);
	return ret;
}

int rk_sfc_irq_init(void)
{
	int ret = 0;

	init_completion(&sfc_irq_complete);
	rk_sfc_irq_config(1, rk_sfc_interrupt);
	return ret;
}

int rk_sfc_irq_deinit(void)
{
	int ret = 0;

	rk_sfc_irq_config(0, rk_sfc_interrupt);
	return ret;
}

static int rksfc_probe(struct platform_device *pdev)
{
	int irq;
	struct resource	*mem;
	void __iomem	*membase;

	g_sfc_device = &pdev->dev;
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	membase = devm_ioremap_resource(&pdev->dev, mem);
	if (membase == 0) {
		dev_err(&pdev->dev, "no reg resource?\n");
		return -1;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq;
	}

	g_sfc_info.irq = irq;
	g_sfc_info.reg_base = membase;

	g_sfc_info.ahb_clk = devm_clk_get(&pdev->dev, "hclk_sfc");
	g_sfc_info.clk = devm_clk_get(&pdev->dev, "clk_sfc");
	if (unlikely(IS_ERR(g_sfc_info.clk)) ||
	    unlikely(IS_ERR(g_sfc_info.ahb_clk))) {
		dev_err(&pdev->dev, "rksfc_probe get clk error\n");
		return -1;
	}
	clk_prepare_enable(g_sfc_info.ahb_clk);
	clk_set_rate(g_sfc_info.clk, 100 * 1000 * 1000);
	g_sfc_info.clk_rate = clk_get_rate(g_sfc_info.clk);
	clk_prepare_enable(g_sfc_info.clk);
	dev_info(&pdev->dev,
		 "rksfc_probe clk rate = %d\n",
		 g_sfc_info.clk_rate);
	rk_sfc_irq_init();
	return rksfc_dev_init(g_sfc_info.reg_base);
}

#ifdef CONFIG_OF
static const struct of_device_id of_rk_sfc_match[] = {
	{.compatible = "rockchip,sfc"},
	{}
};
#endif

static struct platform_driver rksfc_driver = {
	.probe		= rksfc_probe,
	.driver		= {
		.name	= "sfc",
#ifdef CONFIG_OF
		.of_match_table	= of_rk_sfc_match,
#endif
		.owner	= THIS_MODULE,
	},
};

static void __exit rksfc_driver_exit(void)
{
	rksfc_dev_exit();
	rk_sfc_irq_deinit();
	platform_driver_unregister(&rksfc_driver);
}

static int __init rksfc_driver_init(void)
{
	int ret = 0;

	pr_err("%s\n", RKNAND_VERSION_AND_DATE);
	ret = platform_driver_register(&rksfc_driver);
	return ret;
}

module_init(rksfc_driver_init);
module_exit(rksfc_driver_exit);
MODULE_ALIAS(DRIVER_NAME);
