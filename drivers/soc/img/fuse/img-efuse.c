/*
 * Imagination Technologies Generic eFuse driver
 *
 * Copyright (c) 2014 Imagination Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * Based on drivers/misc/eeprom/sunxi_sid.c Copyright (c) 2013 Oliver Schinagl
 */
#include <linux/clk.h>
#include <linux/compiler.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#include <soc/img/img-efuse.h>

struct img_efuse_soc_data {
	unsigned int size;
};

struct img_efuse {
	void __iomem *base;
	const struct img_efuse_soc_data *data;
	struct clk *osc_clk;
	struct clk *sys_clk;
};

static struct img_efuse *efuse_dev;
static bool efuse_initialized;

static u8 img_efuse_read_byte(unsigned int offset)
{
	u32 data;

	if (offset >= efuse_dev->data->size)
		return 0;

	data = readl(efuse_dev->base + round_down(offset, 4));
	data >>= (offset % 4) * 8;

	return data;
}

int img_efuse_readl(unsigned int offset, u32 *value)
{
	if (!efuse_initialized)
		return -ENODEV;

	if ((offset > (efuse_dev->data->size - 4)) || (offset % 4 != 0))
		return -EINVAL;

	*value = readl(efuse_dev->base + offset);

	return 0;
}
EXPORT_SYMBOL_GPL(img_efuse_readl);

static ssize_t img_efuse_read(struct file *fd, struct kobject *kobj,
			      struct bin_attribute *attr, char *buf,
			      loff_t pos, size_t size)
{
	int i;
	struct platform_device *pdev;
	struct img_efuse *efuse;

	pdev = to_platform_device(kobj_to_dev(kobj));
	efuse = platform_get_drvdata(pdev);

	if (pos < 0 || pos >= efuse->data->size)
		return 0;

	if (size > efuse->data->size - pos)
		size = efuse->data->size - pos;

	for (i = 0; i < size; i++)
		buf[i] = img_efuse_read_byte(pos + i);

	return i;
}

static struct bin_attribute img_efuse_bin_attr = {
	.attr = { .name = "efuse", .mode = S_IRUGO, },
	.read = img_efuse_read,
};

static const struct img_efuse_soc_data pistachio_efuse = {
	.size = 16,
};

static const struct of_device_id img_efuse_of_match[] = {
	{ .compatible = "img,pistachio-efuse", .data = &pistachio_efuse },
	{},
};
MODULE_DEVICE_TABLE(of, img_efuse_of_match);

static int img_efuse_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;
	const struct of_device_id *of_dev_id;

	efuse_dev = devm_kzalloc(&pdev->dev, sizeof(struct img_efuse *),
				 GFP_KERNEL);
	if (!efuse_dev)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	efuse_dev->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(efuse_dev->base))
		return PTR_ERR(efuse_dev->base);

	of_dev_id = of_match_device(img_efuse_of_match, &pdev->dev);
	if (!of_dev_id)
		return -ENODEV;
	efuse_dev->data = of_dev_id->data;

	efuse_dev->sys_clk = devm_clk_get(&pdev->dev, "sys");
	if (!IS_ERR(efuse_dev->sys_clk)) {
		ret = clk_prepare_enable(efuse_dev->sys_clk);
		if (ret < 0) {
			dev_err(&pdev->dev, "could not enable system clock\n");
			return ret;
		}
	}

	efuse_dev->osc_clk = devm_clk_get(&pdev->dev, "osc");
	if (!IS_ERR(efuse_dev->osc_clk)) {
		ret = clk_prepare_enable(efuse_dev->osc_clk);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"could not enable oscillator clock\n");
			goto disable_sysclk;
		}
	}

	platform_set_drvdata(pdev, efuse_dev);

	img_efuse_bin_attr.size = efuse_dev->data->size;
	if (device_create_bin_file(&pdev->dev, &img_efuse_bin_attr)) {
		ret = -ENODEV;
		goto disable_oscclk;
	}

	efuse_initialized = true;
	return 0;

disable_oscclk:
	if (!IS_ERR(efuse_dev->osc_clk))
		clk_disable_unprepare(efuse_dev->osc_clk);
disable_sysclk:
	if (!IS_ERR(efuse_dev->sys_clk))
		clk_disable_unprepare(efuse_dev->sys_clk);

	return ret;
}

static int img_efuse_remove(struct platform_device *pdev)
{
	struct img_efuse *efuse = platform_get_drvdata(pdev);

	efuse_initialized = false;
	device_remove_bin_file(&pdev->dev, &img_efuse_bin_attr);

	if (!IS_ERR(efuse->osc_clk))
		clk_disable_unprepare(efuse->osc_clk);

	if (!IS_ERR(efuse->sys_clk))
		clk_disable_unprepare(efuse->sys_clk);

	return 0;
}

static struct platform_driver img_efuse_driver = {
	.probe = img_efuse_probe,
	.remove = img_efuse_remove,
	.driver = {
		.name = "img-efuse",
		.of_match_table = img_efuse_of_match,
	},
};
module_platform_driver(img_efuse_driver);

MODULE_AUTHOR("Arul Ramasamy<Arul.Ramasamy@imgtec.com>");
MODULE_AUTHOR("Jude Abraham<Jude.Abraham@imgtec.com>");
MODULE_DESCRIPTION("Imagination Technologies Generic eFuse driver");
MODULE_LICENSE("GPL v2");
