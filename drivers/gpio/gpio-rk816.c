/*
 *  drivers/gpio/gpio-rk816.c
 *  Driver for Rockchip RK816 PMIC GPIO
 *
 *  Copyright (C) 2016, Rockchip Technology Co., Ltd.
 *  ChenJianhong <chenjh@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/mfd/rk816.h>

#define GPIO_REG	0x50
#define GPIO_DIRMASK	(1 << 4)
#define GPIO_VALMASK	(1 << 3)
#define GPIO_FUNMASK	(1 << 2)

struct rk816_gpio_info {
	struct rk816 *rk816;
	struct gpio_chip gpio_chip;
};

static int rk816_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	int ret;
	struct rk816_gpio_info *gi = dev_get_drvdata(chip->dev);

	ret = rk816_set_bits(gi->rk816, GPIO_REG, GPIO_FUNMASK, GPIO_FUNMASK);
	if (ret < 0) {
		dev_err(chip->dev, "set gpio func fail\n");
		return ret;
	}

	ret = rk816_clear_bits(gi->rk816, GPIO_REG, GPIO_DIRMASK);
	if (ret < 0) {
		dev_err(chip->dev, "set gpio input fail\n");
		return ret;
	}

	return 0;
}

static int rk816_gpio_direction_output(struct gpio_chip *chip,
				       unsigned offset, int value)
{
	int ret;
	struct rk816_gpio_info *gi = dev_get_drvdata(chip->dev);

	ret = rk816_set_bits(gi->rk816, GPIO_REG, GPIO_FUNMASK, GPIO_FUNMASK);
	if (ret < 0) {
		dev_err(chip->dev, "set gpio func fail\n");
		return ret;
	}

	ret = rk816_set_bits(gi->rk816, GPIO_REG, GPIO_DIRMASK, GPIO_DIRMASK);
	if (ret < 0) {
		dev_err(chip->dev, "set gpio direction out fail\n");
		return ret;
	}

	if (value)
		ret = rk816_set_bits(gi->rk816, GPIO_REG, GPIO_VALMASK,
				     GPIO_VALMASK);
	else
		ret = rk816_clear_bits(gi->rk816, GPIO_REG, GPIO_VALMASK);
	if (ret < 0) {
		dev_err(chip->dev, "set gpio value fail\n");
		return ret;
	}

	return 0;
}

static int rk816_gpio_get_value(struct gpio_chip *chip, unsigned offset)
{
	int ret;
	struct rk816_gpio_info *gi = dev_get_drvdata(chip->dev);

	ret = rk816_reg_read(gi->rk816, GPIO_REG);
	if (ret < 0) {
		dev_err(chip->dev, "read gpio register fail\n");
		return ret;
	}

	return (ret & GPIO_VALMASK) ? 1 : 0;
}

static void rk816_gpio_set_value(struct gpio_chip *chip,
				 unsigned offset, int value)
{
	int ret;
	struct rk816_gpio_info *gi = dev_get_drvdata(chip->dev);

	if (value)
		ret = rk816_set_bits(gi->rk816, GPIO_REG, GPIO_VALMASK,
				     GPIO_VALMASK);
	else
		ret = rk816_clear_bits(gi->rk816, GPIO_REG, GPIO_VALMASK);
	if (ret < 0)
		dev_err(chip->dev, "set gpio value fail\n");
}

static int rk816_gpio_probe(struct platform_device *pdev)
{
	struct rk816 *rk816 = dev_get_drvdata(pdev->dev.parent);
	struct rk816_gpio_info *gi;
	int rc;

	gi = devm_kzalloc(&pdev->dev, sizeof(*gi), GFP_KERNEL);
	if (!gi)
		return -ENOMEM;

	gi->rk816 = rk816;
	gi->gpio_chip.direction_input = rk816_gpio_direction_input;
	gi->gpio_chip.direction_output = rk816_gpio_direction_output;
	gi->gpio_chip.get = rk816_gpio_get_value;
	gi->gpio_chip.set = rk816_gpio_set_value;
	gi->gpio_chip.can_sleep = 0;
	gi->gpio_chip.base = -1;
	gi->gpio_chip.ngpio = 1;
	gi->gpio_chip.label = pdev->name;
	gi->gpio_chip.dev = &pdev->dev;
	gi->gpio_chip.owner = THIS_MODULE;
#ifdef CONFIG_OF_GPIO
	gi->gpio_chip.of_node = rk816->dev->of_node;
#endif
	rc = gpiochip_add(&gi->gpio_chip);
	if (rc)
		goto out_dev;

	platform_set_drvdata(pdev, gi);
	dev_info(&pdev->dev, "driver success\n");
	return rc;

out_dev:
	return rc;
}

static int rk816_gpio_remove(struct platform_device *pdev)
{
	struct rk816_gpio_info *gi = platform_get_drvdata(pdev);
	int rc;

	rc = gpiochip_remove(&gi->gpio_chip);

	return 0;
}

static struct platform_driver rk816_gpio_driver = {
	.probe = rk816_gpio_probe,
	.remove = rk816_gpio_remove,
	.driver = {
		.name = "rk816-gpio",
		.owner = THIS_MODULE,
	},
};

static int rk816_gpio_init(void)
{
	return platform_driver_register(&rk816_gpio_driver);
}
fs_initcall_sync(rk816_gpio_init);

static void rk816_gpio_exit(void)
{
	platform_driver_unregister(&rk816_gpio_driver);
}
module_exit(rk816_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ChenJianhong <chenjh@rock-chips.com>");
MODULE_DESCRIPTION("GPIO driver for RK816");
