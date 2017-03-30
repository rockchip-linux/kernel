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

/* rk816 */
#define RK816_GPIO_REG		0x50
#define RK816_OUT0_DIRMASK	BIT(4)
#define RK816_OUT0_VALMASK	BIT(3)
#define RK816_OUT0_FUNMASK	BIT(2)

/* rk805 */
#define RK805_GPIO_REG		0x52
#define RK805_OUT0_VALMASK	BIT(0)
#define RK805_OUT1_VALMASK	BIT(1)

static struct rk8xx_gpio_data *rk8xx_gpio;

struct rk816_gpio_info {
	struct rk816 *rk816;
	struct gpio_chip gpio_chip;
};

static int rk816_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	int ret;
	struct rk816_gpio_info *gi = dev_get_drvdata(chip->dev);

	/* iomux */
	if (rk8xx_gpio->gpio_reg[offset].fun_msk) {
		ret = rk816_set_bits(gi->rk816,
				     rk8xx_gpio->gpio_reg[offset].reg,
				     rk8xx_gpio->gpio_reg[offset].fun_msk,
				     rk8xx_gpio->gpio_reg[offset].fun_msk);
		if (ret < 0) {
			dev_err(chip->dev, "set gpio%d func fail\n", offset);
			return ret;
		}
	}

	/* direction */
	if (rk8xx_gpio->gpio_reg[offset].dir_msk) {
		ret = rk816_clear_bits(gi->rk816,
				       rk8xx_gpio->gpio_reg[offset].reg,
				       rk8xx_gpio->gpio_reg[offset].dir_msk);
		if (ret < 0) {
			dev_err(chip->dev, "set gpio%d input fail\n", offset);
			return ret;
		}
	}

	return 0;
}

static int rk816_gpio_direction_output(struct gpio_chip *chip,
				       unsigned offset, int value)
{
	int ret;
	struct rk816_gpio_info *gi = dev_get_drvdata(chip->dev);

	/* iomux */
	if (rk8xx_gpio->gpio_reg[offset].fun_msk) {
		ret = rk816_set_bits(gi->rk816,
				     rk8xx_gpio->gpio_reg[offset].reg,
				     rk8xx_gpio->gpio_reg[offset].fun_msk,
				     rk8xx_gpio->gpio_reg[offset].fun_msk);
		if (ret < 0) {
			dev_err(chip->dev, "set gpio%d func fail\n", offset);
			return ret;
		}
	}

	/* direction */
	if (rk8xx_gpio->gpio_reg[offset].dir_msk) {
		ret = rk816_set_bits(gi->rk816,
				     rk8xx_gpio->gpio_reg[offset].reg,
				     rk8xx_gpio->gpio_reg[offset].dir_msk,
				     rk8xx_gpio->gpio_reg[offset].dir_msk);
		if (ret < 0) {
			dev_err(chip->dev, "set gpio%d dir out fail\n", offset);
			return ret;
		}
	}

	if (value)
		ret = rk816_set_bits(gi->rk816,
				     rk8xx_gpio->gpio_reg[offset].reg,
				     rk8xx_gpio->gpio_reg[offset].val_msk,
				     rk8xx_gpio->gpio_reg[offset].val_msk);
	else
		ret = rk816_clear_bits(gi->rk816,
				       rk8xx_gpio->gpio_reg[offset].reg,
				       rk8xx_gpio->gpio_reg[offset].val_msk);
	if (ret < 0) {
		dev_err(chip->dev, "set gpio%d value fail\n", offset);
		return ret;
	}

	return 0;
}

static int rk816_gpio_get_value(struct gpio_chip *chip, unsigned offset)
{
	int ret;
	struct rk816_gpio_info *gi = dev_get_drvdata(chip->dev);

	ret = rk816_reg_read(gi->rk816, rk8xx_gpio->gpio_reg[offset].reg);
	if (ret < 0) {
		dev_err(chip->dev, "get gpio%d value fail\n", offset);
		return ret;
	}

	return (ret & rk8xx_gpio->gpio_reg[offset].val_msk) ? 1 : 0;
}

static void rk816_gpio_set_value(struct gpio_chip *chip,
				 unsigned offset, int value)
{
	int ret;
	struct rk816_gpio_info *gi = dev_get_drvdata(chip->dev);

	if (value)
		ret = rk816_set_bits(gi->rk816,
				     rk8xx_gpio->gpio_reg[offset].reg,
				     rk8xx_gpio->gpio_reg[offset].val_msk,
				     rk8xx_gpio->gpio_reg[offset].val_msk);
	else
		ret = rk816_clear_bits(gi->rk816,
				       rk8xx_gpio->gpio_reg[offset].reg,
				       rk8xx_gpio->gpio_reg[offset].val_msk);
	if (ret < 0)
		dev_err(chip->dev, "set gpio%d value fail\n", offset);
}

/* rk816: one gpio: out/input, ts/gpio */
static struct rk8xx_gpio_reg rk816_gpio_reg[] = {
	{
		.reg = RK816_GPIO_REG,
		.dir_msk = RK816_OUT0_DIRMASK,
		.val_msk = RK816_OUT0_VALMASK,
		.fun_msk = RK816_OUT0_FUNMASK,
	},
};

/* rk805: two gpio: output only */
static struct rk8xx_gpio_reg rk805_gpio_reg[] = {
	{
		.reg = RK805_GPIO_REG,
		.val_msk = RK805_OUT0_VALMASK,
	},
	{
		.reg = RK805_GPIO_REG,
		.val_msk = RK805_OUT1_VALMASK,
	},
};

static struct rk8xx_gpio_data rk816_gpio_data = {
	.ngpio = 1,
	.gpio_reg = rk816_gpio_reg,
};

static struct rk8xx_gpio_data rk805_gpio_data = {
	.ngpio = 2,
	.gpio_reg = rk805_gpio_reg,
};

static int rk816_gpio_probe(struct platform_device *pdev)
{
	struct rk816 *rk816 = dev_get_drvdata(pdev->dev.parent);
	struct rk8xx_platform_data *pdata = pdev->dev.platform_data;
	struct rk816_gpio_info *gi;
	int rc;

	if (!strcmp(pdata->chip_name, "rk816")) {
		rk8xx_gpio = &rk816_gpio_data;
	} else if (!strcmp(pdata->chip_name, "rk805")) {
		rk8xx_gpio = &rk805_gpio_data;
	} else {
		dev_err(&pdev->dev, "failed to match device data\n");
		return -EINVAL;
	}

	pr_info("%s: compatible %s\n", __func__, pdata->chip_name);

	gi = devm_kzalloc(&pdev->dev, sizeof(*gi), GFP_KERNEL);
	if (!gi)
		return -ENOMEM;

	gi->rk816 = rk816;
	gi->gpio_chip.direction_input = rk816_gpio_direction_input;
	gi->gpio_chip.direction_output = rk816_gpio_direction_output;
	gi->gpio_chip.get = rk816_gpio_get_value;
	gi->gpio_chip.set = rk816_gpio_set_value;
	gi->gpio_chip.can_sleep = 1;
	gi->gpio_chip.base = -1;
	gi->gpio_chip.ngpio = rk8xx_gpio->ngpio;
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
		.name = "rk8xx-gpio",
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
