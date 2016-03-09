/*
 * driver/input/misc/rk816-pwrkey.c
 * Power Key driver for RK816 power management chip.
 *
 * Copyright (C) 2016, Rockchip Technology Co., Ltd.
 *
 * ChenJianhong <chenjh@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/rk816.h>
#include <linux/regmap.h>

struct rk816_pwr_button {
	struct rk816 *rk816;
	struct input_dev *input_dev;
	struct device		*dev;
	int report_key;
};

static irqreturn_t rk816_powkey_irq_falling(int irq, void *_pwr)
{
	struct rk816_pwr_button *pwr = _pwr;
	int ret = 0;

	ret = rk816_set_bits(pwr->rk816, RK816_INT_STS_REG1,
			     RK816_PWR_FALL_INT_STATUS,
			     RK816_PWR_FALL_INT_STATUS);
	if (ret < 0) {
		pr_err("%s:Failed to read pwrkey status: %d\n", __func__, ret);
		return ret;
	}

	input_report_key(pwr->input_dev, pwr->report_key, 1);
	input_sync(pwr->input_dev);

	return IRQ_HANDLED;
}

static irqreturn_t rk816_powkey_irq_rising(int irq, void *_pwr)
{
	struct rk816_pwr_button *pwr = _pwr;
	int ret = 0;

	ret = rk816_set_bits(pwr->rk816, RK816_INT_STS_REG1,
			     RK816_PWR_RISE_INT_STATUS,
			     RK816_PWR_RISE_INT_STATUS);
	if (ret < 0) {
		pr_err("%s:Failed to read pwrkey status: %d\n", __func__, ret);
		return ret;
	}

	input_report_key(pwr->input_dev, pwr->report_key, 0);
	input_sync(pwr->input_dev);

	return IRQ_HANDLED;
}

static int __init rk816_pwrkey_probe(struct platform_device *pdev)
{
	int err;
	struct rk816_pwr_button *pwrkey;
	struct rk816 *rk816 = dev_get_drvdata(pdev->dev.parent);
	int fall_irq = regmap_irq_get_virq(rk816->irq_data,
					   RK816_IRQ_PWRON_FALL);
	int rise_irq = regmap_irq_get_virq(rk816->irq_data,
					   RK816_IRQ_PWRON_RISE);
	pwrkey = devm_kzalloc(&pdev->dev,
			      sizeof(struct rk816_pwr_button), GFP_KERNEL);
	if (!pwrkey)
		return -ENOMEM;

	pwrkey->input_dev = devm_input_allocate_device(&pdev->dev);
	if (!pwrkey->input_dev) {
		dev_err(&pdev->dev, "Can't allocate power button\n");
		return -ENOMEM;
	}

	pwrkey->rk816 = rk816;
	pwrkey->report_key = KEY_POWER;
	pwrkey->dev = &pdev->dev;
	pwrkey->input_dev->evbit[0] = BIT_MASK(EV_KEY);
	pwrkey->input_dev->keybit[BIT_WORD(pwrkey->report_key)] =
			BIT_MASK(pwrkey->report_key);
	pwrkey->input_dev->name = "rk816_pwrkey";
	pwrkey->input_dev->phys = "rk816_pwrkey/input0";
	pwrkey->input_dev->dev.parent = &pdev->dev;
	err = devm_request_threaded_irq(&pdev->dev, fall_irq,
					NULL, rk816_powkey_irq_falling,
					IRQF_TRIGGER_FALLING,
					"rk816_pwrkey_fall", pwrkey);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't get fall irq for pwrkey: %d\n", err);
		return err;
	}
	err = devm_request_threaded_irq(&pdev->dev, rise_irq,
					NULL, rk816_powkey_irq_rising,
					IRQF_TRIGGER_RISING,
					"rk816_pwrkey_rise", pwrkey);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't get rise irq for pwrkey: %d\n", err);
		return err;
	}

	err = input_register_device(pwrkey->input_dev);
	if (err) {
		dev_err(&pdev->dev, "Can't register power button: %d\n", err);
		return err;
	}

	platform_set_drvdata(pdev, pwrkey);

	return 0;
}

static struct platform_driver rk816_pwrkey_driver = {
	.driver		= {
		.name	= "rk816-pwrkey",
		.owner	= THIS_MODULE,
	},
};

module_platform_driver_probe(rk816_pwrkey_driver,
			     rk816_pwrkey_probe);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rk816_pwrkey");
MODULE_DESCRIPTION("RK816 Power Button");
MODULE_AUTHOR("zhangqing <zhangqing@rock-chips.com>");

