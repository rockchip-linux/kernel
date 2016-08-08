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
	struct workqueue_struct	*workqueue;
	struct work_struct fall_work;
	struct work_struct rise_work;
};

static struct rk8xx_pwrkey_data *rk8xx_pwrkey;

static struct rk8xx_pwrkey_data rk816_pwrkey = {
	.int_status_reg = RK816_INT_STS_REG1,
	.pwr_fall_irq = RK816_IRQ_PWRON_FALL,
	.pwr_rise_irq = RK816_IRQ_PWRON_RISE,
	.pwr_fall_int_status = RK816_PWR_FALL_INT_STATUS,
	.pwr_rise_int_status = RK816_PWR_RISE_INT_STATUS,
};

static struct rk8xx_pwrkey_data rk805_pwrkey = {
	.int_status_reg = RK805_INT_STS_REG,
	.pwr_fall_irq = RK805_IRQ_PWRON_FALL,
	.pwr_rise_irq = RK805_IRQ_PWRON_RISE,
	.pwr_fall_int_status = RK805_PWR_FALL_INT_STATUS,
	.pwr_rise_int_status = RK805_PWR_RISE_INT_STATUS,
};

static irqreturn_t rk816_powkey_irq_falling(int irq, void *_pwr)
{
	struct rk816_pwr_button *pwr = _pwr;

	queue_work(pwr->workqueue, &pwr->fall_work);
	return IRQ_HANDLED;
}

static irqreturn_t rk816_powkey_irq_rising(int irq, void *_pwr)
{
	struct rk816_pwr_button *pwr = _pwr;

	queue_work(pwr->workqueue, &pwr->rise_work);
	return IRQ_HANDLED;
}

static void rk816_pwrkey_fall_work(struct work_struct *work)
{
	int ret = 0;
	struct rk816_pwr_button *pwr = container_of(work,
			struct rk816_pwr_button, fall_work);

	ret = rk816_set_bits(pwr->rk816, rk8xx_pwrkey->int_status_reg,
			     rk8xx_pwrkey->pwr_fall_int_status,
			     rk8xx_pwrkey->pwr_fall_int_status);
	if (ret < 0) {
		pr_err("%s:Failed to read pwrkey status: %d\n", __func__, ret);
		return;
	}

	input_report_key(pwr->input_dev, pwr->report_key, 1);
	input_sync(pwr->input_dev);
}

static void rk816_pwrkey_rise_work(struct work_struct *work)
{
	int ret = 0;
	struct rk816_pwr_button *pwr = container_of(work,
			struct rk816_pwr_button, rise_work);

	ret = rk816_set_bits(pwr->rk816, rk8xx_pwrkey->int_status_reg,
			     rk8xx_pwrkey->pwr_rise_int_status,
			     rk8xx_pwrkey->pwr_rise_int_status);
	if (ret < 0) {
		pr_err("%s:Failed to read pwrkey status: %d\n", __func__, ret);
		return;
	}

	input_report_key(pwr->input_dev, pwr->report_key, 0);
	input_sync(pwr->input_dev);
}

static int rk816_pwrkey_probe(struct platform_device *pdev)
{
	int err;
	struct rk816_pwr_button *pwrkey;
	struct rk816 *rk816 = dev_get_drvdata(pdev->dev.parent);
	struct rk8xx_platform_data *pdata = pdev->dev.platform_data;
	int fall_irq, rise_irq;

	if (!strcmp(pdata->chip_name, "rk816")) {
		rk8xx_pwrkey = &rk816_pwrkey;
	} else if (!strcmp(pdata->chip_name, "rk805")) {
		rk8xx_pwrkey = &rk805_pwrkey;
	} else {
		dev_err(&pdev->dev, "failed to match device data\n");
		return -EINVAL;
	}

	fall_irq = regmap_irq_get_virq(rk816->irq_data,
				       rk8xx_pwrkey->pwr_fall_irq);
	rise_irq = regmap_irq_get_virq(rk816->irq_data,
				       rk8xx_pwrkey->pwr_rise_irq);
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
	pwrkey->workqueue = create_singlethread_workqueue("rk816_pwrkey");
	INIT_WORK(&pwrkey->fall_work, rk816_pwrkey_fall_work);
	INIT_WORK(&pwrkey->rise_work, rk816_pwrkey_rise_work);

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

	pr_info("%s register rk8xx_pwrkey driver\n", pdata->chip_name);

	return 0;
}

static struct platform_driver rk816_pwrkey_driver = {
	.probe = rk816_pwrkey_probe,
	.driver		= {
		.name	= "rk8xx-pwrkey",
		.owner	= THIS_MODULE,
	},
};

static int __init rk816_pwrkey_init(void)
{
	return platform_driver_register(&rk816_pwrkey_driver);
}

static void rk816_pwrkey_exit(void)
{
	platform_driver_unregister(&rk816_pwrkey_driver);
}

module_init(rk816_pwrkey_init);
module_exit(rk816_pwrkey_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rk816_pwrkey");
MODULE_DESCRIPTION("RK816 Power Button");
MODULE_AUTHOR("zhangqing <zhangqing@rock-chips.com>");
