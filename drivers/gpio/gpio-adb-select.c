/*
 * Driver for pwm demo on Firefly board.
 *
 * Copyright (C) 2016, Zhongshan T-chip Intelligent Technology Co.,ltd.
 * Copyright 2006  JC.Lin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

int adb_sel_gpio;
static int adb_sel_gpio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

    adb_sel_gpio = of_get_named_gpio(np, "adb-sel-gpio", 0);
    if (gpio_is_valid(adb_sel_gpio)) { 
        ret = devm_gpio_request(&pdev->dev, adb_sel_gpio, "adb_sel_gpio");
        if(ret < 0){
            dev_err(&pdev->dev, "devm_gpio_request pmic-stby-gpio request ERROR\n");
            goto err_irq;
        }
        printk("adb_sel_gpio(%d)=low\n", adb_sel_gpio);
        gpio_direction_output(adb_sel_gpio, 1);
    } else {
        printk("lsd,Can not read property adb_sel_gpio\n");
    }

    return 0;

err_irq:
	return ret;
}

static int adb_sel_gpio_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id adb_sel_match_table[] = {
	{ .compatible = "firefly,adb-select",},
	{},
};

static struct platform_driver adb_sel_driver = {
	.driver = {
		.name = "adb-sel",
		.owner = THIS_MODULE,
		.of_match_table = adb_sel_match_table,
	},
	.probe = adb_sel_gpio_probe,
	.remove = adb_sel_gpio_remove,
};

static int adb_sel_init(void)
{
	return platform_driver_register(&adb_sel_driver);
}
module_init(adb_sel_init);

static void adb_sel_exit(void)
{
	platform_driver_unregister(&adb_sel_driver);
}
module_exit(adb_sel_exit);

MODULE_AUTHOR("lisd <service@t-firefly.com>");
MODULE_DESCRIPTION("adb select gpio driver");
MODULE_LICENSE("GPL");
