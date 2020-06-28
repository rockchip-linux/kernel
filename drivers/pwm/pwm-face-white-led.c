/*
 * Driver for pwm demo on Firefly board.
 *
 * Copyright (C) 2016, Zhongshan T-chip Intelligent Technology Co.,ltd.
 * Copyright 2006  Tek.Leung
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <dt-bindings/pwm/pwm.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/init.h>

#define SYS_DEV_CONFIG 1

static int brightness_max = 0;
static int brightness_value[16]={0};
static int gval = 0;

struct face_white_led_data {
	int     pwm_id;
	struct pwm_device       *pwm;
	unsigned int            period;
	unsigned int pwm_period_ns;
	unsigned int duty_ns;
	bool	enabled;
};
struct face_white_led_data white_led_data;

#ifdef SYS_DEV_CONFIG
static ssize_t face_white_led_store(struct device *dev, \
		struct device_attribute *attr, const char *buf,size_t count)
{
	unsigned long state;
	int c,ret;
	ret = kstrtoul(buf, 10, &state);
	if (state >= brightness_max)
		state = brightness_max - 1;
	else if (state < 0)
		state = 0;
	c = state;
	if (ret)
		return ret;

	white_led_data.enabled = true;
	pwm_config(white_led_data.pwm, brightness_value[c], white_led_data.pwm_period_ns);
	pwm_enable(white_led_data.pwm);
	gval = brightness_value[c];

	return count;
}
static ssize_t face_white_led_show(struct device *dev, \
		struct device_attribute *attr, char *buf)
{
	int i=0;
	for(i=0; i<brightness_max; i++)
	{
		if(gval == brightness_value[i])
		{
			break;
		}
	}

	return sprintf(buf, "%d\n", i);
}

static struct kobject *pwm_kobj;
static DEVICE_ATTR(face_white_led, (S_IWUSR|S_IRUSR|S_IWGRP|S_IRGRP), face_white_led_show, face_white_led_store);
#endif

static int face_white_led_status_update(struct face_white_led_data *pdata)
{
	if (pdata->enabled)
		return 0;

	pwm_set_polarity(pdata->pwm, PWM_POLARITY_NORMAL);
	pwm_enable(pdata->pwm);
	pwm_config(pdata->pwm, pdata->duty_ns, white_led_data.pwm_period_ns);
	pdata->enabled = true;
	gval = pdata->duty_ns;
	return 0;
}



ssize_t face_white_led_parse_dt(struct face_white_led_data *firefly_pdata, struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const __be32 *duty_ns;
	struct property *prop;
	int  len;
	//int i=0;
	int ret;

	duty_ns = of_get_property(np, "default-brightness-level", &len);
	if (duty_ns)
		firefly_pdata->duty_ns = be32_to_cpu(*duty_ns);


	prop = of_find_property(np, "brightness-levels", &len);
	if (!prop)
		return -EINVAL;

	printk("zjy face_white_led_parse_dt %d \r\n", len);
	brightness_max = len / sizeof(u32);

	ret = of_property_read_u32_array(np, "brightness-levels",
						 brightness_value,
						 brightness_max);
	if (!prop)
		return -EINVAL;

	return 0;
}

//extern bool firefly_hwversion_in_range(const struct device_node *device);

static int face_white_led_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct face_white_led_data *firefly_pdata = pdev->dev.platform_data;
	int ret;
	struct pwm_args pargs;

	if (!np) {
		dev_err(&pdev->dev, "Device Tree node missing\n");
		return -EINVAL;
	}

	//if (!firefly_hwversion_in_range(np))
	//	return -EINVAL;

	firefly_pdata = devm_kzalloc(&pdev->dev, sizeof(*firefly_pdata), GFP_KERNEL);
	if (!firefly_pdata)
		return -ENOMEM;

	printk("face_white_led_probe \r\n");
	firefly_pdata->enabled = false;
	firefly_pdata->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(firefly_pdata->pwm)) {
		dev_err(&pdev->dev, "unable to request legacy PWM\n");
		ret = PTR_ERR(firefly_pdata->pwm);
		goto err;
	}

	if (np)
		ret = face_white_led_parse_dt(firefly_pdata, pdev);

	pwm_get_args(firefly_pdata->pwm, &pargs);

	firefly_pdata->pwm_period_ns = pargs.period;

	if (firefly_pdata->pwm_period_ns > 0)
		pwm_set_period(firefly_pdata->pwm, firefly_pdata->pwm_period_ns);
	firefly_pdata->period = pwm_get_period(firefly_pdata->pwm);

	white_led_data = *firefly_pdata;
	face_white_led_status_update(firefly_pdata);

#ifdef SYS_DEV_CONFIG
	pwm_kobj = kobject_create_and_add("pwm", NULL);
	if (pwm_kobj == NULL) {
		printk("create kobject fail \n");
		ret = -ENOMEM;
		goto err;
	}

	ret = sysfs_create_file(pwm_kobj, &dev_attr_face_white_led.attr);
	if (ret) {
	printk("pwm firefly_sysfs_init: sysfs_create_group failed\n");
	goto err;
	}
#endif
	return 0;
err:
#ifdef SYS_DEV_CONFIG
	kobject_del(pwm_kobj);
#endif
	devm_kfree(&pdev->dev,firefly_pdata);
	return ret;
}

static int face_white_led_remove(struct platform_device *pdev)
{
	struct face_white_led_data *firefly_pdata = pdev->dev.platform_data;
#ifdef SYS_DEV_CONFIG
	kobject_del(pwm_kobj);
#endif
	devm_kfree(&pdev->dev,firefly_pdata);
	return 0;
}

static const struct of_device_id face_white_led_dt_ids[] = {
        { .compatible = "firefly,rk3399-face-white-led"},
        {  }
};
MODULE_DEVICE_TABLE(of, face_white_led_dt_ids);

static struct platform_driver face_white_led_driver = {
	.driver = {
		.name = "face-white-led",
		.of_match_table = face_white_led_dt_ids,
	},
	.probe = face_white_led_probe,
	.remove = face_white_led_remove,
};
module_platform_driver(face_white_led_driver);
MODULE_AUTHOR("lkd <service@t-firefly.com>");
MODULE_DESCRIPTION("face-white-led driver");
MODULE_ALIAS("platform:face-white-led");
MODULE_LICENSE("GPL");
