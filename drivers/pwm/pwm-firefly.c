/*
 * Driver for pwm demo on Firefly board.
 *
 * Copyright (C) 2016, Zhongshan T-chip Intelligent Technology Co.,ltd.
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

static int gval = 0;
struct firefly_pwm_data {
        int     pwm_id;
        struct pwm_device       *pwm;
        unsigned int            period;
        unsigned int pwm_period_ns;
        unsigned int max_period;
        unsigned int min_period;
        unsigned int duty_ns;
        bool    enabled;
};
struct firefly_pwm_data g_firefly_pdata;

#ifdef SYS_DEV_CONFIG
static ssize_t firefly_pwm_store(struct device *dev, \
                struct device_attribute *attr, const char *buf,size_t count)
{
        unsigned long state;
        int c,ret;
        ret = kstrtoul(buf, 10, &state);
        if (state > g_firefly_pdata.max_period)
                state = g_firefly_pdata.max_period;
        else if (state < g_firefly_pdata.min_period)
                state = g_firefly_pdata.min_period;
        c = state;
        if (ret)
                return ret;

        g_firefly_pdata.enabled = true;
        pwm_config(g_firefly_pdata.pwm, c, g_firefly_pdata.pwm_period_ns);
        pwm_enable(g_firefly_pdata.pwm);
        gval = c;

        return count;
}
static ssize_t firefly_pwm_show(struct device *dev, \
                struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%u\n", gval);
}

static struct kobject *pwm_kobj;
static DEVICE_ATTR(pwm, (S_IWUSR|S_IRUSR|S_IWGRP|S_IRGRP), firefly_pwm_show, firefly_pwm_store);
#endif

static int firefly_pwm_status_update(struct firefly_pwm_data *pdata)
{
        if (pdata->enabled)
                return 0;

        pwm_enable(pdata->pwm);
        pwm_config(pdata->pwm, pdata->duty_ns, g_firefly_pdata.pwm_period_ns);
        pdata->enabled = true;
        gval = pdata->duty_ns;
        return 0;
}
ssize_t firefly_pwm_parse_dt(struct firefly_pwm_data *firefly_pdata, struct platform_device *pdev)
{
        struct device_node *np = pdev->dev.of_node;
        const __be32 *id, *min_period, *max_period, *duty_ns;
        int  len;

        id = of_get_property(np, "pwm_id", &len);
        if (id)
                firefly_pdata->pwm_id = be32_to_cpu(*id);

        min_period = of_get_property(np, "min_period", &len);
        if (min_period)
                firefly_pdata->min_period = be32_to_cpu(*min_period);

        max_period = of_get_property(np, "max_period", &len);
        if (max_period)
                firefly_pdata->max_period = be32_to_cpu(*max_period);

        firefly_pdata->pwm_period_ns = firefly_pdata->max_period - firefly_pdata->min_period;

        duty_ns = of_get_property(np, "duty_ns", &len);
        if (duty_ns)
                firefly_pdata->duty_ns = be32_to_cpu(*duty_ns);

        return 0;
}

static int firefly_pwm_probe(struct platform_device *pdev)
{
        struct device_node *np = pdev->dev.of_node;
        struct firefly_pwm_data *firefly_pdata = pdev->dev.platform_data;
        int ret;

        if (!np) {
                dev_err(&pdev->dev, "Device Tree node missing\n");
                return -EINVAL;
        }

        firefly_pdata = devm_kzalloc(&pdev->dev, sizeof(*firefly_pdata), GFP_KERNEL);
        if (!firefly_pdata)
                return -ENOMEM;

        if (np)
                ret = firefly_pwm_parse_dt(firefly_pdata, pdev);

        firefly_pdata->enabled = false;

        firefly_pdata->pwm = pwm_request(firefly_pdata->pwm_id, "firefly-pwm");
        if (IS_ERR(firefly_pdata->pwm)) {
                dev_err(&pdev->dev, "unable to request legacy PWM\n");
                ret = PTR_ERR(firefly_pdata->pwm);
                goto err;
        }
        
        if (firefly_pdata->pwm_period_ns > 0)
                pwm_set_period(firefly_pdata->pwm, firefly_pdata->pwm_period_ns);
        firefly_pdata->period = pwm_get_period(firefly_pdata->pwm);

        g_firefly_pdata = *firefly_pdata;
        firefly_pwm_status_update(firefly_pdata);
        printk("%s: Firefly PWM Demo !\n", __func__);

#ifdef SYS_DEV_CONFIG
        pwm_kobj = kobject_create_and_add("pwm", NULL);
        if (pwm_kobj == NULL) {
                printk("create kobject fail \n");
                ret = -ENOMEM;
                goto err;
        }

        ret = sysfs_create_file(pwm_kobj, &dev_attr_pwm.attr);
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
        pwm_free(firefly_pdata->pwm);
        return ret;
}

static int firefly_pwm_remove(struct platform_device *pdev)
{
        struct firefly_pwm_data *firefly_pdata = pdev->dev.platform_data;
#ifdef SYS_DEV_CONFIG
        kobject_del(pwm_kobj);
#endif
        pwm_free(firefly_pdata->pwm);
        return 0;
}

static const struct of_device_id firefly_pwm_dt_ids[] = {
        { .compatible = "firefly,rk3399-pwm"},
        {  }
};
MODULE_DEVICE_TABLE(of, firefly_pwm_dt_ids);

static struct platform_driver firefly_pwm_driver = {
        .driver = {
                .name = "firefly-pwm",
                .of_match_table = firefly_pwm_dt_ids,
        },
        .probe = firefly_pwm_probe,
        .remove = firefly_pwm_remove,
};
module_platform_driver(firefly_pwm_driver);
MODULE_AUTHOR("lkd <service@t-firefly.com>");
MODULE_DESCRIPTION("Firefly PWM driver");
MODULE_ALIAS("platform:firefly-pwm");
MODULE_LICENSE("GPL");
