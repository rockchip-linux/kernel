/* drivers/misc/rk_modem_control.c
 *
 * Copyright (C) 2012-2016 ROCKCHIP.
 * Author: jerry <jerry.zhang@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/types.h>
#include <linux/kobject.h>
#include <linux/delay.h>

static int power_en_gpio;
static int power_on_gpio;
static int reset_gpio;
static struct kobject *modem_kobj;

static ssize_t power_en_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int var;

	var = gpio_get_value(power_en_gpio);

	return sprintf(buf, "%d\n", var);
}

static ssize_t power_en_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int var;
	int ret;

	ret = sscanf(buf, "%du", &var);
	if (ret < 0)
		return ret;

	if (var == 1)
		gpio_set_value(power_en_gpio, 1);
	else if (var == 0)
		gpio_set_value(power_en_gpio, 0);

	return count;
}

static struct kobj_attribute power_en_attribute = __ATTR_RW(power_en);

static ssize_t reset_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int var;

	var = gpio_get_value(reset_gpio);

	return sprintf(buf, "%d\n", var);
}

static ssize_t reset_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int var;
	int ret;

	ret = sscanf(buf, "%du", &var);
	if (ret < 0)
			return ret;

	if (var == 1)
		gpio_set_value(reset_gpio, 1);
	else if (var == 0)
		gpio_set_value(reset_gpio, 0);

	return count;
}

static struct kobj_attribute reset_attribute = __ATTR_RW(reset);

static ssize_t power_on_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int var;

	var = gpio_get_value(power_on_gpio);

	return sprintf(buf, "%d\n", var);
}

static ssize_t power_on_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int var;
	int ret;

	ret = sscanf(buf, "%du", &var);
	if (ret < 0)
		return ret;

	if (var == 1)
		gpio_set_value(power_on_gpio, 1);
	else if (var == 0)
		gpio_set_value(power_on_gpio, 0);

	return count;
}

static struct kobj_attribute power_on_attribute = __ATTR_RW(power_on);

static struct attribute *modem_sysfs[] = {
	&power_en_attribute.attr,
	&reset_attribute.attr,
	&power_on_attribute.attr,
	NULL,
};

static struct attribute_group modem_attr_group = {
	.attrs = modem_sysfs,
};

static int modem_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;

	power_en_gpio = of_get_named_gpio_flags(np, "power-en_gpio", 0, NULL);
	if (!gpio_is_valid(power_en_gpio)) {
		dev_err(&pdev->dev, "MODEM POWER: invalid power_en_gpio: %d\n", power_en_gpio);
		goto err_to_dev_reg;
	}

	ret = devm_gpio_request(&pdev->dev, power_en_gpio, "modem_power_en_gpio");
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request gpio %d with ret:%d\n", power_en_gpio, ret);
		goto err_to_dev_reg;
	}

	power_on_gpio = of_get_named_gpio_flags(np, "power-on_gpio", 0, NULL);
	if (!gpio_is_valid(power_on_gpio)) {
		dev_err(&pdev->dev, "MODEM POWER: invalid power_on_gpio: %d\n", power_on_gpio);
		goto err_to_dev_reg;
	}

	ret = devm_gpio_request(&pdev->dev, power_on_gpio, "modem_power_on_gpio");
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request gpio %d with ret:%d\n", power_on_gpio, ret);
		goto err_to_dev_reg;
	}

	reset_gpio = of_get_named_gpio_flags(np, "reset_gpio", 0, NULL);
	if (!gpio_is_valid(reset_gpio)) {
		dev_err(&pdev->dev, "MODEM POWER: invalid reset_gpio: %d\n", reset_gpio);
		goto err_to_dev_reg;
	}

	ret = devm_gpio_request(&pdev->dev, reset_gpio, "modem_reset_gpio");
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request gpio %d with ret:%d\n", reset_gpio, ret);
		goto err_to_dev_reg;
	}

	modem_kobj = kobject_create_and_add("modem", NULL);
	if (modem_kobj == NULL)
		goto err_to_dev_reg;

	ret = sysfs_create_group(modem_kobj, &modem_attr_group);
	if (ret)
		goto err_to_dev_reg;

	/* power-on pin default set to output and low state */
	gpio_direction_output(power_on_gpio, 1);
	gpio_set_value(power_on_gpio, 0);
	mdelay(5);

	/* power-en pin default set to output and high state */
	gpio_direction_output(power_en_gpio, 1);
	gpio_set_value(power_en_gpio, 1);
	mdelay(5);

	/* reset pin default set to in and low state */
	gpio_direction_output(reset_gpio, 1);
	gpio_set_value(reset_gpio, 0);
	mdelay(5);

	return 0;

err_to_dev_reg:

	kobject_put(modem_kobj);
	sysfs_remove_group(modem_kobj, &modem_attr_group);

	return ret;
}

static int modem_remove(struct platform_device *pdev)
{
	kobject_put(modem_kobj);
	sysfs_remove_group(modem_kobj, &modem_attr_group);

	return 0;
}

static const struct of_device_id modem_of_match[] = {
	{ .compatible = "rockchip,modem" },
	{ }
};

static struct platform_driver modem_driver = {
	.probe = modem_probe,
	.remove = modem_remove,
	.driver = {
	.name           = "modem",
	.owner          = THIS_MODULE,
		.of_match_table = of_match_ptr(modem_of_match),
	},
};

module_platform_driver(modem_driver);

MODULE_AUTHOR("jerry.zhang@rock-chips.com");
MODULE_DESCRIPTION("RK modem power control driver");
MODULE_LICENSE("GPL");
