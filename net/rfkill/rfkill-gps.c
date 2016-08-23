/*
 * Copyright (C) 2016 ROCKCHIP, Inc.
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
 * Rock-chips rfkill driver for gps
 *
 */

#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <dt-bindings/gpio/gpio.h>
#include <uapi/linux/rfkill.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#define LOG(x...)   printk(KERN_INFO "[GPS_RFKILL]: "x)

struct gps_gpio {
	int     io;      //set the address of gpio
	int     enable;  //set the default value,i.e,GPIO_HIGH or GPIO_LOW
};

struct gps_rfkill_gpio_data {
	char                    *name;
	enum rfkill_type        type;
	bool                    gps_power_remain;
	struct gps_gpio         gps_power;
};

struct rfkill_rk_data {
	struct gps_rfkill_gpio_data    *pdata;
	struct platform_device         *pdev;
	struct rfkill                  *rfkill_dev;
};

static int gps_rfkill_gpio_set_power(void *data, bool blocked)
{
	struct rfkill_rk_data *rfkill = data;

	if (blocked) {
		if (gpio_is_valid(rfkill->pdata->gps_power.io))
			gpio_direction_output(rfkill->pdata->gps_power.io,
				!rfkill->pdata->gps_power.enable);
	} else {
		if (gpio_is_valid(rfkill->pdata->gps_power.io))
			gpio_direction_output(rfkill->pdata->gps_power.io,
				rfkill->pdata->gps_power.enable);
	}
	LOG("%s: %s\n", __func__, blocked?"off":"on");

	return 0;
}

static const struct rfkill_ops gps_rfkill_gpio_ops = {
	.set_block = gps_rfkill_gpio_set_power,
};

#ifdef CONFIG_OF
static int gps_platdata_parse_dt(struct device *dev,
                  struct gps_rfkill_gpio_data *data)
{
	struct device_node *node = dev->of_node;
	int gpio;
	enum of_gpio_flags flags;

	if (!node)
		return -ENODEV;

	if (of_find_property(node, "keep_gps_power_on", NULL)) {
		data->gps_power_remain = true;
		LOG("%s: gps power will enabled while kernel starting and keep on.\n", __func__);
	} else {
		data->gps_power_remain = false;
		LOG("%s: enable gps power controler.\n", __func__);
	}

	gpio = of_get_named_gpio_flags(node, "GPS,power_gpio", 0, &flags);
	if (gpio_is_valid(gpio)){
		data->gps_power.io = gpio;
		data->gps_power.enable = (flags == GPIO_ACTIVE_HIGH)? 1:0;
		LOG("%s: get property: GPS,poweren_gpio = %d\n", __func__, gpio);
	} else {
		data->gps_power.io = -1;
        }

	return 0;
}
#endif

static int gps_rfkill_gpio_probe(struct platform_device *pdev)
{
	struct rfkill_rk_data *rfkill;
	struct gps_rfkill_gpio_data *pdata = pdev->dev.platform_data;
	int ret = 0;

	if (!pdata) {
#ifdef CONFIG_OF
		pdata = devm_kzalloc(&pdev->dev, sizeof(struct gps_rfkill_gpio_data), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		ret = gps_platdata_parse_dt(&pdev->dev, pdata);
		if (ret < 0) {
#endif
			LOG("%s: No platform data specified\n", __func__);
			return -EINVAL;
#ifdef CONFIG_OF
		}
#endif
        }

	if (gpio_is_valid(pdata->gps_power.io)) {
		ret = gpio_request(pdata->gps_power.io, "gps_poweren");
		if (ret) {
			LOG("%s: failed to get gps_poweren.\n", __func__);
			goto fail_gpio;
		}
	}

	if (pdata->gps_power_remain) {
		gpio_direction_output(pdata->gps_power.io, pdata->gps_power.enable);
	}

	pdata->name = (char*)"RFKILL_GPS";
	pdata->type = RFKILL_TYPE_GPS;

	rfkill = devm_kzalloc(&pdev->dev, sizeof(*rfkill), GFP_KERNEL);
	if (!rfkill) {
		ret = -ENOMEM;
		goto fail_gpio;
	}

	rfkill->pdata = pdata;
	rfkill->pdev = pdev;

	rfkill->rfkill_dev = rfkill_alloc(pdata->name, &pdev->dev, pdata->type,
					  &gps_rfkill_gpio_ops, rfkill);
	if (!rfkill->rfkill_dev) {
		ret = -ENOMEM;
		goto fail_gpio;
	}

	ret = rfkill_register(rfkill->rfkill_dev);
	if (ret < 0)
		goto fail_rfkill;

	platform_set_drvdata(pdev, rfkill);

	LOG("%s device registered.\n", pdata->name);

	return 0;

fail_rfkill:
	rfkill_destroy(rfkill->rfkill_dev);
fail_gpio:
	if (gpio_is_valid(pdata->gps_power.io))
		gpio_free(pdata->gps_power.io);

	return ret;
}

static int gps_rfkill_gpio_remove(struct platform_device *pdev)
{
	struct rfkill_rk_data *rfkill = platform_get_drvdata(pdev);

	rfkill_unregister(rfkill->rfkill_dev);
	rfkill_destroy(rfkill->rfkill_dev);
	if (gpio_is_valid(rfkill->pdata->gps_power.io))
		gpio_free(rfkill->pdata->gps_power.io);

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id gps_platdata_of_match[] = {
	{ .compatible = "gps-platdata" },
	{ }
};
MODULE_DEVICE_TABLE(of, gps_platdata_of_match);
#endif //CONFIG_OF

static struct platform_driver gps_rfkill_gpio_driver = {
	.probe = gps_rfkill_gpio_probe,
	.remove = gps_rfkill_gpio_remove,
	.driver = {
	.name = "gps_rfkill_gpio",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(gps_platdata_of_match),
	},
};

module_platform_driver(gps_rfkill_gpio_driver);

MODULE_DESCRIPTION("rockchip gps rfkill");
MODULE_AUTHOR("gwl@rock-chips.com");
MODULE_LICENSE("GPL");
