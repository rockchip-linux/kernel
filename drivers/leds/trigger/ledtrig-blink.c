/*
 * blink LED Trigger
 *
 * Copyright (C) 2016 Rockchip Electronics Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more motorails.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include "../leds.h"

struct blink_trig_data {
	unsigned int blink_default;
	unsigned int blink_ms;
};

static ssize_t led_blink_state_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct blink_trig_data *blink_data = led_cdev->trigger_data;
	int blink_ms, ret, blink_default = 0;

	ret = sscanf(buf, "%d,%d", &blink_ms, &blink_default);
	if (ret != 2)
		return -EINVAL;
	blink_data->blink_default = !!blink_default;
	if (blink_data->blink_ms != blink_ms) {
		if (blink_data->blink_ms) { /* stop and restart blink if need */
			led_stop_software_blink(led_cdev);
		}
		if (blink_data->blink_default)
			__led_set_brightness(led_cdev, LED_FULL);
		else
			__led_set_brightness(led_cdev, LED_OFF);
		if (blink_ms) {
			led_cdev->blink_delay_on = blink_ms / 2 + blink_ms % 2;
			led_cdev->blink_delay_off = blink_ms / 2;
			led_blink_set(led_cdev, &led_cdev->blink_delay_on,
				      &led_cdev->blink_delay_off);
		}
		blink_data->blink_ms = blink_ms;
	} else if (!blink_data->blink_ms) {/* no blink, just set state */
		if (blink_data->blink_default)
			__led_set_brightness(led_cdev, LED_FULL);
		else
			__led_set_brightness(led_cdev, LED_OFF);
	}
	return size;
}

static ssize_t led_blink_state_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct blink_trig_data *blink_data = led_cdev->trigger_data;

	return sprintf(buf, "%u\n", blink_data->blink_ms);
}

static DEVICE_ATTR(blink_state, 0644, led_blink_state_show,
		   led_blink_state_store);

static void blink_trig_activate(struct led_classdev *led_cdev)
{
	struct blink_trig_data *blink_data;
	int rc;

	blink_data = kzalloc(sizeof(*blink_data), GFP_KERNEL);
	if (!blink_data)
		return;
	led_cdev->trigger_data = blink_data;
	rc = device_create_file(led_cdev->dev, &dev_attr_blink_state);
	if (rc)
		goto err_out_trig_data;
	led_cdev->activated = true;
	return;
err_out_trig_data:
	kfree(led_cdev->trigger_data);
}

static void blink_trig_deactivate(struct led_classdev *led_cdev)
{
	struct blink_trig_data *blink_data = led_cdev->trigger_data;

	if (led_cdev->activated) {
		device_remove_file(led_cdev->dev, &dev_attr_blink_state);
		kfree(blink_data);
		led_cdev->activated = false;
	}
	/* Stop blinking */
	led_set_brightness(led_cdev, LED_OFF);
}

static struct led_trigger blink_led_trigger = {
	.name     = "blink",
	.activate = blink_trig_activate,
	.deactivate = blink_trig_deactivate,
};

static int __init blink_trig_init(void)
{
	return led_trigger_register(&blink_led_trigger);
}

static void __exit blink_trig_exit(void)
{
	led_trigger_unregister(&blink_led_trigger);
}

module_init(blink_trig_init);
module_exit(blink_trig_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ledtrig-blink");
MODULE_AUTHOR("ROCKCHIP");
