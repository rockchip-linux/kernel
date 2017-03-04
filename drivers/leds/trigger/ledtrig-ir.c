/*
 * IR LED Trigger
 *
 * Copyright 2016 Rockchip electronic L.T.D
 *
 * Based on ledtrig-timer.c by Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include "../leds.h"

#define KEYMON_NAME	"ir"
#define MATCH_PHYS	"gpio-keys/remotectl"

struct ir_trig_notifier {
	struct notifier_block notifier;
};

struct ir_trig_data {
	struct led_classdev *led;
	struct list_head list;
};

static int bl_blank;
static struct ir_trig_notifier itn;
static DEFINE_SPINLOCK(list_lock);
static LIST_HEAD(pending_list);

static int fb_notifier_callback(struct notifier_block *p,
				unsigned long action, void *data)
{
	struct fb_event *event = data;

	if (action == FB_EARLY_EVENT_BLANK) {
		switch (*((int *)event->data)) {
		case FB_BLANK_UNBLANK:
			break;
		default:
			bl_blank = 1;
			break;
		}
	} else if (action == FB_EVENT_BLANK) {
		switch (*((int *)event->data)) {
		case FB_BLANK_UNBLANK:
			bl_blank = 0;
			break;
		default:
			break;
		}
	}
	return NOTIFY_OK;
}

static bool keymon_match(struct input_handler *handler, struct input_dev *dev)
{
	if (!dev->phys)
		return 0;

	if (!strcmp(MATCH_PHYS, dev->phys))
		return 1;

	return 0;
}

static void keymon_event(struct input_handle *handle, unsigned int type,
			 unsigned int code, int value)
{
	unsigned long flags;
	struct ir_trig_data *itd;

	if (type != EV_KEY)
		return;

	if (code >= KEY_MAX)
		return;

	if (list_empty(&pending_list))
		return;

	spin_lock_irqsave(&list_lock, flags);
	list_for_each_entry(itd, &pending_list, list)
	if (code == KEY_POWER)
		led_set_brightness(itd->led, value);
	else
		led_set_brightness(itd->led, bl_blank ? 0 : value);
	spin_unlock_irqrestore(&list_lock, flags);
}

static int keymon_connect(struct input_handler *handler,
			  struct input_dev *dev,
			  const struct input_device_id *id)
{
	int ret;
	struct input_handle *handle;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = KEYMON_NAME;

	ret = input_register_handle(handle);
	if (ret)
		goto err_input_register_handle;

	ret = input_open_device(handle);
	if (ret)
		goto err_input_open_device;

	return 0;

err_input_open_device:
	input_unregister_handle(handle);
err_input_register_handle:
	kfree(handle);
	return ret;
}

static void keymon_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id keymon_ids[] = {
	{
	 .flags = INPUT_DEVICE_ID_MATCH_EVBIT,
	 .evbit = {BIT_MASK(EV_KEY)},
	 },
	{},
};
MODULE_DEVICE_TABLE(input, keymon_ids);

static struct input_handler keymon_handler = {
	.match = keymon_match,
	.event = keymon_event,
	.connect = keymon_connect,
	.disconnect = keymon_disconnect,
	.name = KEYMON_NAME,
	.id_table = keymon_ids,
};

static void ir_trig_activate(struct led_classdev *led_cdev)
{
	unsigned long flags;
	struct ir_trig_data *ir_data;

	ir_data = kzalloc(sizeof(*ir_data), GFP_KERNEL);
	if (!ir_data)
		return;

	led_cdev->trigger_data = ir_data;

	led_cdev->activated = true;

	ir_data->led = led_cdev;
	spin_lock_irqsave(&list_lock, flags);
	list_add_tail(&ir_data->list, &pending_list);
	spin_unlock_irqrestore(&list_lock, flags);
}

static void ir_trig_deactivate(struct led_classdev *led_cdev)
{
	unsigned long flags;
	struct ir_trig_data *ir_data = led_cdev->trigger_data;

	spin_lock_irqsave(&list_lock, flags);
	list_del(&ir_data->list);
	spin_unlock_irqrestore(&list_lock, flags);

	if (led_cdev->activated) {
		kfree(ir_data);
		led_cdev->activated = false;
	}

	/* Stop blinking */
	led_set_brightness(led_cdev, LED_OFF);
}

static struct led_trigger ir_led_trigger = {
	.name = "ir",
	.activate = ir_trig_activate,
	.deactivate = ir_trig_deactivate,
};

static int __init ir_trig_init(void)
{
	int ret;

	itn.notifier.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&itn.notifier);
	if (ret)
		pr_err("ir_trig: unable to register backlight trigger\n");

	ret = input_register_handler(&keymon_handler);
	if (ret)
		return ret;
	return led_trigger_register(&ir_led_trigger);
}

static void __exit ir_trig_exit(void)
{
	fb_unregister_client(&itn.notifier);
	led_trigger_unregister(&ir_led_trigger);
	input_unregister_handler(&keymon_handler);
}

module_init(ir_trig_init);
module_exit(ir_trig_exit);

MODULE_LICENSE("GPL");
