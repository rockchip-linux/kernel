/*
 * LED Kernel Multi-Control Trigger
 *
 * Control multi leds at one time using ioctl from userspace.
 *
 * Copyright 2017 Allen Zhang <zwp@rock-chips.com>
 *
 * Based on Richard Purdie's ledtrig-timer.c.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/leds.h>
#include "../leds.h"
#include "../leds-multi.h"

struct multi_ctrl_data {
	struct led_ctrl_data *data;
	struct led_classdev *led_cdev;
	struct delayed_work delay_trig_work;
	struct list_head node;
	struct led_ctrl_data old_data;
};

DECLARE_RWSEM(multi_leds_list_lock);
LIST_HEAD(multi_leds_list);

static int led_num;
static struct miscdevice multi_ctrl_miscdev;
static char *mult_ctrl_trigger[TRIGGER_MAX] = {
	"none",
	"default-on",
	"timer",
	"oneshot",
};

static struct led_ctrl_data leds_data[MAX_LEDS_NUMBER];

static void multi_ctrl_delay_trig_func(struct work_struct *ws)
{
	struct multi_ctrl_data *ctrl_data =
		container_of(ws, struct multi_ctrl_data, delay_trig_work.work);
	struct led_classdev *led_cdev = ctrl_data->led_cdev;
	struct led_ctrl_data *led_data = ctrl_data->data;

	/* set brightness*/
	if (led_data->brightness == LED_OFF) {
		led_trigger_remove(led_cdev);
		return;
	}
	/* set delay_on and delay_off */
	led_cdev->blink_delay_off = led_data->delay_off;
	led_cdev->blink_delay_on = led_data->delay_on;
	led_cdev->brightness = led_data->brightness;

	led_trigger_set_by_name(led_cdev, mult_ctrl_trigger[led_data->trigger]);

	if (led_data->trigger == TRIGGER_ONESHOT)
		led_blink_set_oneshot(led_cdev,
				      &led_cdev->blink_delay_on,
				      &led_cdev->blink_delay_off, 0);
}

static int multi_ctrl_set_led(struct multi_ctrl_data *ctrl_data)
{
	struct led_ctrl_data *led_data = ctrl_data->data;
	struct led_classdev *led_cdev = ctrl_data->led_cdev;

	if (!led_data || led_data->trigger >= TRIGGER_MAX)
		return -EINVAL;

	if (led_data->delayed_trigger_ms &&
	    (led_data->trigger == TRIGGER_TIMER ||
	    led_data->trigger == TRIGGER_ONESHOT)) {
		schedule_delayed_work(&ctrl_data->delay_trig_work,
				      msecs_to_jiffies(led_data->delayed_trigger_ms));
	} else {
		/* set brightness*/
		if (led_data->brightness == LED_OFF ||
		    led_data->trigger == TRIGGER_NONE) {
			led_trigger_remove(led_cdev);
			return 0;
		}
		/* set delay_on and delay_off */
		led_cdev->blink_delay_off = led_data->delay_off;
		led_cdev->blink_delay_on = led_data->delay_on;
		led_cdev->brightness = led_data->brightness;

		led_trigger_set_by_name(led_cdev,
					mult_ctrl_trigger[led_data->trigger]);

		if (led_data->trigger == TRIGGER_ONESHOT)
			led_blink_set_oneshot(led_cdev,
					      &led_cdev->blink_delay_on,
					      &led_cdev->blink_delay_off, 0);
	}

	return 0;
}

static int multi_ctrl_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int multi_ctrl_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long multi_ctrl_ioctl(struct file *file,
			     unsigned int cmd, unsigned long arg)
{
	int i = 0;
	int ret = 0;

	switch (cmd) {
	case LEDS_MULTI_CTRL_IOCTL_MULTI_SET:
	{
		struct led_ctrl_data *argp = (struct led_ctrl_data *)arg;
		struct multi_ctrl_data *ctrl_data;

		down_read(&multi_leds_list_lock);
		if (copy_from_user(leds_data, argp,
				   sizeof(struct led_ctrl_data) * led_num)) {
			pr_err("%s, copy from user failed\n", __func__);
			up_read(&multi_leds_list_lock);
			return -EFAULT;
		}
		list_for_each_entry(ctrl_data, &multi_leds_list, node) {
			struct led_classdev *led_cdev = ctrl_data->led_cdev;

			if (delayed_work_pending(&ctrl_data->delay_trig_work))
				cancel_delayed_work_sync(&ctrl_data->delay_trig_work);

			if (i >= led_num) {
				dev_err(led_cdev->dev, "exceed the max number of muti_leds_list\n");
				break;
			}
			ctrl_data->data = &leds_data[i++];
			if (!memcmp(&ctrl_data->old_data, ctrl_data->data,
				    sizeof(struct led_ctrl_data)))
				continue;
			multi_ctrl_set_led(ctrl_data);
			memcpy(&ctrl_data->old_data, ctrl_data->data,
			       sizeof(struct led_ctrl_data));
		}
		up_read(&multi_leds_list_lock);
		break;
	}
	case LEDS_MULTI_CTRL_IOCTL_GET_LED_NUMBER:
	{
		 int *argp = (int *)arg;
		*argp = led_num;
		break;
	}
	default:
		break;
	}

	return ret;
}

static const struct file_operations multi_ctrl_ops = {
	.owner = THIS_MODULE,
	.open = multi_ctrl_open,
	.release = multi_ctrl_release,
	.unlocked_ioctl = multi_ctrl_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = multi_ctrl_ioctl,
#endif
};

int led_multi_control_register(struct led_classdev *led_cdev)
{
	struct multi_ctrl_data *data;

	if (led_num++ >= MAX_LEDS_NUMBER)
		return -EINVAL;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(led_cdev->dev, "malloc multi_ctrl_data failed\n");
		return -ENOMEM;
	}

	data->led_cdev = led_cdev;
	data->old_data.brightness = led_cdev->brightness;
	data->old_data.delay_off = led_cdev->blink_delay_off;
	data->old_data.delay_on = led_cdev->blink_delay_on;

	down_write(&multi_leds_list_lock);
	list_add_tail(&data->node, &multi_leds_list);
	up_write(&multi_leds_list_lock);

	INIT_DELAYED_WORK(&data->delay_trig_work,
			  multi_ctrl_delay_trig_func);

	return 0;
}

int led_multi_control_unregister(struct led_classdev *cdev)
{
	struct multi_ctrl_data *ctrl_data;

	if (led_num-- < 0)
		return -EINVAL;

	down_write(&multi_leds_list_lock);
	list_for_each_entry(ctrl_data, &multi_leds_list, node) {
		if (ctrl_data->led_cdev == cdev) {
			cancel_delayed_work_sync(&ctrl_data->delay_trig_work);
			list_del(&ctrl_data->node);
			break;
		}
	}
	up_write(&multi_leds_list_lock);
	kfree(ctrl_data);

	return 0;
}

int led_multi_control_init(struct device *dev)
{
	int ret;

	multi_ctrl_miscdev.fops = &multi_ctrl_ops;
	multi_ctrl_miscdev.parent = dev;
	multi_ctrl_miscdev.name = "led_multi_ctrl";
	ret = misc_register(&multi_ctrl_miscdev);
	if (ret < 0) {
		pr_err("multi_control_init - Can't allocate misc dev for led multi-control.\n");
		return ret;
	}
	led_num = 0;

	return 0;
}

int led_multi_control_exit(struct device *dev)
{
	misc_deregister(&multi_ctrl_miscdev);

	return 0;
}

MODULE_AUTHOR("Allen.zhang <zwp@rock-chips.com>");
MODULE_DESCRIPTION("Multi-Contorl LED trigger");
MODULE_LICENSE("GPL");
