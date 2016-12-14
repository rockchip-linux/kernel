/*
 * Copyright (C) 2015 ROCKCHIP, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/reboot.h>
#include <linux/syscalls.h>

static int need_restart;
static struct class *resume_reboot_class;

static ssize_t show_flag(struct device *dev,
			 struct device_attribute *dattr, char *buf)
{
	return sprintf(buf, "%d\n", need_restart);
}

static ssize_t store_flag(struct device *dev, struct device_attribute *dattr,
			  const char *buf, size_t count)
{
	if (!strcmp(buf, "restart"))
		need_restart = 1;

	if (!strcmp(buf, "resume"))
		need_restart = 0;

	return count;
}

DEVICE_ATTR(resume_reboot, 0644, show_flag, store_flag);

static int deprecate_user_ctrl_init(void)
{
	int ret;
	struct device *dev;

	resume_reboot_class = class_create(THIS_MODULE, "resume_reboot");

	if (IS_ERR(resume_reboot_class))
		return PTR_ERR(resume_reboot_class);

	dev = device_create(resume_reboot_class, NULL,
		MKDEV(0, 0), NULL, "resume_reboot");
	if (IS_ERR(dev))
		return PTR_ERR(dev);

	ret = device_create_file(dev, &dev_attr_resume_reboot);

	return ret;
}

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *fb_event = data;
	int *blank = fb_event->data;

	if (need_restart &&
	    event == FB_EVENT_BLANK &&
	    *blank == FB_BLANK_UNBLANK) {
		sys_sync();
		kernel_restart(NULL);
	}

	return NOTIFY_OK;
}

static struct notifier_block fb_notifier = {
	.notifier_call = fb_notifier_callback,
};

static int __init stb_sysctl_init(void)
{
	int ret;

	ret = deprecate_user_ctrl_init();
	if (ret)
		pr_err("%s: deprecate_user_ctrl_init failed\n", __func__);

	ret = fb_register_client(&fb_notifier);

	return ret;
}

module_init(stb_sysctl_init);
