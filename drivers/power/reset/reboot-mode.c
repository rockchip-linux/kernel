/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/reboot.h>
#include <linux/sysfs.h>
#include "reboot-mode.h"

#define PREFIX "mode-"

struct mode_info {
	char mode[32];
	unsigned int magic;
	struct list_head list;
};

struct reboot_mode_driver {
	struct list_head head;
	int (*write)(int magic);
	int (*read)(void);
	struct notifier_block reboot_notifier;
	struct notifier_block panic_notifier;
};

static char *boot_mode = "coldboot";

static ssize_t boot_mode_show(struct kobject *kobj, struct kobj_attribute *attr,
			      char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n", boot_mode);
}

static struct kobj_attribute kobj_boot_mode = __ATTR_RO(boot_mode);

static int get_reboot_mode_magic(struct reboot_mode_driver *reboot,
				 const char *cmd)
{
	const char *normal = "normal";
	int magic = 0;
	struct mode_info *info;

	if (!cmd || !cmd[0])
		cmd = normal;

	list_for_each_entry(info, &reboot->head, list) {
		if (!strcmp(info->mode, cmd)) {
			magic = info->magic;
			break;
		}
	}

	return magic;
}

static void reboot_mode_write(struct reboot_mode_driver *reboot,
			      const void *cmd)
{
	int magic;

	magic = get_reboot_mode_magic(reboot, cmd);
	if (!magic)
		magic = get_reboot_mode_magic(reboot, NULL);
	if (magic)
		reboot->write(magic);
}

static int reboot_mode_notify(struct notifier_block *this,
			      unsigned long mode, void *cmd)
{
	struct reboot_mode_driver *reboot;

	reboot = container_of(this, struct reboot_mode_driver, reboot_notifier);
	reboot_mode_write(reboot, cmd);

	return NOTIFY_DONE;
}

static int reboot_mode_panic_notify(struct notifier_block *this,
				      unsigned long ev, void *ptr)
{
	struct reboot_mode_driver *reboot;
	const char *cmd = "panic";

	reboot = container_of(this, struct reboot_mode_driver, panic_notifier);
	reboot_mode_write(reboot, cmd);

	return NOTIFY_DONE;
}

static int boot_mode_parse(struct reboot_mode_driver *reboot)
{
	struct mode_info *info;
	unsigned int magic = reboot->read();

	list_for_each_entry(info, &reboot->head, list) {
		if (info->magic == magic) {
			boot_mode = info->mode;
			break;
		}
	}

	pr_info("Boot mode: %s\n", boot_mode);

	return 0;
}

int reboot_mode_register(struct device *dev, int (*write)(int),
			 int (*read)(void))
{
	struct reboot_mode_driver *reboot;
	struct mode_info *info;
	struct property *prop;
	size_t len = strlen(PREFIX);
	int ret;

	reboot = devm_kzalloc(dev, sizeof(*reboot), GFP_KERNEL);
	if (!reboot)
		return -ENOMEM;

	reboot->write = write;
	reboot->read = read;
	INIT_LIST_HEAD(&reboot->head);
	for_each_property_of_node(dev->of_node, prop) {
		if (len > strlen(prop->name) || strncmp(prop->name, PREFIX, len))
			continue;
		info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
		if (!info)
			return -ENOMEM;
		strcpy(info->mode, prop->name + len);
		if (of_property_read_u32(dev->of_node, prop->name, &info->magic)) {
			dev_err(dev, "reboot mode %s without magic number\n",
				info->mode);
			devm_kfree(dev, info);
			continue;
		}
		list_add_tail(&info->list, &reboot->head);
	}
	boot_mode_parse(reboot);
	reboot->reboot_notifier.notifier_call = reboot_mode_notify;
	reboot->panic_notifier.notifier_call = reboot_mode_panic_notify;
	ret = register_reboot_notifier(&reboot->reboot_notifier);
	ret += atomic_notifier_chain_register(&panic_notifier_list,
				       &reboot->panic_notifier);
	ret += sysfs_create_file(kernel_kobj, &kobj_boot_mode.attr);

	return ret;
}
EXPORT_SYMBOL_GPL(reboot_mode_register);

MODULE_AUTHOR("Andy Yan <andy.yan@rock-chips.com");
MODULE_DESCRIPTION("System reboot mode driver");
MODULE_LICENSE("GPL v2");
