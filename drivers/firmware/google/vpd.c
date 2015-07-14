/*
 *  vpd.c: Driver for exporting VPD content to sysfs.
 *
 *  Copyright 2015 Google Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License v2.0 as published by
 *  the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include "coreboot_table.h"
#include "vpd_decode.h"

#define CB_TAG_VPD	0x2c
#define VPD_CBMEM_MAGIC 0x43524f53

static struct kobject *vpd_kobj;

struct vpd_cbmem {
	uint32_t magic;
	uint32_t version;
	uint32_t ro_size;
	uint32_t rw_size;
	uint8_t blob[0];
};

struct vpd_section {
	bool enabled;
	const char *name;
	char *raw_name;                 /* the string name_raw */
	struct kobject *kobj;           /* vpd/name directory */
	char *baseaddr;
	struct bin_attribute bin_attr;  /* vpd/name_raw bin_attribute */
	struct list_head attribs;  /* a list of key of type vpd_attrib_info */
};

struct vpd_attrib_info {
	char *key;
	const char *value;
	struct bin_attribute bin_attr;
	struct list_head list;
};

static struct vpd_section ro_vpd;
static struct vpd_section rw_vpd;

static ssize_t vpd_attrib_read(struct file *filp, struct kobject *kobp,
		struct bin_attribute *bin_attr, char *buf,
		loff_t pos, size_t count)
{
	struct vpd_attrib_info *info = bin_attr->private;

	return memory_read_from_buffer(buf, count, &pos,
			info->value, info->bin_attr.size);
}

static int vpd_section_attrib_add(const uint8_t *key, int32_t key_len,
		const uint8_t *value, int32_t value_len,
		void *arg)
{
	int ret;
	struct vpd_section *sec = arg;
	struct vpd_attrib_info *info = kzalloc(sizeof(struct vpd_attrib_info),
			GFP_KERNEL);

	info->key = kzalloc(sizeof(key_len + 1), GFP_KERNEL);
	if (!info->key)
		return -ENOMEM;

	memcpy(info->key, key, key_len);

	sysfs_bin_attr_init(&info->bin_attr);
	info->bin_attr.attr.name = info->key;
	info->bin_attr.attr.mode = 0444;
	info->bin_attr.size = value_len;
	info->bin_attr.read = vpd_attrib_read;
	info->bin_attr.private = info;

	info->value = value;

	INIT_LIST_HEAD(&info->list);
	list_add_tail(&info->list, &sec->attribs);

	ret = sysfs_create_bin_file(sec->kobj, &info->bin_attr);
	if (ret) {
		kfree(info->key);
		return ret;
	}

	return 0;
}

static void vpd_section_attrib_destroy(struct vpd_section *sec)
{
	struct vpd_attrib_info *info;
	struct vpd_attrib_info *temp;

	list_for_each_entry_safe(info, temp, &sec->attribs, list) {
		kfree(info->key);
		sysfs_remove_bin_file(sec->kobj, &info->bin_attr);
		kfree(info);
	}
}

static ssize_t vpd_section_read(struct file *filp, struct kobject *kobp,
		struct bin_attribute *bin_attr, char *buf,
		loff_t pos, size_t count)
{
	struct vpd_section *sec = bin_attr->private;

	return memory_read_from_buffer(buf, count, &pos,
			sec->baseaddr, sec->bin_attr.size);
}


static int vpd_section_create_attribs(struct vpd_section *sec)
{
	int32_t consumed;
	int ret;

	consumed = 0;
	do {
		ret = decode_vpd_string(sec->bin_attr.size, sec->baseaddr,
				&consumed, vpd_section_attrib_add, sec);
	} while (ret == VPD_OK);
	return 0;
}

static int vpd_section_init(const char *name, struct vpd_section *sec,
		phys_addr_t physaddr, size_t size)
{
	int ret;
	int raw_len;

	sec->baseaddr = ioremap_cache(physaddr, size);
	if (sec->baseaddr == NULL)
		return -ENOMEM;

	sec->name = name;

	/* We want to export the raw partion with name ${name}_raw */
	raw_len = strlen(name) + 5;
	sec->raw_name = kzalloc(raw_len, GFP_KERNEL);
	strncpy(sec->raw_name, name, raw_len);
	strncat(sec->raw_name, "_raw", raw_len);

	sysfs_bin_attr_init(&sec->bin_attr);
	sec->bin_attr.attr.name = sec->raw_name;
	sec->bin_attr.attr.mode = 0444;
	sec->bin_attr.size = size;
	sec->bin_attr.read = vpd_section_read;
	sec->bin_attr.private = sec;

	ret = sysfs_create_bin_file(vpd_kobj, &sec->bin_attr);
	if (ret)
		goto fail;

	sec->kobj = kobject_create_and_add(name, vpd_kobj);
	if (!sec->kobj) {
		ret = -EINVAL;
		goto fail2;
	}

	INIT_LIST_HEAD(&sec->attribs);
	vpd_section_create_attribs(sec);

	sec->enabled = true;

	return 0;

fail2:
	sysfs_remove_bin_file(vpd_kobj, &sec->bin_attr);
fail:
	kfree(sec->raw_name);
	iounmap(sec->baseaddr);
	return ret;
}

static int vpd_section_destroy(struct vpd_section *sec)
{
	if (sec->enabled) {
		vpd_section_attrib_destroy(sec);
		kobject_del(sec->kobj);
		sysfs_remove_bin_file(vpd_kobj, &sec->bin_attr);
		kfree(sec->raw_name);
		iounmap(sec->baseaddr);
	}
	return 0;
}

static int init_vpd_sections(phys_addr_t physaddr)
{
	struct vpd_cbmem __iomem *temp;
	struct vpd_cbmem header;
	int ret = 0;

	temp = ioremap_cache(physaddr, sizeof(struct vpd_cbmem));
	if (temp == NULL) {
		return -ENOMEM;
	}

	memcpy_fromio(&header, temp, sizeof(struct vpd_cbmem));
	iounmap(temp);

	if (header.magic != VPD_CBMEM_MAGIC)
		return -ENODEV;

	if (header.ro_size) {
		ret = vpd_section_init("ro", &ro_vpd,
				physaddr + sizeof(struct vpd_cbmem),
				header.ro_size);
		if (ret)
			return ret;
	}

	if (header.rw_size) {
		ret = vpd_section_init("rw", &rw_vpd,
				physaddr + sizeof(struct vpd_cbmem) +
				header.ro_size, header.rw_size);
		if (ret)
			return ret;
	}
	return 0;
}

static int vpd_probe(struct platform_device *pdev)
{
	int ret;
	struct lb_cbmem_ref entry;

	ret = coreboot_table_find(CB_TAG_VPD, &entry, sizeof(entry));
	if (ret)
		return ret;

	return init_vpd_sections(entry.cbmem_addr);
}

static struct platform_driver vpd_driver = {
	.probe = vpd_probe,
	.driver = {
		.name = "vpd",
	},
};

static int __init platform_vpd_init(void)
{
	struct platform_device *pdev;

	pdev = platform_device_register_simple("vpd", -1, NULL, 0);
	if (pdev == NULL)
		return -ENODEV;

	vpd_kobj = kobject_create_and_add("vpd", firmware_kobj);
	if (!vpd_kobj)
		return -ENOMEM;

	memset(&ro_vpd, 0, sizeof(ro_vpd));
	memset(&rw_vpd, 0, sizeof(rw_vpd));

	platform_driver_register(&vpd_driver);

	return 0;
}

static void __exit platform_vpd_exit(void)
{
	vpd_section_destroy(&ro_vpd);
	vpd_section_destroy(&rw_vpd);
	kobject_del(vpd_kobj);
}

module_init(platform_vpd_init);
module_exit(platform_vpd_exit);

MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL");
