/*
 *  coreboot_table.c: module providing coreboot table access.
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

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

struct coreboot_table_header {
	char signature[4];
	u32 header_bytes;
	u32 header_checksum;
	u32 table_bytes;
	u32 table_checksum;
	u32 table_entries;
};

struct coreboot_table_entry {
	u32 tag;
	u32 size;
};

static struct coreboot_table_header __iomem *ptr_header =
	(void*)(-EPROBE_DEFER);

/*
 * This function parses the coreboot table for an entry that contains the base
 * address of the given entry tag. The coreboot table consists of a header
 * directly followed by a number of small, variable-sized entries, which each
 * contain an identifying tag and their length as the first two fields.
 */
int coreboot_table_find(int tag, void *data, size_t data_size)
{
	struct coreboot_table_header header;
	struct coreboot_table_entry entry;
	void *ptr_entry;
	int i;

	if (IS_ERR(ptr_header))
		return PTR_ERR(ptr_header);

	memcpy_fromio(&header, ptr_header, sizeof(header));

	if (strncmp(header.signature, "LBIO", sizeof(header.signature))) {
		pr_warn("coreboot_table: coreboot table missing or corrupt!\n");
		return -ENODEV;
	}

	ptr_entry = (void *)ptr_header + header.header_bytes;

	for (i = 0; i < header.table_entries; i++) {
		memcpy_fromio(&entry, ptr_entry, sizeof(entry));
		if (entry.tag == tag) {
			if (data_size < entry.size)
				return -EINVAL;
			memcpy_fromio(data, ptr_entry, entry.size);
			return 0;
		}
		ptr_entry += entry.size;
	}

	return 0;
}
EXPORT_SYMBOL(coreboot_table_find);

static int coreboot_table_of_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node* fw_dn;

	fw_dn = of_find_compatible_node(NULL, NULL, "coreboot");
	if (!fw_dn) {
		ret = -ENODEV;
		goto fail;
	}

	ptr_header = of_iomap(fw_dn, 0);
	of_node_put(fw_dn);

	if (!ptr_header) {
		ret = -ENOMEM;
		goto fail;
	}

	return 0;
fail:
	ptr_header = ERR_PTR(ret);
	return ret;
}

static int coreboot_table_of_remove(struct platform_device *pdev)
{
	if (!IS_ERR(ptr_header))
		iounmap(ptr_header);
	return 0;
}

static struct platform_driver coreboot_table_driver = {
	.probe = coreboot_table_of_probe,
	.remove = coreboot_table_of_remove,
	.driver = {
		.name = "coreboot_table",
	},
};

static int __init platform_coreboot_table_init(void)
{
	struct platform_device* pdev;
	int ret = 0;

	pdev = platform_device_register_simple("coreboot_table", -1, NULL, 0);
	if (pdev == NULL) {
		ret = -ENODEV;
		goto fail;
	}

	ret = platform_driver_register(&coreboot_table_driver);
	if (ret)
		goto fail;

	return 0;
fail:
	ptr_header = ERR_PTR(ret);
	return ret;
}

module_init(platform_coreboot_table_init);

MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL");
