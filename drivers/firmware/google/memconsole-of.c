/*
 * memconsole-of.c
 *
 * Open Firmware / device tree specific parts of the memory based BIOS console.
 *
 * Copyright 2014 Google Inc.
 */

#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include "memconsole.h"

#define CB_TAG_CBMEM_CONSOLE	0x17

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
	u64 physaddr;
};

/*
 * This function parses the coreboot table for an entry that contains the base
 * address of the firmware console. The coreboot table consists of a header
 * directly followed by a number of small, variable-sized entries, which each
 * contain an identifying tag and their length as the first two fields.
 */
static bool memconsole_find(struct platform_device *pdev)
{
	bool ret = false;
	struct coreboot_table_header __iomem header;
	struct coreboot_table_header *ptr_header;
	struct coreboot_table_entry __iomem entry;
	struct coreboot_table_entry *ptr_entry;
	int i;
	struct device_node *coreboot_node = pdev->dev.of_node;

	if (!coreboot_node)
		return false;

	ptr_header = of_iomap(coreboot_node, 0);
	if (!ptr_header)
		goto out_node;

	memcpy_fromio(&header, ptr_header, sizeof(header));

	if (strncmp(header.signature, "LBIO", sizeof(header.signature))) {

		pr_warn("memconsole: coreboot table missing or corrupt!\n");
		goto out_unmap;
	}

	ptr_entry = (void *)ptr_header + header.header_bytes;

	for (i = 0; i < header.table_entries; i++) {
		memcpy_fromio(&entry, ptr_entry, sizeof(entry));

		if (entry.tag == CB_TAG_CBMEM_CONSOLE) {
			ret = memconsole_coreboot_init(entry.physaddr);
			break;
		}

		ptr_entry = (void *)ptr_entry + entry.size;
	}

out_unmap:
	iounmap(ptr_header);
out_node:
	of_node_put(coreboot_node);
	return ret;
}

static int memconsole_of_probe(struct platform_device *pdev)
{
	if (!memconsole_find(pdev))
		return -ENODEV;

	return memconsole_sysfs_init();
}

static int memconsole_of_remove(struct platform_device *pdev)
{
	memconsole_exit();
	return 0;
}

static struct platform_driver memconsole_driver = {
	.probe = memconsole_of_probe,
	.remove = memconsole_of_remove,
	.driver = {
		.name = "memconsole",
	},
};

static int __init platform_memconsole_init(void)
{
	struct device_node *fw_dn;
	struct platform_device *pdev;

	/*
	 * Coreboot systems with OF support might have memconsole.
	 * memconsole_find checks to see if CB_TAG_CBMEM_CONSOLE entry is found.
	 */
	fw_dn = of_find_compatible_node(NULL, NULL, "coreboot");
	if (!fw_dn)
		return -ENODEV;

	pdev = platform_device_register_simple("memconsole", -1, NULL, 0);
	pdev->dev.of_node = fw_dn;

	platform_driver_register(&memconsole_driver);

	return 0;
}

module_init(platform_memconsole_init);
