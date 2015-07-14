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
#include "coreboot_table.h"

#define CB_TAG_CBMEM_CONSOLE	0x17

static int memconsole_of_probe(struct platform_device *pdev)
{
	int ret;
	struct lb_cbmem_ref entry;

	ret = coreboot_table_find(CB_TAG_CBMEM_CONSOLE, &entry, sizeof(entry));
	if (ret)
		return ret;

	ret = memconsole_coreboot_init(entry.cbmem_addr);
	if (ret)
		return ret;

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
	struct platform_device *pdev;

	pdev = platform_device_register_simple("memconsole", -1, NULL, 0);
	if (pdev == NULL)
		return -ENODEV;

	platform_driver_register(&memconsole_driver);

	return 0;
}

module_init(platform_memconsole_init);
