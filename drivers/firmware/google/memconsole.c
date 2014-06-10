/*
 * memconsole.c
 *
 * Architecture-independent parts of the memory based BIOS console.
 *
 * Copyright 2014 Google Inc.
 */

#include <linux/init.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>

#include "memconsole.h"

static char *memconsole_baseaddr;
static size_t memconsole_length;

static ssize_t memconsole_read(struct file *filp, struct kobject *kobp,
			       struct bin_attribute *bin_attr, char *buf,
			       loff_t pos, size_t count)
{
	return memory_read_from_buffer(buf, count, &pos, memconsole_baseaddr,
				       memconsole_length);
}

static struct bin_attribute memconsole_bin_attr = {
	.attr = {.name = "log", .mode = 0444},
	.read = memconsole_read,
};

/* CBMEM firmware console log descriptor. */
struct cbmem_cons {
	u32 buffer_size;
	u32 buffer_cursor;
	u8  buffer_body[0];
} __packed;

static struct cbmem_cons __iomem *cbmem_console;

void memconsole_setup(void *baseaddr, size_t length)
{
	memconsole_baseaddr = baseaddr;
	memconsole_length = length;
}
EXPORT_SYMBOL(memconsole_setup);

bool memconsole_coreboot_init(phys_addr_t physaddr)
{
	struct cbmem_cons __iomem *tmp_cbmc;

	tmp_cbmc = ioremap_cache(physaddr, sizeof(*tmp_cbmc));

	if (tmp_cbmc == NULL)
		return false;

	cbmem_console = ioremap_cache(physaddr, tmp_cbmc->buffer_size +
		sizeof(*cbmem_console));	/* Don't forget counting the header. */

	iounmap(tmp_cbmc);

	if (cbmem_console == NULL)
		return false;

	memconsole_setup(cbmem_console->buffer_body,
		min(cbmem_console->buffer_cursor, cbmem_console->buffer_size));
	return true;
}
EXPORT_SYMBOL(memconsole_coreboot_init);

int memconsole_sysfs_init(void)
{
	memconsole_bin_attr.size = memconsole_length;
	return sysfs_create_bin_file(firmware_kobj,
				     &memconsole_bin_attr);
}
EXPORT_SYMBOL(memconsole_sysfs_init);

void memconsole_exit(void)
{
	if (cbmem_console)
		iounmap(cbmem_console);
	sysfs_remove_bin_file(firmware_kobj, &memconsole_bin_attr);
}
EXPORT_SYMBOL(memconsole_exit);
