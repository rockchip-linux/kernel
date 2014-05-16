/*
 * memconsole.c
 *
 * Infrastructure for importing the BIOS memory based console
 * into the kernel log ringbuffer.
 *
 * Copyright 2010 Google Inc. All rights reserved.
 */

#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/mm.h>
#include <asm/bios_ebda.h>
#include <asm/e820.h>
#include <linux/acpi.h>

#define BIOS_MEMCONSOLE_V1_MAGIC	0xDEADBABE
#define BIOS_MEMCONSOLE_V2_MAGIC	(('M')|('C'<<8)|('O'<<16)|('N'<<24))

struct biosmemcon_ebda {
	u32 signature;
	union {
		struct {
			u8  enabled;
			u32 buffer_addr;
			u16 start;
			u16 end;
			u16 num_chars;
			u8  wrapped;
		} __packed v1;
		struct {
			u32 buffer_addr;
			/* Misdocumented as number of pages! */
			u16 num_bytes;
			u16 start;
			u16 end;
		} __packed v2;
	};
} __packed;

static char *memconsole_baseaddr;
static size_t memconsole_length;
static bool coreboot_system;

static int set_coreboot_system_flag(const struct dmi_system_id *unused)
{
	coreboot_system = true;
	return true;
}

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


static void found_v1_header(struct biosmemcon_ebda *hdr)
{
	printk(KERN_INFO "BIOS console v1 EBDA structure found at %p\n", hdr);
	printk(KERN_INFO "BIOS console buffer at 0x%.8x, "
	       "start = %d, end = %d, num = %d\n",
	       hdr->v1.buffer_addr, hdr->v1.start,
	       hdr->v1.end, hdr->v1.num_chars);

	memconsole_length = hdr->v1.num_chars;
	memconsole_baseaddr = phys_to_virt(hdr->v1.buffer_addr);
}

static void found_v2_header(struct biosmemcon_ebda *hdr)
{
	printk(KERN_INFO "BIOS console v2 EBDA structure found at %p\n", hdr);
	printk(KERN_INFO "BIOS console buffer at 0x%.8x, "
	       "start = %d, end = %d, num_bytes = %d\n",
	       hdr->v2.buffer_addr, hdr->v2.start,
	       hdr->v2.end, hdr->v2.num_bytes);

	memconsole_length = hdr->v2.end - hdr->v2.start;
	memconsole_baseaddr = phys_to_virt(hdr->v2.buffer_addr
					   + hdr->v2.start);
}

/*
 * The function below tries to find the firmware console log saved in the
 * appropriate coreboot CBMEM section, using the structures and constants from
 * coreboot code.
 *
 * Coreboot's CBMEM is a structure allocated by coreboot above usable memory
 * at a 128K aligned address. One of the CBMEM sections could contain the
 * firmare console log.
 */

/* Coreboot CBMEM section descriptor. */
struct cbmem_entry {
	u32 magic;
	u32 id;
	u64 base;
	u64 size;
} __packed;

/* CBMEM firmware console log descriptor. */
struct cbmem_cons {
	u32 buffer_size;
	u32 buffer_cursor;
	u8  buffer_body[0];
} __packed;

struct cbmem_cons __iomem *cbmem_console;

/* coreboot CBMEM constants */
#define CBMEM_ALIGNMENT   (128 * 1024)
#define MEMORY_BOUNDARY   (1024 * 1024 * 1024)
#define MAX_CBMEM_ENTRIES 16
#define CBMEM_ENTRY_MAGIC 0x434f5245
#define CBMEM_CONSOLE_ID  0x434f4e53
#define CBMEM_ACPI_NAME   "\\CMEM"
#define CBMEMC_ACPI_NAME  "\\CBMC"

static bool check_acpi_coreboot_memconsole(void)
{
	acpi_handle handle;
	acpi_status status;
	unsigned long long value;
	struct cbmem_cons __iomem *tmp_cbmc;

	/*
	 * Attempt to use defined ACPI name to locate coreboot mem console.
	 */
	status = acpi_get_handle(NULL, CBMEMC_ACPI_NAME, &handle);
	if (!ACPI_SUCCESS(status))
		return false;

	status = acpi_evaluate_integer(handle, CBMEMC_ACPI_NAME,
				       NULL, &value);
		/* Start scan at this address */
	if (!ACPI_SUCCESS(status) || value == 0)
		return false;

	tmp_cbmc = ioremap_cache(value, sizeof(*tmp_cbmc));

	if (tmp_cbmc == NULL)
		return false;

	cbmem_console = ioremap_cache(value, tmp_cbmc->buffer_size);

	iounmap(tmp_cbmc);

	if (cbmem_console == NULL)
		return false;

	memconsole_baseaddr = cbmem_console->buffer_body;
	memconsole_length = min(cbmem_console->buffer_cursor,
				cbmem_console->buffer_size);
	return true;
}

static bool check_cbmem(void)
{
	struct sysinfo sysi;
	phys_addr_t top_of_ram, scan_addr = 0;
	acpi_handle handle;
	acpi_status status;
	unsigned long long value;

	if (check_acpi_coreboot_memconsole())
		return true;

	/*
	 * Attempt to use defined ACPI name to locate CBMEM TOC.
	 */
	status = acpi_get_handle(NULL, CBMEM_ACPI_NAME, &handle);
	if (ACPI_SUCCESS(status)) {
		status = acpi_evaluate_integer(handle, CBMEM_ACPI_NAME,
					       NULL, &value);
		/* Start scan at this address */
		if (ACPI_SUCCESS(status) && value > 0)
			scan_addr = (phys_addr_t) value;
	}

	/*
	 * Otherwise determine where to start looking for CBMEM signature:
	 * take the top of usable memory and align it up to 128K boundary.
	 */
	if (!scan_addr) {
		si_meminfo(&sysi);
		top_of_ram = (phys_addr_t) sysi.totalram << PAGE_SHIFT;
		scan_addr = ALIGN(top_of_ram, CBMEM_ALIGNMENT) +
			CBMEM_ALIGNMENT;
	}

	while (scan_addr % MEMORY_BOUNDARY) {
		struct cbmem_entry __iomem *pcbm;
		int i, remap_size = sizeof(struct cbmem_entry) * 16;

		/*
		 * See if we reached reserved memory. Bail out if so, as it is
		 * not mappable and is above the region where the CBMEM could
		 * be.
		 */
		if (e820_any_mapped(scan_addr,
				    scan_addr + remap_size,
				    E820_RESERVED))
			break;

		pcbm = ioremap_cache(scan_addr, remap_size);
		if (!pcbm) {
			scan_addr += CBMEM_ALIGNMENT;
			continue;
		}

		if (pcbm->magic != CBMEM_ENTRY_MAGIC) {
			iounmap(pcbm);
			scan_addr += CBMEM_ALIGNMENT;
			continue;
		}

		/* CBMEM found. Is the console log there? */
		for (i = 1; i < MAX_CBMEM_ENTRIES; i++) {
			if ((pcbm[i].magic == CBMEM_ENTRY_MAGIC) &&
			    (pcbm[i].id == CBMEM_CONSOLE_ID)) {
				/* Yes, map the log. */
				cbmem_console = ioremap_cache(pcbm[i].base,
							pcbm[i].size);
				break;
			}
		}
		iounmap(pcbm);
		break;
	}

	if (cbmem_console) {
		memconsole_baseaddr = cbmem_console->buffer_body;
		memconsole_length = min(cbmem_console->buffer_cursor,
					cbmem_console->buffer_size);
		return true;
	}

	printk(KERN_INFO "CBMEM console structure not found!\n");
	return false;
}

/*
 * Search through the EBDA for the BIOS Memory Console, and
 * set the global variables to point to it.  Return true if found.
 */
static bool found_memconsole(void)
{
	unsigned int address;
	size_t length, cur;

	/* Is it communicated through CBMEM? */
	if (coreboot_system && check_cbmem())
		return true;

	address = get_bios_ebda();
	if (!address) {
		printk(KERN_INFO "BIOS EBDA non-existent.\n");
		return false;
	}

	/* EBDA length is byte 0 of EBDA (in KB) */
	length = *(u8 *)phys_to_virt(address);
	length <<= 10; /* convert to bytes */

	/*
	 * Search through EBDA for BIOS memory console structure
	 * note: signature is not necessarily dword-aligned
	 */
	for (cur = 0; cur < length; cur++) {
		struct biosmemcon_ebda *hdr = phys_to_virt(address + cur);

		/* memconsole v1 */
		if (hdr->signature == BIOS_MEMCONSOLE_V1_MAGIC) {
			found_v1_header(hdr);
			return true;
		}

		/* memconsole v2 */
		if (hdr->signature == BIOS_MEMCONSOLE_V2_MAGIC) {
			found_v2_header(hdr);
			return true;
		}
	}

	printk(KERN_INFO "BIOS console EBDA structure not found!\n");
	return false;
}

static struct dmi_system_id memconsole_dmi_table[] __initdata = {
	{
		.ident = "Google Board",
		.matches = {
			DMI_MATCH(DMI_BOARD_VENDOR, "Google, Inc."),
		},
	},
	{
		.ident = "Google Board",
		.callback = set_coreboot_system_flag,
		.matches = {
			DMI_MATCH(DMI_BIOS_VENDOR, "coreboot"),
		},
	},
	{}
};
MODULE_DEVICE_TABLE(dmi, memconsole_dmi_table);

static int __init memconsole_init(void)
{
	int ret;

	if (!dmi_check_system(memconsole_dmi_table))
		return -ENODEV;

	if (!found_memconsole())
		return -ENODEV;

	memconsole_bin_attr.size = memconsole_length;

	ret = sysfs_create_bin_file(firmware_kobj, &memconsole_bin_attr);

	return ret;
}

static void __exit memconsole_exit(void)
{
	if (cbmem_console)
		iounmap(cbmem_console);

	sysfs_remove_bin_file(firmware_kobj, &memconsole_bin_attr);
}

module_init(memconsole_init);
module_exit(memconsole_exit);

MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL");
