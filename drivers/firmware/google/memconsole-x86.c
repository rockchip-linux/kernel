/*
 * memconsole-x86.c
 *
 * x86/ACPI specific parts of the memory based BIOS console.
 * (Could be refactored to allow non-x86 ACPI platforms in the future.)
 *
 * Copyright 2014 Google Inc.
 */

#include <asm/bios_ebda.h>
#include <asm/e820.h>
#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>

#include "memconsole.h"

#define BIOS_MEMCONSOLE_V1_MAGIC	0xDEADBABE
#define BIOS_MEMCONSOLE_V2_MAGIC	(('M')|('C'<<8)|('O'<<16)|('N'<<24))

/* coreboot CBMEM constants */
#define CBMEM_ALIGNMENT   (128 * 1024)
#define MEMORY_BOUNDARY   (1024 * 1024 * 1024)
#define MAX_CBMEM_ENTRIES 16
#define CBMEM_ENTRY_MAGIC 0x434f5245
#define CBMEM_CONSOLE_ID  0x434f4e53
#define CBMEM_TOC_ACPI_NAME   "\\CMEM"
#define CBMEM_CONSOLE_ACPI_NAME  "\\CBMC"

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

struct cbmem_entry {
	u32 magic;
	u32 id;
	u64 base;
	u64 size;
} __packed;

static bool coreboot_system;

static int set_coreboot_system_flag(const struct dmi_system_id *unused)
{
	coreboot_system = true;
	return true;
}

static void found_v1_header(struct biosmemcon_ebda *hdr)
{
	pr_info("memconsole: BIOS console v1 EBDA structure found at %p\n", hdr);
	pr_info("memconsole: BIOS console buffer at 0x%.8x, start = %d, end = %d, num = %d\n",
	       hdr->v1.buffer_addr, hdr->v1.start,
	       hdr->v1.end, hdr->v1.num_chars);

	memconsole_setup(phys_to_virt(hdr->v1.buffer_addr), hdr->v1.num_chars);
}

static void found_v2_header(struct biosmemcon_ebda *hdr)
{
	pr_info("memconsole: BIOS console v2 EBDA structure found at %p\n", hdr);
	pr_info("memconsole: BIOS console buffer at 0x%.8x, start = %d, end = %d, num_bytes = %d\n",
	       hdr->v2.buffer_addr, hdr->v2.start,
	       hdr->v2.end, hdr->v2.num_bytes);

	memconsole_setup(phys_to_virt(hdr->v2.buffer_addr + hdr->v2.start),
			 hdr->v2.end - hdr->v2.start);
}

static phys_addr_t get_address_from_acpi(acpi_string pathname)
{
	acpi_handle handle;
	unsigned long long addr;

	if (!ACPI_SUCCESS(acpi_get_handle(NULL, pathname, &handle)))
		return 0;

	if (!ACPI_SUCCESS(acpi_evaluate_integer(handle, pathname, NULL, &addr)))
		return 0;

	return addr;
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

static phys_addr_t check_cbmem(void)
{
	struct sysinfo sysi;
	phys_addr_t top_of_ram, scan_addr;

	/* Get CBMEM TOC address from ACPI if available. */
	scan_addr = get_address_from_acpi(CBMEM_TOC_ACPI_NAME);

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
				/* Yes, return its address. */
				phys_addr_t ret = pcbm[i].base;
				iounmap(pcbm);
				return ret;
			}
		}
		iounmap(pcbm);
		break;
	}

	pr_warn("memconsole: CBMEM console structure not found!\n");
	return 0;
}

/*
 * Search through the EBDA for the BIOS Memory Console, and
 * set the global variables to point to it.  Return true if found.
 */
static bool memconsole_ebda_init(void)
{
	unsigned int address;
	size_t length, cur;

	address = get_bios_ebda();
	if (!address) {
		pr_info("memconsole: BIOS EBDA non-existent.\n");
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

	pr_info("memconsole: BIOS console EBDA structure not found!\n");
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

static bool __init memconsole_find(void)
{
	phys_addr_t physaddr;

	if (!dmi_check_system(memconsole_dmi_table))
		return false;

	if (coreboot_system) {
		physaddr = get_address_from_acpi(CBMEM_CONSOLE_ACPI_NAME);
		if (physaddr && memconsole_coreboot_init(physaddr) == 0)
			return true;

		physaddr = check_cbmem();
		if (physaddr && memconsole_coreboot_init(physaddr) == 0)
			return true;
	}

	return memconsole_ebda_init();
}

static int __init memconsole_x86_init(void)
{
	if (!memconsole_find())
		return -ENODEV;

	return memconsole_sysfs_init();
}

static void __exit memconsole_x86_exit(void)
{
	memconsole_exit();
}

module_init(memconsole_x86_init);
module_exit(memconsole_x86_exit);

MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL");
