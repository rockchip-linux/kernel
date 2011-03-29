/*
 *  ChromeOS platform support code. Glue layer between higher level functions
 *  and per-platform firmware interfaces.
 *
 *  Copyright (C) 2010 The Chromium OS Authors
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This module isolates ChromeOS platform specific behavior.  In particular,
 * it uses calls from chromeos_acpi.c to control the boot flow, and exports some
 * helper functions for kernel-side consumers of platform configuration, such
 * as nvram flags.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/nvram.h>
#include <linux/types.h>
#include <linux/chromeos_platform.h>

#include "chromeos_acpi.h"

static bool chromeos_inited;
static int chromeos_read_nvram(u8 *nvram_buffer, int buf_size);
static int chromeos_write_nvram(unsigned offset, u8 value);

#define MAX_NVRAM_BUFFER_SIZE 64  /* Should be enough for anything. */

/* the following defines are copied from
 * vboot_reference:firmware/lib/vboot_nvstorage.c.
 */
#define RECOVERY_OFFSET              2
#define VBNV_RECOVERY_RW_INVALID_OS  0x43

bool chromeos_is_devmode(void)
{
	struct chromeos_acpi_datum *chsw = &chromeos_acpi_if_data.switch_state;
	/*
	 * If switch_state is unset, report system as in devmode since we're
	 * probably running on a non-chromeos system.
	 */
	return (!chsw->cad_is_set) || (chsw->cad_value & CHSW_DEVELOPER_MODE);
}
EXPORT_SYMBOL(chromeos_is_devmode);

int chromeos_set_need_recovery(void)
{
	return chromeos_write_nvram(RECOVERY_OFFSET,
				    VBNV_RECOVERY_RW_INVALID_OS);
}
EXPORT_SYMBOL(chromeos_set_need_recovery);

bool chromeos_initialized(void)
{
	return chromeos_inited;
}
EXPORT_SYMBOL(chromeos_initialized);

/*
 * Lifted from vboot_reference:firmware/lib/vboot_nvstorage.c and formatted.
 *
 * Return CRC-8 of the data, using x^8 + x^2 + x + 1 polynomial. A table-based
 * algorithm would be faster, but for only 15 bytes isn't worth the code size.
 */
static u8 crc8(const u8 *data, int len)
{
	unsigned crc = 0;
	int i, j;

	for (j = len; j; j--, data++) {
		crc ^= (*data << 8);
		for (i = 8; i; i--) {
			if (crc & 0x8000)
				crc ^= (0x1070 << 3);
			crc <<= 1;
		}
	}
	return (u8)(crc >> 8);
}

static int chromeos_write_nvram(unsigned offset, u8 value)
{
	u8 nvram_buffer[MAX_NVRAM_BUFFER_SIZE];
	u8 new_crc;
	unsigned size;
	unsigned base;

	if (chromeos_read_nvram(nvram_buffer, sizeof(nvram_buffer)))
		return -ENODEV;

	size = chromeos_acpi_if_data.nv_size.cad_value;
	base = chromeos_acpi_if_data.nv_base.cad_value;;

	if (offset >= (size - 1))
		return -EINVAL;

	nvram_buffer[offset] = value;
	new_crc = crc8(nvram_buffer, size - 1);

	nvram_write_byte(base + offset, value);
	nvram_write_byte(base + size - 1, new_crc);
	return 0;
}

/*
 * read nvram buffer contents and verify it.
 * Return 0 on success and -1 on failure (uninitialized, or corrupted crc8
 * value). The passed in memory space is guaranteed to be large enough for the
 * nvram buffer contents.
 */
static int chromeos_read_nvram(u8 *nvram_buffer, int buf_size)
{
	int start, size, i;

	if (!chromeos_inited)
		return -1;

	start = chromeos_acpi_if_data.nv_base.cad_value;
	size = chromeos_acpi_if_data.nv_size.cad_value;

	if (size > buf_size) {
		printk(KERN_ERR "%s: buffer range exceeded\n", __func__);
		return -1;
	}

	for (i = 0; i < size; i++)
		nvram_buffer[i] = nvram_read_byte(start++);

	if (nvram_buffer[size - 1] != crc8(nvram_buffer, size - 1)) {
		printk(KERN_ERR "%s: NVRAM contents corrupted\n", __func__);
		return -1;
	}
	return 0;
}

static int __init chromeos_init(void)
{
	u8 nvram_buffer[MAX_NVRAM_BUFFER_SIZE];

	if (!chromeos_acpi_if_data.nv_base.cad_is_set ||
	    !chromeos_acpi_if_data.nv_size.cad_is_set ||
	    (chromeos_acpi_if_data.nv_size.cad_value > MAX_NVRAM_BUFFER_SIZE)) {
		printk(KERN_ERR "Chrome OS platform not found\n");
		return -ENODEV;
	}

	chromeos_inited = true;

	printk(KERN_INFO "Chrome OS platform detected\n");

	/* check NVRAM buffer sanity */
	if (chromeos_read_nvram(nvram_buffer, sizeof(nvram_buffer)))
		printk(KERN_ERR "Failed reading NVRAM section\n");

	return 0;
}
subsys_initcall_sync(chromeos_init);
