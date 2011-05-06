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

#include <linux/types.h>
#include <linux/chromeos_platform.h>
#include <linux/module.h>
#include "chromeos.h"

static int chromeos_read_nvram(u8 *nvram_buffer, int buf_size);
static int chromeos_write_nvram_byte(unsigned offset, u8 value);

/* the following defines are copied from
 * vboot_reference:firmware/lib/vboot_nvstorage.c.
 */
#define RECOVERY_OFFSET              2
#define VBNV_RECOVERY_RW_INVALID_OS  0x43

int chromeos_set_need_recovery(void)
{
	if (!chromeos_legacy_set_need_recovery())
		return 0;

	return chromeos_write_nvram_byte(RECOVERY_OFFSET,
					 VBNV_RECOVERY_RW_INVALID_OS);
}

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

static int chromeos_write_nvram_byte(unsigned offset, u8 value)
{
	u8 nvram_buffer[MAX_NVRAM_BUFFER_SIZE];

	int size = chromeos_read_nvram(nvram_buffer, sizeof(nvram_buffer));

	if (size <= 0)
		return -EINVAL;

	if (offset >= (size - 1))
		return -EINVAL;

	if (nvram_buffer[offset] == value)
		return 0;

	nvram_buffer[offset] = value;
	nvram_buffer[size - 1] = crc8(nvram_buffer, size - 1);

	return chromeos_platform_write_nvram(nvram_buffer, size);
}

/*
 * Read nvram buffer contents and verify it. Return 0 on success and -1 on
 * failure (uninitialized subsystem, corrupted crc8 value, not enough room in
 * the buffer, etc.).
 *
 * If everything checks out - return number of bytes in the NVRAM buffer, -1
 * on any error.
 */
static int chromeos_read_nvram(u8 *nvram_buffer, int buf_size)
{
	int size = chromeos_platform_read_nvram(nvram_buffer, buf_size);

	if (size <= 0)
		return -1;

	if (nvram_buffer[size - 1] != crc8(nvram_buffer, size - 1)) {
		pr_err("%s: NVRAM contents corrupted\n", __func__);
		return -1;
	}
	return size;
}
