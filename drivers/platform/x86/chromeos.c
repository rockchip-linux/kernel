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

static void chromeos_set_nvram_flag(int index, unsigned char flag)
{
	unsigned char cur;

	cur = nvram_read_byte(index);
	/* already set. */
	if (cur & flag)
		return;
	nvram_write_byte(cur | flag, index);
}

bool chromeos_is_devmode(void)
{
	int chsw = chromeos_acpi_chsw;

	/*
	 * If chsw is unset, report system as in devmode since we're
	 * probably running on a non-chromeos system.
	 */

	return (chsw < 0) || (chsw & CHSW_DEVELOPER_MODE);
}
EXPORT_SYMBOL(chromeos_is_devmode);

int chromeos_set_need_recovery(void)
{
	if (!chromeos_inited)
		return -ENODEV;

	if (chromeos_acpi_chnv < 0) {
		pr_warning("chromeos_set_need_recovery(): Can't write to nvram\n");
		return -ENODEV;
	}

	chromeos_set_nvram_flag(chromeos_acpi_chnv, CHNV_RECOVERY_FLAG);

	return 0;
}
EXPORT_SYMBOL(chromeos_set_need_recovery);

bool chromeos_initialized(void)
{
	return chromeos_inited;
}
EXPORT_SYMBOL(chromeos_initialized);

static int __init chromeos_init(void)
{
	if (!chromeos_acpi_available)
		return -ENODEV;

	printk(KERN_INFO "Chrome OS platform detected\n");
	chromeos_inited = true;
	return 0;
}
subsys_initcall_sync(chromeos_init);


MODULE_AUTHOR("The Chromium OS Authors");
MODULE_DESCRIPTION("Chrome OS Platform Extras");
MODULE_LICENSE("GPL");
