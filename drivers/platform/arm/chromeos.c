/*
 *  ChromeOS platform support code. Glue layer between higher level functions
 *  and per-platform firmware interfaces.
 *
 *  Copyright (C) 2011 The Chromium OS Authors
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

#include <linux/chromeos_platform.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

u64 phys_window_base;
EXPORT_SYMBOL(phys_window_base);

int chromeos_set_need_recovery(void)
{
	return -1;
}

static int __init get_mem_base(char *p)
{
        char *endptr;   /* local pointer to end of parsed string */
        u64 base = simple_strtoull(p, &endptr, 0);

	/* rudimentary sanity check */
	if (base & ((1 << 20) -1)) {
		pr_err("chromeos: unaligned window base 0x%llx\n", base);
		return -1;
	}

	phys_window_base = base;
	return 0;
}

early_param("cros_shared_mem", get_mem_base);

