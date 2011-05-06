/*
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
#ifndef _DRIVERS_PLATFORM_CHROMEOS_H
#define _DRIVERS_PLATFORM_CHROMEOS_H

#define MAX_NVRAM_BUFFER_SIZE 64  /* Should be enough for anything. */

#ifdef CONFIG_ACPI_CHROMEOS
extern int chromeos_legacy_set_need_recovery(void);
#else
static inline int chromeos_legacy_set_need_recovery(void) { return -ENODEV; }
#endif

extern int chromeos_platform_read_nvram(u8 *nvram_buffer, int buf_size);
extern int chromeos_platform_write_nvram(u8 *nvram_buffer, int buf_size);

#endif /* _DRIVERS_PLATFORM_CHROMEOS_H */
