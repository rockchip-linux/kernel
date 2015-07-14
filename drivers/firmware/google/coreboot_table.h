/*
 *  coreboot_table.c: Internal header for coreboot table access.
 *
 *  Copyright 2015 Google Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License v2.0 as published by
 *  the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#ifndef __COREBOOT_TABLE_H
#define __COREBOOT_TABLE_H

#include <linux/platform_device.h>

/* List of coreboot entry structures that is used */
struct lb_cbmem_ref {
	uint32_t tag;
	uint32_t size;

	uint64_t cbmem_addr;
};

/* Retrieve coreboot table entry with tag *tag* and copy it to data */
int coreboot_table_find(int tag, void *data, size_t data_size);

#endif /* __COREBOOT_TABLE_H */
