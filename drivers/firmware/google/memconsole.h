/*
 * memconsole.h
 *
 * Internal headers of the memory based BIOS console.
 *
 * Copyright 2014 Google Inc.
 */

#ifndef __FIRMWARE_GOOGLE_MEMCONSOLE_H
#define __FIRMWARE_GOOGLE_MEMCONSOLE_H

#include <linux/io.h>
#include <linux/types.h>

/* Initialize the memory console given physical address of console buffer */
int memconsole_coreboot_init(phys_addr_t physaddr);

/* Initialize the memory console from raw (virtual) base address and length. */
void memconsole_setup(void *baseaddr, size_t length);

/* Update memory console length and create binary file for firmware object */
int memconsole_sysfs_init(void);

/* Unmap console buffer */
void memconsole_exit(void);

#endif /* __FIRMWARE_GOOGLE_MEMCONSOLE_H */
