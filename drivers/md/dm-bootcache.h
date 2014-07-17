/*
 * Copyright 2012 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef DM_BOOTCACHE_H
#define DM_BOOTCACHE_H

#include <linux/types.h>
#include <linux/utsname.h>

enum {	BOOTCACHE_MAGIC = 1651470196,
	BOOTCACHE_VERSION = 3,
	MAX_SIGNATURE = 256
};

struct bootcache_trace {
	__u64	sector;	/* Sector offset */
	__u64	count;	/* Number of blocks traced */
	__u64	ino;	/* Inode number of file */
};

struct bootcache_hdr {
	__u64	sector;		/* Sector offset where header is stored */
	__u32	magic;		/* Magic number */
	__u32	version;	/* Verion of boot cache */
	__u32	state;		/* Curent state */
	__u32	num_trace_recs;	/* Number of trace reords */
	__u32	sectors_meta;	/* Size of trace data on disk in sectors*/
	__u32	sectors_data;	/* Size of the data area in sectors*/
	__u32	max_sectors;	/* Max sectors that can to read */
	__u32	max_hw_sectors;	/* Max hardware sectore that can be read */
	__u32	alignment;	/* Alignement on disk */
	/* Date and time dm-bootcache was compiled */
	char	timestamp[__NEW_UTS_LEN + 1];
	char	signature[MAX_SIGNATURE];
};

#endif
