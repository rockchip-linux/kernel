/*
 * Intel Smart Sound Technology (SST) debugfs support
 *
 * Copyright (C) 2014, Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SOUND_SOC_SST_DEBUGFS_H
#define __SOUND_SOC_SST_DEBUGFS_H
#ifdef DEBUG
int sst_debugfs_add_mmio_entry(const char *filename,
			       void *buf, size_t size, void **ctx);
void sst_debugfs_remove_mmio_entry(void *ctx);
int sst_debugfs_init(const char *dirname);
void sst_debugfs_exit(void);
void sst_debugfs_get_root(struct dentry *root);
#else
#define sst_debugfs_add_mmio_entry(a, b, c, d)
#define sst_debugfs_remove_mmio_entry(a)
#define sst_debugfs_init(a)
#define sst_debugfs_exit()
#define sst_debugfs_get_root(a)
#endif

#endif
