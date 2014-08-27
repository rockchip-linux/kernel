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

#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

static struct dentry *rootdir;

struct sst_dfsentry {
	struct dentry *dfsentry;
	size_t size;
	void *buf;
};

static int sst_dfsentry_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t sst_dfsentry_read(struct file *file, char __user *buffer,
				 size_t count, loff_t *ppos)
{
	struct sst_dfsentry *dfse = file->private_data;
	int i, size;
	u32 *buf;

	pr_debug("%s: pbuf: %p, *ppos: 0x%llx", __func__, buffer, *ppos);

	size = dfse->size;

	if (*ppos >= size)
		return 0;
	if (*ppos + count > size)
		count = size - *ppos;

	size = (count + 3) & (~3);
	buf = kzalloc(size, GFP_KERNEL);
	if (!buf) {
		pr_err(" %s: kzalloc failed, aborting\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < size / sizeof(*buf); i++)
		buf[i] = *(u32 *)(dfse->buf + *ppos + i * sizeof(*buf));

	if (copy_to_user(buffer, buf, count))
		return 0;
	kfree(buf);

	*ppos += count;

	pr_debug("%s: *ppos: 0x%llx, count: %zu", __func__, *ppos, count);

	return count;
}

static ssize_t sst_dfsentry_write(struct file *file, const char __user *buffer,
				  size_t count, loff_t *ppos)
{
	struct sst_dfsentry *dfse = file->private_data;
	int i, size;
	u32 *buf;

	pr_debug("%s: pbuf: %p, *ppos: 0x%llx", __func__, buffer, *ppos);

	size = dfse->size;

	if (*ppos >= size)
		return 0;
	if (*ppos + count > size)
		count = size - *ppos;

	size = (count + 3) & (~3);
	buf = kzalloc(size, GFP_KERNEL);
	if (!buf) {
		pr_err(" %s: kzalloc failed, aborting\n", __func__);
		return -ENOMEM;
	}

	if (copy_from_user(buf, buffer, count))
		return 0;

	for (i = 0; i < size / sizeof(*buf); i++)
		*(u32 *)(dfse->buf + *ppos + i * sizeof(*buf)) = buf[i];

	kfree(buf);
	*ppos += count;

	pr_debug("%s: *ppos: 0x%llx, count: %zu", __func__, *ppos, count);

	return count;
}

static const struct file_operations sst_dfs_fops = {
	.owner = THIS_MODULE,
	.open = sst_dfsentry_open,
	.read = sst_dfsentry_read,
	.write = sst_dfsentry_write,
};

int sst_debugfs_add_mmio_entry(const char *filename, void *buf,
			       size_t size, void **ctx)
{
	struct sst_dfsentry *dfse;

	dfse = kzalloc(sizeof(*dfse), GFP_KERNEL);
	if (!dfse) {
		pr_err("%s: cannot create debugfs entry.\n", __func__);
		return -ENOMEM;
	}

	dfse->buf = buf;
	dfse->size = size;
	dfse->dfsentry = debugfs_create_file(
				filename, 0644, rootdir, dfse, &sst_dfs_fops);
	if (!dfse->dfsentry) {
		pr_err("%s: cannot create debugfs entry.\n", __func__);
		return -ENODEV;
	}
	*ctx = dfse;

	return 0;
}

void sst_debugfs_remove_mmio_entry(void *ctx)
{
	struct sst_dfsentry *dfse = (struct sst_dfsentry *)ctx;

	debugfs_remove(dfse->dfsentry);
	kfree(dfse);
}

int sst_debugfs_init(const char *dirname)
{
	if (rootdir) {
		pr_err("%s: debugfs rootdir already initiated.\n", __func__);
		return -EEXIST;
	}
	rootdir = debugfs_create_dir(dirname, NULL);

	return 0;
}

void sst_debugfs_exit(void)
{
	debugfs_remove_recursive(rootdir);
}

void sst_debugfs_get_root(struct dentry *root)
{
	if (root) {
		*root = *rootdir;
	}
}

