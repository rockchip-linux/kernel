/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/mempolicy.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <asm/cacheflush.h>
#include <linux/crc32.h>

#include "typedef.h"
#include "sfc.h"
#include "sfc_nor.h"
#include "rk_sfc_blk.h"

#define SPI_VENDOR_TEST		0
#define	DRM_DEBUG		1

#if DRM_DEBUG
#define DLOG(fmt, args...)	pr_info(fmt, ##args)
#else
#define DLOG(x...)
#endif

void rknand_print_hex(char *s, void *buf, int width, int len)
{
	print_hex_dump(KERN_WARNING, s, DUMP_PREFIX_OFFSET,
		       16, width, buf, len, 0);
}

struct vendor_item {
	u16  id;
	u16  offset;
	u16  size;
	u16  flag;
};

#define	SPI_VENDOR_PART_START	8
#define	SPI_VENDOR_PART_SIZE	8
#define SPI_VENDOR_PART_NUM	4
#define SPI_VENDOR_TAG		0x524B5644
struct tag_vendor_info {
	u32	tag;
	u32	version;
	u16	next_index;
	u16	item_num;
	u16	free_offset;
	u16	free_size;
	struct vendor_item item[62]; /* 62 * 8*/
	u8	data[SPI_VENDOR_PART_SIZE * 512 - 512 - 8];
	u32	hash;
	u32	version2;
};

static struct tag_vendor_info *g_vendor;

u32 spi_vendor_init(void)
{
	u32 i, max_ver, max_index;

	g_vendor = kmalloc(sizeof(*g_vendor), GFP_KERNEL | GFP_DMA);
	if (!g_vendor)
		return 0;

	max_ver = 0;
	max_index = 0;
	for (i = 0; i < SPI_VENDOR_PART_NUM; i++) {
		snor_read(SPI_VENDOR_PART_START + SPI_VENDOR_PART_SIZE * i,
			  SPI_VENDOR_PART_SIZE,
			  g_vendor);
		if (g_vendor->tag == SPI_VENDOR_TAG &&
		    g_vendor->version == g_vendor->version2) {
			if (max_ver < g_vendor->version) {
				max_index = i;
				max_ver = g_vendor->version;
			}
		}
	}
	/* PRINT_E("max_ver = %d\n",max_ver); */
	if (max_ver) {
		snor_read(SPI_VENDOR_PART_START +
		SPI_VENDOR_PART_SIZE * max_index,
		SPI_VENDOR_PART_SIZE,
		g_vendor);
	} else {
		memset(g_vendor, 0, sizeof(g_vendor));
		g_vendor->version = 1;
		g_vendor->tag = SPI_VENDOR_TAG;
		g_vendor->version2 = g_vendor->version;
		g_vendor->free_offset = 0;
		g_vendor->free_size = sizeof(g_vendor->data);
	}
	/* rknand_print_hex("vendor:", g_vendor, 4, 1024); */
	return 0;
}

int spi_vendor_read(u32 id, void *pbuf, u32 size)
{
	u32 i;

	if (!g_vendor)
		return -1;

	for (i = 0; i < g_vendor->item_num; i++) {
		if (g_vendor->item[i].id == id) {
			if (size > g_vendor->item[i].size)
				size = g_vendor->item[i].size;
			memcpy(pbuf,
			       &g_vendor->data[g_vendor->item[i].offset],
			       size);
			return size;
		}
	}
	return (-1);
}
EXPORT_SYMBOL(spi_vendor_read);

int spi_vendor_write(u32 id, void *pbuf, u32 size)
{
	u32 i, next_index, algin_size;
	struct vendor_item *item;

	algin_size = (size + 0x3F) & (~0x3F); /* algin to 64 bytes*/
	next_index = g_vendor->next_index;
	for (i = 0; i < g_vendor->item_num; i++) {
		if (g_vendor->item[i].id == id) {
			if (size > algin_size)
				return -1;
			memcpy(&g_vendor->data[g_vendor->item[i].offset],
			       pbuf,
			       size);
			g_vendor->item[i].size = size;
			g_vendor->version++;
			g_vendor->version2 = g_vendor->version;
			g_vendor->next_index++;
			if (g_vendor->next_index >= SPI_VENDOR_PART_NUM)
				g_vendor->next_index = 0;
			rksfc_device_lock();
			snor_write(SPI_VENDOR_PART_START +
				SPI_VENDOR_PART_SIZE * next_index,
				SPI_VENDOR_PART_SIZE,
				g_vendor);
			rksfc_device_unlock();
			return 0;
		}
	}

	if (g_vendor->free_size >= algin_size) {
		item = &g_vendor->item[g_vendor->item_num];
		item->id = id;
		item->offset = g_vendor->free_offset;
		item->size = algin_size;
		item->size = size;
		g_vendor->free_offset += algin_size;
		g_vendor->free_size -= algin_size;
		memcpy(&g_vendor->data[item->offset], pbuf, size);
		g_vendor->item_num++;
		g_vendor->version++;
		g_vendor->next_index++;
		g_vendor->version2 = g_vendor->version;
		if (g_vendor->next_index >= SPI_VENDOR_PART_NUM)
			g_vendor->next_index = 0;
		rksfc_device_lock();
		snor_write(SPI_VENDOR_PART_START +
			SPI_VENDOR_PART_SIZE * next_index,
			SPI_VENDOR_PART_SIZE,
			g_vendor);
		rksfc_device_unlock();
		return 0;
	}

	return(-1);
}
EXPORT_SYMBOL(spi_vendor_write);

#if (SPI_VENDOR_TEST)
void spi_vendor_test(void)
{
	u32 i;
	u8 test_buf[512];

	memset(test_buf, 0, 512);
	for (i = 0; i < 62; i++) {
		memset(test_buf, i, i+1);
		spi_vendor_write(i, test_buf, i+1);
	}
	memset(test_buf, 0, 512);
	for (i = 0; i < 62; i++) {
		spi_vendor_read(i, test_buf, i+1);
		PRINT_E("id = %d ,size = %d\n", i, i+1);
		rknand_print_hex("data:", test_buf, 1, i+1);
	}
	spi_vendor_init();
	memset(test_buf, 0, 512);
	for (i = 0; i < 62; i++) {
		spi_vendor_read(i, test_buf, i+1);
		PRINT_E("id = %d ,size = %d\n", i, i+1);
		rknand_print_hex("data:", test_buf, 1, i+1);
	}
	while (1)
		;
}
#endif

struct RK_VENDOR_REQ {
	unsigned int tag;
	unsigned short id;
	unsigned short len;
	unsigned char data[1024];
};
#define VENDOR_REQ_TAG		0x56524551
#define VENDOR_READ_IO		_IOW('v', 0x01, unsigned int)
#define VENDOR_WRITE_IO		_IOW('v', 0x02, unsigned int)

static int rk_vendor_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int rk_vendor_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long rk_vendor_ioctl(struct file *file,
			    unsigned int cmd,
			    unsigned long arg)
{
	long ret = -EINVAL;
	int size;
	u32 *temp_buf;
	struct RK_VENDOR_REQ *req;

	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (req == NULL)
		return ret;

	temp_buf = (u32 *)req;

	switch (cmd) {
	case VENDOR_READ_IO:
	{
		if (copy_from_user(temp_buf,
				   (void __user *)arg,
				   sizeof(*req))) {
			DLOG("copy_from_user error\n");
			ret = -EFAULT;
			break;
		}
		if (req->tag == VENDOR_REQ_TAG) {
			size = spi_vendor_read(req->id, req->data, req->len);
			if (size > 0) {
				req->len = size;
				ret = 0;
				if (copy_to_user((void __user *)arg,
						 temp_buf,
						 sizeof(*req)))
					ret = -EFAULT;
			}
		}
	} break;
	case VENDOR_WRITE_IO:
	{
		if (copy_from_user(temp_buf,
				   (void __user *)arg,
				   sizeof(struct RK_VENDOR_REQ))) {
			DLOG("copy_from_user error\n");
			ret = -EFAULT;
			break;
		}
		if (req->tag == VENDOR_REQ_TAG)
			ret = spi_vendor_write(req->id, req->data, req->len);
	} break;
	default:
		return -EINVAL;
	}
	kfree(temp_buf);
	DLOG("rk_vendor_ioctl cmd=%x ret = %lx\n", cmd, ret);
	return ret;
}

const struct file_operations rk_vendor_fops = {
	.open = rk_vendor_open,
	.compat_ioctl	= rk_vendor_ioctl,
	.unlocked_ioctl = rk_vendor_ioctl,
	.release = rk_vendor_release,
};

static struct miscdevice rk_vendor_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "vendor_storage",
	.fops  = &rk_vendor_fops,
};

int spi_flash_init(void __iomem	*reg_addr)
{
	int ret;

	sfc_init(reg_addr);
	ret = snor_init();
	if (ret == SFC_OK) {
		spi_vendor_init();
		misc_register(&rk_vendor_dev);
	}
	#if (SPI_VENDOR_TEST)
	spi_vendor_test();
	#endif
	return ret;
}
EXPORT_SYMBOL_GPL(spi_flash_init);

void spi_flash_read_id(u8 chip_sel, void *buf)
{
	snor_read_id(buf);
}
EXPORT_SYMBOL_GPL(spi_flash_read_id);
