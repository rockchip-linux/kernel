/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/blkpg.h>
#include <linux/spinlock.h>
#include <linux/hdreg.h>
#include <linux/init.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/version.h>

#include "rk_sfc_blk.h"
#include "rk_sfc_api.h"

static struct sfc_part disk_array[MAX_PART_COUNT];
static int g_max_part_num = 4;

#define PART_READONLY 0x85
#define PART_WRITEONLY 0x86
#define PART_NO_ACCESS 0x87

static unsigned long totle_read_data;
static unsigned long totle_write_data;
static unsigned long totle_read_count;
static unsigned long totle_write_count;
static int rk_sfc_dev_initialised;

static char *mtd_read_temp_buffer;
#define MTD_RW_SECTORS (512)

struct STRUCT_PART_INFO g_part;  /* size 2KB */
#define RK_PARTITION_TAG 0x50464B52
unsigned int rk_partition_init(struct sfc_part *part)
{
	int i;

	if (snor_read(0, 4, &g_part) == 0) {
		if (g_part.hdr.ui_fw_tag == RK_PARTITION_TAG) {
			for (i = 0;
				i < g_part.hdr.ui_part_entry_count;
				i++) {
				memcpy(part[i].name,
				       g_part.part[i].sz_name,
				       32);
				part[i].offset = g_part.part[i].ui_pt_off;
				part[i].size = g_part.part[i].ui_pt_sz;
				part[i].type = 0;
			}
			return g_part.hdr.ui_part_entry_count;
		}
	}
	return 0;
}

static int rksfc_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "Totle Read %ld KB\n", totle_read_data >> 1);
	seq_printf(m, "Totle Write %ld KB\n", totle_write_data >> 1);
	seq_printf(m, "totle_write_count %ld\n", totle_write_count);
	seq_printf(m, "totle_read_count %ld\n", totle_read_count);
	return 0;
}

static int rksfc_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rksfc_proc_show, PDE_DATA(inode));
}

static const struct file_operations rksfc_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= rksfc_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int rksfc_create_procfs(void)
{
	struct proc_dir_entry *ent;

	ent = proc_create_data("rksfc", 0x666, NULL, &rksfc_proc_fops,
			       (void *)0);
	if (!ent)
		return -1;

	return 0;
}

static struct mutex g_rk_sfc_ops_mutex;
static void rksfc_device_lock_init(void)
{
	mutex_init(&g_rk_sfc_ops_mutex);
}

void rksfc_device_lock(void)
{
	mutex_lock(&g_rk_sfc_ops_mutex);
}

int rksfc_device_trylock(void)
{
	return mutex_trylock(&g_rk_sfc_ops_mutex);
}

void rksfc_device_unlock(void)
{
	mutex_unlock(&g_rk_sfc_ops_mutex);
}

static int sfc_dev_transfer(struct sfc_blk_dev *dev,
			    unsigned long start,
			    unsigned long nsector,
			    char *buf,
			    int cmd,
			    int totle_nsec)
{
	int ret;

	if (dev->disable_access ||
	    ((cmd == WRITE) && dev->readonly) ||
	    ((cmd == READ) && dev->writeonly)) {
		return -EIO;
	}

	start += dev->off_size;
	rksfc_device_lock();

	switch (cmd) {
	case READ:
		totle_read_data += nsector;
		totle_read_count++;
		ret = snor_read(start, nsector, buf);
		if (ret)
			ret = -EIO;
		break;

	case WRITE:
		totle_write_data += nsector;
		totle_write_count++;
		ret = snor_write(start, nsector, buf);
		if (ret)
			ret = -EIO;
		break;

	default:
		ret = -EIO;
		break;
	}

	rksfc_device_unlock();
	return ret;
}

static DECLARE_WAIT_QUEUE_HEAD(rksfc_thread_wait);
static unsigned long rk_sfc_req_jiffies;
static int req_check_buffer_align(struct request *req, char **pbuf)
{
	int nr_vec = 0;
	struct bio_vec *bv;
	struct req_iterator iter;
	char *buffer;
	void *firstbuf = 0;
	char *nextbuffer = 0;
	unsigned long block, nsect;

	block = blk_rq_pos(req);
	nsect = blk_rq_cur_bytes(req) >> 9;
	rq_for_each_segment(bv, req, iter) {
		buffer = page_address(bv->bv_page) + bv->bv_offset;
		if (firstbuf == 0)
			firstbuf = buffer;
		nr_vec++;
		if (nextbuffer != 0 && nextbuffer != buffer)
			return 0;
		nextbuffer = buffer + bv->bv_len;
	}
	*pbuf = firstbuf;
	return 1;
}

static int sfc_blktrans_thread(void *arg)
{
	struct sfc_blk_ops *sfc_ops = arg;
	struct request_queue *rq = sfc_ops->rq;
	struct request *req = NULL;
	char *buf;
	struct req_iterator rq_iter;
	struct bio_vec *bvec;
	unsigned long long sector_index = ULLONG_MAX;
	unsigned long totle_nsect;
	unsigned long rq_len = 0;
	int rw_flag = 0;

	spin_lock_irq(rq->queue_lock);
	while (!sfc_ops->quit) {
		int res;
		struct sfc_blk_dev *dev;
		DECLARE_WAITQUEUE(wait, current);

		if (!req)
			req = blk_fetch_request(rq);
		if (!req) {
			add_wait_queue(&sfc_ops->thread_wq, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			spin_unlock_irq(rq->queue_lock);
			rk_sfc_req_jiffies = HZ / 10;
			wait_event_timeout(sfc_ops->thread_wq,
					   sfc_ops->quit,
					   rk_sfc_req_jiffies);

			spin_lock_irq(rq->queue_lock);
			remove_wait_queue(&sfc_ops->thread_wq, &wait);
			continue;
		} else {
			rk_sfc_req_jiffies = 1 * HZ;
		}

		dev = req->rq_disk->private_data;
		totle_nsect = (req->__data_len) >> 9;
		sector_index = blk_rq_pos(req);
		rq_len = 0;
		buf = 0;
		res = 0;

		if (req->cmd_flags & REQ_DISCARD) {
			if (!__blk_end_request_cur(req, res))
				req = NULL;
			continue;
		} else if (req->cmd_flags & REQ_FLUSH) {
			if (!__blk_end_request_cur(req, res))
				req = NULL;
			continue;
		}

		rw_flag = req->cmd_flags & REQ_WRITE;
		if (rw_flag == READ && mtd_read_temp_buffer) {
			buf = mtd_read_temp_buffer;
			req_check_buffer_align(req, &buf);
			spin_unlock_irq(rq->queue_lock);
			res = sfc_dev_transfer(dev,
					       sector_index,
					       totle_nsect,
					       buf,
					       rw_flag,
					       totle_nsect);
			spin_lock_irq(rq->queue_lock);
			if (buf == mtd_read_temp_buffer) {
				char *p = buf;

				rq_for_each_segment(bvec, req, rq_iter) {
					memcpy(page_address(bvec->bv_page) +
					       bvec->bv_offset,
					       p,
					       bvec->bv_len);
					p += bvec->bv_len;
				}
			}
		} else {
			rq_for_each_segment(bvec, req, rq_iter) {
				if ((page_address(bvec->bv_page)
					+ bvec->bv_offset)
					== (buf + rq_len)) {
					rq_len += bvec->bv_len;
				} else {
					if (rq_len) {
						spin_unlock_irq(rq->queue_lock);
						res = sfc_dev_transfer(dev,
								sector_index,
								rq_len >> 9,
								buf,
								rw_flag,
								totle_nsect);
						spin_lock_irq(rq->queue_lock);
					}
					sector_index += rq_len >> 9;
					buf = (page_address(bvec->bv_page) +
						bvec->bv_offset);
					rq_len = bvec->bv_len;
				}
			}
			if (rq_len) {
				spin_unlock_irq(rq->queue_lock);
				res = sfc_dev_transfer(dev,
						       sector_index,
						       rq_len >> 9,
						       buf,
						       rw_flag,
						       totle_nsect);
				spin_lock_irq(rq->queue_lock);
			}
		}
		__blk_end_request_all(req, res);
		req = NULL;
	}
	pr_info("sfc th quited\n");
	sfc_ops->sfc_th_quited = 1;
	if (req)
		__blk_end_request_all(req, -EIO);
	while ((req = blk_fetch_request(rq)) != NULL)
		__blk_end_request_all(req, -ENODEV);
	spin_unlock_irq(rq->queue_lock);
	complete_and_exit(&sfc_ops->thread_exit, 0);
	return 0;
}

static void sfc_blk_request(struct request_queue *rq)
{
	struct sfc_blk_ops *sfc_ops = rq->queuedata;
	struct request *req = NULL;

	if (sfc_ops->sfc_th_quited) {
		while ((req = blk_fetch_request(rq)) != NULL)
			__blk_end_request_all(req, -ENODEV);
		return;
	}
	wake_up(&sfc_ops->thread_wq);
}

static int rksfc_open(struct block_device *bdev, fmode_t mode)
{
	return 0;
}

static void rksfc_release(struct gendisk *disk, fmode_t mode)
{
};

#define DISABLE_WRITE _IO('V', 0)
#define ENABLE_WRITE _IO('V', 1)
#define DISABLE_READ _IO('V', 2)
#define ENABLE_READ _IO('V', 3)
static int rksfc_ioctl(struct block_device *bdev, fmode_t mode,
		       unsigned int cmd,
		       unsigned long arg)
{
	struct sfc_blk_dev *dev = bdev->bd_disk->private_data;

	switch (cmd) {
	case ENABLE_WRITE:
		dev->disable_access = 0;
		dev->readonly = 0;
		set_disk_ro(dev->blkcore_priv, 0);
		return 0;

	case DISABLE_WRITE:
		dev->readonly = 1;
		set_disk_ro(dev->blkcore_priv, 1);
		return 0;

	case ENABLE_READ:
		dev->disable_access = 0;
		dev->writeonly = 0;
		return 0;

	case DISABLE_READ:
		dev->writeonly = 1;
		return 0;
	default:
		return -ENOTTY;
	}
}

const struct block_device_operations sfc_blktrans_ops = {
	.owner = THIS_MODULE,
	.open = rksfc_open,
	.release = rksfc_release,
	.ioctl = rksfc_ioctl,
};

static struct sfc_blk_ops mytr = {
	.name =  "rksfc",
	.major = 31,
	.minorbits = 0,
	.owner = THIS_MODULE,
};

static int sfc_add_dev(struct sfc_blk_ops *sfc_ops, struct sfc_part *part)
{
	struct sfc_blk_dev *dev;
	struct gendisk *gd;

	if (part->size == 0)
		return -1;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	gd = alloc_disk(1 << sfc_ops->minorbits);
	if (!gd) {
		kfree(dev);
		return -ENOMEM;
	}

	dev->sfc_ops = sfc_ops;
	dev->size = part->size;
	dev->off_size = part->offset;
	dev->devnum = sfc_ops->last_dev_index;
	list_add_tail(&dev->list, &sfc_ops->devs);
	sfc_ops->last_dev_index++;

	gd->major = sfc_ops->major;
	gd->first_minor = (dev->devnum) << sfc_ops->minorbits;
	gd->fops = &sfc_blktrans_ops;

	if (part->name[0])
		snprintf(gd->disk_name,
			 sizeof(gd->disk_name),
			 "%s",
			 part->name);
	else
		snprintf(gd->disk_name,
			 sizeof(gd->disk_name),
			 "%s%d",
			 sfc_ops->name,
			 dev->devnum);

	set_capacity(gd, dev->size);

	gd->private_data = dev;
	dev->blkcore_priv = gd;
	gd->queue = sfc_ops->rq;
	gd->queue->bypass_depth = 1;

	if (part->type == PART_NO_ACCESS)
		dev->disable_access = 1;

	if (part->type == PART_READONLY)
		dev->readonly = 1;

	if (part->type == PART_WRITEONLY)
		dev->writeonly = 1;

	if (dev->readonly)
		set_disk_ro(gd, 1);

	add_disk(gd);

	return 0;
}

static int sfc_remove_dev(struct sfc_blk_dev *dev)
{
	struct gendisk *gd;

	gd = dev->blkcore_priv;
	list_del(&dev->list);
	gd->queue = NULL;
	del_gendisk(gd);
	put_disk(gd);
	kfree(dev);
	return 0;
}


static int sfc_blk_register(struct sfc_blk_ops *sfc_ops)
{
	struct task_struct *tsk;
	int i, ret;
	int offset;

	sfc_ops->quit = 0;
	sfc_ops->sfc_th_quited = 0;

	mtd_read_temp_buffer = kmalloc(MTD_RW_SECTORS * 512,
				       GFP_KERNEL | GFP_DMA);

	ret = register_blkdev(sfc_ops->major, sfc_ops->name);
	if (ret)
		return -1;

	spin_lock_init(&sfc_ops->queue_lock);
	init_completion(&sfc_ops->thread_exit);
	init_waitqueue_head(&sfc_ops->thread_wq);
	rksfc_device_lock_init();

	sfc_ops->rq = blk_init_queue(sfc_blk_request, &sfc_ops->queue_lock);
	if (!sfc_ops->rq) {
		unregister_blkdev(sfc_ops->major, sfc_ops->name);
		return  -1;
	}

	blk_queue_max_hw_sectors(sfc_ops->rq, MTD_RW_SECTORS);
	blk_queue_max_segments(sfc_ops->rq, MTD_RW_SECTORS);

	queue_flag_set_unlocked(QUEUE_FLAG_DISCARD, sfc_ops->rq);
	blk_queue_max_discard_sectors(sfc_ops->rq, UINT_MAX >> 9);

	sfc_ops->rq->queuedata = sfc_ops;
	INIT_LIST_HEAD(&sfc_ops->devs);
	tsk = kthread_run(sfc_blktrans_thread, (void *)sfc_ops, "rksfc");
	g_max_part_num = rk_partition_init(disk_array);
	offset = 0;
	sfc_ops->last_dev_index = 0;
	/* partition 0 is save vendor data, need hidden */
	for (i = 1; i < g_max_part_num; i++) {
		pr_info("%10s: 0x%09llx -- 0x%09llx (%llu MB)\n",
			disk_array[i].name,
			(u64)disk_array[i].offset * 512,
			(u64)(disk_array[i].offset + disk_array[i].size) * 512,
			(u64)disk_array[i].size / 2048);
		sfc_add_dev(sfc_ops, &disk_array[i]);
	}

	rksfc_create_procfs();
	return 0;
}

static void sfc_blk_unregister(struct sfc_blk_ops *sfc_ops)
{
	struct list_head *this, *next;

	sfc_ops->quit = 1;
	wake_up(&sfc_ops->thread_wq);
	wait_for_completion(&sfc_ops->thread_exit);
	list_for_each_safe(this, next, &sfc_ops->devs) {
		struct sfc_blk_dev *dev
			= list_entry(this, struct sfc_blk_dev, list);

		sfc_remove_dev(dev);
	}
	blk_cleanup_queue(sfc_ops->rq);
	unregister_blkdev(sfc_ops->major, sfc_ops->name);
}

int  rksfc_dev_init(void __iomem *reg_addr)
{
	int ret;

	pr_err("rksfc_dev_init\n");
	ret = spi_flash_init(reg_addr);
	if (ret) {
		pr_err("spi_flash_init fail\n");
		return -1;
	}

	ret = sfc_blk_register(&mytr);
	if (ret) {
		pr_err("sfc_blk_register fail\n");
		return -1;
	}

	rk_sfc_dev_initialised = 1;
	return ret;
}

int rksfc_dev_exit(void)
{
	if (rk_sfc_dev_initialised) {
		rk_sfc_dev_initialised = 0;
		sfc_blk_unregister(&mytr);
		pr_info("rksfc_dev_exit:OK\n");
	}
	return 0;
}

void rksfc_dev_shutdown(void)
{
	pr_info("rksfc_shutdown...\n");
	if (mytr.quit == 0) {
		mytr.quit = 1;
		wake_up(&mytr.thread_wq);
		wait_for_completion(&mytr.thread_exit);
		/* rk_ftl_de_init(); */
	}
	pr_info("rksfc_shutdown:OK\n");
}

