/*
 * Copyright (C) 2011-2014 Imagination Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * AXD is a hardware IP that provides various audio processing capabilities for
 * user applications, offloading the core on which the application is running
 * and saving its valuable MIPS.
 */
#include <linux/axd.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kdev_t.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include "axd_cmds.h"
#include "axd_cmds_internal.h"
#include "axd_hdr.h"
#include "axd_module.h"
#include "axd_platform.h"
#include "axd_sysfs.h"
#include "axd_ts_driver.h"

#define AXD_NAME		"axd"

#define AXD_MGCNUM		0x66445841	/* AXDf */
#define LZO_MGCNUM		0x4f5a4c89	/* .LZO */

#define AXD_LDFW_RETRIES	400

#define WATCHDOG_TIMEOUT	(3*HZ)

/* enums/structs */
enum axd_devtype {
	AXD_UNKNOWN = 0,
	AXD_CTRL,
	AXD_INPUT,
	AXD_OUTPUT,
};

#define SYNC_MGCNUM 0x7FFFFFFF80000000ull

struct axd_sync_data {
	u64 magic;
	u64 pts_us;
};

/* functions start here */
static int minor_to_devtype(unsigned int minor)
{
	if (minor < MAX_CTRL_DEVICES)
		return AXD_CTRL;
	else if (minor < (MAX_IN_DEVICES + MAX_CTRL_DEVICES))
		return AXD_INPUT;
	else if (minor < MAX_NUM_DEVICES)
		return AXD_OUTPUT;
	return AXD_UNKNOWN;
}

/* set the presentation time stamp (pts) for the buffer to be sent next */
static void set_next_pts(struct axd_dev *axd, unsigned int pipe, u64 pts)
{
	int ret;

	if (!axd_get_flag(&axd->cmd.started_flg)) {
		if (axd_ts_reset)
			axd_ts_reset();
		axd_set_flag(&axd->cmd.started_flg, 1);
	}

	if (axd_ts_adjust) {
		ret = axd_ts_adjust(&pts);
		if (ret)
			dev_err(axd->dev, "Timestamp adjust failed\n");
	}

	axd->cmd.in_pipes[pipe].current_ts_high = pts >> 32;
	axd->cmd.in_pipes[pipe].current_ts_low = pts & 0xffffffff;
}

/*
 * note if we plan to support more than 1 AXD instance this will need to become
 * an array indexed by device id.
 */
static struct axd_dev *__axd;

/*
 * only a single process can open an input/output device node at a time. And
 * only that process can release that device node.
 *
 * semaphores ensure this behaviour.
 */
static int axd_open(struct inode *inode, struct file *filp)
{
	struct axd_dev *axd = container_of(inode->i_cdev, struct axd_dev, cdev);
	unsigned int minor = MINOR(inode->i_rdev);
	int type = minor_to_devtype(minor);
	int ret;
	int i;

	/* save the inode for other methods */
	filp->private_data = inode;

	if (axd_get_flag(&axd->cmd.fw_stopped_flg))
		return -EAGAIN;

	switch (type) {
	case AXD_CTRL:
		/* nothing to do in here */
		break;
	case AXD_INPUT:
		if ((filp->f_flags & O_ACCMODE) != O_WRONLY)
			return -EPERM;

		axd->cmd.nonblock = filp->f_flags & O_NONBLOCK;

		ret = down_trylock(&axd->input_locks[MINOR_TO_INPUT(minor)]);
		if (ret)
			return -EBUSY;

		/* Are any pipes running? */
		for (i = 0; i < AXD_MAX_PIPES; i++) {
			if (axd_cmd_inpipe_active(&axd->cmd, i))
				goto pipes_running;
		}

		/* Invalidate any clock tracking from previous use */
		axd_set_flag(&axd->cmd.started_flg, 0);
pipes_running:

		ret = axd_cmd_inpipe_start(&axd->cmd, MINOR_TO_INPUT(minor));
		if (ret) {
			up(&axd->input_locks[MINOR_TO_INPUT(minor)]);
			return ret;
		}

		break;
	case AXD_OUTPUT:
		if ((filp->f_flags & O_ACCMODE) != O_RDONLY)
			return -EPERM;

		axd->cmd.nonblock = filp->f_flags & O_NONBLOCK;

		ret = down_trylock(&axd->output_locks[MINOR_TO_OUTPUT(minor)]);
		if (ret)
			return -EBUSY;
		ret = axd_cmd_outpipe_start(&axd->cmd, MINOR_TO_OUTPUT(minor));
		if (ret) {
			up(&axd->output_locks[MINOR_TO_OUTPUT(minor)]);
			return ret;
		}
		break;
	default:
		dev_err(axd->dev, "Unknown device type\n");
		return -EINVAL;
	}
	return 0;
}

static int axd_release(struct inode *inode, struct file *filp)
{
	struct axd_dev *axd = container_of(inode->i_cdev, struct axd_dev, cdev);
	unsigned int minor = MINOR(inode->i_rdev);
	int type = minor_to_devtype(minor);

	switch (type) {
	case AXD_CTRL:
		/* nothing to do in here */
		break;
	case AXD_INPUT:
		axd_cmd_inpipe_stop(&axd->cmd, MINOR_TO_INPUT(minor));
		up(&axd->input_locks[MINOR_TO_INPUT(minor)]);
		break;
	case AXD_OUTPUT:
		axd_cmd_outpipe_stop(&axd->cmd, MINOR_TO_OUTPUT(minor));
		up(&axd->output_locks[MINOR_TO_OUTPUT(minor)]);
		break;
	default:
		dev_err(axd->dev, "Unknown device type\n");
		return -EINVAL;
	}
	return 0;
}

static ssize_t axd_read_log(struct axd_dev *axd,
				char __user *buff, size_t count, loff_t *offp)
{
	void __iomem *log_addr;
	unsigned int log_size;
	static char *rbuf;
	static int rbuf_rem;
	int ret;

	log_addr = axd->fw_base_m + axd_hdr_get_log_offset();
	log_size = ioread32(log_addr+4);

	if (!rbuf) {
		/*
		 * first time we run, initialise
		 *
		 * TODO: should we free this? In normal operation this wouldn't
		 * be allocated, only if the user asked to print a log.
		 * Constantly allocating and freeing could cause fragmentation
		 * maybe..
		 */
		dev_dbg(axd->ctrldev[0],
			"allocating %u bytes for log buffer\n", log_size);
		rbuf = kzalloc(log_size, GFP_KERNEL);
		if (!rbuf)
			return -ENOMEM;
	}

	if (!*offp) {
		unsigned int flags = axd_platform_lock();
		unsigned int log_offset = ioread32(log_addr);
		unsigned int log_wrapped = ioread32(log_addr+8);
		char __iomem *log_buff = (char __iomem *)(log_addr+12);

		/* new read from beginning, fill up our internal buffer */
		if (!log_wrapped) {
			memcpy_fromio(rbuf, log_buff, log_offset);
			rbuf_rem = log_offset;
		} else {
			char __iomem *pos = log_buff + log_offset;
			unsigned int rem = log_size - log_offset;

			memcpy_fromio(rbuf, pos, rem);
			memcpy_fromio(rbuf + rem, log_buff, log_offset);
			rbuf_rem = log_size;
		}
		axd_platform_unlock(flags);
	}

	if (count > rbuf_rem)
		count = rbuf_rem;

	ret = copy_to_user(buff, rbuf + *offp, count);
	if (ret < 0)
		return ret;

	dev_dbg(axd->ctrldev[0], "read %d bytes from %d\n", count, (int)*offp);
	*offp += count;
	rbuf_rem -= count;

	return count;
}

static ssize_t axd_read(struct file *filp, char __user *buff, size_t count,
								loff_t *offp)
{
	struct inode *inode = filp->private_data;
	struct axd_dev *axd = container_of(inode->i_cdev, struct axd_dev, cdev);
	unsigned int minor = MINOR(inode->i_rdev);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	ssize_t read = 0;

	if (axd_get_flag(&axd->cmd.fw_stopped_flg))
		return 0;

	/* read the log when it's the ctrl device */
	if (!minor)
		return axd_read_log(axd, buff, count, offp);

	if (axd_get_flag(&axd->timestamps_out_flg)) {
		copy_to_user(buff, &axd->cmd.out_pipes[pipe].current_ts_low, 8);
		read += 8;
		buff += 8;
	}

	read += axd_cmd_recv_buffer(&axd->cmd, pipe, buff, count);
	if (read > 0)
		*offp += read;
	return read;
}

static ssize_t axd_write(struct file *filp, const char __user *buff,
						size_t count, loff_t *offp)
{
	struct inode *inode = filp->private_data;
	struct axd_dev *axd = container_of(inode->i_cdev, struct axd_dev, cdev);
	unsigned int minor = MINOR(inode->i_rdev);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	ssize_t written;
	struct axd_sync_data sync_data;

	if (axd_get_flag(&axd->cmd.fw_stopped_flg))
		return 0;

	/* can't write ctrl device */
	if (!minor)
		return count;

	if (count == sizeof(struct axd_sync_data)) {
		/* Read sync data */
		copy_from_user(&sync_data, buff, sizeof(sync_data));

		/* Validate sync data */
		if (sync_data.magic != SYNC_MGCNUM) {
			/* Not valid sync data -- must be normal stream data */
			goto stream_data;
		}

		set_next_pts(axd, pipe, sync_data.pts_us);
		written = count;
	} else {
stream_data:
		written = axd_cmd_send_buffer(&axd->cmd, pipe, buff, count);
	}

	if (written > 0)
		*offp += written;
	return written;
}

static const struct file_operations axd_fops = {
	.owner	= THIS_MODULE,
	.open	= axd_open,
	.read	= axd_read,
	.write	= axd_write,
	.release = axd_release,
};

static int axd_create_chrdev(struct cdev *cdev,
				const struct file_operations *fops, char *name)
{
	dev_t devno;
	int ret;

	ret = alloc_chrdev_region(&devno, 0, MAX_NUM_DEVICES, name);
	if (ret < 0)
		goto alloc_err;
	cdev_init(cdev, fops);
	ret = cdev_add(cdev, devno, MAX_NUM_DEVICES);
	if (ret)
		goto add_err;
	return 0;
add_err:
	unregister_chrdev_region(devno, MAX_NUM_DEVICES);
alloc_err:
	return ret;
}

static void axd_destroy_chrdev(struct cdev *cdev)
{
	dev_t devno = cdev->dev;

	cdev_del(cdev);
	unregister_chrdev_region(devno, MAX_NUM_DEVICES);
}

#ifdef CONFIG_CRYPTO_LZO
#include <linux/crypto.h>
static int decompress_fw(struct axd_dev *axd, const struct firmware *fw)
{
	struct crypto_comp *tfm;
	unsigned int size;
	unsigned int fw_size = axd->fw_size;
	char *cached_fw_base;
	int ret = 0, i = 5;

	tfm = crypto_alloc_comp("lzo", 0, 0);
	if (IS_ERR(tfm)) {
		ret = -EIO;
		goto out;
	}

	do {
		/* allocate bigger memory for uncompressed fw */
		dma_free_coherent(axd->dev, axd->fw_size,
					axd->fw_base_m, axd->fw_base_p);
		axd->fw_size = fw_size * i;
		axd->fw_base_m = dma_alloc_coherent(axd->dev, axd->fw_size,
						&axd->fw_base_p, GFP_KERNEL);
		if (!axd->fw_base_m) {
			ret = -ENOMEM;
			break;
		}

		/* first 4 bytes contain lzo magic number, skip them */
		size = axd->fw_size;
		cached_fw_base = (char *)((int)axd->fw_base_m & ~0x20000000);
		ret = crypto_comp_decompress(tfm, fw->data+4,
					fw->size-4, cached_fw_base, &size);

		if (ret)
			i++;
	} while (ret && i < 10);

	if (ret)
		dev_err(axd->dev, "Failed to decompress the firmware\n");

	crypto_free_comp(tfm);
out:
	return ret;
}
#else /* !CONFIG_CRYPTO_LZO */
static int decompress_fw(struct axd_dev *axd, const struct firmware *fw)
{
	dev_err(axd->dev, "The firmware must be lzo decompressed first, compile driver again with CONFIG_CRYPTO_LZO enabled in kernel or do the decompression in user space.\n");
	return -EIO;
}
#endif /* CONFIG_CRYPTO_LZO */

static int copy_fw(struct axd_dev *axd, const struct firmware *fw)
{
	int mgcnum = *(int *)fw->data;
	int cached_fw_base = (int)axd->fw_base_m & ~0x20000000;

	if (mgcnum != AXD_MGCNUM) {
		if (mgcnum == LZO_MGCNUM)
			return decompress_fw(axd, fw);

		dev_err(axd->dev, "Not a valid firmware binary.\n");
		return -EIO;
	}

	/*
	 * We copy through the cache, fw will do the necessary cache
	 * flushes and syncing at startup.
	 * Copying from uncached makes it more difficult for the
	 * firmware to keep the caches coherent with memory when it sets
	 * tlbs and start running.
	 */
	memcpy_toio((void *)cached_fw_base, fw->data, fw->size);

	/* TODO: do MD5 checksum verification */
	return 0;
}

static void axd_free(struct axd_dev *axd)
{
	if (axd->buf_base_m) {
		dma_free_noncoherent(axd->dev, axd->inbuf_size+axd->outbuf_size,
					axd->buf_base_m, axd->buf_base_p);
		axd->buf_base_m = NULL;
	}
	if (axd->fw_base_m) {
		dma_free_coherent(axd->dev, axd->fw_size,
					axd->fw_base_m, axd->fw_base_p);
		axd->fw_base_m = NULL;
	}
}

static int axd_alloc(struct axd_dev *axd)
{
	/* do the allocation once, return immediately if fw_base_m is set */
	if (axd->fw_base_m)
		return 0;

	axd->fw_base_m = dma_alloc_coherent(axd->dev, axd->fw_size,
						&axd->fw_base_p, GFP_KERNEL);
	if (!axd->fw_base_m)
		return -ENOMEM;

	axd->buf_base_m = dma_alloc_noncoherent(axd->dev,
			axd->inbuf_size+axd->outbuf_size,
			&axd->buf_base_p, GFP_KERNEL);
	if (!axd->buf_base_m) {
		axd_free(axd);
		return -ENOMEM;
	}
	return 0;
}

static int axd_fw_start(struct axd_dev *axd)
{
	unsigned long t0_new_pc;
	unsigned int num_threads = axd_platform_num_threads();
	struct axd_cmd *axd_cmd = &axd->cmd;
	const struct firmware *fw;
	int ret = 0, i;
	char axd_name[16];
	unsigned int gic_irq;

	snprintf(axd_name, 16, "%s%d", AXD_NAME, axd->id);

	/* request the firmware */
	ret = request_firmware(&fw, "axd_firmware.bin", axd->ctrldev[0]);
	if (ret) {
		dev_err(axd->dev, "Failed to load firmware, check that firmware loading is setup correctly in userspace and kernel and that axd_firmware.bin is present in the FS\n");
		goto out;
	}

	axd->fw_size = fw->size;
	if (!axd->inbuf_size)
		axd->inbuf_size = 0x7800;
	if (!axd->outbuf_size)
		axd->outbuf_size = 0x3c000;

	ret = axd_alloc(axd);
	if (ret) {
		dev_err(axd->dev, "Failed to allocate memory for AXD f/w and buffers\n");
		release_firmware(fw);
		goto out;
	}

	dev_info(axd->dev, "Loading firmware at 0x%p ...\n", axd->fw_base_m);

	ret = copy_fw(axd, fw);
	release_firmware(fw);
	if (ret)
		goto out;

	/* setup hdr and memmapped regs */
	axd_hdr_init((unsigned long)axd->fw_base_m);
	/* initialize the cmd structure and the buffers */
	axd_cmd_init(axd_cmd,
		axd_hdr_get_cmdblock_offset()+(unsigned long)axd->fw_base_m,
		(unsigned long)axd->buf_base_m, axd->buf_base_p);

	/*
	 * Tell AXD the frequency at which it's running and the IRQs
	 */
	gic_irq = (axd->host_irq << 16) | axd->axd_irq;
	iowrite32(gic_irq, &axd_cmd->message->gic_irq);
	iowrite32(clk_get_rate(axd->clk)/1000000, &axd_cmd->message->freq);

	axd_platform_init(axd);
	for (i = 0; i < num_threads; i++) {
		ret = axd_cmd_set_pc(axd_cmd, i, axd_hdr_get_pc(i));
		if (ret == -1) {
			dev_err(axd->dev, "Failed to set PC of T%d\n", i);
			goto out;
		}
	}
	/* setup and start master thread */
	t0_new_pc = axd_hdr_get_pc(0);
	if (t0_new_pc == -1UL) {
		ret = -1;
		goto out;
	}
	t0_new_pc = (unsigned long) axd->fw_base_m + (t0_new_pc - 0xD0000000);
	axd_platform_set_pc(t0_new_pc);
	ret = axd_platform_start();
	if (ret)
		goto out;

	/* install the IRQ */
	ret = axd_cmd_install_irq(&axd->cmd, axd->irqnum);
	if (ret) {
		dev_err(axd->dev, "Failed to install IRQ %d, error %d\n",
							axd->irqnum, ret);
		goto out;
	}

	for (i = 0; i < AXD_LDFW_RETRIES; i++) {
		ret = axd_wait_ready(axd_cmd->message);
		if (!ret) {
			/*
			 * Let the firmware know the address of the buffer
			 * region
			 */
			ret = axd_write_reg(axd_cmd,
					AXD_REG_BUFFER_BASE, axd->buf_base_p);
			if (ret) {
				dev_err(axd->dev,
					"Failed to setup buffers base address\n");
				goto out;
			}
			return 0;

		}
	}
out:
	axd_free(axd);
	return ret;
}

static void axd_fw_stop(struct axd_dev *axd)
{
	axd_cmd_free_irq(&axd->cmd, axd->irqnum);
	axd_platform_stop();
}

/*
 * Stops the firmware, reload it, and start it back again to recover from a
 * fatal error.
 */
static void axd_reset(struct work_struct *work)
{
	unsigned int major, minor, patch;
	int i;

	struct axd_dev *axd = container_of(work, struct axd_dev, watchdogwork);


	/* if we got a fatal error, don't reset if watchdog is disabled */
	if (unlikely(!axd->cmd.watchdogenabled))
		return;

	/* stop the watchdog timer until we restart */
	del_timer(&axd->watchdogtimer);

	if (!axd_get_flag(&axd->cmd.fw_stopped_flg)) {
		/* ping the firmware by requesting its version info */
		axd_cmd_get_version(&axd->cmd, &major, &minor, &patch);
		if (!major && !minor && !patch) {
			dev_warn(axd->dev, "Firmware stopped responding...\n");
			axd_set_flag(&axd->cmd.fw_stopped_flg, 1);
		} else {
			goto out;
		}
	}

	axd_platform_print_regs();
	dev_warn(axd->dev, "Reloading AXD firmware...\n");

	axd_fw_stop(axd);

	/* Signal to any active tasks first */
	for (i = 0; i < axd->num_inputs; i++) {
		if (down_trylock(&axd->input_locks[i])) {
			/* trylock failed, pipe in use */
			axd_cmd_send_buffer_abort(&axd->cmd, i);
		} else {
			/*
			 * Increment semaphore as succeeding down_trylock
			 * decremented it
			 */
			up(&axd->input_locks[i]);
		}
	}
	for (i = 0; i < axd->num_outputs; i++) {
		if (down_trylock(&axd->output_locks[i])) {
			/* trylock failed, pipe in use */
			axd_cmd_recv_buffer_abort(&axd->cmd, i);
		} else {
			/*
			 * Increment semaphore as succeeding down_trylock
			 * decremented it
			 */
			up(&axd->output_locks[i]);
		}
	}

	/* wake up any task sleeping on command response */
	wake_up(&axd->cmd.wait);
	/* give chance to user land tasks to react to the crash */
	ssleep(2);

	axd_fw_start(axd);

	for (i = 0; i < axd->num_inputs; i++) {
		if (down_trylock(&axd->input_locks[i]))
			axd_cmd_inpipe_reset(&axd->cmd, i);
		else
			up(&axd->input_locks[i]);
	}
	for (i = 0; i < axd->num_outputs; i++) {
		if (down_trylock(&axd->output_locks[i]))
			axd_cmd_outpipe_reset(&axd->cmd, i);
		else
			up(&axd->output_locks[i]);
	}

	axd_set_flag(&axd->cmd.fw_stopped_flg, 0);
out:
	axd->watchdogtimer.expires = jiffies + WATCHDOG_TIMEOUT;
	add_timer(&axd->watchdogtimer);
}

/*
 * Schedule to perform a reset.
 * We don't perform the reset directly because the request comes from atomic
 * context, and resetting must be done from process context.
 */
void axd_schedule_reset(struct axd_cmd *cmd)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);

	axd_set_flag(&axd->cmd.fw_stopped_flg, 1);
	schedule_work(&axd->watchdogwork);
}

/*
 * Verifies that the firmware is still running by reading the version every few
 * seconds.
 */
static void axd_watchdog_timer(unsigned long arg)
{
	struct axd_dev *axd = (struct axd_dev *)arg;

	/* skip if watchdog is not enabled */
	if (unlikely(!axd->cmd.watchdogenabled))
		goto out;

	schedule_work(&axd->watchdogwork);
	return;
out:
	mod_timer(&axd->watchdogtimer, jiffies + WATCHDOG_TIMEOUT);
}

static void axd_start_watchdog(struct axd_dev *axd)
{
	INIT_WORK(&axd->watchdogwork, axd_reset);
	init_timer(&axd->watchdogtimer);
	axd->watchdogtimer.function = axd_watchdog_timer;
	axd->watchdogtimer.data = (unsigned long)axd;
	axd->watchdogtimer.expires = jiffies + HZ;
	add_timer(&axd->watchdogtimer);
}

static void axd_stop_watchdog(struct axd_dev *axd)
{
	del_timer(&axd->watchdogtimer);
}

static int axd_create(struct axd_dev *axd, int id)
{
	struct cdev *cdev = &axd->cdev;
	struct device *device;
	int ret = 0, i = 0, j = 0;
	char axd_name[16];
	unsigned int major, minor, patch;

	snprintf(axd_name, 16, "%s%d", AXD_NAME, id);
	axd->id = id;

	axd_set_flag(&axd->timestamps_out_flg, 0);

	if (!axd->class) {
		/* Create a new class for AXD */
		axd->class = class_create(THIS_MODULE, AXD_NAME);
		if (IS_ERR(axd->class)) {
			ret = PTR_ERR(axd->class);
			dev_err(axd->dev, "Failed to create class, error %d\n",
									ret);
			goto class_err;
		}
	}

	/* Create a new char device with our own new Major Number */
	ret = axd_create_chrdev(cdev, &axd_fops, axd_name);
	if (ret) {
		dev_err(axd->dev, "Failed to create char device\n");
		goto chr_dev_err;
	}

	/*
	 * ctrl device mainly used to do mixer control.
	 *
	 * NOTE: We should create ctrl devices in a loop, but since it's
	 * unlikely we'll need more than 1, keep things simple until proved
	 * required.
	 */
	device = device_create(axd->class, NULL, CTRL_TO_DEVNO(cdev, 0), NULL,
							"%sctrl", axd_name);
	if (IS_ERR(device)) {
		ret = PTR_ERR(device);
		dev_err(axd->dev,
			"Failed to create ctrl device, error %d\n", ret);
		goto ctrl_dev_err;
	}
	device->platform_data = &axd->cmd;
	axd->ctrldev[0] = device;

	/* Setup and start the threads */
	ret = axd_fw_start(axd);
	if (ret) {
		dev_err(axd->dev, "Failed to start\n");
		ret = -EIO;
		goto fw_start_err;
	}

	/*
	 * Verify that the firmware is ready. In normal cases the firmware
	 * should start immediately, but to be more robust we do this
	 * verification and give the firmware a chance of 3 seconds to be ready
	 * otherwise we exit in failure.
	 */
	for (i = 0; i < AXD_LDFW_RETRIES; i++) {
		axd_cmd_get_version(&axd->cmd, &major, &minor, &patch);
		if (major || minor || patch) {
			/* firmware is ready */
			break;
		}
		/* if we couldn't read the version after 3 tries, error */
		if (i == AXD_LDFW_RETRIES-1) {
			dev_err(axd->dev, "Failed to communicate with the firmware\n");
			ret = -EIO;
			goto fw_start_err;
		}
		/* wait for 10 ms for the firmware to start */
		msleep(10);
	}
	dev_info(axd->dev, "Running firmware version %u.%u.%u %s\n",
				major, minor, patch, axd_hdr_get_build_str());

	/* Start watchdog timer */
	axd_start_watchdog(axd);

	/* Get num of input/output pipes */
	ret = axd_cmd_get_num_pipes(&axd->cmd,
					&axd->num_inputs, &axd->num_outputs);
	if (ret) {
		dev_err(axd->dev, "Failed to get numer of supported pipes\n");
		ret = -EIO;
		goto num_pipes_err;
	}
	axd->cmd.num_inputs = axd->num_inputs;
	axd->cmd.num_outputs = axd->num_outputs;

	/* Invalidate DCPP selector caches */
	for (i = 0; i < axd->cmd.num_outputs; i++) {
		axd->cmd.dcpp_channel_ctrl_cache[i] = -1;
		axd->cmd.dcpp_band_ctrl_cache[i] = -1;
	}

	/* Create input/output locks to control access to the devices */
	axd->input_locks = kcalloc(axd->num_inputs,
					sizeof(struct semaphore), GFP_KERNEL);
	if (!axd->input_locks) {
		ret = -ENOMEM;
		dev_err(axd->dev, "Couldn't create input locks\n");
		goto input_locks_err;
	}
	axd->output_locks = kcalloc(axd->num_outputs,
					sizeof(struct semaphore), GFP_KERNEL);
	if (!axd->output_locks) {
		ret = -ENOMEM;
		dev_err(axd->dev, "Couldn't create output locks\n");
		goto output_locks_err;
	}

	/* Setup sysfs for ctrl dev after f/w has started */
	ret = axd_ctrl_sysfs_add(device);
	if (ret) {
		dev_err(axd->ctrldev[0], "Failed to create sysfs entries\n");
		goto ctrl_sysfs_err;
	}

	/* Create input/output device nodes */
	for (i = 0; i < axd->num_inputs; i++) {
		device = device_create(axd->class, NULL,
					INPUT_TO_DEVNO(cdev, i), NULL,
					"%sinput%d", axd_name, i);
		if (IS_ERR(device)) {
			ret = PTR_ERR(device);
			dev_err(axd->dev, "Failed to create input%d, error %d\n",
									i, ret);
			goto input_dev_err;
		}
		device->platform_data = &axd->cmd;
		ret = axd_input_sysfs_add(device);
		if (ret) {
			dev_err(device, "Failed to create sysfs entries\n");
			goto input_sysfs_err;
		}
		axd->inputdev[i] = device;
		sema_init(&axd->input_locks[i], 1);
	}
	for (j = 0; j < axd->num_outputs; j++) {
		device = device_create(axd->class, NULL,
					OUTPUT_TO_DEVNO(cdev, j), NULL,
					"%soutput%d", axd_name, j);
		if (IS_ERR(device)) {
			ret = PTR_ERR(device);
			dev_err(axd->dev, "Failed to create output%d, error %d\n",
									j, ret);
			goto output_dev_err;
		}
		device->platform_data = &axd->cmd;
		ret = axd_output_sysfs_add(device);
		if (ret) {
			dev_err(device, "Failed to create sysfs entries\n");
			goto output_sysfs_err;
		}
		axd->outputdev[j] = device;
		sema_init(&axd->output_locks[j], 1);
	}

	dev_info(axd->dev, "Created\n");
	return 0;

output_sysfs_err:
	if (j < axd->num_outputs)
		device_destroy(axd->class, OUTPUT_TO_DEVNO(cdev, j));
output_dev_err:
	/* We got an error midst creating devices, clean up the ones that were
	 * successfully created only */
	for (j--; j >= 0; j--) {
		axd_output_sysfs_remove(axd->outputdev[j]);
		device_destroy(axd->class, OUTPUT_TO_DEVNO(cdev, j));
	}
input_sysfs_err:
	if (i < axd->num_inputs)
		device_destroy(axd->class, INPUT_TO_DEVNO(cdev, i));
input_dev_err:
	for (i--; i >= 0; i--) {
		axd_input_sysfs_remove(axd->inputdev[i]);
		device_destroy(axd->class, INPUT_TO_DEVNO(cdev, i));
	}
	axd_ctrl_sysfs_remove(axd->ctrldev[0]);
ctrl_sysfs_err:
	kfree(axd->output_locks);
output_locks_err:
	kfree(axd->input_locks);
input_locks_err:
num_pipes_err:
	axd_stop_watchdog(axd);
fw_start_err:
	axd_fw_stop(axd);
	device_destroy(axd->class, CTRL_TO_DEVNO(cdev, 0));
ctrl_dev_err:
	axd_destroy_chrdev(cdev);
chr_dev_err:
	class_destroy(axd->class);
class_err:
	return ret;
}

static int axd_init(struct device *dev)
{
	int ret;

	/* validate parameters */
	if (!__axd->irqnum) {
		dev_err(dev, "Must provide non-zero irq number\n");
		return -EINVAL;
	}

	if (!__axd->clk) {
		dev_err(dev, "Must provide non-zero clk\n");
		return -EINVAL;
	}

	__axd->dev = dev;
	ret = axd_create(__axd, 0);
	if (ret)
		return ret;
	return 0;
}

static void axd_destroy(struct axd_dev *axd)
{
	struct cdev *cdev = &axd->cdev;
	int count, i;

	axd_stop_watchdog(axd);
	axd_fw_stop(axd);
	count = axd->num_outputs;
	for (i = count-1; i >= 0; i--) {
		axd_output_sysfs_remove(axd->outputdev[i]);
		device_destroy(axd->class, OUTPUT_TO_DEVNO(cdev, i));
	}
	count = axd->num_inputs;
	for (i = count-1; i >= 0; i--) {
		axd_input_sysfs_remove(axd->inputdev[i]);
		device_destroy(axd->class, INPUT_TO_DEVNO(cdev, i));
	}
	axd_ctrl_sysfs_remove(axd->ctrldev[0]);
	device_destroy(axd->class, CTRL_TO_DEVNO(cdev, 0));
	kfree(axd->input_locks);
	kfree(axd->output_locks);
	axd_destroy_chrdev(cdev);
	class_destroy(axd->class);
	dev_info(axd->dev, "Removed\n");
}

static int axd_probe(struct platform_device *pdev)
{
	struct device_node *of_node = pdev->dev.of_node;
	struct axd_platform_config *axd_pconfig = pdev->dev.platform_data;
	int ret = -EINVAL;

	__axd = kzalloc(sizeof(struct axd_dev), GFP_KERNEL);
	if (!__axd)
		return -ENOMEM;

	__axd->irqnum = platform_get_irq(pdev, 0);
	if (__axd->irqnum < 0) {
		dev_err(&pdev->dev, "Couldn't get parameter: 'irq'\n");
		goto error;
	}

	if (of_node) {
		u32 val[2] = {0, 0};
		ret = of_property_read_u32_array(of_node, "gic-irq", val, 2);
		if (ret) {
			dev_warn(&pdev->dev,
					"Operating without GIC in SWT1 mode\n");
		} else {
			__axd->host_irq = val[0];
			__axd->axd_irq = val[1];
		}

		ret = of_property_read_u32(of_node, "vpe", val);
		if (!ret) {
			if (!val[0]) {
				dev_err(&pdev->dev, "'vpe' parameter can't be 0\n");
				goto error;
			}
			__axd->vpe = val[0];
		}

		__axd->clk = devm_clk_get(&pdev->dev, NULL);
		if (IS_ERR(__axd->clk)) {
			dev_err(&pdev->dev, "Couldn't get parameter: 'clocks'\n");
			goto error;
		}

		of_property_read_u32(of_node, "inbuf-size", &__axd->inbuf_size);
		of_property_read_u32(of_node, "outbuf-size", &__axd->outbuf_size);
	} else {
		if (!axd_pconfig) {
			dev_warn(&pdev->dev,
				"No valid platform config was provided\n");
			goto error;
		}
		__axd->host_irq = axd_pconfig->host_irq;
		__axd->axd_irq = axd_pconfig->axd_irq;
		__axd->clk = axd_pconfig->clk;
		__axd->inbuf_size = axd_pconfig->inbuf_size;
		__axd->outbuf_size = axd_pconfig->outbuf_size;

		if (IS_ERR_OR_NULL(__axd->clk)) {
			dev_err(&pdev->dev, "Must provide a valid clock\n");
			goto error;
		}
	}

	ret = clk_prepare_enable(__axd->clk);
	if (ret)
		goto error;

	ret =  axd_init(&pdev->dev);
	if (ret) {
		clk_disable_unprepare(__axd->clk);
		goto error;
	}

	return 0;
error:
	kfree(__axd);
	return ret;
}

static int axd_remove(struct platform_device *pdev)
{
	clk_disable_unprepare(__axd->clk);
	axd_destroy(__axd);
	axd_free(__axd);
	kfree(__axd);
	__axd = NULL;
	return 0;
}

static const struct of_device_id axd_match[] = {
	{ .compatible = "img,axd" },
	{}
};

static struct platform_driver axd_driver = {
	.driver = {
		.name		= "axd",
		.owner		= THIS_MODULE,
		.of_match_table	= axd_match,
	},
	.probe = axd_probe,
	.remove = axd_remove,
};

module_platform_driver(axd_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Imagination Technologies Ltd.");
MODULE_DESCRIPTION("AXD Audio Processing IP Driver");
