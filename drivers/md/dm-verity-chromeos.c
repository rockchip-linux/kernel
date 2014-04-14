/*
 * Copyright (C) 2010 The Chromium OS Authors <chromium-os-dev@chromium.org>
 *                    All Rights Reserved.
 *
 * This file is released under the GPL.
 */
/*
 * Implements a Chrome OS platform specific error handler.
 * When verity detects an invalid block, this error handling will
 * attempt to corrupt the kernel boot image. On reboot, the bios will
 * detect the kernel corruption and switch to the alternate kernel
 * and root file system partitions.
 *
 * Assumptions:
 * 1. Partitions are specified on the command line using uuid.
 * 2. The kernel partition is the partition number is one less
 *    than the root partition number.
 */
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/chromeos_platform.h>
#include <linux/device.h>
#include <linux/device-mapper.h>
#include <linux/err.h>
#include <linux/genhd.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/notifier.h>
#include <linux/string.h>
#include <asm/page.h>

#include "dm-verity.h"

#define DM_MSG_PREFIX "verity-chromeos"

static void chromeos_invalidate_kernel_endio(struct bio *bio, int err)
{
	const char *mode = ((bio->bi_rw & REQ_WRITE) ? "write" : "read");
	if (err)
		chromeos_set_need_recovery();

	if (bio_flagged(bio, BIO_EOPNOTSUPP)) {
		DMERR("invalidate_kernel: %s not supported", mode);
		chromeos_set_need_recovery();
	} else if (!bio_flagged(bio, BIO_UPTODATE)) {
		DMERR("invalidate_kernel: %s not up to date", mode);
		chromeos_set_need_recovery();
	} else {
		DMERR("invalidate_kernel: partition header %s completed", mode);
	}

	complete(bio->bi_private);
}

static int chromeos_invalidate_kernel_submit(struct bio *bio,
					     struct block_device *bdev,
					     int rw, struct page *page)
{
	DECLARE_COMPLETION_ONSTACK(wait);

	bio->bi_private = &wait;
	bio->bi_end_io = chromeos_invalidate_kernel_endio;
	bio->bi_bdev = bdev;

	bio->bi_iter.bi_sector = 0;
	bio->bi_vcnt = 1;
	bio->bi_iter.bi_idx = 0;
	bio->bi_iter.bi_size = 512;
	bio->bi_iter.bi_bvec_done = 0;
	bio->bi_rw = rw;
	bio->bi_io_vec[0].bv_page = page;
	bio->bi_io_vec[0].bv_len = 512;
	bio->bi_io_vec[0].bv_offset = 0;

	submit_bio(rw, bio);
	/* Wait up to 2 seconds for completion or fail. */
	if (!wait_for_completion_timeout(&wait, msecs_to_jiffies(2000)))
		return -1;
	return 0;
}

static char *get_info_from_cmdline(const char *key, char *uuid, int maxlen)
{
	const char *dev;
	const char *end;
	int len;

	dev = strstr(saved_command_line, key);
	if (!dev)
		return NULL;
	len = strlen(key);
	dev = &dev[len];
	end = strchr(dev, ' ');
	len = end - dev;
	if (len >= maxlen)
		return NULL;
	memcpy(uuid, dev, len);
	uuid[len] = '\0';
	return uuid;
}

static dev_t get_boot_dev_from_root_dev(struct block_device *root_bdev)
{
	/* Very basic sanity checking. This should be better. */
	if (!root_bdev || !root_bdev->bd_part ||
	    MAJOR(root_bdev->bd_dev) == 254 ||
	    root_bdev->bd_part->partno <= 1) {
		return 0;
	}
	return MKDEV(MAJOR(root_bdev->bd_dev), MINOR(root_bdev->bd_dev) - 1);
}

/* get_boot_dev is bassed on dm_get_device_by_uuid in dm_bootcache. */
static dev_t get_boot_dev(void)
{
	const char partuuid[] = "PARTUUID=";
	char uuid[2 * sizeof(partuuid) + 36];	/* Room for 2 PARTUUIDs */
	char *uuid_str;
	dev_t devt = 0;

	uuid_str = get_info_from_cmdline(" kern_guid=",
			&uuid[sizeof(partuuid) - 1],
			sizeof(uuid) - sizeof(partuuid));
	if (!uuid_str) {
		DMERR("Couldn't get uuid, try root dev");
		return 0;
	}

	if (strncmp(uuid_str, partuuid, strlen(partuuid)) != 0) {
		/* Not prefixed with "PARTUUID=", so add it */
		memcpy(uuid, partuuid, sizeof(partuuid) - 1);
		uuid_str = uuid;
	}
	devt = name_to_dev_t(uuid_str);
	if (!devt)
		goto found_nothing;
	return devt;

found_nothing:
	DMDEBUG("No matching partition for GUID: %s", uuid_str);
	return 0;
}

/* Replaces the first 8 bytes of a partition with DMVERROR */
static int chromeos_invalidate_kernel(struct block_device *root_bdev)
{
	int ret = 0;
	struct block_device *bdev;
	struct bio *bio;
	struct page *page;
	dev_t devt;
	fmode_t dev_mode;
	/* Ensure we do synchronous unblocked I/O. We may also need
	 * sync_bdev() on completion, but it really shouldn't.
	 */
	int rw = REQ_SYNC | REQ_SOFTBARRIER | REQ_NOIDLE;

	devt = get_boot_dev_from_root_dev(root_bdev);
	if (!devt) {
		devt = get_boot_dev();
		if (!devt)
			return -EINVAL;
	}

	/* First we open the device for reading. */
	dev_mode = FMODE_READ | FMODE_EXCL;
	bdev = blkdev_get_by_dev(devt, dev_mode, chromeos_invalidate_kernel);
	if (IS_ERR(bdev)) {
		DMERR("invalidate_kernel: could not open device for reading");
		dev_mode = 0;
		ret = -1;
		goto failed_to_read;
	}

	bio = bio_alloc(GFP_NOIO, 1);
	if (!bio) {
		ret = -1;
		goto failed_bio_alloc;
	}

	page = alloc_page(GFP_NOIO);
	if (!page) {
		ret = -ENOMEM;
		goto failed_to_alloc_page;
	}

	if (chromeos_invalidate_kernel_submit(bio, bdev, rw, page)) {
		ret = -1;
		goto failed_to_submit_read;
	}

	/* We have a page. Let's make sure it looks right. */
	if (memcmp("CHROMEOS", page_address(page), 8)) {
		DMERR("invalidate_kernel called on non-kernel partition");
		ret = -EINVAL;
		goto invalid_header;
	} else {
		DMERR("invalidate_kernel: found CHROMEOS kernel partition");
	}

	/* Stamp it and rewrite */
	memcpy(page_address(page), "DMVERROR", 8);

	/* The block dev was being changed on read. Let's reopen here. */
	blkdev_put(bdev, dev_mode);
	dev_mode = FMODE_WRITE | FMODE_EXCL;
	bdev = blkdev_get_by_dev(devt, dev_mode, chromeos_invalidate_kernel);
	if (IS_ERR(bdev)) {
		DMERR("invalidate_kernel: could not open device for reading");
		dev_mode = 0;
		ret = -1;
		goto failed_to_write;
	}

	/* We re-use the same bio to do the write after the read. Need to reset
	 * it to initialize bio->bi_remaining.
	 */
	bio_reset(bio);

	rw |= REQ_WRITE;
	if (chromeos_invalidate_kernel_submit(bio, bdev, rw, page)) {
		ret = -1;
		goto failed_to_submit_write;
	}

	DMERR("invalidate_kernel: completed.");
	ret = 0;
failed_to_submit_write:
failed_to_write:
invalid_header:
	__free_page(page);
failed_to_submit_read:
	/* Technically, we'll leak a page with the pending bio, but
	 *  we're about to panic so it's safer to do the panic() we expect.
	 */
failed_to_alloc_page:
	bio_put(bio);
failed_bio_alloc:
	if (dev_mode)
		blkdev_put(bdev, dev_mode);
failed_to_read:
	return ret;
}

static int error_handler(struct notifier_block *nb, unsigned long transient,
			 void *opaque_err)
{
	struct dm_verity_error_state *err =
		(struct dm_verity_error_state *) opaque_err;
	err->behavior = DM_VERITY_ERROR_BEHAVIOR_PANIC;
	if (transient)
		return 0;

	/* TODO(wad) Implement phase 2:
	 * - Attempt to read the dev_status_offset from the hash dev.
	 * - If the status offset is 0, replace the first byte of the sector
	 *   with 01 and panic().
	 * - If the status offset is not 0, invalidate the associated kernel
	 *   partition, then reboot.
	 * - make user space tools clear the last sector
	 */
	if (chromeos_invalidate_kernel(err->dev))
		chromeos_set_need_recovery();
	return 0;
}

static struct notifier_block chromeos_nb = {
	.notifier_call = &error_handler,
	.next = NULL,
	.priority = 1,
};

static int __init dm_verity_chromeos_init(void)
{
	int r;

	r = dm_verity_register_error_notifier(&chromeos_nb);
	if (r < 0)
		DMERR("failed to register handler: %d", r);
	else
		DMINFO("dm-verity-chromeos registered");
	return r;
}

static void __exit dm_verity_chromeos_exit(void)
{
	dm_verity_unregister_error_notifier(&chromeos_nb);
}

module_init(dm_verity_chromeos_init);
module_exit(dm_verity_chromeos_exit);

MODULE_AUTHOR("Will Drewry <wad@chromium.org>");
MODULE_DESCRIPTION("chromeos-specific error handler for dm-verity");
MODULE_LICENSE("GPL");
