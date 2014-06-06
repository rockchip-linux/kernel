/*
 *  Copyright (C) 2012 The Chromium OS Authors
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
 */

#define pr_fmt(fmt) "chromeos_vbc_blk: " fmt

#include <linux/ide.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>

#include "chromeos.h"

static struct {
	bool initialized;
	phandle phandle;
	u64 lba;
	u16 offset, size;
} config;

static int match_of_node(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static struct block_device *vbc_blk_get_device(phandle phandle, fmode_t devmode)
{
	struct device_node *dn;
	struct device *dev;

	dn = of_find_node_by_phandle(phandle);
	if (!dn)
		return ERR_PTR(-ENODEV);

	dev = bus_find_device(&platform_bus_type, NULL, dn, match_of_node);
	if (!dev)
		return ERR_PTR(-ENODEV);

	/*
	 * TODO(chrome-os-partner:16441): Search block_device from the dev
	 * struct we just found instead of hard-coding major and minor here.
	 */
	return blkdev_get_by_dev(MKDEV(MMC_BLOCK_MAJOR, 0), devmode, NULL);
}

static void vbc_blk_endio(struct bio *bio, int err)
{
	struct completion *c = bio->bi_private;
	bio->bi_private = (void *)err;
	complete(c);
}

static void vbc_blk_submit_bio(struct bio *bio, int rq)
{
	DECLARE_COMPLETION_ONSTACK(wait);

	bio->bi_end_io	= vbc_blk_endio;
	bio->bi_private = &wait;
	submit_bio(rq, bio);
	wait_for_completion(&wait);
}

static int vbc_blk_access(struct page *page, sector_t sector, bool is_read)
{
	struct block_device *bdev;
	struct bio *bio;
	int err, rq;
	fmode_t devmode = is_read ? FMODE_READ : FMODE_WRITE;

	bdev = vbc_blk_get_device(config.phandle, devmode);
	if (IS_ERR(bdev)) {
		pr_err("could not open block dev\n");
		return PTR_ERR(bdev);
	}

	/* map the sector to page */
	bio = bio_alloc(GFP_NOIO, 1);
	if (!bio) {
		err = -ENOMEM;
		goto unwind_bdev;
	}
	bio->bi_bdev			= bdev;
	bio->bi_iter.bi_sector		= sector;
	bio->bi_iter.bi_idx		= 0;
	bio->bi_iter.bi_size		= SECTOR_SIZE;
	bio->bi_iter.bi_bvec_done	= 0;
	bio->bi_vcnt			= 1;
	bio->bi_io_vec[0].bv_page	= page;
	bio->bi_io_vec[0].bv_len	= SECTOR_SIZE;
	bio->bi_io_vec[0].bv_offset	= 0;

	/* submit bio */
	rq = REQ_SYNC | REQ_SOFTBARRIER | REQ_NOIDLE;
	if (!is_read)
		rq |= REQ_WRITE;

	vbc_blk_submit_bio(bio, rq);

	/* vbc_blk_endio passes up any error in bi_private */
	err = (int)bio->bi_private;
	bio_put(bio);

unwind_bdev:
	if (!is_read) {
		fsync_bdev(bdev);
		invalidate_bdev(bdev);
	}
	blkdev_put(bdev, devmode);

	return err;
}

/*
 * This function accepts a buffer with exactly config.size bytes. It reads the
 * appropriate mmc one sector block, extacts the nonvolatile data from there
 * and copies it to the provided buffer.
 */
static int vbc_blk_read(u8 *data)
{
	struct page *page;
	char *virtual_addr;
	int ret;

	page = alloc_page(GFP_NOIO);
	if (!page) {
		pr_err("page allocation failed\n");
		return -ENOMEM;
	}

	virtual_addr = page_address(page);
	if (!virtual_addr) {
		pr_err("page not mapped!\n");
		__free_page(page);
		return -EFAULT;
	}

	ret = vbc_blk_access(page, config.lba, 1);
	if (ret)
		goto out;

	memcpy(data, virtual_addr + config.offset, config.size);
	ret = config.size;

out:
	__free_page(page);
	return ret;
}

/*
 * This function accepts a buffer with exactly config.size bytes. It reads the
 * appropriate mmc one sector block, blends in the new vboot context contents
 * and then writes the sector back.
 */
static int vbc_blk_write(const u8 *data)
{
	struct page *page;
	char *virtual_addr;
	int ret;

	page = alloc_page(GFP_NOIO);
	if (!page) {
		pr_err("page allocation failed\n");
		return -ENOMEM;
	}

	virtual_addr = page_address(page);
	if (!virtual_addr) {
		pr_err("page not mapped!\n");
		__free_page(page);
		return -EFAULT;
	}

	ret = vbc_blk_access(page, config.lba, 1);
	if (ret)
		goto out;

	/*
	 * Sector has been read, lets blend in vboot context data and write
	 * the sector back.
	 */
	memcpy(virtual_addr + config.offset, data, config.size);

	ret = vbc_blk_access(page, config.lba, 0);
	if (!ret)
		ret = config.size;

out:
	__free_page(page);
	return ret;
}

static int __init vbc_blk_init(struct device_node *fw_dn)
{
	int err;
	u32 prop[4];

	err = of_property_read_u32_array(fw_dn, "chromeos-vbc-blk", prop,
			sizeof(prop) / sizeof(prop[0]));
	if (err) {
		pr_err("missing chromeos-vbc-blk property\n");
		goto err;
	}

	config.phandle = prop[0];
	config.lba = prop[1];
	config.offset = prop[2];
	config.size = prop[3];

	if ((config.offset + config.size > SECTOR_SIZE) ||
	    (config.size > MAX_VBOOT_CONTEXT_BUFFER_SIZE)) {
		/* vboot context block won't fit into a sector */
		pr_err("bad vboot context location: %d:%d!\n",
		       config.offset, config.size);
		err = -EINVAL;
		goto err;
	}

	config.initialized = true;
	err = 0;
err:
	return err;
}

static ssize_t chromeos_vbc_blk_read(void *buf, size_t count)
{
	if (!config.initialized) {
		pr_err("not initialized\n");
		return -ENODEV;
	}

	if (count < config.size) {
		pr_err("not enough room to read vboot context (%zd < %d)\n",
		       count, config.size);
		return -ENOSPC;
	}

	return vbc_blk_read(buf);
}

static ssize_t chromeos_vbc_blk_write(const void *buf, size_t count)
{
	if (!config.initialized) {
		pr_err("not initialized\n");
		return -ENODEV;
	}

	if (count != config.size) {
		pr_err("wrong write buffer size (%zd != %d)\n",
		       count, config.size);
		return -ENOSPC;
	}

	return vbc_blk_write(buf);
}

static struct chromeos_vbc chromeos_vbc_blk = {
	.name = "chromeos_vbc_blk",
	.read = chromeos_vbc_blk_read,
	.write = chromeos_vbc_blk_write,
};

static int __init chromeos_vbc_blk_init(void)
{
	struct device_node *of_node;
	const char *vbc_type;
	int err;

	of_node = of_find_compatible_node(NULL, NULL, "chromeos-firmware");
	if (!of_node)
		return -ENODEV;

	err = of_property_read_string(of_node, "nonvolatile-context-storage",
			&vbc_type);
	if (err)
		goto exit;

	if (strcmp(vbc_type, "disk")) {
		err = 0;  /* not configured to use vbc_blk, exit normally. */
		goto exit;
	}

	err = vbc_blk_init(of_node);
	if (err < 0)
		goto exit;

	err = chromeos_vbc_register(&chromeos_vbc_blk);
	if (err < 0)
		goto exit;

	err = 0;
exit:
	of_node_put(of_node);
	return err;
}
module_init(chromeos_vbc_blk_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ChromeOS vboot context on block device accessor");
