/*
 *  ChromeOS platform support code. Glue layer between higher level functions
 *  and per-platform firmware interfaces.
 *
 *  Copyright (C) 2011 The Chromium OS Authors
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

#define pr_fmt(fmt) "chromeos_arm: " fmt

#include <linux/chromeos_platform.h>
#include <linux/ide.h>
#include <linux/of.h>


#include "../chromeos.h"

/* TODO:
 * Do a proper search for the right mmc device to use
 */

#define BLKNV_MAJOR MMC_BLOCK_MAJOR
#define BLKNV_MINOR 0

/* location where the nvram data blends into the sector on the MMC device */
static u16 nv_offset, nv_size;
static u64 nv_lba;

/*
 * Functions to support nvram on block device. The actual device used is minor
 * 0 of MMC device class, the sector to use is as encoded in
 * firmware_shared_data->nvcxt_lba, the nvram buffer in the sector starts at
 * offset nv_offset and takes nv_size bytes.
 */
static void blknv_endio(struct bio *bio, int err)
{
	struct completion *c = bio->bi_private;
	bio->bi_private = (void *)err;
	complete(c);
}

static void blknv_submit_bio(struct bio *bio, int rq)
{
	DECLARE_COMPLETION_ONSTACK(wait);

	bio->bi_end_io	= blknv_endio;
	bio->bi_private = &wait;
	submit_bio(rq, bio);
	wait_for_completion(&wait);
}

static int chromeos_access_nvram_block(struct page *page,
				       sector_t sector,
				       bool is_read)
{
	struct block_device *bdev;
	struct bio *bio = NULL;
	dev_t mdev;
	fmode_t devmode = is_read ? FMODE_READ : FMODE_WRITE;
	int rq, ret;

	mdev = MKDEV(BLKNV_MAJOR, BLKNV_MINOR);
	bdev = blkdev_get_by_dev(mdev, devmode, NULL);
	if (IS_ERR(bdev)) {
		pr_err("could not open dev=[%d:%d]\n",
		       BLKNV_MAJOR, BLKNV_MINOR);
		ret = -EFAULT;
		goto out;
	}

	/* map the sector to page */
	bio = bio_alloc(GFP_NOIO, 1);
	if (!bio) {
		ret = -ENOMEM;
		goto out;
	}
	bio->bi_bdev	= bdev;
	bio->bi_sector	= sector;
	bio->bi_vcnt	= 1;
	bio->bi_idx	= 0;
	bio->bi_size	= SECTOR_SIZE;
	bio->bi_io_vec[0].bv_page	= page;
	bio->bi_io_vec[0].bv_len	= SECTOR_SIZE;
	bio->bi_io_vec[0].bv_offset	= 0;

	/* submit bio */
	rq = REQ_SYNC | REQ_SOFTBARRIER | REQ_NOIDLE;
	if (!is_read)
		rq |= REQ_WRITE;

	blknv_submit_bio(bio, rq);

	/* nvblk_endio passes up any error in bi_private */
	ret = (int)bio->bi_private;
out:
	if (bio)
		bio_put(bio);
	if (!is_read) {
		fsync_bdev(bdev);
		invalidate_bdev(bdev);
	}
	if (bdev)
		blkdev_put(bdev, devmode);
	return ret;
}

static int chromeos_read_nvram_block(struct page *page, sector_t sector)
{
	return chromeos_access_nvram_block(page, sector, 1);
}

static int chromeos_write_nvram_block(struct page *page, sector_t sector)
{
	return chromeos_access_nvram_block(page, sector, 0);
}

/*
 * This function accepts a buffer with exactly nv_size bytes. It reads the
 * appropriate mmc one sector block, extacts the nonvolatile data from there
 * and copies it to the provided buffer.
 */
static int _chromeos_read(u8 *data)
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

	ret = chromeos_read_nvram_block(page, nv_lba);
	if (ret)
		goto out;

	memcpy(data, virtual_addr + nv_offset, nv_size);
	ret = nv_size;

out:
	__free_page(page);
	return ret;
}

/*
 * This function accepts a buffer with exactly nv_size bytes. It reads the
 * appropriate mmc one sector block, blends in the new nvram contents and then
 * writes the sector back.
 */
static int _chromeos_write(const u8 *data)
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

	ret = chromeos_read_nvram_block(page, nv_lba);
	if (ret)
		goto out;

	/*
	 * Sector has been read, lets blend in nvram data and write the sector
	 * back.
	 */
	memcpy(virtual_addr + nv_offset, data, nv_size);

	ret = chromeos_write_nvram_block(page, nv_lba);
	if (!ret)
		ret = nv_size;

out:
	__free_page(page);
	return ret;
}

/*
 * Read the nvram buffer contents into the user provided space.
 *
 * returns number of bytes copied, or negative error.
 */
int chromeos_platform_read_nvram(u8 *nvram_buffer, int buf_size)
{
	if (!nv_size) {
		pr_err("%s nonvolatile context not configured!\n", __func__);
		return -ENODEV;
	}

	if (buf_size < nv_size) {
		pr_err("not enough room to read nvram (%d < %d)\n",
		       buf_size, nv_size);
		return -ENOSPC;
	}

	return _chromeos_read(nvram_buffer);
}

int chromeos_platform_write_nvram(u8 *nvram_buffer, int buf_size)
{
	if (!nv_size) {
		pr_err("%s nonvolatile context not configured!\n", __func__);
		return -ENODEV;
	}

	if (buf_size != nv_size) {
		pr_err("wrong write buffer size (%d != %d)\n",
		       buf_size, nv_size);
		return -ENOSPC;
	}

	return _chromeos_write(nvram_buffer);
}

static int __init chromeos_arm_init(void)
{
	int proplen, err, size;
	const int *prop;
	struct device_node *fw_dn;

	fw_dn = of_find_compatible_node(NULL, NULL, "chromeos-firmware");
	if (!fw_dn)
		return -ENODEV;

	prop = of_get_property(fw_dn, "nonvolatile-context-offset", &proplen);
	if (!prop || proplen != 4) {
		pr_err("can't find nonvolatile memory offset\n");
		err = -ENODEV;
		goto err;
	}
	nv_offset = be32_to_cpup(prop);

	prop = of_get_property(fw_dn, "boot-nonvolatile-cache", &proplen);
	if (!prop) {
		pr_err("can't find boot copy of nonvolatile cache\n");
		err = -ENODEV;
		goto err;
	}
	nv_size = proplen;

	prop = of_get_property(fw_dn, "nonvolatile-context-size", &proplen);
	if (!prop || proplen != 4) {
		pr_err("can't find size of nonvolatile memory\n");
		err = -ENODEV;
		goto err;
	}
	size = be32_to_cpup(prop);

	if (size != nv_size) {
		pr_err("nvram size and cache mismatch\n");
		err = -EINVAL;
		goto err;
	}

	if ((nv_offset + nv_size > SECTOR_SIZE) ||
	    (nv_size > MAX_NVRAM_BUFFER_SIZE)) {
		/* nvram block won't fit into a sector */
		pr_err("bad nvram location: %d:%d!\n", nv_offset, nv_size);
		err = -EINVAL;
		goto err;
	}

	prop = of_get_property(fw_dn, "nonvolatile-context-lba", &proplen);
	if (!prop) {
		pr_err("can't find nvcontext lba\n");
		err = -ENODEV;
		goto err;
	}
	switch (proplen) {
	case 4:
		nv_lba = be32_to_cpup(prop);
		break;
	case 8:
		nv_lba = be64_to_cpup((const __be64 *)prop);
		break;
	default:
		pr_err("invalid nvcontext lba\n");
		err = -EINVAL;
		goto err;
	}

	/* XXXOJN FIXME: There should be a search for the right block device to
	 * use for volatile storage here, not just assume mmcblk0. This should
	 * include comparing the cached context passed in through the property.
	 */

	pr_info("chromeos system detected\n");

	err = 0;
err:
	of_node_put(fw_dn);

	return err;
}
subsys_initcall(chromeos_arm_init);
