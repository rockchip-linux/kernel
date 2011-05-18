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

#include <asm/io.h>
#include <asm/setup.h>
#include <linux/chromeos_platform.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/ide.h>
#include "../chromeos.h"

/* TODO:
 * replace the shared memory block with FDT
 * or structured ATAGS
 */

#define FIRMWARE_SHARED_PHYSICAL_SIZE (1024 * 1024)
#define BLKNV_MAJOR MMC_BLOCK_MAJOR
#define BLKNV_MINOR 0
#define MODULE_NAME "chromeos_arm"
#define SHARED_MEM_VERSION 1

/*
 * This structure must match the head of the structure in
 * u-boot-next:files/lib/chromeos/os_storage.c
 */
struct shared_data {
	u32 total_size;
	u8 signature[10];
	u16 version;
	u64 nvcxt_lba;
	u16 vbnv[2];
	u8  nvcxt_cache[1]; /* the actual size is in vbnv[1] */
} __attribute__((packed));

static struct dentry *debugfs_entry;
static struct shared_data *firmware_shared_data;
static u64 phys_window_base = 0x3ff00000;

/* location where the nvram data blends into the sector on the MMC device */
static u16 nv_offset, nv_size;

static int verify_shared_memory(void)
{
	const char signature[] = "CHROMEOS";

	if (firmware_shared_data->version != SHARED_MEM_VERSION) {
		pr_err(MODULE_NAME ": version mismatch: %d != %d\n",
		       firmware_shared_data->version, SHARED_MEM_VERSION);
		return -EINVAL;
	}

	if(memcmp(firmware_shared_data->signature, signature,
		  sizeof(signature))) {
		pr_err(MODULE_NAME ": signature mismatch\n");
		return -EINVAL;
	}

	if ((nv_offset > SECTOR_SIZE) ||
	    !nv_size ||
	    ((nv_offset + nv_size) > SECTOR_SIZE) ||
	    (nv_size > MAX_NVRAM_BUFFER_SIZE)) {
		/* nvram block won't fit into a sector */
		pr_err(MODULE_NAME ": bad nvram location: %d:%d!\n",
		       nv_offset, nv_size);
		return -EINVAL;
	}
	return 0;
}

static int chromeos_arm_show(struct seq_file *s, void *unused)
{
	seq_write(s, firmware_shared_data, firmware_shared_data->total_size);
	return 0;
}

static int debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, chromeos_arm_show, inode->i_private);
}

/*
 * Functions to support nvram on block device. The actual device used is minor
 * 0 of MMC device class, the sector to use is as encoded in
 * firmware_shared_data->nvcxt_lba, the nvram buffer in the sector starts at
 * offset nv_offset and takes nv_size bytes.
 */
static void blknv_endio(struct bio *bio, int err)
{
	complete((struct completion *)bio->bi_private);
	bio->bi_private = (void *)err;
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
		pr_err(MODULE_NAME ": could not open dev=[%d:%d]\n",
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
	if (bio->bi_private)
		ret = (int)bio->bi_private;
	else
		ret = 0;
out:
	if (bio)
		bio_put(bio);
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
 * appropriate mmc one sector block, blends in the new nvram contents and then
 * writes the sector back. If successful, the cached nvram in the shared
 * memory is also updated.
 */
static int _chromeos_write(const u8 *data)
{
	struct page *page;
	char *virtual_addr;
	int rv;

	page = alloc_page(GFP_NOIO);
	if (!page) {
		pr_err(MODULE_NAME ": page allocation failed\n");
		return -ENOMEM;
	}

	virtual_addr = page_address(page);
	if (!virtual_addr) {
		pr_err(MODULE_NAME ": page not mapped!\n");
		__free_page(page);
		return -EFAULT;
	}

	rv = chromeos_read_nvram_block(page, firmware_shared_data->nvcxt_lba);
	if (rv)
		goto unwind;

	/* As a sanity check, let's confirm that what we read is indeed the
	 * sector containing the NVRAM. Note that on a running system the
	 * NVRAM block will include its CRC.
	 */
	if (memcmp(virtual_addr + nv_offset,
		   firmware_shared_data->nvcxt_cache, nv_size)) {
		pr_err(MODULE_NAME ": cached contents mismatch!\n");
		rv = -EFAULT;
		goto unwind;
	}


	/*
	 * Sector has been read, lets blend in nvram data and write the sector
	 * back.
	 */
	memcpy(virtual_addr + nv_offset, data, nv_size);

	rv = chromeos_write_nvram_block(page, firmware_shared_data->nvcxt_lba);
	if (!rv) {
		/* Write was successful, now update the local cache */
		memcpy(firmware_shared_data->nvcxt_cache,
		       virtual_addr + nv_offset,
		       nv_size);
		rv = nv_size;
	}

unwind:
	__free_page(page);
	return rv;
}

static int chromeos_write(struct file *unused1, const char __user *data,
			  size_t len, loff_t *unused2)
{
	u8 local_copy[MAX_NVRAM_BUFFER_SIZE];

	if (!access_ok(VERIFY_READ, data, len))
		return -EFAULT;

	if (len != nv_size) {
		pr_err(MODULE_NAME ": %d != %d, invalid block size\n",
		       len, nv_size);
		return -EINVAL;
	}

	if (copy_from_user(local_copy, data, len)) {
		pr_err(MODULE_NAME ": copy failed!\n");
		return -EFAULT;
	}
	return _chromeos_write(local_copy);
}

static const struct file_operations dbg_fops = {
	.open		= debugfs_open,
	.read		= seq_read,
	.write		= chromeos_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};


/*
 * Read the nvram buffer contents into the user provided space.
 *
 * retrun number of bytes copied, or -1 on any error.
 */
int chromeos_platform_read_nvram(u8 *nvram_buffer, int buf_size)
{
	if (!firmware_shared_data) {
		pr_err(MODULE_NAME ": %s NVRAM not configured!\n", __func__);
		return -1;
	}

	if (buf_size < nv_size) {
		pr_err(MODULE_NAME
		       ": not enough room to read nvram (%d < %d)\n",
		       buf_size, nv_size);
		return -1;
	}

	memcpy(nvram_buffer, firmware_shared_data->nvcxt_cache, nv_size);
	return nv_size;
}

int chromeos_platform_write_nvram(u8 *nvram_buffer, int buf_size)
{
	if (!firmware_shared_data) {
		pr_err(MODULE_NAME ": %sw NVRAM not configured!\n", __func__);
		return -1;
	}

	if (buf_size != nv_size) {
		pr_err(MODULE_NAME ": wrong write buffer size (%d != %d)\n",
		       buf_size, nv_size);
		return -1;
	}

	return _chromeos_write(nvram_buffer);
}

static void __init chromeos_arm_shutdown(void)
{
	if (debugfs_entry) {
		debugfs_remove(debugfs_entry);
		debugfs_entry = NULL;
	}

	if (firmware_shared_data) {
		iounmap(firmware_shared_data);
		firmware_shared_data = NULL;
	}
}

static int __init chromeos_arm_init(void)
{
	if (!phys_window_base) {
		pr_err(MODULE_NAME ": memory window base undefined\n");
		return -ENODEV;
	}

	pr_info(MODULE_NAME ": memory window base at %llx\n", phys_window_base);

	/*
	 * TODO(vbendeb): FIXME: we are getting a pointer to some memory which
	 * is beyond the kernel controlled memory range. Strictly speaking
	 * this pointer should not be just dereferenced, ioread()/iowrite()
	 * should be used instead to comply with some code analysis tools and
	 * run time checks (when enabled). A possible solution would be to
	 * copy data from the window into a kernel allocated buffer.
	 *
	 * This will have to be addressed when the FDT rework comes along.
	 */
	firmware_shared_data = ioremap(phys_window_base,
				       FIRMWARE_SHARED_PHYSICAL_SIZE);
	if (!firmware_shared_data) {
		pr_err(MODULE_NAME ": failed to map address %llx\n",
		       phys_window_base);
		return -EINVAL;
	}

	nv_offset = firmware_shared_data->vbnv[0];
	nv_size = firmware_shared_data->vbnv[1];

	if (verify_shared_memory()) {
		chromeos_arm_shutdown();
		return -EINVAL;
	}


	debugfs_entry = debugfs_create_file(MODULE_NAME, S_IRUGO | S_IWUSR,
					    NULL, NULL, &dbg_fops);
	if (!debugfs_entry) {
		pr_err(MODULE_NAME ": failed to create a debugfs file!\n");
		chromeos_arm_shutdown();
		return -EINVAL;
	}
	pr_debug(MODULE_NAME ": firmware cookie lba = %llx\n",
		 firmware_shared_data->nvcxt_lba);
	return 0;
}
subsys_initcall(chromeos_arm_init);

static int __init get_mem_base(char *p)
{
	char *endptr;   /* local pointer to end of parsed string */
	u64 base = simple_strtoull(p, &endptr, 0);

	/* rudimentary sanity check */
	if (base & ((1 << 20) - 1)) {
		pr_err(MODULE_NAME ": unaligned window base 0x%llx\n", base);
		return -1;
	}

	phys_window_base = base;
	return 0;
}

early_param("cros_shared_mem", get_mem_base);
