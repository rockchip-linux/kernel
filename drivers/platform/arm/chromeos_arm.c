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

/* TODO:
 * let firmware assign gpio_lines
 * u32 gpio_data = (gpio_lines << 1) | is_active_high;
 */

/* gpio lines */
#define GPIO_RECOVERY	56
#define GPIO_DEVELOPER	168
#define GPIO_FW_WP	59

/* TODO:
 * replace the shared memory block with FDT
 * or structured ATAGS
 */

#define FIRMWARE_SHARED_PHYSICAL_SIZE (1024 * 1024)

/* shared data layout in phisical memory */
#define SIZE_SIGNATURE		16
#define SIZE_CHSW		4
#define SIZE_HWID		256
#define SIZE_FWID		256
#define SIZE_FRID		256
#define SIZE_BOOT_REASON	4
#define SIZE_ACTIVE_MAIN	4
#define SIZE_ACTIVE_EC		4
#define SIZE_ACTIVE_MAIN_TYPE	4
#define SIZE_RECOVERY_REASON	4
#define SIZE_GPIO		(4 * 11)
#define SIZE_NV_OFFSET		4
#define SIZE_NV_SIZE		4
#define SIZE_FMAP_ADDR		8
#define SIZE_NVBLK_LBA		8
#define SIZE_NV_COPY		16

#define OFFSET_SIGNATURE	0
#define OFFSET_CHSW		(OFFSET_SIGNATURE + SIZE_SIGNATURE)
#define OFFSET_HWID		(OFFSET_CHSW + SIZE_CHSW)
#define OFFSET_FWID		(OFFSET_HWID + SIZE_HWID)
#define OFFSET_FRID		(OFFSET_FWID + SIZE_FWID)
#define OFFSET_BOOT_REASON	(OFFSET_FRID + SIZE_FRID)
#define OFFSET_ACTIVE_MAIN	(OFFSET_BOOT_REASON + SIZE_BOOT_REASON)
#define OFFSET_ACTIVE_EC	(OFFSET_ACTIVE_MAIN + SIZE_ACTIVE_MAIN)
#define OFFSET_ACTIVE_MAIN_TYPE	(OFFSET_ACTIVE_EC + SIZE_ACTIVE_EC)
#define OFFSET_RECOVERY_REASON	(OFFSET_ACTIVE_MAIN_TYPE + SIZE_ACTIVE_MAIN_TYPE)
#define OFFSET_GPIO		(OFFSET_RECOVERY_REASON + SIZE_RECOVERY_REASON)
#define OFFSET_NV_OFFSET	(OFFSET_GPIO + SIZE_GPIO)
#define OFFSET_NV_SIZE		(OFFSET_NV_OFFSET + SIZE_NV_OFFSET)
#define OFFSET_FMAP_ADDR	(OFFSET_NV_SIZE + SIZE_NV_SIZE)
#define OFFSET_NVBLK_LBA	(OFFSET_FMAP_ADDR + SIZE_FMAP_ADDR)
#define OFFSET_NV_COPY		(OFFSET_NVBLK_LBA + SIZE_NVBLK_LBA)

#define MODULE_NAME "chromeos_arm"
static struct dentry *debugfs_entry;

static void *firmware_shared_data;

static u64 firmware_cookie_lba;

static u64 get_firmware_u64(unsigned offset)
{
	return (*(u64*)(firmware_shared_data + offset));
}

static int check_firmware_signature(void)
{
	const char signature[] = "CHROMEOS";
	return memcmp(firmware_shared_data, signature, sizeof(signature));
}

static int chromeos_arm_show(struct seq_file *s, void *unused)
{
	seq_write(s, firmware_shared_data, OFFSET_NV_COPY + SIZE_NV_COPY);
	return 0;
}

static int debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, chromeos_arm_show, inode->i_private);
}

static const struct file_operations dbg_fops = {
	.open		= debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void chromeos_arm_exit(void)
{
	if (debugfs_entry) {
		debugfs_remove(debugfs_entry);
		debugfs_entry = NULL;
	}

	if (firmware_shared_data) {
		iounmap(firmware_shared_data);
		firmware_shared_data = NULL;
	}
	pr_debug(MODULE_NAME ": removed\n");
}

int chromeos_arm_init(void)
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

	if (check_firmware_signature()) {
		pr_err(MODULE_NAME ": signature verification failed!\n");
		goto error_exit;
	}

	debugfs_entry = debugfs_create_file(MODULE_NAME, S_IRUGO,
					    NULL, NULL, &dbg_fops);
	if (!debugfs_entry) {
		pr_err(MODULE_NAME ": signature verification failed!\n");
		goto error_exit;
	}
	firmware_cookie_lba = get_firmware_u64(OFFSET_NVBLK_LBA);
	pr_debug(MODULE_NAME ": firmware cookie lba = %llx\n",
		 firmware_cookie_lba);
	return 0;

 error_exit:
	chromeos_arm_exit();
	return -ENODEV;
}

MODULE_AUTHOR("ChromiumOS Authors");
MODULE_LICENSE("GPL");

module_init(chromeos_arm_init);
module_exit(chromeos_arm_exit);
