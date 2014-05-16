/*
 * Linux Security Module for Chromium OS
 *
 * Copyright 2011 Google Inc. All Rights Reserved
 *
 * Authors:
 *      Stephan Uphoff  <ups@google.com>
 *      Kees Cook       <keescook@chromium.org>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "Chromium OS LSM: " fmt

#include <linux/module.h>
#include <linux/security.h>
#include <linux/sched.h>	/* current and other task related stuff */
#include <linux/fs.h>
#include <linux/fs_struct.h>
#include <linux/mount.h>
#include <linux/path.h>
#include <linux/root_dev.h>

#include "utils.h"

static int chromiumos_security_sb_mount(const char *dev_name, struct path *path,
					const char *type, unsigned long flags,
					void *data)
{
	int error = current->total_link_count ? -ELOOP : 0;

	if (error) {
		char *cmdline;

		cmdline = printable_cmdline(current);
		pr_notice("Mount path with symlinks prohibited - "
			"pid=%d cmdline=%s\n",
			task_pid_nr(current), cmdline);
		kfree(cmdline);
	}

	return error;
}

static void report_load_module(struct path *path, char *operation)
{
	char *alloced = NULL, *cmdline;
	char *pathname; /* Pointer to either static string or "alloced". */

	if (!path)
		pathname = "<unknown>";
	else {
		/* We will allow 11 spaces for ' (deleted)' to be appended */
		alloced = pathname = kmalloc(PATH_MAX+11, GFP_KERNEL);
		if (!pathname)
			pathname = "<no_memory>";
		else {
			pathname = d_path(path, pathname, PATH_MAX+11);
			if (IS_ERR(pathname))
				pathname = "<too_long>";
			else {
				pathname = printable(pathname);
				kfree(alloced);
				alloced = pathname;
			}
		}
	}

	cmdline = printable_cmdline(current);

	pr_notice("init_module %s module=%s pid=%d cmdline=%s\n",
		  operation, pathname, task_pid_nr(current), cmdline);

	kfree(cmdline);
	kfree(alloced);
}

static int module_locking = 1;
static struct dentry *locked_root;
static DEFINE_SPINLOCK(locked_root_spinlock);

#ifdef CONFIG_SYSCTL
static int zero;
static int one = 1;

static struct ctl_path chromiumos_sysctl_path[] = {
	{ .procname = "kernel", },
	{ .procname = "chromiumos", },
	{ }
};

static struct ctl_table chromiumos_sysctl_table[] = {
	{
		.procname       = "module_locking",
		.data           = &module_locking,
		.maxlen         = sizeof(int),
		.mode           = 0644,
		.proc_handler   = proc_dointvec_minmax,
		.extra1         = &zero,
		.extra2         = &one,
	},
	{ }
};

/* Check if the root device is read-only (e.g. dm-verity is enabled).
 * This must be called after early kernel init, since then the rootdev
 * is available.
 */
static bool rootdev_readonly(void)
{
	bool rc;
	struct block_device *bdev;
	const fmode_t mode = FMODE_WRITE;

	bdev = blkdev_get_by_dev(ROOT_DEV, mode, NULL);
	if (IS_ERR(bdev)) {
		/* In this weird case, assume it is read-only. */
		pr_info("dev(%u,%u): FMODE_WRITE disallowed?!\n",
			MAJOR(ROOT_DEV), MINOR(ROOT_DEV));
		return true;
	}

	rc = bdev_read_only(bdev);
	blkdev_put(bdev, mode);

	pr_info("dev(%u,%u): %s\n", MAJOR(ROOT_DEV), MINOR(ROOT_DEV),
		rc ? "read-only" : "writable");

	return rc;
}

static void check_locking_enforcement(void)
{
	/* If module locking is not being enforced, allow sysctl to change
	 * modes for testing.
	 */
	if (!rootdev_readonly()) {
		if (!register_sysctl_paths(chromiumos_sysctl_path,
					   chromiumos_sysctl_table))
			pr_notice("sysctl registration failed!\n");
		else
			pr_info("module locking can be disabled.\n");
	} else
		pr_info("module locking engaged.\n");
}
#else
static void check_locking_enforcement(void) { }
#endif


static int chromiumos_security_load_module(struct file *file)
{
	struct dentry *module_root;

	if (!file) {
		if (!module_locking) {
			report_load_module(NULL, "old-api-locking-ignored");
			return 0;
		}

		report_load_module(NULL, "old-api-denied");
		return -EPERM;
	}

	module_root = file->f_path.mnt->mnt_root;

	/* First loaded module defines the root for all others. */
	spin_lock(&locked_root_spinlock);
	if (!locked_root) {
		locked_root = dget(module_root);
		/*
		 * Unlock now since it's only locked_root we care about.
		 * In the worst case, we will (correctly) report locking
		 * failures before we have announced that locking is
		 * enabled. This would be purely cosmetic.
		 */
		spin_unlock(&locked_root_spinlock);
		report_load_module(&file->f_path, "locked");
		check_locking_enforcement();
	} else {
		spin_unlock(&locked_root_spinlock);
	}

	if (module_root != locked_root) {
		if (unlikely(!module_locking)) {
			report_load_module(&file->f_path, "locking-ignored");
			return 0;
		}

		report_load_module(&file->f_path, "denied");
		return -EPERM;
	}

	return 0;
}

static struct security_operations chromiumos_security_ops = {
	.name	= "chromiumos",
	.sb_mount = chromiumos_security_sb_mount,
	.kernel_module_from_file = chromiumos_security_load_module,
};


static int __init chromiumos_security_init(void)
{
	int error;

	error = register_security(&chromiumos_security_ops);

	if (error)
		panic("Could not register Chromium OS security module");

	return error;
}
security_initcall(chromiumos_security_init);

module_param(module_locking, int, S_IRUSR);
MODULE_PARM_DESC(module_locking, "Module loading restrictions (default: true)");
