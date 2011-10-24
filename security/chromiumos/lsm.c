/*
 * Linux Security Module for Chromium OS
 *
 * Copyright 2011 Google Inc. All Rights Reserved
 *
 * Author:
 *      Stephan Uphoff  <ups@google.com>
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

#include <linux/module.h>
#include <linux/security.h>
#include <linux/sched.h>	/* current and other task related stuff */

static int chromiumos_security_sb_mount(const char *dev_name, struct path *path,
					const char *type, unsigned long flags,
					void *data)
{
	int error = current->total_link_count ? -ELOOP : 0;

	if (error) {
		char name[sizeof(current->comm)];
		printk(KERN_NOTICE "Chromium OS LSM: Mount path with symlinks"
			" prohibited - Task %s (pid = %d)\n",
			get_task_comm(name, current), task_pid_nr(current));
	}

	return error;
}

static struct security_operations chromiumos_security_ops = {
	.name	= "chromiumos",
	.sb_mount = chromiumos_security_sb_mount,
};


static int __init chromiumos_security_init(void)
{
	int error;

	error = register_security(&chromiumos_security_ops);

	if (error)
		panic("Could not register chromiumos security module");

	return error;
}
security_initcall(chromiumos_security_init);
