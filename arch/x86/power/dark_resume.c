/* Copyright (C) 2013 Google, Inc.
 *
 * Author:
 *      Derek Basehore <dbasehore@chromium.org>
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

#include <linux/list.h>
#include <linux/pm_dark_resume.h>

static bool x86_dark_resume_check(struct list_head *list)
{
	struct dev_dark_resume *dark_resume;

	/* TODO: add functionality for SWS_ */
	list_for_each_entry(dark_resume, list, list_node) {
		if (dark_resume->caused_resume &&
		    dark_resume->caused_resume(dark_resume->dev)) {
			return true;
		}
	}
	return false;
}

struct pm_dark_resume_ops dark_resume_ops = {
	.check = x86_dark_resume_check,
};

static __init int x86_dark_resume_init(void)
{
	pm_dark_resume_register_ops(&dark_resume_ops);
	return 0;
}
arch_initcall(x86_dark_resume_init);
