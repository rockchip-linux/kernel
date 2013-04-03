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

#ifndef _LINUX_PM_DARK_RESUME_H
#define _LINUX_PM_DARK_RESUME_H

#include <linux/device.h>
#include <linux/pm.h>
#include <linux/types.h>

/*
 * For devices that determine if a dark resume should happen or not. caused_wake
 * should be called one time after resume.
 */
struct dev_dark_resume {
	struct list_head list_node;
	struct device *dev;
	bool (*caused_resume)(struct device *dev);
	bool is_source;
};

#ifdef CONFIG_PM_SLEEP
extern int dev_dark_resume_set_source(struct device *dev, bool is_source);
extern void dev_dark_resume_set_active(struct device *dev, bool is_active);
extern int dev_dark_resume_init(struct device *dev,
				struct dev_dark_resume *dark_resume,
				bool (*caused_resume)(struct device *dev));
extern void dev_dark_resume_remove(struct device *dev);
extern bool pm_dark_resume_check(void);
extern bool pm_dark_resume_active(void);

/*
 * Wrapper for device drivers to check if it should do anything different for a
 * dark resume and whether the global dark resume state is set.
 */
static inline bool dev_dark_resume_active(struct device *dev)
{
	return dev->power.use_dark_resume & pm_dark_resume_active();
}

#else

static inline int dev_dark_resume_set_source(struct device *dev, bool is_source)
{
	return -EINVAL;
}

static inline void dev_dark_resume_set_active(struct device *dev,
					      bool is_active) { }

static inline int dev_dark_resume_init(struct device *dev,
		struct dev_dark_resume *dark_resume,
		bool (*caused_resume)(struct device *dev))
{
	return 0;
}

static inline bool pm_dark_resume_check(void)
{
	return false;
}

static inline void dev_dark_resume_remove(struct device *dev) { }

static inline bool pm_dark_resume_active(void)
{
	return false;
}

static inline bool dev_dark_resume_active(struct device *dev)
{
	return false;
}

#endif /* !CONFIG_PM_SLEEP */
#endif /* _LINUX_PM_DARK_RESUME_H */
