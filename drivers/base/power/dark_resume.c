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
 *
 * A dark resume is meant to be a reduced functionality state to resume the
 * system to. It is not meant for user interaction, but to determine whether to
 * shut down/hibernate or suspend again. This is done for power saving measures.
 * To make the experience as seamless as possible drivers may want to alter
 * their resume path to do something different in this case (prevent flashing
 * screens and blinking usb ports).
 *
 * To make this system agnostic, much of the configuration is done in user
 * space.
 */

#include <linux/device.h>
#include <linux/export.h>
#include <linux/pm_dark_resume.h>
#include <linux/mutex.h>
#include <linux/types.h>

#include "power.h"

LIST_HEAD(source_list);
static DEFINE_MUTEX(source_list_lock);
static bool dark_resume_state;
static bool dark_resume_always;
static struct pm_dark_resume_ops *dark_resume_ops;

/**
 * dev_dark_resume_set_source - Set whether a device is a dark resume source.
 * @dev: the struct that contains a pointer to the device we are either adding
 * or removing as a dark resume source
 * @is_source: Set the device to a source if true and remove as source if false
 */
int dev_dark_resume_set_source(struct device *dev, bool is_source)
{
	/*
	 * This can happen if 'enabled' is written to the dark_resume_source
	 * attribute and the driver did not setup a dev_dark_resume struct.
	 */
	if (!dev->power.dark_resume) {
		dev_dbg(dev, "Tried to set device as dark resume source, but "
			     "there is no driver support.");
		return -EINVAL;
	}

	mutex_lock(&source_list_lock);
	if (is_source == dev->power.dark_resume->is_source) {
		mutex_unlock(&source_list_lock);
		return -EINVAL;
	}

	if (is_source)
		list_add(&dev->power.dark_resume->list_node, &source_list);
	else
		list_del(&dev->power.dark_resume->list_node);

	dev->power.dark_resume->is_source = is_source;
	mutex_unlock(&source_list_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(dev_dark_resume_set_source);

/**
 * dev_dark_resume_set_active - This sets up the device to check the dark resume
 * state during resume.
 * @dev: device to set querying of the dark resume state to active or inactive
 * @is_active: true for the device to query the dark resume state, false to not
 * query the dark resume state.
 *
 * Part of dark resume is that devices may resume differently (such as the
 * backlight coming on). This enables a driver to do something different in the
 * resume path by calling dev_dark_resume_active.
 */
void dev_dark_resume_set_active(struct device *dev, bool is_active)
{
	dev->power.use_dark_resume = is_active;
}
EXPORT_SYMBOL_GPL(dev_dark_resume_set_active);

/**
 * dev_dark_resume_init - Initialize the dev_dark_resume struct.
 * @dev: The device struct that the dev_dark_resume struct will be associated
 * with.
 * @dark_resume: The dev_dark_resume struct to initialize.
 * @irq: The platform callback will check (if supported) if this irq is the wake
 * source. Note: probably want a hardware irq instead of virtual irq.
 * @caused_resume: The function pointer that is called by the platform callback
 * (if supported) before devices are resumed to see if dev caused the resume of
 * the system.
 *
 * Devices that query dark resume, but cannot be a source should call this
 * function when initialized with dark_resume and caused_resume as NULL.
 */
int dev_dark_resume_init(struct device *dev,
			 struct dev_dark_resume *dark_resume,
			 int irq,
			 bool (*caused_resume)(struct device *dev))
{
	/* Must be called after device_add since sysfs attributes are added */
	if (!device_is_registered(dev))
		return -EINVAL;

	dev->power.dark_resume = dark_resume;
	dev_dark_resume_set_active(dev, false);
	/* Happens for devices that cannot be a dark resume source. */
	if (!dark_resume)
		return dark_resume_sysfs_add(dev);

	dark_resume->dev = dev;
	dark_resume->is_source = false;
	dark_resume->irq = irq;
	dark_resume->caused_resume = caused_resume;
	INIT_LIST_HEAD(&dark_resume->list_node);
	/* Don't really need to clean up anything if this fails */
	return dark_resume_sysfs_add(dev);
}
EXPORT_SYMBOL_GPL(dev_dark_resume_init);

/**
 * dev_dark_resume_remove - Remove all of the associations of the device to dark
 * resume.
 * @dev: device struct to remove associations to dark resume from.
 *
 * Makes sure that the device is no longer active for dark resume and is not a
 * source.
 */
void dev_dark_resume_remove(struct device *dev)
{
	dev_dark_resume_set_active(dev, false);
	dark_resume_sysfs_remove(dev);
	if (!dev->power.dark_resume)
		return;

	dev_dark_resume_set_source(dev, false);
	dev->power.dark_resume->caused_resume = NULL;
	dev->power.dark_resume->irq = 0;
	dev->power.dark_resume->dev = NULL;
	dev->power.dark_resume = NULL;
}
EXPORT_SYMBOL_GPL(dev_dark_resume_remove);

/**
 * pm_dark_resume_check - Call into the platform specific check function if it
 * exists to check if one of the dark resume sources woke the system.
 */
bool pm_dark_resume_check(void)
{
	if (dark_resume_always) {
		dark_resume_state = true;
		return dark_resume_state;
	}

	if (!dark_resume_ops || !dark_resume_ops->check) {
		dark_resume_state = false;
		return dark_resume_state;
	}

	mutex_lock(&source_list_lock);
	dark_resume_state = dark_resume_ops->check(&source_list);
	mutex_unlock(&source_list_lock);
	return dark_resume_state;
}
EXPORT_SYMBOL_GPL(pm_dark_resume_check);

/**
 * pm_dark_resume_active - Returns the state of dark resume.
 */
bool pm_dark_resume_active(void)
{
	return dark_resume_state;
}
EXPORT_SYMBOL_GPL(pm_dark_resume_active);

/**
 * pm_dark_resume_always - Returns whether we always wake up in dark resume from
 * a suspend.
 */
bool pm_dark_resume_always(void)
{
	return dark_resume_always;
}
EXPORT_SYMBOL_GPL(pm_dark_resume_always);

/**
 * pm_dark_resume_set_always - Sets whether we always wake up in dark resume
 * from a suspend.
 * @always: bool that tells whether we always wake up in dark resume
 */
void pm_dark_resume_set_always(bool always)
{
	dark_resume_always = always;
}
EXPORT_SYMBOL_GPL(pm_dark_resume_set_always);

/**
 * pm_dark_resume_register_ops - Registers the callback function to check
 * whether the system was resumed by something in the source_list.
 * @ops: Container for the callback function
 */
void pm_dark_resume_register_ops(struct pm_dark_resume_ops *ops)
{
	dark_resume_ops = ops;
}

void pm_dark_resume_clear_state_for_pm_test(void)
{
	dark_resume_state = false;
}
