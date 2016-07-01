/* drivers/gpu/t6xx/kbase/src/platform/rk/mali_kbase_platform.c
 *
 * Rockchip SoC Mali-T764 platform-dependent codes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

/*
 * @file mali_kbase_platform.c
 * Implementation of the interfaces declared in mali_kbase_platform.h.
 */

/* #define ENABLE_DEBUG_LOG */
#include "custom_log.h"

#include <mali_kbase.h>
#include <mali_kbase_pm.h>
#include <mali_kbase_uku.h>
#include <mali_kbase_mem.h>
#include <mali_midg_regmap.h>
#include <mali_kbase_mem_linux.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/miscdevice.h>
#include <linux/list.h>
#include <linux/semaphore.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/reboot.h>

#include <platform/rk/mali_kbase_platform.h>
#include <platform/rk/mali_kbase_dvfs.h>

#include <mali_kbase_gator.h>

#include <linux/rockchip/dvfs.h>

/*---------------------------------------------------------------------------*/

int kbase_platform_set_freq_of_clk_gpu(struct kbase_device *kbdev,
				       unsigned long rate)
{
	struct rk_context *platform = get_rk_context(kbdev);

	return dvfs_clk_set_rate(platform->clk_gpu, rate * MALI_KHZ);
}

/*---------------------------------------------------------------------------*/

static int kbase_platform_notify_pm_event(struct notifier_block *nb,
					  unsigned long event,
					  void *cmd)
{
	struct rk_context *platform = container_of(nb,
						   struct rk_context,
						   pm_event_notifier);
	struct kbase_device *kbdev = platform->kbdev;
	int err = NOTIFY_OK;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		kbase_platform_dvfs_enable(kbdev);
		break;
	case PM_POST_SUSPEND:
		kbase_platform_dvfs_disable(kbdev);
		break;
	default:
		break;
	}

	return err;
}

static int kbase_platform_notify_reboot_event(struct notifier_block *this,
					      unsigned long event,
					      void *ptr)
{
	struct rk_context *platform = container_of(this,
						   struct rk_context,
						   reboot_event_notifier);

	kbase_platform_dvfs_disable(platform->kbdev);
	return NOTIFY_OK;
}

/*---------------------------------------------------------------------------*/

static int kbase_platform_pd_init(struct kbase_device *kbdev)
{
	struct rk_context *platform = get_rk_context(kbdev);

	platform->pd_gpu = clk_get(NULL, "pd_gpu");
	if (IS_ERR_OR_NULL(platform->pd_gpu)) {
		E("fail to get pd_gpu");
		platform->pd_gpu = NULL;
		return -EINVAL;
	}
	/* we don't turn on pd_gpu here. */

	return 0;
}

static void kbase_platform_pd_term(struct kbase_device *kbdev)
{
	struct rk_context *platform = get_rk_context(kbdev);

	clk_put(platform->pd_gpu);
}

static int kbase_platform_turn_on_pd_gpu(struct kbase_device *kbdev)
{
	struct rk_context *platform = get_rk_context(kbdev);
	int ret = 0;

	ret = clk_prepare_enable(platform->pd_gpu);
	if (ret) {
		E("fail to enable clk, ret:%d", ret);
		goto EXIT;
	}

EXIT:
	return ret;
}

static void kbase_platform_turn_off_pd_gpu(struct kbase_device *kbdev)
{
	struct rk_context *platform = get_rk_context(kbdev);

	clk_disable_unprepare(platform->pd_gpu);
}

static int kbase_platform_clk_init(struct kbase_device *kbdev)
{
	struct rk_context *platform = get_rk_context(kbdev);

	platform->clk_gpu = clk_get_dvfs_node("clk_gpu");
	if (IS_ERR_OR_NULL(platform->clk_gpu)) {
		E("fail to get clk_gpu");
		platform->clk_gpu = NULL;
		return -ENOENT;
	}
	clk_enable_dvfs(platform->clk_gpu);

	/* we don't turn on clk_gpu here. */

	return 0;
}

static void kbase_platform_clk_term(struct kbase_device *kbdev)
{
	struct rk_context *platform = get_rk_context(kbdev);

	clk_put_dvfs_node(platform->clk_gpu);
}

static int kbase_platform_turn_on_clk_gpu(struct kbase_device *kbdev)
{
	int ret = 0;
	struct rk_context *platform = get_rk_context(kbdev);

	ret = dvfs_clk_prepare_enable(platform->clk_gpu);
	if (ret) {
		E("fail to enable clk_gpu, ret=%d.", ret);
		goto EXIT;
	}

EXIT:
	return ret;
}

static void kbase_platform_turn_off_clk_gpu(struct kbase_device *kbdev)
{
	struct rk_context *platform = get_rk_context(kbdev);

	dvfs_clk_disable_unprepare(platform->clk_gpu);
}

int kbase_platform_turn_on_gpu(struct kbase_device *kbdev)
{
	int ret;

	ret = kbase_platform_turn_on_pd_gpu(kbdev);
	if (ret) {
		E("fail to turn on pd_gpu, ret:%d.", ret);
		goto EXIT;
	}

	ret = kbase_platform_turn_on_clk_gpu(kbdev);
	if (ret) {
		E("fail to turn on clk_gpu, ret:%d.", ret);
		goto EXIT;
	}

EXIT:
	return ret;
}

void kbase_platform_turn_off_gpu(struct kbase_device *kbdev)
{
	kbase_platform_turn_off_clk_gpu(kbdev);
	kbase_platform_turn_off_pd_gpu(kbdev);
}

/*---------------------------------------------------------------------------*/
/* < implementation of sysfs_nodes extended by rk_platform. > */

/*
 * DOC: Sysfs Nodes of Mali Device Extended by RK Platform
 *
 * The dir of mali_device under sysfs on rk_platform
 * might be sys/devices/ffa30000.gpu.
 *
 * In this directory, there are some nodes extended by RK :
 *	clock
 *		"cat clock" will return current gpu_clk_freq
 *		and a list of available_freqs,
 *		such as :
 *
 *			current_gpu_clk_freq :  99000 KHz
 *			available_freqs : 179000, 297000, 417000, 480000 (KHz)
 *
 *		The frequency points present in available_freqs are
 *		the operation_points defined in the dts file.
 *
 *		We can set gpu_clk_freq with command
 *
 *			echo <target_freq_in_khz> > clock
 *
 *		The 'target_freq_in_khz' must be one of the frequence points
 *		in 'available_freqs'.
 *		Otherwise, the command above would have no effect.
 *
 *		In addition, DVFS in enabled by default.
 *		If you want to fix gpu_clk at a specific frequency,
 *		run "echo off > dvfs" to disable DVFS first.
 *
 *	dvfs
 *		"cat dvfs" will return current status of mali_dvfs, along with
 *		gpu_utilisation and current_gpu_clk_freq.
 *
 *		If DEVS is enabled, it might return something like
 *		        mali_dvfs is ON
 *			gpu_utilisation : 100
 *			current_gpu_clk_freq : 480 MHz
 *
 *		While, if DEVS is disabled, it might be
 *			mali_dvfs is OFF
 *			current_gpu_clk_freq : 99 MHz
 *
 *		Writing "off" to the node (echo off > dvfs) will disable DVFS of
 *		mali_device, and set gpu_clk_freq to the highest available one.
 *		We can write "on" to the node to enable DVFS again.
 */

static ssize_t show_clock(struct device *dev,
			  struct device_attribute *attr,
			  char *buf)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);
	struct rk_context *platform = get_rk_context(kbdev);
	ssize_t ret = 0;
	unsigned int clkrate = 0; /* gpu_clk_freq from rk_dvfs_module, in Hz. */
	int i;
	struct mali_dvfs_level_t *dvfs_level_table;
	unsigned int num_of_dvfs_levels;

	if (kbase_platform_dvfs_get_dvfs_level_table(kbdev,
						     &dvfs_level_table,
						     &num_of_dvfs_levels)) {
		E("fail to get dvfs_level_table.");
		return -ENOENT;
	}

	clkrate = dvfs_clk_get_rate(platform->clk_gpu);
	ret += snprintf(buf + ret,
			PAGE_SIZE - ret,
			"current_gpu_clk_freq : %d KHz",
			clkrate / 1000);

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "\navailable_freqs : ");
	for (i = 0; i < num_of_dvfs_levels; i++) {
		if (i < (num_of_dvfs_levels - 1)) {
			ret += snprintf(buf + ret,
					PAGE_SIZE - ret,
					"%d, ",
					dvfs_level_table[i].freq);
		} else {
			ret += snprintf(buf + ret,
					PAGE_SIZE - ret,
					"%d ",
					dvfs_level_table[i].freq);
		}
	}
	ret += snprintf(buf + ret, PAGE_SIZE - ret, "(KHz)\n");

	return ret;
}

static ssize_t set_clock(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);
	struct rk_context *platform = get_rk_context(kbdev);
	unsigned int freq;
	int target_level;
	int ret = 0;

	if (!platform->clk_gpu)
		return -ENODEV;

	ret = kstrtouint(buf, 0, &freq);
	if (ret) {
		E("invalid input freq : %s", buf);
		goto EXIT;
	}
	D("freq : %u.", freq);

	target_level = kbase_platform_dvfs_get_level(kbdev, freq);
	if (-1 == target_level) {
		E("no dvfs_level available for freq '%u' KHz.", freq);
		goto EXIT;
	}

	ret = kbase_platform_dvfs_set_level(kbdev, target_level);
	if (ret) {
		E("fail to set level. target_level:%d.", target_level);
		goto EXIT;
	}

EXIT:
	return count;
}

static ssize_t show_dvfs(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);
	struct rk_context *platform = get_rk_context(kbdev);
	ssize_t ret = 0;
	unsigned int clkrate;

	D("enter.");

	/* get clk_freq of 'gpu_dvfs_node', in Hz. */
	clkrate = dvfs_clk_get_rate(platform->clk_gpu);

	if (kbase_platform_dvfs_is_enabled(kbdev)) {
		ret += snprintf(buf + ret,
				PAGE_SIZE - ret,
				"mali_dvfs is ON\n");

	} else {
		ret += snprintf(buf + ret,
				PAGE_SIZE - ret,
				"mali_dvfs is OFF\n");
	}

	ret += snprintf(buf + ret,
			PAGE_SIZE - ret,
			"gpu_utilisation : %d\ncurrent_gpu_clk_freq : %u MHz\n",
			kbase_platform_dvfs_get_utilisation(kbdev),
			clkrate / 1000000);

	return ret;
}

static ssize_t set_dvfs(struct device *dev,
			struct device_attribute *attr,
			const char *buf,
			size_t count)
{
	int ret = 0;
	struct kbase_device *kbdev = dev_get_drvdata(dev);

	if (sysfs_streq("off", buf)) {
		D("to disable mali_dvfs, and set dvfs_level highest.");
		kbase_platform_dvfs_disable(kbdev);
		ret = kbase_platform_dvfs_set_clk_highest(kbdev);
		if (ret)
			W("fail to set clk highest");
	} else if (sysfs_streq("on", buf)) {
		D("to enable mali_dvfs.");
		kbase_platform_dvfs_enable(kbdev);
	} else {
		E("invalid input '%s', only 'on' or 'off' is acceptable.", buf);
	}

	return count;
}

static DEVICE_ATTR(clock, S_IRUGO | S_IWUSR, show_clock, set_clock);
static DEVICE_ATTR(dvfs, S_IRUGO | S_IWUSR, show_dvfs, set_dvfs);

/*---------------------------------------------------------------------------*/

static int kbase_platform_create_sysfs_file(struct device *dev)
{
	if (device_create_file(dev, &dev_attr_clock)) {
		dev_err(dev, "Couldn't create sysfs file [clock]\n");
		E("couldn't create sysfs file 'clock'.");
		goto out;
	}

	if (device_create_file(dev, &dev_attr_dvfs)) {
		E("couldn't create sysfs file 'dvfs'.");
		goto term_sysfs_clock;
	}

	return 0;

term_sysfs_clock:
	device_remove_file(dev, &dev_attr_clock);
out:
	return -ENOENT;
}

static void kbase_platform_remove_sysfs_file(struct device *dev)
{
	device_remove_file(dev, &dev_attr_clock);
	device_remove_file(dev, &dev_attr_dvfs);
}

int kbase_platform_init(struct kbase_device *kbdev)
{
	struct rk_context *platform = NULL;
	int ret;

	platform = devm_kzalloc(kbdev->dev, sizeof(*platform), GFP_KERNEL);
	if (!platform)
		return -ENOMEM;

	/* .KP : attach 'platform' to 'kbase_device' instance. */
	kbdev->platform_context = (void *)platform;

	platform->kbdev = kbdev;

	platform->is_powered = false;

	/*-----------------------------------*/

	ret = kbase_platform_pd_init(kbdev);
	if (ret) {
		E("fai to init pd_gpu, ret = %d.", ret);
		goto err_init;
	}

	ret = kbase_platform_clk_init(kbdev);
	if (ret) {
		E("fai to init clk_gpu, ret = %d.", ret);
		goto term_pd;
	}

	/*-----------------------------------*/

	ret = kbase_platform_dvfs_init(kbdev);
	if (ret) {
		E("fai to init dvfs, ret = %d.", ret);
		goto term_clk;
	}

	/*-----------------------------------*/

	ret = kbase_platform_create_sysfs_file(kbdev->dev);
	if (ret) {
		E("fai to create sysfs_files, ret = %d.", ret);
		goto term_dvfs;
	}

	/* we don't turn on pd_gpu and clk_gpu here,
	 * leave it to 'power_runtime_on_callback'. */

	/*-----------------------------------*/

	platform->pm_event_notifier.notifier_call
		= kbase_platform_notify_pm_event;
	ret = register_pm_notifier(&platform->pm_event_notifier);
	if (ret) {
		E("fail to register pm_event_notifier.");
		goto remove_sysfs_fiels;
	}

	/*-----------------------------------*/

	platform->reboot_event_notifier.notifier_call
		= kbase_platform_notify_reboot_event;
	ret = register_reboot_notifier(&platform->reboot_event_notifier);
	if (ret) {
		E("fail to register reboot_event_notifier.");
		goto unregister_pm_event_notifier;
	}

	return 0;

unregister_pm_event_notifier:
	unregister_pm_notifier(&platform->pm_event_notifier);
remove_sysfs_fiels:
	kbase_platform_remove_sysfs_file(kbdev->dev);
term_dvfs:
	kbase_platform_dvfs_term(kbdev);
term_clk:
	kbase_platform_clk_term(kbdev);
term_pd:
	kbase_platform_pd_term(kbdev);
err_init:
	kbdev->platform_context = NULL;

	return ret;
}

void kbase_platform_term(struct kbase_device *kbdev)
{
	struct rk_context *platform = get_rk_context(kbdev);

	unregister_reboot_notifier(&platform->reboot_event_notifier);

	unregister_pm_notifier(&platform->pm_event_notifier);

	kbase_platform_remove_sysfs_file(kbdev->dev);

	kbase_platform_dvfs_term(kbdev);

	kbase_platform_clk_term(kbdev);

	kbase_platform_pd_term(kbdev);

	kbdev->platform_context = NULL;
}
