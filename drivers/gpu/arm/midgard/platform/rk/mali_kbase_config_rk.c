/*
 *
 * (C) COPYRIGHT ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 */

/* #define ENABLE_DEBUG_LOG */
#include "custom_log.h"

#include <linux/ioport.h>
#include <mali_kbase.h>
#include <mali_kbase_defs.h>
#include <mali_kbase_config.h>
#ifdef CONFIG_UMP
#include <linux/ump-common.h>
#endif				/* CONFIG_UMP */
#include <platform/rk/mali_kbase_platform.h>
#include <platform/rk/mali_kbase_dvfs.h>
#include <linux/pm_runtime.h>
#include <linux/suspend.h>
#include <linux/reboot.h>

/*
 * @file mali_kbase_config_rk.c
 * Configuration and platform_specific_code of rk_platform.
 *
 * mali_device_driver has 2 parts:
 *	platform_dependent_part:
 *		Codes that depend on a certain platform,
 *		located under <mdd_src_dir>/platform/.
 *		In mdd_doc,
 *		platform_dependent_part is known as platform_specific_code.
 *		.DP : platform_dependent_part, platform_specific_code
 *      common_parts:
 *		Common parts implemented by ARM,
 *		all the source code except platform/ directory.
 *		.DP : common_parts
 */

/*---------------------------------------------------------------------------*/

#ifndef CONFIG_MALI_MIDGARD_DVFS
#error "we need CONFIG_MALI_MIDGARD_DVFS defined."
#endif

struct kbase_platform_funcs_conf platform_funcs = {
	.platform_init_func = &kbase_platform_init,
	.platform_term_func = &kbase_platform_term,
};

/*---------------------------------------------------------------------------*/

static int rk_pm_callback_runtime_on(struct kbase_device *kbdev)
{
	int ret = 0;

	D("to turn on gpu.");
	ret = kbase_platform_turn_on_gpu(kbdev);
	if (ret) {
		E("fail to turn on gpu. ret:%d.", ret);
		goto EXIT;
	}

	ret = kbase_platform_dvfs_set_clk_highest(kbdev);
	if (ret) {
		E("fail to set dvfs_level highest. ret:%d.", ret);
		goto EXIT;
	}

EXIT:
	return ret;
}

static void rk_pm_callback_runtime_off(struct kbase_device *kbdev)
{
	D("to turn off pd_gpu and clk_gpu.");
	kbase_platform_turn_off_gpu(kbdev);
}

static int rk_pm_callback_power_on(struct kbase_device *kbdev)
{
	int ret = 1; /* Assume GPU has been powered off */
	int error = 0;
	struct rk_context *platform = get_rk_context(kbdev);

	V("enter.");

	if (platform->is_powered) {
		D("already powered on.");
		return 0;
	}

	if (pm_runtime_enabled(kbdev->dev)) {
		D("runtime_pm is enabled, to resume mali_dev.");
		error = pm_runtime_get_sync(kbdev->dev);
		if (error < 0) {
			dev_err(kbdev->dev,
				"failed to runtime resume device: %d\n",
				error);
			return error;
		} else if (error == 1) {
			/* runtime_pm_status of mali_dev is active */

			D("has NOT been powered off, no need to re-init.");
			ret = 0;
		}
	} else {
		D("runtime_pm is NOT enabled, to power on mali_dev directly.");
		error = rk_pm_callback_runtime_on(kbdev);
		if (error) {
			E("fail to power on gpu directly, error:%d.", error);
			return error;
		}
	}

	platform->is_powered = true;
	KBASE_TIMELINE_GPU_POWER(kbdev, 1);

	return ret;
}

static void rk_pm_callback_power_off(struct kbase_device *kbdev)
{
	struct rk_context *platform = get_rk_context(kbdev);

	V("enter.");

	if (!platform->is_powered) {
		D("already powered off.");
		return;
	}

	platform->is_powered = false;
	KBASE_TIMELINE_GPU_POWER(kbdev, 0);

	if (pm_runtime_enabled(kbdev->dev)) {
		pm_runtime_mark_last_busy(kbdev->dev);
		pm_runtime_put_autosuspend(kbdev->dev);
	} else {
		D("runtime_pm is NOT enabled, to power off mali_dev directly.");
		rk_pm_callback_runtime_off(kbdev);
	}
}

int rk_kbase_device_runtime_init(struct kbase_device *kbdev)
{
	pm_runtime_set_autosuspend_delay(kbdev->dev, 200);
	pm_runtime_use_autosuspend(kbdev->dev);

	/* here, no need to set runtime_pm_status of mali_dev to active.*/

	D("to enable pm_runtime.");
	pm_runtime_enable(kbdev->dev);

	return 0;
}

void rk_kbase_device_runtime_term(struct kbase_device *kbdev)
{
	D("to disable pm_runtime.");
	pm_runtime_disable(kbdev->dev);
}

struct kbase_pm_callback_conf pm_callbacks = {
	.power_on_callback = rk_pm_callback_power_on,
	.power_off_callback = rk_pm_callback_power_off,

	.power_runtime_init_callback = rk_kbase_device_runtime_init,
	.power_runtime_term_callback = rk_kbase_device_runtime_term,
	.power_runtime_on_callback = rk_pm_callback_runtime_on,
	.power_runtime_off_callback = rk_pm_callback_runtime_off,
};

int kbase_platform_early_init(void)
{
	/* Nothing needed at this stage */
	return 0;
}
