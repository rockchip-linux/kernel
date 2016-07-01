/* drivers/gpu/t6xx/kbase/src/platform/rk/mali_kbase_platform.h
 * Rockchip SoC Mali-T764 platform-dependent codes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

/*
 * @file mali_kbase_platform.h
 * Define work context type of rk_platform_specific_code, and some functions.
 */

#ifndef _KBASE_PLATFORM_H_
#define _KBASE_PLATFORM_H_

#include "mali_kbase_dvfs.h"

/*---------------------------------------------------------------------------*/

/*
 * struct rk_context - work context of platform_dependent_part on rk_platform.
 * @kbdev		pointer to superior 'kbase_device' instance.
 * @is_powered:		record the status of common_parts calling
 *			'power_on_callback' and 'power_off_callback'.
 * @pd_gpu:		pd_gpu.
 * @clk_gpu:		clk_gpu, althoug it isn't a 'struct clk *' indeed.
 * @are_pd_and_clk_on:	internal flag of status of pd_gpu and clk_gpu.
 * @rk_dvfs:		context of mali_dvfs on rk_platform.
 * @pm_event_notifier
 * @reboot_event_notifier
 *
 * A heap instance of the type will be maintained
 * by kbase_device::platform_context.
 */
struct rk_context {
	struct kbase_device *kbdev;

	bool is_powered;

	struct clk *pd_gpu;
	struct dvfs_node *clk_gpu;

	struct rk_dvfs_t rk_dvfs;

	/* to prevent calling rk_dvfs_module in particular system_states. */
	struct notifier_block pm_event_notifier;
	struct notifier_block reboot_event_notifier;
};

/*-------------------------------------------------------*/

static inline struct rk_context *get_rk_context(
		const struct kbase_device *kbdev)
{
	return (struct rk_context *)(kbdev->platform_context);
}

static inline struct rk_dvfs_t *get_rk_dvfs(const struct kbase_device *kbdev)
{
	struct rk_context *platform = get_rk_context(kbdev);

	return &(platform->rk_dvfs);
}

/*
 * kbase_platform_set_freq_of_clk_gpu - set freq_of_clk_gpu,
 *					called by rk_dvfs.
 * @rate:	target freq, in KHz.
 *
 * Return:	 0 on success; negative error code otherwise.
 */
int kbase_platform_set_freq_of_clk_gpu(struct kbase_device *kbdev,
				       unsigned long rate);

int kbase_platform_turn_on_gpu(struct kbase_device *kbdev);
void kbase_platform_turn_off_gpu(struct kbase_device *kbdev);

int kbase_platform_init(struct kbase_device *kbdev);
void kbase_platform_term(struct kbase_device *kbdev);

#endif				/* _KBASE_PLATFORM_H_ */
