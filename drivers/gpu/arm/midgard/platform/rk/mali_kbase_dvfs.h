/* drivers/gpu/midgard/platform/rk/mali_kbase_dvfs.h
 *
 * Rockchip SoC Mali-T764 DVFS driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

/*
 * @file mali_kbase_dvfs.h
 * Declare the interface of mali_dvfs_facility on rk_platform.
 *	.DP : mali_dvfs_facility, mali_dvfs_on_rk_platform:
 */

#ifndef _KBASE_DVFS_H_
#define _KBASE_DVFS_H_

/*
 * struct mali_dvfs_level_t - config info of a mali_dvfs_level(operation_point).
 * @freq:               gpu_clk_freq of this mali_dvfs_level, in KHz.
 * @min_threshold:      if gpu_utilisation is smaller than 'min_threshold',
 *                      counter_of_requests_to_jump_down will increase by 1.
 * @max_threshold:      if gpu_utilisation is greater than 'max_threshold',
 *                      counter_of_requests_to_jump_up will increase by 1.
 */
struct mali_dvfs_level_t {
	unsigned int freq;
	int min_threshold;
	int max_threshold;
};

/*
 * Type of context of mali_dvfs on rk_platform.
 */
struct rk_dvfs_t {
	bool is_enabled;

	/*
	 * Index of current mali_dvfs_level in mali_dvfs_level_table.
	 */
	int current_level;

	/*
	 * The last calculated_utilisation reported by metrics_system.
	 */
	int utilisation;

	/*
	 * The result temperature of last measure_session.
	 */
	u32 temp;
	/*
	 * Times of temp measures which has been taken in current session.
	 */
	u32 times_of_temp_measures;
	/*
	 * sum of temps measued within a session.
	 */
	u32 sum_of_temps;

	/*
	 * counter_of_requests_to_jump_up.
	 */
	u16 requests_to_jump_up;
	/*
	 * counter_of_requests_to_jump_down in dvfs_level_table.
	 */
	u16 requests_to_jump_down;

	/*
	 * Workqueue to run 'mali_dvfs_work'.
	 */
	struct workqueue_struct *mali_dvfs_wq;
	/*
	 * Work to handle dvfs_event that comes from kbase_platform_dvfs_event.
	 * Executed in '*mali_dvfs_wq'.
	 */
	struct work_struct mali_dvfs_work;

	/*
	 * Mutex for accessing 'this', including setting freq_of_clk_gpu.
	 */
	struct mutex dvfs_mutex;
};
#define MALI_KHZ 1000

/*---------------------------------------------------------------------------*/

int kbase_platform_dvfs_init(struct kbase_device *kbdev);
void kbase_platform_dvfs_term(struct kbase_device *kbdev);


/*
 * kbase_platform_dvfs_get_level - get the index of the level_entry whose
 *                                 gpu_clk_freq is exactly 'freq'
 * @free:       gpu_clk_freq to search for, in KHz.
 *
 * Search for the level_entry whose gpu_clk_freq is exactly 'freq'
 * in mali_dvfs_level_table.
 *
 * Return: If found, return the index of the level_entry.
 *         Otherwise, return -1.
 */
int kbase_platform_dvfs_get_level(struct kbase_device *kbdev, int freq);

/*
 * Set current_dvfs_level to the one with the index 'level'
 * in the dvfs_level_table.
 */
int kbase_platform_dvfs_set_level(struct kbase_device *kbdev, int level);

bool kbase_platform_dvfs_is_enabled(struct kbase_device *kbdev);

void kbase_platform_dvfs_enable(struct kbase_device *kbdev);
void kbase_platform_dvfs_disable(struct kbase_device *kbdev);

/*
 * set gpu_clk_freq to the lowest one in mali_dvfs_level_table.
 */
int kbase_platform_dvfs_set_clk_lowest(struct kbase_device *kbdev);

/*
 * set gpu_clk_freq to the highest one in mali_dvfs_level_table.
 */
int kbase_platform_dvfs_set_clk_highest(struct kbase_device *kbdev);

/*
 * return the last gpu_utilisation.
 */
int kbase_platform_dvfs_get_utilisation(struct kbase_device *kbdev);

/*
 * return ptr to mali_dvfs_level_table and num of level_entries.
 */
int kbase_platform_dvfs_get_dvfs_level_table(struct kbase_device *kbdev,
					     struct mali_dvfs_level_t **table,
					     unsigned int *num);

#endif				/* _KBASE_DVFS_H_ */
