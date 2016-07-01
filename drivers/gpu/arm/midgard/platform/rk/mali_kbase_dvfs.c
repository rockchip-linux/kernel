/* drivers/gpu/t6xx/kbase/src/platform/manta/mali_kbase_dvfs.c
 *
 * Rockchip SoC Mali-T764 DVFS driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

/**
 * @file mali_kbase_dvfs.c
 * DVFS
 */

/* #define ENABLE_DEBUG_LOG */
#include "custom_log.h"

#include <mali_kbase.h>
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
#include <linux/cpufreq.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/rk_fb.h>
#include <linux/input.h>
#include <linux/rockchip/common.h>

#include <platform/rk/mali_kbase_platform.h>
#include <platform/rk/mali_kbase_dvfs.h>
#include <mali_kbase_gator.h>
#include <linux/rockchip/dvfs.h>

/*---------------------------------------------------------------------------*/

/*
 * A threshold,
 * if counter_of_requests_to_jump_up_in_dvfs_level_table reaches this value,
 * the current_dvfs_level will jump up one level actually.
 */
#define NUM_OF_REQUESTS_TO_PERFORM_ACTUAL_JUMP_UP 1

/*
 * A threshold,
 * if counter_of_requests_to_jump_down_in_dvfs_level_table reaches this value,
 * the current_dvfs_level will jump down one level actually.
 */
#define NUM_OF_REQUESTS_TO_PERFORM_ACTUAL_JUMP_DOWN 2

/* Upper limit of GPU temp. */
#define TEMP_UPPER_LIMIT		(110)
/*
 * After measuring temperature for 'NUM_OF_TEMP_MEASURES_IN_A_SESSION' times,
 * driver will calculate the average as the result temperature
 * of this temp_measure_session.
 */
#define NUM_OF_TEMP_MEASURES_IN_A_SESSION	(1)

#define level0_min 0
#define level0_max 70
#define levelf_max 100

static u32 div_dvfs;

/*
 * The gpu_clk_freqs of level_entries are from low to high.
 *
 * This this will reinitialiized with 's_mali_freq_table' in runtime.
 * If failed to get 's_mali_freq_table', the default value here will be used.
 * See kbase_platform_dvfs_init.
 */
static struct mali_dvfs_level_t s_mali_dvfs_level_table[] = {
	{100000, 0, 70},
	{160000, 50, 65},
	{266000, 60, 78},
	{350000, 65, 75},
	{400000, 70, 75},
	{500000, 90, 100},
};

/*
 * Max num of level_entries that mali_dvfs_level_table can hold.
 */
static const unsigned int MAX_NUM_OF_MALI_DVFS_LEVELS
	= ARRAY_SIZE(s_mali_dvfs_level_table);

/*
 * s_num_of_mali_dvfs_levels - Num of effective level_entries
 *                             in mali_dvfs_level_table.
 *
 * Runtime value of this might be different from the one
 * initialized in compiling time here.
 */
static unsigned int s_num_of_mali_dvfs_levels
	= ARRAY_SIZE(s_mali_dvfs_level_table);

/*
 * gpu_clk_freq_table_from_rk_dvfs_module.
 * The freq_points(operation_points) are configured in dts file.
 */
static struct cpufreq_frequency_table *s_mali_freq_table;

#define LIMIT_FPS 60
#define LIMIT_FPS_POWER_SAVE 50

/*---------------------------------------------------------------------------*/

static void calculate_dvfs_max_min_threshold(u32 level);

static int set_dvfs_level_internal(struct rk_dvfs_t *dvfs, int target_level);

static inline int get_current_level(const struct rk_dvfs_t *dvfs)
{
	return dvfs->current_level;
}

/*
 * Jump down one level in dvfs_level_table.
 * The caller must ensure that dvfs_level could jump down indeed.
 */
static inline int jump_down_actually(struct rk_dvfs_t *dvfs)
{
	return set_dvfs_level_internal(dvfs, get_current_level(dvfs) - 1);
}

static inline int jump_up_actually(struct rk_dvfs_t *dvfs)
{
	return set_dvfs_level_internal(dvfs, get_current_level(dvfs) + 1);
}

static inline void reset_requests_to_jump_up(struct rk_dvfs_t *dvfs)
{
	dvfs->requests_to_jump_up = 0;
}

static inline void reset_requests_to_jump_down(struct rk_dvfs_t *dvfs)
{
	dvfs->requests_to_jump_down = 0;
}

static inline void inc_requests_to_jump_up(struct rk_dvfs_t *dvfs)
{
	(dvfs->requests_to_jump_up)++;
}

static inline void inc_requests_to_jump_down(struct rk_dvfs_t *dvfs)
{
	(dvfs->requests_to_jump_down)++;
}

static inline bool are_enough_jump_up_requests(const struct rk_dvfs_t *dvfs)
{
	return (dvfs->requests_to_jump_up
			>= NUM_OF_REQUESTS_TO_PERFORM_ACTUAL_JUMP_UP);
}

static inline bool are_enough_jump_down_requests(const struct rk_dvfs_t *dvfs)
{
	return (dvfs->requests_to_jump_down
			>= NUM_OF_REQUESTS_TO_PERFORM_ACTUAL_JUMP_DOWN);
}

static int get_lowest_level(const struct rk_dvfs_t *dvfs)
{
	return 0;
}

static int get_highest_level_available(const struct rk_dvfs_t *dvfs)
{
	return s_num_of_mali_dvfs_levels - 1;
}

static inline bool could_jump_up(const struct rk_dvfs_t *dvfs)
{
	return dvfs->current_level < get_highest_level_available(dvfs);
}

static inline bool could_jump_down(const struct rk_dvfs_t *dvfs)
{
	return dvfs->current_level > get_lowest_level(dvfs);
}

static inline bool is_overheated(const struct rk_dvfs_t *dvfs)
{
	return (dvfs->temp > TEMP_UPPER_LIMIT);
}

static int get_util_max_threshold_of_curr_level(const struct rk_dvfs_t *dvfs)
{
	return s_mali_dvfs_level_table[get_current_level(dvfs)].max_threshold;
}

static int get_util_min_threshold_of_curr_level(const struct rk_dvfs_t *dvfs)
{
	return s_mali_dvfs_level_table[get_current_level(dvfs)].min_threshold;
}

/*
 * Get freq(in KHz) of specific dvfs_level with index 'level'
 * in mali_dvfs_level_table.
 */
static inline unsigned int get_clk_freq_of_level(struct rk_dvfs_t *dvfs,
						 int level)
{
	return s_mali_dvfs_level_table[level].freq;
}

static inline bool is_dvfs_level_valid(struct rk_dvfs_t *dvfs,
				       unsigned int level)
{
	return (get_lowest_level(dvfs) <= level) &&
		(level <= get_highest_level_available(dvfs));
}

/*---------------------------------------------------------------------------*/

#define work_to_dvfs(w) container_of(w, struct rk_dvfs_t, mali_dvfs_work)
#define dvfs_to_rk_context(dvfs) container_of(dvfs, struct rk_context, rk_dvfs)

/*
 * Function of mali_dvfs_work, handling dvfs_event.
 */
static void mali_dvfs_event_proc(struct work_struct *w)
{
	int ret = 0;
	struct rk_dvfs_t *dvfs = work_to_dvfs(w);
	int temp = rockchip_tsadc_get_temp(1, 0);

	if (INVALID_TEMP == temp) {
		D("got invalid temp, reset to 0.");
		temp = 0;
	}
	dvfs->sum_of_temps += temp;
	dvfs->times_of_temp_measures++;

	if (dvfs->times_of_temp_measures >= NUM_OF_TEMP_MEASURES_IN_A_SESSION) {
		dvfs->temp = dvfs->sum_of_temps
			/ NUM_OF_TEMP_MEASURES_IN_A_SESSION;

		dvfs->times_of_temp_measures = 0;
		dvfs->sum_of_temps = 0;
	}

	/*-------------------------------------------------------*/

	mutex_lock(&(dvfs->dvfs_mutex));

	if (is_overheated(dvfs)) {
		if (could_jump_down(dvfs)) {
			I("to jump down for overheated, temp:%d.",
			  dvfs->temp);
			ret = jump_down_actually(dvfs);
			if (ret)
				E("fail to jump down, ret:%d.", ret);
		} else {
			W("overheated! temp:%d, but can't jump down anymore.",
			  dvfs->temp);
		}

		dvfs->temp = 0;
		goto EXIT;
	}

	/* If calculated_utilisation asks current_level to jump up,
	 * and current_level could jump up,
	 * then ....  */
	if (dvfs->utilisation > get_util_max_threshold_of_curr_level(dvfs) &&
	    could_jump_up(dvfs)) {
		inc_requests_to_jump_up(dvfs);

		if (are_enough_jump_up_requests(dvfs)) {
			V("to jump up actually, util:%d, curr_level:%d. ",
			  dvfs->utilisation,
			  get_current_level(dvfs));
			ret = jump_up_actually(dvfs);
			if (ret) {
				E("fail to jump up, ret:%d.", ret);
				goto EXIT;
			}

			reset_requests_to_jump_up(dvfs);
		}

		reset_requests_to_jump_down(dvfs);
		goto EXIT;
	} else if (dvfs->utilisation
			< get_util_min_threshold_of_curr_level(dvfs) &&
		   could_jump_down(dvfs)) {
		inc_requests_to_jump_down(dvfs);

		if (are_enough_jump_down_requests(dvfs)) {
			V("to jump down actually, util:%d, curr_level:%d",
			  dvfs->utilisation,
			  get_current_level(dvfs));
			jump_down_actually(dvfs);

			reset_requests_to_jump_down(dvfs);
		}

		reset_requests_to_jump_up(dvfs);
		goto EXIT;
	} else {
		reset_requests_to_jump_down(dvfs);
		reset_requests_to_jump_up(dvfs);

		V("stay in current_level, util:%d, curr_level:%d."
				dvfs->utilisation,
				get_current_level(dvfs));
	}

EXIT:
	mutex_unlock(&(dvfs->dvfs_mutex));
}

/*---------------------------------------------------------------------------*/

/*
 * Callback called by common_parts,
 * to notify platform_dependent_part of dvfs_event.
 */
int kbase_platform_dvfs_event(struct kbase_device *kbdev,
			      u32 utilisation, /* calculated_utilisation. */
			      u32 util_gl_share_no_use,
			      u32 util_cl_share_no_use[2])
{
	struct rk_dvfs_t *dvfs = get_rk_dvfs(kbdev);

	dvfs->utilisation = utilisation;

	if (dvfs->is_enabled) {
		/* run 'mali_dvfs_work' in 'mali_dvfs_wq', on cpu0. */
		queue_work_on(0, dvfs->mali_dvfs_wq, &(dvfs->mali_dvfs_work));
	}

	return 1;
}

/*---------------------------------------------------------------------------*/

int kbase_platform_dvfs_init(struct kbase_device *kbdev)
{
	struct rk_context *platform = get_rk_context(kbdev);
	struct rk_dvfs_t *dvfs = get_rk_dvfs(kbdev);
	int i;

	D("to get gpu_clk_freq_table from rk_dvfs_module.");
	s_mali_freq_table = dvfs_get_freq_volt_table(platform->clk_gpu);
	if (NULL == s_mali_freq_table) {
		W("mali freq table not assigned yet, use default.");
	} else {
		D("to reinit mali_dvfs_level_table with gpu_clk_freq_table.");

		s_num_of_mali_dvfs_levels = 0;

		for (i = 0;
		     s_mali_freq_table[i].frequency != CPUFREQ_TABLE_END &&
			i < MAX_NUM_OF_MALI_DVFS_LEVELS;
		     i++) {
			s_mali_dvfs_level_table[i].freq
				= s_mali_freq_table[i].frequency;
			s_num_of_mali_dvfs_levels++;
		}

		if (s_num_of_mali_dvfs_levels > 1)
			div_dvfs = round_up((levelf_max - level0_max)
					/ (s_num_of_mali_dvfs_levels - 1),
					1);

		I("num_of_mali_dvfs_levels = %d, div_dvfs = %d.",
		  s_num_of_mali_dvfs_levels,
		  div_dvfs);

		for (i = 0; i < s_num_of_mali_dvfs_levels; i++)
			calculate_dvfs_max_min_threshold(i);
	}

	/*-------------------------------------------------------*/

	mutex_init(&(dvfs->dvfs_mutex));

	dvfs->mali_dvfs_wq = create_singlethread_workqueue("mali_dvfs");
	INIT_WORK(&(dvfs->mali_dvfs_work), mali_dvfs_event_proc);

	dvfs->utilisation = 0;

	dvfs->temp = 0;
	dvfs->times_of_temp_measures = 0;
	dvfs->sum_of_temps = 0;

	dvfs->requests_to_jump_up = 0;
	dvfs->requests_to_jump_down = 0;

	kbase_platform_dvfs_set_clk_lowest(kbdev);

	dvfs->is_enabled = true;

	return 0;
}

void kbase_platform_dvfs_term(struct kbase_device *kbdev)
{
	struct rk_dvfs_t *dvfs = get_rk_dvfs(kbdev);

	cancel_work_sync(&dvfs->mali_dvfs_work);
	if (dvfs->mali_dvfs_wq)
		destroy_workqueue(dvfs->mali_dvfs_wq);
	dvfs->mali_dvfs_wq = NULL;

	dvfs->is_enabled = false;
}

int kbase_platform_dvfs_get_level(struct kbase_device *kbdev, int freq)
{
	int i;

	for (i = 0; i < s_num_of_mali_dvfs_levels; i++) {
		if (s_mali_dvfs_level_table[i].freq == freq)
			return i;
	}

	return -1;
}

int kbase_platform_dvfs_set_level(struct kbase_device *kbdev, int level)
{
	int ret = 0;
	struct rk_dvfs_t *dvfs = get_rk_dvfs(kbdev);

	if (!is_dvfs_level_valid(dvfs, level)) {
		E("invalid level '%d'.", level);
		return -EINVAL;
	}

	if (level == get_current_level(dvfs))
		return 0;

	mutex_lock(&(dvfs->dvfs_mutex));
	ret = set_dvfs_level_internal(dvfs, level);
	mutex_unlock(&(dvfs->dvfs_mutex));
	if (ret) {
		E("fail to set level. ret:%d", ret);
		goto EXIT;
	}

EXIT:
	return ret;
}

bool kbase_platform_dvfs_is_enabled(struct kbase_device *kbdev)
{
	struct rk_dvfs_t *dvfs = get_rk_dvfs(kbdev);

	return dvfs->is_enabled;
}

void kbase_platform_dvfs_enable(struct kbase_device *kbdev)
{
	struct rk_dvfs_t *dvfs = get_rk_dvfs(kbdev);

	mutex_lock(&(dvfs->dvfs_mutex));
	dvfs->is_enabled = true;
	mutex_unlock(&(dvfs->dvfs_mutex));
}

void kbase_platform_dvfs_disable(struct kbase_device *kbdev)
{
	struct rk_dvfs_t *dvfs = get_rk_dvfs(kbdev);

	cancel_work_sync(&(dvfs->mali_dvfs_work));

	mutex_lock(&(dvfs->dvfs_mutex));
	dvfs->is_enabled = false;
	mutex_unlock(&(dvfs->dvfs_mutex));
}

int kbase_platform_dvfs_set_clk_lowest(struct kbase_device *kbdev)
{
	int ret = 0;
	struct rk_dvfs_t *dvfs = get_rk_dvfs(kbdev);

	mutex_lock(&(dvfs->dvfs_mutex));
	ret = set_dvfs_level_internal(dvfs, get_lowest_level(dvfs));
	mutex_unlock(&(dvfs->dvfs_mutex));
	if (ret) {
		E("fail to set clk_lowest, ret:%d.", ret);
		goto EXIT;
	}

EXIT:
	return ret;
}

int kbase_platform_dvfs_set_clk_highest(struct kbase_device *kbdev)
{
	int ret = 0;
	struct rk_dvfs_t *dvfs = get_rk_dvfs(kbdev);

	mutex_lock(&(dvfs->dvfs_mutex));
	ret = set_dvfs_level_internal(dvfs, get_highest_level_available(dvfs));
	mutex_unlock(&(dvfs->dvfs_mutex));
	if (ret) {
		E("fail to set clk_highest, ret:%d.", ret);
		goto EXIT;
	}

EXIT:
	return ret;
}

int kbase_platform_dvfs_get_utilisation(struct kbase_device *kbdev)
{
	struct rk_dvfs_t *dvfs = get_rk_dvfs(kbdev);

	return dvfs->utilisation;
}

int kbase_platform_dvfs_get_dvfs_level_table(struct kbase_device *dvfs,
					     struct mali_dvfs_level_t **table,
					     unsigned int *num)
{
	*table = s_mali_dvfs_level_table;
	*num = s_num_of_mali_dvfs_levels;
	return 0;
}

/*---------------------------------------------------------------------------*/

#define dividend 7
#define fix_float(a) \
	((((a) * dividend) % 10) \
	? ((((a) * dividend) / 10) + 1) \
	: (((a) * dividend) / 10))
/*
 * Calculate and set 'min_threshold' and 'max_threshold' for the level_entry
 * in mali_dvfs_level_table, whose index is 'level'.
 */
static void calculate_dvfs_max_min_threshold(u32 level)
{
	u32 pre_level;
	u32 tmp;
	struct mali_dvfs_level_t *table = s_mali_dvfs_level_table;

	if (0 == level) {
		if ((s_num_of_mali_dvfs_levels - 1) == level) {
			table[level].min_threshold = level0_min;
			table[level].max_threshold = levelf_max;
		} else {
			table[level].min_threshold = level0_min;
			table[level].max_threshold = level0_max;
		}
	} else {
		pre_level = level - 1;
		if ((s_num_of_mali_dvfs_levels - 1) == level) {
			table[level].max_threshold = levelf_max;
		} else {
			table[level].max_threshold
				= table[pre_level].max_threshold
				+ div_dvfs;
		}
		table[level].min_threshold
			= (table[pre_level].max_threshold
					* (table[pre_level].freq / 1000))
			/ (table[level].freq / 1000);

		tmp = table[level].max_threshold - table[level].min_threshold;

		table[level].min_threshold += fix_float(tmp);
	}

	I("dvfs_level_table[%d]: freq:%d, min_threshold:%d, max_threshold:%d",
	  level,
	  table[level].freq,
	  table[level].min_threshold,
	  table[level].max_threshold);
}

/*
 * The caller must hold 'dvfs_mutex' when calling this function.
 */
static int set_dvfs_level_internal(struct rk_dvfs_t *dvfs, int target_level)
{
	int ret;
	struct rk_context *platform = dvfs_to_rk_context(dvfs);
	struct kbase_device *kbdev = platform->kbdev;
	unsigned long target_freq = get_clk_freq_of_level(dvfs, target_level);

	ret = kbase_platform_set_freq_of_clk_gpu(kbdev, target_freq);
	if (ret) {
		E("fail to set freq to %lu kHz, ret:%d", target_freq, ret);
		goto EXIT;
	}

	dvfs->current_level = target_level;

EXIT:
	return ret;
}
