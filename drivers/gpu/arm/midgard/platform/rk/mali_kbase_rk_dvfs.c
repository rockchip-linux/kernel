/*
 * Rockchip SoC Mali-T764 DVFS driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

#include "mali_kbase_rk.h"
#include "mali_kbase_rk_dvfs.h"

struct kbase_rk_dvfs_threshold {
	unsigned long freq;
	unsigned int min;
	unsigned int max;
};

/*
 * if current_utilisation > max
 * level++
 * if current_utilisation < min
 * level--
 * Length of kbase_rk_dvfs_threshold_table must
 * equal to the length of the gpu's operating-point table
 */
static const struct kbase_rk_dvfs_threshold kbase_rk_dvfs_threshold_table[] = {
	{ 100000000, 0, 50 },
	{ 200000000, 20, 40 },
	{ 300000000, 20, 40 },
	{ 400000000, 20, 40 },
	{ 500000000, 20, 40 },
	{ 600000000, 20, 100 },
};

#define work_to_dvfs(w) container_of(w, struct kbase_rk_dvfs, work)
#define dvfs_to_kbase_rk(dvfs) container_of(dvfs, struct kbase_rk, dvfs)

static void kbase_rk_dvfs_event_proc(struct work_struct *w)
{
	struct kbase_rk_dvfs *dvfs = work_to_dvfs(w);
	struct kbase_rk *kbase_rk = dvfs_to_kbase_rk(dvfs);
	struct kbase_device *kbdev = kbase_rk->kbdev;
	unsigned int utilisation = dvfs->utilisation;
	unsigned int level = dvfs->current_level;
	const struct kbase_rk_dvfs_threshold *threshold =
				&kbase_rk_dvfs_threshold_table[level];
	int ret;
	unsigned long target_freq;

	dev_dbg(kbdev->dev, "utilisation = %d\n", kbase_rk->dvfs.utilisation);

	if (utilisation > threshold->max &&
	    level < ARRAY_SIZE(kbase_rk_dvfs_threshold_table) - 1)
		level += 1;
	else if (level > 0 && utilisation < threshold->min)
		level -= 1;
	else
		return;

	target_freq = kbase_rk_dvfs_threshold_table[level].freq;
	dev_dbg(kbdev->dev, "Setting dvfs level %u: freq = %lu Hz\n",
		level, target_freq);

	ret = kbase_rk_set_freq(kbdev, target_freq);
	if (ret) {
		dev_err(kbdev->dev, "set freq error, %d", ret);
		return;
	}

	kbase_rk->dvfs.current_level = level;
}

bool kbase_rk_dvfs_is_enabled(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	struct kbase_rk_dvfs *dvfs = &kbase_rk->dvfs;

	return dvfs->enabled;
}

void kbase_rk_dvfs_enable(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	struct kbase_rk_dvfs *dvfs = &kbase_rk->dvfs;

	dvfs->enabled = true;
}

void kbase_rk_dvfs_disable(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	struct kbase_rk_dvfs *dvfs = &kbase_rk->dvfs;

	dvfs->enabled = false;
	cancel_work_sync(&dvfs->work);
}

unsigned int kbase_rk_dvfs_utilisation(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	struct kbase_rk_dvfs *dvfs = &kbase_rk->dvfs;

	return dvfs->utilisation;
}

int kbase_platform_dvfs_event(struct kbase_device *kbdev, u32 utilisation,
		u32 util_gl_share, u32 util_cl_share[2])
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	struct kbase_rk_dvfs *dvfs = &kbase_rk->dvfs;

	dvfs->utilisation = utilisation;

	if (dvfs->enabled)
		schedule_work(&dvfs->work);

	return MALI_TRUE;
}

int kbase_rk_dvfs_init(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	struct kbase_rk_dvfs *dvfs = &kbase_rk->dvfs;

	INIT_WORK(&dvfs->work, kbase_rk_dvfs_event_proc);
	dvfs->enabled = true;

	return 0;
}

void kbase_rk_dvfs_term(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	struct kbase_rk_dvfs *dvfs = &kbase_rk->dvfs;

	cancel_work_sync(&dvfs->work);
}
