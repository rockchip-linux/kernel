/*
 * Rockchip SoC Mali-T764 DVFS driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

#ifndef _KBASE_DVFS_H_
#define _KBASE_DVFS_H_

#ifdef CONFIG_MALI_MIDGARD_DVFS

struct kbase_rk_dvfs {
	struct work_struct work;
	unsigned int utilisation;
	unsigned int current_level;
	bool enabled;
};

int kbase_rk_dvfs_init(struct kbase_device *kbdev);
void kbase_rk_dvfs_term(struct kbase_device *kbdev);
void kbase_rk_set_dvfs(struct kbase_device *kbdev, bool enable);

bool kbase_rk_dvfs_is_enabled(struct kbase_device *kbdev);
void kbase_rk_dvfs_enable(struct kbase_device *kbdev);
void kbase_rk_dvfs_disable(struct kbase_device *kbdev);

unsigned int kbase_rk_dvfs_utilisation(struct kbase_device *kbdev);

#else

struct kbase_rk_dvfs {
};

static inline int kbase_rk_dvfs_init(struct kbase_device *kbdev)
{
	return 0;
}

static inline void kbase_rk_dvfs_term(struct kbase_device *kbdev)
{
}

static inline void kbase_rk_set_dvfs(struct kbase_device *kbdev, bool enable)
{
}

static inline bool kbase_rk_dvfs_is_enabled(struct kbase_device *kbdev)
{
	return false;
}

static inline void kbase_rk_dvfs_enable(struct kbase_device *kbdev)
{
}

static inline void kbase_rk_dvfs_disable(struct kbase_device *kbdev)
{
}

unsigned int kbase_rk_dvfs_utilisation(struct kbase_device *kbdev)
{
	return 0;
}

#endif
#endif		/*_KBASE_DVFS_H_*/
