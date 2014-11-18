/*
 * Rockchip SoC Mali-T764 platform-dependent codes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/driver.h>
#include "mali_kbase_rk.h"

static int kbase_rk_rt_power_on_callback(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	int error;

	dev_dbg(kbdev->dev, "%s: enabling regulator\n", __func__);

	error = regulator_enable(kbase_rk->regulator);
	if (error) {
		dev_err(kbdev->dev, "failed to enable regulator: %d\n",
			error);
		return error;
	}

	return 0;
}

static void kbase_rk_rt_power_off_callback(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;

	dev_dbg(kbdev->dev, "%s: shutting down regulator\n", __func__);

	regulator_disable(kbase_rk->regulator);
}

static int kbase_rk_rt_idle_callback(struct kbase_device *kbdev)
{
	/*
	 * We are ready to power off as soon as the last PM use is
	 * dropped via pm_runtime_put[_sync].
	 */
	return 0;
}

static int kbase_rk_power_on_callback(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	int ret = 1; /* Assume GPU has been powered off */
	int error;

	if (kbase_rk->is_powered) {
		dev_warn(kbdev->dev,
			 "called %s for already powered device\n", __func__);
		return 0;
	}

	dev_dbg(kbdev->dev, "%s: powering on\n", __func__);

	if (pm_runtime_enabled(kbdev->dev)) {
		error = pm_runtime_get_sync(kbdev->dev);
		if (error < 0) {
			dev_err(kbdev->dev,
				"failed to runtime resume device: %d\n",
				error);
			return error;
		} else if (error == 1) {
			/*
			 * Let core know that the chip has not been
			 * powered off, so we can save on re-initialization.
			 */
			ret = 0;
		}
	} else {
		error = kbase_rk_rt_power_on_callback(kbdev);
		if (error)
			return error;
	}

	error = clk_enable(kbase_rk->clk);
	if (error) {
		dev_err(kbdev->dev, "failed to enable clock: %d\n", error);

		if (pm_runtime_enabled(kbdev->dev))
			pm_runtime_put_sync(kbdev->dev);
		else
			kbase_rk_rt_power_off_callback(kbdev);

		return error;
	}

	kbase_rk->is_powered = true;
	KBASE_TIMELINE_GPU_POWER(kbdev, 1);

	return ret;
}

static void kbase_rk_power_off_callback(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;

	if (!kbase_rk->is_powered) {
		dev_warn(kbdev->dev,
			 "called %s for powered off device\n", __func__);
		return;
	}

	dev_dbg(kbdev->dev, "%s: powering off\n", __func__);

	kbase_rk->is_powered = false;
	KBASE_TIMELINE_GPU_POWER(kbdev, 0);

	clk_disable(kbase_rk->clk);

	if (pm_runtime_enabled(kbdev->dev)) {
		pm_runtime_mark_last_busy(kbdev->dev);
		pm_runtime_put_autosuspend(kbdev->dev);
	} else {
		kbase_rk_rt_power_off_callback(kbdev);
	}
}

static mali_error kbase_rk_power_runtime_init_callback(
		struct kbase_device *kbdev)
{
	pm_runtime_set_autosuspend_delay(kbdev->dev, 200);
	pm_runtime_use_autosuspend(kbdev->dev);

	pm_runtime_set_active(kbdev->dev);
	pm_runtime_enable(kbdev->dev);

	return MALI_ERROR_NONE;
}

static void kbase_rk_power_runtime_term_callback(
		struct kbase_device *kbdev)
{
	pm_runtime_disable(kbdev->dev);
}

static kbase_pm_callback_conf kbase_rk_pm_callbacks = {
	.power_on_callback = kbase_rk_power_on_callback,
	.power_off_callback = kbase_rk_power_off_callback,

	.power_runtime_init_callback = kbase_rk_power_runtime_init_callback,
	.power_runtime_term_callback = kbase_rk_power_runtime_term_callback,
	.power_runtime_on_callback = kbase_rk_rt_power_on_callback,
	.power_runtime_off_callback = kbase_rk_rt_power_off_callback,
	.power_runtime_idle_callback = kbase_rk_rt_idle_callback,
};

int kbase_platform_early_init(void)
{
	/* Nothing needed at this stage */
	return 0;
}

static int kbase_rk_set_level(struct kbase_device *kbdev, int level)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	unsigned long volt;
	unsigned long freq;
	int ret;
	unsigned int current_level;

	mutex_lock(&kbase_rk->set_level_lock);

	current_level = kbase_rk->current_level;
	volt = kbase_rk->fv_table[level].volt;
	freq = kbase_rk->fv_table[level].freq;

	if (level == current_level) {
		mutex_unlock(&kbase_rk->set_level_lock);
		return 0;
	}
	if (level > current_level) {
		ret = regulator_set_voltage(kbase_rk->regulator, volt, volt);
		if (ret)
			goto err_set_level;

		ret = clk_set_rate(kbase_rk->clk, freq);
		if (ret) {
			volt = kbase_rk->fv_table[current_level].volt;
			regulator_set_voltage(kbase_rk->regulator, volt, volt);

			goto err_set_level;
		}
	} else {
		ret = clk_set_rate(kbase_rk->clk, freq);
		if (ret)
			goto err_set_level;

		ret = regulator_set_voltage(kbase_rk->regulator, volt, volt);
		if (ret) {
			freq = kbase_rk->fv_table[current_level].freq;
			clk_set_rate(kbase_rk->clk, freq);

			goto err_set_level;
		}
	}

	kbase_rk->current_level = level;

	mutex_unlock(&kbase_rk->set_level_lock);

	return 0;
err_set_level:
	mutex_unlock(&kbase_rk->set_level_lock);

	return ret;
}

/*
 * Set the gpu operating point (frequency and voltage) to a frequency no
 * greater than the requested frequency.
 *
 * The gpu can only operate at a certain fixed set of operating points
 * (frequency, voltage) pairs, as defined in the device's device tree.
 * This function looks through the operating point table and choose an entry
 * whose frequency is closest to, but not greater than the requested 'freq.
 * If a frequency is requested that is less than the lowest operating point,
 * the lowest frequency operating point is used.
 */
int kbase_rk_set_freq(struct kbase_device *kbdev, unsigned long freq)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	int ret;
	int level;

	/*
	 * Start at highest operating point, and iterate backwards.
	 * Stop when we find a frequency <= freq, or no more entries.
	 */
	for (level = kbase_rk->fv_table_length - 1; level > 0; level--) {
		struct kbase_rk_fv *fv  = &kbase_rk->fv_table[level];
		if (fv->freq <= freq)
			break;
	}

	dev_dbg(kbdev->dev, "Using operating point %d: (%lu Hz, %lu mV) for %lu Hz\n",
		level, kbase_rk->fv_table[level].freq,
		kbase_rk->fv_table[level].volt, freq);

	ret = kbase_rk_set_level(kbdev, level);
	if (ret) {
		dev_err(kbdev->dev, "set level error, %d\n", ret);
		return ret;
	}

	return 0;
}

#ifdef CONFIG_MALI_MIDGARD_DEBUG_SYS

static ssize_t show_available_frequencies(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	ssize_t ret = 0;
	u32 i;

	for (i = 0; i < kbase_rk->fv_table_length; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%lu\n",
				 kbase_rk->fv_table[i].freq);

	return ret;
}

static ssize_t show_clock(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);
	struct kbase_rk *kbase_rk = kbdev->platform_context;

	return scnprintf(buf, PAGE_SIZE, "%lu\n", clk_get_rate(kbase_rk->clk));
}

static ssize_t set_clock(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);
	unsigned long freq;
	ssize_t ret;

	ret = kstrtoul(buf, 10, &freq);
	if (ret)
		return ret;

	ret = kbase_rk_set_freq(kbdev, freq);
	if (ret)
		return ret;

	return count;
}

static ssize_t show_dvfs_enable(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			kbase_rk_dvfs_is_enabled(kbdev));
}

static ssize_t set_dvfs_enable(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);
	unsigned long enable;
	ssize_t ret;

	ret = kstrtoul(buf, 0, &enable);
	if (ret)
		return ret;

	if (enable == 1)
		kbase_rk_dvfs_enable(kbdev);
	else if (enable == 0)
		kbase_rk_dvfs_disable(kbdev);
	else
		return -EINVAL;

	return count;
}

static ssize_t show_memory(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%lu bytes\n",
			 atomic_read(&kbdev->memdev.used_pages) * PAGE_SIZE);
}

static ssize_t show_utilisation(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			kbase_rk_dvfs_utilisation(kbdev));
}

DEVICE_ATTR(available_frequencies, S_IRUGO, show_available_frequencies, NULL);
DEVICE_ATTR(clock, S_IRUGO | S_IWUSR, show_clock, set_clock);
DEVICE_ATTR(dvfs_enable, S_IRUGO | S_IWUSR, show_dvfs_enable, set_dvfs_enable);
DEVICE_ATTR(memory, S_IRUGO, show_memory, NULL);
DEVICE_ATTR(utilisation, S_IRUGO, show_utilisation, NULL);

static struct attribute *mali_kbase_rk_sysfs_entries[] = {
	&dev_attr_available_frequencies.attr,
	&dev_attr_clock.attr,
	&dev_attr_dvfs_enable.attr,
	&dev_attr_memory.attr,
	&dev_attr_utilisation.attr,
	NULL,
};

static const struct attribute_group mali_kbase_rk_attr_group = {
	.attrs	= mali_kbase_rk_sysfs_entries,
};

static int kbase_rk_create_sysfs(struct kbase_device *kbdev)
{
	int ret;

	ret = sysfs_create_group(&kbdev->dev->kobj, &mali_kbase_rk_attr_group);
	if (ret)
		dev_err(kbdev->dev, "create sysfs group error, %d\n", ret);

	return ret;
}

void kbase_rk_remove_sysfs(struct kbase_device *kbdev)
{
	sysfs_remove_group(&kbdev->dev->kobj, &mali_kbase_rk_attr_group);
}

#else

static inline int kbase_rk_create_sysfs(struct kbase_device *kbdev)
{
	return 0;
}

static inline void kbase_rk_remove_sysfs(struct kbase_device *kbdev)
{
}

#endif

static int kbase_rk_freq_init(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	unsigned long volt = kbase_rk->fv_table[0].volt;
	unsigned long freq = kbase_rk->fv_table[0].freq;
	int ret;

	ret = regulator_set_voltage(kbase_rk->regulator, volt, volt);
	if (ret)
		return ret;

	ret = clk_set_rate(kbase_rk->clk, freq);
	if (ret)
		return ret;

	kbase_rk->current_level = 0;
	dev_info(kbdev->dev, "initial freq = %lu\n",
		 clk_get_rate(kbase_rk->clk));

	return 0;
}

static int kbase_rk_get_opp_table(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	const struct property *prop;
	int nr;
	const __be32 *val;
	int i;

	prop = of_find_property(kbdev->dev->of_node, "operating-points", NULL);
	if (!prop)
		return -ENODEV;
	if (!prop->value)
		return -ENODATA;
	/*
	 * Each OPP is a set of tuples consisting of frequency and
	 * voltage like <freq-kHz vol-uV>.
	 */
	nr = prop->length / sizeof(u32);
	if (nr % 2) {
		dev_err(kbdev->dev, "Invalid OPP list\n");
		return -EINVAL;
	}

	kbase_rk->fv_table_length = nr / 2;
	kbase_rk->fv_table = devm_kcalloc(kbdev->dev,
			kbase_rk->fv_table_length, sizeof(*kbase_rk->fv_table),
			GFP_KERNEL);
	if (!kbase_rk->fv_table)
		return -ENOMEM;

	val = prop->value;

	for (i = 0; i < kbase_rk->fv_table_length; ++i) {
		unsigned long freq = be32_to_cpup(val++) * 1000;
		unsigned long volt = be32_to_cpup(val++);

		kbase_rk->fv_table[i].freq = freq;
		kbase_rk->fv_table[i].volt = volt;
		dev_info(kbdev->dev, "freq:%lu Hz volt:%lu uV\n", freq, volt);
	}

	return 0;
}

static int kbase_rk_regulator_init(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	int ret;

	kbase_rk->regulator = devm_regulator_get(kbdev->dev, "vdd_gpu");
	if (IS_ERR(kbase_rk->regulator)) {
		ret = PTR_ERR(kbase_rk->regulator);
		dev_err(kbdev->dev,
			"failed to get kbase rk regulator, %d\n", ret);
		return ret;
	}

	return 0;
}

static int kbase_rk_clk_init(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	int ret;

	kbase_rk->clk = devm_clk_get(kbdev->dev, "aclk_gpu");
	if (IS_ERR(kbase_rk->clk)) {
		ret = PTR_ERR(kbase_rk->clk);
		dev_err(kbdev->dev, "get clk failed, %d\n", ret);
		return ret;
	}

	ret = kbase_rk_freq_init(kbdev);
	if (ret)
		return ret;

	ret = clk_prepare(kbase_rk->clk);
	if (ret) {
		dev_err(kbdev->dev, "prepare clk failed, %d\n", ret);
		return ret;
	}

	return 0;
}

static void kbase_rk_clk_term(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;

	clk_unprepare(kbase_rk->clk);
}

static mali_bool kbase_rk_platform_init(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk;
	int ret;

	kbase_rk = devm_kzalloc(kbdev->dev, sizeof(*kbase_rk), GFP_KERNEL);

	if (!kbase_rk)
		return MALI_FALSE;

	kbdev->platform_context = kbase_rk;
	kbase_rk->kbdev = kbdev;
	mutex_init(&kbase_rk->set_level_lock);

	ret = kbase_rk_get_opp_table(kbdev);
	if (ret)
		goto err_init;

	ret = kbase_rk_regulator_init(kbdev);
	if (ret)
		goto err_init;

	ret = kbase_rk_clk_init(kbdev);
	if (ret)
		goto err_init;

	ret = kbase_rk_create_sysfs(kbdev);
	if (ret)
		goto term_clk;

	ret = kbase_rk_dvfs_init(kbdev);
	if (ret)
		goto remove_sysfs;

	return MALI_TRUE;
remove_sysfs:
	kbase_rk_remove_sysfs(kbdev);
term_clk:
	kbase_rk_clk_term(kbdev);
err_init:
	kbdev->platform_context = NULL;

	return MALI_FALSE;
}

static void kbase_rk_platform_term(struct kbase_device *kbdev)
{
	kbase_rk_remove_sysfs(kbdev);
	kbase_rk_clk_term(kbdev);
	kbase_rk_dvfs_term(kbdev);
	kbdev->platform_context = NULL;
}

kbase_platform_funcs_conf kbase_rk_platform_funcs = {
	.platform_init_func = kbase_rk_platform_init,
	.platform_term_func = kbase_rk_platform_term,
};

static kbase_attribute kbase_rk_config_attributes[] = {
	{
	 KBASE_CONFIG_ATTR_POWER_MANAGEMENT_CALLBACKS,
	 (uintptr_t)&kbase_rk_pm_callbacks
	}, {
	 KBASE_CONFIG_ATTR_PLATFORM_FUNCS,
	 (uintptr_t) &kbase_rk_platform_funcs
	},
#ifdef CONFIG_MALI_DEBUG
	/* Use more aggressive scheduling timeouts in debug builds. */
	{
	 KBASE_CONFIG_ATTR_JS_SCHEDULING_TICK_NS,
	 KBASE_RK_JS_SCHEDULING_TICK_NS_DEBUG
	}, {
	 KBASE_CONFIG_ATTR_JS_SOFT_STOP_TICKS,
	 KBASE_RK_JS_SOFT_STOP_TICKS_DEBUG
	}, {
	 KBASE_CONFIG_ATTR_JS_HARD_STOP_TICKS_SS,
	 KBASE_RK_JS_HARD_STOP_TICKS_SS_DEBUG
	}, {
	 KBASE_CONFIG_ATTR_JS_HARD_STOP_TICKS_NSS,
	 KBASE_RK_JS_HARD_STOP_TICKS_NSS_DEBUG
	}, {
	 KBASE_CONFIG_ATTR_JS_RESET_TICKS_SS,
	 KBASE_RK_JS_RESET_TICKS_SS_DEBUG
	}, {
	 KBASE_CONFIG_ATTR_JS_RESET_TICKS_NSS,
	 KBASE_RK_JS_RESET_TICKS_NSS_DEBUG
	},
#endif /* CONFIG_MALI_DEBUG */
	{
	 KBASE_CONFIG_ATTR_JS_RESET_TIMEOUT_MS,
	 KBASE_RK_JS_RESET_TIMEOUT_MS
	}, {
	 KBASE_CONFIG_ATTR_END,
	 0
	}
};

static kbase_platform_config kbase_rk_platform_config = {
	.attributes = kbase_rk_config_attributes,
};

kbase_platform_config *kbase_get_platform_config(void)
{
	return &kbase_rk_platform_config;
}
