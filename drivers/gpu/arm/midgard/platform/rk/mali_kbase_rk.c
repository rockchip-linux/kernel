/*
 * Rockchip SoC Mali-T764 platform-dependent codes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

#include <linux/of.h>

#include "mali_kbase_rk.h"

static int kbase_rk_power_on_callback(struct kbase_device *kbdev)
{
	int ret;
	struct kbase_rk *kbase_rk = kbdev->platform_context;

	if (kbase_rk->is_powered)
		return 0;

	ret = clk_enable(kbase_rk->clk);
	if (ret) {
		dev_err(kbdev->dev, "enable clk failed, %d\n", ret);
		return ret;
	}

	kbase_rk->is_powered = true;
	KBASE_TIMELINE_GPU_POWER(kbdev, 1);

	return 0;
}

static void kbase_rk_power_off_callback(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;

	if (!kbase_rk->is_powered)
		return;

	clk_disable(kbase_rk->clk);
	kbase_rk->is_powered = false;
	KBASE_TIMELINE_GPU_POWER(kbdev, 0);
}

static kbase_pm_callback_conf kbase_rk_pm_callbacks = {
	.power_on_callback = kbase_rk_power_on_callback,
	.power_off_callback = kbase_rk_power_off_callback,
};

static kbase_attribute kbase_rk_config_attributes[] = {
	{
	 KBASE_CONFIG_ATTR_POWER_MANAGEMENT_CALLBACKS,
	 (uintptr_t)&kbase_rk_pm_callbacks
	}, {
	 KBASE_CONFIG_ATTR_PLATFORM_FUNCS,
	 (uintptr_t) &kbase_rk_platform_funcs
	}, {
	 KBASE_CONFIG_ATTR_GPU_FREQ_KHZ_MAX,
	 KBASE_RK_GPU_FREQ_KHZ_MAX
	}, {
	 KBASE_CONFIG_ATTR_GPU_FREQ_KHZ_MIN,
	 KBASE_RK_GPU_FREQ_KHZ_MIN
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

int kbase_platform_early_init(void)
{
	/* Nothing needed at this stage */
	return 0;
}

#ifdef CONFIG_MALI_MIDGARD_DEBUG_SYS
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
	struct kbase_rk *kbase_rk = kbdev->platform_context;
	unsigned long  clkrate;
	ssize_t ret;

	ret = kstrtoul(buf, 10, &clkrate);
	if (ret)
		return ret;

	/* TODO(xxm): allow all freq in opp table once voltage can be set */
	if (clkrate != kbase_rk->fv_table[0].freq)
		return -EINVAL;

	ret = clk_set_rate(kbase_rk->clk, clkrate);
	if (ret)
		return ret;

	return count;
}

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

static ssize_t show_memory(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%lu bytes\n",
			 atomic_read(&kbdev->memdev.used_pages) * PAGE_SIZE);
}

DEVICE_ATTR(memory, S_IRUGO, show_memory, NULL);
DEVICE_ATTR(clock, S_IRUGO | S_IWUSR, show_clock, set_clock);
DEVICE_ATTR(available_frequencies, S_IRUGO, show_available_frequencies, NULL);

static int kbase_rk_create_sysfs_file(struct kbase_device *kbdev)
{
	int ret;

	ret = device_create_file(kbdev->dev, &dev_attr_clock);
	if (ret) {
		dev_err(kbdev->dev, "Couldn't create sysfs file [clock], %d\n",
			ret);
		return ret;
	}

	ret = device_create_file(kbdev->dev, &dev_attr_available_frequencies);
	if (ret) {
		dev_err(kbdev->dev, "Couldn't create sysfs file [available_frequencies], %d\n",
			ret);
		goto err_remove_clock;
	}

	ret = device_create_file(kbdev->dev, &dev_attr_memory);
	if (ret) {
		dev_err(kbdev->dev, "Couldn't create sysfs file [memory], %d\n",
			ret);
		goto err_remove_available_frequencies;
	}

	return 0;

err_remove_available_frequencies:
	device_remove_file(kbdev->dev, &dev_attr_available_frequencies);
err_remove_clock:
	device_remove_file(kbdev->dev, &dev_attr_clock);

	return ret;
}

void kbase_rk_remove_sysfs_file(struct kbase_device *kbdev)
{
	device_remove_file(kbdev->dev, &dev_attr_memory);
	device_remove_file(kbdev->dev, &dev_attr_available_frequencies);
	device_remove_file(kbdev->dev, &dev_attr_clock);
}
#else
static inline int kbase_rk_create_sysfs_file(struct kbase_device *kbdev)
{
	return 0;
}

static inline void kbase_rk_remove_sysfs_file(struct kbase_device *kbdev)
{
}
#endif

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

static mali_bool kbase_rk_platform_init(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk;
	int ret;

	kbase_rk = devm_kzalloc(kbdev->dev, sizeof(*kbase_rk), GFP_KERNEL);

	if (!kbase_rk)
		return MALI_FALSE;

	kbdev->platform_context = kbase_rk;

	ret = kbase_rk_get_opp_table(kbdev);
	if (ret)
		return MALI_FALSE;

	kbase_rk->clk = devm_clk_get(kbdev->dev, "aclk_gpu");
	if (IS_ERR(kbase_rk->clk)) {
		ret = PTR_ERR(kbase_rk->clk);
		dev_err(kbdev->dev, "get clk failed, %d\n", ret);
		return MALI_FALSE;
	}

	ret = clk_set_rate(kbase_rk->clk, kbase_rk->fv_table[0].freq);
	if (ret)
		return MALI_FALSE;

	ret = clk_prepare(kbase_rk->clk);
	if (ret) {
		dev_err(kbdev->dev, "prepare clk failed, %d\n", ret);
		return MALI_FALSE;
	}

	dev_info(kbdev->dev, "initial freq = %lu\n",
		 clk_get_rate(kbase_rk->clk));

	ret = kbase_rk_create_sysfs_file(kbdev);
	if (ret)
		goto unprepare_clk;

	return MALI_TRUE;
unprepare_clk:
	clk_unprepare(kbase_rk->clk);
	return MALI_FALSE;
}

static void kbase_rk_platform_term(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;

	clk_unprepare(kbase_rk->clk);
	kbase_rk_remove_sysfs_file(kbdev);
	kbdev->platform_context = NULL;
}

kbase_platform_funcs_conf kbase_rk_platform_funcs = {
	.platform_init_func = kbase_rk_platform_init,
	.platform_term_func = kbase_rk_platform_term,
};
