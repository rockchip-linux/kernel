/*
 * Rockchip SoC Mali-T764 platform-dependent codes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

#include <mali_kbase_rk.h>

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

static mali_bool kbase_rk_platform_init(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk;
	int ret;

	kbase_rk = devm_kzalloc(kbdev->dev, sizeof(*kbase_rk), GFP_KERNEL);

	if (!kbase_rk)
		return MALI_FALSE;

	kbase_rk->clk = devm_clk_get(kbdev->dev, "aclk_gpu");
	if (IS_ERR(kbase_rk->clk)) {
		ret = PTR_ERR(kbase_rk->clk);
		dev_err(kbdev->dev, "get clk failed, %d\n", ret);
		return MALI_FALSE;
	}

	dev_err(kbdev->dev, "initial clk = %lu\n", clk_get_rate(kbase_rk->clk));

	ret = clk_prepare(kbase_rk->clk);
	if (ret) {
		dev_err(kbdev->dev, "prepare clk failed, %d\n", ret);
		return MALI_FALSE;
	}

	kbdev->platform_context = kbase_rk;

	return MALI_TRUE;
}

static void kbase_rk_platform_term(struct kbase_device *kbdev)
{
	struct kbase_rk *kbase_rk = kbdev->platform_context;

	clk_unprepare(kbase_rk->clk);
	kbdev->platform_context = NULL;
}

kbase_platform_funcs_conf kbase_rk_platform_funcs = {
	.platform_init_func = kbase_rk_platform_init,
	.platform_term_func = kbase_rk_platform_term,
};
