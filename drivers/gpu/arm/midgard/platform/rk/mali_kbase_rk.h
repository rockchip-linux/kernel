/*
 * Rockchip SoC Mali-T764 platform-dependent codes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
 */

#ifndef _KBASE_PLATFORM_H_
#define _KBASE_PLATFORM_H_

#include <linux/clk.h>
#include <mali_kbase.h>

#define KBASE_RK_GPU_FREQ_KHZ_MAX               600000
#define KBASE_RK_GPU_FREQ_KHZ_MIN               100000

#define KBASE_RK_JS_SCHEDULING_TICK_NS_DEBUG    15000000u
/* 15ms, an aggressive tick for testing purposes.
 * This will reduce performance significantly
 */
#define KBASE_RK_JS_SOFT_STOP_TICKS_DEBUG       1
/* between 15ms and 30ms before soft-stop a job */
#define KBASE_RK_JS_HARD_STOP_TICKS_SS_DEBUG    333
/* 5s before hard-stop */
#define KBASE_RK_JS_HARD_STOP_TICKS_NSS_DEBUG   100000
/* 1500s (25mins) before NSS hard-stop */
#define KBASE_RK_JS_RESET_TICKS_SS_DEBUG        500
/* 45s before resetting GPU, for a certain GLES2 test at
 * 128x128 (bound by combined vertex+tiler job)
 */
#define KBASE_RK_JS_RESET_TICKS_NSS_DEBUG       100166
/* 1502s before resetting GPU */
#define KBASE_RK_JS_RESET_TIMEOUT_MS            500
/* 3s before cancelling stuck jobs */

kbase_platform_funcs_conf kbase_rk_platform_funcs;

struct kbase_rk_fv {
	unsigned long freq;
	unsigned long volt;
};

struct kbase_rk {
	struct clk *clk;
	struct kbase_rk_fv *fv_table;
	unsigned int fv_table_length;
	bool is_powered;
};

#endif				/* _KBASE_PLATFORM_H_ */
