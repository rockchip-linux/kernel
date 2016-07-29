/**
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd
 * author: ZhiChao Yu zhichao.yu@rock-chips.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _ARCH_ARM_MACH_RK_DSP_DBG_H_
#define _ARCH_ARM_MACH_RK_DSP_DBG_H_

/*
 * debug mask usage:
 * +------+-------------------+
 * | 8bit |      24bit        |
 * +------+-------------------+
 *  0~23 bit is for different information type
 * 24~31 bit is for information print format
 */
extern u32 dsp_debug_mask;

#define DEBUG
#define DEBUG_FUNCTION         0x00000001
#define DEBUG_INIT             0x00000002
#define DEBUG_DEVICE           0x00000004
#define DEBUG_SERVICE          0x00000008
#define DEBUG_MBOX             0x00000010
#define DEBUG_LOADER           0x00000020

#define PRINT_FUNCTION         0x80000000
#define PRINT_LINE             0x40000000

#define DEBUG
#ifdef DEBUG
#define dsp_debug_func(type, fmt, args...)                      \
	do {                                                    \
		if (dsp_debug_mask & type)                      \
			pr_info("%s: " fmt, __func__, ##args);	\
	} while (0)

#define dsp_debug(type, fmt, args...)                           \
	do {                                                    \
		if (dsp_debug_mask & type)                      \
			pr_info("%s: " fmt, __func__, ##args);  \
	} while (0)
#else
#define dsp_debug_func(level, fmt, args...)
#define dsp_debug(level, fmt, args...)
#endif

#define dsp_debug_enter() dsp_debug_func(DEBUG_FUNCTION, "enter\n")
#define dsp_debug_leave() dsp_debug_func(DEBUG_FUNCTION, "leave\n")

#define dsp_err(fmt, args...)                              \
		pr_err("%s: " fmt, __func__, ##args)

#endif
