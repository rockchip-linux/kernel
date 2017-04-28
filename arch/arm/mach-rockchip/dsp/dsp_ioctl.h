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
#ifndef _ARCH_ARM_MACH_RK_DSP_H_
#define _ARCH_ARM_MACH_RK_DSP_H_
#include <linux/ioctl.h>

#define DSP_MAX_IMAGE         16

/*
 * DSP driver ioctl definition
 */
#define DSP_IOC_MAGIC             'x'
#define DSP_IOC_QUEUE_WORK         _IOW(DSP_IOC_MAGIC, 1, unsigned long)
#define DSP_IOC_DEQUEUE_WORK       _IOR(DSP_IOC_MAGIC, 2, unsigned long)

/*
 * DSP error codes which will be returned in result
 */
#define DSP_WORK_SUCCESS   0x00000000
#define DSP_WORK_EABANDON  0x80000001
#define DSP_WORK_ECOPY     0x80000002
#define DSP_WORK_ETIMEOUT  0x80000003
#define DSP_WORK_EUNKNOWN  0x8000ffff

/*
 * User work magic, used by DSP kernel driver to check
 * the work parameter is valid or not
 */
#define DSP_ALGORITHM_WORK_MAGIC 0x20462046
#define DSP_CONFIG_WORK_MAGIC    0x95279527

/*
 * DSP config types
 */
#define DSP_CONFIG_INIT   0x80000001

enum dsp_work_type {
	DSP_ALGORITHM_WORK = 1,
	DSP_CONFIG_WORK    = 2,
};

/*
 * dsp_algorithm_params - parameters used by DSP core to process
 * an algorithm request.
 *
 * @type: algorithm type, user specific, known by user application and DSP
 * @packet_phys: algorithm parameter packet physical address
 * @packet_virt: algorithm parameter packet virtual address
 * @size: size of algorithm parameter packet
 */
struct dsp_algorithm_params {
	u32 type;
	u32 packet_phys;
	u32 packet_virt;
	u32 size;
};

/*
 * dsp_config_params - parameters used to config DSP core
 *
 * @type: config type
 * @image_count: image count which DSP can use
 * @image_phy: image data phys address
 * @trace_buffer: a share buffer for DSP debug
 * @trace_buffer_size: trace buffer size
 * @trace_slot_size: trace slot size
 */
struct dsp_config_params {
	u32 type;
	u32 image_count;
	u32 image_phys[DSP_MAX_IMAGE];
	u32 trace_buffer;
	u32 trace_buffer_size;
	u32 trace_slot_size;
	u32 reserve[8];
};

/*
 * dsp_user_work - This struct is used by user ioctl
 *
 * @magic: work magic should be DSP_ALGORITHM_WORK_MAGIC
 * @id: user work id
 * @result: work result, if success result is 0
 * @algorithm: algorithm parameters
 */
struct dsp_user_work {
	u32 magic;
	u32 id;
	u32 result;

	struct dsp_algorithm_params algorithm;
};

#endif
