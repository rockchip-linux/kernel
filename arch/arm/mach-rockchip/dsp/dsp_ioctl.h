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

#define DSP_MAX_IMAGE         8

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

/*
 * User work magic, used by DSP kernel driver to check
 * the work parameter is valid or not
 */
#define DSP_RENDER_WORK_MAGIC 0x20462046
#define DSP_CONFIG_WORK_MAGIC 0x95279527

/*
 * DSP render types. User application should set type
 * in the type member of struct dsp_render_work
 */
#define DSP_RENDER_COPY    0x00000001
#define DSP_RENDER_ADAS    0x00000002
#define DSP_RENDER_2DNR    0x00000004
#define DSP_RENDER_3DNR    0x00000008
#define DSP_RENDER_HDR     0x00000010
#define DSP_RENDER_DEFOG   0x00000020
#define DSP_RENDER_LLE     0x00000040

/*
 * DSP config types
 */
#define DSP_CONFIG_INIT   0x80000001

enum dsp_work_type {
	DSP_RENDER_WORK = 1,
	DSP_CONFIG_WORK = 2,
};

/*
 * dsp_render_params - parameters used by DSP core
 * hardware to render a frame
 *
 * @type: render type, DSP_RENDER_3DNR etc
 * @packet_phys: render algrithm config packet phys address
 * @packet_virt: packet virt address
 * @size: render algrithm config packet size
 */
struct dsp_render_params {
	u32 type;
	u32 packet_phys;
	u32 packet_virt;
	u32 size;
};

/*
 * dsp_config_params - parameters used to config
 * DSP core
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
 * @magic: work magic should be DSP_RENDER_WORK_MAGIC
 * @hdl: user handle
 * @result: work result, if success result is 0
 * @render: render params
 */
struct dsp_user_work {
	u32 magic;
	u32 hdl;
	u32 result;

	struct dsp_render_params render;
};

#endif
