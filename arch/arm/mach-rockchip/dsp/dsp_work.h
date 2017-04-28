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
#ifndef _ARCH_ARM_MACH_RK_DSP_WORK_H_
#define _ARCH_ARM_MACH_RK_DSP_WORK_H_
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include "dsp_ioctl.h"

struct dsp_work;

enum dsp_work_status {
	DSP_WORK_PENDING,
	DSP_WORK_RUNNING,
	DSP_WORK_DONE,
};

/*
 * dsp_work_hw - This struct is used by DSP core
 *
 * @id: user work id
 * @type: work type, see enum dsp_work_type
 * @result: work result return by DSP core
 * @rate: inform DSP its real rate
 * @cycles: work cost cycles returned by DSP core
 * @params: work parameters
 */
struct dsp_work_hw {
	u32 id;
	u32 type;
	u32 result;
	u32 rate;
	u32 cycles;

	union {
		struct dsp_algorithm_params algorithm;
		struct dsp_config_params config;
	} params;
};

/*
 * dsp_work - A work prensents jobs that DSP can do.
 * Currently there has two types of work, render and config.
 *
 * @id: user work id
 * @type: work type, see enum dsp_work_type
 * @result: work result return by DSP core
 * @rate: inform DSP its real rate
 * @cycles: work cost cycles returned by DSP core
 * @params: work parameters
 * @session: session address
 * @status: work status
 * @dma_addr: work dma address
 * @list_node: list node
 */
struct dsp_work {
	u32 id;
	u32 type;
	u32 result;
	u32 rate;
	u32 cycles;

	union {
		struct dsp_algorithm_params algorithm;
		struct dsp_config_params config;
	} params;

	/*
	 * Members below are used by DSP kernel driver.
	 * User and hardware do not need to know anything
	 * about these members.
	 */
	u32 session;
	u32 status;
	u32 dma_addr;
	struct list_head list_node;
};

int dsp_work_set_type(struct dsp_work *work, enum dsp_work_type type);

int dsp_work_set_params(struct dsp_work *work, void *params);

int dsp_work_set_status(struct dsp_work *work, u32 status);

int dsp_work_copy_from_user(struct dma_pool *dma_pool, struct dsp_work *work,
			    void *user);

int dsp_work_copy_to_user(struct dsp_work *work, void *user);

int dsp_work_create(struct dma_pool *dma_pool, void *session,
		    struct dsp_work **work_out);

int dsp_work_destroy(struct dma_pool *dma_pool, struct dsp_work *work);

#endif
