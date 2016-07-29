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
#ifndef _ARCH_ARM_MACH_RK_DSP_DMA_H_
#define _ARCH_ARM_MACH_RK_DSP_DMA_H_
#include <linux/types.h>

struct dsp_dma {
	void __iomem *base;

	int (*transfer_data)(struct dsp_dma *, void *, void *, size_t);
	int (*transfer_code)(struct dsp_dma *, void *, void *, size_t);
};

int dsp_dma_create(void __iomem *dma_base, struct dsp_dma **dma);

int dsp_dma_destroy(struct dsp_dma *dma);

#endif
