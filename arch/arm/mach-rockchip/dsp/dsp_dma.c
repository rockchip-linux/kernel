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
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include "dsp_dma.h"
#include "dsp_dbg.h"

#define DSP_DMA_PDEA    0x408
#define DSP_DMA_PDIA    0x40c
#define DSP_DMA_PDTC    0x410
#define DSP_DMA_DDEA    0x61c
#define DSP_DMA_DDIA    0x620
#define DSP_DMA_DDTC    0x624
#define DSP_DMA_2DCFG1  0x644
#define DSP_DMA_2DCFG2  0x648

int dsp_dma_transfer_code(struct dsp_dma *dma, void *src,
			  void *dst, size_t size)
{
	int ret = 0;
	int timeout = 100;
	u32 pdtc;

	dsp_debug_enter();

	writel_relaxed(virt_to_phys(src), dma->base + DSP_DMA_PDEA);
	writel_relaxed(dst, dma->base + DSP_DMA_PDIA);
	writel_relaxed((size & 0x1fffe0), dma->base + DSP_DMA_PDTC);

	/* Wait here until all code is tansfered to DSP */
	do {
		pdtc = readl_relaxed(dma->base + DSP_DMA_PDTC);
		if (timeout-- < 0) {
			dsp_err("transfer code to dsp timeout\n");
			ret = -EIO;
			goto out;
		}

		usleep_range(100, 500);
	} while ((pdtc & (1 << 29)));

out:
	dsp_debug_leave();
	return ret;
}

int dsp_dma_transfer_data(struct dsp_dma *dma, void *src,
			  void *dst, size_t size)
{
	int ret = 0;
	int timeout = 100;
	u32 ddtc;

	dsp_debug_enter();

	writel_relaxed(virt_to_phys(src), dma->base + DSP_DMA_DDEA);
	writel_relaxed(dst, dma->base + DSP_DMA_DDIA);
	writel_relaxed(0, dma->base + DSP_DMA_2DCFG1);
	writel_relaxed(0, dma->base + DSP_DMA_2DCFG2);
	ddtc = (5 << 25) | size;
	writel_relaxed(ddtc, dma->base + DSP_DMA_DDTC);

	/* Wait here until all data is tansfered to DSP */
	do {
		ddtc = readl_relaxed(dma->base + DSP_DMA_DDTC);
		if (timeout-- < 0) {
			dsp_err("transfer data to dsp timeout\n");
			ret = -EIO;
			goto out;
		}

		usleep_range(100, 500);
	} while ((ddtc & (1 << 29)));

out:
	dsp_debug_leave();
	return ret;
}

int dsp_dma_create(void __iomem *dma_base, struct dsp_dma **dma_out)
{
	int ret = 0;
	struct dsp_dma *dma;

	dsp_debug_enter();

	dma = kzalloc(sizeof(*dma), GFP_KERNEL);
	if (!dma) {
		dsp_err("cannot alloc mem for dsp dma\n");
		ret = -ENOMEM;
		(*dma_out) = NULL;
		goto out;
	}

	dma->base = dma_base;
	dma->transfer_data = dsp_dma_transfer_data;
	dma->transfer_code = dsp_dma_transfer_code;

	(*dma_out) = dma;
	dsp_debug(DEBUG_DEVICE,
		  "dsp dma is created success, base=0x%08x\n", (u32)dma_base);
out:
	dsp_debug_leave();
	return ret;
}

int dsp_dma_destroy(struct dsp_dma *dma)
{
	kfree(dma);
	return 0;
}
