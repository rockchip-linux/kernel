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
#ifndef _ARCH_ARM_MACH_RK_DSP_DEV_H_
#define _ARCH_ARM_MACH_RK_DSP_DEV_H_

#include <linux/platform_device.h>
#include <linux/dmapool.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include "dsp_loader.h"
#include "dsp_dma.h"
#include "dsp_mbox.h"
#include "dsp_work.h"

enum dsp_status {
	DSP_OFF      = 0,
	DSP_ON       = 1,
	DSP_SLEEP    = 2,
};

struct dsp_dev_client {
	void *data;

	int (*device_ready)(struct dsp_dev_client *);
	int (*work_done)(struct dsp_dev_client *, struct dsp_work *);
};

struct dsp_dev {
	enum dsp_status status;

	int (*on)(struct dsp_dev *);
	int (*off)(struct dsp_dev *);
	int (*suspend)(struct dsp_dev *);
	int (*resume)(struct dsp_dev *);
	int (*config)(struct dsp_dev *);
	int (*work)(struct dsp_dev *, struct dsp_work *);

	struct dsp_dma *dma;
	struct dsp_mbox *mbox;
	struct dsp_loader *loader;
	struct dma_pool *dma_pool;
	struct dsp_dev_client *client;
	struct dsp_mbox_client mbox_client;

	char *trace_buffer;
	u32 trace_dma;
	u32 trace_index;

	struct clk *clk_dsp;
	struct clk *clk_iop;
	struct clk *clk_epp;
	struct clk *clk_edp;
	struct clk *clk_edap;

	struct reset_control *core_rst;
	struct reset_control *sys_rst;
	struct reset_control *global_rst;
	struct reset_control *oecm_rst;

	void __iomem *dsp_grf;
	void __iomem *dsp_axi;
	void __iomem *mbox_base;

	/* Lock DSP device */
	struct mutex lock;
};

int dsp_dev_register_client(struct dsp_dev *dev,
			    struct dsp_dev_client *client);

int dsp_dev_create(struct platform_device *pdev, struct dma_pool *dma_pool,
		   struct dsp_dev **dev);

int dsp_dev_destroy(struct dsp_dev *dev);

#endif

