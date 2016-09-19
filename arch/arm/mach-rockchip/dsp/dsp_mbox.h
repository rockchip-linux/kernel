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
#ifndef _ARCH_ARM_MACH_RK_DSP_MBOX_H_
#define _ARCH_ARM_MACH_RK_DSP_MBOX_H_
#include <linux/platform_device.h>

#define MBOX_MAX_CHAN 4

#define DSP_CMD_ARM2DSP    0x01
#define DSP_CMD_DSP2ARM    0x02

#define DSP_CMD_DEFINE(chan, dir, id)  \
		((chan & 0xff) << 16 | (dir & 0xff) << 8 | (id & 0xff))

#define DSP_CMD_WORK        \
		DSP_CMD_DEFINE(MBOX_CHAN_0, DSP_CMD_ARM2DSP, 0x01)
#define DSP_CMD_WORK_DONE   \
		DSP_CMD_DEFINE(MBOX_CHAN_0, DSP_CMD_DSP2ARM, 0x01)

#define DSP_CMD_READY       \
		DSP_CMD_DEFINE(MBOX_CHAN_1, DSP_CMD_DSP2ARM, 0x01)
#define DSP_CMD_CONFIG      \
		DSP_CMD_DEFINE(MBOX_CHAN_1, DSP_CMD_ARM2DSP, 0x02)
#define DSP_CMD_CONFIG_DONE \
		DSP_CMD_DEFINE(MBOX_CHAN_1, DSP_CMD_DSP2ARM, 0x02)

#define DSP_CMD_TRACE       \
		DSP_CMD_DEFINE(MBOX_CHAN_3, DSP_CMD_DSP2ARM, 0x01)
#define DSP_CMD_TRACE_DONE  \
		DSP_CMD_DEFINE(MBOX_CHAN_3, DSP_CMD_ARM2DSP, 0x01)

enum mbox_chan_id {
	MBOX_CHAN_0 = 0,
	MBOX_CHAN_1 = 1,
	MBOX_CHAN_2 = 2,
	MBOX_CHAN_3 = 3
};

struct dsp_mbox_client {
	void *data;

	int (*receive_data)(struct dsp_mbox_client *, u32, u32, u32);

};

struct dsp_mbox_chan {
	u32 fresh;
	u32 cmd;
	u32 data;
};

struct dsp_mbox {
	void __iomem *base;
	int irq;

	struct dsp_mbox_chan chans[MBOX_MAX_CHAN];
	struct dsp_mbox_client *client;

	int (*send_data)(struct dsp_mbox *, u32, u32, u32);
};

int dsp_mbox_register_client(struct dsp_mbox *mbox,
			     struct dsp_mbox_client *client);

int dsp_mbox_create(struct platform_device *pdev, void __iomem *mbox_base,
		    struct dsp_mbox **mbox_out);

int dsp_mbox_destroy(struct dsp_mbox *mbox);

#endif
