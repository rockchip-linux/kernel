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
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include "dsp_dbg.h"
#include "dsp_mbox.h"

#define MAILBOX_A2B_INTEN               0x00
#define MAILBOX_A2B_STATUS              0x04
#define MAILBOX_A2B_CMD(x)              (0x08 + (x) * 8)
#define MAILBOX_A2B_DAT(x)              (0x0c + (x) * 8)

#define MAILBOX_B2A_INTEN               0x28
#define MAILBOX_B2A_STATUS              0x2C
#define MAILBOX_B2A_CMD(x)              (0x30 + (x) * 8)
#define MAILBOX_B2A_DAT(x)              (0x34 + (x) * 8)

#define MAILBOX_ATOMIC_LOCK(x)          (0x100 + (x) * 8)

/* A2B: 0 - 2k */
#define A2B_BUF(size, idx)              ((idx) * (size))
/* B2A: 2k - 4k */
#define B2A_BUF(size, idx)              (((idx) + 4) * (size))

static irqreturn_t dsp_mbox_irq(int irq, void *dev_id)
{
	int idx;
	u32 status;
	struct dsp_mbox *mbox = dev_id;

	status = readl_relaxed(mbox->base + MAILBOX_B2A_STATUS);
	for (idx = 0; idx < MBOX_MAX_CHAN; idx++) {
		if (status & (1 << idx)) {
			/* Read cmd and data */
			mbox->chans[idx].fresh = true;
			mbox->chans[idx].cmd = readl_relaxed(mbox->base +
						MAILBOX_B2A_CMD(idx));
			mbox->chans[idx].data = readl_relaxed(mbox->base +
						MAILBOX_B2A_DAT(idx));

			/* Clear mbox interrupt */
			writel_relaxed(1 << idx, mbox->base
					+ MAILBOX_B2A_STATUS);
		}
	}

	if (status)
		return IRQ_WAKE_THREAD;
	else
		return IRQ_NONE;
}

static irqreturn_t dsp_mbox_isr(int irq, void *dev_id)
{
	int idx;
	struct dsp_mbox *mbox = dev_id;

	for (idx = 0; idx < MBOX_MAX_CHAN; idx++) {
		struct dsp_mbox_chan *chan = &mbox->chans[idx];

		if (chan->fresh) {
			mbox->client->receive_data(mbox->client,
					idx, chan->cmd, chan->data);
			chan->fresh = false;
		}
	}

	return IRQ_HANDLED;
}

int dsp_mbox_send_data(struct dsp_mbox *mbox, u32 chan, u32 cmd, u32 data)
{
	int ret = 0;

	dsp_debug_enter();

	dsp_debug(DEBUG_DEVICE, "chan=%d, cmd=0x%08x, data=0x%08x\n",
		  chan, cmd, data);

	/* Send data to DSP */
	writel_relaxed(cmd, mbox->base + MAILBOX_A2B_CMD(chan));
	writel_relaxed(data, mbox->base + MAILBOX_A2B_DAT(chan));

	dsp_debug_leave();
	return ret;
}

int dsp_mbox_register_client(struct dsp_mbox *mbox,
			     struct dsp_mbox_client *client)
{
	mbox->client = client;
	return 0;
}

int dsp_mbox_create(struct platform_device *pdev, void __iomem *mbox_base,
		    struct dsp_mbox **mbox_out)
{
	int ret = 0;
	struct dsp_mbox *mbox;

	mbox = kzalloc(sizeof(*mbox), GFP_KERNEL);
	if (!mbox) {
		dsp_err("cannot alloc mem for dsp mbox\n");
		ret = -ENOMEM;
		goto out;
	}

	mbox->base = mbox_base;
	mbox->send_data = dsp_mbox_send_data;

	mbox->irq = platform_get_irq(pdev, 0);
	if (mbox->irq < 0) {
		dsp_err("fail to get mbox irq\n");
		ret = -ENXIO;
		goto out;
	}

	ret = devm_request_threaded_irq(&pdev->dev, mbox->irq, dsp_mbox_irq,
					dsp_mbox_isr, IRQF_ONESHOT,
					dev_name(&pdev->dev), mbox);
	if (ret) {
		dsp_err("request mbox irq failed\n");
		goto out;
	}

	/* Enable all B2A interrupts */
	writel_relaxed(0xf, mbox->base + MAILBOX_B2A_INTEN);

	(*mbox_out) = mbox;
out:
	if (ret)
		(*mbox_out) = NULL;
	return ret;
}

int dsp_mbox_destroy(struct dsp_mbox *mbox)
{
	int ret = 0;

	kfree(mbox);
	return ret;
}
