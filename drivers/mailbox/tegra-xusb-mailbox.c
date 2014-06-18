/*
 * NVIDIA Tegra XUSB mailbox driver
 *
 * Copyright (C) 2014 NVIDIA Corporation
 * Copyright (C) 2014 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <soc/tegra/xusb.h>

#define XUSB_CFG_ARU_MBOX_CMD			0xe4
#define  MBOX_DEST_FALC				BIT(27)
#define  MBOX_DEST_PME				BIT(28)
#define  MBOX_DEST_SMI				BIT(29)
#define  MBOX_DEST_XHCI				BIT(30)
#define  MBOX_INT_EN				BIT(31)
#define XUSB_CFG_ARU_MBOX_DATA_IN		0xe8
#define  CMD_DATA_SHIFT				0
#define  CMD_DATA_MASK				0xffffff
#define  CMD_TYPE_SHIFT				24
#define  CMD_TYPE_MASK				0xff
#define XUSB_CFG_ARU_MBOX_DATA_OUT		0xec
#define XUSB_CFG_ARU_MBOX_OWNER			0xf0
#define  MBOX_OWNER_NONE			0
#define  MBOX_OWNER_FW				1
#define  MBOX_OWNER_SW				2
#define XUSB_CFG_ARU_SMI_INTR			0x428
#define  MBOX_SMI_INTR_FW_HANG			BIT(1)
#define  MBOX_SMI_INTR_EN			BIT(3)

struct tegra_xusb_mbox {
	struct mbox_controller mbox;
	int irq;
	void __iomem *regs;
	spinlock_t lock;
	bool vchan_allocated[TEGRA_XUSB_MBOX_NUM_CHANS];
};

static inline u32 mbox_readl(struct tegra_xusb_mbox *mbox, unsigned long offset)
{
	return readl(mbox->regs + offset);
}

static inline void mbox_writel(struct tegra_xusb_mbox *mbox, u32 val,
			       unsigned long offset)
{
	writel(val, mbox->regs + offset);
}

static inline u32 mbox_pack_msg(struct tegra_xusb_mbox_msg *msg)
{
	u32 val;

	val = (msg->cmd & CMD_TYPE_MASK) << CMD_TYPE_SHIFT;
	val |= (msg->data & CMD_DATA_MASK) << CMD_DATA_SHIFT;

	return val;
}

static inline void mbox_unpack_msg(u32 val, struct tegra_xusb_mbox_msg *msg)
{
	msg->cmd = (val >> CMD_TYPE_SHIFT) & CMD_TYPE_MASK;
	msg->data = (val >> CMD_DATA_SHIFT) & CMD_DATA_MASK;
}

static int tegra_xusb_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct tegra_xusb_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	struct tegra_xusb_mbox_msg *msg = data;
	unsigned long flags;
	u32 reg, owner;

	dev_dbg(mbox->mbox.dev, "TX message 0x%x:0x%x\n", msg->cmd, msg->data);

	/* ACK/NAK must be sent with the controller as the mailbox owner */
	if (msg->cmd == MBOX_CMD_ACK || msg->cmd == MBOX_CMD_NAK)
		owner = MBOX_OWNER_FW;
	else
		owner = MBOX_OWNER_SW;

	spin_lock_irqsave(&mbox->lock, flags);

	/* Acquire mailbox */
	if (mbox_readl(mbox, XUSB_CFG_ARU_MBOX_OWNER) != MBOX_OWNER_NONE) {
		dev_err(mbox->mbox.dev, "Mailbox not idle\n");
		goto busy;
	}
	mbox_writel(mbox, owner, XUSB_CFG_ARU_MBOX_OWNER);
	if (mbox_readl(mbox, XUSB_CFG_ARU_MBOX_OWNER) != owner) {
		dev_err(mbox->mbox.dev, "Failed to acquire mailbox");
		goto busy;
	}

	mbox_writel(mbox, mbox_pack_msg(msg), XUSB_CFG_ARU_MBOX_DATA_IN);
	reg = mbox_readl(mbox, XUSB_CFG_ARU_MBOX_CMD);
	reg |= MBOX_INT_EN | MBOX_DEST_FALC;
	mbox_writel(mbox, reg, XUSB_CFG_ARU_MBOX_CMD);

	spin_unlock_irqrestore(&mbox->lock, flags);

	return 0;
busy:
	spin_unlock_irqrestore(&mbox->lock, flags);
	return -EBUSY;
}

static int tegra_xusb_mbox_startup(struct mbox_chan *chan)
{
	struct tegra_xusb_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	int idx = chan - mbox->mbox.chans;
	unsigned long flags;

	spin_lock_irqsave(&mbox->lock, flags);
	mbox->vchan_allocated[idx] = true;
	spin_unlock_irqrestore(&mbox->lock, flags);

	return 0;
}

static void tegra_xusb_mbox_shutdown(struct mbox_chan *chan)
{
	struct tegra_xusb_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	int idx = chan - mbox->mbox.chans;
	unsigned long flags;

	spin_lock_irqsave(&mbox->lock, flags);
	mbox->vchan_allocated[idx] = false;
	spin_unlock_irqrestore(&mbox->lock, flags);
}

static bool tegra_xusb_mbox_last_tx_done(struct mbox_chan *chan)
{
	/*
	 * Transmissions are assumed to be completed as soon as they are
	 * written to the mailbox.
	 */
	return true;
}

static struct mbox_chan_ops tegra_xusb_mbox_chan_ops = {
	.send_data = tegra_xusb_mbox_send_data,
	.startup = tegra_xusb_mbox_startup,
	.shutdown = tegra_xusb_mbox_shutdown,
	.last_tx_done = tegra_xusb_mbox_last_tx_done,
};

static irqreturn_t tegra_xusb_mbox_irq(int irq, void *p)
{
	struct tegra_xusb_mbox *mbox = (struct tegra_xusb_mbox *)p;
	struct tegra_xusb_mbox_msg msg;
	int i;
	u32 reg;

	spin_lock(&mbox->lock);

	/* Clear mbox interrupts */
	reg = mbox_readl(mbox, XUSB_CFG_ARU_SMI_INTR);
	if (reg & MBOX_SMI_INTR_FW_HANG)
		dev_err(mbox->mbox.dev, "Controller firmware hang\n");
	mbox_writel(mbox, reg, XUSB_CFG_ARU_SMI_INTR);

	reg = mbox_readl(mbox, XUSB_CFG_ARU_MBOX_DATA_OUT);
	mbox_unpack_msg(reg, &msg);

	/*
	 * Set the mailbox back to idle.  The recipient of the message is
	 * responsible for sending an ACK/NAK, if necessary.
	 */
	reg = mbox_readl(mbox, XUSB_CFG_ARU_MBOX_CMD);
	reg &= ~MBOX_DEST_SMI;
	mbox_writel(mbox, reg, XUSB_CFG_ARU_MBOX_CMD);
	mbox_writel(mbox, MBOX_OWNER_NONE, XUSB_CFG_ARU_MBOX_OWNER);

	dev_dbg(mbox->mbox.dev, "RX message 0x%x:0x%x\n", msg.cmd, msg.data);
	for (i = 0; i < ARRAY_SIZE(mbox->vchan_allocated); i++) {
		if (mbox->vchan_allocated[i])
			mbox_chan_received_data(&mbox->mbox.chans[i], &msg);
	}

	spin_unlock(&mbox->lock);

	return IRQ_HANDLED;
}

static struct mbox_chan *tegra_xusb_mbox_of_xlate(struct mbox_controller *ctlr,
					const struct of_phandle_args *sp)
{
	struct tegra_xusb_mbox *mbox = dev_get_drvdata(ctlr->dev);
	struct mbox_chan *chan = NULL;
	unsigned long flags;
	int i;

	/* Pick the first available (virtual) channel. */
	spin_lock_irqsave(&mbox->lock, flags);
	for (i = 0; i < ARRAY_SIZE(mbox->vchan_allocated); i++) {
		if (!mbox->vchan_allocated[i]) {
			chan = &ctlr->chans[i];
			break;
		}
	}
	spin_unlock_irqrestore(&mbox->lock, flags);

	return chan;
}

static struct of_device_id tegra_xusb_mbox_of_match[] = {
	{ .compatible = "nvidia,tegra124-xusb-mbox" },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_xusb_mbox_of_match);

static int tegra_xusb_mbox_probe(struct platform_device *pdev)
{
	struct tegra_xusb_mbox *mbox;
	struct resource *res;
	int ret;

	mbox = devm_kzalloc(&pdev->dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;
	platform_set_drvdata(pdev, mbox);
	spin_lock_init(&mbox->lock);

	mbox->mbox.dev = &pdev->dev;
	mbox->mbox.chans = devm_kcalloc(&pdev->dev, TEGRA_XUSB_MBOX_NUM_CHANS,
					sizeof(*mbox->mbox.chans), GFP_KERNEL);
	if (!mbox->mbox.chans)
		return -ENOMEM;
	mbox->mbox.num_chans = TEGRA_XUSB_MBOX_NUM_CHANS;
	mbox->mbox.ops = &tegra_xusb_mbox_chan_ops;
	mbox->mbox.txdone_poll = true;
	mbox->mbox.txpoll_period = 0; /* no need to actually poll */
	mbox->mbox.of_xlate = tegra_xusb_mbox_of_xlate;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;
	mbox->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!mbox->regs)
		return -ENOMEM;

	mbox->irq = platform_get_irq(pdev, 0);
	if (mbox->irq < 0)
		return mbox->irq;
	ret = devm_request_irq(&pdev->dev, mbox->irq, tegra_xusb_mbox_irq, 0,
			       dev_name(&pdev->dev), mbox);
	if (ret < 0)
		return ret;

	ret = mbox_controller_register(&mbox->mbox);
	if (ret < 0)
		dev_err(&pdev->dev, "failed to register mailbox: %d\n", ret);

	return ret;
}

static int tegra_xusb_mbox_remove(struct platform_device *pdev)
{
	struct tegra_xusb_mbox *mbox = platform_get_drvdata(pdev);

	mbox_controller_unregister(&mbox->mbox);

	return 0;
}

static struct platform_driver tegra_xusb_mbox_driver = {
	.probe	= tegra_xusb_mbox_probe,
	.remove	= tegra_xusb_mbox_remove,
	.driver	= {
		.name = "tegra-xusb-mbox",
		.of_match_table = of_match_ptr(tegra_xusb_mbox_of_match),
	},
};
module_platform_driver(tegra_xusb_mbox_driver);

MODULE_AUTHOR("Andrew Bresticker <abrestic@chromium.org>");
MODULE_DESCRIPTION("NVIDIA Tegra XUSB mailbox driver");
MODULE_LICENSE("GPL v2");
