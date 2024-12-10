// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015, Fuzhou Rockchip Electronics Co., Ltd
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <soc/rockchip/rockchip-mailbox.h>

#define MAILBOX_A2B_INTEN		0x00
#define MAILBOX_A2B_STATUS		0x04
#define MAILBOX_A2B_CMD(x)		(0x08 + (x) * 8)
#define MAILBOX_A2B_DAT(x)		(0x0c + (x) * 8)

#define MAILBOX_B2A_INTEN		0x28
#define MAILBOX_B2A_STATUS		0x2C
#define MAILBOX_B2A_CMD(x)		(0x30 + (x) * 8)
#define MAILBOX_B2A_DAT(x)		(0x34 + (x) * 8)

#define MAILBOX_POLLING_MS		5 /* default polling interval 5ms */

struct rockchip_mbox_data {
	int num_chans;
};

struct rockchip_mbox_chan {
	int idx;
	int irq;
};

struct rockchip_mbox {
	struct mbox_controller mbox;
	struct clk *pclk;
	void __iomem *mbox_base;
	spinlock_t cfg_lock; /* Serialise access to the register */
	struct rockchip_mbox_msg *msg;

	struct rockchip_mbox_chan *chans;
};

static int rockchip_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct rockchip_mbox *mb = dev_get_drvdata(chan->mbox->dev);
	struct rockchip_mbox_msg *msg = data;
	struct rockchip_mbox_chan *chans = chan->con_priv;
	u32 status;

	if (!msg)
		return -EINVAL;

	status = readl_relaxed(mb->mbox_base + MAILBOX_A2B_STATUS);
	if (status & (1U << chans->idx)) {
		dev_err(mb->mbox.dev, "The mailbox channel is busy\n");
		return -EBUSY;
	}

	dev_dbg(mb->mbox.dev, "Chan[%d]: A2B message, cmd 0x%08x, data 0x%08x\n",
		chans->idx, msg->cmd, msg->data);

	writel_relaxed(msg->cmd, mb->mbox_base + MAILBOX_A2B_CMD(chans->idx));
	writel_relaxed(msg->data, mb->mbox_base +
		       MAILBOX_A2B_DAT(chans->idx));

	return 0;
}

static int rockchip_mbox_startup(struct mbox_chan *chan)
{
	struct rockchip_mbox *mb = dev_get_drvdata(chan->mbox->dev);
	struct rockchip_mbox_chan *chans = chan->con_priv;
	u32 val = 0U;

	/* Enable the corresponding B2A interrupt */
	spin_lock(&mb->cfg_lock);
	val = readl_relaxed(mb->mbox_base + MAILBOX_B2A_INTEN) |
		(1U << chans->idx);
	writel_relaxed(val, mb->mbox_base + MAILBOX_B2A_INTEN);
	spin_unlock(&mb->cfg_lock);

	return 0;
}

static void rockchip_mbox_shutdown(struct mbox_chan *chan)
{
	struct rockchip_mbox *mb = dev_get_drvdata(chan->mbox->dev);
	struct rockchip_mbox_chan *chans = chan->con_priv;
	u32 val = 0U;

	/* Disable the corresponding B2A interrupt */
	spin_lock(&mb->cfg_lock);
	val = readl_relaxed(mb->mbox_base + MAILBOX_B2A_INTEN) &
		~(1U << chans->idx);
	writel_relaxed(val, mb->mbox_base + MAILBOX_B2A_INTEN);
	spin_unlock(&mb->cfg_lock);
}

static bool rockchip_mbox_last_tx_done(struct mbox_chan *chan)
{
	struct rockchip_mbox *mb = dev_get_drvdata(chan->mbox->dev);
	struct rockchip_mbox_chan *chans = chan->con_priv;
	u32 status;

	status = readl_relaxed(mb->mbox_base + MAILBOX_A2B_STATUS);
	return !(status & (1U << chans->idx));
}

static const struct mbox_chan_ops rockchip_mbox_chan_ops = {
	.send_data	= rockchip_mbox_send_data,
	.startup	= rockchip_mbox_startup,
	.shutdown	= rockchip_mbox_shutdown,
	.last_tx_done	= rockchip_mbox_last_tx_done,
};

int rockchip_mbox_read_msg(struct mbox_chan *chan,
			   struct rockchip_mbox_msg *msg)
{
	struct rockchip_mbox *mb;
	struct rockchip_mbox_chan *chans;

	if (!chan || !msg)
		return -EINVAL;

	mb = dev_get_drvdata(chan->mbox->dev);
	chans = chan->con_priv;

	msg->cmd  = mb->msg[chans->idx].cmd;
	msg->data = mb->msg[chans->idx].data;

	return 0;
}
EXPORT_SYMBOL_GPL(rockchip_mbox_read_msg);

static irqreturn_t rockchip_mbox_irq(int irq, void *dev_id)
{
	int idx;
	struct rockchip_mbox_msg *msg;
	struct rockchip_mbox *mb = (struct rockchip_mbox *)dev_id;
	u32 status = readl_relaxed(mb->mbox_base + MAILBOX_B2A_STATUS);

	for (idx = 0; idx < mb->mbox.num_chans; idx++) {
		if ((status & (1U << idx)) && irq == mb->chans[idx].irq) {
			/* Get cmd/data from the channel of B2A */
			msg = &mb->msg[idx];
			msg->cmd = readl_relaxed(mb->mbox_base +
						 MAILBOX_B2A_CMD(idx));
			msg->data = readl_relaxed(mb->mbox_base +
						  MAILBOX_B2A_DAT(idx));

			dev_dbg(mb->mbox.dev, "Chan[%d]: B2A message, cmd 0x%08x, data 0x%08x\n",
				idx, msg->cmd, msg->data);

			if (mb->mbox.chans[idx].cl)
				mbox_chan_received_data(&mb->mbox.chans[idx], msg);

			/* Clear mbox interrupt */
			writel_relaxed(1U << idx,
				       mb->mbox_base + MAILBOX_B2A_STATUS);
		}
	}

	return IRQ_HANDLED;
}

static const struct rockchip_mbox_data rk3368_drv_data = {
	.num_chans = 4,
};

static const struct of_device_id rockchip_mbox_of_match[] = {
	{ .compatible = "rockchip,rk3368-mailbox", .data = &rk3368_drv_data},
	{ },
};
MODULE_DEVICE_TABLE(of, rockchip_mbox_of_match);

static int rockchip_mbox_probe(struct platform_device *pdev)
{
	struct rockchip_mbox *mb;
	const struct of_device_id *match;
	const struct rockchip_mbox_data *drv_data;
	struct resource *res;
	int ret, irq, i;
	u32 txpoll_period;

	if (!pdev->dev.of_node)
		return -ENODEV;

	match = of_match_node(rockchip_mbox_of_match, pdev->dev.of_node);
	drv_data = (const struct rockchip_mbox_data *)match->data;

	mb = devm_kzalloc(&pdev->dev, sizeof(*mb), GFP_KERNEL);
	if (!mb)
		return -ENOMEM;

	mb->msg = devm_kcalloc(&pdev->dev, drv_data->num_chans,
			       sizeof(*mb->msg), GFP_KERNEL);
	if (!mb->msg)
		return -ENOMEM;

	mb->chans = devm_kcalloc(&pdev->dev, drv_data->num_chans,
				 sizeof(*mb->chans), GFP_KERNEL);
	if (!mb->chans)
		return -ENOMEM;

	mb->mbox.chans = devm_kcalloc(&pdev->dev, drv_data->num_chans,
				      sizeof(*mb->mbox.chans), GFP_KERNEL);
	if (!mb->mbox.chans)
		return -ENOMEM;

	platform_set_drvdata(pdev, mb);

	mb->mbox.dev = &pdev->dev;
	mb->mbox.num_chans = drv_data->num_chans;
	mb->mbox.ops = &rockchip_mbox_chan_ops;
	spin_lock_init(&mb->cfg_lock);

	mb->mbox.txdone_poll = true;
	ret = device_property_read_u32(&pdev->dev, "rockchip,txpoll-period-ms", &txpoll_period);
	mb->mbox.txpoll_period = !ret ? txpoll_period : MAILBOX_POLLING_MS;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	mb->mbox_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mb->mbox_base))
		return PTR_ERR(mb->mbox_base);

	mb->pclk = devm_clk_get(&pdev->dev, "pclk_mailbox");
	if (IS_ERR(mb->pclk)) {
		ret = PTR_ERR(mb->pclk);
		dev_err(&pdev->dev, "failed to get pclk_mailbox clock: %d\n",
			ret);
		return ret;
	}

	ret = clk_prepare_enable(mb->pclk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable pclk: %d\n", ret);
		return ret;
	}

	for (i = 0; i < mb->mbox.num_chans; i++) {
		irq = platform_get_irq(pdev, i);
		if (irq < 0) {
			/* For shared irq case, only could be got one time */
			if (i > 0 && irq == -ENXIO) {
				mb->chans[i].irq = mb->chans[0].irq;
			} else {
				ret = irq;
				goto disable_clk;
			}
		} else {
			mb->chans[i].irq = irq;
		}

		mb->chans[i].idx = i;
		mb->mbox.chans[i].con_priv = &mb->chans[i];
	}

	ret = devm_mbox_controller_register(&pdev->dev, &mb->mbox);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register mailbox: %d\n", ret);
		goto disable_clk;
	}

	for (i = 0; i < mb->mbox.num_chans; i++) {
		/* For shared irq case, only request irq thread one time */
		if (i > 0 && mb->chans[i].irq == mb->chans[0].irq)
			break;

		ret = devm_request_threaded_irq(&pdev->dev, mb->chans[i].irq,
						NULL,
						rockchip_mbox_irq,
						IRQF_ONESHOT,
						dev_name(&pdev->dev),
						mb);
		if (ret < 0)
			goto disable_clk;

		if (device_property_present(&pdev->dev, "wakeup-source"))
			enable_irq_wake(mb->chans[i].irq);
	}

	return 0;

disable_clk:
	clk_disable_unprepare(mb->pclk);
	return ret;
}

static struct platform_driver rockchip_mbox_driver = {
	.probe	= rockchip_mbox_probe,
	.driver = {
		.name = "rockchip-mailbox",
		.of_match_table = of_match_ptr(rockchip_mbox_of_match),
	},
};

#if defined(CONFIG_ROCKCHIP_THUNDER_BOOT)
static int __init rockchip_mbox_driver_init(void)
{
	return platform_driver_register(&rockchip_mbox_driver);
}
core_initcall(rockchip_mbox_driver_init);
#else
module_platform_driver(rockchip_mbox_driver);
#endif

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Rockchip mailbox: communicate between CPU cores and MCU");
MODULE_AUTHOR("Addy Ke <addy.ke@rock-chips.com>");
MODULE_AUTHOR("Caesar Wang <wxt@rock-chips.com>");
