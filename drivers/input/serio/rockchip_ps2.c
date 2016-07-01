/*
 * rockchip_ps2.c   --  PS/2 controller driver
 * refers to drivers/input/serio/xilinx_ps2.c
 *
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/* #define DEBUG */

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/serio.h>
#include <linux/slab.h>

#define DRIVER_NAME		"rockchip_ps2"

struct rkps2data {
	int irq;
	unsigned long clk_rate;
	/* virt. address of control registers */
	void __iomem *base;
	unsigned int flags;
	struct clk *pclk;
	struct clk *clk;
	struct serio *serio;
	struct device *dev;
};

/* PS2C control registers */
#define PS2C_CTRL		(0x00)

/* PS2C receiving buffer register*/
#define PS2C_RBR		(0x04)

/* PS2C sending buffer register */
#define PS2C_TBR		(0x08)

/* PS2C status register */
#define PS2C_STAT		(0x0c)

/* PS2C interrupt enable register */
#define PS2C_IER		(0x10)

/* PS2C interrupt clear register */
#define PS2C_ICR		(0x14)

/* PS2C interrupt status register */
#define PS2C_ISR		(0x18)

/* PS2C time require register1 */
#define PS2C_TRR1		(0x1c)

/* PS2C time require register2 */
#define PS2C_TRR2		(0x20)

/* PS2C time require register3 */
#define PS2C_TRR3		(0x24)

/* PS2C receiving timeout require register */
#define PS2C_RTR		(0x28)

/* PS2C sending timeout require register */
#define PS2C_WTR		(0x2c)

/* PS2C filter width selection register */
#define PS2C_FLT		(0x30)

/* ps2c sending timeout enable */
#define TX_TIMEOUT_EN		BIT(4)

/* ps2c receiving timeout enable */
#define RX_TIMEOUT_EN		BIT(3)

/* PS2C mode */
#define PS2_MODE_MASK		(BIT(1) | BIT(2))

#define PS2_MODE_SHIFT		1

/* ps2c enable */
#define PS2C_CTRL_EN		BIT(0)

/* enable ps2c sending timeout interrupt */
#define TX_TIMEOUT_INT_EN	BIT(3)

/* enable ps2c finish sending one byte interrupt */
#define FSH_TX_INT_EN		BIT(2)

/* enable ps2c receiving timeout interrupt */
#define RX_TIMEOUT_INT_EN	BIT(1)

/* enable ps2c finish receiving one byte interrupt */
#define FSH_RX_INT_EN		BIT(0)

/* clear ps2c sending timeout interrupt */
#define TX_TIMEOUT_INT_CLR	BIT(3)

/* clear ps2c finish sending one byte interrupt */
#define FSH_TX_INT_CLR		BIT(2)

/* clear ps2c receiving timeout interrupt */
#define RX_TIMEOUT_INT_CLR	BIT(1)

/* clear ps2c finish receiving one byte interrupt */
#define FSH_RX_INT_CLR		BIT(0)

/* ps2c sending timeout interrupt status */
#define TX_TIMEOUT_INT_STATUS	BIT(3)

/* ps2c finish sending one byte interrupt status */
#define FSH_TX_INT_STATUS	BIT(2)

/* ps2c receiving timeout interrupt status */
#define RX_TIMEOUT_INT_STATUS	BIT(1)

/* ps2c finish receiving one byte interrupt status */
#define FSH_RX_INT_STATUS	BIT(0)

/* ps2c sending busy */
#define PS2C_STAT_TX_BUSY	BIT(7)

/* ps2c sending timeout */
#define PS2C_STAT_TX_TIMEOUT	BIT(6)

/* ps2c sending error */
#define PS2C_STAT_TX_ERROR	BIT(5)

/* ps2c sending buffer empty */
#define PS2C_STAT_TX_EMPTY	BIT(4)

/* ps2c receiving busy */
#define PS2C_STAT_RX_BUSY	BIT(3)

/* ps2c receiving timeout */
#define PS2C_STAT_RX_TIMEOUT	BIT(2)

/* ps2c receiving error */
#define PS2C_STAT_RX_ERROR	BIT(1)

/* ps2c receiving buffer full */
#define PS2C_STAT_RX_FULL	BIT(0)

enum {
	RX_MODE = 0,
	TX_MODE,
	INHIBITON_MODE,
	RESERVED
};

#define PS2_RX_MODE		(RX_MODE << PS2_MODE_SHIFT)
#define PS2_TX_MODE		(TX_MODE << PS2_MODE_SHIFT)
#define PS2_INHIBITON_MODE	(INHIBITON_MODE << PS2_MODE_SHIFT)

/**
 * rkps2_recv() - attempts to receive a byte from the PS/2 port.
 * @drvdata:	pointer to ps2 device private data structure
 * @byte:	address where the read data will be copied
 *
 * If there is any data available in the PS/2 receiver, this functions reads
 * the data, otherwise it returns error.
 */
static int rkps2_recv(struct rkps2data *drvdata, u8 *byte)
{
	u32 sr;
	u32 tmp;
	int status = -1;

	/* If there is data available in the PS/2 receiver, read it */
	sr = readl(drvdata->base + PS2C_STAT);
	if (sr & PS2C_STAT_RX_ERROR) {
		dev_warn(drvdata->dev, "rx error\n");
		drvdata->flags |= SERIO_PARITY;
		writel((PS2C_STAT_RX_ERROR << 16) | (~PS2C_STAT_RX_ERROR),
		       drvdata->base + PS2C_STAT);
		/* must be read and clear full status */
		if (sr & PS2C_STAT_RX_FULL) {
			dev_warn(drvdata->dev, "rx err but must be read\n");
			tmp = readl(drvdata->base + PS2C_RBR);
			*byte = (tmp & 0xff);
		}
		return status;
	}

	if (sr & PS2C_STAT_RX_TIMEOUT) {
		dev_warn(drvdata->dev, "rx timeout\n");
		writel((PS2C_STAT_RX_TIMEOUT << 16) | (~PS2C_STAT_RX_TIMEOUT),
		       drvdata->base + PS2C_STAT);
		drvdata->flags |= SERIO_TIMEOUT;
		return status;
	}

	if (sr & PS2C_STAT_RX_FULL) {
		tmp = readl(drvdata->base + PS2C_RBR);
		*byte = (tmp & 0xff);
		status = 0;
	}
	return status;
}

static irqreturn_t rockchip_ps2_interrupt(int irq, void *dev_id)
{
	struct rkps2data *drvdata = dev_id;
	u32 intr_sr;
	u8 c;
	int status;

	/* get  interrupt status  */
	intr_sr = readl(drvdata->base + PS2C_ISR);
	/* Check which interrupt is active */
	if (intr_sr & TX_TIMEOUT_INT_STATUS)
		dev_warn(drvdata->dev, "sending timeout\n");

	if (intr_sr & RX_TIMEOUT_INT_STATUS)
		drvdata->flags |= SERIO_TIMEOUT;

	if (intr_sr & FSH_RX_INT_STATUS) {
		status = rkps2_recv(drvdata, &c);

		/* Error, if a byte is not received */
		if (status) {
			dev_err(drvdata->dev,
				"received error (%d)\n", status);
		} else {
			serio_interrupt(drvdata->serio, c, drvdata->flags);
			drvdata->flags = 0;
		}
	}

	/* if finish sending  we enable rx mode */
	if (intr_sr & FSH_TX_INT_STATUS)
		writel((PS2_MODE_MASK << 16 | PS2_RX_MODE),
		       drvdata->base + PS2C_CTRL);

	/* clear interrupts status */
	writel((intr_sr << 16) | intr_sr, drvdata->base + PS2C_ICR);
	return IRQ_HANDLED;
}

/**
 * rockchip_ps2_write() - sends a byte out through the PS/2 port.
 * @pserio:	pointer to the serio structure of the PS/2 port
 * @c:		data that needs to be written to the PS/2 port
 *
 * This function checks if the PS/2 transmitter is empty and sends a byte.
 * Otherwise it returns error. Transmission fails only when nothing is
 * connected to the PS/2 port. Thats why, we do not try to resend the data
 * in case of a failure.
 */
static int rockchip_ps2_write(struct serio *pserio, unsigned char c)
{
	struct rkps2data *drvdata = pserio->port_data;
	u32 sr;
	u32 tmp = 0;
	int status = -1;

	writel((PS2_MODE_MASK << 16) | PS2_TX_MODE, drvdata->base + PS2C_CTRL);

	/* ps2c enable */
	writel((PS2C_CTRL_EN << 16) | PS2C_CTRL_EN, drvdata->base + PS2C_CTRL);

	/* If the PS/2 transmitter is empty send a byte of data */
	sr = readl(drvdata->base + PS2C_STAT);
	if (sr & PS2C_STAT_TX_EMPTY) {
		tmp = tmp | c;
		writel(tmp, drvdata->base + PS2C_TBR);
		status = 0;
	}
	return status;
}

/**
 * rockchip_ps2_open() - called when a port is opened by the higher layer.
 * @pserio:	pointer to the serio structure of the PS/2 device
 *
 * This function enables interrupts for the PS/2 device.
 */
static int rockchip_ps2_open(struct serio *pserio)
{
	struct rkps2data *drvdata = pserio->port_data;
	unsigned long clk_rate = drvdata->clk_rate;
	unsigned long time_req = 0;

	/* ps2c disable, end ps2c inhibition mode */
	writel((PS2C_CTRL_EN << 16) | (~PS2C_CTRL_EN),
	       drvdata->base + PS2C_CTRL);

	/* PS/2 receiving mode  */
	writel((PS2_MODE_MASK << 16 | PS2_RX_MODE), drvdata->base + PS2C_CTRL);

	/*
	 * 1. according to TRM datasheet PS2C cnt_clk = clk_rate
	 * the register PS2C_RTR time unit is one cnt_clk cycle
	 * so 1us = clk_rate / 1000000,
	 * and 1000ms = 1000000 * clk_rate / 1000000 = clk_rate
	 * ps2c receiving timeout require time 1000ms
	 */

	time_req = clk_rate;
	writel(time_req, drvdata->base + PS2C_RTR);

	/*
	 * ps2c receiving timeout enable. Only when this bit is set, there
	 * PS2C_STAT[2](ps2c_rv_timeouot) is access
	 */
	writel((RX_TIMEOUT_EN << 16) | RX_TIMEOUT_EN,
	       drvdata->base + PS2C_CTRL);
	/* ps2c enable */
	writel((PS2C_CTRL_EN << 16) | PS2C_CTRL_EN, drvdata->base + PS2C_CTRL);

	/* enable ps2c receiving timeout interrupt */
	writel((RX_TIMEOUT_INT_EN << 16) | RX_TIMEOUT_INT_EN,
	       drvdata->base + PS2C_IER);

	/* enable ps2c finish receiving one byte interrupt  */
	writel((FSH_RX_INT_EN << 16) | FSH_RX_INT_EN,
	       drvdata->base + PS2C_IER);

	/*
	 * we set ps2c sending time here.
	 * 1. according to TRM datasheet PS2C cnt_clk = clk_rate
	 * 2. And these registers PS2C_TRR1,PS2C_TRR2,PS2C_TRR3
	 * 3. Their time unit is one cnt_clk cycle
	 * 1us = clk_rate / 1000000, so we get
	 * 100us = 100 * clk_rate / 1000000 = clk_rate / 10000
	 * 5us = 5 * clk_rate / 1000000 = clk_rate / 200000
	 * 1000ms = 1000000 * clk_rate / 1000000 = clk_rate
	 */

	/* 100 us */
	time_req = clk_rate / 10000;
	writel(time_req, drvdata->base + PS2C_TRR1);

	/* 5us */
	time_req = clk_rate / 200000;
	writel(time_req, drvdata->base + PS2C_TRR2);

	/* 5us */
	writel(time_req, drvdata->base + PS2C_TRR3);

	/* 1000ms timeout */
	time_req = clk_rate;
	writel(time_req, drvdata->base + PS2C_WTR);

	writel((TX_TIMEOUT_EN << 16) | TX_TIMEOUT_EN,
	       drvdata->base + PS2C_CTRL);

	/* enable ps2c sending timeout interrupt */
	writel((TX_TIMEOUT_INT_EN << 16) | TX_TIMEOUT_INT_EN,
	       drvdata->base + PS2C_IER);

	/* enable ps2c finish sending one byte interrupt  */
	writel((FSH_TX_INT_EN << 16) | FSH_TX_INT_EN,
	       drvdata->base + PS2C_IER);

	return 0;
}

/**
 * rockchip_ps2_close() - frees the interrupt.
 * @pserio:	pointer to the serio structure of the PS/2 device
 *
 * This function disables all interrupts for the PS/2 device.
 */
static void rockchip_ps2_close(struct serio *pserio)
{
	struct rkps2data *drvdata = pserio->port_data;

	/* Disable ps2c receiving timeout interrupt */
	writel((RX_TIMEOUT_INT_EN << 16) | (~RX_TIMEOUT_INT_EN),
	       drvdata->base + PS2C_IER);

	/* Disable ps2c finish receiving one byte interrupt  */
	writel((FSH_RX_INT_EN << 16) | (~FSH_RX_INT_EN),
	       drvdata->base + PS2C_IER);

	/* Disable ps2c sending timeout interrupt */
	writel((TX_TIMEOUT_INT_EN << 16) | (~TX_TIMEOUT_INT_EN),
	       drvdata->base + PS2C_IER);

	/* Disable ps2c finish sending one byte interrupt  */
	writel((FSH_TX_INT_EN << 16) | (~FSH_TX_INT_EN),
	       drvdata->base + PS2C_IER);
}

/**
 * rockchip_ps2_probe - probe method for the PS/2 device.
 * @of_dev:	pointer to OF device structure
 *
 * This function probes the PS/2 device in the device tree.
 * It initializes the driver data structure and the hardware.
 * It returns 0, if the driver is bound to the PS/2 device, or a negative
 * value if there is an error.
 */
static int rockchip_ps2_probe(struct platform_device *ofdev)
{
	struct rkps2data *drvdata;
	struct serio *serio;
	struct device *dev = &ofdev->dev;
	struct resource *res;
	int error;

	dev_info(dev, "Device Tree Probing \'%s\'\n",
		 ofdev->dev.of_node->name);

	drvdata = devm_kzalloc(&ofdev->dev,
			       sizeof(struct rkps2data), GFP_KERNEL);
	serio = devm_kzalloc(&ofdev->dev,
			     sizeof(struct serio), GFP_KERNEL);
	if (!drvdata || !serio) {
		dev_err(dev, "no Memery\n");
		return -ENOMEM;
	}

	drvdata->irq = platform_get_irq(ofdev, 0);
	if (drvdata->irq < 0) {
		dev_err(dev, "no IRQ found\n");
		return -ENODEV;
	}
	drvdata->serio = serio;
	drvdata->dev = dev;

	res = platform_get_resource(ofdev, IORESOURCE_MEM, 0);
	drvdata->base = devm_ioremap_resource(&ofdev->dev, res);
	if (IS_ERR(drvdata->base))
		return PTR_ERR(drvdata->base);

	drvdata->pclk = devm_clk_get(&ofdev->dev, "pclk_ps2c");
	if (IS_ERR(drvdata->pclk)) {
		dev_err(&ofdev->dev, "Can't retrieve pclk_ps2c\n");
		drvdata->pclk = NULL;
		error = -EFAULT;
		return error;
	}
	clk_prepare_enable(drvdata->pclk);

	drvdata->clk = devm_clk_get(&ofdev->dev, "clk_ps2c");
	if (IS_ERR(drvdata->clk)) {
		dev_err(&ofdev->dev, "Can't retrieve clk_ps2c\n");
		drvdata->clk = NULL;
		error = -EFAULT;
		clk_disable_unprepare(drvdata->pclk);
		return error;
	}
	clk_prepare_enable(drvdata->clk);
	drvdata->clk_rate = clk_get_rate(drvdata->clk);

	dev_info(dev, "Rockchip PS2 at 0x%08llX mapped to 0x%p, irq=%d\n",
		 (unsigned long long)res->start,
			drvdata->base, drvdata->irq);

	serio->id.type = SERIO_8042;
	serio->write = rockchip_ps2_write;
	serio->open = rockchip_ps2_open;
	serio->close = rockchip_ps2_close;
	serio->port_data = drvdata;
	serio->dev.parent = dev;

	serio_register_port(serio);

	platform_set_drvdata(ofdev, drvdata);

	error = devm_request_irq(&ofdev->dev,
				 drvdata->irq, &rockchip_ps2_interrupt,
				 IRQF_TRIGGER_RISING, DRIVER_NAME, drvdata);
	if (error) {
		dev_err(drvdata->dev, "Couldn't allocate interrupt %d\n",
			drvdata->irq);
		goto allocate_irq_err;
	}

	/*ps2c works as inhibition mode, inhibits the ps2 device send data */
	writel(((PS2_MODE_MASK << 16) | (PS2_INHIBITON_MODE)),
	       drvdata->base + PS2C_CTRL);
	/* ps2c enable */
	writel((PS2C_CTRL_EN << 16) | PS2C_CTRL_EN, drvdata->base + PS2C_CTRL);

	dev_info(dev, "Rockchip PS2 probe success\n");
	return 0;

allocate_irq_err:

	clk_disable_unprepare(drvdata->clk);
	clk_disable_unprepare(drvdata->pclk);
	serio_unregister_port(drvdata->serio);
	return error;
}

/**
 * rockchip_ps2_remove - unbinds the driver from the PS/2 device.
 * @of_dev:	pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded.
 */
static int rockchip_ps2_remove(struct platform_device *of_dev)
{
	struct rkps2data *drvdata = platform_get_drvdata(of_dev);

	clk_disable_unprepare(drvdata->clk);
	clk_disable_unprepare(drvdata->pclk);
	serio_unregister_port(drvdata->serio);
	platform_set_drvdata(of_dev, NULL);
	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id rockchip_ps2_of_match[] = {
	{ .compatible = "rockchip,ps2c", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, rockchip_ps2_of_match);

static struct platform_driver rockchip_p2s_of_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = rockchip_ps2_of_match,
	},
	.probe		= rockchip_ps2_probe,
	.remove		= rockchip_ps2_remove,
};
module_platform_driver(rockchip_p2s_of_driver);

MODULE_AUTHOR("Luo XiaoTan <lxt@rock-chips.com>");
MODULE_DESCRIPTION("RockChip PS/2 Controller driver");
MODULE_LICENSE("GPL v2");
