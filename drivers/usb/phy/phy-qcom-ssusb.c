/* Copyright (c) 2013-2014, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/phy.h>

/**
 *  USB QSCRATCH Hardware registers
 */
#define PHY_CTRL_REG			(0x00)
#define PHY_PARAM_CTRL_1		(0x04)
#define PHY_PARAM_CTRL_2		(0x08)
#define CR_PROTOCOL_DATA_IN_REG		(0x0c)
#define CR_PROTOCOL_DATA_OUT_REG	(0x10)
#define CR_PROTOCOL_CAP_ADDR_REG	(0x14)
#define CR_PROTOCOL_CAP_DATA_REG	(0x18)
#define CR_PROTOCOL_READ_REG		(0x1c)
#define CR_PROTOCOL_WRITE_REG		(0x20)

#define PHY_1P8_VOL_MIN			1800000 /* uV */
#define PHY_1P8_VOL_MAX			1800000 /* uV */
#define PHY_1P8_HPM_LOAD		23000	/* uA */

/* TODO: these are suspicious */
#define USB_VDDCX_NO			1	/* index */
#define USB_VDDCX_MIN			5	/* index */
#define USB_VDDCX_MAX			7	/* index */

struct qcom_dwc3_ss_phy {
	struct usb_phy		phy;
	void __iomem		*base;
	struct device		*dev;

	struct regulator	*v1p8;
	struct regulator	*vddcx;

	struct clk		*xo_clk;
	struct clk		*ref_clk;
};

#define	phy_to_dw_phy(x)	container_of((x), struct qcom_dwc3_ss_phy, phy)


/**
 * Write register
 *
 * @base - QCOM DWC3 PHY base virtual address.
 * @offset - register offset.
 * @val - value to write.
 */
static inline void qcom_dwc3_ss_write(void __iomem *base, u32 offset, u32 val)
{
	writel(val, base + offset);
}

/**
 * Write register and read back masked value to confirm it is written
 *
 * @base - QCOM DWC3 PHY base virtual address.
 * @offset - register offset.
 * @mask - register bitmask specifying what should be updated
 * @val - value to write.
 */
static inline void qcom_dwc3_ss_write_readback(void __iomem *base, u32 offset,
					    const u32 mask, u32 val)
{
	u32 write_val, tmp = readl(base + offset);

	tmp &= ~mask;		/* retain other bits */
	write_val = tmp | val;

	writel(write_val, base + offset);

	/* Read back to see if val was written */
	tmp = readl(base + offset);
	tmp &= mask;		/* clear other bits */

	if (tmp != val)
		pr_err("write: %x to QSCRATCH: %x FAILED\n", val, offset);
}

/**
 * Write SSPHY register
 *
 * @base - QCOM DWC3 PHY base virtual address.
 * @addr - SSPHY address to write.
 * @val - value to write.
 */
static void qcom_dwc3_ss_write_phycreg(void __iomem *base, u32 addr, u32 val)
{
	writel(addr, base + CR_PROTOCOL_DATA_IN_REG);
	writel(0x1, base + CR_PROTOCOL_CAP_ADDR_REG);
	while (readl(base + CR_PROTOCOL_CAP_ADDR_REG))
		cpu_relax();

	writel(val, base + CR_PROTOCOL_DATA_IN_REG);
	writel(0x1, base + CR_PROTOCOL_CAP_DATA_REG);
	while (readl(base + CR_PROTOCOL_CAP_DATA_REG))
		cpu_relax();

	writel(0x1, base + CR_PROTOCOL_WRITE_REG);
	while (readl(base + CR_PROTOCOL_WRITE_REG))
		cpu_relax();
}

/**
 * Read SSPHY register.
 *
 * @base - QCOM DWC3 PHY base virtual address.
 * @addr - SSPHY address to read.
 */
static u32 qcom_dwc3_ss_read_phycreg(void __iomem *base, u32 addr)
{
	bool first_read = true;

	writel(addr, base + CR_PROTOCOL_DATA_IN_REG);
	writel(0x1, base + CR_PROTOCOL_CAP_ADDR_REG);
	while (readl(base + CR_PROTOCOL_CAP_ADDR_REG))
		cpu_relax();

	/*
	 * Due to hardware bug, first read of SSPHY register might be
	 * incorrect. Hence as workaround, SW should perform SSPHY register
	 * read twice, but use only second read and ignore first read.
	 */
retry:
	writel(0x1, base + CR_PROTOCOL_READ_REG);
	while (readl(base + CR_PROTOCOL_READ_REG))
		cpu_relax();

	if (first_read) {
		readl(base + CR_PROTOCOL_DATA_OUT_REG);
		first_read = false;
		goto retry;
	}

	return readl(base + CR_PROTOCOL_DATA_OUT_REG);
}

static void qcom_dwc3_ss_phy_shutdown(struct usb_phy *x)
{
	struct qcom_dwc3_ss_phy *phy = phy_to_dw_phy(x);
	int ret;

	/* Sequence to put SSPHY in low power state:
	 * 1. Clear REF_PHY_EN in PHY_CTRL_REG
	 * 2. Clear REF_USE_PAD in PHY_CTRL_REG
	 * 3. Set TEST_POWERED_DOWN in PHY_CTRL_REG to enable PHY retention
	 * 4. Disable SSPHY ref clk
	 */
	qcom_dwc3_ss_write_readback(phy->base, PHY_CTRL_REG, (1 << 8), 0x0);
	qcom_dwc3_ss_write_readback(phy->base, PHY_CTRL_REG, (1 << 28), 0x0);
	qcom_dwc3_ss_write_readback(phy->base, PHY_CTRL_REG,
				    (1 << 26), (1 << 26));

	usleep_range(1000, 1200);
	clk_disable_unprepare(phy->ref_clk);

	ret = regulator_set_voltage(phy->vddcx, USB_VDDCX_NO, USB_VDDCX_MAX);
	if (ret)
		dev_err(phy->dev, "cannot set voltage for vddcx\n");

	regulator_disable(phy->vddcx);

	ret = regulator_set_voltage(phy->v1p8, 0, PHY_1P8_VOL_MAX);
	if (ret)
		dev_err(phy->dev, "cannot set v1p8\n");

	regulator_disable(phy->v1p8);
}

static int qcom_dwc3_ss_phy_init(struct usb_phy *x)
{
	struct qcom_dwc3_ss_phy *phy = phy_to_dw_phy(x);
	u32 data = 0;
	int ret;

	ret = regulator_set_voltage(phy->vddcx, USB_VDDCX_MIN, USB_VDDCX_MAX);
	if (ret) {
		dev_err(phy->dev, "cannot set voltage for vddcx\n");
		return ret;
	}

	ret = regulator_enable(phy->vddcx);
	if (ret) {
		dev_err(phy->dev, "cannot enable vddcx\n");
		return ret;
	}

	ret = regulator_set_voltage(phy->v1p8, PHY_1P8_VOL_MIN,
				    PHY_1P8_VOL_MAX);
	if (ret) {
		regulator_disable(phy->vddcx);
		dev_err(phy->dev, "cannot set v1p8\n");
		return ret;
	}

	ret = regulator_enable(phy->v1p8);
	if (ret) {
		regulator_disable(phy->vddcx);
		dev_err(phy->dev, "cannot enable v1p8\n");
		return ret;
	}

	clk_prepare_enable(phy->ref_clk);
	usleep_range(1000, 1200);

	/* reset phy */
	data = readl_relaxed(phy->base + PHY_CTRL_REG);
	writel_relaxed(data | BIT(7), phy->base + PHY_CTRL_REG);
	usleep_range(2000, 2200);
	writel_relaxed(data, phy->base + PHY_CTRL_REG);

	/* clear REF_PAD, we don't have XO clk */
	data &= ~BIT(28);
	writel_relaxed(data, phy->base + PHY_CTRL_REG);
	msleep(30);

	data |= BIT(8) | BIT(24);
	writel_relaxed(data, phy->base + PHY_CTRL_REG);

	/*
	 * WORKAROUND: There is SSPHY suspend bug due to which USB enumerates
	 * in HS mode instead of SS mode. Workaround it by asserting
	 * LANE0.TX_ALT_BLOCK.EN_ALT_BUS to enable TX to use alt bus mode
	 */
	data = qcom_dwc3_ss_read_phycreg(phy->base, 0x102d);
	data |= (1 << 7);
	qcom_dwc3_ss_write_phycreg(phy->base, 0x102D, data);

	data = qcom_dwc3_ss_read_phycreg(phy->base, 0x1010);
	data &= ~0xff0;
	data |= 0x20;
	qcom_dwc3_ss_write_phycreg(phy->base, 0x1010, data);

	/*
	 * Fix RX Equalization setting as follows
	 * LANE0.RX_OVRD_IN_HI. RX_EQ_EN set to 0
	 * LANE0.RX_OVRD_IN_HI.RX_EQ_EN_OVRD set to 1
	 * LANE0.RX_OVRD_IN_HI.RX_EQ set to 3
	 * LANE0.RX_OVRD_IN_HI.RX_EQ_OVRD set to 1
	 */
	data = qcom_dwc3_ss_read_phycreg(phy->base, 0x1006);
	data &= ~(1 << 6);
	data |= (1 << 7);
	data &= ~(0x7 << 8);
	data |= (0x3 << 8);
	data |= (0x1 << 11);
	qcom_dwc3_ss_write_phycreg(phy->base, 0x1006, data);

	/*
	 * Set EQ and TX launch amplitudes as follows
	 * LANE0.TX_OVRD_DRV_LO.PREEMPH set to 22
	 * LANE0.TX_OVRD_DRV_LO.AMPLITUDE set to 127
	 * LANE0.TX_OVRD_DRV_LO.EN set to 1.
	 */
	data = qcom_dwc3_ss_read_phycreg(phy->base, 0x1002);
	data &= ~0x3f80;
	data |= (0x16 << 7);
	data &= ~0x7f;
	data |= (0x7f | (1 << 14));
	qcom_dwc3_ss_write_phycreg(phy->base, 0x1002, data);

	/*
	 * Set the QSCRATCH PHY_PARAM_CTRL1 parameters as follows
	 * TX_FULL_SWING [26:20] amplitude to 127
	 * TX_DEEMPH_3_5DB [13:8] to 22
	 * LOS_BIAS [2:0] to 0x5
	 */
	qcom_dwc3_ss_write_readback(phy->base, PHY_PARAM_CTRL_1,
				   0x07f03f07, 0x07f01605);
	return 0;
}

static int qcom_dwc3_ss_set_suspend(struct usb_phy *x, int suspend)
{
	struct qcom_dwc3_ss_phy *phy = phy_to_dw_phy(x);
	u32 data;

	if (!suspend) {
		/* reset phy */
		data = readl_relaxed(phy->base + PHY_CTRL_REG);
		writel_relaxed(data | BIT(7), phy->base + PHY_CTRL_REG);
		usleep_range(2000, 2200);
		writel_relaxed(data, phy->base + PHY_CTRL_REG);

		/* clear REF_PAD, we don't have XO clk */
		data &= ~BIT(28);
		writel_relaxed(data, phy->base + PHY_CTRL_REG);
		msleep(30);

		data |= BIT(8) | BIT(24);
		writel_relaxed(data, phy->base + PHY_CTRL_REG);

		/*
		 * WORKAROUND: There is SSPHY suspend bug due to which USB
		 * enumerates in HS mode instead of SS mode. Workaround it by
		 * asserting LANE0.TX_ALT_BLOCK.EN_ALT_BUS to enable TX to use
		 * alt bus mode
		 */
		data = qcom_dwc3_ss_read_phycreg(phy->base, 0x102d);
		data |= (1 << 7);
		qcom_dwc3_ss_write_phycreg(phy->base, 0x102D, data);

		data = qcom_dwc3_ss_read_phycreg(phy->base, 0x1010);
		data &= ~0xff0;
		data |= 0x20;
		qcom_dwc3_ss_write_phycreg(phy->base, 0x1010, data);

		/*
		 * Fix RX Equalization setting as follows
		 * LANE0.RX_OVRD_IN_HI. RX_EQ_EN set to 0
		 * LANE0.RX_OVRD_IN_HI.RX_EQ_EN_OVRD set to 1
		 * LANE0.RX_OVRD_IN_HI.RX_EQ set to 3
		 * LANE0.RX_OVRD_IN_HI.RX_EQ_OVRD set to 1
		 */
		data = qcom_dwc3_ss_read_phycreg(phy->base, 0x1006);
		data &= ~(1 << 6);
		data |= (1 << 7);
		data &= ~(0x7 << 8);
		data |= (0x3 << 8);
		data |= (0x1 << 11);
		qcom_dwc3_ss_write_phycreg(phy->base, 0x1006, data);

		/*
		 * Set EQ and TX launch amplitudes as follows
		 * LANE0.TX_OVRD_DRV_LO.PREEMPH set to 22
		 * LANE0.TX_OVRD_DRV_LO.AMPLITUDE set to 127
		 * LANE0.TX_OVRD_DRV_LO.EN set to 1.
		 */
		data = qcom_dwc3_ss_read_phycreg(phy->base, 0x1002);
		data &= ~0x3f80;
		data |= (0x16 << 7);
		data &= ~0x7f;
		data |= (0x7f | (1 << 14));
		qcom_dwc3_ss_write_phycreg(phy->base, 0x1002, data);

		/*
		 * Set the QSCRATCH PHY_PARAM_CTRL1 parameters as follows
		 * TX_FULL_SWING [26:20] amplitude to 127
		 * TX_DEEMPH_3_5DB [13:8] to 22
		 * LOS_BIAS [2:0] to 0x5
		 */
		qcom_dwc3_ss_write_readback(phy->base, PHY_PARAM_CTRL_1,
					   0x07f03f07, 0x07f01605);
	}
	return 0;
}

static int qcom_dwc3_ss_probe(struct platform_device *pdev)
{
	struct qcom_dwc3_ss_phy	*phy;
	struct resource		*res;
	void __iomem		*base;
	int ret;

	phy = devm_kzalloc(&pdev->dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	platform_set_drvdata(pdev, phy);

	phy->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(phy->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	phy->vddcx = devm_regulator_get(phy->dev, "vddcx");
	if (IS_ERR(phy->vddcx)) {
		dev_dbg(phy->dev, "cannot get vddcx\n");
		return  PTR_ERR(phy->vddcx);
	}

	phy->v1p8 = devm_regulator_get(phy->dev, "v1p8");
	if (IS_ERR(phy->v1p8)) {
		dev_dbg(phy->dev, "cannot get v1p8\n");
		return  PTR_ERR(phy->v1p8);
	}

	phy->xo_clk = devm_clk_get(phy->dev, "xo");
	if (IS_ERR(phy->xo_clk)) {
		dev_dbg(phy->dev, "cannot get XO clk, assuming not present\n");
		phy->xo_clk = NULL;
	}

	phy->ref_clk = devm_clk_get(phy->dev, "ref");
	if (IS_ERR(phy->ref_clk)) {
		dev_dbg(phy->dev, "cannot get ref clock handle\n");
		return PTR_ERR(phy->ref_clk);
	}

	clk_set_rate(phy->ref_clk, 125000000);
	if (phy->xo_clk)
		clk_prepare_enable(phy->xo_clk);

	phy->base		= base;
	phy->phy.dev		= phy->dev;
	phy->phy.label		= "qcom-dwc3-ssphy";
	phy->phy.init		= qcom_dwc3_ss_phy_init;
	phy->phy.shutdown       = qcom_dwc3_ss_phy_shutdown;
	phy->phy.set_suspend	= qcom_dwc3_ss_set_suspend;
	phy->phy.type		= USB_PHY_TYPE_USB3;

	ret = usb_add_phy_dev(&phy->phy);
	return ret;
}

static int qcom_dwc3_ss_remove(struct platform_device *pdev)
{
	struct qcom_dwc3_ss_phy *phy = platform_get_drvdata(pdev);

	if (phy->xo_clk)
		clk_disable_unprepare(phy->xo_clk);
	usb_remove_phy(&phy->phy);
	return 0;
}

static const struct of_device_id qcom_dwc3_ss_id_table[] = {
	{ .compatible = "qcom,dwc3-ssphy" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, qcom_dwc3_ss_id_table);

static struct platform_driver qcom_dwc3_ss_driver = {
	.probe		= qcom_dwc3_ss_probe,
	.remove		= qcom_dwc3_ss_remove,
	.driver		= {
		.name	= "qcom-dwc3-ssphy",
		.owner	= THIS_MODULE,
		.of_match_table = qcom_dwc3_ss_id_table,
	},
};

module_platform_driver(qcom_dwc3_ss_driver);

MODULE_ALIAS("platform:qcom-dwc3-ssphy");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 QCOM SSPHY driver");
