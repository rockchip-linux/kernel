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
#define QSCRATCH_CTRL_REG		(0x04)
#define QSCRATCH_GENERAL_CFG		(0x08)
#define PHY_CTRL_REG			(0x10)
#define PARAMETER_OVERRIDE_X_REG	(0x14)
#define CHARGING_DET_CTRL_REG		(0x18)
#define CHARGING_DET_OUTPUT_REG		(0x1c)
#define ALT_INTERRUPT_EN_REG		(0x20)
#define PHY_IRQ_STAT_REG		(0x24)
#define CGCTL_REG			(0x28)

#define PHY_3P3_VOL_MIN			3050000 /* uV */
#define PHY_3P3_VOL_MAX			3300000 /* uV */
#define PHY_3P3_HPM_LOAD		16000	/* uA */

#define PHY_1P8_VOL_MIN			1800000 /* uV */
#define PHY_1P8_VOL_MAX			1800000 /* uV */
#define PHY_1P8_HPM_LOAD		19000	/* uA */

/* TODO: these are suspicious */
#define USB_VDDCX_NO			1	/* index */
#define USB_VDDCX_MIN			5	/* index */
#define USB_VDDCX_MAX			7	/* index */

struct qcom_dwc3_hs_phy {
	struct usb_phy		phy;
	void __iomem		*base;
	struct device		*dev;

	struct clk		*xo_clk;
	struct clk		*utmi_clk;

	struct regulator	*v3p3;
	struct regulator	*v1p8;
	struct regulator	*vddcx;
	struct regulator	*vbus;
};

#define	phy_to_dw_phy(x)	container_of((x), struct qcom_dwc3_hs_phy, phy)


/**
 * Write register.
 *
 * @base - QCOM DWC3 PHY base virtual address.
 * @offset - register offset.
 * @val - value to write.
 */
static inline void qcom_dwc3_hs_write(void __iomem *base, u32 offset, u32 val)
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
static inline void qcom_dwc3_hs_write_readback(void __iomem *base, u32 offset,
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

static int qcom_dwc3_hs_notify_connect(struct usb_phy *x,
	enum usb_device_speed speed)
{
	struct qcom_dwc3_hs_phy *phy = phy_to_dw_phy(x);

	dev_err(phy->dev, "notify connect\n");
	return 0;
}

static int qcom_dwc3_hs_notify_disconnect(struct usb_phy *x,
	enum usb_device_speed speed)
{
	struct qcom_dwc3_hs_phy *phy = phy_to_dw_phy(x);

	dev_err(phy->dev, "notify disconnect\n");
	return 0;
}


static void qcom_dwc3_hs_phy_shutdown(struct usb_phy *x)
{
	struct qcom_dwc3_hs_phy *phy = phy_to_dw_phy(x);
	int ret;

	ret = regulator_set_voltage(phy->v3p3, 0, PHY_3P3_VOL_MAX);
	if (ret)
		dev_err(phy->dev, "cannot set voltage for v3p3\n");

	ret = regulator_set_voltage(phy->v1p8, 0, PHY_1P8_VOL_MAX);
	if (ret)
		dev_err(phy->dev, "cannot set voltage for v1p8\n");

	ret = regulator_disable(phy->v1p8);
	if (ret)
		dev_err(phy->dev, "cannot disable v1p8\n");

	ret = regulator_disable(phy->v3p3);
	if (ret)
		dev_err(phy->dev, "cannot disable v3p3\n");

	ret = regulator_set_voltage(phy->vddcx, USB_VDDCX_NO, USB_VDDCX_MAX);
	if (ret)
		dev_err(phy->dev, "cannot set voltage for vddcx\n");

	ret = regulator_disable(phy->vddcx);
	if (ret)
		dev_err(phy->dev, "cannot enable vddcx\n");

	clk_disable_unprepare(phy->utmi_clk);
}

static int qcom_dwc3_hs_phy_init(struct usb_phy *x)
{
	struct qcom_dwc3_hs_phy	*phy = phy_to_dw_phy(x);
	int ret;
	u32 val;

	clk_prepare_enable(phy->utmi_clk);

	ret = regulator_set_voltage(phy->vddcx, USB_VDDCX_MIN, USB_VDDCX_MAX);
	if (ret)
		dev_err(phy->dev, "cannot set voltage for vddcx\n");

	ret = regulator_enable(phy->vddcx);
	if (ret)
		dev_err(phy->dev, "cannot enable vddcx\n");

	ret = regulator_set_voltage(phy->v3p3, PHY_3P3_VOL_MIN,
				    PHY_3P3_VOL_MAX);
	if (ret)
		dev_err(phy->dev, "cannot set voltage for v3p3\n");

	ret = regulator_set_voltage(phy->v1p8, PHY_1P8_VOL_MIN,
				    PHY_1P8_VOL_MAX);
	if (ret)
		dev_err(phy->dev, "cannot set voltage for v1p8\n");

	ret = regulator_set_optimum_mode(phy->v1p8, PHY_1P8_HPM_LOAD);
	if (ret < 0)
		dev_err(phy->dev, "cannot set HPM of regulator v1p8\n");

	ret = regulator_enable(phy->v1p8);
	if (ret)
		dev_err(phy->dev, "cannot enable v1p8\n");

	ret = regulator_set_optimum_mode(phy->v3p3, PHY_3P3_HPM_LOAD);
	if (ret < 0)
		dev_err(phy->dev, "cannot set HPM of regulator v3p3\n");

	ret = regulator_enable(phy->v3p3);
	if (ret)
		dev_err(phy->dev, "cannot enable v3p3\n");

	/*
	 * HSPHY Initialization: Enable UTMI clock and clamp enable HVINTs,
	 * and disable RETENTION (power-on default is ENABLED)
	 */
	val = readl(phy->base + PHY_CTRL_REG);
	val |= BIT(18) | BIT(20) | BIT(11) | 0x70;
	qcom_dwc3_hs_write(phy->base, PHY_CTRL_REG, val);
	usleep_range(2000, 2200);

	/*
	 * write HSPHY init value to QSCRATCH reg to set HSPHY parameters like
	 * VBUS valid threshold, disconnect valid threshold, DC voltage level,
	 * preempasis and rise/fall time.
	 */
	qcom_dwc3_hs_write_readback(phy->base, PARAMETER_OVERRIDE_X_REG,
				0x03ffffff, 0x00d191a4);

	/* Disable (bypass) VBUS and ID filters */
	qcom_dwc3_hs_write(phy->base, QSCRATCH_GENERAL_CFG, BIT(2));

	return 0;
}

static int qcom_dwc3_hs_phy_set_vbus(struct usb_phy *x, int on)
{
	struct qcom_dwc3_hs_phy	*phy = phy_to_dw_phy(x);
	int ret = 0;

	if (IS_ERR(phy->vbus))
		return on ? PTR_ERR(phy->vbus) : 0;

	if (on)
		ret = regulator_enable(phy->vbus);
	else
		ret = regulator_disable(phy->vbus);

	if (ret)
		dev_err(x->dev, "Cannot %s Vbus\n", on ? "set" : "off");
	return ret;
}

static int qcom_dwc3_hs_probe(struct platform_device *pdev)
{
	struct qcom_dwc3_hs_phy	*phy;
	struct resource		*res;
	void __iomem		*base;

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

	phy->v3p3 = devm_regulator_get(phy->dev, "v3p3");
	if (IS_ERR(phy->v3p3)) {
		dev_dbg(phy->dev, "cannot get v3p3\n");
		return PTR_ERR(phy->v3p3);
	}

	phy->v1p8 = devm_regulator_get(phy->dev, "v1p8");
	if (IS_ERR(phy->v1p8)) {
		dev_dbg(phy->dev, "cannot get v1p8\n");
		return PTR_ERR(phy->v1p8);
	}

	phy->vbus = devm_regulator_get(phy->dev, "vbus");
	if (IS_ERR(phy->vbus))
		dev_dbg(phy->dev, "Failed to get vbus\n");

	phy->utmi_clk = devm_clk_get(phy->dev, "utmi");
	if (IS_ERR(phy->utmi_clk)) {
		dev_dbg(phy->dev, "cannot get UTMI handle\n");
		return PTR_ERR(phy->utmi_clk);
	}

	phy->xo_clk = devm_clk_get(phy->dev, "xo");
	if (IS_ERR(phy->xo_clk)) {
		dev_dbg(phy->dev, "cannot get TCXO buffer handle\n");
		phy->xo_clk = NULL;
	}

	clk_set_rate(phy->utmi_clk, 60000000);

	if (phy->xo_clk)
		clk_prepare_enable(phy->xo_clk);

	phy->base		= base;
	phy->phy.dev		= phy->dev;
	phy->phy.label		= "qcom-dwc3-hsphy";
	phy->phy.init		= qcom_dwc3_hs_phy_init;
	phy->phy.notify_connect	= qcom_dwc3_hs_notify_connect;
	phy->phy.notify_disconnect = qcom_dwc3_hs_notify_disconnect;
	phy->phy.shutdown	= qcom_dwc3_hs_phy_shutdown;
	phy->phy.set_vbus	= qcom_dwc3_hs_phy_set_vbus;
	phy->phy.type		= USB_PHY_TYPE_USB2;
	phy->phy.state          = OTG_STATE_UNDEFINED;

	usb_add_phy_dev(&phy->phy);
	return 0;
}

static int qcom_dwc3_hs_remove(struct platform_device *pdev)
{
	struct qcom_dwc3_hs_phy *phy = platform_get_drvdata(pdev);

	if (phy->xo_clk)
		clk_disable_unprepare(phy->xo_clk);
	usb_remove_phy(&phy->phy);
	return 0;
}

static const struct of_device_id qcom_dwc3_hs_id_table[] = {
	{ .compatible = "qcom,dwc3-hsphy" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, qcom_dwc3_hs_id_table);

static struct platform_driver qcom_dwc3_hs_driver = {
	.probe		= qcom_dwc3_hs_probe,
	.remove		= qcom_dwc3_hs_remove,
	.driver		= {
		.name	= "qcom-dwc3-hsphy",
		.owner	= THIS_MODULE,
		.of_match_table = qcom_dwc3_hs_id_table,
	},
};

module_platform_driver(qcom_dwc3_hs_driver);

MODULE_ALIAS("platform:qcom-dwc3-hsphy");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 QCOM HSPHY driver");
