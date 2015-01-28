/*
 * IMG Pistachio USB PHY driver
 *
 * Copyright (C) 2015 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define USB_PHY_CONTROL1				0x04
#define USB_PHY_CONTROL1_FSEL_SHIFT			2
#define USB_PHY_CONTROL1_FSEL_MASK			0x7
#define USB_PHY_CONTROL1_FSEL_VAL			0x7

#define USB_PHY_STRAP_CONTROL				0x10
#define USB_PHY_STRAP_CONTROL_REFCLKSEL_SHIFT		4
#define USB_PHY_STRAP_CONTROL_REFCLKSEL_MASK		0x3
#define USB_PHY_STRAP_CONTROL_REFCLKSEL_CLKCORE		0x2

#define USB_PHY_STATUS					0x14
#define USB_PHY_STATUS_RX_PHY_CLK			BIT(9)
#define USB_PHY_STATUS_RX_UTMI_CLK			BIT(8)
#define USB_PHY_STATUS_VBUS_FAULT			BIT(7)

struct pistachio_usb_phy {
	struct device *dev;
	struct regmap *cr_top;
	struct clk *phy_clk;
};

static inline u32 phy_readl(struct pistachio_usb_phy *p_phy, u32 reg)
{
	unsigned int val;

	regmap_read(p_phy->cr_top, reg, &val);
	return val;
}

static inline void phy_writel(struct pistachio_usb_phy *p_phy, u32 val, u32 reg)
{
	regmap_write(p_phy->cr_top, reg, val);
}

static int pistachio_usb_phy_power_on(struct phy *phy)
{
	struct pistachio_usb_phy *p_phy = phy_get_drvdata(phy);
	unsigned long timeout;
	u32 val;
	int ret;

	ret = clk_prepare_enable(p_phy->phy_clk);
	if (ret < 0) {
		dev_err(p_phy->dev, "Failed to enable PHY clock: %d\n", ret);
		return ret;
	}

	val = phy_readl(p_phy, USB_PHY_STRAP_CONTROL);
	val &= ~(USB_PHY_STRAP_CONTROL_REFCLKSEL_MASK <<
		 USB_PHY_STRAP_CONTROL_REFCLKSEL_SHIFT);
	val |= USB_PHY_STRAP_CONTROL_REFCLKSEL_CLKCORE <<
		USB_PHY_STRAP_CONTROL_REFCLKSEL_SHIFT;
	phy_writel(p_phy, val, USB_PHY_STRAP_CONTROL);

	val = phy_readl(p_phy, USB_PHY_CONTROL1);
	val &= ~(USB_PHY_CONTROL1_FSEL_MASK <<
		 USB_PHY_CONTROL1_FSEL_SHIFT);
	val |= USB_PHY_CONTROL1_FSEL_VAL <<
		USB_PHY_CONTROL1_FSEL_SHIFT;
	phy_writel(p_phy, val, USB_PHY_CONTROL1);

	timeout = jiffies + msecs_to_jiffies(200);
	while (time_before(jiffies, timeout)) {
		val = phy_readl(p_phy, USB_PHY_STATUS);
		if (val & USB_PHY_STATUS_VBUS_FAULT) {
			dev_err(p_phy->dev, "VBUS fault detected\n");
			ret = -EIO;
			goto disable_clk;
		}
		if ((val & USB_PHY_STATUS_RX_PHY_CLK) &&
		    (val & USB_PHY_STATUS_RX_UTMI_CLK))
			return 0;
		usleep_range(1000, 1500);
	}

	dev_err(p_phy->dev, "Timed out waiting for PHY to power on\n");
	ret = -ETIMEDOUT;

disable_clk:
	clk_disable_unprepare(p_phy->phy_clk);
	return ret;
}

static int pistachio_usb_phy_power_off(struct phy *phy)
{
	struct pistachio_usb_phy *p_phy = phy_get_drvdata(phy);

	clk_disable_unprepare(p_phy->phy_clk);

	return 0;
}

static const struct phy_ops pistachio_usb_phy_ops = {
	.power_on = pistachio_usb_phy_power_on,
	.power_off = pistachio_usb_phy_power_off,
	.owner = THIS_MODULE,
};

static int pistachio_usb_phy_probe(struct platform_device *pdev)
{
	struct pistachio_usb_phy *p_phy;
	struct phy_provider *provider;
	struct phy *phy;

	p_phy = devm_kzalloc(&pdev->dev, sizeof(*p_phy), GFP_KERNEL);
	if (!p_phy)
		return -ENOMEM;
	p_phy->dev = &pdev->dev;
	platform_set_drvdata(pdev, p_phy);

	p_phy->cr_top = syscon_regmap_lookup_by_phandle(p_phy->dev->of_node,
							"img,cr-top");
	if (IS_ERR(p_phy->cr_top)) {
		dev_err(p_phy->dev, "Failed to get CR_TOP registers: %ld\n",
			PTR_ERR(p_phy->cr_top));
		return PTR_ERR(p_phy->cr_top);
	}

	p_phy->phy_clk = devm_clk_get(p_phy->dev, "usb_phy");
	if (IS_ERR(p_phy->phy_clk)) {
		dev_err(p_phy->dev, "Failed to get usb_phy clock: %ld\n",
			PTR_ERR(p_phy->phy_clk));
		return PTR_ERR(p_phy->phy_clk);
	}

	phy = devm_phy_create(p_phy->dev, NULL, &pistachio_usb_phy_ops, NULL);
	if (IS_ERR(phy)) {
		dev_err(p_phy->dev, "Failed to create PHY: %ld\n",
			PTR_ERR(phy));
		return PTR_ERR(phy);
	}
	phy_set_drvdata(phy, p_phy);

	provider = devm_of_phy_provider_register(p_phy->dev,
						 of_phy_simple_xlate);
	if (IS_ERR(provider)) {
		dev_err(p_phy->dev, "Failed to register PHY provider: %ld\n",
			PTR_ERR(provider));
		return PTR_ERR(provider);
	}

	return 0;
}

static const struct of_device_id pistachio_usb_phy_of_match[] = {
	{ .compatible = "img,pistachio-usb-phy", },
	{ },
};
MODULE_DEVICE_TABLE(of, pistachio_usb_phy_of_match);

static struct platform_driver pistachio_usb_phy_driver = {
	.probe		= pistachio_usb_phy_probe,
	.driver		= {
		.name	= "pistachio-usb-phy",
		.of_match_table = pistachio_usb_phy_of_match,
	},
};
module_platform_driver(pistachio_usb_phy_driver);

MODULE_AUTHOR("Andrew Bresticker <abrestic@chromium.org>");
MODULE_DESCRIPTION("IMG Pistachio USB2.0 PHY driver");
MODULE_LICENSE("GPL v2");
