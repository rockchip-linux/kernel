/*
 * RV1108 USB2.0 Host(EHCI & OHCI) PHY Driver
 *
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#define BIT_WRITEABLE_SHIFT	16
#define SCHEDULE_DELAY	(60 * HZ)

enum rv1108_usb2phy_host_state {
	PHY_STATE_HS_ONLINE	= 0,
	PHY_STATE_DISCONNECT	= 1,
	PHY_STATE_CONNECT	= 2,
	PHY_STATE_FS_LS_ONLINE	= 4,
};

struct usb2phy_reg {
	unsigned int	offset;
	unsigned int	bitend;
	unsigned int	bitstart;
	unsigned int	disable;
	unsigned int	enable;
};

/**
 * struct rv1108_usb2phy_cfg: usb-phy register configuration.
 * @phy_sus: phy suspend register.
 * @ls_det_en: linestate detection enable register.
 * @ls_det_st: linestate detection state register.
 * @ls_det_clr: linestate detection clear register.
 * @utmi_ls: utmi linestate state register.
 * @utmi_hstdet: utmi host disconnect register.
 */
struct rv1108_usb2phy_cfg {
	struct usb2phy_reg	phy_sus;
	struct usb2phy_reg	ls_det_en;
	struct usb2phy_reg	ls_det_st;
	struct usb2phy_reg	ls_det_clr;
	struct usb2phy_reg	utmi_ls;
	struct usb2phy_reg	utmi_hstdet;
};

/**
 * struct rv1108_usb2phy: usb2.0 phy driver data.
 * @grf: General Register Files regmap.
 * @usb_grf: USB General Register Files regmap.
 * @clk: clock struct of phy input clk.
 * @suspended: phy suspended flag.
 * @ls_irq: IRQ number assigned for linestate detection.
 * @mutex: for register updating in sm_work.
 * @sm_work: OTG state machine work.
 * @cfg: phy register configuration, assigned by driver data.
 */
struct rv1108_usb2phy {
	struct device		*dev;
	struct phy		*phy;
	struct regmap		*grf;
	struct regmap		*usb_grf;
	struct clk		*clk;
	bool			suspended;
	int			ls_irq;
	struct mutex		mutex;
	struct delayed_work	sm_work;
	const struct	rv1108_usb2phy_cfg	*cfg;
};

static inline int property_enable(struct regmap *base,
				  const struct usb2phy_reg *reg, bool en)
{
	unsigned int val, mask, tmp;

	tmp = en ? reg->enable : reg->disable;
	mask = GENMASK(reg->bitend, reg->bitstart);
	val = (tmp << reg->bitstart) | (mask << BIT_WRITEABLE_SHIFT);

	return regmap_write(base, reg->offset, val);
}

static inline bool property_enabled(struct regmap *base,
				    const struct usb2phy_reg *reg)
{
	int ret;
	unsigned int tmp, orig;
	unsigned int mask = GENMASK(reg->bitend, reg->bitstart);

	ret = regmap_read(base, reg->offset, &orig);
	if (ret)
		return false;

	tmp = (orig & mask) >> reg->bitstart;
	return tmp == reg->enable;
}

static int rv1108_usb2phy_init(struct phy *phy)
{
	struct rv1108_usb2phy *rphy = phy_get_drvdata(phy);
	int ret;

	/* clear linestate and enable linestate detect irq */
	mutex_lock(&rphy->mutex);

	ret = property_enable(rphy->grf, &rphy->cfg->ls_det_clr, true);
	if (ret) {
		mutex_unlock(&rphy->mutex);
		return ret;
	}

	ret = property_enable(rphy->grf, &rphy->cfg->ls_det_en, true);
	if (ret) {
		mutex_unlock(&rphy->mutex);
		return ret;
	}

	mutex_unlock(&rphy->mutex);
	schedule_delayed_work(&rphy->sm_work, SCHEDULE_DELAY);

	return 0;
}

static int rv1108_usb2phy_power_on(struct phy *phy)
{
	struct rv1108_usb2phy *rphy = phy_get_drvdata(phy);
	int ret;

	dev_dbg(rphy->dev, "port power on\n");

	if (!rphy->suspended)
		return 0;

	ret = property_enable(rphy->usb_grf, &rphy->cfg->phy_sus, false);
	if (ret)
		return ret;

	rphy->suspended = false;
	return 0;
}

static int rv1108_usb2phy_power_off(struct phy *phy)
{
	struct rv1108_usb2phy *rphy = phy_get_drvdata(phy);
	int ret;

	dev_dbg(rphy->dev, "port power off\n");

	if (rphy->suspended)
		return 0;

	ret = property_enable(rphy->usb_grf, &rphy->cfg->phy_sus, true);
	if (ret)
		return ret;

	rphy->suspended = true;
	return 0;
}

static int rv1108_usb2phy_exit(struct phy *phy)
{
	struct rv1108_usb2phy *rphy = phy_get_drvdata(phy);

	cancel_delayed_work_sync(&rphy->sm_work);
	return 0;
}

static const struct phy_ops rv1108_usb2phy_ops = {
	.init		= rv1108_usb2phy_init,
	.exit		= rv1108_usb2phy_exit,
	.power_on	= rv1108_usb2phy_power_on,
	.power_off	= rv1108_usb2phy_power_off,
	.owner		= THIS_MODULE,
};

/*
 * The function manage host-phy port state and suspend/resume phy port
 * to save power automatically.
 *
 * we rely on utmi_linestate and utmi_hostdisconnect to identify whether
 * devices is disconnect or not. Besides, we do not need care it is FS/LS
 * disconnected or HS disconnected, actually, we just only need get the
 * device is disconnected at last through rearm the delayed work,
 * to suspend the phy port in _PHY_STATE_DISCONNECT_ case.
 */
static void rv1108_usb2phy_sm_work(struct work_struct *work)
{
	struct rv1108_usb2phy *rphy =
		container_of(work, struct rv1108_usb2phy, sm_work.work);
	unsigned int sh = rphy->cfg->utmi_hstdet.bitend -
			  rphy->cfg->utmi_hstdet.bitstart + 1;
	unsigned int ul, uhd, state;
	unsigned int ul_mask, uhd_mask;
	int ret;

	mutex_lock(&rphy->mutex);

	ret = regmap_read(rphy->grf, rphy->cfg->utmi_ls.offset, &ul);
	if (ret < 0)
		goto next_schedule;

	ret = regmap_read(rphy->grf, rphy->cfg->utmi_hstdet.offset, &uhd);
	if (ret < 0)
		goto next_schedule;

	uhd_mask = GENMASK(rphy->cfg->utmi_hstdet.bitend,
			   rphy->cfg->utmi_hstdet.bitstart);
	ul_mask = GENMASK(rphy->cfg->utmi_ls.bitend,
			  rphy->cfg->utmi_ls.bitstart);

	/* stitch on utmi_ls and utmi_hstdet as phy state */
	state = ((uhd & uhd_mask) >> rphy->cfg->utmi_hstdet.bitstart) |
		(((ul & ul_mask) >> rphy->cfg->utmi_ls.bitstart) << sh);

	switch (state) {
	case PHY_STATE_HS_ONLINE:
		dev_dbg(rphy->dev, "HS online\n");
		break;
	case PHY_STATE_FS_LS_ONLINE:
		/*
		 * For FS/LS device, the online state share with connect state
		 * from utmi_ls and utmi_hstdet register, so we distinguish
		 * them via suspended flag.
		 *
		 * Plus, there are two cases, one is D- Line pull-up, and D+
		 * line pull-down, the state is 4; another is D+ line pull-up,
		 * and D- line pull-down, the state is 2.
		 */
		if (!rphy->suspended) {
			/* D- line pull-up, D+ line pull-down */
			dev_dbg(rphy->dev, "FS/LS online\n");
			break;
		}
		/* fall through */
	case PHY_STATE_CONNECT:
		if (rphy->suspended) {
			dev_dbg(rphy->dev, "Connected\n");
			rv1108_usb2phy_power_on(rphy->phy);
			rphy->suspended = false;
		} else {
			/* D+ line pull-up, D- line pull-down */
			dev_dbg(&rphy->phy->dev, "FS/LS online\n");
		}
		break;
	case PHY_STATE_DISCONNECT:
		if (!rphy->suspended) {
			dev_dbg(rphy->dev, "Disconnected\n");
			rv1108_usb2phy_power_off(rphy->phy);
			rphy->suspended = true;
		}

		/*
		 * activate the linestate detection to get the next device
		 * plug-in irq.
		 */
		property_enable(rphy->grf, &rphy->cfg->ls_det_clr, true);
		property_enable(rphy->grf, &rphy->cfg->ls_det_en, true);

		/*
		 * we don't need to rearm the delayed work when the phy port
		 * is suspended.
		 */
		mutex_unlock(&rphy->mutex);
		return;
	default:
		dev_dbg(rphy->dev, "unknown phy state\n");
		break;
	}

next_schedule:
	mutex_unlock(&rphy->mutex);
	schedule_delayed_work(&rphy->sm_work, SCHEDULE_DELAY);
}

static irqreturn_t rv1108_usb2phy_linestate_irq(int irq, void *data)
{
	struct rv1108_usb2phy *rphy = data;

	if (!property_enabled(rphy->grf, &rphy->cfg->ls_det_st))
		return IRQ_NONE;

	mutex_lock(&rphy->mutex);

	/* disable linestate detect irq and clear its status */
	property_enable(rphy->grf, &rphy->cfg->ls_det_en, false);
	property_enable(rphy->grf, &rphy->cfg->ls_det_clr, true);

	mutex_unlock(&rphy->mutex);

	/*
	 * In this case for host phy, a new device is plugged in, meanwhile,
	 * if the phy port is suspended, we need rearm the work to resume it
	 * and mange its states; otherwise, we just return irq handled.
	 */
	if (rphy->suspended)
		rv1108_usb2phy_sm_work(&rphy->sm_work.work);

	return IRQ_HANDLED;
}

static int rv1108_usb2phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct phy_provider *provider;
	struct rv1108_usb2phy *rphy;
	struct phy *phy;
	const struct of_device_id *match;
	int ret;

	rphy = devm_kzalloc(dev, sizeof(*rphy), GFP_KERNEL);
	if (!rphy)
		return -ENOMEM;

	match = of_match_device(dev->driver->of_match_table, dev);
	if (!match || !match->data) {
		dev_err(dev, "phy configs are not assigned\n");
		return -EINVAL;
	}

	rphy->grf = syscon_regmap_lookup_by_phandle(dev->of_node,
						    "rockchip,grf");
	if (IS_ERR(rphy->grf))
		return PTR_ERR(rphy->grf);

	rphy->usb_grf = syscon_regmap_lookup_by_phandle(dev->of_node,
							"rockchip,usbgrf");
	if (IS_ERR(rphy->usb_grf))
		return PTR_ERR(rphy->usb_grf);

	rphy->dev = dev;
	rphy->cfg = match->data;
	platform_set_drvdata(pdev, rphy);

	rphy->clk = of_clk_get_by_name(np, "phyclk");
	if (!IS_ERR(rphy->clk)) {
		clk_prepare_enable(rphy->clk);
	} else {
		dev_info(&pdev->dev, "no phyclk specified\n");
		rphy->clk = NULL;
	}

	rphy->ls_irq = platform_get_irq_byname(pdev, "linestate");
	if (rphy->ls_irq < 0) {
		dev_err(rphy->dev, "get linestate irq failed\n");
		ret = -ENXIO;
		goto disable_clks;
	}

	phy = devm_phy_create(dev, np, &rv1108_usb2phy_ops);
	if (IS_ERR(phy)) {
		dev_err(dev, "failed to create phy\n");
		ret = PTR_ERR(phy);
		goto disable_clks;
	}

	rphy->phy = phy;
	phy_set_drvdata(phy, rphy);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR_OR_NULL(provider))
		goto disable_clks;

	mutex_init(&rphy->mutex);
	INIT_DELAYED_WORK(&rphy->sm_work, rv1108_usb2phy_sm_work);

	ret = devm_request_threaded_irq(rphy->dev, rphy->ls_irq, NULL,
					rv1108_usb2phy_linestate_irq,
					IRQF_ONESHOT,
					"rv1108_usb2phy", rphy);
	if (ret) {
		dev_err(rphy->dev, "failed to request linestate irq handle\n");
		goto disable_clks;
	}

	return 0;

disable_clks:
	if (rphy->clk) {
		clk_disable_unprepare(rphy->clk);
		clk_put(rphy->clk);
	}
	return ret;
}

static const struct rv1108_usb2phy_cfg rv1108_host_phy_cfg = {
	.phy_sus	= { 0x0104, 15, 0, 0, 0x1d1 },
	.ls_det_en	= { 0x0680, 4, 4, 0, 1 },
	.ls_det_st	= { 0x0690, 4, 4, 0, 1 },
	.ls_det_clr	= { 0x06a0, 4, 4, 0, 1 },
	.utmi_ls	= { 0x0804, 9, 8, 0, 1 },
	.utmi_hstdet	= { 0x0804, 7, 7, 0, 1 }
};

static const struct of_device_id rv1108_usb2phy_dt_match[] = {
	{
		.compatible = "rockchip,rv1108-usb2phy",
		.data = &rv1108_host_phy_cfg
	},
	{}
};
MODULE_DEVICE_TABLE(of, rv1108_usb2phy_dt_match);

static struct platform_driver rv1108_usb2phy_driver = {
	.probe		= rv1108_usb2phy_probe,
	.driver		= {
		.name	= "rv1108-usb2phy",
		.of_match_table = rv1108_usb2phy_dt_match,
	},
};
module_platform_driver(rv1108_usb2phy_driver);

MODULE_AUTHOR("Frank Wang <frank.wang@rock-chips.com>");
MODULE_DESCRIPTION("rv1108 USB2.0 PHY driver");
MODULE_LICENSE("GPL v2");
