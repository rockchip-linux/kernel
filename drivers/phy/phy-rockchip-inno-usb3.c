/*
 * Rockchip USB3.0 PHY with Innosilicon IP block driver
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
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#define U3PHY_PORT_NUM	2
#define U3PHY_MAX_CLKS	4
#define BIT_WRITEABLE_SHIFT	16
#define SCHEDULE_DELAY	(60 * HZ)

#define U3PHY_APB_RST	BIT(0)
#define U3PHY_POR_RST	BIT(1)
#define U3PHY_MAC_RST	BIT(2)

struct rockchip_u3phy;
struct rockchip_u3phy_port;

enum rockchip_u3phy_type {
	U3PHY_TYPE_PIPE,
	U3PHY_TYPE_UTMI,
};

enum rockchip_u3phy_rest_req {
	U2_POR_RSTN	= 0,
	U3_POR_RSTN	= 1,
	PIPE_MAC_RSTN	= 2,
	UTMI_MAC_RSTN	= 3,
	UTMI_APB_RSTN	= 4,
	PIPE_APB_RSTN	= 5,
	U3PHY_RESET_MAX	= 6,
};

enum rockchip_u3phy_utmi_state {
	PHY_UTMI_HS_ONLINE	= 0,
	PHY_UTMI_DISCONNECT	= 1,
	PHY_UTMI_CONNECT	= 2,
	PHY_UTMI_FS_LS_ONLINE	= 4,
};

/*
 * @rvalue: reset value
 * @dvalue: desired value
 */
struct u3phy_reg {
	unsigned int	offset;
	unsigned int	bitend;
	unsigned int	bitstart;
	unsigned int	rvalue;
	unsigned int	dvalue;
};

struct rockchip_u3phy_grfcfg {
	struct u3phy_reg	um_suspend;
	struct u3phy_reg	pp_suspend;
	struct u3phy_reg	ls_det_en;
	struct u3phy_reg	ls_det_st;
	struct u3phy_reg	um_ls;
	struct u3phy_reg	um_hstdct;
};

struct rockchip_u3phy_apbcfg {
	struct u3phy_reg	u2tx_drv_strength;

	struct u3phy_reg	u3tx_bias_current;
	struct u3phy_reg	u3tx_pre_emphasis;
	struct u3phy_reg	u3tx_drv_strength;
	struct u3phy_reg	u3tx_pll_factor;
};

struct rockchip_u3phy_cfg {
	unsigned int	reg;
	const struct rockchip_u3phy_grfcfg	grfcfg;
	const struct rockchip_u3phy_apbcfg	apbcfg;

	int (*phy_tuning)(struct rockchip_u3phy *,
			  struct rockchip_u3phy_port *);
};

struct rockchip_u3phy_port {
	struct phy	*phy;
	void __iomem	*base;
	unsigned int	index;
	unsigned char	type;
	bool		suspended;
	bool		refclk_25m_quirk;
	struct mutex		mutex; /* mutex for updating register */
	struct delayed_work	um_sm_work;
};

struct rockchip_u3phy {
	struct device	*dev;
	struct regmap	*u3phy_grf;
	int		um_ls_irq;
	struct clk	*clks[U3PHY_MAX_CLKS];
	struct reset_control *rsts[U3PHY_RESET_MAX];
	const struct rockchip_u3phy_cfg	*cfgs;
	struct rockchip_u3phy_port	ports[U3PHY_PORT_NUM];
};

static inline int param_write(bool grf, void __iomem *base,
			      const struct u3phy_reg *reg, bool desired)
{
	unsigned int val, mask;
	unsigned int tmp = desired ? reg->dvalue : reg->rvalue;
	int ret = 0;

	if (grf) {
		mask = GENMASK(reg->bitend, reg->bitstart);
		val = (tmp << reg->bitstart) | (mask << BIT_WRITEABLE_SHIFT);
		ret = regmap_write(base, reg->offset, val);
	} else {
		val = readl(base + reg->offset);
		val |= tmp << reg->bitstart;
		writel(val, base + reg->offset);
	}

	return ret;
}

static inline bool param_desired(bool grf, void __iomem *base,
				 const struct u3phy_reg *reg)
{
	int ret;
	unsigned int tmp, orig;
	unsigned int mask = GENMASK(reg->bitend, reg->bitstart);

	if (grf) {
		ret = regmap_read(base, reg->offset, &orig);
		if (ret)
			return false;
	} else {
		orig = readl(base + reg->offset);
	}

	tmp = (orig & mask) >> reg->bitstart;
	return tmp == reg->dvalue;
}

static const char *get_rest_name(enum rockchip_u3phy_rest_req rst)
{
	switch (rst) {
	case U2_POR_RSTN:
		return "u3phy-u2-por";
	case U3_POR_RSTN:
		return "u3phy-u3-por";
	case PIPE_MAC_RSTN:
		return "u3phy-pipe-mac";
	case UTMI_MAC_RSTN:
		return "u3phy-utmi-mac";
	case UTMI_APB_RSTN:
		return "u3phy-utmi-apb";
	case PIPE_APB_RSTN:
		return "u3phy-pipe-apb";
	default:
		return "invalid";
	}
}

static void rockchip_u3phy_rest_deassert(struct rockchip_u3phy *u3phy,
					 unsigned int flag)
{
	int rst;

	if (flag & U3PHY_APB_RST) {
		dev_dbg(u3phy->dev, "deassert APB bus interface reset\n");
		for (rst = UTMI_APB_RSTN; rst <= PIPE_APB_RSTN; rst++) {
			if (u3phy->rsts[rst])
				reset_control_deassert(u3phy->rsts[rst]);
		}
	}

	if (flag & U3PHY_POR_RST) {
		usleep_range(15, 20);
		dev_dbg(u3phy->dev, "deassert u2 and u3 phy power on reset\n");
		for (rst = U2_POR_RSTN; rst <= U3_POR_RSTN; rst++) {
			if (u3phy->rsts[rst])
				reset_control_deassert(u3phy->rsts[rst]);
		}
	}

	if (flag & U3PHY_MAC_RST) {
		usleep_range(2000, 2100);
		dev_dbg(u3phy->dev, "deassert pipe and utmi MAC reset\n");
		for (rst = PIPE_MAC_RSTN; rst <= UTMI_MAC_RSTN; rst++)
			if (u3phy->rsts[rst])
				reset_control_deassert(u3phy->rsts[rst]);
	}
}

static void rockchip_u3phy_rest_assert(struct rockchip_u3phy *u3phy)
{
	int rst;

	dev_dbg(u3phy->dev, "assert u3phy reset\n");
	for (rst = 0; rst < U3PHY_RESET_MAX; rst++)
		if (u3phy->rsts[rst])
			reset_control_assert(u3phy->rsts[rst]);
}

static int rockchip_u3phy_clk_enable(struct rockchip_u3phy *u3phy)
{
	int ret, clk;

	for (clk = 0; clk < U3PHY_MAX_CLKS && u3phy->clks[clk]; clk++) {
		ret = clk_prepare_enable(u3phy->clks[clk]);
		if (ret)
			goto err_disable_clks;
	}
	return 0;

err_disable_clks:
	while (--clk >= 0)
		clk_disable_unprepare(u3phy->clks[clk]);
	return ret;
}

static void rockchip_u3phy_clk_disable(struct rockchip_u3phy *u3phy)
{
	int clk;

	for (clk = U3PHY_MAX_CLKS - 1; clk >= 0; clk--)
		if (u3phy->clks[clk])
			clk_disable_unprepare(u3phy->clks[clk]);
}

static int rockchip_u3phy_init(struct phy *phy)
{
	struct rockchip_u3phy_port *u3phy_port = phy_get_drvdata(phy);
	struct rockchip_u3phy *u3phy = dev_get_drvdata(phy->dev.parent);

	mutex_lock(&u3phy_port->mutex);

	if (u3phy_port->type == U3PHY_TYPE_UTMI) {
		param_write(1, u3phy->u3phy_grf,
			    &u3phy->cfgs->grfcfg.ls_det_st, 1);
		param_write(1, u3phy->u3phy_grf,
			    &u3phy->cfgs->grfcfg.ls_det_en, 1);

		schedule_delayed_work(&u3phy_port->um_sm_work, SCHEDULE_DELAY);
	}

	mutex_unlock(&u3phy_port->mutex);
	return 0;
}

static int rockchip_u3phy_power_on(struct phy *phy)
{
	struct rockchip_u3phy_port *u3phy_port = phy_get_drvdata(phy);
	struct rockchip_u3phy *u3phy = dev_get_drvdata(phy->dev.parent);
	int ret;

	dev_info(&u3phy_port->phy->dev, "u3phy utmi power on\n");

	if (!u3phy_port->suspended)
		return 0;

	if (u3phy_port->type == U3PHY_TYPE_UTMI) {
		ret = param_write(true, u3phy->u3phy_grf,
				  &u3phy->cfgs->grfcfg.um_suspend, false);
		if (ret)
			return ret;
	}

	u3phy_port->suspended = false;
	return 0;
}

static int rockchip_u3phy_power_off(struct phy *phy)
{
	struct rockchip_u3phy_port *u3phy_port = phy_get_drvdata(phy);
	struct rockchip_u3phy *u3phy = dev_get_drvdata(phy->dev.parent);
	int ret;

	dev_info(&u3phy_port->phy->dev, "u3phy utmi power off\n");

	if (u3phy_port->suspended)
		return 0;

	if (u3phy_port->type == U3PHY_TYPE_UTMI) {
		ret = param_write(true, u3phy->u3phy_grf,
				  &u3phy->cfgs->grfcfg.um_suspend, true);
		if (ret)
			return ret;
	}

	u3phy_port->suspended = true;
	return 0;
}

static int rockchip_u3phy_exit(struct phy *phy)
{
	struct rockchip_u3phy_port *u3phy_port = phy_get_drvdata(phy);
	struct rockchip_u3phy *u3phy = dev_get_drvdata(phy->dev.parent);

	if (u3phy_port->type == U3PHY_TYPE_UTMI)
		cancel_delayed_work_sync(&u3phy_port->um_sm_work);

	rockchip_u3phy_clk_disable(u3phy);
	return 0;
}

static __maybe_unused
struct phy *rockchip_u3phy_xlate(struct device *dev,
				 struct of_phandle_args *args)
{
	struct rockchip_u3phy *u3phy = dev_get_drvdata(dev);
	struct rockchip_u3phy_port *u3phy_port = NULL;
	struct device_node *phy_np = args->np;
	int index;

	if (args->args_count != 1) {
		dev_err(dev, "invalid number of cells in 'phy' property\n");
		return ERR_PTR(-EINVAL);
	}

	for (index = 0; index < U3PHY_PORT_NUM; index++) {
		if (phy_np == u3phy->ports[index].phy->dev.of_node) {
			u3phy_port = &u3phy->ports[index];
			break;
		}
	}

	if (!u3phy_port) {
		dev_err(dev, "failed to find appropriate phy\n");
		return ERR_PTR(-EINVAL);
	}

	return u3phy_port->phy;
}

static struct phy_ops rockchip_u3phy_ops = {
	.init		= rockchip_u3phy_init,
	.exit		= rockchip_u3phy_exit,
	.power_on	= rockchip_u3phy_power_on,
	.power_off	= rockchip_u3phy_power_off,
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
static void rockchip_u3phy_um_sm_work(struct work_struct *work)
{
	struct rockchip_u3phy_port *u3phy_port =
		container_of(work, struct rockchip_u3phy_port, um_sm_work.work);
	struct rockchip_u3phy *u3phy =
		dev_get_drvdata(u3phy_port->phy->dev.parent);
	unsigned int sh = u3phy->cfgs->grfcfg.um_hstdct.bitend -
			u3phy->cfgs->grfcfg.um_hstdct.bitstart + 1;
	unsigned int ul, uhd, state;
	unsigned int ul_mask, uhd_mask;
	int ret;

	mutex_lock(&u3phy_port->mutex);

	ret = regmap_read(u3phy->u3phy_grf,
			  u3phy->cfgs->grfcfg.um_ls.offset, &ul);
	if (ret < 0)
		goto next_schedule;

	ret = regmap_read(u3phy->u3phy_grf,
			  u3phy->cfgs->grfcfg.um_hstdct.offset, &uhd);
	if (ret < 0)
		goto next_schedule;

	uhd_mask = GENMASK(u3phy->cfgs->grfcfg.um_hstdct.bitend,
			   u3phy->cfgs->grfcfg.um_hstdct.bitstart);
	ul_mask = GENMASK(u3phy->cfgs->grfcfg.um_ls.bitend,
			  u3phy->cfgs->grfcfg.um_ls.bitstart);

	/* stitch on um_ls and um_hstdct as phy state */
	state = ((uhd & uhd_mask) >> u3phy->cfgs->grfcfg.um_hstdct.bitstart) |
		(((ul & ul_mask) >> u3phy->cfgs->grfcfg.um_ls.bitstart) << sh);

	switch (state) {
	case PHY_UTMI_HS_ONLINE:
		dev_dbg(&u3phy_port->phy->dev, "HS online\n");
		break;
	case PHY_UTMI_FS_LS_ONLINE:
		/*
		 * For FS/LS device, the online state share with connect state
		 * from um_ls and um_hstdct register, so we distinguish
		 * them via suspended flag.
		 *
		 * Plus, there are two cases, one is D- Line pull-up, and D+
		 * line pull-down, the state is 4; another is D+ line pull-up,
		 * and D- line pull-down, the state is 2.
		 */
		if (!u3phy_port->suspended) {
			/* D- line pull-up, D+ line pull-down */
			dev_dbg(&u3phy_port->phy->dev, "FS/LS online\n");
			break;
		}
		/* fall through */
	case PHY_UTMI_CONNECT:
		if (u3phy_port->suspended) {
			dev_dbg(&u3phy_port->phy->dev, "Connected\n");
			rockchip_u3phy_power_on(u3phy_port->phy);
			u3phy_port->suspended = false;
		} else {
			/* D+ line pull-up, D- line pull-down */
			dev_dbg(&u3phy_port->phy->dev, "FS/LS online\n");
		}
		break;
	case PHY_UTMI_DISCONNECT:
		if (!u3phy_port->suspended) {
			dev_dbg(&u3phy_port->phy->dev, "Disconnected\n");
			rockchip_u3phy_power_off(u3phy_port->phy);
			u3phy_port->suspended = true;
		}

		/*
		 * activate the linestate detection to get the next device
		 * plug-in irq.
		 */
		param_write(1, u3phy->u3phy_grf,
			    &u3phy->cfgs->grfcfg.ls_det_st, 1);
		param_write(1, u3phy->u3phy_grf,
			    &u3phy->cfgs->grfcfg.ls_det_en, 1);

		/*
		 * we don't need to rearm the delayed work when the phy port
		 * is suspended.
		 */
		mutex_unlock(&u3phy_port->mutex);
		return;
	default:
		dev_dbg(&u3phy_port->phy->dev, "unknown phy state\n");
		break;
	}

next_schedule:
	mutex_unlock(&u3phy_port->mutex);
	schedule_delayed_work(&u3phy_port->um_sm_work, SCHEDULE_DELAY);
}

static irqreturn_t rockchip_u3phy_um_ls_irq(int irq, void *data)
{
	struct rockchip_u3phy_port *u3phy_port = data;
	struct rockchip_u3phy *u3phy =
		dev_get_drvdata(u3phy_port->phy->dev.parent);

	if (!param_desired(1, u3phy->u3phy_grf, &u3phy->cfgs->grfcfg.ls_det_st))
		return IRQ_NONE;

	dev_dbg(u3phy->dev, "utmi linestate interrupt\n");
	mutex_lock(&u3phy_port->mutex);

	/* disable linestate detect irq and clear its status */
	param_write(1, u3phy->u3phy_grf, &u3phy->cfgs->grfcfg.ls_det_en, 0);
	param_write(1, u3phy->u3phy_grf, &u3phy->cfgs->grfcfg.ls_det_st, 1);

	mutex_unlock(&u3phy_port->mutex);

	/*
	 * In this case for host phy, a new device is plugged in, meanwhile,
	 * if the phy port is suspended, we need rearm the work to resume it
	 * and mange its states; otherwise, we just return irq handled.
	 */
	if (u3phy_port->suspended) {
		dev_dbg(u3phy->dev, "schedule utmi sm work\n");
		rockchip_u3phy_um_sm_work(&u3phy_port->um_sm_work.work);
	}

	return IRQ_HANDLED;
}

static int rockchip_u3phy_parse_dt(struct rockchip_u3phy *u3phy,
				   struct platform_device *pdev)

{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int gpio, ret, i, clk;

	u3phy->um_ls_irq = platform_get_irq_byname(pdev, "linestate");
	if (u3phy->um_ls_irq < 0) {
		dev_err(dev, "get utmi linestate irq failed\n");
		return -ENXIO;
	}

	gpio = of_get_named_gpio(np, "usb30-drv", 0);
	if (gpio_is_valid(gpio)) {
		ret = devm_gpio_request(dev, gpio, "usb30_drv_pullup");
		if (ret) {
			dev_err(dev, "can't request pullup gpio %d, err: %d\n",
				gpio, ret);
			return ret;
		}
		gpio_direction_output(gpio, 1);
	}

	for (clk = 0; clk < U3PHY_MAX_CLKS; clk++) {
		u3phy->clks[clk] = of_clk_get(np, clk);
		if (IS_ERR(u3phy->clks[clk])) {
			ret = PTR_ERR(u3phy->clks[clk]);
			if (ret == -EPROBE_DEFER)
				goto err_put_clks;
			u3phy->clks[clk] = NULL;
			break;
		}
	}

	for (i = 0; i < U3PHY_RESET_MAX; i++) {
		u3phy->rsts[i] = devm_reset_control_get(dev, get_rest_name(i));
		if (IS_ERR(u3phy->rsts[i])) {
			dev_info(dev, "no %s reset control specified\n",
				 get_rest_name(i));
			u3phy->rsts[i] = NULL;
		}
	}

	return 0;

err_put_clks:
	while (--clk >= 0)
		clk_put(u3phy->clks[clk]);
	return ret;
}

static int rockchip_u3phy_port_init(struct rockchip_u3phy *u3phy,
				    struct rockchip_u3phy_port *u3phy_port,
				    struct device_node *child_np)
{
	struct resource res;
	struct phy *phy;
	int ret;

	dev_dbg(u3phy->dev, "u3phy port initialize\n");

	mutex_init(&u3phy_port->mutex);

	phy = devm_phy_create(u3phy->dev, child_np, &rockchip_u3phy_ops);
	if (IS_ERR(phy)) {
		dev_err(u3phy->dev, "failed to create phy\n");
		return PTR_ERR(phy);
	}

	u3phy_port->phy = phy;

	ret = of_address_to_resource(child_np, 0, &res);
	if (ret) {
		dev_err(u3phy->dev, "failed to get address resource(np-%s)\n",
			child_np->name);
		return ret;
	}

	u3phy_port->base = devm_ioremap_resource(&u3phy_port->phy->dev, &res);
	if (IS_ERR(u3phy_port->base)) {
		dev_err(u3phy->dev, "failed to remap phy regs\n");
		return PTR_ERR(u3phy_port->base);
	}

	if (!of_node_cmp(child_np->name, "pipe")) {
		u3phy_port->type = U3PHY_TYPE_PIPE;

		if (of_property_read_bool(child_np, "refclk-25m-quirk"))
			u3phy_port->refclk_25m_quirk = true;

	} else {
		u3phy_port->type = U3PHY_TYPE_UTMI;
		INIT_DELAYED_WORK(&u3phy_port->um_sm_work,
				  rockchip_u3phy_um_sm_work);

		ret = devm_request_threaded_irq(u3phy->dev, u3phy->um_ls_irq,
						NULL, rockchip_u3phy_um_ls_irq,
						IRQF_ONESHOT, "rockchip_u3phy",
						u3phy_port);
		if (ret) {
			dev_err(u3phy->dev, "failed to request utmi linestate irq handle\n");
			return ret;
		}
	}

	if (u3phy->cfgs->phy_tuning) {
		dev_dbg(u3phy->dev, "do u3phy tuning\n");
		ret = u3phy->cfgs->phy_tuning(u3phy, u3phy_port);
		if (ret)
			return ret;
	}

	phy_set_drvdata(u3phy_port->phy, u3phy_port);
	return 0;
	/* return rockchip_u3phy_power_off(u3phy_port->phy); */
}

static int rockchip_u3phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *child_np;
	struct phy_provider *provider;
	struct rockchip_u3phy *u3phy;
	const struct rockchip_u3phy_cfg *phy_cfgs;
	const struct of_device_id *match;
	unsigned int reg[2];
	int index, ret;

	match = of_match_device(dev->driver->of_match_table, dev);
	if (!match || !match->data) {
		dev_err(dev, "phy-cfgs are not assigned!\n");
		return -EINVAL;
	}

	u3phy = devm_kzalloc(dev, sizeof(*u3phy), GFP_KERNEL);
	if (!u3phy)
		return -ENOMEM;

	u3phy->u3phy_grf =
		syscon_regmap_lookup_by_phandle(np, "rockchip,u3phygrf");
	if (IS_ERR(u3phy->u3phy_grf))
		return PTR_ERR(u3phy->u3phy_grf);

	if (of_property_read_u32_array(np, "reg", reg, 2)) {
		dev_err(dev, "the reg property is not assigned in %s node\n",
			np->name);
		return -EINVAL;
	}

	u3phy->dev = dev;
	phy_cfgs = match->data;
	platform_set_drvdata(pdev, u3phy);

	/* find out a proper config which can be matched with dt. */
	index = 0;
	while (phy_cfgs[index].reg) {
		if (phy_cfgs[index].reg == reg[1]) {
			u3phy->cfgs = &phy_cfgs[index];
			break;
		}

		++index;
	}

	if (!u3phy->cfgs) {
		dev_err(dev, "no phy-cfgs can be matched with %s node\n",
			np->name);
		return -EINVAL;
	}

	ret = rockchip_u3phy_parse_dt(u3phy, pdev);
	if (ret) {
		dev_err(dev, "parse dt failed, ret(%d)\n", ret);
		return ret;
	}

	rockchip_u3phy_rest_assert(u3phy);
	rockchip_u3phy_rest_deassert(u3phy, U3PHY_APB_RST | U3PHY_POR_RST);

	index = 0;
	for_each_available_child_of_node(np, child_np) {
		struct rockchip_u3phy_port *u3phy_port = &u3phy->ports[index];

		u3phy_port->index = index;
		ret = rockchip_u3phy_port_init(u3phy, u3phy_port, child_np);
		if (ret) {
			dev_err(dev, "u3phy port init failed,ret(%d)\n", ret);
			goto put_child;
		}

		/* to prevent out of boundary */
		if (++index >= U3PHY_PORT_NUM)
			break;
	}

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR_OR_NULL(provider))
		goto put_child;

	ret = rockchip_u3phy_clk_enable(u3phy);
	if (ret)
		goto put_child;

	rockchip_u3phy_rest_deassert(u3phy, U3PHY_MAC_RST);

	dev_info(dev, "Rockchip u3phy initialized successfully\n");
	return 0;

put_child:
	of_node_put(child_np);
	return ret;
}

static int rk322xh_u3phy_tuning(struct rockchip_u3phy *u3phy,
				struct rockchip_u3phy_port *u3phy_port)
{
	if (u3phy_port->type == U3PHY_TYPE_UTMI) {
		/* 3'b111: always enable pre-emphasis */
		writel(0x0f, u3phy_port->base + 0x030);

		/* 3'b000: pre_emphasize strength configure to the weakest */
		writel(0x41, u3phy_port->base + 0x040);
	} else if (u3phy_port->type == U3PHY_TYPE_PIPE) {
		if (u3phy_port->refclk_25m_quirk) {
			dev_dbg(u3phy->dev, "switch to 25m refclk\n");
			/* ref clk switch to 25M */
			writel(0x64, u3phy_port->base + 0x11c);
			writel(0x64, u3phy_port->base + 0x028);
			writel(0x01, u3phy_port->base + 0x020);
			writel(0x21, u3phy_port->base + 0x030);
			writel(0x06, u3phy_port->base + 0x108);
			writel(0x00, u3phy_port->base + 0x118);
		} else {
			/* configure for 24M ref clk */
			writel(0x80, u3phy_port->base + 0x10c);
			writel(0x01, u3phy_port->base + 0x118);
			writel(0x38, u3phy_port->base + 0x11c);
			writel(0x83, u3phy_port->base + 0x020);
			writel(0x02, u3phy_port->base + 0x108);
		}

		/* enable SSC */
		udelay(3);
		writel(0x08, u3phy_port->base + 0x000);
		writel(0x0c, u3phy_port->base + 0x120);
	} else {
		dev_err(u3phy->dev, "invalid u3phy port type\n");
		return -EINVAL;
	}

	return 0;
}

static const struct rockchip_u3phy_cfg rk322xh_u3phy_cfgs[] = {
	{
		.reg		= 0xff470000,
		.grfcfg		= {
			.um_suspend	= { 0x0004, 15, 0, 0x1452, 0x15d1 },
			.um_ls		= { 0x0030, 5, 4, 0, 1 },
			.um_hstdct	= { 0x0030, 7, 7, 0, 1 },
			.ls_det_en	= { 0x0040, 0, 0, 0, 1 },
			.ls_det_st	= { 0x0044, 0, 0, 0, 1 }
		},
		.phy_tuning	= rk322xh_u3phy_tuning,
	},
	{ /* sentinel */ }
};

static const struct of_device_id rockchip_u3phy_dt_match[] = {
	{ .compatible = "rockchip,rk322xh-u3phy", .data = &rk322xh_u3phy_cfgs },
	{}
};
MODULE_DEVICE_TABLE(of, rockchip_u3phy_dt_match);

static struct platform_driver rockchip_u3phy_driver = {
	.probe		= rockchip_u3phy_probe,
	.driver		= {
		.name	= "rockchip-u3phy",
		.of_match_table = rockchip_u3phy_dt_match,
	},
};
module_platform_driver(rockchip_u3phy_driver);

MODULE_AUTHOR("Frank Wang <frank.wang@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip USB3.0 PHY driver");
MODULE_LICENSE("GPL v2");
