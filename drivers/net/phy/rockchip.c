/**
 * drivers/net/phy/rockchip.c
 *
 * Driver for ROCKCHIP Ethernet PHYs
 *
 * Copyright (c) 2017, Fuzhou Rockchip Electronics Co., Ltd
 *
 * David Wu <david.wu@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/ethtool.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/phy.h>

//#define DEBUG
#ifdef DEBUG
#define DBG(fmt, args...)  pr_info("RK_PHY " fmt, ## args)
#else
#define DBG(fmt, args...)  do { } while (0)
#endif

#define INTERNAL_EPHY_ID			0x1234d400

#define MII_INTERNAL_CTRL_STATUS		17
#define SMI_ADDR_TSTCNTL			20
#define SMI_ADDR_TSTREAD1			21
#define SMI_ADDR_TSTREAD2			22
#define SMI_ADDR_TSTWRITE			23
#define MII_SPECIAL_CONTROL_STATUS		31

#define MII_AUTO_MDIX_EN			BIT(7)
#define MII_MDIX_EN				BIT(6)

#define MII_SPEED_10				BIT(2)
#define MII_SPEED_100				BIT(3)

#define TSTCNTL_RD				(BIT(15) | BIT(10))
#define TSTCNTL_WR				(BIT(14) | BIT(10))

#define TSTMODE_ENABLE				0x400
#define TSTMODE_DISABLE				0x0

#define DSP_REG_BANK_SEL			(0 << 11)
#define WOL_REG_BANK_SEL			BIT(11)
#define BIST_REG_BANK_SEL			(3 << 11)

#define WR_ADDR_A3CFG                           0x14
#define WR_ADDR_A7CFG				0x18

#define RD_ADDR_A3CFG				(0x14 << 5)
#define RD_ADDR_LPISTA				(12 << 5)

#define A3CFG_100M				0xFC00
#define A3CFG_TAG				0xFFFF

#define PHY_ABNORMAL_THRESHOLD			15

struct rk_int_phy_priv {
	int restore_reg0;
	int restore_a3_config;
	int force_10m_full_mode;
	int a3_config_set;
	int txrx_counters_done_count;
};

static int rockchip_integrated_phy_init_tstmode(struct phy_device *phydev)
{
	int ret;

	ret = phy_write(phydev, SMI_ADDR_TSTCNTL, TSTMODE_DISABLE);
	if (ret)
		return ret;

	/* Enable access to Analog and DSP register banks */
	ret = phy_write(phydev, SMI_ADDR_TSTCNTL, TSTMODE_ENABLE);
	if (ret)
		return ret;

	ret = phy_write(phydev, SMI_ADDR_TSTCNTL, TSTMODE_DISABLE);
	if (ret)
		return ret;

	return phy_write(phydev, SMI_ADDR_TSTCNTL, TSTMODE_ENABLE);
}

static int rockchip_integrated_phy_analog_init(struct phy_device *phydev)
{
	int ret;

	DBG("%s: %d\n", __func__, __LINE__);

	ret = rockchip_integrated_phy_init_tstmode(phydev);
	if (ret)
		return ret;

	/* Adjust tx amplitude to make sginal better,
	 * the default value is 0x8.
	 */
	phy_write(phydev, SMI_ADDR_TSTWRITE, 0xB);
	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR | WR_ADDR_A7CFG);

	return ret;
}

static int rockchip_integrated_phy_config_init(struct phy_device *phydev)
{
	int val, ret;

	DBG("%s: %d\n", __func__, __LINE__);

	/* The auto MIDX has linked problem on some board,
	 * workround to disable auto MDIX.
	 */
	val = phy_read(phydev, MII_INTERNAL_CTRL_STATUS);
	if (val < 0)
		return val;
	val &= ~MII_AUTO_MDIX_EN;
	ret = phy_write(phydev, MII_INTERNAL_CTRL_STATUS, val);
	if (ret)
		return ret;

	return rockchip_integrated_phy_analog_init(phydev);
}

static int rockchip_integrated_phy_probe(struct phy_device *phydev)
{
	struct rk_int_phy_priv *priv;

	DBG("%s: %d\n", __func__, __LINE__);

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	phydev->priv = priv;

	return 0;
}

static void rockchip_integrated_phy_remove(struct phy_device *phydev)
{
	struct rk_int_phy_priv *priv = phydev->priv;

	DBG("%s: %d\n", __func__, __LINE__);

	kfree(priv);
}

static void rockchip_integrated_phy_adjust_a3_config(struct phy_device *phydev,
						     int value)
{
	struct rk_int_phy_priv *priv = phydev->priv;

	DBG("%s: %d: value = %x\n", __func__, __LINE__, value);

	if (value == A3CFG_TAG) {
		phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_RD | RD_ADDR_A3CFG);
		priv->restore_a3_config = phy_read(phydev, SMI_ADDR_TSTREAD1);
		priv->a3_config_set = 1;

		phy_write(phydev, SMI_ADDR_TSTWRITE, A3CFG_100M);
	} else {
		phy_write(phydev, SMI_ADDR_TSTWRITE, value);
	}

	phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_WR | WR_ADDR_A3CFG);
}

static int rockchip_integrated_phy_reset(struct phy_device *phydev)
{
	int val;
	int timeout = 12;

	DBG("%s: %d\n", __func__, __LINE__);

	/* soft reset */
	phy_write(phydev, MII_BMCR, BMCR_RESET);
	/* wait end of reset (about 600 ms) */
	do {
		msleep(50);
		if (timeout-- == 0) {
			pr_err("%s: timeout\n", __func__);
			return -ETIMEDOUT;
		}
		val = phy_read(phydev, MII_BMCR);
	} while (val & BMCR_RESET);

	rockchip_integrated_phy_analog_init(phydev);
	return 0;
}

static
void rockchip_integrated_phy_link_change_notify(struct phy_device *phydev)
{
	int speed = SPEED_10;

	if (phydev->autoneg == AUTONEG_ENABLE) {
		int reg = phy_read(phydev, MII_SPECIAL_CONTROL_STATUS);

		if (reg < 0) {
			pr_err("phy_read err: %d.\n", reg);
			return;
		}

		if (reg & MII_SPEED_100)
			speed = SPEED_100;
		else if (reg & MII_SPEED_10)
			speed = SPEED_10;
	} else {
		int bmcr = phy_read(phydev, MII_BMCR);

		if (bmcr < 0) {
			pr_err("phy_read err: %d.\n", bmcr);
			return;
		}

		if (bmcr & BMCR_SPEED100)
			speed = SPEED_100;
		else
			speed = SPEED_10;
	}

	/* If mode switch happens from 10BT to 100BT, all DSP/AFE
	 * registers are set to default values. So any AFE/DSP
	 * registers have to be re-initialized in this case.
	 */
	if (phydev->speed == SPEED_10 && speed == SPEED_100) {
		int ret = rockchip_integrated_phy_analog_init(phydev);

		if (ret)
			pr_err("rockchip_integrated_phy_analog_init err: %d.\n",
			       ret);
	}
}

static void rockchip_integrated_phy_fixup_10M(struct phy_device *phydev)
{
	int val;
	struct rk_int_phy_priv *priv = phydev->priv;

	/* link partner does not have auto-negotiation ability */
	/* setting PHY to 10M full-duplex by force */
	if (phydev->state != PHY_AN && !priv->force_10m_full_mode &&
			phydev->link && phydev->speed == SPEED_10) {
		int an_expan = phy_read(phydev, MII_EXPANSION);

		if (!(an_expan & 0x1)) {
			DBG("%s: force_10m_full_mode\n", __func__);
			priv->force_10m_full_mode = 1;
			priv->restore_reg0 = phy_read(phydev, MII_BMCR);
			val = priv->restore_reg0 & (~BMCR_ANENABLE);
			val &= ~BMCR_SPEED100;
			val |= BMCR_FULLDPLX;
			phy_write(phydev, MII_BMCR, val);
		}
	/* restore BMCR register */
	} else if (!phydev->link && priv->force_10m_full_mode) {
		DBG("%s: restore force_10m_full_mode\n", __func__);
		priv->force_10m_full_mode = 0;
		phy_write(phydev, MII_BMCR, priv->restore_reg0);
	}
}

static void rockchip_integrated_phy_fixup_100M(struct phy_device *phydev)
{
	int wol_reg12;
	struct rk_int_phy_priv *priv = phydev->priv;

	/* reset phy when continue detect phy abnormal */
	if (phydev->link && phydev->speed == SPEED_100) {
		/* read wol bank reg12 and check bit 12 */
		phy_write(phydev, SMI_ADDR_TSTCNTL, TSTCNTL_RD |
					WOL_REG_BANK_SEL | RD_ADDR_LPISTA);
		wol_reg12 = phy_read(phydev, SMI_ADDR_TSTREAD1);

		if (wol_reg12 & 0x1000)
			priv->txrx_counters_done_count = 0;
		if (!(wol_reg12 & 0x1000))
			priv->txrx_counters_done_count++;

		/* DBG("txrx_counters_done_count = %d, threshol = %d\n",
		 *		priv->txrx_counters_done_count,
		 *		PHY_ABNORMAL_THRESHOLD);
		 */
		if (priv->txrx_counters_done_count >=
		    PHY_ABNORMAL_THRESHOLD) {
			priv->txrx_counters_done_count = 0;
			rockchip_integrated_phy_reset(phydev);
		}
	} else {
		priv->txrx_counters_done_count = 0;
	}

	/* adjust A3_CONFIG for optimize long cable when 100M link up*/
	if (phydev->link && phydev->speed == SPEED_100 &&
	    !priv->a3_config_set) {
		rockchip_integrated_phy_adjust_a3_config(phydev, A3CFG_TAG);
	/* restore A3_CONFIG when 100M link down */
	} else if (!phydev->link && priv->a3_config_set) {
		priv->a3_config_set = 0;
		rockchip_integrated_phy_adjust_a3_config(
				phydev,
				priv->restore_a3_config);
	}
}

static int rockchip_integrated_phy_read_status(struct phy_device *phydev)
{
	int err;

	err = genphy_read_status(phydev);
	if (err)
		return err;

	rockchip_integrated_phy_link_change_notify(phydev);

	rockchip_integrated_phy_fixup_10M(phydev);

	rockchip_integrated_phy_fixup_100M(phydev);

	return 0;
}

static int rockchip_integrated_phy_resume(struct phy_device *phydev)
{
	struct rk_int_phy_priv *priv = phydev->priv;

	if (phydev->link && phydev->speed == SPEED_100)
		priv->a3_config_set = 0;

	genphy_resume(phydev);

	return rockchip_integrated_phy_config_init(phydev);
}

static struct phy_driver rockchip_phy_driver = {
	.phy_id			= INTERNAL_EPHY_ID,
	.phy_id_mask		= 0xfffffff0,
	.name			= "Rockchip integrated EPHY",
	.features		= PHY_BASIC_FEATURES,
	.flags			= 0,
	.probe			= rockchip_integrated_phy_probe,
	.remove			= rockchip_integrated_phy_remove,
	.config_init		= rockchip_integrated_phy_config_init,
	.config_aneg		= genphy_config_aneg,
	.read_status		= rockchip_integrated_phy_read_status,
	.suspend		= genphy_suspend,
	.resume			= rockchip_integrated_phy_resume,
	.soft_reset		= rockchip_integrated_phy_reset,
};

static int __init rockchip_init(void)
{
	return phy_driver_register(&rockchip_phy_driver);
}

static void __exit rockchip_exit(void)
{
	phy_driver_unregister(&rockchip_phy_driver);
}

module_init(rockchip_init);
module_exit(rockchip_exit);

static struct mdio_device_id __maybe_unused rockchip_phy_tbl[] = {
	{ INTERNAL_EPHY_ID, 0xfffffff0 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, rockchip_phy_tbl);

MODULE_AUTHOR("David Wu <david.wu@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip Ethernet PHY driver");
MODULE_LICENSE("GPL v2");
