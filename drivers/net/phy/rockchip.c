/**
 * rockchip.c  - Rockchip RK3228 & RK3328 internal ephy specific glue layer
 *
 * Copyright (C) 2017
 *
 * David Wu<david.wu@rock-chips.com>
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


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/netdevice.h>

static int internal_config_init(struct phy_device *phydev)
{
	int val;
	u32 features;

	/*enable auto mdix*/
	phy_write(phydev, 0x11, 0x0080);

	features = (SUPPORTED_TP | SUPPORTED_MII
			| SUPPORTED_AUI | SUPPORTED_FIBRE |
			SUPPORTED_BNC);

	/* Do we support autonegotiation? */
	val = phy_read(phydev, MII_BMSR);
	if (val < 0)
		return val;

	if (val & BMSR_ANEGCAPABLE)
		features |= SUPPORTED_Autoneg;

	if (val & BMSR_100FULL)
		features |= SUPPORTED_100baseT_Full;
	if (val & BMSR_100HALF)
		features |= SUPPORTED_100baseT_Half;
	if (val & BMSR_10FULL)
		features |= SUPPORTED_10baseT_Full;
	if (val & BMSR_10HALF)
		features |= SUPPORTED_10baseT_Half;

	if (val & BMSR_ESTATEN) {
		val = phy_read(phydev, MII_ESTATUS);
		if (val < 0)
			return val;

		if (val & ESTATUS_1000_TFULL)
			features |= SUPPORTED_1000baseT_Full;
		if (val & ESTATUS_1000_THALF)
			features |= SUPPORTED_1000baseT_Half;
	}

	phydev->supported = features;
	phydev->advertising = features;

	return 0;
}

static struct phy_driver rockchip_phy_driver[] = {
{
	.phy_id 		= 0x1234d400,
	.phy_id_mask	= 0xffffffff,
	.name		= "rockchip internal ephy",
	.features		= 0,
	.soft_reset 	= genphy_soft_reset,
	.config_init	= internal_config_init,
	.config_aneg	= genphy_config_aneg,
	.aneg_done	= genphy_aneg_done,
	.read_status	= genphy_read_status,
	.suspend		= genphy_suspend,
	.resume 		= genphy_resume,
	.driver		= { .owner = THIS_MODULE, },
},
};

module_phy_driver(rockchip_phy_driver);

static struct mdio_device_id __maybe_unused rockchip_phy_tbl[] = {
	{ 0x1234d400, 0xffffffff },
	{ }
};

MODULE_DEVICE_TABLE(mdio, rockchip_phy_tbl);
