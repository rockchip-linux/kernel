/*
 * Copyright (c) 2013 The Linux Foundation. All rights reserved.
 * Copyright (c) 2014, The Chromium OS Authors
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/platform_device.h>

#include "qca8337-lite.h"

#define DRV_VERSION "0.0.1"

struct qca8337_dev {
	struct device *dev;
	struct mii_bus *mii_bus;
};

static inline void split_addr(uint32_t regaddr, uint16_t *r1, uint16_t *r2,
			uint16_t *page)
{
	regaddr >>= 1;
	*r1 = regaddr & 0x1e;

	regaddr >>= 5;
	*r2 = regaddr & 0x7;

	regaddr >>= 3;
	*page = regaddr & 0x3ff;
}

/* from qca-ssdk driver */
uint32_t qca8337_lite_reg_read(struct qca8337_dev *qca8337, int reg)
{
	struct mii_bus *bus = qca8337->mii_bus;
	uint16_t r1, r2, page;
	uint16_t lo, hi;
	int irq_preepmt;

	split_addr((uint32_t) reg, &r1, &r2, &page);

retry:
	irq_preepmt = 0;
	if(!in_interrupt()) {
		mutex_lock(&bus->mdio_lock);
	} else {
		if(mutex_trylock(&bus->mdio_lock) == 0) {
			irq_preepmt = 1;
		}
	}

	bus->write(bus, 0x18, 0, page);
	if(!in_interrupt()) {
		usleep_range(400, 500);
	} else {
		udelay(100);
	}

	lo = bus->read(bus, 0x10 | r2, r1);
	hi = bus->read(bus, 0x10 | r2, r1 + 1);

	mutex_unlock(&bus->mdio_lock);
	if(!in_interrupt()) {
		if(irq_preepmt) {
			goto retry;
		}
	}

	return (hi << 16) | lo;
}

void qca8337_lite_reg_write(struct qca8337_dev *qca8337, int reg, uint32_t val)
{
	struct mii_bus *bus = qca8337->mii_bus;
	uint16_t r1, r2, r3;
	uint16_t lo, hi;
	int irq_preepmt;

	split_addr((uint32_t) reg, &r1, &r2, &r3);
	lo = val & 0xffff;
	hi = (uint16_t) (val >> 16);

retry:
	irq_preepmt = 0;
	if(!in_interrupt()) {
		mutex_lock(&bus->mdio_lock);
	} else {
		if(mutex_trylock(&bus->mdio_lock) == 0) {
			irq_preepmt = 1;
		}
	}

	bus->write(bus, 0x18, 0, r3);
	if(!in_interrupt()) {
		usleep_range(400, 500);
	} else {
		udelay(100);
	}

	bus->write(bus, 0x10 | r2, r1, lo);
	bus->write(bus, 0x10 | r2, r1 + 1, hi);

	mutex_unlock(&bus->mdio_lock);
	if(!in_interrupt()) {
		if(irq_preepmt) {
			goto retry;
		}
	}
}

void qca8337_lite_reset_switch(struct qca8337_dev *qca8337)
{
	uint32_t val;
        int count = 0;

	qca8337_lite_reg_write(qca8337, S17_MASK_CTRL_REG, S17_MASK_CTRL_SOFT_RET);
        for (count = 0; count < 100; count++) {
		udelay(10);

		val = qca8337_lite_reg_read(qca8337, S17_MASK_CTRL_REG);
                if (!(val & S17_MASK_CTRL_SOFT_RET))
			break;
	}
}

void qca8337_lite_reg_init(struct qca8337_dev *qca8337)
{
	qca8337_lite_reg_write(qca8337, S17_P0STATUS_REG,
			(S17_SPEED_1000M | S17_TXMAC_EN |
				S17_RXMAC_EN | S17_TX_FLOW_EN |
				S17_RX_FLOW_EN | S17_DUPLEX_FULL));

	qca8337_lite_reg_write(qca8337, S17_GLOFW_CTRL1_REG,
			(S17_IGMP_JOIN_LEAVE_DPALL |
				S17_BROAD_DPALL |
				S17_MULTI_FLOOD_DPALL |
				S17_UNI_FLOOD_DPALL));

	qca8337_lite_reg_write(qca8337, S17_P5PAD_MODE_REG,
			S17_MAC0_RGMII_RXCLK_DELAY);

	qca8337_lite_reg_write(qca8337, S17_P0PAD_MODE_REG,
			(S17_MAC0_RGMII_EN | S17_MAC0_RGMII_TXCLK_DELAY |
				S17_MAC0_RGMII_RXCLK_DELAY | \
				(0x1 << S17_MAC0_RGMII_TXCLK_SHIFT) |	\
				(0x3 << S17_MAC0_RGMII_RXCLK_SHIFT)));
}

void qca8337_lite_reg_init_lan(struct qca8337_dev *qca8337)
{
	uint32_t val;

	qca8337_lite_reg_write(qca8337, S17_P6STATUS_REG,
			(S17_SPEED_1000M | S17_TXMAC_EN |
				S17_RXMAC_EN |
				S17_DUPLEX_FULL));

	val = qca8337_lite_reg_read(qca8337, S17_P6PAD_MODE_REG);
	qca8337_lite_reg_write(qca8337, S17_P6PAD_MODE_REG,
			(val | S17_MAC6_SGMII_EN));

	val = qca8337_lite_reg_read(qca8337, S17_PWS_REG);
	qca8337_lite_reg_write(qca8337, S17_PWS_REG,
			(val | S17c_PWS_SERDES_ANEG_DISABLE));

	qca8337_lite_reg_write(qca8337, S17_SGMII_CTRL_REG,
			(S17c_SGMII_EN_PLL | S17c_SGMII_EN_RX |
				S17c_SGMII_EN_TX | S17c_SGMII_EN_SD |
				S17c_SGMII_BW_HIGH | S17c_SGMII_SEL_CLK125M |
				S17c_SGMII_TXDR_CTRL_600mV | S17c_SGMII_CDR_BW_8 |
				S17c_SGMII_DIS_AUTO_LPI_25M | S17c_SGMII_MODE_CTRL_SGMII_PHY |
				S17c_SGMII_PAUSE_SG_TX_EN_25M | S17c_SGMII_ASYM_PAUSE_25M |
				S17c_SGMII_PAUSE_25M | S17c_SGMII_HALF_DUPLEX_25M |
				S17c_SGMII_FULL_DUPLEX_25M));
}

void qca8337_lite_vlan_config(struct qca8337_dev *qca8337)
{
	qca8337_lite_reg_write(qca8337, S17_P0LOOKUP_CTRL_REG, 0x00140020);
	qca8337_lite_reg_write(qca8337, S17_P0VLAN_CTRL0_REG, 0x20001);

	qca8337_lite_reg_write(qca8337, S17_P1LOOKUP_CTRL_REG, 0x0014005c);
	qca8337_lite_reg_write(qca8337, S17_P1VLAN_CTRL0_REG, 0x10001);

	qca8337_lite_reg_write(qca8337, S17_P2LOOKUP_CTRL_REG, 0x0014005a);
	qca8337_lite_reg_write(qca8337, S17_P2VLAN_CTRL0_REG, 0x10001);

	qca8337_lite_reg_write(qca8337, S17_P3LOOKUP_CTRL_REG, 0x00140056);
	qca8337_lite_reg_write(qca8337, S17_P3VLAN_CTRL0_REG, 0x10001);

	qca8337_lite_reg_write(qca8337, S17_P4LOOKUP_CTRL_REG, 0x0014004e);
	qca8337_lite_reg_write(qca8337, S17_P4VLAN_CTRL0_REG, 0x10001);

	qca8337_lite_reg_write(qca8337, S17_P5LOOKUP_CTRL_REG, 0x00140001);
	qca8337_lite_reg_write(qca8337, S17_P5VLAN_CTRL0_REG, 0x20001);

	qca8337_lite_reg_write(qca8337, S17_P6LOOKUP_CTRL_REG, 0x0014001e);
	qca8337_lite_reg_write(qca8337, S17_P6VLAN_CTRL0_REG, 0x10001);
}

static int qca8337_get_mii_bus(struct qca8337_dev *qca8337)
{
	struct device_node *np = NULL;
	const __be32 *prop = NULL;
	struct device_node *mdio_node = NULL;
	struct platform_device *mdio_plat = NULL;

	np = of_node_get(qca8337->dev->of_node);

	prop = of_get_property(np, "mdiobus", NULL);
	if (!prop) {
		dev_err(qca8337->dev, "cannot get 'mdiobus' property");
		return -ENODEV;
	}

	mdio_node = of_find_node_by_phandle(be32_to_cpup(prop));
	if (!mdio_node) {
		dev_err(qca8337->dev, "cannot find mdio node by phandle");
		return -ENODEV;
	}

	mdio_plat = of_find_device_by_node(mdio_node);
	if (!mdio_plat) {
		dev_err(qca8337->dev,
			"cannot find platform device from mdio node");
		of_node_put(mdio_node);
		return -ENODEV;
	}
	qca8337->mii_bus = dev_get_drvdata(&mdio_plat->dev);

	return 0;
}

static int qca8337_lite_probe(struct platform_device *pdev)
{
	int ret;
	struct qca8337_dev *qca8337 =
		devm_kzalloc(&pdev->dev, sizeof(*qca8337), GFP_KERNEL);

	if (qca8337 == NULL)
		return -ENOMEM;

	qca8337->dev = &pdev->dev;

	if (!qca8337->dev->of_node) {
		dev_err(qca8337->dev,
			"not instantiated though the device tree");
		return -ENODEV;
	}

	if ((ret = qca8337_get_mii_bus(qca8337)) < 0)
		return ret;

	qca8337_lite_reset_switch(qca8337);
	qca8337_lite_reg_init(qca8337);
	qca8337_lite_reg_init_lan(qca8337);
	qca8337_lite_vlan_config(qca8337);

	dev_info(qca8337->dev, "version %s initialized\n", DRV_VERSION);

	return 0;
}

static struct of_device_id qca8337_lite_dt_ids[] = {
	{ .compatible =  "qca8337-lite" },
	{},
};
MODULE_DEVICE_TABLE(of, qca8337_lite_dt_ids);

static struct platform_driver qca8337_lite_driver = {
	.probe		= qca8337_lite_probe,
	.driver		= {
		.name	= "qca8337-lite",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(qca8337_lite_dt_ids),
	},
};

module_platform_driver(qca8337_lite_driver);

MODULE_AUTHOR("Matthias Kaehlcke <mka@google.com>");
MODULE_DESCRIPTION("Driver for basic initialization of the QCA8337 switch");
MODULE_ALIAS("platform:qca8337-lite");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
