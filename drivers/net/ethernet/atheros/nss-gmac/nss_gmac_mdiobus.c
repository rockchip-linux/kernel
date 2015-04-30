/*
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
 * OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */
/*
 * @file
 * IPQ806x MDIO bus support.
 *
 * @note Many of the functions other than the device specific functions
 *  changes for operating system other than Linux 2.6.xx
 *-----------------------------REVISION HISTORY----------------------------------
 * Qualcomm Atheros    		09/Jun/2013			Created
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/phy.h>
#include <linux/device.h>

#if (NSS_GMAC_DT_SUPPORT == 1)
#include <msm_nss_gmac.h>
#else
#include <mach/msm_nss_gmac.h>
#endif

#include <nss_gmac_dev.h>
#include <nss_gmac_network_interface.h>


static int32_t phy_irq[PHY_MAX_ADDR];


/**
 * @brief MDIO bus read
 * @param[in] pointer to struct mii_bus
 * @param[in] Phy MDIO address
 * @param[in] Register number
 * @return Contents of MDIO register
 */
static int32_t nss_gmac_mdiobus_read(struct mii_bus *bus, int32_t phy_id, int32_t regnum)
{
	int32_t status;
	uint16_t data;
	nss_gmac_dev *gmacdev;

	gmacdev = (nss_gmac_dev *)bus->priv;

	status = nss_gmac_read_phy_reg((uint32_t *)gmacdev->mac_base,
					phy_id, regnum,
					&data, gmacdev->mdc_clk_div);

	if (status != 0) {
		data = 0;
	}

	return (int32_t)data;
}


/**
 * @brief MDIO bus write
 * @param[in] pointer to struct mii_bus
 * @param[in] Phy MDIO address
 * @param[in] Register number
 * @param[in] Value to wirte
 * @return 0 on Success
 */
static int32_t nss_gmac_mdiobus_write(struct mii_bus *bus, int32_t phy_id, int32_t regnum, uint16_t val)
{
	nss_gmac_dev *gmacdev;

	gmacdev = (nss_gmac_dev *)bus->priv;

	nss_gmac_write_phy_reg((uint32_t *)gmacdev->mac_base, phy_id,
				regnum, val, gmacdev->mdc_clk_div);

	return 0;
}


/**
 * @brief MDIO bus reset
 * @param[in] pointer to struct mii_bus
 * @return 0 on Success
 */
int32_t nss_gmac_mdiobus_reset(struct mii_bus *bus)
{
	nss_gmac_dev *gmacdev;

	gmacdev = (nss_gmac_dev *)bus->priv;
	gmacdev->mdc_clk_div = MDC_CLK_DIV;
	nss_gmac_info(gmacdev, "%s: GMAC%d MDC Clk div set to - 0x%x",
		      __FUNCTION__, gmacdev->macid, gmacdev->mdc_clk_div);

	return 0;
}


/**
 * @brief Initialize and register MDIO bus
 * @param[in] pointer to nss_gmac_dev
 * @return 0 on Success
 */
int32_t nss_gmac_init_mdiobus(nss_gmac_dev *gmacdev)
{
	struct mii_bus *miibus = NULL;
	struct phy_device *phydev = NULL;

	miibus = mdiobus_alloc();
	if (miibus == NULL)
		return -ENOMEM;

	miibus->name = "nss gmac mdio bus";
	snprintf(miibus->id, MII_BUS_ID_SIZE, "mdiobus%x", gmacdev->macid);

	miibus->priv = (void *)gmacdev;
	miibus->read = nss_gmac_mdiobus_read;
	miibus->write = nss_gmac_mdiobus_write;
	miibus->reset = nss_gmac_mdiobus_reset;
	mutex_init(&(miibus->mdio_lock));
	miibus->parent = &(gmacdev->pdev->dev);

	phy_irq[gmacdev->phy_base] = PHY_POLL;
	miibus->irq = phy_irq;
	miibus->phy_mask = ~((uint32_t)(1 << gmacdev->phy_base));

	if (mdiobus_register(miibus) != 0) {
		mdiobus_free(miibus);
		nss_gmac_info(gmacdev, "%s: mdiobus_reg failed", __FUNCTION__);
		return -EIO;
	}

	phydev = miibus->phy_map[gmacdev->phy_base];
	if (!phydev) {
		nss_gmac_info(gmacdev, "%s: No phy device", __FUNCTION__);
		mdiobus_unregister(miibus);
		mdiobus_free(miibus);
		return -ENODEV;
	}

	switch (gmacdev->phy_mii_type) {
	case GMAC_INTF_RGMII:
		phydev->interface = PHY_INTERFACE_MODE_RGMII;
		break;

	case GMAC_INTF_SGMII:
		phydev->interface = PHY_INTERFACE_MODE_SGMII;
		break;

	case GMAC_INTF_QSGMII:
		phydev->interface = PHY_INTERFACE_MODE_SGMII;
		break;
	}

	gmacdev->miibus = miibus;
	return 0;
}


/**
 * @brief De-initialize MDIO bus
 * @param[in] pointer to nss_gmac_dev
 * @return void
 */
void nss_gmac_deinit_mdiobus(nss_gmac_dev *gmacdev)
{
	mdiobus_unregister(gmacdev->miibus);
	mdiobus_free(gmacdev->miibus);
	gmacdev->miibus = NULL;
}


