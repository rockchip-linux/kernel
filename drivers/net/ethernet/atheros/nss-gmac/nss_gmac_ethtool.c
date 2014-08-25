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
 * This is the network dependent layer to handle ethtool related functionality.
 * This file is tightly coupled to neworking frame work of linux kernel.
 * The functionality carried out in this file should be treated as an
 * example only if the underlying operating system is not Linux.
 *-----------------------------REVISION HISTORY--------------------------------
 * Qualcomm Atheros			01/Mar/2010			Created
 */

#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/phy.h>

#if (NSS_GMAC_DT_SUPPORT == 1)
#include <msm_nss_macsec.h>
#else
#include <mach/msm_nss_macsec.h>
#endif

#include <nss_gmac_dev.h>
#include <nss_gmac_network_interface.h>

#include <nss_api_if.h>

struct nss_gmac_ethtool_stats {
	uint8_t stat_string[ETH_GSTRING_LEN];
	uint32_t stat_offset;
};

#define DRVINFO_LEN		32
#define NSS_GMAC_STAT(m)	offsetof(struct nss_gmac_sync, m)
#define HW_ERR_SIZE		sizeof(uint32_t)

/**
 * @brief Array of strings describing statistics
 */
static const struct nss_gmac_ethtool_stats gmac_gstrings_stats[] = {
	{"rx_bytes", NSS_GMAC_STAT(rx_bytes)},
	{"rx_packets", NSS_GMAC_STAT(rx_packets)},
	{"rx_errors", NSS_GMAC_STAT(rx_errors)},
	{"rx_receive_errors", NSS_GMAC_STAT(rx_receive_errors)},
	{"rx_overflow_errors", NSS_GMAC_STAT(rx_overflow_errors)},
	{"rx_descriptor_errors", NSS_GMAC_STAT(rx_descriptor_errors)},
	{"rx_watchdog_timeout_errors", NSS_GMAC_STAT(rx_watchdog_timeout_errors)},
	{"rx_crc_errors", NSS_GMAC_STAT(rx_crc_errors)},
	{"rx_late_collision_errors", NSS_GMAC_STAT(rx_late_collision_errors)},
	{"rx_dribble_bit_errors", NSS_GMAC_STAT(rx_dribble_bit_errors)},
	{"rx_length_errors", NSS_GMAC_STAT(rx_length_errors)},
	{"rx_ip_header_errors", NSS_GMAC_STAT(rx_ip_header_errors)},
	{"rx_ip_payload_errors", NSS_GMAC_STAT(rx_ip_payload_errors)},
	{"rx_no_buffer_errors", NSS_GMAC_STAT(rx_no_buffer_errors)},
	{"rx_transport_csum_bypassed", NSS_GMAC_STAT(rx_transport_csum_bypassed)},
	{"tx_bytes", NSS_GMAC_STAT(tx_bytes)},
	{"tx_packets", NSS_GMAC_STAT(tx_packets)},
	{"tx_collisions", NSS_GMAC_STAT(tx_collisions)},
	{"tx_errors", NSS_GMAC_STAT(tx_errors)},
	{"tx_jabber_timeout_errors", NSS_GMAC_STAT(tx_jabber_timeout_errors)},
	{"tx_frame_flushed_errors", NSS_GMAC_STAT(tx_frame_flushed_errors)},
	{"tx_loss_of_carrier_errors", NSS_GMAC_STAT(tx_loss_of_carrier_errors)},
	{"tx_no_carrier_errors", NSS_GMAC_STAT(tx_no_carrier_errors)},
	{"tx_late_collision_errors", NSS_GMAC_STAT(tx_late_collision_errors)},
	{"tx_excessive_collision_errors", NSS_GMAC_STAT(tx_excessive_collision_errors)},
	{"tx_excessive_deferral_errors", NSS_GMAC_STAT(tx_excessive_deferral_errors)},
	{"tx_underflow_errors", NSS_GMAC_STAT(tx_underflow_errors)},
	{"tx_ip_header_errors", NSS_GMAC_STAT(tx_ip_header_errors)},
	{"tx_ip_payload_errors", NSS_GMAC_STAT(tx_ip_payload_errors)},
	{"tx_dropped", NSS_GMAC_STAT(tx_dropped)},
	{"rx_missed", NSS_GMAC_STAT(rx_missed)},
	{"fifo_overflows", NSS_GMAC_STAT(fifo_overflows)},
	{"rx_scatter_errors", NSS_GMAC_STAT(rx_scatter_errors)},
	{"pmt_interrupts", NSS_GMAC_STAT(hw_errs[0])},
	{"mmc_interrupts", NSS_GMAC_STAT(hw_errs[0]) + (1 * HW_ERR_SIZE)},
	{"line_interface_interrupts", NSS_GMAC_STAT(hw_errs[0]) + (2 * HW_ERR_SIZE)},
	{"fatal_bus_error_interrupts", NSS_GMAC_STAT(hw_errs[0]) + (3 * HW_ERR_SIZE)},
	{"rx_buffer_unavailable_interrupts", NSS_GMAC_STAT(hw_errs[0]) + (4 * HW_ERR_SIZE)},
	{"rx_process_stopped_interrupts", NSS_GMAC_STAT(hw_errs[0]) + (5 * HW_ERR_SIZE)},
	{"tx_underflow_interrupts", NSS_GMAC_STAT(hw_errs[0]) + (6 * HW_ERR_SIZE)},
	{"rx_overflow_interrupts", NSS_GMAC_STAT(hw_errs[0]) + (7 * HW_ERR_SIZE)},
	{"tx_jabber_timeout_interrutps", NSS_GMAC_STAT(hw_errs[0]) + (8 * HW_ERR_SIZE)},
	{"tx_process_stopped_interrutps", NSS_GMAC_STAT(hw_errs[0]) + (9 * HW_ERR_SIZE)},
	{"gmac_total_ticks", NSS_GMAC_STAT(gmac_total_ticks)},
	{"gmac_worst_case_ticks", NSS_GMAC_STAT(gmac_worst_case_ticks)},
	{"gmac_iterations", NSS_GMAC_STAT(gmac_iterations)},
};

#define NSS_GMAC_STATS_LEN	ARRAY_SIZE(gmac_gstrings_stats)


/*
 * Convert NSS GMAC speed id to ethtool id.
 * @param[in] nss gmac specific speed
 * @return Returns ethtool speed
 */
static int32_t nss_gmac_to_ethtool_speed(int32_t speed)
{
	int32_t ret;

	switch (speed) {
	case SPEED10:
		ret = SPEED_10;
		break;

	case SPEED100:
		ret = SPEED_100;
		break;

	case SPEED1000:
		ret = SPEED_1000;
		break;

	default:
		ret = SPEED_UNKNOWN;
		break;
	}

	return ret;
}

/*
 * Convert NSS GMAC duplex id to ethtool id.
 * @param[in] nss gmac specific duplex value
 * @return Returns ethtool duplex value
 */
static int32_t nss_gmac_to_ethtool_duplex(int32_t duplex)
{
	int32_t ret;

	switch (duplex) {
	case HALFDUPLEX:
		ret = DUPLEX_HALF;
		break;

	case FULLDUPLEX:
		ret = DUPLEX_FULL;
		break;

	default:
		ret = DUPLEX_UNKNOWN;
		break;
	}

	return ret;
}


/**
 * @brief Get number of strings that describe requested objects.
 * @param[in] pointer to struct net_device.
 * @param[in] string set to get
 */
static int32_t nss_gmac_get_strset_count(struct net_device *netdev, int32_t sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return NSS_GMAC_STATS_LEN;
		break;

	default:
		nss_gmac_early_dbg("%s: Invalid string set", __FUNCTION__);
		return -EOPNOTSUPP;
		break;
	}
}


/**
 * @brief Get strings that describe requested objects
 * @param[in] pointer to struct net_device.
 * @param[in] string set to get
 * @param[out] pointer to buffer
 */
static void nss_gmac_get_strings(struct net_device *netdev, uint32_t stringset,
			  uint8_t *data)
{
	uint8_t *p = data;
	uint32_t i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < NSS_GMAC_STATS_LEN; i++) {
			memcpy(p, gmac_gstrings_stats[i].stat_string,
				ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	}
}


/**
 * @brief Get statistics
 * @param[in] pointer to struct net_device.
 * @param[in] string set to get
 * @param[out] pointer to buffer
 */
static void nss_gmac_get_ethtool_stats(struct net_device *netdev,
				struct ethtool_stats *stats, uint64_t *data)
{
	nss_gmac_dev *gmacdev = netdev_priv(netdev);
	int32_t i;
	uint8_t *p = NULL;

	spin_lock_bh(&gmacdev->stats_lock);
	for (i = 0; i < NSS_GMAC_STATS_LEN; i++) {
		p = (uint8_t *)&(gmacdev->nss_stats) + gmac_gstrings_stats[i].stat_offset;
		data[i] = *(uint32_t *)p;
	}
	spin_unlock_bh(&gmacdev->stats_lock);
}


/**
 * @brief Return driver information.
 *	  Note: Fields are 32 bytes in length.
 * @param[in] pointer to struct net_device.
 * @param[out] pointer to struct ethtool_drvinfo
 */
static void nss_gmac_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, nss_gmac_driver_string, DRVINFO_LEN);
	strlcpy(info->version, nss_gmac_driver_version, DRVINFO_LEN);
	strlcpy(info->bus_info, "NSS", ETHTOOL_BUSINFO_LEN);
}


/**
 * @brief Return pause parameters.
 * @param[in] pointer to struct net_device.
 * @param[in] pointer to ethtool_pauseparam structure.
 */
static void nss_gmac_get_pauseparam(struct net_device *netdev,
			     struct ethtool_pauseparam *pause)
{
	nss_gmac_dev *gmacdev = NULL;

	gmacdev = (nss_gmac_dev *)netdev_priv(netdev);
	BUG_ON(gmacdev == NULL);
	BUG_ON(gmacdev->netdev != netdev);

	pause->rx_pause = gmacdev->pause & FLOW_CTRL_RX ? 1 : 0;
	pause->tx_pause = gmacdev->pause & FLOW_CTRL_TX ? 1 : 0;

	pause->autoneg = AUTONEG_ENABLE;
}

/**
 * @brief Return pause parameters.
 * @param[in] pointer to struct net_device.
 * @param[in] pointer to ethtool_pauseparam structure.
 */
static int nss_gmac_set_pauseparam(struct net_device *netdev,
			    struct ethtool_pauseparam *pause)
{
	nss_gmac_dev *gmacdev = NULL;
	struct phy_device *phydev = NULL;

	gmacdev = (nss_gmac_dev *)netdev_priv(netdev);
	BUG_ON(gmacdev == NULL);
	BUG_ON(gmacdev->netdev != netdev);

	/* set flow control settings */
	gmacdev->pause = 0;
	if (pause->rx_pause) {
		gmacdev->pause |= FLOW_CTRL_RX;
	}

	if (pause->tx_pause) {
		gmacdev->pause |= FLOW_CTRL_TX;
	}

	/*
	 * If the link polling for this GMAC is disabled, we do not
	 * attempt to make changes to the PHY settings.
	 */
	if (!test_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags)) {
		return 0;
	}

	phydev = gmacdev->phydev;

	/* Update flow control advertisment */
	phydev->advertising &= ~(ADVERTISED_Pause | ADVERTISED_Asym_Pause);

	if (gmacdev->pause & FLOW_CTRL_RX) {
		phydev->advertising |= (ADVERTISED_Pause | ADVERTISED_Asym_Pause);
	}

	if (gmacdev->pause & FLOW_CTRL_TX) {
		phydev->advertising |= ADVERTISED_Asym_Pause;
	}

	genphy_config_aneg(gmacdev->phydev);

	return 0;
}

/**
 * @brief Restart autonegotiation
 * @param[in] pointer to struct net_device.
 */
static int nss_gmac_nway_reset(struct net_device *netdev)
{
	nss_gmac_dev *gmacdev = NULL;

	gmacdev = (nss_gmac_dev *)netdev_priv(netdev);
	BUG_ON(gmacdev == NULL);

	if (!netif_running(netdev)) {
		return -EAGAIN;
	}

	/*
	 * If the link polling for this GMAC is disabled, we probably
	 * do not have a PHY attached.
	 */
	if (!test_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags)) {
		return -EINVAL;
	}

	if (!test_bit(__NSS_GMAC_AUTONEG, &gmacdev->flags)) {
		return -EINVAL;
	}

	genphy_restart_aneg(gmacdev->phydev);

	return 0;
}



/**
 * @brief Get Wake On Lan settings
 * @param[in] pointer to struct net_device.
 * @param[in] pointer to struct ethtool_wolinfo.
 */
static void nss_gmac_get_wol(struct net_device *netdev, struct ethtool_wolinfo *wol)
{
	wol->supported = 0;
	wol->wolopts = 0;
}

/**
 * @brief Get message level
 * @param[in] pointer to struct net_device.
 */
static uint32_t nss_gmac_get_msglevel(struct net_device *netdev)
{
	return 0;
}

/**
 * @brief Get Settings
 * @param[in] pointer to struct net_device.
 * @param[in] pointer to struct ethtool_cmd.
 */
static int32_t nss_gmac_get_settings(struct net_device *netdev,
			      struct ethtool_cmd *ecmd)
{
	nss_gmac_dev *gmacdev = NULL;
	struct phy_device *phydev = NULL;
	uint16_t phyreg;

	gmacdev = (nss_gmac_dev *)netdev_priv(netdev);
	BUG_ON(gmacdev == NULL);

	/* Populate supported capabilities */
	ecmd->supported = NSS_GMAC_SUPPORTED_FEATURES;

	/*
	 * If the speed/duplex for this GMAC is forced and we are not
	 * polling for link state changes, return the values as specified by
	 * platform. This will be true for GMACs connected to switch, and
	 * interfaces that do not use a PHY.
	 */
	if (!test_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags)) {
		if (gmacdev->forced_speed != SPEED_UNKNOWN) {
			ethtool_cmd_speed_set(ecmd, nss_gmac_to_ethtool_speed(gmacdev->forced_speed));
			ecmd->duplex = nss_gmac_to_ethtool_duplex(gmacdev->forced_duplex);
			ecmd->mdio_support = 0;
			ecmd->lp_advertising = 0;
			return 0;
		} else {
			/* Non-link polled interfaced must have a forced speed/duplex */
			return -EIO;
		}
	}

	phydev = gmacdev->phydev;

	/* update PHY status */
	if (genphy_read_status(phydev) != 0)
		return -EIO;

	/* Populate capabilities advertised by self */
	ecmd->advertising = phydev->advertising;

	ecmd->autoneg = phydev->autoneg;
	ethtool_cmd_speed_set(ecmd, phydev->speed);
	ecmd->duplex = phydev->duplex;

	if (gmacdev->link_state == LINKDOWN) {
		ethtool_cmd_speed_set(ecmd, SPEED_UNKNOWN);
		ecmd->duplex = DUPLEX_UNKNOWN;
	}

	ecmd->port = PORT_TP;
	ecmd->phy_address = gmacdev->phy_base;
	ecmd->transceiver = XCVR_EXTERNAL;

	ecmd->mdio_support = ETH_MDIO_SUPPORTS_C22;

	/* Populate capabilities advertised by link partner */
	phyreg = nss_gmac_mii_rd_reg(gmacdev, gmacdev->phy_base, MII_LPA);
	if (phyreg & LPA_10HALF) {
		ecmd->lp_advertising |= ADVERTISED_10baseT_Half;
	}

	if (phyreg & LPA_10FULL) {
		ecmd->lp_advertising |= ADVERTISED_10baseT_Full;
	}

	if (phyreg & LPA_100HALF) {
		ecmd->lp_advertising |= ADVERTISED_100baseT_Half;
	}

	if (phyreg & LPA_100FULL) {
		ecmd->lp_advertising |= ADVERTISED_100baseT_Full;
	}

	if (phyreg & LPA_PAUSE_CAP) {
		ecmd->lp_advertising |= ADVERTISED_Pause;
	}

	if (phyreg & LPA_PAUSE_ASYM) {
		ecmd->lp_advertising |= ADVERTISED_Asym_Pause;
	}

	phyreg = nss_gmac_mii_rd_reg(gmacdev, gmacdev->phy_base, MII_STAT1000);
	if (phyreg & LPA_1000HALF) {
		ecmd->lp_advertising |= ADVERTISED_1000baseT_Half;
	}

	if (phyreg & LPA_1000FULL) {
		ecmd->lp_advertising |= ADVERTISED_1000baseT_Full;
	}

	return 0;
}

/**
 * @brief Set Settings
 * @param[in] pointer to struct net_device.
 * @param[in] pointer to struct ethtool_cmd.
 */
static int32_t nss_gmac_set_settings(struct net_device *netdev,
			      struct ethtool_cmd *ecmd)
{
	nss_gmac_dev *gmacdev = NULL;
	struct phy_device *phydev = NULL;

	gmacdev = (nss_gmac_dev *)netdev_priv(netdev);
	BUG_ON(gmacdev == NULL);

	/*
	 * If the speed for this GMAC is forced, do not proceed with the
	 * changes below. This would be true for GMACs connected to switch
	 * and interfaces that do not use a PHY.
	 */
	if (gmacdev->forced_speed != SPEED_UNKNOWN) {
		return -EPERM;
	}

	phydev = gmacdev->phydev;

	mutex_lock(&gmacdev->link_mutex);
	nss_gmac_linkdown(gmacdev);
	mutex_unlock(&gmacdev->link_mutex);

	phydev->advertising = ecmd->advertising;
	phydev->autoneg = ecmd->autoneg;

	phydev->speed = ethtool_cmd_speed(ecmd);
	phydev->duplex = ecmd->duplex;

	if (ecmd->autoneg == AUTONEG_ENABLE) {
		test_and_set_bit(__NSS_GMAC_AUTONEG, &gmacdev->flags);
	} else {
		test_and_clear_bit(__NSS_GMAC_AUTONEG, &gmacdev->flags);
	}

	genphy_config_aneg(phydev);

	return 0;
}


/**
 * Ethtool operations
 */
struct ethtool_ops nss_gmac_ethtool_ops = {
	.get_drvinfo = &nss_gmac_get_drvinfo,
	.get_link = &ethtool_op_get_link,
	.get_msglevel = &nss_gmac_get_msglevel,
	.get_pauseparam = &nss_gmac_get_pauseparam,
	.set_pauseparam = &nss_gmac_set_pauseparam,
	.nway_reset = &nss_gmac_nway_reset,
	.get_wol = &nss_gmac_get_wol,
	.get_settings = &nss_gmac_get_settings,
	.set_settings = &nss_gmac_set_settings,
	.get_strings = &nss_gmac_get_strings,
	.get_sset_count = &nss_gmac_get_strset_count,
	.get_ethtool_stats = &nss_gmac_get_ethtool_stats,
};


/**
 * @brief Register ethtool_ops
 * @param[in] pointer to struct net_device
 */
void nss_gmac_ethtool_register(struct net_device *netdev)
{
	netdev->ethtool_ops = &nss_gmac_ethtool_ops;
}

