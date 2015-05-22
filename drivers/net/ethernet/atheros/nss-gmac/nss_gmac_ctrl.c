/*
 **************************************************************************
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 **************************************************************************
 */
/*
 * @file
 * This is the network dependent layer to handle network related functionality.
 * This file is tightly coupled to neworking frame work of linux 2.6.xx kernel.
 * The functionality carried out in this file should be treated as an example
 * only if the underlying operating system is not Linux.
 *
 * @note Many of the functions other than the device specific functions
 * changes for operating system other than Linux 2.6.xx
 *----------------------REVISION HISTORY-----------------------------------
 * Qualcomm Atheros     01/Mar/2013			Modified for QCA NSS
 * Ubicom		01/Mar/2010			Modified for Ubicom32
 * Synopsys		01/Aug/2007			Created
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/phy.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <msm_nss_gmac.h>
#else
#include <mach/msm_iomap.h>
#include <mach/msm_nss_gmac.h>
#endif

#include <nss_gmac_dev.h>
#include <nss_gmac_network_interface.h>


#define NSS_GMAC_PHY_FIXUP_UID		0x004D0000
#define NSS_GMAC_PHY_FIXUP_MASK		0xFFFF0000
#define NSS_GMAC_COMMON_DEVICE_NODE	"nss-gmac-common"

/* Prototypes */

/* Global data */
struct nss_gmac_global_ctx ctx;

/**
 * Sample Wake-up frame filter configurations
 */
uint32_t nss_gmac_wakeup_filter_config0[] = {
	0x00000000,	/* For Filter0 CRC is not computed may be it is 0x0000*/
	0x00000000,	/* For Filter1 CRC is not computed may be it is 0x0000*/
	0x00000000,	/* For Filter2 CRC is not computed may be it is 0x0000*/
	0x5F5F5F5F,	/* For Filter3 CRC is based on 0,1,2,3,4,6,8,9,10,11,12,
			   14,16,17,18,19,20,22,24,25,26,27,28,30
			   bytes from offset                                  */
	0x09000000,	/* Filter 0,1,2 are disabled, Filter3 is enabled and
			   filtering applies to only multicast packets        */
	0x1C000000,	/* Filter 0,1,2 (no significance), filter 3 offset is 28
			   bytes from start of Destination MAC address        */
	0x00000000,	/* No significance of CRC for Filter0 and Filter1     */
	0xBDCC0000	/* No significance of CRC for Filter2,
			   Filter3 CRC is 0xBDCC                              */
};

uint32_t nss_gmac_wakeup_filter_config1[] = {
	0x00000000,	/* For Filter0 CRC is not computed may be it is 0x0000*/
	0x00000000,	/* For Filter1 CRC is not computed may be it is 0x0000*/
	0x7A7A7A7A,	/* For Filter2 CRC is based on 1,3,4,5,6,9,11,12,13,14,
			   17,19,20,21,25,27,28,29,30 bytes from offset       */
	0x00000000,	/* For Filter3 CRC is not computed may be it is 0x0000*/
	0x00010000,	/* Filter 0,1,3 are disabled, Filter2 is enabled and
			   filtering applies to only unicast packets          */
	0x00100000,	/* Filter 0,1,3 (no significance), filter 2 offset is 16
			   bytes from start of Destination MAC address        */
	0x00000000,	/* No significance of CRC for Filter0 and Filter1     */
	0x0000A0FE	/* No significance of CRC for Filter3,
			   Filter2 CRC is 0xA0FE                              */
};

uint32_t nss_gmac_wakeup_filter_config2[] = {
	0x00000000,	/* For Filter0 CRC is not computed may be it is 0x0000*/
	0x000000FF,	/* For Filter1 CRC is computed on 0,1,2,3,4,5,6,7
			   bytes from offset                                  */
	0x00000000,	/* For Filter2 CRC is not computed may be it is 0x0000*/
	0x00000000,	/* For Filter3 CRC is not computed may be it is 0x0000*/
	0x00000100,	/* Filter 0,2,3 are disabled, Filter 1 is enabled and
			   filtering applies to only unicast packets          */
	0x0000DF00,	/* Filter 0,2,3 (no significance), filter 1 offset is
			   223 bytes from start of Destination MAC address    */
	0xDB9E0000,	/* No significance of CRC for Filter0,
			   Filter1 CRC is 0xDB9E                              */
	0x00000000	/* No significance of CRC for Filter2 and Filter3     */
};

/**
 * The nss_gmac_wakeup_filter_config3[] is a sample configuration for wake up
 * filter.
 * Filter1 is used here
 * Filter1 offset is programmed to 50 (0x32)
 * Filter1 mask is set to 0x000000FF, indicating First 8 bytes are used by the
 * filter
 * Filter1 CRC= 0x7EED this is the CRC computed on data 0x55 0x55 0x55 0x55 0x55
 *  0x55 0x55 0x55
 *
 * Refer accompanied software DWC_gmac_crc_example.c for CRC16 generation and
 * how to use the same.
 */
uint32_t nss_gmac_wakeup_filter_config3[] = {
	0x00000000,	/* For Filter0 CRC is not computed may be it is 0x0000*/
	0x000000FF,	/* For Filter1 CRC is computed on 0,1,2,3,4,5,6,7
			   bytes from offset                                  */
	0x00000000,	/* For Filter2 CRC is not computed may be it is 0x0000*/
	0x00000000,	/* For Filter3 CRC is not computed may be it is 0x0000*/
	0x00000100,	/* Filter 0,2,3 are disabled, Filter 1 is enabled and
			   filtering applies to only unicast packets          */
	0x00003200,	/* Filter 0,2,3 (no significance), filter 1 offset is 50
			   bytes from start of Destination MAC address        */
	0x7eED0000,	/* No significance of CRC for Filter0,
			   Filter1 CRC is 0x7EED,                             */
	0x00000000	/* No significance of CRC for Filter2 and Filter3     */
};

/**
 * This gives up the receive Descriptor queue in ring or chain mode.
 * This function is tightly coupled to the platform and operating system
 * Once device's Dma is stopped the memory descriptor memory and the buffer
 * memory deallocation, is completely handled by the operating system,
 * this call is kept outside the device driver Api. This function should be
 * treated as an example code to de-allocate the descriptor structures in ring
 * mode or chain mode and network buffer deallocation. This function depends on
 * the device structure for dma-able memory deallocation for both descriptor
 * memory and the network buffer memory under linux.
 * The responsibility of this function is to
 *  - Free the network buffer memory if any.
 *	- Fee the memory allocated for the descriptors.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] pointer to device structure.
 * @param[in] number of descriptor expected in rx descriptor queue.
 * @param[in] whether descriptors to be created in RING mode or CHAIN mode.
 * @return 0 upon success. Error code upon failure.
 * @note No referece should be made to descriptors once this function is called.
 * This function is invoked when the device is closed.
 */
static void nss_gmac_giveup_rx_desc_queue(struct nss_gmac_dev *gmacdev,
				struct device *dev,
				uint32_t desc_mode) __attribute__((unused));
static void nss_gmac_giveup_rx_desc_queue(struct nss_gmac_dev *gmacdev,
				struct device *dev,
				uint32_t desc_mode)
{
	int32_t i;
	uint32_t status;
	dma_addr_t dma_addr1;
	uint32_t length1;
	uint32_t data1;

	for (i = 0; i < gmacdev->rx_desc_count; i++) {
		nss_gmac_get_desc_data(gmacdev->rx_desc + i, &status,
				       &dma_addr1, &length1, &data1);

		if ((length1 != 0) && (data1 != 0)) {
			dma_unmap_single(dev, (dma_addr_t)dma_addr1,
					 length1, DMA_FROM_DEVICE);
			dev_kfree_skb_any((struct sk_buff *)data1);
		}
	}

	dma_free_coherent(dev, (sizeof(struct dma_desc) * gmacdev->rx_desc_count)
			 , gmacdev->rx_desc, gmacdev->rx_desc_dma);

	netdev_dbg(gmacdev->netdev, "Memory allocated %08x for Rx Descriptors (ring) is given back"
						, (uint32_t)gmacdev->rx_desc);

	gmacdev->rx_desc = NULL;
	gmacdev->rx_desc_dma = 0;
}


/**
 * This gives up the transmit Descriptor queue in ring or chain mode.
 * This function is tightly coupled to the platform and operating system
 * Once device's Dma is stopped the memory descriptor memory and the buffer
 * memory deallocation, is completely handled by the operating system, this
 * call is kept outside the device driver Api. This function should be treated
 * as an example code to de-allocate the descriptor structures in ring mode or
 * chain mode and network buffer deallocation. This function depends on the
 * device structure for dma-able memory deallocation for both descriptor memory
 * and the network buffer memory under linux.
 * The responsibility of this function is to
 *  - Free the network buffer memory if any.
 *	- Fee the memory allocated for the descriptors.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] pointer to device structure.
 * @param[in] number of descriptor expected in tx descriptor queue.
 * @param[in] whether descriptors to be created in RING mode or CHAIN mode.
 * @return 0 upon success. Error code upon failure.
 * @note No reference should be made to descriptors once this function is called
 * This function is invoked when the device is closed.
 */
static void nss_gmac_giveup_tx_desc_queue(struct nss_gmac_dev *gmacdev,
				  struct device *dev,
				  uint32_t desc_mode) __attribute__((unused));
static void nss_gmac_giveup_tx_desc_queue(struct nss_gmac_dev *gmacdev,
				  struct device *dev,
				  uint32_t desc_mode)
{
	int32_t i;
	uint32_t status;
	dma_addr_t dma_addr1;
	uint32_t length1;
	uint32_t data1;

	for (i = 0; i < gmacdev->tx_desc_count; i++) {
		nss_gmac_get_desc_data(gmacdev->tx_desc + i, &status,
				       &dma_addr1, &length1, &data1);

		if ((length1 != 0) && (data1 != 0)) {
			dma_unmap_single(dev, (dma_addr_t)dma_addr1, length1,
					 DMA_TO_DEVICE);
			dev_kfree_skb_any((struct sk_buff *)data1);
		}
	}

	dma_free_coherent(dev, (sizeof(struct dma_desc) * gmacdev->tx_desc_count),
			  gmacdev->tx_desc, gmacdev->tx_desc_dma);

	netdev_dbg(gmacdev->netdev, "Memory allocated %08x for Tx Descriptors (ring) is given back"
						, (uint32_t)gmacdev->tx_desc);

	gmacdev->tx_desc = NULL;
	gmacdev->tx_desc_dma = 0;
}


/**
 * @brief Initialize tx/rx descriptors
 * @param[in] pointer to nss_gmac_dev
 * @return void
 */
void nss_gmac_tx_rx_desc_init(struct nss_gmac_dev *gmacdev)
{
	/* Init Tx/Rx descriptor rings */
	nss_gmac_tx_desc_init_ring(gmacdev->tx_desc, gmacdev->tx_desc_count);
	nss_gmac_rx_desc_init_ring(gmacdev->rx_desc, gmacdev->rx_desc_count);

	/* Init Tx/Rx counters in device private structure */
	gmacdev->tx_next = 0;
	gmacdev->tx_busy = 0;
	gmacdev->tx_next_desc = gmacdev->tx_desc;
	gmacdev->tx_busy_desc = gmacdev->tx_desc;
	gmacdev->busy_tx_desc = 0;
	gmacdev->rx_next = 0;
	gmacdev->rx_busy = 0;
	gmacdev->rx_next_desc = gmacdev->rx_desc;
	gmacdev->rx_busy_desc = gmacdev->rx_desc;
	gmacdev->busy_rx_desc = 0;

	/* take Tx/Rx desc ownership */
	nss_gmac_take_desc_ownership_rx(gmacdev);
	nss_gmac_take_desc_ownership_tx(gmacdev);
}


/**
 * @brief Function provides the network interface statistics.
 * Function is registered to linux get_stats() function. This function is
 * called whenever ifconfig (in Linux) asks for networkig statistics
 * (for example "ifconfig eth0").
 * @param[in] pointer to net_device structure.
 * @param[in] pointer to net_device_stats64 structure.
 * @return Returns pointer to net_device_stats64 structure.
 */
struct rtnl_link_stats64 *nss_gmac_get_stats64(struct net_device *netdev,
						struct rtnl_link_stats64 *stats)
{
	struct nss_gmac_dev *gmacdev = NULL;

	gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);
	BUG_ON(gmacdev == NULL);

	spin_lock_bh(&gmacdev->stats_lock);
	memcpy(stats, &gmacdev->stats, sizeof(*stats));
	spin_unlock_bh(&gmacdev->stats_lock);

	return stats;
}


/**
 * @brief Function to set ethernet address of the NIC.
 * @param[in] pointer to net_device structure.
 * @param[in] pointer to an address structure.
 * @return Returns 0 on success Error code on failure.
 */
static int32_t nss_gmac_set_mac_address(struct net_device *netdev,
					      void *macaddr)
{
	struct nss_gmac_dev *gmacdev = NULL;
	struct sockaddr *addr = (struct sockaddr *)macaddr;

	gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);
	BUG_ON(gmacdev == NULL);
	BUG_ON(gmacdev->netdev != netdev);

	netdev_dbg(netdev, "%s: AddrFamily: %d, %0x:%0x:%0x:%0x:%0x:%0x",
		      __func__, addr->sa_family, addr->sa_data[0],
		      addr->sa_data[1], addr->sa_data[2], addr->sa_data[3],
		      addr->sa_data[4], addr->sa_data[5]);

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	nss_gmac_set_mac_addr(gmacdev, gmac_addr0_high, gmac_addr0_low,
			      addr->sa_data);
	nss_gmac_get_mac_addr(gmacdev, gmac_addr0_high, gmac_addr0_low,
			      netdev->dev_addr);

	return 0;
}


/**
 * @brief IOCTL interface.
 * This function is mainly for debugging purpose.
 * This provides hooks for Register read write, Retrieve descriptor status
 * and Retreiving Device structure information.
 * @param[in] pointer to net_device structure.
 * @param[in] pointer to ifreq structure.
 * @param[in] ioctl command.
 * @return Returns 0 on success Error code on failure.
 */
static int32_t nss_gmac_do_ioctl(struct net_device *netdev,
				       struct ifreq *ifr, int32_t cmd)
{
	int32_t retval;
	struct nss_gmac_dev *gmacdev = NULL;
	struct mii_ioctl_data *mii_data = if_mii(ifr);

	struct ifr_data_struct {
		uint32_t unit;
		uint32_t addr;
		uint32_t data;
	} *req;

	if (netdev == NULL)
		return -EINVAL;
	if (ifr == NULL)
		return -EINVAL;

	req = (struct ifr_data_struct *)ifr->ifr_data;

	gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);
	BUG_ON(gmacdev == NULL);
	BUG_ON(gmacdev->netdev != netdev);

	netdev_dbg(netdev, "%s :: on device %s req->unit = %08x req->addr = %08x req->data = %08x cmd = %08x"
		, __func__, netdev->name, req->unit, req->addr, req->data, cmd);

	retval = 0;
	switch (cmd) {
	case SIOCGMIIPHY:
		netdev_dbg(netdev, "SIOCGMIIPHY: ");
		mii_data->phy_id = gmacdev->phy_base;
		break;

	case SIOCSMIIREG:
		netdev_dbg(netdev, "SIOCSMIIREG: ");
		nss_gmac_mii_wr_reg(gmacdev, gmacdev->phy_base,
			      (mii_data->reg_num & 0x1F), mii_data->val_in);
		break;

	case SIOCGMIIREG:
		netdev_dbg(netdev, "SIOCGMIIREG: ");
		mii_data->val_out = nss_gmac_mii_rd_reg(gmacdev,
						 gmacdev->phy_base,
						 (mii_data->reg_num & 0x1F));
		break;

	default:
		retval = -EINVAL;
		netdev_dbg(netdev, "Unsupported ioctl");
		break;
	}

	return retval;
}


/**
 * The set_rx_mode entry point is called whenever the unicast or multicast
 * address lists or the network interface flags are updated. This routine is
 * responsible for configuring the hardware for proper unicast, multicast,
 * promiscuous mode, and all-multi behavior.
 * @param[in] pointer to net_device structure.
 * @return Returns void.
 */
static void nss_gmac_set_rx_mode(struct net_device *netdev)
{
	struct nss_gmac_dev *gmacdev = NULL;

	gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);
	BUG_ON(gmacdev == NULL);

	netdev_dbg(netdev, "%s: flags - 0x%x", __func__, netdev->flags);

	/* Check for Promiscuous and All Multicast modes */
	if (netdev->flags & IFF_PROMISC) {
		nss_gmac_promisc_enable(gmacdev);
	} else {
		nss_gmac_promisc_disable(gmacdev);

		if (netdev->flags & IFF_ALLMULTI)
			nss_gmac_multicast_enable(gmacdev);
		else
			nss_gmac_multicast_disable(gmacdev);
	}
}


/**
 * @brief Enable/Disable the requested features.
 * @param[in] pointer to net_device structure.
 * @param[in] net_device features
 * @return Returns 0 on success Error code on failure.
 */
static int32_t nss_gmac_set_features(struct net_device *netdev,
					       netdev_features_t features)
{
	struct nss_gmac_dev *gmacdev = NULL;
	netdev_features_t changed;

	gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);
	BUG_ON(gmacdev == NULL);

	changed = features ^ netdev->features;
	if (!(changed & (NETIF_F_RXCSUM | NETIF_F_HW_CSUM | NETIF_F_GRO)))
		return 0;

	if (changed & NETIF_F_RXCSUM) {
		if (features & NETIF_F_RXCSUM)
			set_bit(__NSS_GMAC_RXCSUM, &gmacdev->flags);
		else
			clear_bit(__NSS_GMAC_RXCSUM, &gmacdev->flags);
		nss_gmac_ipc_offload_init(gmacdev);
	}

	if (changed & NETIF_F_GRO) {
		if (!(features & NETIF_F_GRO)) {
			napi_gro_flush(&gmacdev->napi, false);
		}
	}

	return 0;
}


/**
 * Netdevice operations
 */
static const struct net_device_ops nss_gmac_netdev_ops = {
	.ndo_open = &nss_gmac_open,
	.ndo_stop = &nss_gmac_close,
	.ndo_start_xmit = &nss_gmac_xmit_frames,
	.ndo_get_stats64 = &nss_gmac_get_stats64,
	.ndo_set_mac_address = &nss_gmac_set_mac_address,
	.ndo_validate_addr = &eth_validate_addr,
	.ndo_change_mtu = &nss_gmac_change_mtu,
	.ndo_do_ioctl = &nss_gmac_do_ioctl,
	.ndo_tx_timeout = &nss_gmac_tx_timeout,
	.ndo_set_rx_mode = &nss_gmac_set_rx_mode,
	.ndo_set_features = &nss_gmac_set_features,
};

/**
 * @brief Update list of supported, advertised features
 * @param[in] pointer to supported features
 * @param[in] pointer to advertised features
 * @return void
 */
static void nss_gmac_update_features(uint32_t *supp, uint32_t *adv)
{
	*supp &= NSS_GMAC_SUPPORTED_FEATURES;
	*adv &= NSS_GMAC_ADVERTISED_FEATURES;
}


/**
 * @brief PHY fixup
 * @param[in] pointer to PHY device
 * @return 0 on Success
 */
static int32_t nss_gmac_phy_fixup(struct phy_device *phydev)
{
	int32_t ret = 0;

	/* Disable QCA Smart 802.3az in PHY */
	if (nss_gmac_ath_phy_disable_smart_802az(phydev) != 0)
		ret = -EFAULT;

	/* Disable IEEE 802.3az in PHY */
	if (nss_gmac_ath_phy_disable_802az(phydev) != 0)
		ret = -EFAULT;

	return ret;
}

#ifdef CONFIG_OF
/**
 * @brief Get device data via device tree node
 * @param[in] np pointer to device tree node
 * @param[in] netdev pointer to net_device
 * @param[out] netdev pointer to gmac configuration data
 * @return 0 on Success
 */
static int32_t nss_gmac_of_get_pdata(struct device_node *np,
				     struct net_device *netdev,
				     struct msm_nss_gmac_platform_data *gmaccfg)
{
	uint8_t *maddr = NULL;
	struct nss_gmac_dev *gmacdev = NULL;
	struct resource memres_devtree = {0};

	gmacdev = netdev_priv(netdev);

	if (of_property_read_u32(np, "qcom,id", &gmacdev->macid)
		|| of_property_read_u32(np, "qcom,phy_mii_type", &gmaccfg->phy_mii_type)
		|| of_property_read_u32(np, "qcom,phy_mdio_addr", &gmaccfg->phy_mdio_addr)
		|| of_property_read_u32(np, "qcom,rgmii_delay", &gmaccfg->rgmii_delay)
		|| of_property_read_u32(np, "qcom,poll_required", &gmaccfg->poll_required)
		|| of_property_read_u32(np, "qcom,forced_speed", &gmaccfg->forced_speed)
		|| of_property_read_u32(np, "qcom,forced_duplex", &gmaccfg->forced_duplex)
		|| of_property_read_u32(np, "qcom,irq", &netdev->irq)
		|| of_property_read_u32(np, "qcom,socver", &gmaccfg->socver)) {
		pr_err("%s: error reading critical device node properties\n", np->name);
		return -EFAULT;
	}

	maddr = (uint8_t *)of_get_mac_address(np);
	if (maddr)
		memcpy(gmaccfg->mac_addr, maddr, ETH_ALEN);
	else
		memset(gmaccfg->mac_addr, 0, ETH_ALEN);

	if (of_address_to_resource(np, 0, &memres_devtree) != 0)
		return -EFAULT;

	netdev->base_addr = memres_devtree.start;

	return 0;
}
#endif

/**
 * @brief Do GMAC driver common initialization.
 * @param[in] pdev pointer to platform_device
 * @return 0 on Success
 */
static int32_t nss_gmac_do_common_init(struct platform_device *pdev)
{
	int32_t ret = -EFAULT;
	struct resource res_nss_base = {0};
	struct resource res_qsgmii_base = {0};
	struct resource res_clk_ctl_base = {0};

#ifdef CONFIG_OF
	struct device_node *common_device_node = NULL;
	/*
	 * Device tree based common init.
	 */
	common_device_node = of_find_node_by_name(NULL, NSS_GMAC_COMMON_DEVICE_NODE);
	if (!common_device_node) {
		pr_info("Cannot find device tree node "NSS_GMAC_COMMON_DEVICE_NODE);
		ret = -EFAULT;
		goto nss_gmac_cmn_init_fail;
	}
	if (of_address_to_resource(common_device_node, 0, &res_nss_base) != 0) {
		ret = -EFAULT;
		goto nss_gmac_cmn_init_fail;
	}
	if (of_address_to_resource(common_device_node, 1, &res_qsgmii_base) != 0) {
		ret = -EFAULT;
		goto nss_gmac_cmn_init_fail;
	}
	if (of_address_to_resource(common_device_node, 2, &res_clk_ctl_base) != 0) {
		ret = -EFAULT;
		goto nss_gmac_cmn_init_fail;
	}
#else
	res_nss_base.start = NSS_REG_BASE;
	res_nss_base.end = NSS_REG_BASE + NSS_REG_LEN - 1;
	res_nss_base.flags = IORESOURCE_MEM;

	res_qsgmii_base.start = QSGMII_REG_BASE;
	res_qsgmii_base.end =  QSGMII_REG_BASE + QSGMII_REG_LEN - 1;
	res_qsgmii_base.flags = IORESOURCE_MEM;

	res_clk_ctl_base.start = IPQ806X_CLK_CTL_PHYS;
	res_clk_ctl_base.end = IPQ806X_CLK_CTL_PHYS + IPQ806X_CLK_CTL_SIZE - 1;
	res_clk_ctl_base.flags = IORESOURCE_MEM;
#endif

	ctx.nss_base = (uint8_t *)ioremap_nocache(res_nss_base.start,
						  resource_size(&res_nss_base));
	if (!ctx.nss_base) {
		pr_info("Error mapping NSS GMAC registers");
		ret = -EIO;
		goto nss_gmac_cmn_init_fail;
	}
	pr_debug("%s: NSS base ioremap OK.", __func__);

	ctx.qsgmii_base = (uint32_t *)ioremap_nocache(res_qsgmii_base.start,
					      resource_size(&res_qsgmii_base));
	if (!ctx.qsgmii_base) {
		pr_info("Error mapping QSGMII registers");
		ret = -EIO;
		goto nss_gmac_qsgmii_map_err;
	}
	pr_debug("%s: QSGMII base ioremap OK, vaddr = 0x%p",
						__func__, ctx.qsgmii_base);

	ctx.clk_ctl_base = (uint32_t *)ioremap_nocache(res_clk_ctl_base.start,
				       resource_size(&res_clk_ctl_base));
	if (!ctx.clk_ctl_base) {
		pr_info("Error mapping Clk control registers");
		ret = -EIO;
		goto nss_gmac_clkctl_map_err;
	}
	pr_debug("%s: Clk control base ioremap OK, vaddr = 0x%p", __func__,
							ctx.clk_ctl_base);

	if (nss_gmac_common_init(&ctx) == 0) {
		ret = 0;
		ctx.common_init_done = true;
		goto nss_gmac_cmn_init_ok;
	}

nss_gmac_clkctl_map_err:
	iounmap(ctx.qsgmii_base);
	ctx.qsgmii_base = NULL;

nss_gmac_qsgmii_map_err:
	iounmap(ctx.nss_base);
	ctx.nss_base = NULL;

nss_gmac_cmn_init_fail:
	pr_info("%s: platform init fail\n", __func__);

nss_gmac_cmn_init_ok:
#ifdef CONFIG_OF
	if (common_device_node) {
		of_node_put(common_device_node);
		common_device_node = NULL;
	}
#endif
	return ret;
}


/**
 * @brief Function to initialize the Linux network interface.
 * Linux dependent Network interface is setup here. This provides
 * an example to handle the network dependent functionality.
 * @param[in] pointer to struct platform_device
 * @return Returns 0 on success and Error code on failure.
 */
static int32_t nss_gmac_probe(struct platform_device *pdev)
{
	struct net_device *netdev = NULL;
	struct msm_nss_gmac_platform_data *gmaccfg = NULL;
	struct nss_gmac_dev *gmacdev = NULL;
	int32_t ret = 0;
	phy_interface_t phyif = 0;
	uint8_t phy_id[MII_BUS_ID_SIZE + 3];
#ifdef CONFIG_OF
	struct msm_nss_gmac_platform_data gmaccfg_devicetree;
	struct device_node *np = NULL;
	const __be32 *prop = NULL;
	struct device_node *mdio_node = NULL;
	struct platform_device *mdio_plat = NULL;
#else
	struct device *miidev;
	uint8_t busid[MII_BUS_ID_SIZE];
#endif

	if (ctx.common_init_done == false) {
		ret = nss_gmac_do_common_init(pdev);
		if (ret != 0)
			return ret;
	}

	/*
	 * Lets allocate and set up an ethernet device, it takes the sizeof
	 * the private structure. This is mandatory as a 32 byte allignment
	 * is required for the private data structure.
	 */
	netdev = alloc_etherdev(sizeof(struct nss_gmac_dev));
	if (!netdev) {
		pr_info("%s: alloc_etherdev() failed", __func__);
		return -ENOMEM;
	}

	gmacdev = netdev_priv(netdev);
	memset((void *)gmacdev, 0, sizeof(struct nss_gmac_dev));
	SET_NETDEV_DEV(netdev, &pdev->dev);

	spin_lock_init(&gmacdev->stats_lock);
	spin_lock_init(&gmacdev->slock);
	mutex_init(&gmacdev->link_mutex);

	gmacdev->pdev = pdev;
	gmacdev->netdev = netdev;
	gmacdev->loop_back_mode = NOLOOPBACK;

#ifdef CONFIG_OF
	np = of_node_get(pdev->dev.of_node);
	ret = nss_gmac_of_get_pdata(np, netdev, &gmaccfg_devicetree);
	if (ret != 0) {
		free_netdev(netdev);
		return ret;
	}

	gmaccfg = &gmaccfg_devicetree;
#else
	gmaccfg = (struct msm_nss_gmac_platform_data *)((pdev->dev).platform_data);

	netdev->base_addr = pdev->resource[0].start;
	netdev->irq = pdev->resource[1].start;
	gmacdev->macid = pdev->id;
#endif
	gmacdev->phy_mii_type = gmaccfg->phy_mii_type;
	gmacdev->phy_base = gmaccfg->phy_mdio_addr;
	gmacdev->rgmii_delay = gmaccfg->rgmii_delay;

	if (ctx.socver == 0)
		ctx.socver = gmaccfg->socver;

	if (gmaccfg->poll_required) {
		set_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags);
		gmacdev->drv_flags |= NSS_GMAC_PRIV_FLAG(LINKPOLL);
	}

	switch (gmaccfg->forced_speed) {
	case SPEED_10:
		gmacdev->forced_speed = SPEED10;
		break;

	case SPEED_100:
		gmacdev->forced_speed = SPEED100;
		break;

	case SPEED_1000:
		gmacdev->forced_speed = SPEED1000;
		break;

	default:
		gmacdev->forced_speed = SPEED_UNKNOWN;
		break;
	}

	switch (gmaccfg->forced_duplex) {
	case DUPLEX_HALF:
		gmacdev->forced_duplex = HALFDUPLEX;
		break;

	case DUPLEX_FULL:
		gmacdev->forced_duplex = FULLDUPLEX;
		break;

	default:
		gmacdev->forced_duplex = DUPLEX_UNKNOWN;
		break;
	}

	/* save global context within each GMAC context */
	gmacdev->ctx = &ctx;

	ctx.nss_gmac[gmacdev->macid] = gmacdev;

	/* Init for individual GMACs */
	nss_gmac_dev_init(gmacdev);

	if (nss_gmac_attach(gmacdev, netdev->base_addr,
			    pdev->resource[0].end - pdev->resource[0].start +
			    1) < 0) {
		netdev_dbg(netdev, "attach failed for %s", netdev->name);
		ret = -EIO;
		goto nss_gmac_attach_fail;
	}

#ifdef CONFIG_OF
	prop = of_get_property(np, "mdiobus", NULL);
	if (!prop) {
		netdev_dbg(netdev, "cannot get 'mdiobus' property\n");
		ret = -EIO;
		goto mdiobus_init_fail;
	}

	mdio_node = of_find_node_by_phandle(be32_to_cpup(prop));
	if (!mdio_node) {
		netdev_dbg(netdev, "cannot find mdio node by phandle\n");
		ret = -EIO;
		goto mdiobus_init_fail;
	}

	mdio_plat = of_find_device_by_node(mdio_node);
	if (!mdio_plat) {
		netdev_dbg(netdev, "cannot find platform device from mdio node\n");
		of_node_put(mdio_node);
		ret = -EIO;
		goto mdiobus_init_fail;
	}

	gmacdev->miibus = dev_get_drvdata(&mdio_plat->dev);
	if (!gmacdev->miibus) {
		netdev_dbg(netdev, "cannot get mii bus reference from device data\n");
		of_node_put(mdio_node);
		ret = -EIO;
		goto mdiobus_init_fail;
	}
#else
	snprintf(busid, MII_BUS_ID_SIZE, "%s.%d", IPQ806X_MDIO_BUS_NAME,
							IPQ806X_MDIO_BUS_NUM);

	miidev = bus_find_device_by_name(&platform_bus_type, NULL, busid);
	if (!miidev) {
		netdev_dbg(netdev, "mdio bus '%s' get FAIL.\n", busid);
		ret = -EIO;
		goto mdiobus_init_fail;
	}

	gmacdev->miibus = dev_get_drvdata(miidev);
	if (!gmacdev->miibus) {
		netdev_dbg(netdev, "mdio bus '%s' get FAIL.\n", busid);
		ret = -EIO;
		goto mdiobus_init_fail;
	}
#endif

	netdev_dbg(netdev, "mdio bus '%s' OK.\n", gmacdev->miibus->id);

	/*
	 * This just fill in some default MAC address
	 */
	if (is_valid_ether_addr(gmaccfg->mac_addr)) {
		memcpy(netdev->dev_addr, &gmaccfg->mac_addr, ETH_ALEN);
	} else {
		random_ether_addr(netdev->dev_addr);
		pr_info("GMAC%d(%p) Invalid MAC@ - using %02x:%02x:%02x:%02x:%02x:%02x",
			gmacdev->macid, gmacdev,
			netdev->dev_addr[0], netdev->dev_addr[1],
			netdev->dev_addr[2], netdev->dev_addr[3],
			netdev->dev_addr[4], netdev->dev_addr[5]);
	}

	netdev->watchdog_timeo = 5 * HZ;
	netdev->netdev_ops = &nss_gmac_netdev_ops;
	nss_gmac_ethtool_register(netdev);

	/* Initialize workqueue */
	INIT_DELAYED_WORK(&gmacdev->gmacwork, nss_gmac_open_work);

	switch (gmacdev->phy_mii_type) {
	case GMAC_INTF_RGMII:
		phyif = PHY_INTERFACE_MODE_RGMII;
		break;

	case GMAC_INTF_SGMII:
		phyif = PHY_INTERFACE_MODE_SGMII;
		break;

	case GMAC_INTF_QSGMII:
		phyif = PHY_INTERFACE_MODE_SGMII;
		break;
	}

	/* create a phyid using MDIO bus id and MDIO bus address of phy */
	snprintf(phy_id, MII_BUS_ID_SIZE + 3, PHY_ID_FMT,
		 gmacdev->miibus->id, gmacdev->phy_base);

	/* register PHY fixup */
	if (gmacdev->phy_base != NSS_GMAC_NO_MDIO_PHY) {
		ret = phy_register_fixup((const char *)phy_id,
				NSS_GMAC_PHY_FIXUP_UID,
				NSS_GMAC_PHY_FIXUP_MASK,
				&nss_gmac_phy_fixup);
		if (ret	!= 0) {
			netdev_dbg(netdev, "PHY fixup register Error.");
			goto nss_gmac_phy_attach_fail;
		}
	}

	/* connect PHY */
	if (test_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags)) {
		gmacdev->phydev = phy_connect(netdev, (const char *)phy_id,
					      &nss_gmac_adjust_link, phyif);

		if (IS_ERR(gmacdev->phydev)) {
			netdev_dbg(netdev, "PHY %s attach FAIL", phy_id);
			ret = -EIO;
			goto nss_gmac_phy_attach_fail;
		}

		nss_gmac_update_features(&(gmacdev->phydev->supported),
					&(gmacdev->phydev->advertising));
		gmacdev->phydev->irq = PHY_POLL;
		netdev_dbg(netdev, "PHY %s attach OK", phy_id);

		/* reset corresponding Phy */
		nss_gmac_reset_phy(gmacdev, gmacdev->phy_base);

		if (gmacdev->phy_mii_type == GMAC_INTF_RGMII) {
			/* RGMII Tx delay */
			netdev_dbg(netdev, "%s: Program RGMII Tx delay..... ", __func__);
			mdiobus_write(gmacdev->miibus, gmacdev->phy_base, 0x1D, 0x05);
			mdiobus_write(gmacdev->miibus, gmacdev->phy_base, 0x1E, 0x100);
			mdiobus_write(gmacdev->miibus, gmacdev->phy_base, 0x1D, 0x0B);
			mdiobus_write(gmacdev->miibus, gmacdev->phy_base, 0x1E,
				(0xBC00 | NSS_GMAC_PHY_RGMII_TX_DELAY(gmacdev->rgmii_delay)));
		}

		/* XXX: Test code to verify if MDIO access is OK. Remove after
		 * bringup. */
		netdev_dbg(netdev, "%s MII_PHYSID1 - 0x%04x", netdev->name,
		      nss_gmac_mii_rd_reg(gmacdev, gmacdev->phy_base, MII_PHYSID1));
		netdev_dbg(netdev, "%s MII_PHYSID2 - 0x%04x", netdev->name,
		      nss_gmac_mii_rd_reg(gmacdev, gmacdev->phy_base, MII_PHYSID2));
	} else if (gmacdev->phy_base != NSS_GMAC_NO_MDIO_PHY) {
		/*
		 * Issue a phy_attach for the interface connected to a switch
		 */
		gmacdev->phydev = phy_attach(netdev,
						(const char *)phy_id, phyif);
		if (IS_ERR(gmacdev->phydev)) {
			netdev_dbg(netdev, "PHY %s attach FAIL", phy_id);
			ret = -EIO;
			goto nss_gmac_phy_attach_fail;
		}
	} else {
		/* no phy was connected or attached */
		gmacdev->phydev = ERR_PTR(-ENODEV);
	}

	set_bit(__NSS_GMAC_RXCSUM, &gmacdev->flags);
	nss_gmac_ipc_offload_init(gmacdev);

	/* Register the network interface */
	if (register_netdev(netdev)) {
		netdev_dbg(netdev, "Error registering netdevice %s",
			      netdev->name);
		ret = -EFAULT;
		goto nss_gmac_reg_fail;
	}

	/* GRO disabled by default */
	rtnl_lock();
	netdev->features &= ~NETIF_F_GRO;
	netdev->wanted_features &= ~NETIF_F_GRO;
	netdev_change_features(netdev);
	rtnl_unlock();

	netdev_dbg(netdev, "Initialized NSS GMAC%d interface %s: (base = 0x%lx, irq = %d, PhyId = %d, PollLink = %d)"
			, gmacdev->macid, netdev->name, netdev->base_addr
			, netdev->irq, gmacdev->phy_base
			, test_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags));

#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		of_node_put(np);
		np = NULL;
	}
#endif
	return 0;

nss_gmac_reg_fail:
	unregister_netdev(gmacdev->netdev);

	if (!IS_ERR(gmacdev->phydev)) {
		phy_disconnect(gmacdev->phydev);
		gmacdev->phydev = NULL;
	}

nss_gmac_phy_attach_fail:

mdiobus_init_fail:
	nss_gmac_detach(gmacdev);

nss_gmac_attach_fail:
	free_netdev(netdev);

#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		of_node_put(np);
		np = NULL;
	}
#endif
	return ret;
}


/**
 * @brief Remove Linux dependent Network interface.
 * @param[in] pointer to struct platform_device
 * @return Returns 0 on success and Error code on failure.
 */
static int nss_gmac_remove(struct platform_device *pdev)
{
	struct net_device *netdev = NULL;
	struct nss_gmac_dev *gmacdev = NULL;

	gmacdev = ctx.nss_gmac[pdev->id];
	if (!gmacdev) {
		pr_info("Invalid GMAC");
		return -EINVAL;
	}

	netdev = gmacdev->netdev;

	if (!IS_ERR(gmacdev->phydev)) {
		phy_disconnect(gmacdev->phydev);
		gmacdev->phydev = NULL;
	}

	nss_gmac_detach(gmacdev);
	unregister_netdev(gmacdev->netdev);
	free_netdev(gmacdev->netdev);
	ctx.nss_gmac[pdev->id] = NULL;

	return 0;
}

static struct of_device_id nss_gmac_dt_ids[] = {
	{ .compatible =  "qcom,nss-gmac0" },
	{ .compatible =  "qcom,nss-gmac1" },
	{ .compatible =  "qcom,nss-gmac2" },
	{ .compatible =  "qcom,nss-gmac3" },
	{},
};
MODULE_DEVICE_TABLE(of, nss_gmac_dt_ids);

/**
 * @brief Linux Platform driver for GMACs
 */
static struct platform_driver nss_gmac_drv = {
	.probe = nss_gmac_probe,
	.remove = nss_gmac_remove,
	.driver = {
		   .name = "nss-gmac",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = of_match_ptr(nss_gmac_dt_ids),
#endif
		  },
};


/**
 * @brief Register Linux platform driver.
 * @return Returns 0 on success and -ve on failure.
 */
int32_t __init nss_gmac_register_driver(void)
{
	ctx.common_init_done = false;

	ctx.gmac_workqueue =
			create_singlethread_workqueue(NSS_GMAC_WORKQUEUE_NAME);
	if (!ctx.gmac_workqueue) {
		pr_info("%s: cannot create workqueue.\n",
			     __func__);
		goto link_state_wq_fail;
	}

	if (platform_driver_register(&nss_gmac_drv) != 0) {
		pr_info("platform drv reg failure\n");
		goto drv_register_fail;
	}

	return 0;

drv_register_fail:
	destroy_workqueue(ctx.gmac_workqueue);

link_state_wq_fail:
	return -EIO;
}


/**
 * @brief De-register network interfaces.
 * @return void
 */
void nss_gmac_exit_network_interfaces(void)
{
	uint32_t i;
	struct nss_gmac_dev *gmacdev;

	for (i = 0; i < NSS_MAX_GMACS; i++) {
		gmacdev = ctx.nss_gmac[i];
		if (gmacdev) {
			unregister_netdev(gmacdev->netdev);
			free_netdev(gmacdev->netdev);
			nss_gmac_detach(gmacdev);
			ctx.nss_gmac[i] = NULL;
		}
	}
}


/**
 * @brief Deregister Linux platform driver.
 */
void __exit nss_gmac_deregister_driver(void)
{
	nss_gmac_exit_network_interfaces();
	platform_driver_unregister(&nss_gmac_drv);

	if (ctx.gmac_workqueue) {
		destroy_workqueue(ctx.gmac_workqueue);
		ctx.gmac_workqueue = NULL;
	}

	nss_gmac_common_deinit(&ctx);
}


/**
 * @brief Module Init
 */
int __init nss_gmac_host_interface_init(void)
{
	pr_info("**********************************************************");
	pr_info("* Driver    :%s", nss_gmac_driver_string);
	pr_info("* Version   :%s", nss_gmac_driver_version);
	pr_info("* Copyright :%s", nss_gmac_copyright);
	pr_info("**********************************************************");

	/* Initialize the Network dependent services */
	if (nss_gmac_register_driver() != 0)
		return -EFAULT;

	return 0;
}


/**
 * @brief Module Exit
 */
void __exit nss_gmac_host_interface_exit(void)
{
	nss_gmac_deregister_driver();
}

module_init(nss_gmac_host_interface_init);
module_exit(nss_gmac_host_interface_exit);

MODULE_AUTHOR("Qualcomm Atheros");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NSS GMAC Network Driver");

