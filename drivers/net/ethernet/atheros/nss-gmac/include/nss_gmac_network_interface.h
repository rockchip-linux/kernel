/* Copyright (c) 2013, The Linux Foundation. All rights reserved.*/
/*
 * @file
 * Header file for the nework dependent functionality.
 * The function prototype listed here are linux dependent.
 * ---------------------------REVISION HISTORY-------------------
 * Qualcomm Atheros	01/Mar/2013		Modified for QCA NSS
 * Ubicom		01/Mar/2010		Modified for Ubicom32
 * Synopsys		01/Aug/2007		Created
 */

#ifndef __NSS_GMAC_NETWORK_INTERFACE_H__
#define __NSS_GMAC_NETWORK_INTERFACE_H__

#include <linux/ethtool.h>

#include <nss_gmac_dev.h>
#include <nss_api_if.h>

#define NET_IF_TIMEOUT (10*HZ)
#define NSS_GMAC_LINK_CHECK_TIME (HZ)

/* Private ioctls supported by GMACs */
#define IOCTL_READ_REGISTER	(SIOCDEVPRIVATE + 1)
#define IOCTL_WRITE_REGISTER	(SIOCDEVPRIVATE + 2)

/* Linux network interface APIs */
int32_t nss_gmac_linux_xmit_frames(struct sk_buff *skb,
				   struct net_device *netdev);
int32_t nss_gmac_linux_close(struct net_device *netdev);
int32_t nss_gmac_linux_open(struct net_device *netdev);
int32_t nss_gmac_linux_change_mtu(struct net_device *netdev, int32_t newmtu);
void nss_gmac_linux_tx_timeout(struct net_device *netdev);

/* NSS driver interface APIs */
void nss_gmac_receive(void *if_ctx, void *os_buf);
void nss_gmac_event_receive(void *if_ctx, nss_gmac_event_t ev_type,
			    void *os_buf, uint32_t len);
void nss_gmac_work(struct work_struct *work);
void nss_gmac_ethtool_register(struct net_device *netdev);
void __exit nss_gmac_deregister_driver(void);
int32_t __init nss_gmac_register_driver(void);
void nss_gmac_linux_powerup_mac(nss_gmac_dev *gmacdev);
void nss_gmac_linux_powerdown_mac(nss_gmac_dev *gmacdev);
void nss_gmac_linkdown(nss_gmac_dev *gmacdev);
void nss_gmac_linkup(nss_gmac_dev *gmacdev);
void nss_gmac_adjust_link(struct net_device *netdev);

#endif /* End of file */
