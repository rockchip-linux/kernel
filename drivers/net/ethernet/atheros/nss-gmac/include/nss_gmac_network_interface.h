/*
 **************************************************************************
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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
void nss_gmac_receive(struct net_device *netdev, struct sk_buff *skb,
						struct napi_struct *napi);
void nss_gmac_event_receive(void *if_ctx, int ev_type,
			    void *os_buf, uint32_t len);
void nss_gmac_open_work(struct work_struct *work);
void nss_gmac_ethtool_register(struct net_device *netdev);
void __exit nss_gmac_deregister_driver(void);
int32_t __init nss_gmac_register_driver(void);
void nss_gmac_linkdown(struct nss_gmac_dev *gmacdev);
void nss_gmac_linkup(struct nss_gmac_dev *gmacdev);
void nss_gmac_adjust_link(struct net_device *netdev);

#endif /* End of file */
