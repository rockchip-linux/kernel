/*
 **************************************************************************
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all copies.
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
 * nss_tunipip6.c
 *
 * This file is the NSS DS-lit and IPP6  tunnel module
 * ------------------------REVISION HISTORY-----------------------------
 * Qualcomm Atheros	    15/sep/2013		     Created
 */

#include <linux/types.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <net/ipv6.h>
#include <net/ipip.h>
#include <net/ip6_tunnel.h>
#include <linux/if_arp.h>
#include <nss_api_if.h>

/*
 * NSS tunipip6 debug macros
 */
#if (NSS_TUNIPIP6_DEBUG_LEVEL < 1)
#define nss_tunipip6_assert(fmt, args...)
#else
#define nss_tunipip6_assert(c) if (!(c)) { BUG_ON(!(c)); }
#endif

#if (NSS_TUNIPIP6_DEBUG_LEVEL < 2)
#define nss_tunipip6_error(fmt, args...)
#else
#define nss_tunipip6_error(fmt, args...) printk(KERN_WARNING "nss tunipip6:"fmt, ##args)
#endif

#if (NSS_TUNIPIP6_DEBUG_LEVEL < 3)
#define nss_tunipip6_warning(fmt, args...)
#else
#define nss_tunipip6_warning(fmt, args...) printk(KERN_WARNING "nss tunipip6:"fmt, ##args)
#endif

#if (NSS_TUNIPIP6_DEBUG_LEVEL < 4)
#define nss_tunipip6_info(fmt, args...)
#else
#define nss_tunipip6_info(fmt, args...) printk(KERN_INFO "nss tunipip6 :"fmt, ##args)
#endif

#if (NSS_TUNIPIP6_DEBUG_LEVEL < 5)
#define nss_tunipip6_trace(fmt, args...)
#else
#define nss_tunipip6_trace(fmt, args...) printk(KERN_DEBUG "nss tunipip6 :"fmt, ##args)
#endif

void nss_tunipip6_exception(void *ctx, void *buf);
void nss_tunipip6_event_receive(void *ctx, struct nss_tunipip6_msg *msg);

/*
 * nss_tunipip6_tunnel
 *	DS-lite and ipip 6tunnel host instance
 */
struct nss_tunipip6_tunnel{
	struct nss_ctx_instance *nss_ctx;
	uint32_t if_num;
	struct net_device *netdev;
	uint32_t device_up;
};

/*
 *  tunipip6 stats structure
 */
struct nss_tunipip6_stats {
	uint32_t rx_packets;	/* Number of received packets */
	uint32_t rx_bytes;	/* Number of received bytes */
	uint32_t tx_packets;	/* Number of transmitted packets */
	uint32_t tx_bytes;	/* Number of transmitted bytes */
};

struct nss_tunipip6_tunnel g_tunipip6;

/*
 * Internal function
 */
static int
nss_tunipip6_dev_event(struct notifier_block  *nb,
			unsigned long event,
			void  *dev);

/*
 * Linux Net device Notifier
 */
struct notifier_block nss_tunipip6_notifier = {
	.notifier_call = nss_tunipip6_dev_event,
};

/*
 * nss_tunipip6_dev_up()
 *	IPIP6 Tunnel device i/f up handler
 */
void nss_tunipip6_dev_up( struct net_device * netdev)
{
	struct ip6_tnl *tunnel;
	struct nss_tunipip6_msg tnlmsg;
	struct nss_tunipip6_create_msg *tnlcfg;
	struct flowi6 *fl6;
	nss_tx_status_t status;

	/*
	 * Validate netdev for ipv6-in-ipv4  Tunnel
	 */
	if (netdev->type != ARPHRD_TUNNEL6 ) {
		return;
	}

	tunnel = (struct ip6_tnl *)netdev_priv(netdev);

	/*
	 * Find he Tunnel device flow information
	 */

	fl6  = &tunnel->fl.u.ip6;

	nss_tunipip6_trace(" Tunnel Param srcaddr %x:%x:%x:%x  daddr %x:%x:%x:%x \n",
			fl6->saddr.s6_addr32[0], fl6->saddr.s6_addr32[1],
			fl6->saddr.s6_addr32[2], fl6->saddr.s6_addr32[3],
			fl6->daddr.s6_addr32[0], fl6->daddr.s6_addr32[1],
			fl6->daddr.s6_addr32[2], fl6->daddr.s6_addr32[3] );
	nss_tunipip6_trace(" hop limit %d \n", tunnel->parms.hop_limit);
	nss_tunipip6_trace(" tunnel param flag %x  fl6.flowlabel %x  \n", tunnel->parms.flags, fl6->flowlabel);

	/*
	 * Register ipip6 tunnel with NSS
	 */
	g_tunipip6.nss_ctx = nss_register_tunipip6_if(g_tunipip6.if_num,
				nss_tunipip6_exception,
				nss_tunipip6_event_receive,
				netdev);
	if (g_tunipip6.nss_ctx == NULL) {
		nss_tunipip6_trace("nss_register_tunipip6_if Failed \n");
		return;
	} else {
		nss_tunipip6_trace("nss_register_tunipip6_if Success \n");
	}

	/*
	 *Prepare The Tunnel configuration parameter to send to nss
	 */
	memset(&tnlmsg, 0, sizeof(struct nss_tunipip6_msg));
	tnlcfg = &tnlmsg.msg.tunipip6_create;

	tnlcfg->saddr[0] = ntohl(fl6->saddr.s6_addr32[0]);
	tnlcfg->saddr[1] = ntohl(fl6->saddr.s6_addr32[1]);
	tnlcfg->saddr[2] = ntohl(fl6->saddr.s6_addr32[2]);
	tnlcfg->saddr[3] = ntohl(fl6->saddr.s6_addr32[3]);
	tnlcfg->daddr[0] = ntohl(fl6->daddr.s6_addr32[0]);
	tnlcfg->daddr[1] = ntohl(fl6->daddr.s6_addr32[1]);
	tnlcfg->daddr[2] = ntohl(fl6->daddr.s6_addr32[2]);
	tnlcfg->daddr[3] = ntohl(fl6->daddr.s6_addr32[3]);
	tnlcfg->hop_limit = tunnel->parms.hop_limit;
	tnlcfg->flags = ntohl(tunnel->parms.flags);
	tnlcfg->flowlabel = fl6->flowlabel;  /*flow Label In kernel is stored in big endian format*/
	nss_tunipip6_trace(" Tunnel Param srcaddr %x:%x:%x:%x  daddr %x:%x:%x:%x \n",
			tnlcfg->saddr[0], tnlcfg->saddr[1],
			tnlcfg->saddr[2], tnlcfg->saddr[3],
			tnlcfg->daddr[0], tnlcfg->daddr[1],
			tnlcfg->daddr[2], tnlcfg->daddr[3] );


	nss_tunipip6_trace("Sending IPIP6 tunnel i/f up command to NSS	%x \n",
			(int)g_tunipip6.nss_ctx);

	/*
	 * Send IPIP6 Tunnel UP command to NSS
	 */
	nss_cmn_msg_init(&tnlmsg.cm, NSS_TUNIPIP6_INTERFACE, NSS_TUNIPIP6_TX_IF_CREATE,
			sizeof(struct nss_tunipip6_create_msg), NULL, NULL);

	status = nss_tunipip6_tx(g_tunipip6.nss_ctx, &tnlmsg);
	if (status != NSS_TX_SUCCESS) {
		nss_tunipip6_error("Tunnel up command error %d \n", status);
		return;
	}

	g_tunipip6.device_up = 1;
}

/*
 * nss_tunipip6_dev_down()
 *	IPP6 Tunnel device i/f down handler
 */
void nss_tunipip6_dev_down( struct net_device * netdev)
{
	struct nss_tunipip6_msg tnlmsg;
	struct nss_tunipip6_destroy_msg *tnlcfg;
	nss_tx_status_t status;

	/*
	 * Check if tunnel ipip6 is registered ?
	 */
	if(g_tunipip6.nss_ctx == NULL){
		return;
	}

	/*
	 * Validate netdev for ipv6-in-ipv4  Tunnel
	 */
	if (netdev->type != ARPHRD_TUNNEL6) {
		return;
	}

	/*
	 * TODO: Strick check required if its the same tunnel
	 * registerd with us
	 */

	memset(&tnlmsg, 0, sizeof(struct nss_tunipip6_msg));
	tnlcfg = &tnlmsg.msg.tunipip6_destroy;

	nss_tunipip6_trace("Sending Tunnel ipip6 Down command %x \n",g_tunipip6.if_num);

	/*
	 * Send IPIP6 Tunnel DOWN command to NSS
	 */
	nss_cmn_msg_init(&tnlmsg.cm, NSS_TUNIPIP6_INTERFACE, NSS_TUNIPIP6_TX_IF_DESTROY,
			sizeof(struct nss_tunipip6_destroy_msg), NULL, NULL);

	status = nss_tunipip6_tx(g_tunipip6.nss_ctx, &tnlmsg);
	if (status != NSS_TX_SUCCESS) {
		nss_tunipip6_error("Tunnel down command error %d \n", status);
		return;
	}

	/*
	 * Un-Register IPIP6 tunnel with NSS
	 */
	nss_unregister_tunipip6_if(g_tunipip6.if_num);
	g_tunipip6.nss_ctx = NULL;
	g_tunipip6.device_up = 0;
}

/*
 * nss_tun6rd_dev_event()
 *	Net device notifier for ipip6 module
 */
static int nss_tunipip6_dev_event(struct notifier_block  *nb,
		unsigned long event, void  *dev)
{
	struct net_device *netdev = (struct net_device *)dev;

	nss_tunipip6_trace("%s\n",__FUNCTION__);
	switch (event) {
	case NETDEV_UP:
		nss_tunipip6_trace(" NETDEV_UP :event %lu name %s \n",
				event,netdev->name);
		nss_tunipip6_dev_up(netdev);
		break;

	case NETDEV_DOWN:
		nss_tunipip6_trace(" NETDEV_DOWN :event %lu name %s \n",
				event,netdev->name);
		nss_tunipip6_dev_down(netdev);
		break;

	default:
		nss_tunipip6_trace("Unhandled notifier dev %s  event %x  \n",
				netdev->name,(int)event);
		break;
	}

	return NOTIFY_DONE;
}

/*
 * nss_tunipip6_exception()
 *	Exception handler registered to NSS driver
 */
void nss_tunipip6_exception(void *ctx, void *buf)
{
	struct net_device *dev = (struct net_device *)ctx;
	struct sk_buff *skb = (struct sk_buff *)buf;
	const struct iphdr *iph;

	skb->dev = dev;
	nss_tunipip6_info("received - %d bytes name %s ver %x \n",
			skb->len,dev->name,skb->data[0]);

	iph = (const struct iphdr *)skb->data;

	/*
	 *Packet after Decap/Encap Did not find the Rule.
	 */
	if (iph->version == 4) {
		skb->protocol = htons(ETH_P_IP);
	} else {
		skb->protocol = htons(ETH_P_IPV6);
	}

	skb_reset_network_header(skb);
	skb->pkt_type = PACKET_HOST;
	skb->skb_iif = dev->ifindex;
	skb->ip_summed = CHECKSUM_NONE;
	netif_receive_skb(skb);
}

/*
 *  nss_tunipip6_update_dev_stats
 *	Update the Dev stats received from NetAp
 */
static void nss_tunipip6_update_dev_stats(struct net_device *dev,
					struct nss_tunipip6_stats_sync_msg *sync_stats)
{
	void *ptr;
	struct nss_tunipip6_stats stats;

	stats.rx_packets = sync_stats->node_stats.rx_packets;
	stats.rx_bytes = sync_stats->node_stats.rx_bytes;
	stats.tx_packets = sync_stats->node_stats.tx_packets;
	stats.tx_bytes = sync_stats->node_stats.tx_bytes;

	ptr = (void *)&stats;
	ip6_update_offload_stats(dev, ptr);

}

/**
 * @brief Event Callback to receive events from NSS
 * @param[in] pointer to net device context
 * @param[in] event type
 * @param[in] pointer to buffer
 * @param[in] length of buffer
 * @return Returns void
 */
void nss_tunipip6_event_receive(void *if_ctx, struct nss_tunipip6_msg *tnlmsg)
{
	struct net_device *netdev = NULL;
	netdev = (struct net_device *)if_ctx;

	switch (tnlmsg->cm.type) {
	case NSS_TUNIPIP6_RX_STATS_SYNC:
		 nss_tunipip6_update_dev_stats(netdev, (struct nss_tunipip6_stats_sync_msg *)&tnlmsg->msg.stats_sync );
		break;

	default:
		nss_tunipip6_info("%s: Unknown Event from NSS",
			      __FUNCTION__);
		break;
	}
}

/*
 * nss_tunipip6_init_module()
 *	Tunnel ipip6 module init function
 */
int __init nss_tunipip6_init_module(void)
{
	nss_tunipip6_info("module (platform - IPQ806x , Build - %s:%s) loaded\n",
			__DATE__, __TIME__);

	register_netdevice_notifier(&nss_tunipip6_notifier);
	nss_tunipip6_trace("Netdev Notifier registerd \n");

	g_tunipip6.if_num = NSS_TUNIPIP6_INTERFACE;
	g_tunipip6.netdev = NULL;
	g_tunipip6.device_up = 0;
	g_tunipip6.nss_ctx = NULL;

	return 0;
}

/*
 * nss_tunipip6_exit_module()
 *	Tunnel ipip6 module exit function
 */
void __exit nss_tunipip6_exit_module(void)
{

	unregister_netdevice_notifier(&nss_tunipip6_notifier);
	nss_tunipip6_info("module unloaded\n");
}

module_init(nss_tunipip6_init_module);
module_exit(nss_tunipip6_exit_module);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("NSS tunipip6 offload manager");
