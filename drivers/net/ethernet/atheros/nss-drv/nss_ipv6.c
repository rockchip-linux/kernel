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
 * nss_ipv6.c
 *	NSS IPv6 APIs
 */
#include <linux/ppp_channel.h>
#include "nss_tx_rx_common.h"

extern void nss_rx_metadata_ipv6_rule_establish(struct nss_ctx_instance *nss_ctx, struct nss_ipv6_rule_establish *nire);
extern void nss_rx_ipv6_sync(struct nss_ctx_instance *nss_ctx, struct nss_ipv6_conn_sync *nirs);

/*
 * nss_ipv6_driver_conn_sync_update()
 *	Update driver specific information from the messsage.
 */
static void nss_ipv6_driver_conn_sync_update(struct nss_ctx_instance *nss_ctx, struct nss_ipv6_conn_sync *nics)
{
	struct nss_top_instance *nss_top = nss_ctx->nss_top;
#if (NSS_PPP_SUPPORT == 1)
	struct net_device *pppoe_dev = NULL;
#endif

	/*
	 * Update statistics maintained by NSS driver
	 */
	spin_lock_bh(&nss_top->stats_lock);
	nss_top->stats_ipv6[NSS_STATS_IPV6_ACCELERATED_RX_PKTS] += nics->flow_rx_packet_count + nics->return_rx_packet_count;
	nss_top->stats_ipv6[NSS_STATS_IPV6_ACCELERATED_RX_BYTES] += nics->flow_rx_byte_count + nics->return_rx_byte_count;
	nss_top->stats_ipv6[NSS_STATS_IPV6_ACCELERATED_TX_PKTS] += nics->flow_tx_packet_count + nics->return_tx_packet_count;
	nss_top->stats_ipv6[NSS_STATS_IPV6_ACCELERATED_TX_BYTES] += nics->flow_tx_byte_count + nics->return_tx_byte_count;
	spin_unlock(&nss_top->stats_lock);

	/*
	 * Update the PPPoE interface stats, if there is any PPPoE session on the interfaces.
	 */
#if (NSS_PPP_SUPPORT == 1)
	if (nics->flow_pppoe_session_id) {
		pppoe_dev = ppp_session_to_netdev(nics->flow_pppoe_session_id, (uint8_t *)nics->flow_pppoe_remote_mac);
		if (pppoe_dev) {
			ppp_update_stats(pppoe_dev, nics->flow_rx_packet_count, nics->flow_rx_byte_count,
					nics->flow_tx_packet_count, nics->flow_tx_byte_count);
			dev_put(pppoe_dev);
		}
	}

	if (nics->return_pppoe_session_id) {
		pppoe_dev = ppp_session_to_netdev(nics->return_pppoe_session_id, (uint8_t *)nics->return_pppoe_remote_mac);
		if (pppoe_dev) {
			ppp_update_stats(pppoe_dev, nics->return_rx_packet_count, nics->return_rx_byte_count,
				nics->return_tx_packet_count, nics->return_tx_byte_count);
			dev_put(pppoe_dev);
		}
	}
#endif
}

/*
 * nss_ipv6_driver_node_sync_update)
 *	Update driver specific information from the messsage.
 */
static void nss_ipv6_driver_node_sync_update(struct nss_ctx_instance *nss_ctx, struct nss_ipv6_node_sync *nins)
{
	struct nss_top_instance *nss_top = nss_ctx->nss_top;
	uint32_t i;

	/*
	 * Update statistics maintained by NSS driver
	 */
	spin_lock_bh(&nss_top->stats_lock);
	nss_top->stats_node[NSS_IPV6_RX_INTERFACE][NSS_STATS_NODE_RX_PKTS] += nins->node_stats.rx_packets;
	nss_top->stats_node[NSS_IPV6_RX_INTERFACE][NSS_STATS_NODE_RX_BYTES] += nins->node_stats.rx_bytes;
	nss_top->stats_node[NSS_IPV6_RX_INTERFACE][NSS_STATS_NODE_RX_DROPPED] += nins->node_stats.rx_dropped;
	nss_top->stats_node[NSS_IPV6_RX_INTERFACE][NSS_STATS_NODE_TX_PKTS] += nins->node_stats.tx_packets;
	nss_top->stats_node[NSS_IPV6_RX_INTERFACE][NSS_STATS_NODE_TX_BYTES] += nins->node_stats.tx_bytes;

	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_CREATE_REQUESTS] += nins->ipv6_connection_create_requests;
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_CREATE_COLLISIONS] += nins->ipv6_connection_create_collisions;
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_CREATE_INVALID_INTERFACE] += nins->ipv6_connection_create_invalid_interface;
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_DESTROY_REQUESTS] += nins->ipv6_connection_destroy_requests;
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_DESTROY_MISSES] += nins->ipv6_connection_destroy_misses;
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_HASH_HITS] += nins->ipv6_connection_hash_hits;
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_HASH_REORDERS] += nins->ipv6_connection_hash_reorders;
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_FLUSHES] += nins->ipv6_connection_flushes;
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_EVICTIONS] += nins->ipv6_connection_evictions;

	for (i = 0; i < NSS_EXCEPTION_EVENT_IPV6_MAX; i++) {
		 nss_top->stats_if_exception_ipv6[i] += nins->exception_events[i];
	}
	spin_unlock_bh(&nss_top->stats_lock);
}

/*
 * nss_ipv6_rx_msg_handler()
 *	Handle NSS -> HLOS messages for IPv6 bridge/route
 */
static void nss_ipv6_rx_msg_handler(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm, __attribute__((unused))void *app_data)
{
	struct nss_ipv6_msg *nim = (struct nss_ipv6_msg *)ncm;
	nss_ipv6_msg_callback_t cb;

	BUG_ON(ncm->interface != NSS_IPV6_RX_INTERFACE);

	/*
	 * Is this a valid request/response packet?
	 */
	if (nim->cm.type >= NSS_IPV6_MAX_MSG_TYPES) {
		nss_warning("%p: received invalid message %d for IPv6 interface", nss_ctx, nim->cm.type);
		return;
	}

	if (ncm->len > sizeof(struct nss_ipv6_msg)) {
		nss_warning("%p: tx request for another interface: %d", nss_ctx, ncm->interface);
		return;
	}

	/*
	 * Log failures
	 */
	nss_core_log_msg_failures(nss_ctx, ncm);

	/*
	 * Handle deprecated messages.  Eventually these messages should be removed.
	 */
	switch (nim->cm.type) {
	case NSS_IPV6_RX_ESTABLISH_RULE_MSG:
		nss_rx_metadata_ipv6_rule_establish(nss_ctx, &nim->msg.rule_establish);
		break;

	case NSS_IPV6_RX_NODE_STATS_SYNC_MSG:
		/*
		* Update driver statistics on node sync.
		*/
		nss_ipv6_driver_node_sync_update(nss_ctx, &nim->msg.node_stats);
		break;

	case NSS_IPV6_RX_CONN_STATS_SYNC_MSG:
		/*
		 * Update driver statistics on connection sync.
		 */
		nss_ipv6_driver_conn_sync_update(nss_ctx, &nim->msg.conn_stats);
		return nss_rx_ipv6_sync(nss_ctx, &nim->msg.conn_stats);
		break;
	}

	/*
	 * Update the callback and app_data for NOTIFY messages, IPv6 sends all notify messages
	 * to the same callback/app_data.
	 */
	if (nim->cm.response == NSS_CMM_RESPONSE_NOTIFY) {
		ncm->cb = (uint32_t)nss_ctx->nss_top->ipv6_callback;
		ncm->app_data = (uint32_t)nss_ctx->nss_top->ipv6_ctx;
	}

	/*
	 * Do we have a callback?
	 */
	if (!ncm->cb) {
		return;
	}

	/*
	 * Callback
	 */
	cb = (nss_ipv6_msg_callback_t)ncm->cb;
	cb((void *)ncm->app_data, nim);
}

/*
 * nss_ipv6_tx()
 *	Transmit an ipv6 message to the FW.
 */
nss_tx_status_t nss_ipv6_tx(struct nss_ctx_instance *nss_ctx, struct nss_ipv6_msg *nim)
{
	struct nss_ipv6_msg *nim2;
	struct nss_cmn_msg *ncm = &nim->cm;
	struct sk_buff *nbuf;
	int32_t status;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("%p: ipv6 msg dropped as core not ready", nss_ctx);
		return NSS_TX_FAILURE_NOT_READY;
	}

	/*
	 * Sanity check the message
	 */
	if (ncm->interface != NSS_IPV6_RX_INTERFACE) {
		nss_warning("%p: tx request for another interface: %d", nss_ctx, ncm->interface);
		return NSS_TX_FAILURE;
	}

	if (ncm->type > NSS_IPV6_MAX_MSG_TYPES) {
		nss_warning("%p: message type out of range: %d", nss_ctx, ncm->type);
		return NSS_TX_FAILURE;
	}

	if (ncm->len > sizeof(struct nss_ipv6_msg)) {
		nss_warning("%p: tx request for another interface: %d", nss_ctx, ncm->interface);
		return NSS_TX_FAILURE;
	}

	nbuf = dev_alloc_skb(NSS_NBUF_PAYLOAD_SIZE);
	if (unlikely(!nbuf)) {
		spin_lock_bh(&nss_ctx->nss_top->stats_lock);
		nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]++;
		spin_unlock_bh(&nss_ctx->nss_top->stats_lock);
		nss_warning("%p: msg dropped as command allocation failed", nss_ctx);
		return NSS_TX_FAILURE;
	}

	/*
	 * Copy the message to our skb.
	 */
	nim2 = (struct nss_ipv6_msg *)skb_put(nbuf, sizeof(struct nss_ipv6_msg));
	memcpy(nim2, nim, sizeof(struct nss_ipv6_msg));

	status = nss_core_send_buffer(nss_ctx, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		dev_kfree_skb_any(nbuf);
		nss_warning("%p: Unable to enqueue 'Destroy IPv6' rule\n", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit,
								NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_CMD_REQ]);
	return NSS_TX_SUCCESS;
}

/*
 **********************************
 Register/Unregister/Miscellaneous APIs
 **********************************
 */

/*
 * nss_ipv6_notify_register()
 *	Register to received IPv6 events.
 *
 * NOTE: Do we want to pass an nss_ctx here so that we can register for ipv6 on any core?
 */
struct nss_ctx_instance *nss_ipv6_notify_register(nss_ipv6_msg_callback_t cb, void *app_data)
{
	/*
	 * TODO: We need to have a new array in support of the new API
	 * TODO: If we use a per-context array, we would move the array into nss_ctx based.
	 */
	nss_top_main.ipv6_callback = cb;
	nss_top_main.ipv6_ctx = app_data;
	return &nss_top_main.nss[nss_top_main.ipv6_handler_id];
}

/*
 * nss_ipv6_notify_unregister()
 *	Unregister to received IPv6 events.
 *
 * NOTE: Do we want to pass an nss_ctx here so that we can register for ipv6 on any core?
 */
void nss_ipv6_notify_unregister(void)
{
	nss_top_main.ipv6_callback = NULL;
}

/*
 * nss_ipv6_get_mgr()
 *
 * TODO: This only suppports a single ipv6, do we ever want to support more?
 */
struct nss_ctx_instance *nss_ipv6_get_mgr(void)
{
	return (void *)&nss_top_main.nss[nss_top_main.ipv6_handler_id];
}

/*
 * nss_ipv6_register_handler()
 *	Register our handler to receive messages for this interface
 */
void nss_ipv6_register_handler()
{
	if (nss_core_register_handler(NSS_IPV6_RX_INTERFACE, nss_ipv6_rx_msg_handler, NULL) != NSS_CORE_STATUS_SUCCESS) {
		nss_warning("IPv6 handler failed to register");
	}
}

EXPORT_SYMBOL(nss_ipv6_tx);
EXPORT_SYMBOL(nss_ipv6_notify_register);
EXPORT_SYMBOL(nss_ipv6_notify_unregister);
EXPORT_SYMBOL(nss_ipv6_get_mgr);
EXPORT_SYMBOL(nss_ipv6_register_handler);
