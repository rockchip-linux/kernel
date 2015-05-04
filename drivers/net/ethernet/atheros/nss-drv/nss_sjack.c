/*
 **************************************************************************
 * Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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

#include "nss_tx_rx_common.h"

/*
 * nss_sjack_node_sync_update()
 *	Update sjack node stats.
 */
static void nss_sjack_node_sync_update(struct nss_ctx_instance *nss_ctx, struct nss_sjack_stats_sync_msg *nins)
{
	struct nss_top_instance *nss_top = nss_ctx->nss_top;

	/*
	 * Update SJACK node stats.
	 */
	spin_lock_bh(&nss_top->stats_lock);
	nss_top->stats_node[NSS_SJACK_INTERFACE][NSS_STATS_NODE_RX_PKTS] += nins->node_stats.rx_packets;
	nss_top->stats_node[NSS_SJACK_INTERFACE][NSS_STATS_NODE_RX_BYTES] += nins->node_stats.rx_bytes;
	nss_top->stats_node[NSS_SJACK_INTERFACE][NSS_STATS_NODE_RX_DROPPED] += nins->node_stats.rx_dropped;
	nss_top->stats_node[NSS_SJACK_INTERFACE][NSS_STATS_NODE_TX_PKTS] += nins->node_stats.tx_packets;
	nss_top->stats_node[NSS_SJACK_INTERFACE][NSS_STATS_NODE_TX_BYTES] += nins->node_stats.tx_bytes;
	spin_unlock_bh(&nss_top->stats_lock);
}

/*
 * nss_sjack_handler()
 * 	Handle NSS -> HLOS messages for sjack
 */
static void nss_sjack_handler(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm,
				__attribute__((unused))void *app_data)
{
	void *ctx;
	nss_sjack_msg_callback_t cb;
	struct nss_sjack_msg *nsm = (struct nss_sjack_msg *)ncm;

	BUG_ON(ncm->interface != NSS_SJACK_INTERFACE);

	/*
	 * Is this a valid request/response packet?
	 */
	if (ncm->type >=  NSS_SJACK_MAX_MSG_TYPE) {
		nss_warning("%p: received invalid message %d for sjack interface", nss_ctx, ncm->type);
		return;
	}

	if (ncm->len > sizeof(struct nss_sjack_msg)) {
		nss_warning("%p: tx request for another interface: %d", nss_ctx, ncm->interface);
		return;
	}

	/*
	 * Update the callback and app_data for NOTIFY messages, sjack sends all notify messages
	 * to the same callback/app_data.
	 */
	if (ncm->response == NSS_CMM_RESPONSE_NOTIFY) {
		ncm->cb = (uint32_t)nss_ctx->nss_top->if_rx_msg_callback[ncm->interface];
	}

	/*
	 * Log failures
	 */
	nss_core_log_msg_failures(nss_ctx, ncm);

	switch (ncm->type) {
	case NSS_SJACK_STATS_SYNC_MSG:
		/*
		 * Update sjack statistics on node sync.
		 */
		nss_sjack_node_sync_update(nss_ctx, &nsm->msg.stats_sync);
		break;
	}

	/*
	 * Do we have a call back
	 */
	if (!ncm->cb) {
		return;
	}

	/*
	 * callback
	 */
	cb = (nss_sjack_msg_callback_t)ncm->cb;
	ctx =  nss_ctx->nss_top->subsys_dp_register[ncm->interface].ndev;

	cb(ctx, ncm);
}


/*
 * nss_sjack_tx_msg()
 * 	Transmit a sjack message to NSSFW
 */
nss_tx_status_t nss_sjack_tx_msg(struct nss_ctx_instance *nss_ctx, struct nss_sjack_msg *msg)
{
	struct nss_sjack_msg *nm;
	struct nss_cmn_msg *ncm = &msg->cm;
	struct sk_buff *nbuf;
	int32_t status;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("%p: sjack msg dropped as core not ready", nss_ctx);
		return NSS_TX_FAILURE_NOT_READY;
	}

	/*
	 * Sanity check the message
	 */
	if (ncm->interface != NSS_SJACK_INTERFACE) {
		nss_warning("%p: tx request for another interface: %d", nss_ctx, ncm->interface);
		return NSS_TX_FAILURE;
	}

	if (ncm->type > NSS_SJACK_MAX_MSG_TYPE) {
		nss_warning("%p: message type out of range: %d", nss_ctx, ncm->type);
		return NSS_TX_FAILURE;
	}

	if (ncm->len > sizeof(struct nss_sjack_msg)) {
		nss_warning("%p: message length is invalid: %d", nss_ctx, ncm->len);
		return NSS_TX_FAILURE;
	}

	nbuf = dev_alloc_skb(NSS_NBUF_PAYLOAD_SIZE);
	if (unlikely(!nbuf)) {
		NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]);
		nss_warning("%p: msg dropped as command allocation failed", nss_ctx);
		return NSS_TX_FAILURE;
	}

	/*
	 * Copy the message to our skb
	 */
	nm = (struct nss_sjack_msg *)skb_put(nbuf, sizeof(struct nss_sjack_msg));
	memcpy(nm, msg, sizeof(struct nss_sjack_msg));

	status = nss_core_send_buffer(nss_ctx, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		dev_kfree_skb_any(nbuf);
		nss_warning("%p: Unable to enqueue 'sjack message' \n", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit,
				NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_CMD_REQ]);
	return NSS_TX_SUCCESS;
}

/*
 * nss_sjack_register_if()
 */
struct nss_ctx_instance *nss_sjack_register_if(uint32_t if_num, struct net_device *netdev,
						nss_sjack_msg_callback_t event_callback)
{
	nss_assert(if_num == NSS_SJACK_INTERFACE);

	nss_top_main.subsys_dp_register[if_num].ndev = netdev;

	nss_top_main.if_rx_msg_callback[if_num] = event_callback;

	return (struct nss_ctx_instance *)&nss_top_main.nss[nss_top_main.sjack_handler_id];
}

/*
 * nss_sjack_unregister_if()
 */
void nss_sjack_unregister_if(uint32_t if_num)
{
	nss_assert(if_num == NSS_SJACK_INTERFACE);

	nss_top_main.subsys_dp_register[if_num].ndev = NULL;
	nss_top_main.if_rx_msg_callback[if_num] = NULL;

	return;
}

/*
 * nss_sjack_register_handler()
 *	Registering handler for sending msg to sjack node on NSS.
 */
void nss_sjack_register_handler(void)
{
	nss_core_register_handler(NSS_SJACK_INTERFACE, nss_sjack_handler, NULL);
}

EXPORT_SYMBOL(nss_sjack_register_if);
EXPORT_SYMBOL(nss_sjack_unregister_if);
EXPORT_SYMBOL(nss_sjack_tx_msg);
