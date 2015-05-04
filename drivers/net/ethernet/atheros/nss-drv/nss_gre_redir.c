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
 * Spinlock to update tunnel stats
 */
DEFINE_SPINLOCK(nss_gre_redir_stats_lock);

/*
 * Array to hold tunnel stats along with if_num
 */
static struct nss_gre_redir_tunnel_stats tun_stats[NSS_GRE_REDIR_MAX_INTERFACES];

/*
 * nss_gre_redir_tunnel_update_stats()
 * 	Update gre_redir tunnel stats.
 */
static void nss_gre_redir_tunnel_update_stats(struct nss_ctx_instance *nss_ctx, int if_num, struct nss_gre_redir_stats_sync_msg *ngss)
{
	int i;

	spin_lock_bh(&nss_gre_redir_stats_lock);
	for (i = 0; i < NSS_GRE_REDIR_MAX_INTERFACES; i++) {
		if ((tun_stats[i].if_num == if_num) && (tun_stats[i].valid)) {

			tun_stats[i].node_stats.rx_packets += ngss->node_stats.rx_packets;
			tun_stats[i].node_stats.rx_bytes += ngss->node_stats.rx_bytes;
			tun_stats[i].node_stats.tx_packets += ngss->node_stats.tx_packets;
			tun_stats[i].node_stats.tx_bytes += ngss->node_stats.tx_bytes;
			tun_stats[i].node_stats.rx_dropped += ngss->node_stats.rx_dropped;
			tun_stats[i].tx_dropped += ngss->tx_dropped;

			break;
		}
	}
	spin_unlock_bh(&nss_gre_redir_stats_lock);
}


/*
 * nss_gre_redir_handler()
 * 	Handle NSS -> HLOS messages for gre tunnel
 */
static void nss_gre_redir_msg_handler(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm, __attribute__((unused))void *app_data)
{
	struct nss_gre_redir_msg *ngrm = (struct nss_gre_redir_msg *)ncm;
	void *ctx;
	nss_gre_redir_msg_callback_t cb;

	/*
	 * interface should either be dynamic interface for receiving tunnel msg or GRE_REDIR interface for
	 * receiving base node messages.
	 */
	BUG_ON(((ncm->interface < NSS_DYNAMIC_IF_START) || (ncm->interface >= (NSS_DYNAMIC_IF_START + NSS_MAX_DYNAMIC_INTERFACES))) &&
		ncm->interface != NSS_GRE_REDIR_INTERFACE);

	/*
	 * Is this a valid request/response packet?
	 */
	if (ncm->type >=  NSS_GRE_REDIR_MAX_MSG_TYPES) {
		nss_warning("%p: received invalid message %d for gre interface", nss_ctx, ncm->type);
		return;
	}

	if (ncm->len > sizeof(struct nss_gre_redir_msg)) {
		nss_warning("%p: tx request for another interface: %d", nss_ctx, ncm->interface);
		return;
	}

	/*
	 * Update the callback and app_data for NOTIFY messages, gre sends all notify messages
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
	case NSS_GRE_REDIR_RX_STATS_SYNC_MSG:
		/*
		 * Update Tunnel statistics.
		 */
		if (!(nss_is_dynamic_interface(ncm->interface))) {
			nss_warning("%p: stats received for wrong interface %d\n", nss_ctx, ncm->interface);
			break;
		}

		nss_gre_redir_tunnel_update_stats(nss_ctx, ncm->interface, &ngrm->msg.stats_sync);
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
	cb = (nss_gre_redir_msg_callback_t)ncm->cb;
	ctx =  nss_ctx->nss_top->subsys_dp_register[ncm->interface].ndev;

	/*
	 * call gre tunnel callback
	 */
	cb(ctx, ncm);
}

/*
 * nss_gre_redir_get_stats()
 * 	get gre_redir tunnel stats.
 */
bool nss_gre_redir_get_stats(int index, struct nss_gre_redir_tunnel_stats *stats)
{
	spin_lock_bh(&nss_gre_redir_stats_lock);
	if (!tun_stats[index].valid) {
		spin_unlock_bh(&nss_gre_redir_stats_lock);
		return false;
	}

	if (nss_is_dynamic_interface(tun_stats[index].if_num) == false) {
		spin_unlock_bh(&nss_gre_redir_stats_lock);
		return false;
	}

	memcpy(stats, &tun_stats[index], sizeof(struct nss_gre_redir_tunnel_stats));
	spin_unlock_bh(&nss_gre_redir_stats_lock);

	return true;
}

/*
 * nss_gre_redir_tx_msg()
 * 	Transmit a gre message to NSSFW
 */
nss_tx_status_t nss_gre_redir_tx_msg(struct nss_ctx_instance *nss_ctx, struct nss_gre_redir_msg *msg)
{
	struct nss_gre_redir_msg *nm;
	struct nss_cmn_msg *ncm = &msg->cm;
	struct sk_buff *nbuf;
	int32_t status;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("%p: gre msg dropped as core not ready", nss_ctx);
		return NSS_TX_FAILURE_NOT_READY;
	}

	/*
	 * Sanity check the message
	 */

	/*
	 * interface should either be dynamic interface to transmit tunnel msg or GRE_REDIR interface to transmit
	 * base node messages.
	 */
	if (((ncm->interface < NSS_DYNAMIC_IF_START) || (ncm->interface >= (NSS_DYNAMIC_IF_START + NSS_MAX_DYNAMIC_INTERFACES))) &&
		ncm->interface != NSS_GRE_REDIR_INTERFACE) {
		nss_warning("%p: tx request for another interface: %d", nss_ctx, ncm->interface);
		return NSS_TX_FAILURE;
	}

	if (ncm->type > NSS_GRE_REDIR_MAX_MSG_TYPES) {
		nss_warning("%p: message type out of range: %d", nss_ctx, ncm->type);
		return NSS_TX_FAILURE;
	}

	if (ncm->len > sizeof(struct nss_gre_redir_msg)) {
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
	nm = (struct nss_gre_redir_msg *)skb_put(nbuf, sizeof(struct nss_gre_redir_msg));
	memcpy(nm, msg, sizeof(struct nss_gre_redir_msg));

	status = nss_core_send_buffer(nss_ctx, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		dev_kfree_skb_any(nbuf);
		nss_warning("%p: Unable to enqueue 'gre message' \n", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit,
				NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_CMD_REQ]);
	return NSS_TX_SUCCESS;
}

/*
 * nss_gre_redir_tx_buf()
 *	Send packet to gre_redir interface owned by NSS
 */
nss_tx_status_t nss_gre_redir_tx_buf(struct nss_ctx_instance *nss_ctx, struct sk_buff *os_buf, uint32_t if_num)
{
	int32_t status;

	nss_trace("%p: gre_redir If Tx packet, id:%d, data=%p", nss_ctx, if_num, os_buf->data);

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("%p: 'Phys If Tx' packet dropped as core not ready", nss_ctx);
		return NSS_TX_FAILURE_NOT_READY;
	}

	status = nss_core_send_buffer(nss_ctx, if_num, os_buf, NSS_IF_DATA_QUEUE_0, H2N_BUFFER_PACKET, 0);
	if (unlikely(status != NSS_CORE_STATUS_SUCCESS)) {
		nss_warning("%p: Unable to enqueue 'Phys If Tx' packet\n", nss_ctx);
		if (status == NSS_CORE_STATUS_FAILURE_QUEUE) {
			return NSS_TX_FAILURE_QUEUE;
		}

		return NSS_TX_FAILURE;
	}

	/*
	 * Kick the NSS awake so it can process our new entry.
	 */
	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_DATA_QUEUE_0].desc_ring.int_bit,
				NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_PACKET]);
	return NSS_TX_SUCCESS;
}

/*
 ***********************************
 * Register/Unregister/Miscellaneous APIs
 ***********************************
 */

/*
 * nss_gre_redir_register_if()
 */
struct nss_ctx_instance *nss_gre_redir_register_if(uint32_t if_num, struct net_device *netdev, nss_gre_redir_data_callback_t cb_func_data,
							nss_gre_redir_msg_callback_t cb_func_msg, uint32_t features)
{
	uint32_t status;
	int i;

	nss_assert((if_num >= NSS_DYNAMIC_IF_START) && (if_num < (NSS_DYNAMIC_IF_START + NSS_MAX_DYNAMIC_INTERFACES)));

	/*
	 * Registering handler for sending tunnel interface msgs to NSS.
	 */
	status = nss_core_register_handler(if_num, nss_gre_redir_msg_handler, NULL);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		nss_warning("Not able to register handler for gre_redir interface %d with NSS core\n", if_num);
		return NULL;
	}

	nss_top_main.subsys_dp_register[if_num].ndev = netdev;
        nss_top_main.subsys_dp_register[if_num].cb = cb_func_data;
	nss_top_main.subsys_dp_register[if_num].app_data = NULL;
	nss_top_main.subsys_dp_register[if_num].features = features;

        nss_top_main.if_rx_msg_callback[if_num] = cb_func_msg;

	spin_lock_bh(&nss_gre_redir_stats_lock);
	for (i = 0; i < NSS_GRE_REDIR_MAX_INTERFACES; i++) {
		if (!(tun_stats[i].valid)) {
			tun_stats[i].valid = true;
			tun_stats[i].if_num = if_num;
			break;
		}
	}
	spin_unlock_bh(&nss_gre_redir_stats_lock);

        return (struct nss_ctx_instance *)&nss_top_main.nss[nss_top_main.gre_redir_handler_id];
}

/*
 * nss_gre_redir_unregister_if()
 */
void nss_gre_redir_unregister_if(uint32_t if_num)
{
	uint32_t status;
	int i;

	nss_assert((if_num >= NSS_DYNAMIC_IF_START) && (if_num < (NSS_DYNAMIC_IF_START + NSS_MAX_DYNAMIC_INTERFACES)));

	status = nss_core_unregister_handler(if_num);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		nss_warning("Not able to unregister handler for gre_redir interface %d with NSS core\n", if_num);
		return;
	}

	nss_top_main.subsys_dp_register[if_num].ndev = NULL;
        nss_top_main.subsys_dp_register[if_num].cb = NULL;
	nss_top_main.subsys_dp_register[if_num].app_data = NULL;
	nss_top_main.subsys_dp_register[if_num].features = 0;

	nss_top_main.if_rx_msg_callback[if_num] = NULL;

	spin_lock_bh(&nss_gre_redir_stats_lock);
	for (i = 0; i < NSS_GRE_REDIR_MAX_INTERFACES; i++) {
		if ((tun_stats[i].if_num == if_num) && (tun_stats[i].valid)) {
			tun_stats[i].valid = false;
			tun_stats[i].if_num = -1;
			break;
		}
	}
	spin_unlock_bh(&nss_gre_redir_stats_lock);
}

/*
 * nss_gre_redir_register_handler()
 *	Registering handler for sending msg to base gre_redir node on NSS.
 */
void nss_gre_redir_register_handler(void)
{
	uint32_t status = nss_core_register_handler(NSS_GRE_REDIR_INTERFACE, nss_gre_redir_msg_handler, NULL);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		nss_warning("Not able to register handler for gre_redir base interface with NSS core\n");
		return;
	}
}

EXPORT_SYMBOL(nss_gre_redir_tx_msg);
EXPORT_SYMBOL(nss_gre_redir_tx_buf);
EXPORT_SYMBOL(nss_gre_redir_register_if);
EXPORT_SYMBOL(nss_gre_redir_unregister_if);
EXPORT_SYMBOL(nss_gre_redir_get_stats);
