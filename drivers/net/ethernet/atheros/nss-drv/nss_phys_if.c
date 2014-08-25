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
 * nss_phy_if.c
 *	NSS physical interface functions
 */

#include "nss_tx_rx_common.h"

/*
 * TODO: Once we are moved to the new API, this function is deprecated.
 */
extern void nss_rx_metadata_gmac_stats_sync(struct nss_ctx_instance *nss_ctx,
		struct nss_phys_if_stats *stats, uint16_t interface);


/*
 * nss_phys_if_update_driver_stats()
 *	Snoop the extended message and update driver statistics.
 */
static void nss_phys_if_update_driver_stats(struct nss_ctx_instance *nss_ctx, uint32_t id, struct nss_phys_if_stats *stats)
{
	struct nss_top_instance *nss_top = nss_ctx->nss_top;
	uint64_t *top_stats = &(nss_top->stats_gmac[id][0]);

	spin_lock_bh(&nss_top->stats_lock);
	top_stats[NSS_STATS_GMAC_TOTAL_TICKS] += stats->estats.gmac_total_ticks;
	if (unlikely(top_stats[NSS_STATS_GMAC_WORST_CASE_TICKS] < stats->estats.gmac_worst_case_ticks)) {
		top_stats[NSS_STATS_GMAC_WORST_CASE_TICKS] = stats->estats.gmac_worst_case_ticks;
	}
	top_stats[NSS_STATS_GMAC_ITERATIONS] += stats->estats.gmac_iterations;
	spin_unlock_bh(&nss_top->stats_lock);
}

/*
 * nss_phys_if_msg_handler()
 *	Handle NSS -> HLOS messages for physical interface/gmacs
 */
static void nss_phys_if_msg_handler(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm,
		__attribute__((unused))void *app_data)
{
	struct nss_phys_if_msg *nim = (struct nss_phys_if_msg *)ncm;
	nss_phys_if_msg_callback_t cb;

	/*
	 * Sanity check the message type
	 */
	if (ncm->type > NSS_PHYS_IF_MAX_MSG_TYPES) {
		nss_warning("%p: message type out of range: %d", nss_ctx, ncm->type);
		return;
	}

	if (ncm->interface > NSS_MAX_PHYSICAL_INTERFACES) {
		nss_warning("%p: response for another interface: %d", nss_ctx, ncm->interface);
		return;
	}

	if (ncm->len > sizeof(struct nss_phys_if_msg)) {
		nss_warning("%p: message length too big: %d", nss_ctx, ncm->len);
		return;
	}

	/*
	 * Messages value that are within the base class are handled by the base class.
	 */
	if (ncm->type < NSS_IF_MAX_MSG_TYPES) {
		return nss_if_msg_handler(nss_ctx, ncm, app_data);
	}

	/*
	 * Log failures
	 */
	nss_core_log_msg_failures(nss_ctx, ncm);

	/*
	 * Snoop messages for local driver and handle deprecated interfaces.
	 */
	switch (nim->cm.type) {
	case NSS_PHYS_IF_EXTENDED_STATS_SYNC:
		/*
		 * To create the old API gmac statistics, we use the new extended GMAC stats.
		 */
		nss_phys_if_update_driver_stats(nss_ctx, ncm->interface, &nim->msg.stats);
		nss_rx_metadata_gmac_stats_sync(nss_ctx, &nim->msg.stats, ncm->interface);
		break;
	}

	/*
	 * Update the callback and app_data for NOTIFY messages, IPv4 sends all notify messages
	 * to the same callback/app_data.
	 */
	if (ncm->response == NSS_CMM_RESPONSE_NOTIFY) {
		ncm->cb = (uint32_t)nss_ctx->nss_top->phys_if_msg_callback[ncm->interface];
		ncm->app_data = (uint32_t)nss_ctx->nss_top->if_ctx[ncm->interface];
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
	cb = (nss_phys_if_msg_callback_t)ncm->cb;
	cb((void *)ncm->app_data, nim);
}

/*
 * nss_phys_if_tx_buf()
 *	Send packet to physical interface owned by NSS
 */
nss_tx_status_t nss_phys_if_tx_buf(struct nss_ctx_instance *nss_ctx, struct sk_buff *os_buf, uint32_t if_num)
{
	int32_t status;

	nss_trace("%p: Phys If Tx packet, id:%d, data=%p", nss_ctx, if_num, os_buf->data);

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("%p: 'Phys If Tx' packet dropped as core not ready", nss_ctx);
		return NSS_TX_FAILURE_NOT_READY;
	}

	status = nss_core_send_buffer(nss_ctx, if_num, os_buf, NSS_IF_DATA_QUEUE, H2N_BUFFER_PACKET, 0);
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
	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_DATA_QUEUE].desc_ring.int_bit,
									NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_PACKET]);
	return NSS_TX_SUCCESS;
}

/*
 * nss_phys_if_tx_msg()
 */
nss_tx_status_t nss_phys_if_tx_msg(struct nss_ctx_instance *nss_ctx, struct nss_phys_if_msg *nim)
{
	struct nss_cmn_msg *ncm = &nim->cm;
	struct nss_phys_if_msg *nim2;
	struct net_device *dev;
	struct sk_buff *nbuf;
	uint32_t if_num;
	int32_t status;

	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("Interface could not be created as core not ready");
		return NSS_TX_FAILURE;
	}

	/*
	 * Sanity check the message
	 */
	if (!NSS_IS_IF_TYPE(PHYSICAL, ncm->interface)) {
		nss_warning("%p: tx request for another interface: %d", nss_ctx, ncm->interface);
		return NSS_TX_FAILURE;
	}

	if (ncm->type > NSS_PHYS_IF_MAX_MSG_TYPES) {
		nss_warning("%p: message type out of range: %d", nss_ctx, ncm->type);
		return NSS_TX_FAILURE;
	}

	if (ncm->len > sizeof(struct nss_phys_if_msg)) {
		nss_warning("%p: invalid length: %d", nss_ctx, ncm->len);
		return NSS_TX_FAILURE;
	}

	if_num = ncm->interface;
	dev = nss_top_main.if_ctx[if_num];
	if (!dev) {
		nss_warning("%p: Unregister physical interface %d: no context", nss_ctx, if_num);
		return NSS_TX_FAILURE_BAD_PARAM;
	}

	nbuf = dev_alloc_skb(NSS_NBUF_PAYLOAD_SIZE);
	if (unlikely(!nbuf)) {
		spin_lock_bh(&nss_ctx->nss_top->stats_lock);
		nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]++;
		spin_unlock_bh(&nss_ctx->nss_top->stats_lock);
		nss_warning("%p: physical interface %p: command allocation failed", nss_ctx, dev);
		return NSS_TX_FAILURE;
	}

	nim2 = (struct nss_phys_if_msg *)skb_put(nbuf, sizeof(struct nss_phys_if_msg));
	memcpy(nim2, nim, sizeof(struct nss_phys_if_msg));

	status = nss_core_send_buffer(nss_ctx, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		dev_kfree_skb_any(nbuf);
		nss_warning("%p: Unable to enqueue 'physical interface' command\n", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit,
		NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	return NSS_TX_SUCCESS;
}

/*
 **********************************
 Register/Unregister/Miscellaneous APIs
 **********************************
 */

/*
 * nss_phys_if_register()
 */
struct nss_ctx_instance *nss_phys_if_register(uint32_t if_num,
				nss_phys_if_rx_callback_t rx_callback,
				nss_phys_if_msg_callback_t msg_callback,
				struct net_device *if_ctx)
{
	uint8_t id = nss_top_main.phys_if_handler_id[if_num];
	struct nss_ctx_instance *nss_ctx = &nss_top_main.nss[id];

	nss_assert(if_num <= NSS_MAX_PHYSICAL_INTERFACES);

	nss_top_main.if_ctx[if_num] = (void *)if_ctx;
	nss_top_main.if_rx_callback[if_num] = rx_callback;
	nss_top_main.phys_if_msg_callback[if_num] = msg_callback;

	nss_ctx->phys_if_mtu[if_num] = NSS_ETH_NORMAL_FRAME_MTU;
	return nss_ctx;
}

/*
 * nss_phys_if_unregister()
 */
void nss_phys_if_unregister(uint32_t if_num)
{
	nss_assert(if_num < NSS_MAX_PHYSICAL_INTERFACES);
	nss_top_main.if_rx_callback[if_num] = NULL;
	nss_top_main.phys_if_msg_callback[if_num] = NULL;
	nss_top_main.if_ctx[if_num] = NULL;
	nss_top_main.nss[0].phys_if_mtu[if_num] = 0;
	nss_top_main.nss[1].phys_if_mtu[if_num] = 0;
}

/*
 * nss_phys_if_get_napi_ctx()
 *	Get napi context
 */
nss_tx_status_t nss_phys_if_get_napi_ctx(struct nss_ctx_instance *nss_ctx, struct napi_struct **napi_ctx)
{
	nss_info("%p: Get interrupt context, GMAC\n", nss_ctx);

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		return NSS_TX_FAILURE_NOT_READY;
	}

	*napi_ctx = &nss_ctx->int_ctx[0].napi;
	return NSS_TX_SUCCESS;
}

/*
 * nss_phys_if_register_handler()
 */
void nss_phys_if_register_handler(uint32_t if_num)
{
	uint32_t ret;

	ret = nss_core_register_handler(if_num, nss_phys_if_msg_handler, NULL);

	if (ret != NSS_CORE_STATUS_SUCCESS) {
		nss_warning("Message handler FAILED to be registered for interface %d", if_num);
	}
}

EXPORT_SYMBOL(nss_phys_if_tx_msg);
EXPORT_SYMBOL(nss_phys_if_get_napi_ctx);
EXPORT_SYMBOL(nss_phys_if_register);
EXPORT_SYMBOL(nss_phys_if_unregister);
