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
 * nss_if.c
 *	NSS base interfaces
 */

#include "nss_tx_rx_common.h"

/*
 * nss_if_msg_handler()
 *	Handle NSS -> HLOS messages for base class interfaces
 */
void nss_if_msg_handler(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm,
		__attribute__((unused))void *app_data)
{
	struct nss_if_msg *nim = (struct nss_if_msg *)ncm;
	nss_if_msg_callback_t cb;

	/*
	 * We only support base class messages with this interface
	 */
	if (ncm->type > NSS_IF_MAX_MSG_TYPES) {
		nss_warning("%p: message type out of range: %d", nss_ctx, ncm->type);
		return;
	}

	/*
	 * As the base class we allow both virtual and physical interfaces.
	 */
	if (ncm->interface > NSS_TUNNEL_IF_START) {
		nss_warning("%p: response for another interface: %d", nss_ctx, ncm->interface);
		return;
	}

	if (ncm->len > sizeof(struct nss_if_msg)) {
		nss_warning("%p: message length too big: %d", nss_ctx, ncm->len);
		return;
	}

	/*
	 * Log failures
	 */
	nss_core_log_msg_failures(nss_ctx, ncm);

	/*
	 * Do we have a callback?
	 */
	if (!ncm->cb) {
		return;
	}

	/*
	 * Callback
	 */
	cb = (nss_if_msg_callback_t)ncm->cb;
	cb((void *)ncm->app_data, nim);
}

/*
 * nss_if_tx_buf()
 *	Send packet to interface owned by NSS
 */
nss_tx_status_t nss_if_tx_buf(struct nss_ctx_instance *nss_ctx, struct sk_buff *os_buf, uint32_t if_num)
{
	int32_t status;

	nss_trace("%p: If Tx packet, id:%d, data=%p", nss_ctx, if_num, os_buf->data);

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
 * nss_if_tx_msg()
 *	Transmit a message to the specific interface on this core.
 */
nss_tx_status_t nss_if_tx_msg(struct nss_ctx_instance *nss_ctx, struct nss_if_msg *nim)
{
	struct nss_cmn_msg *ncm = &nim->cm;
	struct nss_if_msg *nim2;
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
	/*
	 * As the base class we allow both virtual and physical interfaces.
	 */
	if (ncm->interface > NSS_TUNNEL_IF_START) {
		nss_warning("%p: tx request for another interface: %d", nss_ctx, ncm->interface);
		return NSS_TX_FAILURE;
	}

	if (ncm->type > NSS_IF_MAX_MSG_TYPES) {
		nss_warning("%p: message type out of range: %d", nss_ctx, ncm->type);
		return NSS_TX_FAILURE;
	}

	if (ncm->len > sizeof(struct nss_if_msg)) {
		nss_warning("%p: invalid length: %d", nss_ctx, ncm->len);
		return NSS_TX_FAILURE;
	}

	if_num = ncm->interface;
	dev = nss_top_main.if_ctx[if_num];
	if (!dev) {
		nss_warning("%p: Unregister interface %d: no context", nss_ctx, if_num);
		return NSS_TX_FAILURE_BAD_PARAM;
	}

	nbuf = dev_alloc_skb(NSS_NBUF_PAYLOAD_SIZE);
	if (unlikely(!nbuf)) {
		spin_lock_bh(&nss_ctx->nss_top->stats_lock);
		nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]++;
		spin_unlock_bh(&nss_ctx->nss_top->stats_lock);
		nss_warning("%p: interface %p: command allocation failed", nss_ctx, dev);
		return NSS_TX_FAILURE;
	}

	nim2 = (struct nss_if_msg *)skb_put(nbuf, sizeof(struct nss_if_msg));
	memcpy(nim2, nim, sizeof(struct nss_if_msg));

	status = nss_core_send_buffer(nss_ctx, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		dev_kfree_skb_any(nbuf);
		nss_warning("%p: Unable to enqueue 'interface' command\n", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit,
					NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);
	return NSS_TX_SUCCESS;
}

/*
 * nss_if_register()
 *	Primary registration for receiving data and msgs from an interface.
 */
struct nss_ctx_instance *nss_if_register(uint32_t if_num,
				nss_if_rx_callback_t rx_callback,
				nss_if_msg_callback_t msg_callback,
				struct net_device *if_ctx)
{
	return NULL;
}

/*
 * nss_if_unregister()
 *	Unregisteer the callback for this interface
 */
void nss_if_unregister(uint32_t if_num)
{
}

EXPORT_SYMBOL(nss_if_tx_msg);
EXPORT_SYMBOL(nss_if_register);
EXPORT_SYMBOL(nss_if_unregister);
