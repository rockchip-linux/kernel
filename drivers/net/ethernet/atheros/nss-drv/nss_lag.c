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
 * nss_tx_rx_lag.c
 *	NSS LAG Tx APIs
 */

#include <linux/if_bonding.h>

#include "nss_tx_rx_common.h"

/*
 * nss_lag_tx()
 *	Transmit a LAG msg to the firmware.
 */
nss_tx_status_t nss_lag_tx(struct nss_ctx_instance *nss_ctx, struct nss_lag_msg *msg)
{
	struct sk_buff *nbuf;
	int32_t status;
	struct nss_lag_msg *nm;

	nss_info("%p: NSS LAG Tx\n", nss_ctx);

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("%p: LAG msg dropped as core not ready", nss_ctx);
		return NSS_TX_FAILURE_NOT_READY;
	}

	nbuf = dev_alloc_skb(NSS_NBUF_PAYLOAD_SIZE);
	if (unlikely(!nbuf)) {
		spin_lock_bh(&nss_ctx->nss_top->stats_lock);
		nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]++;
		spin_unlock_bh(&nss_ctx->nss_top->stats_lock);
		nss_warning("%p: LAG msg dropped as command allocation failed", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nm = (struct nss_lag_msg *)skb_put(nbuf, sizeof(struct nss_lag_msg));
	memcpy(nm, msg, sizeof(struct nss_lag_msg));

	status = nss_core_send_buffer(nss_ctx, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		dev_kfree_skb_any(nbuf);
		nss_warning("%p: Unable to enqueue LAG msg\n", nss_ctx);
		return NSS_TX_FAILURE;
	}
	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit,
									NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_CMD_REQ]);
	return NSS_TX_SUCCESS;
}
EXPORT_SYMBOL(nss_lag_tx);

/**
 * nss_register_lag_if()
 */
void nss_register_lag_if(uint32_t if_num,
			 nss_lag_callback_t lag_cb,
			 nss_lag_event_callback_t lag_ev_cb,
			 struct net_device *netdev)
{
	nss_assert((if_num == NSS_LAG0_INTERFACE_NUM) || (if_num == NSS_LAG1_INTERFACE_NUM));

	nss_top_main.if_ctx[if_num] = netdev;
	nss_top_main.if_rx_callback[if_num] = lag_cb;
	nss_top_main.lag_event_callback = lag_ev_cb;
}
EXPORT_SYMBOL(nss_register_lag_if);


/**
 * nss_unregister_lag_if()
 */
void nss_unregister_lag_if(uint32_t if_num)
{
	nss_assert((if_num == NSS_LAG0_INTERFACE_NUM) || (if_num == NSS_LAG1_INTERFACE_NUM));

	nss_top_main.if_rx_callback[if_num] = NULL;
	nss_top_main.if_ctx[if_num] = NULL;
	nss_top_main.lag_event_callback = NULL;
}
EXPORT_SYMBOL(nss_unregister_lag_if);


/**
 * nss_lag_handler()
 */
void nss_lag_handler(struct nss_ctx_instance *nss_ctx,
		     struct nss_cmn_msg *ncm,
		     void *app_data)
{
	struct nss_lag_msg *lm = (struct nss_lag_msg *)ncm;
	void *ctx = NULL;
	nss_lag_event_callback_t cb;

	BUG_ON(ncm->interface != NSS_LAG0_INTERFACE_NUM
	       && ncm->interface != NSS_LAG1_INTERFACE_NUM);

	if (ncm->type >= NSS_TX_METADATA_LAG_MAX) {
		nss_warning("%p: received invalid message %d for LAG interface", nss_ctx, ncm->type);
		return;
	}

	if (ncm->len > sizeof(struct nss_lag_msg)) {
		nss_warning("%p: invalid length for LAG message: %d", nss_ctx, ncm->len);
		return;
	}

	/**
	 * Update the callback and app_data for NOTIFY messages.
	 * LAG sends all notify messages to the same callback.
	 */
	if (ncm->response == NSS_CMM_RESPONSE_NOTIFY) {
		ncm->cb = (uint32_t)nss_ctx->nss_top->lag_event_callback;
	}

	/**
	 * Log failures
	 */
	nss_core_log_msg_failures(nss_ctx, ncm);

	/**
	 * Do we have a call back
	 */
	if (!ncm->cb) {
		return;
	}

	/**
	 * callback
	 */
	cb = (nss_lag_event_callback_t)ncm->cb;
	ctx = nss_ctx->nss_top->if_ctx[ncm->interface];

	cb(ctx, lm);
}


/**
 * nss_lag_register_handler()
 */
void nss_lag_register_handler(void)
{
	nss_core_register_handler(NSS_LAG0_INTERFACE_NUM, nss_lag_handler, NULL);
	nss_core_register_handler(NSS_LAG1_INTERFACE_NUM, nss_lag_handler, NULL);
}

