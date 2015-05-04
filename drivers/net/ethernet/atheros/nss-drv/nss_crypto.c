/*
 **************************************************************************
 * Copyright (c) 2013,2015, The Linux Foundation. All rights reserved.
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
 * nss_crypto.c
 *	NSS Crypto APIs
 */

#include "nss_tx_rx_common.h"
#include "nss_crypto.h"

/*
 **********************************
 General APIs
 **********************************
 */

#define nss_crypto_warning(fmt, arg...) nss_warning("Crypto:"fmt, ##arg)
#define nss_crypto_info(fmt, arg...) nss_info("Crypto:"fmt, ##arg)
#define nss_crypto_trace(fmt, arg...) nss_trace("Crypto:"fmt, ##arg)

/*
 * nss_crypto_set_msg_callback()
 * 	this sets the message callback handler and its associated context
 */
static inline void nss_crypto_set_msg_callback(struct nss_ctx_instance *nss_ctx, nss_crypto_msg_callback_t cb, void *crypto_ctx)
{
	struct nss_top_instance *nss_top = nss_ctx->nss_top;

	nss_top->crypto_ctx = crypto_ctx;
	nss_top->crypto_msg_callback = cb;
}

/*
 * nss_crypto_get_msg_callback()
 * 	this gets the message callback handler and its associated context
 */
static inline nss_crypto_msg_callback_t nss_crypto_get_msg_callback(struct nss_ctx_instance *nss_ctx, void **crypto_ctx)
{
	struct nss_top_instance *nss_top = nss_ctx->nss_top;

	*crypto_ctx = nss_top->crypto_ctx;
	return nss_top->crypto_msg_callback;
}

/*
 **********************************
 Rx APIs
 **********************************
 */

/*
 * nss_crypto_buf_handler()
 *	RX packet handler for crypto buf, note the crypto buf is special
 */
void nss_crypto_buf_handler(struct nss_ctx_instance *nss_ctx, void *buf, uint32_t paddr, uint16_t len)
{
	struct nss_top_instance *nss_top = nss_ctx->nss_top;
	void *app_data = nss_top->crypto_ctx;
	nss_crypto_buf_callback_t cb = nss_top->crypto_buf_callback;

	if (unlikely(!cb)) {
		nss_crypto_trace("%p: rx data handler has been unregistered for i/f", nss_ctx);
		return;
	}

	cb(app_data, buf, paddr, len);
}
/*
 * nss_crypto_msg_handler()
 * 	this handles all the IPsec events and responses
 */
static void nss_crypto_msg_handler(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm, void *app_data __attribute((unused)))
{
	struct nss_crypto_msg *nim = (struct nss_crypto_msg *)ncm;
	nss_crypto_msg_callback_t cb = NULL;
	void *crypto_ctx = NULL;

	/*
	 * Sanity check the message type
	 */
	if (ncm->type > NSS_CRYPTO_MSG_TYPE_MAX) {
		nss_crypto_warning("%p: rx message type out of range: %d", nss_ctx, ncm->type);
		return;
	}

	if (ncm->len > sizeof(struct nss_crypto_msg)) {
		nss_crypto_warning("%p: rx request for another interface: %d", nss_ctx, ncm->interface);
		return;
	}

	if (ncm->interface != NSS_CRYPTO_INTERFACE) {
		nss_crypto_warning("%p: rx message request for another interface: %d", nss_ctx, ncm->interface);
		return;
	}

	if (ncm->response == NSS_CMN_RESPONSE_LAST) {
		nss_crypto_warning("%p: rx message response for if %d, type %d, is invalid: %d", nss_ctx, ncm->interface,
				ncm->type, ncm->response);
		return;
	}

	if (ncm->response == NSS_CMM_RESPONSE_NOTIFY) {
		ncm->cb = (uint32_t)nss_crypto_get_msg_callback(nss_ctx, &crypto_ctx);
		ncm->app_data = (uint32_t)crypto_ctx;
	}


	nss_core_log_msg_failures(nss_ctx, ncm);

	/*
	 * Load, Test & call
	 */
	cb = (nss_crypto_msg_callback_t)ncm->cb;
	if (unlikely(!cb)) {
		nss_crypto_trace("%p: rx handler has been unregistered for i/f: %d", nss_ctx, ncm->interface);
		return;
	}
	cb((void *)ncm->app_data, nim);
}
/*
 **********************************
 Tx APIs
 **********************************
 */

/*
 * nss_crypto_tx_msg
 *	Send crypto config to NSS.
 */
nss_tx_status_t nss_crypto_tx_msg(struct nss_ctx_instance *nss_ctx, struct nss_crypto_msg *msg)
{
	struct nss_cmn_msg *ncm = &msg->cm;
	struct nss_crypto_msg *nim;
	struct sk_buff *nbuf;
	int32_t status;

	nss_crypto_info("%p: tx message %d for if %d\n", nss_ctx, ncm->type, ncm->interface);

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_crypto_warning("%p: tx message dropped as core not ready", nss_ctx);
		return NSS_TX_FAILURE_NOT_READY;
	}


	if (NSS_NBUF_PAYLOAD_SIZE < sizeof(struct nss_crypto_msg)) {
		nss_crypto_warning("%p: tx message request is too large: %d (desired), %d (requested)", nss_ctx,
				NSS_NBUF_PAYLOAD_SIZE, sizeof(struct nss_crypto_msg));
		return NSS_TX_FAILURE_TOO_LARGE;
	}

	if (ncm->interface != NSS_CRYPTO_INTERFACE) {
		nss_crypto_warning("%p: tx message request for another interface: %d", nss_ctx, ncm->interface);
	}

	if (ncm->type > NSS_CRYPTO_MSG_TYPE_MAX) {
		nss_crypto_warning("%p: tx message type out of range: %d", nss_ctx, ncm->type);
		return NSS_TX_FAILURE;
	}

	if (ncm->len > sizeof(struct nss_crypto_msg)) {
		nss_crypto_warning("%p: tx message request len for if %d, is bad: %d", nss_ctx, ncm->interface, ncm->len);
		return NSS_TX_FAILURE_BAD_PARAM;
	}

	nbuf = dev_alloc_skb(NSS_NBUF_PAYLOAD_SIZE);
	if (unlikely(!nbuf)) {
		NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]);
		nss_crypto_warning("%p: tx config dropped as command allocation failed", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nss_crypto_info("msg params version:%d, interface:%d, type:%d, cb:%d, app_data:%d, len:%d\n",
			ncm->version, ncm->interface, ncm->type, ncm->cb, ncm->app_data, ncm->len);

	nim = (struct nss_crypto_msg *)skb_put(nbuf, sizeof(struct nss_crypto_msg));
	memcpy(nim, msg, sizeof(struct nss_crypto_msg));

	status = nss_core_send_buffer(nss_ctx, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		dev_kfree_skb_any(nbuf);
		nss_crypto_warning("%p: Unable to enqueue message\n", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit,
				NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	return NSS_TX_SUCCESS;
}

/*
 * nss_crypto_tx_data()
 *	NSS crypto TX data API. Sends a crypto buffer to NSS.
 */
nss_tx_status_t nss_crypto_tx_buf(struct nss_ctx_instance *nss_ctx, void *buf, uint32_t buf_paddr, uint16_t len)
{
	int32_t status;

	nss_crypto_trace("%p: tx_data buf=%p", nss_ctx, buf);

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_crypto_warning("%p: tx_data packet dropped as core not ready", nss_ctx);
		return NSS_TX_FAILURE_NOT_READY;
	}

	status = nss_core_send_crypto(nss_ctx, buf, buf_paddr, len);
	if (unlikely(status != NSS_CORE_STATUS_SUCCESS)) {
		nss_crypto_warning("%p: tx_data Unable to enqueue packet", nss_ctx);
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

	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_CRYPTO_REQ]);

	return NSS_TX_SUCCESS;
}

/*
 **********************************
 Register APIs
 **********************************
 */

/*
 * nss_crypto_notify_register()
 * 	register message notifier for crypto interface
 */
struct nss_ctx_instance *nss_crypto_notify_register(nss_crypto_msg_callback_t cb, void *app_data)
{
	struct nss_ctx_instance *nss_ctx;

	nss_ctx = &nss_top_main.nss[nss_top_main.crypto_handler_id];

	nss_crypto_set_msg_callback(nss_ctx, cb, app_data);

	return nss_ctx;
}

/*
 * nss_crypto_notify_unregister()
 * 	unregister message notifier for crypto interface
 */
void nss_crypto_notify_unregister(struct nss_ctx_instance *nss_ctx)
{
	nss_crypto_set_msg_callback(nss_ctx, NULL, NULL);
}

/*
 * nss_crypto_data_register()
 * 	register a data callback routine
 */
struct nss_ctx_instance *nss_crypto_data_register(nss_crypto_buf_callback_t cb, void *app_data)
{
	struct nss_ctx_instance *nss_ctx;

	nss_ctx = &nss_top_main.nss[nss_top_main.crypto_handler_id];

	nss_ctx->nss_top->crypto_ctx = app_data;
	nss_ctx->nss_top->crypto_buf_callback = cb;

	return nss_ctx;
}

/*
 * nss_crypto_data_unregister()
 * 	unregister a data callback routine
 */
void nss_crypto_data_unregister(struct nss_ctx_instance *nss_ctx)
{
	nss_ctx->nss_top->crypto_ctx = NULL;
	nss_ctx->nss_top->crypto_buf_callback = NULL;
}

/*
 * nss_crypto_register_handler()
 */
void nss_crypto_register_handler()
{
	nss_core_register_handler(NSS_CRYPTO_INTERFACE, nss_crypto_msg_handler, NULL);
}

/*
 * nss_crypto_msg_init()
 *	Initialize crypto message
 */
void nss_crypto_msg_init(struct nss_crypto_msg *ncm, uint16_t if_num, uint32_t type, uint32_t len,
				nss_crypto_msg_callback_t cb, void *app_data)
{
	nss_cmn_msg_init(&ncm->cm, if_num, type, len, (void *)cb, app_data);
}

EXPORT_SYMBOL(nss_crypto_notify_register);
EXPORT_SYMBOL(nss_crypto_notify_unregister);
EXPORT_SYMBOL(nss_crypto_data_register);
EXPORT_SYMBOL(nss_crypto_data_unregister);
EXPORT_SYMBOL(nss_crypto_tx_msg);
EXPORT_SYMBOL(nss_crypto_tx_buf);
EXPORT_SYMBOL(nss_crypto_msg_init);
