/*
 **************************************************************************
 * Copyright (c) 2013 - 2014, The Linux Foundation. All rights reserved.
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
 * nss_ipsec.c
 *	NSS IPsec APIs
 */

#include "nss_tx_rx_common.h"
#include "nss_ipsec.h"

/*
 **********************************
 General APIs
 **********************************
 */

#define nss_ipsec_warning(fmt, arg...) nss_warning("IPsec:"fmt, ##arg)
#define nss_ipsec_info(fmt, arg...) nss_info("IPsec:"fmt, ##arg)
#define nss_ipsec_trace(fmt, arg...) nss_trace("IPsec:"fmt, ##arg)

/*
 * nss_ipsec_set_msg_callback()
 * 	this sets the message callback handler and its associated context
 */
static inline nss_tx_status_t nss_ipsec_set_msg_callback(struct nss_ctx_instance *nss_ctx, uint32_t if_num,
							nss_ipsec_msg_callback_t cb, void *ipsec_ctx)
{
	struct nss_top_instance *nss_top;

	nss_top = nss_ctx->nss_top;

	switch (if_num) {
	case NSS_IPSEC_ENCAP_IF_NUMBER:
		nss_top->ipsec_encap_ctx = ipsec_ctx;
		nss_top->ipsec_encap_callback = cb;

	case NSS_IPSEC_DECAP_IF_NUMBER:
		nss_top->ipsec_decap_ctx = ipsec_ctx;
		nss_top->ipsec_decap_callback = cb;

	default:
		nss_ipsec_warning("%p: cannot 'set' message callback, incorrect I/F: %d", nss_ctx, if_num);
		return NSS_TX_FAILURE;
	}

	return NSS_TX_SUCCESS;
}

/*
 * nss_ipsec_get_msg_callback()
 * 	this gets the message callback handler and its associated context
 */
static inline nss_ipsec_msg_callback_t nss_ipsec_get_msg_callback(struct nss_ctx_instance *nss_ctx, uint32_t if_num, void **ipsec_ctx)
{
	struct nss_top_instance *nss_top;

	nss_top = nss_ctx->nss_top;

	switch (if_num) {
	case NSS_IPSEC_ENCAP_IF_NUMBER:
		*ipsec_ctx = nss_top->ipsec_encap_ctx;
		return nss_top->ipsec_encap_callback;

	case NSS_IPSEC_DECAP_IF_NUMBER:
		*ipsec_ctx = nss_top->ipsec_decap_ctx;
		return nss_top->ipsec_decap_callback;

	default:
		*ipsec_ctx = NULL;
		nss_ipsec_warning("%p: cannot 'get' message callback, incorrect I/F: %d", nss_ctx, if_num);
		return NULL;
	}
}

/*
 **********************************
 Rx APIs
 **********************************
 */

/*
 * nss_ipsec_msg_handler()
 * 	this handles all the IPsec events and responses
 */
static void nss_ipsec_msg_handler(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm, void *app_data __attribute((unused)))
{
	struct nss_ipsec_msg *nim = (struct nss_ipsec_msg *)ncm;
	nss_ipsec_msg_callback_t cb = NULL;
	uint32_t if_num = ncm->interface;
	void *ipsec_ctx;

	/*
	 * Sanity check the message type
	 */
	if (ncm->type > NSS_IPSEC_MSG_TYPE_MAX) {
		nss_ipsec_warning("%p: rx message type out of range: %d", nss_ctx, ncm->type);
		return;
	}

	if (ncm->len > sizeof(struct nss_ipsec_msg)) {
		nss_ipsec_warning("%p: rx request for another interface: %d", nss_ctx, ncm->interface);
		return;
	}

	if ((ncm->interface != NSS_IPSEC_ENCAP_IF_NUMBER) && (ncm->interface != NSS_IPSEC_DECAP_IF_NUMBER)) {
		nss_ipsec_warning("%p: rx message request for another interface: %d", nss_ctx, ncm->interface);
		return;
	}

	if (ncm->response == NSS_CMN_RESPONSE_LAST) {
		nss_ipsec_warning("%p: rx message response for if %d, type %d, is invalid: %d", nss_ctx, ncm->interface,
				ncm->type, ncm->response);
		return;
	}

	/*
	 * Is this a notification? if, yes then fill up the callback and app_data from
	 * locally stored state
	 */
	if (ncm->response == NSS_CMM_RESPONSE_NOTIFY) {
		ncm->cb = (uint32_t)nss_ipsec_get_msg_callback(nss_ctx, if_num, &ipsec_ctx);
		ncm->app_data = (uint32_t)ipsec_ctx;
	}


	nss_core_log_msg_failures(nss_ctx, ncm);

	/*
	 * load, test & call
	 */
	cb = (nss_ipsec_msg_callback_t)ncm->cb;
	if (unlikely(!cb)) {
		nss_ipsec_trace("%p: rx handler has been unregistered for i/f: %d", nss_ctx, ncm->interface);
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
 * nss_ipsec_tx_msg
 *	Send ipsec rule to NSS.
 */
nss_tx_status_t nss_ipsec_tx_msg(struct nss_ctx_instance *nss_ctx, struct nss_ipsec_msg *msg)
{
	struct nss_cmn_msg *ncm = &msg->cm;
	struct nss_ipsec_msg *nim;
	struct sk_buff *nbuf;
	int32_t status;

	nss_ipsec_info("%p: message %d for if %d\n", nss_ctx, ncm->type, ncm->interface);

	NSS_VERIFY_CTX_MAGIC(nss_ctx);

	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_ipsec_warning("%p: tx message dropped as core not ready", nss_ctx);
		return NSS_TX_FAILURE_NOT_READY;
	}

	if (NSS_NBUF_PAYLOAD_SIZE < sizeof(struct nss_ipsec_msg)) {
		nss_ipsec_warning("%p: tx message request is too large: %d (desired), %d (requested)", nss_ctx,
				NSS_NBUF_PAYLOAD_SIZE, sizeof(struct nss_ipsec_msg));
		return NSS_TX_FAILURE_TOO_LARGE;
	}

	if ((ncm->interface != NSS_IPSEC_ENCAP_IF_NUMBER) && (ncm->interface != NSS_IPSEC_DECAP_IF_NUMBER)) {
		nss_ipsec_warning("%p: tx message request for another interface: %d", nss_ctx, ncm->interface);
		return NSS_TX_FAILURE;
	}

	if (ncm->type > NSS_IPSEC_MSG_TYPE_MAX) {
		nss_ipsec_warning("%p: tx message type out of range: %d", nss_ctx, ncm->type);
		return NSS_TX_FAILURE;
	}

	if (ncm->len > sizeof(struct nss_ipsec_msg)) {
		nss_ipsec_warning("%p: tx message request len for if %d, is bad: %d", nss_ctx, ncm->interface, ncm->len);
		return NSS_TX_FAILURE_BAD_PARAM;
	}

	nbuf = dev_alloc_skb(NSS_NBUF_PAYLOAD_SIZE);
	if (unlikely(!nbuf)) {
		spin_lock_bh(&nss_ctx->nss_top->stats_lock);
		nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]++;
		spin_unlock_bh(&nss_ctx->nss_top->stats_lock);
		nss_ipsec_warning("%p: tx rule dropped as command allocation failed", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nim = (struct nss_ipsec_msg *)skb_put(nbuf, sizeof(struct nss_ipsec_msg));
	memcpy(nim, msg, sizeof(struct nss_ipsec_msg));

	status = nss_core_send_buffer(nss_ctx, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		dev_kfree_skb_any(nbuf);
		nss_ipsec_warning("%p: tx Unable to enqueue message \n", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit,
									NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	return NSS_TX_SUCCESS;
}

/*
 **********************************
 Register APIs
 **********************************
 */

/*
 * nss_ipsec_msg_notify_register()
 * 	register message notifier for the given interface (if_num)
 */
struct nss_ctx_instance *nss_ipsec_notify_register(uint32_t if_num, nss_ipsec_msg_callback_t cb, void *app_data)
{
	struct nss_ctx_instance *nss_ctx;

	nss_ctx = &nss_top_main.nss[nss_top_main.ipsec_handler_id];

	if (if_num >= NSS_MAX_NET_INTERFACES) {
		nss_ipsec_warning("%p: notfiy register received for invalid interface %d", nss_ctx, if_num);
		return NULL;
	}

	if (nss_ipsec_set_msg_callback(nss_ctx, if_num, cb, app_data) != NSS_TX_SUCCESS) {
		nss_ipsec_warning("%p: register failed\n", nss_ctx);
		return NULL;
	}

	return nss_ctx;
}

/*
 * nss_ipsec_msg_notify_unregister()
 * 	unregister the IPsec notifier for the given interface number (if_num)
 */
void nss_ipsec_notify_unregister(struct nss_ctx_instance *nss_ctx, uint32_t if_num)
{
	if (if_num >= NSS_MAX_NET_INTERFACES) {
		nss_ipsec_warning("%p: notify unregister received for invalid interface %d", nss_ctx, if_num);
		return;
	}

	if (nss_ipsec_set_msg_callback(nss_ctx, if_num, NULL, NULL) != NSS_TX_SUCCESS) {
		nss_ipsec_warning("%p: unregister failed\n", nss_ctx);
		return;
	}
}

/*
 * nss_ipsec_data_notify_register()
 * 	register a data callback routine
 */
struct nss_ctx_instance *nss_ipsec_data_register(uint32_t if_num, nss_ipsec_buf_callback_t cb, void *app_data)
{
	struct nss_ctx_instance *nss_ctx;

	nss_ctx = &nss_top_main.nss[nss_top_main.ipsec_handler_id];

	if ((if_num >= NSS_MAX_NET_INTERFACES) && (if_num < NSS_MAX_PHYSICAL_INTERFACES)){
		nss_ipsec_warning("%p: data register received for invalid interface %d", nss_ctx, if_num);
		return NULL;
	}

	nss_ctx->nss_top->if_ctx[if_num] = app_data;
	nss_ctx->nss_top->if_rx_callback[if_num] = cb;

	return nss_ctx;
}

/*
 * nss_ipsec_data_notify_unregister()
 * 	unregister a data callback routine
 */
void nss_ipsec_data_unregister(struct nss_ctx_instance *nss_ctx, uint32_t if_num)
{
	if ((if_num >= NSS_MAX_NET_INTERFACES) && (if_num < NSS_MAX_PHYSICAL_INTERFACES)){
		nss_ipsec_warning("%p: data unregister received for invalid interface %d", nss_ctx, if_num);
		return;
	}

	nss_ctx->nss_top->if_ctx[if_num] = NULL;
	nss_ctx->nss_top->if_rx_callback[if_num] = NULL;
}

/*
 * nss_ipsec_register_handler()
 */
void nss_ipsec_register_handler()
{
	struct nss_ctx_instance *nss_ctx = &nss_top_main.nss[nss_top_main.ipsec_handler_id];

	nss_ipsec_set_msg_callback(nss_ctx, NSS_IPSEC_ENCAP_IF_NUMBER, NULL, NULL);
	nss_core_register_handler(NSS_IPSEC_ENCAP_IF_NUMBER, nss_ipsec_msg_handler, NULL);

	nss_ipsec_set_msg_callback(nss_ctx, NSS_IPSEC_DECAP_IF_NUMBER, NULL, NULL);
	nss_core_register_handler(NSS_IPSEC_DECAP_IF_NUMBER, nss_ipsec_msg_handler, NULL);
}

EXPORT_SYMBOL(nss_ipsec_notify_register);
EXPORT_SYMBOL(nss_ipsec_notify_unregister);
EXPORT_SYMBOL(nss_ipsec_data_register);
EXPORT_SYMBOL(nss_ipsec_data_unregister);
EXPORT_SYMBOL(nss_ipsec_tx_msg);
