/*
 **************************************************************************
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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
 * nss_profiler.c
 *	NSS profiler APIs
 */

#include "nss_tx_rx_common.h"

/*
 * nss_profiler_rx_msg_handler()
 *	Handle profiler information.
 */
static void nss_profiler_rx_msg_handler(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm, void *app)
{
	struct nss_profiler_msg *pm = (struct nss_profiler_msg*)ncm;
	void *ctx = nss_ctx->nss_top->profiler_ctx[nss_ctx->id];
	nss_profiler_callback_t cb = nss_ctx->nss_top->profiler_callback[nss_ctx->id];

	if (ncm->type >= NSS_PROFILER_MAX_MSG_TYPES) {
		nss_warning("%p: message type out of range: %d", nss_ctx, ncm->type);
		return;
	}

	if (ncm->type <= NSS_PROFILER_FLOWCTRL_MSG) {
		if (ncm->len > sizeof(pm->payload.pcmdp)) {
			nss_warning("%p: reply for cmd %d size is wrong %d : %d\n", nss_ctx, ncm->type, ncm->len, ncm->interface);
			return;
		}
	} else if (ncm->type <= NSS_PROFILER_DEBUG_REPLY_MSG) {
		if (ncm->len > sizeof(pm->payload.pdm)) {
			nss_warning("%p: reply for debug %d is too big %d\n", nss_ctx, ncm->type, ncm->len);
			return;
		}
	} else if (ncm->type <= NSS_PROFILER_COUNTERS_MSG) {
		if (ncm->len < (sizeof(pm->payload.pcmdp) - (PROFILE_MAX_APP_COUNTERS - pm->payload.pcmdp.num_counters) * sizeof(pm->payload.pcmdp.counters[0])) || ncm->len > sizeof(pm->payload.pcmdp)) {
			nss_warning("%p: %d params data is too big %d : %d\n", nss_ctx, ncm->type, ncm->len, ncm->interface);
			return;
		}
	}

	/*
	 * status per request callback
	 */
	if (ncm->response != NSS_CMM_RESPONSE_NOTIFY && ncm->cb) {
		nss_info("%p: reply CB %x for %d %d\n", nss_ctx, ncm->cb, ncm->type, ncm->response);
		cb = (nss_profiler_callback_t)ncm->cb;
	}

	/*
	 * sample related callback
	 */
	if (!cb || !ctx) {
		nss_warning("%p: Event received for profiler interface before registration", nss_ctx);
		return;
	}

	cb(ctx, (struct nss_profiler_msg *)ncm);
}

/*
 * nss_tx_profiler_if_buf()
 *	NSS profiler Tx API
 */
nss_tx_status_t nss_profiler_if_tx_buf(void *ctx, void *buf, uint32_t len, void *cb)
{
	struct nss_ctx_instance *nss_ctx = (struct nss_ctx_instance *)ctx;
	struct sk_buff *nbuf;
	int32_t status;
	struct nss_profiler_msg *npm;
	struct nss_profiler_data_msg *pdm = (struct nss_profiler_data_msg *)buf;

	nss_trace("%p: Profiler If Tx, buf=%p", nss_ctx, buf);

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("%p: 'Profiler If Tx' rule dropped as core not ready", nss_ctx);
		return NSS_TX_FAILURE_NOT_READY;
	}

	if (NSS_NBUF_PAYLOAD_SIZE < (len + sizeof(*npm))) {
		return NSS_TX_FAILURE_TOO_LARGE;
	}

	nbuf = dev_alloc_skb(NSS_NBUF_PAYLOAD_SIZE);
	if (unlikely(!nbuf)) {
		NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]);
		nss_warning("%p: 'Profiler If Tx' rule dropped as command allocation failed", nss_ctx);
		return NSS_TX_FAILURE;
	}

	npm = (struct nss_profiler_msg *)skb_put(nbuf, sizeof(npm->cm) + len);
	nss_profiler_msg_init(npm, NSS_PROFILER_INTERFACE, pdm->hd_magic & 0xFF, len,
				cb, ctx);
	memcpy(&npm->payload, pdm, len);

	status = nss_core_send_buffer(nss_ctx, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		dev_kfree_skb_any(nbuf);
		nss_warning("%p: Unable to enqueue 'Profiler If cmd Tx\n", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit,
								NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_CMD_REQ]);
	return NSS_TX_SUCCESS;
}

/*
 * nss_profiler_notify_register()
 */
void *nss_profiler_notify_register(nss_core_id_t core_id, nss_profiler_callback_t profiler_callback, void *ctx)
{
	nss_assert(core_id < NSS_CORE_MAX);

	if ((core_id == NSS_CORE_0) && (NSS_CORE_STATUS_SUCCESS !=
		nss_core_register_handler(NSS_PROFILER_INTERFACE, nss_profiler_rx_msg_handler, NULL))) {
			nss_warning("Message handler FAILED to be registered for profiler");
			return NULL;
	}

	nss_top_main.profiler_ctx[core_id] = ctx;
	nss_top_main.profiler_callback[core_id] = profiler_callback;

	return (void *)&nss_top_main.nss[core_id];
}

/*
 * nss_profiler_notify_unregister()
 */
void nss_profiler_notify_unregister(nss_core_id_t core_id)
{
	nss_assert(core_id < NSS_CORE_MAX);

	nss_core_register_handler(NSS_PROFILER_INTERFACE, NULL, NULL);
	nss_top_main.profiler_callback[core_id] = NULL;
	nss_top_main.profiler_ctx[core_id] = NULL;
}

/*
 * nss_profiler_msg_init()
 *      Initialize profiler message.
 */
void nss_profiler_msg_init(struct nss_profiler_msg *npm, uint16_t if_num, uint32_t type, uint32_t len,
				nss_profiler_callback_t cb, void *app_data)
{
	nss_cmn_msg_init(&npm->cm, if_num, type, len, (void *)cb, app_data);
}

EXPORT_SYMBOL(nss_profiler_notify_register);
EXPORT_SYMBOL(nss_profiler_notify_unregister);
EXPORT_SYMBOL(nss_profiler_if_tx_buf);
EXPORT_SYMBOL(nss_profiler_msg_init);
