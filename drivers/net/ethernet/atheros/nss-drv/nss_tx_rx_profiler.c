/*
 **************************************************************************
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
 * nss_tx_rx_profiler.c
 *	NSS profiler APIs
 */

#include "nss_tx_rx_common.h"

/*
 * nss_tx_profiler_if_buf()
 *	NSS profiler Tx API
 */
nss_tx_status_t nss_tx_profiler_if_buf(void *ctx, uint8_t *buf, uint32_t len)
{
	struct nss_ctx_instance *nss_ctx = (struct nss_ctx_instance *)ctx;
	struct sk_buff *nbuf;
	int32_t status;
	struct nss_tx_metadata_object *ntmo;
	struct nss_profiler_tx *npt;

	nss_trace("%p: Profiler If Tx, buf=%p", nss_ctx, buf);

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("%p: 'Profiler If Tx' rule dropped as core not ready", nss_ctx);
		return NSS_TX_FAILURE_NOT_READY;
	}

	if (NSS_NBUF_PAYLOAD_SIZE < (len + sizeof(uint32_t) + sizeof(struct nss_profiler_tx))) {
		return NSS_TX_FAILURE_TOO_LARGE;
	}

	nbuf = dev_alloc_skb(NSS_NBUF_PAYLOAD_SIZE);
	if (unlikely(!nbuf)) {
		spin_lock_bh(&nss_ctx->nss_top->stats_lock);
		nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]++;
		spin_unlock_bh(&nss_ctx->nss_top->stats_lock);
		nss_warning("%p: 'Profiler If Tx' rule dropped as command allocation failed", nss_ctx);
		return NSS_TX_FAILURE;
	}

	ntmo = (struct nss_tx_metadata_object *)skb_put(nbuf, sizeof(struct nss_tx_metadata_object));
	ntmo->type = NSS_TX_METADATA_TYPE_PROFILER_TX;

	npt = &ntmo->sub.profiler_tx;
	npt->len = len;
	memcpy(npt->buf, buf, len);

	status = nss_core_send_buffer(nss_ctx, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		dev_kfree_skb_any(nbuf);
		nss_warning("%p: Unable to enqueue 'Profiler If Tx' rule\n", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit,
								NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_CMD_REQ]);
	return NSS_TX_SUCCESS;
}


/*
 * nss_rx_metadata_profiler_sync()
 *	Handle the syncing of profiler information.
 */
static void nss_rx_metadata_profiler_sync(struct nss_ctx_instance *nss_ctx, struct nss_profiler_sync *profiler_sync)
{
	void *ctx = nss_ctx->nss_top->profiler_ctx[nss_ctx->id];
	nss_profiler_callback_t cb = nss_ctx->nss_top->profiler_callback[nss_ctx->id];

	if (!cb || !ctx) {
		nss_warning("%p: Event received for profiler interface before registration", nss_ctx);
	}

	cb(ctx, profiler_sync->buf, profiler_sync->len);
}

/*
 * nss_register_profiler_if()
 */
void *nss_register_profiler_if(nss_profiler_callback_t profiler_callback, nss_core_id_t core_id, void *ctx)
{
	nss_assert(core_id < NSS_CORE_MAX);

	nss_top_main.profiler_ctx[core_id] = ctx;
	nss_top_main.profiler_callback[core_id] = profiler_callback;

	return (void *)&nss_top_main.nss[core_id];
}

/*
 * nss_unregister_profiler_if()
 */
void nss_unregister_profiler_if(nss_core_id_t core_id)
{
	nss_assert(core_id < NSS_CORE_MAX);

	nss_top_main.profiler_callback[core_id] = NULL;
	nss_top_main.profiler_ctx[core_id] = NULL;
}

EXPORT_SYMBOL(nss_register_profiler_if);
EXPORT_SYMBOL(nss_unregister_profiler_if);
EXPORT_SYMBOL(nss_tx_profiler_if_buf);

