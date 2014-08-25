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
 * nss_tx_rx_n2h.c
 *	NSS N2H node APIs
 */

#include "nss_tx_rx_common.h"

/*
 **********************************
 Rx APIs
 **********************************
 */

/*
 * nss_rx_metadata_n2h_stats_sync()
 *	Handle the syncing of NSS statistics.
 */
static void nss_rx_metadata_n2h_stats_sync(struct nss_ctx_instance *nss_ctx, struct nss_n2h_stats_sync *nnss)
{
	struct nss_top_instance *nss_top = nss_ctx->nss_top;

	spin_lock_bh(&nss_top->stats_lock);

	/*
	 * common node stats
	 */
	nss_ctx->stats_n2h[NSS_STATS_NODE_RX_PKTS] += nnss->node_stats.rx_packets;
	nss_ctx->stats_n2h[NSS_STATS_NODE_RX_BYTES] += nnss->node_stats.rx_bytes;
	nss_ctx->stats_n2h[NSS_STATS_NODE_RX_DROPPED] += nnss->node_stats.rx_dropped;
	nss_ctx->stats_n2h[NSS_STATS_NODE_TX_PKTS] += nnss->node_stats.tx_packets;
	nss_ctx->stats_n2h[NSS_STATS_NODE_TX_BYTES] += nnss->node_stats.tx_bytes;

	/*
	 * General N2H stats
	 */
	nss_ctx->stats_n2h[NSS_STATS_N2H_QUEUE_DROPPED] += nnss->queue_dropped;
	nss_ctx->stats_n2h[NSS_STATS_N2H_TOTAL_TICKS] += nnss->total_ticks;
	nss_ctx->stats_n2h[NSS_STATS_N2H_WORST_CASE_TICKS] += nnss->worst_case_ticks;
	nss_ctx->stats_n2h[NSS_STATS_N2H_ITERATIONS] += nnss->iterations;

	/*
	 * pbuf/payload mgr stats
	 */
	nss_ctx->stats_n2h[NSS_STATS_N2H_PBUF_ALLOC_FAILS] += nnss->pbuf_alloc_fails;
	nss_ctx->stats_n2h[NSS_STATS_N2H_PAYLOAD_ALLOC_FAILS] += nnss->payload_alloc_fails;

	spin_unlock_bh(&nss_top->stats_lock);
}

/*
 * nss_rx_n2h_interface_handler()
 *	Handle NSS -> HLOS messages for N2H node
 */
static void nss_rx_n2h_interface_handler(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm, __attribute__((unused))void *app_data)
{
	struct nss_n2h_msg *nnm = (struct nss_n2h_msg *)ncm;

	/*
	 * Is this a valid request/response packet?
	 */
	if (nnm->cm.type >= NSS_METADATA_TYPE_N2H_MAX) {
		nss_warning("%p: received invalid message %d for Offload stats interface", nss_ctx, nnm->cm.type);
		return;
	}

	switch (nnm->cm.type) {
	case NSS_RX_METADATA_TYPE_N2H_STATS_SYNC:
		nss_rx_metadata_n2h_stats_sync(nss_ctx, &nnm->msg.stats_sync);
		break;

	default:
		if (ncm->response != NSS_CMN_RESPONSE_ACK) {
			/*
			 * Check response
			 */
			nss_info("%p: Received response %d for type %d, interface %d",
						nss_ctx, ncm->response, ncm->type, ncm->interface);
		}
	}
}

/*
 * nss_n2h_register_handler()
 */
void nss_n2h_register_handler()
{
	nss_core_register_handler(NSS_N2H_INTERFACE, nss_rx_n2h_interface_handler, NULL);
}
