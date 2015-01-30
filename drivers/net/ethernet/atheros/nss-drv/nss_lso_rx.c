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
 * nss_lso_rx.c
 *	NSS LSO_RX APIs
 */

#include "nss_tx_rx_common.h"

/*
 * nss_rx_lso_rx_stats_sync()
 *	Handle the syncing of lso_rx node statistics.
 */
static void nss_rx_lso_rx_stats_sync(struct nss_ctx_instance *nss_ctx, struct nss_lso_rx_stats_sync *nlrss)
{
	struct nss_top_instance *nss_top = nss_ctx->nss_top;

	spin_lock_bh(&nss_top->stats_lock);

	/*
	 * common node stats
	 */
	nss_top->stats_node[NSS_LSO_RX_INTERFACE][NSS_STATS_NODE_RX_PKTS] += nlrss->node_stats.rx_packets;
	nss_top->stats_node[NSS_LSO_RX_INTERFACE][NSS_STATS_NODE_RX_BYTES] += nlrss->node_stats.rx_bytes;
	nss_top->stats_node[NSS_LSO_RX_INTERFACE][NSS_STATS_NODE_RX_DROPPED] += nlrss->node_stats.rx_dropped;
	nss_top->stats_node[NSS_LSO_RX_INTERFACE][NSS_STATS_NODE_TX_PKTS] += nlrss->node_stats.tx_packets;
	nss_top->stats_node[NSS_LSO_RX_INTERFACE][NSS_STATS_NODE_TX_BYTES] += nlrss->node_stats.tx_bytes;

	/*
	 * General LSO_RX stats
	 */
	nss_top->stats_lso_rx[NSS_STATS_LSO_RX_TX_DROPPED] += nlrss->tx_dropped;
	nss_top->stats_lso_rx[NSS_STATS_LSO_RX_DROPPED] += nlrss->dropped;

	/*
	 * pbuf
	 */
	nss_top->stats_lso_rx[NSS_STATS_LSO_RX_PBUF_ALLOC_FAIL] += nlrss->pbuf_alloc_fail;
	nss_top->stats_lso_rx[NSS_STATS_LSO_RX_PBUF_REFERENCE_FAIL] += nlrss->pbuf_reference_fail;

	spin_unlock_bh(&nss_top->stats_lock);
}

/*
 * nss_rx_lso_rx_interface_handler()
 *	Handle NSS -> HLOS messages for LSO_RX Changes and Statistics
 */
static void nss_rx_lso_rx_interface_handler(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm, __attribute__((unused))void *app_data) {

	struct nss_lso_rx_msg *nlrm = (struct nss_lso_rx_msg *)ncm;

	switch (nlrm->cm.type) {
	case NSS_LSO_RX_STATS_SYNC_MSG:
		nss_rx_lso_rx_stats_sync(nss_ctx, &nlrm->msg.stats_sync);
		break;

	default:
		if (ncm->response != NSS_CMN_RESPONSE_ACK) {
			/*
			 * Check response
			 */
			nss_info("%p: Received response %d for type %d, interface %d", nss_ctx, ncm->response, ncm->type, ncm->interface);
		}
	}
}

/*
 * nss_lso_rx_register_handler()
 *	Register handler for messaging
 */
void nss_lso_rx_register_handler(void)
{
	nss_core_register_handler(NSS_LSO_RX_INTERFACE, nss_rx_lso_rx_interface_handler, NULL);
}
