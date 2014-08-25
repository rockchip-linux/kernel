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
 * nss_tx_rx_offload.c
 *	NSS offload stats APIs
 */

#include "nss_tx_rx_common.h"

/*
 **********************************
 Rx APIs
 **********************************
 */

/*
 * nss_rx_metadata_interface_stats_sync()
 *	Handle the syncing of interface statistics.
 */
static void nss_rx_metadata_interface_stats_sync(struct nss_ctx_instance *nss_ctx, struct nss_per_if_stats_sync *niss, uint32_t id)
{
	struct nss_top_instance *nss_top = nss_ctx->nss_top;
	uint32_t i;

	spin_lock_bh(&nss_top->stats_lock);

	nss_top->stats_if_host[id][NSS_STATS_IF_HOST_RX_PKTS] += niss->host_rx_packets;
	nss_top->stats_if_host[id][NSS_STATS_IF_HOST_RX_BYTES] += niss->host_rx_bytes;
	nss_top->stats_if_host[id][NSS_STATS_IF_HOST_TX_PKTS] += niss->host_tx_packets;
	nss_top->stats_if_host[id][NSS_STATS_IF_HOST_TX_BYTES] += niss->host_tx_bytes;

	for (i = 0; i < NSS_EXCEPTION_EVENT_UNKNOWN_MAX; i++) {
		nss_top->stats_if_exception_unknown[id][i] += niss->exception_events_unknown[i];
	}

	for (i = 0; i < NSS_EXCEPTION_EVENT_IPV4_MAX; i++) {
		nss_top->stats_if_exception_ipv4[id][i] += niss->exception_events_ipv4[i];
	}

	for (i = 0; i < NSS_EXCEPTION_EVENT_IPV6_MAX; i++) {
		nss_top->stats_if_exception_ipv6[id][i] += niss->exception_events_ipv6[i];
	}

	spin_unlock_bh(&nss_top->stats_lock);
}

/*
 * nss_rx_metadata_offload_stats_sync()
 *	Handle the syncing of NSS statistics.
 */
static void nss_rx_metadata_offload_stats_sync(struct nss_ctx_instance *nss_ctx, struct nss_offload_stats_sync *noss)
{
	struct nss_top_instance *nss_top = nss_ctx->nss_top;

	spin_lock_bh(&nss_top->stats_lock);

	/*
	 * IPv4 stats
	 */
	nss_top->stats_ipv4[NSS_STATS_IPV4_CONNECTION_CREATE_REQUESTS] += noss->ipv4_connection_create_requests;
	nss_top->stats_ipv4[NSS_STATS_IPV4_CONNECTION_CREATE_COLLISIONS] += noss->ipv4_connection_create_collisions;
	nss_top->stats_ipv4[NSS_STATS_IPV4_CONNECTION_CREATE_INVALID_INTERFACE] += noss->ipv4_connection_create_invalid_interface;
	nss_top->stats_ipv4[NSS_STATS_IPV4_CONNECTION_DESTROY_REQUESTS] += noss->ipv4_connection_destroy_requests;
	nss_top->stats_ipv4[NSS_STATS_IPV4_CONNECTION_DESTROY_MISSES] += noss->ipv4_connection_destroy_misses;
	nss_top->stats_ipv4[NSS_STATS_IPV4_CONNECTION_HASH_HITS] += noss->ipv4_connection_hash_hits;
	nss_top->stats_ipv4[NSS_STATS_IPV4_CONNECTION_HASH_REORDERS] += noss->ipv4_connection_hash_reorders;
	nss_top->stats_ipv4[NSS_STATS_IPV4_CONNECTION_FLUSHES] += noss->ipv4_connection_flushes;
	nss_top->stats_ipv4[NSS_STATS_IPV4_CONNECTION_EVICTIONS] += noss->ipv4_connection_evictions;

	/*
	 * IPv6 stats
	 */
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_CREATE_REQUESTS] += noss->ipv6_connection_create_requests;
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_CREATE_COLLISIONS] += noss->ipv6_connection_create_collisions;
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_CREATE_INVALID_INTERFACE] += noss->ipv6_connection_create_invalid_interface;
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_DESTROY_REQUESTS] += noss->ipv6_connection_destroy_requests;
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_DESTROY_MISSES] += noss->ipv6_connection_destroy_misses;
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_HASH_HITS] += noss->ipv6_connection_hash_hits;
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_HASH_REORDERS] += noss->ipv6_connection_hash_reorders;
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_FLUSHES] += noss->ipv6_connection_flushes;
	nss_top->stats_ipv6[NSS_STATS_IPV6_CONNECTION_EVICTIONS] += noss->ipv6_connection_evictions;

	/*
	 * pppoe stats
	 */
	nss_top->stats_pppoe[NSS_STATS_PPPOE_SESSION_CREATE_REQUESTS] += noss->pppoe_session_create_requests;
	nss_top->stats_pppoe[NSS_STATS_PPPOE_SESSION_CREATE_FAILURES] += noss->pppoe_session_create_failures;
	nss_top->stats_pppoe[NSS_STATS_PPPOE_SESSION_DESTROY_REQUESTS] += noss->pppoe_session_destroy_requests;
	nss_top->stats_pppoe[NSS_STATS_PPPOE_SESSION_DESTROY_MISSES] += noss->pppoe_session_destroy_misses;

	/*
	 * n2h stats
	 */
	nss_top->stats_n2h[NSS_STATS_N2H_QUEUE_DROPPED] += noss->except_queue_dropped;
	nss_top->stats_n2h[NSS_STATS_N2H_TOTAL_TICKS] += noss->except_total_ticks;
	if (unlikely(nss_top->stats_n2h[NSS_STATS_N2H_WORST_CASE_TICKS] < noss->except_worst_case_ticks)) {
		nss_top->stats_n2h[NSS_STATS_N2H_WORST_CASE_TICKS] = noss->except_worst_case_ticks;
	}
	nss_top->stats_n2h[NSS_STATS_N2H_ITERATIONS] += noss->except_iterations;

	/*
	 * pbuf_mgr stats
	 */
	nss_top->stats_pbuf[NSS_STATS_PBUF_ALLOC_FAILS] += noss->pbuf_alloc_fails;
	nss_top->stats_pbuf[NSS_STATS_PBUF_PAYLOAD_ALLOC_FAILS] += noss->pbuf_payload_alloc_fails;

	/*
	 * TODO: Clean-up PE stats (there is no PE on NSS now)
	 */
	nss_top->pe_queue_dropped += noss->pe_queue_dropped;
	nss_top->pe_total_ticks += noss->pe_total_ticks;
	if (unlikely(nss_top->pe_worst_case_ticks < noss->pe_worst_case_ticks)) {
		nss_top->pe_worst_case_ticks = noss->pe_worst_case_ticks;
	}
	nss_top->pe_iterations += noss->pe_iterations;

	spin_unlock_bh(&nss_top->stats_lock);
}

/*
 * nss_rx_offload_interface_handler()
 *	Handle NSS -> HLOS messages for general NSS acceleration
 */
static void nss_rx_offload_stats_interface_handler(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm, __attribute__((unused))void *app_data)
{
	struct nss_offload_msg *nom = (struct nss_offload_msg *)ncm;

	/*
	 * Is this a valid request/response packet?
	 */
	if (nom->cm.type >= NSS_METADATA_TYPE_OFFLOAD_STATS_MAX) {
		nss_warning("%p: received invalid message %d for Offload stats interface", nss_ctx, nom->cm.type);
		return;
	}

	switch (nom->cm.type) {
	case NSS_RX_METADATA_TYPE_PER_INTERFACE_STATS_SYNC:
		nss_rx_metadata_interface_stats_sync(nss_ctx, &nom->msg.per_if_stats_sync, ncm->interface);
		break;

	case NSS_RX_METADATA_TYPE_NSS_OFFLOAD_STATS_SYNC:
		nss_rx_metadata_offload_stats_sync(nss_ctx, &nom->msg.offload_stats_sync);
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
 * nss_offload_stats_register_handler()
 */
void nss_offload_stats_register_handler()
{
	nss_core_register_handler(NSS_N2H_INTERFACE, nss_rx_offload_stats_interface_handler, NULL);
}
