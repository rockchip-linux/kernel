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
 * nss_tx_rx_phys_if.c
 *	NSS Physical i/f (gmac) APIs
 */
#include "nss_tx_rx_common.h"

/*
 * Maintain a private list of callbacks as the nss_top list
 * if for new registrations.
 */
nss_phys_if_event_callback_t nss_tx_rx_phys_if_event_callback[NSS_MAX_PHYSICAL_INTERFACES];

/*
 **********************************
 Rx APIs
 **********************************
 */

/*
 * nss_rx_metadata_gmac_stats_sync()
 *	Handle the syncing of GMAC stats.
 */
void nss_rx_metadata_gmac_stats_sync(struct nss_ctx_instance *nss_ctx,
		struct nss_phys_if_stats *stats, uint16_t interface)
{
	void *ctx;
	nss_phys_if_event_callback_t cb;
	struct nss_gmac_sync gmac_stats;
	uint32_t id = interface;

	/*
	 * Since the new extended statistics are not the same as the older stats
	 * parameter, we must do a field by field copy.
	 */
	gmac_stats.interface = interface;
	gmac_stats.rx_bytes = stats->if_stats.rx_bytes;
	gmac_stats.rx_packets = stats->if_stats.rx_packets;
	gmac_stats.rx_errors = stats->estats.rx_errors;
	gmac_stats.rx_receive_errors = stats->estats.rx_receive_errors;
	gmac_stats.rx_overflow_errors = stats->estats.rx_overflow_errors;
	gmac_stats.rx_descriptor_errors = stats->estats.rx_descriptor_errors;
	gmac_stats.rx_watchdog_timeout_errors = stats->estats.rx_watchdog_timeout_errors;
	gmac_stats.rx_crc_errors = stats->estats.rx_crc_errors;
	gmac_stats.rx_late_collision_errors = stats->estats.rx_late_collision_errors;
	gmac_stats.rx_dribble_bit_errors = stats->estats.rx_dribble_bit_errors;
	gmac_stats.rx_length_errors = stats->estats.rx_length_errors;
	gmac_stats.rx_ip_header_errors = stats->estats.rx_ip_header_errors;
	gmac_stats.rx_ip_payload_errors = stats->estats.rx_ip_payload_errors;
	gmac_stats.rx_no_buffer_errors = stats->estats.rx_no_buffer_errors;
	gmac_stats.rx_transport_csum_bypassed = stats->estats.rx_transport_csum_bypassed;
	gmac_stats.tx_bytes = stats->if_stats.tx_bytes;
	gmac_stats.tx_packets = stats->if_stats.tx_packets;
	gmac_stats.tx_collisions = stats->estats.tx_collisions;
	gmac_stats.tx_errors = stats->estats.tx_errors;
	gmac_stats.tx_jabber_timeout_errors = stats->estats.tx_jabber_timeout_errors;
	gmac_stats.tx_frame_flushed_errors = stats->estats.tx_frame_flushed_errors;
	gmac_stats.tx_loss_of_carrier_errors = stats->estats.tx_loss_of_carrier_errors;
	gmac_stats.tx_no_carrier_errors = stats->estats.tx_no_carrier_errors;
	gmac_stats.tx_late_collision_errors = stats->estats.tx_late_collision_errors;
	gmac_stats.tx_excessive_collision_errors = stats->estats.tx_excessive_collision_errors;
	gmac_stats.tx_excessive_deferral_errors = stats->estats.tx_excessive_deferral_errors;
	gmac_stats.tx_underflow_errors = stats->estats.tx_underflow_errors;
	gmac_stats.tx_ip_header_errors = stats->estats.tx_ip_header_errors;
	gmac_stats.tx_ip_payload_errors = stats->estats.tx_ip_payload_errors;
	gmac_stats.tx_dropped = stats->estats.tx_dropped;
	gmac_stats.hw_errs[0] = stats->estats.hw_errs[0];
	gmac_stats.hw_errs[1] = stats->estats.hw_errs[1];
	gmac_stats.hw_errs[2] = stats->estats.hw_errs[2];
	gmac_stats.hw_errs[3] = stats->estats.hw_errs[3];
	gmac_stats.hw_errs[4] = stats->estats.hw_errs[4];
	gmac_stats.hw_errs[5] = stats->estats.hw_errs[5];
	gmac_stats.hw_errs[6] = stats->estats.hw_errs[6];
	gmac_stats.hw_errs[7] = stats->estats.hw_errs[7];
	gmac_stats.hw_errs[8] = stats->estats.hw_errs[8];
	gmac_stats.hw_errs[9] = stats->estats.hw_errs[9];
	gmac_stats.rx_missed = stats->estats.rx_missed;
	gmac_stats.fifo_overflows = stats->estats.fifo_overflows;
	gmac_stats.rx_scatter_errors = stats->estats.rx_scatter_errors;
	gmac_stats.gmac_total_ticks = stats->estats.gmac_total_ticks;
	gmac_stats.gmac_worst_case_ticks = stats->estats.gmac_worst_case_ticks;
	gmac_stats.gmac_iterations = stats->estats.gmac_iterations;

	/*
	 * Get the locally registered callback.
	 */
	cb = nss_tx_rx_phys_if_event_callback[id];
	ctx = nss_ctx->nss_top->if_ctx[id];

	/*
	 * Call GMAC driver callback
	 */
	if (!cb || !ctx) {
		nss_warning("%p: Event received for GMAC interface %d before registration", nss_ctx, interface);
		return;
	}

	cb(ctx, NSS_GMAC_EVENT_STATS, (void *)&gmac_stats, sizeof(struct nss_gmac_sync));


}

/*
 **********************************
 Tx APIs
 **********************************
 */

/*
 * nss_tx_phys_if_buf ()
 *	Send packet to physical interface owned by NSS
 */
nss_tx_status_t nss_tx_phys_if_buf(void *ctx, struct sk_buff *os_buf, uint32_t if_num)
{
	struct nss_ctx_instance *nss_ctx = (struct nss_ctx_instance *) ctx;
	return nss_phys_if_tx_buf(nss_ctx, os_buf, if_num);
}

/*
 * nss_tx_phys_if_open()
 *	Send open command to physical interface
 */
nss_tx_status_t nss_tx_phys_if_open(void *ctx, uint32_t tx_desc_ring, uint32_t rx_desc_ring, uint32_t if_num)
{
	struct nss_ctx_instance *nss_ctx = (struct nss_ctx_instance *) ctx;
	struct nss_phys_if_msg nim;
	struct nss_if_open *nio;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	nss_info("%p: Phys If Open, id:%d, TxDesc: %x, RxDesc: %x\n", nss_ctx, if_num, tx_desc_ring, rx_desc_ring);

	nss_cmn_msg_init(&nim.cm, if_num, NSS_TX_METADATA_TYPE_INTERFACE_OPEN,
			sizeof(struct nss_if_open), NULL, NULL);

	nio = &nim.msg.if_msg.open;
	nio->tx_desc_ring = tx_desc_ring;
	nio->rx_desc_ring = rx_desc_ring;
	return nss_phys_if_tx_msg(nss_ctx, &nim);
}

/*
 * nss_tx_phys_if_close()
 *	Send close command to physical interface
 */
nss_tx_status_t nss_tx_phys_if_close(void *ctx, uint32_t if_num)
{
	struct nss_ctx_instance *nss_ctx = (struct nss_ctx_instance *) ctx;
	struct nss_phys_if_msg nim;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	nss_info("%p: Phys If Close, id:%d \n", nss_ctx, if_num);

	nss_cmn_msg_init(&nim.cm, if_num, NSS_TX_METADATA_TYPE_INTERFACE_CLOSE,
			sizeof(struct nss_if_close), NULL, NULL);

	return nss_phys_if_tx_msg(nss_ctx, &nim);
}

/*
 * nss_tx_phys_if_link_state()
 *	Send link state to physical interface
 */
nss_tx_status_t nss_tx_phys_if_link_state(void *ctx, uint32_t link_state, uint32_t if_num)
{
	struct nss_ctx_instance *nss_ctx = (struct nss_ctx_instance *) ctx;
	struct nss_phys_if_msg nim;
	struct nss_if_link_state_notify *nils;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	nss_info("%p: Phys If Link State, id:%d, State: %x\n", nss_ctx, if_num, link_state);

	nss_cmn_msg_init(&nim.cm, if_num, NSS_TX_METADATA_TYPE_INTERFACE_LINK_STATE_NOTIFY,
			sizeof(struct nss_if_link_state_notify), NULL, NULL);

	nils = &nim.msg.if_msg.link_state_notify;
	nils->state = link_state;
	return nss_phys_if_tx_msg(nss_ctx, &nim);
}

/*
 * nss_tx_phys_if_mac_addr()
 *	Send a MAC address to physical interface
 */
nss_tx_status_t nss_tx_phys_if_mac_addr(void *ctx, uint8_t *addr, uint32_t if_num)
{
	struct nss_ctx_instance *nss_ctx = (struct nss_ctx_instance *) ctx;
	struct nss_phys_if_msg nim;
	struct nss_if_mac_address_set *nmas;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	nss_info("%p: Phys If MAC Address, id:%d\n", nss_ctx, if_num);
	nss_assert(addr != 0);

	nss_cmn_msg_init(&nim.cm, if_num, NSS_TX_METADATA_TYPE_INTERFACE_MAC_ADDR_SET,
			sizeof(struct nss_if_mac_address_set), NULL, NULL);

	nmas = &nim.msg.if_msg.mac_address_set;
	memcpy(nmas->mac_addr, addr, ETH_ALEN);
	return nss_phys_if_tx_msg(nss_ctx, &nim);
}

/*
 * nss_tx_phys_if_change_mtu()
 *	Send a MTU change command
 */
nss_tx_status_t nss_tx_phys_if_change_mtu(void *ctx, uint32_t mtu, uint32_t if_num)
{
	struct nss_ctx_instance *nss_ctx = (struct nss_ctx_instance *) ctx;
	struct nss_phys_if_msg nim;
	struct nss_if_mtu_change *nimc;
	int32_t i;
	uint16_t max_mtu;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	nss_info("%p: Phys If Change MTU, id:%d, mtu=%d\n", nss_ctx, if_num, mtu);
	nss_cmn_msg_init(&nim.cm, if_num, NSS_TX_METADATA_TYPE_INTERFACE_MTU_CHANGE,
			sizeof(struct nss_if_mtu_change), NULL, NULL);

	nimc = &nim.msg.if_msg.mtu_change;
	nimc->min_buf_size = (uint16_t)mtu + NSS_NBUF_ETH_EXTRA;

	/*
	 * TODO: If we want to maintain this in the long run, we will need to place
	 * this in the ack side of the MTU_CHANGE request.
	 */
	nss_ctx->phys_if_mtu[if_num] = (uint16_t)mtu;
	max_mtu = nss_ctx->phys_if_mtu[0];
	for (i = 1; i < NSS_MAX_PHYSICAL_INTERFACES; i++) {
		if (max_mtu < nss_ctx->phys_if_mtu[i]) {
			max_mtu = nss_ctx->phys_if_mtu[i];
		}
	}

	if (max_mtu <= NSS_ETH_NORMAL_FRAME_MTU) {
		max_mtu = NSS_ETH_NORMAL_FRAME_MTU;
	} else if (max_mtu <= NSS_ETH_MINI_JUMBO_FRAME_MTU) {
		max_mtu = NSS_ETH_MINI_JUMBO_FRAME_MTU;
	} else if (max_mtu <= NSS_ETH_FULL_JUMBO_FRAME_MTU) {
		max_mtu = NSS_ETH_FULL_JUMBO_FRAME_MTU;
	}
	nss_ctx->max_buf_size = ((max_mtu + ETH_HLEN + SMP_CACHE_BYTES - 1) & ~(SMP_CACHE_BYTES - 1)) + NSS_NBUF_PAD_EXTRA;

	return nss_phys_if_tx_msg(nss_ctx, &nim);
}

/*
 **********************************
 Register/Unregister/Miscellaneous APIs
 **********************************
 */

/*
 * nss_register_phys_if()
 */
void *nss_register_phys_if(uint32_t if_num,
				nss_phys_if_rx_callback_t rx_callback,
				nss_phys_if_event_callback_t event_callback, struct net_device *if_ctx)
{
	uint8_t id = nss_top_main.phys_if_handler_id[if_num];
	struct nss_ctx_instance *nss_ctx = &nss_top_main.nss[id];

	nss_assert(if_num <= NSS_MAX_PHYSICAL_INTERFACES);

	nss_top_main.if_ctx[if_num] = (void *)if_ctx;
	nss_top_main.if_rx_callback[if_num] = rx_callback;
	nss_tx_rx_phys_if_event_callback[if_num] = event_callback;

	nss_ctx->phys_if_mtu[if_num] = NSS_ETH_NORMAL_FRAME_MTU;
	return nss_ctx;
}

/*
 * nss_unregister_phys_if_()
 */
void nss_unregister_phys_if(uint32_t if_num)
{
	nss_assert(if_num < NSS_MAX_PHYSICAL_INTERFACES);
	nss_top_main.if_rx_callback[if_num] = NULL;
	nss_tx_rx_phys_if_event_callback[if_num] = NULL;
	nss_top_main.if_ctx[if_num] = NULL;
	nss_top_main.nss[0].phys_if_mtu[if_num] = 0;
	nss_top_main.nss[1].phys_if_mtu[if_num] = 0;
}

/*
 * nss_tx_phys_if_get_napi_ctx()
 *	Obtain the napi context that is used to handle packet processing.
 */
nss_tx_status_t nss_tx_phys_if_get_napi_ctx(void *nss_ctx, struct napi_struct **napi_ctx)
{
	return nss_phys_if_get_napi_ctx((struct nss_ctx_instance *)nss_ctx, napi_ctx);
}


/**
 * nss_get_state()
 *	return the NSS initialization state
 */
nss_state_t nss_get_state(void *ctx)
{
	return nss_cmn_get_state(ctx);
}


EXPORT_SYMBOL(nss_tx_phys_if_buf);
EXPORT_SYMBOL(nss_tx_phys_if_open);
EXPORT_SYMBOL(nss_tx_phys_if_close);
EXPORT_SYMBOL(nss_tx_phys_if_link_state);
EXPORT_SYMBOL(nss_tx_phys_if_change_mtu);
EXPORT_SYMBOL(nss_tx_phys_if_mac_addr);
EXPORT_SYMBOL(nss_tx_phys_if_get_napi_ctx);
EXPORT_SYMBOL(nss_register_phys_if);
EXPORT_SYMBOL(nss_unregister_phys_if);
EXPORT_SYMBOL(nss_get_state);


