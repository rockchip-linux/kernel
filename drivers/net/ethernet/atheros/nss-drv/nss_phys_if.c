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
 * nss_phy_if.c
 *	NSS physical interface functions
 */

#include "nss_tx_rx_common.h"

#define NSS_PHYS_IF_TX_TIMEOUT 3000 /* 3 Seconds */

/*
 * Private data structure for phys_if interface
 */
static struct nss_phys_if_pvt {
	struct semaphore sem;
	struct completion complete;
	int response;
} phif;

static int nss_phys_if_sem_init_done;

/*
 * nss_phys_if_gmac_stats_sync()
 *	Handle the syncing of GMAC stats.
 */
void nss_phys_if_gmac_stats_sync(struct nss_ctx_instance *nss_ctx,
		struct nss_phys_if_stats *stats, uint16_t interface)
{
	void *ctx;
	struct nss_gmac_stats gmac_stats;
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
	 * Get the netdev ctx
	 */
	ctx = nss_ctx->nss_top->subsys_dp_register[id].ndev;

	/*
	 * Pass through gmac exported api
	 */
	if (!ctx) {
		nss_warning("%p: Event received for GMAC interface %d before registration", nss_ctx, interface);
		return;
	}

	nss_gmac_event_receive(ctx, NSS_GMAC_EVENT_STATS, (void *)&gmac_stats, sizeof(struct nss_gmac_stats));
}

/*
 * nss_phys_if_update_driver_stats()
 *	Snoop the extended message and update driver statistics.
 */
static void nss_phys_if_update_driver_stats(struct nss_ctx_instance *nss_ctx, uint32_t id, struct nss_phys_if_stats *stats)
{
	struct nss_top_instance *nss_top = nss_ctx->nss_top;
	uint64_t *top_stats = &(nss_top->stats_gmac[id][0]);

	spin_lock_bh(&nss_top->stats_lock);
	top_stats[NSS_STATS_GMAC_TOTAL_TICKS] += stats->estats.gmac_total_ticks;
	if (unlikely(top_stats[NSS_STATS_GMAC_WORST_CASE_TICKS] < stats->estats.gmac_worst_case_ticks)) {
		top_stats[NSS_STATS_GMAC_WORST_CASE_TICKS] = stats->estats.gmac_worst_case_ticks;
	}
	top_stats[NSS_STATS_GMAC_ITERATIONS] += stats->estats.gmac_iterations;
	spin_unlock_bh(&nss_top->stats_lock);
}

/*
 * nss_phys_if_msg_handler()
 *	Handle NSS -> HLOS messages for physical interface/gmacs
 */
static void nss_phys_if_msg_handler(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm,
		__attribute__((unused))void *app_data)
{
	struct nss_phys_if_msg *nim = (struct nss_phys_if_msg *)ncm;
	nss_phys_if_msg_callback_t cb;

	/*
	 * Sanity check the message type
	 */
	if (ncm->type > NSS_PHYS_IF_MAX_MSG_TYPES) {
		nss_warning("%p: message type out of range: %d", nss_ctx, ncm->type);
		return;
	}

	if (!NSS_IS_IF_TYPE(PHYSICAL, ncm->interface)) {
		nss_warning("%p: response for another interface: %d", nss_ctx, ncm->interface);
		return;
	}

	if (ncm->len > sizeof(struct nss_phys_if_msg)) {
		nss_warning("%p: message length too big: %d", nss_ctx, ncm->len);
		return;
	}

	/*
	 * Messages value that are within the base class are handled by the base class.
	 */
	if (ncm->type < NSS_IF_MAX_MSG_TYPES) {
		return nss_if_msg_handler(nss_ctx, ncm, app_data);
	}

	/*
	 * Log failures
	 */
	nss_core_log_msg_failures(nss_ctx, ncm);

	/*
	 * Snoop messages for local driver and handle deprecated interfaces.
	 */
	switch (nim->cm.type) {
	case NSS_PHYS_IF_EXTENDED_STATS_SYNC:
		/*
		 * To create the old API gmac statistics, we use the new extended GMAC stats.
		 */
		nss_phys_if_update_driver_stats(nss_ctx, ncm->interface, &nim->msg.stats);
		nss_phys_if_gmac_stats_sync(nss_ctx, &nim->msg.stats, ncm->interface);
		break;
	}

	/*
	 * Update the callback and app_data for NOTIFY messages, IPv4 sends all notify messages
	 * to the same callback/app_data.
	 */
	if (ncm->response == NSS_CMM_RESPONSE_NOTIFY) {
		ncm->cb = (uint32_t)nss_ctx->nss_top->phys_if_msg_callback[ncm->interface];
		ncm->app_data = (uint32_t)nss_ctx->nss_top->subsys_dp_register[ncm->interface].ndev;
	}

	/*
	 * Do we have a callback?
	 */
	if (!ncm->cb) {
		return;
	}

	/*
	 * Callback
	 */
	cb = (nss_phys_if_msg_callback_t)ncm->cb;
	cb((void *)ncm->app_data, nim);
}

/*
 * nss_phys_if_callback
 *	Callback to handle the completion of NSS ->HLOS messages.
 */
static void nss_phys_if_callback(void *app_data, struct nss_phys_if_msg *nim)
{
	if(nim->cm.response != NSS_CMN_RESPONSE_ACK) {
		nss_warning("phys_if Error response %d\n", nim->cm.response);
		phif.response = NSS_TX_FAILURE;
		complete(&phif.complete);
		return;
	}

	phif.response = NSS_TX_SUCCESS;
	complete(&phif.complete);
}

/*
 * nss_phys_if_buf()
 *	Send packet to physical interface owned by NSS
 */
nss_tx_status_t nss_phys_if_buf(struct nss_ctx_instance *nss_ctx, struct sk_buff *os_buf, uint32_t if_num)
{
	int32_t status;

	nss_trace("%p: Phys If Tx packet, id:%d, data=%p", nss_ctx, if_num, os_buf->data);

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("%p: 'Phys If Tx' packet dropped as core not ready", nss_ctx);
		return NSS_TX_FAILURE_NOT_READY;
	}

	status = nss_core_send_buffer(nss_ctx, if_num, os_buf, NSS_IF_DATA_QUEUE_0, H2N_BUFFER_PACKET, 0);
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
	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_DATA_QUEUE_0].desc_ring.int_bit,
									NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_PACKET]);
	return NSS_TX_SUCCESS;
}

/*
 * nss_phys_if_msg()
 */
nss_tx_status_t nss_phys_if_msg(struct nss_ctx_instance *nss_ctx, struct nss_phys_if_msg *nim)
{
	struct nss_cmn_msg *ncm = &nim->cm;
	struct nss_phys_if_msg *nim2;
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
	if (!NSS_IS_IF_TYPE(PHYSICAL, ncm->interface)) {
		nss_warning("%p: tx request for another interface: %d", nss_ctx, ncm->interface);
		return NSS_TX_FAILURE;
	}

	if (ncm->type > NSS_PHYS_IF_MAX_MSG_TYPES) {
		nss_warning("%p: message type out of range: %d", nss_ctx, ncm->type);
		return NSS_TX_FAILURE;
	}

	if (ncm->len > sizeof(struct nss_phys_if_msg)) {
		nss_warning("%p: invalid length: %d", nss_ctx, ncm->len);
		return NSS_TX_FAILURE;
	}

	if_num = ncm->interface;
	dev = nss_top_main.subsys_dp_register[if_num].ndev;
	if (!dev) {
		nss_warning("%p: Unregister physical interface %d: no context", nss_ctx, if_num);
		return NSS_TX_FAILURE_BAD_PARAM;
	}

	nbuf = dev_alloc_skb(NSS_NBUF_PAYLOAD_SIZE);
	if (unlikely(!nbuf)) {
		spin_lock_bh(&nss_ctx->nss_top->stats_lock);
		nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]++;
		spin_unlock_bh(&nss_ctx->nss_top->stats_lock);
		nss_warning("%p: physical interface %p: command allocation failed", nss_ctx, dev);
		return NSS_TX_FAILURE;
	}

	nim2 = (struct nss_phys_if_msg *)skb_put(nbuf, sizeof(struct nss_phys_if_msg));
	memcpy(nim2, nim, sizeof(struct nss_phys_if_msg));

	status = nss_core_send_buffer(nss_ctx, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		dev_kfree_skb_any(nbuf);
		nss_warning("%p: Unable to enqueue 'physical interface' command\n", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit,
		NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	return NSS_TX_SUCCESS;
}

/*
 * nss_phys_if_tx_msg_sync()
 *	Send a message to physical interface & wait for the response.
 */
nss_tx_status_t nss_phys_if_msg_sync(struct nss_ctx_instance *nss_ctx, struct nss_phys_if_msg *nim)
{
	nss_tx_status_t status;
	int ret = 0;

	down(&phif.sem);

	status = nss_phys_if_msg(nss_ctx, nim);
	if(status != NSS_TX_SUCCESS)
	{
		nss_warning("%p: nss_phys_if_msg failed\n", nss_ctx);
		up(&phif.sem);
		return status;
	}

	ret = wait_for_completion_timeout(&phif.complete, msecs_to_jiffies(NSS_PHYS_IF_TX_TIMEOUT));

	if(!ret)
	{
		nss_warning("%p: phys_if tx failed due to timeout\n", nss_ctx);
		phif.response = NSS_TX_FAILURE;
	}

	status = phif.response;
	up(&phif.sem);

	return status;
}

/*
 **********************************
 Register/Unregister/Miscellaneous APIs
 **********************************
 */

/*
 * nss_phys_if_register()
 */
struct nss_ctx_instance *nss_phys_if_register(uint32_t if_num,
				nss_phys_if_rx_callback_t rx_callback,
				nss_phys_if_msg_callback_t msg_callback,
				struct net_device *netdev,
				uint32_t features)
{
	uint8_t id = nss_top_main.phys_if_handler_id[if_num];
	struct nss_ctx_instance *nss_ctx = &nss_top_main.nss[id];

	nss_assert(if_num <= NSS_MAX_PHYSICAL_INTERFACES);

	nss_top_main.subsys_dp_register[if_num].ndev = netdev;
	nss_top_main.subsys_dp_register[if_num].cb = rx_callback;
	nss_top_main.subsys_dp_register[if_num].app_data = NULL;
	nss_top_main.subsys_dp_register[if_num].features = features;

	nss_top_main.phys_if_msg_callback[if_num] = msg_callback;

	nss_ctx->phys_if_mtu[if_num] = NSS_GMAC_NORMAL_FRAME_MTU;
	return nss_ctx;
}

/*
 * nss_phys_if_unregister()
 */
void nss_phys_if_unregister(uint32_t if_num)
{
	nss_assert(if_num < NSS_MAX_PHYSICAL_INTERFACES);

	nss_top_main.subsys_dp_register[if_num].ndev = NULL;
	nss_top_main.subsys_dp_register[if_num].cb = NULL;
	nss_top_main.subsys_dp_register[if_num].app_data = NULL;
	nss_top_main.subsys_dp_register[if_num].features = 0;

	nss_top_main.phys_if_msg_callback[if_num] = NULL;

	nss_top_main.nss[0].phys_if_mtu[if_num] = 0;
	nss_top_main.nss[1].phys_if_mtu[if_num] = 0;
}

/*
 * nss_phys_if_register_handler()
 */
void nss_phys_if_register_handler(uint32_t if_num)
{
	uint32_t ret;

	ret = nss_core_register_handler(if_num, nss_phys_if_msg_handler, NULL);

	if (ret != NSS_CORE_STATUS_SUCCESS) {
		nss_warning("Message handler FAILED to be registered for interface %d", if_num);
		return;
	}

	if(!nss_phys_if_sem_init_done) {
		sema_init(&phif.sem, 1);
		init_completion(&phif.complete);
		nss_phys_if_sem_init_done = 1;
	}
}

/*
 * nss_phys_if_open()
 *	Send open command to physical interface
 */
nss_tx_status_t nss_phys_if_open(struct nss_ctx_instance *nss_ctx, uint32_t tx_desc_ring, uint32_t rx_desc_ring, uint32_t mode, uint32_t if_num)
{
	struct nss_phys_if_msg nim;
	struct nss_if_open *nio;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	nss_info("%p: Phys If Open, id:%d, TxDesc: %x, RxDesc: %x\n", nss_ctx, if_num, tx_desc_ring, rx_desc_ring);

	nss_cmn_msg_init(&nim.cm, if_num, NSS_TX_METADATA_TYPE_INTERFACE_OPEN,
			sizeof(struct nss_if_open), nss_phys_if_callback, NULL);

	nio = &nim.msg.if_msg.open;
	nio->tx_desc_ring = tx_desc_ring;
	nio->rx_desc_ring = rx_desc_ring;
	if (mode == NSS_GMAC_MODE0) {
		nio->rx_forward_if = NSS_ETH_RX_INTERFACE;
		nio->alignment_mode = NSS_IF_DATA_ALIGN_2BYTE;
	} else if(mode == NSS_GMAC_MODE1) {
		nio->rx_forward_if = NSS_SJACK_INTERFACE;
		nio->alignment_mode = NSS_IF_DATA_ALIGN_4BYTE;
	} else {
		nss_info("%p: Phys If Open, unknown mode %d\n", nss_ctx, mode);
		return NSS_GMAC_FAILURE;
	}

	return nss_phys_if_msg_sync(nss_ctx, &nim);
}

/*
 * nss_phys_if_close()
 *	Send close command to physical interface
 */
nss_tx_status_t nss_phys_if_close(struct nss_ctx_instance *nss_ctx, uint32_t if_num)
{
	struct nss_phys_if_msg nim;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	nss_info("%p: Phys If Close, id:%d \n", nss_ctx, if_num);

	nss_cmn_msg_init(&nim.cm, if_num, NSS_TX_METADATA_TYPE_INTERFACE_CLOSE,
			sizeof(struct nss_if_close), nss_phys_if_callback, NULL);

	return nss_phys_if_msg_sync(nss_ctx, &nim);
}

/*
 * nss_phys_if_link_state()
 *	Send link state to physical interface
 */
nss_tx_status_t nss_phys_if_link_state(struct nss_ctx_instance *nss_ctx, uint32_t link_state, uint32_t if_num)
{
	struct nss_phys_if_msg nim;
	struct nss_if_link_state_notify *nils;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	nss_info("%p: Phys If Link State, id:%d, State: %x\n", nss_ctx, if_num, link_state);

	nss_cmn_msg_init(&nim.cm, if_num, NSS_TX_METADATA_TYPE_INTERFACE_LINK_STATE_NOTIFY,
			sizeof(struct nss_if_link_state_notify), nss_phys_if_callback, NULL);

	nils = &nim.msg.if_msg.link_state_notify;
	nils->state = link_state;
	return nss_phys_if_msg_sync(nss_ctx, &nim);
}

/*
 * nss_phys_if_mac_addr()
 *	Send a MAC address to physical interface
 */
nss_tx_status_t nss_phys_if_mac_addr(struct nss_ctx_instance *nss_ctx, uint8_t *addr, uint32_t if_num)
{
	struct nss_phys_if_msg nim;
	struct nss_if_mac_address_set *nmas;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	nss_info("%p: Phys If MAC Address, id:%d\n", nss_ctx, if_num);
	nss_assert(addr != 0);

	nss_cmn_msg_init(&nim.cm, if_num, NSS_TX_METADATA_TYPE_INTERFACE_MAC_ADDR_SET,
			sizeof(struct nss_if_mac_address_set), nss_phys_if_callback, NULL);

	nmas = &nim.msg.if_msg.mac_address_set;
	memcpy(nmas->mac_addr, addr, ETH_ALEN);
	return nss_phys_if_msg_sync(nss_ctx, &nim);
}

/*
 * nss_phys_if_change_mtu()
 *	Send a MTU change command
 */
nss_tx_status_t nss_phys_if_change_mtu(struct nss_ctx_instance *nss_ctx, uint32_t mtu, uint32_t if_num)
{
	struct nss_phys_if_msg nim;
	struct nss_if_mtu_change *nimc;
	int32_t i;
	uint16_t max_mtu;

	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	nss_info("%p: Phys If Change MTU, id:%d, mtu=%d\n", nss_ctx, if_num, mtu);
	nss_cmn_msg_init(&nim.cm, if_num, NSS_TX_METADATA_TYPE_INTERFACE_MTU_CHANGE,
			sizeof(struct nss_if_mtu_change), nss_phys_if_callback, NULL);

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

	if (max_mtu <= NSS_GMAC_NORMAL_FRAME_MTU) {
		max_mtu = NSS_GMAC_NORMAL_FRAME_MTU;
	} else if (max_mtu <= NSS_GMAC_MINI_JUMBO_FRAME_MTU) {
		max_mtu = NSS_GMAC_MINI_JUMBO_FRAME_MTU;
	} else if (max_mtu <= NSS_GMAC_FULL_JUMBO_FRAME_MTU) {
		max_mtu = NSS_GMAC_FULL_JUMBO_FRAME_MTU;
	} else {
		nss_info("%p: MTU larger than FULL_JUMBO_FRAME(%d)", nss_ctx, NSS_GMAC_FULL_JUMBO_FRAME_MTU);
		return NSS_TX_FAILURE;
	}
	nss_ctx->max_buf_size = ((max_mtu + ETH_HLEN + SMP_CACHE_BYTES - 1) & ~(SMP_CACHE_BYTES - 1)) + NSS_NBUF_PAD_EXTRA;

	return nss_phys_if_msg_sync(nss_ctx, &nim);
}

/**
 * nss_get_state()
 *	return the NSS initialization state
 */
nss_state_t nss_get_state(void *ctx)
{
	return nss_cmn_get_state(ctx);
}

EXPORT_SYMBOL(nss_get_state);
