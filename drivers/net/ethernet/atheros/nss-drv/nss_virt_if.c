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
 * nss_virt_if.c
 *	NSS virtual/redirect handler APIs
 */

#include "nss_tx_rx_common.h"
#include <net/arp.h>

extern int nss_ctl_redirect;

/*
 * nss_virt_if_msg_handler()
 *	Handle msg responses from the FW on virtual interfaces
 */
static void nss_virt_if_msg_handler(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm, __attribute__((unused))void *app_data)
{
	struct nss_virt_if_msg *nvim = (struct nss_virt_if_msg *)ncm;
	nss_virt_if_msg_callback_t cb;

	/*
	 * Sanity check the message type
	 */
	if (ncm->type > NSS_VIRT_IF_MAX_MSG_TYPES) {
		nss_warning("%p: message type out of range: %d", nss_ctx, ncm->type);
		return;
	}

	if (ncm->interface > NSS_MAX_VIRTUAL_INTERFACES) {
		nss_warning("%p: response for another interface: %d", nss_ctx, ncm->interface);
		return;
	}

	if (ncm->len > sizeof(struct nss_virt_if_msg)) {
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
	 * Update the callback and app_data for NOTIFY messages, IPv4 sends all notify messages
	 * to the same callback/app_data.
	 */
	if (ncm->response == NSS_CMM_RESPONSE_NOTIFY) {
		ncm->cb = (uint32_t)nss_ctx->nss_top->virt_if_msg_callback[ncm->interface];
		ncm->app_data = (uint32_t)nss_ctx->nss_top->if_ctx[ncm->interface];
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
	cb = (nss_virt_if_msg_callback_t)ncm->cb;
	cb((void *)ncm->app_data, nvim);
}

/*
 * nss_virt_if_tx_rxbuf()
 *	HLOS interface has received a packet which we redirect to the NSS, if appropriate to do so.
 */
static nss_tx_status_t nss_virt_if_tx_rxbuf(int32_t if_num, struct sk_buff *skb, uint32_t nwifi)
{
	int32_t status;
	struct nss_ctx_instance *nss_ctx = &nss_top_main.nss[nss_top_main.ipv4_handler_id];
	uint32_t bufftype;

	if (unlikely(nss_ctl_redirect == 0) || unlikely(skb->vlan_tci)) {
		return NSS_TX_FAILURE_NOT_SUPPORTED;
	}

	nss_assert(NSS_IS_IF_TYPE(VIRTUAL, if_num));
	nss_trace("%p: Virtual Rx packet, if_num:%d, skb:%p", nss_ctx, if_num, skb);

	/*
	 * Get the NSS context that will handle this packet and check that it is initialised and ready
	 */
	NSS_VERIFY_CTX_MAGIC(nss_ctx);
	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("%p: Virtual Rx packet dropped as core not ready", nss_ctx);
		return NSS_TX_FAILURE_NOT_READY;
	}

	/*
	 * Sanity check the SKB to ensure that it's suitable for us
	 */
	if (unlikely(skb->len <= ETH_HLEN)) {
		nss_warning("%p: Virtual Rx packet: %p too short", nss_ctx, skb);
		return NSS_TX_FAILURE_TOO_SHORT;
	}

	if (unlikely(skb_shinfo(skb)->nr_frags != 0)) {
		/*
		 * TODO: If we have a connection matching rule for this skbuff,
		 * do we need to flush it??
		 */
		nss_warning("%p: Delivering the packet to Linux because of fragmented skb: %p\n", nss_ctx, skb);
		return NSS_TX_FAILURE_NOT_SUPPORTED;
	}

	if (nwifi) {
		bufftype = H2N_BUFFER_NATIVE_WIFI;
	} else {
		bufftype = H2N_BUFFER_PACKET;

		/*
		 * NSS expects to see buffer from Ethernet header onwards
		 * Assumption: eth_type_trans has been done by WLAN driver
		 */
		skb_push(skb, ETH_HLEN);
	}

	/*
	 * Direct the buffer to the NSS
	 */
	status = nss_core_send_buffer(nss_ctx, if_num, skb, NSS_IF_DATA_QUEUE, bufftype, H2N_BIT_FLAG_VIRTUAL_BUFFER);
	if (unlikely(status != NSS_CORE_STATUS_SUCCESS)) {
		nss_warning("%p: Virtual Rx packet unable to enqueue\n", nss_ctx);
		if (!nwifi) {
			skb_pull(skb, ETH_HLEN);
		}
		return NSS_TX_FAILURE_QUEUE;
	}

	/*
	 * Kick the NSS awake so it can process our new entry.
	 */
	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_DATA_QUEUE].desc_ring.int_bit,
						NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);
	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_PACKET]);
	return NSS_TX_SUCCESS;
}

/*
 * nss_virt_if_tx_eth_rxbuf()
 */
nss_tx_status_t nss_virt_if_tx_eth_rxbuf(int32_t if_num, struct sk_buff *skb)
{
	return nss_virt_if_tx_rxbuf(if_num, skb, 0);
}

/*
 * nss_virt_if_tx_nwifi_rxbuf()
 */
nss_tx_status_t nss_virt_if_tx_nwifi_rxbuf(int32_t if_num, struct sk_buff *skb)
{
	return nss_virt_if_tx_rxbuf(if_num, skb, 1);
}


/*
 * nss_virt_if_tx_msg()
 */
nss_tx_status_t nss_virt_if_tx_msg(struct nss_virt_if_msg *nvim)
{
	int32_t status;
	struct sk_buff *nbuf;
	struct nss_cmn_msg *ncm = &nvim->cm;
	struct nss_virt_if_msg *nvim2;
	struct nss_ctx_instance *nss_ctx = &nss_top_main.nss[nss_top_main.ipv4_handler_id];

	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("Interface could not be created as core not ready");
		return NSS_TX_FAILURE;
	}

	/*
	 * Sanity check the message
	 */
	if (!NSS_IS_IF_TYPE(VIRTUAL, ncm->interface)) {
		nss_warning("%p: tx request for another interface: %d", nss_ctx, ncm->interface);
		return NSS_TX_FAILURE;
	}

	if (ncm->type > NSS_VIRT_IF_MAX_MSG_TYPES) {
		nss_warning("%p: message type out of range: %d", nss_ctx, ncm->type);
		return NSS_TX_FAILURE;
	}

	if (ncm->len > sizeof(struct nss_virt_if_msg)) {
		nss_warning("%p: invalid length: %d. Length of virt msg is %d",
				nss_ctx, ncm->len, sizeof(struct nss_virt_if_msg));
		return NSS_TX_FAILURE;
	}

	nbuf = dev_alloc_skb(NSS_NBUF_PAYLOAD_SIZE);
	if (unlikely(!nbuf)) {
		spin_lock_bh(&nss_ctx->nss_top->stats_lock);
		nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]++;
		spin_unlock_bh(&nss_ctx->nss_top->stats_lock);
		nss_warning("%p: virtual interface %d: command allocation failed", nss_ctx, ncm->interface);
		return NSS_TX_FAILURE;
	}

	nvim2 = (struct nss_virt_if_msg *)skb_put(nbuf, sizeof(struct nss_virt_if_msg));
	memcpy(nvim2, nvim, sizeof(struct nss_virt_if_msg));

	status = nss_core_send_buffer(nss_ctx, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (status != NSS_CORE_STATUS_SUCCESS) {
		dev_kfree_skb_any(nbuf);
		nss_warning("%p: Unable to enqueue 'virtual interface' command\n", nss_ctx);
		return NSS_TX_FAILURE;
	}

	nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit,
		NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	/*
	 * The context returned is the virtual interface # which is, essentially, the index into the if_ctx
	 * array that is holding the net_device pointer
	 */
	return NSS_TX_SUCCESS;
}

/*
 * nss_virt_if_assign_if_num()
 */
int32_t nss_virt_if_assign_if_num(struct net_device *if_ctx)
{
	int32_t if_num;
	struct nss_ctx_instance *nss_ctx __attribute__((unused)) = &nss_top_main.nss[nss_top_main.ipv4_handler_id];

	/*
	 * Check if net_device is Ethernet type
	 */
	if (if_ctx->type != ARPHRD_ETHER) {
		nss_warning("%p:Register virtual interface %p: type incorrect: %d ", nss_ctx, if_ctx, if_ctx->type);
		return NSS_TX_FAILURE;
	}

	/*
	 * Find a free virtual interface
	 */
	spin_lock_bh(&nss_top_main.lock);
	for (if_num = NSS_MAX_PHYSICAL_INTERFACES; if_num < NSS_MAX_DEVICE_INTERFACES; ++if_num) {
		if (!nss_top_main.if_ctx[if_num]) {
			/*
			 * Use this redirection interface
			 */
			nss_top_main.if_ctx[if_num] = (void *)if_ctx;
			break;
		}
	}

	spin_unlock_bh(&nss_top_main.lock);
	if (if_num == NSS_MAX_DEVICE_INTERFACES) {
		/*
		 * No available virtual contexts
		 */
		nss_warning("%p:Register virtual interface %p: no contexts available:", nss_ctx, if_ctx);
		return NSS_TX_FAILURE;
	}

	nss_info("%p:Registered virtual interface %d: context %p", nss_ctx, if_num, if_ctx);
	return if_num;
}

/*
 * nss_virt_if_register()
 */
struct nss_ctx_instance *nss_virt_if_register(uint32_t if_num,
				nss_virt_if_msg_callback_t msg_callback,
				struct net_device *if_ctx)
{
	uint8_t id = nss_top_main.virt_if_handler_id[if_num];
	struct nss_ctx_instance *nss_ctx = &nss_top_main.nss[id];

	/*
	 * TODO: the use of if_ctx as a net_dev and forcing this
	 * for the caller is not how app_data is typically handled.
	 * Re-think this.
	 *
	 * TODO: Where does the received buffers transmitted from
	 * a managed ethernet device land if they are to be output
	 * by the Wi-Fi driver?
	 */
	nss_top_main.if_ctx[if_num] = (void *)if_ctx;
	nss_top_main.virt_if_msg_callback[if_num] = msg_callback;

	return nss_ctx;
}

/*
 * nss_virt_if_unregister()
 */
void nss_virt_if_unregister(uint32_t if_num)
{
	nss_top_main.if_ctx[if_num] = NULL;
	nss_top_main.virt_if_msg_callback[if_num] = NULL;
}

/*
 * nss_virt_if_get_interface_num()
 *	Get interface number for a virtual interface
 */
int32_t nss_virt_if_get_interface_num(void *if_ctx)
{
	int32_t if_num = (int32_t)if_ctx;
	nss_assert(NSS_IS_IF_TYPE(VIRTUAL, if_num));
	return if_num;
}

/*
 * nss_phys_if_register_handler()
 */
void nss_virt_if_register_handler(void)
{
	int i;
	uint32_t ret;
	int end = NSS_VIRTUAL_IF_START + NSS_MAX_VIRTUAL_INTERFACES;

	for (i = NSS_VIRTUAL_IF_START; i < end; i++) {
		ret = nss_core_register_handler(i, nss_virt_if_msg_handler, NULL);
		if (ret != NSS_CORE_STATUS_SUCCESS) {
			nss_warning("Failed to register message handler for virtual interface : %d", i);
		}
	}
}

EXPORT_SYMBOL(nss_virt_if_assign_if_num);
EXPORT_SYMBOL(nss_virt_if_tx_msg);
EXPORT_SYMBOL(nss_virt_if_tx_nwifi_rxbuf);
EXPORT_SYMBOL(nss_virt_if_tx_eth_rxbuf);
EXPORT_SYMBOL(nss_virt_if_get_interface_num);

