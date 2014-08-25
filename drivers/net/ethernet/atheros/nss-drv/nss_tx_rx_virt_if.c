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
 * nss_tx_rx_virt_if.c
 *	NSS virtual/redirect handler APIs
 */

#include "nss_tx_rx_common.h"
#include <net/arp.h>

extern int nss_ctl_redirect;

/*
 * nss_tx_virt_if_recvbuf()
 *	HLOS interface has received a packet which we redirect to the NSS, if appropriate to do so.
 */
nss_tx_status_t nss_tx_virt_if_recvbuf(void *ctx, struct sk_buff *os_buf, uint32_t nwifi)
{
	int32_t status;
	struct nss_ctx_instance *nss_ctx = &nss_top_main.nss[nss_top_main.ipv4_handler_id];
	int32_t if_num = (int32_t)ctx;
	uint32_t bufftype;

	if (unlikely(nss_ctl_redirect == 0) || unlikely(os_buf->vlan_tci)) {
		return NSS_TX_FAILURE_NOT_SUPPORTED;
	}

	nss_assert(NSS_IS_IF_TYPE(VIRTUAL, if_num));
	nss_trace("%p: Virtual Rx packet, if_num:%d, skb:%p", nss_ctx, if_num, os_buf);

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
	if (unlikely(os_buf->len <= ETH_HLEN)) {
		nss_warning("%p: Virtual Rx packet: %p too short", nss_ctx, os_buf);
		return NSS_TX_FAILURE_TOO_SHORT;
	}

	if (unlikely(skb_shinfo(os_buf)->nr_frags != 0)) {
		/*
		 * TODO: If we have a connection matching rule for this skbuff,
		 * do we need to flush it??
		 */
		nss_warning("%p: Delivering the packet to Linux because of fragmented skb: %p\n", nss_ctx, os_buf);
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
		skb_push(os_buf, ETH_HLEN);
	}

	/*
	 * Direct the buffer to the NSS
	 */
	status = nss_core_send_buffer(nss_ctx, if_num, os_buf, NSS_IF_DATA_QUEUE, bufftype, H2N_BIT_FLAG_VIRTUAL_BUFFER);
	if (unlikely(status != NSS_CORE_STATUS_SUCCESS)) {
		nss_warning("%p: Virtual Rx packet unable to enqueue\n", nss_ctx);
		if (!nwifi) {
			skb_pull(os_buf, ETH_HLEN);
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
 * @brief Forward virtual interface packets
 *	This function expects packet with L3 header and eth_type_trans
 *	has been called before calling this api
 *
 * @param nss_ctx NSS context (provided during registeration)
 * @param os_buf OS buffer (e.g. skbuff)
 * @return nss_tx_status_t Tx status
 */
nss_tx_status_t nss_tx_virt_if_rxbuf(void *ctx, struct sk_buff *os_buf)
{

	return nss_tx_virt_if_recvbuf(ctx, os_buf, 0);
}

/*
 * @brief Forward Native wifi packet from virtual interface
 *	Expects packet with qca-nwifi format
 *
 * @param nss_ctx NSS context (provided during registeration)
 * @param os_buf OS buffer (e.g. skbuff)
 * @return nss_tx_status_t Tx status
 */
nss_tx_status_t nss_tx_virt_if_rx_nwifibuf(void *ctx, struct sk_buff *os_buf)
{

	return nss_tx_virt_if_recvbuf(ctx, os_buf, 1);
}

/*
 * nss_create_virt_if()
 */
void *nss_create_virt_if(struct net_device *if_ctx)
{
	struct nss_virt_if_msg nvim;
	struct nss_virt_if_create *nvic;
	struct nss_ctx_instance *nss_ctx = &nss_top_main.nss[nss_top_main.ipv4_handler_id];
	int32_t if_num;

	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("Interface could not be created as core not ready");
		return NULL;
	}

	/*
	 * Check if net_device is Ethernet type
	 */
	if (if_ctx->type != ARPHRD_ETHER) {
		nss_warning("%p:Register virtual interface %p: type incorrect: %d ", nss_ctx, if_ctx, if_ctx->type);
		return NULL;
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
		return NULL;
	}

	nss_cmn_msg_init(&nvim.cm, if_num, NSS_VIRT_IF_TX_CREATE_MSG,
			sizeof(struct nss_virt_if_create), NULL, NULL);

	nvic = &nvim.msg.create;
	nvic->flags = 0;
	memcpy(nvic->mac_addr, if_ctx->dev_addr, ETH_HLEN);

	(void)nss_virt_if_tx_msg(&nvim);

	/*
	 * Hold a reference to the net_device
	 */
	dev_hold(if_ctx);
	nss_info("%p:Registered virtual interface %d: context %p", nss_ctx, if_num, if_ctx);

	/*
	 * The context returned is the virtual interface # which is, essentially, the index into the if_ctx
	 * array that is holding the net_device pointer
	 */
	return (void *)if_num;
}

/*
 * nss_destroy_virt_if()
 */
nss_tx_status_t nss_destroy_virt_if(void *ctx)
{
	int32_t if_num;
	struct nss_virt_if_msg nvim;
	struct net_device *dev;
	struct nss_ctx_instance *nss_ctx = &nss_top_main.nss[nss_top_main.ipv4_handler_id];

	if (unlikely(nss_ctx->state != NSS_CORE_STATE_INITIALIZED)) {
		nss_warning("Interface could not be destroyed as core not ready");
		return NSS_TX_FAILURE_NOT_READY;
	}

	if_num = (int32_t)ctx;
	nss_assert(NSS_IS_IF_TYPE(VIRTUAL, if_num));

	spin_lock_bh(&nss_top_main.lock);
	if (!nss_top_main.if_ctx[if_num]) {
		spin_unlock_bh(&nss_top_main.lock);
		nss_warning("%p: Unregister virtual interface %d: no context", nss_ctx, if_num);
		return NSS_TX_FAILURE_BAD_PARAM;
	}

	/*
	 * Set this context to NULL
	 */
	dev = nss_top_main.if_ctx[if_num];
	nss_top_main.if_ctx[if_num] = NULL;
	spin_unlock_bh(&nss_top_main.lock);
	nss_info("%p:Unregister virtual interface %d (%p)", nss_ctx, if_num, dev);
	dev_put(dev);

	nss_cmn_msg_init(&nvim.cm, if_num, NSS_VIRT_IF_TX_DESTROY_MSG,
			sizeof(struct nss_virt_if_destroy), NULL, NULL);

	return nss_virt_if_tx_msg(&nvim);
}

EXPORT_SYMBOL(nss_tx_virt_if_rxbuf);
EXPORT_SYMBOL(nss_tx_virt_if_rx_nwifibuf);
EXPORT_SYMBOL(nss_create_virt_if);
EXPORT_SYMBOL(nss_destroy_virt_if);


