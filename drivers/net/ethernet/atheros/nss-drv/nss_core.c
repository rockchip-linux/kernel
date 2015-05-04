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
 * nss_core.c
 *	NSS driver core APIs source file.
 */

#include "nss_core.h"
#include <linux/module.h>
#include <nss_hal.h>
#include <net/dst.h>
#include <linux/etherdevice.h>
#include "nss_tx_rx_common.h"
#include "nss_data_plane.h"

#define NSS_CORE_JUMBO_LINEAR_BUF_SIZE 128

/*
 * Atomic variables to control jumbo_mru & paged_mode
 */
static atomic_t jumbo_mru;
static atomic_t paged_mode;

/*
 * local structure declarations
 */

/*
 * NSS Rx per interface callback structure
 */
struct nss_rx_cb_list {
	nss_core_rx_callback_t cb;
	void *app_data;
};

static struct nss_rx_cb_list nss_rx_interface_handlers[NSS_MAX_NET_INTERFACES];

/*
 * nss_core_set_jumbo_mru()
 *	Set the jumbo_mru to the specified value
 */
void nss_core_set_jumbo_mru(int jumbo)
{
	atomic_set(&jumbo_mru, jumbo);
}

/*
 * nss_core_get_jumbo_mru()
 *	Does an atomic read of jumbo_mru
 */
int nss_core_get_jumbo_mru(void)
{
	return atomic_read(&jumbo_mru);
}

/*
 * nss_core_set_paged_mode()
 *	Set the paged_mode to the specified value
 */
void nss_core_set_paged_mode(int mode)
{
	atomic_set(&paged_mode, mode);
}

/*
 * nss_core_get_paged_mode()
 *	Does an atomic read of paged_mode
 */
int nss_core_get_paged_mode(void)
{
	return atomic_read(&paged_mode);
}

/*
 * nss_core_register_handler()
 *	Register a callback per interface code. Only one per interface.
 */
uint32_t nss_core_register_handler(uint32_t interface, nss_core_rx_callback_t cb, void *app_data)
{
	nss_assert(cb != NULL);

	/*
	 * Validate interface id
	 */
	if (interface > NSS_MAX_NET_INTERFACES) {
		printk("Error - Interface %d not Supported\n", interface);
		return NSS_CORE_STATUS_FAILURE;
	}

	/*
	 * Check if already registered
	 */
	if (nss_rx_interface_handlers[interface].cb != NULL) {
		printk("Error - Duplicate Interface CB Registered for interface %d\n", interface);
		return NSS_CORE_STATUS_FAILURE;
	}

	nss_rx_interface_handlers[interface].cb = cb;
	nss_rx_interface_handlers[interface].app_data = app_data;

	return NSS_CORE_STATUS_SUCCESS;
}

uint32_t nss_core_unregister_handler(uint32_t interface)
{
	/*
	 * Validate interface id
	 */
	if (interface > NSS_MAX_NET_INTERFACES) {
		printk("Error - Interface %d not Supported\n", interface);
		return NSS_CORE_STATUS_FAILURE;
	}

	nss_rx_interface_handlers[interface].cb = NULL;
	nss_rx_interface_handlers[interface].app_data = NULL;

	return NSS_CORE_STATUS_SUCCESS;
}

/*
 * nss_core_handle_nss_status_pkt()
 *	Handle the metadata/status packet.
 */
void nss_core_handle_nss_status_pkt(struct nss_ctx_instance *nss_ctx, struct sk_buff *nbuf)
{
	struct nss_cmn_msg *ncm;
	uint32_t expected_version = NSS_HLOS_MESSAGE_VERSION;
	nss_core_rx_callback_t cb;
	void *app_data;

	if (skb_shinfo(nbuf)->nr_frags > 0) {
		ncm = (struct nss_cmn_msg *)skb_frag_address(&skb_shinfo(nbuf)->frags[0]);
	} else {
		ncm = (struct nss_cmn_msg *)nbuf->data;
	}

	/*
	 * Check for version number
	 */
	if (ncm->version != expected_version) {
		nss_warning("%p: Message %d for interface %d received with invalid version %d, expected version %d",
							nss_ctx, ncm->type, ncm->interface, ncm->version, expected_version);
		return;
	}

	/*
	 * Validate message size
	 */
	if (ncm->len > nbuf->len) {
		nss_warning("%p: Message %d for interface %d received with invalid length %d, expected length %d",
							nss_ctx, ncm->type, ncm->interface, nbuf->len, ncm->len);
		return;
	}

	/*
	 * Check for validity of interface number
	 */
	if (ncm->interface > NSS_MAX_NET_INTERFACES && ncm->interface < 0) {
		nss_warning("%p: Message %d received with invalid interface number %d", nss_ctx, ncm->type, ncm->interface);
		return;
	}

	cb = nss_rx_interface_handlers[ncm->interface].cb;
	app_data = nss_rx_interface_handlers[ncm->interface].app_data;

	if (!cb) {
		nss_warning("%p: Callback not registered for interface %d", nss_ctx, ncm->interface);
		return;
	}

	cb(nss_ctx, ncm, app_data);

	return;
}

/*
 * nss_send_c2c_map()
 *	Send C2C map to NSS
 */
static int32_t nss_send_c2c_map(struct nss_ctx_instance *nss_own, struct nss_ctx_instance *nss_other)
{
	struct sk_buff *nbuf;
	int32_t status;
	struct nss_c2c_msg *ncm;
	struct nss_c2c_tx_map *nctm;

	nss_info("%p: C2C map:%x\n", nss_own, nss_other->c2c_start);

	nbuf = dev_alloc_skb(NSS_NBUF_PAYLOAD_SIZE);
	if (unlikely(!nbuf)) {
		struct nss_top_instance *nss_top = nss_own->nss_top;

		NSS_PKT_STATS_INCREMENT(nss_own, &nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]);
		nss_warning("%p: Unable to allocate memory for 'C2C tx map'", nss_own);
		return NSS_CORE_STATUS_FAILURE;
	}

	ncm = (struct nss_c2c_msg *)skb_put(nbuf, sizeof(struct nss_c2c_msg));
	ncm->cm.interface = NSS_C2C_TX_INTERFACE;
	ncm->cm.version = NSS_HLOS_MESSAGE_VERSION;
	ncm->cm.type = NSS_TX_METADATA_TYPE_C2C_TX_MAP;
	ncm->cm.len = sizeof(struct nss_c2c_msg);

	nctm = &ncm->msg.tx_map;
	nctm->c2c_start = nss_other->c2c_start;
	nctm->c2c_int_addr = (uint32_t)(nss_other->nphys) + NSS_REGS_C2C_INTR_SET_OFFSET;

	status = nss_core_send_buffer(nss_own, 0, nbuf, NSS_IF_CMD_QUEUE, H2N_BUFFER_CTRL, 0);
	if (unlikely(status != NSS_CORE_STATUS_SUCCESS)) {
		dev_kfree_skb_any(nbuf);
		nss_warning("%p: Unable to enqueue 'c2c tx map'\n", nss_own);
		return NSS_CORE_STATUS_FAILURE;
	}

	nss_hal_send_interrupt(nss_own->nmap, nss_own->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit,
								NSS_REGS_H2N_INTR_STATUS_DATA_COMMAND_QUEUE);

	return NSS_CORE_STATUS_SUCCESS;
}

/*
 * nss_core_cause_to_queue()
 *	Map interrupt cause to queue id
 */
static inline uint16_t nss_core_cause_to_queue(uint16_t cause)
{
	if (likely(cause == NSS_REGS_N2H_INTR_STATUS_DATA_COMMAND_QUEUE)) {
		return NSS_IF_DATA_QUEUE_0;
	}

	if (likely(cause == NSS_REGS_N2H_INTR_STATUS_DATA_QUEUE_1)) {
		return NSS_IF_DATA_QUEUE_1;
	}

	if (likely(cause == NSS_REGS_N2H_INTR_STATUS_EMPTY_BUFFER_QUEUE)) {
		return NSS_IF_EMPTY_BUFFER_QUEUE;
	}

	/*
	 * There is no way we can reach here as cause was already identified to be related to valid queue
	 */
	nss_assert(0);
	return 0;
}

/*
 * nss_dump_desc()
 *	Prints descriptor data
 */
static inline void nss_dump_desc(struct nss_ctx_instance *nss_ctx, struct n2h_descriptor *desc)
{
	printk("bad descriptor dump for nss core = %d\n", nss_ctx->id);
	printk("\topaque = %x\n", desc->opaque);
	printk("\tinterface = %d\n", desc->interface_num);
	printk("\tbuffer_type = %d\n", desc->buffer_type);
	printk("\tbit_flags = %x\n", desc->bit_flags);
	printk("\tbuffer_addr = %x\n", desc->buffer);
	printk("\tbuffer_len = %d\n", desc->buffer_len);
	printk("\tpayload_offs = %d\n", desc->payload_offs);
	printk("\tpayload_len = %d\n", desc->payload_len);
}

/*
 * nss_core_skb_needs_linearize()
 *	Looks at if this skb needs to be linearized of not.
 */
static inline int nss_core_skb_needs_linearize(struct sk_buff *skb, uint32_t features)
{
	return skb_is_nonlinear(skb) &&
			((skb_has_frag_list(skb) &&
				!(features & NETIF_F_FRAGLIST)) ||
			(skb_shinfo(skb)->nr_frags &&
				!(features & NETIF_F_SG)));
}

/*
 * nss_core_handle_bounced_pkt()
 * 	Bounced packet is returned from an interface/bridge bounce operation.
 *
 * Return the skb to the registrant.
 */
static inline void nss_core_handle_bounced_pkt(struct nss_ctx_instance *nss_ctx,
						struct nss_shaper_bounce_registrant *reg,
						struct sk_buff *nbuf)
{
	void *app_data;
	struct module *owner;
	nss_shaper_bounced_callback_t bounced_callback;
	struct nss_top_instance *nss_top = nss_ctx->nss_top;

	spin_lock_bh(&nss_top->lock);

	/*
	 * Do we have a registrant?
	 */
	if (!reg->registered) {
		spin_unlock_bh(&nss_top->lock);
		dev_kfree_skb_any(nbuf);
		return;
	}

	/*
	 * Get handle to the owning registrant
	 */
	bounced_callback = reg->bounced_callback;
	app_data = reg->app_data;
	owner = reg->owner;

	/*
	 * Callback is active, unregistration is not permitted while this is in progress
	 */
	reg->callback_active = true;
	spin_unlock_bh(&nss_top->lock);
	if (!try_module_get(owner)) {
		spin_lock_bh(&nss_top->lock);
		reg->callback_active = false;
		spin_unlock_bh(&nss_top->lock);
		dev_kfree_skb_any(nbuf);
		return;
	}

	/*
	 * Pass bounced packet back to registrant
	 */
	bounced_callback(app_data, nbuf);
	spin_lock_bh(&nss_top->lock);
	reg->callback_active = false;
	spin_unlock_bh(&nss_top->lock);
	module_put(owner);
}

/*
 * nss_core_handle_virt_if_pkt()
 *	Handle packet destined to virtual interface.
 */
static inline void nss_core_handle_virt_if_pkt(struct nss_ctx_instance *nss_ctx,
						unsigned int interface_num,
						struct sk_buff *nbuf)
{
	struct nss_top_instance *nss_top = nss_ctx->nss_top;
	struct nss_subsystem_dataplane_register *subsys_dp_reg = &nss_top->subsys_dp_register[interface_num];
	struct net_device *ndev = NULL;

	uint32_t xmit_ret;

	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_top->stats_drv[NSS_STATS_DRV_RX_VIRTUAL]);

	/*
	 * Checksum is already done by NSS for packets forwarded to virtual interfaces
	 */
	nbuf->ip_summed = CHECKSUM_NONE;

	/*
	 * Obtain net_device pointer
	 */
	ndev = subsys_dp_reg->ndev;
	if (unlikely(ndev == NULL)) {
		nss_warning("%p: Received packet for unregistered virtual interface %d",
			nss_ctx, interface_num);

		/*
		 * NOTE: The assumption is that gather support is not
		 * implemented in fast path and hence we can not receive
		 * fragmented packets and so we do not need to take care
		 * of freeing a fragmented packet
		 */
		dev_kfree_skb_any(nbuf);
		return;
	}

	/*
	 * TODO: Need to ensure the ndev is not removed before we take dev_hold().
	 */
	dev_hold(ndev);
	nbuf->dev = ndev;
	/*
	 * Linearize the skb if needed
	 */
	 if (nss_core_skb_needs_linearize(nbuf, (uint32_t)netif_skb_features(nbuf)) && __skb_linearize(nbuf)) {
		/*
		 * We needed to linearize, but __skb_linearize() failed. Therefore
		 * we free the nbuf.
		 */
		dev_put(ndev);
		dev_kfree_skb_any(nbuf);
		return;
	}

	/*
	 * Send the packet to virtual interface
	 * NOTE: Invoking this will BYPASS any assigned QDisc - this is OKAY
	 * as TX packets out of the NSS will have been shaped inside the NSS.
	 */
	xmit_ret = ndev->netdev_ops->ndo_start_xmit(nbuf, ndev);
	if (unlikely(xmit_ret == NETDEV_TX_BUSY)) {
		dev_kfree_skb_any(nbuf);
		nss_info("%p: Congestion at virtual interface %d, %p", nss_ctx, interface_num, ndev);
	}
	dev_put(ndev);
}

/*
 * nss_core_handle_buffer_pkt()
 * 	Handle data packet received on physical or virtual interface.
 */
static inline void nss_core_handle_buffer_pkt(struct nss_ctx_instance *nss_ctx,
						unsigned int interface_num,
						struct sk_buff *nbuf,
						struct napi_struct *napi,
						uint16_t flags)
{
	struct nss_top_instance *nss_top = nss_ctx->nss_top;
	struct nss_subsystem_dataplane_register *subsys_dp_reg = &nss_top->subsys_dp_register[interface_num];
	uint32_t netif_flags = subsys_dp_reg->features;
	struct net_device *ndev = NULL;
	nss_phys_if_rx_callback_t cb;

	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_top->stats_drv[NSS_STATS_DRV_RX_PACKET]);

	/*
	 * Check if NSS was able to obtain checksum
	 */
	nbuf->ip_summed = CHECKSUM_UNNECESSARY;
	if (unlikely(!(flags & N2H_BIT_FLAG_IP_TRANSPORT_CHECKSUM_VALID))) {
		nbuf->ip_summed = CHECKSUM_NONE;
	}

	ndev = subsys_dp_reg->ndev;
	cb = subsys_dp_reg->cb;
	if (likely(cb) && likely(ndev)) {
		/*
		 * Packet was received on Physical interface
		 */
		if (nss_core_skb_needs_linearize(nbuf, netif_flags) && __skb_linearize(nbuf)) {
			/*
			 * We needed to linearize, but __skb_linearize() failed. So free the nbuf.
			 */
			dev_kfree_skb_any(nbuf);
			return;
		}

		cb(ndev, (void *)nbuf, napi);
		return;
	}

	if (NSS_IS_IF_TYPE(DYNAMIC, interface_num) || NSS_IS_IF_TYPE(VIRTUAL, interface_num)) {
		/*
		 * Packet was received on Virtual interface
		 */

		/*
		 * Give the packet to stack
		 *
		 * TODO: Change to gro receive later
		 */
		ndev = subsys_dp_reg->ndev;
		if (ndev) {
			dev_hold(ndev);
			nbuf->dev = ndev;
			nbuf->protocol = eth_type_trans(nbuf, ndev);
			netif_receive_skb(nbuf);
			dev_put(ndev);
		} else {
			/*
			 * Interface has gone down
			 */
			nss_warning("%p: Received exception packet from bad virtual interface %d",
					nss_ctx, interface_num);
			dev_kfree_skb_any(nbuf);
		}
		return;
	}

	dev_kfree_skb_any(nbuf);
}

/*
 * nss_core_rx_pbuf()
 *	Receive a pbuf from the NSS into Linux.
 */
static inline void nss_core_rx_pbuf(struct nss_ctx_instance *nss_ctx, struct n2h_descriptor *desc, struct napi_struct *napi, uint8_t buffer_type, struct sk_buff *nbuf)
{
	unsigned int interface_num = desc->interface_num;
	struct nss_top_instance *nss_top = nss_ctx->nss_top;
	struct nss_shaper_bounce_registrant *reg = NULL;

	NSS_PKT_STATS_DECREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NSS_SKB_COUNT]);

	switch (buffer_type) {
	case N2H_BUFFER_SHAPER_BOUNCED_INTERFACE:
		reg = &nss_top->bounce_interface_registrants[interface_num];
		nss_core_handle_bounced_pkt(nss_ctx, reg, nbuf);
		break;
	case N2H_BUFFER_SHAPER_BOUNCED_BRIDGE:
		reg = &nss_top->bounce_bridge_registrants[interface_num];
		nss_core_handle_bounced_pkt(nss_ctx, reg, nbuf);
		break;
	case N2H_BUFFER_PACKET_VIRTUAL:
		nss_core_handle_virt_if_pkt(nss_ctx, interface_num, nbuf);
		break;

	case N2H_BUFFER_PACKET:
		nss_core_handle_buffer_pkt(nss_ctx, interface_num, nbuf, napi, desc->bit_flags);
		break;

	case N2H_BUFFER_STATUS:
		NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_top->stats_drv[NSS_STATS_DRV_RX_STATUS]);
		nss_core_handle_nss_status_pkt(nss_ctx, nbuf);
		dev_kfree_skb_any(nbuf);
		break;

	case N2H_BUFFER_EMPTY:
		NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_top->stats_drv[NSS_STATS_DRV_RX_EMPTY]);

		/*
		 * Warning: On non-Krait HW, we need to unmap fragments.
		 *
		 * It's not a problem on Krait HW. We don't save dma handle for those
		 * fragments and that's why we are not able to unmap. However, on
		 * Kraits dma_map_single() does not allocate any resource and hence unmap is a
		 * NOP and does not have to free up any resource.
		 */
		dev_kfree_skb_any(nbuf);
		break;

	default:
		nss_warning("%p: Invalid buffer type %d received from NSS", nss_ctx, buffer_type);
	}
}

/*
 * nss_core_handle_nrfrag_skb()
 *	Handled the processing of fragmented skb's
 */
static inline bool nss_core_handle_nr_frag_skb(struct sk_buff **nbuf_ptr, struct sk_buff **jumbo_start_ptr, struct n2h_descriptor *desc, unsigned int buffer_type)
{
	struct sk_buff *nbuf = *nbuf_ptr;
	struct sk_buff *jumbo_start = *jumbo_start_ptr;

	uint16_t payload_len = desc->payload_len;
	uint16_t payload_offs = desc->payload_offs;
	uint16_t bit_flags = desc->bit_flags;

	nss_assert(desc->payload_offs + desc->payload_len <= PAGE_SIZE, "%p: payload offset %u + payload len %u larger than page size %u", nbuf, desc->payload_offs, desc->payload_len, PAGE_SIZE);
	dma_unmap_page(NULL, (desc->buffer + desc->payload_offs), desc->payload_len, DMA_FROM_DEVICE);

	/*
	 * The first and last bits are both set. Hence the received frame can't have
	 * chains (or it's not a scattered one).
	 */
	if (likely(bit_flags & N2H_BIT_FLAG_FIRST_SEGMENT) && likely(bit_flags & N2H_BIT_FLAG_LAST_SEGMENT)) {

		/*
		 * We have received another head before we saw the last segment.
		 * Free the old head as the frag list is corrupt.
		 */
		if (unlikely(jumbo_start)) {
			nss_warning("%p: received the second head before a last", jumbo_start);
			dev_kfree_skb_any(jumbo_start);
			*jumbo_start_ptr = NULL;
			return false;
		}

		/*
		 * NOTE: Need to use __skb_fill since we do not want to
		 * increment nr_frags again. We just want to adjust the offset
		 * and the length.
		 */
		__skb_fill_page_desc(nbuf, 0, skb_frag_page(&skb_shinfo(nbuf)->frags[0]), payload_offs, payload_len);

		/*
		 * We do not update truesize. We just keep the initial set value.
		 */
		nbuf->data_len = payload_len;
		nbuf->len = payload_len;
		goto pull;
	}

	/*
	 * NSS sent us an SG chain.
	 * Build a frags[] out of segments.
	 */
	if (unlikely((bit_flags & N2H_BIT_FLAG_FIRST_SEGMENT))) {

		/*
		 * We have received another head before we saw the last segment.
		 * Free the old head as the frag list is corrupt.
		 */
		if (unlikely(jumbo_start)) {
			nss_warning("%p: received the second head before a last", jumbo_start);
			dev_kfree_skb_any(jumbo_start);
			*jumbo_start_ptr = NULL;
			return false;
		}

		/*
		 * We do not update truesize. We just keep the initial set value.
		 */
		__skb_fill_page_desc(nbuf, 0, skb_frag_page(&skb_shinfo(nbuf)->frags[0]), payload_offs, payload_len);
		nbuf->data_len = payload_len;
		nbuf->len = payload_len;

		/*
		 * Set jumbo pointer to nbuf
		 */
		*jumbo_start_ptr = nbuf;

		/*
		 * Skip sending until last is received.
		 */
		return false;
	}

	/*
	 * We've received a middle or a last segment.
	 * Check that we have received a head first to avoid null deferencing.
	 */
	if (unlikely(jumbo_start == NULL)) {
		/*
		 * Middle before first! Free the middle.
		 */
		nss_warning("%p: saw a middle skb before head", nbuf);
		dev_kfree_skb_any(nbuf);
		return false;
	}

	/*
	 * Free the skb after attaching the frag to the head skb.
	 * Our page is safe although we are freeing it because we
	 * just took a reference to it.
	 */
	skb_add_rx_frag(jumbo_start, skb_shinfo(jumbo_start)->nr_frags, skb_frag_page(&skb_shinfo(nbuf)->frags[0]), payload_offs, payload_len, PAGE_SIZE);
	skb_frag_ref(jumbo_start, skb_shinfo(jumbo_start)->nr_frags - 1);
	dev_kfree_skb_any(nbuf);

	if (!(bit_flags & N2H_BIT_FLAG_LAST_SEGMENT)) {
		/*
		 * Skip sending until last is received.
		 */
		return false;
	}

	/*
	 * Last is received. Set nbuf pointer to point to
	 * the jumbo skb so that it continues to get processed.
	 */
	nbuf = jumbo_start;
	*nbuf_ptr = nbuf;
	*jumbo_start_ptr = NULL;
	prefetch((void *)(nbuf->data));

pull:
	/*
	 * We need eth hdr to be in the linear part of the skb
	 * for data packets. Otherwise eth_type_trans fails.
	 */
	if (buffer_type != N2H_BUFFER_STATUS) {
		if (!pskb_may_pull(nbuf, ETH_HLEN)) {
			dev_kfree_skb(nbuf);
			nss_warning("%p: could not pull eth header", nbuf);
			return false;
		}
	}

	return true;
}

/*
 * nss_core_handle_linear_skb()
 *	Handler for processing linear skbs.
 */
static inline bool nss_core_handle_linear_skb(struct nss_ctx_instance *nss_ctx, struct sk_buff **nbuf_ptr, struct sk_buff **head_ptr,
						struct sk_buff **tail_ptr, struct n2h_descriptor *desc)
{
	uint16_t bit_flags = desc->bit_flags;
	struct sk_buff *nbuf = *nbuf_ptr;
	struct sk_buff *head = *head_ptr;
	struct sk_buff *tail = *tail_ptr;

	/*
	 * We are in linear SKB mode.
	 */
	nbuf->data = nbuf->head + desc->payload_offs;
	nbuf->len = desc->payload_len;
	nbuf->tail = nbuf->data + nbuf->len;
	dma_unmap_single(NULL, (desc->buffer + desc->payload_offs), desc->payload_len, DMA_FROM_DEVICE);
	prefetch((void *)(nbuf->data));

	if (likely(bit_flags & N2H_BIT_FLAG_FIRST_SEGMENT) && likely(bit_flags & N2H_BIT_FLAG_LAST_SEGMENT)) {

		/*
		 * We have received another head before we saw the last segment.
		 * Free the old head as the frag list is corrupt.
		 */
		if (unlikely(head)) {
			nss_warning("%p: received the second head before a last", head);
			dev_kfree_skb_any(head);
			*head_ptr = NULL;
			return false;
		}

		/*
		 * TODO: Check if there is any issue wrt map and unmap,
		 * NSS should playaround with data area and should not
		 * touch HEADROOM area
		 */
		NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_RX_SIMPLE]);
		return true;
	}

	/*
	 * NSS sent us an SG chain.
	 * Build a frag list out of segments.
	 */
	if (unlikely((bit_flags & N2H_BIT_FLAG_FIRST_SEGMENT))) {

		/*
		 * We have received another head before we saw the last segment.
		 * Free the old head as the frag list is corrupt.
		 */
		if (unlikely(head)) {
			nss_warning("%p: received the second head before a last", head);
			dev_kfree_skb_any(head);
			*head_ptr = NULL;
			return false;
		}

		/*
		 * Found head.
		 */
		if (unlikely(skb_has_frag_list(nbuf))) {
			/*
			 * We don't support chain in a chain.
			 */
			nss_warning("%p: skb already has a fraglist", nbuf);
			dev_kfree_skb_any(nbuf);
			return false;
		}

		skb_frag_list_init(nbuf);
		nbuf->data_len = 0;
		nbuf->truesize = desc->payload_len;

		*head_ptr = nbuf;

		/*
		 * Skip sending until last is received.
		 */
		return false;
	}

	/*
	 * We've received a middle segment.
	 * Check that we have received a head first to avoid null deferencing.
	 */
	if (unlikely(head == NULL)) {

		/*
		 * Middle before first! Free the middle.
		 */
		nss_warning("%p: saw a middle skb before head", nbuf);
		dev_kfree_skb_any(nbuf);

		return false;
	}

	if (!skb_has_frag_list(head)) {
		/*
		 * 2nd skb in the chain. head's frag_list should point to him.
		 */
		skb_frag_add_head(head, nbuf);
	} else {
		/*
		 * 3rd, 4th... skb in the chain. The chain's previous tail's
		 * next should point to him.
		 */
		tail->next = nbuf;
		nbuf->next = NULL;
	}
	*tail_ptr = nbuf;

	/*
	 * Now we've added a new nbuf to the chain.
	 * Update the chain length.
	 */
	head->data_len += desc->payload_len;
	head->len += desc->payload_len;
	head->truesize += desc->payload_len;

	if (!(bit_flags & N2H_BIT_FLAG_LAST_SEGMENT)) {
		/*
		 * Skip sending until last is received.
		 */
		return false;
	}

	/*
	 * Last is received. Send the frag_list.
	 */
	*nbuf_ptr = head;
	*head_ptr = NULL;
	*tail_ptr = NULL;
	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_RX_SKB_FRAGLIST]);
	return true;
}

/*
 * nss_core_handle_cause_queue()
 *	Handle interrupt cause related to N2H/H2N queues
 */
static int32_t nss_core_handle_cause_queue(struct int_ctx_instance *int_ctx, uint16_t cause, int16_t weight)
{
	int16_t count, count_temp;
	uint16_t size, mask, qid;
	uint32_t nss_index, hlos_index;
	struct sk_buff *nbuf, *head, *tail, *jumbo_start;
	struct hlos_n2h_desc_ring *n2h_desc_ring;
	struct n2h_desc_if_instance *desc_if;
	struct n2h_descriptor *desc_ring;
	struct n2h_descriptor *desc;
	struct nss_ctx_instance *nss_ctx = int_ctx->nss_ctx;
	struct nss_if_mem_map *if_map = (struct nss_if_mem_map *)nss_ctx->vmap;

	qid = nss_core_cause_to_queue(cause);

	/*
	 * Make sure qid < num_rings
	 */
	nss_assert(qid < if_map->n2h_rings);

	n2h_desc_ring = &nss_ctx->n2h_desc_ring[qid];
	desc_if = &n2h_desc_ring->desc_if;
	desc_ring = desc_if->desc;
	nss_index = if_map->n2h_nss_index[qid];
	hlos_index = n2h_desc_ring->hlos_index;
	size = desc_if->size;
	mask = size - 1;

	/*
	 * Check if there is work to be done for this queue
	 */
	count = ((nss_index - hlos_index) + size) & (mask);
	if (unlikely(count == 0)) {
		return 0;
	}

	/*
	 * Restrict ourselves to suggested weight
	 */
	if (count > weight) {
		count = weight;
	}

	head = tail = jumbo_start = NULL;
	count_temp = count;
	while (count_temp) {
		unsigned int buffer_type;
		uint32_t opaque;
		uint16_t bit_flags;

		desc = &desc_ring[hlos_index];
		buffer_type = desc->buffer_type;
		opaque = desc->opaque;
		bit_flags = desc->bit_flags;
		if (unlikely((buffer_type == N2H_BUFFER_CRYPTO_RESP))) {
			NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_RX_CRYPTO_RESP]);
			/*
			 * This is a crypto buffer hence send it to crypto driver
			 *
			 * NOTE: Crypto buffers require special handling as they do not
			 *	use OS network buffers (e.g. skb). Hence, OS buffer operations
			 *	are not applicable to crypto buffers
			 */
			nss_crypto_buf_handler(nss_ctx, (void *)opaque, desc->buffer, desc->payload_len);
			goto next;
		}

		/*
		 * Obtain nbuf
		 */
		nbuf = (struct sk_buff *)opaque;
		if (unlikely(nbuf < (struct sk_buff *)PAGE_OFFSET)) {
			/*
			 * Invalid opaque pointer
			 */
			nss_dump_desc(nss_ctx, desc);
			NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_RX_BAD_DESCRIPTOR]);
			goto next;
		}

		/*
		 * Handle Empty Buffer Returns.
		 */
		if (unlikely(buffer_type == N2H_BUFFER_EMPTY)) {
			/*
			 * Since we only return the primary skb, we have no way to unmap
			 * properly. Simple skb's are properly mapped but page data skbs
			 * have the payload mapped (and not the skb->data slab payload).
			 *
			 * Warning: On non-Krait HW, we need to unmap fragments.
			 *
			 * This only unmaps the first segment either slab payload or
			 * skb page data. Eventually, we need to unmap all of a frag_list
			 * or all of page_data however this is not a big concern as of now
			 * since on Kriats dma_map_single() does not allocate any resource
			 * and hence dma_unmap_single() is sort off a nop.
			 *
			 * No need to invalidate for Tx Completions, so set dma direction = DMA_TO_DEVICE;
			 * Similarly prefetch is not needed for an empty buffer.
			 */
			dma_unmap_single(NULL, (desc->buffer + desc->payload_offs), desc->payload_len, DMA_TO_DEVICE);
			goto consume;
		}

		/*
		 * Shaping uses the singleton approach as well. No need to unmap all the segments since only
		 * one of them is actually looked at.
		 */
		if ((unlikely(buffer_type == N2H_BUFFER_SHAPER_BOUNCED_INTERFACE)) || (unlikely(buffer_type == N2H_BUFFER_SHAPER_BOUNCED_BRIDGE))) {
			dma_unmap_page(NULL, (desc->buffer + desc->payload_offs), desc->payload_len, DMA_TO_DEVICE);
			goto consume;
		}

		/*
		 * Check if we are received a paged skb.
		 */
		if (skb_shinfo(nbuf)->nr_frags > 0) {
			/*
			 * Check if we received paged skb while constructing
			 * a linear skb chain. If so we need to free.
			 */
			if (unlikely(head)) {
				nss_warning("%p: we should not have an incomplete paged skb while"
								" constructing a linear skb %p", nbuf, head);

				NSS_PKT_STATS_DECREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NSS_SKB_COUNT]);
				dev_kfree_skb_any(head);
				head = NULL;
				goto next;
			}

			if (!nss_core_handle_nr_frag_skb(&nbuf, &jumbo_start, desc, buffer_type)) {
				goto next;
			}
			NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_RX_NR_FRAGS]);
			goto consume;
		}

		/*
		 * Check if we received a linear skb while constructing
		 * a paged skb. If so we need to free.
		 */
		if (unlikely(jumbo_start)) {
			nss_warning("%p: we should not have an incomplete linear skb while"
							" constructing a paged skb %p", nbuf, jumbo_start);

			NSS_PKT_STATS_DECREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NSS_SKB_COUNT]);
			dev_kfree_skb_any(jumbo_start);
			jumbo_start = NULL;
			goto next;
		}

		/*
		 * This is a simple linear skb. Use the the linear skb
		 * handler to process it.
		 */
		if (!nss_core_handle_linear_skb(nss_ctx, &nbuf, &head, &tail, desc)) {
			goto next;
		}

consume:
		nss_core_rx_pbuf(nss_ctx, desc, &(int_ctx->napi), buffer_type, nbuf);

next:
		hlos_index = (hlos_index + 1) & (mask);
		count_temp--;
	}

	n2h_desc_ring->hlos_index = hlos_index;
	if_map->n2h_hlos_index[qid] = hlos_index;
	return count;
}

/*
 * nss_core_init_nss()
 *	Initialize NSS core state
 */
static void nss_core_init_nss(struct nss_ctx_instance *nss_ctx, struct nss_if_mem_map *if_map)
{
	int32_t i;
	struct nss_top_instance *nss_top;

	/*
	 * NOTE: A commonly found error is that sizes and start address of per core
	 *	virtual register map do not match in NSS and HLOS builds. This will lead
	 *	to some hard to trace issues such as spinlock magic check failure etc.
	 *	Following checks verify that proper virtual map has been initialized
	 */
	nss_assert(if_map->magic == DEV_MAGIC);

	/*
	 * Copy ring addresses to cacheable locations.
	 * We do not wish to read ring start address through NC accesses
	 */
	for (i = 0; i < if_map->n2h_rings; i++) {
		struct hlos_n2h_desc_ring *n2h_desc_ring = &nss_ctx->n2h_desc_ring[i];
		n2h_desc_ring->desc_if.desc =
			(struct n2h_descriptor *)((uint32_t)if_map->n2h_desc_if[i].desc - (uint32_t)nss_ctx->vphys + (uint32_t)nss_ctx->vmap);
		n2h_desc_ring->desc_if.size = if_map->n2h_desc_if[i].size;
		n2h_desc_ring->desc_if.int_bit = if_map->n2h_desc_if[i].int_bit;
		n2h_desc_ring->hlos_index = if_map->n2h_hlos_index[i];
	}

	for (i = 0; i < if_map->h2n_rings; i++) {
		struct hlos_h2n_desc_rings *h2n_desc_ring = &nss_ctx->h2n_desc_rings[i];
		h2n_desc_ring->desc_ring.desc =
			(struct h2n_descriptor *)((uint32_t)if_map->h2n_desc_if[i].desc - (uint32_t)nss_ctx->vphys + (uint32_t)nss_ctx->vmap);
		h2n_desc_ring->desc_ring.size = if_map->h2n_desc_if[i].size;
		h2n_desc_ring->desc_ring.int_bit = if_map->h2n_desc_if[i].int_bit;
		h2n_desc_ring->hlos_index = if_map->h2n_hlos_index[i];
		spin_lock_init(&h2n_desc_ring->lock);
	}

	nss_ctx->c2c_start = if_map->c2c_start;

	nss_top = nss_ctx->nss_top;
	spin_lock_bh(&nss_top->lock);
	nss_ctx->state = NSS_CORE_STATE_INITIALIZED;
	spin_unlock_bh(&nss_top->lock);

	/*
	 * If nss core0 is up, then we are ready to hook to nss-gmac
	 */
	if (nss_ctx->id == 0) {
		for (i = 0; i < NSS_MAX_PHYSICAL_INTERFACES; i++) {
			if (nss_data_plane_register_to_nss_gmac(nss_ctx, i)) {
				nss_info("Register data plan to gmac%d success\n", i);
			}
		}
	}
}

/*
 * nss_core_handle_cause_nonqueue()
 *	Handle non-queue interrupt causes (e.g. empty buffer SOS, Tx unblocked)
 */
static void nss_core_handle_cause_nonqueue(struct int_ctx_instance *int_ctx, uint32_t cause, int16_t weight)
{
	struct nss_ctx_instance *nss_ctx = int_ctx->nss_ctx;
	struct nss_if_mem_map *if_map = (struct nss_if_mem_map *)(nss_ctx->vmap);
	uint16_t max_buf_size = (uint16_t) nss_ctx->max_buf_size;
	int32_t i;

	nss_assert((cause == NSS_REGS_N2H_INTR_STATUS_EMPTY_BUFFERS_SOS) || (cause == NSS_REGS_N2H_INTR_STATUS_TX_UNBLOCKED));

	/*
	 * TODO: find better mechanism to handle empty buffers
	 */
	if (likely(cause == NSS_REGS_N2H_INTR_STATUS_EMPTY_BUFFERS_SOS)) {
		struct sk_buff *nbuf;
		struct page *npage;
		uint16_t count, size, mask;
		int32_t nss_index, hlos_index;
		struct hlos_h2n_desc_rings *h2n_desc_ring = &nss_ctx->h2n_desc_rings[NSS_IF_EMPTY_BUFFER_QUEUE];
		struct h2n_desc_if_instance *desc_if = &h2n_desc_ring->desc_ring;
		struct h2n_descriptor *desc_ring;
		struct nss_top_instance *nss_top = nss_ctx->nss_top;
		int paged_mode = nss_core_get_paged_mode();
		int jumbo_mru = nss_core_get_jumbo_mru();


		/*
		 * If this is the first time we are receiving this interrupt then
		 * we need to initialize local state of NSS core. This helps us save an
		 * interrupt cause bit. Hopefully, unlikley and branch prediction algorithm
		 * of processor will prevent any excessive penalties.
		 */
		if (unlikely(nss_ctx->state == NSS_CORE_STATE_UNINITIALIZED)) {
			nss_core_init_nss(nss_ctx, if_map);

			/*
			 * Pass C2C addresses of already brought up cores to the recently brought
			 * up core. No NSS core knows the state of other other cores in system so
			 * NSS driver needs to mediate and kick start C2C between them
			 */
#if (NSS_MAX_CORES > 1)
			for (i = 0; i < NSS_MAX_CORES; i++) {
				/*
				 * Loop through all NSS cores and send exchange C2C addresses
				 * TODO: Current implementation utilizes the fact that there are
				 *	only two cores in current design. And ofcourse ignore
				 *	the core that we are trying to initialize.
				 */
				if (&nss_top->nss[i] != nss_ctx) {
					/*
					 * Block initialization routine of any other NSS cores running on other
					 * processors. We do not want them to mess around with their initialization
					 * state and C2C addresses while we check their state.
					 */
					spin_lock_bh(&nss_top->lock);
					if (nss_top->nss[i].state == NSS_CORE_STATE_INITIALIZED) {
						spin_unlock_bh(&nss_top->lock);
						nss_send_c2c_map(&nss_top->nss[i], nss_ctx);
						nss_send_c2c_map(nss_ctx, &nss_top->nss[i]);
						continue;
					}
					spin_unlock_bh(&nss_top->lock);
				}
			}
#endif
		}

		/*
		 * Do this late in case we weren't actually initialized before.
		 */
		desc_ring = desc_if->desc;

		/*
		 * Check how many empty buffers could be filled in queue
		 */
		nss_index = if_map->h2n_nss_index[NSS_IF_EMPTY_BUFFER_QUEUE];
		hlos_index = h2n_desc_ring->hlos_index;
		size = h2n_desc_ring->desc_ring.size;

		mask = size - 1;
		count = ((nss_index - hlos_index - 1) + size) & (mask);

		nss_trace("%p: Adding %d buffers to empty queue", nss_ctx, count);

		/*
		 * Fill empty buffer queue with buffers leaving one empty descriptor
		 * Note that total number of descriptors in queue cannot be more than (size - 1)
		 */
		while (count) {
			struct h2n_descriptor *desc = &desc_ring[hlos_index];
			dma_addr_t buffer;

			if (paged_mode) {
				/*
				 * Alloc an skb AND a page.
				 */
				nbuf = dev_alloc_skb(NSS_CORE_JUMBO_LINEAR_BUF_SIZE);
				if (unlikely(!nbuf)) {
					/*
					 * ERR:
					 */
					NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]);
					nss_warning("%p: Could not obtain empty buffer", nss_ctx);
					break;
				}

				npage = alloc_page(GFP_ATOMIC);
				if (unlikely(!npage)) {
					/*
					 * ERR:
					 */
					dev_kfree_skb_any(nbuf);
					NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]);
					nss_warning("%p: Could not obtain empty page", nss_ctx);
					break;
				}

				/*
				 * When we alloc an skb, initially head = data = tail and len = 0.
				 * So nobody will try to read the linear part of the skb.
				 */
				skb_fill_page_desc(nbuf, 0, npage, 0, PAGE_SIZE);
				nbuf->data_len += PAGE_SIZE;
				nbuf->len += PAGE_SIZE;
				nbuf->truesize += PAGE_SIZE;

				/* Map the page for jumbo */
				buffer = dma_map_page(NULL, npage, 0, PAGE_SIZE, DMA_FROM_DEVICE);
				desc->buffer_len = PAGE_SIZE;
				desc->payload_offs = 0;

			} else if (jumbo_mru) {
				nbuf = dev_alloc_skb(jumbo_mru);
				if (unlikely(!nbuf)) {
					/*
					 * ERR:
					 */
					NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]);
					nss_warning("%p: Could not obtain empty jumbo mru buffer", nss_ctx);
					break;
				}

				/*
				 * Map the skb
				 */
				buffer = dma_map_single(NULL, nbuf->head, jumbo_mru, DMA_FROM_DEVICE);
				desc->buffer_len = jumbo_mru;
				desc->payload_offs = (uint16_t) (nbuf->data - nbuf->head);

			} else {
				nbuf = dev_alloc_skb(max_buf_size);
				if (unlikely(!nbuf)) {
					/*
					 * ERR:
					 */
					NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]);
					nss_warning("%p: Could not obtain empty buffer", nss_ctx);
					break;
				}

				/*
				 * Map the skb
				 */
				buffer = dma_map_single(NULL, nbuf->head, max_buf_size, DMA_FROM_DEVICE);
				desc->buffer_len = max_buf_size;
				desc->payload_offs = (uint16_t) (nbuf->data - nbuf->head);
			}

			if (unlikely(dma_mapping_error(NULL, buffer))) {
				/*
				 * ERR:
				 */
				dev_kfree_skb_any(nbuf);
				nss_warning("%p: DMA mapping failed for empty buffer", nss_ctx);
				break;
			}

			/*
			 * We are holding this skb in NSS FW, let kmemleak know about it
			 */
			kmemleak_not_leak(nbuf);
			NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NSS_SKB_COUNT]);

			desc->opaque = (uint32_t)nbuf;
			desc->buffer = buffer;
			desc->buffer_type = H2N_BUFFER_EMPTY;
			hlos_index = (hlos_index + 1) & (mask);
			count--;
		}

		h2n_desc_ring->hlos_index = hlos_index;
		if_map->h2n_hlos_index[NSS_IF_EMPTY_BUFFER_QUEUE] = hlos_index;

		/*
		 * Inform NSS that new buffers are available
		 */
		nss_hal_send_interrupt(nss_ctx->nmap, desc_if->int_bit, NSS_REGS_H2N_INTR_STATUS_EMPTY_BUFFER_QUEUE);
		NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_top->stats_drv[NSS_STATS_DRV_TX_EMPTY]);
	} else if (cause == NSS_REGS_N2H_INTR_STATUS_TX_UNBLOCKED) {
		nss_trace("%p: Data queue unblocked", nss_ctx);

		/*
		 * Call callback functions of drivers that have registered with us
		 */
		spin_lock_bh(&nss_ctx->decongest_cb_lock);

		for (i = 0; i< NSS_MAX_CLIENTS; i++) {
			if (nss_ctx->queue_decongestion_callback[i]) {
				nss_ctx->queue_decongestion_callback[i](nss_ctx->queue_decongestion_ctx[i]);
			}
		}

		spin_unlock_bh(&nss_ctx->decongest_cb_lock);
		nss_ctx->h2n_desc_rings[NSS_IF_DATA_QUEUE_0].flags &= ~NSS_H2N_DESC_RING_FLAGS_TX_STOPPED;

		/*
		 * Mask Tx unblocked interrupt and unmask it again when queue full condition is reached
		 */
		nss_hal_disable_interrupt(nss_ctx->nmap, nss_ctx->int_ctx[0].irq,
				nss_ctx->int_ctx[0].shift_factor, NSS_REGS_N2H_INTR_STATUS_TX_UNBLOCKED);
	}
}

/*
 * nss_core_get_prioritized_cause()
 *	Obtain proritized cause (from multiple interrupt causes) that
 *	must be handled by NSS driver before other causes
 */
static uint32_t nss_core_get_prioritized_cause(uint32_t cause, uint32_t *type, int16_t *weight)
{
	*type = NSS_INTR_CAUSE_INVALID;
	*weight = 0;

	/*
	 * NOTE: This is a very simple algorithm with fixed weight and strict priority
	 *
	 * TODO: Modify the algorithm later with proper weights and Round Robin
	 */

	if (cause & NSS_REGS_N2H_INTR_STATUS_EMPTY_BUFFERS_SOS) {
		*type = NSS_INTR_CAUSE_NON_QUEUE;
		*weight = NSS_EMPTY_BUFFER_SOS_PROCESSING_WEIGHT;
		return NSS_REGS_N2H_INTR_STATUS_EMPTY_BUFFERS_SOS;
	}

	if (cause & NSS_REGS_N2H_INTR_STATUS_EMPTY_BUFFER_QUEUE) {
		*type = NSS_INTR_CAUSE_QUEUE;
		*weight = NSS_EMPTY_BUFFER_RETURN_PROCESSING_WEIGHT;
		return NSS_REGS_N2H_INTR_STATUS_EMPTY_BUFFER_QUEUE;
	}

	if (cause & NSS_REGS_N2H_INTR_STATUS_TX_UNBLOCKED) {
		*type = NSS_INTR_CAUSE_NON_QUEUE;
		*weight = NSS_TX_UNBLOCKED_PROCESSING_WEIGHT;
		return NSS_REGS_N2H_INTR_STATUS_TX_UNBLOCKED;
	}

	if (cause & NSS_REGS_N2H_INTR_STATUS_DATA_COMMAND_QUEUE) {
		*type = NSS_INTR_CAUSE_QUEUE;
		*weight = NSS_DATA_COMMAND_BUFFER_PROCESSING_WEIGHT;
		return NSS_REGS_N2H_INTR_STATUS_DATA_COMMAND_QUEUE;
	}

	if (cause & NSS_REGS_N2H_INTR_STATUS_DATA_QUEUE_1) {
		*type = NSS_INTR_CAUSE_QUEUE;
		*weight = NSS_DATA_COMMAND_BUFFER_PROCESSING_WEIGHT;
		return NSS_REGS_N2H_INTR_STATUS_DATA_QUEUE_1;
	}

	if (cause & NSS_REGS_H2N_INTR_STATUS_COREDUMP_END) {
		printk("COREDUMP SIGNAL END");
		return NSS_REGS_H2N_INTR_STATUS_COREDUMP_END;
	}

	return 0;
}

/*
 * nss_core_handle_napi()
 *	NAPI handler for NSS
 */
int nss_core_handle_napi(struct napi_struct *napi, int budget)
{
	int16_t processed, weight, count = 0;
	uint32_t prio_cause, int_cause, cause_type;
	struct netdev_priv_instance *ndev_priv = netdev_priv(napi->dev);
	struct int_ctx_instance *int_ctx = ndev_priv->int_ctx;
	struct nss_ctx_instance *nss_ctx = int_ctx->nss_ctx;

	/*
	 * Read cause of interrupt
	 */
	nss_hal_read_interrupt_cause(nss_ctx->nmap, int_ctx->irq, int_ctx->shift_factor, &int_cause);
	nss_hal_clear_interrupt_cause(nss_ctx->nmap, int_ctx->irq, int_ctx->shift_factor, int_cause);
	int_ctx->cause |= int_cause;

	do {
		while ((int_ctx->cause) && (budget)) {

			/*
			 * Obtain the cause as per priority. Also obtain the weight
			 *
			 * NOTE: The idea is that all causes are processed as per priority and weight
			 * so that no single cause can overwhelm the system.
			 */
			prio_cause = nss_core_get_prioritized_cause(int_ctx->cause, &cause_type, &weight);
			if (budget < weight) {
				weight = budget;
			}

		processed = 0;
		switch (cause_type) {
		case NSS_INTR_CAUSE_QUEUE:
			processed = nss_core_handle_cause_queue(int_ctx, prio_cause, weight);

			count += processed;
			budget -= processed;
			if (processed < weight) {
				/*
				 * If #packets processed were lesser than weight then
				 * processing for this queue/cause is complete and
				 * we can clear this interrupt cause from interrupt context
				 * structure
				 */
				int_ctx->cause &= ~prio_cause;
			}
			break;

		case NSS_INTR_CAUSE_NON_QUEUE:
			nss_core_handle_cause_nonqueue(int_ctx, prio_cause, weight);
			int_ctx->cause &= ~prio_cause;
			break;

		default:
			nss_warning("%p: Invalid cause %x received from nss", nss_ctx, int_cause);
			nss_assert(0);
			break;
		}
	}

		nss_hal_read_interrupt_cause(nss_ctx->nmap, int_ctx->irq, int_ctx->shift_factor, &int_cause);
		nss_hal_clear_interrupt_cause(nss_ctx->nmap, int_ctx->irq, int_ctx->shift_factor, int_cause);
		int_ctx->cause |= int_cause;
	} while ((int_ctx->cause) && (budget));

	if (int_ctx->cause == 0) {
		napi_complete(napi);

		/*
		 * Re-enable any further interrupt from this IRQ
		 */
		nss_hal_enable_interrupt(nss_ctx->nmap, int_ctx->irq, int_ctx->shift_factor, NSS_HAL_SUPPORTED_INTERRUPTS);
	}

	return count;
}

/*
 * nss_core_send_crypto()
 *	Send crypto buffer to NSS
 */
int32_t nss_core_send_crypto(struct nss_ctx_instance *nss_ctx, void *buf, uint32_t buf_paddr, uint16_t len)
{
	int16_t count, hlos_index, nss_index, size;
	struct h2n_descriptor *desc;
	struct hlos_h2n_desc_rings *h2n_desc_ring = &nss_ctx->h2n_desc_rings[NSS_IF_DATA_QUEUE_0];
	struct h2n_desc_if_instance *desc_if = &h2n_desc_ring->desc_ring;
	struct nss_if_mem_map *if_map = (struct nss_if_mem_map *) nss_ctx->vmap;

	/*
	 * Take a lock for queue
	 */
	spin_lock_bh(&h2n_desc_ring->lock);

	/*
	 * We need to work out if there's sufficent space in our transmit descriptor
	 * ring to place the crypto packet.
	 */
	nss_index = if_map->h2n_nss_index[NSS_IF_DATA_QUEUE_0];
	hlos_index = h2n_desc_ring->hlos_index;

	size = desc_if->size;
	count = ((nss_index - hlos_index - 1) + size) & (size - 1);

	if (unlikely(count < 1)) {
		/* TODO: What is the use case of TX_STOPPED_FLAGS */
		h2n_desc_ring->tx_q_full_cnt++;
		h2n_desc_ring->flags |= NSS_H2N_DESC_RING_FLAGS_TX_STOPPED;
		spin_unlock_bh(&h2n_desc_ring->lock);
		nss_warning("%p: Data/Command Queue full reached", nss_ctx);

#if (NSS_PKT_STATS_ENABLED == 1)
		if (nss_ctx->id == NSS_CORE_0) {
			NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_QUEUE_FULL_0]);
		} else if (nss_ctx->id == NSS_CORE_1) {
			NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_QUEUE_FULL_1]);
		} else {
			nss_warning("%p: Invalid nss core: %d\n", nss_ctx, nss_ctx->id);
		}
#endif

		/*
		 * Enable de-congestion interrupt from NSS
		 */
		nss_hal_enable_interrupt(nss_ctx->nmap, nss_ctx->int_ctx[0].irq,
				nss_ctx->int_ctx[0].shift_factor, NSS_REGS_N2H_INTR_STATUS_TX_UNBLOCKED);

		return NSS_CORE_STATUS_FAILURE_QUEUE;
	}

	desc = &(desc_if->desc[hlos_index]);
	desc->opaque = (uint32_t) buf;
	desc->buffer_type = H2N_BUFFER_CRYPTO_REQ;
	desc->buffer = buf_paddr;
	desc->buffer_len = len;
	desc->payload_len = len;
	desc->payload_offs = 0;
	desc->bit_flags = H2N_BIT_FLAG_FIRST_SEGMENT | H2N_BIT_FLAG_LAST_SEGMENT;

	/*
	 * Update our host index so the NSS sees we've written a new descriptor.
	 */
	hlos_index = (hlos_index + 1) & (size - 1);
	h2n_desc_ring->hlos_index = hlos_index;
	if_map->h2n_hlos_index[NSS_IF_DATA_QUEUE_0] = hlos_index;
	spin_unlock_bh(&h2n_desc_ring->lock);

	return NSS_CORE_STATUS_SUCCESS;
}

/*
 * nss_core_write_one_descriptor()
 *	Fills-up a descriptor with required fields.
 */
static inline void nss_core_write_one_descriptor(struct h2n_descriptor *desc,
	uint16_t buffer_type, uint32_t buffer, uint32_t if_num,
	uint32_t opaque, uint16_t payload_off, uint16_t payload_len, uint16_t buffer_len,
	uint32_t qos_tag, uint16_t mss, uint16_t bit_flags)
{
	desc->buffer_type = buffer_type;
	desc->buffer = buffer;
	desc->interface_num = (int8_t)if_num;
	desc->opaque = opaque;
	desc->payload_offs = payload_off;
	desc->payload_len = payload_len;
	desc->buffer_len = buffer_len;
	desc->qos_tag = qos_tag;
	desc->mss = mss;
	desc->bit_flags = bit_flags;
}

/*
 * nss_core_send_buffer_simple_skb()
 *	Sends one skb to NSS FW
 */
static inline int32_t nss_core_send_buffer_simple_skb(struct nss_ctx_instance *nss_ctx,
	struct h2n_desc_if_instance *desc_if, uint32_t if_num, uint32_t frag0phyaddr,
	struct sk_buff *nbuf, uint16_t hlos_index, uint16_t flags, uint8_t buffer_type, uint16_t mss)
{
	struct h2n_descriptor *desc_ring = desc_if->desc;
	struct h2n_descriptor *desc;
	uint16_t bit_flags;
	uint16_t mask;

	bit_flags = flags | H2N_BIT_FLAG_FIRST_SEGMENT | H2N_BIT_FLAG_LAST_SEGMENT;

	if (likely(nbuf->ip_summed == CHECKSUM_PARTIAL)) {
		bit_flags |= H2N_BIT_FLAG_GEN_IP_TRANSPORT_CHECKSUM;
	} else if (nbuf->ip_summed == CHECKSUM_UNNECESSARY) {
		bit_flags |= H2N_BIT_FLAG_GEN_IP_TRANSPORT_CHECKSUM_NONE;
	}

	mask = desc_if->size - 1;
	desc = &desc_ring[hlos_index];

	nss_core_write_one_descriptor(desc, buffer_type, frag0phyaddr, if_num,
		(uint32_t)nbuf, (uint16_t)(nbuf->data - nbuf->head), nbuf->len,
		(uint16_t)(nbuf->end - nbuf->head), (uint32_t)nbuf->priority, mss, bit_flags);

	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_SIMPLE]);
	return 1;
}

//Note to Thomas:  Linux has support for atomic_inc() on a 64 bit value.  Hint, use iot

/*
 * nss_core_send_buffer_nr_frags()
 *	Sends frags array (NETIF_F_SG) to NSS FW
 */
static inline int32_t nss_core_send_buffer_nr_frags(struct nss_ctx_instance *nss_ctx,
	struct h2n_desc_if_instance *desc_if, uint32_t if_num, uint32_t frag0phyaddr,
	struct sk_buff *nbuf, uint16_t hlos_index, uint16_t flags, uint8_t buffer_type, uint16_t mss)
{
	struct h2n_descriptor *desc_ring = desc_if->desc;
	struct h2n_descriptor *desc;
	const skb_frag_t *frag;
	dma_addr_t buffer;
	uint32_t nr_frags;
	uint16_t bit_flags;
	int16_t i;
	uint16_t mask;

	/*
	 * Set the appropriate flags.
	 */
	bit_flags = (flags | H2N_BIT_FLAG_DISCARD);
	if (likely(nbuf->ip_summed == CHECKSUM_PARTIAL)) {
		bit_flags |= H2N_BIT_FLAG_GEN_IP_TRANSPORT_CHECKSUM;
	}

	mask = desc_if->size - 1;
	desc = &desc_ring[hlos_index];

	/*
	 * First fragment/descriptor is special
	 */
	nss_core_write_one_descriptor(desc, buffer_type, frag0phyaddr, if_num,
		(uint32_t)NULL, nbuf->data - nbuf->head, nbuf->len - nbuf->data_len,
		nbuf->end - nbuf->head, (uint32_t)nbuf->priority, mss, bit_flags | H2N_BIT_FLAG_FIRST_SEGMENT);

	/*
	 * Now handle rest of the fragments.
	 */
	nr_frags = skb_shinfo(nbuf)->nr_frags;
	BUG_ON(nr_frags > MAX_SKB_FRAGS);
	for (i = 0; i < nr_frags; i++) {
		frag = &skb_shinfo(nbuf)->frags[i];

		buffer = skb_frag_dma_map(NULL, frag, 0, skb_frag_size(frag), DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(NULL, buffer))) {
			nss_warning("%p: DMA mapping failed for fragment", nss_ctx);
			return -(i + 1);
		}

		hlos_index = (hlos_index + 1) & (mask);
		desc = &(desc_if->desc[hlos_index]);

		nss_core_write_one_descriptor(desc, buffer_type, buffer, if_num,
			(uint32_t)NULL, 0, skb_frag_size(frag), skb_frag_size(frag),
			nbuf->priority, mss, bit_flags);
	}

	/*
	 * Update bit flag for last descriptor.
	 * The discard flag shall be set for all fragments except the
	 * the last one.The NSS returns the last fragment to HLOS
	 * after the packet processing is done.We do need to send the
	 * packet buffer address (skb) in the descriptor of last segment
	 * when the decriptor returns from NSS the HLOS uses the
	 * opaque field to free the memory allocated.
	 */
	desc->bit_flags |= H2N_BIT_FLAG_LAST_SEGMENT;
	desc->bit_flags &= ~(H2N_BIT_FLAG_DISCARD);
	desc->opaque = (uint32_t)nbuf;
	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_NR_FRAGS]);
	return i+1;
}

/*
 * nss_core_send_buffer_fraglist()
 *	Sends fraglist (NETIF_F_FRAGLIST) to NSS FW
 */
static inline int32_t nss_core_send_buffer_fraglist(struct nss_ctx_instance *nss_ctx,
	struct h2n_desc_if_instance *desc_if, uint32_t if_num, uint32_t frag0phyaddr,
	struct sk_buff *nbuf, uint16_t hlos_index, uint16_t flags, uint8_t buffer_type, uint16_t mss)
{
	struct h2n_descriptor *desc_ring = desc_if->desc;
	struct h2n_descriptor *desc;
	dma_addr_t buffer;
	uint16_t mask;
	struct sk_buff *iter;
	uint16_t bit_flags;
	int16_t i;

	/*
	 * Set the appropriate flags.
	 */
	bit_flags = (flags | H2N_BIT_FLAG_DISCARD);
	if (likely(nbuf->ip_summed == CHECKSUM_PARTIAL)) {
		bit_flags |= H2N_BIT_FLAG_GEN_IP_TRANSPORT_CHECKSUM;
	}

	mask = desc_if->size - 1;
	desc = &desc_ring[hlos_index];

	/*
	 * First fragment/descriptor is special
	 */
	nss_core_write_one_descriptor(desc, buffer_type, frag0phyaddr, if_num,
		(uint32_t)NULL, nbuf->data - nbuf->head, nbuf->len - nbuf->data_len,
		nbuf->end - nbuf->head, (uint32_t)nbuf->priority, mss, bit_flags | H2N_BIT_FLAG_FIRST_SEGMENT);

	/*
	 * Walk the frag_list in nbuf
	 */
	i = 0;
	skb_walk_frags(nbuf, iter) {
		uint32_t nr_frags;

		buffer = (uint32_t)dma_map_single(NULL, iter->head, (iter->tail - iter->head), DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(NULL, buffer))) {
			nss_warning("%p: DMA mapping failed for virtual address = %x", nss_ctx, (uint32_t)iter->head);
			return -(i+1);
		}

		/*
		 * We currently don't support frags[] array inside a
		 * fraglist.
		 */
		nr_frags = skb_shinfo(iter)->nr_frags;
		if (unlikely(nr_frags > 0)) {
			nss_warning("%p: fraglist with page data are not supported: %p\n", nss_ctx, iter);
			return -(i+1);
		}

		/*
		 * Update index.
		 */
		hlos_index = (hlos_index + 1) & (mask);
		desc = &(desc_if->desc[hlos_index]);

		nss_core_write_one_descriptor(desc, buffer_type, buffer, if_num,
			(uint32_t)NULL, iter->data - iter->head, iter->len - iter->data_len,
			iter->end - iter->head, iter->priority, mss, bit_flags);

		i++;
	}

	/*
	 * The firmware send's empty buffers as singleton buffers.
	 * Make sure we treat frag_list buffers as singelton.
	 */
	skb_frag_list_init(nbuf);

	/*
	 * Update bit flag for last descriptor. The discard flag shall
	 * be set for all fragments except the the last one.
	 */
	desc->bit_flags |= H2N_BIT_FLAG_LAST_SEGMENT;
	desc->bit_flags &= ~(H2N_BIT_FLAG_DISCARD);
	desc->opaque = (uint32_t)nbuf;
	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_FRAGLIST]);
	return i+1;
}

/*
 * nss_core_send_unwind_dma()
 *	It unwinds (or unmap) DMA from descriptors
 */
static inline void nss_core_send_unwind_dma(struct h2n_desc_if_instance *desc_if,
	uint16_t hlos_index, int16_t count)
{
	struct h2n_descriptor *desc_ring = desc_if->desc;
	struct h2n_descriptor *desc;
	int16_t i, mask;

	mask = desc_if->size - 1;
	for (i = 0; i < count; i++) {
		desc = &desc_ring[hlos_index];
		dma_unmap_single(NULL, desc->buffer, desc->buffer_len, DMA_TO_DEVICE);
		hlos_index = (hlos_index + 1) & mask;
	}
}

/*
 * nss_core_send_buffer()
 *	Send network buffer to NSS
 */
int32_t nss_core_send_buffer(struct nss_ctx_instance *nss_ctx, uint32_t if_num,
					struct sk_buff *nbuf, uint16_t qid,
					uint8_t buffer_type, uint16_t flags)
{
	int16_t count, hlos_index, nss_index, size, mask;
	uint32_t segments;
	struct hlos_h2n_desc_rings *h2n_desc_ring = &nss_ctx->h2n_desc_rings[qid];
	struct h2n_desc_if_instance *desc_if = &h2n_desc_ring->desc_ring;
	struct h2n_descriptor *desc_ring;
	struct h2n_descriptor *desc;
	struct nss_if_mem_map *if_map = (struct nss_if_mem_map *)nss_ctx->vmap;
	uint16_t mss = 0;
	uint32_t frag0phyaddr = 0;
	bool is_bounce = ((buffer_type == H2N_BUFFER_SHAPER_BOUNCE_INTERFACE) || (buffer_type == H2N_BUFFER_SHAPER_BOUNCE_BRIDGE));


	desc_ring = desc_if->desc;
	size = desc_if->size;
	mask = size - 1;

	frag0phyaddr = (uint32_t)dma_map_single(NULL, nbuf->head, (nbuf->tail - nbuf->head), DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(NULL, frag0phyaddr))) {
		nss_warning("%p: DMA mapping failed for virtual address = %x", nss_ctx,(uint32_t)nbuf->head);
		return NSS_CORE_STATUS_FAILURE;
	}

	/*
	 * If nbuf does not have fraglist, then update nr_frags
	 * from frags[] array. Otherwise walk the frag_list.
	 */
	if (!skb_has_frag_list(nbuf)) {
		segments = skb_shinfo(nbuf)->nr_frags;
		BUG_ON(segments > MAX_SKB_FRAGS);
	} else {
		struct sk_buff *iter;
		segments = 0;
		skb_walk_frags(nbuf, iter) {
			segments ++;
		}
	}

	/*
	 * Take a lock for queue
	 */
	spin_lock_bh(&h2n_desc_ring->lock);

	/*
	 * We need to work out if there's sufficent space in our transmit descriptor
	 * ring to place all the segments of a nbuf.
	 */
	nss_index = if_map->h2n_nss_index[qid];
	hlos_index = h2n_desc_ring->hlos_index;

	size = desc_if->size;
	mask = size - 1;
	count = ((nss_index - hlos_index - 1) + size) & (mask);

	if (unlikely(count < (segments + 1))) {
		/*
		 * NOTE: tx_q_full_cnt and TX_STOPPED flags will be used
		 *	when we will add support for DESC Q congestion management
		 *	in future
		 */
		h2n_desc_ring->tx_q_full_cnt++;
		h2n_desc_ring->flags |= NSS_H2N_DESC_RING_FLAGS_TX_STOPPED;
		spin_unlock_bh(&h2n_desc_ring->lock);
		nss_warning("%p: Data/Command Queue full reached", nss_ctx);

#if (NSS_PKT_STATS_ENABLED == 1)
		if (nss_ctx->id == NSS_CORE_0) {
			NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_QUEUE_FULL_0]);
		} else if (nss_ctx->id == NSS_CORE_1) {
			NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_TX_QUEUE_FULL_1]);
		} else {
			nss_warning("%p: Invalid nss core: %d\n", nss_ctx, nss_ctx->id);
		}
#endif

		/*
		 * Enable de-congestion interrupt from NSS
		 */
		nss_hal_enable_interrupt(nss_ctx->nmap, nss_ctx->int_ctx[0].irq,
				nss_ctx->int_ctx[0].shift_factor, NSS_REGS_N2H_INTR_STATUS_TX_UNBLOCKED);

		return NSS_CORE_STATUS_FAILURE_QUEUE;
	}

	desc = &desc_ring[hlos_index];

	/*
	 * Check if segmentation enabled.
	 * Configure descriptor bit flags accordingly
	 */

	/*
	 * When CONFIG_HIGHMEM is enabled OS is giving a single big chunk buffer without
	 * any scattered frames.
	 *
	 * NOTE: We dont have to perform segmentation offload for packets that are being
	 * bounced. These packets WILL return to the HLOS for freeing or further processing.
	 * They will NOT be transmitted by the NSS.
	 */
	if (skb_is_gso(nbuf) && !is_bounce) {

		mss = skb_shinfo(nbuf)->gso_size;
		flags |= H2N_BIT_FLAG_SEGMENTATION_ENABLE;
		if (skb_shinfo(nbuf)->gso_type & SKB_GSO_TCPV4) {
			flags |= H2N_BIT_FLAG_SEGMENT_TSO;
		} else if (skb_shinfo(nbuf)->gso_type & SKB_GSO_TCPV6) {
			flags |= H2N_BIT_FLAG_SEGMENT_TSO6;
		} else if (skb_shinfo(nbuf)->gso_type & SKB_GSO_UDP) {
			flags |= H2N_BIT_FLAG_SEGMENT_UFO;
		} else {
			/*
			 * Invalid segmentation type
			 */
			nss_assert(0);
		}
	}

	/*
	 * WARNING! : The following "is_bounce" check has a potential to cause corruption
	 * if things change in the NSS. This check allows fragmented packets to be sent down
	 * with incomplete payload information since NSS does not care about the payload content
	 * when packets are bounced for shaping. If it starts caring in future, then this code
	 * will have to change.
	 *
	 * WHY WE ARE DOING THIS - Skipping S/G processing helps with performance.
	 *
	 */
	count = 0;
	if (likely((segments == 0) || is_bounce)) {
		count = nss_core_send_buffer_simple_skb(nss_ctx, desc_if, if_num, frag0phyaddr,
			nbuf, hlos_index, flags, buffer_type, mss);
	} else if (skb_has_frag_list(nbuf)) {
		count = nss_core_send_buffer_fraglist(nss_ctx, desc_if, if_num, frag0phyaddr,
			nbuf, hlos_index, flags, buffer_type, mss);
	} else {
		count = nss_core_send_buffer_nr_frags(nss_ctx, desc_if, if_num, frag0phyaddr,
			nbuf, hlos_index, flags, buffer_type, mss);
	}

	if (unlikely(count < 0)) {
		/*
		 * We failed and hence we need to unmap dma regions
		 */
		nss_warning("%p: failed to map DMA regions:%d", nss_ctx, -count);
		nss_core_send_unwind_dma(desc_if, hlos_index, -count);
		spin_unlock_bh(&nss_ctx->h2n_desc_rings[qid].lock);
		return NSS_CORE_STATUS_FAILURE;
	}

	/*
	 * Update our host index so the NSS sees we've written a new descriptor.
	 */
	hlos_index = (hlos_index + count) & mask;
	h2n_desc_ring->hlos_index = hlos_index;
	if_map->h2n_hlos_index[qid] = hlos_index;

	/*
	 * We are holding this skb in NSS FW, let kmemleak know about it
	 */
	kmemleak_not_leak(nbuf);
	NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_ctx->nss_top->stats_drv[NSS_STATS_DRV_NSS_SKB_COUNT]);

	spin_unlock_bh(&h2n_desc_ring->lock);
	return NSS_CORE_STATUS_SUCCESS;
}
