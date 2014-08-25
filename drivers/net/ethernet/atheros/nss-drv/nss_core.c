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
 * nss_core.c
 *	NSS driver core APIs source file.
 */

#include "nss_core.h"
#include <linux/module.h>
#include <nss_hal.h>
#include <net/dst.h>
#include <linux/etherdevice.h>
#include "nss_tx_rx_common.h"

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

	ncm = (struct nss_cmn_msg *)nbuf->data;

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

		spin_lock_bh(&nss_top->stats_lock);
		nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]++;
		spin_unlock_bh(&nss_top->stats_lock);
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
		return NSS_IF_DATA_QUEUE;
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
	printk("\tpayload_offs = %d\n", desc->payload_offs);
	printk("\tpayload_len = %d\n", desc->payload_len);
}

/*
 * nss_core_handle_cause_queue()
 *	Handle interrupt cause related to N2H/H2N queues
 */
static int32_t nss_core_handle_cause_queue(struct int_ctx_instance *int_ctx, uint16_t cause, int16_t weight)
{
	void *ctx;
	nss_phys_if_rx_callback_t cb;
	int16_t count, count_temp;
	uint16_t size, mask, qid;
	uint32_t nss_index, hlos_index;
	struct sk_buff *nbuf;
	struct net_device *ndev;
	struct hlos_n2h_desc_ring *n2h_desc_ring;
	struct n2h_desc_if_instance *desc_if;
	struct n2h_descriptor *desc_ring;
	struct n2h_descriptor *desc;
	uint32_t nr_frags;
	struct nss_ctx_instance *nss_ctx = int_ctx->nss_ctx;
	struct nss_top_instance *nss_top = nss_ctx->nss_top;
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

	count_temp = count;
	while (count_temp) {
		unsigned int buffer_type;
		uint32_t opaque;
		desc = &desc_ring[hlos_index];
		buffer_type = desc->buffer_type;
		opaque = desc->opaque;

		if (unlikely((buffer_type == N2H_BUFFER_CRYPTO_RESP))) {
			NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_top->stats_drv[NSS_STATS_DRV_RX_CRYPTO_RESP]);


			/*
			 * This is a crypto buffer hence send it to crypto driver
			 *
			 * NOTE: Crypto buffers require special handling as they do not
			 *	use OS network buffers (e.g. skb). Hence, OS buffer operations
			 *	are not applicable to crypto buffers
			 */
			nss_crypto_buf_handler(nss_ctx, (void *)opaque, desc->buffer, desc->payload_len);
		} else {
			unsigned int interface_num = desc->interface_num;

			/*
			* Obtain nbuf
			*/
			nbuf = (struct sk_buff *)opaque;
			if (unlikely(nbuf < (struct sk_buff *)PAGE_OFFSET)) {
				/*
				 * Invalid opaque pointer
				 */
				nss_dump_desc(nss_ctx, desc);
			}

			/*
			 * Get the number of fragments
			 */
			nr_frags = skb_shinfo(nbuf)->nr_frags;
			if (likely(nr_frags == 0)) {
				unsigned int payload_offs = desc->payload_offs;
				unsigned int payload_len = desc->payload_len;
				dma_addr_t buffer = desc->buffer;

				/*
				 * Set relevant fields within nbuf (len, head, tail)
				 */
				nbuf->data = nbuf->head + payload_offs;
				nbuf->len = payload_len;
				nbuf->tail = nbuf->data + nbuf->len;

				/*
				 * TODO: Check if there is any issue wrt map and unmap,
				 * NSS should playaround with data area and should not
				 * touch HEADROOM area
				 */
				dma_unmap_single(NULL, (buffer + payload_offs), payload_len, DMA_FROM_DEVICE);
				prefetch((void *)(nbuf->data));
			}

			/*
			 * The Assumption here is that the scattered SKB will be
			 * given back by NSS POP buffer only to free them.
			 * Hence it is assumed that there is no need to unmap the
			 * scattered segments and the first segment of SKB
			 */

			switch (buffer_type) {
			case N2H_BUFFER_SHAPER_BOUNCED_INTERFACE:
				{
					/*
					 * Bounced packet is returned from an interface bounce operation
					 * Obtain the registrant to which to return the skb
					 */
					nss_shaper_bounced_callback_t bounced_callback;
					void *app_data;
					struct module *owner;
					struct nss_shaper_bounce_registrant *reg = &nss_top->bounce_interface_registrants[interface_num];

					spin_lock_bh(&nss_top->lock);

					/*
					 * Do we have a registrant?
					 */
					if (!reg->registered) {
						spin_unlock_bh(&nss_top->lock);
						break;
					}

					/*
					 * Get handle to the owning registrant
					 */
					bounced_callback = reg->bounced_callback;
					app_data = reg->app_data;
					owner = reg->owner;
					if (!try_module_get(owner)) {
						spin_unlock_bh(&nss_top->lock);
						break;
					}

					/*
					 * Callback is active, unregistration is not permitted while this is in progress
					 */
					reg->callback_active = true;
					spin_unlock_bh(&nss_top->lock);

					/*
					 * Pass bounced packet back to registrant
					 */
					bounced_callback(app_data, nbuf);
					spin_lock_bh(&nss_top->lock);
					reg->callback_active = false;
					spin_unlock_bh(&nss_top->lock);
					module_put(owner);
				}
				break;
			case N2H_BUFFER_SHAPER_BOUNCED_BRIDGE:
				/*
				 * Bounced packet is returned from a bridge bounce operation
				 */
				{
					/*
					 * Bounced packet is returned from a bridge bounce operation
					 * Obtain the registrant to which to return the skb
					 */
					nss_shaper_bounced_callback_t bounced_callback;
					void *app_data;
					struct module *owner;
					struct nss_shaper_bounce_registrant *reg = &nss_top->bounce_bridge_registrants[interface_num];

					spin_lock_bh(&nss_top->lock);

					/*
					 * Do we have a registrant?
					 */
					if (!reg->registered) {
						spin_unlock_bh(&nss_top->lock);
						break;
					}

					/*
					 * Get handle to the owning registrant
					 */
					bounced_callback = reg->bounced_callback;
					app_data = reg->app_data;
					owner = reg->owner;
					if (!try_module_get(owner)) {
						spin_unlock_bh(&nss_top->lock);
						break;
					}

					/*
					 * Callback is active, unregistration is not permitted while this is in progress
					 */
					reg->callback_active = true;
					spin_unlock_bh(&nss_top->lock);

					/*
					 * Pass bounced packet back to registrant
					 */
					bounced_callback(app_data, nbuf);
					spin_lock_bh(&nss_top->lock);
					reg->callback_active = false;
					spin_unlock_bh(&nss_top->lock);
					module_put(owner);
				}
				break;
			case N2H_BUFFER_PACKET_VIRTUAL:
				NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_top->stats_drv[NSS_STATS_DRV_RX_VIRTUAL]);

				/*
				 * Checksum is already done by NSS for packets forwarded to virtual interfaces
				 */
				nbuf->ip_summed = CHECKSUM_NONE;

				/*
				 * Obtain net_device pointer
				 */
				ndev = (struct net_device *)nss_top->if_ctx[interface_num];
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
					break;
				}

				dev_hold(ndev);		// GGG FIXME THIS IS BROKEN AS NDEV COULD BE DESTROYED BEFORE THE HOLD IS TAKEN!  NDEV SHOULD BE HELD WHEN THE VIRTUAL IS REGISTERED
							// AND THE HOLD HERE TAKEN INSIDE OF SOME KIND OF MUTEX LOCK WITH VIRTUAL UNREGISTRATION
				nbuf->dev = ndev;

				/*
				 * Send the packet to virtual interface
				 * NOTE: Invoking this will BYPASS any assigned QDisc - this is OKAY
				 * as TX packets out of the NSS will have been shaped inside the NSS.
				 */
				ndev->netdev_ops->ndo_start_xmit(nbuf, ndev);
				dev_put(ndev);
				break;

			case N2H_BUFFER_PACKET:
				NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_top->stats_drv[NSS_STATS_DRV_RX_PACKET]);

				/*
				 * Check if NSS was able to obtain checksum
				 */
				nbuf->ip_summed = CHECKSUM_UNNECESSARY;
				if (unlikely(!(desc->bit_flags & N2H_BIT_FLAG_IP_TRANSPORT_CHECKSUM_VALID))) {
					nbuf->ip_summed = CHECKSUM_NONE;
				}

				ctx = nss_top->if_ctx[interface_num];
				cb = nss_top->if_rx_callback[interface_num];
				if (likely(cb) && likely(ctx)) {
					/*
					 * Packet was received on Physical interface
					 */
					cb(ctx, (void *)nbuf);
				} else if (NSS_IS_IF_TYPE(VIRTUAL, interface_num)) {
					/*
					 * Packet was received on Virtual interface
					 */

					/*
					 * Give the packet to stack
					 *
					 * TODO: Change to gro receive later
					 */
					ctx = nss_top->if_ctx[interface_num];
					if (ctx) {
						dev_hold(ctx);
						nbuf->dev = (struct net_device*) ctx;
						nbuf->protocol = eth_type_trans(nbuf, ctx);
						netif_receive_skb(nbuf);
						dev_put(ctx);
					} else {
						/*
						 * Interface has gone down
						 */
						nss_warning("%p: Received exception packet from bad virtual interface %d",
								nss_ctx, interface_num);
						dev_kfree_skb_any(nbuf);
					}
				} else {
					dev_kfree_skb_any(nbuf);
				}
				break;

			case N2H_BUFFER_STATUS:
				NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_top->stats_drv[NSS_STATS_DRV_RX_STATUS]);
				nss_core_handle_nss_status_pkt(nss_ctx, nbuf);
				dev_kfree_skb_any(nbuf);
				break;

			case N2H_BUFFER_EMPTY:
				NSS_PKT_STATS_INCREMENT(nss_ctx, &nss_top->stats_drv[NSS_STATS_DRV_RX_EMPTY]);

				/*
				 * TODO: Unmap fragments
				 */
				dev_kfree_skb_any(nbuf);
				break;

			default:
				/*
				 * ERROR:
				 */
				nss_warning("%p: Invalid buffer type %d received from NSS", nss_ctx, buffer_type);
			}
		}

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
		uint16_t count, size, mask;
		int32_t nss_index, hlos_index;
		struct hlos_h2n_desc_rings *h2n_desc_ring = &nss_ctx->h2n_desc_rings[NSS_IF_EMPTY_BUFFER_QUEUE];
		struct h2n_desc_if_instance *desc_if = &h2n_desc_ring->desc_ring;
		struct h2n_descriptor *desc_ring;
		struct nss_top_instance *nss_top = nss_ctx->nss_top;


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

			nbuf = dev_alloc_skb(max_buf_size);
			if (unlikely(!nbuf)) {
				/*
				 * ERR:
				 */
				spin_lock_bh(&nss_top->stats_lock);
				nss_top->stats_drv[NSS_STATS_DRV_NBUF_ALLOC_FAILS]++;
				spin_unlock_bh(&nss_top->stats_lock);
				nss_warning("%p: Could not obtain empty buffer", nss_ctx);
				break;
			}

			buffer = dma_map_single(NULL, nbuf->head, max_buf_size, DMA_FROM_DEVICE);
			if (unlikely(dma_mapping_error(NULL, buffer))) {
				/*
				 * ERR:
				 */
				dev_kfree_skb_any(nbuf);
				nss_warning("%p: DMA mapping failed for empty buffer", nss_ctx);
				break;
			}

			desc->opaque = (uint32_t)nbuf;
			desc->payload_offs = (uint16_t) (nbuf->data - nbuf->head);
			desc->buffer = buffer;
			desc->buffer_len = max_buf_size;
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
		nss_ctx->h2n_desc_rings[NSS_IF_DATA_QUEUE].flags &= ~NSS_H2N_DESC_RING_FLAGS_TX_STOPPED;

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
	struct hlos_h2n_desc_rings *h2n_desc_ring = &nss_ctx->h2n_desc_rings[NSS_IF_DATA_QUEUE];
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
	nss_index = if_map->h2n_nss_index[NSS_IF_DATA_QUEUE];
	hlos_index = h2n_desc_ring->hlos_index;

	size = desc_if->size;
	count = ((nss_index - hlos_index - 1) + size) & (size - 1);

	if (unlikely(count < 1)) {
		/* TODO: What is the use case of TX_STOPPED_FLAGS */
		h2n_desc_ring->tx_q_full_cnt++;
		h2n_desc_ring->flags |= NSS_H2N_DESC_RING_FLAGS_TX_STOPPED;
		spin_unlock_bh(&h2n_desc_ring->lock);
		nss_warning("%p: Data/Command Queue full reached", nss_ctx);

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
	desc->bit_flags = 0;

	/*
	 * Update our host index so the NSS sees we've written a new descriptor.
	 */
	hlos_index = (hlos_index + 1) & (size - 1);
	h2n_desc_ring->hlos_index = hlos_index;
	if_map->h2n_hlos_index[NSS_IF_DATA_QUEUE] = hlos_index;
	spin_unlock_bh(&h2n_desc_ring->lock);

	return NSS_CORE_STATUS_SUCCESS;
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
	uint32_t nr_frags;
	struct hlos_h2n_desc_rings *h2n_desc_ring = &nss_ctx->h2n_desc_rings[qid];
	struct h2n_desc_if_instance *desc_if = &h2n_desc_ring->desc_ring;
	struct h2n_descriptor *desc_ring;
	struct h2n_descriptor *desc;
	struct nss_if_mem_map *if_map = (struct nss_if_mem_map *)nss_ctx->vmap;
	uint16_t mss = 0;
	uint32_t frag0phyaddr = 0;

	nr_frags = skb_shinfo(nbuf)->nr_frags;

	BUG_ON(nr_frags > MAX_SKB_FRAGS);

	desc_ring = desc_if->desc;
	size = desc_if->size;
	mask = size - 1;

	frag0phyaddr = (uint32_t)dma_map_single(NULL, nbuf->head, (nbuf->tail - nbuf->head), DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(NULL, frag0phyaddr))) {
		nss_warning("%p: DMA mapping failed for virtual address = %x", nss_ctx,(uint32_t)nbuf->head);
		return NSS_CORE_STATUS_FAILURE;
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

	if (unlikely(count < (nr_frags + 1))) {
		/*
		 * NOTE: tx_q_full_cnt and TX_STOPPED flags will be used
		 *	when we will add support for DESC Q congestion management
		 *	in future
		 */
		h2n_desc_ring->tx_q_full_cnt++;
		h2n_desc_ring->flags |= NSS_H2N_DESC_RING_FLAGS_TX_STOPPED;
		spin_unlock_bh(&h2n_desc_ring->lock);
		nss_warning("%p: Data/Command Queue full reached", nss_ctx);

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
	 * any scattered frames
	 */
	if (skb_is_gso(nbuf)) {
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

	if (likely(nr_frags == 0)) {
		uint16_t bit_flags;

		bit_flags = flags | H2N_BIT_FLAG_FIRST_SEGMENT | H2N_BIT_FLAG_LAST_SEGMENT;

		if (likely(nbuf->ip_summed == CHECKSUM_PARTIAL)) {
			bit_flags |= H2N_BIT_FLAG_GEN_IP_TRANSPORT_CHECKSUM;
		}

		desc->buffer_type = buffer_type;
		desc->buffer = frag0phyaddr;
		desc->interface_num = (int8_t)if_num;
		desc->opaque = (uint32_t)nbuf;
		desc->payload_offs = (uint16_t)(nbuf->data - nbuf->head);
		desc->payload_len = nbuf->len;
		desc->buffer_len = (uint16_t)(nbuf->end - nbuf->head);
		desc->mss = mss;
		desc->qos_tag = (uint32_t)nbuf->priority;

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3,6,0))
		if (unlikely(!NSS_IS_IF_TYPE(VIRTUAL, if_num))) {
			if (likely(nbuf->destructor == NULL)) {
				if (likely(skb_recycle_check(nbuf, nss_ctx->max_buf_size))) {
					bit_flags |= H2N_BIT_BUFFER_REUSE;
					desc->buffer_len = nss_ctx->max_buf_size + NET_SKB_PAD;
				}
			}
		}
#endif

		desc->bit_flags = bit_flags;
	} else {
		/*
		 * TODO: convert to BUGON/ASSERT
		 */
		uint32_t i = 0;
		const skb_frag_t *frag;
		dma_addr_t buffer;
		uint16_t bit_flags;

		/*
		 * Handle all fragments
		 */

		/*
		 * First fragment/descriptor is special
		 */
		bit_flags = (flags | H2N_BIT_FLAG_FIRST_SEGMENT | H2N_BIT_FLAG_DISCARD);
		if (likely(nbuf->ip_summed == CHECKSUM_PARTIAL)) {
			bit_flags |= H2N_BIT_FLAG_GEN_IP_TRANSPORT_CHECKSUM;
		}

		desc->buffer_type = buffer_type;
		desc->interface_num = (int8_t)if_num;
		desc->opaque = (uint32_t)NULL;
		desc->payload_offs = nbuf->data - nbuf->head;
		desc->payload_len = nbuf->len - nbuf->data_len;
		desc->buffer_len = nbuf->end - nbuf->head;
		desc->buffer = frag0phyaddr;
		desc->mss = mss;
		desc->bit_flags = bit_flags;
		desc->qos_tag = (uint32_t)nbuf->priority;

		/*
		 * Now handle rest of the fragments.
		 */
		while (likely(i < (nr_frags))) {
			frag = &skb_shinfo(nbuf)->frags[i++];
			hlos_index = (hlos_index + 1) & (mask);
			desc = &(desc_if->desc[hlos_index]);
			bit_flags = (flags | H2N_BIT_FLAG_DISCARD);
			if (likely(nbuf->ip_summed == CHECKSUM_PARTIAL)) {
				bit_flags |= H2N_BIT_FLAG_GEN_IP_TRANSPORT_CHECKSUM;
			}

			buffer = skb_frag_dma_map(NULL, frag, 0, skb_frag_size(frag), DMA_TO_DEVICE);
			if (unlikely(dma_mapping_error(NULL, buffer))) {
				spin_unlock_bh(&nss_ctx->h2n_desc_rings[qid].lock);
				nss_warning("%p: DMA mapping failed for fragment", nss_ctx);
				return NSS_CORE_STATUS_FAILURE;
			}

			desc->buffer_type = buffer_type;
			desc->buffer = buffer;
			desc->interface_num = (int8_t)if_num;
			desc->opaque = (uint32_t)NULL;
			desc->payload_offs = 0;
			desc->payload_len = skb_frag_size(frag);
			desc->buffer_len = skb_frag_size(frag);
			desc->qos_tag = (uint32_t)nbuf->priority;
			desc->mss = mss;
			desc->bit_flags = bit_flags;
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
	}

	/*
	 * Update our host index so the NSS sees we've written a new descriptor.
	 */
	hlos_index = (hlos_index + 1) & mask;
	h2n_desc_ring->hlos_index = hlos_index;
	if_map->h2n_hlos_index[qid] = hlos_index;

	spin_unlock_bh(&h2n_desc_ring->lock);

	return NSS_CORE_STATUS_SUCCESS;
}
