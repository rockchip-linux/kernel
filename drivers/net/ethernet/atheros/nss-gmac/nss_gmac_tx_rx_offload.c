/*
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER
 * RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE
 * USE OR PERFORMANCE OF THIS SOFTWARE.
 */
/*
 * @file
 * This is the network dependent layer to handle network related functionality.
 * This file is tightly coupled to neworking frame work of linux kernel.
 * The functionality carried out in this file should be treated as an
 * example only if the underlying operating system is not Linux.
 *
 * @note Many of the functions other than the device specific functions
 *  changes for operating system other than Linux 2.6.xx
 *-----------------------------REVISION HISTORY---------------------------------
 * Qualcomm Atheros		15/Feb/2013			Created
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/bitops.h>
#include <linux/phy.h>
#include <linux/interrupt.h>

#include <nss_gmac_dev.h>
#include <nss_gmac_network_interface.h>

#define NSS_GMAC_NAPI_BUDGET	64
#define dma_int_enable		(dma_ie_normal | dma_int_tx_norm_mask | dma_int_rx_norm_mask)

/**
 * This sets up the transmit Descriptor queue in ring or chain mode.
 * This function is tightly coupled to the platform and operating system
 * Device is interested only after the descriptors are setup. Therefore this
 * function is not included in the device driver API. This function should be
 * treated as an example code to design the descriptor structures for ring mode
 * or chain mode.
 * This function depends on the device structure for allocation consistent
 * dma-able memory in case of linux.
 *	- Allocates the memory for the descriptors.
 *	- Initialize the Busy and Next descriptors indices to 0(Indicating
 *	  first descriptor).
 *	- Initialize the Busy and Next descriptors to first descriptor address.
 *	- Initialize the last descriptor with the endof ring in case of ring
 *	  mode.
 *	- Initialize the descriptors in chain mode.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] pointer to device structure.
 * @param[in] number of descriptor expected in tx descriptor queue.
 * @param[in] whether descriptors to be created in RING mode or CHAIN mode.
 * @return 0 upon success. Error code upon failure.
 * @note This function fails if allocation fails for required number of
 * descriptors in Ring mode, but in chain mode function returns -ENOMEM in the
 * process of descriptor chain creation. once returned from this function user
 * should for gmacdev->tx_desc_count to see how many descriptors are there in
 * the chain.
 * Should continue further only if the number of descriptors in the
 * chain meets the requirements.
 */
static int32_t nss_gmac_setup_tx_desc_queue(struct nss_gmac_dev *gmacdev,
						struct device *dev,
						uint32_t no_of_desc,
						uint32_t desc_mode)
{
	int32_t i;
	struct dma_desc *first_desc = NULL;
	dma_addr_t dma_addr;

	gmacdev->tx_desc_count = 0;

	BUG_ON(desc_mode != RINGMODE);
	BUG_ON((no_of_desc & (no_of_desc - 1)) != 0);

	netdev_dbg(gmacdev->netdev, "Total size of memory required for Tx Descriptors in Ring Mode = 0x%08x"
			, (uint32_t) ((sizeof(struct dma_desc) * no_of_desc)));

	first_desc = dma_alloc_coherent(dev, sizeof(struct dma_desc) * no_of_desc
					, &dma_addr, GFP_KERNEL);
	if (first_desc == NULL) {
		netdev_dbg(gmacdev->netdev,
				"Error in Tx Descriptors memory allocation");
		return -ENOMEM;
	}

	gmacdev->tx_desc_count = no_of_desc;
	gmacdev->tx_desc = first_desc;
	gmacdev->tx_desc_dma = dma_addr;
	netdev_dbg(gmacdev->netdev, "Tx Descriptors in Ring Mode: No. of descriptors = %d base = 0x%08x dma = 0x%08x"
			, no_of_desc, (uint32_t)first_desc, dma_addr);

	for (i = 0; i < gmacdev->tx_desc_count; i++) {
		nss_gmac_tx_desc_init_ring(gmacdev->tx_desc + i,
					i == (gmacdev->tx_desc_count - 1));
	}

	gmacdev->tx_next = 0;
	gmacdev->tx_busy = 0;
	gmacdev->tx_next_desc = gmacdev->tx_desc;
	gmacdev->tx_busy_desc = gmacdev->tx_desc;
	gmacdev->busy_tx_desc = 0;

	return 0;
}


/**
 * This sets up the receive Descriptor queue in ring or chain mode.
 * This function is tightly coupled to the platform and operating system
 * Device is interested only after the descriptors are setup. Therefore this
 * function is not included in the device driver API. This function should be
 * treated as an example code to design the descriptor structures in ring mode
 * or chain mode.
 * This function depends on the device structure for allocation of
 * consistent dma-able memory in case of linux.
 *	- Allocates the memory for the descriptors.
 *	- Initialize the Busy and Next descriptors indices to 0(Indicating first
 *	  descriptor).
 *	- Initialize the Busy and Next descriptors to first descriptor address.
 *	- Initialize the last descriptor with the endof ring in case of ring
 *	   mode.
 *	- Initialize the descriptors in chain mode.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] pointer to device structure.
 * @param[in] number of descriptor expected in rx descriptor queue.
 * @param[in] whether descriptors to be created in RING mode or CHAIN mode.
 * @return 0 upon success. Error code upon failure.
 * @note This function fails if allocation fails for required number of
 * descriptors in Ring mode, but in chain mode function returns -ENOMEM in the
 * process of descriptor chain creation. once returned from this function user
 * should for gmacdev->rx_desc_count to see how many descriptors are there in
 * the chain.
 * Should continue further only if the number of descriptors in the
 * chain meets the requirements.
 */
static int32_t nss_gmac_setup_rx_desc_queue(struct nss_gmac_dev *gmacdev,
						struct device *dev,
						uint32_t no_of_desc,
						uint32_t desc_mode)
{
	int32_t i;
	struct dma_desc *first_desc = NULL;
	dma_addr_t dma_addr;

	gmacdev->rx_desc_count = 0;

	BUG_ON(desc_mode != RINGMODE);
	BUG_ON((no_of_desc & (no_of_desc - 1)) != 0);

	netdev_dbg(gmacdev->netdev, "total size of memory required for Rx Descriptors in Ring Mode = 0x%08x"
			, (uint32_t) ((sizeof(struct dma_desc) * no_of_desc)));

	first_desc = dma_alloc_coherent(dev, sizeof(struct dma_desc) * no_of_desc
					, &dma_addr, GFP_KERNEL);
	if (first_desc == NULL) {
		netdev_dbg(gmacdev->netdev, "Error in Rx Descriptor Memory allocation in Ring mode");
		return -ENOMEM;
	}

	gmacdev->rx_desc_count = no_of_desc;
	gmacdev->rx_desc = first_desc;
	gmacdev->rx_desc_dma = dma_addr;
	netdev_dbg(gmacdev->netdev, "Rx Descriptors in Ring Mode: No. of descriptors = %d base = 0x%08x dma = 0x%08x",
			no_of_desc, (uint32_t)first_desc, dma_addr);

	for (i = 0; i < gmacdev->rx_desc_count; i++) {
		nss_gmac_rx_desc_init_ring(gmacdev->rx_desc + i,
					i == (gmacdev->rx_desc_count - 1));
	}

	gmacdev->rx_next = 0;
	gmacdev->rx_busy = 0;
	gmacdev->rx_next_desc = gmacdev->rx_desc;
	gmacdev->rx_busy_desc = gmacdev->rx_desc;
	gmacdev->busy_rx_desc = 0;

	return 0;
}

/*
 * nss_gmac_rx_refill()
 *	Refill the RX descrptor
 */
static inline void nss_gmac_rx_refill(struct nss_gmac_dev *gmacdev)
{
	int count = NSS_GMAC_RX_DESC_SIZE - gmacdev->busy_rx_desc;
	dma_addr_t dma_addr;
	int i;
	struct sk_buff *skb;

	for (i = 0; i < count; i++) {
		skb = __netdev_alloc_skb(gmacdev->netdev,
				NSS_GMAC_MINI_JUMBO_FRAME_MTU, GFP_KERNEL);
		if (unlikely(skb == NULL)) {
			netdev_dbg(gmacdev->netdev, "Unable to allocate skb, will try next time");
			break;
		}
		skb_reserve(skb, NET_IP_ALIGN);
		dma_addr = dma_map_single(&gmacdev->netdev->dev, skb->data,
				NSS_GMAC_MINI_JUMBO_FRAME_MTU, DMA_FROM_DEVICE);
		nss_gmac_set_rx_qptr(gmacdev, dma_addr,
				NSS_GMAC_MINI_JUMBO_FRAME_MTU, (uint32_t)skb);
	}
}

/*
 * nss_gmac_rx()
 *	Process RX packets
 */
static inline int nss_gmac_rx(struct nss_gmac_dev *gmacdev, int budget)
{
	struct dma_desc *desc = NULL;
	int frame_length, busy;
	uint32_t status;
	struct sk_buff *rx_skb;

	if (!gmacdev->busy_rx_desc) {
		/* no desc are hold by gmac dma, we are done */
		return 0;
	}

	busy = gmacdev->busy_rx_desc;
	if (busy > budget)
		busy = budget;

	do {
		desc = gmacdev->rx_busy_desc;
		if (nss_gmac_is_desc_owned_by_dma(desc)) {
			/* desc still hold by gmac dma, so we are done */
			break;
		}

		status = desc->status;
		rx_skb = (struct sk_buff *)desc->reserved1;
		dma_unmap_single(&gmacdev->netdev->dev, desc->buffer1,
				NSS_GMAC_MINI_JUMBO_FRAME_MTU, DMA_FROM_DEVICE);

		if (likely(nss_gmac_is_rx_desc_valid(status))) {
			/* We have a pkt to process get the frame length */
			frame_length = nss_gmac_get_rx_desc_frame_length(status);
			/* Get rid of FCS: 4 */
			frame_length -= ETH_FCS_LEN;

			/* Valid packet, collect stats */
			gmacdev->stats.rx_packets++;
			gmacdev->stats.rx_bytes += frame_length;

			/* type_trans and deliver to linux */
			skb_put(rx_skb, frame_length);
			rx_skb->protocol = eth_type_trans(rx_skb, gmacdev->netdev);
			rx_skb->ip_summed = CHECKSUM_UNNECESSARY;
			napi_gro_receive(&gmacdev->napi, rx_skb);

		} else {
			gmacdev->stats.rx_errors++;
			dev_kfree_skb(rx_skb);

			if (status & (desc_rx_crc | desc_rx_collision |
					desc_rx_damaged | desc_rx_dribbling |
					desc_rx_length_error)) {
				gmacdev->stats.rx_crc_errors += (status & desc_rx_crc) ? 1 : 0;
				gmacdev->stats.collisions += (status & desc_rx_collision) ? 1 : 0;
				gmacdev->stats.rx_over_errors += (status & desc_rx_damaged) ? 1 : 0;
				gmacdev->stats.rx_frame_errors += (status & desc_rx_dribbling) ? 1 : 0;
				gmacdev->stats.rx_length_errors += (status & desc_rx_length_error) ? 1 : 0;
			}
		}

		nss_gmac_reset_rx_qptr(gmacdev);
		busy--;
	} while (busy > 0);
	return budget - busy;
}

/*
 * nss_gmac_process_tx_complete
 *	Xmit complete, clear descriptor and free the skb
 */
static inline void nss_gmac_process_tx_complete(struct nss_gmac_dev *gmacdev)
{
	int busy, len;
	uint32_t status;
	struct dma_desc *desc = NULL;
	struct sk_buff *skb;

	spin_lock(&gmacdev->slock);
	busy = gmacdev->busy_tx_desc;

	if (!busy) {
		/* No desc are hold by gmac dma, we are done */
		spin_unlock(&gmacdev->slock);
		return;
	}

	do {
		desc = gmacdev->tx_busy_desc;
		if (nss_gmac_is_desc_owned_by_dma(desc)) {
			/* desc still hold by gmac dma, so we are done */
			break;
		}
		len = (desc->length & desc_size1_mask) >> desc_size1_shift;
		dma_unmap_single(&gmacdev->netdev->dev, desc->buffer1, len,
								DMA_TO_DEVICE);

		status = desc->status;
		if (status & desc_tx_last) {
			/* TX is done for this whole skb, we can free it */
			skb = (struct sk_buff *)desc->reserved1;
			BUG_ON(!skb);
			dev_kfree_skb(skb);

			if (unlikely(status & desc_error)) {
				/* Some error happen, collect statistics */
				gmacdev->stats.tx_errors++;
				gmacdev->stats.tx_carrier_errors += (status & desc_tx_lost_carrier) ? 1 : 0;
				gmacdev->stats.tx_carrier_errors += (status & desc_tx_no_carrier) ? 1 : 0;
				gmacdev->stats.tx_window_errors += (status & desc_tx_late_collision) ? 1 : 0;
				gmacdev->stats.tx_fifo_errors += (status & desc_tx_underflow) ? 1 : 0;
			} else {
				/* No error, recored tx pkts/bytes and
				 * collision
				 */
				gmacdev->stats.tx_packets++;
				gmacdev->stats.collisions += nss_gmac_get_tx_collision_count(status);
				gmacdev->stats.tx_bytes += len;
			}
		}
		nss_gmac_reset_tx_qptr(gmacdev);
		busy--;
	} while (busy > 0);
	spin_unlock(&gmacdev->slock);
}

/*
 * nss_gmac_poll
 *	Scheduled by napi to process RX and TX complete
 */
int nss_gmac_poll(struct napi_struct *napi, int budget)
{
	struct nss_gmac_dev *gmacdev = container_of(napi,
					struct nss_gmac_dev, napi);
	int work_done;

	nss_gmac_process_tx_complete(gmacdev);
	work_done = nss_gmac_rx(gmacdev, budget);
	nss_gmac_rx_refill(gmacdev);

	if (work_done < budget) {
		napi_complete(napi);
		nss_gmac_enable_interrupt(gmacdev, dma_int_enable);
	}
	return work_done;
}

/*
 * nss_gmac_handle_irq
 *	Process IRQ and schedule napi
 */
irqreturn_t nss_gmac_handle_irq(int irq, void *ctx)
{
	struct nss_gmac_dev *gmacdev = (struct nss_gmac_dev *)ctx;

	nss_gmac_clear_interrupt(gmacdev);

	/*
	 * Disable interrupt and schedule napi
	 */
	nss_gmac_disable_interrupt(gmacdev, dma_int_enable);
	napi_schedule(&gmacdev->napi);
	return IRQ_HANDLED;
}

/*
 * nss_gmac_slowpath_if_open
 *	Do slow path data plane open
 */
static int nss_gmac_slowpath_if_open(void *app_data, uint32_t tx_desc_ring,
					uint32_t rx_desc_ring, uint32_t mode)
{
	return NSS_GMAC_SUCCESS;
}

static int nss_gmac_slowpath_if_close(void *app_data)
{
	return NSS_GMAC_SUCCESS;
}

static int nss_gmac_slowpath_if_link_state(void *app_data, uint32_t link_state)
{
	struct net_device *netdev = (struct net_device *)app_data;
	struct nss_gmac_dev *gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);

	if (link_state) {
		napi_enable(&gmacdev->napi);
		nss_gmac_enable_dma_rx(gmacdev);
		nss_gmac_enable_dma_tx(gmacdev);
		nss_gmac_enable_interrupt(gmacdev, dma_int_enable);
	} else if (gmacdev->link_state == LINKUP) {
		nss_gmac_disable_interrupt(gmacdev, dma_int_enable);
		napi_disable(&gmacdev->napi);
	}
	return NSS_GMAC_SUCCESS;
}

static int nss_gmac_slowpath_if_mac_addr(void *app_data, uint8_t *addr)
{
	return NSS_GMAC_SUCCESS;
}
static int nss_gmac_slowpath_if_change_mtu(void *app_data, uint32_t mtu)
{
	return NSS_GMAC_SUCCESS;
}

static int nss_gmac_slowpath_if_xmit(void *app_data, struct sk_buff *skb)
{
	struct net_device *netdev = (struct net_device *)app_data;
	struct nss_gmac_dev *gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);
	unsigned int len = skb_headlen(skb);
	dma_addr_t dma_addr;
	int nfrags = skb_shinfo(skb)->nr_frags;

	/*
	 * We don't have enough tx descriptor for this pkt, return busy
	 */
	if ((NSS_GMAC_TX_DESC_SIZE - gmacdev->busy_tx_desc) < nfrags + 1)
		return NETDEV_TX_BUSY;

	/*
	 * Most likely, it is not a fragmented pkt, optimize for that
	 */
	if (likely(nfrags == 0)) {
		dma_addr = dma_map_single(&netdev->dev, skb->data, len,
								DMA_TO_DEVICE);
		spin_lock(&gmacdev->slock);
		nss_gmac_set_tx_qptr(gmacdev, dma_addr, len, (uint32_t)skb,
				(skb->ip_summed == CHECKSUM_PARTIAL),
				(desc_tx_last | desc_tx_first),
				desc_own_by_dma);
		gmacdev->busy_tx_desc++;
		spin_unlock(&gmacdev->slock);
		nss_gmac_resume_dma_tx(gmacdev);

		return NSS_GMAC_SUCCESS;
	}

	/*
	 * Handle frag pkts here if we decided to
	 */

	return NSS_GMAC_FAILURE;
}

struct nss_gmac_data_plane_ops nss_gmac_slowpath_ops = {
	.open		= nss_gmac_slowpath_if_open,
	.close		= nss_gmac_slowpath_if_close,
	.link_state	= nss_gmac_slowpath_if_link_state,
	.mac_addr	= nss_gmac_slowpath_if_mac_addr,
	.change_mtu	= nss_gmac_slowpath_if_change_mtu,
	.xmit		= nss_gmac_slowpath_if_xmit,
};

/**
 * @brief Save GMAC statistics
 * @param[in] pointer to gmac context
 * @param[in] pointer to gmac statistics
 * @return Returns void.
 */
static void nss_gmac_copy_stats(struct nss_gmac_dev *gmacdev,
				struct nss_gmac_stats *gstat)
{
	BUG_ON(!spin_is_locked(&gmacdev->stats_lock));

	gmacdev->nss_stats.rx_bytes += gstat->rx_bytes;
	gmacdev->nss_stats.rx_packets += gstat->rx_packets;
	gmacdev->nss_stats.rx_errors += gstat->rx_errors;
	gmacdev->nss_stats.rx_receive_errors += gstat->rx_receive_errors;
	gmacdev->nss_stats.rx_overflow_errors += gstat->rx_overflow_errors;
	gmacdev->nss_stats.rx_descriptor_errors += gstat->rx_descriptor_errors;
	gmacdev->nss_stats.rx_watchdog_timeout_errors +=
		gstat->rx_watchdog_timeout_errors;
	gmacdev->nss_stats.rx_crc_errors += gstat->rx_crc_errors;
	gmacdev->nss_stats.rx_late_collision_errors +=
		gstat->rx_late_collision_errors;
	gmacdev->nss_stats.rx_dribble_bit_errors += gstat->rx_dribble_bit_errors;
	gmacdev->nss_stats.rx_length_errors += gstat->rx_length_errors;
	gmacdev->nss_stats.rx_ip_header_errors += gstat->rx_ip_header_errors;
	gmacdev->nss_stats.rx_ip_payload_errors += gstat->rx_ip_payload_errors;
	gmacdev->nss_stats.rx_no_buffer_errors += gstat->rx_no_buffer_errors;
	gmacdev->nss_stats.rx_transport_csum_bypassed +=
		gstat->rx_transport_csum_bypassed;
	gmacdev->nss_stats.tx_bytes += gstat->tx_bytes;
	gmacdev->nss_stats.tx_packets += gstat->tx_packets;
	gmacdev->nss_stats.tx_collisions += gstat->tx_collisions;
	gmacdev->nss_stats.tx_errors += gstat->tx_errors;
	gmacdev->nss_stats.tx_jabber_timeout_errors +=
		gstat->tx_jabber_timeout_errors;
	gmacdev->nss_stats.tx_frame_flushed_errors +=
		gstat->tx_frame_flushed_errors;
	gmacdev->nss_stats.tx_loss_of_carrier_errors +=
		gstat->tx_loss_of_carrier_errors;
	gmacdev->nss_stats.tx_no_carrier_errors += gstat->tx_no_carrier_errors;
	gmacdev->nss_stats.tx_late_collision_errors +=
		gstat->tx_late_collision_errors;
	gmacdev->nss_stats.tx_excessive_collision_errors +=
		gstat->tx_excessive_collision_errors;
	gmacdev->nss_stats.tx_excessive_deferral_errors +=
		gstat->tx_excessive_deferral_errors;
	gmacdev->nss_stats.tx_underflow_errors += gstat->tx_underflow_errors;
	gmacdev->nss_stats.tx_ip_header_errors += gstat->tx_ip_header_errors;
	gmacdev->nss_stats.tx_ip_payload_errors += gstat->tx_ip_payload_errors;
	gmacdev->nss_stats.tx_dropped += gstat->tx_dropped;
	gmacdev->nss_stats.hw_errs[0] += gstat->hw_errs[0];
	gmacdev->nss_stats.hw_errs[1] += gstat->hw_errs[1];
	gmacdev->nss_stats.hw_errs[2] += gstat->hw_errs[2];
	gmacdev->nss_stats.hw_errs[3] += gstat->hw_errs[3];
	gmacdev->nss_stats.hw_errs[4] += gstat->hw_errs[4];
	gmacdev->nss_stats.hw_errs[5] += gstat->hw_errs[5];
	gmacdev->nss_stats.hw_errs[6] += gstat->hw_errs[6];
	gmacdev->nss_stats.hw_errs[7] += gstat->hw_errs[7];
	gmacdev->nss_stats.hw_errs[8] += gstat->hw_errs[8];
	gmacdev->nss_stats.hw_errs[9] += gstat->hw_errs[9];
	gmacdev->nss_stats.rx_missed += gstat->rx_missed;
	gmacdev->nss_stats.fifo_overflows += gstat->fifo_overflows;
	gmacdev->nss_stats.rx_scatter_errors += gstat->rx_scatter_errors;
	gmacdev->nss_stats.gmac_total_ticks += gstat->gmac_total_ticks;
	gmacdev->nss_stats.gmac_worst_case_ticks += gstat->gmac_worst_case_ticks;
	gmacdev->nss_stats.gmac_iterations += gstat->gmac_iterations;
}


/**
 * @brief Stats Callback to receive statistics from NSS
 * @param[in] pointer to gmac context
 * @param[in] pointer to gmac statistics
 * @return Returns void.
 */
static void nss_gmac_stats_receive(struct nss_gmac_dev *gmacdev,
					struct nss_gmac_stats *gstat)
{
	struct net_device *netdev = NULL;

	netdev = (struct net_device *)gmacdev->netdev;

	if (!test_bit(__NSS_GMAC_UP, &gmacdev->flags))
		return;

	spin_lock(&gmacdev->stats_lock);

	nss_gmac_copy_stats(gmacdev, gstat);

	gmacdev->stats.rx_packets += gstat->rx_packets;
	gmacdev->stats.rx_bytes += gstat->rx_bytes;
	gmacdev->stats.rx_errors += gstat->rx_errors;
	gmacdev->stats.rx_dropped += gstat->rx_errors;
	gmacdev->stats.rx_length_errors += gstat->rx_length_errors;
	gmacdev->stats.rx_over_errors += gstat->rx_overflow_errors;
	gmacdev->stats.rx_crc_errors += gstat->rx_crc_errors;
	gmacdev->stats.rx_frame_errors += gstat->rx_dribble_bit_errors;
	gmacdev->stats.rx_fifo_errors += gstat->fifo_overflows;
	gmacdev->stats.rx_missed_errors += gstat->rx_missed;
	gmacdev->stats.collisions += gstat->tx_collisions
		+ gstat->rx_late_collision_errors;
	gmacdev->stats.tx_packets += gstat->tx_packets;
	gmacdev->stats.tx_bytes += gstat->tx_bytes;
	gmacdev->stats.tx_errors += gstat->tx_errors;
	gmacdev->stats.tx_dropped += gstat->tx_dropped;
	gmacdev->stats.tx_carrier_errors += gstat->tx_loss_of_carrier_errors
		+ gstat->tx_no_carrier_errors;
	gmacdev->stats.tx_fifo_errors += gstat->tx_underflow_errors;
	gmacdev->stats.tx_window_errors += gstat->tx_late_collision_errors;

	spin_unlock(&gmacdev->stats_lock);
}
EXPORT_SYMBOL(nss_gmac_receive);

/**
 * NSS Driver interface APIs
 */

/**
 * @brief Rx Callback to receive frames from NSS
 * @param[in] pointer to net device context
 * @param[in] pointer to skb
 * @return Returns void
 */
void nss_gmac_receive(struct net_device *netdev, struct sk_buff *skb,
						struct napi_struct *napi)
{
	struct nss_gmac_dev *gmacdev;

	BUG_ON(netdev == NULL);

	gmacdev = netdev_priv(netdev);

	BUG_ON(gmacdev->netdev != netdev);

	skb->dev = netdev;
	skb->protocol = eth_type_trans(skb, netdev);
	netdev_dbg(netdev,
			"%s: Rx on gmac%d, packet len %d, CSUM %d",
			__func__, gmacdev->macid, skb->len, skb->ip_summed);

	napi_gro_receive(napi, skb);
}


/**
 * @brief Event Callback to receive events from NSS
 * @param[in] pointer to net device context
 * @param[in] event type
 * @param[in] pointer to buffer
 * @param[in] length of buffer
 * @return Returns void
 */
void nss_gmac_event_receive(void *if_ctx, int ev_type,
				void *os_buf, uint32_t len)
{
	struct net_device *netdev = NULL;
	struct nss_gmac_dev *gmacdev = NULL;

	netdev = (struct net_device *)if_ctx;
	gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);
	BUG_ON(!gmacdev);

	switch (ev_type) {
	case NSS_GMAC_EVENT_STATS:
		nss_gmac_stats_receive(gmacdev,
				(struct nss_gmac_stats *)os_buf);
		break;

	default:
		netdev_dbg(netdev, "%s: Unknown Event from NSS", __func__);
		break;
	}
}
EXPORT_SYMBOL(nss_gmac_event_receive);

/**
 * @brief Notify linkup event to NSS
 * @param[in] pointer to gmac context
 * @return Returns void.
 */
static void nss_notify_linkup(struct nss_gmac_dev *gmacdev)
{
	uint32_t link = 0;

	if (!test_bit(__NSS_GMAC_UP, &gmacdev->flags))
		return;

	link = 0x1;
	if (gmacdev->speed == SPEED1000)
		link |= 0x4;
	else if (gmacdev->speed == SPEED100)
		link |= 0x2;

	gmacdev->data_plane_ops->link_state(gmacdev->data_plane_ctx, link);
}

/**
 * This function checks for completion of PHY init
 * and proceeds to initialize mac based on parameters
 * read from PHY registers. It indicates presence of carrier to OS.
 * @param[in] pointer to gmac context
 * @return Returns void.
 */
void nss_gmac_linkup(struct nss_gmac_dev *gmacdev)
{
	struct net_device *netdev = gmacdev->netdev;
	uint32_t gmac_tx_desc = 0, gmac_rx_desc = 0;
	uint32_t mode = NSS_GMAC_MODE0;

	nss_gmac_spare_ctl(gmacdev);

	if (nss_gmac_check_phy_init(gmacdev) != 0) {
		gmacdev->link_state = LINKDOWN;
		return;
	}

	gmacdev->link_state = LINKUP;
	if (nss_gmac_dev_set_speed(gmacdev) != 0)
		return;

	if (gmacdev->first_linkup_done == 0) {
		nss_gmac_disable_interrupt_all(gmacdev);
		nss_gmac_reset(gmacdev);
		nss_gmac_clear_interrupt(gmacdev);

		/* Program Tx/Rx descriptor base addresses */
		nss_gmac_init_tx_desc_base(gmacdev);
		nss_gmac_init_rx_desc_base(gmacdev);
		nss_gmac_dma_bus_mode_init(gmacdev, dma_bus_mode_val);
		nss_gmac_dma_axi_bus_mode_init(gmacdev, dma_axi_bus_mode_val);
		nss_gmac_dma_control_init(gmacdev, dma_omr);
		nss_gmac_disable_mmc_tx_interrupt(gmacdev, 0xFFFFFFFF);
		nss_gmac_disable_mmc_rx_interrupt(gmacdev, 0xFFFFFFFF);
		nss_gmac_disable_mmc_ipc_rx_interrupt(gmacdev, 0xFFFFFFFF);

		/* Restore the Jumbo support settings as per corresponding
		 * interface mtu
		 */
		nss_gmac_linux_change_mtu(gmacdev->netdev, gmacdev->netdev->mtu);
		gmacdev->first_linkup_done = 1;
	}

	nss_gmac_mac_init(gmacdev);

	if (gmacdev->data_plane_ops->open(gmacdev->data_plane_ctx, gmac_tx_desc,
				gmac_rx_desc, mode) != NSS_GMAC_SUCCESS) {
		netdev_dbg(netdev, "%s: data plane open command un-successful",
								__func__);
		gmacdev->link_state = LINKDOWN;
		return;
	}
	netdev_dbg(netdev, "%s: data plane open command successfully issued",
								__func__);

	nss_notify_linkup(gmacdev);

	netif_carrier_on(netdev);
}


/**
 * Save current state of link and
 * indicate absence of carrier to OS.
 * @param[in] nss_gmac_dev *
 * @return Returns void.
 */
void nss_gmac_linkdown(struct nss_gmac_dev *gmacdev)
{
	struct net_device *netdev = gmacdev->netdev;

	netdev_info(netdev, "Link down");

	if (test_bit(__NSS_GMAC_UP, &gmacdev->flags)) {
		netif_carrier_off(netdev);

		gmacdev->data_plane_ops->link_state(gmacdev->data_plane_ctx, 0);
	}
	gmacdev->link_state = LINKDOWN;
	gmacdev->duplex_mode = 0;
	gmacdev->speed = 0;
}


/**
 * @brief Link state change callback
 * @param[in] struct net_device *
 * @return Returns void.
 */
void nss_gmac_adjust_link(struct net_device *netdev)
{
	int32_t status = 0;
	struct nss_gmac_dev *gmacdev = NULL;

	gmacdev = netdev_priv(netdev);

	if (!test_bit(__NSS_GMAC_UP, &gmacdev->flags))
		return;

	status = nss_gmac_check_link(gmacdev);
	mutex_lock(&gmacdev->link_mutex);
	if (status == LINKUP && gmacdev->link_state == LINKDOWN)
		nss_gmac_linkup(gmacdev);
	else if (status == LINKDOWN && gmacdev->link_state == LINKUP) {
		/*
		 * Process a link down notification only if
		 * link polling is enabled via private flags.
		 */
		if (gmacdev->drv_flags & NSS_GMAC_PRIV_FLAG(LINKPOLL)) {
			nss_gmac_linkdown(gmacdev);
		}
	}
	mutex_unlock(&gmacdev->link_mutex);
}

void nss_gmac_start_up(struct nss_gmac_dev *gmacdev)
{
	if (test_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags)) {
		if (!IS_ERR(gmacdev->phydev)) {
			netdev_dbg(gmacdev->netdev, "%s: start phy 0x%x",
					__func__, gmacdev->phydev->phy_id);
			phy_start(gmacdev->phydev);
			phy_start_aneg(gmacdev->phydev);
		} else {
			netdev_dbg(gmacdev->netdev, "%s: Invalid PHY device for a link polled interface",
								__func__);
		}
		return;
	}
	netdev_dbg(gmacdev->netdev, "%s: Force link up", __func__);
	/*
	 * Force link up if link polling is disabled
	 */
	mutex_lock(&gmacdev->link_mutex);
	nss_gmac_linkup(gmacdev);
	mutex_unlock(&gmacdev->link_mutex);
}

/**
 * @brief Function to transmit a given packet on the wire.
 *
 * Whenever Linux Kernel has a packet ready to be transmitted, this function is
 * called.
 * The function prepares a packet and prepares the descriptor and
 * enables/resumes the transmission.
 * @param[in] pointer to sk_buff structure.
 * @param[in] pointer to net_device structure.
 * @return NETDEV_TX_xxx
 */
int32_t nss_gmac_linux_xmit_frames(struct sk_buff *skb,
						struct net_device *netdev)
{
	int msg_status = 0;
	struct nss_gmac_dev *gmacdev = NULL;

	BUG_ON(skb == NULL);
	if (skb->len < ETH_HLEN) {
		netdev_dbg(netdev, "%s: skb->len < ETH_HLEN", __func__);
		goto drop;
	}

	gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);
	BUG_ON(gmacdev == NULL);
	BUG_ON(gmacdev->netdev != netdev);

	netdev_dbg(netdev, "%s:Tx packet, len %d, CSUM %d",
			__func__, skb->len, skb->ip_summed);

	msg_status = gmacdev->data_plane_ops->xmit(gmacdev->data_plane_ctx, skb);

	if (likely(msg_status == NSS_GMAC_SUCCESS))
		goto tx_done;

drop:
	/*
	 * Now drop it
	 */
	netdev_dbg(netdev, "dropping skb");
	dev_kfree_skb_any(skb);
	netdev->stats.tx_dropped++;

tx_done:
	return NETDEV_TX_OK;
}

/**
 * @brief Function used when the interface is opened for use.
 *
 * We register nss_gmac_linux_open function to linux open(). Basically this
 * function prepares the the device for operation. This function is called
 * whenever ifconfig (in Linux) activates the device (for example
 * "ifconfig eth0 up"). This function registers system resources needed.
 *	- Disables interrupts
 *	- Starts Linux network queue interface
 *	- Checks for NSS init completion and determines initial link status
 *	- Starts timer to detect cable plug/unplug
 * @param[in] pointer to net_device structure.
 * @return Returns 0 on success and error status upon failure.
 */
int nss_gmac_linux_open(struct net_device *netdev)
{
	struct device *dev = NULL;
	struct nss_gmac_dev *gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);
	struct nss_gmac_global_ctx *ctx = NULL;
	int err;

	if (!gmacdev)
		return -EINVAL;

	dev = &netdev->dev;
	ctx = gmacdev->ctx;

	netif_carrier_off(netdev);

	if (!gmacdev->data_plane_ops) {
		netdev_dbg(netdev, "%s: offload is not enabled, bring up gmac with slowpath",
								__func__);

		netif_napi_add(netdev, &gmacdev->napi, nss_gmac_poll,
							NSS_GMAC_NAPI_BUDGET);
		/* Initial the RX/TX ring */
		dma_set_coherent_mask(dev, 0xffffffff);
		nss_gmac_setup_rx_desc_queue(gmacdev, dev,
					NSS_GMAC_RX_DESC_SIZE, RINGMODE);
		nss_gmac_setup_tx_desc_queue(gmacdev, dev,
					NSS_GMAC_TX_DESC_SIZE, RINGMODE);
		nss_gmac_rx_refill(gmacdev);

		/* Register IRQ */
		err = request_irq(netdev->irq, nss_gmac_handle_irq,
					IRQF_DISABLED, "nss-gmac", gmacdev);
		if (err) {
			netdev_dbg(netdev, "Mac %d IRQ %d request failed",
						gmacdev->macid, netdev->irq);
			return err;
		}

		gmacdev->data_plane_ops = &nss_gmac_slowpath_ops;
		gmacdev->data_plane_ctx = gmacdev->netdev;
	}

	/**
	 * Now platform dependent initialization.
	 */
	nss_gmac_disable_interrupt_all(gmacdev);

	gmacdev->speed = SPEED100;
	gmacdev->duplex_mode = FULLDUPLEX;

	/**
	 * Lets read the version of ip in to device structure
	 */
	nss_gmac_read_version(gmacdev);

	/*
	 * Inform the Linux Networking stack about the hardware
	 * capability of checksum offloading and other features.
	 */
	netdev->features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM | NETIF_F_TSO | NETIF_F_SG | NETIF_F_FRAGLIST | NETIF_F_UFO | NETIF_F_TSO6;
	netdev->hw_features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM | NETIF_F_TSO | NETIF_F_SG | NETIF_F_FRAGLIST | NETIF_F_UFO | NETIF_F_TSO6;
	netdev->vlan_features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM | NETIF_F_TSO | NETIF_F_SG | NETIF_F_FRAGLIST | NETIF_F_UFO | NETIF_F_TSO6;
	netdev->wanted_features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM | NETIF_F_TSO | NETIF_F_SG | NETIF_F_FRAGLIST | NETIF_F_UFO | NETIF_F_TSO6;

	/**
	 * Set GMAC state to UP before link state is checked
	 */
	test_and_set_bit(__NSS_GMAC_UP, &gmacdev->flags);
	netif_start_queue(netdev);

	gmacdev->link_state = LINKDOWN;

	nss_gmac_start_up(gmacdev);

	gmacdev->data_plane_ops->mac_addr(gmacdev->data_plane_ctx,
					(uint8_t *)gmacdev->netdev->dev_addr);

	return 0;
}

/**
 * @brief Function used when the interface is closed.
 *
 * This function is registered to linux stop() function. This function is
 * called whenever ifconfig (in Linux) closes the device (for example
 * "ifconfig eth0 down"). This releases all the system resources allocated
 * during open call.
 *	- Disable the device interrupts
 *	- Send a link change event to NSS GMAC driver.
 *	- Stop the Linux network queue interface
 *	- Cancel timer rgistered for cable plug/unplug tracking
 * @param[in] pointer to net_device structure.
 * @return Returns 0 on success and error status upon failure.
 */
int nss_gmac_linux_close(struct net_device *netdev)
{
	struct nss_gmac_dev *gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);

	if (!gmacdev)
		return -EINVAL;

	test_and_set_bit(__NSS_GMAC_CLOSING, &gmacdev->flags);

	netif_stop_queue(netdev);
	netif_carrier_off(netdev);

	nss_gmac_rx_disable(gmacdev);
	nss_gmac_tx_disable(gmacdev);

	nss_gmac_disable_interrupt_all(gmacdev);
	gmacdev->data_plane_ops->link_state(gmacdev->data_plane_ctx, 0);

	if (!IS_ERR(gmacdev->phydev))
		phy_stop(gmacdev->phydev);

	test_and_clear_bit(__NSS_GMAC_UP, &gmacdev->flags);
	test_and_clear_bit(__NSS_GMAC_CLOSING, &gmacdev->flags);

	gmacdev->data_plane_ops->close(gmacdev->data_plane_ctx);

	return 0;
}

/**
 * @brief Function to handle a Tx Hang.
 * This is a software hook (Linux) to handle transmitter hang if any.
 * @param[in] pointer to net_device structure
 * @return void.
 */
void nss_gmac_linux_tx_timeout(struct net_device *netdev)
{
	struct nss_gmac_dev *gmacdev = NULL;

	gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);
	BUG_ON(gmacdev == NULL);

	if (gmacdev->gmac_power_down == 0) {
		/* If Mac is in powerdown */
		netdev_dbg(netdev,
				"%s TX time out during power down is ignored",
				netdev->name);
		return;
	}

	netif_carrier_off(netdev);
	nss_gmac_disable_dma_tx(gmacdev);
	nss_gmac_flush_tx_fifo(gmacdev);
	nss_gmac_enable_dma_tx(gmacdev);
	netif_carrier_on(netdev);
	netif_start_queue(netdev);
}


/**
 * @brief Function to change the Maximum Transfer Unit.
 * @param[in] pointer to net_device structure.
 * @param[in] New value for maximum frame size.
 * @return Returns 0 on success Errorcode on failure.
 */
int32_t nss_gmac_linux_change_mtu(struct net_device *netdev, int32_t newmtu)
{
	struct nss_gmac_dev *gmacdev = NULL;

	gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);
	if (!gmacdev)
		return -EINVAL;

	if (newmtu > NSS_GMAC_JUMBO_MTU)
		return -EINVAL;

	if (gmacdev->data_plane_ops->change_mtu(gmacdev->data_plane_ctx, newmtu)
							 != NSS_GMAC_SUCCESS)
		return -EAGAIN;

	if (newmtu <= NSS_GMAC_NORMAL_FRAME_MTU) {
		nss_gmac_jumbo_frame_disable(gmacdev);
		nss_gmac_twokpe_frame_disable(gmacdev);
	} else if (newmtu <= NSS_GMAC_MINI_JUMBO_FRAME_MTU) {
		nss_gmac_jumbo_frame_disable(gmacdev);
		nss_gmac_twokpe_frame_enable(gmacdev);
	} else if (newmtu <= NSS_GMAC_FULL_JUMBO_FRAME_MTU) {
		nss_gmac_jumbo_frame_enable(gmacdev);
	}

	netdev->mtu = newmtu;
	return 0;
}

/*
 * nss_gmac_is_in_open_state()
 *	Return if a gmac is opened or not
 */
bool nss_gmac_is_in_open_state(struct net_device *netdev)
{
	struct nss_gmac_dev *gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);

	if (test_bit(__NSS_GMAC_UP, &gmacdev->flags))
		return true;
	return false;
}
EXPORT_SYMBOL(nss_gmac_is_in_open_state);

/*
 * nss_gmac_register_offload()
 *
 * @param[netdev] netdev instance that is going to register
 * @param[dp_ops] dataplan ops for chaning mac addr/mtu/link status
 * @param[ctx] passing the ctx of this nss_phy_if to gmac
 *
 * @return Return SUCCESS or FAILURE
 */
int nss_gmac_override_data_plane(struct net_device *netdev,
				struct nss_gmac_data_plane_ops *dp_ops,
				void *ctx)
{
	struct nss_gmac_dev *gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);

	BUG_ON(!gmacdev);

	if (!dp_ops->open || !dp_ops->close || !dp_ops->link_state
		|| !dp_ops->mac_addr || !dp_ops->change_mtu || !dp_ops->xmit) {
		netdev_dbg(netdev, "%s: All the op functions must be present, reject this registeration",
								__func__);
		return NSS_GMAC_FAILURE;
	}

	/*
	 * If this gmac is up, close the netdev to force TX/RX stop
	 */
	if (test_bit(__NSS_GMAC_UP, &gmacdev->flags))
		nss_gmac_linux_close(netdev);

	/* Recored the data_plane_ctx, data_plane_ops */
	gmacdev->data_plane_ctx = ctx;
	gmacdev->data_plane_ops = dp_ops;
	gmacdev->first_linkup_done = 0;

	return NSS_GMAC_SUCCESS;
}
EXPORT_SYMBOL(nss_gmac_override_data_plane);

/*
 * nss_gmac_restore_data_plane()
 *	Data plane to inform netdev it is ready to start
 * @param[netdev] net_device context
 * @param[ctx] context of the data plane
 */
void nss_gmac_start_data_plane(struct net_device *netdev, void *ctx)
{
	struct nss_gmac_dev *gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);
	struct nss_gmac_global_ctx *global_ctx = gmacdev->ctx;

	if (test_bit(__NSS_GMAC_UP, &gmacdev->flags)) {
		netdev_dbg(netdev, "This netdev already up, something is wrong\n");
		return;
	}
	if (gmacdev->data_plane_ctx == ctx) {
		netdev_dbg(netdev, "Data plane cookie matches, let's start the netdev again\n");
		queue_delayed_work(global_ctx->gmac_workqueue,
				&gmacdev->gmacwork, NSS_GMAC_LINK_CHECK_TIME);
	}
}
EXPORT_SYMBOL(nss_gmac_start_data_plane);

/*
 * gmac_unregister_nss_if()
 *
 * @param[if_num] gmac device id - 0~3
 */
void nss_gmac_restore_data_plane(struct net_device *netdev)
{
	struct nss_gmac_dev *gmacdev = (struct nss_gmac_dev *)netdev_priv(netdev);

	/*
	 * If this gmac is up, close the netdev to force TX/RX stop
	 */
	if (test_bit(__NSS_GMAC_UP, &gmacdev->flags))
		nss_gmac_linux_close(netdev);
	gmacdev->data_plane_ctx = netdev;
	gmacdev->data_plane_ops = &nss_gmac_slowpath_ops;
}
EXPORT_SYMBOL(nss_gmac_restore_data_plane);

/*
 * nss_gmac_get_netdev_by_macid()
 *	return the net device of the corrsponding macid if exist
 */
struct net_device *nss_gmac_get_netdev_by_macid(int macid)
{
	struct nss_gmac_dev *gmacdev = ctx.nss_gmac[macid];

	if (!gmacdev)
		return NULL;
	return gmacdev->netdev;
}
EXPORT_SYMBOL(nss_gmac_get_netdev_by_macid);

/*
 * nss_gmac_open_work()
 *	Schedule delayed work to open the netdev again
 */
void nss_gmac_open_work(struct work_struct *work)
{
	struct nss_gmac_dev *gmacdev = container_of(to_delayed_work(work),
						struct nss_gmac_dev, gmacwork);

	netdev_dbg(gmacdev->netdev, "Do the network up in delayed queue %s\n",
							gmacdev->netdev->name);
	nss_gmac_linux_open(gmacdev->netdev);
}
