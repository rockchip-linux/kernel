/*
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
 * OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
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
 *-----------------------------REVISION HISTORY----------------------------------
 * Qualcomm Atheros    		15/Feb/2013			Created
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/bitops.h>
#include <linux/phy.h>

#include <nss_gmac_dev.h>
#include <nss_gmac_network_interface.h>

#include <nss_api_if.h>

/**
 * @brief Save GMAC statistics
 * @param[in] pointer to gmac context
 * @param[in] pointer to gmac statistics
 * @return Returns void.
 */
static void nss_gmac_copy_stats(nss_gmac_dev *gmacdev,
				struct nss_gmac_sync *gstat)
{
	struct nss_gmac_sync *pstat = NULL;

	pstat = &(gmacdev->nss_stats);

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
	pstat->hw_errs[0] += gstat->hw_errs[0];
	pstat->hw_errs[1] += gstat->hw_errs[1];
	pstat->hw_errs[2] += gstat->hw_errs[2];
	pstat->hw_errs[3] += gstat->hw_errs[3];
	pstat->hw_errs[4] += gstat->hw_errs[4];
	pstat->hw_errs[5] += gstat->hw_errs[5];
	pstat->hw_errs[6] += gstat->hw_errs[6];
	pstat->hw_errs[7] += gstat->hw_errs[7];
	pstat->hw_errs[8] += gstat->hw_errs[8];
	pstat->hw_errs[9] += gstat->hw_errs[9];
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
static void nss_gmac_stats_receive(nss_gmac_dev *gmacdev,
				   struct nss_gmac_sync *gstat)
{
	struct net_device *netdev = NULL;

	netdev = (struct net_device *)gmacdev->netdev;

	if (!test_bit(__NSS_GMAC_UP, &gmacdev->flags)) {
		return;
	}

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


/**
 * NSS Driver interface APIs
 */

/**
 * @brief Rx Callback to receive frames from NSS
 * @param[in] pointer to net device context
 * @param[in] pointer to skb
 * @return Returns void
 */
void nss_gmac_receive(void *if_ctx, void *os_buf)
{
	struct net_device *netdev;
	struct sk_buff *skb;
	nss_gmac_dev *gmacdev;

	BUG_ON(if_ctx == NULL);

	netdev = (struct net_device *)if_ctx;
	gmacdev = netdev_priv(netdev);

	BUG_ON(gmacdev->netdev != netdev);

	skb = (struct sk_buff *)os_buf;
	skb->dev = netdev;
	skb->protocol = eth_type_trans(skb, netdev);
	nss_gmac_info(gmacdev,
		      "%s: Rx on gmac%d, packet len %d, CSUM %d",
		      __FUNCTION__, gmacdev->macid, skb->len, skb->ip_summed);

	napi_gro_receive(gmacdev->napi, skb);
}


/**
 * @brief Event Callback to receive events from NSS
 * @param[in] pointer to net device context
 * @param[in] event type
 * @param[in] pointer to buffer
 * @param[in] length of buffer
 * @return Returns void
 */
void nss_gmac_event_receive(void *if_ctx, nss_gmac_event_t ev_type,
			    void *os_buf, uint32_t len)
{
	struct net_device *netdev = NULL;
	nss_gmac_dev *gmacdev = NULL;

	netdev = (struct net_device *)if_ctx;
	gmacdev = (nss_gmac_dev *)netdev_priv(netdev);
	BUG_ON(!gmacdev);

	switch (ev_type) {
	case NSS_GMAC_EVENT_STATS:
		nss_gmac_stats_receive(gmacdev, (struct nss_gmac_sync *)os_buf);
		break;

	default:
		nss_gmac_info(gmacdev, "%s: Unknown Event from NSS",
			      __FUNCTION__);
		break;
	}
}


/**
 * @brief Notify linkup event to NSS
 * @param[in] pointer to gmac context
 * @return Returns void.
 */
static void nss_notify_linkup(nss_gmac_dev *gmacdev)
{
	uint32_t link = 0;

	if (!test_bit(__NSS_GMAC_UP, &gmacdev->flags) || !gmacdev->notify_open) {
		return;
	}

	link = 0x1;
	if (gmacdev->speed == SPEED1000) {
		link |= 0x4;
	} else if (gmacdev->speed == SPEED100) {
		link |= 0x2;
	}

	nss_tx_phys_if_link_state(gmacdev->nss_gmac_ctx,
			       link, gmacdev->macid);
}

/**
 * This function checks for completion of PHY init
 * and proceeds to initialize mac based on parameters
 * read from PHY registers. It indicates presence of carrier to OS.
 * @param[in] pointer to gmac context
 * @return Returns void.
 */
void nss_gmac_linkup(nss_gmac_dev *gmacdev)
{
	struct net_device *netdev = gmacdev->netdev;
	uint32_t gmac_tx_desc = 0, gmac_rx_desc = 0;

	nss_gmac_spare_ctl(gmacdev);

	if (nss_gmac_check_phy_init(gmacdev) != 0) {
		gmacdev->link_state = LINKDOWN;
		return;
	}

	gmacdev->link_state = LINKUP;
	if (nss_gmac_dev_set_speed(gmacdev) != 0) {
		return;
	}

	if (gmacdev->first_linkup_done == 0) {
		nss_gmac_disable_interrupt_all(gmacdev);
		nss_gmac_reset(gmacdev);
		nss_gmac_clear_interrupt(gmacdev);

		/* Program Tx/Rx descriptor base addresses */
		nss_gmac_init_tx_desc_base(gmacdev);
		nss_gmac_init_rx_desc_base(gmacdev);
		nss_gmac_dma_bus_mode_init(gmacdev, DmaBusModeVal);
		nss_gmac_dma_axi_bus_mode_init(gmacdev, DmaAxiBusModeVal);
		nss_gmac_dma_control_init(gmacdev, DmaOMR);
		nss_gmac_disable_mmc_tx_interrupt(gmacdev, 0xFFFFFFFF);
		nss_gmac_disable_mmc_rx_interrupt(gmacdev, 0xFFFFFFFF);
		nss_gmac_disable_mmc_ipc_rx_interrupt(gmacdev, 0xFFFFFFFF);

		/* Restore the Jumbo support settings as per corresponding interface mtu */
		nss_gmac_linux_change_mtu(gmacdev->netdev, gmacdev->netdev->mtu);
		gmacdev->first_linkup_done = 1;
	}

	nss_gmac_mac_init(gmacdev);

	if (gmacdev->notify_open == 0) {
		/*
		 * The NSS allocates the descriptors in TCM, so it
		 * does not expect descriptors from host.
		 */
		if (nss_tx_phys_if_open(gmacdev->nss_gmac_ctx,
				     gmac_tx_desc, gmac_rx_desc,
				     gmacdev->macid) != NSS_TX_SUCCESS) {
			nss_gmac_info(gmacdev,
				      "%s: NSS open command un-successful",
				      __FUNCTION__);
			gmacdev->link_state = LINKDOWN;
			return;
		}

		nss_gmac_info(gmacdev,
			      "%s: NSS open command successfully issued",
			      __FUNCTION__);
		gmacdev->notify_open = 1;
	}

	nss_notify_linkup(gmacdev);

	netif_carrier_on(netdev);
}


/**
 * Save current state of link and
 * indicate absence of carrier to OS.
 * @param[in] nss_gmac_dev *
 * @return Returns void.
 */
void nss_gmac_linkdown(nss_gmac_dev *gmacdev)
{
	struct net_device *netdev = gmacdev->netdev;

	nss_gmac_msg("%s Link %s", netdev->name, "down");

	gmacdev->link_state = LINKDOWN;
	gmacdev->duplex_mode = 0;
	gmacdev->speed = 0;

	if (test_bit(__NSS_GMAC_UP, &gmacdev->flags)) {
		netif_carrier_off(netdev);
		nss_tx_phys_if_link_state(gmacdev->nss_gmac_ctx, 0,
				       gmacdev->macid);
	}
}


/**
 * @brief Link state change callback
 * @param[in] struct net_device *
 * @return Returns void.
 */
void nss_gmac_adjust_link(struct net_device *netdev)
{
	int32_t status = 0;
	nss_gmac_dev *gmacdev = NULL;

	gmacdev = netdev_priv(netdev);

	if (gmacdev->nss_state != NSS_STATE_INITIALIZED
		|| !test_bit(__NSS_GMAC_UP, &gmacdev->flags)) {
		return;
	}

	status = nss_gmac_check_link(gmacdev);

	mutex_lock(&gmacdev->link_mutex);
	if (status == LINKUP && gmacdev->link_state == LINKDOWN) {
		nss_gmac_linkup(gmacdev);
	} else if (status == LINKDOWN && gmacdev->link_state == LINKUP) {
		nss_gmac_linkdown(gmacdev);
	}
	mutex_unlock(&gmacdev->link_mutex);
}


/**
 * @brief Gmac work function to check for NSS initialization.
 * @param[in] struct work_struct *
 * @return Returns void.
 */
void nss_gmac_work(struct work_struct *work)
{
	struct nss_gmac_global_ctx *ctx = NULL;

	nss_gmac_dev *gmacdev = container_of(to_delayed_work(work),
					nss_gmac_dev, gmacwork);
	ctx = gmacdev->ctx;

	gmacdev->nss_state = nss_get_state(gmacdev->nss_gmac_ctx);
	if (gmacdev->nss_state == NSS_STATE_INITIALIZED) {
		nss_gmac_info(gmacdev, "NSS Initialized...");
		nss_tx_phys_if_mac_addr(gmacdev->nss_gmac_ctx,
					(uint8_t *)
					gmacdev->netdev->dev_addr,
					gmacdev->macid);

		nss_tx_phys_if_get_napi_ctx(gmacdev->nss_gmac_ctx, &gmacdev->napi);

		if (test_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags)) {
			if (!IS_ERR_OR_NULL(gmacdev->phydev)) {
				nss_gmac_info(gmacdev, "%s: start phy 0x%x", __FUNCTION__, gmacdev->phydev->phy_id);
				phy_start(gmacdev->phydev);
				phy_start_aneg(gmacdev->phydev);
			} else {
				nss_gmac_info(gmacdev, "%s: Invalid PHY device for a link polled interface", __FUNCTION__);
			}
		} else {
			/*
			 * Force link up if link polling is disabled
			 */
			mutex_lock(&gmacdev->link_mutex);
			nss_gmac_linkup(gmacdev);
			mutex_unlock(&gmacdev->link_mutex);
		}

		return;
	}

	nss_gmac_info(gmacdev, "NSS Un-initialized...");
	queue_delayed_work(ctx->gmac_workqueue, &gmacdev->gmacwork,
			   NSS_GMAC_LINK_CHECK_TIME);
}


/**
 * @brief Function to transmit a given packet on the wire.
 *
 * Whenever Linux Kernel has a packet ready to be transmitted, this function is called.
 * The function prepares a packet and prepares the descriptor and
 * enables/resumes the transmission.
 * @param[in] pointer to sk_buff structure.
 * @param[in] pointer to net_device structure.
 * @return NETDEV_TX_xxx
 */
int32_t nss_gmac_linux_xmit_frames(struct sk_buff *skb, struct net_device *netdev)
{
	nss_tx_status_t nss_status = 0;
	nss_gmac_dev *gmacdev = NULL;

	BUG_ON(skb == NULL);
	if (skb->len < ETH_HLEN) {
		nss_gmac_info(gmacdev, "%s: skb->len < ETH_HLEN",
			      __FUNCTION__);
		goto drop;
	}

	gmacdev = (nss_gmac_dev *)netdev_priv(netdev);
	BUG_ON(gmacdev == NULL);
	BUG_ON(gmacdev->netdev != netdev);

	nss_gmac_info(gmacdev, "%s:Tx packet, len %d, CSUM %d",
		      __FUNCTION__, skb->len, skb->ip_summed);

	nss_status = nss_tx_phys_if_buf(gmacdev->nss_gmac_ctx,
				    (void *)skb, gmacdev->macid);

	if (likely(nss_status == NSS_TX_SUCCESS)) {
		goto tx_done;
	}

drop:
	/*
	 * Now drop it
	 */
	nss_gmac_info(gmacdev, "dropping skb");
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
 *      - Disables interrupts
 *	- Starts Linux network queue interface
 *	- Checks for NSS init completion and determines initial link status
 *      - Starts timer to detect cable plug/unplug
 * @param[in] pointer to net_device structure.
 * @return Returns 0 on success and error status upon failure.
 */
int nss_gmac_linux_open(struct net_device *netdev)
{
	struct device *dev = NULL;
	nss_gmac_dev *gmacdev = (nss_gmac_dev *)netdev_priv(netdev);
	struct nss_gmac_global_ctx *ctx = NULL;

	if (!gmacdev) {
		return -EINVAL;
	}

	dev = &netdev->dev;
	ctx = gmacdev->ctx;

	netif_carrier_off(netdev);


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
	 * capability of checksum offloading
	 */
	netdev->features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM | NETIF_F_TSO | NETIF_F_SG | NETIF_F_UFO | NETIF_F_TSO6;
	netdev->hw_features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM | NETIF_F_TSO | NETIF_F_SG | NETIF_F_UFO | NETIF_F_TSO6;
	netdev->vlan_features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM | NETIF_F_TSO | NETIF_F_SG | NETIF_F_UFO | NETIF_F_TSO6;
	netdev->wanted_features |= NETIF_F_HW_CSUM | NETIF_F_RXCSUM | NETIF_F_TSO | NETIF_F_SG | NETIF_F_UFO | NETIF_F_TSO6;

	/**
	 * Set GMAC state to UP before link state is checked
	 */
	test_and_set_bit(__NSS_GMAC_UP, &gmacdev->flags);
	netif_start_queue(netdev);

	gmacdev->link_state = LINKDOWN;

	queue_delayed_work(ctx->gmac_workqueue, &gmacdev->gmacwork,
			   NSS_GMAC_LINK_CHECK_TIME);

	return 0;
}

/**
 * @brief Function used when the interface is closed.
 *
 * This function is registered to linux stop() function. This function is
 * called whenever ifconfig (in Linux) closes the device (for example
 * "ifconfig eth0 down"). This releases all the system resources allocated
 * during open call.
 *      - Disable the device interrupts
 *	- Send a link change event to NSS GMAC driver.
 *      - Stop the Linux network queue interface
 *      - Cancel timer rgistered for cable plug/unplug tracking
 * @param[in] pointer to net_device structure.
 * @return Returns 0 on success and error status upon failure.
 */
int nss_gmac_linux_close(struct net_device *netdev)
{
	nss_gmac_dev *gmacdev = (nss_gmac_dev *)netdev_priv(netdev);

	if (!gmacdev) {
		return -EINVAL;
	}

	test_and_set_bit(__NSS_GMAC_CLOSING, &gmacdev->flags);

	netif_stop_queue(netdev);
	netif_carrier_off(netdev);

	nss_gmac_rx_disable(gmacdev);
	nss_gmac_tx_disable(gmacdev);

	nss_gmac_disable_interrupt_all(gmacdev);

	nss_tx_phys_if_link_state(gmacdev->nss_gmac_ctx, 0, gmacdev->macid);

	cancel_delayed_work_sync(&gmacdev->gmacwork);

	if(!IS_ERR_OR_NULL(gmacdev->phydev)) {
		phy_stop(gmacdev->phydev);
	}

	test_and_clear_bit(__NSS_GMAC_UP, &gmacdev->flags);
	test_and_clear_bit(__NSS_GMAC_CLOSING, &gmacdev->flags);

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
	nss_gmac_dev *gmacdev = NULL;

	gmacdev = (nss_gmac_dev *)netdev_priv(netdev);
	BUG_ON(gmacdev == NULL);

	if (gmacdev->gmac_power_down == 0) {
		/* If Mac is in powerdown */
		nss_gmac_info(gmacdev,
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
	nss_gmac_dev *gmacdev = NULL;
	nss_tx_status_t nss_status;

	gmacdev = (nss_gmac_dev *)netdev_priv(netdev);
	if (!gmacdev) {
		return -EINVAL;
	}

	if (newmtu > NSS_GMAC_JUMBO_MTU) {
		return -EINVAL;
	}

	nss_status =
	    nss_tx_phys_if_change_mtu(gmacdev->nss_gmac_ctx, newmtu,
				   gmacdev->macid);
	if (nss_status != NSS_TX_SUCCESS) {
		return -EAGAIN;
	}

	if (newmtu <= NSS_ETH_NORMAL_FRAME_MTU) {
		nss_gmac_jumbo_frame_disable(gmacdev);
		nss_gmac_twokpe_frame_disable(gmacdev);
        } else if (newmtu <= NSS_ETH_MINI_JUMBO_FRAME_MTU) {
		nss_gmac_jumbo_frame_disable(gmacdev);
		nss_gmac_twokpe_frame_enable(gmacdev);
        } else if (newmtu <= NSS_ETH_FULL_JUMBO_FRAME_MTU) {
		nss_gmac_jumbo_frame_enable(gmacdev);
        }

	netdev->mtu = newmtu;
	return 0;
}
