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

/**
 * NSS Interface Messages
 */

#ifndef __NSS_IF_H
#define __NSS_IF_H

/**
 * NSS interface message numbers
 */
enum nss_if_message_types {
	NSS_IF_OPEN,
	NSS_IF_CLOSE,
	NSS_IF_LINK_STATE_NOTIFY,
	NSS_IF_MTU_CHANGE,
	NSS_IF_MAC_ADDR_SET,
	NSS_IF_STATS,
	NSS_IF_ISHAPER_ASSIGN,
	NSS_IF_BSHAPER_ASSIGN,
	NSS_IF_ISHAPER_UNASSIGN,
	NSS_IF_BSHAPER_UNASSIGN,
	NSS_IF_ISHAPER_CONFIG,
	NSS_IF_BSHAPER_CONFIG,
	NSS_IF_MAX_MSG_TYPES = 9999,
};

/**
 * NSS interface errors
 */
enum nss_if_error_types {
	NSS_IF_ERROR_NO_ISHAPERS,
	NSS_IF_ERROR_NO_BSHAPERS,
	NSS_IF_ERROR_NO_ISHAPER,
	NSS_IF_ERROR_NO_BSHAPER,
	NSS_IF_ERROR_ISHAPER_OLD,
	NSS_IF_ERROR_BSHAPER_OLD,
	NSS_IF_ERROR_ISHAPER_CONFIG_FAILED,
	NSS_IF_ERROR_BSHAPER_CONFIG_FAILED,
	NSS_IF_ERROR_TYPE_UNKNOWN,
	NSS_IF_ERROR_TYPE_EOPEN,
};

/**
 * Interface open command
 */
struct nss_if_open {
	uint32_t tx_desc_ring;		/**< Tx descriptor ring address */
	uint32_t rx_desc_ring;		/**< Rx descriptor ring address */
};

/**
 * Interface close command
 */
struct nss_if_close {
	uint32_t reserved;		/**< Place holder */
};

/**
 * Link state notification to NSS
 */
struct nss_if_link_state_notify {
	uint32_t state;			/**< Link State (UP/DOWN), speed/duplex settings */
};

/**
 * Interface mtu change
 */
struct nss_if_mtu_change {
	uint16_t min_buf_size;		/**< Changed min buf size value */
};

/**
 * Interface statistics.
 */
struct nss_if_stats {
	uint32_t rx_packets;		/**< Number of packets received */
	uint32_t rx_bytes;		/**< Number of bytes received */
	uint32_t rx_dropped;		/**< Number of RX dropped packets */
	uint32_t tx_packets;		/**< Number of packets transmitted */
	uint32_t tx_bytes;		/**< Number of bytes transmitted */
};

/**
 * The NSS MAC address structure.
 */
struct nss_if_mac_address_set {
	uint8_t mac_addr[ETH_ALEN];	/**< MAC address */
};

/**
 * nss_if shaper assign message structure.
 */
struct nss_if_shaper_assign {
	/*
	 * Request
	 */
	uint32_t shaper_id;

	/*
	 * Response
	 */
	uint32_t new_shaper_id;
};

/**
 * nss_if shaper unassign message structure.
 */
struct nss_if_shaper_unassign {
	uint32_t shaper_id;
};

/**
 * nss_if shaper configure message structure.
 */
struct nss_if_shaper_configure {
	struct nss_shaper_configure config;
};

/**
 * Message structure to send/receive phys i/f commands
 */
union nss_if_msgs {
	struct nss_if_link_state_notify link_state_notify;	/**< Message: notify link status */
	struct nss_if_open open;				/**< Message: open interface */
	struct nss_if_close close;				/**< Message: close interface */
	struct nss_if_mtu_change mtu_change;			/**< Message: MTU change notification */
	struct nss_if_mac_address_set mac_address_set;		/**< Message: set MAC address for i/f */
	struct nss_if_stats stats;				/**< Message: statistics sync */
	struct nss_if_shaper_assign shaper_assign;		/**< Message: shaper assign */
	struct nss_if_shaper_unassign shaper_unassign;		/**< Message: shaper unassign */
	struct nss_if_shaper_configure shaper_configure; 	/**< Message: shaper configure */
};

/**
 * Base class message structure for all interface types.
 */
struct nss_if_msg {
	struct nss_cmn_msg cm;					/**< Message Header */
	union nss_if_msgs msg;					/**< Interfaces messages */
};

/**
 * @brief Callback to receive interface messages
 *
 * @param app_data Application context for this message
 * @param msg Interface message
 *
 * @return void
 */
typedef void (*nss_if_msg_callback_t)(void *app_data, struct nss_if_msg *msg);

/**
 * @brief Callback to receive interface data
 *	  TODO: Adjust to pass app_data as unknown to the
 *	  list layer and netdev/sk as known.
 *
 * @param app_data Application context for this message
 * @param os_buf Data buffer
 *
 * @return void
 */
typedef void (*nss_if_rx_callback_t)(void *app_data, void *os_buf);

/**
 * @brief Register to send/receive GMAC packets/messages
 *
 * @param if_num GMAC i/f number
 * @param rx_callback Receive callback for packets
 * @param msg_callback Receive callback for events
 * @param if_ctx Interface context provided in callback
 *		(must be OS network device context pointer e.g.
 *		struct net_device * in Linux)
 *
 * @return NSS context
 */
extern struct nss_ctx_instance *nss_if_register(uint32_t if_num,
					nss_if_rx_callback_t rx_callback,
					nss_if_msg_callback_t msg_callback,
					struct net_device *if_ctx);

/**
 * @brief Send GMAC packet
 *
 * @param nss_ctx NSS context
 * @param os_buf OS buffer (e.g. skbuff)
 * @param if_num GMAC i/f number
 *
 * @return nss_tx_status_t Tx status
 */
extern nss_tx_status_t nss_if_tx_buf(struct nss_ctx_instance *nss_ctx, struct sk_buff *os_buf, uint32_t if_num);

/**
 * @brief Send message to interface
 *
 * @param nss_ctx NSS context
 * @param nim NSS interfce message
 *
 * @return command Tx status
 */
nss_tx_status_t nss_if_tx_msg(struct nss_ctx_instance *nss_ctx, struct nss_if_msg *nim);

/**
 * @brief Callback to receive physical interface messages
 *
 * @param app_data Application context for this message
 * @param msg NSS interface message
 *
 * @return void
 */
typedef void (*nss_if_msg_callback_t)(void *app_data, struct nss_if_msg *msg);

#endif /*  __NSS_IF_H */
