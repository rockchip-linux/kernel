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
  * nss_gre_redir.h
  * 	NSS TO HLOS interface definitions.
  */

#ifndef __NSS_GRE_REDIR_H
#define __NSS_GRE_REDIR_H

#define NSS_GRE_REDIR_MAX_INTERFACES 3
#define NSS_GRE_REDIR_IP_DF_OVERRIDE_FLAG 0x80
#define NSS_GRE_REDIR_PER_PACKET_METADATA_OFFSET 4
#define NSS_GRE_REDIR_IP_HDR_TYPE_IPV4 1
#define NSS_GRE_REDIR_IP_HDR_TYPE_IPV6 2

/*
 * nss_gre_redir_direction
 *	when pkt goes from host to NSS, host sets dir as NSS_GRE_REDIR_HLOS_TO_NSS
 *	when pkt goes from NSS to host, NSS sets dir as NSS_GRE_REDIR_NSS_TO_HLOS
 *	if host receives any pkt with dir set as NSS_GRE_REDIR_HLOS_TO_NSS, then it is an exception pkt. Handle
 *	it appropriately.
 */
enum nss_gre_redir_direction {
	NSS_GRE_REDIR_HLOS_TO_NSS = 1,
	NSS_GRE_REDIR_NSS_TO_HLOS = 2
};

/**
 * gre_redir messages
 */

/**
 * GRE Redirect request/response types
 */
enum nss_gre_redir_message_types {
	NSS_GRE_REDIR_TX_TUNNEL_CONFIGURE_MSG,	/**< GRE_REDIR Tunnel configuration */
	NSS_GRE_REDIR_TX_INTERFACE_MAP_MSG,	/**< GRE_REDIR tunnel ID to nex NSS if mapping */
	NSS_GRE_REDIR_TX_INTERFACE_UNMAP_MSG,	/**< unmap GRE_REDIR tunnel ID to next if mapping */
	NSS_GRE_REDIR_RX_STATS_SYNC_MSG,	/**< sync GRE_REDIR tunnel stats */
	NSS_GRE_REDIR_MAX_MSG_TYPES,		/**< GRE_REDIR message max type number */
};

/**
 * @brief: GRE tunnel configure message
 */
struct nss_gre_redir_configure_msg {
	uint32_t ip_hdr_type;			/**< IP hdr type (IPv4 or IPv6) */
	uint32_t ip_src_addr[4];		/**< IPv4/IPv6 src addr(lower 4 bytes are applicable for IPv4) */
	uint32_t ip_dest_addr[4];		/**< IPv4/IPv6 dest addr(lower 4 bytes are applicable for IPv4) */
	uint8_t ip_df_policy;			/**< IP hdr default Do Not Fragment policy */
	uint8_t ip_ttl;				/**< IP hdr Time To Live value */
	uint8_t gre_version;			/**< GRE hdr version */
	uint8_t rps_hint;			/**< 0: Use Core 0, 1: Use Core 1 */
};

/**
 * @brief GRE tunnel interface mapping message
 */
struct nss_gre_redir_interface_map_msg {
	uint32_t nss_if_num;		/**< NSS Interface used to forward packets for GRE Tunnel ID */
	uint16_t gre_tunnel_id;		/**< GRE Tunnel ID */
};

/**
 * @brief GRE tunnel interface unmap message
 */
struct nss_gre_redir_interface_unmap_msg {
	uint16_t gre_tunnel_id;		/**< GRE Tunnel ID */
};

/**
 * @brief: GRE tunnel statistics sync message structure.
 */
struct nss_gre_redir_stats_sync_msg {
	struct nss_cmn_node_stats node_stats;	/**< Tunnel stats sync */
	uint32_t tx_dropped;			/**< Tunnel Tx drops */
};

/**
 * @brief: GRE tunnel statistics as seen by HLOS.
 */
struct nss_gre_redir_tunnel_stats {
	int if_num;				/**< Tunnel Interface num */
	bool valid;				/**< Tunnel validity flag */
	struct nss_cmn_node_stats node_stats;	/**< Tunnel stats sync */
	uint32_t tx_dropped;			/**< Tunnel Tx drops */
};

/**
 * @brief Message structure to send/receive GRE tunnel message.
 */
struct nss_gre_redir_msg {
	struct nss_cmn_msg cm;          					/**< Message Header */
	union {
		struct nss_gre_redir_configure_msg configure;			/**< msg: configure tunnel */
		struct nss_gre_redir_interface_map_msg interface_map;		/**< msg: map tunnel id to if_num */
		struct nss_gre_redir_interface_unmap_msg interface_unmap;	/**< msg: unmap interface mapping */
		struct nss_gre_redir_stats_sync_msg stats_sync;			/**< msg: tunnel statistics sync */
	} msg;
};

/**
 * @brief: HLOS -> NSS Per packet metadata information
 */
struct nss_gre_redir_encap_per_pkt_metadata {
	uint8_t dir;		/**< Direction in which packet is forwaded ( HLOS -> NSS) */
	uint8_t gre_flags;	/**< GRE Flags */
	uint8_t gre_prio;	/**< GRE Priority */
	uint8_t gre_seq;	/**< Sequence Number */
	uint16_t gre_tunnel_id;	/**< Tunnel ID */
	uint8_t ip_dscp;	/**< DSCP values */
	uint8_t ip_df_override;	/**< Override default DF policy Set bit 8 if override required for this packet.
					Lower 7 bits provide DF value to be used for this packet. */
};

/**
 * @brief: NSS -> HLOS Per packet metadata information
 */
struct nss_gre_redir_decap_per_pkt_metadata {
	uint8_t dir;		/**< Direction in which packet is forwarded ( NSS -> HLOS) */
	uint8_t gre_flags;	/**< GRE Flags */
	uint8_t gre_prio;	/**< GRE Priority */
	uint8_t gre_seq;	/**< Sequence Number */
	uint16_t gre_tunnel_id;	/**< Tunnel ID */
	uint16_t res;		/**< Reserved to round it off to word boundary */
};

/**
 * @brief Callback to receive gre tunnel data
 *
 * @param app_data Application context of the message
 * @param os_buf Pointer to data buffer
 *
 * @return void
 */
typedef void (*nss_gre_redir_data_callback_t)(struct net_device *netdev, struct sk_buff *skb, struct napi_struct *napi);

/**
 * @brief Callback to receive gre tunnel messages
 *
 * @param app_data Application context of the message
 * @param msg Message data
 *
 * @return void
 */
typedef void (*nss_gre_redir_msg_callback_t)(void *app_data, struct nss_cmn_msg *msg);

/* @brief Register to send/receive gre tunnel messages to NSS
 *
 * @param if_num NSS interface number
 * @param netdev netdevice associated with the gre tunnel
 * @param cb_func_data Callback for gre tunnel data
 * @param cb_func_msg Callback for gre tunnel messages
 * @param features denote the skb types supported by this interface.
 *
 * @return NSS context
 */
extern struct nss_ctx_instance *nss_gre_redir_register_if(uint32_t if_num, struct net_device *dev_ctx,
							nss_gre_redir_data_callback_t cb_func_data,
							nss_gre_redir_msg_callback_t cb_func_msg,
							uint32_t features);

/**
 * @brief Unregister gre tunnel interface with NSS
 *
 * @param if_num NSS interface number
 *
 * @return void
 */
extern void nss_gre_redir_unregister_if(uint32_t if_num);

/**
 * @brief  Send gre_redir Tunnel messages
 *
 * @param nss_ctx NSS context
 * @param msg GRE tunnel message
 *
 * @return Tx status
 */
extern nss_tx_status_t nss_gre_redir_tx_msg(struct nss_ctx_instance *nss_ctx, struct nss_gre_redir_msg *msg);

/**
 * @brief Send gre_redir Tunnel packet
 *
 * @param nss_ctx NSS context
 * @param os_buf OS buffer (e.g. skbuff)
 * @param if_num GRE tunnel i/f number
 *
 * @return Tx status
 */
extern nss_tx_status_t nss_gre_redir_tx_buf(struct nss_ctx_instance *nss_ctx, struct sk_buff *os_buf,
						uint32_t if_num);

/**
 * @brief Get gre_redir tunnel statistics
 *
 * @param index index in tunnel stats array.
 * @param stats tunnel stats structure.
 *
 * @return true or false.
 */
extern bool nss_gre_redir_get_stats(int index, struct nss_gre_redir_tunnel_stats *stats);

#endif /* __NSS_GRE_REDIR_H */
