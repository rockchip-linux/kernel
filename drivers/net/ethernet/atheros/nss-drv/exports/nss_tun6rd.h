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
  * nss_tun6rd.h
  * 	NSS TO HLOS interface definitions.
  */

#ifndef __NSS_TUN6RD_H
#define __NSS_TUN6RD_H

/*
 * 6RD (IPv6 in IPv4) tunnel messages
 */

/**
 * 6rd tunnel request/response types
 */
enum nss_tun6rd_metadata_types {
	NSS_TUN6RD_ATTACH_PNODE,
	NSS_TUN6RD_RX_STATS_SYNC,
	NSS_TUN6RD_ADD_UPDATE_PEER,
	NSS_TUN6RD_MAX,
};

/**
 * 6rd tunnel configuration message structure
 */
struct nss_tun6rd_attach_tunnel_msg {
	uint32_t prefix[4];		/* 6rd prefix */
	uint32_t relay_prefix;		/* Relay prefix */
	uint16_t prefixlen;		/* 6rd prefix len */
	uint16_t relay_prefixlen;	/* Relay prefix length*/
	uint32_t saddr;		/* Tunnel source address */
	uint32_t daddr;		/* Tunnel destination addresss */
	uint8_t  tos;			/* Tunnel tos field */
	uint8_t  ttl;			/* Tunnel ttl field */
	uint16_t reserved;		/* Reserved field */
};

/**
 * 6rd tunnel statistics sync message structure.
 */
struct nss_tun6rd_sync_stats_msg {
	struct nss_cmn_node_stats node_stats;	/* Node statstics*/
};

/**
 * 6rd tunnel peer addr.
 */
struct nss_tun6rd_set_peer_msg {
	uint32_t ipv6_address[4];	/* The peer's ipv6 addr*/
	uint32_t dest;			/* The peer's ipv4 addr*/
};

/**
 * Message structure to send/receive 6rd tunnel messages
 */
struct nss_tun6rd_msg {
	struct nss_cmn_msg cm;					/* Message Header */
	union {
		struct nss_tun6rd_attach_tunnel_msg tunnel;	/* Message: Attach 6rd tunnel */
		struct nss_tun6rd_sync_stats_msg stats;	/* Message: interface stats sync */
		struct nss_tun6rd_set_peer_msg peer;		/* Message: add/update peer */
	} msg;
};

/**
 * @brief Callback to receive 6rd tunnel messages
 *
 * @param app_data Application context of the message
 * @param msg Message data
 *
 * @return void
 */
typedef void (*nss_tun6rd_msg_callback_t)(void *app_data, struct nss_tun6rd_msg *msg);

/**
 * @brief Send 6rd tunnel messages
 *
 * @param nss_ctx NSS context
 * @param msg NSS 6rd tunnel message
 *
 * @return nss_tx_status_t Tx status
 */
extern nss_tx_status_t nss_tun6rd_tx(struct nss_ctx_instance *nss_ctx, struct nss_tun6rd_msg *msg);

/**
 * @brief Get the tun6rd context used in the nss_tun6rd_tx
 *
 * @return struct nss_ctx_instance *NSS context
 */
extern struct nss_ctx_instance *nss_tun6rd_get_context(void);

/**
 * @brief Callback to receive 6rd tunnel data
 *
 * @param app_data Application context of the message
 * @param skb Pointer to data buffer
 *
 * @return void
 */
typedef void (*nss_tun6rd_callback_t)(struct net_device *netdev, struct sk_buff *skb, struct napi_struct *napi);

/**
 * @brief Register to send/receive 6rd tunnel messages to NSS
 *
 * @param if_num NSS interface number
 * @param tun6rd_callback Callback for 6rd tunnel data
 * @param msg_callback Callback for 6rd tunnel messages
 * @param netdev netdevice associated with the 6rd tunnel
 * @param features denotes the skb types supported by this interface
 *
 * @return nss_ctx_instance* NSS context
 */
extern struct nss_ctx_instance *nss_register_tun6rd_if(uint32_t if_num, nss_tun6rd_callback_t tun6rd_callback,
					nss_tun6rd_msg_callback_t msg_callback, struct net_device *netdev, uint32_t features);

/**
 * @brief Unregister 6rd tunnel interface with NSS
 *
 * @param if_num NSS interface number
 *
 * @return void
 */
extern void nss_unregister_tun6rd_if(uint32_t if_num);

/**
 * @brief Initialize tun6rd msg
 * @param nss_tun6rd_msg
 * @param if_num Interface number
 * @param type Message type
 * @param len Message length
 * @param cb message callback
 * @param app_data
 *
 * @return None
 */
extern void nss_tun6rd_msg_init(struct nss_tun6rd_msg *ncm, uint16_t if_num, uint32_t type,  uint32_t len, void *cb, void *app_data);

#endif /* __NSS_TUN6RD_H */
