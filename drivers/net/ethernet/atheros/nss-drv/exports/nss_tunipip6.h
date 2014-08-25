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
  * nss_tunipip6.h
  * 	NSS TO HLOS interface definitions.
  */

#ifndef __NSS_TUNIPIP6_H
#define __NSS_TUNIPIP6_H

/*
 * DS-Lite (IPv4 in IPv6) tunnel messages
 */

/**
 * DS-Lite request/response types
 */
enum nss_tunipip6_metadata_types {
	NSS_TUNIPIP6_TX_IF_CREATE,
	NSS_TUNIPIP6_TX_IF_DESTROY,
	NSS_TUNIPIP6_RX_STATS_SYNC,
	NSS_TUNIPIP6_MAX,
};

/**
 * DS-Lite configuration message structure
 */
struct nss_tunipip6_create_msg {
	uint32_t saddr[4];	/* Tunnel source address */
	uint32_t daddr[4];	/* Tunnel destination address */
	uint32_t flowlabel;	/* Tunnel ipv6 flowlabel */
	uint32_t flags;		/* Tunnel additional flags */
	uint8_t  hop_limit;	/* Tunnel ipv6 hop limit */
	uint8_t reserved;	/* Place holder */
	uint16_t reserved1;	/* Place holder */
};

/**
 * DS-Lite interface down message structure
 */
struct nss_tunipip6_destroy_msg {
	uint32_t reserved;	/* Place holder */
};

/**
 * DS-Lite statistics sync message structure.
 */
struct nss_tunipip6_stats_sync_msg {
	struct nss_cmn_node_stats node_stats;
};

/**
 * Message structure to send/receive DS-Lite messages
 */
struct nss_tunipip6_msg {
	struct nss_cmn_msg cm;		/* Message Header */
	union {
		struct nss_tunipip6_create_msg tunipip6_create;	/* Message: Create DS-Lite tunnel */
		struct nss_tunipip6_destroy_msg tunipip6_destroy;	/* Message: Destroy DS-Lite tunnel */
		struct nss_tunipip6_stats_sync_msg stats_sync;	/* Message: interface stats sync */
	} msg;
};

/**
 * @brief Callback to receive DS-Lite messages
 *
 * @param app_data Application context of the message
 * @param msg Message data
 *
 * @return void
 */
typedef void (*nss_tunipip6_msg_callback_t)(void *app_data, struct nss_tunipip6_msg *msg);

/**
 * @brief Send DS-Lite  messages
 *
 * @param nss_ctx NSS context
 * @param msg NSS DS-Lite message
 *
 * @return nss_tx_status_t Tx status
 */
extern nss_tx_status_t nss_tunipip6_tx(struct nss_ctx_instance *nss_ctx, struct nss_tunipip6_msg *msg);

/**
 * @brief Callback to receive DS-Lite data
 *
 * @param app_data Application context of the message
 * @param os_buf  Pointer to data buffer
 *
 * @return void
 */
typedef void (*nss_tunipip6_callback_t)(void *app_data, void *os_buf);

/*
 * @brief Register to send/receive DS-Lite messages to NSS
 *
 * @param if_num NSS interface number
 * @param tunipip6_callback Callback for DS-Lite data
 * @param msg_callback Callback for DS-Lite messages
 * @param netdev netdevice associated with the DS-Lite
 *
 * @return nss_ctx_instance* NSS context
 */
extern struct nss_ctx_instance *nss_register_tunipip6_if(uint32_t if_num, nss_tunipip6_callback_t tunipip6_callback,
					nss_tunipip6_msg_callback_t event_callback, struct net_device *netdev);

/**
 * @brief Unregister DS-Lite interface with NSS
 *
 * @param if_num NSS interface number
 *
 * @return void
 */
extern void nss_unregister_tunipip6_if(uint32_t if_num);

#endif /* __NSS_TUN6RD_H */
