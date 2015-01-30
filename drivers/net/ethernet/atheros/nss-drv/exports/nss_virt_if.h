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
 * nss_virt_if
 *	Virtual interface message Structure and APIs
 */

#ifndef __NSS_VIRT_IF_H
#define __NSS_VIRT_IF_H

#include "nss_if.h"

/*
 * Virtual IF/Redirect
 */

/**
 * @brief Request/Response types
 */
enum nss_virt_if_msg_types {
	NSS_VIRT_IF_OPEN = NSS_IF_OPEN,
	NSS_VIRT_IF_CLOSE = NSS_IF_CLOSE,
	NSS_VIRT_IF_LINK_STATE_NOTIFY = NSS_IF_LINK_STATE_NOTIFY,
	NSS_VIRT_IF_MTU_CHANGE = NSS_IF_MTU_CHANGE,
	NSS_VIRT_IF_MAC_ADDR_SET = NSS_IF_MAC_ADDR_SET,
	NSS_VIRT_IF_STATS_SYNC = NSS_IF_STATS,
	NSS_VIRT_IF_ISHAPER_ASSIGN = NSS_IF_ISHAPER_ASSIGN,
	NSS_VIRT_IF_BSHAPER_ASSIGN = NSS_IF_BSHAPER_ASSIGN,
	NSS_VIRT_IF_ISHAPER_UNASSIGN = NSS_IF_ISHAPER_UNASSIGN,
	NSS_VIRT_IF_BSHAPER_UNASSIGN = NSS_IF_BSHAPER_UNASSIGN,
	NSS_VIRT_IF_ISHAPER_CONFIG = NSS_IF_ISHAPER_CONFIG,
	NSS_VIRT_IF_BSHAPER_CONFIG = NSS_IF_BSHAPER_CONFIG,
	NSS_VIRT_IF_TX_CREATE_MSG = NSS_IF_MAX_MSG_TYPES + 1,
	NSS_VIRT_IF_TX_DESTROY_MSG,
	NSS_VIRT_IF_MAX_MSG_TYPES,
};

/**
 * The NSS virtual interface creation structure.
 */
struct nss_virt_if_create {
	uint32_t flags;			/**> Interface flags */
	uint8_t mac_addr[ETH_ALEN];	/**> MAC address */
};

/**
 * The NSS virtual interface destruction structure.
 */
struct nss_virt_if_destroy {
	int32_t reserved;		/**> place holder */
};

/**
 * Message structure to send/receive virtual interface commands
 */
struct nss_virt_if_msg {
	struct nss_cmn_msg cm;				/**> Message Header */
	union {
		union nss_if_msgs if_msgs;
		struct nss_virt_if_create create;	/**> Message: create virt if rule */
		struct nss_virt_if_destroy destroy;	/**> Message: destroy virt if rule */
	} msg;
};

typedef void (*nss_virt_if_data_callback_t)(struct net_device *netdev, struct sk_buff *skb, struct napi_struct *napi);
typedef void (*nss_virt_if_msg_callback_t)(void *app_data, struct nss_cmn_msg *msg);

/**
 * @brief Send message to virtual interface
 *
 * @param nss_ctx NSS context (provided during registration)
 * @param nvim Virtual interface message
 *
 * @return command Tx status
 */
extern nss_tx_status_t nss_virt_if_tx_msg(struct nss_ctx_instance *nss_ctx, struct nss_virt_if_msg *nvim);

/**
 * @brief Forward virtual interface packets
 *
 * @param nss_ctx NSS context (provided during registration)
 * @param if_num Interface number (provided during dynamic interface allocation)
 * @param skb HLOS data buffer (sk_buff in Linux)
 *
 * @return command Tx status
 */
extern nss_tx_status_t nss_virt_if_tx_rxbuf(struct nss_ctx_instance *nss_ctx, int32_t if_num, struct sk_buff *skb);


/**
 * @brief Register Virtual Interface with NSS driver
 *
 * @param if_num Interface number (provied during dynamic_interface allocation)
 * @param data_callback Callback handler for virtual data packets
 * @param msg_callback Callback handler for virtual interface messages
 * @param if_ctx netdevice structure of virtual interface
 *
 * @return command Tx status
 */
extern struct nss_ctx_instance *nss_virt_if_register(uint32_t if_num, nss_virt_if_data_callback_t data_callback,
							nss_virt_if_msg_callback_t msg_callback, struct net_device *if_ctx);


/**
 * @brief Unregister virtual interface from NSS driver
 *
 * @param if_num Interface number (provied during dynamic_interface allocation)
 */
void nss_virt_if_unregister(uint32_t if_num);
#endif /* __NSS_VIRT_IF_H */
