/*
 **************************************************************************
 * Copyright (c) 2014 - 2015, The Linux Foundation. All rights reserved.
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
  * nss_sjack.h
  * 	NSS TO HLOS interface definitions.
  */

#ifndef __NSS_SJACK_H
#define __NSS_SJACK_H

/**
 * nss sjack messages
 */

/**
 * @brief: sjack request/response types
 */
enum nss_sjack_msg_types {
	NSS_SJACK_CONFIGURE_MSG,	/**< SJACK configuration msg */
	NSS_SJACK_UNCONFIGURE_MSG,	/**< SJACK configuration msg */
	NSS_SJACK_STATS_SYNC_MSG,	/**< sync SJACK stats */
	NSS_SJACK_MAX_MSG_TYPE
};

/**
 * @brief sjack configuration message
 */
struct nss_sjack_configure_msg {
	uint32_t ingress_if_num;	/**< ingress interface if num corrosponding to the sjack device */
	uint32_t egress_if_num;		/**< egress interface if num corrosponding to the sjack device */
	uint16_t tunnel_id;             /**< sjack tunnel ID */
	uint8_t ip_dscp;                /**< DSCP value */
	uint8_t gre_prio;               /**< GRE priority info */
	uint8_t gre_flags;		/**< GRE flags */
	uint8_t use_ipsec_sa_pattern;	/**< IPsec SA pattern flag */
};

/**
 * @brief sjack uncofigure message
 */
struct nss_sjack_unconfigure_msg {
	uint32_t ingress_if_num;		/**< ingress interface if num corrosponding to the sjack device */
};

/**
 * #brief: sjack statistics sync message
 */
struct nss_sjack_stats_sync_msg {
	struct nss_cmn_node_stats node_stats; /**< Tunnel stats sync */
};

/**
 * @brief Message structure to send/receive sjack messages.
 */
struct nss_sjack_msg {
	struct nss_cmn_msg cm;					/**< Message Header */
	union {
		struct nss_sjack_configure_msg configure;	/**< msg: configure sjack */
		struct nss_sjack_unconfigure_msg unconfigure;	/**< msg: unconfigure sjack */
		struct nss_sjack_stats_sync_msg stats_sync;	/**< msg: sjack stats sync */
	} msg;
};

/**
 * @brief Callback to receive sjack messages
 *
 * @param app_data Application context of the message
 * @param msg Message data
 *
 * @return void
 */
typedef void (*nss_sjack_msg_callback_t)(void *app_data, struct nss_cmn_msg *msg);

/**
 * @brief Reigster to send/receive sjack messages to NSS
 * @param dev net_device structure for sjack
 * @param if_num SJACK interface number
 * @param event_callback callback to receive statistics
 *
 * @return NSS context
 */
extern struct nss_ctx_instance *nss_sjack_register_if(uint32_t if_num, struct net_device *dev, nss_sjack_msg_callback_t event_callback);

/**
 * @brief Unregister sjack interface with NSS
 *
 * @param if_num sjack interface number
 *
 * @return void
 */
extern void nss_sjack_unregister_if(uint32_t if_num);

/**
 * @brief Send sjack messages to NSS
 *
 * @param nss_ctx NSS context
 * @param msg nss_sjack_msg structure
 *
 * @return Tx status
 */
extern nss_tx_status_t nss_sjack_tx_msg(struct nss_ctx_instance *nss_ctx, struct nss_sjack_msg *msg);

#endif /* __NSS_SJACK_H */
