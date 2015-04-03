/*
 **************************************************************************
 * Copyright (c) 2014,2015, The Linux Foundation. All rights reserved.
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
  * nss_capwap.h
  * 	NSS TO HLOS interface definitions.
  */

#ifndef __NSS_CAPWAP_H
#define __NSS_CAPWAP_H

#include "nss_capwap_user.h"

/**
 * Headroom required for CAPWAP packets.
 */
#define NSS_CAPWAP_HEADROOM 256

/**
 * CAPWAP message types
 */
typedef enum nss_capwap_msg_type {
	NSS_CAPWAP_MSG_TYPE_NONE = 0,
	NSS_CAPWAP_MSG_TYPE_CFG_RULE = 1,		/**< Create a rule for a tunnel */
	NSS_CAPWAP_MSG_TYPE_UNCFG_RULE = 2,		/**< Remove a rule of a tunnel */
	NSS_CAPWAP_MSG_TYPE_ENABLE_TUNNEL = 3,		/**< Enable a tunnel */
	NSS_CAPWAP_MSG_TYPE_DISABLE_TUNNEL = 4,		/**< Disable a tunnel */
	NSS_CAPWAP_MSG_TYPE_UPDATE_PATH_MTU = 5,	/**< Update Path MTU*/
	NSS_CAPWAP_MSG_TYPE_SYNC_STATS = 6,		/**< stats sync message */
	NSS_CAPWAP_MSG_TYPE_VERSION = 7,		/**< Choose between CAPWAP version. Default is V1 */
	NSS_CAPWAP_MSG_TYPE_MAX,
} nss_capwap_msg_type_t;

/**
 * Error types for CAPWAP response to messages from Host.
 */
typedef enum nss_capwap_msg_response {
	NSS_CAPWAP_ERROR_MSG_INVALID_REASSEMBLY_TIMEOUT,	/**< Invalid reassembly timeout passed */
	NSS_CAPWAP_ERROR_MSG_INVALID_PATH_MTU,			/**< Invalid path mtu passed */
	NSS_CAPWAP_ERROR_MSG_INVALID_MAX_FRAGMENT,		/**< Invalid max-fragment passed */
	NSS_CAPWAP_ERROR_MSG_INVALID_BUFFER_SIZE,		/**< Invalid max-buffer-size passed */
	NSS_CAPWAP_ERROR_MSG_INVALID_L3_PROTO,			/**< Invalid L3 protocol passed */
	NSS_CAPWAP_ERROR_MSG_INVALID_UDP_PROTO,			/**< Invalid UDP protocol passed */
	NSS_CAPWAP_ERROR_MSG_INVALID_VERSION,			/**< Inavlid CAPWAP version */
	NSS_CAPWAP_ERROR_MSG_TUNNEL_DISABLED,			/**< Tunnel is already disabled */
	NSS_CAPWAP_ERROR_MSG_TUNNEL_ENABLED,			/**< Tunnel is already enabled */
	NSS_CAPWAP_ERROR_MSG_TUNNEL_NOT_CFG,			/**< Tunnel is not configured yet */
} nss_capwap_msg_response_t;

/**
 * Per-tunnel stats message from NSS FW
 */
struct nss_capwap_stats_msg {
	struct nss_if_stats pnode_stats;	/**< NSS FW common stats */
	uint32_t rx_dup_frag;			/**< Number of duplicate fragments */
	uint32_t rx_segments;			/**< Number of segments/fragments */
	uint32_t tx_segments;			/**< Number of segments/fragments */
	uint32_t dtls_pkts;			/**< Number of DTLS pkts flowing through */
	uint32_t oversize_drops;		/**< Size of packet > than payload size */
	uint32_t frag_timeout_drops;		/**< Drops due to reassembly timeout */
	uint32_t rx_queue_full_drops;		/**< Drops due to queue full condition in CAPWAP node */
	uint32_t rx_n2h_queue_full_drops;	/**< Drops due to queue full condition in N2H node */
	uint32_t tx_queue_full_drops;		/**< Drops due to queue full condition */
	uint32_t rx_mem_failure_drops;		/**< Drops due to Memory Failure */
	uint32_t tx_mem_failure_drops;		/**< Drops due to Memory Failure */
	uint32_t tx_dropped;			/**< Dropped packets in encap not covered by above */
};

/**
 * CAPWAP version message.
 */
struct nss_capwap_version_msg {
	uint32_t version;
};

/**
 * Path MTU message.
 */
struct nss_capwap_path_mtu_msg {
	uint32_t path_mtu;	/**< New Path MTU */
};

/**
 * The CAPWAP message structure.
 */
struct nss_capwap_msg {
	struct nss_cmn_msg cm;				/**< Message Header */
	union {
		struct nss_capwap_rule_msg rule;	/**< Rule information */
		struct nss_capwap_path_mtu_msg mtu;	/**< New MTU information */
		struct nss_capwap_stats_msg stats;	/**< CAPWAP Statistics */
		struct nss_capwap_version_msg version;	/**< CAPWAP version to use */
	} msg;
};

/**
 * @brief Callback to receive capwap tunnel data
 *
 * @param app_data Application context of the message
 * @param skb  Pointer to data buffer
 *
 * @return void
 */
typedef void (*nss_capwap_buf_callback_t)(struct net_device *netdev, struct sk_buff *skb, struct napi_struct *napi);

/**
 * @brief Callback to receive capwap tunnel messages
 *
 * @param app_data Application context of the message
 * @param msg Message data
 *
 * @return void
 */
typedef void (*nss_capwap_msg_callback_t)(void *app_data, struct nss_capwap_msg *msg);

/**
 * @brief Register to send/receive capwap tunnel messages to NSS
 *
 * @param if_num NSS interface number
 * @param capwap_callback Callback for capwap tunnel data
 * @param msg_callback Callback for capwap tunnel messages
 * @param netdev associated netdevice
 * @param features denote the skb types supported by this interface.
 *
 * @return nss_ctx_instance* NSS context
 */
extern struct nss_ctx_instance *nss_capwap_data_register(uint32_t if_num, nss_capwap_buf_callback_t capwap_callback, struct net_device *netdev, uint32_t features);

/**
 * @brief Send CAPWAP tunnel messages.
 *
 * @param nss_ctx NSS context
 * @param msg NSS CAPWAP tunnel message
 *
 * @return nss_tx_status_t Tx status
 *
 * @note Don't call this function from softirq/interrupt as it
 *	may sleep if NSS FW is busy serving another host thread.
 */
extern nss_tx_status_t nss_capwap_tx_msg(struct nss_ctx_instance *nss_ctx, struct nss_capwap_msg *msg);

/**
 * @brief Send CAPWAP tunnel data buffer to NSS interface number
 *
 * @param nss_ctx NSS context
 * @param skb_buff (or data buffer)
 * @param interface number
 *
 * @return nss_tx_status_t Tx status
 */
extern nss_tx_status_t nss_capwap_tx_data(struct nss_ctx_instance *nss_ctx, struct sk_buff *os_buf, uint32_t if_num);

/**
 * @brief Unregister capwap tunnel interface with NSS
 *
 * @param if_num NSS interface number
 *
 * @return true or false
 */
extern bool nss_capwap_data_unregister(uint32_t if_num);

/**
 * @brief register a event callback handler with HLOS driver.
 *
 * @param if_num - interface number
 * @param cb event callback function
 * @param app_data context of the callback user
 *
 * @return nss context
 *
 * @note This function shouldn't be called from softirq or interrupt.
 */
extern struct nss_ctx_instance *nss_capwap_notify_register(uint32_t if_num, nss_capwap_msg_callback_t cb, void *app_data);

/**
 * @brief unregister the message notifier
 *
 * @param ctx HLOS driver's context
 * @param if_num interface number to unregister from
 *
 * @return
 *
 * @note This function shouldn't be called from softirq or interrupt.
 */
extern nss_tx_status_t nss_capwap_notify_unregister(struct nss_ctx_instance *ctx, uint32_t if_num);

/**
 * @brief Gets NSS context
 *
 * @param None
 *
 * @return Pointer to struct nss_ctx_instance
 */
extern struct nss_ctx_instance *nss_capwap_get_ctx(void);

/**
 * @brief Gets NSS max_buf_size
 *
 * @param NSS context
 *
 * @return ctx->max_buf_size.
 */
extern uint32_t nss_capwap_get_max_buf_size(struct nss_ctx_instance *nss_ctx);

/**
 * @brief Return per-tunnel statistics
 *
 * @param interface numbe
 * @param pointer to struct nss_capwap_tunnel_stats
 *
 * @return true or false
 */
extern bool nss_capwap_get_stats(uint32_t if_num, struct nss_capwap_tunnel_stats *stats);

/**
 * @brief Initialize CAPWAP
 *
 * @return None
 */
extern void nss_capwap_init(void);

/**
 * @brief Initialize capwap msg
 *
 * @return None
 */
extern void nss_capwap_msg_init(struct nss_capwap_msg *ncm, uint16_t if_num, uint32_t type, uint32_t len,
                                nss_capwap_msg_callback_t cb, void *app_data);
#endif /* __NSS_CAPWAP_H */
