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
 * nss_ipsec.h
 *	NSS to HLOS IPSec interface definitions.
 */

#ifndef __NSS_IPSEC_H
#define __NSS_IPSEC_H

/*
 * For some reason Linux doesn't define this in if_arp.h,
 * refer http://www.iana.org/assignments/arp-parameters/arp-parameters.xhtml
 * for the full list
 */
#define NSS_IPSEC_ARPHRD_IPSEC 31	/**< iana.org ARP Hardware type for IPsec tunnel*/
#define NSS_IPSEC_MAX_RULES 256 	/**< maximum rules supported */
#define NSS_IPSEC_MAX_SA NSS_CRYPTO_MAX_IDXS /**< maximum SAs supported */

#if (~(NSS_IPSEC_MAX_RULES - 1) & (NSS_IPSEC_MAX_RULES >> 1))
#error "NSS Max SA should be a power of 2"
#endif

#define NSS_IPSEC_MSG_LEN (sizeof(struct nss_ipsec_msg) - sizeof(struct nss_cmn_msg))

/*
 * @brief IPsec rule types
 */
enum nss_ipsec_msg_type {
	NSS_IPSEC_MSG_TYPE_NONE = 0,		/**< nothing to do */
	NSS_IPSEC_MSG_TYPE_ADD_RULE = 1,	/**< add rule to the table */
	NSS_IPSEC_MSG_TYPE_DEL_RULE = 2,	/**< delete rule from the table */
	NSS_IPSEC_MSG_TYPE_SYNC_STATS = 3,	/**< stats sync message */
	NSS_IPSEC_MSG_TYPE_FLUSH_TUN = 4,	/**< delete all SA(s) for a tunnel */
	NSS_IPSEC_MSG_TYPE_MAX
};

/**
 * @brief NSS IPsec status
 */
typedef enum nss_ipsec_status {
	NSS_IPSEC_STATUS_OK = 0,
	NSS_IPSEC_STATUS_ENOMEM = 1,
	NSS_IPSEC_STATUS_ENOENT = 2,
	NSS_IPSEC_STATUS_MAX
} nss_ipsec_status_t;

/*
 * @brief IPsec rule selector tuple for encap & decap
 *
 * @note This is a common selector which is used for preparing
 * a lookup tuple for incoming packets. The tuple is used
 * for deriving the index into the rule table. Choosing the
 * selector fields is dependent upon IPsec encap or
 * decap package itself. Host has zero understanding of
 * these index derivation from the selector fields and
 * hence provides information for all entries in the structure.
 * The packages {Encap or Decap} returns the index into their
 * respective tables to the Host for storing the rule which can
 * be referenced by host in future
 */
struct nss_ipsec_rule_sel {
	uint32_t ipv4_dst;	/**< IPv4 destination to use */
	uint32_t ipv4_src;	/**< IPv4 source to use */
	uint32_t esp_spi;	/**< SPI index */

	uint16_t dst_port;	/**< destination port (UDP or TCP) */
	uint16_t src_port;	/**< source port (UDP or TCP) */

	uint8_t ipv4_proto;	/**< IPv4 protocol types */
	uint8_t res[3];
};

/**
 * @brief IPsec rule outer IP header info to be applied for encapsulation
 */
struct nss_ipsec_rule_oip {
	uint32_t ipv4_dst;		/**< IPv4 destination address to apply */
	uint32_t ipv4_src;		/**< IPv4 source address to apply */
	uint32_t esp_spi;		/**< ESP SPI index to apply */

	uint8_t ipv4_ttl;		/**< IPv4 Time-to-Live value to apply */
	uint8_t res[3];			/**< reserve for 4-byte alignment */
};

/**
 * @brief IPsec rule data to be used for per packet transformation
 */
struct nss_ipsec_rule_data {

	uint16_t crypto_index;		/**< crypto index for the SA */
	uint16_t window_size;		/**< ESP sequence number window */

	uint8_t cipher_algo;		/**< Cipher algorithm */
	uint8_t auth_algo;		/**< Authentication algorithm */
	uint8_t nat_t_req;		/**< NAT-T required */
	uint8_t esp_icv_len;		/**< ESP trailers ICV length to apply */

	uint8_t esp_seq_skip;		/**< Skip ESP sequence number */
	uint8_t esp_tail_skip;		/**< Skip ESP trailer */
	uint8_t use_pattern;		/**< Use random pattern in hash calculation */
	uint8_t res;			/**< Reserve bytes for alignment */
};

/*
 * @brief IPsec rule push message, sent from Host --> NSS for
 * 	  performing a operation on NSS rule tables
 */
struct nss_ipsec_rule {
	struct nss_ipsec_rule_sel sel;		/**< rule selector */
	struct nss_ipsec_rule_oip oip;		/**< per rule outer IP info */
	struct nss_ipsec_rule_data data;	/**< per rule data */

	uint32_t rule_idx;			/**< rule index provided by NSS */
	uint32_t sa_idx;			/**< index into SA table */
};

/**
 * @brief Packet stats for individual SA
 */
struct nss_ipsec_pkt_stats {
	uint32_t processed;			/**< packets processed */
	uint32_t dropped;			/**< packets dropped */
	uint32_t failed;			/**< processing failed */
};

/**
 * @brief NSS IPsec per SA statistics
 */
struct nss_ipsec_sa_stats {
	uint32_t seqnum;			/**< SA sequence number */
	uint32_t sa_idx;			/**< index into SA table */
	struct nss_ipsec_pkt_stats pkts;	/**< packet statistics */
};

/*
 * @brief Message structure to send/receive ipsec messages
 */
struct nss_ipsec_msg {
	struct nss_cmn_msg cm;				/**< Message Header */

	uint32_t tunnel_id;				/**< tunnel index associated with the message */
	union {
		struct nss_ipsec_rule push;		/**< Message: IPsec rule */
		struct nss_ipsec_sa_stats stats;	/**< Message: Retreive stats for tunnel */
	} msg;
};

/**
 * @brief Message notification callback
 *
 * @param app_data[IN] context of the callback user
 * @param msg[IN] notification event data
 *
 * @return
 */
typedef void (*nss_ipsec_msg_callback_t)(void *app_data, struct nss_ipsec_msg *msg);

/**
 * @brief data callback
 *
 * @param app_data[IN] context of the callback user
 * @param skb[IN] data buffer
 *
 * @return
 */
typedef void (*nss_ipsec_buf_callback_t)(struct net_device *netdev, struct sk_buff *skb, struct napi_struct *napi);

/**
 * @brief send an IPsec message
 *
 * @param nss_ctx[IN] NSS HLOS driver's context
 * @param msg[IN] control message
 *
 * @return
 */
extern nss_tx_status_t nss_ipsec_tx_msg(struct nss_ctx_instance *nss_ctx, struct nss_ipsec_msg *msg);

/**
 * @brief send an IPsec process request
 *
 * @param skb Data buffer
 * @param if_num NSS interface number
 *
 * @return Status
 */
extern nss_tx_status_t nss_ipsec_tx_buf(struct sk_buff *skb, uint32_t if_num);

/**
 * @brief register a event callback handler with HLOS driver
 *
 * @param if_num[IN] receive events from this interface (Encap, Decap or C2C)
 * @param cb[IN] event callback function
 * @param app_data[IN] context of the callback user
 *
 * @return
 */
extern struct nss_ctx_instance *nss_ipsec_notify_register(uint32_t if_num, nss_ipsec_msg_callback_t cb, void *app_data);

/**
 * @brief register a data callback handler with HLOS driver
 *
 * @param if_num[IN] receive data from this interface (Encap, Decap or C2C)
 * @param cb[IN] data callback function
 * @param netdev associated netdevice.
 * @param features denote the skb types supported by this interface.
 *
 * @return
 */
extern struct nss_ctx_instance *nss_ipsec_data_register(uint32_t if_num, nss_ipsec_buf_callback_t cb, struct net_device *netdev, uint32_t features);

/**
 * @brief unregister the message notifier
 *
 * @param ctx[IN] HLOS driver's context
 * @param if_num[IN] interface number to unregister from
 *
 * @return
 */
extern void nss_ipsec_notify_unregister(struct nss_ctx_instance *ctx, uint32_t if_num);

/**
 * @brief unregister the data notifier
 *
 * @param ctx[IN] HLOS driver's context
 * @param if_num[IN] interface number to unregister from
 *
 * @return
 */
extern void nss_ipsec_data_unregister(struct nss_ctx_instance *ctx, uint32_t if_num);

/**
 * @brief get the NSS context for the IPsec handle
 *
 * @return nss_ctx_instance
 */
extern struct nss_ctx_instance *nss_ipsec_get_context(void);

/**
 * @brief Initialize ipsec message
 *
 * @return void
 */
extern void nss_ipsec_msg_init(struct nss_ipsec_msg *nim, uint16_t if_num, uint32_t type, uint32_t len,
				nss_ipsec_msg_callback_t cb, void *app_data);

/*
 * @brief get the NSS interface number to be used for IPsec requests
 *
 * @param ctx[IN] HLOS driver's context
 *
 * @return interface number
 */
extern int32_t nss_ipsec_get_interface(struct nss_ctx_instance *ctx);
#endif /* __NSS_IPSEC_H */
