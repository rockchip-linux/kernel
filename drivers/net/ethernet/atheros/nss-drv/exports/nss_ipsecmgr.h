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

/* * nss_ipsecmgr.h
 *	NSS to HLOS IPSec Manager interface definitions.
 */
#ifndef __NSS_IPSECMGR_H
#define __NSS_IPSECMGR_H

#define NSS_IPSECMGR_DEBUG_LVL_ERROR 1
#define NSS_IPSECMGR_DEBUG_LVL_WARN 2
#define NSS_IPSECMGR_DEBUG_LVL_INFO 3
#define NSS_IPSECMGR_DEBUG_LVL_TRACE 4

#define NSS_IPSECMGR_TUN_NAME "ipsectun%d"
#define NSS_IPSECMGR_MAX_TUNNELS (NSS_CRYPTO_MAX_IDXS/2)

/*
 * This is a rough estimate need to be accurate but large enough to
 * accomodate most usecases
 */
#define NSS_IPSECMGR_TUN_MAX_HDR_LEN 96

/*
 * Space required in the head and tail of the buffer
 */
#define NSS_IPSECMGR_TUN_HEADROOM 128
#define NSS_IPSECMGR_TUN_TAILROOM 128

#define NSS_IPSECMGR_TUN_MTU(x) (x - NSS_IPSECMGR_TUN_MAX_HDR_LEN)

#define NSS_IPSECMGR_NATT_PORT_DATA 4500

/**
 * @brief Definition of an IPsec encapsulation rule for an add operation
 */
struct nss_ipsecmgr_encap_add {
	uint32_t inner_ipv4_src;	/**< inner IPv4 source address */
	uint32_t inner_ipv4_dst;	/**< inner IPv4 destination address */

	uint32_t outer_ipv4_src;	/**< outer IPv4 source address */
	uint32_t outer_ipv4_dst;	/**< outer IPv4 destination address */

	uint32_t esp_spi;		/**< ESP header's SPI index */

	uint16_t inner_src_port;	/**< inner protocol's source port */
	uint16_t inner_dst_port;	/**< inner protocol's destination port */

	uint16_t crypto_index;		/**< crypto session index returned by the driver */
	uint8_t cipher_algo;		/**< Cipher algorithm */
	uint8_t auth_algo;		/**< Authentication algorithm */

	uint8_t nat_t_req;		/**< apply NAT-T header */
	uint8_t inner_ipv4_proto;	/**< inner IPv4 protocol */
	uint8_t outer_ipv4_ttl;		/**< outer IPv4 time to live */
	uint8_t esp_icv_len;		/**< ESP trailer's ICV length */

	uint8_t esp_seq_skip;		/**< Skip ESP sequence number in header*/
	uint8_t esp_tail_skip;		/**< Skip ESP trailer*/
	uint8_t use_pattern;		/**< Use random pattern in hash calculation */
	uint8_t res;			/**< reserve for 4-byte alignment */
};

/**
 * @brief Definition of an IPsec encapsulation rule for a delete operation
 */
struct nss_ipsecmgr_encap_del {
	uint32_t inner_ipv4_src;	/**< inner IPv4 source address */
	uint32_t inner_ipv4_dst;	/**< inner IPv4 destination address */

	uint16_t inner_src_port;	/**< inner protocol's source port */
	uint16_t inner_dst_port;	/**< inner protocol's destination port */

	uint8_t inner_ipv4_proto;	/**< inner IPv4 protocol */
	uint8_t res[3];			/**< reserve for 4-byte alignment */
};

/**
 * @brief Definition of an IPsec decapsulation rule for an add operation
 */
struct nss_ipsecmgr_decap_add {
	uint32_t outer_ipv4_src;	/**< outer IPv4 source address */
	uint32_t outer_ipv4_dst;	/**< outer IPv4 destination address */

	uint32_t esp_spi;		/**< ESP header's SPI index */

	uint16_t crypto_index;		/**< crypto session index returned by the driver */
	uint16_t window_size;		/**< sequence number window size for anti-replay */

	uint8_t cipher_algo;		/**< Cipher algorithm */
	uint8_t auth_algo;		/**< Authentication algorithm */
	uint8_t esp_icv_len;		/**< ESP trailer's ICV length */
	uint8_t nat_t_req;		/**< Remove NAT-T header */

	uint8_t esp_seq_skip;		/**< Skip ESP sequence number in header*/
	uint8_t esp_tail_skip;		/**< Skip ESP trailer*/
	uint8_t res[2];			/**< reserve for 4-byte alignment */
};

/**
 * @brief Definition of an IPsec decapsulation rule for a delete operation
 */
struct nss_ipsecmgr_decap_del {
	uint32_t outer_ipv4_src;	/**< outer IPv4 source address */
	uint32_t outer_ipv4_dst;	/**< outer IPv4 destination address */

	uint32_t esp_spi;		/**< ESP header's SPI index */
};

/**
 * @brief Rule types
 */
enum nss_ipsecmgr_rule_type {
	NSS_IPSECMGR_RULE_TYPE_NONE = 0,	/**< Invalid rule type */
	NSS_IPSECMGR_RULE_TYPE_ENCAP = 1,	/**< rule is for encap */
	NSS_IPSECMGR_RULE_TYPE_DECAP = 2,	/**< rule is for decap */
	NSS_IPSECMGR_RULE_TYPE_MAX
};

/**
 * @brief NSS IPsec manager rule definition
 */
union nss_ipsecmgr_rule {
	struct nss_ipsecmgr_encap_add encap_add;	/**< encap rule add */
	struct nss_ipsecmgr_encap_del encap_del;	/**< encap rule del */
	struct nss_ipsecmgr_decap_add decap_add;	/**< decap rule add */
	struct nss_ipsecmgr_decap_del decap_del;	/**< decap rule del */
};

/**
 * @brief SA stats exported by NSS IPsec manager
 */
struct nss_ipsecmgr_sa_stats {
	enum nss_ipsecmgr_rule_type type;		/**< Encap/Decap */
	uint32_t esp_spi;				/**< ESP SPI */
	uint32_t seqnum;				/**< SA sequence number */
	uint32_t crypto_index;				/**< crypto session index */
	uint32_t pkts_processed;			/**< packets processed */
	uint32_t pkts_dropped;				/**< packets dropped */
	uint32_t pkts_failed;				/**< packets failed to be processed */
};

/**
 * @brief NSS IPsec manager event type
 */
enum nss_ipsecmgr_event_type {
	NSS_IPSECMGR_EVENT_NONE = 0,			/**< invalid event type */
	NSS_IPSECMGR_EVENT_SA_STATS,			/**< statistics sync */
	NSS_IPSECMGR_EVENT_MAX
};

/**
 * @brief NSS IPsec manager event
 */
struct nss_ipsecmgr_event {
	enum nss_ipsecmgr_event_type type;		/**< Event type */
	union {
		struct nss_ipsecmgr_sa_stats stats;	/**< Event: SA statistics */
	}data;
};

/**
 * @brief Callback function registered by the IPsec tunnel users
 *
 * @param ctx[IN] callback context associated with the tunnel
 * @param skb[IN] the packet
 *
 * @return
 */
typedef void (*nss_ipsecmgr_data_cb_t) (void *ctx, struct sk_buff *skb);

/**
 * @brief Callback function registered by the IPsec tunnel users
 * 	  to receive NSS IPsec manager events
 *
 * @param ctx[IN] callback context associated with the tunnel
 * @param ev[IN] IPsec event
 *
 * @return
 */
typedef void (*nss_ipsecmgr_event_cb_t) (void *ctx, struct nss_ipsecmgr_event *ev);

/**
 * @brief Create a new IPsec tunnel interface
 *
 * @param ctx[IN] context that the caller wants to be stored per tunnel
 * @param cb[IN] the callback function for receiving data
 * @param event_cb[IN] the callback function for receiving events
 *
 * @return Netdevice for the IPsec tunnel interface
 *
 * @note This needs to be created for receiving data from NSS IPsec
 * 	 and sending data to the NSS IPsec (if requried). The need for
 * 	 this is to provide a data interface on Host which can use it
 * 	 to either receive IPsec decapsulated packets or send plain text
 * 	 packets to get IPsec encapsulated. This will help bind SA(s) to
 * 	 tunnels so when the tunnel goes away all associated SA(s)
 */
struct net_device *nss_ipsecmgr_tunnel_add(void *ctx, nss_ipsecmgr_data_cb_t data_cb, nss_ipsecmgr_event_cb_t event_cb);

/**
 * @brief Delete the IPsec tunnel
 *
 * @param tun[IN] IPsec tunnel device on host
 *
 * @return true when successful
 */
bool nss_ipsecmgr_tunnel_del(struct net_device *tun);

/**
 * @brief Add a new Security Association to the IPsec tunnel
 *
 * @param tun[IN] pseudo IPsec tunnel device
 * @param rule[IN] IPsec rule structure associated with the SA
 * @param type[IN] ingress or egress type
 *
 * @return
 */
bool nss_ipsecmgr_sa_add(struct net_device *tun, union nss_ipsecmgr_rule *rule, enum nss_ipsecmgr_rule_type type);

/**
 * @brief Delete an existing security association from the IPsec tunnel
 *
 * @param tun[IN] pseudo IPsec tunnel device
 * @param rule[IN] IPsec rule structure associated with the SA
 * @param type[IN] ingress or egress type
 *
 * @return
 */
bool nss_ipsecmgr_sa_del(struct net_device *tun, union nss_ipsecmgr_rule *rule, enum nss_ipsecmgr_rule_type type);

#endif /* __NSS_IPSECMGR_H */
