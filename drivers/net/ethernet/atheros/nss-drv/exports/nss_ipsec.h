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
 * nss_ipsec.h
 *	NSS to HLOS IPSec interface definitions.
 */

#ifndef __NSS_IPSEC_H
#define __NSS_IPSEC_H

#define NSS_IPSEC_MAX_IV_LEN 16
#define NSS_IPSEC_MAX_RULE_SHIFT 8
#define NSS_IPSEC_MAX_RULE (1 << NSS_IPSEC_MAX_RULE_SHIFT)

/*
 * @brief IPsec rule types
 */
enum nss_ipsec_msg_type {
	NSS_IPSEC_MSG_TYPE_NONE = 0,		/**< nothing to do */
	NSS_IPSEC_MSG_TYPE_ADD_RULE = 1,	/**< add rule to the table */
	NSS_IPSEC_MSG_TYPE_DEL_RULE = 2,	/**< delete rule from the table */
	NSS_IPSEC_MSG_TYPE_DEL_SID = 3,		/**< flush all rules for a crypto_sid */
	NSS_IPSEC_MSG_TYPE_DEL_ALL = 4,		/**< remove all rules from table */
	NSS_IPSEC_MSG_TYPE_SYNC_STATS = 5,	/**< stats sync message */
	NSS_IPSEC_MSG_TYPE_MAX
};

/**
 * @brief IPv4 header
 */
struct nss_ipsec_ipv4_hdr {
        uint8_t ver_ihl;	/**< version and header length */
        uint8_t tos;		/**< type of service */
        uint16_t tot_len;	/**< total length of the payload */
        uint16_t id;		/**< packet sequence number */
        uint16_t frag_off;	/**< fragmentation offset */
        uint8_t ttl;		/**< time to live */
        uint8_t protocol;	/**< next header protocol (TCP, UDP, ESP etc.) */
        uint16_t checksum;	/**< IP checksum */
        uint32_t src_ip;	/**< source IP address */
        uint32_t dst_ip;	/**< destination IP address */
};

/**
 * @brief ESP (Encapsulating Security Payload) header
 */
struct nss_ipsec_esp_hdr {
	uint32_t spi;				/**< security Parameter Index */
	uint32_t seq_no;			/**< esp sequence number */
	uint8_t iv[NSS_IPSEC_MAX_IV_LEN];	/**< iv for esp header */
};

/**
 * @brief TCP (Transmission Control Protocol)  header
 */
struct nss_ipsec_tcp_hdr {
	uint16_t src_port;	/**< source port */
	uint16_t dst_port;	/**< destination port */
	uint32_t seq_no;	/**< tcp sequence number */
	uint32_t ack_no;	/**< acknowledgment number */
	uint16_t flags;		/**< tcp flags */
	uint16_t window_size;	/**< tcp window size */
	uint16_t checksum;	/**< tcp checksum */
	uint16_t urgent;	/**< location where urgent data ends */
};

/**
 * @brief UDP header
 */
struct nss_ipsec_udp_hdr {
	uint16_t src_port;	/**< source port */
	uint16_t dst_port;	/**< destination port */
	uint16_t len;		/**< payload length */
	uint16_t checksum;	/**< udp checksum */
};
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
	uint32_t dst_ip;	/**< destination IP */
	uint32_t src_ip;	/**< source IP */
	uint32_t spi;		/**< SPI index */
	uint16_t dst_port;	/**< destination port (UDP or TCP) */
	uint16_t src_port;	/**< source port (UDP or TCP) */
	uint8_t protocol;	/**< IPv4 protocol types */
	uint8_t res[3];
};

/*
 * @brief IPsec rule data to be used for tunneling purposes
 */
struct nss_ipsec_rule_data {
	struct nss_ipsec_ipv4_hdr ip;	/**< tunnel IPv4 header to use */
	struct nss_ipsec_esp_hdr esp;	/**< tunnel ESP header for IPsec */
	uint32_t crypto_sid;	/**< crypto session ID for encrypt or decrypt operation */
};

/*
 * @brief IPsec rule push message, sent from Host --> NSS for
 * 	  performing a operation on NSS rule tables
 */
struct nss_ipsec_rule_push {
	struct nss_ipsec_rule_sel sel;		/**< rule selector */
	struct nss_ipsec_rule_data data;	/**< rule data */
};

/*
 * @brief IPsec rule sync message, sent from NSS to host for
 * 	  synchronizing the rule tables between NSS & Host
 *
 * @note  the decision to use the index.num or index.map is
 * 	  dependent upon the rule.op
 */
struct nss_ipsec_rule_sync {
	struct nss_ipsec_rule_sel sel;			/**< rule selector for host use */
	struct nss_ipsec_rule_data data;		/**< rule data for host use */
	union {
		uint32_t num;				/**< table index incase of single rule sync(s) */
		uint8_t map[NSS_IPSEC_MAX_RULE];	/**< multiple indexes incase of multiple rule sync(s) */
	} index;
};

/*
 * @brief Message structure to send/receive ipsec messages
 */
struct nss_ipsec_msg {
	struct nss_cmn_msg cm;				/**< Message Header */
	union {
		struct nss_ipsec_rule_push push;	/**< Message: IPsec rule */
		struct nss_ipsec_rule_sync sync;	/**< Message: IPsec events sync */
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
 * @param os_buf[IN] data buffer
 *
 * @return
 */
typedef void (*nss_ipsec_buf_callback_t)(void *app_data, void *os_buf);

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
 * @param app_data[IN] conext of the callback user
 *
 * @return
 */
extern struct nss_ctx_instance *nss_ipsec_data_register(uint32_t if_num, nss_ipsec_buf_callback_t cb, void *app_data);

/**
 * @brief unregister the message notifier
 *
 * @param ctx[IN] HLOS driver's context
 * @param if_num[IN] interface number to unregister from
 *
 * @return
 */
extern void nss_ipsec_msg_unregister(struct nss_ctx_instance *ctx, uint32_t if_num);

/**
 * @brief unregister the data notifier
 *
 * @param ctx[IN] HLOS driver's context
 * @param if_num[IN] interface number to unregister from
 *
 * @return
 */
extern void nss_ipsec_data_unregister(struct nss_ctx_instance *ctx, uint32_t if_num);

#endif /* __NSS_IPSEC_H */
