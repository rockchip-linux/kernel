/*
 **************************************************************************
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
  * nss_capwap_user.h
  * 	NSS CAPWAP definitions for kernel and user space
  */

#ifndef __NSS_CAPWAP_USER_H
#define __NSS_CAPWAP_USER_H

/**
 * Maxmimum values for rule configuration parameters
 */
#define NSS_CAPWAP_MAX_MTU			9000
				/**< maximum MTU supported by NSS FW */
#define NSS_CAPWAP_MAX_BUFFER_SIZE		9000
				/**< maximum buffer-size supported by NSS FW */
#define NSS_CAPWAP_MAX_FRAGMENTS		10
				/**< maximum fragments for reassembly */
#define NSS_CAPWAP_MAX_REASSEMBLY_TIMEOUT	(10 * 1000)
				/**< maximum timeout for reassembly - 10 seconds */

/**
 * CAPWAP Rule configure message flags
 */
#define NSS_CAPWAP_TUNNEL_IPV4		2
				/**< IPv4 tunnel */
#define NSS_CAPWAP_TUNNEL_IPV6		3
				/**< IPv6 tunnel */
#define NSS_CAPWAP_TUNNEL_UDP		4
				/**< UDP tunnel */
#define NSS_CAPWAP_TUNNEL_UDPLite	5
				/**< UDPLite tunnel */

/**
 * CAPWAP tunnel create and type flags. These flags are used
 * to determine packet header size during encapsulation.
 */
#define NSS_CAPWAP_RULE_CREATE_VLAN_CONFIGURED	0x1
				/**< VLAN Configured for CAPWAP tunnel */
#define NSS_CAPWAP_RULE_CREATE_PPPOE_CONFIGURED	0x2
				/**< PPPoE configured for CAPWAP tunnel */
#define NSS_CAPWAP_ENCAP_UDPLITE_HDR_CSUM	0x4
				/**< Generate only UDP-Lite header checksum. Otherwise whole UDP-Lite payload */

/**
 * CAPWAP version
 */
#define NSS_CAPWAP_VERSION_V1		0x1
				/**< RFC CAPWAP version */
#define NSS_CAPWAP_VERSION_V2		0x2
				/**< Initial CAPWAP version for a customer */

/**
 * Type of packet. These are mutually exclusive fields.
 */
#define NSS_CAPWAP_PKT_TYPE_UNKNOWN	0x0000
				/**< Don't know the type of CAPWAP packet */
#define NSS_CAPWAP_PKT_TYPE_CONTROL	0x0001
				/** It's a control CAPWAP packet src_port=5247 */
#define NSS_CAPWAP_PKT_TYPE_DATA	0x0002
				/**< It's a data CAPWAP packet src_port=5246 */

/**
 * Addtional fields for identifying what's there in the packet.
 */
#define NSS_CAPWAP_PKT_TYPE_DTLS_ENABLED	0x0004
				/**< It's a DTLS packet. */
#define NSS_CAPWAP_PKT_TYPE_WIRELESS_INFO	0x0008
				/**< W=1, wireless info present */
#define NSS_CAPWAP_PKT_TYPE_802_11		0x0010
				/**< T=1, then set wbid=1 */

/**
 * CAPWAP metaheader per-packet for both encap (TX) and decap (RX).
 */
struct nss_capwap_metaheader {
	uint8_t version;	/**< CAPWAP version */
	uint8_t rid;		/**< Radio ID */
	uint16_t tunnel_id;	/**< Tunnel-ID */
	uint8_t dscp;		/**< DSCP value */
	uint8_t vlan_pcp;	/**< VLAN priority .P marking */
	uint16_t type;		/**< Type of CAPWAP packet & What was there in CAPWAP header */
	uint16_t nwireless;	/**< Number of wireless info sections in CAPWAP header */
	uint16_t magic;		/**< Magic for verification purpose. Use only for debugging */
} __attribute__((packed aligned(4)));

/**
 * IPv4/IPv6 structure
 */
struct nss_capwap_ip {
	union {
		uint32_t ipv4;		/**< IPv4 address */
		uint32_t ipv6[4];	/**< IPv6 address */
	} ip;
};

/**
 * Encap information for CAPWAP tunnel
 */
struct nss_capwap_encap_rule {
	struct  nss_capwap_ip src_ip;	/**< Source IP */
	uint32_t src_port;		/**< Source Port */
	struct nss_capwap_ip dest_ip;	/**< Destination IP */
	uint32_t dest_port;		/**< Destination Port */
	uint32_t path_mtu;		/**< Path MTU */
};

/**
 * Decap information for CAPWAP tunnel
 */
struct nss_capwap_decap_rule {
	uint32_t reassembly_timeout;	/**< In milli-seconds */
	uint32_t max_fragments;		/**< Max number of fragments expected */
	uint32_t max_buffer_size;	/**< Max size of the payload buffer */
};

/**
 * Rule structure for CAPWAP. The same rule structure applies for both encap and decap
 * in a tunnel.
 */
struct nss_capwap_rule_msg {
	struct nss_capwap_encap_rule encap;	/**< Encap portion of the rule */
	struct nss_capwap_decap_rule decap;	/**< Decap portion of the rule */
	uint32_t stats_timer;			/**< Stats interval timer in mill-seconds */
	int8_t rps;				/**< Core to choose for receiving packets. Set to -1 for NSS FW to decide */
	uint8_t type_flags;			/**< VLAN and/or PPPOE configured */
	uint8_t l3_proto;			/**< NSS_CAPWAP_TUNNEL_IPV4 or NSS_CAPWAP_TUNNEL_IPV6 */
	uint8_t which_udp;			/**< NSS_CAPWAP_TUNNEL_UDP or NSS_CAPWAP_TUNNEL_UDPLite */
};

/**
 * 64-bit version of pnode_stats.
 */
struct nss_capwap_pn_stats {
	uint64_t rx_packets;	/**< Number of packets received */
	uint64_t rx_bytes;	/**< Number of bytes received */
	uint64_t rx_dropped;	/**< Number of RX dropped packets */
	uint64_t tx_packets;	/**< Number of packets transmitted */
	uint64_t tx_bytes;	/**< Number of bytes transmitted */
};

/**
 * Per-tunnel statistics seen by HLOS
 */
struct nss_capwap_tunnel_stats {
	struct nss_capwap_pn_stats pnode_stats;	/**< NSS FW common stats */
	uint64_t rx_dup_frag;			/**< Number of duplicate fragments */
	uint64_t rx_segments;			/**< Number of segments/fragments */
	uint64_t tx_segments;			/**< Number of segments/fragments */
	uint64_t dtls_pkts;			/**< Number of DTLS pkts flowing through */
	uint64_t oversize_drops;		/**< Size of packet > than payload size */
	uint64_t frag_timeout_drops;		/**< Drops due to reassembly timeout */
	uint64_t rx_queue_full_drops;		/**< Drops due to queue full condition in CAPWAP node */
	uint64_t rx_n2h_queue_full_drops;	/**< Drops due to queue full condition in N2H node */
	uint64_t tx_queue_full_drops;		/**< Drops due to queue full condition */
	uint64_t rx_mem_failure_drops;		/**< Drops due to Memory Failure */
	uint64_t tx_mem_failure_drops;		/**< Drops due to Memory Failure */
	uint64_t tx_dropped;			/**< Dropped packets in encap not covered by above */
	uint32_t rx_csum_drops;		/**< Dropped RX packets due to checksum mismatch */
};

#endif /* __NSS_CAPWAP_USER_H */
