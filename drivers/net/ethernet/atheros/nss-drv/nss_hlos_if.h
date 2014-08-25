/*
 **************************************************************************
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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
 * nss_hlos_if.h
 *	NSS to HLOS interface definitions.
 */

#ifndef __NSS_HLOS_IF_H
#define __NSS_HLOS_IF_H

/*
 * Request/Response types
 */
enum nss_if_metadata_types {
	NSS_TX_METADATA_TYPE_INTERFACE_OPEN,
	NSS_TX_METADATA_TYPE_INTERFACE_CLOSE,
	NSS_TX_METADATA_TYPE_INTERFACE_LINK_STATE_NOTIFY,
	NSS_TX_METADATA_TYPE_INTERFACE_MTU_CHANGE,
	NSS_TX_METADATA_TYPE_INTERFACE_MAC_ADDR_SET,
	NSS_TX_METADATA_TYPE_INTERFACE_MSS_SET,
	NSS_RX_METADATA_TYPE_INTERFACE_STATS_SYNC,
	NSS_METADATA_TYPE_INTERFACE_MAX,
};

/*
 * ETH_RX
*/

/*
 * Request/Response types
 */
enum nss_eth_rx_metadata_types {
	NSS_RX_METADATA_TYPE_ETH_RX_STATS_SYNC,
	NSS_METADATA_TYPE_ETH_RX_MAX,
};

/*
 * Exception events from bridge/route handler
 */
enum exception_events_eth_rx {
	NSS_EXCEPTION_EVENT_ETH_RX_UNKNOWN_L3_PROTOCOL,
	NSS_EXCEPTION_EVENT_ETH_RX_MAX,
};

/*
 * The NSS eth_rx node stats structure.
 */
struct nss_eth_rx_node_sync {
	struct nss_cmn_node_stats node_stats;
				/* Common node stats for ETH_RX */
	uint32_t exception_events[NSS_EXCEPTION_EVENT_ETH_RX_MAX];
				/* Number of ETH_RX exception events */
};

/*
 * Message structure to send/receive eth_rx commands
 */
struct nss_eth_rx_msg {
	struct nss_cmn_msg cm;		/* Message Header */
	union {
		struct nss_eth_rx_node_sync node_sync;	/* Message: node statistics sync */
	} msg;
};

/*
 * PPPoE
 */

/*
 * Request/Response types
 */
enum nss_pppoe_metadata_types {
	NSS_TX_METADATA_TYPE_PPPOE_DESTROY_SESSION,
	NSS_RX_METADATA_TYPE_PPPOE_RULE_STATUS,
	NSS_RX_METADATA_TYPE_PPPOE_CONN_STATS_SYNC,
	NSS_RX_METADATA_TYPE_PPPOE_NODE_STATS_SYNC,
	NSS_METADATA_TYPE_PPPOE_MAX,
};

/*
 * Exception events from bridge/route handler
 */
enum exception_events_pppoe {
	NSS_EXCEPTION_EVENT_PPPOE_WRONG_VERSION_OR_TYPE,
	NSS_EXCEPTION_EVENT_PPPOE_WRONG_CODE,
	NSS_EXCEPTION_EVENT_PPPOE_HEADER_INCOMPLETE,
	NSS_EXCEPTION_EVENT_PPPOE_UNSUPPORTED_PPP_PROTOCOL,
	NSS_EXCEPTION_EVENT_PPPOE_MAX,
};

/*
 * The NSS PPPoE rule destruction structure.
 */
struct nss_pppoe_destroy {
	uint16_t pppoe_session_id;	/* PPPoE session ID */
	uint16_t pppoe_remote_mac[3];	/* PPPoE server MAC address */
};

/*
 * The NSS PPPoE rule create success structure.
 */
struct nss_pppoe_rule_status {
	uint16_t pppoe_session_id;	/* PPPoE session ID on which stats are based */
	uint8_t pppoe_remote_mac[ETH_ALEN];
					/* PPPoE server MAC address */
};

/*
 * The NSS PPPoE node stats structure.
 */
struct nss_pppoe_node_sync {
	struct nss_cmn_node_stats node_stats;
	uint32_t pppoe_session_create_requests;
					/* PPPoE session create requests */
	uint32_t pppoe_session_create_failures;
					/* PPPoE session create failures */
	uint32_t pppoe_session_destroy_requests;
					/* PPPoE session destroy requests */
	uint32_t pppoe_session_destroy_misses;
					/* PPPoE session destroy failures */
};

/*
 * The NSS PPPoE exception statistics sync structure.
 */
struct nss_pppoe_conn_sync {
	uint16_t pppoe_session_id;	/* PPPoE session ID on which stats are based */
	uint8_t pppoe_remote_mac[ETH_ALEN];
					/* PPPoE server MAC address */
	uint32_t exception_events_pppoe[NSS_EXCEPTION_EVENT_PPPOE_MAX];
					/* PPPoE exception events */
	uint32_t index;			/* Per interface array index */
	uint32_t interface_num;		/* Interface number on which this session is created */
};

/*
 * Message structure to send/receive PPPoE session commands
 */
struct nss_pppoe_msg {
	struct nss_cmn_msg cm;		/* Message Header */
	union {
		struct nss_pppoe_destroy destroy;	/* Message: destroy pppoe rule */
		struct nss_pppoe_rule_status rule_status;	/* Message: rule status response */
		struct nss_pppoe_conn_sync conn_sync;	/* Message: exception statistics sync */
		struct nss_pppoe_node_sync node_sync;	/* Message: node statistics sync */
	} msg;
};


/*
 * C2C message structures
 */

/*
 * Request/Response types
 */
enum nss_c2c_metadata_types {
	NSS_TX_METADATA_TYPE_C2C_TX_MAP,
	NSS_METADATA_TYPE_C2C_MAX,
};

/*
 * NSS Tx Map
 */
struct nss_c2c_tx_map {
	uint32_t c2c_start;		/* Peer core C2C Rx queue start address */
	uint32_t c2c_int_addr;		/* Peer core C2C interrupt register address */
};

/*
 * Message structure to send/receive phys i/f commands
 */
struct nss_c2c_msg {
	struct nss_cmn_msg cm;		/* Message Header */
	union {
		struct nss_c2c_tx_map tx_map;
	} msg;
};

/*
 * General statistics messages
 */

/*
 * Request/Response types
 */
enum nss_n2h_metadata_types {
	NSS_RX_METADATA_TYPE_N2H_STATS_SYNC,
	NSS_METADATA_TYPE_N2H_MAX,
};

/*
 * The NSS N2H statistics sync structure.
 */
struct nss_n2h_stats_sync {
	struct nss_cmn_node_stats node_stats;
					/* Common node stats for N2H */
	uint32_t queue_dropped;		/* Number of packets dropped because the PE queue is too full */
	uint32_t total_ticks;		/* Total clock ticks spend inside the PE */
	uint32_t worst_case_ticks;	/* Worst case iteration of the PE in ticks */
	uint32_t iterations;		/* Number of iterations around the PE */
	uint32_t pbuf_alloc_fails;	/* Number of pbuf allocations that have failed */
	uint32_t payload_alloc_fails;
};

/*
 * Message structure to send/receive phys i/f commands
 */
struct nss_n2h_msg {
	struct nss_cmn_msg cm;			/* Message Header */
	union {
		struct nss_n2h_stats_sync stats_sync;	/* Message: N2H stats sync */
	} msg;
};

/*
 * Generic interface messages
 */
enum nss_generic_metadata_types {
	NSS_TX_METADATA_TYPE_GENERIC_IF_PARAMS,
	NSS_METADATA_TYPE_GENERIC_IF_MAX
};

/*
 * Interface params command
 */
struct nss_generic_if_params {
	uint8_t buf[1];		/* Buffer */
};

/*
 * Message structure to send/receive ipsec messages
 */
struct nss_generic_msg {
	struct nss_cmn_msg cm;			/* Message Header */
	union {
		struct nss_generic_if_params rule;	/* Message: generic rule */
	} msg;
};

/*
 * NSS Profiler messages
 */

/*
 * Profiler Tx command
 */
struct nss_profiler_tx {
	uint32_t len;		/* Valid information length */
	uint8_t buf[1];		/* Buffer */
};

/*
 * Profiler sync
 */
struct nss_profiler_sync {
	uint32_t len;		/* Valid information length */
	uint8_t buf[1];		/* Buffer */
};

/*
 * NSS frequency scaling messages
 */
enum nss_freq_stats_metadata_types {
	COREFREQ_METADATA_TYPE_ERROR = 0,
	COREFREQ_METADATA_TYPE_RX_FREQ_CHANGE = 1,
	COREFREQ_METADATA_TYPE_TX_FREQ_ACK = 2,
	COREFREQ_METADATA_TYPE_TX_CORE_STATS = 3,
};

/*
 * The NSS freq start or stop strcture
 */
struct nss_freq_change {
	/* Request */
	uint32_t frequency;
	uint32_t start_or_end;
	uint32_t stats_enable;

	/* Response */
	uint32_t freq_current;
	int32_t ack;
};

/*
 * NSS core stats
 */
struct nss_core_stats {
	uint32_t inst_cnt_total;
};

/*
 * Message structure to send/receive NSS Freq commands
 */
struct nss_corefreq_msg {
	struct nss_cmn_msg cm;			/* Message Header */
	union {
		struct nss_freq_change nfc;	/* Message: freq stats */
		struct nss_core_stats ncs;	/* Message: NSS stats sync */
	} msg;
};

/*
 * Types of TX metadata.
 */
enum nss_tx_metadata_types {
	NSS_TX_METADATA_TYPE_PROFILER_TX,
	NSS_TX_METADATA_TYPE_NSS_FREQ_CHANGE,
	NSS_TX_METADATA_TYPE_SHAPER_CONFIGURE,
};

/*
 * Structure that describes all TX metadata objects.
 */
struct nss_tx_metadata_object {
	enum nss_tx_metadata_types type;	/* Object type */
	union {				/* Sub-message type */
		struct nss_profiler_tx profiler_tx;
		struct nss_freq_change freq_change;
	} sub;
};

/*
 * Structure that describes all TX metadata objects.
 */
struct nss_tx_metadata_object1 {
	enum nss_tx_metadata_types type;	/* Object type */
	union {				/* Sub-message type */
		struct nss_profiler_tx profiler_tx;
		struct nss_freq_change freq_change;
	} sub;
};

/*
 * Types of RX metadata.
 */
enum nss_rx_metadata_types {
	NSS_RX_METADATA_TYPE_PROFILER_SYNC,
	NSS_RX_METADATA_TYPE_SHAPER_RESPONSE,
};

/*
 * Structure that describes all RX metadata objects.
 */
struct nss_rx_metadata_object {
	enum nss_rx_metadata_types type;	/* Object type */
	union {				/* Sub-message type */
		struct nss_profiler_sync profiler_sync;
	} sub;
};

/*
 * H2N Buffer Types
 */
#define H2N_BUFFER_EMPTY			0
#define H2N_BUFFER_PACKET			2
#define H2N_BUFFER_CTRL				4
#define H2N_BUFFER_CRYPTO_REQ			7
#define H2N_BUFFER_NATIVE_WIFI	    8
#define H2N_BUFFER_SHAPER_BOUNCE_INTERFACE	9
#define H2N_BUFFER_SHAPER_BOUNCE_BRIDGE	10
#define H2N_BUFFER_MAX				16

/*
 * H2N Bit Flag Definitions
 */
#define H2N_BIT_FLAG_GEN_IPV4_IP_CHECKSUM	0x0001
#define H2N_BIT_FLAG_GEN_IP_TRANSPORT_CHECKSUM	0x0002
#define H2N_BIT_FLAG_FIRST_SEGMENT		0x0004
#define H2N_BIT_FLAG_LAST_SEGMENT		0x0008

#define H2N_BIT_FLAG_DISCARD			0x0080
#define H2N_BIT_FLAG_SEGMENTATION_ENABLE	0x0100
#define H2N_BIT_FLAG_SEGMENT_TSO		0x0200
#define H2N_BIT_FLAG_SEGMENT_UFO		0x0400
#define H2N_BIT_FLAG_SEGMENT_TSO6		0x0800

#define H2N_BIT_FLAG_VIRTUAL_BUFFER		0x2000

#define H2N_BIT_BUFFER_REUSE			0x8000

/*
 * HLOS to NSS descriptor structure.
 */
struct h2n_descriptor {
	uint32_t opaque;
				/* 32-bit value provided by the HLOS to associate with the buffer. The cookie has no meaning to the NSS */
	uint32_t buffer;
				/* Physical buffer address. This is the address of the start of the usable buffer being provided by the HLOS */
	uint16_t buffer_len;
				/* Length of the buffer (in bytes) */
	uint16_t metadata_off;
				/* Reserved for future use */
	uint16_t payload_len;
				/* Length of the active payload of the buffer (in bytes) */
	uint16_t mss;	/* MSS to be used with TSO/UFO */
	uint16_t payload_offs;
				/* Offset from the start of the buffer to the start of the payload (in bytes) */
	uint16_t interface_num;
				/* Interface number to which the buffer is to be sent (where appropriate) */
	uint8_t buffer_type;
				/* Type of buffer */
	uint8_t reserved3;
				/* Reserved for future use */
	uint16_t bit_flags;
				/* Bit flags associated with the buffer */
	uint32_t qos_tag;
				/* QoS tag information of the buffer (where appropriate) */
	uint32_t reserved4;	/* Reserved for future use */
};

/*
 * N2H Buffer Types
 */
#define N2H_BUFFER_EMPTY			1
#define N2H_BUFFER_PACKET			3
#define N2H_BUFFER_COMMAND_RESP			5
#define N2H_BUFFER_STATUS			6
#define N2H_BUFFER_CRYPTO_RESP			8
#define N2H_BUFFER_PACKET_VIRTUAL		10
#define N2H_BUFFER_SHAPER_BOUNCED_INTERFACE	11
#define N2H_BUFFER_SHAPER_BOUNCED_BRIDGE	12
#define N2H_BUFFER_MAX				16

/*
 * Command Response Types
 */
#define N2H_COMMAND_RESP_OK			0
#define N2H_COMMAND_RESP_BUFFER_TOO_SMALL	1
#define N2H_COMMAND_RESP_BUFFER_NOT_WRITEABLE	2
#define N2H_COMMAND_RESP_UNSUPPORTED_COMMAND	3
#define N2H_COMMAND_RESP_INVALID_PARAMETERS	4
#define N2H_COMMAND_RESP_INACTIVE_SUBSYSTEM	5

/*
 * N2H Bit Flag Definitions
 */
#define N2H_BIT_FLAG_IPV4_IP_CHECKSUM_VALID		0x0001
#define N2H_BIT_FLAG_IP_TRANSPORT_CHECKSUM_VALID	0x0002
#define N2H_BIT_FLAG_FIRST_SEGMENT			0x0004
#define N2H_BIT_FLAG_LAST_SEGMENT			0x0008
#define N2H_BIT_FLAG_VIRTUAL_BUFFER			0x2000

/*
 * NSS to HLOS descriptor structure
 */
struct n2h_descriptor {
	uint32_t opaque;
				/* 32-bit value provided by the HLOS to associate with the buffer. The cookie has no meaning to the NSS */
	uint32_t buffer;
				/* Physical buffer address. This is the address of the start of the usable buffer being provided by the HLOS */
	uint16_t buffer_len;
				/* Length of the buffer (in bytes) */
	uint16_t reserved1;
				/* Reserved for future use */
	uint16_t payload_len;
				/* Length of the active payload of the buffer (in bytes) */
	uint16_t reserved2;
				/* Reserved for future use */
	uint16_t payload_offs;
				/* Offset from the start of the buffer to the start of the payload (in bytes) */
	uint16_t interface_num;
				/* Interface number to which the buffer is to be sent (where appropriate) */
	uint8_t buffer_type;
				/* Type of buffer */
	uint8_t response_type;
				/* Response type if the buffer is a command response */
	uint16_t bit_flags;
				/* Bit flags associated with the buffer */
	uint32_t timestamp_lo;
				/* Low 32 bits of any timestamp associated with the buffer */
	uint32_t timestamp_hi;
				/* High 32 bits of any timestamp associated with the buffer */
};

/*
 * Device Memory Map Definitions
 */
#define DEV_MAGIC		0x4e52522e
#define DEV_INTERFACE_VERSION	1
#define DEV_DESCRIPTORS		256 /* Do we need it here? */

/**
 * H2N descriptor ring
 */
struct h2n_desc_if_instance {
	struct h2n_descriptor *desc;
	uint16_t size;			/* Size in entries of the H2N0 descriptor ring */
	uint16_t int_bit;		/* H2N0 descriptor ring interrupt */
};

/**
 * N2H descriptor ring
 */
struct n2h_desc_if_instance {
	struct n2h_descriptor *desc;
	uint16_t size;			/* Size in entries of the H2N0 descriptor ring */
	uint16_t int_bit;		/* H2N0 descriptor ring interrupt */
};

/**
 * NSS virtual interface map
 */
struct nss_if_mem_map {
	struct h2n_desc_if_instance h2n_desc_if[16];	/* Base address of H2N0 descriptor ring */
	struct n2h_desc_if_instance n2h_desc_if[15];	/* Base address of N2H0 descriptor ring */
	uint32_t magic;				/* Magic value used to identify NSS implementations (must be 0x4e52522e) */
	uint16_t if_version;			/* Interface version number (must be 1 for this version) */
	uint8_t h2n_rings;			/* Number of descriptor rings in the H2N direction */
	uint8_t n2h_rings;			/* Number of descriptor rings in the N2H direction */
	uint32_t h2n_nss_index[16];
			/* Index number for the next descriptor that will be read by the NSS in the H2N0 descriptor ring (NSS owned) */
	volatile uint32_t n2h_nss_index[15];
			/* Index number for the next descriptor that will be written by the NSS in the N2H0 descriptor ring (NSS owned) */
	uint8_t num_phys_ports;
	uint8_t reserved1[3];	/* Reserved for future use */
	uint32_t h2n_hlos_index[16];
			/* Index number for the next descriptor that will be written by the HLOS in the H2N0 descriptor ring (HLOS owned) */
	volatile uint32_t n2h_hlos_index[15];
			/* Index number for the next descriptor that will be read by the HLOS in the N2H0 descriptor ring (HLOS owned) */
	uint32_t c2c_start;	/* Reserved for future use */
};
#endif /* __NSS_HLOS_IF_H */
