
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

 /**
  * nss_pppoe.h
  * 	NSS TO HLOS interface definitions.
  */

#ifndef __NSS_PPPOE_H
#define __NSS_PPPOE_H

/**
 * PPPoE messages
 */

/**
 * PPPoE Request/Response types
 */
enum nss_pppoe_metadata_types {
	NSS_PPPOE_TX_CONN_RULE_DESTROY,
	NSS_PPPOE_RX_CONN_RULE_SUCCESS,
	NSS_PPPOE_RX_CONN_STATS_SYNC,
	NSS_PPPOE_RX_NODE_STATS_SYNC,
	NSS_PPPOE_MAX,
};

/**
 * Exception events from PPPoE handler
 */
enum nss_pppoe_exception_events {
	NSS_PPPOE_EXCEPTION_EVENT_WRONG_VERSION_OR_TYPE,
	NSS_PPPOE_EXCEPTION_EVENT_WRONG_CODE,
	NSS_PPPOE_EXCEPTION_EVENT_HEADER_INCOMPLETE,
	NSS_PPPOE_EXCEPTION_EVENT_UNSUPPORTED_PPP_PROTOCOL,
	NSS_PPPOE_EXCEPTION_EVENT_INTERFACE_MISMATCH,
	NSS_PPPOE_EXCEPTION_EVENT_MAX,
};

/**
 * The NSS PPPoE rule destruction structure.
 */
struct nss_pppoe_rule_destroy_msg {
	uint16_t pppoe_session_id;	/* PPPoE session ID */
	uint16_t pppoe_remote_mac[3];	/* PPPoE server MAC address */
};

/**
 * The NSS PPPoE rule create success structure.
 */
struct nss_pppoe_rule_create_success_msg {
	uint16_t pppoe_session_id;	/* PPPoE session ID on which stats are based */
	uint8_t pppoe_remote_mac[ETH_ALEN];
					/* PPPoE server MAC address */
};

/**
 * The NSS PPPoE node stats structure.
 */
struct nss_pppoe_node_stats_sync_msg {
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

/**
 * The NSS PPPoE exception statistics sync structure.
 */
struct nss_pppoe_conn_stats_sync_msg {
	uint16_t pppoe_session_id;	/* PPPoE session ID on which stats are based */
	uint8_t pppoe_remote_mac[ETH_ALEN];
					/* PPPoE server MAC address */
	uint32_t exception_events_pppoe[NSS_PPPOE_EXCEPTION_EVENT_MAX];
					/* PPPoE exception events */
	uint32_t index;			/* Per interface array index */
	uint32_t interface_num;		/* Interface number on which this session is created */
};

/**
 * Message structure to send/receive PPPoE session commands
 */
struct nss_pppoe_msg {
	struct nss_cmn_msg cm;						/* Message Header */
	union {
		struct nss_pppoe_rule_destroy_msg pppoe_rule_destroy;	/* Message: destroy pppoe rule */
		struct nss_pppoe_rule_create_success_msg pppoe_rule_create_success;
									/* Message: rule status response */
		struct nss_pppoe_conn_stats_sync_msg pppoe_conn_stats_sync;
									/* Message: exception statistics sync */
		struct nss_pppoe_node_stats_sync_msg pppoe_node_stats_sync;
									/* Message: node statistics sync */
	} msg;
};

/**
 * @brief Send PPPoE messages
 *
 * @param nss_ctx NSS context
 * @param msg NSS PPPoE message
 *
 * @return nss_tx_status_t Tx status
 */
extern nss_tx_status_t nss_pppoe_tx(struct nss_ctx_instance *nss_ctx, struct nss_pppoe_msg *msg);

/**
 * @brief PPPoE specific message init
 *	Initialize PPPoE specific message
 *
 * @return
 */
extern void nss_pppoe_msg_init(struct nss_pppoe_msg *npm, uint16_t if_num, uint32_t type, uint32_t len,
				void *cb, void *app_data);

#endif /* __NSS_PPPOE_H */
