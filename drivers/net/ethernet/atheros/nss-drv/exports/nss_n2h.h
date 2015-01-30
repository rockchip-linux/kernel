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
 * nss_n2h.h
 *	NSS to HLOS interface definitions.
 */

/*
 * TODO: Since this is the now the public header file for writting an
 * IPv4 message, we need to convert the entire file to doxygen.
 */

#ifndef __NSS_N2H_H
#define __NSS_N2H_H

/*
 * Private data structure for configure general configs
 */
struct nss_n2h_cfg_pvt {
	struct semaphore sem;			/* Semaphore structure */
	struct completion complete;		/* completion structure */
	int current_value;			/* valid entry */
	int response;				/* Response from FW */
};

/*
 * Request/Response types
 */
enum nss_n2h_metadata_types {
	NSS_RX_METADATA_TYPE_N2H_STATS_SYNC=0,
	NSS_TX_METADATA_TYPE_N2H_RPS_CFG,
	NSS_TX_METADATA_TYPE_N2H_EMPTY_POOL_BUF_CFG,
	NSS_METADATA_TYPE_N2H_MAX,
};

struct nss_n2h_rps {
	uint32_t enable; /* Enable NSS RPS */
};

struct nss_n2h_empty_pool_buf {
	uint32_t pool_size; /* Empty pool buf size */
};

/*
 * NSS Pbuf mgr stats
 */
struct nss_n2h_pbuf_mgr_stats {
	uint32_t pbuf_alloc_fails;		/* Pbuf ocm alloc fail */
	uint32_t pbuf_free_count;		/* Pbuf ocm free count */
	uint32_t pbuf_total_count;		/* Pbuf ocm total count */
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

	struct nss_n2h_pbuf_mgr_stats pbuf_ocm_stats;
					/* Pbuf OCM Stats */
	struct nss_n2h_pbuf_mgr_stats pbuf_default_stats;
					/* Pbuf Default Stats */

	uint32_t payload_alloc_fails;	/* Number of payload alloc failures */

	uint32_t h2n_ctrl_pkts;		/* Control packets received from HLOS */
	uint32_t h2n_ctrl_bytes;	/* Control bytes received from HLOS */
	uint32_t n2h_ctrl_pkts;		/* Control packets sent to HLOS */
	uint32_t n2h_ctrl_bytes;	/* Control bytes sent to HLOS */

	uint32_t h2n_data_pkts;		/* Data packets received from HLOS */
	uint32_t h2n_data_bytes;	/* Data bytes received from HLOS */
	uint32_t n2h_data_pkts;		/* Data packets sent to HLOS */
	uint32_t n2h_data_bytes;	/* Data bytes sent to HLOS */
};

/*
 * Message structure to send/receive phys i/f commands
 */
struct nss_n2h_msg {
	struct nss_cmn_msg cm;			/* Message Header */
	union {
		struct nss_n2h_stats_sync stats_sync;	/* Message: N2H stats sync */
		struct nss_n2h_rps rps_cfg; 		/* Message: RPS configuration */
		struct nss_n2h_empty_pool_buf empty_pool_buf_cfg;
							/* Message: empty pool buf configuration */
	} msg;
};

/**
 * Callback to be called when IPv4 message is received
 */
typedef void (*nss_n2h_msg_callback_t)(void *app_data, struct nss_n2h_msg *msg);

/*
 * nss_n2h_tx_msg()
 * 	API to send messaged to n2h package.
 */
extern nss_tx_status_t nss_n2h_tx_msg(struct nss_ctx_instance *nss_ctx, struct nss_n2h_msg *nnm);

/*
 * nss_n2h_tx()
 * 	API to enable/disable Host RPS support in NSS
 */
extern nss_tx_status_t nss_n2h_tx(struct nss_ctx_instance *nss_ctx, uint32_t enable_rps);

/*
 * nss_n2h_empty_pool_buf_register_sysctl()
 *	API to register sysctl for empty pool buffer in n2h.
 */
extern void nss_n2h_empty_pool_buf_register_sysctl(void);

/*
 * nss_n2h_empty_pool_buf_unregister_sysctl()
 *	API to unregister sysctl for empty pool buffer in n2h.
 */
extern void nss_n2h_empty_pool_buf_unregister_sysctl(void);

/*
 * nss_n2h_msg_init()
 *	API to initialize the message for N2H package from Host to NSS
 */
typedef void (*nss_n2h_msg_callback_t)(void *app_data, struct nss_n2h_msg *msg);
extern void nss_n2h_msg_init(struct nss_n2h_msg *nim, uint16_t if_num, uint32_t type, uint32_t len,
			nss_n2h_msg_callback_t *cb, void *app_data);

extern struct nss_ctx_instance *nss_ipv4_notify_register(nss_ipv4_msg_callback_t cb, void *app_data);
#endif // __NSS_N2H_H


