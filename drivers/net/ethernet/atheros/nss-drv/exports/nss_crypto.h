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
 * nss_crypto.h
 *	NSS to HLOS Crypto interface definitions.
 */

#ifndef __NSS_CRYPTO_H
#define __NSS_CRYPTO_H

#define NSS_CRYPTO_MAX_IDXS 16			/**< Max supported sessions */
#define NSS_CRYPTO_MAX_ENGINES 4		/**< Max engines available */
#define NSS_CRYPTO_BAM_PP 4			/**< BAM Pipe Pairs */

/**
 * @brief sync types supported.
 */
enum nss_crypto_msg_type {
	NSS_CRYPTO_MSG_TYPE_NONE = 0,		/**< sync type none */
	NSS_CRYPTO_MSG_TYPE_OPEN_ENG = 1,	/**< open engine sync */
	NSS_CRYPTO_MSG_TYPE_CLOSE_ENG = 2,	/**< close engine sync */
	NSS_CRYPTO_MSG_TYPE_RESET_SESSION = 3,	/**< reset session */
	NSS_CRYPTO_MSG_TYPE_STATS = 4,		/**< stats sync */
	NSS_CRYPTO_MSG_TYPE_MAX
};

/*
 * @brief Crypto Response types
 */
enum nss_crypto_msg_error {
	NSS_CRYPTO_MSG_ERROR_NONE = 0,
	NSS_CRYPTO_MSG_ERROR_INVAL_ENG = 1,	/**< invalid engine id */
	NSS_CRYPTO_MSG_ERROR_UNSUPP_OP = 2,	/**< unsupported operation type */
	NSS_CRYPTO_MSG_ERROR_INVAL_OP = 3,	/**< invalid operation type */
	NSS_CRYPTO_MSG_ERROR_MAX
};

/*
 * @brief crypto session index type
 */
struct nss_crypto_idx {
	uint16_t pp_num;	/**< pipe pair index */
	uint16_t cmd_len;	/**< command block length to program */
	uint32_t cblk_paddr;	/**< phy_addr of the command block */
};

/*
 * @brief Command sent for opening the engine from host, this is called to
 * 	  initialize the Crypto NSS engine specific data structures. Ideally
 * 	  host can send a single probe for all engines but current implementation
 * 	  relies on probe per engine
 */
struct nss_crypto_config_eng {
	uint32_t eng_id;				/**< engine number to open */
	uint32_t bam_pbase;				/**< BAM base addr (physical) */
	uint32_t crypto_pbase;				/**< Crypto base addr (physical) */
	uint32_t desc_paddr[NSS_CRYPTO_BAM_PP];		/**< pipe desc addr (physical) */
	struct nss_crypto_idx idx[NSS_CRYPTO_MAX_IDXS];	/**< allocated session indexes */
};

/*
 * @brief Reset session related state.
 */
struct nss_crypto_config_session {
	uint32_t idx;		/**< session idx on which will be reset */
};

/*
 * @brief crypto statistics
 */
struct nss_crypto_stats {
	uint32_t queued;	/**< number of frames waiting to be processed*/
	uint32_t completed;	/**< number of frames processed */
	uint32_t dropped;	/**< number of frames dropped or not processed */
};

/**
 * @brief stats structure synced to HOST
 */
struct nss_crypto_sync_stats {
	struct nss_crypto_stats eng_stats[NSS_CRYPTO_MAX_ENGINES];	/**< Engine stats */
	struct nss_crypto_stats idx_stats[NSS_CRYPTO_MAX_IDXS];		/**< Session stats */
	struct nss_crypto_stats total;				/**< Total crypto stats */
};

/*
 * @brief Config message.
 */
struct nss_crypto_msg {
	struct nss_cmn_msg cm;					/**< message header */
	union {
		struct nss_crypto_config_eng eng;		/**< open engine msg structure */
		struct nss_crypto_config_session session;	/**< reset stats msg structure */
		struct nss_crypto_sync_stats stats;		/**< statistics sync */
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
typedef void (*nss_crypto_msg_callback_t)(void *app_data, struct nss_crypto_msg *msg);

/**
 * @brief data callback
 *
 * @param app_data[IN] context of the callback user
 * @param buf[IN] crypto data buffer
 *
 * @return
 */
typedef void (*nss_crypto_buf_callback_t)(void *app_data, void *buf, uint32_t paddr, uint16_t len);

/**
 * @brief send an Crypto message
 *
 * @param nss_ctx[IN] NSS HLOS driver's context
 * @param msg[IN] control message
 *
 * @return
 */
extern nss_tx_status_t nss_crypto_tx_msg(struct nss_ctx_instance *nss_ctx, struct nss_crypto_msg *msg);

/**
 * @brief Send a crypto data
 *
 * @param nss_ctx[IN] HLOS driver's context
 * @param buf[IN] buffer pointer (this is a generic/opaque buffer pointer)
 * @param buf_paddr[IN] phyical address of the buffer
 * @param len[IN] length of the buffer data
 *
 * @return
 */
extern nss_tx_status_t nss_crypto_tx_buf(struct nss_ctx_instance *nss_ctx, void *buf, uint32_t buf_paddr, uint16_t len);

/**
 * @brief register a event callback handler with HLOS driver
 *
 * @param cb[IN] event callback function
 * @param app_data[IN] context of the callback user
 *
 * @return
 */
extern struct nss_ctx_instance *nss_crypto_notify_register(nss_crypto_msg_callback_t cb, void *app_data);

/**
 * @brief register a data callback handler with HLOS driver
 *
 * @param cb[IN] data callback function
 * @param app_data[IN] conext of the callback user
 *
 * @return
 */
extern struct nss_ctx_instance *nss_crypto_data_register(nss_crypto_buf_callback_t cb, void *app_data);

/**
 * @brief unregister the message notifier
 *
 * @param ctx[IN] HLOS driver's context
 *
 * @return
 */
extern void nss_crypto_notify_unregister(struct nss_ctx_instance *ctx);

/**
 * @brief unregister the data notifier
 *
 * @param ctx[IN] HLOS driver's context
 *
 * @return
 */
extern void nss_crypto_data_unregister(struct nss_ctx_instance *ctx);

#endif /* __NSS_CRYPTO_H */
