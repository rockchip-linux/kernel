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
 * nss_lag.h
 *	NSS LAG APIs
 */

/**
 * NSS LAG messages
 */
enum nss_lag_metadata_types {
	NSS_TX_METADATA_LAG_STATE_CHANGE = 0,
	NSS_TX_METADATA_LAG_MAX,
};

/**
 * NSS LAG state change events
 */
enum nss_lag_state_change_ev {
	NSS_LAG_RELEASE = 0,
	NSS_LAG_ENSLAVE = 1,
};

/**
 * LAG return values
 */
enum nss_lag_error_types {
	NSS_LAG_ERROR_EINTERFACE = 1,
	NSS_LAG_ERROR_EMSG = 2,
};

/**
 * NSS LAG state change message
 */
struct nss_lag_state_change {
	uint32_t lagid;					/**< LAG group id */
	uint32_t interface;				/**< Physical interface on which state change happened */
	enum nss_lag_state_change_ev event;		/**< State change event */
};

/**
 * Message structure to send/receive Link aggregation messages
 */
struct nss_lag_msg {
	struct nss_cmn_msg cm;				/**< Message Header */
	union {
		struct nss_lag_state_change state;	/**< Message: state change */
	} msg;
};

/**
 * @brief Send LAG message
 *
 * @param nss_ctx NSS context
 * @param msg NSS LAG message
 *
 * @return nss_tx_status_t Tx status
 */
extern nss_tx_status_t nss_lag_tx(struct nss_ctx_instance *nss_ctx, struct nss_lag_msg *msg);

/**
 * @brief Callback to receive LAG data
 *
 * @param app_data Application context for this message
 * @param msg Data buffer
 *
 * @return void
 */
typedef void (*nss_lag_callback_t)(void *app_data, void *os_buf);

/**
 * @brief Callback to receive LAG events
 *
 * @param app_data Application context for this message
 * @param msg NSS LAG message
 *
 * @return void
 */
typedef void (*nss_lag_event_callback_t)(void *app_data, struct nss_lag_msg *msg);

/**
 * @brief Reigster with NSS to send/receive LAG data/messages
 *
 * @param if_num
 * @param lag_cb Callback to receive LAG data
 * @param lag_ev_cb Callback to receive LAG events
 * @param netdev netdevice associated with the LAG interface
 *
 * @return void
 */
extern void nss_register_lag_if(uint32_t if_num,
				nss_lag_callback_t lag_cb,
				nss_lag_event_callback_t lag_ev_cb,
				struct net_device *netdev);

/**
 * @brief Unreigster LAG interface with NSS
 *
 * @param if_num LAG interface number
 *
 * @return void
 */
extern void nss_unregister_lag_if(uint32_t if_num);
