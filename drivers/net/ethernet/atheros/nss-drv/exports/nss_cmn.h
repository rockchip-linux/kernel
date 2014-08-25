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
 * nss_cmn
 *	Common Message Structure and APIs
 */

#ifndef __NSS_CMN_H
#define __NSS_CMN_H

struct nss_ctx_instance;

/**
 * Common enumerations
 */

/**
 * Tx command status
 */
typedef enum {
	NSS_TX_SUCCESS = 0,	/**< Success */
	NSS_TX_FAILURE,		/**< Command failure other than descriptor not available */
	NSS_TX_FAILURE_QUEUE,	/**< Command failure due to descriptor not available */
	NSS_TX_FAILURE_NOT_READY,	/**< Command failure due to NSS state uninitialized */
	NSS_TX_FAILURE_TOO_LARGE,	/**< Command is too large to fit in one message */
	NSS_TX_FAILURE_TOO_SHORT,	/**< Command/Packet is shorter than expected size */
	NSS_TX_FAILURE_NOT_SUPPORTED,	/**< Command/Packet not accepted for forwarding */
	NSS_TX_FAILURE_BAD_PARAM,	/**< Command failure due to bad parameters */
} nss_tx_status_t;

/**
 * NSS state status
 */
typedef enum {
	NSS_STATE_UNINITIALIZED = 0,	/**< NSS state is initailized */
	NSS_STATE_INITIALIZED		/**< NSS state is uninitialized */
} nss_state_t;

/**
 * NSS core id
 */
typedef enum {
	NSS_CORE_0 = 0,
	NSS_CORE_1,
	NSS_CORE_MAX
} nss_core_id_t;

/**
 * Callback register status
 */
typedef enum {
	NSS_CB_REGISTER_SUCCESS = 0,	/**< Callback register successful */
	NSS_CB_REGISTER_FAILED,		/**< Callback register failed */
} nss_cb_register_status_t;

/**
 * Callback unregister status
 */
typedef enum {
	NSS_CB_UNREGISTER_SUCCESS = 0,	/**< Callback unregister successful */
	NSS_CB_UNREGISTER_FAILED,		/**< Callback unregister failed */
} nss_cb_unregister_status_t;

/**
 * Common response structure
 */
enum nss_cmn_response {
	NSS_CMN_RESPONSE_ACK,		/**< Message Acknowledge */
	NSS_CMN_RESPONSE_EVERSION,	/**< Message Version Error */
	NSS_CMN_RESPONSE_EINTERFACE,	/**< Message Interface Error */
	NSS_CMN_RESPONSE_ELENGTH,	/**< Message Length Error */
	NSS_CMN_RESPONSE_EMSG,		/**< Message Error */
	NSS_CMM_RESPONSE_NOTIFY,	/**< Message Independant of Request */
	NSS_CMN_RESPONSE_LAST
};

/**
 * Common structures
 */

/**
 * Common message structure
 */
struct nss_cmn_msg {
	uint16_t version;		/**< Version id for main message format */
	uint16_t interface;		/**< Primary Key for all messages */
	enum nss_cmn_response response;	/**< Primary response */
	uint32_t type;			/**< Decetralized request #, to be used to match response # */
	uint32_t error;			/**< Decentralized specific error message, response == EMSG */
	uint32_t cb;			/**< Place for callback pointer */
	uint32_t app_data;		/**< Place for app data */
	uint32_t len;			/**< What is the length of the message excluding this header */
};

/**
 * Common per node stats structure
 */
struct nss_cmn_node_stats {
	uint32_t rx_packets;		/**< Number of packets received */
	uint32_t rx_bytes;		/**< Number of bytes received */
	uint32_t rx_dropped;		/**< Number of receive drops due to queue full */
	uint32_t tx_packets;		/**< Number of packets transmitted */
	uint32_t tx_bytes;		/**< Number of bytes transmitted */
};

/**
 * @brief Initialize common area of Host to NSS message
 *
 * @param ncm Common message
 * @param if_num Interface number
 * @param type Message type
 * @param len Size of payload
 * @param cb Callback function
 * @param app_data Application context for this message
 *
 * @return none
 */
extern void nss_cmn_msg_init(struct nss_cmn_msg *ncm, uint16_t if_num, uint32_t type,  uint32_t len,
	void *cb, void *app_data);

/**
 * @brief Obtain interface number
 *
 * @param nss_ctx NSS context
 * @param dev OS network device pointer
 *
 * @return int32_t Interface number
 */
extern int32_t nss_cmn_get_interface_number(struct nss_ctx_instance *nss_ctx, struct net_device *dev);

/**
 * @brief Determine if the interface number is a represented as a virtual interface in the NSS
 *
 * @param nss_ctx NSS context
 * @param interface_num The NSS interface number
 *
 * @return bool true if it is a virtual.
 */
extern bool nss_cmn_interface_is_virtual(void *nss_ctx, int32_t interface_num);

/**
 * @brief Obtain interface device pointer
 *
 * @param nss_ctx NSS context
 * @param if_num Interface number
 *
 * @return struct net_device* Interface device pointer
 */
extern struct net_device *nss_cmn_get_interface_dev(struct nss_ctx_instance *nss_ctx, uint32_t if_num);

/**
 * @brief Obtain the NSS state
 *
 * @param nss_ctx NSS context
 *
 * @return nss_state_t NSS state
 */
extern nss_state_t nss_cmn_get_state(struct nss_ctx_instance *nss_ctx);

/**
 * Callback for queue decongestion message
 */
typedef void (*nss_cmn_queue_decongestion_callback_t)(void *app_data);

/**
 * @brief Register for queue decongestion event
 *
 * @param nss_ctx NSS context
 * @param event_callback Event callback
 * @param app_data Callee's application context to be returned in callback
 *
 * @return nss_cb_register_status_t NSS_CB_REGISTER_SUCCESS if registration successful, else NSS_CB_REGISTER_FAILED
 *
 * @note Callback function will be called with spinlock taken
 */
extern nss_cb_register_status_t nss_cmn_register_queue_decongestion(struct nss_ctx_instance *nss_ctx, nss_cmn_queue_decongestion_callback_t event_callback, void *app_data);

/**
 * @brief Unregister for queue decongestion event
 *
 * @param nss_ctx NSS context
 * @param event_callback Event callback
 *
 * @return nss_cb_register_status_t NSS_CB_REGISTER_SUCCESS if registration successful, else NSS_CB_REGISTER_FAILED
 *
 */
extern nss_cb_unregister_status_t nss_cmn_unregister_queue_decongestion(struct nss_ctx_instance *nss_ctx, nss_cmn_queue_decongestion_callback_t event_callback);

#endif /* __NSS_CMN_MSG_H */
