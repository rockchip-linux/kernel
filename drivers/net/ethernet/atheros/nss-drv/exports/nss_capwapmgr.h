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
  * nss_capwapmgr.h
  * 	CAPWAP manager for NSS
  */
#ifndef __NSS_CAPWAPMGR_H
#define __NSS_CAPWAPMGR_H

/*
 * Maximum number of tunnels currently supported
 */
#define NSS_CAPWAPMGR_MAX_TUNNELS		32

#define NSS_CAPWAPMGR_TUNNEL_STATE_CONFIGURED	0x1
					/**< Bit is set if tunnel has been configured */
#define NSS_CAPWAPMGR_TUNNEL_STATE_ENABLED	0x2
					/**< Bit is set if tunnel has been enabled */

/*
 * All CAPWAP messages to NSS FW are sync in nature. It means we have
 * to wait for ACK/NACK from NSS FW before proceeding further.
 * Keep a NSS FW response table to wakeup sync message caller.
 */
struct nss_capwapmgr_response {
	atomic_t seq;
	struct semaphore sem;
	wait_queue_head_t wq;
	enum nss_cmn_response response;
	nss_capwap_msg_response_t error;
};

/**
 * Mapping table from tunnel-id to if_num and rule.
 */
struct nss_capwapmgr_tunnel {
	uint32_t if_num;			/**< Interface number of NSS */
	uint32_t tunnel_state;			/**< Tunnel state */
	union {
		struct nss_ipv4_create v4;	/**< IPv4 rule structure */
		struct nss_ipv6_create v6;	/**< IPv6 rule struture */
	} ip_rule;
	struct nss_capwap_rule_msg capwap_rule;	/**< Copy of CAPWAP rule */
};

struct nss_capwapmgr_priv {
	struct nss_ctx_instance *nss_ctx;	/**< Pointer to NSS context */
	struct nss_capwapmgr_tunnel *tunnel;	/**< Pointer to tunnel data */
	uint8_t *if_num_to_tunnel_id;		/**< Mapping table from if_num to tunnel_id. */
	struct nss_capwapmgr_response *resp;	/**< Response housekeeping */
};

/*
 * CAPWAP status enums
 */
typedef enum {
	/*
	 * nss_tx_status_t enums
	 */
	NSS_CAPWAPMGR_SUCCESS = NSS_TX_SUCCESS,
	NSS_CAPWAPMGR_FAILURE = NSS_TX_FAILURE,
	NSS_CAPWAPMGR_FAILURE_QUEUE = NSS_TX_FAILURE_QUEUE,
	NSS_CAPWAPMGR_FAILURE_NOT_READY = NSS_TX_FAILURE_NOT_READY,
	NSS_CAPWAPMGR_FAILURE_TOO_LARGE = NSS_TX_FAILURE_TOO_LARGE,
	NSS_CAPWAPMGR_FAILURE_TOO_SHORT = NSS_TX_FAILURE_TOO_SHORT,
	NSS_CAPWAPMGR_FAILURE_NOT_SUPPORTED = NSS_TX_FAILURE_NOT_SUPPORTED,
	NSS_CAPWAPMGR_FAILURE_BAD_PARAM = NSS_TX_FAILURE_BAD_PARAM,

	/*
	 * CAPWAP specific ones.
	 */
	NSS_CAPWAPMGR_FAILURE_TUNNEL_ENABLED = 100,	/**< Tunnel is enabled */
	NSS_CAPWAPMGR_FAILURE_TUNNEL_DISABLED,		/**< Tunnel is disabled */
	NSS_CAPWAPMGR_FAILURE_TUNNEL_NOT_CFG,		/**< Tunnel is not configured yet */
	NSS_CAPWAPMGR_FAILURE_TUNNEL_EXISTS,		/**< Tunnel already exisits */
	NSS_CAPWAPMGR_FAILURE_DI_ALLOC_FAILED,		/**< Dynamic interface alloc failed */
	NSS_CAPWAPMGR_FAILURE_CAPWAP_RULE,		/**< Failed to create CAPWAP rule */
	NSS_CAPWAPMGR_FAILURE_IP_RULE,			/**< Failed to create IP rule */
	NSS_CAPWAPMGR_FAILURE_REGISTER_NSS,		/**< Failed to register with NSS */
	NSS_CAPWAPMGR_FAILURE_CMD_TIMEOUT,		/**< NSS Driver Command timed-out */
	NSS_CAPWAPMGR_FAILURE_INVALID_REASSEMBLY_TIMEOUT,
	NSS_CAPWAPMGR_FAILURE_INVALID_PATH_MTU,
	NSS_CAPWAPMGR_FAILURE_INVALID_MAX_FRAGMENT,
	NSS_CAPWAPMGR_FAILURE_INVALID_BUFFER_SIZE,
	NSS_CAPWAPMGR_FAILURE_INVALID_L3_PROTO,
	NSS_CAPWAPMGR_FAILURE_INVALID_UDP_PROTO,
	NSS_CAPWAPMGR_FAILURE_INVALID_VERSION,
} nss_capwapmgr_status_t;

/**
 * @brief Creates a CAPWAP netdevice
 *
 * @return Pointer to a newly created netdevice
 *
 * @note First CAPWAP interface name is capwap0 and so on
 */
extern struct net_device *nss_capwapmgr_netdev_create(void);

/**
 * @brief Creates a CAPWAP tunnel
 *
 * @param netdevice
 * @param tunnel_id
 * @param IPv4 rule structure
 * @param CAPWAP rule structure
 *
 * @return nss_capwapmgr_status_t
 */
extern nss_capwapmgr_status_t nss_capwapmgr_ipv4_tunnel_create(struct net_device *dev, uint8_t tunnel_id,
			struct nss_ipv4_create *ip_rule, struct nss_capwap_rule_msg *capwap_rule);

/**
 * @brief Enable a CAPWAP tunnel
 *
 * @param netdevice
 * @param tunnel_id
 *
 * @return nss_capwapmgr_status_t
 */
extern nss_capwapmgr_status_t nss_capwapmgr_enable_tunnel(struct net_device *dev, uint8_t tunnel_id);

/**
 * @brief Enable a CAPWAP tunnel
 *
 * @param netdevice
 * @param tunnel_id
 *
 * @return nss_capwapmgr_status_t
 */
extern nss_capwapmgr_status_t nss_capwapmgr_disable_tunnel(struct net_device *dev, uint8_t tunnel_id);

/**
 * @brief Updates Path MTU of a CAPWAP tunnel
 *
 * @param netdevice
 * @param tunnel_id
 * @param New Path MTU
 *
 * @return nss_capwapmgr_status_t
 */
extern nss_capwapmgr_status_t nss_capwapmgr_update_path_mtu(struct net_device *dev, uint8_t tunnel_id, uint32_t mtu);

/**
 * @brief Changes version of a CAPWAP tunnel
 *
 * @param netdevice
 * @param tunnel_id
 * @param New version
 *
 * @return nss_capwapmgr_status_t
 */
extern nss_capwapmgr_status_t nss_capwapmgr_change_version(struct net_device *dev, uint8_t tunnel_id, uint8_t ver);

/**
 * @brief Destroy a CAPWAP tunnel
 *
 * @param netdevice
 * @param tunnel_id
 *
 * @return nss_capwapmgr_status_t
 *
 * @note CAPWAP tunnel must be disabled before destroy operation.
 */
extern nss_capwapmgr_status_t nss_capwapmgr_tunnel_destroy(struct net_device *dev, uint8_t tunnel_id);

/**
 * @brief Destroy a netdevice
 *
 * @param netdevice
 *
 * @return nss_capwapmgr_status_t
 *
 * @note CAPWAP tunnel must be distroyed first.
 */
extern nss_capwapmgr_status_t nss_capwapmgr_netdev_destroy(struct net_device *netdev);

#if defined(NSS_CAPWAPMGR_ONE_NETDEV)
/**
 * @brief Returns netdevice used by NSS CAPWAP manager
 *
 * @param void
 *
 * @return Pointer to struct net_device
 */
extern struct net_device *nss_capwapmgr_get_netdev(void);
#endif /* NSS_CAPWAPMGR_ONE_NETDEV */
#endif /* __NSS_CAPWAPMGR_H */
