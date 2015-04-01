/*
 **************************************************************************
 * Copyright (c) 2014,2015, The Linux Foundation. All rights reserved.
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
 * Listener
 * Listeners are entities that monitor database events
 */
struct ecm_db_listener_instance;
typedef void (*ecm_db_listener_final_callback_t)(void *arg);		/* Finaliser callback */

/*
 * Interface
 * An interface represents an interface of this device to which nodes may be reached
 */
struct ecm_db_iface_instance;

/*
 * Node
 * A node instance is the ethernet representation of the host, i.e. the mac address to send packets to when reaching the host
 */
struct ecm_db_node_instance;

/*
 * Host
 * A host instance identifies a node by IP address
 */
struct ecm_db_host_instance;

/*
 * Host owner events
 */
typedef void (*ecm_db_host_final_callback_t)(void *arg);		/* Finaliser callback */

/*
 * Host listener events
 */
typedef void (*ecm_db_host_listener_added_callback_t)(void *arg, struct ecm_db_host_instance *hi);		/* Added callback */
typedef void (*ecm_db_host_listener_removed_callback_t)(void *arg, struct ecm_db_host_instance *hi);		/* Removed callback */

/*
 * Mapping
 * A mapping defines a port number.  Non-port based protocols will use -1 so they will all share to the same mapping instance.
 */
struct ecm_db_mapping_instance;

/*
 * Mapping owner events
 */
typedef void (*ecm_db_mapping_final_callback_t)(void *arg);		/* Finaliser callback */

/*
 * Mapping listener events
 */
typedef void (*ecm_db_mapping_listener_added_callback_t)(void *arg, struct ecm_db_mapping_instance *mi);		/* Added callback */
typedef void (*ecm_db_mapping_listener_removed_callback_t)(void *arg, struct ecm_db_mapping_instance *mi);	/* Removed callback */

/*
 * Node
 */
typedef void (*ecm_db_node_final_callback_t)(void *arg);		/* Finaliser callback */
typedef void (*ecm_db_node_listener_added_callback_t)(void *arg, struct ecm_db_node_instance *ni);		/* Added callback */
typedef void (*ecm_db_node_listener_removed_callback_t)(void *arg, struct ecm_db_node_instance *ni);		/* Removed callback */

/*
 * Interface
 */
typedef void (*ecm_db_iface_final_callback_t)(void *arg);		/* Finaliser callback */
typedef void (*ecm_db_iface_listener_added_callback_t)(void *arg, struct ecm_db_iface_instance *ii);		/* Added callback */
typedef void (*ecm_db_iface_listener_removed_callback_t)(void *arg, struct ecm_db_iface_instance *ii);		/* Removed callback */


/*
 * Time out values - in seconds - used to configure timer groups
 */
#define ECM_DB_CLASSIFIER_DETERMINE_GENERIC_TIMEOUT 5
#define ECM_DB_CONNECTION_GENERIC_TIMEOUT 240
#define ECM_DB_CONNECTION_TCP_RST_TIMEOUT 240
#define ECM_DB_CONNECTION_TCP_SHORT_TIMEOUT 240
#define ECM_DB_CONNECTION_TCP_LONG_TIMEOUT 3600
#define ECM_DB_CONNECTION_UDP_TIMEOUT 300
#define ECM_DB_CONNECTION_IGMP_TIMEOUT 240
#define ECM_DB_CONNECTION_ICMP_TIMEOUT 60
#define ECM_DB_CONNECTION_PPTP_DATA_TIMEOUT 300
#define ECM_DB_CONNECTION_RTCP_TIMEOUT 1800
#define ECM_DB_CONNECTION_RTSP_FAST_TIMEOUT 5
#define ECM_DB_CONNECTION_RTSP_SLOW_TIMEOUT 120
#define ECM_DB_CONNECTION_DNS_TIMEOUT 30
#define ECM_DB_CONNECTION_FTP_TIMEOUT 60
#define ECM_DB_CONNECTION_H323_TIMEOUT 28800
#define ECM_DB_CONNECTION_IKE_TIMEOUT 54000
#define ECM_DB_CONNECTION_ESP_TIMEOUT 7800
#define ECM_DB_CONNECTION_ESP_PENDING_TIMEOUT 3
#define ECM_DB_CONNECTION_SDP_TIMEOUT 120
#define ECM_DB_CONNECTION_SIP_TIMEOUT 28800
#define ECM_DB_CONNECTION_BITTORRENT_TIMEOUT 120

/*
 * Timer groups.
 * WARNING: Only connections may use a connection timer group as these are subject to reaping.
 */
enum ecm_db_timer_groups {
	ECM_DB_TIMER_GROUPS_CLASSIFIER_DETERMINE_GENERIC_TIMEOUT = 0,
								/* Generic timeout for a classifier in determine phase */
	ECM_DB_TIMER_GROUPS_CONNECTION_GENERIC_TIMEOUT,		/* Generic timeout for a connection */
	ECM_DB_TIMER_GROUPS_CONNECTION_UDP_GENERIC_TIMEOUT,	/* Standard UDP Timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_UDP_WKP_TIMEOUT,		/* Standard UDP Timeout for all connections where at least one port is < 1024 */
	ECM_DB_TIMER_GROUPS_CONNECTION_ICMP_TIMEOUT,		/* Standard ICMP Timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_TCP_SHORT_TIMEOUT,	/* TCP Short timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_TCP_RESET_TIMEOUT,	/* TCP Reset timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_TCP_LONG_TIMEOUT,	/* TCP Long timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_PPTP_DATA_TIMEOUT,	/* PPTP Tunnel Data timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_RTCP_TIMEOUT,		/* RTCP timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_RTSP_TIMEOUT,		/* RTSP connection timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_RTSP_FAST_TIMEOUT,	/* RTSP fast static NAT connection timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_RTSP_SLOW_TIMEOUT,	/* RTSP slow static NAT connection timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_DNS_TIMEOUT,		/* DNS timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_FTP_TIMEOUT,		/* FTP connection timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_H323_TIMEOUT,		/* H323 timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_IKE_TIMEOUT,		/* IKE timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_ESP_TIMEOUT,		/* ESP timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_ESP_PENDING_TIMEOUT,	/* ESP Pending connection timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_SDP_TIMEOUT,		/* SDP timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_SIP_TIMEOUT,		/* SIP timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_IGMP_TIMEOUT,		/* IGMP timeout */
	ECM_DB_TIMER_GROUPS_CONNECTION_BITTORRENT_TIMEOUT,	/* Bittorrent connections timeout */
	ECM_DB_TIMER_GROUPS_MAX					/* Always the last one */
};
typedef enum ecm_db_timer_groups ecm_db_timer_group_t;
typedef void (*ecm_db_timer_group_entry_callback_t)(void *arg);	/* Timer entry has expired */

/*
 * struct ecm_db_timer_group_entry
 *	Time entry structure used to request timer service
 * WARNING: Do NOT inspect any of these fields - they are for exclusive use of the DB timer code.  Use the API's to control the timer and inspect its state.
 */
struct ecm_db_timer_group_entry {
	struct ecm_db_timer_group_entry *next;			/* Link to the next entry in the timer group chain */
	struct ecm_db_timer_group_entry *prev;			/* Link to the previous entry in the timer group chain */
	uint32_t timeout;					/* Time this entry expires providing the timer group is not the ECM_DB_TIMER_GROUPS_MAX */
	ecm_db_timer_group_t group;				/* The timer group to which this entry belongs, if this is ECM_DB_TIMER_GROUPS_MAX then the timer is not running */
	void *arg;						/* Argument returned in callback */
	ecm_db_timer_group_entry_callback_t fn;			/* Function called when timer expires */
};

/*
 * Connection
 * A connection links two mappings (hence two hosts).  This forms a channel of communication between two hosts.
 */
struct ecm_db_connection_instance;

enum ecm_db_directions {
	ECM_DB_DIRECTION_EGRESS_NAT,			/* LAN->WAN NAT */
	ECM_DB_DIRECTION_INGRESS_NAT,			/* WAN->LAN NAT */
	ECM_DB_DIRECTION_NON_NAT,			/* NET<>NET */
	ECM_DB_DIRECTION_BRIDGED,			/* BRIDGED */
};
typedef enum ecm_db_directions ecm_db_direction_t;

/*
 * Connection listener events
 */
typedef void (*ecm_db_connection_listener_added_callback_t)(void *arg, struct ecm_db_connection_instance *ci);	/* Connection added callback */
typedef void (*ecm_db_connection_listener_removed_callback_t)(void *arg, struct ecm_db_connection_instance *ci);	/* Connection removed callback */

/*
 * Connection creator events
 */
typedef void (*ecm_db_connection_final_callback_t)(void *arg);		/* Finaliser callback */

/*
 * Connection defunct event
 */
typedef void (*ecm_db_connection_defunct_callback_t)(void *arg);	/* Defunct callback */

/*
 * Device Type for IPSec Tunnel devices
 */
#define ECM_ARPHRD_IPSEC_TUNNEL_TYPE 31			/* GGG Should be part of the kernel as a general ARPHRD_XYZ but is not */

/*
 * Interface types
 */
enum ecm_db_iface_types {
	ECM_DB_IFACE_TYPE_ETHERNET = 0,			/* Interface is an ethernet type */
	ECM_DB_IFACE_TYPE_PPPOE,			/* Interface is a PPPoE interface (a specific form of PPP that we recognise in the ECM) */
	ECM_DB_IFACE_TYPE_LAG,				/* Interface is a Link Aggregated interface */
	ECM_DB_IFACE_TYPE_VLAN,				/* Interface is a VLAN interface (802.1Q) */
	ECM_DB_IFACE_TYPE_BRIDGE,			/* Interface is a bridge interface */
	ECM_DB_IFACE_TYPE_LOOPBACK,			/* Interface is a loopback interface */
	ECM_DB_IFACE_TYPE_IPSEC_TUNNEL,			/* Interface is a IPSec tunnel interface */
	ECM_DB_IFACE_TYPE_UNKNOWN,			/* Interface is unknown to the ECM */
	ECM_DB_IFACE_TYPE_SIT,				/* IPv6 in IPv4 tunnel (SIT) interface */
	ECM_DB_IFACE_TYPE_TUNIPIP6,			/* IPIP6 Tunnel (TUNNEL6) interface */
	ECM_DB_IFACE_TYPE_COUNT,			/* Number of interface types */
};
typedef enum ecm_db_iface_types ecm_db_iface_type_t;

/*
 * Interface information as stored in the ECM db for known types of interface
 */
struct ecm_db_interface_info_ethernet {			/* type == ECM_DB_IFACE_TYPE_ETHERNET */
	uint8_t address[ETH_ALEN];			/* MAC Address of this Interface */
};



struct ecm_db_interface_info_bridge {			/* type == ECM_DB_IFACE_TYPE_BRIDGE */
	uint8_t address[ETH_ALEN];			/* MAC Address of this Interface */
};


struct ecm_db_interface_info_unknown {			/* type == ECM_DB_IFACE_TYPE_UNKNOWN */
	uint32_t os_specific_ident;			/* Operating system specific identifier (known only by front end) */
};

struct ecm_db_interface_info_loopback {			/* type == ECM_DB_IFACE_TYPE_LOOPBACK */
	uint32_t os_specific_ident;			/* Operating system specific identifier (known only by front end) */
};



/*
 * Interface Heirarchy
 * Each connection instance keeps four lists of interfaces.
 * These lists record the 'interface heirarchy' in the from or to direction for the connection for both NAT and non-NAT.
 * For example, a connection that is 'from' a vlan might have two interfaces in its 'from' heirarchy:
 * The raw interface, e.g. eth0 and the vlan interface perhaps eth0.1.
 * NOTE: Commonly referred to as "inner to outermost" interfaces.
 * For a connection the ci->mi->hi->ni->ii records the interfaces that the connection 'sees' for the from/to paths.  I.e. Innermost.
 * But heirarchy lists record the path from the ii to the actual outward facing interface - well, as far as is possible to detect.
 * A heirarchy list is recorded in reverse so in the example here it would list eth0 followed by eth0.1.
 * Therefore the first interface in the list is the outermost interface, which is for acceleration, hopefully an NSS supported interface.
 * Lists have a finite size.
 */
#define ECM_DB_IFACE_HEIRARCHY_MAX 10		/* This is the number of interfaces allowed in a heirarchy */
