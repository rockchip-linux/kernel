/*
 **************************************************************************
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
 * nss_connmgr_ipv4.c
 *
 * This file is the NSS connection manager for managing IPv4 connections.It
 * forms an interface between the fast-path NSS driver and Linux
 * connection track module for updating/syncing connection level information
 * between the two.It is responsible for maintaing  all connection (flow) level
 * information and statistics for all fast path connections.
 * ------------------------REVISION HISTORY-----------------------------
 * Qualcomm Atheros         01/Mar/2013              Created
 */

#include <linux/types.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/icmp.h>
#include <linux/sysctl.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/pkt_sched.h>
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/netdevice.h>
#include <linux/notifier.h>

#include <net/route.h>
#include <net/ip.h>
#include <net/tcp.h>
#include <asm/unaligned.h>
#include <asm/uaccess.h> /* for put_user */

#include <linux/netfilter_ipv4.h>
#include <linux/netfilter_bridge.h>
#include <linux/if_bridge.h>
#include <linux/if_bonding.h>
#include <net/netfilter/nf_conntrack.h>
#include <net/netfilter/nf_conntrack_acct.h>
#include <net/netfilter/nf_conntrack_helper.h>
#include <net/netfilter/nf_conntrack_l4proto.h>
#include <net/netfilter/nf_conntrack_l3proto.h>
#include <net/netfilter/nf_conntrack_zones.h>
#include <net/netfilter/nf_conntrack_core.h>
#include <net/netfilter/ipv4/nf_conntrack_ipv4.h>
#include <net/netfilter/ipv4/nf_defrag_ipv4.h>

#include <net/arp.h>
#include <net/neighbour.h>

#include <nss_api_if.h>
#include <linux/../../net/8021q/vlan.h>
#include <linux/if_vlan.h>
#include <linux/../../net/offload/offload.h>

/*
 * Debug output levels
 * 0 = OFF
 * 1 = ASSERTS / ERRORS
 * 2 = 1 + WARN
 * 3 = 2 + INFO
 * 4 = 3 + TRACE
 */

#if (NSS_CONNMGR_DEBUG_LEVEL < 1)
#define NSS_CONNMGR_DEBUG_ASSERT(s, ...)
#define NSS_CONNMGR_DEBUG_ERROR(s, ...)
#define NSS_CONNMGR_DEBUG_CHECK_MAGIC(i, m, s, ...)
#define NSS_CONNMGR_DEBUG_SET_MAGIC(i, m)
#define NSS_CONNMGR_DEBUG_CLEAR_MAGIC(i)
#else
#define NSS_CONNMGR_DEBUG_ASSERT(c, s, ...) if (!(c)) { printk("%s:%d:ASSERT:", __FILE__, __LINE__);printk(s, ##__VA_ARGS__); while(1); }
#define NSS_CONNMGR_DEBUG_ERROR(s, ...) printk("%s:%d:ERROR:", __FILE__, __LINE__);printk(s, ##__VA_ARGS__)
#define NSS_CONNMGR_DEBUG_CHECK_MAGIC(i, m, s, ...) if (i->magic != m) { NSS_CONNMGR_DEBUG_ASSERT(FALSE, s, ##__VA_ARGS__); }
#define NSS_CONNMGR_DEBUG_SET_MAGIC(i, m) i->magic = m
#define NSS_CONNMGR_DEBUG_CLEAR_MAGIC(i) i->magic = 0
#endif

#if (NSS_CONNMGR_DEBUG_LEVEL < 2)
#define NSS_CONNMGR_DEBUG_WARN(s, ...)
#else
#define NSS_CONNMGR_DEBUG_WARN(s, ...) { printk("%s:%d:WARN:", __FILE__, __LINE__);printk(s, ##__VA_ARGS__); }
#endif

#if (NSS_CONNMGR_DEBUG_LEVEL < 3)
#define NSS_CONNMGR_DEBUG_INFO(s, ...)
#else
#define NSS_CONNMGR_DEBUG_INFO(s, ...) { printk("%s:%d:INFO:", __FILE__, __LINE__);printk(s, ##__VA_ARGS__); }
#endif

#if (NSS_CONNMGR_DEBUG_LEVEL < 4)
#define NSS_CONNMGR_DEBUG_TRACE(s, ...)
#else
#define NSS_CONNMGR_DEBUG_TRACE(s, ...) { printk("%s:%d:TRACE:", __FILE__, __LINE__);printk(s, ##__VA_ARGS__); }
#endif

/*
 * Custom types recognised within the Connection Manager
 */
typedef uint8_t mac_addr_t[6];
typedef uint32_t ipv4_addr_t;

#define is_bridge_port(dev) (dev && (dev->priv_flags & IFF_BRIDGE_PORT))
#define is_bridge_device(dev) (dev->priv_flags & IFF_EBRIDGE)
#define is_lag_master(dev)	((dev->flags & IFF_MASTER)		\
				 && (dev->priv_flags & IFF_BONDING))
#define is_lag_slave(dev)	((dev->flags & IFF_SLAVE)		\
				 && (dev->priv_flags & IFF_BONDING))

/*
 * Tuple Match
 */
#define IS_TUPLE_MATCH( connection , sync) (\
		(connection->src_addr == establish->flow_ip) && \
		(connection->src_port == establish->flow_ident) && \
		(connection->src_addr_xlate == establish->flow_ip_xlate) && \
		(connection->src_port_xlate == establish->flow_ident_xlate) && \
		(connection->dest_addr == establish->return_ip) && \
		(connection->dest_port == establish->return_ident) && \
		(connection->dest_addr_xlate == establish->return_ip_xlate) && \
		(connection->dest_port_xlate == establish->return_ident_xlate))

/*
 * Local Host IP = 127.0.0.1 (0x7f:00:00:01)
 */
#define IS_LOCAL_HOST(ip) (ip == 0x7f000001)

/*
 * Displaying addresses
 */
#define MAC_AS_BYTES(mac_addr) mac_addr[5], mac_addr[4], mac_addr[3], mac_addr[2], mac_addr[1], mac_addr[0]

#define MAC_FMT "%02x:%02x:%02x:%02x:%02x:%02x"

#define IPV4_ADDR_FMT "%d.%d.%d.%d"

#define IPV4_ADDR_TO_QUAD(ip)  (ip) >> 24, ((ip) & 0x00ff0000) >> 16, ((ip) & 0x0000ff00) >> 8, ((ip) & 0x000000ff)

/*
 * Max NSS IPV4 Flow entries
 *
 * TODO - This information should come from NSS. Once NSS driver team adds an
 * API for this, change this.
 */
#define NSS_CONNMGR_IPV4_CONN_MAX 4096

/*
 * size of buffer allocated for stats printing (using debugfs)
 */
#define NSS_CONNMGR_IPV4_DEBUGFS_BUF_SZ (NSS_CONNMGR_IPV4_CONN_MAX*512)

/*
 * Maximum string length:
 * This should be equal to maximum string size of any stats
 * inclusive of stats value
 */
#define NSS_CONNMGR_IPV4_MAX_STR_LENGTH 96
#define NSS_CONNMGR_VLAN_ID_NOT_CONFIGURED 0xFFF
#define NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED 0xFFFF
#define NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED 0xFF

/*
 * Device Type for IPSec Tunnel devices
 */
#define ARPHRD_IPSEC_TUNNEL_TYPE 31

/*
 * IPV4 Connection statistics
 */
enum nss_connmgr_ipv4_conn_statistics {
	NSS_CONNMGR_IPV4_ACCELERATED_RX_PKTS = 0,
	/* Accelerated IPv4 RX packets */
	NSS_CONNMGR_IPV4_ACCELERATED_RX_BYTES,
	/* Accelerated IPv4 RX bytes */
	NSS_CONNMGR_IPV4_ACCELERATED_TX_PKTS,
	/* Accelerated IPv4 TX packets */
	NSS_CONNMGR_IPV4_ACCELERATED_TX_BYTES,
	/* Accelerated IPv4 TX bytes */
	NSS_CONNMGR_IPV4_STATS_MAX
};

typedef enum nss_connmgr_ipv4_conn_statistics nss_connmgr_ipv4_conn_statistics_t;

/*
 * Debug statistics
 */
enum nss_connmgr_ipv4_debug_statistics {
	NSS_CONNMGR_IPV4_ACTIVE_CONN,
				/* Active connections */
	NSS_CONNMGR_IPV4_CREATE_FAIL,
				/* Rule create failures */
	NSS_CONNMGR_IPV4_DESTROY_FAIL,
				/* Rule destroy failures */
	NSS_CONNMGR_IPV4_ESTABLISH_MISS,
				/* No establish response from NSS  */
	NSS_CONNMGR_IPV4_DESTROY_MISS,
				/* No Flush/Evict/Destroy response from NSS */
	NSS_CONNMGR_IPV4_DEBUG_STATS_MAX
};

typedef enum nss_connmgr_ipv4_debug_statistics nss_connmgr_debug_ipv4_statistics_t;

/*
 * nss_connmgr_ipv4_conn_stats_str
 *      Connection statistics strings
 */
static char *nss_connmgr_ipv4_conn_stats_str[] = {
	"rx_pkts",
	"rx_bytes",
	"tx_pkts",
	"tx_bytes",
};

/*
 * nss_connmgr_ipv4_stats_str
 *      Debug statistics strings
 */
static char *nss_connmgr_ipv4_debug_stats_str[] = {
	"active_conns",
	"create_fail",
	"destroy_fail",
	"establish_miss",
	"destroy_miss",
};

/*
 * Connection states as defined by connection manager
 *
 * INACTIVE    Connection is not active
 * ESTABLISHED Connection is established, and NSS sends periodic sync messages
 * STALE       Linux and NSS are out of sync.
 *		Conntrack -> Conn Mgr sent a rule destroy command,
 *		but the command did  not reach NSS
 *
 */
typedef enum  {
	NSS_CONNMGR_IPV4_STATE_INACTIVE,
	NSS_CONNMGR_IPV4_STATE_ESTABLISHED,
	NSS_CONNMGR_IPV4_STATE_STALE,
} nss_connmgr_ipv4_conn_state_t;

/*
 * IPv4 connection info
 */
struct nss_connmgr_ipv4_connection {
	nss_connmgr_ipv4_conn_state_t  state;
					/* Connection state */
	uint8_t  protocol;		/* Protocol number */
	int32_t  src_interface;		/* Flow interface number */
	uint32_t src_addr;		/* Non-NAT source address, i.e. the creator of the connection */
	int32_t  src_port;		/* Non-NAT source port */
	uint32_t src_addr_xlate;	/* NAT translated source address, i.e. the creator of the connection */
	int32_t  src_port_xlate;	/* NAT translated source port */
	char	 src_mac_addr[ETH_ALEN];	/* Source MAC address */
	uint16_t ingress_vlan_tag;	/* Ingress VLAN tag */
	int32_t  dest_interface;	/* Return interface number */
	uint32_t dest_addr;		/* Non-NAT destination address, i.e. the to whom the connection was created */
	int32_t  dest_port;		/* Non-NAT destination port */
	uint32_t dest_addr_xlate;	/* NAT translated destination address, i.e. the to whom the connection was created */
	int32_t  dest_port_xlate;	/* NAT translated destination port */
	char	 dest_mac_addr[ETH_ALEN];	/* Destination MAC address */
	uint16_t egress_vlan_tag;	/* Egress VLAN tag */
	uint64_t stats[NSS_CONNMGR_IPV4_STATS_MAX];
	/* Connection statistics */
	uint32_t last_sync;		/* Last sync time as jiffies */
};

/*
 * Global connection manager instance object.
 */
struct nss_connmgr_ipv4_instance {
	spinlock_t lock;		/* Lock to Protect against SMP access. */
	struct kobject *nom_v4;	/* Sysfs link. Represents the sysfs folder /sys/nom_v4 */
	int32_t stopped;		/* General operational control.When non-zero further traffic will not be processed */
	int32_t terminate;		/* Signal to tell the control thread to terminate */
	struct task_struct *thread;
	/* Control thread */
	void *nss_context;		/* Registration context used to identify the manager in calls to the NSS driver */
#ifdef CONFIG_NF_CONNTRACK_CHAIN_EVENTS
	struct notifier_block conntrack_notifier;
#else
	struct nf_ct_event_notifier conntrack_notifier;
#endif
					/* NF conntrack event system to monitor connection tracking changes */
#ifndef CONFIG_NF_CONNTRACK_CHAIN_EVENTS
	int (*conntrack_event_cb) (struct nf_conn *ct);
					/* conntrack event callback to propagate events to other clients (ipv6) */
#endif
	struct notifier_block netdev_notifier;
	struct nss_connmgr_ipv4_connection connection[NSS_CONNMGR_IPV4_CONN_MAX];
					/* Connection Table */

	/*
	 * Notification callback for IPv6
	 * manager about LAG slave link up.
	 */
	void (*bond_slave_linkup) (struct net_device *slave);
	struct dentry *dent;		/* Debugfs directory */
	uint32_t debug_stats[NSS_CONNMGR_IPV4_DEBUG_STATS_MAX];
					/* Debug statistics */
	uint32_t need_mark;		/* When 0 needing to see a mark value is disabled.  When != 0 we only process packets that have the given skb->mark value */
};

static unsigned int nss_connmgr_ipv4_post_routing_hook(unsigned int hooknum,
				struct sk_buff *skb,
				const struct net_device *in_unused,
				const struct net_device *out,
				int (*okfn)(struct sk_buff *));

static unsigned int nss_connmgr_ipv4_bridge_post_routing_hook(unsigned int hooknum,
                                 struct sk_buff *skb,
                                 const struct net_device *in_unused,
                                 const struct net_device *out,
                                 int (*okfn)(struct sk_buff *));

static ssize_t nss_connmgr_ipv4_read_conn_stats(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos);

static ssize_t nss_connmgr_ipv4_read_debug_stats(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos);

static ssize_t nss_connmgr_ipv4_clear_stats(struct file *fp, const char __user *ubuf, size_t count, loff_t *ppos);

#ifdef CONFIG_NF_CONNTRACK_CHAIN_EVENTS
static int nss_connmgr_ipv4_conntrack_event(struct notifier_block *this,
		unsigned long events, void *ptr);
#else
static int nss_connmgr_ipv4_conntrack_event(unsigned int events, struct nf_ct_event *item);
#endif


static struct nss_connmgr_ipv4_instance nss_connmgr_ipv4 = {
		.stopped = 0,
		.terminate= 0,
		.thread = NULL,
		.nss_context = NULL,
		.conntrack_notifier = {
#ifdef CONFIG_NF_CONNTRACK_CHAIN_EVENTS
			.notifier_call = nss_connmgr_ipv4_conntrack_event,
#else
			.fcn = nss_connmgr_ipv4_conntrack_event,
#endif
		},
		.need_mark = 0,
};

/*
 * Post routing netfilter hook
 * 	This will pick up local outbound and packets going from one interface to another
 */
static struct nf_hook_ops nss_connmgr_ipv4_ops_post_routing[] __read_mostly = {
	{
	.hook           = nss_connmgr_ipv4_post_routing_hook,
	.owner          = THIS_MODULE,
	.pf             = PF_INET,
	.hooknum        = NF_INET_POST_ROUTING,
	.priority       = NF_IP_PRI_NAT_SRC + 1, /*
						  * Refer to include/linux/netfiler_ipv4.h for priority levels.
						  * Examine packets after NAT translation (and potentially any ALG processing)
						  */
	},
	{
	.hook           = nss_connmgr_ipv4_bridge_post_routing_hook,
	.owner          = THIS_MODULE,
	.pf             = PF_BRIDGE,
	.hooknum        = NF_BR_POST_ROUTING,
	.priority       = NF_BR_PRI_FILTER_OTHER,	/*
							 * Refer to include/linux/netfiler_bridge.h for priority levels.
							 * Examine packets that are being forwarded by a bridge slave
							 */
	},
};

/*
 * Expose what should be static flags in the TCP connection tracker.
 */
extern int nf_ct_tcp_be_liberal;
extern int nf_ct_tcp_no_window_check;

static const struct file_operations nss_connmgr_ipv4_show_stats_ops = {
	.open = simple_open,
	.read = nss_connmgr_ipv4_read_conn_stats,
};

static const struct file_operations nss_connmgr_ipv4_show_debug_stats_ops = {
	.open = simple_open,
	.read = nss_connmgr_ipv4_read_debug_stats,
};

static const struct file_operations nss_connmgr_ipv4_clear_stats_ops = {
	.write = nss_connmgr_ipv4_clear_stats,
};

static struct bond_cb nss_connmgr_bond_cb;
extern struct net_device *bond_get_tx_dev(struct sk_buff *skb, uint8_t *src_mac,
					  uint8_t *dst_mac, void *src,
					  void *dst, uint16_t protocol,
					  struct net_device *bond_dev);

/**
 * @brief Send a LAG state change message
 * @param ctx NSS context
 * @param netdev The slave netdevice
 *
 * @return nss_tx_status_t
 */
extern nss_tx_status_t nss_send_lag_state(struct nss_ctx_instance *ctx, struct net_device *netdev);

extern nss_lag_event_callback_t nss_connmgr_lag_event_cb;

/*
 * Network flow
 *
 * HOST1-----------ROUTER------------HOST2
 * HOST1 has IPa and MACa
 * ROUTER has IPb and MACb facing HOST1 and IPc and MACc facing HOST2
 * HOST2 has IPd and MACd
 * i.e.
 * HOST1-----------------ROUTER----------------HOST2
 * IPa/MACa--------IPb/MACb:::IPc/MACc---------IPd/MACd
 *
 * Sending a packet from HOST1 to HOST2, the packet is mangled as follows (NAT'ed to HOST2):
 * IPa/MACa---------->IPd/MACb:::::IPc/MACc---------->IPd/MACd
 *
 * Reply:
 * IPa/MACa<----------IPd/MACb:::::IPc/MACc<----------IPd/MACd
 *
 * NOTE: IPx is considered to be IP addressing, protocol and port information combined.
 */

/*
 * nss_connmgr_ipv4_neigh_get()
 * 	Returns neighbour reference for a given IP address
 *
 */
static struct neighbour *nss_connmgr_ipv4_neigh_get(ipv4_addr_t addr)
{
	struct neighbour *neigh;
	struct rtable *rt;
	struct dst_entry *dst;

	/*
	 * Get the MAC addresses that correspond to source and destination host addresses.
	 * We look up the rtable entries and, from its neighbour structure, obtain the hardware address.
	 * This means we will also work if the neighbours are routers too.
	 */
	rt = ip_route_output(&init_net, addr, 0, 0, 0);
	if (IS_ERR(rt)) {
		return NULL;
	}

	dst = (struct dst_entry *)rt;

	neigh = dst_get_neighbour_noref(dst);

	if (!neigh) {
		neigh = neigh_lookup(&arp_tbl, &addr, dst->dev);
	} else {
		neigh_hold(neigh);
	}

	dst_release(dst);

	return neigh;
}

/*
 * nss_connmgr_ipv4_mac_addr_get()
 *	Return the hardware (MAC) address of the given IPv4 address, if any.
 *
 * Returns 0 on success or a negative result on failure.
 * We look up the rtable entry for the address and,
 * from its neighbour structure,obtain the hardware address.
 * This means we will also work if the neighbours are routers too.
 */
static int nss_connmgr_ipv4_mac_addr_get(ipv4_addr_t addr, mac_addr_t mac_addr)
{
	struct neighbour *neigh;

	rcu_read_lock();

	neigh = nss_connmgr_ipv4_neigh_get(addr);

	if (!neigh) {
		rcu_read_unlock();
		NSS_CONNMGR_DEBUG_WARN("Error: No neigh reference \n");
		return -1;
	}

	if (!(neigh->nud_state & NUD_VALID)) {
		rcu_read_unlock();
		neigh_release(neigh);
		NSS_CONNMGR_DEBUG_WARN("NUD Invalid \n");
		return -2;
	}

	if (!neigh->dev) {
		rcu_read_unlock();
		neigh_release(neigh);
		NSS_CONNMGR_DEBUG_WARN("Neigh Dev Invalid \n");
		return -3;
	}

	memcpy(mac_addr, neigh->ha, (size_t)neigh->dev->addr_len);

	rcu_read_unlock();
	neigh_release(neigh);

	/*
	 * If this mac looks like a multicast then it MAY be either truly multicast or it could be broadcast
	 * Either way we fail!  We don't want to deal with multicasts or any sort because the NSS cannot deal with them.
	 */
	if (is_multicast_ether_addr(mac_addr)) {
		NSS_CONNMGR_DEBUG_TRACE("Mac is multicast / broadcast - ignoring\n");
		return -4;
	}

	return 0;
}

static struct net_device *nss_connmgr_get_dev_from_ip_address(ipv4_addr_t addr)
{
	struct rtable *rt;
	struct dst_entry *dst;
	struct net_device *dev;

	rt = ip_route_output(&init_net, addr, 0, 0, 0);
	if (IS_ERR(rt)) {
		NSS_CONNMGR_DEBUG_INFO("couldnt get route entry for address 0x%x\n", addr);
		return NULL;
	}

	dst = (struct dst_entry *)rt;
	dev = dst->dev;
	dst_release(dst);

	return dev;
}

/*
 * nss_connmgr_bridge_post_routing_hook()
 *	Called for bridged packets about to leave the box through a bridge slave interface
 */
static unsigned int nss_connmgr_ipv4_bridge_post_routing_hook(unsigned int hooknum,
                                 struct sk_buff *skb,
                                 const struct net_device *in_unused,
                                 const struct net_device *out,
                                 int (*okfn)(struct sk_buff *))
{
	struct nss_ipv4_create unic;
	struct net_device *in;
	struct nf_conn *ct;
	enum ip_conntrack_info ctinfo;
	struct net_device *src_dev;
	struct net_device *dest_dev;
	struct net_device *br_port_in_dev, *physical_out_dev, *rt_dev;
	struct nf_conntrack_tuple orig_tuple;
	struct nf_conntrack_tuple reply_tuple;
	struct ethhdr *eh;
	uint8_t *lag_smac = NULL;
	struct net_device *dest_slave = NULL;
	struct net_device *src_slave = NULL;
	nss_tx_status_t	nss_tx_status;
	ipv4_addr_t src_ipaddr;
	struct sock *sk = NULL;
	struct udp_sock *usk = NULL;

	/*
	 * Variables needed for PPPoE WAN mode.
	 */
	struct net_device *eth_out = NULL;
	struct net_device *ppp_in = NULL;
	bool is_flow_pppoe;
	bool is_return_pppoe;

	sk = skb->sk;

	/*
	 * If the 'need_mark' flag is set and this packet does not have the relevant mark
	 * then we don't accelerate at all
	 */
	if (nss_connmgr_ipv4.need_mark && (nss_connmgr_ipv4.need_mark != skb->mark)) {
		NSS_CONNMGR_DEBUG_TRACE("Mark %x not seen, ignoring: %p\n", nss_connmgr_ipv4.need_mark, skb);
		return NF_ACCEPT;
	}

	/*
	 * Only process IPV4 packets in bridge hook
	 */
	if(skb->protocol != htons(ETH_P_IP)){
		NSS_CONNMGR_DEBUG_TRACE("non ipv4 , ignoring: %p\n", skb);
		return NF_ACCEPT;
	}

	/*
	 * Don't process broadcast or multicast
	 */
	if (skb->pkt_type == PACKET_BROADCAST) {
		NSS_CONNMGR_DEBUG_TRACE("Broadcast, ignoring: %p\n", skb);
		return NF_ACCEPT;
	}
	if (skb->pkt_type == PACKET_MULTICAST) {
		NSS_CONNMGR_DEBUG_TRACE("Multicast, ignoring: %p\n", skb);
		return NF_ACCEPT;
	}

	/*
	 * Only process packets being forwarded across the router - obtain the input interface for the skb
	 * IMPORTANT 'in' must be released with dev_put(), this is not true for 'out'
	 */
	in = dev_get_by_index(&init_net, skb->skb_iif);
	if  (!in) {
		NSS_CONNMGR_DEBUG_TRACE("Not forwarded, ignoring: %p\n", skb);
		return NF_ACCEPT;
	}

	/*
	 * Only work with standard 802.3 mac address sizes
	 * Skip mac address check for Tunnel interface
	 */
	if ((in->addr_len != 6) && (in->type != ARPHRD_SIT) && (in->type != ARPHRD_TUNNEL6)) {
		dev_put(in);
		NSS_CONNMGR_DEBUG_TRACE("in device (%s) not 802.3 hw addr len (%u), ignoring: %p\n", in->name, (unsigned)in->addr_len, skb);
		return NF_ACCEPT;
	}

	if ((out->addr_len != 6) && (out->type != ARPHRD_SIT) && (out->type != ARPHRD_TUNNEL6)) {
		dev_put(in);
		NSS_CONNMGR_DEBUG_TRACE("out device (%s) not 802.3 hw addr len (%u), ignoring: %p\n", out->name, (unsigned)out->addr_len, skb);
		return NF_ACCEPT;
	}

	/*
	 * Only process packets that are also tracked by conntrack
	 */
	ct = nf_ct_get(skb, &ctinfo);
	if (!ct) {
		dev_put(in);
		NSS_CONNMGR_DEBUG_TRACE("No conntrack connection, ignoring: %p\n", skb);
		return NF_ACCEPT;
	}
	NSS_CONNMGR_DEBUG_TRACE("skb: %p tracked by connection: %p\n", skb, ct);


	/*
	 * Ignore packets that are related to a valid connection, such as ICMP destination
	 * unreachable error packets
	 */
	if ((ctinfo == IP_CT_RELATED) || (ctinfo == IP_CT_RELATED_REPLY)) {
		dev_put(in);
		NSS_CONNMGR_DEBUG_TRACE("skb: Related packet %p tracked by connection: %p\n", skb, ct);
		return NF_ACCEPT;
	}

	/*
	 * Special untracked connection is not monitored
	 */
	if (ct == &nf_conntrack_untracked) {
		dev_put(in);
		NSS_CONNMGR_DEBUG_TRACE("%p: Untracked connection\n", ct);
		return NF_ACCEPT;
	}

	/*
	 * Any connection needing support from a 'helper' (aka NAT ALG) is kept away from the Network Accelerator
	 */
	if (nfct_help(ct)) {
		dev_put(in);
		NSS_CONNMGR_DEBUG_TRACE("%p: Connection has helper\n", ct);
		return NF_ACCEPT;
	}

	is_flow_pppoe = false;
	ppp_in = ppp_get_ppp_netdev(in);
	if (unlikely(ppp_in)) {
#if (NSS_CONNMGR_PPPOE_SUPPORT == 1)
		if (!ppp_get_session_id(ppp_in)) {
			goto out;
		}

		is_flow_pppoe = true;
#else
		goto out;
#endif
	}

	is_return_pppoe = false;
	eth_out = ppp_get_eth_netdev((struct net_device *)out);
	if (unlikely(eth_out)) {
#if (NSS_CONNMGR_PPPOE_SUPPORT == 1)
		if (!ppp_get_session_id((struct net_device *)out)) {
			goto out;
		}

		is_return_pppoe = true;
		new_out = eth_out;
#else
		goto out;
#endif
	}

	/*
	 * If egress interface is not a bridge slave port, ignore
	 */
	if (!is_bridge_port(out)) {
		dev_put(in);
		return NF_ACCEPT;
	}

	/*
	 * If destination is not reachable, ignore.
	 */
	if (skb_mac_header_was_set(skb)) {
		struct net_device *dest_dev = br_port_dev_get(out->master, eth_hdr(skb)->h_dest);

		if (dest_dev == NULL) {
			dev_put(in);
			NSS_CONNMGR_DEBUG_TRACE("Dest not reachable, skb(%p)\n", skb);
			return NF_ACCEPT;
		} else {
			dev_put(dest_dev);
		}
	}

	/*
	 * Now examine conntrack to identify the protocol, IP addresses and portal information involved
	 * IMPORTANT: The information here will be as the 'ORIGINAL' direction, i.e. who established the connection.
	 * This MAY NOT be the same as the current packet direction, for example, this packet direction may be eth1->eth0 but
	 * originally the connection may be been started from a packet going from eth0->eth1.
	 */
	orig_tuple = ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple;
	reply_tuple = ct->tuplehash[IP_CT_DIR_REPLY].tuple;
	unic.protocol = (int32_t)orig_tuple.dst.protonum;

	/*
	 * Get addressing information, non-NAT first
	 */
	unic.src_ip = (ipv4_addr_t)orig_tuple.src.u3.ip;
	unic.dest_ip = (ipv4_addr_t)orig_tuple.dst.u3.ip;

	/*
	 * NAT'ed addresses - note these are as seen from the 'reply' direction
	 * When NAT does not apply to this connection these will be identical to the above.
	 */
	unic.src_ip_xlate = (ipv4_addr_t)reply_tuple.dst.u3.ip;
	unic.dest_ip_xlate = (ipv4_addr_t)reply_tuple.src.u3.ip;

	unic.flags = 0;

	/*
	 * Store the skb->priority as the qos tag
	 */
	unic.qos_tag = (uint32_t)skb->priority;

	/*
	 * Only set the routed flag if the interface from which this packet came
	 * was NOT a bridge interface OR if it is then it is not of the same bridge we are outputting onto.
	 */
	if (!is_bridge_port(in) || (out->master != in->master)) {
		unic.flags |= NSS_IPV4_CREATE_FLAG_ROUTED;
	}

	/*
	 * Reset the pppoe session info
	 */
	unic.return_pppoe_session_id = 0;
	unic.flow_pppoe_session_id = 0;

	switch (unic.protocol) {
	case IPPROTO_TCP:
		unic.src_port = (int32_t)orig_tuple.src.u.tcp.port;
		unic.dest_port = (int32_t)orig_tuple.dst.u.tcp.port;
		unic.src_port_xlate = (int32_t)reply_tuple.dst.u.tcp.port;
		unic.dest_port_xlate = (int32_t)reply_tuple.src.u.tcp.port;
		unic.flow_window_scale = ct->proto.tcp.seen[0].td_scale;
		unic.flow_max_window = ct->proto.tcp.seen[0].td_maxwin;
		unic.flow_end = ct->proto.tcp.seen[0].td_end;
		unic.flow_max_end = ct->proto.tcp.seen[0].td_maxend;
		unic.return_window_scale = ct->proto.tcp.seen[1].td_scale;
		unic.return_max_window = ct->proto.tcp.seen[1].td_maxwin;
		unic.return_end = ct->proto.tcp.seen[1].td_end;
		unic.return_max_end = ct->proto.tcp.seen[1].td_maxend;

		if (nf_ct_tcp_be_liberal || nf_ct_tcp_no_window_check ||
			(ct->proto.tcp.seen[0].flags & IP_CT_TCP_FLAG_BE_LIBERAL) ||
			(ct->proto.tcp.seen[1].flags & IP_CT_TCP_FLAG_BE_LIBERAL)) {

			unic.flags |= NSS_IPV4_CREATE_FLAG_NO_SEQ_CHECK;
		}

		/*
		 * Don't try to manage a non-established connection.
		 */
		if (!test_bit(IPS_ASSURED_BIT, &ct->status)) {
			dev_put(in);
			NSS_CONNMGR_DEBUG_TRACE("%p: Non-established connection\n", ct);
			return NF_ACCEPT;
		}

		/*
		 * If the connection is shutting down do not manage it.
		 * state can not be SYN_SENT, SYN_RECV because connection is assured
		 * Not managed states: FIN_WAIT, CLOSE_WAIT, LAST_ACK, TIME_WAIT, CLOSE.
		 */
		spin_lock_bh(&ct->lock);
		if (ct->proto.tcp.state != TCP_CONNTRACK_ESTABLISHED) {
			spin_unlock_bh(&ct->lock);
			dev_put(in);
			NSS_CONNMGR_DEBUG_TRACE("%p: Connection in termination state %#X\n", ct, ct->proto.tcp.state);
			return NF_ACCEPT;
		}
		spin_unlock_bh(&ct->lock);

		break;

	case IPPROTO_UDP:
		unic.src_port = (int32_t)orig_tuple.src.u.udp.port;
		unic.dest_port = (int32_t)orig_tuple.dst.u.udp.port;
		unic.src_port_xlate = (int32_t)reply_tuple.dst.u.udp.port;
		unic.dest_port_xlate = (int32_t)reply_tuple.src.u.udp.port;
		usk = udp_sk(sk);
		break;

	case IPPROTO_IPV6:
		unic.src_port = 0;
		unic.dest_port = 0;
		unic.src_port_xlate = 0;
		unic.dest_port_xlate = 0;
		break;

	default:
		/*
		 * Streamengine compatibility - database stores non-ported protocols with port numbers equal to negative protocol number
		 * to indicate unused.
		 */
		dev_put(in);
		NSS_CONNMGR_DEBUG_TRACE("%p: Unhandled protocol %d\n", ct, unic.protocol);
		return NF_ACCEPT;
	}

	/* Handle L2TP UDP encapsulated frames */
	if (usk) {
		if (usk->encap_type == UDP_ENCAP_L2TPINUDP) {
			dev_put(in);
			NSS_CONNMGR_DEBUG_TRACE("%p: skip packets for L2TP tunnel\n", ct);
			return NF_ACCEPT;
		}
	}

	/*
	 * Initialize the VLAN tag information.
	 */
	unic.ingress_vlan_tag = NSS_CONNMGR_VLAN_ID_NOT_CONFIGURED;
	unic.egress_vlan_tag = NSS_CONNMGR_VLAN_ID_NOT_CONFIGURED;
	/*
	 * Initialize DSCP and VLAN MARKING information
	 */
	unic.dscp_itag =  NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED;
	unic.dscp_imask =  NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED;
	unic.dscp_omask =  NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED;
	unic.dscp_oval =  NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED;
	unic.vlan_itag =  NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED;
	unic.vlan_imask=  NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED;
	unic.vlan_omask =  NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED;
	unic.vlan_oval =  NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED;

	/*
	 * Access the original source IP address before any NAT translation
	 */
	if (ctinfo < IP_CT_IS_REPLY) {
		src_ipaddr = (ipv4_addr_t)orig_tuple.src.u3.ip;
	} else {
		src_ipaddr = (ipv4_addr_t)reply_tuple.src.u3.ip;
	}

	/*
	 * Access the ingress routed interface
	 */
	rt_dev = nss_connmgr_get_dev_from_ip_address(src_ipaddr);

	/*
	 * Is this pure bridge flow with no Layer-3 routing involved in the path?
	 */
	if ((rt_dev == NULL)
		|| (is_bridge_device(rt_dev) && (rt_dev == (struct net_device *)out->master))) {
		/*
		 * Ensure MAC header is in place, which may be sripped off in some cases, for
		 * example, for partially accelerated IPv6 tunnel exception packets.
		 */
		if (!skb_mac_header_was_set(skb)) {
			dev_put(in);
			NSS_CONNMGR_DEBUG_TRACE("Bridge-CM: MAC address not present for bridge %s packet\n",out->master->name);
			return NF_ACCEPT;
		}

		/*
		 * Is the ingress bridge port is a virtual interface?
		 */
		if (!is_bridge_port(in)) {
			/*
			 * Try to access the ingress bridge slave interface for this packet
			 */
			br_port_in_dev = br_port_dev_get(out->master, eth_hdr(skb)->h_source);
			if (br_port_in_dev == NULL) {
				dev_put(in);
				NSS_CONNMGR_DEBUG_TRACE("Bridge-CM: Ingress Virtual Port not found for bridge %s\n",out->master->name);
				return NF_ACCEPT;
			}
			NSS_CONNMGR_DEBUG_INFO("Bridge-CM: Bridge Port Ingress Virtual Interface = %s\n", br_port_in_dev->name);

			/*
			 * Is the ingress slave interface of the bridge a VLAN interface?
			 */
			if (is_vlan_dev(br_port_in_dev)) {
				/*
				 * Access the VLAN ID of the VLAN interface
				 */
				if (ctinfo < IP_CT_IS_REPLY) {
					unic.ingress_vlan_tag = vlan_dev_priv(br_port_in_dev)->vlan_id;
					NSS_CONNMGR_DEBUG_TRACE("Bridge-CM: Ingress VLAN ID = %d\n",vlan_dev_priv(br_port_in_dev)->vlan_id);
				} else {
					unic.egress_vlan_tag = vlan_dev_priv(br_port_in_dev)->vlan_id;
					NSS_CONNMGR_DEBUG_TRACE("Bridge-CM: Egress VLAN ID = %d\n",vlan_dev_priv(br_port_in_dev)->vlan_id);
				}
			}
			dev_put(br_port_in_dev);
		}
		unic.flags |= NSS_IPV4_CREATE_FLAG_BRIDGE_FLOW;

		/*
		 * Configure the MAC addresses for the flow
		 */
		eh = (struct ethhdr *)skb->mac_header;
		if (ctinfo < IP_CT_IS_REPLY) {
			memcpy(unic.src_mac, eh->h_source, ETH_HLEN);
			memcpy(unic.dest_mac, eh->h_dest, ETH_HLEN);
			memcpy(unic.src_mac_xlate, eh->h_source, ETH_HLEN);
			memcpy(unic.dest_mac_xlate, eh->h_dest, ETH_HLEN);
		} else {
			memcpy(unic.src_mac, eh->h_dest, ETH_HLEN);
			memcpy(unic.dest_mac, eh->h_source, ETH_HLEN);
			memcpy(unic.src_mac_xlate, eh->h_dest, ETH_HLEN);
			memcpy(unic.dest_mac_xlate, eh->h_source, ETH_HLEN);
		}
	} else {
		/*
		 * This is a routed + bridged flow
		 */

		/*
		 * Check if the ingress is a VLAN interface
		 */
		if (is_vlan_dev(rt_dev)) {
			if (ctinfo < IP_CT_IS_REPLY) {
				unic.ingress_vlan_tag = vlan_dev_priv(rt_dev)->vlan_id;
				NSS_CONNMGR_DEBUG_TRACE("Bridge-CM: Ingress VLAN ID = %d\n",vlan_dev_priv(rt_dev)->vlan_id);
			} else {
				unic.egress_vlan_tag = vlan_dev_priv(rt_dev)->vlan_id;
				NSS_CONNMGR_DEBUG_TRACE("Bridge-CM: Egress VLAN ID = %d\n",vlan_dev_priv(rt_dev)->vlan_id);
			}
		}

		/*
		 * Get the MAC addresses that correspond to source and destination host addresses.
		 */
		if (nss_connmgr_ipv4_mac_addr_get(unic.src_ip, unic.src_mac)) {
			dev_put(in);
			NSS_CONNMGR_DEBUG_TRACE("%p: Failed to find MAC address for src IP: %pI4\n", ct, &unic.src_ip);
			return NF_ACCEPT;
		}

		if (nss_connmgr_ipv4_mac_addr_get(unic.src_ip_xlate, unic.src_mac_xlate)) {
			dev_put(in);
			NSS_CONNMGR_DEBUG_TRACE("%p: Failed to find MAC address for xlate src IP: %pI4\n", ct, &unic.src_ip_xlate);
			return NF_ACCEPT;
		}

		/*
		 * Do dest now
		 */
		if (nss_connmgr_ipv4_mac_addr_get(unic.dest_ip, unic.dest_mac)) {
			dev_put(in);
			NSS_CONNMGR_DEBUG_TRACE("%p: Failed to find MAC address for dest IP: %pI4\n", ct, &unic.dest_ip);
			return NF_ACCEPT;
		}

		if (nss_connmgr_ipv4_mac_addr_get(unic.dest_ip_xlate, unic.dest_mac_xlate)) {
			dev_put(in);
			NSS_CONNMGR_DEBUG_TRACE("%p: Failed to find MAC address for xlate dest IP: %pI4\n", ct, &unic.dest_ip_xlate);
			return NF_ACCEPT;
		}
	}

	physical_out_dev = (struct net_device *)out;

	/*
	 * Check if we have a VLAN interface at the egress
	 */
	if (is_vlan_dev((struct net_device *)out)) {
		/*
		 * Access the physical egress interface
		 */
		physical_out_dev = vlan_dev_priv(out)->real_dev;

		if (ctinfo < IP_CT_IS_REPLY) {
			unic.egress_vlan_tag = vlan_dev_priv(out)->vlan_id;
			NSS_CONNMGR_DEBUG_TRACE("Bridge-CM: Egress VLAN ID = %d\n",vlan_dev_priv(out)->vlan_id);
		} else {
			unic.ingress_vlan_tag = vlan_dev_priv(out)->vlan_id;
			NSS_CONNMGR_DEBUG_TRACE("Bridge-CM: Ingress VLAN ID = %d\n",vlan_dev_priv(out)->vlan_id);
		}
	}

	/*
	 * Get MAC addresses
	 * NOTE: We are dealing with the ORIGINAL direction here so 'in' and 'out' dev may need
	 * to be swapped if this packet is a reply
	 */
	if (ctinfo < IP_CT_IS_REPLY) {
		NSS_CONNMGR_DEBUG_TRACE("%p: dir: Original\n", ct);
		src_dev = in;
		dest_dev = physical_out_dev;
	} else {
		NSS_CONNMGR_DEBUG_TRACE("%p: dir: Reply\n", ct);
		src_dev = physical_out_dev;
		dest_dev = in;
	}

	/*
	 * Handle Link Aggregation
	 */
	if (ctinfo < IP_CT_IS_REPLY) {
		/*
		 * If ingress interface is a lag slave, we cannot assume that
		 * reply packets will use this interface for egress; call
		 * bonding driver API to find the egress interface
		 * for reply packets.
		 */
		if (is_lag_slave(src_dev)) {
			if (unic.flags & NSS_IPV4_CREATE_FLAG_BRIDGE_FLOW) {
				lag_smac = unic.dest_mac_xlate;
			} else {
				lag_smac = (uint8_t *)src_dev->master->dev_addr;
			}

			src_slave = bond_get_tx_dev(NULL, lag_smac,
						    unic.src_mac,
						    (void *)&unic.dest_ip_xlate,
						    (void *)&unic.src_ip_xlate,
						    skb->protocol,
						    src_dev->master);
			if (src_slave == NULL
			    || !netif_carrier_ok(src_slave)) {
				dev_put(in);
				return NF_ACCEPT;
			}

			src_dev = src_slave;
		}

		/*
		 * Determine egress LAG slave interface
		 * for packet in original direction.
		 */
		if (is_lag_master(dest_dev)) {
			if (unic.flags & NSS_IPV4_CREATE_FLAG_BRIDGE_FLOW) {
				lag_smac = unic.src_mac;
			} else {
				lag_smac = (uint8_t *)dest_dev->dev_addr;
			}

			dest_slave = bond_get_tx_dev(NULL, lag_smac,
						     unic.dest_mac_xlate,
						     (void *)&unic.src_ip_xlate,
						     (void *)&unic.dest_ip_xlate,
						     skb->protocol, dest_dev);

			if (dest_slave == NULL
			    || !netif_carrier_ok(dest_slave)) {
				dev_put(in);
				return NF_ACCEPT;
			}

			dest_dev = dest_slave;
		}
	} else {
		/*
		 * At this point, src_dev points to ingress IF and and dest_dev
		 * points to egress 'physical' IF, both in original direction.
		 */

		/*
		 * If, in original direction, the ingress [virtual] IF was a
		 * LAG master, then we must determine the egress LAG slave for
		 * reply packets (such as the current packet).
		 */
		if (is_lag_master(src_dev)) {
			if (unic.flags & NSS_IPV4_CREATE_FLAG_BRIDGE_FLOW) {
				lag_smac = unic.dest_mac_xlate;
			} else {
				lag_smac = src_dev->dev_addr;
			}

			src_slave = bond_get_tx_dev(NULL, lag_smac,
						    unic.src_mac,
						    (void *)&unic.dest_ip_xlate,
						    (void *)&unic.src_ip_xlate,
						    skb->protocol, src_dev);
			if (src_slave == NULL || !netif_carrier_ok(src_slave)) {
				dev_put(in);
				return NF_ACCEPT;
			}

			src_dev = src_slave;
		}

		/*
		 * If, in original direction, the egress IF was a LAG slave
		 * then we must determine the correct egress LAG slave, for
		 * packets in original direction.
		 */
		if (is_lag_slave(dest_dev)) {
			if (unic.flags & NSS_IPV4_CREATE_FLAG_BRIDGE_FLOW) {
				lag_smac = unic.src_mac;
			} else {
				lag_smac = (uint8_t *)dest_dev->master->dev_addr;
			}

			dest_slave = bond_get_tx_dev(NULL, lag_smac,
						     unic.dest_mac_xlate,
						     (void *)&unic.src_ip_xlate,
						     (void *)&unic.dest_ip_xlate,
						     skb->protocol,
						     dest_dev->master);
			if (dest_slave == NULL || !netif_carrier_ok(dest_slave)) {
				dev_put(in);
				return NF_ACCEPT;
			}

			dest_dev = dest_slave;
		}
	}



	/*
	 * Only devices that are NSS devices may be accelerated.
	 */
	unic.src_interface_num = nss_cmn_get_interface_number(nss_connmgr_ipv4.nss_context, src_dev);
	if (unic.src_interface_num < 0) {
		dev_put(in);
		NSS_CONNMGR_DEBUG_INFO("Source Interface %s NOT owned by NSS\n",src_dev->name);
		return NF_ACCEPT;
	}

	unic.dest_interface_num = nss_cmn_get_interface_number(nss_connmgr_ipv4.nss_context, dest_dev);
	if (unic.dest_interface_num < 0) {
		dev_put(in);
		NSS_CONNMGR_DEBUG_INFO("Dest Interface %s NOT owned by NSS\n",dest_dev->name);
		return NF_ACCEPT;
	}

	/*
	 * We have everything we need (hopefully :-])
	 */
	NSS_CONNMGR_DEBUG_TRACE("\n%p: Bridge Conntrack connection\n"
			"skb: %p\n"
			"dir: %s\n"
			"Protocol: %d\n"
			"src_ip: " IPV4_ADDR_FMT ":%d\n"
			"dest_ip: " IPV4_ADDR_FMT "%d"
			"src_ip_xlate: " IPV4_ADDR_FMT "%d\n"
			"dest_ip_xlate: " IPV4_ADDR_FMT "%d\n"
			"src_mac: " MAC_FMT "\n"
			"dest_mac: " MAC_FMT "\n"
			"src_mac_xlate: " MAC_FMT "\n"
			"dest_mac_xlate: " MAC_FMT "\n"
			"src_dev: %s\n"
			"dest_dev: %s\n"
			"src_iface_num: %u\n"
			"dest_iface_num: %u\n"
			"ingress_vlan_tag: %u\n"
			"egress_vlan_tag: %u\n"
			"qos_tag: %u\n",
			ct,
			skb,
			(ctinfo < IP_CT_IS_REPLY)? "Original" : "Reply",
			unic.protocol,
			IPV4_ADDR_TO_QUAD(unic.src_ip), unic.src_port,
			IPV4_ADDR_TO_QUAD(unic.dest_ip), unic.dest_port,
			IPV4_ADDR_TO_QUAD(unic.src_ip_xlate), unic.src_port_xlate,
			IPV4_ADDR_TO_QUAD(unic.dest_ip_xlate), unic.dest_port_xlate,
			MAC_AS_BYTES(unic.src_mac),
			MAC_AS_BYTES(unic.dest_mac),
			MAC_AS_BYTES(unic.src_mac_xlate),
			MAC_AS_BYTES(unic.dest_mac_xlate),
			src_dev->name,
			dest_dev->name,
			unic.src_interface_num,
			unic.dest_interface_num,
			unic.ingress_vlan_tag,
			unic.egress_vlan_tag,
			unic.qos_tag);

	if (!offload_dscpremark_get_target_info(ct, &unic.dscp_imask, &unic.dscp_itag, &unic.dscp_omask, &unic.dscp_oval)) {
		NSS_CONNMGR_DEBUG_INFO("DSCP remark information is not present\n");
	}

	if (!offload_vlantag_get_target_info(ct, &unic.vlan_imask, &unic.vlan_itag, &unic.vlan_omask, &unic.vlan_oval)) {
		NSS_CONNMGR_DEBUG_INFO("VLAN tagging information is not present\n");
	}

	NSS_CONNMGR_DEBUG_INFO("unic.dscp_imask = %d\n", unic.dscp_imask);
	NSS_CONNMGR_DEBUG_INFO("unic.dscp_itag = %d\n", unic.dscp_itag);
	NSS_CONNMGR_DEBUG_INFO("unic.dscp_omask = %d\n", unic.dscp_omask);
	NSS_CONNMGR_DEBUG_INFO("unic.dscp_oval = %d\n", unic.dscp_oval);

	/*
	 * Setting the appropriate flags for  DSCP and VLAN marking.
	 * API's to set the data for remarking should be called before this.
	 */
	if(unic.dscp_oval != NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED)
		unic.flags |= NSS_IPV4_CREATE_FLAG_DSCP_MARKING;
	if(unic.vlan_oval != NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED)
		unic.flags |= NSS_IPV4_CREATE_FLAG_VLAN_MARKING;

	/*
	 * Create the Network Accelerator connection cache entries
	 *
	 * NOTE: All of the information we have is from the point of view of who created the connection however
	 * the skb may actually be in the 'reply' direction - which is important to know when configuring the NSS as we have to set the right information
	 * on the right "match" and "forwarding" entries.
	 * We can use the ctinfo to determine which direction the skb is in and then swap fields as necessary.
	 */
	unic.src_ip = ntohl(unic.src_ip);
	unic.dest_ip = ntohl(unic.dest_ip);
	unic.src_ip_xlate = ntohl(unic.src_ip_xlate);
	unic.dest_ip_xlate = ntohl(unic.dest_ip_xlate);
	unic.src_port = ntohs(unic.src_port);
	unic.dest_port = ntohs(unic.dest_port);
	unic.src_port_xlate = ntohs(unic.src_port_xlate);
	unic.dest_port_xlate = ntohs(unic.dest_port_xlate);

	/*
	 * Get MTU values for source and destination interfaces.
	 * TODO: Review this for VLAN, bridge
	 */
	unic.from_mtu = in->mtu;
	unic.to_mtu = out->mtu;

	/*
	 * If operations have stopped then do not proceed further
	 */
	spin_lock_bh(&nss_connmgr_ipv4.lock);
	if (unlikely(nss_connmgr_ipv4.stopped)) {
		spin_unlock_bh(&nss_connmgr_ipv4.lock);
		dev_put(in);
		NSS_CONNMGR_DEBUG_TRACE("Stopped, ignoring: %p\n", skb);
		return NF_ACCEPT;
	}
	spin_unlock_bh(&nss_connmgr_ipv4.lock);

	/*
	 * Calling ipv4_rule1 for DSCP and VLAN marking else we use the regular
	 * ipv4_rule
	 */
	if(unic.dscp_oval == NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED &&
		unic.vlan_oval == NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED) {
		nss_tx_status = nss_tx_create_ipv4_rule(nss_connmgr_ipv4.nss_context, &unic);
	} else {
		nss_tx_status = nss_tx_create_ipv4_rule1(nss_connmgr_ipv4.nss_context, &unic);
	}

	if (nss_tx_status == NSS_TX_SUCCESS) {
		goto out;
	} else if (nss_tx_status == NSS_TX_FAILURE_NOT_READY) {
		NSS_CONNMGR_DEBUG_ERROR("NSS not ready to accept rule \n");
		nss_connmgr_ipv4.debug_stats[NSS_CONNMGR_IPV4_CREATE_FAIL]++;
	} else {
		NSS_CONNMGR_DEBUG_TRACE("NSS create rule failed  skb: %p\n", skb);
		nss_connmgr_ipv4.debug_stats[NSS_CONNMGR_IPV4_CREATE_FAIL]++;
	}

out:
	/*
	 * Release the interface on which this skb arrived
	 */
	dev_put(in);

	if (ppp_in) {
		dev_put(ppp_in);
	}

	if (eth_out) {
		dev_put(eth_out);
	}

	return NF_ACCEPT;
}

/*
 * nss_connmgr_destroy_ipv4_rule()
 *     Destroy an ipv4 rule. Called with nss_connmgr_ipv4.lock held.
 */
static int32_t nss_connmgr_destroy_ipv4_rule(struct nss_connmgr_ipv4_connection *connection)
{
	struct nss_ipv4_destroy unid;
	nss_tx_status_t nss_tx_status;

	unid.protocol = connection->protocol;
	unid.src_ip = connection->src_addr;
	unid.dest_ip = connection->dest_addr;
	unid.src_port = connection->src_port;
	unid.dest_port = connection->dest_port;

	nss_tx_status = nss_tx_destroy_ipv4_rule(nss_connmgr_ipv4.nss_context,
						 &unid);
	if (nss_tx_status != NSS_TX_SUCCESS) {
		NSS_CONNMGR_DEBUG_ERROR("Unable to destroy IPv4 rule."
					"src %X:%d dst %X:%d proto %d\n",
					unid.src_ip, unid.src_port,
					unid.dest_ip, unid.dest_port,
					unid.protocol);
		return -EIO;
	}

	return 0;
}


/*
 * nss_connmgr_link_down()
 * 	Handle a link down event on an interface.
 *	Only handles interfaces used by NSS.
 */
static void nss_connmgr_link_down(struct net_device *dev)
{
	uint32_t if_num = 0;
	uint32_t i = 0;
	int32_t if_src = 0;
	int32_t if_dst = 0;
	uint16_t vlan_id = 0;
	struct nss_connmgr_ipv4_connection *connection = NULL;
	nss_connmgr_ipv4_conn_state_t cstate;
	struct net_device *phys_dev = dev;

	/*
	 * Get the physical device and VLAN ID for vlan interfaces
	 */
	if (is_vlan_dev(dev)) {
		phys_dev = vlan_dev_priv(dev)->real_dev;
		vlan_id = vlan_dev_priv(dev)->vlan_id;
		if (phys_dev == NULL) {
			NSS_CONNMGR_DEBUG_WARN("Invalid physical device for VLAN device %s\n",dev->name);
			return;
		}
	}

	if_num = nss_cmn_get_interface_number(nss_connmgr_ipv4.nss_context, phys_dev);

	if (if_num < 0) {
		NSS_CONNMGR_DEBUG_WARN("Cannot find NSS if num for dev %s\n", phys_dev->name);
		return;
	}

	for (i = 0; i < NSS_CONNMGR_IPV4_CONN_MAX; i++) {
		spin_lock_bh(&nss_connmgr_ipv4.lock);
		connection = &nss_connmgr_ipv4.connection[i];
		cstate = connection->state;
		if_src = connection->src_interface;
		if_dst = connection->dest_interface;

		if (cstate != NSS_CONNMGR_IPV4_STATE_ESTABLISHED) {
			spin_unlock_bh(&nss_connmgr_ipv4.lock);
			continue;
		}

		if (if_num == if_src
		    || if_num == if_dst) {
			/*
			 * Check if we have a VLAN rule to delete
			 */
			if (is_vlan_dev(dev)) {
				if ((vlan_id != connection->ingress_vlan_tag)
					&& (vlan_id != connection->egress_vlan_tag)) {
					spin_unlock_bh(&nss_connmgr_ipv4.lock);
					continue;
				}
			}
			NSS_CONNMGR_DEBUG_INFO("destroy NSS rule at index %d\n", i);
			nss_connmgr_destroy_ipv4_rule(connection);
		}
		spin_unlock_bh(&nss_connmgr_ipv4.lock);
	}
}

/*
 * nss_connmgr_bond_link_up()
 *     Callback used to signal a link up on an interface
 *     that is part of a link aggregation.
 */
static void nss_connmgr_bond_link_up(struct net_device *slave_dev)
{
	uint32_t if_num = 0;
	uint32_t i = 0;
	struct nss_connmgr_ipv4_connection *connection = NULL;
	uint8_t flush_rule;
	uint32_t src_if;
	uint32_t dst_if;
	struct net_device *indev;
	struct net_device *outdev;
	nss_connmgr_ipv4_conn_state_t cstate;

	if_num = nss_cmn_get_interface_number(nss_connmgr_ipv4.nss_context, slave_dev);
	if (if_num < 0) {
		NSS_CONNMGR_DEBUG_ERROR("Cannot find NSS if num for slave dev %s\n", slave_dev->name);
		return;
	}

	for (i = 0; i < NSS_CONNMGR_IPV4_CONN_MAX; i++)
	{
		spin_lock_bh(&nss_connmgr_ipv4.lock);
		connection = &nss_connmgr_ipv4.connection[i];
		cstate = connection->state;
		src_if = connection->src_interface;
		dst_if = connection->dest_interface;
		spin_unlock_bh(&nss_connmgr_ipv4.lock);

		if (cstate != NSS_CONNMGR_IPV4_STATE_ESTABLISHED) {
			continue;
		}

		flush_rule = 0;

		indev = nss_cmn_get_interface_dev(nss_connmgr_ipv4.nss_context, src_if);
		if (indev != NULL) {
			if (is_lag_slave(indev)) {
				flush_rule = 1;
			}
		}

		outdev = nss_cmn_get_interface_dev(nss_connmgr_ipv4.nss_context, dst_if);
		if (outdev != NULL) {
			if (is_lag_slave(outdev)) {
				flush_rule = 1;
			}
		}

		if (flush_rule) {
			spin_lock_bh(&nss_connmgr_ipv4.lock);
			if (connection->state != NSS_CONNMGR_IPV4_STATE_ESTABLISHED) {
				spin_unlock_bh(&nss_connmgr_ipv4.lock);
				continue;
			}

			NSS_CONNMGR_DEBUG_INFO("destroying NSS rule at index %d\n", i);
			nss_connmgr_destroy_ipv4_rule(connection);
			spin_unlock_bh(&nss_connmgr_ipv4.lock);
		}
	}

	/* Notify IPv6 connection manager */
	if (nss_connmgr_ipv4.bond_slave_linkup) {
		nss_connmgr_ipv4.bond_slave_linkup(slave_dev);
	}
}

/*
 * nss_connmgr_bond_release()
 *	Callback used to signal a physical interface
 *	leaving an aggregation.
 */
static void nss_connmgr_bond_release(struct net_device *slave_dev)
{
	nss_send_lag_state(nss_connmgr_ipv4.nss_context, slave_dev);
}

/*
 * nss_connmgr_bond_release()
 *	Callback used to signal a physical interface
 *	joining an aggregation.
 */
static void nss_connmgr_bond_enslave(struct net_device *slave_dev)
{
	nss_send_lag_state(nss_connmgr_ipv4.nss_context, slave_dev);
}

/*
 * nss_connmgr_ipv4_post_routing_hook()
 *	Called for packets about to leave the box - either locally generated or forwarded from another interface
 */
static unsigned int nss_connmgr_ipv4_post_routing_hook(unsigned int hooknum,
				struct sk_buff *skb,
				const struct net_device *in_unused,
				const struct net_device *out,
				int (*okfn)(struct sk_buff *))
{
	struct nss_ipv4_create unic;

	struct net_device *in;
	struct nf_conn *ct;
	enum ip_conntrack_info ctinfo;
	struct net_device *src_dev;
	struct net_device *dest_dev;
	struct net_device *rt_dev, *br_port_dev;
	struct nf_conntrack_tuple orig_tuple;
	struct nf_conntrack_tuple reply_tuple;
	struct iphdr *iph;
	ipv4_addr_t src_ipaddr;
	struct sock *sk = NULL;
	struct udp_sock *usk = NULL;

	nss_tx_status_t nss_tx_status;

	/*
	 * Variables needed for PPPoE WAN mode.
	 */
	struct net_device *new_out = (struct net_device *)out;
	struct net_device *eth_out = NULL;
	struct net_device *ppp_in = NULL;
	struct net_device *ppp_src = NULL;
	struct net_device *ppp_dest = NULL;
	bool is_flow_pppoe = false;
	bool is_return_pppoe = false;
	bool is_tmp_pppoe = false;

	/*
	 * Variables needed for LAG
	 */
	struct net_device *dest_slave = NULL;
	struct net_device *src_slave = NULL;

	sk = skb->sk;

	/*
	 * If the 'need_mark' flag is set and this packet does not have the relevant mark
	 * then we don't accelerate at all
	 */
	if (nss_connmgr_ipv4.need_mark && (nss_connmgr_ipv4.need_mark != skb->mark)) {
		NSS_CONNMGR_DEBUG_TRACE("Mark %x not seen, ignoring: %p\n", nss_connmgr_ipv4.need_mark, skb);
		return NF_ACCEPT;
	}

	/*
	 * Don't process broadcast or multicast
	 */
	if (skb->pkt_type == PACKET_BROADCAST) {
		NSS_CONNMGR_DEBUG_TRACE("Broadcast, ignoring: %p\n", skb);
		return NF_ACCEPT;
	}

	if (skb->pkt_type == PACKET_MULTICAST) {
		NSS_CONNMGR_DEBUG_TRACE("Multicast, ignoring: %p\n", skb);
		return NF_ACCEPT;
	}

	/*
	 * Only process packets being forwarded across the router - obtain the input interface for the skb
	 * IMPORTANT 'in' must be released with dev_put(), this is not true for 'out'
	 */
	in = dev_get_by_index(&init_net, skb->skb_iif);
	if  (!in) {
		NSS_CONNMGR_DEBUG_TRACE("Not forwarded, ignoring: skb %p iif %d\n", skb, skb->skb_iif);
		return NF_ACCEPT;
	}

	/*
	 * Only work with standard 802.3 mac address sizes
	 */
	if ((in->addr_len != 6) && (in->type != ARPHRD_SIT) && (in->type != ARPHRD_TUNNEL6)) {
		NSS_CONNMGR_DEBUG_TRACE("in device (%s) not 802.3 hw addr len (%u), ignoring: %p\n", in->name, (unsigned)in->addr_len, skb);
		goto out;
	}

	/*
	 * If egress interface is a bridge,ignore; the rule will be added by bridge post-routing hook
	 */
	if (is_bridge_device(out)) {
		NSS_CONNMGR_DEBUG_TRACE("ignoring bridge device in post-routing hook: %p\n", skb);
		goto out;
	}

	/*
	 * Only process packets that are also tracked by conntrack
	 */
	ct = nf_ct_get(skb, &ctinfo);
	if (!ct) {
		NSS_CONNMGR_DEBUG_TRACE("No conntrack connection, ignoring: %p\n", skb);
		goto out;
	}
	NSS_CONNMGR_DEBUG_TRACE("skb: %p tracked by connection: %p\n", skb, ct);

	/*
	 * Ignore packets that are related to a valid connection, such as ICMP destination
	 * unreachable error packets
	 */
	if ((ctinfo == IP_CT_RELATED) || (ctinfo == IP_CT_RELATED_REPLY)) {
		NSS_CONNMGR_DEBUG_TRACE("skb: Related packet %p tracked by connection: %p\n", skb, ct);
		goto out;
	}

	/*
	 * Special untracked connection is not monitored
	 */
	if (ct == &nf_conntrack_untracked) {
		NSS_CONNMGR_DEBUG_TRACE("%p: Untracked connection\n", ct);
		goto out;
	}

	/*
	 * Any connection needing support from a 'helper' (aka NAT ALG) is kept away from the Network Accelerator
	 */
	if (nfct_help(ct)) {
		NSS_CONNMGR_DEBUG_TRACE("%p: Connection has helper\n", ct);
		goto out;
	}

	/*
	 * In PPPoE connection case, the interfaces will be as below in this function.
	 *
	 *
	 * ORIGINAL DIRECTION for LAN to WAN:
	 *
	 * PC1 --------> LAN (eth2) -----------> WAN (eth3 - pppoe-wan) -------> PC2
	 *				in = eth2
	 *				out = pppoe-wan
	 *
	 * REPLY DIRECTION for LAN to WAN:
	 *
	 * PC1 <-------- LAN (eth2) <----------- WAN (eth3 - pppoe-wan) <------- PC2
	 *				in = eth3
	 *				out = eth2
	 *
	 * ORIGINAL DIRECTION for WAN to LAN:
	 *
	 * PC1 <-------- LAN (eth2) <----------- WAN (eth3 - pppoe-wan) <------- PC2
	 *				in = eth3
	 *				out = eth2
	 *
	 * REPLY DIRECTION for WAN to LAN:
	 *
	 * PC1 --------> LAN (eth2) -----------> WAN (eth3 - pppoe-wan) -------> PC2
	 *				in = eth2
	 *				out = pppoe-wan
	 *
	 */

	/*
	 * Input device is always physical device (eth0, eth1, etc...), because dev_get_by_index()
	 * returns the physical interface device.
	 */
	ppp_in = ppp_get_ppp_netdev(in);
	if (unlikely(ppp_in)) {
#if (NSS_CONNMGR_PPPOE_SUPPORT == 1)
		/*
		 * It mway not be pppoe interface. It may be PPTP or L2TP.
		 */
		if (!ppp_get_session_id(ppp_in)) {
			goto out;
		}

		is_flow_pppoe = true;
#else
		goto out;
#endif
	}

	/*
	 * ppp_get_eth_netdev returns the corresponding Ethernet net_device if "out" is a PPP net_device.
	 * Otherwise it returns NULL. Let's obtain the physical device from the PPP device, then do our
	 * all further processes with this physical device.
	 */
	eth_out = ppp_get_eth_netdev((struct net_device *)out);
	if (unlikely(eth_out)) {
#if (NSS_CONNMGR_PPPOE_SUPPORT == 1)
		/*
		 * It may not be PPPoE interface. It may be PPTP or L2TP. In that case,
		 * we shouldn't set the is_return_pppoe flag.
		 */
		if (!ppp_get_session_id((struct net_device *)out)) {
			goto out;
		}

		is_return_pppoe = true;
		new_out = eth_out;
#else
		goto out;
#endif
	}

	/*
	 * Only work with standard 802.3 mac address sizes
	 */
	if (new_out->addr_len != 6 && (new_out->type != ARPHRD_SIT) && (new_out->type != ARPHRD_TUNNEL6)) {
		NSS_CONNMGR_DEBUG_TRACE("out device (%s) not 802.3 hw addr len (%u), ignoring: %p\n", new_out->name, (unsigned)new_out->addr_len, skb);
		goto out;
	}

	/*
	 * Now examine conntrack to identify the protocol, IP addresses and portal information involved
	 * IMPORTANT: The information here will be as the 'ORIGINAL' direction, i.e. who established the connection.
	 * This MAY NOT be the same as the current packet direction, for example, this packet direction may be eth1->eth0 but
	 * originally the connection may be been started from a packet going from eth0->eth1.
	 */
	orig_tuple = ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple;
	reply_tuple = ct->tuplehash[IP_CT_DIR_REPLY].tuple;
	unic.protocol = (int32_t)orig_tuple.dst.protonum;

	/*
	 * Get addressing information, non-NAT first
	 */
	unic.src_ip = (ipv4_addr_t)orig_tuple.src.u3.ip;
	unic.dest_ip = (ipv4_addr_t)orig_tuple.dst.u3.ip;

	/*
	 * NAT'ed addresses - note these are as seen from the 'reply' direction
	 * When NAT does not apply to this connection these will be identical to the above.
	 */
	unic.src_ip_xlate = (ipv4_addr_t)reply_tuple.dst.u3.ip;
	unic.dest_ip_xlate = (ipv4_addr_t)reply_tuple.src.u3.ip;

	unic.flags = 0;

	/*
	 * Store the skb->priority as the qos tag
	 */
	unic.qos_tag = (uint32_t)skb->priority;

	/*
	 * Always a routed path
	 */
	unic.flags |= NSS_IPV4_CREATE_FLAG_ROUTED;

	/*
	 * Set the PPPoE values to the defaults, just in case there is not any PPPoE connection.
	 */
	unic.return_pppoe_session_id = 0;
	unic.flow_pppoe_session_id = 0;
	memset(unic.return_pppoe_remote_mac, 0, ETH_ALEN);
	memset(unic.flow_pppoe_remote_mac, 0, ETH_ALEN);

	switch (unic.protocol) {
	case IPPROTO_TCP:
		unic.src_port = (int32_t)orig_tuple.src.u.tcp.port;
		unic.dest_port = (int32_t)orig_tuple.dst.u.tcp.port;
		unic.src_port_xlate = (int32_t)reply_tuple.dst.u.tcp.port;
		unic.dest_port_xlate = (int32_t)reply_tuple.src.u.tcp.port;
		unic.flow_window_scale = ct->proto.tcp.seen[0].td_scale;
		unic.flow_max_window = ct->proto.tcp.seen[0].td_maxwin;
		unic.flow_end = ct->proto.tcp.seen[0].td_end;
		unic.flow_max_end = ct->proto.tcp.seen[0].td_maxend;
		unic.return_window_scale = ct->proto.tcp.seen[1].td_scale;
		unic.return_max_window = ct->proto.tcp.seen[1].td_maxwin;
		unic.return_end = ct->proto.tcp.seen[1].td_end;
		unic.return_max_end = ct->proto.tcp.seen[1].td_maxend;
		if ( nf_ct_tcp_be_liberal || nf_ct_tcp_no_window_check ||
			(ct->proto.tcp.seen[0].flags & IP_CT_TCP_FLAG_BE_LIBERAL) ||
			(ct->proto.tcp.seen[1].flags & IP_CT_TCP_FLAG_BE_LIBERAL)) {

			unic.flags |= NSS_IPV4_CREATE_FLAG_NO_SEQ_CHECK;
		}

		/*
		 * Don't try to manage a non-established connection.
		 */
		if (!test_bit(IPS_ASSURED_BIT, &ct->status)) {
			NSS_CONNMGR_DEBUG_TRACE("%p: Non-established connection\n", ct);
			goto out;
		}

		/*
		 * If the connection is shutting down do not manage it.
		 * state can not be SYN_SENT, SYN_RECV because connection is assured
		 * Not managed states: FIN_WAIT, CLOSE_WAIT, LAST_ACK, TIME_WAIT, CLOSE.
		 */
		spin_lock_bh(&ct->lock);
		if (ct->proto.tcp.state != TCP_CONNTRACK_ESTABLISHED) {
			spin_unlock_bh(&ct->lock);
			NSS_CONNMGR_DEBUG_TRACE("%p: Connection in termination state %#X\n", ct, ct->proto.tcp.state);
			goto out;
		}
		spin_unlock_bh(&ct->lock);

		break;

	case IPPROTO_UDP:
		unic.src_port = (int32_t)orig_tuple.src.u.udp.port;
		unic.dest_port = (int32_t)orig_tuple.dst.u.udp.port;
		unic.src_port_xlate = (int32_t)reply_tuple.dst.u.udp.port;
		unic.dest_port_xlate = (int32_t)reply_tuple.src.u.udp.port;
		usk = udp_sk(sk);
		break;

	case IPPROTO_IPV6:
		unic.src_port = 0;
		unic.dest_port = 0;
		unic.src_port_xlate = 0;
		unic.dest_port_xlate = 0;
		break;

	case IPPROTO_ESP:
		unic.src_port = 0;
		unic.dest_port = 0;
		unic.src_port_xlate = 0;
		unic.dest_port_xlate = 0;
		break;

	default:
		/*
		 * Streamengine compatibility - database stores non-ported protocols with port numbers equal to negative protocol number
		 * to indicate unused.
		 */
		NSS_CONNMGR_DEBUG_TRACE("%p: Unhandled protocol %d\n", ct, unic.protocol);
		goto out;
	}

	/* Handle L2TP UDP encapsulated frames */
	if (usk) {
		if (usk->encap_type == UDP_ENCAP_L2TPINUDP) {
			NSS_CONNMGR_DEBUG_TRACE("%p: skip packets for L2TP tunnel\n", ct);
			goto out;
		}
	}

	/*
	 * Initialize VLAN tag information
	 */
	unic.ingress_vlan_tag = NSS_CONNMGR_VLAN_ID_NOT_CONFIGURED;
	unic.egress_vlan_tag = NSS_CONNMGR_VLAN_ID_NOT_CONFIGURED;
	/*
	 * Initialize DSCP and VLAN MARKING information
	 */
	unic.dscp_itag =  NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED;
	unic.dscp_imask =  NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED;
	unic.dscp_omask =  NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED;
	unic.dscp_oval =  NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED;
	unic.vlan_itag =  NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED;
	unic.vlan_imask=  NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED;
	unic.vlan_omask =  NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED;
	unic.vlan_oval =  NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED;

	/*
	 * Set the source and destination interfaces, and the original source
	 * IP address before any NAT translation.
	 * NOTE: We are dealing with the ORIGINAL direction here so 'in' and 'out' dev may need
	 * to be swapped if this packet is a reply
	 */
	if (ctinfo < IP_CT_IS_REPLY) {
		src_dev = in;
		dest_dev = new_out;
		src_ipaddr = (ipv4_addr_t)orig_tuple.src.u3.ip;
		NSS_CONNMGR_DEBUG_TRACE("%p: dir: Original\n", ct);
	} else {
		src_dev = new_out;
		dest_dev = in;
		src_ipaddr = (ipv4_addr_t)reply_tuple.src.u3.ip;
		NSS_CONNMGR_DEBUG_TRACE("%p: dir: Reply\n", ct);

		/*
		 * Swap the is_pppoe_xxx flags.
		 */
		is_tmp_pppoe = is_flow_pppoe;
		is_flow_pppoe = is_return_pppoe;
		is_return_pppoe = is_tmp_pppoe;

	}

	/*
	 * Get the MAC addresses that correspond to source and destination host addresses.
	 * NOTE: XXX We cannot find the MAC addresses of the hosts which are behind the PPPoE servers. So,
	 * we are just copying the PPPoE server MAC addresses here as a src or dest MAC addresses. Then, we will
	 * send the packets to the PPPoE server and it will handle the packet.
	 */
	if (unlikely(is_flow_pppoe)) {
		ppp_src = ppp_get_ppp_netdev(src_dev);
		if (!ppp_src) {
			NSS_CONNMGR_DEBUG_TRACE("%p: ppp_src is NULL\n", ct);
			goto out;
		}

		memcpy(unic.src_mac, (uint8_t *)ppp_get_remote_mac(ppp_src), ETH_ALEN);
		memcpy(unic.src_mac_xlate, (uint8_t *)ppp_get_remote_mac(ppp_src), ETH_ALEN);
		unic.flow_pppoe_session_id = (uint16_t)ppp_get_session_id(ppp_src);
		memcpy(unic.flow_pppoe_remote_mac, (uint8_t *)ppp_get_remote_mac(ppp_src), ETH_ALEN);
	} else {
		if (nss_connmgr_ipv4_mac_addr_get(unic.src_ip, unic.src_mac)) {
			NSS_CONNMGR_DEBUG_TRACE("%p: Failed to find MAC address for src IP: %pI4\n", ct, &unic.src_ip);
			goto out;
		}

		if (nss_connmgr_ipv4_mac_addr_get(unic.src_ip_xlate, unic.src_mac_xlate)) {
			NSS_CONNMGR_DEBUG_TRACE("%p: Failed to find MAC address for xlate src IP: %pI4\n", ct, &unic.src_ip_xlate);
			goto out;
		}
	}

	/*
	 * Do dest now
	 */
	if (unlikely(is_return_pppoe)) {
		ppp_dest =  ppp_get_ppp_netdev(dest_dev);
		if (!ppp_dest) {
			NSS_CONNMGR_DEBUG_TRACE("%p: ppp_dest is NULL\n", ct);
			goto out;
		}

		memcpy(unic.dest_mac, (uint8_t *)ppp_get_remote_mac(ppp_dest), ETH_ALEN);
		memcpy(unic.dest_mac_xlate, (uint8_t *)ppp_get_remote_mac(ppp_dest), ETH_ALEN);
		unic.return_pppoe_session_id = (uint16_t)ppp_get_session_id(ppp_dest);
		memcpy(unic.return_pppoe_remote_mac, (uint8_t *)ppp_get_remote_mac(ppp_dest), ETH_ALEN);
	} else {
		if (nss_connmgr_ipv4_mac_addr_get(unic.dest_ip, unic.dest_mac)) {
			NSS_CONNMGR_DEBUG_TRACE("%p: Failed to find MAC address for dest IP: %pI4\n", ct, &unic.dest_ip);
			goto out;
		}

		if (nss_connmgr_ipv4_mac_addr_get(unic.dest_ip_xlate, unic.dest_mac_xlate)) {
			NSS_CONNMGR_DEBUG_TRACE("%p: Failed to find MAC address for xlate dest IP: %pI4\n", ct, &unic.dest_ip_xlate);
			goto out;
		}
	}

	/*
	 * Access the ingress routed interface
	 */
	rt_dev = nss_connmgr_get_dev_from_ip_address(src_ipaddr);
	if (!rt_dev) {
		/*
		 * This should not happen
		 */
		goto out;
	}

	/*
	 * The ingress-routed interface is a virtual interface, which could possibly be a VLAN interface
	 */
	if (rt_dev != in) {
		/*
		 * The ingress is a VLAN interface
		 */
		if (is_vlan_dev(rt_dev)) {
			if (ctinfo < IP_CT_IS_REPLY) {
				unic.ingress_vlan_tag = vlan_dev_priv(rt_dev)->vlan_id;
				NSS_CONNMGR_DEBUG_INFO("Route-CM: Ingress VLAN ID = %d\n",vlan_dev_priv(rt_dev)->vlan_id);
			} else {
				unic.egress_vlan_tag = vlan_dev_priv(rt_dev)->vlan_id;
				NSS_CONNMGR_DEBUG_INFO("Route-CM: Egress VLAN ID = %d\n",vlan_dev_priv(rt_dev)->vlan_id);
			}
		} else if (is_bridge_device(rt_dev)) {
			/*
			 * Get the slave bridge port which is the ingress interface
			 * for this packet. Ensure MAC header is in place, which
			 * may be sripped off in some cases, for example, for partially
			 * accelerated IPv6 tunnel exception packets.
			 */
			if (skb_mac_header_was_set(skb)) {
				br_port_dev = br_port_dev_get(rt_dev, eth_hdr(skb)->h_source);
			} else {
				goto out;
			}

			if (br_port_dev) {
				/*
				 * Is the ingress bridge port a VLAN interface?
				 */
				if (is_vlan_dev(br_port_dev)) {
					if (ctinfo < IP_CT_IS_REPLY) {
						unic.ingress_vlan_tag = vlan_dev_priv(br_port_dev)->vlan_id;
						NSS_CONNMGR_DEBUG_INFO("Route-CM: Ingress VLAN ID = %d\n",vlan_dev_priv(br_port_dev)->vlan_id);
					} else {
						unic.egress_vlan_tag = vlan_dev_priv(br_port_dev)->vlan_id;
						NSS_CONNMGR_DEBUG_INFO("Route-CM: Egress VLAN ID = %d\n",vlan_dev_priv(br_port_dev)->vlan_id);
					}
				} else {
					NSS_CONNMGR_DEBUG_INFO("Route-CM: Ingress Bridge Physical device name: %s\n", br_port_dev->name);
				}
				dev_put(br_port_dev);
			} else {
				NSS_CONNMGR_DEBUG_INFO("Route-CM: Could not get slave interface from bridge device %s\n",rt_dev->name);
				goto out;
			}
		}
	}

	/*
	 * Check if the egress interface is a VLAN interface
	 */
	if (is_vlan_dev(new_out)) {
		if (ctinfo < IP_CT_IS_REPLY) {
			unic.egress_vlan_tag = vlan_dev_priv(new_out)->vlan_id;
			dest_dev = vlan_dev_priv(new_out)->real_dev;
			NSS_CONNMGR_DEBUG_INFO("Route-CM: Egress VLAN ID = %d\n",vlan_dev_priv(new_out)->vlan_id);
		} else {
			unic.ingress_vlan_tag = vlan_dev_priv(new_out)->vlan_id;
			src_dev = vlan_dev_priv(new_out)->real_dev;
			NSS_CONNMGR_DEBUG_INFO("Route-CM: Ingress VLAN ID = %d\n",vlan_dev_priv(new_out)->vlan_id);
		}
	}

	iph = ip_hdr(skb);

	/*
	 * Handle Link Aggregation
	 */
	if (ctinfo < IP_CT_IS_REPLY) {
		/*
		 * If ingress interface is a lag slave, we cannot assume that
		 * reply packets will use this interface for egress; call
		 * bonding driver API to find the egress interface
		 * for reply packets.
		 */
		if (is_lag_slave(src_dev)) {
			src_slave = bond_get_tx_dev(NULL,
						    (uint8_t *)src_dev->master->dev_addr,
						    unic.src_mac,
						    (void *)&unic.dest_ip_xlate,
						    (void *)&unic.src_ip_xlate,
						    skb->protocol,
						    src_dev->master);
			if (src_slave == NULL
			    || !netif_carrier_ok(src_slave)) {
				dev_put(in);
				return NF_ACCEPT;
			}

			src_dev = src_slave;
		}

		/*
		 * Determine egress LAG slave interface
		 * for packet in original direction.
		 */
		if (is_lag_master(dest_dev)) {
			dest_slave = bond_get_tx_dev(NULL,
						     (uint8_t *)dest_dev->dev_addr,
						     unic.dest_mac_xlate,
						     (void *)&unic.src_ip_xlate,
						     (void *)&unic.dest_ip_xlate,
						     skb->protocol, dest_dev);
			if (dest_slave == NULL
			    || !netif_carrier_ok(dest_slave)) {
				goto out;
			}

			dest_dev = dest_slave;
		}
	} else {
		/*
		 * At this point, src_dev points to ingress IF and and dest_dev
		 * points to egress 'physical' IF, both in original direction.
		 */

		/*
		 * If, in original direction, the ingress [virtual] IF was a
		 * LAG master, then we must determine the egress LAG slave for
		 * reply packets (such as the current packet).
		 */
		if (is_lag_master(src_dev)) {
			src_slave = bond_get_tx_dev(NULL,
						    (uint8_t *)src_dev->dev_addr,
						    unic.src_mac,
						    (void *)&unic.dest_ip_xlate,
						    (void *)&unic.src_ip_xlate,
						    skb->protocol, src_dev);
			if (src_slave == NULL
			    || !netif_carrier_ok(src_slave)) {
				dev_put(in);
				return NF_ACCEPT;
			}
			src_dev = src_slave;
		}

		/*
		 * If, in original direction, the egress IF was a LAG slave,
		 * then we must determine the correct egress LAG slave, for
		 * packets in original direction.
		 */
		if (is_lag_slave(dest_dev)) {
			dest_slave = bond_get_tx_dev(NULL,
						     (uint8_t *)dest_dev->master->dev_addr,
						     unic.dest_mac_xlate,
						     (void *)&unic.src_ip_xlate,
						     (void *)&unic.dest_ip_xlate,
						     skb->protocol,
						     dest_dev->master);
			if (dest_slave == NULL
			    || !netif_carrier_ok(dest_slave)) {
				dev_put(in);
				return NF_ACCEPT;
			}

			dest_dev = dest_slave;
		}
	}

	/*
	 * Only devices that are NSS devices, or IPSec tunnel devices are accelerated.
	 */
	if ((src_dev->type == ARPHRD_IPSEC_TUNNEL_TYPE) ||
		((unic.protocol == IPPROTO_ESP) && (ctinfo < IP_CT_IS_REPLY))) {
		unic.src_interface_num = NSS_C2C_TX_INTERFACE;
	} else {
		unic.src_interface_num = nss_cmn_get_interface_number(nss_connmgr_ipv4.nss_context, src_dev);
		if (unic.src_interface_num < 0) {
			goto out;
		}
	}

	if ((dest_dev->type == ARPHRD_IPSEC_TUNNEL_TYPE) ||
		((unic.protocol == IPPROTO_ESP) && (ctinfo >= IP_CT_IS_REPLY))) {
		unic.dest_interface_num = NSS_C2C_TX_INTERFACE;
	} else {

		unic.dest_interface_num = nss_cmn_get_interface_number(nss_connmgr_ipv4.nss_context, dest_dev);
		if (unic.dest_interface_num < 0) {
			goto out;
		}
	}

	/*
	 * Create the Network Accelerator connection cache entries
	 *
	 * NOTE: All of the information we have is from the point of view of who created the connection however
	 * the skb may actually be in the 'reply' direction - which is important to know when configuring the NSS as we have to set the right information
	 * on the right "match" and "forwarding" entries.
	 * We can use the ctinfo to determine which direction the skb is in and then swap fields as necessary.
	 */
	unic.src_ip = ntohl(unic.src_ip);
	unic.dest_ip = ntohl(unic.dest_ip);
	unic.src_ip_xlate = ntohl(unic.src_ip_xlate);
	unic.dest_ip_xlate = ntohl(unic.dest_ip_xlate);
	unic.src_port = ntohs(unic.src_port);
	unic.dest_port = ntohs(unic.dest_port);
	unic.src_port_xlate = ntohs(unic.src_port_xlate);
	unic.dest_port_xlate = ntohs(unic.dest_port_xlate);
	unic.flow_pppoe_session_id = ntohs(unic.flow_pppoe_session_id);
	unic.return_pppoe_session_id = ntohs(unic.return_pppoe_session_id);

	/*
	 * Get MTU values for source and destination interfaces.
	 */
	unic.from_mtu = in->mtu;
	unic.to_mtu = out->mtu;

	/*
	 * We have everything we need (hopefully :-])
	 */
	NSS_CONNMGR_DEBUG_TRACE("\n%p: Conntrack connection\n"
			"skb: %p\n"
			"dir: %s\n"
			"Protocol: %d\n"
			"src_ip: " IPV4_ADDR_FMT ":%d\n"
			"dest_ip: " IPV4_ADDR_FMT ":%d\n"
			"src_ip_xlate: " IPV4_ADDR_FMT ":%d\n"
			"dest_ip_xlate: " IPV4_ADDR_FMT ":%d\n"
			"src_mac: " MAC_FMT "\n"
			"dest_mac: " MAC_FMT "\n"
			"src_mac_xlate: " MAC_FMT "\n"
			"dest_mac_xlate: " MAC_FMT "\n"
			"src_dev: %s\n"
			"dest_dev: %s\n"
			"src_iface_num: %u\n"
			"dest_iface_num: %u\n"
			"ingress_vlan_tag: %u\n"
			"egress_vlan_tag: %u\n"
			"flow_pppoe_session_id: %u\n"
			"return_pppoe_session_id: %u\n"
			"qos_tag: %u\n",
			ct,
			skb,
			(ctinfo < IP_CT_IS_REPLY)? "Original" : "Reply",
			unic.protocol,
			IPV4_ADDR_TO_QUAD(unic.src_ip), unic.src_port,
			IPV4_ADDR_TO_QUAD(unic.dest_ip), unic.dest_port,
			IPV4_ADDR_TO_QUAD(unic.src_ip_xlate), unic.src_port_xlate,
			IPV4_ADDR_TO_QUAD(unic.dest_ip_xlate), unic.dest_port_xlate,
			MAC_AS_BYTES(unic.src_mac),
			MAC_AS_BYTES(unic.dest_mac),
			MAC_AS_BYTES(unic.src_mac_xlate),
			MAC_AS_BYTES(unic.dest_mac_xlate),
			src_dev->name,
			dest_dev->name,
			unic.src_interface_num,
			unic.dest_interface_num,
			unic.ingress_vlan_tag,
			unic.egress_vlan_tag,
			unic.flow_pppoe_session_id,
			unic.return_pppoe_session_id,
			unic.qos_tag);

	if (!offload_dscpremark_get_target_info(ct, &unic.dscp_imask, &unic.dscp_itag, &unic.dscp_omask, &unic.dscp_oval)) {
		NSS_CONNMGR_DEBUG_INFO("DSCP remark information is not present\n");
	}

	if (!offload_vlantag_get_target_info(ct, &unic.vlan_imask, &unic.vlan_itag, &unic.vlan_omask, &unic.vlan_oval)) {
		NSS_CONNMGR_DEBUG_INFO("VLAN tagging information is not present\n");
	}

	/*
	 * Setting the appropriate flags dfoe DSCP and VLAN marking.
	 * API's to set the data for remarking should be called before this.
	 */
	if(unic.dscp_oval != NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED)
		unic.flags |= NSS_IPV4_CREATE_FLAG_DSCP_MARKING;
	if(unic.vlan_oval != NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED)
		unic.flags |= NSS_IPV4_CREATE_FLAG_VLAN_MARKING;

	/*
	 * If operations have stopped then do not proceed further
	 */
	spin_lock_bh(&nss_connmgr_ipv4.lock);
	if (unlikely(nss_connmgr_ipv4.stopped)) {
		spin_unlock_bh(&nss_connmgr_ipv4.lock);
		NSS_CONNMGR_DEBUG_TRACE("Stopped, ignoring: %p\n", skb);
		goto out;
	}
	spin_unlock_bh(&nss_connmgr_ipv4.lock);

	/*
	 * Calling ipv4_rule1 for DSCP and VLAN marking else we use the regular
	 * ipv4_rule
	 */
	if(unic.dscp_oval == NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED &&
		unic.vlan_oval == NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED) {
		nss_tx_status = nss_tx_create_ipv4_rule(nss_connmgr_ipv4.nss_context, &unic);
	} else {
		nss_tx_status = nss_tx_create_ipv4_rule1(nss_connmgr_ipv4.nss_context, &unic);
	}

	if (nss_tx_status == NSS_TX_SUCCESS) {
		goto out;
	} else if (nss_tx_status == NSS_TX_FAILURE_NOT_READY) {
		NSS_CONNMGR_DEBUG_ERROR("NSS not ready to accept rule \n");

		spin_lock_bh(&nss_connmgr_ipv4.lock);
		nss_connmgr_ipv4.debug_stats[NSS_CONNMGR_IPV4_CREATE_FAIL]++;
		spin_unlock_bh(&nss_connmgr_ipv4.lock);
	} else {
		NSS_CONNMGR_DEBUG_TRACE("NSS create rule failed  skb: %p\n", skb);

		spin_lock_bh(&nss_connmgr_ipv4.lock);
		nss_connmgr_ipv4.debug_stats[NSS_CONNMGR_IPV4_CREATE_FAIL]++;
		spin_unlock_bh(&nss_connmgr_ipv4.lock);
	}

out:
	/*
	 * Release the interface on which this skb arrived
	 */
	dev_put(in);

	if (ppp_in) {
		dev_put(ppp_in);
	}

	if (eth_out) {
		dev_put(eth_out);
	}

	if (ppp_src) {
		dev_put(ppp_src);
	}

	if (ppp_dest) {
		dev_put(ppp_dest);
	}

	return NF_ACCEPT;
}

/*
 * nss_connmgr_ipv4_update_bridge_dev()
 *	Update bridge device at host with packet statistics and refresh bridge MAC entries
 */
void nss_connmgr_ipv4_update_bridge_dev(struct nss_connmgr_ipv4_connection *connection, struct nss_ipv4_sync *sync, int final_sync)
{
	struct net_device *indev, *outdev;
	struct rtnl_link_stats64 stats;

	indev = nss_cmn_get_interface_dev(nss_connmgr_ipv4.nss_context, connection->src_interface);
	if (indev == NULL) {
		/*
		 * Possible sync for a deleted interface
		 */
		return;
	}

	if (is_lag_slave(indev)) {
		if (indev->master == NULL) {
			NSS_CONNMGR_DEBUG_TRACE("Could not find master for LAG slave %s\n", indev->name);
			return;
		}
		indev = indev->master;
	} else if (connection->ingress_vlan_tag != NSS_CONNMGR_VLAN_ID_NOT_CONFIGURED) {
		indev = __vlan_find_dev_deep(indev, connection->ingress_vlan_tag);
		if (indev == NULL) {
			/*
			 * Possible sync for a deleted VLAN interface
			 */
			return;
		}
	}

	outdev = nss_cmn_get_interface_dev(nss_connmgr_ipv4.nss_context, connection->dest_interface);
	if (outdev == NULL) {
		/*
		 * Possible sync for a deleted interface
		 */
		return;
	}

	if (is_lag_slave(outdev)) {
		if (outdev->master == NULL) {
			NSS_CONNMGR_DEBUG_TRACE("Could not find master for LAG slave %s\n", outdev->name);
			return;
		}
		outdev = outdev->master;
	} else if (connection->egress_vlan_tag != NSS_CONNMGR_VLAN_ID_NOT_CONFIGURED) {
		outdev = __vlan_find_dev_deep(outdev, connection->egress_vlan_tag);
		if (outdev == NULL) {
			/*
			 * Possible sync for a deleted VLAN interface
			 */
			return;
		}
	}

	if (is_bridge_port(indev))
	{
		/*
		 * Refresh bridge MAC table if necessary
		 */
		if (!final_sync && sync->flow_rx_packet_count) {
			br_refresh_fdb_entry(indev, connection->src_mac_addr);
		}

		/*
		 * Update bridge interface stats for L3 flows involving a bridge
		 */
		if (indev->master != outdev->master) {
			stats.rx_packets = sync->flow_rx_packet_count;
			stats.rx_bytes = sync->flow_rx_byte_count;
			stats.tx_packets = sync->flow_tx_packet_count;
			stats.tx_bytes = sync->flow_tx_byte_count;
			br_dev_update_stats(indev->master, &stats);
		}
	}

	if (is_bridge_port(outdev))
	{
		/*
		 * Refresh bridge MAC table if necessary
		 */
		if (!final_sync && sync->return_rx_packet_count) {
			br_refresh_fdb_entry(outdev, connection->dest_mac_addr);
		}

		/*
		 * Update bridge interface stats for L3 flows involving a bridge
		 */
		if (indev->master != outdev->master) {
			stats.rx_packets = sync->return_rx_packet_count;
			stats.rx_bytes = sync->return_rx_byte_count;
			stats.tx_packets = sync->return_tx_packet_count;
			stats.tx_bytes = sync->return_tx_byte_count;
			br_dev_update_stats(outdev->master, &stats);
		}
	}
}

/*
 * nss_connmgr_ipv4_update_vlan_dev_stats()
 *	Update VLAN device statistics
 */
void nss_connmgr_ipv4_update_vlan_dev_stats(struct nss_connmgr_ipv4_connection *connection, struct nss_ipv4_sync *sync)
{
	struct net_device *vlandev, *physdev;
	struct rtnl_link_stats64 stats;

	if (connection->ingress_vlan_tag != NSS_CONNMGR_VLAN_ID_NOT_CONFIGURED)
	{
		physdev = nss_cmn_get_interface_dev(nss_connmgr_ipv4.nss_context, connection->src_interface);
		if (unlikely(!physdev)) {
			NSS_CONNMGR_DEBUG_WARN("Invalid src dev reference from NSS \n");
			return;
		}
		rcu_read_lock();
		vlandev = __vlan_find_dev_deep(physdev, connection->ingress_vlan_tag);
		rcu_read_unlock();

		if (vlandev) {
			stats.rx_packets = sync->flow_rx_packet_count;
			stats.rx_bytes = sync->flow_rx_byte_count;
			stats.tx_packets = sync->flow_tx_packet_count;
			stats.tx_bytes = sync->flow_tx_byte_count;
			__vlan_dev_update_accel_stats(vlandev, &stats);
		} else {
			NSS_CONNMGR_DEBUG_WARN("Could not find VLAN IN device for ingress vlan %d\n", connection->ingress_vlan_tag);
			return;
		}
	}

	if (connection->egress_vlan_tag != NSS_CONNMGR_VLAN_ID_NOT_CONFIGURED)
	{
		physdev = nss_cmn_get_interface_dev(nss_connmgr_ipv4.nss_context, connection->dest_interface);
		if (unlikely(!physdev)) {
			NSS_CONNMGR_DEBUG_WARN("Invalid dest dev reference from NSS \n");
			return;
		}
		rcu_read_lock();
		vlandev = __vlan_find_dev_deep(physdev, connection->egress_vlan_tag);
		rcu_read_unlock();

		if (vlandev) {
			stats.rx_packets = sync->return_rx_packet_count;
			stats.rx_bytes = sync->return_rx_byte_count;
			stats.tx_packets = sync->return_tx_packet_count;
			stats.tx_bytes = sync->return_tx_byte_count;
			__vlan_dev_update_accel_stats(vlandev, &stats);
		} else {
			NSS_CONNMGR_DEBUG_WARN("Could not find VLAN OUT device for egress vlan %d\n", connection->egress_vlan_tag);
			return;
		}
	}
}

/*
 * nss_connmgr_ipv4_net_dev_callback()
 *	Callback handler from the linux device driver for the Network Accelerator.
 *
 * The NSS passes sync and stats information to the device driver - which then passes them
 * onto this callback. This arrangement means that dependencies on driver and conntrack modules
 * are solely in this module. * These callbacks aim to keep conntrack connections alive and to ensure that connection
 * state - especially for TCP connections that track sequence space for example - is kept in synchronisation
 * before packets flow back through linux from a period of NSS processing.
 */
static void nss_connmgr_ipv4_net_dev_callback(struct nss_ipv4_cb_params *nicp)
{
	struct nf_conntrack_tuple_hash *h;
	struct nf_conntrack_tuple tuple;
	struct nf_conn *ct;
	struct nf_conn_counter *acct;
	struct nss_connmgr_ipv4_connection *connection;

	struct nss_ipv4_sync *sync;
	struct nss_ipv4_establish *establish;

	struct neighbour *neigh;
	struct net_device *flow_dev;
	struct net_device *return_dev;
	uint32_t final_sync = 0;

	switch (nicp->reason)
	{
		case NSS_IPV4_CB_REASON_ESTABLISH:
			establish = &nicp->params.establish;

			if (unlikely(establish->index >= NSS_CONNMGR_IPV4_CONN_MAX)) {
				NSS_CONNMGR_DEBUG_WARN("Bad establish index: %d\n", establish->index);
				return;
			}

			/*
			 * Connection manager uses the same index value as NSS , for the connection table.
			 * The index is sent by NSS as part of Establish (and sync) packets.
			 */
			connection = &nss_connmgr_ipv4.connection[establish->index];

			NSS_CONNMGR_DEBUG_TRACE("NSS IPv4 Establish callback %d \n", establish->index);

			spin_lock_bh(&nss_connmgr_ipv4.lock);

			/*
			 * TODO Capture the condition state == ESTABLISHED in a variable,
			 * and use the variable to print the error message later to avoid spin unlock/lock
			 */
			if (unlikely(connection->state == NSS_CONNMGR_IPV4_STATE_ESTABLISHED)) {
				spin_unlock_bh(&nss_connmgr_ipv4.lock);

				NSS_CONNMGR_DEBUG_TRACE("Invalid establish callback. Conn already established : %d \n", establish->index);
				return;

			}

			connection->state = NSS_CONNMGR_IPV4_STATE_ESTABLISHED;
			nss_connmgr_ipv4.debug_stats[NSS_CONNMGR_IPV4_ACTIVE_CONN]++;

			connection->protocol = establish->protocol;
			connection->src_interface = establish->flow_interface;
			connection->src_addr = establish->flow_ip;
			connection->src_port = establish->flow_ident;
			connection->src_addr_xlate = establish->flow_ip_xlate;
			connection->src_port_xlate = establish->flow_ident_xlate;

			connection->dest_interface = establish->return_interface;
			connection->dest_addr = establish->return_ip;
			connection->dest_port = establish->return_ident;
			connection->dest_addr_xlate = establish->return_ip_xlate;
			connection->dest_port_xlate = establish->return_ident_xlate;
			connection->ingress_vlan_tag = establish->ingress_vlan_tag;
			connection->egress_vlan_tag = establish->egress_vlan_tag;

			memcpy(connection->src_mac_addr, (char *)establish->flow_mac, ETH_ALEN);
			memcpy(connection->dest_mac_addr, (char *)establish->return_mac, ETH_ALEN);

			memset(connection->stats, 0, (8*NSS_CONNMGR_IPV4_STATS_MAX));

			spin_unlock_bh(&nss_connmgr_ipv4.lock);

			return;

		case NSS_IPV4_CB_REASON_SYNC:
			sync = &nicp->params.sync;

			if (unlikely(sync->index >= NSS_CONNMGR_IPV4_CONN_MAX)) {
				NSS_CONNMGR_DEBUG_WARN("Bad sync index: %d\n", sync->index);
				return;
			}

			connection = &nss_connmgr_ipv4.connection[sync->index];

			NSS_CONNMGR_DEBUG_TRACE("NSS IPv4 Sync callback %d %d \n", sync->index, sync->reason);

			spin_lock_bh(&nss_connmgr_ipv4.lock);
			/*
			 * TODO Capture the condition state != ESTABLISHED in a variable,
			 * and use the variable to print the error message later to avoid spin unlock/lock
			 */
			if (unlikely(connection->state != NSS_CONNMGR_IPV4_STATE_ESTABLISHED)) {
				spin_unlock_bh(&nss_connmgr_ipv4.lock);
				NSS_CONNMGR_DEBUG_WARN("Invalid sync callback. Conn not established : %d \n", sync->index);
				return;
			}

			spin_unlock_bh(&nss_connmgr_ipv4.lock);

			switch (sync->reason) {

				case NSS_IPV4_SYNC_REASON_FLUSH:
				case NSS_IPV4_SYNC_REASON_EVICT:
				case NSS_IPV4_SYNC_REASON_DESTROY:
					spin_lock_bh(&nss_connmgr_ipv4.lock);
					connection->state = NSS_CONNMGR_IPV4_STATE_INACTIVE;
					nss_connmgr_ipv4.debug_stats[NSS_CONNMGR_IPV4_ACTIVE_CONN]--;
					spin_unlock_bh(&nss_connmgr_ipv4.lock);
					final_sync = 1;
					/* Fall through to increment stats */

				case NSS_IPV4_SYNC_REASON_STATS:
					spin_lock_bh(&nss_connmgr_ipv4.lock);
					connection->stats[NSS_CONNMGR_IPV4_ACCELERATED_RX_PKTS] += sync->flow_rx_packet_count + sync->return_rx_packet_count;
					connection->stats[NSS_CONNMGR_IPV4_ACCELERATED_RX_BYTES] += sync->flow_rx_byte_count + sync->return_rx_byte_count;
					connection->stats[NSS_CONNMGR_IPV4_ACCELERATED_TX_PKTS] += sync->flow_tx_packet_count + sync->return_tx_packet_count;
					connection->stats[NSS_CONNMGR_IPV4_ACCELERATED_TX_BYTES] += sync->flow_tx_byte_count + sync->return_tx_byte_count;

					/*
					 * Update VLAN device statistics if required
					 */
					if ((connection->ingress_vlan_tag & connection->egress_vlan_tag) != NSS_CONNMGR_VLAN_ID_NOT_CONFIGURED) {
						nss_connmgr_ipv4_update_vlan_dev_stats(connection, sync);
					}

					/*
					 * Update bridge device if required
					 */
					nss_connmgr_ipv4_update_bridge_dev(connection, sync, final_sync);

					spin_unlock_bh(&nss_connmgr_ipv4.lock);
					break;


				case NSS_IPV4_SYNC_REASON_PPPOE_DESTROY:
					break;

				default:
					NSS_CONNMGR_DEBUG_TRACE("%d: Invalid Sync Message\n", sync->reason);
					return;

			}
			break;

		default:
			NSS_CONNMGR_DEBUG_WARN("%d: Unhandled callback\n", nicp->reason);
			return;
	}

	/*
	 * Create a tuple so as to be able to look up a connection
	 */
	memset(&tuple, 0, sizeof(tuple));
	tuple.src.u3.ip = ntohl(connection->src_addr);
	tuple.src.u.all = ntohs((__be16)connection->src_port);
	tuple.src.l3num = AF_INET;

	tuple.dst.u3.ip = ntohl(connection->dest_addr);
	tuple.dst.dir = IP_CT_DIR_ORIGINAL;
	tuple.dst.protonum = (uint8_t)connection->protocol;
	tuple.dst.u.all = ntohs((__be16)connection->dest_port);

	NSS_CONNMGR_DEBUG_TRACE("\nNSS Update, lookup connection using\n"
			"Protocol: %d\n"
			"src_addr: %pI4:%d\n"
			"dest_addr: %pI4:%d\n",
			(int)tuple.dst.protonum,
			&(tuple.src.u3.ip), (int)tuple.src.u.all,
			&(tuple.dst.u3.ip), (int)tuple.dst.u.all);

	/*
	 * Look up conntrack connection
	 */
	h = nf_conntrack_find_get(&init_net, NF_CT_DEFAULT_ZONE, &tuple);
	if (!h) {
		NSS_CONNMGR_DEBUG_WARN("Conntrack not found for sync\n");
		return;
	}

	ct = nf_ct_tuplehash_to_ctrack(h);
	NF_CT_ASSERT(ct->timeout.data == (unsigned long)ct);

	/*
	 * Only update if this is not a fixed timeout
	 */
	if (!test_bit(IPS_FIXED_TIMEOUT_BIT, &ct->status)) {
		spin_lock_bh(&ct->lock);
		ct->timeout.expires += sync->delta_jiffies;
		spin_unlock_bh(&ct->lock);
	}

	acct = nf_conn_acct_find(ct);
	if (acct) {
		spin_lock_bh(&ct->lock);
		atomic64_add(sync->flow_rx_packet_count, &acct[IP_CT_DIR_ORIGINAL].packets);
		atomic64_add(sync->flow_rx_byte_count, &acct[IP_CT_DIR_ORIGINAL].bytes);

		atomic64_add(sync->return_rx_packet_count, &acct[IP_CT_DIR_REPLY].packets);
		atomic64_add(sync->return_rx_byte_count, &acct[IP_CT_DIR_REPLY].bytes);
		spin_unlock_bh(&ct->lock);
	}

	switch (connection->protocol) {
	case IPPROTO_TCP:
		spin_lock_bh(&ct->lock);
		if (ct->proto.tcp.seen[0].td_maxwin < sync->flow_max_window) {
			ct->proto.tcp.seen[0].td_maxwin = sync->flow_max_window;
		}
		if ((int32_t)(ct->proto.tcp.seen[0].td_end - sync->flow_end) < 0) {
			ct->proto.tcp.seen[0].td_end = sync->flow_end;
		}
		if ((int32_t)(ct->proto.tcp.seen[0].td_maxend - sync->flow_max_end) < 0) {
			ct->proto.tcp.seen[0].td_maxend = sync->flow_max_end;
		}
		if (ct->proto.tcp.seen[1].td_maxwin < sync->return_max_window) {
			ct->proto.tcp.seen[1].td_maxwin = sync->return_max_window;
		}
		if ((int32_t)(ct->proto.tcp.seen[1].td_end - sync->return_end) < 0) {
			ct->proto.tcp.seen[1].td_end = sync->return_end;
		}
		if ((int32_t)(ct->proto.tcp.seen[1].td_maxend - sync->return_max_end) < 0) {
			ct->proto.tcp.seen[1].td_maxend = sync->return_max_end;
		}
		spin_unlock_bh(&ct->lock);
		break;
	}

	flow_dev = nss_cmn_get_interface_dev(nss_connmgr_ipv4.nss_context, connection->src_interface);
	if (unlikely(!flow_dev)) {
		NSS_CONNMGR_DEBUG_WARN("Invalid src dev reference from NSS \n");
		goto out;
	}

	if ((flow_dev->flags & IFF_SLAVE) && (flow_dev->priv_flags & IFF_BONDING)) {
		flow_dev = flow_dev->master;
	}

	return_dev = nss_cmn_get_interface_dev(nss_connmgr_ipv4.nss_context, connection->dest_interface);
	if (unlikely(!return_dev)) {
		NSS_CONNMGR_DEBUG_WARN("Invalid dest dev reference from NSS \n");
		goto out;
	}

	if ((return_dev->flags & IFF_SLAVE) && (return_dev->priv_flags & IFF_BONDING)) {
		return_dev = return_dev->master;
	}

	/*
	 * Update the neighbour entry for source IP address
	 */
	neigh = nss_connmgr_ipv4_neigh_get(ntohl(connection->src_addr));
	if (neigh) {
		if (!final_sync) {
			neigh_update(neigh, NULL, neigh->nud_state, NEIGH_UPDATE_F_WEAK_OVERRIDE);
		}
		neigh_release(neigh);
	} else {
		NSS_CONNMGR_DEBUG_TRACE("Neighbour entry could not be found for onward flow\n");
	}

	/*
	 * Update the neighbour entry for destination IP address
	 */
	neigh = nss_connmgr_ipv4_neigh_get(ntohl(connection->dest_addr));
	if (neigh) {
		if (!final_sync) {
			neigh_update(neigh, NULL, neigh->nud_state, NEIGH_UPDATE_F_WEAK_OVERRIDE);
		}
		neigh_release(neigh);
	} else {
		NSS_CONNMGR_DEBUG_TRACE("Neighbour entry could not be found for return flow\n");
	}
out:
	/*
	 * Release connection
	 */
	nf_ct_put(ct);

	return;
}

#ifdef CONFIG_NF_CONNTRACK_EVENTS
/*
 * nss_connmgr_ipv4_conntrack_event()
 *	Callback event invoked when conntrack connection state changes, currently we handle destroy events to quickly release state
 */
#ifdef CONFIG_NF_CONNTRACK_CHAIN_EVENTS
static int nss_connmgr_ipv4_conntrack_event(struct notifier_block *this,
                                unsigned long events, void *ptr)
#else
static int nss_connmgr_ipv4_conntrack_event(unsigned int events, struct nf_ct_event *item)
#endif
{
#ifdef CONFIG_NF_CONNTRACK_CHAIN_EVENTS
	struct nf_ct_event *item = ptr;
#endif
	struct nss_ipv4_destroy unid;
	struct nf_conn *ct = item->ct;
	struct nf_conntrack_tuple orig_tuple;
	struct nf_conntrack_tuple reply_tuple;
#ifndef CONFIG_NF_CONNTRACK_CHAIN_EVENTS
	int (*event_cb) (struct nf_conn *ct);
#endif


	nss_tx_status_t nss_tx_status;

	/*
	 * Basic sanity checks on the event
	 */
	if (!ct) {
		NSS_CONNMGR_DEBUG_WARN("No ct in conntrack event callback\n");
		return NOTIFY_DONE;
	}
	if (ct == &nf_conntrack_untracked) {
		NSS_CONNMGR_DEBUG_TRACE("%p: ignoring untracked connn", ct);
		return NOTIFY_DONE;
	}

	/*
	 * Only interested in destroy events
	 */
	if (!(events & (1 << IPCT_DESTROY))) {
		return NOTIFY_DONE;
	}

	/*
	 * Invoke ipv6 callback, if registered
	 */
#ifndef CONFIG_NF_CONNTRACK_CHAIN_EVENTS
	event_cb = nss_connmgr_ipv4.conntrack_event_cb;
	if ((nf_ct_l3num(ct) == AF_INET6) && event_cb) {
		int ret = event_cb(ct);
		if (ret) {
			NSS_CONNMGR_DEBUG_WARN("conntrack_event_cb failed %d \n", ret);
		}
		return NOTIFY_DONE;
	}
#endif

	/*
	 * Only interested if this is IPv4
	 */
	if (nf_ct_l3num(ct) != AF_INET) {
		return NOTIFY_DONE;
	}

	/*
	 * Now examine conntrack to identify the protocol, IP addresses and portal information involved
	 * IMPORTANT: The information here will be as the 'ORIGINAL' direction, i.e. who established the connection.
	 * This MAY NOT be the same as the current packet direction, for example, this packet direction may be eth1->eth0 but
	 * originally the connection may be been started from a packet going from eth0->eth1.
	 */
	orig_tuple = ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple;
	reply_tuple = ct->tuplehash[IP_CT_DIR_REPLY].tuple;
	unid.protocol = (int32_t)orig_tuple.dst.protonum;

	/*
	 * Get addressing information, non-NAT first
	 */
	unid.src_ip = (ipv4_addr_t)orig_tuple.src.u3.ip;
	unid.dest_ip = (ipv4_addr_t)orig_tuple.dst.u3.ip;

	switch (unid.protocol) {
	case IPPROTO_TCP:
		unid.src_port = (int32_t)orig_tuple.src.u.tcp.port;
		unid.dest_port = (int32_t)orig_tuple.dst.u.tcp.port;
		break;

	case IPPROTO_UDP:
		unid.src_port = (int32_t)orig_tuple.src.u.udp.port;
		unid.dest_port = (int32_t)orig_tuple.dst.u.udp.port;
		break;

	case IPPROTO_IPV6:
		unid.src_port = 0;
		unid.dest_port = 0;
		break;

	case IPPROTO_ESP:
		unid.src_port = 0;
		unid.dest_port = 0;
		break;

	default:
		/*
		 * Streamengine compatibility - database stores non-ported protocols with port numbers equal to negative protocol number
		 * to indicate unused.
		 */
		NSS_CONNMGR_DEBUG_TRACE("%p: Unhandled protocol %d\n", ct, unid.protocol);
		unid.src_port = -unid.protocol;
		unid.dest_port = -unid.protocol;
	}

	/*
	 * Only deal with TCP or UDP or V6 over V4 Tunnel
	 */
	if ((unid.protocol != IPPROTO_TCP) && (unid.protocol != IPPROTO_UDP) &&
		(unid.protocol != IPPROTO_IPV6) && (unid.protocol != IPPROTO_ESP)) {
		return NOTIFY_DONE;
	}

	if (IS_LOCAL_HOST(ntohl(unid.src_ip))) {
		NSS_CONNMGR_DEBUG_TRACE("localhost packet ignored \n");
		return NOTIFY_DONE;
	}

	NSS_CONNMGR_DEBUG_TRACE("\n%p: Connection destroyed\n"
			"Protocol: %d\n"
			"src_ip: %pI4:%d\n"
			"dest_ip: %pI4:%d\n",
			ct,
			unid.protocol,
			&(unid.src_ip), unid.src_port,
			&(unid.dest_ip), unid.dest_port);

	unid.src_ip = ntohl(unid.src_ip);
	unid.dest_ip = ntohl(unid.dest_ip);
	unid.src_port = ntohs(unid.src_port);
	unid.dest_port = ntohs(unid.dest_port);

	/*
	 * Destroy the Network Accelerator connection cache entries.
	 */
	nss_tx_status = nss_tx_destroy_ipv4_rule(nss_connmgr_ipv4.nss_context, &unid);

	if (nss_tx_status == NSS_TX_SUCCESS) {
		goto out;
	} else if (nss_tx_status == NSS_TX_FAILURE_NOT_READY) {
		spin_lock_bh(&nss_connmgr_ipv4.lock);
		nss_connmgr_ipv4.debug_stats[NSS_CONNMGR_IPV4_DESTROY_FAIL]++;
		spin_unlock_bh(&nss_connmgr_ipv4.lock);
		NSS_CONNMGR_DEBUG_ERROR("NSS not ready to accept 'destroy IPv4' rule \n");
	} else {
		spin_lock_bh(&nss_connmgr_ipv4.lock);
		nss_connmgr_ipv4.debug_stats[NSS_CONNMGR_IPV4_DESTROY_FAIL]++;
		spin_unlock_bh(&nss_connmgr_ipv4.lock);
		NSS_CONNMGR_DEBUG_TRACE("NSS destroy rule fail \n");
	}
out:
	return NOTIFY_DONE;
}

#ifndef CONFIG_NF_CONNTRACK_CHAIN_EVENTS
/*
 * nss_connmgr_ipv4_register_conntrack_event_cb
 * 	Registers a callback for NF conntrack destoy event
 *
 * Linux netfilter conntrack module allows only one notifier to be registered for conntrack events.
 * since we are using up that slot, we are providing a callback mechanism for other clients (ipv6 connection mgr)
 * to register a notifier for conntrack events.
 *
 */
void nss_connmgr_ipv4_register_conntrack_event_cb(int (*event_cb)(struct nf_conn *))
{
	nss_connmgr_ipv4.conntrack_event_cb = event_cb;
	return;
}
EXPORT_SYMBOL(nss_connmgr_ipv4_register_conntrack_event_cb);

/*
 * nss_connmgr_ipv4_unregister_conntrack_event_cb
 * 	Unregisters callback for NF conntrack destoy event
 */
void nss_connmgr_ipv4_unregister_conntrack_event_cb(void)
{
	nss_connmgr_ipv4.conntrack_event_cb = NULL;
	return;
}
EXPORT_SYMBOL(nss_connmgr_ipv4_unregister_conntrack_event_cb);
#endif
#endif

/*
 * nss_connmgr_ipv4_register_bond_slave_linkup_cb
 * 	Registers callback for LAG slave linkup
 *	notification to IPv6 manager.
 */
void nss_connmgr_ipv4_register_bond_slave_linkup_cb(void (*event_cb) (struct net_device *))
{
	nss_connmgr_ipv4.bond_slave_linkup = event_cb;
}
EXPORT_SYMBOL(nss_connmgr_ipv4_register_bond_slave_linkup_cb);

/*
 * nss_connmgr_ipv4_unregister_bond_slave_linkup_cb
 * 	Unregisters callback for LAG slave linkup
 *	notification to IPv6 manager.
 */
void nss_connmgr_ipv4_unregister_bond_slave_linkup_cb(void)
{
	nss_connmgr_ipv4.bond_slave_linkup = NULL;
}
EXPORT_SYMBOL(nss_connmgr_ipv4_unregister_bond_slave_linkup_cb);

/*
 * nss_connmgr_do_bond_down
 * 	Go through list of interfaces on the system and delete
 *	rules using slave interfaces of this LAG master.
 *	Called with dev_base_lock held.
 */
static void nss_connmgr_do_bond_down(struct net *net,
				     struct net_device *bond_dev)
{
	struct net_device *dev = NULL;

	for_each_netdev(net, dev) {
		if (is_lag_slave(dev) && (dev->master == bond_dev)) {
			nss_connmgr_link_down(dev);
		}
	}
}

/*
 * nss_connmgr_bond_down
 * 	LAG master closed
 */
static void nss_connmgr_bond_down(struct net_device *bond_dev)
{
	struct net *net = NULL;

	net = dev_net(bond_dev);
	if (!net) {
		NSS_CONNMGR_DEBUG_WARN("Unable to get network namespace "
				       "for %s\n", bond_dev->name);
		return;
	}

	read_lock(&dev_base_lock);
	nss_connmgr_do_bond_down(net, bond_dev);
	read_unlock(&dev_base_lock);
}

/*
 * nss_connmgr_bridge_down
 * 	Bridge interface closed
 */
static void nss_connmgr_bridge_down(struct net_device *br_dev)
{
	struct net_device *dev = NULL;
	struct net *net = NULL;

	net = dev_net(br_dev);
	if (!net) {
		NSS_CONNMGR_DEBUG_WARN("Unable to get network namespace "
				       "for %s\n", br_dev->name);
		return;
	}

	read_lock(&dev_base_lock);
	for_each_netdev(net, dev) {
		if (is_bridge_port(dev) && (dev->master == br_dev)) {
			if (is_lag_master(dev)) {
				nss_connmgr_do_bond_down(net, dev);
			} else {
				nss_connmgr_link_down(dev);
			}
		}
	}
	read_unlock(&dev_base_lock);
}

/*
 * nss_connmgr_netdev_notifier_cb
 * 	Netdevice notifier callback
 */
static int nss_connmgr_netdev_notifier_cb(struct notifier_block *this,
					  unsigned long event, void *ptr)
{
	struct net_device *event_dev = (struct net_device *)ptr;

	switch (event) {
	case NETDEV_DOWN:
		if (is_lag_master(event_dev)) {
			nss_connmgr_bond_down(event_dev);
		} else if(is_bridge_device(event_dev)) {
			nss_connmgr_bridge_down(event_dev);
		} else {
			nss_connmgr_link_down(event_dev);
		}
		break;

	case NETDEV_CHANGE:
		if (!netif_carrier_ok(event_dev)) {
			nss_connmgr_link_down(event_dev);
		}
		break;

	case NETDEV_CHANGEMTU:
		nss_connmgr_link_down(event_dev);
		break;

	}

	return NOTIFY_DONE;
}


/*
 * nss_connmgr_ipv4_init_connection_table()
 *	initialize connection table
 */
static void nss_connmgr_ipv4_init_connection_table(void)
{
	int32_t i,j;

	spin_lock_bh(&nss_connmgr_ipv4.lock);
	for (i = 0; (i < NSS_CONNMGR_IPV4_CONN_MAX); i++) {
		nss_connmgr_ipv4.connection[i].state = NSS_CONNMGR_IPV4_STATE_INACTIVE;

		for (j = 0; (j < NSS_CONNMGR_IPV4_STATS_MAX); j++) {
			nss_connmgr_ipv4.connection[i].stats[j] = 0;
		}
	}

	for (i = 0; (i < NSS_CONNMGR_IPV4_DEBUG_STATS_MAX); i++) {
		nss_connmgr_ipv4.debug_stats[i] = 0;
	}

	spin_unlock_bh(&nss_connmgr_ipv4.lock);

	return;
}


/*
 * nss_connmgr_ipv4_get_terminate()
 */
static ssize_t nss_connmgr_ipv4_get_terminate(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	ssize_t count;
	int num;

	/*
	 * Operate under our locks
	 */
	spin_lock_bh(&nss_connmgr_ipv4.lock);
	num = nss_connmgr_ipv4.terminate;
	spin_unlock_bh(&nss_connmgr_ipv4.lock);

	count = snprintf(buf, (ssize_t)PAGE_SIZE, "%d\n", num);
	return count;
}

/*
 * nss_connmgr_ipv4_set_terminate()
 *	Writing anything to this 'file' will cause the default classifier to terminate
 */
static ssize_t nss_connmgr_ipv4_set_terminate(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{


	NSS_CONNMGR_DEBUG_INFO("Terminate\n");
	spin_lock_bh(&nss_connmgr_ipv4.lock);
	nss_connmgr_ipv4.terminate = 1;
	wake_up_process(nss_connmgr_ipv4.thread);
	spin_unlock_bh(&nss_connmgr_ipv4.lock);

	return count;
}

/*
 * nss_connmgr_ipv4_get_stop()
 * 	Get the value of "stopped" operational control variable
 */
static ssize_t nss_connmgr_ipv4_get_stop(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	ssize_t count;
	int num;

	/*
	 * Operate under our locks
	 */
	spin_lock_bh(&nss_connmgr_ipv4.lock);
	num = nss_connmgr_ipv4.stopped;
	spin_unlock_bh(&nss_connmgr_ipv4.lock);

	count = snprintf(buf, (ssize_t)PAGE_SIZE, "%d\n", num);
	return count;
}

/*
 * nss_connmgr_ipv4_set_stop()
 * 	Set the value of "stopped" operational control variable.
 * 	This will stop further processing of packets and adding new rules
 * 	to NSS (fastpath).
 */
static ssize_t nss_connmgr_ipv4_set_stop(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	char num_buf[12];
	int num;


	/*
	 * Get the number from buf into a properly z-termed number buffer
	 */
	if (count > 11) {
		return 0;
	}
	memcpy(num_buf, buf, count);
	num_buf[count] = '\0';
	sscanf(num_buf, "%d", &num);
	NSS_CONNMGR_DEBUG_TRACE("nss_connmgr_ipv4_stop = %d\n", num);

	/*
	 * Operate under our locks and stop further processing of packets
	 */
	spin_lock_bh(&nss_connmgr_ipv4.lock);
	nss_connmgr_ipv4.stopped = num;
	spin_unlock_bh(&nss_connmgr_ipv4.lock);

	return count;
}

/*
 * nss_connmgr_ipv4_get_need_mark()
 * 	Get the value of "need_mark" operational control variable
 */
static ssize_t nss_connmgr_ipv4_get_need_mark(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	ssize_t count;
	uint32_t num;

	/*
	 * Operate under our locks
	 */
	spin_lock_bh(&nss_connmgr_ipv4.lock);
	num = nss_connmgr_ipv4.need_mark;
	spin_unlock_bh(&nss_connmgr_ipv4.lock);

	count = snprintf(buf, (ssize_t)PAGE_SIZE, "%x\n", num);
	return count;
}

/*
 * nss_connmgr_ipv4_set_need_mark()
 * 	Set the value of "need_mark" operational control variable.
 */
static ssize_t nss_connmgr_ipv4_set_need_mark(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	char num_buf[12];
	uint32_t num;


	/*
	 * Get the hex number from buf into a properly z-termed number buffer
	 */
	if (count > 11) {
		return 0;
	}
	memcpy(num_buf, buf, count);
	num_buf[count] = '\0';
	sscanf(num_buf, "%x", &num);
	NSS_CONNMGR_DEBUG_TRACE("nss_connmgr_ipv4_need_mark = %x\n", num);

	/*
	 * Operate under our locks and stop further processing of packets
	 */
	spin_lock_bh(&nss_connmgr_ipv4.lock);
	nss_connmgr_ipv4.need_mark = num;
	spin_unlock_bh(&nss_connmgr_ipv4.lock);

	return count;
}

/*
 * SysFS attributes for the default classifier itself.
 */
static const struct device_attribute nss_connmgr_ipv4_terminate_attr =
		__ATTR(terminate, S_IWUGO | S_IRUGO, nss_connmgr_ipv4_get_terminate, nss_connmgr_ipv4_set_terminate);
static const struct device_attribute nss_connmgr_ipv4_stop_attr =
		__ATTR(stop, S_IWUGO | S_IRUGO, nss_connmgr_ipv4_get_stop, nss_connmgr_ipv4_set_stop);
static const struct device_attribute nss_connmgr_ipv4_need_mark_attr =
		__ATTR(need_mark, S_IWUGO | S_IRUGO, nss_connmgr_ipv4_get_need_mark, nss_connmgr_ipv4_set_need_mark);

/*
 * nss_connmgr_ipv4_thread_fn()
 *	A thread to handle tasks that can only be done in thread context.
 */
static int nss_connmgr_ipv4_thread_fn(void *arg)
{
	int result = 0;

	NSS_CONNMGR_DEBUG_INFO("NSS Connection manager thread start\n");

	/*
	 * Get reference to this module - we release it when the thread exits
	 */
	try_module_get(THIS_MODULE);

	/*
	 * Create /sys/nom_v4
	 */
	nss_connmgr_ipv4.nom_v4 = kobject_create_and_add("nom_v4", NULL);
	if (unlikely(nss_connmgr_ipv4.nom_v4 == NULL)) {
		NSS_CONNMGR_DEBUG_ERROR("Failed to register nom_v4\n");
		goto task_cleanup_1;
	}

	/*
	 * Create files, one for each parameter supported by this module
	 */
	result = sysfs_create_file(nss_connmgr_ipv4.nom_v4, &nss_connmgr_ipv4_terminate_attr.attr);
	if (result) {
		NSS_CONNMGR_DEBUG_ERROR("Failed to register terminate file %d\n", result);
		goto task_cleanup_2;
	}

	/*
	 * Register this module with the Linux NSS driver (net_device)
	 */
	nss_connmgr_ipv4.nss_context = nss_register_ipv4_mgr(nss_connmgr_ipv4_net_dev_callback);

	/*
	 * Initialize connection table
	 */
	nss_connmgr_ipv4_init_connection_table();

	/*
	 * Register netfilter hooks - IPV4
	 */
	result = nf_register_hooks(nss_connmgr_ipv4_ops_post_routing, ARRAY_SIZE(nss_connmgr_ipv4_ops_post_routing));
	if (result < 0) {
		NSS_CONNMGR_DEBUG_ERROR("Can't register nf post routing hook %d\n", result);
		goto task_cleanup_3;
	}

	result = sysfs_create_file(nss_connmgr_ipv4.nom_v4, &nss_connmgr_ipv4_stop_attr.attr);
	if (result) {
		NSS_CONNMGR_DEBUG_ERROR("Failed to register stop file %d\n", result);
		goto task_cleanup_4;
	}

#ifdef CONFIG_NF_CONNTRACK_EVENTS
	/*
	 * Eventing subsystem is available so we register a notifier hook to get fast notifications of expired connections
	 */
	result = nf_conntrack_register_notifier(&init_net, &nss_connmgr_ipv4.conntrack_notifier);
	if (result < 0) {
		NSS_CONNMGR_DEBUG_ERROR("Can't register nf notifier hook %d\n", result);
		goto task_cleanup_5;
	}
#endif

	nss_connmgr_ipv4.netdev_notifier.notifier_call = nss_connmgr_netdev_notifier_cb;
	result = register_netdevice_notifier(&nss_connmgr_ipv4.netdev_notifier);
	if (result != 0) {
		NSS_CONNMGR_DEBUG_ERROR("Can't register netdevice notifier %d\n", result);
		goto task_cleanup_6;
	}

	result = sysfs_create_file(nss_connmgr_ipv4.nom_v4, &nss_connmgr_ipv4_need_mark_attr.attr);
	if (result) {
		NSS_CONNMGR_DEBUG_ERROR("Failed to register need mark file %d\n", result);
		goto task_cleanup_7;
	}

#ifndef CONFIG_NF_CONNTRACK_CHAIN_EVENTS
	/*
	 * Initialize the conntrack event callback function to NULL.
	 * This will remain NULL unless other clients (ipv6 conn mgr) register a CB
	 */
	nss_connmgr_ipv4.conntrack_event_cb = NULL;
#endif

	/*
	 * Allow wakeup signals
	 */
	allow_signal(SIGCONT);
	spin_lock_bh(&nss_connmgr_ipv4.lock);
	while (!nss_connmgr_ipv4.terminate) {
		/*
		 * Sleep and wait for an instruction
		 */
		spin_unlock_bh(&nss_connmgr_ipv4.lock);
		NSS_CONNMGR_DEBUG_TRACE("nss_connmgr_ipv4 sleep\n");
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
		spin_lock_bh(&nss_connmgr_ipv4.lock);
	}
	spin_unlock_bh(&nss_connmgr_ipv4.lock);
	NSS_CONNMGR_DEBUG_INFO("nss_connmgr_ipv4 terminate\n");

	result = 0;


	sysfs_remove_file(nss_connmgr_ipv4.nom_v4, &nss_connmgr_ipv4_need_mark_attr.attr);
task_cleanup_7:
	unregister_netdevice_notifier(&nss_connmgr_ipv4.netdev_notifier);
task_cleanup_6:
#ifdef CONFIG_NF_CONNTRACK_EVENTS
	nf_conntrack_unregister_notifier(&init_net, &nss_connmgr_ipv4.conntrack_notifier);
task_cleanup_5:
#endif
	sysfs_remove_file(nss_connmgr_ipv4.nom_v4, &nss_connmgr_ipv4_stop_attr.attr);
task_cleanup_4:
	nf_unregister_hooks(nss_connmgr_ipv4_ops_post_routing, ARRAY_SIZE(nss_connmgr_ipv4_ops_post_routing));
task_cleanup_3:
	nss_unregister_ipv4_mgr();
	sysfs_remove_file(nss_connmgr_ipv4.nom_v4, &nss_connmgr_ipv4_terminate_attr.attr);
task_cleanup_2:
	kobject_put(nss_connmgr_ipv4.nom_v4);
task_cleanup_1:

	module_put(THIS_MODULE);
	return result;
}

/*
 * nss_connmgr_ipv4_read_conn_stats
 *      Read IPV4 stats
 */
static ssize_t nss_connmgr_ipv4_read_conn_stats(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	int32_t i;
	int32_t j;
	int32_t k;
	size_t size_al = NSS_CONNMGR_IPV4_DEBUGFS_BUF_SZ;
	size_t size_wr = 0;
	ssize_t bytes_read = 0;

	/*
	 * Temporary array to hold statistics for printing. To avoid holding
	 * spinlocks for long time while printing, we copy the stats to this
	 * temporary buffer while holding the spinlock, unlock and then print.
	 * Also to avoid using big chunk of memory on stack, we use an array of
	 * only 8 connections, and print stats for 8 connections at a time
	 */
	uint64_t stats[8][NSS_CONNMGR_IPV4_STATS_MAX];
	nss_connmgr_ipv4_conn_state_t  state[8];
	uint32_t src_addr[8];
	int32_t  src_port[8];
	uint32_t dest_addr[8];
	int32_t  dest_port[8];
	uint8_t  protocol[8];


	char *lbuf = kzalloc(size_al, GFP_KERNEL);
	if (unlikely(lbuf == NULL)) {
		NSS_CONNMGR_DEBUG_WARN("Could not allocate memory for local statistics buffer \n");
		return 0;
	}

	/*
	 * TODO : Handle buffer full conditions by putting some checks , so it
	 * doesn't crash or print something rubbish
	 */
	size_wr = scnprintf(lbuf, size_al,"connection mgr ipv4 stats :\n\n");

	for (i = 0; (i < NSS_CONNMGR_IPV4_CONN_MAX) && (size_wr < size_al) ; i+=8) {

		/* 1. Take a lock */
		spin_lock_bh(&nss_connmgr_ipv4.lock);

		/* 2. Copy stats for 8 connections into local buffer */
		for (j = 0; j < 8; j++) {
			state[j] = nss_connmgr_ipv4.connection[i+j].state;
			src_addr[j] = nss_connmgr_ipv4.connection[i+j].src_addr;
			src_port[j] = nss_connmgr_ipv4.connection[i+j].src_port;
			dest_addr[j] = nss_connmgr_ipv4.connection[i+j].dest_addr;
			dest_port[j] = nss_connmgr_ipv4.connection[i+j].dest_port;
			protocol[j] = nss_connmgr_ipv4.connection[i+j].protocol;

			for (k = 0; (k < NSS_CONNMGR_IPV4_STATS_MAX); k++) {
				stats[j][k] = nss_connmgr_ipv4.connection[i+j].stats[k];
			}
		}

		/* 3. Release the lock */
		spin_unlock_bh(&nss_connmgr_ipv4.lock);

		/* 4. Print stats for 8 connections */
		for (j = 0; (j < 8); j++)
		{
			if (state[j] != NSS_CONNMGR_IPV4_STATE_ESTABLISHED) {
				continue;
			}

			src_addr[j] = ntohl(src_addr[j]);
			dest_addr[j] = ntohl(dest_addr[j]);

			size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "---------------------------- \n");

			if (protocol[j] == IPPROTO_TCP) {
				size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "tcp ");
			} else if (protocol[j] == IPPROTO_UDP) {
				size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "udp ");
			} else {
				size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "proto=%d ", protocol[j]);
			}

			size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"src: %pI4:%d\n"
					"dest: %pI4:%d\n",
					&(src_addr[j]), (int)ntohs(src_port[j]),
					&(dest_addr[j]), (int)ntohs(dest_port[j]));

			for (k = 0; (k < NSS_CONNMGR_IPV4_STATS_MAX); k++) {
				size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
						"%s = %llu\n", nss_connmgr_ipv4_conn_stats_str[k], stats[j][k]);
			}
		}
	}

	if (size_wr == size_al) {
		NSS_CONNMGR_DEBUG_WARN("Print Buffer not available for debug stats \n");
	}
	else {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,"\nconnection mgr stats end\n\n");
	}

	bytes_read = simple_read_from_buffer(ubuf, sz, ppos, lbuf, strlen(lbuf));
	kfree(lbuf);

	return bytes_read;
}

/*
 * nss_connmgr_ipv4_read_debug_stats
 *	Read connection manager debug stats
 */
static ssize_t nss_connmgr_ipv4_read_debug_stats(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	int32_t i;

	/*
	 * max output lines = #stats + start tag line + end tag line + three blank lines
	 */
	uint32_t max_output_lines = NSS_CONNMGR_IPV4_DEBUG_STATS_MAX + 5;
	size_t size_al = NSS_CONNMGR_IPV4_MAX_STR_LENGTH * max_output_lines;
	size_t size_wr = 0;
	ssize_t bytes_read = 0;
	uint32_t stats_shadow[NSS_CONNMGR_IPV4_DEBUG_STATS_MAX];

	char *lbuf = kzalloc(size_al, GFP_KERNEL);
	if (unlikely(lbuf == NULL)) {
		NSS_CONNMGR_DEBUG_WARN("Could not allocate memory for local statistics buffer");
		return 0;
	}

	size_wr = scnprintf(lbuf, size_al,"debug stats start:\n\n");
	spin_lock_bh(&nss_connmgr_ipv4.lock);

	for (i = 0; (i < NSS_CONNMGR_IPV4_DEBUG_STATS_MAX); i++) {
		stats_shadow[i] = nss_connmgr_ipv4.debug_stats[i];
	}

	spin_unlock_bh(&nss_connmgr_ipv4.lock);

	for (i = 0; (i < NSS_CONNMGR_IPV4_DEBUG_STATS_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %u\n", nss_connmgr_ipv4_debug_stats_str[i], stats_shadow[i]);
	}

	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,"\ndebug stats end\n\n");
	bytes_read = simple_read_from_buffer(ubuf, sz, ppos, lbuf, strlen(lbuf));
	kfree(lbuf);

	return bytes_read;
}

/*
 * nss_connmgr_ipv4_clear_stats
 *      Clear IPV4 stats
 */
static ssize_t nss_connmgr_ipv4_clear_stats(struct file *fp, const char __user *ubuf, size_t count , loff_t *ppos)
{
	int32_t i,j;

	spin_lock_bh(&nss_connmgr_ipv4.lock);
	for (i = 0; (i < NSS_CONNMGR_IPV4_CONN_MAX); i++) {
		for (j = 0; (j < NSS_CONNMGR_IPV4_STATS_MAX); j++) {
			nss_connmgr_ipv4.connection[i].stats[j] = 0;
		}
	}
	spin_unlock_bh(&nss_connmgr_ipv4.lock);

	return count;
}

/*
 * nss_connmgr_ipv4_module_get()
 *	Take a reference to the module
 */
void nss_connmgr_ipv4_module_get(void)
{
	try_module_get(THIS_MODULE);
}
EXPORT_SYMBOL(nss_connmgr_ipv4_module_get);

/*
 * nss_connmgr_ipv4_module_put()
 *	Release a reference to the module
 */
void nss_connmgr_ipv4_module_put(void)
{
	module_put(THIS_MODULE);
}
EXPORT_SYMBOL(nss_connmgr_ipv4_module_put);

/*
 * nss_connmgr_ipv4_init()
 */
static int __init nss_connmgr_ipv4_init(void)
{
	struct dentry *dent, *dfile;

	NSS_CONNMGR_DEBUG_INFO("NSS Connection Manager Module init\n");

	/*
	 * Initialise our global database lock
	 */
	spin_lock_init(&nss_connmgr_ipv4.lock);

	/*
	 * Create a thread to handle the start/stop.
	 * NOTE: We use a thread as some things we need to do cannot be done in this context
	 */
	nss_connmgr_ipv4.terminate = 0;
	nss_connmgr_ipv4.thread = kthread_create(nss_connmgr_ipv4_thread_fn, NULL, "%s", "nss_connmgr_ipv4_thread");
	if (!nss_connmgr_ipv4.thread) {
		return -EINVAL;
	}
	wake_up_process(nss_connmgr_ipv4.thread);

	/*
	 * Create debugfs files for stats
	 */
	dent = debugfs_create_dir("qca-nss-connmgr-ipv4", NULL);

	if (unlikely(dent == NULL)) {

		/*
		 * non-availability of debugfs dir is not catastrophe.
		 * Print a message and gracefully return
		 */
		NSS_CONNMGR_DEBUG_WARN("Failed to create nss_connmgr_ipv4 dir in debugfs \n");
		return 0;
	}

	nss_connmgr_ipv4.dent = dent;

	dfile = debugfs_create_file("connection_stats", S_IRUGO , dent, &nss_connmgr_ipv4, &nss_connmgr_ipv4_show_stats_ops);

	if (unlikely(dfile == NULL))
	{
		NSS_CONNMGR_DEBUG_WARN("Failed to create nss_connmgr_ipv4/connection_stats in debugfs \n");
		debugfs_remove(dent);
	}

	dfile = debugfs_create_file("debug_stats", S_IRUGO , dent, &nss_connmgr_ipv4, &nss_connmgr_ipv4_show_debug_stats_ops);

	if (unlikely(dfile == NULL))
	{
		NSS_CONNMGR_DEBUG_WARN("Failed to create nss_connmgr_ipv4/debug_stats in debugfs \n");
		debugfs_remove(dent);
	}

	dfile = debugfs_create_file("clear_stats", S_IRUGO | S_IWUSR , dent, &nss_connmgr_ipv4, &nss_connmgr_ipv4_clear_stats_ops);

	if (unlikely(dfile == NULL))
	{
		NSS_CONNMGR_DEBUG_WARN("Failed to create nss_connmgr_ipv4/clear_stats in debugfs \n");
		debugfs_remove(dent);
	}

	/* Register Link Aggregation interfaces with NSS driver */
	nss_register_lag_if(NSS_LAG0_INTERFACE_NUM, NULL, nss_connmgr_lag_event_cb, NULL);
	nss_register_lag_if(NSS_LAG1_INTERFACE_NUM, NULL, nss_connmgr_lag_event_cb, NULL);

	/*
	 * Register Link Aggregation callbacks
	 */
	nss_connmgr_bond_cb.bond_cb_link_up = nss_connmgr_bond_link_up;
	nss_connmgr_bond_cb.bond_cb_release = nss_connmgr_bond_release;
	nss_connmgr_bond_cb.bond_cb_enslave = nss_connmgr_bond_enslave;
	bond_register_cb(&nss_connmgr_bond_cb);

	return 0;
}

/*
 * nss_connmgr_ipv4_exit()
 * 	Module exit function
 */
static void __exit nss_connmgr_ipv4_exit(void)
{
	bond_unregister_cb();

	/* Unregister Link Aggregation interfaces with NSS driver */
	nss_unregister_lag_if(NSS_LAG0_INTERFACE_NUM);
	nss_unregister_lag_if(NSS_LAG1_INTERFACE_NUM);

	/*
	 * Remove debugfs tree
	 */
	if (likely(nss_connmgr_ipv4.dent != NULL)) {
		debugfs_remove_recursive(nss_connmgr_ipv4.dent);
	}

	NSS_CONNMGR_DEBUG_INFO("NSS Connection Manager Module exit\n");
}

module_init(nss_connmgr_ipv4_init);
module_exit(nss_connmgr_ipv4_exit);

MODULE_AUTHOR("Qualcomm Atheros Inc");
MODULE_DESCRIPTION("NSS IPv4 Connection manager");
MODULE_LICENSE("Dual BSD/GPL");

