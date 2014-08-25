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
 * @file
 * This file is the NSS connection manager for managing IPv6 connections.It
 * forms an interface between the fast-path NSS driver and Linux
 * connection track module for updating/syncing connection level information
 * between the two.It is responsible for maintaing  all connection (flow) level
 * information and statistics for all fast path connections.
 *
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
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/pkt_sched.h>
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/netdevice.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <net/ip6_fib.h>
#include <net/ip6_route.h>
#include <net/tcp.h>
#include <asm/unaligned.h>
#include <asm/uaccess.h> /* for put_user */

#include <linux/netfilter_ipv6.h>
#include <linux/netfilter_bridge.h>
#include <linux/if_bridge.h>
#include <net/netfilter/nf_conntrack.h>
#include <net/netfilter/nf_conntrack_acct.h>
#include <net/netfilter/nf_conntrack_helper.h>
#include <net/netfilter/nf_conntrack_l4proto.h>
#include <net/netfilter/nf_conntrack_l3proto.h>
#include <net/netfilter/nf_conntrack_zones.h>
#include <net/netfilter/nf_conntrack_core.h>
#include <net/netfilter/ipv6/nf_conntrack_ipv6.h>
#include <net/netfilter/ipv6/nf_defrag_ipv6.h>

#include <net/arp.h>
#include <net/neighbour.h>

#include <nss_api_if.h>
#include <linux/../../net/8021q/vlan.h>
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


/**
 * This macro converts format for IPv6 address (from Linux to NSS)
 */
#define IN6_ADDR_TO_IPV6_ADDR(ipv6, in6) \
	{ \
		((uint32_t *)ipv6)[0] = in6.in6_u.u6_addr32[0]; \
		((uint32_t *)ipv6)[1] = in6.in6_u.u6_addr32[1]; \
		((uint32_t *)ipv6)[2] = in6.in6_u.u6_addr32[2]; \
		((uint32_t *)ipv6)[3] = in6.in6_u.u6_addr32[3]; \
	}

/**
 * This macro converts format for IPv6 address (from NSS to Linux)
 */
#define IPV6_ADDR_TO_IN6_ADDR(in6, ipv6) \
	{ \
		in6.in6_u.u6_addr32[0] = ((uint32_t *)ipv6)[0]; \
		in6.in6_u.u6_addr32[1] = ((uint32_t *)ipv6)[1]; \
		in6.in6_u.u6_addr32[2] = ((uint32_t *)ipv6)[2]; \
		in6.in6_u.u6_addr32[3] = ((uint32_t *)ipv6)[3]; \
	}

/**
 * This macro converts format for IPv6 address (from NSS to Linux)
 */
#define IPV6_ADDR_NTOH_IN6_ADDR(in6, ipv6) \
	{ \
		in6.in6_u.u6_addr32[0] = ntohl(((uint32_t *)ipv6)[0]); \
		in6.in6_u.u6_addr32[1] = ntohl(((uint32_t *)ipv6)[1]); \
		in6.in6_u.u6_addr32[2] = ntohl(((uint32_t *)ipv6)[2]); \
		in6.in6_u.u6_addr32[3] = ntohl(((uint32_t *)ipv6)[3]); \
	}
/**
 * This macro converts IPv6 address from network order to host order
 */
#define IPV6_ADDR_NTOH(ipv6_h, ipv6_n) \
	{ \
		ipv6_h[0] = (ntohl(ipv6_n[0])); \
		ipv6_h[1] = (ntohl(ipv6_n[1])); \
		ipv6_h[2] = (ntohl(ipv6_n[2])); \
		ipv6_h[3] = (ntohl(ipv6_n[3])); \
	}


/*
 * Custom types recognised within the Connection Manager (Linux independent)
 */
typedef uint8_t mac_addr_t[6];
typedef uint32_t ipv6_addr_t[4];

#define is_bridge_port(dev) (dev && (dev->priv_flags & IFF_BRIDGE_PORT))
#define is_bridge_device(dev) (dev->priv_flags & IFF_EBRIDGE)
#define is_lag_master(dev)	((dev->flags & IFF_MASTER)		\
				 && (dev->priv_flags & IFF_BONDING))
#define is_lag_slave(dev)	((dev->flags & IFF_SLAVE)		\
				 && (dev->priv_flags & IFF_BONDING))



/*
 * Displaying addresses
 */
#define MAC_AS_BYTES(mac_addr) mac_addr[5], mac_addr[4], mac_addr[3],mac_addr[2], mac_addr[1], mac_addr[0]

#define IPV6_ADDR_OCTAL_FMT "%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x"

#define IPV6_ADDR_TO_OCTAL(ipv6) ((uint16_t *)ipv6)[0], ((uint16_t *)ipv6)[1], ((uint16_t *)ipv6)[2], ((uint16_t *)ipv6)[3], ((uint16_t *)ipv6)[4], ((uint16_t *)ipv6)[5], ((uint16_t *)ipv6)[6], ((uint16_t *)ipv6)[7]

#define MAC_FMT "%02x:%02x:%02x:%02x:%02x:%02x"

/*
 * Max NSS ipv6 Flow entries
 */
#define NSS_CONNMGR_IPV6_CONN_MAX 4096

/*
 * size of buffer allocated for stats printing (using debugfs)
 */
#define NSS_CONNMGR_IPV6_DEBUGFS_BUF_SZ (NSS_CONNMGR_IPV6_CONN_MAX*512)

/*
 * Maximum string length:
 * This should be equal to maximum string size of any stats
 * inclusive of stats value
 */
#define NSS_CONNMGR_IPV6_MAX_STR_LENGTH 96
#define NSS_CONNMGR_VLAN_ID_NOT_CONFIGURED 0xFFF
#define NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED 0xFFFF
#define NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED 0xFF

enum nss_connmgr_ipv6_conn_stats {
	NSS_CONNMGR_IPV6_ACCELERATED_RX_PKTS = 0,
					/* Accelerated ipv6 RX packets */
	NSS_CONNMGR_IPV6_ACCELERATED_RX_BYTES,
					/* Accelerated ipv6 RX bytes */
	NSS_CONNMGR_IPV6_ACCELERATED_TX_PKTS,
					/* Accelerated ipv6 TX packets */
	NSS_CONNMGR_IPV6_ACCELERATED_TX_BYTES,
					/* Accelerated ipv6 TX bytes */
	NSS_CONNMGR_IPV6_STATS_MAX
};

typedef enum nss_connmgr_ipv6_conn_statistics nss_connmgr_ipv6_conn_statistics_t;

/*
 * Debug statistics
 */
enum nss_connmgr_ipv6_debug_statistics {
	NSS_CONNMGR_IPV6_ACTIVE_CONN,
				/* Active connections */
	NSS_CONNMGR_IPV6_CREATE_FAIL,
				/* Rule create failures */
	NSS_CONNMGR_IPV6_DESTROY_FAIL,
				/* Rule destroy failures */
	NSS_CONNMGR_IPV6_ESTABLISH_MISS,
				/* No establish response from NSS  */
	NSS_CONNMGR_IPV6_DESTROY_MISS,
				/* No Flush/Evict/Destroy response from NSS */
	NSS_CONNMGR_IPV6_DEBUG_STATS_MAX
};

typedef enum nss_connmgr_ipv6_debug_statistics nss_connmgr_debug_ipv6_statistics_t;

/*
 * nss_connmgr_ipv6_conn_stats_str
 *      Connection statistics strings
 */
static char *nss_connmgr_ipv6_conn_stats_str[] = {
	"rx_pkts",
	"rx_bytes",
	"tx_pkts",
	"tx_bytes",
};

/*
 * nss_connmgr_ipv6_stats_str
 *      Debug statistics strings
 */
static char *nss_connmgr_ipv6_debug_stats_str[] = {
	"active_conns",
	"create_fail",
	"destroy_fail",
	"establish_miss",
	"destroy_miss",
};

/*
 * Connection states as defined by connection manager
 *
 * INACTIVE	Connection is not active
 * ESTABLISHED	Connection is established, and NSS sends periodic sync messages
 * STALE	Linux and NSS are out of sync.
 * 		Conntrack -> Conn Mgr sent a rule destroy command,
 * 		but the command did  not reach NSS
 *
 */
typedef enum  {
	NSS_CONNMGR_IPV6_STATE_INACTIVE,
	NSS_CONNMGR_IPV6_STATE_ESTABLISHED,
	NSS_CONNMGR_IPV6_STATE_STALE,
} nss_connmgr_ipv6_conn_state_t;

/*
 * ipv6 statistics
 */
struct nss_connmgr_ipv6_connection {
	uint8_t  state;		/* Connection state - established, not established, destroyed */
	uint8_t  protocol;	/* Protocol number */
	int32_t  src_interface;	/* Flow interface number */
	uint32_t src_addr[4];	/* Source address, i.e. the creator of the connection */
	int32_t  src_port;	/* Source port */
	uint16_t ingress_vlan_tag;/* Ingress VLAN tag */
	char	 src_mac_addr[ETH_ALEN];	/* Source MAC address */
	int32_t  dest_interface;
				/* Return interface number */
	uint32_t dest_addr[4];	/* Destination address, i.e. the to whom the connection was created */
	int32_t  dest_port;	/* Destination port */
	uint16_t egress_vlan_tag;/* Egress VLAN tag */
	char	 dest_mac_addr[ETH_ALEN];	/* Destination MAC address */
	uint64_t stats[NSS_CONNMGR_IPV6_STATS_MAX];
				/* Connection statistics */
	uint32_t last_sync;	/* Last sync time as jiffies */
};

/*
 * Global connection manager instance object.
 */
struct nss_connmgr_ipv6_instance {
	spinlock_t lock;	/* Lock to Protect against SMP access. */
	struct kobject *nom_v6;
				/* Sysfs link. Represents the sysfs folder /sys/nom_v6 */
	int32_t stopped;
				/* General operational control.When non-zero further traffic will not be processed */
	int32_t terminate;
				/* Signal to tell the control thread to terminate */
	struct task_struct *thread;
				/* Control thread */
	void *nss_context;
				/* Registration context used to identify the manager in calls to the NSS driver */
#ifdef CONFIG_NF_CONNTRACK_CHAIN_EVENTS
	struct notifier_block conntrack_notifier;
#endif
	struct nss_connmgr_ipv6_connection connection[NSS_CONNMGR_IPV6_CONN_MAX];
				/* Connection Table */
	struct notifier_block netdev_notifier;
	struct dentry *dent;	/* Debugfs directory */
	uint32_t debug_stats[NSS_CONNMGR_IPV6_DEBUG_STATS_MAX];
				/* Debug statistics */
	uint32_t need_mark;	/* When 0 needing to see a mark value is disabled.  When != 0 we only process packets that have the given skb->mark value */
};

static unsigned int nss_connmgr_ipv6_post_routing_hook(unsigned int hooknum,
				struct sk_buff *skb,
				const struct net_device *in_unused,
				const struct net_device *out,
				int (*okfn)(struct sk_buff *));

static unsigned int nss_connmgr_ipv6_bridge_post_routing_hook(unsigned int hooknum,
		struct sk_buff *skb,
		const struct net_device *in_unused,
		const struct net_device *out,
		int (*okfn)(struct sk_buff *));

static ssize_t nss_connmgr_ipv6_read_stats(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos);
static ssize_t nss_connmgr_ipv6_read_debug_stats(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos);
static ssize_t nss_connmgr_ipv6_clear_stats(struct file *fp, const char __user *ubuf, size_t count, loff_t *ppos);

#if defined(CONFIG_NF_CONNTRACK_EVENTS) && !defined(CONFIG_NF_CONNTRACK_CHAIN_EVENTS)
static int nss_connmgr_ipv6_conntrack_event(struct nf_conn *ct);
#elif defined(CONFIG_NF_CONNTRACK_EVENTS)
static int nss_connmgr_ipv6_conntrack_event(struct notifier_block *this,
		unsigned long events, void *ptr);
#endif

static struct nss_connmgr_ipv6_instance nss_connmgr_ipv6 = {
	.stopped = 0,
	.terminate = 0,
	.thread = NULL,
	.nss_context = NULL,
#ifdef CONFIG_NF_CONNTRACK_CHAIN_EVENTS
	.conntrack_notifier = {
			.notifier_call = nss_connmgr_ipv6_conntrack_event,
	},
#endif
};

extern struct net_device *bond_get_tx_dev(struct sk_buff *skb,
					  uint8_t *src_mac, uint8_t *dst_mac,
					  void *src, void *dst,
					  uint16_t protocol,
					  struct net_device *bond_dev);

/*
 * nss_connmgr_ipv6_ops_post_routing[]
 *
 * Hooks into the post routing netfilter point -
 * this will pick up local outbound and packets going from one interface to another
 */
static struct nf_hook_ops nss_connmgr_ipv6_ops_post_routing[] __read_mostly = {
	{
		.hook           = nss_connmgr_ipv6_post_routing_hook,
		.owner          = THIS_MODULE,
		.pf             = PF_INET6,
		.hooknum        = NF_INET_POST_ROUTING,
		.priority       = NF_IP6_PRI_NAT_SRC + 1 ,
					/*
					 * Refer to include/linux/netfiler_ipv6.h for priority levels.
					 * Examine packets after NAT translation (and potentially any ALG processing)
					 */
	},
	{
	.hook           = nss_connmgr_ipv6_bridge_post_routing_hook,
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
 * Expose what should be a static flag in the TCP connection tracker.
 */
extern int nf_ct_tcp_be_liberal;
extern int nf_ct_tcp_no_window_check;

#ifdef CONFIG_NF_CONNTRACK_EVENTS
extern void nss_connmgr_ipv4_register_conntrack_event_cb(int (*event_cb)(struct nf_conn *));
extern void nss_connmgr_ipv4_unregister_conntrack_event_cb(void);
#endif

extern void nss_connmgr_ipv4_register_bond_slave_linkup_cb(void (*event_cb)(struct net_device *));
extern void nss_connmgr_ipv4_unregister_bond_slave_linkup_cb(void);

static const struct file_operations nss_connmgr_ipv6_show_stats_ops = {
	.open = simple_open,
	.read = nss_connmgr_ipv6_read_stats,
};

static const struct file_operations nss_connmgr_ipv6_show_debug_stats_ops = {
	.open = simple_open,
	.read = nss_connmgr_ipv6_read_debug_stats,
};

static const struct file_operations nss_connmgr_ipv6_clear_stats_ops = {
	.write = nss_connmgr_ipv6_clear_stats,
};

/*
 * Network flow:
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

static struct neighbour *nss_connmgr_ipv6_neigh_get(ipv6_addr_t addr) {

	struct dst_entry *dst;
	struct in6_addr daddr;
	struct rt6_info *rt6i;
	struct neighbour *neigh;

	/*
	 * Look up the route to this address
	 */
	IPV6_ADDR_TO_IN6_ADDR(daddr, addr);
	rt6i = rt6_lookup(&init_net, &daddr, NULL, 0, 0);
	if (!rt6i) {
		NSS_CONNMGR_DEBUG_TRACE("rt6_lookup failed for: " IPV6_ADDR_OCTAL_FMT "\n", IPV6_ADDR_TO_OCTAL(addr));
		return NULL;
	}

	/*
	 * refering to rt6_lookup an rt6_info is a subclass of dst_entry so we can cast it.
	 */
	dst = (struct dst_entry *)rt6i;

	/*
	 * Get the neighbour to which this destination is pointing
	 */
	neigh = dst_get_neighbour_noref(dst);

	if (!neigh) {
		neigh = neigh_lookup(&nd_tbl, &daddr, dst->dev);
	} else {
		neigh_hold(neigh);
	}

	/* Release dst reference */
	dst_release(dst);

	return neigh;
}

/*
 * nss_connmgr_ipv6_mac_addr_get()
 *	Return the hardware (MAC) address of the given ipv6 address, if any.
 *
 * Returns 0 on success or a negative result on failure.
 * We first look up for an entry in the neighbour discovery table, if entry is not found,
 * We look up the rtable entry for the address and, from its neighbour structure, obtain the hardware address.
 * This means we will also work if the neighbours are routers too.
 */
static int nss_connmgr_ipv6_mac_addr_get(ipv6_addr_t addr, mac_addr_t mac_addr)
{
	struct neighbour *neigh;

	rcu_read_lock();

	neigh = nss_connmgr_ipv6_neigh_get(addr);

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
		NSS_CONNMGR_DEBUG_TRACE("Mac is multicast / broadcast - ignoring \n");
		return -4;
	}

	return 0;
}

static struct net_device *nss_connmgr_get_dev_from_ipv6_address(struct net *net, int oif, struct in6_addr addr)
{
	struct rt6_info *rt6i;
	struct dst_entry *dst;
	struct net_device *dev;

	/*
	 * Look up the route to this address
	 */
	rt6i = rt6_lookup(net, &addr, NULL, oif, 0);
	if (!rt6i) {
		NSS_CONNMGR_DEBUG_TRACE("rt6_lookup failed for: " IPV6_ADDR_OCTAL_FMT "\n", IPV6_ADDR_TO_OCTAL(addr.in6_u.u6_addr32));
		return NULL;
	}

	dst = (struct dst_entry *)rt6i;
	dev = dst->dev;
	dst_release(dst);

	return dev;
}

/*
 * nss_connmgr_destroy_ipv6_rule()
 *     Destroy an ipv6 rule. Called with nss_connmgr_ipv6.lock held.
 */
static int32_t nss_connmgr_destroy_ipv6_rule(struct nss_connmgr_ipv6_connection *connection)
{
	struct nss_ipv6_destroy unid;
	nss_tx_status_t nss_tx_status;

	unid.protocol = connection->protocol;
	memcpy(unid.src_ip, connection->src_addr, sizeof(unid.src_ip));
	memcpy(unid.dest_ip, connection->dest_addr, sizeof(unid.src_ip));
	unid.src_port = connection->src_port;
	unid.dest_port = connection->dest_port;

	nss_tx_status = nss_tx_destroy_ipv6_rule(nss_connmgr_ipv6.nss_context, &unid);
	if (nss_tx_status != NSS_TX_SUCCESS) {
		NSS_CONNMGR_DEBUG_ERROR("Unable to destroy IPv6 rule."
					"src " IPV6_ADDR_OCTAL_FMT ":%d dst "
					IPV6_ADDR_OCTAL_FMT ":%d proto %d\n",
					IPV6_ADDR_TO_OCTAL(unid.src_ip), unid.src_port,
					IPV6_ADDR_TO_OCTAL(unid.dest_ip), unid.dest_port,
					unid.protocol);
		return -EIO;
	}

	return 0;
}


/*
 * nss_connmgr_link_down()
 *	Handle a link down on an interface.
 *	Only handles interfaces used by NSS.
 */
static void nss_connmgr_link_down(struct net_device *dev)
{
	uint32_t if_num = 0;
	uint32_t i = 0;
	int32_t if_src = 0;
	int32_t if_dst = 0;
	uint16_t vlan_id = 0;
	struct nss_connmgr_ipv6_connection *connection = NULL;
	nss_connmgr_ipv6_conn_state_t cstate;
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

	if_num = nss_cmn_get_interface_number(nss_connmgr_ipv6.nss_context, phys_dev);
	if (if_num < 0) {
		NSS_CONNMGR_DEBUG_WARN("Cannot find NSS if num for dev %s\n", phys_dev->name);
		return;
	}

	for (i = 0; i < NSS_CONNMGR_IPV6_CONN_MAX; i++) {
		spin_lock_bh(&nss_connmgr_ipv6.lock);
		connection = &nss_connmgr_ipv6.connection[i];
		cstate = connection->state;
		if_src = connection->src_interface;
		if_dst = connection->dest_interface;

		if (cstate != NSS_CONNMGR_IPV6_STATE_ESTABLISHED) {
			spin_unlock_bh(&nss_connmgr_ipv6.lock);
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
					spin_unlock_bh(&nss_connmgr_ipv6.lock);
					continue;
				}
			}
			NSS_CONNMGR_DEBUG_INFO("destroy NSS rule at index %d\n", i);
			nss_connmgr_destroy_ipv6_rule(connection);
		}
		spin_unlock_bh(&nss_connmgr_ipv6.lock);
	}
}


/*
 * nss_connmgr_bond_link_up()
 *	Callback used to signal a link up on an interface
 *	that is part of a link aggregation.
 */
static void nss_connmgr_bond_link_up(struct net_device *slave_dev)
{
	uint32_t if_num = 0;
	uint32_t i = 0;
	struct nss_connmgr_ipv6_connection *connection = NULL;
	uint8_t flush_rule;
	uint32_t src_if;
	uint32_t dst_if;
	struct net_device *indev;
	struct net_device *outdev;
	nss_connmgr_ipv6_conn_state_t cstate;

	if_num = nss_cmn_get_interface_number(nss_connmgr_ipv6.nss_context, slave_dev);
	if (if_num < 0) {
		NSS_CONNMGR_DEBUG_ERROR("Cannot find NSS if num for slave dev %s\n", slave_dev->name);
		return;
	}

	for (i = 0; i < NSS_CONNMGR_IPV6_CONN_MAX; i++)
	{
		spin_lock_bh(&nss_connmgr_ipv6.lock);
		connection = &nss_connmgr_ipv6.connection[i];
		cstate = connection->state;
		src_if = connection->src_interface;
		dst_if = connection->dest_interface;
		spin_unlock_bh(&nss_connmgr_ipv6.lock);

		if (cstate != NSS_CONNMGR_IPV6_STATE_ESTABLISHED) {
			continue;
		}

		flush_rule = 0;

		indev = nss_cmn_get_interface_dev(nss_connmgr_ipv6.nss_context, src_if);
		if (indev != NULL) {
			if (is_lag_slave(indev)) {
				flush_rule = 1;
			}
		}

		outdev = nss_cmn_get_interface_dev(nss_connmgr_ipv6.nss_context, dst_if);
		if (outdev != NULL) {
			if (is_lag_slave(outdev)) {
				flush_rule = 1;
			}
		}

		if (flush_rule) {
			spin_lock_bh(&nss_connmgr_ipv6.lock);
			if (connection->state != NSS_CONNMGR_IPV6_STATE_ESTABLISHED) {
				spin_unlock_bh(&nss_connmgr_ipv6.lock);
				continue;
			}

			NSS_CONNMGR_DEBUG_INFO("destroying NSS rule at index %d\n", i);
			nss_connmgr_destroy_ipv6_rule(connection);
			spin_unlock_bh(&nss_connmgr_ipv6.lock);
		}
	}
}

/*
 * nss_connmgr_do_bond_down
 *	Go through list of interfaces on the system and delete
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
 *      LAG master closed
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
 *	Bridge interface closed
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
 *      Netdevice notifier callback
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
 * nss_connmgr_ipv6_bridge_post_routing_hook()
 *	Called for packets about to leave the box through a bridge slave interface
 */
static unsigned int nss_connmgr_ipv6_bridge_post_routing_hook(unsigned int hooknum,
		struct sk_buff *skb,
		const struct net_device *in_unused,
		const struct net_device *out,
		int (*okfn)(struct sk_buff *))
{
	struct nss_ipv6_create unic;

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
	struct sock *sk = NULL;
	struct udp_sock *usk = NULL;

	/*
	 * Variables needed for PPPoE WAN mode.
	 */
	struct net_device *eth_out = NULL;
	struct net_device *ppp_in = NULL;
	bool is_flow_pppoe;
	bool is_return_pppoe;

	nss_tx_status_t nss_tx_status;

	sk = skb->sk;

	/*
	 * If the 'need_mark' flag is set and this packet does not have the relevant mark
	 * then we don't accelerate at all
	 */
	if (nss_connmgr_ipv6.need_mark && (nss_connmgr_ipv6.need_mark != skb->mark)) {
		NSS_CONNMGR_DEBUG_TRACE("Mark %x not seen, ignoring: %p\n", nss_connmgr_ipv6.need_mark, skb);
		return NF_ACCEPT;
	}

	/*
	 * Only process IPV6 packets in bridge hook
	 */
	if(skb->protocol != htons(ETH_P_IPV6)){
		NSS_CONNMGR_DEBUG_TRACE("non ipv6 , ignoring: %p\n", skb);
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
		NSS_CONNMGR_DEBUG_TRACE("in device (%s) not 802.3 hw addr len (%u), ignoring: %p\n", in->name, (unsigned)in->addr_len, skb);
		dev_put(in);
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
	 * Get addressing information
	 */
	IN6_ADDR_TO_IPV6_ADDR(unic.src_ip, orig_tuple.src.u3.in6);
	IN6_ADDR_TO_IPV6_ADDR(unic.dest_ip, orig_tuple.dst.u3.in6);

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
		unic.flags |= NSS_IPV6_CREATE_FLAG_ROUTED;
	}

	/*
	 * Set the PPPoE values to the defaults, just in case there is not any PPPoE connection.
	 */
	unic.return_pppoe_session_id = 0;
	unic.flow_pppoe_session_id = 0;

	switch (unic.protocol) {
	case IPPROTO_TCP:
		unic.src_port = (int32_t)orig_tuple.src.u.tcp.port;
		unic.dest_port = (int32_t)orig_tuple.dst.u.tcp.port;
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

			unic.flags |= NSS_IPV6_CREATE_FLAG_NO_SEQ_CHECK;
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
		usk = udp_sk(sk);
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
	 * Access the ingress routed interface
	 */
	rt_dev = nss_connmgr_get_dev_from_ipv6_address(dev_net(in), in->ifindex, ipv6_hdr(skb)->saddr);

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
			 * Get the slave bridge port which is the ingress interface
			 * for this packet.
			 */
			br_port_in_dev = br_port_dev_get(out->master, eth_hdr(skb)->h_source);
			if (br_port_in_dev == NULL) {
				dev_put(in);
				NSS_CONNMGR_DEBUG_TRACE("Bridge-CM: Ingress Virtual Port not found for bridge %s\n",out->master->name);
				return NF_ACCEPT;
			}
			NSS_CONNMGR_DEBUG_TRACE("Bridge-CM: Bridge Port Ingress Virtual Interface = %s\n", br_port_in_dev->name);

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
		unic.flags |= NSS_IPV6_CREATE_FLAG_BRIDGE_FLOW;

		/*
		 * Configure the MAC addresses for the flow
		 */
		eh = (struct ethhdr *)skb->mac_header;
		if (ctinfo < IP_CT_IS_REPLY) {
			memcpy(unic.src_mac, eh->h_source, ETH_HLEN);
			memcpy(unic.dest_mac, eh->h_dest, ETH_HLEN);
		} else {
			memcpy(unic.src_mac, eh->h_dest, ETH_HLEN);
			memcpy(unic.dest_mac, eh->h_source, ETH_HLEN);
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
		src_dev = in;
		dest_dev = physical_out_dev;
		NSS_CONNMGR_DEBUG_TRACE("%p: dir: Original\n", ct);
	} else {
		src_dev = physical_out_dev;
		dest_dev = in;
		NSS_CONNMGR_DEBUG_TRACE("%p: dir: Reply\n", ct);
	}


	/*
	 * Collect the MAC address information if routing is involved in the flow
	 */
	if ((rt_dev != NULL)
		&& !(is_bridge_device(rt_dev) && (rt_dev == (struct net_device *)out->master))) {
		/*
		 * Get the MAC addresses that correspond to source and destination host addresses.
		 */
		if (nss_connmgr_ipv6_mac_addr_get(unic.src_ip, unic.src_mac)) {
			dev_put(in);
			NSS_CONNMGR_DEBUG_TRACE("%p: Failed to find MAC address for src IP: %pI6\n", ct, &unic.src_ip);
			return NF_ACCEPT;
		}

		/*
		 * Do dest now
		 */
		if (nss_connmgr_ipv6_mac_addr_get(unic.dest_ip, unic.dest_mac)) {
			dev_put(in);
			NSS_CONNMGR_DEBUG_TRACE("%p: Failed to find MAC address for dest IP: %pI6\n", ct, &unic.dest_ip);
			return NF_ACCEPT;
		}
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
			if (unic.flags & NSS_IPV6_CREATE_FLAG_BRIDGE_FLOW) {
				lag_smac = unic.dest_mac;
			} else {
				lag_smac = (uint8_t *)src_dev->master->dev_addr;
			}

			src_slave = bond_get_tx_dev(NULL, lag_smac,
						    unic.src_mac,
						    (void *)&unic.dest_ip,
						    (void *)&unic.src_ip,
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
			if (unic.flags & NSS_IPV6_CREATE_FLAG_BRIDGE_FLOW) {
				lag_smac = unic.src_mac;
			} else {
				lag_smac = (uint8_t *)dest_dev->dev_addr;
			}

			dest_slave = bond_get_tx_dev(NULL, lag_smac,
						     unic.dest_mac,
						     (void *)&unic.src_ip,
						     (void *)&unic.dest_ip,
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
			if (unic.flags & NSS_IPV6_CREATE_FLAG_BRIDGE_FLOW) {
				lag_smac = unic.dest_mac;
			} else {
				lag_smac = src_dev->dev_addr;
			}

			src_slave = bond_get_tx_dev(NULL, lag_smac,
						    unic.src_mac,
						    (void *)&unic.dest_ip,
						    (void *)&unic.src_ip,
						    skb->protocol, src_dev);
			if (src_slave == NULL
			    || !netif_carrier_ok(src_slave)) {
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
			if (unic.flags & NSS_IPV6_CREATE_FLAG_BRIDGE_FLOW) {
				lag_smac = unic.src_mac;
			} else {
				lag_smac = (uint8_t *)dest_dev->master->dev_addr;
			}

			dest_slave = bond_get_tx_dev(NULL, lag_smac,
						     unic.dest_mac,
						     (void *)&unic.src_ip,
						     (void *)&unic.dest_ip,
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
	unic.src_interface_num = nss_cmn_get_interface_number(nss_connmgr_ipv6.nss_context, src_dev);
	if (unic.src_interface_num < 0) {
		dev_put(in);
		return NF_ACCEPT;
	}

	unic.dest_interface_num = nss_cmn_get_interface_number(nss_connmgr_ipv6.nss_context, dest_dev);
	if (unic.dest_interface_num < 0) {
		dev_put(in);
		return NF_ACCEPT;
	}

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
			"src_ip: " IPV6_ADDR_OCTAL_FMT ":%d\n"
			"dest_ip: " IPV6_ADDR_OCTAL_FMT ":%d\n"
			"src_mac: " MAC_FMT "\n"
			"dest_mac: " MAC_FMT "\n"
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
			IPV6_ADDR_TO_OCTAL(unic.src_ip), unic.src_port,
			IPV6_ADDR_TO_OCTAL(unic.dest_ip), unic.dest_port,
			MAC_AS_BYTES(unic.src_mac),
			MAC_AS_BYTES(unic.dest_mac),
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

	/*
	 * Setting the appropriate flags for DSCP and VLAN marking.
	 * API's to set the data for remarking should be called before this.
	 */
	if(unic.dscp_oval != NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED)
		unic.flags |= NSS_IPV6_CREATE_FLAG_DSCP_MARKING;
	if(unic.vlan_oval != NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED)
		unic.flags |= NSS_IPV6_CREATE_FLAG_VLAN_MARKING;

	/*
	 * Create the Network Accelerator connection cache entries
	 *
	 * NOTE: All of the information we have is from the point of view of who created the connection however
	 * the skb may actually be in the 'reply' direction - which is important to know when configuring the NSS as we have to set the right information
	 * on the right "match" and "forwarding" entries.
	 * We can use the ctinfo to determine which direction the skb is in and then swap fields as necessary.
	 */
	IPV6_ADDR_NTOH(unic.src_ip, unic.src_ip);
	IPV6_ADDR_NTOH(unic.dest_ip, unic.dest_ip);
	unic.src_port = ntohs(unic.src_port);
	unic.dest_port = ntohs(unic.dest_port);

	/*
	 * If operations have stopped then do not process packets
	 */
	spin_lock_bh(&nss_connmgr_ipv6.lock);
	if (unlikely(nss_connmgr_ipv6.stopped)) {
		spin_unlock_bh(&nss_connmgr_ipv6.lock);
		dev_put(in);
		NSS_CONNMGR_DEBUG_TRACE("Stopped, ignoring: %p\n", skb);

		return NF_ACCEPT;
	}
	spin_unlock_bh(&nss_connmgr_ipv6.lock);

	/*
	 * Calling ipv6_rule1 for DSCP and VLAN marking else we use the regular
	 * ipv6_rule
	 */
	if(unic.dscp_oval == NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED &&
		unic.vlan_oval == NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED) {
		nss_tx_status = nss_tx_create_ipv6_rule(nss_connmgr_ipv6.nss_context, &unic);
	} else {
		nss_tx_status = nss_tx_create_ipv6_rule1(nss_connmgr_ipv6.nss_context, &unic);
	}

	if (nss_tx_status == NSS_TX_SUCCESS) {
		goto out;
	} else if (nss_tx_status == NSS_TX_FAILURE_NOT_READY) {
		NSS_CONNMGR_DEBUG_ERROR("NSS not ready to accept rule \n");
		spin_lock_bh(&nss_connmgr_ipv6.lock);
		nss_connmgr_ipv6.debug_stats[NSS_CONNMGR_IPV6_CREATE_FAIL]++;
		spin_unlock_bh(&nss_connmgr_ipv6.lock);
	} else {
		NSS_CONNMGR_DEBUG_TRACE("NSS create rule failed  skb: %p\n", skb);
		spin_lock_bh(&nss_connmgr_ipv6.lock);
		nss_connmgr_ipv6.debug_stats[NSS_CONNMGR_IPV6_CREATE_FAIL]++;
		spin_unlock_bh(&nss_connmgr_ipv6.lock);
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
 * nss_connmgr_ipv6_post_routing_hook()
 *	Called for packets about to leave the box - either locally generated or forwarded from another interface
 */
static unsigned int nss_connmgr_ipv6_post_routing_hook(unsigned int hooknum,
		struct sk_buff *skb,
		const struct net_device *in_unused,
		const struct net_device *out,
		int (*okfn)(struct sk_buff *))
{
	struct nss_ipv6_create unic;

	struct net_device *in;
	struct nf_conn *ct;
	enum ip_conntrack_info ctinfo;
	struct net_device *src_dev;
	struct net_device *dest_dev;
	struct net_device *rt_dev, *br_port_dev;
	struct nf_conntrack_tuple orig_tuple;
	struct nf_conntrack_tuple reply_tuple;
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
	if (nss_connmgr_ipv6.need_mark && (nss_connmgr_ipv6.need_mark != skb->mark)) {
		NSS_CONNMGR_DEBUG_TRACE("Mark %x not seen, ignoring: %p\n", nss_connmgr_ipv6.need_mark, skb);
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

#if (NSS_CONNMGR_PPPOE_SUPPORT == 1)

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
#endif

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
	 * Get addressing information
	 */
	IN6_ADDR_TO_IPV6_ADDR(unic.src_ip, orig_tuple.src.u3.in6);
	IN6_ADDR_TO_IPV6_ADDR(unic.dest_ip, orig_tuple.dst.u3.in6);

	unic.flags = 0;

	/*
	 * Store the skb->priority as the qos tag
	 */
	unic.qos_tag = (uint32_t)skb->priority;

	/*
	 * Always a routed path
	 */
	unic.flags |= NSS_IPV6_CREATE_FLAG_ROUTED;

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

			unic.flags |= NSS_IPV6_CREATE_FLAG_NO_SEQ_CHECK;
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
		usk = udp_sk(sk);
		break;

	case IPPROTO_IPIP:
		unic.src_port = 0;
		unic.dest_port = 0;
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
	 * Get MAC addresses
	 * NOTE: We are dealing with the ORIGINAL direction here so 'in' and 'out' dev may need
	 * to be swapped if this packet is a reply
	 */
	if (ctinfo < IP_CT_IS_REPLY) {
		src_dev = in;
		dest_dev = new_out;
		NSS_CONNMGR_DEBUG_TRACE("%p: dir: Original\n", ct);
	} else {
		src_dev = new_out;
		dest_dev = in;
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
		unic.flow_pppoe_session_id = (uint16_t)ppp_get_session_id(ppp_src);
		memcpy(unic.flow_pppoe_remote_mac, (uint8_t *)ppp_get_remote_mac(ppp_src), ETH_ALEN);
	} else {
		if (nss_connmgr_ipv6_mac_addr_get(unic.src_ip, unic.src_mac)) {
			NSS_CONNMGR_DEBUG_TRACE("%p: Failed to find MAC address for src IP: %pI6\n", ct, &unic.src_ip);
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
		unic.return_pppoe_session_id = (uint16_t)ppp_get_session_id(ppp_dest);
		memcpy(unic.return_pppoe_remote_mac, (uint8_t *)ppp_get_remote_mac(ppp_dest), ETH_ALEN);
	} else {
		if (nss_connmgr_ipv6_mac_addr_get(unic.dest_ip, unic.dest_mac)) {
			NSS_CONNMGR_DEBUG_TRACE("%p: Failed to find MAC address for dest IP: %pI6\n", ct, &unic.dest_ip);
			goto out;
		}
	}

	/*
	 * Access the ingress routed interface
	 */
	rt_dev = nss_connmgr_get_dev_from_ipv6_address(dev_net(in), in->ifindex, ipv6_hdr(skb)->saddr);
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
						    (void *)&unic.dest_ip,
						    (void *)&unic.src_ip,
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
						     unic.dest_mac,
						     (void *)&unic.src_ip,
						     (void *)&unic.dest_ip,
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
						    (void *)&unic.dest_ip,
						    (void *)&unic.src_ip,
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
						     unic.dest_mac,
						     (void *)&unic.src_ip,
						     (void *)&unic.dest_ip,
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
	 * Only devices that are NSS devices may be accelerated.
	 */
	unic.src_interface_num = nss_cmn_get_interface_number(nss_connmgr_ipv6.nss_context, src_dev);
	if (unic.src_interface_num < 0) {
		goto out;
	}

	unic.dest_interface_num = nss_cmn_get_interface_number(nss_connmgr_ipv6.nss_context, dest_dev);
	if (unic.dest_interface_num < 0) {
		goto out;
	}

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
			"src_ip: " IPV6_ADDR_OCTAL_FMT ":%d\n"
			"dest_ip: " IPV6_ADDR_OCTAL_FMT ":%d\n"
			"src_mac: " MAC_FMT "\n"
			"dest_mac: " MAC_FMT "\n"
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
			IPV6_ADDR_TO_OCTAL(unic.src_ip), unic.src_port,
			IPV6_ADDR_TO_OCTAL(unic.dest_ip), unic.dest_port,
			MAC_AS_BYTES(unic.src_mac),
			MAC_AS_BYTES(unic.dest_mac),
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
		unic.flags |= NSS_IPV6_CREATE_FLAG_DSCP_MARKING;
	if(unic.vlan_oval != NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED)
		unic.flags |= NSS_IPV6_CREATE_FLAG_VLAN_MARKING;

	/*
	 * Create the Network Accelerator connection cache entries
	 *
	 * NOTE: All of the information we have is from the point of view of who created the connection however
	 * the skb may actually be in the 'reply' direction - which is important to know when configuring the NSS as we have to set the right information
	 * on the right "match" and "forwarding" entries.
	 * We can use the ctinfo to determine which direction the skb is in and then swap fields as necessary.
	 */
	IPV6_ADDR_NTOH(unic.src_ip, unic.src_ip);
	IPV6_ADDR_NTOH(unic.dest_ip, unic.dest_ip);
	unic.src_port = ntohs(unic.src_port);
	unic.dest_port = ntohs(unic.dest_port);

	/*
	 * If operations have stopped then do not process packets
	 */
	spin_lock_bh(&nss_connmgr_ipv6.lock);
	if (unlikely(nss_connmgr_ipv6.stopped)) {
		spin_unlock_bh(&nss_connmgr_ipv6.lock);
		NSS_CONNMGR_DEBUG_TRACE("Stopped, ignoring: %p\n", skb);
		goto out;
	}
	spin_unlock_bh(&nss_connmgr_ipv6.lock);

	/*
	 * Calling ipv6_rule1 for DSCP and VLAN marking else we use the regular
	 * ipv6_rule
	 */
	if(unic.dscp_oval == NSS_CONNMGR_DSCP_MARKING_NOT_CONFIGURED &&
		unic.vlan_oval == NSS_CONNMGR_VLAN_MARKING_NOT_CONFIGURED) {
		nss_tx_status = nss_tx_create_ipv6_rule(nss_connmgr_ipv6.nss_context, &unic);
	} else {
		nss_tx_status = nss_tx_create_ipv6_rule1(nss_connmgr_ipv6.nss_context, &unic);
	}

	if (nss_tx_status == NSS_TX_SUCCESS) {
		goto out;
	} else if (nss_tx_status == NSS_TX_FAILURE_NOT_READY) {
		NSS_CONNMGR_DEBUG_ERROR("NSS not ready to accept rule \n");
		spin_lock_bh(&nss_connmgr_ipv6.lock);
		nss_connmgr_ipv6.debug_stats[NSS_CONNMGR_IPV6_CREATE_FAIL]++;
		spin_unlock_bh(&nss_connmgr_ipv6.lock);
	} else {
		NSS_CONNMGR_DEBUG_TRACE("NSS create rule failed  skb: %p\n", skb);
		spin_lock_bh(&nss_connmgr_ipv6.lock);
		nss_connmgr_ipv6.debug_stats[NSS_CONNMGR_IPV6_CREATE_FAIL]++;
		spin_unlock_bh(&nss_connmgr_ipv6.lock);
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
 * nss_connmgr_ipv6_update_bridge_dev()
 *	Update bridge device at host with packet statistics and refresh bridge MAC entries
 */
void nss_connmgr_ipv6_update_bridge_dev(struct nss_connmgr_ipv6_connection *connection, struct nss_ipv6_sync *sync)
{
	struct net_device *indev, *outdev;
	struct rtnl_link_stats64 stats;

	indev = nss_cmn_get_interface_dev(nss_connmgr_ipv6.nss_context, connection->src_interface);
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

	outdev = nss_cmn_get_interface_dev(nss_connmgr_ipv6.nss_context, connection->dest_interface);
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
		if (!sync->final_sync && sync->flow_rx_packet_count) {
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
		if (!sync->final_sync && sync->return_rx_packet_count) {
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
 * nss_connmgr_ipv6_update_vlan_dev_stats()
 *	Update VLAN device statistics
 */
void nss_connmgr_ipv6_update_vlan_dev_stats(struct nss_connmgr_ipv6_connection *connection, struct nss_ipv6_sync *sync)
{
	struct net_device *vlandev, *physdev;
	struct rtnl_link_stats64 stats;

	if (connection->ingress_vlan_tag != NSS_CONNMGR_VLAN_ID_NOT_CONFIGURED)
	{
		physdev = nss_cmn_get_interface_dev(nss_connmgr_ipv6.nss_context, connection->src_interface);
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
		physdev = nss_cmn_get_interface_dev(nss_connmgr_ipv6.nss_context, connection->dest_interface);
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
 * nss_connmgr_ipv6_net_dev_callback()
 *	Callback handler from the linux device driver for the Network Accelerator.
 *
 * The NSS passes sync and stats information to the device driver - which then passes them
 * onto this callback.
 * This arrangement means that dependencies on driver and conntrack modules are solely in this module.
 * These callbacks aim to keep conntrack connections alive and to ensure that connection
 * state - especially for TCP connections that track sequence space for example - is kept in synchronisation
 * before packets flow back through linux from a period of NSS processing.
 */
static void nss_connmgr_ipv6_net_dev_callback(struct nss_ipv6_cb_params *nicp)
{
	struct nf_conntrack_tuple_hash *h;
	struct nf_conntrack_tuple tuple;
	struct nf_conn *ct;
	struct nf_conn_counter *acct;
	struct nss_connmgr_ipv6_connection *connection;

	struct nss_ipv6_sync *sync;
	struct nss_ipv6_establish *establish;
	struct neighbour *neigh;

	ipv6_addr_t daddr;

	switch(nicp->reason)
	{
		case NSS_IPV6_CB_REASON_ESTABLISH:
			establish = &nicp->params.establish;

			if (unlikely(establish->index >= NSS_CONNMGR_IPV6_CONN_MAX)) {
				NSS_CONNMGR_DEBUG_TRACE("Bad establish index: %d\n", establish->index);
				return;
			}

			connection = &nss_connmgr_ipv6.connection[establish->index];

			spin_lock_bh(&nss_connmgr_ipv6.lock);
			if (unlikely(connection->state == NSS_CONNMGR_IPV6_STATE_ESTABLISHED)) {
				spin_unlock_bh(&nss_connmgr_ipv6.lock);
				NSS_CONNMGR_DEBUG_WARN("Invalid sync callback. Conn already established : %d \n", establish->index);
				return;
			}

			connection->state = NSS_CONNMGR_IPV6_STATE_ESTABLISHED;
			nss_connmgr_ipv6.debug_stats[NSS_CONNMGR_IPV6_ACTIVE_CONN]++;

			connection->protocol = establish->protocol;

			connection->src_interface = establish->flow_interface;
			connection->src_addr[0] = establish->flow_ip[0];
			connection->src_addr[1] = establish->flow_ip[1];
			connection->src_addr[2] = establish->flow_ip[2];
			connection->src_addr[3] = establish->flow_ip[3];
			connection->src_port = establish->flow_ident;

			connection->dest_interface = establish->return_interface;
			connection->dest_addr[0] = establish->return_ip[0];
			connection->dest_addr[1] = establish->return_ip[1];
			connection->dest_addr[2] = establish->return_ip[2];
			connection->dest_addr[3] = establish->return_ip[3];
			connection->dest_port = establish->return_ident;

			connection->ingress_vlan_tag = establish->ingress_vlan_tag;
			connection->egress_vlan_tag = establish->egress_vlan_tag;
			memcpy(connection->src_mac_addr, (char *)establish->flow_mac, ETH_ALEN);
			memcpy(connection->dest_mac_addr, (char *)establish->return_mac, ETH_ALEN);

			connection->stats[NSS_CONNMGR_IPV6_ACCELERATED_RX_PKTS] = 0;
			connection->stats[NSS_CONNMGR_IPV6_ACCELERATED_RX_BYTES] = 0;
			connection->stats[NSS_CONNMGR_IPV6_ACCELERATED_TX_PKTS] = 0;
			connection->stats[NSS_CONNMGR_IPV6_ACCELERATED_TX_BYTES] = 0;

			spin_unlock_bh(&nss_connmgr_ipv6.lock);

			return;

		case NSS_IPV6_CB_REASON_SYNC:
			sync = &nicp->params.sync;

			if (unlikely(sync->index >= NSS_CONNMGR_IPV6_CONN_MAX)) {
				NSS_CONNMGR_DEBUG_TRACE("Bad sync index: %d\n", sync->index);
				return;
			}

			connection = &nss_connmgr_ipv6.connection[sync->index];

			NSS_CONNMGR_DEBUG_TRACE("NSS ipv6 Sync callback %d \n", sync->index);

			spin_lock_bh(&nss_connmgr_ipv6.lock);
			if (unlikely(connection->state != NSS_CONNMGR_IPV6_STATE_ESTABLISHED)) {
				spin_unlock_bh(&nss_connmgr_ipv6.lock);
				NSS_CONNMGR_DEBUG_WARN("Invalid sync callback. Conn not established : %d \n", sync->index);
				return;
			}
			spin_unlock_bh(&nss_connmgr_ipv6.lock);

			break;

		default:
			NSS_CONNMGR_DEBUG_WARN("%d: Unhandled callback\n", nicp->reason);
			return;
	}

	spin_lock_bh(&nss_connmgr_ipv6.lock);
	connection->stats[NSS_CONNMGR_IPV6_ACCELERATED_RX_PKTS] += sync->flow_rx_packet_count + sync->return_rx_packet_count;
	connection->stats[NSS_CONNMGR_IPV6_ACCELERATED_RX_BYTES] += sync->flow_rx_byte_count + sync->return_rx_byte_count;
	connection->stats[NSS_CONNMGR_IPV6_ACCELERATED_TX_PKTS]+= sync->flow_tx_packet_count + sync->return_tx_packet_count;
	connection->stats[NSS_CONNMGR_IPV6_ACCELERATED_TX_BYTES] += sync->flow_tx_byte_count + sync->return_tx_byte_count;

	/*
	 * Update VLAN device statistics if required
	 */
	if ((connection->ingress_vlan_tag & connection->egress_vlan_tag) != NSS_CONNMGR_VLAN_ID_NOT_CONFIGURED) {
		nss_connmgr_ipv6_update_vlan_dev_stats(connection, sync);
	}

	/*
	 * Update bridge device if required
	 */
	nss_connmgr_ipv6_update_bridge_dev(connection, sync);

	if (sync->final_sync) {
		connection->state = NSS_CONNMGR_IPV6_STATE_INACTIVE;
		nss_connmgr_ipv6.debug_stats[NSS_CONNMGR_IPV6_ACTIVE_CONN]--;
	}
	spin_unlock_bh(&nss_connmgr_ipv6.lock);


	/*
	 * Create a tuple so as to be able to look up a connection
	 */
	memset(&tuple, 0, sizeof(tuple));
	IPV6_ADDR_NTOH_IN6_ADDR(tuple.src.u3.in6, connection->src_addr);
	tuple.src.u.all = ntohs((__be16)connection->src_port);
	tuple.src.l3num = AF_INET6;

	IPV6_ADDR_NTOH_IN6_ADDR(tuple.dst.u3.in6, connection->dest_addr);
	tuple.dst.u.all = ntohs((__be16)connection->dest_port);
	tuple.dst.dir = IP_CT_DIR_ORIGINAL;

	tuple.dst.protonum = (uint8_t)connection->protocol;

	NSS_CONNMGR_DEBUG_TRACE("\nNSS Update, lookup connection using\n"
			"Protocol: %d\n"
			"src_addr: %pI6:%d\n"
			"dest_addr: %pI6:%d\n",
			(int)tuple.dst.protonum,
			&(tuple.src.u3.in6), (int)tuple.src.u.all,
			&(tuple.dst.u3.in6), (int)tuple.dst.u.all);

	/*
	 * Look up conntrack connection
	 */
	h = nf_conntrack_find_get(&init_net, NF_CT_DEFAULT_ZONE, &tuple);
	if (!h) {
		NSS_CONNMGR_DEBUG_WARN("Conntrack could not be found in sync");
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

	/*
	 * Update neighbour entry for source IP address.
	 */
	IPV6_ADDR_NTOH(daddr, connection->src_addr);
	neigh = nss_connmgr_ipv6_neigh_get(daddr);

	if (neigh) {
		if (!sync->final_sync) {
			neigh_update(neigh, NULL, neigh->nud_state, NEIGH_UPDATE_F_WEAK_OVERRIDE);
		}
		neigh_release(neigh);
	} else {
		NSS_CONNMGR_DEBUG_TRACE("Neighbour entry could not be found for onward flow\n");
	}

	/*
	 * Update neighbour entry for destination IP address.
	 */
	IPV6_ADDR_NTOH(daddr, connection->dest_addr);
	neigh = nss_connmgr_ipv6_neigh_get(daddr);

	if (neigh) {
		if (!sync->final_sync) {
			neigh_update(neigh, NULL, neigh->nud_state, NEIGH_UPDATE_F_WEAK_OVERRIDE);
		}
		neigh_release(neigh);
	} else {
		NSS_CONNMGR_DEBUG_TRACE("Neighbour entry could not be found for return flow\n");
	}

	/*
	 * Release connection
	 */
	nf_ct_put(ct);

	return;
}

#ifdef CONFIG_NF_CONNTRACK_EVENTS
/*
 * nss_connmgr_ipv6_conntrack_event()
 *	Callback event invoked when conntrack connection state changes, currently we handle destroy events to quickly release state
 */
#ifdef CONFIG_NF_CONNTRACK_CHAIN_EVENTS
static int nss_connmgr_ipv6_conntrack_event(struct notifier_block *this,
		unsigned long events, void *ptr)
#else
static int nss_connmgr_ipv6_conntrack_event(struct nf_conn *ct)
#endif
{
#ifdef CONFIG_NF_CONNTRACK_CHAIN_EVENTS
        struct nf_conn *ct = ((struct nf_ct_event *)ptr)->ct;
#endif
	struct nss_ipv6_destroy unid;
	struct nf_conntrack_tuple orig_tuple;
	struct nf_conntrack_tuple reply_tuple;

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

#ifdef CONFIG_NF_CONNTRACK_CHAIN_EVENTS
	/*
	 * Only interested in destroy events
	 */
	if (!(events & (1 << IPCT_DESTROY))) {
		return NOTIFY_DONE;
	}

	/*
	 * Only interested if this is IPv4
	 */
	if (nf_ct_l3num(ct) != AF_INET6) {
		return NOTIFY_DONE;
	}
#endif

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
	IN6_ADDR_TO_IPV6_ADDR(unid.src_ip, orig_tuple.src.u3.in6);
	IN6_ADDR_TO_IPV6_ADDR(unid.dest_ip, orig_tuple.dst.u3.in6);

	switch (unid.protocol) {
	case IPPROTO_TCP:
		unid.src_port = (int32_t)orig_tuple.src.u.tcp.port;
		unid.dest_port = (int32_t)orig_tuple.dst.u.tcp.port;
		break;

	case IPPROTO_UDP:
		unid.src_port = (int32_t)orig_tuple.src.u.udp.port;
		unid.dest_port = (int32_t)orig_tuple.dst.u.udp.port;
		break;

	case IPPROTO_IPIP:
		unid.src_port = 0;
		unid.dest_port = 0;
		break;

	default:
		/*
		 * Database stores non-ported protocols with port numbers equal to negative protocol number
		 * to indicate unused.
		 */
		NSS_CONNMGR_DEBUG_TRACE("%p: Unhandled protocol %d\n", ct, unid.protocol);
		unid.src_port = -unid.protocol;
		unid.dest_port = -unid.protocol;
	}

	/*
	 * Only deal with TCP or UDP or IP Tunnel
	 */
	if ((unid.protocol != IPPROTO_TCP) && (unid.protocol != IPPROTO_UDP) && (unid.protocol !=  IPPROTO_IPIP)) {
		return NOTIFY_DONE;
	}

	NSS_CONNMGR_DEBUG_TRACE("\n%p: Connection destroyed\n"
			"Protocol: %d\n"
			"src_ip: " IPV6_ADDR_OCTAL_FMT ":%d\n"
			"dest_ip: " IPV6_ADDR_OCTAL_FMT ":%d\n",
			ct,
			unid.protocol,
			IPV6_ADDR_TO_OCTAL(unid.src_ip), unid.src_port,
			IPV6_ADDR_TO_OCTAL(unid.dest_ip), unid.dest_port);

	/*
	 * Destroy the Network Accelerator connection cache entries.
	 */
	unid.src_ip[0] = ntohl(unid.src_ip[0]);
	unid.src_ip[1] = ntohl(unid.src_ip[1]);
	unid.src_ip[2] = ntohl(unid.src_ip[2]);
	unid.src_ip[3] = ntohl(unid.src_ip[3]);
	unid.dest_ip[0] = ntohl(unid.dest_ip[0]);
	unid.dest_ip[1] = ntohl(unid.dest_ip[1]);
	unid.dest_ip[2] = ntohl(unid.dest_ip[2]);
	unid.dest_ip[3] = ntohl(unid.dest_ip[3]);
	unid.src_port = ntohs(unid.src_port);
	unid.dest_port = ntohs(unid.dest_port);
	/*
	 * Destroy the Network Accelerator connection cache entries.
	 */
	nss_tx_status = nss_tx_destroy_ipv6_rule(nss_connmgr_ipv6.nss_context, &unid);
	if (nss_tx_status == NSS_TX_SUCCESS) {
		goto out;
	} else if (nss_tx_status == NSS_TX_FAILURE_NOT_READY) {
		NSS_CONNMGR_DEBUG_ERROR("NSS not ready to accept 'destroy IPv6' rule \n");
		spin_lock_bh(&nss_connmgr_ipv6.lock);
		nss_connmgr_ipv6.debug_stats[NSS_CONNMGR_IPV6_DESTROY_FAIL]++;
		spin_unlock_bh(&nss_connmgr_ipv6.lock);
	} else {
		NSS_CONNMGR_DEBUG_TRACE("NSS destroy rule fail \n");
		spin_lock_bh(&nss_connmgr_ipv6.lock);
		nss_connmgr_ipv6.debug_stats[NSS_CONNMGR_IPV6_DESTROY_FAIL]++;
		spin_unlock_bh(&nss_connmgr_ipv6.lock);
	}
out:
	return NOTIFY_DONE;
}

#endif
/*
 * nss_connmgr_ipv6_init_connection_table()
 *      initialize connection table
 */
static void nss_connmgr_ipv6_init_connection_table(void)
{
	int32_t i,j;

	spin_lock_bh(&nss_connmgr_ipv6.lock);

	for (i = 0; (i < NSS_CONNMGR_IPV6_CONN_MAX); i++) {
		nss_connmgr_ipv6.connection[i].state = NSS_CONNMGR_IPV6_STATE_INACTIVE;

		for (j = 0; (j < NSS_CONNMGR_IPV6_STATS_MAX); j++) {
			nss_connmgr_ipv6.connection[i].stats[j] = 0;
		}
	}

	for (i = 0; (i < NSS_CONNMGR_IPV6_DEBUG_STATS_MAX); i++) {
		nss_connmgr_ipv6.debug_stats[i] = 0;
	}

	spin_unlock_bh(&nss_connmgr_ipv6.lock);

	return;
}

/*
 * nss_connmgr_ipv6_get_terminate()
 */
static ssize_t nss_connmgr_ipv6_get_terminate(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	ssize_t count;
	int num;


	/*
	 * Operate under our locks
	 */
	spin_lock_bh(&nss_connmgr_ipv6.lock);
	num = nss_connmgr_ipv6.terminate;
	spin_unlock_bh(&nss_connmgr_ipv6.lock);

	count = snprintf(buf, (ssize_t)PAGE_SIZE, "%d\n", num);
	return count;
}

/*
 * nss_connmgr_ipv6_set_terminate()
 *	Writing anything to this 'file' will cause the default classifier to terminate
 */
static ssize_t nss_connmgr_ipv6_set_terminate(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{


	NSS_CONNMGR_DEBUG_INFO("Terminate\n");
	spin_lock_bh(&nss_connmgr_ipv6.lock);
	nss_connmgr_ipv6.terminate = 1;
	wake_up_process(nss_connmgr_ipv6.thread);
	spin_unlock_bh(&nss_connmgr_ipv6.lock);

	return count;
}

/*
 * nss_connmgr_ipv6_get_stop()
 */
static ssize_t nss_connmgr_ipv6_get_stop(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	ssize_t count;
	int num;

	/*
	 * Operate under our locks
	 */
	spin_lock_bh(&nss_connmgr_ipv6.lock);
	num = nss_connmgr_ipv6.stopped;
	spin_unlock_bh(&nss_connmgr_ipv6.lock);

	count = snprintf(buf, (ssize_t)PAGE_SIZE, "%d\n", num);
	return count;
}

/*
 * nss_connmgr_ipv6_set_stop()
 */
static ssize_t nss_connmgr_ipv6_set_stop(struct device *dev,
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
	NSS_CONNMGR_DEBUG_TRACE("nss_connmgr_ipv6_stop = %d\n", num);

	/*
	 * Operate under our locks and stop further processing of packets
	 */
	spin_lock_bh(&nss_connmgr_ipv6.lock);
	nss_connmgr_ipv6.stopped = num;
	spin_unlock_bh(&nss_connmgr_ipv6.lock);

	return count;
}

/*
 * nss_connmgr_ipv6_get_need_mark()
 * 	Get the value of "need_mark" operational control variable
 */
static ssize_t nss_connmgr_ipv6_get_need_mark(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	ssize_t count;
	uint32_t num;

	/*
	 * Operate under our locks
	 */
	spin_lock_bh(&nss_connmgr_ipv6.lock);
	num = nss_connmgr_ipv6.need_mark;
	spin_unlock_bh(&nss_connmgr_ipv6.lock);

	count = snprintf(buf, (ssize_t)PAGE_SIZE, "%x\n", num);
	return count;
}

/*
 * nss_connmgr_ipv6_set_need_mark()
 * 	Set the value of "need_mark" operational control variable.
 */
static ssize_t nss_connmgr_ipv6_set_need_mark(struct device *dev,
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
	NSS_CONNMGR_DEBUG_TRACE("nss_connmgr_ipv6_need_mark = %x\n", num);

	/*
	 * Operate under our locks and stop further processing of packets
	 */
	spin_lock_bh(&nss_connmgr_ipv6.lock);
	nss_connmgr_ipv6.need_mark = num;
	spin_unlock_bh(&nss_connmgr_ipv6.lock);

	return count;
}

/*
 * SysFS attributes for the default classifier itself.
 */
static const struct device_attribute nss_connmgr_ipv6_terminate_attr =
		__ATTR(terminate, S_IWUGO | S_IRUGO, nss_connmgr_ipv6_get_terminate, nss_connmgr_ipv6_set_terminate);
static const struct device_attribute nss_connmgr_ipv6_stop_attr =
		__ATTR(stop, S_IWUGO | S_IRUGO, nss_connmgr_ipv6_get_stop, nss_connmgr_ipv6_set_stop);
static const struct device_attribute nss_connmgr_ipv6_need_mark_attr =
		__ATTR(need_mark, S_IWUGO | S_IRUGO, nss_connmgr_ipv6_get_need_mark, nss_connmgr_ipv6_set_need_mark);

/*
 * nss_connmgr_ipv6_thread_fn()
 *	A thread to handle tasks that can only be done in thread context.
 */
static int nss_connmgr_ipv6_thread_fn(void *arg)
{
	int result = 0;


	NSS_CONNMGR_DEBUG_INFO("NSS Connection manager thread start\n");

	/*
	 * Get reference to this module - we release it when the thread exits
	 */
	try_module_get(THIS_MODULE);

	/*
	 * Create /sys/nom_v6
	 */
	nss_connmgr_ipv6.nom_v6 = kobject_create_and_add("nom_v6", NULL);
	if (unlikely(nss_connmgr_ipv6.nom_v6 == NULL)) {
		NSS_CONNMGR_DEBUG_ERROR("Failed to register nom_v6\n");
		goto task_cleanup_1;
	}

	/*
	 * Create files, one for each parameter supported by this module
	 */
	result = sysfs_create_file(nss_connmgr_ipv6.nom_v6, &nss_connmgr_ipv6_terminate_attr.attr);
	if (result) {
		NSS_CONNMGR_DEBUG_ERROR("Failed to register terminate file %d\n", result);
		goto task_cleanup_2;
	}

	/*
	 * Register this module with the Linux NSS driver (net_device)
	 */
	nss_connmgr_ipv6.nss_context = nss_register_ipv6_mgr(nss_connmgr_ipv6_net_dev_callback);

	/*
	 * Initialize connection table
	 */
	nss_connmgr_ipv6_init_connection_table();

	/*
	 * Register netfilter hooks - ipv6
	 */
	result = nf_register_hooks(nss_connmgr_ipv6_ops_post_routing, ARRAY_SIZE(nss_connmgr_ipv6_ops_post_routing));
	if (result < 0) {
		NSS_CONNMGR_DEBUG_ERROR("Can't register nf post routing hook %d\n", result);
		goto task_cleanup_3;
	}

	result = sysfs_create_file(nss_connmgr_ipv6.nom_v6, &nss_connmgr_ipv6_stop_attr.attr);
	if (result) {
		NSS_CONNMGR_DEBUG_ERROR("Failed to register stop file %d\n", result);
		goto task_cleanup_4;
	}

	/*
	 * We register a callback with ipv4 connection manager module
	 * to get NF conntrack notifications of expired connections
	 */
#if defined(CONFIG_NF_CONNTRACK_EVENTS) && !defined(CONFIG_NF_CONNTRACK_CHAIN_EVENTS)
	nss_connmgr_ipv4_register_conntrack_event_cb(nss_connmgr_ipv6_conntrack_event);
#elif defined(CONFIG_NF_CONNTRACK_CHAIN_EVENTS)
	result = nf_conntrack_register_notifier(&init_net, &nss_connmgr_ipv6.conntrack_notifier);
	if (result < 0) {
		NSS_CONNMGR_DEBUG_ERROR("Can't register nf notifier hook %d\n", result);
		goto task_cleanup_5;
	}
#endif

	nss_connmgr_ipv6.netdev_notifier.notifier_call = nss_connmgr_netdev_notifier_cb;
	result = register_netdevice_notifier(&nss_connmgr_ipv6.netdev_notifier);
	if (result != 0) {
		NSS_CONNMGR_DEBUG_ERROR("Can't register ipv6 netdevice notifier %d\n", result);
		goto task_cleanup_6;
	}

	result = sysfs_create_file(nss_connmgr_ipv6.nom_v6, &nss_connmgr_ipv6_need_mark_attr.attr);
	if (result) {
		NSS_CONNMGR_DEBUG_ERROR("Failed to register need mark file %d\n", result);
		goto task_cleanup_7;
	}

	nss_connmgr_ipv4_register_bond_slave_linkup_cb(nss_connmgr_bond_link_up);

	/*
	 * Allow wakeup signals
	 */
	allow_signal(SIGCONT);
	spin_lock_bh(&nss_connmgr_ipv6.lock);
	while (!nss_connmgr_ipv6.terminate) {
		/*
		 * Sleep and wait for an instruction
		 */
		spin_unlock_bh(&nss_connmgr_ipv6.lock);
		NSS_CONNMGR_DEBUG_TRACE("nss_connmgr_ipv6 sleep\n");
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
		spin_lock_bh(&nss_connmgr_ipv6.lock);
	}
	spin_unlock_bh(&nss_connmgr_ipv6.lock);
	NSS_CONNMGR_DEBUG_INFO("nss_connmgr_ipv6 terminate\n");

	nss_connmgr_ipv4_unregister_bond_slave_linkup_cb();

	sysfs_remove_file(nss_connmgr_ipv6.nom_v6, &nss_connmgr_ipv6_need_mark_attr.attr);
task_cleanup_7:
	unregister_netdevice_notifier(&nss_connmgr_ipv6.netdev_notifier);
task_cleanup_6:
#if defined(CONFIG_NF_CONNTRACK_EVENTS) && !defined(CONFIG_NF_CONNTRACK_CHAIN_EVENTS)
	nss_connmgr_ipv4_unregister_conntrack_event_cb();
#elif defined(CONFIG_NF_CONNTRACK_CHAIN_EVENTS)
	nf_conntrack_unregister_notifier(&init_net, &nss_connmgr_ipv6.conntrack_notifier);
task_cleanup_5:
#endif
	sysfs_remove_file(nss_connmgr_ipv6.nom_v6, &nss_connmgr_ipv6_stop_attr.attr);
task_cleanup_4:
	nf_unregister_hooks(nss_connmgr_ipv6_ops_post_routing, ARRAY_SIZE(nss_connmgr_ipv6_ops_post_routing));
task_cleanup_3:
	nss_unregister_ipv6_mgr();
	sysfs_remove_file(nss_connmgr_ipv6.nom_v6, &nss_connmgr_ipv6_terminate_attr.attr);
task_cleanup_2:
	kobject_put(nss_connmgr_ipv6.nom_v6);
task_cleanup_1:

	module_put(THIS_MODULE);
	return result;
}

/*
 * nss_connmgr_ipv6_read_stats
 *      Read ipv6 stats
 */
static ssize_t nss_connmgr_ipv6_read_stats(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	int32_t i;
	int32_t j;
	int32_t k;
	size_t size_al = NSS_CONNMGR_IPV6_DEBUGFS_BUF_SZ;
	size_t size_wr = 0;
	ssize_t bytes_read = 0;

	/*
	 * Temporary array to hold statistics for printing. To avoid holding
	 * spinlocks for long time while printing, we copy the stats to this
	 * temporary buffer while holding the spinlock, unlock and then print.
	 * Also to avoid using big chunk of memory on stack, we use an array of
	 * only 8 connections, and print stats for 8 connections at a time
	 */
	uint64_t stats[8][NSS_CONNMGR_IPV6_STATS_MAX];
	uint8_t state[8];
	uint32_t src_addr[8][4];
	int32_t  src_port[8];
	uint32_t dest_addr[8][4];
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
	size_wr = scnprintf(lbuf, size_al,"connection mgr ipv6 stats :\n\n");

	for (i = 0; (i < NSS_CONNMGR_IPV6_CONN_MAX) && (size_wr < size_al) ; i+=8) {

		/* 1. Take a lock */
		spin_lock_bh(&nss_connmgr_ipv6.lock);

		/* 2. Copy stats for 8 connections into local buffer */
		for (j = 0; j < 8; j++) {
			state[j] = nss_connmgr_ipv6.connection[i+j].state;
			src_port[j] = nss_connmgr_ipv6.connection[i+j].src_port;
			dest_port[j] = nss_connmgr_ipv6.connection[i+j].dest_port;
			memcpy(src_addr[j], nss_connmgr_ipv6.connection[i+j].src_addr, 16);
			memcpy(dest_addr[j], nss_connmgr_ipv6.connection[i+j].dest_addr, 16);
			protocol[j] = nss_connmgr_ipv6.connection[i+j].protocol;
		}

		memcpy(stats, nss_connmgr_ipv6.connection[i].stats, sizeof(stats));

		/* 3. Release the lock */
		spin_unlock_bh(&nss_connmgr_ipv6.lock);

		/* 4. Print stats for 8 connections */
		for (j = 0; (j < 8); j++) {

			if (state[j] != NSS_CONNMGR_IPV6_STATE_ESTABLISHED) {
				continue;
			}

			IPV6_ADDR_NTOH(src_addr[j], src_addr[j]);
			IPV6_ADDR_NTOH(dest_addr[j], dest_addr[j]);

			size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "---------------------------------\n");
			if (protocol[j] == IPPROTO_TCP) {
				size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "tcp ");
			} else if (protocol[j] == IPPROTO_UDP) {
				size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "udp ");
			} else {
				size_wr += scnprintf(lbuf + size_wr, size_al - size_wr, "proto=%d ",protocol[j]);
			}

			size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"src_addr: %pI6:%d\n"
					"dest_addr: %pI6:%d\n",
					&(src_addr[j]), (int)ntohs(src_port[j]),
					&(dest_addr[j]), (int)ntohs(dest_port[j]));

			for (k = 0; (k < NSS_CONNMGR_IPV6_STATS_MAX); k++) {
				size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
						"%s = %llu\n", nss_connmgr_ipv6_conn_stats_str[k], stats[j][k]);
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
 * nss_connmgr_ipv6_read_debug_stats
 *	Read connection manager debug stats
 */
static ssize_t nss_connmgr_ipv6_read_debug_stats(struct file *fp, char __user *ubuf, size_t sz, loff_t *ppos)
{
	int32_t i;

	/*
	 * max output lines = #stats + start tag line + end tag line + three blank lines
	 */
	uint32_t max_output_lines = NSS_CONNMGR_IPV6_DEBUG_STATS_MAX + 5;
	size_t size_al = NSS_CONNMGR_IPV6_MAX_STR_LENGTH * max_output_lines;
	size_t size_wr = 0;
	ssize_t bytes_read = 0;
	uint32_t stats_shadow[NSS_CONNMGR_IPV6_DEBUG_STATS_MAX];

	char *lbuf = kzalloc(size_al, GFP_KERNEL);
	if (unlikely(lbuf == NULL)) {
		NSS_CONNMGR_DEBUG_WARN("Could not allocate memory for local statistics buffer");
		return 0;
	}

	size_wr = scnprintf(lbuf, size_al,"debug stats start:\n\n");
	spin_lock_bh(&nss_connmgr_ipv6.lock);

	for (i = 0; (i < NSS_CONNMGR_IPV6_DEBUG_STATS_MAX); i++) {
		stats_shadow[i] = nss_connmgr_ipv6.debug_stats[i];
	}

	spin_unlock_bh(&nss_connmgr_ipv6.lock);

	for (i = 0; (i < NSS_CONNMGR_IPV6_DEBUG_STATS_MAX); i++) {
		size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,
					"%s = %u\n", nss_connmgr_ipv6_debug_stats_str[i], stats_shadow[i]);
	}

	size_wr += scnprintf(lbuf + size_wr, size_al - size_wr,"\ndebug stats end\n\n");
	bytes_read = simple_read_from_buffer(ubuf, sz, ppos, lbuf, strlen(lbuf));
	kfree(lbuf);

	return bytes_read;
}

/*
 * nss_connmgr_ipv6_clear_stats
 *      Clear ipv6 stats
 */
static ssize_t nss_connmgr_ipv6_clear_stats(struct file *fp, const char __user *ubuf, size_t count , loff_t *ppos)
{
	int32_t i,j;

	spin_lock_bh(&nss_connmgr_ipv6.lock);

	for (i = 0; (i < NSS_CONNMGR_IPV6_CONN_MAX); i++) {
		for (j = 0; (j < NSS_CONNMGR_IPV6_STATS_MAX); j++) {
			nss_connmgr_ipv6.connection[i].stats[j] = 0;
		}
	}

	for (i = 0; (i < NSS_CONNMGR_IPV6_DEBUG_STATS_MAX); i++) {
		nss_connmgr_ipv6.debug_stats[i] = 0;
	}

	spin_unlock_bh(&nss_connmgr_ipv6.lock);

	return count;
}

/*
 * nss_connmgr_ipv6_module_get()
 *	Take a reference to the module
 */
void nss_connmgr_ipv6_module_get(void)
{
	try_module_get(THIS_MODULE);
}
EXPORT_SYMBOL(nss_connmgr_ipv6_module_get);

/*
 * nss_connmgr_ipv6_module_put()
 *	Release a reference to the module
 */
void nss_connmgr_ipv6_module_put(void)
{
	module_put(THIS_MODULE);
}
EXPORT_SYMBOL(nss_connmgr_ipv6_module_put);

/*
 * nss_connmgr_ipv6_init()
 */
static int __init nss_connmgr_ipv6_init(void)
{
	struct dentry *dent, *dfile;

	NSS_CONNMGR_DEBUG_INFO("NSS Connection Manager Module init\n");

	/*
	 * Initialise our global database lock
	 */
	spin_lock_init(&nss_connmgr_ipv6.lock);

	/*
	 * Create a thread to handle the start/stop.
	 * NOTE: We use a thread as some things we need to do cannot be done in this context
	 */
	nss_connmgr_ipv6.terminate = 0;
	nss_connmgr_ipv6.thread = kthread_create(nss_connmgr_ipv6_thread_fn, NULL, "%s", "nss_connmgr_ipv6_thread");
	if (!nss_connmgr_ipv6.thread) {
		return -EINVAL;
	}
	wake_up_process(nss_connmgr_ipv6.thread);

	/*
	 * Create debugfs files for stats
	 */
	dent = debugfs_create_dir("qca-nss-connmgr-ipv6", NULL);

	if (unlikely(dent == NULL)) {

		/*
		 * Non-availability of debugfs dir is not catastrophe.
		 * Print a message and gracefully return
		 */
		NSS_CONNMGR_DEBUG_WARN("Failed to create nss_connmgr_ipv6 dir in debugfs \n");
		return 0;
	}

	nss_connmgr_ipv6.dent = dent;
	dfile = debugfs_create_file("connection_stats", S_IRUGO , dent, &nss_connmgr_ipv6, &nss_connmgr_ipv6_show_stats_ops);

	if (unlikely(dfile == NULL))
	{
		NSS_CONNMGR_DEBUG_WARN("Failed to create nss_connmgr_ipv6/connection_stats in debugfs \n");
		debugfs_remove(dent);
	}

	dfile = debugfs_create_file("debug_stats", S_IRUGO , dent, &nss_connmgr_ipv6, &nss_connmgr_ipv6_show_debug_stats_ops);

	if (unlikely(dfile == NULL))
	{
		NSS_CONNMGR_DEBUG_WARN("Failed to create nss_connmgr_ipv6/show_stats in debugfs \n");
		debugfs_remove(dent);
	}

	dfile = debugfs_create_file("clear_stats", S_IRUGO | S_IWUSR , dent, &nss_connmgr_ipv6, &nss_connmgr_ipv6_clear_stats_ops);

	if (unlikely(dfile == NULL))
	{
		NSS_CONNMGR_DEBUG_WARN("Failed to create nss_connmgr_ipv6/clear_stats in debugfs \n");
		debugfs_remove(dent);
	}

	return 0;
}

/*
 * nss_connmgr_ipv6_exit()
 */
static void __exit nss_connmgr_ipv6_exit(void)
{
	/*
	 * Remove debugfs tree
	 */
	if (likely(nss_connmgr_ipv6.dent != NULL)) {
		debugfs_remove_recursive(nss_connmgr_ipv6.dent);
	}

	NSS_CONNMGR_DEBUG_INFO("NSS Connection Manager Module exit\n");
}

module_init(nss_connmgr_ipv6_init);
module_exit(nss_connmgr_ipv6_exit);

MODULE_AUTHOR("Qualcomm Atheros Inc");
MODULE_DESCRIPTION("NSS ipv6 Connection manager");
MODULE_LICENSE("Dual BSD/GPL");

