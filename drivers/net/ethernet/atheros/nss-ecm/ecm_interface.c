/*
 **************************************************************************
 * Copyright (c) 2014,2015 The Linux Foundation.  All rights reserved.
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

#include <linux/version.h>
#include <linux/types.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/icmp.h>
#include <linux/sysctl.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/pkt_sched.h>
#include <linux/string.h>
#include <net/ip6_route.h>
#include <net/ip6_fib.h>
#include <net/ipv6.h>
#include <net/route.h>
#include <net/ip_fib.h>
#include <net/ip.h>
#include <net/tcp.h>
#include <asm/unaligned.h>
#include <asm/uaccess.h>	/* for put_user */
#include <linux/inet.h>
#include <linux/in6.h>
#include <linux/in.h>
#include <linux/udp.h>
#include <linux/tcp.h>


#include <linux/inetdevice.h>
#if defined(ECM_INTERFACE_TUNIPIP6_ENABLE) || defined(ECM_INTERFACE_SIT_ENABLE)
#include <net/ipip.h>
#endif
#include <net/ip6_tunnel.h>
#include <net/addrconf.h>
#include <linux/if_arp.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netfilter_bridge.h>
#include <linux/if_bridge.h>
#include <net/arp.h>
#include <net/netfilter/nf_conntrack.h>
#include <net/netfilter/nf_conntrack_acct.h>
#include <net/netfilter/nf_conntrack_helper.h>
#include <net/netfilter/nf_conntrack_l4proto.h>
#include <net/netfilter/nf_conntrack_l3proto.h>
#include <net/netfilter/nf_conntrack_zones.h>
#include <net/netfilter/nf_conntrack_core.h>
#include <net/netfilter/ipv4/nf_conntrack_ipv4.h>
#include <net/netfilter/ipv4/nf_defrag_ipv4.h>

/*
 * Debug output levels
 * 0 = OFF
 * 1 = ASSERTS / ERRORS
 * 2 = 1 + WARN
 * 3 = 2 + INFO
 * 4 = 3 + TRACE
 */
#define DEBUG_LEVEL ECM_INTERFACE_DEBUG_LEVEL

#include <nss_api_if.h>

#include "ecm_types.h"
#include "ecm_db_types.h"
#include "ecm_tracker.h"
#include "ecm_classifier.h"
#include "ecm_front_end_types.h"
#include "ecm_tracker_datagram.h"
#include "ecm_tracker_udp.h"
#include "ecm_tracker_tcp.h"
#include "ecm_db.h"
#include "ecm_interface.h"


/*
 * Locking - concurrency control
 */
static spinlock_t ecm_interface_lock;			/* Protect against SMP access between netfilter, events and private threaded function. */

/*
 * System device linkage
 */
static struct device ecm_interface_dev;		/* System device linkage */

/*
 * General operational control
 */
static int ecm_interface_stopped = 0;			/* When non-zero further traffic will not be processed */

/*
 * Management thread control
 */
static bool ecm_interface_terminate_pending = false;		/* True when the user has signalled we should quit */

/*
 * ecm_interface_get_and_hold_dev_master()
 *	Returns the master device of a net device if any.
 */
struct net_device *ecm_interface_get_and_hold_dev_master(struct net_device *dev)
{
	struct net_device *master;
	rcu_read_lock();
	master = netdev_master_upper_dev_get_rcu(dev);
	if (!master) {
		rcu_read_unlock();
		return NULL;
	}
	dev_hold(master);
	rcu_read_unlock();
	return master;
}
EXPORT_SYMBOL(ecm_interface_get_and_hold_dev_master);

/*
 * ecm_interface_dev_find_by_local_addr_ipv4()
 *	Return a hold to the device for the given local IP address.  Returns NULL on failure.
 */
static struct net_device *ecm_interface_dev_find_by_local_addr_ipv4(ip_addr_t addr)
{
	__be32 be_addr;
	struct net_device *dev;

	ECM_IP_ADDR_TO_NIN4_ADDR(be_addr, addr);
	dev = ip_dev_find(&init_net, be_addr);
	return dev;
}


/*
 * ecm_interface_dev_find_by_local_addr()
 *	Return the device on which the local address resides.
 *
 * Returns a hold to the device or NULL on failure.
 */
struct net_device *ecm_interface_dev_find_by_local_addr(ip_addr_t addr)
{
	char __attribute__((unused)) addr_str[40];

	DEBUG_ECM_IP_ADDR_TO_STRING(addr_str, addr);
	DEBUG_TRACE("Locate dev for: %s\n", addr_str);

	if (ECM_IP_ADDR_IS_V4(addr)) {
		return ecm_interface_dev_find_by_local_addr_ipv4(addr);
	}

	return NULL;
}
EXPORT_SYMBOL(ecm_interface_dev_find_by_local_addr);

/*
 * ecm_interface_dev_find_by_addr()
 *	Return the net device on which the given IP address resides.  Returns NULL on faiure.
 *
 * NOTE: The device may be the device upon which has a default gateway to reach the address.
 * from_local_addr is true when the device was found by a local address search.
 */
struct net_device *ecm_interface_dev_find_by_addr(ip_addr_t addr, bool *from_local_addr)
{
	char __attribute__((unused)) addr_str[40];
	struct ecm_interface_route ecm_rt;
	struct net_device *dev;
	struct dst_entry *dst;

	DEBUG_ECM_IP_ADDR_TO_STRING(addr_str, addr);

	/*
	 * Is the address a local IP?
	 */
	DEBUG_TRACE("find net device for address: %s\n", addr_str);
	dev = ecm_interface_dev_find_by_local_addr(addr);
	if (dev) {
		 *from_local_addr = true;
		DEBUG_TRACE("addr: %s is local: %p (%s)\n", addr_str, dev, dev->name);
		return dev;
	}

	DEBUG_TRACE("addr: %s is not local\n", addr_str);

	/*
	 * Try a route to the address instead
	 * NOTE: This will locate a route entry in the route destination *cache*.
	 */
	if (!ecm_interface_find_route_by_addr(addr, &ecm_rt)) {
		DEBUG_WARN("addr: %s - no dev locatable\n", addr_str);
		return NULL;
	}

	*from_local_addr = false;
	dst = ecm_rt.dst;
	dev = dst->dev;
	dev_hold(dev);
	ecm_interface_route_release(&ecm_rt);
	DEBUG_TRACE("dest_addr: %s uses dev: %p(%s)\n", addr_str, dev, dev->name);
	return dev;
}
EXPORT_SYMBOL(ecm_interface_dev_find_by_addr);


/*
 * ecm_interface_mac_addr_get_ipv4()
 *	Return mac for an IPv4 address
 */
static bool ecm_interface_mac_addr_get_ipv4(ip_addr_t addr, uint8_t *mac_addr, bool *on_link, ip_addr_t gw_addr)
{
	struct neighbour *neigh;
	struct ecm_interface_route ecm_rt;
	struct rtable *rt;
	struct dst_entry *dst;
	__be32 ipv4_addr;

	/*
	 * Get the MAC address that corresponds to IP address given.
	 * We look up the rtable entries and, from its neighbour structure, obtain the hardware address.
	 * This means we will also work if the neighbours are routers too.
	 * We also locate the MAC if the address is a local host address.
	 */
	ECM_IP_ADDR_TO_NIN4_ADDR(ipv4_addr, addr);
	if (!ecm_interface_find_route_by_addr(addr, &ecm_rt)) {
		*on_link = false;
		return false;
	}
	DEBUG_ASSERT(ecm_rt.v4_route, "Did not locate a v4 route!\n");
	DEBUG_TRACE("Found route\n");

	/*
	 * Is this destination on link or off-link via a gateway?
	 */
	rt = ecm_rt.rt.rtv4;
	if (rt->rt_uses_gateway || (rt->rt_flags & RTF_GATEWAY)) {
		*on_link = false;
		ECM_NIN4_ADDR_TO_IP_ADDR(gw_addr, rt->rt_gateway)
	} else {
		*on_link = true;
	}

	/*
	 * Get the neighbour entry for the address
	 */
	rcu_read_lock();
	dst = ecm_rt.dst;
	neigh = dst_neigh_lookup(dst, &ipv4_addr);
	if (!neigh) {
		neigh = neigh_lookup(&arp_tbl, &ipv4_addr, dst->dev);
	}
	if (!neigh) {
		rcu_read_unlock();
		ecm_interface_route_release(&ecm_rt);
		DEBUG_WARN("no neigh\n");
		return false;
	}
	if (!(neigh->nud_state & NUD_VALID)) {
		rcu_read_unlock();
		neigh_release(neigh);
		ecm_interface_route_release(&ecm_rt);
		DEBUG_WARN("neigh nud state is not valid\n");
		return false;
	}
	if (!neigh->dev) {
		rcu_read_unlock();
		neigh_release(neigh);
		ecm_interface_route_release(&ecm_rt);
		DEBUG_WARN("neigh has no device\n");
		return false;
	}

	/*
	 * If the device is loopback this will be because the address is a local address
	 * In this case locate the device that has this local address and get its mac.
	 */
	if (neigh->dev->type == ARPHRD_LOOPBACK) {
		struct net_device *dev;

		DEBUG_TRACE("%pI4 finds loopback device, dev: %p (%s)\n", &ipv4_addr, neigh->dev, neigh->dev->name);
		rcu_read_unlock();
		neigh_release(neigh);
		ecm_interface_route_release(&ecm_rt);

		/*
		 * Lookup the device that has this IP address assigned
		 */
		dev = ip_dev_find(&init_net, ipv4_addr);
		if (!dev) {
			DEBUG_WARN("Unable to locate dev for: %pI4\n", &ipv4_addr);
			return false;
		}
		memcpy(mac_addr, dev->dev_addr, (size_t)dev->addr_len);
		DEBUG_TRACE("is local addr: %pI4, mac: %pM, dev ifindex: %d, dev: %p (%s), dev_type: %d\n",
				&ipv4_addr, mac_addr, dev->ifindex, dev, dev->name, dev->type);
		dev_put(dev);
		return true;
	}

	if (!(neigh->dev->flags & IFF_NOARP)) {
		memcpy(mac_addr, neigh->ha, (size_t)neigh->dev->addr_len);
	} else {
		DEBUG_TRACE("non-arp device: %p (%s, type: %d) to reach %pI4\n", neigh->dev, neigh->dev->name, neigh->dev->type, &ipv4_addr);
		memset(mac_addr, 0, 6);
	}
	DEBUG_TRACE("addr: %pI4, mac: %pM, iif: %d, neigh dev ifindex: %d, dev: %p (%s), dev_type: %d\n",
			&ipv4_addr, mac_addr, rt->rt_iif, neigh->dev->ifindex, neigh->dev, neigh->dev->name, neigh->dev->type);

	rcu_read_unlock();
	neigh_release(neigh);
	ecm_interface_route_release(&ecm_rt);
	return true;
}

/*
 * ecm_interface_mac_addr_get()
 *	Return the mac address for the given IP address.  Returns false on failure.
 *
 * dev is the device on which the addr was sent/received.  If addr is a local address then mac shall be the given dev mac.
 *
 * GGG TODO Make this function work for IPv6!!!!!!!!!!!!!!
 */
bool ecm_interface_mac_addr_get(ip_addr_t addr, uint8_t *mac_addr, bool *on_link, ip_addr_t gw_addr)
{
	if (ECM_IP_ADDR_IS_V4(addr)) {
		return ecm_interface_mac_addr_get_ipv4(addr, mac_addr, on_link, gw_addr);
	}

	return false;
}
EXPORT_SYMBOL(ecm_interface_mac_addr_get);

/*
 * ecm_interface_addr_find_route_by_addr_ipv4()
 *	Return the route for the given IP address.  Returns NULL on failure.
 */
static bool ecm_interface_find_route_by_addr_ipv4(ip_addr_t addr, struct ecm_interface_route *ecm_rt)
{
	__be32 be_addr;

	/*
	 * Get a route to the given IP address, this will allow us to also find the interface
	 * it is using to communicate with that IP address.
	 */
	ECM_IP_ADDR_TO_NIN4_ADDR(be_addr, addr);
	ecm_rt->rt.rtv4 = ip_route_output(&init_net, be_addr, 0, 0, 0);
	if (IS_ERR(ecm_rt->rt.rtv4)) {
		DEBUG_TRACE("No output route to: %pI4n\n", &be_addr);
		return false;
	}
	DEBUG_TRACE("Output route to: %pI4n is: %p\n", &be_addr, ecm_rt->rt.rtv4);
	ecm_rt->dst = (struct dst_entry *)ecm_rt->rt.rtv4;
	ecm_rt->v4_route = true;
	return true;
}


/*
 * ecm_interface_addr_find_route_by_addr()
 *	Return the route (in the given parameter) for the given IP address.  Returns false on failure.
 *
 * Route is the device on which the addr is reachable, which may be loopback for local addresses.
 *
 * Returns true if the route was able to be located.  The route must be released using ecm_interface_route_release().
 */
bool ecm_interface_find_route_by_addr(ip_addr_t addr, struct ecm_interface_route *ecm_rt)
{
	char __attribute__((unused)) addr_str[40];

	DEBUG_ECM_IP_ADDR_TO_STRING(addr_str, addr);
	DEBUG_TRACE("Locate route to: %s\n", addr_str);

	if (ECM_IP_ADDR_IS_V4(addr)) {
		return ecm_interface_find_route_by_addr_ipv4(addr, ecm_rt);
	}

	return false;
}
EXPORT_SYMBOL(ecm_interface_find_route_by_addr);

/*
 * ecm_interface_route_release()
 *	Release an ecm route
 */
void ecm_interface_route_release(struct ecm_interface_route *rt)
{
	dst_release(rt->dst);
}
EXPORT_SYMBOL(ecm_interface_route_release);



/*
 * ecm_interface_bridge_interface_establish()
 *	Returns a reference to a iface of the BRIDGE type, possibly creating one if necessary.
 * Returns NULL on failure or a reference to interface.
 */
static struct ecm_db_iface_instance *ecm_interface_bridge_interface_establish(struct ecm_db_interface_info_bridge *type_info,
							char *dev_name, int32_t dev_interface_num, int32_t nss_interface_num, int32_t mtu)
{
	struct ecm_db_iface_instance *nii;
	struct ecm_db_iface_instance *ii;

	DEBUG_INFO("Establish BRIDGE iface: %s with address: %pM, MTU: %d, if num: %d, nss if id: %d\n",
			dev_name, type_info->address, mtu, dev_interface_num, nss_interface_num);

	/*
	 * Locate the iface
	 */
	ii = ecm_db_iface_find_and_ref_bridge(type_info->address);
	if (ii) {
		DEBUG_TRACE("%p: iface established\n", ii);
		return ii;
	}

	/*
	 * No iface - create one
	 */
	nii = ecm_db_iface_alloc();
	if (!nii) {
		DEBUG_WARN("Failed to establish iface\n");
		return NULL;
	}

	/*
	 * Add iface into the database, atomically to avoid races creating the same thing
	 */
	spin_lock_bh(&ecm_interface_lock);
	ii = ecm_db_iface_find_and_ref_bridge(type_info->address);
	if (ii) {
		spin_unlock_bh(&ecm_interface_lock);
		ecm_db_iface_deref(nii);
		return ii;
	}
	ecm_db_iface_add_bridge(nii, type_info->address, dev_name,
			mtu, dev_interface_num, nss_interface_num, NULL, nii);
	spin_unlock_bh(&ecm_interface_lock);

	DEBUG_TRACE("%p: bridge iface established\n", nii);
	return nii;
}


/*
 * ecm_interface_ethernet_interface_establish()
 *	Returns a reference to a iface of the ETHERNET type, possibly creating one if necessary.
 * Returns NULL on failure or a reference to interface.
 */
static struct ecm_db_iface_instance *ecm_interface_ethernet_interface_establish(struct ecm_db_interface_info_ethernet *type_info,
							char *dev_name, int32_t dev_interface_num, int32_t nss_interface_num, int32_t mtu)
{
	struct ecm_db_iface_instance *nii;
	struct ecm_db_iface_instance *ii;

	DEBUG_INFO("Establish ETHERNET iface: %s with address: %pM, MTU: %d, if num: %d, nss if id: %d\n",
			dev_name, type_info->address, mtu, dev_interface_num, nss_interface_num);

	/*
	 * Locate the iface
	 */
	ii = ecm_db_iface_ifidx_find_and_ref_ethernet(type_info->address, dev_interface_num);

	if (ii) {
		DEBUG_TRACE("%p: iface established\n", ii);
		return ii;
	}

	/*
	 * No iface - create one
	 */
	nii = ecm_db_iface_alloc();
	if (!nii) {
		DEBUG_WARN("Failed to establish iface\n");
		return NULL;
	}

	/*
	 * Add iface into the database, atomically to avoid races creating the same thing
	 */
	spin_lock_bh(&ecm_interface_lock);
	ii = ecm_db_iface_ifidx_find_and_ref_ethernet(type_info->address, dev_interface_num);
	if (ii) {
		spin_unlock_bh(&ecm_interface_lock);
		ecm_db_iface_deref(nii);
		return ii;
	}
	ecm_db_iface_add_ethernet(nii, type_info->address, dev_name,
			mtu, dev_interface_num, nss_interface_num, NULL, nii);
	spin_unlock_bh(&ecm_interface_lock);

	DEBUG_TRACE("%p: ethernet iface established\n", nii);
	return nii;
}


/*
 * ecm_interface_unknown_interface_establish()
 *	Returns a reference to a iface of the UNKNOWN type, possibly creating one if necessary.
 * Returns NULL on failure or a reference to interface.
 */
static struct ecm_db_iface_instance *ecm_interface_unknown_interface_establish(struct ecm_db_interface_info_unknown *type_info,
							char *dev_name, int32_t dev_interface_num, int32_t nss_interface_num, int32_t mtu)
{
	struct ecm_db_iface_instance *nii;
	struct ecm_db_iface_instance *ii;

	DEBUG_INFO("Establish UNKNOWN iface: %s with os_specific_ident: %u, MTU: %d, if num: %d, nss if id: %d\n",
			dev_name, type_info->os_specific_ident, mtu, dev_interface_num, nss_interface_num);

	/*
	 * Locate the iface
	 */
	ii = ecm_db_iface_find_and_ref_unknown(type_info->os_specific_ident);
	if (ii) {
		DEBUG_TRACE("%p: iface established\n", ii);
		return ii;
	}

	/*
	 * No iface - create one
	 */
	nii = ecm_db_iface_alloc();
	if (!nii) {
		DEBUG_WARN("Failed to establish iface\n");
		return NULL;
	}

	/*
	 * Add iface into the database, atomically to avoid races creating the same thing
	 */
	spin_lock_bh(&ecm_interface_lock);
	ii = ecm_db_iface_find_and_ref_unknown(type_info->os_specific_ident);
	if (ii) {
		spin_unlock_bh(&ecm_interface_lock);
		ecm_db_iface_deref(nii);
		return ii;
	}
	ecm_db_iface_add_unknown(nii, type_info->os_specific_ident, dev_name,
			mtu, dev_interface_num, nss_interface_num, NULL, nii);
	spin_unlock_bh(&ecm_interface_lock);

	DEBUG_TRACE("%p: unknown iface established\n", nii);
	return nii;
}

/*
 * ecm_interface_loopback_interface_establish()
 *	Returns a reference to a iface of the LOOPBACK type, possibly creating one if necessary.
 * Returns NULL on failure or a reference to interface.
 */
static struct ecm_db_iface_instance *ecm_interface_loopback_interface_establish(struct ecm_db_interface_info_loopback *type_info,
							char *dev_name, int32_t dev_interface_num, int32_t nss_interface_num, int32_t mtu)
{
	struct ecm_db_iface_instance *nii;
	struct ecm_db_iface_instance *ii;

	DEBUG_INFO("Establish LOOPBACK iface: %s with os_specific_ident: %u, MTU: %d, if num: %d, nss if id: %d\n",
			dev_name, type_info->os_specific_ident, mtu, dev_interface_num, nss_interface_num);

	/*
	 * Locate the iface
	 */
	ii = ecm_db_iface_find_and_ref_loopback(type_info->os_specific_ident);
	if (ii) {
		DEBUG_TRACE("%p: iface established\n", ii);
		return ii;
	}

	/*
	 * No iface - create one
	 */
	nii = ecm_db_iface_alloc();
	if (!nii) {
		DEBUG_WARN("Failed to establish iface\n");
		return NULL;
	}

	/*
	 * Add iface into the database, atomically to avoid races creating the same thing
	 */
	spin_lock_bh(&ecm_interface_lock);
	ii = ecm_db_iface_find_and_ref_loopback(type_info->os_specific_ident);
	if (ii) {
		spin_unlock_bh(&ecm_interface_lock);
		ecm_db_iface_deref(nii);
		return ii;
	}
	ecm_db_iface_add_loopback(nii, type_info->os_specific_ident, dev_name,
			mtu, dev_interface_num, nss_interface_num, NULL, nii);
	spin_unlock_bh(&ecm_interface_lock);

	DEBUG_TRACE("%p: loopback iface established\n", nii);
	return nii;
}




/*
 * ecm_interface_establish_and_ref()
 *	Establish an interface instance for the given interface detail.
 */
struct ecm_db_iface_instance *ecm_interface_establish_and_ref(struct net_device *dev)
{
	int32_t dev_interface_num;
	char *dev_name;
	int32_t dev_type;
	int32_t dev_mtu;
	int32_t nss_interface_num;
	struct ecm_db_iface_instance *ii;
	union {
		struct ecm_db_interface_info_ethernet ethernet;		/* type == ECM_DB_IFACE_TYPE_ETHERNET */
		struct ecm_db_interface_info_bridge bridge;		/* type == ECM_DB_IFACE_TYPE_BRIDGE */
		struct ecm_db_interface_info_unknown unknown;		/* type == ECM_DB_IFACE_TYPE_UNKNOWN */
		struct ecm_db_interface_info_loopback loopback;		/* type == ECM_DB_IFACE_TYPE_LOOPBACK */
	} type_info;


	/*
	 * Get basic information about the given device
	 */
	dev_interface_num = dev->ifindex;
	dev_name = dev->name;
	dev_type = dev->type;
	dev_mtu = dev->mtu;

	/*
	 * Does the NSS recognise this interface?
	 */
	nss_interface_num = nss_cmn_get_interface_number_by_dev(dev);

	DEBUG_TRACE("Establish interface instance for device: %p is type: %d, name: %s, ifindex: %d, nss_if: %d, mtu: %d\n",
			dev, dev_type, dev_name, dev_interface_num, nss_interface_num, dev_mtu);

	/*
	 * Extract from the device more type-specific information
	 */
	if (dev_type == ARPHRD_ETHER) {
		/*
		 * Ethernet - but what sub type?
		 */


		/*
		 * BRIDGE?
		 */
		if (ecm_front_end_is_bridge_device(dev)) {
			/*
			 * Bridge
			 */
			memcpy(type_info.bridge.address, dev->dev_addr, 6);

			DEBUG_TRACE("Net device: %p is BRIDGE, mac: %pM\n",
					dev, type_info.bridge.address);

			/*
			 * Establish this type of interface
			 */
			ii = ecm_interface_bridge_interface_establish(&type_info.bridge, dev_name, dev_interface_num, nss_interface_num, dev_mtu);
			return ii;
		}


		/*
		 * ETHERNET!
		 * Just plain ethernet it seems
		 */
		memcpy(type_info.ethernet.address, dev->dev_addr, 6);
		DEBUG_TRACE("Net device: %p is ETHERNET, mac: %pM\n",
				dev, type_info.ethernet.address);

		/*
		 * Establish this type of interface
		 */
		ii = ecm_interface_ethernet_interface_establish(&type_info.ethernet, dev_name, dev_interface_num, nss_interface_num, dev_mtu);
		return ii;
	}

	/*
	 * LOOPBACK?
	 */
	if (dev_type == ARPHRD_LOOPBACK) {
		DEBUG_TRACE("Net device: %p is LOOPBACK type: %d\n", dev, dev_type);
		type_info.loopback.os_specific_ident = dev_interface_num;
		ii = ecm_interface_loopback_interface_establish(&type_info.loopback, dev_name, dev_interface_num, nss_interface_num, dev_mtu);
		return ii;
	}




	/*
	 * If this is NOT PPP then it is unknown to the ecm
	 */
	if (dev_type != ARPHRD_PPP) {
		DEBUG_TRACE("Net device: %p is UNKNOWN type: %d\n", dev, dev_type);
		type_info.unknown.os_specific_ident = dev_interface_num;

		/*
		 * Establish this type of interface
		 */
		ii = ecm_interface_unknown_interface_establish(&type_info.unknown, dev_name, dev_interface_num, nss_interface_num, dev_mtu);
		return ii;
	}

	/*
	 * PPP support is NOT provided for.
	 * Interface is therefore unknown
	 */
	DEBUG_TRACE("Net device: %p is UNKNOWN (PPP Unsupported) type: %d\n", dev, dev_type);
	type_info.unknown.os_specific_ident = dev_interface_num;

	/*
	 * Establish this type of interface
	 */
	ii = ecm_interface_unknown_interface_establish(&type_info.unknown, dev_name, dev_interface_num, nss_interface_num, dev_mtu);
	return ii;
}
EXPORT_SYMBOL(ecm_interface_establish_and_ref);

/*
 * ecm_interface_heirarchy_construct()
 *	Construct an interface heirarchy.
 *
 * Using the given addressing, locate the interface heirarchy used to emit packets to that destination.
 * This is the heirarchy of interfaces a packet would transit to emit from the device.
 *
 * We will use the given src/dest devices when is_routed is false.
 * When is_routed is true we will try routing tables first, failing back to any given.
 *
 * For example, with this network arrangement:
 *
 * PPPoE--VLAN--BRIDGE--BRIDGE_PORT(LAG_MASTER)--LAG_SLAVE_0--10.22.33.11
 *
 * Given the packet_dest_addr IP address 10.22.33.11 this will create an interface heirarchy (in interracfes[]) of:
 * LAG_SLAVE_0 @ [ECM_DB_IFACE_HEIRARCHY_MAX - 5]
 * LAG_MASTER @ [ECM_DB_IFACE_HEIRARCHY_MAX - 4]
 * BRIDGE @ [ECM_DB_IFACE_HEIRARCHY_MAX - 3]
 * VLAN @ [ECM_DB_IFACE_HEIRARCHY_MAX - 2]
 * PPPOE @ [ECM_DB_IFACE_HEIRARCHY_MAX - 1]
 * The value returned is (ECM_DB_IFACE_HEIRARCHY_MAX - 5)
 *
 * IMPORTANT: This function will return any known interfaces in the database, when interfaces do not exist in the database
 * they will be created and added automatically to the database.
 *
 * GGG TODO Make this function work for IPv6!!!!!!!!!!!!!!
 */
int32_t ecm_interface_heirarchy_construct(struct ecm_db_iface_instance *interfaces[], ip_addr_t packet_src_addr, ip_addr_t packet_dest_addr, int packet_protocol,
						struct net_device *given_dest_dev, bool is_routed, struct net_device *given_src_dev, uint8_t *dest_node_addr, uint8_t *src_node_addr)
{
	int protocol;
	ip_addr_t src_addr;
	ip_addr_t dest_addr;
	struct net_device *dest_dev;
	char *dest_dev_name;
	int32_t dest_dev_type;
	struct net_device *src_dev;
	char *src_dev_name;
	int32_t src_dev_type;
	int32_t current_interface_index;
	bool from_local_addr;

	/*
	 * Get a big endian of the IPv4 address we have been given as our starting point.
	 */
	protocol = packet_protocol;
	ECM_IP_ADDR_COPY(src_addr, packet_src_addr);
	ECM_IP_ADDR_COPY(dest_addr, packet_dest_addr);
	DEBUG_TRACE("Construct interface heirarchy for from src_addr: " ECM_IP_ADDR_DOT_FMT " to dest_addr: " ECM_IP_ADDR_DOT_FMT ", protocol: %d\n",
			ECM_IP_ADDR_TO_DOT(src_addr), ECM_IP_ADDR_TO_DOT(dest_addr), protocol);

	/*
	 * Get device to reach the given destination address.
	 * If the heirarchy is for a routed connection we must try route lookup first, falling back to any given_dest_dev.
	 * If the heirarchy is NOT for a routed connection we try the given_dest_dev first, followed by routed lookup.
	 */
	from_local_addr = false;
	if (is_routed) {
		dest_dev = ecm_interface_dev_find_by_addr(dest_addr, &from_local_addr);
		if (!dest_dev && given_dest_dev) {
			/*
			 * Fall back to any given
			 */
			dest_dev = given_dest_dev;
			dev_hold(dest_dev);
		}
	} else if (given_dest_dev) {
		dest_dev = given_dest_dev;
		dev_hold(dest_dev);
	} else {
		/*
		 * Fall back to routed look up
		 */
		dest_dev = ecm_interface_dev_find_by_addr(dest_addr, &from_local_addr);
	}

	/*
	 * GGG ALERT: If the address is a local address and protocol is an IP tunnel
	 * then this connection is a tunnel endpoint made to this device.
	 * In which case we circumvent all proper procedure and just hack the devices to make stuff work.
	 * GGG TODO THIS MUST BE FIXED - WE MUST USE THE INTERFACE HIERARCHY FOR ITS INTENDED PURPOSE TO
	 * PARSE THE DEVICES AND WORK OUT THE PROPER INTERFACES INVOLVED.
	 * E.G. IF WE TRIED TO RUN A TUNNEL OVER A VLAN OR QINQ THIS WILL BREAK AS WE DON'T DISCOVER THAT HIERARCHY
	 */
	if (dest_dev && from_local_addr && (protocol == IPPROTO_IPV6)) {
		dev_put(dest_dev);
		dest_dev = given_dest_dev;
		if (dest_dev) {
			dev_hold(dest_dev);
			DEBUG_TRACE("HACK: IPV6 tunnel packet with dest_addr: " ECM_IP_ADDR_OCTAL_FMT " uses dev: %p(%s)\n", ECM_IP_ADDR_TO_OCTAL(dest_addr), dest_dev, dest_dev->name);
		}
	}
	if (!dest_dev) {
		DEBUG_WARN("dest_addr: " ECM_IP_ADDR_OCTAL_FMT " - cannot locate device\n", ECM_IP_ADDR_TO_OCTAL(dest_addr));
		return ECM_DB_IFACE_HEIRARCHY_MAX;
	}
	dest_dev_name = dest_dev->name;
	dest_dev_type = dest_dev->type;

	/*
	 * Get device to reach the given source address.
	 * If the heirarchy is for a routed connection we must try route lookup first, falling back to any given_src_dev.
	 * If the heirarchy is NOT for a routed connection we try the given_src_dev first, followed by routed lookup.
	 */
	from_local_addr = false;
	if (is_routed) {
		src_dev = ecm_interface_dev_find_by_addr(src_addr, &from_local_addr);
		if (!src_dev && given_src_dev) {
			/*
			 * Fall back to any given
			 */
			src_dev = given_src_dev;
			dev_hold(src_dev);
		}
	} else if (given_src_dev) {
		src_dev = given_src_dev;
		dev_hold(src_dev);
	} else {
		/*
		 * Fall back to routed look up
		 */
		src_dev = ecm_interface_dev_find_by_addr(src_addr, &from_local_addr);
	}

	/*
	 * GGG ALERT: If the address is a local address and protocol is an IP tunnel
	 * then this connection is a tunnel endpoint made to this device.
	 * In which case we circumvent all proper procedure and just hack the devices to make stuff work.
	 * GGG TODO THIS MUST BE FIXED - WE MUST USE THE INTERFACE HIERARCHY FOR ITS INTENDED PURPOSE TO
	 * PARSE THE DEVICES AND WORK OUT THE PROPER INTERFACES INVOLVED.
	 * E.G. IF WE TRIED TO RUN A TUNNEL OVER A VLAN OR QINQ THIS WILL BREAK AS WE DON'T DISCOVER THAT HIERARCHY
	 */
	if (src_dev && from_local_addr && (protocol == IPPROTO_IPV6)) {
		dev_put(src_dev);
		src_dev = given_src_dev;
		if (src_dev) {
			dev_hold(src_dev);
			DEBUG_TRACE("HACK: IPV6 tunnel packet with src_addr: " ECM_IP_ADDR_OCTAL_FMT " uses dev: %p(%s)\n", ECM_IP_ADDR_TO_OCTAL(src_addr), src_dev, src_dev->name);
		}
	}
	if (!src_dev) {
		DEBUG_WARN("src_addr: " ECM_IP_ADDR_OCTAL_FMT " - cannot locate device\n", ECM_IP_ADDR_TO_OCTAL(src_addr));
		dev_put(dest_dev);
		return ECM_DB_IFACE_HEIRARCHY_MAX;
	}
	src_dev_name = src_dev->name;
	src_dev_type = src_dev->type;

	/*
	 * Check if source and dest dev are same.
	 * For the forwarded flows which involve tunnels this will happen when called from input hook.
	 */
	if (src_dev == dest_dev) {
		DEBUG_TRACE("Protocol is :%d source dev and dest dev are same\n", protocol);
		if ((protocol == IPPROTO_IPV6) || (protocol == IPPROTO_ESP)) {
			/*
			 * This happens from the input hook
			 * We do not want to create a connection entry for this
			 * GGG TODO YES WE DO.
			 * GGG TODO THIS CONCERNS ME AS THIS SHOULD BE CAUGHT MUCH
			 * EARLIER IN THE FRONT END IF POSSIBLE TO AVOID PERFORMANCE PENALTIES.
			 * WE HAVE DONE A TREMENDOUS AMOUT OF WORK TO GET TO THIS POINT.
			 * WE WILL ABORT HERE AND THIS WILL BE REPEATED FOR EVERY PACKET.
			 * IN KEEPING WITH THE ECM DESIGN IT IS BETTER TO CREATE A CONNECTION AND RECORD IN THE HIERARCHY
			 * ENOUGH INFORMATION TO ENSURE THAT ACCELERATION IS NOT BROKEN / DOES NOT OCCUR AT ALL.
			 * THAT WAY WE DO A HEAVYWEIGHT ESTABLISHING OF A CONNECTION ONCE AND NEVER AGAIN...
			 */
			dev_put(src_dev);
			dev_put(dest_dev);
			return ECM_DB_IFACE_HEIRARCHY_MAX;
		}
	}

	/*
	 * Iterate until we are done or get to the max number of interfaces we can record.
	 * NOTE: current_interface_index tracks the position of the first interface position in interfaces[]
	 * because we add from the end first_interface grows downwards.
	 */
	current_interface_index = ECM_DB_IFACE_HEIRARCHY_MAX;
	while (current_interface_index > 0) {
		struct ecm_db_iface_instance *ii;
		struct net_device *next_dev;

		/*
		 * Get the ecm db interface instance for the device at hand
		 */
		ii = ecm_interface_establish_and_ref(dest_dev);

		/*
		 * If the interface could not be established then we abort
		 */
		if (!ii) {
			DEBUG_WARN("Failed to establish interface: %p, name: %s\n", dest_dev, dest_dev_name);
			dev_put(src_dev);
			dev_put(dest_dev);

			/*
			 * Release the interfaces heirarchy we constructed to this point.
			 */
			ecm_db_connection_interfaces_deref(interfaces, current_interface_index);
			return ECM_DB_IFACE_HEIRARCHY_MAX;
		}

		/*
		 * Record the interface instance into the interfaces[]
		 */
		current_interface_index--;
		interfaces[current_interface_index] = ii;

		/*
		 * Now we have to figure out what the next device will be (in the transmission path) the skb
		 * will use to emit to the destination address.
		 */
		do {

			DEBUG_TRACE("Net device: %p is type: %d, name: %s\n", dest_dev, dest_dev_type, dest_dev_name);
			next_dev = NULL;

			if (dest_dev_type == ARPHRD_ETHER) {
				/*
				 * Ethernet - but what sub type?
				 */


				/*
				 * BRIDGE?
				 */
				if (ecm_front_end_is_bridge_device(dest_dev)) {
					/*
					 * Bridge
					 * Figure out which port device the skb will go to using the dest_addr.
					 */
					bool on_link;
					ip_addr_t gw_addr = ECM_IP_ADDR_NULL;
					uint8_t mac_addr[ETH_ALEN];
					if (!ecm_interface_mac_addr_get(dest_addr, mac_addr, &on_link, gw_addr)) {
						/*
						 * Possible ARP does not know the address yet
						 */
						DEBUG_INFO("Unable to obtain MAC address for " ECM_IP_ADDR_DOT_FMT "\n", ECM_IP_ADDR_TO_DOT(dest_addr));
						if (ECM_IP_ADDR_IS_V4(dest_addr)) {
							__be32 ipv4_addr;
							__be32 src_ip;

							/*
							 * Issue an ARP request for it, select the src_ip from which to issue the request.
							 */
							ECM_IP_ADDR_TO_NIN4_ADDR(ipv4_addr, dest_addr);
							src_ip = inet_select_addr(dest_dev, ipv4_addr, RT_SCOPE_LINK);
							if (!src_ip) {
								DEBUG_TRACE("failed to lookup IP for %pI4\n", &ipv4_addr);

								dev_put(src_dev);
								dev_put(dest_dev);

								/*
								* Release the interfaces heirarchy we constructed to this point.
								*/
								ecm_db_connection_interfaces_deref(interfaces, current_interface_index);
								return ECM_DB_IFACE_HEIRARCHY_MAX;
							}

							/*
							 * If we have a GW for this address, then we have to send ARP request to the GW
							 */
							if (!on_link && !ECM_IP_ADDR_IS_NULL(gw_addr)) {
								ECM_IP_ADDR_TO_NIN4_ADDR(ipv4_addr, gw_addr);
							}

							DEBUG_TRACE("Send ARP for %pI4 using src_ip as %pI4\n", &ipv4_addr, &src_ip);
							arp_send(ARPOP_REQUEST, ETH_P_ARP, ipv4_addr, dest_dev, src_ip, NULL, NULL, NULL);
						}

						DEBUG_WARN("Unable to obtain MAC address for " ECM_IP_ADDR_DOT_FMT "\n", ECM_IP_ADDR_TO_DOT(dest_addr));
						dev_put(src_dev);
						dev_put(dest_dev);

						/*
						 * Release the interfaces heirarchy we constructed to this point.
						 */
						ecm_db_connection_interfaces_deref(interfaces, current_interface_index);
						return ECM_DB_IFACE_HEIRARCHY_MAX;
					}
					next_dev = br_port_dev_get(dest_dev, mac_addr);
					if (!next_dev) {
						DEBUG_WARN("Unable to obtain output port for: %pM\n", mac_addr);
						dev_put(src_dev);
						dev_put(dest_dev);

						/*
						 * Release the interfaces heirarchy we constructed to this point.
						 */
						ecm_db_connection_interfaces_deref(interfaces, current_interface_index);
						return ECM_DB_IFACE_HEIRARCHY_MAX;
					}
					DEBUG_TRACE("Net device: %p is BRIDGE, next_dev: %p (%s)\n", dest_dev, next_dev, next_dev->name);
					break;
				}


				/*
				 * ETHERNET!
				 * Just plain ethernet it seems.
				 */
				DEBUG_TRACE("Net device: %p is ETHERNET\n", dest_dev);
				break;
			}

			/*
			 * LOOPBACK?
			 */
			if (dest_dev_type == ARPHRD_LOOPBACK) {
				DEBUG_TRACE("Net device: %p is LOOPBACK type: %d\n", dest_dev, dest_dev_type);
				break;
			}

			/*
			 * IPSEC?
			 */
			if (dest_dev_type == ECM_ARPHRD_IPSEC_TUNNEL_TYPE) {
				DEBUG_TRACE("Net device: %p is IPSec tunnel type: %d\n", dest_dev, dest_dev_type);
				// GGG TODO Figure out the next device the tunnel is using...
				break;
			}

			/*
			 * SIT (6-in-4)?
			 */
			if (dest_dev_type == ARPHRD_SIT) {
				DEBUG_TRACE("Net device: %p is SIT (6-in-4) type: %d\n", dest_dev, dest_dev_type);
				// GGG TODO Figure out the next device the tunnel is using...
				break;
			}

			/*
			 * IPIP6 Tunnel?
			 */
			if (dest_dev_type == ARPHRD_TUNNEL6) {
				DEBUG_TRACE("Net device: %p is TUNIPIP6 type: %d\n", dest_dev, dest_dev_type);
				// GGG TODO Figure out the next device the tunnel is using...
				break;
			}

			/*
			 * If this is NOT PPP then it is unknown to the ecm and we cannot figure out it's next device.
			 */
			if (dest_dev_type != ARPHRD_PPP) {
				DEBUG_TRACE("Net device: %p is UNKNOWN type: %d\n", dest_dev, dest_dev_type);
				break;
			}

			DEBUG_TRACE("Net device: %p is UNKNOWN (PPP Unsupported) type: %d\n", dest_dev, dest_dev_type);
		} while (false);

		/*
		 * No longer need dest_dev as it may become next_dev
		 */
		dev_put(dest_dev);

		/*
		 * Check out the next_dev, if any
		 */
		if (!next_dev) {
			int32_t i __attribute__((unused));
			DEBUG_INFO("Completed interface heirarchy construct with first interface @: %d\n", current_interface_index);
#if DEBUG_LEVEL > 1
			for (i = current_interface_index; i < ECM_DB_IFACE_HEIRARCHY_MAX; ++i) {
				DEBUG_TRACE("\tInterface @ %d: %p, type: %d, name: %s\n",
						i, interfaces[i], ecm_db_connection_iface_type_get(interfaces[i]), ecm_db_interface_type_to_string(ecm_db_connection_iface_type_get(interfaces[i])));

			}
#endif

			/*
			 * Release src_dev now
			 */
			dev_put(src_dev);
			return current_interface_index;
		}

		/*
		 * dest_dev becomes next_dev
		 */
		dest_dev = next_dev;
		dest_dev_name = dest_dev->name;
		dest_dev_type = dest_dev->type;
	}

	DEBUG_WARN("Too many interfaces: %d\n", current_interface_index);
	DEBUG_ASSERT(current_interface_index == 0, "Bad logic handling current_interface_index: %d\n", current_interface_index);
	dev_put(src_dev);
	dev_put(dest_dev);

	/*
	 * Release the interfaces heirarchy we constructed to this point.
	 */
	ecm_db_connection_interfaces_deref(interfaces, current_interface_index);
	return ECM_DB_IFACE_HEIRARCHY_MAX;
}
EXPORT_SYMBOL(ecm_interface_heirarchy_construct);

/*
 * ecm_interface_list_stats_update()
 *	Given an interface list, walk the interfaces and update the stats for certain types.
 */
static void ecm_interface_list_stats_update(int iface_list_first, struct ecm_db_iface_instance *iface_list[], uint8_t *mac_addr,
						uint32_t tx_packets, uint32_t tx_bytes, uint32_t rx_packets, uint32_t rx_bytes)
{
	int list_index;

	for (list_index = iface_list_first; (list_index < ECM_DB_IFACE_HEIRARCHY_MAX); list_index++) {
		struct ecm_db_iface_instance *ii;
		ecm_db_iface_type_t ii_type;
		char *ii_name;
		struct net_device *dev;

		ii = iface_list[list_index];
		ii_type = ecm_db_connection_iface_type_get(ii);
		ii_name = ecm_db_interface_type_to_string(ii_type);
		DEBUG_TRACE("list_index: %d, ii: %p, type: %d (%s)\n", list_index, ii, ii_type, ii_name);

		/*
		 * Locate real device in system
		 */
		dev = dev_get_by_index(&init_net, ecm_db_iface_interface_identifier_get(ii));
		if (!dev) {
			DEBUG_WARN("Could not locate interface\n");
			continue;
		}
		DEBUG_TRACE("found dev: %p (%s)\n", dev, dev->name);

		/*
		 * Refresh the bridge forward table entry if the port is a bridge port
		 * Note: A bridge port can be of different interface type, e.g VLAN, ethernet.
		 * This check, therefore, should be performed for all interface types.
		 */
		if (is_valid_ether_addr(mac_addr) && ecm_front_end_is_bridge_port(dev)) {
			DEBUG_TRACE("Update bridge fdb entry for mac: %pM\n", mac_addr);
			br_refresh_fdb_entry(dev, mac_addr);
		}

		switch (ii_type) {
			struct rtnl_link_stats64 stats;

			case ECM_DB_IFACE_TYPE_BRIDGE:
				DEBUG_INFO("BRIDGE\n");
				stats.rx_packets = rx_packets;
				stats.rx_bytes = rx_bytes;
				stats.tx_packets = tx_packets;
				stats.tx_bytes = tx_bytes;
				br_dev_update_stats(dev, &stats);
				break;

			default:
				/*
				 * TODO: Extend it accordingly
				 */
				break;
		}

		dev_put(dev);
	}
}

/*
 * ecm_interface_stats_update()
 *	Using the interface lists for the given connection, update the interface statistics for each.
 *
 * 'from' here is wrt the connection 'from' side.  Likewise with 'to'.
 * TX is wrt what the interface has transmitted.  RX is what the interface has received.
 */
void ecm_interface_stats_update(struct ecm_db_connection_instance *ci,
						uint32_t from_tx_packets, uint32_t from_tx_bytes, uint32_t from_rx_packets, uint32_t from_rx_bytes,
						uint32_t to_tx_packets, uint32_t to_tx_bytes, uint32_t to_rx_packets, uint32_t to_rx_bytes)
{
	struct ecm_db_iface_instance *from_ifaces[ECM_DB_IFACE_HEIRARCHY_MAX];
	struct ecm_db_iface_instance *to_ifaces[ECM_DB_IFACE_HEIRARCHY_MAX];
	int from_ifaces_first;
	int to_ifaces_first;
	uint8_t mac_addr[ETH_ALEN];

	/*
	 * Iterate the 'from' side interfaces and update statistics and state for the real HLOS interfaces
	 * from_tx_packets / bytes: the amount transmitted by the 'from' interface
	 * from_rx_packets / bytes: the amount received by the 'from' interface
	 */
	DEBUG_INFO("%p: Update from interface stats\n", ci);
	from_ifaces_first = ecm_db_connection_from_interfaces_get_and_ref(ci, from_ifaces);
	ecm_db_connection_from_node_address_get(ci, mac_addr);
	ecm_interface_list_stats_update(from_ifaces_first, from_ifaces, mac_addr, from_tx_packets, from_tx_bytes, from_rx_packets, from_rx_bytes);
	ecm_db_connection_interfaces_deref(from_ifaces, from_ifaces_first);

	/*
	 * Iterate the 'to' side interfaces and update statistics and state for the real HLOS interfaces
	 * to_tx_packets / bytes: the amount transmitted by the 'to' interface
	 * to_rx_packets / bytes: the amount received by the 'to' interface
	 */
	DEBUG_INFO("%p: Update to interface stats\n", ci);
	to_ifaces_first = ecm_db_connection_to_interfaces_get_and_ref(ci, to_ifaces);
	ecm_db_connection_to_node_address_get(ci, mac_addr);
	ecm_interface_list_stats_update(to_ifaces_first, to_ifaces, mac_addr, to_tx_packets, to_tx_bytes, to_rx_packets, to_rx_bytes);
	ecm_db_connection_interfaces_deref(to_ifaces, to_ifaces_first);
}
EXPORT_SYMBOL(ecm_interface_stats_update);

/*
 * ecm_interface_regenerate_connection()
 *	Re-generate a specific connection
 */
void ecm_interface_regenerate_connection(struct ecm_db_connection_instance *ci)
{
	struct ecm_front_end_connection_instance *feci;

	DEBUG_TRACE("Regenerate connection: %p\n", ci);

	/*
	 * Flag the connection as needing re-generation.
	 * Re-generation occurs when we next see traffic OR an acceleration engine sync for this connection.
	 * Refer to front end protocol specific process() functions.
	 */
	ecm_db_connection_classifier_generation_change(ci);

	/*
	 * If the connection is accelerated then force deceleration.
	 * Under normal circumstances deceleration would occur on the next sync received,
	 * however, there is a situation where a sync may not occur if, say, a cable has been pulled.
	 * The acceleration engine would see no further traffic to trigger sending a sync and so
	 * re-generation would not occur.
	 * The connection would stall and no-regeneration would happen leaving the connection in bad state.
	 * NOTE: We can just call decelerate() upon the front end - if its not accelerated this will have no effect.
	 */
	feci = ecm_db_connection_front_end_get_and_ref(ci);
	feci->decelerate(feci);
	feci->deref(feci);
}

/*
 * ecm_interface_regenerate_connections()
 *	Cause regeneration of all connections that are using the specified interface.
 */
static void ecm_interface_regenerate_connections(struct ecm_db_iface_instance *ii)
{

	DEBUG_TRACE("Regenerate connections using interface: %p\n", ii);

	/*
	 * An interface has changed, re-generate the connections to ensure all state is updated.
	 */
	ecm_db_classifier_generation_change();

	DEBUG_TRACE("%p: Regenerate COMPLETE\n", ii);
}

/*
 * ecm_interface_dev_regenerate_connections()
 *	Cause regeneration of all connections that are using the specified interface.
 */
void ecm_interface_dev_regenerate_connections(struct net_device *dev)
{
	struct ecm_db_iface_instance *ii;

	DEBUG_INFO("Regenerate connections for: %p (%s)\n", dev, dev->name);

	/*
	 * Establish the interface for the given device.
	 * NOTE: The cute thing here is even if dev is previously unknown to us this will create an interface instance
	 * but it will have no connections to regen and will be destroyed at the end of the function when we deref - so no harm done.
	 * However if the interface is known to us then we will get it returned by this function and process it accordingly.
	 */
	ii = ecm_interface_establish_and_ref(dev);
	if (!ii) {
		DEBUG_WARN("%p: No interface instance could be established for this dev\n", dev);
		return;
	}
	ecm_interface_regenerate_connections(ii);
	DEBUG_TRACE("%p: Regenerate for %p: COMPLETE\n", dev, ii);
	ecm_db_iface_deref(ii);
}

/*
 * ecm_interface_mtu_change()
 *	MTU of interface has changed
 */
static void ecm_interface_mtu_change(struct net_device *dev)
{
	int mtu;
	struct ecm_db_iface_instance *ii;

	mtu = dev->mtu;
	DEBUG_INFO("%p (%s): MTU Change to: %d\n", dev, dev->name, mtu);

	/*
	 * Establish the interface for the given device.
	 */
	ii = ecm_interface_establish_and_ref(dev);
	if (!ii) {
		DEBUG_WARN("%p: No interface instance could be established for this dev\n", dev);
		return;
	}

	/*
	 * Change the mtu
	 */
	ecm_db_iface_mtu_reset(ii, mtu);
	DEBUG_TRACE("%p (%s): MTU Changed to: %d\n", dev, dev->name, mtu);
	ecm_interface_regenerate_connections(ii);
	DEBUG_TRACE("%p: Regenerate for %p: COMPLETE\n", dev, ii);
	ecm_db_iface_deref(ii);
}

/*
 * ecm_interface_netdev_notifier_callback()
 * 	Netdevice notifier callback to inform us of change of state of a netdevice
 */
static int ecm_interface_netdev_notifier_callback(struct notifier_block *this, unsigned long event, void *ptr)
{
	struct net_device *dev = netdev_notifier_info_to_dev(ptr);

	DEBUG_INFO("Net device notifier for: %p, name: %s, event: %lx\n", dev, dev->name, event);

	switch (event) {
	case NETDEV_DOWN:
		DEBUG_INFO("Net device: %p, DOWN\n", dev);
		ecm_interface_dev_regenerate_connections(dev);
		break;

	case NETDEV_CHANGE:
		DEBUG_INFO("Net device: %p, CHANGE\n", dev);
		if (!netif_carrier_ok(dev)) {
			DEBUG_INFO("Net device: %p, CARRIER BAD\n", dev);
			if (netif_is_bond_slave(dev)) {
				struct net_device *master;
				master = ecm_interface_get_and_hold_dev_master(dev);
				DEBUG_ASSERT(master, "Expected a master\n");
				ecm_interface_dev_regenerate_connections(master);
				dev_put(master);
			} else {
				ecm_interface_dev_regenerate_connections(dev);
			}
		}
		break;

	case NETDEV_CHANGEMTU:
		DEBUG_INFO("Net device: %p, MTU CHANGE\n", dev);
		ecm_interface_mtu_change(dev);
		break;

	default:
		DEBUG_TRACE("Net device: %p, UNHANDLED: %lx\n", dev, event);
		break;
	}

	return NOTIFY_DONE;
}

/*
 * struct notifier_block ecm_interface_netdev_notifier
 *	Registration for net device changes of state.
 */
static struct notifier_block ecm_interface_netdev_notifier __read_mostly = {
	.notifier_call		= ecm_interface_netdev_notifier_callback,
};

/*
 * ecm_interface_get_stop()
 */
static ssize_t ecm_interface_get_stop(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	ssize_t count;
	int num;

	/*
	 * Operate under our locks
	 */
	spin_lock_bh(&ecm_interface_lock);
	num = ecm_interface_stopped;
	spin_unlock_bh(&ecm_interface_lock);

	count = snprintf(buf, (ssize_t)PAGE_SIZE, "%d\n", num);
	return count;
}

void ecm_interface_stop(int num)
{
	/*
	 * Operate under our locks and stop further processing of packets
	 */
	spin_lock_bh(&ecm_interface_lock);
	ecm_interface_stopped = num;
	spin_unlock_bh(&ecm_interface_lock);

}
EXPORT_SYMBOL(ecm_interface_stop);


/*
 * ecm_interface_set_stop()
 */
static ssize_t ecm_interface_set_stop(struct device *dev,
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
	DEBUG_TRACE("ecm_interface_stop = %d\n", num);

	ecm_interface_stop(num);

	return count;
}

/*
 * System device attributes for the ECM interface.
 */
static DEVICE_ATTR(stop, 0644, ecm_interface_get_stop, ecm_interface_set_stop);

/*
 * Sub system node.
 * Sys device control points can be found at /sys/devices/system/ecm_interface/ecm_interfaceX/
 */
static struct bus_type ecm_interface_subsys = {
	.name = "ecm_interface",
	.dev_name = "ecm_interface",
};

/*
 * ecm_interface_dev_release()
 *	This is a dummy release function for device.
 */
static void ecm_interface_dev_release(struct device *dev)
{

}

/*
 * ecm_interface_init()
 */
int ecm_interface_init(void)
{
	int result;
	DEBUG_INFO("ECM Interface init\n");

	/*
	 * Initialise our global lock
	 */
	spin_lock_init(&ecm_interface_lock);

	/*
	 * Register the sub system
	 */
	result = subsys_system_register(&ecm_interface_subsys, NULL);
	if (result) {
		DEBUG_ERROR("Failed to register sub system %d\n", result);
		return result;
	}

	/*
	 * Register system device control
	 */
	memset(&ecm_interface_dev, 0, sizeof(ecm_interface_dev));
	ecm_interface_dev.id = 0;
	ecm_interface_dev.bus = &ecm_interface_subsys;
	ecm_interface_dev.release = &ecm_interface_dev_release;
	result = device_register(&ecm_interface_dev);
	if (result) {
		DEBUG_ERROR("Failed to register system device %d\n", result);
		goto task_cleanup_1;
	}

	/*
	 * Create files, one for each parameter supported by this module
	 */
	result = device_create_file(&ecm_interface_dev, &dev_attr_stop);
	if (result) {
		DEBUG_ERROR("Failed to register stop file %d\n", result);
		goto task_cleanup_2;
	}

	result = register_netdevice_notifier(&ecm_interface_netdev_notifier);
	if (result != 0) {
		DEBUG_ERROR("Failed to register netdevice notifier %d\n", result);
		goto task_cleanup_2;
	}

	return 0;

task_cleanup_2:
	device_unregister(&ecm_interface_dev);
task_cleanup_1:
	bus_unregister(&ecm_interface_subsys);

	return result;
}
EXPORT_SYMBOL(ecm_interface_init);

/*
 * ecm_interface_exit()
 */
void ecm_interface_exit(void)
{
	DEBUG_INFO("ECM Interface exit\n");

	spin_lock_bh(&ecm_interface_lock);
	ecm_interface_terminate_pending  = true;
	spin_unlock_bh(&ecm_interface_lock);

	unregister_netdevice_notifier(&ecm_interface_netdev_notifier);

	device_remove_file(&ecm_interface_dev, &dev_attr_stop);
	device_unregister(&ecm_interface_dev);
	bus_unregister(&ecm_interface_subsys);
}
EXPORT_SYMBOL(ecm_interface_exit);
