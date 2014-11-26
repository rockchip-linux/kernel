/*
 *	Linux ethernet bridge
 *
 *	Authors:
 *	Lennert Buytenhek		<buytenh@gnu.org>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */
#ifndef _LINUX_IF_BRIDGE_H
#define _LINUX_IF_BRIDGE_H


#include <linux/netdevice.h>
#include <uapi/linux/if_bridge.h>

extern void brioctl_set(int (*ioctl_hook)(struct net *, unsigned int, void __user *));
extern struct net_device *br_port_dev_get(struct net_device *dev, unsigned char *addr);
extern void br_refresh_fdb_entry(struct net_device *dev, const char *addr);
extern void br_dev_update_stats(struct net_device *dev, struct rtnl_link_stats64 *nlstats);

typedef int br_should_route_hook_t(struct sk_buff *skb);
extern br_should_route_hook_t __rcu *br_should_route_hook;

#endif
