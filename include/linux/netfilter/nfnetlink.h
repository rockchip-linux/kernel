/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _NFNETLINK_H
#define _NFNETLINK_H

#include <linux/netlink.h>
#include <linux/capability.h>
#include <linux/android_kabi.h>
#include <net/netlink.h>
#include <uapi/linux/netfilter/nfnetlink.h>

struct nfnl_callback {
	int (*call)(struct net *net, struct sock *nl, struct sk_buff *skb,
		    const struct nlmsghdr *nlh,
		    const struct nlattr * const cda[],
		    struct netlink_ext_ack *extack);
	int (*call_rcu)(struct net *net, struct sock *nl, struct sk_buff *skb,
			const struct nlmsghdr *nlh,
			const struct nlattr * const cda[],
			struct netlink_ext_ack *extack);
	int (*call_batch)(struct net *net, struct sock *nl, struct sk_buff *skb,
			  const struct nlmsghdr *nlh,
			  const struct nlattr * const cda[],
			  struct netlink_ext_ack *extack);
	const struct nla_policy *policy;	/* netlink attribute policy */
	const u_int16_t attr_count;		/* number of nlattr's */

	ANDROID_KABI_RESERVE(1);
};

enum nfnl_abort_action {
	NFNL_ABORT_NONE		= 0,
	NFNL_ABORT_AUTOLOAD,
	NFNL_ABORT_VALIDATE,
};

struct nfnetlink_subsystem {
	const char *name;
	__u8 subsys_id;			/* nfnetlink subsystem ID */
	__u8 cb_count;			/* number of callbacks */
	const struct nfnl_callback *cb;	/* callback for individual types */
	struct module *owner;
	int (*commit)(struct net *net, struct sk_buff *skb);
	int (*abort)(struct net *net, struct sk_buff *skb,
		     enum nfnl_abort_action action);
	bool (*valid_genid)(struct net *net, u32 genid);

	ANDROID_KABI_RESERVE(1);
};

int nfnetlink_subsys_register(const struct nfnetlink_subsystem *n);
int nfnetlink_subsys_unregister(const struct nfnetlink_subsystem *n);

int nfnetlink_has_listeners(struct net *net, unsigned int group);
int nfnetlink_send(struct sk_buff *skb, struct net *net, u32 portid,
		   unsigned int group, int echo, gfp_t flags);
int nfnetlink_set_err(struct net *net, u32 portid, u32 group, int error);
int nfnetlink_unicast(struct sk_buff *skb, struct net *net, u32 portid);

static inline u16 nfnl_msg_type(u8 subsys, u8 msg_type)
{
	return subsys << 8 | msg_type;
}

static inline void nfnl_fill_hdr(struct nlmsghdr *nlh, u8 family, u8 version,
				 __be16 res_id)
{
	struct nfgenmsg *nfmsg;

	nfmsg = nlmsg_data(nlh);
	nfmsg->nfgen_family = family;
	nfmsg->version = version;
	nfmsg->res_id = res_id;
}

static inline struct nlmsghdr *nfnl_msg_put(struct sk_buff *skb, u32 portid,
					    u32 seq, int type, int flags,
					    u8 family, u8 version,
					    __be16 res_id)
{
	struct nlmsghdr *nlh;

	nlh = nlmsg_put(skb, portid, seq, type, sizeof(struct nfgenmsg), flags);
	if (!nlh)
		return NULL;

	nfnl_fill_hdr(nlh, family, version, res_id);

	return nlh;
}

void nfnl_lock(__u8 subsys_id);
void nfnl_unlock(__u8 subsys_id);
#ifdef CONFIG_PROVE_LOCKING
bool lockdep_nfnl_is_held(__u8 subsys_id);
#else
static inline bool lockdep_nfnl_is_held(__u8 subsys_id)
{
	return true;
}
#endif /* CONFIG_PROVE_LOCKING */

#define MODULE_ALIAS_NFNL_SUBSYS(subsys) \
	MODULE_ALIAS("nfnetlink-subsys-" __stringify(subsys))

#endif	/* _NFNETLINK_H */
