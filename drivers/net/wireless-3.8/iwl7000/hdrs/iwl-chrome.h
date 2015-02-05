#ifndef __IWL_CHROME
#define __IWL_CHROME
/* This file is pre-included from the Makefile (cc command line) */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/idr.h>
#include <net/genetlink.h>

/* get the CPTCFG_* preprocessor symbols */
#include <hdrs/config.h>

/* mac80211 & backport */
#include <hdrs/mac80211-exp.h>
#include <hdrs/ieee80211.h>
#include <hdrs/mac80211-bp.h>
/* need to include mac80211 here, otherwise we get the regular kernel one */
#include <hdrs/mac80211.h>

/* artifacts of backports - never in upstream */
#define genl_info_snd_portid(__genl_info) (__genl_info->snd_portid)
#define NETLINK_CB_PORTID(__skb) NETLINK_CB(cb->skb).portid
#define netlink_notify_portid(__notify) __notify->portid

/* things that may or may not be upstream depending on the version */
#ifndef ETH_P_802_3_MIN
#define ETH_P_802_3_MIN 0x0600
#endif

#ifndef U8_MAX
#define U8_MAX		((u8)~0U)
#endif

#ifndef S8_MAX
#define S8_MAX		((s8)(U8_MAX>>1))
#endif

#ifndef S8_MIN
#define S8_MIN		((s8)(-S8_MAX - 1))
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0)
/* backport IDR APIs */
static inline void iwl7000_idr_destroy(struct idr *idp)
{
	idr_remove_all(idp);
	idr_destroy(idp);
}
#define idr_destroy(idp) iwl7000_idr_destroy(idp)

static inline int idr_alloc(struct idr *idr, void *ptr, int start, int end,
			    gfp_t gfp_mask)
{
	int id, ret;

	do {
		if (!idr_pre_get(idr, gfp_mask))
			return -ENOMEM;
		ret = idr_get_new_above(idr, ptr, start, &id);
		if (!ret && id > end) {
			idr_remove(idr, id);
			ret = -ENOSPC;
		}
	} while (ret == -EAGAIN);

	return ret ? ret : id;
}

static inline void idr_preload(gfp_t gfp_mask)
{
}

static inline void idr_preload_end(void)
{
}
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
#define netdev_notifier_info_to_dev(ndev)	ndev
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,12,0)
/* PCIe device capabilities flags have been renamed in (upstream)
 * commit d2ab1fa68c61f01b28ab0859a972c892d81f5d32 (PCI: Rename PCIe
 * capability definitions to follow convention).  This was just a
 * clean rename, without any functional changes.  We use one of the
 * renamed flags, so define it to the old one.
 */
#define PCI_EXP_DEVCTL2_LTR_EN PCI_EXP_LTR_EN

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,12,0) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
#define __genl_const
static inline int
_genl_register_family_with_ops_grps(struct genl_family *family,
				    struct genl_ops *ops, size_t n_ops,
				    struct genl_multicast_group *mcgrps,
				    size_t n_mcgrps)
{
	int ret, i;

	ret = genl_register_family_with_ops(family, ops, n_ops);
	if (ret)
		return ret;
	for (i = 0; i < n_mcgrps; i++) {
		ret = genl_register_mc_group(family, &mcgrps[i]);
		if (ret) {
			genl_unregister_family(family);
			return ret;
		}
	}

	return 0;
}
#define genl_register_family_with_ops_groups(family, ops, grps)		\
	_genl_register_family_with_ops_grps((family),			\
					    (ops), ARRAY_SIZE(ops),	\
					    (grps), ARRAY_SIZE(grps))
#else
#define __genl_const const
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,17,0)
/* interface name assignment types (sysfs name_assign_type attribute) */
#define NET_NAME_UNKNOWN	0	/* unknown origin (not exposed to userspace) */
#define NET_NAME_ENUM		1	/* enumerated by kernel */
#define NET_NAME_PREDICTABLE	2	/* predictably named by the kernel */
#define NET_NAME_USER		3	/* provided by user-space */
#define NET_NAME_RENAMED	4	/* renamed by user-space */

static inline struct net_device *
backport_alloc_netdev_mqs(int sizeof_priv, const char *name,
			  unsigned char name_assign_type,
			  void (*setup)(struct net_device *),
			  unsigned int txqs, unsigned int rxqs)
{
	return alloc_netdev_mqs(sizeof_priv, name, setup, txqs, rxqs);
}

#define alloc_netdev_mqs backport_alloc_netdev_mqs

#undef alloc_netdev
static inline struct net_device *
backport_alloc_netdev(int sizeof_priv, const char *name,
		      unsigned char name_assign_type,
		      void (*setup)(struct net_device *))
{
	return backport_alloc_netdev_mqs(sizeof_priv, name, name_assign_type,
					 setup, 1, 1);
}
#define alloc_netdev backport_alloc_netdev
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,17,0) */

#endif /* __IWL_CHROME */
