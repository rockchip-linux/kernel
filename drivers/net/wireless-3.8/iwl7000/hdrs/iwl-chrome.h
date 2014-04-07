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
#define compat_pci_suspend(fn)
#define compat_pci_resume(fn)
#define genl_info_snd_portid(__genl_info) (__genl_info->snd_portid)
#define NETLINK_CB_PORTID(__skb) NETLINK_CB(cb->skb).portid

/* things that may or may not be upstream depending on the version */
#ifndef ETH_P_802_3_MIN
#define ETH_P_802_3_MIN 0x0600
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

#endif /* __IWL_CHROME */
