#ifndef __IWL_CHROME
#define __IWL_CHROME
/* This file is pre-included from the Makefile (cc command line) */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/idr.h>

/* get the CPTCFG_* preprocessor symbols */
#include <hdrs/config.h>

/* mac80211 & backport */
#include <hdrs/mac80211-exp.h>
#include <hdrs/ieee80211.h>
#include <hdrs/mac80211-bp.h>
/* need to include mac80211 here, otherwise we get the regular kernel one */
#include <hdrs/mac80211.h>

/* PCI compat stuff not needed for this kernel */
#define compat_pci_suspend(fn)
#define compat_pci_resume(fn)

/* this might not be there */
#ifndef ETH_P_802_3_MIN
#define ETH_P_802_3_MIN 0x0600
#endif

/* backport IDR APIs */
static inline void iwl7000_idr_destroy(struct idr *idp)
{
	idr_remove_all(idp);
	idr_destroy(idp);
}
#define idr_destroy(idp) iwl7000_idr_destroy(idp)

#if 0 // remove as it was implemented in 3.10 include/linux/idr.h
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
#endif

#define genl_info_snd_portid(__genl_info) (__genl_info->snd_portid)
#define NETLINK_CB_PORTID(__skb) NETLINK_CB(cb->skb).portid

#define netdev_notifier_info_to_dev(ndev)	ndev

#endif /* __IWL_CHROME */
