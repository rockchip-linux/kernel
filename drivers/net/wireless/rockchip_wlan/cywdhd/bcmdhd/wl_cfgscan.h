/*
 * Header for Linux cfg80211 scan
 *
 * Portions of this code are copyright (c) 2022 Cypress Semiconductor Corporation
 *
 * Copyright (C) 1999-2017, Broadcom Corporation
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id$
 */

#ifndef _wl_cfgscan_h_
#define _wl_cfgscan_h_

#include <linux/wireless.h>
#include <typedefs.h>
#include <ethernet.h>
#include <wlioctl.h>
#include <linux/wireless.h>
#include <net/cfg80211.h>
#include <linux/rfkill.h>
#include <osl.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 0))
#define GET_SCAN_WDEV(scan_request) \
	(scan_request && scan_request->dev) ? scan_request->dev->ieee80211_ptr : NULL;
#else
#define GET_SCAN_WDEV(scan_request) \
	scan_request ? scan_request->wdev : NULL;
#endif // endif
#ifdef WL_SCHED_SCAN
#define GET_SCHED_SCAN_WDEV(scan_request) \
	(scan_request && scan_request->dev) ? scan_request->dev->ieee80211_ptr : NULL;
#endif /* WL_SCHED_SCAN */

extern s32 wl_escan_handler(struct bcm_cfg80211 *cfg, bcm_struct_cfgdev *cfgdev,
	const wl_event_msg_t *e, void *data);
extern s32 wl_do_escan(struct bcm_cfg80211 *cfg, struct wiphy *wiphy,
	struct net_device *ndev, struct cfg80211_scan_request *request);
extern s32 __wl_cfg80211_scan(struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_scan_request *request, struct cfg80211_ssid *this_ssid);
#if defined(WL_CFG80211_P2P_DEV_IF)
extern s32 wl_cfg80211_scan(struct wiphy *wiphy, struct cfg80211_scan_request *request);
#else
extern s32 wl_cfg80211_scan(struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_scan_request *request);
extern int wl_cfg80211_scan_stop(struct bcm_cfg80211 *cfg, bcm_struct_cfgdev *cfgdev);
#endif /* WL_CFG80211_P2P_DEV_IF */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0))
extern void wl_cfg80211_abort_scan(struct wiphy *wiphy, struct wireless_dev *wdev);
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)) */
extern void wl_cfg80211_scan_abort(struct bcm_cfg80211 *cfg);
extern s32 wl_init_scan(struct bcm_cfg80211 *cfg);
extern int wl_cfg80211_scan_stop(struct bcm_cfg80211 *cfg, bcm_struct_cfgdev *cfgdev);
extern s32 wl_notify_scan_status(struct bcm_cfg80211 *cfg, bcm_struct_cfgdev *cfgdev,
	const wl_event_msg_t *e, void *data);
extern void wl_cfg80211_set_passive_scan(struct net_device *dev, char *command);
#ifdef PNO_SUPPORT
extern s32 wl_notify_pfn_status(struct bcm_cfg80211 *cfg, bcm_struct_cfgdev *cfgdev,
	const wl_event_msg_t *e, void *data);
#endif /* PNO_SUPPORT */
#ifdef GSCAN_SUPPORT
extern s32 wl_notify_gscan_event(struct bcm_cfg80211 *cfg, bcm_struct_cfgdev *cfgdev,
	const wl_event_msg_t *e, void *data);
#endif /* GSCAN_SUPPORT */

#ifdef WES_SUPPORT
#ifdef CUSTOMER_SCAN_TIMEOUT_SETTING
#define CUSTOMER_WL_SCAN_TIMER_INTERVAL_MS	25000 /* Scan timeout */
enum wl_custom_scan_time_type {
	WL_CUSTOM_SCAN_CHANNEL_TIME = 0,
	WL_CUSTOM_SCAN_UNASSOC_TIME,
	WL_CUSTOM_SCAN_PASSIVE_TIME,
	WL_CUSTOM_SCAN_HOME_TIME,
	WL_CUSTOM_SCAN_HOME_AWAY_TIME
};
extern s32 wl_cfg80211_custom_scan_time(struct net_device *dev,
		enum wl_custom_scan_time_type type, int time);
#endif /* CUSTOMER_SCAN_TIMEOUT_SETTING */
#endif /* WES_SUPPORT */

#if defined(SUPPORT_RANDOM_MAC_SCAN)
int wl_cfg80211_set_random_mac(struct net_device *dev, bool enable);
int wl_cfg80211_random_mac_enable(struct net_device *dev);
int wl_cfg80211_random_mac_disable(struct net_device *dev);
int wl_cfg80211_scan_mac_enable(struct net_device *dev, uint8 *rand_mac, uint8 *rand_mask);
int wl_cfg80211_scan_mac_disable(struct net_device *dev);
#endif /* SUPPORT_RANDOM_MAC_SCAN */

#ifdef WL_SCHED_SCAN
extern int wl_cfg80211_sched_scan_start(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_sched_scan_request *request);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0))
int
wl_cfg80211_sched_scan_stop(struct wiphy *wiphy, struct net_device *dev, u64 req);
#else
int wl_cfg80211_sched_scan_stop(struct wiphy *wiphy, struct net_device *dev);
#endif /* LINUX_VERSION_CODE <= KERNEL_VERSION(4, 11, 0) */
#endif /* WL_SCHED_SCAN */
#endif /* _wl_cfgscan_h_ */
