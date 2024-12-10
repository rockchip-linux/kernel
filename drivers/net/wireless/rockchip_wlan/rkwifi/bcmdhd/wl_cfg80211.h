/*
 * Linux cfg80211 driver
 *
 * Copyright (C) 2020, Broadcom.
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
 *
 * <<Broadcom-WL-IPTag/Dual:>>
 */

/**
 * Older Linux versions support the 'iw' interface, more recent ones the 'cfg80211' interface.
 */

#ifndef _wl_cfg80211_h_
#define _wl_cfg80211_h_

#include <linux/wireless.h>
#include <typedefs.h>
#include <ethernet.h>
#include <wlioctl.h>
#include <linux/wireless.h>
#include <net/cfg80211.h>
#include <linux/rfkill.h>
#include <osl.h>
#if defined(BCMDONGLEHOST)
#include <dngl_stats.h>
#include <dhd.h>
#endif /* BCMDONGLEHOST */

#define WL_CFG_DRV_LOCK(lock, flags)	(flags) = osl_spin_lock(lock)
#define WL_CFG_DRV_UNLOCK(lock, flags)	osl_spin_unlock((lock), (flags))

#define WL_CFG_WPS_SYNC_LOCK(lock, flags)	(flags) = osl_spin_lock(lock)
#define WL_CFG_WPS_SYNC_UNLOCK(lock, flags)	osl_spin_unlock((lock), (flags))

#define WL_CFG_NET_LIST_SYNC_LOCK(lock, flags)		(flags) = osl_spin_lock(lock)
#define WL_CFG_NET_LIST_SYNC_UNLOCK(lock, flags)	osl_spin_unlock((lock), (flags))

#define WL_CFG_EQ_LOCK(lock, flags)	(flags) = osl_spin_lock(lock)
#define WL_CFG_EQ_UNLOCK(lock, flags)	osl_spin_unlock((lock), (flags))

#define WL_CFG_BAM_LOCK(lock, flags)	(flags) = osl_spin_lock(lock)
#define WL_CFG_BAM_UNLOCK(lock, flags)	osl_spin_unlock((lock), (flags))

#define WL_CFG_VNDR_OUI_SYNC_LOCK(lock, flags)		(flags) = osl_spin_lock(lock)
#define WL_CFG_VNDR_OUI_SYNC_UNLOCK(lock, flags)	osl_spin_unlock((lock), (flags))

#include <wl_cfgp2p.h>
#include <wl_android.h>
#ifdef WL_NAN
#include <wl_cfgnan.h>
#endif /* WL_NAN */
#ifdef WL_BAM
#include <wl_bam.h>
#endif  /* WL_BAM */

#ifdef BIGDATA_SOFTAP
#include <wl_bigdata.h>
#endif /* BIGDATA_SOFTAP */
#include <dhd_dbg.h>

struct wl_conf;
struct wl_iface;
struct bcm_cfg80211;
struct wl_security;
struct wl_ibss;

/* Enable by default */
#define WL_WTC

/*
 * Common feature. If this becomes customer specific,
 * move it to customer specific makefile when required
 */
#define WL_5G_SOFTAP_ONLY_ON_DEF_CHAN

#if !defined(WL_CLIENT_SAE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 17, 0))
#define WL_CLIENT_SAE
#endif
#if defined(WL_SAE) && (LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0))
#error "Can not support WL_SAE befor kernel 3.14"
#endif
#if defined(WL_CLIENT_SAE) && (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
#error "Can not support WL_CLIENT_SAE before kernel 3.10"
#endif
#if defined(WL_CLIENT_SAE) && defined(WL_SAE)
#error "WL_SAE is for dongle-offload and WL_CLIENT_SAE is for wpa_supplicant. Please choose one."
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 17, 0) && !defined(WL_SCAN_TYPE))
#define WL_SCAN_TYPE
#endif /* WL_SCAN_TYPE */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)) && !defined(WL_FILS)
#define WL_FILS
#endif /* WL_FILS */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 18, 0)) && !defined(WL_FILS_ROAM_OFFLD)
#define WL_FILS_ROAM_OFFLD
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
/* Use driver managed regd */
#define WL_SELF_MANAGED_REGDOM
#endif /* KERNEL >= 4.0 */

#define CH_TO_CHSPC(band, _channel) \
	((_channel | band) | WL_CHANSPEC_BW_20 | WL_CHANSPEC_CTL_SB_NONE)
#define CHAN2G(_channel, _freq, _flags) {			\
	.band			= IEEE80211_BAND_2GHZ,		\
	.center_freq		= (_freq),			\
	.hw_value		= CH_TO_CHSPC(WL_CHANSPEC_BAND_2G, _channel),			\
	.flags			= (_flags),			\
	.max_antenna_gain	= 0,				\
	.max_power		= 30,				\
}

#define CHAN5G(_channel, _flags) {				\
	.band			= IEEE80211_BAND_5GHZ,		\
	.center_freq		= 5000 + (5 * (_channel)),	\
	.hw_value		= CH_TO_CHSPC(WL_CHANSPEC_BAND_5G, _channel),			\
	.flags			= (_flags),			\
	.max_antenna_gain	= 0,				\
	.max_power		= 30,				\
}

#ifdef CFG80211_6G_SUPPORT
#define CHAN6G(_channel, _flags) {				\
	.band			= IEEE80211_BAND_6GHZ,		\
	.center_freq		= 5950 + (5 * (_channel)),	\
	.hw_value		= CH_TO_CHSPC(WL_CHANSPEC_BAND_6G, _channel),			\
	.flags			= (_flags),			\
	.max_antenna_gain	= 0,				\
	.max_power		= 30,				\
}

#define CHAN6G_CHAN2(_flags) {					\
	.band			= IEEE80211_BAND_6GHZ,		\
	.center_freq		= 5935,				\
	.hw_value		= 0x5002,			\
	.flags			= (_flags),			\
	.max_antenna_gain	= 0,				\
	.max_power		= 30,				\
}
#else
#define CHAN6G(_channel, _flags) {				\
	.band			= IEEE80211_BAND_5GHZ,		\
	.center_freq		= 5950 + (5 * (_channel)),	\
	.hw_value		= CH_TO_CHSPC(WL_CHANSPEC_BAND_6G, _channel),			\
	.flags			= (_flags),			\
	.max_antenna_gain	= 0,				\
	.max_power		= 30,				\
}

#define CHAN6G_CHAN2(_flags) {					\
	.band			= IEEE80211_BAND_5GHZ,		\
	.center_freq		= 5935,				\
	.hw_value		= 0x5002,			\
	.flags			= (_flags),			\
	.max_antenna_gain	= 0,				\
	.max_power		= 30,				\
}
#endif /* CFG80211_6G_SUPPORT */

#ifdef WL_SAE
#define IS_AKM_SAE(akm) (akm == WLAN_AKM_SUITE_SAE)
#else
#define IS_AKM_SAE(akm) FALSE
#endif
#ifdef WL_OWE
#define IS_AKM_OWE(akm) (akm == WLAN_AKM_SUITE_OWE)
#else
#define IS_AKM_OWE(akm) FALSE
#endif

#if defined(IL_BIGENDIAN)
#include <bcmendian.h>
#define htod32(i) (bcmswap32(i))
#define htod16(i) (bcmswap16(i))
#define dtoh64(i) (bcmswap64(i))
#define dtoh32(i) (bcmswap32(i))
#define dtoh16(i) (bcmswap16(i))
#define htodchanspec(i) htod16(i)
#define dtohchanspec(i) dtoh16(i)
#else
#define htod32(i) (i)
#define htod16(i) (i)
#define dtoh64(i) (i)
#define dtoh32(i) (i)
#define dtoh16(i) (i)
#define htodchanspec(i) (i)
#define dtohchanspec(i) (i)
#endif /* IL_BIGENDIAN */

#define WL_DBG_NONE	0
#define WL_DBG_P2P_ACTION	(1 << 5)
#define WL_DBG_TRACE	(1 << 4)
#define WL_DBG_SCAN	(1 << 3)
#define WL_DBG_DBG	(1 << 2)
#define WL_DBG_INFO	(1 << 1)
#define WL_DBG_ERR	(1 << 0)

#ifndef WAIT_FOR_DISCONNECT_MAX
#define WAIT_FOR_DISCONNECT_MAX 10
#endif /* WAIT_FOR_DISCONNECT_MAX */
#define WAIT_FOR_DISCONNECT_STATE_SYNC 10

#if defined(CONFIG_6GHZ_BKPORT) || (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
/* Native 6GHz band supported available. For Backported
 * kernels, kernels/customer makefiles should explicitly
 * define CONFIG_6GHZ_BKPORT
 */
#if defined(WL_6G_BAND)
#define CFG80211_6G_SUPPORT
#endif
#endif /* CONFIG_6GHZ_BKPORT || LINUX_VER >= 5.4 */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0))
/* Newer kernels use defines from nl80211.h */
#define IEEE80211_BAND_2GHZ	NL80211_BAND_2GHZ
#define IEEE80211_BAND_5GHZ	NL80211_BAND_5GHZ
#define IEEE80211_BAND_60GHZ	NL80211_BAND_60GHZ
#ifdef CFG80211_6G_SUPPORT
#define IEEE80211_BAND_6GHZ	NL80211_BAND_6GHZ
#endif /* CFG80211_6G_SUPPORT */
#define IEEE80211_NUM_BANDS	NUM_NL80211_BANDS
#endif /* LINUX_VER >= 4.7 */

/* Max BAND support */
#define WL_MAX_BAND_SUPPORT 3

#ifdef DHD_LOG_DUMP
extern void dhd_log_dump_write(int type, char *binary_data,
		int binary_len, const char *fmt, ...);
extern char *dhd_log_dump_get_timestamp(void);
extern char *dhd_dbg_get_system_timestamp(void);
#ifndef _DHD_LOG_DUMP_DEFINITIONS_
#define DHD_LOG_DUMP_WRITE(fmt, ...) \
	dhd_log_dump_write(DLD_BUF_TYPE_GENERAL, NULL, 0, fmt, ##__VA_ARGS__)
#define DHD_LOG_DUMP_WRITE_EX(fmt, ...) \
	dhd_log_dump_write(DLD_BUF_TYPE_SPECIAL, NULL, 0, fmt, ##__VA_ARGS__)
#define DHD_LOG_DUMP_WRITE_PRSRV(fmt, ...) \
	dhd_log_dump_write(DLD_BUF_TYPE_PRESERVE, NULL, 0, fmt, ##__VA_ARGS__)
#endif /* !_DHD_LOG_DUMP_DEFINITIONS_ */

#ifndef DHD_LOG_DUMP_RING_DEFINITIONS
#define DHD_PREFIX_TS "[%s]: ", dhd_log_dump_get_timestamp()
#define DHD_PREFIX_TS_FN "[%s] %s: ", dhd_log_dump_get_timestamp(), __func__

#define DHD_LOG_DUMP_WRITE_TS		DHD_LOG_DUMP_WRITE(DHD_PREFIX_TS)
#define DHD_LOG_DUMP_WRITE_TS_FN	DHD_LOG_DUMP_WRITE(DHD_PREFIX_TS_FN)

#define DHD_LOG_DUMP_WRITE_EX_TS	DHD_LOG_DUMP_WRITE_EX(DHD_PREFIX_TS)
#define DHD_LOG_DUMP_WRITE_EX_TS_FN	DHD_LOG_DUMP_WRITE_EX(DHD_PREFIX_TS_FN)

#define DHD_LOG_DUMP_WRITE_PRSRV_TS	DHD_LOG_DUMP_WRITE_PRSRV(DHD_PREFIX_TS)
#define DHD_LOG_DUMP_WRITE_PRSRV_TS_FN	DHD_LOG_DUMP_WRITE_PRSRV(DHD_PREFIX_TS_FN)

#define DHD_LOG_DUMP_WRITE_ROAM_TS	DHD_LOG_DUMP_WRITE(DHD_PREFIX_TS)
#define DHD_LOG_DUMP_WRITE_ROAM_TS_FN	DHD_LOG_DUMP_WRITE(DHD_PREFIX_TS_FN)
#endif /* DHD_LOG_DUMP_RING_DEFINITIONS */
#endif /* DHD_LOG_DUMP */

/* Data Element Definitions */
#define WPS_ID_CONFIG_METHODS     0x1008
#define WPS_ID_REQ_TYPE           0x103A
#define WPS_ID_DEVICE_NAME        0x1011
#define WPS_ID_VERSION            0x104A
#define WPS_ID_DEVICE_PWD_ID      0x1012
#define WPS_ID_REQ_DEV_TYPE       0x106A
#define WPS_ID_SELECTED_REGISTRAR_CONFIG_METHODS 0x1053
#define WPS_ID_PRIM_DEV_TYPE      0x1054

/* Device Password ID */
#define DEV_PW_DEFAULT 0x0000
#define DEV_PW_USER_SPECIFIED 0x0001,
#define DEV_PW_MACHINE_SPECIFIED 0x0002
#define DEV_PW_REKEY 0x0003
#define DEV_PW_PUSHBUTTON 0x0004
#define DEV_PW_REGISTRAR_SPECIFIED 0x0005

/* Config Methods */
#define WPS_CONFIG_USBA 0x0001
#define WPS_CONFIG_ETHERNET 0x0002
#define WPS_CONFIG_LABEL 0x0004
#define WPS_CONFIG_DISPLAY 0x0008
#define WPS_CONFIG_EXT_NFC_TOKEN 0x0010
#define WPS_CONFIG_INT_NFC_TOKEN 0x0020
#define WPS_CONFIG_NFC_INTERFACE 0x0040
#define WPS_CONFIG_PUSHBUTTON 0x0080
#define WPS_CONFIG_KEYPAD 0x0100
#define WPS_CONFIG_VIRT_PUSHBUTTON 0x0280
#define WPS_CONFIG_PHY_PUSHBUTTON 0x0480
#define WPS_CONFIG_VIRT_DISPLAY 0x2008
#define WPS_CONFIG_PHY_DISPLAY 0x4008

#define PM_BLOCK 1
#define PM_ENABLE 0

#ifdef SUPPORT_AP_RADIO_PWRSAVE
#define RADIO_PWRSAVE_PPS               10
#define RADIO_PWRSAVE_QUIET_TIME        10
#define RADIO_PWRSAVE_LEVEL             3
#define RADIO_PWRSAVE_STAS_ASSOC_CHECK  0

#define RADIO_PWRSAVE_LEVEL_MIN         1
#define RADIO_PWRSAVE_LEVEL_MAX         9
#define RADIO_PWRSAVE_PPS_MIN           1
#define RADIO_PWRSAVE_QUIETTIME_MIN     1
#define RADIO_PWRSAVE_ASSOCCHECK_MIN    0
#define RADIO_PWRSAVE_ASSOCCHECK_MAX    1

#define RADIO_PWRSAVE_MAJOR_VER         1
#define RADIO_PWRSAVE_MINOR_VER         1
#define RADIO_PWRSAVE_MAJOR_VER_SHIFT   8
#define RADIO_PWRSAVE_VERSION \
	((RADIO_PWRSAVE_MAJOR_VER << RADIO_PWRSAVE_MAJOR_VER_SHIFT)| RADIO_PWRSAVE_MINOR_VER)
#endif /* SUPPORT_AP_RADIO_PWRSAVE */

#ifdef BCMWAPI_WPI
#ifdef CFG80211_WAPI_BKPORT
#define IS_WAPI_VER(version) (version == NL80211_WAPI_VERSION_1)
#undef WLAN_AKM_SUITE_WAPI_PSK
#define WLAN_AKM_SUITE_WAPI_PSK			0x000FAC13
#undef WLAN_AKM_SUITE_WAPI_CERT
#define WLAN_AKM_SUITE_WAPI_CERT		0x000FAC14
#else
#ifdef OEM_ANDROID
#undef NL80211_WAPI_VERSION_1
#define NL80211_WAPI_VERSION_1		0

#undef WLAN_AKM_SUITE_WAPI_PSK
#define WLAN_AKM_SUITE_WAPI_PSK		0x000FACFE /* WAPI */

#undef WLAN_AKM_SUITE_WAPI_CERT
#define WLAN_AKM_SUITE_WAPI_CERT	0x000FACFF /* WAPI */

#define IS_WAPI_VER(version) (version == NL80211_WAPI_VERSION_1)
#else
#undef WLAN_AKM_SUITE_WAPI_PSK
#define WLAN_AKM_SUITE_WAPI_PSK         0x000FAC04

#undef WLAN_AKM_SUITE_WAPI_CERT
#define WLAN_AKM_SUITE_WAPI_CERT        0x000FAC12

#undef NL80211_WAPI_VERSION_1
#define NL80211_WAPI_VERSION_1			1 << 2
#define IS_WAPI_VER(version) (version & NL80211_WAPI_VERSION_1)
#endif /* OEM_ANDROID */
#endif /* CFG80211_WAPI_BKPORT */
#endif /* BCMWAPI_WPI */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
#define IS_REGDOM_SELF_MANAGED(wiphy)	\
	(wiphy->regulatory_flags & REGULATORY_WIPHY_SELF_MANAGED)
#else
#define IS_REGDOM_SELF_MANAGED(wiphy)	(false)
#endif /* KERNEL >= 4.0 */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)) && \
	defined(WL_SELF_MANAGED_REGDOM)
#define WL_UPDATE_CUSTOM_REGULATORY(wiphy) \
	wiphy->regulatory_flags |= REGULATORY_WIPHY_SELF_MANAGED;
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
#define WL_UPDATE_CUSTOM_REGULATORY(wiphy) \
	wiphy->regulatory_flags |= REGULATORY_CUSTOM_REG;
#else /* kernel > 4.0 && WL_SELF_MANAGED_REGDOM */
/* Kernels < 3.14 */
#define WL_UPDATE_CUSTOM_REGULATORY(wiphy) \
	wiphy->flags |= WIPHY_FLAG_CUSTOM_REGULATORY;
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0)) */

/* GCMP crypto supported above kernel v4.0 */
#if (LINUX_VERSION_CODE > KERNEL_VERSION(4, 0, 0)) && defined(WL_GCMP_SUPPORT)
/* Check for minimal kernel version before enabling WL_GCMP */
#define WL_GCMP
#endif /* (LINUX_VERSION > KERNEL_VERSION(4, 0, 0) && WL_GCMP_SUPPORT */

#ifndef IBSS_COALESCE_ALLOWED
#define IBSS_COALESCE_ALLOWED IBSS_COALESCE_DEFAULT
#endif

#ifndef IBSS_INITIAL_SCAN_ALLOWED
#define IBSS_INITIAL_SCAN_ALLOWED IBSS_INITIAL_SCAN_ALLOWED_DEFAULT
#endif

#define CUSTOM_RETRY_MASK 0xff000000 /* Mask for retry counter of custom dwell time */

/* On some MSM platform, it uses different version
 * of linux kernel and cfg code as not synced.
 * MSM defined CFG80211_DISCONNECTED_V2 as the flag
 * when they uses different kernel/cfg version.
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 2, 0)) || \
	   (defined(CONFIG_ARCH_MSM) && defined(CFG80211_DISCONNECTED_V2))
#define CFG80211_DISCONNECTED(dev, reason, ie, len, loc_gen, gfp) \
	cfg80211_disconnected(dev, reason, ie, len, loc_gen, gfp);
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0))
#define CFG80211_DISCONNECTED(dev, reason, ie, len, loc_gen, gfp) \
	BCM_REFERENCE(loc_gen); \
	cfg80211_disconnected(dev, reason, ie, len, gfp);
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 2, 0)) */

/* 0 invalidates all debug messages.  default is 1 */
#define WL_DBG_LEVEL 0xFF

#if defined(CUSTOMER_DBG_SYSTEM_TIME) && defined(DHD_DEBUGABILITY_LOG_DUMP_RING)
#define WL_DBG_PRINT_SYSTEM_TIME \
	pr_cont("[%s]", dhd_dbg_get_system_timestamp())
#else
#define WL_DBG_PRINT_SYSTEM_TIME
#endif /* defined(CUSTOMER_DBG_SYSTEM_TIME) && defined(DHD_DEBUGABILITY_LOG_DUMP_RING) */

#if defined(CUSTOMER_DBG_PREFIX_ENABLE)
#define USER_PREFIX_CFG80211		"[cfg80211][wlan] "
#define CFG80211_INFO_TEXT		USER_PREFIX_CFG80211
#define CFG80211_ERROR_TEXT		USER_PREFIX_CFG80211
#define CFG80211_SCAN_TEXT		USER_PREFIX_CFG80211
#define CFG80211_TRACE_TEXT		USER_PREFIX_CFG80211
#define CFG80211_DEBUG_TEXT		USER_PREFIX_CFG80211
#else
#define CFG80211_INFO_TEXT		"CFG80211-INFO) "
/* Samsung want to print INFO2 instead of ERROR
 * because most of case, ERROR message is not a real ERROR.
 * but it can be regarded as real error case for Tester
 */
#ifdef CUSTOMER_HW4_DEBUG
#define CFG80211_ERROR_TEXT		"CFG80211-INFO2) "
#else
#define CFG80211_ERROR_TEXT		"CFG80211-ERROR) "
#endif /* CUSTOMER_HW4_DEBUG */
#define CFG80211_SCAN_TEXT		"CFG80211-SCAN) "
#define CFG80211_TRACE_TEXT		"CFG80211-TRACE) "
#define CFG80211_DEBUG_TEXT		"CFG80211-DEBUG) "
#endif /* defined(CUSTOMER_DBG_PREFIX_ENABLE) */

#ifdef DHD_DEBUG
#ifdef DHD_LOG_DUMP
#define	WL_ERR_MSG(x, args...)	\
do {	\
	if (wl_dbg_level & WL_DBG_ERR) {	\
		printf(CFG80211_ERROR_TEXT "%s : " x, __func__, ## args);	\
		DHD_LOG_DUMP_WRITE_TS_FN;	\
		DHD_LOG_DUMP_WRITE(x, ## args);	\
	}	\
} while (0)
#define WL_ERR(x) WL_ERR_MSG x
#define WL_ERR_KERN_MSG(x, args...)	\
do {	\
	if (wl_dbg_level & WL_DBG_ERR) {	\
		printf(CFG80211_ERROR_TEXT "%s : " x, __func__, ## args);	\
	}	\
} while (0)
#define WL_ERR_KERN(x) WL_ERR_KERN_MSG x
#define	WL_ERR_MEM_MSG(x, args...)	\
do {	\
	if (wl_dbg_level & WL_DBG_ERR) {	\
		DHD_LOG_DUMP_WRITE_TS_FN;	\
		DHD_LOG_DUMP_WRITE(x, ## args);	\
	}	\
} while (0)
/* Prints to debug ring by default. If dbg level is enabled, prints on to
 * console as well
 */
#define	WL_DBG_MEM_MSG(x, args...)	\
do {	\
	if (wl_dbg_level & WL_DBG_DBG) {	\
		printf(CFG80211_INFO_TEXT "%s : " x, __func__, ## args);	\
	}	\
	DHD_LOG_DUMP_WRITE_TS_FN;		\
	DHD_LOG_DUMP_WRITE(x, ## args);	\
} while (0)
#define WL_DBG_MEM(x) WL_DBG_MEM_MSG x
#define WL_ERR_MEM(x) WL_ERR_MEM_MSG x
#define	WL_INFORM_MEM_MSG(x, args...)	\
do {	\
	if (wl_dbg_level & WL_DBG_INFO) {	\
		printf(CFG80211_INFO_TEXT "%s : " x, __func__, ## args);	\
		DHD_LOG_DUMP_WRITE_TS_FN;	\
		DHD_LOG_DUMP_WRITE(x, ## args);	\
	}	\
} while (0)
#define WL_INFORM_MEM(x) WL_INFORM_MEM_MSG x
#define	WL_ERR_EX_MSG(x, args...)	\
do {	\
	if (wl_dbg_level & WL_DBG_ERR) {	\
		printf(CFG80211_ERROR_TEXT "%s : " x, __func__, ## args);	\
		DHD_LOG_DUMP_WRITE_EX_TS_FN;	\
		DHD_LOG_DUMP_WRITE_EX(x, ## args);	\
	}	\
} while (0)
#define WL_ERR_EX(x) WL_ERR_EX_MSG x
#define	WL_MEM(args)	\
do {	\
	DHD_LOG_DUMP_WRITE_TS_FN;	\
	DHD_LOG_DUMP_WRITE args;	\
} while (0)
#else
#define	WL_ERR_MSG(x, args...)									\
do {										\
	if (wl_dbg_level & WL_DBG_ERR) {				\
		printf(CFG80211_ERROR_TEXT "%s : " x, __func__, ## args);	\
	}								\
} while (0)
#define WL_ERR(x) WL_ERR_MSG x
#define WL_ERR_KERN(args) WL_ERR(args)
#define WL_ERR_MEM(args) WL_ERR(args)
#define WL_INFORM_MEM(args) WL_INFORM(args)
#define WL_DBG_MEM(args) WL_DBG(args)
#define WL_ERR_EX(args) WL_ERR(args)
#define WL_MEM(args) WL_DBG(args)
#endif /* DHD_LOG_DUMP */
#else /* defined(DHD_DEBUG) */
#define	WL_ERR_MSG(x, args...)									\
do {										\
	if ((wl_dbg_level & WL_DBG_ERR) && net_ratelimit()) {				\
		printf(CFG80211_ERROR_TEXT "%s : " x, __func__, ## args);	\
	}								\
} while (0)
#define WL_ERR(x) WL_ERR_MSG x
#define WL_ERR_KERN(args) WL_ERR(args)
#define WL_ERR_MEM(args) WL_ERR(args)
#define WL_INFORM_MEM(args) WL_INFORM(args)
#define WL_DBG_MEM(args) WL_DBG(args)
#define WL_ERR_EX(args) WL_ERR(args)
#define WL_MEM(args) WL_DBG(args)
#endif /* defined(DHD_DEBUG) */

#if defined(__linux__) && !defined(DHD_EFI)
#define WL_PRINT_RATE_LIMIT_PERIOD 4000000000u /* 4s in units of ns */
#endif
#if defined(__linux__) && !defined(DHD_EFI)
#define WL_ERR_RLMT(args) \
do {	\
	if (wl_dbg_level & WL_DBG_ERR) {	\
		static uint64 __err_ts = 0; \
		static uint32 __err_cnt = 0; \
		uint64 __cur_ts = 0; \
		__cur_ts = local_clock(); \
		if (__err_ts == 0 || (__cur_ts > __err_ts && \
		(__cur_ts - __err_ts > WL_PRINT_RATE_LIMIT_PERIOD))) { \
			__err_ts = __cur_ts; \
			WL_ERR(args);	\
			WL_ERR(("[Repeats %u times]\n", __err_cnt)); \
			__err_cnt = 0; \
		} else { \
			++__err_cnt; \
		} \
	}	\
} while (0)
#else /* defined(__linux__) && !defined(DHD_EFI) */
#define WL_ERR_RLMT(args) WL_ERR(args)
#endif /* defined(__linux__) && !defined(DHD_EFI) */

#ifdef WL_INFORM
#undef WL_INFORM
#endif

#define	WL_INFORM_MSG(x, args...)									\
do {										\
	if (wl_dbg_level & WL_DBG_INFO) {				\
		printf(CFG80211_INFO_TEXT "%s : " x, __func__, ## args);	\
	}								\
} while (0)
#define WL_INFORM(x) WL_INFORM_MSG x

#ifdef WL_SCAN
#undef WL_SCAN
#endif
#define	WL_SCAN_MSG(x, args...)								\
do {									\
	if (wl_dbg_level & WL_DBG_SCAN) {			\
		printf(CFG80211_SCAN_TEXT "%s : " x, __func__, ## args);	\
	}									\
} while (0)
#define WL_SCAN(x) WL_SCAN_MSG x
#ifdef WL_TRACE
#undef WL_TRACE
#endif
#define	WL_TRACE_MSG(x, args...)								\
do {									\
	if (wl_dbg_level & WL_DBG_TRACE) {			\
		printf(CFG80211_TRACE_TEXT "%s : " x, __func__, ## args); \
	}									\
} while (0)
#define WL_TRACE(x) WL_TRACE_MSG x
#ifdef WL_TRACE_HW4
#undef WL_TRACE_HW4
#endif
#ifdef CUSTOMER_HW4_DEBUG
#define	WL_TRACE_HW4_MSG(x, args...)					\
do {										\
	if (wl_dbg_level & WL_DBG_ERR) {				\
		printf(CFG80211_TRACE_TEXT "%s : " x, __func__, ## args); \
	}								\
} while (0)
#define WL_TRACE_HW4(x) WL_TRACE_HW4_MSG x
#else
#define	WL_TRACE_HW4			WL_TRACE
#endif /* CUSTOMER_HW4_DEBUG */
#if (WL_DBG_LEVEL > 0)
#define	WL_DBG_MSG(x, args...)								\
do {									\
	if (wl_dbg_level & WL_DBG_DBG) {			\
		printf(CFG80211_DEBUG_TEXT "%s : " x, __func__, ## args); \
	}									\
} while (0)
#define WL_DBG(x) WL_DBG_MSG x
#else				/* !(WL_DBG_LEVEL > 0) */
#define	WL_DBG(args)
#endif				/* (WL_DBG_LEVEL > 0) */
#define WL_PNO(x)
#define WL_SD(x)

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0))
#define ieee80211_band nl80211_band
#define IEEE80211_BAND_2GHZ NL80211_BAND_2GHZ
#define IEEE80211_BAND_5GHZ NL80211_BAND_5GHZ
#define IEEE80211_NUM_BANDS NUM_NL80211_BANDS
#endif

#define WL_SCAN_RETRY_MAX   3
#define WL_NUM_PMKIDS_MAX   MAXPMKID
#define WL_SCAN_BUF_MAX     (1024 * 8)
#define WL_TLV_INFO_MAX     1500
#define WL_SCAN_IE_LEN_MAX  2048
#define WL_BSS_INFO_MAX     2048
#define WL_ASSOC_INFO_MAX   512
/* the length of pmkid_info iovar is 1416
 * It exceed the original 1024 limitation
 * so change WL_EXTRA_LEN_MAX to 2048
 */
#define WL_IOCTL_LEN_MAX        2048
#define WL_EXTRA_BUF_MAX        2048
#define WL_SCAN_ERSULTS_LAST    (WL_SCAN_RESULTS_NO_MEM+1)
#define WL_AP_MAX			    256
#define WL_FILE_NAME_MAX        256
#define WL_DEFAULT_DWELL_TIME   200
#define WL_MED_DWELL_TIME       400
#define WL_MIN_DWELL_TIME       100
#define WL_LONG_DWELL_TIME      1000
#define IFACE_MAX_CNT           5
#define WL_SCAN_CONNECT_DWELL_TIME_MS		200
#define WL_SCAN_JOIN_PROBE_INTERVAL_MS		20
#define WL_SCAN_JOIN_ACTIVE_DWELL_TIME_MS	320
#define WL_BCAST_SCAN_JOIN_ACTIVE_DWELL_TIME_MS	80
#define WL_SCAN_JOIN_PASSIVE_DWELL_TIME_MS	400
#define WL_AF_TX_MAX_RETRY	5
#define WL_AF_TX_MIN_RETRY	3

#define WL_AF_SEARCH_TIME_MAX		450
#define WL_AF_TX_EXTRA_TIME_MAX		200

#define WL_SCAN_TIMER_INTERVAL_MS	10000 /* Scan timeout */
#ifdef WL_NAN
#define WL_SCAN_TIMER_INTERVAL_MS_NAN	15000 /* Scan timeout */
#endif /* WL_NAN */
#ifdef WL_6G_BAND
/* additional scan timeout for 6GHz, 6000msec */
#define WL_SCAN_TIMER_INTERVAL_MS_6G	6000
#endif /* WL_6G_BAND */
#define CHSPEC_IS_6G_PSC(chspec) (CHSPEC_IS6G(chspec) && ((CHSPEC_CHANNEL(chspec) % 16) == 5))
#define WL_CHANNEL_SYNC_RETRY	5
#define WL_INVALID		-1

#ifdef DHD_LOSSLESS_ROAMING
#define WL_ROAM_TIMEOUT_MS	1000 /* Roam timeout */
#endif
/* Bring down SCB Timeout to 20secs from 60secs default */
#ifndef WL_SCB_TIMEOUT
#define WL_SCB_TIMEOUT	20
#endif

#if defined(ROAM_ENABLE) || defined(ROAM_CHANNEL_CACHE)
#define  ESCAN_CHANNEL_CACHE
#endif

#ifndef WL_SCB_ACTIVITY_TIME
#define WL_SCB_ACTIVITY_TIME	5
#endif

#ifndef WL_SCB_MAX_PROBE
#define WL_SCB_MAX_PROBE	3
#endif

#ifndef WL_PSPRETEND_RETRY_LIMIT
#define WL_PSPRETEND_RETRY_LIMIT 1
#endif

#ifndef WL_MIN_PSPRETEND_THRESHOLD
#define WL_MIN_PSPRETEND_THRESHOLD	2
#endif

/* Cipher suites */
#ifndef WLAN_CIPHER_SUITE_PMK
#define WLAN_CIPHER_SUITE_PMK			0x00904C00
#endif /* WLAN_CIPHER_SUITE_PMK */

#ifndef WLAN_AKM_SUITE_FT_8021X
#define WLAN_AKM_SUITE_FT_8021X			0x000FAC03
#endif /* WLAN_AKM_SUITE_FT_8021X */

#ifndef WLAN_AKM_SUITE_FT_PSK
#define WLAN_AKM_SUITE_FT_PSK			0x000FAC04
#endif /* WLAN_AKM_SUITE_FT_PSK */

#ifndef WLAN_AKM_SUITE_8021X_SUITE_B
#define WLAN_AKM_SUITE_8021X_SUITE_B		0x000FAC0B
#define WLAN_AKM_SUITE_8021X_SUITE_B_192	0x000FAC0C
#endif /* WLAN_AKM_SUITE_8021X_SUITE_B */

/* TODO: even in upstream linux(v5.0), FT-1X-SHA384 isn't defined and supported yet.
 * need to revisit here to sync correct name later.
 */
#ifndef WLAN_AKM_SUITE_FT_8021X_SHA384
#define WLAN_AKM_SUITE_FT_8021X_SHA384		0x000FAC0D
#endif /* WLAN_AKM_SUITE_FT_8021X_SHA384 */

#define WL_AKM_SUITE_SHA256_1X  0x000FAC05
#define WL_AKM_SUITE_SHA256_PSK 0x000FAC06

#define WLAN_AKM_SUITE_SAE_SHA256		0x000FAC08

#ifndef WLAN_AKM_SUITE_FILS_SHA256
#define WLAN_AKM_SUITE_FILS_SHA256		0x000FAC0E
#define WLAN_AKM_SUITE_FILS_SHA384		0x000FAC0F
#define WLAN_AKM_SUITE_FT_FILS_SHA256		0x000FAC10
#define WLAN_AKM_SUITE_FT_FILS_SHA384		0x000FAC11
#endif /* WLAN_AKM_SUITE_FILS_SHA256 */

#define MIN_VENDOR_EXTN_IE_LEN		2
#ifdef WL_OWE
#ifndef WLAN_AKM_SUITE_OWE
#define WLAN_AKM_SUITE_OWE                0X000FAC12
#endif /* WPA_KEY_MGMT_OWE */
#endif /* WL_OWE */
#define WLAN_AKM_SUITE_DPP                0X506F9A02

/*
 * BRCM local.
 * Use a high number that's unlikely to clash with linux upstream for a while until we can
 * submit these changes to the community.
*/
#define NL80211_FEATURE_FW_4WAY_HANDSHAKE (1<<31)

/* SCAN_SUPPRESS timer values in ms */
#define WL_SCAN_SUPPRESS_TIMEOUT 31000 /* default Framwork DHCP timeout is 30 sec */
#define WL_SCAN_SUPPRESS_RETRY 3000

#define WL_PM_ENABLE_TIMEOUT 10000

/* cfg80211 wowlan definitions */
#define WL_WOWLAN_MAX_PATTERNS			8
#define WL_WOWLAN_MIN_PATTERN_LEN		1
#define WL_WOWLAN_MAX_PATTERN_LEN		255
#define WL_WOWLAN_PKT_FILTER_ID_FIRST	201
#define WL_WOWLAN_PKT_FILTER_ID_LAST	(WL_WOWLAN_PKT_FILTER_ID_FIRST + \
									WL_WOWLAN_MAX_PATTERNS - 1)
#ifdef WLAIBSS
#define IBSS_COALESCE_DEFAULT 0
#define IBSS_INITIAL_SCAN_ALLOWED_DEFAULT 0
#else	/* WLAIBSS */
#define IBSS_COALESCE_DEFAULT 1
#define IBSS_INITIAL_SCAN_ALLOWED_DEFAULT 1
#endif	/* WLAIBSS */

#ifdef WLTDLS
#define TDLS_TUNNELED_PRB_REQ	"\x7f\x50\x6f\x9a\04"
#define TDLS_TUNNELED_PRB_RESP	"\x7f\x50\x6f\x9a\05"
#define TDLS_MAX_IFACE_FOR_ENABLE 1
#endif /* WLTDLS */

#ifdef WLAIBSS
/* Custom AIBSS beacon parameters */
#define AIBSS_INITIAL_MIN_BCN_DUR	500
#define AIBSS_MIN_BCN_DUR		5000
#define AIBSS_BCN_FLOOD_DUR		5000
#define AIBSS_PEER_FREE			3
#endif /* WLAIBSS */

#ifndef FILS_INDICATION_IE_TAG_FIXED_LEN
#define FILS_INDICATION_IE_TAG_FIXED_LEN		2
#endif

#if defined(STRICT_GCC_WARNINGS) && defined(__GNUC__) &&\
(__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6))
#define BCM_SET_LIST_FIRST_ENTRY(entry, ptr, type, member) \
GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST(); \
(entry) = list_first_entry((ptr), type, member); \
GCC_DIAGNOSTIC_POP(); \

#define BCM_SET_CONTAINER_OF(entry, ptr, type, member) \
GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST(); \
entry = container_of((ptr), type, member); \
GCC_DIAGNOSTIC_POP(); \

#else
#define BCM_SET_LIST_FIRST_ENTRY(entry, ptr, type, member) \
(entry) = list_first_entry((ptr), type, member); \

#define BCM_SET_CONTAINER_OF(entry, ptr, type, member) \
entry = container_of((ptr), type, member); \

#endif /* STRICT_GCC_WARNINGS */

/* DPP Public Action Frame types */
enum wl_dpp_ftype {
    DPP_AUTH_REQ = 0,
    DPP_AUTH_RESP = 1,
    DPP_AUTH_CONF = 2,
    DPP_PEER_DISC_REQ = 5,
    DPP_PEER_DISC_RESP = 6,
    DPP_PKEX_EX_REQ = 7,
    DPP_PKEX_EX_RESP = 8,
    DPP_PKEX_COMMIT_REVEAL_REQ = 9,
    DPP_PKEX_COMMIT_REVEAL_RESP = 10,
    DPP_CONFIGURATION_RESULT = 11
};

/* DPP Public Action Frame */
struct wl_dpp_pub_act_frame {
    uint8   category;        /* PUB_AF_CATEGORY */
    uint8   action;          /* PUB_AF_ACTION */
    uint8   oui[3];          /* OUI */
    uint8   oui_type;        /* OUI type */
    uint8   crypto_suite;    /* OUI subtype */
    uint8   ftype;           /* nonzero, identifies req/rsp transaction */
    uint8   elts[1];         /* Variable length information elements. */
} __attribute__ ((packed));
typedef struct wl_dpp_pub_act_frame wl_dpp_pa_frame_t;

#define WL_PUB_AF_CATEGORY        0x04
#define WL_PUB_AF_ACTION          0x09   /* Vendor specific */
#define WL_PUB_AF_WFA_STYPE_DPP   0x1A   /* WFA Subtype DPP */
#define WL_PUB_AF_STYPE_INVALID	  255
#define WL_GAS_MIN_LEN            8
#define WL_GAS_WFA_OFFSET         3
#define WL_GAS_RESP_OFFSET        4
#define WL_GAS_STYPE_OFFSET       6
#define WL_GAS_WFA_STYPE_DPP      0x1A
#define WL_GAS_DPP_ADV_ID         0x7ddd

/* Action value for GAS Initial Request AF */
#define WL_PUB_AF_GAS_IREQ    0x0a
/* Action value for GAS Initial Response AF */
#define WL_PUB_AF_GAS_IRESP   0x0b
/* Action value for GAS Comeback Request AF */
#define WL_PUB_AF_GAS_CREQ    0x0c
/* Action value for GAS Comeback Response AF */
#define WL_PUB_AF_GAS_CRESP   0x0d
/* Advertisement Protocol IE ID */
#define WL_PUB_AF_GAS_AD_EID  0x6c

typedef wifi_p2psd_gas_pub_act_frame_t wl_dpp_gas_af_t;

/* driver status */
enum wl_status {
	WL_STATUS_READY = 0,
	WL_STATUS_SCANNING,
	WL_STATUS_SCAN_ABORTING,
	WL_STATUS_CONNECTING,
	WL_STATUS_CONNECTED,
	WL_STATUS_DISCONNECTING,
	WL_STATUS_AP_CREATING,
	WL_STATUS_AP_CREATED,
	/* whole sending action frame procedure:
	 * includes a) 'finding common channel' for public action request frame
	 * and b) 'sending af via 'actframe' iovar'
	 */
	WL_STATUS_SENDING_ACT_FRM,
	/* find a peer to go to a common channel before sending public action req frame */
	WL_STATUS_FINDING_COMMON_CHANNEL,
	/* waiting for next af to sync time of supplicant.
	 * it includes SENDING_ACT_FRM and WAITING_NEXT_ACT_FRM_LISTEN
	 */
	WL_STATUS_WAITING_NEXT_ACT_FRM,
#ifdef WL_CFG80211_SYNC_GON
	/* go to listen state to wait for next af after SENDING_ACT_FRM */
	WL_STATUS_WAITING_NEXT_ACT_FRM_LISTEN,
#endif /* WL_CFG80211_SYNC_GON */
	/* it will be set when upper layer requests listen and succeed in setting listen mode.
	 * if set, other scan request can abort current listen state
	 */
	WL_STATUS_REMAINING_ON_CHANNEL,
#ifdef WL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST
	/* it's fake listen state to keep current scan state.
	 * it will be set when upper layer requests listen but scan is running. then just run
	 * a expire timer without actual listen state.
	 * if set, other scan request does not need to abort scan.
	 */
	WL_STATUS_FAKE_REMAINING_ON_CHANNEL,
#endif /* WL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST */
	WL_STATUS_NESTED_CONNECT,
	WL_STATUS_CFG80211_CONNECT,
	WL_STATUS_AUTHORIZED
};

typedef enum wl_iftype {
	WL_IF_TYPE_STA = 0,
	WL_IF_TYPE_AP = 1,
#ifdef WLMESH_CFG80211
	WL_IF_TYPE_MESH = 2,
#endif /* WLMESH_CFG80211 */

#ifdef WLAWDL
	WL_IF_TYPE_AWDL = 2,
#endif /* WLAWDL */

	WL_IF_TYPE_NAN_NMI = 3,
	WL_IF_TYPE_NAN = 4,
	WL_IF_TYPE_P2P_GO = 5,
	WL_IF_TYPE_P2P_GC = 6,
	WL_IF_TYPE_P2P_DISC = 7,
	WL_IF_TYPE_IBSS = 8,
	WL_IF_TYPE_MONITOR = 9,
	WL_IF_TYPE_AIBSS = 10,
	WL_IF_TYPE_MAX
} wl_iftype_t;

typedef enum wl_interface_state {
	WL_IF_CREATE_REQ,
	WL_IF_CREATE_DONE,
	WL_IF_DELETE_REQ,
	WL_IF_DELETE_DONE,
	WL_IF_CHANGE_REQ,
	WL_IF_CHANGE_DONE,
	WL_IF_STATE_MAX,	/* Retain as last one */
} wl_interface_state_t;

/* wi-fi mode */
enum wl_mode {
	WL_MODE_BSS = 0,
	WL_MODE_IBSS = 1,
	WL_MODE_AP = 2,

#ifdef WLAWDL
	WL_MODE_AWDL = 3,
#endif /* WLAWDL */

	WL_MODE_NAN = 4,
#ifdef WLMESH_CFG80211
	WL_MODE_MESH = 5,
#endif /* WLMESH_CFG80211 */
	WL_MODE_MAX
};

/* driver profile list */
enum wl_prof_list {
	WL_PROF_MODE,
	WL_PROF_SSID,
	WL_PROF_SEC,
	WL_PROF_IBSS,
	WL_PROF_BAND,
	WL_PROF_CHAN,
	WL_PROF_BSSID,
	WL_PROF_ACT,
	WL_PROF_BEACONINT,
	WL_PROF_DTIMPERIOD,
	WL_PROF_LATEST_BSSID
};

/* donlge escan state */
enum wl_escan_state {
	WL_ESCAN_STATE_IDLE,
	WL_ESCAN_STATE_SCANING
};
/* fw downloading status */
enum wl_fw_status {
	WL_FW_LOADING_DONE,
	WL_NVRAM_LOADING_DONE
};

enum wl_management_type {
	WL_BEACON = 0x1,
	WL_PROBE_RESP = 0x2,
	WL_ASSOC_RESP = 0x4
};

enum wl_pm_workq_act_type {
	WL_PM_WORKQ_SHORT,
	WL_PM_WORKQ_LONG,
	WL_PM_WORKQ_DEL
};

enum wl_tdls_config {
    TDLS_STATE_AP_CREATE,
    TDLS_STATE_AP_DELETE,
    TDLS_STATE_CONNECT,
    TDLS_STATE_DISCONNECT,
    TDLS_STATE_SETUP,
    TDLS_STATE_TEARDOWN,
    TDLS_STATE_IF_CREATE,
    TDLS_STATE_IF_DELETE,
    TDLS_STATE_NMI_CREATE
};

typedef enum wl_assoc_state {
	WL_STATE_ASSOC_IDLE,
	WL_STATE_ASSOCIATING,
	WL_STATE_ASSOCIATED
} wl_assoc_state_t;

typedef enum wl_link_action {
	WL_LINK_NONE,
	WL_LINK_ASSOC_FAIL,
	WL_LINK_ASSOC_DONE,
	WL_LINK_DOWN,
	WL_LINK_ROAM_DONE,
	WL_LINK_FORCE_DEAUTH
} wl_link_action_t;

typedef struct wl_assoc_status {
	u16 flags;
	u16 assoc_state;
	u32 event_type;
	u32 status;
	u32 reason;
	wl_link_action_t link_action;
	u8 curbssid[ETH_ALEN];
	u8 addr[ETH_ALEN];
	u16 data_len;
	void *data;
	struct net_device *ndev;
	const wl_event_msg_t *event_msg;
} wl_assoc_status_t;

/* beacon / probe_response */
struct beacon_proberesp {
	__le64 timestamp;
	__le16 beacon_int;
	__le16 capab_info;
	u8 variable[0];
} __attribute__ ((packed));

/* driver configuration */
struct wl_conf {
	u32 frag_threshold;
	u32 rts_threshold;
	u32 retry_short;
	u32 retry_long;
	s32 tx_power;
	struct ieee80211_channel channel;
};

typedef s32(*EVENT_HANDLER) (struct bcm_cfg80211 *cfg, bcm_struct_cfgdev *cfgdev,
                            const wl_event_msg_t *e, void *data);

/* bss inform structure for cfg80211 interface */
struct wl_cfg80211_bss_info {
	u16 band;
	u16 channel;
	s16 rssi;
	u16 frame_len;
	u8 frame_buf[1];
};

/* basic structure of scan request */
struct wl_scan_req {
	struct wlc_ssid ssid;
};

/* basic structure of information element */
struct wl_ie {
	u16 offset;
	u8 buf[WL_TLV_INFO_MAX];
};

/* event queue for cfg80211 main event */
struct wl_event_q {
	struct list_head eq_list;
	u32 etype;
	u32 id;			/* counter to track events */
	wl_event_msg_t emsg;
	u32 datalen;
	s8 edata[1];
};

/* security information with currently associated ap */
struct wl_security {
	u32 wpa_versions;
	u32 auth_type;
	u32 cipher_pairwise;
	u32 cipher_group;
	u32 wpa_auth;
	u32 auth_assoc_res_status;
	u32 fw_wpa_auth;
	u32 fw_auth;
	u32 fw_wsec;
	u32 fw_mfp;
};

/* ibss information for currently joined ibss network */
struct wl_ibss {
	u8 beacon_interval;	/* in millisecond */
	u8 atim;		/* in millisecond */
	s8 join_only;
	u8 band;
	u8 channel;
};

typedef struct wl_bss_vndr_ies {
	u8  probe_req_ie[VNDR_IES_BUF_LEN];
	u8  probe_res_ie[VNDR_IES_MAX_BUF_LEN];
	u8  assoc_req_ie[VNDR_IES_BUF_LEN];
	u8  assoc_res_ie[VNDR_IES_BUF_LEN];
	u8  beacon_ie[VNDR_IES_MAX_BUF_LEN];
	u8  disassoc_ie[VNDR_IES_BUF_LEN];
	u32 probe_req_ie_len;
	u32 probe_res_ie_len;
	u32 assoc_req_ie_len;
	u32 assoc_res_ie_len;
	u32 beacon_ie_len;
	u32 disassoc_ie_len;
} wl_bss_vndr_ies_t;

typedef struct wl_cfgbss {
	u8 *wpa_ie;
	u8 *rsn_ie;
	u8 *wps_ie;
	u8 *fils_ind_ie;
	bool security_mode;
	struct wl_bss_vndr_ies ies;	/* Common for STA, P2P GC, GO, AP, P2P Disc Interface */
} wl_cfgbss_t;

/* cfg driver profile */
struct wl_profile {
	u32 mode;
	s32 band;
	u32 channel;
	struct wlc_ssid ssid;
	struct wl_security sec;
	struct wl_ibss ibss;
	u8 bssid[ETHER_ADDR_LEN];
	u16 beacon_interval;
	u8 dtim_period;
	bool active;
	u8 latest_bssid[ETHER_ADDR_LEN];
};

struct wl_wps_ie {
	uint8	id;		/* IE ID: 0xDD */
	uint8	len;		/* IE length */
	uint8	OUI[3];		/* WiFi WPS specific OUI */
	uint8	oui_type;	/*  Vendor specific OUI Type */
	uint8	attrib[1];	/* variable length attributes */
} __attribute__ ((packed));
typedef struct wl_wps_ie wl_wps_ie_t;

struct wl_eap_msg {
	uint16 attrib;
	uint16 len;
	uint8 type;
} __attribute__ ((packed));
typedef struct wl_eap_msg wl_eap_msg_t;

struct wl_eap_exp {
	uint8 OUI[3];
	uint32 oui_type;
	uint8 opcode;
	u8 flags;
	u8 data[1];
} __attribute__ ((packed));
typedef struct wl_eap_exp wl_eap_exp_t;

struct net_info {
	struct net_device *ndev;
	struct wireless_dev *wdev;
	struct wl_profile profile;
	wl_iftype_t iftype;
	s32 roam_off;
	unsigned long sme_state;
	bool pm_restore;
	bool pm_block;
	s32 pm;
	s32 bssidx;
	wl_cfgbss_t bss;
	u8 ifidx;
	struct list_head list; /* list of all net_info structure */
};

#ifdef WL_BCNRECV
/* PERIODIC Beacon receive for detecting FakeAPs */
typedef struct wl_bcnrecv_result {
	uint8      SSID[DOT11_MAX_SSID_LEN];    /**< SSID String */
	struct ether_addr BSSID;                /**< Network BSSID */
	uint8      channel;                     /**< Channel */
	uint16     beacon_interval;
	uint32	   timestamp[2];		/**< Beacon Timestamp */
	uint64     system_time;
} wl_bcnrecv_result_t;

typedef struct wl_bcnrecv_info {
	uint bcnrecv_state;		/* TO know the fakeap state */
} wl_bcnrecv_info_t;

typedef enum wl_bcnrecv_state {
	BEACON_RECV_IDLE = 0,
	BEACON_RECV_STARTED,
	BEACON_RECV_STOPPED,
	BEACON_RECV_SUSPENDED
} wl_bcnrecv_state_t;

typedef enum wl_bcnrecv_reason {
	WL_BCNRECV_INVALID = 0,
	WL_BCNRECV_USER_TRIGGER,
	WL_BCNRECV_SUSPEND,
	WL_BCNRECV_SCANBUSY,
	WL_BCNRECV_CONCURRENCY,
	WL_BCNRECV_LISTENBUSY,
	WL_BCNRECV_ROAMABORT,
	WL_BCNRECV_HANG
} wl_bcnrecv_reason_t;

typedef enum wl_bcnrecv_status {
	WL_BCNRECV_STARTED = 0,
	WL_BCNRECV_STOPPED,
	WL_BCNRECV_ABORTED,
	WL_BCNRECV_SUSPENDED,
	WL_BCNRECV_MAX
} wl_bcnrecv_status_t;

typedef enum wl_bcnrecv_attr_type {
	BCNRECV_ATTR_STATUS = 1,
	BCNRECV_ATTR_REASON,
	BCNRECV_ATTR_BCNINFO
} wl_bcnrecv_attr_type_t;
#endif /* WL_BCNRECV */
#ifdef WL_CHAN_UTIL
#define CU_ATTR_PERCENTAGE 1
#define CU_ATTR_HDR_LEN 30
#endif /* WL_CHAN_UTIL */

/* association inform */
#define MAX_REQ_LINE 1024u
struct wl_connect_info {
	u8 req_ie[MAX_REQ_LINE];
	u32 req_ie_len;
	u8 resp_ie[MAX_REQ_LINE];
	u32 resp_ie_len;
};
#define WL_MAX_FILS_KEY_LEN 64

struct wl_fils_info {
	u8 fils_kek[WL_MAX_FILS_KEY_LEN];
	u32 fils_kek_len;
	u8 fils_pmk[WL_MAX_FILS_KEY_LEN];
	u32 fils_pmk_len;
	u8 fils_pmkid[WL_MAX_FILS_KEY_LEN];
	u16 fils_erp_next_seq_num;
	bool fils_roam_disabled;
	u32 fils_bcn_timeout_cache;
};

/* firmware /nvram downloading controller */
struct wl_fw_ctrl {
	const struct firmware *fw_entry;
	unsigned long status;
	u32 ptr;
	s8 fw_name[WL_FILE_NAME_MAX];
	s8 nvram_name[WL_FILE_NAME_MAX];
};

/* assoc ie length */
struct wl_assoc_ielen {
	u32 req_len;
	u32 resp_len;
};

#define WL_EXTJOIN_VERSION_V1	1
/* MIN branch version supporting join iovar versioning */
#define MIN_JOINEXT_V1_FW_MAJOR 17u
/* Branch/es supporting join iovar versioning prior to
 * MIN_JOINEXT_V1_FW_MAJOR
 */
#define MIN_JOINEXT_V1_BR2_FW_MAJOR 16u
#define MIN_JOINEXT_V1_BR2_FW_MINOR 1u

#define MIN_JOINEXT_V1_BR1_FW_MAJOR 14u
#define MIN_JOINEXT_V1_BR1_FW_MINOR 2u

#define PMKDB_WLC_VER 14
#define MIN_PMKID_LIST_V3_FW_MAJOR 13
#define MIN_PMKID_LIST_V3_FW_MINOR 0

#define MIN_PMKID_LIST_V2_FW_MAJOR 12
#define MIN_PMKID_LIST_V2_FW_MINOR 0

/* wpa2 pmk list */
struct wl_pmk_list {
	pmkid_list_v3_t pmkids;
	pmkid_v3_t foo[MAXPMKID];
};

#define KEY_PERM_PMK 0xFFFFFFFF

#ifdef DHD_MAX_IFS
#define WL_MAX_IFS DHD_MAX_IFS
#else
#define WL_MAX_IFS 16
#endif

#define MAC_RAND_BYTES	3
#define ESCAN_BUF_SIZE (64 * 1024)

struct escan_info {
	u32 escan_state;
#ifdef STATIC_WL_PRIV_STRUCT
#ifndef CONFIG_DHD_USE_STATIC_BUF
#error STATIC_WL_PRIV_STRUCT should be used with CONFIG_DHD_USE_STATIC_BUF
#endif /* CONFIG_DHD_USE_STATIC_BUF */
#ifdef DUAL_ESCAN_RESULT_BUFFER
	u8 *escan_buf[2];
#else
	u8 *escan_buf;
#endif /* DUAL_ESCAN_RESULT_BUFFER */
#else
#ifdef DUAL_ESCAN_RESULT_BUFFER
	u8 escan_buf[2][ESCAN_BUF_SIZE];
#else
	u8 escan_buf[ESCAN_BUF_SIZE];
#endif /* DUAL_ESCAN_RESULT_BUFFER */
#endif /* STATIC_WL_PRIV_STRUCT */
#ifdef DUAL_ESCAN_RESULT_BUFFER
	u8 cur_sync_id;
	u8 escan_type[2];
#endif /* DUAL_ESCAN_RESULT_BUFFER */
	struct wiphy *wiphy;
	struct net_device *ndev;
#ifdef DHD_SEND_HANG_ESCAN_SYNCID_MISMATCH
	bool prev_escan_aborted;
#endif /* DHD_SEND_HANG_ESCAN_SYNCID_MISMATCH */
};

#ifdef ESCAN_BUF_OVERFLOW_MGMT
#define BUF_OVERFLOW_MGMT_COUNT 3
typedef struct {
	int RSSI;
	int length;
	struct ether_addr BSSID;
} removal_element_t;
#endif /* ESCAN_BUF_OVERFLOW_MGMT */

struct afx_hdl {
	wl_af_params_t *pending_tx_act_frm;
	struct ether_addr	tx_dst_addr;
	struct net_device *dev;
	struct work_struct work;
	s32 bssidx;
	u32 retry;
	s32 peer_chan;
	s32 peer_listen_chan; /* search channel: configured by upper layer */
	s32 my_listen_chan;	/* listen chanel: extract it from prb req or gon req */
	bool is_listen;
	bool ack_recv;
	bool is_active;
};

struct parsed_ies {
	const wpa_ie_fixed_t *wps_ie;
	u32 wps_ie_len;
	const wpa_ie_fixed_t *wpa_ie;
	u32 wpa_ie_len;
	const bcm_tlv_t *wpa2_ie;
	u32 wpa2_ie_len;
	const bcm_tlv_t *fils_ind_ie;
	u32 fils_ind_ie_len;
};

#ifdef WL_SDO
/* Service discovery */
typedef struct {
	uint8	transaction_id; /* Transaction ID */
	uint8   protocol;       /* Service protocol type */
	uint16  query_len;      /* Length of query */
	uint16  response_len;   /* Length of response */
	uint8   qrbuf[1];
} wl_sd_qr_t;

typedef struct {
	uint16	period;                 /* extended listen period */
	uint16	interval;               /* extended listen interval */
} wl_sd_listen_t;

#define WL_SD_STATE_IDLE 0x0000
#define WL_SD_SEARCH_SVC 0x0001
#define WL_SD_ADV_SVC    0x0002

enum wl_dd_state {
	WL_DD_STATE_IDLE,
	WL_DD_STATE_SEARCH,
	WL_DD_STATE_LISTEN
};

#define MAX_SDO_PROTO_STR_LEN 20
typedef struct wl_sdo_proto {
	char str[MAX_SDO_PROTO_STR_LEN];
	u32 val;
} wl_sdo_proto_t;

typedef struct sd_offload {
	u32 sd_state;
	enum wl_dd_state dd_state;
	wl_sd_listen_t sd_listen;
} sd_offload_t;

typedef struct sdo_event {
	u8 addr[ETH_ALEN];
	uint16	freq;        /* channel Freq */
	uint8	count;       /* Tlv count  */
	uint16	update_ind;
} sdo_event_t;
#endif /* WL_SDO */

#ifdef P2P_LISTEN_OFFLOADING
typedef struct {
	uint16	period;                 /* listen offload period */
	uint16	interval;               /* listen offload interval */
	uint16	count;			/* listen offload count */
	uint16	pad;                    /* pad for 32bit align */
} wl_p2plo_listen_t;
#endif /* P2P_LISTEN_OFFLOADING */

#ifdef WL11U
/* Max length of Interworking element */
#define IW_IES_MAX_BUF_LEN 		8
#endif
#ifdef WLFBT
#define FBT_KEYLEN		32
#endif
#define MAX_EVENT_BUF_NUM 16
typedef struct wl_eventmsg_buf {
	u16 num;
	struct {
		u16 type;
		bool set;
	} event [MAX_EVENT_BUF_NUM];
} wl_eventmsg_buf_t;

typedef struct wl_if_event_info {
	bool valid;
	int ifidx;
	int bssidx;
	uint8 mac[ETHER_ADDR_LEN];
	char name[IFNAMSIZ+1];
	uint8 role;
} wl_if_event_info;

#ifdef SUPPORT_AP_RADIO_PWRSAVE
typedef struct ap_rps_info {
	bool enable;
	int sta_assoc_check;
	int pps;
	int quiet_time;
	int level;
} ap_rps_info_t;
#endif /* SUPPORT_AP_RADIO_PWRSAVE */

#ifdef SUPPORT_RSSI_SUM_REPORT
#define RSSILOG_FLAG_FEATURE_SW		0x1
#define RSSILOG_FLAG_REPORT_READY	0x2
typedef struct rssilog_set_param {
	uint8 enable;
	uint8 rssi_threshold;
	uint8 time_threshold;
	uint8 pad;
} rssilog_set_param_t;

typedef struct rssilog_get_param {
	uint8 report_count;
	uint8 enable;
	uint8 rssi_threshold;
	uint8 time_threshold;
} rssilog_get_param_t;

typedef struct rssi_ant_param {
	struct ether_addr ea;
	chanspec_t chanspec;
} rssi_ant_param_t;

typedef struct wl_rssi_ant_mimo {
	uint32 version;
	uint32 count;
	int8 rssi_ant[WL_RSSI_ANT_MAX];
	int8 rssi_sum;
	int8 PAD[3];
} wl_rssi_ant_mimo_t;
#endif /* SUPPORT_RSSI_SUM_REPORT */

/* MBO-OCE prune event reason codes */
#if defined(WL_MBO) || defined(WL_OCE)
typedef enum wl_prune_evt_reason {
	WIFI_PRUNE_UNSPECIFIED = 0,		/* Unspecified event reason code */
	WIFI_PRUNE_ASSOC_RETRY_DELAY = 1,	/* MBO assoc retry delay */
	WIFI_PRUNE_RSSI_ASSOC_REJ = 2		/* OCE RSSI-based assoc rejection */
} wl_prune_evt_reason_t;
#endif /* WL_MBO || WL_OCE */

#if defined(DHD_ENABLE_BIGDATA_LOGGING)
#define GET_BSS_INFO_LEN 90
#endif /* DHD_ENABLE_BIGDATA_LOGGING */

#ifdef WL_MBO
typedef struct wl_event_mbo wl_event_mbo_t;
typedef struct wl_event_mbo_cell_nw_switch wl_event_mbo_cell_nw_switch_t;
typedef struct wl_btm_event_type_data wl_btm_event_type_data_t;
#endif /* WL_MBO */

#if defined(WL_MBO) || defined(WL_OCE)
typedef struct wl_bssid_prune_evt_info wl_bssid_pruned_evt_info_t;
#endif /* WL_MBO || WL_OCE */

#define WL_CCODE_LEN 2

#ifdef WL_NAN
#ifdef WL_NANP2P
#define WL_CFG_P2P_DISC_BIT 0x1u
#define WL_CFG_NAN_DISC_BIT 0x2u
#define WL_NANP2P_CONC_SUPPORT	(WL_CFG_P2P_DISC_BIT | WL_CFG_NAN_DISC_BIT)
#endif /* WL_NAN2P */
#endif /* WL_NAN */

#ifdef WL_IFACE_MGMT
#define WL_IFACE_NOT_PRESENT -1

typedef enum iface_conc_policy {
	WL_IF_POLICY_DEFAULT		= 0,
	WL_IF_POLICY_FCFS		= 1,
	WL_IF_POLICY_LP			= 2,
	WL_IF_POLICY_ROLE_PRIORITY	= 3,
	WL_IF_POLICY_CUSTOM		= 4,
	WL_IF_POLICY_INVALID
} iface_conc_policy_t;

typedef struct iface_mgmt_data {
	uint8 policy;
	uint8 priority[WL_IF_TYPE_MAX];
} iface_mgmt_data_t;
#endif /* WL_IFACE_MGMT */

#ifdef WL_WPS_SYNC
#define EAP_PACKET              0
#define EAP_EXPANDED_TYPE       254
#define EAP_EXP_OPCODE_OFFSET   7
#define EAP_EXP_FRAGMENT_LEN_OFFSET	2
#define EAP_EXP_FLAGS_FRAGMENTED_DATA	2
#define EAP_EXP_FLAGS_MORE_DATA	1
#define EAPOL_EAP_HDR_LEN       5
#define EAP_EXP_HDR_MIN_LENGTH	(EAPOL_EAP_HDR_LEN + EAP_EXP_OPCODE_OFFSET)
#define EAP_ATTRIB_MSGTYPE  0x1022
#define EAP_WSC_UPNP        0
#define EAP_WSC_START       1
#define EAP_WSC_ACK         2
#define EAP_WSC_NACK        3
#define EAP_WSC_MSG         4
#define EAP_WSC_DONE        5
#define EAP_WSC_MSG_M8      12
#define EAP_CODE_FAILURE    4
#define WL_WPS_REAUTH_TIMEOUT	10000

struct wl_eap_header {
	unsigned char code; /* EAP code */
	unsigned char id;   /* Current request ID */
	unsigned short length;  /* Length including header */
	unsigned char type; /* EAP type (optional) */
	unsigned char data[1];  /* Type data (optional) */
} __attribute__ ((packed));
typedef struct wl_eap_header wl_eap_header_t;

typedef enum wl_wps_state {
	WPS_STATE_IDLE = 0,
	WPS_STATE_STARTED,
	WPS_STATE_M8_SENT,
	WPS_STATE_M8_RECVD,
	WPS_STATE_EAP_FAIL,
	WPS_STATE_REAUTH_WAIT,
	WPS_STATE_LINKUP,
	WPS_STATE_LINKDOWN,
	WPS_STATE_DISCONNECT,
	WPS_STATE_DISCONNECT_CLIENT,
	WPS_STATE_CONNECT_FAIL,
	WPS_STATE_AUTHORIZE,
	WPS_STATE_DONE,
	WPS_STATE_INVALID
} wl_wps_state_t;

#define WPS_MAX_SESSIONS	2
typedef struct wl_wps_session {
	bool in_use;
	timer_list_compat_t timer;
	struct net_device *ndev;
	wl_wps_state_t state;
	u16 mode;
	u8 peer_mac[ETHER_ADDR_LEN];
} wl_wps_session_t;
#endif /* WL_WPS_SYNC */

#ifndef WL_STATIC_IFNAME_PREFIX
#define WL_STATIC_IFNAME_PREFIX "wlan%d"
#endif /* WL_STATIC_IFNAME */

typedef struct buf_data {
	u32 ver; /* version of struct */
	u32 len; /* Total len */
	/* size of each buffer in case of split buffers (0 - single buffer). */
	u32 buf_threshold;
	const void *data_buf[1]; /* array of user space buffer pointers. */
} buf_data_t;

typedef struct wl_loc_info {
	bool in_progress;             /* for tracking listen in progress        */
	struct delayed_work work;     /* for taking care of listen timeout      */
	struct wireless_dev *wdev;    /* interface on which listen is requested */
} wl_loc_info_t;

typedef enum wl_sar_modes {
	HEAD_SAR_BACKOFF_DISABLE = -1,
	HEAD_SAR_BACKOFF_ENABLE = 0,
	GRIP_SAR_BACKOFF_DISABLE,
	GRIP_SAR_BACKOFF_ENABLE,
	NR_mmWave_SAR_BACKOFF_DISABLE,
	NR_mmWave_SAR_BACKOFF_ENABLE,
	NR_Sub6_SAR_BACKOFF_DISABLE,
	NR_Sub6_SAR_BACKOFF_ENABLE,
	SAR_BACKOFF_DISABLE_ALL
} wl_sar_modes_t;

/* Pre selected Power scenarios to be applied from BDF file */
typedef enum {
	WIFI_POWER_SCENARIO_INVALID = -2,
	WIFI_POWER_SCENARIO_DEFAULT = -1,
	WIFI_POWER_SCENARIO_VOICE_CALL = 0,
	WIFI_POWER_SCENARIO_ON_HEAD_CELL_OFF = 1,
	WIFI_POWER_SCENARIO_ON_HEAD_CELL_ON = 2,
	WIFI_POWER_SCENARIO_ON_BODY_CELL_OFF = 3,
	WIFI_POWER_SCENARIO_ON_BODY_CELL_ON = 4,
	WIFI_POWER_SCENARIO_ON_BODY_BT = 5
} wifi_power_scenario;

/* Log timestamp */
#define LOG_TS(cfg, ts)	cfg->tsinfo.ts = OSL_LOCALTIME_NS();
#define CLR_TS(cfg, ts)	cfg->tsinfo.ts = 0;
#define GET_TS(cfg, ts)	cfg->tsinfo.ts;
typedef struct wl_ctx_tsinfo {
	uint64 scan_start;
	uint64 scan_enq;             /* scan event enqueue time       */
	uint64 scan_deq;
	uint64 scan_hdlr_cmplt;
	uint64 scan_cmplt;           /* scan event handler completion */
	uint64 conn_start;
	uint64 conn_cmplt;
	uint64 wl_evt_deq;
	uint64 authorize_start;
	uint64 authorize_cmplt;
	uint64 wl_evt_hdlr_entry;
	uint64 wl_evt_hdlr_exit;
} wl_ctx_tsinfo_t;

typedef struct wlcfg_assoc_info {
	bool targeted_join;     /* Unicast bssid. Host selected bssid for join */
	bool reassoc;
	bool bssid_hint;
	u8 bssid[ETH_ALEN];
	u16 ssid_len;
	u8 ssid[DOT11_MAX_SSID_LEN];
	s32 bssidx;
	u32 chan_cnt;
	chanspec_t chanspecs[MAX_ROAM_CHANNEL];
} wlcfg_assoc_info_t;

#define MAX_NUM_OF_ASSOCIATED_DEV       64
struct bcm_assoclist {
	u32 count;
	u8 mac[MAX_NUM_OF_ASSOCIATED_DEV][ETH_ALEN];
};

typedef struct wl_event_idx {
	u32 enqd;
	u32 in_progress;
	u32 event_type;
	u32 min_connect_idx;
} wl_event_idx_t;

/* private data of cfg80211 interface */
struct bcm_cfg80211 {
	struct wireless_dev *wdev;	/* representing cfg cfg80211 device */

	struct wireless_dev *p2p_wdev;	/* representing cfg cfg80211 device for P2P */
	struct net_device *p2p_net;    /* reference to p2p0 interface */

	struct wl_conf *conf;
	struct cfg80211_scan_request *scan_request;	/* scan request object */
	EVENT_HANDLER evt_handler[WLC_E_LAST];
	struct list_head eq_list;	/* used for event queue */
	struct list_head net_list;     /* used for struct net_info */
	spinlock_t net_list_sync;	/* to protect scan status (and others if needed) */
	spinlock_t eq_lock;	/* for event queue synchronization */
	spinlock_t cfgdrv_lock;	/* to protect scan status (and others if needed) */
	struct completion act_frm_scan;
	struct completion iface_disable;
	struct completion wait_next_af;
	struct mutex usr_sync;	/* maily for up/down synchronization */
	struct mutex if_sync;	/* maily for iface op synchronization */
	struct mutex scan_sync;	/* scan sync from different scan contexts  */
	wl_scan_results_t *bss_list;
	wl_scan_results_t *scan_results;

	/* scan request object for internal purpose */
	struct wl_scan_req *scan_req_int;
	/* information element object for internal purpose */
#if defined(STATIC_WL_PRIV_STRUCT)
	struct wl_ie *ie;
#else
	struct wl_ie ie;
#endif

	/* association information container */
#if defined(STATIC_WL_PRIV_STRUCT)
	struct wl_connect_info *conn_info;
#else
	struct wl_connect_info conn_info;
#endif
	struct wl_pmk_list *pmk_list;	/* wpa2 pmk list */
	tsk_ctl_t event_tsk;  		/* task of main event handler thread */
	dhd_pub_t *pub;
	u32 iface_cnt;
	u32 channel;		/* current channel */
	u32 af_sent_channel;	/* channel action frame is sent */
	/* next af subtype to cancel the remained dwell time in rx process */
	u8 next_af_subtype;
#ifdef WL_CFG80211_SYNC_GON
	ulong af_tx_sent_jiffies;
#endif /* WL_CFG80211_SYNC_GON */
	struct escan_info escan_info;   /* escan information */
	bool active_scan;	/* current scan mode */
	bool ibss_starter;	/* indicates this sta is ibss starter */
	bool link_up;		/* link/connection up flag */

	/* indicate whether chip to support power save mode */
	bool pwr_save;
	bool roam_on;		/* on/off switch for self-roaming */
	bool scan_tried;	/* indicates if first scan attempted */
#if defined(BCMSDIO) || defined(BCMDBUS)
	bool wlfc_on;
#endif
	bool vsdb_mode;
#define WL_ROAM_OFF_ON_CONCURRENT 	0x0001
#define WL_ROAM_REVERT_STATUS		0x0002
	u32 roam_flags;
	u8 *ioctl_buf;		/* ioctl buffer */
	struct mutex ioctl_buf_sync;
	u8 *escan_ioctl_buf;
	u8 *extra_buf;	/* maily to grab assoc information */
	struct dentry *debugfsdir;
	struct rfkill *rfkill;
	bool rf_blocked;
	struct ieee80211_channel remain_on_chan;
	enum nl80211_channel_type remain_on_chan_type;
	u64 send_action_id;
	u64 last_roc_id;
	wait_queue_head_t netif_change_event;
	wl_if_event_info if_event_info;
	struct completion send_af_done;
	struct afx_hdl *afx_hdl;
	struct p2p_info *p2p;
	bool p2p_supported;
	void *btcoex_info;
	timer_list_compat_t scan_timeout;   /* Timer for catch scan event timeout */
#ifdef WL_CFG80211_GON_COLLISION
	u8 block_gon_req_tx_count;
	u8 block_gon_req_rx_count;
#endif /* WL_CFG80211_GON_COLLISION */
#if defined(P2P_IE_MISSING_FIX)
	bool p2p_prb_noti;
#endif
	s32(*state_notifier) (struct bcm_cfg80211 *cfg,
		struct net_info *_net_info, enum wl_status state, bool set);
	unsigned long interrested_state;
	wlc_ssid_t hostapd_ssid;
#ifdef WL_SDO
	sd_offload_t *sdo;
#endif
#ifdef WL11U
	bool wl11u;
#endif /* WL11U */
	bool sched_scan_running;	/* scheduled scan req status */
	struct cfg80211_sched_scan_request *sched_scan_req;	/* scheduled scan req */
#ifdef WL_HOST_BAND_MGMT
	u8 curr_band;
#endif /* WL_HOST_BAND_MGMT */
	bool scan_suppressed;

#ifdef OEM_ANDROID
	timer_list_compat_t scan_supp_timer;
	struct work_struct wlan_work;
#endif /* OEM_ANDROID */

	struct mutex event_sync;	/* maily for up/down synchronization */
	bool disable_roam_event;
	struct delayed_work pm_enable_work;

#ifdef OEM_ANDROID
	struct workqueue_struct *event_workq;   /* workqueue for event */
#endif /* OEM_ANDROID */

#ifndef OEM_ANDROID
	bool event_workq_init;
#endif /* OEM_ANDROID */
	struct work_struct event_work;		/* work item for event */
	struct mutex pm_sync;	/* mainly for pm work synchronization */

	vndr_ie_setbuf_t *ibss_vsie;	/* keep the VSIE for IBSS */
	int ibss_vsie_len;
#ifdef WLAIBSS
	u32 aibss_txfail_pid;
	u32 aibss_txfail_seq;
#endif /* WLAIBSS */
#ifdef WL_RELMCAST
	u32 rmc_event_pid;
	u32 rmc_event_seq;
#endif /* WL_RELMCAST */
#ifdef WLAIBSS_MCHAN
	struct ether_addr ibss_if_addr;
	bcm_struct_cfgdev *ibss_cfgdev; /* For AIBSS */
#endif /* WLAIBSS_MCHAN */
	bool bss_pending_op;		/* indicate where there is a pending IF operation */
#ifdef WLFBT
	uint8 fbt_key[FBT_KEYLEN];
#endif
	int roam_offload;
#ifdef WL_NAN
	wl_nancfg_t *nancfg;
#ifdef WL_NANP2P
	uint8 conc_disc;
	bool nan_p2p_supported;
#endif /* WL_NANP2P */
#endif /* WL_NAN */
#ifdef WL_IFACE_MGMT
	iface_mgmt_data_t iface_data;
#endif /* WL_IFACE_MGMT */
#ifdef P2PLISTEN_AP_SAMECHN
	bool p2p_resp_apchn_status;
#endif /* P2PLISTEN_AP_SAMECHN */
	struct wl_wsec_key wep_key;
#ifdef WLTDLS
	u8 *tdls_mgmt_frame;
	u32 tdls_mgmt_frame_len;
	s32 tdls_mgmt_freq;
#endif /* WLTDLS */
	bool need_wait_afrx;
#ifdef QOS_MAP_SET
	uint8	 *up_table;	/* user priority table, size is UP_TABLE_MAX */
#endif /* QOS_MAP_SET */
	struct ether_addr last_roamed_addr;
	bool rcc_enabled;	/* flag for Roam channel cache feature */
#if defined(DHD_ENABLE_BIGDATA_LOGGING)
	char bss_info[GET_BSS_INFO_LEN];
	wl_event_msg_t event_auth_assoc;
	u32 assoc_reject_status;
	u32 roam_count;
#endif /* DHD_ENABLE_BIGDATA_LOGGING */
	u16 ap_oper_channel;
#if defined(SUPPORT_RANDOM_MAC_SCAN)
	bool random_mac_enabled;
#endif /* SUPPORT_RANDOM_MAC_SCAN */
#ifdef DHD_LOSSLESS_ROAMING
	timer_list_compat_t roam_timeout;   /* Timer for catch roam timeout */
#endif
#ifndef DUAL_ESCAN_RESULT_BUFFER
	uint16 escan_sync_id_cntr;
#endif
#ifdef WLTDLS
	uint8 tdls_supported;
	struct mutex tdls_sync;	/* protect tdls config operations */
#endif /* WLTDLS */
#ifdef MFP
	const uint8 *bip_pos;
	int mfp_mode;
#endif /* MFP */
#ifdef WES_SUPPORT
#ifdef CUSTOMER_SCAN_TIMEOUT_SETTING
	int custom_scan_channel_time;
	int custom_scan_unassoc_time;
	int custom_scan_passive_time;
	int custom_scan_home_time;
	int custom_scan_home_away_time;
#endif /* CUSTOMER_SCAN_TIMEOUT_SETTING */
#endif /* WES_SUPPORT */
	uint8 vif_count;	/* Virtual Interface count */
#ifdef WBTEXT
	struct list_head wbtext_bssid_list;
#endif /* WBTEXT */
#ifdef SUPPORT_AP_RADIO_PWRSAVE
	ap_rps_info_t ap_rps_info;
#endif /* SUPPORT_AP_RADIO_PWRSAVE */
	u16 vif_macaddr_mask;
	osl_t *osh;
	struct list_head vndr_oui_list;
	spinlock_t vndr_oui_sync;	/* to protect vndr_oui_list */
	bool rssi_sum_report;
	int rssi;	/* previous RSSI (backup) of get_station */
#ifdef WL_WPS_SYNC
	wl_wps_session_t wps_session[WPS_MAX_SESSIONS];
	spinlock_t wps_sync;	/* to protect wps states (and others if needed) */
#endif /* WL_WPS_SYNC */
	struct wl_fils_info fils_info;
#ifdef WL_BAM
	wl_bad_ap_mngr_t bad_ap_mngr;
#endif  /* WL_BAM */

#ifdef BIGDATA_SOFTAP
	struct wl_ap_sta_info *ap_sta_info;
#endif /* BIGDATA_SOFTAP */

	uint8 scanmac_enabled;
	bool scanmac_config;
#ifdef WL_BCNRECV
	/* structure used for fake ap detection info */
	struct mutex bcn_sync;  /* mainly for bcn resume/suspend synchronization */
	wl_bcnrecv_info_t bcnrecv_info;
#endif /* WL_BCNRECV */
	struct net_device *static_ndev[DHD_MAX_STATIC_IFS];
	uint8 static_ndev_state[DHD_MAX_STATIC_IFS];
	bool hal_started;
	wl_wlc_version_t wlc_ver;
	bool scan_params_v2;
#ifdef SUPPORT_AP_BWCTRL
	u32 bw_cap_5g;
#endif /* SUPPORT_AP_BWCTRL */
#ifdef WL_6G_BAND
	bool band_6g_supported;
#endif /* WL_6G_BAND */
	wl_loc_info_t loc;    /* listen on channel state info */
	int roamscan_mode;
	int wes_mode;
	int ncho_mode;
	int ncho_band;
#ifdef WL_SAR_TX_POWER
	wifi_power_scenario wifi_tx_power_mode;
#endif /* WL_SAR_TX_POWER */
	struct mutex connect_sync;  /* For assoc/resssoc state sync */
	wl_ctx_tsinfo_t tsinfo;
	struct wl_pmk_list *spmk_info_list;	/* single pmk info list */
	struct bcm_assoclist assoclist;
	chanspec_t acs_chspec;	/* Selected chanspec in case of ACS */
	u32 join_iovar_ver;
	struct delayed_work ap_work;     /* AP linkup timeout handler */
	wl_event_idx_t eidx;	/* event state tracker */
#if defined (WL_SCHED_SCAN) && defined (SCHED_SCAN_DELAYED_WORK)
	struct delayed_work sched_scan_stop_work;
#endif
#ifdef WL_P2P_6G
	bool p2p_6g_enabled;	/* P2P 6G support enabled */
#endif /* WL_P2P_6G */
	u32 halpid;
	u8 country[WLC_CNTRY_BUF_SZ];
#if defined(RSSIAVG)
	wl_rssi_cache_ctrl_t g_rssi_cache_ctrl;
	wl_rssi_cache_ctrl_t g_connected_rssi_cache_ctrl;
#endif
#if defined(BSSCACHE)
	wl_bss_cache_ctrl_t g_bss_cache_ctrl;
#endif
	int autochannel;
	int best_2g_ch;
	int best_5g_ch;
	int best_6g_ch;
};

/* Max auth timeout allowed in case of EAP is 70sec, additional 5 sec for
* inter-layer overheads
*/
#define WL_DS_SKIP_THRESHOLD_USECS  (75000L * 1000L)

enum wl_state_type {
	WL_STATE_IDLE,
	WL_STATE_SCANNING,
	WL_STATE_CONNECTING,
	WL_STATE_LISTEN,
	WL_STATE_AUTHORIZING /* Assocated to authorized */
};

#define WL_STATIC_IFIDX	(DHD_MAX_IFS)
enum static_ndev_states {
	NDEV_STATE_NONE,
	NDEV_STATE_OS_IF_CREATED,
	NDEV_STATE_FW_IF_CREATED,
	NDEV_STATE_FW_IF_FAILED,
	NDEV_STATE_FW_IF_DELETED
};
#ifdef WL_STATIC_IF
bool wl_cfg80211_static_if(struct bcm_cfg80211 *cfg, struct net_device *ndev);
int wl_cfg80211_static_ifidx(struct bcm_cfg80211 *cfg, struct net_device *ndev);
struct net_device *wl_cfg80211_static_if_active(struct bcm_cfg80211 *cfg);
int wl_cfg80211_static_if_name(struct bcm_cfg80211 *cfg, const char *name);
void wl_cfg80211_static_if_dev_close(struct net_device *dev);
#endif /* WL_STATIC_IF */

#ifdef WL_SAE
typedef struct wl_sae_key_info {
	uint8 peer_mac[ETHER_ADDR_LEN];
	uint16 pmk_len;
	uint16 pmkid_len;
	const uint8 *pmk;
	const uint8 *pmkid;
} wl_sae_key_info_t;
#endif /* WL_SAE */

typedef enum wl_concurrency_mode {
	CONCURRENCY_MODE_NONE = 0,
	CONCURRENCY_SCC_MODE,
	CONCURRENCY_VSDB_MODE,
	CONCURRENCY_RSDB_MODE
} wl_concurrency_mode_t;

typedef struct wl_wips_event_info {
	uint32 timestamp;
	struct ether_addr   bssid;
	uint16 misdeauth;
	int16 current_RSSI;
	int16 deauth_RSSI;
} wl_wips_event_info_t;

s32 wl_iftype_to_mode(wl_iftype_t iftype);

#define BCM_LIST_FOR_EACH_ENTRY_SAFE(pos, next, head, member) \
	list_for_each_entry_safe((pos), (next), (head), member)
extern int ioctl_version;

static inline wl_bss_info_t
*next_bss(wl_scan_results_t *list, wl_bss_info_t *bss)
{
	return bss = bss ?
		(wl_bss_info_t *)((uintptr) bss + dtoh32(bss->length)) : list->bss_info;
}

static inline void
wl_probe_wdev_all(struct bcm_cfg80211 *cfg)
{
	struct net_info *_net_info, *next;
	unsigned long int flags;
	int idx = 0;
	WL_CFG_NET_LIST_SYNC_LOCK(&cfg->net_list_sync, flags);
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	BCM_LIST_FOR_EACH_ENTRY_SAFE(_net_info, next,
		&cfg->net_list, list) {
		GCC_DIAGNOSTIC_POP();
		WL_INFORM_MEM(("wl_probe_wdev_all: net_list[%d] bssidx: %d\n",
			idx++, _net_info->bssidx));
	}
	WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
	return;
}

static inline struct net_info *
wl_get_netinfo_by_fw_idx(struct bcm_cfg80211 *cfg, s32 bssidx, u8 ifidx)
{
	struct net_info *_net_info, *next, *info = NULL;
	unsigned long int flags;

	WL_CFG_NET_LIST_SYNC_LOCK(&cfg->net_list_sync, flags);
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	BCM_LIST_FOR_EACH_ENTRY_SAFE(_net_info, next, &cfg->net_list, list) {
		GCC_DIAGNOSTIC_POP();
		if ((bssidx >= 0) && (_net_info->bssidx == bssidx) &&
			(_net_info->ifidx == ifidx)) {
			info = _net_info;
			break;
		}
	}
	WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
	return info;
}

static inline void
wl_dealloc_netinfo_by_wdev(struct bcm_cfg80211 *cfg, struct wireless_dev *wdev)
{
	struct net_info *_net_info, *next;
	unsigned long int flags;

#ifdef DHD_IFDEBUG
	WL_INFORM_MEM(("dealloc_netinfo enter wdev=%p \n", OSL_OBFUSCATE_BUF(wdev)));
#endif
	WL_CFG_NET_LIST_SYNC_LOCK(&cfg->net_list_sync, flags);
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	BCM_LIST_FOR_EACH_ENTRY_SAFE(_net_info, next, &cfg->net_list, list) {
		GCC_DIAGNOSTIC_POP();
		if (wdev && (_net_info->wdev == wdev)) {
			wl_cfgbss_t *bss = &_net_info->bss;

			if (bss->wpa_ie) {
				MFREE(cfg->osh, bss->wpa_ie, bss->wpa_ie[1]
					+ WPA_RSN_IE_TAG_FIXED_LEN);
				bss->wpa_ie = NULL;
			}

			if (bss->rsn_ie) {
				MFREE(cfg->osh, bss->rsn_ie,
					bss->rsn_ie[1] + WPA_RSN_IE_TAG_FIXED_LEN);
				bss->rsn_ie = NULL;
			}

			if (bss->wps_ie) {
				MFREE(cfg->osh, bss->wps_ie, bss->wps_ie[1] + 2);
				bss->wps_ie = NULL;
			}
			list_del(&_net_info->list);
			cfg->iface_cnt--;
			MFREE(cfg->osh, _net_info, sizeof(struct net_info));
		}
	}
	WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
#ifdef DHD_IFDEBUG
	WL_INFORM_MEM(("dealloc_netinfo exit iface_cnt=%d \n", cfg->iface_cnt));
#endif
}

static inline s32
wl_alloc_netinfo(struct bcm_cfg80211 *cfg, struct net_device *ndev,
	struct wireless_dev * wdev, wl_iftype_t iftype, bool pm_block, u8 bssidx, u8 ifidx)
{
	struct net_info *_net_info;
	s32 err = 0;
	unsigned long int flags;
#ifdef DHD_IFDEBUG
	WL_INFORM_MEM(("alloc_netinfo enter bssidx=%d wdev=%p\n",
		bssidx, OSL_OBFUSCATE_BUF(wdev)));
#endif
	/* Check whether there is any duplicate entry for the
	 *  same bssidx && ifidx.
	 */
	if ((_net_info = wl_get_netinfo_by_fw_idx(cfg, bssidx, ifidx))) {
		/* We have a duplicate entry for the same bssidx
		 * already present which shouldn't have been the case.
		 * Attempt recovery.
		 */
		WL_ERR(("Duplicate entry for bssidx=%d ifidx=%d present."
			" Can't add new entry\n", bssidx, ifidx));
		wl_probe_wdev_all(cfg);
#ifdef DHD_DEBUG
		ASSERT(0);
#endif /* DHD_DEBUG */
		return -EINVAL;
	}
	if (cfg->iface_cnt == IFACE_MAX_CNT)
		return -ENOMEM;
	_net_info = (struct net_info *)MALLOCZ(cfg->osh, sizeof(struct net_info));
	if (!_net_info)
		err = -ENOMEM;
	else {
		_net_info->iftype = iftype;
		_net_info->ndev = ndev;
		_net_info->wdev = wdev;
		_net_info->pm_restore = 0;
		_net_info->pm = 0;
		_net_info->pm_block = pm_block;
		_net_info->roam_off = WL_INVALID;
		_net_info->bssidx = bssidx;
		_net_info->ifidx = ifidx;
		WL_CFG_NET_LIST_SYNC_LOCK(&cfg->net_list_sync, flags);
		cfg->iface_cnt++;
		list_add(&_net_info->list, &cfg->net_list);
		WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
	}
#ifdef DHD_IFDEBUG
	WL_DBG(("alloc_netinfo exit iface_cnt=%d \n", cfg->iface_cnt));
#endif
	return err;
}

static inline void
wl_delete_all_netinfo(struct bcm_cfg80211 *cfg)
{
	struct net_info *_net_info, *next;
	unsigned long int flags;

	WL_CFG_NET_LIST_SYNC_LOCK(&cfg->net_list_sync, flags);
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	BCM_LIST_FOR_EACH_ENTRY_SAFE(_net_info, next, &cfg->net_list, list) {
		wl_cfgbss_t *bss = &_net_info->bss;
		GCC_DIAGNOSTIC_POP();

		if (bss->wpa_ie) {
			MFREE(cfg->osh, bss->wpa_ie, bss->wpa_ie[1]
				+ WPA_RSN_IE_TAG_FIXED_LEN);
			bss->wpa_ie = NULL;
		}

		if (bss->rsn_ie) {
			MFREE(cfg->osh, bss->rsn_ie, bss->rsn_ie[1]
				+ WPA_RSN_IE_TAG_FIXED_LEN);
			bss->rsn_ie = NULL;
		}

		if (bss->wps_ie) {
			MFREE(cfg->osh, bss->wps_ie, bss->wps_ie[1] + 2);
			bss->wps_ie = NULL;
		}

		if (bss->fils_ind_ie) {
			MFREE(cfg->osh, bss->fils_ind_ie, bss->fils_ind_ie[1]
				+ FILS_INDICATION_IE_TAG_FIXED_LEN);
			bss->fils_ind_ie = NULL;
		}
		list_del(&_net_info->list);
		if (_net_info->wdev) {
			MFREE(cfg->osh, _net_info->wdev, sizeof(struct wireless_dev));
		}
		MFREE(cfg->osh, _net_info, sizeof(struct net_info));
	}
	cfg->iface_cnt = 0;
	WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
}
static inline u32
wl_get_status_all(struct bcm_cfg80211 *cfg, s32 status)

{
	struct net_info *_net_info, *next;
	u32 cnt = 0;
	unsigned long int flags;

	WL_CFG_NET_LIST_SYNC_LOCK(&cfg->net_list_sync, flags);
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	BCM_LIST_FOR_EACH_ENTRY_SAFE(_net_info, next, &cfg->net_list, list) {
		GCC_DIAGNOSTIC_POP();
		if (_net_info->ndev &&
			test_bit(status, &_net_info->sme_state))
			cnt++;
	}
	WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
	return cnt;
}
static inline void
wl_set_status_all(struct bcm_cfg80211 *cfg, s32 status, u32 op)
{
	struct net_info *_net_info, *next;
	unsigned long int flags;

	WL_CFG_NET_LIST_SYNC_LOCK(&cfg->net_list_sync, flags);
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	BCM_LIST_FOR_EACH_ENTRY_SAFE(_net_info, next, &cfg->net_list, list) {
		GCC_DIAGNOSTIC_POP();
		switch (op) {
			case 1:
				break; /* set all status is not allowed */
			case 2:
				/*
				 * Release the spinlock before calling notifier. Else there
				 * will be nested calls
				 */
				WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
				clear_bit(status, &_net_info->sme_state);
				if (cfg->state_notifier &&
					test_bit(status, &(cfg->interrested_state)))
					cfg->state_notifier(cfg, _net_info, status, false);
				return;
			case 4:
				break; /* change all status is not allowed */
			default:
				break; /* unknown operation */
		}
	}
	WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
}
static inline void
wl_set_status_by_netdev(struct bcm_cfg80211 *cfg, s32 status,
	struct net_device *ndev, u32 op)
{

	struct net_info *_net_info, *next;
	unsigned long int flags;

	if (status >= BITS_PER_LONG) {
		/* max value for shift operation is
		 * (BITS_PER_LONG -1) for unsigned long.
		 * if status crosses BIT_PER_LONG, the variable
		 * sme_state should be correspondingly updated.
		 */
		ASSERT(0);
		return;
	}

	WL_CFG_NET_LIST_SYNC_LOCK(&cfg->net_list_sync, flags);
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	BCM_LIST_FOR_EACH_ENTRY_SAFE(_net_info, next, &cfg->net_list, list) {
		if (ndev && (_net_info->ndev == ndev)) {
			GCC_DIAGNOSTIC_POP();
			switch (op) {
				case 1:
					/*
					 * Release the spinlock before calling notifier. Else there
					 * will be nested calls
					 */
					WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
					set_bit(status, &_net_info->sme_state);
					if (cfg->state_notifier &&
						test_bit(status, &(cfg->interrested_state)))
						cfg->state_notifier(cfg, _net_info, status, true);
					return;
				case 2:
					/*
					 * Release the spinlock before calling notifier. Else there
					 * will be nested calls
					 */
					WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
					clear_bit(status, &_net_info->sme_state);
					if (cfg->state_notifier &&
						test_bit(status, &(cfg->interrested_state)))
						cfg->state_notifier(cfg, _net_info, status, false);
					return;
				case 4:
					change_bit(status, &_net_info->sme_state);
					break;
			}
		}

	}
	WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);

}

static inline wl_cfgbss_t *
wl_get_cfgbss_by_wdev(struct bcm_cfg80211 *cfg,
	struct wireless_dev *wdev)
{
	struct net_info *_net_info, *next;
	wl_cfgbss_t *bss = NULL;
	unsigned long int flags;

	WL_CFG_NET_LIST_SYNC_LOCK(&cfg->net_list_sync, flags);
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	BCM_LIST_FOR_EACH_ENTRY_SAFE(_net_info, next, &cfg->net_list, list) {
		GCC_DIAGNOSTIC_POP();
		if (wdev && (_net_info->wdev == wdev)) {
			bss = &_net_info->bss;
			break;
		}
	}

	WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
	return bss;
}

static inline u32
wl_get_status_by_netdev(struct bcm_cfg80211 *cfg, s32 status,
	struct net_device *ndev)
{
	struct net_info *_net_info, *next;
	u32 stat = 0;
	unsigned long int flags;

	WL_CFG_NET_LIST_SYNC_LOCK(&cfg->net_list_sync, flags);
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	BCM_LIST_FOR_EACH_ENTRY_SAFE(_net_info, next, &cfg->net_list, list) {
		GCC_DIAGNOSTIC_POP();
		if (ndev && (_net_info->ndev == ndev)) {
			stat = test_bit(status, &_net_info->sme_state);
			break;
		}
	}
	WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
	return stat;
}

static inline s32
wl_get_mode_by_netdev(struct bcm_cfg80211 *cfg, struct net_device *ndev)
{
	struct net_info *_net_info, *next;
	s32 mode = -1;
	unsigned long int flags;

	WL_CFG_NET_LIST_SYNC_LOCK(&cfg->net_list_sync, flags);
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	BCM_LIST_FOR_EACH_ENTRY_SAFE(_net_info, next, &cfg->net_list, list) {
		GCC_DIAGNOSTIC_POP();
		if (_net_info->ndev && (_net_info->ndev == ndev)) {
			mode = wl_iftype_to_mode(_net_info->iftype);
			break;
		}
	}
	WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
	return mode;
}

static inline s32
wl_get_bssidx_by_wdev(struct bcm_cfg80211 *cfg, struct wireless_dev *wdev)
{
	struct net_info *_net_info, *next;
	s32 bssidx = -1;
	unsigned long int flags;

	WL_CFG_NET_LIST_SYNC_LOCK(&cfg->net_list_sync, flags);
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	BCM_LIST_FOR_EACH_ENTRY_SAFE(_net_info, next, &cfg->net_list, list) {
		GCC_DIAGNOSTIC_POP();
		if (_net_info->wdev && (_net_info->wdev == wdev)) {
			bssidx = _net_info->bssidx;
			break;
		}
	}
	WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
	return bssidx;
}

static inline struct wireless_dev *
wl_get_wdev_by_fw_idx(struct bcm_cfg80211 *cfg, s32 bssidx, s32 ifidx)
{
	struct net_info *_net_info, *next;
	struct wireless_dev *wdev = NULL;
	unsigned long int flags;

	if (bssidx < 0)
		return NULL;
	WL_CFG_NET_LIST_SYNC_LOCK(&cfg->net_list_sync, flags);
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	BCM_LIST_FOR_EACH_ENTRY_SAFE(_net_info, next, &cfg->net_list, list) {
		GCC_DIAGNOSTIC_POP();
		if ((_net_info->bssidx == bssidx) && (_net_info->ifidx == ifidx)) {
			wdev = _net_info->wdev;
			break;
		}
	}
	WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
	return wdev;
}

static inline struct wl_profile *
wl_get_profile_by_netdev(struct bcm_cfg80211 *cfg, struct net_device *ndev)
{
	struct net_info *_net_info, *next;
	struct wl_profile *prof = NULL;
	unsigned long int flags;

	WL_CFG_NET_LIST_SYNC_LOCK(&cfg->net_list_sync, flags);
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	BCM_LIST_FOR_EACH_ENTRY_SAFE(_net_info, next, &cfg->net_list, list) {
		GCC_DIAGNOSTIC_POP();
		if (ndev && (_net_info->ndev == ndev)) {
			prof = &_net_info->profile;
			break;
		}
	}
	WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
	return prof;
}
static inline struct net_info *
wl_get_netinfo_by_netdev(struct bcm_cfg80211 *cfg, struct net_device *ndev)
{
	struct net_info *_net_info, *next, *info = NULL;
	unsigned long int flags;

	WL_CFG_NET_LIST_SYNC_LOCK(&cfg->net_list_sync, flags);
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	BCM_LIST_FOR_EACH_ENTRY_SAFE(_net_info, next, &cfg->net_list, list) {
		GCC_DIAGNOSTIC_POP();
		if (ndev && (_net_info->ndev == ndev)) {
			info = _net_info;
			break;
		}
	}
	WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
	return info;
}

static inline struct net_info *
wl_get_netinfo_by_wdev(struct bcm_cfg80211 *cfg, struct wireless_dev *wdev)
{
	struct net_info *_net_info, *next, *info = NULL;
	unsigned long int flags;

	WL_CFG_NET_LIST_SYNC_LOCK(&cfg->net_list_sync, flags);
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	BCM_LIST_FOR_EACH_ENTRY_SAFE(_net_info, next, &cfg->net_list, list) {
		GCC_DIAGNOSTIC_POP();
		if (wdev && (_net_info->wdev == wdev)) {
			info = _net_info;
			break;
		}
	}
	WL_CFG_NET_LIST_SYNC_UNLOCK(&cfg->net_list_sync, flags);
	return info;
}

static inline char *
wl_iftype_to_str(int wl_iftype)
{
	switch (wl_iftype) {
		case (WL_IF_TYPE_STA):
			return "WL_IF_TYPE_STA";
		case (WL_IF_TYPE_AP):
			return "WL_IF_TYPE_AP";

#ifdef WLAWDL
		case (WL_IF_TYPE_AWDL):
			return "WL_IF_TYPE_AWDL";
#endif /* WLAWDL */

		case (WL_IF_TYPE_NAN_NMI):
			return "WL_IF_TYPE_NAN_NMI";
		case (WL_IF_TYPE_NAN):
			return "WL_IF_TYPE_NAN";
		case (WL_IF_TYPE_P2P_GO):
			return "WL_IF_TYPE_P2P_GO";
		case (WL_IF_TYPE_P2P_GC):
			return "WL_IF_TYPE_P2P_GC";
		case (WL_IF_TYPE_P2P_DISC):
			return "WL_IF_TYPE_P2P_DISC";
		case (WL_IF_TYPE_IBSS):
			return "WL_IF_TYPE_IBSS";
		case (WL_IF_TYPE_MONITOR):
			return "WL_IF_TYPE_MONITOR";
		case (WL_IF_TYPE_AIBSS):
			return "WL_IF_TYPE_AIBSS";
		default:
			return "WL_IF_TYPE_UNKNOWN";
	}
}

#define is_discovery_iface(iface) (((iface == WL_IF_TYPE_P2P_DISC) || \
	(iface == WL_IF_TYPE_NAN_NMI)) ? 1 : 0)
#define IS_P2P_GC(wdev) \
		((wdev->iftype == NL80211_IFTYPE_P2P_CLIENT) ? 1 : 0)
#define IS_P2P_GO(wdev) \
		((wdev->iftype == NL80211_IFTYPE_P2P_GO) ? 1 : 0)
#define is_p2p_group_iface(wdev) (((wdev->iftype == NL80211_IFTYPE_P2P_GO) || \
		(wdev->iftype == NL80211_IFTYPE_P2P_CLIENT)) ? 1 : 0)
#define bcmcfg_to_wiphy(cfg) (cfg->wdev->wiphy)
#define bcmcfg_to_prmry_ndev(cfg) (cfg->wdev->netdev)
#define bcmcfg_to_prmry_wdev(cfg) (cfg->wdev)
#define bcmcfg_to_p2p_wdev(cfg) (cfg->p2p_wdev)
#define ndev_to_wl(n) (wdev_to_wl(n->ieee80211_ptr))
#define ndev_to_wdev(ndev) (ndev->ieee80211_ptr)
#define wdev_to_ndev(wdev) (wdev->netdev)

#ifdef WL_BLOCK_P2P_SCAN_ON_STA
#define IS_P2P_IFACE(wdev) (wdev && \
		((wdev->iftype == NL80211_IFTYPE_P2P_DEVICE) || \
		(wdev->iftype == NL80211_IFTYPE_P2P_GO) || \
		(wdev->iftype == NL80211_IFTYPE_P2P_CLIENT)))
#endif /* WL_BLOCK_P2P_SCAN_ON_STA */

#define IS_PRIMARY_NDEV(cfg, ndev)	(ndev == bcmcfg_to_prmry_ndev(cfg))
#define IS_STA_IFACE(wdev) (wdev && \
		(wdev->iftype == NL80211_IFTYPE_STATION))

#define IS_AP_IFACE(wdev) (wdev && \
	(wdev->iftype == NL80211_IFTYPE_AP))

#if defined(WL_ENABLE_P2P_IF)
#define ndev_to_wlc_ndev(ndev, cfg)	((ndev == cfg->p2p_net) ? \
	bcmcfg_to_prmry_ndev(cfg) : ndev)
#else
#define ndev_to_wlc_ndev(ndev, cfg)	(ndev)
#endif /* WL_ENABLE_P2P_IF */

#define wdev_to_wlc_ndev(wdev, cfg)	\
	(wdev_to_ndev(wdev) ? \
	wdev_to_ndev(wdev) : bcmcfg_to_prmry_ndev(cfg))
#if defined(WL_CFG80211_P2P_DEV_IF)
#define cfgdev_to_wlc_ndev(cfgdev, cfg)	wdev_to_wlc_ndev(cfgdev, cfg)
#define bcmcfg_to_prmry_cfgdev(cfgdev, cfg) bcmcfg_to_prmry_wdev(cfg)
#elif defined(WL_ENABLE_P2P_IF)
#define cfgdev_to_wlc_ndev(cfgdev, cfg)	ndev_to_wlc_ndev(cfgdev, cfg)
#define bcmcfg_to_prmry_cfgdev(cfgdev, cfg) bcmcfg_to_prmry_ndev(cfg)
#else
#define cfgdev_to_wlc_ndev(cfgdev, cfg)	(cfgdev)
#define bcmcfg_to_prmry_cfgdev(cfgdev, cfg) (cfgdev)
#endif /* WL_CFG80211_P2P_DEV_IF */

#if defined(WL_CFG80211_P2P_DEV_IF)
#define cfgdev_to_wdev(cfgdev)	(cfgdev)
#define ndev_to_cfgdev(ndev)	ndev_to_wdev(ndev)
#define cfgdev_to_ndev(cfgdev)	(cfgdev ? (cfgdev->netdev) : NULL)
#define wdev_to_cfgdev(cfgdev)	(cfgdev)
#define discover_cfgdev(cfgdev, cfg) (cfgdev->iftype == NL80211_IFTYPE_P2P_DEVICE)
#else
#define cfgdev_to_wdev(cfgdev)	(cfgdev->ieee80211_ptr)
#define wdev_to_cfgdev(cfgdev)	cfgdev ? (cfgdev->netdev) : NULL
#define ndev_to_cfgdev(ndev)	(ndev)
#define cfgdev_to_ndev(cfgdev)	(cfgdev)
#define discover_cfgdev(cfgdev, cfg) (cfgdev == cfg->p2p_net)
#endif /* WL_CFG80211_P2P_DEV_IF */

#if defined(WL_CFG80211_P2P_DEV_IF)
#define scan_req_match(cfg)	(((cfg) && (cfg->scan_request) && \
	(cfg->scan_request->wdev == cfg->p2p_wdev)) ? true : false)
#elif defined(WL_ENABLE_P2P_IF)
#define scan_req_match(cfg)	(((cfg) && (cfg->scan_request) && \
	(cfg->scan_request->dev == cfg->p2p_net)) ? true : false)
#else
#define scan_req_match(cfg)	(((cfg) && p2p_is_on(cfg) && p2p_scan(cfg)) ? \
	true : false)
#endif /* WL_CFG80211_P2P_DEV_IF */

#define	PRINT_WDEV_INFO(cfgdev)	\
	{ \
		struct wireless_dev *wdev = cfgdev_to_wdev(cfgdev); \
		struct net_device *netdev = wdev ? wdev->netdev : NULL; \
		WL_DBG(("wdev_ptr:%p ndev_ptr:%p ifname:%s iftype:%d\n", OSL_OBFUSCATE_BUF(wdev), \
			OSL_OBFUSCATE_BUF(netdev),	\
			netdev ? netdev->name : "NULL (non-ndev device)",	\
			wdev ? wdev->iftype : 0xff)); \
	}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 0))
#define scan_req_iftype(req) (req->dev->ieee80211_ptr->iftype)
#else
#define scan_req_iftype(req) (req->wdev->iftype)
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 0) */

#define wl_to_sr(w) (w->scan_req_int)
#if defined(STATIC_WL_PRIV_STRUCT)
#define wl_to_ie(w) (w->ie)
#define wl_to_conn(w) (w->conn_info)
#else
#define wl_to_ie(w) (&w->ie)
#define wl_to_conn(w) (&w->conn_info)
#endif
#define wl_to_fils_info(w) (&w->fils_info)
#define wiphy_from_scan(w) (w->escan_info.wiphy)
#define wl_get_drv_status_all(cfg, stat) \
	(wl_get_status_all(cfg, WL_STATUS_ ## stat))
#define wl_get_drv_status(cfg, stat, ndev)  \
	(wl_get_status_by_netdev(cfg, WL_STATUS_ ## stat, ndev))
#define wl_set_drv_status(cfg, stat, ndev)  \
	(wl_set_status_by_netdev(cfg, WL_STATUS_ ## stat, ndev, 1))
#define wl_clr_drv_status(cfg, stat, ndev)  \
	(wl_set_status_by_netdev(cfg, WL_STATUS_ ## stat, ndev, 2))
#define wl_clr_drv_status_all(cfg, stat)  \
	(wl_set_status_all(cfg, WL_STATUS_ ## stat, 2))
#define wl_chg_drv_status(cfg, stat, ndev)  \
	(wl_set_status_by_netdev(cfg, WL_STATUS_ ## stat, ndev, 4))

#define for_each_bss(list, bss, __i)	\
	for (__i = 0; __i < list->count && __i < WL_AP_MAX; __i++, bss = next_bss(list, bss))

#define for_each_ndev(cfg, iter, next) \
	list_for_each_entry_safe(iter, next, &cfg->net_list, list)

/* In case of WPS from wpa_supplicant, pairwise siute and group suite is 0.
 * In addtion to that, wpa_version is WPA_VERSION_1
 */
#define is_wps_conn(_sme) \
	((wl_cfgp2p_find_wpsie(_sme->ie, _sme->ie_len) != NULL) && \
	 (!_sme->crypto.n_ciphers_pairwise) && \
	 (!_sme->crypto.cipher_group))

#ifdef WLFBT
#if defined(WLAN_AKM_SUITE_FT_8021X) && defined(WLAN_AKM_SUITE_FT_PSK)
#define IS_AKM_SUITE_FT(sec) (sec->wpa_auth == WLAN_AKM_SUITE_FT_8021X || \
		sec->wpa_auth == WLAN_AKM_SUITE_FT_PSK)
#elif defined(WLAN_AKM_SUITE_FT_8021X)
#define IS_AKM_SUITE_FT(sec) (sec->wpa_auth == WLAN_AKM_SUITE_FT_8021X)
#elif defined(WLAN_AKM_SUITE_FT_PSK)
#define IS_AKM_SUITE_FT(sec) (sec->wpa_auth == WLAN_AKM_SUITE_FT_PSK)
#else
#define IS_AKM_SUITE_FT(sec) ({BCM_REFERENCE(sec); FALSE;})
#endif /* WLAN_AKM_SUITE_FT_8021X && WLAN_AKM_SUITE_FT_PSK */
#else
#define IS_AKM_SUITE_FT(sec) ({BCM_REFERENCE(sec); FALSE;})
#endif /* WLFBT */

#define IS_AKM_SUITE_CCKM(sec) ({BCM_REFERENCE(sec); FALSE;})

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0))
#define STA_INFO_BIT(info) (1ul << NL80211_STA_ ## info)
#ifdef strnicmp
#undef strnicmp
#endif /* strnicmp */
#define strnicmp(str1, str2, len) strncasecmp((str1), (str2), (len))
#else
#define STA_INFO_BIT(info) (STATION_ ## info)
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)) */

extern s32 wl_cfg80211_attach(struct net_device *ndev, void *context);
extern void wl_cfg80211_detach(struct bcm_cfg80211 *cfg);

extern void wl_cfg80211_event(struct net_device *ndev, const wl_event_msg_t *e,
            void *data);
extern s32 wl_cfg80211_handle_critical_events(struct bcm_cfg80211 *cfg,
	struct wireless_dev *wdev, const wl_event_msg_t * e);

void wl_cfg80211_set_parent_dev(void *dev);
struct device *wl_cfg80211_get_parent_dev(void);
struct bcm_cfg80211 *wl_cfg80211_get_bcmcfg(void);
void wl_cfg80211_set_bcmcfg(struct bcm_cfg80211 *cfg);

/* clear IEs */
extern s32 wl_cfg80211_clear_mgmt_vndr_ies(struct bcm_cfg80211 *cfg);
extern s32 wl_cfg80211_clear_per_bss_ies(struct bcm_cfg80211 *cfg, struct wireless_dev *wdev);
extern void wl_cfg80211_clear_p2p_disc_ies(struct bcm_cfg80211 *cfg);
#ifdef WL_STATIC_IF
extern int32 wl_cfg80211_update_iflist_info(struct bcm_cfg80211 *cfg, struct net_device *ndev,
	int ifidx, uint8 *addr, int bssidx, char *name, int if_state);
#endif /* WL_STATIC_IF */
extern s32 wl_cfg80211_up(struct net_device *net);
extern s32 wl_cfg80211_down(struct net_device *net);
extern void wl_cfg80211_sta_ifdown(struct net_device *net);
extern s32 wl_cfg80211_notify_ifadd(struct net_device * dev, int ifidx, char *name, uint8 *mac,
	uint8 bssidx, uint8 role);
extern s32 wl_cfg80211_notify_ifdel(struct net_device * dev, int ifidx, char *name, uint8 *mac,
	uint8 bssidx);
extern s32 wl_cfg80211_notify_ifchange(struct net_device * dev, int ifidx, char *name, uint8 *mac,
	uint8 bssidx);
extern struct net_device* wl_cfg80211_allocate_if(struct bcm_cfg80211 *cfg, int ifidx,
	const char *name, uint8 *mac, uint8 bssidx, const char *dngl_name);
extern int wl_cfg80211_register_if(struct bcm_cfg80211 *cfg,
	int ifidx, struct net_device* ndev, bool rtnl_lock_reqd);
extern int wl_cfg80211_remove_if(struct bcm_cfg80211 *cfg,
	int ifidx, struct net_device* ndev, bool rtnl_lock_reqd);
extern void wl_cfg80211_cleanup_if(struct net_device *dev);
extern bool wl_cfg80211_is_concurrent_mode(struct net_device * dev);
extern void wl_cfg80211_disassoc(struct net_device *ndev, uint32 reason);
extern void wl_cfg80211_del_all_sta(struct net_device *ndev, uint32 reason);
extern void* wl_cfg80211_get_dhdp(struct net_device * dev);
extern bool wl_cfg80211_is_p2p_active(struct net_device * dev);
extern bool wl_cfg80211_is_roam_offload(struct net_device * dev);
extern bool wl_cfg80211_is_event_from_connected_bssid(struct net_device * dev,
		const wl_event_msg_t *e, int ifidx);
extern void wl_cfg80211_dbg_level(u32 level);
extern s32 wl_cfg80211_get_p2p_dev_addr(struct net_device *net, struct ether_addr *p2pdev_addr);
extern s32 wl_cfg80211_set_p2p_noa(struct net_device *net, char* buf, int len);
extern s32 wl_cfg80211_get_p2p_noa(struct net_device *net, char* buf, int len);
extern s32 wl_cfg80211_set_wps_p2p_ie(struct net_device *net, char *buf, int len,
	enum wl_management_type type);
extern s32 wl_cfg80211_set_p2p_ps(struct net_device *net, char* buf, int len);
extern s32 wl_cfg80211_set_p2p_ecsa(struct net_device *net, char* buf, int len);
extern s32 wl_cfg80211_increase_p2p_bw(struct net_device *net, char* buf, int len);
#ifdef P2PLISTEN_AP_SAMECHN
extern s32 wl_cfg80211_set_p2p_resp_ap_chn(struct net_device *net, s32 enable);
#endif /* P2PLISTEN_AP_SAMECHN */

/* btcoex functions */
void* wl_cfg80211_btcoex_init(struct net_device *ndev);
void wl_cfg80211_btcoex_deinit(void);

extern chanspec_t wl_chspec_from_legacy(chanspec_t legacy_chspec);
extern chanspec_t wl_chspec_driver_to_host(chanspec_t chanspec);

#ifdef WL_SDO
extern s32 wl_cfg80211_sdo_init(struct bcm_cfg80211 *cfg);
extern s32 wl_cfg80211_sdo_deinit(struct bcm_cfg80211 *cfg);
extern s32 wl_cfg80211_sd_offload(struct net_device *net, char *cmd, char* buf, int len);
extern s32 wl_cfg80211_pause_sdo(struct net_device *dev, struct bcm_cfg80211 *cfg);
extern s32 wl_cfg80211_resume_sdo(struct net_device *dev, struct bcm_cfg80211 *cfg);

#endif
#ifdef WL_SUPPORT_AUTO_CHANNEL
#define CHANSPEC_BUF_SIZE	2048
#define CHANINFO_LIST_BUF_SIZE     (1024 * 4)
#define CHAN_SEL_IOCTL_DELAY	300
#define CHAN_SEL_RETRY_COUNT	15
#define CHANNEL_IS_RADAR(channel)	(((channel & WL_CHAN_RADAR) || \
	(channel & WL_CHAN_PASSIVE)) ? true : false)
#define CHANNEL_IS_2G(channel)	(((channel >= 1) && (channel <= 14)) ? \
	true : false)
#define CHANNEL_IS_5G(channel)	(((channel >= 36) && (channel <= 165)) ? \
	true : false)
extern s32 wl_cfg80211_get_best_channels(struct net_device *dev, char* command,
	int total_len);
#endif /* WL_SUPPORT_AUTO_CHANNEL */
extern int wl_cfg80211_ether_atoe(const char *a, struct ether_addr *n);
extern int wl_cfg80211_hang(struct net_device *dev, u16 reason);
extern bool wl_cfg80211_macaddr_sync_reqd(struct net_device *dev);
void wl_cfg80211_generate_mac_addr(struct ether_addr *ea_addr);
extern s32 wl_mode_to_nl80211_iftype(s32 mode);
int wl_cfg80211_do_driver_init(struct net_device *net);
void wl_cfg80211_enable_trace(u32 level);
extern s32 wl_update_wiphybands(struct bcm_cfg80211 *cfg, bool notify);
extern s32 wl_cfg80211_if_is_group_owner(void);
extern chanspec_t wl_chspec_host_to_driver(chanspec_t chanspec);
extern chanspec_t wl_ch_host_to_driver(u16 channel);
extern s32 wl_set_tx_power(struct net_device *dev,
	enum nl80211_tx_power_setting type, s32 dbm);
extern s32 wl_get_tx_power(struct net_device *dev, s32 *dbm);
extern s32 wl_add_remove_eventmsg(struct net_device *ndev, u16 event, bool add);
extern void wl_stop_wait_next_action_frame(struct bcm_cfg80211 *cfg, struct net_device *ndev,
	u8 bsscfgidx);
#ifdef WL_HOST_BAND_MGMT
extern s32 wl_cfg80211_set_band(struct net_device *ndev, int band);
#endif /* WL_HOST_BAND_MGMT */

#if defined(OEM_ANDROID) && defined(DHCP_SCAN_SUPPRESS)
extern int wl_cfg80211_scan_suppress(struct net_device *dev, int suppress);
#endif /* OEM_ANDROID */

extern void wl_cfg80211_add_to_eventbuffer(wl_eventmsg_buf_t *ev, u16 event, bool set);
extern s32 wl_cfg80211_apply_eventbuffer(struct net_device *ndev,
	struct bcm_cfg80211 *cfg, wl_eventmsg_buf_t *ev);
extern void get_primary_mac(struct bcm_cfg80211 *cfg, struct ether_addr *mac);
extern void wl_cfg80211_update_power_mode(struct net_device *dev);
extern void wl_terminate_event_handler(struct net_device *dev);
#if defined(DHD_ENABLE_BIGDATA_LOGGING)
extern s32 wl_cfg80211_get_bss_info(struct net_device *dev, char* cmd, int total_len);
extern s32 wl_cfg80211_get_connect_failed_status(struct net_device *dev, char* cmd, int total_len);
#endif /* DHD_ENABLE_BIGDATA_LOGGING */
extern struct bcm_cfg80211 *wl_get_cfg(struct net_device *ndev);
extern s32 wl_cfg80211_set_if_band(struct net_device *ndev, int band);
extern s32 wl_cfg80211_set_country_code(struct net_device *dev, char *country_code,
        bool notify, bool user_enforced, int revinfo);
extern bool wl_cfg80211_is_hal_started(struct bcm_cfg80211 *cfg);
#ifdef WL_WIPSEVT
extern int wl_cfg80211_wips_event(uint16 misdeauth, char* bssid);
extern int wl_cfg80211_wips_event_ext(wl_wips_event_info_t *wips_event);
#endif /* WL_WIPSEVT */

#define SCAN_BUF_CNT	2
#define SCAN_BUF_NEXT	1
#define WL_SCANTYPE_LEGACY	0x1
#define WL_SCANTYPE_P2P		0x2
extern void wl_cfg80211_ibss_vsie_set_buffer(struct net_device *dev, vndr_ie_setbuf_t *ibss_vsie,
	int ibss_vsie_len);
extern s32 wl_cfg80211_ibss_vsie_delete(struct net_device *dev);
#ifdef WLAIBSS
extern void wl_cfg80211_set_txfail_pid(struct net_device *dev, int pid);
#endif /* WLAIBSS */
#ifdef WL_RELMCAST
extern void wl_cfg80211_set_rmc_pid(struct net_device *dev, int pid);
#endif /* WL_RELMCAST */
extern int wl_cfg80211_set_mgmt_vndr_ies(struct bcm_cfg80211 *cfg,
	bcm_struct_cfgdev *cfgdev, s32 bssidx, s32 pktflag,
	const u8 *vndr_ie, u32 vndr_ie_len);

#ifdef WLFBT
extern int wl_cfg80211_get_fbt_key(struct net_device *dev, uint8 *key, int total_len);
#endif

/* Action frame specific functions */
extern u8 wl_get_action_category(void *frame, u32 frame_len);
extern int wl_get_public_action(void *frame, u32 frame_len, u8 *ret_action);

#ifdef WL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST
struct net_device *wl_cfg80211_get_remain_on_channel_ndev(struct bcm_cfg80211 *cfg);
#endif /* WL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST */

#ifdef WL_SUPPORT_ACS
#define ACS_MSRMNT_DELAY 1000 /* dump_obss delay in ms */
#define IOCTL_RETRY_COUNT 5
#define CHAN_NOISE_DUMMY -80
#define OBSS_TOKEN_IDX 15
#define IBSS_TOKEN_IDX 15
#define TX_TOKEN_IDX 14
#define CTG_TOKEN_IDX 13
#define PKT_TOKEN_IDX 15
#define IDLE_TOKEN_IDX 12
#endif /* WL_SUPPORT_ACS */

#ifdef BCMWAPI_WPI
#define is_wapi(cipher) (cipher == WLAN_CIPHER_SUITE_SMS4) ? 1 : 0
#endif /* BCMWAPI_WPI */

extern int wl_cfg80211_get_ioctl_version(void);
extern int wl_cfg80211_enable_roam_offload(struct net_device *dev, int enable);
#ifdef WBTEXT
extern s32 wl_cfg80211_wbtext_set_default(struct net_device *ndev);
extern s32 wl_cfg80211_wbtext_config(struct net_device *ndev, char *data,
		char *command, int total_len);
extern int wl_cfg80211_wbtext_weight_config(struct net_device *ndev, char *data,
		char *command, int total_len);
extern int wl_cfg80211_wbtext_table_config(struct net_device *ndev, char *data,
		char *command, int total_len);
extern s32 wl_cfg80211_wbtext_delta_config(struct net_device *ndev, char *data,
		char *command, int total_len);
#endif /* WBTEXT */
extern s32 wl_cfg80211_get_band_chanspecs(struct net_device *ndev,
		void *buf, s32 buflen, chanspec_band_t band, bool acs_req);

extern s32 wl_cfg80211_bss_up(struct bcm_cfg80211 *cfg,
	struct net_device *ndev, s32 bsscfg_idx, s32 up);
extern bool wl_cfg80211_bss_isup(struct net_device *ndev, int bsscfg_idx);

struct net_device *wl_cfg80211_post_ifcreate(struct net_device *ndev,
	wl_if_event_info *event, u8 *addr, const char *name, bool rtnl_lock_reqd);
extern s32 wl_cfg80211_post_ifdel(struct net_device *ndev, bool rtnl_lock_reqd, s32 ifidx);
#if defined(PKT_FILTER_SUPPORT) && defined(APSTA_BLOCK_ARP_DURING_DHCP)
extern void wl_cfg80211_block_arp(struct net_device *dev, int enable);
#endif /* PKT_FILTER_SUPPORT && APSTA_BLOCK_ARP_DURING_DHCP */

#ifdef WLTDLS
extern s32 wl_cfg80211_tdls_config(struct bcm_cfg80211 *cfg,
	enum wl_tdls_config state, bool tdls_mode);
extern s32 wl_tdls_event_handler(struct bcm_cfg80211 *cfg, bcm_struct_cfgdev *cfgdev,
	const wl_event_msg_t *e, void *data);
#endif /* WLTDLS */

#ifdef WL_NAN
extern int wl_cfgvendor_send_nan_event(struct wiphy * wiphy,
	struct net_device *dev, int event_id,
	nan_event_data_t *nan_event_data);
#ifdef RTT_SUPPORT
extern s32 wl_cfgvendor_send_as_rtt_legacy_event(struct wiphy *wiphy,
	struct net_device *dev, wl_nan_ev_rng_rpt_ind_t *range_res,
	uint32 status);
#endif /* RTT_SUPPORT */
#ifdef WL_NANP2P
extern int wl_cfg80211_set_iface_conc_disc(struct net_device *ndev,
	uint8 arg_val);
extern uint8 wl_cfg80211_get_iface_conc_disc(struct net_device *ndev);
#endif /* WL_NANP2P */
#endif /* WL_NAN */

#ifdef WL_CFG80211_P2P_DEV_IF
extern void wl_cfg80211_del_p2p_wdev(struct net_device *dev);
#endif /* WL_CFG80211_P2P_DEV_IF */
#ifdef WL_CFG80211_SYNC_GON
#define WL_DRV_STATUS_SENDING_AF_FRM_EXT(cfg) \
	(wl_get_drv_status_all(cfg, SENDING_ACT_FRM) || \
		wl_get_drv_status_all(cfg, WAITING_NEXT_ACT_FRM_LISTEN))
#else
#define WL_DRV_STATUS_SENDING_AF_FRM_EXT(cfg) wl_get_drv_status_all(cfg, SENDING_ACT_FRM)
#endif /* WL_CFG80211_SYNC_GON */

#ifdef P2P_LISTEN_OFFLOADING
extern s32 wl_cfg80211_p2plo_deinit(struct bcm_cfg80211 *cfg);
#endif /* P2P_LISTEN_OFFLOADING */

/* Function to flush the FW log buffer content */
extern void wl_flush_fw_log_buffer(struct net_device *dev, uint32 logset_mask);

#define RETURN_EIO_IF_NOT_UP(wlpriv)                        \
do {                                    \
	struct net_device *checkSysUpNDev = bcmcfg_to_prmry_ndev(wlpriv);           \
	if (unlikely(!wl_get_drv_status(wlpriv, READY, checkSysUpNDev))) {  \
		WL_INFORM(("device is not ready\n"));           \
		return -EIO;                        \
	}                               \
} while (0)

#ifdef QOS_MAP_SET
extern uint8 *wl_get_up_table(dhd_pub_t * dhdp, int idx);
#endif /* QOS_MAP_SET */

#define P2PO_COOKIE     65535
u64 wl_cfg80211_get_new_roc_id(struct bcm_cfg80211 *cfg);

#define ROAMSCAN_MODE_NORMAL	0
#define ROAMSCAN_MODE_WES	1

#ifdef SUPPORT_RSSI_SUM_REPORT
int wl_get_rssi_logging(struct net_device *dev, void *param);
int wl_set_rssi_logging(struct net_device *dev, void *param);
int wl_get_rssi_per_ant(struct net_device *dev, char *ifname, char *peer_mac, void *param);
#endif /* SUPPORT_RSSI_SUM_REPORT */
struct wireless_dev * wl_cfg80211_add_if(struct bcm_cfg80211 *cfg, struct net_device *primary_ndev,
	wl_iftype_t wl_iftype, const char *name, u8 *mac);
extern s32 wl_cfg80211_del_if(struct bcm_cfg80211 *cfg, struct net_device *primary_ndev,
	struct wireless_dev *wdev, char *name);
s32 _wl_cfg80211_del_if(struct bcm_cfg80211 *cfg, struct net_device *primary_ndev,
	struct wireless_dev *wdev, char *ifname);
s32 wl_cfg80211_delete_iface(struct bcm_cfg80211 *cfg, wl_iftype_t sec_data_if_type);

#ifdef WL_STATIC_IF
extern struct net_device *wl_cfg80211_register_static_if(struct bcm_cfg80211 *cfg,
	u16 iftype, char *ifname, int static_ifidx);
extern void wl_cfg80211_unregister_static_if(struct bcm_cfg80211 * cfg);
extern s32 wl_cfg80211_static_if_open(struct net_device *net);
extern s32 wl_cfg80211_static_if_close(struct net_device *net);
extern struct net_device * wl_cfg80211_post_static_ifcreate(struct bcm_cfg80211 *cfg,
	wl_if_event_info *event, u8 *addr, s32 iface_type, int static_ifidx);
extern s32 wl_cfg80211_post_static_ifdel(struct bcm_cfg80211 *cfg, struct net_device *ndev);
#endif  /* WL_STATIC_IF */
extern struct wireless_dev *wl_cfg80211_get_wdev_from_ifname(struct bcm_cfg80211 *cfg,
	const char *name);
struct net_device* wl_get_netdev_by_name(struct bcm_cfg80211 *cfg, char *ifname);
extern int wl_cfg80211_ifstats_counters(struct net_device *dev, wl_if_stats_t *if_stats);
extern s32 wl_cfg80211_set_dbg_verbose(struct net_device *ndev, u32 level);
extern int wl_cfg80211_deinit_p2p_discovery(struct bcm_cfg80211 * cfg);
extern int wl_cfg80211_set_frameburst(struct bcm_cfg80211 *cfg, bool enable);
extern int wl_cfg80211_determine_p2p_rsdb_scc_mode(struct bcm_cfg80211 *cfg);
extern uint8 wl_cfg80211_get_bus_state(struct bcm_cfg80211 *cfg);
#ifdef WL_WPS_SYNC
void wl_handle_wps_states(struct net_device *ndev, u8 *dump_data, u16 len, bool direction);
#endif /* WL_WPS_SYNC */
extern int wl_features_set(u8 *array, uint8 len, u32 ftidx);
extern void *wl_read_prof(struct bcm_cfg80211 *cfg, struct net_device *ndev, s32 item);
extern s32 wl_cfg80211_sup_event_handler(struct bcm_cfg80211 *cfg, bcm_struct_cfgdev *cfgdev,
        const wl_event_msg_t *event, void *data);
#ifdef CUSTOMER_HW4_DEBUG
extern void wl_scan_timeout_dbg_clear(void);
#endif /* CUSTOMER_HW4_DEBUG */
extern s32 cfg80211_to_wl_iftype(uint16 type, uint16 *role, uint16 *mode);
extern s32 wl_cfg80211_net_attach(struct net_device *primary_ndev);
extern void wl_print_verinfo(struct bcm_cfg80211 *cfg);
extern const u8 *wl_find_attribute(const u8 *buf, u16 len, u16 element_id);
extern int wl_cfg80211_get_concurrency_mode(struct bcm_cfg80211 *cfg);
extern s32 wl_cfg80211_config_suspend_events(struct net_device *ndev, bool enable);
#ifdef PCIE_INB_DW
bool wl_cfg80211_check_in_progress(struct net_device *dev);
#endif
#ifdef WES_SUPPORT
extern int wl_android_set_ncho_mode(struct net_device *dev, int mode);
#endif /* WES_SUPPORT */
#ifdef KEEP_ALIVE
extern int wl_cfg80211_start_mkeep_alive(struct bcm_cfg80211 *cfg, uint8 mkeep_alive_id,
	uint16 ether_type, uint8 *ip_pkt, uint16 ip_pkt_len, uint8* src_mac_addr,
	uint8* dst_mac_addr, uint32 period_msec);
extern int wl_cfg80211_stop_mkeep_alive(struct bcm_cfg80211 *cfg, uint8 mkeep_alive_id);
#endif /* KEEP_ALIVE */

extern s32 wl_cfg80211_handle_macaddr_change(struct net_device *dev, u8 *macaddr);
extern int wl_cfg80211_handle_hang_event(struct net_device *ndev,
	uint16 hang_reason, uint32 memdump_type);
#ifndef OEM_ANDROID
extern s32 wl_cfg80211_resume(struct bcm_cfg80211 *cfg);
extern s32 wl_cfg80211_suspend(struct bcm_cfg80211 *cfg);
#endif /* !OEM_ANDROID */
bool wl_cfg80211_is_dpp_frame(void *frame, u32 frame_len);
const char *get_dpp_pa_ftype(enum wl_dpp_ftype ftype);
bool wl_cfg80211_is_dpp_gas_action(void *frame, u32 frame_len);
extern bool wl_cfg80211_find_gas_subtype(u8 subtype, u16 adv_id, u8* data, u32 len);
#ifdef ESCAN_CHANNEL_CACHE
extern void update_roam_cache(struct bcm_cfg80211 *cfg, int ioctl_ver);
#endif /* ESCAN_CHANNEL_CACHE */

#ifdef WL_NAN
extern int wl_cfgnan_get_stats(struct bcm_cfg80211 *cfg);
#endif /* WL_NAN */

#ifdef WL_SAE
extern s32 wl_cfg80211_set_wsec_info(struct net_device *dev, uint32 *data,
	uint16 data_len, int tag);
#endif /* WL_SAE */
#define WL_CHANNEL_ARRAY_INIT(band_chan_arr)	\
do {	\
	u32 arr_size, k;	\
	arr_size = ARRAYSIZE(band_chan_arr);	\
	for (k = 0; k < arr_size; k++) {	\
		band_chan_arr[k].flags = IEEE80211_CHAN_DISABLED;	\
	}	\
} while (0)

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0))
#define CFG80211_PUT_BSS(wiphy, bss) cfg80211_put_bss(wiphy, bss);
#else
#define CFG80211_PUT_BSS(wiphy, bss) cfg80211_put_bss(bss);
#endif /* (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0) */

#ifdef RSSI_OFFSET
static inline s32 wl_rssi_offset(s32 rssi)
{
	rssi += RSSI_OFFSET;
	if (rssi > 0)
		rssi = 0;
	return rssi;
}
#else
#define wl_rssi_offset(x)	x
#endif
extern int wl_channel_to_frequency(u32 chan, chanspec_band_t band);
extern int wl_cfg80211_config_rsnxe_ie(struct bcm_cfg80211 *cfg, struct net_device *dev,
		const u8 *parse, u32 len);
extern bool dhd_force_country_change(struct net_device *dev);
extern u32 wl_dbg_level;
extern u32 wl_cfg80211_debug_data_dump(struct net_device *dev, u8 *buf, u32 buf_len);
extern void wl_cfg80211_concurrent_roam(struct bcm_cfg80211 *cfg, int enable);

extern void wl_cfg80211_iface_state_ops(struct wireless_dev *wdev, wl_interface_state_t state,
	wl_iftype_t wl_iftype, u16 wl_mode);
extern chanspec_t wl_cfg80211_get_shared_freq(struct wiphy *wiphy);
#ifdef SUPPORT_SET_CAC
extern void wl_cfg80211_set_cac(struct bcm_cfg80211 *cfg, int enable);
#endif /* SUPPORT_SET_CAC */
extern s32 wl_cfg80211_add_del_bss(struct bcm_cfg80211 *cfg,
	struct net_device *ndev, s32 bsscfg_idx,
	wl_iftype_t brcm_iftype, s32 del, u8 *addr);
extern s32 wl_bss_handle_sae_auth(struct bcm_cfg80211 *cfg, struct net_device *ndev,
	const wl_event_msg_t *event, void *data);
#ifdef WL_WPS_SYNC
extern s32 wl_wps_session_update(struct net_device *ndev, u16 state, const u8 *peer_mac);
#endif /* WL_WPS_SYNC */
extern s32 wl_update_prof(struct bcm_cfg80211 *cfg, struct net_device *ndev,
	const wl_event_msg_t *e, const void *data, s32 item);
#ifdef WL_CLIENT_SAE
extern s32 wl_handle_auth_event(struct bcm_cfg80211 *cfg, struct net_device *ndev,
	const wl_event_msg_t *e, void *data);
#endif /* WL_CLIENT_SAE */
#ifdef CUSTOMER_HW6
extern bool wl_customer6_legacy_chip_check(struct bcm_cfg80211 *cfg,
	struct net_device *ndev);
#endif /* CUSTOMER_HW6 */
void wl_wlfc_enable(struct bcm_cfg80211 *cfg, bool enable);
s32 wl_handle_join(struct bcm_cfg80211 *cfg, struct net_device *dev,
	wlcfg_assoc_info_t *assoc_info);
s32 wl_handle_reassoc(struct bcm_cfg80211 *cfg, struct net_device *dev,
	wlcfg_assoc_info_t *info);
s32 wl_cfg80211_autochannel(struct net_device *dev, char* command, int total_len);
#endif /* _wl_cfg80211_h_ */
