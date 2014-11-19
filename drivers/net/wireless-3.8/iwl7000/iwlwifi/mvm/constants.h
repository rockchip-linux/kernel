/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2013 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <ilw@linux.intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2013 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2014 Intel Mobile Communications GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
#ifndef __MVM_CONSTANTS_H
#define __MVM_CONSTANTS_H

#include <linux/ieee80211.h>

#ifndef CPTCFG_IWLWIFI_SUPPORT_DEBUG_OVERRIDES
#define IWL_MVM_DEFAULT_PS_TX_DATA_TIMEOUT	(100 * USEC_PER_MSEC)
#define IWL_MVM_DEFAULT_PS_RX_DATA_TIMEOUT	(100 * USEC_PER_MSEC)
#define IWL_MVM_WOWLAN_PS_TX_DATA_TIMEOUT	(10 * USEC_PER_MSEC)
#define IWL_MVM_WOWLAN_PS_RX_DATA_TIMEOUT	(10 * USEC_PER_MSEC)
#define IWL_MVM_UAPSD_RX_DATA_TIMEOUT		(50 * USEC_PER_MSEC)
#define IWL_MVM_UAPSD_TX_DATA_TIMEOUT		(50 * USEC_PER_MSEC)
#define IWL_MVM_UAPSD_QUEUES		(IEEE80211_WMM_IE_STA_QOSINFO_AC_VO |\
					 IEEE80211_WMM_IE_STA_QOSINFO_AC_VI |\
					 IEEE80211_WMM_IE_STA_QOSINFO_AC_BK |\
					 IEEE80211_WMM_IE_STA_QOSINFO_AC_BE)
#define IWL_MVM_PS_HEAVY_TX_THLD_PACKETS	20
#define IWL_MVM_PS_HEAVY_RX_THLD_PACKETS	8
#define IWL_MVM_PS_SNOOZE_HEAVY_TX_THLD_PACKETS	30
#define IWL_MVM_PS_SNOOZE_HEAVY_RX_THLD_PACKETS	20
#define IWL_MVM_PS_HEAVY_TX_THLD_PERCENT	50
#define IWL_MVM_PS_HEAVY_RX_THLD_PERCENT	50
#define IWL_MVM_PS_SNOOZE_INTERVAL		25
#define IWL_MVM_PS_SNOOZE_WINDOW		50
#define IWL_MVM_WOWLAN_PS_SNOOZE_WINDOW		25
#define IWL_MVM_LOWLAT_QUOTA_MIN_PERCENT	64
#define IWL_MVM_BT_COEX_EN_RED_TXP_THRESH	62
#define IWL_MVM_BT_COEX_DIS_RED_TXP_THRESH	65
#define IWL_MVM_BT_COEX_SYNC2SCO		1
#define IWL_MVM_BT_COEX_CORUNNING		1
#define IWL_MVM_BT_COEX_MPLUT			1
#define IWL_MVM_FW_MCAST_FILTER_PASS_ALL	0
#define IWL_MVM_FW_BCAST_FILTER_PASS_ALL	0
#ifdef CPTCFG_IWLMVM_TCM
#define IWL_MVM_TCM_LOAD_MEDIUM_THRESH		10 /* percentage */
#define IWL_MVM_TCM_LOAD_HIGH_THRESH		50 /* percentage */
#define IWL_MVM_TCM_LOWLAT_ENABLE_THRESH	100 /* packets/10 seconds */
#define IWL_MVM_QUOTA_AIRTIME_THRESH		16 /* 16 percent */
#define IWL_MVM_LOWLAT_QUOTA_LOWTRAF_PERCENT	80 /* for low BSS traffic */
#endif /* CPTCFG_IWLMVM_TCM */
#define IWL_MVM_QUOTA_THRESHOLD			8
#define IWL_MVM_RS_RSSI_BASED_INIT_RATE         0
#else /* CPTCFG_IWLWIFI_SUPPORT_DEBUG_OVERRIDES */
#define IWL_MVM_DEFAULT_PS_TX_DATA_TIMEOUT	(mvm->trans->dbg_cfg.MVM_DEFAULT_PS_TX_DATA_TIMEOUT)
#define IWL_MVM_DEFAULT_PS_RX_DATA_TIMEOUT	(mvm->trans->dbg_cfg.MVM_DEFAULT_PS_RX_DATA_TIMEOUT)
#define IWL_MVM_WOWLAN_PS_TX_DATA_TIMEOUT	(mvm->trans->dbg_cfg.MVM_WOWLAN_PS_TX_DATA_TIMEOUT)
#define IWL_MVM_WOWLAN_PS_RX_DATA_TIMEOUT	(mvm->trans->dbg_cfg.MVM_WOWLAN_PS_RX_DATA_TIMEOUT)
#define IWL_MVM_UAPSD_TX_DATA_TIMEOUT		(mvm->trans->dbg_cfg.MVM_UAPSD_TX_DATA_TIMEOUT)
#define IWL_MVM_UAPSD_RX_DATA_TIMEOUT		(mvm->trans->dbg_cfg.MVM_UAPSD_RX_DATA_TIMEOUT)
#define IWL_MVM_UAPSD_QUEUES			(mvm->trans->dbg_cfg.MVM_UAPSD_QUEUES)
#define IWL_MVM_PS_HEAVY_TX_THLD_PACKETS	(mvm->trans->dbg_cfg.MVM_PS_HEAVY_TX_THLD_PACKETS)
#define IWL_MVM_PS_HEAVY_RX_THLD_PACKETS	(mvm->trans->dbg_cfg.MVM_PS_HEAVY_RX_THLD_PACKETS)
#define IWL_MVM_PS_SNOOZE_HEAVY_TX_THLD_PACKETS	(mvm->trans->dbg_cfg.MVM_PS_SNOOZE_HEAVY_TX_THLD_PACKETS)
#define IWL_MVM_PS_SNOOZE_HEAVY_RX_THLD_PACKETS	(mvm->trans->dbg_cfg.MVM_PS_SNOOZE_HEAVY_RX_THLD_PACKETS)
#define IWL_MVM_PS_HEAVY_TX_THLD_PERCENT	(mvm->trans->dbg_cfg.MVM_PS_HEAVY_TX_THLD_PERCENT)
#define IWL_MVM_PS_HEAVY_RX_THLD_PERCENT	(mvm->trans->dbg_cfg.MVM_PS_HEAVY_RX_THLD_PERCENT)
#define IWL_MVM_PS_SNOOZE_INTERVAL		(mvm->trans->dbg_cfg.MVM_PS_SNOOZE_INTERVAL)
#define IWL_MVM_PS_SNOOZE_WINDOW		(mvm->trans->dbg_cfg.MVM_PS_SNOOZE_WINDOW)
#define IWL_MVM_WOWLAN_PS_SNOOZE_WINDOW		(mvm->trans->dbg_cfg.MVM_WOWLAN_PS_SNOOZE_WINDOW)
#define IWL_MVM_LOWLAT_QUOTA_MIN_PERCENT	(mvm->trans->dbg_cfg.MVM_LOWLAT_QUOTA_MIN_PERCENT)
#define IWL_MVM_BT_COEX_EN_RED_TXP_THRESH	(mvm->trans->dbg_cfg.MVM_BT_COEX_EN_RED_TXP_THRESH)
#define IWL_MVM_BT_COEX_DIS_RED_TXP_THRESH	(mvm->trans->dbg_cfg.MVM_BT_COEX_DIS_RED_TXP_THRESH)
#define IWL_MVM_BT_COEX_SYNC2SCO		(mvm->trans->dbg_cfg.MVM_BT_COEX_SYNC2SCO)
#define IWL_MVM_BT_COEX_CORUNNING		(mvm->trans->dbg_cfg.MVM_BT_COEX_CORUNNING)
#define IWL_MVM_BT_COEX_MPLUT			(mvm->trans->dbg_cfg.MVM_BT_COEX_MPLUT)
#define IWL_MVM_FW_MCAST_FILTER_PASS_ALL	(mvm->trans->dbg_cfg.MVM_FW_MCAST_FILTER_PASS_ALL)
#define IWL_MVM_FW_BCAST_FILTER_PASS_ALL	(mvm->trans->dbg_cfg.MVM_FW_BCAST_FILTER_PASS_ALL)
#ifdef CPTCFG_IWLMVM_TCM
#define IWL_MVM_TCM_LOAD_MEDIUM_THRESH		(mvm->trans->dbg_cfg.MVM_TCM_LOAD_MEDIUM_THRESH)
#define IWL_MVM_TCM_LOAD_HIGH_THRESH		(mvm->trans->dbg_cfg.MVM_TCM_LOAD_HIGH_THRESH)
#define IWL_MVM_TCM_LOWLAT_ENABLE_THRESH	(mvm->trans->dbg_cfg.MVM_TCM_LOWLAT_ENABLE_THRESH)
#define IWL_MVM_QUOTA_AIRTIME_THRESH		(mvm->trans->dbg_cfg.MVM_QUOTA_AIRTIME_THRESH)
#define IWL_MVM_LOWLAT_QUOTA_LOWTRAF_PERCENT	(mvm->trans->dbg_cfg.MVM_LOWLAT_QUOTA_LOWTRAF_PERCENT)
#endif /* CPTCFG_IWLMVM_TCM */
#define IWL_MVM_QUOTA_THRESHOLD			(mvm->trans->dbg_cfg.MVM_QUOTA_THRESHOLD)
#define IWL_MVM_RS_RSSI_BASED_INIT_RATE         (mvm->trans->dbg_cfg.MVM_RS_RSSI_BASED_INIT_RATE)
#endif /* CPTCFG_IWLWIFI_SUPPORT_DEBUG_OVERRIDES */

#endif /* __MVM_CONSTANTS_H */
