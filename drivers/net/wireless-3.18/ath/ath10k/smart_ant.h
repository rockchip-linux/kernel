/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "debug.h"
#include "txrx.h"
#include "wmi-ops.h"

#ifndef _SMART_ANT_H_
#define _SMART_ANT_H_

#define ATH10K_TX_FEEDBACK_CONFIG_DEFAULT 0xe4

#define ATH10K_SMART_ANT_PKTLOG_TYPE_TXCTL	1
#define ATH10K_SMART_ANT_PKTLOG_TYPE_TXSTATS	2

#define ATH10K_SMART_ANT_FEEDBACK	29

#define ATH10K_SMART_ANT_NPKTS		29
#define ATH10K_SMART_ANT_NPKTS_M	0xffff
#define ATH10K_SMART_ANT_NPKTS_S	0

#define ATH10K_SMART_ANT_NBAD		29
#define ATH10K_SMART_ANT_NBAD_M		0x1fff0000
#define ATH10K_SMART_ANT_NBAD_S		16

#define ATH10K_SMART_ANT_COMB_FB	29
#define ATH10K_SMART_ANT_COMB_FB_M	0x60000000
#define ATH10K_SMART_ANT_COMB_FB_S	29

#define ATH10K_SMART_ANT_TRAIN_PKT	29
#define ATH10K_SMART_ANT_TRAIN_PKT_M	0x80000000
#define ATH10K_SMART_ANT_TRAIN_PKT_S	31

#define ATH10K_RX_ANT_MASK	0x00ffffff

#define ATH10K_TXC_PEERID	1
#define ATH10K_TXC_FTYPE	13
#define ATH10K_TXC_FTYPE_M	0x3c00000
#define ATH10K_TXC_FTYPE_S	22
#define ATH10K_FTYPE_DATA	0

#define ATH10K_TXC_ANT_S0	18
#define ATH10K_TXC_ANT_S0_M	0x00ffffff
#define ATH10K_TXC_ANT_S0_S	0

#define ATH10K_TXC_ANT_S1	19
#define ATH10K_TXC_ANT_S1_M	0x00ffffff
#define ATH10K_TXC_ANT_S1_S	0

#define ATH10K_TXC_S0_RATE_BW20		22
#define ATH10K_TXC_S0_RATE_BW20_M	0xff000000
#define ATH10K_TXC_S0_RATE_BW20_S	24

#define ATH10K_TXC_S0_RATE_BW40		26
#define ATH10K_TXC_S0_RATE_BW40_M	0xff000000
#define ATH10K_TXC_S0_RATE_BW40_S	16

#define ATH10K_TXC_S0_RATE_BW80		30
#define ATH10K_TXC_S0_RATE_BW80_M	0xff000000
#define ATH10K_TXC_S0_RATE_BW80_S	8

#define ATH10K_TXC_S0_RATE_BW160	34
#define ATH10K_TXC_S0_RATE_BW160_M	0xff000000
#define ATH10K_TXC_S0_RATE_BW160_S	0

#define ATH10K_TXC_S1_RATE_BW20		38
#define ATH10K_TXC_S1_RATE_BW20_M	0xff000000
#define ATH10K_TXC_S1_RATE_BW20_S	24

#define ATH10K_TXC_S1_RATE_BW40		42
#define ATH10K_TXC_S1_RATE_BW40_M	0xff000000
#define ATH10K_TXC_S1_RATE_BW40_S	16

#define ATH10K_TXC_S1_RATE_BW80		46
#define ATH10K_TXC_S1_RATE_BW80_M	0xff000000
#define ATH10K_TXC_S1_RATE_BW80_S	8

#define ATH10K_TXC_S1_RATE_BW160	50
#define ATH10K_TXC_S1_RATE_BW160_M	0xff000000
#define ATH10K_TXC_S1_RATE_BW160_S	0

#define TXCS_MS(desc, info) \
		((__le32_to_cpu(desc[info]) & info## _M) >> info## _S)

#define ATH10K_SMART_ANT_DEFAULT_ANT	2

enum ath10k_smart_ant_rtcount {
	ATH10K_SMART_ANT_RTCNT_LEGACY,
	ATH10K_SMART_ANT_RTCNT_20,
	ATH10K_SMART_ANT_RTCNT_40,
	ATH10K_SMART_ANT_RTCNT_80,
	ATH10K_SMART_ANT_RTCNT_MAX,
};

static char *rate_code_map[] = { "Legacy CCK/OFDM",
				 "VHT/HT 20",
				 "VHT/HT 40",
				 "VHT80" };

static inline void smart_ant_dbg_ratelist(struct ath10k *ar,
				struct ath10k_peer_ratecode_list *rtcode,
				enum ath10k_smart_ant_rtcount mode)
{
	u8 *rlist;
	u8 rcount;
	int i;

	switch (mode) {
	case ATH10K_SMART_ANT_RTCNT_LEGACY:
		rlist = rtcode->rtcode_legacy;
		break;
	case ATH10K_SMART_ANT_RTCNT_20:
		rlist = rtcode->rtcode_20;
		break;
	case ATH10K_SMART_ANT_RTCNT_40:
		rlist = rtcode->rtcode_40;
		break;
	case ATH10K_SMART_ANT_RTCNT_80:
		rlist = rtcode->rtcode_80;
		break;
	case ATH10K_SMART_ANT_RTCNT_MAX:
	default:
		ath10k_info(ar, "Not a valid mode\n");
		return;
	}

	rcount = rtcode->rt_count[mode];
	ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
		   "rate code list for mode %s\n", rate_code_map[mode]);
	for (i = 0; i < rcount; i++)
		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT, "0x%x\n", rlist[i]);
}

/* This API is to process rx feedback such as rssi, PER and antennas.
 * Based on the stats a better antenna combination can be found for rx.
 * Better rx antenna can be configured using ath10k_wmi_pdev_set_rx_ant().
 */

static inline void
ath10k_smart_ant_proc_rx_feedback(struct ath10k *ar,
				  struct htt_rx_desc *rx_desc)
{
	struct ath10k_smart_ant_info *sa_info = &ar->smart_ant_info;

	if (!ath10k_smart_ant_enabled(ar) || !ar->smart_ant_info.enabled)
		return;

	if (!(__le32_to_cpu(rx_desc->attention.flags) &
	     RX_ATTENTION_FLAGS_LAST_MPDU))
		return;

	sa_info->rx_antenna = __le32_to_cpu(rx_desc->ppdu_end.common.info0) &
			      ATH10K_RX_ANT_MASK;
}

/* This API is to process tx feedback information such as tx rate
 * PER, rssi and antennas used for tx. Based on feedback stats a
 * a better antenna combination can be chosen for tx.
 * Better tx antenna can be configured using ath10k_wmi_peer_set_smart_tx_ant().
 * When needed this API can also request for feedback on packets with particular
 * antenna at a particular rate.  This is called packet training and the params
 * needed for training can be configured using
 * ath10k_wmi_peer_set_smart_ant_train_info().
 */
static inline void
ath10k_smart_ant_proc_tx_feedback(struct ath10k *ar, u8 *data)
{
	struct ath10k_pktlog_hdr *pl_hdr = (struct ath10k_pktlog_hdr *)data;
	u16 log_type = __le16_to_cpu(pl_hdr->log_type);
	struct ath10k_peer *peer;
	struct ath10k_smart_ant_info *info = &ar->smart_ant_info;

	if (!ath10k_smart_ant_enabled(ar) || !ar->smart_ant_info.enabled)
		return;

	if (log_type != ATH10K_SMART_ANT_PKTLOG_TYPE_TXCTL &&
	    log_type != ATH10K_SMART_ANT_PKTLOG_TYPE_TXSTATS)
		return;

	if (log_type == ATH10K_SMART_ANT_PKTLOG_TYPE_TXSTATS) {
		memcpy((u8 *)info->tx_ppdu_end, pl_hdr->payload,
		       sizeof(info->tx_ppdu_end));
	} else {
		struct ieee80211_sta *sta;
		u32 *tx_ctrl_desc, *tx_status_desc;
		u32 peer_id;
		u32 ftype;
		u8 peer_mac[ETH_ALEN];

		tx_status_desc = info->tx_ppdu_end;
		tx_ctrl_desc = (u32 *)pl_hdr->payload;

		peer_id = __le32_to_cpu(tx_ctrl_desc[ATH10K_TXC_PEERID]);
		ftype = TXCS_MS(tx_ctrl_desc, ATH10K_TXC_FTYPE);

		if (ftype != ATH10K_FTYPE_DATA)
			return;

		if (!tx_status_desc[ATH10K_SMART_ANT_FEEDBACK])
			return;

		spin_lock_bh(&ar->data_lock);
		peer = ath10k_peer_find_by_id(ar, peer_id);
		if (!peer) {
			spin_unlock_bh(&ar->data_lock);
			return;
		}
		ether_addr_copy(peer_mac, peer->addr);
		spin_unlock_bh(&ar->data_lock);

		rcu_read_lock();
		sta = ieee80211_find_sta_by_ifaddr(ar->hw, peer_mac, NULL);
		if (!sta) {
			rcu_read_unlock();
			ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
				   "Sta entry for %pM not found\n", peer_mac);
			return;
		}

		rcu_read_unlock();

		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
			   "Tx feeback for peer %pM\n", peer_mac);

		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
			   "Tx feedback: num_combined_feedback %d num_pkts %d num_bad %d is_training %s\n",
			   TXCS_MS(tx_status_desc, ATH10K_SMART_ANT_COMB_FB),
			   TXCS_MS(tx_status_desc, ATH10K_SMART_ANT_NPKTS),
			   TXCS_MS(tx_status_desc, ATH10K_SMART_ANT_NBAD),
			   TXCS_MS(tx_status_desc, ATH10K_SMART_ANT_TRAIN_PKT) ?
			   "true" : "false");

		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
			   "Tx feedback: tx_antenna[0] %d tx_antenna[1] %d rate_mcs[0] 0x%x rate_mcs[1] 0x%x\n",
			   TXCS_MS(tx_ctrl_desc, ATH10K_TXC_ANT_S0),
			   TXCS_MS(tx_ctrl_desc, ATH10K_TXC_ANT_S1),
			   TXCS_MS(tx_ctrl_desc, ATH10K_TXC_S0_RATE_BW20) |
			   TXCS_MS(tx_ctrl_desc, ATH10K_TXC_S0_RATE_BW40) |
			   TXCS_MS(tx_ctrl_desc, ATH10K_TXC_S0_RATE_BW80) |
			   TXCS_MS(tx_ctrl_desc, ATH10K_TXC_S0_RATE_BW160),
			   TXCS_MS(tx_ctrl_desc, ATH10K_TXC_S1_RATE_BW20) |
			   TXCS_MS(tx_ctrl_desc, ATH10K_TXC_S1_RATE_BW40) |
			   TXCS_MS(tx_ctrl_desc, ATH10K_TXC_S1_RATE_BW80) |
			   TXCS_MS(tx_ctrl_desc, ATH10K_TXC_S1_RATE_BW160));
	}
}

/* In AP mode, this API notifies of disassociation of a station.
 * Station specific information related to smart antenna should
 * be reset in this API.
 */
static inline void
ath10k_smart_ant_sta_disconnect(struct ath10k *ar, struct ieee80211_sta *sta)
{
	if (!ath10k_smart_ant_enabled(ar) || !ar->smart_ant_info.enabled)
		return;

	ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
		   "Smart antenna disconnect for %pM\n", sta->addr);
}

/* In AP mode, this API is to notify of association of a station. Station
 * specific information used for smart antenna may be initialized in this
 * API. Peer specific smart antenna configuration in fw may need to be
 * don from this API using ath10k_wmi_peer_cfg_smart_ant().
 */
static inline int
ath10k_smart_ant_sta_connect(struct ath10k *ar, struct ath10k_vif *arvif,
			     struct ieee80211_sta *sta)
{
	struct wmi_smart_ant_sta_cfg_arg arg;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	int i, ret;

	if (!ath10k_smart_ant_enabled(ar) || !ar->smart_ant_info.enabled)
		return 0;

	lockdep_assert_held(&ar->conf_mutex);

	if (arvif->vdev_type != WMI_VDEV_TYPE_AP ||
	    arvif->vdev_subtype != WMI_VDEV_SUBTYPE_NONE) {
		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
			   "Smart antenna logic not enabled for non-AP interface\n");
		return 0;
	}

	ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
		   "Smart antenna connect for %pM\n", sta->addr);

	for (i = 0; i < ATH10K_SMART_ANT_RTCNT_MAX; i++)
		smart_ant_dbg_ratelist(ar, &ar->ratecode_list, i);

	/* Configure to get feedback for every N PPDUs.
	 * ATH10K_TX_FEEDBACK_CONFIG_DEFAULT - b2:b0 Number of PPDUs
	 * during non-training and b5:b3 during training.
	 */
	arg.num_cfg = 1;
	arg.cfg[0] = ATH10K_TX_FEEDBACK_CONFIG_DEFAULT;
	arg.vdev_id = arsta->arvif->vdev_id;
	ether_addr_copy(arg.mac_addr.addr, sta->addr);

	ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
		   "%s mac %pM vdev_id %d num_cfg %d\n",
		   __func__, arg.mac_addr.addr, arg.vdev_id, arg.num_cfg);

	for (i = 0; i < arg.num_cfg; i++) {
		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
			   "cfg[%d] 0x%x\n", i, arg.cfg[i]);
	}

	/* Configure feedback option for this station, i.e tx feedback
	 * for how many PPDUs during training and non-training period.
	 */
	ret = ath10k_wmi_peer_cfg_smart_ant(ar, &arg);

	if (ret) {
		ath10k_warn(ar, "Failed to set feedback config\n");
		return ret;
	}
	return 0;
}

/* This API is to set initial tx/rx antennas */
static inline int
ath10k_smart_ant_set_default(struct ath10k *ar, struct ath10k_vif *arvif)
{
	return 0;
}

/* This API reverts the configurations done in ath10k_smart_ant_enable().
 * ath10k_wmi_pdev_disable_smart_ant needs to be called to disable
 * smart antenna logic in fw.
 */
static inline void
ath10k_smart_ant_disable(struct ath10k *ar, struct ath10k_vif *arvif)
{
}

/* This smart antenna API configures fw with initial smart antenna params
 * such as mode of antenna control and tx/rx antennas.
 * This API calls ath10k_wmi_pdev_enable_smart_ant() to configure initial
 * parameters for fw to start smart antenna. This API may also need to
 * enable tx feedback through packetlog.
 */
static inline int
ath10k_smart_ant_enable(struct ath10k *ar, struct ath10k_vif *arvif)
{
	return 0;
}
#endif
