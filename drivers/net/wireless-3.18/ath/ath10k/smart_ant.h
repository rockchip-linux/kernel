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

struct ath10k_peer_ratecode_list {
	u8 mac_addr[ETH_ALEN];
	u8 rtcode_legacy[WMI_CCK_OFDM_RATES_MAX];
	u8 rtcode_20[WMI_MCS_RATES_MAX];
	u8 rtcode_40[WMI_MCS_RATES_MAX];
	u8 rtcode_80[WMI_MCS_RATES_MAX];
	u8 rt_count[WMI_RATE_COUNT_MAX];
};

#define ATH10K_PPDU_SIZE_MAX		32

struct ath10k_smart_ant_info {
	u8 rx_antenna;
	bool enabled;
	u32 tx_ppdu_end[ATH10K_PPDU_SIZE_MAX];
};

#ifdef CONFIG_ATH10K_SMART_ANT_ALG
/* This API is to process rx feedback such as rssi, PER and antennas.
 * Based on the stats a better antenna combination can be found for rx.
 * Better rx antenna can be configured using ath10k_wmi_pdev_set_rx_ant().
 */
void ath10k_smart_ant_proc_rx_feedback(struct ath10k *ar,
					struct htt_rx_desc *rx_desc);

/* This API is to process tx feedback information such as tx rate
 * PER, rssi and antennas used for tx. Based on feedback stats a
 * a better antenna combination can be chosen for tx.
 * Better tx antenna can be configured using ath10k_wmi_peer_set_smart_tx_ant().
 * When needed this API can also request for feedback on packets with particular
 * antenna at a particular rate.  This is called packet training and the params
 * needed for training can be configured using
 * ath10k_wmi_peer_set_smart_ant_train_info().
 */
void ath10k_smart_ant_proc_tx_feedback(struct ath10k *ar, u8 *data);

/* In AP mode, this API notifies of disassociation of a station.
 * Station specific information related to smart antenna should
 * be reset in this API.
 */
void ath10k_smart_ant_sta_disconnect(struct ath10k *ar,
					struct ieee80211_sta *sta);

/* In AP mode, this API is to notify of association of a station. Station
 * specific information used for smart antenna may be initialized in this
 * API. Peer specific smart antenna configuration in fw may need to be
 * don from this API using ath10k_wmi_peer_cfg_smart_ant().
 */
int ath10k_smart_ant_sta_connect(struct ath10k *ar,
					struct ath10k_vif *arvif,
					struct ieee80211_sta *sta);

/* This API is to set initial tx/rx antennas */
int ath10k_smart_ant_set_default(struct ath10k *ar,
					struct ath10k_vif *arvif);

/* This API reverts the configurations done in ath10k_smart_ant_enable().
 * ath10k_wmi_pdev_disable_smart_ant needs to be called to disable
 * smart antenna logic in fw.
 */
void ath10k_smart_ant_disable(struct ath10k *ar, struct ath10k_vif *arvif);

/* This smart antenna API configures fw with initial smart antenna params
 * such as mode of antenna control and tx/rx antennas.
 * This API calls ath10k_wmi_pdev_enable_smart_ant() to configure initial
 * parameters for fw to start smart antenna. This API may also need to
 * enable tx feedback through packetlog.
 */
int ath10k_smart_ant_enable(struct ath10k *ar, struct ath10k_vif *arvif);
#else
static inline
void ath10k_smart_ant_proc_rx_feedback(struct ath10k *ar,
					struct htt_rx_desc *rx_desc)
{
}

static inline
void ath10k_smart_ant_proc_tx_feedback(struct ath10k *ar, u8 *data)
{
}

static inline
void ath10k_smart_ant_sta_disconnect(struct ath10k *ar,
					struct ieee80211_sta *sta)
{
}

static inline
int ath10k_smart_ant_sta_connect(struct ath10k *ar,
					struct ath10k_vif *arvif,
					struct ieee80211_sta *sta)
{
	return 0;
}

static inline
int ath10k_smart_ant_set_default(struct ath10k *ar,
					struct ath10k_vif *arvif)
{
	return 0;
}

static inline
void ath10k_smart_ant_disable(struct ath10k *ar, struct ath10k_vif *arvif)
{
}

static inline
int ath10k_smart_ant_enable(struct ath10k *ar, struct ath10k_vif *arvif)
{
	return 0;
}
#endif /* CONFIG_ATH10K_SMART_ANT_ALG */

#endif
