/*
 * Copyright (c) 2015, The Linux Foundation.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of  The Linux Foundation nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.

 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _SMART_ANT_H_
#define _SMART_ANT_H_

#define ATH10K_SMART_ANT_MAX_CHAINS	3

/* Bit positions 0 1 and 2 indicate number of non-training feedback
 * that can be combined
 * Bit positions 3 4 and 5 indicate number of training feedback
 * that can be combined
 * Bit position 6 indicate whether we need combined feedback or not
 * Bit position 7 indicate whether we need feedback or not
 * The default value of 0xe4 (1110 0100) means:
 *     Feedback is enabled
 *     Combined feedback is enabled
 *     Max number of combined feedback for training and non-training is 4
 */
#define ATH10K_TX_FEEDBACK_CONFIG_DEFAULT 0xe4

#define ATH10K_SMART_ANT_PKTLOG_TYPE_TXCTL	1
#define ATH10K_SMART_ANT_PKTLOG_TYPE_TXSTATS	2

#define ATH10K_SMART_ANT_RSSI_SAMPLE	10
#define ATH10K_SMART_ANT_PER_MAX	100

#define ATH10K_SMART_ANT_DEFAULT_ANT	5
#define ATH10K_PPDU_SIZE_MAX		32

/* Max number of antenna combinations 2 ^ max_supported_ant */
#define ATH10K_SMART_ANT_COMB_MAX	8

#define ATH10K_SMART_ANT_FALLBACK_RATE_DEFAULT	1

#define ATH10K_AVG_GPUT_FACTOR		7
#define ATH10K_GPUT_SHIFT		3

#define ATH10K_SMART_ANT_GOODPUT_INTVL_AVG      2
#define ATH10K_SMART_ANT_IGNORE_GOODPUT_INTVL	1
#define ATH10K_SMART_ANT_PRETRAIN_PKTS_MAX	600
#define ATH10K_SMART_ANT_BW_THRESHOLD		64

#define ATH10K_SMART_ANT_PER_MIN_THRESHOLD	20
#define ATH10K_SMART_ANT_PER_MAX_THRESHOLD	80
#define ATH10K_SMART_ANT_PER_DIFF_THRESHOLD	3
#define ATH10K_SMART_ANT_PKT_LEN_DEFAULT	1536
#define ATH10K_SMART_ANT_TPUT_DELTA_DEFAULT	10
#define ATH10K_SMART_ANT_RETRAIN_INTVL		(2 * 60000)	/* msecs */
#define ATH10K_SMART_ANT_PERF_TRAIN_INTVL	2000		/* msecs */
#define ATH10K_SMART_ANT_NUM_PKT_MIN		344
#define ATH10K_SMART_ANT_HYSTERISYS_DEFAULT	3
#define ATH10K_SMART_ANT_MIN_GOODPUT_THRESHOLD	6
#define ATH10K_SMART_ANT_NUM_TRAIN_PPDU_MAX	50
#define ATH10K_SMART_ANT_NUM_PKT_MASK		0x7fff

#define ATH10K_SMART_ANT_NUM_PKT_THRESHOLD_20	20
#define ATH10K_SMART_ANT_NUM_PKT_THRESHOLD_40	10
#define ATH10K_SMART_ANT_NUM_PKT_THRESHOLD_80	5

#define ATH10K_SMART_ANT_RATE_MASK	    0xff

#define ATH10K_SMART_ANT_TRAIN_PPDU	20
#define ATH10K_SMART_ANT_TRAIN_PKT_MAX  640
#define ATH10K_SMART_ANT_TRAIN_LGCY_PKT	200

#define ATH10K_SMART_ANT_RETRIES_MAX	8
#define ATH10K_SMART_ANT_COMB_FB_MAX	2
#define ATH10K_SMART_ANT_DYN_BW_MAX	4

#define ATH10K_TXS_TRY_SERIES_MASK	0x01000000

#define ATH10K_TXS_TRY_BW_M		0x30000000
#define ATH10K_TXS_TRY_BW_S		28

#define ATH10K_TXS_LRETRY		21
#define ATH10K_TXS_SRETRY		22
#define ATH10K_TXS_ACK_RSSI             23

#define ATH10K_TXS_TOT_TRIES		27
#define ATH10K_TXS_TOT_TRIES_M		0x1f000000
#define ATH10K_TXS_TOT_TRIES_S          24
#define ATH10K_SMART_ANT_FEEDBACK	29
#define ATH10K_SMART_ANT_FEEDBACK_2     30
#define ATH10K_SMART_ANT_GPUT		31

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

#define ATH10K_FB_RATE_POSITION		8
#define ATH10K_NFB_COMB_FB_MASK		0xf
#define ATH10K_FB_BW_MASK		0x3
#define ATH10K_COMB_FB_BW_SHIFT         4
#define ATH10K_FB_RATE_SERIES_MASK      0x1
#define ATH10K_FB_RATE_SERIES_SHIFT     2

#define ATH10K_RATE_IDX_MASK	0x0f
#define ATH10K_RATE_MODE_MASK	0xf0
#define ATH10K_NSS_SHIFT	4
#define ATH10K_NSS_MASK		0x3
#define ATH10K_RIDX_11AG_OFFSET 4

#define ATH10K_MODE_11B		0x40
#define ATH10K_MODE_11AG	0x00

#define ATH10K_RATE_20_SHIFT	0
#define ATH10K_RATE_40_SHIFT	8
#define ATH10K_RATE_80_SHIFT	16

#define ATH10K_RATE_20_MASK ATH10K_SMART_ANT_RATE_MASK
#define ATH10K_RATE_2040_MASK	0xffff

#define ATH10K_RATE_20(rate) \
	((rate >> ATH10K_RATE_20_SHIFT) & ATH10K_SMART_ANT_RATE_MASK)
#define ATH10K_RATE_40(rate) \
	((rate >> ATH10K_RATE_40_SHIFT) & ATH10K_SMART_ANT_RATE_MASK)
#define ATH10K_RATE_80(rate) \
	((rate >> ATH10K_RATE_80_SHIFT) & ATH10K_SMART_ANT_RATE_MASK)

#define ATH10K_VHT_MCS_MAX      10
#define ATH10K_HT_MCS_MAX       8

#define ATH10K_NFB_COMB_FB(bw) \
	(bw & ATH10K_NFB_COMB_FB_MASK)
#define ATH10K_COMB_FB_BW(bw) \
	((bw >> ATH10K_COMB_FB_BW_SHIFT) & ATH10K_NFB_COMB_FB_MASK)
#define ATH10K_FB_BW(ridx) (ridx & ATH10K_FB_BW_MASK)
#define ATH10K_FB_RATE(rate, bw) \
	(rate >> (bw * ATH10K_FB_RATE_POSITION))
#define ATH10K_FB_RATE_SERIES(ridx) \
	((ridx >> ATH10K_FB_RATE_SERIES_SHIFT) & ATH10K_FB_RATE_SERIES_MASK)

#define ATH10K_NSS_FROM_RATE(rate) \
	((rate >> ATH10K_NSS_SHIFT) & ATH10K_NSS_MASK)
#define ATH10K_MCS_FROM_RATE(rate) \
	(rate & ATH10K_RATE_IDX_MASK)
#define ATH10K_RATE_IDX_11AC(rate) \
	((ATH10K_NSS_FROM_RATE(rate) * ATH10K_VHT_MCS_MAX) + \
	 ATH10K_MCS_FROM_RATE(rate))
#define ATH10K_RATE_IDX_11N(rate) \
	((ATH10K_NSS_FROM_RATE(rate) * ATH10K_HT_MCS_MAX) + \
	 ATH10K_MCS_FROM_RATE(rate))

enum ath10k_smart_ant_bw {
	ATH10K_SMART_ANT_BW_20,
	ATH10K_SMART_ANT_BW_40,
	ATH10K_SMART_ANT_BW_80,
	ATH10K_SMART_ANT_BW_MAX
};

enum ath10k_wireless_mode {
	ATH10K_WIRELESS_MODE_LEGACY,
	ATH10K_WIRELESS_MODE_HT,
	ATH10K_WIRELESS_MODE_VHT,
};

enum ath10k_smart_ant_cfg_msg_type {
	ATH10K_SMART_ANT_TYPE_TX_CFG,
	ATH10K_SMART_ANT_TYPE_RX_CFG,
	ATH10K_SMART_ANT_TYPE_TRAIN_INFO,
	ATH10K_SMART_ANT_TYPE_MAX
};

enum ath10k_smart_ant_fb_update_action {
	ATH10K_SMART_ANT_ACT_RX_CFG		= 1 << 0,
	ATH10K_SMART_ANT_ACT_TRAIN		= 1 << 1,
	ATH10K_SMART_ANT_ACT_TX_CFG		= 1 << 2,
	ATH10K_SMART_ANT_ACT_TX_FB_CFG		= 1 << 3,
};
enum ath10k_smart_ant_rtcount {
	ATH10K_SMART_ANT_RTCNT_LEGACY,
	ATH10K_SMART_ANT_RTCNT_20,
	ATH10K_SMART_ANT_RTCNT_40,
	ATH10K_SMART_ANT_RTCNT_80,
	ATH10K_SMART_ANT_RTCNT_MAX,
};

enum ath10k_smart_ant_train_trigger {
	ATH10K_SMART_ANT_TRAIN_TRIGGER_PERIODIC = 1 << 0,
	ATH10K_SMART_ANT_TRAIN_TRIGGER_PERF	= 1 << 1,
	ATH10K_SMART_ANT_TRAIN_TRIGGER_RX	= 1 << 2,
};

enum ath10k_smart_ant_state {
	ATH10K_SMART_ANT_STATE_INIT,
	ATH10K_SMART_ANT_STATE_PRETRAIN,
	ATH10K_SMART_ANT_STATE_TRAIN_PROGRESS,
};

enum ath10k_smart_ant_trigger_type {
	ATH10K_SMART_ANT_TRIGGER_TYPE_INIT,
	ATH10K_SMART_ANT_TRIGGER_TYPE_PVE,
	ATH10K_SMART_ANT_TRIGGER_TYPE_NVE,
};


enum ath10k_smart_ant_feedback {
	ATH10K_SMART_ANT_TX_FEEDBACK = 1 << 0,
	ATH10K_SMART_ANT_RX_FEEDBACK = 1 << 1,
};

enum ath10k_smart_ant_debug_level {
	ATH10K_SMART_ANT_DBG_LVL_TOP_DECISION,
	ATH10K_SMART_ANT_DBG_LVL_TRAIN_STAGES,
	ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATES,
	ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATS,
	ATH10K_SMART_ANT_DBG_LVL_ALL,
};

struct ath10k_smart_ant_wmi_cfg_param {
	struct list_head list;
	enum ath10k_smart_ant_cfg_msg_type type;
	union {
		struct {
			u32 rx_ant;
		} rx_ant_cfg;

		struct {
			u8 peer_mac[ETH_ALEN];
			int vdev_id;
			u32 tx_ants[WMI_SMART_ANT_RATE_SERIES_MAX];
		} tx_ant_cfg;

		struct {
			u8 peer_mac[ETH_ALEN];
			int vdev_id;
			struct wmi_peer_sant_set_train_arg train_info;
		} train_info_cfg;
	};
};

struct ath10k_smart_ant_perf_info {
	enum ath10k_smart_ant_trigger_type trig_type;
	u8 hysteresis;
	u8 gput_avg_intvl;
	u32 avg_gput;
};

struct ath10k_smart_ant_train_stats {
	u32 rate;
	u8 antenna;
	u16 nbad;
	u16 next_ant_nbad;
	u16 nframes;
	u16 next_ant_nframes;
	u8 per;
	u8 next_ant_per;
	u8 bw;
	u8 rssi[ATH10K_SMART_ANT_MAX_CHAINS][ATH10K_SMART_ANT_RSSI_SAMPLE];
	u8 nxt_rssi[ATH10K_SMART_ANT_MAX_CHAINS][ATH10K_SMART_ANT_RSSI_SAMPLE];
	bool first_per;
	bool bw_change;
	int last_train_dir;
	u32 last_rate;
	u32 ant_map[ATH10K_SMART_ANT_COMB_MAX];
	u32 skip_mask;
};

struct ath10k_smart_ant_train_info {
	u8 train_bitmap;
	enum ath10k_smart_ant_state train_state;
	bool intense_train;
	u8 feedback_cfg;
	u32 sel_ant;
	u32 prev_sel_ant;
	u32 prev_rate_max;
	u32 num_tx_pkts;
	enum ath10k_smart_ant_bw last_bw;
	u8 retrain_misses;
	u8 long_retrain_cnt;
	u8 no_traffic_cnt;
	struct ath10k_smart_ant_perf_info perf_info[ATH10K_SMART_ANT_BW_MAX];
	u32 perf_mon_slot;
	u32 train_start_ts;
	u32 train_end_ts;
	struct ath10k_smart_ant_train_stats train_stats;
	bool train_start;
	bool ant_change_ind;
	u32 num_ppdu_bw[ATH10K_SMART_ANT_BW_MAX];
	u32 num_ppdu;
};

struct ath10k_smart_ant_train_data {
	u8 antenna;
	u32 rate_code;
	u16 nframes;
	u16 nbad;
	u8 rssi[ATH10K_SMART_ANT_MAX_CHAINS][ATH10K_SMART_ANT_RSSI_SAMPLE];
	u16 last_nframes;
	u16 num_pkts;
	u8 samples;
};

struct ath10k_smart_ant_rate_stats {
	u32 npkts_legacy[WMI_CCK_OFDM_RATES_MAX];
	u32 npkts_mcs[ATH10K_SMART_ANT_BW_MAX][WMI_MCS_RATES_MAX];
	u32 nppdu_bw[ATH10K_SMART_ANT_BW_MAX];
	u32 nppdu_gput[ATH10K_SMART_ANT_BW_MAX];
	u32 ins_gput[ATH10K_SMART_ANT_BW_MAX];
	u32 last_rate_mcs;
	u32 last_rate_max_phy;
};

struct ath10k_peer_ratecode_list {
	u8 mac_addr[ETH_ALEN];
	u8 rtcode_legacy[WMI_CCK_OFDM_RATES_MAX];
	u8 rtcode_20[WMI_MCS_RATES_MAX];
	u8 rtcode_40[WMI_MCS_RATES_MAX];
	u8 rtcode_80[WMI_MCS_RATES_MAX];
	u8 rt_count[WMI_RATE_COUNT_MAX];
};

struct ath10k_smart_ant_sta {
	struct ath10k *ar;
	u8 mac_addr[ETH_ALEN];
	struct ath10k_peer_ratecode_list rate_cap;
	struct ath10k_smart_ant_train_info train_info;
	struct ath10k_smart_ant_rate_stats rate_stats;
	struct ath10k_smart_ant_train_data train_data;
	enum ath10k_wireless_mode wmode;
	struct work_struct sa_wmi_cfg_work;
	struct list_head cfg_list;
	/* Lock to protect cfg_list */
	spinlock_t cfg_lock;
};

struct ath10k_smart_ant_comb_fb {
	u8 nbad;
	u8 npkts;
	u8 bw;
	u8 rate;
} __packed;

struct ath10k_smart_ant_tx_fb {
	u16 npkts;
	u16 nbad;
	u16 nshort_retries[WMI_SMART_ANT_RATE_SERIES_MAX];
	u16 nlong_retries[WMI_SMART_ANT_RATE_SERIES_MAX];
	u32 tx_antenna[WMI_SMART_ANT_RATE_SERIES_MAX];
	u32 rssi[ATH10K_SMART_ANT_MAX_CHAINS];
	u32 rate_mcs[WMI_SMART_ANT_RATE_SERIES_MAX];
	bool train_pkt;
	u32 rate_maxphy;
	u32 gput;
	u8 ridx;
	u8 num_comb_fb;
	struct ath10k_smart_ant_comb_fb comb_fb[ATH10K_SMART_ANT_COMB_FB_MAX];
};

struct ath10k_smart_ant_params {
	u8 low_rate_threshold;
	u8 hi_rate_threshold;
	u8 per_diff_threshold;
	u16 num_train_pkts;
	u16 pkt_len;
	u8 num_tx_ant_comb;
	u16 num_min_pkt;
	u32 retrain_interval;
	u32 perf_train_interval;
	u8 max_perf_delta;
	u8 hysteresis;
	u8 min_goodput_threshold;
	u8 avg_goodput_interval;
	u8 ignore_goodput_interval;
	u16 num_pretrain_pkts;
	u16 num_other_bw_pkts_threshold;
	u8 enabled_train;
	u16 num_pkt_min_threshod[ATH10K_SMART_ANT_BW_MAX];
	u32 default_tx_ant;
	u8 ant_change_ind;
	u16 max_train_ppdu;
};

struct ath10k_smart_ant_info {
	struct ath10k_smart_ant_params smart_ant_params;
	u32 enabled_feedback;
	u8 mode;
	u8 default_ant;
	u8 num_fallback_rate;
	u8 num_sta_per_ant[ATH10K_SMART_ANT_COMB_MAX];
	u16 num_sta_conneted;
	u8 rx_antenna;
	bool enabled;
	u32 tx_ppdu_end[ATH10K_PPDU_SIZE_MAX];
	u32 num_enabled_vif;
	u8 debug_level;
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

#endif /* _SMART_ANT_H_ */
