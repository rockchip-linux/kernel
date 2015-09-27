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

#include "debug.h"
#include "txrx.h"
#include "wmi-ops.h"
#include "smart_ant.h"

static char *rate_code_map[] = {"Legacy CCK/OFDM",
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
	if (ar->smart_ant_info.debug_level >=
		ATH10K_SMART_ANT_DBG_LVL_TRAIN_STAGES) {
		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
			"rate code list for mode %s\n", rate_code_map[mode]);
		for (i = 0; i < rcount; i++)
			ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
					"0x%x\n", rlist[i]);
	}
}

static void smart_ant_dbg_feedback(struct ath10k *ar,
					struct ath10k_smart_ant_tx_fb *fb)
{
	int i;

	if (fb->num_comb_fb) {
		for (i = 0; i < fb->num_comb_fb; i++) {
			if (ar->smart_ant_info.debug_level >=
				ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATS) {
				ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
				"Combined feedback[%d] npkts %d nbad %d bw %d rate 0x%x num_comb_fb %d"
				" Train pkt: %s rate_maxphy 0x%x rate_idx %d"
				" goodput %d tx_antenna[0]: %d tx_antenna[1]: %d"
				" rate_mcs[0] 0x%x rate_mcs[1] 0x%x\n",
				i, fb->comb_fb[i].npkts, fb->comb_fb[i].nbad,
				ATH10K_COMB_FB_BW(fb->comb_fb[i].bw),
				fb->comb_fb[i].rate, fb->num_comb_fb,
				fb->train_pkt ? "True" : "False",
				fb->rate_maxphy, fb->ridx, fb->gput,
				fb->tx_antenna[0], fb->tx_antenna[0],
				fb->rate_mcs[0], fb->rate_mcs[1]);
			}
		}
	} else {
		if (ar->smart_ant_info.debug_level >=
			ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATS) {
			ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
			"Regular feedback npkts %d nbad %d bw %d rate 0x%x num_comb_fb %d"
			" Train pkt: %s rate_maxphy 0x%x rate_idx %d goodput"
			" %d tx_antenna[0]: %d tx_antenna[1]: %d"
			" rate_mcs[0] 0x%x rate_mcs[1] 0x%x\n",
			fb->npkts, fb->nbad, ATH10K_FB_BW(fb->ridx),
			ATH10K_FB_RATE(fb->rate_mcs[0], ATH10K_FB_BW(fb->ridx)),
			fb->num_comb_fb, fb->train_pkt ? "True" : "False",
			fb->rate_maxphy,
			fb->ridx, fb->gput,
			fb->tx_antenna[0], fb->tx_antenna[0],
			fb->rate_mcs[0], fb->rate_mcs[1]);
		}
	}

	if (ar->smart_ant_info.debug_level >=
		ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATS) {
		for (i = 0; i < ATH10K_SMART_ANT_MAX_CHAINS; i++) {
			ath10k_dbg(ar, ATH10K_DBG_SMART_ANT, "rssi[%d] %d\n",
				i, (u8)fb->rssi[i]);
		}
	}
}

static void smart_ant_update_antmap(struct ath10k *ar,
				struct ath10k_smart_ant_sta *sant_sta,
				u8 sel_ant)
{
	struct ath10k_smart_ant_params *sparams =
				&ar->smart_ant_info.smart_ant_params;
	int i;
	u8 tmp_ant;

	for (i = 0; i < sparams->num_tx_ant_comb; i++) {
		if (sant_sta->train_info.train_stats.ant_map[i] == sel_ant)
			break;
	}

	if (i == sparams->num_tx_ant_comb)
		return;

	tmp_ant =  sant_sta->train_info.train_stats.ant_map[0];
	sant_sta->train_info.train_stats.ant_map[0] = sel_ant;
	sant_sta->train_info.train_stats.ant_map[i] = tmp_ant;
}

static void smart_ant_get_tx_ant(struct ath10k_smart_ant_sta *sa_sta,
					u32 *tx_ants)
{
	struct ath10k_smart_ant_train_info *tinfo = &sa_sta->train_info;
	int i;

	for (i = 0; i < WMI_SMART_ANT_RATE_SERIES_MAX; i++)
		tx_ants[i] = tinfo->sel_ant;
}

static u32 ath10k_sant_get_rate(u8 rate_code, u8 mode, bool is_vht,
				enum ath10k_smart_ant_bw bw, int rt_indx)
{
	struct rate_info rinfo;

	if (mode == ATH10K_MODE_11B ||
		mode == ATH10K_MODE_11AG) {
		return mode == ATH10K_MODE_11B ?
			rate_code & ATH10K_RATE_IDX_MASK :
			((rate_code & ATH10K_RATE_IDX_MASK) +
			ATH10K_RIDX_11AG_OFFSET);
	} else {
		memset(&rinfo, 0, sizeof(rinfo));
		rinfo.flags |= is_vht ? RATE_INFO_FLAGS_VHT_MCS :
					RATE_INFO_FLAGS_MCS;
		rinfo.flags |= RATE_INFO_FLAGS_SHORT_GI;
		if (bw == ATH10K_SMART_ANT_BW_80)
			rinfo.bw = RATE_INFO_BW_80;
		else if (bw == ATH10K_SMART_ANT_BW_40)
			rinfo.bw = RATE_INFO_BW_40;
		else
			rinfo.bw = RATE_INFO_BW_20;
		if (is_vht) {
			rinfo.mcs = ATH10K_MCS_FROM_RATE(rate_code);
			rinfo.nss = ATH10K_NSS_FROM_RATE(rate_code);
		} else
			rinfo.mcs = rt_indx;
	}

	return cfg80211_calculate_bitrate(&rinfo);
}

static bool ath10k_sant_is_legacy_rate(u8 rate)
{
	return ((rate & ATH10K_RATE_MODE_MASK) == ATH10K_MODE_11B) ||
		((rate & ATH10K_RATE_MODE_MASK) == ATH10K_MODE_11AG);
}

static u8 ath10k_smart_ant_get_ridx(u8 rate)
{
	u8 ridx = rate & ATH10K_RATE_IDX_MASK;

	switch (rate & ATH10K_RATE_MODE_MASK) {
	case ATH10K_MODE_11B:
		return ridx;
	case ATH10K_MODE_11AG:
		ridx += ATH10K_RIDX_11AG_OFFSET;
		return ridx;
	default:
		ridx += ATH10K_NSS_FROM_RATE(rate) * 10;
		return ridx;
	}
}

static void smart_ant_get_rate_tbl(struct ath10k_smart_ant_sta *sa_sta,
					u8 **rt_tble, u8 *size)
{
	struct ath10k_peer_ratecode_list *rcap = &sa_sta->rate_cap;

	switch (sa_sta->train_info.train_stats.bw) {
	case ATH10K_SMART_ANT_BW_20:
		if (sa_sta->wmode == ATH10K_WIRELESS_MODE_LEGACY) {
			*size = rcap->rt_count[ATH10K_SMART_ANT_RTCNT_LEGACY];
			*rt_tble = rcap->rtcode_legacy;
		} else {
			*size = rcap->rt_count[ATH10K_SMART_ANT_RTCNT_20];
			*rt_tble = rcap->rtcode_20;
		}
		break;
	case ATH10K_SMART_ANT_BW_40:
		*size = rcap->rt_count[ATH10K_SMART_ANT_RTCNT_40];
		*rt_tble = rcap->rtcode_40;
		break;
	case ATH10K_SMART_ANT_BW_80:
	default:
		*size = rcap->rt_count[ATH10K_SMART_ANT_RTCNT_80];
		*rt_tble = rcap->rtcode_80;
	}
}

static u8 smart_ant_get_ridx_max(struct ath10k_smart_ant_sta *sa_sta)
{
	u8 *rt_tble = NULL;
	u8 num_rates = 0;

	smart_ant_get_rate_tbl(sa_sta, &rt_tble, &num_rates);
	if (num_rates)
		return rt_tble[num_rates - 1];

	return 0;
}

static enum ath10k_smart_ant_bw smart_ant_bw_used_most(
			struct ath10k_smart_ant_train_info *tinfo,
			u32 *nppdu_bw,
			enum ath10k_smart_ant_bw bw)
{
	enum ath10k_smart_ant_bw bw_used = bw;
	u32 more_ppdus_bw = nppdu_bw[bw_used];
	int i;
	bool bw_trained = false;

	for (i = bw - 1; i >= ATH10K_SMART_ANT_BW_20; i--) {
		if (!nppdu_bw[i])
			continue;

		bw_trained = true;
		if (nppdu_bw[i] > more_ppdus_bw) {
			more_ppdus_bw = nppdu_bw[i];
			bw_used = i;
		}
	}

	if (!bw_trained && !nppdu_bw[bw])
		bw_used = tinfo->last_bw;

	return bw_used;
}

static void smart_ant_set_train_params(struct ath10k *ar,
					struct ath10k_smart_ant_sta *sa_sta,
					u8 ant_map)
{
	struct ath10k_smart_ant_train_data *tdata = &sa_sta->train_data;
	struct ath10k_smart_ant_train_stats *tstats =
					&sa_sta->train_info.train_stats;
	u32 rate_mcs = sa_sta->rate_stats.last_rate_mcs;
	u32 rate_code, bitrate;
	u16 num_pkts;
	int i;
	u8 *rt_tble = NULL;
	u8 num_rates = 0;

	rate_code = tstats->rate;
	tdata->antenna = ant_map;

	switch (tstats->bw) {
	case ATH10K_SMART_ANT_BW_20:
		tdata->rate_code = rate_code;
		break;
	case ATH10K_SMART_ANT_BW_40:
		tdata->rate_code = (rate_code << ATH10K_RATE_40_SHIFT) |
					(rate_mcs & ATH10K_RATE_20_MASK);
		break;
	case ATH10K_SMART_ANT_BW_80:
		tdata->rate_code = (rate_code << ATH10K_RATE_80_SHIFT) |
					(rate_mcs & ATH10K_RATE_2040_MASK);
		break;
	}

	if (sa_sta->wmode == ATH10K_WIRELESS_MODE_LEGACY) {
		tdata->num_pkts = ATH10K_SMART_ANT_TRAIN_LGCY_PKT;
		return;
	}

	smart_ant_get_rate_tbl(sa_sta, &rt_tble, &num_rates);
	for (i = 0; i < num_rates; i++) {
		if (rt_tble[i] == rate_code)
			break;
	}

	bitrate = ath10k_sant_get_rate(rate_code,
				rate_code & ATH10K_RATE_MODE_MASK,
				sa_sta->wmode == ATH10K_WIRELESS_MODE_VHT,
				tstats->bw, i);

	/* 1.5KB sized packets per sec */
	num_pkts = (bitrate * 1000) / 12;

	/* Num packets for 4ms dur * Num ppdus for training */
	num_pkts = ((4 * num_pkts) / 1000) * ATH10K_SMART_ANT_TRAIN_PPDU;

	/*OW: this change is suggested by repo upload script*/
	/*num_pkts = min(num_pkts, (u16) ATH10K_SMART_ANT_TRAIN_PKT_MAX);*/
	min_t(u16, num_pkts, ATH10K_SMART_ANT_TRAIN_PKT_MAX);

	tdata->num_pkts = num_pkts;
	if (ar->smart_ant_info.debug_level >=
		ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATES) {
		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
		"set_train_params tx_antenna %d rate %x "
		"nFrames %d num_pkts %d\n",
		tdata->antenna, tdata->rate_code,
		tdata->nframes, tdata->num_pkts);
	}
}

static u8 ath10k_smart_ant_get_train_rate(struct ath10k_smart_ant_sta *sa_sta)
{
	struct ath10k_smart_ant_train_info *tinfo = &sa_sta->train_info;
	enum ath10k_smart_ant_bw bw;
	u32 *rstats;
	u32 max_pkts_sent = 0;
	u8 *rt_tble = NULL;
	u8 num_rates = 0, ridx, max_ridx = 0;
	int i;
	bool pkts_sent = false;

	bw = smart_ant_bw_used_most(tinfo, sa_sta->rate_stats.nppdu_bw,
					ATH10K_SMART_ANT_BW_MAX - 1);
	tinfo->train_stats.bw = bw;
	tinfo->last_bw = bw;
	rstats = sa_sta->wmode == ATH10K_WIRELESS_MODE_LEGACY ?
		sa_sta->rate_stats.npkts_legacy :
		sa_sta->rate_stats.npkts_mcs[bw];
	smart_ant_get_rate_tbl(sa_sta, &rt_tble, &num_rates);
	for (i = 0; i < num_rates; i++) {
		ridx = ath10k_smart_ant_get_ridx(rt_tble[i]);
		if (rstats[ridx])
			pkts_sent = true;

		if (rstats[ridx] >= max_pkts_sent) {
			max_pkts_sent = rstats[ridx];
			max_ridx = i;
		}
	}

	/* reset rate stats */
	memset(&sa_sta->rate_stats, 0, sizeof(sa_sta->rate_stats));

	/* No pkts sent during this period */
	if (!pkts_sent)
		return smart_ant_get_ridx_max(sa_sta);

	return rt_tble[max_ridx];
}

static void
ath10k_smart_ant_init_train_info(struct ath10k_smart_ant_train_info *tinfo,
				u8 rate_code)
{
	struct ath10k_smart_ant_train_stats *tstats = &tinfo->train_stats;
	if (tinfo->train_state == ATH10K_SMART_ANT_STATE_PRETRAIN)
		tstats->skip_mask = 0;
	else if (tinfo->train_state == ATH10K_SMART_ANT_STATE_TRAIN_PROGRESS &&
		 tstats->bw_change)
		tstats->skip_mask &= ~1;

	tinfo->train_stats.antenna = 0;
	tinfo->train_stats.rate = rate_code;
	tinfo->intense_train = true;
	tinfo->train_stats.bw_change = false;
	tinfo->train_stats.first_per = true;
	tinfo->train_stats.last_train_dir = 0;
	tinfo->train_stats.last_rate = 0;
}

static u8 ath10k_sant_get_rate_maxphy(struct ath10k_smart_ant_sta *sa_sta)
{
	u32 rate_maxphy = sa_sta->rate_stats.last_rate_max_phy;

	switch (sa_sta->train_info.train_stats.bw) {
	case ATH10K_SMART_ANT_BW_20:
		return ATH10K_RATE_20(rate_maxphy);
	case ATH10K_SMART_ANT_BW_40:
		return ATH10K_RATE_40(rate_maxphy);
	case ATH10K_SMART_ANT_BW_80:
		return ATH10K_RATE_80(rate_maxphy);
	default:
		return rate_maxphy & ATH10K_SMART_ANT_RATE_MASK;
	}
}

static void
ath10k_sant_fill_train_rate_ant(struct ath10k_smart_ant_sta *sa_sta,
				struct wmi_peer_sant_set_train_arg *train_info)
{
	struct ath10k_smart_ant_train_data *tdata = &sa_sta->train_data;
	int i;

	for (i = 0; i < WMI_SMART_ANT_RATE_SERIES_MAX; i++) {
		train_info->rates[i] = tdata->rate_code;
		train_info->antennas[i] = tdata->antenna;
	}

	train_info->num_pkts = tdata->num_pkts | ATH10K_SMART_ANT_NUM_PKT_MASK;
}

static void ath10k_sant_reset_train_data(struct ath10k_smart_ant_sta *sa_sta)
{
	struct ath10k_smart_ant_train_info *tinfo = &sa_sta->train_info;
	struct ath10k_smart_ant_train_data *tdata = &sa_sta->train_data;

	tdata->nbad = 0;
	tdata->nframes = 0;
	tdata->samples = 0;
	tinfo->num_ppdu = 0;
	memset(tdata->rssi, 0, sizeof(tdata->rssi));
	memset(tinfo->num_ppdu_bw, 0, sizeof(tinfo->num_ppdu_bw));
}

static void smart_ant_get_train_info(struct ath10k *ar,
				struct ath10k_smart_ant_sta *sa_sta,
				struct wmi_peer_sant_set_train_arg *train_info)
{
	struct ath10k_smart_ant_train_info *tinfo = &sa_sta->train_info;
	int i;

	switch (tinfo->train_state) {
	case ATH10K_SMART_ANT_STATE_PRETRAIN:
		ath10k_smart_ant_init_train_info(tinfo,
				ath10k_smart_ant_get_train_rate(sa_sta));
		tinfo->train_state = ATH10K_SMART_ANT_STATE_TRAIN_PROGRESS;
		tinfo->train_start_ts = jiffies;

		/* Reset perf info */
		for (i = 0; i < ATH10K_SMART_ANT_BW_MAX; i++) {
			memset(&tinfo->perf_info[i], 0,
				sizeof(struct ath10k_smart_ant_perf_info));
		}

		smart_ant_set_train_params(ar, sa_sta,
						tinfo->train_stats.ant_map[0]);
		break;
	case ATH10K_SMART_ANT_STATE_TRAIN_PROGRESS:
		if (tinfo->train_stats.bw_change) {
			/* Bandwidth change, get the max_phy rate for the bw */
			ath10k_smart_ant_init_train_info(tinfo,
					ath10k_sant_get_rate_maxphy(sa_sta));
			smart_ant_set_train_params(ar, sa_sta,
						tinfo->train_stats.ant_map[0]);
		}
		break;
	case ATH10K_SMART_ANT_STATE_INIT:
		sa_sta->train_data.antenna = tinfo->sel_ant;
		sa_sta->train_data.num_pkts = 0;
		sa_sta->train_data.rate_code = sa_sta->rate_stats.last_rate_mcs;
		break;
	}

	ath10k_sant_fill_train_rate_ant(sa_sta, train_info);
	ath10k_sant_reset_train_data(sa_sta);
}

static void smart_ant_update_rates(struct ath10k *ar,
				struct ath10k_smart_ant_sta *sa_sta,
				struct ath10k_smart_ant_tx_fb *fb)
{
	struct ath10k_smart_ant_comb_fb *comb_fb;
	struct ath10k_smart_ant_rate_stats *rstats = &sa_sta->rate_stats;
	u8 nfb, rate, bw, rseries, ridx;
	u16 npkts;
	bool legacy;
	int i;

	if (fb->train_pkt)
		return;

	for (i = fb->num_comb_fb; i >= 0; i--) {
		comb_fb = i ? &fb->comb_fb[i - 1] : NULL;
		nfb = comb_fb ? ATH10K_NFB_COMB_FB(comb_fb->bw) : 1;
		bw = comb_fb ? ATH10K_COMB_FB_BW(comb_fb->bw) :
				ATH10K_FB_BW(fb->ridx);
		if (!comb_fb)
			rseries = ATH10K_FB_RATE_SERIES(fb->ridx);
		rate = comb_fb ? comb_fb->rate :
				ATH10K_FB_RATE(fb->rate_mcs[rseries], bw);
		npkts = comb_fb ? comb_fb->npkts : fb->npkts;

		ridx = ath10k_smart_ant_get_ridx(rate);
		legacy = ath10k_sant_is_legacy_rate(rate);
		if (legacy)
			rstats->npkts_legacy[ridx] += npkts;
		else
			rstats->npkts_mcs[bw][ridx] += npkts;
		rstats->nppdu_bw[bw] += nfb;
	}

	rstats->ins_gput[bw] = (rstats->ins_gput[bw] * rstats->nppdu_bw[bw] +
				fb->gput) / (rstats->nppdu_gput[bw] + 1);
	rstats->nppdu_gput[bw]++;
	rstats->last_rate_mcs = fb->rate_mcs[0];
	if (ATH10K_RATE_20(fb->rate_maxphy) &&
		ATH10K_RATE_40(fb->rate_maxphy) &&
		ATH10K_RATE_80(fb->rate_maxphy))
			rstats->last_rate_max_phy = fb->rate_maxphy;
}

static void smart_ant_change_bw(struct ath10k *ar,
					struct ath10k_smart_ant_sta *sa_sta)
{
	struct ath10k_smart_ant_train_info *tinfo = &sa_sta->train_info;
	enum ath10k_smart_ant_bw bw = ATH10K_SMART_ANT_BW_20;

	if (tinfo->train_stats.bw != ATH10K_SMART_ANT_BW_20)
		bw = smart_ant_bw_used_most(tinfo, tinfo->num_ppdu_bw,
						tinfo->train_stats.bw);
	if (bw != tinfo->train_stats.bw) {
		tinfo->train_stats.bw = bw;
		tinfo->train_stats.bw_change = true;
	}
}

static u8 smart_ant_sel_rx_ant(struct ath10k *ar,
					struct ath10k_smart_ant_sta *sa_sta)
{
	struct ath10k_smart_ant_train_info *tinfo = &sa_sta->train_info;
	struct ath10k_smart_ant_info *sa_info = &ar->smart_ant_info;
	u16 nsta_sel_ant;
	u8 status = 0;

	if (tinfo->prev_sel_ant == tinfo->sel_ant)
		return status;

	sa_info->num_sta_per_ant[tinfo->prev_sel_ant]--;
	sa_info->num_sta_per_ant[tinfo->sel_ant]++;
	nsta_sel_ant = sa_info->num_sta_per_ant[tinfo->sel_ant];

	if (ar->smart_ant_info.debug_level >=
		ATH10K_SMART_ANT_DBG_LVL_TRAIN_STAGES) {
		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
		"sa_info->default_ant = %d, tinfo->sel_ant= %d\n"
		"sa_info->num_sta_conneted = %d, nsta_sel_ant = %d\n",
		sa_info->default_ant, tinfo->sel_ant,
		sa_info->num_sta_conneted, nsta_sel_ant);
	}

	/* When all stations select the same tx_ant, rx_ant follows tx_ant.
	* Otherwise, rx_ant is set to be default_ant.
	*/
	if (sa_info->num_sta_conneted == nsta_sel_ant)
		sa_info->rx_ant = tinfo->sel_ant;
	else
		sa_info->rx_ant = sa_info->default_ant;

	status = ATH10K_SMART_ANT_ACT_RX_CFG;

	if (ar->smart_ant_info.debug_level >=
		ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATES) {
		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
		"Rx antenna is going to be config to %d\n",
		sa_info->rx_ant);
	}

	tinfo->prev_sel_ant = tinfo->sel_ant;
	return status;
}

static s32 smart_ant_move_rate(struct ath10k_smart_ant_sta *sa_sta,
					int rate_dir)
{
	u8 *rt_tble = NULL;
	u8 num_rates = 0;
	u16 rate_code = sa_sta->train_info.train_stats.rate;
	u32 cur_rate, nxt_rate;
	enum ath10k_smart_ant_bw bw =
			sa_sta->train_info.train_stats.bw;
	int i;

	if (!rate_dir)
		return -1;

	smart_ant_get_rate_tbl(sa_sta, &rt_tble, &num_rates);
	for (i = 0; i < num_rates; i++) {
		if (rt_tble[i] == rate_code)
			break;
	}

	cur_rate = ath10k_sant_get_rate(rate_code,
				rate_code & ATH10K_RATE_MODE_MASK,
				sa_sta->wmode == ATH10K_WIRELESS_MODE_VHT,
				bw, i);

	if (rate_dir > 0) {
		while (i++ < num_rates - 1) {
			nxt_rate = ath10k_sant_get_rate(rt_tble[i],
				rate_code & ATH10K_RATE_MODE_MASK,
				sa_sta->wmode == ATH10K_WIRELESS_MODE_VHT,
				bw, i);
			if (nxt_rate > cur_rate)
				return rt_tble[i];
		}
	} else {
		while (i--) {
			nxt_rate = ath10k_sant_get_rate(rt_tble[i],
				rate_code & ATH10K_RATE_IDX_MASK,
				sa_sta->wmode == ATH10K_WIRELESS_MODE_VHT,
				bw, i);
			if (nxt_rate < cur_rate)
				return rt_tble[i];
		}
	}

	return -1;
}

static bool smart_ant_sec_metric(struct ath10k_smart_ant_sta *sa_sta)
{
	struct ath10k_smart_ant_train_data *tdata = &sa_sta->train_data;
	struct ath10k_smart_ant_train_stats *tstats =
					&sa_sta->train_info.train_stats;
	u8 cmin_rssi, cmax_rssi, tmin_rssi, tmax_rssi;
	u32 cgoodant_cnt = 0, tgood_ant_cnt = 0;
	int i, j;

	for (i = 0; i < ATH10K_SMART_ANT_RSSI_SAMPLE; i++) {
		cmin_rssi = cmax_rssi = tdata->rssi[0][i];
		tmin_rssi = tmax_rssi = tstats->rssi[0][i];
		for (j = 0; j < ATH10K_SMART_ANT_MAX_CHAINS; j++) {
			cmin_rssi = (tdata->rssi[j][i] >= 0 &&
					cmin_rssi > tdata->rssi[j][i]) ?
					tdata->rssi[j][i] : cmin_rssi;
			cmax_rssi = (tdata->rssi[j][i] >= 0 &&
					cmax_rssi < tdata->rssi[j][i]) ?
					tdata->rssi[j][i] : cmax_rssi;
			tmin_rssi = (tstats->rssi[j][i] >= 0 &&
					tmin_rssi > tstats->rssi[j][i]) ?
					tstats->rssi[j][i] : tmin_rssi;
			tmax_rssi = (tstats->rssi[j][i] >= 0 &&
					tmax_rssi < tstats->rssi[j][i]) ?
					tstats->rssi[j][i] : tmax_rssi;
		}

		if (tmin_rssi == cmin_rssi) {
			if (cmax_rssi == tmax_rssi)
				continue;
			if (cmax_rssi > tmax_rssi)
				cgoodant_cnt++;
			else
				tgood_ant_cnt++;
		} else if (tmin_rssi > cmin_rssi) {
			tgood_ant_cnt++;
		} else {
			cgoodant_cnt++;
		}
	}

	return cgoodant_cnt > tgood_ant_cnt ? true : false;
}

static u8 smart_ant_proc_train_stats(struct ath10k *ar,
					struct ath10k_smart_ant_sta *sa_sta)
{
	struct ath10k_smart_ant_params *sparams =
					&ar->smart_ant_info.smart_ant_params;
	struct ath10k_smart_ant_train_data *tdata = &sa_sta->train_data;
	struct ath10k_smart_ant_train_info *tinfo = &sa_sta->train_info;
	struct ath10k_smart_ant_train_stats *tstats = &tinfo->train_stats;
	int nxt_rate_dir = 0;
	u32 per = ATH10K_SMART_ANT_PER_MAX, per_diff = 0;
	bool switch_ant = false;
	u32 status = 0;
	u8 nxt_antenna = 1;

	if (tdata->nframes)
		per = (ATH10K_SMART_ANT_PER_MAX * tdata->nbad) / tdata->nframes;

	if (ar->smart_ant_info.debug_level >=
		ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATS) {
		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
		"Stats for antenna %d: rate %x,"
		"nFrames: %d nBad %d, nppdu %d, RSSI %d %d %d PER: %d ",
		tdata->antenna, tdata->rate_code, tdata->nframes,
		tdata->nbad, tinfo->num_ppdu_bw[tstats->bw], tdata->rssi[0][0],
		tdata->rssi[1][0], tdata->rssi[2][0], per);
	}

	/* TODO: Process extra stats */
	if (tinfo->sel_ant == tstats->ant_map[tstats->antenna]) {
		if (per <= sparams->hi_rate_threshold ||
			tstats->last_train_dir < 0 || tstats->first_per) {
			tstats->per = per;
			tstats->nbad = tdata->nbad;
			tstats->nframes = tdata->nframes;
			memcpy(tstats->rssi, tdata->rssi, sizeof(tdata->rssi));
		}

		/* Currently selected antenna */
		if (tstats->first_per) {
			tstats->first_per = false;
			tstats->last_rate = tstats->rate;
			/* Move training rates up/down based on per */
			if (per > sparams->hi_rate_threshold)
				nxt_rate_dir = -1;
			else if (per < sparams->low_rate_threshold)
				nxt_rate_dir = 1;
		} else if (per < sparams->low_rate_threshold) {
			nxt_rate_dir = tstats->last_train_dir > 0 ? 1 : 0;
			if (tstats->last_train_dir <= 0)
				tstats->last_rate = tstats->rate;
		} else if (per < sparams->hi_rate_threshold) {
			tstats->last_rate = tstats->rate;
		} else {
			if (tstats->last_train_dir < 0)
				nxt_rate_dir = -1;
			else
				tstats->rate = tstats->last_rate;
		}
	} else { /* Update from other antennas */
		if (per < sparams->low_rate_threshold &&
			smart_ant_get_ridx_max(sa_sta) != tstats->rate) {
			nxt_antenna = 0;
			nxt_rate_dir = 1;
			per_diff = per < tstats->per ? tstats->per - per : 0;
			if ((tstats->rate > tstats->last_rate) ||
				((tstats->rate == tstats->last_rate) &&
				per_diff > sparams->per_diff_threshold &&
				tstats->per > sparams->low_rate_threshold)){
				switch_ant = true;

				if (ar->smart_ant_info.debug_level >=
					ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATS) {
					ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
					"switch ant on higher rate: curr rate %d "
					"last rate %d "
					"per_diff %d per_diff_threshold %d "
					"stat PER %d low_rate_threshold %d\n",
					tstats->rate, tstats->last_rate,
					per_diff, sparams->per_diff_threshold,
					tstats->per,
					sparams->low_rate_threshold);
				}
			}

			if (!switch_ant && tstats->rate == tstats->last_rate) {
				tstats->next_ant_per = per;
				tstats->next_ant_nbad = tdata->nbad;
				tstats->next_ant_nframes = tdata->nframes;
				memcpy(tstats->nxt_rssi, tdata->rssi,
					sizeof(tdata->rssi));
			}
		} else {
			if (tstats->rate > tstats->last_rate &&
				per < sparams->hi_rate_threshold) {
				switch_ant = true;

				if (ar->smart_ant_info.debug_level >=
					ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATS) {
					ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
					"switch ant on higher rate: curr rate %d last rate %d "
					"curr PER %d hi_rate_threshold %d\n",
					tstats->rate, tstats->last_rate,
					per, sparams->hi_rate_threshold);
				}
			} else if (tstats->rate > tstats->last_rate) {
				per = tstats->next_ant_per;
				tdata->nbad = tstats->next_ant_nbad;
				tdata->nframes = tstats->next_ant_nframes;
				memcpy(tdata->rssi, tstats->nxt_rssi,
					sizeof(tstats->nxt_rssi));
				tstats->rate = tstats->last_rate;
				nxt_antenna = 1;
			}

			per_diff = per < tstats->per ?
				(tstats->per - per) : (per - tstats->per);
			if (tstats->rate == tstats->last_rate &&
				per_diff <= sparams->per_diff_threshold) {
				switch_ant = smart_ant_sec_metric(sa_sta);

				if (ar->smart_ant_info.debug_level >=
					ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATS &&
					switch_ant == true) {
					ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
					"switch ant on higher RSSI: curr rate %d "
					"last rate %d "
					"per_diff %d per_diff_threshold %d\n",
					tstats->rate, tstats->last_rate,
					per_diff, sparams->per_diff_threshold);
				}
			} else if (tstats->rate == tstats->last_rate &&
				per < tstats->per) {
				switch_ant = true;

				if (ar->smart_ant_info.debug_level >=
					ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATS) {
					ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
					"switch ant on lower per: curr rate %d last rate %d "
					"curr PER %d last PER %d\n",
					tstats->rate, tstats->last_rate,
					per, tstats->per);
				}
			}
		}

		if (switch_ant) {
			tinfo->sel_ant = tstats->ant_map[tstats->antenna];
			status = ATH10K_SMART_ANT_ACT_TX_CFG;
			tstats->per = per;
			tstats->next_ant_nbad = tdata->nbad;
			tstats->next_ant_nframes = tdata->nframes;
			tstats->last_rate = tstats->rate;
			memcpy(tstats->rssi, tdata->rssi, sizeof(tdata->rssi));
			if (per < sparams->low_rate_threshold)
				nxt_rate_dir = 1;
			if (ar->smart_ant_info.debug_level >=
				ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATS) {
				ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
				"switching to:antenna %d antMapIdx %d "
				"PER %d RSSI %d %d %d rate %x\n",
				tstats->ant_map[tstats->antenna],
				tstats->antenna, tstats->per,
				tstats->rssi[0][0], tstats->rssi[1][0],
				tstats->rssi[2][0], tdata->rate_code);
			}
		}
	}

	if (nxt_rate_dir) {
		s32 ridx;
		ridx = smart_ant_move_rate(sa_sta, nxt_rate_dir);
		tstats->last_rate = tstats->rate;
		nxt_antenna = ridx >= 0 ? 0 : 1;
		if (!nxt_antenna) {
			tstats->last_train_dir = nxt_rate_dir;
			tstats->rate = ridx;
			status |= ATH10K_SMART_ANT_ACT_TRAIN;
			smart_ant_set_train_params(ar, sa_sta,
					tstats->ant_map[tstats->antenna]);
		}
	}

	if (!nxt_antenna)
		return status;

	tstats->last_train_dir = 0;
	if (tstats->ant_map[0] != tinfo->sel_ant)
		smart_ant_update_antmap(ar, sa_sta, tinfo->sel_ant);
	tstats->skip_mask |= (1 << tstats->antenna);
	/* To skip the antenna combination already trained */
	while (tstats->antenna < sparams->num_tx_ant_comb) {
		tstats->antenna++;
		if (tstats->skip_mask & ~(1 << tstats->antenna))
			break;
	}

	if (tstats->antenna < sparams->num_tx_ant_comb) {
		status |= ATH10K_SMART_ANT_ACT_TRAIN;
		smart_ant_set_train_params(ar, sa_sta,
				tstats->ant_map[tstats->antenna]);
	} else { /* Training has completed */
		tinfo->train_state = ATH10K_SMART_ANT_STATE_INIT;
		tinfo->intense_train = false;
		status |= smart_ant_sel_rx_ant(ar, sa_sta);
		tinfo->train_end_ts = jiffies;
		tinfo->train_start = false;
		tinfo->perf_mon_slot = tinfo->train_end_ts;
		if (ar->smart_ant_info.debug_level >=
			ATH10K_SMART_ANT_DBG_LVL_TOP_DECISION) {
			ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
			"Training completed:antenna selected %d "
			"PER %d RSSI %d %d %d rate %x\n",
			tstats->ant_map[0],
			tstats->per,
			tstats->rssi[0][0], tstats->rssi[1][0],
			tstats->rssi[2][0], tdata->rate_code);
		}
		/* force config tx/rx antenna after SA decision is made */
		status |= ATH10K_SMART_ANT_ACT_TX_CFG;
		status |= ATH10K_SMART_ANT_ACT_RX_CFG;
	}

	return status;
}

static u8 smart_ant_perf_train_trigger(struct ath10k *ar,
					struct ath10k_smart_ant_sta *sa_sta,
					int bw)
{
	struct ath10k_smart_ant_params sparams =
					ar->smart_ant_info.smart_ant_params;
	struct ath10k_smart_ant_perf_info *pinfo;
	struct ath10k_smart_ant_rate_stats *rstats = &sa_sta->rate_stats;
	u32 pdelta, thrshld;
	u8 status = 0;

	pinfo = &sa_sta->train_info.perf_info[bw];

	if (rstats->nppdu_bw[bw] <= sparams.num_pkt_min_threshod[bw]) {
		if (pinfo->gput_avg_intvl >=
			(sparams.ignore_goodput_interval +
			sparams.avg_goodput_interval))
				return status;

		if (pinfo->gput_avg_intvl > sparams.ignore_goodput_interval)
			pinfo->gput_avg_intvl = sparams.ignore_goodput_interval;

		return status;
	}

	if (pinfo->gput_avg_intvl < (sparams.ignore_goodput_interval +
		sparams.avg_goodput_interval)) {
		if (pinfo->gput_avg_intvl++ >=
			sparams.ignore_goodput_interval) {
			pinfo->avg_gput = pinfo->avg_gput ?
				(pinfo->avg_gput + rstats->ins_gput[bw]) >> 1
				: rstats->ins_gput[bw];
			if (ar->smart_ant_info.debug_level >=
				ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATS) {
				ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
				"BW: %d avg gput %d ins gput %d hyster %d",
				bw, pinfo->avg_gput, rstats->ins_gput[bw],
				pinfo->hysteresis);
			}
		}
	} else {
		thrshld = (pinfo->avg_gput * sparams.max_perf_delta) / 100;
		if (sparams.min_goodput_threshold > thrshld)
			thrshld = sparams.min_goodput_threshold;
		pdelta = abs(pinfo->avg_gput - rstats->ins_gput[bw]);

		if (ar->smart_ant_info.debug_level >=
			ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATS) {
			ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
			"BW %d avg gput %d ins gput %d"
			" hyster %d thrshld %d pdelta: %d",
			bw, pinfo->avg_gput, rstats->ins_gput[bw],
			pinfo->hysteresis, thrshld, pdelta);
		}

		if (pdelta < thrshld) {
			pinfo->hysteresis = 0;
			pinfo->trig_type = ATH10K_SMART_ANT_TRIGGER_TYPE_INIT;
			pinfo->avg_gput =
				(ATH10K_AVG_GPUT_FACTOR * pinfo->avg_gput +
				 rstats->ins_gput[bw]) >> ATH10K_GPUT_SHIFT;
			return status;
		}

		switch (pinfo->trig_type) {
		case ATH10K_SMART_ANT_TRIGGER_TYPE_INIT:
			pinfo->hysteresis++;
			pinfo->trig_type =
				pinfo->avg_gput < rstats->ins_gput[bw] ?
				ATH10K_SMART_ANT_TRIGGER_TYPE_PVE :
				ATH10K_SMART_ANT_TRIGGER_TYPE_NVE;
			break;
		case ATH10K_SMART_ANT_TRIGGER_TYPE_PVE:
			if (pinfo->avg_gput < rstats->ins_gput[bw]) {
				pinfo->hysteresis++;
			} else {
				pinfo->hysteresis = 0;
				pinfo->trig_type =
					ATH10K_SMART_ANT_TRIGGER_TYPE_INIT;
			}
			break;
		case ATH10K_SMART_ANT_TRIGGER_TYPE_NVE:
			if (pinfo->avg_gput > rstats->ins_gput[bw]) {
				pinfo->hysteresis++;
			} else {
				pinfo->hysteresis = 0;
				pinfo->trig_type =
					ATH10K_SMART_ANT_TRIGGER_TYPE_INIT;
			}
			break;
		default:
			ath10k_err(ar, "invalid trigger type\n");
			return status;
		}

		if (pinfo->hysteresis >= sparams.hysteresis) {
			pinfo->hysteresis = 0;
			pinfo->avg_gput = 0;
			pinfo->gput_avg_intvl = 0;
			pinfo->trig_type = ATH10K_SMART_ANT_TRIGGER_TYPE_INIT;
			sa_sta->train_info.train_state =
					ATH10K_SMART_ANT_STATE_PRETRAIN;
			status = ATH10K_SMART_ANT_ACT_TRAIN;
			if (ar->smart_ant_info.debug_level >=
				ATH10K_SMART_ANT_DBG_LVL_TOP_DECISION) {
				ath10k_dbg(ar, ATH10K_DBG_SMART_ANT, "start Perf train\n");
			}
		}
	}

	return status;
}

static u8 smart_ant_retrain_trigger(struct ath10k *ar,
					struct ath10k_smart_ant_sta *sa_sta)
{
	struct ath10k_smart_ant_params sparams =
					ar->smart_ant_info.smart_ant_params;
	u32 elapsed_ts, current_ts;
	u8 status = 0;
	int i;

	current_ts = jiffies;
	elapsed_ts = current_ts - sa_sta->train_info.train_end_ts;
	if (elapsed_ts > sparams.retrain_interval ||
		sa_sta->train_info.train_start) {
		if (ar->smart_ant_info.debug_level >=
			ATH10K_SMART_ANT_DBG_LVL_TOP_DECISION) {
			ath10k_dbg(ar, ATH10K_DBG_SMART_ANT, "start Periodic train\n");
		}
		sa_sta->train_info.train_state =
			ATH10K_SMART_ANT_STATE_PRETRAIN;
		return ATH10K_SMART_ANT_ACT_TRAIN;
	}

	/* Perf based train */
	elapsed_ts = current_ts - sa_sta->train_info.perf_mon_slot;
	if (elapsed_ts < sparams.perf_train_interval)
		goto rst_stats;

	for (i = 0; i < ATH10K_SMART_ANT_BW_MAX; i++) {
		status = smart_ant_perf_train_trigger(ar, sa_sta, i);
		if (status)
			break;
		sa_sta->rate_stats.ins_gput[i] = 0;
		sa_sta->rate_stats.nppdu_bw[i] = 0;
		sa_sta->rate_stats.nppdu_gput[i] = 0;
	}

	sa_sta->train_info.perf_mon_slot = current_ts;
rst_stats:
	if (!status) {
		memset(sa_sta->rate_stats.npkts_legacy, 0,
			sizeof(sa_sta->rate_stats.npkts_legacy));
		memset(sa_sta->rate_stats.npkts_mcs, 0,
			sizeof(sa_sta->rate_stats.npkts_mcs));
	}

	return status;
}

static void smart_ant_update_training(struct ath10k *ar,
					struct ath10k_smart_ant_sta *sa_sta,
					struct ath10k_smart_ant_tx_fb *fb,
					u16 *action)
{
	struct ath10k_smart_ant_comb_fb *comb_fb;
	struct ath10k_smart_ant_params sparams =
				ar->smart_ant_info.smart_ant_params;
	struct ath10k_smart_ant_train_data *tdata = &sa_sta->train_data;
	struct ath10k_smart_ant_train_info *tinfo = &sa_sta->train_info;
	u8 nfb, npkts, nbad, rate, bw;
	u8 rate_cfg;
	bool chk_bw_change = false;
	int i, j;

	if (!fb->train_pkt) {
		if (fb->npkts > fb->nbad)
			smart_ant_update_rates(ar, sa_sta, fb);
		goto proc_stats;
	}

	for (i = fb->num_comb_fb; i >= 0; i--) {
		comb_fb = i ? &fb->comb_fb[i - 1] : NULL;
		nfb = comb_fb ? ATH10K_NFB_COMB_FB(comb_fb->bw) : 1;
		bw = comb_fb ? ATH10K_COMB_FB_BW(comb_fb->bw) :
				ATH10K_FB_BW(fb->ridx);
		rate = comb_fb ? comb_fb->rate :
				ATH10K_FB_RATE(fb->rate_mcs[0], bw);
		npkts = comb_fb ? comb_fb->npkts : fb->npkts;
		nbad = comb_fb ? comb_fb->nbad : fb->nbad;

		rate_cfg = ATH10K_FB_RATE(tdata->rate_code, bw);
		if (fb->tx_antenna[0] != tdata->antenna || rate_cfg != rate) {
			if (ar->smart_ant_info.debug_level >=
				ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATS) {
				ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
				"train pkt with mismatch: tdata antenna: %d"
				" fb_antenna %d cfg_rates: %x fb_rate: %x\n",
				tdata->antenna, fb->tx_antenna[0],
				rate_cfg, rate);
			}
			continue;
		}

		tinfo->num_ppdu_bw[bw] += nfb;
		tinfo->num_ppdu += nfb;
		if (bw != tinfo->train_stats.bw) {
			chk_bw_change = true;
			continue;
		}

		tdata->nbad += nbad;
		tdata->nframes += npkts;

		if (!i) {
			for (j = 0; j < ATH10K_SMART_ANT_MAX_CHAINS;  j++) {
				tdata->rssi[j][tdata->samples] =
							(u8) fb->rssi[j];
			}
			tdata->samples++;
			if (tdata->samples >= ATH10K_SMART_ANT_RSSI_SAMPLE)
				tdata->samples = 0;
		}
	}

	if (chk_bw_change &&
		tinfo->num_ppdu > sparams.num_other_bw_pkts_threshold &&
		tinfo->train_stats.bw != ATH10K_SMART_ANT_BW_20) {
		smart_ant_change_bw(ar, sa_sta);
		if (tinfo->train_stats.bw_change) {
			if (ar->smart_ant_info.debug_level >=
				ATH10K_SMART_ANT_DBG_LVL_TRAIN_STAGES) {
				ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
				"Trigger training due to bandwidth change\n");
			}
			*action |= ATH10K_SMART_ANT_ACT_TRAIN;
		}
	}

proc_stats:
	if (tdata->nframes >= tdata->num_pkts ||
	tinfo->num_ppdu_bw[tinfo->train_stats.bw] > sparams.max_train_ppdu)
		*action |= smart_ant_proc_train_stats(ar, sa_sta);
}

/* Worker to process various smart antenna related wmi configuration
 * request. The request is queued based on the decision made in feedback
 * processing.
 */
static void smart_ant_cfg_work(struct work_struct *work)
{
	struct ath10k_smart_ant_sta *sa_sta = container_of(work,
					struct ath10k_smart_ant_sta,
					sa_wmi_cfg_work);
	struct ath10k *ar = sa_sta->ar;
	struct ath10k_smart_ant_wmi_cfg_param *cfg, *tmp;
	int ret;

	spin_lock_bh(&sa_sta->cfg_lock);
	list_for_each_entry_safe(cfg, tmp, &sa_sta->cfg_list, list) {
		list_del(&cfg->list);
		spin_unlock_bh(&sa_sta->cfg_lock);

		mutex_lock(&ar->conf_mutex);

		switch (cfg->type) {
		case ATH10K_SMART_ANT_TYPE_RX_CFG:
			ret = ath10k_wmi_pdev_set_rx_ant(ar,
						cfg->rx_ant_cfg.rx_ant);
			if (ret)
				ath10k_warn(ar, "Unable to set Rx antenna\n");
			break;
		case ATH10K_SMART_ANT_TYPE_TX_CFG:
			ret = ath10k_wmi_peer_set_smart_tx_ant(ar,
						cfg->tx_ant_cfg.vdev_id,
						cfg->tx_ant_cfg.peer_mac,
						cfg->tx_ant_cfg.tx_ants,
						WMI_SMART_ANT_RATE_SERIES_MAX);
			if (ret)
				ath10k_warn(ar, "Unable to set Tx antenna for %pM\n",
					cfg->tx_ant_cfg.peer_mac);
			break;
		case ATH10K_SMART_ANT_TYPE_TRAIN_INFO:
			ret = ath10k_wmi_peer_set_smart_ant_train_info(ar,
					cfg->train_info_cfg.vdev_id,
					cfg->train_info_cfg.peer_mac,
					&cfg->train_info_cfg.train_info);
			if (ret)
				ath10k_warn(ar, "Unable to set training information for %pM\n",
					cfg->train_info_cfg.peer_mac);
			break;
		default:
		case ATH10K_SMART_ANT_TYPE_MAX:
			if (ar->smart_ant_info.debug_level >=
				ATH10K_SMART_ANT_DBG_LVL_TRAIN_STAGES) {
				ath10k_dbg(ar, ATH10K_DBG_SMART_ANT, "Not a supported config type :%d\n",
				cfg->type);
			}
		}

		mutex_unlock(&ar->conf_mutex);
		kfree(cfg);
		spin_lock_bh(&sa_sta->cfg_lock);
	}
	spin_unlock_bh(&sa_sta->cfg_lock);
}

static int smart_ant_config_tx(struct ath10k *ar,
				struct ath10k_smart_ant_sta *sa_sta,
				u8 *peer_mac, int vdev_id)
{
	struct ath10k_smart_ant_wmi_cfg_param *cfg;

	cfg = kzalloc(sizeof(*cfg), GFP_ATOMIC);
	if (!cfg) {
		ath10k_err(ar, "Failed to allocate memory for smart antenna wmi config\n");
		return -ENOMEM;
	}

	cfg->type = ATH10K_SMART_ANT_TYPE_TX_CFG;
	cfg->tx_ant_cfg.vdev_id = vdev_id;
	ether_addr_copy(cfg->tx_ant_cfg.peer_mac, peer_mac);
	smart_ant_get_tx_ant(sa_sta, cfg->tx_ant_cfg.tx_ants);

	if (ar->smart_ant_info.debug_level >=
		ATH10K_SMART_ANT_DBG_LVL_TOP_DECISION) {
		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT, "Configure tx antenna for %pM to %d\n",
		cfg->tx_ant_cfg.peer_mac, cfg->tx_ant_cfg.tx_ants[0]);
	}

	spin_lock_bh(&sa_sta->cfg_lock);
	list_add_tail(&cfg->list, &sa_sta->cfg_list);
	spin_unlock_bh(&sa_sta->cfg_lock);
	ieee80211_queue_work(ar->hw, &sa_sta->sa_wmi_cfg_work);
	return 0;
}

static int smart_ant_config_rx(struct ath10k *ar,
				struct ath10k_smart_ant_sta *sa_sta)
{
	struct ath10k_smart_ant_wmi_cfg_param *cfg;
	struct ath10k_smart_ant_info *info = &ar->smart_ant_info;

	cfg = kzalloc(sizeof(*cfg), GFP_ATOMIC);
	if (!cfg) {
		ath10k_err(ar, "Failed to allocate memory for smart antenna wmi config\n");
		return -ENOMEM;
	}

	cfg->type = ATH10K_SMART_ANT_TYPE_RX_CFG;
	cfg->rx_ant_cfg.rx_ant = info->rx_ant;

	if (ar->smart_ant_info.debug_level >=
		ATH10K_SMART_ANT_DBG_LVL_TOP_DECISION) {
		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT, "Configure Rx antenna to %d\n",
		cfg->rx_ant_cfg.rx_ant);
	}
	spin_lock_bh(&sa_sta->cfg_lock);
	list_add_tail(&cfg->list, &sa_sta->cfg_list);
	spin_unlock_bh(&sa_sta->cfg_lock);
	ieee80211_queue_work(ar->hw, &sa_sta->sa_wmi_cfg_work);
	return 0;
}

static int smart_ant_config_train_info(struct ath10k *ar,
					struct ath10k_smart_ant_sta *sa_sta,
					u8 *peer_mac, int vdev_id)
{
	struct ath10k_smart_ant_wmi_cfg_param *cfg;

	cfg = kzalloc(sizeof(*cfg), GFP_ATOMIC);
	if (!cfg) {
		ath10k_err(ar, "Failed to allocate memory for smart antenna wmi config\n");
		return -ENOMEM;
	}

	cfg->type = ATH10K_SMART_ANT_TYPE_TRAIN_INFO;
	cfg->train_info_cfg.vdev_id = vdev_id;
	ether_addr_copy(cfg->train_info_cfg.peer_mac, peer_mac);
	smart_ant_get_train_info(ar, sa_sta,
					&cfg->train_info_cfg.train_info);

	if (ar->smart_ant_info.debug_level >=
		ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATES) {
		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
		"Configure train information peer: %pM vdev_id: %d"
		" num_pkts:%d tx_antenna[0] %d tx_antenna[1] %d"
		" rates [0] %x",
		cfg->train_info_cfg.peer_mac, cfg->train_info_cfg.vdev_id,
		cfg->train_info_cfg.train_info.num_pkts,
		cfg->train_info_cfg.train_info.antennas[0],
		cfg->train_info_cfg.train_info.antennas[1],
		cfg->train_info_cfg.train_info.rates[0]);
	}

	spin_lock_bh(&sa_sta->cfg_lock);
	list_add_tail(&cfg->list, &sa_sta->cfg_list);
	spin_unlock_bh(&sa_sta->cfg_lock);
	ieee80211_queue_work(ar->hw, &sa_sta->sa_wmi_cfg_work);
	return 0;
}

static void smart_ant_tx_stats_update(struct ath10k *ar,
				struct ath10k_smart_ant_sta *sa_sta,
				struct ath10k_smart_ant_tx_fb *fb,
				u16 *action)
{
	struct ath10k_smart_ant_info *sinfo = &ar->smart_ant_info;
	struct ath10k_smart_ant_train_info *tinfo;
	struct ath10k_smart_ant_params *sparams = &sinfo->smart_ant_params;
	int i;

	tinfo = &sa_sta->train_info;

	switch (sa_sta->train_info.train_state) {
	case ATH10K_SMART_ANT_STATE_PRETRAIN:
		if (fb->npkts > fb->nbad) {
			smart_ant_update_rates(ar, sa_sta, fb);
			tinfo->num_tx_pkts += fb->npkts;
			fb->num_comb_fb =
				fb->num_comb_fb < ATH10K_SMART_ANT_COMB_FB_MAX
				? fb->num_comb_fb :
				ATH10K_SMART_ANT_COMB_FB_MAX;
			for (i = 0; i < fb->num_comb_fb; i++)
				tinfo->num_tx_pkts += fb->comb_fb[i].npkts;
		}

		if (tinfo->num_tx_pkts >= sparams->num_pretrain_pkts) {
			if (ar->smart_ant_info.debug_level >=
				ATH10K_SMART_ANT_DBG_LVL_TRAIN_STAGES) {
				ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
				"Exceeded number of pretrain pkts, trigger training\n");
			}
			*action |= ATH10K_SMART_ANT_ACT_TRAIN;
		}
		break;
	case ATH10K_SMART_ANT_STATE_TRAIN_PROGRESS:
		smart_ant_update_training(ar, sa_sta, fb, action);
		break;
	default:
		*action |= smart_ant_retrain_trigger(ar, sa_sta);
		if (fb->train_pkt)
			*action |= ATH10K_SMART_ANT_ACT_TRAIN;
		else if (fb->npkts > fb->nbad)
			smart_ant_update_rates(ar, sa_sta, fb);
	}

	if (*action != 0) {
		if (ar->smart_ant_info.debug_level >=
			ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATES) {
			ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
			"Action after Tx status update:0x%x train_state:%d\n",
			*action, sa_sta->train_info.train_state);
		}
	}
}

static void smart_ant_tx_fb_fill(struct ath10k *ar,
					u32 *tx_ctrl_desc,
					struct ath10k_smart_ant_tx_fb *fb)
{
	struct ath10k_smart_ant_info *info = &ar->smart_ant_info;
	u32 *tx_status_desc = info->tx_ppdu_end;
	u32 tot_tries;
	u32 try_status;
	u8 sbw_idx_suc;
	int i;

	fb->npkts = TXCS_MS(tx_status_desc, ATH10K_SMART_ANT_NPKTS);
	fb->nbad = TXCS_MS(tx_status_desc, ATH10K_SMART_ANT_NBAD);
	fb->num_comb_fb = TXCS_MS(tx_status_desc, ATH10K_SMART_ANT_COMB_FB);
	fb->train_pkt = TXCS_MS(tx_status_desc, ATH10K_SMART_ANT_TRAIN_PKT) ?
			true : false;
	tot_tries = TXCS_MS(tx_status_desc, ATH10K_TXS_TOT_TRIES);

	fb->tx_antenna[0] = TXCS_MS(tx_ctrl_desc, ATH10K_TXC_ANT_S0);
	fb->tx_antenna[1] = TXCS_MS(tx_ctrl_desc, ATH10K_TXC_ANT_S1);
	fb->rate_mcs[0] =  TXCS_MS(tx_ctrl_desc, ATH10K_TXC_S0_RATE_BW20) |
			TXCS_MS(tx_ctrl_desc, ATH10K_TXC_S0_RATE_BW40) |
			TXCS_MS(tx_ctrl_desc, ATH10K_TXC_S0_RATE_BW80) |
			TXCS_MS(tx_ctrl_desc, ATH10K_TXC_S0_RATE_BW160);

	fb->rate_mcs[1] =  TXCS_MS(tx_ctrl_desc, ATH10K_TXC_S1_RATE_BW20) |
			TXCS_MS(tx_ctrl_desc, ATH10K_TXC_S1_RATE_BW40) |
			TXCS_MS(tx_ctrl_desc, ATH10K_TXC_S1_RATE_BW80) |
			TXCS_MS(tx_ctrl_desc, ATH10K_TXC_S1_RATE_BW160);

	for (i = 0; i < ATH10K_SMART_ANT_MAX_CHAINS; i++)
		fb->rssi[i] = __le32_to_cpu(
				tx_status_desc[ATH10K_TXS_ACK_RSSI + i]);

	try_status = __le32_to_cpu(tx_status_desc[tot_tries - 1]);
	sbw_idx_suc = (try_status & ATH10K_TXS_TRY_SERIES_MASK) ?
			ATH10K_SMART_ANT_DYN_BW_MAX : 0;
	sbw_idx_suc += ((try_status & ATH10K_TXS_TRY_BW_M) >>
			ATH10K_TXS_TRY_BW_S);

	fb->ridx = sbw_idx_suc;
	fb->rate_maxphy = __le32_to_cpu(
				tx_status_desc[ATH10K_SMART_ANT_FEEDBACK_2]);
	fb->gput = __le32_to_cpu(tx_status_desc[ATH10K_SMART_ANT_GPUT]);
	memcpy((u8 *)&fb->comb_fb[0],
		(u8 *)&tx_status_desc[ATH10K_TXS_LRETRY],
		sizeof(fb->comb_fb[0]));
	memcpy((u8 *)&fb->comb_fb[1],
		(u8 *)&tx_status_desc[ATH10K_TXS_SRETRY],
		sizeof(fb->comb_fb[1]));

	smart_ant_dbg_feedback(ar, fb);
}

static int smart_ant_get_streams(struct ath10k *ar)
{
	u32 num_chains = 0;
	int i;

	for (i = 0; i < ATH10K_SMART_ANT_MAX_CHAINS; i++) {
		if (ar->supp_tx_chainmask & (1 << i))
			num_chains++;
	}

	return min(num_chains, ar->num_rf_chains);
}

static void smart_ant_init_param(struct ath10k *ar)
{
	struct ath10k_smart_ant_info *info = &ar->smart_ant_info;
	struct ath10k_smart_ant_params *sa_params =
					&ar->smart_ant_info.smart_ant_params;

	info->mode = WMI_SMART_ANT_MODE_PARALLEL;
	info->default_ant = ATH10K_SMART_ANT_DEFAULT_ANT;
	info->num_fallback_rate = ATH10K_SMART_ANT_FALLBACK_RATE_DEFAULT;
	info->enabled_feedback = ATH10K_SMART_ANT_TX_FEEDBACK|
				ATH10K_SMART_ANT_RX_FEEDBACK;

	sa_params->low_rate_threshold	= ATH10K_SMART_ANT_PER_MIN_THRESHOLD;
	sa_params->hi_rate_threshold	= ATH10K_SMART_ANT_PER_MAX_THRESHOLD;
	sa_params->per_diff_threshold	= ATH10K_SMART_ANT_PER_DIFF_THRESHOLD;
	sa_params->num_train_pkts	= 0;
	sa_params->pkt_len		= ATH10K_SMART_ANT_PKT_LEN_DEFAULT;
	sa_params->num_tx_ant_comb	= 1 << smart_ant_get_streams(ar);
	sa_params->num_min_pkt		= ATH10K_SMART_ANT_NUM_PKT_MIN;
	sa_params->retrain_interval	= msecs_to_jiffies(
					ATH10K_SMART_ANT_RETRAIN_INTVL);
	sa_params->perf_train_interval	= msecs_to_jiffies(
					ATH10K_SMART_ANT_PERF_TRAIN_INTVL);
	sa_params->max_perf_delta	= ATH10K_SMART_ANT_TPUT_DELTA_DEFAULT;
	sa_params->hysteresis		= ATH10K_SMART_ANT_HYSTERISYS_DEFAULT;
	sa_params->min_goodput_threshold =
				ATH10K_SMART_ANT_MIN_GOODPUT_THRESHOLD;
	sa_params->avg_goodput_interval	= ATH10K_SMART_ANT_GOODPUT_INTVL_AVG;
	sa_params->ignore_goodput_interval =
				ATH10K_SMART_ANT_IGNORE_GOODPUT_INTVL;
	sa_params->num_pretrain_pkts = ATH10K_SMART_ANT_PRETRAIN_PKTS_MAX;
	sa_params->num_other_bw_pkts_threshold = ATH10K_SMART_ANT_BW_THRESHOLD;
	sa_params->enabled_train = ATH10K_SMART_ANT_TRAIN_TRIGGER_PERIODIC |
				ATH10K_SMART_ANT_TRAIN_TRIGGER_PERF |
				ATH10K_SMART_ANT_TRAIN_TRIGGER_RX;
	sa_params->num_pkt_min_threshod[ATH10K_SMART_ANT_BW_20] =
				ATH10K_SMART_ANT_NUM_PKT_THRESHOLD_20;
	sa_params->num_pkt_min_threshod[ATH10K_SMART_ANT_BW_40] =
				ATH10K_SMART_ANT_NUM_PKT_THRESHOLD_40;
	sa_params->num_pkt_min_threshod[ATH10K_SMART_ANT_BW_80] =
				ATH10K_SMART_ANT_NUM_PKT_THRESHOLD_80;
	sa_params->default_tx_ant	= ATH10K_SMART_ANT_DEFAULT_ANT;
	sa_params->ant_change_ind	= 0;
	sa_params->max_train_ppdu	= ATH10K_SMART_ANT_NUM_TRAIN_PPDU_MAX;
}

/* Global functions called from smart antenna APIs */
void ath10k_smart_ant_proc_rx_feedback(struct ath10k *ar,
					struct htt_rx_desc *rx_desc)
{
	struct ath10k_smart_ant_info *sa_info = &ar->smart_ant_info;

	if (!ath10k_smart_ant_enabled(ar) || !sa_info->enabled)
		return;

	if (!(__le32_to_cpu(rx_desc->attention.flags) &
		RX_ATTENTION_FLAGS_LAST_MPDU))
		return;

	sa_info->rx_antenna = __le32_to_cpu(rx_desc->ppdu_end.common.info0) &
				ATH10K_RX_ANT_MASK;
}

void ath10k_smart_ant_proc_tx_feedback(struct ath10k *ar, u8 *data)
{
	struct ath10k_pktlog_hdr *pl_hdr = (struct ath10k_pktlog_hdr *)data;
	u16 log_type = __le16_to_cpu(pl_hdr->log_type);
	struct ath10k_peer *peer;
	struct ath10k_smart_ant_info *info = &ar->smart_ant_info;

	if (!ath10k_smart_ant_enabled(ar) || !info->enabled)
		return;

	if (!(info->enabled_feedback & ATH10K_SMART_ANT_TX_FEEDBACK))
		return;

	if (log_type != ATH10K_SMART_ANT_PKTLOG_TYPE_TXCTL &&
		log_type != ATH10K_SMART_ANT_PKTLOG_TYPE_TXSTATS)
		return;

	if (log_type == ATH10K_SMART_ANT_PKTLOG_TYPE_TXSTATS) {
		memcpy((u8 *)info->tx_ppdu_end, pl_hdr->payload,
			sizeof(info->tx_ppdu_end));
	} else {
		struct ieee80211_sta *sta;
		struct ath10k_smart_ant_tx_fb feed_back;
		struct ath10k_sta *arsta;
		u32 *tx_ctrl_desc, *tx_status_desc;
		u32 peer_id;
		u32 ftype;
		u8 peer_mac[ETH_ALEN];
		u16 action = 0;
		int ret;

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
			if (ar->smart_ant_info.debug_level >=
				ATH10K_SMART_ANT_DBG_LVL_TRAIN_STAGES) {
				ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
				"Sta entry for %pM not found\n", peer_mac);
			}
			return;
		}

		arsta = (struct ath10k_sta *)sta->drv_priv;

		if (!arsta->smart_ant_sta)
			goto exit;

		memset(&feed_back, 0, sizeof(feed_back));
		if (ar->smart_ant_info.debug_level >=
			ATH10K_SMART_ANT_DBG_LVL_TRAIN_STATES) {
			ath10k_dbg(ar, ATH10K_DBG_SMART_ANT, "Tx feedback from sta: %pM\n",
				peer_mac);
		}
		smart_ant_tx_fb_fill(ar, tx_ctrl_desc, &feed_back);

		smart_ant_tx_stats_update(ar, arsta->smart_ant_sta,
					&feed_back, &action);
		if (action & ATH10K_SMART_ANT_ACT_TX_CFG) {
			ret = smart_ant_config_tx(ar, arsta->smart_ant_sta,
					peer_mac,
					arsta->arvif->vdev_id);
			if (ret)
				goto exit;
		}

		if (action & ATH10K_SMART_ANT_ACT_RX_CFG) {
			ret = smart_ant_config_rx(ar, arsta->smart_ant_sta);
			if (ret)
				goto exit;
		}

		if (action & ATH10K_SMART_ANT_ACT_TRAIN) {
			smart_ant_config_train_info(ar,
					arsta->smart_ant_sta,
					peer_mac,
					arsta->arvif->vdev_id);
		}
exit:
		rcu_read_unlock();
	}
}

void ath10k_smart_ant_sta_disconnect(struct ath10k *ar,
					struct ieee80211_sta *sta)
{
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k_smart_ant_sta *sa_sta;
	struct ath10k_smart_ant_wmi_cfg_param *cfg, *tmp_cfg;
	struct ath10k_smart_ant_info *info = &ar->smart_ant_info;

	if (!ath10k_smart_ant_enabled(ar) || !arsta->smart_ant_sta)
		return;

	if (ar->smart_ant_info.debug_level >=
		ATH10K_SMART_ANT_DBG_LVL_TOP_DECISION) {
		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
		"Smart antenna disconnect for %pM\n", sta->addr);
	}

	sa_sta = arsta->smart_ant_sta;
	cancel_work_sync(&sa_sta->sa_wmi_cfg_work);

	spin_lock_bh(&sa_sta->cfg_lock);
	list_for_each_entry_safe(cfg, tmp_cfg, &sa_sta->cfg_list, list) {
		list_del(&cfg->list);
		kfree(cfg);
	}
	spin_unlock_bh(&sa_sta->cfg_lock);

	info->num_sta_conneted--;
	info->num_sta_per_ant[sa_sta->train_info.prev_sel_ant]--;

	arsta->smart_ant_sta = NULL;
	kfree(arsta->smart_ant_sta);
}

int ath10k_smart_ant_sta_connect(struct ath10k *ar,
				struct ath10k_vif *arvif,
				struct ieee80211_sta *sta)
{
	struct wmi_smart_ant_sta_cfg_arg arg;
	struct ath10k_sta *arsta = (struct ath10k_sta *)sta->drv_priv;
	struct ath10k_smart_ant_sta *smart_ant_sta;
	struct ath10k_smart_ant_info *info = &ar->smart_ant_info;
	struct ath10k_smart_ant_train_stats *train_stats;
	u8 tx_chainmask = ar->cfg_tx_chainmask ? : ar->supp_tx_chainmask;
	int i, ret;

	lockdep_assert_held(&ar->conf_mutex);

	if (!ath10k_smart_ant_enabled(ar) || !info->enabled)
		return 0;

	if (arvif->vdev_type != WMI_VDEV_TYPE_AP ||
		arvif->vdev_subtype != WMI_VDEV_SUBTYPE_NONE) {
		if (ar->smart_ant_info.debug_level >=
			ATH10K_SMART_ANT_DBG_LVL_TOP_DECISION) {
			ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
			"Smart antenna logic not enabled for non-AP interface\n");
		}
		return 0;
	}

	smart_ant_sta = kzalloc(sizeof(*smart_ant_sta), GFP_KERNEL);
	if (!smart_ant_sta) {
		ath10k_warn(ar, "failed to allocate memory for sta specific smart antenna\n");
		return -ENOMEM;
	}

	memcpy(&smart_ant_sta->rate_cap, &ar->ratecode_list,
		sizeof(smart_ant_sta->rate_cap));

	if (ar->smart_ant_info.debug_level >=
		ATH10K_SMART_ANT_DBG_LVL_TOP_DECISION) {
		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
		"Smart antenna connect for %pM\n", sta->addr);
	}

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

	if (ar->smart_ant_info.debug_level >=
		ATH10K_SMART_ANT_DBG_LVL_TRAIN_STAGES) {
		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
		"%s mac %pM vdev_id %d num_cfg %d\n",
		__func__, arg.mac_addr.addr, arg.vdev_id, arg.num_cfg);

		for (i = 0; i < arg.num_cfg; i++) {
			ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
				"cfg[%d] 0x%x\n", i, arg.cfg[i]);
		}
	}

	/* Configure feedback option for this station, i.e tx feedback
	 * for how many PPDUs during training and non-training period.
	 */
	ret = ath10k_wmi_peer_cfg_smart_ant(ar, &arg);

	if (ret) {
		ath10k_warn(ar, "Failed to set feedback config\n");
		kfree(smart_ant_sta);
		return ret;
	}

	arsta->smart_ant_sta = smart_ant_sta;

	spin_lock_init(&smart_ant_sta->cfg_lock);
	INIT_LIST_HEAD(&smart_ant_sta->cfg_list);
	INIT_WORK(&smart_ant_sta->sa_wmi_cfg_work, smart_ant_cfg_work);

	smart_ant_sta->ar = ar;
	smart_ant_sta->train_info.sel_ant =
			info->smart_ant_params.default_tx_ant;
	smart_ant_sta->train_info.prev_sel_ant =
			info->smart_ant_params.default_tx_ant;
	smart_ant_sta->train_info.last_bw =
			ATH10K_SMART_ANT_BW_20;
	smart_ant_sta->train_info.feedback_cfg =
			ATH10K_TX_FEEDBACK_CONFIG_DEFAULT;
	smart_ant_sta->train_info.train_bitmap =
			ATH10K_SMART_ANT_TRAIN_TRIGGER_PERIODIC |
			ATH10K_SMART_ANT_TRAIN_TRIGGER_PERF |
			ATH10K_SMART_ANT_TRAIN_TRIGGER_RX;
	smart_ant_sta->train_info.train_state =
			ATH10K_SMART_ANT_STATE_PRETRAIN;

	if (sta->vht_cap.vht_supported)
		smart_ant_sta->wmode = ATH10K_WIRELESS_MODE_VHT;
	else if (sta->ht_cap.ht_supported)
		smart_ant_sta->wmode = ATH10K_WIRELESS_MODE_HT;
	else
		smart_ant_sta->wmode = ATH10K_WIRELESS_MODE_LEGACY;

	train_stats = &smart_ant_sta->train_info.train_stats;
	for (i = 0; i < info->smart_ant_params.num_tx_ant_comb; i++) {
		if ((i & tx_chainmask) == i)
			train_stats->ant_map[i] = i;
		else
			/* num_tx_ant_comb is used to mark ant_map[i] as an
			 * invalid antenna combination
			*/
			train_stats->ant_map[i] =
				info->smart_ant_params.num_tx_ant_comb;
	}

	smart_ant_update_antmap(ar, smart_ant_sta,
				smart_ant_sta->train_info.sel_ant);

	info->num_sta_conneted++;
	info->num_sta_per_ant[smart_ant_sta->train_info.prev_sel_ant]++;
	return 0;
}

int ath10k_smart_ant_set_default(struct ath10k *ar,
				struct ath10k_vif *arvif)
{
	struct ath10k_smart_ant_info *info = &ar->smart_ant_info;
	int ret, i;
	u32 tx_ants[WMI_SMART_ANT_RATE_SERIES_MAX];

	lockdep_assert_held(&ar->conf_mutex);

	if (!ath10k_smart_ant_enabled(ar))
		return 0;

	if (!info->enabled)
		return 0;

	if (arvif->vdev_type != WMI_VDEV_TYPE_AP ||
		arvif->vdev_subtype != WMI_VDEV_SUBTYPE_NONE) {
		if (ar->smart_ant_info.debug_level >=
			ATH10K_SMART_ANT_DBG_LVL_TOP_DECISION) {
			ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
			"Smart antenna logic not enabled for non-AP interface\n");
		}
		return 0;
	}

	/* Set default tx/rx antennas to start with */
	ret = ath10k_wmi_pdev_set_rx_ant(ar, info->default_ant);
	if (ret) {
		ath10k_warn(ar, "Failed to set rx antenna\n");
		return ret;
	}

	/* Tx antenna for every fallback rate series */
	for (i = 0; i <= info->num_fallback_rate; i++)
		tx_ants[i] = info->default_ant;

	ret = ath10k_wmi_peer_set_smart_tx_ant(ar, arvif->vdev_id,
						arvif->vif->addr, tx_ants,
						info->num_fallback_rate + 1);
	if (ret)
		ath10k_warn(ar, "Failed to set tx antenna\n");

	return ret;
}

void ath10k_smart_ant_disable(struct ath10k *ar, struct ath10k_vif *arvif)
{
	struct ath10k_smart_ant_info *info = &ar->smart_ant_info;
	int ret;

	lockdep_assert_held(&ar->conf_mutex);

	if (!ath10k_smart_ant_enabled(ar))
		return;

	if (!info->enabled)
		return;

	if (arvif->vdev_type != WMI_VDEV_TYPE_AP ||
		arvif->vdev_subtype != WMI_VDEV_SUBTYPE_NONE)
		return;

	/* See if this is the last vif requesting to disable smart antenna */
	info->num_enabled_vif--;
	if (info->num_enabled_vif != 0)
		return;

	/* Disable smart antenna logic in fw */
	ret = ath10k_wmi_pdev_disable_smart_ant(ar, info->mode, 0, 0);
	if (ret) {
		ath10k_err(ar, "Wmi command to disable smart antenna is failed\n");
		return;
	}

	info->enabled = false;
	ath10k_wmi_pdev_pktlog_disable(ar);
}

int ath10k_smart_ant_enable(struct ath10k *ar, struct ath10k_vif *arvif)
{
	struct ath10k_smart_ant_info *info = &ar->smart_ant_info;
	int ret;

	lockdep_assert_held(&ar->conf_mutex);

	if (!ath10k_smart_ant_enabled(ar))
		return 0;

	/* Smart antenna is tested with only AP mode, it can also be enabled
	 * for other modes, just needs more testing.
	 */
	if (arvif->vdev_type != WMI_VDEV_TYPE_AP ||
		arvif->vdev_subtype != WMI_VDEV_SUBTYPE_NONE) {
		if (ar->smart_ant_info.debug_level >=
			ATH10K_SMART_ANT_DBG_LVL_TOP_DECISION) {
			ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
			"Smart antenna logic not enabled for non-AP interface\n");
		}
		return 0;
	}

	info->num_enabled_vif++;
	if (info->enabled)
		return 0;

	smart_ant_init_param(ar);

	if (ar->smart_ant_info.debug_level >=
		ATH10K_SMART_ANT_DBG_LVL_TOP_DECISION) {
		ath10k_dbg(ar, ATH10K_DBG_SMART_ANT,
			"Hw supports Smart antenna, enabling it in driver\n");
	}

	/* Enable smart antenna logic in fw with mode and default antenna */
	ret = ath10k_wmi_pdev_enable_smart_ant(ar, info->mode,
						info->default_ant,
						info->default_ant);
	if (ret) {
		ath10k_err(ar, "Wmi command to enable smart antenna is failed\n");
		return ret;
	}

	info->enabled = true;

	/* Enable tx feedback through packetlog */
	return ath10k_wmi_pdev_pktlog_enable(ar, ATH10K_PKTLOG_SMART_ANT);
}
