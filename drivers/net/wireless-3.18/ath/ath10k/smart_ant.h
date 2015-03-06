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

/* This API is to process rx feedback such as rssi, PER and antennas.
 * Based on the stats a better antenna combination can be found for rx.
 * Better rx antenna can be configured using ath10k_wmi_pdev_set_rx_ant().
 */

#ifndef _SMART_ANT_H_
#define _SMART_ANT_H_

static inline void
ath10k_smart_ant_proc_rx_feedback(struct ath10k *ar,
				  struct htt_rx_desc *rx_desc)
{
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
}

/* In AP mode, this API notifies of disassociation of a station.
 * Station specific information related to smart antenna should
 * be reset in this API.
 */
static inline void
ath10k_smart_ant_sta_disconnect(struct ath10k *ar, struct ieee80211_sta *sta)
{
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
