#include <linux/if_ether.h>
#include <net/cfg80211.h>

/* common backward compat code */

static inline void netdev_attach_ops(struct net_device *dev,
				     const struct net_device_ops *ops)
{
	dev->netdev_ops = ops;
}

#define WIPHY_FLAG_HAS_CHANNEL_SWITCH 0

#define mc_addr(ha)	(ha)->addr

static inline void
cfg80211_ch_switch_started_notify(struct net_device *dev,
				  struct cfg80211_chan_def *chandef)
{
}

/* cfg80211 version specific backward compat code follows */
#define CFG80211_VERSION KERNEL_VERSION(3,8,0)

#if CFG80211_VERSION < KERNEL_VERSION(3,9,0)
struct cfg80211_wowlan_wakeup {
	bool disconnect, magic_pkt, gtk_rekey_failure,
	     eap_identity_req, four_way_handshake,
	     rfkill_release, packet_80211,
	     tcp_match, tcp_connlost, tcp_nomoretokens;
	s32 pattern_idx;
	u32 packet_present_len, packet_len;
	const void *packet;
};

struct wiphy_wowlan_tcp_support {
	const struct nl80211_wowlan_tcp_data_token_feature *tok;
	u32 data_payload_max;
	u32 data_interval_max;
	u32 wake_payload_max;
	bool seq;
};

struct cfg80211_wowlan_tcp {
	struct socket *sock;
	__be32 src, dst;
	u16 src_port, dst_port;
	u8 dst_mac[ETH_ALEN];
	int payload_len;
	const u8 *payload;
	struct nl80211_wowlan_tcp_data_seq payload_seq;
	u32 data_interval;
	u32 wake_len;
	const u8 *wake_data, *wake_mask;
	u32 tokens_size;
	/* must be last, variable member */
	struct nl80211_wowlan_tcp_data_token payload_tok;
};

static inline void
cfg80211_report_wowlan_wakeup(struct wireless_dev *wdev,
			      struct cfg80211_wowlan_wakeup *wakeup, gfp_t gfp)
{
}
#endif /* CFG80211_VERSION < KERNEL_VERSION(3,9,0) */

#if CFG80211_VERSION < KERNEL_VERSION(3,10,0)
static inline bool
ieee80211_operating_class_to_band(u8 operating_class,
				  enum ieee80211_band *band)
{
	switch (operating_class) {
	case 112:
	case 115 ... 127:
		*band = IEEE80211_BAND_5GHZ;
		return true;
	case 81:
	case 82:
	case 83:
	case 84:
		*band = IEEE80211_BAND_2GHZ;
		return true;
	case 180:
		*band = IEEE80211_BAND_60GHZ;
		return true;
	}

	/* stupid compiler */
	*band = IEEE80211_BAND_2GHZ;

	return false;
}

#define NL80211_FEATURE_USERSPACE_MPM 0
#endif /* CFG80211_VERSION < KERNEL_VERSION(3,10,0) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,11,0)
#define IEEE80211_RADIOTAP_MCS_HAVE_STBC 0
#endif

#if CFG80211_VERSION < KERNEL_VERSION(3,11,0)
#define IEEE80211_MAX_CHAINS 4

#define MONITOR_FLAG_ACTIVE 0

static inline void cfg80211_rx_unprot_mlme_mgmt(struct net_device *dev,
						void *data, int len)
{
	struct ieee80211_hdr *hdr = data;

	if (ieee80211_is_deauth(hdr->frame_control))
		cfg80211_send_unprot_deauth(dev, data, len);
	else
		cfg80211_send_unprot_disassoc(dev, data, len);
}

static inline void cfg80211_tx_mlme_mgmt(struct net_device *dev,
					 void *data, int len)
{
	struct ieee80211_hdr *hdr = data;

	if (ieee80211_is_deauth(hdr->frame_control))
		cfg80211_send_deauth(dev, data, len);
	else
		cfg80211_send_disassoc(dev, data, len);
}

static inline void cfg80211_rx_mlme_mgmt(struct net_device *dev,
					 void *data, int len)
{
	struct ieee80211_hdr *hdr = data;

	if (ieee80211_is_auth(hdr->frame_control))
		cfg80211_send_rx_auth(dev, data, len);
	else if (ieee80211_is_deauth(hdr->frame_control))
		cfg80211_send_deauth(dev, data, len);
	else
		cfg80211_send_disassoc(dev, data, len);
}

static inline void cfg80211_assoc_timeout(struct net_device *dev,
					  struct cfg80211_bss *bss)
{
	cfg80211_send_assoc_timeout(dev, bss->bssid);
}

static inline void cfg80211_auth_timeout(struct net_device *dev,
					 const u8 *bssid)
{
	cfg80211_send_auth_timeout(dev, bssid);
}

static inline void cfg80211_rx_assoc_resp(struct net_device *dev,
					  struct cfg80211_bss *bss,
					  void *data, int len)
{
	cfg80211_send_rx_assoc(dev, bss, data, len);
}

static inline enum ieee80211_rate_flags
ieee80211_chandef_rate_flags(struct cfg80211_chan_def *chandef)
{
	return 0;
}

#define IEEE80211_RADIOTAP_MCS_STBC_SHIFT	5

/* on older versions this is safe - no RTNL use there */
#define cfg80211_sched_scan_stopped_rtnl cfg80211_sched_scan_stopped
#endif /* CFG80211_VERSION < KERNEL_VERSION(3,11,0) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,12,0)
#define IEEE80211_CHAN_HALF 0
#define IEEE80211_CHAN_QUARTER 0
#endif

#if CFG80211_VERSION < KERNEL_VERSION(3,12,0)
static inline int
ieee80211_chandef_max_power(struct cfg80211_chan_def *chandef)
{
	return chandef->chan->max_power;
}

static inline struct cfg80211_bss * __must_check
cfg80211_inform_bss_width_frame(struct wiphy *wiphy,
				struct ieee80211_channel *channel,
				enum nl80211_bss_scan_width scan_width,
				struct ieee80211_mgmt *mgmt, size_t len,
				s32 signal, gfp_t gfp)
{
	return cfg80211_inform_bss_frame(wiphy, channel, mgmt,
					 len, signal, gfp);
}

static inline enum nl80211_bss_scan_width
cfg80211_chandef_to_scan_width(const struct cfg80211_chan_def *chandef)
{
	return NL80211_BSS_CHAN_WIDTH_20;
}

static inline bool
iwl7000_cfg80211_rx_mgmt(struct wireless_dev *wdev, int freq, int sig_dbm,
			 const u8 *buf, size_t len, u32 flags, gfp_t gfp)
{
	return cfg80211_rx_mgmt(wdev, freq, sig_dbm, buf, len, gfp);
}
#define cfg80211_rx_mgmt iwl7000_cfg80211_rx_mgmt

struct cfg80211_csa_settings {
	struct cfg80211_chan_def chandef;
	struct cfg80211_beacon_data beacon_csa;
	u16 counter_offset_beacon, counter_offset_presp;
	struct cfg80211_beacon_data beacon_after;
	bool radar_required;
	bool block_tx;
	u8 count;
};

static inline u32
ieee80211_mandatory_rates(struct ieee80211_supported_band *sband)
{
	struct ieee80211_rate *bitrates;
	u32 mandatory_rates = 0;
	enum ieee80211_rate_flags mandatory_flag;
	int i;

	if (WARN_ON(!sband))
		return 1;

	if (sband->band == IEEE80211_BAND_2GHZ)
		mandatory_flag = IEEE80211_RATE_MANDATORY_B;
	else
		mandatory_flag = IEEE80211_RATE_MANDATORY_A;

	bitrates = sband->bitrates;
	for (i = 0; i < sband->n_bitrates; i++)
		if (bitrates[i].flags & mandatory_flag)
			mandatory_rates |= BIT(i);
	return mandatory_rates;
}

#define ieee80211_mandatory_rates(sband, width) ieee80211_mandatory_rates(sband)
#endif /* CFG80211_VERSION < KERNEL_VERSION(3,12,0) */

#if CFG80211_VERSION < KERNEL_VERSION(3,13,0)
static inline int cfg80211_chandef_get_width(const struct cfg80211_chan_def *c)
{
	int width;

	switch (c->width) {
	case NL80211_CHAN_WIDTH_20:
	case NL80211_CHAN_WIDTH_20_NOHT:
		width = 20;
		break;
	case NL80211_CHAN_WIDTH_40:
		width = 40;
		break;
	case NL80211_CHAN_WIDTH_80P80:
	case NL80211_CHAN_WIDTH_80:
		width = 80;
		break;
	case NL80211_CHAN_WIDTH_160:
		width = 160;
		break;
	default:
		WARN_ON_ONCE(1);
		return -1;
	}
	return width;
}

static inline int cfg80211_get_chans_dfs_required(struct wiphy *wiphy,
						  u32 center_freq,
						  u32 bandwidth)
{
	struct ieee80211_channel *c;
	u32 freq, start_freq, end_freq;

	if (bandwidth <= 20) {
		start_freq = center_freq;
		end_freq = center_freq;
	} else {
		start_freq = center_freq - bandwidth/2 + 10;
		end_freq = center_freq + bandwidth/2 - 10;
	}

	for (freq = start_freq; freq <= end_freq; freq += 20) {
		c = ieee80211_get_channel(wiphy, freq);
		if (!c)
			return -EINVAL;

		if (c->flags & IEEE80211_CHAN_RADAR)
			return 1;
	}
	return 0;
}

#define cfg80211_radar_event(...) do { } while (0)
#endif /* CFG80211_VERSION < KERNEL_VERSION(3,13,0) */

#if CFG80211_VERSION < KERNEL_VERSION(3,14,0)
struct cfg80211_mgmt_tx_params {
	struct ieee80211_channel *chan;
	bool offchan;
	unsigned int wait;
	const u8 *buf;
	size_t len;
	bool no_cck;
	bool dont_wait_for_ack;
};

#define regulatory_flags flags

#define REGULATORY_CUSTOM_REG WIPHY_FLAG_CUSTOM_REGULATORY
#define REGULATORY_DISABLE_BEACON_HINTS WIPHY_FLAG_DISABLE_BEACON_HINTS

#define IEEE80211_CHAN_NO_IR (IEEE80211_CHAN_PASSIVE_SCAN |\
			      IEEE80211_CHAN_NO_IBSS)
#endif /* CFG80211_VERSION < KERNEL_VERSION(3,14,0) */

#if CFG80211_VERSION < KERNEL_VERSION(3,15,0)
#define IEEE80211_RADIOTAP_CODING_LDPC_USER0	0x1

#define cfg80211_ibss_joined(dev, bssid, chan, gfp) \
	cfg80211_ibss_joined(dev, bssid, gfp)
#endif /* CFG80211_VERSION < KERNEL_VERSION(3,15,0) */

#if CFG80211_VERSION < KERNEL_VERSION(3,16,0)
#define REGULATORY_ENABLE_RELAX_NO_IR 0

#define cfg80211_reg_can_beacon(wiphy, chandef, iftype) \
	cfg80211_reg_can_beacon(wiphy, chandef)

static inline int
cfg80211_chandef_dfs_required(struct wiphy *wiphy,
			      const struct cfg80211_chan_def *chandef,
			      enum nl80211_iftype iftype)
{
	int width;
	int ret;

	if (WARN_ON(!cfg80211_chandef_valid(chandef)))
		return -EINVAL;

	switch (iftype) {
	case NL80211_IFTYPE_ADHOC:
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_P2P_GO:
	case NL80211_IFTYPE_MESH_POINT:
		width = cfg80211_chandef_get_width(chandef);
		if (width < 0)
			return -EINVAL;

		ret = cfg80211_get_chans_dfs_required(wiphy,
						      chandef->center_freq1,
						      width);
		if (ret < 0)
			return ret;
		else if (ret > 0)
			return BIT(chandef->width);

		if (!chandef->center_freq2)
			return 0;

		ret = cfg80211_get_chans_dfs_required(wiphy,
						      chandef->center_freq2,
						      width);
		if (ret < 0)
			return ret;
		else if (ret > 0)
			return BIT(chandef->width);

		break;
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_P2P_CLIENT:
	case NL80211_IFTYPE_MONITOR:
	case NL80211_IFTYPE_AP_VLAN:
	case NL80211_IFTYPE_WDS:
	case NL80211_IFTYPE_P2P_DEVICE:
		break;
	case NL80211_IFTYPE_UNSPECIFIED:
	case NUM_NL80211_IFTYPES:
		WARN_ON(1);
	}

	return 0;
}

static inline int
cfg80211_iter_combinations(struct wiphy *wiphy,
			   const int num_different_channels,
			   const u8 radar_detect,
			   const int iftype_num[NUM_NL80211_IFTYPES],
			   void (*iter)(const struct ieee80211_iface_combination *c,
					void *data),
			   void *data)
{
	int i, j, iftype;
	int num_interfaces = 0;
	u32 used_iftypes = 0;

	for (iftype = 0; iftype < NUM_NL80211_IFTYPES; iftype++) {
		num_interfaces += iftype_num[iftype];
		if (iftype_num[iftype] > 0 &&
		    !(wiphy->software_iftypes & BIT(iftype)))
			used_iftypes |= BIT(iftype);
	}

	for (i = 0; i < wiphy->n_iface_combinations; i++) {
		const struct ieee80211_iface_combination *c;
		struct ieee80211_iface_limit *limits;
		u32 all_iftypes = 0;

		c = &wiphy->iface_combinations[i];

		if (num_interfaces > c->max_interfaces)
			continue;
		if (num_different_channels > c->num_different_channels)
			continue;

		limits = kmemdup(c->limits, sizeof(limits[0]) * c->n_limits,
				 GFP_KERNEL);
		if (!limits)
			return -ENOMEM;

		for (iftype = 0; iftype < NUM_NL80211_IFTYPES; iftype++) {
			if (wiphy->software_iftypes & BIT(iftype))
				continue;
			for (j = 0; j < c->n_limits; j++) {
				all_iftypes |= limits[j].types;
				if (!(limits[j].types & BIT(iftype)))
					continue;
				if (limits[j].max < iftype_num[iftype])
					goto cont;
				limits[j].max -= iftype_num[iftype];
			}
		}

		if (radar_detect)
			goto cont;

		/* Finally check that all iftypes that we're currently
		 * using are actually part of this combination. If they
		 * aren't then we can't use this combination and have
		 * to continue to the next.
		 */
		if ((all_iftypes & used_iftypes) != used_iftypes)
			goto cont;

		/* This combination covered all interface types and
		 * supported the requested numbers, so we're good.
		 */

		(*iter)(c, data);
 cont:
		kfree(limits);
	}

	return 0;
}

static void
cfg80211_iter_sum_ifcombs(const struct ieee80211_iface_combination *c,
			  void *data)
{
	int *num = data;
	(*num)++;
}

static inline int
cfg80211_check_combinations(struct wiphy *wiphy,
			    const int num_different_channels,
			    const u8 radar_detect,
			    const int iftype_num[NUM_NL80211_IFTYPES])
{
	int err, num = 0;

	err = cfg80211_iter_combinations(wiphy, num_different_channels,
					 radar_detect, iftype_num,
					 cfg80211_iter_sum_ifcombs, &num);
	if (err)
		return err;
	if (num == 0)
		return -EBUSY;

	return 0;
}
#endif /* CFG80211_VERSION < KERNEL_VERSION(3,16,0) */
