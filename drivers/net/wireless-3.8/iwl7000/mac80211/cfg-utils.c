/*
 * Wireless utility functions
 *
 * Copyright 2007-2009	Johannes Berg <johannes@sipsolutions.net>
 * Copyright 2013-2014  Intel Mobile Communications GmbH
 */
#include <linux/export.h>
#include <net/cfg80211.h>

#if CFG80211_VERSION < KERNEL_VERSION(4,1,0)
static bool ieee80211_id_in_list(const u8 *ids, int n_ids, u8 id)
{
	int i;

	for (i = 0; i < n_ids; i++)
		if (ids[i] == id)
			return true;
	return false;
}

size_t ieee80211_ie_split_ric(const u8 *ies, size_t ielen,
			      const u8 *ids, int n_ids,
			      const u8 *after_ric, int n_after_ric,
			      size_t offset)
{
	size_t pos = offset;

	while (pos < ielen && ieee80211_id_in_list(ids, n_ids, ies[pos])) {
		if (ies[pos] == WLAN_EID_RIC_DATA && n_after_ric) {
			pos += 2 + ies[pos + 1];

			while (pos < ielen &&
			       !ieee80211_id_in_list(after_ric, n_after_ric,
						     ies[pos]))
				pos += 2 + ies[pos + 1];
		} else {
			pos += 2 + ies[pos + 1];
		}
	}

	return pos;
}
EXPORT_SYMBOL(ieee80211_ie_split_ric);

size_t ieee80211_ie_split(const u8 *ies, size_t ielen,
			  const u8 *ids, int n_ids, size_t offset)
{
	return ieee80211_ie_split_ric(ies, ielen, ids, n_ids, NULL, 0, offset);
}
EXPORT_SYMBOL(ieee80211_ie_split);
#endif
