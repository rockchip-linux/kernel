/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *                                        
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
#define _RTW_RF_C_

#include <drv_types.h>
#include <hal_data.h>

u8 center_ch_5g_all[CENTER_CH_5G_ALL_NUM] = {
		36, 38, 40, 42, 44, 46, 48,			/* Band 1 */
		52, 54, 56, 58, 60, 62, 64,			/* Band 2 */
		100, 102, 104, 106, 108, 110, 112,	/* Band 3 */
		116, 118, 120, 122, 124, 126, 128,	/* Band 3 */
		132, 134, 136, 138, 140, 142, 144,	/* Band 3 */
		149, 151, 153, 155, 157, 159, 161,	/* Band 4 */
		165, 167, 169, 171, 173, 175, 177};	/* Band 4 */

u8 center_ch_5g_20m[CENTER_CH_5G_20M_NUM] = {
	36, 40, 44, 48,
	52, 56, 60, 64,
	100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
	149, 153, 157, 161, 165, 169, 173, 177
};

u8 center_ch_5g_40m[CENTER_CH_5G_40M_NUM] = {38, 46, 54, 62, 102, 110, 118, 126, 134, 142, 151, 159, 167, 175};

u8 center_ch_5g_80m[CENTER_CH_5G_80M_NUM] = {42, 58, 106, 122, 138, 155, 171};

struct center_chs_ent {
	u8 ch_num;
	u8 *chs;
};

struct center_chs_ent center_chs_5g_by_bw[] = {
	{CENTER_CH_5G_20M_NUM, center_ch_5g_20m},
	{CENTER_CH_5G_40M_NUM, center_ch_5g_40m},
	{CENTER_CH_5G_80M_NUM, center_ch_5g_80m},
};

inline u8 center_chs_5g_num(u8 bw)
{
	if (bw >= CHANNEL_WIDTH_160)
		return 0;
	
	return center_chs_5g_by_bw[bw].ch_num;
}

inline u8 center_chs_5g(u8 bw, u8 id)
{
	if (bw >= CHANNEL_WIDTH_160)
		return 0;

	if (id >= center_chs_5g_num(bw))
		return 0;
		
	return center_chs_5g_by_bw[bw].chs[id];
}

int rtw_ch2freq(int chan)
{
	/* see 802.11 17.3.8.3.2 and Annex J
	* there are overlapping channel numbers in 5GHz and 2GHz bands */

	/*
	* RTK: don't consider the overlapping channel numbers: 5G channel <= 14,
	* because we don't support it. simply judge from channel number
	*/

	if (chan >= 1 && chan <= 14) {
		if (chan == 14)
			return 2484;
		else if (chan < 14)
			return 2407 + chan * 5;
	} else if (chan >= 36 && chan <= 177) {
		return 5000 + chan * 5;
	}

	return 0; /* not supported */
}

int rtw_freq2ch(int freq)
{
	/* see 802.11 17.3.8.3.2 and Annex J */
	if (freq == 2484)
		return 14;
	else if (freq < 2484)
		return (freq - 2407) / 5;
	else if (freq >= 4910 && freq <= 4980)
		return (freq - 4000) / 5;
	else if (freq <= 45000) /* DMG band lower limit */
		return (freq - 5000) / 5;
	else if (freq >= 58320 && freq <= 64800)
		return (freq - 56160) / 2160;
	else
		return 0;
}

bool rtw_chbw_to_freq_range(u8 ch, u8 bw, u8 offset, u32 *hi, u32 *lo)
{
	u8 c_ch;
	u32 freq;
	u32 hi_ret = 0, lo_ret = 0;
	int i;
	bool valid = _FALSE;

	if (hi)
		*hi = 0;
	if (lo)
		*lo = 0;

	c_ch = rtw_get_center_ch(ch, bw, offset);
	freq = rtw_ch2freq(c_ch);

	if (!freq) {
		rtw_warn_on(1);
		goto exit;
	}

	if (bw == CHANNEL_WIDTH_80) {
		hi_ret = freq + 40;
		lo_ret = freq - 40;
	} else if (bw == CHANNEL_WIDTH_40) {
		hi_ret = freq + 20;
		lo_ret = freq - 20;
	} else if (bw == CHANNEL_WIDTH_20) {
		hi_ret = freq + 10;
		lo_ret = freq - 10;
	} else {
		rtw_warn_on(1);
	}

	if (hi)
		*hi = hi_ret;
	if (lo)
		*lo = lo_ret;

	valid = _TRUE;

exit:
	return valid;
}

const char * const _ch_width_str[] = {
	"20MHz",
	"40MHz",
	"80MHz",
	"160MHz",
	"80_80MHz",
	"CHANNEL_WIDTH_MAX",
};

const u8 _ch_width_to_bw_cap[] = {
	BW_CAP_20M,
	BW_CAP_40M,
	BW_CAP_80M,
	BW_CAP_160M,
	BW_CAP_80_80M,
	0,
};

const char * const _band_str[] = {
	"2.4G",
	"5G",
	"BOTH",
	"BAND_MAX",
};

const u8 _band_to_band_cap[] = {
	BAND_CAP_2G,
	BAND_CAP_5G,
	0,
	0,
};

struct country_chplan {
	char alpha2[2];
	u8 chplan;
};

static const struct country_chplan country_chplan_map[] = {
	{"AD", 0x26}, /* Andorra */
	{"AE", 0x26}, /* United Arab Emirates */
	{"AG", 0x30}, /* Antigua & Barbuda */
	{"AI", 0x26}, /* Anguilla(UK) */
	{"AL", 0x26}, /* Albania */
	{"AM", 0x34}, /* Armenia */
	{"AO", 0x26}, /* Angola */
	{"AQ", 0x26}, /* Antarctica */
	{"AR", 0x57}, /* Argentina */
	{"AS", 0x34}, /* American Samoa */
	{"AT", 0x26}, /* Austria */
	{"AU", 0x45}, /* Australia */
	{"AW", 0x34}, /* Aruba */
	{"AZ", 0x26}, /* Azerbaijan */
	{"BA", 0x26}, /* Bosnia & Herzegovina */
	{"BD", 0x26}, /* Bangladesh */
	{"BE", 0x26}, /* Belgium */
	{"BG", 0x26}, /* Bulgaria */
	{"BH", 0x47}, /* Bahrain */
	{"BO", 0x30}, /* Bolivia */
	{"BR", 0x34}, /* Brazil */
	{"CA", 0x34}, /* Canada */
	{"CH", 0x26}, /* Switzerland */
	{"CL", 0x30}, /* Chile */
	{"CN", 0x48}, /* China */
	{"CO", 0x34}, /* Colombia */
	{"CR", 0x34}, /* Costa Rica */
	{"CY", 0x26}, /* Cyprus */
	{"CZ", 0x26}, /* Czech Republic */
	{"DE", 0x26}, /* Germany */
	{"DK", 0x26}, /* Denmark */
	{"DO", 0x34}, /* Dominican Republic */
	{"EC", 0x34}, /* Ecuador */
	{"EE", 0x26}, /* Estonia */
	{"EG", 0x47}, /* Egypt */
	{"ES", 0x26}, /* Spain */
	{"FI", 0x26}, /* Finland */
	{"FR", 0x26}, /* France */
	{"GB", 0x26}, /* Great Britain (United Kingdom; England) */
	{"GH", 0x26}, /* Ghana */
	{"GR", 0x26}, /* Greece */
	{"GT", 0x34}, /* Guatemala */
	{"HK", 0x26}, /* Hong Kong */
	{"HN", 0x32}, /* Honduras */
	{"HR", 0x26}, /* Croatia */
	{"HU", 0x26}, /* Hungary */
	{"ID", 0x54}, /* Indonesia */
	{"IE", 0x26}, /* Ireland */
	{"IL", 0x47}, /* Israel */
	{"IN", 0x47}, /* India */
	{"IQ", 0x26}, /* Iraq */
	{"IS", 0x26}, /* Iceland */
	{"IT", 0x26}, /* Italy */
	{"JM", 0x51}, /* Jamaica */
	{"JO", 0x49}, /* Jordan */
	{"JP", 0x27}, /* Japan- Telec */
	{"KE", 0x47}, /* Kenya */
	{"KG", 0x26}, /* Kyrgyzstan */
	{"KH", 0x26}, /* Cambodia */
	{"KR", 0x28}, /* South Korea */
	{"KW", 0x47}, /* Kuwait */
	{"KZ", 0x26}, /* Kazakhstan */
	{"LB", 0x26}, /* Lebanon */
	{"LI", 0x26}, /* Liechtenstein */
	{"LK", 0x26}, /* Sri Lanka */
	{"LS", 0x26}, /* Lesotho */
	{"LT", 0x26}, /* Lithuania */
	{"LU", 0x26}, /* Luxembourg */
	{"LV", 0x26}, /* Latvia */
	{"MA", 0x47}, /* Morocco */
	{"MC", 0x26}, /* Monaco */
	{"ME", 0x26}, /* Montenegro */
	{"MK", 0x26}, /* Republic of Macedonia (FYROM) */
	{"MT", 0x26}, /* Malta */
	{"MX", 0x34}, /* Mexico */
	{"MY", 0x47}, /* Malaysia */
	{"MZ", 0x26}, /* Mozambique */
	{"NA", 0x26}, /* Namibia */
	{"NG", 0x50}, /* Nigeria */
	{"NI", 0x34}, /* Nicaragua */
	{"NL", 0x26}, /* Netherlands */
	{"NO", 0x26}, /* Norway */
	{"NZ", 0x45}, /* New Zealand */
	{"OM", 0x26}, /* Oman */
	{"PA", 0x34}, /* Panama */
	{"PE", 0x34}, /* Peru */
	{"PG", 0x26}, /* Papua New Guinea */
	{"PH", 0x26}, /* Philippines */
	{"PK", 0x51}, /* Pakistan */
	{"PL", 0x26}, /* Poland */
	{"PR", 0x34}, /* Puerto Rico */
	{"PT", 0x26}, /* Portugal */
	{"PY", 0x34}, /* Paraguay */
	{"QA", 0x51}, /* Qatar */
	{"RO", 0x26}, /* Romania */
	{"RS", 0x26}, /* Serbia */
	{"RU", 0x59}, /* Russia, fac/gost */
	{"SA", 0x26}, /* Saudi Arabia */
	{"SE", 0x26}, /* Sweden */
	{"SG", 0x47}, /* Singapore */
	{"SI", 0x26}, /* Slovenia */
	{"SK", 0x26}, /* Slovakia */
	{"SN", 0x26}, /* Senegal */
	{"SV", 0x30}, /* El Salvador */
	{"TH", 0x26}, /* Thailand */
	{"TN", 0x47}, /* Tunisia */
	{"TR", 0x26}, /* Turkey */
	{"TT", 0x42}, /* Trinidad & Tobago */
	{"TW", 0x39}, /* Taiwan */
	{"UA", 0x26}, /* Ukraine */
	{"US", 0x34}, /* United States of America (USA) */
	{"UY", 0x34}, /* Uruguay */
	{"VE", 0x30}, /* Venezuela */
	{"VN", 0x26}, /* Vietnam */
	{"YE", 0x26}, /* Yemen */
	{"ZA", 0x26}, /* South Africa */
	{"ZW", 0x26}, /* Zimbabwe */
};

u16 country_chplan_map_sz = sizeof(country_chplan_map)/sizeof(struct country_chplan);

/*
* rtw_get_chplan_from_country -
* @country_code: string of country code
*
* Return channel_plan index or -1 when unsupported country_code is given
*/
int rtw_get_chplan_from_country(const char *country_code)
{
	int channel_plan = -1;
	int i;

	/* TODO: should consider 3-character country code? */

	for (i = 0; i < country_chplan_map_sz; i++) {
		if (strncmp(country_code, country_chplan_map[i].alpha2, 2) == 0) {
			channel_plan = country_chplan_map[i].chplan;
			break;
		}
	}

	return channel_plan;
}

int rtw_ch_to_bb_gain_sel(int ch)
{
	int sel = -1;

	if (ch >= 1 && ch <= 14)
		sel = BB_GAIN_2G;
#ifdef CONFIG_IEEE80211_BAND_5GHZ
	else if (ch >= 36 && ch < 48)
		sel = BB_GAIN_5GLB1;
	else if (ch >= 52 && ch <= 64)
		sel = BB_GAIN_5GLB2;
	else if (ch >= 100 && ch <= 120)
		sel = BB_GAIN_5GMB1;
	else if (ch >= 124 && ch <= 144)
		sel = BB_GAIN_5GMB2;
	else if (ch >= 149 && ch <= 177)
		sel = BB_GAIN_5GHB;
#endif

	return sel;
}

s8 rtw_rf_get_kfree_tx_gain_offset(_adapter *padapter, u8 path, u8 ch)
{
	s8 kfree_offset = 0;

#ifdef CONFIG_RF_GAIN_OFFSET
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(padapter);
	struct kfree_data_t *kfree_data = GET_KFREE_DATA(padapter);
	s8 bb_gain_sel = rtw_ch_to_bb_gain_sel(ch);

	if (bb_gain_sel < BB_GAIN_2G || bb_gain_sel >= BB_GAIN_NUM) {
		rtw_warn_on(1);
		goto exit;
	}

	if (kfree_data->flag & KFREE_FLAG_ON) {
		kfree_offset = kfree_data->bb_gain[bb_gain_sel][path];
		if (1)
			DBG_871X("%s path:%u, ch:%u, bb_gain_sel:%d, kfree_offset:%d\n"
				, __func__, path, ch, bb_gain_sel, kfree_offset);
	}
exit:
#endif /* CONFIG_RF_GAIN_OFFSET */
	return kfree_offset;
}

void rtw_rf_set_tx_gain_offset(_adapter *adapter, u8 path, s8 offset)
{
	u8 write_value;

	DBG_871X("kfree gain_offset 0x55:0x%x ", rtw_hal_read_rfreg(adapter, path, 0x55, 0xffffffff));
	switch (rtw_get_chip_type(adapter)) {
#ifdef CONFIG_RTL8703B
	case RTL8703B:
		write_value = RF_TX_GAIN_OFFSET_8703B(offset);
		rtw_hal_write_rfreg(adapter, path, 0x55, 0x0fc000, write_value);
		break;
#endif /* CONFIG_RTL8703B */
#ifdef CONFIG_RTL8188F
	case RTL8188F:
		write_value = RF_TX_GAIN_OFFSET_8188F(offset);
		rtw_hal_write_rfreg(adapter, path, 0x55, 0x0fc000, write_value);
		break;
#endif /* CONFIG_RTL8188F */
#ifdef CONFIG_RTL8192E
	case RTL8192E:
		write_value = RF_TX_GAIN_OFFSET_8192E(offset);
		rtw_hal_write_rfreg(adapter, path, 0x55, 0x0f8000, write_value);
		break;
#endif /* CONFIG_RTL8188F */

#ifdef CONFIG_RTL8821A
	case RTL8821:
		write_value = RF_TX_GAIN_OFFSET_8821A(offset);
		rtw_hal_write_rfreg(adapter, path, 0x55, 0x0f8000, write_value);
		break;
#endif /* CONFIG_RTL8821A */
#ifdef CONFIG_RTL8814A
		case RTL8814A:
		DBG_871X("\nkfree by PhyDM on the sw CH. path %d\n", path);
		break;
#endif /* CONFIG_RTL8821A */

	default:
		rtw_warn_on(1);
		break;
	}

	DBG_871X(" after :0x%x\n", rtw_hal_read_rfreg(adapter, path, 0x55, 0xffffffff));
}

void rtw_rf_apply_tx_gain_offset(_adapter *adapter, u8 ch)
{
	HAL_DATA_TYPE *hal_data = GET_HAL_DATA(adapter);
	s8 kfree_offset = 0;
	s8 tx_pwr_track_offset = 0; /* TODO: 8814A should consider tx pwr track when setting tx gain offset */
	s8 total_offset;
	int i;

	for (i = 0; i < hal_data->NumTotalRFPath; i++) {
		kfree_offset = rtw_rf_get_kfree_tx_gain_offset(adapter, i, ch);
		total_offset = kfree_offset + tx_pwr_track_offset;
		rtw_rf_set_tx_gain_offset(adapter, i, total_offset);
	}
}

bool rtw_is_dfs_range(u32 hi, u32 lo)
{
	return rtw_is_range_overlap(hi, lo, 5720 + 10, 5260 - 10)?_TRUE:_FALSE;
}

bool rtw_is_dfs_ch(u8 ch, u8 bw, u8 offset)
{
	u32 hi, lo;

	if (rtw_chbw_to_freq_range(ch, bw, offset, &hi, &lo) == _FALSE)
		return _FALSE;

	return rtw_is_dfs_range(hi, lo)?_TRUE:_FALSE;
}

bool rtw_is_long_cac_range(u32 hi, u32 lo)
{
	return rtw_is_range_overlap(hi, lo, 5660 + 10, 5600 - 10)?_TRUE:_FALSE;
}

bool rtw_is_long_cac_ch(u8 ch, u8 bw, u8 offset)
{
	u32 hi, lo;

	if (rtw_chbw_to_freq_range(ch, bw, offset, &hi, &lo) == _FALSE)
		return _FALSE;

	return rtw_is_long_cac_range(hi, lo)?_TRUE:_FALSE;
}

