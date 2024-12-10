/*
 * Misc utility routines used by kernel or app-level.
 * Contents are wifi-specific, used by any kernel or app-level
 * software that might want wifi things as it grows.
 *
 * Portions of this code are copyright (c) 2022 Cypress Semiconductor Corporation
 *
 * Copyright (C) 1999-2017, Broadcom Corporation
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
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id: bcmwifi_channels.c 695288 2017-04-19 17:20:39Z $
 */

#include <bcm_cfg.h>
#include <typedefs.h>
#include <bcmutils.h>

#ifdef BCMDRIVER
#include <osl.h>
#define strtoul(nptr, endptr, base) bcm_strtoul((nptr), (endptr), (base))
#define tolower(c) (bcm_isupper((c)) ? ((c) + 'a' - 'A') : (c))
#else
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#ifndef ASSERT
#define ASSERT(exp)
#endif // endif
#endif /* BCMDRIVER */

#include <bcmwifi_channels.h>

#if defined(WIN32) && (defined(BCMDLL) || defined(WLMDLL))
#include <bcmstdlib.h> 	/* For wl/exe/GNUmakefile.brcm_wlu and GNUmakefile.wlm_dll */
#endif // endif

#include <802.11.h>

/* Definitions for D11AC capable (80MHz+) Chanspec type */

/* Chanspec ASCII representation:
 * [<band> 'g'] <channel> ['/'<bandwidth> [<primary-sideband>]['/'<1st80channel>'-'<2nd80channel>]]
 *
 * <band>:
 *      (optional) 2, 3, 4, 5 for 2.4GHz, 3GHz, 4GHz, and 5GHz respectively.
 *      Default value is 2g if channel <= 14, otherwise 5g.
 * <channel>:
 *      channel number of the 5MHz, 10MHz, 20MHz channel,
 *      or primary channel of 40MHz, 80MHz, 160MHz, or 80+80MHz channel.
 * <bandwidth>:
 *      (optional) 5, 10, 20, 40, 80, 160, or 80+80. Default value is 20.
 * <primary-sideband>:
 *      (only for 2.4GHz band 40MHz) U for upper sideband primary, L for lower.
 *
 *      For 2.4GHz band 40MHz channels, the same primary channel may be the
 *      upper sideband for one 40MHz channel, and the lower sideband for an
 *      overlapping 40MHz channel.  The U/L disambiguates which 40MHz channel
 *      is being specified.
 *
 *      For 40MHz in the 5GHz band and all channel bandwidths greater than
 *      40MHz, the U/L specificaion is not allowed since the channels are
 *      non-overlapping and the primary sub-band is derived from its
 *      position in the wide bandwidth channel.
 *
 * <1st80Channel>:
 * <2nd80Channel>:
 *      Required for 80+80, otherwise not allowed.
 *      Specifies the center channel of the primary and secondary 80MHz band.
 *
 * In its simplest form, it is a 20MHz channel number, with the implied band
 * of 2.4GHz if channel number <= 14, and 5GHz otherwise.
 *
 * To allow for backward compatibility with scripts, the old form for
 * 40MHz channels is also allowed: <channel><primary-sideband>
 *
 * <channel>:
 *	primary channel of 40MHz, channel <= 14 is 2GHz, otherwise 5GHz
 * <primary-sideband>:
 *	"U" for upper, "L" for lower (or lower case "u" "l")
 *
 * 5 GHz Examples:
 *      Chanspec        BW        Center Ch  Channel Range  Primary Ch
 *      5g8             20MHz     8          -              -
 *      52              20MHz     52         -              -
 *      52/40           40MHz     54         52-56          52
 *      56/40           40MHz     54         52-56          56
 *      52/80           80MHz     58         52-64          52
 *      56/80           80MHz     58         52-64          56
 *      60/80           80MHz     58         52-64          60
 *      64/80           80MHz     58         52-64          64
 *      52/160          160MHz    50         36-64          52
 *      36/160          160MGz    50         36-64          36
 *      36/80+80/42-106 80+80MHz  42,106     36-48,100-112  36
 *
 * 2 GHz Examples:
 *      Chanspec        BW        Center Ch  Channel Range  Primary Ch
 *      2g8             20MHz     8          -              -
 *      8               20MHz     8          -              -
 *      6               20MHz     6          -              -
 *      6/40l           40MHz     8          6-10           6
 *      6l              40MHz     8          6-10           6
 *      6/40u           40MHz     4          2-6            6
 *      6u              40MHz     4          2-6            6
 */

/* bandwidth ASCII string */
static const char *wf_chspec_bw_str[] =
{
	"5",
	"10",
	"20",
	"40",
	"80",
	"160",
	"80+80",
	"na"
};

static const uint8 wf_chspec_bw_mhz[] =
{5, 10, 20, 40, 80, 160, 160};

#define WF_NUM_BW \
	(sizeof(wf_chspec_bw_mhz)/sizeof(uint8))

/* 40MHz channels in 5GHz band */
static const uint8 wf_5g_40m_chans[] =
{38, 46, 54, 62, 102, 110, 118, 126, 134, 142, 151, 159, 167, 175};
#define WF_NUM_5G_40M_CHANS \
	(sizeof(wf_5g_40m_chans)/sizeof(uint8))

/* 80MHz channels in 5GHz band */
static const uint8 wf_5g_80m_chans[] =
{42, 58, 106, 122, 138, 155, 171};
#define WF_NUM_5G_80M_CHANS \
	(sizeof(wf_5g_80m_chans)/sizeof(uint8))

/* 160MHz channels in 5GHz band */
static const uint8 wf_5g_160m_chans[] =
{50, 114};
#define WF_NUM_5G_160M_CHANS \
	(sizeof(wf_5g_160m_chans)/sizeof(uint8))

/* Based on IEEE 802.11ax D6.1 */
/* 40MHz channels in 6GHz band */
static const uint8 wf_6g_40m_chans[] =
{3, 11, 19, 27, 35, 43, 51, 59, 67, 75, 83, 91, 99,
107, 115, 123, 131, 139, 147, 155, 163, 171, 179,
187, 195, 203, 211, 219, 227};
#define WF_NUM_6G_40M_CHANS \
	(sizeof(wf_6g_40m_chans)/sizeof(uint8))

/* 80MHz channels in 6GHz band */
static const uint8 wf_6g_80m_chans[] =
{7, 23, 39, 55, 71, 87, 103, 119, 135, 151, 167, 183,
199, 215};
#define WF_NUM_6G_80M_CHANS \
	(sizeof(wf_6g_80m_chans)/sizeof(uint8))

/* 160MHz channels in 6GHz band */
static const uint8 wf_6g_160m_chans[] =
{15, 47, 79, 111, 143, 175, 207};
#define WF_NUM_6G_160M_CHANS \
	(sizeof(wf_6g_160m_chans)/sizeof(uint8))

/* 6GHz PSC channels */
uint8 wf_6g_psc_chans[] =
{5, 21, 37, 53, 69, 85, 101, 117, 133, 149, 165, 181,
197, 213, 229};
#define WF_NUM_6G_PSC_CHANS \
	(sizeof(wf_6g_psc_chans)/sizeof(uint8))

/* opclass and channel information for US. Table E-1 */
static const uint16 opclass_data[] = {
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_20)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_20)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_20)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_20)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_20)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_5)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_5)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_10)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_10)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_20)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_20)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_2G |((WL_CHANSPEC_BW_20)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_3G |((WL_CHANSPEC_BW_20)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_3G |((WL_CHANSPEC_BW_10)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_3G |((WL_CHANSPEC_BW_5)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_5)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_10)&WL_CHANSPEC_BW_MASK)),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_20)&WL_CHANSPEC_BW_MASK)),
	0,
	0,
	0,
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_40)&WL_CHANSPEC_BW_MASK)|WL_CHANSPEC_CTL_SB_LOWER),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_40)&WL_CHANSPEC_BW_MASK)|WL_CHANSPEC_CTL_SB_LOWER),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_40)&WL_CHANSPEC_BW_MASK)|WL_CHANSPEC_CTL_SB_LOWER),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_40)&WL_CHANSPEC_BW_MASK)|WL_CHANSPEC_CTL_SB_LOWER),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_40)&WL_CHANSPEC_BW_MASK)|WL_CHANSPEC_CTL_SB_LOWER),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_40)&WL_CHANSPEC_BW_MASK)|WL_CHANSPEC_CTL_SB_UPPER),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_40)&WL_CHANSPEC_BW_MASK)|WL_CHANSPEC_CTL_SB_UPPER),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_40)&WL_CHANSPEC_BW_MASK)|WL_CHANSPEC_CTL_SB_UPPER),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_40)&WL_CHANSPEC_BW_MASK)|WL_CHANSPEC_CTL_SB_UPPER),
	(WL_CHANSPEC_BAND_5G |((WL_CHANSPEC_BW_40)&WL_CHANSPEC_BW_MASK)|WL_CHANSPEC_CTL_SB_UPPER),
	(WL_CHANSPEC_BAND_2G |((WL_CHANSPEC_BW_40)&WL_CHANSPEC_BW_MASK)|WL_CHANSPEC_CTL_SB_LOWER),
	(WL_CHANSPEC_BAND_2G |((WL_CHANSPEC_BW_40)&WL_CHANSPEC_BW_MASK)|WL_CHANSPEC_CTL_SB_UPPER),
};

/**
 * Return the chanspec bandwidth in MHz
 * Bandwidth of 160 MHz will be returned for 80+80MHz chanspecs.
 *
 * @param	chspec		chanspec_t
 *
 * @return	bandwidth of chspec in MHz units
 */
uint
wf_bw_chspec_to_mhz(chanspec_t chspec)
{
	uint bw;

	bw = (chspec & WL_CHANSPEC_BW_MASK) >> WL_CHANSPEC_BW_SHIFT;
	return (bw >= WF_NUM_BW ? 0 : wf_chspec_bw_mhz[bw]);
}

/* bw in MHz, return the channel count from the center channel to the
 * the channel at the edge of the band
 */
static uint8
center_chan_to_edge(uint bw)
{
	/* edge channels separated by BW - 10MHz on each side
	 * delta from cf to edge is half of that,
	 * MHz to channel num conversion is 5MHz/channel
	 */
	return (uint8)(((bw - 20) / 2) / 5);
}

/* return channel number of the low edge of the band
 * given the center channel and BW
 */
static uint8
channel_low_edge(uint center_ch, uint bw)
{
	return (uint8)(center_ch - center_chan_to_edge(bw));
}

/* return side band number given center channel and primary20 channel
 * return -1 on error
 */
static int
channel_to_sb(uint center_ch, uint primary_ch, uint bw)
{
	uint lowest = channel_low_edge(center_ch, bw);
	uint sb;

	if ((primary_ch - lowest) % 4) {
		/* bad primary channel, not mult 4 */
		return -1;
	}

	sb = ((primary_ch - lowest) / 4);

	/* sb must be a index to a 20MHz channel in range */
	if (sb >= (bw / 20)) {
		/* primary_ch must have been too high for the center_ch */
		return -1;
	}

	return (int)sb;
}

/* return primary20 channel given center channel and side band */
static uint8
channel_to_primary20_chan(uint center_ch, uint bw, uint sb)
{
	return (uint8)(channel_low_edge(center_ch, bw) + sb * 4);
}

/* return index of 80MHz channel from channel number
 * return -1 on error
 */
static int
channel_80mhz_to_id(uint ch)
{
	uint i;
	for (i = 0; i < WF_NUM_5G_80M_CHANS; i ++) {
		if (ch == wf_5g_80m_chans[i])
			return (int)i;
	}

	return -1;
}

/* wrapper function for wf_chspec_ntoa. In case of an error it puts
 * the original chanspec in the output buffer, prepended with "invalid".
 * Can be directly used in print routines as it takes care of null
 */
char *
wf_chspec_ntoa_ex(chanspec_t chspec, char *buf)
{
	if (wf_chspec_ntoa(chspec, buf) == NULL)
		snprintf(buf, CHANSPEC_STR_LEN, "invalid 0x%04x", chspec);
	return buf;
}

/* given a chanspec and a string buffer, format the chanspec as a
 * string, and return the original pointer a.
 * Min buffer length must be CHANSPEC_STR_LEN.
 * On error return NULL
 */
char *
wf_chspec_ntoa(chanspec_t chspec, char *buf)
{
	const char *band;
	uint pri_chan;

	if (wf_chspec_malformed(chspec))
		return NULL;

	band = "";

	/* check for non-default band spec */
	if ((CHSPEC_IS2G(chspec) && CHSPEC_CHANNEL(chspec) > CH_MAX_2G_CHANNEL) ||
	    (CHSPEC_IS5G(chspec) && CHSPEC_CHANNEL(chspec) <= CH_MAX_2G_CHANNEL))
		band = (CHSPEC_IS2G(chspec)) ? "2g" : "5g";

	/* primary20 channel */
	pri_chan = wf_chspec_primary20_chan(chspec);

	/* bandwidth and primary20 sideband */
	if (CHSPEC_IS20(chspec)) {
		snprintf(buf, CHANSPEC_STR_LEN, "%s%d", band, pri_chan);
	} else if (!CHSPEC_IS8080(chspec)) {
		const char *bw;
		const char *sb = "";

		bw = wf_chspec_to_bw_str(chspec);

#ifdef CHANSPEC_NEW_40MHZ_FORMAT
		/* primary20 sideband string if needed for 2g 40MHz */
		if (CHSPEC_IS40(chspec) && CHSPEC_IS2G(chspec)) {
			sb = CHSPEC_SB_UPPER(chspec) ? "u" : "l";
		}

		snprintf(buf, CHANSPEC_STR_LEN, "%s%d/%s%s", band, pri_chan, bw, sb);
#else
		/* primary20 sideband string instead of BW for 40MHz */
		if (CHSPEC_IS40(chspec)) {
			sb = CHSPEC_SB_UPPER(chspec) ? "u" : "l";
			snprintf(buf, CHANSPEC_STR_LEN, "%s%d%s", band, pri_chan, sb);
		} else {
			snprintf(buf, CHANSPEC_STR_LEN, "%s%d/%s", band, pri_chan, bw);
		}
#endif /* CHANSPEC_NEW_40MHZ_FORMAT */

	} else {
		/* 80+80 */
		uint chan1 = (chspec & WL_CHANSPEC_CHAN1_MASK) >> WL_CHANSPEC_CHAN1_SHIFT;
		uint chan2 = (chspec & WL_CHANSPEC_CHAN2_MASK) >> WL_CHANSPEC_CHAN2_SHIFT;

		/* convert to channel number */
		chan1 = (chan1 < WF_NUM_5G_80M_CHANS) ? wf_5g_80m_chans[chan1] : 0;
		chan2 = (chan2 < WF_NUM_5G_80M_CHANS) ? wf_5g_80m_chans[chan2] : 0;

		/* Outputs a max of CHANSPEC_STR_LEN chars including '\0'  */
		snprintf(buf, CHANSPEC_STR_LEN, "%d/80+80/%d-%d", pri_chan, chan1, chan2);
	}

	return (buf);
}

static int
read_uint(const char **p, unsigned int *num)
{
	unsigned long val;
	char *endp = NULL;

	val = strtoul(*p, &endp, 10);
	/* if endp is the initial pointer value, then a number was not read */
	if (endp == *p)
		return 0;

	/* advance the buffer pointer to the end of the integer string */
	*p = endp;
	/* return the parsed integer */
	*num = (unsigned int)val;

	return 1;
}

/* given a chanspec string, convert to a chanspec.
 * if bandwidth not specified in chanspec input string, then use default_bw as bandwidth.
 * On error return 0
 */
chanspec_t
wf_chspec_aton_ex(const char *a, const uint default_bw)
{
	chanspec_t chspec;
	uint chspec_ch, chspec_band, bw, chspec_bw, chspec_sb;
	uint num, pri_ch;
	uint ch1, ch2;
	char c, sb_ul = '\0';
	int i;

	bw = 20;
	chspec_sb = 0;
	chspec_ch = ch1 = ch2 = 0;

	/* parse channel num or band */
	if (!read_uint(&a, &num))
		return 0;
	/* if we are looking at a 'g', then the first number was a band */
	c = tolower(a[0]);
	if (c == 'g') {
		a++; /* consume the char */

		/* band must be "2" or "5" */
		if (num == 2)
			chspec_band = WL_CHANSPEC_BAND_2G;
		else if (num == 5)
			chspec_band = WL_CHANSPEC_BAND_5G;
		else
			return 0;

		/* read the channel number */
		if (!read_uint(&a, &pri_ch))
			return 0;

		c = tolower(a[0]);
	}
	else {
		/* first number is channel, use default for band */
		pri_ch = num;
		chspec_band = ((pri_ch <= CH_MAX_2G_CHANNEL) ?
		               WL_CHANSPEC_BAND_2G : WL_CHANSPEC_BAND_5G);
	}

	if (c == '\0') {
		/* bandwidth not specified in chanspec input string, so use default_bw bandwidth */
		chspec_bw = default_bw;
		bw = wf_bw_chspec_to_mhz(default_bw);
		goto done_read;
	}

	a ++; /* consume the 'u','l', or '/' */

	/* check 'u'/'l' */
	if (c == 'u' || c == 'l') {
		sb_ul = c;
		chspec_bw = WL_CHANSPEC_BW_40;
		goto done_read;
	}

	/* next letter must be '/' */
	if (c != '/')
		return 0;

	/* read bandwidth */
	if (!read_uint(&a, &bw))
		return 0;

	/* convert to chspec value */
	if (bw == 5) {
		chspec_bw = WL_CHANSPEC_BW_5;
	} else if (bw == 10) {
		chspec_bw = WL_CHANSPEC_BW_10;
	} else if (bw == 20) {
		chspec_bw = WL_CHANSPEC_BW_20;
	} else if (bw == 40) {
		chspec_bw = WL_CHANSPEC_BW_40;
	} else if (bw == 80) {
		chspec_bw = WL_CHANSPEC_BW_80;
	} else if (bw == 160) {
		chspec_bw = WL_CHANSPEC_BW_160;
	} else {
		return 0;
	}

	/* So far we have <band>g<chan>/<bw>
	 * Can now be followed by u/l if bw = 40,
	 * or '+80' if bw = 80, to make '80+80' bw.
	 */

	c = (char)tolower((int)a[0]);

	/* if we have a 2g/40 channel, we should have a l/u spec now */
	if (chspec_band == WL_CHANSPEC_BAND_2G && bw == 40) {
		if (c == 'u' || c == 'l') {
			a ++; /* consume the u/l char */
			sb_ul = c;
			goto done_read;
		}
	}

	/* check for 80+80 */
	if (c == '+') {
		/* 80+80 */
		const char plus80[] = "80/";

		/* must be looking at '+80/'
		 * check and consume this string.
		 */
		chspec_bw = WL_CHANSPEC_BW_8080;

		a ++; /* consume the char '+' */

		/* consume the '80/' string */
		for (i = 0; i < 3; i++) {
			if (*a++ != plus80[i]) {
				return 0;
			}
		}

		/* read primary 80MHz channel */
		if (!read_uint(&a, &ch1))
			return 0;

		/* must followed by '-' */
		if (a[0] != '-')
			return 0;
		a ++; /* consume the char */

		/* read secondary 80MHz channel */
		if (!read_uint(&a, &ch2))
			return 0;
	}

done_read:
	/* skip trailing white space */
	while (a[0] == ' ') {
		a ++;
	}

	/* must be end of string */
	if (a[0] != '\0')
		return 0;

	/* Now have all the chanspec string parts read;
	 * chspec_band, pri_ch, chspec_bw, sb_ul, ch1, ch2.
	 * chspec_band and chspec_bw are chanspec values.
	 * Need to convert pri_ch, sb_ul, and ch1,ch2 into
	 * a center channel (or two) and sideband.
	 */

	/* if a sb u/l string was given, just use that,
	 * guaranteed to be bw = 40 by sting parse.
	 */
	if (sb_ul != '\0') {
		if (sb_ul == 'l') {
			chspec_ch = UPPER_20_SB(pri_ch);
			chspec_sb = WL_CHANSPEC_CTL_SB_LLL;
		} else if (sb_ul == 'u') {
			chspec_ch = LOWER_20_SB(pri_ch);
			chspec_sb = WL_CHANSPEC_CTL_SB_LLU;
		}
	}
	/* if the bw is 20, center and sideband are trivial */
	else if (chspec_bw == WL_CHANSPEC_BW_20) {
		chspec_ch = pri_ch;
		chspec_sb = WL_CHANSPEC_CTL_SB_NONE;
	}
	/* if the bw is 40/80/160, not 80+80, a single method
	 * can be used to to find the center and sideband
	 */
	else if (chspec_bw != WL_CHANSPEC_BW_8080) {
		/* figure out primary20 sideband based on primary20 channel and bandwidth */
		const uint8 *center_ch = NULL;
		int num_ch = 0;
		int sb = -1;

		if (chspec_bw == WL_CHANSPEC_BW_40) {
			center_ch = wf_5g_40m_chans;
			num_ch = WF_NUM_5G_40M_CHANS;
		} else if (chspec_bw == WL_CHANSPEC_BW_80) {
			center_ch = wf_5g_80m_chans;
			num_ch = WF_NUM_5G_80M_CHANS;
		} else if (chspec_bw == WL_CHANSPEC_BW_160) {
			center_ch = wf_5g_160m_chans;
			num_ch = WF_NUM_5G_160M_CHANS;
		} else {
			return 0;
		}

		for (i = 0; i < num_ch; i ++) {
			sb = channel_to_sb(center_ch[i], pri_ch, bw);
			if (sb >= 0) {
				chspec_ch = center_ch[i];
				chspec_sb = (uint)(sb << WL_CHANSPEC_CTL_SB_SHIFT);
				break;
			}
		}

		/* check for no matching sb/center */
		if (sb < 0) {
			return 0;
		}
	}
	/* Otherwise, bw is 80+80. Figure out channel pair and sb */
	else {
		int ch1_id = 0, ch2_id = 0;
		int sb;

		/* look up the channel ID for the specified channel numbers */
		ch1_id = channel_80mhz_to_id(ch1);
		ch2_id = channel_80mhz_to_id(ch2);

		/* validate channels */
		if (ch1_id < 0 || ch2_id < 0)
			return 0;

		/* combine 2 channel IDs in channel field of chspec */
		chspec_ch = (((uint)ch1_id << WL_CHANSPEC_CHAN1_SHIFT) |
		             ((uint)ch2_id << WL_CHANSPEC_CHAN2_SHIFT));

		/* figure out primary 20 MHz sideband */

		/* is the primary channel contained in the 1st 80MHz channel? */
		sb = channel_to_sb(ch1, pri_ch, bw);
		if (sb < 0) {
			/* no match for primary channel 'pri_ch' in segment0 80MHz channel */
			return 0;
		}

		chspec_sb = (uint)(sb << WL_CHANSPEC_CTL_SB_SHIFT);
	}

	chspec = (chanspec_t)(chspec_ch | chspec_band | chspec_bw | chspec_sb);

	if (wf_chspec_malformed(chspec))
		return 0;

	return chspec;
}

/* given a chanspec string, convert to a chanspec.
 * On error return 0
 */
chanspec_t
wf_chspec_aton(const char *a)
{
	return wf_chspec_aton_ex(a, WL_CHANSPEC_BW_20);
}

/*
 * Verify the chanspec is using a legal set of parameters, i.e. that the
 * chanspec specified a band, bw, pri_sb and channel and that the
 * combination could be legal given any set of circumstances.
 * RETURNS: TRUE is the chanspec is malformed, false if it looks good.
 */
bool
wf_chspec_malformed(chanspec_t chanspec)
{
	uint chspec_bw = CHSPEC_BW(chanspec);
	uint chspec_ch = CHSPEC_CHANNEL(chanspec);

	if (CHSPEC_IS2G(chanspec)) {
		/* must be valid bandwidth and channel */
		if (!BW_LE40(chspec_bw) || (chspec_ch > CH_MAX_2G_CHANNEL)) {
			return TRUE;
		}
	} else if (CHSPEC_IS5G(chanspec)) {
		if (chspec_bw == WL_CHANSPEC_BW_8080) {
			uint ch1_id, ch2_id;

			/* channel IDs in 80+80 must be in range */
			ch1_id = CHSPEC_CHAN1(chanspec);
			ch2_id = CHSPEC_CHAN2(chanspec);
			if (ch1_id >= WF_NUM_5G_80M_CHANS || ch2_id >= WF_NUM_5G_80M_CHANS)
				return TRUE;

		} else if (chspec_bw == WL_CHANSPEC_BW_20 || chspec_bw == WL_CHANSPEC_BW_40 ||
		           chspec_bw == WL_CHANSPEC_BW_80 || chspec_bw == WL_CHANSPEC_BW_160) {

			if (chspec_ch > MAXCHANNEL) {
				return TRUE;
			}
		} else {
			/* invalid bandwidth */
			return TRUE;
		}
	} else if (CHSPEC_IS6G(chanspec)) {
		if (chspec_bw == WL_CHANSPEC_BW_8080) {
			uint ch1_id, ch2_id;

			/* channel IDs in 80+80 must be in range */
			ch1_id = CHSPEC_CHAN1(chanspec);
			ch2_id = CHSPEC_CHAN2(chanspec);
			if (ch1_id >= WF_NUM_6G_80M_CHANS || ch2_id >= WF_NUM_6G_80M_CHANS)
				return TRUE;

		} else if (chspec_bw == WL_CHANSPEC_BW_20 || chspec_bw == WL_CHANSPEC_BW_40 ||
		           chspec_bw == WL_CHANSPEC_BW_80 || chspec_bw == WL_CHANSPEC_BW_160) {

			if (chspec_ch > MAXCHANNEL) {
				return TRUE;
			}
		} else {
			/* invalid bandwidth */
			return TRUE;
		}
	} else {

		/* must be 2G, 5G or 6G band */
		return TRUE;
	}

	/* side band needs to be consistent with bandwidth */
	if (chspec_bw == WL_CHANSPEC_BW_20) {
		if (CHSPEC_CTL_SB(chanspec) != WL_CHANSPEC_CTL_SB_LLL)
			return TRUE;
	} else if (chspec_bw == WL_CHANSPEC_BW_40) {
		if (CHSPEC_CTL_SB(chanspec) > WL_CHANSPEC_CTL_SB_LLU)
			return TRUE;
	} else if (chspec_bw == WL_CHANSPEC_BW_80 ||
	           chspec_bw == WL_CHANSPEC_BW_8080) {
		/* both 80MHz and 80+80MHz use 80MHz side bands.
		 * 80+80 SB info is relative to the primary 80MHz sub-band.
		 */
		if (CHSPEC_CTL_SB(chanspec) > WL_CHANSPEC_CTL_SB_LUU)
			return TRUE;
	}
	else if (chspec_bw == WL_CHANSPEC_BW_160) {
		ASSERT(CHSPEC_CTL_SB(chanspec) <= WL_CHANSPEC_CTL_SB_UUU);
	}
	return FALSE;
}

/*
 * Verify the chanspec specifies a valid channel according to 802.11.
 * RETURNS: TRUE if the chanspec is a valid 802.11 channel
 */
bool
wf_chspec_valid(chanspec_t chanspec)
{
	uint chspec_bw = CHSPEC_BW(chanspec);
	uint chspec_ch = CHSPEC_CHANNEL(chanspec);

	if (wf_chspec_malformed(chanspec))
		return FALSE;

	if (CHSPEC_IS2G(chanspec)) {
		/* must be valid bandwidth and channel range */
		if (chspec_bw == WL_CHANSPEC_BW_20) {
			if (chspec_ch >= 1 && chspec_ch <= 14)
				return TRUE;
		} else if (chspec_bw == WL_CHANSPEC_BW_40) {
			if (chspec_ch >= 3 && chspec_ch <= 11)
				return TRUE;
		}
	} else if (CHSPEC_IS5G(chanspec)) {
		if (chspec_bw == WL_CHANSPEC_BW_8080) {
			uint16 ch1, ch2;

			ch1 = wf_5g_80m_chans[CHSPEC_CHAN1(chanspec)];
			ch2 = wf_5g_80m_chans[CHSPEC_CHAN2(chanspec)];

			/* the two channels must be separated by more than 80MHz by VHT req */
			if ((ch2 > ch1 + CH_80MHZ_APART) ||
			    (ch1 > ch2 + CH_80MHZ_APART))
				return TRUE;
		} else {
			const uint8 *center_ch;
			uint num_ch, i;

			if (chspec_bw == WL_CHANSPEC_BW_20 || chspec_bw == WL_CHANSPEC_BW_40) {
				center_ch = wf_5g_40m_chans;
				num_ch = WF_NUM_5G_40M_CHANS;
			} else if (chspec_bw == WL_CHANSPEC_BW_80) {
				center_ch = wf_5g_80m_chans;
				num_ch = WF_NUM_5G_80M_CHANS;
			} else if (chspec_bw == WL_CHANSPEC_BW_160) {
				center_ch = wf_5g_160m_chans;
				num_ch = WF_NUM_5G_160M_CHANS;
			} else {
				/* invalid bandwidth */
				return FALSE;
			}

			/* check for a valid center channel */
			if (chspec_bw == WL_CHANSPEC_BW_20) {
				/* We don't have an array of legal 20MHz 5G channels, but they are
				 * each side of the legal 40MHz channels.  Check the chanspec
				 * channel against either side of the 40MHz channels.
				 */
				for (i = 0; i < num_ch; i ++) {
					if (chspec_ch == (uint)LOWER_20_SB(center_ch[i]) ||
					    chspec_ch == (uint)UPPER_20_SB(center_ch[i]))
						break; /* match found */
				}

				if (i == num_ch) {
					/* check for channel 165 which is not the side band
					 * of 40MHz 5G channel
					 */
					if (chspec_ch == 165)
						i = 0;

					/* check for legacy JP channels on failure */
					if (chspec_ch == 34 || chspec_ch == 38 ||
					    chspec_ch == 42 || chspec_ch == 46)
						i = 0;
				}
			} else {
				/* check the chanspec channel to each legal channel */
				for (i = 0; i < num_ch; i ++) {
					if (chspec_ch == center_ch[i])
						break; /* match found */
				}
			}

			if (i < num_ch) {
				/* match found */
				return TRUE;
			}
		}
	}

	return FALSE;
}

/*
 * This function returns TRUE if both the chanspec can co-exist in PHY.
 * Addition to primary20 channel, the function checks for side band for 2g 40 channels
 */
bool
wf_chspec_coexist(chanspec_t chspec1, chanspec_t chspec2)
{
	bool same_primary;

	same_primary = (wf_chspec_primary20_chan(chspec1) == wf_chspec_primary20_chan(chspec2));

	if (same_primary && CHSPEC_IS2G(chspec1)) {
	    if (CHSPEC_IS40(chspec1) && CHSPEC_IS40(chspec2)) {
	        return (CHSPEC_CTL_SB(chspec1) == CHSPEC_CTL_SB(chspec2));
	    }
	}
	return same_primary;
}

/**
 * Create a 20MHz chanspec for the given band.
 *
 * This function returns a 20MHz chanspec in the given band.
 *
 * @param	channel   20MHz channel number
 * @param	band      a chanspec band (e.g. WL_CHANSPEC_BAND_2G)
 *
 * @return Returns a 20MHz chanspec, or IVNCHANSPEC in case of error.
 */
chanspec_t
wf_create_20MHz_chspec(uint channel, chanspec_band_t band)
{
	chanspec_t chspec;

	if (channel <= WL_CHANSPEC_CHAN_MASK &&
	    (band == WL_CHANSPEC_BAND_2G ||
	     band == WL_CHANSPEC_BAND_5G)) {
		chspec = band | WL_CHANSPEC_BW_20 | WL_CHANSPEC_CTL_SB_NONE | channel;
		if (!wf_chspec_valid(chspec)) {
			chspec = INVCHANSPEC;
		}
	} else {
		chspec = INVCHANSPEC;
	}

	return chspec;
}

/**
 * Return the primary 20MHz channel.
 *
 * This function returns the channel number of the primary 20MHz channel. For
 * 20MHz channels this is just the channel number. For 40MHz or wider channels
 * it is the primary 20MHz channel specified by the chanspec.
 *
 * @param	chspec    input chanspec
 *
 * @return Returns the channel number of the primary 20MHz channel
 */
uint8
wf_chspec_primary20_chan(chanspec_t chspec)
{
	uint center_chan;
	uint bw_mhz;
	uint sb;

	ASSERT(!wf_chspec_malformed(chspec));

	/* Is there a sideband ? */
	if (CHSPEC_IS20(chspec)) {
		return CHSPEC_CHANNEL(chspec);
	} else {
		sb = CHSPEC_CTL_SB(chspec) >> WL_CHANSPEC_CTL_SB_SHIFT;

		if (CHSPEC_IS8080(chspec)) {
			/* For an 80+80 MHz channel, the sideband 'sb' field is an 80 MHz sideband
			 * (LL, LU, UL, LU) for the 80 MHz frequency segment 0.
			 */
			uint chan_id = CHSPEC_CHAN1(chspec);

			bw_mhz = 80;

			/* convert from channel index to channel number */
			center_chan = wf_5g_80m_chans[chan_id];
		}
		else {
			bw_mhz = wf_bw_chspec_to_mhz(chspec);
			center_chan = CHSPEC_CHANNEL(chspec) >> WL_CHANSPEC_CHAN_SHIFT;
		}

		return (channel_to_primary20_chan(center_chan, bw_mhz, sb));
	}
}

/* given a chanspec, return the bandwidth string */
const char *
BCMRAMFN(wf_chspec_to_bw_str)(chanspec_t chspec)
{
	return wf_chspec_bw_str[(CHSPEC_BW(chspec) >> WL_CHANSPEC_BW_SHIFT)];
}

/*
 * Return the primary 20MHz chanspec of the given chanspec
 */
chanspec_t
wf_chspec_primary20_chspec(chanspec_t chspec)
{
	chanspec_t pri_chspec = chspec;
	uint8 pri_chan;

	ASSERT(!wf_chspec_malformed(chspec));

	/* Is there a sideband ? */
	if (!CHSPEC_IS20(chspec)) {
		pri_chan = wf_chspec_primary20_chan(chspec);
		pri_chspec = pri_chan | WL_CHANSPEC_BW_20;
		pri_chspec |= CHSPEC_BAND(chspec);
	}
	return pri_chspec;
}

/* return chanspec given primary 20MHz channel and bandwidth
 * return 0 on error
 */
uint16
wf_channel2chspec(uint pri_ch, uint bw)
{
	uint16 chspec;
	const uint8 *center_ch = NULL;
	int num_ch = 0;
	int sb = -1;
	int i = 0;

	chspec = ((pri_ch <= CH_MAX_2G_CHANNEL) ? WL_CHANSPEC_BAND_2G : WL_CHANSPEC_BAND_5G);

	chspec |= bw;

	if (bw == WL_CHANSPEC_BW_40) {
		center_ch = wf_5g_40m_chans;
		num_ch = WF_NUM_5G_40M_CHANS;
		bw = 40;
	} else if (bw == WL_CHANSPEC_BW_80) {
		center_ch = wf_5g_80m_chans;
		num_ch = WF_NUM_5G_80M_CHANS;
		bw = 80;
	} else if (bw == WL_CHANSPEC_BW_160) {
		center_ch = wf_5g_160m_chans;
		num_ch = WF_NUM_5G_160M_CHANS;
		bw = 160;
	} else if (bw == WL_CHANSPEC_BW_20) {
		chspec |= pri_ch;
		return chspec;
	} else {
		return 0;
	}

	for (i = 0; i < num_ch; i ++) {
		sb = channel_to_sb(center_ch[i], pri_ch, bw);
		if (sb >= 0) {
			chspec |= center_ch[i];
			chspec |= (sb << WL_CHANSPEC_CTL_SB_SHIFT);
			break;
		}
	}

	/* check for no matching sb/center */
	if (sb < 0) {
		return 0;
	}

	return chspec;
}

/*
 * This function returns the chanspec for the primary 40MHz of an 80MHz or wider channel.
 * The primary 20MHz channel of the returned 40MHz chanspec is the same as the primary 20MHz
 * channel of the input chanspec.
 */
extern chanspec_t wf_chspec_primary40_chspec(chanspec_t chspec)
{
	chanspec_t chspec40 = chspec;
	uint center_chan;
	uint sb;

	ASSERT(!wf_chspec_malformed(chspec));

	/* if the chanspec is > 80MHz, use the helper routine to find the primary 80 MHz channel */
	if (CHSPEC_IS8080(chspec) || CHSPEC_IS160(chspec)) {
		chspec = wf_chspec_primary80_chspec(chspec);
	}

	/* determine primary 40 MHz sub-channel of an 80 MHz chanspec */
	if (CHSPEC_IS80(chspec)) {
		center_chan = CHSPEC_CHANNEL(chspec);
		sb = CHSPEC_CTL_SB(chspec);

		if (sb < WL_CHANSPEC_CTL_SB_UL) {
			/* Primary 40MHz is on lower side */
			center_chan -= CH_20MHZ_APART;
			/* sideband bits are the same for LL/LU and L/U */
		} else {
			/* Primary 40MHz is on upper side */
			center_chan += CH_20MHZ_APART;
			/* sideband bits need to be adjusted by UL offset */
			sb -= WL_CHANSPEC_CTL_SB_UL;
		}

		/* Create primary 40MHz chanspec */
		chspec40 = (chanspec_t)(WL_CHANSPEC_BAND_5G | WL_CHANSPEC_BW_40 |
		            sb | center_chan);
	}

	return chspec40;
}

/*
 * Return the channel number for a given frequency and base frequency.
 * The returned channel number is relative to the given base frequency.
 * If the given base frequency is zero, a base frequency of 5 GHz is assumed for
 * frequencies from 5 - 6 GHz, and 2.407 GHz is assumed for 2.4 - 2.5 GHz.
 *
 * Frequency is specified in MHz.
 * The base frequency is specified as (start_factor * 500 kHz).
 * Constants WF_CHAN_FACTOR_2_4_G, WF_CHAN_FACTOR_5_G are defined for
 * 2.4 GHz and 5 GHz bands.
 *
 * The returned channel will be in the range [1, 14] in the 2.4 GHz band
 * and [0, 200] otherwise.
 * -1 is returned if the start_factor is WF_CHAN_FACTOR_2_4_G and the
 * frequency is not a 2.4 GHz channel, or if the frequency is not and even
 * multiple of 5 MHz from the base frequency to the base plus 1 GHz.
 *
 * Reference 802.11-2016, section 17.3.8.3 and section 16.3.6.3
 */
int
wf_mhz2channel(uint freq, uint start_factor)
{
	int ch = -1;
	uint base;
	int offset;

	/* take the default channel start frequency */
	if (start_factor == 0) {
		if (freq >= 2400 && freq <= 2500)
			start_factor = WF_CHAN_FACTOR_2_4_G;
		else if (freq >= 5000 && freq <= 6000)
			start_factor = WF_CHAN_FACTOR_5_G;
	}

	if (freq == 2484 && start_factor == WF_CHAN_FACTOR_2_4_G)
		return 14;

	base = start_factor / 2;

	/* check that the frequency is in 1GHz range of the base */
	if ((freq < base) || (freq > base + 1000))
		return -1;

	offset = (int)(freq - base);
	ch = offset / 5;

	/* check that frequency is a 5MHz multiple from the base */
	if (offset != (ch * 5))
		return -1;

	/* restricted channel range check for 2.4G */
	if (start_factor == WF_CHAN_FACTOR_2_4_G && (ch < 1 || ch > 13))
		return -1;

	return ch;
}

/*
 * Return the center frequency in MHz of the given channel and base frequency.
 * The channel number is interpreted relative to the given base frequency.
 *
 * The valid channel range is [1, 14] in the 2.4 GHz band and [0, 200] otherwise.
 * The base frequency is specified as (start_factor * 500 kHz).
 * Constants WF_CHAN_FACTOR_2_4_G, WF_CHAN_FACTOR_4_G, and WF_CHAN_FACTOR_5_G
 * are defined for 2.4 GHz, 4 GHz, and 5 GHz bands.
 * The channel range of [1, 14] is only checked for a start_factor of
 * WF_CHAN_FACTOR_2_4_G (4814 = 2407 * 2).
 * Odd start_factors produce channels on .5 MHz boundaries, in which case
 * the answer is rounded down to an integral MHz.
 * -1 is returned for an out of range channel.
 *
 * Reference 802.11-2016, section 17.3.8.3 and section 16.3.6.3
 */
int
wf_channel2mhz(uint ch, uint start_factor)
{
	int freq;

	if ((start_factor == WF_CHAN_FACTOR_2_4_G && (ch < 1 || ch > 14)) ||
	    (ch > 200))
		freq = -1;
	else if ((start_factor == WF_CHAN_FACTOR_2_4_G) && (ch == 14))
		freq = 2484;
	else
		freq = (int)(ch * 5 + start_factor / 2);

	return freq;
}

static const uint16 sidebands[] = {
	WL_CHANSPEC_CTL_SB_LLL, WL_CHANSPEC_CTL_SB_LLU,
	WL_CHANSPEC_CTL_SB_LUL, WL_CHANSPEC_CTL_SB_LUU,
	WL_CHANSPEC_CTL_SB_ULL, WL_CHANSPEC_CTL_SB_ULU,
	WL_CHANSPEC_CTL_SB_UUL, WL_CHANSPEC_CTL_SB_UUU
};

/*
 * Returns the chanspec 80Mhz channel corresponding to the following input
 * parameters
 *
 *	primary_channel - primary 20Mhz channel
 *	center_channel   - center frequecny of the 80Mhz channel
 *
 * The center_channel can be one of {42, 58, 106, 122, 138, 155}
 *
 * returns INVCHANSPEC in case of error
 */
chanspec_t
wf_chspec_80(uint8 center_channel, uint8 primary_channel)
{

	chanspec_t chanspec = INVCHANSPEC;
	chanspec_t chanspec_cur;
	uint i;

	for (i = 0; i < WF_NUM_SIDEBANDS_80MHZ; i++) {
		chanspec_cur = CH80MHZ_CHSPEC(center_channel, sidebands[i]);
		if (primary_channel == wf_chspec_primary20_chan(chanspec_cur)) {
			chanspec = chanspec_cur;
			break;
		}
	}
	/* If the loop ended early, we are good, otherwise we did not
	* find a 80MHz chanspec with the given center_channel that had a primary channel
	*matching the given primary_channel.
	*/
	return chanspec;
}

/*
 * Returns the 80+80 chanspec corresponding to the following input parameters
 *
 *    primary_20mhz - Primary 20 MHz channel
 *    chan0 - center channel number of one frequency segment
 *    chan1 - center channel number of the other frequency segment
 *
 * Parameters chan0 and chan1 are channel numbers in {42, 58, 106, 122, 138, 155}.
 * The primary channel must be contained in one of the 80MHz channels. This routine
 * will determine which frequency segment is the primary 80 MHz segment.
 *
 * Returns INVCHANSPEC in case of error.
 *
 * Refer to 802.11-2016 section 22.3.14 "Channelization".
 */
chanspec_t
wf_chspec_get8080_chspec(uint8 primary_20mhz, uint8 chan0, uint8 chan1)
{
	int sb = 0;
	uint16 chanspec = 0;
	int chan0_id = 0, chan1_id = 0;
	int seg0, seg1;

	chan0_id = channel_80mhz_to_id(chan0);
	chan1_id = channel_80mhz_to_id(chan1);

	/* make sure the channel numbers were valid */
	if (chan0_id == -1 || chan1_id == -1)
		return INVCHANSPEC;

	/* does the primary channel fit with the 1st 80MHz channel ? */
	sb = channel_to_sb(chan0, primary_20mhz, 80);
	if (sb >= 0) {
		/* yes, so chan0 is frequency segment 0, and chan1 is seg 1 */
		seg0 = chan0_id;
		seg1 = chan1_id;
	} else {
		/* no, so does the primary channel fit with the 2nd 80MHz channel ? */
		sb = channel_to_sb(chan1, primary_20mhz, 80);
		if (sb < 0) {
			/* no match for pri_ch to either 80MHz center channel */
			return INVCHANSPEC;
		}
		/* swapped, so chan1 is frequency segment 0, and chan0 is seg 1 */
		seg0 = chan1_id;
		seg1 = chan0_id;
	}

	chanspec = (uint16)((seg0 << WL_CHANSPEC_CHAN1_SHIFT) |
	            (seg1 << WL_CHANSPEC_CHAN2_SHIFT) |
	            (sb << WL_CHANSPEC_CTL_SB_SHIFT) |
	            WL_CHANSPEC_BW_8080 |
	            WL_CHANSPEC_BAND_5G);

	return chanspec;
}

/*
 * This function returns the 80Mhz channel for the given id.
 */
static uint8
wf_chspec_get80Mhz_ch(uint8 chan_80Mhz_id)
{
	if (chan_80Mhz_id < WF_NUM_5G_80M_CHANS)
		return wf_5g_80m_chans[chan_80Mhz_id];

	return 0;
}

/*
 * Returns the center channel of the primary 80 MHz sub-band of the provided chanspec
 */
uint8
wf_chspec_primary80_channel(chanspec_t chanspec)
{
	chanspec_t primary80_chspec;
	uint8 primary80_chan;

	primary80_chspec = wf_chspec_primary80_chspec(chanspec);

	if (primary80_chspec == INVCHANSPEC) {
		primary80_chan = INVCHANNEL;
	} else {
		primary80_chan = CHSPEC_CHANNEL(primary80_chspec);
	}

	return primary80_chan;
}

/*
 * Returns the center channel of the secondary 80 MHz sub-band of the provided chanspec
 */
uint8
wf_chspec_secondary80_channel(chanspec_t chanspec)
{
	chanspec_t secondary80_chspec;
	uint8 secondary80_chan;

	secondary80_chspec = wf_chspec_secondary80_chspec(chanspec);

	if (secondary80_chspec == INVCHANSPEC) {
		secondary80_chan = INVCHANNEL;
	} else {
		secondary80_chan = CHSPEC_CHANNEL(secondary80_chspec);
	}

	return secondary80_chan;
}

/*
 * Returns the chanspec for the primary 80MHz sub-band of an 160MHz or 80+80 channel
 */
chanspec_t
wf_chspec_primary80_chspec(chanspec_t chspec)
{
	chanspec_t chspec80;
	uint center_chan;
	uint sb;

	ASSERT(!wf_chspec_malformed(chspec));

	if (CHSPEC_IS80(chspec)) {
		chspec80 = chspec;
	}
	else if (CHSPEC_IS8080(chspec)) {
		sb = CHSPEC_CTL_SB(chspec);

		/* primary sub-band is stored in seg0 */
		center_chan = wf_chspec_get80Mhz_ch(CHSPEC_CHAN1(chspec));

		/* Create primary 80MHz chanspec */
		chspec80 = (chanspec_t)(WL_CHANSPEC_BAND_5G | WL_CHANSPEC_BW_80 | sb | center_chan);
	}
	else if (CHSPEC_IS160(chspec)) {
		center_chan = CHSPEC_CHANNEL(chspec);
		sb = CHSPEC_CTL_SB(chspec);

		if (sb < WL_CHANSPEC_CTL_SB_ULL) {
			/* Primary 80MHz is on lower side */
			center_chan -= CH_40MHZ_APART;
		}
		else {
			/* Primary 80MHz is on upper side */
			center_chan += CH_40MHZ_APART;
			sb -= WL_CHANSPEC_CTL_SB_ULL;
		}

		/* Create primary 80MHz chanspec */
		chspec80 = (chanspec_t)(WL_CHANSPEC_BAND_5G | WL_CHANSPEC_BW_80 | sb | center_chan);
	}
	else {
		chspec80 = INVCHANSPEC;
	}

	return chspec80;
}

/*
 * Returns the chanspec for the secondary 80MHz sub-band of an 160MHz or 80+80 channel
 */
chanspec_t
wf_chspec_secondary80_chspec(chanspec_t chspec)
{
	chanspec_t chspec80;
	uint center_chan;

	ASSERT(!wf_chspec_malformed(chspec));

	if (CHSPEC_IS8080(chspec)) {
		/* secondary sub-band is stored in seg1 */
		center_chan = wf_chspec_get80Mhz_ch(CHSPEC_CHAN2(chspec));

		/* Create secondary 80MHz chanspec */
		chspec80 = (chanspec_t)(WL_CHANSPEC_BAND_5G |
		            WL_CHANSPEC_BW_80 |
		            WL_CHANSPEC_CTL_SB_LL |
		            center_chan);
	}
	else if (CHSPEC_IS160(chspec)) {
		center_chan = CHSPEC_CHANNEL(chspec);

		if (CHSPEC_CTL_SB(chspec) < WL_CHANSPEC_CTL_SB_ULL) {
			/* Primary 80MHz is on lower side */
			center_chan -= CH_40MHZ_APART;
		}
		else {
			/* Primary 80MHz is on upper side */
			center_chan += CH_40MHZ_APART;
		}

		/* Create secondary 80MHz chanspec */
		chspec80 = (chanspec_t)(WL_CHANSPEC_BAND_5G |
		            WL_CHANSPEC_BW_80 |
		            WL_CHANSPEC_CTL_SB_LL |
		            center_chan);
	}
	else {
		chspec80 = INVCHANSPEC;
	}

	return chspec80;
}

/*
 * For 160MHz or 80P80 chanspec, set ch[0]/ch[1] to be the low/high 80 Mhz channels
 *
 * For 20/40/80MHz chanspec, set ch[0] to be the center freq, and chan[1]=-1
 */
void
wf_chspec_get_80p80_channels(chanspec_t chspec, uint8 *ch)
{

	if (CHSPEC_IS8080(chspec)) {
		ch[0] = wf_chspec_get80Mhz_ch(CHSPEC_CHAN1(chspec));
		ch[1] = wf_chspec_get80Mhz_ch(CHSPEC_CHAN2(chspec));
	}
	else if (CHSPEC_IS160(chspec)) {
		uint8 center_chan = CHSPEC_CHANNEL(chspec);
		ch[0] = center_chan - CH_40MHZ_APART;
		ch[1] = center_chan + CH_40MHZ_APART;
	}
	else {
		/* for 20, 40, and 80 Mhz */
		ch[0] = CHSPEC_CHANNEL(chspec);
		ch[1] = 0xFFu;
	}
	return;

}

#ifdef WL11AC_80P80
uint8
wf_chspec_channel(chanspec_t chspec)
{
	if (CHSPEC_IS8080(chspec)) {
		return wf_chspec_primary80_channel(chspec);
	}
	else {
		return ((uint8)((chspec) & WL_CHANSPEC_CHAN_MASK));
	}
}
#endif /* WL11AC_80P80 */

/* This routine returns the chanspec for a given operating class and
 * channel number
 */
chanspec_t
wf_channel_create_chspec_frm_opclass(uint8 opclass, uint8 channel)
{
	chanspec_t chanspec = 0;
	uint16 opclass_info = 0;
	uint16 lookupindex = 0;
	switch (opclass) {
		case 115:
			lookupindex = 1;
			break;
		case 124:
			lookupindex = 3;
			break;
		case 125:
			lookupindex = 5;
			break;
		case 81:
			lookupindex = 12;
			break;
		case 116:
			lookupindex = 22;
			break;
		case 119:
			lookupindex = 23;
			break;
		case 126:
			lookupindex = 25;
			break;
		case 83:
			lookupindex = 32;
			break;
		case 84:
			lookupindex = 33;
			break;
		default:
			lookupindex = 12;
	}

	if (lookupindex < 33) {
		opclass_info = opclass_data[lookupindex-1];
	}
	else {
		opclass_info = opclass_data[11];
	}
	chanspec = opclass_info | (uint16)channel;
	return chanspec;
}

/* This routine returns the opclass for a given chanspec */
int
wf_channel_create_opclass_frm_chspec(chanspec_t chspec)
{
	BCM_REFERENCE(chspec);
	/* TODO: Implement this function ! */
	return 12; /* opclass 12 for basic 2G channels */
}

/* Populates array with all 20MHz side bands of a given chanspec_t in the following order:
 *		primary20, secondary20, two secondary40s, four secondary80s.
 *    'chspec' is the chanspec of interest
 *    'pext' must point to an uint8 array of long enough to hold all side bands of the given chspec
 *
 * Works with 20, 40, 80, 80p80 and 160MHz chspec
 */
void
wf_get_all_ext(chanspec_t chspec, uint8 *pext)
{
#ifdef WL11N_20MHZONLY
	GET_ALL_SB(chspec, pext);
#else /* !WL11N_20MHZONLY */
	chanspec_t t = (CHSPEC_IS160(chspec) || CHSPEC_IS8080(chspec)) ? /* if bw > 80MHz */
	wf_chspec_primary80_chspec(chspec) : (chspec); /* extract primary 80 */
	/* primary20 channel as first element */
	uint8 pri_ch = (pext)[0] = wf_chspec_primary20_chan(t);
	if (CHSPEC_IS20(chspec)) return; /* nothing more to do since 20MHz chspec */
	/* 20MHz EXT */
	(pext)[1] = pri_ch + (uint8)(IS_CTL_IN_L20(t) ? CH_20MHZ_APART : -CH_20MHZ_APART);
	if (CHSPEC_IS40(chspec)) return; /* nothing more to do since 40MHz chspec */
	/* center 40MHz EXT */
	t = wf_channel2chspec((uint)(pri_ch + (IS_CTL_IN_L40(chspec) ?
		CH_40MHZ_APART : -CH_40MHZ_APART)), WL_CHANSPEC_BW_40);
	GET_ALL_SB(t, &((pext)[2])); /* get the 20MHz side bands in 40MHz EXT */
	if (CHSPEC_IS80(chspec)) return; /* nothing more to do since 80MHz chspec */
	t = CH80MHZ_CHSPEC(wf_chspec_secondary80_channel(chspec), WL_CHANSPEC_CTL_SB_LLL);
	/* get the 20MHz side bands in 80MHz EXT (secondary) */
	GET_ALL_SB(t, &((pext)[4]));
#endif /* !WL11N_20MHZONLY */
}

/*
 * Given two chanspecs, returns true if they overlap.
 * (Overlap: At least one 20MHz subband is common between the two chanspecs provided)
 */
bool wf_chspec_overlap(chanspec_t chspec0, chanspec_t chspec1)
{
	uint8 ch0, ch1;

	FOREACH_20_SB(chspec0, ch0) {
		FOREACH_20_SB(chspec1, ch1) {
			if (ABS(ch0 - ch1) < CH_20MHZ_APART) {
				return TRUE;
			}
		}
	}

	return FALSE;
}

uint8
channel_bw_to_width(chanspec_t chspec)
{
	uint8 channel_width;

	if (CHSPEC_IS80(chspec))
		channel_width = VHT_OP_CHAN_WIDTH_80;
	else if (CHSPEC_IS160(chspec))
		channel_width = VHT_OP_CHAN_WIDTH_160;
	else if (CHSPEC_IS8080(chspec))
		channel_width = VHT_OP_CHAN_WIDTH_80_80;
	else
		channel_width = VHT_OP_CHAN_WIDTH_20_40;

	return channel_width;
}
