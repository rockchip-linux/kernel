/*
 * Copyright (C) 2011-2014 Imagination Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * AXD Commands API - Decoder Configuration functions
 */
#include "axd_cmds.h"
#include "axd_cmds_codec_internal.h"
#include "axd_cmds_internal.h"
#include "axd_sysfs.h"

/** PCM PASSTHROUGH (input) Config **/
static void get_pcm_config(struct axd_cmd *cmd, unsigned int pipe,
								char *config)
{
	unsigned int reg;
	unsigned int data;
	char str[32];

	reg = axd_get_decoder_pcm_samplerate_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}

	parse_pcm_samplerate(data, config);

	reg = axd_get_decoder_pcm_channels_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_pcm_channels(data, str);
	str_append(config, str);

	reg = axd_get_decoder_pcm_bitspersample_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_pcm_bitspersample(data, str);
	str_append(config, str);

	reg = axd_get_decoder_pcm_justification_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_pcm_justification(data, str);
	str_append(config, str);
}

static void set_pcm_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config)
{
	unsigned int reg;
	int data;

	if (CMP_PARAM(config, PCM_SAMPLERATE_PARAM)) {
		data = PARAM_VALUE(config, PCM_SAMPLERATE_PARAM);
		switch (data) {
		case 16000:
		case 32000:
		case 44100:
		case 48000:
		case 64000:
		case 96000:
			break;
		default:
			return;
		}
		reg = axd_get_decoder_pcm_samplerate_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, PCM_CHANNELS_PARAM)) {
		data = PARAM_VALUE(config, PCM_CHANNELS_PARAM);
		if (unlikely(data > 8 || data < 0))
			return;
		reg = axd_get_decoder_pcm_channels_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, PCM_BITSPERSAMPLE_PARAM)) {
		data = PARAM_VALUE(config, PCM_BITSPERSAMPLE_PARAM);
		switch (data) {
		case 8:
		case 16:
		case 24:
		case 32:
			break;
		default:
			return;
		}
		reg = axd_get_decoder_pcm_bitspersample_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, PCM_JUSTIFICATION_PARAM)) {
		data = PARAM_VALUE(config, PCM_JUSTIFICATION_PARAM);
		if (unlikely(data > 1 || data < 0))
			return;
		reg = axd_get_decoder_pcm_justification_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	}
}

/** MPEG (2/3) Config **/
static void parse_mpeg_numchannels(unsigned int numchannels, char *config)
{
	sprintf(config, "numchannels = %u\n", numchannels);
}
static void parse_mpeg_mlchannel(unsigned int mlchannel, char *config)
{
	sprintf(config, "mlchannel = %u\n", mlchannel);
}
static void get_mpeg_config(struct axd_cmd *cmd, unsigned int pipe,
								char *config)
{
	unsigned int reg;
	unsigned int data;
	char str[32];

	reg = axd_get_decoder_mpeg_numchannels_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_mpeg_numchannels(data, config);

	reg = axd_get_decoder_mpeg_mlchannel_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_mpeg_mlchannel(data, str);
	str_append(config, str);
}

static void set_mpeg_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config)
{
#define MPEG_NUMCHANNELS_PARAM		"numchannels"
#define MPEG_MLCHANNEL_PARAM		"mlchannel"
	unsigned int reg;
	int data;

	if (CMP_PARAM(config, MPEG_NUMCHANNELS_PARAM)) {
		data = PARAM_VALUE(config, MPEG_NUMCHANNELS_PARAM);
		if (unlikely(data > 0xFF || data < 0))
			return;
		reg = axd_get_decoder_mpeg_numchannels_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, MPEG_MLCHANNEL_PARAM)) {
		data = PARAM_VALUE(config, MPEG_MLCHANNEL_PARAM);
		if (unlikely(data > 0xFF || data < 0))
			return;
		reg = axd_get_decoder_mpeg_mlchannel_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	}
}

/** Dolby AC3 Config **/
static void parse_ac3_channels(unsigned int channels, char *config)
{
	sprintf(config, "channels = %u\n", channels);
}

static void
parse_ac3_channel_order(unsigned int channels, char *config, size_t sz)
{
	static const char * const channel[] = {
		"Left           (0)",
		"Right          (1)",
		"Centre         (2)",
		"Left Surround  (3)",
		"Right Surround (4)",
		"L.F.E.         (5)"
	};

	snprintf(config, sz, "channel-order:\n"
			"%s = %s\n"
			"%s = %s\n"
			"%s = %s\n"
			"%s = %s\n"
			"%s = %s\n"
			"%s = %s\n",
			channel[0],
			channel[channels & 0xF],
			channel[1],
			channel[(channels >> 4) & 0xF],
			channel[2],
			channel[(channels >> 8) & 0xF],
			channel[3],
			channel[(channels >> 12) & 0xF],
			channel[4],
			channel[(channels >> 16) & 0xF],
			channel[5],
			channel[(channels >> 20) & 0xF]
	);

}

static void parse_ac3_mode(unsigned int mode, char *config)
{
	static const char * const modestr[] = {
		"0 = Dual Mono Mode",
		"1 = C (1/0)",
		"2 = L,R (2/0)",
		"3 = L,C,R (3/0)",
		"4 = L,R,S (2/1)",
		"5 = L,C,R,S (3/1)",
		"6 = L,R,SL,SR,(2/2)",
		"7 = L,C,R,SL,SR,(3/2)"
	};
	static const char * const compmodestr[] = { "Line", "RF"};

	sprintf(config, "mode = %s\n"
			"compmode = %s\n",
			modestr[mode&0xf],
			compmodestr[mode >> 31]);
}

static void get_ac3_config(struct axd_cmd *cmd, unsigned int pipe,
								char *config)
{
#undef BUFF_SIZE
#define BUFF_SIZE 64

	unsigned int reg;
	unsigned int data;
	char str[BUFF_SIZE];

	reg = axd_get_decoder_ac3_channels_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_ac3_channels(data, config);

	reg = axd_get_decoder_ac3_channel_order_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_ac3_channel_order(data, str, BUFF_SIZE);
	str_append(config, str);

	reg = axd_get_decoder_ac3_mode_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_ac3_mode(data, str);
	str_append(config, str);
}

static void set_ac3_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config)
{
#define AC3_CHANNELS_PARAM		"channels"
#define AC3_CHANNEL_ORDER_PARAM		"channel-order"
#define AC3_MODE_PARAM			"mode"
#define AC3_COMPMODE_PARAM		"compmode"
	unsigned int reg;
	int data, temp[6];

	if (CMP_PARAM(config, AC3_CHANNELS_PARAM)) {
		data = PARAM_VALUE(config, AC3_CHANNELS_PARAM);
		if (unlikely(data > 6 || data < 0))
			return;
		reg = axd_get_decoder_ac3_channels_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, AC3_CHANNEL_ORDER_PARAM)) {
		sscanf(config, "channel-order=%d,%d,%d,%d,%d,%d",
				&temp[0], &temp[1], &temp[2],
				&temp[3], &temp[4], &temp[5]);
		data =  ((temp[0] & 0xF) <<  0)	|
			((temp[1] & 0xF) <<  4)	|
			((temp[2] & 0xF) <<  8) |
			((temp[3] & 0xF) << 12) |
			((temp[4] & 0xF) << 16) |
			((temp[5] & 0xF) << 20);
		reg = axd_get_decoder_ac3_channel_order_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, AC3_MODE_PARAM)) {
		temp[0] = PARAM_VALUE(config, AC3_MODE_PARAM);
		if (unlikely(temp[0] > 7 || temp[0] < 0))
			return;
		reg = axd_get_decoder_ac3_mode_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &data))
			return;
		data &= ~0xF;
		data |= temp[0] & 0xF;
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, AC3_COMPMODE_PARAM)) {
		temp[0] = PARAM_VALUE(config, AC3_COMPMODE_PARAM);
		if (unlikely(temp[0] > 1 || temp[0] < 0))
			return;
		reg = axd_get_decoder_ac3_mode_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &data))
			return;
		data &= ~0x80000000;
		data |= (temp[0] & 0x1) << 31;
		axd_write_reg(cmd, reg, data);
	} }

/** AAC Config **/
static void parse_aac_version(unsigned int version, char *config)
{
#define AAC_VERSION_0_STR		"AAC Low Complexity"
#define AAC_VERSION_1_STR		"MPEG4 LC"
#define AAC_VERSION_2_STR		"MPEG4 HE"
#define AAC_VERSION_3_STR		"MPEG4 DABPLUS"
	const char *str;

	switch (version) {
	case 0:
		str = AAC_VERSION_0_STR;
		break;
	case 1:
		str = AAC_VERSION_1_STR;
		break;
	case 2:
		str = AAC_VERSION_2_STR;
		break;
	case 3:
		str = AAC_VERSION_3_STR;
		break;
	default:
		return;
	}
	sprintf(config, "version = %s\n", str);
}
static void parse_aac_channels(unsigned int channels, char *config)
{
	sprintf(config, "channels = %u\n", channels);
}
static void parse_aac_profile(unsigned int profile, char *config)
{
#define AAC_PROFILE_0_STR		"Main Profile (MP)"
#define AAC_PROFILE_1_STR		"Low Complexity (LC)"
#define AAC_PROFILE_2_STR		"Scalable Sample Rate (SSR)"
	const char *str;

	switch (profile) {
	case 0:
		str = AAC_PROFILE_0_STR;
		break;
	case 1:
		str = AAC_PROFILE_1_STR;
		break;
	case 2:
		str = AAC_PROFILE_2_STR;
		break;
	default:
		str = "Unknown";
	}
	sprintf(config, "profile = %s\n", str);
}
static void parse_aac_streamtype(unsigned int streamtype, char *config)
{
#define AAC_STREAMTYPE_0_STR		"Auto Detect"
#define AAC_STREAMTYPE_1_STR		"ADTS"
#define AAC_STREAMTYPE_2_STR		"ADIF"
#define AAC_STREAMTYPE_3_STR		"RAW"
	const char *str;

	switch (streamtype) {
	case 0:
		str = AAC_STREAMTYPE_0_STR;
		break;
	case 1:
		str = AAC_STREAMTYPE_1_STR;
		break;
	case 2:
		str = AAC_STREAMTYPE_2_STR;
		break;
	case 3:
		str = AAC_STREAMTYPE_3_STR;
		break;
	default:
		str = "Unknown";
	}
	sprintf(config, "streamtype = %s\n", str);
}

static void parse_aac_samplerate(unsigned int samplerate, char *config)
{
	sprintf(config, "samplerate = %u\n", samplerate);
}

static void get_aac_config(struct axd_cmd *cmd, unsigned int pipe,
								char *config)
{
	unsigned int reg;
	unsigned int data;
	char str[32];

	reg = axd_get_decoder_aac_version_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_aac_version(data, config);

	reg = axd_get_decoder_aac_channels_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_aac_channels(data, str);
	str_append(config, str);

	reg = axd_get_decoder_aac_profile_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_aac_profile(data, str);
	str_append(config, str);

	reg = axd_get_decoder_aac_streamtype_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_aac_streamtype(data, str);
	str_append(config, str);

	reg = axd_get_decoder_aac_samplerate_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_aac_samplerate(data, str);
	str_append(config, str);
}

static void set_aac_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config)
{
#define AAC_VERSION_PARAM		"version"
#define AAC_CHANNELS_PARAM		"channels"
#define AAC_PROFILE_PARAM		"profile"
#define AAC_STREAMTYPE_PARAM		"streamtype"
#define AAC_SAMPLERATE_PARAM		"samplerate"
	unsigned int reg;
	int data;

	if (CMP_PARAM(config, AAC_VERSION_PARAM)) {
		data = PARAM_VALUE(config, AAC_VERSION_PARAM);
		if (unlikely(data > 3 || data < 0))
			return;
		reg = axd_get_decoder_aac_version_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, AAC_CHANNELS_PARAM)) {
		data = PARAM_VALUE(config, AAC_CHANNELS_PARAM);
		reg = axd_get_decoder_aac_channels_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, AAC_PROFILE_PARAM)) {
		data = PARAM_VALUE(config, AAC_PROFILE_PARAM);
		if (unlikely(data > 2 || data < 0))
			return;
		reg = axd_get_decoder_aac_profile_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, AAC_STREAMTYPE_PARAM)) {
		data = PARAM_VALUE(config, AAC_STREAMTYPE_PARAM);
		if (unlikely(data > 3 || data < 0))
			return;
		reg = axd_get_decoder_aac_streamtype_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, AAC_SAMPLERATE_PARAM)) {
		data = PARAM_VALUE(config, AAC_SAMPLERATE_PARAM);
		if (unlikely(data <= 0))
			return;
		reg = axd_get_decoder_aac_samplerate_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	}
}

/** Ogg Vorbis Config **/
static void get_ogg_config(struct axd_cmd *cmd, unsigned int pipe,
								char *config)
{
	strcpy(config, "Ogg Vorbis Config not supported\n");
}

static void set_ogg_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config)
{
}

/** FLAC Config **/
static void parse_flac_channels(unsigned int channels, char *config)
{
	sprintf(config, "channels = %u\n", channels);
}
static void parse_flac_samplerate(unsigned int samplerate, char *config)
{
	sprintf(config, "samplerate = %u\n", samplerate);
}
static void parse_flac_bitspersample(unsigned int bitspersample, char *config)
{
	sprintf(config, "bitspersample = %u\n", bitspersample);
}
static void parse_flac_md5checking(unsigned int md5checking, char *config)
{
	sprintf(config, "md5checking = %u\n", md5checking);
}
static void get_flac_config(struct axd_cmd *cmd, unsigned int pipe,
								char *config)
{
	unsigned int reg;
	unsigned int data;
	char str[32];

	reg = axd_get_decoder_flac_channels_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_channels(data, config);

	reg = axd_get_decoder_flac_samplerate_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_samplerate(data, str);
	str_append(config, str);

	reg = axd_get_decoder_flac_bitspersample_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_bitspersample(data, str);
	str_append(config, str);

	reg = axd_get_decoder_flac_md5checking_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_md5checking(data, str);
	str_append(config, str);
}

static void set_flac_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config)
{
#define FLAC_CHANNELS_PARAM		"channels"
#define FLAC_SAMPLERATE_PARAM		"samplerate"
#define FLAC_BITSPERSAMPLE_PARAM	"bitspersample"
#define FLAC_MD5CHECKING_PARAM		"md5checking"
	unsigned int reg;
	int data;

	if (CMP_PARAM(config, FLAC_CHANNELS_PARAM)) {
		data = PARAM_VALUE(config, FLAC_CHANNELS_PARAM);
		if (unlikely(data > 0x7 || data < 0))
			return;
		reg = axd_get_decoder_flac_channels_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, FLAC_SAMPLERATE_PARAM)) {
		data = PARAM_VALUE(config, FLAC_SAMPLERATE_PARAM);
		if (unlikely(data > 0xFFFFF || data < 0))
			return;
		reg = axd_get_decoder_flac_samplerate_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, FLAC_BITSPERSAMPLE_PARAM)) {
		data = PARAM_VALUE(config, FLAC_BITSPERSAMPLE_PARAM);
		if (unlikely(data > 0x3F || data < 0))
			return;
		reg = axd_get_decoder_flac_bitspersample_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, FLAC_MD5CHECKING_PARAM)) {
		data = PARAM_VALUE(config, FLAC_MD5CHECKING_PARAM);
		if (unlikely(data > 1 || data < 0))
			return;
		reg = axd_get_decoder_flac_md5checking_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	}
}

/** Cook Config **/
static void parse_cook_flavour(unsigned int flavour, char *config)
{
	sprintf(config, "flavour = %d\n", flavour);
}
static void get_cook_config(struct axd_cmd *cmd, unsigned int pipe,
								char *config)
{
	unsigned int reg;
	unsigned int data;

	reg = axd_get_decoder_cook_flavour_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_cook_flavour(data, config);
}

static void set_cook_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config)
{
#define COOK_FLAVOUR_PARAM		"flavour"
	unsigned int reg;
	int data;

	if (CMP_PARAM(config, COOK_FLAVOUR_PARAM)) {
		data = PARAM_VALUE(config, COOK_FLAVOUR_PARAM);
		if (unlikely(data > 29 || data < 0))
			return;
		reg = axd_get_decoder_cook_flavour_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	}
}

/** WMA Config **/
static void parse_wma_playeropt(unsigned int playeropt, char *config)
{
	sprintf(config, "playeropt = 0x%04X\n", playeropt);
}
static void parse_wma_drcsetting(unsigned int drcsetting, char *config)
{
	sprintf(config, "drcsetting = %u\n", drcsetting);
}
static void parse_wma_peakampref(unsigned int peakampref, char *config)
{
	sprintf(config, "peakampref = %u\n", peakampref);
}
static void parse_wma_rmsampref(unsigned int rmsampref, char *config)
{
	sprintf(config, "rmsampref = %u\n", rmsampref);
}
static void parse_wma_peakamptarget(unsigned int peakamptarget, char *config)
{
	sprintf(config, "peakamptarget = %u\n", peakamptarget);
}
static void parse_wma_rmsamptarget(unsigned int rmsamptarget, char *config)
{
	sprintf(config, "rmsamptarget = %u\n", rmsamptarget);
}
static void parse_wma_pcmvalidbitspersample(unsigned int pcmvalidbitspersample,
								char *config)
{
	sprintf(config, "pcmvalidbitspersample = %u\n", pcmvalidbitspersample);
}
static void parse_wma_pcmcontainersize(unsigned int pcmcontainersize,
								char *config)
{
	sprintf(config, "pcmcontainersize = %u\n", pcmcontainersize);
}
static void parse_wma_wmaformattag(unsigned int wmaformattag, char *config)
{
#define WMAFORMATTAG_0x160_STR		"std V1"
#define WMAFORMATTAG_0x161_STR		"std V2"
#define WMAFORMATTAG_0x162_STR		"Pro"
#define WMAFORMATTAG_0x163_STR		"Lossless"
	char *str;

	switch (wmaformattag) {
	case 0x160:
		str = WMAFORMATTAG_0x160_STR;
		break;
	case 0x161:
		str = WMAFORMATTAG_0x161_STR;
		break;
	case 0x162:
		str = WMAFORMATTAG_0x162_STR;
		break;
	case 0x163:
		str = WMAFORMATTAG_0x163_STR;
		break;
	default:
		return;

	}
	sprintf(config, "wmaformattag = %s\n", str);
}
static void parse_wma_wmanumchannels(unsigned int wmanumchannels, char *config)
{
	sprintf(config, "wmanumchannels = %u\n", wmanumchannels);
}
static void parse_wma_wmasamplespersec(unsigned int wmasamplespersec,
								char *config)
{
	sprintf(config, "wmasamplespersec = %u\n", wmasamplespersec);
}
static void parse_wma_wmaaveragebytespersec(unsigned int wmaaveragebytespersec,
								char *config)
{
	sprintf(config, "wmaaveragebytespersec = %u\n", wmaaveragebytespersec);
}
static void parse_wma_wmablockalign(unsigned int wmablockalign, char *config)
{
	sprintf(config, "wmablockalign = %u\n", wmablockalign);
}
static void parse_wma_wmavalidbitspersample(unsigned int wmavalidbitspersample,
								char *config)
{
	sprintf(config, "wmavalidbitspersample = %u\n", wmavalidbitspersample);
}
static void parse_wma_wmachannelmask(unsigned int wmachannelmask, char *config)
{
	sprintf(config, "wmachannelmask = %u\n", wmachannelmask);
}
static void parse_wma_wmaencodeoptions(unsigned int wmaencodeoptions,
								char *config)
{
	sprintf(config, "wmaencodeoptions = 0x%04X\n", wmaencodeoptions);
}
static void get_wma_config(struct axd_cmd *cmd, unsigned int pipe,
								char *config)
{
	unsigned int reg;
	unsigned int data;
	char str[32];

	reg = axd_get_decoder_wma_playeropt_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_wma_playeropt(data, config);

	reg = axd_get_decoder_wma_drcsetting_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_wma_drcsetting(data, str);
	str_append(config, str);

	reg = axd_get_decoder_wma_peakampref_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_wma_peakampref(data, str);
	str_append(config, str);

	reg = axd_get_decoder_wma_rmsampref_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_wma_rmsampref(data, str);
	str_append(config, str);

	reg = axd_get_decoder_wma_peakamptarget_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_wma_peakamptarget(data, str);
	str_append(config, str);

	reg = axd_get_decoder_wma_rmsamptarget_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_wma_rmsamptarget(data, str);
	str_append(config, str);

	reg = axd_get_decoder_wma_pcmvalidbitspersample_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_wma_pcmvalidbitspersample(data, str);
	str_append(config, str);

	reg = axd_get_decoder_wma_pcmcontainersize_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_wma_pcmcontainersize(data, str);
	str_append(config, str);

	reg = axd_get_decoder_wma_wmaformattag_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_wma_wmaformattag(data, str);
	str_append(config, str);

	reg = axd_get_decoder_wma_wmanumchannels_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_wma_wmanumchannels(data, str);
	str_append(config, str);

	reg = axd_get_decoder_wma_wmasamplespersec_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_wma_wmasamplespersec(data, str);
	str_append(config, str);

	reg = axd_get_decoder_wma_wmaaveragebytespersec_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_wma_wmaaveragebytespersec(data, str);
	str_append(config, str);

	reg = axd_get_decoder_wma_wmablockalign_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_wma_wmablockalign(data, str);
	str_append(config, str);

	reg = axd_get_decoder_wma_wmavalidbitspersample_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_wma_wmavalidbitspersample(data, str);
	str_append(config, str);

	reg = axd_get_decoder_wma_wmachannelmask_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_wma_wmachannelmask(data, str);
	str_append(config, str);

	reg = axd_get_decoder_wma_wmaencodeoptions_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_wma_wmaencodeoptions(data, str);
	str_append(config, str);
}

static void set_wma_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config)
{
#define WMA_PLAYEROPT_PARAM		"playeropt"
#define WMA_DRCSETTING_PARAM		"drcsetting"
#define WMA_PEAKAMPREF_PARAM		"peakampref"
#define WMA_RMSAMPREF_PARAM		"rmsampref"
#define WMA_PEAKAMPTARGET_PARAM		"peakamptarget"
#define WMA_RMSAMPTARGET_PARAM		"rmsamptarget"
#define WMA_PCMVALIDBITSPERSAMPLE_PARAM	"pcmvalidbitspersample"
#define WMA_PCMCONTAINERSIZE_PARAM	"pcmcontainersize"
#define WMA_WMAFORMATTAG_PARAM		"wmaformattag"
#define WMA_WMANUMCHANNELS_PARAM	"wmanumchannels"
#define WMA_WMASAMPLESPERSEC_PARAM	"wmasamplespersec"
#define WMA_WMAAVERAGEBYTESPERSEC_PARAM	"wmaaveragebytespersec"
#define WMA_WMABLOCKALIGN_PARAM		"wmablockalign"
#define WMA_WMAVALIDBITSPERSAMPLE_PARAM	"wmavalidbitspersample"
#define WMA_WMACHANNELMASK_PARAM	"wmachannelmask"
#define WMA_WMAENCODEOPTIONS_PARAM	"wmaencodeoptions"
	unsigned int reg;
	int data;

	if (CMP_PARAM(config, WMA_PLAYEROPT_PARAM)) {
		data = PARAM_VALUE(config, WMA_PLAYEROPT_PARAM);
		if (unlikely(data > 0xFFFF || data < 0))
			return;
		reg = axd_get_decoder_wma_playeropt_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, WMA_DRCSETTING_PARAM)) {
		data = PARAM_VALUE(config, WMA_DRCSETTING_PARAM);
		if (unlikely(data > 0xFFFFF || data < 0))
			return;
		reg = axd_get_decoder_wma_drcsetting_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, WMA_PEAKAMPREF_PARAM)) {
		data = PARAM_VALUE(config, WMA_PEAKAMPREF_PARAM);
		reg = axd_get_decoder_wma_peakampref_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, WMA_RMSAMPREF_PARAM)) {
		data = PARAM_VALUE(config, WMA_RMSAMPREF_PARAM);
		reg = axd_get_decoder_wma_rmsampref_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, WMA_PEAKAMPTARGET_PARAM)) {
		data = PARAM_VALUE(config, WMA_PEAKAMPTARGET_PARAM);
		reg = axd_get_decoder_wma_peakamptarget_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, WMA_RMSAMPTARGET_PARAM)) {
		data = PARAM_VALUE(config, WMA_RMSAMPTARGET_PARAM);
		reg = axd_get_decoder_wma_rmsamptarget_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, WMA_PCMVALIDBITSPERSAMPLE_PARAM)) {
		data = PARAM_VALUE(config, WMA_PCMVALIDBITSPERSAMPLE_PARAM);
		reg = axd_get_decoder_wma_pcmvalidbitspersample_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, WMA_PCMCONTAINERSIZE_PARAM)) {
		data = PARAM_VALUE(config, WMA_PCMCONTAINERSIZE_PARAM);
		reg = axd_get_decoder_wma_pcmcontainersize_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, WMA_WMAFORMATTAG_PARAM)) {
		data = PARAM_VALUE(config, WMA_WMAFORMATTAG_PARAM);
		if (unlikely(data > 0xFFFFF || data < 0))
			return;
		reg = axd_get_decoder_wma_wmaformattag_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, WMA_WMANUMCHANNELS_PARAM)) {
		data = PARAM_VALUE(config, WMA_WMANUMCHANNELS_PARAM);
		if (unlikely(data > 0xFFFFF || data < 0))
			return;
		reg = axd_get_decoder_wma_wmanumchannels_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, WMA_WMASAMPLESPERSEC_PARAM)) {
		data = PARAM_VALUE(config, WMA_WMASAMPLESPERSEC_PARAM);
		reg = axd_get_decoder_wma_wmasamplespersec_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, WMA_WMAAVERAGEBYTESPERSEC_PARAM)) {
		data = PARAM_VALUE(config, WMA_WMAAVERAGEBYTESPERSEC_PARAM);
		reg = axd_get_decoder_wma_wmaaveragebytespersec_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, WMA_WMABLOCKALIGN_PARAM)) {
		data = PARAM_VALUE(config, WMA_WMABLOCKALIGN_PARAM);
		if (unlikely(data > 0xFFFFF || data < 0))
			return;
		reg = axd_get_decoder_wma_wmablockalign_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, WMA_WMAVALIDBITSPERSAMPLE_PARAM)) {
		data = PARAM_VALUE(config, WMA_WMAVALIDBITSPERSAMPLE_PARAM);
		if (unlikely(data > 0xFFFFF || data < 0))
			return;
		reg = axd_get_decoder_wma_wmavalidbitspersample_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, WMA_WMACHANNELMASK_PARAM)) {
		data = PARAM_VALUE(config, WMA_WMACHANNELMASK_PARAM);
		reg = axd_get_decoder_wma_wmachannelmask_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, WMA_WMAENCODEOPTIONS_PARAM)) {
		data = PARAM_VALUE(config, WMA_WMAENCODEOPTIONS_PARAM);
		if (unlikely(data > 0xFFFFF || data < 0))
			return;
		reg = axd_get_decoder_wma_wmaencodeoptions_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	}
}



/** DDplus Config **/

static void parse_ddplus_config(unsigned int configreg, char *config)
{
	static const char * const dectype[] = {
		"0 = Reserved",
		"1 = DCV(5.1)",
		"2 = DEC7.1"
	};
	static const char * const output[] = {"0 = PCM", "1 = AC3"};

	sprintf(config,	"DecoderType = %s\n"
			"Output = %s\n"
			"OutNChans = %d\n",
			dectype[configreg & 0x3],
			output[(configreg & 0x10) >> 4],
			(configreg >> 8) & 0xFF
	);
}

static void
parse_ddplus_channel_order(unsigned int channels, char *config, size_t sz)
{
	static const char * const channel[] = {
		"Front Left      (0)",
		"Centre          (1)",
		"Front Right     (2)",
		"Left Surround   (3)",
		"Right Surround  (4)",
		"L.F.E.          (5)",
		"Surround Back L (6)",
		"Surround Back R (7)"
	};

	snprintf(config, sz, "channel-order:\n"
			"%s = %s\n"
			"%s = %s\n"
			"%s = %s\n"
			"%s = %s\n"
			"%s = %s\n"
			"%s = %s\n"
			"%s = %s\n"
			"%s = %s\n",
			channel[0],
			channel[channels & 0xF],
			channel[1],
			channel[(channels >> 4) & 0xF],
			channel[2],
			channel[(channels >> 8) & 0xF],
			channel[3],
			channel[(channels >> 12) & 0xF],
			channel[4],
			channel[(channels >> 16) & 0xF],
			channel[5],
			channel[(channels >> 20) & 0xF],
			channel[6],
			channel[(channels >> 24) & 0xF],
			channel[7],
			channel[(channels >> 28) & 0xF]
	);
}

static void get_ddplus_config(struct axd_cmd *cmd, unsigned int pipe,
								char *config)
{
#undef BUFF_SIZE
#define BUFF_SIZE 512
	unsigned int reg;
	unsigned int data;
	char str[BUFF_SIZE];

	reg = axd_get_decoder_ddplus_config_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_ddplus_config(data, config);

	reg = axd_get_decoder_ddplus_channel_order_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_ddplus_channel_order(data, str, BUFF_SIZE);
	str_append(config, str);

}

static void set_ddplus_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config)
{
#define DDP_DECTYPE_PARAM		"DecoderType"
#define DDP_OUTPUT_PARAM		"Output"
#define DDP_CHANNEL_ORDER_PARAM		"channel-order"
#define DDP_OUTNCHANS_PARAM		"OutNChans"

	unsigned int reg;
	int data, temp[8];

	if (CMP_PARAM(config, DDP_DECTYPE_PARAM)) {
		temp[0] = PARAM_VALUE(config, DDP_DECTYPE_PARAM);
		if (unlikely(temp[0] > 2 || temp[0] < 0))
			return;
		reg = axd_get_decoder_ddplus_config_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &data))
			return;
		data &= ~0x3;
		data |= temp[0] & 0x3;
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, DDP_OUTPUT_PARAM)) {
		temp[0] = PARAM_VALUE(config, DDP_OUTPUT_PARAM);
		if (unlikely(temp[0] > 1 || temp[0] < 0))
			return;
		reg = axd_get_decoder_ddplus_config_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &data))
			return;

		data &= ~0x10;
		data |= (temp[0] << 4) & 0x10;
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, DDP_CHANNEL_ORDER_PARAM)) {
		sscanf(config, "channel-order=%d,%d,%d,%d,%d,%d,%d,%d",
				&temp[0], &temp[1], &temp[2], &temp[3],
				&temp[4], &temp[5], &temp[6], &temp[7]);
		data =  ((temp[0] & 0xF) <<  0)	|
			((temp[1] & 0xF) <<  4) |
			((temp[2] & 0xF) <<  8) |
			((temp[3] & 0xF) << 12) |
			((temp[4] & 0xF) << 16) |
			((temp[5] & 0xF) << 20) |
			((temp[6] & 0xF) << 24) |
			((temp[7] & 0xF) << 28);
		reg = axd_get_decoder_ddplus_channel_order_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, DDP_OUTNCHANS_PARAM)) {
		temp[0] = PARAM_VALUE(config, DDP_OUTNCHANS_PARAM);
		if (unlikely(temp[0] > 8 || temp[0] < 0))
			return;
		reg = axd_get_decoder_ddplus_config_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &data))
			return;
		data &= ~0xFF00;
		data |= (temp[0] << 8);
		axd_write_reg(cmd, reg, data);
	}
}

/** ALAC Config **/
static void parse_alac_channels(uint8_t numchannels, char *config)
{
	sprintf(config, "channels = %u\n", numchannels);
}
static void parse_alac_depth(uint8_t bitdepth, char *config)
{
	sprintf(config, "depth = %u\n", bitdepth);
}
static void parse_alac_samplerate(uint32_t samplerate, char *config)
{
	sprintf(config, "samplerate = %u\n", samplerate);
}
static void parse_alac_framelength(uint32_t framelength, char *config)
{
	sprintf(config, "framelength = %u\n", framelength);
}
static void parse_alac_maxframebytes(uint32_t maxframebytes, char *config)
{
	sprintf(config, "maxframebytes = %u\n", maxframebytes);
}
static void parse_alac_avgbitrate(uint32_t avgbitrate, char *config)
{
	sprintf(config, "avgbitrate = %u\n", avgbitrate);
}
static void get_alac_config(struct axd_cmd *cmd,
					unsigned int pipe, char *config)
{
	unsigned int reg;
	unsigned int data;
	char str[32];

	reg = axd_get_decoder_alac_channels_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_alac_channels(data, config);

	reg = axd_get_decoder_alac_depth_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_alac_depth(data, str);
	str_append(config, str);

	reg = axd_get_decoder_alac_samplerate_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_alac_samplerate(data, str);
	str_append(config, str);

	reg = axd_get_decoder_alac_framelength_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_alac_framelength(data, str);
	str_append(config, str);

	reg = axd_get_decoder_alac_maxframebytes_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_alac_maxframebytes(data, str);
	str_append(config, str);

	reg = axd_get_decoder_alac_avgbitrate_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_alac_avgbitrate(data, str);
	str_append(config, str);
}
static void set_alac_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config)
{
#define ALAC_CHANNELS_PARAM		"channels"
#define ALAC_DEPTH_PARAM		"depth"
#define ALAC_SAMPLERATE_PARAM		"samplerate"
#define ALAC_FRAMELENGTH_PARAM		"framelength"
#define ALAC_MAXFRAMEBYTES_PARAM	"maxframebytes"
#define ALAC_AVGBITRATE_PARAM		"avgbitrate"

	unsigned int reg;
	int data;

	if (CMP_PARAM(config, ALAC_CHANNELS_PARAM)) {
		data = PARAM_VALUE(config, ALAC_CHANNELS_PARAM);
		switch (data) {
		case 1:
		case 2:
			/* TSTODO Add multichannel support if we can. */
			break;
		default:
			return;
		}
		reg = axd_get_decoder_alac_channels_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, ALAC_DEPTH_PARAM)) {
		data = PARAM_VALUE(config, ALAC_DEPTH_PARAM);
		switch (data) {
		case 16:
		case 20: /* TSTODO test vectors for this */
		case 24:
		case 32:
			break;
		default:
			return;
		}
		reg = axd_get_decoder_alac_depth_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, ALAC_SAMPLERATE_PARAM)) {
		data = PARAM_VALUE(config, ALAC_SAMPLERATE_PARAM);
		reg = axd_get_decoder_alac_samplerate_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, ALAC_FRAMELENGTH_PARAM)) {
		data = PARAM_VALUE(config, ALAC_FRAMELENGTH_PARAM);
		/* TSTODO sanitize */
		reg = axd_get_decoder_alac_framelength_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, ALAC_MAXFRAMEBYTES_PARAM)) {
		data = PARAM_VALUE(config, ALAC_MAXFRAMEBYTES_PARAM);
		/* TSTODO sanitize */
		reg = axd_get_decoder_alac_maxframebytes_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, ALAC_AVGBITRATE_PARAM)) {
		data = PARAM_VALUE(config, ALAC_AVGBITRATE_PARAM);
		/* TSTODO sanitize */
		reg = axd_get_decoder_alac_avgbitrate_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	}
}

/** SBC Config **/
static void parse_sbc_samplerate(uint8_t samplerate, char *config)
{
	sprintf(config, "samplerate = %u\n", samplerate);
}
static void parse_sbc_audiomode(uint8_t audiomode, char *config)
{
	sprintf(config, "audiomode = %u\n", audiomode);
}
static void parse_sbc_blocks(uint8_t blocks, char *config)
{
	sprintf(config, "blocks = %u\n", blocks);
}
static void parse_sbc_subbands(uint8_t subbands, char *config)
{
	sprintf(config, "subbands = %u\n", subbands);
}
static void parse_sbc_bitpool(uint8_t bitpool, char *config)
{
	sprintf(config, "bitpool = %u\n", bitpool);
}
static void parse_sbc_allocationmode(uint8_t allocationmode, char *config)
{
	sprintf(config, "allocationmode = %u\n", allocationmode);
}
static void get_sbc_config(struct axd_cmd *cmd, unsigned int pipe, char *config)
{
	unsigned int reg;
	unsigned int data;
	char str[32];

	reg = axd_get_decoder_sbc_samplerate_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_sbc_samplerate(data, config);

	reg = axd_get_decoder_sbc_audiomode_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_sbc_audiomode(data, str);
	str_append(config, str);

	reg = axd_get_decoder_sbc_blocks_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_sbc_blocks(data, str);
	str_append(config, str);

	reg = axd_get_decoder_sbc_subbands_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_sbc_subbands(data, str);
	str_append(config, str);

	reg = axd_get_decoder_sbc_bitpool_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_sbc_bitpool(data, str);
	str_append(config, str);

	reg = axd_get_decoder_sbc_allocationmode_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_sbc_allocationmode(data, str);
	str_append(config, str);
}

/** MS11 Config **/
static void parse_ms11_mode(uint32_t mode, char *config)
{
	static const char * const typestr[] = {
		"DDC",
		"DDT",
		"External PCM"
	};
	int input_type = (mode >> 0) & 0x3;
	int dual_input = (mode >> 2) & 0x1;
	int dolby_volume_258 = (mode >> 3) & 0x1;
	int ms10_mode = (mode >> 4) & 0x1;
	int main_only = (mode >> 5) & 0x1;

	sprintf(config, "input_type = %s\n"
			"dual_input = %d\n"
			"dolby_volume_258 = %d\n"
			"ms10_mode = %d\n"
			"main_only = %d\n",
		typestr[input_type], dual_input, dolby_volume_258,
		ms10_mode, main_only);
}
static void parse_ms11_common_config0(uint32_t common_config0, char *config)
{
	int drc_cut_fac_6ch = (common_config0 >> 0) & 0xFF;
	int drc_boost_fac_6ch = (common_config0 >> 8) & 0xFF;
	int drc_cut_fac_2ch = (common_config0 >> 16) & 0xFF;
	int drc_boost_fac_2ch = (common_config0 >> 24) & 0xFF;

	sprintf(config, "drc_cut_fac_6ch = %d\n"
			"drc_boost_fac_6ch = %d\n"
			"drc_cut_fac_2ch = %d\n"
			"drc_boost_fac_2ch = %d\n",
		drc_cut_fac_6ch, drc_boost_fac_6ch,
		drc_cut_fac_2ch, drc_boost_fac_2ch);
}
static void parse_ms11_common_config1(uint32_t common_config1, char *config)
{
	int downmix_type = (common_config1 >> 0) & 0x3;
	char *drc_mode = (common_config1 & 0x4) ? "RF mode" : "Line mode";
	int dual_mono = (common_config1 >> 3) & 0x3;
	int output_multichannel_enable = (common_config1 >> 5) & 0x1;
	int associated_audio_mixing = (common_config1 >> 6) & 0x1;
	int16_t user_balance_adjustment = (common_config1 >> 16) & 0xFFFF;

	sprintf(config, "downmixtype = %d\n"
			"drc_mode = %s\n"
			"dual_mono = %d\n"
			"output_multichannel_enable = %d\n"
			"associated_audio_mixing = %d\n"
			"user_balance_adjustment = %d\n",
		downmix_type, drc_mode, dual_mono, output_multichannel_enable,
		associated_audio_mixing, user_balance_adjustment);
}
static void parse_ms11_ddt_config0(uint32_t ddt_config0, char *config)
{
	int ddt_default_dialnorm = (ddt_config0 >> 0) & 0xFF;
	int ddt_transport_format = (ddt_config0 >> 8) & 0xFF;
	int ddt_mixing_mode = (ddt_config0 >> 16) & 0x1;

	sprintf(config, "ddt_default_dialnorm = %d\n"
			"ddt_transport_format = %d\n"
			"ddt_mixing_mode = %d\n",
		ddt_default_dialnorm, ddt_transport_format, ddt_mixing_mode);
}
static void parse_ms11_ddc_config0(uint32_t ddc_config0, char *config)
{
	int ddc_associated_substream = (ddc_config0 >> 0) & 0xFF;
	int ddc_out_mode = (ddc_config0 >> 8) & 0xFF;
	int ddc_out_lfe = (ddc_config0 >> 16) & 0x1;

	sprintf(config, "ddc_associated_substream = %d\n"
			"ddc_out_mode = %d\n"
			"ddc_out_lfe = %d\n",
		ddc_associated_substream, ddc_out_mode, ddc_out_lfe);
}
static void parse_ms11_ext_pcm_config0(uint32_t ext_pcm_config0, char *config)
{
	int ext_pcm_number_in_samples = (ext_pcm_config0 >> 0) & 0xFFFF;
	int ext_pcm_audio_coding_mode = (ext_pcm_config0 >> 16) & 0x3;
	int ext_pcm_lfe_present = (ext_pcm_config0 >> 18) & 0x1;
	int ext_pcm_dsur_mode = (ext_pcm_config0 >> 19) & 0x1;

	sprintf(config, "ext_pcm_number_in_samples = %d\n"
			"ext_pcm_audio_coding_mode = %d\n"
			"ext_pcm_lfe_present = %d\n"
			"ext_pcm_dsur_mode = %d\n",
		ext_pcm_number_in_samples, ext_pcm_audio_coding_mode,
		ext_pcm_lfe_present, ext_pcm_dsur_mode);
}
static void get_ms11_config(struct axd_cmd *cmd,
					unsigned int pipe, char *config)
{
	unsigned int reg;
	unsigned int data;
	unsigned int input_type;
	char str[164];

	reg = axd_get_decoder_ms11_mode_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_ms11_mode(data, config);

	input_type = data & 0x3;

	reg = axd_get_decoder_ms11_common_config0_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_ms11_common_config0(data, str);
	str_append(config, str);

	reg = axd_get_decoder_ms11_common_config1_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_ms11_common_config1(data, str);
	str_append(config, str);

	switch (input_type) {
	case 0: /* DDC */
		reg = axd_get_decoder_ms11_ddc_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &data)) {
			print_timeout_msg(config);
			return;
		}
		parse_ms11_ddc_config0(data, str);
		str_append(config, str);
		break;
	case 1: /* DDT */
		reg = axd_get_decoder_ms11_ddt_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &data)) {
			print_timeout_msg(config);
			return;
		}
		parse_ms11_ddt_config0(data, str);
		str_append(config, str);
		break;
	case 2: /* EXTERNAL PCM */
		reg = axd_get_decoder_ms11_ext_pcm_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &data)) {
			print_timeout_msg(config);
			return;
		}
		parse_ms11_ext_pcm_config0(data, str);
		str_append(config, str);
		break;
	default:
		return;
	}
}
static void set_ms11_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config)
{
#define MS11_INPUT_TYPE_PARAM			"input_type"
#define MS11_DUAL_INPUT_PARAM			"dual_input"
#define MS11_DOLBY_VOLUME_258_PARAM		"dolby_volume_258"
#define MS11_MS10_MODE_PARAM			"ms10_mode"
#define MS11_MAIN_ONLY_PARAM			"main_only"
#define MS11_DRC_CUT_FAC_6CH_PARAM		"drc_cut_fac_6ch"
#define MS11_DRC_BOOST_FAC_6CH_PARAM		"drc_boost_fac_6ch"
#define MS11_DRC_CUT_FAC_2CH_PARAM		"drc_cut_fac_2ch"
#define MS11_DRC_BOOST_FAC_2CH_PARAM		"drc_boost_fac_2ch"
#define MS11_DOWNMIX_TYPE_PARAM			"downmix_type"
#define MS11_DRC_MODE_PARAM			"drc_mode"
#define MS11_DUAL_MONO_PARAM			"dual_mono"
#define MS11_OUTPUT_MULTICHANNEL_ENABLE_PARAM	"output_multichannel_enable"
#define MS11_ASSOCIATED_AUDIO_MIXING_PARAM	"associated_audio_mixing"
#define MS11_USER_BALANCE_ADJUSTMENT_PARAM	"user_balance_adjustment"
#define MS11_DDT_DEFAULT_DIALNORM_PARAM		"ddt_default_dialnorm"
#define MS11_DDT_TRANSPORT_FORMAT_PARAM		"ddt_transport_format"
#define MS11_DDT_MIXING_MODE_PARAM		"ddt_mixing_mode"
#define MS11_DDC_ASSOCIATED_SUBSTREAM_PARAM	"ddc_associated_substream"
#define MS11_DDC_OUT_MODE_PARAM			"ddc_out_mode"
#define MS11_DDC_OUT_LFE_PARAM			"ddc_out_lfe"
#define MS11_EXT_PCM_NUMBER_IN_SAMPLES_PARAM	"ext_pcm_number_in_samples"
#define MS11_EXT_PCM_AUDIO_CODING_MODE_PARAM	"ext_pcm_audio_coding_mode"
#define MS11_EXT_PCM_LFE_PRESENT_PARAM		"ext_pcm_lfe_present"
#define MS11_EXT_PCM_DOLBY_SURROUND_MODE_PARAM	"ext_pcm_dsur_mode"

	unsigned int reg;
	int data;
	int temp = 0;

	if (CMP_PARAM(config, MS11_INPUT_TYPE_PARAM)) {
		data = PARAM_VALUE(config, MS11_INPUT_TYPE_PARAM);
		reg = axd_get_decoder_ms11_mode_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0x3 << 0);
		temp |= (data & 0x3) << 0;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_DUAL_INPUT_PARAM)) {
		data = PARAM_VALUE(config, MS11_DUAL_INPUT_PARAM);
		reg = axd_get_decoder_ms11_mode_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0x1 << 2);
		temp |= (!!data) << 2;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_DOLBY_VOLUME_258_PARAM)) {
		data = PARAM_VALUE(config, MS11_DOLBY_VOLUME_258_PARAM);
		reg = axd_get_decoder_ms11_mode_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0x1 << 3);
		temp |= (!!data) << 3;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_MS10_MODE_PARAM)) {
		data = PARAM_VALUE(config, MS11_MS10_MODE_PARAM);
		reg = axd_get_decoder_ms11_mode_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0x1 << 4);
		temp |= (!!data) << 4;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_MAIN_ONLY_PARAM)) {
		data = PARAM_VALUE(config, MS11_MAIN_ONLY_PARAM);
		reg = axd_get_decoder_ms11_mode_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0x1 << 5);
		temp |= (!!data) << 5;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_DRC_CUT_FAC_6CH_PARAM)) {
		data = PARAM_VALUE(config, MS11_DRC_CUT_FAC_6CH_PARAM);
		reg = axd_get_decoder_ms11_common_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0xFF << 0);
		temp |= (data & 0xFF) << 0;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_DRC_BOOST_FAC_6CH_PARAM)) {
		data = PARAM_VALUE(config, MS11_DRC_BOOST_FAC_6CH_PARAM);
		reg = axd_get_decoder_ms11_common_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0xFF << 8);
		temp |= (data & 0xFF) << 8;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_DRC_CUT_FAC_2CH_PARAM)) {
		data = PARAM_VALUE(config, MS11_DRC_CUT_FAC_2CH_PARAM);
		reg = axd_get_decoder_ms11_common_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0xFF << 16);
		temp |= (data & 0xFF) << 16;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_DRC_BOOST_FAC_2CH_PARAM)) {
		data = PARAM_VALUE(config, MS11_DRC_BOOST_FAC_2CH_PARAM);
		reg = axd_get_decoder_ms11_common_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0xFF << 24);
		temp |= (data & 0xFF) << 24;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_DOWNMIX_TYPE_PARAM)) {
		data = PARAM_VALUE(config, MS11_DOWNMIX_TYPE_PARAM);
		reg = axd_get_decoder_ms11_common_config1_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0x3 << 0);
		temp |= (data & 0x3) << 0;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_DRC_MODE_PARAM)) {
		data = PARAM_VALUE(config, MS11_DRC_MODE_PARAM);
		reg = axd_get_decoder_ms11_common_config1_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0x1 << 2);
		temp |= (!!data) << 2;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_DUAL_MONO_PARAM)) {
		data = PARAM_VALUE(config, MS11_DUAL_MONO_PARAM);
		reg = axd_get_decoder_ms11_common_config1_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0x3 << 3);
		temp |= (data & 0x3) << 3;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_OUTPUT_MULTICHANNEL_ENABLE_PARAM)) {
		data = PARAM_VALUE(config,
					MS11_OUTPUT_MULTICHANNEL_ENABLE_PARAM);
		reg = axd_get_decoder_ms11_common_config1_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0x1 << 5);
		temp |= (!!data) << 5;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_ASSOCIATED_AUDIO_MIXING_PARAM)) {
		data = PARAM_VALUE(config, MS11_ASSOCIATED_AUDIO_MIXING_PARAM);
		reg = axd_get_decoder_ms11_common_config1_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0x1 << 6);
		temp |= (!!data) << 6;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_USER_BALANCE_ADJUSTMENT_PARAM)) {
		data = PARAM_VALUE(config, MS11_USER_BALANCE_ADJUSTMENT_PARAM);
		reg = axd_get_decoder_ms11_common_config1_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0xFFFF << 16);
		temp |= (data & 0xFFFF) << 16;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_DDT_DEFAULT_DIALNORM_PARAM)) {
		data = PARAM_VALUE(config, MS11_DDT_DEFAULT_DIALNORM_PARAM);
		reg = axd_get_decoder_ms11_ddt_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0xFF << 0);
		temp |= (data & 0xFF) << 0;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_DDT_TRANSPORT_FORMAT_PARAM)) {
		data = PARAM_VALUE(config, MS11_DDT_TRANSPORT_FORMAT_PARAM);
		reg = axd_get_decoder_ms11_ddt_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0xFF << 8);
		temp |= (data & 0xFF) << 8;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_DDT_MIXING_MODE_PARAM)) {
		data = PARAM_VALUE(config, MS11_DDT_MIXING_MODE_PARAM);
		reg = axd_get_decoder_ms11_ddt_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0x1 << 16);
		temp |= (!!data) << 16;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_DDC_ASSOCIATED_SUBSTREAM_PARAM)) {
		data = PARAM_VALUE(config, MS11_DDC_ASSOCIATED_SUBSTREAM_PARAM);
		reg = axd_get_decoder_ms11_ddc_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0xFF << 0);
		temp |= (data & 0xFF) << 0;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_DDC_OUT_MODE_PARAM)) {
		data = PARAM_VALUE(config, MS11_DDC_OUT_MODE_PARAM);
		reg = axd_get_decoder_ms11_ddc_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0xFF << 8);
		temp |= (data & 0xFF) << 8;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_DDC_OUT_LFE_PARAM)) {
		data = PARAM_VALUE(config, MS11_DDC_OUT_LFE_PARAM);
		reg = axd_get_decoder_ms11_ddc_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0x1 << 16);
		temp |= (!!data) << 16;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_EXT_PCM_NUMBER_IN_SAMPLES_PARAM)) {
		data = PARAM_VALUE(config,
					MS11_EXT_PCM_NUMBER_IN_SAMPLES_PARAM);
		reg = axd_get_decoder_ms11_ext_pcm_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0xFFFF << 0);
		temp |= (data & 0xFFFF) << 0;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_EXT_PCM_AUDIO_CODING_MODE_PARAM)) {
		data = PARAM_VALUE(config,
					MS11_EXT_PCM_AUDIO_CODING_MODE_PARAM);
		reg = axd_get_decoder_ms11_ext_pcm_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0x3 << 16);
		temp |= (data & 0x3) << 16;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_EXT_PCM_LFE_PRESENT_PARAM)) {
		data = PARAM_VALUE(config, MS11_EXT_PCM_LFE_PRESENT_PARAM);
		reg = axd_get_decoder_ms11_ext_pcm_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0x1 << 18);
		temp |= (!!data) << 18;
		axd_write_reg(cmd, reg, temp);
	} else if (CMP_PARAM(config, MS11_EXT_PCM_DOLBY_SURROUND_MODE_PARAM)) {
		data = PARAM_VALUE(config,
					MS11_EXT_PCM_DOLBY_SURROUND_MODE_PARAM);
		reg = axd_get_decoder_ms11_ext_pcm_config0_reg(cmd, pipe);
		if (axd_read_reg(cmd, reg, &temp))
			return;
		temp &= ~(0x1 << 19);
		temp |= (!!data) << 19;
		axd_write_reg(cmd, reg, temp);
	}
}

void axd_cmd_input_get_decoder_config(struct axd_cmd *cmd, unsigned int pipe,
								char *config)
{
	unsigned int codec = axd_get_input_codec_number(cmd, pipe);

	switch (codec) {
	case 0:
		get_pcm_config(cmd, pipe, config);
		return;
	case 1:
		get_mpeg_config(cmd, pipe, config);
		return;
	case 2:
		get_ac3_config(cmd, pipe, config);
		return;
	case 3:
		get_aac_config(cmd, pipe, config);
		return;
	case 4:
		get_ogg_config(cmd, pipe, config);
		return;
	case 5:
		get_flac_config(cmd, pipe, config);
		return;
	case 6:
		get_cook_config(cmd, pipe, config);
		return;
	case 7:
		get_wma_config(cmd, pipe, config);
		return;
	case 8:
		get_ddplus_config(cmd, pipe, config);
		return;
	case 11:
		get_alac_config(cmd, pipe, config);
		return;
	case 12:
		get_ms11_config(cmd, pipe, config);
		return;
	case 13:
		get_sbc_config(cmd, pipe, config);
		return;
	default:
		*config = '\0';
		return;
	}
}

void axd_cmd_input_set_decoder_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config)
{
	unsigned int codec = axd_get_input_codec_number(cmd, pipe);

	switch (codec) {
	default:
	case 0:
		set_pcm_config(cmd, pipe, config);
		return;
	case 1:
		set_mpeg_config(cmd, pipe, config);
		return;
	case 2:
		set_ac3_config(cmd, pipe, config);
		return;
	case 3:
		set_aac_config(cmd, pipe, config);
		return;
	case 4:
		set_ogg_config(cmd, pipe, config);
		return;
	case 5:
		set_flac_config(cmd, pipe, config);
		return;
	case 6:
		set_cook_config(cmd, pipe, config);
		return;
	case 7:
		set_wma_config(cmd, pipe, config);
		return;
	case 8:
		set_ddplus_config(cmd, pipe, config);
		return;
	case 11:
		set_alac_config(cmd, pipe, config);
		return;
	case 12:
		set_ms11_config(cmd, pipe, config);
		return;
	case 13:
		/* No configuration needed; autoconfigured by stream */
		return;
	}
}
