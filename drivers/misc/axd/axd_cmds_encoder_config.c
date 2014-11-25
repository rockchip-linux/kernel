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
 * AXD Commands API - Encoder Configuration functions
 */
#include "axd_cmds.h"
#include "axd_cmds_internal.h"
#include "axd_cmds_codec_internal.h"
#include "axd_sysfs.h"

static void set_pcm_config(struct axd_cmd *cmd,
					unsigned int pipe, const char *config)
{
	unsigned int reg;
	int data;

	if (CMP_PARAM(config, PCM_BITSPERSAMPLE_PARAM)) {
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
		reg = axd_get_encoder_pcm_bitspersample_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	}
}

static void get_pcm_config(struct axd_cmd *cmd, unsigned int pipe, char *config)
{
	unsigned int reg;
	unsigned int data;

	reg = axd_get_encoder_pcm_bitspersample_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_pcm_bitspersample(data, config);
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
static void parse_flac_totalsamples(unsigned int totalsamples, char *config)
{
	sprintf(config, "totalsamples = %u\n", totalsamples);
}
static void parse_flac_blocksize(unsigned int blocksize, char *config)
{
	sprintf(config, "blocksize = %u\n", blocksize);
}
static void parse_flac_domidsidestereo(unsigned int domidsidestereo,
								char *config)
{
	sprintf(config, "domidsidestereo = %u\n", domidsidestereo);
}
static void parse_flac_loosemidsidestereo(unsigned int loosemidsidestereo,
								char *config)
{
	sprintf(config, "loosemidsidestereo = %u\n", loosemidsidestereo);
}
static void parse_flac_doexhaustivemodelsearch(
			unsigned int doexhaustivemodelsearch, char *config)
{
	sprintf(config, "doexhaustivemodelsearch = %u\n",
						doexhaustivemodelsearch);
}
static void parse_flac_minresidualpartitionorder(
			unsigned int minresidualpartitionorder, char *config)
{
	sprintf(config, "minresidualpartitionorder = %u\n",
						minresidualpartitionorder);
}
static void parse_flac_maxresidualpartitionorder(
			unsigned int maxresidualpartitionorder, char *config)
{
	sprintf(config, "maxresidualpartitionorder = %u\n",
						maxresidualpartitionorder);
}
static void parse_flac_framebytes(unsigned int framebytes, char *config)
{
	sprintf(config, "framebytes = %u\n", framebytes);
}
static void parse_flac_framecount(unsigned int framecount, char *config)
{
	sprintf(config, "framecount = %u\n", framecount);
}
static void parse_flac_samplecount(unsigned int samplecount, char *config)
{
	sprintf(config, "samplecount = %u\n", samplecount);
}
static void parse_flac_bytecount(unsigned int bytecount, char *config)
{
	sprintf(config, "bytecount = %u\n", bytecount);
}
static void get_flac_config(struct axd_cmd *cmd,
					unsigned int pipe, char *config)
{
	unsigned int reg;
	unsigned int data;
	char str[32];

	reg = axd_get_encoder_flac_channels_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_channels(data, config);

	reg = axd_get_encoder_flac_samplerate_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_samplerate(data, str);
	str_append(config, str);

	reg = axd_get_encoder_flac_bitspersample_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_bitspersample(data, str);
	str_append(config, str);

	reg = axd_get_encoder_flac_totalsamples_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_totalsamples(data, str);
	str_append(config, str);

	reg = axd_get_encoder_flac_blocksize_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_blocksize(data, str);
	str_append(config, str);

	reg = axd_get_encoder_flac_domidsidestereo_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_domidsidestereo(data, str);
	str_append(config, str);

	reg = axd_get_encoder_flac_loosemidsidestereo_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_loosemidsidestereo(data, str);
	str_append(config, str);

	reg = axd_get_encoder_flac_doexhaustivemodelsearch_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_doexhaustivemodelsearch(data, str);
	str_append(config, str);

	reg = axd_get_encoder_flac_minresidualpartitionorder_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_minresidualpartitionorder(data, str);
	str_append(config, str);

	reg = axd_get_encoder_flac_maxresidualpartitionorder_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_maxresidualpartitionorder(data, str);
	str_append(config, str);

	reg = axd_get_encoder_flac_framebytes_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_framebytes(data, str);
	str_append(config, str);

	reg = axd_get_encoder_flac_framecount_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_framecount(data, str);
	str_append(config, str);

	reg = axd_get_encoder_flac_samplecount_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_samplecount(data, str);
	str_append(config, str);

	reg = axd_get_encoder_flac_bytecount_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_flac_bytecount(data, str);
	str_append(config, str);
}

static void set_flac_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config)
{
#define FLAC_CHANNELS_PARAM			"channels"
#define FLAC_SAMPLERATE_PARAM			"samplerate"
#define FLAC_BITSPERSAMPLE_PARAM		"bitspersample"
#define FLAC_TOTALSAMPLES_PARAM			"totalsamples"
#define FLAC_BLOCKSIZE_PARAM			"blocksize"
#define FLAC_DOMIDSIDESTEREO_PARAM		"domidsidestereo"
#define FLAC_LOOSEMIDSIDESTEREO_PARAM		"loosemidsidestereo"
#define FLAC_DOEXHAUSTIVEMODELSEARCH_PARAM	"doexhaustivemodelsearch"
#define FLAC_MINRESIDUALPARTITIONORDER_PARAM	"minresidualpartitionorder"
#define FLAC_MAXRESIDUALPARTITIONORDER_PARAM	"maxresidualpartitionorder"
#define FLAC_FRAMEBYTES_PARAM			"frameBytes"
#define FLAC_FRAMECOUNT_PARAM			"frameCount"
#define FLAC_SAMPLECOUNT_PARAM			"sampleCount"
#define FLAC_BYTECOUNT_PARAM			"byteCount"
	unsigned int reg;
	int data;

	if (CMP_PARAM(config, FLAC_CHANNELS_PARAM)) {
		data = PARAM_VALUE(config, FLAC_CHANNELS_PARAM);
		if (unlikely(data < 1 || data > 8))
			return;
		reg = axd_get_encoder_flac_channels_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, FLAC_SAMPLERATE_PARAM)) {
		data = PARAM_VALUE(config, FLAC_SAMPLERATE_PARAM);
		if (unlikely(data < 1 || data > 655350))
			return;
		reg = axd_get_encoder_flac_samplerate_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, FLAC_BITSPERSAMPLE_PARAM)) {
		data = PARAM_VALUE(config, FLAC_BITSPERSAMPLE_PARAM);
		if (unlikely(data < 8 || data > 24))
			return;
		reg = axd_get_encoder_flac_bitspersample_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, FLAC_TOTALSAMPLES_PARAM)) {
		data = PARAM_VALUE(config, FLAC_TOTALSAMPLES_PARAM);
		reg = axd_get_encoder_flac_totalsamples_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, FLAC_BLOCKSIZE_PARAM)) {
		data = PARAM_VALUE(config, FLAC_BLOCKSIZE_PARAM);
		if (unlikely(data != 4096))
			return;
		reg = axd_get_encoder_flac_blocksize_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, FLAC_DOMIDSIDESTEREO_PARAM)) {
		data = PARAM_VALUE(config, FLAC_DOMIDSIDESTEREO_PARAM);
		if (unlikely(data < 0 || data > 1))
			return;
		reg = axd_get_encoder_flac_domidsidestereo_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, FLAC_LOOSEMIDSIDESTEREO_PARAM)) {
		data = PARAM_VALUE(config, FLAC_LOOSEMIDSIDESTEREO_PARAM);
		if (unlikely(data < 0 || data > 1))
			return;
		reg = axd_get_encoder_flac_loosemidsidestereo_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, FLAC_DOEXHAUSTIVEMODELSEARCH_PARAM)) {
		data = PARAM_VALUE(config, FLAC_DOEXHAUSTIVEMODELSEARCH_PARAM);
		if (unlikely(data < 0 || data > 1))
			return;
		reg = axd_get_encoder_flac_doexhaustivemodelsearch_reg(cmd,
									pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, FLAC_MINRESIDUALPARTITIONORDER_PARAM)) {
		data = PARAM_VALUE(config,
					FLAC_MINRESIDUALPARTITIONORDER_PARAM);
		if (unlikely(data < 0 || data > 6)) /* Actual upper limit: 16 */
			return;
		reg = axd_get_encoder_flac_minresidualpartitionorder_reg(cmd,
									pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, FLAC_MAXRESIDUALPARTITIONORDER_PARAM)) {
		data = PARAM_VALUE(config,
					FLAC_MAXRESIDUALPARTITIONORDER_PARAM);
		if (unlikely(data < 0 || data > 6)) /* Actual upper limit: 16 */
			return;
		reg = axd_get_encoder_flac_maxresidualpartitionorder_reg(cmd,
									pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, FLAC_FRAMEBYTES_PARAM)) {
		/* Read-only */
		return;
	} else if (CMP_PARAM(config, FLAC_FRAMECOUNT_PARAM)) {
		/* Read-only */
		return;
	} else if (CMP_PARAM(config, FLAC_SAMPLECOUNT_PARAM)) {
		/* Read-only */
		return;
	} else if (CMP_PARAM(config, FLAC_BYTECOUNT_PARAM)) {
		/* Read-only */
		return;
	}

}

/** ALAC Config **/
static void parse_alac_channels(unsigned int channels, char *config)
{
	sprintf(config, "channels = %u\n", channels);
}
static void parse_alac_depth(unsigned int depth, char *config)
{
	sprintf(config, "depth = %u\n", depth);
}
static void parse_alac_samplerate(unsigned int samplerate, char *config)
{
	sprintf(config, "samplerate = %u\n", samplerate);
}
static void parse_alac_framelength(unsigned int framelength, char *config)
{
	sprintf(config, "framelength = %u\n", framelength);
}
static void parse_alac_maxframebytes(unsigned int maxframebytes, char *config)
{
	sprintf(config, "maxframebytes = %u\n", maxframebytes);
}
static void parse_alac_avgbitrate(unsigned int avgbitrate, char *config)
{
	sprintf(config, "avgbitrate = %u\n", avgbitrate);
}
static void parse_alac_fastmode(unsigned int fastmode, char *config)
{
	sprintf(config, "fastmode = %u\n", fastmode);
}

static void get_alac_config(struct axd_cmd *cmd,
					unsigned int pipe, char *config)
{
	unsigned int reg;
	unsigned int data;
	char str[32];

	reg = axd_get_encoder_alac_channels_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_alac_channels(data, config);

	reg = axd_get_encoder_alac_depth_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_alac_depth(data, str);
	str_append(config, str);

	reg = axd_get_encoder_alac_samplerate_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_alac_samplerate(data, str);
	str_append(config, str);

	reg = axd_get_encoder_alac_framelength_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_alac_framelength(data, str);
	str_append(config, str);

	reg = axd_get_encoder_alac_maxframebytes_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_alac_maxframebytes(data, str);
	str_append(config, str);

	reg = axd_get_encoder_alac_avgbitrate_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_alac_avgbitrate(data, str);
	str_append(config, str);

	reg = axd_get_encoder_alac_fastmode_reg(cmd, pipe);
	if (axd_read_reg(cmd, reg, &data)) {
		print_timeout_msg(config);
		return;
	}
	parse_alac_fastmode(data, str);
	str_append(config, str);
}

static void set_alac_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config)
{
#define ALAC_CHANNELS_PARAM			"channels"
#define ALAC_DEPTH_PARAM			"depth"
#define ALAC_SAMPLERATE_PARAM			"samplerate"
#define ALAC_FRAMELENGTH_PARAM			"framelength"
#define ALAC_MAXFRAMEBYTES_PARAM		"maxframebytes"
#define ALAC_AVGBITRATE_PARAM			"avgbitrate"
#define ALAC_FASTMODE_PARAM			"fastmode"
	unsigned int reg;
	int data;

	if (CMP_PARAM(config, ALAC_CHANNELS_PARAM)) {
		data = PARAM_VALUE(config, ALAC_CHANNELS_PARAM);
		if (unlikely(data < 1 || data > 8))
			return;
		reg = axd_get_encoder_alac_channels_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, ALAC_DEPTH_PARAM)) {
		data = PARAM_VALUE(config, ALAC_DEPTH_PARAM);
		if (unlikely(data < 8 || data > 32))
			return;
		reg = axd_get_encoder_alac_depth_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, ALAC_SAMPLERATE_PARAM)) {
		data = PARAM_VALUE(config, ALAC_SAMPLERATE_PARAM);
		/* TSTODO validate */
		reg = axd_get_encoder_alac_samplerate_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, ALAC_FRAMELENGTH_PARAM)) {
		data = PARAM_VALUE(config, ALAC_FRAMELENGTH_PARAM);
		/* TSTODO validate */
		reg = axd_get_encoder_alac_framelength_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, ALAC_MAXFRAMEBYTES_PARAM)) {
		data = PARAM_VALUE(config, ALAC_MAXFRAMEBYTES_PARAM);
		/* TSTODO validate */
		reg = axd_get_encoder_alac_maxframebytes_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, ALAC_AVGBITRATE_PARAM)) {
		data = PARAM_VALUE(config, ALAC_AVGBITRATE_PARAM);
		/* TSTODO validate */
		reg = axd_get_encoder_alac_avgbitrate_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	} else if (CMP_PARAM(config, ALAC_FASTMODE_PARAM)) {
		data = PARAM_VALUE(config, ALAC_FASTMODE_PARAM);
		if (unlikely(data < 0 || data > 1))
			return;
		reg = axd_get_encoder_alac_fastmode_reg(cmd, pipe);
		axd_write_reg(cmd, reg, data);
	}
}

void axd_cmd_output_get_encoder_config(struct axd_cmd *cmd, unsigned int pipe,
								char *config)
{
	unsigned int codec = axd_get_output_codec_number(cmd, pipe);

	switch (codec) {
	case 0:
		get_pcm_config(cmd, pipe, config);
		return;
	case 5:
		get_flac_config(cmd, pipe, config);
		return;
	case 11:
		get_alac_config(cmd, pipe, config);
		return;
	default:
		*config = '\0';
		return;
	}
}

void axd_cmd_output_set_encoder_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config)
{
	unsigned int codec = axd_get_output_codec_number(cmd, pipe);

	switch (codec) {
	default:
	case 0:
		set_pcm_config(cmd, pipe, config);
		return;
	case 5:
		set_flac_config(cmd, pipe, config);
		return;
	case 11:
		set_alac_config(cmd, pipe, config);
		return;
	}
}
