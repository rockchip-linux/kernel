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
 * Common functionality required by the codec config files.
 */
#include "axd_cmds.h"
#include "axd_cmds_codec_internal.h"

void str_append(char *dest, const char *src)
{
	int len = strlen(dest);

	dest += len;
	strcpy(dest, src);
}

void print_timeout_msg(char *config)
{
	strcpy(config, "Read Timeout\n\0");
}

/* Common decoder/encoder option parse functions */

void parse_pcm_samplerate(unsigned int samplerate, char *config)
{
	sprintf(config, "samplerate = %u\n", samplerate);
}
void parse_pcm_channels(unsigned int channels, char *config)
{
	sprintf(config, "channels = %u\n", channels);
}
void parse_pcm_bitspersample(unsigned int bitspersample, char *config)
{
	sprintf(config, "bitspersample = %u\n", bitspersample);
}
void parse_pcm_justification(unsigned int justification, char *config)
{
#define PCM_JUSTIFICATION_LEFT_STR	"Left"
#define PCM_JUSTIFICATION_RIGHT_STR	"Right"
	const char *str;

	switch (justification) {
	case 0:
		str = PCM_JUSTIFICATION_LEFT_STR;
		break;
	case 1:
		str = PCM_JUSTIFICATION_RIGHT_STR;
		break;
	default:
		return;
	}
	sprintf(config, "justification = %s\n", str);
}
