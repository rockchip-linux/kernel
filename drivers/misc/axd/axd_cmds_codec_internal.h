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
#ifndef AXD_CMDS_CODEC_INTERNAL_H_
#define AXD_CMDS_CODEC_INTERNAL_H_

void str_append(char *dest, const char *src);

void print_timeout_msg(char *config);

/* Common option parse functions */
#define PCM_SAMPLERATE_PARAM		"samplerate"
#define PCM_CHANNELS_PARAM		"channels"
#define PCM_BITSPERSAMPLE_PARAM		"bitspersample"
#define PCM_JUSTIFICATION_PARAM		"justification"
void parse_pcm_samplerate(unsigned int samplerate, char *config);
void parse_pcm_channels(unsigned int channels, char *config);
void parse_pcm_bitspersample(unsigned int bitspersample, char *config);
void parse_pcm_justification(unsigned int justification, char *config);

#endif
