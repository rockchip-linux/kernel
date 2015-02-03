/*
 * dw-hdmi-audio.h -- DW HDMI ALSA SoC Audio driver
 *
 * Copyright 2011-2012 DesignerWare Products
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DW_HDMI_AUDIO_H
#define _DW_HDMI_AUDIO_H

enum hdmi_audio_reg {
	HDMI_PHY_STAT0 = 0x3004,
	HDMI_AUD_CONF0 = 0x3100,
	HDMI_AUD_CONF1 = 0x3101,
	HDMI_AUD_INPUTCLKFS = 0x3206,
	HDMI_MC_CLKDIS = 0x4001,
};

enum {
	HDMI_PHY_HPD = 0x2,
	HDMI_MC_CLKDIS_AUDCLK_DISABLE = 0x8,
};

enum hdmi_audio_samplerate {
	AUDIO_SAMPLERATE_32K = 32000,
	AUDIO_SAMPLERATE_44K1 = 44100,
	AUDIO_SAMPLERATE_48K = 48000,
	AUDIO_SAMPLERATE_88K2 = 88200,
	AUDIO_SAMPLERATE_96K = 96000,
	AUDIO_SAMPLERATE_176K4 = 176400,
	AUDIO_SAMPLERATE_192K = 192000,
};

#define AUDIO_CONF1_DATWIDTH_MSK	0x1F
enum hdmi_audio_wordlength {
	AUDIO_WORDLENGTH_16BIT = 16,
	AUDIO_WORDLENGTH_24BIT = 24,
};

#define AUDIO_CONF1_DATAMODE_MSK	0xE0
enum hdmi_audio_daifmt {
	AUDIO_DAIFMT_IIS = 0x00,
	AUDIO_DAIFMT_RIGHT_J = 0x20,
	AUDIO_DAIFMT_LEFT_J = 0x40,
	AUDIO_DAIFMT_BURST_1 = 0x60,
	AUDIO_DAIFMT_BURST_2 = 0x80,
};

#define AUDIO_CONF0_INTERFACE_MSK	0x20
enum hdmi_audio_inputtype {
	AUDIO_INPUTTYPE_IIS = 0x20,
	AUDIO_INPUTTYPE_SPDIF = 0x00,
};

#define AUDIO_CONF0_I2SINEN_MSK		0x0F
enum hdmi_audio_channelnum {
	AUDIO_CHANNELNUM_2 = 0x01,
	AUDIO_CHANNELNUM_4 = 0x03,
	AUDIO_CHANNELNUM_6 = 0x07,
	AUDIO_CHANNELNUM_8 = 0x0F,
};

enum hdmi_jack_status {
	JACK_LINEOUT,
	JACK_NO_LINEOUT,
};

struct hdmi_audio_fmt {
	enum hdmi_audio_inputtype input_type;
	enum hdmi_audio_channelnum chan_num;
	enum hdmi_audio_samplerate sample_rate;
	enum hdmi_audio_wordlength word_length;
	enum hdmi_audio_daifmt dai_fmt;
};

#endif
