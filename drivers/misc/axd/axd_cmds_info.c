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
 * AXD Commands API - Info functions.
 */
#include "axd_cmds.h"
#include "axd_cmds_internal.h"

/* static functions */
/* Fills @source with the source as set in the bit map @bits */
static void parse_source(unsigned int bits, char *source)
{
#define SOURCE_PIPE_STR		"pipe\n"
#define SOURCE_AUX_STR		"aux\n"

	switch (bits) {
	case 0:
		strcpy(source, SOURCE_PIPE_STR);
		source += sizeof(SOURCE_PIPE_STR);
		source--; /* terminating null */
		break;
	case 1:
		strcpy(source, SOURCE_AUX_STR);
		source += sizeof(SOURCE_AUX_STR);
		source--; /* terminating null */
		break;
	default:
		break;
	}
	*source = '\0';
}

/* Fills @sink with the sink as set in the bit map @bits */
static void parse_sink(unsigned int bits, char *sink)
{
#define SINK_PIPE_STR		"pipe\n"
#define SINK_I2S_STR		"i2s\n"

	switch (bits) {
	case 0:
		strcpy(sink, SINK_PIPE_STR);
		sink += sizeof(SINK_PIPE_STR);
		sink--; /* terminating null */
		break;
	case 1:
		strcpy(sink, SINK_I2S_STR);
		sink += sizeof(SINK_I2S_STR);
		sink--; /* terminating null */
		break;
	default:
		break;
	}
	*sink = '\0';
}

/* Fills @codec with the list of codecs as set in the bit map @bits */
static void parse_codec_by_bit(unsigned int bits, char *codecs)
{
#define CODEC_BIT0_MASK		(1 << 0)
#define CODEC_BIT0_STR		"PCM Pass through\n"
#define CODEC_BIT1_MASK		(1 << 1)
#define CODEC_BIT1_STR		"MPEG (MP2/3)\n"
#define CODEC_BIT2_MASK		(1 << 2)
#define CODEC_BIT2_STR		"Dolby AC3\n"
#define CODEC_BIT3_MASK		(1 << 3)
#define CODEC_BIT3_STR		"AAC\n"
#define CODEC_BIT4_MASK		(1 << 4)
#define CODEC_BIT4_STR		"Ogg Vorbis\n"
#define CODEC_BIT5_MASK		(1 << 5)
#define CODEC_BIT5_STR		"FLAC\n"
#define CODEC_BIT6_MASK		(1 << 6)
#define CODEC_BIT6_STR		"Cook\n"
#define CODEC_BIT7_MASK		(1 << 7)
#define CODEC_BIT7_STR		"WMA\n"
#define CODEC_BIT8_MASK		(1 << 8)
#define CODEC_BIT8_STR		"Dolby Digital Plus\n"
#define CODEC_BIT11_MASK	(1 << 11)
#define CODEC_BIT11_STR		"ALAC\n"
#define CODEC_BIT12_MASK	(1 << 12)
#define CODEC_BIT12_STR		"MS11\n"
#define CODEC_BIT13_MASK	(1 << 13)
#define CODEC_BIT13_STR		"SBC\n"

	if (bits & CODEC_BIT0_MASK) {
		strcpy(codecs, CODEC_BIT0_STR);
		codecs += sizeof(CODEC_BIT0_STR);
		codecs--; /* terminating null */
	}
	if (bits & CODEC_BIT1_MASK) {
		strcpy(codecs, CODEC_BIT1_STR);
		codecs += sizeof(CODEC_BIT1_STR);
		codecs--; /* terminating null */
	}
	if (bits & CODEC_BIT2_MASK) {
		strcpy(codecs, CODEC_BIT2_STR);
		codecs += sizeof(CODEC_BIT2_STR);
		codecs--; /* terminating null */
	}
	if (bits & CODEC_BIT3_MASK) {
		strcpy(codecs, CODEC_BIT3_STR);
		codecs += sizeof(CODEC_BIT3_STR);
		codecs--; /* terminating null */
	}
	if (bits & CODEC_BIT4_MASK) {
		strcpy(codecs, CODEC_BIT4_STR);
		codecs += sizeof(CODEC_BIT4_STR);
		codecs--; /* terminating null */
	}
	if (bits & CODEC_BIT5_MASK) {
		strcpy(codecs, CODEC_BIT5_STR);
		codecs += sizeof(CODEC_BIT5_STR);
		codecs--; /* terminating null */
	}
	if (bits & CODEC_BIT6_MASK) {
		strcpy(codecs, CODEC_BIT6_STR);
		codecs += sizeof(CODEC_BIT6_STR);
		codecs--; /* terminating null */
	}
	if (bits & CODEC_BIT7_MASK) {
		strcpy(codecs, CODEC_BIT7_STR);
		codecs += sizeof(CODEC_BIT7_STR);
		codecs--; /* terminating null */
	}
	if (bits & CODEC_BIT8_MASK) {
		strcpy(codecs, CODEC_BIT8_STR);
		codecs += sizeof(CODEC_BIT8_STR);
		codecs--; /* terminating null */
	}
	if (bits & CODEC_BIT11_MASK) {
		strcpy(codecs, CODEC_BIT11_STR);
		codecs += sizeof(CODEC_BIT11_STR);
		codecs--; /* terminating null */
	}
	if (bits & CODEC_BIT12_MASK) {
		strcpy(codecs, CODEC_BIT12_STR);
		codecs += sizeof(CODEC_BIT12_STR);
		codecs--; /* terminating null */
	}
	if (bits & CODEC_BIT13_MASK) {
		strcpy(codecs, CODEC_BIT13_STR);
		codecs += sizeof(CODEC_BIT13_STR);
		codecs--; /* terminating null */
	}
	*codecs = '\0';
}

/* Fills @codec with the codec corresponding to @codec_num */
static void parse_codec_by_number(unsigned int codec_num, char *codecs)
{
	parse_codec_by_bit(1 << codec_num, codecs);
}

/* Fills @upmix by the current upmix setting */
static void parse_upmix(unsigned int setting, char *upmix)
{
#define UPMIX_0_STR		"Pass through\n"
#define UPMIX_1_STR		"Simple 5.1\n"
#define UPMIX_2_STR		"Dolby Pro Logic 2\n"

	switch (setting) {
	case 0:
		strcpy(upmix, UPMIX_0_STR);
		upmix += sizeof(UPMIX_0_STR);
		upmix--; /* terminating null */
		break;
	case 1:
		strcpy(upmix, UPMIX_1_STR);
		upmix += sizeof(UPMIX_1_STR);
		upmix--; /* terminating null */
		break;
	case 2:
		strcpy(upmix, UPMIX_2_STR);
		upmix += sizeof(UPMIX_2_STR);
		upmix--; /* terminating null */
		break;
	default:
		break;
	}
	*upmix = '\0';
}

/* Fills @downmix by the current downmix setting */
static void parse_downmix(unsigned int setting, char *downmix)
{
#define DOWNMIX_0_STR		"Pass through\n"
#define DOWNMIX_1_STR		"5.1\n"
#define DOWNMIX_2_STR		"2.0\n"

	switch (setting) {
	case 0:
		strcpy(downmix, DOWNMIX_0_STR);
		downmix += sizeof(DOWNMIX_0_STR);
		downmix--; /* terminating null */
		break;
	case 1:
		strcpy(downmix, DOWNMIX_1_STR);
		downmix += sizeof(DOWNMIX_1_STR);
		downmix--; /* terminating null */
		break;
	case 2:
		strcpy(downmix, DOWNMIX_2_STR);
		downmix += sizeof(DOWNMIX_2_STR);
		downmix--; /* terminating null */
		break;
	default:
		break;
	}
	*downmix = '\0';
}

/* Fills @mux by the current setting of mixer's output @pipe mux */
static void parse_mux(unsigned int setting, char *mux)
{
#define MUX_0_STR		"mix\n"
#define MUX_1_STR		"input 0\n"
#define MUX_2_STR		"input 1\n"
#define MUX_3_STR		"input 2\n"
#define MUX_4_STR		"input 3\n"

	switch (setting) {
	case 0:
		strcpy(mux, MUX_0_STR);
		mux += sizeof(MUX_0_STR);
		mux--; /* terminating null */
		break;
	case 1:
		strcpy(mux, MUX_1_STR);
		mux += sizeof(MUX_1_STR);
		mux--; /* terminating null */
		break;
	case 2:
		strcpy(mux, MUX_2_STR);
		mux += sizeof(MUX_2_STR);
		mux--; /* terminating null */
		break;
	case 3:
		strcpy(mux, MUX_3_STR);
		mux += sizeof(MUX_3_STR);
		mux--; /* terminating null */
		break;
	case 4:
		strcpy(mux, MUX_4_STR);
		mux += sizeof(MUX_4_STR);
		mux--; /* terminating null */
		break;
	default:
		break;
	}
	*mux = '\0';
}

/* Info API */
/* Sets the major and minor numbers of the currently running AXD firmware */
void axd_cmd_get_version(struct axd_cmd *cmd,
					int *major, int *minor, int *patch)
{
	unsigned int version;

	axd_read_reg(cmd, AXD_REG_VERSION, &version);
	if (unlikely(!major || !minor))
		return;
	*major = (version >> 22);		/* top 10 bits */
	*minor = (version >> 12) & 0x3FF;	/* middle 10 bits */
	*patch = version & 0xFFF;		/* bottom 12 bits */
}

/* Sets the number of supported in/out pipes */
int axd_cmd_get_num_pipes(struct axd_cmd *cmd,
				unsigned int *inpipes, unsigned int *outpipes)
{
	unsigned int config0;
	int ret;

	ret = axd_read_reg(cmd, AXD_REG_CONFIG0, &config0);
	if (unlikely(!inpipes || !outpipes))
		return -1;
	if (ret)
		return -1;
	*inpipes = config0 >> 16;
	*inpipes &= 0xFF;
	*outpipes = config0 & 0xFF;
	return 0;
}

/* Fills @codecs with a list of supported codecs */
void axd_cmd_get_decoders(struct axd_cmd *cmd, char *codecs)
{
	unsigned int config1;

	axd_read_reg(cmd, AXD_REG_CONFIG1, &config1);
	if (unlikely(!codecs))
		return;
	parse_codec_by_bit(config1, codecs);
}

/* Fills @codecs with a list of supported codecs */
void axd_cmd_get_encoders(struct axd_cmd *cmd, char *codecs)
{
	unsigned int config2;

	axd_read_reg(cmd, AXD_REG_CONFIG2, &config2);
	if (unlikely(!codecs))
		return;
	parse_codec_by_bit(config2, codecs);
}

/* Returns non-zero if Mix/Xbar is present. Zero otherwise. */
int axd_cmd_xbar_present(struct axd_cmd *cmd)
{
	unsigned int temp;

	axd_read_reg(cmd, AXD_REG_CONFIG3, &temp);
	return temp & 0x1;
}

/* Returns non-zero if mixer EQ is enabled. Zero otherwise. */
int axd_cmd_mixer_get_eqenabled(struct axd_cmd *cmd, unsigned int pipe)
{
	unsigned int control;

	axd_read_reg(cmd, AXD_REG_EQ_CTRL_GAIN, &control);
	return (control & AXD_EQCTRL_ENABLE_BITS) >> AXD_EQCTRL_ENABLE_SHIFT;
}

/* Sets @gain to the currently set output EQ Master gain value */
void axd_cmd_mixer_get_eqmastergain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain)
{
	unsigned int control;

	axd_read_reg(cmd, AXD_REG_EQ_CTRL_GAIN, &control);
	*gain = (control & AXD_EQCTRL_GAIN_BITS) >> AXD_EQCTRL_GAIN_SHIFT;
}

/* Sets @gain to the currently set output EQ Band0 gain value */
void axd_cmd_mixer_get_eqband0gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain)
{
	unsigned int temp;

	axd_read_reg(cmd, AXD_REG_EQ_BAND0, &temp);
	*gain = ((int)temp & AXD_EQBANDX_GAIN_BITS) >> AXD_EQBANDX_GAIN_SHIFT;
}

/* Sets @gain to the currently set output EQ Band1 gain value */
void axd_cmd_mixer_get_eqband1gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain)
{
	unsigned int  temp;

	axd_read_reg(cmd, AXD_REG_EQ_BAND1, &temp);
	*gain = ((int)temp & AXD_EQBANDX_GAIN_BITS) >> AXD_EQBANDX_GAIN_SHIFT;
}

/* Sets @gain to the currently set output EQ Band2 gain value */
void axd_cmd_mixer_get_eqband2gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain)
{
	unsigned int  temp;

	axd_read_reg(cmd, AXD_REG_EQ_BAND2, &temp);
	*gain = ((int)temp & AXD_EQBANDX_GAIN_BITS) >> AXD_EQBANDX_GAIN_SHIFT;
}

/* Sets @gain to the currently set output EQ Band3 gain value */
void axd_cmd_mixer_get_eqband3gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain)
{
	unsigned int  temp;

	axd_read_reg(cmd, AXD_REG_EQ_BAND3, &temp);
	*gain = ((int)temp & AXD_EQBANDX_GAIN_BITS) >> AXD_EQBANDX_GAIN_SHIFT;
}

/* Sets @gain to the currently set output EQ Band4 gain value */
void axd_cmd_mixer_get_eqband4gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain)
{
	unsigned int  temp;

	axd_read_reg(cmd, AXD_REG_EQ_BAND4, &temp);
	*gain = ((int)temp & AXD_EQBANDX_GAIN_BITS) >> AXD_EQBANDX_GAIN_SHIFT;
}

/* Sets @mux to the currently selected mux output @pipe of mixer */
void axd_cmd_mixer_get_mux(struct axd_cmd *cmd, unsigned int pipe,
								char *mux)
{
	unsigned int reg = axd_get_mixer_mux_reg(cmd, pipe);
	unsigned int setting;

	if (unlikely(!reg || !mux))
		return;
	axd_read_reg(cmd, reg, &setting);
	parse_mux(setting, mux);
}

/* Returns non-zero of input @pipe is enabled. Zero otherwise. */
int axd_cmd_input_get_enabled(struct axd_cmd *cmd, unsigned int pipe)
{
	unsigned int reg = axd_get_input_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return (control & AXD_INCTRL_ENABLE_BITS) >> AXD_INCTRL_ENABLE_SHIFT;
}

/* Sets @source to the currently selected source of input @pipe */
void axd_cmd_input_get_source(struct axd_cmd *cmd, unsigned int pipe,
								char *source)
{
	unsigned int reg = axd_get_input_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg || !source))
		return;
	axd_read_reg(cmd, reg, &control);
	control = (control & AXD_INCTRL_SOURCE_BITS) >> AXD_INCTRL_SOURCE_SHIFT;
	parse_source(control, source);
}

/* Sets @codec to the currently selected codec of input @pipe */
void axd_cmd_input_get_codec(struct axd_cmd *cmd, unsigned int pipe,
								char *codec)
{
	unsigned int codec_num = axd_get_input_codec_number(cmd, pipe);

	if (unlikely(!codec))
		return;
	parse_codec_by_number(codec_num, codec);
}

/* Sets @gain to the currently set input gain value */
void axd_cmd_input_get_gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain)
{
	unsigned int reg = axd_get_input_gain_reg(cmd, pipe);

	if (unlikely(!reg || !gain))
		return;
	axd_read_reg(cmd, reg, gain);
}

/* Sets @gain to the currently set input gain value */
void axd_cmd_input_get_mute(struct axd_cmd *cmd, unsigned int pipe,
								int *muted)
{
	unsigned int reg = axd_get_input_gain_reg(cmd, pipe);

	if (unlikely(!reg || !muted))
		return;
	axd_read_reg(cmd, reg, muted);
}

/* Sets @upmix to the currently selected upmix setting of input @pipe */
void axd_cmd_input_get_upmix(struct axd_cmd *cmd, unsigned int pipe,
								char *upmix)
{
	unsigned int reg = axd_get_input_upmix_reg(cmd, pipe);
	unsigned int setting;

	if (unlikely(!reg || !upmix))
		return;
	axd_read_reg(cmd, reg, &setting);
	parse_upmix(setting, upmix);
}

/* Returns the buffer occupancy value of @pipe. */
unsigned int axd_cmd_input_get_buffer_occupancy(struct axd_cmd *cmd,
							unsigned int pipe)
{
	unsigned int bo;
	unsigned int reg = axd_get_input_buffer_occupancy_reg(cmd, pipe);

	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &bo);
	return bo;
}

/* Returns non-zero of output @pipe is enabled. Zero otherwise. */
int axd_cmd_output_get_enabled(struct axd_cmd *cmd, unsigned int pipe)
{
	unsigned int reg = axd_get_output_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return (control & AXD_OUTCTRL_ENABLE_BITS) >> AXD_OUTCTRL_ENABLE_SHIFT;
}

/* Sets @codec to the currently selected codec of output @pipe */
void axd_cmd_output_get_codec(struct axd_cmd *cmd, unsigned int pipe,
								char *codec)
{
	unsigned int codec_num = axd_get_output_codec_number(cmd, pipe);

	if (unlikely(!codec))
		return;
	parse_codec_by_number(codec_num, codec);
}

/* Sets @sink to the currently selected sink of output @pipe */
void axd_cmd_output_get_sink(struct axd_cmd *cmd, unsigned int pipe,
								char *sink)
{
	unsigned int reg = axd_get_output_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg || !sink))
		return;
	axd_read_reg(cmd, reg, &control);
	control = (control & AXD_OUTCTRL_SINK_BITS) >> AXD_OUTCTRL_SINK_SHIFT;
	parse_sink(control, sink);
}

/* Sets @downmix to the currently selected downmix setting of output @pipe */
void axd_cmd_output_get_downmix(struct axd_cmd *cmd, unsigned int pipe,
								char *downmix)
{
	unsigned int reg = axd_get_output_downmix_reg(cmd, pipe);
	unsigned int setting;

	if (unlikely(!reg || !downmix))
		return;
	axd_read_reg(cmd, reg, &setting);
	parse_downmix(setting, downmix);
}

/* Returns non-zero of output @pipe EQ is enabled. Zero otherwise. */
int axd_cmd_output_get_eqenabled(struct axd_cmd *cmd, unsigned int pipe)
{
	unsigned int reg = axd_get_output_eqcontrol_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return (control & AXD_EQCTRL_ENABLE_BITS) >> AXD_EQCTRL_ENABLE_SHIFT;
}

/* Sets @gain to the currently set output EQ Master gain value */
void axd_cmd_output_get_eqmastergain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain)
{
	unsigned int reg = axd_get_output_eqcontrol_reg(cmd, pipe);
	unsigned int temp;

	if (unlikely(!reg || !gain))
		return;
	axd_read_reg(cmd, reg, &temp);
	*gain = ((int)temp & AXD_EQCTRL_GAIN_BITS) >> AXD_EQCTRL_GAIN_SHIFT;
}

/* Sets @gain to the currently set output EQ Band0 gain value */
void axd_cmd_output_get_eqband0gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain)
{
	unsigned int reg = axd_get_output_eqband0_reg(cmd, pipe);
	unsigned int temp;

	if (unlikely(!reg || !gain))
		return;
	axd_read_reg(cmd, reg, &temp);
	*gain = ((int)temp & AXD_EQBANDX_GAIN_BITS) >> AXD_EQBANDX_GAIN_SHIFT;
}

/* Sets @gain to the currently set output EQ Band1 gain value */
void axd_cmd_output_get_eqband1gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain)
{
	unsigned int reg = axd_get_output_eqband1_reg(cmd, pipe);
	unsigned int temp;

	if (unlikely(!reg || !gain))
		return;
	axd_read_reg(cmd, reg, &temp);
	*gain = ((int)temp & AXD_EQBANDX_GAIN_BITS) >> AXD_EQBANDX_GAIN_SHIFT;
}

/* Sets @gain to the currently set output EQ Band2 gain value */
void axd_cmd_output_get_eqband2gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain)
{
	unsigned int reg = axd_get_output_eqband2_reg(cmd, pipe);
	unsigned int temp;

	if (unlikely(!reg || !gain))
		return;
	axd_read_reg(cmd, reg, &temp);
	*gain = ((int)temp & AXD_EQBANDX_GAIN_BITS) >> AXD_EQBANDX_GAIN_SHIFT;
}

/* Sets @gain to the currently set output EQ Band3 gain value */
void axd_cmd_output_get_eqband3gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain)
{
	unsigned int reg = axd_get_output_eqband3_reg(cmd, pipe);
	unsigned int temp;

	if (unlikely(!reg || !gain))
		return;
	axd_read_reg(cmd, reg, &temp);
	*gain = ((int)temp & AXD_EQBANDX_GAIN_BITS) >> AXD_EQBANDX_GAIN_SHIFT;
}

/* Sets @gain to the currently set output EQ Band4 gain value */
void axd_cmd_output_get_eqband4gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain)
{
	unsigned int reg = axd_get_output_eqband4_reg(cmd, pipe);
	unsigned int temp;

	if (unlikely(!reg || !gain))
		return;
	axd_read_reg(cmd, reg, &temp);
	*gain = ((int)temp & AXD_EQBANDX_GAIN_BITS) >> AXD_EQBANDX_GAIN_SHIFT;
}

void axd_cmd_output_get_geq_power(struct axd_cmd *cmd, unsigned int pipe,
							char *buf, int channel)
{
	u32 data[5];
	int i;

	if (channel < 4) {
		for (i = 0; i < 5; i++) {
			u32 reg = axd_get_output_eq_power_reg_ch0_3(cmd,
								pipe, i);

			if (unlikely(!reg))
				return;

			if (axd_read_reg(cmd, reg, &data[i]))
				return;
		}

		sprintf(buf, "%d, %d, %d, %d, %d\n",
				(data[0] >> (channel * 8)) & 0xFF,
				(data[1] >> (channel * 8)) & 0xFF,
				(data[2] >> (channel * 8)) & 0xFF,
				(data[3] >> (channel * 8)) & 0xFF,
				(data[3] >> (channel * 8)) & 0xFF);

	} else {
		for (i = 0; i < 5; i++) {
			u32 reg = axd_get_output_eq_power_reg_ch4_7(cmd,
								pipe, i);

			if (unlikely(!reg))
				return;

			if (axd_read_reg(cmd, reg, &data[i]))
				return;
		}

		sprintf(buf, "%d, %d, %d, %d, %d\n",
				(data[0] >> (channel-4 * 8)) & 0xFF,
				(data[1] >> (channel-4 * 8)) & 0xFF,
				(data[2] >> (channel-4 * 8)) & 0xFF,
				(data[3] >> (channel-4 * 8)) & 0xFF,
				(data[4] >> (channel-4 * 8)) & 0xFF);
	}
}

unsigned int axd_cmd_info_get_resampler_fin(struct axd_cmd *cmd,
							unsigned int pipe)
{
	unsigned int temp;
	unsigned int reg = axd_get_resample_fin_reg(cmd, pipe);

	axd_read_reg(cmd, reg, &temp);

	return temp;
}

unsigned int axd_cmd_info_get_resampler_fout(struct axd_cmd *cmd,
							unsigned int pipe)
{
	unsigned int temp;
	unsigned int reg = axd_get_resample_fout_reg(cmd, pipe);

	axd_read_reg(cmd, reg, &temp);

	return temp;
}

void axd_cmd_info_set_resampler_fout(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int fout)
{
	unsigned int reg = axd_get_resample_fout_reg(cmd, pipe);

	axd_write_reg(cmd, reg, fout);
}

unsigned int axd_cmd_output_get_dcpp_enabled(struct axd_cmd *cmd,
							unsigned int pipe)
{
	unsigned int reg = axd_get_output_dcpp_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return (control & AXD_DCPP_CTRL_ENABLE_BITS) >>
						AXD_DCPP_CTRL_ENABLE_SHIFT;
}

unsigned int axd_cmd_output_get_dcpp_mode(struct axd_cmd *cmd,
							unsigned int pipe)
{
	unsigned int reg = axd_get_output_dcpp_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return (control & AXD_DCPP_CTRL_MODE_BITS) >> AXD_DCPP_CTRL_MODE_SHIFT;
}

unsigned int axd_cmd_output_get_dcpp_channels(struct axd_cmd *cmd,
							unsigned int pipe)
{
	unsigned int reg = axd_get_output_dcpp_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return (control & AXD_DCPP_CTRL_CHANNELS_BITS) >>
						AXD_DCPP_CTRL_CHANNELS_SHIFT;
}

unsigned int axd_cmd_output_get_dcpp_eq_mode(struct axd_cmd *cmd,
							unsigned int pipe)
{
	unsigned int reg = axd_get_output_dcpp_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return (control & AXD_DCPP_CTRL_EQ_MODE_BITS) >>
						AXD_DCPP_CTRL_EQ_MODE_SHIFT;
}

unsigned int axd_cmd_output_get_dcpp_eq_bands(struct axd_cmd *cmd,
							unsigned int pipe)
{
	unsigned int reg = axd_get_output_dcpp_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return (control & AXD_DCPP_CTRL_EQ_BANDS_BITS) >>
						AXD_DCPP_CTRL_EQ_BANDS_SHIFT;
}

unsigned int axd_cmd_output_get_dcpp_max_delay_samples(struct axd_cmd *cmd,
							unsigned int pipe)
{
	unsigned int reg = axd_get_output_dcpp_max_delay_samples_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_delay_samples(struct axd_cmd *cmd,
				unsigned int pipe, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_delay_samples_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_eq_output_volume(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_eq_output_volume_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_eq_passthrough_gain(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_eq_passthrough_gain_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_eq_inverse_passthrough_gain(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_eq_inverse_passthrough_gain_reg(cmd,
									pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_bass_shelf_shift(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_bass_shelf_shift_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_bass_shelf_a0(struct axd_cmd *cmd,
				unsigned int pipe, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_bass_shelf_a0_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_bass_shelf_a1(struct axd_cmd *cmd,
				unsigned int pipe, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_bass_shelf_a1_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_bass_shelf_a2(struct axd_cmd *cmd,
				unsigned int pipe, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_bass_shelf_a2_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_bass_shelf_b0(struct axd_cmd *cmd,
				unsigned int pipe, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_bass_shelf_b0_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_bass_shelf_b1(struct axd_cmd *cmd,
				unsigned int pipe, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_bass_shelf_b1_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_treble_shelf_shift(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_treble_shelf_shift_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_treble_shelf_a0(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_treble_shelf_a0_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_treble_shelf_a1(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_treble_shelf_a1_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_treble_shelf_a2(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_treble_shelf_a2_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_treble_shelf_b0(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_treble_shelf_b0_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_treble_shelf_b1(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_treble_shelf_b1_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_eq_gain(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int band)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_gain_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_eq_a0(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int band)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_a0_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_eq_a1(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int band)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_a1_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_eq_a2(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int band)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_a2_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_eq_b0(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int band)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_b0_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_eq_b1(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int band)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_b1_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_channel_eq_shift(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int band)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_shift_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_subband_eq_bands(struct axd_cmd *cmd,
							unsigned int pipe)
{
	unsigned int reg = axd_get_output_dcpp_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return (control & AXD_DCPP_CTRL_SUBBAND_EQ_BANDS_BITS)
				>> AXD_DCPP_CTRL_SUBBAND_EQ_BANDS_SHIFT;
}

unsigned int axd_cmd_output_get_dcpp_subband_enabled(struct axd_cmd *cmd,
							unsigned int pipe)
{
	unsigned int reg = axd_get_output_dcpp_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return (control & AXD_DCPP_CTRL_SUBBAND_ENABLE_BITS)
				>> AXD_DCPP_CTRL_SUBBAND_ENABLE_SHIFT;
}

unsigned int axd_cmd_output_get_dcpp_subband_input_channel_mask(
					struct axd_cmd *cmd, unsigned int pipe)
{
	unsigned int reg = axd_get_output_dcpp_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return (control & AXD_DCPP_CTRL_SUBBAND_CHANNEL_MASK_BITS)
				>> AXD_DCPP_CTRL_SUBBAND_CHANNEL_MASK_SHIFT;
}

unsigned int axd_cmd_output_get_dcpp_subband_delay_samples(struct axd_cmd *cmd,
							unsigned int pipe)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);

	reg = axd_get_output_dcpp_channel_delay_samples_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_subband_eq_output_volume(
					struct axd_cmd *cmd, unsigned int pipe)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);

	reg = axd_get_output_dcpp_channel_eq_output_volume_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_subband_eq_passthrough_gain(
					struct axd_cmd *cmd, unsigned int pipe)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);

	reg = axd_get_output_dcpp_channel_eq_passthrough_gain_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_subband_eq_inverse_passthrough_gain(
					struct axd_cmd *cmd, unsigned int pipe)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);

	reg = axd_get_output_dcpp_channel_eq_inverse_passthrough_gain_reg(cmd,
									pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_subband_eq_gain(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int band)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_gain_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_subband_eq_a0(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int band)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_a0_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_subband_eq_a1(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int band)
{
	unsigned int control;
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_a1_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_subband_eq_a2(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int band)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_a2_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_subband_eq_b0(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int band)
{
	unsigned int control;
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_b0_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_subband_eq_b1(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int band)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_b1_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_subband_eq_shift(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int band)
{
	unsigned int reg;
	unsigned int control;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_shift_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_subband_low_pass_filter_a0(
					struct axd_cmd *cmd, unsigned int pipe)
{
	unsigned int reg;
	unsigned int control;

	reg = axd_get_output_dcpp_subband_low_pass_filter_a0_reg(cmd, pipe);

	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_subband_low_pass_filter_a1(
					struct axd_cmd *cmd, unsigned int pipe)
{
	unsigned int reg;
	unsigned int control;

	reg = axd_get_output_dcpp_subband_low_pass_filter_a1_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_subband_low_pass_filter_a2(
					struct axd_cmd *cmd, unsigned int pipe)
{
	unsigned int reg;
	unsigned int control;

	reg = axd_get_output_dcpp_subband_low_pass_filter_a2_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_subband_low_pass_filter_b0(
					struct axd_cmd *cmd, unsigned int pipe)
{
	unsigned int reg;
	unsigned int control;

	reg = axd_get_output_dcpp_subband_low_pass_filter_b0_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}

unsigned int axd_cmd_output_get_dcpp_subband_low_pass_filter_b1(
					struct axd_cmd *cmd, unsigned int pipe)
{
	unsigned int reg;
	unsigned int control;

	reg = axd_get_output_dcpp_subband_low_pass_filter_b1_reg(cmd, pipe);
	if (unlikely(!reg))
		return 0;
	axd_read_reg(cmd, reg, &control);
	return control;
}
