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
 * AXD Commands API - Configuration functions.
 */
#include "axd_cmds.h"
#include "axd_cmds_internal.h"


/*
 * Enable/Disable Mixer EQ.
 * @pipe: pipe number.
 * @enable:
 *	Enable	= !0
 *	Disable	= 0
 */
void axd_cmd_mixer_set_eqenabled(struct axd_cmd *cmd, unsigned int pipe,
								int enable)
{
	unsigned int reg = AXD_REG_EQ_CTRL_GAIN;
	unsigned int control;

	if (axd_read_reg(cmd, reg, &control))
		return;

	if (enable)
		control |= AXD_EQCTRL_ENABLE_BITS;
	else
		control &= ~AXD_EQCTRL_ENABLE_BITS;
	axd_write_reg(cmd, reg, control);
}

/*
 * Set the Master gain of the EQ of the Mixer
 * @pipe: pipe number.
 * @gain: 0-99 gain value
 */
void axd_cmd_mixer_set_eqmastergain(struct axd_cmd *cmd, unsigned int pipe,
								int gain)
{
	unsigned int reg = AXD_REG_EQ_CTRL_GAIN;
	unsigned int control;

	if (unlikely(gain > 99 || gain < 0))
		return;

	if (axd_read_reg(cmd, reg, &control))
		return;

	gain = (gain << AXD_EQCTRL_GAIN_SHIFT) & AXD_EQCTRL_GAIN_BITS;
	control &= ~AXD_EQCTRL_GAIN_BITS;
	control |= gain;
	axd_write_reg(cmd, reg, control);
}

/*
 * Set the gain of the EQ Band0 of the Mixer
 * @pipe: pipe number.
 * @gain: Signed 8 bit 2'compliment gain value.
 */
void axd_cmd_mixer_set_eqband0gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain)
{
	gain = (gain << AXD_EQBANDX_GAIN_SHIFT) & AXD_EQBANDX_GAIN_BITS;
	axd_write_reg(cmd, AXD_REG_EQ_BAND0, gain);
}

/*
 * Set the gain of the EQ Band1 of the Mixer
 * @pipe: pipe number.
 * @gain: Signed 8 bit 2'compliment gain value.
 */
void axd_cmd_mixer_set_eqband1gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain)
{
	gain = (gain << AXD_EQBANDX_GAIN_SHIFT) & AXD_EQBANDX_GAIN_BITS;
	axd_write_reg(cmd, AXD_REG_EQ_BAND1, gain);
}

/*
 * Set the gain of the EQ Band2 of Mixer
 * @pipe: pipe number.
 * @gain: Signed 8 bit 2'compliment gain value.
 */
void axd_cmd_mixer_set_eqband2gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain)
{
	gain = (gain << AXD_EQBANDX_GAIN_SHIFT) & AXD_EQBANDX_GAIN_BITS;
	axd_write_reg(cmd, AXD_REG_EQ_BAND2, gain);
}

/*
 * Set the gain of the EQ Band3 of the Mixer
 * @pipe: pipe number.
 * @gain: Signed 8 bit 2'compliment gain value.
 */
void axd_cmd_mixer_set_eqband3gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain)
{
	gain = (gain << AXD_EQBANDX_GAIN_SHIFT) & AXD_EQBANDX_GAIN_BITS;
	axd_write_reg(cmd, AXD_REG_EQ_BAND3, gain);
}

/*
 * Set the gain of the EQ Band4 of the Mixer
 * @pipe: pipe number.
 * @gain: Signed 8 bit 2'compliment gain value.
 */
void axd_cmd_mixer_set_eqband4gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain)
{
	gain = (gain << AXD_EQBANDX_GAIN_SHIFT) & AXD_EQBANDX_GAIN_BITS;
	axd_write_reg(cmd, AXD_REG_EQ_BAND4, gain);
}

/*
 * Select Mixer's Mux output
 * @pipe: pipe number.
 * @mux:
 *	Mix	= 0
 *	Input 0	= 1
 *	Input 1	= 2
 *	Input 2	= 3
 *	Input 3	= 4
 */
void axd_cmd_mixer_set_mux(struct axd_cmd *cmd, unsigned int pipe,
								int mux)
{
	unsigned int reg = axd_get_mixer_mux_reg(cmd, pipe);

	if (unlikely(mux > 4 || mux < 0))
		return;
	axd_write_reg(cmd, reg, mux);
}

/*
 * Enable/Disable input.
 * @pipe: pipe number.
 * @enable:
 *	Enable	= !0
 *	Disable	= 0
 */
int axd_cmd_input_set_enabled(struct axd_cmd *cmd, unsigned int pipe,
								int enable)
{
	unsigned int reg = axd_get_input_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return -1;

	if (axd_read_reg(cmd, reg, &control))
		return -1;

	if (enable)
		control |= AXD_INCTRL_ENABLE_BITS;
	else
		control &= ~AXD_INCTRL_ENABLE_BITS;
	if (axd_write_reg(cmd, reg, control))
		return -1;
	return 0;
}

/*
 * Set the source of the input pipe.
 * @pipe: pipe number.
 * @source:
 *	Pipe	= 0
 *	Aux	= 1
 */
void axd_cmd_input_set_source(struct axd_cmd *cmd, unsigned int pipe,
								int source)
{
	unsigned int reg = axd_get_input_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg || source > 1 || source < 0))
		return;
	if (axd_read_reg(cmd, reg, &control))
		return;
	source = (source << AXD_INCTRL_SOURCE_SHIFT) & AXD_INCTRL_SOURCE_BITS;
	control &= ~AXD_INCTRL_SOURCE_BITS;
	control |= source;
	axd_write_reg(cmd, reg, control);
}

/*
 * Set the codec of the input pipe.
 * @pipe: pipe number.
 * @codec:
 *	PCM Pass Through	= 0
 *	MPEG (2/3)		= 1
 *	Dolby AC3		= 2
 *	AAC			= 3
 *	Ogg Vorbis		= 4
 *	FLAC			= 5
 *	Cook			= 6
 *	WMA			= 7
 *	DDPlus			= 8
 *	DTS			= 9   Unsupported
 *	DTS-HD			= 10  Unsupported
 *	ALAC			= 11
 *	SBC			= 13
 */
void axd_cmd_input_set_codec(struct axd_cmd *cmd, unsigned int pipe,
								int codec)
{
	unsigned int reg = axd_get_input_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg || codec > 13 || codec < 0 ||
						codec == 9 || codec == 10))
		return;
	if (axd_read_reg(cmd, reg, &control))
		return;
	codec = (codec << AXD_INCTRL_CODEC_SHIFT) & AXD_INCTRL_CODEC_BITS;
	control &= ~AXD_INCTRL_CODEC_BITS;
	control |= codec;
	axd_write_reg(cmd, reg, control);
}

/*
 * Set the gain of the input pipe.
 * @pipe: pipe number.
 * @gain: Signed 32 bit 2'compliment gain value.
 *	Gain Cut or Boost in 0.25dB increment. ie: 4 = 1dB.
 */
void axd_cmd_input_set_gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain)
{
	unsigned int reg = axd_get_input_gain_reg(cmd, pipe);

	if (unlikely(!reg))
		return;
	axd_write_reg(cmd, reg, gain);
}

/*
 * Mute/Unmute the input pipe.
 * @pipe: pipe number.
 * @mute:  0	= OFF
 *	  !0	= ON
 */
void axd_cmd_input_set_mute(struct axd_cmd *cmd, unsigned int pipe,
								int mute)
{
	unsigned int reg = axd_get_input_mute_reg(cmd, pipe);

	if (unlikely(!reg))
		return;
	axd_write_reg(cmd, reg, mute);
}

/*
 * Set the upmix of the input pipe.
 * @pipe: pipe number.
 * @upmix:
 *	Pass through		= 0
 *	Simple 5.1		= 1
 *	Dolby Pro Logic 2	= 2
 */
void axd_cmd_input_set_upmix(struct axd_cmd *cmd, unsigned int pipe,
								int upmix)
{
	unsigned int reg = axd_get_input_upmix_reg(cmd, pipe);

	if (unlikely(!reg || upmix > 2 || upmix < 0))
		return;
	axd_write_reg(cmd, reg, upmix);
}

/* Set the buffer occupancy value of @pipe. */
void axd_cmd_input_set_buffer_occupancy(struct axd_cmd *cmd, unsigned int pipe,
								unsigned int bo)
{
	unsigned int reg = axd_get_input_buffer_occupancy_reg(cmd, pipe);

	axd_write_reg(cmd, reg, bo);
}

/*
 * Enable/Disable output.
 * @pipe: pipe number.
 * @enable:
 *	Enable	= !0
 *	Disable	= 0
 */
int axd_cmd_output_set_enabled(struct axd_cmd *cmd, unsigned int pipe,
								int enable)
{
	unsigned int reg = axd_get_output_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return -1;
	if (axd_read_reg(cmd, reg, &control))
		return -1;
	if (enable)
		control |= AXD_OUTCTRL_ENABLE_BITS;
	else
		control &= ~AXD_OUTCTRL_ENABLE_BITS;
	if (axd_write_reg(cmd, reg, control))
		return -1;
	return 0;
}

/*
 * Set the codec of the output pipe.
 * @pipe: pipe number.
 * @codec:
 *	PCM Pass Through	=  0
 *	MPEG (2/3)		=  1 Unsupported
 *	Dolby AC3		=  2 Unsupported
 *	AAC			=  3 Unsupported
 *	Ogg Vorbis		=  4 Unsupported
 *	FLAC			=  5
 *	Cook			=  6 Unsupported
 *	WMA			=  7 Unsupported
 *	DDPlus			=  8 Unsupported
 *	DTS			=  9 Unsupported
 *	DTS-HD			= 10 Unsupported
 *	ALAC			= 11
 *	SBC			= 13 Unsupported
 */
void axd_cmd_output_set_codec(struct axd_cmd *cmd, unsigned int pipe,
								int codec)
{
	unsigned int reg = axd_get_output_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg || !(codec == 0 || codec == 5 || codec == 11)))
		return;
	if (axd_read_reg(cmd, reg, &control))
		return;
	codec = (codec << AXD_OUTCTRL_CODEC_SHIFT) & AXD_OUTCTRL_CODEC_BITS;
	control &= ~AXD_OUTCTRL_CODEC_BITS;
	control |= codec;
	axd_write_reg(cmd, reg, control);
}

/*
 * Set the sink of the output pipe.
 * @pipe: pipe number.
 * @source:
 *	Pipe	= 0
 */
void axd_cmd_output_set_sink(struct axd_cmd *cmd, unsigned int pipe,
								int sink)
{
	unsigned int reg = axd_get_output_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg || (sink < 0 && sink > 3)))
		return;
	if (axd_read_reg(cmd, reg, &control))
		return;
	sink = (sink << AXD_OUTCTRL_SINK_SHIFT) & AXD_OUTCTRL_SINK_BITS;
	control &= ~AXD_OUTCTRL_SINK_BITS;
	control |= sink;
	axd_write_reg(cmd, reg, control);
}

/*
 * Set the downmix of the output pipe.
 * @pipe: pipe number.
 * @downmix:
 *	Pass through	= 0
 *	5.1		= 1
 *	2.0		= 2
 */
void axd_cmd_output_set_downmix(struct axd_cmd *cmd, unsigned int pipe,
								int downmix)
{
	unsigned int reg = axd_get_output_downmix_reg(cmd, pipe);

	if (unlikely(!reg || downmix > 2 || downmix < 0))
		return;
	axd_write_reg(cmd, reg, downmix);
}

/*
 * Enable/Disable output EQ.
 * @pipe: pipe number.
 * @enable:
 *	Enable	= !0
 *	Disable	= 0
 */
void axd_cmd_output_set_eqenabled(struct axd_cmd *cmd, unsigned int pipe,
								int enable)
{
	unsigned int reg = axd_get_output_eqcontrol_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return;
	if (axd_read_reg(cmd, reg, &control))
		return;

	if (enable)
		control |= AXD_EQCTRL_ENABLE_BITS;
	else
		control &= ~AXD_EQCTRL_ENABLE_BITS;
	axd_write_reg(cmd, reg, control);
}

/*
 * Set the Master gain of the EQ of output pipe.
 * @pipe: pipe number.
 * @gain: 0-99 gain value
 */
void axd_cmd_output_set_eqmastergain(struct axd_cmd *cmd, unsigned int pipe,
								int gain)
{
	unsigned int reg = axd_get_output_eqcontrol_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg || gain > 99 || gain < 0))
		return;
	if (axd_read_reg(cmd, reg, &control))
		return;

	gain = (gain << AXD_EQCTRL_GAIN_SHIFT) & AXD_EQCTRL_GAIN_BITS;
	control &= ~AXD_EQCTRL_GAIN_BITS;
	control |= gain;
	axd_write_reg(cmd, reg, control);
}

/*
 * Set the gain of the EQ Band0 of output pipe.
 * @pipe: pipe number.
 * @gain: Signed 8 bit 2'compliment gain value.
 */
void axd_cmd_output_set_eqband0gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain)
{
	unsigned int reg = axd_get_output_eqband0_reg(cmd, pipe);

	if (unlikely(!reg))
		return;
	gain = (gain << AXD_EQBANDX_GAIN_SHIFT) & AXD_EQBANDX_GAIN_BITS;
	axd_write_reg(cmd, reg, gain);
}

/*
 * Set the gain of the EQ Band1 of output pipe.
 * @pipe: pipe number.
 * @gain: Signed 8 bit 2'compliment gain value.
 */
void axd_cmd_output_set_eqband1gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain)
{
	unsigned int reg = axd_get_output_eqband1_reg(cmd, pipe);

	if (unlikely(!reg))
		return;
	gain = (gain << AXD_EQBANDX_GAIN_SHIFT) & AXD_EQBANDX_GAIN_BITS;
	axd_write_reg(cmd, reg, gain);
}

/*
 * Set the gain of the EQ Band2 of output pipe.
 * @pipe: pipe number.
 * @gain: Signed 8 bit 2'compliment gain value.
 */
void axd_cmd_output_set_eqband2gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain)
{
	unsigned int reg = axd_get_output_eqband2_reg(cmd, pipe);

	if (unlikely(!reg))
		return;
	gain = (gain << AXD_EQBANDX_GAIN_SHIFT) & AXD_EQBANDX_GAIN_BITS;
	axd_write_reg(cmd, reg, gain);
}

/*
 * Set the gain of the EQ Band3 of output pipe.
 * @pipe: pipe number.
 * @gain: Signed 8 bit 2'compliment gain value.
 */
void axd_cmd_output_set_eqband3gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain)
{
	unsigned int reg = axd_get_output_eqband3_reg(cmd, pipe);

	if (unlikely(!reg))
		return;
	gain = (gain << AXD_EQBANDX_GAIN_SHIFT) & AXD_EQBANDX_GAIN_BITS;
	axd_write_reg(cmd, reg, gain);
}

/*
 * Set the gain of the EQ Band4 of output pipe.
 * @pipe: pipe number.
 * @gain: Signed 8 bit 2'compliment gain value.
 */
void axd_cmd_output_set_eqband4gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain)
{
	unsigned int reg = axd_get_output_eqband4_reg(cmd, pipe);

	if (unlikely(!reg))
		return;
	gain = (gain << AXD_EQBANDX_GAIN_SHIFT) & AXD_EQBANDX_GAIN_BITS;
	axd_write_reg(cmd, reg, gain);
}

/* DCPP */

int axd_cmd_output_set_dcpp_enabled(struct axd_cmd *cmd, unsigned int pipe,
								int enable)
{
	unsigned int reg;
	unsigned int control;

	reg = axd_get_output_dcpp_control_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;
	if (axd_read_reg(cmd, reg, &control))
		return -1;

	if (enable)
		control |= AXD_DCPP_CTRL_ENABLE_BITS;
	else
		control &= ~AXD_DCPP_CTRL_ENABLE_BITS;

	return axd_write_reg_buf(cmd, reg, control);
}

int axd_cmd_output_set_dcpp_mode(struct axd_cmd *cmd, unsigned int pipe,
							unsigned int mode)
{
	unsigned int reg;
	unsigned int control;

	reg = axd_get_output_dcpp_control_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;
	if (axd_read_reg(cmd, reg, &control))
		return -1;

	/* Conditionally mask in mode bit */
	control ^= ((control ^ (mode << AXD_DCPP_CTRL_MODE_SHIFT))
						& AXD_DCPP_CTRL_MODE_BITS);

	return axd_write_reg_buf(cmd, reg, control);
}

int axd_cmd_output_set_dcpp_eq_mode(struct axd_cmd *cmd, unsigned int pipe,
							unsigned int mode)
{
	unsigned int reg;
	unsigned int control;

	reg = axd_get_output_dcpp_control_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;
	if (axd_read_reg(cmd, reg, &control))
		return -1;

	/* Conditionally mask in mode bit */
	control ^= ((control ^ (mode << AXD_DCPP_CTRL_EQ_MODE_SHIFT))
						& AXD_DCPP_CTRL_EQ_MODE_BITS);

	return axd_write_reg_buf(cmd, reg, control);
}

int axd_cmd_output_set_dcpp_channel_delay_samples(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_delay_samples_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_eq_output_volume(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_eq_output_volume_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_eq_passthrough_gain(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_eq_passthrough_gain_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_eq_inverse_passthrough_gain(
					struct axd_cmd *cmd, unsigned int pipe,
					unsigned int channel, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_eq_inverse_passthrough_gain_reg(cmd,
									pipe);
	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_bass_shelf_shift(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_bass_shelf_shift_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_bass_shelf_a0(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_bass_shelf_a0_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_bass_shelf_a1(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_bass_shelf_a1_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_bass_shelf_a2(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_bass_shelf_a2_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_bass_shelf_b0(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_bass_shelf_b0_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_bass_shelf_b1(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_bass_shelf_b1_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_treble_shelf_shift(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_treble_shelf_shift_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_treble_shelf_a0(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_treble_shelf_a0_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_treble_shelf_a1(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_treble_shelf_a1_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_treble_shelf_a2(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_treble_shelf_a2_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_treble_shelf_b0(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_treble_shelf_b0_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_treble_shelf_b1(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);

	reg = axd_get_output_dcpp_channel_treble_shelf_b1_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_eq_gain(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int channel,
					unsigned int band, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_gain_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_eq_a0(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int channel,
					unsigned int band, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_a0_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_eq_a1(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int channel,
					unsigned int band, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_a1_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_eq_a2(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int channel,
					unsigned int band, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_a2_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_eq_b0(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int channel,
					unsigned int band, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_b0_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_eq_b1(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int channel,
					unsigned int band, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_b1_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_channel_eq_shift(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int channel,
					unsigned int band, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, false, channel);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_shift_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_enabled(struct axd_cmd *cmd,
						unsigned int pipe, int enable)
{
	unsigned int reg;
	unsigned int control;

	reg = axd_get_output_dcpp_control_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;
	if (axd_read_reg(cmd, reg, &control))
		return -1;

	if (enable)
		control |= AXD_DCPP_CTRL_SUBBAND_ENABLE_BITS;
	else
		control &= ~AXD_DCPP_CTRL_SUBBAND_ENABLE_BITS;

	return axd_write_reg_buf(cmd, reg, enable);
}

int axd_cmd_output_set_dcpp_subband_delay_samples(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);

	reg = axd_get_output_dcpp_channel_delay_samples_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_input_channel_mask(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data)
{
	unsigned int reg;
	unsigned int control;

	reg = axd_get_output_dcpp_control_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;
	if (axd_read_reg(cmd, reg, &control))
		return -1;

	control &= ~AXD_DCPP_CTRL_SUBBAND_CHANNEL_MASK_BITS;
	control |= data << AXD_DCPP_CTRL_SUBBAND_CHANNEL_MASK_SHIFT;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_eq_output_volume(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);

	reg = axd_get_output_dcpp_channel_eq_output_volume_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_eq_passthrough_gain(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);

	reg = axd_get_output_dcpp_channel_eq_passthrough_gain_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_eq_inverse_passthrough_gain(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);

	reg = axd_get_output_dcpp_channel_eq_inverse_passthrough_gain_reg(cmd,
									pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_low_pass_filter_a0(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data)
{
	unsigned int reg;

	reg = axd_get_output_dcpp_subband_low_pass_filter_a0_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_low_pass_filter_a1(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data)
{
	unsigned int reg;

	reg = axd_get_output_dcpp_subband_low_pass_filter_a1_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_low_pass_filter_a2(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data)
{
	unsigned int reg;

	reg = axd_get_output_dcpp_subband_low_pass_filter_a2_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_low_pass_filter_b0(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data)
{
	unsigned int reg;

	reg = axd_get_output_dcpp_subband_low_pass_filter_b0_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_low_pass_filter_b1(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data)
{
	unsigned int reg;

	reg = axd_get_output_dcpp_subband_low_pass_filter_b1_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_eq_gain(struct axd_cmd *cmd,
			unsigned int pipe, unsigned int band, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_gain_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_eq_a0(struct axd_cmd *cmd,
			unsigned int pipe, unsigned int band, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_a0_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_eq_a1(struct axd_cmd *cmd,
			unsigned int pipe, unsigned int band, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_a1_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_eq_a2(struct axd_cmd *cmd,
			unsigned int pipe, unsigned int band, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_a2_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_eq_b0(struct axd_cmd *cmd,
			unsigned int pipe, unsigned int band, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_b0_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_eq_b1(struct axd_cmd *cmd,
			unsigned int pipe, unsigned int band, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_b1_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}

int axd_cmd_output_set_dcpp_subband_eq_shift(struct axd_cmd *cmd,
			unsigned int pipe, unsigned int band, unsigned int data)
{
	unsigned int reg;

	axd_cmd_output_dcpp_select_channel(cmd, pipe, true, 0);
	axd_cmd_output_dcpp_select_band(cmd, pipe, band);

	reg = axd_get_output_dcpp_channel_eq_shift_reg(cmd, pipe);

	if (unlikely(!reg))
		return -1;

	return axd_write_reg_buf(cmd, reg, data);
}
