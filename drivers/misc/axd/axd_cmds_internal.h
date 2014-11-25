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
 * Common functionality required by other axd_cmds_*.c files.
 */
#ifndef AXD_CMDS_INTERNAL_H_
#define AXD_CMDS_INTERNAL_H_

#include <linux/io.h>

#include "axd_cmds.h"

void axd_ctrl_kick(struct axd_memory_map __iomem *message);
void axd_kick_status_clear(struct axd_memory_map __iomem *message);
int axd_wait_ready(struct axd_memory_map __iomem *message);

int axd_write_ctrl(struct axd_cmd *cmd, unsigned int ctrl_command,
						unsigned int ctrl_data);

int axd_read_reg(struct axd_cmd *cmd, unsigned int reg, unsigned int *data);
int axd_write_reg(struct axd_cmd *cmd, unsigned int reg, unsigned int value);

int axd_write_reg_buf(struct axd_cmd *cmd, unsigned int reg,
							unsigned int value);

unsigned int axd_get_mixer_mux_reg(struct axd_cmd *cmd, unsigned int pipe);

unsigned int axd_get_input_codec_number(struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_input_control_reg(struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_input_gain_reg(struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_input_mute_reg(struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_input_upmix_reg(struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_input_buffer_occupancy_reg(struct axd_cmd *cmd,
							unsigned int pipe);

unsigned int axd_get_output_codec_number(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_control_reg(struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_downmix_reg(struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_eqcontrol_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_eqband0_reg(struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_eqband1_reg(struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_eqband2_reg(struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_eqband3_reg(struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_eqband4_reg(struct axd_cmd *cmd, unsigned int pipe);

unsigned int axd_get_output_eq_power_reg_ch0_3(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int band);
unsigned int axd_get_output_eq_power_reg_ch4_7(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int band);

unsigned int axd_get_output_dcpp_control_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_max_delay_samples_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_ctrl_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_band_ctrl_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_delay_samples_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_eq_output_volume_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_eq_passthrough_gain_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_eq_inverse_passthrough_gain_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_bass_shelf_shift_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_bass_shelf_a0_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_bass_shelf_a1_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_bass_shelf_a2_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_bass_shelf_b0_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_bass_shelf_b1_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_treble_shelf_shift_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_treble_shelf_a0_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_treble_shelf_a1_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_treble_shelf_a2_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_treble_shelf_b0_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_treble_shelf_b1_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_eq_gain_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_eq_a0_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_eq_a1_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_eq_a2_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_eq_b0_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_eq_b1_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_channel_eq_shift_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_subband_input_select_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_output_dcpp_subband_low_pass_filter_a0_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_dcpp_subband_low_pass_filter_a1_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_dcpp_subband_low_pass_filter_a2_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_dcpp_subband_low_pass_filter_b0_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_output_dcpp_subband_low_pass_filter_b1_reg(
					struct axd_cmd *cmd, unsigned int pipe);

unsigned int axd_get_decoder_aac_version_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_aac_channels_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_aac_profile_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_aac_streamtype_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_aac_samplerate_reg(struct axd_cmd *cmd,
							unsigned int pipe);

unsigned int axd_get_decoder_ac3_channels_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_ac3_channel_order_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_ac3_mode_reg(struct axd_cmd *cmd,
							unsigned int pipe);

unsigned int axd_get_decoder_cook_flavour_reg(struct axd_cmd *cmd,
							unsigned int pipe);

unsigned int axd_get_decoder_flac_channels_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_flac_samplerate_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_flac_bitspersample_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_flac_md5checking_reg(struct axd_cmd *cmd,
							unsigned int pipe);

unsigned int axd_get_decoder_mpeg_numchannels_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_mpeg_mlchannel_reg(struct axd_cmd *cmd,
							unsigned int pipe);

unsigned int axd_get_decoder_wma_playeropt_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_wma_drcsetting_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_wma_peakampref_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_wma_rmsampref_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_wma_peakamptarget_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_wma_rmsamptarget_reg(struct axd_cmd *cmd,
							unsigned int pipe);

unsigned int axd_get_decoder_wma_pcmvalidbitspersample_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_wma_pcmcontainersize_reg(struct axd_cmd *cmd,
							unsigned int pipe);

unsigned int axd_get_decoder_wma_wmaformattag_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_wma_wmanumchannels_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_wma_wmasamplespersec_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_wma_wmaaveragebytespersec_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_wma_wmablockalign_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_wma_wmavalidbitspersample_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_wma_wmachannelmask_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_wma_wmaencodeoptions_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_pcm_samplerate_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_pcm_channels_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_pcm_bitspersample_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_pcm_justification_reg(struct axd_cmd *cmd,
							unsigned int pipe);

unsigned int axd_get_decoder_ddplus_config_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_ddplus_channel_order_reg(struct axd_cmd *cmd,
							unsigned int pipe);

unsigned int axd_get_decoder_alac_channels_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_alac_depth_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_alac_samplerate_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_alac_framelength_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_alac_maxframebytes_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_alac_avgbitrate_reg(struct axd_cmd *cmd,
							unsigned int pipe);

unsigned int axd_get_decoder_sbc_samplerate_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_sbc_audiomode_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_sbc_blocks_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_sbc_subbands_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_sbc_bitpool_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_sbc_allocationmode_reg(struct axd_cmd *cmd,
							unsigned int pipe);

unsigned int axd_get_decoder_ms11_mode_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_ms11_common_config0_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_ms11_common_config1_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_ms11_ddt_config0_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_ms11_ddc_config0_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_decoder_ms11_ext_pcm_config0_reg(struct axd_cmd *cmd,
							unsigned int pipe);

unsigned int axd_get_encoder_pcm_bitspersample_reg(struct axd_cmd *cmd,
							unsigned int pipe);

unsigned int axd_get_encoder_flac_channels_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_encoder_flac_bitspersample_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_encoder_flac_samplerate_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_encoder_flac_totalsamples_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_encoder_flac_domidsidestereo_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_encoder_flac_loosemidsidestereo_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_encoder_flac_doexhaustivemodelsearch_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_encoder_flac_minresidualpartitionorder_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_encoder_flac_maxresidualpartitionorder_reg(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_encoder_flac_blocksize_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_encoder_flac_bytecount_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_encoder_flac_samplecount_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_encoder_flac_framecount_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_encoder_flac_framebytes_reg(struct axd_cmd *cmd,
							unsigned int pipe);

unsigned int axd_get_encoder_alac_channels_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_encoder_alac_depth_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_encoder_alac_samplerate_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_encoder_alac_framelength_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_encoder_alac_maxframebytes_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_encoder_alac_avgbitrate_reg(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_get_encoder_alac_fastmode_reg(struct axd_cmd *cmd,
							unsigned int pipe);

unsigned int axd_get_resample_fin_reg(struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_get_resample_fout_reg(struct axd_cmd *cmd, unsigned int pipe);

void axd_cmd_inpipe_init(struct axd_cmd *cmd, unsigned int pipe);
void axd_cmd_outpipe_init(struct axd_cmd *cmd, unsigned int pipe);

#endif /* AXD_CMDS_INTERNAL_H_ */
