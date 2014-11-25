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
 * AXD API commands Helper functions.
 */
#ifndef AXD_CMDS_H_
#define AXD_CMDS_H_

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include "linux/workqueue.h"

#include "axd_api.h"
#include "axd_buffers.h"

/**
 * struct axd_desc_ctrl - axd desctriptors control structure
 * @rd_idx:	read index of next available descriptor
 * @wr_idx:	write index of empty slot ot return a descriptor to
 * @rd_sem:	semaphore to block when no more descriptors are available
 * @wr_sem:	semaphore to block when all descriptors are available
 * @rd_lock:	smp critical section protection for reads
 * @wr_lock:	smp critical section protection for writes
 * @buf_desc:	pointer to iomem where the descriptors are
 * @num_desc:	total number of descriptors provided by axd
 *
 * axd has a number of input and output descriptors to pass buffers around, this
 * structure provides a mean for the driver to manage access to these
 * descriptors.
 */
struct axd_desc_ctrl {
	unsigned int rd_idx;
	unsigned int wr_idx;
	struct semaphore rd_sem;
	struct semaphore wr_sem;
	spinlock_t rd_lock;
	spinlock_t wr_lock;
	struct axd_buffer_desc __iomem *buf_desc;
	unsigned int num_desc;
};

struct axd_cmd;

/**
 * struct axd_pipe - axd pipe management structure
 * @work:		work for top half of the interrupt
 * @desc_ctrl:		axd_desc_ctrl structure to manage this pipe's
 *			descriptors
 * @desc_bufferq:	buffer queue send through the descriptors
 * @user_bufferq:	buffer queue of buffers to be read by the user. only
 *			makes sense for an output pipe where the user doesn't
 *			have to read the returned buffers synchronously when we
 *			get an interrupt
 * @cur_buf:		pointer to the current user_bufferq being read
 * @cur_buf_offset:	offset of the current user_bufferq to start reading from
 * @cur_buf_size:	remaining size of data in current user_bufferq
 * @discard_flg:	a flag to indicate we should discard the remaining data
 *			if the user closed output node before reading all data,
 *			default to true.
 * @enabled_flg:	a flag indicates that this pipe is actively handling a
 *			stream
 * @eos_flg:		a flag indicates that eos was reached and we should do
 *			clean up as soon as possible
 * @eos_mutex:		for input pipes we need to protect against possible
 *			simulataneous sending of eos
 * @intcount:		number of interrupts received since last service.
 *			indicates the number of buffers services by axd.
 *			used by top half workqueue to know how many interrupts
 *			it needs to service in one go
 * @id:			pipe number or id
 * @tsk:		the userland task that opened this pipe
 * @buf_size:		the size of the buffer this pipe is configured to use
 * @current_ts_low:	lower half of the 64-bit timestamp for current buffer
 * @current_ts_high:	top half of the 64-bit timestamp for current buffer
 * @cmd:		pointer to axd_cmd struct for quick access
 * @buf_desc:		pointer to axd_buffer_desc struct for quick access
 *
 * axd could provide a number of pipes each of which handles a separate stream.
 * this structure manages descriptors, buffers and other control bits associated
 * with each input/output pipe.
 */
struct axd_pipe {
	struct work_struct work;
	struct axd_desc_ctrl desc_ctrl;
	struct axd_bufferq desc_bufferq;
	struct axd_bufferq user_bufferq;
	char *cur_buf;
	unsigned int cur_buf_offset;
	unsigned int cur_buf_size;
	unsigned int  discard_flg;
	unsigned int  enabled_flg;
	unsigned int  eos_flg;
	struct mutex eos_mutex;
	atomic_t intcount;
	unsigned int id;
	struct task_struct *tsk;
	unsigned int buf_size;
	u32 current_ts_low;
	u32 current_ts_high;
	struct axd_cmd *cmd;
	struct axd_buffer_desc __iomem *buf_desc;
};

/**
 * struct axd_cmd - axd command structure
 * @message:		iomapped axd massage region, see axd_memory_map struct
 * @cm_lock:		mutex to control access to the message region
 * @wait:		wait for ControlCommand response, or command completion
 * @response_flg:	condition variable to wait on for response
 * @in_workq:		array of workqueues for input pipes
 * @out_workq:		array of workqueues for output pipes
 * @in_pipes:		array of input axd_pipe structs
 * @out_pipes:		array of output axd_pipe structs
 * @watchdogenabled:	software watchdog switch
 * @discard_flg:	master flag to control whether to discard data when user
 *			closes output node
 * @nonblock:		operate in nonblocking mode
 * @fw_stopped_flg:	this flag indicates that software watchdog detected that
 *			the firmware is not responding
 * @num_inputs:		number of input pipes
 * @num_outputs:	number of output pipes
 * @ctrlbuf_active_flg:	this flag indicates ctrl buffer mechanism is in use
 * @dcpp_channel_ctrl_cache:	dcpp channel configuration cache
 * @dcpp_band_ctrl_cache:	dcpp band configuration cache
 *
 * manage the iomapped area to exchange messages/commands with axd
 */
struct axd_cmd {
	struct axd_memory_map __iomem *message;
	struct mutex cm_lock;
	wait_queue_head_t wait;
	unsigned int response_flg;
	struct workqueue_struct *in_workq;
	struct workqueue_struct *out_workq;
	struct axd_pipe in_pipes[AXD_MAX_PIPES];
	struct axd_pipe out_pipes[AXD_MAX_PIPES];
	int watchdogenabled;
	unsigned int discard_flg;
	unsigned int nonblock;
	unsigned int fw_stopped_flg;
	int num_inputs;
	int num_outputs;
	unsigned int ctrlbuf_active_flg;
	int dcpp_channel_ctrl_cache[AXD_MAX_PIPES];
	int dcpp_band_ctrl_cache[AXD_MAX_PIPES];
	unsigned int started_flg;
};

static inline void axd_set_flag(unsigned int *flag, unsigned int value)
{
	*flag = value;
	smp_wmb();	/* guarantee smp ordering */
}

static inline unsigned int axd_get_flag(unsigned int *flag)
{
	smp_rmb();	/* guarantee smp ordering */
	return *flag;
}

#define	CMD_TIMEOUT	(1*HZ)

/* Generic setup API */
void axd_cmd_init(struct axd_cmd *cmd, unsigned long cmd_address,
			unsigned long io_address, unsigned long phys_address);
int axd_cmd_set_pc(struct axd_cmd *cmd, unsigned int thread, unsigned long pc);
unsigned long  axd_cmd_get_datain_address(struct axd_cmd *cmd);
unsigned long  axd_cmd_get_datain_size(struct axd_cmd *cmd);
unsigned long  axd_cmd_get_dataout_address(struct axd_cmd *cmd);
unsigned long  axd_cmd_get_dataout_size(struct axd_cmd *cmd);

/* Info API */
void axd_cmd_get_version(struct axd_cmd *cmd, int *major,
						int *minor, int *patch);
int axd_cmd_get_num_pipes(struct axd_cmd *cmd, unsigned int *inpipes,
						unsigned int *outpipes);
void axd_cmd_get_decoders(struct axd_cmd *cmd, char *codecs);
void axd_cmd_get_encoders(struct axd_cmd *cmd, char *codecs);
int axd_cmd_xbar_present(struct axd_cmd *cmd);
int axd_cmd_mixer_get_eqenabled(struct axd_cmd *cmd, unsigned int pipe);
void axd_cmd_mixer_get_eqmastergain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain);
void axd_cmd_mixer_get_eqband0gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain);
void axd_cmd_mixer_get_eqband1gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain);
void axd_cmd_mixer_get_eqband2gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain);
void axd_cmd_mixer_get_eqband3gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain);
void axd_cmd_mixer_get_eqband4gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain);
void axd_cmd_mixer_get_mux(struct axd_cmd *cmd, unsigned int pipe,
								char *mux);
int axd_cmd_input_get_enabled(struct axd_cmd *cmd, unsigned int pipe);
void axd_cmd_input_get_source(struct axd_cmd *cmd, unsigned int pipe,
								char *source);
void axd_cmd_input_get_codec(struct axd_cmd *cmd, unsigned int pipe,
								char *codec);
void axd_cmd_input_get_gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain);
void axd_cmd_input_get_mute(struct axd_cmd *cmd, unsigned int pipe,
								int *muted);
void axd_cmd_input_get_upmix(struct axd_cmd *cmd, unsigned int pipe,
								char *upmix);
void axd_cmd_input_get_decoder_config(struct axd_cmd *cmd, unsigned int pipe,
								char *config);
int axd_cmd_output_get_enabled(struct axd_cmd *cmd, unsigned int pipe);
void axd_cmd_output_get_codec(struct axd_cmd *cmd, unsigned int pipe,
								char *codec);
void axd_cmd_output_get_sink(struct axd_cmd *cmd, unsigned int pipe,
								char *sink);
void axd_cmd_output_get_downmix(struct axd_cmd *cmd, unsigned int pipe,
								char *downmix);
int axd_cmd_output_get_eqenabled(struct axd_cmd *cmd, unsigned int pipe);
void axd_cmd_output_get_eqmastergain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain);
void axd_cmd_output_get_eqband0gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain);
void axd_cmd_output_get_eqband1gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain);
void axd_cmd_output_get_eqband2gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain);
void axd_cmd_output_get_eqband3gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain);
void axd_cmd_output_get_eqband4gain(struct axd_cmd *cmd, unsigned int pipe,
								int *gain);
void axd_cmd_output_get_encoder_config(struct axd_cmd *cmd, unsigned int pipe,
								char *config);
void axd_cmd_output_get_geq_power(struct axd_cmd *cmd, unsigned int pipe,
							char *buf, int channel);
/* DCPP */
int axd_cmd_output_dcpp_select_channel(struct axd_cmd *cmd, unsigned int pipe,
					bool subband, unsigned int channel);
int axd_cmd_output_dcpp_select_band(struct axd_cmd *cmd, unsigned int pipe,
							unsigned int band);

unsigned int axd_cmd_output_get_dcpp_enabled(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_mode(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_channels(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_eq_mode(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_eq_bands(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_max_delay_samples(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_channel_delay_samples(struct axd_cmd *cmd,
				unsigned int pipe, unsigned int channel);
unsigned int axd_cmd_output_get_dcpp_channel_eq_output_volume(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel);
unsigned int axd_cmd_output_get_dcpp_channel_eq_passthrough_gain(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel);
unsigned int axd_cmd_output_get_dcpp_channel_eq_inverse_passthrough_gain(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel);
unsigned int axd_cmd_output_get_dcpp_channel_bass_shelf_shift(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel);
unsigned int axd_cmd_output_get_dcpp_channel_bass_shelf_a0(struct axd_cmd *cmd,
				unsigned int pipe, unsigned int channel);
unsigned int axd_cmd_output_get_dcpp_channel_bass_shelf_a1(struct axd_cmd *cmd,
				unsigned int pipe, unsigned int channel);
unsigned int axd_cmd_output_get_dcpp_channel_bass_shelf_a2(struct axd_cmd *cmd,
				unsigned int pipe, unsigned int channel);
unsigned int axd_cmd_output_get_dcpp_channel_bass_shelf_b0(struct axd_cmd *cmd,
				unsigned int pipe, unsigned int channel);
unsigned int axd_cmd_output_get_dcpp_channel_bass_shelf_b1(struct axd_cmd *cmd,
				unsigned int pipe, unsigned int channel);
unsigned int axd_cmd_output_get_dcpp_channel_treble_shelf_shift(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel);
unsigned int axd_cmd_output_get_dcpp_channel_treble_shelf_a0(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel);
unsigned int axd_cmd_output_get_dcpp_channel_treble_shelf_a1(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel);
unsigned int axd_cmd_output_get_dcpp_channel_treble_shelf_a2(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel);
unsigned int axd_cmd_output_get_dcpp_channel_treble_shelf_b0(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel);
unsigned int axd_cmd_output_get_dcpp_channel_treble_shelf_b1(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int channel);
unsigned int axd_cmd_output_get_dcpp_channel_eq_gain(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int band);
unsigned int axd_cmd_output_get_dcpp_channel_eq_a0(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int band);
unsigned int axd_cmd_output_get_dcpp_channel_eq_a1(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int band);
unsigned int axd_cmd_output_get_dcpp_channel_eq_a2(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int band);
unsigned int axd_cmd_output_get_dcpp_channel_eq_b0(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int band);
unsigned int axd_cmd_output_get_dcpp_channel_eq_b1(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int band);
unsigned int axd_cmd_output_get_dcpp_channel_eq_shift(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int band);
unsigned int axd_cmd_output_get_dcpp_subband_eq_bands(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_subband_enabled(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_subband_input_channel_mask(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_subband_delay_samples(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_subband_eq_output_volume(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_subband_eq_passthrough_gain(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_subband_eq_inverse_passthrough_gain(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_subband_eq_gain(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int band);
unsigned int axd_cmd_output_get_dcpp_subband_eq_a0(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int band);
unsigned int axd_cmd_output_get_dcpp_subband_eq_a1(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int band);
unsigned int axd_cmd_output_get_dcpp_subband_eq_a2(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int band);
unsigned int axd_cmd_output_get_dcpp_subband_eq_b0(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int band);
unsigned int axd_cmd_output_get_dcpp_subband_eq_b1(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int band);
unsigned int axd_cmd_output_get_dcpp_subband_eq_shift(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int band);
unsigned int axd_cmd_output_get_dcpp_subband_low_pass_filter_a0(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_subband_low_pass_filter_a1(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_subband_low_pass_filter_a2(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_subband_low_pass_filter_b0(
					struct axd_cmd *cmd, unsigned int pipe);
unsigned int axd_cmd_output_get_dcpp_subband_low_pass_filter_b1(
					struct axd_cmd *cmd, unsigned int pipe);
/* Config API */
void axd_cmd_mixer_set_eqenabled(struct axd_cmd *cmd, unsigned int pipe,
								int enable);
void axd_cmd_mixer_set_eqmastergain(struct axd_cmd *cmd, unsigned int pipe,
								int gain);
void axd_cmd_mixer_set_eqband0gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain);
void axd_cmd_mixer_set_eqband1gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain);
void axd_cmd_mixer_set_eqband2gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain);
void axd_cmd_mixer_set_eqband3gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain);
void axd_cmd_mixer_set_eqband4gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain);
void axd_cmd_mixer_set_mux(struct axd_cmd *cmd, unsigned int pipe,
								int mux);
int axd_cmd_input_set_enabled(struct axd_cmd *cmd, unsigned int pipe,
								int enable);
void axd_cmd_input_set_source(struct axd_cmd *cmd, unsigned int pipe,
								int source);
void axd_cmd_input_set_codec(struct axd_cmd *cmd, unsigned int pipe,
								int codec);
void axd_cmd_input_set_gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain);
void axd_cmd_input_set_mute(struct axd_cmd *cmd, unsigned int pipe,
								int mute);
void axd_cmd_input_set_upmix(struct axd_cmd *cmd, unsigned int pipe,
								int upmix);
void axd_cmd_input_set_decoder_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config);
int axd_cmd_output_set_enabled(struct axd_cmd *cmd, unsigned int pipe,
								int enable);
void axd_cmd_output_set_codec(struct axd_cmd *cmd, unsigned int pipe,
								int codec);
void axd_cmd_output_set_sink(struct axd_cmd *cmd, unsigned int pipe,
								int sink);
void axd_cmd_output_set_downmix(struct axd_cmd *cmd, unsigned int pipe,
								int downmix);
void axd_cmd_output_set_eqenabled(struct axd_cmd *cmd, unsigned int pipe,
								int enable);
void axd_cmd_output_set_eqmastergain(struct axd_cmd *cmd, unsigned int pipe,
								int gain);
void axd_cmd_output_set_eqband0gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain);
void axd_cmd_output_set_eqband1gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain);
void axd_cmd_output_set_eqband2gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain);
void axd_cmd_output_set_eqband3gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain);
void axd_cmd_output_set_eqband4gain(struct axd_cmd *cmd, unsigned int pipe,
								int gain);
/* DCPP */
int axd_cmd_output_set_dcpp_enabled(struct axd_cmd *cmd, unsigned int pipe,
								int enable);
int axd_cmd_output_set_dcpp_mode(struct axd_cmd *cmd, unsigned int pipe,
							unsigned int mode);
int axd_cmd_output_set_dcpp_eq_mode(struct axd_cmd *cmd, unsigned int pipe,
							unsigned int mode);
int axd_cmd_output_set_dcpp_channel_delay_samples(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data);
int axd_cmd_output_set_dcpp_channel_eq_output_volume(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data);
int axd_cmd_output_set_dcpp_channel_eq_passthrough_gain(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data);
int axd_cmd_output_set_dcpp_channel_eq_inverse_passthrough_gain(
				struct axd_cmd *cmd, unsigned int pipe,
				unsigned int channel, unsigned int data);
int axd_cmd_output_set_dcpp_channel_bass_shelf_shift(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data);
int axd_cmd_output_set_dcpp_channel_bass_shelf_a0(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data);
int axd_cmd_output_set_dcpp_channel_bass_shelf_a1(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data);
int axd_cmd_output_set_dcpp_channel_bass_shelf_a2(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data);
int axd_cmd_output_set_dcpp_channel_bass_shelf_b0(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data);
int axd_cmd_output_set_dcpp_channel_bass_shelf_b1(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data);
int axd_cmd_output_set_dcpp_channel_treble_shelf_shift(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data);
int axd_cmd_output_set_dcpp_channel_treble_shelf_a0(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data);
int axd_cmd_output_set_dcpp_channel_treble_shelf_a1(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data);
int axd_cmd_output_set_dcpp_channel_treble_shelf_a2(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data);
int axd_cmd_output_set_dcpp_channel_treble_shelf_b0(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data);
int axd_cmd_output_set_dcpp_channel_treble_shelf_b1(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int channel, unsigned int data);
int axd_cmd_output_set_dcpp_channel_eq_gain(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int channel,
					unsigned int band, unsigned int data);
int axd_cmd_output_set_dcpp_channel_eq_a0(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int channel,
					unsigned int band, unsigned int data);
int axd_cmd_output_set_dcpp_channel_eq_a1(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int channel,
					unsigned int band, unsigned int data);
int axd_cmd_output_set_dcpp_channel_eq_a2(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int channel,
					unsigned int band, unsigned int data);
int axd_cmd_output_set_dcpp_channel_eq_b0(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int channel,
					unsigned int band, unsigned int data);
int axd_cmd_output_set_dcpp_channel_eq_b1(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int channel,
					unsigned int band, unsigned int data);
int axd_cmd_output_set_dcpp_channel_eq_shift(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int channel,
					unsigned int band, unsigned int data);
int axd_cmd_output_set_dcpp_subband_enabled(struct axd_cmd *cmd,
						unsigned int pipe, int enable);
int axd_cmd_output_set_dcpp_subband_input_channel_mask(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data);
int axd_cmd_output_set_dcpp_subband_delay_samples(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data);
int axd_cmd_output_set_dcpp_subband_eq_output_volume(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data);
int axd_cmd_output_set_dcpp_subband_eq_passthrough_gain(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data);
int axd_cmd_output_set_dcpp_subband_eq_inverse_passthrough_gain(
		struct axd_cmd *cmd, unsigned int pipe, unsigned int data);
int axd_cmd_output_set_dcpp_subband_eq_gain(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int band, unsigned int data);
int axd_cmd_output_set_dcpp_subband_eq_a0(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int band, unsigned int data);
int axd_cmd_output_set_dcpp_subband_eq_a1(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int band, unsigned int data);
int axd_cmd_output_set_dcpp_subband_eq_a2(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int band, unsigned int data);
int axd_cmd_output_set_dcpp_subband_eq_b0(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int band, unsigned int data);
int axd_cmd_output_set_dcpp_subband_eq_b1(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int band, unsigned int data);
int axd_cmd_output_set_dcpp_subband_eq_shift(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int band, unsigned int data);
int axd_cmd_output_set_dcpp_subband_low_pass_filter_a0(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data);
int axd_cmd_output_set_dcpp_subband_low_pass_filter_a1(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data);
int axd_cmd_output_set_dcpp_subband_low_pass_filter_a2(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data);
int axd_cmd_output_set_dcpp_subband_low_pass_filter_b0(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data);
int axd_cmd_output_set_dcpp_subband_low_pass_filter_b1(struct axd_cmd *cmd,
					unsigned int pipe, unsigned int data);
void axd_cmd_output_set_encoder_config(struct axd_cmd *cmd, unsigned int pipe,
							const char *config);
unsigned int axd_cmd_info_get_resampler_fin(struct axd_cmd *cmd,
							unsigned int pipe);
unsigned int axd_cmd_info_get_resampler_fout(struct axd_cmd *cmd,
							unsigned int pipe);
void axd_cmd_info_set_resampler_fout(struct axd_cmd *cmd, unsigned int pipe,
							unsigned int fout);
unsigned int axd_cmd_input_get_buffer_occupancy(struct axd_cmd *cmd,
							unsigned int pipe);
void axd_cmd_input_set_buffer_occupancy(struct axd_cmd *cmd, unsigned int pipe,
							unsigned int bo);

/* Channel setup and access API */
int axd_cmd_inpipe_start(struct axd_cmd *cmd, unsigned int pipe);
void axd_cmd_inpipe_stop(struct axd_cmd *cmd, unsigned int pipe);
void axd_cmd_inpipe_reset(struct axd_cmd *cmd, unsigned int pipe);
int axd_cmd_inpipe_active(struct axd_cmd *cmd, unsigned int pipe);
int axd_cmd_outpipe_start(struct axd_cmd *cmd, unsigned int pipe);
void axd_cmd_outpipe_stop(struct axd_cmd *cmd, unsigned int pipe);
void axd_cmd_outpipe_reset(struct axd_cmd *cmd, unsigned int pipe);
int axd_cmd_send_buffer(struct axd_cmd *cmd, unsigned int pipe,
				const char __user *buf, unsigned int size);
void axd_cmd_send_buffer_abort(struct axd_cmd *cmd, unsigned int pipe);
int axd_cmd_recv_buffer(struct axd_cmd *cmd, unsigned int pipe,
					char __user *buf, unsigned int size);
void axd_cmd_recv_buffer_abort(struct axd_cmd *cmd, unsigned int pipe);
int axd_cmd_install_irq(struct axd_cmd *cmd, unsigned int irqnum);
void axd_cmd_free_irq(struct axd_cmd *cmd, unsigned int irqnum);
int axd_cmd_reset_pipe(struct axd_cmd *cmd, unsigned int pipe);

/* generic helper function required in several places */
char *axd_io_2_phys(const char *buf);
char *axd_phys_2_io(const char *buf);

/* Register write buffer */
int axd_flush_reg_buf(struct axd_cmd *cmd);

#endif /* AXD_CMDS_H_ */
