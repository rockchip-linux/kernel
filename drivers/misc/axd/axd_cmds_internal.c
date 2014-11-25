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
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/sched.h>

#include "axd_cmds_internal.h"
#include "axd_module.h"
#include "axd_platform.h"

#define WRONG_PIPE_STR		"Wrong pipe number: %d\n"
#define WRONG_BAND_STR		"Wrong band number: %d\n"

/*
 * Send/Clear control kick.
 *
 * NOTE:
 * Must acquire axd_platform_lock() before accessing kick and interrupt status
 * registers as the AXD firmware might be accessing them at the same time.
 */
inline void axd_ctrl_kick(struct axd_memory_map __iomem *message)
{
	unsigned long flags;
	unsigned int temp;

	flags = axd_platform_lock();
	temp = ioread32(&message->kick) | AXD_ANY_KICK_BIT | AXD_KICK_CTRL_BIT;
	iowrite32(temp, &message->kick);
	axd_platform_unlock(flags);
	axd_platform_kick();
}
inline void axd_kick_status_clear(struct axd_memory_map __iomem *message)
{
	unsigned long flags;
	unsigned int temp;

	flags = axd_platform_lock();
	temp = ioread32(&message->int_status) & ~AXD_INT_KICK_DONE;
	iowrite32(temp, &message->int_status);
	axd_platform_unlock(flags);
}
/*
 * Wait until axd is ready again. Must be called while cm_lock is held.
 */
int axd_wait_ready(struct axd_memory_map __iomem *message)
{
#define BUSYWAIT_TIME		1
#define BUSYWAIT_TIMEOUT	100
	unsigned int timeout = 0;

	while (ioread32(&message->control_command) != AXD_CTRL_CMD_READY) {
		mdelay(BUSYWAIT_TIME);
		timeout += BUSYWAIT_TIME;
		if (timeout == BUSYWAIT_TIMEOUT)
			return -1;
	}
	return 0;
}

/*
 * Reads a register from the MemoryMapped register interface.
 * @cmd: pointer to initialized struct axd_cmd.
 * @reg: the register address to be accessed.
 */
int axd_read_reg(struct axd_cmd *cmd, unsigned int reg, unsigned int *data)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	struct axd_memory_map __iomem *message = cmd->message;
	struct mutex *cm_lock = &cmd->cm_lock;
	int ret;

	mutex_lock(cm_lock);
	if (axd_get_flag(&cmd->fw_stopped_flg)) {
		mutex_unlock(cm_lock);
		return -1;
	}
	axd_set_flag(&cmd->response_flg, 0);
	iowrite32(AXD_CTRL_CMD_READ_REGISTER | reg, &message->control_command);
	axd_ctrl_kick(message);
	ret = wait_event_timeout(cmd->wait,
			axd_get_flag(&cmd->response_flg) != 0, CMD_TIMEOUT);
	*data = ioread32(&message->control_data);
	mutex_unlock(cm_lock);
	if (!ret) {
		dev_warn(axd->dev, "failed to read reg 0x%04X\n", reg);
		*data = 0;
		return -1;
	}
	return 0;
}

/*
 * Writes control data to the MemoryMapped control interface.
 * We assume that cm_lock is held before this function is called.
 * @cmd: pointer to initialized struct axd_cmd.
 * @ctrl_command: the control command to write.
 * @ctrl_data: the control value to write.
 */
int axd_write_ctrl(struct axd_cmd *cmd, unsigned int ctrl_command,
						unsigned int ctrl_data)
{
	struct axd_memory_map __iomem *message = cmd->message;
	int ret;

	axd_set_flag(&cmd->response_flg, 0);
	iowrite32(ctrl_data, &message->control_data);
	iowrite32(ctrl_command, &message->control_command);
	axd_ctrl_kick(message);
	ret = wait_event_timeout(cmd->wait,
			axd_get_flag(&cmd->response_flg) != 0, CMD_TIMEOUT);
	return ret;
}

/*
 * Writes value to a register int the MemoryMapped register interface.
 * @cmd: pointer to initialized struct axd_cmd.
 * @reg: the register address to be accessed.
 * @value: the new value to write.
 */
int axd_write_reg(struct axd_cmd *cmd, unsigned int reg, unsigned int value)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	struct mutex *cm_lock = &cmd->cm_lock;
	int ret;

	mutex_lock(cm_lock);
	if (axd_get_flag(&cmd->fw_stopped_flg)) {
		mutex_unlock(cm_lock);
		return -1;
	}
	ret = axd_write_ctrl(cmd, AXD_CTRL_CMD_WRITE_REGISTER | reg, value);
	mutex_unlock(cm_lock);
	if (!ret) {
		dev_warn(axd->dev, "failed to write reg 0x%04X\n", reg);
		return -1;
	}

	return 0;
}

int axd_write_reg_buf(struct axd_cmd *cmd, unsigned int reg, unsigned int value)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	struct mutex *cm_lock = &cmd->cm_lock;
	struct axd_ctrlbuf_item __iomem *buf;
	unsigned int ctrlbuf_ctrl = ioread32(&cmd->message->ctrlbuf_ctrl);
	unsigned int ctrlbuf_size = ioread32(&cmd->message->ctrlbuf_size);
	unsigned int temp;

	if (!axd_get_flag(&cmd->ctrlbuf_active_flg)) {
		/* If the ctrlbuf isn't active, fall back to simple reg write */
		return axd_write_reg(cmd, reg, value);
	}

	mutex_lock(cm_lock);
	if (axd_get_flag(&cmd->fw_stopped_flg)) {
		mutex_unlock(cm_lock);
		return -1;
	}

	if (ctrlbuf_ctrl >= ctrlbuf_size) {
		mutex_unlock(cm_lock);
		dev_err(axd->dev, "Could not write to ctrlbuf: full\n");
		return -1;
	}

	buf = &cmd->message->ctrlbuf[ctrlbuf_ctrl];

	iowrite32(AXD_CTRL_CMD_WRITE_REGISTER | reg, &buf->reg);
	iowrite32(value, &buf->val);

	temp = ioread32(&cmd->message->ctrlbuf_ctrl) + 1;
	iowrite32(temp, &cmd->message->ctrlbuf_ctrl);

	mutex_unlock(cm_lock);

	return 0;
}

int axd_flush_reg_buf(struct axd_cmd *cmd)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	struct mutex *cm_lock = &cmd->cm_lock;
	int ret;

	mutex_lock(cm_lock);
	if (axd_get_flag(&cmd->fw_stopped_flg)) {
		mutex_unlock(cm_lock);
		return -1;
	}

	if (ioread32(&cmd->message->ctrlbuf_ctrl) == 0) {
		mutex_unlock(cm_lock);
		dev_warn(axd->dev, "Tried to flush empty ctrlbuf\n");
		return -1;
	}

	ret = axd_write_ctrl(cmd, AXD_CTRL_CMD_CTRLBUF_FLUSH, 0);
	if (!ret) {
		/* Drop buffer and ignore any response */
		iowrite32(0, &cmd->message->ctrlbuf_ctrl);

		mutex_unlock(cm_lock);
		dev_err(axd->dev, "Could not write control command to flush buffer");
		return -EIO;
	}

	/* Ignore any response */
	iowrite32(0, &cmd->message->ctrlbuf_ctrl);

	mutex_unlock(cm_lock);

	return 0;
}

/* Returns the address of the correct mixer mux register for @pipe */
unsigned int axd_get_mixer_mux_reg(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_MUX0;
		break;
	case 1:
		reg = AXD_REG_MUX1;
		break;
	case 2:
		reg = AXD_REG_MUX2;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the number of the currently set input codec */
unsigned int axd_get_input_codec_number(struct axd_cmd *cmd, unsigned int pipe)
{
	unsigned int reg = axd_get_input_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return -1;
	axd_read_reg(cmd, reg, &control);
	return (control & AXD_INCTRL_CODEC_BITS) >> AXD_INCTRL_CODEC_SHIFT;
}

/* Returns the address of the correct input control register for @pipe */
unsigned int axd_get_input_control_reg(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_INPUT0_CONTROL;
		break;
	case 1:
		reg = AXD_REG_INPUT1_CONTROL;
		break;
	case 2:
		reg = AXD_REG_INPUT2_CONTROL;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the correct input gain register for @pipe */
unsigned int axd_get_input_gain_reg(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_INPUT0_GAIN;
		break;
	case 1:
		reg = AXD_REG_INPUT1_GAIN;
		break;
	case 2:
		reg = AXD_REG_INPUT2_GAIN;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the correct input mute register for @pipe */
unsigned int axd_get_input_mute_reg(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_INPUT0_MUTE;
		break;
	case 1:
		reg = AXD_REG_INPUT1_MUTE;
		break;
	case 2:
		reg = AXD_REG_INPUT2_MUTE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the correct input UpMix register for @pipe */
unsigned int axd_get_input_upmix_reg(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_INPUT0_UPMIX;
		break;
	case 1:
		reg = AXD_REG_INPUT1_UPMIX;
		break;
	case 2:
		reg = AXD_REG_INPUT2_UPMIX;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the correct input bufer occupancy reg for @pipe */
unsigned int axd_get_input_buffer_occupancy_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_INPUT0_BUFFER_OCCUPANCY;
		break;
	case 1:
		reg = AXD_REG_INPUT1_BUFFER_OCCUPANCY;
		break;
	case 2:
		reg = AXD_REG_INPUT2_BUFFER_OCCUPANCY;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the number of the currently set output codec */
unsigned int axd_get_output_codec_number(struct axd_cmd *cmd, unsigned int pipe)
{
	unsigned int reg = axd_get_output_control_reg(cmd, pipe);
	unsigned int control;

	if (unlikely(!reg))
		return -1;
	axd_read_reg(cmd, reg, &control);
	return (control & AXD_OUTCTRL_CODEC_BITS) >> AXD_OUTCTRL_CODEC_SHIFT;
}

/* Returns the address of the correct output control register for @pipe */
unsigned int axd_get_output_control_reg(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_CONTROL;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_CONTROL;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_CONTROL;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the correct output DownMix register for @pipe */
unsigned int axd_get_output_downmix_reg(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DOWNMIX;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DOWNMIX;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DOWNMIX;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/*
 * Returns the address of the output EQ Ctrl / Master Gain register for
 * @pipe
 */
unsigned int axd_get_output_eqcontrol_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_EQCTRL;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_EQCTRL;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_EQCTRL;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the output EQ Band0 register for @pipe*/
unsigned int axd_get_output_eqband0_reg(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_EQBAND0;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_EQBAND0;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_EQBAND0;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the output EQ Band1 register for @pipe*/
unsigned int axd_get_output_eqband1_reg(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_EQBAND1;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_EQBAND1;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_EQBAND1;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the output EQ Band2 register for @pipe*/
unsigned int axd_get_output_eqband2_reg(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_EQBAND2;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_EQBAND2;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_EQBAND2;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the output EQ Band3 register for @pipe*/
unsigned int axd_get_output_eqband3_reg(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_EQBAND3;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_EQBAND3;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_EQBAND3;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the output EQ Band4 register for @pipe*/
unsigned int axd_get_output_eqband4_reg(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_EQBAND4;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_EQBAND4;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_EQBAND4;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* DCPP */

int axd_cmd_output_dcpp_select_channel(struct axd_cmd *cmd, unsigned int pipe,
					bool subband, unsigned int channel)
{
	unsigned int reg;
	unsigned int control;
	int ret;

	reg = axd_get_output_dcpp_channel_ctrl_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	/* Generate channel selector */
	control = 0;

	if (subband)
		control = AXD_DCPP_CHANNEL_CTRL_SUBBAND_BITS;
	else
		control = channel << AXD_DCPP_CHANNEL_CTRL_CHANNEL_SHIFT;

	/* Compare with last channel selector */
	if (control == cmd->dcpp_channel_ctrl_cache[pipe]) {
		ret = 0;
	} else {
		ret = axd_write_reg_buf(cmd, reg, control);
		cmd->dcpp_channel_ctrl_cache[pipe] = control;
	}

	return ret;
}

int axd_cmd_output_dcpp_select_band(struct axd_cmd *cmd, unsigned int pipe,
							unsigned int band)
{
	unsigned int reg;
	unsigned int control;
	int ret;

	reg = axd_get_output_dcpp_band_ctrl_reg(cmd, pipe);
	if (unlikely(!reg))
		return -1;

	/* Generate band selector */
	control = band;

	/* Compare with last band selector */
	if (control == cmd->dcpp_band_ctrl_cache[pipe]) {
		ret = 0;
	} else {
		ret = axd_write_reg_buf(cmd, reg, control);
		cmd->dcpp_band_ctrl_cache[pipe] = control;
	}

	return ret;
}

unsigned int axd_get_output_dcpp_control_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CONTROL;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CONTROL;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CONTROL;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_max_delay_samples_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_MAX_DELAY_SAMPLES;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_MAX_DELAY_SAMPLES;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_MAX_DELAY_SAMPLES;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_ctrl_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg = 0;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_CONTROL;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_CONTROL;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_CONTROL;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
	}
	return reg;
}

unsigned int axd_get_output_dcpp_band_ctrl_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg = 0;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_BAND_CONTROL;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_BAND_CONTROL;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_BAND_CONTROL;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_delay_samples_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_DELAY_SAMPLES;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_DELAY_SAMPLES;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_DELAY_SAMPLES;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_eq_output_volume_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_EQ_OUTPUT_VOLUME;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_EQ_OUTPUT_VOLUME;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_EQ_OUTPUT_VOLUME;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_eq_passthrough_gain_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_EQ_PASSTHROUGH_GAIN;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_EQ_PASSTHROUGH_GAIN;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_EQ_PASSTHROUGH_GAIN;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_eq_inverse_passthrough_gain_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_EQ_INVERSE_PASSTHROUGH_GAIN;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_EQ_INVERSE_PASSTHROUGH_GAIN;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_EQ_INVERSE_PASSTHROUGH_GAIN;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_bass_shelf_shift_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_BASS_SHELF_SHIFT;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_BASS_SHELF_SHIFT;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_BASS_SHELF_SHIFT;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_bass_shelf_a0_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_BASS_SHELF_A0;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_BASS_SHELF_A0;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_BASS_SHELF_A0;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_bass_shelf_a1_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_BASS_SHELF_A1;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_BASS_SHELF_A1;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_BASS_SHELF_A1;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_bass_shelf_a2_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_BASS_SHELF_A2;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_BASS_SHELF_A2;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_BASS_SHELF_A2;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_bass_shelf_b0_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_BASS_SHELF_B0;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_BASS_SHELF_B0;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_BASS_SHELF_B0;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_bass_shelf_b1_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_BASS_SHELF_B1;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_BASS_SHELF_B1;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_BASS_SHELF_B1;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_treble_shelf_shift_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_TREBLE_SHELF_SHIFT;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_TREBLE_SHELF_SHIFT;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_TREBLE_SHELF_SHIFT;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_treble_shelf_a0_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_TREBLE_SHELF_A0;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_TREBLE_SHELF_A0;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_TREBLE_SHELF_A0;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_treble_shelf_a1_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_TREBLE_SHELF_A1;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_TREBLE_SHELF_A1;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_TREBLE_SHELF_A1;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_treble_shelf_a2_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_TREBLE_SHELF_A2;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_TREBLE_SHELF_A2;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_TREBLE_SHELF_A2;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_treble_shelf_b0_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_TREBLE_SHELF_B0;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_TREBLE_SHELF_B0;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_TREBLE_SHELF_B0;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_treble_shelf_b1_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_TREBLE_SHELF_B1;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_TREBLE_SHELF_B1;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_TREBLE_SHELF_B1;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_eq_gain_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_EQ_BAND_GAIN;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_EQ_BAND_GAIN;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_EQ_BAND_GAIN;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_eq_a0_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_EQ_BAND_A0;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_EQ_BAND_A0;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_EQ_BAND_A0;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_eq_a1_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_EQ_BAND_A1;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_EQ_BAND_A1;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_EQ_BAND_A1;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_eq_a2_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_EQ_BAND_A2;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_EQ_BAND_A2;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_EQ_BAND_A2;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_eq_b0_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_EQ_BAND_B0;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_EQ_BAND_B0;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_EQ_BAND_B0;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_eq_b1_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_EQ_BAND_B1;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_EQ_BAND_B1;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_EQ_BAND_B1;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_channel_eq_shift_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_CHANNEL_EQ_BAND_SHIFT;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_CHANNEL_EQ_BAND_SHIFT;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_CHANNEL_EQ_BAND_SHIFT;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_subband_low_pass_filter_a0_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg = 0;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_SUBBAND_LOW_PASS_FILTER_A0;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_SUBBAND_LOW_PASS_FILTER_A0;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_SUBBAND_LOW_PASS_FILTER_A0;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_subband_low_pass_filter_a1_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg = 0;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_SUBBAND_LOW_PASS_FILTER_A1;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_SUBBAND_LOW_PASS_FILTER_A1;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_SUBBAND_LOW_PASS_FILTER_A1;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_subband_low_pass_filter_a2_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg = 0;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_SUBBAND_LOW_PASS_FILTER_A2;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_SUBBAND_LOW_PASS_FILTER_A2;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_SUBBAND_LOW_PASS_FILTER_A2;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_subband_low_pass_filter_b0_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg = 0;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_SUBBAND_LOW_PASS_FILTER_B0;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_SUBBAND_LOW_PASS_FILTER_B0;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_SUBBAND_LOW_PASS_FILTER_B0;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_dcpp_subband_low_pass_filter_b1_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg = 0;

	switch (pipe) {
	case 0:
		reg = AXD_REG_OUTPUT0_DCPP_SUBBAND_LOW_PASS_FILTER_B1;
		break;
	case 1:
		reg = AXD_REG_OUTPUT1_DCPP_SUBBAND_LOW_PASS_FILTER_B1;
		break;
	case 2:
		reg = AXD_REG_OUTPUT2_DCPP_SUBBAND_LOW_PASS_FILTER_B1;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the aac version for decoder at @pipe*/
unsigned int axd_get_decoder_aac_version_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_AAC_VERSION;
		break;
	case 1:
		reg = AXD_REG_DEC1_AAC_VERSION;
		break;
	case 2:
		reg = AXD_REG_DEC2_AAC_VERSION;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the aac pipes for decoder at @pipe*/
unsigned int axd_get_decoder_aac_channels_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_AAC_CHANNELS;
		break;
	case 1:
		reg = AXD_REG_DEC1_AAC_CHANNELS;
		break;
	case 2:
		reg = AXD_REG_DEC2_AAC_CHANNELS;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the aac profile for decoder at @pipe*/
unsigned int axd_get_decoder_aac_profile_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_AAC_PROFILE;
		break;
	case 1:
		reg = AXD_REG_DEC1_AAC_PROFILE;
		break;
	case 2:
		reg = AXD_REG_DEC2_AAC_PROFILE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the aac stream type for decoder at @pipe*/
unsigned int axd_get_decoder_aac_streamtype_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_AAC_STREAM_TYPE;
		break;
	case 1:
		reg = AXD_REG_DEC1_AAC_STREAM_TYPE;
		break;
	case 2:
		reg = AXD_REG_DEC2_AAC_STREAM_TYPE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the aac stream type for decoder at @pipe*/
unsigned int axd_get_decoder_aac_samplerate_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_AAC_SAMPLERATE;
		break;
	case 1:
		reg = AXD_REG_DEC1_AAC_SAMPLERATE;
		break;
	case 2:
		reg = AXD_REG_DEC2_AAC_SAMPLERATE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_decoder_ac3_channels_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_AC3_CHANNELS;
		break;
	case 1:
		reg = AXD_REG_DEC1_AC3_CHANNELS;
		break;
	case 2:
		reg = AXD_REG_DEC2_AC3_CHANNELS;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_decoder_ac3_channel_order_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_AC3_CHANNEL_ORDER;
		break;
	case 1:
		reg = AXD_REG_DEC1_AC3_CHANNEL_ORDER;
		break;
	case 2:
		reg = AXD_REG_DEC2_AC3_CHANNEL_ORDER;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_decoder_ac3_mode_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_AC3_MODE;
		break;
	case 1:
		reg = AXD_REG_DEC1_AC3_MODE;
		break;
	case 2:
		reg = AXD_REG_DEC2_AC3_MODE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the cook flavour for decoder at @pipe*/
unsigned int axd_get_decoder_cook_flavour_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_COOK_FLAVOUR;
		break;
	case 1:
		reg = AXD_REG_DEC1_COOK_FLAVOUR;
		break;
	case 2:
		reg = AXD_REG_DEC2_COOK_FLAVOUR;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the flac pipes for decoder at @pipe*/
unsigned int axd_get_decoder_flac_channels_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_FLAC_CHANNELS;
		break;
	case 1:
		reg = AXD_REG_DEC1_FLAC_CHANNELS;
		break;
	case 2:
		reg = AXD_REG_DEC2_FLAC_CHANNELS;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the flac sample rate for decoder at @pipe*/
unsigned int axd_get_decoder_flac_samplerate_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_FLAC_SAMPLERATE;
		break;
	case 1:
		reg = AXD_REG_DEC1_FLAC_SAMPLERATE;
		break;
	case 2:
		reg = AXD_REG_DEC2_FLAC_SAMPLERATE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the flac bits per sample for decoder at @pipe*/
unsigned int axd_get_decoder_flac_bitspersample_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_FLAC_BITS_PER_SAMPLE;
		break;
	case 1:
		reg = AXD_REG_DEC1_FLAC_BITS_PER_SAMPLE;
		break;
	case 2:
		reg = AXD_REG_DEC2_FLAC_BITS_PER_SAMPLE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the flac md5 checking for decoder at @pipe*/
unsigned int axd_get_decoder_flac_md5checking_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_FLAC_MD5_CHECKING;
		break;
	case 1:
		reg = AXD_REG_DEC1_FLAC_MD5_CHECKING;
		break;
	case 2:
		reg = AXD_REG_DEC2_FLAC_MD5_CHECKING;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the mpeg num pipes for decoder at @pipe*/
unsigned int axd_get_decoder_mpeg_numchannels_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_MPEG_CHANNELS;
		break;
	case 1:
		reg = AXD_REG_DEC1_MPEG_CHANNELS;
		break;
	case 2:
		reg = AXD_REG_DEC2_MPEG_CHANNELS;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the mpeg mlchannel for decoder at @pipe*/
unsigned int axd_get_decoder_mpeg_mlchannel_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_MPEG_MLCHANNEL;
		break;
	case 1:
		reg = AXD_REG_DEC1_MPEG_MLCHANNEL;
		break;
	case 2:
		reg = AXD_REG_DEC2_MPEG_MLCHANNEL;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the wma player opt for decoder at @pipe*/
unsigned int axd_get_decoder_wma_playeropt_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_WMA_PLAYER_OPT;
		break;
	case 1:
		reg = AXD_REG_DEC1_WMA_PLAYER_OPT;
		break;
	case 2:
		reg = AXD_REG_DEC2_WMA_PLAYER_OPT;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the wma player drc setting for decoder at @pipe*/
unsigned int axd_get_decoder_wma_drcsetting_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_WMA_DRC_SETTING;
		break;
	case 1:
		reg = AXD_REG_DEC1_WMA_DRC_SETTING;
		break;
	case 2:
		reg = AXD_REG_DEC2_WMA_DRC_SETTING;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the wma player peak ref for decoder at @pipe*/
unsigned int axd_get_decoder_wma_peakampref_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_WMA_PEAK_AMP_REF;
		break;
	case 1:
		reg = AXD_REG_DEC1_WMA_PEAK_AMP_REF;
		break;
	case 2:
		reg = AXD_REG_DEC2_WMA_PEAK_AMP_REF;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the wma player rms ref for decoder at @pipe*/
unsigned int axd_get_decoder_wma_rmsampref_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_WMA_RMS_AMP_REF;
		break;
	case 1:
		reg = AXD_REG_DEC1_WMA_RMS_AMP_REF;
		break;
	case 2:
		reg = AXD_REG_DEC2_WMA_RMS_AMP_REF;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the wma player peak target for decoder at @pipe*/
unsigned int axd_get_decoder_wma_peakamptarget_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_WMA_PEAK_AMP_TARGET;
		break;
	case 1:
		reg = AXD_REG_DEC1_WMA_PEAK_AMP_TARGET;
		break;
	case 2:
		reg = AXD_REG_DEC2_WMA_PEAK_AMP_TARGET;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the wma player rms target for decoder at @pipe*/
unsigned int axd_get_decoder_wma_rmsamptarget_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_WMA_RMS_AMP_TARGET;
		break;
	case 1:
		reg = AXD_REG_DEC2_WMA_RMS_AMP_TARGET;
		break;
	case 2:
		reg = AXD_REG_DEC1_WMA_RMS_AMP_TARGET;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the wma pcm valid bits for decoder at @pipe*/
unsigned int axd_get_decoder_wma_pcmvalidbitspersample_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_WMA_PCM_VAL_BITS_PER_SAMPLE;
		break;
	case 1:
		reg = AXD_REG_DEC1_WMA_PCM_VAL_BITS_PER_SAMPLE;
		break;
	case 2:
		reg = AXD_REG_DEC2_WMA_PCM_VAL_BITS_PER_SAMPLE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the wma pcm container size for decoder at @pipe*/
unsigned int axd_get_decoder_wma_pcmcontainersize_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_WMA_PCM_CONTAINER_SIZE;
		break;
	case 1:
		reg = AXD_REG_DEC1_WMA_PCM_CONTAINER_SIZE;
		break;
	case 2:
		reg = AXD_REG_DEC2_WMA_PCM_CONTAINER_SIZE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the wma format tag for decoder at @pipe*/
unsigned int axd_get_decoder_wma_wmaformattag_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_WMA_WMA_FORMAT_TAG;
		break;
	case 1:
		reg = AXD_REG_DEC1_WMA_WMA_FORMAT_TAG;
		break;
	case 2:
		reg = AXD_REG_DEC2_WMA_WMA_FORMAT_TAG;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the wma format num pipes for decoder at @pipe*/
unsigned int axd_get_decoder_wma_wmanumchannels_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_WMA_WMA_CHANNELS;
		break;
	case 1:
		reg = AXD_REG_DEC1_WMA_WMA_CHANNELS;
		break;
	case 2:
		reg = AXD_REG_DEC2_WMA_WMA_CHANNELS;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the wma format sample/s for decoder at @pipe*/
unsigned int axd_get_decoder_wma_wmasamplespersec_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_WMA_WMA_SAMPLES_PER_SEC;
		break;
	case 1:
		reg = AXD_REG_DEC1_WMA_WMA_SAMPLES_PER_SEC;
		break;
	case 2:
		reg = AXD_REG_DEC2_WMA_WMA_SAMPLES_PER_SEC;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/*
 * Returns the address of the wma format average bytes per sample for decoder
 * at @pipe
 */
unsigned int axd_get_decoder_wma_wmaaveragebytespersec_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_WMA_WMA_AVG_BYTES_PER_SEC;
		break;
	case 1:
		reg = AXD_REG_DEC1_WMA_WMA_AVG_BYTES_PER_SEC;
		break;
	case 2:
		reg = AXD_REG_DEC2_WMA_WMA_AVG_BYTES_PER_SEC;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the wma format block align for decoder at @pipe*/
unsigned int axd_get_decoder_wma_wmablockalign_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_WMA_WMA_BLOCK_ALIGN;
		break;
	case 1:
		reg = AXD_REG_DEC1_WMA_WMA_BLOCK_ALIGN;
		break;
	case 2:
		reg = AXD_REG_DEC2_WMA_WMA_BLOCK_ALIGN;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the wma format valid bits for decoder at @pipe*/
unsigned int axd_get_decoder_wma_wmavalidbitspersample_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_WMA_WMA_VAL_BITS_PER_SAMPLE;
		break;
	case 1:
		reg = AXD_REG_DEC1_WMA_WMA_VAL_BITS_PER_SAMPLE;
		break;
	case 2:
		reg = AXD_REG_DEC2_WMA_WMA_VAL_BITS_PER_SAMPLE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the wma format pipe mask for decoder at @pipe*/
unsigned int axd_get_decoder_wma_wmachannelmask_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_WMA_WMA_CHANNEL_MASK;
		break;
	case 1:
		reg = AXD_REG_DEC1_WMA_WMA_CHANNEL_MASK;
		break;
	case 2:
		reg = AXD_REG_DEC2_WMA_WMA_CHANNEL_MASK;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the wma format encode options for decoder at @pipe*/
unsigned int axd_get_decoder_wma_wmaencodeoptions_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_WMA_WMA_ENCODE_OPTS;
		break;
	case 1:
		reg = AXD_REG_DEC1_WMA_WMA_ENCODE_OPTS;
		break;
	case 2:
		reg = AXD_REG_DEC2_WMA_WMA_ENCODE_OPTS;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the pcm samplerate reg for decoder at @pipe*/
unsigned int axd_get_decoder_pcm_samplerate_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_PCMIN0_SAMPLE_RATE;
		break;
	case 1:
		reg = AXD_REG_PCMIN1_SAMPLE_RATE;
		break;
	case 2:
		reg = AXD_REG_PCMIN2_SAMPLE_RATE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the pcm channels reg for decoder at @pipe*/
unsigned int axd_get_decoder_pcm_channels_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_PCMIN0_CHANNELS;
		break;
	case 1:
		reg = AXD_REG_PCMIN1_CHANNELS;
		break;
	case 2:
		reg = AXD_REG_PCMIN2_CHANNELS;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the pcm bitspersample reg for decoder at @pipe*/
unsigned int axd_get_decoder_pcm_bitspersample_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_PCMIN0_BITS_PER_SAMPLE;
		break;
	case 1:
		reg = AXD_REG_PCMIN1_BITS_PER_SAMPLE;
		break;
	case 2:
		reg = AXD_REG_PCMIN2_BITS_PER_SAMPLE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

/* Returns the address of the pcm justification reg for decoder at @pipe*/
unsigned int axd_get_decoder_pcm_justification_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_PCMIN0_JUSTIFICATION;
		break;
	case 1:
		reg = AXD_REG_PCMIN1_JUSTIFICATION;
		break;
	case 2:
		reg = AXD_REG_PCMIN2_JUSTIFICATION;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_decoder_ddplus_config_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_DDPLUS_CONFIG;
		break;
	case 1:
		reg = AXD_REG_DEC1_DDPLUS_CONFIG;
		break;
	case 2:
		reg = AXD_REG_DEC2_DDPLUS_CONFIG;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_decoder_ddplus_channel_order_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_DDPLUS_CHANNEL_ORDER;
		break;
	case 1:
		reg = AXD_REG_DEC1_DDPLUS_CHANNEL_ORDER;
		break;
	case 2:
		reg = AXD_REG_DEC2_DDPLUS_CHANNEL_ORDER;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_decoder_alac_channels_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_ALAC_CHANNELS;
		break;
	case 1:
		reg = AXD_REG_DEC1_ALAC_CHANNELS;
		break;
	case 2:
		reg = AXD_REG_DEC2_ALAC_CHANNELS;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_decoder_alac_depth_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_ALAC_DEPTH;
		break;
	case 1:
		reg = AXD_REG_DEC1_ALAC_DEPTH;
		break;
	case 2:
		reg = AXD_REG_DEC2_ALAC_DEPTH;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_decoder_alac_samplerate_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_ALAC_SAMPLE_RATE;
		break;
	case 1:
		reg = AXD_REG_DEC1_ALAC_SAMPLE_RATE;
		break;
	case 2:
		reg = AXD_REG_DEC2_ALAC_SAMPLE_RATE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_decoder_alac_framelength_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_ALAC_FRAME_LENGTH;
		break;
	case 1:
		reg = AXD_REG_DEC1_ALAC_FRAME_LENGTH;
		break;
	case 2:
		reg = AXD_REG_DEC2_ALAC_FRAME_LENGTH;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_decoder_alac_maxframebytes_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_ALAC_MAX_FRAME_BYTES;
		break;
	case 1:
		reg = AXD_REG_DEC1_ALAC_MAX_FRAME_BYTES;
		break;
	case 2:
		reg = AXD_REG_DEC2_ALAC_MAX_FRAME_BYTES;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_decoder_alac_avgbitrate_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_ALAC_AVG_BIT_RATE;
		break;
	case 1:
		reg = AXD_REG_DEC1_ALAC_AVG_BIT_RATE;
		break;
	case 2:
		reg = AXD_REG_DEC2_ALAC_AVG_BIT_RATE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_decoder_sbc_samplerate_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_SBC_SAMPLE_RATE;
		break;
	case 1:
		reg = AXD_REG_DEC1_SBC_SAMPLE_RATE;
		break;
	case 2:
		reg = AXD_REG_DEC2_SBC_SAMPLE_RATE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_decoder_sbc_audiomode_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_SBC_AUDIO_MODE;
		break;
	case 1:
		reg = AXD_REG_DEC1_SBC_AUDIO_MODE;
		break;
	case 2:
		reg = AXD_REG_DEC2_SBC_AUDIO_MODE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_decoder_sbc_blocks_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_SBC_BLOCKS;
		break;
	case 1:
		reg = AXD_REG_DEC1_SBC_BLOCKS;
		break;
	case 2:
		reg = AXD_REG_DEC2_SBC_BLOCKS;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_decoder_sbc_subbands_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_SBC_SUBBANDS;
		break;
	case 1:
		reg = AXD_REG_DEC1_SBC_SUBBANDS;
		break;
	case 2:
		reg = AXD_REG_DEC2_SBC_SUBBANDS;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_decoder_sbc_bitpool_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_SBC_BITPOOL;
		break;
	case 1:
		reg = AXD_REG_DEC1_SBC_BITPOOL;
		break;
	case 2:
		reg = AXD_REG_DEC2_SBC_BITPOOL;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_decoder_sbc_allocationmode_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_DEC0_SBC_ALLOCATION_MODE;
		break;
	case 1:
		reg = AXD_REG_DEC1_SBC_ALLOCATION_MODE;
		break;
	case 2:
		reg = AXD_REG_DEC2_SBC_ALLOCATION_MODE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}


unsigned int axd_get_decoder_ms11_mode_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	return AXD_REG_MS11_MODE;
}

unsigned int axd_get_decoder_ms11_common_config0_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	return AXD_REG_MS11_COMMON_CONFIG0;
}

unsigned int axd_get_decoder_ms11_common_config1_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	return AXD_REG_MS11_COMMON_CONFIG1;
}

unsigned int axd_get_decoder_ms11_ddt_config0_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	return AXD_REG_MS11_DDT_CONFIG0;
}

unsigned int axd_get_decoder_ms11_ddc_config0_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	return AXD_REG_MS11_DDC_CONFIG0;
}

unsigned int axd_get_decoder_ms11_ext_pcm_config0_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	return AXD_REG_MS11_EXT_PCM_CONFIG0;
}

/* Returns the address of the pcm bitspersample reg for output at @pipe*/
unsigned int axd_get_encoder_pcm_bitspersample_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_PCMOUT0_BITS_PER_SAMPLE;
		break;
	case 1:
		reg = AXD_REG_PCMOUT1_BITS_PER_SAMPLE;
		break;
	case 2:
		reg = AXD_REG_PCMOUT2_BITS_PER_SAMPLE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_encoder_flac_channels_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_FLAC_CHANNELS;
		break;
	case 1:
		reg = AXD_REG_ENC1_FLAC_CHANNELS;
		break;
	case 2:
		reg = AXD_REG_ENC2_FLAC_CHANNELS;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_flac_bitspersample_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_FLAC_BITS_PER_SAMPLE;
		break;
	case 1:
		reg = AXD_REG_ENC1_FLAC_BITS_PER_SAMPLE;
		break;
	case 2:
		reg = AXD_REG_ENC2_FLAC_BITS_PER_SAMPLE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_flac_samplerate_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_FLAC_SAMPLE_RATE;
		break;
	case 1:
		reg = AXD_REG_ENC1_FLAC_SAMPLE_RATE;
		break;
	case 2:
		reg = AXD_REG_ENC2_FLAC_SAMPLE_RATE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_flac_totalsamples_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_FLAC_TOTAL_SAMPLES;
		break;
	case 1:
		reg = AXD_REG_ENC1_FLAC_TOTAL_SAMPLES;
		break;
	case 2:
		reg = AXD_REG_ENC2_FLAC_TOTAL_SAMPLES;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_flac_domidsidestereo_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_FLAC_DO_MID_SIDE_STEREO;
		break;
	case 1:
		reg = AXD_REG_ENC1_FLAC_DO_MID_SIDE_STEREO;
		break;
	case 2:
		reg = AXD_REG_ENC2_FLAC_DO_MID_SIDE_STEREO;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_flac_loosemidsidestereo_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_FLAC_LOOSE_MID_SIDE_STEREO;
		break;
	case 1:
		reg = AXD_REG_ENC1_FLAC_LOOSE_MID_SIDE_STEREO;
		break;
	case 2:
		reg = AXD_REG_ENC2_FLAC_LOOSE_MID_SIDE_STEREO;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_flac_doexhaustivemodelsearch_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_FLAC_DO_EXHAUSTIVE_MODEL_SEARCH;
		break;
	case 1:
		reg = AXD_REG_ENC1_FLAC_DO_EXHAUSTIVE_MODEL_SEARCH;
		break;
	case 2:
		reg = AXD_REG_ENC2_FLAC_DO_EXHAUSTIVE_MODEL_SEARCH;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_flac_minresidualpartitionorder_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_FLAC_MIN_RESIDUAL_PARTITION_ORDER;
		break;
	case 1:
		reg = AXD_REG_ENC1_FLAC_MIN_RESIDUAL_PARTITION_ORDER;
		break;
	case 2:
		reg = AXD_REG_ENC2_FLAC_MIN_RESIDUAL_PARTITION_ORDER;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_flac_maxresidualpartitionorder_reg(
					struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_FLAC_MAX_RESIDUAL_PARTITION_ORDER;
		break;
	case 1:
		reg = AXD_REG_ENC1_FLAC_MAX_RESIDUAL_PARTITION_ORDER;
		break;
	case 2:
		reg = AXD_REG_ENC2_FLAC_MAX_RESIDUAL_PARTITION_ORDER;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_flac_blocksize_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_FLAC_BLOCK_SIZE;
		break;
	case 1:
		reg = AXD_REG_ENC1_FLAC_BLOCK_SIZE;
		break;
	case 2:
		reg = AXD_REG_ENC2_FLAC_BLOCK_SIZE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_flac_bytecount_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_FLAC_BYTE_COUNT;
		break;
	case 1:
		reg = AXD_REG_ENC1_FLAC_BYTE_COUNT;
		break;
	case 2:
		reg = AXD_REG_ENC2_FLAC_BYTE_COUNT;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_flac_samplecount_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_FLAC_SAMPLE_COUNT;
		break;
	case 1:
		reg = AXD_REG_ENC1_FLAC_SAMPLE_COUNT;
		break;
	case 2:
		reg = AXD_REG_ENC2_FLAC_SAMPLE_COUNT;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_flac_framecount_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_FLAC_FRAME_COUNT;
		break;
	case 1:
		reg = AXD_REG_ENC1_FLAC_FRAME_COUNT;
		break;
	case 2:
		reg = AXD_REG_ENC2_FLAC_FRAME_COUNT;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_flac_framebytes_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_FLAC_FRAME_BYTES;
		break;
	case 1:
		reg = AXD_REG_ENC1_FLAC_FRAME_BYTES;
		break;
	case 2:
		reg = AXD_REG_ENC2_FLAC_FRAME_BYTES;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_encoder_alac_channels_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_ALAC_CHANNELS;
		break;
	case 1:
		reg = AXD_REG_ENC1_ALAC_CHANNELS;
		break;
	case 2:
		reg = AXD_REG_ENC2_ALAC_CHANNELS;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_alac_depth_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_ALAC_DEPTH;
		break;
	case 1:
		reg = AXD_REG_ENC1_ALAC_DEPTH;
		break;
	case 2:
		reg = AXD_REG_ENC2_ALAC_DEPTH;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_alac_samplerate_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_ALAC_SAMPLE_RATE;
		break;
	case 1:
		reg = AXD_REG_ENC1_ALAC_SAMPLE_RATE;
		break;
	case 2:
		reg = AXD_REG_ENC2_ALAC_SAMPLE_RATE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_alac_framelength_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_ALAC_FRAME_LENGTH;
		break;
	case 1:
		reg = AXD_REG_ENC1_ALAC_FRAME_LENGTH;
		break;
	case 2:
		reg = AXD_REG_ENC2_ALAC_FRAME_LENGTH;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_alac_maxframebytes_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_ALAC_MAX_FRAME_BYTES;
		break;
	case 1:
		reg = AXD_REG_ENC1_ALAC_MAX_FRAME_BYTES;
		break;
	case 2:
		reg = AXD_REG_ENC2_ALAC_MAX_FRAME_BYTES;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_alac_avgbitrate_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_ALAC_AVG_BIT_RATE;
		break;
	case 1:
		reg = AXD_REG_ENC1_ALAC_AVG_BIT_RATE;
		break;
	case 2:
		reg = AXD_REG_ENC2_ALAC_AVG_BIT_RATE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
unsigned int axd_get_encoder_alac_fastmode_reg(struct axd_cmd *cmd,
							unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_ENC0_ALAC_FAST_MODE;
		break;
	case 1:
		reg = AXD_REG_ENC1_ALAC_FAST_MODE;
		break;
	case 2:
		reg = AXD_REG_ENC2_ALAC_FAST_MODE;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_output_eq_power_reg_ch0_3(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int band)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg = 0;

	if (pipe == 0) {
		switch (band) {
		case 0:
			reg = AXD_REG_EQ_OUT0_POWER_B0_C0_C3;
			break;
		case 1:
			reg = AXD_REG_EQ_OUT0_POWER_B1_C0_C3;
			break;
		case 2:
			reg = AXD_REG_EQ_OUT0_POWER_B2_C0_C3;
			break;
		case 3:
			reg = AXD_REG_EQ_OUT0_POWER_B3_C0_C3;
			break;
		case 4:
			reg = AXD_REG_EQ_OUT0_POWER_B4_C0_C3;
			break;
		default:
			dev_err(axd->dev, WRONG_BAND_STR, band);
			return 0;
		}
	} else if (pipe == 1) {
		switch (band) {
		case 0:
			reg = AXD_REG_EQ_OUT1_POWER_B0_C0_C3;
			break;
		case 1:
			reg = AXD_REG_EQ_OUT1_POWER_B1_C0_C3;
			break;
		case 2:
			reg = AXD_REG_EQ_OUT1_POWER_B2_C0_C3;
			break;
		case 3:
			reg = AXD_REG_EQ_OUT1_POWER_B3_C0_C3;
			break;
		case 4:
			reg = AXD_REG_EQ_OUT1_POWER_B4_C0_C3;
			break;
		default:
			dev_err(axd->dev, WRONG_BAND_STR, band);
			return 0;
		}
	} else if (pipe == 2) {
		switch (band) {
		case 0:
			reg = AXD_REG_EQ_OUT2_POWER_B0_C0_C3;
			break;
		case 1:
			reg = AXD_REG_EQ_OUT2_POWER_B1_C0_C3;
			break;
		case 2:
			reg = AXD_REG_EQ_OUT2_POWER_B2_C0_C3;
			break;
		case 3:
			reg = AXD_REG_EQ_OUT2_POWER_B3_C0_C3;
			break;
		case 4:
			reg = AXD_REG_EQ_OUT2_POWER_B4_C0_C3;
			break;
		default:
			dev_err(axd->dev, WRONG_BAND_STR, band);
			return 0;
		}
	}
	return reg;
}

unsigned int axd_get_output_eq_power_reg_ch4_7(struct axd_cmd *cmd,
		unsigned int pipe, unsigned int band)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg = 0;

	if (pipe == 0) {
		switch (band) {
		case 0:
			reg = AXD_REG_EQ_OUT0_POWER_B0_C4_C7;
			break;
		case 1:
			reg = AXD_REG_EQ_OUT0_POWER_B1_C4_C7;
			break;
		case 2:
			reg = AXD_REG_EQ_OUT0_POWER_B2_C4_C7;
			break;
		case 3:
			reg = AXD_REG_EQ_OUT0_POWER_B3_C4_C7;
			break;
		case 4:
			reg = AXD_REG_EQ_OUT0_POWER_B4_C4_C7;
			break;
		default:
			dev_err(axd->dev, WRONG_BAND_STR, band);
			return 0;
		}
	} else if (pipe == 1) {
		switch (band) {
		case 0:
			reg = AXD_REG_EQ_OUT1_POWER_B0_C4_C7;
			break;
		case 1:
			reg = AXD_REG_EQ_OUT1_POWER_B1_C4_C7;
			break;
		case 2:
			reg = AXD_REG_EQ_OUT1_POWER_B2_C4_C7;
			break;
		case 3:
			reg = AXD_REG_EQ_OUT1_POWER_B3_C4_C7;
			break;
		case 4:
			reg = AXD_REG_EQ_OUT1_POWER_B4_C4_C7;
			break;
		default:
			dev_err(axd->dev, WRONG_BAND_STR, band);
			return 0;
		}
	} else if (pipe == 2) {
		switch (band) {
		case 0:
			reg = AXD_REG_EQ_OUT2_POWER_B0_C4_C7;
			break;
		case 1:
			reg = AXD_REG_EQ_OUT2_POWER_B1_C4_C7;
			break;
		case 2:
			reg = AXD_REG_EQ_OUT2_POWER_B2_C4_C7;
			break;
		case 3:
			reg = AXD_REG_EQ_OUT2_POWER_B3_C4_C7;
			break;
		case 4:
			reg = AXD_REG_EQ_OUT2_POWER_B4_C4_C7;
			break;
		default:
			dev_err(axd->dev, WRONG_BAND_STR, band);
			return 0;
		}
	}
	return reg;
}

unsigned int axd_get_resample_fin_reg(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_RESAMPLER0_FIN;
		break;
	case 1:
		reg = AXD_REG_RESAMPLER1_FIN;
		break;
	case 2:
		reg = AXD_REG_RESAMPLER2_FIN;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}

unsigned int axd_get_resample_fout_reg(struct axd_cmd *cmd, unsigned int pipe)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);
	unsigned int reg;

	switch (pipe) {
	case 0:
		reg = AXD_REG_RESAMPLER0_FOUT;
		break;
	case 1:
		reg = AXD_REG_RESAMPLER1_FOUT;
		break;
	case 2:
		reg = AXD_REG_RESAMPLER2_FOUT;
		break;
	default:
		dev_err(axd->dev, WRONG_PIPE_STR, pipe);
		return 0;
	}
	return reg;
}
