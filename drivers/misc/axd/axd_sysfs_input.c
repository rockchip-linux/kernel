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
 * AXD sysfs Support API for input device
 */
#include <linux/device.h>
#include <linux/kdev_t.h>

#include "axd_cmds.h"
#include "axd_hdr.h"
#include "axd_module.h"
#include "axd_sysfs.h"

/* Input Device Sysfs Attributes */
/* Enable/Disable */
static ssize_t show_enabled(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	int enabled = axd_cmd_input_get_enabled(cmd, pipe);

	if (enabled)
		return sprintf(buf, "enabled\n");
	return sprintf(buf, "disabled\n");
}
static ssize_t store_enabled(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	int enable = buf[0] != '0';

	axd_cmd_input_set_enabled(cmd, pipe, enable);
	return count;
}
static DEVICE_ATTR(enabled, RD_PERMS, show_enabled, store_enabled);

/* Source Control */
static ssize_t show_source(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	char source[32];

	memset(source, 0, 32);
	axd_cmd_input_get_source(cmd, pipe, source);
	return sprintf(buf, "%s", source);
}
static ssize_t store_source(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	int source;

	if (!kstrtoint(buf, 0, &source))
		axd_cmd_input_set_source(cmd, pipe, source);
	return count;
}
static DEVICE_ATTR(source, RD_PERMS | WR_PERMS, show_source, store_source);

/* Codec Control */
static ssize_t show_codec(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	char codec[32];

	memset(codec, 0, 32);
	axd_cmd_input_get_codec(cmd, pipe, codec);
	return sprintf(buf, "%s", codec);
}
static ssize_t store_codec(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	int codec;

	if (!kstrtoint(buf, 0, &codec))
		axd_cmd_input_set_codec(cmd, pipe, codec);
	return count;
}
static DEVICE_ATTR(codec, RD_PERMS | WR_PERMS, show_codec, store_codec);

/* Gain Control */
static ssize_t show_gain(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	int gain;

	axd_cmd_input_get_gain(cmd, pipe, &gain);
	return sprintf(buf, "gain\t= %d\n", gain);
}
static ssize_t store_gain(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	int gain;

	if (!kstrtoint(buf, 0, &gain))
		axd_cmd_input_set_gain(cmd, pipe, gain);
	return count;
}
static DEVICE_ATTR(gain, RD_PERMS | WR_PERMS, show_gain, store_gain);

/* Mute Control */
static ssize_t show_mute(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	int muted;

	axd_cmd_input_get_mute(cmd, pipe, &muted);
	return sprintf(buf, "%d\n", !!muted);
}
static ssize_t store_mute(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	int mute;

	if (!kstrtoint(buf, 0, &mute))
		axd_cmd_input_set_mute(cmd, pipe, mute);
	return count;
}
static DEVICE_ATTR(mute, RD_PERMS | WR_PERMS, show_mute, store_mute);

/* UpMix Control */
static ssize_t show_upmix(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	char upmix[32];

	axd_cmd_input_get_upmix(cmd, pipe, upmix);
	return sprintf(buf, "%s", upmix);
}
static ssize_t store_upmix(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	int upmix;

	if (!kstrtoint(buf, 0, &upmix))
		axd_cmd_input_set_upmix(cmd, pipe, upmix);
	return count;
}
static DEVICE_ATTR(upmix, RD_PERMS | WR_PERMS, show_upmix, store_upmix);

/* Decoder Config */
static ssize_t show_decoder_config(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);

	axd_cmd_input_get_decoder_config(cmd, pipe, buf);
	return strlen(buf)+1;
}
static ssize_t store_decoder_config(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);

	axd_cmd_input_set_decoder_config(cmd, pipe, buf);
	return count;
}
static DEVICE_ATTR(decoderConfig, RD_PERMS | WR_PERMS, show_decoder_config,
							store_decoder_config);

/* Abort blocked write */
static ssize_t store_abort_write(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);

	axd_cmd_send_buffer_abort(cmd, pipe);
	return count;
}
static DEVICE_ATTR(abortWrite, WR_PERMS, NULL, store_abort_write);

/*Resampler Frequencies*/
static ssize_t show_resampler_fin(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	u32 fin = axd_cmd_info_get_resampler_fin(cmd, pipe);

	return sprintf(buf, "%d\n", fin);
}
static DEVICE_ATTR(resampler_fin, RD_PERMS, show_resampler_fin, NULL);

static ssize_t show_resampler_fout(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	u32 fout = axd_cmd_info_get_resampler_fout(cmd, pipe);

	return sprintf(buf, "%d\n", fout);
}
static ssize_t store_resampler_fout(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	unsigned int fout;

	if (!kstrtouint(buf, 0, &fout))
		axd_cmd_info_set_resampler_fout(cmd, pipe, fout);
	return count;
}
static DEVICE_ATTR(resampler_fout, RD_PERMS | WR_PERMS, show_resampler_fout,
					store_resampler_fout);

static ssize_t show_buffer_size(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);

	struct axd_pipe *axd_pipe = &cmd->in_pipes[pipe];

	return sprintf(buf, "%dKiB\n", axd_pipe->buf_size / 1024);
}
static ssize_t store_buffer_size(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	struct axd_pipe *axd_pipe = &cmd->in_pipes[pipe];
	unsigned int buf_size = 0;
	unsigned long total_size = axd_cmd_get_datain_size(cmd);
	unsigned long num_buffers;

	if (!kstrtouint(buf, 0, &buf_size))
		buf_size *= 1024;

	/*
	 * if valid, the change will take effect the next time the user opens
	 * the pipe
	 */
	if (buf_size) {
		num_buffers = total_size / (cmd->num_inputs * buf_size);
		if (num_buffers)
			axd_pipe->buf_size = buf_size;
	}

	return count;
}
static DEVICE_ATTR(buffer_size, RD_PERMS | WR_PERMS, show_buffer_size,
					store_buffer_size);

static ssize_t show_buffer_occupancy(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	unsigned int bo = axd_cmd_input_get_buffer_occupancy(cmd, pipe);

	return sprintf(buf, "%u%%\n", bo);
}
static ssize_t store_buffer_occupancy(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);
	unsigned int bo;

	if (!kstrtouint(buf, 0, &bo) && bo >= 0 && bo <= 100) {
		/* Valid */
		axd_cmd_input_set_buffer_occupancy(cmd, pipe, bo);
	}
	return count;
}
static DEVICE_ATTR(buffer_occupancy, RD_PERMS | WR_PERMS, show_buffer_occupancy,
					store_buffer_occupancy);

int axd_input_sysfs_add(struct device *dev)
{
	int ret;

	ret = device_create_file(dev, &dev_attr_enabled);
	if (ret)
		goto enabled_err;
	ret = device_create_file(dev, &dev_attr_source);
	if (ret)
		goto source_err;
	ret = device_create_file(dev, &dev_attr_codec);
	if (ret)
		goto codec_err;
	ret = device_create_file(dev, &dev_attr_gain);
	if (ret)
		goto gain_err;
	ret = device_create_file(dev, &dev_attr_mute);
	if (ret)
		goto mute_err;
	ret = device_create_file(dev, &dev_attr_upmix);
	if (ret)
		goto upmix_err;
	ret = device_create_file(dev, &dev_attr_decoderConfig);
	if (ret)
		goto decoder_config_err;
	ret = device_create_file(dev, &dev_attr_abortWrite);
	if (ret)
		goto abort_write_err;
	ret = device_create_file(dev, &dev_attr_resampler_fin);
	if (ret)
		goto resampler_fin_err;
	ret = device_create_file(dev, &dev_attr_resampler_fout);
	if (ret)
		goto resampler_fout_err;
	ret = device_create_file(dev, &dev_attr_buffer_size);
	if (ret)
		goto buffer_size_err;
	ret = device_create_file(dev, &dev_attr_buffer_occupancy);
	if (ret)
		goto buffer_occupancy_err;


	return 0;

buffer_occupancy_err:
	device_remove_file(dev, &dev_attr_buffer_size);
buffer_size_err:
	device_remove_file(dev, &dev_attr_resampler_fout);
resampler_fout_err:
	device_remove_file(dev, &dev_attr_resampler_fin);
resampler_fin_err:
	device_remove_file(dev, &dev_attr_abortWrite);
abort_write_err:
	device_remove_file(dev, &dev_attr_decoderConfig);
decoder_config_err:
	device_remove_file(dev, &dev_attr_upmix);
upmix_err:
	device_remove_file(dev, &dev_attr_mute);
mute_err:
	device_remove_file(dev, &dev_attr_gain);
gain_err:
	device_remove_file(dev, &dev_attr_codec);
codec_err:
	device_remove_file(dev, &dev_attr_source);
source_err:
	device_remove_file(dev, &dev_attr_enabled);
enabled_err:
	return ret;
}

void axd_input_sysfs_remove(struct device *dev)
{
	if (!dev)
		return;
	device_remove_file(dev, &dev_attr_buffer_occupancy);
	device_remove_file(dev, &dev_attr_buffer_size);
	device_remove_file(dev, &dev_attr_resampler_fout);
	device_remove_file(dev, &dev_attr_resampler_fin);
	device_remove_file(dev, &dev_attr_abortWrite);
	device_remove_file(dev, &dev_attr_decoderConfig);
	device_remove_file(dev, &dev_attr_upmix);
	device_remove_file(dev, &dev_attr_mute);
	device_remove_file(dev, &dev_attr_gain);
	device_remove_file(dev, &dev_attr_codec);
	device_remove_file(dev, &dev_attr_source);
	device_remove_file(dev, &dev_attr_enabled);
}
