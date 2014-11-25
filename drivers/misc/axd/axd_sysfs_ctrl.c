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
 * AXD sysfs Support API for ctrl device
 */
#include <linux/device.h>
#include <linux/kdev_t.h>

#include "axd_cmds.h"
#include "axd_cmds_internal.h"
#include "axd_module.h"
#include "axd_sysfs.h"

#define ENABLE_EQCONTROLS	0

/* Control Device Sysfs Attributes */
/* version */
static ssize_t show_version(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int major, minor, patch;

	axd_cmd_get_version(cmd, &major, &minor, &patch);
	return sprintf(buf, "%u.%u.%u\n", major, minor, patch);
}
static DEVICE_ATTR(version, RD_PERMS, show_version, NULL);

/* num of supported pipes */
static ssize_t show_pipes(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int inpipes, outpipes;
	int ret = axd_cmd_get_num_pipes(cmd, &inpipes, &outpipes);

	if (ret == -1) {
		/* No pipes available if error when getting value */
		inpipes = 0;
		outpipes = 0;
	}
	return sprintf(buf, "Number of supported input/output pipes:\n"
		"inputs\t= %u\noutputs\t= %u\n", inpipes, outpipes);
}
static DEVICE_ATTR(pipes, RD_PERMS, show_pipes, NULL);

/* Decoders supported */
static ssize_t show_decoders(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	char decoders[128];

	memset(decoders, 0, 128);
	axd_cmd_get_decoders(cmd, decoders);
	return sprintf(buf, "Supported decoders:\n%s", decoders);
}
static DEVICE_ATTR(decoders, RD_PERMS, show_decoders, NULL);

/* Encoders supported */
static ssize_t show_encoders(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	char encoders[128];

	memset(encoders, 0, 128);
	axd_cmd_get_encoders(cmd, encoders);
	return sprintf(buf, "Supported encoders:\n%s", encoders);
}
static DEVICE_ATTR(encoders, RD_PERMS, show_encoders, NULL);

/* Mix/Xbar present */
static ssize_t show_mix(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int present = axd_cmd_xbar_present(cmd);

	if (present)
		return sprintf(buf, "Mix/Xbar present\n");
	return sprintf(buf,
		"No Mix/Xbar, 1:1 mapping between input and output pipes\n");
}
static DEVICE_ATTR(mix, RD_PERMS, show_mix, NULL);

#if ENABLE_EQCONTROLS
/* Mixer EQ Enable/Disable Control */
static ssize_t show_eqenabled(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int enable = axd_cmd_mixer_get_eqenabled(cmd, 0);

	return sprintf(buf, "%d\n", !!enable);
}
static ssize_t store_eqenabled(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int enable;

	if (!kstrtoint(buf, 0, &enable))
		axd_cmd_mixer_set_eqenabled(cmd, 0, enable);
	return count;
}
static DEVICE_ATTR(eqenabled, RD_PERMS | WR_PERMS, show_eqenabled,
							store_eqenabled);

/* Mixer EQ Master Gain Control */
static ssize_t show_eqmastergain(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int gain;

	axd_cmd_mixer_get_eqmastergain(cmd, 0, &gain);
	return sprintf(buf, "gain\t= %d\n", gain);
}
static ssize_t store_eqmastergain(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int gain;

	if (!kstrtoint(buf, 0, &gain))
		axd_cmd_mixer_set_eqmastergain(cmd, 0, gain);
	return count;
}
static DEVICE_ATTR(eqmastergain, RD_PERMS | WR_PERMS, show_eqmastergain,
							store_eqmastergain);

/* Mixer EQ Band0 Gain Control */
static ssize_t show_eqband0gain(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int gain;

	axd_cmd_mixer_get_eqband0gain(cmd, 0, &gain);
	return sprintf(buf, "gain\t= %d\n", gain);
}
static ssize_t store_eqband0gain(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int gain;

	if (!kstrtoint(buf, 0, &gain))
		axd_cmd_mixer_set_eqband0gain(cmd, 0, gain);
	return count;
}
static DEVICE_ATTR(eqband0gain, RD_PERMS | WR_PERMS, show_eqband0gain,
							store_eqband0gain);

/* Mixer EQ Band1 Gain Control */
static ssize_t show_eqband1gain(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int gain;

	axd_cmd_mixer_get_eqband1gain(cmd, 0, &gain);
	return sprintf(buf, "gain\t= %d\n", gain);
}
static ssize_t store_eqband1gain(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int gain;

	if (!kstrtoint(buf, 0, &gain))
		axd_cmd_mixer_set_eqband1gain(cmd, 0, gain);
	return count;
}
static DEVICE_ATTR(eqband1gain, RD_PERMS | WR_PERMS, show_eqband1gain,
							store_eqband1gain);

/* Mixer EQ Band2 Gain Control */
static ssize_t show_eqband2gain(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int gain;

	axd_cmd_mixer_get_eqband2gain(cmd, 0, &gain);
	return sprintf(buf, "gain\t= %d\n", gain);
}
static ssize_t store_eqband2gain(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int gain;

	if (!kstrtoint(buf, 0, &gain))
		axd_cmd_mixer_set_eqband2gain(cmd, 0, gain);
	return count;
}
static DEVICE_ATTR(eqband2gain, RD_PERMS | WR_PERMS, show_eqband2gain,
							store_eqband2gain);

/* Mixer EQ Band3 Gain Control */
static ssize_t show_eqband3gain(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int gain;

	axd_cmd_mixer_get_eqband3gain(cmd, 0, &gain);
	return sprintf(buf, "gain\t= %d\n", gain);
}
static ssize_t store_eqband3gain(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int gain;

	if (!kstrtoint(buf, 0, &gain))
		axd_cmd_mixer_set_eqband3gain(cmd, 0, gain);
	return count;
}
static DEVICE_ATTR(eqband3gain, RD_PERMS | WR_PERMS, show_eqband3gain,
							store_eqband3gain);

/* Mixer EQ Band4 Gain Control */
static ssize_t show_eqband4gain(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int gain;

	axd_cmd_mixer_get_eqband4gain(cmd, 0, &gain);
	return sprintf(buf, "gain\t= %d\n", gain);
}
static ssize_t store_eqband4gain(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int gain;

	if (!kstrtoint(buf, 0, &gain))
		axd_cmd_mixer_set_eqband4gain(cmd, 0, gain);
	return count;
}
static DEVICE_ATTR(eqband4gain, RD_PERMS | WR_PERMS, show_eqband4gain,
							store_eqband4gain);
#endif

/* Enable/Disable watchdog timer */
static ssize_t show_watchdog(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;

	if (cmd->watchdogenabled)
		return sprintf(buf, "enabled\n");
	return sprintf(buf, "disabled\n");
}
static ssize_t store_watchdog(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int wd_enabled;

	if (!kstrtoint(buf, 0, &wd_enabled))
		cmd->watchdogenabled = wd_enabled;
	return count;
}
static DEVICE_ATTR(watchdog,
			RD_PERMS | WR_PERMS, show_watchdog, store_watchdog);

/*
 * Enable/Disable discarding output buffers on close without EOS reached.
 *
 * By default this is enabled, which means that all pending buffers will be
 * discarded if an output device is closed before EOS is reached.
 * When disabled, opening the output device after closing it before EOS is
 * reached will resume from where it stopped.
 */
static ssize_t show_discard(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;

	if (axd_get_flag(&cmd->discard_flg))
		return sprintf(buf, "enabled\n");
	return sprintf(buf, "disabled\n");
}
static ssize_t store_discard(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int discard_flg;

	if (!kstrtouint(buf, 0, &discard_flg))
		axd_set_flag(&cmd->discard_flg, discard_flg);
	return count;
}
static DEVICE_ATTR(discard, RD_PERMS | WR_PERMS, show_discard, store_discard);

/*
 * Reset Pipe command, stops and flushes the stream, needed in scenarios
 * where the stream doesn't get automatically closed, for example the mixed
 * output, or if an input gets opened and then closed without its output being
 * opened, so the end flag on close never gets seen and the pipe remains in an
 * active state.
 */

static ssize_t store_reset_pipe(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int pipe;

	if (!kstrtouint(buf, 0, &pipe))
		axd_cmd_outpipe_reset(cmd, pipe);
	return count;
}
static DEVICE_ATTR(reset_pipe, WR_PERMS, NULL, store_reset_pipe);

/*
 * Set axd debug mask which can be helpful for reporting firmware problems
 */
static ssize_t show_debug_mask(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int mask = 0;

	axd_read_reg(cmd, AXD_REG_DEBUG_MASK, &mask);
	return sprintf(buf, "0x%08x\n", mask);
}
static ssize_t store_debug_mask(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int mask;

	if (!kstrtouint(buf, 0, &mask))
		axd_write_reg(cmd, AXD_REG_DEBUG_MASK, mask);
	return count;
}
static DEVICE_ATTR(debug_mask,
			RD_PERMS | WR_PERMS, show_debug_mask, store_debug_mask);

static ssize_t show_sync_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int sync_mode = 0;

	axd_read_reg(cmd, AXD_REG_SYNC_MODE, &sync_mode);
	switch (sync_mode) {
	case 0:
		return sprintf(buf, "open_loop (%u)\n", sync_mode);
	case 1:
		return sprintf(buf, "fixed_frequency (%u)\n", sync_mode);
	case 2:
		return sprintf(buf, "network_ts (%u)\n", sync_mode);
	default:
		return sprintf(buf, "unknown (%u)\n", sync_mode);
	}
}
static ssize_t store_sync_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int sync_mode;

	if (!kstrtouint(buf, 0, &sync_mode))
		axd_write_reg(cmd, AXD_REG_SYNC_MODE, sync_mode);
	return count;
}
static DEVICE_ATTR(sync_mode,
			RD_PERMS | WR_PERMS, show_sync_mode, store_sync_mode);

static ssize_t show_timestamps_out(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);

	if (axd_get_flag(&axd->timestamps_out_flg))
		return sprintf(buf, "enabled\n");
	return sprintf(buf, "disabled\n");
}
static ssize_t store_timestamps_out(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	int enabled;

	if (!kstrtoint(buf, 0, &enabled)) {
		struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);

		axd_set_flag(&axd->timestamps_out_flg, enabled);
	}
	return count;
}
static DEVICE_ATTR(timestamps_out,
		RD_PERMS | WR_PERMS, show_timestamps_out, store_timestamps_out);

int axd_ctrl_sysfs_add(struct device *dev)
{
#if ENABLE_EQCONTROLS
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
#endif
	int ret;

	ret = device_create_file(dev, &dev_attr_version);
	if (ret)
		goto version_err;
	ret = device_create_file(dev, &dev_attr_pipes);
	if (ret)
		goto pipes_err;
	ret = device_create_file(dev, &dev_attr_decoders);
	if (ret)
		goto decoders_err;
	ret = device_create_file(dev, &dev_attr_encoders);
	if (ret)
		goto encoders_err;
	ret = device_create_file(dev, &dev_attr_mix);
	if (ret)
		goto mix_err;
#if ENABLE_EQCONTROLS
	if (axd_cmd_xbar_present(cmd)) {
		ret = device_create_file(dev, &dev_attr_eqenabled);
		if (ret)
			goto eqenabled_err;
		ret = device_create_file(dev, &dev_attr_eqmastergain);
		if (ret)
			goto eqmastergain_err;
		ret = device_create_file(dev, &dev_attr_eqband0gain);
		if (ret)
			goto eqband0gain_err;
		ret = device_create_file(dev, &dev_attr_eqband1gain);
		if (ret)
			goto eqband1gain_err;
		ret = device_create_file(dev, &dev_attr_eqband2gain);
		if (ret)
			goto eqband2gain_err;
		ret = device_create_file(dev, &dev_attr_eqband3gain);
		if (ret)
			goto eqband3gain_err;
		ret = device_create_file(dev, &dev_attr_eqband4gain);
		if (ret)
			goto eqband4gain_err;
	}
#endif
	ret = device_create_file(dev, &dev_attr_watchdog);
	if (ret)
		goto watchdog_err;
	ret = device_create_file(dev, &dev_attr_discard);
	if (ret)
		goto discard_err;
	ret = device_create_file(dev, &dev_attr_reset_pipe);
	if (ret)
		goto reset_pipe_err;
	ret = device_create_file(dev, &dev_attr_debug_mask);
	if (ret)
		goto debug_mask_err;
	ret = device_create_file(dev, &dev_attr_sync_mode);
	if (ret)
		goto sync_mode_err;
	ret = device_create_file(dev, &dev_attr_timestamps_out);
	if (ret)
		goto timestamps_out_err;
	return 0;

timestamps_out_err:
	device_remove_file(dev, &dev_attr_sync_mode);
sync_mode_err:
	device_remove_file(dev, &dev_attr_debug_mask);
debug_mask_err:
	device_remove_file(dev, &dev_attr_reset_pipe);
reset_pipe_err:
	device_remove_file(dev, &dev_attr_discard);
discard_err:
	device_remove_file(dev, &dev_attr_watchdog);
watchdog_err:
#if ENABLE_EQCONTROLS
	device_remove_file(dev, &dev_attr_eqband4gain);
eqband4gain_err:
	device_remove_file(dev, &dev_attr_eqband3gain);
eqband3gain_err:
	device_remove_file(dev, &dev_attr_eqband2gain);
eqband2gain_err:
	device_remove_file(dev, &dev_attr_eqband1gain);
eqband1gain_err:
	device_remove_file(dev, &dev_attr_eqband0gain);
eqband0gain_err:
	device_remove_file(dev, &dev_attr_eqmastergain);
eqmastergain_err:
	device_remove_file(dev, &dev_attr_eqenabled);
eqenabled_err:
#endif
	device_remove_file(dev, &dev_attr_mix);
mix_err:
	device_remove_file(dev, &dev_attr_encoders);
encoders_err:
	device_remove_file(dev, &dev_attr_decoders);
decoders_err:
	device_remove_file(dev, &dev_attr_pipes);
pipes_err:
	device_remove_file(dev, &dev_attr_version);
version_err:
	return ret;
}

void axd_ctrl_sysfs_remove(struct device *dev)
{
	if (!dev)
		return;
	device_remove_file(dev, &dev_attr_timestamps_out);
	device_remove_file(dev, &dev_attr_sync_mode);
	device_remove_file(dev, &dev_attr_debug_mask);
	device_remove_file(dev, &dev_attr_reset_pipe);
	device_remove_file(dev, &dev_attr_discard);
	device_remove_file(dev, &dev_attr_watchdog);
#if ENABLE_EQCONTROLS
	device_remove_file(dev, &dev_attr_eqband4gain);
	device_remove_file(dev, &dev_attr_eqband3gain);
	device_remove_file(dev, &dev_attr_eqband2gain);
	device_remove_file(dev, &dev_attr_eqband1gain);
	device_remove_file(dev, &dev_attr_eqband0gain);
	device_remove_file(dev, &dev_attr_eqmastergain);
	device_remove_file(dev, &dev_attr_eqenabled);
#endif
	device_remove_file(dev, &dev_attr_mix);
	device_remove_file(dev, &dev_attr_encoders);
	device_remove_file(dev, &dev_attr_decoders);
	device_remove_file(dev, &dev_attr_pipes);
	device_remove_file(dev, &dev_attr_version);
}
