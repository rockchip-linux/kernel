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
 * AXD sysfs Support API for output device
 */
#include <linux/device.h>
#include <linux/kdev_t.h>

#include "axd_cmds.h"
#include "axd_hdr.h"
#include "axd_module.h"
#include "axd_sysfs.h"

#define ENABLE_EQMASTERGAIN	0

/*
 * The following macros operate on DCPP. They help in hiding the verbosity of
 * the function names and make the code more readable and less than 80 char in
 * length.
 */
#define DCPP_CMP(param)							\
	CMP_PARAM(remaining_config, param)

/* DCPP set for a channel parameter */
#define DSET_CHAN(name, param)						\
	do {								\
		data = PARAM_VALUE_ADV(remaining_config, param);	\
		axd_cmd_output_set_dcpp_channel_##name(cmd, pipe,	\
							channel, data);	\
	} while (0)

/*DCPP set for a channel parameter with band in args - hence chand */
#define DSET_CHAND(name, param)						\
	do {								\
		data = PARAM_VALUE_ADV(remaining_config, param);	\
		axd_cmd_output_set_dcpp_channel_##name(cmd, pipe,	\
						channel, band, data);	\
	} while (0)

/*DCPP set for a subband parameter */
#define DSET_SBAND(name, param)						\
	do {								\
		data = PARAM_VALUE_ADV(remaining_config, param);	\
		axd_cmd_output_set_dcpp_subband_##name(cmd, pipe,	\
								data);	\
	} while (0)

/*DCPP set for a subband parameter with band in args - hence 2 */
#define DSET_SBAND2(name, param)					\
	do {								\
		data = PARAM_VALUE_ADV(remaining_config, param);	\
		axd_cmd_output_set_dcpp_subband_##name(cmd, pipe,	\
							band, data);	\
	} while (0)

/* Output Device Sysfs Attributes */
/* Enable/Disable */
static ssize_t show_enabled(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int enabled = axd_cmd_output_get_enabled(cmd, pipe);

	if (enabled)
		return sprintf(buf, "enabled\n");
	return sprintf(buf, "disabled\n");
}
static ssize_t store_enabled(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int enable = buf[0] != '0';

	axd_cmd_output_set_enabled(cmd, pipe, enable);
	return count;
}
static DEVICE_ATTR(enabled, RD_PERMS, show_enabled, store_enabled);

/* Sink Control */
static ssize_t show_sink(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	char sink[32];

	memset(sink, 0, 32);
	axd_cmd_output_get_sink(cmd, pipe, sink);
	return sprintf(buf, "%s", sink);
}
static ssize_t store_sink(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int sink;

	if (!kstrtoint(buf, 0, &sink))
		axd_cmd_output_set_sink(cmd, pipe, sink);
	return count;
}
static DEVICE_ATTR(sink, RD_PERMS | WR_PERMS, show_sink, store_sink);

/* Encoder Codec Control */
static ssize_t show_encoder(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	char encoder[32];

	memset(encoder, 0, 32);
	axd_cmd_output_get_codec(cmd, pipe, encoder);
	return sprintf(buf, "%s", encoder);
}
static ssize_t store_encoder(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int encoder;

	if (!kstrtoint(buf, 0, &encoder))
		axd_cmd_output_set_codec(cmd, pipe, encoder);
	return count;
}
static DEVICE_ATTR(codec, RD_PERMS | WR_PERMS, show_encoder, store_encoder);

/* DownMix Control */
static ssize_t show_downmix(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	char downmix[32];

	axd_cmd_output_get_downmix(cmd, pipe, downmix);
	return sprintf(buf, "%s", downmix);
}
static ssize_t store_downmix(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int downmix;

	if (!kstrtoint(buf, 0, &downmix))
		axd_cmd_output_set_downmix(cmd, pipe, downmix);
	return count;
}
static DEVICE_ATTR(downmix, RD_PERMS | WR_PERMS, show_downmix, store_downmix);

/* EQ Enable/Disable Control */
static ssize_t show_eqenabled(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int enable = axd_cmd_output_get_eqenabled(cmd, pipe);

	return sprintf(buf, "%d\n", !!enable);
}
static ssize_t store_eqenabled(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int enable;

	if (!kstrtoint(buf, 0, &enable))
		axd_cmd_output_set_eqenabled(cmd, pipe, enable);
	return count;
}
static DEVICE_ATTR(eqenabled, RD_PERMS | WR_PERMS, show_eqenabled,
							store_eqenabled);

#if ENABLE_EQMASTERGAIN
/* EQ Master Gain Control */
static ssize_t show_eqmastergain(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int gain;

	axd_cmd_output_get_eqmastergain(cmd, pipe, &gain);
	return sprintf(buf, "gain\t= %d\n", gain);
}
static ssize_t store_eqmastergain(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int gain;

	if (!kstrtoint(buf, 0, &gain))
		axd_cmd_output_set_eqmastergain(cmd, pipe, gain);
	return count;
}
static DEVICE_ATTR(eqmastergain, RD_PERMS | WR_PERMS, show_eqmastergain,
							store_eqmastergain);
#endif

/* EQ Band0 Gain Control */
static ssize_t show_eqband0gain(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int gain;

	axd_cmd_output_get_eqband0gain(cmd, pipe, &gain);
	return sprintf(buf, "gain\t= %d\n", gain);
}
static ssize_t store_eqband0gain(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int gain;

	if (!kstrtoint(buf, 0, &gain))
		axd_cmd_output_set_eqband0gain(cmd, pipe, gain);
	return count;
}
static DEVICE_ATTR(eqband0gain, RD_PERMS | WR_PERMS, show_eqband0gain,
							store_eqband0gain);

/* EQ Band1 Gain Control */
static ssize_t show_eqband1gain(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int gain;

	axd_cmd_output_get_eqband1gain(cmd, pipe, &gain);
	return sprintf(buf, "gain\t= %d\n", gain);
}
static ssize_t store_eqband1gain(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int gain;

	if (!kstrtoint(buf, 0, &gain))
		axd_cmd_output_set_eqband1gain(cmd, pipe, gain);
	return count;
}
static DEVICE_ATTR(eqband1gain, RD_PERMS | WR_PERMS, show_eqband1gain,
							store_eqband1gain);

/* EQ Band2 Gain Control */
static ssize_t show_eqband2gain(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int gain;

	axd_cmd_output_get_eqband2gain(cmd, pipe, &gain);
	return sprintf(buf, "gain\t= %d\n", gain);
}
static ssize_t store_eqband2gain(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int gain;

	if (!kstrtoint(buf, 0, &gain))
		axd_cmd_output_set_eqband2gain(cmd, pipe, gain);
	return count;
}
static DEVICE_ATTR(eqband2gain, RD_PERMS | WR_PERMS, show_eqband2gain,
							store_eqband2gain);

/* EQ Band3 Gain Control */
static ssize_t show_eqband3gain(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int gain;

	axd_cmd_output_get_eqband3gain(cmd, pipe, &gain);
	return sprintf(buf, "gain\t= %d\n", gain);
}
static ssize_t store_eqband3gain(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int gain;

	if (!kstrtoint(buf, 0, &gain))
		axd_cmd_output_set_eqband3gain(cmd, pipe, gain);
	return count;
}
static DEVICE_ATTR(eqband3gain, RD_PERMS | WR_PERMS, show_eqband3gain,
							store_eqband3gain);

/* EQ Band4 Gain Control */
static ssize_t show_eqband4gain(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int gain;

	axd_cmd_output_get_eqband4gain(cmd, pipe, &gain);
	return sprintf(buf, "gain\t= %d\n", gain);
}
static ssize_t store_eqband4gain(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int gain;

	if (!kstrtoint(buf, 0, &gain))
		axd_cmd_output_set_eqband4gain(cmd, pipe, gain);
	return count;
}
static DEVICE_ATTR(eqband4gain, RD_PERMS | WR_PERMS, show_eqband4gain,
							store_eqband4gain);

#define DCPP_ENABLED_PARAM		"enabled"
#define DCPP_CHANNELS_PARAM		"channels"
#define DCPP_MODE_PARAM			"mode"
#define DCPP_EQMODE_PARAM		"eq_mode"
#define DCPP_EQBANDS_PARAM		"eq_bands"
#define DCPP_MAXDELAYSAMPLES_PARAM	"max_delay_samples"
static ssize_t show_dcpp(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);

	ssize_t ret;
	ssize_t n = 0;

	ret = sprintf(buf + n, DCPP_ENABLED_PARAM " = %u\n",
			axd_cmd_output_get_dcpp_enabled(cmd, pipe));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_CHANNELS_PARAM " = %u\n",
			axd_cmd_output_get_dcpp_channels(cmd, pipe));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_MODE_PARAM " = %u\n",
			axd_cmd_output_get_dcpp_mode(cmd, pipe));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_EQMODE_PARAM " = %u\n",
			axd_cmd_output_get_dcpp_eq_mode(cmd, pipe));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_EQBANDS_PARAM " = %u\n",
			axd_cmd_output_get_dcpp_eq_bands(cmd, pipe));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_MAXDELAYSAMPLES_PARAM " = %u\n",
			axd_cmd_output_get_dcpp_max_delay_samples(cmd, pipe));
	if (ret < 0)
		return ret;
	n += ret;

	return n;
}
static ssize_t store_dcpp(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);

	unsigned int data;

	if (CMP_PARAM(buf, DCPP_ENABLED_PARAM)) {
		data = PARAM_VALUE(buf, DCPP_ENABLED_PARAM);
		axd_cmd_output_set_dcpp_enabled(cmd, pipe, data);
	} else if (CMP_PARAM(buf, DCPP_MODE_PARAM)) {
		data = PARAM_VALUE(buf, DCPP_MODE_PARAM);
		axd_cmd_output_set_dcpp_mode(cmd, pipe, data);
	} else if (CMP_PARAM(buf, DCPP_EQMODE_PARAM)) {
		data = PARAM_VALUE(buf, DCPP_EQMODE_PARAM);
		axd_cmd_output_set_dcpp_eq_mode(cmd, pipe, data);
	}

	return count;
}
static DEVICE_ATTR(dcpp, RD_PERMS | WR_PERMS, show_dcpp, store_dcpp);

#define DCPP_SHIFT_SUFFIX			"_shift"
#define DCPP_GAIN_SUFFIX			"_gain"
#define DCPP_A0_SUFFIX				"_a0"
#define DCPP_A1_SUFFIX				"_a1"
#define DCPP_A2_SUFFIX				"_a2"
#define DCPP_B0_SUFFIX				"_b0"
#define DCPP_B1_SUFFIX				"_b1"
#define DCPP_DELAYSAMPLES_PARAM			"delay_samples"
#define DCPP_EQ_OUTPUTVOLUME_PARAM		"eq_output_volume"
#define DCPP_EQ_PASSTHROUGHGAIN_PARAM		"eq_passthrough_gain"
#define DCPP_EQ_INVERSEPASSTHROUGHGAIN_PARAM	"eq_inverse_passthrough_gain"
#define DCPP_EQ_BAND_PREFIX			"eq_band"
#define DCPP_EQ_BAND_GAIN_PARAM			DCPP_EQ_BAND_PREFIX "%d" DCPP_GAIN_SUFFIX
#define DCPP_EQ_BAND_A0_PARAM			DCPP_EQ_BAND_PREFIX "%d" DCPP_A0_SUFFIX
#define DCPP_EQ_BAND_A1_PARAM			DCPP_EQ_BAND_PREFIX "%d" DCPP_A1_SUFFIX
#define DCPP_EQ_BAND_A2_PARAM			DCPP_EQ_BAND_PREFIX "%d" DCPP_A2_SUFFIX
#define DCPP_EQ_BAND_B0_PARAM			DCPP_EQ_BAND_PREFIX "%d" DCPP_B0_SUFFIX
#define DCPP_EQ_BAND_B1_PARAM			DCPP_EQ_BAND_PREFIX "%d" DCPP_B1_SUFFIX
#define DCPP_EQ_BAND_SHIFT_PARAM		DCPP_EQ_BAND_PREFIX "%d" DCPP_SHIFT_SUFFIX
#define DCPP_CH_BASS_SHIFT_PARAM		"bass_shelf" DCPP_SHIFT_SUFFIX
#define DCPP_CH_BASS_A0_PARAM			"bass_shelf" DCPP_A0_SUFFIX
#define DCPP_CH_BASS_A1_PARAM			"bass_shelf" DCPP_A1_SUFFIX
#define DCPP_CH_BASS_A2_PARAM			"bass_shelf" DCPP_A2_SUFFIX
#define DCPP_CH_BASS_B0_PARAM			"bass_shelf" DCPP_B0_SUFFIX
#define DCPP_CH_BASS_B1_PARAM			"bass_shelf" DCPP_B1_SUFFIX
#define DCPP_CH_TREBLE_SHIFT_PARAM		"treble_shelf" DCPP_SHIFT_SUFFIX
#define DCPP_CH_TREBLE_A0_PARAM			"treble_shelf" DCPP_A0_SUFFIX
#define DCPP_CH_TREBLE_A1_PARAM			"treble_shelf" DCPP_A1_SUFFIX
#define DCPP_CH_TREBLE_A2_PARAM			"treble_shelf" DCPP_A2_SUFFIX
#define DCPP_CH_TREBLE_B0_PARAM			"treble_shelf" DCPP_B0_SUFFIX
#define DCPP_CH_TREBLE_B1_PARAM			"treble_shelf" DCPP_B1_SUFFIX
static ssize_t show_dcpp_ch(struct device *dev, struct device_attribute *attr,
						char *buf, unsigned int channel)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);

	int i, ret, bands;
	ssize_t n = 0;

	bands = axd_cmd_output_get_dcpp_eq_bands(cmd, pipe);

	ret = sprintf(buf + n, DCPP_DELAYSAMPLES_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_channel_delay_samples(cmd, pipe,
								channel));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_EQ_OUTPUTVOLUME_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_channel_eq_output_volume(cmd, pipe,
								channel));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_EQ_PASSTHROUGHGAIN_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_channel_eq_passthrough_gain(cmd, pipe,
								channel));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_EQ_INVERSEPASSTHROUGHGAIN_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_channel_eq_inverse_passthrough_gain(cmd,
								pipe, channel));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_CH_BASS_SHIFT_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_channel_bass_shelf_shift(cmd, pipe,
								channel));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_CH_BASS_A0_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_channel_bass_shelf_a0(cmd, pipe,
								channel));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_CH_BASS_A1_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_channel_bass_shelf_a1(cmd, pipe,
								channel));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_CH_BASS_A2_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_channel_bass_shelf_a2(cmd, pipe,
								channel));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_CH_BASS_B0_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_channel_bass_shelf_b0(cmd, pipe,
								channel));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_CH_BASS_B1_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_channel_bass_shelf_b1(cmd, pipe,
								channel));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_CH_TREBLE_SHIFT_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_channel_treble_shelf_shift(cmd, pipe,
								channel));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_CH_TREBLE_A0_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_channel_treble_shelf_a0(cmd, pipe,
								channel));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_CH_TREBLE_A1_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_channel_treble_shelf_a1(cmd, pipe,
								channel));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_CH_TREBLE_A2_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_channel_treble_shelf_a2(cmd, pipe,
								channel));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_CH_TREBLE_B0_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_channel_treble_shelf_b0(cmd, pipe,
								channel));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_CH_TREBLE_B1_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_channel_treble_shelf_b1(cmd, pipe,
								channel));
	if (ret < 0)
		return ret;
	n += ret;

	for (i = 0; i < bands; i++) {

		ret = sprintf(buf + n, DCPP_EQ_BAND_GAIN_PARAM " = %u\n", i,
			axd_cmd_output_get_dcpp_channel_eq_gain(cmd, pipe,
								channel, i));
		if (ret < 0)
			return ret;
		n += ret;

		ret = sprintf(buf + n, DCPP_EQ_BAND_A0_PARAM " = %u\n", i,
			axd_cmd_output_get_dcpp_channel_eq_a0(cmd, pipe,
								channel, i));
		if (ret < 0)
			return ret;
		n += ret;

		ret = sprintf(buf + n, DCPP_EQ_BAND_A1_PARAM " = %u\n", i,
			axd_cmd_output_get_dcpp_channel_eq_a1(cmd, pipe,
								channel, i));
		if (ret < 0)
			return ret;
		n += ret;

		ret = sprintf(buf + n, DCPP_EQ_BAND_A2_PARAM " = %u\n", i,
			axd_cmd_output_get_dcpp_channel_eq_a2(cmd, pipe,
								channel, i));
		if (ret < 0)
			return ret;
		n += ret;

		ret = sprintf(buf + n, DCPP_EQ_BAND_B0_PARAM " = %u\n", i,
			axd_cmd_output_get_dcpp_channel_eq_b0(cmd, pipe,
								channel, i));
		if (ret < 0)
			return ret;
		n += ret;

		ret = sprintf(buf + n, DCPP_EQ_BAND_B1_PARAM " = %u\n", i,
			axd_cmd_output_get_dcpp_channel_eq_b1(cmd, pipe,
								channel, i));
		if (ret < 0)
			return ret;
		n += ret;

		ret = sprintf(buf + n, DCPP_EQ_BAND_SHIFT_PARAM " = %u\n", i,
			axd_cmd_output_get_dcpp_channel_eq_shift(cmd, pipe,
								channel, i));
		if (ret < 0)
			return ret;
		n += ret;
	}

	return n;
}
static ssize_t store_dcpp_ch(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count, unsigned int channel)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);

	unsigned int data;
	char *remaining_config = (char *)&buf[0];

	axd_set_flag(&cmd->ctrlbuf_active_flg, 1);

	while (remaining_config[0] != '\0') {
		if (DCPP_CMP(DCPP_DELAYSAMPLES_PARAM)) {
			DSET_CHAN(delay_samples, DCPP_DELAYSAMPLES_PARAM);
		} else if (DCPP_CMP(DCPP_EQ_OUTPUTVOLUME_PARAM)) {
			DSET_CHAN(eq_output_volume, DCPP_EQ_OUTPUTVOLUME_PARAM);
		} else if (DCPP_CMP(DCPP_EQ_PASSTHROUGHGAIN_PARAM)) {
			DSET_CHAN(eq_passthrough_gain,
						DCPP_EQ_PASSTHROUGHGAIN_PARAM);
		} else if (DCPP_CMP(DCPP_EQ_INVERSEPASSTHROUGHGAIN_PARAM)) {
			DSET_CHAN(eq_inverse_passthrough_gain,
					DCPP_EQ_INVERSEPASSTHROUGHGAIN_PARAM);
		} else if (DCPP_CMP(DCPP_CH_BASS_SHIFT_PARAM)) {
			DSET_CHAN(bass_shelf_shift, DCPP_CH_BASS_SHIFT_PARAM);
		} else if (DCPP_CMP(DCPP_CH_BASS_A0_PARAM)) {
			DSET_CHAN(bass_shelf_a0, DCPP_CH_BASS_A0_PARAM);
		} else if (DCPP_CMP(DCPP_CH_BASS_A1_PARAM)) {
			DSET_CHAN(bass_shelf_a1, DCPP_CH_BASS_A1_PARAM);
		} else if (DCPP_CMP(DCPP_CH_BASS_A2_PARAM)) {
			DSET_CHAN(bass_shelf_a2, DCPP_CH_BASS_A2_PARAM);
		} else if (DCPP_CMP(DCPP_CH_BASS_B0_PARAM)) {
			DSET_CHAN(bass_shelf_b0, DCPP_CH_BASS_B0_PARAM);
		} else if (DCPP_CMP(DCPP_CH_BASS_B1_PARAM)) {
			DSET_CHAN(bass_shelf_b1, DCPP_CH_BASS_B1_PARAM);
		} else if (DCPP_CMP(DCPP_CH_TREBLE_SHIFT_PARAM)) {
			DSET_CHAN(treble_shelf_shift,
						DCPP_CH_TREBLE_SHIFT_PARAM);
		} else if (DCPP_CMP(DCPP_CH_TREBLE_A0_PARAM)) {
			DSET_CHAN(treble_shelf_a0, DCPP_CH_TREBLE_A0_PARAM);
		} else if (DCPP_CMP(DCPP_CH_TREBLE_A1_PARAM)) {
			DSET_CHAN(treble_shelf_a1, DCPP_CH_TREBLE_A1_PARAM);
		} else if (DCPP_CMP(DCPP_CH_TREBLE_A2_PARAM)) {
			DSET_CHAN(treble_shelf_a2, DCPP_CH_TREBLE_A2_PARAM);
		} else if (DCPP_CMP(DCPP_CH_TREBLE_B0_PARAM)) {
			DSET_CHAN(treble_shelf_b0, DCPP_CH_TREBLE_B0_PARAM);
		} else if (DCPP_CMP(DCPP_CH_TREBLE_B1_PARAM)) {
			DSET_CHAN(treble_shelf_b1, DCPP_CH_TREBLE_B1_PARAM);
		} else {
			size_t len = sizeof(DCPP_EQ_BAND_PREFIX)-1;

			if (!strncmp(remaining_config,
						DCPP_EQ_BAND_PREFIX, len)) {
				unsigned int band;

				/* Skip prefix */
				remaining_config += len;
				sscanf(remaining_config, "%u", &band);

				/* Skip band number */
				remaining_config++;
				if (band > 9)
					remaining_config++;

				if (DCPP_CMP(DCPP_GAIN_SUFFIX))
					DSET_CHAND(eq_gain, DCPP_GAIN_SUFFIX);
				else if (DCPP_CMP(DCPP_A0_SUFFIX))
					DSET_CHAND(eq_a0, DCPP_A0_SUFFIX);
				else if (DCPP_CMP(DCPP_A1_SUFFIX))
					DSET_CHAND(eq_a1, DCPP_A1_SUFFIX);
				else if (DCPP_CMP(DCPP_A2_SUFFIX))
					DSET_CHAND(eq_a2, DCPP_A2_SUFFIX);
				else if (DCPP_CMP(DCPP_B0_SUFFIX))
					DSET_CHAND(eq_b0, DCPP_B0_SUFFIX);
				else if (DCPP_CMP(DCPP_B1_SUFFIX))
					DSET_CHAND(eq_b1, DCPP_B1_SUFFIX);
				else if (DCPP_CMP(DCPP_SHIFT_SUFFIX))
					DSET_CHAND(eq_shift, DCPP_SHIFT_SUFFIX);
			}
		}
		/* Advance pointer */
		remaining_config++;
	}

	axd_flush_reg_buf(cmd);

	axd_set_flag(&cmd->ctrlbuf_active_flg, 0);

	return count;
}

static ssize_t show_dcpp_ch0(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_dcpp_ch(dev, attr, buf, 0);
}
static ssize_t store_dcpp_ch0(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	return store_dcpp_ch(dev, attr, buf, count, 0);
}
static DEVICE_ATTR(dcpp_ch0,
			RD_PERMS | WR_PERMS, show_dcpp_ch0, store_dcpp_ch0);

static ssize_t show_dcpp_ch1(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_dcpp_ch(dev, attr, buf, 1);
}
static ssize_t store_dcpp_ch1(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return store_dcpp_ch(dev, attr, buf, count, 1);
}
static DEVICE_ATTR(dcpp_ch1,
			RD_PERMS | WR_PERMS, show_dcpp_ch1, store_dcpp_ch1);

#define DCPP_SUBBAND_INPUTCHANNELMASK_PARAM	"input_channel_mask"
#define DCPP_SUBBAND_LPF_A0_PARAM		"low_pass_filter_a0"
#define DCPP_SUBBAND_LPF_A1_PARAM		"low_pass_filter_a1"
#define DCPP_SUBBAND_LPF_A2_PARAM		"low_pass_filter_a2"
#define DCPP_SUBBAND_LPF_B0_PARAM		"low_pass_filter_b0"
#define DCPP_SUBBAND_LPF_B1_PARAM		"low_pass_filter_b1"
static ssize_t show_dcpp_subband(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);

	ssize_t ret;
	ssize_t n = 0;
	int i;
	int bands = axd_cmd_output_get_dcpp_subband_eq_bands(cmd, pipe);

	ret = sprintf(buf + n, DCPP_ENABLED_PARAM " = %u\n",
			axd_cmd_output_get_dcpp_subband_enabled(cmd, pipe));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_EQBANDS_PARAM " = %u\n", bands);
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_DELAYSAMPLES_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_subband_delay_samples(cmd, pipe));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_SUBBAND_INPUTCHANNELMASK_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_subband_input_channel_mask(cmd, pipe));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_SUBBAND_LPF_A0_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_subband_low_pass_filter_a0(cmd, pipe));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_SUBBAND_LPF_A1_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_subband_low_pass_filter_a1(cmd, pipe));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_SUBBAND_LPF_A2_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_subband_low_pass_filter_a2(cmd, pipe));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_SUBBAND_LPF_B0_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_subband_low_pass_filter_b0(cmd, pipe));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_SUBBAND_LPF_B1_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_subband_low_pass_filter_b1(cmd, pipe));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_EQ_OUTPUTVOLUME_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_subband_eq_output_volume(cmd, pipe));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_EQ_PASSTHROUGHGAIN_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_subband_eq_passthrough_gain(cmd, pipe));
	if (ret < 0)
		return ret;
	n += ret;

	ret = sprintf(buf + n, DCPP_EQ_INVERSEPASSTHROUGHGAIN_PARAM " = %u\n",
		axd_cmd_output_get_dcpp_subband_eq_inverse_passthrough_gain(cmd,
									pipe));
	if (ret < 0)
		return ret;
	n += ret;

	for (i = 0; i < bands; i++) {
		ret = sprintf(buf + n, DCPP_EQ_BAND_GAIN_PARAM " = %u\n", i,
			axd_cmd_output_get_dcpp_subband_eq_gain(cmd, pipe, i));
		if (ret < 0)
			return ret;
		n += ret;

		ret = sprintf(buf + n, DCPP_EQ_BAND_A0_PARAM " = %u\n", i,
			axd_cmd_output_get_dcpp_subband_eq_a0(cmd, pipe, i));
		if (ret < 0)
			return ret;
		n += ret;

		ret = sprintf(buf + n, DCPP_EQ_BAND_A1_PARAM " = %u\n", i,
			axd_cmd_output_get_dcpp_subband_eq_a1(cmd, pipe, i));
		if (ret < 0)
			return ret;
		n += ret;

		ret = sprintf(buf + n, DCPP_EQ_BAND_A2_PARAM " = %u\n", i,
			axd_cmd_output_get_dcpp_subband_eq_a2(cmd, pipe, i));
		if (ret < 0)
			return ret;
		n += ret;

		ret = sprintf(buf + n, DCPP_EQ_BAND_B0_PARAM " = %u\n", i,
			axd_cmd_output_get_dcpp_subband_eq_b0(cmd, pipe, i));
		if (ret < 0)
			return ret;
		n += ret;

		ret = sprintf(buf + n, DCPP_EQ_BAND_B1_PARAM " = %u\n", i,
			axd_cmd_output_get_dcpp_subband_eq_b1(cmd, pipe, i));
		if (ret < 0)
			return ret;
		n += ret;

		ret = sprintf(buf + n, DCPP_EQ_BAND_SHIFT_PARAM " = %u\n", i,
			axd_cmd_output_get_dcpp_subband_eq_shift(cmd, pipe, i));
		if (ret < 0)
			return ret;
		n += ret;
	}

	return n;
}
static ssize_t store_dcpp_subband(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);

	unsigned int data;
	char *remaining_config = (char *)&buf[0];

	axd_set_flag(&cmd->ctrlbuf_active_flg, 1);

	while (remaining_config[0] != '\0') {
		if (DCPP_CMP(DCPP_ENABLED_PARAM)) {
			DSET_SBAND(enabled, DCPP_ENABLED_PARAM);
		} else if (DCPP_CMP(DCPP_DELAYSAMPLES_PARAM)) {
			DSET_SBAND(delay_samples, DCPP_DELAYSAMPLES_PARAM);
		} else if (DCPP_CMP(DCPP_SUBBAND_INPUTCHANNELMASK_PARAM)) {
			DSET_SBAND(input_channel_mask,
					DCPP_SUBBAND_INPUTCHANNELMASK_PARAM);
		} else if (DCPP_CMP(DCPP_SUBBAND_LPF_A0_PARAM)) {
			DSET_SBAND(low_pass_filter_a0,
						DCPP_SUBBAND_LPF_A0_PARAM);
		} else if (DCPP_CMP(DCPP_SUBBAND_LPF_A1_PARAM)) {
			DSET_SBAND(low_pass_filter_a1,
						DCPP_SUBBAND_LPF_A1_PARAM);
		} else if (DCPP_CMP(DCPP_SUBBAND_LPF_A2_PARAM)) {
			DSET_SBAND(low_pass_filter_a2,
						DCPP_SUBBAND_LPF_A2_PARAM);
		} else if (DCPP_CMP(DCPP_SUBBAND_LPF_B0_PARAM)) {
			DSET_SBAND(low_pass_filter_b0,
						DCPP_SUBBAND_LPF_B0_PARAM);
		} else if (DCPP_CMP(DCPP_SUBBAND_LPF_B1_PARAM)) {
			DSET_SBAND(low_pass_filter_b1,
						DCPP_SUBBAND_LPF_B1_PARAM);
		} else if (DCPP_CMP(DCPP_EQ_OUTPUTVOLUME_PARAM)) {
			DSET_SBAND(eq_output_volume,
						DCPP_EQ_OUTPUTVOLUME_PARAM);
		} else if (DCPP_CMP(DCPP_EQ_PASSTHROUGHGAIN_PARAM)) {
			DSET_SBAND(eq_passthrough_gain,
						DCPP_EQ_PASSTHROUGHGAIN_PARAM);
		} else if (DCPP_CMP(DCPP_EQ_INVERSEPASSTHROUGHGAIN_PARAM)) {
			DSET_SBAND(eq_inverse_passthrough_gain,
					DCPP_EQ_INVERSEPASSTHROUGHGAIN_PARAM);
		} else {
			size_t len = sizeof(DCPP_EQ_BAND_PREFIX)-1;

			if (!strncmp(remaining_config,
						DCPP_EQ_BAND_PREFIX, len)) {
				unsigned int band;

				/* Skip prefix */
				remaining_config += len;
				sscanf(remaining_config, "%u", &band);

				/* Skip band number */
				remaining_config++;
				if (band > 9)
					remaining_config++;

				if (DCPP_CMP(DCPP_GAIN_SUFFIX))
					DSET_SBAND2(eq_gain, DCPP_GAIN_SUFFIX);
				else if (DCPP_CMP(DCPP_A0_SUFFIX))
					DSET_SBAND2(eq_a0, DCPP_A0_SUFFIX);
				else if (DCPP_CMP(DCPP_A1_SUFFIX))
					DSET_SBAND2(eq_a1, DCPP_A1_SUFFIX);
				else if (DCPP_CMP(DCPP_A2_SUFFIX))
					DSET_SBAND2(eq_a2, DCPP_A2_SUFFIX);
				else if (DCPP_CMP(DCPP_B0_SUFFIX))
					DSET_SBAND2(eq_b0, DCPP_B0_SUFFIX);
				else if (DCPP_CMP(DCPP_B1_SUFFIX))
					DSET_SBAND2(eq_b1, DCPP_B1_SUFFIX);
				else if (DCPP_CMP(DCPP_SHIFT_SUFFIX))
					DSET_SBAND2(eq_shift,
							DCPP_SHIFT_SUFFIX);
			}
		}
		/* Advance pointer */
		remaining_config++;
	}

	axd_flush_reg_buf(cmd);

	axd_set_flag(&cmd->ctrlbuf_active_flg, 0);

	return count;
}
static DEVICE_ATTR(dcpp_subband,
		RD_PERMS | WR_PERMS, show_dcpp_subband, store_dcpp_subband);

/* Abort blocked read */
static ssize_t store_abort_read(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_INPUT(minor);

	axd_cmd_recv_buffer_abort(cmd, pipe);
	return count;
}
static DEVICE_ATTR(abortRead, WR_PERMS, NULL, store_abort_read);

/* Encoder configuration */
static ssize_t show_encoder_config(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);

	axd_cmd_output_get_encoder_config(cmd, pipe, buf);
	return strlen(buf)+1;
}
static ssize_t store_encoder_config(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);

	axd_cmd_output_set_encoder_config(cmd, pipe, buf);
	return count;
}
static DEVICE_ATTR(encoderConfig, RD_PERMS | WR_PERMS, show_encoder_config,
							store_encoder_config);

/* GEQ power levels */
static ssize_t show_geq_power_c0(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);

	axd_cmd_output_get_geq_power(cmd, pipe, buf, 0);
	return strlen(buf)+1;
}
static DEVICE_ATTR(eqpower_ch0, RD_PERMS , show_geq_power_c0, NULL);


static ssize_t show_geq_power_c1(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);

	axd_cmd_output_get_geq_power(cmd, pipe, buf, 1);
	return strlen(buf)+1;
}
static DEVICE_ATTR(eqpower_ch1, RD_PERMS , show_geq_power_c1, NULL);

static ssize_t show_geq_power_c2(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);

	axd_cmd_output_get_geq_power(cmd, pipe, buf, 2);
	return strlen(buf)+1;
}
static DEVICE_ATTR(eqpower_ch2, RD_PERMS , show_geq_power_c2, NULL);

static ssize_t show_geq_power_c3(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);

	axd_cmd_output_get_geq_power(cmd, pipe, buf, 3);
	return strlen(buf)+1;
}
static DEVICE_ATTR(eqpower_ch3, RD_PERMS , show_geq_power_c3, NULL);

static ssize_t show_geq_power_c4(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);

	axd_cmd_output_get_geq_power(cmd, pipe, buf, 4);
	return strlen(buf)+1;
}
static DEVICE_ATTR(eqpower_ch4, RD_PERMS , show_geq_power_c4, NULL);

static ssize_t show_geq_power_c5(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);

	axd_cmd_output_get_geq_power(cmd, pipe, buf, 5);
	return strlen(buf)+1;
}
static DEVICE_ATTR(eqpower_ch5, RD_PERMS , show_geq_power_c5, NULL);

static ssize_t show_geq_power_c6(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);

	axd_cmd_output_get_geq_power(cmd, pipe, buf, 6);
	return strlen(buf)+1;
}
static DEVICE_ATTR(eqpower_ch6, RD_PERMS , show_geq_power_c6, NULL);

static ssize_t show_geq_power_c7(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);

	axd_cmd_output_get_geq_power(cmd, pipe, buf, 7);
	return strlen(buf)+1;
}
static DEVICE_ATTR(eqpower_ch7, RD_PERMS , show_geq_power_c7, NULL);

static ssize_t show_buffer_size(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	struct axd_pipe *axd_pipe = &cmd->out_pipes[pipe];

	return sprintf(buf, "%dKiB\n", axd_pipe->buf_size / 1024);
}
static ssize_t store_buffer_size(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	struct axd_pipe *axd_pipe = &cmd->out_pipes[pipe];
	unsigned int buf_size = 0;
	unsigned long total_size = axd_cmd_get_dataout_size(cmd);
	unsigned long num_buffers;

	if (!kstrtouint(buf, 0, &buf_size))
		buf_size *= 1024;

	/*
	 * if valid, the change will take effect the next time the user opens
	 * the pipe
	 */
	if (buf_size) {
		num_buffers = total_size / (cmd->num_outputs * buf_size);
		if (num_buffers)
			axd_pipe->buf_size = buf_size;
	}

	return count;
}
static DEVICE_ATTR(buffer_size, RD_PERMS | WR_PERMS, show_buffer_size,
					store_buffer_size);

/* Mux Output Select */
static ssize_t show_mux(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	char mux[32];

	axd_cmd_mixer_get_mux(cmd, pipe, mux);
	return sprintf(buf, "%s", mux);
}
static ssize_t store_mux(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct axd_cmd *cmd = (struct axd_cmd *)dev->platform_data;
	unsigned int minor = MINOR(dev->devt);
	unsigned int pipe = MINOR_TO_OUTPUT(minor);
	int mux;

	if (!kstrtoint(buf, 0, &mux))
		axd_cmd_mixer_set_mux(cmd, pipe, mux);
	return count;
}
static DEVICE_ATTR(mux, RD_PERMS | WR_PERMS, show_mux, store_mux);


int axd_output_sysfs_add(struct device *dev)
{
	int ret;

	ret = device_create_file(dev, &dev_attr_enabled);
	if (ret)
		goto enabled_err;
	ret = device_create_file(dev, &dev_attr_sink);
	if (ret)
		goto sink_err;
	ret = device_create_file(dev, &dev_attr_downmix);
	if (ret)
		goto downmix_err;
	ret = device_create_file(dev, &dev_attr_codec);
	if (ret)
		goto codec_err;
	ret = device_create_file(dev, &dev_attr_eqenabled);
	if (ret)
		goto eqenabled_err;
#if ENABLE_EQMASTERGAIN
	ret = device_create_file(dev, &dev_attr_eqmastergain);
	if (ret)
		goto eqmastergain_err;
#endif
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
	ret = device_create_file(dev, &dev_attr_eqpower_ch0);
	if (ret)
		goto eqpower_ch0_err;
	ret = device_create_file(dev, &dev_attr_eqpower_ch1);
	if (ret)
		goto eqpower_ch1_err;
	ret = device_create_file(dev, &dev_attr_eqpower_ch2);
	if (ret)
		goto eqpower_ch2_err;
	ret = device_create_file(dev, &dev_attr_eqpower_ch3);
	if (ret)
		goto eqpower_ch3_err;
	ret = device_create_file(dev, &dev_attr_eqpower_ch4);
	if (ret)
		goto eqpower_ch4_err;
	ret = device_create_file(dev, &dev_attr_eqpower_ch5);
	if (ret)
		goto eqpower_ch5_err;
	ret = device_create_file(dev, &dev_attr_eqpower_ch6);
	if (ret)
		goto eqpower_ch6_err;
	ret = device_create_file(dev, &dev_attr_eqpower_ch7);
	if (ret)
		goto eqpower_ch7_err;
	ret = device_create_file(dev, &dev_attr_dcpp);
	if (ret)
		goto dcpp_err;
	ret = device_create_file(dev, &dev_attr_dcpp_ch0);
	if (ret)
		goto dcpp_ch0_err;
	ret = device_create_file(dev, &dev_attr_dcpp_ch1);
	if (ret)
		goto dcpp_ch1_err;
	ret = device_create_file(dev, &dev_attr_dcpp_subband);
	if (ret)
		goto dcpp_subband_err;
	ret = device_create_file(dev, &dev_attr_abortRead);
	if (ret)
		goto abort_read_err;
	ret = device_create_file(dev, &dev_attr_encoderConfig);
	if (ret)
		goto encoder_config_err;
	ret = device_create_file(dev, &dev_attr_buffer_size);
	if (ret)
		goto buffer_size_err;
	ret = device_create_file(dev, &dev_attr_mux);
	if (ret)
		goto mux_err;
	return 0;

mux_err:
	device_remove_file(dev, &dev_attr_buffer_size);
buffer_size_err:
	device_remove_file(dev, &dev_attr_encoderConfig);
encoder_config_err:
	device_remove_file(dev, &dev_attr_abortRead);
abort_read_err:
	device_remove_file(dev, &dev_attr_dcpp_subband);
dcpp_subband_err:
	device_remove_file(dev, &dev_attr_dcpp_ch1);
dcpp_ch1_err:
	device_remove_file(dev, &dev_attr_dcpp_ch0);
dcpp_ch0_err:
	device_remove_file(dev, &dev_attr_dcpp);
dcpp_err:
	device_remove_file(dev, &dev_attr_eqpower_ch7);
eqpower_ch7_err:
	device_remove_file(dev, &dev_attr_eqpower_ch6);
eqpower_ch6_err:
	device_remove_file(dev, &dev_attr_eqpower_ch5);
eqpower_ch5_err:
	device_remove_file(dev, &dev_attr_eqpower_ch4);
eqpower_ch4_err:
	device_remove_file(dev, &dev_attr_eqpower_ch3);
eqpower_ch3_err:
	device_remove_file(dev, &dev_attr_eqpower_ch2);
eqpower_ch2_err:
	device_remove_file(dev, &dev_attr_eqpower_ch1);
eqpower_ch1_err:
	device_remove_file(dev, &dev_attr_eqpower_ch0);
eqpower_ch0_err:
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
#if ENABLE_EQMASTERGAIN
	device_remove_file(dev, &dev_attr_eqmastergain);
eqmastergain_err:
#endif
	device_remove_file(dev, &dev_attr_eqenabled);
eqenabled_err:
	device_remove_file(dev, &dev_attr_codec);
codec_err:
	device_remove_file(dev, &dev_attr_downmix);
downmix_err:
	device_remove_file(dev, &dev_attr_sink);
sink_err:
	device_remove_file(dev, &dev_attr_enabled);
enabled_err:
	return ret;
}

void axd_output_sysfs_remove(struct device *dev)
{
	if (!dev)
		return;
	device_remove_file(dev, &dev_attr_mux);
	device_remove_file(dev, &dev_attr_buffer_size);
	device_remove_file(dev, &dev_attr_encoderConfig);
	device_remove_file(dev, &dev_attr_abortRead);
	device_remove_file(dev, &dev_attr_dcpp_subband);
	device_remove_file(dev, &dev_attr_dcpp_ch1);
	device_remove_file(dev, &dev_attr_dcpp_ch0);
	device_remove_file(dev, &dev_attr_dcpp);
	device_remove_file(dev, &dev_attr_eqpower_ch7);
	device_remove_file(dev, &dev_attr_eqpower_ch6);
	device_remove_file(dev, &dev_attr_eqpower_ch5);
	device_remove_file(dev, &dev_attr_eqpower_ch4);
	device_remove_file(dev, &dev_attr_eqpower_ch3);
	device_remove_file(dev, &dev_attr_eqpower_ch2);
	device_remove_file(dev, &dev_attr_eqpower_ch1);
	device_remove_file(dev, &dev_attr_eqpower_ch0);
	device_remove_file(dev, &dev_attr_eqband4gain);
	device_remove_file(dev, &dev_attr_eqband3gain);
	device_remove_file(dev, &dev_attr_eqband2gain);
	device_remove_file(dev, &dev_attr_eqband1gain);
	device_remove_file(dev, &dev_attr_eqband0gain);
#if ENABLE_EQMASTERGAIN
	device_remove_file(dev, &dev_attr_eqmastergain);
#endif
	device_remove_file(dev, &dev_attr_eqenabled);
	device_remove_file(dev, &dev_attr_codec);
	device_remove_file(dev, &dev_attr_downmix);
	device_remove_file(dev, &dev_attr_sink);
	device_remove_file(dev, &dev_attr_enabled);
}
