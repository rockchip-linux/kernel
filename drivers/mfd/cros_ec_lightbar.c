/*
 * cros_ec_lightbar - expose the Chromebook Pixel lightbar to userspace
 *
 * Copyright (C) 2014 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#define pr_fmt(fmt) "cros_ec_lightbar: " fmt

#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/mfd/cros_ec_dev.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "cros_ec_dev.h"

/* Rate-limit the lightbar interface to prevent DoS. */
static unsigned long lb_interval_jiffies = 50 * HZ / 1000;

static ssize_t show_interval_msec(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	unsigned long msec = lb_interval_jiffies * 1000 / HZ;
	return scnprintf(buf, PAGE_SIZE, "%lu\n", msec);
}

static ssize_t store_interval_msec(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret;
	unsigned long msec;

	ret = sscanf(buf, "%lu", &msec);
	if (ret != 1)
		return -EINVAL;

	lb_interval_jiffies = msec * HZ / 1000;

	return count;
}

static DEFINE_MUTEX(lb_mutex);
/* Return 0 if able to throttle correctly, error otherwise */
static int lb_throttle(void)
{
	static unsigned long last_access;
	unsigned long now, next_timeslot;
	long delay;
	int ret = 0;

	mutex_lock(&lb_mutex);

	now = jiffies;
	next_timeslot = last_access + lb_interval_jiffies;

	if (time_before(now, next_timeslot)) {
		delay = (long)(next_timeslot) - (long)now;
		set_current_state(TASK_INTERRUPTIBLE);
		if (schedule_timeout(delay) > 0) {
			/* interrupted - just abort */
			ret = -EINTR;
			goto out;
		}
		now = jiffies;
	}

	last_access = now;
out:
	mutex_unlock(&lb_mutex);

	return ret;
}

#define INIT_MSG(ec, P, R) { \
		.command = EC_CMD_LIGHTBAR_CMD + (ec)->cmd_offset, \
		.outdata = (uint8_t *)&P, \
		.outsize = sizeof(P), \
		.indata = (uint8_t *)&R, \
		.insize = sizeof(R), \
	}

static int get_lightbar_version(struct cros_ec_dev *ec,
				uint32_t *ver_ptr, uint32_t *flg_ptr)
{
	struct ec_params_lightbar param;
	struct ec_response_lightbar resp;
	struct cros_ec_command msg = INIT_MSG(ec, param, resp);
	int ret;

	param.cmd = LIGHTBAR_CMD_VERSION;
	ret = cros_ec_cmd_xfer(ec->ec_dev, &msg);
	if (ret < 0)
		return 0;

	switch (msg.result) {
	case EC_RES_INVALID_PARAM:
		/* Pixel had no version command. */
		if (ver_ptr)
			*ver_ptr = 0;
		if (flg_ptr)
			*flg_ptr = 0;
		return 1;

	case EC_RES_SUCCESS:
		/* Future devices w/lightbars should implement this command */
		if (ver_ptr)
			*ver_ptr = resp.version.num;
		if (flg_ptr)
			*flg_ptr = resp.version.flags;
		return 1;
	}

	/* Anything else (ie, EC_RES_INVALID_COMMAND) - no lightbar */
	return 0;
}

static ssize_t show_version(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	uint32_t version, flags;
	struct cros_ec_dev *ec = container_of(
			dev, struct cros_ec_dev, class_dev);
	int ret;

	ret = lb_throttle();
	if (ret)
		return ret;

	/* This should always succeed, because we check during init. */
	if (!get_lightbar_version(ec, &version, &flags))
		return -EIO;

	return scnprintf(buf, PAGE_SIZE, "%d %d\n", version, flags);
}

static ssize_t store_brightness(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ec_params_lightbar param;
	struct ec_response_lightbar resp;
	struct cros_ec_dev *ec = container_of(
			dev, struct cros_ec_dev, class_dev);
	struct cros_ec_command msg = INIT_MSG(ec, param, resp);
	int ret;
	unsigned int val;

	ret = sscanf(buf, "%i", &val);
	if (ret != 1)
		return -EINVAL;
	param.cmd = LIGHTBAR_CMD_SET_BRIGHTNESS;
	param.set_brightness.num = val;
	ret = lb_throttle();
	if (ret)
		return ret;
	ret = cros_ec_cmd_xfer(ec->ec_dev, &msg);
	if (ret < 0)
		return ret;
	if (msg.result != EC_RES_SUCCESS)
		return -EINVAL;

	return count;
}


/*
 * We expect numbers, and we'll keep reading until we find them, skipping over
 * any whitespace (sysfs guarantees that the input is null-terminated). Every
 * four numbers are sent to the lightbar as <LED,R,G,B>. We fail at the first
 * parsing error, if we don't parse any numbers, or if we have numbers left
 * over.
 */
static ssize_t store_rgb(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct ec_params_lightbar param;
	struct ec_response_lightbar resp;
	struct cros_ec_dev *ec = container_of(
			dev, struct cros_ec_dev, class_dev);
	struct cros_ec_command msg = INIT_MSG(ec, param, resp);
	unsigned int val[4];
	int ret, i = 0, j = 0, ok = 0;

	do {
		/* Skip any whitespace */
		while (*buf && isspace(*buf))
			buf++;
		if (!*buf)
			break;

		ret = sscanf(buf, "%i", &val[i++]);
		if (ret == 0)
			return -EINVAL;

		if (i == 4) {
			param.cmd = LIGHTBAR_CMD_SET_RGB;
			param.set_rgb.led = val[0];
			param.set_rgb.red = val[1];
			param.set_rgb.green = val[2];
			param.set_rgb.blue = val[3];
			/*
			 * Throttle only the first of every four transactions,
			 * so that the user can update all four LEDs at once.
			 */
			if ((j++ % 4) == 0) {
				ret = lb_throttle();
				if (ret)
					return ret;
			}
			ret = cros_ec_cmd_xfer(ec->ec_dev, &msg);
			if (ret < 0)
				return ret;
			if (msg.result != EC_RES_SUCCESS)
				return -EINVAL;
			i = 0;
			ok = 1;
		}

		/* Skip over the number we just read */
		while (*buf && !isspace(*buf))
			buf++;

	} while (*buf);

	return (ok && i == 0) ? count : -EINVAL;
}

static const char const *seqname[] = {
	"ERROR", "S5", "S3", "S0", "S5S3", "S3S0",
	"S0S3", "S3S5", "STOP", "RUN", "PULSE", "TEST", "KONAMI",
};

static ssize_t show_seq(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ec_params_lightbar param;
	struct ec_response_lightbar resp;
	struct cros_ec_dev *ec = container_of(
			dev, struct cros_ec_dev, class_dev);
	struct cros_ec_command msg = INIT_MSG(ec, param, resp);
	int ret;

	param.cmd = LIGHTBAR_CMD_GET_SEQ;
	ret = lb_throttle();
	if (ret)
		return ret;
	ret = cros_ec_cmd_xfer(ec->ec_dev, &msg);
	if (ret < 0)
		return ret;
	if (msg.result != EC_RES_SUCCESS)
		return scnprintf(buf, PAGE_SIZE,
				 "ERROR: EC returned %d\n", msg.result);

	if (resp.get_seq.num >= ARRAY_SIZE(seqname))
		return scnprintf(buf, PAGE_SIZE, "%d\n", resp.get_seq.num);
	else
		return scnprintf(buf, PAGE_SIZE, "%s\n",
				 seqname[resp.get_seq.num]);
}

static ssize_t store_seq(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct ec_params_lightbar param;
	struct ec_response_lightbar resp;
	struct cros_ec_dev *ec = container_of(
			dev, struct cros_ec_dev, class_dev);
	struct cros_ec_command msg = INIT_MSG(ec, param, resp);
	unsigned int num;
	int ret, len;

	for (len = 0; len < count; len++)
		if (!isalnum(buf[len]))
			break;

	for (num = 0; num < ARRAY_SIZE(seqname); num++)
		if (!strncasecmp(seqname[num], buf, len))
			break;

	if (num >= ARRAY_SIZE(seqname))
		sscanf(buf, "%u", &num);

	param.cmd = LIGHTBAR_CMD_SEQ;
	param.seq.num = num;
	ret = lb_throttle();
	if (ret)
		return ret;
	ret = cros_ec_cmd_xfer(ec->ec_dev, &msg);
	if (ret < 0)
		return ret;
	if (msg.result != EC_RES_SUCCESS)
		return -EINVAL;

	return count;
}

/* Module initialization */

static DEVICE_ATTR(interval_msec, S_IWUSR | S_IRUGO,
		   show_interval_msec, store_interval_msec);
static DEVICE_ATTR(version, S_IRUGO, show_version, 0);
static DEVICE_ATTR(brightness, S_IWUGO, 0, store_brightness);
static DEVICE_ATTR(led_rgb, S_IWUGO, 0, store_rgb);
static DEVICE_ATTR(sequence, S_IWUGO | S_IRUGO, show_seq, store_seq);

static struct attribute *__lb_cmds_attrs[] = {
	&dev_attr_interval_msec.attr,
	&dev_attr_version.attr,
	&dev_attr_brightness.attr,
	&dev_attr_led_rgb.attr,
	&dev_attr_sequence.attr,
	NULL,
};

static umode_t cros_ec_lightbar_attrs_are_visible(struct kobject *kobj,
						  struct attribute *a, int n)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cros_ec_dev *ec = container_of(
			dev, struct cros_ec_dev, class_dev);
	struct platform_device *pdev  = container_of(
			ec->dev, struct platform_device, dev);

	/* TODO(gwendal): Workaround for crbug/412434 */
	if (pdev->id != 0)
		return 0;

	/* Only instantiate this stuff if the EC has a lightbar */
	if (get_lightbar_version(ec, NULL, NULL))
		return a->mode;
	else
		return 0;
}

struct attribute_group cros_ec_lightbar_attr_group = {
	.name = "lightbar",
	.attrs = __lb_cmds_attrs,
	.is_visible = cros_ec_lightbar_attrs_are_visible,
};

