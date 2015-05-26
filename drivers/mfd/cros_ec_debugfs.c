/*
 * cros_ec_debugfs - debug logs for Chrome OS EC
 *
 * Copyright 2015 Google, Inc.
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

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/mfd/cros_ec.h>
#include <linux/mfd/cros_ec_commands.h>
#include <linux/mfd/cros_ec_dev.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include "cros_ec_debugfs.h"

#define LOG_SHIFT		14
#define LOG_SIZE		(1 << LOG_SHIFT)
#define LOG_POLL_SEC		30
#define LOG_DROPPED_MESSAGE	"\n... some logs may have been dropped ...\n"

#define CIRC_NEXT(idx, size)	(((idx) + 1) & ((size) - 1))

/*
 * TODO(ejcaruso): Move this from a recurring task to being triggered by
 * host events in the future. For now, this should expose a stable interface
 * for a userspace log concatenator.
 */
static void cros_ec_console_log_work(struct work_struct *__work)
{
	struct cros_ec_debugfs *debug_info =
		container_of(to_delayed_work(__work),
			     struct cros_ec_debugfs,
			     log_poll_work);
	struct cros_ec_dev *ec = debug_info->ec;
	struct circ_buf *cb = &debug_info->log_buffer;
	struct cros_ec_command snapshot_msg = {
		.version = 0,
		.command = EC_CMD_CONSOLE_SNAPSHOT + ec->cmd_offset,
		.outdata = NULL,
		.outsize = 0,
		.indata = NULL,
		.insize = 0,
	};

	struct ec_params_console_read_v1 params = {
		.subcmd = CONSOLE_READ_RECENT,
	};
	int log_buffer_size = ec->ec_dev->max_response;
	uint8_t *log_buffer;
	struct cros_ec_command read_msg = {
		.version = 1,
		.command = EC_CMD_CONSOLE_READ + ec->cmd_offset,
		.outdata = (void *)&params,
		.outsize = sizeof(params),
		.indata = 0,
		.insize = log_buffer_size,
	};

	int idx;
	int buf_space;
	int ret;

	ret = cros_ec_cmd_xfer(ec->ec_dev, &snapshot_msg);
	if (ret < 0) {
		dev_err(ec->dev, "EC communication failed\n");
		goto resched;
	}
	if (snapshot_msg.result != EC_RES_SUCCESS) {
		dev_err(ec->dev, "EC failed to snapshot the console log\n");
		goto resched;
	}

	/* Loop until we have read everything, or there's an error. */
	mutex_lock(&debug_info->log_mutex);
	buf_space = CIRC_SPACE(cb->head, cb->tail, LOG_SIZE);

	log_buffer = kmalloc(log_buffer_size, GFP_KERNEL);
	if (!log_buffer)
		goto resched;
	read_msg.indata = log_buffer;

	while (1) {
		if (!buf_space) {
			debug_info->log_may_have_dropped = true;
			break;
		}

		memset(log_buffer, '\0', log_buffer_size);
		ret = cros_ec_cmd_xfer(ec->ec_dev, &read_msg);
		if (ret < 0) {
			dev_err(ec->dev, "EC communication failed\n");
			break;
		}
		if (read_msg.result != EC_RES_SUCCESS) {
			dev_err(ec->dev,
				"EC failed to read the console log\n");
			break;
		}

		/* If the buffer is empty, we're done here. */
		if (log_buffer[0] == '\0')
			break;

		idx = 0;
		while (log_buffer[idx] != '\0' &&
		       idx < log_buffer_size &&
		       buf_space > 0) {
			cb->buf[cb->head] = log_buffer[idx];
			cb->head = CIRC_NEXT(cb->head, LOG_SIZE);
			idx++;
			buf_space--;
		}

		wake_up(&debug_info->log_wq);
	}

	mutex_unlock(&debug_info->log_mutex);
	kfree(log_buffer);

resched:
	schedule_delayed_work(&debug_info->log_poll_work,
			      msecs_to_jiffies(LOG_POLL_SEC * 1000));

	return;
}

static int cros_ec_console_log_show(struct seq_file *m, void *data)
{
	struct cros_ec_debugfs *debug_info = m->private;
	struct circ_buf *cb = &debug_info->log_buffer;
	int to_write;
	int bytes_written;
	int ret;

	mutex_lock(&debug_info->log_mutex);
	wait_event_cmd(debug_info->log_wq,
		       CIRC_CNT(cb->head, cb->tail, LOG_SIZE),
		       mutex_unlock(&debug_info->log_mutex),
		       mutex_lock(&debug_info->log_mutex));

	to_write = CIRC_CNT_TO_END(cb->head, cb->tail, LOG_SIZE);
	ret = seq_write(m, cb->buf + cb->tail, to_write);
	if (ret < 0)
		goto error;
	bytes_written = ret;

	if (cb->head >= cb->tail)
		goto out;

	ret = seq_write(m, cb->buf, cb->head);
	if (ret < 0)
		goto error;
	ret += bytes_written;

out:
	if (debug_info->log_may_have_dropped) {
		seq_puts(m, LOG_DROPPED_MESSAGE);
		debug_info->log_may_have_dropped = false;
	}

	WARN_ON(ret != CIRC_CNT(cb->head, cb->tail, LOG_SIZE));
	cb->tail = cb->head;

error:
	mutex_unlock(&debug_info->log_mutex);
	return ret;
}

static int cros_ec_console_log_open(struct inode *inode, struct file *file)
{
	struct cros_ec_debugfs *debug_info = inode->i_private;

	return single_open_size(file,
				cros_ec_console_log_show,
				debug_info,
				LOG_SIZE + sizeof(LOG_DROPPED_MESSAGE));
}

static unsigned int cros_ec_console_log_poll(struct file *file,
					     poll_table *wait)
{
	struct seq_file *m = file->private_data;
	struct cros_ec_debugfs *debug_info = m->private;
	unsigned int mask = 0;

	poll_wait(file, &debug_info->log_wq, wait);

	mutex_lock(&debug_info->log_mutex);
	if (CIRC_CNT(debug_info->log_buffer.head,
		     debug_info->log_buffer.tail,
		     LOG_SIZE))
		mask |= POLLIN | POLLRDNORM;
	mutex_unlock(&debug_info->log_mutex);

	return mask;
}

const struct file_operations cros_ec_console_log_fops = {
	.owner = THIS_MODULE,
	.open = cros_ec_console_log_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.poll = cros_ec_console_log_poll,
	.release = single_release,
};

static int ec_read_version_supported(struct cros_ec_dev *ec)
{
	struct ec_params_get_cmd_versions_v1 params = {
		.cmd = EC_CMD_CONSOLE_READ,
	};
	struct ec_response_get_cmd_versions response;
	struct cros_ec_command msg = {
		.version = 0,
		.command = EC_CMD_GET_CMD_VERSIONS + ec->cmd_offset,
		.outdata = (void *)&params,
		.outsize = sizeof(params),
		.indata = (void *)&response,
		.insize = sizeof(response),
	};
	int ret;

	ret = cros_ec_cmd_xfer(ec->ec_dev, &msg);
	return ret >= 0 &&
	       msg.result == EC_RES_SUCCESS &&
	       (response.version_mask & EC_VER_MASK(1));
}

static int cros_ec_create_console_log(struct cros_ec_debugfs *debug_info)
{
	struct cros_ec_dev *ec = debug_info->ec;
	char *buf;

	if (!ec_read_version_supported(ec)) {
		dev_warn(ec->dev, "device does not support read recent\n");
		return -EINVAL;
	}

	buf = kmalloc(LOG_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (!debugfs_create_file("console_log",
				 S_IFREG | S_IRUGO,
				 debug_info->dir,
				 debug_info,
				 &cros_ec_console_log_fops)) {
		kfree(buf);
		return -ENOMEM;
	}

	debug_info->log_buffer.buf = buf;
	debug_info->log_buffer.head = 0;
	debug_info->log_buffer.tail = 0;
	debug_info->log_may_have_dropped = false;

	mutex_init(&debug_info->log_mutex);
	init_waitqueue_head(&debug_info->log_wq);

	INIT_DELAYED_WORK(&debug_info->log_poll_work,
			  cros_ec_console_log_work);
	schedule_delayed_work(&debug_info->log_poll_work, 0);

	return 0;
}

static void cros_ec_cleanup_console_log(struct cros_ec_debugfs *debug_info)
{
	if (debug_info->log_buffer.buf) {
		cancel_delayed_work_sync(&debug_info->log_poll_work);
		mutex_destroy(&debug_info->log_mutex);
		kfree(debug_info->log_buffer.buf);
	}
}

int cros_ec_debugfs_init(struct cros_ec_dev *ec)
{
	struct cros_ec_platform *ec_platform = dev_get_platdata(ec->dev);
	const char *name = ec_platform->ec_name;
	struct cros_ec_debugfs *debug_info;
	int ret;

	debug_info = kmalloc(sizeof(*debug_info), GFP_KERNEL);
	if (!debug_info)
		return -ENOMEM;

	debug_info->ec = ec;
	debug_info->dir = debugfs_create_dir(name, NULL);
	if (IS_ERR_OR_NULL(debug_info->dir)) {
		kfree(debug_info);
		return -ENOMEM;
	}

	ret = cros_ec_create_console_log(debug_info);
	if (ret) {
		debugfs_remove_recursive(debug_info->dir);
		kfree(debug_info);
		return ret;
	}

	return 0;
}

void cros_ec_debugfs_remove(struct cros_ec_dev *ec)
{
	if (!ec->debug_info)
		return;

	debugfs_remove_recursive(ec->debug_info->dir);
	cros_ec_cleanup_console_log(ec->debug_info);
}
