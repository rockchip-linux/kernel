/*
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

#ifndef _DRV_CROS_EC_DEBUGFS_H_
#define _DRV_CROS_EC_DEBUGFS_H_

#include <linux/circ_buf.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#include "cros_ec_dev.h"

/* struct cros_ec_debugfs - ChromeOS EC debugging information
 *
 * @ec: EC device this debugfs information belongs to
 * @dir: dentry for debugfs files
 * @log_buffer: circular buffer for console log information
 * @log_may_have_dropped: true if the buffer has filled during a poll
 * @log_mutex: mutex to protect circular buffer
 * @log_wq: waitqueue for log readers
 * @log_poll_work: recurring task to poll EC for new console log data
 */
struct cros_ec_debugfs {
	struct cros_ec_dev *ec;
	struct dentry *dir;
	struct circ_buf log_buffer;
	bool log_may_have_dropped;
	struct mutex log_mutex;
	wait_queue_head_t log_wq;
	struct delayed_work log_poll_work;
};

/* debugfs stuff */
int cros_ec_debugfs_init(struct cros_ec_dev *ec);
void cros_ec_debugfs_remove(struct cros_ec_dev *ec);

#endif  /* _DRV_CROS_EC_DEBUGFS_H_ */
