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
 * AXD Commands API - generic setup functions.
 */
#include "axd_api.h"
#include "axd_cmds.h"
#include "axd_cmds_internal.h"
#include "axd_module.h"

static unsigned long __io_address;
static unsigned long __phys_address;

void axd_cmd_init(struct axd_cmd *cmd, unsigned long cmd_address,
			unsigned long io_address, unsigned long phys_address)
{
	int i;

	cmd->message = (struct axd_memory_map __iomem *)cmd_address;
	mutex_init(&cmd->cm_lock);
	init_waitqueue_head(&cmd->wait);
	axd_set_flag(&cmd->response_flg, 0);
	axd_set_flag(&cmd->fw_stopped_flg, 0);
	for (i = 0; i < AXD_MAX_PIPES; i++) {
		axd_cmd_inpipe_init(cmd, i);
		axd_cmd_outpipe_init(cmd, i);
	}
	__io_address = io_address;
	__phys_address = phys_address;
	cmd->watchdogenabled = 1;
	/*
	 * By default, always discard any pending buffers if an output device is
	 * closed before EOS is reached.
	 * This behaviour can be changed through sysfs. If discard is disabled,
	 * then upon closing an output device before EOS is reached, it'll
	 * resume from where it stopped.
	 */
	axd_set_flag(&cmd->discard_flg, 1);
	axd_set_flag(&cmd->ctrlbuf_active_flg, 0);
}

int axd_cmd_set_pc(struct axd_cmd *cmd, unsigned int thread, unsigned long pc)
{
	if (thread >= THREAD_COUNT)
		return -1;
	iowrite32(pc, &cmd->message->pc[thread]);
	return 0;
}

unsigned long  axd_cmd_get_datain_address(struct axd_cmd *cmd)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);

	return (unsigned long) axd->buf_base_m;
}

unsigned long  axd_cmd_get_datain_size(struct axd_cmd *cmd)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);

	return axd->inbuf_size;
}

unsigned long  axd_cmd_get_dataout_address(struct axd_cmd *cmd)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);

	return ((unsigned long) axd->buf_base_m) + axd->inbuf_size;
}

unsigned long  axd_cmd_get_dataout_size(struct axd_cmd *cmd)
{
	struct axd_dev *axd = container_of(cmd, struct axd_dev, cmd);

	return axd->outbuf_size;
}

/*
 * The driver understands IO address, while f/w understands physical addresses.
 * A couple of helper functions to aid in converting when exchanging buffers.
 *
 * NOTE:
 * buf must NOT be NULL - we want this as fast as possible, so omit the check
 * for NULLl
 */
inline char *axd_io_2_phys(const char *buf)
{
	return (char *)(buf - __io_address + __phys_address);
}
inline char *axd_phys_2_io(const char *buf)
{
	return (char *)(buf - __phys_address + __io_address);
}
