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
 * AXD is a hardware IP that provides various audio decoding capabilities for
 * user applications, offloading the core on which the application is running
 * and saving its valuable MIPS.
 */
#ifndef AXD_MODULE_H_
#define AXD_MODULE_H_
#include <linux/cdev.h>
#include <linux/clk.h>

#include "axd_api.h"
#include "axd_cmds.h"

#define MAX_CTRL_DEVICES		1
#define MAX_IN_DEVICES			AXD_MAX_PIPES
#define MAX_OUT_DEVICES			AXD_MAX_PIPES
#define MAX_NUM_DEVICES			(MAX_CTRL_DEVICES + MAX_IN_DEVICES + MAX_OUT_DEVICES)

#define CTRL_TO_DEVNO(cdev, i)		((cdev)->dev+(i))
#define INPUT_TO_DEVNO(cdev, i)		(CTRL_TO_DEVNO((cdev), MAX_CTRL_DEVICES) + (i))
#define OUTPUT_TO_DEVNO(cdev, i)	(INPUT_TO_DEVNO((cdev), MAX_IN_DEVICES) + (i))

#define MINOR_TO_CTRL(minor)		(minor)
#define MINOR_TO_INPUT(minor)		((minor) - MAX_CTRL_DEVICES)
#define MINOR_TO_OUTPUT(minor)		((minor) - (MAX_IN_DEVICES + MAX_CTRL_DEVICES))

void axd_schedule_reset(struct axd_cmd *cmd);


/**
 * struct axd_dev - axd device structure
 * @cdev:		char device structure
 * @class:		class structure
 * @dev:		pointer to struct device from platform_device
 * @ctrldev:		array of pointers to created ctrl devices
 *			(usually 1 only)
 * @inputdev:		array of pointers to created input devices
 * @outputdev:		array of pointers to created output devices
 * @id:			id of this axd device
 * @num_inputs:		number of inputs AXD hardware reported it can handle
 * @num_outputs:	number of outputs AXD hardware reported it provides
 * @axd_cmd:		axd_cmd structure
 * @input_locks:	semaphores to regulate access to input nodes
 * @output_locks:	semaphores to regulate access to output nodes
 * @fw_base_m:		pointer to mapped fw base address
 * @fw_base_p:		physical address of fw base
 * @fw_size:		size of reserved fw region
 * @buf_base_m:		pointer to mapped buffers base address
 * @buf_base_p:		physical address of buffers base
 * @inbuf_size:		size of reserved input buffers region
 * @outbuf_size:	size of reserved output buffers region
 * @host_irq:		gic irq of the host
 * @axd_irq:		gic irq of axd
 * @irqnum:		linux linear irq number for request_irq()
 * @freq:		clock frequency of axd counter
 * @watchdogtimer:	software watchdogtimer to check if axd is alive
 * @watchdogwork:	the work to execute to check if firwmare is still alive
 *			and restart if it discovers the firmware stopped
 *			responding.
 * @timestamps_out_flg:	a flag that indicates whether we should pass output
 *			timestamps or not
 */
struct axd_dev {
	struct cdev cdev;
	struct class *class;
	struct device *dev;
	struct device *ctrldev[MAX_CTRL_DEVICES];
	struct device *inputdev[MAX_IN_DEVICES];
	struct device *outputdev[MAX_OUT_DEVICES];
	int id;
	int num_inputs;
	int num_outputs;
	struct axd_cmd cmd;
	struct semaphore *input_locks;
	struct semaphore *output_locks;
	void __iomem *fw_base_m;
	dma_addr_t fw_base_p;
	unsigned int fw_size;
	void __iomem *buf_base_m;
	dma_addr_t buf_base_p;
	unsigned int inbuf_size;
	unsigned int outbuf_size;
	int host_irq;
	int axd_irq;
	int irqnum;
	struct clk *clk;
	unsigned int vpe;
	struct timer_list watchdogtimer;
	struct work_struct watchdogwork;
	int timestamps_out_flg;
};

#endif /* AXD_MODULE_H_ */
