/*
 * Copyright (C) 2014 Imagination Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __AXD_H__
#define __AXD_H__
#include <linux/clk.h>

/**
 * struct axd_platform_config - axd platform configuration structure
 * @host_irq:		gic irq of host
 * @axd_irq:		gic irq of axd
 * @vpe:		vpe number on which axd should start
 * @clk:		clk struct for axd
 * @inbuf_size:		size of shared input buffers area.
 *			leave 0 for the driver to use the default 0x7800.
 * @outbuf_size:	size of shared output buffers area.
 *			leave 0 for the driver to use the default 0x3c000.
 */
struct axd_platform_config {
	unsigned int host_irq;
	unsigned int axd_irq;
	unsigned int vpe;
	struct clk *clk;
	unsigned int inbuf_size;
	unsigned int outbuf_size;
};
#endif /* __AXD_H_ */
