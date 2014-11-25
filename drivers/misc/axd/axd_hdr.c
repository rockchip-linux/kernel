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
 * Helper functions to parse AXD Header in the firmware binary.
 */
#include <linux/kernel.h>

#include "axd_api.h"
#include "axd_hdr.h"


static struct axd_hdr *hdr;

static void dump_hdr(void)
{
	unsigned int offset = 0;
	unsigned long address = (unsigned long)hdr;

	pr_debug("header <0x%08lX>:\n", address);
	while (offset <= sizeof(*hdr)) {
		pr_debug("0x%08X\t", *(unsigned int *)(address+offset));
		offset += 4;
		if ((offset % (4*4)) == 0)
			pr_debug("\n");
	}
	pr_debug("\n");
}

void axd_hdr_init(unsigned long address)
{
	hdr = (struct axd_hdr *)address;
	dump_hdr();
}

unsigned long axd_hdr_get_pc(unsigned int thread)
{
	if (thread >= THREAD_COUNT)
		return -1;
	return hdr->thread_pc[thread];
}

unsigned long axd_hdr_get_cmdblock_offset(void)
{
	pr_debug("cmdblock_offset = 0x%08X\n", hdr->cmd_block_offset);
	return hdr->cmd_block_offset;
}

char *axd_hdr_get_build_str(void)
{
	return hdr->build_str;
}

unsigned long axd_hdr_get_log_offset(void)
{
	return hdr->log_offset;
}
