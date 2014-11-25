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
 * Helper functions to parse AXD Header in the firmware binary
 */
#ifndef AXD_HDR_H_
#define AXD_HDR_H_

void axd_hdr_init(unsigned long address);
unsigned long axd_hdr_get_pc(unsigned int thread);
unsigned long axd_hdr_get_cmdblock_offset(void);
char *axd_hdr_get_build_str(void);
unsigned long axd_hdr_get_log_offset(void);

#endif /* AXD_HDR_H_ */
