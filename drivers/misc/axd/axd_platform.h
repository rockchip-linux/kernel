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
 * Platform Specific helper functions.
 */
#ifndef AXD_PLATFORM_H_
#define AXD_PLATFORM_H_
#include "axd_module.h"

void axd_platform_init(struct axd_dev *axd);
void axd_platform_set_pc(unsigned long pc);
int axd_platform_start(void);
void axd_platform_stop(void);
unsigned int axd_platform_num_threads(void);
void axd_platform_kick(void);
void axd_platform_irq_ack(void);
void axd_platform_print_regs(void);

/*
 * protect against simultaneous access to shared memory mapped registers area
 * between axd and the host
 */
unsigned long axd_platform_lock(void);
void axd_platform_unlock(unsigned long flags);

#endif /* AXD_PLATFORM_H_ */
