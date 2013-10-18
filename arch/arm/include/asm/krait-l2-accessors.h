/*
 * Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ASMARM_KRAIT_L2_ACCESSORS_H
#define __ASMARM_KRAIT_L2_ACCESSORS_H

extern void krait_set_l2_indirect_reg(u32 addr, u32 val);
extern u32 krait_get_l2_indirect_reg(u32 addr);

#endif
