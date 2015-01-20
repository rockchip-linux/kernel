/*
 * Copyright (C) 2014 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/of_platform.h>
#include <asm/mach/arch.h>
#include <asm/hardware/cache-l2x0.h>

static void __init bcm_cygnus_dt_init(void)
{
	l2x0_of_init(0, ~0UL);
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}


static const char const *bcm_cygnus_dt_compat[] = {
	"brcm,cygnus",
	NULL,
};

DT_MACHINE_START(BCM_CYGNUS_DT, "Broadcom Cygnus SoC")
	.init_machine	= bcm_cygnus_dt_init,
	.dt_compat = bcm_cygnus_dt_compat,
MACHINE_END
