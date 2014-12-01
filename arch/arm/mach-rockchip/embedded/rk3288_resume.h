/*
 * Rockchip resume header (API from kernel to embedded code)
 *
 * Copyright (c) 2014 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MACH_ROCKCHIP_RK3288_RESUME_H
#define __MACH_ROCKCHIP_RK3288_RESUME_H

/**
 * rk3288_resume_params - Parameter space for the resume code
 *
 * This structure is at the start of the resume blob and is used to communicate
 * between the resume blob and the callers.
 *
 * WARNING: This structure is sitting in PMU SRAM.  If you try to write to that
 * memory using an 8-bit access (or even 16-bit) you'll get an imprecise data
 * abort and it will be very hard to debug.  Keep everything in here as 32-bit
 * wide and aligned.  YOU'VE BEEN WARNED.
 *
 * @resume_loc:		The value here should be the resume address that the CPU
 *			is programmed to go to at resume time.
 *
 * @l2ctlr_f:		If non-zero we'll set l2ctlr at resume time.
 * @l2ctlr:		The value to set l2ctlr to at resume time.
 *
 * @cpu_resume:		The function to jump to when we're all done.
 */
struct rk3288_resume_params {
	/* This is compiled in and can be read to find the resume location */
	__noreturn void (*resume_loc)(void);

	/* Filled in by the client of the resume code */
	u32 l2ctlr_f;		/* u32 not bool to avoid 8-bit SRAM access */
	u32 l2ctlr;

	__noreturn void (*cpu_resume)(void);
};

#endif /* __MACH_ROCKCHIP_RK3288_RESUME_H */
