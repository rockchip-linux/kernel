/*
 **************************************************************************
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all copies.
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 **************************************************************************
 */

/**
 * nss_hal_pvt.h
 *	NSS HAL private declarations.for IPQ806x platform
 */

#ifndef __NSS_HAL_PVT_H
#define __NSS_HAL_PVT_H

#include "nss_regs.h"
#include <linux/types.h>
#include <linux/platform_device.h>

#define NSS_HAL_SUPPORTED_INTERRUPTS (NSS_REGS_N2H_INTR_STATUS_DATA_COMMAND_QUEUE | \
					NSS_REGS_N2H_INTR_STATUS_EMPTY_BUFFERS_SOS)

/*
 * __nss_hal_read_interrupt_cause()
 */
static inline void __nss_hal_read_interrupt_cause(uint32_t map, uint32_t irq __attribute__ ((unused)), uint32_t shift_factor, uint32_t *cause)
{
	uint32_t value = nss_read_32(map, NSS_REGS_N2H_INTR_STATUS_OFFSET);
	*cause = (((value)>> shift_factor) & 0x7FFF);
}

/*
 * __nss_hal_clear_interrupt_cause()
 */
static inline void __nss_hal_clear_interrupt_cause(uint32_t map, uint32_t irq __attribute__ ((unused)), uint32_t shift_factor, uint32_t cause)
{
	nss_write_32(map, NSS_REGS_N2H_INTR_CLR_OFFSET, (cause << shift_factor));
}

/*
 * __nss_hal_disable_interrupt()
 */
static inline void __nss_hal_disable_interrupt(uint32_t map, uint32_t irq __attribute__ ((unused)), uint32_t shift_factor, uint32_t cause)
{
	nss_write_32(map, NSS_REGS_N2H_INTR_MASK_CLR_OFFSET, (cause << shift_factor));
}

/*
 * __nss_hal_enable_interrupt()
 */
static inline void __nss_hal_enable_interrupt(uint32_t map, uint32_t irq __attribute__ ((unused)), uint32_t shift_factor, uint32_t cause)
{
	nss_write_32(map, NSS_REGS_N2H_INTR_MASK_SET_OFFSET, (cause << shift_factor));
}

/*
 * __nss_hal_send_interrupt()
 */
static inline void __nss_hal_send_interrupt(uint32_t map, uint32_t irq __attribute__ ((unused)), uint32_t cause)
{
	nss_write_32(map, NSS_REGS_C2C_INTR_SET_OFFSET, cause);
}

extern void __nss_hal_core_reset(uint32_t map_base, uint32_t reset_addr);
#if (NSS_DT_SUPPORT != 1)
extern void __nss_hal_common_reset(uint32_t *clk_src);
#endif
extern void __nss_hal_debug_enable(void);
#endif /* __NSS_HAL_PVT_H */
