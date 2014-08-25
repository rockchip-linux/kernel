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
 * nss_hal_pvt.c
 *	NSS HAL private APIs.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/version.h>

#if (NSS_DT_SUPPORT != 1)
#include <mach/gpiomux.h>
#endif

#include "nss_hal_pvt.h"
#include "nss_clocks.h"
#include "nss_core.h"

/*
 * Global declarations
 */
extern struct nss_top_instance nss_top_main;

#if (NSS_FW_DBG_SUPPORT == 1)
/*
 * NSS debug pins configuration
 */

/*
 * Core 0, Data
 * No pull up, Function 2
 */
static struct gpiomux_setting nss_spi_data_0 = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

/*
 * Core 0, CLK, CS
 * Pull up high, Function 2
 */
static struct gpiomux_setting nss_spi_cs_clk_0 = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

/*
 * Core 1, CS
 * Pull up high, Function 4
 */
static struct gpiomux_setting nss_spi_cs_1 = {
	.func = GPIOMUX_FUNC_4,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

/*
 * Core 1, CLK
 * Pull up high, Function 5
 */
static struct gpiomux_setting nss_spi_clk_1 = {
	.func = GPIOMUX_FUNC_5,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

/*
 * Core 1, Data
 * Pull up none, Function 5
 */
static struct gpiomux_setting nss_spi_data_1 = {
	.func = GPIOMUX_FUNC_5,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config nss_spi_gpiomux[] = {
	{
		.gpio = 14,
		.settings = {
			[GPIOMUX_ACTIVE] = &nss_spi_data_0,
			[GPIOMUX_SUSPENDED] = &nss_spi_data_0,
		},
	},
	{
		.gpio = 15,
		.settings = {
			[GPIOMUX_ACTIVE] = &nss_spi_data_0,
			[GPIOMUX_SUSPENDED] = &nss_spi_data_0,
		},
	},
	{
		.gpio = 16,
		.settings = {
			[GPIOMUX_ACTIVE] = &nss_spi_cs_clk_0,
			[GPIOMUX_SUSPENDED] = &nss_spi_cs_clk_0,
		},
	},
	{
		.gpio = 17,
		.settings = {
			[GPIOMUX_ACTIVE] = &nss_spi_cs_clk_0,
			[GPIOMUX_SUSPENDED] = &nss_spi_cs_clk_0,
		},
	},
	{
		.gpio = 55,
		.settings = {
			[GPIOMUX_ACTIVE] = &nss_spi_data_1,
			[GPIOMUX_SUSPENDED] = &nss_spi_data_1,
		},
	},
	{
		.gpio = 56,
		.settings = {
			[GPIOMUX_ACTIVE] = &nss_spi_data_1,
			[GPIOMUX_SUSPENDED] = &nss_spi_data_1,
		},
	},
	{
		.gpio = 57,
		.settings = {
			[GPIOMUX_ACTIVE] = &nss_spi_cs_1,
			[GPIOMUX_SUSPENDED] = &nss_spi_cs_1,
		},
	},
	{
		.gpio = 58,
		.settings = {
			[GPIOMUX_ACTIVE] = &nss_spi_clk_1,
			[GPIOMUX_SUSPENDED] = &nss_spi_clk_1,
		},
	},
};
#endif

/*
 * clk_reg_write_32()
 *	Write clock register
 */
static inline void clk_reg_write_32(uint32_t addr_base, uint32_t addr_offset, uint32_t val)
{
	writel(val, (void *)(addr_base + addr_offset));
}

/*
 * clk_reg_read_32()
 *	Write clock register
 */
static inline uint32_t clk_reg_read_32(uint32_t addr_base, uint32_t addr_offset)
{
	return readl((volatile void *)(addr_base + addr_offset));
}

/*
 * __nss_hal_debug_enable()
 *	Enable NSS debug
 */
void __nss_hal_debug_enable(void)
{
#if (NSS_FW_DBG_SUPPORT == 1)
	msm_gpiomux_install(nss_spi_gpiomux,
				ARRAY_SIZE(nss_spi_gpiomux));
#endif
}

#if (NSS_DT_SUPPORT != 1)
/*
 * __nss_hal_common_reset
 *	Do reset/clock configuration common to all cores
 */
void __nss_hal_common_reset(uint32_t *clk_src)
{
	return;
}
#endif

/*
 * __nss_hal_core_reset
 */
void __nss_hal_core_reset(uint32_t map_base, uint32_t reset_addr)
{
	/*
	* Apply ubi32 core reset
	*/
	nss_write_32(map_base, NSS_REGS_RESET_CTRL_OFFSET, 1);

	/*
	 * Program address configuration
	 */
	nss_write_32(map_base, NSS_REGS_CORE_AMC_OFFSET, 1);
	nss_write_32(map_base, NSS_REGS_CORE_BAR_OFFSET, 0x3c000000);
	nss_write_32(map_base, NSS_REGS_CORE_BOOT_ADDR_OFFSET, reset_addr);

	/*
	 * C2C interrupts are level sensitive
	 */
	nss_write_32(map_base, NSS_REGS_CORE_INT_STAT2_TYPE_OFFSET, 0xFFFF);

	/*
	 * Set IF check value
	 */
	 nss_write_32(map_base, NSS_REGS_CORE_IFETCH_RANGE_OFFSET, 0xBF004001);

	/*
	 * De-assert ubi32 core reset
	 */
	nss_write_32(map_base, NSS_REGS_RESET_CTRL_OFFSET, 0);
}
