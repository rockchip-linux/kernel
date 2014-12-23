/*
 * Copyright (c) 2010-2011,2013-2015 The Linux Foundation. All rights reserved.
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

#ifndef __LPASS_H__
#define __LPASS_H__

#define LPASS_AHBIX_CLOCK_FREQUENCY		131072

/* Both the CPU DAI driver and platform driver will access this data */
struct lpass_data {

	/* clocks inside the low-power audio subsystem (LPASS) domain */
	struct clk *ahbix_clk;
	struct clk *mi2s_bit_clk;
	struct clk *mi2s_osr_clk;

	/* default system (or OSR) clock frequency */
	unsigned int default_sysclk_freq;

	/*
	 * if enabled and the target BIT clock frequency is below the range of
	 * the clock divider, then the system clock needs to be reduced by
	 * right-shifting the default clock frequency
	 */
	bool sysclk_shift_enable;
	unsigned int sysclk_shift_compare;
	unsigned int sysclk_shift_amount;

	/*
	 * if enabled and the target BIT clock frequency can not be accurately
	 * derived by the clock divider for the given bitwidth, then use an
	 * alternate system clock frequency
	 */
	bool alt_sysclk_enable;
	unsigned int alt_sysclk_enable_bitwidth;
	unsigned int alt_sysclk_freq;

	/* memory-mapped registers for the low-power audio interface (LPAIF) */
	void __iomem *lpaif;
	struct regmap *lpaif_map;

	/* handle for the low-power audio interface (LPAIF) interrupts */
	int lpaif_irq;

	/* memory-mapped RAM designated for holding the audio buffer(s) */
	void __iomem *lpm;
	dma_addr_t lpm_phys;
	unsigned int lpm_size;
	atomic_t lpm_lock;
};

int asoc_qcom_lpass_platform_register(struct platform_device *);

#endif /* __LPASS_H__ */
