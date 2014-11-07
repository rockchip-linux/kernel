/*
 * Copyright (c) 2010-2011,2013-2014 The Linux Foundation. All rights reserved.
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

#ifndef _LPASS_CPU_MI2S_H
#define _LPASS_CPU_MI2S_H

enum pinctrl_pin_state {
	STATE_DISABLED = 0,
	STATE_ENABLED = 1
};
static const char *const pin_states[] = {"Disabled", "Enabled"};

struct mi2s_pinctrl {
	struct pinctrl *pinctrl;
	struct pinctrl_state *disabled;
	struct pinctrl_state *enabled;
	enum pinctrl_pin_state curr_state;
};

/*
 * Device data for the multi-channel I2S port in the low-power audio
 * interface (LPAIF) within the low-power audio subsystem (LPASS).
 * Both the CPU DAI driver and platform driver will access this.
 */
struct lpass_cpu_mi2s_data {
	void __iomem *base;
	struct clk *ahbix_clk;
	struct clk *mi2s_bit_clk;
	struct clk *mi2s_osr_clk;
	int mi2s_clocks_enabled;
	struct mi2s_pinctrl mi2s;
	int irqnum;
	int irq_acquired;
	uint8_t prepare_start;
	uint32_t period_index;
};

int lpass_pcm_mi2s_platform_register(struct device *dev);

#endif /* _LPASS_CPU_MI2S_H */
