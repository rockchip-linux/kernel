/*
 * linux/sound/rt5677.h -- Platform data for RT5677
 *
 * Copyright 2013 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_SND_RT5677_H
#define __LINUX_SND_RT5677_H

enum rt5677_micbias {
	RT5677_MICBIAS_1_476V = 0,
	RT5677_MICBIAS_2_970V = 1,
	RT5677_MICBIAS_1_242V = 2,
	RT5677_MICBIAS_2_475V = 3,
};

enum rt5677_dmic2_clk {
	RT5677_DMIC_CLK1 = 0,
	RT5677_DMIC_CLK2 = 1,
};

enum rt5677_pdm_clk_div {
	RT5677_PDM_CLK_DIV1 = 0,
	RT5677_PDM_CLK_DIV2 = 1,
	RT5677_PDM_CLK_DIV4 = 2,
	RT5677_PDM_CLK_DIV3 = 3,
};

struct rt5677_platform_data {
	/* MICBIAS output voltage control */
	enum rt5677_micbias micbias1;
	/* Select codec internal 1.8V as DACREF source optionally */
	bool internal_dacref_en;
	/* IN1/IN2/LOUT1/LOUT2/LOUT3 can optionally be differential */
	bool in1_diff;
	bool in2_diff;
	bool lout1_diff;
	bool lout2_diff;
	bool lout3_diff;
	/* DMIC2 clock source selection */
	enum rt5677_dmic2_clk dmic2_clk_pin;
	/* System clock to PDM filter divider */
	enum rt5677_pdm_clk_div pdm_clk_div;

	/* configures GPIO, 0 - floating, 1 - pulldown, 2 - pullup */
	u8 gpio_config[6];
	/* Asynchronous Sample Rate Converter can be optionally enabled */
	bool asrc_en;

	/* jd1 can select 0 ~ 3 as OFF, GPIO1, GPIO2 and GPIO3 respectively */
	unsigned int jd1_gpio;
	/* jd2 and jd3 can select 0 ~ 3 as
		OFF, GPIO4, GPIO5 and GPIO6 respectively */
	unsigned int jd2_gpio;
	unsigned int jd3_gpio;
};

#endif
