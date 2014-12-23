/*
 * Pistachio SoC pinctrl driver
 *
 * Copyright (C) 2014 Imagination Technologies Ltd.
 *
 * Author: Damien Horsley <Damien.Horsley@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>

#include "pinctrl-img.h"

#define MODULE_NAME "pinctrl-pistachio"

#define PISTACHIO_GPIOS	90
#define PISTACHIO_MFIOS	90
#define PISTACHIO_PINS	99

#define PISTACHIO_MFIO_PIN(p) PINCTRL_PIN(p, "mfio" #p)

struct pinctrl_pin_desc pistachio_pins[PISTACHIO_PINS] = {
	PISTACHIO_MFIO_PIN(0),
	PISTACHIO_MFIO_PIN(1),
	PISTACHIO_MFIO_PIN(2),
	PISTACHIO_MFIO_PIN(3),
	PISTACHIO_MFIO_PIN(4),
	PISTACHIO_MFIO_PIN(5),
	PISTACHIO_MFIO_PIN(6),
	PISTACHIO_MFIO_PIN(7),
	PISTACHIO_MFIO_PIN(8),
	PISTACHIO_MFIO_PIN(9),
	PISTACHIO_MFIO_PIN(10),
	PISTACHIO_MFIO_PIN(11),
	PISTACHIO_MFIO_PIN(12),
	PISTACHIO_MFIO_PIN(13),
	PISTACHIO_MFIO_PIN(14),
	PISTACHIO_MFIO_PIN(15),
	PISTACHIO_MFIO_PIN(16),
	PISTACHIO_MFIO_PIN(17),
	PISTACHIO_MFIO_PIN(18),
	PISTACHIO_MFIO_PIN(19),
	PISTACHIO_MFIO_PIN(20),
	PISTACHIO_MFIO_PIN(21),
	PISTACHIO_MFIO_PIN(22),
	PISTACHIO_MFIO_PIN(23),
	PISTACHIO_MFIO_PIN(24),
	PISTACHIO_MFIO_PIN(25),
	PISTACHIO_MFIO_PIN(26),
	PISTACHIO_MFIO_PIN(27),
	PISTACHIO_MFIO_PIN(28),
	PISTACHIO_MFIO_PIN(29),
	PISTACHIO_MFIO_PIN(30),
	PISTACHIO_MFIO_PIN(31),
	PISTACHIO_MFIO_PIN(32),
	PISTACHIO_MFIO_PIN(33),
	PISTACHIO_MFIO_PIN(34),
	PISTACHIO_MFIO_PIN(35),
	PISTACHIO_MFIO_PIN(36),
	PISTACHIO_MFIO_PIN(37),
	PISTACHIO_MFIO_PIN(38),
	PISTACHIO_MFIO_PIN(39),
	PISTACHIO_MFIO_PIN(40),
	PISTACHIO_MFIO_PIN(41),
	PISTACHIO_MFIO_PIN(42),
	PISTACHIO_MFIO_PIN(43),
	PISTACHIO_MFIO_PIN(44),
	PISTACHIO_MFIO_PIN(45),
	PISTACHIO_MFIO_PIN(46),
	PISTACHIO_MFIO_PIN(47),
	PISTACHIO_MFIO_PIN(48),
	PISTACHIO_MFIO_PIN(49),
	PISTACHIO_MFIO_PIN(50),
	PISTACHIO_MFIO_PIN(51),
	PISTACHIO_MFIO_PIN(52),
	PISTACHIO_MFIO_PIN(53),
	PISTACHIO_MFIO_PIN(54),
	PISTACHIO_MFIO_PIN(55),
	PISTACHIO_MFIO_PIN(56),
	PISTACHIO_MFIO_PIN(57),
	PISTACHIO_MFIO_PIN(58),
	PISTACHIO_MFIO_PIN(59),
	PISTACHIO_MFIO_PIN(60),
	PISTACHIO_MFIO_PIN(61),
	PISTACHIO_MFIO_PIN(62),
	PISTACHIO_MFIO_PIN(63),
	PISTACHIO_MFIO_PIN(64),
	PISTACHIO_MFIO_PIN(65),
	PISTACHIO_MFIO_PIN(66),
	PISTACHIO_MFIO_PIN(67),
	PISTACHIO_MFIO_PIN(68),
	PISTACHIO_MFIO_PIN(69),
	PISTACHIO_MFIO_PIN(70),
	PISTACHIO_MFIO_PIN(71),
	PISTACHIO_MFIO_PIN(72),
	PISTACHIO_MFIO_PIN(73),
	PISTACHIO_MFIO_PIN(74),
	PISTACHIO_MFIO_PIN(75),
	PISTACHIO_MFIO_PIN(76),
	PISTACHIO_MFIO_PIN(77),
	PISTACHIO_MFIO_PIN(78),
	PISTACHIO_MFIO_PIN(79),
	PISTACHIO_MFIO_PIN(80),
	PISTACHIO_MFIO_PIN(81),
	PISTACHIO_MFIO_PIN(82),
	PISTACHIO_MFIO_PIN(83),
	PISTACHIO_MFIO_PIN(84),
	PISTACHIO_MFIO_PIN(85),
	PISTACHIO_MFIO_PIN(86),
	PISTACHIO_MFIO_PIN(87),
	PISTACHIO_MFIO_PIN(88),
	PISTACHIO_MFIO_PIN(89),
	PINCTRL_PIN(90, "TCK"),
	PINCTRL_PIN(91, "TRSTN"),
	PINCTRL_PIN(92, "TDI"),
	PINCTRL_PIN(93, "TMS"),
	PINCTRL_PIN(94, "TDO"),
	PINCTRL_PIN(95, "JTAG_COMPLY"),
	PINCTRL_PIN(96, "SAFE_MODE"),
	PINCTRL_PIN(97, "POR_DISABLE"),
	PINCTRL_PIN(98, "RESETN"),
};

unsigned int pistachio_mfio_reg_ctrl[PISTACHIO_MFIOS] = {
	0x00000000,
	0x020000C0,
	0x020200C0,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x020400C0,
	0x020600C0,
	0x020800C0,
	0x020A00C0,
	0x020C00C0,
	0x020E00C0,
	0x021000C0,
	0x021200C0,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x011400C0,
	0x011500C0,
	0x011600C0,
	0x011700C0,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x011800C0,
	0x011900C0,
	0x011A00C0,
	0x011B00C0,
	0x011C00C0,
	0x011D00C0,
	0x011E00C0,
	0x011F00C0,
	0x00000000,
	0x010000C4,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x020100C4,
	0x020300C4,
	0x020500C4,
	0x020700C4,
	0x010900C4,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x020A00C4,
	0x020C00C4,
	0x020E00C4,
	0x021000C4,
	0x021200C4,
	0x021400C4,
	0x021600C4,
	0x021800C4,
	0x00000000,
	0x00000000,
	0x021A00C4,
	0x021C00C4,
	0x021E00C4,
	0x020000C8,
	0x020200C8,
	0x020400C8,
	0x020600C8,
	0x020800C8,
	0x020A00C8,
	0x020C00C8,
	0x020E00C8,
	0x021000C8,
	0x021200C8,
	0x021400C8,
	0x021600C8,
	0x021800C8,
	0x021A00C8,
};

const const char *pistachio_spim1_group_names[] = {
		"spim1"
};
unsigned int pistachio_group_spim1_pins[] = {
		3, 4, 5
};
unsigned int pistachio_group_spim1_select[] = {
		0, 0, 0
};
const const char *pistachio_spim1_quad_group_names[] = {
		"spim1_quad"
};
unsigned int pistachio_group_spim1_quad_pins[] = {
		6, 7
};
unsigned int pistachio_group_spim1_quad_select[] = {
		0, 0
};
const const char *pistachio_spim1_cs0_group_names[] = {
		"spim1_cs0"
};
unsigned int pistachio_group_spim1_cs0_pins[] = {
		0
};
unsigned int pistachio_group_spim1_cs0_select[] = {
		0
};
const const char *pistachio_spim1_cs1_group_names[] = {
		"spim1_cs1_0", "spim1_cs1_1"
};
unsigned int pistachio_group_spim1_cs1_0_pins[] = {
		1
};
unsigned int pistachio_group_spim1_cs1_0_select[] = {
		0
};
unsigned int pistachio_group_spim1_cs1_1_pins[] = {
		58
};
unsigned int pistachio_group_spim1_cs1_1_select[] = {
		1
};
const const char *pistachio_spim1_cs2_group_names[] = {
		"spim1_cs2_0", "spim1_cs2_1", "spim1_cs2_2"
};
unsigned int pistachio_group_spim1_cs2_0_pins[] = {
		2
};
unsigned int pistachio_group_spim1_cs2_0_select[] = {
		0
};
unsigned int pistachio_group_spim1_cs2_1_pins[] = {
		31
};
unsigned int pistachio_group_spim1_cs2_1_select[] = {
		1
};
unsigned int pistachio_group_spim1_cs2_2_pins[] = {
		55
};
unsigned int pistachio_group_spim1_cs2_2_select[] = {
		2
};
const const char *pistachio_spim1_cs3_group_names[] = {
		"spim1_cs3"
};
unsigned int pistachio_group_spim1_cs3_pins[] = {
		56
};
unsigned int pistachio_group_spim1_cs3_select[] = {
		2
};
const const char *pistachio_spim1_cs4_group_names[] = {
		"spim1_cs4"
};
unsigned int pistachio_group_spim1_cs4_pins[] = {
		57
};
unsigned int pistachio_group_spim1_cs4_select[] = {
		2
};

const char *pistachio_spim0_group_names[] = {
		"spim0"
};
unsigned int pistachio_group_spim0_pins[] = {
		8, 9, 10,
};
unsigned int pistachio_group_spim0_select[] = {
		0, 0, 0,
};
const char *pistachio_spim0_cs0_group_names[] = {
		"spim0_cs0"
};
unsigned int pistachio_group_spim0_cs0_pins[] = {
		2
};
unsigned int pistachio_group_spim0_cs0_select[] = {
		1
};
const char *pistachio_spim0_cs1_group_names[] = {
		"spim0_cs1"
};
unsigned int pistachio_group_spim0_cs1_pins[] = {
		1
};
unsigned int pistachio_group_spim0_cs1_select[] = {
		1
};
const char *pistachio_spim0_cs2_group_names[] = {
		"spim0_cs2_0", "spim0_cs2_1"
};
unsigned int pistachio_group_spim0_cs2_0_pins[] = {
		55
};
unsigned int pistachio_group_spim0_cs2_0_select[] = {
		1
};
unsigned int pistachio_group_spim0_cs2_1_pins[] = {
		28
};
unsigned int pistachio_group_spim0_cs2_1_select[] = {
		1
};
const char *pistachio_spim0_cs3_group_names[] = {
		"spim0_cs3_0", "spim0_cs3_1"
};
unsigned int pistachio_group_spim0_cs3_0_pins[] = {
		56
};
unsigned int pistachio_group_spim0_cs3_0_select[] = {
		1
};
unsigned int pistachio_group_spim0_cs3_1_pins[] = {
		29
};
unsigned int pistachio_group_spim0_cs3_1_select[] = {
		1
};
const char *pistachio_spim0_cs4_group_names[] = {
		"spim0_cs4_0", "spim0_cs4_1"
};
unsigned int pistachio_group_spim0_cs4_0_pins[] = {
		57
};
unsigned int pistachio_group_spim0_cs4_0_select[] = {
		1
};
unsigned int pistachio_group_spim0_cs4_1_pins[] = {
		30
};
unsigned int pistachio_group_spim0_cs4_1_select[] = {
		1
};

const char *pistachio_spis_group_names[] = {
		"spis"
};
unsigned int pistachio_group_spis_pins[] = {
		11, 12, 13, 14
};
unsigned int pistachio_group_spis_select[] = {
		0, 0, 0, 0
};

const char *pistachio_sdhost_group_names[] = {
		"sdhost"
};
unsigned int pistachio_group_sdhost_pins[] = {
		15, 16, 17, 25, 26, 27
};
unsigned int pistachio_group_sdhost_select[] = {
		0, 0, 0, 0, 0, 0
};
const char *pistachio_sdhost_dat1to3_group_names[] = {
		"sdhost_dat1to3"
};
unsigned int pistachio_group_sdhost_dat1to3_pins[] = {
		18, 19, 20
};
unsigned int pistachio_group_sdhost_dat1to3_select[] = {
		0, 0, 0
};
const char *pistachio_sdhost_dat4to7_group_names[] = {
		"sdhost_dat4to7"
};
unsigned int pistachio_group_sdhost_dat4to7_pins[] = {
		21, 22, 23, 24
};
unsigned int pistachio_group_sdhost_dat4to7_select[] = {
		0, 0, 0, 0
};

const char *pistachio_i2c0_group_names[] = {
		"i2c0"
};
unsigned int pistachio_group_i2c0_pins[] = {
		28, 29
};
unsigned int pistachio_group_i2c0_select[] = {
		0, 0
};

const char *pistachio_i2c1_group_names[] = {
		"i2c1"
};
unsigned int pistachio_group_i2c1_pins[] = {
		30, 31
};
unsigned int pistachio_group_i2c1_select[] = {
		0, 0
};

const char *pistachio_i2c2_group_names[] = {
		"i2c2"
};
unsigned int pistachio_group_i2c2_pins[] = {
		32, 33
};
unsigned int pistachio_group_i2c2_select[] = {
		0, 0
};

const char *pistachio_i2c3_group_names[] = {
		"i2c3"
};
unsigned int pistachio_group_i2c3_pins[] = {
		34, 35
};
unsigned int pistachio_group_i2c3_select[] = {
		0, 0
};

const char *pistachio_audioclkin_group_names[] = {
		"audio_clk_in"
};
unsigned int pistachio_group_audioclkin_pins[] = {
		36
};
unsigned int pistachio_group_audioclkin_select[] = {
		1
};

const char *pistachio_i2sout_group_names[] = {
		"i2sout"
};
unsigned int pistachio_group_i2sout_pins[] = {
		37, 38, 39
};
unsigned int pistachio_group_i2sout_select[] = {
		0, 0, 0
};
const char *pistachio_i2sout_mclk_group_names[] = {
		"i2sout_mclk"
};
unsigned int pistachio_group_i2sout_mclk_pins[] = {
		36
};
unsigned int pistachio_group_i2sout_mclk_select[] = {
		0
};
const char *pistachio_i2sout_dat1_group_names[] = {
		"i2sout_dat1"
};
unsigned int pistachio_group_i2sout_dat1_pins[] = {
		40
};
unsigned int pistachio_group_i2sout_dat1_select[] = {
		0
};
const char *pistachio_i2sout_dat2_group_names[] = {
		"i2sout_dat2"
};
unsigned int pistachio_group_i2sout_dat2_pins[] = {
		41
};
unsigned int pistachio_group_i2sout_dat2_select[] = {
		0
};
const char *pistachio_i2sout_dat3_group_names[] = {
		"i2sout_dat3"
};
unsigned int pistachio_group_i2sout_dat3_pins[] = {
		42
};
unsigned int pistachio_group_i2sout_dat3_select[] = {
		0
};
const char *pistachio_i2sout_dat4_group_names[] = {
		"i2sout_dat4"
};
unsigned int pistachio_group_i2sout_dat4_pins[] = {
		43
};
unsigned int pistachio_group_i2sout_dat4_select[] = {
		0
};
const char *pistachio_i2sout_dat5_group_names[] = {
		"i2sout_dat5"
};
unsigned int pistachio_group_i2sout_dat5_pins[] = {
		44
};
unsigned int pistachio_group_i2sout_dat5_select[] = {
		0
};

const char *pistachio_i2sdac_group_names[] = {
		"i2sdac"
};
unsigned int pistachio_group_i2sdac_pins[] = {
		45
};
unsigned int pistachio_group_i2sdac_select[] = {
		0
};

const char *pistachio_audiosync_group_names[] = {
		"audiosync"
};
unsigned int pistachio_group_audiosync_pins[] = {
		45
};
unsigned int pistachio_group_audiosync_select[] = {
		1
};

const char *pistachio_audiotrigger_group_names[] = {
		"audiotrigger"
};
unsigned int pistachio_group_audiotrigger_pins[] = {
		46
};
unsigned int pistachio_group_audiotrigger_select[] = {
		0
};

const char *pistachio_i2sin_group_names[] = {
		"i2sin"
};
unsigned int pistachio_group_i2sin_pins[] = {
		47, 48, 49
};
unsigned int pistachio_group_i2sin_select[] = {
		0, 0, 0
};
const char *pistachio_i2sin_dat1_group_names[] = {
		"i2sin_dat1"
};
unsigned int pistachio_group_i2sin_dat1_pins[] = {
		50
};
unsigned int pistachio_group_i2sin_dat1_select[] = {
		0
};
const char *pistachio_i2sin_dat2_group_names[] = {
		"i2sin_dat2"
};
unsigned int pistachio_group_i2sin_dat2_pins[] = {
		51
};
unsigned int pistachio_group_i2sin_dat2_select[] = {
		0
};
const char *pistachio_i2sin_dat3_group_names[] = {
		"i2sin_dat3"
};
unsigned int pistachio_group_i2sin_dat3_pins[] = {
		52
};
unsigned int pistachio_group_i2sin_dat3_select[] = {
		0
};
const char *pistachio_i2sin_dat4_group_names[] = {
		"i2sin_dat4"
};
unsigned int pistachio_group_i2sin_dat4_pins[] = {
		53
};
unsigned int pistachio_group_i2sin_dat4_select[] = {
		0
};
const char *pistachio_i2sin_dat5_group_names[] = {
		"i2sin_dat5"
};
unsigned int pistachio_group_i2sin_dat5_pins[] = {
		54
};
unsigned int pistachio_group_i2sin_dat5_select[] = {
		0
};

const char *pistachio_uart0_group_names[] = {
		"uart0"
};
unsigned int pistachio_group_uart0_pins[] = {
		55, 56
};
unsigned int pistachio_group_uart0_select[] = {
		0, 0
};
const char *pistachio_uart0_cts_rts_group_names[] = {
		"uart0_cts_rts"
};
unsigned int pistachio_group_uart0_cts_rts_pins[] = {
		57, 58
};
unsigned int pistachio_group_uart0_cts_rts_select[] = {
		0, 0
};

const char *pistachio_uart1_group_names[] = {
		"uart1"
};
unsigned int pistachio_group_uart1_pins[] = {
		59, 60
};
unsigned int pistachio_group_uart1_select[] = {
		0, 0
};
const char *pistachio_uart1_cts_rts_group_names[] = {
		"uart1_cts_rts"
};
unsigned int pistachio_group_uart1_cts_rts_pins[] = {
		1, 2
};
unsigned int pistachio_group_uart1_cts_rts_select[] = {
		2, 2
};

const char *pistachio_spdifout_group_names[] = {
		"spdifout"
};
unsigned int pistachio_group_spdifout_pins[] = {
		61
};
unsigned int pistachio_group_spdifout_select[] = {
		0
};

const char *pistachio_spdifin0_group_names[] = {
		"spdifin_0", "spdifin_1"
};
unsigned int pistachio_group_spdifin_0_pins[] = {
		62
};
unsigned int pistachio_group_spdifin_0_select[] = {
		0
};
unsigned int pistachio_group_spdifin_1_pins[] = {
		54
};
unsigned int pistachio_group_spdifin_1_select[] = {
		2
};

const char *pistachio_eth_group_names[] = {
		"eth",
};
unsigned int pistachio_group_eth_pins[] = {
		63, 64, 65, 66, 67, 68, 69, 70, 71
};
unsigned int pistachio_group_eth_select[] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0
};

const char *pistachio_ir_group_names[] = {
		"ir",
};
unsigned int pistachio_group_ir_pins[] = {
		72
};
unsigned int pistachio_group_ir_select[] = {
		0
};

const char *pistachio_pwmpdm0_group_names[] = {
		"pwmpdm0"
};
unsigned int pistachio_group_pwmpdm0_pins[] = {
		73
};
unsigned int pistachio_group_pwmpdm0_select[] = {
		0
};
const char *pistachio_pwmpdm1_group_names[] = {
		"pwmpdm1"
};
unsigned int pistachio_group_pwmpdm1_pins[] = {
		74
};
unsigned int pistachio_group_pwmpdm1_select[] = {
		0
};
const char *pistachio_pwmpdm2_group_names[] = {
		"pwmpdm2"
};
unsigned int pistachio_group_pwmpdm2_pins[] = {
		75
};
unsigned int pistachio_group_pwmpdm2_select[] = {
		0
};
const char *pistachio_pwmpdm3_group_names[] = {
		"pwmpdm3"
};
unsigned int pistachio_group_pwmpdm3_pins[] = {
		76
};
unsigned int pistachio_group_pwmpdm3_select[] = {
		0
};

const char *pistachio_mips_trace_clk_group_names[] = {
		"mips_trace_clk_0", "mips_trace_clk_1", "mips_trace_clk_2"
};
unsigned int pistachio_group_mips_trace_clk_0_pins[] = {
		15
};
unsigned int pistachio_group_mips_trace_clk_0_select[] = {
		1
};
unsigned int pistachio_group_mips_trace_clk_1_pins[] = {
		63
};
unsigned int pistachio_group_mips_trace_clk_1_select[] = {
		1
};
unsigned int pistachio_group_mips_trace_clk_2_pins[] = {
		73
};
unsigned int pistachio_group_mips_trace_clk_2_select[] = {
		1
};

const char *pistachio_mips_trace_dint_group_names[] = {
		"mips_trace_dint_0", "mips_trace_dint_1", "mips_trace_dint_2"
};
unsigned int pistachio_group_mips_trace_dint_0_pins[] = {
		16
};
unsigned int pistachio_group_mips_trace_dint_0_select[] = {
		1
};
unsigned int pistachio_group_mips_trace_dint_1_pins[] = {
		64
};
unsigned int pistachio_group_mips_trace_dint_1_select[] = {
		1
};
unsigned int pistachio_group_mips_trace_dint_2_pins[] = {
		74
};
unsigned int pistachio_group_mips_trace_dint_2_select[] = {
		1
};

const char *pistachio_mips_trace_trigout_group_names[] = {
		"mips_trace_trigout_0", "mips_trace_trigout_1",
		"mips_trace_trigout_2"
};
unsigned int pistachio_group_mips_trace_trigout_0_pins[] = {
		17
};
unsigned int pistachio_group_mips_trace_trigout_0_select[] = {
		1
};
unsigned int pistachio_group_mips_trace_trigout_1_pins[] = {
		65
};
unsigned int pistachio_group_mips_trace_trigout_1_select[] = {
		1
};
unsigned int pistachio_group_mips_trace_trigout_2_pins[] = {
		75
};
unsigned int pistachio_group_mips_trace_trigout_2_select[] = {
		1
};

const char *pistachio_mips_trace_trigin_group_names[] = {
		"mips_trace_trigin_0", "mips_trace_trigin_1",
		"mips_trace_trigin_2"
};
unsigned int pistachio_group_mips_trace_trigin_0_pins[] = {
		18
};
unsigned int pistachio_group_mips_trace_trigin_0_select[] = {
		1
};
unsigned int pistachio_group_mips_trace_trigin_1_pins[] = {
		66
};
unsigned int pistachio_group_mips_trace_trigin_1_select[] = {
		1
};
unsigned int pistachio_group_mips_trace_trigin_2_pins[] = {
		76
};
unsigned int pistachio_group_mips_trace_trigin_2_select[] = {
		1
};

const char *pistachio_mips_trace_dm_group_names[] = {
		"mips_trace_dm_0", "mips_trace_dm_1", "mips_trace_dm_2"
};
unsigned int pistachio_group_mips_trace_dm_0_pins[] = {
		19
};
unsigned int pistachio_group_mips_trace_dm_0_select[] = {
		1
};
unsigned int pistachio_group_mips_trace_dm_1_pins[] = {
		67
};
unsigned int pistachio_group_mips_trace_dm_1_select[] = {
		1
};
unsigned int pistachio_group_mips_trace_dm_2_pins[] = {
		77
};
unsigned int pistachio_group_mips_trace_dm_2_select[] = {
		1
};

const char *pistachio_mips_probe_n_group_names[] = {
		"mips_probe_n_0", "mips_probe_n_1", "mips_probe_n_2"
};
unsigned int pistachio_group_mips_probe_n_0_pins[] = {
		20
};
unsigned int pistachio_group_mips_probe_n_0_select[] = {
		1
};
unsigned int pistachio_group_mips_probe_n_1_pins[] = {
		68
};
unsigned int pistachio_group_mips_probe_n_1_select[] = {
		1
};
unsigned int pistachio_group_mips_probe_n_2_pins[] = {
		78
};
unsigned int pistachio_group_mips_probe_n_2_select[] = {
		1
};

const char *pistachio_mips_trace_data_group_names[] = {
		"mips_trace_data_0", "mips_trace_data_1", "mips_trace_data_2"
};
unsigned int pistachio_group_mips_trace_data_0_pins[] = {
		15, 16, 17, 18, 19, 20, 21, 22
};
unsigned int pistachio_group_mips_trace_data_0_select[] = {
		2, 2, 2, 2, 2, 2, 2, 2
};
unsigned int pistachio_group_mips_trace_data_1_pins[] = {
		63, 64, 65, 66, 67, 68, 69, 70
};
unsigned int pistachio_group_mips_trace_data_1_select[] = {
		2, 2, 2, 2, 2, 2, 2, 2
};
unsigned int pistachio_group_mips_trace_data_2_pins[] = {
		79, 80, 81, 82, 83, 84, 85, 86
};
unsigned int pistachio_group_mips_trace_data_2_select[] = {
		1, 1, 1, 1, 1, 1, 1, 1
};

const char *pistachio_valid_enable0_group_names[] = {
		"valid_enable0"
};
unsigned int pistachio_group_valid_enable0_pins[] = {
		73, 74
};
unsigned int pistachio_group_valid_enable0_select[] = {
		2, 2
};

const char *pistachio_valid_enable1_group_names[] = {
		"valid_enable1"
};
unsigned int pistachio_group_valid_enable1_pins[] = {
		75, 76
};
unsigned int pistachio_group_valid_enable1_select[] = {
		2, 2
};

const char *pistachio_valid_enable2_group_names[] = {
		"valid_enable2"
};
unsigned int pistachio_group_valid_enable2_pins[] = {
		77, 78
};
unsigned int pistachio_group_valid_enable2_select[] = {
		2, 2
};

const char *pistachio_valid_enable3_group_names[] = {
		"valid_enable3"
};
unsigned int pistachio_group_valid_enable3_pins[] = {
		79, 80
};
unsigned int pistachio_group_valid_enable3_select[] = {
		2, 2
};

const char *pistachio_valid_enable4_group_names[] = {
		"valid_enable4"
};
unsigned int pistachio_group_valid_enable4_pins[] = {
		81, 82
};
unsigned int pistachio_group_valid_enable4_select[] = {
		2, 2
};

const char *pistachio_valid_enable5_group_names[] = {
		"valid_enable5"
};
unsigned int pistachio_group_valid_enable5_pins[] = {
		83, 84
};
unsigned int pistachio_group_valid_enable5_select[] = {
		2, 2
};

const char *pistachio_valid_enable6_group_names[] = {
		"valid_enable6"
};
unsigned int pistachio_group_valid_enable6_pins[] = {
		85, 86
};
unsigned int pistachio_group_valid_enable6_select[] = {
		2, 2
};

const char *pistachio_valid_enable7_group_names[] = {
		"valid_enable7"
};
unsigned int pistachio_group_valid_enable7_pins[] = {
		87, 88
};
unsigned int pistachio_group_valid_enable7_select[] = {
		2, 2
};

const char *pistachio_valid_enable8_group_names[] = {
		"valid_enable8"
};
unsigned int pistachio_group_valid_enable8_pins[] = {
		77, 78
};
unsigned int pistachio_group_valid_enable8_select[] = {
		0, 0
};

const char *pistachio_valid_enable9_group_names[] = {
		"valid_enable9"
};
unsigned int pistachio_group_valid_enable9_pins[] = {
		79, 80
};
unsigned int pistachio_group_valid_enable9_select[] = {
		0, 0
};

const char *pistachio_dreq0_group_names[] = {
		"dreq0"
};
unsigned int pistachio_group_dreq0_pins[] = {
		81
};
unsigned int pistachio_group_dreq0_select[] = {
		0
};

const char *pistachio_dreq1_group_names[] = {
		"dreq1"
};
unsigned int pistachio_group_dreq1_pins[] = {
		82
};
unsigned int pistachio_group_dreq1_select[] = {
		0
};

const char *pistachio_dreq2_group_names[] = {
		"dreq2"
};
unsigned int pistachio_group_dreq2_pins[] = {
		87
};
unsigned int pistachio_group_dreq2_select[] = {
		1
};

const char *pistachio_dreq3_group_names[] = {
		"dreq3"
};
unsigned int pistachio_group_dreq3_pins[] = {
		88
};
unsigned int pistachio_group_dreq3_select[] = {
		1
};

const char *pistachio_dreq4_group_names[] = {
		"dreq4"
};
unsigned int pistachio_group_dreq4_pins[] = {
		89
};
unsigned int pistachio_group_dreq4_select[] = {
		1
};

const char *pistachio_dreq5_group_names[] = {
		"dreq5"
};
unsigned int pistachio_group_dreq5_pins[] = {
		89
};
unsigned int pistachio_group_dreq5_select[] = {
		2
};

const char *pistachio_pll_lock0_group_names[] = {
		"pll_lock0"
};
unsigned int pistachio_group_pll_lock0_pins[] = {
		83
};
unsigned int pistachio_group_pll_lock0_select[] = {
		0
};

const char *pistachio_pll_lock1_group_names[] = {
		"pll_lock1"
};
unsigned int pistachio_group_pll_lock1_pins[] = {
		84
};
unsigned int pistachio_group_pll_lock1_select[] = {
		0
};

const char *pistachio_pll_lock2_group_names[] = {
		"pll_lock2"
};
unsigned int pistachio_group_pll_lock2_pins[] = {
		85
};
unsigned int pistachio_group_pll_lock2_select[] = {
		0
};

const char *pistachio_pll_lock3_group_names[] = {
		"pll_lock3"
};
unsigned int pistachio_group_pll_lock3_pins[] = {
		86
};
unsigned int pistachio_group_pll_lock3_select[] = {
		0
};

const char *pistachio_pll_lock4_group_names[] = {
		"pll_lock4"
};
unsigned int pistachio_group_pll_lock4_pins[] = {
		87
};
unsigned int pistachio_group_pll_lock4_select[] = {
		0
};

const char *pistachio_pll_lock5_group_names[] = {
		"pll_lock5"
};
unsigned int pistachio_group_pll_lock5_pins[] = {
		88
};
unsigned int pistachio_group_pll_lock5_select[] = {
		0
};

const char *pistachio_pll_lock6_group_names[] = {
		"pll_lock6"
};
unsigned int pistachio_group_pll_lock6_pins[] = {
		89
};
unsigned int pistachio_group_pll_lock6_select[] = {
		0
};

const char *pistachio_debug_raw_cca_ind_group_names[] = {
		"debug_raw_cca_ind"
};
unsigned int pistachio_group_debug_raw_cca_ind_pins[] = {
		37
};
unsigned int pistachio_group_debug_raw_cca_ind_select[] = {
		0
};

const char *pistachio_debug_ed_sec20_cca_ind_group_names[] = {
		"debug_ed_sec20_cca_ind"
};
unsigned int pistachio_group_debug_ed_sec20_cca_ind_pins[] = {
		38
};
unsigned int pistachio_group_debug_ed_sec20_cca_ind_select[] = {
		0
};

const char *pistachio_debug_ed_sec40_cca_ind_group_names[] = {
		"debug_ed_sec40_cca_ind"
};
unsigned int pistachio_group_debug_ed_sec40_cca_ind_pins[] = {
		39
};
unsigned int pistachio_group_debug_ed_sec40_cca_ind_select[] = {
		0
};

const char *pistachio_debug_agc_done0_group_names[] = {
		"debug_agc_done0"
};
unsigned int pistachio_group_debug_agc_done0_pins[] = {
		40
};
unsigned int pistachio_group_debug_agc_done0_select[] = {
		0
};

const char *pistachio_debug_agc_done1_group_names[] = {
		"debug_agc_done1"
};
unsigned int pistachio_group_debug_agc_done1_pins[] = {
		41
};
unsigned int pistachio_group_debug_agc_done1_select[] = {
		0
};

const char *pistachio_debug_ed_cca_ind_group_names[] = {
		"debug_ed_cca_ind"
};
unsigned int pistachio_group_debug_ed_cca_ind_pins[] = {
		42
};
unsigned int pistachio_group_debug_ed_cca_ind_select[] = {
		0
};

const char *pistachio_debug_s2l_done_group_names[] = {
		"debug_s2l_done"
};
unsigned int pistachio_group_debug_s2l_done_pins[] = {
		43
};
unsigned int pistachio_group_debug_s2l_done_select[] = {
		0
};

#define PISTACHIO_GROUP(name)					\
	{							\
		#name,						\
		pistachio_group_##name##_pins,			\
		ARRAY_SIZE(pistachio_group_##name##_pins),	\
		pistachio_group_##name##_select,		\
		0x00000000,					\
		0x00000000					\
	}

#define PISTACHIO_GROUP_SCENARIO(name, scenario_mask, scenario_val)	\
	{								\
		#name,							\
		pistachio_group_##name##_pins,				\
		ARRAY_SIZE(pistachio_group_##name##_pins),		\
		pistachio_group_##name##_select,			\
		scenario_mask,						\
		scenario_val						\
	}

struct img_pinctrl_group pistachio_groups[] = {
		PISTACHIO_GROUP(spim1),
		PISTACHIO_GROUP(spim1_quad),
		PISTACHIO_GROUP(spim1_cs0),
		PISTACHIO_GROUP(spim1_cs1_0),
		PISTACHIO_GROUP(spim1_cs1_1),
		PISTACHIO_GROUP(spim1_cs2_0),
		PISTACHIO_GROUP(spim1_cs2_1),
		PISTACHIO_GROUP(spim1_cs2_2),
		PISTACHIO_GROUP(spim1_cs3),
		PISTACHIO_GROUP(spim1_cs4),
		PISTACHIO_GROUP(spim0),
		PISTACHIO_GROUP(spim0_cs0),
		PISTACHIO_GROUP(spim0_cs1),
		PISTACHIO_GROUP(spim0_cs2_0),
		PISTACHIO_GROUP(spim0_cs2_1),
		PISTACHIO_GROUP(spim0_cs3_0),
		PISTACHIO_GROUP(spim0_cs3_1),
		PISTACHIO_GROUP(spim0_cs4_0),
		PISTACHIO_GROUP(spim0_cs4_1),
		PISTACHIO_GROUP(spis),
		PISTACHIO_GROUP(sdhost),
		PISTACHIO_GROUP(sdhost_dat1to3),
		PISTACHIO_GROUP(sdhost_dat4to7),
		PISTACHIO_GROUP(i2c0),
		PISTACHIO_GROUP(i2c1),
		PISTACHIO_GROUP(i2c2),
		PISTACHIO_GROUP(i2c3),
		PISTACHIO_GROUP(audioclkin),
		PISTACHIO_GROUP(i2sout),
		PISTACHIO_GROUP(i2sout_mclk),
		PISTACHIO_GROUP(i2sout_dat1),
		PISTACHIO_GROUP(i2sout_dat2),
		PISTACHIO_GROUP(i2sout_dat3),
		PISTACHIO_GROUP(i2sout_dat4),
		PISTACHIO_GROUP(i2sout_dat5),
		PISTACHIO_GROUP(i2sdac),
		PISTACHIO_GROUP(audiosync),
		PISTACHIO_GROUP(audiotrigger),
		PISTACHIO_GROUP(i2sin),
		PISTACHIO_GROUP(i2sin_dat1),
		PISTACHIO_GROUP(i2sin_dat2),
		PISTACHIO_GROUP(i2sin_dat3),
		PISTACHIO_GROUP(i2sin_dat4),
		PISTACHIO_GROUP(i2sin_dat5),
		PISTACHIO_GROUP(uart0),
		PISTACHIO_GROUP(uart0_cts_rts),
		PISTACHIO_GROUP(uart1),
		PISTACHIO_GROUP(uart1_cts_rts),
		PISTACHIO_GROUP(spdifout),
		PISTACHIO_GROUP_SCENARIO(spdifin_0, 0x1, 0x0),
		PISTACHIO_GROUP_SCENARIO(spdifin_1, 0x1, 0x1),
		PISTACHIO_GROUP(ir),
		PISTACHIO_GROUP(eth),
		PISTACHIO_GROUP(pwmpdm0),
		PISTACHIO_GROUP(pwmpdm1),
		PISTACHIO_GROUP(pwmpdm2),
		PISTACHIO_GROUP(pwmpdm3),
		PISTACHIO_GROUP(mips_trace_clk_0),
		PISTACHIO_GROUP(mips_trace_clk_1),
		PISTACHIO_GROUP(mips_trace_clk_2),
		PISTACHIO_GROUP_SCENARIO(mips_trace_dint_0, 0x6, 0x0),
		PISTACHIO_GROUP_SCENARIO(mips_trace_dint_1, 0x6, 0x2),
		PISTACHIO_GROUP_SCENARIO(mips_trace_dint_2, 0x6, 0x4),
		PISTACHIO_GROUP(mips_trace_trigout_0),
		PISTACHIO_GROUP(mips_trace_trigout_1),
		PISTACHIO_GROUP(mips_trace_trigout_2),
		PISTACHIO_GROUP_SCENARIO(mips_trace_trigin_0, 0x18, 0x00),
		PISTACHIO_GROUP_SCENARIO(mips_trace_trigin_1, 0x18, 0x08),
		PISTACHIO_GROUP_SCENARIO(mips_trace_trigin_2, 0x18, 0x10),
		PISTACHIO_GROUP(mips_trace_dm_0),
		PISTACHIO_GROUP(mips_trace_dm_1),
		PISTACHIO_GROUP(mips_trace_dm_2),
		PISTACHIO_GROUP_SCENARIO(mips_probe_n_0, 0x60, 0x00),
		PISTACHIO_GROUP_SCENARIO(mips_probe_n_1, 0x60, 0x20),
		PISTACHIO_GROUP_SCENARIO(mips_probe_n_2, 0x60, 0x40),
		PISTACHIO_GROUP(mips_trace_data_0),
		PISTACHIO_GROUP(mips_trace_data_1),
		PISTACHIO_GROUP(mips_trace_data_2),
		PISTACHIO_GROUP(valid_enable0),
		PISTACHIO_GROUP(valid_enable1),
		PISTACHIO_GROUP(valid_enable2),
		PISTACHIO_GROUP(valid_enable3),
		PISTACHIO_GROUP(valid_enable4),
		PISTACHIO_GROUP(valid_enable5),
		PISTACHIO_GROUP(valid_enable6),
		PISTACHIO_GROUP(valid_enable7),
		PISTACHIO_GROUP(valid_enable8),
		PISTACHIO_GROUP(valid_enable9),
		PISTACHIO_GROUP(dreq0),
		PISTACHIO_GROUP(dreq1),
		PISTACHIO_GROUP(dreq2),
		PISTACHIO_GROUP(dreq3),
		PISTACHIO_GROUP(dreq4),
		PISTACHIO_GROUP(dreq5),
		PISTACHIO_GROUP(pll_lock0),
		PISTACHIO_GROUP(pll_lock1),
		PISTACHIO_GROUP(pll_lock2),
		PISTACHIO_GROUP(pll_lock3),
		PISTACHIO_GROUP(pll_lock4),
		PISTACHIO_GROUP(pll_lock5),
		PISTACHIO_GROUP(pll_lock6),
		PISTACHIO_GROUP(debug_raw_cca_ind),
		PISTACHIO_GROUP(debug_ed_sec20_cca_ind),
		PISTACHIO_GROUP(debug_ed_sec40_cca_ind),
		PISTACHIO_GROUP(debug_agc_done0),
		PISTACHIO_GROUP(debug_agc_done1),
		PISTACHIO_GROUP(debug_ed_cca_ind),
		PISTACHIO_GROUP(debug_s2l_done),
};

#define PISTACHIO_FUNCTION(name)				\
	{							\
		#name,						\
		pistachio_##name##_group_names,			\
		ARRAY_SIZE(pistachio_##name##_group_names)	\
	}

struct img_pinctrl_function pistachio_functions[] = {
		PISTACHIO_FUNCTION(spim1),
		PISTACHIO_FUNCTION(spim1_quad),
		PISTACHIO_FUNCTION(spim1_cs0),
		PISTACHIO_FUNCTION(spim1_cs1),
		PISTACHIO_FUNCTION(spim1_cs2),
		PISTACHIO_FUNCTION(spim1_cs3),
		PISTACHIO_FUNCTION(spim1_cs4),
		PISTACHIO_FUNCTION(spim0),
		PISTACHIO_FUNCTION(spim0_cs0),
		PISTACHIO_FUNCTION(spim0_cs1),
		PISTACHIO_FUNCTION(spim0_cs2),
		PISTACHIO_FUNCTION(spim0_cs3),
		PISTACHIO_FUNCTION(spim0_cs4),
		PISTACHIO_FUNCTION(spis),
		PISTACHIO_FUNCTION(sdhost),
		PISTACHIO_FUNCTION(sdhost_dat1to3),
		PISTACHIO_FUNCTION(sdhost_dat4to7),
		PISTACHIO_FUNCTION(i2c0),
		PISTACHIO_FUNCTION(i2c1),
		PISTACHIO_FUNCTION(i2c2),
		PISTACHIO_FUNCTION(i2c3),
		PISTACHIO_FUNCTION(audioclkin),
		PISTACHIO_FUNCTION(i2sout),
		PISTACHIO_FUNCTION(i2sout_mclk),
		PISTACHIO_FUNCTION(i2sout_dat1),
		PISTACHIO_FUNCTION(i2sout_dat2),
		PISTACHIO_FUNCTION(i2sout_dat3),
		PISTACHIO_FUNCTION(i2sout_dat4),
		PISTACHIO_FUNCTION(i2sout_dat5),
		PISTACHIO_FUNCTION(i2sdac),
		PISTACHIO_FUNCTION(audiosync),
		PISTACHIO_FUNCTION(audiotrigger),
		PISTACHIO_FUNCTION(i2sin),
		PISTACHIO_FUNCTION(i2sin_dat1),
		PISTACHIO_FUNCTION(i2sin_dat2),
		PISTACHIO_FUNCTION(i2sin_dat3),
		PISTACHIO_FUNCTION(i2sin_dat4),
		PISTACHIO_FUNCTION(i2sin_dat5),
		PISTACHIO_FUNCTION(uart0),
		PISTACHIO_FUNCTION(uart0_cts_rts),
		PISTACHIO_FUNCTION(uart1),
		PISTACHIO_FUNCTION(uart1_cts_rts),
		PISTACHIO_FUNCTION(spdifout),
		PISTACHIO_FUNCTION(spdifin0),
		PISTACHIO_FUNCTION(ir),
		PISTACHIO_FUNCTION(eth),
		PISTACHIO_FUNCTION(pwmpdm0),
		PISTACHIO_FUNCTION(pwmpdm1),
		PISTACHIO_FUNCTION(pwmpdm2),
		PISTACHIO_FUNCTION(pwmpdm3),
		PISTACHIO_FUNCTION(mips_trace_clk),
		PISTACHIO_FUNCTION(mips_trace_dint),
		PISTACHIO_FUNCTION(mips_trace_trigout),
		PISTACHIO_FUNCTION(mips_trace_trigin),
		PISTACHIO_FUNCTION(mips_trace_dm),
		PISTACHIO_FUNCTION(mips_probe_n),
		PISTACHIO_FUNCTION(mips_trace_data),
		PISTACHIO_FUNCTION(valid_enable0),
		PISTACHIO_FUNCTION(valid_enable1),
		PISTACHIO_FUNCTION(valid_enable2),
		PISTACHIO_FUNCTION(valid_enable3),
		PISTACHIO_FUNCTION(valid_enable4),
		PISTACHIO_FUNCTION(valid_enable5),
		PISTACHIO_FUNCTION(valid_enable6),
		PISTACHIO_FUNCTION(valid_enable7),
		PISTACHIO_FUNCTION(valid_enable8),
		PISTACHIO_FUNCTION(valid_enable9),
		PISTACHIO_FUNCTION(dreq0),
		PISTACHIO_FUNCTION(dreq1),
		PISTACHIO_FUNCTION(dreq2),
		PISTACHIO_FUNCTION(dreq3),
		PISTACHIO_FUNCTION(dreq4),
		PISTACHIO_FUNCTION(dreq5),
		PISTACHIO_FUNCTION(pll_lock0),
		PISTACHIO_FUNCTION(pll_lock1),
		PISTACHIO_FUNCTION(pll_lock2),
		PISTACHIO_FUNCTION(pll_lock3),
		PISTACHIO_FUNCTION(pll_lock4),
		PISTACHIO_FUNCTION(pll_lock5),
		PISTACHIO_FUNCTION(pll_lock6),
		PISTACHIO_FUNCTION(debug_raw_cca_ind),
		PISTACHIO_FUNCTION(debug_ed_sec20_cca_ind),
		PISTACHIO_FUNCTION(debug_ed_sec40_cca_ind),
		PISTACHIO_FUNCTION(debug_agc_done0),
		PISTACHIO_FUNCTION(debug_agc_done1),
		PISTACHIO_FUNCTION(debug_ed_cca_ind),
		PISTACHIO_FUNCTION(debug_s2l_done),
};

struct img_gpio_irqdata pistachio_gpio_irq[DIV_ROUND_UP(PISTACHIO_GPIOS, 16)];

static const struct img_pinctrl_soc_data pistachio_pinctrl = {
	.pins = pistachio_pins,
	.num_pins = PISTACHIO_PINS,
	.num_gpios = PISTACHIO_GPIOS,
	.num_mfios = PISTACHIO_MFIOS,
	.functions = pistachio_functions,
	.num_functions = ARRAY_SIZE(pistachio_functions),
	.groups = pistachio_groups,
	.num_groups = ARRAY_SIZE(pistachio_groups),
	.gpio_irq_data = pistachio_gpio_irq,
	.num_gpio_banks = ARRAY_SIZE(pistachio_gpio_irq),
	.mfio_reg_ctrl = pistachio_mfio_reg_ctrl
};

static int pistachio_pinctrl_probe(struct platform_device *pdev)
{
	return img_pinctrl_probe(pdev, &pistachio_pinctrl);
}

static struct of_device_id pistachio_pinctrl_match[] = {
	{ .compatible = "img,pistachio-pinctrl" },
	{}
};
MODULE_DEVICE_TABLE(of, pistachio_pinctrl_match);

static struct platform_driver pistachio_pinctrl_driver = {
	.probe = pistachio_pinctrl_probe,
	.remove = img_pinctrl_remove,
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = pistachio_pinctrl_match
	}
};
module_platform_driver(pistachio_pinctrl_driver);

MODULE_AUTHOR("Damien Horsley <Damien.Horsley@imgtec.com>");
MODULE_DESCRIPTION("Pistachio pinctrl driver");
MODULE_LICENSE("GPL v2");
