/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 */

#ifndef __ROCKCHIP_PSCI_DDR_H
#define __ROCKCHIP_PSCI_DDR_H

/*
 * define PSCI_SIP_DRAM_FREQ_CONFIG call type
 */
#define DRAM_FREQ_CONFIG_DRAM_INIT		(0)
#define DRAM_FREQ_CONFIG_DRAM_FREQ_CHANGE	(1)
#define DRAM_FREQ_CONFIG_DRAM_ROUND_RATE	(2)
#define DRAM_FREQ_CONFIG_DRAM_SET_AT_SR		(3)
#define DRAM_FREQ_CONFIG_DRAM_GET_BW		(4)
#define DRAM_FREQ_CONFIG_DRAM_GET_RATE		(5)

uint32_t psci_ddr_init(uint32_t freq, void *dram_param);
uint32_t psci_ddr_change_freq(uint32_t freq);
uint32_t psci_ddr_round_rate(uint32_t n_mhz);
uint32_t psci_ddr_recalc_rate(void);
void psci_ddr_set_auto_self_refresh(bool en);
void psci_ddr_bandwidth_get(void *ddr_bw_ch0, void *ddr_bw_ch1);

#endif /* __ROCKCHIP_PSCI_DDR_H */
