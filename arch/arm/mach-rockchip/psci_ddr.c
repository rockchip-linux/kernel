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

#include <linux/kernel.h>
#include <linux/rockchip/psci.h>
#include <linux/rockchip/psci_ddr.h>

uint32_t psci_ddr_init(uint32_t freq, void *dram_param)
{
	return rockchip_psci_smc_write(PSCI_SIP_DRAM_FREQ_CONFIG, freq,
				       (uint32_t)dram_param,
				       DRAM_FREQ_CONFIG_DRAM_INIT);
}

uint32_t psci_ddr_change_freq(uint32_t freq)
{
	return rockchip_psci_smc_write(PSCI_SIP_DRAM_FREQ_CONFIG, freq, 0,
				       DRAM_FREQ_CONFIG_DRAM_FREQ_CHANGE);
}

uint32_t psci_ddr_round_rate(uint32_t n_mhz)
{
	return rockchip_psci_smc_write(PSCI_SIP_DRAM_FREQ_CONFIG, n_mhz, 0,
				       DRAM_FREQ_CONFIG_DRAM_ROUND_RATE);
}

void psci_ddr_set_auto_self_refresh(bool en)
{
	rockchip_psci_smc_write(PSCI_SIP_DRAM_FREQ_CONFIG, en, 0,
				DRAM_FREQ_CONFIG_DRAM_SET_AT_SR);
}

void psci_ddr_bandwidth_get(void *ddr_bw_ch0, void *ddr_bw_ch1)
{
	rockchip_psci_smc_write(PSCI_SIP_DRAM_FREQ_CONFIG,
				(uint32_t)ddr_bw_ch0, (uint32_t)ddr_bw_ch1,
				DRAM_FREQ_CONFIG_DRAM_GET_BW);
}

uint32_t psci_ddr_recalc_rate(void)
{
	return rockchip_psci_smc_write(PSCI_SIP_DRAM_FREQ_CONFIG, 0, 0,
				       DRAM_FREQ_CONFIG_DRAM_GET_RATE);
}
