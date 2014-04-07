/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2013 - 2014 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <ilw@linux.intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2013 - 2014 Intel Corporation. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
#if !defined(__IWL_DBG_CFG_H__) || defined(DBG_CFG_REINCLUDE)
#undef __IWL_DBG_CFG_H__ /* avoid warning */
#define __IWL_DBG_CFG_H__
/*
 * with DBG_CFG_REINCLUDE set this file should contain nothing
 * but IWL_DBG_CFG() macro invocations so it can be used in
 * the C file to generate more code (e.g. for debugfs and the
 * struct initialization with default values)
 */
#ifndef DBG_CFG_REINCLUDE
#include <linux/types.h>
#include <linux/debugfs.h>

struct iwl_dbg_cfg_bin {
	const void *data;
	unsigned int len;
};

#define DBG_CFG_bool	u32 /* for debugfs */
#define DBG_CFG_u8	u8
#define DBG_CFG_u16	u16
#define DBG_CFG_u32	u32
struct iwl_dbg_cfg {
#define IWL_DBG_CFG(type, name)		DBG_CFG_##type name;
#define IWL_DBG_CFG_NODEF(type, name)	DBG_CFG_##type name;
#define IWL_DBG_CFG_BIN(name)		struct iwl_dbg_cfg_bin name;
#define IWL_DBG_CFG_BINA(name, max)	struct iwl_dbg_cfg_bin name[max]; \
					int n_ ## name;
#endif /* DBG_CFG_REINCLUDE */
#if IS_ENABLED(CPTCFG_IWLXVT)
	IWL_DBG_CFG(u32, XVT_DEFAULT_DBGM_MEM_POWER)
	IWL_DBG_CFG(u32, XVT_DEFAULT_DBGM_LMAC_MASK)
	IWL_DBG_CFG(u32, XVT_DEFAULT_DBGM_PRPH_MASK)
#endif
#if IS_ENABLED(CPTCFG_IWLMVM)
	IWL_DBG_CFG(u32, MVM_DEFAULT_PS_TX_DATA_TIMEOUT)
	IWL_DBG_CFG(u32, MVM_DEFAULT_PS_RX_DATA_TIMEOUT)
	IWL_DBG_CFG(u32, MVM_WOWLAN_PS_TX_DATA_TIMEOUT)
	IWL_DBG_CFG(u32, MVM_WOWLAN_PS_RX_DATA_TIMEOUT)
	IWL_DBG_CFG(u32, MVM_UAPSD_TX_DATA_TIMEOUT)
	IWL_DBG_CFG(u32, MVM_UAPSD_RX_DATA_TIMEOUT)
	IWL_DBG_CFG_NODEF(u32, MVM_CALIB_OVERRIDE_CONTROL)
	IWL_DBG_CFG_NODEF(u32, MVM_CALIB_INIT_FLOW)
	IWL_DBG_CFG_NODEF(u32, MVM_CALIB_INIT_EVENT)
	IWL_DBG_CFG_NODEF(u32, MVM_CALIB_D0_FLOW)
	IWL_DBG_CFG_NODEF(u32, MVM_CALIB_D0_EVENT)
	IWL_DBG_CFG_NODEF(u32, MVM_CALIB_D3_FLOW)
	IWL_DBG_CFG_NODEF(u32, MVM_CALIB_D3_EVENT)
	IWL_DBG_CFG_NODEF(u32, MVM_WD_TIMEOUT)
	IWL_DBG_CFG(u8, MVM_PS_HEAVY_TX_THLD_PACKETS)
	IWL_DBG_CFG(u8, MVM_PS_HEAVY_RX_THLD_PACKETS)
	IWL_DBG_CFG(u8, MVM_PS_SNOOZE_HEAVY_TX_THLD_PACKETS)
	IWL_DBG_CFG(u8, MVM_PS_SNOOZE_HEAVY_RX_THLD_PACKETS)
	IWL_DBG_CFG(u8, MVM_PS_HEAVY_TX_THLD_PERCENT)
	IWL_DBG_CFG(u8, MVM_PS_HEAVY_RX_THLD_PERCENT)
	IWL_DBG_CFG(u16, MVM_PS_SNOOZE_INTERVAL)
	IWL_DBG_CFG(u16, MVM_PS_SNOOZE_WINDOW)
	IWL_DBG_CFG(u16, MVM_WOWLAN_PS_SNOOZE_WINDOW)
	IWL_DBG_CFG(u8, MVM_LOWLAT_QUOTA_MIN_PERCENT)
	IWL_DBG_CFG(bool, MVM_BT_COEX_SYNC2SCO)
	IWL_DBG_CFG(bool, MVM_BT_COEX_CORUNNING)
	IWL_DBG_CFG(bool, MVM_BT_COEX_MPLUT)
#ifdef CPTCFG_IWLMVM_TCM
	IWL_DBG_CFG(u32, MVM_TCM_LOAD_MEDIUM_THRESH)
	IWL_DBG_CFG(u32, MVM_TCM_LOAD_HIGH_THRESH)
	IWL_DBG_CFG(u32, MVM_TCM_LOWLAT_ENABLE_THRESH)
	IWL_DBG_CFG(u32, MVM_QUOTA_AIRTIME_THRESH)
	IWL_DBG_CFG(u8, MVM_LOWLAT_QUOTA_LOWTRAF_PERCENT)
#endif /* CPTCFG_IWLMVM_TCM */
#endif /* CPTCFG_IWLMVM */
#ifdef CPTCFG_IWLWIFI_DEVICE_TESTMODE
	IWL_DBG_CFG_NODEF(u32, dnt_out_mode)
	/* XXX: should be dbgm_ or dbg_mon_ for consistency? */
	IWL_DBG_CFG_NODEF(u32, dbm_destination_path)
	/* XXX: should be dbg_mon_ for consistency? */
	IWL_DBG_CFG_NODEF(u32, dbgm_enable_mode)
	IWL_DBG_CFG_NODEF(u32, dbgm_mem_power)
	IWL_DBG_CFG_NODEF(u32, dbg_flags)
	IWL_DBG_CFG_NODEF(bool, d0_is_usniffer)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_sample_ctl_addr)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_sample_ctl_val)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_buff_base_addr_reg_addr)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_buff_end_addr_reg_addr)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_data_sel_ctl_addr)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_data_sel_ctl_val)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_mc_msk_addr)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_mc_msk_val)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_sample_mask_addr)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_sample_mask_val)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_start_mask_addr)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_start_mask_val)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_end_mask_addr)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_end_mask_val)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_end_threshold_addr)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_end_threshold_val)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_sample_period_addr)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_sample_period_val)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_wr_ptr_addr)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_cyc_cnt_addr)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_dmarb_rd_ctl_addr)
	IWL_DBG_CFG_NODEF(u32, dbg_mon_dmarb_rd_data_addr)
	IWL_DBG_CFG_NODEF(u32, dbg_marbh_conf_reg)
	IWL_DBG_CFG_NODEF(u32, dbg_marbh_conf_mask)
	IWL_DBG_CFG_NODEF(u32, dbg_marbh_access_type)
	IWL_DBG_CFG_NODEF(u32, dbgc_hb_base_addr)
	IWL_DBG_CFG_NODEF(u32, dbgc_hb_end_addr)
	IWL_DBG_CFG_NODEF(u32, dbgc_dram_wrptr_addr)
	IWL_DBG_CFG_NODEF(u32, dbgc_hb_base_val_mipi)
	IWL_DBG_CFG_NODEF(u32, dbgc_hb_end_val_mipi)
	IWL_DBG_CFG_NODEF(u32, dbgc_hb_base_val_smem)
	IWL_DBG_CFG_NODEF(u32, dbgc_hb_end_val_smem)
	IWL_DBG_CFG_BIN(dbg_conf_monitor_host_command)
	IWL_DBG_CFG_BIN(log_level_cmd)
	IWL_DBG_CFG_BINA(ldbg_cmd, 32)
	IWL_DBG_CFG_NODEF(u8, log_level_cmd_id)
	IWL_DBG_CFG_NODEF(u8, dbg_conf_monitor_cmd_id)
	IWL_DBG_CFG_NODEF(u8, ldbg_cmd_nums)
#endif /* CPTCFG_IWLWIFI_DEVICE_TESTMODE */
#undef IWL_DBG_CFG
#undef IWL_DBG_CFG_NODEF
#undef IWL_DBG_CFG_BIN
#undef IWL_DBG_CFG_BINA
#ifndef DBG_CFG_REINCLUDE
};

extern struct iwl_dbg_cfg current_dbg_config;
void iwl_dbg_cfg_init_dbgfs(struct dentry *root);
void iwl_dbg_cfg_free(struct iwl_dbg_cfg *dbgcfg);
void iwl_dbg_cfg_load_ini(struct device *dev, struct iwl_dbg_cfg *dbgcfg);
#endif /* DBG_CFG_REINCLUDE */

#endif /* __IWL_DBG_CFG_H__ || DBG_CFG_REINCLUDE */
