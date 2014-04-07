/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2014 Intel Corporation. All rights reserved.
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
 * Copyright(c) 2014 Intel Corporation. All rights reserved.
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
#include <linux/types.h>
#include <linux/export.h>

#include "iwl-debug.h"
#include "iwl-io.h"
#include "iwl-trans.h"
#include "iwl-tm-gnl.h"
#include "iwl-dnt-cfg.h"
#include "iwl-dnt-dev-if.h"

static void iwl_dnt_dev_if_configure_mipi(struct iwl_trans *trans)
{
	/* ABB_CguDTClkCtrl - set system trace and mtm clock souce as PLLA */
	iowrite32(0x30303, (void __iomem *)0xe640110c);

	/* ABB_SpcuMemPower - set the power of the trace memory */
	iowrite32(0x1, (void __iomem *)0xe640201c);

	/* set MIPI2 PCL, PCL_26 - PCL_30 */
	iowrite32(0x10, (void __iomem *)0xe6300274);
	iowrite32(0x10, (void __iomem *)0xe6300278);
	iowrite32(0x10, (void __iomem *)0xe630027c);
	iowrite32(0x10, (void __iomem *)0xe6300280);
	iowrite32(0x10, (void __iomem *)0xe6300284);

	/* ARB0_CNF - enable generic arbiter */
	iowrite32(0xc0000000, (void __iomem *)0xe6700108);

	/* enable WLAN arbiter */
	iowrite32(0x80000006, (void __iomem *)0xe6700140);

#ifdef IWL_MIPI_IDI
	/* enable IDI arbiter for all channels - this code is
	 * needed in case we'd like to look on IDI bus logs
	 * via MIPI
	 */
	iowrite32(0xB0000004, (void __iomem *)0xe6700124);
	iowrite32(0xC0000000, (void __iomem *)0xe6700128);
#endif
}

static void iwl_dnt_dev_if_configure_marbh(struct iwl_trans *trans)
{
	struct iwl_dbg_cfg *cfg = &trans->dbg_cfg;
	u32 ret, reg_val = 0;

	if (cfg->dbg_marbh_access_type == ACCESS_TYPE_DIRECT) {
		iwl_trans_set_bits_mask(trans, cfg->dbg_marbh_conf_reg,
					cfg->dbg_marbh_conf_mask,
					cfg->dbg_marbh_conf_mask);
	} else if (cfg->dbg_marbh_access_type == ACCESS_TYPE_INDIRECT) {
		ret = iwl_trans_read_mem(trans, cfg->dbg_marbh_conf_reg,
					 &reg_val, 1);
		if (ret) {
			IWL_ERR(trans, "Failed to read MARBH conf reg\n");
			return;
		}
		reg_val |= cfg->dbg_marbh_conf_mask;
		ret = iwl_trans_write_mem(trans, cfg->dbg_marbh_conf_reg,
							 &reg_val, 1);
		if (ret) {
			IWL_ERR(trans, "Failed to write MARBH conf reg\n");
			return;
		}
	} else {
		IWL_ERR(trans, "Invalid MARBH access type\n");
	}
}

static void iwl_dnt_dev_if_configure_dbgc_registers(struct iwl_trans *trans,
						    u32 base_addr,
						    u32 end_addr)
{
	struct iwl_dbg_cfg *cfg = &trans->dbg_cfg;

	switch (trans->tmdev->dnt->cur_mon_type) {
	case MIPI:
		iwl_write_prph(trans, cfg->dbgc_hb_base_addr,
			       cfg->dbgc_hb_base_val_mipi);
		iwl_write_prph(trans, cfg->dbgc_hb_end_addr,
			       cfg->dbgc_hb_end_val_mipi);
		break;

	case SMEM:
		iwl_write_prph(trans, cfg->dbgc_hb_base_addr,
			       cfg->dbgc_hb_base_val_smem);
		iwl_write_prph(trans, cfg->dbgc_hb_end_addr,
			       cfg->dbgc_hb_end_val_smem);

		/*
		 * SMEM requires the same internal configuration as MARBH,
		 * which preceeded it.
		 */
		iwl_dnt_dev_if_configure_marbh(trans);
		break;

	case DMA:
	default:
		/*
		 * The given addresses are already shifted by 4 places so we
		 * need to shift by another 4
		 */
		iwl_write_prph(trans, cfg->dbgc_hb_base_addr, base_addr >> 4);
		iwl_write_prph(trans, cfg->dbgc_hb_end_addr, end_addr >> 4);
		break;
	};
}

static void iwl_dnt_dev_if_configure_dbgm_registers(struct iwl_trans *trans,
						    u32 base_addr,
						    u32 end_addr)
{
	struct iwl_dbg_cfg *cfg = &trans->dbg_cfg;

	/* If we're running a device that supports DBGC - use it */
	if (trans->cfg->device_family == IWL_DEVICE_FAMILY_8000) {
		iwl_dnt_dev_if_configure_dbgc_registers(trans, base_addr,
							end_addr);
		return;
	}

	/* configuring monitor */
	iwl_write_prph(trans, cfg->dbg_mon_buff_base_addr_reg_addr, base_addr);
	iwl_write_prph(trans, cfg->dbg_mon_buff_end_addr_reg_addr, end_addr);
	iwl_write_prph(trans, cfg->dbg_mon_data_sel_ctl_addr,
		       cfg->dbg_mon_data_sel_ctl_val);
	iwl_write_prph(trans, cfg->dbg_mon_mc_msk_addr,
		       cfg->dbg_mon_mc_msk_val);
	iwl_write_prph(trans, cfg->dbg_mon_sample_mask_addr,
		       cfg->dbg_mon_sample_mask_val);
	iwl_write_prph(trans, cfg->dbg_mon_start_mask_addr,
		       cfg->dbg_mon_start_mask_val);
	iwl_write_prph(trans, cfg->dbg_mon_end_threshold_addr,
		       cfg->dbg_mon_end_threshold_val);
	iwl_write_prph(trans, cfg->dbg_mon_end_mask_addr,
		       cfg->dbg_mon_end_mask_val);
	iwl_write_prph(trans, cfg->dbg_mon_sample_period_addr,
		       cfg->dbg_mon_sample_period_val);
	/* starting monitor */
	iwl_write_prph(trans, cfg->dbg_mon_sample_ctl_addr,
		       cfg->dbg_mon_sample_ctl_val);
}

static int iwl_dnt_dev_if_retrieve_dma_monitor_data(struct iwl_dnt *dnt,
						    struct iwl_trans *trans,
						    void *buffer,
						    u32 buffer_size)
{
	struct iwl_dbg_cfg *cfg = &trans->dbg_cfg;
	u32 wr_ptr;
	u8 *temp_buf = NULL;
	/* FIXME send stop command to FW */
	if (WARN_ON_ONCE(!dnt->mon_buf_cpu_addr)) {
		IWL_ERR(trans, "Can't retrieve data - DMA wasn't allocated\n");
		return -ENOMEM;
	}

	/* If we're running a device that supports DBGC - use it */
	if (trans->cfg->device_family == IWL_DEVICE_FAMILY_8000)
		wr_ptr = iwl_read_prph(trans, cfg->dbgc_dram_wrptr_addr);
	else
		wr_ptr = iwl_read_prph(trans, cfg->dbg_mon_wr_ptr_addr);
	/* iwl_read_prph returns 0x5a5a5a5a when it fails to grab nic access */
	if (wr_ptr == 0x5a5a5a5a) {
		IWL_ERR(trans, "Can't read write pointer\n");
		return -ENODEV;
	}

	/* If we're running a device that supports DBGC.... */
	if (trans->cfg->device_family == IWL_DEVICE_FAMILY_8000)
		wr_ptr = (wr_ptr - (dnt->mon_base_addr >> 6)) << 6;
	else
		wr_ptr = (wr_ptr << 4) - dnt->mon_base_addr;
	temp_buf = kmemdup(dnt->mon_buf_cpu_addr, dnt->mon_buf_size,
			   GFP_KERNEL);
	if (!temp_buf)
		return -ENOMEM;

	memcpy(buffer, temp_buf + wr_ptr, dnt->mon_buf_size - wr_ptr);
	memcpy(buffer + dnt->mon_buf_size - wr_ptr, temp_buf, wr_ptr);
	kfree(temp_buf);

	return dnt->mon_buf_size;
}

static int iwl_dnt_dev_if_retrieve_iccm_monitor_data(struct iwl_dnt *dnt,
						     struct iwl_trans *trans,
						     void *buffer,
						     u32 buffer_size)
{
	return 0;
}

static int iwl_dnt_dev_if_retrieve_marbh_monitor_data(struct iwl_dnt *dnt,
						      struct iwl_trans *trans,
						      u8 *buffer,
						      u32 buffer_size)
{
	struct iwl_dbg_cfg *cfg = &trans->dbg_cfg;
	int buf_size_in_dwords, buf_index, i;
	u32 wr_ptr, read_val;

	/* FIXME send stop command to FW */

	wr_ptr = iwl_read_prph(trans, cfg->dbg_mon_wr_ptr_addr);
	/* iwl_read_prph returns 0x5a5a5a5a when it fails to grab nic access */
	if (wr_ptr == 0x5a5a5a5a) {
		IWL_ERR(trans, "Can't read write pointer\n");
		return -ENODEV;
	}

	read_val = iwl_read_prph(trans, cfg->dbg_mon_buff_base_addr_reg_addr);
	if (read_val == 0x5a5a5a5a) {
		IWL_ERR(trans, "Can't read monitor base address\n");
		return -ENODEV;
	}
	dnt->mon_base_addr = read_val;

	read_val = iwl_read_prph(trans, cfg->dbg_mon_buff_end_addr_reg_addr);
	if (read_val == 0x5a5a5a5a) {
		IWL_ERR(trans, "Can't read monitor end address\n");
		return -ENODEV;
	}
	dnt->mon_end_addr = read_val;

	wr_ptr = wr_ptr - dnt->mon_base_addr;
	iwl_write_prph(trans, cfg->dbg_mon_dmarb_rd_ctl_addr, 0x00000001);

	/* buf size includes the end_addr as well */
	buf_size_in_dwords = dnt->mon_end_addr - dnt->mon_base_addr + 1;
	for (i = 0; i < buf_size_in_dwords; i++) {
		/* reordering cyclic buffer */
		buf_index = (i + (buf_size_in_dwords - wr_ptr)) %
			    buf_size_in_dwords;
		read_val = iwl_read_prph(trans,
					 cfg->dbg_mon_dmarb_rd_data_addr);
		memcpy(&buffer[buf_index * sizeof(u32)], &read_val,
		       sizeof(u32));
	}
	iwl_write_prph(trans, cfg->dbg_mon_dmarb_rd_ctl_addr, 0x00000000);

	return buf_size_in_dwords * sizeof(u32);
}

int iwl_dnt_dev_if_configure_monitor(struct iwl_dnt *dnt,
				     struct iwl_trans *trans)
{
	u32 base_addr, end_addr;

	switch (dnt->cur_mon_type) {
	case NO_MONITOR:
		IWL_INFO(trans, "Monitor is disabled\n");
		dnt->iwl_dnt_status &= ~IWL_DNT_STATUS_MON_CONFIGURED;
		break;
	case MARBH:
		iwl_dnt_dev_if_configure_marbh(trans);
		break;
	case DMA:
		if (!dnt->mon_buf_cpu_addr) {
			IWL_ERR(trans,
				"Can't configure DMA monitor: no cpu addr\n");
			return -ENOMEM;
		}
		base_addr = dnt->mon_base_addr >> 4;
		end_addr = dnt->mon_end_addr >> 4;
		iwl_dnt_dev_if_configure_dbgm_registers(trans, base_addr,
							end_addr);
		break;
	case MIPI:
		base_addr = 0;
		end_addr = 0;

		/* If not working with DBGC... */
		if (trans->cfg->device_family != IWL_DEVICE_FAMILY_8000) {
			iwl_dnt_dev_if_configure_mipi(trans);
			break;
		}
	case SMEM:
		iwl_dnt_dev_if_configure_dbgm_registers(trans, base_addr,
							end_addr);
		break;
	case INTERFACE:
		base_addr = 0;
		end_addr = 0x400;
		iwl_dnt_dev_if_configure_dbgm_registers(trans, base_addr,
							end_addr);
		break;
	case ICCM:
	default:
		dnt->iwl_dnt_status &= ~IWL_DNT_STATUS_MON_CONFIGURED;
		IWL_INFO(trans, "Invalid monitor type\n");
		return -EINVAL;
	}


	dnt->iwl_dnt_status |= IWL_DNT_STATUS_MON_CONFIGURED;

	return 0;
}

static int iwl_dnt_dev_if_send_dbgm(struct iwl_dnt *dnt,
				    struct iwl_trans *trans)
{
	struct iwl_dbg_cfg *cfg = &trans->dbg_cfg;
	struct iwl_host_cmd host_cmd = {
		.id = cfg->dbg_conf_monitor_cmd_id,
		.data[0] = cfg->dbg_conf_monitor_host_command.data,
		.len[0] = cfg->dbg_conf_monitor_host_command.len,
		.dataflags[0] = IWL_HCMD_DFL_NOCOPY,
		.flags = CMD_SYNC | CMD_WANT_SKB,
	};
	int ret;

	ret = iwl_trans_send_cmd(trans, &host_cmd);
	if (ret) {
		IWL_ERR(trans, "Failed to send monitor command\n");
		dnt->iwl_dnt_status |= IWL_DNT_STATUS_FAILED_START_MONITOR;
	}

	return ret;
}

static int iwl_dnt_dev_if_send_ldbg(struct iwl_dnt *dnt,
				    struct iwl_trans *trans,
				    int cmd_index)
{
	struct iwl_dbg_cfg *cfg = &trans->dbg_cfg;
	struct iwl_host_cmd host_cmd = {
		.id = cfg->dbg_conf_monitor_cmd_id,
		.data[0] = cfg->ldbg_cmd[cmd_index].data,
		.len[0] = DNT_LDBG_CMD_SIZE,
		.dataflags[0] = IWL_HCMD_DFL_NOCOPY,
		.flags = CMD_SYNC | CMD_WANT_SKB,
	};


	return iwl_trans_send_cmd(trans, &host_cmd);
}

int iwl_dnt_dev_if_start_monitor(struct iwl_dnt *dnt,
				 struct iwl_trans *trans)
{
	struct iwl_dbg_cfg *cfg = &trans->dbg_cfg;
	int i, ret;

	switch (cfg->dbgm_enable_mode) {
	case DEBUG:
		return iwl_dnt_dev_if_send_dbgm(dnt, trans);
	case SNIFFER:
		ret = 0;
		for (i = 0; i < cfg->ldbg_cmd_nums; i++) {
			ret = iwl_dnt_dev_if_send_ldbg(dnt, trans, i);
			if (ret) {
				IWL_ERR(trans,
					"Failed to send ldbg command\n");
				break;
			}
		}
		return ret;
	default:
		WARN_ONCE(1, "invalid option: %d\n", cfg->dbgm_enable_mode);
		return -EINVAL;
	}
}

int iwl_dnt_dev_if_set_log_level(struct iwl_dnt *dnt,
				 struct iwl_trans *trans)
{
	struct iwl_dbg_cfg *cfg = &trans->dbg_cfg;
	struct iwl_host_cmd host_cmd = {
		.id = cfg->log_level_cmd_id,
		.data[0] = cfg->log_level_cmd.data,
		.len[0] = cfg->log_level_cmd.len,
		.dataflags[0] = IWL_HCMD_DFL_NOCOPY,
		.flags = CMD_SYNC | CMD_WANT_SKB,
	};
	int ret;

	ret = iwl_trans_send_cmd(trans, &host_cmd);
	if (ret)
		IWL_ERR(trans, "Failed to send log level cmd\n");

	return ret;
}

int iwl_dnt_dev_if_retrieve_monitor_data(struct iwl_dnt *dnt,
					 struct iwl_trans *trans,
					 u8 *buffer, u32 buffer_size)
{
	switch (dnt->cur_mon_type) {
	case DMA:
		return iwl_dnt_dev_if_retrieve_dma_monitor_data(dnt, trans,
								buffer,
								buffer_size);
	case MARBH:
		return iwl_dnt_dev_if_retrieve_marbh_monitor_data(dnt, trans,
								  buffer,
								  buffer_size);
	case ICCM:
		return iwl_dnt_dev_if_retrieve_iccm_monitor_data(dnt, trans,
								 buffer,
								 buffer_size);
	case INTERFACE:
	default:
		WARN_ONCE(1, "invalid option: %d\n", dnt->cur_mon_type);
		return -EINVAL;
	}
}

int iwl_dnt_dev_if_read_sram(struct iwl_dnt *dnt, struct iwl_trans *trans)
{
	struct dnt_crash_data *crash = &dnt->dispatch.crash;
	int ofs, len = 0;

	ofs = dnt->image->sec[IWL_UCODE_SECTION_DATA].offset;
	len = dnt->image->sec[IWL_UCODE_SECTION_DATA].len;

	crash->sram =  kzalloc(len , GFP_ATOMIC);
	if (!crash->sram)
		return -ENOMEM;

	crash->sram_buf_size = len;
	return iwl_trans_read_mem(trans, ofs, crash->sram, len);
}
IWL_EXPORT_SYMBOL(iwl_dnt_dev_if_read_sram);

int iwl_dnt_dev_if_read_rx(struct iwl_dnt *dnt, struct iwl_trans *trans)
{
	struct dnt_crash_data *crash = &dnt->dispatch.crash;
	int i, reg_val;
	u32 buf32_size, offset = 0;
	u32 *buf32;
	unsigned long flags;

	/* reading buffer size */
	reg_val = iwl_trans_read_prph(trans, RXF_SIZE_ADDR);
	crash->rx_buf_size = (reg_val & RXF_SIZE_BYTE_CNT_MSK) >> 6;
	if (!crash->rx_buf_size)
		return -ENOMEM;

	buf32_size = crash->rx_buf_size / sizeof(u32);

	crash->rx =  kzalloc(crash->rx_buf_size , GFP_ATOMIC);
	if (!crash->rx)
		return -ENOMEM;

	buf32 = (u32 *)crash->rx;

	if (!iwl_trans_grab_nic_access(trans, false, &flags)) {
		kfree(crash->rx);
		return -EBUSY;
	}
	for (i = 0; i < buf32_size; i++) {
		iwl_trans_write_prph(trans, RXF_LD_FENCE_OFFSET_ADDR, offset);
		offset += sizeof(u32);
		buf32[i] = iwl_trans_read_prph(trans, RXF_FIFO_RD_FENCE_ADDR);
	}
	iwl_trans_release_nic_access(trans, &flags);

	return 0;
}
IWL_EXPORT_SYMBOL(iwl_dnt_dev_if_read_rx);
