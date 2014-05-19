/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
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
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
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
#include <net/mac80211.h>

#include "iwl-debug.h"
#include "iwl-io.h"

#include "mvm.h"
#include "fw-api-rs.h"
#ifdef CPTCFG_IWLMVM_TCM
#include "vendor-cmd.h"
#endif

/*
 * Will return 0 even if the cmd failed when RFKILL is asserted unless
 * CMD_WANT_SKB is set in cmd->flags.
 */
int iwl_mvm_send_cmd(struct iwl_mvm *mvm, struct iwl_host_cmd *cmd)
{
	int ret;

#if defined(CPTCFG_IWLWIFI_DEBUGFS) && defined(CONFIG_PM_SLEEP)
	if (WARN_ON(mvm->d3_test_active))
		return -EIO;
#endif

	/*
	 * Synchronous commands from this op-mode must hold
	 * the mutex, this ensures we don't try to send two
	 * (or more) synchronous commands at a time.
	 */
	if (!(cmd->flags & CMD_ASYNC))
		lockdep_assert_held(&mvm->mutex);

	ret = iwl_trans_send_cmd(mvm->trans, cmd);

	/*
	 * If the caller wants the SKB, then don't hide any problems, the
	 * caller might access the response buffer which will be NULL if
	 * the command failed.
	 */
	if (cmd->flags & CMD_WANT_SKB)
		return ret;

	/* Silently ignore failures if RFKILL is asserted */
	if (!ret || ret == -ERFKILL)
		return 0;
	return ret;
}

int iwl_mvm_send_cmd_pdu(struct iwl_mvm *mvm, u8 id,
			 u32 flags, u16 len, const void *data)
{
	struct iwl_host_cmd cmd = {
		.id = id,
		.len = { len, },
		.data = { data, },
		.flags = flags,
	};

	return iwl_mvm_send_cmd(mvm, &cmd);
}

/*
 * We assume that the caller set the status to the sucess value
 */
int iwl_mvm_send_cmd_status(struct iwl_mvm *mvm, struct iwl_host_cmd *cmd,
			    u32 *status)
{
	struct iwl_rx_packet *pkt;
	struct iwl_cmd_response *resp;
	int ret, resp_len;

	lockdep_assert_held(&mvm->mutex);

#if defined(CPTCFG_IWLWIFI_DEBUGFS) && defined(CONFIG_PM_SLEEP)
	if (WARN_ON(mvm->d3_test_active))
		return -EIO;
#endif

	/*
	 * Only synchronous commands can wait for status,
	 * we use WANT_SKB so the caller can't.
	 */
	if (WARN_ONCE(cmd->flags & (CMD_ASYNC | CMD_WANT_SKB),
		      "cmd flags %x", cmd->flags))
		return -EINVAL;

	cmd->flags |= CMD_SYNC | CMD_WANT_SKB;

	ret = iwl_trans_send_cmd(mvm->trans, cmd);
	if (ret == -ERFKILL) {
		/*
		 * The command failed because of RFKILL, don't update
		 * the status, leave it as success and return 0.
		 */
		return 0;
	} else if (ret) {
		return ret;
	}

	pkt = cmd->resp_pkt;
	/* Can happen if RFKILL is asserted */
	if (!pkt) {
		ret = 0;
		goto out_free_resp;
	}

	if (pkt->hdr.flags & IWL_CMD_FAILED_MSK) {
		ret = -EIO;
		goto out_free_resp;
	}

	resp_len = iwl_rx_packet_payload_len(pkt);
	if (WARN_ON_ONCE(resp_len != sizeof(*resp))) {
		ret = -EIO;
		goto out_free_resp;
	}

	resp = (void *)pkt->data;
	*status = le32_to_cpu(resp->status);
 out_free_resp:
	iwl_free_resp(cmd);
	return ret;
}

/*
 * We assume that the caller set the status to the sucess value
 */
int iwl_mvm_send_cmd_pdu_status(struct iwl_mvm *mvm, u8 id, u16 len,
				const void *data, u32 *status)
{
	struct iwl_host_cmd cmd = {
		.id = id,
		.len = { len, },
		.data = { data, },
	};

	return iwl_mvm_send_cmd_status(mvm, &cmd, status);
}

#define IWL_DECLARE_RATE_INFO(r) \
	[IWL_RATE_##r##M_INDEX] = IWL_RATE_##r##M_PLCP

/*
 * Translate from fw_rate_index (IWL_RATE_XXM_INDEX) to PLCP
 */
static const u8 fw_rate_idx_to_plcp[IWL_RATE_COUNT] = {
	IWL_DECLARE_RATE_INFO(1),
	IWL_DECLARE_RATE_INFO(2),
	IWL_DECLARE_RATE_INFO(5),
	IWL_DECLARE_RATE_INFO(11),
	IWL_DECLARE_RATE_INFO(6),
	IWL_DECLARE_RATE_INFO(9),
	IWL_DECLARE_RATE_INFO(12),
	IWL_DECLARE_RATE_INFO(18),
	IWL_DECLARE_RATE_INFO(24),
	IWL_DECLARE_RATE_INFO(36),
	IWL_DECLARE_RATE_INFO(48),
	IWL_DECLARE_RATE_INFO(54),
};

int iwl_mvm_legacy_rate_to_mac80211_idx(u32 rate_n_flags,
					enum ieee80211_band band)
{
	int rate = rate_n_flags & RATE_LEGACY_RATE_MSK;
	int idx;
	int band_offset = 0;

	/* Legacy rate format, search for match in table */
	if (band == IEEE80211_BAND_5GHZ)
		band_offset = IWL_FIRST_OFDM_RATE;
	for (idx = band_offset; idx < IWL_RATE_COUNT_LEGACY; idx++)
		if (fw_rate_idx_to_plcp[idx] == rate)
			return idx - band_offset;

	return -1;
}

u8 iwl_mvm_mac80211_idx_to_hwrate(int rate_idx)
{
	/* Get PLCP rate for tx_cmd->rate_n_flags */
	return fw_rate_idx_to_plcp[rate_idx];
}

int iwl_mvm_rx_fw_error(struct iwl_mvm *mvm, struct iwl_rx_cmd_buffer *rxb,
			  struct iwl_device_cmd *cmd)
{
	struct iwl_rx_packet *pkt = rxb_addr(rxb);
	struct iwl_error_resp *err_resp = (void *)pkt->data;

	IWL_ERR(mvm, "FW Error notification: type 0x%08X cmd_id 0x%02X\n",
		le32_to_cpu(err_resp->error_type), err_resp->cmd_id);
	IWL_ERR(mvm, "FW Error notification: seq 0x%04X service 0x%08X\n",
		le16_to_cpu(err_resp->bad_cmd_seq_num),
		le32_to_cpu(err_resp->error_service));
	IWL_ERR(mvm, "FW Error notification: timestamp 0x%16llX\n",
		le64_to_cpu(err_resp->timestamp));
	return 0;
}

/*
 * Returns the first antenna as ANT_[ABC], as defined in iwl-config.h.
 * The parameter should also be a combination of ANT_[ABC].
 */
u8 first_antenna(u8 mask)
{
	BUILD_BUG_ON(ANT_A != BIT(0)); /* using ffs is wrong if not */
	if (WARN_ON_ONCE(!mask)) /* ffs will return 0 if mask is zeroed */
		return BIT(0);
	return BIT(ffs(mask) - 1);
}

/*
 * Toggles between TX antennas to send the probe request on.
 * Receives the bitmask of valid TX antennas and the *index* used
 * for the last TX, and returns the next valid *index* to use.
 * In order to set it in the tx_cmd, must do BIT(idx).
 */
u8 iwl_mvm_next_antenna(struct iwl_mvm *mvm, u8 valid, u8 last_idx)
{
	u8 ind = last_idx;
	int i;

	for (i = 0; i < RATE_MCS_ANT_NUM; i++) {
		ind = (ind + 1) % RATE_MCS_ANT_NUM;
		if (valid & BIT(ind))
			return ind;
	}

	WARN_ONCE(1, "Failed to toggle between antennas 0x%x", valid);
	return last_idx;
}

static const struct {
	const char *name;
	u8 num;
} advanced_lookup[] = {
	{ "NMI_INTERRUPT_WDG", 0x34 },
	{ "SYSASSERT", 0x35 },
	{ "UCODE_VERSION_MISMATCH", 0x37 },
	{ "BAD_COMMAND", 0x38 },
	{ "NMI_INTERRUPT_DATA_ACTION_PT", 0x3C },
	{ "FATAL_ERROR", 0x3D },
	{ "NMI_TRM_HW_ERR", 0x46 },
	{ "NMI_INTERRUPT_TRM", 0x4C },
	{ "NMI_INTERRUPT_BREAK_POINT", 0x54 },
	{ "NMI_INTERRUPT_WDG_RXF_FULL", 0x5C },
	{ "NMI_INTERRUPT_WDG_NO_RBD_RXF_FULL", 0x64 },
	{ "NMI_INTERRUPT_HOST", 0x66 },
	{ "NMI_INTERRUPT_ACTION_PT", 0x7C },
	{ "NMI_INTERRUPT_UNKNOWN", 0x84 },
	{ "NMI_INTERRUPT_INST_ACTION_PT", 0x86 },
	{ "ADVANCED_SYSASSERT", 0 },
};

static const char *desc_lookup(u32 num)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(advanced_lookup) - 1; i++)
		if (advanced_lookup[i].num == num)
			return advanced_lookup[i].name;

	/* No entry matches 'num', so it is the last: ADVANCED_SYSASSERT */
	return advanced_lookup[i].name;
}

/*
 * Note: This structure is read from the device with IO accesses,
 * and the reading already does the endian conversion. As it is
 * read with u32-sized accesses, any members with a different size
 * need to be ordered correctly though!
 */
struct iwl_error_event_table {
	u32 valid;		/* (nonzero) valid, (0) log is empty */
	u32 error_id;		/* type of error */
	u32 pc;			/* program counter */
	u32 blink1;		/* branch link */
	u32 blink2;		/* branch link */
	u32 ilink1;		/* interrupt link */
	u32 ilink2;		/* interrupt link */
	u32 data1;		/* error-specific data */
	u32 data2;		/* error-specific data */
	u32 data3;		/* error-specific data */
	u32 bcon_time;		/* beacon timer */
	u32 tsf_low;		/* network timestamp function timer */
	u32 tsf_hi;		/* network timestamp function timer */
	u32 gp1;		/* GP1 timer register */
	u32 gp2;		/* GP2 timer register */
	u32 gp3;		/* GP3 timer register */
	u32 ucode_ver;		/* uCode version */
	u32 hw_ver;		/* HW Silicon version */
	u32 brd_ver;		/* HW board version */
	u32 log_pc;		/* log program counter */
	u32 frame_ptr;		/* frame pointer */
	u32 stack_ptr;		/* stack pointer */
	u32 hcmd;		/* last host command header */
	u32 isr0;		/* isr status register LMPM_NIC_ISR0:
				 * rxtx_flag */
	u32 isr1;		/* isr status register LMPM_NIC_ISR1:
				 * host_flag */
	u32 isr2;		/* isr status register LMPM_NIC_ISR2:
				 * enc_flag */
	u32 isr3;		/* isr status register LMPM_NIC_ISR3:
				 * time_flag */
	u32 isr4;		/* isr status register LMPM_NIC_ISR4:
				 * wico interrupt */
	u32 isr_pref;		/* isr status register LMPM_NIC_PREF_STAT */
	u32 wait_event;		/* wait event() caller address */
	u32 l2p_control;	/* L2pControlField */
	u32 l2p_duration;	/* L2pDurationField */
	u32 l2p_mhvalid;	/* L2pMhValidBits */
	u32 l2p_addr_match;	/* L2pAddrMatchStat */
	u32 lmpm_pmg_sel;	/* indicate which clocks are turned on
				 * (LMPM_PMG_SEL) */
	u32 u_timestamp;	/* indicate when the date and time of the
				 * compilation */
	u32 flow_handler;	/* FH read/write pointers, RX credit */
} __packed;

/*
 * UMAC error struct - relevant starting from family 8000 chip.
 * Note: This structure is read from the device with IO accesses,
 * and the reading already does the endian conversion. As it is
 * read with u32-sized accesses, any members with a different size
 * need to be ordered correctly though!
 */
struct iwl_umac_error_event_table {
	u32 valid;		/* (nonzero) valid, (0) log is empty */
	u32 error_id;		/* type of error */
	u32 pc;			/* program counter */
	u32 blink1;		/* branch link */
	u32 blink2;		/* branch link */
	u32 ilink1;		/* interrupt link */
	u32 ilink2;		/* interrupt link */
	u32 data1;		/* error-specific data */
	u32 data2;		/* error-specific data */
	u32 line;		/* source code line of error */
	u32 umac_ver;		/* umac version */
} __packed;

#define ERROR_START_OFFSET  (1 * sizeof(u32))
#define ERROR_ELEM_SIZE     (7 * sizeof(u32))

static void iwl_mvm_dump_umac_error_log(struct iwl_mvm *mvm)
{
	struct iwl_trans *trans = mvm->trans;
	struct iwl_umac_error_event_table table;
	u32 base;

	base = mvm->umac_error_event_table;

	if (base < 0x800000 || base >= 0x80C000) {
		IWL_ERR(mvm,
			"Not valid error log pointer 0x%08X for %s uCode\n",
			base,
			(mvm->cur_ucode == IWL_UCODE_INIT)
					? "Init" : "RT");
		return;
	}

	iwl_trans_read_mem_bytes(trans, base, &table, sizeof(table));

	if (ERROR_START_OFFSET <= table.valid * ERROR_ELEM_SIZE) {
		IWL_ERR(trans, "Start IWL Error Log Dump:\n");
		IWL_ERR(trans, "Status: 0x%08lX, count: %d\n",
			mvm->status, table.valid);
	}

	IWL_ERR(mvm, "0x%08X | %-28s\n", table.error_id,
		desc_lookup(table.error_id));
	IWL_ERR(mvm, "0x%08X | umac uPc\n", table.pc);
	IWL_ERR(mvm, "0x%08X | umac branchlink1\n", table.blink1);
	IWL_ERR(mvm, "0x%08X | umac branchlink2\n", table.blink2);
	IWL_ERR(mvm, "0x%08X | umac interruptlink1\n", table.ilink1);
	IWL_ERR(mvm, "0x%08X | umac interruptlink2\n", table.ilink2);
	IWL_ERR(mvm, "0x%08X | umac data1\n", table.data1);
	IWL_ERR(mvm, "0x%08X | umac data2\n", table.data2);
	IWL_ERR(mvm, "0x%08X | umac version\n", table.umac_ver);
}

void iwl_mvm_dump_nic_error_log(struct iwl_mvm *mvm)
{
	struct iwl_trans *trans = mvm->trans;
	struct iwl_error_event_table table;
	u32 base;

	base = mvm->error_event_table;
	if (mvm->cur_ucode == IWL_UCODE_INIT) {
		if (!base)
			base = mvm->fw->init_errlog_ptr;
	} else {
		if (!base)
			base = mvm->fw->inst_errlog_ptr;
	}

	if (base < 0x800000) {
		IWL_ERR(mvm,
			"Not valid error log pointer 0x%08X for %s uCode\n",
			base,
			(mvm->cur_ucode == IWL_UCODE_INIT)
					? "Init" : "RT");
		return;
	}

	iwl_trans_read_mem_bytes(trans, base, &table, sizeof(table));

	if (ERROR_START_OFFSET <= table.valid * ERROR_ELEM_SIZE) {
		IWL_ERR(trans, "Start IWL Error Log Dump:\n");
		IWL_ERR(trans, "Status: 0x%08lX, count: %d\n",
			mvm->status, table.valid);
	}

	IWL_ERR(mvm, "Loaded firmware version: %s\n", mvm->fw->fw_version);

	trace_iwlwifi_dev_ucode_error(trans->dev, table.error_id, table.tsf_low,
				      table.data1, table.data2, table.data3,
				      table.blink1, table.blink2, table.ilink1,
				      table.ilink2, table.bcon_time, table.gp1,
				      table.gp2, table.gp3, table.ucode_ver,
				      table.hw_ver, table.brd_ver);
	IWL_ERR(mvm, "0x%08X | %-28s\n", table.error_id,
		desc_lookup(table.error_id));
	IWL_ERR(mvm, "0x%08X | uPc\n", table.pc);
	IWL_ERR(mvm, "0x%08X | branchlink1\n", table.blink1);
	IWL_ERR(mvm, "0x%08X | branchlink2\n", table.blink2);
	IWL_ERR(mvm, "0x%08X | interruptlink1\n", table.ilink1);
	IWL_ERR(mvm, "0x%08X | interruptlink2\n", table.ilink2);
	IWL_ERR(mvm, "0x%08X | data1\n", table.data1);
	IWL_ERR(mvm, "0x%08X | data2\n", table.data2);
	IWL_ERR(mvm, "0x%08X | data3\n", table.data3);
	IWL_ERR(mvm, "0x%08X | beacon time\n", table.bcon_time);
	IWL_ERR(mvm, "0x%08X | tsf low\n", table.tsf_low);
	IWL_ERR(mvm, "0x%08X | tsf hi\n", table.tsf_hi);
	IWL_ERR(mvm, "0x%08X | time gp1\n", table.gp1);
	IWL_ERR(mvm, "0x%08X | time gp2\n", table.gp2);
	IWL_ERR(mvm, "0x%08X | time gp3\n", table.gp3);
	IWL_ERR(mvm, "0x%08X | uCode version\n", table.ucode_ver);
	IWL_ERR(mvm, "0x%08X | hw version\n", table.hw_ver);
	IWL_ERR(mvm, "0x%08X | board version\n", table.brd_ver);
	IWL_ERR(mvm, "0x%08X | hcmd\n", table.hcmd);
	IWL_ERR(mvm, "0x%08X | isr0\n", table.isr0);
	IWL_ERR(mvm, "0x%08X | isr1\n", table.isr1);
	IWL_ERR(mvm, "0x%08X | isr2\n", table.isr2);
	IWL_ERR(mvm, "0x%08X | isr3\n", table.isr3);
	IWL_ERR(mvm, "0x%08X | isr4\n", table.isr4);
	IWL_ERR(mvm, "0x%08X | isr_pref\n", table.isr_pref);
	IWL_ERR(mvm, "0x%08X | wait_event\n", table.wait_event);
	IWL_ERR(mvm, "0x%08X | l2p_control\n", table.l2p_control);
	IWL_ERR(mvm, "0x%08X | l2p_duration\n", table.l2p_duration);
	IWL_ERR(mvm, "0x%08X | l2p_mhvalid\n", table.l2p_mhvalid);
	IWL_ERR(mvm, "0x%08X | l2p_addr_match\n", table.l2p_addr_match);
	IWL_ERR(mvm, "0x%08X | lmpm_pmg_sel\n", table.lmpm_pmg_sel);
	IWL_ERR(mvm, "0x%08X | timestamp\n", table.u_timestamp);
	IWL_ERR(mvm, "0x%08X | flow_handler\n", table.flow_handler);

	if (mvm->support_umac_log)
		iwl_mvm_dump_umac_error_log(mvm);
}

void iwl_mvm_dump_sram(struct iwl_mvm *mvm)
{
	const struct fw_img *img;
	int ofs, len = 0;
	int i;
	__le32 *buf;

	if (!mvm->ucode_loaded)
		return;

	img = &mvm->fw->img[mvm->cur_ucode];
	ofs = img->sec[IWL_UCODE_SECTION_DATA].offset;
	len = img->sec[IWL_UCODE_SECTION_DATA].len;

	buf = kzalloc(len, GFP_ATOMIC);
	if (!buf)
		return;

	iwl_trans_read_mem_bytes(mvm->trans, ofs, buf, len);
	len = len >> 2;
	for (i = 0; i < len; i++) {
		IWL_ERR(mvm, "0x%08X\n", le32_to_cpu(buf[i]));
		/* Add a small delay to let syslog catch up */
		udelay(10);
	}

	kfree(buf);
}

/**
 * iwl_mvm_send_lq_cmd() - Send link quality command
 * @init: This command is sent as part of station initialization right
 *        after station has been added.
 *
 * The link quality command is sent as the last step of station creation.
 * This is the special case in which init is set and we call a callback in
 * this case to clear the state indicating that station creation is in
 * progress.
 */
int iwl_mvm_send_lq_cmd(struct iwl_mvm *mvm, struct iwl_lq_cmd *lq, bool init)
{
	struct iwl_host_cmd cmd = {
		.id = LQ_CMD,
		.len = { sizeof(struct iwl_lq_cmd), },
		.flags = init ? CMD_SYNC : CMD_ASYNC,
		.data = { lq, },
	};

	if (WARN_ON(lq->sta_id == IWL_MVM_STATION_COUNT))
		return -EINVAL;

	return iwl_mvm_send_cmd(mvm, &cmd);
}

/**
 * iwl_mvm_update_smps - Get a requst to change the SMPS mode
 * @req_type: The part of the driver who call for a change.
 * @smps_requests: The request to change the SMPS mode.
 *
 * Get a requst to change the SMPS mode,
 * and change it according to all other requests in the driver.
 */
void iwl_mvm_update_smps(struct iwl_mvm *mvm, struct ieee80211_vif *vif,
			 enum iwl_mvm_smps_type_request req_type,
			 enum ieee80211_smps_mode smps_request)
{
	struct iwl_mvm_vif *mvmvif;
	enum ieee80211_smps_mode smps_mode;
	int i;

	lockdep_assert_held(&mvm->mutex);

	/* SMPS is irrelevant for NICs that don't have at least 2 RX antenna */
	if (num_of_ant(mvm->fw->valid_rx_ant) == 1)
		return;

	if (vif->type == NL80211_IFTYPE_AP)
		smps_mode = IEEE80211_SMPS_OFF;
	else
		smps_mode = IEEE80211_SMPS_AUTOMATIC;

	mvmvif = iwl_mvm_vif_from_mac80211(vif);
	mvmvif->smps_requests[req_type] = smps_request;
	for (i = 0; i < NUM_IWL_MVM_SMPS_REQ; i++) {
		if (mvmvif->smps_requests[i] == IEEE80211_SMPS_STATIC) {
			smps_mode = IEEE80211_SMPS_STATIC;
			break;
		}
		if (mvmvif->smps_requests[i] == IEEE80211_SMPS_DYNAMIC)
			smps_mode = IEEE80211_SMPS_DYNAMIC;
	}

	ieee80211_request_smps(vif, smps_mode);
}

static void iwl_mvm_diversity_iter(void *_data, u8 *mac,
				   struct ieee80211_vif *vif)
{
	struct iwl_mvm_vif *mvmvif = iwl_mvm_vif_from_mac80211(vif);
	bool *result = _data;
	int i;

	for (i = 0; i < NUM_IWL_MVM_SMPS_REQ; i++) {
		if (mvmvif->smps_requests[i] == IEEE80211_SMPS_STATIC ||
		    mvmvif->smps_requests[i] == IEEE80211_SMPS_DYNAMIC)
			*result = false;
	}
}

bool iwl_mvm_rx_diversity_allowed(struct iwl_mvm *mvm)
{
	bool result = true;

	lockdep_assert_held(&mvm->mutex);

	if (num_of_ant(mvm->fw->valid_rx_ant) == 1)
		return false;

	if (!mvm->cfg->rx_with_siso_diversity)
		return false;

	ieee80211_iterate_active_interfaces_atomic(
			mvm->hw, IEEE80211_IFACE_ITER_NORMAL,
			iwl_mvm_diversity_iter, &result);

	return result;
}

int iwl_mvm_update_low_latency(struct iwl_mvm *mvm, struct ieee80211_vif *vif,
			       bool value)
{
	struct iwl_mvm_vif *mvmvif = iwl_mvm_vif_from_mac80211(vif);
	int res;

	lockdep_assert_held(&mvm->mutex);

	if (mvmvif->low_latency == value)
		return 0;

	mvmvif->low_latency = value;

	res = iwl_mvm_update_quotas(mvm, NULL);
	if (res)
		return res;

	iwl_mvm_bt_coex_vif_change(mvm);

#ifdef CPTCFG_IWLMVM_TCM
	iwl_mvm_send_tcm_event(mvm, vif);
#endif

	return iwl_mvm_power_update_mac(mvm, vif);
}

static void iwl_mvm_ll_iter(void *_data, u8 *mac, struct ieee80211_vif *vif)
{
	bool *result = _data;

	if (iwl_mvm_vif_low_latency(iwl_mvm_vif_from_mac80211(vif)))
		*result = true;
}

bool iwl_mvm_low_latency(struct iwl_mvm *mvm)
{
	bool result = false;

	ieee80211_iterate_active_interfaces_atomic(
			mvm->hw, IEEE80211_IFACE_ITER_NORMAL,
			iwl_mvm_ll_iter, &result);

	return result;
}

static void iwl_mvm_idle_iter(void *_data, u8 *mac, struct ieee80211_vif *vif)
{
	bool *idle = _data;

	if (!vif->bss_conf.idle)
		*idle = false;
}

bool iwl_mvm_is_idle(struct iwl_mvm *mvm)
{
	bool idle = true;

	ieee80211_iterate_active_interfaces_atomic(
			mvm->hw, IEEE80211_IFACE_ITER_NORMAL,
			iwl_mvm_idle_iter, &idle);

	return idle;
}

#ifdef CPTCFG_IWLMVM_TCM
static enum iwl_mvm_vendor_load
iwl_mvm_tcm_load(struct iwl_mvm *mvm, u32 airtime, unsigned long elapsed)
{
	unsigned long load = (100 * airtime / elapsed) / USEC_PER_MSEC;

	if (load > IWL_MVM_TCM_LOAD_HIGH_THRESH)
		return IWL_MVM_VENDOR_LOAD_HIGH;
	if (load > IWL_MVM_TCM_LOAD_MEDIUM_THRESH)
		return IWL_MVM_VENDOR_LOAD_MEDIUM;
	return IWL_MVM_VENDOR_LOAD_LOW;
}

void iwl_mvm_tcm_timer(unsigned long data)
{
	iwl_mvm_recalc_tcm((void *)data);
}

struct iwl_mvm_tcm_iter_data {
	struct iwl_mvm *mvm;
	bool any_sent;
};

static void iwl_mvm_tcm_iter(void *_data, u8 *mac, struct ieee80211_vif *vif)
{
	struct iwl_mvm_tcm_iter_data *data = _data;
	struct iwl_mvm *mvm = data->mvm;
	struct iwl_mvm_vif *mvmvif = iwl_mvm_vif_from_mac80211(vif);
	bool low_latency;

	if (mvmvif->id >= NUM_MAC_INDEX_DRIVER)
		return;

	low_latency = mvm->tcm.result.low_latency[mvmvif->id];

	if (!mvm->tcm.result.change[mvmvif->id] &&
	    mvmvif->low_latency == low_latency)
		return;

	if (mvmvif->low_latency != low_latency) {
		/* this sends traffic load and updates quota as well */
		iwl_mvm_update_low_latency(mvm, vif, low_latency);
	} else {
		iwl_mvm_send_tcm_event(mvm, vif);
		iwl_mvm_update_quotas(mvm, NULL);
	}

	data->any_sent = true;
}

void iwl_mvm_tcm_work(struct work_struct *work)
{
	struct iwl_mvm *mvm = container_of(work, struct iwl_mvm, tcm.work);
	struct iwl_mvm_tcm_iter_data data = {
		.mvm = mvm,
		.any_sent = false,
	};

	mutex_lock(&mvm->mutex);

	ieee80211_iterate_active_interfaces(
		mvm->hw, IEEE80211_IFACE_ITER_NORMAL,
		iwl_mvm_tcm_iter, &data);

	/* send global only */
	if (mvm->tcm.result.global_change && !data.any_sent)
		iwl_mvm_send_tcm_event(mvm, NULL);

	mutex_unlock(&mvm->mutex);
}

static unsigned long iwl_mvm_calc_tcm_stats(struct iwl_mvm *mvm,
					    unsigned long ts)
{
	unsigned int elapsed = jiffies_to_msecs(ts - mvm->tcm.ts);
	u32 total_airtime = 0;
	int ac, mac;
	bool low_latency = false;
	u8 load;
	bool handle_ll = time_after(ts, mvm->tcm.ll_ts + MVM_LL_PERIOD);

	if (handle_ll)
		mvm->tcm.ll_ts = ts;

	for (mac = 0; mac < NUM_MAC_INDEX_DRIVER; mac++) {
		u32 vo_vi_pkts = 0;
		u32 airtime = 0;

		for (ac = IEEE80211_AC_VO; ac <= IEEE80211_AC_BK; ac++)
			airtime += mvm->tcm.data[mac].rx.airtime[ac] +
				   mvm->tcm.data[mac].tx.airtime[ac];
		total_airtime += airtime;

		load = iwl_mvm_tcm_load(mvm, airtime, elapsed);
		mvm->tcm.result.change[mac] = load != mvm->tcm.result.load[mac];
		mvm->tcm.result.load[mac] = load;
		mvm->tcm.result.airtime[mac] = airtime;

		for (ac = IEEE80211_AC_VO; ac <= IEEE80211_AC_VI; ac++)
			vo_vi_pkts += mvm->tcm.data[mac].rx.pkts[ac] +
				      mvm->tcm.data[mac].tx.pkts[ac];

		/* enable immediately with enough packets but defer disabling */
		if (vo_vi_pkts > IWL_MVM_TCM_LOWLAT_ENABLE_THRESH)
			mvm->tcm.result.low_latency[mac] = true;
		else if (handle_ll)
			mvm->tcm.result.low_latency[mac] = false;

		if (handle_ll) {
			/* clear old data */
			memset(&mvm->tcm.data[mac].rx.pkts, 0,
			       sizeof(mvm->tcm.data[mac].rx.pkts));
			memset(&mvm->tcm.data[mac].tx.pkts, 0,
			       sizeof(mvm->tcm.data[mac].tx.pkts));
		}
		low_latency |= mvm->tcm.result.low_latency[mac];

		/* clear old data */
		memset(&mvm->tcm.data[mac].rx.airtime, 0,
		       sizeof(mvm->tcm.data[mac].rx.airtime));
		memset(&mvm->tcm.data[mac].tx.airtime, 0,
		       sizeof(mvm->tcm.data[mac].tx.airtime));
	}

	load = iwl_mvm_tcm_load(mvm, total_airtime, elapsed);
	mvm->tcm.result.global_change = load != mvm->tcm.result.global_load;
	mvm->tcm.result.global_load = load;

	schedule_work(&mvm->tcm.work);

	if (load != IWL_MVM_VENDOR_LOAD_LOW)
		return MVM_TCM_PERIOD;
	if (low_latency)
		return MVM_LL_PERIOD;
	return 0;
}

void iwl_mvm_recalc_tcm(struct iwl_mvm *mvm)
{
	unsigned long ts = jiffies;

	spin_lock(&mvm->tcm.lock);
	/* re-check if somebody else won the recheck race */
	if (!mvm->tcm.paused && time_after(ts, mvm->tcm.ts + MVM_TCM_PERIOD)) {
		/* calculate statistics */
		unsigned long timer_delay = iwl_mvm_calc_tcm_stats(mvm, ts);

		/* the memset needs to be visible before the timestamp */
		smp_mb();
		mvm->tcm.ts = ts;
		if (timer_delay)
			mod_timer(&mvm->tcm.timer,
				  mvm->tcm.ts + timer_delay);
	}
	spin_unlock(&mvm->tcm.lock);
}

void iwl_mvm_pause_tcm(struct iwl_mvm *mvm)
{
	spin_lock(&mvm->tcm.lock);
	del_timer(&mvm->tcm.timer);
	mvm->tcm.paused = true;
	spin_unlock(&mvm->tcm.lock);
}

void iwl_mvm_resume_tcm(struct iwl_mvm *mvm)
{
	int mac;

	spin_lock(&mvm->tcm.lock);
	mvm->tcm.ts = jiffies;
	mvm->tcm.ll_ts = jiffies;
	for (mac = 0; mac < NUM_MAC_INDEX_DRIVER; mac++) {
		memset(&mvm->tcm.data[mac].rx.pkts, 0,
		       sizeof(mvm->tcm.data[mac].rx.pkts));
		memset(&mvm->tcm.data[mac].tx.pkts, 0,
		       sizeof(mvm->tcm.data[mac].tx.pkts));

		memset(&mvm->tcm.data[mac].rx.airtime, 0,
		       sizeof(mvm->tcm.data[mac].rx.airtime));
		memset(&mvm->tcm.data[mac].tx.airtime, 0,
		       sizeof(mvm->tcm.data[mac].tx.airtime));
	}
	/* The TCM data needs to be reset before "paused" flag changes */
	smp_mb();
	mvm->tcm.paused = false;
	spin_unlock(&mvm->tcm.lock);
}
#endif
