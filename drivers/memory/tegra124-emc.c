/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_data/tegra_emc.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <soc/tegra/pmc.h>
#include <soc/tegra/mc.h>

#include <asm/cputime.h>

#include "tegra124-emc-reg.h"

#define EMC_CLK_DIV_SHIFT			0
#define EMC_CLK_DIV_MASK			(0xFF << EMC_CLK_DIV_SHIFT)
#define EMC_CLK_SOURCE				0x19C
#define EMC_CLK_SOURCE_SHIFT			29
#define EMC_CLK_SOURCE_MASK			(0x7 << EMC_CLK_SOURCE_SHIFT)
#define EMC_CLK_LOW_JITTER_ENABLE		(0x1 << 31)
#define EMC_CLK_MC_SAME_FREQ			(0x1 << 16)

#define TEGRA_EMC_TABLE_MAX_SIZE		16
#define EMC_STATUS_UPDATE_TIMEOUT		1000

#define PRE_WAIT_SREF_US			5
#define PRE_WAIT_BGBIAS_US			5
#define PRE_WAIT_DQS_US				30

#define STRAPPING_OPT_A			0x464
#define STRAPPING_OPT_A_RAM_CODE_SHIFT	4
#define STRAPPING_OPT_A_RAM_CODE_MASK	(0xF << STRAPPING_OPT_A_RAM_CODE_SHIFT)

static bool emc_enable = true;
module_param(emc_enable, bool, 0644);

enum tegra124_mem_reg_type {
	TEGRA124_MEM_REG_EMC,
	TEGRA124_MEM_REG_MC,
};

#define BURST_REG_LIST	\
{	\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_RC),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_RFC),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_RFC_SLR),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_RAS),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_RP),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_R2W),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_W2R),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_R2P),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_W2P),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_RD_RCD),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_WR_RCD),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_RRD),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_REXT),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_WEXT),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_WDV),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_WDV_MASK),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_QUSE),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_QUSE_WIDTH),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_IBDLY),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_EINPUT),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_EINPUT_DURATION),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_PUTERM_EXTRA),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_PUTERM_WIDTH),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_PUTERM_ADJ),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_CDB_CNTL_1),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_CDB_CNTL_2),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_CDB_CNTL_3),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_QRST),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_QSAFE),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_RDV),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_RDV_MASK),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_REFRESH),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_BURST_REFRESH_NUM),	\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_PRE_REFRESH_REQ_CNT),	\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_PDEX2WR),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_PDEX2RD),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_PCHG2PDEN),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_ACT2PDEN),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_AR2PDEN),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_RW2PDEN),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_TXSR),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_TXSRDLL),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_TCKE),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_TCKESR),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_TPD),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_TFAW),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_TRPAB),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_TCLKSTABLE),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_TCLKSTOP),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_TREFBW),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_FBIO_CFG6),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_ODT_WRITE),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_ODT_READ),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_FBIO_CFG5),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_CFG_DIG_DLL),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_CFG_DIG_DLL_PERIOD),	\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQS0),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQS1),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQS2),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQS3),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQS4),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQS5),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQS6),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQS7),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQS8),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQS9),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQS10),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQS11),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQS12),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQS13),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQS14),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQS15),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_QUSE0),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_QUSE1),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_QUSE2),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_QUSE3),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_QUSE4),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_QUSE5),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_QUSE6),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_QUSE7),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_ADDR0),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_ADDR1),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_ADDR2),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_ADDR3),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_ADDR4),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_ADDR5),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_QUSE8),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_QUSE9),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_QUSE10),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_QUSE11),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_QUSE12),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_QUSE13),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_QUSE14),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_QUSE15),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLI_TRIM_TXDQS0),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLI_TRIM_TXDQS1),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLI_TRIM_TXDQS2),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLI_TRIM_TXDQS3),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLI_TRIM_TXDQS4),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLI_TRIM_TXDQS5),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLI_TRIM_TXDQS6),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLI_TRIM_TXDQS7),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLI_TRIM_TXDQS8),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLI_TRIM_TXDQS9),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLI_TRIM_TXDQS10),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLI_TRIM_TXDQS11),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLI_TRIM_TXDQS12),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLI_TRIM_TXDQS13),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLI_TRIM_TXDQS14),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLI_TRIM_TXDQS15),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQ0),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQ1),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQ2),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQ3),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQ4),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQ5),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQ6),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DLL_XFORM_DQ7),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_XM2CMDPADCTRL),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_XM2CMDPADCTRL4),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_XM2CMDPADCTRL5),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_XM2DQSPADCTRL2),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_XM2DQPADCTRL2),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_XM2DQPADCTRL3),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_XM2CLKPADCTRL),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_XM2CLKPADCTRL2),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_XM2COMPPADCTRL),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_XM2VTTGENPADCTRL),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_XM2VTTGENPADCTRL2),	\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_XM2VTTGENPADCTRL3),	\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_XM2DQSPADCTRL3),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_XM2DQSPADCTRL4),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_XM2DQSPADCTRL5),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_XM2DQSPADCTRL6),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DSR_VTTGEN_DRV),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_TXDSRVTTGEN),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_FBIO_SPARE),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_ZCAL_INTERVAL),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_ZCAL_WAIT_CNT),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_MRS_WAIT_CNT),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_MRS_WAIT_CNT2),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_CTT),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_CTT_DURATION),		\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_CFG_PIPE),			\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_DYN_SELF_REF_CONTROL),	\
	DEFINE_REG(TEGRA124_MEM_REG_EMC, EMC_QPOP),			\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_CFG),		\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_OUTSTANDING_REQ),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_TIMING_RCD),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_TIMING_RP),		\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_TIMING_RC),		\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_TIMING_RAS),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_TIMING_FAW),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_TIMING_RRD),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_TIMING_RAP2PRE),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_TIMING_WAP2PRE),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_TIMING_R2R),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_TIMING_W2W),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_TIMING_R2W),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_TIMING_W2R),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_DA_TURNS),		\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_DA_COVERS),		\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_MISC0),		\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_MISC1),		\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_EMEM_ARB_RING1_THROTTLE),	\
}

#define BURST_UP_DOWN_REG_LIST	\
{	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_MLL_MPCORER_PTSA_RATE),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_PTSA_GRANT_DECREMENT),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_XUSB_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_XUSB_1),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_TSEC_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_SDMMCA_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_SDMMCAA_0),\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_SDMMC_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_SDMMCAB_0),\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_PPCS_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_PPCS_1),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_MPCORE_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_MPCORELP_0),\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_HC_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_HC_1),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_AVPC_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_GPU_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_MSENC_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_HDA_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_VIC_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_VI2_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_ISP2_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_ISP2_1),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_ISP2B_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_ISP2B_1),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_VDE_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_VDE_1),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_VDE_2),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_VDE_3),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_SATA_0),	\
	DEFINE_REG(TEGRA124_MEM_REG_MC, MC_LATENCY_ALLOWANCE_AFI_0),	\
}

#define DEFINE_REG(type, reg)		reg##_INDEX
enum BURST_REG_LIST;
enum BURST_UP_DOWN_REG_LIST;
#undef DEFINE_REG

#define DEFINE_REG(type, reg)		reg##_TRIM_INDEX
enum EMC_TRIMMERS_REG_LIST;
#undef DEFINE_REG

#define DEFINE_REG(type, reg)		(reg)
static const u32 burst_reg_addr[] = BURST_REG_LIST;
static const u32 burst_up_down_reg_addr[] = BURST_UP_DOWN_REG_LIST;
#undef DEFINE_REG

#define DEFINE_REG(type, reg)		(type)
static const u32 burst_reg_type[] = BURST_REG_LIST;
#undef DEFINE_REG

enum {
	DLL_CHANGE_NONE = 0,
	DLL_CHANGE_ON,
	DLL_CHANGE_OFF,
};

enum TEGRA_EMC_SOURCE {
	TEGRA_EMC_SRC_PLLM,
	TEGRA_EMC_SRC_PLLC,
	TEGRA_EMC_SRC_PLLP,
	TEGRA_EMC_SRC_CLKM,
	TEGRA_EMC_SRC_PLLM_UD,
	TEGRA_EMC_SRC_PLLC2,
	TEGRA_EMC_SRC_PLLC3,
	TEGRA_EMC_SRC_PLLC_UD,
	TEGRA_EMC_SRC_COUNT,
};

struct emc_table {
	u32 rev;
	unsigned long rate;
	int emc_min_mv;
	int gk20a_min_mv;
	const char *src_name;
	u32 src_sel_reg;

	int burst_regs_num;
	int up_down_regs_num;

	/* unconditionally updated in one burst shot */
	u32 *burst_regs;

	/* one burst shot, but update time depends on rate change direction */
	u32 *up_down_regs;

	/* updated separately under some conditions */
	u32 emc_zcal_cnt_long;
	u32 emc_acal_interval;
	u32 emc_ctt_term_ctrl;
	u32 emc_cfg;
	u32 emc_cfg_2;
	u32 emc_sel_dpd_ctrl;
	u32 emc_cfg_dig_dll;
	u32 emc_bgbias_ctl0;
	u32 emc_auto_cal_config2;
	u32 emc_auto_cal_config3;
	u32 emc_auto_cal_config;
	u32 emc_mode_reset;
	u32 emc_mode_1;
	u32 emc_mode_2;
	u32 emc_mode_4;
	u32 clock_change_latency;

	struct clk	*input;
	u32		value;
	unsigned long	input_rate;
};

struct emc_stats {
	cputime64_t time_at_clock[TEGRA_EMC_TABLE_MAX_SIZE];
	int last_sel;
	u64 last_update;
	u64 clkchange_count;
	spinlock_t spinlock;
};

static DEFINE_SPINLOCK(emc_access_lock);
static ktime_t clkchange_time;
static int tegra_emc_table_size;
static int clkchange_delay = 100;
static int last_round_idx;
static u32 tegra_dram_dev_num;
static u32 tegra_dram_type = -1;
static u32 tegra_ram_code;
static struct regmap *tegra_pmc_regs;
static bool tegra_emc_init_done;
static void __iomem *tegra_emc_base;
static void __iomem *tegra_clk_base;
static unsigned long emc_backup_rate;
static unsigned long emc_max_rate;
static unsigned long emc_boot_rate;
static struct emc_stats tegra_emc_stats;
static struct emc_table *tegra_emc_table;
static struct emc_table *emc_timing;
static struct emc_table start_timing;
static struct clk *emc_clk;
static struct clk *emc_override_clk;
static struct clk *emc_backup_src;
static struct clk *tegra_emc_src[TEGRA_EMC_SRC_COUNT];
static const char *tegra_emc_src_names[TEGRA_EMC_SRC_COUNT] = {
	[TEGRA_EMC_SRC_PLLM] = "pll_m",
	[TEGRA_EMC_SRC_PLLC] = "pll_c",
	[TEGRA_EMC_SRC_PLLP] = "pll_p",
	[TEGRA_EMC_SRC_CLKM] = "clk_m",
	[TEGRA_EMC_SRC_PLLM_UD] = "pll_m_ud",
	[TEGRA_EMC_SRC_PLLC2] = "pll_c2",
	[TEGRA_EMC_SRC_PLLC3] = "pll_c3",
	[TEGRA_EMC_SRC_PLLC_UD] = "pll_c_ud",
};

static inline void emc_writel(u32 val, unsigned long addr)
{
	writel(val, tegra_emc_base + addr);
}

static inline u32 emc_readl(unsigned long addr)
{
	return readl(tegra_emc_base + addr);
}

static inline int get_start_idx(unsigned long rate)
{
	if (tegra_emc_table[last_round_idx].rate == rate)
		return last_round_idx;
	return 0;
}

static inline void ccfifo_writel(u32 val, unsigned long addr)
{
	writel(val, tegra_emc_base + EMC_CCFIFO_DATA);
	writel(addr, tegra_emc_base + EMC_CCFIFO_ADDR);
}

static inline void burst_reg_writel(u32 val, int index)
{
	if (burst_reg_type[index] == TEGRA124_MEM_REG_EMC)
		return emc_writel(val, burst_reg_addr[index]);

	return tegra124_mc_writel(val, burst_reg_addr[index]);
}

static inline u32 burst_reg_readl(int index)
{
	if (burst_reg_type[index] == TEGRA124_MEM_REG_EMC)
		return emc_readl(burst_reg_addr[index]);

	return tegra124_mc_readl(burst_reg_addr[index]);
}

static inline void clk_cfg_writel(u32 val)
{
	writel(val, tegra_clk_base + EMC_CLK_SOURCE);
}

static inline u32 clk_cfg_readl(void)
{
	return readl(tegra_clk_base + EMC_CLK_SOURCE);
}

static inline u32 emc_src_val(u32 val)
{
	return (val & EMC_CLK_SOURCE_MASK) >> EMC_CLK_SOURCE_SHIFT;
}

static inline u32 emc_div_val(u32 val)
{
	return (val & EMC_CLK_DIV_MASK) >> EMC_CLK_DIV_SHIFT;
}

void tegra124_emc_timing_invalidate(void)
{
	emc_timing = NULL;
}
EXPORT_SYMBOL(tegra124_emc_timing_invalidate);

bool tegra124_emc_is_ready(void)
{
	return tegra_emc_init_done;
}
EXPORT_SYMBOL(tegra124_emc_is_ready);

unsigned long tegra124_predict_emc_rate(int millivolts)
{
	int i;
	unsigned long ret = -EINVAL;

	if (!emc_enable)
		return -ENODEV;

	if (!tegra_emc_init_done || !tegra_emc_table_size)
		return -EINVAL;

	for (i = 0; i < tegra_emc_table_size; i++) {
		if (tegra_emc_table[i].input == NULL)
			continue;
		if (tegra_emc_table[i].emc_min_mv > millivolts)
			break;
		ret = tegra_emc_table[i].rate;
	}

	return ret;
}
EXPORT_SYMBOL(tegra124_predict_emc_rate);

static unsigned long tegra124_emc_get_rate(void)
{
	u32 val;
	u32 div_value;
	u32 src_value;
	unsigned long rate;

	if (!emc_enable)
		return -ENODEV;

	if (!tegra_emc_init_done || !tegra_emc_table_size)
		return -EINVAL;

	val = clk_cfg_readl();

	div_value = emc_div_val(val);
	src_value = emc_src_val(val);

	rate = __clk_get_rate(tegra_emc_src[src_value]);

	do_div(rate, div_value + 2);

	return rate * 2;
}

static long tegra124_emc_round_rate(unsigned long rate)
{
	int i;
	int max = 0;

	if (!emc_enable)
		return 0;

	if (!tegra_emc_init_done || !tegra_emc_table_size)
		return 0;

	i = get_start_idx(rate);
	for (; i < tegra_emc_table_size; i++) {
		if (tegra_emc_table[i].input == NULL)
			continue;

		max = i;
		if (tegra_emc_table[i].rate >= rate) {
			last_round_idx = i;
			return tegra_emc_table[i].rate;
		}
	}

	return tegra_emc_table[max].rate;
}

static inline void emc_get_timing(struct emc_table *timing)
{
	int i;

	for (i = 0; i < timing->burst_regs_num; i++) {
		if (burst_reg_addr[i])
			timing->burst_regs[i] = burst_reg_readl(i);
		else
			timing->burst_regs[i] = 0;
	}

	timing->emc_acal_interval = 0;
	timing->emc_zcal_cnt_long = 0;
	timing->emc_mode_reset = 0;
	timing->emc_mode_1 = 0;
	timing->emc_mode_2 = 0;
	timing->emc_mode_4 = 0;
	timing->emc_cfg = emc_readl(EMC_CFG);
	timing->rate = __clk_get_rate(emc_clk);
}

static inline int get_dll_change(const struct emc_table *next_timing,
				const struct emc_table *last_timing)
{
	bool next_dll_enabled = !(next_timing->emc_mode_1 & 0x1);
	bool last_dll_enabled = !(last_timing->emc_mode_1 & 0x1);

	if (next_dll_enabled == last_dll_enabled)
		return DLL_CHANGE_NONE;
	else if (next_dll_enabled)
		return DLL_CHANGE_ON;
	else
		return DLL_CHANGE_OFF;
}

static inline u32 disable_power_features(u32 inreg)
{
	u32 mod_reg = inreg;

	mod_reg &= ~(EMC_CFG_DYN_SREF);
	mod_reg &= ~(EMC_CFG_DRAM_ACPD);
	mod_reg &= ~(EMC_CFG_DRAM_CLKSTOP_SR);
	mod_reg &= ~(EMC_CFG_DRAM_CLKSTOP_PD);
	mod_reg &= ~(EMC_CFG_DSR_VTTGEN_DRV_EN);

	return mod_reg;
}

static inline u32 emc_sel_dpd_ctrl_enabled(u32 inreg)
{
	if (tegra_dram_type == DRAM_TYPE_DDR3)
		return inreg & (EMC_SEL_DPD_CTRL_DDR3_MASK);
	else
		return inreg & (EMC_SEL_DPD_CTRL_MASK);
}

static inline u32 disable_emc_sel_dpd_ctrl(u32 inreg)
{
	u32 mod_reg = inreg;

	mod_reg &= ~(EMC_SEL_DPD_CTRL_DATA_SEL_DPD);
	mod_reg &= ~(EMC_SEL_DPD_CTRL_ODT_SEL_DPD);
	if (tegra_dram_type == DRAM_TYPE_DDR3)
		mod_reg &= ~(EMC_SEL_DPD_CTRL_RESET_SEL_DPD);
	mod_reg &= ~(EMC_SEL_DPD_CTRL_CA_SEL_DPD);
	mod_reg &= ~(EMC_SEL_DPD_CTRL_CLK_SEL_DPD);

	return mod_reg;
}

static inline bool bgbias_preset(const struct emc_table *next_timing,
					const struct emc_table *last_timing)
{
	bool ret = false;
	unsigned int data, reg;

	data = last_timing->emc_bgbias_ctl0;
	reg = emc_readl(EMC_BGBIAS_CTL0);

	if (!(next_timing->emc_bgbias_ctl0 &
	     EMC_BGBIAS_CTL0_BIAS0_DSC_E_PWRD_IBIAS_RX) &&
	    (reg & EMC_BGBIAS_CTL0_BIAS0_DSC_E_PWRD_IBIAS_RX)) {
		data &= ~EMC_BGBIAS_CTL0_BIAS0_DSC_E_PWRD_IBIAS_RX;
		ret = true;
	}

	if ((reg & EMC_BGBIAS_CTL0_BIAS0_DSC_E_PWRD) ||
	    (reg & EMC_BGBIAS_CTL0_BIAS0_DSC_E_PWRD_IBIAS_VTTGEN))
		ret = true;

	if (ret)
		emc_writel(data, EMC_BGBIAS_CTL0);

	return ret;
}

static inline bool dqs_preset(const struct emc_table *next_timing,
				const struct emc_table *last_timing)
{
	bool ret = false;
	unsigned int data;

	data = emc_readl(EMC_XM2DQSPADCTRL2);

#define DQS_SET(reg, bit)						\
	do {								\
		if ((next_timing->burst_regs[EMC_##reg##_INDEX] &	\
			EMC_##reg##_##bit##_ENABLE) &&			\
			(!(data & EMC_##reg##_##bit##_ENABLE))) {	\
			data |= EMC_##reg##_##bit##_ENABLE;		\
			ret = true;					\
		}							\
	} while (0)
	DQS_SET(XM2DQSPADCTRL2, VREF);
	DQS_SET(XM2DQSPADCTRL2, RX_FT_REC);
#undef DQS_SET

	if (ret)
		emc_writel(data, EMC_XM2DQSPADCTRL2);

	return ret;
}

static int wait_for_update(u32 status_reg, u32 bit_mask, bool updated_state)
{
	int i;
	for (i = 0; i < EMC_STATUS_UPDATE_TIMEOUT; i++) {
		if (!!(emc_readl(status_reg) & bit_mask) == updated_state)
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

static inline void wait_auto_cal_disable(void)
{
	int err;

	err = wait_for_update(EMC_AUTO_CAL_STATUS, EMC_AUTO_CAL_STATUS_ACTIVE,
		false);
	if (err) {
		pr_err("%s: wait disable auto-cal error: %d", __func__, err);
		BUG();
	}
}

static inline void auto_cal_disable(void)
{
	emc_writel(0, EMC_AUTO_CAL_INTERVAL);
	wait_auto_cal_disable();
}

static inline void emc_timing_update(void)
{
	int err;

	emc_writel(0x1, EMC_TIMING_CONTROL);
	err = wait_for_update(EMC_STATUS, EMC_STATUS_TIMING_UPDATE_STALLED,
		false);
	if (err) {
		pr_err("%s: timing update error: %d", __func__, err);
		BUG();
	}
}

static inline void overwrite_mrs_wait_cnt(const struct emc_table *next_timing,
	bool zcal_long)
{
	u32 reg;
	u32 cnt = 512;

	/* For ddr3 when DLL is re-started: overwrite EMC DFS table settings
	   for MRS_WAIT_LONG with maximum of MRS_WAIT_SHORT settings and
	   expected operation length. Reduce the latter by the overlapping
	   zq-calibration, if any */
	if (zcal_long)
		cnt -= tegra_dram_dev_num * 256;

	reg = (next_timing->burst_regs[EMC_MRS_WAIT_CNT_INDEX] &
		EMC_MRS_WAIT_CNT_SHORT_WAIT_MASK) >>
		EMC_MRS_WAIT_CNT_SHORT_WAIT_SHIFT;
	if (cnt < reg)
		cnt = reg;

	reg = (next_timing->burst_regs[EMC_MRS_WAIT_CNT_INDEX] &
		(~EMC_MRS_WAIT_CNT_LONG_WAIT_MASK));
	reg |= (cnt << EMC_MRS_WAIT_CNT_LONG_WAIT_SHIFT) &
		EMC_MRS_WAIT_CNT_LONG_WAIT_MASK;

	emc_writel(reg, EMC_MRS_WAIT_CNT);
}

static inline void set_dram_mode(const struct emc_table *next_timing,
				 const struct emc_table *last_timing,
				 int dll_change)
{
	if (tegra_dram_type == DRAM_TYPE_DDR3) {
		if (next_timing->emc_mode_1 != last_timing->emc_mode_1)
			ccfifo_writel(next_timing->emc_mode_1, EMC_EMRS);
		if (next_timing->emc_mode_2 != last_timing->emc_mode_2)
			ccfifo_writel(next_timing->emc_mode_2, EMC_EMRS2);

		if ((next_timing->emc_mode_reset != last_timing->emc_mode_reset)
			|| (dll_change == DLL_CHANGE_ON)) {
			u32 reg = next_timing->emc_mode_reset &
				(~EMC_MODE_SET_DLL_RESET);
			if (dll_change == DLL_CHANGE_ON) {
				reg |= EMC_MODE_SET_DLL_RESET;
				reg |= EMC_MODE_SET_LONG_CNT;
			}
			ccfifo_writel(reg, EMC_MRS);
		}
	} else {
		if (next_timing->emc_mode_2 != last_timing->emc_mode_2)
			ccfifo_writel(next_timing->emc_mode_2, EMC_MRW2);
		if (next_timing->emc_mode_1 != last_timing->emc_mode_1)
			ccfifo_writel(next_timing->emc_mode_1, EMC_MRW);
		if (next_timing->emc_mode_4 != last_timing->emc_mode_4)
			ccfifo_writel(next_timing->emc_mode_4, EMC_MRW4);
	}
}

static inline void do_clock_change(u32 clk_setting)
{
	int err;

	tegra124_mc_readl(MC_EMEM_ADR_CFG);
	emc_readl(EMC_INTSTATUS);
	clk_cfg_writel(clk_setting);
	clk_cfg_readl();

	err = wait_for_update(EMC_INTSTATUS, EMC_INTSTATUS_CLKCHANGE_COMPLETE,
		true);
	if (err) {
		pr_err("%s: clock change completion error: %d", __func__, err);
		BUG();
	}
}

static void emc_set_clock(const struct emc_table *next_timing,
				   const struct emc_table *last_timing,
				   u32 clk_setting)
{
	int i, dll_change, pre_wait, ctt_term_changed;
	bool cfg_pow_features_enabled, zcal_long;
	u32 auto_cal_config;

	u32 emc_cfg_reg = emc_readl(EMC_CFG);
	u32 emc_cfg_2_reg = emc_readl(EMC_CFG_2);
	u32 sel_dpd_ctrl = emc_readl(EMC_SEL_DPD_CTRL);
	u32 auto_cal_status = emc_readl(EMC_AUTO_CAL_STATUS);

	cfg_pow_features_enabled = (emc_cfg_reg & EMC_CFG_PWR_MASK);
	dll_change = get_dll_change(next_timing, last_timing);
	zcal_long = (next_timing->burst_regs[EMC_ZCAL_INTERVAL_INDEX] != 0)
		&& (last_timing->burst_regs[EMC_ZCAL_INTERVAL_INDEX] == 0);

	/* 1. clear clkchange_complete interrupts */
	emc_writel(EMC_INTSTATUS_CLKCHANGE_COMPLETE, EMC_INTSTATUS);

	/* 2. disable dynamic self-refresh and preset dqs vref, then wait for
	   possible self-refresh entry/exit and/or dqs vref settled - waiting
	   before the clock change decreases worst case change stall time */
	pre_wait = 0;
	if (cfg_pow_features_enabled) {
		emc_cfg_reg  = disable_power_features(emc_cfg_reg);
		emc_writel(emc_cfg_reg, EMC_CFG);
		pre_wait = PRE_WAIT_SREF_US;
	}

	/* 2.1 disable sel_dpd_ctrl before starting clock change */
	if (emc_sel_dpd_ctrl_enabled(sel_dpd_ctrl)) {
		sel_dpd_ctrl = disable_emc_sel_dpd_ctrl(sel_dpd_ctrl);
		emc_writel(sel_dpd_ctrl, EMC_SEL_DPD_CTRL);
	}

	/* 2.5 check dq/dqs vref delay */
	if (bgbias_preset(next_timing, last_timing)) {
		if (pre_wait < PRE_WAIT_BGBIAS_US)
			pre_wait = PRE_WAIT_BGBIAS_US;
	}

	if (dqs_preset(next_timing, last_timing)) {
		if (pre_wait < PRE_WAIT_DQS_US)
			pre_wait = PRE_WAIT_DQS_US;
	}

	if (pre_wait) {
		emc_timing_update();
		udelay(pre_wait);
	}

	/* 2.5.1 Disable auto_cal for clock change*/
	emc_writel(0, EMC_AUTO_CAL_INTERVAL);
	auto_cal_config = emc_readl(EMC_AUTO_CAL_CONFIG);
	auto_cal_status = emc_readl(EMC_AUTO_CAL_STATUS);

	if ((next_timing->emc_auto_cal_config &
	     EMC_AUTO_CAL_CONFIG_AUTO_CAL_START) &&
	    !(auto_cal_status & EMC_AUTO_CAL_STATUS_ACTIVE)) {
		auto_cal_config |= EMC_AUTO_CAL_CONFIG_AUTO_CAL_START;
		emc_writel(auto_cal_config, EMC_AUTO_CAL_CONFIG);
	}

	/* 2.6 Program CTT_TERM Control if it changed since last time*/
	ctt_term_changed = (last_timing->emc_ctt_term_ctrl
				!= next_timing->emc_ctt_term_ctrl);
	if (last_timing->emc_ctt_term_ctrl != next_timing->emc_ctt_term_ctrl) {
		auto_cal_disable();
		emc_writel(next_timing->emc_ctt_term_ctrl, EMC_CTT_TERM_CTRL);
	}

	if (ctt_term_changed)
		emc_timing_update();

	/* 4. program burst shadow registers */
	for (i = 0; i < next_timing->burst_regs_num; i++) {
		if (!burst_reg_addr[i])
			continue;
		burst_reg_writel(next_timing->burst_regs[i], i);
	}

	emc_cfg_reg = disable_power_features(next_timing->emc_cfg);
	ccfifo_writel(emc_cfg_reg, EMC_CFG);

	/* 4.1 program auto_cal_config registers */
	if (last_timing->emc_auto_cal_config2 !=
		 next_timing->emc_auto_cal_config2)
		ccfifo_writel(next_timing->emc_auto_cal_config2,
			EMC_AUTO_CAL_CONFIG2);
	if (last_timing->emc_auto_cal_config3 !=
		next_timing->emc_auto_cal_config3)
		ccfifo_writel(next_timing->emc_auto_cal_config3,
			EMC_AUTO_CAL_CONFIG3);
	if (last_timing->emc_auto_cal_config !=
		next_timing->emc_auto_cal_config) {
		auto_cal_config = next_timing->emc_auto_cal_config;
		auto_cal_config &= ~EMC_AUTO_CAL_CONFIG_AUTO_CAL_START;
		ccfifo_writel(auto_cal_config, EMC_AUTO_CAL_CONFIG);
	}

	wmb();
	barrier();

	/* 4.1 On ddr3 when DLL is re-started predict MRS long wait count and
	   overwrite DFS table setting */
	if ((tegra_dram_type == DRAM_TYPE_DDR3)
		&& (dll_change == DLL_CHANGE_ON))
		overwrite_mrs_wait_cnt(next_timing, zcal_long);

	/* 5.3 post cfg_2 write and dis ob clock gate */
	emc_cfg_2_reg = next_timing->emc_cfg_2;

	if (emc_cfg_2_reg & EMC_CFG_2_DIS_STP_OB_CLK_DURING_NON_WR)
		emc_cfg_2_reg &= ~EMC_CFG_2_DIS_STP_OB_CLK_DURING_NON_WR;
	ccfifo_writel(emc_cfg_2_reg, EMC_CFG_2);

	/* 6. turn Off dll and enter self-refresh on DDR3 */
	if (tegra_dram_type == DRAM_TYPE_DDR3) {
		if (dll_change == DLL_CHANGE_OFF)
			ccfifo_writel(next_timing->emc_mode_1, EMC_EMRS);
	}

	/* 6.1, disable refresh controller using ccfifo */
	ccfifo_writel(EMC_REFCTRL_DISABLE_ALL(tegra_dram_dev_num), EMC_REFCTRL);
	if (tegra_dram_type == DRAM_TYPE_DDR3) {
		ccfifo_writel(DRAM_BROADCAST(tegra_dram_dev_num) |
			EMC_SELF_REF_CMD_ENABLED, EMC_SELF_REF);
	}

	/* 7. flow control marker 2 */
	ccfifo_writel(1, EMC_STALL_THEN_EXE_AFTER_CLKCHANGE);

	/* 8. exit self-refresh on DDR3 */
	if (tegra_dram_type == DRAM_TYPE_DDR3)
		ccfifo_writel(DRAM_BROADCAST(tegra_dram_dev_num), EMC_SELF_REF);
	ccfifo_writel(EMC_REFCTRL_ENABLE_ALL(tegra_dram_dev_num), EMC_REFCTRL);

	/* 9. set dram mode registers */
	set_dram_mode(next_timing, last_timing, dll_change);

	/* 10. issue zcal command if turning zcal On */
	if (zcal_long) {
		ccfifo_writel(EMC_ZQ_CAL_LONG_CMD_DEV0, EMC_ZQ_CAL);
		if (tegra_dram_dev_num > 1)
			ccfifo_writel(EMC_ZQ_CAL_LONG_CMD_DEV1, EMC_ZQ_CAL);
	}

	/* 10.1 dummy write to RO register to remove stall after change */
	ccfifo_writel(0, EMC_CCFIFO_STATUS);

	/* 11.1 DIS_STP_OB_CLK_DURING_NON_WR ->0 */
	if (next_timing->emc_cfg_2 & EMC_CFG_2_DIS_STP_OB_CLK_DURING_NON_WR) {
		emc_cfg_2_reg = next_timing->emc_cfg_2;
		ccfifo_writel(emc_cfg_2_reg, EMC_CFG_2);
	}

	/* 11.2 disable auto_cal for clock change */
	wait_auto_cal_disable();

	/* 11.5 program burst_up_down registers if emc rate is going down */
	if (next_timing->rate < last_timing->rate) {
		for (i = 0; i < next_timing->up_down_regs_num; i++)
			tegra124_mc_writel(next_timing->up_down_regs[i],
				burst_up_down_reg_addr[i]);
		wmb();
	}

	/* 12-14. read any MC register to ensure the programming is done
	   change EMC clock source register wait for clk change completion */
	do_clock_change(clk_setting);

	/* 14.2 program burst_up_down registers if emc rate is going up */
	if (next_timing->rate > last_timing->rate) {
		for (i = 0; i < next_timing->up_down_regs_num; i++)
			tegra124_mc_writel(next_timing->up_down_regs[i],
				burst_up_down_reg_addr[i]);
		wmb();
	}

	/* 15. set auto-cal interval */
	if (last_timing->emc_ctt_term_ctrl != next_timing->emc_ctt_term_ctrl)
		emc_writel(next_timing->emc_acal_interval,
			EMC_AUTO_CAL_INTERVAL);

	/* 16. restore dynamic self-refresh */
	if (next_timing->emc_cfg & EMC_CFG_PWR_MASK) {
		emc_cfg_reg = next_timing->emc_cfg;
		emc_writel(emc_cfg_reg, EMC_CFG);
	}

	/* 17. set zcal wait count */
	emc_writel(next_timing->emc_zcal_cnt_long, EMC_ZCAL_WAIT_CNT);

	/* 17.1 turning of bgbias if lpddr3 dram and freq is low */
	auto_cal_config = emc_readl(EMC_AUTO_CAL_STATUS);
	if (tegra_dram_type == DRAM_TYPE_DDR3) {
		if (emc_readl(EMC_BGBIAS_CTL0) != next_timing->emc_bgbias_ctl0)
			emc_writel(next_timing->emc_bgbias_ctl0,
				EMC_BGBIAS_CTL0);
	}
	emc_writel(next_timing->emc_acal_interval, EMC_AUTO_CAL_INTERVAL);

	/* 18. update restored timing */
	udelay(2);
	emc_writel(next_timing->emc_sel_dpd_ctrl, EMC_SEL_DPD_CTRL);
	emc_timing_update();
}

static void emc_last_stats_update(int last_sel)
{
	unsigned long flags;
	u64 cur_jiffies = get_jiffies_64();

	spin_lock_irqsave(&tegra_emc_stats.spinlock, flags);

	if (tegra_emc_stats.last_sel < TEGRA_EMC_TABLE_MAX_SIZE)
		tegra_emc_stats.time_at_clock[tegra_emc_stats.last_sel] =
			tegra_emc_stats.time_at_clock[tegra_emc_stats.last_sel]
			+ (cur_jiffies - tegra_emc_stats.last_update);

	tegra_emc_stats.last_update = cur_jiffies;

	if (last_sel < TEGRA_EMC_TABLE_MAX_SIZE) {
		tegra_emc_stats.clkchange_count++;
		tegra_emc_stats.last_sel = last_sel;
	}
	spin_unlock_irqrestore(&tegra_emc_stats.spinlock, flags);
}

static int emc_table_lookup(unsigned long rate)
{
	int i;
	i = get_start_idx(rate);
	for (; i < tegra_emc_table_size; i++) {
		if (tegra_emc_table[i].input == NULL)
			continue;

		if (tegra_emc_table[i].rate == rate)
			break;
	}

	if (i >= tegra_emc_table_size)
		return -EINVAL;
	return i;
}

static int tegra124_emc_set_rate(unsigned long rate)
{
	int i;
	u32 clk_setting;
	struct emc_table *last_timing;
	unsigned long flags;
	s64 last_change_delay;

	if (!emc_enable)
		return -ENODEV;

	if (!tegra_emc_init_done || !tegra_emc_table_size)
		return -EINVAL;

	if (rate == tegra124_emc_get_rate())
		return 0;

	i = emc_table_lookup(rate);

	if (IS_ERR_VALUE(i))
		return i;

	if (!emc_timing) {
		emc_get_timing(&start_timing);
		last_timing = &start_timing;
	} else
		last_timing = emc_timing;

	clk_setting = tegra_emc_table[i].value;

	last_change_delay = ktime_us_delta(ktime_get(), clkchange_time);
	if ((last_change_delay >= 0) && (last_change_delay < clkchange_delay))
		udelay(clkchange_delay - (int)last_change_delay);

	spin_lock_irqsave(&emc_access_lock, flags);
	emc_set_clock(&tegra_emc_table[i], last_timing, clk_setting);
	clkchange_time = ktime_get();
	emc_timing = &tegra_emc_table[i];
	spin_unlock_irqrestore(&emc_access_lock, flags);

	emc_last_stats_update(i);

	return 0;
}

static struct clk *tegra124_emc_predict_parent(unsigned long rate,
						unsigned long *parent_rate)
{
	int val;
	u32 src_val;

	if (!tegra_emc_table)
		return ERR_PTR(-EINVAL);

	val = emc_table_lookup(rate);
	if (IS_ERR_VALUE(val))
		return ERR_PTR(val);

	*parent_rate = tegra_emc_table[val].input_rate;
	src_val = emc_src_val(tegra_emc_table[val].src_sel_reg);
	return tegra_emc_src[src_val];
}

static void tegra124_emc_get_backup_parent(struct clk **backup_parent,
						unsigned long *backup_rate)
{
	*backup_parent = emc_backup_src;
	*backup_rate = emc_backup_rate;
}

static const struct emc_clk_ops tegra124_emc_clk_ops = {
	.emc_get_rate = tegra124_emc_get_rate,
	.emc_set_rate = tegra124_emc_set_rate,
	.emc_round_rate = tegra124_emc_round_rate,
	.emc_predict_parent = tegra124_emc_predict_parent,
	.emc_get_backup_parent = tegra124_emc_get_backup_parent,
};

const struct emc_clk_ops *tegra124_emc_get_ops(void)
{
	return &tegra124_emc_clk_ops;
}
EXPORT_SYMBOL(tegra124_emc_get_ops);

static int find_matching_input(struct emc_table *table)
{
	u32 div_value;
	u32 src_value;
	unsigned long input_rate = 0;
	struct clk *input_clk;

	div_value = emc_div_val(table->src_sel_reg);
	src_value = emc_src_val(table->src_sel_reg);

	if (div_value & 0x1) {
		pr_warn("Tegra EMC: invalid odd divider for EMC rate %lu\n",
			table->rate);
		return -EINVAL;
	}

	if (src_value >= __clk_get_num_parents(emc_clk)) {
		pr_warn("Tegra EMC: no matching input found for rate %lu\n",
			table->rate);
		return -EINVAL;
	}

	if (div_value && (table->src_sel_reg & EMC_CLK_LOW_JITTER_ENABLE)) {
		pr_warn("Tegra EMC: invalid LJ path for EMC rate %lu\n",
			table->rate);
		return -EINVAL;
	}

	if (!(table->src_sel_reg & EMC_CLK_MC_SAME_FREQ) !=
		!(MC_EMEM_ARB_MISC0_EMC_SAME_FREQ &
		table->burst_regs[MC_EMEM_ARB_MISC0_INDEX])) {
		pr_warn("Tegra EMC: ambiguous EMC to MC ratio for rate %lu\n",
			table->rate);
		return -EINVAL;
	}

	input_clk = tegra_emc_src[src_value];
	if (src_value == TEGRA_EMC_SRC_PLLM_UD ||
					src_value == TEGRA_EMC_SRC_PLLM)
		input_rate = table->rate * (1 + div_value / 2);
	else {
		input_rate = clk_get_rate(input_clk);
		if (input_rate != (table->rate * (1 + div_value / 2))) {
			pr_warn("Tegra EMC: rate %lu doesn't match input\n",
				table->rate);
			return -EINVAL;
		}
	}

	if (IS_ERR(emc_backup_src) && (src_value == TEGRA_EMC_SRC_PLLC ||
					src_value == TEGRA_EMC_SRC_PLLC_UD)) {
		emc_backup_src = tegra_emc_src[src_value];
		emc_backup_rate = table->rate;
	}

	table->input = input_clk;
	table->input_rate = input_rate;
	table->value = table->src_sel_reg;

	return 0;
}

static void purge_emc_table(void)
{
	int i;

	if (!IS_ERR(emc_backup_src))
		return;

	for (i = 0; i < tegra_emc_table_size; i++) {
		struct emc_table *table = &tegra_emc_table[i];
		if (table->input) {
			if (__clk_get_rate(table->input) != table->input_rate) {
				table->input = NULL;
				table->input_rate = 0;
				table->value = 0;
			}
		}
	}
}

static struct device_node *tegra124_emc_find_table(struct device_node *np)
{
	struct device_node *iter;
	struct property *prop;
	const __be32 *p;
	u32 u;
	bool use_ram_code = false;

	for_each_child_of_node(np, iter) {
		if (of_find_property(iter, "nvidia,ram-code", NULL)) {
			use_ram_code = true;
			of_property_for_each_u32(iter, "nvidia,ram-code", prop,
						 p, u) {
				if (u == tegra_ram_code)
					return iter;
			}
		}
	}

	if (use_ram_code)
		return ERR_PTR(-ENODATA);

	return np;
}

static void tegra124_parse_dt_data(struct platform_device *pdev)
{
	struct device_node *iter;
	struct device_node *tablenode = NULL;
	int i;
	u32 prop;
	int ret;

	tablenode = tegra124_emc_find_table(pdev->dev.of_node);
	if (IS_ERR(tablenode))
		return;

	ret = of_property_read_u32(tablenode, "max-clock-frequency", &prop);
	if (!ret)
		emc_max_rate = prop * 1000;

	i = 0;
	for_each_child_of_node(tablenode, iter)
		if (of_device_is_compatible(iter, "nvidia,tegra12-emc-table"))
			i++;
	if (!i)
		return;

	tegra_emc_table = devm_kzalloc(&pdev->dev,
		sizeof(struct emc_table) * i, GFP_KERNEL);
	if (!tegra_emc_table)
		return;

	i = 0;
	for_each_child_of_node(tablenode, iter) {
		ret = of_property_read_u32(iter, "nvidia,revision",
			&tegra_emc_table[i].rev);
		if (ret)
			continue;
		if (tegra_emc_table[i].rev < 0x19)
			continue;
		ret = of_property_read_u32(iter, "nvidia,src-sel-reg",
			&tegra_emc_table[i].src_sel_reg);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,emc-min-mv",
			&tegra_emc_table[i].emc_min_mv);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,gk20a-min-mv",
			&tegra_emc_table[i].gk20a_min_mv);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,emc-zcal-cnt-long",
			&tegra_emc_table[i].emc_zcal_cnt_long);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,emc-acal-interval",
			&tegra_emc_table[i].emc_acal_interval);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,emc-ctt-term-ctrl",
			&tegra_emc_table[i].emc_ctt_term_ctrl);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,emc-cfg",
			&tegra_emc_table[i].emc_cfg);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,emc-cfg-2",
			&tegra_emc_table[i].emc_cfg_2);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,emc-sel-dpd-ctrl",
			&tegra_emc_table[i].emc_sel_dpd_ctrl);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,emc-cfg-dig-dll",
			&tegra_emc_table[i].emc_cfg_dig_dll);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,emc-bgbias-ctl0",
			&tegra_emc_table[i].emc_bgbias_ctl0);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,emc-auto-cal-config2",
			&tegra_emc_table[i].emc_auto_cal_config2);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,emc-auto-cal-config3",
			&tegra_emc_table[i].emc_auto_cal_config3);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,emc-auto-cal-config",
			&tegra_emc_table[i].emc_auto_cal_config);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,emc-mode-reset",
			&tegra_emc_table[i].emc_mode_reset);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,emc-mode-1",
			&tegra_emc_table[i].emc_mode_1);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,emc-mode-2",
			&tegra_emc_table[i].emc_mode_2);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,emc-mode-4",
			&tegra_emc_table[i].emc_mode_4);
		if (ret)
			continue;
		ret = of_property_read_u32(iter,
			"nvidia,emc-clock-latency-change",
			&tegra_emc_table[i].clock_change_latency);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "nvidia,burst-regs-num",
			&tegra_emc_table[i].burst_regs_num);
		if (ret)
			continue;
		ret = of_property_read_u32(iter,
			"nvidia,burst-up-down-regs-num",
			&tegra_emc_table[i].up_down_regs_num);
		if (ret)
			continue;
		tegra_emc_table[i].burst_regs = devm_kzalloc(&pdev->dev,
			sizeof(u32) * tegra_emc_table[i].burst_regs_num,
			GFP_KERNEL);
		ret = of_property_read_u32_array(iter, "nvidia,emc-registers",
			tegra_emc_table[i].burst_regs,
			tegra_emc_table[i].burst_regs_num);
		if (ret)
			continue;
		tegra_emc_table[i].up_down_regs = devm_kzalloc(&pdev->dev,
			sizeof(u32) * tegra_emc_table[i].up_down_regs_num,
			GFP_KERNEL);
		ret = of_property_read_u32_array(iter,
			"nvidia,emc-burst-up-down-regs",
			tegra_emc_table[i].up_down_regs,
			tegra_emc_table[i].up_down_regs_num);
		if (ret)
			continue;
		ret = of_property_read_u32(iter, "clock-frequency", &prop);
		if (ret)
			continue;
		tegra_emc_table[i].rate = prop * 1000;
		i++;
	}
	tegra_emc_table_size = i;
}

static int tegra124_init_emc_data(struct platform_device *pdev)
{
	int i;
	u32 val;
	unsigned long table_rate;
	unsigned long old_rate;
	int regs_count;

	emc_clk = devm_clk_get(&pdev->dev, "emc");
	if (IS_ERR(emc_clk)) {
		dev_err(&pdev->dev, "Can not find EMC clock\n");
		return -EINVAL;
	}
	emc_boot_rate = clk_get_rate(emc_clk);

	emc_override_clk = devm_clk_get(&pdev->dev, "emc_override");
	if (IS_ERR(emc_override_clk))
		dev_err(&pdev->dev, "Cannot find EMC override clock\n");

	for (i = 0; i < TEGRA_EMC_SRC_COUNT; i++) {
		tegra_emc_src[i] = devm_clk_get(&pdev->dev,
			tegra_emc_src_names[i]);
		if (IS_ERR(tegra_emc_src[i])) {
			dev_err(&pdev->dev, "Can not find EMC source clock\n");
			return -ENODATA;
		}
	}

	tegra_emc_stats.clkchange_count = 0;
	spin_lock_init(&tegra_emc_stats.spinlock);
	tegra_emc_stats.last_update = get_jiffies_64();
	tegra_emc_stats.last_sel = TEGRA_EMC_TABLE_MAX_SIZE;

	tegra_dram_type = (emc_readl(EMC_FBIO_CFG5) & EMC_CFG5_TYPE_MASK)
		>> EMC_CFG5_TYPE_SHIFT;

	tegra_dram_dev_num = (tegra124_mc_readl(MC_EMEM_ADR_CFG) & 0x1) + 1;

	if (tegra_dram_type != DRAM_TYPE_DDR3 &&
	    tegra_dram_type != DRAM_TYPE_LPDDR2) {
		dev_err(&pdev->dev, "DRAM not supported\n");
		return -ENODATA;
	}

	tegra124_parse_dt_data(pdev);

	if (!tegra_emc_table_size ||
		tegra_emc_table_size > TEGRA_EMC_TABLE_MAX_SIZE) {
		dev_err(&pdev->dev, "Error invalid table size %d\n",
			tegra_emc_table_size);
		return -EINVAL;
	}

	old_rate = clk_get_rate(emc_clk);

	emc_backup_src = ERR_PTR(-EINVAL);

	for (i = 0; i < tegra_emc_table_size; i++) {
		table_rate = tegra_emc_table[i].rate;
		if (!table_rate)
			continue;

		if (emc_max_rate && table_rate > emc_max_rate)
			break;

		if (i && (table_rate <= tegra_emc_table[i-1].rate))
			continue;

		if (find_matching_input(&tegra_emc_table[i]))
			continue;

		if (table_rate == old_rate)
			tegra_emc_stats.last_sel = i;
	}

	purge_emc_table();

	dev_info(&pdev->dev, "validated EMC DFS table\n");

	val = emc_readl(EMC_CFG_2) & (~EMC_CFG_2_MODE_MASK);
	val |= ((tegra_dram_type == DRAM_TYPE_LPDDR2) ? EMC_CFG_2_PD_MODE :
		EMC_CFG_2_SREF_MODE) << EMC_CFG_2_MODE_SHIFT;
	emc_writel(val, EMC_CFG_2);

	start_timing.burst_regs_num = ARRAY_SIZE(burst_reg_addr);
	start_timing.up_down_regs_num = ARRAY_SIZE(burst_up_down_reg_addr);

	regs_count = start_timing.burst_regs_num +
		start_timing.up_down_regs_num;
	start_timing.burst_regs = devm_kzalloc(&pdev->dev,
		sizeof(u32) * regs_count, GFP_KERNEL);

	start_timing.up_down_regs = start_timing.burst_regs +
		start_timing.burst_regs_num;

	return 0;
}

static int tegra124_emc_probe(struct platform_device *pdev)
{
	struct device_node *mc_np;
	struct platform_device *mc_dev;
	struct device_node *car_np;
	struct platform_device *car_dev;
	u32 val;
	int ret;

	mc_np = of_parse_phandle(pdev->dev.of_node, "nvidia,mc", 0);
	if (!mc_np) {
		dev_err(&pdev->dev, "Error parsing MC phandle.\n");
		return -EINVAL;
	}

	mc_dev = of_find_device_by_node(mc_np);
	if (!mc_dev) {
		dev_err(&pdev->dev, "Error finding MC device.\n");
		ret = -EINVAL;
		goto err_mc;
	}

	if (!mc_dev->dev.driver) {
		dev_err(&pdev->dev, "Error MC driver not ready.\n");
		ret = -EPROBE_DEFER;
		goto err_mc;
	}

	car_np = of_parse_phandle(pdev->dev.of_node, "clocks", 0);
	if (!car_np) {
		dev_err(&pdev->dev, "Error parsing clocks phandle.");
		ret = -EINVAL;
		goto err_mc;
	}

	car_dev = of_find_device_by_node(car_np);
	if (!car_dev) {
		dev_err(&pdev->dev, "Error finding clocks device.");
		ret = -EINVAL;
		goto err_car;
	}

	tegra_pmc_regs = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							 "nvidia,pmc");
	if (IS_ERR(tegra_pmc_regs)) {
		dev_err(&pdev->dev, "Error finding PMC regmap.");
		ret = PTR_ERR(tegra_pmc_regs);
		goto err_car;
	}
	ret = regmap_read(tegra_pmc_regs, STRAPPING_OPT_A, &val);
	if (ret < 0) {
		dev_err(&pdev->dev, "Error reading PMC reg 0x%d\n",
			STRAPPING_OPT_A);
		goto err_car;
	}

	tegra_ram_code = (val & STRAPPING_OPT_A_RAM_CODE_MASK) >>
		STRAPPING_OPT_A_RAM_CODE_SHIFT;
	dev_info(&pdev->dev, "Ram code %u\n", tegra_ram_code);

	tegra_clk_base = of_iomap(car_dev->dev.of_node, 0);
	tegra_emc_base = of_iomap(pdev->dev.of_node, 0);

	ret = tegra124_init_emc_data(pdev);
	if (ret)
		goto err_car;

	tegra_emc_init_done = true;

	return 0;

err_car:
	of_node_put(car_np);
err_mc:
	of_node_put(mc_np);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int tegra124_emc_suspend(struct device *dev)
{
	if (!IS_ERR(emc_override_clk)) {
		clk_set_rate(emc_override_clk, emc_boot_rate);
		clk_prepare_enable(emc_override_clk);
	}

	return 0;
}

static int tegra124_emc_resume(struct device *dev)
{
	if (!IS_ERR(emc_override_clk))
		clk_disable_unprepare(emc_override_clk);

	return 0;
}
#endif

static const struct dev_pm_ops tegra124_emc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tegra124_emc_suspend, tegra124_emc_resume)
};

static struct of_device_id tegra124_emc_of_match[] = {
	{ .compatible = "nvidia,tegra124-emc", },
	{ },
};

static struct platform_driver tegra124_emc_driver = {
	.driver         = {
		.name   = "tegra124-emc",
		.owner  = THIS_MODULE,
		.of_match_table = tegra124_emc_of_match,
		.pm	= &tegra124_emc_pm_ops,
	},
	.probe          = tegra124_emc_probe,
};

static int __init tegra124_emc_init(void)
{
	return platform_driver_register(&tegra124_emc_driver);
}
subsys_initcall(tegra124_emc_init);

#ifdef CONFIG_DEBUG_FS

static int emc_stats_show(struct seq_file *s, void *data)
{
	int i;

	emc_last_stats_update(TEGRA_EMC_TABLE_MAX_SIZE);

	seq_printf(s, "%-10s %-10s\n", "rate kHz", "time");
	for (i = 0; i < tegra_emc_table_size; i++) {
		if (tegra_emc_table[i].input == NULL)
			continue;

		seq_printf(s, "%-10lu %-10llu\n", tegra_emc_table[i].rate,
			cputime64_to_clock_t(tegra_emc_stats.time_at_clock[i]));
	}
	seq_printf(s, "%-15s %llu\n", "transitions:",
		tegra_emc_stats.clkchange_count);
	seq_printf(s, "%-15s %llu\n", "time-stamp:",
		cputime64_to_clock_t(tegra_emc_stats.last_update));

	return 0;
}

static int emc_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, emc_stats_show, inode->i_private);
}

static const struct file_operations emc_stats_fops = {
	.open		= emc_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init tegra_emc_debug_init(void)
{
	struct dentry *emc_debugfs_root;

	if (!tegra_emc_init_done)
		return -ENODEV;

	emc_debugfs_root = debugfs_create_dir("tegra_emc", NULL);
	if (!emc_debugfs_root)
		return -ENOMEM;

	if (!debugfs_create_file(
		"stats", S_IRUGO, emc_debugfs_root, NULL, &emc_stats_fops))
		goto err_out;

	if (!debugfs_create_u32("clkchange_delay", S_IRUGO | S_IWUSR,
		emc_debugfs_root, (u32 *)&clkchange_delay))
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(emc_debugfs_root);
	return -ENOMEM;
}

late_initcall(tegra_emc_debug_init);
#endif
