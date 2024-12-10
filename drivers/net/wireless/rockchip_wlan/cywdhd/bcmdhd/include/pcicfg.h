/*
 * pcicfg.h: PCI configuration constants and structures.
 *
 * Portions of this code are copyright (c) 2022 Cypress Semiconductor Corporation
 *
 * Copyright (C) 1999-2017, Broadcom Corporation
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id: pcicfg.h 690133 2017-03-14 21:02:02Z $
 */

#ifndef	_h_pcicfg_
#define	_h_pcicfg_

/* pci config status reg has a bit to indicate that capability ptr is present */

#define PCI_CAPPTR_PRESENT	0x0010

/* A structure for the config registers is nice, but in most
 * systems the config space is not memory mapped, so we need
 * field offsetts. :-(
 */
#define	PCI_CFG_VID		0
#define	PCI_CFG_DID		2
#define	PCI_CFG_CMD		4
#define	PCI_CFG_STAT		6
#define	PCI_CFG_REV		8
#define	PCI_CFG_PROGIF		9
#define	PCI_CFG_SUBCL		0xa
#define	PCI_CFG_BASECL		0xb
#define	PCI_CFG_CLSZ		0xc
#define	PCI_CFG_LATTIM		0xd
#define	PCI_CFG_HDR		0xe
#define	PCI_CFG_BIST		0xf
#define	PCI_CFG_BAR0		0x10
/*
* TODO: PCI_CFG_BAR1 is wrongly defined to be 0x14 whereas it should be
* 0x18 as per the PCIe full dongle spec. Need to modify the values below
* correctly at a later point of time
*/
#define	PCI_CFG_BAR1		0x14
#define	PCI_CFG_BAR2		0x18
#define	PCI_CFG_BAR3		0x1c
#define	PCI_CFG_BAR4		0x20
#define	PCI_CFG_BAR5		0x24
#define	PCI_CFG_CIS		0x28
#define	PCI_CFG_SVID		0x2c
#define	PCI_CFG_SSID		0x2e
#define	PCI_CFG_ROMBAR		0x30
#define PCI_CFG_CAPPTR		0x34
#define	PCI_CFG_INT		0x3c
#define	PCI_CFG_PIN		0x3d
#define	PCI_CFG_MINGNT		0x3e
#define	PCI_CFG_MAXLAT		0x3f
#define	PCI_CFG_DEVCTRL		0xd8
#define PCI_CFG_TLCNTRL_5	0x814

/* PCI CAPABILITY DEFINES */
#define PCI_CAP_POWERMGMTCAP_ID		0x01
#define PCI_CAP_MSICAP_ID		0x05
#define PCI_CAP_VENDSPEC_ID		0x09
#define PCI_CAP_PCIECAP_ID		0x10
#define PCI_CAP_MSIXCAP_ID		0x11

/* Data structure to define the Message Signalled Interrupt facility
 * Valid for PCI and PCIE configurations
 */
typedef struct _pciconfig_cap_msi {
	uint8	capID;
	uint8	nextptr;
	uint16	msgctrl;
	uint32	msgaddr;
} pciconfig_cap_msi;
#define MSI_ENABLE	0x1		/* bit 0 of msgctrl */

/* Data structure to define the Power managment facility
 * Valid for PCI and PCIE configurations
 */
typedef struct _pciconfig_cap_pwrmgmt {
	uint8	capID;
	uint8	nextptr;
	uint16	pme_cap;
	uint16	pme_sts_ctrl;
	uint8	pme_bridge_ext;
	uint8	data;
} pciconfig_cap_pwrmgmt;

#define PME_CAP_PM_STATES (0x1f << 27)	/* Bits 31:27 states that can generate PME */
#define PME_CSR_OFFSET	    0x4		/* 4-bytes offset */
#define PME_CSR_PME_EN	  (1 << 8)	/* Bit 8 Enable generating of PME */
#define PME_CSR_PME_STAT  (1 << 15)	/* Bit 15 PME got asserted */

/* Data structure to define the PCIE capability */
typedef struct _pciconfig_cap_pcie {
	uint8	capID;
	uint8	nextptr;
	uint16	pcie_cap;
	uint32	dev_cap;
	uint16	dev_ctrl;
	uint16	dev_status;
	uint32	link_cap;
	uint16	link_ctrl;
	uint16	link_status;
	uint32	slot_cap;
	uint16	slot_ctrl;
	uint16	slot_status;
	uint16	root_ctrl;
	uint16	root_cap;
	uint32	root_status;
} pciconfig_cap_pcie;

/* PCIE Enhanced CAPABILITY DEFINES */
#define PCIE_EXTCFG_OFFSET	0x100
#define PCIE_ADVERRREP_CAPID	0x0001
#define PCIE_VC_CAPID		0x0002
#define PCIE_DEVSNUM_CAPID	0x0003
#define PCIE_PWRBUDGET_CAPID	0x0004

/* PCIE Extended configuration */
#define PCIE_ADV_CORR_ERR_MASK	0x114
#define PCIE_ADV_CORR_ERR_MASK_OFFSET	0x14
#define CORR_ERR_RE	(1 << 0) /* Receiver  */
#define CORR_ERR_BT	(1 << 6) /* Bad TLP  */
#define CORR_ERR_BD	(1 << 7) /* Bad DLLP */
#define CORR_ERR_RR	(1 << 8) /* REPLAY_NUM rollover */
#define CORR_ERR_RT	(1 << 12) /* Reply timer timeout */
#define CORR_ERR_AE	(1 << 13) /* Adviosry Non-Fital Error Mask */
#define ALL_CORR_ERRORS (CORR_ERR_RE | CORR_ERR_BT | CORR_ERR_BD | \
			 CORR_ERR_RR | CORR_ERR_RT)

/* PCIE Root Control Register bits (Host mode only) */
#define	PCIE_RC_CORR_SERR_EN		0x0001
#define	PCIE_RC_NONFATAL_SERR_EN	0x0002
#define	PCIE_RC_FATAL_SERR_EN		0x0004
#define	PCIE_RC_PME_INT_EN		0x0008
#define	PCIE_RC_CRS_EN			0x0010

/* PCIE Root Capability Register bits (Host mode only) */
#define	PCIE_RC_CRS_VISIBILITY		0x0001

/* PCIe PMCSR Register bits */
#define PCIE_PMCSR_PMESTAT		0x8000

/* Header to define the PCIE specific capabilities in the extended config space */
typedef struct _pcie_enhanced_caphdr {
	uint16	capID;
	uint16	cap_ver : 4;
	uint16	next_ptr : 12;
} pcie_enhanced_caphdr;

#define PCIE_CFG_PMCSR		0x4C
#define	PCI_BAR0_WIN		0x80	/* backplane addres space accessed by BAR0 */
#define	PCI_BAR1_WIN		0x84	/* backplane addres space accessed by BAR1 */
#define	PCI_SPROM_CONTROL	0x88	/* sprom property control */
#define	PCIE_CFG_SUBSYSTEM_CONTROL	0x88	/* used as subsystem control in PCIE devices */
#define	PCI_BAR1_CONTROL	0x8c	/* BAR1 region burst control */
#define	PCI_INT_STATUS		0x90	/* PCI and other cores interrupts */
#define	PCI_INT_MASK		0x94	/* mask of PCI and other cores interrupts */
#define PCI_TO_SB_MB		0x98	/* signal backplane interrupts */
#define PCI_BACKPLANE_ADDR	0xa0	/* address an arbitrary location on the system backplane */
#define PCI_BACKPLANE_DATA	0xa4	/* data at the location specified by above address */
#define	PCI_CLK_CTL_ST		0xa8	/* pci config space clock control/status (>=rev14) */
#define	PCI_BAR0_WIN2		0xac	/* backplane addres space accessed by second 4KB of BAR0 */
#define	PCI_GPIO_IN		0xb0	/* pci config space gpio input (>=rev3) */
#define	PCIE_CFG_DEVICE_CAPABILITY	0xb0	/* used as device capability in PCIE devices */
#define	PCI_GPIO_OUT		0xb4	/* pci config space gpio output (>=rev3) */
#define PCIE_CFG_DEVICE_CONTROL 0xb4    /* 0xb4 is used as device control in PCIE devices */
#define PCIE_DC_AER_CORR_EN		(1u << 0u)
#define PCIE_DC_AER_NON_FATAL_EN	(1u << 1u)
#define PCIE_DC_AER_FATAL_EN		(1u << 2u)
#define PCIE_DC_AER_UNSUP_EN		(1u << 3u)

#define PCI_BAR0_WIN2_OFFSET		0x1000u
#define PCIE2_BAR0_CORE2_WIN2_OFFSET	0x5000u

#define	PCI_GPIO_OUTEN		0xb8	/* pci config space gpio output enable (>=rev3) */
#define	PCI_PM_L1SS_CTRL2	0x24c	/* The L1 PM Substates Control register */

/* Private Registers */
#define	PCI_STAT_CTRL		0xa80
#define	PCI_L0_EVENTCNT		0xa84
#define	PCI_L0_STATETMR		0xa88
#define	PCI_L1_EVENTCNT		0xa8c
#define	PCI_L1_STATETMR		0xa90
#define	PCI_L1_1_EVENTCNT	0xa94
#define	PCI_L1_1_STATETMR	0xa98
#define	PCI_L1_2_EVENTCNT	0xa9c
#define	PCI_L1_2_STATETMR	0xaa0
#define	PCI_L2_EVENTCNT		0xaa4
#define	PCI_L2_STATETMR		0xaa8

#define	PCI_LINK_STATUS		0x4dc
#define	PCI_LINK_SPEED_MASK	(15u << 0u)
#define	PCI_LINK_SPEED_SHIFT	(0)
#define PCIE_LNK_SPEED_GEN1		0x1
#define PCIE_LNK_SPEED_GEN2		0x2
#define PCIE_LNK_SPEED_GEN3		0x3

#define	PCI_PL_SPARE	0x1808	/* Config to Increase external clkreq deasserted minimum time */
#define	PCI_CONFIG_EXT_CLK_MIN_TIME_MASK	(1u << 31u)
#define	PCI_CONFIG_EXT_CLK_MIN_TIME_SHIFT	(31)

#define PCI_ADV_ERR_CAP			0x100
#define	PCI_UC_ERR_STATUS		0x104
#define	PCI_UNCORR_ERR_MASK		0x108
#define PCI_UCORR_ERR_SEVR		0x10c
#define	PCI_CORR_ERR_STATUS		0x110
#define	PCI_CORR_ERR_MASK		0x114
#define	PCI_ERR_CAP_CTRL		0x118
#define	PCI_TLP_HDR_LOG1		0x11c
#define	PCI_TLP_HDR_LOG2		0x120
#define	PCI_TLP_HDR_LOG3		0x124
#define	PCI_TLP_HDR_LOG4		0x128
#define	PCI_TL_CTRL_5			0x814
#define	PCI_TL_HDR_FC_ST		0x980
#define	PCI_TL_TGT_CRDT_ST		0x990
#define	PCI_TL_SMLOGIC_ST		0x998
#define	PCI_DL_ATTN_VEC			0x1040
#define	PCI_DL_STATUS			0x1048

#define	PCI_PHY_CTL_0			0x1800
#define	PCI_SLOW_PMCLK_EXT_RLOCK	(1 << 7)

#define	PCI_LINK_STATE_DEBUG	0x1c24
#define PCI_RECOVERY_HIST		0x1ce4
#define PCI_PHY_LTSSM_HIST_0	0x1cec
#define PCI_PHY_LTSSM_HIST_1	0x1cf0
#define PCI_PHY_LTSSM_HIST_2	0x1cf4
#define PCI_PHY_LTSSM_HIST_3	0x1cf8
#define PCI_PHY_DBG_CLKREG_0	0x1e10
#define PCI_PHY_DBG_CLKREG_1	0x1e14
#define PCI_PHY_DBG_CLKREG_2	0x1e18
#define PCI_PHY_DBG_CLKREG_3	0x1e1c

/* Bit settings for PCIE_CFG_SUBSYSTEM_CONTROL register */
#define PCIE_BAR1COHERENTACCEN_BIT	8
#define PCIE_BAR2COHERENTACCEN_BIT	9
#define PCIE_SSRESET_STATUS_BIT		13
#define PCIE_SSRESET_DISABLE_BIT	14
#define PCIE_SSRESET_DIS_ENUM_RST_BIT		15

#define PCIE_BARCOHERENTACCEN_MASK	0x300

/* Bit settings for PCI_UC_ERR_STATUS register */
#define PCI_UC_ERR_URES			(1 << 20)	/* Unsupported Request Error Status */
#define PCI_UC_ERR_ECRCS		(1 << 19)	/* ECRC Error Status */
#define PCI_UC_ERR_MTLPS		(1 << 18)	/* Malformed TLP Status */
#define PCI_UC_ERR_ROS			(1 << 17)	/* Receiver Overflow Status */
#define PCI_UC_ERR_UCS			(1 << 16)	/* Unexpected Completion Status */
#define PCI_UC_ERR_CAS			(1 << 15)	/* Completer Abort Status */
#define PCI_UC_ERR_CTS			(1 << 14)	/* Completer Timeout Status */
#define PCI_UC_ERR_FCPES		(1 << 13)	/* Flow Control Protocol Error Status */
#define PCI_UC_ERR_PTLPS		(1 << 12)	/* Poisoned TLP Status */
#define PCI_UC_ERR_DLPES		(1 << 4)	/* Data Link Protocol Error Status */

#define PCI_DL_STATUS_PHY_LINKUP    (1 << 13) /* Status of LINK */

#define	PCI_PMCR_REFUP		0x1814	/* Trefup time */
#define PCI_PMCR_TREFUP_LO_MASK		0x3f
#define PCI_PMCR_TREFUP_LO_SHIFT	24
#define PCI_PMCR_TREFUP_LO_BITS		6
#define PCI_PMCR_TREFUP_HI_MASK		0xf
#define PCI_PMCR_TREFUP_HI_SHIFT	5
#define PCI_PMCR_TREFUP_HI_BITS		4
#define PCI_PMCR_TREFUP_MAX			0x400
#define PCI_PMCR_TREFUP_MAX_SCALE	0x2000

#define	PCI_PMCR_REFUP_EXT	0x1818	/* Trefup extend Max */
#define PCI_PMCR_TREFUP_EXT_SHIFT	22
#define PCI_PMCR_TREFUP_EXT_SCALE	3
#define PCI_PMCR_TREFUP_EXT_ON		1
#define PCI_PMCR_TREFUP_EXT_OFF		0

#define PCI_TPOWER_SCALE_MASK 0x3
#define PCI_TPOWER_SCALE_SHIFT 3 /* 0:1 is scale and 2 is rsvd */

#define	PCI_BAR0_SHADOW_OFFSET	(2 * 1024)	/* bar0 + 2K accesses sprom shadow (in pci core) */
#define	PCI_BAR0_SPROM_OFFSET	(4 * 1024)	/* bar0 + 4K accesses external sprom */
#define	PCI_BAR0_PCIREGS_OFFSET	(6 * 1024)	/* bar0 + 6K accesses pci core registers */
#define	PCI_BAR0_PCISBR_OFFSET	(4 * 1024)	/* pci core SB registers are at the end of the
						 * 8KB window, so their address is the "regular"
						 * address plus 4K
						 */
/*
 * PCIE GEN2 changed some of the above locations for
 * Bar0WrapperBase, SecondaryBAR0Window and SecondaryBAR0WrapperBase
 * BAR0 maps 32K of register space
*/
#define PCIE2_BAR0_WIN2		0x70 /* backplane addres space accessed by second 4KB of BAR0 */
#define PCIE2_BAR0_CORE2_WIN	0x74 /* backplane addres space accessed by second 4KB of BAR0 */
#define PCIE2_BAR0_CORE2_WIN2	0x78 /* backplane addres space accessed by second 4KB of BAR0 */
#define PCIE2_BAR0_WINSZ	0x8000

#define PCI_BAR0_WIN2_OFFSET		0x1000u
#define PCI_CORE_ENUM_OFFSET		0x2000u
#define PCI_CC_CORE_ENUM_OFFSET		0x3000u
#define PCI_SEC_BAR0_WIN_OFFSET		0x4000u
#define PCI_SEC_BAR0_WRAP_OFFSET	0x5000u
#define PCI_CORE_ENUM2_OFFSET		0x6000u
#define PCI_CC_CORE_ENUM2_OFFSET	0x7000u
#define PCI_LAST_OFFSET			0x8000u

#define PCI_BAR0_WINSZ		(16 * 1024)	/* bar0 window size Match with corerev 13 */
/* On pci corerev >= 13 and all pcie, the bar0 is now 16KB and it maps: */
#define	PCI_16KB0_PCIREGS_OFFSET (8 * 1024)	/* bar0 + 8K accesses pci/pcie core registers */
#define	PCI_16KB0_CCREGS_OFFSET	(12 * 1024)	/* bar0 + 12K accesses chipc core registers */
#define PCI_16KBB0_WINSZ	(16 * 1024)	/* bar0 window size */
#define PCI_SECOND_BAR0_OFFSET	(16 * 1024)	/* secondary  bar 0 window */

/* On AI chips we have a second window to map DMP regs are mapped: */
#define	PCI_16KB0_WIN2_OFFSET	(4 * 1024)	/* bar0 + 4K is "Window 2" */

/* PCI_INT_STATUS */
#define	PCI_SBIM_STATUS_SERR	0x4	/* backplane SBErr interrupt status */

/* PCI_INT_MASK */
#define	PCI_SBIM_SHIFT		8	/* backplane core interrupt mask bits offset */
#define	PCI_SBIM_MASK		0xff00	/* backplane core interrupt mask */
#define	PCI_SBIM_MASK_SERR	0x4	/* backplane SBErr interrupt mask */
#define	PCI_CTO_INT_SHIFT	16	/* backplane SBErr interrupt mask */
#define	PCI_CTO_INT_MASK	(1 << PCI_CTO_INT_SHIFT)	/* backplane SBErr interrupt mask */

/* PCI_SPROM_CONTROL */
#define SPROM_SZ_MSK		0x02	/* SPROM Size Mask */
#define SPROM_LOCKED		0x08	/* SPROM Locked */
#define	SPROM_BLANK		0x04	/* indicating a blank SPROM */
#define SPROM_WRITEEN		0x10	/* SPROM write enable */
#define SPROM_BOOTROM_WE	0x20	/* external bootrom write enable */
#define SPROM_BACKPLANE_EN	0x40	/* Enable indirect backplane access */
#define SPROM_OTPIN_USE		0x80	/* device OTP In use */
#define SPROM_CFG_TO_SB_RST	0x400	/* backplane reset */

/* Bits in PCI command and status regs */
#define PCI_CMD_IO		0x00000001	/* I/O enable */
#define PCI_CMD_MEMORY		0x00000002	/* Memory enable */
#define PCI_CMD_MASTER		0x00000004	/* Master enable */
#define PCI_CMD_SPECIAL		0x00000008	/* Special cycles enable */
#define PCI_CMD_INVALIDATE	0x00000010	/* Invalidate? */
#define PCI_CMD_VGA_PAL		0x00000040	/* VGA Palate */
#define PCI_STAT_TA		0x08000000	/* target abort status */

/* Header types */
#define	PCI_HEADER_MULTI	0x80
#define	PCI_HEADER_MASK		0x7f
typedef enum {
	PCI_HEADER_NORMAL,
	PCI_HEADER_BRIDGE,
	PCI_HEADER_CARDBUS
} pci_header_types;

#define PCI_CONFIG_SPACE_SIZE	256

#define DWORD_ALIGN(x)  (x & ~(0x03))
#define BYTE_POS(x) (x & 0x3)
#define WORD_POS(x) (x & 0x1)

#define BYTE_SHIFT(x)  (8 * BYTE_POS(x))
#define WORD_SHIFT(x)  (16 * WORD_POS(x))

#define BYTE_VAL(a, x) ((a >> BYTE_SHIFT(x)) & 0xFF)
#define WORD_VAL(a, x) ((a >> WORD_SHIFT(x)) & 0xFFFF)

#define read_pci_cfg_byte(a) \
	(BYTE_VAL(OSL_PCI_READ_CONFIG(osh, DWORD_ALIGN(a), 4), a) & 0xff)

#define read_pci_cfg_word(a) \
	(WORD_VAL(OSL_PCI_READ_CONFIG(osh, DWORD_ALIGN(a), 4), a) & 0xffff)

#define write_pci_cfg_byte(a, val) do { \
	uint32 tmpval; \
	tmpval = (OSL_PCI_READ_CONFIG(osh, DWORD_ALIGN(a), 4) & ~0xFF << BYTE_POS(a)) | \
	        val << BYTE_POS(a); \
	OSL_PCI_WRITE_CONFIG(osh, DWORD_ALIGN(a), 4, tmpval); \
	} while (0)

#define write_pci_cfg_word(a, val) do { \
	uint32 tmpval; \
	tmpval = (OSL_PCI_READ_CONFIG(osh, DWORD_ALIGN(a), 4) & ~0xFFFF << WORD_POS(a)) | \
	        val << WORD_POS(a); \
	OSL_PCI_WRITE_CONFIG(osh, DWORD_ALIGN(a), 4, tmpval); \
	} while (0)

#endif	/* _h_pcicfg_ */
