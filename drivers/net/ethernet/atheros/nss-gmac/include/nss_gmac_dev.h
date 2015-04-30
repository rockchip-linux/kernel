/*
 **************************************************************************
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 **************************************************************************
 */
/*
 * @file
 * This file defines the function prototypes for the NSS GMAC device.
 * Since the phy register mapping are standardised, the phy register map and the
 * bit definitions remain the same for other phy as well.
 * This also defines some of the Ethernet related parmeters.
 *  ---------------------------REVISION HISTORY---------------------------------
 * Qualcomm Atheros		01/Mar/2013		Modified for QCA NSS
 * Synopsys			01/Aug/2007			Created
 */

#ifndef __NSS_GMAC_DEV_H__
#define __NSS_GMAC_DEV_H__

#include <linux/if_vlan.h>
#include <linux/platform_device.h>
#include <linux/ethtool.h>
#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>
#include <asm/io.h>

#ifdef CONFIG_OF
#include <msm_nss_gmac.h>
#else
#include <mach/msm_nss_gmac.h>
#endif
#include <nss_gmac_api_if.h>


#define NSS_GMAC_IPC_OFFLOAD

#define NSS_GMAC_MACBASE	0x0000	/* Offset of Mac registers within
					   GMAC register space                */
#define NSS_GMAC_DMABASE	0x1000	/* Offset of Dma registers within
					   GMAC register space                */
#define NSS_GMAC_REG_BLOCK_LEN	0x4000	/* Length of the register block to map*/

#define NSS_GMAC_TX_DESC_SIZE	128	/* Tx Descriptors needed in the
					   Descriptor pool/queue              */
#define NSS_GMAC_RX_DESC_SIZE	128	/* Rx Descriptors needed in the
					   Descriptor pool/queue              */
#define DEFAULT_DELAY_VARIABLE  10
#define DEFAULT_LOOP_VARIABLE   10
#define MDC_CLK_DIV             (gmii_csr_clk0)

#define NSS_GMAC_EXTRA			NET_IP_ALIGN
#define NSS_GMAC_JUMBO_MTU		9600	/* Max jumbo frame size	*/

/* Max size of buffer that can be programed into one field of desc */
#define NSS_GMAC_MAX_DESC_BUFF		0x1FFF
#define NSS_GMAC_RTL_VER		"(3.72a)"

/* Ethtool specific list of GMAC supported features */
#define NSS_GMAC_SUPPORTED_FEATURES	(SUPPORTED_10baseT_Half		\
					| SUPPORTED_10baseT_Full	\
					| SUPPORTED_100baseT_Half	\
					| SUPPORTED_100baseT_Full	\
					| SUPPORTED_1000baseT_Full	\
					| SUPPORTED_Autoneg		\
					| SUPPORTED_TP			\
					| SUPPORTED_Pause		\
					| SUPPORTED_Asym_Pause)

/* Ethtool specific list of GMAC advertised features */
#define NSS_GMAC_ADVERTISED_FEATURES	(ADVERTISED_10baseT_Half	\
					| ADVERTISED_10baseT_Full	\
					| ADVERTISED_100baseT_Half	\
					| ADVERTISED_100baseT_Full	\
					| ADVERTISED_1000baseT_Full	\
					| ADVERTISED_Autoneg		\
					| ADVERTISED_TP			\
					| ADVERTISED_Pause		\
					| ADVERTISED_Asym_Pause)

/* MDIO address space register offsets */
#define ATH_MII_MMD_ACCESS_CTRL				0xD
#define ATH_MII_MMD_ACCESS_ADDR_DATA			0xE

/* MMD device addresses */
#define ATH_MMD_DEVADDR_3				3
#define ATH_MMD_DEVADDR_7				7

static const uint8_t nss_gmac_driver_string[] =
	"NSS GMAC Driver for RTL v" NSS_GMAC_RTL_VER;
static const uint8_t nss_gmac_driver_version[] = "1.0";
static const uint8_t nss_gmac_copyright[] =
	"Copyright (c) 2013-2014 The Linux Foundation. All rights reserved.";

/**
 * @brief DMA Descriptor Structure
 * The structure is common for both receive and transmit descriptors
 * The descriptor is of 4 words, but our structrue contains 6 words where last
 * two words are to hold the virtual address of the network buffer pointers for
 * driver's use.From the GMAC core release 3.50a onwards, the Enhanced
 * Descriptor structure got changed. The descriptor (both transmit and receive)
 * are of 8 words each rather the 4 words of normal descriptor structure.
 * Whenever IEEE 1588 Timestamping is enabled TX/RX DESC6 provides the lower 32
 * bits of Timestamp value and TX/RX DESC7 provides the upper 32 bits of
 * Timestamp value.
 * In addition to this whenever extended status bit is set (RX DESC0 bit 0),
 * RX DESC4 contains the extended status information.
 */
struct dma_desc {
	uint32_t status;	/* Status                             */
	uint32_t length;	/* Buffer 1  and Buffer 2 length      */
	uint32_t buffer1;	/* Network Buffer 1 pointer (Dma-able)*/
	uint32_t data1;		/* This holds virtual address of
				   buffer1, not used by DMA	      */

	/* This data below is used only by driver */
	uint32_t extstatus;	/* Extended status of a Rx Descriptor */
	uint32_t reserved1;	/* Reserved word                      */
	uint32_t timestamplow;	/* Lower 32 bits of the 64
				   bit timestamp value                */
	uint32_t timestamphigh;	/* Higher 32 bits of the 64
					   bit timestamp value        */
};

enum desc_mode {
	RINGMODE = 0x00000001,
	CHAINMODE = 0x00000002,
};

#define NSS_GMAC_WORKQUEUE_NAME		"gmac_workqueue"
struct nss_gmac_global_ctx;

/**
 * @brief NSS GMAC device data
 */
struct nss_gmac_dev {
	uint32_t mac_base;	/* base address of MAC registers              */
	uint32_t dma_base;	/* base address of DMA registers              */
	uint32_t phy_base;	/* PHY device address on MII interface        */
	uint32_t macid;		/* Sequence number of Mac on the platform     */
	uint32_t version;	/* Gmac Revision version                      */
	uint32_t emulation;	/* Running on emulation platform	      */
	unsigned long int flags;/* status flags				      */
	uint32_t drv_flags;	/* Driver specific feature flags		*/

	dma_addr_t tx_desc_dma;	/* Dma-able address of first tx descriptor
				   either in ring or chain mode, this is used
				   by the GMAC device                         */

	dma_addr_t rx_desc_dma;	/* Dma-albe address of first rx descriptor
				   either in ring or chain mode, this is
				   used by the GMAC device                    */

	struct dma_desc *tx_desc;/* start address of TX descriptors ring or
				   chain, this is used by the driver          */

	struct dma_desc *rx_desc;/* start address of RX descriptors ring or
				   chain, this is used by the driver          */

	uint32_t busy_tx_desc;	/* Number of Tx Descriptors owned by
				   DMA at any given time                      */
	uint32_t busy_rx_desc;	/* Number of Rx Descriptors owned by
				   DMA at any given time                      */

	uint32_t rx_desc_count;	/* number of rx descriptors in the
				   tx descriptor queue/pool                   */
	uint32_t tx_desc_count;	/* number of tx descriptors in the
				   rx descriptor queue/pool                   */

	uint32_t tx_busy;	/* index of the tx descriptor owned by DMA,
				   is obtained by nss_gmac_get_tx_qptr()      */

	uint32_t tx_next;	/* index of the tx descriptor next available
				   with driver, given to DMA by
				   nss_gmac_set_tx_qptr()                     */

	uint32_t rx_busy;	/* index of the rx descriptor owned by DMA,
				   obtained by nss_gmac_get_rx_qptr()         */

	uint32_t rx_next;	/* index of the rx descriptor next available
				   with driver, given to DMA by
				   nss_gmac_set_rx_qptr()                     */

	struct dma_desc *tx_busy_desc;	/* Tx Descriptor address corresponding
				   to the index tx_busy                       */
	struct dma_desc *tx_next_desc;	/* Tx Descriptor address corresponding
				   to the index tx_next                       */
	struct dma_desc *rx_busy_desc;	/* Rx Descriptor address corresponding
				   to the index tx_busy                       */
	struct dma_desc *rx_next_desc;	/* Rx Descriptor address corresponding
				   to the index rx_next                       */

	/*
	 * Phy related stuff
	 */
	uint32_t mdc_clk_div;	/* Clock divider value programmed in the
				   hardware				      */
	uint32_t link_state;	/* Link status as reported by the Phy         */
	uint32_t duplex_mode;	/* Duplex mode of the Phy                     */
	uint32_t speed;		/* Speed of the Phy                           */
	uint32_t loop_back_mode;/* Loopback status of the Phy                 */
	uint32_t phy_mii_type;	/* RGMII/SGMII/QSGMII                         */
	uint32_t rgmii_delay;	/* RGMII delay settings                       */
	uint32_t pause;		/* Current flow control settings              */
	uint32_t first_linkup_done;	/* when set, it indicates that first
					   link up detection after interface
					   bring up has been done             */
	int32_t forced_speed;		/* Forced Speed */
	int32_t forced_duplex;		/* Forced Duplex */

	struct net_device *netdev;
	struct platform_device *pdev;
	struct delayed_work gmacwork;
	struct napi_struct napi;
	struct rtnl_link_stats64 stats;	/* statistics counters                */
	spinlock_t stats_lock;		/* Lock to retrieve stats atomically  */
	spinlock_t slock;		/* Lock to protect datapath	      */
	struct mutex link_mutex;	/* Lock to protect link status change */
	uint32_t gmac_power_down;	/* indicate to ISR whether the
					   interrupts occurred in the process
					   of powering down                   */

	struct nss_gmac_global_ctx *ctx;/* Global NSS GMAC context            */
	struct resource *memres;	/* memory resource                    */

	void *data_plane_ctx;		/* context when NSS owns GMACs        */
	struct phy_device *phydev;	/* Phy device			      */
	struct nss_gmac_stats nss_stats;/* Stats synced from NSS	      */
	struct mii_bus *miibus;		/* MDIO bus associated with this GMAC */
	struct nss_gmac_data_plane_ops *data_plane_ops;
					/* ops to send messages to nss-drv    */
};


/**
 * @brief Events from the NSS GMAC
 */
#define NSS_GMAC_SPEED_SET		0x0001

/**
 * @brief GMAC speed context
 */
struct nss_gmac_speed_ctx {
	uint32_t mac_id;
	uint32_t speed;
};

extern struct nss_gmac_global_ctx ctx;

/**
 * @brief GMAC driver context
 */
struct nss_gmac_global_ctx {
	struct workqueue_struct *gmac_workqueue;
	char *gmac_workqueue_name;
	uint8_t *nss_base;	/* Base address of NSS GMACs'
				   global interface registers */
	uint32_t *qsgmii_base;
	uint32_t *clk_ctl_base;	/* Base address of platform
				   clock control registers */
	spinlock_t reg_lock;	/* Lock to protect NSS register	*/
	uint32_t socver;	/* SOC version */
	struct nss_gmac_dev *nss_gmac[NSS_MAX_GMACS];
	bool common_init_done;	/* Flag to hold common init done state */
};


enum nss_gmac_state {
	__NSS_GMAC_UP,		/* set to indicate the interface is UP	      */
	__NSS_GMAC_CLOSING,	/* set to indicate the interface is closing   */
	__NSS_GMAC_RXCSUM,	/* Rx checksum enabled			      */
	__NSS_GMAC_AUTONEG,	/* Autonegotiation Enabled		      */
	__NSS_GMAC_RXPAUSE,
	__NSS_GMAC_TXPAUSE,
	__NSS_GMAC_LINKPOLL,	/* Poll link status			      */
};

/**
 * @brief Driver specific feature flags (nss_gmac_dev->drv_flags).
 * These flags are to be used with
 * {get/set}_priv_flags ethtool operations.
 */
enum nss_gmac_priv_flags {
	__NSS_GMAC_PRIV_FLAG_LINKPOLL,
	__NSS_GMAC_PRIV_FLAG_MAX,
};
#define NSS_GMAC_PRIV_FLAG(x)	(1 << __NSS_GMAC_PRIV_FLAG_ ## x)

enum mii_link_status {
	LINKDOWN = 0,
	LINKUP = 1,
};

enum mii_duplex_mode {
	HALFDUPLEX = 1,
	FULLDUPLEX = 2,
};

enum mii_link_speed {
	SPEED10 = 1,
	SPEED100 = 2,
	SPEED1000 = 3,
};

enum mii_loop_back {
	NOLOOPBACK = 0,
	LOOPBACK = 1,
};


/*
 * PHY Registers
 */
/* MDIO Managebale Device (MMD) register offsets */
enum ath_mmd_register {
	ath_mmd_smart_eee_ctrl_3 = 0x805d,	/* MMD smart EEE control 3 */
	ath_mmd_eee_adv = 0x003c,		/* MMD EEE Advertisment */
};

/* MMD Access Control function bits */
enum ath_mmd_access_ctrl_function_bit_descriptions {
	ath_mmd_acc_ctrl_addr = 0x0000,		/* address */
	ath_mmd_acc_ctrl_data_no_incr = 0x4000,	/* data, no post incr */
	ath_mmd_acc_ctrl_data_incr_rw = 0x8000,	/* data, post incr on r/w */
	ath_mmd_acc_ctrl_data_incr_w = 0xc000,	/* data, post incr on write
						   only			  */
};

/* MMD Smart EEE control 3 register bits */
enum ath_mmd_smart_eee_ctrl_bit_descriptions {
	ath_mmd_smart_eee_ctrl3_lpi_en = 0x0100,
};

/* MMD EEE Advertisment register bits */
enum ath_mmd_eee_adv_bit_descriptions {
	ath_mmd_eee_adv_100BT = 0x0002,
	ath_mmd_eee_adv_1000BT = 0x0004,
};

/**********************************************************
 * GMAC registers Map
 * For Pci based system address is BARx + gmac_register_base
 * For any other system translation is done accordingly
 **********************************************************/
enum gmac_registers {
	gmac_config = 0x0000,		/* Mac config Register                */
	gmac_frame_filter = 0x0004,	/* Mac frame filtering controls       */
	gmac_hash_high = 0x0008,	/* Multi-cast hash table high         */
	gmac_hash_low = 0x000c,		/* Multi-cast hash table low          */
	gmac_gmii_addr = 0x0010,	/* GMII address Register(ext. Phy)    */
	gmac_gmii_data = 0x0014,	/* GMII data Register(ext. Phy)       */
	gmac_flow_control = 0x0018,	/* Flow control Register              */
	gmac_vlan = 0x001c,		/* VLAN tag Register (IEEE 802.1Q)    */
	gmac_version = 0x0020,		/* GMAC Core Version Register         */
	gmac_wakeup_addr = 0x0028,	/* GMAC wake-up frame filter adrress
					   reg				      */
	gmac_pmt_ctrl_status = 0x002c,	/* PMT control and status register    */
	gmac_interrupt_status = 0x0038,	/* Mac Interrupt ststus register      */
	gmac_interrupt_mask = 0x003C,	/* Mac Interrupt Mask register        */
	gmac_addr0_high = 0x0040,	/* Mac address0 high Register         */
	gmac_addr0_low = 0x0044,	/* Mac address0 low Register          */
	gmac_addr1_high = 0x0048,	/* Mac address1 high Register         */
	gmac_addr1_low = 0x004C,	/* Mac address1 low Register          */
	gmac_addr2_high = 0x0050,	/* Mac address2 high Register         */
	gmac_addr2_low = 0x0054,	/* Mac address2 low Register          */
	gmac_addr3_high = 0x0058,	/* Mac address3 high Register         */
	gmac_addr3_low = 0x005C,	/* Mac address3 low Register          */
	gmac_addr4_high = 0x0060,	/* Mac address4 high Register         */
	gmac_addr4_low = 0x0064,	/* Mac address4 low Register          */
	gmac_addr5_high = 0x0068,	/* Mac address5 high Register         */
	gmac_addr5_low = 0x006C,	/* Mac address5 low Register          */
	gmac_addr6_high = 0x0070,	/* Mac address6 high Register         */
	gmac_addr6_low = 0x0074,	/* Mac address6 low Register          */
	gmac_addr7_high = 0x0078,	/* Mac address7 high Register         */
	gmac_addr7_low = 0x007C,	/* Mac address7 low Register          */
	gmac_addr8_high = 0x0080,	/* Mac address8 high Register         */
	gmac_addr8_low = 0x0084,	/* Mac address8 low Register          */
	gmac_addr9_high = 0x0088,	/* Mac address9 high Register         */
	gmac_addr9_low = 0x008C,	/* Mac address9 low Register          */
	gmac_addr10_high = 0x0090,	/* Mac address10 high Register        */
	gmac_addr10_low = 0x0094,	/* Mac address10 low Register         */
	gmac_addr11_high = 0x0098,	/* Mac address11 high Register        */
	gmac_addr11_low = 0x009C,	/* Mac address11 low Register         */
	gmac_addr12_high = 0x00A0,	/* Mac address12 high Register        */
	gmac_addr12_low = 0x00A4,	/* Mac address12 low Register         */
	gmac_addr13_high = 0x00A8,	/* Mac address13 high Register        */
	gmac_addr13_low = 0x00AC,	/* Mac address13 low Register         */
	gmac_addr14_high = 0x00B0,	/* Mac address14 high Register        */
	gmac_addr14_low = 0x00B4,	/* Mac address14 low Register         */
	gmac_addr15_high = 0x00B8,	/* Mac address15 high Register        */
	gmac_addr15_low = 0x00BC,	/* Mac address15 low Register         */
	gmac_mii_status = 0x00D8,	/* SGMII/RGMII/SMII Status Register   */

	/* Time Stamp Register Map */
	gmac_ts_control = 0x0700,	/* Controls the Timestamp update logic:
					   only when IEEE 1588 time stamping is
					   enabled in corekit                 */

	gmac_ts_sub_sec_incr = 0x0704,	/* 8 bit value by which sub second
					   register is incremented : only when
					   IEEE 1588 time stamping without
					   external timestamp input           */

	gmac_ts_high = 0x0708,		/* 32 bit seconds(MS): only when
					   IEEE 1588 time stamping without
					   external timestamp input           */

	gmac_ts_low = 0x070C,		/* 32 bit nano seconds(MS): only when
					   IEEE 1588 time stamping without
					   external timestamp input           */

	gmac_ts_high_update = 0x0710,	/* 32bit seconds(MS) to be written/added
					   /subtracted: only when IEEE 1588 time
					   stamping without external timestamp*/

	gmac_ts_low_update = 0x0714,	/* 32 bit nano seconds(MS) to be
					   writeen/added/subtracted: only when
					   IEEE 1588 time stamping without
					   external timestamp input           */

	gmac_ts_addend = 0x0718,	/* Used by Software to readjust the
					   clock frequency linearly: only when
					   IEEE 1588 time stamping without
					   external timestamp input           */

	gmac_ts_target_time_high = 0x071C,/* 32 bit seconds(MS) to be compared
					   with system time: only when IEEE 1588
					   time stamping without external
					   timestamp input                    */

	gmac_ts_target_time_low = 0x0720,	/* 32 bit nano seconds(MS) to be
					   compared with system time: only when
					   IEEE 1588 time stamping without
					   external timestamp input	      */

	gmac_ts_high_word = 0x0724,	/* Time Stamp Higher Word Register(Ver.
					   2 only); only lower 16 bits are
					   valid			      */

	/*gmac_ts_high_word_update    = 0x072C, */
					/* Time Stamp Higher Word Update
					   Register(Version 2 only); only lower
					   16 bits are valid                  */

	gmac_ts_status = 0x0728,	/* Time Stamp Status Register         */
};

/**********************************************************
 * GMAC Network interface registers
 * This explains the Register's Layout

 * FES is Read only by default and is enabled only when Tx
 * Config Parameter is enabled for RGMII/SGMII interface
 * during core_kit Config.

 * DM is Read only with value 1'b1 in Full duplex only Config
 **********************************************************/

/* gmac_config              = 0x0000,    Mac config Register  Layout */
enum gmac_config_reg {
	gmac_twokpe = 0x08000000,
	gmac_twokpe_enable = 0x08000000,
	gmac_twokpe_disable = 0x00000000,
	gmac_tc_enable = 0x01000000,
	gmac_watchdog = 0x00800000,
	gmac_watchdog_disable = 0x00800000,	/* (WD)Disable watchdog timer
						   on Rx		      */
	gmac_watchdog_enable = 0x00000000,	/* Enable watchdog timer      */
	gmac_jabber = 0x00400000,
	gmac_jabber_disable = 0x00400000,	/* (JD)Disable jabber timer
						   on Tx		      */
	gmac_jabber_enable = 0x00000000,	/* Enable jabber timer        */
	gmac_frame_burst = 0x00200000,
	gmac_frame_burst_enable = 0x00200000,	/* (BE)Enable frame bursting
						    during Tx		      */
	gmac_frame_burst_disable = 0x00000000,	/* Disable frame bursting     */
	gmac_jumbo_frame = 0x00100000,
	gmac_jumbo_frame_enable = 0x00100000,	/* (JE)Enable jumbo frame for
						   Tx			      */
	gmac_jumbo_frame_disable = 0x00000000,	/* Disable jumbo frame        */
	gmac_inter_frame_gap7 = 0x000E0000,	/* (IFG) Config7 - 40bit times*/
	gmac_inter_frame_gap6 = 0x000C0000,	/* (IFG) Config6 - 48bit times*/
	gmac_inter_frame_gap5 = 0x000A0000,	/* (IFG) Config5 - 56bit times*/
	gmac_inter_frame_gap4 = 0x00080000,	/* (IFG) Config4 - 64bit times*/
	gmac_inter_frame_gap3 = 0x00040000,	/* (IFG) Config3 - 72bit times*/
	gmac_inter_frame_gap2 = 0x00020000,	/* (IFG) Config2 - 80bit times*/
	gmac_inter_frame_gap1 = 0x00010000,	/* (IFG) Config1 - 88bit times*/
	gmac_inter_frame_gap0 = 0x00000000,	/* (IFG) Config0 - 96bit times*/
	gmac_disable_crs = 0x00010000,
	gmac_mii_gmii = 0x00008000,
	gmac_select_mii = 0x00008000,		/* (PS)Port Select-MII mode   */
	gmac_select_gmii = 0x00000000,		/* GMII mode                  */
	gmac_fe_speed100 = 0x00004000,		/*(FES)Fast Ethernet speed
						   100Mbps		      */
	gmac_fe_speed10 = 0x00000000,		/* 10Mbps                     */
	gmac_rx_own = 0x00002000,
	gmac_disable_rx_own = 0x00002000,	/* (DO)Disable receive own
						   packets		      */
	gmac_enable_rx_own = 0x00000000,	/* Enable receive own packets */
	gmac_loopback = 0x00001000,
	gmac_loopback_on = 0x00001000,		/* (LM)Loopback mode for
						   GMII/MII		      */
	gmac_loopback_off = 0x00000000,		/* Normal mode                */
	gmac_duplex = 0x00000800,
	gmac_full_duplex = 0x00000800,		/* (DM)Full duplex mode       */
	gmac_half_duplex = 0x00000000,		/* Half duplex mode           */
	gmac_rx_ipc_offload = 0x00000400,	/* IPC checksum offload       */
	gmac_retry = 0x00000200,
	gmac_retry_disable = 0x00000200,	/* (DR)Disable Retry          */
	gmac_retry_enable = 0x00000000,		/* Enable retransmission as per
						   BL			      */
	gmac_link_up = 0x00000100,		/* (LUD)Link UP               */
	gmac_link_down = 0x00000100,		/* Link Down                  */
	gmac_pad_crc_strip = 0x00000080,
	gmac_pad_crc_strip_enable = 0x00000080,	/* (ACS) Automatic Pad/Crc
						   strip enable		      */
	gmac_pad_crc_strip_disable = 0x00000000,/* Automatic Pad/Crc stripping
						   disable		      */
	gmac_backoff_limit = 0x00000060,
	gmac_backoff_limit3 = 0x00000060,	/* (BL)Back-off limit in HD
						   mode			      */
	gmac_backoff_limit2 = 0x00000040,
	gmac_backoff_limit1 = 0x00000020,
	gmac_backoff_limit0 = 0x00000000,
	gmac_deferral_check = 0x00000010,
	gmac_deferral_check_enable = 0x00000010,/* (DC)Deferral check enable in
						   HD mode		      */
	gmac_deferral_check_disable = 0x00000000,/* Deferral check disable    */
	gmac_tx = 0x00000008,
	gmac_tx_enable = 0x00000008,		/* (TE)Transmitter enable     */
	gmac_tx_disable = 0x00000000,		/* Transmitter disable        */
	gmac_rx = 0x00000004,
	gmac_rx_enable = 0x00000004,		/* (RE)Receiver enable        */
	gmac_rx_disable = 0x00000000,		/* Receiver disable           */
};

/* gmac_frame_filter = 0x0004,	Mac frame filtering controls Register Layout */
enum gmac_frame_filter_reg {
	gmac_filter = 0x80000000,
	gmac_filter_off = 0x80000000,		/* (RA)Receive all incoming
						   packets		      */
	gmac_filter_on = 0x00000000,		/* Receive filtered pkts only */
	gmac_hash_perfect_filter = 0x00000400,	/* Hash or Perfect Filter
						   enable		      */
	gmac_src_addr_filter = 0x00000200,
	gmac_src_addr_filter_enable = 0x00000200,	/* (SAF)Source Address
							   Filter enable      */
	gmac_src_addr_filter_disable = 0x00000000,
	gmac_src_inva_addr_filter = 0x00000100,
	gmac_src_inv_addr_filter_en = 0x00000100,	/* (SAIF)Inv Src Addr
							   Filter enable      */
	gmac_src_inv_addr_filter_dis = 0x00000000,
	gmac_pass_control = 0x000000C0,
	gmac_pass_control3 = 0x000000C0,	/* (PCS)Forwards ctrl frms that
						   pass AF		      */
	gmac_pass_control2 = 0x00000080,	/* Forwards all control frames*/
	gmac_pass_control1 = 0x00000040,	/* Don't pass control frames  */
	gmac_pass_control0 = 0x00000000,	/* Don't pass control frames  */
	gmac_broadcast = 0x00000020,
	gmac_broadcast_disable = 0x00000020,	/* (DBF)Disable Rx of broadcast
						   frames		      */
	gmac_broadcast_enable = 0x00000000,	/* Enable broadcast frames    */
	gmac_multicast_filter = 0x00000010,
	gmac_multicast_filter_off = 0x00000010,	/* (PM) Pass all multicast
						   packets		      */
	gmac_multicast_filter_on = 0x00000000,	/* Pass filtered multicast
						   packets		      */
	gmac_dest_addr_filter = 0x00000008,
	gmac_dest_addr_filter_inv = 0x00000008,	/* (DAIF)Inverse filtering for
						   DA			      */
	gmac_dest_addr_filter_nor = 0x00000000,	/* Normal filtering for DA    */
	gmac_mcast_hash_filter = 0x00000004,
	gmac_mcast_hash_filter_on = 0x00000004,	/* (HMC)perfom multicast hash
						   filtering		      */
	gmac_mcast_hash_filter_off = 0x00000000,/* perfect filtering only     */
	gmac_ucast_hash_filter = 0x00000002,
	gmac_ucast_hash_filter_on = 0x00000002,	/* (HUC)Unicast Hash filtering
						   only			      */
	gmac_ucast_hash_filter_off = 0x00000000,/* perfect filtering only     */
	gmac_promiscuous_mode = 0x00000001,
	gmac_promiscuous_mode_on = 0x00000001,	/* Receive all frames         */
	gmac_promiscuous_mode_off = 0x00000000,	/* Receive filtered packets
						   only			      */
};

/* gmac_gmii_addr = 0x0010,	GMII address Register(ext. Phy) Layout	*/
enum gmac_gmii_addr_reg {
	gmii_dev_mask = 0x0000F800,	/* (PA)GMII device address            */
	gmii_dev_shift = 11,
	gmii_reg_mask = 0x000007C0,	/* (GR)GMII register in selected Phy  */
	gmii_reg_shift = 6,
	gmii_csr_clk_shift = 2,		/* CSR Clock bit Shift                */
	gmii_csr_clk_mask = 0x0000003C,	/* CSR Clock bit Mask                 */
	gmii_csr_clk5 = 0x00000014,	/* (CR)CSR Clock Range  250-300 MHz   */
	gmii_csr_clk4 = 0x00000010,	/*                      150-250 MHz   */
	gmii_csr_clk3 = 0x0000000C,	/*                      35-60 MHz     */
	gmii_csr_clk2 = 0x00000008,	/*                      20-35 MHz     */
	gmii_csr_clk1 = 0x00000004,	/*                      100-150 MHz   */
	gmii_csr_clk0 = 0x00000000,	/*                      60-100 MHz    */
	gmii_write = 0x00000002,		/* (GW)Write to register      */
	gmii_read = 0x00000000,		/* Read from register                 */
	gmii_busy = 0x00000001,		/* (GB)GMII interface is busy         */
};

/* gmac_gmii_data = 0x0014,	GMII data Register(ext. Phy) Layout	*/
enum gmac_gmii_data_reg {
	gmii_data_mask = 0x0000FFFF,		/* (GD)GMII Data              */
};

/* gmac_flow_control = 0x0018,	Flow control Register Layout		*/
enum gmac_flow_control_reg {
	gmac_pause_time_mask = 0xFFFF0000,	/* (PT) PAUSE TIME field
						   in the control frame       */
	gmac_pause_time_shift = 16,
	gmac_pause_low_thresh = 0x00000030,
	gmac_pause_low_thresh3 = 0x00000030,	/* (PLT)thresh for pause
						   tmr 256 slot time          */
	gmac_pause_low_thresh2 = 0x00000020,	/*      144 slot time         */
	gmac_pause_low_thresh1 = 0x00000010,	/*      28 slot time          */
	gmac_pause_low_thresh0 = 0x00000000,	/*      4 slot time           */
	gmac_unicast_pause_frame = 0x00000008,
	gmac_unicast_pause_frame_on = 0x00000008,/* (UP)Detect pause frame
						   with unicast addr.         */
	gmac_unicast_pause_frame_off = 0x00000000,/* Detect only pause frame
						   with multicast addr.       */
	gmac_rx_flow_control = 0x00000004,
	gmac_rx_flow_control_enable = 0x00000004,	/* (RFE)Enable Rx flow
							   control	      */
	gmac_rx_flow_control_disable = 0x00000000,	/* Disable Rx flow
							   control	      */
	gmac_tx_flow_control = 0x00000002,
	gmac_tx_flow_control_enable = 0x00000002,	/* (TFE)Enable Tx flow
							   control	      */
	gmac_tx_flow_control_disable = 0x00000000,	/* Disable flow
							   control	      */
	gmac_flow_control_back_pressure = 0x00000001,
	gmac_send_pause_frame = 0x00000001,	/* (FCB/PBA)send pause
						   frm/Apply back pressure    */
};

/* gmac_interrupt_statusi = 0x0038,	Mac Interrupt ststus register*/
enum gmac_interrupt_status_bit_definition {
	gmac_ts_int_sts = 0x00000200,	/* set if int generated due to TS (Read
					   Time Stamp Status Register to know
					    details)			      */
	gmac_mmc_rx_chksum_offload = 0x00000080,/* set if int generated in MMC
						   RX CHECKSUM OFFLOAD int
						   register		      */
	gmac_mmc_tx_int_sts = 0x00000040,	/* set if int generated in MMC
						   TX Int register	      */
	gmac_mmc_rx_int_sts = 0x00000020,	/* set if int generated in MMC
						   RX Int register	      */
	gmac_mmc_int_sts = 0x00000010,	/* set if any of the above bit [7:5] is
					   set				      */
	gmac_pmt_int_sts = 0x00000008,	/* set whenever magic pkt/wake-on-lan
					   frame is received                  */
	gmac_pcs_an_complete = 0x00000004,	/* set when AN is complete in
						   TBI/RTBI/SGMIII phy interface
						*/
	gmac_pcs_lnk_sts_change = 0x00000002,	/* set if any lnk status change
						   in TBI/RTBI/SGMII interface*/
	gmac_rgmii_int_sts = 0x00000001,	/* set if any change in lnk
						   status of RGMII interface  */
};

/* gmac_interrupt_mask	= 0x003C,	Mac Interrupt Mask register	*/
enum gmac_interrupt_mask_bit_definition {
	gmac_tSInt_mask = 0x00000200,		/* when set disables the time
						   stamp interrupt generation */
	gmac_pmt_int_mask = 0x00000008,		/* when set Disables the
						   assertion of PMT interrupt */
	gmac_pcs_an_int_mask = 0x00000004,	/* When set disables the
						   assertion of PCS AN complete
						    interrupt		      */
	gmac_pcs_lnk_sts_int_mask = 0x00000002,	/* when set disables the
						   assertion of PCS lnk status
						   change interrupt	      */
	gmac_rgmii_int_mask = 0x00000001,	/* when set disables the
						   assertion of RGMII
						   interrupt		      */
};

/**********************************************************
 * GMAC DMA registers
 * For Pci based system address is BARx + gma_dma_base
 * For any other system translation is done accordingly
 **********************************************************/

enum dma_registers {
	dma_bus_mode = 0x0000,		/* CSR0 - Bus Mode Register	      */
	dma_tx_poll_demand = 0x0004,	/* CSR1 - TX Poll Demand Register*/
	dma_rx_poll_demand = 0x0008,	/* CSR2 - RX Poll Demand Register */
	dma_rx_base_addr = 0x000C,	/* CSR3 - RX Descriptor list base addr*/
	dma_tx_base_addr = 0x0010,	/* CSR4 - TX Descriptor list base addr*/
	dma_status = 0x0014,		/* CSR5 - Dma status Register         */
	dma_control = 0x0018,		/* CSR6 - Dma Operation Mode Register */
	dma_interrupt = 0x001C,		/* CSR7 - Interrupt enable            */
	dma_missed_fr = 0x0020,		/* CSR8 - Missed Frame & Buffer
					   overflow Counter		      */
	dma_axi_bus_mode = 0x0028,	/* AXI Bus Mode Settings	      */
	dma_tx_curr_desc = 0x0048,	/* Current host Tx Desc Register      */
	dma_rx_curr_desc = 0x004C,	/* Current host Rx Desc Register      */
	dma_tx_curr_addr = 0x0050,	/* CSR20 - Current host TX buffer addr*/
	dma_rx_curr_addr = 0x0054,	/* CSR21 - Current host RX buffer addr*/
};

/**********************************************************
 * DMA Engine registers Layout
 **********************************************************/

/* dma_bus_mode	= 0x0000,	CSR0 - Bus Mode				*/
enum dma_bus_mode_reg {
	dma_fixed_burst_enable = 0x00010000,	/* (FB)Fixed Burst SINGLE, INCR4
						   , INCR8 or INCR16          */
	dma_fixed_burst_disable = 0x00000000,	/* SINGLE, INCR               */
	dma_tx_priority_ratio11 = 0x00000000,	/* (PR)TX:RX DMA priority ratio
						   1:1			      */
	dma_tx_priority_ratio21 = 0x00004000,	/* (PR)TX:RX DMA priority ratio
						   2:1			      */
	dma_tx_priority_ratio31 = 0x00008000,	/* (PR)TX:RX DMA priority ratio
						   3:1			      */
	dma_tx_priority_ratio41 = 0x0000C000,	/* (PR)TX:RX DMA priority ratio
						   4:1			      */
	dma_address_aligned_beats = 0x02000000,	/* Address Aligned beats      */
	dma_burst_lengthx8 = 0x01000000,	/* When set mutiplies the PBL by
						   8			      */
	dma_burst_length256 = 0x01002000,	/*(dma_burst_lengthx8
						   | dma_burst_length32) = 256*/
	dma_burst_length128 = 0x01001000,	/*(dma_burst_lengthx8
						   | dma_burst_length16) = 128*/
	dma_burst_length64 = 0x01000800,	/*(dma_burst_lengthx8
						   | dma_burst_length8) = 64  */
	dma_burst_length32 = 0x00002000,	/* (PBL) programmable
						   Dma burst length = 32      */
	dma_burst_length16 = 0x00001000,	/* Dma burst length = 16      */
	dma_burst_length8 = 0x00000800,		/* Dma burst length = 8       */
	dma_burst_length4 = 0x00000400,		/* Dma burst length = 4       */
	dma_burst_length2 = 0x00000200,		/* Dma burst length = 2       */
	dma_burst_length1 = 0x00000100,		/* Dma burst length = 1       */
	dma_burst_length0 = 0x00000000,		/* Dma burst length = 0       */
	dma_descriptor8_words = 0x00000080,	/* Enh Descriptor works  1=>
						   8 word descriptor          */
	dma_descriptor4_words = 0x00000000,	/* Enh Descriptor works  0=>
						   4 word descriptor          */
	dma_descriptor_skip16 = 0x00000040,	/* (DSL)Descriptor skip
						   length (no.of dwords)      */
	dma_descriptor_skip8 = 0x00000020,	/* between two unchained
						   descriptors		      */
	dma_descriptor_skip4 = 0x00000010,
	dma_descriptor_skip2 = 0x00000008,
	dma_descriptor_skip1 = 0x00000004,
	dma_descriptor_skip0 = 0x00000000,
	dma_arbit_rr = 0x00000000,		/* (DA) DMA RR arbitration    */
	dma_arbit_pr = 0x00000002,		/* Rx has priority over Tx    */
	dma_reset_on = 0x00000001,		/* (SWR)Software Reset DMA
						   engine		      */
	dma_reset_off = 0x00000000,
};

/* dma_status	= 0x0014,	CSR5 - Dma status Register		*/
enum dma_status_reg {
	gmac_pmt_intr = 0x10000000,	/* (GPI)Gmac subsystem interrupt      */
	gmac_mmc_intr = 0x08000000,	/* (GMI)Gmac MMC subsystem interrupt  */
	gmac_line_intf_intr = 0x04000000,	/* Line interface interrupt   */
	dma_error_bit2 = 0x02000000,	/* (EB)Error bits 0-data buffer,
					   1-desc. access                     */
	dma_error_bit1 = 0x01000000,	/* (EB)Error bits 0-write trnsf,
					   1-read transfr                     */
	dma_error_bit0 = 0x00800000,	/* (EB)Error bits 0-Rx DMA, 1-Tx DMA  */
	dma_tx_state = 0x00700000,	/* (TS)Transmit process state         */
	dma_tx_stopped = 0x00000000,	/* Stopped - Reset or Stop Tx
					   Command issued                     */
	dma_tx_fetching = 0x00100000,	/* Running - fetching the Tx
					   descriptor			      */
	dma_tx_waiting = 0x00200000,	/* Running - waiting for status       */
	dma_tx_reading = 0x00300000,	/* Running - reading the data
					   from host memory                   */
	dma_tx_suspended = 0x00600000,	/* Suspended - Tx Descriptor
					   unavailabe			      */
	dma_tx_closing = 0x00700000,	/* Running - closing Rx descriptor    */
	dma_rx_state = 0x000E0000,	/* (RS)Receive process state          */
	dma_rx_stopped = 0x00000000,	/* Stopped - Reset or Stop
					   Rx Command issued                  */
	dma_rx_fetching = 0x00020000,	/* Running - fetching the Rx
					   descriptor			      */
	dma_rx_waiting = 0x00060000,	/* Running - waiting for packet       */
	dma_rx_suspended = 0x00080000,	/* Suspended - Rx Descriptor
					   unavailable			      */
	dma_rx_closing = 0x000A0000,	/* Running - closing descriptor       */
	dma_rx_queuing = 0x000E0000,	/* Running - queuing the receive
					   frame into host memory             */
	dma_int_normal = 0x00010000,	/* (NIS)Normal interrupt summary      */
	dma_int_abnormal = 0x00008000,	/* (AIS)Abnormal interrupt summary    */

	dma_int_early_rx = 0x00004000,	/* Early receive interrupt (Normal)   */
	dma_int_bus_error = 0x00002000,	/* Fatal bus error (Abnormal)         */
	dma_int_early_tx = 0x00000400,	/* Early transmit interrupt (Abnormal)*/
	dma_int_rx_wdog_to = 0x00000200,/* Receive Watchdog Timeout (Abnormal)*/
	dma_int_rx_stopped = 0x00000100,/* Receive process stopped (Abnormal) */
	dma_int_rx_no_buffer = 0x00000080,/* RX buffer unavailable (Abnormal) */
	dma_int_rx_completed = 0x00000040,/* Completion of frame RX (Normal)  */
	dma_int_tx_underflow = 0x00000020,/* Transmit underflow (Abnormal)    */
	dma_int_rcv_overflow = 0x00000010,/* RX Buffer overflow interrupt     */
	dma_int_tx_jabber_to = 0x00000008,/* TX Jabber Timeout (Abnormal)     */
	dma_int_tx_no_buffer = 0x00000004,/* TX buffer unavailable (Normal)   */
	dma_int_tx_stopped = 0x00000002,/* TX process stopped (Abnormal)      */
	dma_int_tx_completed = 0x00000001,/* Transmit completed (Normal)      */
};

/* dma_control	= 0x0018,	CSR6 - Dma Operation Mode Register	*/
enum dma_control_reg {
	dma_disable_drop_tcp_cs = 0x04000000,	/* (DT) Dis. drop. of tcp/ip
						   CS error frames            */
	dma_rx_store_and_forward = 0x02000000,	/* Rx (SF)Store and forward   */
	dma_rx_frame_flush = 0x01000000,	/* Disable Receive Frame Flush*/
	dma_store_and_forward = 0x00200000,	/* (SF)Store and forward      */
	dma_flush_tx_fifo = 0x00100000,		/* (FTF)Tx FIFO controller
						   is reset to default        */
	dma_tx_thresh_ctrl = 0x0001C000,	/* (TTC)Controls thre Threh of
						   MTL tx Fifo                */
	dma_tx_thresh_ctrl16 = 0x0001C000,	/* (TTC)Controls thre Threh of
						   MTL tx Fifo 16             */
	dma_tx_thresh_ctrl24 = 0x00018000,	/* (TTC)Controls thre Threh of
						   MTL tx Fifo 24             */
	dma_tx_thresh_ctrl32 = 0x00014000,	/* (TTC)Controls thre Threh of
						   MTL tx Fifo 32             */
	dma_tx_thresh_ctrl40 = 0x00010000,	/* (TTC)Controls thre Threh of
						   MTL tx Fifo 40             */
	dma_tx_thresh_ctrl256 = 0x0000c000,	/* (TTC)Controls thre Threh of
						   MTL tx Fifo 256            */
	dma_tx_thresh_ctrl192 = 0x00008000,	/* (TTC)Controls thre Threh of
						   MTL tx Fifo 192            */
	dma_tx_thresh_ctrl128 = 0x00004000,	/* (TTC)Controls thre Threh of
						   MTL tx Fifo 128            */
	dma_tx_thresh_ctrl64 = 0x00000000,	/* (TTC)Controls thre Threh of
						   MTL tx Fifo 64             */
	dma_tx_start = 0x00002000,		/* (ST)Start/Stop transmission*/
	dma_rx_flow_ctrl_deact = 0x00401800,	/* (RFD)Rx flow control
						   deact. threhold            */
	dma_rx_flow_ctrl_deact1K = 0x00000000,	/* (RFD)Rx flow control
						   deact. threhold (1kbytes)  */
	dma_rx_flow_ctrl_deact2K = 0x00000800,	/* (RFD)Rx flow control
						   deact. threhold (2kbytes)  */
	dma_rx_flow_ctrl_deact3K = 0x00001000,	/* (RFD)Rx flow control
						   deact. threhold (3kbytes)  */
	dma_rx_flow_ctrl_deact4K = 0x00001800,	/* (RFD)Rx flow control
						   deact. threhold (4kbytes)  */
	dma_rx_flow_ctrl_deact5K = 0x00400000,	/* (RFD)Rx flow control
						   deact. threhold (4kbytes)  */
	dma_rx_flow_ctrl_deact6K = 0x00400800,	/* (RFD)Rx flow control
						   deact. threhold (4kbytes)  */
	dma_rx_flow_ctrl_deact7K = 0x00401000,	/* (RFD)Rx flow control
						   deact. threhold (4kbytes)  */
	dma_rx_flow_ctrl_act = 0x00800600,	/* (RFA)Rx flow control
						   Act. threhold              */
	dma_rx_flow_ctrl_act1K = 0x00000000,	/* (RFA)Rx flow control
						   Act. threhold (1kbytes)    */
	dma_rx_flow_ctrl_act2K = 0x00000200,	/* (RFA)Rx flow control
						   Act. threhold (2kbytes)    */
	dma_rx_flow_ctrl_act3K = 0x00000400,	/* (RFA)Rx flow control
						   Act. threhold (3kbytes)    */
	dma_rx_flow_ctrl_act4K = 0x00000600,	/* (RFA)Rx flow control
						   Act. threhold (4kbytes)    */
	dma_rx_flow_ctrl_act5K = 0x00800000,	/* (RFA)Rx flow control
						   Act. threhold (5kbytes)    */
	dma_rx_flow_ctrl_act6K = 0x00800200,	/* (RFA)Rx flow control
						   Act. threhold (6kbytes)    */
	dma_rx_flow_ctrl_act7K = 0x00800400,	/* (RFA)Rx flow control
						   Act. threhold (7kbytes)    */
	dma_rx_thresh_ctrl = 0x00000018,	/* (RTC)Controls thre
						   Threh of MTL rx Fifo       */
	dma_rx_thresh_ctrl64 = 0x00000000,	/* (RTC)Controls thre
						   Threh of MTL tx Fifo 64    */
	dma_rx_thresh_ctrl32 = 0x00000008,	/* (RTC)Controls thre
						   Threh of MTL tx Fifo 32    */
	dma_rx_thresh_ctrl96 = 0x00000010,	/* (RTC)Controls thre
						   Threh of MTL tx Fifo 96    */
	dma_rx_thresh_ctrl128 = 0x00000018,	/* (RTC)Controls thre
						   Threh of MTL tx Fifo 128   */
	dma_en_hw_flow_ctrl = 0x00000100,	/* (EFC)Enable HW flow control*/
	dma_dis_hw_flow_ctrl = 0x00000000,	/* Disable HW flow control    */
	dma_fwd_error_frames = 0x00000080,	/* (FEF)Forward error frames  */
	dma_fwd_under_sz_frames = 0x00000040,	/* (FUF)Forward undersize
						   frames		      */
	dma_tx_second_frame = 0x00000004,	/* (OSF)Operate on 2nd frame  */
	dma_rx_start = 0x00000002,		/* (SR)Start/Stop reception   */
};

/* dma_interrupt = 0x001C,	CSR7 - Interrupt enable Register Layout	*/
enum dma_interrupt_reg {
	dma_ie_normal = dma_int_normal,		/* Normal interrupt enable    */
	dma_ie_abnormal = dma_int_abnormal,	/* Abnormal interrupt enable  */
	dma_ie_early_rx = dma_int_early_rx,	/* Early RX interrupt enable  */
	dma_ie_bus_error = dma_int_bus_error,	/* Fatal bus error enable     */
	dma_ie_early_tx = dma_int_early_tx,	/* Early TX interrupt enable  */
	dma_ie_rx_wdog_to = dma_int_rx_wdog_to,	/* RX Watchdog Timeout enable */
	dma_ie_rx_stopped = dma_int_rx_stopped,	/* RX process stopped enable  */
	dma_ie_rx_no_buffer = dma_int_rx_no_buffer,	/* Receive buffer
							   unavailable enable */
	dma_ie_rx_completed = dma_int_rx_completed,	/* Completion of frame
							   reception enable   */
	dma_ie_tx_underflow = dma_int_tx_underflow,	/* TX underflow enable*/
	dma_ie_rx_overflow = dma_int_rcv_overflow,	/* RX Buffer overflow
							   interrupt	      */
	dma_ie_tx_jabber_to = dma_int_tx_jabber_to,	/* TX Jabber Timeout
							   enable	      */
	dma_ie_tx_no_buffer = dma_int_tx_no_buffer,	/* TX buffer unavailable
							   enable	      */
	dma_ie_tx_stopped = dma_int_tx_stopped,		/* TX process stopped
							   enable	      */
	dma_ie_tx_completed = dma_int_tx_completed,	/* TX completed enable*/
};

/* dma_axi_bus_mod	= 0x,0028 */
enum dma_axi_bus_mode_reg {
	dma_en_lpi = 0x80000000,
	dma_lpi_xit_frm = 0x40000000,
	dma_wr_osr_num_reqs16 = 0x00F00000,
	dma_wr_osr_num_reqs8 = 0x00700000,
	dma_wr_osr_num_reqs4 = 0x00300000,
	dma_wr_osr_num_reqs2 = 0x00100000,
	dma_wr_osr_num_reqs1 = 0x00000000,
	dma_rd_osr_num_reqs16 = 0x000F0000,
	dma_rd_osr_num_reqs8 = 0x00070000,
	dma_rd_osr_num_reqs4 = 0x00030000,
	dma_rd_osr_num_reqs2 = 0x00010000,
	dma_rd_osr_num_reqs1 = 0x00000000,
	dma_onekbbe = 0x00002000,
	dma_axi_aal = 0x00001000,
	dma_axi_blen256 = 0x00000080,
	dma_axi_blen128 = 0x00000040,
	dma_axi_blen64 = 0x00000020,
	dma_axi_blen32 = 0x00000010,
	dma_axi_blen16 = 0x00000008,
	dma_axi_blen8 = 0x00000004,
	dma_axi_blen4 = 0x00000002,
	dma_undefined = 0x00000001,
};

/**********************************************************
 * DMA Engine descriptors
 **********************************************************/
/*
******Enhanced Descritpor structure to support 8K buffer per buffer *******

dma_rx_base_addr = 0x000C,	CSR3 - Receive Descriptor list base address
dma_rx_base_addr is the pointer to the first Rx Descriptors.
The Descriptor format in Little endian with a 32 bit Data bus is as shown below.

Similarly
dma_tx_base_addr     = 0x0010,  CSR4 - Transmit Descriptor list base address
dma_tx_base_addr is the pointer to the first Rx Descriptors.
The Descriptor format in Little endian with a 32 bit Data bus is as shown below.
	------------------------------------------------------------------------
 RDES0	|OWN (31)| Status						       |
	------------------------------------------------------------------------
 RDES1	| Ctrl | Res | Byte Count Buffer 2 | Ctrl | Res | Byte Count Buffer 1  |
	------------------------------------------------------------------------
 RDES2	|  Buffer 1 Address                                                    |
	------------------------------------------------------------------------
 RDES3	|  Buffer 2 Address / Next Descriptor Address                          |
	------------------------------------------------------------------------

	------------------------------------------------------------------------
 TDES0	|OWN (31)| Ctrl | Res | Ctrl | Res | Status                            |
	------------------------------------------------------------------------
 TDES1	| Res | Byte Count Buffer 2 | Res |         Byte Count Buffer 1        |
	------------------------------------------------------------------------
 TDES2	|  Buffer 1 Address                                                    |
	------------------------------------------------------------------------
 TDES3	|  Buffer 2 Address / Next Descriptor Address                          |
	------------------------------------------------------------------------

*/

/* status word of DMA descriptor */
enum dma_descriptor_status {
	desc_own_by_dma = 0x80000000,		/* (OWN)Descriptor is
						   owned by DMA engine        */
	desc_da_filter_fail = 0x40000000,	/* (AFM)Rx - DA Filter
						   Fail for the rx frame      */
	desc_frame_length_mask = 0x3FFF0000,	/* (FL)Receive descriptor
						   frame length               */
	desc_frame_length_shift = 16,
	desc_error = 0x00008000,		/* (ES)Error summary bit
						   - OR of the  following bits:
						   DE || OE || IPC || LC  || RWT
						   || RE || CE                */
	desc_rx_truncated = 0x00004000,		/* (DE)Rx - no more descriptors
						   for receive frame          */
	desc_sa_filter_fail = 0x00002000,	/* (SAF)Rx - SA Filter Fail for
						   the received frame         */
	desc_rx_length_error = 0x00001000,	/* (LE)Rx - frm size not
						   matching with len field    */
	desc_rx_damaged = 0x00000800,		/* (OE)Rx - frm was damaged due
						   to buffer overflow         */
	desc_rx_vlan_tag = 0x00000400,		/* (VLAN)Rx - received frame
						   is a VLAN frame            */
	desc_rx_first = 0x00000200,		/* (FS)Rx - first
						   descriptor of the frame    */
	desc_rx_last = 0x00000100,		/* (LS)Rx - last
						   descriptor of the frame    */
	desc_rx_long_frame = 0x00000080,	/* (Giant Frame)Rx - frame is
						   longer than 1518/1522      */
	desc_rx_collision = 0x00000040,		/* (LC)Rx - late collision
						   occurred during reception  */
	desc_rx_frame_ether = 0x00000020,	/* (FT)Rx - Frame type - Ether,
						   otherwise 802.3            */
	desc_rx_watchdog = 0x00000010,		/* (RWT)Rx - watchdog timer
						   expired during reception   */
	desc_rx_mii_error = 0x00000008,		/* (RE)Rx - error reported
						   by MII interface           */
	desc_rx_dribbling = 0x00000004,		/* (DE)Rx - frame contains non
						   int multiple of 8 bits     */
	desc_rx_crc = 0x00000002,		/* (CE)Rx - CRC error         */
	/*desc_rx_mac_match = 0x00000001,*/	/* (RX MAC Addr) Rx mac addr
						   reg(1 to 15)match   0     */
	desc_rx_ext_sts = 0x00000001,		/* Extended Status Available
						   in RDES4		      */
	desc_tx_int_enable = 0x40000000,	/* (IC)Tx - interrupt on
						   completion		      */
	desc_tx_last = 0x20000000,		/* (LS)Tx - Last segment of the
						   frame		      */
	desc_tx_first = 0x10000000,		/* (FS)Tx - First segment of the
						    frame		      */
	desc_tx_disable_crc = 0x08000000,	/* (DC)Tx - Add CRC disabled
						   (first segment only)       */
	desc_tx_disable_padd = 0x04000000,	/* (DP)disable padding,
						   added by - reyaz           */
	desc_tx_cis_mask = 0x00c00000,		/* Tx checksum offloading
						   control mask               */
	desc_tx_cis_bypass = 0x00000000,	/* Checksum bypass            */
	desc_tx_cis_ipv4_hdr_cs = 0x00400000,	/* IPv4 header checksum       */
	desc_tx_cis_tcp_only_cs = 0x00800000,	/* TCP/UDP/ICMP checksum.
						   Pseudo header  checksum
						   is assumed to be present   */
	desc_tx_cis_tcp_pseudo_cs = 0x00c00000,	/* TCP/UDP/ICMP checksum fully
						   in hardware  including
						   pseudo header              */
	tx_desc_end_of_ring = 0x00200000,	/* (TER)End of descriptor ring*/
	tx_desc_chain = 0x00100000,		/* (TCH)Second buffer address
						   is chain address           */
	desc_rx_chk_bit0 = 0x00000001,		/* Rx Payload Checksum Error  */
	desc_rx_chk_bit7 = 0x00000080,		/* (IPC CS ERROR)Rx - Ipv4
						   header checksum error      */
	desc_rx_chk_bit5 = 0x00000020,		/* (FT)Rx - Frame type - Ether,
						   otherwise 802.3            */
	desc_rx_ts_avail = 0x00000080,		/* Time stamp available       */
	desc_rx_frame_type = 0x00000020,	/* (FT)Rx - Frame type - Ether,
						   otherwise 802.3            */
	desc_tx_ipv4_chk_error = 0x00010000,	/* (IHE) Tx Ip header error   */
	desc_tx_timeout = 0x00004000,		/* (JT)Tx - Transmit
						   jabber timeout             */
	desc_tx_frame_flushed = 0x00002000,	/* (FF)Tx - DMA/MTL flushed
						   the frame  due to SW flush */
	desc_tx_pay_chk_error = 0x00001000,	/* (PCE) Tx Payload checksum
						   Error		      */
	desc_tx_lost_carrier = 0x00000800,	/* (LC)Tx - carrier lost
						   during tramsmission        */
	desc_tx_no_carrier = 0x00000400,	/* (NC)Tx - no carrier signal
						   from the tranceiver        */
	desc_tx_late_collision = 0x00000200,	/* (LC)Tx - transmission aborted
						   due to collision           */
	desc_tx_exc_collisions = 0x00000100,	/* (EC)Tx - transmission aborted
						   after 16 collisions        */
	desc_tx_vlan_frame = 0x00000080,	/* (VF)Tx - VLAN-type frame   */
	desc_tx_coll_mask = 0x00000078,		/* (CC)Tx - Collision count   */
	desc_tx_coll_shift = 3,
	desc_tx_exc_deferral = 0x00000004,	/* (ED)Tx - excessive deferral*/
	desc_tx_underflow = 0x00000002,		/* (UF)Tx - late data arrival
						   from the memory            */
	desc_tx_deferred = 0x00000001,		/* (DB)Tx - frame
						   transmision deferred       */

	/*
	 * This explains the RDES1/TDES1 bits layout
	 *             ------------------------------------------------------
	 * RDES1/TDES1 | Control Bits | Byte Count Buf 2 | Byte Count Buf 1 |
	 *             ------------------------------------------------------
	 */

	/*dma_descriptor_length *//* length word of DMA descriptor */
	rx_dis_int_compl = 0x80000000,	/* (Disable Rx int on completion)     */
	rx_desc_end_of_ring = 0x00008000,	/* (TER)End of descriptor ring*/
	rx_desc_chain = 0x00004000,	/* (TCH)Second buffer address
					   is chain address                   */
	desc_size2_mask = 0x1FFF0000,	/* (TBS2) Buffer 2 size               */
	desc_size2_shift = 16,
	desc_size1_mask = 0x00001FFF,	/* (TBS1) Buffer 1 size               */
	desc_size1_shift = 0,

	/*
	 * This explains the RDES4 Extended Status bits layout
	 *              --------------------------------------------------------
	 *   RDES4      |                 Extended Status                      |
	 *              --------------------------------------------------------
	 */
	desc_rx_ptp_avail = 0x00004000,		/* PTP snapshot available     */
	desc_rx_ptp_ver = 0x00002000,		/* When set indicates IEEE1584
						   Version 2 (else Ver1)      */
	desc_rx_ptp_frame_type = 0x00001000,	/* PTP frame type Indicates PTP
						   sent over ethernet         */
	desc_rx_ptp_message_type = 0x00000F00,	/* Message Type               */
	desc_rx_ptp_no = 0x00000000,		/* 0000 => No PTP message rcvd*/
	desc_rx_ptp_sync = 0x00000100,		/* 0001 => Sync (all clock
						   types) received            */
	desc_rx_ptp_follow_up = 0x00000200,	/* 0010 => Follow_Up (all clock
						   types) received            */
	desc_rx_ptp_delay_req = 0x00000300,	/* 0011 => Delay_Req (all clock
						   types) received            */
	desc_rx_ptp_delay_resp = 0x00000400,	/* 0100 => Delay_Resp (all clock
						   types) received            */
	desc_rx_ptp_pdelay_req = 0x00000500,	/* 0101 => Pdelay_Req (in P
						   to P tras clk)  or Announce
						   in Ord and Bound clk       */
	desc_rx_ptp_pdelay_resp = 0x00000600,	/* 0110 => Pdealy_Resp(in P to
						   P trans clk) or Management in
						   Ord and Bound clk          */
	desc_rx_ptp_pdelay_resp_fP = 0x00000700,/* 0111 => Pdealy_Resp_Follow_Up
						   (in P to P trans clk) or
						   Signaling in Ord and Bound
						   clk			      */
	desc_rx_ptp_ipv6 = 0x00000080,		/* Received Packet is in IPV6 */
	desc_rx_ptp_ipv4 = 0x00000040,		/* Received Packet is in IPV4 */
	desc_rx_chk_sum_bypass = 0x00000020,	/* When set indicates checksum
						   offload engine is bypassed */
	desc_rx_ip_payload_error = 0x00000010,	/* When set indicates 16bit IP
						   payload CS is in error     */
	desc_rx_ip_header_error = 0x00000008,	/* When set indicates 16bit IPV4
						   hdr CS is err or IP datagram
						   version is not consistent
						   with Ethernet type value   */
	desc_rx_ip_payload_type = 0x00000007,	/* Indicate the type of payload
						   encapsulated in IPdatagram
						   processed by COE (Rx)      */
	desc_rx_ip_payload_unknown = 0x00000000,/* Unknown or didnot process
						   IP payload                 */
	desc_rx_ip_payload_udp = 0x00000001,	/* UDP                        */
	desc_rx_ip_payload_tcp = 0x00000002,	/* TCP                        */
	desc_rx_ip_payload_icmp = 0x00000003,	/* ICMP                       */
};

/**********************************************************
 * Initial register values
 **********************************************************/
enum initial_registers {
	/* Full-duplex mode with perfect filter on */
	gmac_config_init_fdx1000 = gmac_watchdog_enable | gmac_jabber_enable
	    | gmac_frame_burst_enable | gmac_jumbo_frame_disable
	    | gmac_select_gmii | gmac_enable_rx_own
	    | gmac_loopback_off | gmac_full_duplex | gmac_retry_enable
	    | gmac_pad_crc_strip_disable | gmac_backoff_limit0
	    | gmac_deferral_check_disable | gmac_tx_enable | gmac_rx_enable,

	/* Full-duplex mode with perfect filter on */
	gmac_config_init_fdx110 = gmac_watchdog_enable | gmac_jabber_enable
	    | gmac_frame_burst_enable
	    | gmac_jumbo_frame_disable | gmac_select_mii | gmac_enable_rx_own
	    | gmac_loopback_off | gmac_full_duplex | gmac_retry_enable
	    | gmac_pad_crc_strip_disable | gmac_backoff_limit0
	    | gmac_deferral_check_disable | gmac_tx_enable | gmac_rx_enable,

	/* Full-duplex mode */
	/*      CHANGED: Pass control config, dest addr filter normal,
	   added source address filter, multicast & unicast
	 */

	/* Hash filter. */
	/* = gmac_filter_off | gmac_pass_control_off | gmac_broadcast_enable */
	gmac_frame_filter_init_fdx =
	    gmac_filter_on | gmac_pass_control0 | gmac_broadcast_enable |
	    gmac_src_addr_filter_disable | gmac_multicast_filter_on |
	    gmac_dest_addr_filter_nor | gmac_mcast_hash_filter_off |
	    gmac_promiscuous_mode_off | gmac_ucast_hash_filter_off,

	/* Full-duplex mode */
	gmac_flow_control_init_fdx =
	    gmac_unicast_pause_frame_off | gmac_rx_flow_control_enable |
	    gmac_tx_flow_control_enable,

	/* Full-duplex mode */
	gmac_gmii_addr_init_fdx = gmii_csr_clk2,

	/* Half-duplex mode with perfect filter on */
	/* CHANGED: Removed Endian configuration, added single bit
	 *config for PAD/CRC strip,
	 */

	gmac_config_init_hdx1000 = gmac_watchdog_enable | gmac_jabber_enable
	    | gmac_frame_burst_enable | gmac_jumbo_frame_disable
	    | gmac_select_gmii | gmac_disable_rx_own
	    | gmac_loopback_off | gmac_half_duplex | gmac_retry_enable
	    | gmac_pad_crc_strip_disable | gmac_backoff_limit0
	    | gmac_deferral_check_disable | gmac_tx_enable | gmac_rx_enable,

	/* Half-duplex mode with perfect filter on */
	gmac_config_init_hdx110 = gmac_watchdog_enable | gmac_jabber_enable
	    | gmac_frame_burst_enable | gmac_jumbo_frame_disable
	    | gmac_select_mii | gmac_disable_rx_own | gmac_loopback_off
	    | gmac_half_duplex | gmac_retry_enable
	    | gmac_pad_crc_strip_disable | gmac_backoff_limit0
	    | gmac_deferral_check_disable | gmac_tx_enable | gmac_rx_enable,

	/* Half-duplex mode */
	gmac_frame_filter_init_hdx = gmac_filter_on | gmac_pass_control0
	    | gmac_broadcast_enable | gmac_src_addr_filter_disable
	    | gmac_multicast_filter_on | gmac_dest_addr_filter_nor
	    | gmac_mcast_hash_filter_off | gmac_ucast_hash_filter_off
	    | gmac_promiscuous_mode_off,

	/* Half-duplex mode */
	gmac_flow_control_init_hdx = gmac_unicast_pause_frame_off
	    | gmac_rx_flow_control_disable | gmac_tx_flow_control_disable,

	/* Half-duplex mode */
	gmac_gmii_addr_init_hdx = gmii_csr_clk2,

/*********************************************
* DMA configurations
**********************************************/

	dma_bus_mode_init = dma_fixed_burst_enable | dma_burst_length8
	    | dma_descriptor_skip2 | dma_reset_off,

	dma_bus_mode_val = dma_burst_length32
	    | dma_burst_lengthx8 | dma_descriptor_skip0
	    | dma_descriptor8_words | dma_arbit_pr | dma_address_aligned_beats,

	/* 1000 Mb/s mode */
	dma_control_init1000 = dma_store_and_forward,

	/* 100 Mb/s mode */
	dma_control_init100 = dma_store_and_forward,

	/* 10 Mb/s mode */
	dma_control_init10 = dma_store_and_forward,

	dma_omr = dma_store_and_forward | dma_rx_store_and_forward
	    | dma_rx_thresh_ctrl128 | dma_tx_second_frame,

	/* Interrupt groups */
	dma_int_error_mask = dma_int_bus_error,		/* Error              */
	dma_int_rx_abn_mask = dma_int_rx_no_buffer,	/* RX abnormal intr   */
	dma_int_rx_norm_mask = dma_int_rx_completed,	/* RXnormal intr      */
	dma_int_rx_stopped_mask = dma_int_rx_stopped,	/* RXstopped          */
	dma_int_tx_abn_mask = dma_int_tx_underflow,	/* TX abnormal intr   */
	dma_int_tx_norm_mask = dma_int_tx_completed,	/* TX normal intr     */
	dma_int_tx_stopped_mask = dma_int_tx_stopped,	/* TX stopped         */

	dma_int_enable = dma_ie_normal | dma_ie_abnormal | dma_int_error_mask
	    | dma_int_rx_abn_mask | dma_int_rx_norm_mask
	    | dma_int_rx_stopped_mask | dma_int_tx_abn_mask
	    | dma_int_tx_norm_mask | dma_int_tx_stopped_mask,
	dma_int_disable = 0,
	dma_axi_bus_mode_val = dma_axi_blen16 | dma_rd_osr_num_reqs8 |
							dma_wr_osr_num_reqs8,
};
/**********************************************************
 * Mac Management Counters (MMC)
 **********************************************************/
enum mmc_enable {
	gmac_mmc_cntrl = 0x0100,		/* mmc control for operating
					   mode of MMC                  */
	gmac_mmc_intr_rx = 0x0104,		/* maintains interrupts
					   generated by rx counters     */
	gmac_mmc_intr_tx = 0x0108,		/* maintains interrupts
					   generated by tx counters     */
	gmac_mmc_intr_mask_rx = 0x010C,	/* mask for interrupts
					   generated from rx counters   */
	gmac_mmc_intr_mask_tx = 0x0110,	/* mask for interrupts
					   generated from tx counters   */
};

enum mmc_ip_related {
	gmac_mmc_rx_ipc_intr_mask = 0x0200,
/*Maintains the mask for interrupt generated from rx IPC statistic counters   */
};

/*******************Ip checksum offloading APIs********************************/
void nss_gmac_enable_rx_chksum_offload(struct nss_gmac_dev *gmacdev);
void nss_gmac_disable_rx_chksum_offload(struct nss_gmac_dev *gmacdev);
void nss_gmac_rx_tcpip_chksum_drop_enable(struct nss_gmac_dev *gmacdev);
void nss_gmac_rx_tcpip_chksum_drop_disable(struct nss_gmac_dev *gmacdev);

/**
 * The check summ offload engine is enabled to do complete checksum computation.
 * Hardware computes the tcp ip checksum including the pseudo header checksum.
 * Here the tcp payload checksum field should be set to 0000.
 * Ipv4 header checksum is also inserted.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] Pointer to tx descriptor for which pointer to nss_gmac_dev.
 * @return returns void.
 */
static inline void nss_gmac_tx_checksum_offload_tcp_pseudo(struct nss_gmac_dev *
						   gmacdev,
						   struct dma_desc *desc)
{
	desc->status =
	    ((desc->status & (~desc_tx_cis_mask)) | desc_tx_cis_tcp_pseudo_cs);
}

/**********************************************************
 * Common functions
 **********************************************************/
/**
 * @brief Low level function to read register contents from Hardware.
 * @param[in] pointer containing address of register base
 * @param[in] register offset
 * @return contents of register
 */
static inline uint32_t nss_gmac_read_reg(uint32_t *regbase,
					     uint32_t regoffset)
{
	uint32_t addr = 0;
	uint32_t data;

	spin_lock(&ctx.reg_lock);
	addr = (uint32_t)regbase + regoffset;
	data = readl_relaxed((unsigned char *)addr);
	spin_unlock(&ctx.reg_lock);

	return data;
}


/**
 * @brief Low level function to write to a register in Hardware.
 * @param[in] pointer containing address of register base
 * @param[in] register offset
 * @param[in] data to be written
 * @return void
 */
static inline void nss_gmac_write_reg(uint32_t *regbase,
					  uint32_t regoffset,
					  uint32_t regdata)
{
	uint32_t addr = 0;

	spin_lock(&ctx.reg_lock);
	addr = (uint32_t)regbase + regoffset;
	writel_relaxed(regdata, (unsigned char *)addr);
	spin_unlock(&ctx.reg_lock);
}


/**
 * @brief Low level function to set bits of a register in Hardware.
 * @param[in] pointer containing address of register base
 * @param[in] register offset
 * @param[in] bit mask of bits to be set
 * @return void
 */
static inline void nss_gmac_set_reg_bits(uint32_t *regbase,
					     uint32_t regoffset,
					     uint32_t bitpos)
{
	uint32_t data = 0;

	data = bitpos | nss_gmac_read_reg(regbase, regoffset);
	nss_gmac_write_reg(regbase, regoffset, data);
}


/**
 * @brief Low level function to clear bits of a register in Hardware.
 * @param[in] pointer containing address of register base
 * @param[in] register offset
 * @param[in] bit mask of bits to be cleared
 * @return void
 */
static inline void nss_gmac_clear_reg_bits(uint32_t *regbase,
					       uint32_t regoffset,
					       uint32_t bitpos)
{
	uint32_t data = 0;

	data = ~bitpos & nss_gmac_read_reg(regbase, regoffset);
	nss_gmac_write_reg(regbase, regoffset, data);
}


/**
 * @brief Low level function to Check the setting of the bits.
 * @param[in] pointer containing address of register base
 * @param[in] register offset
 * @param[in] bit mask of bits to be checked
 * @return True if bits corresponding to the given bitmask are set.
 */
static inline bool nss_gmac_check_reg_bits(uint32_t *regbase,
					       uint32_t regoffset,
					       uint32_t bitpos)
{
	uint32_t data;

	data = bitpos & nss_gmac_read_reg(regbase, regoffset);

	return data != 0;
}

uint16_t nss_gmac_mii_rd_reg(struct nss_gmac_dev *gmacdev, uint32_t phy,
			     uint32_t reg);
void nss_gmac_mii_wr_reg(struct nss_gmac_dev *gmacdev, uint32_t phy,
			 uint32_t reg, uint16_t data);
int32_t nss_gmac_read_version(struct nss_gmac_dev *gmacdev);
void nss_gmac_reset(struct nss_gmac_dev *gmacdev);
int32_t nss_gmac_dma_bus_mode_init(struct nss_gmac_dev *gmacdev,
						uint32_t init_value);
int32_t nss_gmac_dma_axi_bus_mode_init(struct nss_gmac_dev *gmacdev,
						uint32_t init_value);
int32_t nss_gmac_dma_control_init(struct nss_gmac_dev *gmacdev,
						uint32_t init_value);
void nss_gmac_wd_enable(struct nss_gmac_dev *gmacdev);
void nss_gmac_jab_enable(struct nss_gmac_dev *gmacdev);
void nss_gmac_frame_burst_enable(struct nss_gmac_dev *gmacdev);
void nss_gmac_jumbo_frame_enable(struct nss_gmac_dev *gmacdev);
void nss_gmac_jumbo_frame_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_twokpe_frame_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_twokpe_frame_enable(struct nss_gmac_dev *gmacdev);
void nss_gmac_select_gmii(struct nss_gmac_dev *gmacdev);
void nss_gmac_select_mii(struct nss_gmac_dev *gmacdev);
void nss_gmac_rx_own_enable(struct nss_gmac_dev *gmacdev);
void nss_gmac_rx_own_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_loopback_off(struct nss_gmac_dev *gmacdev);
void nss_gmac_set_full_duplex(struct nss_gmac_dev *gmacdev);
void nss_gmac_set_half_duplex(struct nss_gmac_dev *gmacdev);
void nss_gmac_retry_enable(struct nss_gmac_dev *gmacdev);
void nss_gmac_retry_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_pad_crc_strip_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_back_off_limit(struct nss_gmac_dev *gmacdev, uint32_t value);
void nss_gmac_deferral_check_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_rx_enable(struct nss_gmac_dev *gmacdev);
void nss_gmac_rx_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_tx_enable(struct nss_gmac_dev *gmacdev);
void nss_gmac_tx_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_frame_filter_enable(struct nss_gmac_dev *gmacdev);
void nss_gmac_src_addr_filter_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_dst_addr_filter_normal(struct nss_gmac_dev *gmacdev);
void nss_gmac_set_pass_control(struct nss_gmac_dev *gmacdev,
				uint32_t passcontrol);
void nss_gmac_broadcast_enable(struct nss_gmac_dev *gmacdev);
void nss_gmac_multicast_enable(struct nss_gmac_dev *gmacdev);
void nss_gmac_multicast_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_multicast_hash_filter_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_promisc_enable(struct nss_gmac_dev *gmacdev);
void nss_gmac_promisc_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_unicast_hash_filter_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_unicast_pause_frame_detect_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_rx_flow_control_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_tx_flow_control_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_tx_pause_enable(struct nss_gmac_dev *gmacdev);
void nss_gmac_tx_pause_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_rx_pause_enable(struct nss_gmac_dev *gmacdev);
void nss_gmac_rx_pause_disable(struct nss_gmac_dev *gmacdev);
void nss_gmac_flush_tx_fifo(struct nss_gmac_dev *gmacdev);
void nss_gmac_mac_init(struct nss_gmac_dev *gmacdev);
int32_t nss_gmac_check_phy_init(struct nss_gmac_dev *gmacdev);
int32_t nss_gmac_ath_phy_mmd_wr(struct phy_device *phydev,
		uint32_t mmd_dev_addr, uint32_t reg, uint16_t val);
int32_t nss_gmac_ath_phy_mmd_rd(struct phy_device *phydev,
			uint32_t mmd_dev_addr, uint32_t reg);
int32_t nss_gmac_ath_phy_disable_smart_802az(struct phy_device *phydev);
int32_t nss_gmac_ath_phy_disable_802az(struct phy_device *phydev);
void nss_gmac_set_mac_addr(struct nss_gmac_dev *gmacdev,
			uint32_t mac_high, uint32_t mac_low, uint8_t *mac_addr);
void nss_gmac_get_mac_addr(struct nss_gmac_dev *gmacdev,
			uint32_t mac_high, uint32_t mac_low, uint8_t *mac_addr);
int32_t nss_gmac_attach(struct nss_gmac_dev *gmacdev, uint32_t reg_base,
			uint32_t reglen);
void nss_gmac_detach(struct nss_gmac_dev *gmacdev);
int32_t nss_gmac_check_link(struct nss_gmac_dev *gmacdev);
void nss_gmac_ipc_offload_init(struct nss_gmac_dev *gmacdev);
void nss_gmac_tx_rx_desc_init(struct nss_gmac_dev *gmacdev);
int32_t nss_gmac_init_mdiobus(struct nss_gmac_dev *gmacdev);
void nss_gmac_deinit_mdiobus(struct nss_gmac_dev *gmacdev);
void nss_gmac_reset_phy(struct nss_gmac_dev *gmacdev, uint32_t phyid);
int32_t nss_gmac_write_phy_reg(uint32_t *reg_base, uint32_t phy_base,
			       uint32_t reg_offset, uint16_t data,
			       uint32_t mdc_clk_div);
int32_t nss_gmac_read_phy_reg(uint32_t *reg_base, uint32_t phy_base,
			      uint32_t reg_offset, uint16_t *data,
			      uint32_t mdc_clk_div);

/*
 * nss_gmac_common_init()
 *	Init commom to all GMACs.
 */
int32_t nss_gmac_common_init(struct nss_gmac_global_ctx *ctx);

/*
 * nss_gmac_common_deinit()
 *	Global common deinit.
 */
void nss_gmac_common_deinit(struct nss_gmac_global_ctx *ctx);

/*
 * nss_gmac_dev_init()
 *	GMAC device initializaton.
 */
void nss_gmac_dev_init(struct nss_gmac_dev *gmacdev);

/*
 * nss_gmac_dev_set_speed()
 *	Set GMAC speed.
 */
int32_t nss_gmac_dev_set_speed(struct nss_gmac_dev *gmacdev);

/*
 * nss_gmac_spare_ctl()
 *	Spare Control reset. Required only for emulation.
 */
void nss_gmac_spare_ctl(struct nss_gmac_dev *gmacdev);

/**
 * Initialize the rx descriptors for ring or chain mode operation.
 *	- Status field is initialized to 0.
 *	- end_of_ring set for the last descriptor.
 *	- buffer1 and buffer2 set to 0 for ring mode of operation. (note)
 *	- data1 and data2 set to 0. (note)
 * @param[in] pointer to dma_desc structure.
 * @param[in] whether end of ring
 * @return void.
 * @note Initialization of the buffer1, buffer2, data1,data2 and status are not
 * done here. This only initializes whether one wants to use this descriptor
 * in chain mode or ring mode. For chain mode of operation the buffer2 and data2
 * are programmed before calling this function.
 */
static inline void nss_gmac_rx_desc_init_ring(struct dma_desc *desc,
					      bool last_ring_desc)
{
	desc->status = 0;
	desc->length = last_ring_desc ? rx_desc_end_of_ring : 0;
	desc->buffer1 = 0;
	desc->data1 = 0;
}

/**
 * Initialize the tx descriptors for ring or chain mode operation.
 *	- Status field is initialized to 0.
 *	- end_of_ring set for the last descriptor.
 *	- buffer1 and buffer2 set to 0 for ring mode of operation. (note)
 *	- data1 and data2 set to 0. (note)
 * @param[in] pointer to dma_desc structure.
 * @param[in] whether end of ring
 * @return void.
 * @note Initialization of the buffer1, buffer2, data1,data2 and status are not
 * done here. This only initializes whether one wants to use this descriptor
 * in chain mode or ring mode. For chain mode of operation the buffer2 and data2
 * are programmed before calling this function.
 */
static inline void nss_gmac_tx_desc_init_ring(struct dma_desc *desc,
					      bool last_ring_desc)
{
	desc->status = last_ring_desc ? tx_desc_end_of_ring : 0;
	desc->length = 0;
	desc->buffer1 = 0;
	desc->data1 = 0;
}

void nss_gmac_init_rx_desc_base(struct nss_gmac_dev *gmacdev);
void nss_gmac_init_tx_desc_base(struct nss_gmac_dev *gmacdev);
void nss_gmac_set_owner_dma(struct dma_desc *desc);
void nss_gmac_set_desc_sof(struct dma_desc *desc);
void nss_gmac_set_desc_eof(struct dma_desc *desc);
bool nss_gmac_is_sof_in_rx_desc(struct dma_desc *desc);
bool nss_gmac_is_eof_in_rx_desc(struct dma_desc *desc);
bool nss_gmac_is_da_filter_failed(struct dma_desc *desc);
bool nss_gmac_is_sa_filter_failed(struct dma_desc *desc);

/**
 * Checks whether the descriptor is owned by DMA.
 * If descriptor is owned by DMA then the OWN bit is set to 1.
 * This API is same for both ring and chain mode.
 * @param[in] pointer to dma_desc structure.
 * @return returns true if Dma owns descriptor and false if not.
 */
static inline bool nss_gmac_is_desc_owned_by_dma(struct dma_desc *desc)
{
	return (desc->status & desc_own_by_dma) == desc_own_by_dma;
}


/**
 * returns the byte length of received frame including CRC.
 * This returns the no of bytes received in the received ethernet frame
 * including CRC(FCS).
 * @param[in] pointer to dma_desc structure.
 * @return returns the length of received frame lengths in bytes.
 */
static inline uint32_t nss_gmac_get_rx_desc_frame_length(uint32_t status)
{
	return (status & desc_frame_length_mask) >> desc_frame_length_shift;
}


/**
 * Checks whether the descriptor is valid
 * if no errors such as CRC/Receive Error/Watchdog Timeout/Late collision/
 * Giant Frame/Overflow/Descriptor error the descritpor is said to be a valid
 * descriptor.
 * @param[in] pointer to dma_desc structure.
 * @return True if desc valid. false if error.
 */
static inline bool nss_gmac_is_desc_valid(uint32_t status)
{
	return (status & desc_error) == 0;
}


/**
 * Checks whether the descriptor is empty.
 * If the buffer1 and buffer2 lengths are zero in ring mode descriptor is empty.
 * In chain mode buffer2 length is 0 but buffer2 itself contains the next
 * descriptor address.
 * @param[in] pointer to dma_desc structure.
 * @return returns true if descriptor is empty, false if not empty.
 */
static inline bool nss_gmac_is_desc_empty(struct dma_desc *desc)
{
	/* if length of both buffer1 & buffer2 are zero then desc is empty */
	return (desc->length & desc_size1_mask) == 0;
}


/**
 * Checks whether the rx descriptor is valid.
 * if rx descripor is not in error and complete frame is available in the same
 * descriptor
 * @param[in] status
 * @return returns true if no error and first and last desc bits are set,
 * otherwise it returns false.
 */
static inline bool nss_gmac_is_rx_desc_valid(uint32_t status)
{
	return (status & (desc_error | desc_rx_first | desc_rx_last)) ==
		(desc_rx_first | desc_rx_last);
}

bool nss_gmac_is_tx_aborted(uint32_t status);
bool nss_gmac_is_tx_carrier_error(uint32_t status);
bool nss_gmac_is_tx_underflow_error(uint32_t status);
bool nss_gmac_is_tx_lc_error(uint32_t status);


/**
 * Gives the transmission collision count.
 * returns the transmission collision count indicating number of
 * collisions occurred before the frame was transmitted.
 * Make sure to check excessive collision didnot happen to ensure the count is
 * valid.
 * @param[in] status
 * @return returns the count value of collision.
 */
static inline uint32_t nss_gmac_get_tx_collision_count(uint32_t status)
{
	return (status & desc_tx_coll_mask) >> desc_tx_coll_shift;
}

static inline uint32_t nss_gmac_is_exc_tx_collisions(uint32_t status)
{
	return (status & desc_tx_exc_collisions) == desc_tx_exc_collisions;
}

bool nss_gmac_is_rx_frame_damaged(uint32_t status);
bool nss_gmac_is_rx_frame_collision(uint32_t status);
bool nss_gmac_is_rx_crc(uint32_t status);
bool nss_gmac_is_frame_dribbling_errors(uint32_t status);
bool nss_gmac_is_rx_frame_length_errors(uint32_t status);


/**
 * Checks whether this rx descriptor is last rx descriptor.
 * This returns true if it is last descriptor either in ring mode or chain mode.
 * @param[in] pointer to devic structure.
 * @param[in] pointer to dma_desc structure.
 * @return returns true if it is last descriptor, false if not.
 */
static inline bool nss_gmac_is_last_rx_desc(struct nss_gmac_dev *gmacdev,
					    struct dma_desc *desc)
{
	return unlikely((desc->length & rx_desc_end_of_ring) != 0);
}


/**
 * Checks whether this tx descriptor is last tx descriptor.
 * This returns true if it is last descriptor either in ring mode or chain mode.
 * @param[in] pointer to devic structure.
 * @param[in] pointer to dma_desc structure.
 * @return returns true if it is last descriptor, false if not.
 */
static inline bool nss_gmac_is_last_tx_desc(struct nss_gmac_dev *gmacdev,
					    struct dma_desc *desc)
{
	return unlikely((desc->status & tx_desc_end_of_ring) != 0);
}


/**
 * Checks whether this rx descriptor is in chain mode.
 * This returns true if it is this descriptor is in chain mode.
 * @param[in] pointer to dma_desc structure.
 * @return returns true if chain mode is set, false if not.
 */
static inline bool nss_gmac_is_rx_desc_chained(struct dma_desc *desc)
{
	/*
	 * Use ring mode only.
	 * This is also the only way to support jumbo in the future.
	 */
	return 0;
}


/**
 * Checks whether this tx descriptor is in chain mode.
 * This returns true if it is this descriptor is in chain mode.
 * @param[in] pointer to dma_desc structure.
 * @return returns true if chain mode is set, false if not.
 */
static inline bool nss_gmac_is_tx_desc_chained(struct dma_desc *desc)
{
	/*
	 * Use ring mode only.
	 * This is also the only way to support jumbo in the future.
	 */
	return 0;
}

void nss_gmac_get_desc_data(struct dma_desc *desc, uint32_t *Status,
			    uint32_t *buffer1, uint32_t *length1,
			    uint32_t *data1);

/**
 * Get the index and address of Tx desc.
 * This api is same for both ring mode and chain mode.
 * This function tracks the tx descriptor the DMA just closed after the
 * transmission of data from this descriptor is over. This returns the
 * descriptor fields to the caller.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns present tx descriptor index on success. Negative value if
 * error.
 */
static inline struct dma_desc *nss_gmac_get_tx_qptr(struct nss_gmac_dev *gmacdev)
{
	struct dma_desc *txdesc = gmacdev->tx_busy_desc;

	if (unlikely(gmacdev->busy_tx_desc == 0))
		return NULL;

	if (nss_gmac_is_desc_owned_by_dma(txdesc))
		return NULL;

	BUG_ON(nss_gmac_is_desc_empty(txdesc));

	return txdesc;
}


/**
 * Reset the descriptor after Tx is over.
 * Update descriptor pointers.
 * @param[in] pointer to nss_gmac_dev.
 * @return Returns void
 */
static inline void nss_gmac_reset_tx_qptr(struct nss_gmac_dev *gmacdev)
{
	uint32_t txover = gmacdev->tx_busy;
	struct dma_desc *txdesc = gmacdev->tx_busy_desc;

	BUG_ON(txdesc != (gmacdev->tx_desc + txover));
	gmacdev->tx_busy = (txover + 1) & (gmacdev->tx_desc_count - 1);
	gmacdev->tx_busy_desc = gmacdev->tx_desc + gmacdev->tx_busy;

	txdesc->status &= tx_desc_end_of_ring;
	txdesc->length = 0;
	txdesc->buffer1 = 0;
	txdesc->data1 = 0;
	txdesc->reserved1 = 0;

	/*
	 * Busy tx descriptor is reduced by one as
	 * it will be handed over to Processor now.
	 */
	(gmacdev->busy_tx_desc)--;
}


/**
 * Populate the tx desc structure with the buffer address.
 * Once the driver has a packet ready to be transmitted, this function is called
 * with the valid dma-able buffer addresses and their lengths. This function
 * populates the descriptor and make the DMA the owner for the descriptor. This
 * function also controls whetther Checksum offloading to be done in hardware or
 * not.
 * This api is same for both ring mode and chain mode.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] Dma-able buffer1 pointer.
 * @param[in] length of buffer1 (Max is 2048).
 * @param[in] virtual pointer for buffer1.
 * @param[in] uint32_t indicating whether the checksum offloading in HW/SW.
 * @param[in] uint32_t indicating TX control flag - the first, last segment and
 * interrupt state.
 * @param[in] uint32_t indicating descriptor DMA flag state.
 * @return returns present tx descriptor pointer.
 */
static inline struct dma_desc *nss_gmac_set_tx_qptr(struct nss_gmac_dev *gmacdev,
					   uint32_t Buffer1, uint32_t Length1,
					   uint32_t Data1,
					   uint32_t offload_needed,
					   uint32_t tx_cntl, uint32_t set_dma)
{
	uint32_t txnext = gmacdev->tx_next;
	struct dma_desc *txdesc = gmacdev->tx_next_desc;

	BUG_ON(gmacdev->busy_tx_desc > gmacdev->tx_desc_count);
	BUG_ON(txdesc != (gmacdev->tx_desc + txnext));
	BUG_ON(!nss_gmac_is_desc_empty(txdesc));
	BUG_ON(nss_gmac_is_desc_owned_by_dma(txdesc));

	if (Length1 > NSS_GMAC_MAX_DESC_BUFF) {
		txdesc->length |=
		    (NSS_GMAC_MAX_DESC_BUFF << desc_size1_shift) & desc_size1_mask;
		txdesc->length |=
		    ((Length1 -
		      NSS_GMAC_MAX_DESC_BUFF) << desc_size2_shift) & desc_size2_mask;
	} else {
		txdesc->length |= ((Length1 << desc_size1_shift) & desc_size1_mask);
	}

	txdesc->status |= tx_cntl;

	txdesc->buffer1 = Buffer1;
	txdesc->reserved1 = Data1;

	/* Program second buffer address if using two buffers. */
	if (Length1 > NSS_GMAC_MAX_DESC_BUFF)
		txdesc->data1 = Buffer1 + NSS_GMAC_MAX_DESC_BUFF;
	else
		txdesc->data1 = 0;

	if (likely(offload_needed))
		nss_gmac_tx_checksum_offload_tcp_pseudo(gmacdev, txdesc);

	/*
	 * Ensure all write completed before setting own by dma bit so when gmac
	 * HW takeover this descriptor, all the fields are filled correctly
	 */
	wmb();
	txdesc->status |= set_dma;

	gmacdev->tx_next = (txnext + 1) & (gmacdev->tx_desc_count - 1);
	gmacdev->tx_next_desc = gmacdev->tx_desc + gmacdev->tx_next;

	return txdesc;
}


/**
 * Prepares the descriptor to receive packets.
 * The descriptor is allocated with the valid buffer addresses (sk_buff address)
 * and the length fields and handed over to DMA by setting the ownership. After
 * successful return from this function the descriptor is added to the receive
 * descriptor pool/queue.
 * This api is same for both ring mode and chain mode.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] Dma-able buffer1 pointer.
 * @param[in] length of buffer1 (Max is 2048).
 * @param[in] pointer to buffer context.
 * @return returns present rx descriptor index on success. Negative value if
 * error.
 */
static inline int32_t nss_gmac_set_rx_qptr(struct nss_gmac_dev *gmacdev,
					   uint32_t Buffer1, uint32_t Length1,
					   uint32_t Data1)
{
	uint32_t rxnext = gmacdev->rx_next;
	struct dma_desc *rxdesc = gmacdev->rx_next_desc;

	BUG_ON(gmacdev->busy_rx_desc >= gmacdev->rx_desc_count);
	BUG_ON(rxdesc != (gmacdev->rx_desc + rxnext));
	BUG_ON(!nss_gmac_is_desc_empty(rxdesc));
	BUG_ON(nss_gmac_is_desc_owned_by_dma(rxdesc));

	if (Length1 > NSS_GMAC_MAX_DESC_BUFF) {
		rxdesc->length |=
		    (NSS_GMAC_MAX_DESC_BUFF << desc_size1_shift) & desc_size1_mask;
		rxdesc->length |=
		    ((Length1 -
		      NSS_GMAC_MAX_DESC_BUFF) << desc_size2_shift) & desc_size2_mask;
	} else {
		rxdesc->length |= ((Length1 << desc_size1_shift) & desc_size1_mask);
	}

	rxdesc->buffer1 = Buffer1;
	rxdesc->reserved1 = Data1;

	/* Program second buffer address if using two buffers. */
	if (Length1 > NSS_GMAC_MAX_DESC_BUFF)
		rxdesc->data1 = Buffer1 + NSS_GMAC_MAX_DESC_BUFF;
	else
		rxdesc->data1 = 0;

	rxdesc->extstatus = 0;
	rxdesc->timestamplow = 0;
	rxdesc->timestamphigh = 0;

	/*
	 * Ensure all write completed before setting own by dma bit so when gmac
	 * HW takeover this descriptor, all the fields are filled correctly
	 */
	wmb();
	rxdesc->status = desc_own_by_dma;

	gmacdev->rx_next = (rxnext + 1) & (gmacdev->rx_desc_count - 1);
	gmacdev->rx_next_desc = gmacdev->rx_desc + gmacdev->rx_next;

	/*
	 * 1 descriptor will be given to HW. So busy count incremented by 1.
	 */
	(gmacdev->busy_rx_desc)++;

	return rxnext;
}


/**
 * Get back the descriptor from DMA after data has been received.
 * When the DMA indicates that the data is received (interrupt is generated),
 * this function should be called to get the descriptor and hence the data
 * buffers received. With successful return from this function caller gets the
 * descriptor fields for processing. check the parameters to understand the
 * fields returned.`
 * @param[in] pointer to nss_gmac_dev.
 * @return returns pointer to dma_desc on success. Negative value if error.
 */
static inline struct dma_desc *nss_gmac_get_rx_qptr(struct nss_gmac_dev *gmacdev)
{
	struct dma_desc *rxdesc = gmacdev->rx_busy_desc;

	if (unlikely(gmacdev->busy_rx_desc == 0))
		return NULL;

	if (nss_gmac_is_desc_owned_by_dma(rxdesc))
		return NULL;

	BUG_ON(nss_gmac_is_desc_empty(rxdesc));

	return rxdesc;
}


/**
 * Reset the descriptor after Rx is over.
 * Update descriptor pointers.
 * @param[in] pointer to nss_gmac_dev.
 * @return Returns void
 */
static inline void nss_gmac_reset_rx_qptr(struct nss_gmac_dev *gmacdev)
{

	/* Index of descriptor the DMA just completed.
	 * May be useful when data is spread over multiple buffers/descriptors
	 */
	uint32_t rxnext = gmacdev->rx_busy;
	struct dma_desc *rxdesc = gmacdev->rx_busy_desc;

	BUG_ON(rxdesc != (gmacdev->rx_desc + rxnext));
	gmacdev->rx_busy = (rxnext + 1) & (gmacdev->rx_desc_count - 1);
	gmacdev->rx_busy_desc = gmacdev->rx_desc + gmacdev->rx_busy;

	rxdesc->status = 0;
	rxdesc->length &= rx_desc_end_of_ring;
	rxdesc->buffer1 = 0;
	rxdesc->data1 = 0;
	rxdesc->reserved1 = 0;

	/* This returns one descriptor to processor.
	 * So busy count will be decremented by one
	 */
	(gmacdev->busy_rx_desc)--;
}


/**
 * Clears all the pending interrupts.
 * If the Dma status register is read then all the interrupts gets cleared
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
static inline void nss_gmac_clear_interrupt(struct nss_gmac_dev *gmacdev)
{
	uint32_t data;

	data = readl_relaxed((unsigned char *)gmacdev->dma_base + dma_status);
	writel_relaxed(data, (unsigned char *)gmacdev->dma_base + dma_status);
}


/**
 * Returns the all unmasked interrupt status after reading the dma_status
 * register.
 * @param[in] pointer to nss_gmac_dev.
 * @return 0 upon success. Error code upon failure.
 */
static inline uint32_t nss_gmac_get_interrupt_type(struct nss_gmac_dev *gmacdev)
{
	uint32_t interrupts = 0;

	interrupts =
	    nss_gmac_read_reg((uint32_t *)gmacdev->dma_base, dma_status);

	/* Clear interrupt here */
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base, dma_status,
			   interrupts);

	return interrupts;
}


/**
 * Returns the interrupt mask.
 * @param[in] pointer to nss_gmac_dev.
 * @return 0 upon success. Error code upon failure.
 */
static inline uint32_t nss_gmac_get_interrupt_mask(struct nss_gmac_dev *gmacdev)
{
	return nss_gmac_read_reg((uint32_t *)gmacdev->dma_base, dma_interrupt);
}


/**
 * @brief Enables the DMA interrupt as specified by the bit mask.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] bit mask of interrupts to be enabled.
 * @return returns void.
 */
static inline void nss_gmac_enable_interrupt(struct nss_gmac_dev *gmacdev,
					     uint32_t interrupts)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base, dma_interrupt,
			   interrupts);
}


/**
 * @brief Disable all the interrupts.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
static inline void nss_gmac_disable_mac_interrupt(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base, gmac_interrupt_mask,
			   0xffffffff);
}


/**
 * Disable all the interrupts.
 * Disables all DMA interrupts.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 * @note This function disabled all the interrupts, if you want to disable a
 * particular interrupt then use nss_gmac_disable_interrupt().
 */
static inline void nss_gmac_disable_interrupt_all(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base, dma_interrupt,
			   dma_int_disable);
	nss_gmac_disable_mac_interrupt(gmacdev);
}


/**
 * Disable interrupt according to the bitfield supplied.
 * Disables only those interrupts specified in the bit mask in second argument.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] bit mask for interrupts to be disabled.
 * @return returns void.
 */
static inline void nss_gmac_disable_interrupt(struct nss_gmac_dev *gmacdev,
					      uint32_t interrupts)
{
	uint32_t data = 0;

	data = ~interrupts & readl_relaxed((unsigned char *)gmacdev->dma_base
							+ dma_interrupt);
	writel_relaxed(data, (unsigned char *)gmacdev->dma_base
							+ dma_interrupt);
}


void nss_gmac_enable_dma_rx(struct nss_gmac_dev *gmacdev);
void nss_gmac_enable_dma_tx(struct nss_gmac_dev *gmacdev);


/**
 * Resumes the DMA Transmission.
 * the dma_tx_poll_demand is written. (the data writeen could be anything).
 * This forces the DMA to resume transmission.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
static inline void nss_gmac_resume_dma_tx(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base,
							dma_tx_poll_demand, 0);
}


/**
 * Resumes the DMA Reception.
 * the dma_rx_poll_demand is written. (the data writeen could be anything).
 * This forces the DMA to resume reception.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
static inline void nss_gmac_resume_dma_rx(struct nss_gmac_dev *gmacdev)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base,
							dma_rx_poll_demand, 0);
}

void nss_gmac_take_desc_ownership(struct dma_desc *desc);
void nss_gmac_take_desc_ownership_rx(struct nss_gmac_dev *gmacdev);
void nss_gmac_take_desc_ownership_tx(struct nss_gmac_dev *gmacdev);
void nss_gmac_disable_dma_tx(struct nss_gmac_dev *gmacdev);
void nss_gmac_disable_dma_rx(struct nss_gmac_dev *gmacdev);

/*******************MMC APIs***************************************/
void nss_gmac_disable_mmc_tx_interrupt(struct nss_gmac_dev *gmacdev,
							uint32_t mask);
void nss_gmac_disable_mmc_rx_interrupt(struct nss_gmac_dev *gmacdev,
							uint32_t mask);
void nss_gmac_disable_mmc_ipc_rx_interrupt(struct nss_gmac_dev *gmacdev,
					   uint32_t mask);

#endif /* End of file */
