/* Copyright (c) 2013, The Linux Foundation. All rights reserved.*/
/*
 * @file
 * This file defines the function prototypes for the NSS GMAC device.
 * Since the phy register mapping are standardised, the phy register map and the
 * bit definitions remain the same for other phy as well.
 * This also defines some of the Ethernet related parmeters.
 *  -----------------------------REVISION HISTORY------------------------------------
 * Qualcomm Atheros		   01/Mar/2013             	Modified for QCA NSS
 * Synopsys			   01/Aug/2007				Created
 */

#ifndef __NSS_GMAC_DEV_H__
#define __NSS_GMAC_DEV_H__

#include <linux/if_vlan.h>
#include <linux/platform_device.h>
#include <linux/ethtool.h>
#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>
#include <asm/io.h>

#if (NSS_GMAC_DT_SUPPORT == 1)
#include <msm_nss_gmac.h>
#else
#include <mach/msm_nss_gmac.h>
#endif

#include <nss_api_if.h>

#define NSS_GMAC_IPC_OFFLOAD

#define NSS_GMAC_MACBASE	0x0000	/* Offset of Mac registers within
					   GMAC register space                  */
#define NSS_GMAC_DMABASE	0x1000	/* Offset of Dma registers within
					   GMAC register space                  */
#define NSS_GMAC_REG_BLOCK_LEN	0x4000	/* Length of the register block to map  */

#define NSS_GMAC_TX_DESC_SIZE	128	/* Tx Descriptors needed in the
					   Descriptor pool/queue                */
#define NSS_GMAC_RX_DESC_SIZE	128	/* Rx Descriptors needed in the
					   Descriptor pool/queue                */
#define DEFAULT_DELAY_VARIABLE  10
#define DEFAULT_LOOP_VARIABLE   10
#define MDC_CLK_DIV             (GmiiCsrClk0)

#define NSS_GMAC_EXTRA			NET_IP_ALIGN
#define NSS_GMAC_JUMBO_MTU		9600		/* Max jumbo frame size	*/

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
					| SUPPORTED_Asym_Pause);

/* Ethtool specific list of GMAC advertised features */
#define NSS_GMAC_ADVERTISED_FEATURES	(ADVERTISED_10baseT_Half	\
					| ADVERTISED_10baseT_Full	\
					| ADVERTISED_100baseT_Half	\
					| ADVERTISED_100baseT_Full	\
					| ADVERTISED_1000baseT_Full	\
					| ADVERTISED_Autoneg		\
					| ADVERTISED_TP			\
					| ADVERTISED_Pause		\
					| ADVERTISED_Asym_Pause);

/* MDIO address space register offsets */
#define ATH_MII_MMD_ACCESS_CTRL				0xD
#define ATH_MII_MMD_ACCESS_ADDR_DATA			0xE

/* MMD deivce addresses */
#define ATH_MMD_DEVADDR_3				3
#define ATH_MMD_DEVADDR_7				7

const static uint8_t nss_gmac_driver_string[] =
    "NSS GMAC Driver for RTL v" NSS_GMAC_RTL_VER;
const static uint8_t nss_gmac_driver_version[] = "1.0";
const static uint8_t nss_gmac_copyright[] =
    "Copyright (c) 2013 The Linux Foundation. All rights reserved.";

/**
 * @brief DMA Descriptor Structure
 * The structure is common for both receive and transmit descriptors
 * The descriptor is of 4 words, but our structrue contains 6 words where last
 * two words are to hold the virtual address of the network buffer pointers for
 * driver's use.From the GMAC core release 3.50a onwards, the Enhanced Descriptor
 * structure got changed. The descriptor (both transmit and receive) are of 8 words
 * each rather the 4 words of normal descriptor structure.
 * Whenever IEEE 1588 Timestamping is enabled TX/RX DESC6 provides the lower 32 bits
 * of Timestamp value and TX/RX DESC7 provides the upper 32 bits of Timestamp value.
 * In addition to this whenever extended status bit is set (RX DESC0 bit 0),
 * RX DESC4 contains the extended status information.
 */
typedef struct DmaDescStruct {
	volatile uint32_t status;	/* Status                               */
	volatile uint32_t length;	/* Buffer 1  and Buffer 2 length        */
	volatile uint32_t buffer1;	/* Network Buffer 1 pointer (Dma-able)  */
	volatile uint32_t data1;	/* This holds virtual address of buffer1,
					   not used by DMA                      */

	/* This data below is used only by driver */
	volatile uint32_t extstatus;	/* Extended status of a Rx Descriptor   */
	volatile uint32_t reserved1;	/* Reserved word                        */
	volatile uint32_t timestamplow;	/* Lower 32 bits of the 64
					   bit timestamp value                  */
	volatile uint32_t timestamphigh;	/* Higher 32 bits of the 64
						   bit timestamp value          */
} DmaDesc;

enum DescMode {
	RINGMODE = 0x00000001,
	CHAINMODE = 0x00000002,
};

#define NSS_GMAC_WORKQUEUE_NAME		"gmac_workqueue"
struct nss_gmac_global_ctx;

/**
 * @brief NSS GMAC device data
 */
typedef struct _nss_gmac_dev {
	uint32_t mac_base;	/* base address of MAC registers                */
	uint32_t dma_base;	/* base address of DMA registers                */
	uint32_t phy_base;	/* PHY device address on MII interface          */
	uint32_t macid;		/* Sequence number of Mac on the platform       */
	uint32_t version;	/* Gmac Revision version                        */
	uint32_t emulation;	/* Running on emulation platform		*/
	volatile unsigned long int flags;	/* status flags			*/

	dma_addr_t tx_desc_dma;	/* Dma-able address of first tx descriptor
				   either in ring or chain mode, this is used
				   by the GMAC device                           */

	dma_addr_t rx_desc_dma;	/* Dma-albe address of first rx descriptor
				   either in ring or chain mode, this is
				   used by the GMAC device                      */

	DmaDesc *tx_desc;	/* start address of TX descriptors ring or
				   chain, this is used by the driver            */

	DmaDesc *rx_desc;	/* start address of RX descriptors ring or
				   chain, this is used by the driver            */

	uint32_t busy_tx_desc;	/* Number of Tx Descriptors owned by
				   DMA at any given time                        */
	uint32_t busy_rx_desc;	/* Number of Rx Descriptors owned by
				   DMA at any given time                        */

	uint32_t rx_desc_count;	/* number of rx descriptors in the
				   tx descriptor queue/pool                     */
	uint32_t tx_desc_count;	/* number of tx descriptors in the
				   rx descriptor queue/pool                     */

	uint32_t tx_busy;	/* index of the tx descriptor owned by DMA,
				   is obtained by nss_gmac_get_tx_qptr()        */

	uint32_t tx_next;	/* index of the tx descriptor next available
				   with driver, given to DMA by
				   nss_gmac_set_tx_qptr()                       */

	uint32_t rx_busy;	/* index of the rx descriptor owned by DMA,
				   obtained by nss_gmac_get_rx_qptr()           */

	uint32_t rx_next;	/* index of the rx descriptor next available
				   with driver, given to DMA by
				   nss_gmac_set_rx_qptr()                       */

	DmaDesc *tx_busy_desc;	/* Tx Descriptor address corresponding
				   to the index TxBusy                          */
	DmaDesc *tx_next_desc;	/* Tx Descriptor address corresponding
				   to the index TxNext                          */
	DmaDesc *rx_busy_desc;	/* Rx Descriptor address corresponding
				   to the index TxBusy                          */
	DmaDesc *rx_next_desc;	/* Rx Descriptor address corresponding
				   to the index RxNext                          */

	/*
	 * Phy related stuff
	 */
	uint32_t mdc_clk_div;	/* Clock divider value programmed in the hardware */
	uint32_t link_state;	/* Link status as reported by the Phy           */
	uint32_t duplex_mode;	/* Duplex mode of the Phy                       */
	uint32_t speed;		/* Speed of the Phy                             */
	uint32_t loop_back_mode;/* Loopback status of the Phy                   */
	uint32_t phy_mii_type;	/* RGMII/SGMII/QSGMII                           */
	uint32_t rgmii_delay;	/* RGMII delay settings                         */
	uint32_t pause;		/* Current flow control settings                */
	uint32_t first_linkup_done;	/* when set, it indicates that first
					   link up detection after interface
					   bring up has been done               */
	int32_t forced_speed;		/* Forced Speed */
	int32_t forced_duplex;		/* Forced Duplex */

	struct net_device *netdev;
	struct platform_device *pdev;
	struct delayed_work gmacwork;
	struct napi_struct *napi;
	struct rtnl_link_stats64 stats;	/* statistics counters                  */
	spinlock_t stats_lock;		/* Lock to retrieve stats atomically    */
	spinlock_t slock;		/* Lock to protect datapath		*/
	struct mutex link_mutex;	/* Lock to protect link status change	*/
	uint32_t gmac_power_down;	/* indicate to ISR whether the
					   interrupts occured in the process
					   of powering down                     */

	struct nss_gmac_global_ctx *ctx;/* Global NSS GMAC context              */
	struct resource *memres;	/* memory resource                      */

	void *nss_gmac_ctx;		/* context when NSS owns GMACs          */
	struct phy_device *phydev;	/* Phy device 				*/
	uint32_t notify_open;		/* 0 = Open for gmac not issued to NSS
					   1 = Open for gmac issued to NSS	*/
	uint32_t nss_state;		/* NSS initialization state		*/
	struct nss_gmac_sync nss_stats;	/* Stats synced from NSS                */
	struct mii_bus *miibus;		/* MDIO bus associated with this GMAC	*/
} nss_gmac_dev;


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
	uint8_t *nss_base;		/* Base address of NSS GMACs'
					   global interface registers		*/
	uint32_t *tcsr_base;		/* Base address of tcsr registers	*/
	uint32_t *qsgmii_base;
	uint32_t *clk_ctl_base;		/* Base address of platform
					   clock control registers */
	spinlock_t reg_lock;	/* Lock to protect NSS register	*/
	nss_gmac_dev *nss_gmac[NSS_MAX_GMACS];
	bool common_init_done;			/* Flag to hold common init done state */
};


enum nss_gmac_state {
	__NSS_GMAC_UP,		/* set to indicate the interface is UP		*/
	__NSS_GMAC_CLOSING,	/* set to indicate the interface is closing	*/
	__NSS_GMAC_RXCSUM,	/* Rx checksum enabled				*/
	__NSS_GMAC_AUTONEG,	/* Autonegotiation Enabled			*/
	__NSS_GMAC_RXPAUSE,
	__NSS_GMAC_TXPAUSE,
	__NSS_GMAC_LINKPOLL,	/* Poll link status				*/
};

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
	ath_mmd_smart_eee_ctrl_3 = 0x805D,	/* MMD smart EEE control 3 */
	ath_mmd_eee_adv = 0x003C,		/* MMD EEE Advertisment */
};

/* MMD Access Control function bits */
enum ath_mmd_access_ctrl_function_bit_descriptions {
	ath_mmd_acc_ctrl_addr = 0x0000,		/* address */
	ath_mmd_acc_ctrl_data_no_incr = 0x4000,	/* data, no post incr */
	ath_mmd_acc_ctrl_data_incr_rw = 0x8000,	/* data, post incr on r/w */
	ath_mmd_acc_ctrl_data_incr_w = 0xC000,	/* data, post incr on write only */
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
 * For Pci based system address is BARx + GmacRegisterBase
 * For any other system translation is done accordingly
 **********************************************************/
enum GmacRegisters {
	GmacConfig = 0x0000,		/* Mac config Register                  */
	GmacFrameFilter = 0x0004,	/* Mac frame filtering controls         */
	GmacHashHigh = 0x0008,		/* Multi-cast hash table high           */
	GmacHashLow = 0x000C,		/* Multi-cast hash table low            */
	GmacGmiiAddr = 0x0010,		/* GMII address Register(ext. Phy)      */
	GmacGmiiData = 0x0014,		/* GMII data Register(ext. Phy)         */
	GmacFlowControl = 0x0018,	/* Flow control Register                */
	GmacVlan = 0x001C,		/* VLAN tag Register (IEEE 802.1Q)      */
	GmacVersion = 0x0020,		/* GMAC Core Version Register           */
	GmacWakeupAddr = 0x0028,	/* GMAC wake-up frame filter adrress reg */
	GmacPmtCtrlStatus = 0x002C,	/* PMT control and status register      */
	GmacInterruptStatus = 0x0038,	/* Mac Interrupt ststus register        */
	GmacInterruptMask = 0x003C,	/* Mac Interrupt Mask register          */
	GmacAddr0High = 0x0040,		/* Mac address0 high Register           */
	GmacAddr0Low = 0x0044,		/* Mac address0 low Register            */
	GmacAddr1High = 0x0048,		/* Mac address1 high Register           */
	GmacAddr1Low = 0x004C,		/* Mac address1 low Register            */
	GmacAddr2High = 0x0050,		/* Mac address2 high Register           */
	GmacAddr2Low = 0x0054,		/* Mac address2 low Register            */
	GmacAddr3High = 0x0058,		/* Mac address3 high Register           */
	GmacAddr3Low = 0x005C,		/* Mac address3 low Register            */
	GmacAddr4High = 0x0060,		/* Mac address4 high Register           */
	GmacAddr4Low = 0x0064,		/* Mac address4 low Register            */
	GmacAddr5High = 0x0068,		/* Mac address5 high Register           */
	GmacAddr5Low = 0x006C,		/* Mac address5 low Register            */
	GmacAddr6High = 0x0070,		/* Mac address6 high Register           */
	GmacAddr6Low = 0x0074,		/* Mac address6 low Register            */
	GmacAddr7High = 0x0078,		/* Mac address7 high Register           */
	GmacAddr7Low = 0x007C,		/* Mac address7 low Register            */
	GmacAddr8High = 0x0080,		/* Mac address8 high Register           */
	GmacAddr8Low = 0x0084,		/* Mac address8 low Register            */
	GmacAddr9High = 0x0088,		/* Mac address9 high Register           */
	GmacAddr9Low = 0x008C,		/* Mac address9 low Register            */
	GmacAddr10High = 0x0090,	/* Mac address10 high Register          */
	GmacAddr10Low = 0x0094,		/* Mac address10 low Register           */
	GmacAddr11High = 0x0098,	/* Mac address11 high Register          */
	GmacAddr11Low = 0x009C,		/* Mac address11 low Register           */
	GmacAddr12High = 0x00A0,	/* Mac address12 high Register          */
	GmacAddr12Low = 0x00A4,		/* Mac address12 low Register           */
	GmacAddr13High = 0x00A8,	/* Mac address13 high Register          */
	GmacAddr13Low = 0x00AC,		/* Mac address13 low Register           */
	GmacAddr14High = 0x00B0,	/* Mac address14 high Register          */
	GmacAddr14Low = 0x00B4,		/* Mac address14 low Register           */
	GmacAddr15High = 0x00B8,	/* Mac address15 high Register          */
	GmacAddr15Low = 0x00BC,		/* Mac address15 low Register           */
	GmacMiiStatus = 0x00D8,		/* SGMII/RGMII/SMII Status Register     */

	/* Time Stamp Register Map */
	GmacTSControl = 0x0700,		/* Controls the Timestamp update logic:
					   only when IEEE 1588 time stamping is
					   enabled in corekit                   */

	GmacTSSubSecIncr = 0x0704,	/* 8 bit value by which sub second
					   register is incremented : only when
					   IEEE 1588 time stamping without
					   external timestamp input             */

	GmacTSHigh = 0x0708,		/* 32 bit seconds(MS): only when
					   IEEE 1588 time stamping without
					   external timestamp input             */

	GmacTSLow = 0x070C,		/* 32 bit nano seconds(MS): only when
					   IEEE 1588 time stamping without
					   external timestamp input             */

	GmacTSHighUpdate = 0x0710,	/* 32 bit seconds(MS) to be written/added
					   /subtracted: only when IEEE 1588 time
					   stamping without external timestamp  */

	GmacTSLowUpdate = 0x0714,	/* 32 bit nano seconds(MS) to be
					   writeen/added/subtracted: only when
					   IEEE 1588 time stamping without
					   external timestamp input             */

	GmacTSAddend = 0x0718,		/* Used by Software to readjust the clock
					   frequency linearly: only when
					   IEEE 1588 time stamping without
					   external timestamp input             */

	GmacTSTargetTimeHigh = 0x071C,	/* 32 bit seconds(MS) to be compared
					   with system time: only when IEEE 1588
					   time stamping without external
					   timestamp input                      */

	GmacTSTargetTimeLow = 0x0720,	/* 32 bit nano seconds(MS) to be compared
					   with system time: only when IEEE 1588
					   time stamping without external
					   timestamp input                      */

	GmacTSHighWord = 0x0724,	/* Time Stamp Higher Word Register(Version
					   2 only); only lower 16 bits are valid */

	/*GmacTSHighWordUpdate    = 0x072C, *//* Time Stamp Higher Word Update Register
	   (Version 2 only); only lower
	   16 bits are valid                    */

	GmacTSStatus = 0x0728,	/* Time Stamp Status Register           */
};

/**********************************************************
 * GMAC Network interface registers
 * This explains the Register's Layout

 * FES is Read only by default and is enabled only when Tx
 * Config Parameter is enabled for RGMII/SGMII interface
 * during CoreKit Config.

 * DM is Read only with value 1'b1 in Full duplex only Config
 **********************************************************/

/* GmacConfig              = 0x0000,    Mac config Register  Layout */
enum GmacConfigReg {
	GmacTwokpe = 0x08000000,
	GmacTwokpeEnable = 0x08000000,
	GmacTwokpeDisable = 0x00000000,
	GmacTCEnable = 0x01000000,
	GmacWatchdog = 0x00800000,
	GmacWatchdogDisable = 0x00800000,	/* (WD)Disable watchdog timer on Rx     */
	GmacWatchdogEnable = 0x00000000,	/* Enable watchdog timer                */
	GmacJabber = 0x00400000,
	GmacJabberDisable = 0x00400000,		/* (JD)Disable jabber timer on Tx       */
	GmacJabberEnable = 0x00000000,		/* Enable jabber timer                  */
	GmacFrameBurst = 0x00200000,
	GmacFrameBurstEnable = 0x00200000,	/* (BE)Enable frame bursting during Tx  */
	GmacFrameBurstDisable = 0x00000000,	/* Disable frame bursting               */
	GmacJumboFrame = 0x00100000,
	GmacJumboFrameEnable = 0x00100000,	/* (JE)Enable jumbo frame for Tx        */
	GmacJumboFrameDisable = 0x00000000,	/* Disable jumbo frame                  */
	GmacInterFrameGap7 = 0x000E0000,	/* (IFG) Config7 - 40 bit times         */
	GmacInterFrameGap6 = 0x000C0000,	/* (IFG) Config6 - 48 bit times         */
	GmacInterFrameGap5 = 0x000A0000,	/* (IFG) Config5 - 56 bit times         */
	GmacInterFrameGap4 = 0x00080000,	/* (IFG) Config4 - 64 bit times         */
	GmacInterFrameGap3 = 0x00040000,	/* (IFG) Config3 - 72 bit times         */
	GmacInterFrameGap2 = 0x00020000,	/* (IFG) Config2 - 80 bit times         */
	GmacInterFrameGap1 = 0x00010000,	/* (IFG) Config1 - 88 bit times         */
	GmacInterFrameGap0 = 0x00000000,	/* (IFG) Config0 - 96 bit times         */
	GmacDisableCrs = 0x00010000,
	GmacMiiGmii = 0x00008000,
	GmacSelectMii = 0x00008000,		/* (PS)Port Select-MII mode             */
	GmacSelectGmii = 0x00000000,		/* GMII mode                            */
	GmacFESpeed100 = 0x00004000,		/*(FES)Fast Ethernet speed 100Mbps      */
	GmacFESpeed10 = 0x00000000,		/* 10Mbps                               */
	GmacRxOwn = 0x00002000,
	GmacDisableRxOwn = 0x00002000,		/* (DO)Disable receive own packets      */
	GmacEnableRxOwn = 0x00000000,		/* Enable receive own packets           */
	GmacLoopback = 0x00001000,
	GmacLoopbackOn = 0x00001000,		/* (LM)Loopback mode for GMII/MII       */
	GmacLoopbackOff = 0x00000000,		/* Normal mode                          */
	GmacDuplex = 0x00000800,
	GmacFullDuplex = 0x00000800,		/* (DM)Full duplex mode                 */
	GmacHalfDuplex = 0x00000000,		/* Half duplex mode                     */
	GmacRxIpcOffload = 0x00000400,		/* IPC checksum offload                 */
	GmacRetry = 0x00000200,
	GmacRetryDisable = 0x00000200,		/* (DR)Disable Retry                    */
	GmacRetryEnable = 0x00000000,		/* Enable retransmission as per BL      */
	GmacLinkUp = 0x00000100,		/* (LUD)Link UP                         */
	GmacLinkDown = 0x00000100,		/* Link Down                            */
	GmacPadCrcStrip = 0x00000080,
	GmacPadCrcStripEnable = 0x00000080,	/* (ACS) Automatic Pad/Crc strip enable */
	GmacPadCrcStripDisable = 0x00000000,	/* Automatic Pad/Crc stripping disable  */
	GmacBackoffLimit = 0x00000060,
	GmacBackoffLimit3 = 0x00000060,		/* (BL)Back-off limit in HD mode        */
	GmacBackoffLimit2 = 0x00000040,
	GmacBackoffLimit1 = 0x00000020,
	GmacBackoffLimit0 = 0x00000000,
	GmacDeferralCheck = 0x00000010,
	GmacDeferralCheckEnable = 0x00000010,	/* (DC)Deferral check enable in HD mode */
	GmacDeferralCheckDisable = 0x00000000,	/* Deferral check disable                */
	GmacTx = 0x00000008,
	GmacTxEnable = 0x00000008,		/* (TE)Transmitter enable               */
	GmacTxDisable = 0x00000000,		/* Transmitter disable                  */
	GmacRx = 0x00000004,
	GmacRxEnable = 0x00000004,		/* (RE)Receiver enable                  */
	GmacRxDisable = 0x00000000,		/* Receiver disable                     */
};

/* GmacFrameFilter	= 0x0004,	Mac frame filtering controls Register Layout */
enum GmacFrameFilterReg {
	GmacFilter = 0x80000000,
	GmacFilterOff = 0x80000000,		/* (RA)Receive all incoming packets */
	GmacFilterOn = 0x00000000,		/* Receive filtered packets only */
	GmacHashPerfectFilter = 0x00000400,	/* Hash or Perfect Filter enable */
	GmacSrcAddrFilter = 0x00000200,
	GmacSrcAddrFilterEnable = 0x00000200,	/* (SAF)Source Address Filter enable */
	GmacSrcAddrFilterDisable = 0x00000000,
	GmacSrcInvaAddrFilter = 0x00000100,
	GmacSrcInvAddrFilterEn = 0x00000100,	/* (SAIF)Inv Src Addr Filter enable */
	GmacSrcInvAddrFilterDis = 0x00000000,
	GmacPassControl = 0x000000C0,
	GmacPassControl3 = 0x000000C0,		/* (PCS)Forwards ctrl frms that pass AF */
	GmacPassControl2 = 0x00000080,		/* Forwards all control frames          */
	GmacPassControl1 = 0x00000040,		/* Does not pass control frames         */
	GmacPassControl0 = 0x00000000,		/* Does not pass control frames         */
	GmacBroadcast = 0x00000020,
	GmacBroadcastDisable = 0x00000020,	/* (DBF)Disable Rx of broadcast frames  */
	GmacBroadcastEnable = 0x00000000,	/* Enable broadcast frames              */
	GmacMulticastFilter = 0x00000010,
	GmacMulticastFilterOff = 0x00000010,	/* (PM) Pass all multicast packets      */
	GmacMulticastFilterOn = 0x00000000,	/* Pass filtered multicast packets      */
	GmacDestAddrFilter = 0x00000008,
	GmacDestAddrFilterInv = 0x00000008,	/* (DAIF)Inverse filtering for DA       */
	GmacDestAddrFilterNor = 0x00000000,	/* Normal filtering for DA              */
	GmacMcastHashFilter = 0x00000004,
	GmacMcastHashFilterOn = 0x00000004,	/* (HMC)perfom multicast hash filtering */
	GmacMcastHashFilterOff = 0x00000000,	/* perfect filtering only               */
	GmacUcastHashFilter = 0x00000002,
	GmacUcastHashFilterOn = 0x00000002,	/* (HUC)Unicast Hash filtering only     */
	GmacUcastHashFilterOff = 0x00000000,	/* perfect filtering only               */
	GmacPromiscuousMode = 0x00000001,
	GmacPromiscuousModeOn = 0x00000001,	/* Receive all frames                   */
	GmacPromiscuousModeOff = 0x00000000,	/* Receive filtered packets only        */
};

/* GmacGmiiAddr		= 0x0010,	GMII address Register(ext. Phy) Layout	*/
enum GmacGmiiAddrReg {
	GmiiDevMask = 0x0000F800,	/* (PA)GMII device address              */
	GmiiDevShift = 11,
	GmiiRegMask = 0x000007C0,	/* (GR)GMII register in selected Phy    */
	GmiiRegShift = 6,
	GmiiCsrClkShift = 2,		/* CSR Clock bit Shift                  */
	GmiiCsrClkMask = 0x0000003C,	/* CSR Clock bit Mask                   */
	GmiiCsrClk5 = 0x00000014,	/* (CR)CSR Clock Range  250-300 MHz     */
	GmiiCsrClk4 = 0x00000010,	/*                      150-250 MHz     */
	GmiiCsrClk3 = 0x0000000C,	/*                      35-60 MHz       */
	GmiiCsrClk2 = 0x00000008,	/*                      20-35 MHz       */
	GmiiCsrClk1 = 0x00000004,	/*                      100-150 MHz     */
	GmiiCsrClk0 = 0x00000000,	/*                      60-100 MHz      */
	GmiiWrite = 0x00000002,		/* (GW)Write to register                */
	GmiiRead = 0x00000000,		/* Read from register                   */
	GmiiBusy = 0x00000001,		/* (GB)GMII interface is busy           */
};

/* GmacGmiiData		= 0x0014,	GMII data Register(ext. Phy) Layout	*/
enum GmacGmiiDataReg {
	GmiiDataMask = 0x0000FFFF,		/* (GD)GMII Data                */
};

/* GmacFlowControl	= 0x0018,	Flow control Register Layout		*/
enum GmacFlowControlReg {
	GmacPauseTimeMask = 0xFFFF0000,		/* (PT) PAUSE TIME field
						   in the control frame         */
	GmacPauseTimeShift = 16,
	GmacPauseLowThresh = 0x00000030,
	GmacPauseLowThresh3 = 0x00000030,	/* (PLT)thresh for pause
						   tmr 256 slot time            */
	GmacPauseLowThresh2 = 0x00000020,	/*      144 slot time           */
	GmacPauseLowThresh1 = 0x00000010,	/*      28 slot time            */
	GmacPauseLowThresh0 = 0x00000000,	/*      4 slot time             */
	GmacUnicastPauseFrame = 0x00000008,
	GmacUnicastPauseFrameOn = 0x00000008,	/* (UP)Detect pause frame
						   with unicast addr.           */
	GmacUnicastPauseFrameOff = 0x00000000,	/* Detect only pause frame
						   with multicast addr.         */
	GmacRxFlowControl = 0x00000004,
	GmacRxFlowControlEnable = 0x00000004,	/* (RFE)Enable Rx flow control  */
	GmacRxFlowControlDisable = 0x00000000,	/* Disable Rx flow control      */
	GmacTxFlowControl = 0x00000002,
	GmacTxFlowControlEnable = 0x00000002,	/* (TFE)Enable Tx flow control  */
	GmacTxFlowControlDisable = 0x00000000,	/* Disable flow control         */
	GmacFlowControlBackPressure = 0x00000001,
	GmacSendPauseFrame = 0x00000001,	/* (FCB/PBA)send pause
						   frm/Apply back pressure      */
};

/* GmacInterruptStatus	= 0x0038,	Mac Interrupt ststus register		*/
enum GmacInterruptStatusBitDefinition {
	GmacTSIntSts = 0x00000200,	/* set if int generated due to TS (Read
					   Time Stamp Status Register to know details) */
	GmacMmcRxChksumOffload = 0x00000080,	/* set if int generated in MMC RX
						   CHECKSUM OFFLOAD int register        */
	GmacMmcTxIntSts = 0x00000040,	/* set if int generated in MMC TX Int register */
	GmacMmcRxIntSts = 0x00000020,	/* set if int generated in MMC RX Int register */
	GmacMmcIntSts = 0x00000010,	/* set if any of the above bit [7:5] is set */
	GmacPmtIntSts = 0x00000008,	/* set whenver magic pkt/wake-on-lan
					   frame is received                    */
	GmacPcsAnComplete = 0x00000004,	/* set when AN is complete in
					   TBI/RTBI/SGMIII phy interface        */
	GmacPcsLnkStsChange = 0x00000002,	/* set if any lnk status change in
						   TBI/RTBI/SGMII interface             */
	GmacRgmiiIntSts = 0x00000001,	/* set if any change in lnk
					   status of RGMII interface            */
};

/* GmacInterruptMask	= 0x003C,	Mac Interrupt Mask register		*/
enum GmacInterruptMaskBitDefinition {
	GmacTSIntMask = 0x00000200,		/* when set disables the time
						   stamp interrupt generation   */
	GmacPmtIntMask = 0x00000008,		/* when set Disables the assertion
						   of PMT interrupt             */
	GmacPcsAnIntMask = 0x00000004,		/* When set disables the assertion
						   of PCS AN complete interrupt */
	GmacPcsLnkStsIntMask = 0x00000002,	/* when set disables the assertion of
						   PCS lnk status change interrupt */
	GmacRgmiiIntMask = 0x00000001,		/* when set disables the
						   assertion of RGMII interrupt */
};

/**********************************************************
 * GMAC DMA registers
 * For Pci based system address is BARx + GmaDmaBase
 * For any other system translation is done accordingly
 **********************************************************/

enum DmaRegisters {
	DmaBusMode = 0x0000,	/* CSR0 - Bus Mode Register                     */
	DmaTxPollDemand = 0x0004,	/* CSR1 - Transmit Poll Demand Register         */
	DmaRxPollDemand = 0x0008,	/* CSR2 - Receive Poll Demand Register          */
	DmaRxBaseAddr = 0x000C,	/* CSR3 - Receive Descriptor list base address  */
	DmaTxBaseAddr = 0x0010,	/* CSR4 - Transmit Descriptor list base address */
	DmaStatus = 0x0014,	/* CSR5 - Dma status Register                   */
	DmaControl = 0x0018,	/* CSR6 - Dma Operation Mode Register           */
	DmaInterrupt = 0x001C,	/* CSR7 - Interrupt enable                      */
	DmaMissedFr = 0x0020,	/* CSR8 - Missed Frame & Buffer overflow Counter */
	DmaAxiBusMode = 0x0028,	/* AXI Bus Mode Settings			*/
	DmaTxCurrDesc = 0x0048,	/*      - Current host Tx Desc Register         */
	DmaRxCurrDesc = 0x004C,	/*      - Current host Rx Desc Register         */
	DmaTxCurrAddr = 0x0050,	/* CSR20 - Current host transmit buffer address */
	DmaRxCurrAddr = 0x0054,	/* CSR21 - Current host receive buffer address  */
};

/**********************************************************
 * DMA Engine registers Layout
 **********************************************************/

/* DmaBusMode	= 0x0000,	CSR0 - Bus Mode					*/
enum DmaBusModeReg {
	DmaFixedBurstEnable = 0x00010000,	/* (FB)Fixed Burst SINGLE, INCR4,
						   INCR8 or INCR16                      */
	DmaFixedBurstDisable = 0x00000000,	/* SINGLE, INCR                         */
	DmaTxPriorityRatio11 = 0x00000000,	/* (PR)TX:RX DMA priority ratio 1:1     */
	DmaTxPriorityRatio21 = 0x00004000,	/* (PR)TX:RX DMA priority ratio 2:1     */
	DmaTxPriorityRatio31 = 0x00008000,	/* (PR)TX:RX DMA priority ratio 3:1     */
	DmaTxPriorityRatio41 = 0x0000C000,	/* (PR)TX:RX DMA priority ratio 4:1     */
	DmaAddressAlignedBeats = 0x02000000,	/* Address Aligned beats		*/
	DmaBurstLengthx8 = 0x01000000,		/* When set mutiplies the PBL by 8      */
	DmaBurstLength256 = 0x01002000,		/*(DmaBurstLengthx8
						   | DmaBurstLength32) = 256            */
	DmaBurstLength128 = 0x01001000,		/*(DmaBurstLengthx8
						   | DmaBurstLength16) = 128            */
	DmaBurstLength64 = 0x01000800,		/*(DmaBurstLengthx8
						   | DmaBurstLength8) = 64              */
	DmaBurstLength32 = 0x00002000,		/* (PBL) programmable
						   Dma burst length = 32                */
	DmaBurstLength16 = 0x00001000,		/* Dma burst length = 16                */
	DmaBurstLength8 = 0x00000800,		/* Dma burst length = 8                 */
	DmaBurstLength4 = 0x00000400,		/* Dma burst length = 4                 */
	DmaBurstLength2 = 0x00000200,		/* Dma burst length = 2                 */
	DmaBurstLength1 = 0x00000100,		/* Dma burst length = 1                 */
	DmaBurstLength0 = 0x00000000,		/* Dma burst length = 0                 */
	DmaDescriptor8Words = 0x00000080,	/* Enh Descriptor works  1=>
						   8 word descriptor                    */
	DmaDescriptor4Words = 0x00000000,	/* Enh Descriptor works  0=>
						   4 word descriptor                    */
	DmaDescriptorSkip16 = 0x00000040,	/* (DSL)Descriptor skip
						   length (no.of dwords)                */
	DmaDescriptorSkip8 = 0x00000020,	/* between two unchained descriptors    */
	DmaDescriptorSkip4 = 0x00000010,
	DmaDescriptorSkip2 = 0x00000008,
	DmaDescriptorSkip1 = 0x00000004,
	DmaDescriptorSkip0 = 0x00000000,
	DmaArbitRr = 0x00000000,		/* (DA) DMA RR arbitration              */
	DmaArbitPr = 0x00000002,		/* Rx has priority over Tx              */
	DmaResetOn = 0x00000001,		/* (SWR)Software Reset DMA engine       */
	DmaResetOff = 0x00000000,
};

/* DmaStatus	= 0x0014,	CSR5 - Dma status Register			*/
enum DmaStatusReg {
	GmacPmtIntr = 0x10000000,	/* (GPI)Gmac subsystem interrupt        */
	GmacMmcIntr = 0x08000000,	/* (GMI)Gmac MMC subsystem interrupt    */
	GmacLineIntfIntr = 0x04000000,	/* Line interface interrupt             */
	DmaErrorBit2 = 0x02000000,	/* (EB)Error bits 0-data buffer,
					   1-desc. access                       */
	DmaErrorBit1 = 0x01000000,	/* (EB)Error bits 0-write trnsf,
					   1-read transfr                       */
	DmaErrorBit0 = 0x00800000,	/* (EB)Error bits 0-Rx DMA, 1-Tx DMA    */
	DmaTxState = 0x00700000,	/* (TS)Transmit process state           */
	DmaTxStopped = 0x00000000,	/* Stopped - Reset or Stop Tx
					   Command issued                       */
	DmaTxFetching = 0x00100000,	/* Running - fetching the Tx descriptor */
	DmaTxWaiting = 0x00200000,	/* Running - waiting for status         */
	DmaTxReading = 0x00300000,	/* Running - reading the data
					   from host memory                     */
	DmaTxSuspended = 0x00600000,	/* Suspended - Tx Descriptor unavailabe */
	DmaTxClosing = 0x00700000,	/* Running - closing Rx descriptor      */
	DmaRxState = 0x000E0000,	/* (RS)Receive process state            */
	DmaRxStopped = 0x00000000,	/* Stopped - Reset or Stop
					   Rx Command issued                    */
	DmaRxFetching = 0x00020000,	/* Running - fetching the Rx descriptor */
	DmaRxWaiting = 0x00060000,	/* Running - waiting for packet         */
	DmaRxSuspended = 0x00080000,	/* Suspended - Rx Descriptor unavailable */
	DmaRxClosing = 0x000A0000,	/* Running - closing descriptor         */
	DmaRxQueuing = 0x000E0000,	/* Running - queuing the recieve
					   frame into host memory               */
	DmaIntNormal = 0x00010000,	/* (NIS)Normal interrupt summary        */
	DmaIntAbnormal = 0x00008000,	/* (AIS)Abnormal interrupt summary      */

	DmaIntEarlyRx = 0x00004000,	/* Early receive interrupt (Normal)     */
	DmaIntBusError = 0x00002000,	/* Fatal bus error (Abnormal)           */
	DmaIntEarlyTx = 0x00000400,	/* Early transmit interrupt (Abnormal)  */
	DmaIntRxWdogTO = 0x00000200,	/* Receive Watchdog Timeout (Abnormal)  */
	DmaIntRxStopped = 0x00000100,	/* Receive process stopped (Abnormal)   */
	DmaIntRxNoBuffer = 0x00000080,	/* Receive buffer unavailable (Abnormal) */
	DmaIntRxCompleted = 0x00000040,	/* Completion of frame reception (Normal) */
	DmaIntTxUnderflow = 0x00000020,	/* Transmit underflow (Abnormal)        */
	DmaIntRcvOverflow = 0x00000010,	/* Receive Buffer overflow interrupt    */
	DmaIntTxJabberTO = 0x00000008,	/* Transmit Jabber Timeout (Abnormal)   */
	DmaIntTxNoBuffer = 0x00000004,	/* Transmit buffer unavailable (Normal) */
	DmaIntTxStopped = 0x00000002,	/* Transmit process stopped (Abnormal)  */
	DmaIntTxCompleted = 0x00000001,	/* Transmit completed (Normal)          */
};

/* DmaControl	= 0x0018,	CSR6 - Dma Operation Mode Register		*/
enum DmaControlReg {
	DmaDisableDropTcpCs = 0x04000000,	/* (DT) Dis. drop. of tcp/ip
						   CS error frames              */
	DmaRxStoreAndForward = 0x02000000,	/* Rx (SF)Store and forward     */
	DmaRxFrameFlush = 0x01000000,		/* Disable Receive Frame Flush  */
	DmaStoreAndForward = 0x00200000,	/* (SF)Store and forward        */
	DmaFlushTxFifo = 0x00100000,		/* (FTF)Tx FIFO controller
						   is reset to default          */
	DmaTxThreshCtrl = 0x0001C000,		/* (TTC)Controls thre Threh of
						   MTL tx Fifo                  */
	DmaTxThreshCtrl16 = 0x0001C000,		/* (TTC)Controls thre Threh of
						   MTL tx Fifo 16               */
	DmaTxThreshCtrl24 = 0x00018000,		/* (TTC)Controls thre Threh of
						   MTL tx Fifo 24               */
	DmaTxThreshCtrl32 = 0x00014000,		/* (TTC)Controls thre Threh of
						   MTL tx Fifo 32               */
	DmaTxThreshCtrl40 = 0x00010000,		/* (TTC)Controls thre Threh of
						   MTL tx Fifo 40               */
	DmaTxThreshCtrl256 = 0x0000c000,	/* (TTC)Controls thre Threh of
						   MTL tx Fifo 256              */
	DmaTxThreshCtrl192 = 0x00008000,	/* (TTC)Controls thre Threh of
						   MTL tx Fifo 192              */
	DmaTxThreshCtrl128 = 0x00004000,	/* (TTC)Controls thre Threh of
						   MTL tx Fifo 128              */
	DmaTxThreshCtrl64 = 0x00000000,		/* (TTC)Controls thre Threh of
						   MTL tx Fifo 64               */
	DmaTxStart = 0x00002000,		/* (ST)Start/Stop transmission  */
	DmaRxFlowCtrlDeact = 0x00401800,	/* (RFD)Rx flow control
						   deact. threhold              */
	DmaRxFlowCtrlDeact1K = 0x00000000,	/* (RFD)Rx flow control
						   deact. threhold (1kbytes)    */
	DmaRxFlowCtrlDeact2K = 0x00000800,	/* (RFD)Rx flow control
						   deact. threhold (2kbytes)    */
	DmaRxFlowCtrlDeact3K = 0x00001000,	/* (RFD)Rx flow control
						   deact. threhold (3kbytes)    */
	DmaRxFlowCtrlDeact4K = 0x00001800,	/* (RFD)Rx flow control
						   deact. threhold (4kbytes)    */
	DmaRxFlowCtrlDeact5K = 0x00400000,	/* (RFD)Rx flow control
						   deact. threhold (4kbytes)    */
	DmaRxFlowCtrlDeact6K = 0x00400800,	/* (RFD)Rx flow control
						   deact. threhold (4kbytes)    */
	DmaRxFlowCtrlDeact7K = 0x00401000,	/* (RFD)Rx flow control
						   deact. threhold (4kbytes)    */
	DmaRxFlowCtrlAct = 0x00800600,		/* (RFA)Rx flow control
						   Act. threhold                */
	DmaRxFlowCtrlAct1K = 0x00000000,	/* (RFA)Rx flow control
						   Act. threhold (1kbytes)      */
	DmaRxFlowCtrlAct2K = 0x00000200,	/* (RFA)Rx flow control
						   Act. threhold (2kbytes)      */
	DmaRxFlowCtrlAct3K = 0x00000400,	/* (RFA)Rx flow control
						   Act. threhold (3kbytes)      */
	DmaRxFlowCtrlAct4K = 0x00000600,	/* (RFA)Rx flow control
						   Act. threhold (4kbytes)      */
	DmaRxFlowCtrlAct5K = 0x00800000,	/* (RFA)Rx flow control
						   Act. threhold (5kbytes)      */
	DmaRxFlowCtrlAct6K = 0x00800200,	/* (RFA)Rx flow control
						   Act. threhold (6kbytes)      */
	DmaRxFlowCtrlAct7K = 0x00800400,	/* (RFA)Rx flow control
						   Act. threhold (7kbytes)      */
	DmaRxThreshCtrl = 0x00000018,		/* (RTC)Controls thre
						   Threh of MTL rx Fifo         */
	DmaRxThreshCtrl64 = 0x00000000,		/* (RTC)Controls thre
						   Threh of MTL tx Fifo 64      */
	DmaRxThreshCtrl32 = 0x00000008,		/* (RTC)Controls thre
						   Threh of MTL tx Fifo 32      */
	DmaRxThreshCtrl96 = 0x00000010,		/* (RTC)Controls thre
						   Threh of MTL tx Fifo 96      */
	DmaRxThreshCtrl128 = 0x00000018,	/* (RTC)Controls thre
						   Threh of MTL tx Fifo 128     */
	DmaEnHwFlowCtrl = 0x00000100,		/* (EFC)Enable HW flow control  */
	DmaDisHwFlowCtrl = 0x00000000,		/* Disable HW flow control      */
	DmaFwdErrorFrames = 0x00000080,		/* (FEF)Forward error frames    */
	DmaFwdUnderSzFrames = 0x00000040,	/* (FUF)Forward undersize frames */
	DmaTxSecondFrame = 0x00000004,		/* (OSF)Operate on second frame */
	DmaRxStart = 0x00000002,		/* (SR)Start/Stop reception     */
};

/* DmaInterrupt		= 0x001C,	CSR7 - Interrupt enable Register Layout	*/
enum DmaInterruptReg {
	DmaIeNormal = DmaIntNormal,		/* Normal interrupt enable              */
	DmaIeAbnormal = DmaIntAbnormal,		/* Abnormal interrupt enable            */
	DmaIeEarlyRx = DmaIntEarlyRx,		/* Early receive interrupt enable       */
	DmaIeBusError = DmaIntBusError,		/* Fatal bus error enable               */
	DmaIeEarlyTx = DmaIntEarlyTx,		/* Early transmit interrupt enable      */
	DmaIeRxWdogTO = DmaIntRxWdogTO,		/* Receive Watchdog Timeout enable      */
	DmaIeRxStopped = DmaIntRxStopped,	/* Receive process stopped enable       */
	DmaIeRxNoBuffer = DmaIntRxNoBuffer,	/* Receive buffer unavailable enable    */
	DmaIeRxCompleted = DmaIntRxCompleted,	/* Completion of frame reception enable */
	DmaIeTxUnderflow = DmaIntTxUnderflow,	/* Transmit underflow enable            */
	DmaIeRxOverflow = DmaIntRcvOverflow,	/* Receive Buffer overflow interrupt    */
	DmaIeTxJabberTO = DmaIntTxJabberTO,	/* Transmit Jabber Timeout enable       */
	DmaIeTxNoBuffer = DmaIntTxNoBuffer,	/* Transmit buffer unavailable enable   */
	DmaIeTxStopped = DmaIntTxStopped,	/* Transmit process stopped enable      */
	DmaIeTxCompleted = DmaIntTxCompleted,	/* Transmit completed enable            */
};

/* DmaMissedFr	= 0x,0020	CSR8 - Missed Frame & Buffer overflow Counter	*/
enum DmaMissedFrReg {
	DmaMissFrmCntMask = 0x0000FFFF,
	DmaMissFrmCntShift = 0,
	DmaMissFrmCntOvf = 0x00010000,
	DmaOvfFrmCntMask = 0x0FFE0000,
	DmaOvfFrmCntShift = 17,
	DmaOvfFrmCntOvf = 0x10000000,
};


/* DmaAxiBusMod	= 0x,0028 */
enum DmaAxiBusModeReg {
	DmaEnLpi = 0x80000000,
	DmaLpiXitFrm = 0x40000000,
	DmaWrOsrNumReqs16 = 0x00F00000,
	DmaWrOsrNumReqs8 = 0x00700000,
	DmaWrOsrNumReqs4 = 0x00300000,
	DmaWrOsrNumReqs2 = 0x00100000,
	DmaWrOsrNumReqs1 = 0x00000000,
	DmaRdOsrNumReqs16 = 0x000F0000,
	DmaRdOsrNumReqs8 = 0x00070000,
	DmaRdOsrNumReqs4 = 0x00030000,
	DmaRdOsrNumReqs2 = 0x00010000,
	DmaRdOsrNumReqs1 = 0x00000000,
	DmaOnekbbe = 0x00002000,
	DmaAxiAal = 0x00001000,
	DmaAxiBlen256 = 0x00000080,
	DmaAxiBlen128 = 0x00000040,
	DmaAxiBlen64 = 0x00000020,
	DmaAxiBlen32 = 0x00000010,
	DmaAxiBlen16 = 0x00000008,
	DmaAxiBlen8 = 0x00000004,
	DmaAxiBlen4 = 0x00000002,
	DmaUndefined = 0x00000001,
};

/**********************************************************
 * DMA Engine descriptors
 **********************************************************/
/*
******Enhanced Descritpor structure to support 8K buffer per buffer *******

DmaRxBaseAddr	= 0x000C,	CSR3 - Receive Descriptor list base address
DmaRxBaseAddr is the pointer to the first Rx Descriptors.
The Descriptor format in Little endian with a 32 bit Data bus is as shown below.

Similarly
DmaTxBaseAddr     = 0x0010,  CSR4 - Transmit Descriptor list base address
DmaTxBaseAddr is the pointer to the first Rx Descriptors.
The Descriptor format in Little endian with a 32 bit Data bus is as shown below.
                  --------------------------------------------------------------------------
    RDES0	  |OWN (31)| Status                                                        |
		  --------------------------------------------------------------------------
    RDES1	  | Ctrl | Res | Byte Count Buffer 2 | Ctrl | Res | Byte Count Buffer 1    |
		  --------------------------------------------------------------------------
    RDES2	  |  Buffer 1 Address                                                      |
		  --------------------------------------------------------------------------
    RDES3	  |  Buffer 2 Address / Next Descriptor Address                            |
		  --------------------------------------------------------------------------

                  --------------------------------------------------------------------------
    TDES0	  |OWN (31)| Ctrl | Res | Ctrl | Res | Status                              |
		  --------------------------------------------------------------------------
    TDES1	  | Res | Byte Count Buffer 2 | Res |         Byte Count Buffer 1          |
		  --------------------------------------------------------------------------
    TDES2	  |  Buffer 1 Address                                                      |
		  --------------------------------------------------------------------------
    TDES3         |  Buffer 2 Address / Next Descriptor Address                            |
		  --------------------------------------------------------------------------

*/

/* status word of DMA descriptor */
enum DmaDescriptorStatus {
	DescOwnByDma = 0x80000000,		/* (OWN)Descriptor is
						   owned by DMA engine          */
	DescDAFilterFail = 0x40000000,		/* (AFM)Rx - DA Filter
						   Fail for the rx frame        */
	DescFrameLengthMask = 0x3FFF0000,	/* (FL)Receive descriptor
						   frame length                 */
	DescFrameLengthShift = 16,
	DescError = 0x00008000,			/* (ES)Error summary bit
						   - OR of the  following bits:
						   DE || OE || IPC || LC  || RWT
						   || RE || CE                  */
	DescRxTruncated = 0x00004000,		/* (DE)Rx - no more descriptors
						   for receive frame            */
	DescSAFilterFail = 0x00002000,		/* (SAF)Rx - SA Filter Fail for
						   the received frame           */
	DescRxLengthError = 0x00001000,		/* (LE)Rx - frm size not matching
						   with len field               */
	DescRxDamaged = 0x00000800,		/* (OE)Rx - frm was damaged due
						   to buffer overflow           */
	DescRxVLANTag = 0x00000400,		/* (VLAN)Rx - received frame
						   is a VLAN frame              */
	DescRxFirst = 0x00000200,		/* (FS)Rx - first
						   descriptor of the frame      */
	DescRxLast = 0x00000100,		/* (LS)Rx - last
						   descriptor of the frame      */
	DescRxLongFrame = 0x00000080,		/* (Giant Frame)Rx - frame is
						   longer than 1518/1522        */
	DescRxCollision = 0x00000040,		/* (LC)Rx - late collision occurred
						   during reception             */
	DescRxFrameEther = 0x00000020,		/* (FT)Rx - Frame type - Ethernet,
						   otherwise 802.3               */
	DescRxWatchdog = 0x00000010,		/* (RWT)Rx - watchdog timer expired
						   during reception             */
	DescRxMiiError = 0x00000008,		/* (RE)Rx - error reported
						   by MII interface             */
	DescRxDribbling = 0x00000004,		/* (DE)Rx - frame contains non int
						   multiple of 8 bits           */
	DescRxCrc = 0x00000002,			/* (CE)Rx - CRC error           */
	/*DescRxMacMatch        = 0x00000001, *//* (RX MAC Address) Rx mac address
						   reg(1 to 15)match   0        */
	DescRxEXTsts = 0x00000001,		/* Extended Status Available (RDES4) */
	DescTxIntEnable = 0x40000000,		/* (IC)Tx - interrupt on completion */
	DescTxLast = 0x20000000,		/* (LS)Tx - Last segment of the frame */
	DescTxFirst = 0x10000000,		/* (FS)Tx - First segment of the frame */
	DescTxDisableCrc = 0x08000000,		/* (DC)Tx - Add CRC disabled
						   (first segment only)         */
	DescTxDisablePadd = 0x04000000,		/* (DP)disable padding,
						   added by - reyaz             */
	DescTxCisMask = 0x00c00000,		/* Tx checksum offloading
						   control mask                 */
	DescTxCisBypass = 0x00000000,		/* Checksum bypass              */
	DescTxCisIpv4HdrCs = 0x00400000,	/* IPv4 header checksum         */
	DescTxCisTcpOnlyCs = 0x00800000,	/* TCP/UDP/ICMP checksum.
						   Pseudo header  checksum
						   is assumed to be present     */
	DescTxCisTcpPseudoCs = 0x00c00000,	/* TCP/UDP/ICMP checksum fully
						   in hardware  including
						   pseudo header                */
	TxDescEndOfRing = 0x00200000,		/* (TER)End of descriptors ring */
	TxDescChain = 0x00100000,		/* (TCH)Second buffer address
						   is chain address             */
	DescRxChkBit0 = 0x00000001,		/*()  Rx - Rx Payload Checksum Error */
	DescRxChkBit7 = 0x00000080,		/* (IPC CS ERROR)Rx - Ipv4
						   header checksum error        */
	DescRxChkBit5 = 0x00000020,		/* (FT)Rx - Frame type - Ethernet,
						   otherwise 802.3              */
	DescRxTSavail = 0x00000080,		/* Time stamp available         */
	DescRxFrameType = 0x00000020,		/* (FT)Rx - Frame type - Ethernet,
						   otherwise 802.3              */
	DescTxIpv4ChkError = 0x00010000,	/* (IHE) Tx Ip header error     */
	DescTxTimeout = 0x00004000,		/* (JT)Tx - Transmit
						   jabber timeout               */
	DescTxFrameFlushed = 0x00002000,	/* (FF)Tx - DMA/MTL flushed
						   the frame  due to SW flush   */
	DescTxPayChkError = 0x00001000,		/* (PCE) Tx Payload checksum Error */
	DescTxLostCarrier = 0x00000800,		/* (LC)Tx - carrier lost
						   during tramsmission          */
	DescTxNoCarrier = 0x00000400,		/* (NC)Tx - no carrier signal
						   from the tranceiver          */
	DescTxLateCollision = 0x00000200,	/* (LC)Tx - transmission aborted
						   due to collision             */
	DescTxExcCollisions = 0x00000100,	/* (EC)Tx - transmission aborted
						   after 16 collisions          */
	DescTxVLANFrame = 0x00000080,		/* (VF)Tx - VLAN-type frame     */
	DescTxCollMask = 0x00000078,		/* (CC)Tx - Collision count     */
	DescTxCollShift = 3,
	DescTxExcDeferral = 0x00000004,		/* (ED)Tx - excessive deferral  */
	DescTxUnderflow = 0x00000002,		/* (UF)Tx - late data arrival
						   from the memory              */
	DescTxDeferred = 0x00000001,		/* (DB)Tx - frame
						   transmision deferred         */

	/*
	 * This explains the RDES1/TDES1 bits layout
	 *                --------------------------------------------------------------------
	 *   RDES1/TDES1  | Control Bits | Byte Count Buffer 2 | Byte Count Buffer 1          |
	 *                --------------------------------------------------------------------
	 */

	/*DmaDescriptorLength *//* length word of DMA descriptor */
	RxDisIntCompl = 0x80000000,	/* (Disable Rx int on completion)       */
	RxDescEndOfRing = 0x00008000,	/* (TER)End of descriptors ring         */
	RxDescChain = 0x00004000,	/* (TCH)Second buffer address
					   is chain address                     */
	DescSize2Mask = 0x1FFF0000,	/* (TBS2) Buffer 2 size                 */
	DescSize2Shift = 16,
	DescSize1Mask = 0x00001FFF,	/* (TBS1) Buffer 1 size                 */
	DescSize1Shift = 0,

	/*
	 * This explains the RDES4 Extended Status bits layout
	 *                --------------------------------------------------------------------
	 *   RDES4        |                             Extended Status                        |
	 *                --------------------------------------------------------------------
	 */
	DescRxPtpAvail = 0x00004000,		/* PTP snapshot available       */
	DescRxPtpVer = 0x00002000,		/* When set indicates IEEE1584
						   Version 2 (else Ver1)        */
	DescRxPtpFrameType = 0x00001000,	/* PTP frame type Indicates PTP
						   sent over ethernet           */
	DescRxPtpMessageType = 0x00000F00,	/* Message Type                 */
	DescRxPtpNo = 0x00000000,		/* 0000 => No PTP message received */
	DescRxPtpSync = 0x00000100,		/* 0001 => Sync (all clock
						   types) received              */
	DescRxPtpFollowUp = 0x00000200,		/* 0010 => Follow_Up (all clock
						   types) received              */
	DescRxPtpDelayReq = 0x00000300,		/* 0011 => Delay_Req (all clock
						   types) received              */
	DescRxPtpDelayResp = 0x00000400,	/* 0100 => Delay_Resp (all clock
						   types) received              */
	DescRxPtpPdelayReq = 0x00000500,	/* 0101 => Pdelay_Req (in P
						   to P tras clk)  or Announce
						   in Ord and Bound clk         */
	DescRxPtpPdelayResp = 0x00000600,	/* 0110 => Pdealy_Resp(in P to
						   P trans clk) or Management in
						   Ord and Bound clk            */
	DescRxPtpPdelayRespFP = 0x00000700,	/* 0111 => Pdealy_Resp_Follow_Up
						   (in P to P trans clk) or Signaling
						   in Ord and Bound clk         */
	DescRxPtpIPV6 = 0x00000080,		/* Received Packet is  in IPV6 Packet */
	DescRxPtpIPV4 = 0x00000040,		/* Received Packet is  in IPV4 Packet */
	DescRxChkSumBypass = 0x00000020,	/* When set indicates checksum offload
						   engine is bypassed           */
	DescRxIpPayloadError = 0x00000010,	/* When set indicates 16bit IP
						   payload CS is in error       */
	DescRxIpHeaderError = 0x00000008,	/* When set indicates 16bit IPV4
						   header  CS is in error or IP
						   datagram version is  not
						   consistent with Ethernet type value  */
	DescRxIpPayloadType = 0x00000007,	/* Indicate the type of payload
						   encapsulated in IPdatagram
						   processed by COE (Rx)        */
	DescRxIpPayloadUnknown = 0x00000000,	/* Unknown or didnot process
						   IP payload                   */
	DescRxIpPayloadUDP = 0x00000001,	/* UDP                          */
	DescRxIpPayloadTCP = 0x00000002,	/* TCP                          */
	DescRxIpPayloadICMP = 0x00000003,	/* ICMP                         */
};

/**********************************************************
 * Initial register values
 **********************************************************/
enum InitialRegisters {
	/* Full-duplex mode with perfect filter on */
	GmacConfigInitFdx1000 = GmacWatchdogEnable | GmacJabberEnable
	    | GmacFrameBurstEnable | GmacJumboFrameDisable
	    | GmacSelectGmii | GmacEnableRxOwn
	    | GmacLoopbackOff | GmacFullDuplex | GmacRetryEnable
	    | GmacPadCrcStripDisable | GmacBackoffLimit0
	    | GmacDeferralCheckDisable | GmacTxEnable | GmacRxEnable,

	/* Full-duplex mode with perfect filter on */
	GmacConfigInitFdx110 = GmacWatchdogEnable | GmacJabberEnable
	    | GmacFrameBurstEnable
	    | GmacJumboFrameDisable | GmacSelectMii | GmacEnableRxOwn
	    | GmacLoopbackOff | GmacFullDuplex | GmacRetryEnable
	    | GmacPadCrcStripDisable | GmacBackoffLimit0
	    | GmacDeferralCheckDisable | GmacTxEnable | GmacRxEnable,

	/* Full-duplex mode */
	/*      CHANGED: Pass control config, dest addr filter normal,
	   added source address filter, multicast & unicast
	 */

	/* Hash filter. */
	/* = GmacFilterOff | GmacPassControlOff | GmacBroadcastEnable */
	GmacFrameFilterInitFdx =
	    GmacFilterOn | GmacPassControl0 | GmacBroadcastEnable |
	    GmacSrcAddrFilterDisable | GmacMulticastFilterOn |
	    GmacDestAddrFilterNor | GmacMcastHashFilterOff |
	    GmacPromiscuousModeOff | GmacUcastHashFilterOff,

	/* Full-duplex mode */
	GmacFlowControlInitFdx =
	    GmacUnicastPauseFrameOff | GmacRxFlowControlEnable |
	    GmacTxFlowControlEnable,

	/* Full-duplex mode */
	GmacGmiiAddrInitFdx = GmiiCsrClk2,

	/* Half-duplex mode with perfect filter on */
	/* CHANGED: Removed Endian configuration, added single bit
	 *config for PAD/CRC strip,
	 */

	GmacConfigInitHdx1000 = GmacWatchdogEnable | GmacJabberEnable
	    | GmacFrameBurstEnable | GmacJumboFrameDisable
	    | GmacSelectGmii | GmacDisableRxOwn
	    | GmacLoopbackOff | GmacHalfDuplex | GmacRetryEnable
	    | GmacPadCrcStripDisable | GmacBackoffLimit0
	    | GmacDeferralCheckDisable | GmacTxEnable | GmacRxEnable,

	/* Half-duplex mode with perfect filter on */
	GmacConfigInitHdx110 = GmacWatchdogEnable | GmacJabberEnable
	    | GmacFrameBurstEnable | GmacJumboFrameDisable
	    | GmacSelectMii | GmacDisableRxOwn | GmacLoopbackOff
	    | GmacHalfDuplex | GmacRetryEnable
	    | GmacPadCrcStripDisable | GmacBackoffLimit0
	    | GmacDeferralCheckDisable | GmacTxEnable | GmacRxEnable,

	/* Half-duplex mode */
	GmacFrameFilterInitHdx = GmacFilterOn | GmacPassControl0
	    | GmacBroadcastEnable | GmacSrcAddrFilterDisable
	    | GmacMulticastFilterOn | GmacDestAddrFilterNor
	    | GmacMcastHashFilterOff | GmacUcastHashFilterOff
	    | GmacPromiscuousModeOff,

	/* Half-duplex mode */
	GmacFlowControlInitHdx = GmacUnicastPauseFrameOff
	    | GmacRxFlowControlDisable | GmacTxFlowControlDisable,

	/* Half-duplex mode */
	GmacGmiiAddrInitHdx = GmiiCsrClk2,

/*********************************************
* DMA configurations
**********************************************/

	DmaBusModeInit = DmaFixedBurstEnable | DmaBurstLength8
	    | DmaDescriptorSkip2 | DmaResetOff,

	DmaBusModeVal = DmaBurstLength32
	    | DmaBurstLengthx8 | DmaDescriptorSkip0
	    | DmaDescriptor8Words | DmaArbitPr | DmaAddressAlignedBeats,

	/* 1000 Mb/s mode */
	DmaControlInit1000 = DmaStoreAndForward,

	/* 100 Mb/s mode */
	DmaControlInit100 = DmaStoreAndForward,

	/* 10 Mb/s mode */
	DmaControlInit10 = DmaStoreAndForward,

	DmaOMR = DmaStoreAndForward | DmaRxStoreAndForward
	    | DmaRxThreshCtrl128 | DmaTxSecondFrame,

	/* Interrupt groups */
	DmaIntErrorMask = DmaIntBusError,	/* Error                        */
	DmaIntRxAbnMask = DmaIntRxNoBuffer,	/* receiver abnormal interrupt  */
	DmaIntRxNormMask = DmaIntRxCompleted,	/* receiver normal interrupt    */
	DmaIntRxStoppedMask = DmaIntRxStopped,	/* receiver stopped             */
	DmaIntTxAbnMask = DmaIntTxUnderflow,	/* transmitter abnormal interrupt */
	DmaIntTxNormMask = DmaIntTxCompleted,	/* transmitter normal interrupt */
	DmaIntTxStoppedMask = DmaIntTxStopped,	/* transmitter stopped          */

	DmaIntEnable = DmaIeNormal | DmaIeAbnormal | DmaIntErrorMask
	    | DmaIntRxAbnMask | DmaIntRxNormMask
	    | DmaIntRxStoppedMask | DmaIntTxAbnMask
	    | DmaIntTxNormMask | DmaIntTxStoppedMask,
	DmaIntDisable = 0,
	DmaAxiBusModeVal = DmaAxiBlen16 | DmaRdOsrNumReqs8 | DmaWrOsrNumReqs8,
};

/**********************************************************
 * Mac Management Counters (MMC)
 **********************************************************/
enum mmc_enable {
	GmacMmcCntrl = 0x0100,		/* mmc control for operating
					   mode of MMC                  */
	GmacMmcIntrRx = 0x0104,		/* maintains interrupts
					   generated by rx counters     */
	GmacMmcIntrTx = 0x0108,		/* maintains interrupts
					   generated by tx counters     */
	GmacMmcIntrMaskRx = 0x010C,	/* mask for interrupts
					   generated from rx counters   */
	GmacMmcIntrMaskTx = 0x0110,	/* mask for interrupts
					   generated from tx counters   */
};

enum mmc_tx {
	GmacMmcTxOctetCountGb = 0x0114,	/*Bytes Tx excl. of preamble and retried bytes     (Good or Bad)                        */
	GmacMmcTxFrameCountGb = 0x0118,	/*Frames Tx excl. of retried frames                (Good or Bad)                        */
	GmacMmcTxBcFramesG = 0x011C,	/*Broadcast Frames Tx                              (Good)                               */
	GmacMmcTxMcFramesG = 0x0120,	/*Multicast Frames Tx                              (Good)                               */

	GmacMmcTx64OctetsGb = 0x0124,	/*Tx with len 64 bytes excl. of pre and retried    (Good or Bad)                        */
	GmacMmcTx65To127OctetsGb = 0x0128,	/*Tx with len >64 bytes <=127 excl. of pre and retried    (Good or Bad)                 */
	GmacMmcTx128To255OctetsGb = 0x012C,	/*Tx with len >128 bytes <=255 excl. of pre and retried   (Good or Bad)                 */
	GmacMmcTx256To511OctetsGb = 0x0130,	/*Tx with len >256 bytes <=511 excl. of pre and retried   (Good or Bad)                 */
	GmacMmcTx512To1023OctetsGb = 0x0134,	/*Tx with len >512 bytes <=1023 excl. of pre and retried  (Good or Bad)                 */
	GmacMmcTx1024ToMaxOctetsGb = 0x0138,	/*Tx with len >1024 bytes <=MaxSize excl. of pre and retried (Good or Bad)              */

	GmacMmcTxUcFramesGb = 0x013C,	/*Unicast Frames Tx                                      (Good or Bad)                  */
	GmacMmcTxMcFramesGb = 0x0140,	/*Multicast Frames Tx                              (Good and Bad)                       */
	GmacMmcTxBcFramesGb = 0x0144,	/*Broadcast Frames Tx                              (Good and Bad)                       */
	GmacMmcTxUnderFlowError = 0x0148,	/*Frames aborted due to Underflow error                                                 */
	GmacMmcTxSingleColG = 0x014C,	/*Successfully Tx Frames after singel collision in Half duplex mode                     */
	GmacMmcTxMultiColG = 0x0150,	/*Successfully Tx Frames after more than singel collision in Half duplex mode           */
	GmacMmcTxDeferred = 0x0154,	/*Successfully Tx Frames after a deferral in Half duplex mode                           */
	GmacMmcTxLateCol = 0x0158,	/*Frames aborted due to late collision error                                            */
	GmacMmcTxExessCol = 0x015C,	/*Frames aborted due to excessive (16) collision errors                                 */
	GmacMmcTxCarrierError = 0x0160,	/*Frames aborted due to carrier sense error (No carrier or Loss of carrier)             */
	GmacMmcTxOctetCountG = 0x0164,	/*Bytes Tx excl. of preamble and retried bytes     (Good)                               */
	GmacMmcTxFrameCountG = 0x0168,	/*Frames Tx                                        (Good)                               */
	GmacMmcTxExessDef = 0x016C,	/*Frames aborted due to excessive deferral errors (deferred for more than 2 max-sized frame times) */

	GmacMmcTxPauseFrames = 0x0170,	/*Number of good pause frames Tx.                                                       */
	GmacMmcTxVlanFramesG = 0x0174,	/*Number of good Vlan frames Tx excl. retried frames                                    */
};
enum mmc_rx {
	GmacMmcRxFrameCountGb = 0x0180,	/*Frames Rx                                        (Good or Bad)                        */
	GmacMmcRxOctetCountGb = 0x0184,	/*Bytes Rx excl. of preamble and retried bytes     (Good or Bad)                        */
	GmacMmcRxOctetCountG = 0x0188,	/*Bytes Rx excl. of preamble and retried bytes     (Good)                               */
	GmacMmcRxBcFramesG = 0x018C,	/*Broadcast Frames Rx                              (Good)                               */
	GmacMmcRxMcFramesG = 0x0190,	/*Multicast Frames Rx                              (Good)                               */

	GmacMmcRxCrcError = 0x0194,	/*Number of frames received with CRC error                                              */
	GmacMmcRxAlignError = 0x0198,	/*Number of frames received with alignment (dribble) error. Only in 10/100mode          */
	GmacMmcRxRuntError = 0x019C,	/*Number of frames received with runt (<64 bytes and CRC error) error                   */
	GmacMmcRxJabberError = 0x01A0,	/*Number of frames rx with jabber (>1518/1522 or >9018/9022 and CRC)                    */
	GmacMmcRxUnderSizeG = 0x01A4,	/*Number of frames received with <64 bytes without any error                            */
	GmacMmcRxOverSizeG = 0x01A8,	/*Number of frames received with >1518/1522 bytes without any error                     */

	GmacMmcRx64OctetsGb = 0x01AC,	/*Rx with len 64 bytes excl. of pre and retried    (Good or Bad)                        */
	GmacMmcRx65To127OctetsGb = 0x01B0,	/*Rx with len >64 bytes <=127 excl. of pre and retried    (Good or Bad)                 */
	GmacMmcRx128To255OctetsGb = 0x01B4,	/*Rx with len >128 bytes <=255 excl. of pre and retried   (Good or Bad)                 */
	GmacMmcRx256To511OctetsGb = 0x01B8,	/*Rx with len >256 bytes <=511 excl. of pre and retried   (Good or Bad)                 */
	GmacMmcRx512To1023OctetsGb = 0x01BC,	/*Rx with len >512 bytes <=1023 excl. of pre and retried  (Good or Bad)                 */
	GmacMmcRx1024ToMaxOctetsGb = 0x01C0,	/*Rx with len >1024 bytes <=MaxSize excl. of pre and retried (Good or Bad)              */

	GmacMmcRxUcFramesG = 0x01C4,	/*Unicast Frames Rx                                      (Good)                         */
	GmacMmcRxLengthError = 0x01C8,	/*Number of frames received with Length type field != frame size                        */
	GmacMmcRxOutOfRangeType = 0x01CC,	/*Number of frames received with length field != valid frame size                       */

	GmacMmcRxPauseFrames = 0x01D0,	/*Number of good pause frames Rx.                                                       */
	GmacMmcRxFifoOverFlow = 0x01D4,	/*Number of missed rx frames due to FIFO overflow                                       */
	GmacMmcRxVlanFramesGb = 0x01D8,	/*Number of good Vlan frames Rx                                                         */

	GmacMmcRxWatchdobError = 0x01DC,	/*Number of frames rx with error due to watchdog timeout error                          */
};
enum mmc_ip_related {
	GmacMmcRxIpcIntrMask = 0x0200,	/*Maintains the mask for interrupt generated from rx IPC statistic counters             */
	GmacMmcRxIpcIntr = 0x0208,	/*Maintains the interrupt that rx IPC statistic counters generate                       */

	GmacMmcRxIpV4FramesG = 0x0210,	/*Good IPV4 datagrams received                                                          */
	GmacMmcRxIpV4HdrErrFrames = 0x0214,	/*Number of IPV4 datagrams received with header errors                                  */
	GmacMmcRxIpV4NoPayFrames = 0x0218,	/*Number of IPV4 datagrams received which didnot have TCP/UDP/ICMP payload              */
	GmacMmcRxIpV4FragFrames = 0x021C,	/*Number of IPV4 datagrams received with fragmentation                                  */
	GmacMmcRxIpV4UdpChkDsblFrames = 0x0220,	/*Number of IPV4 datagrams received that had a UDP payload checksum disabled            */

	GmacMmcRxIpV6FramesG = 0x0224,	/*Good IPV6 datagrams received                                                          */
	GmacMmcRxIpV6HdrErrFrames = 0x0228,	/*Number of IPV6 datagrams received with header errors                                  */
	GmacMmcRxIpV6NoPayFrames = 0x022C,	/*Number of IPV6 datagrams received which didnot have TCP/UDP/ICMP payload              */

	GmacMmcRxUdpFramesG = 0x0230,	/*Number of good IP datagrams with good UDP payload                                     */
	GmacMmcRxUdpErrorFrames = 0x0234,	/*Number of good IP datagrams with UDP payload having checksum error                    */

	GmacMmcRxTcpFramesG = 0x0238,	/*Number of good IP datagrams with good TDP payload                                     */
	GmacMmcRxTcpErrorFrames = 0x023C,	/*Number of good IP datagrams with TCP payload having checksum error                    */

	GmacMmcRxIcmpFramesG = 0x0240,	/*Number of good IP datagrams with good Icmp payload                                    */
	GmacMmcRxIcmpErrorFrames = 0x0244,	/*Number of good IP datagrams with Icmp payload having checksum error                   */

	GmacMmcRxIpV4OctetsG = 0x0250,	/*Good IPV4 datagrams received excl. Ethernet hdr,FCS,Pad,Ip Pad bytes                  */
	GmacMmcRxIpV4HdrErrorOctets = 0x0254,	/*Number of bytes in IPV4 datagram with header errors                                   */
	GmacMmcRxIpV4NoPayOctets = 0x0258,	/*Number of bytes in IPV4 datagram with no TCP/UDP/ICMP payload                         */
	GmacMmcRxIpV4FragOctets = 0x025C,	/*Number of bytes received in fragmented IPV4 datagrams                                 */
	GmacMmcRxIpV4UdpChkDsblOctets = 0x0260,	/*Number of bytes received in UDP segment that had UDP checksum disabled                */

	GmacMmcRxIpV6OctetsG = 0x0264,	/*Good IPV6 datagrams received excl. Ethernet hdr,FCS,Pad,Ip Pad bytes                  */
	GmacMmcRxIpV6HdrErrorOctets = 0x0268,	/*Number of bytes in IPV6 datagram with header errors                                   */
	GmacMmcRxIpV6NoPayOctets = 0x026C,	/*Number of bytes in IPV6 datagram with no TCP/UDP/ICMP payload                         */

	GmacMmcRxUdpOctetsG = 0x0270,	/*Number of bytes in IP datagrams with good UDP payload                                 */
	GmacMmcRxUdpErrorOctets = 0x0274,	/*Number of bytes in IP datagrams with UDP payload having checksum error                */

	GmacMmcRxTcpOctetsG = 0x0278,	/*Number of bytes in IP datagrams with good TDP payload                                 */
	GmacMmcRxTcpErrorOctets = 0x027C,	/*Number of bytes in IP datagrams with TCP payload having checksum error                */

	GmacMmcRxIcmpOctetsG = 0x0280,	/*Number of bytes in IP datagrams with good Icmp payload                                */
	GmacMmcRxIcmpErrorOctets = 0x0284,	/*Number of bytes in IP datagrams with Icmp payload having checksum error               */
};

enum MMC_CNTRL_REG_BIT_DESCRIPTIONS {
	GmacMmcCounterFreeze = 0x00000008,	/* when set MMC counters freeze to current value                                */
	GmacMmcCounterResetOnRead = 0x00000004,	/* when set MMC counters will be reset to 0 after read                          */
	GmacMmcCounterStopRollover = 0x00000002,	/* when set counters will not rollover after max value                          */
	GmacMmcCounterReset = 0x00000001,	/* when set all counters wil be reset (automatically cleared after 1 clk)       */
};

enum mmc_rx_intr_mask_and_status_bit_descriptions {
	GmacMmcRxWDInt = 0x00800000,	/* set when rxwatchdog error reaches half of max value                          */
	GmacMmcRxVlanInt = 0x00400000,	/* set when GmacMmcRxVlanFramesGb counter reaches half of max value             */
	GmacMmcRxFifoOverFlowInt = 0x00200000,	/* set when GmacMmcRxFifoOverFlow counter reaches half of max value             */
	GmacMmcRxPauseFrameInt = 0x00100000,	/* set when GmacMmcRxPauseFrames counter reaches half of max value              */
	GmacMmcRxOutOfRangeInt = 0x00080000,	/* set when GmacMmcRxOutOfRangeType counter reaches half of max value           */
	GmacMmcRxLengthErrorInt = 0x00040000,	/* set when GmacMmcRxLengthError counter reaches half of max value              */
	GmacMmcRxUcFramesInt = 0x00020000,	/* set when GmacMmcRxUcFramesG counter reaches half of max value                */
	GmacMmcRx1024OctInt = 0x00010000,	/* set when GmacMmcRx1024ToMaxOctetsGb counter reaches half of max value        */
	GmacMmcRx512OctInt = 0x00008000,	/* set when GmacMmcRx512To1023OctetsGb counter reaches half of max value        */
	GmacMmcRx256OctInt = 0x00004000,	/* set when GmacMmcRx256To511OctetsGb counter reaches half of max value         */
	GmacMmcRx128OctInt = 0x00002000,	/* set when GmacMmcRx128To255OctetsGb counter reaches half of max value         */
	GmacMmcRx65OctInt = 0x00001000,	/* set when GmacMmcRx65To127OctetsG counter reaches half of max value           */
	GmacMmcRx64OctInt = 0x00000800,	/* set when GmacMmcRx64OctetsGb counter reaches half of max value               */
	GmacMmcRxOverSizeInt = 0x00000400,	/* set when GmacMmcRxOverSizeG counter reaches half of max value                */
	GmacMmcRxUnderSizeInt = 0x00000200,	/* set when GmacMmcRxUnderSizeG counter reaches half of max value               */
	GmacMmcRxJabberErrorInt = 0x00000100,	/* set when GmacMmcRxJabberError counter reaches half of max value              */
	GmacMmcRxRuntErrorInt = 0x00000080,	/* set when GmacMmcRxRuntError counter reaches half of max value                */
	GmacMmcRxAlignErrorInt = 0x00000040,	/* set when GmacMmcRxAlignError counter reaches half of max value               */
	GmacMmcRxCrcErrorInt = 0x00000020,	/* set when GmacMmcRxCrcError counter reaches half of max value                 */
	GmacMmcRxMcFramesInt = 0x00000010,	/* set when GmacMmcRxMcFramesG counter reaches half of max value                */
	GmacMmcRxBcFramesInt = 0x00000008,	/* set when GmacMmcRxBcFramesG counter reaches half of max value                */
	GmacMmcRxOctetGInt = 0x00000004,	/* set when GmacMmcRxOctetCountG counter reaches half of max value              */
	GmacMmcRxOctetGbInt = 0x00000002,	/* set when GmacMmcRxOctetCountGb counter reaches half of max value             */
	GmacMmcRxFrameInt = 0x00000001,	/* set when GmacMmcRxFrameCountGb counter reaches half of max value             */
};

enum mmc_tx_intr_mask_and_status_bit_descriptions {

	GmacMmcTxVlanInt = 0x01000000,	/* set when GmacMmcTxVlanFramesG counter reaches half of max value              */
	GmacMmcTxPauseFrameInt = 0x00800000,	/* set when GmacMmcTxPauseFrames counter reaches half of max value              */
	GmacMmcTxExessDefInt = 0x00400000,	/* set when GmacMmcTxExessDef counter reaches half of max value                 */
	GmacMmcTxFrameInt = 0x00200000,	/* set when GmacMmcTxFrameCount counter reaches half of max value               */
	GmacMmcTxOctetInt = 0x00100000,	/* set when GmacMmcTxOctetCountG counter reaches half of max value              */
	GmacMmcTxCarrierErrorInt = 0x00080000,	/* set when GmacMmcTxCarrierError counter reaches half of max value             */
	GmacMmcTxExessColInt = 0x00040000,	/* set when GmacMmcTxExessCol counter reaches half of max value                 */
	GmacMmcTxLateColInt = 0x00020000,	/* set when GmacMmcTxLateCol counter reaches half of max value                  */
	GmacMmcTxDeferredInt = 0x00010000,	/* set when GmacMmcTxDeferred counter reaches half of max value                 */
	GmacMmcTxMultiColInt = 0x00008000,	/* set when GmacMmcTxMultiColG counter reaches half of max value                */
	GmacMmcTxSingleCol = 0x00004000,	/* set when GmacMmcTxSingleColG counter reaches half of max value               */
	GmacMmcTxUnderFlowErrorInt = 0x00002000,	/* set when GmacMmcTxUnderFlowError counter reaches half of max value           */
	GmacMmcTxBcFramesGbInt = 0x00001000,	/* set when GmacMmcTxBcFramesGb counter reaches half of max value               */
	GmacMmcTxMcFramesGbInt = 0x00000800,	/* set when GmacMmcTxMcFramesGb counter reaches half of max value               */
	GmacMmcTxUcFramesInt = 0x00000400,	/* set when GmacMmcTxUcFramesGb counter reaches half of max value               */
	GmacMmcTx1024OctInt = 0x00000200,	/* set when GmacMmcTx1024ToMaxOctetsGb counter reaches half of max value        */
	GmacMmcTx512OctInt = 0x00000100,	/* set when GmacMmcTx512To1023OctetsGb counter reaches half of max value        */
	GmacMmcTx256OctInt = 0x00000080,	/* set when GmacMmcTx256To511OctetsGb counter reaches half of max value         */
	GmacMmcTx128OctInt = 0x00000040,	/* set when GmacMmcTx128To255OctetsGb counter reaches half of max value         */
	GmacMmcTx65OctInt = 0x00000020,	/* set when GmacMmcTx65To127OctetsGb counter reaches half of max value          */
	GmacMmcTx64OctInt = 0x00000010,	/* set when GmacMmcTx64OctetsGb counter reaches half of max value               */
	GmacMmcTxMcFramesInt = 0x00000008,	/* set when GmacMmcTxMcFramesG counter reaches half of max value                */
	GmacMmcTxBcFramesInt = 0x00000004,	/* set when GmacMmcTxBcFramesG counter reaches half of max value                */
	GmacMmcTxFrameGbInt = 0x00000002,	/* set when GmacMmcTxFrameCountGb counter reaches half of max value             */
	GmacMmcTxOctetGbInt = 0x00000001,	/* set when GmacMmcTxOctetCountGb counter reaches half of max value             */
};

/**********************************************************
 * Power Management (PMT) Block
 **********************************************************/

/*
 * PMT supports the reception of network (remote) wake-up frames and Magic packet frames.
 * It generates interrupts for wake-up frames and Magic packets received by GMAC.
 * PMT sits in Rx path and is enabled with remote wake-up frame enable and Magic packet enable.
 * These enable are in PMT control and Status register and are programmed by apllication.
 *
 * When power down mode is enabled in PMT, all rx frames are dropped by the core. Core comes
 * out of power down mode only when either Magic packe tor a Remote wake-up frame is received
 * and the corresponding detection is enabled.
 *
 * Driver need not be modified to support this feature. Only Api to put the device in to power
 * down mode is sufficient
  */

#define WAKEUP_REG_LENGTH		8	/*This is the reg length for wake up register configuration */

enum GmacPmtCtrlStatusBitDefinition {
	GmacPmtFrmFilterPtrReset = 0x80000000,	/* when set remote wake-up frame filter register pointer to 3'b000 */
	GmacPmtGlobalUnicast = 0x00000200,	/* When set enables any unicast packet to be a wake-up frame       */
	GmacPmtWakeupFrameReceived = 0x00000040,	/* Wake up frame received                                          */
	GmacPmtMagicPktReceived = 0x00000020,	/* Magic Packet received                                           */
	GmacPmtWakeupFrameEnable = 0x00000004,	/* Wake-up frame enable                                            */
	GmacPmtMagicPktEnable = 0x00000002,	/* Magic packet enable                                             */
	GmacPmtPowerDown = 0x00000001,	/* Power Down                                                              */
};

/**********************************************************
 * IEEE 1588-2008 Precision Time Protocol (PTP) Support
 **********************************************************/
enum PTPMessageType {
	SYNC = 0x0,
	Delay_Req = 0x1,
	Pdelay_Req = 0x2,
	Pdelay_Resp = 0x3,
	Follow_up = 0x8,
	Delay_Resp = 0x9,
	Pdelay_Resp_Follow_Up = 0xA,
	Announce = 0xB,
	Signaling = 0xC,
	Management = 0xD,
};

typedef struct TimeStampStruct {
	uint32_t TSversion;	/* PTP Version 1 or PTP version2               */
	uint32_t TSmessagetype;	/* Message type associated with this
				   time stamp                                  */
	uint16_t TShighest16;	/* Highest 16 bit time stamp value, Valid
				   onley when ADV_TIME_HIGH_WORD
				   configured in corekit                       */
	uint32_t TSupper32;	/* Most significant 32 bit time stamp value    */
	uint32_t TSlower32;	/* Least Significat 32 bit time stamp value    */
} TimeStamp;

enum GmacMiiStatusReg {
	GmacMiiLinkSpeed_1000 = 0x00000004,
	GmacMiiLinkSpeed_100 = 0x00000002,
	GmacMiiLinkSpeed_10 = 0x00000000,
	GmacMiiLinkSpeedMask = 0x00000003,
	GmacMiiModeFull = 0x00000001,
	GmacMiiModeMask = 0x00000001,
};

/*
 * IEEE 1588-2008 is the optional module to support Ethernet frame time stamping.
 * Sixty four (+16) bit time stamps are given in each frames transmit and receive status.
 * The driver assumes the following
 *  1. "IEEE 1588 Time Stamping" "TIME_STAMPING"is ENABLED in corekit
 *  2. "IEEE 1588 External Time Stamp Input Enable" "EXT_TIME_STAMPING" is DISABLED in corekit
 *  3. "IEEE 1588 Advanced Time Stamp support" "ADV_TIME_STAMPING" is ENABLED in corekit
 *  4. "IEEE 1588 Higher Word Register Enable" "ADV_TIME_HIGH_WORD" is ENABLED in corekit
 */

/*	GmacTSControl  = 0x0700,	Controls the Timestamp update logic  : only when IEEE 1588 time stamping is enabled in corekit	*/
enum GmacTSControlReg {
	GmacTSENMACADDR = 0x00040000,	/* Enable Mac Addr for PTP filtering    */
	GmacTSCLKTYPE = 0x00030000,	/* Select the type of clock node        */
/*
 *   TSCLKTYPE        TSMSTRENA      TSEVNTENA   Messages for wihich TS snapshot is taken
 *    00/01                X             0        SYNC, FOLLOW_UP, DELAY_REQ, DELAY_RESP
 *    00/01                1             0        DELAY_REQ
 *    00/01                0             1        SYNC
 *     10                  NA            0        SYNC, FOLLOW_UP, DELAY_REQ, DELAY_RESP
 *     10                  NA            1        SYNC, FOLLOW_UP
 *     11                  NA            0        SYNC, FOLLOW_UP, DELAY_REQ, DELAY_RESP,
 *						  PDELAY_REQ, PDELAY_RESP
 *     11                  NA            1        SYNC, PDELAY_REQ, PDELAY_RESP
 */

	GmacTSOrdClk = 0x00000000,	/* 00=> Ordinary clock          */
	GmacTSBouClk = 0x00010000,	/* 01=> Boundary clock          */
	GmacTSEtoEClk = 0x00020000,	/* 10=> End-to-End
					   transparent clock            */
	GmacTSPtoPClk = 0x00030000,	/* 11=> P-to-P transparent clock */
	GmacTSMSTRENA = 0x00008000,	/* Ena TS Snapshot for
					   Master Messages              */
	GmacTSEVNTENA = 0x00004000,	/* Ena TS Snapshot for
					   Event Messages               */
	GmacTSIPV4ENA = 0x00002000,	/* Ena TS snapshot for IPv4     */
	GmacTSIPV6ENA = 0x00001000,	/* Ena TS snapshot for IPv6     */
	GmacTSIPENA = 0x00000800,	/* Ena TS snapshot for
					   PTP over E'net               */
	GmacTSVER2ENA = 0x00000400,	/* Ena PTP snooping
					   for version 2                */
	GmacTSCTRLSSR = 0x00000200,	/* Digital or Binary Rollover   */
	GmacTSENALL = 0x00000100,	/* Enable TS fro all
					   frames (Ver2 only)           */
	GmacTSADDREG = 0x00000020,	/* Addend Register Update       */
	GmacTSUPDT = 0x00000008,	/* Time Stamp Update            */
	GmacTSINT = 0x00000004,		/* Time Atamp Initialize        */
	GmacTSTRIG = 0x00000010,	/* Time stamp interrupt
					   Trigger Enable               */
	GmacTSCFUPDT = 0x00000002,	/* Time Stamp Fine/Coarse       */
	GmacTSCUPDTCoarse = 0x00000000,	/* 0=> Time Stamp update method
					   is coarse                    */
	GmacTSCUPDTFine = 0x00000002,	/* 1=> Time Stamp update method
					   is fine                      */
	GmacTSENA = 0x00000001,		/* Time Stamp Enable            */
};

/* GmacTSSubSecIncr	= 0x0704,	8 bit value by which sub second register is incremented:
 * 					only when IEEE 1588 time stamping without
 * 					external timestamp input.
 */
enum GmacTSSubSecIncrReg {
	GmacSSINCMsk = 0x000000FF,	/* Only Lower 8 bits are valid bits */
};

/* GmacTSLow	= 0x070C,	Indicates whether the timestamp low count is positive or negative;
 *				for Adv timestamp it is always zero.
 */
enum GmacTSSign {
	GmacTSSign = 0x80000000,
	GmacTSPositive = 0x00000000,
	GmacTSNegative = 0x80000000,
};

/* GmacTargetTimeLow	= 0x0718,	32 bit nano seconds(MS) to be compared with
 * 					system time: only when IEEE 1588 time
 * 					stamping without external timestamp input.
 */
enum GmacTSLowReg {
	GmacTSDecThr = 0x3B9AC9FF,	/* when TSCTRLSSR is set the max
					   value for GmacTargetTimeLowReg
					   and GmacTimeStampLow register
					   is 0x3B9AC9FF at 1ns precision */
};

/* GmacTSHighWord	= 0x0724,	Time Stamp Higher Word Register (Version 2 only);
 *					only lower 16 bits are valid
 */
enum GmacTSHighWordReg {
	GmacTSHighWordMask = 0x0000FFFF,
};

/* GmacTSStatus		= 0x0728,	Time Stamp Status Register */
enum GmacTSStatusReg {
	GmacTSTargTimeReached = 0x00000002,
	GmacTSSecondsOverflow = 0x00000001,
};

/*
 * Debugging APIs to print NSS GMAC driver messages.
 *
 * @param[in] Pointer to device private structure or NULL.
 * @param[in] Linux Kernel Loglevel.
 * @param[in] Pointer to message format string.
 * @param[in] variable number of arguments for format string.
 *
 */
#define nss_gmac_msg(msg, args...)							\
			do{								\
				printk("nss_gmac: " msg "\n", ##args);			\
			}while(0)

#if (NSS_GMAC_DEBUG_LEVEL >= 1)
#define nss_gmac_warn(gmacdev, msg, args...)						\
			do{								\
				printk("nss_gmac: GMAC%d(%p)" msg "\n", 		\
					gmacdev->macid, gmacdev, ##args);		\
			}while(0)

#define nss_gmac_early_dbg(msg, args...)						\
			do{								\
				printk("nss_gmac: " msg "\n", ##args);			\
			}while(0)
#else
#define nss_gmac_warn(gmacdev,  msg, args...)	do{	}while(0)
#define nss_gmac_early_dbg(msg, args...)	do{	}while(0)
#endif

#if (NSS_GMAC_DEBUG_LEVEL >= 2)
#define nss_gmac_info(gmacdev, msg, args...)						\
			do{								\
				printk("nss_gmac: GMAC%d(%p)" msg "\n", 		\
					gmacdev->macid, gmacdev, ##args);		\
			}while(0)
#else
#define nss_gmac_info(gmacdev,  msg, args...)	do{	}while(0)
#endif

#if (NSS_GMAC_DEBUG_LEVEL >= 3)
#define nss_gmac_trace(gmacdev, msg, args...)						\
			do{								\
				printk("nss_gmac: GMAC%d(%p)" msg "\n", 		\
					gmacdev->macid, gmacdev, ##args);		\
			}while(0)
#else
#define nss_gmac_trace(gmacdev,  msg, args...)	do{	}while(0)
#endif

/**********************************************************
 * Time stamp related functions
 **********************************************************/
void nss_gmac_ts_enable(nss_gmac_dev *gmacdev);
void nss_gmac_ts_disable(nss_gmac_dev *gmacdev);

void nss_gmac_ts_int_enable(nss_gmac_dev *gmacdev);
void nss_gmac_ts_int_disable(nss_gmac_dev *gmacdev);

void nss_gmac_ts_mac_addr_filt_enable(nss_gmac_dev *gmacdev);
void nss_gmac_ts_mac_addr_filt_disable(nss_gmac_dev *gmacdev);
void nss_gmac_ts_set_clk_type(nss_gmac_dev *gmacdev, uint32_t clk_type);

/* Only for Ordinary clock and Boundary clock and "Advanced Time Stamp" */
void nss_gmac_ts_master_enable(nss_gmac_dev *gmacdev);

/* Only for Ordinary clock and Boundary clock and "Advanced Time Stamp" */
void nss_gmac_ts_master_disable(nss_gmac_dev *gmacdev);

void nss_gmac_ts_event_enable(nss_gmac_dev *gmacdev);	/* Only for "Advanced Time Stamp" */
void nss_gmac_ts_event_disable(nss_gmac_dev *gmacdev);	/* Only for "Advanced Time Stamp" */
void nss_gmac_ts_ipv4_enable(nss_gmac_dev *gmacdev);	/* Only for "Advanced Time Stamp" */
void nss_gmac_ts_ipv4_disable(nss_gmac_dev *gmacdev);	/* Only for "Advanced Time Stamp" */
void nss_gmac_ts_ipv6_enable(nss_gmac_dev *gmacdev);	/* Only for "Advanced Time Stamp" */
void nss_gmac_ts_ipv6_disable(nss_gmac_dev *gmacdev);	/* Only for "Advanced Time Stamp" */
void nss_gmac_ts_ptp_over_ethernet_enable(nss_gmac_dev *gmacdev);	/* Only for "Advanced Time Stamp" */
void nss_gmac_ts_ptp_over_ethernet_disable(nss_gmac_dev *gmacdev);	/* Only for "Advanced Time Stamp" */
void nss_gmac_ts_pkt_snoop_ver2(nss_gmac_dev *gmacdev);	/* Only for "Advanced Time Stamp" */
void nss_gmac_ts_pkt_snoop_ver1(nss_gmac_dev *gmacdev);	/* Only for "Advanced Time Stamp" */

void nss_gmac_ts_digital_rollover_enable(nss_gmac_dev *gmacdev);
void nss_gmac_ts_binary_rollover_enable(nss_gmac_dev *gmacdev);
void nss_gmac_ts_all_frames_enable(nss_gmac_dev *gmacdev);	/* Only for "Advanced Time Stamp" */
void nss_gmac_ts_all_frames_disable(nss_gmac_dev *gmacdev);	/* Only for "Advanced Time Stamp" */

int32_t nss_gmac_ts_addend_update(nss_gmac_dev *gmacdev,
				  uint32_t addend_value);
int32_t nss_gmac_ts_timestamp_update(nss_gmac_dev *gmacdev,
				     uint32_t high_value, uint32_t low_value);
int32_t nss_gmac_ts_timestamp_init(nss_gmac_dev *gmacdev, uint32_t high_value,
				   uint32_t low_value);

void nss_gmac_ts_coarse_update(nss_gmac_dev *gmacdev);	/* Only if "fine correction" enabled */
void nss_gmac_ts_fine_update(nss_gmac_dev *gmacdev);	/* Only if "fine correction" enabled */

/* Update should happen making use of subsecond mask */
void nss_gmac_ts_subsecond_init(nss_gmac_dev *gmacdev,
				uint32_t sub_sec_inc_val);

/*
 * Reads the timestamp low,high and higher(Ver2) registers
 * in the the struct pointer; readonly contents
 */
void nss_gmac_ts_read_timestamp(nss_gmac_dev *gmacdev,
				uint16_t *higher_sec_val, uint32_t *sec_val,
				uint32_t *sub_sec_val);

/* Loads the timestamp target register with the values provided */
void nss_gmac_ts_load_target_timestamp(nss_gmac_dev *gmacdev,
				       uint32_t sec_val, uint32_t sub_sec_val);

void nss_gmac_ts_load_timestamp_higher_val(nss_gmac_dev *gmacdev,
					   uint32_t higher_sec_val);
void nss_gmac_ts_read_timestamp_higher_val(nss_gmac_dev *gmacdev,
					   uint16_t *higher_sec_val);

/* Read the target time stamp register contents */
void nss_gmac_ts_read_target_timestamp(nss_gmac_dev *gmacdev,
				       uint32_t *sec_val,
				       uint32_t *sub_sec_val);

/*******************Ip checksum offloading APIs**********************************/
void nss_gmac_enable_rx_chksum_offload(nss_gmac_dev *gmacdev);
void nss_gmac_disable_rx_chksum_offload(nss_gmac_dev *gmacdev);
void nss_gmac_rx_tcpip_chksum_drop_enable(nss_gmac_dev *gmacdev);
void nss_gmac_rx_tcpip_chksum_drop_disable(nss_gmac_dev *gmacdev);

/**
 * Decodes the Rx Descriptor status to various checksum error conditions.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] uint32_t status field of the corresponding descriptor.
 * @return returns decoded enum (uint32_t) indicating the status.
 */
static inline uint32_t nss_gmac_is_rx_checksum_error(nss_gmac_dev *gmacdev,
						     uint32_t status)
{
	return (status & (DescRxChkBit7 | DescRxChkBit5 | DescRxChkBit0));
}

/**
 * Checks if any Ipv4 header checksum error in the frame just transmitted.
 * This serves as indication that error occureed in the IPv4 header checksum insertion.
 * The sent out frame doesnot carry any ipv4 header checksum inserted by the hardware.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] uint32_t status field of the corresponding descriptor.
 * @return returns true if error in ipv4 header checksum, else returns false.
 */
static inline bool nss_gmac_is_tx_ipv4header_checksum_error(nss_gmac_dev *
							    gmacdev,
							    uint32_t status)
{
	return ((status & DescTxIpv4ChkError) == DescTxIpv4ChkError);
}

/**
 * Checks if any payload checksum error in the frame just transmitted.
 * This serves as indication that error occureed in the payload checksum insertion.
 * The sent out frame doesnot carry any payload checksum inserted by the hardware.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] uint32_t status field of the corresponding descriptor.
 * @return returns true if error in ipv4 header checksum, else returns false.
 */
static inline bool nss_gmac_is_tx_payload_checksum_error(nss_gmac_dev *gmacdev,
							 uint32_t status)
{
	return ((status & DescTxPayChkError) == DescTxPayChkError);
}

/**
 * The check summ offload engine is bypassed in the tx path.
 * Checksum is not computed in the Hardware.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] Pointer to tx descriptor for which  ointer to nss_gmac_dev.
 * @return returns void.
 */
static inline void nss_gmac_tx_checksum_offload_bypass(nss_gmac_dev *gmacdev,
						       DmaDesc *desc)
{
	desc->status = (desc->length & (~DescTxCisMask));
}

/**
 * The check summ offload engine is enabled to do only IPV4 header checksum.
 * IPV4 header Checksum is computed in the Hardware.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] Pointer to tx descriptor for which  ointer to nss_gmac_dev.
 * @return returns void.
 */
static inline void nss_gmac_tx_checksum_offload_ipv4hdr(nss_gmac_dev *gmacdev,
							DmaDesc *desc)
{
	desc->status = ((desc->status & (~DescTxCisMask)) | DescTxCisIpv4HdrCs);
}

/**
 * The check summ offload engine is enabled to do TCPIP checsum assuming Pseudo header is available.
 * Hardware computes the tcp ip checksum assuming pseudo header checksum is computed in software.
 * Ipv4 header checksum is also inserted.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] Pointer to tx descriptor for which  ointer to nss_gmac_dev.
 * @return returns void.
 */
static inline void nss_gmac_tx_checksum_offload_tcponly(nss_gmac_dev *gmacdev,
							DmaDesc *desc)
{
	desc->status = ((desc->status & (~DescTxCisMask)) | DescTxCisTcpOnlyCs);
}

/**
 * The check summ offload engine is enabled to do complete checksum computation.
 * Hardware computes the tcp ip checksum including the pseudo header checksum.
 * Here the tcp payload checksum field should be set to 0000.
 * Ipv4 header checksum is also inserted.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] Pointer to tx descriptor for which pointer to nss_gmac_dev.
 * @return returns void.
 */
static inline void nss_gmac_tx_checksum_offload_tcp_pseudo(nss_gmac_dev *
							   gmacdev,
							   DmaDesc *desc)
{
	desc->status =
	    ((desc->status & (~DescTxCisMask)) | DescTxCisTcpPseudoCs);
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
static uint32_t __inline__ nss_gmac_read_reg(uint32_t *regbase,
					     uint32_t regoffset)
{
	uint32_t addr = 0;
	uint32_t data;

	spin_lock(&ctx.reg_lock);
	addr = (uint32_t)regbase + regoffset;
	data = readl((unsigned char *)addr);
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
static void __inline__ nss_gmac_write_reg(uint32_t *regbase,
					  uint32_t regoffset,
					  uint32_t regdata)
{
	uint32_t addr = 0;

	spin_lock(&ctx.reg_lock);
	addr = (uint32_t)regbase + regoffset;
	writel(regdata, (unsigned char *)addr);
	spin_unlock(&ctx.reg_lock);
}


/**
 * @brief Low level function to set bits of a register in Hardware.
 * @param[in] pointer containing address of register base
 * @param[in] register offset
 * @param[in] bit mask of bits to be set
 * @return void
 */
static void __inline__ nss_gmac_set_reg_bits(uint32_t *regbase,
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
static void __inline__ nss_gmac_clear_reg_bits(uint32_t *regbase,
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
static bool __inline__ nss_gmac_check_reg_bits(uint32_t *regbase,
					       uint32_t regoffset,
					       uint32_t bitpos)
{
	uint32_t data;

	data = bitpos & nss_gmac_read_reg(regbase, regoffset);

	return (data != 0);
}

int32_t nss_gmac_set_mdc_clk_div(nss_gmac_dev *gmacdev, uint32_t clk_div_val);
uint32_t nss_gmac_get_mdc_clk_div(nss_gmac_dev *gmacdev);
uint16_t nss_gmac_mii_rd_reg(nss_gmac_dev *gmacdev, uint32_t phy,
			     uint32_t reg);
void nss_gmac_mii_wr_reg(nss_gmac_dev *gmacdev, uint32_t phy,
			 uint32_t reg, uint16_t data);
int32_t nss_gmac_phy_loopback(nss_gmac_dev *gmacdev, bool loopback);
int32_t nss_gmac_read_version(nss_gmac_dev *gmacdev);
void nss_gmac_reset(nss_gmac_dev *gmacdev);
int32_t nss_gmac_dma_bus_mode_init(nss_gmac_dev *gmacdev, uint32_t init_value);
int32_t nss_gmac_dma_axi_bus_mode_init(nss_gmac_dev *gmacdev, uint32_t init_value);
int32_t nss_gmac_dma_control_init(nss_gmac_dev *gmacdev, uint32_t init_value);
void nss_gmac_wd_enable(nss_gmac_dev *gmacdev);
void nss_gmac_wd_disable(nss_gmac_dev *gmacdev);
void nss_gmac_jab_enable(nss_gmac_dev *gmacdev);
void nss_gmac_jab_disable(nss_gmac_dev *gmacdev);
void nss_gmac_frame_burst_enable(nss_gmac_dev *gmacdev);
void nss_gmac_frame_burst_disable(nss_gmac_dev *gmacdev);
void nss_gmac_jumbo_frame_enable(nss_gmac_dev *gmacdev);
void nss_gmac_jumbo_frame_disable(nss_gmac_dev *gmacdev);
void nss_gmac_twokpe_frame_disable(nss_gmac_dev *gmacdev);
void nss_gmac_twokpe_frame_enable(nss_gmac_dev *gmacdev);
void nss_gmac_select_gmii(nss_gmac_dev *gmacdev);
void nss_gmac_select_mii(nss_gmac_dev *gmacdev);
void nss_gmac_rx_own_enable(nss_gmac_dev *gmacdev);
void nss_gmac_rx_own_disable(nss_gmac_dev *gmacdev);
void nss_gmac_loopback_on(nss_gmac_dev *gmacdev);
void nss_gmac_loopback_off(nss_gmac_dev *gmacdev);
void nss_gmac_set_full_duplex(nss_gmac_dev *gmacdev);
void nss_gmac_set_half_duplex(nss_gmac_dev *gmacdev);
void nss_gmac_retry_enable(nss_gmac_dev *gmacdev);
void nss_gmac_retry_disable(nss_gmac_dev *gmacdev);
void nss_gmac_pad_crc_strip_enable(nss_gmac_dev *gmacdev);
void nss_gmac_pad_crc_strip_disable(nss_gmac_dev *gmacdev);
void nss_gmac_back_off_limit(nss_gmac_dev *gmacdev, uint32_t value);
void nss_gmac_deferral_check_enable(nss_gmac_dev *gmacdev);
void nss_gmac_deferral_check_disable(nss_gmac_dev *gmacdev);
void nss_gmac_rx_enable(nss_gmac_dev *gmacdev);
void nss_gmac_rx_disable(nss_gmac_dev *gmacdev);
void nss_gmac_tx_enable(nss_gmac_dev *gmacdev);
void nss_gmac_tx_disable(nss_gmac_dev *gmacdev);
void nss_gmac_frame_filter_enable(nss_gmac_dev *gmacdev);
void nss_gmac_frame_filter_disable(nss_gmac_dev *gmacdev);
void nss_gmac_write_hash_table_high(nss_gmac_dev *gmacdev, uint32_t data);
void nss_gmac_write_hash_table_low(nss_gmac_dev *gmacdev, uint32_t data);
void nss_gmac_hash_perfect_filter_enable(nss_gmac_dev *gmacdev);
void nss_gmac_Hash_filter_only_enable(nss_gmac_dev *gmacdev);
void nss_gmac_src_addr_filter_enable(nss_gmac_dev *gmacdev);
void nss_gmac_src_addr_filter_disable(nss_gmac_dev *gmacdev);
void nss_gmac_dst_addr_filter_inverse(nss_gmac_dev *gmacdev);
void nss_gmac_dst_addr_filter_normal(nss_gmac_dev *gmacdev);
void nss_gmac_set_pass_control(nss_gmac_dev *gmacdev, uint32_t passcontrol);
void nss_gmac_broadcast_enable(nss_gmac_dev *gmacdev);
void nss_gmac_broadcast_disable(nss_gmac_dev *gmacdev);
void nss_gmac_multicast_enable(nss_gmac_dev *gmacdev);
void nss_gmac_multicast_disable(nss_gmac_dev *gmacdev);
void nss_gmac_multicast_hash_filter_enable(nss_gmac_dev *gmacdev);
void nss_gmac_multicast_hash_filter_disable(nss_gmac_dev *gmacdev);
void nss_gmac_promisc_enable(nss_gmac_dev *gmacdev);
void nss_gmac_promisc_disable(nss_gmac_dev *gmacdev);
void nss_gmac_unicast_hash_filter_enable(nss_gmac_dev *gmacdev);
void nss_gmac_unicast_hash_filter_disable(nss_gmac_dev *gmacdev);
void nss_gmac_unicast_pause_frame_detect_enable(nss_gmac_dev *gmacdev);
void nss_gmac_unicast_pause_frame_detect_disable(nss_gmac_dev *gmacdev);
void nss_gmac_rx_flow_control_enable(nss_gmac_dev *gmacdev);
void nss_gmac_rx_flow_control_disable(nss_gmac_dev *gmacdev);
void nss_gmac_tx_flow_control_enable(nss_gmac_dev *gmacdev);
void nss_gmac_tx_flow_control_disable(nss_gmac_dev *gmacdev);
void nss_gmac_tx_pause_enable(nss_gmac_dev *gmacdev);
void nss_gmac_tx_pause_disable(nss_gmac_dev *gmacdev);
void nss_gmac_rx_pause_enable(nss_gmac_dev *gmacdev);
void nss_gmac_rx_pause_disable(nss_gmac_dev *gmacdev);
void nss_gmac_flush_tx_fifo(nss_gmac_dev *gmacdev);
void nss_gmac_mac_init(nss_gmac_dev *gmacdev);
int32_t nss_gmac_check_phy_init(nss_gmac_dev *gmacdev);
int32_t nss_gmac_ath_phy_mmd_wr(struct phy_device *phydev, uint32_t mmd_dev_addr,
			uint32_t reg, uint16_t val);
int32_t nss_gmac_ath_phy_mmd_rd(struct phy_device *phydev,
			uint32_t mmd_dev_addr, uint32_t reg);
int32_t nss_gmac_ath_phy_disable_smart_802az(struct phy_device *phydev);
int32_t nss_gmac_ath_phy_disable_802az(struct phy_device *phydev);
void nss_gmac_set_mac_addr(nss_gmac_dev *gmacdev,
			      uint32_t MacHigh, uint32_t MacLow, uint8_t *MacAddr);
void nss_gmac_get_mac_addr(nss_gmac_dev *gmacdev,
			      uint32_t MacHigh, uint32_t MacLow, uint8_t *MacAddr);
int32_t nss_gmac_attach(nss_gmac_dev *gmacdev, uint32_t regBase,
			uint32_t reglen);
void nss_gmac_detach(nss_gmac_dev *gmacdev);
int32_t nss_gmac_check_link(nss_gmac_dev *gmacdev);
void nss_gmac_ipc_offload_init(nss_gmac_dev *gmacdev);
void nss_gmac_tx_rx_desc_init(nss_gmac_dev *gmacdev);
int32_t nss_gmac_init_mdiobus(nss_gmac_dev *gmacdev);
void nss_gmac_deinit_mdiobus(nss_gmac_dev *gmacdev);
void nss_gmac_reset_phy(nss_gmac_dev *gmacdev, uint32_t phyid);
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
void nss_gmac_dev_init(nss_gmac_dev *gmacdev);

/*
 * nss_gmac_dev_set_speed()
 *	Set GMAC speed.
 */
int32_t nss_gmac_dev_set_speed(nss_gmac_dev *gmacdev);

/*
 * nss_gmac_spare_ctl()
 *	Spare Control reset. Required only for emulation.
 */
void nss_gmac_spare_ctl(nss_gmac_dev *gmacdev);

/**
 * Initialize the rx descriptors for ring or chain mode operation.
 * 	- Status field is initialized to 0.
 *	- EndOfRing set for the last descriptor.
 *	- buffer1 and buffer2 set to 0 for ring mode of operation. (note)
 *	- data1 and data2 set to 0. (note)
 * @param[in] pointer to DmaDesc structure.
 * @param[in] whether end of ring
 * @return void.
 * @note Initialization of the buffer1, buffer2, data1,data2 and status
 * are not done here. This only initializes whether one wants to use this descriptor
 * in chain mode or ring mode. For chain mode of operation the buffer2 and data2
 * are programmed before calling this function.
 */
static inline void nss_gmac_rx_desc_init_ring(DmaDesc *desc,
					      bool last_ring_desc)
{
	desc->status = 0;
	desc->length = last_ring_desc ? RxDescEndOfRing : 0;
	desc->buffer1 = 0;
	desc->data1 = 0;
}

/**
 * Initialize the tx descriptors for ring or chain mode operation.
 * 	- Status field is initialized to 0.
 *	- EndOfRing set for the last descriptor.
 *	- buffer1 and buffer2 set to 0 for ring mode of operation. (note)
 *	- data1 and data2 set to 0. (note)
 * @param[in] pointer to DmaDesc structure.
 * @param[in] whether end of ring
 * @return void.
 * @note Initialization of the buffer1, buffer2, data1,data2 and status
 * are not done here. This only initializes whether one wants to use this descriptor
 * in chain mode or ring mode. For chain mode of operation the buffer2 and data2
 * are programmed before calling this function.
 */
static inline void nss_gmac_tx_desc_init_ring(DmaDesc *desc,
					      bool last_ring_desc)
{
	desc->status = last_ring_desc ? TxDescEndOfRing : 0;
	desc->length = 0;
	desc->buffer1 = 0;
	desc->data1 = 0;
}

void nss_gmac_init_rx_desc_base(nss_gmac_dev *gmacdev);
void nss_gmac_init_tx_desc_base(nss_gmac_dev *gmacdev);
void nss_gmac_set_owner_dma(DmaDesc *desc);
void nss_gmac_set_desc_sof(DmaDesc *desc);
void nss_gmac_set_desc_eof(DmaDesc *desc);
bool nss_gmac_is_sof_in_rx_desc(DmaDesc *desc);
bool nss_gmac_is_eof_in_rx_desc(DmaDesc *desc);
bool nss_gmac_is_da_filter_failed(DmaDesc *desc);
bool nss_gmac_is_sa_filter_failed(DmaDesc *desc);

/**
 * Checks whether the descriptor is owned by DMA.
 * If descriptor is owned by DMA then the OWN bit is set to 1.
 * This API is same for both ring and chain mode.
 * @param[in] pointer to DmaDesc structure.
 * @return returns true if Dma owns descriptor and false if not.
 */
static inline bool nss_gmac_is_desc_owned_by_dma(DmaDesc *desc)
{
	return ((desc->status & DescOwnByDma) == DescOwnByDma);
}


/**
 * returns the byte length of received frame including CRC.
 * This returns the no of bytes received in the received ethernet frame including CRC(FCS).
 * @param[in] pointer to DmaDesc structure.
 * @return returns the length of received frame lengths in bytes.
 */
static inline uint32_t nss_gmac_get_rx_desc_frame_length(uint32_t status)
{
	return ((status & DescFrameLengthMask) >> DescFrameLengthShift);
}


/**
 * Checks whether the descriptor is valid
 * if no errors such as CRC/Receive Error/Watchdog Timeout/Late collision/Giant Frame/Overflow/Descriptor
 * error the descritpor is said to be a valid descriptor.
 * @param[in] pointer to DmaDesc structure.
 * @return True if desc valid. false if error.
 */
static inline bool nss_gmac_is_desc_valid(uint32_t status)
{
	return ((status & DescError) == 0);
}


/**
 * Checks whether the descriptor is empty.
 * If the buffer1 and buffer2 lengths are zero in ring mode descriptor is empty.
 * In chain mode buffer2 length is 0 but buffer2 itself contains the next descriptor address.
 * @param[in] pointer to DmaDesc structure.
 * @return returns true if descriptor is empty, false if not empty.
 */
static inline bool nss_gmac_is_desc_empty(DmaDesc *desc)
{
	/* if both the buffer1 length and buffer2 length are zero desc is empty */
	return ((desc->length & DescSize1Mask) == 0);
}


/**
 * Checks whether the rx descriptor is valid.
 * if rx descripor is not in error and complete frame is available in the same descriptor
 * @param[in] status
 * @return returns true if no error and first and last desc bits are set, otherwise it returns false.
 */
static inline bool nss_gmac_is_rx_desc_valid(uint32_t status)
{
	return ((status & (DescError | DescRxFirst | DescRxLast)) ==
		(DescRxFirst | DescRxLast));
}

bool nss_gmac_is_tx_aborted(uint32_t status);
bool nss_gmac_is_tx_carrier_error(uint32_t status);
bool nss_gmac_is_tx_underflow_error(uint32_t status);
bool nss_gmac_is_tx_lc_error(uint32_t status);


/**
 * Gives the transmission collision count.
 * returns the transmission collision count indicating number of
 * collisions occured before the frame was transmitted.
 * Make sure to check excessive collision didnot happen to ensure the count is valid.
 * @param[in] status
 * @return returns the count value of collision.
 */
static inline uint32_t nss_gmac_get_tx_collision_count(uint32_t status)
{
	return ((status & DescTxCollMask) >> DescTxCollShift);
}

static inline uint32_t nss_gmac_is_exc_tx_collisions(uint32_t status)
{
	return ((status & DescTxExcCollisions) == DescTxExcCollisions);
}

bool nss_gmac_is_rx_frame_damaged(uint32_t status);
bool nss_gmac_is_rx_frame_collision(uint32_t status);
bool nss_gmac_is_rx_crc(uint32_t status);
bool nss_gmac_is_frame_dribbling_errors(uint32_t status);
bool nss_gmac_is_rx_frame_length_errors(uint32_t status);


/**
 * Checks whether this rx descriptor is last rx descriptor.
 * This returns true if it is last descriptor either in ring mode or in chain mode.
 * @param[in] pointer to devic structure.
 * @param[in] pointer to DmaDesc structure.
 * @return returns true if it is last descriptor, false if not.
 */
static inline bool nss_gmac_is_last_rx_desc(nss_gmac_dev *gmacdev,
					    DmaDesc *desc)
{
	return (unlikely((desc->length & RxDescEndOfRing) != 0));
}


/**
 * Checks whether this tx descriptor is last tx descriptor.
 * This returns true if it is last descriptor either in ring mode or in chain mode.
 * @param[in] pointer to devic structure.
 * @param[in] pointer to DmaDesc structure.
 * @return returns true if it is last descriptor, false if not.
 */
static inline bool nss_gmac_is_last_tx_desc(nss_gmac_dev *gmacdev,
					    DmaDesc *desc)
{
	return (unlikely((desc->status & TxDescEndOfRing) != 0));
}


/**
 * Checks whether this rx descriptor is in chain mode.
 * This returns true if it is this descriptor is in chain mode.
 * @param[in] pointer to DmaDesc structure.
 * @return returns true if chain mode is set, false if not.
 */
static inline bool nss_gmac_is_rx_desc_chained(DmaDesc *desc)
{
	/*
	 * Use ring mode only.
	 * This is also the only way to support jumbo in the futrue.
	 */
	return 0;
}


/**
 * Checks whether this tx descriptor is in chain mode.
 * This returns true if it is this descriptor is in chain mode.
 * @param[in] pointer to DmaDesc structure.
 * @return returns true if chain mode is set, false if not.
 */
static inline bool nss_gmac_is_tx_desc_chained(DmaDesc *desc)
{
	/*
	 * Use ring mode only.
	 * This is also the only way to support jumbo in the futrue.
	 */
	return 0;
}

void nss_gmac_get_desc_data(DmaDesc *desc, uint32_t *Status,
			    uint32_t *Buffer1, uint32_t *Length1,
			    uint32_t *Data1);

/**
 * Get the index and address of Tx desc.
 * This api is same for both ring mode and chain mode.
 * This function tracks the tx descriptor the DMA just closed after the transmission of data from this descriptor is
 * over. This returns the descriptor fields to the caller.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns present tx descriptor index on success. Negative value if error.
 */
static inline DmaDesc *nss_gmac_get_tx_qptr(nss_gmac_dev *gmacdev)
{
	DmaDesc *txdesc = gmacdev->tx_busy_desc;

	if (unlikely(gmacdev->busy_tx_desc == 0)) {
		return NULL;
	}

	if (nss_gmac_is_desc_owned_by_dma(txdesc)) {
		return NULL;
	}

	BUG_ON(nss_gmac_is_desc_empty(txdesc));

	return txdesc;
}


/**
 * Reset the descriptor after Tx is over.
 * Update descriptor pointers.
 * @param[in] pointer to nss_gmac_dev.
 * @return Returns void
 */
static inline void nss_gmac_reset_tx_qptr(nss_gmac_dev *gmacdev)
{
	uint32_t txover = gmacdev->tx_busy;
	DmaDesc *txdesc = gmacdev->tx_busy_desc;

	BUG_ON(txdesc != (gmacdev->tx_desc + txover));
	gmacdev->tx_busy = (txover + 1) & (gmacdev->tx_desc_count - 1);
	gmacdev->tx_busy_desc = gmacdev->tx_desc + gmacdev->tx_busy;

	txdesc->status &= TxDescEndOfRing;
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
 * Once the driver has a packet ready to be transmitted, this function is called with the
 * valid dma-able buffer addresses and their lengths. This function populates the descriptor
 * and make the DMA the owner for the descriptor. This function also controls whetther Checksum
 * offloading to be done in hardware or not.
 * This api is same for both ring mode and chain mode.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] Dma-able buffer1 pointer.
 * @param[in] length of buffer1 (Max is 2048).
 * @param[in] virtual pointer for buffer1.
 * @param[in] uint32_t indicating whether the checksum offloading in HW/SW.
 * @param[in] uint32_t indicating TX control flag - the first, last segment and interrupt state.
 * @param[in] uint32_t indicating descriptor DMA flag state.
 * @return returns present tx descriptor index on success. Negative value if error.
 */
static inline int32_t nss_gmac_set_tx_qptr(nss_gmac_dev *gmacdev,
					   uint32_t Buffer1, uint32_t Length1,
					   uint32_t Data1,
					   uint32_t offload_needed,
					   uint32_t tx_cntl, uint32_t set_dma)
{
	uint32_t txnext = gmacdev->tx_next;
	DmaDesc *txdesc = gmacdev->tx_next_desc;

	BUG_ON(gmacdev->busy_tx_desc > gmacdev->tx_desc_count);
	BUG_ON(txdesc != (gmacdev->tx_desc + txnext));
	BUG_ON(!nss_gmac_is_desc_empty(txdesc));
	BUG_ON(nss_gmac_is_desc_owned_by_dma(txdesc));

	if (Length1 > NSS_GMAC_MAX_DESC_BUFF) {
		txdesc->length |=
		    (NSS_GMAC_MAX_DESC_BUFF << DescSize1Shift) & DescSize1Mask;
		txdesc->length |=
		    ((Length1 -
		      NSS_GMAC_MAX_DESC_BUFF) << DescSize2Shift) & DescSize2Mask;
	} else {
		txdesc->length |= ((Length1 << DescSize1Shift) & DescSize1Mask);
	}

	txdesc->status |= tx_cntl;

	txdesc->buffer1 = Buffer1;
	txdesc->reserved1 = Data1;

	/* Program second buffer address if using two buffers. */
	if (Length1 > NSS_GMAC_MAX_DESC_BUFF) {
		txdesc->data1 = Buffer1 + NSS_GMAC_MAX_DESC_BUFF;
	} else {
		txdesc->data1 = 0;
	}

	if (likely(offload_needed)) {
		nss_gmac_tx_checksum_offload_tcp_pseudo(gmacdev, txdesc);
	}

	txdesc->status |= set_dma;

	gmacdev->tx_next = (txnext + 1) & (gmacdev->tx_desc_count - 1);
	gmacdev->tx_next_desc = gmacdev->tx_desc + gmacdev->tx_next;

	return txnext;
}


/**
 * Prepares the descriptor to receive packets.
 * The descriptor is allocated with the valid buffer addresses (sk_buff address) and the length fields
 * and handed over to DMA by setting the ownership. After successful return from this function the
 * descriptor is added to the receive descriptor pool/queue.
 * This api is same for both ring mode and chain mode.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] Dma-able buffer1 pointer.
 * @param[in] length of buffer1 (Max is 2048).
 * @param[in] pointer to buffer context.
 * @return returns present rx descriptor index on success. Negative value if error.
 */
static inline int32_t nss_gmac_set_rx_qptr(nss_gmac_dev *gmacdev,
					   uint32_t Buffer1, uint32_t Length1,
					   uint32_t Data1)
{
	uint32_t rxnext = gmacdev->rx_next;
	DmaDesc *rxdesc = gmacdev->rx_next_desc;

	BUG_ON(gmacdev->busy_rx_desc >= gmacdev->rx_desc_count);
	BUG_ON(rxdesc != (gmacdev->rx_desc + rxnext));
	BUG_ON(!nss_gmac_is_desc_empty(rxdesc));
	BUG_ON(nss_gmac_is_desc_owned_by_dma(rxdesc));

	if (Length1 > NSS_GMAC_MAX_DESC_BUFF) {
		rxdesc->length |=
		    (NSS_GMAC_MAX_DESC_BUFF << DescSize1Shift) & DescSize1Mask;
		rxdesc->length |=
		    ((Length1 -
		      NSS_GMAC_MAX_DESC_BUFF) << DescSize2Shift) & DescSize2Mask;
	} else {
		rxdesc->length |= ((Length1 << DescSize1Shift) & DescSize1Mask);
	}

	rxdesc->buffer1 = Buffer1;
	rxdesc->reserved1 = Data1;

	/* Program second buffer address if using two buffers. */
	if (Length1 > NSS_GMAC_MAX_DESC_BUFF) {
		rxdesc->data1 = Buffer1 + NSS_GMAC_MAX_DESC_BUFF;
	} else {
		rxdesc->data1 = 0;
	}

	rxdesc->extstatus = 0;
	rxdesc->timestamplow = 0;
	rxdesc->timestamphigh = 0;

	rxdesc->status = DescOwnByDma;

	gmacdev->rx_next = (rxnext + 1) & (gmacdev->rx_desc_count - 1);
	gmacdev->rx_next_desc = gmacdev->rx_desc + gmacdev->rx_next;

	/* One descriptor will be given to Hardware. So busy count incremented by one. */
	(gmacdev->busy_rx_desc)++;

	return rxnext;
}


/**
 * Get back the descriptor from DMA after data has been received.
 * When the DMA indicates that the data is received (interrupt is generated), this function should be
 * called to get the descriptor and hence the data buffers received. With successful return from this
 * function caller gets the descriptor fields for processing. check the parameters to understand the
 * fields returned.`
 * @param[in] pointer to nss_gmac_dev.
 * @return returns pointer to DmaDesc on success. Negative value if error.
 */
static inline DmaDesc *nss_gmac_get_rx_qptr(nss_gmac_dev *gmacdev)
{
	DmaDesc *rxdesc = gmacdev->rx_busy_desc;

	if (unlikely(gmacdev->busy_rx_desc == 0)) {
		return NULL;
	}

	if (nss_gmac_is_desc_owned_by_dma(rxdesc)) {
		return NULL;
	}
	BUG_ON(nss_gmac_is_desc_empty(rxdesc));

	return rxdesc;
}


/**
 * Reset the descriptor after Rx is over.
 * Update descriptor pointers.
 * @param[in] pointer to nss_gmac_dev.
 * @return Returns void
 */
static inline void nss_gmac_reset_rx_qptr(nss_gmac_dev *gmacdev)
{

	/* Index of descriptor the DMA just completed.
	 * May be useful when data is spread over multiple buffers/descriptors
	 */
	uint32_t rxnext = gmacdev->rx_busy;
	DmaDesc *rxdesc = gmacdev->rx_busy_desc;

	BUG_ON(rxdesc != (gmacdev->rx_desc + rxnext));
	gmacdev->rx_busy = (rxnext + 1) & (gmacdev->rx_desc_count - 1);
	gmacdev->rx_busy_desc = gmacdev->rx_desc + gmacdev->rx_busy;

	rxdesc->status = 0;
	rxdesc->length &= RxDescEndOfRing;
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
static inline void nss_gmac_clear_interrupt(nss_gmac_dev *gmacdev)
{
	uint32_t data;
	data = nss_gmac_read_reg((uint32_t *)gmacdev->dma_base, DmaStatus);
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base, DmaStatus, data);
}


/**
 * Returns the all unmasked interrupt status after reading the DmaStatus register.
 * @param[in] pointer to nss_gmac_dev.
 * @return 0 upon success. Error code upon failure.
 */
static inline uint32_t nss_gmac_get_interrupt_type(nss_gmac_dev *gmacdev)
{
	uint32_t interrupts = 0;

	interrupts =
	    nss_gmac_read_reg((uint32_t *)gmacdev->dma_base, DmaStatus);

	/* Clear interrupt here */
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base, DmaStatus,
			   interrupts);

	return interrupts;
}


/**
 * Returns the interrupt mask.
 * @param[in] pointer to nss_gmac_dev.
 * @return 0 upon success. Error code upon failure.
 */
static inline uint32_t nss_gmac_get_interrupt_mask(nss_gmac_dev *gmacdev)
{
	return (nss_gmac_read_reg((uint32_t *)gmacdev->dma_base, DmaInterrupt));
}


/**
 * @brief Enables the DMA interrupt as specified by the bit mask.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] bit mask of interrupts to be enabled.
 * @return returns void.
 */
static inline void nss_gmac_enable_interrupt(nss_gmac_dev *gmacdev,
					     uint32_t interrupts)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base, DmaInterrupt,
			   interrupts);
}


/**
 * @brief Disable all the interrupts.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
static inline void nss_gmac_disable_mac_interrupt(nss_gmac_dev *gmacdev)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base, GmacInterruptMask,
			   0xFFFFFFFF);
}


/**
 * Disable all the interrupts.
 * Disables all DMA interrupts.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 * @note This function disabled all the interrupts, if you want to disable a particular interrupt then
 *  use nss_gmac_disable_interrupt().
 */
static inline void nss_gmac_disable_interrupt_all(nss_gmac_dev *gmacdev)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base, DmaInterrupt,
			   DmaIntDisable);
	nss_gmac_disable_mac_interrupt(gmacdev);
}


/**
 * Disable interrupt according to the bitfield supplied.
 * Disables only those interrupts specified in the bit mask in second argument.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] bit mask for interrupts to be disabled.
 * @return returns void.
 */
static inline void nss_gmac_disable_interrupt(nss_gmac_dev *gmacdev,
					      uint32_t interrupts)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->dma_base, DmaInterrupt,
				interrupts);
}


/**
 * Read interrupt status
 * @param[in] pointer to nss_gmac_dev.
 * @return returns bitmask of pending interrupts
 */
static inline uint32_t nss_gmac_check_interrupt(nss_gmac_dev *gmacdev)
{
	uint32_t int_status;

	int_status =
	    nss_gmac_read_reg((uint32_t *)gmacdev->dma_base, DmaStatus);

	return int_status;
}

void nss_gmac_enable_dma_rx(nss_gmac_dev *gmacdev);
void nss_gmac_enable_dma_tx(nss_gmac_dev *gmacdev);


/**
 * Resumes the DMA Transmission.
 * the DmaTxPollDemand is written. (the data writeen could be anything).
 * This forces the DMA to resume transmission.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
static inline void nss_gmac_resume_dma_tx(nss_gmac_dev *gmacdev)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base, DmaTxPollDemand, 0);
}


/**
 * Resumes the DMA Reception.
 * the DmaRxPollDemand is written. (the data writeen could be anything).
 * This forces the DMA to resume reception.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
static inline void nss_gmac_resume_dma_rx(nss_gmac_dev *gmacdev)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base, DmaRxPollDemand, 0);
}

void nss_gmac_take_desc_ownership(DmaDesc *desc);
void nss_gmac_take_desc_ownership_rx(nss_gmac_dev *gmacdev);
void nss_gmac_take_desc_ownership_tx(nss_gmac_dev *gmacdev);
void nss_gmac_disable_dma_tx(nss_gmac_dev *gmacdev);
void nss_gmac_disable_dma_rx(nss_gmac_dev *gmacdev);

/**
 * This function indicates whether extended status is available in the RDES0.
 * Any function which accesses the fields of extended status register must
 * ensure a check on this has been made.
 * Extended status present indicates that the RDES4 need to be probed
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] uint32_t status field of the corresponding descriptor.
 * @return returns TRUE or FALSE.
 */
static inline bool nss_gmac_is_ext_status(nss_gmac_dev *gmacdev,
					  uint32_t status)
{
	return ((status & DescRxEXTsts) != 0);
}


/**
 * This function returns true if the IP header checksum bit is set in
 * the extended status. Valid only when enhaced status available is set
 * in RDES0 bit 0.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] uint32_t status field of the corresponding descriptor.
 * @return returns TRUE or FALSE.
 */
static inline bool nss_gmac_ES_is_IP_header_error(nss_gmac_dev *gmacdev,
						  uint32_t ext_status)
{
	return ((ext_status & DescRxIpHeaderError) != 0);
}


/**
 * This function returns true if the Checksum is bypassed in the hardware.
 * Valid only when enhaced status available is set in RDES0 bit 0.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] uint32_t status field of the corresponding descriptor.
 * @return returns TRUE or FALSE.
 */
static inline bool nss_gmac_ES_is_rx_checksum_bypassed(nss_gmac_dev *gmacdev,
						       uint32_t ext_status)
{
	return ((ext_status & DescRxChkSumBypass) != 0);
}


/**
 * This function returns true if payload checksum error
 * (UDP/TCP/ICMP checksum error) is set in the extended status.
 * Valid only when enhaced status available is set in RDES0 bit 0.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] uint32_t status field of the corresponding descriptor.
 * @return returns TRUE or FALSE.
 */
static inline bool nss_gmac_ES_is_IP_payload_error(nss_gmac_dev *gmacdev,
						   uint32_t ext_status)
{
	return ((ext_status & DescRxIpPayloadError) != 0);
}

/*******************PMT APIs***************************************/
void nss_gmac_pmt_int_enable(nss_gmac_dev *gmacdev);
void nss_gmac_pmt_int_disable(nss_gmac_dev *gmacdev);
void nss_gmac_power_down_enable(nss_gmac_dev *gmacdev);
void nss_gmac_power_down_disable(nss_gmac_dev *gmacdev);
void nss_gmac_magic_packet_enable(nss_gmac_dev *gmacdev);
void nss_gmac_wakeup_frame_enable(nss_gmac_dev *gmacdev);
void nss_gmac_pmt_unicast_enable(nss_gmac_dev *gmacdev);
bool nss_gmac_is_magic_packet_received(nss_gmac_dev *gmacdev);
bool nss_gmac_is_wakeup_frame_received(nss_gmac_dev *gmacdev);
void nss_gmac_write_wakeup_frame_register(nss_gmac_dev *gmacdev,
					  uint32_t *filter_contents);

/*******************MMC APIs***************************************/
void nss_gmac_mmc_counters_stop(nss_gmac_dev *gmacdev);
void nss_gmac_mmc_counters_resume(nss_gmac_dev *gmacdev);
void nss_gmac_mmc_counters_set_selfclear(nss_gmac_dev *gmacdev);
void nss_gmac_mmc_counters_reset_selfclear(nss_gmac_dev *gmacdev);
void nss_gmac_mmc_counters_disable_rollover(nss_gmac_dev *gmacdev);
void nss_gmac_mmc_counters_enable_rollover(nss_gmac_dev *gmacdev);
uint32_t nss_gmac_read_mmc_counter(nss_gmac_dev *gmacdev, uint32_t counter);
uint32_t nss_gmac_read_mmc_rx_int_status(nss_gmac_dev *gmacdev);
uint32_t nss_gmac_read_mmc_tx_int_status(nss_gmac_dev *gmacdev);
void nss_gmac_disable_mmc_tx_interrupt(nss_gmac_dev *gmacdev, uint32_t mask);
void nss_gmac_enable_mmc_tx_interrupt(nss_gmac_dev *gmacdev, uint32_t mask);
void nss_gmac_disable_mmc_rx_interrupt(nss_gmac_dev *gmacdev, uint32_t mask);
void nss_gmac_enable_mmc_rx_interrupt(nss_gmac_dev *gmacdev, uint32_t mask);
void nss_gmac_enable_mmc_ipc_rx_interrupt(nss_gmac_dev *gmacdev,
					  uint32_t mask);
void nss_gmac_disable_mmc_ipc_rx_interrupt(nss_gmac_dev *gmacdev,
					   uint32_t mask);


#endif /* End of file */
