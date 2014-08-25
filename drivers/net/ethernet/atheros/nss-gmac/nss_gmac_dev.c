/* Copyright (c) 2013, The Linux Foundation. All rights reserved.*/
/*
 * @file
 * This file defines the NSS GMAC device dependent functions.
 * Most of the operations on the GMAC device are available in this file.
 * Functions for initiliasing and accessing MAC/DMA/PHY registers and
 * the DMA descriptors are encapsulated in this file. The functions are
 * platform/host/OS independent.
 * These functions in turn use the low level device dependent (HAL) functions
 * to access the register space.
 * ------------------------REVISION HISTORY---------------------------------
 * Qualcomm Atheros         01/Mar/2013                 Modified for QCA NSS
 * Synopsys                 01/Aug/2007                 Created
 */

#include <linux/mii.h>
#include <linux/phy.h>

#if (NSS_GMAC_DT_SUPPORT != 1)
#include <mach/msm_iomap.h>
#endif

#include <nss_gmac_dev.h>
#include <nss_gmac_network_interface.h>

/*
 * Function to check the current link status
 * @param[in] pointer to device structure.
 * @return Returns LINKUP or LINKDOWN
 */
int32_t nss_gmac_check_link(nss_gmac_dev *gmacdev)
{
	struct phy_device *phydev = gmacdev->phydev;

	if (!test_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags)) {
		return LINKUP;
	}

	if (gmacdev->emulation && (gmacdev->phy_mii_type == GMAC_INTF_SGMII
			|| gmacdev->phy_mii_type == GMAC_INTF_QSGMII)) {
		return LINKUP;
	}

	genphy_read_status(phydev);

	if (phydev->link) {
		return LINKUP;
	}

	return LINKDOWN;
}

/*
 * Function to set the MDC clock for mdio transactiona
 * @param[in] pointer to device structure.
 * @param[in] clk divider value.
 * @return Reuturns 0 on success else return the error value.
 */
int32_t nss_gmac_set_mdc_clk_div(nss_gmac_dev *gmacdev, uint32_t clk_div_val)
{
	uint32_t orig_data;

	/* set the mdc clock to the user defined value */
	orig_data =
	    nss_gmac_read_reg((uint32_t *)gmacdev->mac_base, GmacGmiiAddr);
	orig_data &= (~GmiiCsrClkMask);
	orig_data |= clk_div_val;
	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base, GmacGmiiAddr,
			   orig_data);

	return 0;
}

/*
 * Returns the current MDC divider value programmed in the ip.
 * @param[in] pointer to device structure.
 * @param[in] clk divider value.
 * @return Returns the MDC divider value read.
 */
uint32_t nss_gmac_get_mdc_clk_div(nss_gmac_dev *gmacdev)
{
	uint32_t data;
	data = nss_gmac_read_reg((uint32_t *)gmacdev->mac_base, GmacGmiiAddr);
	data &= GmiiCsrClkMask;
	return data;
}

/*
 * Function to read the Phy register. The access to phy register
 * is a slow process as the data is moved accross MDI/MDO interface
 * Caller is required to call this function in an SMP safe manner.
 * @param[in] pointer to Register Base (It is the mac base in our case).
 * @param[in] PhyBase register is the index of one of supported 32 PHY devices.
 * @param[in] Register offset is the index of one of the 32 phy register.
 * @param[out] uint16_t data read from the respective phy register
 * (only valid iff return value is 0).
 * @param[in] MDC clock divider value.
 * @return Returns 0 on success else return the error status.
 */
int32_t nss_gmac_read_phy_reg(uint32_t *RegBase, uint32_t PhyBase,
			      uint32_t RegOffset, uint16_t *data,
			      uint32_t mdc_clk_div)
{
	uint32_t addr = 0;
	uint32_t loop_variable;
	volatile uint32_t temp;

	addr = ((PhyBase << GmiiDevShift) & GmiiDevMask)
	    | (((uint32_t)RegOffset << GmiiRegShift) & GmiiRegMask)
	    | mdc_clk_div;

	/* Gmii busy bit */
	addr = addr | GmiiBusy;

	/* write the address from where the data to be read in
	 * GmiiGmiiAddr register of NSS GMAC ip
	 */
	nss_gmac_write_reg(RegBase, GmacGmiiAddr, addr);

	/* Wait till the busy bit gets cleared */
	for (loop_variable = 0; loop_variable
	     < DEFAULT_LOOP_VARIABLE; loop_variable++) {
		temp = nss_gmac_read_reg(RegBase, GmacGmiiAddr);
		if (!(temp & GmiiBusy)) {
			*data =
		    	    (uint16_t)(nss_gmac_read_reg(RegBase,
							 GmacGmiiData) &
							 0xFFFF);
			return 0;

		}
		mdelay(100);
	}

	nss_gmac_early_dbg
	    ("Error::: PHY not responding; Busy bit not cleared!! addr:%x, data:%x\n",
	     temp, *data);

	return -EIO;
}

/*
 * Function to write to the Phy register. The access to phy register
 * is a slow process as the data is moved accross MDI/MDO interface
 * Caller is required to call this function in an SMP safe manner.
 * @param[in] pointer to Register Base (It is the mac base in our case).
 * @param[in] PhyBase register is the index of one of supported 32 PHY devices.
 * @param[in] Register offset is the index of one of the 32 phy register.
 * @param[in] data to be written to the respective phy register.
 * @param[in] MDC clock divider value.
 * @return Returns 0 on success else return the error status.
 */
int32_t nss_gmac_write_phy_reg(uint32_t *RegBase, uint32_t PhyBase,
			       uint32_t RegOffset, uint16_t data,
			       uint32_t mdc_clk_div)
{
	uint32_t addr = 0;
	uint32_t loop_variable;
	volatile uint32_t temp;

	/* write the data in to GmacGmiiData register of GMAC ip */
	nss_gmac_write_reg(RegBase, GmacGmiiData, data);

	addr = ((PhyBase << GmiiDevShift) & GmiiDevMask)
	    | ((RegOffset << GmiiRegShift) & GmiiRegMask)
	    | GmiiWrite | mdc_clk_div;

	addr = addr | GmiiBusy;

	nss_gmac_write_reg(RegBase, GmacGmiiAddr, addr);

	for (loop_variable = 0; loop_variable
	     < DEFAULT_LOOP_VARIABLE; loop_variable++) {
		temp = nss_gmac_read_reg(RegBase, GmacGmiiAddr);
		if (!(temp & GmiiBusy)) {
			return 0;
		}
		mdelay(100);
	}

	nss_gmac_early_dbg
	    ("Error::: PHY not responding; Busy bit not cleared!! addr:data %x:%x",
	     temp, data);

	return -EIO;
}


/**
 * @brief Read a register from an external PHY
 * @param[in] pointer to gmac context
 * @param[in] phy id
 * @param[in] register id
 * @return Returns value read from phy register on success, 0 otherwise.
 */
uint16_t nss_gmac_mii_rd_reg(nss_gmac_dev *gmacdev, uint32_t phy,
			     uint32_t reg)
{
	uint16_t data = 0;

	if (IS_ERR_OR_NULL(gmacdev->phydev)) {
		nss_gmac_info(gmacdev, "Error: Reading uninitialized PHY...");
		return 0;
	}

	data = (uint16_t)phy_read(gmacdev->phydev, reg);

	return data;
}


/**
 * @brief Write a register of an external PHY
 * @param[in] pointer to gmac context
 * @param[in] phy id
 * @param[in] register id
 * @param[in] register id
 * @return void
 */
void nss_gmac_mii_wr_reg(nss_gmac_dev *gmacdev, uint32_t phy,
			 uint32_t reg, uint16_t data)
{
	if (IS_ERR_OR_NULL(gmacdev->phydev)) {
		nss_gmac_info(gmacdev, "Error: Writing uninitialized PHY...");
		return;
	}

	phy_write(gmacdev->phydev, reg, data);

	return;
}



/*
 * Function to configure the phy in loopback mode.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] enable or disable the loopback.
 * @return 0 on success else return the error status.
 * @note Don't get confused with mac loop-back nss_gmac_loopback_on(nss_gmac_dev *)
 * and nss_gmac_loopback_off(nss_gmac_dev *)functions.
 * @return 0 on success.
 */
int32_t nss_gmac_phy_loopback(nss_gmac_dev *gmacdev, bool loopback)
{
	uint32_t bmcr = 0;

	bmcr = nss_gmac_mii_rd_reg(gmacdev, gmacdev->phy_base, MII_BMCR);

	if (loopback) {
		bmcr |= BMCR_LOOPBACK;
	} else {
		bmcr &= ~BMCR_LOOPBACK;
	}

	nss_gmac_mii_wr_reg(gmacdev, gmacdev->phy_base, MII_BMCR, bmcr);

	return 0;
}


/**
 * @brief Reset the Phy specified by phyid
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] phy id
 * @return void
 */
void nss_gmac_reset_phy(nss_gmac_dev *gmacdev, uint32_t phyid)
{
	if (gmacdev->emulation && (gmacdev->phy_mii_type != GMAC_INTF_RGMII)) {
		return;
	}

	nss_gmac_mii_wr_reg(gmacdev, phyid, MII_BMCR, BMCR_RESET);
	nss_gmac_mii_wr_reg(gmacdev, phyid, MII_BMCR,
			    nss_gmac_mii_rd_reg(gmacdev, phyid, MII_BMCR)
			    | BMCR_ANENABLE);

	test_and_set_bit(__NSS_GMAC_AUTONEG, &gmacdev->flags);
	nss_gmac_info(gmacdev, "Phy %u reset OK", phyid);
}


/*
 * Function to read the GMAC IP Version and populates the
 * same in device data structure.
 * @param[in] pointer to nss_gmac_dev.
 * @return Always return 0.
 */
int32_t nss_gmac_read_version(nss_gmac_dev *gmacdev)
{
	uint32_t data = 0;
	data = nss_gmac_read_reg((uint32_t *)gmacdev->mac_base, GmacVersion);
	gmacdev->version = data;
	return 0;
}

/*
 * Function to reset the GMAC core.
 * This reests the DMA and GMAC core. After reset all the
 * registers holds their respective reset value.
 * @param[in] pointer to nss_gmac_dev.
 * @return 0 on success else return the error status.
 */
void nss_gmac_reset(nss_gmac_dev *gmacdev)
{
	uint32_t data = 0;
	uint32_t reset_time __attribute__ ((unused)) = jiffies;
	struct net_device *netdev = NULL;
	struct nss_gmac_global_ctx *ctx;

	netdev = gmacdev->netdev;
	ctx = gmacdev->ctx;

	nss_gmac_info(gmacdev, "%s: %s resetting...",
		      __FUNCTION__, netdev->name);

	reset_time = jiffies;
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base,
			   DmaBusMode, DmaResetOn);
	do {
		mdelay(DEFAULT_LOOP_VARIABLE);
		data =
		    nss_gmac_read_reg((uint32_t *)gmacdev->dma_base,
				      DmaBusMode);
	} while (data & DmaResetOn);

	mdelay(1000);
	data = nss_gmac_read_reg((uint32_t *)gmacdev->dma_base, DmaBusMode);

	nss_gmac_info(gmacdev, "GMAC reset completed in %d jiffies; "
		      "DmaBusMode - 0x%x", (int)(jiffies - reset_time), data);
}

/*
 * Function to program DMA bus mode register.
 * The Bus Mode register is programmed with the value given.
 * The bits to be set are bit wise or'ed and sent as the second
 * argument to this function.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] the data to be programmed.
 * @return 0 on success else return the error status.
 */
int32_t nss_gmac_dma_bus_mode_init(nss_gmac_dev *gmacdev, uint32_t init_value)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base, DmaBusMode,
			   init_value);
	return 0;
}

/*
 * Function to program DMA AXI bus mode register.
 * The Bus Mode register is programmed with the value given.
 * The bits to be set are bit wise or'ed and sent as the second
 * argument to this function.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] the data to be programmed.
 * @return 0 on success else return the error status.
 */
int32_t nss_gmac_dma_axi_bus_mode_init(nss_gmac_dev *gmacdev, uint32_t init_value)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base, DmaAxiBusMode,
			   init_value);
	return 0;
}

/*
 * Function to program DMA Control register.
 * The Dma Control register is programmed with the value given.
 * The bits to be set are bit wise or'ed and sent as the second
 * argument to this function.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] the data to be programmed.
 * @return 0 on success else return the error status.
 */
int32_t nss_gmac_dma_control_init(nss_gmac_dev *gmacdev, uint32_t init_value)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base, DmaControl,
			   init_value);
	return 0;
}

/* Gmac configuration functions */

/*
 * Enable the watchdog timer on the receiver.
 * When enabled, Gmac enables Watchdog timer, and GMAC allows no more than
 * 2048 bytes of data (10,240 if Jumbo frame enabled).
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_wd_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacConfig, GmacWatchdog);
}

/*
 * Disable the watchdog timer on the receiver.
 * When disabled, Gmac disabled watchdog timer, and can receive frames up to
 * 16,384 bytes.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_wd_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacConfig, GmacWatchdog);
}

/*
 * Enables the Jabber frame support.
 * When enabled, GMAC disabled the jabber timer, and can transfer
 * 16,384 byte frames.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_jab_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacConfig, GmacJabber);
}

/*
 * Disables the Jabber frame support.
 * When disabled, GMAC enables jabber timer.
 * It cuts of transmitter if application sends more than 2048
 * bytes of data (10240 if Jumbo frame enabled).
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_jab_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacConfig, GmacJabber);
}

/*
 * Enables Frame bursting (Only in Half Duplex Mode).
 * When enabled, GMAC allows frame bursting in GMII Half Duplex mode.
 * Reserved in 10/100 and Full-Duplex configurations.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_frame_burst_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacConfig, GmacFrameBurst);
}

/*
 * Disables Frame bursting.
 * When Disabled, frame bursting is not supported.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_frame_burst_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacConfig, GmacFrameBurst);
}

/*
 * Enable Jumbo frame support.
 * When Enabled GMAC supports jumbo frames of 9018/9022(VLAN tagged).
 * Giant frame error is not reported in receive frame status.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_jumbo_frame_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacConfig, GmacJumboFrame);
}

/*
 * Disable Jumbo frame support.
 * When Disabled GMAC does not supports jumbo frames.
 * Giant frame error is reported in receive frame status.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_jumbo_frame_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacConfig, GmacJumboFrame);
}

/*
 * Enable twokpe frame support.
 * When Enabled GMAC supports jumbo frames of <= 2000 bytes.
 * Giant frame error is not reported in receive frame status.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_twokpe_frame_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacConfig, GmacTwokpe);
}

/*
 * Disable twokpe SUPPORT.
 * When disabled gmac does not support frames of length > 1522 bytes.
 * Giant frame error is reported in receive frame status
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_twokpe_frame_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacConfig, GmacTwokpe);
}

/*
 * Disable Carrier sense.
 * When Disabled GMAC ignores CRS signal during frame transmission
 * in half duplex mode.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_disable_crs(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacConfig, GmacDisableCrs);
}

/*
 * Selects the GMII port.
 * When called GMII (1000Mbps) port is selected (programmable only in 10/100/1000 Mbps configuration).
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_select_gmii(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacConfig, GmacMiiGmii);
}

/*
 * Selects the MII port.
 * When called MII (10/100Mbps) port is selected (programmable only in 10/100/1000 Mbps configuration).
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_select_mii(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacConfig, GmacMiiGmii);

	if (gmacdev->speed == SPEED100) {
		nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
				      GmacConfig, GmacFESpeed100);
		return;
	}

	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacConfig, GmacFESpeed100);
}

/*
 * Enables Receive Own bit (Only in Half Duplex Mode).
 * When enaled GMAC receives all the packets given by phy while transmitting.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_rx_own_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacConfig, GmacRxOwn);
}

/*
 * Disables Receive Own bit (Only in Half Duplex Mode).
 * When enaled GMAC disables the reception of frames when
 * gmii_txen_o is asserted.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_rx_own_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacConfig, GmacRxOwn);
}

/*
 * Sets the GMAC in loopback mode.
 * When on GMAC operates in loop-back mode at GMII/MII.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 * @note (G)MII Receive clock is required for loopback to work properly,
 * as transmit clock is not looped back internally.
 */
void nss_gmac_loopback_on(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacConfig, GmacLoopback);
}

/*
 * Sets the GMAC in Normal mode.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_loopback_off(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacConfig, GmacLoopback);
}

/*
 * Sets the GMAC core in Full-Duplex mode.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_set_full_duplex(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacConfig, GmacDuplex);
}

/*
 * Sets the GMAC core in Half-Duplex mode.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_set_half_duplex(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacConfig, GmacDuplex);
}

/*
 * GMAC tries retransmission (Only in Half Duplex mode).
 * If collision occurs on the GMII/MII, GMAC attempt retries based on the
 * back off limit configured.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 * @note This function is tightly coupled with
 * nss_gmac_back_off_limit(nss_gmac_dev *, uint32_t).
 */
void nss_gmac_retry_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacConfig, GmacRetry);
}

/*
 * GMAC tries only one transmission (Only in Half Duplex mode).
 * If collision occurs on the GMII/MII, GMAC will ignore the current frami
 * transmission and report a frame abort with excessive collision
 * in tranmit frame status.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_retry_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacConfig, GmacRetry);
}

/*
 * GMAC strips the Pad/FCS field of incoming frames.
 * This is true only if the length field value is less than or equal to
 * 1500 bytes. All received frames with length field greater than or equal to
 * 1501 bytes are passed to the application without stripping the Pad/FCS field.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_pad_crc_strip_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacConfig, GmacPadCrcStrip);
}

/*
 * GMAC doesnot strips the Pad/FCS field of incoming frames.
 * GMAC will pass all the incoming frames to Host unmodified.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_pad_crc_strip_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacConfig, GmacPadCrcStrip);
}

/*
 * GMAC programmed with the back off limit value.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 * @note This function is tightly coupled with
 * nss_gmac_retry_enable(nss_gmac_dev *gmacdev)
 */
void nss_gmac_back_off_limit(nss_gmac_dev *gmacdev, uint32_t value)
{
	uint32_t data;
	data = nss_gmac_read_reg((uint32_t *)gmacdev->mac_base, GmacConfig);
	data &= (~GmacBackoffLimit);
	data |= value;
	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base, GmacConfig, data);
}

/*
 * Enables the Deferral check in GMAC (Only in Half Duplex mode)
 * GMAC issues a Frame Abort Status, along with the excessive
 * deferral error bit set in the transmit frame status when transmit
 * state machine is deferred for more than
 * 	- 24,288 bit times in 10/100Mbps mode
 * 	- 155,680 bit times in 1000Mbps mode or Jumbo frame
 *	mode in 10/100Mbps operation.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 * @note Deferral begins when transmitter is ready to transmit,
 * but is prevented because  of
 * an active CRS (carrier sense)
 */
void nss_gmac_deferral_check_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacConfig, GmacDeferralCheck);
}

/*
 * Disables the Deferral check in GMAC (Only in Half Duplex mode).
 * GMAC defers until the CRS signal goes inactive.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_deferral_check_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacConfig, GmacDeferralCheck);
}

/*
 * Enable the reception of frames on GMII/MII.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_rx_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base, GmacConfig,
			      GmacRx);
}

/*
 * Disable the reception of frames on GMII/MII.
 * GMAC receive state machine is disabled after completion of reception of current frame.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_rx_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacConfig, GmacRx);

}

/*
 * Enable the transmission of frames on GMII/MII.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_tx_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base, GmacConfig,
			      GmacTx);
}

/*
 * Disable the transmission of frames on GMII/MII.
 * GMAC transmit state machine is disabled after completion of
 * transmission of current frame.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_tx_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacConfig, GmacTx);
}

/*Receive frame filter configuration functions*/

/*
 * Enables reception of all the frames to application.
 * GMAC passes all the frames received to application
 * irrespective of whether they pass SA/DA address filtering or not.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_frame_filter_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacFrameFilter, GmacFilter);
}

/*
 * Disables reception of all the frames to application.
 * GMAC passes only those received frames to application which
 * pass SA/DA address filtering.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_frame_filter_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacFrameFilter, GmacFilter);
}

/*
 * Populates the Hash High register with the data supplied.
 * This function is called when the Hash filtering is to be enabled.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] data to be written to hash table high register.
 * @return void.
 */
void nss_gmac_write_hash_table_high(nss_gmac_dev *gmacdev, uint32_t data)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base, GmacHashHigh, data);
}

/*
 * Populates the Hash Low register with the data supplied.
 * This function is called when the Hash filtering is to be enabled.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] data to be written to hash table low register.
 * @return void.
 */
void nss_gmac_write_hash_table_low(nss_gmac_dev *gmacdev, uint32_t data)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base, GmacHashLow, data);
}

/*
 * Enables Hash or Perfect filter (only if Hash filter is enabled in H/W).
 * Only frames matching either perfect filtering or Hash Filtering as per HMC and HUC
 * configuration are sent to application.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_hash_perfect_filter_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacFrameFilter, GmacHashPerfectFilter);
}

/*
 * Enables only Hash(only if Hash filter is enabled in H/W).
 * Only frames matching Hash Filtering as per HMC and HUC
 * configuration are sent to application.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_Hash_filter_only_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacFrameFilter, GmacHashPerfectFilter);
}

/*
 * Enables Source address filtering.
 * When enabled source address filtering is performed. Only frames matching SA filtering are passed  to application with
 * SAMatch bit of RxStatus is set. GMAC drops failed frames.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 * @note This function is overriden by nss_gmac_frame_filter_disable(nss_gmac_dev *)
 */
void nss_gmac_src_addr_filter_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacFrameFilter, GmacSrcAddrFilter);
}

/*
 * Disables Source address filtering.
 * When disabled GMAC forwards the received frames with updated
 * SAMatch bit in RxStatus.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_src_addr_filter_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacFrameFilter, GmacSrcAddrFilter);
}

/*
 * Enables Inverse Destination address filtering.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_dst_addr_filter_inverse(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacFrameFilter, GmacDestAddrFilterInv);
}

/*
 * Enables the normal Destination address filtering.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_dst_addr_filter_normal(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacFrameFilter, GmacDestAddrFilterInv);
}

/*
 * Enables forwarding of control frames.
 * When set forwards all the control frames
 * (incl. unicast and multicast PAUSE frames).
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] pass control.
 * @return void.
 * @note Depends on RFE of FlowControlRegister[2]
 */
void nss_gmac_set_pass_control(nss_gmac_dev *gmacdev, uint32_t passcontrol)
{
	uint32_t data;
	data =
	    nss_gmac_read_reg((uint32_t *)gmacdev->mac_base, GmacFrameFilter);
	data &= (~GmacPassControl);
	data |= passcontrol;
	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base, GmacFrameFilter,
			   data);
}

/*
 * Enables Broadcast frames.
 * When enabled Address filtering module passes all incoming broadcast frames.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_broadcast_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacFrameFilter, GmacBroadcast);
}

/*
 * Disable Broadcast frames.
 * When disabled Address filtering module filters all incoming broadcast frames.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_broadcast_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacFrameFilter, GmacBroadcast);
}

/*
 * Enables Multicast frames.
 * When enabled all multicast frames are passed.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_multicast_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacFrameFilter, GmacMulticastFilter);
}

/*
 * Disable Multicast frames.
 * When disabled multicast frame filtering depends on HMC bit.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_multicast_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacFrameFilter, GmacMulticastFilter);
}

/*
 * Enables multicast hash filtering.
 * When enabled GMAC performs teh destination address filtering according
 * to the hash table.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_multicast_hash_filter_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacFrameFilter, GmacMcastHashFilter);
}

/*
 * Disables multicast hash filtering.
 * When disabled GMAC performs perfect destination address filtering
 * for multicast frames, it compares DA field with the value programmed
 * in DA register.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_multicast_hash_filter_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacFrameFilter, GmacMcastHashFilter);
}

/*
 * Enables promiscous mode.
 * When enabled Address filter modules pass all incoming frames
 * regardless of their Destination and source addresses.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_promisc_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacFrameFilter, GmacPromiscuousMode);
}

/*
 * Clears promiscous mode.
 * When called the GMAC falls back to normal operation from promiscous mode.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_promisc_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacFrameFilter, GmacPromiscuousMode);
}

/*
 * Enables unicast hash filtering.
 * When enabled GMAC performs the destination address filtering of
 * unicast frames according to the hash table.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_unicast_hash_filter_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacFrameFilter, GmacUcastHashFilter);
}

/*
 * Disables multicast hash filtering.
 * When disabled GMAC performs perfect destination address filtering for unicast frames, it compares
 * DA field with the value programmed in DA register.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_unicast_hash_filter_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacFrameFilter, GmacUcastHashFilter);
}

/*Flow control configuration functions*/

/*
 * Enables detection of pause frames with stations unicast address.
 * When enabled GMAC detects the pause frames with stations unicast address in addition to the
 * detection of pause frames with unique multicast address.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_unicast_pause_frame_detect_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacFlowControl, GmacUnicastPauseFrame);
}

/*
 * Disables detection of pause frames with stations unicast address.
 * When disabled GMAC only detects with the unique multicast address (802.3x).
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_unicast_pause_frame_detect_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacFlowControl, GmacUnicastPauseFrame);
}

/*
 * Rx flow control enable.
 * When Enabled GMAC will decode the rx pause frame and disable the tx for a specified time.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_rx_flow_control_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacFlowControl, GmacRxFlowControl);
}

/*
 * Rx flow control disable.
 * When disabled GMAC will not decode pause frame.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_rx_flow_control_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacFlowControl, GmacRxFlowControl);
}

/*
 * Tx flow control enable.
 * When Enabled
 * - In full duplex GMAC enables flow control operation to
 *   transmit pause frames.
 * - In Half duplex GMAC enables the back pressure operation
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_tx_flow_control_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacFlowControl, GmacTxFlowControl);
}

/*
 * Tx flow control disable.
 * When Disabled
 * 	- In full duplex GMAC will not transmit any pause frames.
 *	- In Half duplex GMAC disables the back pressure feature.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_tx_flow_control_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacFlowControl, GmacTxFlowControl);
}


/*
 * This enables processing of received pause frame.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_tx_pause_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_info(gmacdev, "%s: enable Tx flow control", __FUNCTION__);

	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacFlowControl, GmacTxFlowControl);
}

/*
 * disable processing of received pause frame.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_tx_pause_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_info(gmacdev, "%s: disable Tx flow control", __FUNCTION__);

	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacFlowControl, GmacTxFlowControl);

}

/*
 * This enables pause frame generation after
 * programming the appropriate registers.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_rx_pause_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_info(gmacdev, "%s: enable Rx flow control", __FUNCTION__);

	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->dma_base, DmaControl,
				DmaEnHwFlowCtrl
				| DmaRxFlowCtrlAct3K | DmaRxFlowCtrlDeact4K);

	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacFlowControl, GmacRxFlowControl);
}

/*
 * Disable pause frame generation.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_rx_pause_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_info(gmacdev, "%s: disable Rx flow control", __FUNCTION__);

	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->dma_base,
				DmaControl, DmaEnHwFlowCtrl);

	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacFlowControl, GmacRxFlowControl);
}


/*
 * Flush Dma Tx fifo.
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_flush_tx_fifo(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->dma_base, DmaControl, DmaFlushTxFifo);
}

/*
 * Configure and set Tx/Rx flow control
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
void nss_gmac_config_flow_control(nss_gmac_dev *gmacdev)
{
	uint16_t phyreg;

	nss_gmac_info(gmacdev, "%s:", __FUNCTION__);

	if (gmacdev->pause == 0) {
		nss_gmac_rx_pause_disable(gmacdev);
		nss_gmac_tx_pause_disable(gmacdev);
		return;
	}

	phyreg = nss_gmac_mii_rd_reg(gmacdev, gmacdev->phy_base, MII_LPA);

	if (phyreg & LPA_PAUSE_CAP) {
		/* link partner can do Tx/Rx flow control */
		nss_gmac_info(gmacdev,
			      "%s: Link partner supports Tx/Rx flow control",
			      __FUNCTION__);

		if (gmacdev->pause & FLOW_CTRL_RX) {
			nss_gmac_rx_pause_enable(gmacdev);
		}

		if (gmacdev->pause & FLOW_CTRL_TX) {
			nss_gmac_tx_pause_enable(gmacdev);
		}

		return;
	}

	if (phyreg & LPA_PAUSE_ASYM) {
		/* link partner can do Rx flow control only */
		nss_gmac_info(gmacdev,
			      "%s: Link partner supports Rx flow control only",
			      __FUNCTION__);

		/* disable Rx flow control as link
		 * partner cannot process pause frames
		 */
		nss_gmac_rx_pause_disable(gmacdev);
		if (gmacdev->pause & FLOW_CTRL_TX) {
			nss_gmac_tx_pause_enable(gmacdev);
		}

		return;
	}

	/* link partner does not support Tx/Rx flow control */
	nss_gmac_info(gmacdev,
		      "%s: Link partner does not support Tx/Rx flow control",
		      __FUNCTION__);
	nss_gmac_rx_flow_control_disable(gmacdev);
	nss_gmac_tx_flow_control_disable(gmacdev);
}

/*
 * Initialize IPC Checksum offloading.
 * @param[in] pointer to nss_gmac_dev.
 * @return void
 */
void nss_gmac_ipc_offload_init(nss_gmac_dev *gmacdev)
{
	if (test_bit(__NSS_GMAC_RXCSUM, &gmacdev->flags)) {
		/* Enable the offload engine in the receive path */
		nss_gmac_enable_rx_chksum_offload(gmacdev);

		/*
		 * DMA drops the packets if error in encapsulated ethernet payload.
		 */
		nss_gmac_rx_tcpip_chksum_drop_enable(gmacdev);
		nss_gmac_info(gmacdev, "%s: enable Rx checksum", __FUNCTION__);
	} else {
		nss_gmac_disable_rx_chksum_offload(gmacdev);
		nss_gmac_info(gmacdev, "%s: disable Rx checksum", __FUNCTION__);
	}
}


/*
 * Mac initialization sequence.
 * This function calls the initialization routines
 * to initialize the GMAC register.
 * @param[in] pointer to nss_gmac_dev.
 * @return void
 */
void nss_gmac_mac_init(nss_gmac_dev *gmacdev)
{
	nss_gmac_wd_enable(gmacdev);
	nss_gmac_jab_enable(gmacdev);
	nss_gmac_frame_burst_enable(gmacdev);
	nss_gmac_loopback_off(gmacdev);

	if (gmacdev->speed == SPEED1000) {
		nss_gmac_select_gmii(gmacdev);
	} else {
		nss_gmac_select_mii(gmacdev);
	}

	if (gmacdev->duplex_mode == FULLDUPLEX) {
		nss_gmac_set_full_duplex(gmacdev);
		nss_gmac_rx_own_enable(gmacdev);
		nss_gmac_retry_disable(gmacdev);
	} else {
		nss_gmac_set_half_duplex(gmacdev);
		nss_gmac_rx_own_disable(gmacdev);
		nss_gmac_retry_enable(gmacdev);
	}

	nss_gmac_pad_crc_strip_disable(gmacdev);
	nss_gmac_back_off_limit(gmacdev, GmacBackoffLimit0);
	nss_gmac_deferral_check_disable(gmacdev);

	nss_gmac_set_mac_addr(gmacdev, GmacAddr0High,
			      GmacAddr0Low, gmacdev->netdev->dev_addr);

	/*Frame Filter Configuration */
	nss_gmac_frame_filter_enable(gmacdev);
	nss_gmac_set_pass_control(gmacdev, GmacPassControl0);
	nss_gmac_broadcast_enable(gmacdev);
	nss_gmac_src_addr_filter_disable(gmacdev);
	nss_gmac_multicast_enable(gmacdev);
	gmacdev->netdev->flags |= IFF_ALLMULTI;
	nss_gmac_dst_addr_filter_normal(gmacdev);
	nss_gmac_multicast_hash_filter_disable(gmacdev);
	nss_gmac_promisc_enable(gmacdev);
	nss_gmac_unicast_hash_filter_disable(gmacdev);

	nss_gmac_ipc_offload_init(gmacdev);

	/* Flow Control Configuration */
	nss_gmac_unicast_pause_frame_detect_disable(gmacdev);
	nss_gmac_config_flow_control(gmacdev);

	nss_gmac_tx_enable(gmacdev);
	nss_gmac_rx_enable(gmacdev);
}


static void nss_gmac_check_pcs_status(nss_gmac_dev *gmacdev)
{
	struct nss_gmac_global_ctx *ctx = NULL;
	uint32_t *qsgmii_base = NULL;
	uint32_t id = 0;
	uint32_t reg = 0;

	ctx = gmacdev->ctx;
	qsgmii_base = ctx->qsgmii_base;
	id = gmacdev->macid;

	gmacdev->link_state = LINKDOWN;

	/* confirm link is up in PCS_QSGMII_MAC_STATUS register */
	reg = nss_gmac_read_reg(qsgmii_base, PCS_QSGMII_MAC_STAT);
	if (!(reg & PCS_MAC_STAT_CHn_LINK(id))) {
		return;
	}

	gmacdev->link_state = LINKUP;

	/* save duplexity */
	if (reg & PCS_MAC_STAT_CHn_DUPLEX(id)) {
		gmacdev->duplex_mode = FULLDUPLEX;
	} else {
		gmacdev->duplex_mode = HALFDUPLEX;
	}

	/* save speed */
	switch (PCS_MAC_STAT_CHn_SPEED(id, reg)) {
	case 0:
		gmacdev->speed = SPEED10;
		break;

	case 1:
		gmacdev->speed = SPEED100;
		break;

	case 2:
		gmacdev->speed = SPEED1000;
		break;
	}
}


/*
 * Handle Q/SGMII linkup
 * @param[in] pointer to nss_gmac_dev.
 * @return void.
 */
static void nss_gmac_check_sgmii_link(nss_gmac_dev *gmacdev)
{
	struct nss_gmac_global_ctx *ctx = NULL;
	uint32_t *qsgmii_base = NULL;
	uint32_t id = 0;
	uint32_t reg = 0;
	uint32_t previous_linkup_duplex = 0;
	uint32_t previous_linkup_speed = 0;
	uint32_t new_duplex = 0;
	uint32_t new_speed = 0;
	int32_t timeout = 0;
	int32_t timeout_count = 0;

	ctx = gmacdev->ctx;
	qsgmii_base = ctx->qsgmii_base;
	id = gmacdev->macid;

	previous_linkup_speed = gmacdev->speed;
	previous_linkup_duplex = gmacdev->duplex_mode;

reheck_pcs_mac_status:
	nss_gmac_check_pcs_status(gmacdev);
	if (gmacdev->link_state == LINKDOWN) {
		if (gmacdev->phydev->link) {
			nss_gmac_warn(gmacdev, "SGMII PCS error. Resetting PHY using MDIO");
			phy_write(gmacdev->phydev, MII_BMCR,
				  BMCR_RESET | phy_read(gmacdev->phydev, MII_BMCR));
		}

		return;
	}

	new_speed = gmacdev->speed;
	new_duplex = gmacdev->duplex_mode;

	/* reinitiate autoneg in QSGMII CSR. */
	nss_gmac_set_reg_bits(qsgmii_base, PCS_MODE_CTL,
				PCS_MODE_CTL_CHn_AUTONEG_RESTART(id));
	nss_gmac_clear_reg_bits(qsgmii_base, PCS_MODE_CTL,
				PCS_MODE_CTL_CHn_AUTONEG_RESTART(id));
	timeout = 50;
	reg = nss_gmac_read_reg(qsgmii_base, PCS_ALL_CH_STAT);
	while(!(reg & PCS_CHn_AUTONEG_COMPLETE(id)) && timeout > 0) {
		timeout--;
		mdelay(10);
		reg = nss_gmac_read_reg(qsgmii_base, PCS_ALL_CH_STAT);
	}

	/* handle autoneg timeout */
	if (timeout == 0) {
		nss_gmac_info(gmacdev, "%s: PCS ch %d autoneg timeout", __FUNCTION__, id);
		timeout_count++;
		if (timeout_count == 2) {
			gmacdev->link_state = LINKDOWN;
			nss_gmac_set_reg_bits(qsgmii_base, PCS_MODE_CTL,
					      PCS_MODE_CTL_CHn_PHY_RESET(id));
			return;
		}
		goto reheck_pcs_mac_status;
	}
	nss_gmac_trace(gmacdev, "%s: PCS ch %d autoneg complete", __FUNCTION__, id);

	nss_gmac_check_pcs_status(gmacdev);

	if ((gmacdev->link_state == LINKDOWN) || (new_speed != gmacdev->speed)) {
		gmacdev->link_state = LINKDOWN;
			nss_gmac_warn(gmacdev, "SGMII PCS error. Resetting PHY using MDIO");
			phy_write(gmacdev->phydev, MII_BMCR,
				  BMCR_RESET | phy_read(gmacdev->phydev, MII_BMCR));
		return;
	}

	/* check if initial speed has changed */
	if (previous_linkup_speed != gmacdev->speed) {
		/* switch clock dividers */
		nss_gmac_dev_set_speed(gmacdev);

		/* flush GMAC fifo */
		nss_gmac_flush_tx_fifo(gmacdev);
	}
}


/*
 * This function checks to see if phy PHY autonegotiation is complete.
 * It reads PHY registers to retrieve current speed and duplexity settings.
 * @param[in] pointer to nss_gmac_dev.
 * @return 0 on success. If successful, it updates gmacdev->speed and
 * 	   gmacdev->duplex_mode with current speed and duplex mode.
 */
int32_t nss_gmac_check_phy_init(nss_gmac_dev *gmacdev)
{
	struct phy_device *phydev = NULL;
	int32_t count;

	/*
	 * If link polling is disabled, we need to use the forced speed
	 * and duplex configured for the interface.
	 */
	if (!test_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags)
					&& !gmacdev->emulation) {
		if (gmacdev->forced_speed != SPEED_UNKNOWN) {
			gmacdev->speed = gmacdev->forced_speed;
			gmacdev->duplex_mode = gmacdev->forced_duplex;
			return 0;
		} else {
			nss_gmac_info(gmacdev, "%s: Invalid forced speed/duplex configuration with link polling disabled", __FUNCTION__);
			return -1;
		}
	}

	if (gmacdev->emulation && (gmacdev->phy_mii_type == GMAC_INTF_SGMII
			|| gmacdev->phy_mii_type == GMAC_INTF_QSGMII)) {
		/* Emulation build, Q/SGMII interface. Returning 100Mbps FD */
		gmacdev->speed = SPEED100;
		gmacdev->duplex_mode = FULLDUPLEX;
		goto out;
	}

	if (gmacdev->phy_mii_type == GMAC_INTF_SGMII
		|| gmacdev->phy_mii_type == GMAC_INTF_QSGMII) {
		nss_gmac_check_sgmii_link(gmacdev);
		if (gmacdev->link_state == LINKDOWN) {
			nss_gmac_info(gmacdev, "%s: SGMII phy linkup ERROR.", __FUNCTION__);
			return -EIO;
		}

		nss_gmac_trace(gmacdev, "%s: SGMII phy linkup OK.", __FUNCTION__);
		goto out;
	}

	/*
	 * Read the link status from the PHY for RGMII interfaces
	 * with link polling enabled.
	 */
	phydev = gmacdev->phydev;

	for (count = 0; count < DEFAULT_LOOP_VARIABLE; count++) {
		if (phydev->state == PHY_RUNNING) {
			nss_gmac_info(gmacdev, "%s: %s Autoneg. complete",
				      __FUNCTION__, gmacdev->netdev->name);
			break;
		}
	}

	if (count == DEFAULT_LOOP_VARIABLE) {
		nss_gmac_info(gmacdev, "%s: %s Timeout waiting for autoneg.",
			      __FUNCTION__, gmacdev->netdev->name);
		return -EIO;
	}

	genphy_read_status(phydev);

	switch (phydev->speed) {
	case SPEED_10:
		gmacdev->speed = SPEED10;
		break;

	case SPEED_100:
		gmacdev->speed = SPEED100;
		break;

	case SPEED_1000:
		gmacdev->speed = SPEED1000;
		break;
	}

	switch (phydev->duplex) {
	case DUPLEX_HALF:
		gmacdev->duplex_mode = HALFDUPLEX;
		break;

	case DUPLEX_FULL:
		gmacdev->duplex_mode = FULLDUPLEX;
		break;
	}

out:
	nss_gmac_msg("%s %sMbps %sDuplex",
			gmacdev->netdev->name, (gmacdev->speed == SPEED1000) ?
			"1000" : ((gmacdev->speed == SPEED100) ? "100" : "10"),
			(gmacdev->duplex_mode == FULLDUPLEX) ? "Full" : "Half");

	/*
	 * We may want to force speed and duplex settings even after link
	 * polling. This may be for a GMAC connected to a switch where the
	 * parameters of link between GAMC and switch are forced.
	 */
	if (gmacdev->forced_speed != SPEED_UNKNOWN) {
		gmacdev->speed = gmacdev->forced_speed;
		gmacdev->duplex_mode = gmacdev->forced_duplex;
	}

	return 0;
}

/*
 * Write a MDIO Manageable Device(MMD) register of a Phy.
 * @phydev[in] pointer to struct phy_device
 * @mmd_dev_addr[in] MMD device address
 * @reg[in] register offset
 * @val[in] value to be written
 * @return 0 on success
 */
int32_t nss_gmac_ath_phy_mmd_wr(struct phy_device *phydev, uint32_t mmd_dev_addr,
			uint32_t reg, uint16_t val)
{
	if(IS_ERR_OR_NULL(phydev)) {
		return -EINVAL;
	}

	phy_write(phydev, ATH_MII_MMD_ACCESS_CTRL, mmd_dev_addr);
	phy_write(phydev, ATH_MII_MMD_ACCESS_ADDR_DATA, reg);
	phy_write(phydev, ATH_MII_MMD_ACCESS_CTRL,
		  ath_mmd_acc_ctrl_data_no_incr | mmd_dev_addr);
	phy_write(phydev, ATH_MII_MMD_ACCESS_ADDR_DATA, val);

	return 0;
}

/*
 * Read a MDIO Manageable Device(MMD) register form a Phy.
 * @phydev[in] pointer to struct phy_device
 * @mmd_dev_addr[in] MMD device address
 * @reg[in] register offset
 * @return -EINVAL on failure.
 * 	   Register value on success.
 */
int32_t nss_gmac_ath_phy_mmd_rd(struct phy_device *phydev,
			uint32_t mmd_dev_addr, uint32_t reg)
{
	if(IS_ERR_OR_NULL(phydev)) {
		return -EINVAL;
	}

	phy_write(phydev, ATH_MII_MMD_ACCESS_CTRL, mmd_dev_addr);
	phy_write(phydev, ATH_MII_MMD_ACCESS_ADDR_DATA, reg);
	phy_write(phydev, ATH_MII_MMD_ACCESS_CTRL,
		  ath_mmd_acc_ctrl_data_no_incr | mmd_dev_addr);
	return phy_read(phydev, ATH_MII_MMD_ACCESS_ADDR_DATA);
}

/*
 * Disable QCA Smart Energy Efficient Ethernet on a Phy.
 * @phydev[in] pointer to struct phy_device
 * @return 0 on success.
 */
int32_t nss_gmac_ath_phy_disable_smart_802az(struct phy_device *phydev)
{
	uint16_t val = 0;

	if(IS_ERR_OR_NULL(phydev)) {
		return -EINVAL;
	}

	val = nss_gmac_ath_phy_mmd_rd(phydev, ATH_MMD_DEVADDR_3, ath_mmd_smart_eee_ctrl_3);
	val &= ~ath_mmd_smart_eee_ctrl3_lpi_en;
	nss_gmac_ath_phy_mmd_wr(phydev, ATH_MMD_DEVADDR_3, ath_mmd_smart_eee_ctrl_3, val);

	return 0;
}

/*
 * Disable Energy Efficient Ethernet (IEEE 802.3az) on a Phy.
 * @phydev[in] pointer to struct phy_device
 * @return 0 on success.
 */
int32_t nss_gmac_ath_phy_disable_802az(struct phy_device *phydev)
{
	uint16_t val = 0;

	if(IS_ERR_OR_NULL(phydev)) {
		return -EINVAL;
	}

	val = nss_gmac_ath_phy_mmd_rd(phydev, ATH_MMD_DEVADDR_7, ath_mmd_eee_adv);
	val &= ~(ath_mmd_eee_adv_100BT | ath_mmd_eee_adv_1000BT);
	nss_gmac_ath_phy_mmd_wr(phydev, ATH_MMD_DEVADDR_7, ath_mmd_eee_adv, val);

	return 0;
}

/*
 * Sets the Mac address in to GMAC register.
 * This function sets the MAC address to the MAC register in question.
 * @param[in] pointer to nss_gmac_dev to populate mac dma and phy addresses.
 * @param[in] Register offset for Mac address high
 * @param[in] Register offset for Mac address low
 * @param[in] buffer containing mac address to be programmed.
 * @return void
 */
void nss_gmac_set_mac_addr(nss_gmac_dev *gmacdev, uint32_t MacHigh,
			      uint32_t MacLow, uint8_t *MacAddr)
{
	uint32_t data;

	nss_gmac_info(gmacdev, "Set addr %02x:%02x:%02x:%02x:%02x:%02x",
		      MacAddr[0], MacAddr[1], MacAddr[2],
		      MacAddr[3], MacAddr[4], MacAddr[5]);

	data = (MacAddr[5] << 8) | MacAddr[4] | 0x80000000;
	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base, MacHigh, data);
	data = (MacAddr[3] << 24) | (MacAddr[2] << 16)
	    | (MacAddr[1] << 8) | MacAddr[0];
	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base, MacLow, data);
}

/*
 * Get the Mac address in to the address specified.
 * The mac register contents are read and written to buffer passed.
 * @param[in] pointer to nss_gmac_dev to populate mac dma and phy addresses.
 * @param[in] Register offset for Mac address high
 * @param[in] Register offset for Mac address low
 * @param[out] buffer containing the device mac address.
 * @return void
 */
void nss_gmac_get_mac_addr(nss_gmac_dev *gmacdev, uint32_t MacHigh,
			      uint32_t MacLow, uint8_t *MacAddr)
{
	uint32_t data;

	data = nss_gmac_read_reg((uint32_t *)gmacdev->mac_base, MacHigh);
	MacAddr[5] = (data >> 8) & 0xff;
	MacAddr[4] = (data) & 0xff;

	data = nss_gmac_read_reg((uint32_t *)gmacdev->mac_base, MacLow);
	MacAddr[3] = (data >> 24) & 0xff;
	MacAddr[2] = (data >> 16) & 0xff;
	MacAddr[1] = (data >> 8) & 0xff;
	MacAddr[0] = (data) & 0xff;
}

/*
 * Attaches the NSS GMAC device structure to the hardware.
 * Device structure is populated with MAC/DMA and PHY base addresses.
 * @param[in] pointer to nss_gmac_dev to populate mac dma and phy addresses.
 * @param[in] GMAC IP register base address.
 * @param[in] GMAC IP register length.
 * @return 0 upon success. Error code upon failure.
 * @note This is important function.
 */
int32_t nss_gmac_attach(nss_gmac_dev *gmacdev,
			uint32_t regBase, uint32_t reglen)
{
	struct net_device *netdev = NULL;
	netdev = gmacdev->netdev;

	/*Populate the mac and dma base addresses */
	gmacdev->memres = request_mem_region(regBase, reglen, netdev->name);
	if (!gmacdev->memres) {
		nss_gmac_info(gmacdev, "Unable to request resource.");
		return -EIO;
	}

	/* ioremap addresses */
	gmacdev->mac_base = (uint32_t)ioremap_nocache(regBase,
						      NSS_GMAC_REG_BLOCK_LEN);
	if (!gmacdev->mac_base) {
		nss_gmac_info(gmacdev, "ioremap fail.");
		return -EIO;
	}

	nss_gmac_info(gmacdev, "ioremap OK. Size 0x%x. "
		      "regBase 0x%x. mac_base 0x%x.",
		      NSS_GMAC_REG_BLOCK_LEN, regBase, gmacdev->mac_base);

	gmacdev->dma_base = gmacdev->mac_base + NSS_GMAC_DMABASE;

	return 0;
}

/**
 * Detaches the NSS GMAC device structure from hardware.
 * MAC/DMA base addresses are freed from device structure.
 * @param[in] pointer to nss_gmac_dev to populate mac dma and phy addresses.
 * @return void
 * @note This is important function.
 */
void nss_gmac_detach(nss_gmac_dev *gmacdev)
{
	uint32_t reglen;

	reglen = gmacdev->memres->end - gmacdev->memres->start + 1;
	iounmap((void *)gmacdev->mac_base);
	release_mem_region((gmacdev->memres)->start, reglen);

	gmacdev->memres = NULL;

	gmacdev->mac_base = 0;
	gmacdev->dma_base = 0;
}


/*
 * Programs the DmaRxBaseAddress with the Rx descriptor base address.
 * Rx Descriptor's base address is available in the gmacdev structure.
 * This function progrms the Dma Rx Base address with the starting address
 * of the descriptor ring or chain.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_init_rx_desc_base(nss_gmac_dev *gmacdev)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base,
			   DmaRxBaseAddr, (uint32_t)gmacdev->rx_desc_dma);
}

/*
 * Programs the DmaTxBaseAddress with the Tx descriptor base address.
 * Tx Descriptor's base address is available in the gmacdev structure.
 * This function progrms the Dma Tx Base address with the starting
 * address of the descriptor ring or chain.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_init_tx_desc_base(nss_gmac_dev *gmacdev)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base,
			   DmaTxBaseAddr, (uint32_t)gmacdev->tx_desc_dma);
}

/*
 * Makes the Dma as owner for this descriptor.
 * This function sets the own bit of status field of the DMA descriptor,
 * indicating the DMA is the owner for this descriptor.
 * @param[in] pointer to DmaDesc structure.
 * @return returns void.
 */
void nss_gmac_set_owner_dma(DmaDesc * desc)
{
	desc->status |= DescOwnByDma;
}

/*
 * set tx descriptor to indicate SOF.
 * This Descriptor contains the start of ethernet frame.
 * @param[in] pointer to DmaDesc structure.
 * @return returns void.
 */
void nss_gmac_set_desc_sof(DmaDesc * desc)
{
	desc->status |= DescTxFirst;
}

/*
 * set tx descriptor to indicate EOF.
 * This descriptor contains the End of ethernet frame.
 * @param[in] pointer to DmaDesc structure.
 * @return returns void.
 */
void nss_gmac_set_desc_eof(DmaDesc * desc)
{
	desc->status |= DescTxLast;
}

/*
 * checks whether this descriptor contains start of frame.
 * This function is to check whether the descriptor's data buffer
 * contains a fresh ethernet frame?
 * @param[in] pointer to DmaDesc structure.
 * @return returns true if SOF in current descriptor, else returns fail.
 */
bool nss_gmac_is_sof_in_rx_desc(DmaDesc * desc)
{
	return ((desc->status & DescRxFirst) == DescRxFirst);
}

/*
 * checks whether this descriptor contains end of frame.
 * This function is to check whether the descriptor's data buffer
 * contains end of ethernet frame?
 * @param[in] pointer to DmaDesc structure.
 * @return returns true if SOF in current descriptor, else returns fail.
 */
bool nss_gmac_is_eof_in_rx_desc(DmaDesc * desc)
{
	return ((desc->status & DescRxLast) == DescRxLast);
}

/*
 * checks whether destination address filter failed in the rx frame.
 * @param[in] pointer to DmaDesc structure.
 * @return returns true if Failed, false if not.
 */
bool nss_gmac_is_da_filter_failed(DmaDesc * desc)
{
	return ((desc->status & DescDAFilterFail) == DescDAFilterFail);
}

/*
 * checks whether source address filter failed in the rx frame.
 * @param[in] pointer to DmaDesc structure.
 * @return returns true if Failed, false if not.
 */
bool nss_gmac_is_sa_filter_failed(DmaDesc * desc)
{
	return ((desc->status & DescSAFilterFail) == DescSAFilterFail);
}

/*
 * Checks whether the tx is aborted due to collisions.
 * @param[in] pointer to DmaDesc structure.
 * @return returns true if collisions, else returns false.
 */
bool nss_gmac_is_tx_aborted(uint32_t status)
{
	return (((status & DescTxLateCollision) == DescTxLateCollision)
		|| ((status & DescTxExcCollisions) == DescTxExcCollisions));

}

/*
 * Checks whether the tx carrier error.
 * @param[in] Tx completion status.
 * @return returns true if carrier error occured, else returns false.
 */
bool nss_gmac_is_tx_carrier_error(uint32_t status)
{
	return (((status & DescTxLostCarrier) == DescTxLostCarrier)
		|| ((status & DescTxNoCarrier) == DescTxNoCarrier));
}

/*
 * Checks whether for tx underflow.
 * @param[in] Tx completion status.
 * @return returns true if tx underflow occured, else returns false.
 */
bool nss_gmac_is_tx_underflow_error(uint32_t status)
{
	return ((status & DescTxUnderflow) == DescTxUnderflow);
}

/*
 * Checks whether for tx late collision.
 * @param[in] Tx completion status.
 * @return returns true if tx late collision occured, else returns false.
 */
bool nss_gmac_is_tx_lc_error(uint32_t status)
{
	return ((status & DescTxLateCollision) == DescTxLateCollision);
}

/*
 * Check for damaged frame due to overflow or collision.
 * Retruns true if rx frame was damaged due to buffer overflow
 * in MTL or late collision in half duplex mode.
 * @param[in] pointer to DmaDesc structure.
 * @return returns true if error else returns false.
 */
bool nss_gmac_is_rx_frame_damaged(uint32_t status)
{
	return (((status & DescRxDamaged) == DescRxDamaged)
		|| ((status & DescRxCollision) == DescRxCollision));
}

/*
 * Check for damaged frame due to collision.
 * Retruns true if rx frame was damaged due to late collision
 * in half duplex mode.
 * @param[in] pointer to DmaDesc structure.
 * @return returns true if error else returns false.
 */
bool nss_gmac_is_rx_frame_collision(uint32_t status)
{
	return ((status & DescRxCollision) == DescRxCollision);
}

/*
 * Check for receive CRC error.
 * Retruns true if rx frame CRC error occured.
 * @param[in] pointer to DmaDesc structure.
 * @return returns true if error else returns false.
 */
bool nss_gmac_is_rx_crc(uint32_t status)
{
	return ((status & DescRxCrc) == DescRxCrc);
}

/*
 * Indicates rx frame has non integer multiple of bytes. (odd nibbles).
 * Retruns true if dribbling error in rx frame.
 * @param[in] pointer to DmaDesc structure.
 * @return returns true if error else returns false.
 */
bool nss_gmac_is_frame_dribbling_errors(uint32_t status)
{
	return ((status & DescRxDribbling) == DescRxDribbling);
}

/*
 * Indicates error in rx frame length.
 * Retruns true if received frame length doesnot match with the length field
 * @param[in] pointer to DmaDesc structure.
 * @return returns true if error else returns false.
 */
bool nss_gmac_is_rx_frame_length_errors(uint32_t status)
{
	return ((status & DescRxLengthError) == DescRxLengthError);
}

/*
 * Driver Api to get the descriptor field information.
 * This returns the status, dma-able address of buffer1, the length of
 * buffer1, virtual address of buffer1 dma-able address of buffer2, length
 * of buffer2, virtural adddress of buffer2.
 * @param[in]  pointer to DmaDesc structure.
 * @param[out] pointer to status field fo descriptor.
 * @param[out] dma-able address of buffer1.
 * @param[out] length of buffer1.
 * @param[out] virtual address of buffer1.
 * @return returns void.
 */
void nss_gmac_get_desc_data(DmaDesc * desc,
			    uint32_t *Status, uint32_t *Buffer1,
			    uint32_t *Length1, uint32_t *Data1)
{
	/*
	 * The first time, we map the descriptor as DMA_TO_DEVICE.
	 * Then we only wait for changes from device, so we use DMA_FROM_DEVICE.
	 */
	if (Status != 0) {
		*Status = desc->status;
	}

	if (Buffer1 != 0) {
		*Buffer1 = desc->buffer1;
	}

	if (Length1 != 0) {
		*Length1 = (desc->length & DescSize1Mask) >> DescSize1Shift;
	}

	if (Data1 != 0) {
		*Data1 = desc->data1;
	}
}

/*
 * Enable the DMA Reception.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_enable_dma_rx(nss_gmac_dev *gmacdev)
{
	uint32_t data;
	data = nss_gmac_read_reg((uint32_t *)gmacdev->dma_base, DmaControl);
	data |= DmaRxStart;
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base, DmaControl, data);
}

/*
 * Enable the DMA Transmission.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_enable_dma_tx(nss_gmac_dev *gmacdev)
{
	uint32_t data;
	data = nss_gmac_read_reg((uint32_t *)gmacdev->dma_base, DmaControl);
	data |= DmaTxStart;
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base, DmaControl, data);
}

/*
 * Take ownership of this Descriptor.
 * The function is same for both the ring mode and
 * the chain mode DMA structures.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_take_desc_ownership(DmaDesc * desc)
{
	if (desc) {
		/* Clear the DMA own bit */
		desc->status &= ~DescOwnByDma;
	}
}

/*
 * Take ownership of all the rx Descriptors.
 * This function is called when there is fatal error in DMA transmission.
 * When called it takes the ownership of all the rx descriptor in rx
 * descriptor pool/queue from DMA. The function is same for both the ring
 * mode and the chain mode DMA structures.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 * @note Make sure to disable the transmission before calling this function,
 * otherwise may result in racing situation.
 */
void nss_gmac_take_desc_ownership_rx(nss_gmac_dev *gmacdev)
{
	int32_t i;
	DmaDesc *desc;
	desc = gmacdev->rx_desc;
	for (i = 0; i < gmacdev->rx_desc_count; i++) {
		nss_gmac_take_desc_ownership(desc + i);
	}
}

/*
 * Take ownership of all the rx Descriptors.
 * This function is called when there is fatal error in DMA transmission.
 * When called it takes the ownership of all the tx descriptor in
 * tx descriptor pool/queue from DMA. The function is same for both the
 * ring mode and the chain mode DMA structures.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 * @note Make sure to disable the transmission before calling this function,
 * otherwise may result in racing situation.
 */
void nss_gmac_take_desc_ownership_tx(nss_gmac_dev *gmacdev)
{
	int32_t i;
	DmaDesc *desc;
	desc = gmacdev->tx_desc;
	for (i = 0; i < gmacdev->tx_desc_count; i++) {
		nss_gmac_take_desc_ownership(desc + i);
	}
}

/*
 * Disable the DMA for Transmission.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_disable_dma_tx(nss_gmac_dev *gmacdev)
{
	uint32_t data;

	data = nss_gmac_read_reg((uint32_t *)gmacdev->dma_base, DmaControl);
	data &= (~DmaTxStart);
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base, DmaControl, data);
}

/*
 * Disable the DMA for Reception.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_disable_dma_rx(nss_gmac_dev *gmacdev)
{
	uint32_t data;

	data = nss_gmac_read_reg((uint32_t *)gmacdev->dma_base, DmaControl);
	data &= (~DmaRxStart);
	nss_gmac_write_reg((uint32_t *)gmacdev->dma_base, DmaControl, data);
}

/*******************PMT APIs***********************/

/*
 * Enables the assertion of PMT interrupt.
 * This enables the assertion of PMT interrupt due to
 * Magic Pkt or Wakeup frame reception.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_pmt_int_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacInterruptMask, GmacPmtIntMask);
}

/*
 * Disables the assertion of PMT interrupt.
 * This disables the assertion of PMT interrupt due to
 * Magic Pkt or Wakeup frame reception.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_pmt_int_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacInterruptMask, GmacPmtIntMask);
}

/*
 * Enables the power down mode of GMAC.
 * This function puts the Gmac in power down mode.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_power_down_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacPmtCtrlStatus, GmacPmtPowerDown);
}

/*
 * Disables the powerd down setting of GMAC.
 * If the driver wants to bring up the GMAC from powerdown mode,
 * even though the magic packet or the wake up frames received from the
 * network, this function should be called.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_power_down_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacPmtCtrlStatus, GmacPmtPowerDown);
}

/*
 * Enables GMAC to look for Magic packet.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_magic_packet_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacPmtCtrlStatus, GmacPmtMagicPktEnable);
}

/*
 * Enables GMAC to look for wake up frame.
 * Wake up frame is defined by the user.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_wakeup_frame_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacPmtCtrlStatus, GmacPmtWakeupFrameEnable);
}

/*
 * Enables wake-up frame filter to handle unicast packets.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_pmt_unicast_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacPmtCtrlStatus, GmacPmtGlobalUnicast);
}

/*
 * Checks whether the packet received is a magic packet?.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns True if magic packet received else returns false.
 */
bool nss_gmac_is_magic_packet_received(nss_gmac_dev *gmacdev)
{
	uint32_t data;
	data = nss_gmac_read_reg((uint32_t *)gmacdev->mac_base,
				 GmacPmtCtrlStatus);
	return ((data & GmacPmtMagicPktReceived) == GmacPmtMagicPktReceived);
}

/*
 * Checks whether the packet received is a wakeup frame?.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns true if wakeup frame received else returns false.
 */
bool nss_gmac_is_wakeup_frame_received(nss_gmac_dev *gmacdev)
{
	uint32_t data;
	data = nss_gmac_read_reg((uint32_t *)gmacdev->mac_base,
				 GmacPmtCtrlStatus);
	return ((data & GmacPmtWakeupFrameReceived)
		== GmacPmtWakeupFrameReceived);
}

/*
 * Populates the remote wakeup frame registers.
 * Consecutive 8 writes to GmacWakeupAddr writes the wakeup frame
 * filter registers.
 * Before commensing a new write, frame filter pointer is reset to 0x0000.
 * A small delay is introduced to allow frame filter pointer reset operation.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] pointer to frame filter contents array.
 * @return returns void.
 */
void nss_gmac_write_wakeup_frame_register(nss_gmac_dev *gmacdev,
					  uint32_t *filter_contents)
{
	int32_t i;

	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacPmtCtrlStatus, GmacPmtFrmFilterPtrReset);
	mdelay(10);
	for (i = 0; i < WAKEUP_REG_LENGTH; i++) {
		nss_gmac_write_reg((uint32_t *)gmacdev->mac_base,
				   GmacWakeupAddr, *(filter_contents + i));
	}
}

/*******************PMT APIs****************************/

/*******************MMC APIs****************************/

/*
 * Freezes the MMC counters.
 * This function call freezes the MMC counters. None of the MMC counters
 *  are updated due to any tx or rx frames until nss_gmac_mmc_counters_resume
 * is called.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_mmc_counters_stop(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacMmcCntrl, GmacMmcCounterFreeze);
}

/*
 * Resumes the MMC counter updation.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_mmc_counters_resume(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacMmcCntrl, GmacMmcCounterFreeze);
}

/*
 * Configures the MMC in Self clearing mode.
 * Programs MMC interface so that counters are cleared when the
 * counters are read.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_mmc_counters_set_selfclear(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacMmcCntrl, GmacMmcCounterResetOnRead);
}

/*
 * Configures the MMC in non-Self clearing mode.
 * Programs MMC interface so that counters are cleared when the
 * counters are read.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_mmc_counters_reset_selfclear(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacMmcCntrl, GmacMmcCounterResetOnRead);
}

/*
 * Configures the MMC to stop rollover.
 * Programs MMC interface so that counters will not rollover
 * after reaching maximum value.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_mmc_counters_disable_rollover(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacMmcCntrl, GmacMmcCounterStopRollover);
}

/*
 * Configures the MMC to rollover.
 * Programs MMC interface so that counters will rollover after
 * reaching maximum value.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_mmc_counters_enable_rollover(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacMmcCntrl, GmacMmcCounterStopRollover);
}

/*
 * Read the MMC Counter.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] the counter to be read.
 * @return returns the read count value.
 */
uint32_t nss_gmac_read_mmc_counter(nss_gmac_dev *gmacdev, uint32_t counter)
{
	return (nss_gmac_read_reg((uint32_t *)gmacdev->mac_base, counter));
}

/*
 * Read the MMC Rx interrupt status.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns the Rx interrupt status.
 */
uint32_t nss_gmac_read_mmc_rx_int_status(nss_gmac_dev *gmacdev)
{
	return (nss_gmac_read_reg
		((uint32_t *)gmacdev->mac_base, GmacMmcIntrRx));
}

/*
 * Read the MMC Tx interrupt status.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns the Tx interrupt status.
 */
uint32_t nss_gmac_read_mmc_tx_int_status(nss_gmac_dev *gmacdev)
{
	return (nss_gmac_read_reg
		((uint32_t *)gmacdev->mac_base, GmacMmcIntrTx));
}

/*
 * Disable the MMC Tx interrupt.
 * The MMC tx interrupts are masked out as per the mask specified.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] tx interrupt bit mask for which interrupts needs to be disabled.
 * @return returns void.
 */
void nss_gmac_disable_mmc_tx_interrupt(nss_gmac_dev *gmacdev, uint32_t mask)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacMmcIntrMaskTx, mask);
}

/*
 * Enable the MMC Tx interrupt.
 * The MMC tx interrupts are enabled as per the mask specified.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] tx interrupt bit mask for which interrupts needs to be enabled.
 * @return returns void.
 */
void nss_gmac_enable_mmc_tx_interrupt(nss_gmac_dev *gmacdev, uint32_t mask)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacMmcIntrMaskTx, mask);
}

/*
 * Disable the MMC Rx interrupt.
 * The MMC rx interrupts are masked out as per the mask specified.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] rx interrupt bit mask for which interrupts needs to be disabled.
 * @return returns void.
 */
void nss_gmac_disable_mmc_rx_interrupt(nss_gmac_dev *gmacdev, uint32_t mask)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacMmcIntrMaskRx, mask);
}

/*
 * Enable the MMC Rx interrupt.
 * The MMC rx interrupts are enabled as per the mask specified.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] rx interrupt bit mask for which interrupts needs to be enabled.
 * @return returns void.
 */
void nss_gmac_enable_mmc_rx_interrupt(nss_gmac_dev *gmacdev, uint32_t mask)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacMmcIntrMaskRx, mask);
}

/*
 * Disable the MMC ipc rx checksum offload interrupt.
 * The MMC ipc rx checksum offload interrupts are masked out as
 * per the mask specified.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] rx interrupt bit mask for which interrupts needs to be disabled.
 * @return returns void.
 */
void nss_gmac_disable_mmc_ipc_rx_interrupt(nss_gmac_dev *gmacdev,
					   uint32_t mask)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacMmcRxIpcIntrMask, mask);
}

/*
 * Enable the MMC ipc rx checksum offload interrupt.
 * The MMC ipc rx checksum offload interrupts are enabled as
 * per the mask specified.
 * @param[in] pointer to nss_gmac_dev.
 * @param[in] rx interrupt bit mask for which interrupts needs to be enabled.
 * @return returns void.
 */
void nss_gmac_enable_mmc_ipc_rx_interrupt(nss_gmac_dev *gmacdev, uint32_t mask)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacMmcRxIpcIntrMask, mask);
}

/*******************MMC APIs*************************/

/************Ip checksum offloading APIs*************/

/*
 * Enables the ip checksum offloading in receive path.
 * When set GMAC calculates 16 bit 1's complement of all received
 * ethernet frame payload. It also checks IPv4 Header checksum is correct.
 * GMAC core appends the 16 bit checksum calculated for payload of IP
 * datagram and appends it to Ethernet frame transferred to the application.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_enable_rx_chksum_offload(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacConfig, GmacRxIpcOffload);
}

/*
 * Disable the ip checksum offloading in receive path.
 * Ip checksum offloading is disabled in the receive path.
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_disable_rx_chksum_offload(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacConfig, GmacRxIpcOffload);
}

/*
 * Instruct the DMA to drop the packets fails tcp ip checksum.
 * This is to instruct the receive DMA engine to drop the recevied
 * packet if they fails the tcp/ip checksum in hardware. Valid only when
 * full checksum offloading is enabled(type-2).
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_rx_tcpip_chksum_drop_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->dma_base,
				DmaControl, DmaDisableDropTcpCs);
}

/*
 * Instruct the DMA not to drop the packets even if it fails tcp ip checksum.
 * This is to instruct the receive DMA engine to allow the packets
 * even if recevied packet fails the tcp/ip checksum in hardware.
 * Valid only when full checksum offloading is enabled(type-2).
 * @param[in] pointer to nss_gmac_dev.
 * @return returns void.
 */
void nss_gmac_rx_tcpip_chksum_drop_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->dma_base,
			      DmaControl, DmaDisableDropTcpCs);
}

/*******************Ip checksum offloading APIs**********************/

/*******************IEEE 1588 Timestamping API***********************/

/*
 * This function enables the timestamping. This enables the timestamping
 * for transmit and receive frames. When disabled timestamp is not added
 * to tx and receive frames and timestamp generator is suspended.
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacTSControl, GmacTSENA);
}

/*
 * This function disables the timestamping.
 * When disabled timestamp is not added to tx and receive
 * frames and timestamp generator is suspended.
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacTSControl, GmacTSENA);
}

/*
 * Enable the interrupt to get timestamping interrupt.
 * This enables the host to get the interrupt when (1) system time
 * is greater or equal to the target time high and low register or
 * (2) there is a overflow in th esecond register.
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_int_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacInterruptMask, GmacTSIntMask);
}

/*
 * Disable the interrupt to get timestamping interrupt.
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_int_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacInterruptMask, GmacTSIntMask);
}

/*
 * Enable MAC address for PTP frame filtering.
 * When enabled, uses MAC address (apart from MAC address 0)
 * to filter the PTP frames when
 * PTP is sent directly over Ethernet.
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_mac_addr_filt_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacTSControl, GmacTSENMACADDR);
}

/*
 * Disables MAC address for PTP frame filtering.
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_mac_addr_filt_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacTSControl, GmacTSENMACADDR);
}

/*
 * Selet the type of clock mode for PTP.
 * Please note to use one of the follwoing as the clk_type argument.
 * GmacTSOrdClk		= 0x00000000,	00=> Ordinary clock
 * GmacTSBouClk		= 0x00010000,	01=> Boundary clock
 * GmacTSEtoEClk	= 0x00020000,	10=> End-to-End transparent clock
 * GmacTSPtoPClk	= 0x00030000,	11=> P-to-P transparent clock
 * @param[in] pointer to nss_gmac_dev
 * @param[in] uint32_t value representing one of the above clk value
 * @return returns void
 */
void nss_gmac_ts_set_clk_type(nss_gmac_dev *gmacdev, uint32_t clk_type)
{
	uint32_t clkval;
	clkval =
	    nss_gmac_read_reg((uint32_t *)gmacdev->mac_base, GmacTSControl);

	clkval = clkval | clk_type;
	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base, GmacTSControl,
			   clkval);
}

/*
 * Enable Snapshot for messages relevant to Master.
 * When enabled, snapshot is taken for messages relevant to master
 * mode only, else snapshot is taken for messages relevant to slave node.
 * Valid only for Ordinary clock and Boundary clock
 * Reserved when "Advanced Time Stamp" is not selected
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_master_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacTSControl, GmacTSMSTRENA);
}

/*
 * Disable Snapshot for messages relevant to Master.
 * When disabled, snapshot is taken for messages relevant
 * to slave node.
 * Valid only for Ordinary clock and Boundary clock
 * Reserved when "Advanced Time Stamp" is not selected
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_master_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacTSControl, GmacTSMSTRENA);
}

/*
 * Enable Snapshot for Event messages.
 * When enabled, snapshot is taken for event messages only
 * (SYNC, Delay_Req, Pdelay_Req or Pdelay_Resp). When disabled, snapshot
 * is taken for all messages except Announce, Management and Signaling.
 * Reserved when "Advanced Time Stamp" is not selected
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_event_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacTSControl, GmacTSEVNTENA);
}

/*
 * Disable Snapshot for Event messages.
 * When disabled, snapshot is taken for all messages except Announce,
 * Management and Signaling. Reserved when "Advanced Time Stamp"
 * is not selected.
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_event_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacTSControl, GmacTSEVNTENA);
}

/*
 * Enable time stamp snapshot for IPV4 frames.
 * When enabled, time stamp snapshot is taken for IPV4 frames
 * Reserved when "Advanced Time Stamp" is not selected
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_ipv4_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacTSControl, GmacTSIPV4ENA);
}

/*
 * Disable time stamp snapshot for IPV4 frames.
 * When disabled, time stamp snapshot is not taken for IPV4 frames
 * Reserved when "Advanced Time Stamp" is not selected
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 * ***** Only for "Advanced Time Stamp"
 */
void nss_gmac_ts_ipv4_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacTSControl, GmacTSIPV4ENA);
}

/*
 * Enable time stamp snapshot for IPV6 frames.
 * When enabled, time stamp snapshot is taken for IPV6 frames
 * Reserved when "Advanced Time Stamp" is not selected
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_ipv6_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacTSControl, GmacTSIPV6ENA);
}

/*
 * Disable time stamp snapshot for IPV6 frames.
 * When disabled, time stamp snapshot is not taken for IPV6 frames
 * Reserved when "Advanced Time Stamp" is not selected
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_ipv6_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacTSControl, GmacTSIPV6ENA);
}

/*
 * Enable time stamp snapshot for PTP over Ethernet frames.
 * When enabled, time stamp snapshot is taken for PTP over Ethernet frames
 * Reserved when "Advanced Time Stamp" is not selected
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_ptp_over_ethernet_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacTSControl, GmacTSIPENA);
}

/*
 * Disable time stamp snapshot for PTP over Ethernet frames.
 * When disabled, time stamp snapshot is not taken for PTP over Ethernet frames
 * Reserved when "Advanced Time Stamp" is not selected
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_ptp_over_ethernet_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacTSControl, GmacTSIPENA);
}

/*
 * Snoop PTP packet for version 2 format
 * When set the PTP packets are snooped using the version 2 format.
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_pkt_snoop_ver2(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacTSControl, GmacTSVER2ENA);
}

/*
 * Snoop PTP packet for version 2 format
 * When set the PTP packets are snooped using the version 2 format.
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_pkt_snoop_ver1(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacTSControl, GmacTSVER2ENA);
}

/*
 * Timestamp digital rollover
 * When set the timestamp low register rolls over after 0x3B9A_C9FF value.
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_digital_rollover_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacTSControl, GmacTSCTRLSSR);
}

/*
 * Timestamp binary rollover
 * When set the timestamp low register rolls over after 0x7FFF_FFFF value.
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_binary_rollover_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacTSControl, GmacTSCTRLSSR);
}

/*
 * Enable Time Stamp for All frames
 * When set the timestamp snap shot is enabled for all frames
 * received by the core. Reserved when "Advanced Time Stamp" is not selected.
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_all_frames_enable(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacTSControl, GmacTSENALL);
}

/*
 * Disable Time Stamp for All frames
 * When reset the timestamp snap shot is not enabled for all
 * frames received by the core. Reserved when "Advanced Time Stamp"
 * is not selected.
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_all_frames_disable(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacTSControl, GmacTSENALL);
}

/*
 * Addend Register Update
 * This function loads the contents of Time stamp addend register
 * with the supplied 32 value. This is reserved function when only
 * coarse correction option is selected.
 * @param[in] pointer to nss_gmac_dev
 * @param[in] 32 bit addend value
 * @return returns 0 for Success or else Failure
 */
int32_t nss_gmac_ts_addend_update(nss_gmac_dev *gmacdev, uint32_t addend_value)
{
	uint32_t loop_variable;

	/* Load the addend_value in to Addend register */
	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base,
			   GmacTSAddend, addend_value);

	for (loop_variable = 0;
	     loop_variable < DEFAULT_LOOP_VARIABLE; loop_variable++) {
		/* Wait till the busy bit gets cleared */
		if (!((nss_gmac_read_reg((uint32_t *)gmacdev->mac_base,
					 GmacTSControl)) & GmacTSADDREG)) {
			nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
					      GmacTSControl, GmacTSADDREG);
			return 0;

		}
		udelay(DEFAULT_DELAY_VARIABLE);
	}

	nss_gmac_info(gmacdev,
		      "Error::: The TSADDREG bit is not getting cleared !!!!!!\n");
	return -EIO;
}

/*
 * time stamp Update
 * This function updates (adds/subtracts) with the value specified
 * in the Timestamp High Update and Timestamp Low Update register.
 * @param[in] pointer to nss_gmac_dev
 * @param[in] Timestamp High Update value
 * @param[in] Timestamp Low Update value
 * @return returns 0 for Success or else Failure
 */
int32_t nss_gmac_ts_timestamp_update(nss_gmac_dev *gmacdev,
				     uint32_t high_value, uint32_t low_value)
{
	uint32_t loop_variable;

	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base,
			   GmacTSHighUpdate, high_value);

	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base,
			   GmacTSLowUpdate, low_value);

	for (loop_variable = 0;
	     loop_variable < DEFAULT_LOOP_VARIABLE; loop_variable++) {
		if (!((nss_gmac_read_reg((uint32_t *)gmacdev->mac_base,
					 GmacTSControl)) & GmacTSUPDT)) {
			nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
					      GmacTSControl, GmacTSUPDT);
			return 0;

		}
		udelay(DEFAULT_DELAY_VARIABLE);
	}

	nss_gmac_info(gmacdev,
		      "Error::: The TSADDREG bit is not getting cleared !!!!!!\n");
	return -EIO;
}

/*
 * time stamp Initialize
 * This function Loads/Initializes the value specified in
 * the Timestamp High Update and Timestamp Low Update register.
 * @param[in] pointer to nss_gmac_dev
 * @param[in] Timestamp High Load value
 * @param[in] Timestamp Low Load value
 * @return returns 0 for Success or else Failure
 */
int32_t nss_gmac_ts_timestamp_init(nss_gmac_dev *gmacdev,
				   uint32_t high_value, uint32_t low_value)
{
	uint32_t loop_variable;

	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base,
			   GmacTSHighUpdate, high_value);

	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base,
			   GmacTSLowUpdate, low_value);

	for (loop_variable = 0;
	     loop_variable < DEFAULT_LOOP_VARIABLE; loop_variable++) {
		if (!((nss_gmac_read_reg((uint32_t *)gmacdev->mac_base,
					 GmacTSControl)) & GmacTSINT)) {
			nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
					      GmacTSControl, GmacTSINT);
			return 0;

		}
		udelay(DEFAULT_DELAY_VARIABLE);
	}

	nss_gmac_info(gmacdev,
		      "Error::: The TSADDREG bit is not getting cleared !!!!!!\n");
	return -EIO;
}

/*
 * Time Stamp Update Coarse
 * When reset the timestamp update is done using coarse method.
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_coarse_update(nss_gmac_dev *gmacdev)
{
	nss_gmac_clear_reg_bits((uint32_t *)gmacdev->mac_base,
				GmacTSControl, GmacTSCFUPDT);
}

/*
 * Time Stamp Update Fine
 * When reset the timestamp update is done using Fine method.
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_fine_update(nss_gmac_dev *gmacdev)
{
	nss_gmac_set_reg_bits((uint32_t *)gmacdev->mac_base,
			      GmacTSControl, GmacTSCFUPDT);
}

/*
 * Load the Sub Second Increment value in to Sub Second increment register
 * @param[in] pointer to nss_gmac_dev
 * @return returns void
 */
void nss_gmac_ts_subsecond_init(nss_gmac_dev *gmacdev,
				uint32_t sub_sec_inc_value)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base,
			   GmacTSSubSecIncr,
			   (sub_sec_inc_value & GmacSSINCMsk));
}

/*
 * Reads the time stamp contents in to the respective pointers
 * These registers are readonly.
 * This function returns the 48 bit time stamp assuming Version 2
 * timestamp with higher word is selected.
 * @param[in] pointer to nss_gmac_dev
 * @param[in] pointer to hold 16 higher bit second register contents
 * @param[in] pointer to hold 32 bit second register contents
 * @param[in] pointer to hold 32 bit subnanosecond register contents
 * @return returns void
 * @note Please note that since the atomic access to the
 * timestamp registers is not possible, the contents read may
 * be different from the actual time stamp.
 */
void nss_gmac_ts_read_timestamp(nss_gmac_dev *gmacdev,
				uint16_t *higher_sec_val, uint32_t *sec_val,
				uint32_t *sub_sec_val)
{
	*higher_sec_val =
	    (uint16_t)(nss_gmac_read_reg
			((uint32_t *)gmacdev->mac_base,
			 GmacTSHighWord) & GmacTSHighWordMask);

	*sec_val = nss_gmac_read_reg((uint32_t *)gmacdev->mac_base, GmacTSHigh);

	*sub_sec_val = nss_gmac_read_reg((uint32_t *)gmacdev->mac_base,
					 GmacTSLow);
}

/*
 * Reads the time stamp higher sec value to respective pointers
 * @param[in] pointer to nss_gmac_dev
 * @param[in] pointer to hold 16 higher bit second register contents
 * @return returns void
 */
void nss_gmac_ts_read_timestamp_higher_val(nss_gmac_dev *gmacdev,
					   uint16_t *higher_sec_val)
{
	*higher_sec_val =
	    (uint16_t)(nss_gmac_read_reg
			((uint32_t *)gmacdev->mac_base,
			 GmacTSHighWord) & GmacTSHighWordMask);
}

/*
 * Load the Target time stamp registers
 * This function Loads the target time stamp registers with the values provided.
 * @param[in] pointer to nss_gmac_dev
 * @param[in] target Timestamp High value
 * @param[in] target Timestamp Low  value
 * @return void
 */
void nss_gmac_ts_load_target_timestamp(nss_gmac_dev *gmacdev,
				       uint32_t sec_val, uint32_t sub_sec_val)
{
	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base,
			   GmacTSTargetTimeHigh, sec_val);
	nss_gmac_write_reg((uint32_t *)gmacdev->mac_base,
			   GmacTSTargetTimeLow, sub_sec_val);
}

/*
 * Reads the Target time stamp registers
 * This function Loads the target time stamp registers with the values proviced
 * @param[in] pointer to nss_gmac_dev
 * @param[in] pointer to hold target Timestamp High value
 * @param[in] pointer to hold target Timestamp Low  value
 * @return void
 */
void nss_gmac_ts_read_target_timestamp(nss_gmac_dev *gmacdev,
				       uint32_t *sec_val,
				       uint32_t *sub_sec_val)
{
	*sec_val = nss_gmac_read_reg((uint32_t *)gmacdev->mac_base,
				     GmacTSTargetTimeHigh);
	*sub_sec_val = nss_gmac_read_reg((uint32_t *)gmacdev->mac_base,
					 GmacTSTargetTimeLow);
}
