/*
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
 * OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */
/*
 * @file
 * This file defines the APIs for accessing global NSS GMAC
 * software interface register space.
 * ------------------------REVISION HISTORY-----------------------------
 * Qualcomm Atheros         01/Mar/2013              Created
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/of.h>

#if (NSS_GMAC_DT_SUPPORT == 1)
#include <msm_nss_gmac.h>
#include <msm_nss_macsec.h>
#else
#include <mach/msm_nss_gmac.h>
#include <mach/msm_nss_macsec.h>
#endif

#include <nss_gmac_dev.h>
#include <nss_gmac_clocks.h>
#include <nss_gmac_network_interface.h>

/* Initialize notifier list for NSS GMAC */
static BLOCKING_NOTIFIER_HEAD(nss_gmac_notifier_list);

/**
 * @brief Emulation specific initialization.
 *
 * @param[in] nss_gmac_dev *
 * @return void
 */
void nss_gmac_spare_ctl(nss_gmac_dev *gmacdev)
{
	uint32_t val;
	uint32_t count;
	uint32_t id = gmacdev->macid;
	uint32_t *nss_base = (uint32_t *)(gmacdev->ctx->nss_base);

	if (!gmacdev->emulation) {
		return;
	}

	val = 1 << id;
	nss_gmac_set_reg_bits(nss_base, NSS_ETH_SPARE_CTL, val);

	val = nss_gmac_read_reg(nss_base, NSS_ETH_SPARE_CTL);
	nss_gmac_info(gmacdev, "NSS_ETH_SPARE_CTL - 0x%x", val);

	val = 1 << id;
	nss_gmac_clear_reg_bits(nss_base, NSS_ETH_SPARE_CTL, val);

	val = nss_gmac_read_reg(nss_base, NSS_ETH_SPARE_CTL);
	nss_gmac_info(gmacdev,
		      "NSS_ETH_SPARE_CTL - 0x%x after clear for gmac %d", val,
		      id);

	val = nss_gmac_read_reg(nss_base, NSS_ETH_SPARE_STAT);
	nss_gmac_info(gmacdev,
		      "NSS_ETH_SPARE_STAT - 0x%x; gmac %d spare ctl reset...",
		      val, id);
	count = 0;
	while ((val & (1 << id)) != (1 << id)) {
		mdelay(10);
		val = nss_gmac_read_reg(nss_base,
					NSS_ETH_SPARE_STAT);
		if (count++ > 20) {
			nss_gmac_info(gmacdev,
				      "!!!!!! Timeout waiting for NSS_ETH_SPARE_STAT bit to set.");
			break;
		}
	}
}


/**
 * @brief QSGMII Init for Emulation
 *
 * @param[in] nss_gmac_dev *
 * @return void
 */
static void nss_gmac_rumi_qsgmii_init(nss_gmac_dev *gmacdev)
{
	nss_gmac_dev *gmac1_dev;
	uint16_t phy_reg_val;
	uint32_t *qsgmii_base;
	uint8_t *nss_base;

	nss_gmac_info(gmacdev, "%s:", __FUNCTION__);

	gmac1_dev = gmacdev->ctx->nss_gmac[1];
	qsgmii_base = gmacdev->ctx->qsgmii_base;
	nss_base = (uint8_t *)(gmacdev->ctx->nss_base);

	/*
	 * _SGMII: Set only bit 3, with no polling for reset completion
	 * inside status register for GMAC2
	 */
	nss_gmac_info(gmacdev,"Eth2: spare_ctl_reg value before setting = 0x%x",
		      nss_gmac_read_reg((uint32_t *)nss_base, NSS_ETH_SPARE_CTL));
	nss_gmac_set_reg_bits((uint32_t *)nss_base, NSS_ETH_SPARE_CTL, 0x8);
	nss_gmac_info(gmacdev,"Eth2: spare_ctl_reg value after setting = 0x%x",
		      nss_gmac_read_reg((uint32_t *)nss_base, NSS_ETH_SPARE_CTL));

	nss_gmac_info(gmac1_dev, "%s: GMAC1's MACBASE = 0x%x",__FUNCTION__, gmac1_dev->mac_base);

	/* Put PHY in SGMII Mode */
	nss_gmac_write_reg(qsgmii_base, QSGMII_PHY_MODE_CTL, 0x0);

	/* Set SERDES signal detects for channel2, bypass SDO */
	nss_gmac_write_reg(qsgmii_base, PCS_QSGMII_CTL, 0x4213B);

	/* SERDES Configuration, drive strength settings through GMAC1's MDIO */

	/* Configure SERDES to SGMII-1+SGMII-2 mode */
	nss_gmac_mii_wr_reg(gmac1_dev, 0x0, 0x1, 0x8241);
	nss_gmac_mii_wr_reg(gmac1_dev, 0x0, 0x3, 0xB909);

	/* Writes to SERDES registers using MDIO debug registers */
	nss_gmac_mii_wr_reg(gmac1_dev, 0x0, 0x1D, 0x10);
	phy_reg_val = nss_gmac_mii_rd_reg(gmac1_dev, 0x0, 0x1E);

	nss_gmac_mii_wr_reg(gmac1_dev, 0x0, 0x1D, 0x10);
	nss_gmac_mii_wr_reg(gmac1_dev, 0x0, 0x1E, 0x2000);
	nss_gmac_mii_wr_reg(gmac1_dev, 0x0, 0x1D, 0x10);
	nss_gmac_mii_wr_reg(gmac1_dev, 0x0, 0x1E, 0x0);
	nss_gmac_mii_wr_reg(gmac1_dev, 0x0, 0x1D, 0x10);

	phy_reg_val = nss_gmac_mii_rd_reg(gmac1_dev, 0x0, 0x1E);

	nss_gmac_mii_wr_reg(gmac1_dev, 0x0, 0x1D, 0x0A);
	phy_reg_val = nss_gmac_mii_rd_reg(gmac1_dev, 0x0, 0x1E);

	nss_gmac_info(gmacdev, "Reg 1A reset val:  0x%x",phy_reg_val);

	nss_gmac_mii_wr_reg(gmac1_dev, 0x0, 0x1D, 0x0A);
	nss_gmac_mii_wr_reg(gmac1_dev, 0x0, 0x1E, 0x3F9);
	nss_gmac_mii_wr_reg(gmac1_dev, 0x0, 0x1D, 0x0A);

	phy_reg_val = nss_gmac_mii_rd_reg(gmac1_dev, 0x0, 0x1E);

	nss_gmac_info(gmacdev, "Reg 1A after programming:  0x%x", phy_reg_val);
	nss_gmac_mii_wr_reg(gmac1_dev, 0x0, 0x18, 0x30);

	/* Put PCS in SGMII Mode */
	nss_gmac_write_reg(qsgmii_base, PCS_QSGMII_SGMII_MODE, 0x0);

	/* Channel 2 force speed */
	nss_gmac_write_reg(qsgmii_base, PCS_ALL_CH_CTL, 0xF0000600);
}


/**
 * @brief QSGMII dev init
 *
 * @param[in] nss_gmac_dev *
 * @return void
 */
void nss_gmac_qsgmii_dev_init(nss_gmac_dev *gmacdev)
{
	uint32_t val = 0;
	uint32_t id = gmacdev->macid;
	uint8_t *nss_base = (uint8_t *)(gmacdev->ctx->nss_base);
	uint32_t *qsgmii_base = (uint32_t *)(gmacdev->ctx->qsgmii_base);

	if (gmacdev->emulation) {
		nss_gmac_rumi_qsgmii_init(gmacdev);
	}

	if (gmacdev->phy_mii_type == GMAC_INTF_SGMII) {
		switch (gmacdev->macid) {
		case 1:
			nss_gmac_write_reg((uint32_t *)qsgmii_base,
					   QSGMII_PHY_QSGMII_CTL, QSGMII_PHY_CDR_EN
								  | QSGMII_PHY_RX_FRONT_EN
								  | QSGMII_PHY_RX_SIGNAL_DETECT_EN
								  | QSGMII_PHY_TX_DRIVER_EN
								  | QSGMII_PHY_QSGMII_EN
								  | QSGMII_PHY_PHASE_LOOP_GAIN(0x4)
								  | QSGMII_PHY_RX_DC_BIAS(0x2)
								  | QSGMII_PHY_RX_INPUT_EQU(0x1)
								  | QSGMII_PHY_CDR_PI_SLEW(0x2)
								  | QSGMII_PHY_TX_DRV_AMP(0xC));

			val = nss_gmac_read_reg((uint32_t *)qsgmii_base, QSGMII_PHY_QSGMII_CTL);
			nss_gmac_trace(gmacdev,"%s: QSGMII_PHY_QSGMII_CTL(0x%x) - 0x%x",
						__FUNCTION__, QSGMII_PHY_QSGMII_CTL, val);
			break;

		case 2:
			nss_gmac_write_reg((uint32_t *)qsgmii_base,
					   QSGMII_PHY_SGMII_1_CTL, QSGMII_PHY_CDR_EN
								  | QSGMII_PHY_RX_FRONT_EN
								  | QSGMII_PHY_RX_SIGNAL_DETECT_EN
								  | QSGMII_PHY_TX_DRIVER_EN
								  | QSGMII_PHY_QSGMII_EN
								  | QSGMII_PHY_PHASE_LOOP_GAIN(0x4)
								  | QSGMII_PHY_RX_DC_BIAS(0x3)
								  | QSGMII_PHY_RX_INPUT_EQU(0x1)
								  | QSGMII_PHY_CDR_PI_SLEW(0x2)
								  | QSGMII_PHY_TX_DRV_AMP(0xC));

			val = nss_gmac_read_reg((uint32_t *)qsgmii_base, QSGMII_PHY_SGMII_1_CTL);
			nss_gmac_trace(gmacdev,"%s: QSGMII_PHY_SGMII_1_CTL(0x%x) - 0x%x",
						__FUNCTION__, QSGMII_PHY_SGMII_1_CTL, val);
			break;

		case 3:
			nss_gmac_write_reg((uint32_t *)qsgmii_base,
					   QSGMII_PHY_SGMII_2_CTL, QSGMII_PHY_CDR_EN
								  | QSGMII_PHY_RX_FRONT_EN
								  | QSGMII_PHY_RX_SIGNAL_DETECT_EN
								  | QSGMII_PHY_TX_DRIVER_EN
								  | QSGMII_PHY_QSGMII_EN
								  | QSGMII_PHY_PHASE_LOOP_GAIN(0x4)
								  | QSGMII_PHY_RX_DC_BIAS(0x3)
								  | QSGMII_PHY_RX_INPUT_EQU(0x1)
								  | QSGMII_PHY_CDR_PI_SLEW(0x2)
								  | QSGMII_PHY_TX_DRV_AMP(0xC));

			val = nss_gmac_read_reg((uint32_t *)qsgmii_base, QSGMII_PHY_SGMII_2_CTL);
			nss_gmac_trace(gmacdev,"%s: QSGMII_PHY_SGMII_2_CTL(0x%x) - 0x%x",
						__FUNCTION__, QSGMII_PHY_SGMII_2_CTL, val);
			break;
		}
	}

	/* Enable clk for GMACn */
	val = 0;
	if ((gmacdev->phy_mii_type == GMAC_INTF_SGMII) || (gmacdev->phy_mii_type == GMAC_INTF_QSGMII)) {
		val |= GMACn_QSGMII_RX_CLK(id) | GMACn_QSGMII_TX_CLK(id);
	}

	nss_gmac_clear_reg_bits((uint32_t *)nss_base, NSS_QSGMII_CLK_CTL, val);

	val = nss_gmac_read_reg((uint32_t *)nss_base, NSS_QSGMII_CLK_CTL);
	nss_gmac_trace(gmacdev,"%s: NSS_QSGMII_CLK_CTL(0x%x) - 0x%x",
		      __FUNCTION__, NSS_QSGMII_CLK_CTL, val);

	/* Enable autonegotiation between PCS and PHY */
	if (test_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags)) {
		nss_gmac_clear_reg_bits(qsgmii_base, PCS_ALL_CH_CTL,
					PCS_CHn_SPEED_MASK(gmacdev->macid));
		nss_gmac_clear_reg_bits(qsgmii_base, PCS_ALL_CH_CTL,
					PCS_CHn_FORCE_SPEED(gmacdev->macid));
	}
}


/**
 * @brief Clear all NSS GMAC interface registers.
 * @return returns 0 on success.
 */
static void nss_gmac_clear_all_regs(uint32_t *nss_base)
{
	nss_gmac_clear_reg_bits((uint32_t *)nss_base,
				NSS_ETH_CLK_GATE_CTL, 0xFFFFFFFF);
	nss_gmac_clear_reg_bits((uint32_t *)nss_base,
				NSS_ETH_CLK_DIV0, 0xFFFFFFFF);
	nss_gmac_clear_reg_bits((uint32_t *)nss_base,
				NSS_ETH_CLK_DIV1, 0xFFFFFFFF);
	nss_gmac_clear_reg_bits((uint32_t *)nss_base,
				NSS_ETH_CLK_SRC_CTL, 0xFFFFFFFF);
	nss_gmac_clear_reg_bits((uint32_t *)nss_base,
				NSS_ETH_CLK_INV_CTL, 0xFFFFFFFF);
	nss_gmac_clear_reg_bits((uint32_t *)nss_base,
				NSS_GMAC0_CTL, 0xFFFFFFFF);
	nss_gmac_clear_reg_bits((uint32_t *)nss_base,
				NSS_GMAC1_CTL, 0xFFFFFFFF);
	nss_gmac_clear_reg_bits((uint32_t *)nss_base,
				NSS_GMAC2_CTL, 0xFFFFFFFF);
	nss_gmac_clear_reg_bits((uint32_t *)nss_base,
				NSS_GMAC3_CTL, 0xFFFFFFFF);
	nss_gmac_clear_reg_bits((uint32_t *)nss_base,
				NSS_QSGMII_CLK_CTL, 0xFFFFFFFF);
}

/**
 * @brief Determine board type and return relevant
 * 	  NSS GMAC Phy profile.
 *
 * @return Phy profile
 */
static int32_t nss_gmac_get_phy_profile(void)
{
	if (of_machine_is_compatible("qcom,ipq8064")) {
		return NSS_GMAC_PHY_PROFILE_2R_2S;
	}

	return NSS_GMAC_PHY_PROFILE_1R_3S;
}


/**
 * @brief QSGMII common init
 *
 * @param[in] nss_gmac_dev *
 * @return void
 */
static void nss_gmac_qsgmii_common_init(struct nss_gmac_global_ctx *ctx)
{
	uint32_t val ;
	uint32_t *qsgmii_base = ctx->qsgmii_base;

	if (nss_gmac_get_phy_profile() == NSS_GMAC_PHY_PROFILE_QS) {
		/* Configure QSGMII Block for QSGMII mode */

		/* Put PHY in QSGMII Mode */
		nss_gmac_write_reg(qsgmii_base, QSGMII_PHY_MODE_CTL, QSGMII_PHY_MODE_QSGMII);

		/* Put PCS in QSGMII Mode */
		nss_gmac_write_reg(qsgmii_base, PCS_QSGMII_SGMII_MODE, PCS_QSGMII_MODE_QSGMII);

		nss_gmac_clear_reg_bits(qsgmii_base, QSGMII_PHY_QSGMII_CTL, QSGMII_PHY_TX_SLEW_MASK);

		goto out;
	}

	/* Configure QSGMII Block for 3xSGMII mode */

	/* Put PHY in SGMII Mode */
	nss_gmac_write_reg(qsgmii_base, QSGMII_PHY_MODE_CTL, QSGMII_PHY_MODE_SGMII);

	/* Put PCS in SGMII Mode */
	nss_gmac_write_reg(qsgmii_base, PCS_QSGMII_SGMII_MODE, PCS_QSGMII_MODE_SGMII);

out:
	val = nss_gmac_read_reg(qsgmii_base, QSGMII_PHY_MODE_CTL);
	nss_gmac_early_dbg("%s: qsgmii_base(0x%x) + QSGMII_PHY_MODE_CTL(0x%x): 0x%x",
	       __FUNCTION__, (uint32_t)qsgmii_base, (uint32_t)QSGMII_PHY_MODE_CTL, val);

	val = nss_gmac_read_reg(qsgmii_base, PCS_QSGMII_SGMII_MODE);
	nss_gmac_early_dbg("%s: qsgmii_base(0x%x) + PCS_QSGMII_SGMII_MODE(0x%x): 0x%x",
	       __FUNCTION__, (uint32_t)qsgmii_base, (uint32_t)PCS_QSGMII_SGMII_MODE, val);

	/* Mode ctrl signal for mode selection */
	nss_gmac_clear_reg_bits(qsgmii_base, PCS_MODE_CTL, PCS_MODE_CTL_SGMII_MAC
							   | PCS_MODE_CTL_SGMII_PHY);
	nss_gmac_set_reg_bits(qsgmii_base, PCS_MODE_CTL, PCS_MODE_CTL_SGMII_MAC);

	/* Apply reset to PCS and release */
	nss_gmac_write_reg((uint32_t *)(ctx->clk_ctl_base), NSS_RESET_SPARE, 0x3FFFFFF);
	udelay(100);
	nss_gmac_write_reg((uint32_t *)(ctx->clk_ctl_base), NSS_RESET_SPARE, 0x0);

	val = nss_gmac_read_reg((uint32_t *)(ctx->clk_ctl_base), NSS_RESET_SPARE);
	nss_gmac_early_dbg("%s: qsgmii_base(0x%x) + NSS_RESET_SPARE(0x%x): 0x%x",
	       __FUNCTION__, (uint32_t)(ctx->clk_ctl_base), (uint32_t)NSS_RESET_SPARE, val);

	/* signal detect and channel enable */
	nss_gmac_write_reg(qsgmii_base,
			   PCS_QSGMII_CTL, PCS_QSGMII_SW_VER_1_7 | PCS_QSGMII_ATHR_CSCO_AUTONEG
						/*| PCS_QSGMII_CUTTHROUGH_TX | PCS_QSGMII_CUTTHROUGH_RX*/
						| PCS_QSGMII_SHORT_THRESH | PCS_QSGMII_SHORT_LATENCY
						| PCS_QSGMII_DEPTH_THRESH(1) | PCS_CHn_SERDES_SN_DETECT(0)
						| PCS_CHn_SERDES_SN_DETECT(1) | PCS_CHn_SERDES_SN_DETECT(2)
						| PCS_CHn_SERDES_SN_DETECT(3) | PCS_CHn_SERDES_SN_DETECT_2(0)
						| PCS_CHn_SERDES_SN_DETECT_2(1) | PCS_CHn_SERDES_SN_DETECT_2(2)
						| PCS_CHn_SERDES_SN_DETECT_2(3));
	val = nss_gmac_read_reg(qsgmii_base, PCS_QSGMII_CTL);
	nss_gmac_early_dbg("%s: qsgmii_base(0x%x) + PCS_QSGMII_CTL(0x%x): 0x%x",
	       __FUNCTION__, (uint32_t)qsgmii_base, (uint32_t)PCS_QSGMII_CTL, val);

	/* set debug bits */
	nss_gmac_set_reg_bits((uint32_t *)qsgmii_base, PCS_ALL_CH_CTL, 0xF0000000);
}


/*
 * @brief Initialization commom to all GMACs.
 * @return returns 0 on success.
 */
int32_t nss_gmac_common_init(struct nss_gmac_global_ctx *ctx)
{
	volatile uint32_t val;

	spin_lock_init(&ctx->reg_lock);

	nss_gmac_clear_all_regs((uint32_t *)ctx->nss_base);

	nss_gmac_write_reg((uint32_t *)(ctx->qsgmii_base), QSGMII_PHY_QSGMII_CTL,
					QSGMII_PHY_CDR_EN | QSGMII_PHY_RX_FRONT_EN
					| QSGMII_PHY_RX_SIGNAL_DETECT_EN | QSGMII_PHY_TX_DRIVER_EN
					| QSGMII_PHY_QSGMII_EN | QSGMII_PHY_DEEMPHASIS_LVL(0x2)
					| QSGMII_PHY_PHASE_LOOP_GAIN(0x2) | QSGMII_PHY_RX_DC_BIAS(0x2)
					| QSGMII_PHY_RX_INPUT_EQU(0x1) | QSGMII_PHY_CDR_PI_SLEW(0x2)
					| QSGMII_PHY_TX_SLEW(0x2) | QSGMII_PHY_TX_DRV_AMP(0xC));

	nss_gmac_write_reg((uint32_t *)(ctx->clk_ctl_base), NSS_RESET_SPARE, 0x0FFFFFFF);
	udelay(100);
	nss_gmac_clear_reg_bits((uint32_t *)(ctx->clk_ctl_base), NSS_RESET_SPARE, CAL_PBRS_RST_N_RESET);
	mdelay(10);
	nss_gmac_clear_reg_bits((uint32_t *)(ctx->clk_ctl_base), NSS_RESET_SPARE, LCKDT_RST_N_RESET);
	nss_gmac_write_reg((uint32_t *)(ctx->qsgmii_base), PCS_CAL_LCKDT_CTL, PCS_LCKDT_RST);
	nss_gmac_write_reg((uint32_t *)(ctx->clk_ctl_base), NSS_RESET_SPARE, 0x0);
	nss_gmac_write_reg((ctx->tcsr_base), 0xc0, 0x0);

	/*
	 * Deaassert GMAC AHB reset
	 */
	nss_gmac_clear_reg_bits((uint32_t *)(ctx->clk_ctl_base), GMAC_AHB_RESET, 0x1);
	val = nss_gmac_read_reg(ctx->clk_ctl_base, GMAC_AHB_RESET);
	nss_gmac_early_dbg("%s: ctx->clk_ctl_base(0x%x) + GMAC_AHB_RESET(0x%x): 0x%x",
		       __FUNCTION__, (uint32_t)ctx->clk_ctl_base, (uint32_t)GMAC_AHB_RESET, val);

	/* Bypass MACSEC */
	nss_gmac_set_reg_bits((uint32_t *)(ctx->nss_base), NSS_MACSEC_CTL,
			      GMACn_MACSEC_BYPASS(1) | GMACn_MACSEC_BYPASS(2) | GMACn_MACSEC_BYPASS(3));

	val = nss_gmac_read_reg((uint32_t *)ctx->nss_base, NSS_MACSEC_CTL);
	nss_gmac_early_dbg("%s: nss_bsae(0x%x) + NSS_MACSEC_CTL(0x%x): 0x%x",
		       __FUNCTION__, (uint32_t)ctx->nss_base, (uint32_t)NSS_MACSEC_CTL, val);

	nss_gmac_qsgmii_common_init(ctx);

	/*
	 * Initialize ACC_GMAC_CUST field of NSS_ACC_REG register
	 * for GMAC and MACSEC memories.
	 */
	nss_gmac_clear_reg_bits((uint32_t *)(ctx->clk_ctl_base), NSS_ACC_REG, GMAC_ACC_CUST_MASK);
	val = nss_gmac_read_reg(ctx->clk_ctl_base, NSS_ACC_REG);
	nss_gmac_early_dbg("%s: ctx->clk_ctl_base(0x%x) + NSS_ACC_REG(0x%x): 0x%x",
		       __FUNCTION__, (uint32_t)ctx->clk_ctl_base, (uint32_t)NSS_ACC_REG, val);

	return 0;
}

/**
 * @brief Global common deinitialization.
 * @return void
 */
void nss_gmac_common_deinit(struct nss_gmac_global_ctx *ctx)
{
	nss_gmac_clear_all_regs((uint32_t *)ctx->nss_base);

	if (ctx->tcsr_base) {
		iounmap(ctx->tcsr_base);
		ctx->tcsr_base = NULL;
	}

	if (ctx->qsgmii_base) {
		iounmap(ctx->qsgmii_base);
		ctx->qsgmii_base = NULL;
	}

	if (ctx->clk_ctl_base) {
		iounmap(ctx->clk_ctl_base);
		ctx->clk_ctl_base = NULL;
	}

	if (ctx->nss_base) {
		iounmap(ctx->nss_base);
		ctx->nss_base = NULL;
	}
}

/*
 * @brief Return clock divider value for QSGMII PHY.
 * @param[in] nss_gmac_dev *
 * @return returns QSGMII clock divider value.
 */
static uint32_t clk_div_qsgmii(nss_gmac_dev *gmacdev)
{
	uint32_t div;

	switch (gmacdev->speed) {
	case SPEED1000:
		div = QSGMII_CLK_DIV_1000;
		break;

	case SPEED100:
		div = QSGMII_CLK_DIV_100;
		break;

	case SPEED10:
		div = QSGMII_CLK_DIV_10;
		break;

	default:
		div = QSGMII_CLK_DIV_1000;
		break;
	}

	return div;
}

/**
 * @brief Return clock divider value for SGMII PHY.
 * @param[in] nss_gmac_dev *
 * @return returns SGMII clock divider value.
 */
static uint32_t clk_div_sgmii(nss_gmac_dev *gmacdev)
{
	uint32_t div;

	switch (gmacdev->speed) {
	case SPEED1000:
		div = SGMII_CLK_DIV_1000;
		break;

	case SPEED100:
		div = SGMII_CLK_DIV_100;
		break;

	case SPEED10:
		div = SGMII_CLK_DIV_10;
		break;

	default:
		div = SGMII_CLK_DIV_1000;
		break;
	}

	return div;
}

/**
 * @brief Return clock divider value for RGMII PHY.
 * @param[in] nss_gmac_dev *
 * @return returns RGMII clock divider value.
 */
static uint32_t clk_div_rgmii(nss_gmac_dev *gmacdev)
{
	uint32_t div;

	switch (gmacdev->speed) {
	case SPEED1000:
		div = RGMII_CLK_DIV_1000;
		break;

	case SPEED100:
		div = RGMII_CLK_DIV_100;
		break;

	case SPEED10:
		div = RGMII_CLK_DIV_10;
		break;

	default:
		div = RGMII_CLK_DIV_1000;
		break;
	}

	return div;
}

/**
 * @brief Return PCS Channel speed values
 * @param[in] nss_gmac_dev *
 * @return returns PCS speed values.
 */
static uint32_t get_pcs_speed(nss_gmac_dev *gmacdev)
{
	uint32_t speed;

	switch (gmacdev->speed) {
	case SPEED1000:
		speed = PCS_CH_SPEED_1000;
		break;

	case SPEED100:
		speed = PCS_CH_SPEED_100;
		break;

	case SPEED10:
		speed = PCS_CH_SPEED_10;
		break;

	default:
		speed = PCS_CH_SPEED_1000;
		break;
	}

	return speed;
}
/**
 * @brief Set GMAC speed.
 * @param[in] nss_gmac_dev *
 * @return returns 0 on success.
 */
int32_t nss_gmac_dev_set_speed(nss_gmac_dev *gmacdev)
{
	uint32_t val = 0;
	uint32_t id = gmacdev->macid;
	uint32_t div = 0, pcs_speed = 0;
	uint32_t clk = 0;
	uint32_t *nss_base = (uint32_t *)(gmacdev->ctx->nss_base);
	uint32_t *qsgmii_base = (uint32_t *)(gmacdev->ctx->qsgmii_base);
	struct nss_gmac_speed_ctx gmac_speed_ctx = {0, 0};

	switch (gmacdev->phy_mii_type) {
	case GMAC_INTF_RGMII:
		div = clk_div_rgmii(gmacdev);
		break;

	case GMAC_INTF_SGMII:
		div = clk_div_sgmii(gmacdev);
		break;

	case GMAC_INTF_QSGMII:
		div = clk_div_qsgmii(gmacdev);
		break;

	default:
		nss_gmac_info(gmacdev, "%s: Invalid MII type", __FUNCTION__);
		return -EINVAL;
	}

	/* Force speed control signal if link polling is disabled */
	if (!test_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags)) {
		if (gmacdev->phy_mii_type == GMAC_INTF_SGMII) {
			pcs_speed = get_pcs_speed(gmacdev);
			nss_gmac_set_reg_bits(qsgmii_base, PCS_ALL_CH_CTL,
					      PCS_CHn_FORCE_SPEED(id));
			nss_gmac_clear_reg_bits(qsgmii_base, PCS_ALL_CH_CTL, PCS_CHn_SPEED_MASK(id));
			nss_gmac_set_reg_bits(qsgmii_base, PCS_ALL_CH_CTL,
					      PCS_CHn_SPEED(id, pcs_speed));
		}
	}

	clk = 0;
	/* Disable GMACn Tx/Rx clk */
	if (gmacdev->phy_mii_type == GMAC_INTF_RGMII) {
		clk |= GMACn_RGMII_RX_CLK(id) | GMACn_RGMII_TX_CLK(id);
	} else {
		clk |= GMACn_GMII_RX_CLK(id) | GMACn_GMII_TX_CLK(id);
	}
	nss_gmac_clear_reg_bits(nss_base, NSS_ETH_CLK_GATE_CTL, clk);

	/* set clock divider */
	val = nss_gmac_read_reg(nss_base, NSS_ETH_CLK_DIV0);
	val &= ~GMACn_CLK_DIV(id, GMACn_CLK_DIV_SIZE);
	val |= GMACn_CLK_DIV(id, div);
	nss_gmac_write_reg(nss_base, NSS_ETH_CLK_DIV0, val);

	/* Enable GMACn Tx/Rx clk */
	nss_gmac_set_reg_bits(nss_base, NSS_ETH_CLK_GATE_CTL, clk);

	val = nss_gmac_read_reg(nss_base, NSS_ETH_CLK_DIV0);
	nss_gmac_info(gmacdev, "%s:NSS_ETH_CLK_DIV0(0x%x) - 0x%x",
		      __FUNCTION__, NSS_ETH_CLK_DIV0, val);

	if (gmacdev->phy_mii_type == GMAC_INTF_SGMII
	    || gmacdev->phy_mii_type == GMAC_INTF_QSGMII) {
		nss_gmac_clear_reg_bits(qsgmii_base, PCS_MODE_CTL,
					PCS_MODE_CTL_CHn_AUTONEG_EN(id));

		/* Enable autonegotiation from MII register of PHY */
		if (test_bit(__NSS_GMAC_LINKPOLL, &gmacdev->flags)) {
			nss_gmac_set_reg_bits(qsgmii_base, PCS_MODE_CTL,
					      PCS_MODE_CTL_CHn_AUTONEG_EN(id));
		}

		val = nss_gmac_read_reg(qsgmii_base, PCS_MODE_CTL);
		nss_gmac_trace(gmacdev, "%s: qsgmii_base(0x%x) + PCS_MODE_CTL(0x%x): 0x%x",
		       __FUNCTION__, (uint32_t)qsgmii_base, (uint32_t)PCS_MODE_CTL, val);

	}

	/* Notify link speed change to notifier list */
	gmac_speed_ctx.mac_id = gmacdev->macid;
	gmac_speed_ctx.speed = gmacdev->speed;
	blocking_notifier_call_chain(&nss_gmac_notifier_list, NSS_GMAC_SPEED_SET, &gmac_speed_ctx);

	return 0;
}

/**
 * @brief GMAC device initializaton.
 * @param[in] nss_gmac_dev *
 * @return void
 */
void nss_gmac_dev_init(nss_gmac_dev *gmacdev)
{
	uint32_t val = 0;
	uint32_t div = 0;
	uint32_t id = gmacdev->macid;
	uint32_t *nss_base = (uint32_t *)(gmacdev->ctx->nss_base);
	struct nss_gmac_global_ctx *ctx = gmacdev->ctx;

	/*
	 * Initialize wake and sleep counter values of
	 * GMAC memory footswitch control.
	 */
	nss_gmac_set_reg_bits(ctx->clk_ctl_base, GMAC_COREn_CLK_FS(id) , GMAC_FS_S_W_VAL);
	val = nss_gmac_read_reg(ctx->clk_ctl_base, GMAC_COREn_CLK_FS(id));
	nss_gmac_trace(gmacdev, "%s: ctx->clk_ctl_base(0x%x) + GMAC_COREn_CLK_FS(%d)(0x%x): 0x%x",
		       __FUNCTION__, (uint32_t)ctx->clk_ctl_base, id, (uint32_t)GMAC_COREn_CLK_FS(id), val);

	/*
	 * Bring up GMAC core clock
	 */
	/* a) Program GMAC_COREn_CLK_SRC_CTL register */
	nss_gmac_clear_reg_bits(ctx->clk_ctl_base, GMAC_COREn_CLK_SRC_CTL(id),
				GMAC_DUAL_MN8_SEL |
				GMAC_CLK_ROOT_ENA |
				GMAC_CLK_LOW_PWR_ENA);
	nss_gmac_set_reg_bits(ctx->clk_ctl_base, GMAC_COREn_CLK_SRC_CTL(id),
			      GMAC_CLK_ROOT_ENA);

	val = nss_gmac_read_reg(ctx->clk_ctl_base, GMAC_COREn_CLK_SRC_CTL(id));
	nss_gmac_trace(gmacdev, "%s: ctx->clk_ctl_base(0x%x) + GMAC_COREn_CLK_SRC_CTL(%d)(0x%x): 0x%x",
		       __FUNCTION__, (uint32_t)ctx->clk_ctl_base, id, (uint32_t)GMAC_COREn_CLK_SRC_CTL(id), val);

	/* b) Program M & D values in GMAC_COREn_CLK_SRC[0,1]_MD register. */
	nss_gmac_write_reg(ctx->clk_ctl_base, GMAC_COREn_CLK_SRC0_MD(id), 0);
	nss_gmac_write_reg(ctx->clk_ctl_base, GMAC_COREn_CLK_SRC1_MD(id), 0);
	nss_gmac_set_reg_bits(ctx->clk_ctl_base, GMAC_COREn_CLK_SRC0_MD(id),
			      GMAC_CORE_CLK_M_VAL | GMAC_CORE_CLK_D_VAL);
	nss_gmac_set_reg_bits(ctx->clk_ctl_base, GMAC_COREn_CLK_SRC1_MD(id),
			      GMAC_CORE_CLK_M_VAL | GMAC_CORE_CLK_D_VAL);

	val = nss_gmac_read_reg(ctx->clk_ctl_base, GMAC_COREn_CLK_SRC0_MD(id));
	nss_gmac_trace(gmacdev, "%s: ctx->clk_ctl_base(0x%x) + GMAC_COREn_CLK_SRC0_MD(%d)(0x%x): 0x%x",
		       __FUNCTION__, (uint32_t)ctx->clk_ctl_base, id, (uint32_t)GMAC_COREn_CLK_SRC0_MD(id), val);
	val = nss_gmac_read_reg(ctx->clk_ctl_base, GMAC_COREn_CLK_SRC1_MD(id));
	nss_gmac_trace(gmacdev, "%s: ctx->clk_ctl_base(0x%x) + GMAC_COREn_CLK_SRC1_MD(%d)(0x%x): 0x%x",
		       __FUNCTION__, (uint32_t)ctx->clk_ctl_base, id, (uint32_t)GMAC_COREn_CLK_SRC1_MD(id), val);


	/* c) Program N values on GMAC_COREn_CLK_SRC[0,1]_NS register */
	nss_gmac_write_reg(ctx->clk_ctl_base, GMAC_COREn_CLK_SRC0_NS(id), 0);
	nss_gmac_write_reg(ctx->clk_ctl_base, GMAC_COREn_CLK_SRC1_NS(id), 0);
	nss_gmac_set_reg_bits(ctx->clk_ctl_base, GMAC_COREn_CLK_SRC0_NS(id),
			      GMAC_CORE_CLK_N_VAL
			      | GMAC_CORE_CLK_MNCNTR_EN
			      | GMAC_CORE_CLK_MNCNTR_MODE_DUAL
			      | GMAC_CORE_CLK_PRE_DIV_SEL_BYP
			      | GMAC_CORE_CLK_SRC_SEL_PLL0);
	nss_gmac_set_reg_bits(ctx->clk_ctl_base, GMAC_COREn_CLK_SRC1_NS(id),
			      GMAC_CORE_CLK_N_VAL
			      | GMAC_CORE_CLK_MNCNTR_EN
			      | GMAC_CORE_CLK_MNCNTR_MODE_DUAL
			      | GMAC_CORE_CLK_PRE_DIV_SEL_BYP
			      | GMAC_CORE_CLK_SRC_SEL_PLL0);

	val = nss_gmac_read_reg(ctx->clk_ctl_base, GMAC_COREn_CLK_SRC0_NS(id));
	nss_gmac_trace(gmacdev, "%s: ctx->clk_ctl_base(0x%x) + GMAC_COREn_CLK_SRC0_NS(%d)(0x%x): 0x%x",
		       __FUNCTION__, (uint32_t)ctx->clk_ctl_base, id, (uint32_t)GMAC_COREn_CLK_SRC0_NS(id), val);
	val = nss_gmac_read_reg(ctx->clk_ctl_base, GMAC_COREn_CLK_SRC1_NS(id));
	nss_gmac_trace(gmacdev, "%s: ctx->clk_ctl_base(0x%x) + GMAC_COREn_CLK_SRC1_NS(%d)(0x%x): 0x%x",
		       __FUNCTION__, (uint32_t)ctx->clk_ctl_base, id, (uint32_t)GMAC_COREn_CLK_SRC1_NS(id), val);

	/* d) Un-halt GMACn clock */
	nss_gmac_clear_reg_bits(ctx->clk_ctl_base, CLK_HALT_NSSFAB0_NSSFAB1_STATEA,
				GMACn_CORE_CLK_HALT(id));
	val = nss_gmac_read_reg(ctx->clk_ctl_base, CLK_HALT_NSSFAB0_NSSFAB1_STATEA);
	nss_gmac_trace(gmacdev, "%s: ctx->clk_ctl_base(0x%x) + CLK_HALT_NSSFAB0_NSSFAB1_STATEA(0x%x): 0x%x",
		       __FUNCTION__, (uint32_t)ctx->clk_ctl_base, (uint32_t)CLK_HALT_NSSFAB0_NSSFAB1_STATEA, val);

	/* e) CLK_COREn_CLK_CTL: select branch enable and disable clk invert */
	nss_gmac_clear_reg_bits(ctx->clk_ctl_base, GMAC_COREn_CLK_CTL(id), GMAC_CLK_INV);
	nss_gmac_set_reg_bits(ctx->clk_ctl_base, GMAC_COREn_CLK_CTL(id), GMAC_CLK_BRANCH_EN);
	val = nss_gmac_read_reg(ctx->clk_ctl_base, GMAC_COREn_CLK_CTL(id));
	nss_gmac_trace(gmacdev, "%s: ctx->clk_ctl_base(0x%x) + GMAC_COREn_CLK_CTL(%d)(0x%x): 0x%x",
		       __FUNCTION__, (uint32_t)ctx->clk_ctl_base, id, (uint32_t)GMAC_COREn_CLK_CTL(id), val);

	/* Set GMACn Ctl: Phy interface select, IFG, AXI low power request signal (CSYSREQ) */
	val = GMAC_IFG_CTL(GMAC_IFG) | GMAC_IFG_LIMIT(GMAC_IFG) | GMAC_CSYS_REQ;
	if (gmacdev->phy_mii_type == GMAC_INTF_RGMII) {
		val |= GMAC_PHY_RGMII;
	} else {
		val &= ~GMAC_PHY_RGMII;
	}

	nss_gmac_write_reg(nss_base, NSS_GMACn_CTL(id), 0x0);
	nss_gmac_write_reg(nss_base, NSS_GMACn_CTL(id), val);

	val = nss_gmac_read_reg(nss_base, NSS_GMACn_CTL(id));
	nss_gmac_trace(gmacdev, "%s: nss_base(0x%x) + NSS_GMACn_CTL(%d)(0x%x): 0x%x",
		       __FUNCTION__, (uint32_t)nss_base, id, (uint32_t)NSS_GMACn_CTL(id), val);

	/*
	 * Optionally enable/disable MACSEC bypass.
	 * We are doing this in nss_gmac_plat_init()
	 */

	/*
	 * Deassert GMACn power on reset
	 */
	nss_gmac_clear_reg_bits(ctx->clk_ctl_base, GMAC_COREn_RESET(id), 0x1);

	/* Configure clock dividers for 1000Mbps default */
	gmacdev->speed = SPEED1000;
	switch (gmacdev->phy_mii_type) {
	case GMAC_INTF_RGMII:
		div = clk_div_rgmii(gmacdev);
		break;

	case GMAC_INTF_SGMII:
		div = clk_div_sgmii(gmacdev);
		break;

	case GMAC_INTF_QSGMII:
		div = clk_div_qsgmii(gmacdev);
		break;
	}
	val = nss_gmac_read_reg(nss_base, NSS_ETH_CLK_DIV0);
	val &= ~GMACn_CLK_DIV(id, GMACn_CLK_DIV_SIZE);
	val |= GMACn_CLK_DIV(id, div);
	nss_gmac_write_reg(nss_base, NSS_ETH_CLK_DIV0, val);

	val = nss_gmac_read_reg(nss_base, NSS_ETH_CLK_DIV0);
	nss_gmac_trace(gmacdev, "%s: nss_base(0x%x) + NSS_ETH_CLK_DIV0(0x%x): 0x%x",
		       __FUNCTION__, (uint32_t)nss_base, (uint32_t)NSS_ETH_CLK_DIV0, val);

	/* Select Tx/Rx CLK source */
	val = 0;
	if (id == 0 || id == 1) {
		if (gmacdev->phy_mii_type == GMAC_INTF_RGMII) {
			val |= (1 << id);
		}
	} else {
		if (gmacdev->phy_mii_type == GMAC_INTF_SGMII) {
			val |= (1 << id);
		}
	}
	nss_gmac_set_reg_bits(nss_base, NSS_ETH_CLK_SRC_CTL, val);

	/* Enable xGMII clk for GMACn */
	val = 0;
	if (gmacdev->phy_mii_type == GMAC_INTF_RGMII) {
		val |= GMACn_RGMII_RX_CLK(id) | GMACn_RGMII_TX_CLK(id);
	} else {
		val |= GMACn_GMII_RX_CLK(id) | GMACn_GMII_TX_CLK(id);
	}

	/* Optionally configure RGMII CDC delay */

	/* Enable PTP clock */
	val |= GMACn_PTP_CLK(id);
	nss_gmac_set_reg_bits(nss_base, NSS_ETH_CLK_GATE_CTL, val);

	if ((gmacdev->phy_mii_type == GMAC_INTF_SGMII)
	     || (gmacdev->phy_mii_type == GMAC_INTF_QSGMII)) {
		nss_gmac_qsgmii_dev_init(gmacdev);
		nss_gmac_info(gmacdev, "SGMII Specific Init for GMAC%d Done!", id);
	}
}

#if (NSS_MACSEC_SUPPORT == 1)
/**
 * @brief Do macsec related initialization in gmac register scope.
 * @return void.
 */
void nss_macsec_pre_init(void)
{
	uint32_t val = 0;
	uint32_t *nss_base = (uint32_t *)ctx.nss_base;

	/*
	 * Initialize wake and sleep counter values of
	 * MACSEC memory footswitch control.
	 */
	nss_gmac_write_reg(nss_base, NSS_MACSEC1_CORE_CLK_FS_CTL, MACSEC_CLK_FS_CTL_S_W_VAL);
	nss_gmac_write_reg(nss_base, NSS_MACSEC2_CORE_CLK_FS_CTL, MACSEC_CLK_FS_CTL_S_W_VAL);
	nss_gmac_write_reg(nss_base, NSS_MACSEC3_CORE_CLK_FS_CTL, MACSEC_CLK_FS_CTL_S_W_VAL);

	/* MACSEC reset */
	nss_gmac_write_reg(MSM_CLK_CTL_BASE, MACSEC_CORE1_RESET, 1);
	nss_gmac_write_reg(MSM_CLK_CTL_BASE, MACSEC_CORE2_RESET, 1);
	nss_gmac_write_reg(MSM_CLK_CTL_BASE, MACSEC_CORE3_RESET, 1);
	mdelay(100);

	/* Deassert MACSEC reset */
	nss_gmac_write_reg(MSM_CLK_CTL_BASE, MACSEC_CORE1_RESET, 0);
	nss_gmac_write_reg(MSM_CLK_CTL_BASE, MACSEC_CORE2_RESET, 0);
	nss_gmac_write_reg(MSM_CLK_CTL_BASE, MACSEC_CORE3_RESET, 0);

	/* Enable MACSEC clocks */
	val = nss_gmac_read_reg(nss_base, NSS_ETH_CLK_GATE_CTL);
	val |= (MACSEC_CORE_CLKEN_VAL | MACSEC_GMII_RX_CLKEN_VAL | MACSEC_GMII_TX_CLKEN_VAL);
	nss_gmac_write_reg(nss_base, NSS_ETH_CLK_GATE_CTL, val);

	/* Bypass all MACSECs */
	nss_gmac_write_reg(nss_base, NSS_MACSEC_CTL, MACSEC_EXT_BYPASS_EN_MASK | MACSEC_DP_RST_VAL);
}
EXPORT_SYMBOL(nss_macsec_pre_init);

/**
 * @brief reset MACSEC IFG register
 * @param[in] gmac_id
 * @return void
 */
static void nss_gmac_ifg_reset(uint32_t gmac_id)
{
	uint32_t val = 0;
	uint32_t *nss_base = (uint32_t *)ctx.nss_base;

	val = nss_gmac_read_reg(nss_base, NSS_GMACn_CTL(gmac_id));
	val &= ~(IFG_MASK | GMAC_IFG_LIMIT(IFG_MASK));
	val |= (GMAC_IFG_CTL(GMAC_IFG) | GMAC_IFG_LIMIT(GMAC_IFG));
	nss_gmac_write_reg(nss_base, NSS_GMACn_CTL(gmac_id), val);
}

/**
 * @brief set gmac link status into expected state
 * @param[in] gmac_id
 * @param[in] link_state
 * @return void
 */
static void nss_gmac_link_status_set(uint32_t gmac_id, uint32_t link_state)
{
	nss_gmac_dev *gmac_dev = NULL;

	gmac_dev = ctx.nss_gmac[gmac_id];
	if(gmac_dev == NULL)
		return;

	if (gmac_dev->nss_state != NSS_STATE_INITIALIZED
		|| !test_bit(__NSS_GMAC_UP, &gmac_dev->flags)) {
		return;
	}

	if (link_state == LINKDOWN && gmac_dev->link_state == LINKUP) {
		nss_gmac_linkdown(gmac_dev);
	} else if (link_state == LINKUP && gmac_dev->link_state == LINKDOWN) {
		nss_gmac_linkup(gmac_dev);
	}

}

/**
 * @brief enable or disable MACSEC bypass function
 * @param[in] gmac_id
 * @param[in] enable
 * @return void
 */
void nss_macsec_bypass_en_set(uint32_t gmac_id, bool enable)
{
	uint32_t val = 0;
	uint32_t *nss_base = (uint32_t *)ctx.nss_base;
	nss_gmac_dev *gmac_dev = NULL;
	uint32_t link_reset_flag = 0;
	struct nss_gmac_speed_ctx gmac_speed_ctx = {0, 0};

	if((gmac_id == 0) || (gmac_id > 3))
		return;

	gmac_dev = ctx.nss_gmac[gmac_id];
	if(gmac_dev == NULL)
		return;

	mutex_lock(&gmac_dev->link_mutex);

	/* If gmac is in link up state, it need to simulate link down event
	 * before setting IFG and simulate link up event after the operation
	 */
	if(gmac_dev->link_state == LINKUP)
		link_reset_flag = 1;

	/* simulate a gmac link down event */
	if(link_reset_flag)
		nss_gmac_link_status_set(gmac_id, LINKDOWN);

	/* Set MACSEC_IFG value */
	if(enable) {
		nss_gmac_ifg_reset(gmac_id);
	} else {
		val = nss_gmac_read_reg(nss_base, NSS_GMACn_CTL(gmac_id));
		val &= ~(IFG_MASK | GMAC_IFG_LIMIT(IFG_MASK));
		val |= (GMAC_IFG_CTL(MACSEC_IFG) | GMAC_IFG_LIMIT(MACSEC_IFG));
		nss_gmac_write_reg(nss_base, NSS_GMACn_CTL(gmac_id), val);
	}

	/* Enable/Disable MACSEC for related port */
	val = nss_gmac_read_reg(nss_base, NSS_MACSEC_CTL);
	val |= MACSEC_DP_RST_VAL;
	if(enable)
		val |= (1<<(gmac_id - 1));
	else
		val &= ~(1<<(gmac_id - 1));
	nss_gmac_write_reg(nss_base, NSS_MACSEC_CTL, val);

	/* simulate a gmac link up event */
	if(link_reset_flag)
		nss_gmac_link_status_set(gmac_id, LINKUP);

	mutex_unlock(&gmac_dev->link_mutex);

	/* Set MACSEC speed */
	gmac_speed_ctx.mac_id = gmac_dev->macid;
	gmac_speed_ctx.speed = gmac_dev->speed;
	blocking_notifier_call_chain(&nss_gmac_notifier_list, NSS_GMAC_SPEED_SET, &gmac_speed_ctx);
}
EXPORT_SYMBOL(nss_macsec_bypass_en_set);

/**
 * @brief  Do macsec related exist function in gmac register scope
 * @return void
 */
void nss_macsec_pre_exit(void)
{
	uint32_t *nss_base = (uint32_t *)ctx.nss_base;
	nss_gmac_dev *gmac_dev = NULL;
	uint32_t gmac_id = 0;
	uint32_t link_reset_flag = 0;

	/* MACSEC reset */
	nss_gmac_write_reg(MSM_CLK_CTL_BASE, MACSEC_CORE1_RESET, 1);
	nss_gmac_write_reg(MSM_CLK_CTL_BASE, MACSEC_CORE2_RESET, 1);
	nss_gmac_write_reg(MSM_CLK_CTL_BASE, MACSEC_CORE3_RESET, 1);

	/* Bypass all MACSECs */
	nss_gmac_write_reg(nss_base, NSS_MACSEC_CTL, MACSEC_EXT_BYPASS_EN_MASK | MACSEC_DP_RST_VAL);

	/* Reset GMAC_IFG value */
	for(gmac_id = 1; gmac_id < 4; gmac_id++) {
		gmac_dev = ctx.nss_gmac[gmac_id];
		if(gmac_dev == NULL)
			continue;

		/* If gmac is in link up state, it need to simulate link down event
		 * before setting IFG and simulate link up event after the operation
		 */
		link_reset_flag = 0;

		mutex_lock(&gmac_dev->link_mutex);

		if(gmac_dev->link_state == LINKUP)
			link_reset_flag = 1;

		/* simulate a gmac link down event */
		if(link_reset_flag)
			nss_gmac_link_status_set(gmac_id, LINKDOWN);

		nss_gmac_ifg_reset(gmac_id);

		/* simulate a gmac link up event */
		if(link_reset_flag)
			nss_gmac_link_status_set(gmac_id, LINKUP);

		mutex_unlock(&gmac_dev->link_mutex);
	}
}
EXPORT_SYMBOL(nss_macsec_pre_exit);
#endif /** NSS_MACSEC_SUPPORT */

/**
 * @brief register notifier into gmac module
 * @param[in] struct notifier_block *
 * @return void
 */
void nss_gmac_link_state_change_notify_register(struct notifier_block *nb)
{
	blocking_notifier_chain_register(&nss_gmac_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(nss_gmac_link_state_change_notify_register);

/**
 * @brief unregister notifier into gmac module
 * @param[in] struct notifier_block *
 * @return void
 */
void nss_gmac_link_state_change_notify_unregister(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&nss_gmac_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(nss_gmac_link_state_change_notify_unregister);
