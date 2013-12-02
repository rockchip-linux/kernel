/*
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/pm_runtime.h>

#include "sdhci-pltfm.h"

/* Tegra SDHOST controller vendor register definitions */
#define SDHCI_VNDR_CLK_CTRL	0x100
#define SDHCI_VNDR_CLK_CTRL_SDMMC_CLK	0x1
#define SDHCI_VNDR_CLK_CTRL_PADPIPE_CLKEN_OVERRIDE	0x8
#define SDHCI_VNDR_CLK_CTRL_SPI_MODE_CLKEN_OVERRIDE	0x4
#define SDHCI_VNDR_CLK_CTRL_INPUT_IO_CLK		0x2
#define SDHCI_VNDR_CLK_CTRL_TAP_VALUE_SHIFT	16
#define SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_SHIFT	24
#define SDHCI_VNDR_CLK_CTRL_SDR50_TUNING		0x20
#define SDHCI_VNDR_CLK_CTRL_INTERNAL_CLK	0x2

#define SDHCI_TEGRA_VENDOR_MISC_CTRL		0x120
#define SDHCI_VNDR_MISC_CTRL_ENABLE_SDR104_SUPPORT	0x8
#define SDHCI_VNDR_MISC_CTRL_ENABLE_SDR50_SUPPORT	0x10
#define SDHCI_VNDR_MISC_CTRL_ENABLE_DDR50_SUPPORT	0x200
#define SDHCI_MISC_CTRL_ENABLE_SDHCI_SPEC_300	0x20
#define SDHCI_VNDR_MISC_CTRL_INFINITE_ERASE_TIMEOUT	0x1
#define SDHCI_VNDR_MISC_CTRL_PIPE_STAGES_MASK	0x180

#define SDMMC_SDMEMCOMPPADCTRL	0x1E0
#define SDMMC_SDMEMCOMPPADCTRL_VREF_SEL_MASK	0xF
#define SDMMC_SDMEMCOMPPADCTRL_PAD_E_INPUT_OR_E_PWRD_MASK	0x80000000

#define SDMMC_AUTO_CAL_CONFIG	0x1E4
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_START	0x80000000
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE	0x20000000
#define SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET_SHIFT	0x8

#define SDMMC_AUTO_CAL_STATUS	0x1EC
#define SDMMC_AUTO_CAL_STATUS_AUTO_CAL_ACTIVE	0x80000000
#define SDMMC_AUTO_CAL_STATUS_PULLDOWN_OFFSET	24
#define PULLUP_ADJUSTMENT_OFFSET	20

#define MMC_1V8_CALIB_OFFSET_SDR12      0x1
#define MMC_1V8_CALIB_OFFSET_SDR25      0x2
#define MMC_1V8_CALIB_OFFSET_SDR50      0x4
#define MMC_1V8_CALIB_OFFSET_DDR50      0x8
#define MMC_1V8_CALIB_OFFSET_SDR104     0x10
#define MMC_1V8_CALIB_OFFSET_HS200      0x20

#define NVQUIRK_FORCE_SDHCI_SPEC_200	BIT(0)
#define NVQUIRK_ENABLE_BLOCK_GAP_DET	BIT(1)
#define NVQUIRK_ENABLE_SDHCI_SPEC_300	BIT(2)
#define NVQUIRK_SET_CALIBRATION_OFFSETS	BIT(3)
#define NVQUIRK_DISABLE_AUTO_CALIBRATION	BIT(4)
#define NVQUIRK_SET_DRIVE_STRENGTH		BIT(5)
#define NVQUIRK_ENABLE_PADPIPE_CLKEN		BIT(6)
#define NVQUIRK_DISABLE_SPI_MODE_CLKEN		BIT(7)
#define NVQUIRK_SET_TAP_DELAY			BIT(8)
#define NVQUIRK_SET_TRIM_DELAY			BIT(9)
#define NVQUIRK_ENABLE_SD_3_0			BIT(10)
#define NVQUIRK_ENABLE_SDR50			BIT(11)
#define NVQUIRK_ENABLE_SDR104			BIT(12)
#define NVQUIRK_ENABLE_DDR50			BIT(13)
#define NVQUIRK_ENABLE_SDR50_TUNING		BIT(14)
#define NVQUIRK_ENABLE_HS200			BIT(15)
#define NVQUIRK_INFINITE_ERASE_TIMEOUT		BIT(16)
#define NVQUIRK_EN_FEEDBACK_CLK			BIT(17)
#define NVQUIRK_DISABLE_AUTO_CMD23		BIT(18)
#define NVQUIRK_SHADOW_XFER_MODE_REG		BIT(19)
#define NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD	BIT(20)
#define NVQUIRK_SET_PIPE_STAGES_MASK_0		BIT(21)

/* Common subset of quirks for Tegra3 and later sdmmc controllers */
#define TEGRA30_SDHCI_NVQUIRKS	(NVQUIRK_ENABLE_PADPIPE_CLKEN | \
		  NVQUIRK_DISABLE_SPI_MODE_CLKEN | \
		  NVQUIRK_EN_FEEDBACK_CLK | \
		  NVQUIRK_SET_TAP_DELAY | \
		  NVQUIRK_ENABLE_SDR50_TUNING | \
		  NVQUIRK_ENABLE_SDR50 | \
		  NVQUIRK_ENABLE_SDR104 | \
		  NVQUIRK_SHADOW_XFER_MODE_REG | \
		  NVQUIRK_DISABLE_AUTO_CMD23)

#define TEGRA_SDHCI_AUTOSUSPEND_DELAY	1500

#define MMC_UHS_MASK_SDR12	0x1
#define MMC_UHS_MASK_SDR25	0x2
#define MMC_UHS_MASK_SDR50	0x4
#define MMC_UHS_MASK_DDR50	0x8
#define MMC_UHS_MASK_SDR104	0x10
#define MMC_MASK_HS200		0x20

struct sdhci_tegra_soc_data {
	const struct sdhci_pltfm_data *pdata;
	u32 nvquirks;
};

struct sdhci_tegra {
	const struct sdhci_tegra_soc_data *soc_data;
	int power_gpio;
	bool no_runtime_pm;
	/* max ddr clk supported by the platform */
	unsigned int ddr_clk_limit;
	unsigned int tap_delay;
	unsigned int trim_delay;
	int ddr_trim_delay;
	unsigned int uhs_mask;
	unsigned int calib_3v3_offsets;	/* Format to be filled: 0xXXXXPDPU */
	unsigned int calib_1v8_offsets;	/* Format to be filled: 0xXXXXPDPU */
	unsigned int calib_1v8_offsets_uhs_modes;
	bool set_1v8_calib_offsets;
	bool calib_1v8_offsets_done;
};

static void tegra_sdhci_do_calibration(struct sdhci_host *sdhci);

static u32 tegra_sdhci_readl(struct sdhci_host *host, int reg)
{
	u32 val;

	if (unlikely(reg == SDHCI_PRESENT_STATE)) {
		/* Use wp_gpio here instead? */
		val = readl(host->ioaddr + reg);
		return val | SDHCI_WRITE_PROTECT;
	}

	return readl(host->ioaddr + reg);
}

static u16 tegra_sdhci_readw(struct sdhci_host *host, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;

	if (unlikely((soc_data->nvquirks & NVQUIRK_FORCE_SDHCI_SPEC_200) &&
			(reg == SDHCI_HOST_VERSION))) {
		/* Erratum: Version register is invalid in HW. */
		return SDHCI_SPEC_200;
	}

	return readw(host->ioaddr + reg);
}

static void tegra_sdhci_writel(struct sdhci_host *host, u32 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;

	/* Seems like we're getting spurious timeout and crc errors, so
	 * disable signalling of them. In case of real errors software
	 * timers should take care of eventually detecting them.
	 */
	if (unlikely(reg == SDHCI_SIGNAL_ENABLE))
		val &= ~(SDHCI_INT_TIMEOUT|SDHCI_INT_CRC);

	writel(val, host->ioaddr + reg);

	if (unlikely((soc_data->nvquirks & NVQUIRK_ENABLE_BLOCK_GAP_DET) &&
			(reg == SDHCI_INT_ENABLE))) {
		/* Erratum: Must enable block gap interrupt detection */
		u8 gap_ctrl = readb(host->ioaddr + SDHCI_BLOCK_GAP_CONTROL);
		if (val & SDHCI_INT_CARD_INT)
			gap_ctrl |= 0x8;
		else
			gap_ctrl &= ~0x8;
		writeb(gap_ctrl, host->ioaddr + SDHCI_BLOCK_GAP_CONTROL);
	}
}

static unsigned int tegra_sdhci_get_ro(struct sdhci_host *host)
{
	return mmc_gpio_get_ro(host->mmc);
}

static int tegra_sdhci_set_uhs_signaling(struct sdhci_host *host,
		unsigned int uhs)
{
	u16 clk, ctrl_2;
	u32 vndr_ctrl;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	unsigned int calib_1v8_uhs_modes;

	ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);

	calib_1v8_uhs_modes = tegra_host->calib_1v8_offsets_uhs_modes;
	/* Select Bus Speed Mode for host
	 * For HS200 we need to set UHS_MODE_SEL to SDR104.
	 * It works as SDR 104 in SD 4-bit mode and HS200 in eMMC 8-bit mode.
	 */
	ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
	switch (uhs) {
	case MMC_TIMING_UHS_SDR12:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR12;
		if (calib_1v8_uhs_modes & MMC_1V8_CALIB_OFFSET_SDR12)
			tegra_host->set_1v8_calib_offsets = true;
		break;
	case MMC_TIMING_UHS_SDR25:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR25;
		if (calib_1v8_uhs_modes & MMC_1V8_CALIB_OFFSET_SDR25)
			tegra_host->set_1v8_calib_offsets = true;
		break;
	case MMC_TIMING_UHS_SDR50:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR50;
		if (calib_1v8_uhs_modes & MMC_1V8_CALIB_OFFSET_SDR50)
			tegra_host->set_1v8_calib_offsets = true;
		break;
	case MMC_TIMING_UHS_SDR104:
	case MMC_TIMING_MMC_HS200:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR104;
		if ((calib_1v8_uhs_modes & MMC_1V8_CALIB_OFFSET_SDR104) ||
			(calib_1v8_uhs_modes & MMC_1V8_CALIB_OFFSET_HS200))
			tegra_host->set_1v8_calib_offsets = true;
		break;
	case MMC_TIMING_UHS_DDR50:
		ctrl_2 |= SDHCI_CTRL_UHS_DDR50;
		if (calib_1v8_uhs_modes & MMC_1V8_CALIB_OFFSET_DDR50)
			tegra_host->set_1v8_calib_offsets = true;
		break;
	}

	sdhci_writew(host, ctrl_2, SDHCI_HOST_CONTROL2);

	if (uhs == MMC_TIMING_UHS_DDR50) {
		clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
		clk &= ~(0xFF << SDHCI_DIVIDER_SHIFT);
		clk |= 1 << SDHCI_DIVIDER_SHIFT;
		sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

		/* Set the ddr mode trim delay if required */
		if (tegra_host->ddr_trim_delay != -1) {
			vndr_ctrl = sdhci_readl(host, SDHCI_VNDR_CLK_CTRL);
			vndr_ctrl &= ~(0x1F <<
				SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_SHIFT);
			vndr_ctrl |= (tegra_host->ddr_trim_delay <<
				SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_SHIFT);
			sdhci_writel(host, vndr_ctrl, SDHCI_VNDR_CLK_CTRL);
		}
	}

	if (tegra_host->set_1v8_calib_offsets &&
		!tegra_host->calib_1v8_offsets_done) {
		tegra_sdhci_do_calibration(host);
		tegra_host->calib_1v8_offsets_done = true;
	}
	return 0;
}

static unsigned int get_calibration_offsets(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	unsigned int offsets = 0;

	if (sdhci->mmc->ios.signal_voltage == MMC_SIGNAL_VOLTAGE_330)
		offsets = tegra_host->calib_3v3_offsets;
	else if (sdhci->mmc->ios.signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		offsets = tegra_host->calib_1v8_offsets;
		/*
		 * After any mode selection, ios timing would be set. So, if
		 * ios timing is set but 1.8V calibration offsets requirement
		 * is not set, it indicates that the current mode doesn't
		 * require calibration offsets to be programmed.
		 */
		if (sdhci->mmc->ios.timing &&
			!tegra_host->set_1v8_calib_offsets)
			offsets = 0;
	}

	return offsets;
}

static void tegra_sdhci_do_calibration(struct sdhci_host *sdhci)
{
	unsigned int val;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;
	unsigned int timeout = 10;
	unsigned int calib_offsets;

	if (unlikely(soc_data->nvquirks & NVQUIRK_DISABLE_AUTO_CALIBRATION))
		return;

	val = sdhci_readl(sdhci, SDMMC_SDMEMCOMPPADCTRL);
	val &= ~SDMMC_SDMEMCOMPPADCTRL_VREF_SEL_MASK;
	if (soc_data->nvquirks & NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD)
		val |= SDMMC_SDMEMCOMPPADCTRL_PAD_E_INPUT_OR_E_PWRD_MASK;
	val |= 0x7;
	sdhci_writel(sdhci, val, SDMMC_SDMEMCOMPPADCTRL);

	/* Enable Auto Calibration*/
	val = sdhci_readl(sdhci, SDMMC_AUTO_CAL_CONFIG);
	val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_ENABLE;
	val |= SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_START;
	if (unlikely(soc_data->nvquirks & NVQUIRK_SET_CALIBRATION_OFFSETS)) {
		calib_offsets = get_calibration_offsets(sdhci);
		if (calib_offsets) {
			/* Program Auto cal PD offset(bits 8:14) */
			val &= ~(0x7F <<
				SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET_SHIFT);
			val |= (((calib_offsets >> 8) & 0xFF) <<
				SDMMC_AUTO_CAL_CONFIG_AUTO_CAL_PD_OFFSET_SHIFT);
			/* Program Auto cal PU offset(bits 0:6) */
			val &= ~0x7F;
			val |= (calib_offsets & 0xFF);
		}
	}
	sdhci_writel(sdhci, val, SDMMC_AUTO_CAL_CONFIG);

	/* Wait until the calibration is done */
	do {
		if (!(sdhci_readl(sdhci, SDMMC_AUTO_CAL_STATUS) &
			SDMMC_AUTO_CAL_STATUS_AUTO_CAL_ACTIVE))
			break;

		msleep(1);
		timeout--;
	} while (timeout);

	if (!timeout)
		dev_err(mmc_dev(sdhci->mmc), "Auto calibration failed\n");

	if (soc_data->nvquirks & NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD) {
		val = sdhci_readl(sdhci, SDMMC_SDMEMCOMPPADCTRL);
		val &= ~SDMMC_SDMEMCOMPPADCTRL_PAD_E_INPUT_OR_E_PWRD_MASK;
		sdhci_writel(sdhci, val, SDMMC_SDMEMCOMPPADCTRL);
	}
}

static void tegra_sdhci_reset_exit(struct sdhci_host *host, u8 mask)
{
	u16 misc_ctrl;
	u32 vendor_ctrl;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;
	unsigned int best_tap_value;

	if (mask & SDHCI_RESET_ALL) {
		vendor_ctrl = sdhci_readl(host, SDHCI_VNDR_CLK_CTRL);
		if (soc_data->nvquirks & NVQUIRK_ENABLE_PADPIPE_CLKEN) {
			vendor_ctrl |=
				SDHCI_VNDR_CLK_CTRL_PADPIPE_CLKEN_OVERRIDE;
		}
		if (soc_data->nvquirks & NVQUIRK_DISABLE_SPI_MODE_CLKEN) {
			vendor_ctrl &=
				~SDHCI_VNDR_CLK_CTRL_SPI_MODE_CLKEN_OVERRIDE;
		}
		if (soc_data->nvquirks & NVQUIRK_EN_FEEDBACK_CLK) {
			vendor_ctrl &=
				~SDHCI_VNDR_CLK_CTRL_INPUT_IO_CLK;
		} else {
			vendor_ctrl |= SDHCI_VNDR_CLK_CTRL_INTERNAL_CLK;
		}

		if (soc_data->nvquirks & NVQUIRK_SET_TAP_DELAY) {
			best_tap_value = tegra_host->tap_delay;
			vendor_ctrl &= ~(0xFF <<
				SDHCI_VNDR_CLK_CTRL_TAP_VALUE_SHIFT);
			vendor_ctrl |= (best_tap_value <<
				SDHCI_VNDR_CLK_CTRL_TAP_VALUE_SHIFT);
		}

		if (soc_data->nvquirks & NVQUIRK_SET_TRIM_DELAY) {
			vendor_ctrl &= ~(0x1F <<
			SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_SHIFT);
			vendor_ctrl |= (tegra_host->trim_delay <<
			SDHCI_VNDR_CLK_CTRL_TRIM_VALUE_SHIFT);
		}
		if (soc_data->nvquirks & NVQUIRK_ENABLE_SDR50_TUNING)
			vendor_ctrl |= SDHCI_VNDR_CLK_CTRL_SDR50_TUNING;
		sdhci_writel(host, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);

		misc_ctrl = sdhci_readw(host, SDHCI_TEGRA_VENDOR_MISC_CTRL);
		if (soc_data->nvquirks & NVQUIRK_ENABLE_SD_3_0)
			misc_ctrl |= SDHCI_MISC_CTRL_ENABLE_SDHCI_SPEC_300;
		if (soc_data->nvquirks & NVQUIRK_ENABLE_SDR104) {
			misc_ctrl |=
			SDHCI_VNDR_MISC_CTRL_ENABLE_SDR104_SUPPORT;
		}
		if (soc_data->nvquirks & NVQUIRK_ENABLE_SDR50) {
			misc_ctrl |=
			SDHCI_VNDR_MISC_CTRL_ENABLE_SDR50_SUPPORT;
		}

		if (soc_data->nvquirks & NVQUIRK_ENABLE_DDR50 &&
			!(tegra_host->uhs_mask & MMC_UHS_MASK_DDR50))
				misc_ctrl |=
				SDHCI_VNDR_MISC_CTRL_ENABLE_DDR50_SUPPORT;

		if (soc_data->nvquirks & NVQUIRK_INFINITE_ERASE_TIMEOUT) {
			misc_ctrl |=
			SDHCI_VNDR_MISC_CTRL_INFINITE_ERASE_TIMEOUT;
		}
		if (soc_data->nvquirks & NVQUIRK_SET_PIPE_STAGES_MASK_0)
			misc_ctrl &= ~SDHCI_VNDR_MISC_CTRL_PIPE_STAGES_MASK;
		sdhci_writew(host, misc_ctrl, SDHCI_TEGRA_VENDOR_MISC_CTRL);

		if (soc_data->nvquirks & NVQUIRK_DISABLE_AUTO_CMD23)
			host->flags &= ~SDHCI_AUTO_CMD23;

		if (tegra_host->uhs_mask & MMC_UHS_MASK_SDR104)
			host->mmc->caps &= ~MMC_CAP_UHS_SDR104;

		if (tegra_host->uhs_mask & MMC_UHS_MASK_DDR50)
			host->mmc->caps &= ~MMC_CAP_UHS_DDR50;

		if (tegra_host->uhs_mask & MMC_UHS_MASK_SDR50)
			host->mmc->caps &= ~MMC_CAP_UHS_SDR50;

		if (tegra_host->uhs_mask & MMC_UHS_MASK_SDR25)
			host->mmc->caps &= ~MMC_CAP_UHS_SDR25;

		if (tegra_host->uhs_mask & MMC_UHS_MASK_SDR12)
			host->mmc->caps &= ~MMC_CAP_UHS_SDR12;

		if (tegra_host->uhs_mask & MMC_MASK_HS200)
			host->mmc->caps2 &= ~MMC_CAP2_HS200;
	}
}

static int tegra_sdhci_buswidth(struct sdhci_host *host, int bus_width)
{
	u32 ctrl;

	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	if ((host->mmc->caps & MMC_CAP_8_BIT_DATA) &&
	    (bus_width == MMC_BUS_WIDTH_8)) {
		ctrl &= ~SDHCI_CTRL_4BITBUS;
		ctrl |= SDHCI_CTRL_8BITBUS;
	} else {
		ctrl &= ~SDHCI_CTRL_8BITBUS;
		if (bus_width == MMC_BUS_WIDTH_4)
			ctrl |= SDHCI_CTRL_4BITBUS;
		else
			ctrl &= ~SDHCI_CTRL_4BITBUS;
	}
	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
	return 0;
}

static const struct sdhci_ops tegra_sdhci_ops = {
	.get_ro     = tegra_sdhci_get_ro,
	.read_l     = tegra_sdhci_readl,
	.read_w     = tegra_sdhci_readw,
	.write_l    = tegra_sdhci_writel,
	.platform_bus_width = tegra_sdhci_buswidth,
	.platform_reset_exit = tegra_sdhci_reset_exit,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.set_uhs_signaling	= tegra_sdhci_set_uhs_signaling,
};

static const struct sdhci_pltfm_data sdhci_tegra20_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra20 = {
	.pdata = &sdhci_tegra20_pdata,
	.nvquirks = NVQUIRK_FORCE_SDHCI_SPEC_200 |
		    NVQUIRK_ENABLE_BLOCK_GAP_DET,
};

static const struct sdhci_pltfm_data sdhci_tegra30_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra30 = {
	.pdata = &sdhci_tegra30_pdata,
	.nvquirks = NVQUIRK_ENABLE_SDHCI_SPEC_300,
};

static const struct sdhci_pltfm_data sdhci_tegra114_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra114 = {
	.pdata = &sdhci_tegra114_pdata,
};

static const struct sdhci_pltfm_data sdhci_tegra132_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra132 = {
	.pdata = &sdhci_tegra132_pdata,
	.nvquirks = NVQUIRK_ENABLE_HS200,
};

static const struct of_device_id sdhci_tegra_dt_match[] = {
	{ .compatible = "nvidia,tegra132-sdhci", .data = &soc_data_tegra132 },
	{ .compatible = "nvidia,tegra124-sdhci", .data = &soc_data_tegra114 },
	{ .compatible = "nvidia,tegra114-sdhci", .data = &soc_data_tegra114 },
	{ .compatible = "nvidia,tegra30-sdhci", .data = &soc_data_tegra30 },
	{ .compatible = "nvidia,tegra20-sdhci", .data = &soc_data_tegra20 },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_tegra_dt_match);

static int sdhci_tegra_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	int ret;

	tegra_host->power_gpio = of_get_named_gpio(np, "power-gpios", 0);
	ret = mmc_of_parse(host->mmc);

	if (of_get_property(np, "nvidia,no-runtime-suspend", NULL))
		tegra_host->no_runtime_pm = true;

	if (of_get_property(np, "nvidia,host-off-card-on", NULL))
		host->quirks2 |= SDHCI_QUIRK2_HOST_OFF_CARD_ON;

	if (of_get_property(np, "no-1-8-v", NULL))
		host->quirks2 |= SDHCI_QUIRK2_NO_1_8_V;

	of_property_read_u32(np, "tap-delay", &tegra_host->tap_delay);
	of_property_read_u32(np, "trim-delay", &tegra_host->trim_delay);
	of_property_read_u32(np, "ddr-clk-limit", &tegra_host->ddr_clk_limit);
	of_property_read_u32(np, "uhs-mask", &tegra_host->uhs_mask);
	of_property_read_u32(np,
		"calib-3v3-offsets", &tegra_host->calib_3v3_offsets);
	of_property_read_u32(np,
		"calib-1v8-offsets", &tegra_host->calib_1v8_offsets);
	of_property_read_u32(np, "calib-1v8-offsets-uhs-modes",
		&tegra_host->calib_1v8_offsets_uhs_modes);

	if (of_property_read_u32(np, "ddr-trim-delay",
			&tegra_host->ddr_trim_delay))
		tegra_host->ddr_trim_delay = -1;

	return ret;
}

static int sdhci_tegra_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	const struct sdhci_tegra_soc_data *soc_data;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_tegra *tegra_host;
	struct clk *clk;
	int rc;

	match = of_match_device(sdhci_tegra_dt_match, &pdev->dev);
	if (!match)
		return -EINVAL;
	soc_data = match->data;

	host = sdhci_pltfm_init(pdev, soc_data->pdata, 0);
	if (IS_ERR(host))
		return PTR_ERR(host);
	pltfm_host = sdhci_priv(host);

	tegra_host = devm_kzalloc(&pdev->dev, sizeof(*tegra_host), GFP_KERNEL);
	if (!tegra_host) {
		dev_err(mmc_dev(host->mmc), "failed to allocate tegra_host\n");
		rc = -ENOMEM;
		goto err_alloc_tegra_host;
	}
	tegra_host->soc_data = soc_data;
	pltfm_host->priv = tegra_host;

	rc = sdhci_tegra_parse_dt(&pdev->dev);
	if (rc)
		goto err_parse_dt;

	if (gpio_is_valid(tegra_host->power_gpio)) {
		rc = gpio_request(tegra_host->power_gpio, "sdhci_power");
		if (rc) {
			dev_err(mmc_dev(host->mmc),
				"failed to allocate power gpio\n");
			goto err_power_req;
		}
		gpio_direction_output(tegra_host->power_gpio, 1);
	}

	clk = clk_get(mmc_dev(host->mmc), NULL);
	if (IS_ERR(clk)) {
		dev_err(mmc_dev(host->mmc), "clk err\n");
		rc = PTR_ERR(clk);
		goto err_clk_get;
	}
	clk_prepare_enable(clk);
	pltfm_host->clk = clk;

	pm_runtime_set_active(&pdev->dev);
	if (!tegra_host->no_runtime_pm)
		pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev,
					 TEGRA_SDHCI_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_suspend_ignore_children(&pdev->dev, true);

	if (soc_data->nvquirks & NVQUIRK_ENABLE_HS200)
		host->mmc->caps2 |= MMC_CAP2_HS200;

	rc = sdhci_add_host(host);
	if (rc)
		goto err_add_host;

	pm_runtime_put_autosuspend(&pdev->dev);

	return 0;

err_add_host:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	clk_disable_unprepare(pltfm_host->clk);
	clk_put(pltfm_host->clk);
err_clk_get:
	if (gpio_is_valid(tegra_host->power_gpio))
		gpio_free(tegra_host->power_gpio);
err_power_req:
err_parse_dt:
err_alloc_tegra_host:
	sdhci_pltfm_free(pdev);
	return rc;
}

static int sdhci_tegra_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	int dead = (readl(host->ioaddr + SDHCI_INT_STATUS) == 0xffffffff);

	pm_runtime_get_sync(&pdev->dev);
	sdhci_remove_host(host, dead);
	pm_runtime_disable(&pdev->dev);

	if (gpio_is_valid(tegra_host->power_gpio))
		gpio_free(tegra_host->power_gpio);

	clk_disable_unprepare(pltfm_host->clk);
	clk_put(pltfm_host->clk);

	sdhci_pltfm_free(pdev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_sdhci_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	int ret;

	pm_runtime_get_sync(dev);

	ret = sdhci_suspend_host(host);
	clk_disable_unprepare(pltfm_host->clk);
	if (!mmc_card_keep_power(host->mmc))
		if (gpio_is_valid(tegra_host->power_gpio))
			gpio_direction_output(tegra_host->power_gpio, 0);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

static int tegra_sdhci_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	int ret;

	pm_runtime_get_sync(dev);

	if (!mmc_card_keep_power(host->mmc))
		if (gpio_is_valid(tegra_host->power_gpio))
			gpio_direction_output(tegra_host->power_gpio, 1);
	clk_prepare_enable(pltfm_host->clk);
	ret = sdhci_resume_host(host);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int tegra_sdhci_runtime_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	int ret;

	ret = sdhci_runtime_suspend_host(host);
	clk_disable_unprepare(pltfm_host->clk);

	return ret;
}

static int tegra_sdhci_runtime_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	int ret;

	clk_prepare_enable(pltfm_host->clk);
	ret = sdhci_runtime_resume_host(host);

	return ret;
}
#endif

static const struct dev_pm_ops sdhci_tegra_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tegra_sdhci_suspend, tegra_sdhci_resume)
	SET_RUNTIME_PM_OPS(tegra_sdhci_runtime_suspend,
			   tegra_sdhci_runtime_resume, NULL)
};

static struct platform_driver sdhci_tegra_driver = {
	.driver		= {
		.name	= "sdhci-tegra",
		.owner	= THIS_MODULE,
		.of_match_table = sdhci_tegra_dt_match,
		.pm	= &sdhci_tegra_pm_ops,
	},
	.probe		= sdhci_tegra_probe,
	.remove		= sdhci_tegra_remove,
};

module_platform_driver(sdhci_tegra_driver);

MODULE_DESCRIPTION("SDHCI driver for Tegra");
MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL v2");
