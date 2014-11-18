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
#include <linux/slab.h>
#include <soc/tegra/tegra-dvfs.h>

#include "sdhci-pltfm.h"

/* Tegra SDHOST controller vendor register definitions */
#define SDHCI_VNDR_CLK_CTRL	0x100
#define SDHCI_VNDR_CLK_CTRL_SDMMC_CLK	0x1
#define SDHCI_VNDR_CLK_CTRL_PADPIPE_CLKEN_OVERRIDE	0x8
#define SDHCI_VNDR_CLK_CTRL_SPI_MODE_CLKEN_OVERRIDE	0x4
#define SDHCI_VNDR_CLK_CTRL_INPUT_IO_CLK		0x2
#define SDHCI_VNDR_CLK_CTRL_BASE_CLK_FREQ_SHIFT	8
#define SDHCI_VNDR_CLK_CTRL_BASE_CLK_FREQ_MASK	0xff
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
#define SDHCI_VNDR_MISC_CTRL_EN_EXT_LOOPBACK	0x20000

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

/* Tuning related definitions */
#define MMC_TUNING_BLOCK_SIZE_BUS_WIDTH_8	128
#define MMC_TUNING_BLOCK_SIZE_BUS_WIDTH_4	64
#define MAX_TAP_VALUES		255
#define MAX_TAP_WINDOWS		10
#define MIN_TAP_WINDOW_SIZE	5
#define TUNING_RETRIES		1
#define TUNING_OP_TIMEOUT	5
#define TUNING_INHIBIT_TIMEOUT	10

#define RESET_TIMEOUT	100

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

struct tuning_t2t_coeffs {
	int vmax;
	int vmin;
	unsigned int t2t_vnom_slope;
	unsigned int t2t_vnom_int;
	unsigned int t2t_vmax_slope;
	unsigned int t2t_vmax_int;
	unsigned int t2t_vmin_slope;
	unsigned int t2t_vmin_int;
};

struct tap_hole_coeffs {
	unsigned int freq_hz;
	unsigned int thole_vnom_slope;
	unsigned int thole_vnom_int;
	unsigned int thole_vmax_slope;
	unsigned int thole_vmax_int;
	unsigned int thole_vmin_slope;
	unsigned int thole_vmin_int;
};

struct tuning_ui {
	int ui;
	bool is_valid_ui;
};

enum tap_win_edge_attr {
	WIN_EDGE_BOUND_START,
	WIN_EDGE_BOUND_END,
	WIN_EDGE_HOLE,
};

struct tap_window_data {
	int win_start;
	int win_end;
	enum tap_win_edge_attr win_start_attr;
	enum tap_win_edge_attr win_end_attr;
	int win_size;
};

struct tuning_values {
	int t2t_vmax;
	int t2t_vmin;
	int ui;
	int ui_vmin;
	int vmax_thole;
	int vmin_thole;
};

struct tegra_tuning_data {
	int best_tap_value;
	struct tuning_values est_values;
	struct tuning_values calc_values;
	struct tap_window_data tap_data[MAX_TAP_WINDOWS];
	int num_of_valid_tap_wins;
	bool is_partial_win_valid;
};

struct sdhci_tegra {
	const struct sdhci_tegra_soc_data *soc_data;
	int power_gpio;
	bool disable_ext_loopback;
	bool non_removable;
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
	/* Tuning related structures and variables */
	/* Tap-to-tap tuning coefficients */
	struct tuning_t2t_coeffs t2t_coeffs;
	/* Tap hole tuning coefficients (per tuning frequency) */
	struct tap_hole_coeffs *thole_coeffs;
	int nr_tuning_freqs;
	int tuning_freq_idx;
	/* Tuning opcode to be used */
	unsigned int tuning_opcode;
	/* Tuning packet size */
	unsigned int tuning_bsize;
	/* Tuning status */
	bool tuning_done;
	/* Freq tuning information for each sampling clock freq */
	struct tegra_tuning_data tuning_data;
	u32 speedo;
	int boot_vcore_mv;
	unsigned long current_clk;
};

static void tegra_sdhci_do_calibration(struct sdhci_host *sdhci);
static void tegra_sdhci_reset(struct sdhci_host *sdhci, u8 mask);

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

static void tegra_sdhci_clock_enable(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	u8 ctrl;

	clk_prepare_enable(pltfm_host->clk);
	ctrl = sdhci_readb(sdhci, SDHCI_VNDR_CLK_CTRL);
	ctrl |= SDHCI_VNDR_CLK_CTRL_SDMMC_CLK;
	sdhci_writeb(sdhci, ctrl, SDHCI_VNDR_CLK_CTRL);
}

static void tegra_sdhci_clock_disable(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	u8 ctrl;

	ctrl = sdhci_readb(sdhci, SDHCI_VNDR_CLK_CTRL);
	ctrl &= ~SDHCI_VNDR_CLK_CTRL_SDMMC_CLK;
	sdhci_writeb(sdhci, ctrl, SDHCI_VNDR_CLK_CTRL);
	clk_disable_unprepare(pltfm_host->clk);
}

static void tegra_sdhci_init_clock(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	unsigned long rate;
	u32 val;
	int i;

	if (!sdhci->mmc->f_max) {
		/*
		 * If "max-frequency" wasn't specified in the DT, use the
		 * maximum available from the tuning data table.
		 */
		tegra_host->tuning_freq_idx = tegra_host->nr_tuning_freqs - 1;
	} else {
		for (i = 0; i < tegra_host->nr_tuning_freqs; i++) {
			if (tegra_host->thole_coeffs[i].freq_hz >
			    sdhci->mmc->f_max)
				break;
		}
		tegra_host->tuning_freq_idx = i ? i - 1 : 0;
	}

	/*
	 * Set the source clock to the maximum frequency.
	 * The divider in SDHCI_CLOCK_CONTROL will be used to scale down
	 * the clock later in sdhci_set_clock().
	 */
	rate = tegra_host->thole_coeffs[tegra_host->tuning_freq_idx].freq_hz;
	clk_set_rate(pltfm_host->clk, rate);
	rate = clk_get_rate(pltfm_host->clk);
	tegra_host->current_clk = rate;

	/*
	 * Program BASE_CLK_FREQ so that sdhci_add_host() reads the correct
	 * clock frequency in SDHCI_CAPABILITIES.
	 */
	val = sdhci_readl(sdhci, SDHCI_VNDR_CLK_CTRL);
	rate /= 1000000;
	val &= ~(SDHCI_VNDR_CLK_CTRL_BASE_CLK_FREQ_MASK <<
		 SDHCI_VNDR_CLK_CTRL_BASE_CLK_FREQ_SHIFT);
	val |= (rate & SDHCI_VNDR_CLK_CTRL_BASE_CLK_FREQ_MASK) <<
		SDHCI_VNDR_CLK_CTRL_BASE_CLK_FREQ_SHIFT;
	sdhci_writel(sdhci, val, SDHCI_VNDR_CLK_CTRL);
}

static void tegra_sdhci_set_tap_delay(struct sdhci_host *sdhci,
	unsigned int tap_delay)
{
	u32 vendor_ctrl;

	/* Max tap delay value is 255 */
	if (WARN_ON(tap_delay > MAX_TAP_VALUES))
		return;

	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VNDR_CLK_CTRL);
	vendor_ctrl &= ~(0xFF << SDHCI_VNDR_CLK_CTRL_TAP_VALUE_SHIFT);
	vendor_ctrl |= (tap_delay << SDHCI_VNDR_CLK_CTRL_TAP_VALUE_SHIFT);
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VNDR_CLK_CTRL);
}

static int tegra_sdhci_issue_tuning_cmd(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	int err = 0;
	u8 ctrl;
	u32 mask;
	unsigned int timeout = TUNING_INHIBIT_TIMEOUT;
	int flags;
	u32 intstatus;
	int timeout_val;

	mask = SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT;
	while (sdhci_readl(sdhci, SDHCI_PRESENT_STATE) & mask) {
		if (timeout == 0) {
			dev_err(mmc_dev(sdhci->mmc),
				"Controller never released inhibit bit(s).\n");
			err = -ETIMEDOUT;
			goto out;
		}
		timeout--;
		usleep_range(1000, 1100);
	}

	ctrl = sdhci_readb(sdhci, SDHCI_HOST_CONTROL2);
	ctrl &= ~SDHCI_CTRL_TUNED_CLK;
	sdhci_writeb(sdhci, ctrl, SDHCI_HOST_CONTROL2);

	ctrl = sdhci_readb(sdhci, SDHCI_HOST_CONTROL2);
	ctrl |= SDHCI_CTRL_EXEC_TUNING;
	sdhci_writeb(sdhci, ctrl, SDHCI_HOST_CONTROL2);

	/*
	 * In response to CMD19, the card sends 64 bytes of tuning
	 * block to the Host Controller. So we set the block size
	 * to 64 here.
	 * In response to CMD21, the card sends 128 bytes of tuning
	 * block for MMC_BUS_WIDTH_8 and 64 bytes for MMC_BUS_WIDTH_4
	 * to the Host Controller. So we set the block size to 64 here.
	 */
	sdhci_writew(sdhci, SDHCI_MAKE_BLKSZ(7, tegra_host->tuning_bsize),
		SDHCI_BLOCK_SIZE);

	/*
	 * Choose a HW timeout value to match what we'll wait for before
	 * timing out in SW.  The hardware timeout value is equal to:
	 *    2 << (SDHCI_TIMEOUT_CONTROL + 13) / SDCLK
	 * where SDHCI_TIMEOUT_CONTROL is between 0 and 14.
	 */
	timeout_val = TUNING_OP_TIMEOUT * (tegra_host->current_clk / 1000);
	timeout_val = clamp(fls(timeout_val) - 13, 0, 14);
	sdhci_writeb(sdhci, timeout_val, SDHCI_TIMEOUT_CONTROL);

	sdhci_writew(sdhci, SDHCI_TRNS_READ, SDHCI_TRANSFER_MODE);

	sdhci_writel(sdhci, 0x0, SDHCI_ARGUMENT);

	/* Set the cmd flags */
	flags = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_DATA;
	/* Issue the command */
	sdhci_writew(sdhci, SDHCI_MAKE_CMD(
		tegra_host->tuning_opcode, flags), SDHCI_COMMAND);

	timeout = TUNING_OP_TIMEOUT;
	do {
		usleep_range(1000, 1100);
		intstatus = sdhci_readl(sdhci, SDHCI_INT_STATUS);
		if (intstatus) {
			sdhci_writel(sdhci, intstatus, SDHCI_INT_STATUS);
			break;
		}
	} while (--timeout > 0);
	if (timeout == 0) {
		dev_err(mmc_dev(sdhci->mmc),
			"Timed out waiting for interrupt\n");
		return -ETIMEDOUT;
	}

	if ((intstatus & SDHCI_INT_DATA_AVAIL) &&
		!(intstatus & SDHCI_INT_DATA_CRC)) {
		err = 0;
		ctrl = sdhci_readb(sdhci, SDHCI_HOST_CONTROL2);
		if (!(ctrl & SDHCI_CTRL_EXEC_TUNING) &&
			(ctrl & SDHCI_CTRL_TUNED_CLK))
			err = 0;
		else
			err = -EIO;
	} else {
		tegra_sdhci_reset(sdhci, SDHCI_RESET_CMD);
		tegra_sdhci_reset(sdhci, SDHCI_RESET_DATA);
		err = -EIO;
	}

	usleep_range(1000, 1100);
out:
	return err;
}

static int tegra_sdhci_scan_tap_values(struct sdhci_host *sdhci,
		unsigned int starting_tap, bool expect_failure)
{
	unsigned int tap_value = starting_tap;
	int err;
	unsigned int retry = TUNING_RETRIES;

	do {
		/* Set the tap delay */
		tegra_sdhci_set_tap_delay(sdhci, tap_value);

		/* Run frequency tuning */
		err = tegra_sdhci_issue_tuning_cmd(sdhci);
		if (err == -ETIMEDOUT)
			return err;
		if (err && retry) {
			retry--;
			continue;
		} else {
			retry = TUNING_RETRIES;
			if ((expect_failure && !err) ||
				(!expect_failure && err))
				break;
		}
		tap_value++;
	} while (tap_value <= MAX_TAP_VALUES);

	return tap_value;
}

/*
 * Estimate tap hole locations based on tap-to-tap and tap hole coefficients.
 */
static void tegra_sdhci_estimate_tuning_values(struct sdhci_host *sdhci,
				struct tegra_tuning_data *tuning_data)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct tuning_t2t_coeffs *t2t_coeffs = &tegra_host->t2t_coeffs;
	struct tap_hole_coeffs *thole_coeffs =
		&tegra_host->thole_coeffs[tegra_host->tuning_freq_idx];
	struct tuning_values *est_values = &tuning_data->est_values;
	int speedo = tegra_host->speedo;
	int voltage_mv = tegra_host->boot_vcore_mv;
	int slope, inpt;
	int vmax_t2t, vmin_t2t;
	int vmax_thole, vmin_thole;

	/* Est_T2T_Vmax = (speedo*(-t2t_slope)+t2t_int */
	vmax_t2t = (t2t_coeffs->t2t_vmax_int - (speedo *
		t2t_coeffs->t2t_vmax_slope)) / 1000;
	vmin_t2t = (t2t_coeffs->t2t_vmin_int - (speedo *
		t2t_coeffs->t2t_vmin_slope)) / 1000;
	est_values->t2t_vmin = vmin_t2t;

	if (voltage_mv == t2t_coeffs->vmin) {
		est_values->t2t_vmax = vmin_t2t;
	} else if (voltage_mv == t2t_coeffs->vmax) {
		est_values->t2t_vmax = vmax_t2t;
	} else {
		vmax_t2t = 1000 / vmax_t2t;
		vmin_t2t = 1000 / vmin_t2t;
		/*
		 * For any intermediate voltage between 0.95V and 1.25V,
		 * calculate the slope and intercept from the T2T and tap hole
		 * values of 0.95V and 1.25V and use them to calculate the
		 * actual values. 1/T2T is a linear function of voltage.
		 */
		slope = ((vmax_t2t - vmin_t2t) * 1000) /
			(t2t_coeffs->vmax - t2t_coeffs->vmin);
		inpt = (vmax_t2t * 1000 - (slope * t2t_coeffs->vmax)) / 1000;
		est_values->t2t_vmax = (slope * voltage_mv) / 1000 + inpt;
		est_values->t2t_vmax = (1000 / est_values->t2t_vmax);
	}

	/* Est_UI  = (1000000/freq_MHz)/Est_T2T_Vmax */
	est_values->ui = (1000000 / (thole_coeffs->freq_hz / 1000000)) /
		est_values->t2t_vmax;

	/*
	 * Est_1'st_hole = (Est_T2T_Vmax*(-thole_slope)) + thole_int.
	 */
	vmax_thole = (thole_coeffs->thole_vmax_int -
		(thole_coeffs->thole_vmax_slope * est_values->t2t_vmax)) / 1000;
	vmin_thole = (thole_coeffs->thole_vmin_int -
		(thole_coeffs->thole_vmin_slope * est_values->t2t_vmax)) / 1000;

	if (voltage_mv == t2t_coeffs->vmin) {
		est_values->vmax_thole = vmin_thole;
	} else if (voltage_mv == t2t_coeffs->vmax) {
		est_values->vmax_thole = vmax_thole;
	} else {
		/*
		 * For any intermediate voltage between 0.95V and 1.25V,
		 * calculate the slope and intercept from the t2t and tap hole
		 * values of 0.95V and 1.25V and use them to calculate the
		 * actual values. Tap hole is a linear function of voltage.
		 */
		slope = ((vmax_thole - vmin_thole) * 1000) /
			(t2t_coeffs->vmax - t2t_coeffs->vmin);
		inpt = (vmax_thole * 1000 - (slope * t2t_coeffs->vmax)) / 1000;
		est_values->vmax_thole = (slope * voltage_mv) / 1000 + inpt;
	}
	est_values->vmin_thole = vmin_thole;
}

/*
 * Calculate tap hole locations at Vmax.
 */
static void tegra_sdhci_calculate_vmax_values(struct sdhci_host *sdhci,
				struct tegra_tuning_data *tuning_data)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct tuning_t2t_coeffs *t2t_coeffs = &tegra_host->t2t_coeffs;
	struct tap_hole_coeffs *thole_coeffs =
		&tegra_host->thole_coeffs[tegra_host->tuning_freq_idx];
	struct tuning_values *calc_values = &tuning_data->calc_values;
	struct tap_window_data *tap_data;
	int voltage_mv = tegra_host->boot_vcore_mv;
	int slope, inpt;
	int vmax_thole, vmin_thole;
	u8 i;

	/* T2T_Vmax = (1000000/freq_MHz)/Calc_UI */
	calc_values->t2t_vmax = (1000000 / (thole_coeffs->freq_hz / 1000000)) /
		calc_values->ui;

	/*
	 * Interpolate the tap hole.
	 * Vmax_1'st_hole = (Calc_T2T_Vmax*(-thole_slope)+thole_tint.
	 */
	vmax_thole = (thole_coeffs->thole_vmax_int -
		(thole_coeffs->thole_vmax_slope * calc_values->t2t_vmax)) /
		1000;
	vmin_thole = (thole_coeffs->thole_vmin_int -
		(thole_coeffs->thole_vmin_slope * calc_values->t2t_vmax)) /
		1000;
	if (voltage_mv == t2t_coeffs->vmin) {
		calc_values->vmax_thole = vmin_thole;
	} else if (voltage_mv == t2t_coeffs->vmax) {
		calc_values->vmax_thole = vmax_thole;
	} else {
		slope = (vmax_thole - vmin_thole) /
			(t2t_coeffs->vmax - t2t_coeffs->vmin);
		inpt = ((vmax_thole * 1000) - (slope * 1250)) / 1000;
		calc_values->vmax_thole = slope * voltage_mv + inpt;
	}

	/* Calculate negative margin if partial win is valid */
	if (tuning_data->is_partial_win_valid) {
		/* Find second boundary start and adjust partial win start */
		for (i = 1; i < tuning_data->num_of_valid_tap_wins; i++) {
			tap_data = &tuning_data->tap_data[i];
			if (tap_data->win_start_attr == WIN_EDGE_BOUND_START) {
				tuning_data->tap_data[0].win_start =
					(tap_data->win_start - calc_values->ui);
				break;
			}
		}
	}
}

/*
 * Insert the calculated holes and get the final tap windows with the
 * boundaries and holes set.
 */
static void tegra_sdhci_insert_tap_holes(struct sdhci_host *sdhci,
	struct tegra_tuning_data *tuning_data)
{
	struct tap_window_data *tap_data, *final_tap_data;
	struct tap_window_data temp_tap_data[MAX_TAP_WINDOWS];
	struct tuning_values *calc_values = &tuning_data->calc_values;
	int tap_hole, next_tap_hole;
	int i = 0, j = 0;

	tap_hole = calc_values->vmax_thole;
	do {
		tap_data = &tuning_data->tap_data[i];
		final_tap_data = &temp_tap_data[j];
		if (tap_hole < tap_data->win_start) {
			tap_hole += calc_values->ui;
		} else if (tap_hole > tap_data->win_end) {
			memcpy(final_tap_data, tap_data,
				sizeof(*final_tap_data));
			i++;
			j++;
		} else if ((tap_hole >= tap_data->win_start) &&
			(tap_hole <= tap_data->win_end)) {
			/* Split the window around the tap holes */
			if (tap_hole > tap_data->win_start) {
				final_tap_data->win_start = tap_data->win_start;
				final_tap_data->win_start_attr =
					tap_data->win_start_attr;
				final_tap_data->win_end = tap_hole - 1;
				final_tap_data->win_end_attr = WIN_EDGE_HOLE;
				j++;
			}

			next_tap_hole = tap_hole + calc_values->ui;
			while (next_tap_hole <= tap_data->win_end &&
				j < MAX_TAP_WINDOWS) {
				final_tap_data = &temp_tap_data[j];
				final_tap_data->win_start = tap_hole + 1;
				final_tap_data->win_start_attr = WIN_EDGE_HOLE;
				final_tap_data->win_end = next_tap_hole - 1;
				final_tap_data->win_end_attr = WIN_EDGE_HOLE;
				j++;
				tap_hole = next_tap_hole;
				next_tap_hole += calc_values->ui;
			}

			if (tap_hole < tap_data->win_end &&
				j < MAX_TAP_WINDOWS) {
				final_tap_data = &temp_tap_data[j];
				final_tap_data->win_start = tap_hole + 1;
				final_tap_data->win_start_attr = WIN_EDGE_HOLE;
				final_tap_data->win_end = tap_data->win_end;
				final_tap_data->win_end_attr =
					tap_data->win_end_attr;
				j++;
			}
			i++;
			tap_hole = next_tap_hole;
		}
	} while (i < tuning_data->num_of_valid_tap_wins && j < MAX_TAP_WINDOWS);

	/* Update the num of valid wins count after tap holes insertion */
	tuning_data->num_of_valid_tap_wins = j;
	memcpy(tuning_data->tap_data, temp_tap_data,
		sizeof(tuning_data->tap_data));
}

/*
 * Scan for all tap values and get all passing tap windows.
 */
static int tegra_sdhci_get_tap_window_data(struct sdhci_host *sdhci,
	struct tegra_tuning_data *tuning_data)
{
	struct tap_window_data *tap_data;
	struct tuning_ui tuning_ui[MAX_TAP_WINDOWS];
	int calc_ui = 0, ref_ui = 0;
	int tap_value, prev_boundary_end = 0;
	int num_of_wins = 0, num_of_uis = 0, valid_num_uis = 0;
	int i;

	tap_value = 0;
	do {
		tap_data = &tuning_data->tap_data[num_of_wins];
		/* Get the window start */
		tap_value = tegra_sdhci_scan_tap_values(sdhci, tap_value, true);
		if (tap_value < 0)
			return tap_value;
		tap_data->win_start = min_t(unsigned int, tap_value,
					    MAX_TAP_VALUES);
		tap_value++;
		if (tap_value >= MAX_TAP_VALUES) {
			/* If it's first iteration, then all taps failed */
			if (!num_of_wins) {
				dev_err(mmc_dev(sdhci->mmc),
					"All tap values failed\n");
				return -EINVAL;
			} else {
				/* All windows obtained */
				break;
			}
		}

		/* Get the window end */
		tap_value = tegra_sdhci_scan_tap_values(sdhci,
				tap_value, false);
		if (tap_value < 0)
			return tap_value;
		tap_data->win_end = min_t(int, (tap_value - 1), MAX_TAP_VALUES);
		tap_data->win_size = tap_data->win_end - tap_data->win_start;
		tap_value++;

		/*
		 * If the size of window is more than 4 taps wide, then it is a
		 * valid window. If tap value 0 has passed, then a partial
		 * window exists. Mark all the window edges as boundary edges.
		 */
		if (tap_data->win_size >= MIN_TAP_WINDOW_SIZE) {
			if (tap_data->win_start == 0)
				tuning_data->is_partial_win_valid = true;
			tap_data->win_start_attr = WIN_EDGE_BOUND_START;
			tap_data->win_end_attr = WIN_EDGE_BOUND_END;
		} else {
			/* Invalid window as size is less than 5 taps */
			continue;
		}

		/* Ignore first and last partial UIs */
		if (tap_data->win_end_attr == WIN_EDGE_BOUND_END) {
			if (prev_boundary_end &&
				(tap_data->win_end != MAX_TAP_VALUES)) {
				tuning_ui[num_of_uis].ui = tap_data->win_end -
					prev_boundary_end;
				num_of_uis++;
			}
			prev_boundary_end = tap_data->win_end;
		}
		num_of_wins++;
	} while (tap_value < MAX_TAP_VALUES && num_of_wins < MAX_TAP_WINDOWS);

	tuning_data->num_of_valid_tap_wins = num_of_wins;
	valid_num_uis = num_of_uis;

	/* Calculate 0.75*est_UI */
	ref_ui = (75 * tuning_data->est_values.ui) / 100;

	/*
	 * Check for valid UIs and discredit invalid UIs. A UI is considered
	 * valid if it's greater than (0.75*est_UI). If an invalid UI is found,
	 * also discredit the smaller of the two adjacent windows.
	 */
	for (i = 0; i < num_of_uis; i++) {
		if (tuning_ui[i].ui > ref_ui) {
			tuning_ui[i].is_valid_ui = true;
		} else {
			tuning_ui[i].is_valid_ui = false;
			valid_num_uis--;
			/* Compare the adjacent uis */
			if (i > 0) {
				if (tuning_ui[i - 1].ui > tuning_ui[i + 1].ui) {
					tuning_ui[i + 1].is_valid_ui = false;
					i++;
				} else {
					tuning_ui[i - 1].is_valid_ui = false;
				}
				valid_num_uis--;
			}
		}
	}

	/* Calculate the cumulative UI if there are valid UIs left */
	if (valid_num_uis) {
		for (i = 0; i < num_of_uis; i++)
			if (tuning_ui[i].is_valid_ui)
				calc_ui += tuning_ui[i].ui;
	}

	if (calc_ui)
		tuning_data->calc_values.ui = (calc_ui / valid_num_uis);
	else
		tuning_data->calc_values.ui = tuning_data->est_values.ui;

	/* Calculate location of tap holes at Vmax */
	tegra_sdhci_calculate_vmax_values(sdhci, tuning_data);

	/* Insert calculated holes into the windows */
	tegra_sdhci_insert_tap_holes(sdhci, tuning_data);

	return 0;
}

/*
 * Calculate tap hole locations at the given Vmin.
 */
static void tegra_sdhci_calculate_vmin_values(struct sdhci_host *sdhci,
	struct tegra_tuning_data *tuning_data, int vmin)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct tuning_values *est_values = &tuning_data->est_values;
	struct tuning_values *calc_values = &tuning_data->calc_values;
	struct tuning_t2t_coeffs *t2t_coeffs = &tegra_host->t2t_coeffs;
	struct tap_hole_coeffs *thole_coeffs =
		&tegra_host->thole_coeffs[tegra_host->tuning_freq_idx];
	int boot_mv = tegra_host->boot_vcore_mv;
	int vmin_slope, vmin_int, temp_calc_vmin;
	int t2t_vmax, t2t_vmin;
	int vmax_thole, vmin_thole;

	/*
	 * If current vmin is equal to vmin or vmax of tuning data, use the
	 * previously calculated estimated T2T values directly. Note that the
	 * estimated T2T_vmax is not at Vmax specified in tuning data. It is
	 * the T2T at the boot or max voltage for the current SKU. Hence,
	 * boot_mv is used in place of t2t_coeffs->vmax.
	 */
	if (vmin == t2t_coeffs->vmin) {
		t2t_vmin = est_values->t2t_vmin;
	} else if (vmin == boot_mv) {
		t2t_vmin = est_values->t2t_vmax;
	} else {
		/*
		 * For any intermediate voltage between boot voltage and vmin
		 * of tuning data, calculate the slope and intercept from the
		 * t2t at boot_mv and vmin and calculate the actual values.
		 */
		t2t_vmax = 1000 / est_values->t2t_vmax;
		t2t_vmin = 1000 / est_values->t2t_vmin;
		vmin_slope = ((t2t_vmax - t2t_vmin) * 1000) /
			(boot_mv - t2t_coeffs->vmin);
		vmin_int = (t2t_vmax * 1000 - (vmin_slope * boot_mv)) / 1000;
		t2t_vmin = (vmin_slope * vmin) / 1000 + vmin_int;
		t2t_vmin = (1000 / t2t_vmin);
	}

	calc_values->t2t_vmin = (t2t_vmin * calc_values->t2t_vmax) /
		est_values->t2t_vmax;

	calc_values->ui_vmin = (1000000 / (thole_coeffs->freq_hz / 1000000)) /
		calc_values->t2t_vmin;

	/* Calculate the vmin tap hole at vmin of tuning data */
	temp_calc_vmin = (est_values->t2t_vmin * calc_values->t2t_vmax) /
		est_values->t2t_vmax;
	vmin_thole = (thole_coeffs->thole_vmin_int -
		(thole_coeffs->thole_vmin_slope * temp_calc_vmin)) /
		1000;
	vmax_thole = calc_values->vmax_thole;

	if (vmin == t2t_coeffs->vmin) {
		calc_values->vmin_thole = vmin_thole;
	} else if (vmin == boot_mv) {
		calc_values->vmin_thole = vmax_thole;
	} else {
		/*
		 * Interpolate the tap hole for any intermediate voltage.
		 * Calculate the slope and intercept from the available data
		 * and use them to calculate the actual values.
		 */
		vmin_slope = ((vmax_thole - vmin_thole) * 1000) /
			(boot_mv - t2t_coeffs->vmin);
		vmin_int = (vmax_thole * 1000 - (vmin_slope * boot_mv)) / 1000;
		calc_values->vmin_thole = (vmin_slope * vmin) / 1000 + vmin_int;
	}

	/* Adjust the partial win start for Vmin boundary */
	if (tuning_data->is_partial_win_valid)
		tuning_data->tap_data[0].win_start =
			(tuning_data->tap_data[0].win_start *
			tuning_data->calc_values.t2t_vmax) /
			tuning_data->calc_values.t2t_vmin;
}

static int slide_window_start(struct tegra_tuning_data *tuning_data,
	int tap_value, enum tap_win_edge_attr edge_attr, int tap_hole)
{
	if (edge_attr == WIN_EDGE_BOUND_START) {
		if (tap_value < 0)
			tap_value += (1000 / tuning_data->calc_values.t2t_vmin);
		else
			tap_value += (1000 / tuning_data->calc_values.t2t_vmax);
	} else if (edge_attr == WIN_EDGE_HOLE) {
		tap_value += ((7 * tap_hole) / 100) +
			((450 / tuning_data->calc_values.t2t_vmax) + (1 / 2));
	}

	if (tap_value > MAX_TAP_VALUES)
		tap_value = MAX_TAP_VALUES;

	return tap_value;
}

static int slide_window_end(struct tegra_tuning_data *tuning_data,
	int tap_value, enum tap_win_edge_attr edge_attr, int tap_hole)
{
	if (edge_attr == WIN_EDGE_BOUND_END) {
		tap_value = (tap_value * tuning_data->calc_values.t2t_vmax) /
			tuning_data->calc_values.t2t_vmin;
		tap_value -= (1000 / tuning_data->calc_values.t2t_vmin);
	} else if (edge_attr == WIN_EDGE_HOLE) {
		if (tap_hole > 0)
			tap_value = tap_hole;
		tap_value -= ((7 * tap_hole) / 100) +
			((450 / tuning_data->calc_values.t2t_vmin) + (1 / 2));
	}

	return tap_value;
}

static void tegra_sdhci_adjust_window_boundaries(struct sdhci_host *sdhci,
	struct tegra_tuning_data *tuning_data)
{
	struct tap_window_data *tap_data;
	int vmin_tap_hole;
	int vmax_tap_hole;
	u8 i = 0;

	vmax_tap_hole = tuning_data->calc_values.vmax_thole;
	vmin_tap_hole = tuning_data->calc_values.vmin_thole;
	for (i = 0; i < tuning_data->num_of_valid_tap_wins; i++) {
		tap_data = &tuning_data->tap_data[i];
		tap_data->win_start = slide_window_start(tuning_data,
			tap_data->win_start, tap_data->win_start_attr,
			vmax_tap_hole);
		/* Update with next hole if first hole is taken care of */
		if (tap_data->win_start_attr == WIN_EDGE_HOLE)
			vmax_tap_hole += tuning_data->calc_values.ui;

		tap_data->win_end = slide_window_end(tuning_data,
			tap_data->win_end, tap_data->win_end_attr,
			vmin_tap_hole);
		/* Update with next hole if first hole is taken care of */
		if (tap_data->win_end_attr == WIN_EDGE_HOLE)
			vmin_tap_hole += tuning_data->calc_values.ui_vmin;
	}
}

static int tegra_sdhci_find_best_tap_value(struct sdhci_host *sdhci,
	struct tegra_tuning_data *tuning_data)
{
	struct tap_window_data *tap_data;
	u8 i = 0, sel_win = 0;
	int pref_win = 0, curr_win_size = 0;
	int best_tap_value = 0;

	for (i = 0; i < tuning_data->num_of_valid_tap_wins; i++) {
		tap_data = &tuning_data->tap_data[i];
		if (!i && tuning_data->is_partial_win_valid) {
			pref_win = tap_data->win_end - tap_data->win_start;
			if ((tap_data->win_end * 2) < pref_win)
				pref_win = tap_data->win_end * 2;
			sel_win = 0;
		} else {
			curr_win_size = tap_data->win_end - tap_data->win_start;
			if ((curr_win_size > 0) && (curr_win_size > pref_win)) {
				pref_win = curr_win_size;
				sel_win = i;
			}
		}
	}

	if (pref_win <= 0)
		return -EINVAL;

	tap_data = &tuning_data->tap_data[sel_win];
	if (!sel_win && tuning_data->is_partial_win_valid) {
		best_tap_value = tap_data->win_end - (pref_win / 2);
		if (best_tap_value < 0)
			best_tap_value = 0;
	} else {
		best_tap_value = tap_data->win_start +
			((tap_data->win_end - tap_data->win_start) *
			tuning_data->calc_values.t2t_vmin) /
			(tuning_data->calc_values.t2t_vmin +
			tuning_data->calc_values.t2t_vmax);
	}

	return best_tap_value;
}

static int tegra_sdhci_calculate_best_tap(struct sdhci_host *sdhci,
	struct tegra_tuning_data *tuning_data)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct tap_window_data old_tap_data[MAX_TAP_WINDOWS];
	int vmin, curr_vmin, best_tap_value;
	int err = 0;

	curr_vmin = tegra_dvfs_predict_millivolts(pltfm_host->clk,
						tegra_host->current_clk);
	memcpy(old_tap_data, tuning_data->tap_data, sizeof(old_tap_data));
	for (vmin = curr_vmin; vmin <= tegra_host->boot_vcore_mv; vmin += 50) {
		/* Calculate tap hole locations at current Vmin */
		tegra_sdhci_calculate_vmin_values(sdhci, tuning_data, vmin);

		/* Adjust boundaries/holes for Vmin */
		tegra_sdhci_adjust_window_boundaries(sdhci, tuning_data);

		/* Select the tap value from the adjusted windows */
		best_tap_value = tegra_sdhci_find_best_tap_value(sdhci,
								tuning_data);
		if (best_tap_value >= 0)
			break;

		/* No tap value found? Increase Vmin and try again */
		memcpy(tuning_data->tap_data, old_tap_data,
			sizeof(old_tap_data));
	}

	if (best_tap_value < 0) {
		dev_err(mmc_dev(sdhci->mmc), "No tap values found for %luHz\n",
			tegra_host->current_clk);
		return -EINVAL;
	}
	tuning_data->best_tap_value = best_tap_value;

	/* Set the new vmin if there is any change. */
	if ((tuning_data->best_tap_value >= 0) && (curr_vmin != vmin)) {
		dev_info(mmc_dev(sdhci->mmc), "min %dmV at %luHz\n", vmin,
			tegra_host->current_clk);
		err = tegra_dvfs_set_fmax_at_vmin(pltfm_host->clk,
			tegra_host->current_clk, vmin);
	}

	return err;
}

static int tegra_sdhci_verify_best_tap(struct sdhci_host *sdhci)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct tegra_tuning_data *tuning_data;
	int err = 0;

	tuning_data = &tegra_host->tuning_data;

	if ((tuning_data->best_tap_value < 0) ||
		(tuning_data->best_tap_value > MAX_TAP_VALUES)) {
		dev_err(mmc_dev(sdhci->mmc), "Invalid best tap value\n");
		return -EINVAL;
	}

	/* Set the best tap value */
	tegra_sdhci_set_tap_delay(sdhci, tuning_data->best_tap_value);

	/* Run tuning after setting the best tap value */
	err = tegra_sdhci_issue_tuning_cmd(sdhci);
	if (err)
		dev_err(mmc_dev(sdhci->mmc),
			"Tap value verification failed at %luHz: %d\n",
			tegra_host->current_clk, err);

	dev_info(mmc_dev(sdhci->mmc), "Tuning success at %luHz with tap %d\n",
		tegra_host->current_clk, tuning_data->best_tap_value);

	return err;
}

static int tegra_sdhci_execute_tuning(struct sdhci_host *sdhci, u32 opcode)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhci);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	struct tegra_tuning_data *tuning_data;
	int err;
	u32 ier, misc_ctrl;

	tuning_data = &tegra_host->tuning_data;
	if (tegra_host->tuning_done && tegra_host->non_removable) {
		tegra_sdhci_set_tap_delay(sdhci, tuning_data->best_tap_value);
		return 0;
	} else
		tegra_host->tuning_done = false;

	/* Tuning should be done only for MMC_BUS_WIDTH_8 and MMC_BUS_WIDTH_4 */
	if (sdhci->mmc->ios.bus_width == MMC_BUS_WIDTH_8)
		tegra_host->tuning_bsize = MMC_TUNING_BLOCK_SIZE_BUS_WIDTH_8;
	else if (sdhci->mmc->ios.bus_width == MMC_BUS_WIDTH_4)
		tegra_host->tuning_bsize = MMC_TUNING_BLOCK_SIZE_BUS_WIDTH_4;
	else
		return -EINVAL;

	if (tegra_host->disable_ext_loopback) {
		misc_ctrl = sdhci_readl(sdhci, SDHCI_TEGRA_VENDOR_MISC_CTRL);
		misc_ctrl &= ~SDHCI_VNDR_MISC_CTRL_EN_EXT_LOOPBACK;
		sdhci_writel(sdhci, misc_ctrl, SDHCI_TEGRA_VENDOR_MISC_CTRL);
	}

	/* Set the tuning command to be used */
	tegra_host->tuning_opcode = opcode;

	/*
	 * Disable all interrupts signalling. Enable interrupt status
	 * detection for buffer read ready and data crc. We use
	 * polling for tuning as it involves less overhead.
	 */
	ier = sdhci_readl(sdhci, SDHCI_INT_ENABLE);
	sdhci_writel(sdhci, 0, SDHCI_SIGNAL_ENABLE);
	sdhci_writel(sdhci, SDHCI_INT_DATA_AVAIL |
		SDHCI_INT_DATA_CRC, SDHCI_INT_ENABLE);

	tegra_sdhci_estimate_tuning_values(sdhci, tuning_data);

	/*
	 * Core voltage needs to remain the same throughout the tuning
	 * process.
	 */
	tegra_dvfs_core_lock();

	err = tegra_sdhci_get_tap_window_data(sdhci, tuning_data);
	if (err)
		goto out;

	err = tegra_sdhci_calculate_best_tap(sdhci, tuning_data);
	if (err)
		goto out;

	err = tegra_sdhci_verify_best_tap(sdhci);
	if (!err)
		tegra_host->tuning_done = true;

out:
	tegra_dvfs_core_unlock();

	if (tegra_host->disable_ext_loopback && err) {
			/*
			 * Tuning failed and card will try to enumerate in
			 * Legacy High Speed mode.  So we reenable the loopback
			 * if we had disabled it
			 */
			misc_ctrl = sdhci_readl(sdhci,
						SDHCI_TEGRA_VENDOR_MISC_CTRL);
			misc_ctrl |= SDHCI_VNDR_MISC_CTRL_EN_EXT_LOOPBACK;
			sdhci_writel(sdhci, misc_ctrl,
				     SDHCI_TEGRA_VENDOR_MISC_CTRL);
	}

	/* Enable interrupts */
	sdhci_writel(sdhci, ier, SDHCI_INT_ENABLE);
	sdhci_writel(sdhci, ier, SDHCI_SIGNAL_ENABLE);

	return err;
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

		mdelay(1);
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
	struct tegra_tuning_data *tuning_data;
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
			tuning_data = &tegra_host->tuning_data;
			if (tegra_host->tuning_done)
				best_tap_value = tuning_data->best_tap_value;
			else
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

		if (tegra_host->disable_ext_loopback) {
			if (tegra_host->tuning_done) {
				misc_ctrl &=
					~SDHCI_VNDR_MISC_CTRL_EN_EXT_LOOPBACK;
			} else {
				misc_ctrl |=
					SDHCI_VNDR_MISC_CTRL_EN_EXT_LOOPBACK;
			}
		}
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

static void tegra_sdhci_reset(struct sdhci_host *sdhci, u8 mask)
{
	unsigned long timeout;

	sdhci_writeb(sdhci, mask, SDHCI_SOFTWARE_RESET);

	/* Wait max 100 ms */
	timeout = RESET_TIMEOUT;

	/* hw clears the bit when it's done */
	while (sdhci_readb(sdhci, SDHCI_SOFTWARE_RESET) & mask) {
		if (timeout == 0) {
			dev_err(mmc_dev(sdhci->mmc),
				"Reset 0x%x never completed.\n", (int)mask);
			return;
		}
		timeout--;
		mdelay(1);
	}

	tegra_sdhci_reset_exit(sdhci, mask);
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

static unsigned int tegra_sdhci_get_timeout_clock(struct sdhci_host *sdhci)
{
	return 0;
}

static void tegra_sdhci_card_event(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;

	tegra_host->tuning_done = false;
}

static const struct sdhci_ops tegra_sdhci_ops = {
	.get_ro     = tegra_sdhci_get_ro,
	.read_l     = tegra_sdhci_readl,
	.read_w     = tegra_sdhci_readw,
	.write_l    = tegra_sdhci_writel,
	.platform_bus_width = tegra_sdhci_buswidth,
	.platform_execute_tuning = tegra_sdhci_execute_tuning,
	.platform_reset_exit = tegra_sdhci_reset_exit,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.set_uhs_signaling	= tegra_sdhci_set_uhs_signaling,
	.switch_signal_voltage_exit = tegra_sdhci_do_calibration,
	.get_timeout_clock	= tegra_sdhci_get_timeout_clock,
	.card_event		= tegra_sdhci_card_event,
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
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.ops  = &tegra_sdhci_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra132 = {
	.pdata = &sdhci_tegra132_pdata,
	.nvquirks = TEGRA30_SDHCI_NVQUIRKS |
		    NVQUIRK_SET_TRIM_DELAY |
		    NVQUIRK_ENABLE_DDR50 |
		    NVQUIRK_ENABLE_HS200 |
		    NVQUIRK_INFINITE_ERASE_TIMEOUT |
		    NVQUIRK_SET_PAD_E_INPUT_OR_E_PWRD |
		    NVQUIRK_SET_CALIBRATION_OFFSETS,
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
	struct device_node *iter;
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	int ret;
	int i;

	tegra_host->power_gpio = of_get_named_gpio(np, "power-gpios", 0);
	ret = mmc_of_parse(host->mmc);

	if (of_get_property(np, "nvidia,disable-ext-loopback", NULL))
		tegra_host->disable_ext_loopback = true;

	if (of_get_property(np, "non-removable", NULL))
		tegra_host->non_removable = true;

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

	/* Required HS/UHS tuning parameters */
	if (of_property_read_u32(np, "nvidia,vmax",
				 &tegra_host->t2t_coeffs.vmax))
		return -EINVAL;
	if (of_property_read_u32(np, "nvidia,vmin",
				 &tegra_host->t2t_coeffs.vmin))
		return -EINVAL;
	if (of_property_read_u32(np, "nvidia,t2t-vnom-slope",
				 &tegra_host->t2t_coeffs.t2t_vnom_slope))
		return -EINVAL;
	if (of_property_read_u32(np, "nvidia,t2t-vnom-int",
				 &tegra_host->t2t_coeffs.t2t_vnom_int))
		return -EINVAL;
	if (of_property_read_u32(np, "nvidia,t2t-vmax-slope",
				 &tegra_host->t2t_coeffs.t2t_vmax_slope))
		return -EINVAL;
	if (of_property_read_u32(np, "nvidia,t2t-vmax-int",
				 &tegra_host->t2t_coeffs.t2t_vmax_int))
		return -EINVAL;
	if (of_property_read_u32(np, "nvidia,t2t-vmin-slope",
				 &tegra_host->t2t_coeffs.t2t_vmin_slope))
		return -EINVAL;
	if (of_property_read_u32(np, "nvidia,t2t-vmin-int",
				 &tegra_host->t2t_coeffs.t2t_vmin_int))
		return -EINVAL;

	i = 0;
	for_each_child_of_node(np, iter)
		if (of_device_is_compatible(iter, "nvidia,sdhci-tuning-data"))
			i++;
	tegra_host->thole_coeffs = devm_kzalloc(dev,
					i * sizeof(*tegra_host->thole_coeffs),
					GFP_KERNEL);
	if (!tegra_host->thole_coeffs)
		return -ENOMEM;
	i = 0;
	for_each_child_of_node(np, iter) {
		struct tap_hole_coeffs *coeffs;

		coeffs = &tegra_host->thole_coeffs[i];
		if (!of_device_is_compatible(iter, "nvidia,sdhci-tuning-data"))
			continue;
		if (of_property_read_u32(iter, "clock-frequency",
					 &coeffs->freq_hz))
			continue;
		if (i > 0 &&
		    coeffs->freq_hz <= tegra_host->thole_coeffs[i - 1].freq_hz)
			continue;
		if (of_property_read_u32(iter, "nvidia,thole-vnom-slope",
					 &coeffs->thole_vnom_slope))
			continue;
		if (of_property_read_u32(iter, "nvidia,thole-vnom-int",
					 &coeffs->thole_vnom_int))
			continue;
		if (of_property_read_u32(iter, "nvidia,thole-vmax-slope",
					 &coeffs->thole_vmax_slope))
			continue;
		if (of_property_read_u32(iter, "nvidia,thole-vmax-int",
					 &coeffs->thole_vmax_int))
			continue;
		if (of_property_read_u32(iter, "nvidia,thole-vmin-slope",
					 &coeffs->thole_vmin_slope))
			continue;
		if (of_property_read_u32(iter, "nvidia,thole-vmin-int",
					 &coeffs->thole_vmin_int))
			continue;
		i++;
	}
	if (!i) {
		dev_err(mmc_dev(host->mmc), "no valid tuning frequencies\n");
		return -EINVAL;
	}
	tegra_host->nr_tuning_freqs = i;

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
	if (rc) {
		dev_err(mmc_dev(host->mmc), "failed to parse DT data\n");
		goto err_alloc_tegra_host;
	}

	if (gpio_is_valid(tegra_host->power_gpio)) {
		rc = gpio_request(tegra_host->power_gpio, "sdhci_power");
		if (rc) {
			dev_err(mmc_dev(host->mmc),
				"failed to allocate power gpio\n");
			goto err_power_req;
		}
		gpio_direction_output(tegra_host->power_gpio, 0);
		usleep_range(10000, 11000);
		gpio_direction_output(tegra_host->power_gpio, 1);
	}

	clk = clk_get(mmc_dev(host->mmc), NULL);
	if (IS_ERR(clk)) {
		dev_err(mmc_dev(host->mmc), "clk err\n");
		rc = PTR_ERR(clk);
		goto err_clk_get;
	}
	pltfm_host->clk = clk;
	tegra_sdhci_clock_enable(host);

	tegra_sdhci_init_clock(host);

	/*
	 * Temporary disable runtime PM when soc support high speed bus.
	 * It seems like mmc autocal will fail if we enable runtime PM
	 * presumably because the clocks are off while we are trying to autocal.
	 */
	if (!((soc_data->nvquirks & NVQUIRK_ENABLE_HS200) ||
	    (soc_data->nvquirks & NVQUIRK_ENABLE_SDR104))) {
		pm_runtime_set_active(&pdev->dev);
		if (!tegra_host->no_runtime_pm)
			pm_runtime_enable(&pdev->dev);
		pm_runtime_get_sync(&pdev->dev);
		pm_runtime_set_autosuspend_delay(&pdev->dev,
						 TEGRA_SDHCI_AUTOSUSPEND_DELAY);
		pm_runtime_use_autosuspend(&pdev->dev);
		pm_suspend_ignore_children(&pdev->dev, true);
	}

	if (soc_data->nvquirks & NVQUIRK_ENABLE_HS200)
		host->mmc->caps2 |= MMC_CAP2_HS200;

	if (soc_data->nvquirks & NVQUIRK_ENABLE_SDR50)
		host->mmc->caps |= MMC_CAP_UHS_SDR50;

	if (soc_data->nvquirks & NVQUIRK_ENABLE_SDR104)
		host->mmc->caps |= MMC_CAP_UHS_SDR104;

	tegra_host->speedo = tegra_get_cpu_speedo_value();
	dev_info(mmc_dev(host->mmc), "Speedo value %d\n", tegra_host->speedo);

	tegra_host->boot_vcore_mv = tegra_dvfs_get_core_nominal_millivolts();
	dev_info(mmc_dev(host->mmc),
		"Tegra Vcore boot mV value %d\n", tegra_host->boot_vcore_mv);

	rc = sdhci_add_host(host);
	if (rc)
		goto err_add_host;

	pm_runtime_put_autosuspend(&pdev->dev);
	device_enable_async_suspend(&pdev->dev);

	return 0;

err_add_host:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	tegra_sdhci_clock_disable(host);
	clk_put(pltfm_host->clk);
err_clk_get:
	if (gpio_is_valid(tegra_host->power_gpio))
		gpio_free(tegra_host->power_gpio);
err_power_req:
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

	tegra_sdhci_clock_disable(host);
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
	if (!mmc_card_keep_power(host->mmc))
		if (gpio_is_valid(tegra_host->power_gpio))
			gpio_direction_output(tegra_host->power_gpio, 0);

	/*
	 * We should be able to gate the controller clock here, but the
	 * Marvell SD8897 is generating SDIO interrupts after it ought to
	 * have been suspended, causing a hard hang if the SDIO IRQ thread
	 * runs after the clock is gated.
	 */

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

	/* If the device may have changed in sleep, we need to retune */
	if (!tegra_host->non_removable)
		tegra_host->tuning_done = false;

	pm_runtime_get_sync(dev);

	if (!mmc_card_keep_power(host->mmc))
		if (gpio_is_valid(tegra_host->power_gpio))
			gpio_direction_output(tegra_host->power_gpio, 1);
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
	int ret;

	ret = sdhci_runtime_suspend_host(host);
	tegra_sdhci_clock_disable(host);

	return ret;
}

static int tegra_sdhci_runtime_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	int ret;

	tegra_sdhci_clock_enable(host);
	ret = sdhci_runtime_resume_host(host);

	return ret;
}
#endif

static const struct dev_pm_ops sdhci_tegra_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tegra_sdhci_suspend, tegra_sdhci_resume)
	SET_RUNTIME_PM_OPS(tegra_sdhci_runtime_suspend,
			   tegra_sdhci_runtime_resume, NULL)
};

static void sdhci_tegra_shutdown(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;

	if (gpio_is_valid(tegra_host->power_gpio))
		gpio_direction_output(tegra_host->power_gpio, 0);
}

static struct platform_driver sdhci_tegra_driver = {
	.driver		= {
		.name	= "sdhci-tegra",
		.owner	= THIS_MODULE,
		.of_match_table = sdhci_tegra_dt_match,
		.pm	= &sdhci_tegra_pm_ops,
	},
	.probe		= sdhci_tegra_probe,
	.remove		= sdhci_tegra_remove,
	.shutdown	= sdhci_tegra_shutdown,
};

module_platform_driver(sdhci_tegra_driver);

MODULE_DESCRIPTION("SDHCI driver for Tegra");
MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL v2");
