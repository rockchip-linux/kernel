// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 Rockchip Electronics Co. Ltd.
 *
 * Author: Wyon Bi <bivvy.bi@rock-chips.com>
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/pm_runtime.h>
#include <linux/mfd/syscon.h>
#include <linux/rockchip/cpu.h>

#define PSEC_PER_SEC	1000000000000LL

#define UPDATE(x, h, l)	(((x) << (l)) & GENMASK((h), (l)))

/*
 * The offset address[7:0] is distributed two parts, one from the bit7 to bit5
 * is the first address, the other from the bit4 to bit0 is the second address.
 * when you configure the registers, you must set both of them. The Clock Lane
 * and Data Lane use the same registers with the same second address, but the
 * first address is different.
 */
#define FIRST_ADDRESS(x)		(((x) & 0x7) << 5)
#define SECOND_ADDRESS(x)		(((x) & 0x1f) << 0)
#define PHY_REG(first, second)		(FIRST_ADDRESS(first) | \
					 SECOND_ADDRESS(second))

/* Analog Register Part: reg00 */
#define BANDGAP_POWER_MASK			BIT(7)
#define BANDGAP_POWER_DOWN			BIT(7)
#define BANDGAP_POWER_ON			0
#define LANE_EN_MASK				GENMASK(6, 2)
#define LANE_EN_CK				BIT(6)
#define LANE_EN_3				BIT(5)
#define LANE_EN_2				BIT(4)
#define LANE_EN_1				BIT(3)
#define LANE_EN_0				BIT(2)
#define POWER_WORK_MASK				GENMASK(1, 0)
#define POWER_WORK_ENABLE			UPDATE(1, 1, 0)
#define POWER_WORK_DISABLE			UPDATE(2, 1, 0)
/* Analog Register Part: reg01 */
#define REG_SYNCRST_MASK			BIT(2)
#define REG_SYNCRST_RESET			BIT(2)
#define REG_SYNCRST_NORMAL			0
#define REG_LDOPD_MASK				BIT(1)
#define REG_LDOPD_POWER_DOWN			BIT(1)
#define REG_LDOPD_POWER_ON			0
#define REG_PLLPD_MASK				BIT(0)
#define REG_PLLPD_POWER_DOWN			BIT(0)
#define REG_PLLPD_POWER_ON			0
/* Analog Register Part: reg03 */
#define REG_FBDIV_HI_MASK			BIT(5)
#define REG_FBDIV_HI(x)				UPDATE((x >> 8), 5, 5)
#define REG_PREDIV_MASK				GENMASK(4, 0)
#define REG_PREDIV(x)				UPDATE(x, 4, 0)
/* Analog Register Part: reg04 */
#define REG_FBDIV_LO_MASK			GENMASK(7, 0)
#define REG_FBDIV_LO(x)				UPDATE(x, 7, 0)
/* Analog Register Part: reg05 */
#define SAMPLE_CLOCK_PHASE_MASK			GENMASK(6, 4)
#define SAMPLE_CLOCK_PHASE(x)			UPDATE(x, 6, 4)
#define CLOCK_LANE_SKEW_PHASE_MASK		GENMASK(2, 0)
#define CLOCK_LANE_SKEW_PHASE(x)		UPDATE(x, 2, 0)
/* Analog Register Part: reg06 */
#define DATA_LANE_3_SKEW_PHASE_MASK		GENMASK(6, 4)
#define DATA_LANE_3_SKEW_PHASE(x)		UPDATE(x, 6, 4)
#define DATA_LANE_2_SKEW_PHASE_MASK		GENMASK(2, 0)
#define DATA_LANE_2_SKEW_PHASE(x)		UPDATE(x, 2, 0)
/* Analog Register Part: reg07 */
#define DATA_LANE_1_SKEW_PHASE_MASK		GENMASK(6, 4)
#define DATA_LANE_1_SKEW_PHASE(x)		UPDATE(x, 6, 4)
#define DATA_LANE_0_SKEW_PHASE_MASK		GENMASK(2, 0)
#define DATA_LANE_0_SKEW_PHASE(x)		UPDATE(x, 2, 0)
/* Analog Register Part: reg08 */
#define PRE_EMPHASIS_ENABLE_MASK		BIT(7)
#define PRE_EMPHASIS_ENABLE			BIT(7)
#define PRE_EMPHASIS_DISABLE			0
#define PLL_POST_DIV_ENABLE_MASK		BIT(5)
#define PLL_POST_DIV_ENABLE			BIT(5)
#define PLL_POST_DIV_DISABLE			0
#define DATA_LANE_VOD_RANGE_SET_MASK		GENMASK(3, 0)
#define DATA_LANE_VOD_RANGE_SET(x)		UPDATE(x, 3, 0)
#define SAMPLE_CLOCK_DIRECTION_MASK		BIT(4)
#define SAMPLE_CLOCK_DIRECTION_REVERSE		BIT(4)
#define SAMPLE_CLOCK_DIRECTION_FORWARD		0
#define LOWFRE_EN_MASK                          BIT(5)
#define PLL_OUTPUT_FREQUENCY_DIV_BY_1           0
#define PLL_OUTPUT_FREQUENCY_DIV_BY_2           1
/* Analog Register Part: reg1e */
#define PLL_MODE_SEL_MASK			GENMASK(6, 5)
#define PLL_MODE_SEL_LVDS_MODE			0
#define PLL_MODE_SEL_MIPI_MODE			BIT(5)
/* Analog Register Part: reg0b */
#define CLOCK_LANE_VOD_RANGE_SET_MASK	GENMASK(3, 0)
#define CLOCK_LANE_VOD_RANGE_SET(x)	UPDATE(x, 3, 0)
#define VOD_MIN_RANGE			0x1
#define VOD_MID_RANGE			0x3
#define VOD_BIG_RANGE			0x7
#define VOD_MAX_RANGE			0xf
/* Digital Register Part: reg00 */
#define REG_DIG_RSTN_MASK			BIT(0)
#define REG_DIG_RSTN_NORMAL			BIT(0)
#define REG_DIG_RSTN_RESET			0
/* Digital Register Part: reg01 */
#define INVERT_TXCLKESC_MASK			BIT(1)
#define INVERT_TXCLKESC_ENABLE			BIT(1)
#define INVERT_TXCLKESC_DISABLE			0
#define INVERT_TXBYTECLKHS_MASK			BIT(0)
#define INVERT_TXBYTECLKHS_ENABLE		BIT(0)
#define INVERT_TXBYTECLKHS_DISABLE		0
/* Clock/Data0/Data1/Data2/Data3 Lane Register Part: reg05 */
#define T_LPX_CNT_MASK				GENMASK(5, 0)
#define T_LPX_CNT(x)				UPDATE(x, 5, 0)
/* Clock/Data0/Data1/Data2/Data3 Lane Register Part: reg06 */
#define T_HS_ZERO_CNT_HI_MASK			BIT(7)
#define T_HS_ZERO_CNT_HI(x)			UPDATE(x, 7, 7)
#define T_HS_PREPARE_CNT_MASK			GENMASK(6, 0)
#define T_HS_PREPARE_CNT(x)			UPDATE(x, 6, 0)
/* Clock/Data0/Data1/Data2/Data3 Lane Register Part: reg07 */
#define T_HS_ZERO_CNT_LO_MASK			GENMASK(5, 0)
#define T_HS_ZERO_CNT_LO(x)			UPDATE(x, 5, 0)
/* Clock/Data0/Data1/Data2/Data3 Lane Register Part: reg08 */
#define T_HS_TRAIL_CNT_MASK			GENMASK(6, 0)
#define T_HS_TRAIL_CNT(x)			UPDATE(x, 6, 0)
/* Clock/Data0/Data1/Data2/Data3 Lane Register Part: reg09 */
#define T_HS_EXIT_CNT_LO_MASK			GENMASK(4, 0)
#define T_HS_EXIT_CNT_LO(x)			UPDATE(x, 4, 0)
/* Clock/Data0/Data1/Data2/Data3 Lane Register Part: reg0a */
#define T_CLK_POST_CNT_LO_MASK			GENMASK(3, 0)
#define T_CLK_POST_CNT_LO(x)			UPDATE(x, 3, 0)
/* Clock/Data0/Data1/Data2/Data3 Lane Register Part: reg0c */
#define LPDT_TX_PPI_SYNC_MASK			BIT(2)
#define LPDT_TX_PPI_SYNC_ENABLE			BIT(2)
#define LPDT_TX_PPI_SYNC_DISABLE		0
#define T_WAKEUP_CNT_HI_MASK			GENMASK(1, 0)
#define T_WAKEUP_CNT_HI(x)			UPDATE(x, 1, 0)
/* Clock/Data0/Data1/Data2/Data3 Lane Register Part: reg0d */
#define T_WAKEUP_CNT_LO_MASK			GENMASK(7, 0)
#define T_WAKEUP_CNT_LO(x)			UPDATE(x, 7, 0)
/* Clock/Data0/Data1/Data2/Data3 Lane Register Part: reg0e */
#define T_CLK_PRE_CNT_MASK			GENMASK(3, 0)
#define T_CLK_PRE_CNT(x)			UPDATE(x, 3, 0)
/* Clock/Data0/Data1/Data2/Data3 Lane Register Part: reg10 */
#define T_CLK_POST_HI_MASK			GENMASK(7, 6)
#define T_CLK_POST_HI(x)			UPDATE(x, 7, 6)
#define T_TA_GO_CNT_MASK			GENMASK(5, 0)
#define T_TA_GO_CNT(x)				UPDATE(x, 5, 0)
/* Clock/Data0/Data1/Data2/Data3 Lane Register Part: reg11 */
#define T_HS_EXIT_CNT_HI_MASK			BIT(6)
#define T_HS_EXIT_CNT_HI(x)			UPDATE(x, 6, 6)
#define T_TA_SURE_CNT_MASK			GENMASK(5, 0)
#define T_TA_SURE_CNT(x)			UPDATE(x, 5, 0)
/* Clock/Data0/Data1/Data2/Data3 Lane Register Part: reg12 */
#define T_TA_WAIT_CNT_MASK			GENMASK(5, 0)
#define T_TA_WAIT_CNT(x)			UPDATE(x, 5, 0)
/* LVDS Register Part: reg00 */
#define LVDS_DIGITAL_INTERNAL_RESET_MASK	BIT(2)
#define LVDS_DIGITAL_INTERNAL_RESET_DISABLE	BIT(2)
#define LVDS_DIGITAL_INTERNAL_RESET_ENABLE	0
/* LVDS Register Part: reg01 */
#define LVDS_DIGITAL_INTERNAL_ENABLE_MASK	BIT(7)
#define LVDS_DIGITAL_INTERNAL_ENABLE		BIT(7)
#define LVDS_DIGITAL_INTERNAL_DISABLE		0
/* LVDS Register Part: reg03 */
#define MODE_ENABLE_MASK			GENMASK(2, 0)
#define TTL_MODE_ENABLE				BIT(2)
#define LVDS_MODE_ENABLE			BIT(1)
#define MIPI_MODE_ENABLE			BIT(0)
/* LVDS Register Part: reg0b */
#define LVDS_LANE_EN_MASK			GENMASK(7, 3)
#define LVDS_DATA_LANE0_EN			BIT(7)
#define LVDS_DATA_LANE1_EN			BIT(6)
#define LVDS_DATA_LANE2_EN			BIT(5)
#define LVDS_DATA_LANE3_EN			BIT(4)
#define LVDS_CLK_LANE_EN			BIT(3)
#define LVDS_PLL_POWER_MASK			BIT(2)
#define LVDS_PLL_POWER_OFF			BIT(2)
#define LVDS_PLL_POWER_ON			0
#define LVDS_BANDGAP_POWER_MASK			BIT(0)
#define LVDS_BANDGAP_POWER_DOWN			BIT(0)
#define LVDS_BANDGAP_POWER_ON			0

#define DSI_PHY_RSTZ		0xa0
#define PHY_ENABLECLK		BIT(2)
#define DSI_PHY_STATUS		0xb0
#define PHY_LOCK		BIT(0)

enum soc_type {
	PX30,
	PX30S,
	RK3128,
	RK3368,
	RK3562,
	RK3568,
	RV1126,
};

enum phy_max_rate {
	MAX_1GHZ,
	MAX_2_5GHZ,
};

struct inno_mipi_dphy_timing {
	unsigned int max_lane_mbps;
	u8 lpx;
	u8 hs_prepare;
	u8 clk_lane_hs_zero;
	u8 data_lane_hs_zero;
	u8 hs_trail;
};

struct inno_dsidphy {
	struct device *dev;
	struct clk *ref_clk;
	struct clk *pclk_phy;
	struct clk *pclk_host;
	void __iomem *phy_base;
	void __iomem *host_base;
	struct reset_control *rst;
	struct phy_configure_opts_mipi_dphy dphy_cfg;
	unsigned int lanes;
	const struct inno_dsidphy_plat_data *pdata;

	struct clk *pll_clk;
	struct {
		struct clk_hw hw;
		u8 prediv;
		u16 fbdiv;
		unsigned long rate;
	} pll;
};

struct inno_dsidphy_plat_data {
	enum soc_type soc_type;
	const struct inno_mipi_dphy_timing *inno_mipi_dphy_timing_table;
	const unsigned int num_timings;
	enum phy_max_rate max_rate;
};

enum {
	REGISTER_PART_ANALOG,
	REGISTER_PART_DIGITAL,
	REGISTER_PART_CLOCK_LANE,
	REGISTER_PART_DATA0_LANE,
	REGISTER_PART_DATA1_LANE,
	REGISTER_PART_DATA2_LANE,
	REGISTER_PART_DATA3_LANE,
	REGISTER_PART_LVDS,
};

static const
struct inno_mipi_dphy_timing inno_mipi_dphy_timing_table_max_1GHz[] = {
	{ 110, 0x0, 0x20, 0x16, 0x02, 0x22},
	{ 150, 0x0, 0x06, 0x16, 0x03, 0x45},
	{ 200, 0x0, 0x18, 0x17, 0x04, 0x0b},
	{ 250, 0x0, 0x05, 0x17, 0x05, 0x16},
	{ 300, 0x0, 0x51, 0x18, 0x06, 0x2c},
	{ 400, 0x0, 0x64, 0x19, 0x07, 0x33},
	{ 500, 0x0, 0x20, 0x1b, 0x07, 0x4e},
	{ 600, 0x0, 0x6a, 0x1d, 0x08, 0x3a},
	{ 700, 0x0, 0x3e, 0x1e, 0x08, 0x6a},
	{ 800, 0x0, 0x21, 0x1f, 0x09, 0x29},
	{1000, 0x0, 0x09, 0x20, 0x09, 0x27},
};

static const
struct inno_mipi_dphy_timing inno_mipi_dphy_timing_table_max_2_5GHz[] = {
	{ 110, 0x02, 0x7f, 0x16, 0x02, 0x02},
	{ 150, 0x02, 0x7f, 0x16, 0x03, 0x02},
	{ 200, 0x02, 0x7f, 0x17, 0x04, 0x02},
	{ 250, 0x02, 0x7f, 0x17, 0x05, 0x04},
	{ 300, 0x02, 0x7f, 0x18, 0x06, 0x04},
	{ 400, 0x03, 0x7e, 0x19, 0x07, 0x04},
	{ 500, 0x03, 0x7c, 0x1b, 0x07, 0x08},
	{ 600, 0x03, 0x70, 0x1d, 0x08, 0x10},
	{ 700, 0x05, 0x40, 0x1e, 0x08, 0x30},
	{ 800, 0x05, 0x02, 0x1f, 0x09, 0x30},
	{1000, 0x05, 0x08, 0x20, 0x09, 0x30},
	{1200, 0x06, 0x03, 0x32, 0x14, 0x0f},
	{1400, 0x09, 0x03, 0x32, 0x14, 0x0f},
	{1600, 0x0d, 0x42, 0x36, 0x0e, 0x0f},
	{1800, 0x0e, 0x47, 0x7a, 0x0e, 0x0f},
	{2000, 0x11, 0x64, 0x7a, 0x0e, 0x0b},
	{2200, 0x13, 0x64, 0x7e, 0x15, 0x0b},
	{2400, 0x13, 0x33, 0x7f, 0x15, 0x6a},
	{2500, 0x15, 0x54, 0x7f, 0x15, 0x6a},
};

static inline struct inno_dsidphy *hw_to_inno(struct clk_hw *hw)
{
	return container_of(hw, struct inno_dsidphy, pll.hw);
}

static void phy_update_bits(struct inno_dsidphy *inno,
			    u8 first, u8 second, u8 mask, u8 val)
{
	u32 reg = PHY_REG(first, second) << 2;
	unsigned int tmp, orig;

	orig = readl(inno->phy_base + reg);
	tmp = orig & ~mask;
	tmp |= val & mask;
	writel(tmp, inno->phy_base + reg);
}

static void host_update_bits(struct inno_dsidphy *inno,
			     u32 reg, u32 mask, u32 val)
{
	unsigned int tmp, orig;

	orig = readl(inno->host_base + reg);
	tmp = orig & ~mask;
	tmp |= val & mask;
	writel(tmp, inno->host_base + reg);
}

static unsigned long inno_dsidphy_pll_calc_rate(struct inno_dsidphy *inno,
						unsigned long rate)
{
	unsigned long prate = clk_get_rate(inno->ref_clk);
	unsigned long best_freq = 0;
	unsigned long fref, fout;
	u8 min_prediv, max_prediv;
	u8 _prediv, best_prediv = 1;
	u16 _fbdiv, best_fbdiv = 1;
	u32 min_delta = UINT_MAX;

	/*
	 * The PLL output frequency can be calculated using a simple formula:
	 * PLL_Output_Frequency = (FREF / PREDIV * FBDIV) / 2
	 * PLL_Output_Frequency: it is equal to DDR-Clock-Frequency * 2
	 */
	fref = prate / 2;
	if (!fref)
		return 0;

	if (rate > 1000000000UL)
		fout = 1000000000UL;
	else
		fout = rate;

	/* 5Mhz < Fref / prediv < 40MHz */
	min_prediv = DIV_ROUND_UP(fref, 40000000);
	max_prediv = fref / 5000000;

	for (_prediv = min_prediv; _prediv <= max_prediv; _prediv++) {
		u64 tmp;
		u32 delta;

		if (!_prediv)
			continue;

		tmp = (u64)fout * _prediv;
		do_div(tmp, fref);
		_fbdiv = tmp;

		/*
		 * The possible settings of feedback divider are
		 * 12, 13, 14, 16, ~ 511
		 */
		if (_fbdiv == 15)
			continue;

		if (_fbdiv < 12 || _fbdiv > 511)
			continue;

		tmp = (u64)_fbdiv * fref;
		do_div(tmp, _prediv);

		delta = abs(fout - tmp);
		if (!delta) {
			best_prediv = _prediv;
			best_fbdiv = _fbdiv;
			best_freq = tmp;
			break;
		} else if (delta < min_delta) {
			best_prediv = _prediv;
			best_fbdiv = _fbdiv;
			best_freq = tmp;
			min_delta = delta;
		}
	}

	if (best_freq) {
		inno->pll.prediv = best_prediv;
		inno->pll.fbdiv = best_fbdiv;
		inno->pll.rate = best_freq;
	}

	return best_freq;
}

static const struct inno_mipi_dphy_timing *
inno_mipi_dphy_get_timing(struct inno_dsidphy *inno)
{
	const struct inno_mipi_dphy_timing *timings;
	unsigned int num_timings;
	unsigned int lane_mbps = inno->pll.rate / USEC_PER_SEC;
	unsigned int i;

	timings = inno->pdata->inno_mipi_dphy_timing_table;
	num_timings = inno->pdata->num_timings;

	for (i = 0; i < num_timings; i++)
		if (lane_mbps <= timings[i].max_lane_mbps)
			break;

	if (i == num_timings)
		--i;

	return &timings[i];
}

static void inno_mipi_dphy_max_2_5GHz_pll_enable(struct inno_dsidphy *inno)
{

	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x03,
			REG_PREDIV_MASK, REG_PREDIV(inno->pll.prediv));
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x03,
			REG_FBDIV_HI_MASK, REG_FBDIV_HI(inno->pll.fbdiv));
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x04,
			REG_FBDIV_LO_MASK, REG_FBDIV_LO(inno->pll.fbdiv));
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x08,
			PLL_POST_DIV_ENABLE_MASK, PLL_POST_DIV_ENABLE);
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x0b,
			CLOCK_LANE_VOD_RANGE_SET_MASK,
			CLOCK_LANE_VOD_RANGE_SET(VOD_MAX_RANGE));
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x01,
			 REG_LDOPD_MASK | REG_PLLPD_MASK,
			 REG_LDOPD_POWER_ON | REG_PLLPD_POWER_ON);
}

static void inno_mipi_dphy_max_1GHz_pll_enable(struct inno_dsidphy *inno)
{
	/* Configure PLL */
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x03,
			REG_PREDIV_MASK, REG_PREDIV(inno->pll.prediv));
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x03,
			REG_FBDIV_HI_MASK, REG_FBDIV_HI(inno->pll.fbdiv));
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x04,
			REG_FBDIV_LO_MASK, REG_FBDIV_LO(inno->pll.fbdiv));
	/* Enable PLL and LDO */
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x01,
			REG_LDOPD_MASK | REG_PLLPD_MASK,
			REG_LDOPD_POWER_ON | REG_PLLPD_POWER_ON);
}

static void inno_mipi_dphy_reset(struct inno_dsidphy *inno)
{
	/* Reset analog */
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x01,
			REG_SYNCRST_MASK, REG_SYNCRST_RESET);
	udelay(1);
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x01,
			REG_SYNCRST_MASK, REG_SYNCRST_NORMAL);
	/* Reset digital */
	phy_update_bits(inno, REGISTER_PART_DIGITAL, 0x00,
			REG_DIG_RSTN_MASK, REG_DIG_RSTN_RESET);
	udelay(1);
	phy_update_bits(inno, REGISTER_PART_DIGITAL, 0x00,
			REG_DIG_RSTN_MASK, REG_DIG_RSTN_NORMAL);
}

static void inno_mipi_dphy_timing_init(struct inno_dsidphy *inno)
{
	struct phy_configure_opts_mipi_dphy *cfg = &inno->dphy_cfg;
	u32 t_txbyteclkhs, t_txclkesc;
	u32 txbyteclkhs, txclkesc, esc_clk_div;
	u32 hs_exit, clk_post, clk_pre, wakeup, lpx, ta_go, ta_sure, ta_wait;
	u32 hs_prepare, hs_trail, hs_zero, clk_lane_hs_zero, data_lane_hs_zero;
	const struct inno_mipi_dphy_timing *timing;
	unsigned int i;

	txbyteclkhs = inno->pll.rate / 8;
	t_txbyteclkhs = div_u64(PSEC_PER_SEC, txbyteclkhs);

	esc_clk_div = DIV_ROUND_UP(txbyteclkhs, 20000000);
	txclkesc = txbyteclkhs / esc_clk_div;
	t_txclkesc = div_u64(PSEC_PER_SEC, txclkesc);

	/*
	 * The value of counter for HS Ths-exit
	 * Ths-exit = Tpin_txbyteclkhs * value
	 */
	hs_exit = DIV_ROUND_UP(cfg->hs_exit, t_txbyteclkhs);
	/*
	 * The value of counter for HS Tclk-post
	 * Tclk-post = Tpin_txbyteclkhs * value
	 */
	clk_post = DIV_ROUND_UP(cfg->clk_post, t_txbyteclkhs);
	/*
	 * The value of counter for HS Tclk-pre
	 * Tclk-pre = Tpin_txbyteclkhs * value
	 */
	clk_pre = DIV_ROUND_UP(cfg->clk_pre, t_txbyteclkhs);
	/*
	 * The value of counter for HS Tta-go
	 * Tta-go for turnaround
	 * Tta-go = Ttxclkesc * value
	 */
	ta_go = DIV_ROUND_UP(cfg->ta_go, t_txclkesc);
	/*
	 * The value of counter for HS Tta-sure
	 * Tta-sure for turnaround
	 * Tta-sure = Ttxclkesc * value
	 */
	ta_sure = DIV_ROUND_UP(cfg->ta_sure, t_txclkesc);
	/*
	 * The value of counter for HS Tta-wait
	 * Tta-wait for turnaround
	 * Tta-wait = Ttxclkesc * value
	 */
	ta_wait = DIV_ROUND_UP(cfg->ta_get, t_txclkesc);

	timing = inno_mipi_dphy_get_timing(inno);
	/*
	 * The value of counter for HS Tlpx Time
	 * Tlpx = Tpin_txbyteclkhs * (2 + value)
	 */
	if (inno->pdata->max_rate == MAX_1GHZ) {
		lpx = DIV_ROUND_UP(cfg->lpx, t_txbyteclkhs);
		if (lpx >= 2)
			lpx -= 2;
	} else
		lpx = timing->lpx;

	hs_prepare = timing->hs_prepare;
	hs_trail = timing->hs_trail;
	clk_lane_hs_zero = timing->clk_lane_hs_zero;
	data_lane_hs_zero = timing->data_lane_hs_zero;
	wakeup = 0x3ff;

	for (i = REGISTER_PART_CLOCK_LANE; i <= REGISTER_PART_DATA3_LANE; i++) {
		if (i == REGISTER_PART_CLOCK_LANE)
			hs_zero = clk_lane_hs_zero;
		else
			hs_zero = data_lane_hs_zero;

		phy_update_bits(inno, i, 0x05, T_LPX_CNT_MASK,
				T_LPX_CNT(lpx));
		phy_update_bits(inno, i, 0x06, T_HS_PREPARE_CNT_MASK,
				T_HS_PREPARE_CNT(hs_prepare));

		if (inno->pdata->max_rate == MAX_2_5GHZ)
			phy_update_bits(inno, i, 0x06, T_HS_ZERO_CNT_HI_MASK,
					T_HS_ZERO_CNT_HI(hs_zero >> 6));

		phy_update_bits(inno, i, 0x07, T_HS_ZERO_CNT_LO_MASK,
				T_HS_ZERO_CNT_LO(hs_zero));
		phy_update_bits(inno, i, 0x08, T_HS_TRAIL_CNT_MASK,
				T_HS_TRAIL_CNT(hs_trail));

		if (inno->pdata->max_rate == MAX_2_5GHZ)
			phy_update_bits(inno, i, 0x11, T_HS_EXIT_CNT_HI_MASK,
					T_HS_EXIT_CNT_HI(hs_exit >> 5));

		phy_update_bits(inno, i, 0x09, T_HS_EXIT_CNT_LO_MASK,
				T_HS_EXIT_CNT_LO(hs_exit));

		if (inno->pdata->max_rate == MAX_2_5GHZ)
			phy_update_bits(inno, i, 0x10, T_CLK_POST_HI_MASK,
					T_CLK_POST_HI(clk_post >> 4));

		phy_update_bits(inno, i, 0x0a, T_CLK_POST_CNT_LO_MASK,
				T_CLK_POST_CNT_LO(clk_post));
		phy_update_bits(inno, i, 0x0e, T_CLK_PRE_CNT_MASK,
				T_CLK_PRE_CNT(clk_pre));
		phy_update_bits(inno, i, 0x0c, T_WAKEUP_CNT_HI_MASK,
				T_WAKEUP_CNT_HI(wakeup >> 8));
		phy_update_bits(inno, i, 0x0d, T_WAKEUP_CNT_LO_MASK,
				T_WAKEUP_CNT_LO(wakeup));
		phy_update_bits(inno, i, 0x10, T_TA_GO_CNT_MASK,
				T_TA_GO_CNT(ta_go));
		phy_update_bits(inno, i, 0x11, T_TA_SURE_CNT_MASK,
				T_TA_SURE_CNT(ta_sure));
		phy_update_bits(inno, i, 0x12, T_TA_WAIT_CNT_MASK,
				T_TA_WAIT_CNT(ta_wait));
	}
}

static void inno_mipi_dphy_lane_enable(struct inno_dsidphy *inno)
{
	u8 val = LANE_EN_CK;

	switch (inno->lanes) {
	case 1:
		val |= LANE_EN_0;
		break;
	case 2:
		val |= LANE_EN_1 | LANE_EN_0;
		break;
	case 3:
		val |= LANE_EN_2 | LANE_EN_1 | LANE_EN_0;
		break;
	case 4:
	default:
		val |= LANE_EN_3 | LANE_EN_2 | LANE_EN_1 | LANE_EN_0;
		break;
	}

	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x00, LANE_EN_MASK, val);
}

static void inno_dsidphy_mipi_mode_enable(struct inno_dsidphy *inno)
{
	/* Select MIPI mode */
	phy_update_bits(inno, REGISTER_PART_LVDS, 0x03,
			MODE_ENABLE_MASK, MIPI_MODE_ENABLE);

	/* set pin_txclkesc_0 pin_txbyteclk invert disable */
	if (inno->pdata->soc_type == PX30S)
		phy_update_bits(inno, REGISTER_PART_DIGITAL, 0x01,
				INVERT_TXCLKESC_MASK, INVERT_TXCLKESC_DISABLE);

	if (inno->pdata->max_rate == MAX_2_5GHZ)
		inno_mipi_dphy_max_2_5GHz_pll_enable(inno);
	else
		inno_mipi_dphy_max_1GHz_pll_enable(inno);

	inno_mipi_dphy_reset(inno);
	inno_mipi_dphy_timing_init(inno);
	inno_mipi_dphy_lane_enable(inno);
}

static void inno_dsidphy_lvds_mode_enable(struct inno_dsidphy *inno)
{
	u8 prediv = 2;
	u16 fbdiv = 28;

	/* Sample clock reverse direction */
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x08,
			SAMPLE_CLOCK_DIRECTION_MASK | LOWFRE_EN_MASK,
			SAMPLE_CLOCK_DIRECTION_REVERSE |
			PLL_OUTPUT_FREQUENCY_DIV_BY_1);

	/* Reset LVDS digital logic */
	phy_update_bits(inno, REGISTER_PART_LVDS, 0x00,
			LVDS_DIGITAL_INTERNAL_RESET_MASK,
			LVDS_DIGITAL_INTERNAL_RESET_ENABLE);
	udelay(1);
	phy_update_bits(inno, REGISTER_PART_LVDS, 0x00,
			LVDS_DIGITAL_INTERNAL_RESET_MASK,
			LVDS_DIGITAL_INTERNAL_RESET_DISABLE);

	/* Select LVDS mode */
	phy_update_bits(inno, REGISTER_PART_LVDS, 0x03,
			MODE_ENABLE_MASK, LVDS_MODE_ENABLE);
	/* Configure PLL */
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x03,
			REG_PREDIV_MASK, REG_PREDIV(prediv));
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x03,
			REG_FBDIV_HI_MASK, REG_FBDIV_HI(fbdiv));
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x04,
			REG_FBDIV_LO_MASK, REG_FBDIV_LO(fbdiv));
	phy_update_bits(inno, REGISTER_PART_LVDS, 0x08, 0xff, 0xfc);
	/* Enable PLL and Bandgap */
	phy_update_bits(inno, REGISTER_PART_LVDS, 0x0b,
			LVDS_PLL_POWER_MASK | LVDS_BANDGAP_POWER_MASK,
			LVDS_PLL_POWER_ON | LVDS_BANDGAP_POWER_ON);

	msleep(20);

	/* Select PLL mode */
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x1e,
			PLL_MODE_SEL_MASK, PLL_MODE_SEL_LVDS_MODE);

	/* Enable LVDS digital logic */
	phy_update_bits(inno, REGISTER_PART_LVDS, 0x01,
			LVDS_DIGITAL_INTERNAL_ENABLE_MASK,
			LVDS_DIGITAL_INTERNAL_ENABLE);
	/* Enable LVDS analog driver */
	phy_update_bits(inno, REGISTER_PART_LVDS, 0x0b,
			LVDS_LANE_EN_MASK, LVDS_CLK_LANE_EN |
			LVDS_DATA_LANE0_EN | LVDS_DATA_LANE1_EN |
			LVDS_DATA_LANE2_EN | LVDS_DATA_LANE3_EN);
}

static void inno_dsidphy_phy_ttl_mode_enable(struct inno_dsidphy *inno)
{
	/* Reset digital logic */
	phy_update_bits(inno, REGISTER_PART_LVDS, 0x00,
			LVDS_DIGITAL_INTERNAL_RESET_MASK,
			LVDS_DIGITAL_INTERNAL_RESET_ENABLE);
	udelay(1);
	phy_update_bits(inno, REGISTER_PART_LVDS, 0x00,
			LVDS_DIGITAL_INTERNAL_RESET_MASK,
			LVDS_DIGITAL_INTERNAL_RESET_DISABLE);

	/* Select TTL mode */
	phy_update_bits(inno, REGISTER_PART_LVDS, 0x03,
			MODE_ENABLE_MASK, TTL_MODE_ENABLE);

	/* Enable digital logic */
	phy_update_bits(inno, REGISTER_PART_LVDS, 0x01,
			LVDS_DIGITAL_INTERNAL_ENABLE_MASK,
			LVDS_DIGITAL_INTERNAL_ENABLE);
	/* Enable analog driver */
	phy_update_bits(inno, REGISTER_PART_LVDS, 0x0b,
			LVDS_LANE_EN_MASK, LVDS_CLK_LANE_EN |
			LVDS_DATA_LANE0_EN | LVDS_DATA_LANE1_EN |
			LVDS_DATA_LANE2_EN | LVDS_DATA_LANE3_EN);
	/* Enable for clk lane in TTL mode */
	host_update_bits(inno, DSI_PHY_RSTZ, PHY_ENABLECLK, PHY_ENABLECLK);
}

static int inno_dsidphy_power_on(struct phy *phy)
{
	struct inno_dsidphy *inno = phy_get_drvdata(phy);
	enum phy_mode mode = phy_get_mode(phy);

	clk_prepare_enable(inno->pclk_phy);
	clk_prepare_enable(inno->ref_clk);
	pm_runtime_get_sync(inno->dev);

	/* Bandgap power on */
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x00,
			BANDGAP_POWER_MASK, BANDGAP_POWER_ON);
	/* Enable power work */
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x00,
			POWER_WORK_MASK, POWER_WORK_ENABLE);

	switch (mode) {
	case PHY_MODE_MIPI_DPHY:
		inno_dsidphy_mipi_mode_enable(inno);
		break;
	case PHY_MODE_LVDS:
		inno_dsidphy_lvds_mode_enable(inno);
		break;
	default:
		inno_dsidphy_phy_ttl_mode_enable(inno);
	}

	return 0;
}

static int inno_dsidphy_power_off(struct phy *phy)
{
	struct inno_dsidphy *inno = phy_get_drvdata(phy);

	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x00, LANE_EN_MASK, 0);
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x01,
			REG_LDOPD_MASK | REG_PLLPD_MASK,
			REG_LDOPD_POWER_DOWN | REG_PLLPD_POWER_DOWN);
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x00,
			POWER_WORK_MASK, POWER_WORK_DISABLE);
	phy_update_bits(inno, REGISTER_PART_ANALOG, 0x00,
			BANDGAP_POWER_MASK, BANDGAP_POWER_DOWN);

	phy_update_bits(inno, REGISTER_PART_LVDS, 0x0b, LVDS_LANE_EN_MASK, 0);
	phy_update_bits(inno, REGISTER_PART_LVDS, 0x01,
			LVDS_DIGITAL_INTERNAL_ENABLE_MASK,
			LVDS_DIGITAL_INTERNAL_DISABLE);
	phy_update_bits(inno, REGISTER_PART_LVDS, 0x0b,
			LVDS_PLL_POWER_MASK | LVDS_BANDGAP_POWER_MASK,
			LVDS_PLL_POWER_OFF | LVDS_BANDGAP_POWER_DOWN);

	pm_runtime_put(inno->dev);
	clk_disable_unprepare(inno->ref_clk);
	clk_disable_unprepare(inno->pclk_phy);

	return 0;
}

static int inno_dsidphy_set_mode(struct phy *phy, enum phy_mode mode,
				   int submode)
{
	return 0;
}

static int inno_dsidphy_configure(struct phy *phy,
				  union phy_configure_opts *opts)
{
	struct inno_dsidphy *inno = phy_get_drvdata(phy);
	struct phy_configure_opts_mipi_dphy *cfg = &inno->dphy_cfg;
	enum phy_mode mode = phy_get_mode(phy);
	int ret;

	if (mode != PHY_MODE_MIPI_DPHY)
		return -EINVAL;

	ret = phy_mipi_dphy_config_validate(&opts->mipi_dphy);
	if (ret)
		return ret;

	memcpy(&inno->dphy_cfg, &opts->mipi_dphy, sizeof(inno->dphy_cfg));

	inno_dsidphy_pll_calc_rate(inno, cfg->hs_clk_rate);
	cfg->hs_clk_rate = inno->pll.rate;
	opts->mipi_dphy.hs_clk_rate = inno->pll.rate;

	return 0;
}

static int inno_dsidphy_init(struct phy *phy)
{
	struct inno_dsidphy *inno = phy_get_drvdata(phy);

	clk_prepare_enable(inno->pclk_phy);
	clk_prepare_enable(inno->ref_clk);
	pm_runtime_get_sync(inno->dev);

	return 0;
}

static int inno_dsidphy_exit(struct phy *phy)
{
	struct inno_dsidphy *inno = phy_get_drvdata(phy);

	pm_runtime_put(inno->dev);
	clk_disable_unprepare(inno->ref_clk);
	clk_disable_unprepare(inno->pclk_phy);

	return 0;
}

static const struct phy_ops inno_dsidphy_ops = {
	.configure = inno_dsidphy_configure,
	.set_mode = inno_dsidphy_set_mode,
	.power_on = inno_dsidphy_power_on,
	.power_off = inno_dsidphy_power_off,
	.init = inno_dsidphy_init,
	.exit = inno_dsidphy_exit,
	.owner = THIS_MODULE,
};

static const struct inno_dsidphy_plat_data px30_video_phy_plat_data = {
	.soc_type = PX30,
	.inno_mipi_dphy_timing_table = inno_mipi_dphy_timing_table_max_1GHz,
	.num_timings = ARRAY_SIZE(inno_mipi_dphy_timing_table_max_1GHz),
	.max_rate = MAX_1GHZ,
};

static const struct inno_dsidphy_plat_data px30s_video_phy_plat_data = {
	.soc_type = PX30S,
	.inno_mipi_dphy_timing_table = inno_mipi_dphy_timing_table_max_2_5GHz,
	.num_timings = ARRAY_SIZE(inno_mipi_dphy_timing_table_max_2_5GHz),
	.max_rate = MAX_2_5GHZ,
};

static const struct inno_dsidphy_plat_data rk3128_video_phy_plat_data = {
	.soc_type = RK3128,
	.inno_mipi_dphy_timing_table = inno_mipi_dphy_timing_table_max_1GHz,
	.num_timings = ARRAY_SIZE(inno_mipi_dphy_timing_table_max_1GHz),
	.max_rate = MAX_1GHZ,
};

static const struct inno_dsidphy_plat_data rk3368_video_phy_plat_data = {
	.soc_type = RK3368,
	.inno_mipi_dphy_timing_table = inno_mipi_dphy_timing_table_max_1GHz,
	.num_timings = ARRAY_SIZE(inno_mipi_dphy_timing_table_max_1GHz),
	.max_rate = MAX_1GHZ,
};

static const struct inno_dsidphy_plat_data rk3562_video_phy_plat_data = {
	.soc_type = RK3562,
	.inno_mipi_dphy_timing_table = inno_mipi_dphy_timing_table_max_2_5GHz,
	.num_timings = ARRAY_SIZE(inno_mipi_dphy_timing_table_max_2_5GHz),
	.max_rate = MAX_2_5GHZ,
};

static const struct inno_dsidphy_plat_data rk3568_video_phy_plat_data = {
	.soc_type = RK3568,
	.inno_mipi_dphy_timing_table = inno_mipi_dphy_timing_table_max_2_5GHz,
	.num_timings = ARRAY_SIZE(inno_mipi_dphy_timing_table_max_2_5GHz),
	.max_rate = MAX_2_5GHZ,
};

static const struct inno_dsidphy_plat_data rv1126_video_phy_plat_data = {
	.soc_type = RV1126,
	.inno_mipi_dphy_timing_table = inno_mipi_dphy_timing_table_max_2_5GHz,
	.num_timings = ARRAY_SIZE(inno_mipi_dphy_timing_table_max_2_5GHz),
	.max_rate = MAX_2_5GHZ,
};

static int inno_dsidphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct inno_dsidphy *inno;
	struct phy_provider *phy_provider;
	struct phy *phy;
	struct resource *res;
	int ret;

	inno = devm_kzalloc(dev, sizeof(*inno), GFP_KERNEL);
	if (!inno)
		return -ENOMEM;

	inno->dev = dev;
	inno->pdata = of_device_get_match_data(inno->dev);
	if (soc_is_px30s())
		inno->pdata = &px30s_video_phy_plat_data;

	platform_set_drvdata(pdev, inno);

	inno->phy_base = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(inno->phy_base))
		return PTR_ERR(inno->phy_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "host");
	if (!res) {
		dev_err(dev, "invalid host resource\n");
		return -EINVAL;
	}

	inno->host_base = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(inno->host_base))
		return PTR_ERR(inno->host_base);

	inno->ref_clk = devm_clk_get(dev, "ref");
	if (IS_ERR(inno->ref_clk)) {
		ret = PTR_ERR(inno->ref_clk);
		dev_err(dev, "failed to get ref clock: %d\n", ret);
		return ret;
	}

	inno->pclk_phy = devm_clk_get(dev, "pclk");
	if (IS_ERR(inno->pclk_phy)) {
		ret = PTR_ERR(inno->pclk_phy);
		dev_err(dev, "failed to get phy pclk: %d\n", ret);
		return ret;
	}

	inno->pclk_host = devm_clk_get(dev, "pclk_host");
	if (IS_ERR(inno->pclk_host)) {
		ret = PTR_ERR(inno->pclk_host);
		dev_err(dev, "failed to get host pclk: %d\n", ret);
		return ret;
	}

	inno->rst = devm_reset_control_get(dev, "apb");
	if (IS_ERR(inno->rst)) {
		ret = PTR_ERR(inno->rst);
		dev_err(dev, "failed to get system reset control: %d\n", ret);
		return ret;
	}

	phy = devm_phy_create(dev, NULL, &inno_dsidphy_ops);
	if (IS_ERR(phy)) {
		ret = PTR_ERR(phy);
		dev_err(dev, "failed to create phy: %d\n", ret);
		return ret;
	}

	if (of_property_read_u32(dev->of_node, "inno,lanes", &inno->lanes))
		inno->lanes = 4;

	phy_set_drvdata(phy, inno);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		ret = PTR_ERR(phy_provider);
		dev_err(dev, "failed to register phy provider: %d\n", ret);
		return ret;
	}

	pm_runtime_enable(dev);

	return 0;
}

static int inno_dsidphy_remove(struct platform_device *pdev)
{
	struct inno_dsidphy *inno = platform_get_drvdata(pdev);

	pm_runtime_disable(inno->dev);

	return 0;
}

static const struct of_device_id inno_dsidphy_of_match[] = {
	{
		.compatible = "rockchip,px30-dsi-dphy",
		.data = &px30_video_phy_plat_data,
	}, {
		.compatible = "rockchip,px30s-dsi-dphy",
		.data = &px30s_video_phy_plat_data,
	}, {
		.compatible = "rockchip,rk3128-dsi-dphy",
		.data = &rk3128_video_phy_plat_data,
	}, {
		.compatible = "rockchip,rk3368-dsi-dphy",
		.data = &rk3368_video_phy_plat_data,
	}, {
		.compatible = "rockchip,rk3562-dsi-dphy",
		.data = &rk3562_video_phy_plat_data,
	}, {
		.compatible = "rockchip,rk3568-dsi-dphy",
		.data = &rk3568_video_phy_plat_data,
	}, {
		.compatible = "rockchip,rv1126-mipi-dphy",
		.data = &rv1126_video_phy_plat_data,
	},
	{}
};
MODULE_DEVICE_TABLE(of, inno_dsidphy_of_match);

static struct platform_driver inno_dsidphy_driver = {
	.driver = {
		.name = "inno-dsidphy",
		.of_match_table	= of_match_ptr(inno_dsidphy_of_match),
	},
	.probe = inno_dsidphy_probe,
	.remove = inno_dsidphy_remove,
};
module_platform_driver(inno_dsidphy_driver);

MODULE_AUTHOR("Wyon Bi <bivvy.bi@rock-chips.com>");
MODULE_DESCRIPTION("Innosilicon MIPI/LVDS/TTL Video Combo PHY driver");
MODULE_LICENSE("GPL v2");
