/*
 * Analogix DP (Display port) core register interface driver.
 *
 * Copyright (C) 2012 Samsung Electronics Co., Ltd.
 * Author: Jingoo Han <jg1.han@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <drm/bridge/analogix_dp.h>

#include "analogix_dp_core.h"
#include "analogix_dp_reg.h"

#define COMMON_INT_MASK_1	0
#define COMMON_INT_MASK_2	0
#define COMMON_INT_MASK_3	0
#define COMMON_INT_MASK_4	(HOTPLUG_CHG | HPD_LOST | PLUG)
#define INT_STA_MASK		INT_HPD

static void analogix_dp_write(struct analogix_dp_device *dp, u32 reg, u32 val)
{
	if (dp->plat_data && dp->plat_data->dev_type == ROCKCHIP_DP) {
		readl(dp->reg_base);
		writel(val, dp->reg_base + reg);
	}

	writel(val, dp->reg_base + reg);
}

static u32 analogix_dp_read(struct analogix_dp_device *dp, u32 reg)
{
	if (dp->plat_data && dp->plat_data->dev_type == ROCKCHIP_DP)
		readl(dp->reg_base + reg);

	return readl(dp->reg_base + reg);
}

void analogix_dp_enable_video_mute(struct analogix_dp_device *dp, bool enable)
{
	u32 reg;

	if (enable) {
		reg = analogix_dp_read(dp, ANALOGIX_DP_VIDEO_CTL_1);
		reg |= HDCP_VIDEO_MUTE;
		analogix_dp_write(dp, ANALOGIX_DP_VIDEO_CTL_1, reg);
	} else {
		reg = analogix_dp_read(dp, ANALOGIX_DP_VIDEO_CTL_1);
		reg &= ~HDCP_VIDEO_MUTE;
		analogix_dp_write(dp, ANALOGIX_DP_VIDEO_CTL_1, reg);
	}
}

void analogix_dp_stop_video(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_VIDEO_CTL_1);
	reg &= ~VIDEO_EN;
	analogix_dp_write(dp, ANALOGIX_DP_VIDEO_CTL_1, reg);
}

void analogix_dp_lane_swap(struct analogix_dp_device *dp, bool enable)
{
	u32 reg;

	if (enable)
		reg = LANE3_MAP_LOGIC_LANE_0 | LANE2_MAP_LOGIC_LANE_1 |
		      LANE1_MAP_LOGIC_LANE_2 | LANE0_MAP_LOGIC_LANE_3;
	else
		reg = LANE3_MAP_LOGIC_LANE_3 | LANE2_MAP_LOGIC_LANE_2 |
		      LANE1_MAP_LOGIC_LANE_1 | LANE0_MAP_LOGIC_LANE_0;

	analogix_dp_write(dp, ANALOGIX_DP_LANE_MAP, reg);
}

void analogix_dp_init_analog_param(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = TX_TERMINAL_CTRL_50_OHM;
	analogix_dp_write(dp, ANALOGIX_DP_ANALOG_CTL_1, reg);

	reg = SEL_24M | TX_DVDD_BIT_1_0625V;
	analogix_dp_write(dp, ANALOGIX_DP_ANALOG_CTL_2, reg);

	if (dp->plat_data && (dp->plat_data->dev_type == ROCKCHIP_DP)) {
		reg = REF_CLK_24M;
		if (dp->plat_data->subdev_type == RK3288_DP ||
		    dp->plat_data->subdev_type == RK3368_EDP)
			reg ^= REF_CLK_MASK;

		analogix_dp_write(dp, ANALOGIX_DP_PLL_REG_1, reg);
		analogix_dp_write(dp, ANALOGIX_DP_PLL_REG_2, 0x95);
		analogix_dp_write(dp, ANALOGIX_DP_PLL_REG_3, 0x40);
		analogix_dp_write(dp, ANALOGIX_DP_PLL_REG_4, 0x58);
		analogix_dp_write(dp, ANALOGIX_DP_PLL_REG_5, 0x22);
	}

	reg = DRIVE_DVDD_BIT_1_0625V | VCO_BIT_600_MICRO;
	analogix_dp_write(dp, ANALOGIX_DP_ANALOG_CTL_3, reg);

	reg = PD_RING_OSC | AUX_TERMINAL_CTRL_50_OHM |
		TX_CUR1_2X | TX_CUR_16_MA;
	analogix_dp_write(dp, ANALOGIX_DP_PLL_FILTER_CTL_1, reg);

	reg = CH3_AMP_400_MV | CH2_AMP_400_MV |
		CH1_AMP_400_MV | CH0_AMP_400_MV;
	analogix_dp_write(dp, ANALOGIX_DP_TX_AMP_TUNING_CTL, reg);
}

void analogix_dp_init_interrupt(struct analogix_dp_device *dp)
{
	/* Set interrupt pin assertion polarity as high */
	analogix_dp_write(dp, ANALOGIX_DP_INT_CTL, INT_POL1 | INT_POL0);

	/* Clear pending regisers */
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_STA_1, 0xff);
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_STA_2, 0x4f);
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_STA_3, 0xe0);
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_STA_4, 0xe7);
	analogix_dp_write(dp, ANALOGIX_DP_INT_STA, 0x63);

	/* 0:mask,1: unmask */
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_MASK_1, 0x00);
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_MASK_2, 0x00);
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_MASK_3, 0x00);
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_MASK_4, 0x00);
	analogix_dp_write(dp, ANALOGIX_DP_INT_STA_MASK, 0x00);
}

void analogix_dp_reset(struct analogix_dp_device *dp)
{
	u32 reg;

	analogix_dp_stop_video(dp);
	analogix_dp_enable_video_mute(dp, 0);

	reg = MASTER_VID_FUNC_EN_N | SLAVE_VID_FUNC_EN_N |
		AUD_FIFO_FUNC_EN_N | AUD_FUNC_EN_N |
		HDCP_FUNC_EN_N | SW_FUNC_EN_N;
	analogix_dp_write(dp, ANALOGIX_DP_FUNC_EN_1, reg);

	reg = SSC_FUNC_EN_N | AUX_FUNC_EN_N |
		SERDES_FIFO_FUNC_EN_N |
		LS_CLK_DOMAIN_FUNC_EN_N;
	analogix_dp_write(dp, ANALOGIX_DP_FUNC_EN_2, reg);

	usleep_range(20, 30);

	analogix_dp_lane_swap(dp, 0);

	analogix_dp_write(dp, ANALOGIX_DP_SYS_CTL_1, 0x0);
	analogix_dp_write(dp, ANALOGIX_DP_SYS_CTL_2, 0x40);
	analogix_dp_write(dp, ANALOGIX_DP_SYS_CTL_3, 0x0);
	analogix_dp_write(dp, ANALOGIX_DP_SYS_CTL_4, 0x0);

	analogix_dp_write(dp, ANALOGIX_DP_PKT_SEND_CTL, 0x0);
	analogix_dp_write(dp, ANALOGIX_DP_HDCP_CTL, 0x0);

	analogix_dp_write(dp, ANALOGIX_DP_HPD_DEGLITCH_L, 0x5e);
	analogix_dp_write(dp, ANALOGIX_DP_HPD_DEGLITCH_H, 0x1a);

	analogix_dp_write(dp, ANALOGIX_DP_LINK_DEBUG_CTL, 0x10);

	analogix_dp_write(dp, ANALOGIX_DP_PHY_TEST, 0x0);

	analogix_dp_write(dp, ANALOGIX_DP_VIDEO_FIFO_THRD, 0x0);
	analogix_dp_write(dp, ANALOGIX_DP_AUDIO_MARGIN, 0x20);

	analogix_dp_write(dp, ANALOGIX_DP_M_VID_GEN_FILTER_TH, 0x4);
	analogix_dp_write(dp, ANALOGIX_DP_M_AUD_GEN_FILTER_TH, 0x2);

	analogix_dp_write(dp, ANALOGIX_DP_SOC_GENERAL_CTL, 0x00000101);
}

void analogix_dp_swreset(struct analogix_dp_device *dp)
{
	analogix_dp_write(dp, ANALOGIX_DP_TX_SW_RESET, RESET_DP_TX);
}

void analogix_dp_config_interrupt(struct analogix_dp_device *dp)
{
	u32 reg;

	/* 0: mask, 1: unmask */
	reg = COMMON_INT_MASK_1;
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_MASK_1, reg);

	reg = COMMON_INT_MASK_2;
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_MASK_2, reg);

	reg = COMMON_INT_MASK_3;
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_MASK_3, reg);

	reg = COMMON_INT_MASK_4;
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_MASK_4, reg);

	reg = INT_STA_MASK;
	analogix_dp_write(dp, ANALOGIX_DP_INT_STA_MASK, reg);
}

void analogix_dp_mute_hpd_interrupt(struct analogix_dp_device *dp)
{
	u32 reg;

	/* 0: mask, 1: unmask */
	reg = analogix_dp_read(dp, ANALOGIX_DP_COMMON_INT_MASK_4);
	reg &= ~COMMON_INT_MASK_4;
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_MASK_4, reg);

	reg = analogix_dp_read(dp, ANALOGIX_DP_INT_STA_MASK);
	reg &= ~INT_STA_MASK;
	analogix_dp_write(dp, ANALOGIX_DP_INT_STA_MASK, reg);
}

void analogix_dp_unmute_hpd_interrupt(struct analogix_dp_device *dp)
{
	u32 reg;

	/* 0: mask, 1: unmask */
	reg = COMMON_INT_MASK_4;
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_MASK_4, reg);

	reg = INT_STA_MASK;
	analogix_dp_write(dp, ANALOGIX_DP_INT_STA_MASK, reg);
}

enum pll_status analogix_dp_get_pll_lock_status(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_DEBUG_CTL);
	if (reg & PLL_LOCK)
		return PLL_LOCKED;
	else
		return PLL_UNLOCKED;
}

void analogix_dp_set_pll_power_down(struct analogix_dp_device *dp, bool enable)
{
	u32 reg;

	if (enable) {
		reg = analogix_dp_read(dp, ANALOGIX_DP_PLL_CTL);
		reg |= DP_PLL_PD;
		analogix_dp_write(dp, ANALOGIX_DP_PLL_CTL, reg);
	} else {
		reg = analogix_dp_read(dp, ANALOGIX_DP_PLL_CTL);
		reg &= ~DP_PLL_PD;
		analogix_dp_write(dp, ANALOGIX_DP_PLL_CTL, reg);
	}
}

void analogix_dp_set_analog_power_down(struct analogix_dp_device *dp,
				       enum analog_power_block block,
				       bool enable)
{
	u32 reg;
	u32 phy_pd_addr = ANALOGIX_DP_PHY_PD;

	if (dp->plat_data && (dp->plat_data->dev_type == ROCKCHIP_DP))
		phy_pd_addr = ANALOGIX_DP_PD;

	switch (block) {
	case AUX_BLOCK:
		if (enable) {
			reg = analogix_dp_read(dp, phy_pd_addr);
			reg |= AUX_PD;
			analogix_dp_write(dp, phy_pd_addr, reg);
		} else {
			reg = analogix_dp_read(dp, phy_pd_addr);
			reg &= ~AUX_PD;
			analogix_dp_write(dp, phy_pd_addr, reg);
		}
		break;
	case CH0_BLOCK:
		if (enable) {
			reg = analogix_dp_read(dp, phy_pd_addr);
			reg |= CH0_PD;
			analogix_dp_write(dp, phy_pd_addr, reg);
		} else {
			reg = analogix_dp_read(dp, phy_pd_addr);
			reg &= ~CH0_PD;
			analogix_dp_write(dp, phy_pd_addr, reg);
		}
		break;
	case CH1_BLOCK:
		if (enable) {
			reg = analogix_dp_read(dp, phy_pd_addr);
			reg |= CH1_PD;
			analogix_dp_write(dp, phy_pd_addr, reg);
		} else {
			reg = analogix_dp_read(dp, phy_pd_addr);
			reg &= ~CH1_PD;
			analogix_dp_write(dp, phy_pd_addr, reg);
		}
		break;
	case CH2_BLOCK:
		if (enable) {
			reg = analogix_dp_read(dp, phy_pd_addr);
			reg |= CH2_PD;
			analogix_dp_write(dp, phy_pd_addr, reg);
		} else {
			reg = analogix_dp_read(dp, phy_pd_addr);
			reg &= ~CH2_PD;
			analogix_dp_write(dp, phy_pd_addr, reg);
		}
		break;
	case CH3_BLOCK:
		if (enable) {
			reg = analogix_dp_read(dp, phy_pd_addr);
			reg |= CH3_PD;
			analogix_dp_write(dp, phy_pd_addr, reg);
		} else {
			reg = analogix_dp_read(dp, phy_pd_addr);
			reg &= ~CH3_PD;
			analogix_dp_write(dp, phy_pd_addr, reg);
		}
		break;
	case ANALOG_TOTAL:
		if (enable) {
			reg = analogix_dp_read(dp, phy_pd_addr);
			reg |= DP_PHY_PD;
			analogix_dp_write(dp, phy_pd_addr, reg);
		} else {
			reg = analogix_dp_read(dp, phy_pd_addr);
			reg &= ~DP_PHY_PD;
			analogix_dp_write(dp, phy_pd_addr, reg);
		}
		break;
	case POWER_ALL:
		if (enable) {
			reg = DP_PHY_PD | AUX_PD | CH3_PD | CH2_PD |
				CH1_PD | CH0_PD;
			analogix_dp_write(dp, phy_pd_addr, reg);
		} else {
			analogix_dp_write(dp, phy_pd_addr, 0x00);
		}
		break;
	default:
		break;
	}
}

void analogix_dp_init_analog_func(struct analogix_dp_device *dp)
{
	u32 reg;
	int timeout_loop = 0;

	analogix_dp_set_analog_power_down(dp, POWER_ALL, 0);

	reg = PLL_LOCK_CHG;
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_STA_1, reg);

	reg = analogix_dp_read(dp, ANALOGIX_DP_DEBUG_CTL);
	reg &= ~(F_PLL_LOCK | PLL_LOCK_CTRL);
	analogix_dp_write(dp, ANALOGIX_DP_DEBUG_CTL, reg);

	/* Power up PLL */
	if (analogix_dp_get_pll_lock_status(dp) == PLL_UNLOCKED) {
		analogix_dp_set_pll_power_down(dp, 0);

		while (analogix_dp_get_pll_lock_status(dp) == PLL_UNLOCKED) {
			timeout_loop++;
			if (DP_TIMEOUT_LOOP_COUNT < timeout_loop) {
				dev_err(dp->dev, "failed to get pll lock status\n");
				return;
			}
			usleep_range(10, 20);
		}
	}

	/* Enable Serdes FIFO function and Link symbol clock domain module */
	reg = analogix_dp_read(dp, ANALOGIX_DP_FUNC_EN_2);
	reg &= ~(SERDES_FIFO_FUNC_EN_N | LS_CLK_DOMAIN_FUNC_EN_N
		| AUX_FUNC_EN_N);
	analogix_dp_write(dp, ANALOGIX_DP_FUNC_EN_2, reg);
}

void analogix_dp_clear_hotplug_interrupts(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = HOTPLUG_CHG | HPD_LOST | PLUG;
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_STA_4, reg);

	reg = INT_HPD;
	analogix_dp_write(dp, ANALOGIX_DP_INT_STA, reg);
}

void analogix_dp_init_hpd(struct analogix_dp_device *dp)
{
	u32 reg;

	analogix_dp_clear_hotplug_interrupts(dp);

	reg = analogix_dp_read(dp, ANALOGIX_DP_SYS_CTL_3);
	reg &= ~(F_HPD | HPD_CTRL);
	analogix_dp_write(dp, ANALOGIX_DP_SYS_CTL_3, reg);
}

void analogix_dp_force_hpd(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_SYS_CTL_3);
	reg |= (F_HPD | HPD_CTRL);
	analogix_dp_write(dp, ANALOGIX_DP_SYS_CTL_3, reg);
}

enum dp_irq_type analogix_dp_get_irq_type(struct analogix_dp_device *dp)
{
	u32 reg;

	/* Parse hotplug interrupt status register */
	reg = analogix_dp_read(dp, ANALOGIX_DP_COMMON_INT_STA_4);

	if (reg & PLUG)
		return DP_IRQ_TYPE_HP_CABLE_IN;

	if (reg & HPD_LOST)
		return DP_IRQ_TYPE_HP_CABLE_OUT;

	if (reg & HOTPLUG_CHG)
		return DP_IRQ_TYPE_HP_CHANGE;

	return DP_IRQ_TYPE_UNKNOWN;
}

void analogix_dp_reset_aux(struct analogix_dp_device *dp)
{
	u32 reg;

	/* Disable AUX channel module */
	reg = analogix_dp_read(dp, ANALOGIX_DP_FUNC_EN_2);
	reg |= AUX_FUNC_EN_N;
	analogix_dp_write(dp, ANALOGIX_DP_FUNC_EN_2, reg);
}

void analogix_dp_init_aux(struct analogix_dp_device *dp)
{
	u32 reg;

	/* Clear inerrupts related to AUX channel */
	reg = RPLY_RECEIV | AUX_ERR;
	analogix_dp_write(dp, ANALOGIX_DP_INT_STA, reg);

	analogix_dp_reset_aux(dp);

	/* Disable AUX transaction H/W retry */
	if (dp->plat_data && (dp->plat_data->dev_type == ROCKCHIP_DP))
		reg = AUX_BIT_PERIOD_EXPECTED_DELAY(0) |
		      AUX_HW_RETRY_COUNT_SEL(3) |
		      AUX_HW_RETRY_INTERVAL_600_MICROSECONDS;
	else
		reg = AUX_BIT_PERIOD_EXPECTED_DELAY(3) |
		      AUX_HW_RETRY_COUNT_SEL(0) |
		      AUX_HW_RETRY_INTERVAL_600_MICROSECONDS;
	analogix_dp_write(dp, ANALOGIX_DP_AUX_HW_RETRY_CTL, reg);

	/* Receive AUX Channel DEFER commands equal to DEFFER_COUNT*64 */
	reg = DEFER_CTRL_EN | DEFER_COUNT(1);
	analogix_dp_write(dp, ANALOGIX_DP_AUX_CH_DEFER_CTL, reg);

	/* Enable AUX channel module */
	reg = analogix_dp_read(dp, ANALOGIX_DP_FUNC_EN_2);
	reg &= ~AUX_FUNC_EN_N;
	analogix_dp_write(dp, ANALOGIX_DP_FUNC_EN_2, reg);
}

int analogix_dp_get_plug_in_status(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_SYS_CTL_3);
	if (reg & HPD_STATUS)
		return 0;

	return -EINVAL;
}

void analogix_dp_enable_sw_function(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_FUNC_EN_1);
	reg &= ~SW_FUNC_EN_N;
	analogix_dp_write(dp, ANALOGIX_DP_FUNC_EN_1, reg);
}

void analogix_dp_set_link_bandwidth(struct analogix_dp_device *dp, u32 bwtype)
{
	u32 reg;

	reg = bwtype;
	analogix_dp_write(dp, ANALOGIX_DP_LINK_BW_SET, reg);
}

void analogix_dp_get_link_bandwidth(struct analogix_dp_device *dp, u32 *bwtype)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_LINK_BW_SET);
	*bwtype = reg;
}

void analogix_dp_set_lane_count(struct analogix_dp_device *dp, u32 count)
{
	u32 reg;

	reg = count;
	analogix_dp_write(dp, ANALOGIX_DP_LANE_COUNT_SET, reg);
}

void analogix_dp_get_lane_count(struct analogix_dp_device *dp, u32 *count)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_LANE_COUNT_SET);
	*count = reg;
}

void analogix_dp_enable_enhanced_mode(struct analogix_dp_device *dp,
				      bool enable)
{
	u32 reg;

	if (enable) {
		reg = analogix_dp_read(dp, ANALOGIX_DP_SYS_CTL_4);
		reg |= ENHANCED;
		analogix_dp_write(dp, ANALOGIX_DP_SYS_CTL_4, reg);
	} else {
		reg = analogix_dp_read(dp, ANALOGIX_DP_SYS_CTL_4);
		reg &= ~ENHANCED;
		analogix_dp_write(dp, ANALOGIX_DP_SYS_CTL_4, reg);
	}
}

void analogix_dp_set_training_pattern(struct analogix_dp_device *dp,
				      enum pattern_set pattern)
{
	u32 reg;

	switch (pattern) {
	case PRBS7:
		reg = SCRAMBLING_ENABLE | LINK_QUAL_PATTERN_SET_PRBS7;
		analogix_dp_write(dp, ANALOGIX_DP_TRAINING_PTN_SET, reg);
		break;
	case D10_2:
		reg = SCRAMBLING_ENABLE | LINK_QUAL_PATTERN_SET_D10_2;
		analogix_dp_write(dp, ANALOGIX_DP_TRAINING_PTN_SET, reg);
		break;
	case TRAINING_PTN1:
		reg = SCRAMBLING_DISABLE | SW_TRAINING_PATTERN_SET_PTN1;
		analogix_dp_write(dp, ANALOGIX_DP_TRAINING_PTN_SET, reg);
		break;
	case TRAINING_PTN2:
		reg = SCRAMBLING_DISABLE | SW_TRAINING_PATTERN_SET_PTN2;
		analogix_dp_write(dp, ANALOGIX_DP_TRAINING_PTN_SET, reg);
		break;
	case TRAINING_PTN3:
		reg = SCRAMBLING_DISABLE | SW_TRAINING_PATTERN_SET_PTN3;
		analogix_dp_write(dp, ANALOGIX_DP_TRAINING_PTN_SET, reg);
		break;
	case DP_NONE:
		reg = SCRAMBLING_ENABLE |
			LINK_QUAL_PATTERN_SET_DISABLE |
			SW_TRAINING_PATTERN_SET_NORMAL;
		analogix_dp_write(dp, ANALOGIX_DP_TRAINING_PTN_SET, reg);
		break;
	default:
		break;
	}
}

void analogix_dp_set_lane0_pre_emphasis(struct analogix_dp_device *dp,
					u32 level)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_LN0_LINK_TRAINING_CTL);
	reg &= ~PRE_EMPHASIS_SET_MASK;
	reg |= level << PRE_EMPHASIS_SET_SHIFT;
	analogix_dp_write(dp, ANALOGIX_DP_LN0_LINK_TRAINING_CTL, reg);
}

void analogix_dp_set_lane1_pre_emphasis(struct analogix_dp_device *dp,
					u32 level)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_LN1_LINK_TRAINING_CTL);
	reg &= ~PRE_EMPHASIS_SET_MASK;
	reg |= level << PRE_EMPHASIS_SET_SHIFT;
	analogix_dp_write(dp, ANALOGIX_DP_LN1_LINK_TRAINING_CTL, reg);
}

void analogix_dp_set_lane2_pre_emphasis(struct analogix_dp_device *dp,
					u32 level)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_LN2_LINK_TRAINING_CTL);
	reg &= ~PRE_EMPHASIS_SET_MASK;
	reg |= level << PRE_EMPHASIS_SET_SHIFT;
	analogix_dp_write(dp, ANALOGIX_DP_LN2_LINK_TRAINING_CTL, reg);
}

void analogix_dp_set_lane3_pre_emphasis(struct analogix_dp_device *dp,
					u32 level)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_LN3_LINK_TRAINING_CTL);
	reg &= ~PRE_EMPHASIS_SET_MASK;
	reg |= level << PRE_EMPHASIS_SET_SHIFT;
	analogix_dp_write(dp, ANALOGIX_DP_LN3_LINK_TRAINING_CTL, reg);
}

void analogix_dp_set_lane0_link_training(struct analogix_dp_device *dp,
					 u32 training_lane)
{
	u32 reg;

	reg = training_lane;
	analogix_dp_write(dp, ANALOGIX_DP_LN0_LINK_TRAINING_CTL, reg);
}

void analogix_dp_set_lane1_link_training(struct analogix_dp_device *dp,
					 u32 training_lane)
{
	u32 reg;

	reg = training_lane;
	analogix_dp_write(dp, ANALOGIX_DP_LN1_LINK_TRAINING_CTL, reg);
}

void analogix_dp_set_lane2_link_training(struct analogix_dp_device *dp,
					 u32 training_lane)
{
	u32 reg;

	reg = training_lane;
	analogix_dp_write(dp, ANALOGIX_DP_LN2_LINK_TRAINING_CTL, reg);
}

void analogix_dp_set_lane3_link_training(struct analogix_dp_device *dp,
					 u32 training_lane)
{
	u32 reg;

	reg = training_lane;
	analogix_dp_write(dp, ANALOGIX_DP_LN3_LINK_TRAINING_CTL, reg);
}

u32 analogix_dp_get_lane0_link_training(struct analogix_dp_device *dp)
{
	return analogix_dp_read(dp, ANALOGIX_DP_LN0_LINK_TRAINING_CTL);
}

u32 analogix_dp_get_lane1_link_training(struct analogix_dp_device *dp)
{
	return analogix_dp_read(dp, ANALOGIX_DP_LN1_LINK_TRAINING_CTL);
}

u32 analogix_dp_get_lane2_link_training(struct analogix_dp_device *dp)
{
	return analogix_dp_read(dp, ANALOGIX_DP_LN2_LINK_TRAINING_CTL);
}

u32 analogix_dp_get_lane3_link_training(struct analogix_dp_device *dp)
{
	return analogix_dp_read(dp, ANALOGIX_DP_LN3_LINK_TRAINING_CTL);
}

void analogix_dp_reset_macro(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_PHY_TEST);
	reg |= MACRO_RST;
	analogix_dp_write(dp, ANALOGIX_DP_PHY_TEST, reg);

	/* 10 us is the minimum reset time. */
	usleep_range(10, 20);

	reg &= ~MACRO_RST;
	analogix_dp_write(dp, ANALOGIX_DP_PHY_TEST, reg);
}

void analogix_dp_init_video(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = VSYNC_DET | VID_FORMAT_CHG | VID_CLK_CHG;
	analogix_dp_write(dp, ANALOGIX_DP_COMMON_INT_STA_1, reg);

	reg = 0x0;
	analogix_dp_write(dp, ANALOGIX_DP_SYS_CTL_1, reg);

	reg = CHA_CRI(4) | CHA_CTRL;
	analogix_dp_write(dp, ANALOGIX_DP_SYS_CTL_2, reg);

	reg = 0x0;
	analogix_dp_write(dp, ANALOGIX_DP_SYS_CTL_3, reg);

	reg = VID_HRES_TH(2) | VID_VRES_TH(0);
	analogix_dp_write(dp, ANALOGIX_DP_VIDEO_CTL_8, reg);
}

void analogix_dp_set_video_color_format(struct analogix_dp_device *dp)
{
	u32 reg;

	/* Configure the input color depth, color space, dynamic range */
	reg = (dp->video_info.dynamic_range << IN_D_RANGE_SHIFT) |
		(dp->video_info.color_depth << IN_BPC_SHIFT) |
		(dp->video_info.color_space << IN_COLOR_F_SHIFT);
	analogix_dp_write(dp, ANALOGIX_DP_VIDEO_CTL_2, reg);

	/* Set Input Color YCbCr Coefficients to ITU601 or ITU709 */
	reg = analogix_dp_read(dp, ANALOGIX_DP_VIDEO_CTL_3);
	reg &= ~IN_YC_COEFFI_MASK;
	if (dp->video_info.ycbcr_coeff)
		reg |= IN_YC_COEFFI_ITU709;
	else
		reg |= IN_YC_COEFFI_ITU601;
	analogix_dp_write(dp, ANALOGIX_DP_VIDEO_CTL_3, reg);
}

int analogix_dp_is_slave_video_stream_clock_on(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_SYS_CTL_1);
	analogix_dp_write(dp, ANALOGIX_DP_SYS_CTL_1, reg);

	reg = analogix_dp_read(dp, ANALOGIX_DP_SYS_CTL_1);

	if (!(reg & DET_STA)) {
		dev_dbg(dp->dev, "Input stream clock not detected.\n");
		return -EINVAL;
	}

	reg = analogix_dp_read(dp, ANALOGIX_DP_SYS_CTL_2);
	analogix_dp_write(dp, ANALOGIX_DP_SYS_CTL_2, reg);

	reg = analogix_dp_read(dp, ANALOGIX_DP_SYS_CTL_2);
	dev_dbg(dp->dev, "wait SYS_CTL_2.\n");

	if (reg & CHA_STA) {
		dev_dbg(dp->dev, "Input stream clk is changing\n");
		return -EINVAL;
	}

	return 0;
}

void analogix_dp_set_video_cr_mn(struct analogix_dp_device *dp,
				 enum clock_recovery_m_value_type type,
				 u32 m_value, u32 n_value)
{
	u32 reg;

	if (type == REGISTER_M) {
		reg = analogix_dp_read(dp, ANALOGIX_DP_SYS_CTL_4);
		reg |= FIX_M_VID;
		analogix_dp_write(dp, ANALOGIX_DP_SYS_CTL_4, reg);
		reg = m_value & 0xff;
		analogix_dp_write(dp, ANALOGIX_DP_M_VID_0, reg);
		reg = (m_value >> 8) & 0xff;
		analogix_dp_write(dp, ANALOGIX_DP_M_VID_1, reg);
		reg = (m_value >> 16) & 0xff;
		analogix_dp_write(dp, ANALOGIX_DP_M_VID_2, reg);

		reg = n_value & 0xff;
		analogix_dp_write(dp, ANALOGIX_DP_N_VID_0, reg);
		reg = (n_value >> 8) & 0xff;
		analogix_dp_write(dp, ANALOGIX_DP_N_VID_1, reg);
		reg = (n_value >> 16) & 0xff;
		analogix_dp_write(dp, ANALOGIX_DP_N_VID_2, reg);
	} else  {
		reg = analogix_dp_read(dp, ANALOGIX_DP_SYS_CTL_4);
		reg &= ~FIX_M_VID;
		analogix_dp_write(dp, ANALOGIX_DP_SYS_CTL_4, reg);

		analogix_dp_write(dp, ANALOGIX_DP_N_VID_0, 0x00);
		analogix_dp_write(dp, ANALOGIX_DP_N_VID_1, 0x80);
		analogix_dp_write(dp, ANALOGIX_DP_N_VID_2, 0x00);
	}
}

void analogix_dp_set_video_timing_mode(struct analogix_dp_device *dp, u32 type)
{
	u32 reg;

	if (type == VIDEO_TIMING_FROM_CAPTURE) {
		reg = analogix_dp_read(dp, ANALOGIX_DP_VIDEO_CTL_10);
		reg &= ~FORMAT_SEL;
		analogix_dp_write(dp, ANALOGIX_DP_VIDEO_CTL_10, reg);
	} else {
		reg = analogix_dp_read(dp, ANALOGIX_DP_VIDEO_CTL_10);
		reg |= FORMAT_SEL;
		analogix_dp_write(dp, ANALOGIX_DP_VIDEO_CTL_10, reg);
	}
}

void analogix_dp_enable_video_master(struct analogix_dp_device *dp, bool enable)
{
	u32 reg;

	if (enable) {
		reg = analogix_dp_read(dp, ANALOGIX_DP_SOC_GENERAL_CTL);
		reg &= ~VIDEO_MODE_MASK;
		reg |= VIDEO_MASTER_MODE_EN | VIDEO_MODE_MASTER_MODE;
		analogix_dp_write(dp, ANALOGIX_DP_SOC_GENERAL_CTL, reg);
	} else {
		reg = analogix_dp_read(dp, ANALOGIX_DP_SOC_GENERAL_CTL);
		reg &= ~VIDEO_MODE_MASK;
		reg |= VIDEO_MODE_SLAVE_MODE;
		analogix_dp_write(dp, ANALOGIX_DP_SOC_GENERAL_CTL, reg);
	}
}

void analogix_dp_start_video(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_VIDEO_CTL_1);
	reg |= VIDEO_EN;
	analogix_dp_write(dp, ANALOGIX_DP_VIDEO_CTL_1, reg);
}

int analogix_dp_is_video_stream_on(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_SYS_CTL_3);
	analogix_dp_write(dp, ANALOGIX_DP_SYS_CTL_3, reg);

	reg = analogix_dp_read(dp, ANALOGIX_DP_SYS_CTL_3);
	if (!(reg & STRM_VALID)) {
		dev_dbg(dp->dev, "Input video stream is not detected.\n");
		return -EINVAL;
	}

	return 0;
}

void analogix_dp_config_video_slave_mode(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_FUNC_EN_1);
	reg &= ~(MASTER_VID_FUNC_EN_N | SLAVE_VID_FUNC_EN_N);
	reg |= MASTER_VID_FUNC_EN_N;
	analogix_dp_write(dp, ANALOGIX_DP_FUNC_EN_1, reg);

	reg = analogix_dp_read(dp, ANALOGIX_DP_VIDEO_CTL_10);
	reg &= ~INTERACE_SCAN_CFG;
	reg |= (dp->video_info.interlaced << 2);
	analogix_dp_write(dp, ANALOGIX_DP_VIDEO_CTL_10, reg);

	reg = analogix_dp_read(dp, ANALOGIX_DP_VIDEO_CTL_10);
	reg &= ~VSYNC_POLARITY_CFG;
	reg |= (dp->video_info.v_sync_polarity << 1);
	analogix_dp_write(dp, ANALOGIX_DP_VIDEO_CTL_10, reg);

	reg = analogix_dp_read(dp, ANALOGIX_DP_VIDEO_CTL_10);
	reg &= ~HSYNC_POLARITY_CFG;
	reg |= (dp->video_info.h_sync_polarity << 0);
	analogix_dp_write(dp, ANALOGIX_DP_VIDEO_CTL_10, reg);

	reg = AUDIO_MODE_SPDIF_MODE | VIDEO_MODE_SLAVE_MODE;
	analogix_dp_write(dp, ANALOGIX_DP_SOC_GENERAL_CTL, reg);
}

void analogix_dp_enable_scrambling(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_TRAINING_PTN_SET);
	reg &= ~SCRAMBLING_DISABLE;
	analogix_dp_write(dp, ANALOGIX_DP_TRAINING_PTN_SET, reg);
}

void analogix_dp_disable_scrambling(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_TRAINING_PTN_SET);
	reg |= SCRAMBLING_DISABLE;
	analogix_dp_write(dp, ANALOGIX_DP_TRAINING_PTN_SET, reg);
}

void analogix_dp_set_video_format(struct analogix_dp_device *dp)
{
	struct video_info *video = &dp->video_info;
	const struct drm_display_mode *mode = &video->mode;
	unsigned int hsw, hfp, hbp, vsw, vfp, vbp;

	hsw = mode->hsync_end - mode->hsync_start;
	hfp = mode->hsync_start - mode->hdisplay;
	hbp = mode->htotal - mode->hsync_end;
	vsw = mode->vsync_end - mode->vsync_start;
	vfp = mode->vsync_start - mode->vdisplay;
	vbp = mode->vtotal - mode->vsync_end;

	/* Set Video Format Parameters */
	analogix_dp_write(dp, ANALOGIX_DP_TOTAL_LINE_CFG_L,
			  TOTAL_LINE_CFG_L(mode->vtotal));
	analogix_dp_write(dp, ANALOGIX_DP_TOTAL_LINE_CFG_H,
			  TOTAL_LINE_CFG_H(mode->vtotal >> 8));
	analogix_dp_write(dp, ANALOGIX_DP_ACTIVE_LINE_CFG_L,
			  ACTIVE_LINE_CFG_L(mode->vdisplay));
	analogix_dp_write(dp, ANALOGIX_DP_ACTIVE_LINE_CFG_H,
			  ACTIVE_LINE_CFG_H(mode->vdisplay >> 8));
	analogix_dp_write(dp, ANALOGIX_DP_V_F_PORCH_CFG,
			  V_F_PORCH_CFG(vfp));
	analogix_dp_write(dp, ANALOGIX_DP_V_SYNC_WIDTH_CFG,
			  V_SYNC_WIDTH_CFG(vsw));
	analogix_dp_write(dp, ANALOGIX_DP_V_B_PORCH_CFG,
			  V_B_PORCH_CFG(vbp));
	analogix_dp_write(dp, ANALOGIX_DP_TOTAL_PIXEL_CFG_L,
			  TOTAL_PIXEL_CFG_L(mode->htotal));
	analogix_dp_write(dp, ANALOGIX_DP_TOTAL_PIXEL_CFG_H,
			  TOTAL_PIXEL_CFG_H(mode->htotal >> 8));
	analogix_dp_write(dp, ANALOGIX_DP_ACTIVE_PIXEL_CFG_L,
			  ACTIVE_PIXEL_CFG_L(mode->hdisplay));
	analogix_dp_write(dp, ANALOGIX_DP_ACTIVE_PIXEL_CFG_H,
			  ACTIVE_PIXEL_CFG_H(mode->hdisplay >> 8));
	analogix_dp_write(dp, ANALOGIX_DP_H_F_PORCH_CFG_L,
			  H_F_PORCH_CFG_L(hfp));
	analogix_dp_write(dp, ANALOGIX_DP_H_F_PORCH_CFG_H,
			  H_F_PORCH_CFG_H(hfp >> 8));
	analogix_dp_write(dp, ANALOGIX_DP_H_SYNC_CFG_L,
			  H_SYNC_CFG_L(hsw));
	analogix_dp_write(dp, ANALOGIX_DP_H_SYNC_CFG_H,
			  H_SYNC_CFG_H(hsw >> 8));
	analogix_dp_write(dp, ANALOGIX_DP_H_B_PORCH_CFG_L,
			  H_B_PORCH_CFG_L(hbp));
	analogix_dp_write(dp, ANALOGIX_DP_H_B_PORCH_CFG_H,
			  H_B_PORCH_CFG_H(hbp >> 8));
}

void analogix_dp_video_bist_enable(struct analogix_dp_device *dp)
{
	u32 reg;

	/* Enable Video BIST */
	analogix_dp_write(dp, ANALOGIX_DP_VIDEO_CTL_4, BIST_EN);

	/*
	 * Note that if BIST_EN is set to 1, F_SEL must be cleared to 0
	 * although video format information comes from registers set by user.
	 */
	reg = analogix_dp_read(dp, ANALOGIX_DP_VIDEO_CTL_10);
	reg &= ~FORMAT_SEL;
	analogix_dp_write(dp, ANALOGIX_DP_VIDEO_CTL_10, reg);
}

ssize_t analogix_dp_transfer(struct analogix_dp_device *dp,
			     struct drm_dp_aux_msg *msg)
{
	u32 reg;
	u8 *buffer = msg->buffer;
	int timeout_loop = 0;
	unsigned int i;
	int num_transferred = 0;

	/* Buffer size of AUX CH is 16 bytes */
	if (WARN_ON(msg->size > 16))
		return -E2BIG;

	/* Clear AUX CH data buffer */
	reg = BUF_CLR;
	analogix_dp_write(dp, ANALOGIX_DP_BUFFER_DATA_CTL, reg);

	switch (msg->request & ~DP_AUX_I2C_MOT) {
	case DP_AUX_I2C_WRITE:
		reg = AUX_TX_COMM_WRITE | AUX_TX_COMM_I2C_TRANSACTION;
		if (msg->request & DP_AUX_I2C_MOT)
			reg |= AUX_TX_COMM_MOT;
		break;

	case DP_AUX_I2C_READ:
		reg = AUX_TX_COMM_READ | AUX_TX_COMM_I2C_TRANSACTION;
		if (msg->request & DP_AUX_I2C_MOT)
			reg |= AUX_TX_COMM_MOT;
		break;

	case DP_AUX_NATIVE_WRITE:
		reg = AUX_TX_COMM_WRITE | AUX_TX_COMM_DP_TRANSACTION;
		break;

	case DP_AUX_NATIVE_READ:
		reg = AUX_TX_COMM_READ | AUX_TX_COMM_DP_TRANSACTION;
		break;

	default:
		return -EINVAL;
	}

	reg |= AUX_LENGTH(msg->size);
	analogix_dp_write(dp, ANALOGIX_DP_AUX_CH_CTL_1, reg);

	/* Select DPCD device address */
	reg = AUX_ADDR_7_0(msg->address);
	analogix_dp_write(dp, ANALOGIX_DP_AUX_ADDR_7_0, reg);
	reg = AUX_ADDR_15_8(msg->address);
	analogix_dp_write(dp, ANALOGIX_DP_AUX_ADDR_15_8, reg);
	reg = AUX_ADDR_19_16(msg->address);
	analogix_dp_write(dp, ANALOGIX_DP_AUX_ADDR_19_16, reg);

	if (!(msg->request & DP_AUX_I2C_READ)) {
		for (i = 0; i < msg->size; i++) {
			reg = buffer[i];
			analogix_dp_write(dp, ANALOGIX_DP_BUF_DATA_0 + 4 * i,
					  reg);
			num_transferred++;
		}
	}

	/* Enable AUX CH operation */
	reg = AUX_EN;

	/* Zero-sized messages specify address-only transactions. */
	if (msg->size < 1)
		reg |= ADDR_ONLY;

	analogix_dp_write(dp, ANALOGIX_DP_AUX_CH_CTL_2, reg);

	/* Is AUX CH command reply received? */
	/* TODO: Wait for an interrupt instead of looping? */
	reg = analogix_dp_read(dp, ANALOGIX_DP_INT_STA);
	while (!(reg & RPLY_RECEIV)) {
		timeout_loop++;
		if (timeout_loop > DP_TIMEOUT_LOOP_COUNT) {
			dev_err(dp->dev, "AUX CH command reply failed!\n");
			return -ETIMEDOUT;
		}
		reg = analogix_dp_read(dp, ANALOGIX_DP_INT_STA);
		usleep_range(10, 11);
	}

	/* Clear interrupt source for AUX CH command reply */
	analogix_dp_write(dp, ANALOGIX_DP_INT_STA, RPLY_RECEIV);

	/* Clear interrupt source for AUX CH access error */
	reg = analogix_dp_read(dp, ANALOGIX_DP_INT_STA);
	if (reg & AUX_ERR) {
		analogix_dp_write(dp, ANALOGIX_DP_INT_STA, AUX_ERR);
		return -EREMOTEIO;
	}

	/* Check AUX CH error access status */
	reg = analogix_dp_read(dp, ANALOGIX_DP_AUX_CH_STA);
	if ((reg & AUX_STATUS_MASK)) {
		dev_err(dp->dev, "AUX CH error happened: %d\n\n",
			reg & AUX_STATUS_MASK);
		return -EREMOTEIO;
	}

	if (msg->request & DP_AUX_I2C_READ) {
		for (i = 0; i < msg->size; i++) {
			reg = analogix_dp_read(dp,
					       ANALOGIX_DP_BUF_DATA_0 + 4 * i);
			buffer[i] = (unsigned char)reg;
			num_transferred++;
		}
	}

	/* Check if Rx sends defer */
	reg = analogix_dp_read(dp, ANALOGIX_DP_AUX_RX_COMM);
	if (reg == AUX_RX_COMM_AUX_DEFER)
		msg->reply = DP_AUX_NATIVE_REPLY_DEFER;
	else if (reg == AUX_RX_COMM_I2C_DEFER)
		msg->reply = DP_AUX_I2C_REPLY_DEFER;
	else if ((msg->request & ~DP_AUX_I2C_MOT) == DP_AUX_I2C_WRITE ||
		 (msg->request & ~DP_AUX_I2C_MOT) == DP_AUX_I2C_READ)
		msg->reply = DP_AUX_I2C_REPLY_ACK;
	else if ((msg->request & ~DP_AUX_I2C_MOT) == DP_AUX_NATIVE_WRITE ||
		 (msg->request & ~DP_AUX_I2C_MOT) == DP_AUX_NATIVE_READ)
		msg->reply = DP_AUX_NATIVE_REPLY_ACK;

	return num_transferred;
}

void analogix_dp_ssc_enable(struct analogix_dp_device *dp)
{
	u8 reg;

	/* 4500ppm */
	analogix_dp_write(dp, ANALOIGX_DP_SSC_REG, 0x19);
	/*
	 * To apply updated SSC parameters into SSC operation,
	 * firmware must disable and enable this bit.
	 */
	reg = analogix_dp_read(dp, ANALOGIX_DP_FUNC_EN_2);
	reg |= SSC_FUNC_EN_N;
	analogix_dp_write(dp, ANALOGIX_DP_FUNC_EN_2, reg);
	reg &= ~SSC_FUNC_EN_N;
	analogix_dp_write(dp, ANALOGIX_DP_FUNC_EN_2, reg);
}

void analogix_dp_ssc_disable(struct analogix_dp_device *dp)
{
	u8 reg;

	reg = analogix_dp_read(dp, ANALOGIX_DP_FUNC_EN_2);
	reg |= SSC_FUNC_EN_N;
	analogix_dp_write(dp, ANALOGIX_DP_FUNC_EN_2, reg);
}
