/*
* Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
* Author:
*      Andy yan <andy.yan@rock-chips.com>
*      Jeff chen <jeff.chen@rock-chips.com>
*
* based on exynos_dp_core.h
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation; either version 2 of the License, or (at your
* option) any later version.
*/

#ifndef _ROCKCHIP_EDP_CORE_H
#define _ROCKCHIP_EDP_CORE_H

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_panel.h>
#include "rockchip_drm_drv.h"

#define DP_TIMEOUT_LOOP_CNT 100
#define MAX_CR_LOOP 5
#define MAX_EQ_LOOP 5

enum link_lane_count_type {
	LANE_CNT1 = 1,
	LANE_CNT2 = 2,
	LANE_CNT4 = 4
};

enum link_training_state {
	LT_START,
	LT_CLK_RECOVERY,
	LT_EQ_TRAINING,
	FINISHED,
	FAILED
};

enum voltage_swing_level {
	VOLTAGE_LEVEL_0,
	VOLTAGE_LEVEL_1,
	VOLTAGE_LEVEL_2,
	VOLTAGE_LEVEL_3,
};

enum pre_emphasis_level {
	PRE_EMPHASIS_LEVEL_0,
	PRE_EMPHASIS_LEVEL_1,
	PRE_EMPHASIS_LEVEL_2,
	PRE_EMPHASIS_LEVEL_3,
};

enum pattern_set {
	PRBS7,
	D10_2,
	TRAINING_PTN1,
	TRAINING_PTN2,
	DP_NONE
};

enum color_space {
	CS_RGB,
	CS_YCBCR422,
	CS_YCBCR444
};

enum color_depth {
	COLOR_6,
	COLOR_8,
	COLOR_10,
	COLOR_12
};

enum color_coefficient {
	COLOR_YCBCR601,
	COLOR_YCBCR709
};

enum dynamic_range {
	VESA,
	CEA
};

enum pll_status {
	DP_PLL_UNLOCKED,
	DP_PLL_LOCKED
};

enum clock_recovery_m_value_type {
	CALCULATED_M,
	REGISTER_M
};

enum video_timing_recognition_type {
	VIDEO_TIMING_FROM_CAPTURE,
	VIDEO_TIMING_FROM_REGISTER
};

enum analog_power_block {
	AUX_BLOCK,
	CH0_BLOCK,
	CH1_BLOCK,
	CH2_BLOCK,
	CH3_BLOCK,
	ANALOG_TOTAL,
	POWER_ALL
};

enum dp_irq_type {
	DP_IRQ_TYPE_HP_CABLE_IN,
	DP_IRQ_TYPE_HP_CABLE_OUT,
	DP_IRQ_TYPE_HP_CHANGE,
	DP_IRQ_TYPE_UNKNOWN,
};

struct video_info {
	char *name;

	bool h_sync_polarity;
	bool v_sync_polarity;
	bool interlaced;

	enum color_space color_space;
	enum dynamic_range dynamic_range;
	enum color_coefficient ycbcr_coeff;
	enum color_depth color_depth;

	u8 link_rate;
	enum link_lane_count_type lane_count;
};

struct rockchip_edp_device {
	struct device *dev;
	struct drm_device *drm_dev;
	struct drm_panel *panel;
	struct drm_connector connector;
	struct drm_encoder encoder;
	struct drm_display_mode mode;

	void __iomem *regs;
	struct regmap *grf;
	unsigned int irq;
	struct clk *clk_edp;
	struct clk *clk_24m_parent;
	struct clk *clk_24m;
	struct clk *pclk;
	struct reset_control *rst;
	struct video_info video_info;

	int dpms_mode;
	struct drm_dp_aux aux;
	struct drm_dp_link link;
	u8 train_set[4];
	u8 tries;
};

void rockchip_edp_enable_video_mute(struct rockchip_edp_device *edp,
				    bool enable);
void rockchip_edp_stop_video(struct rockchip_edp_device *edp);
void rockchip_edp_lane_swap(struct rockchip_edp_device *edp, bool enable);
void rockchip_edp_init_refclk(struct rockchip_edp_device *edp);
void rockchip_edp_init_interrupt(struct rockchip_edp_device *edp);
void rockchip_edp_reset(struct rockchip_edp_device *edp);
void rockchip_edp_config_interrupt(struct rockchip_edp_device *edp);
u32 rockchip_edp_get_pll_lock_status(struct rockchip_edp_device *edp);
void rockchip_edp_analog_power_ctr(struct rockchip_edp_device *edp,
				   bool enable);
void rockchip_edp_init_analog_func(struct rockchip_edp_device *edp);
void rockchip_edp_force_hpd(struct rockchip_edp_device *edp);
void rockchip_edp_reset_aux(struct rockchip_edp_device *edp);
void rockchip_edp_init_aux(struct rockchip_edp_device *edp);
bool rockchip_edp_get_plug_in_status(struct rockchip_edp_device *edp);
void rockchip_edp_enable_sw_function(struct rockchip_edp_device *edp);
void rockchip_edp_set_link_bandwidth(struct rockchip_edp_device *edp,
				     u32 bwtype);
void rockchip_edp_get_link_bandwidth(struct rockchip_edp_device *edp,
				     u32 *bwtype);
void rockchip_edp_set_lane_count(struct rockchip_edp_device *edp,
				 u32 count);
void rockchip_edp_get_lane_count(struct rockchip_edp_device *edp,
				 u32 *count);
void rockchip_edp_enable_enhanced_mode(struct rockchip_edp_device *edp,
				       bool enable);
void rockchip_edp_set_training_pattern(struct rockchip_edp_device *edp,
				       enum pattern_set pattern);
int rockchip_edp_init_video(struct rockchip_edp_device *edp);

void rockchip_edp_set_video_color_format(struct rockchip_edp_device *edp,
					 u32 color_depth,
					 u32 color_space,
					 u32 dynamic_range,
					 u32 coeff);
int
rockchip_edp_check_video_stream_clock_on(struct rockchip_edp_device *edp);
void rockchip_edp_set_video_cr_mn(struct rockchip_edp_device *edp,
				  enum clock_recovery_m_value_type type,
				  u32 m_value,
				  u32 n_value);
void rockchip_edp_set_video_timing_mode(struct rockchip_edp_device *edp,
					u32 type);
void rockchip_edp_start_video(struct rockchip_edp_device *edp);
int rockchip_edp_is_video_stream_on(struct rockchip_edp_device *edp);
void rockchip_edp_config_video_slave_mode(struct rockchip_edp_device *edp,
					  struct video_info *video_info);
void rockchip_edp_enable_scrambling(struct rockchip_edp_device *edp);
void rockchip_edp_disable_scrambling(struct rockchip_edp_device *edp);
void rockchip_edp_hw_link_training_en(struct rockchip_edp_device *edp);
int rockchip_edp_get_hw_lt_status(struct rockchip_edp_device *edp);
int rockchip_edp_wait_hw_lt_done(struct rockchip_edp_device *edp);
enum dp_irq_type rockchip_edp_get_irq_type(struct rockchip_edp_device *edp);
void rockchip_edp_clear_hotplug_interrupts(struct rockchip_edp_device *edp);
int rockchip_edp_transfer(struct drm_dp_aux *aux, struct drm_dp_aux_msg *msg);
void rockchip_edp_set_link_training(struct rockchip_edp_device *edp,
					void *values);


int rockchip_edp_transfer_from_dpcd(struct drm_dp_aux *aux,
				    struct drm_dp_aux_msg *msg);




/* I2C EDID Chip ID, Slave Address */
#define I2C_EDID_DEVICE_ADDR			0x50
#define I2C_E_EDID_DEVICE_ADDR			0x30

/* DPCD_ADDR_MAX_LANE_COUNT */
#define DPCD_ENHANCED_FRAME_CAP(x)		(((x) >> 7) & 0x1)
#define DPCD_MAX_LANE_COUNT(x)			((x) & 0x1f)

/* DPCD_ADDR_LANE_COUNT_SET */
#define DPCD_LANE_COUNT_SET(x)			((x) & 0x1f)

/* DPCD_ADDR_TRAINING_LANE0_SET */
#define DPCD_PRE_EMPHASIS_SET(x)		(((x) & 0x3) << 3)
#define DPCD_PRE_EMPHASIS_GET(x)		(((x) >> 3) & 0x3)
#define DPCD_VOLTAGE_SWING_SET(x)		(((x) & 0x3) << 0)
#define DPCD_VOLTAGE_SWING_GET(x)		(((x) >> 0) & 0x3)

#endif  /* _ROCKCHIP_EDP_CORE_H */
