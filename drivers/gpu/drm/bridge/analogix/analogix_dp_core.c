// SPDX-License-Identifier: GPL-2.0-or-later
/*
* Analogix DP (Display Port) core interface driver.
*
* Copyright (C) 2012 Samsung Electronics Co., Ltd.
* Author: Jingoo Han <jg1.han@samsung.com>
*/

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/extcon-provider.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#include <drm/bridge/analogix_dp.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include "analogix_dp_core.h"
#include "analogix_dp_reg.h"

#define to_dp(nm)	container_of(nm, struct analogix_dp_device, nm)

static const bool verify_fast_training;

static const unsigned int analogix_dp_cable[] = {
	EXTCON_DISP_DP,
	EXTCON_NONE,
};

struct bridge_init {
	struct i2c_client *client;
	struct device_node *node;
};

static bool analogix_dp_bandwidth_ok(struct analogix_dp_device *dp,
				     const struct drm_display_mode *mode,
				     unsigned int rate, unsigned int lanes)
{
	u32 max_bw, req_bw, bpp = 24;

	req_bw = mode->clock * bpp / 8;
	max_bw = lanes * rate;
	if (req_bw > max_bw)
		return false;

	return true;
}

static int analogix_dp_init_dp(struct analogix_dp_device *dp)
{
	int ret;

	analogix_dp_reset(dp);

	analogix_dp_swreset(dp);

	analogix_dp_init_analog_param(dp);
	analogix_dp_init_interrupt(dp);

	/* SW defined function Normal operation */
	analogix_dp_enable_sw_function(dp);

	analogix_dp_config_interrupt(dp);
	ret = analogix_dp_init_analog_func(dp);
	if (ret)
		return ret;

	analogix_dp_init_hpd(dp);
	analogix_dp_init_aux(dp);
	return 0;
}

static int analogix_dp_panel_prepare(struct analogix_dp_device *dp)
{
	int ret = 0;

	mutex_lock(&dp->panel_lock);

	if (dp->panel_is_prepared)
		goto out;

	ret = drm_panel_prepare(dp->plat_data->panel);
	if (ret)
		goto out;

	dp->panel_is_prepared = true;

out:
	mutex_unlock(&dp->panel_lock);
	return ret;
}

static int analogix_dp_panel_unprepare(struct analogix_dp_device *dp)
{
	int ret;

	mutex_lock(&dp->panel_lock);

	if (!dp->panel_is_prepared)
		goto out;

	ret = drm_panel_unprepare(dp->plat_data->panel);
	if (ret)
		goto out;

	dp->panel_is_prepared = false;

out:
	mutex_unlock(&dp->panel_lock);
	return 0;
}

static int analogix_dp_detect_hpd(struct analogix_dp_device *dp)
{
	int timeout_loop = 0;

	while (timeout_loop < DP_TIMEOUT_LOOP_COUNT) {
		if (analogix_dp_get_plug_in_status(dp) == 0)
			return 0;

		timeout_loop++;
		usleep_range(1000, 1100);
	}

	/*
	 * Some edp screen do not have hpd signal, so we can't just
	 * return failed when hpd plug in detect failed, DT property
	 * "force-hpd" would indicate whether driver need this.
	 */
	if (!dp->force_hpd)
		return -ETIMEDOUT;

	/*
	 * The eDP TRM indicate that if HPD_STATUS(RO) is 0, AUX CH
	 * will not work, so we need to give a force hpd action to
	 * set HPD_STATUS manually.
	 */
	dev_dbg(dp->dev, "failed to get hpd plug status, try to force hpd\n");

	analogix_dp_force_hpd(dp);

	if (analogix_dp_get_plug_in_status(dp) != 0) {
		dev_err(dp->dev, "failed to get hpd plug in status\n");
		return -EINVAL;
	}

	dev_dbg(dp->dev, "success to get plug in status after force hpd\n");

	return 0;
}

static bool analogix_dp_detect_sink_psr(struct analogix_dp_device *dp)
{
	unsigned char psr_version;
	int ret;

	if (!device_property_read_bool(dp->dev, "support-psr"))
		return 0;

	ret = drm_dp_dpcd_readb(&dp->aux, DP_PSR_SUPPORT, &psr_version);
	if (ret != 1) {
		dev_err(dp->dev, "failed to get PSR version, disable it\n");
		return false;
	}

	dev_dbg(dp->dev, "Panel PSR version : %x\n", psr_version);
	return psr_version & DP_PSR_IS_SUPPORTED;
}

static int analogix_dp_enable_sink_psr(struct analogix_dp_device *dp)
{
	unsigned char psr_en;
	int ret;

	/* Disable psr function */
	ret = drm_dp_dpcd_readb(&dp->aux, DP_PSR_EN_CFG, &psr_en);
	if (ret != 1) {
		dev_err(dp->dev, "failed to get psr config\n");
		goto end;
	}

	psr_en &= ~DP_PSR_ENABLE;
	ret = drm_dp_dpcd_writeb(&dp->aux, DP_PSR_EN_CFG, psr_en);
	if (ret != 1) {
		dev_err(dp->dev, "failed to disable panel psr\n");
		goto end;
	}

	/* Main-Link transmitter remains active during PSR active states */
	psr_en = DP_PSR_CRC_VERIFICATION;
	ret = drm_dp_dpcd_writeb(&dp->aux, DP_PSR_EN_CFG, psr_en);
	if (ret != 1) {
		dev_err(dp->dev, "failed to set panel psr\n");
		goto end;
	}

	/* Enable psr function */
	psr_en = DP_PSR_ENABLE | DP_PSR_CRC_VERIFICATION;
	ret = drm_dp_dpcd_writeb(&dp->aux, DP_PSR_EN_CFG, psr_en);
	if (ret != 1) {
		dev_err(dp->dev, "failed to set panel psr\n");
		goto end;
	}

	analogix_dp_enable_psr_crc(dp);

	dp->psr_supported = true;

	return 0;
end:
	dev_err(dp->dev, "enable psr fail, force to disable psr\n");

	return ret;
}

static int
analogix_dp_enable_rx_to_enhanced_mode(struct analogix_dp_device *dp,
				       bool enable)
{
	u8 data;
	int ret;

	ret = drm_dp_dpcd_readb(&dp->aux, DP_LANE_COUNT_SET, &data);
	if (ret != 1)
		return ret;

	if (enable)
		ret = drm_dp_dpcd_writeb(&dp->aux, DP_LANE_COUNT_SET,
					 DP_LANE_COUNT_ENHANCED_FRAME_EN |
					 DPCD_LANE_COUNT_SET(data));
	else
		ret = drm_dp_dpcd_writeb(&dp->aux, DP_LANE_COUNT_SET,
					 DPCD_LANE_COUNT_SET(data));

	return ret < 0 ? ret : 0;
}

static int analogix_dp_is_enhanced_mode_available(struct analogix_dp_device *dp,
						  u8 *enhanced_mode_support)
{
	u8 data;
	int ret;

	ret = drm_dp_dpcd_readb(&dp->aux, DP_MAX_LANE_COUNT, &data);
	if (ret != 1) {
		*enhanced_mode_support = 0;
		return ret;
	}

	*enhanced_mode_support = DPCD_ENHANCED_FRAME_CAP(data);

	return 0;
}

static int analogix_dp_set_enhanced_mode(struct analogix_dp_device *dp)
{
	u8 data;
	int ret;

	ret = analogix_dp_is_enhanced_mode_available(dp, &data);
	if (ret < 0)
		return ret;

	ret = analogix_dp_enable_rx_to_enhanced_mode(dp, data);
	if (ret < 0)
		return ret;

	if (!data) {
		/*
		 * A setting of 1 indicates that this is an eDP device that
		 * uses only Enhanced Framing, independently of the setting by
		 * the source of ENHANCED_FRAME_EN
		 */
		ret = drm_dp_dpcd_readb(&dp->aux, DP_EDP_CONFIGURATION_CAP,
					&data);
		if (ret < 0)
			return ret;

		data = !!(data & DP_FRAMING_CHANGE_CAP);
	}

	analogix_dp_enable_enhanced_mode(dp, data);

	dp->link_train.enhanced_framing = data;

	return 0;
}

static int analogix_dp_training_pattern_dis(struct analogix_dp_device *dp)
{
	int ret;

	analogix_dp_set_training_pattern(dp, DP_NONE);

	ret = drm_dp_dpcd_writeb(&dp->aux, DP_TRAINING_PATTERN_SET,
				 DP_TRAINING_PATTERN_DISABLE);

	return ret < 0 ? ret : 0;
}

static bool analogix_dp_get_vrr_capable(struct analogix_dp_device *dp)
{
	struct drm_connector *connector = &dp->connector;
	struct drm_display_info *info = &connector->display_info;

	if (!info->monitor_range.max_vfreq)
		return false;
	if (!info->monitor_range.min_vfreq)
		return false;
	if (info->monitor_range.max_vfreq < info->monitor_range.min_vfreq)
		return false;
	if (!drm_dp_sink_can_do_video_without_timing_msa(dp->dpcd))
		return false;

	return true;
}

static int analogix_dp_link_start(struct analogix_dp_device *dp)
{
	u8 buf[4];
	int lane, lane_count, retval;

	lane_count = dp->link_train.lane_count;

	dp->link_train.lt_state = CLOCK_RECOVERY;
	dp->link_train.eq_loop = 0;

	for (lane = 0; lane < lane_count; lane++)
		dp->link_train.cr_loop[lane] = 0;

	/* Set link rate and count as you want to establish*/
	analogix_dp_set_link_bandwidth(dp, dp->link_train.link_rate);
	analogix_dp_set_lane_count(dp, dp->link_train.lane_count);

	/* Setup RX configuration */
	buf[0] = dp->link_train.link_rate;
	buf[1] = dp->link_train.lane_count;
	retval = drm_dp_dpcd_write(&dp->aux, DP_LINK_BW_SET, buf, 2);
	if (retval < 0)
		return retval;

	/* Spread AMP if required, enable 8b/10b coding */
	buf[0] = analogix_dp_ssc_supported(dp) ? DP_SPREAD_AMP_0_5 : 0;
	if (analogix_dp_get_vrr_capable(dp))
		buf[0] |= DP_MSA_TIMING_PAR_IGNORE_EN;
	buf[1] = DP_SET_ANSI_8B10B;
	retval = drm_dp_dpcd_write(&dp->aux, DP_DOWNSPREAD_CTRL, buf, 2);
	if (retval < 0)
		return retval;

	/* set enhanced mode if available */
	retval = analogix_dp_set_enhanced_mode(dp);
	if (retval < 0) {
		dev_err(dp->dev, "failed to set enhance mode\n");
		return retval;
	}

	/* Set TX voltage-swing and pre-emphasis to minimum */
	for (lane = 0; lane < lane_count; lane++)
		dp->link_train.training_lane[lane] =
					DP_TRAIN_VOLTAGE_SWING_LEVEL_0 |
					DP_TRAIN_PRE_EMPH_LEVEL_0;
	analogix_dp_set_lane_link_training(dp);

	/* Set training pattern 1 */
	analogix_dp_set_training_pattern(dp, TRAINING_PTN1);

	/* Set RX training pattern */
	retval = drm_dp_dpcd_writeb(&dp->aux, DP_TRAINING_PATTERN_SET,
				    DP_LINK_SCRAMBLING_DISABLE |
					DP_TRAINING_PATTERN_1);
	if (retval < 0)
		return retval;

	for (lane = 0; lane < lane_count; lane++)
		buf[lane] = DP_TRAIN_PRE_EMPH_LEVEL_0 |
			    DP_TRAIN_VOLTAGE_SWING_LEVEL_0;

	retval = drm_dp_dpcd_write(&dp->aux, DP_TRAINING_LANE0_SET, buf,
				   lane_count);
	if (retval < 0)
		return retval;

	return 0;
}

static unsigned char analogix_dp_get_lane_status(u8 link_status[2], int lane)
{
	int shift = (lane & 1) * 4;
	u8 link_value = link_status[lane >> 1];

	return (link_value >> shift) & 0xf;
}

static int analogix_dp_clock_recovery_ok(u8 link_status[2], int lane_count)
{
	int lane;
	u8 lane_status;

	for (lane = 0; lane < lane_count; lane++) {
		lane_status = analogix_dp_get_lane_status(link_status, lane);
		if ((lane_status & DP_LANE_CR_DONE) == 0)
			return -EINVAL;
	}
	return 0;
}

static int analogix_dp_channel_eq_ok(u8 link_status[2], u8 link_align,
				     int lane_count)
{
	int lane;
	u8 lane_status;

	if ((link_align & DP_INTERLANE_ALIGN_DONE) == 0)
		return -EINVAL;

	for (lane = 0; lane < lane_count; lane++) {
		lane_status = analogix_dp_get_lane_status(link_status, lane);
		lane_status &= DP_CHANNEL_EQ_BITS;
		if (lane_status != DP_CHANNEL_EQ_BITS)
			return -EINVAL;
	}

	return 0;
}

static unsigned char
analogix_dp_get_adjust_request_voltage(u8 adjust_request[2], int lane)
{
	int shift = (lane & 1) * 4;
	u8 link_value = adjust_request[lane >> 1];

	return (link_value >> shift) & 0x3;
}

static unsigned char analogix_dp_get_adjust_request_pre_emphasis(
					u8 adjust_request[2],
					int lane)
{
	int shift = (lane & 1) * 4;
	u8 link_value = adjust_request[lane >> 1];

	return ((link_value >> shift) & 0xc) >> 2;
}

static void analogix_dp_reduce_link_rate(struct analogix_dp_device *dp)
{
	analogix_dp_training_pattern_dis(dp);
	analogix_dp_set_enhanced_mode(dp);

	dp->link_train.lt_state = FAILED;
}

static void analogix_dp_get_adjust_training_lane(struct analogix_dp_device *dp,
						 u8 adjust_request[2])
{
	int lane, lane_count;
	u8 voltage_swing, pre_emphasis, training_lane;

	lane_count = dp->link_train.lane_count;
	for (lane = 0; lane < lane_count; lane++) {
		voltage_swing = analogix_dp_get_adjust_request_voltage(
						adjust_request, lane);
		pre_emphasis = analogix_dp_get_adjust_request_pre_emphasis(
						adjust_request, lane);
		training_lane = DPCD_VOLTAGE_SWING_SET(voltage_swing) |
				DPCD_PRE_EMPHASIS_SET(pre_emphasis);

		if (voltage_swing == VOLTAGE_LEVEL_3)
			training_lane |= DP_TRAIN_MAX_SWING_REACHED;
		if (pre_emphasis == PRE_EMPHASIS_LEVEL_3)
			training_lane |= DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;

		dp->link_train.training_lane[lane] = training_lane;
	}
}

static bool analogix_dp_tps3_supported(struct analogix_dp_device *dp)
{
	bool source_tps3_supported, sink_tps3_supported;
	u8 dpcd = 0;

	source_tps3_supported =
		dp->video_info.max_link_rate == DP_LINK_BW_5_4;
	drm_dp_dpcd_readb(&dp->aux, DP_MAX_LANE_COUNT, &dpcd);
	sink_tps3_supported = dpcd & DP_TPS3_SUPPORTED;

	return source_tps3_supported && sink_tps3_supported;
}

static int analogix_dp_process_clock_recovery(struct analogix_dp_device *dp)
{
	int lane, lane_count, retval;
	u8 voltage_swing, pre_emphasis, training_lane;
	u8 link_status[2], adjust_request[2];
	u8 training_pattern = TRAINING_PTN2;

	drm_dp_link_train_clock_recovery_delay(dp->dpcd);

	lane_count = dp->link_train.lane_count;

	retval = drm_dp_dpcd_read(&dp->aux, DP_LANE0_1_STATUS, link_status, 2);
	if (retval < 0)
		return retval;

	if (analogix_dp_clock_recovery_ok(link_status, lane_count) == 0) {
		if (analogix_dp_tps3_supported(dp))
			training_pattern = TRAINING_PTN3;

		/* set training pattern for EQ */
		analogix_dp_set_training_pattern(dp, training_pattern);

		retval = drm_dp_dpcd_writeb(&dp->aux, DP_TRAINING_PATTERN_SET,
					    DP_LINK_SCRAMBLING_DISABLE |
					    (training_pattern == TRAINING_PTN3 ?
					     DP_TRAINING_PATTERN_3 : DP_TRAINING_PATTERN_2));
		if (retval < 0)
			return retval;

		dev_dbg(dp->dev, "Link Training Clock Recovery success\n");
		dp->link_train.lt_state = EQUALIZER_TRAINING;

		return 0;
	} else {
		retval = drm_dp_dpcd_read(&dp->aux, DP_ADJUST_REQUEST_LANE0_1,
					  adjust_request, 2);
		if (retval < 0)
			return retval;

		for (lane = 0; lane < lane_count; lane++) {
			training_lane = analogix_dp_get_lane_link_training(
							dp, lane);
			voltage_swing = analogix_dp_get_adjust_request_voltage(
							adjust_request, lane);
			pre_emphasis = analogix_dp_get_adjust_request_pre_emphasis(
							adjust_request, lane);

			if (DPCD_VOLTAGE_SWING_GET(training_lane) ==
					voltage_swing &&
			    DPCD_PRE_EMPHASIS_GET(training_lane) ==
					pre_emphasis)
				dp->link_train.cr_loop[lane]++;

			if (dp->link_train.cr_loop[lane] == MAX_CR_LOOP ||
			    voltage_swing == VOLTAGE_LEVEL_3 ||
			    pre_emphasis == PRE_EMPHASIS_LEVEL_3) {
				dev_err(dp->dev, "CR Max reached (%d,%d,%d)\n",
					dp->link_train.cr_loop[lane],
					voltage_swing, pre_emphasis);
				analogix_dp_reduce_link_rate(dp);
				return -EIO;
			}
		}
	}

	analogix_dp_get_adjust_training_lane(dp, adjust_request);
	analogix_dp_set_lane_link_training(dp);

	retval = drm_dp_dpcd_write(&dp->aux, DP_TRAINING_LANE0_SET,
				   dp->link_train.training_lane, lane_count);
	if (retval < 0)
		return retval;

	return 0;
}

static int analogix_dp_process_equalizer_training(struct analogix_dp_device *dp)
{
	int lane_count, retval;
	u32 reg;
	u8 link_align, link_status[2], adjust_request[2];

	drm_dp_link_train_channel_eq_delay(dp->dpcd);

	lane_count = dp->link_train.lane_count;

	retval = drm_dp_dpcd_read(&dp->aux, DP_LANE0_1_STATUS, link_status, 2);
	if (retval < 0)
		return retval;

	if (analogix_dp_clock_recovery_ok(link_status, lane_count)) {
		analogix_dp_reduce_link_rate(dp);
		return -EIO;
	}

	retval = drm_dp_dpcd_readb(&dp->aux, DP_LANE_ALIGN_STATUS_UPDATED,
				   &link_align);
	if (retval < 0)
		return retval;

	if (!analogix_dp_channel_eq_ok(link_status, link_align, lane_count)) {
		/* traing pattern Set to Normal */
		retval = analogix_dp_training_pattern_dis(dp);
		if (retval < 0)
			return retval;

		dev_dbg(dp->dev, "Link Training success!\n");
		analogix_dp_get_link_bandwidth(dp, &reg);
		dp->link_train.link_rate = reg;
		dev_dbg(dp->dev, "final bandwidth = %.2x\n",
			dp->link_train.link_rate);

		analogix_dp_get_lane_count(dp, &reg);
		dp->link_train.lane_count = reg;
		dev_dbg(dp->dev, "final lane count = %.2x\n",
			dp->link_train.lane_count);

		dp->link_train.lt_state = FINISHED;

		return 0;
	}

	/* not all locked */
	dp->link_train.eq_loop++;

	if (dp->link_train.eq_loop > MAX_EQ_LOOP) {
		dev_err(dp->dev, "EQ Max loop\n");
		analogix_dp_reduce_link_rate(dp);
		return -EIO;
	}

	retval = drm_dp_dpcd_read(&dp->aux, DP_ADJUST_REQUEST_LANE0_1,
				  adjust_request, 2);
	if (retval < 0)
		return retval;

	analogix_dp_get_adjust_training_lane(dp, adjust_request);
	analogix_dp_set_lane_link_training(dp);

	retval = drm_dp_dpcd_write(&dp->aux, DP_TRAINING_LANE0_SET,
				   dp->link_train.training_lane, lane_count);
	if (retval < 0)
		return retval;

	return 0;
}

static int analogix_dp_get_max_rx_bandwidth(struct analogix_dp_device *dp,
					    u8 *bandwidth)
{
	u8 data;
	int ret;

	/*
	 * For DP rev.1.1, Maximum link rate of Main Link lanes
	 * 0x06 = 1.62 Gbps, 0x0a = 2.7 Gbps
	 * For DP rev.1.2, Maximum link rate of Main Link lanes
	 * 0x06 = 1.62 Gbps, 0x0a = 2.7 Gbps, 0x14 = 5.4Gbps
	 */
	ret = drm_dp_dpcd_readb(&dp->aux, DP_MAX_LINK_RATE, &data);
	if (ret < 0)
		return ret;

	*bandwidth = data;

	return 0;
}

static int analogix_dp_get_max_rx_lane_count(struct analogix_dp_device *dp,
					     u8 *lane_count)
{
	u8 data;
	int ret;

	/*
	 * For DP rev.1.1, Maximum number of Main Link lanes
	 * 0x01 = 1 lane, 0x02 = 2 lanes, 0x04 = 4 lanes
	 */
	ret = drm_dp_dpcd_readb(&dp->aux, DP_MAX_LANE_COUNT, &data);
	if (ret < 0)
		return ret;

	*lane_count = DPCD_MAX_LANE_COUNT(data);

	return 0;
}

static int analogix_dp_full_link_train(struct analogix_dp_device *dp,
				       u32 max_lanes, u32 max_rate)
{
	struct video_info *video = &dp->video_info;
	int retval = 0;
	bool training_finished = false;
	u8 dpcd;

	/*
	 * MACRO_RST must be applied after the PLL_LOCK to avoid
	 * the DP inter pair skew issue for at least 10 us
	 */
	analogix_dp_reset_macro(dp);

	/* Initialize by reading RX's DPCD */
	analogix_dp_get_max_rx_bandwidth(dp, &dp->link_train.link_rate);
	analogix_dp_get_max_rx_lane_count(dp, &dp->link_train.lane_count);

	/* Setup TX lane count & rate */
	dp->link_train.lane_count = min_t(u32, dp->link_train.lane_count, max_lanes);
	dp->link_train.link_rate = min_t(u32, dp->link_train.link_rate, max_rate);

	if (!analogix_dp_bandwidth_ok(dp, &video->mode,
				      drm_dp_bw_code_to_link_rate(dp->link_train.link_rate),
				      dp->link_train.lane_count)) {
		dev_err(dp->dev, "bandwidth overflow\n");
		return -EINVAL;
	}

	drm_dp_dpcd_readb(&dp->aux, DP_MAX_DOWNSPREAD, &dpcd);
	dp->link_train.ssc = !!(dpcd & DP_MAX_DOWNSPREAD_0_5);

	/* All DP analog module power up */
	analogix_dp_set_analog_power_down(dp, POWER_ALL, 0);

	dp->link_train.lt_state = START;

	/* Process here */
	while (!retval && !training_finished) {
		switch (dp->link_train.lt_state) {
		case START:
			retval = analogix_dp_link_start(dp);
			if (retval)
				dev_err(dp->dev, "LT link start failed!\n");
			break;
		case CLOCK_RECOVERY:
			retval = analogix_dp_process_clock_recovery(dp);
			if (retval)
				dev_err(dp->dev, "LT CR failed!\n");
			break;
		case EQUALIZER_TRAINING:
			retval = analogix_dp_process_equalizer_training(dp);
			if (retval)
				dev_err(dp->dev, "LT EQ failed!\n");
			break;
		case FINISHED:
			training_finished = 1;
			break;
		case FAILED:
			return -EREMOTEIO;
		}
	}
	if (retval)
		dev_err(dp->dev, "eDP link training failed (%d)\n", retval);

	return retval;
}

static int analogix_dp_fast_link_train(struct analogix_dp_device *dp)
{
	int ret;
	u8 link_align, link_status[2];

	analogix_dp_reset_macro(dp);

	analogix_dp_set_link_bandwidth(dp, dp->link_train.link_rate);
	analogix_dp_set_lane_count(dp, dp->link_train.lane_count);
	analogix_dp_set_lane_link_training(dp);
	analogix_dp_enable_enhanced_mode(dp, dp->link_train.enhanced_framing);

	/* source Set training pattern 1 */
	analogix_dp_set_training_pattern(dp, TRAINING_PTN1);
	/* From DP spec, pattern must be on-screen for a minimum 500us */
	usleep_range(500, 600);

	analogix_dp_set_training_pattern(dp, TRAINING_PTN2);
	/* From DP spec, pattern must be on-screen for a minimum 500us */
	usleep_range(500, 600);

	analogix_dp_set_training_pattern(dp, DP_NONE);

	/*
	 * Useful for debugging issues with fast link training, disable for more
	 * speed
	 */
	if (verify_fast_training) {
		ret = drm_dp_dpcd_readb(&dp->aux, DP_LANE_ALIGN_STATUS_UPDATED,
					&link_align);
		if (ret < 0) {
			DRM_DEV_ERROR(dp->dev, "Read align status failed %d\n",
				      ret);
			return ret;
		}

		ret = drm_dp_dpcd_read(&dp->aux, DP_LANE0_1_STATUS, link_status,
				       2);
		if (ret < 0) {
			DRM_DEV_ERROR(dp->dev, "Read link status failed %d\n",
				      ret);
			return ret;
		}

		if (analogix_dp_clock_recovery_ok(link_status,
						  dp->link_train.lane_count)) {
			DRM_DEV_ERROR(dp->dev, "Clock recovery failed\n");
			analogix_dp_reduce_link_rate(dp);
			return -EIO;
		}

		if (analogix_dp_channel_eq_ok(link_status, link_align,
					      dp->link_train.lane_count)) {
			DRM_DEV_ERROR(dp->dev, "Channel EQ failed\n");
			analogix_dp_reduce_link_rate(dp);
			return -EIO;
		}
	}

	return 0;
}

static int analogix_dp_train_link(struct analogix_dp_device *dp)
{
	if (dp->fast_train_enable)
		return analogix_dp_fast_link_train(dp);

	return analogix_dp_full_link_train(dp, dp->video_info.max_lane_count,
					   dp->video_info.max_link_rate);
}

static int analogix_dp_config_video(struct analogix_dp_device *dp)
{
	int timeout_loop = 0;
	int done_count = 0;

	analogix_dp_config_video_slave_mode(dp);

	analogix_dp_set_video_color_format(dp);

	if (analogix_dp_get_pll_lock_status(dp) == PLL_UNLOCKED) {
		dev_err(dp->dev, "PLL is not locked yet.\n");
		return -EINVAL;
	}

	for (;;) {
		timeout_loop++;
		if (analogix_dp_is_slave_video_stream_clock_on(dp) == 0)
			break;
		if (timeout_loop > DP_TIMEOUT_LOOP_COUNT) {
			dev_err(dp->dev, "Timeout of slave video streamclk ok\n");
			return -ETIMEDOUT;
		}
		usleep_range(1000, 1001);
	}

	/* Set to use the register calculated M/N video */
	analogix_dp_set_video_cr_mn(dp, CALCULATED_M, 0, 0);

	/* For video bist, Video timing must be generated by register */
	analogix_dp_set_video_timing_mode(dp, VIDEO_TIMING_FROM_REGISTER);

	/* Disable video mute */
	analogix_dp_enable_video_mute(dp, 0);

	/* Configure video slave mode */
	analogix_dp_enable_video_master(dp, 0);

	/* Enable video */
	analogix_dp_start_video(dp);

	timeout_loop = 0;

	for (;;) {
		timeout_loop++;
		if (analogix_dp_is_video_stream_on(dp) == 0) {
			done_count++;
			if (done_count > 10)
				break;
		} else if (done_count) {
			done_count = 0;
		}
		if (timeout_loop > DP_TIMEOUT_LOOP_COUNT) {
			dev_warn(dp->dev,
				 "Ignoring timeout of video streamclk ok\n");
			break;
		}

		usleep_range(1000, 1001);
	}

	return 0;
}

static int analogix_dp_enable_scramble(struct analogix_dp_device *dp,
				       bool enable)
{
	u8 data;
	int ret;

	if (enable) {
		analogix_dp_enable_scrambling(dp);

		ret = drm_dp_dpcd_readb(&dp->aux, DP_TRAINING_PATTERN_SET,
					&data);
		if (ret != 1)
			return ret;
		ret = drm_dp_dpcd_writeb(&dp->aux, DP_TRAINING_PATTERN_SET,
				   (u8)(data & ~DP_LINK_SCRAMBLING_DISABLE));
	} else {
		analogix_dp_disable_scrambling(dp);

		ret = drm_dp_dpcd_readb(&dp->aux, DP_TRAINING_PATTERN_SET,
					&data);
		if (ret != 1)
			return ret;
		ret = drm_dp_dpcd_writeb(&dp->aux, DP_TRAINING_PATTERN_SET,
				   (u8)(data | DP_LINK_SCRAMBLING_DISABLE));
	}
	return ret < 0 ? ret : 0;
}

static u8 analogix_dp_autotest_phy_pattern(struct analogix_dp_device *dp)
{
	struct drm_dp_phy_test_params *data = &dp->compliance.phytest;

	if (drm_dp_get_phy_test_pattern(&dp->aux, data)) {
		dev_err(dp->dev, "DP Phy Test pattern AUX read failure\n");
		return DP_TEST_NAK;
	}

	if (data->link_rate > drm_dp_bw_code_to_link_rate(dp->video_info.max_link_rate)) {
		dev_err(dp->dev, "invalid link rate = 0x%x\n", data->link_rate);
		return DP_TEST_NAK;
	}

	/* Set test active flag here so userspace doesn't interrupt things */
	dp->compliance.test_active = true;

	return DP_TEST_ACK;
}

static void analogix_dp_handle_test_request(struct analogix_dp_device *dp)
{
	u8 response = DP_TEST_NAK;
	u8 request = 0;
	int ret;

	ret = drm_dp_dpcd_readb(&dp->aux, DP_TEST_REQUEST, &request);
	if (ret < 0) {
		dev_err(dp->dev, "Could not read test request from sink\n");
		goto update_status;
	}

	switch (request) {
	case DP_TEST_LINK_PHY_TEST_PATTERN:
		dev_info(dp->dev, "PHY_PATTERN test requested\n");
		response = analogix_dp_autotest_phy_pattern(dp);
		break;
	default:
		dev_err(dp->dev, "Invalid test request '%02x'\n", request);
		break;
	}

	if (response & DP_TEST_ACK)
		dp->compliance.test_type = request;

update_status:
	ret = drm_dp_dpcd_writeb(&dp->aux, DP_TEST_RESPONSE, response);
	if (ret < 0)
		dev_err(dp->dev, "Could not write test response to sink\n");
}

void analogix_dp_check_device_service_irq(struct analogix_dp_device *dp)
{
	u8 val;
	int ret;

	ret = drm_dp_dpcd_readb(&dp->aux, DP_DEVICE_SERVICE_IRQ_VECTOR, &val);
	if (ret < 0 || !val)
		return;

	ret = drm_dp_dpcd_writeb(&dp->aux, DP_DEVICE_SERVICE_IRQ_VECTOR, val);
	if (ret < 0)
		return;

	if (val & DP_AUTOMATED_TEST_REQUEST)
		analogix_dp_handle_test_request(dp);
}
EXPORT_SYMBOL_GPL(analogix_dp_check_device_service_irq);

static void analogix_dp_process_phy_request(struct analogix_dp_device *dp)
{
	struct drm_dp_phy_test_params *data = &dp->compliance.phytest;
	u8 spread, adjust_request[2];
	int ret;

	dp->link_train.link_rate = drm_dp_link_rate_to_bw_code(data->link_rate);
	dp->link_train.lane_count = data->num_lanes;

	ret = drm_dp_dpcd_readb(&dp->aux, DP_MAX_DOWNSPREAD, &spread);
	if (ret < 0) {
		dev_err(dp->dev, "Could not read ssc from sink\n");
		return;
	}

	dp->link_train.ssc = !!(spread & DP_MAX_DOWNSPREAD_0_5);

	ret = drm_dp_dpcd_read(&dp->aux, DP_ADJUST_REQUEST_LANE0_1,
			       adjust_request, 2);
	if (ret < 0) {
		dev_err(dp->dev, "Could not read swing/pre-emphasis\n");
		return;
	}

	analogix_dp_set_link_bandwidth(dp, dp->link_train.link_rate);
	analogix_dp_set_lane_count(dp, dp->link_train.lane_count);
	analogix_dp_get_adjust_training_lane(dp, adjust_request);
	analogix_dp_set_lane_link_training(dp);

	switch (data->phy_pattern) {
	case DP_PHY_TEST_PATTERN_NONE:
		dev_info(dp->dev, "Disable Phy Test Pattern\n");
		analogix_dp_set_training_pattern(dp, DP_NONE);
		break;
	case DP_PHY_TEST_PATTERN_D10_2:
		dev_info(dp->dev, "Set D10.2 Phy Test Pattern\n");
		analogix_dp_set_training_pattern(dp, D10_2);
		break;
	case DP_PHY_TEST_PATTERN_PRBS7:
		dev_info(dp->dev, "Set PRBS7 Phy Test Pattern\n");
		analogix_dp_set_training_pattern(dp, PRBS7);
		break;
	case DP_PHY_TEST_PATTERN_80BIT_CUSTOM:
		dev_info(dp->dev, "Set 80Bit Custom Phy Test Pattern\n");
		analogix_dp_set_training_pattern(dp, TEST_PATTERN_80BIT);
		break;
	case DP_PHY_TEST_PATTERN_CP2520:
		dev_info(dp->dev, "Set HBR2 compliance Phy Test Pattern\n");
		analogix_dp_set_training_pattern(dp, TEST_PATTERN_HBR2);
		break;
	default:
		dev_err(dp->dev, "Invalid Phy Test Pattern: %d\n", data->phy_pattern);
		return;
	}

	drm_dp_set_phy_test_pattern(&dp->aux, data, 0x11);
}

void analogix_dp_phy_test(struct analogix_dp_device *dp)
{
	struct drm_device *dev = dp->drm_dev;
	struct drm_modeset_acquire_ctx ctx;
	int ret;

	DRM_DEV_INFO(dp->dev, "PHY test\n");

	drm_modeset_acquire_init(&ctx, 0);
	for (;;) {
		ret = drm_modeset_lock(&dev->mode_config.connection_mutex, &ctx);
		if (ret != -EDEADLK)
			break;

		drm_modeset_backoff(&ctx);
	}

	analogix_dp_process_phy_request(dp);
	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);
}
EXPORT_SYMBOL_GPL(analogix_dp_phy_test);

static irqreturn_t analogix_dp_hpd_irq_handler(int irq, void *arg)
{
	struct analogix_dp_device *dp = arg;

	if (dp->drm_dev)
		drm_helper_hpd_irq_event(dp->drm_dev);

	return IRQ_HANDLED;
}

static irqreturn_t analogix_dp_irq_thread(int irq, void *arg)
{
	struct analogix_dp_device *dp = arg;

	analogix_dp_irq_handler(dp);

	return IRQ_HANDLED;
}

static int analogix_dp_fast_link_train_detection(struct analogix_dp_device *dp)
{
	int ret;
	u8 spread;

	ret = drm_dp_dpcd_readb(&dp->aux, DP_MAX_DOWNSPREAD, &spread);
	if (ret != 1) {
		dev_err(dp->dev, "failed to read downspread %d\n", ret);
		return ret;
	}
	dp->fast_train_enable = !!(spread & DP_NO_AUX_HANDSHAKE_LINK_TRAINING);
	dev_dbg(dp->dev, "fast link training %s\n",
		dp->fast_train_enable ? "supported" : "unsupported");
	return 0;
}

static int analogix_dp_link_power_up(struct analogix_dp_device *dp)
{
	u8 value;
	int ret;

	if (dp->dpcd[DP_DPCD_REV] < 0x11)
		return 0;

	ret = drm_dp_dpcd_readb(&dp->aux, DP_SET_POWER, &value);
	if (ret < 0)
		return ret;

	value &= ~DP_SET_POWER_MASK;
	value |= DP_SET_POWER_D0;

	ret = drm_dp_dpcd_writeb(&dp->aux, DP_SET_POWER, value);
	if (ret < 0)
		return ret;

	usleep_range(1000, 2000);

	return 0;
}

static int analogix_dp_link_power_down(struct analogix_dp_device *dp)
{
	u8 value;
	int ret;

	if (dp->dpcd[DP_DPCD_REV] < 0x11)
		return 0;

	ret = drm_dp_dpcd_readb(&dp->aux, DP_SET_POWER, &value);
	if (ret < 0)
		return ret;

	value &= ~DP_SET_POWER_MASK;
	value |= DP_SET_POWER_D3;

	ret = drm_dp_dpcd_writeb(&dp->aux, DP_SET_POWER, value);
	if (ret < 0)
		return ret;

	return 0;
}

static int analogix_dp_commit(struct analogix_dp_device *dp)
{
	struct video_info *video = &dp->video_info;
	int ret;

	ret = drm_dp_read_dpcd_caps(&dp->aux, dp->dpcd);
	if (ret < 0) {
		dev_err(dp->dev, "failed to read dpcd caps: %d\n", ret);
		return ret;
	}

	ret = analogix_dp_link_power_up(dp);
	if (ret) {
		dev_err(dp->dev, "failed to power up link: %d\n", ret);
		return ret;
	}

	if (device_property_read_bool(dp->dev, "panel-self-test"))
		return drm_dp_dpcd_writeb(&dp->aux, DP_EDP_CONFIGURATION_SET,
					  DP_PANEL_SELF_TEST_ENABLE);

	ret = analogix_dp_train_link(dp);
	if (ret) {
		dev_err(dp->dev, "unable to do link train, ret=%d\n", ret);
		return ret;
	}

	ret = analogix_dp_enable_scramble(dp, 1);
	if (ret < 0) {
		dev_err(dp->dev, "can not enable scramble\n");
		return ret;
	}

	analogix_dp_init_video(dp);
	analogix_dp_set_video_format(dp);

	if (video->video_bist_enable)
		analogix_dp_video_bist_enable(dp);

	ret = analogix_dp_config_video(dp);
	if (ret) {
		dev_err(dp->dev, "unable to config video\n");
		return ret;
	}

	/* Check whether panel supports fast training */
	ret = analogix_dp_fast_link_train_detection(dp);
	if (ret)
		return ret;

	if (analogix_dp_detect_sink_psr(dp)) {
		ret = analogix_dp_enable_sink_psr(dp);
		if (ret)
			return ret;
	}

	return ret;
}

static int analogix_dp_enable_psr(struct analogix_dp_device *dp)
{
	struct dp_sdp psr_vsc;
	int ret;
	u8 sink;

	ret = drm_dp_dpcd_readb(&dp->aux, DP_PSR_STATUS, &sink);
	if (ret != 1)
		DRM_DEV_ERROR(dp->dev, "Failed to read psr status %d\n", ret);
	else if (sink == DP_PSR_SINK_ACTIVE_RFB)
		return 0;

	/* Prepare VSC packet as per EDP 1.4 spec, Table 6.9 */
	memset(&psr_vsc, 0, sizeof(psr_vsc));
	psr_vsc.sdp_header.HB0 = 0;
	psr_vsc.sdp_header.HB1 = 0x7;
	psr_vsc.sdp_header.HB2 = 0x2;
	psr_vsc.sdp_header.HB3 = 0x8;
	psr_vsc.db[0] = 0;
	psr_vsc.db[1] = EDP_VSC_PSR_STATE_ACTIVE | EDP_VSC_PSR_CRC_VALUES_VALID;

	ret = analogix_dp_send_psr_spd(dp, &psr_vsc, true);
	if (!ret) {
		analogix_dp_set_analog_power_down(dp, POWER_ALL, true);

		if (dp->phy) {
			union phy_configure_opts phy_cfg = {0};

			phy_cfg.dp.lanes = 0;
			phy_cfg.dp.set_lanes = true;
			ret = phy_configure(dp->phy, &phy_cfg);
			if (ret)
				return ret;
		}
	}

	return ret;
}

static int analogix_dp_disable_psr(struct analogix_dp_device *dp)
{
	struct dp_sdp psr_vsc;
	int ret;
	u8 sink;

	analogix_dp_set_analog_power_down(dp, POWER_ALL, false);

	ret = drm_dp_dpcd_writeb(&dp->aux, DP_SET_POWER, DP_SET_POWER_D0);
	if (ret != 1) {
		DRM_DEV_ERROR(dp->dev, "Failed to set DP Power0 %d\n", ret);
		return ret;
	}

	ret = drm_dp_dpcd_readb(&dp->aux, DP_PSR_STATUS, &sink);
	if (ret != 1) {
		DRM_DEV_ERROR(dp->dev, "Failed to read psr status %d\n", ret);
		return ret;
	} else if (sink == DP_PSR_SINK_INACTIVE) {
		DRM_DEV_ERROR(dp->dev, "sink inactive, skip disable psr");
		return 0;
	}

	ret = analogix_dp_train_link(dp);
	if (ret) {
		DRM_DEV_ERROR(dp->dev, "Failed to train the link %d\n", ret);
		return ret;
	}

	/* Prepare VSC packet as per EDP 1.4 spec, Table 6.9 */
	memset(&psr_vsc, 0, sizeof(psr_vsc));
	psr_vsc.sdp_header.HB0 = 0;
	psr_vsc.sdp_header.HB1 = 0x7;
	psr_vsc.sdp_header.HB2 = 0x2;
	psr_vsc.sdp_header.HB3 = 0x8;

	psr_vsc.db[0] = 0;
	psr_vsc.db[1] = 0;

	return analogix_dp_send_psr_spd(dp, &psr_vsc, true);
}

static int analogix_dp_get_modes(struct drm_connector *connector)
{
	struct analogix_dp_device *dp = to_dp(connector);
	struct edid *edid;
	int ret, num_modes = 0;

	if (dp->plat_data->right && dp->plat_data->right->plat_data->bridge) {
		struct drm_bridge *bridge = dp->plat_data->right->plat_data->bridge;

		if (bridge->ops & DRM_BRIDGE_OP_MODES) {
			if (!drm_bridge_get_modes(bridge, connector))
				return 0;
		}
	}

	if (dp->plat_data->panel)
		num_modes += drm_panel_get_modes(dp->plat_data->panel, connector);

	if (dp->plat_data->bridge)
		num_modes += drm_bridge_get_modes(dp->plat_data->bridge, connector);

	if (!num_modes) {
		ret = analogix_dp_phy_power_on(dp);
		if (ret)
			return 0;

		if (dp->plat_data->panel)
			analogix_dp_panel_prepare(dp);

		edid = drm_get_edid(connector, &dp->aux.ddc);
		if (edid) {
			drm_connector_update_edid_property(&dp->connector,
							   edid);
			num_modes += drm_add_edid_modes(&dp->connector, edid);
			kfree(edid);
		}

		analogix_dp_phy_power_off(dp);
	}

	if (dp->plat_data->get_modes)
		num_modes += dp->plat_data->get_modes(dp->plat_data, connector);

	if (num_modes > 0 && dp->plat_data->split_mode) {
		struct drm_display_mode *mode;

		list_for_each_entry(mode, &connector->probed_modes, head)
			dp->plat_data->convert_to_split_mode(mode);
	}

	return num_modes;
}

static struct drm_encoder *
analogix_dp_best_encoder(struct drm_connector *connector)
{
	struct analogix_dp_device *dp = to_dp(connector);

	return dp->encoder;
}


static int analogix_dp_atomic_check(struct drm_connector *connector,
				    struct drm_atomic_state *state)
{
	struct analogix_dp_device *dp = to_dp(connector);
	struct drm_connector_state *conn_state;
	struct drm_crtc_state *crtc_state;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (WARN_ON(!conn_state))
		return -ENODEV;

	conn_state->self_refresh_aware = true;

	if (!conn_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	if (!crtc_state)
		return 0;

	if (crtc_state->self_refresh_active && !dp->psr_supported)
		return -EINVAL;

	return 0;
}

static const struct drm_connector_helper_funcs analogix_dp_connector_helper_funcs = {
	.get_modes = analogix_dp_get_modes,
	.best_encoder = analogix_dp_best_encoder,
	.atomic_check = analogix_dp_atomic_check,
};

static enum drm_connector_status
analogix_dp_detect(struct analogix_dp_device *dp)
{
	enum drm_connector_status status = connector_status_disconnected;
	int ret;

	ret = analogix_dp_phy_power_on(dp);
	if (ret) {
		extcon_set_state_sync(dp->extcon, EXTCON_DISP_DP, false);
		return status;
	}

	if (dp->plat_data->panel) {
		ret = analogix_dp_panel_prepare(dp);
		if (ret < 0) {
			dev_dbg(dp->dev, "failed to prepare panel (%d)\n", ret);
			return status;
		}
	}

	if (!analogix_dp_detect_hpd(dp)) {
		ret = analogix_dp_get_max_rx_bandwidth(dp, &dp->link_train.link_rate);
		if (ret) {
			dev_err(dp->dev, "failed to read max link rate\n");
			goto out;
		}

		ret = analogix_dp_get_max_rx_lane_count(dp, &dp->link_train.lane_count);
		if (ret) {
			dev_err(dp->dev, "failed to read max lane count\n");
			goto out;
		}

		status = connector_status_connected;
	}

	if (dp->plat_data->bridge) {
		struct drm_bridge *next_bridge = dp->plat_data->bridge;

		if (next_bridge->ops & DRM_BRIDGE_OP_DETECT)
			status = drm_bridge_detect(next_bridge);
	}

out:
	analogix_dp_phy_power_off(dp);

	if (status == connector_status_connected)
		extcon_set_state_sync(dp->extcon, EXTCON_DISP_DP, true);
	else
		extcon_set_state_sync(dp->extcon, EXTCON_DISP_DP, false);

	return status;
}

static enum drm_connector_status
analogix_dp_connector_detect(struct drm_connector *connector, bool force)
{
	struct analogix_dp_device *dp = to_dp(connector);

	if (dp->plat_data->right && analogix_dp_detect(dp->plat_data->right) != connector_status_connected)
		return connector_status_disconnected;

	return analogix_dp_detect(dp);
}

static void analogix_dp_connector_force(struct drm_connector *connector)
{
	struct analogix_dp_device *dp = to_dp(connector);

	if (connector->status == connector_status_connected)
		extcon_set_state_sync(dp->extcon, EXTCON_DISP_DP, true);
	else
		extcon_set_state_sync(dp->extcon, EXTCON_DISP_DP, false);
}

static const struct drm_connector_funcs analogix_dp_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = analogix_dp_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.force = analogix_dp_connector_force,
};

static int analogix_dp_bridge_attach(struct drm_bridge *bridge,
				     enum drm_bridge_attach_flags flags)
{
	struct analogix_dp_device *dp = bridge->driver_private;
	struct drm_encoder *encoder = dp->encoder;
	struct drm_connector *connector = NULL;
	int ret = 0;

	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found");
		return -ENODEV;
	}

	if (dp->plat_data->bridge) {
		ret = drm_bridge_attach(bridge->encoder, dp->plat_data->bridge, bridge,
					dp->plat_data->skip_connector ?
					0 : DRM_BRIDGE_ATTACH_NO_CONNECTOR);
		if (ret) {
			DRM_ERROR("Failed to attach external bridge: %d\n", ret);
			return ret;
		}
	}

	if (flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR)
		return 0;

	if (!dp->plat_data->skip_connector) {
		int connector_type = DRM_MODE_CONNECTOR_eDP;

		if (dp->plat_data->bridge &&
		    dp->plat_data->bridge->type != DRM_MODE_CONNECTOR_Unknown)
			connector_type = dp->plat_data->bridge->type;

		connector = &dp->connector;
		connector->polled = DRM_CONNECTOR_POLL_HPD;
		if (dp->plat_data->bridge && dp->plat_data->bridge->ops & DRM_BRIDGE_OP_DETECT)
			connector->polled = DRM_CONNECTOR_POLL_CONNECT |
					    DRM_CONNECTOR_POLL_DISCONNECT;

		ret = drm_connector_init(dp->drm_dev, connector,
					 &analogix_dp_connector_funcs,
					 connector_type);
		if (ret) {
			DRM_ERROR("Failed to initialize connector with drm\n");
			return ret;
		}

		drm_connector_helper_add(connector,
					 &analogix_dp_connector_helper_funcs);
		drm_connector_attach_encoder(connector, encoder);
	}

	/*
	 * NOTE: the connector registration is implemented in analogix
	 * platform driver, that to say connector would be exist after
	 * plat_data->attch return, that's why we record the connector
	 * point after plat attached.
	 */
	if (dp->plat_data->attach) {
		ret = dp->plat_data->attach(dp->plat_data, bridge, connector);
		if (ret) {
			DRM_ERROR("Failed at platform attach func\n");
			return ret;
		}
	}

	return 0;
}

static void analogix_dp_bridge_detach(struct drm_bridge *bridge)
{
	struct analogix_dp_device *dp = bridge->driver_private;

	if (dp->plat_data->detach)
		dp->plat_data->detach(dp->plat_data, bridge);
}

static
struct drm_crtc *analogix_dp_get_new_crtc(struct analogix_dp_device *dp,
					  struct drm_atomic_state *state)
{
	struct drm_bridge *bridge = &dp->bridge;
	struct drm_encoder *encoder = bridge->encoder;
	struct drm_connector *connector;
	struct drm_connector_state *conn_state;

	connector = drm_atomic_get_new_connector_for_encoder(state, encoder);
	if (!connector)
		return NULL;

	conn_state = drm_atomic_get_new_connector_state(state, connector);
	if (!conn_state)
		return NULL;

	return conn_state->crtc;
}

static void
analogix_dp_bridge_atomic_pre_enable(struct drm_bridge *bridge,
				     struct drm_bridge_state *old_bridge_state)
{
	struct drm_atomic_state *old_state = old_bridge_state->base.state;
	struct analogix_dp_device *dp = bridge->driver_private;
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;

	crtc = analogix_dp_get_new_crtc(dp, old_state);
	if (!crtc)
		return;

	old_crtc_state = drm_atomic_get_old_crtc_state(old_state, crtc);
	/* Don't touch the panel if we're coming back from PSR */
	if (old_crtc_state && old_crtc_state->self_refresh_active)
		return;

	if (dp->plat_data->panel)
		analogix_dp_panel_prepare(dp);
}

static int analogix_dp_set_bridge(struct analogix_dp_device *dp)
{
	int ret;

	if (dp->plat_data->power_on_start)
		dp->plat_data->power_on_start(dp->plat_data);

	ret = analogix_dp_phy_power_on(dp);
	if (ret)
		return ret;

	ret = analogix_dp_init_dp(dp);
	if (ret)
		goto out_dp_init;

	/*
	 * According to DP spec v1.3 chap 3.5.1.2 Link Training,
	 * We should first make sure the HPD signal is asserted high by device
	 * when we want to establish a link with it.
	 */
	ret = analogix_dp_detect_hpd(dp);
	if (ret) {
		DRM_ERROR("failed to get hpd single ret = %d\n", ret);
		goto out_dp_init;
	}

	ret = analogix_dp_commit(dp);
	if (ret < 0) {
		DRM_ERROR("dp commit error, ret = %d\n", ret);
		goto out_dp_init;
	}

	if (dp->plat_data->panel)
		drm_panel_enable(dp->plat_data->panel);

	if (dp->plat_data->power_on_end)
		dp->plat_data->power_on_end(dp->plat_data);

	return 0;

out_dp_init:
	analogix_dp_phy_power_off(dp);
	if (dp->plat_data->power_off)
		dp->plat_data->power_off(dp->plat_data);
	return ret;
}

static void analogix_dp_modeset_retry_work_fn(struct work_struct *work)
{
	struct analogix_dp_device *dp =
			container_of(work, typeof(*dp), modeset_retry_work);

	/* Send Hotplug uevent so userspace can reprobe */
	drm_kms_helper_hotplug_event(dp->bridge.dev);
}

static void
analogix_dp_bridge_atomic_enable(struct drm_bridge *bridge,
				 struct drm_bridge_state *old_bridge_state)
{
	struct drm_atomic_state *old_state = old_bridge_state->base.state;
	struct analogix_dp_device *dp = bridge->driver_private;
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;
	int timeout_loop = 0;
	int ret;

	crtc = analogix_dp_get_new_crtc(dp, old_state);
	if (!crtc)
		return;

	old_crtc_state = drm_atomic_get_old_crtc_state(old_state, crtc);
	/* Not a full enable, just disable PSR and continue */
	if (old_crtc_state && old_crtc_state->self_refresh_active) {
		ret = analogix_dp_disable_psr(dp);
		if (ret)
			DRM_ERROR("Failed to disable psr %d\n", ret);
		return;
	}

	if (dp->dpms_mode == DRM_MODE_DPMS_ON)
		return;

	while (timeout_loop < MAX_PLL_LOCK_LOOP) {
		if (analogix_dp_set_bridge(dp) == 0) {
			dp->dpms_mode = DRM_MODE_DPMS_ON;
			return;
		}
		dev_err(dp->dev, "failed to set bridge, retry: %d\n",
			timeout_loop);
		timeout_loop++;
		usleep_range(10, 11);
	}
	dev_err(dp->dev, "too many times retry set bridge, give it up\n");

	/* Schedule a Hotplug Uevent to userspace to start modeset */
	schedule_work(&dp->modeset_retry_work);
}

static void analogix_dp_bridge_disable(struct drm_bridge *bridge)
{
	struct analogix_dp_device *dp = bridge->driver_private;

	if (dp->dpms_mode != DRM_MODE_DPMS_ON)
		return;

	if (dp->plat_data->panel) {
		if (drm_panel_disable(dp->plat_data->panel)) {
			DRM_ERROR("failed to disable the panel\n");
			return;
		}
	}

	if (!analogix_dp_get_plug_in_status(dp))
		analogix_dp_link_power_down(dp);

	if (dp->plat_data->power_off)
		dp->plat_data->power_off(dp->plat_data);

	analogix_dp_set_analog_power_down(dp, POWER_ALL, 1);
	analogix_dp_phy_power_off(dp);

	if (dp->plat_data->panel)
		analogix_dp_panel_unprepare(dp);

	dp->fast_train_enable = false;
	dp->psr_supported = false;
	dp->dpms_mode = DRM_MODE_DPMS_OFF;
}

static void
analogix_dp_bridge_atomic_disable(struct drm_bridge *bridge,
				  struct drm_bridge_state *old_bridge_state)
{
	struct drm_atomic_state *old_state = old_bridge_state->base.state;
	struct analogix_dp_device *dp = bridge->driver_private;
	struct drm_crtc *crtc;
	struct drm_crtc_state *new_crtc_state = NULL;

	crtc = analogix_dp_get_new_crtc(dp, old_state);
	if (!crtc)
		goto out;

	new_crtc_state = drm_atomic_get_new_crtc_state(old_state, crtc);
	if (!new_crtc_state)
		goto out;

	/* Don't do a full disable on PSR transitions */
	if (new_crtc_state->self_refresh_active)
		return;

out:
	analogix_dp_bridge_disable(bridge);
}

static void
analogix_dp_bridge_atomic_post_disable(struct drm_bridge *bridge,
				struct drm_bridge_state *old_bridge_state)
{
	struct drm_atomic_state *old_state = old_bridge_state->base.state;
	struct analogix_dp_device *dp = bridge->driver_private;
	struct drm_crtc *crtc;
	struct drm_crtc_state *new_crtc_state;
	int ret;

	crtc = analogix_dp_get_new_crtc(dp, old_state);
	if (!crtc)
		return;

	new_crtc_state = drm_atomic_get_new_crtc_state(old_state, crtc);
	if (!new_crtc_state || !new_crtc_state->self_refresh_active)
		return;

	ret = analogix_dp_enable_psr(dp);
	if (ret)
		DRM_ERROR("Failed to enable psr (%d)\n", ret);
}

static void analogix_dp_bridge_mode_set(struct drm_bridge *bridge,
				const struct drm_display_mode *orig_mode,
				const struct drm_display_mode *adj_mode)
{
	struct analogix_dp_device *dp = bridge->driver_private;
	struct drm_display_info *display_info = &dp->connector.display_info;
	struct video_info *video = &dp->video_info;
	struct drm_display_mode *mode = &video->mode;
	struct device_node *dp_node = dp->dev->of_node;
	int vic;

	drm_mode_copy(mode, adj_mode);
	if (dp->plat_data->split_mode)
		dp->plat_data->convert_to_origin_mode(mode);

	/* Input video interlaces & hsync pol & vsync pol */
	video->interlaced = !!(mode->flags & DRM_MODE_FLAG_INTERLACE);
	video->v_sync_polarity = !!(mode->flags & DRM_MODE_FLAG_NVSYNC);
	video->h_sync_polarity = !!(mode->flags & DRM_MODE_FLAG_NHSYNC);

	/* Input video dynamic_range & colorimetry */
	vic = drm_match_cea_mode(mode);
	if ((vic == 6) || (vic == 7) || (vic == 21) || (vic == 22) ||
	    (vic == 2) || (vic == 3) || (vic == 17) || (vic == 18))
		video->dynamic_range = CEA;
	else if (vic)
		video->dynamic_range = CEA;
	else
		video->dynamic_range = VESA;

	/* Input vide bpc and color_formats */
	switch (display_info->bpc) {
	case 12:
		video->color_depth = COLOR_12;
		break;
	case 10:
		video->color_depth = COLOR_10;
		break;
	case 8:
		video->color_depth = COLOR_8;
		break;
	case 6:
		video->color_depth = COLOR_6;
		break;
	default:
		video->color_depth = COLOR_8;
		break;
	}
	if (display_info->color_formats & DRM_COLOR_FORMAT_YCRCB444) {
		video->color_space = COLOR_YCBCR444;
		video->ycbcr_coeff = COLOR_YCBCR709;
	} else if (display_info->color_formats & DRM_COLOR_FORMAT_YCRCB422) {
		video->color_space = COLOR_YCBCR422;
		video->ycbcr_coeff = COLOR_YCBCR709;
	} else {
		video->color_space = COLOR_RGB;
		video->ycbcr_coeff = COLOR_YCBCR601;
	}

	/*
	 * NOTE: those property parsing code is used for providing backward
	 * compatibility for samsung platform.
	 * Due to we used the "of_property_read_u32" interfaces, when this
	 * property isn't present, the "video_info" can keep the original
	 * values and wouldn't be modified.
	 */
	of_property_read_u32(dp_node, "samsung,color-space",
			     &video->color_space);
	of_property_read_u32(dp_node, "samsung,dynamic-range",
			     &video->dynamic_range);
	of_property_read_u32(dp_node, "samsung,ycbcr-coeff",
			     &video->ycbcr_coeff);
	of_property_read_u32(dp_node, "samsung,color-depth",
			     &video->color_depth);
	if (of_property_read_bool(dp_node, "hsync-active-high"))
		video->h_sync_polarity = true;
	if (of_property_read_bool(dp_node, "vsync-active-high"))
		video->v_sync_polarity = true;
	if (of_property_read_bool(dp_node, "interlaced"))
		video->interlaced = true;
}

static bool analogix_dp_link_config_validate(u8 link_rate, u8 lane_count)
{
	switch (link_rate) {
	case DP_LINK_BW_1_62:
	case DP_LINK_BW_2_7:
	case DP_LINK_BW_5_4:
		break;
	default:
		return false;
	}

	switch (lane_count) {
	case 1:
	case 2:
	case 4:
		break;
	default:
		return false;
	}

	return true;
}

static enum drm_mode_status
analogix_dp_bridge_mode_valid(struct drm_bridge *bridge,
			      const struct drm_display_info *info,
			      const struct drm_display_mode *mode)
{
	struct analogix_dp_device *dp = bridge->driver_private;
	struct drm_display_mode m;
	u32 max_link_rate, max_lane_count;

	drm_mode_copy(&m, mode);

	if (dp->plat_data->split_mode)
		dp->plat_data->convert_to_origin_mode(&m);

	max_link_rate = min_t(u32, dp->video_info.max_link_rate,
			      dp->link_train.link_rate);
	max_lane_count = min_t(u32, dp->video_info.max_lane_count,
			       dp->link_train.lane_count);
	if (analogix_dp_link_config_validate(max_link_rate, max_lane_count) &&
	    !analogix_dp_bandwidth_ok(dp, &m,
				      drm_dp_bw_code_to_link_rate(max_link_rate),
				      max_lane_count))
		return MODE_BAD;

	return MODE_OK;
}

static const struct drm_bridge_funcs analogix_dp_bridge_funcs = {
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.atomic_pre_enable = analogix_dp_bridge_atomic_pre_enable,
	.atomic_enable = analogix_dp_bridge_atomic_enable,
	.atomic_disable = analogix_dp_bridge_atomic_disable,
	.atomic_post_disable = analogix_dp_bridge_atomic_post_disable,
	.mode_set = analogix_dp_bridge_mode_set,
	.attach = analogix_dp_bridge_attach,
	.detach = analogix_dp_bridge_detach,
	.mode_valid = analogix_dp_bridge_mode_valid,
};

static int analogix_dp_bridge_init(struct analogix_dp_device *dp)
{
	struct drm_bridge *bridge = &dp->bridge;
	int ret;

	if (!dp->plat_data->left) {
		ret = drm_bridge_attach(dp->encoder, bridge, NULL, 0);
		if (ret) {
			DRM_ERROR("failed to attach drm bridge\n");
			return ret;
		}
	}

	if (dp->plat_data->right) {
		struct analogix_dp_device *secondary = dp->plat_data->right;
		struct drm_bridge *last_bridge =
			list_last_entry(&bridge->encoder->bridge_chain,
					struct drm_bridge, chain_node);

		ret = drm_bridge_attach(dp->encoder, &secondary->bridge, last_bridge,
					DRM_BRIDGE_ATTACH_NO_CONNECTOR);
		if (ret)
			return ret;
	}

	return 0;
}

static int analogix_dp_dt_parse_pdata(struct analogix_dp_device *dp)
{
	struct device_node *dp_node = dp->dev->of_node;
	struct video_info *video_info = &dp->video_info;
	struct property *prop;
	int ret, len, num_lanes;
	u32 max_link_rate;

	switch (dp->plat_data->dev_type) {
	case RK3288_DP:
	case RK3568_EDP:
		/*
		 * Like Rk3288 DisplayPort TRM indicate that "Main link
		 * containing 4 physical lanes of 2.7/1.62 Gbps/lane".
		 */
		video_info->max_link_rate = 0x0A;
		video_info->max_lane_count = 0x04;
		break;
	case RK3399_EDP:
	case RK3588_EDP:
		video_info->max_link_rate = 0x14;
		video_info->max_lane_count = 0x04;
		break;
	case EXYNOS_DP:
		/*
		 * NOTE: those property parseing code is used for
		 * providing backward compatibility for samsung platform.
		 */
		of_property_read_u32(dp_node, "samsung,link-rate",
				     &video_info->max_link_rate);
		of_property_read_u32(dp_node, "samsung,lane-count",
				     &video_info->max_lane_count);
		break;
	}

	if (!of_property_read_u32(dp_node, "max-link-rate", &max_link_rate)) {
		switch (max_link_rate) {
		case 1620:
		case 2700:
		case 5400:
			break;
		default:
			dev_err(dp->dev, "invalid max-link-rate value: %d\n",
				max_link_rate);
			return -EINVAL;
		}

		max_link_rate *= 100;

		if (max_link_rate < drm_dp_bw_code_to_link_rate(video_info->max_link_rate))
			video_info->max_link_rate =
				drm_dp_link_rate_to_bw_code(max_link_rate);
	}

	video_info->video_bist_enable =
		of_property_read_bool(dp_node, "analogix,video-bist-enable");

	prop = of_find_property(dp_node, "data-lanes", &len);
	if (!prop) {
		video_info->lane_map[0] = 0;
		video_info->lane_map[1] = 1;
		video_info->lane_map[2] = 2;
		video_info->lane_map[3] = 3;
		DRM_DEV_DEBUG(dp->dev, "failed to find data lane mapping, using default\n");
		return 0;
	}

	num_lanes = len / sizeof(u32);

	if (num_lanes < 1 || num_lanes > 4 || num_lanes == 3) {
		DRM_DEV_ERROR(dp->dev, "bad number of data lanes\n");
		return -EINVAL;
	}

	video_info->max_lane_count = num_lanes;

	ret = of_property_read_u32_array(dp_node, "data-lanes",
					 video_info->lane_map, num_lanes);
	if (ret) {
		DRM_DEV_ERROR(dp->dev, "failed to read lane data\n");
		return ret;
	}

	return 0;
}

static ssize_t analogix_dpaux_transfer(struct drm_dp_aux *aux,
				       struct drm_dp_aux_msg *msg)
{
	struct analogix_dp_device *dp = to_dp(aux);

	return analogix_dp_transfer(dp, msg);
}

int analogix_dp_audio_hw_params(struct analogix_dp_device *dp,
				struct hdmi_codec_daifmt *daifmt,
				struct hdmi_codec_params *params)
{
	switch (daifmt->fmt) {
	case HDMI_SPDIF:
		analogix_dp_audio_config_spdif(dp);
		break;
	case HDMI_I2S:
		analogix_dp_audio_config_i2s(dp);
		break;
	default:
		DRM_DEV_ERROR(dp->dev, "invalid daifmt %d\n", daifmt->fmt);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(analogix_dp_audio_hw_params);

void analogix_dp_audio_shutdown(struct analogix_dp_device *dp)
{
	analogix_dp_audio_disable(dp);
}
EXPORT_SYMBOL_GPL(analogix_dp_audio_shutdown);

int analogix_dp_audio_startup(struct analogix_dp_device *dp)
{
	analogix_dp_audio_enable(dp);

	return 0;
}
EXPORT_SYMBOL_GPL(analogix_dp_audio_startup);

int analogix_dp_audio_get_eld(struct analogix_dp_device *dp, u8 *buf, size_t len)
{
	memcpy(buf, dp->connector.eld, min(sizeof(dp->connector.eld), len));

	return 0;
}
EXPORT_SYMBOL_GPL(analogix_dp_audio_get_eld);

static void analogix_dp_link_train_restore(struct analogix_dp_device *dp)
{
	u32 link_rate, lane_count;
	u8 lane, spread;

	analogix_dp_get_link_bandwidth(dp, &link_rate);
	analogix_dp_get_lane_count(dp, &lane_count);
	drm_dp_dpcd_readb(&dp->aux, DP_MAX_DOWNSPREAD, &spread);

	dp->link_train.link_rate = link_rate;
	dp->link_train.lane_count = lane_count;
	dp->link_train.enhanced_framing = analogix_dp_get_enhanced_mode(dp);
	dp->link_train.ssc = !!(spread & DP_MAX_DOWNSPREAD_0_5);

	for (lane = 0; lane < 4; lane++)
		dp->link_train.training_lane[lane] =
				analogix_dp_get_lane_link_training(dp, lane);
}

int analogix_dp_loader_protect(struct analogix_dp_device *dp)
{
	int ret;

	ret = analogix_dp_phy_power_on(dp);
	if (ret)
		return ret;

	dp->dpms_mode = DRM_MODE_DPMS_ON;

	analogix_dp_link_train_restore(dp);

	ret = analogix_dp_fast_link_train_detection(dp);
	if (ret)
		return ret;

	if (analogix_dp_detect_sink_psr(dp)) {
		ret = analogix_dp_enable_sink_psr(dp);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(analogix_dp_loader_protect);

struct analogix_dp_device *
analogix_dp_probe(struct device *dev, struct analogix_dp_plat_data *plat_data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct analogix_dp_device *dp;
	struct resource *res;
	int ret;

	if (!plat_data) {
		dev_err(dev, "Invalided input plat_data\n");
		return ERR_PTR(-EINVAL);
	}

	dp = devm_kzalloc(dev, sizeof(struct analogix_dp_device), GFP_KERNEL);
	if (!dp)
		return ERR_PTR(-ENOMEM);

	dp->dev = &pdev->dev;
	dp->dpms_mode = DRM_MODE_DPMS_OFF;
	INIT_WORK(&dp->modeset_retry_work, analogix_dp_modeset_retry_work_fn);

	mutex_init(&dp->panel_lock);
	dp->panel_is_prepared = false;

	/*
	 * platform dp driver need containor_of the plat_data to get
	 * the driver private data, so we need to store the point of
	 * plat_data, not the context of plat_data.
	 */
	dp->plat_data = plat_data;

	ret = analogix_dp_dt_parse_pdata(dp);
	if (ret)
		return ERR_PTR(ret);

	dp->phy = devm_phy_get(dp->dev, "dp");
	if (IS_ERR(dp->phy)) {
		dev_err(dp->dev, "no DP phy configured\n");
		ret = PTR_ERR(dp->phy);
		if (ret) {
			/*
			 * phy itself is not enabled, so we can move forward
			 * assigning NULL to phy pointer.
			 */
			if (ret == -ENOSYS || ret == -ENODEV)
				dp->phy = NULL;
			else
				return ERR_PTR(ret);
		}
	}

	ret = devm_clk_bulk_get_all(dev, &dp->clks);
	if (ret < 0) {
		dev_err(dev, "failed to get clocks %d\n", ret);
		return ERR_PTR(ret);
	}

	dp->nr_clks = ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	dp->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dp->reg_base))
		return ERR_CAST(dp->reg_base);

	dp->force_hpd = of_property_read_bool(dev->of_node, "force-hpd");

	/* Try two different names */
	dp->hpd_gpiod = devm_gpiod_get_optional(dev, "hpd", GPIOD_IN);
	if (!dp->hpd_gpiod)
		dp->hpd_gpiod = devm_gpiod_get_optional(dev, "samsung,hpd",
							GPIOD_IN);
	if (IS_ERR(dp->hpd_gpiod)) {
		dev_err(dev, "error getting HDP GPIO: %ld\n",
			PTR_ERR(dp->hpd_gpiod));
		return ERR_CAST(dp->hpd_gpiod);
	}

	if (dp->hpd_gpiod) {
		ret = devm_request_threaded_irq(dev,
						gpiod_to_irq(dp->hpd_gpiod),
						NULL,
						analogix_dp_hpd_irq_handler,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						"analogix-hpd", dp);
		if (ret) {
			dev_err(dev, "failed to request hpd IRQ: %d\n", ret);
			return ERR_PTR(ret);
		}
	}

	dp->irq = platform_get_irq(pdev, 0);
	if (dp->irq == -ENXIO) {
		dev_err(&pdev->dev, "failed to get irq\n");
		return ERR_PTR(-ENODEV);
	}

	irq_set_status_flags(dp->irq, IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, dp->irq, NULL,
					analogix_dp_irq_thread,
					IRQF_ONESHOT, dev_name(dev), dp);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq\n");
		return ERR_PTR(ret);
	}

	dp->extcon = devm_extcon_dev_allocate(dev, analogix_dp_cable);
	if (IS_ERR(dp->extcon)) {
		dev_err(dev, "failed to allocate extcon device\n");
		return ERR_CAST(dp->extcon);
	}

	ret = devm_extcon_dev_register(dev, dp->extcon);
	if (ret) {
		dev_err(dev, "failed to register extcon device\n");
		return ERR_PTR(ret);
	}

	dp->bridge.driver_private = dp;
	dp->bridge.funcs = &analogix_dp_bridge_funcs;

	return dp;
}
EXPORT_SYMBOL_GPL(analogix_dp_probe);

int analogix_dp_bind(struct analogix_dp_device *dp, struct drm_device *drm_dev)
{
	int ret;

	dp->drm_dev = drm_dev;
	dp->encoder = dp->plat_data->encoder;

	dp->aux.name = "DP-AUX";
	dp->aux.transfer = analogix_dpaux_transfer;
	dp->aux.dev = dp->dev;

	ret = drm_dp_aux_register(&dp->aux);
	if (ret)
		return ret;

	pm_runtime_enable(dp->dev);
	pm_runtime_get_sync(dp->dev);
	analogix_dp_init(dp);

	ret = analogix_dp_bridge_init(dp);
	if (ret) {
		DRM_ERROR("failed to init bridge (%d)\n", ret);
		goto err_disable_pm_runtime;
	}

	enable_irq(dp->irq);

	return 0;

err_disable_pm_runtime:
	pm_runtime_put(dp->dev);
	pm_runtime_disable(dp->dev);
	drm_dp_aux_unregister(&dp->aux);

	return ret;
}
EXPORT_SYMBOL_GPL(analogix_dp_bind);

void analogix_dp_unbind(struct analogix_dp_device *dp)
{
	disable_irq(dp->irq);
	if (dp->connector.funcs->destroy)
		dp->connector.funcs->destroy(&dp->connector);
	drm_dp_aux_unregister(&dp->aux);
	pm_runtime_put(dp->dev);
	pm_runtime_disable(dp->dev);
}
EXPORT_SYMBOL_GPL(analogix_dp_unbind);

void analogix_dp_remove(struct analogix_dp_device *dp)
{
	cancel_work_sync(&dp->modeset_retry_work);
}
EXPORT_SYMBOL_GPL(analogix_dp_remove);

int analogix_dp_suspend(struct analogix_dp_device *dp)
{
	pm_runtime_force_suspend(dp->dev);

	return 0;
}
EXPORT_SYMBOL_GPL(analogix_dp_suspend);

int analogix_dp_resume(struct analogix_dp_device *dp)
{
	pm_runtime_force_resume(dp->dev);
	analogix_dp_init(dp);

	return 0;
}
EXPORT_SYMBOL_GPL(analogix_dp_resume);

int analogix_dp_runtime_suspend(struct analogix_dp_device *dp)
{
	clk_bulk_disable_unprepare(dp->nr_clks, dp->clks);

	return 0;
}
EXPORT_SYMBOL_GPL(analogix_dp_runtime_suspend);

int analogix_dp_runtime_resume(struct analogix_dp_device *dp)
{
	return clk_bulk_prepare_enable(dp->nr_clks, dp->clks);
}
EXPORT_SYMBOL_GPL(analogix_dp_runtime_resume);

int analogix_dp_start_crc(struct drm_connector *connector)
{
	struct analogix_dp_device *dp = to_dp(connector);

	if (!connector->state->crtc) {
		DRM_ERROR("Connector %s doesn't currently have a CRTC.\n",
			  connector->name);
		return -EINVAL;
	}

	return drm_dp_start_crc(&dp->aux, connector->state->crtc);
}
EXPORT_SYMBOL_GPL(analogix_dp_start_crc);

int analogix_dp_stop_crc(struct drm_connector *connector)
{
	struct analogix_dp_device *dp = to_dp(connector);

	return drm_dp_stop_crc(&dp->aux);
}
EXPORT_SYMBOL_GPL(analogix_dp_stop_crc);

MODULE_AUTHOR("Jingoo Han <jg1.han@samsung.com>");
MODULE_DESCRIPTION("Analogix DP Core Driver");
MODULE_LICENSE("GPL v2");
