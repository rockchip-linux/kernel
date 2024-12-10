// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Rockchip Electronics Co. Ltd.
 *
 * Author: Dingxian Wen <shawn.wen@rock-chips.com>
 */

#include <linux/clk.h>
#include <linux/compat.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/rk-camera-module.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <linux/rk_hdmirx_class.h>
#include <media/v4l2-controls_rockchip.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <video/videomode.h>

#include "rk628.h"
#include "rk628_combrxphy.h"
#include "rk628_combtxphy.h"
#include "rk628_csi.h"
#include "rk628_cru.h"
#include "rk628_dsi.h"
#include "rk628_hdmirx.h"
#include "rk628_mipi_dphy.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x0, 0x8)
#define RK628_CSI_NAME			"rk628-csi"

#define EDID_NUM_BLOCKS_MAX		2
#define EDID_BLOCK_SIZE			128

#define RK628_CSI_LINK_FREQ_LOW		350000000
#define RK628_CSI_LINK_FREQ_HIGH	600000000
#define RK628_CSI_PIXEL_RATE_LOW	400000000
#define RK628_CSI_PIXEL_RATE_HIGH	600000000
#define MIPI_DATARATE_MBPS_LOW		700
#define MIPI_DATARATE_MBPS_HIGH		1250

#define POLL_INTERVAL_MS		1000
#define MODETCLK_CNT_NUM		1000
#define MODETCLK_HZ			49500000
#define RXPHY_CFG_MAX_TIMES		15
#define CSITX_ERR_RETRY_TIMES		3

#define YUV422_8BIT			0x1e

enum tx_mode_type {
	CSI_MODE,
	DSI_MODE,
};

struct rk628_plat_data {
	int bus_fmt;
	int tx_mode;
};

struct rk628_csi {
	struct device *dev;
	struct i2c_client *i2c_client;
	struct rk628 *rk628;
	struct media_pad pad;
	struct v4l2_subdev sd;
	struct v4l2_dv_timings src_timings;
	struct v4l2_dv_timings timings;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *detect_tx_5v_ctrl;
	struct v4l2_ctrl *audio_sampling_rate_ctrl;
	struct v4l2_ctrl *audio_present_ctrl;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *power_gpio;
	struct gpio_desc *plugin_det_gpio;
	struct clk *soc_24M;
	struct clk *clk_hdmirx_aud;
	struct clk *clk_vop;
	struct clk *clk_rx_read;
	struct delayed_work delayed_work_enable_hotplug;
	struct delayed_work delayed_work_res_change;
	struct timer_list timer;
	struct work_struct work_i2c_poll;
	struct mutex confctl_mutex;
	const struct rk628_csi_mode *cur_mode;
	const char *module_facing;
	const char *module_name;
	const char *len_name;
	u32 module_index;
	u8 edid_blocks_written;
	u64 lane_mbps;
	u8 csi_lanes_in_use;
	u32 mbus_fmt_code;
	u8 fps;
	u32 stream_state;
	int hdmirx_irq;
	int plugin_irq;
	bool nosignal;
	bool rxphy_pwron;
	bool txphy_pwron;
	bool enable_hdcp;
	bool scaler_en;
	bool hpd_output_inverted;
	bool avi_rcv_rdy;
	bool vid_ints_en;
	bool continues_clk;
	struct rk628_hdcp hdcp;
	bool i2s_enable_default;
	HAUDINFO audio_info;
	struct rk628_combtxphy *txphy;
	struct rk628_dsi dsi;
	const struct rk628_plat_data *plat_data;
	struct device *classdev;
};

struct rk628_csi_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
};

static const s64 link_freq_menu_items[] = {
	RK628_CSI_LINK_FREQ_LOW,
	RK628_CSI_LINK_FREQ_HIGH,
};

static const struct v4l2_dv_timings_cap rk628_csi_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	/* keep this initialization for compatibility with GCC < 4.4.6 */
	.reserved = { 0 },
	V4L2_INIT_BT_TIMINGS(1, 10000, 1, 10000, 0, 600000000,
			V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			V4L2_DV_BT_STD_GTF | V4L2_DV_BT_STD_CVT,
			V4L2_DV_BT_CAP_PROGRESSIVE | V4L2_DV_BT_CAP_INTERLACED |
			V4L2_DV_BT_CAP_REDUCED_BLANKING |
			V4L2_DV_BT_CAP_CUSTOM)
};

static u8 edid_init_data[] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	0x49, 0x73, 0x8D, 0x62, 0x00, 0x88, 0x88, 0x88,
	0x08, 0x1E, 0x01, 0x03, 0x80, 0x00, 0x00, 0x78,
	0x0A, 0x0D, 0xC9, 0xA0, 0x57, 0x47, 0x98, 0x27,
	0x12, 0x48, 0x4C, 0x00, 0x00, 0x00, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3A,
	0x80, 0x18, 0x71, 0x38, 0x2D, 0x40, 0x58, 0x2C,
	0x45, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x1E,
	0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0, 0x1E, 0x20,
	0x6E, 0x28, 0x55, 0x00, 0xC4, 0x8E, 0x21, 0x00,
	0x00, 0x1E, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x54,
	0x37, 0x34, 0x39, 0x2D, 0x66, 0x48, 0x44, 0x37,
	0x32, 0x30, 0x0A, 0x20, 0x00, 0x00, 0x00, 0xFD,
	0x00, 0x14, 0x78, 0x01, 0xFF, 0x1D, 0x00, 0x0A,
	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x18,

	0x02, 0x03, 0x1A, 0x71, 0x47, 0x5F, 0x90, 0x22,
	0x04, 0x11, 0x02, 0x01, 0x23, 0x09, 0x07, 0x01,
	0x83, 0x01, 0x00, 0x00, 0x65, 0x03, 0x0C, 0x00,
	0x10, 0x00, 0x02, 0x3A, 0x80, 0x18, 0x71, 0x38,
	0x2D, 0x40, 0x58, 0x2C, 0x45, 0x00, 0x20, 0xC2,
	0x31, 0x00, 0x00, 0x1E, 0x01, 0x1D, 0x00, 0x72,
	0x51, 0xD0, 0x1E, 0x20, 0x6E, 0x28, 0x55, 0x00,
	0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E, 0x02, 0x3A,
	0x80, 0xD0, 0x72, 0x38, 0x2D, 0x40, 0x10, 0x2C,
	0x45, 0x80, 0x20, 0xC2, 0x31, 0x00, 0x00, 0x1E,
	0x01, 0x1D, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40,
	0x58, 0x2C, 0x45, 0x00, 0xC0, 0x6C, 0x00, 0x00,
	0x00, 0x18, 0x01, 0x1D, 0x80, 0x18, 0x71, 0x1C,
	0x16, 0x20, 0x58, 0x2C, 0x25, 0x00, 0xC0, 0x6C,
	0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC1,
};

static struct rkmodule_csi_dphy_param rk3588_dcphy_param = {
	.vendor = PHY_VENDOR_SAMSUNG,
	.lp_vol_ref = 0,
	.lp_hys_sw = {3, 0, 0, 0},
	.lp_escclk_pol_sel = {1, 0, 0, 0},
	.skew_data_cal_clk = {0, 3, 3, 3},
	.clk_hs_term_sel = 2,
	.data_hs_term_sel = {2, 2, 2, 2},
	.reserved = {0},
};

static const struct rk628_csi_mode supported_modes[] = {
	{
		.width = 3840,
		.height = 2160,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.hts_def = 4400,
		.vts_def = 2250,
	}, {
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 2200,
		.vts_def = 1125,
	}, {
		.width = 1280,
		.height = 720,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 1650,
		.vts_def = 750,
	}, {
		.width = 720,
		.height = 576,
		.max_fps = {
			.numerator = 10000,
			.denominator = 500000,
		},
		.hts_def = 864,
		.vts_def = 625,
	}, {
		.width = 720,
		.height = 480,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.hts_def = 858,
		.vts_def = 525,
	},
};

static struct v4l2_dv_timings dst_timing = {
	.type = V4L2_DV_BT_656_1120,
	.bt = {
		.interlaced = V4L2_DV_PROGRESSIVE,
		.width = 1920,
		.height = 1080,
		.hfrontporch = 88,
		.hsync = 44,
		.hbackporch = 148,
		.vfrontporch = 4,
		.vsync = 5,
		.vbackporch = 36,
		.pixelclock = 148500000,
	},
};

static void rk628_post_process_setup(struct v4l2_subdev *sd);
static void rk628_csi_enable_interrupts(struct v4l2_subdev *sd, bool en);
static int rk628_csi_s_ctrl_detect_tx_5v(struct v4l2_subdev *sd);
static int rk628_csi_s_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings);
static int rk628_csi_s_edid(struct v4l2_subdev *sd,
				struct v4l2_subdev_edid *edid);
static int mipi_dphy_power_on(struct rk628_csi *csi);
static void mipi_dphy_power_off(struct rk628_csi *csi);
static int rk628_hdmirx_phy_power_on(struct v4l2_subdev *sd);
static int rk628_hdmirx_phy_power_off(struct v4l2_subdev *sd);
static int rk628_hdmirx_phy_setup(struct v4l2_subdev *sd);
static void rk628_csi_format_change(struct v4l2_subdev *sd);
static void enable_stream(struct v4l2_subdev *sd, bool enable);
static void rk628_hdmirx_vid_enable(struct v4l2_subdev *sd, bool en);
static void rk628_csi_set_csi(struct v4l2_subdev *sd);
static void rk628_hdmirx_hpd_ctrl(struct v4l2_subdev *sd, bool en);
static void rk628_hdmirx_controller_reset(struct v4l2_subdev *sd);
static bool rk628_rcv_supported_res(struct v4l2_subdev *sd, u32 width,
				    u32 height);
static void rk628_dsi_set_scs(struct rk628_csi *csi);
static void rk628_dsi_enable(struct v4l2_subdev *sd);

static inline struct rk628_csi *to_csi(struct v4l2_subdev *sd)
{
	return container_of(sd, struct rk628_csi, sd);
}

static bool tx_5v_power_present(struct v4l2_subdev *sd)
{
	bool ret;
	int val, i, cnt;
	struct rk628_csi *csi = to_csi(sd);

	/* Direct Mode */
	if (!csi->plugin_det_gpio)
		return true;

	cnt = 0;
	for (i = 0; i < 5; i++) {
		val = gpiod_get_value(csi->plugin_det_gpio);
		if (val > 0)
			cnt++;
		usleep_range(500, 600);
	}

	ret = (cnt >= 3) ? true : false;
	v4l2_dbg(1, debug, sd, "%s: %d\n", __func__, ret);

	return ret;
}

static inline bool no_signal(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);

	v4l2_dbg(1, debug, sd, "%s no signal:%d\n", __func__, csi->nosignal);
	return csi->nosignal;
}

static inline bool audio_present(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);

	return rk628_hdmirx_audio_present(csi->audio_info);
}

static int get_audio_sampling_rate(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);

	if (no_signal(sd))
		return 0;

	return rk628_hdmirx_audio_fs(csi->audio_info);
}

static void rk628_hdmirx_ctrl_enable(struct v4l2_subdev *sd, int en)
{
	u32 mask;
	struct rk628_csi *csi = to_csi(sd);

	if (en) {
		/* don't enable audio until N CTS updated */
		mask = HDMI_ENABLE_MASK;
		v4l2_dbg(1, debug, sd, "%s: %#x %d\n", __func__, mask, en);
		rk628_i2c_update_bits(csi->rk628, HDMI_RX_DMI_DISABLE_IF,
				   mask, HDMI_ENABLE(1) | AUD_ENABLE(1));
	} else {
		mask = AUD_ENABLE_MASK | HDMI_ENABLE_MASK;
		v4l2_dbg(1, debug, sd, "%s: %#x %d\n", __func__, mask, en);
		rk628_i2c_update_bits(csi->rk628, HDMI_RX_DMI_DISABLE_IF,
				   mask, HDMI_ENABLE(0) | AUD_ENABLE(0));
	}
}

static int rk628_csi_get_detected_timings(struct v4l2_subdev *sd,
				     struct v4l2_dv_timings *timings)
{
	struct rk628_csi *csi = to_csi(sd);
	struct v4l2_bt_timings *bt = &timings->bt;
	u32 hact, vact, htotal, vtotal, fps, status;
	u32 val;
	u32 modetclk_cnt_hs, modetclk_cnt_vs, hs, vs;
	u32 hofs_pix, hbp, hfp, vbp, vfp;
	u32 tmds_clk, tmdsclk_cnt;
	u64 tmp_data;
	int retry = 0;

__retry:
	memset(timings, 0, sizeof(struct v4l2_dv_timings));
	timings->type = V4L2_DV_BT_656_1120;
	rk628_i2c_read(csi->rk628, HDMI_RX_SCDC_REGS1, &val);
	status = val;

	rk628_i2c_read(csi->rk628, HDMI_RX_MD_STS, &val);
	bt->interlaced = val & ILACE_STS ?
		V4L2_DV_INTERLACED : V4L2_DV_PROGRESSIVE;

	rk628_i2c_read(csi->rk628, HDMI_RX_MD_HACT_PX, &val);
	hact = val & 0xffff;
	rk628_i2c_read(csi->rk628, HDMI_RX_MD_VAL, &val);
	vact = val & 0xffff;
	rk628_i2c_read(csi->rk628, HDMI_RX_MD_HT1, &val);
	htotal = (val >> 16) & 0xffff;
	rk628_i2c_read(csi->rk628, HDMI_RX_MD_VTL, &val);
	vtotal = val & 0xffff;
	rk628_i2c_read(csi->rk628, HDMI_RX_MD_HT1, &val);
	hofs_pix = val & 0xffff;
	rk628_i2c_read(csi->rk628, HDMI_RX_MD_VOL, &val);
	vbp = (val & 0xffff) + 1;

	rk628_i2c_read(csi->rk628, HDMI_RX_HDMI_CKM_RESULT, &val);
	tmdsclk_cnt = val & 0xffff;
	tmp_data = tmdsclk_cnt;
	tmp_data = ((tmp_data * MODETCLK_HZ) + MODETCLK_CNT_NUM / 2);
	do_div(tmp_data, MODETCLK_CNT_NUM);
	tmds_clk = tmp_data;
	if (!htotal || !vtotal) {
		v4l2_err(&csi->sd, "timing err, htotal:%d, vtotal:%d\n",
				htotal, vtotal);
		if (retry++ < 5)
			goto __retry;

		goto TIMING_ERR;
	}
	fps = (tmds_clk + (htotal * vtotal) / 2) / (htotal * vtotal);

	rk628_i2c_read(csi->rk628, HDMI_RX_MD_HT0, &val);
	modetclk_cnt_hs = val & 0xffff;
	hs = (tmdsclk_cnt * modetclk_cnt_hs + MODETCLK_CNT_NUM / 2) /
		MODETCLK_CNT_NUM;

	rk628_i2c_read(csi->rk628, HDMI_RX_MD_VSC, &val);
	modetclk_cnt_vs = val & 0xffff;
	vs = (tmdsclk_cnt * modetclk_cnt_vs + MODETCLK_CNT_NUM / 2) /
		MODETCLK_CNT_NUM;
	vs = (vs + htotal / 2) / htotal;

	if ((hofs_pix < hs) || (htotal < (hact + hofs_pix)) ||
			(vtotal < (vact + vs + vbp))) {
		v4l2_err(sd, "timing err, total:%dx%d, act:%dx%d, hofs:%d, hs:%d, vs:%d, vbp:%d\n",
			 htotal, vtotal, hact, vact, hofs_pix, hs, vs, vbp);
		goto TIMING_ERR;
	}
	hbp = hofs_pix - hs;
	hfp = htotal - hact - hofs_pix;
	vfp = vtotal - vact - vs - vbp;

	v4l2_dbg(2, debug, sd, "cnt_num:%d, tmds_cnt:%d, hs_cnt:%d, vs_cnt:%d, hofs:%d\n",
		 MODETCLK_CNT_NUM, tmdsclk_cnt, modetclk_cnt_hs, modetclk_cnt_vs, hofs_pix);

	bt->width = hact;
	bt->height = vact;
	bt->hfrontporch = hfp;
	bt->hsync = hs;
	bt->hbackporch = hbp;
	bt->vfrontporch = vfp;
	bt->vsync = vs;
	bt->vbackporch = vbp;
	bt->pixelclock = htotal * vtotal * fps;

	if (bt->interlaced == V4L2_DV_INTERLACED) {
		bt->height *= 2;
		bt->il_vsync = bt->vsync + 1;
		bt->pixelclock /= 2;
	}

	if (vact == 1080 && vtotal > 1500)
		goto __retry;

	v4l2_dbg(1, debug, sd, "SCDC_REGS1:%#x, act:%dx%d, total:%dx%d, fps:%d, pixclk:%llu\n",
		 status, hact, vact, htotal, vtotal, fps, bt->pixelclock);
	v4l2_dbg(1, debug, sd, "hfp:%d, hs:%d, hbp:%d, vfp:%d, vs:%d, vbp:%d, interlace:%d\n",
		 bt->hfrontporch, bt->hsync, bt->hbackporch, bt->vfrontporch, bt->vsync,
		 bt->vbackporch, bt->interlaced);

	csi->src_timings = *timings;
	if (csi->scaler_en)
		*timings = csi->timings;

	return 0;

TIMING_ERR:
	return -ENOLCK;
}

static void rk628_hdmirx_config_all(struct v4l2_subdev *sd)
{
	int ret;
	struct rk628_csi *csi = to_csi(sd);

	rk628_hdmirx_controller_setup(csi->rk628);
	ret = rk628_hdmirx_phy_setup(sd);
	if (ret >= 0) {
		rk628_csi_format_change(sd);
		csi->nosignal = false;
	}
}

static void rk628_csi_delayed_work_enable_hotplug(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct rk628_csi *csi = container_of(dwork, struct rk628_csi,
			delayed_work_enable_hotplug);
	struct v4l2_subdev *sd = &csi->sd;
	bool plugin;

	mutex_lock(&csi->confctl_mutex);
	csi->avi_rcv_rdy = false;
	plugin = tx_5v_power_present(sd);
	v4l2_ctrl_s_ctrl(csi->detect_tx_5v_ctrl, plugin);
	v4l2_dbg(1, debug, sd, "%s: 5v_det:%d\n", __func__, plugin);
	if (plugin) {
		rk628_csi_enable_interrupts(sd, false);
		rk628_hdmirx_audio_setup(csi->audio_info);
		rk628_hdmirx_set_hdcp(csi->rk628, &csi->hdcp, csi->enable_hdcp);
		rk628_hdmirx_hpd_ctrl(sd, true);
		rk628_hdmirx_config_all(sd);
		rk628_csi_enable_interrupts(sd, true);
		rk628_i2c_update_bits(csi->rk628, GRF_SYSTEM_CON0,
				SW_I2S_DATA_OEN_MASK, SW_I2S_DATA_OEN(0));
	} else {
		rk628_csi_enable_interrupts(sd, false);
		enable_stream(sd, false);
		cancel_delayed_work(&csi->delayed_work_res_change);
		rk628_hdmirx_audio_cancel_work_audio(csi->audio_info, true);
		rk628_hdmirx_hpd_ctrl(sd, false);
		rk628_hdmirx_phy_power_off(sd);
		rk628_hdmirx_controller_reset(sd);
		csi->nosignal = true;
	}
	mutex_unlock(&csi->confctl_mutex);
	if (csi->plat_data->tx_mode == DSI_MODE && plugin)
		enable_stream(sd, true);
}

static int rk628_check_resulotion_change(struct v4l2_subdev *sd)
{
	u32 val;
	struct rk628_csi *csi = to_csi(sd);
	u32 htotal, vtotal;
	u32 old_htotal, old_vtotal;
	struct v4l2_bt_timings *bt = &csi->src_timings.bt;

	rk628_i2c_read(csi->rk628, HDMI_RX_MD_HT1, &val);
	htotal = (val >> 16) & 0xffff;
	rk628_i2c_read(csi->rk628, HDMI_RX_MD_VTL, &val);
	vtotal = val & 0xffff;

	old_htotal = bt->hfrontporch + bt->hsync + bt->width + bt->hbackporch;
	old_vtotal = bt->vfrontporch + bt->vsync + bt->height + bt->vbackporch;

	v4l2_dbg(1, debug, sd, "new mode: %d x %d\n", htotal, vtotal);
	v4l2_dbg(1, debug, sd, "old mode: %d x %d\n", old_htotal, old_vtotal);

	if (htotal != old_htotal || vtotal != old_vtotal)
		return 1;

	return 0;
}

static void rk628_delayed_work_res_change(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct rk628_csi *csi = container_of(dwork, struct rk628_csi,
			delayed_work_res_change);
	struct v4l2_subdev *sd = &csi->sd;
	bool plugin;

	mutex_lock(&csi->confctl_mutex);
	csi->avi_rcv_rdy = false;
	plugin = tx_5v_power_present(sd);
	v4l2_dbg(1, debug, sd, "%s: 5v_det:%d\n", __func__, plugin);
	if (plugin) {
		if (rk628_check_resulotion_change(sd)) {
			v4l2_dbg(1, debug, sd, "res change, recfg ctrler and phy!\n");
			rk628_hdmirx_audio_cancel_work_audio(csi->audio_info, true);
			rk628_hdmirx_phy_power_off(sd);
			rk628_hdmirx_controller_reset(sd);
			rk628_hdmirx_audio_setup(csi->audio_info);
			rk628_hdmirx_set_hdcp(csi->rk628, &csi->hdcp, csi->enable_hdcp);
			rk628_hdmirx_hpd_ctrl(sd, true);
			rk628_hdmirx_config_all(sd);
			rk628_csi_enable_interrupts(sd, true);
			rk628_i2c_update_bits(csi->rk628, GRF_SYSTEM_CON0,
					      SW_I2S_DATA_OEN_MASK,
					      SW_I2S_DATA_OEN(0));
		} else {
			rk628_csi_format_change(sd);
			csi->nosignal = false;
			rk628_csi_enable_interrupts(sd, true);
		}
	}
	mutex_unlock(&csi->confctl_mutex);
	if (csi->plat_data->tx_mode == DSI_MODE && plugin)
		rk628_dsi_enable(sd);
}

static void rk628_hdmirx_hpd_ctrl(struct v4l2_subdev *sd, bool en)
{
	u8 en_level, set_level;
	struct rk628_csi *csi = to_csi(sd);

	v4l2_dbg(1, debug, sd, "%s: %sable, hpd invert:%d\n", __func__,
			en ? "en" : "dis", csi->hpd_output_inverted);
	en_level = csi->hpd_output_inverted ? 0 : 1;
	set_level = en ? en_level : !en_level;
	rk628_i2c_update_bits(csi->rk628, HDMI_RX_HDMI_SETUP_CTRL,
			HOT_PLUG_DETECT_MASK, HOT_PLUG_DETECT(set_level));
}


static int rk628_csi_s_ctrl_detect_tx_5v(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);

	return v4l2_ctrl_s_ctrl(csi->detect_tx_5v_ctrl,
			tx_5v_power_present(sd));
}

static int rk628_csi_s_ctrl_audio_sampling_rate(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);

	return v4l2_ctrl_s_ctrl(csi->audio_sampling_rate_ctrl,
			get_audio_sampling_rate(sd));
}

static int rk628_csi_s_ctrl_audio_present(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);

	return v4l2_ctrl_s_ctrl(csi->audio_present_ctrl,
			audio_present(sd));
}

static int rk628_csi_update_controls(struct v4l2_subdev *sd)
{
	int ret = 0;

	ret |= rk628_csi_s_ctrl_detect_tx_5v(sd);
	ret |= rk628_csi_s_ctrl_audio_sampling_rate(sd);
	ret |= rk628_csi_s_ctrl_audio_present(sd);

	return ret;
}

static void rk62_csi_reset(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);

	rk628_control_assert(csi->rk628, RGU_CSI);
	udelay(10);
	rk628_control_deassert(csi->rk628, RGU_CSI);

	rk628_i2c_write(csi->rk628, CSITX_SYS_CTRL0_IMD, 0x1);
	usleep_range(1000, 1100);
	rk628_i2c_write(csi->rk628, CSITX_SYS_CTRL0_IMD, 0x0);
}

static void enable_csitx(struct v4l2_subdev *sd)
{
	u32 i, ret, val;
	struct rk628_csi *csi = to_csi(sd);

	for (i = 0; i < CSITX_ERR_RETRY_TIMES; i++) {
		rk628_csi_set_csi(sd);
		rk628_i2c_update_bits(csi->rk628, CSITX_CSITX_EN,
					DPHY_EN_MASK |
					CSITX_EN_MASK,
					DPHY_EN(1) |
					CSITX_EN(1));
		rk628_i2c_write(csi->rk628, CSITX_CONFIG_DONE, CONFIG_DONE_IMD);
		msleep(40);
		rk628_i2c_write(csi->rk628, CSITX_ERR_INTR_CLR_IMD, 0xffffffff);
		rk628_i2c_update_bits(csi->rk628, CSITX_SYS_CTRL1,
				BYPASS_SELECT_MASK, BYPASS_SELECT(0));
		rk628_i2c_write(csi->rk628, CSITX_CONFIG_DONE, CONFIG_DONE_IMD);
		msleep(40);
		ret = rk628_i2c_read(csi->rk628, CSITX_ERR_INTR_RAW_STATUS_IMD, &val);
		if (!ret && !val)
			break;

		v4l2_err(sd, "%s csitx err, retry:%d, err status:%#x, ret:%d\n",
				__func__, i, val, ret);
	}
}

static void rk628_dsi_set_scs(struct rk628_csi *csi)
{
	u8 video_fmt;
	u32 val;
	int avi_rdy;

	mutex_lock(&csi->confctl_mutex);
	avi_rdy = rk628_is_avi_ready(csi->rk628, csi->avi_rcv_rdy);
	mutex_unlock(&csi->confctl_mutex);

	rk628_i2c_read(csi->rk628, HDMI_RX_PDEC_AVI_PB, &val);
	video_fmt = (val & VIDEO_FORMAT_MASK) >> 5;
	v4l2_info(&csi->sd, "%s PDEC_AVI_PB:%#x, video format:%d\n",
			__func__, val, video_fmt);
	if (video_fmt) {
		if (csi->dsi.vid_mode == VIDEO_MODE)
			rk628_i2c_write(csi->rk628, GRF_CSC_CTRL_CON,
					SW_Y2R_EN(1) | SW_YUV2VYU_SWP(1));
		else
			rk628_i2c_write(csi->rk628, GRF_CSC_CTRL_CON,
					SW_Y2R_EN(1) | SW_YUV2VYU_SWP(0));
	} else {
		if (csi->dsi.vid_mode == VIDEO_MODE)
			rk628_i2c_write(csi->rk628, GRF_CSC_CTRL_CON,
					SW_Y2R_EN(0) | SW_YUV2VYU_SWP(1));
		else
			rk628_i2c_write(csi->rk628, GRF_CSC_CTRL_CON,
					SW_Y2R_EN(0) | SW_YUV2VYU_SWP(0));
	}

	/* if avi packet is not stable, reset ctrl*/
	if (!avi_rdy) {
		csi->nosignal = true;
		schedule_delayed_work(&csi->delayed_work_enable_hotplug, HZ / 20);
	}
}

static void rk628_dsi_enable(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);

	rk628_post_process_setup(sd);

	if (csi->txphy_pwron) {
		v4l2_dbg(1, debug, sd,
			"%s: txphy already power on, power off\n", __func__);
		mipi_dphy_power_off(csi);
		csi->txphy_pwron = false;
	}

	csi->dsi.rk628 = csi->rk628;
	csi->dsi.timings = csi->timings;
	csi->dsi.lane_mbps = csi->lane_mbps;
	rk628_mipi_dsi_power_on(&csi->dsi);
	csi->txphy_pwron = true;
	v4l2_dbg(2, debug, sd, "%s: txphy power on!\n", __func__);
	usleep_range(1000, 1500);
	rk628_dsi_set_scs(csi);
}

static void enable_dsitx(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);

	/* rst for dsi0 */
	rk628_control_assert(csi->rk628, RGU_DSI0);
	usleep_range(20, 40);
	rk628_control_deassert(csi->rk628, RGU_DSI0);
	usleep_range(20, 40);

	rk628_dsi_enable(sd);
}

static void rk628_dsi_enable_stream(struct v4l2_subdev *sd, bool en)
{
	struct rk628_csi *csi = to_csi(sd);

	if (en) {
		rk628_hdmirx_vid_enable(sd, true);
		rk628_i2c_write(csi->rk628, GRF_SCALER_CON0, SCL_EN(1));
		rk628_dsi_set_scs(csi);
		return;
	}

	rk628_hdmirx_vid_enable(sd, false);
	rk628_i2c_write(csi->rk628, GRF_SCALER_CON0, SCL_EN(0));
}

static void enable_stream(struct v4l2_subdev *sd, bool en)
{
	struct rk628_csi *csi = to_csi(sd);

	v4l2_dbg(1, debug, sd, "%s: %sable\n", __func__, en ? "en" : "dis");
	if (en) {
		rk628_hdmirx_vid_enable(sd, true);
		if (csi->plat_data->tx_mode == DSI_MODE)
			enable_dsitx(sd);
		else
			enable_csitx(sd);
	} else {
		if (csi->plat_data->tx_mode == CSI_MODE) {
			rk628_hdmirx_vid_enable(sd, false);
			rk628_i2c_update_bits(csi->rk628, CSITX_CSITX_EN,
					      DPHY_EN_MASK |
					      CSITX_EN_MASK,
					      DPHY_EN(0) |
					      CSITX_EN(0));
			rk628_i2c_write(csi->rk628, CSITX_CONFIG_DONE,
					CONFIG_DONE_IMD);
		} else {
			rk628_dsi_enable_stream(sd, en);
		}
	}
}

static void rk628_post_process_setup(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);
	struct v4l2_bt_timings *bt = &csi->src_timings.bt;
	struct v4l2_bt_timings *dst_bt = &csi->timings.bt;
	struct videomode src, dst;
	u64 dst_pclk;

	src.hactive = bt->width;
	src.hfront_porch = bt->hfrontporch;
	src.hsync_len = bt->hsync;
	src.hback_porch = bt->hbackporch;
	src.vactive = bt->height;
	src.vfront_porch = bt->vfrontporch;
	src.vsync_len = bt->vsync;
	src.vback_porch = bt->vbackporch;
	src.pixelclock = bt->pixelclock;
	src.flags = 0;
	if (bt->interlaced == V4L2_DV_INTERLACED)
		src.flags |= DISPLAY_FLAGS_INTERLACED;
	if (!src.pixelclock) {
		enable_stream(sd, false);
		csi->nosignal = true;
		schedule_delayed_work(&csi->delayed_work_enable_hotplug, HZ / 20);
		return;
	}

	dst.hactive = dst_bt->width;
	dst.hfront_porch = dst_bt->hfrontporch;
	dst.hsync_len = dst_bt->hsync;
	dst.hback_porch = dst_bt->hbackporch;
	dst.vactive = dst_bt->height;
	dst.vfront_porch = dst_bt->vfrontporch;
	dst.vsync_len = dst_bt->vsync;
	dst.vback_porch = dst_bt->vbackporch;
	dst.pixelclock = dst_bt->pixelclock;

	rk628_post_process_en(csi->rk628, &src, &dst, &dst_pclk);
	dst_bt->pixelclock = dst_pclk;
}

static void rk628_csi_set_csi(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);
	u8 video_fmt;
	u8 lanes = csi->csi_lanes_in_use;
	u8 lane_num;
	u8 dphy_lane_en;
	u32 wc_usrdef, val;
	int avi_rdy;

	lane_num = lanes - 1;
	dphy_lane_en = (1 << (lanes + 1)) - 1;
	wc_usrdef = csi->timings.bt.width * 2;

	rk62_csi_reset(sd);
	rk628_post_process_setup(sd);

	if (csi->txphy_pwron) {
		v4l2_dbg(1, debug, sd,
			"%s: txphy already power on, power off\n", __func__);
		mipi_dphy_power_off(csi);
		csi->txphy_pwron = false;
	}

	mipi_dphy_power_on(csi);
	csi->txphy_pwron = true;
	v4l2_dbg(2, debug, sd, "%s: txphy power on!\n", __func__);
	usleep_range(1000, 1500);

	rk628_i2c_update_bits(csi->rk628, CSITX_CSITX_EN,
			VOP_UV_SWAP_MASK |
			VOP_YUV422_EN_MASK |
			VOP_P2_EN_MASK |
			LANE_NUM_MASK |
			DPHY_EN_MASK |
			CSITX_EN_MASK,
			VOP_UV_SWAP(1) |
			VOP_YUV422_EN(1) |
			VOP_P2_EN(1) |
			LANE_NUM(lane_num) |
			DPHY_EN(0) |
			CSITX_EN(0));
	rk628_i2c_update_bits(csi->rk628, CSITX_SYS_CTRL1,
			BYPASS_SELECT_MASK,
			BYPASS_SELECT(1));
	rk628_i2c_write(csi->rk628, CSITX_CONFIG_DONE, CONFIG_DONE_IMD);
	rk628_i2c_write(csi->rk628, CSITX_SYS_CTRL2, VOP_WHOLE_FRM_EN | VSYNC_ENABLE);
	if (csi->continues_clk)
		rk628_i2c_update_bits(csi->rk628, CSITX_SYS_CTRL3_IMD,
			CONT_MODE_CLK_CLR_MASK |
			CONT_MODE_CLK_SET_MASK |
			NON_CONTINUOUS_MODE_MASK,
			CONT_MODE_CLK_CLR(0) |
			CONT_MODE_CLK_SET(1) |
			NON_CONTINUOUS_MODE(0));
	else
		rk628_i2c_update_bits(csi->rk628, CSITX_SYS_CTRL3_IMD,
			CONT_MODE_CLK_CLR_MASK |
			CONT_MODE_CLK_SET_MASK |
			NON_CONTINUOUS_MODE_MASK,
			CONT_MODE_CLK_CLR(0) |
			CONT_MODE_CLK_SET(0) |
			NON_CONTINUOUS_MODE(1));

	rk628_i2c_write(csi->rk628, CSITX_VOP_PATH_CTRL,
			VOP_WC_USERDEFINE(wc_usrdef) |
			VOP_DT_USERDEFINE(YUV422_8BIT) |
			VOP_PIXEL_FORMAT(0) |
			VOP_WC_USERDEFINE_EN(1) |
			VOP_DT_USERDEFINE_EN(1) |
			VOP_PATH_EN(1));
	rk628_i2c_update_bits(csi->rk628, CSITX_DPHY_CTRL,
				CSI_DPHY_EN_MASK,
				CSI_DPHY_EN(dphy_lane_en));
	rk628_i2c_write(csi->rk628, CSITX_CONFIG_DONE, CONFIG_DONE_IMD);
	v4l2_dbg(1, debug, sd, "%s csi cofig done\n", __func__);

	mutex_lock(&csi->confctl_mutex);
	avi_rdy = rk628_is_avi_ready(csi->rk628, csi->avi_rcv_rdy);
	mutex_unlock(&csi->confctl_mutex);

	rk628_i2c_read(csi->rk628, HDMI_RX_PDEC_AVI_PB, &val);
	video_fmt = (val & VIDEO_FORMAT_MASK) >> 5;
	v4l2_dbg(1, debug, &csi->sd, "%s PDEC_AVI_PB:%#x, video format:%d\n",
			__func__, val, video_fmt);
	if (video_fmt) {
		/* yuv data: cfg SW_YUV2VYU_SWP */
		rk628_i2c_write(csi->rk628, GRF_CSC_CTRL_CON,
				SW_YUV2VYU_SWP(1) |
				SW_R2Y_EN(0));
	} else {
		/* rgb data: cfg SW_R2Y_EN */
		rk628_i2c_write(csi->rk628, GRF_CSC_CTRL_CON,
				SW_YUV2VYU_SWP(0) |
				SW_R2Y_EN(1));
	}

	/* if avi packet is not stable, reset ctrl*/
	if (!avi_rdy) {
		csi->nosignal = true;
		schedule_delayed_work(&csi->delayed_work_enable_hotplug, HZ / 20);
	}
}

static int rk628_hdmirx_phy_power_on(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);
	int ret, f;

	/* Bit31 is used to distinguish HDMI cable mode and direct connection
	 * mode in the rk628_combrxphy driver.
	 * Bit31: 0 -direct connection mode;
	 *        1 -cable mode;
	 * The cable mode is to know the input clock frequency through cdr_mode
	 * in the rk628_combrxphy driver, and the cable mode supports up to
	 * 297M, so 297M is passed uniformly here.
	 */
	f = 297000 | BIT(31);

	if (csi->rxphy_pwron) {
		v4l2_dbg(1, debug, sd, "rxphy already power on, power off!\n");
		ret = rk628_rxphy_power_off(csi->rk628);
		if (ret)
			v4l2_err(sd, "hdmi rxphy power off failed!\n");
		else
			csi->rxphy_pwron = false;
		usleep_range(100, 110);
	}

	if (csi->rxphy_pwron == false) {
		rk628_hdmirx_ctrl_enable(sd, 0);
		ret = rk628_rxphy_power_on(csi->rk628, f);
		if (ret) {
			csi->rxphy_pwron = false;
			v4l2_err(sd, "hdmi rxphy power on failed\n");
		} else {
			csi->rxphy_pwron = true;
		}
		rk628_hdmirx_ctrl_enable(sd, 1);
		msleep(100);
	}

	return ret;
}

static int rk628_hdmirx_phy_power_off(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);

	if (csi->rxphy_pwron) {
		v4l2_dbg(1, debug, sd, "rxphy power off!\n");
		rk628_rxphy_power_off(csi->rk628);
		csi->rxphy_pwron = false;
	}
	usleep_range(100, 100);

	return 0;
}

static void rk628_hdmirx_vid_enable(struct v4l2_subdev *sd, bool en)
{
	struct rk628_csi *csi = to_csi(sd);

	v4l2_dbg(1, debug, sd, "%s: %sable\n", __func__, en ? "en" : "dis");
	if (en) {
		if (!csi->i2s_enable_default)
			rk628_hdmirx_audio_i2s_ctrl(csi->audio_info, true);
		rk628_i2c_update_bits(csi->rk628, HDMI_RX_DMI_DISABLE_IF,
				      VID_ENABLE_MASK, VID_ENABLE(1));
	} else {
		if (!csi->i2s_enable_default)
			rk628_hdmirx_audio_i2s_ctrl(csi->audio_info, false);
		rk628_i2c_update_bits(csi->rk628, HDMI_RX_DMI_DISABLE_IF,
				      VID_ENABLE_MASK, VID_ENABLE(0));
	}
}

static void rk628_hdmirx_controller_reset(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);

	v4l2_dbg(1, debug, sd, "%s reset hdmirx_controller\n", __func__);
	rk628_control_assert(csi->rk628, RGU_HDMIRX_PON);
	udelay(10);
	rk628_control_deassert(csi->rk628, RGU_HDMIRX_PON);
	udelay(10);
	rk628_i2c_write(csi->rk628, HDMI_RX_DMI_SW_RST, 0x000101ff);
	rk628_i2c_write(csi->rk628, HDMI_RX_DMI_DISABLE_IF, 0x00000000);
	rk628_i2c_write(csi->rk628, HDMI_RX_DMI_DISABLE_IF, 0x0000017f);
	rk628_i2c_write(csi->rk628, HDMI_RX_DMI_DISABLE_IF, 0x0001017f);
}

static bool rk628_rcv_supported_res(struct v4l2_subdev *sd, u32 width,
		u32 height)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if ((supported_modes[i].width == width) &&
		    (supported_modes[i].height == height)) {
			break;
		}
	}
	if (i == ARRAY_SIZE(supported_modes)) {
		v4l2_err(sd, "%s do not support res wxh: %dx%d\n", __func__,
				width, height);
		return false;
	} else {
		return true;
	}
}

static int rk628_hdmirx_phy_setup(struct v4l2_subdev *sd)
{
	u32 i, cnt, val;
	u32 width, height, frame_width, frame_height, status;
	struct rk628_csi *csi = to_csi(sd);
	int ret;

	for (i = 0; i < RXPHY_CFG_MAX_TIMES; i++) {
		ret = rk628_hdmirx_phy_power_on(sd);
		if (ret < 0) {
			msleep(50);
			continue;
		}
		cnt = 0;

		do {
			cnt++;
			rk628_i2c_read(csi->rk628, HDMI_RX_MD_HACT_PX, &val);
			width = val & 0xffff;
			rk628_i2c_read(csi->rk628, HDMI_RX_MD_VAL, &val);
			height = val & 0xffff;
			rk628_i2c_read(csi->rk628, HDMI_RX_MD_HT1, &val);
			frame_width = (val >> 16) & 0xffff;
			rk628_i2c_read(csi->rk628, HDMI_RX_MD_VTL, &val);
			frame_height = val & 0xffff;
			rk628_i2c_read(csi->rk628, HDMI_RX_SCDC_REGS1, &val);
			status = val;
			v4l2_dbg(1, debug, sd, "%s read wxh:%dx%d, total:%dx%d, SCDC_REGS1:%#x, cnt:%d\n",
				 __func__, width, height, frame_width, frame_height, status, cnt);

			rk628_i2c_read(csi->rk628, HDMI_RX_PDEC_STS, &val);
			if (val & DVI_DET)
				dev_info(csi->dev, "DVI mode detected\n");

			if (!tx_5v_power_present(sd)) {
				v4l2_info(sd, "HDMI pull out, return!\n");
				return -1;
			}

			if (cnt >= 15)
				break;
		} while (((status & 0xfff) != 0xf00) ||
				(!rk628_rcv_supported_res(sd, width, height)));

		if (((status & 0xfff) != 0xf00) ||
				(!rk628_rcv_supported_res(sd, width, height))) {
			v4l2_err(sd, "%s hdmi rxphy lock failed, retry:%d\n",
					__func__, i);
			continue;
		} else {
			break;
		}
	}

	if (i == RXPHY_CFG_MAX_TIMES)
		return -1;

	return 0;
}

static void rk628_csi_initial_setup(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);
	struct v4l2_subdev_edid def_edid;

	/* selete int io function */
	rk628_i2c_write(csi->rk628, GRF_GPIO3AB_SEL_CON, 0x30002000);
	rk628_i2c_write(csi->rk628, GRF_GPIO1AB_SEL_CON, HIWORD_UPDATE(0x7, 10, 8));
	/* I2S_SCKM0 */
	rk628_i2c_write(csi->rk628, GRF_GPIO0AB_SEL_CON, HIWORD_UPDATE(0x1, 2, 2));
	/* I2SLR_M0 */
	rk628_i2c_write(csi->rk628, GRF_GPIO0AB_SEL_CON, HIWORD_UPDATE(0x1, 3, 3));
	/* I2SM0D0 */
	rk628_i2c_write(csi->rk628, GRF_GPIO0AB_SEL_CON, HIWORD_UPDATE(0x1, 5, 4));
	/* hdmirx int en */
	rk628_i2c_write(csi->rk628, GRF_INTR0_EN, 0x01000100);

	udelay(10);
	rk628_control_assert(csi->rk628, RGU_HDMIRX);
	rk628_control_assert(csi->rk628, RGU_HDMIRX_PON);
	rk628_control_assert(csi->rk628, RGU_CSI);
	udelay(10);
	rk628_control_deassert(csi->rk628, RGU_HDMIRX);
	rk628_control_deassert(csi->rk628, RGU_HDMIRX_PON);
	rk628_control_deassert(csi->rk628, RGU_CSI);
	udelay(10);

	rk628_i2c_update_bits(csi->rk628, GRF_SYSTEM_CON0,
			SW_INPUT_MODE_MASK |
			SW_OUTPUT_MODE_MASK |
			SW_EFUSE_HDCP_EN_MASK |
			SW_HSYNC_POL_MASK |
			SW_VSYNC_POL_MASK,
			SW_INPUT_MODE(INPUT_MODE_HDMI) |
			SW_OUTPUT_MODE(OUTPUT_MODE_CSI) |
			SW_EFUSE_HDCP_EN(0) |
			SW_HSYNC_POL(1) |
			SW_VSYNC_POL(1));
	rk628_hdmirx_controller_reset(sd);

	def_edid.pad = 0;
	def_edid.start_block = 0;
	def_edid.blocks = 2;
	def_edid.edid = edid_init_data;
	rk628_csi_s_edid(sd, &def_edid);
	rk628_hdmirx_set_hdcp(csi->rk628, &csi->hdcp, false);

	if (csi->plat_data->tx_mode == CSI_MODE) {
		mipi_dphy_reset(csi->rk628);
		mipi_dphy_power_on(csi);
	}
	csi->txphy_pwron = true;
	if (tx_5v_power_present(sd))
		schedule_delayed_work(&csi->delayed_work_enable_hotplug, msecs_to_jiffies(1000));
}

static void rk628_csi_format_change(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);
	struct v4l2_dv_timings timings;
	const struct v4l2_event rk628_csi_ev_fmt = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	rk628_csi_get_detected_timings(sd, &timings);
	if (!v4l2_match_dv_timings(&csi->timings, &timings, 0, false)) {
		/* automatically set timing rather than set by userspace */
		rk628_csi_s_dv_timings(sd, &timings);
		v4l2_print_dv_timings(sd->name,
				"rk628_csi_format_change: New format: ",
				&timings, false);
	}

	if (sd->devnode)
		v4l2_subdev_notify_event(sd, &rk628_csi_ev_fmt);
}

static void rk628_csi_enable_interrupts(struct v4l2_subdev *sd, bool en)
{
	u32 pdec_ien, md_ien;
	u32 pdec_mask = 0, md_mask = 0;
	struct rk628_csi *csi = to_csi(sd);

	pdec_mask |= AVI_RCV_ENSET;
	md_mask = VACT_LIN_ENSET | HACT_PIX_ENSET | HS_CLK_ENSET |
		  DE_ACTIVITY_ENSET | VS_ACT_ENSET | HS_ACT_ENSET;
	v4l2_dbg(1, debug, sd, "%s: %sable\n", __func__, en ? "en" : "dis");
	/* clr irq */
	rk628_i2c_write(csi->rk628, HDMI_RX_MD_ICLR, md_mask);
	rk628_i2c_write(csi->rk628, HDMI_RX_PDEC_ICLR, pdec_mask);
	if (en) {
		rk628_i2c_write(csi->rk628, HDMI_RX_MD_IEN_SET, md_mask);
		rk628_i2c_write(csi->rk628, HDMI_RX_PDEC_IEN_SET, pdec_mask);
		csi->vid_ints_en = true;
	} else {
		rk628_i2c_write(csi->rk628, HDMI_RX_MD_IEN_CLR, md_mask);
		rk628_i2c_write(csi->rk628, HDMI_RX_PDEC_IEN_CLR, pdec_mask);
		rk628_i2c_write(csi->rk628, HDMI_RX_AUD_FIFO_IEN_CLR, 0x1f);
		csi->vid_ints_en = false;
	}
	usleep_range(5000, 5000);
	rk628_i2c_read(csi->rk628, HDMI_RX_MD_IEN, &md_ien);
	rk628_i2c_read(csi->rk628, HDMI_RX_PDEC_IEN, &pdec_ien);
	v4l2_dbg(1, debug, sd, "%s MD_IEN:%#x, PDEC_IEN:%#x\n", __func__, md_ien, pdec_ien);
}

static int rk628_csi_isr(struct v4l2_subdev *sd, u32 status, bool *handled)
{
	u32 md_ints, pdec_ints, fifo_ints, hact, vact;
	bool plugin;
	struct rk628_csi *csi = to_csi(sd);
	void *audio_info = csi->audio_info;

	if (handled == NULL) {
		v4l2_err(sd, "handled NULL, err return!\n");
		return -EINVAL;
	}
	rk628_i2c_read(csi->rk628, HDMI_RX_PDEC_ISTS, &pdec_ints);
	if (rk628_audio_ctsnints_enabled(audio_info)) {
		if (pdec_ints & (ACR_N_CHG_ICLR | ACR_CTS_CHG_ICLR)) {
			rk628_csi_isr_ctsn(audio_info, pdec_ints);
			pdec_ints &= ~(ACR_CTS_CHG_ICLR | ACR_CTS_CHG_ICLR);
			*handled = true;
		}
	}
	if (rk628_audio_fifoints_enabled(audio_info)) {
		rk628_i2c_read(csi->rk628, HDMI_RX_AUD_FIFO_ISTS, &fifo_ints);
		if (fifo_ints & 0x18) {
			rk628_csi_isr_fifoints(audio_info, fifo_ints);
			*handled = true;
		}
	}
	if (csi->vid_ints_en) {
		rk628_i2c_read(csi->rk628, HDMI_RX_MD_ISTS, &md_ints);
		plugin = tx_5v_power_present(sd);
		v4l2_dbg(1, debug, sd, "%s: md_ints: %#x, pdec_ints:%#x, plugin: %d\n",
			 __func__, md_ints, pdec_ints, plugin);

		if ((md_ints & (VACT_LIN_ISTS | HACT_PIX_ISTS |
				HS_CLK_ISTS | DE_ACTIVITY_ISTS |
				VS_ACT_ISTS | HS_ACT_ISTS))
				&& plugin) {

			rk628_i2c_read(csi->rk628, HDMI_RX_MD_HACT_PX, &hact);
			rk628_i2c_read(csi->rk628, HDMI_RX_MD_VAL, &vact);
			v4l2_dbg(1, debug, sd, "%s: HACT:%#x, VACT:%#x\n",
				 __func__, hact, vact);

			rk628_csi_enable_interrupts(sd, false);
			enable_stream(sd, false);
			csi->nosignal = true;
			schedule_delayed_work(&csi->delayed_work_res_change, HZ / 2);

			v4l2_dbg(1, debug, sd, "%s: hact/vact change, md_ints: %#x\n",
				 __func__, (u32)(md_ints & (VACT_LIN_ISTS | HACT_PIX_ISTS)));
			*handled = true;
		}

		if ((pdec_ints & AVI_RCV_ISTS) && plugin && !csi->avi_rcv_rdy) {
			v4l2_dbg(1, debug, sd, "%s: AVI RCV INT!\n", __func__);
			if (csi->plat_data->tx_mode == DSI_MODE)
				enable_stream(sd, false);
			csi->avi_rcv_rdy = true;
			/* After get the AVI_RCV interrupt state, disable interrupt. */
			rk628_i2c_write(csi->rk628, HDMI_RX_PDEC_IEN_CLR, AVI_RCV_ISTS);

			*handled = true;
		}
	}
	if (*handled != true)
		v4l2_dbg(1, debug, sd, "%s: unhandled interrupt!\n", __func__);

	/* clear interrupts */
	rk628_i2c_write(csi->rk628, HDMI_RX_MD_ICLR, 0xffffffff);
	rk628_i2c_write(csi->rk628, HDMI_RX_PDEC_ICLR, 0xffffffff);
	rk628_i2c_write(csi->rk628, GRF_INTR0_CLR_EN, 0x01000100);

	return 0;
}

static irqreturn_t rk628_csi_irq_handler(int irq, void *dev_id)
{
	struct rk628_csi *csi = dev_id;
	bool handled = false;

	rk628_csi_isr(&csi->sd, 0, &handled);

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static void rk628_csi_irq_poll_timer(struct timer_list *t)
{
	struct rk628_csi *csi = from_timer(csi, t, timer);

	schedule_work(&csi->work_i2c_poll);
	mod_timer(&csi->timer, jiffies + msecs_to_jiffies(POLL_INTERVAL_MS));
}

static void rk628_csi_work_i2c_poll(struct work_struct *work)
{
	struct rk628_csi *csi = container_of(work, struct rk628_csi,
			work_i2c_poll);
	struct v4l2_subdev *sd = &csi->sd;

	rk628_csi_format_change(sd);
}

static int rk628_csi_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				    struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subdev_subscribe_event(sd, fh, sub);
	default:
		return -EINVAL;
	}
}

static int rk628_csi_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct rk628_csi *csi = to_csi(sd);
	static u8 cnt;

	*status = 0;
	*status |= no_signal(sd) ? V4L2_IN_ST_NO_SIGNAL : 0;

	if (no_signal(sd) && tx_5v_power_present(sd)) {
		if (cnt++ >= 6) {
			cnt = 0;
			v4l2_info(sd, "no signal but 5v_det, recfg hdmirx!\n");
			schedule_delayed_work(&csi->delayed_work_enable_hotplug,
					HZ / 20);
		}
	} else {
		cnt = 0;
	}

	v4l2_dbg(1, debug, sd, "%s: status = 0x%x\n", __func__, *status);

	return 0;
}

static int rk628_csi_s_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct rk628_csi *csi = to_csi(sd);

	if (!timings)
		return -EINVAL;

	if (debug)
		v4l2_print_dv_timings(sd->name, "rk628_csi_s_dv_timings: ",
				timings, false);

	if (v4l2_match_dv_timings(&csi->timings, timings, 0, false)) {
		v4l2_dbg(1, debug, sd, "%s: no change\n", __func__);
		return 0;
	}

	if (!v4l2_valid_dv_timings(timings, &rk628_csi_timings_cap, NULL,
				NULL)) {
		v4l2_dbg(1, debug, sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	csi->timings = *timings;
	enable_stream(sd, false);

	return 0;
}

static int rk628_csi_g_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct rk628_csi *csi = to_csi(sd);

	*timings = csi->timings;

	return 0;
}

static int rk628_csi_enum_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_enum_dv_timings *timings)
{
	if (timings->pad != 0)
		return -EINVAL;

	return v4l2_enum_dv_timings_cap(timings, &rk628_csi_timings_cap, NULL,
			NULL);
}

static int rk628_csi_query_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	int ret;
	struct rk628_csi *csi = to_csi(sd);

	mutex_lock(&csi->confctl_mutex);
	ret = rk628_csi_get_detected_timings(sd, timings);
	mutex_unlock(&csi->confctl_mutex);
	if (ret)
		return ret;

	if (debug)
		v4l2_print_dv_timings(sd->name, "rk628_csi_query_dv_timings: ",
				timings, false);

	if (!v4l2_valid_dv_timings(timings, &rk628_csi_timings_cap, NULL,
				NULL)) {
		v4l2_dbg(1, debug, sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	return 0;
}

static int rk628_csi_dv_timings_cap(struct v4l2_subdev *sd,
		struct v4l2_dv_timings_cap *cap)
{
	if (cap->pad != 0)
		return -EINVAL;

	*cap = rk628_csi_timings_cap;

	return 0;
}

static int rk628_csi_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
			     struct v4l2_mbus_config *cfg)
{
	struct rk628_csi *csi = to_csi(sd);

	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->flags = V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	switch (csi->csi_lanes_in_use) {
	case 1:
		cfg->flags |= V4L2_MBUS_CSI2_1_LANE;
		break;
	case 2:
		cfg->flags |= V4L2_MBUS_CSI2_2_LANE;
		break;
	case 3:
		cfg->flags |= V4L2_MBUS_CSI2_3_LANE;
		break;
	case 4:
		cfg->flags |= V4L2_MBUS_CSI2_4_LANE;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int rk628_csi_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct rk628_csi *csi = to_csi(sd);

	if (csi->plat_data->tx_mode == CSI_MODE)
		enable_stream(sd, enable);
	else
		rk628_dsi_enable_stream(sd, enable);

	return 0;
}

static int rk628_csi_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	struct rk628_csi *csi = to_csi(sd);

	switch (code->index) {
	case 0:
		code->code = csi->plat_data->bus_fmt;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int rk628_csi_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct rk628_csi *csi = to_csi(sd);

	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != csi->plat_data->bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int rk628_csi_enum_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	struct rk628_csi *csi = to_csi(sd);

	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fie->code = csi->plat_data->bus_fmt;

	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static int rk628_csi_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct rk628_csi *csi = to_csi(sd);

	mutex_lock(&csi->confctl_mutex);
	format->format.code = csi->mbus_fmt_code;
	format->format.width = csi->timings.bt.width;
	format->format.height = csi->timings.bt.height;
	format->format.field = csi->timings.bt.interlaced ?
		V4L2_FIELD_INTERLACED : V4L2_FIELD_NONE;
	mutex_unlock(&csi->confctl_mutex);

	if (((csi->timings.bt.width == 3840) && (csi->timings.bt.height == 2160)) ||
		csi->csi_lanes_in_use <= 2) {
		v4l2_dbg(1, debug, sd,
			"%s res wxh:%dx%d, link freq:%llu, pixrate:%u\n",
			__func__, csi->timings.bt.width, csi->timings.bt.height,
			link_freq_menu_items[1], RK628_CSI_PIXEL_RATE_HIGH);
		__v4l2_ctrl_s_ctrl(csi->link_freq, 1);
		__v4l2_ctrl_s_ctrl_int64(csi->pixel_rate,
			RK628_CSI_PIXEL_RATE_HIGH);
	} else {
		v4l2_dbg(1, debug, sd,
			"%s res wxh:%dx%d, link freq:%llu, pixrate:%u\n",
			__func__, csi->timings.bt.width, csi->timings.bt.height,
			link_freq_menu_items[0], RK628_CSI_PIXEL_RATE_LOW);
		__v4l2_ctrl_s_ctrl(csi->link_freq, 0);
		__v4l2_ctrl_s_ctrl_int64(csi->pixel_rate,
			RK628_CSI_PIXEL_RATE_LOW);
	}

	v4l2_dbg(1, debug, sd, "%s: fmt code:%d, w:%d, h:%d, field code:%d\n",
			__func__, format->format.code, format->format.width,
			format->format.height, format->format.field);

	return 0;
}

static int rk628_csi_get_reso_dist(const struct rk628_csi_mode *mode,
		struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct rk628_csi_mode *
rk628_csi_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = rk628_csi_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int rk628_csi_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct rk628_csi *csi = to_csi(sd);
	const struct rk628_csi_mode *mode;

	u32 code = format->format.code; /* is overwritten by get_fmt */
	int ret = rk628_csi_get_fmt(sd, cfg, format);

	format->format.code = code;

	if (ret)
		return ret;

	switch (code) {
	case MEDIA_BUS_FMT_UYVY8_2X8:
		if (csi->plat_data->bus_fmt == MEDIA_BUS_FMT_UYVY8_2X8)
			break;
		return -EINVAL;
	case MEDIA_BUS_FMT_RGB888_1X24:
		if (csi->plat_data->bus_fmt == MEDIA_BUS_FMT_RGB888_1X24)
			break;
		return -EINVAL;
	default:
		return -EINVAL;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		if (csi->plat_data->bus_fmt == MEDIA_BUS_FMT_UYVY8_2X8)
			return 0;

		*v4l2_subdev_get_try_format(sd, cfg, format->pad) = format->format;
	}

	csi->mbus_fmt_code = format->format.code;
	mode = rk628_csi_find_best_fit(format);
	csi->cur_mode = mode;

	enable_stream(sd, false);

	return 0;
}

static int rk628_csi_g_edid(struct v4l2_subdev *sd,
		struct v4l2_subdev_edid *edid)
{
	struct rk628_csi *csi = to_csi(sd);
	u32 i, val;

	memset(edid->reserved, 0, sizeof(edid->reserved));

	if (edid->pad != 0)
		return -EINVAL;

	if (edid->start_block == 0 && edid->blocks == 0) {
		edid->blocks = csi->edid_blocks_written;
		return 0;
	}

	if (csi->edid_blocks_written == 0)
		return -ENODATA;

	if (edid->start_block >= csi->edid_blocks_written ||
			edid->blocks == 0)
		return -EINVAL;

	if (edid->start_block + edid->blocks > csi->edid_blocks_written)
		edid->blocks = csi->edid_blocks_written - edid->start_block;

	/* edid access by apb when read, i2c slave addr: 0x0 */
	rk628_i2c_update_bits(csi->rk628, GRF_SYSTEM_CON0,
			SW_ADAPTER_I2CSLADR_MASK |
			SW_EDID_MODE_MASK,
			SW_ADAPTER_I2CSLADR(0) |
			SW_EDID_MODE(1));

	for (i = 0; i < (edid->blocks * EDID_BLOCK_SIZE); i++) {
		rk628_i2c_read(csi->rk628, EDID_BASE + ((edid->start_block *
				EDID_BLOCK_SIZE) + i) * 4, &val);
		edid->edid[i] = val;
	}

	rk628_i2c_update_bits(csi->rk628, GRF_SYSTEM_CON0,
			SW_EDID_MODE_MASK,
			SW_EDID_MODE(0));

	return 0;
}

static int rk628_csi_s_edid(struct v4l2_subdev *sd,
				struct v4l2_subdev_edid *edid)
{
	struct rk628_csi *csi = to_csi(sd);
	u16 edid_len = edid->blocks * EDID_BLOCK_SIZE;
	u32 i, val;

	v4l2_dbg(1, debug, sd, "%s, pad %d, start block %d, blocks %d\n",
		 __func__, edid->pad, edid->start_block, edid->blocks);

	memset(edid->reserved, 0, sizeof(edid->reserved));

	if (edid->pad != 0)
		return -EINVAL;

	if (edid->start_block != 0)
		return -EINVAL;

	if (edid->blocks > EDID_NUM_BLOCKS_MAX) {
		edid->blocks = EDID_NUM_BLOCKS_MAX;
		return -E2BIG;
	}

	rk628_hdmirx_hpd_ctrl(sd, false);

	if (edid->blocks == 0) {
		csi->edid_blocks_written = 0;
		return 0;
	}

	/* edid access by apb when write, i2c slave addr: 0x0 */
	rk628_i2c_update_bits(csi->rk628, GRF_SYSTEM_CON0,
			SW_ADAPTER_I2CSLADR_MASK |
			SW_EDID_MODE_MASK,
			SW_ADAPTER_I2CSLADR(0) |
			SW_EDID_MODE(1));

	for (i = 0; i < edid_len; i++)
		rk628_i2c_write(csi->rk628, EDID_BASE + i * 4, edid->edid[i]);

	/* read out for debug */
	if (debug >= 3) {
		pr_info("%s: Read EDID: ======\n", __func__);
		for (i = 0; i < edid_len; i++) {
			rk628_i2c_read(csi->rk628, EDID_BASE + i * 4, &val);
			pr_info("0x%02x ", val);
			if ((i + 1) % 8 == 0)
				pr_info("\n");
		}
		pr_info("%s: ======\n", __func__);
	}

	/* edid access by RX's i2c, i2c slave addr: 0x0 */
	rk628_i2c_update_bits(csi->rk628, GRF_SYSTEM_CON0,
			SW_ADAPTER_I2CSLADR_MASK |
			SW_EDID_MODE_MASK,
			SW_ADAPTER_I2CSLADR(0) |
			SW_EDID_MODE(0));
	csi->edid_blocks_written = edid->blocks;
	udelay(100);

	if (tx_5v_power_present(sd))
		rk628_hdmirx_hpd_ctrl(sd, true);

	return 0;
}

static int rk628_csi_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct rk628_csi *csi = to_csi(sd);
	const struct rk628_csi_mode *mode = csi->cur_mode;

	mutex_lock(&csi->confctl_mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&csi->confctl_mutex);

	return 0;
}

static void rk628_csi_get_module_inf(struct rk628_csi *rk628_csi,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, RK628_CSI_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, rk628_csi->module_name,
		sizeof(inf->base.module));
	strscpy(inf->base.lens, rk628_csi->len_name, sizeof(inf->base.lens));
}

static long rk628_csi_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct rk628_csi *csi = to_csi(sd);
	long ret = 0;
	struct rkmodule_csi_dphy_param *dphy_param;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		rk628_csi_get_module_inf(csi, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDMI_MODE:
		*(int *)arg = RKMODULE_HDMIIN_MODE;
		break;
	case RKMODULE_SET_CSI_DPHY_PARAM:
		dphy_param = (struct rkmodule_csi_dphy_param *)arg;
		if (dphy_param->vendor == PHY_VENDOR_SAMSUNG)
			rk3588_dcphy_param = *dphy_param;
		v4l2_dbg(1, debug, sd,
			"sensor set dphy param\n");
		break;
	case RKMODULE_GET_CSI_DPHY_PARAM:
		dphy_param = (struct rkmodule_csi_dphy_param *)arg;
		*dphy_param = rk3588_dcphy_param;

		v4l2_dbg(1, debug, sd,
			"sensor get dphy param\n");
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static int mipi_dphy_power_on(struct rk628_csi *csi)
{
	unsigned int val;
	u32 bus_width, mask;
	struct v4l2_subdev *sd = &csi->sd;

	if ((csi->timings.bt.width == 3840 && csi->timings.bt.height == 2160) ||
	    csi->csi_lanes_in_use <= 2) {
		csi->lane_mbps = MIPI_DATARATE_MBPS_HIGH;
	} else {
		csi->lane_mbps = MIPI_DATARATE_MBPS_LOW;
	}

	bus_width =  csi->lane_mbps << 8;
	bus_width |= COMBTXPHY_MODULEA_EN;
	v4l2_dbg(1, debug, sd, "%s mipi bitrate:%llu mbps\n", __func__,
			csi->lane_mbps);
	rk628_txphy_set_bus_width(csi->rk628, bus_width);
	rk628_txphy_set_mode(csi->rk628, PHY_MODE_VIDEO_MIPI);

	if (csi->lane_mbps == MIPI_DATARATE_MBPS_HIGH)
		mipi_dphy_init_hsmanual(csi->rk628, true);
	else
		mipi_dphy_init_hsmanual(csi->rk628, false);
	mipi_dphy_init_hsfreqrange(csi->rk628, csi->lane_mbps);
	usleep_range(1500, 2000);
	rk628_txphy_power_on(csi->rk628);

	usleep_range(1500, 2000);
	mask = DPHY_PLL_LOCK;
	rk628_i2c_read(csi->rk628, CSITX_CSITX_STATUS1, &val);
	if ((val & mask) != mask) {
		dev_err(csi->dev, "PHY is not locked\n");
		return -1;
	}

	udelay(10);

	return 0;
}

static void mipi_dphy_power_off(struct rk628_csi *csi)
{
	rk628_txphy_power_off(csi->rk628);
}

#ifdef CONFIG_COMPAT
static long rk628_csi_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	long ret;
	int *seq;
	struct rkmodule_csi_dphy_param *dphy_param;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = rk628_csi_ioctl(sd, cmd, inf);
		if (!ret) {
			ret = copy_to_user(up, inf, sizeof(*inf));
			if (ret)
				ret = -EFAULT;
		}
		kfree(inf);
		break;
	case RKMODULE_GET_HDMI_MODE:
		seq = kzalloc(sizeof(*seq), GFP_KERNEL);
		if (!seq) {
			ret = -ENOMEM;
			return ret;
		}

		ret = rk628_csi_ioctl(sd, cmd, seq);
		if (!ret) {
			ret = copy_to_user(up, seq, sizeof(*seq));
			if (ret)
				ret = -EFAULT;
		}
		kfree(seq);
		break;
	case RKMODULE_SET_CSI_DPHY_PARAM:
		dphy_param = kzalloc(sizeof(*dphy_param), GFP_KERNEL);
		if (!dphy_param) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(dphy_param, up, sizeof(*dphy_param));
		if (!ret)
			ret = rk628_csi_ioctl(sd, cmd, dphy_param);
		else
			ret = -EFAULT;
		kfree(dphy_param);
		break;
	case RKMODULE_GET_CSI_DPHY_PARAM:
		dphy_param = kzalloc(sizeof(*dphy_param), GFP_KERNEL);
		if (!dphy_param) {
			ret = -ENOMEM;
			return ret;
		}

		ret = rk628_csi_ioctl(sd, cmd, dphy_param);
		if (!ret) {
			ret = copy_to_user(up, dphy_param, sizeof(*dphy_param));
			if (ret)
				ret = -EFAULT;
		}
		kfree(dphy_param);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static const struct v4l2_subdev_core_ops rk628_csi_core_ops = {
	.interrupt_service_routine = rk628_csi_isr,
	.subscribe_event = rk628_csi_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.ioctl = rk628_csi_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = rk628_csi_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops rk628_csi_video_ops = {
	.g_input_status = rk628_csi_g_input_status,
	.s_dv_timings = rk628_csi_s_dv_timings,
	.g_dv_timings = rk628_csi_g_dv_timings,
	.query_dv_timings = rk628_csi_query_dv_timings,
	.s_stream = rk628_csi_s_stream,
	.g_frame_interval = rk628_csi_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops rk628_csi_pad_ops = {
	.enum_mbus_code = rk628_csi_enum_mbus_code,
	.enum_frame_size = rk628_csi_enum_frame_sizes,
	.enum_frame_interval = rk628_csi_enum_frame_interval,
	.set_fmt = rk628_csi_set_fmt,
	.get_fmt = rk628_csi_get_fmt,
	.get_edid = rk628_csi_g_edid,
	.set_edid = rk628_csi_s_edid,
	.enum_dv_timings = rk628_csi_enum_dv_timings,
	.dv_timings_cap = rk628_csi_dv_timings_cap,
	.get_mbus_config = rk628_csi_g_mbus_config,
};

static const struct v4l2_subdev_ops rk628_csi_ops = {
	.core = &rk628_csi_core_ops,
	.video = &rk628_csi_video_ops,
	.pad = &rk628_csi_pad_ops,
};

static int rk628_csi_get_custom_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = -EINVAL;
	struct rk628_csi *csi = container_of(ctrl->handler, struct rk628_csi,
			hdl);
	struct v4l2_subdev *sd = &csi->sd;

	if (ctrl->id == RK_V4L2_CID_AUDIO_SAMPLING_RATE) {
		ret = get_audio_sampling_rate(sd);
		*ctrl->p_new.p_s32 = ret;
	}

	return ret;
}

static const struct v4l2_ctrl_ops rk628_csi_custom_ctrl_ops = {
	.g_volatile_ctrl = rk628_csi_get_custom_ctrl,
};

static const struct v4l2_ctrl_config rk628_csi_ctrl_audio_sampling_rate = {
	.ops = &rk628_csi_custom_ctrl_ops,
	.id = RK_V4L2_CID_AUDIO_SAMPLING_RATE,
	.name = "Audio sampling rate",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 768000,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config rk628_csi_ctrl_audio_present = {
	.id = RK_V4L2_CID_AUDIO_PRESENT,
	.name = "Audio present",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static irqreturn_t plugin_detect_irq(int irq, void *dev_id)
{
	struct rk628_csi *csi = dev_id;
	struct v4l2_subdev *sd = &csi->sd;

	/* control hpd after 50ms */
	schedule_delayed_work(&csi->delayed_work_enable_hotplug, HZ / 20);
	tx_5v_power_present(sd);

	return IRQ_HANDLED;
}

static int rk628_csi_probe_of(struct rk628_csi *csi)
{
	struct device *dev = csi->dev;
	struct v4l2_fwnode_endpoint endpoint = { .bus_type = 0 };
	struct device_node *ep;
	int ret = -EINVAL;
	bool hdcp1x_enable = false, i2s_enable_default = false;
	bool scaler_en = false;
	bool continues_clk = false;

	csi->soc_24M = devm_clk_get(dev, "soc_24M");
	if (csi->soc_24M == ERR_PTR(-ENOENT))
		csi->soc_24M = NULL;
	if (IS_ERR(csi->soc_24M)) {
		ret = PTR_ERR(csi->soc_24M);
		dev_err(dev, "Unable to get soc_24M: %d\n", ret);
	}
	clk_prepare_enable(csi->soc_24M);

	csi->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						     GPIOD_OUT_LOW);
	if (IS_ERR(csi->enable_gpio)) {
		ret = PTR_ERR(csi->enable_gpio);
		dev_err(dev, "failed to request enable GPIO: %d\n", ret);
		goto clk_put;
	}

	csi->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(csi->reset_gpio)) {
		ret = PTR_ERR(csi->reset_gpio);
		dev_err(dev, "failed to request reset GPIO: %d\n", ret);
		goto clk_put;
	}

	csi->power_gpio = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_HIGH);
	if (IS_ERR(csi->power_gpio)) {
		dev_err(dev, "failed to get power gpio\n");
		ret = PTR_ERR(csi->power_gpio);
		goto clk_put;
	}

	csi->plugin_det_gpio = devm_gpiod_get_optional(dev, "plugin-det",
						    GPIOD_IN);
	if (IS_ERR(csi->plugin_det_gpio)) {
		dev_err(dev, "failed to get hdmirx det gpio\n");
		ret = PTR_ERR(csi->plugin_det_gpio);
		goto clk_put;
	}

	if (csi->enable_gpio) {
		gpiod_set_value(csi->enable_gpio, 1);
		usleep_range(10000, 11000);
	}
	gpiod_set_value(csi->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value(csi->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value(csi->reset_gpio, 0);
	usleep_range(10000, 11000);

	if (csi->power_gpio) {
		gpiod_set_value(csi->power_gpio, 1);
		usleep_range(500, 510);
	}

	if (of_property_read_bool(dev->of_node, "hdcp-enable"))
		hdcp1x_enable = true;

	if (of_property_read_bool(dev->of_node, "i2s-enable-default"))
		i2s_enable_default = true;

	if (csi->plat_data->tx_mode == DSI_MODE) {
		if (of_property_read_bool(dev->of_node, "dsi-video-mode"))
			csi->dsi.vid_mode = VIDEO_MODE;
		else
			csi->dsi.vid_mode = COMMAND_MODE;
	}

	if (of_property_read_bool(dev->of_node, "scaler-en"))
		scaler_en = true;

	if (of_property_read_bool(dev->of_node, "continues-clk"))
		continues_clk = true;

	ep = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!ep) {
		dev_err(dev, "missing endpoint node\n");
		ret = -EINVAL;
		goto clk_put;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(of_fwnode_handle(ep), &endpoint);
	if (ret) {
		dev_err(dev, "failed to parse endpoint\n");
		goto put_node;
	}

	if (endpoint.bus_type != V4L2_MBUS_CSI2_DPHY ||
	    endpoint.bus.mipi_csi2.num_data_lanes == 0) {
		dev_err(dev, "missing CSI-2 properties in endpoint\n");
		goto free_endpoint;
	}

	csi->csi_lanes_in_use = endpoint.bus.mipi_csi2.num_data_lanes;
	csi->enable_hdcp = hdcp1x_enable;
	csi->i2s_enable_default = i2s_enable_default;
	csi->scaler_en = scaler_en;
	if (csi->scaler_en)
		csi->timings = dst_timing;
	csi->continues_clk = continues_clk;

	csi->rxphy_pwron = false;
	csi->txphy_pwron = false;
	csi->nosignal = true;
	csi->stream_state = 0;
	csi->avi_rcv_rdy = false;

	ret = 0;

free_endpoint:
	v4l2_fwnode_endpoint_free(&endpoint);
put_node:
	of_node_put(ep);
clk_put:
	clk_disable_unprepare(csi->soc_24M);

	return ret;
}

static const struct rk628_plat_data rk628_csi_data = {
	.bus_fmt = MEDIA_BUS_FMT_UYVY8_2X8,
	.tx_mode = CSI_MODE,
};

static const struct rk628_plat_data rk628_dsi_data = {
	.bus_fmt = MEDIA_BUS_FMT_RGB888_1X24,
	.tx_mode = DSI_MODE,
};

static const struct i2c_device_id rk628_csi_i2c_id[] = {
	{ "rk628-csi-v4l2", 0 },
	{ "rk628-dsi-v4l2", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, rk628_csi_i2c_id);

static const struct of_device_id rk628_csi_of_match[] = {
	{ .compatible = "rockchip,rk628-csi-v4l2", .data = &rk628_csi_data },
	{ .compatible = "rockchip,rk628-dsi-v4l2", .data = &rk628_dsi_data },
	{}
};
MODULE_DEVICE_TABLE(of, rk628_csi_of_match);

static bool tx_5v_power_present(struct v4l2_subdev *sd);

static ssize_t audio_rate_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct rk628_csi *csi = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d", rk628_hdmirx_audio_fs(csi->audio_info));
}

static ssize_t audio_present_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct rk628_csi *csi = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d",
			tx_5v_power_present(&csi->sd) ?
			rk628_hdmirx_audio_present(csi->audio_info) : 0);
}

static DEVICE_ATTR_RO(audio_rate);
static DEVICE_ATTR_RO(audio_present);

static struct attribute *rk628_attrs[] = {
	&dev_attr_audio_rate.attr,
	&dev_attr_audio_present.attr,
	NULL
};
ATTRIBUTE_GROUPS(rk628);

static int rk628_csi_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct rk628_csi *csi;
	struct v4l2_subdev *sd;
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	char facing[2];
	int err;
	u32 val;
	struct rk628 *rk628;
	unsigned long irq_flags;
	const struct of_device_id *match;

	dev_info(dev, "RK628 I2C driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	if (!of_device_is_available(dev->of_node))
		return -ENODEV;

	csi = devm_kzalloc(dev, sizeof(*csi), GFP_KERNEL);
	if (!csi)
		return -ENOMEM;

	csi->dev = dev;
	csi->i2c_client = client;
	rk628 = rk628_i2c_register(client);
	if (!rk628)
		return -ENOMEM;

	match = of_match_node(rk628_csi_of_match, dev->of_node);
	csi->plat_data = match->data;

	csi->rk628 = rk628;
	csi->dsi.rk628 = rk628;
	csi->cur_mode = &supported_modes[0];
	csi->hdmirx_irq = client->irq;
	sd = &csi->sd;
	sd->dev = dev;

	csi->hpd_output_inverted = of_property_read_bool(node,
			"hpd-output-inverted");
	err = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &csi->module_index);
	err |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &csi->module_facing);
	err |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &csi->module_name);
	err |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &csi->len_name);
	if (err) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	err = rk628_csi_probe_of(csi);
	if (err) {
		v4l2_err(sd, "rk628_csi_probe_of failed! err:%d\n", err);
		return err;
	}

	rk628_cru_initialize(csi->rk628);

	v4l2_subdev_init(sd, &rk628_csi_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	/* i2c access, read chip id*/
	err = rk628_i2c_read(csi->rk628, CSITX_CSITX_VERSION, &val);
	if (err) {
		v4l2_err(sd, "i2c access failed! err:%d\n", err);
		return -ENODEV;
	}
	v4l2_dbg(1, debug, sd, "CSITX VERSION: %#x\n", val);

	mutex_init(&csi->confctl_mutex);

	csi->txphy = rk628_txphy_register(rk628);
	if (!csi->txphy) {
		v4l2_err(sd, "register txphy failed\n");
		return -ENOMEM;
	}

	/* control handlers */
	v4l2_ctrl_handler_init(&csi->hdl, 4);
	csi->link_freq = v4l2_ctrl_new_int_menu(&csi->hdl, NULL,
			V4L2_CID_LINK_FREQ,
			ARRAY_SIZE(link_freq_menu_items) - 1,
			0, link_freq_menu_items);
	csi->pixel_rate = v4l2_ctrl_new_std(&csi->hdl, NULL,
			V4L2_CID_PIXEL_RATE, 0, RK628_CSI_PIXEL_RATE_HIGH, 1,
			RK628_CSI_PIXEL_RATE_HIGH);
	csi->detect_tx_5v_ctrl = v4l2_ctrl_new_std(&csi->hdl,
			NULL, V4L2_CID_DV_RX_POWER_PRESENT,
			0, 1, 0, 0);

	/* custom controls */
	csi->audio_sampling_rate_ctrl = v4l2_ctrl_new_custom(&csi->hdl,
			&rk628_csi_ctrl_audio_sampling_rate, NULL);
	csi->audio_present_ctrl = v4l2_ctrl_new_custom(&csi->hdl,
			&rk628_csi_ctrl_audio_present, NULL);

	sd->ctrl_handler = &csi->hdl;
	if (csi->hdl.error) {
		err = csi->hdl.error;
		v4l2_err(sd, "cfg v4l2 ctrls failed! err:%d\n", err);
		goto err_hdl;
	}

	if (rk628_csi_update_controls(sd)) {
		err = -ENODEV;
		v4l2_err(sd, "update v4l2 ctrls failed! err:%d\n", err);
		goto err_hdl;
	}

	csi->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	err = media_entity_pads_init(&sd->entity, 1, &csi->pad);
	if (err < 0) {
		v4l2_err(sd, "media entity init failed! err:%d\n", err);
		goto err_hdl;
	}

	if (csi->plat_data->tx_mode == DSI_MODE)
		csi->mbus_fmt_code = MEDIA_BUS_FMT_RGB888_1X24;
	else
		csi->mbus_fmt_code = MEDIA_BUS_FMT_UYVY8_2X8;

	memset(facing, 0, sizeof(facing));
	if (strcmp(csi->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 csi->module_index, facing,
		 RK628_CSI_NAME, dev_name(sd->dev));
	err = v4l2_async_register_subdev(sd);
	if (err < 0) {
		v4l2_err(sd, "v4l2 register subdev failed! err:%d\n", err);
		goto err_hdl;
	}

	csi->classdev = device_create_with_groups(rk_hdmirx_class(),
						  dev, MKDEV(0, 0),
						  csi,
						  rk628_groups,
						  "rk628");
	if (IS_ERR(csi->classdev))
		goto err_hdl;

	INIT_DELAYED_WORK(&csi->delayed_work_enable_hotplug,
			rk628_csi_delayed_work_enable_hotplug);
	INIT_DELAYED_WORK(&csi->delayed_work_res_change,
			rk628_delayed_work_res_change);
	csi->audio_info = rk628_hdmirx_audioinfo_alloc(dev,
						       &csi->confctl_mutex,
						       rk628,
						       csi->i2s_enable_default);
	if (!csi->audio_info) {
		v4l2_err(sd, "request audio info fail\n");
		goto err_work_queues;
	}
	rk628_csi_initial_setup(sd);

	if (csi->hdmirx_irq) {
		irq_flags = irqd_get_trigger_type(irq_get_irq_data(csi->hdmirx_irq));
		v4l2_dbg(1, debug, sd, "cfg hdmirx irq, flags: %lu!\n", irq_flags);
		err = devm_request_threaded_irq(dev, csi->hdmirx_irq, NULL,
				rk628_csi_irq_handler, irq_flags |
				IRQF_ONESHOT, "rk628_csi", csi);
		if (err) {
			v4l2_err(sd, "request rk628-csi irq failed! err:%d\n",
					err);
			goto err_work_queues;
		}
	} else {
		v4l2_dbg(1, debug, sd, "no irq, cfg poll!\n");
		INIT_WORK(&csi->work_i2c_poll,
			  rk628_csi_work_i2c_poll);
		timer_setup(&csi->timer, rk628_csi_irq_poll_timer, 0);
		csi->timer.expires = jiffies +
				       msecs_to_jiffies(POLL_INTERVAL_MS);
		add_timer(&csi->timer);
	}

	if (csi->plugin_det_gpio) {
		csi->plugin_irq = gpiod_to_irq(csi->plugin_det_gpio);
		if (csi->plugin_irq < 0) {
			dev_err(csi->dev, "failed to get plugin det irq\n");
			err = csi->plugin_irq;
			goto err_work_queues;
		}

		err = devm_request_threaded_irq(dev, csi->plugin_irq, NULL,
				plugin_detect_irq, IRQF_TRIGGER_FALLING |
				IRQF_TRIGGER_RISING | IRQF_ONESHOT, "rk628_csi", csi);
		if (err) {
			dev_err(csi->dev, "failed to register plugin det irq (%d)\n", err);
			goto err_work_queues;
		}
	}

	err = v4l2_ctrl_handler_setup(sd->ctrl_handler);
	if (err) {
		v4l2_err(sd, "v4l2 ctrl handler setup failed! err:%d\n", err);
		goto err_work_queues;
	}

	v4l2_info(sd, "%s found @ 0x%x (%s)\n", client->name,
		  client->addr << 1, client->adapter->name);

	return 0;

err_work_queues:
	if (!csi->hdmirx_irq)
		flush_work(&csi->work_i2c_poll);
	cancel_delayed_work(&csi->delayed_work_enable_hotplug);
	cancel_delayed_work(&csi->delayed_work_res_change);
	rk628_hdmirx_audio_destroy(csi->audio_info);
err_hdl:
	mutex_destroy(&csi->confctl_mutex);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&csi->hdl);
	return err;
}

static int rk628_csi_remove(struct i2c_client *client)
{
	struct rk628_csi *csi = i2c_get_clientdata(client);

	if (!csi->hdmirx_irq) {
		del_timer_sync(&csi->timer);
		flush_work(&csi->work_i2c_poll);
	}
	rk628_hdmirx_audio_cancel_work_audio(csi->audio_info, true);
	rk628_hdmirx_audio_cancel_work_rate_change(csi->audio_info, true);
	cancel_delayed_work_sync(&csi->delayed_work_enable_hotplug);
	cancel_delayed_work_sync(&csi->delayed_work_res_change);

	if (csi->rxphy_pwron)
		rk628_rxphy_power_off(csi->rk628);
	if (csi->txphy_pwron)
		mipi_dphy_power_off(csi);

	mutex_destroy(&csi->confctl_mutex);

	rk628_control_assert(csi->rk628, RGU_HDMIRX);
	rk628_control_assert(csi->rk628, RGU_HDMIRX_PON);
	rk628_control_assert(csi->rk628, RGU_DECODER);
	rk628_control_assert(csi->rk628, RGU_CLK_RX);
	rk628_control_assert(csi->rk628, RGU_VOP);
	rk628_control_assert(csi->rk628, RGU_CSI);

	return 0;
}

static struct i2c_driver rk628_csi_i2c_driver = {
	.driver = {
		.name = "rk628-csi-v4l2",
		.of_match_table = of_match_ptr(rk628_csi_of_match),
	},
	.id_table = rk628_csi_i2c_id,
	.probe	= rk628_csi_probe,
	.remove = rk628_csi_remove,
};

module_i2c_driver(rk628_csi_i2c_driver);

MODULE_DESCRIPTION("Rockchip RK628 HDMI to MIPI CSI-2 bridge I2C driver");
MODULE_AUTHOR("Dingxian Wen <shawn.wen@rock-chips.com>");
MODULE_AUTHOR("Shunqing Chen <csq@rock-chips.com>");
MODULE_LICENSE("GPL");
