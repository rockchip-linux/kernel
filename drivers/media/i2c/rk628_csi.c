// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Rockchip Electronics Co. Ltd.
 *
 * Author: Dingxian Wen <shawn.wen@rock-chips.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/mfd/rk628.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/rk-camera-module.h>
#include <linux/soc/rockchip/rk_vendor_storage.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <media/v4l2-controls_rockchip.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <video/videomode.h>
#include "rk628_csi.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x0, 0x9)
#define RK628_CSI_NAME			"rk628-csi"

#define EDID_NUM_BLOCKS_MAX 		2
#define EDID_BLOCK_SIZE 		128

#define RK628_CSI_LINK_FREQ_LOW		350000000
#define RK628_CSI_LINK_FREQ_HIGH	400000000
#define RK628_CSI_PIXEL_RATE_LOW	400000000
#define RK628_CSI_PIXEL_RATE_HIGH	600000000
#define MIPI_DATARATE_MBPS_LOW		750
#define MIPI_DATARATE_MBPS_HIGH		1250

#define POLL_INTERVAL_MS		1000
#define MODETCLK_CNT_NUM		1000
#define MODETCLK_HZ			49500000
#define RXPHY_CFG_MAX_TIMES		3
#define CSITX_ERR_RETRY_TIMES		3

#define HDCP_KEY_KSV_SIZE		8
#define HDCP_PRIVATE_KEY_SIZE		280
#define HDCP_KEY_SHA_SIZE		20
#define HDCP_KEY_SIZE			308
#define HDCP_KEY_SEED_SIZE		2
#define KSV_LEN				5

#define HDMIRX_HDCP1X_ID		13

#define YUV422_8BIT			0x1e
/* Test Code: 0x44 (HS RX Control of Lane 0) */
#define HSFREQRANGE(x)			UPDATE(x, 6, 1)
#define INIT_FIFO_STATE			64

struct hdcp_keys {
	u8 KSV[HDCP_KEY_KSV_SIZE];
	u8 devicekey[HDCP_PRIVATE_KEY_SIZE];
	u8 sha[HDCP_KEY_SHA_SIZE];
};

struct rk628_hdcp {
	char *seeds;
	struct hdcp_keys *keys;
};

struct rk628_audiostate {
	u32 hdmirx_aud_clkrate;
	u32 fs_audio;
	u32 ctsn_flag;
	u32 fifo_flag;
	int init_state;
	int pre_state;
	bool fifo_int;
};

#define is_validfs(x) (x == 32000 || \
			x == 44100 || \
			x == 48000 || \
			x == 88200 || \
			x == 96000 || \
			x == 176400 || \
			x == 192000 || \
			x == 768000)

struct rk628_csi {
	struct device *dev;
	struct rk628 *parent;
	struct i2c_client *i2c_client;
	struct media_pad pad;
	struct v4l2_subdev sd;
	struct v4l2_dv_timings timings;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *detect_tx_5v_ctrl;
	struct v4l2_ctrl *audio_sampling_rate_ctrl;
	struct v4l2_ctrl *audio_present_ctrl;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *power_gpio;
	struct gpio_desc *plugin_det_gpio;
	struct gpio_desc *hpd_gpio;
	struct reset_control *rst_hdmirx;
	struct reset_control *rst_hdmirx_pon;
	struct reset_control *rst_decoder;
	struct reset_control *rst_clk_rx;
	struct reset_control *rst_vop;
	struct reset_control *rst_csi0;
	struct clk *clk_hdmirx;
	struct clk *clk_imodet;
	struct clk *clk_hdmirx_aud;
	struct clk *clk_hdmirx_cec;
	struct clk *clk_vop;
	struct clk *clk_rx_read;
	struct clk *clk_csi0;
	struct clk *clk_i2s_mclk;
	struct regmap *grf;
	struct regmap *rxphy_regmap;
	struct regmap *hdmirx_regmap;
	struct regmap *key_regmap;
	struct regmap *csi_regmap;
	struct delayed_work delayed_work_enable_hotplug;
	struct delayed_work delayed_work_res_change;
	struct delayed_work delayed_work_audio_rate_change;
	struct delayed_work delayed_work_audio;
	struct timer_list timer;
	struct work_struct work_i2c_poll;
	struct phy *rxphy;
	struct phy *txphy;
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
	int hdmirx_irq;
	int plugin_irq;
	bool nosignal;
	bool rxphy_pwron;
	bool txphy_pwron;
	bool enable_hdcp;
	bool audio_present;
	bool hpd_output_inverted;
	bool avi_rcv_rdy;
	bool ctsn_ints_en;
	bool vid_ints_en;
	bool fifo_ints_en;
	struct rk628_hdcp hdcp;
	struct rk628_audiostate audio_state;
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
	V4L2_INIT_BT_TIMINGS(1, 10000, 1, 10000, 0, 400000000,
			V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			V4L2_DV_BT_STD_GTF | V4L2_DV_BT_STD_CVT,
			V4L2_DV_BT_CAP_PROGRESSIVE | V4L2_DV_BT_CAP_INTERLACED |
			V4L2_DV_BT_CAP_REDUCED_BLANKING |
			V4L2_DV_BT_CAP_CUSTOM)
};

static u8 edid_init_data[] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	0x52, 0x62, 0x01, 0x88, 0x00, 0x88, 0x88, 0x88,
	0x1C, 0x15, 0x01, 0x03, 0x80, 0x00, 0x00, 0x78,
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
	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x7B,

	0x02, 0x03, 0x1A, 0x71, 0x47, 0x5F, 0x90, 0x22,
	0x04, 0x11, 0x02, 0x01, 0x23, 0x09, 0x07, 0x01,
	0x83, 0x01, 0x00, 0x00, 0x65, 0x03, 0x0C, 0x00,
	0x10, 0x00, 0x8C, 0x0A, 0xD0, 0x8A, 0x20, 0xE0,
	0x2D, 0x10, 0x10, 0x3E, 0x96, 0x00, 0x13, 0x8E,
	0x21, 0x00, 0x00, 0x1E, 0xD8, 0x09, 0x80, 0xA0,
	0x20, 0xE0, 0x2D, 0x10, 0x10, 0x60, 0xA2, 0x00,
	0xC4, 0x8E, 0x21, 0x00, 0x00, 0x18, 0x8C, 0x0A,
	0xD0, 0x90, 0x20, 0x40, 0x31, 0x20, 0x0C, 0x40,
	0x55, 0x00, 0x48, 0x39, 0x00, 0x00, 0x00, 0x18,
	0x01, 0x1D, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40,
	0x58, 0x2C, 0x45, 0x00, 0xC0, 0x6C, 0x00, 0x00,
	0x00, 0x18, 0x01, 0x1D, 0x80, 0x18, 0x71, 0x1C,
	0x16, 0x20, 0x58, 0x2C, 0x25, 0x00, 0xC0, 0x6C,
	0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD8,
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

static void rk628_post_process_setup(struct v4l2_subdev *sd);
static void rk628_csi_enable_interrupts(struct v4l2_subdev *sd, bool en);
static int rk628_csi_s_ctrl_detect_tx_5v(struct v4l2_subdev *sd);
static int rk628_csi_s_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings);
static int rk628_csi_s_edid(struct v4l2_subdev *sd,
				struct v4l2_subdev_edid *edid);
static int mipi_dphy_power_on(struct rk628_csi *csi);
static int mipi_dphy_reset(struct rk628_csi *csi);
static void mipi_dphy_power_off(struct rk628_csi *csi);
static void mipi_dphy_init_hsfreqrange(struct rk628_csi *csi);
static int rk628_hdmirx_phy_power_on(struct v4l2_subdev *sd);
static int rk628_hdmirx_phy_power_off(struct v4l2_subdev *sd);
static int rk628_hdmirx_phy_setup(struct v4l2_subdev *sd);
static void rk628_hdmirx_controller_setup(struct v4l2_subdev *sd);
static void rk628_csi_format_change(struct v4l2_subdev *sd);
static void enable_stream(struct v4l2_subdev *sd, bool enable);
static void rk628_hdmirx_audio_setup(struct v4l2_subdev *sd);
static void rk628_hdmirx_vid_enable(struct v4l2_subdev *sd, bool en);
static void rk628_csi_set_csi(struct v4l2_subdev *sd);
static void rk628_hdmirx_hpd_ctrl(struct v4l2_subdev *sd, bool en);
static void rk628_csi_set_hdmi_hdcp(struct v4l2_subdev *sd, bool en);
static void rk628_hdmirx_controller_reset(struct v4l2_subdev *sd);
static bool rk628_rcv_supported_res(struct v4l2_subdev *sd, u32 width,
				    u32 height);

static inline struct rk628_csi *to_csi(struct v4l2_subdev *sd)
{
	return container_of(sd, struct rk628_csi, sd);
}

static bool tx_5v_power_present(struct v4l2_subdev *sd)
{
	bool ret;
	int val, i, cnt;
	struct rk628_csi *csi = to_csi(sd);

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

	return csi->audio_present;
}

static int get_audio_sampling_rate(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);

	if (no_signal(sd))
		return 0;

	return csi->audio_state.fs_audio;
}

static void rk628_hdmirx_ctrl_enable(struct v4l2_subdev *sd, int en)
{
	u32 mask;
	struct rk628_csi *csi = to_csi(sd);

	if (en) {
		/* don't enable audio until N CTS updated */
		mask = HDMI_ENABLE_MASK;
		v4l2_dbg(1, debug, sd, "%s: %#x %d\n", __func__, mask, en);
		regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_DMI_DISABLE_IF,
				   mask, HDMI_ENABLE(1) | AUD_ENABLE(1));
	} else {
		mask = AUD_ENABLE_MASK | HDMI_ENABLE_MASK;
		v4l2_dbg(1, debug, sd, "%s: %#x %d\n", __func__, mask, en);
		regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_DMI_DISABLE_IF,
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

	memset(timings, 0, sizeof(struct v4l2_dv_timings));
	timings->type = V4L2_DV_BT_656_1120;
	regmap_read(csi->hdmirx_regmap, HDMI_RX_SCDC_REGS1, &val);
	status = val;

	regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_STS, &val);
	bt->interlaced = val & ILACE_STS ?
		V4L2_DV_INTERLACED : V4L2_DV_PROGRESSIVE;

	regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_HACT_PX, &val);
	hact = val & 0xffff;
	regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_VAL, &val);
	vact = val & 0xffff;
	regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_HT1, &val);
	htotal = (val >> 16) & 0xffff;
	regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_VTL, &val);
	vtotal = val & 0xffff;
	regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_HT1, &val);
	hofs_pix = val & 0xffff;
	regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_VOL, &val);
	vbp = (val & 0xffff) + 1;

	regmap_read(csi->hdmirx_regmap, HDMI_RX_HDMI_CKM_RESULT, &val);
	tmdsclk_cnt = val & 0xffff;
	tmp_data = tmdsclk_cnt;
	tmp_data = ((tmp_data * MODETCLK_HZ) + MODETCLK_CNT_NUM / 2);
	do_div(tmp_data, MODETCLK_CNT_NUM);
	tmds_clk = tmp_data;
	if (!(htotal * vtotal)) {
		v4l2_err(sd, "timing err, htotal:%d, vtotal:%d\n",
				htotal, vtotal);
		goto TIMING_ERR;
	}
	fps = (tmds_clk + (htotal * vtotal) / 2) / (htotal * vtotal);

	regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_HT0, &val);
	modetclk_cnt_hs = val & 0xffff;
	hs = (tmdsclk_cnt * modetclk_cnt_hs + MODETCLK_CNT_NUM / 2) /
		MODETCLK_CNT_NUM;

	regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_VSC, &val);
	modetclk_cnt_vs = val & 0xffff;
	vs = (tmdsclk_cnt * modetclk_cnt_vs + MODETCLK_CNT_NUM / 2) /
		MODETCLK_CNT_NUM;
	vs = (vs + htotal / 2) / htotal;

	if ((hofs_pix < hs) || (htotal < (hact + hofs_pix)) ||
			(vtotal < (vact + vs + vbp))) {
		v4l2_err(sd, "timing err, total:%dx%d, act:%dx%d, hofs:%d, "
				"hs:%d, vs:%d, vbp:%d\n", htotal, vtotal, hact,
				vact, hofs_pix, hs, vs, vbp);
		goto TIMING_ERR;
	}
	hbp = hofs_pix - hs;
	hfp = htotal - hact - hofs_pix;
	vfp = vtotal - vact - vs - vbp;

	v4l2_dbg(2, debug, sd, "cnt_num:%d, tmds_cnt:%d, hs_cnt:%d, vs_cnt:%d,"
			" hofs:%d\n", MODETCLK_CNT_NUM, tmdsclk_cnt,
			modetclk_cnt_hs, modetclk_cnt_vs, hofs_pix);

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

	v4l2_dbg(1, debug, sd, "SCDC_REGS1:%#x, act:%dx%d, total:%dx%d, fps:%d,"
			" pixclk:%llu\n", status, hact, vact, htotal, vtotal,
			fps, bt->pixelclock);
	v4l2_dbg(1, debug, sd, "hfp:%d, hs:%d, hbp:%d, vfp:%d, vs:%d, vbp:%d,"
			" interlace:%d\n", bt->hfrontporch, bt->hsync,
			bt->hbackporch, bt->vfrontporch, bt->vsync,
			bt->vbackporch, bt->interlaced);
	return 0;

TIMING_ERR:
	return -ENOLCK;
}

static void rk628_hdmirx_config_all(struct v4l2_subdev *sd)
{
	int ret;
	struct rk628_csi *csi = to_csi(sd);

	rk628_hdmirx_controller_setup(sd);
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
	v4l2_dbg(1, debug, sd, "%s: 5v_det:%d\n", __func__, plugin);
	if (plugin) {
		rk628_csi_enable_interrupts(sd, false);
		rk628_hdmirx_audio_setup(sd);
		rk628_csi_set_hdmi_hdcp(sd, csi->enable_hdcp);
		rk628_hdmirx_hpd_ctrl(sd, true);
		rk628_hdmirx_config_all(sd);
		rk628_csi_enable_interrupts(sd, true);
		regmap_update_bits(csi->grf, GRF_SYSTEM_CON0,
				SW_I2S_DATA_OEN_MASK, SW_I2S_DATA_OEN(0));
	} else {
		rk628_csi_enable_interrupts(sd, false);
		enable_stream(sd, false);
		cancel_delayed_work(&csi->delayed_work_res_change);
		cancel_delayed_work(&csi->delayed_work_audio);
		rk628_hdmirx_hpd_ctrl(sd, false);
		rk628_hdmirx_phy_power_off(sd);
		rk628_hdmirx_controller_reset(sd);
		csi->nosignal = true;
	}
	mutex_unlock(&csi->confctl_mutex);
}

static int rk628_check_resulotion_change(struct v4l2_subdev *sd)
{
	u32 val;
	struct rk628_csi *csi = to_csi(sd);
	u32 htotal, vtotal;
	u32 old_htotal, old_vtotal;
	struct v4l2_bt_timings *bt = &csi->timings.bt;

	regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_HT1, &val);
	htotal = (val >> 16) & 0xffff;
	regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_VTL, &val);
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
			cancel_delayed_work_sync(&csi->delayed_work_audio);
			rk628_hdmirx_phy_power_off(sd);
			rk628_hdmirx_controller_reset(sd);
			rk628_hdmirx_audio_setup(sd);
			rk628_csi_set_hdmi_hdcp(sd, csi->enable_hdcp);
			rk628_hdmirx_hpd_ctrl(sd, true);
			rk628_hdmirx_config_all(sd);
			rk628_csi_enable_interrupts(sd, true);
			regmap_update_bits(csi->grf, GRF_SYSTEM_CON0,
					   SW_I2S_DATA_OEN_MASK, SW_I2S_DATA_OEN(0));
		} else {
			rk628_csi_format_change(sd);
			csi->nosignal = false;
			rk628_csi_enable_interrupts(sd, true);
		}
	}
	mutex_unlock(&csi->confctl_mutex);
}

static int hdcp_load_keys_cb(struct rk628_csi *csi)
{
	struct rk628_hdcp *hdcp = &csi->hdcp;
	int size;
	u8 hdcp_vendor_data[320];

	hdcp->keys = kmalloc(HDCP_KEY_SIZE, GFP_KERNEL);
	if (!hdcp->keys)
		return -ENOMEM;

	hdcp->seeds = kmalloc(HDCP_KEY_SEED_SIZE, GFP_KERNEL);
	if (!hdcp->seeds) {
		kfree(hdcp->keys);
		hdcp->keys = NULL;
		return -ENOMEM;
	}

	size = rk_vendor_read(HDMIRX_HDCP1X_ID, hdcp_vendor_data, 314);
	if (size < (HDCP_KEY_SIZE + HDCP_KEY_SEED_SIZE)) {
		dev_dbg(csi->dev, "HDCP: read size %d\n", size);
		kfree(hdcp->keys);
		hdcp->keys = NULL;
		kfree(hdcp->seeds);
		hdcp->seeds = NULL;
		return -EINVAL;
	}
	memcpy(hdcp->keys, hdcp_vendor_data, HDCP_KEY_SIZE);
	memcpy(hdcp->seeds, hdcp_vendor_data + HDCP_KEY_SIZE,
	       HDCP_KEY_SEED_SIZE);

	return 0;
}

static int rk628_hdmi_hdcp_load_key(struct rk628_csi *csi)
{
	int i;
	int ret;
	struct hdcp_keys *hdcp_keys;
	struct rk628_hdcp *hdcp = &csi->hdcp;
	u32 seeds = 0;

	if (!hdcp->keys) {
		ret = hdcp_load_keys_cb(csi);
		if (ret) {
			dev_err(csi->dev, "HDCP: load key failed\n");
			return ret;
		}
	}
	hdcp_keys = hdcp->keys;

	regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_HDCP_CTRL,
			HDCP_ENABLE_MASK |
			HDCP_ENC_EN_MASK,
			HDCP_ENABLE(0) |
			HDCP_ENC_EN(0));
	regmap_update_bits(csi->grf, GRF_SYSTEM_CON0,
			SW_ADAPTER_I2CSLADR_MASK |
			SW_EFUSE_HDCP_EN_MASK,
			SW_ADAPTER_I2CSLADR(0) |
			SW_EFUSE_HDCP_EN(1));
	/* The useful data in ksv should be 5 byte */
	for (i = 0; i < KSV_LEN; i++)
		regmap_write(csi->key_regmap, HDCP_KEY_KSV0 + i * 4,
			     hdcp_keys->KSV[i]);

	for (i = 0; i < HDCP_PRIVATE_KEY_SIZE; i++)
		regmap_write(csi->key_regmap, HDCP_KEY_DPK0 + i * 4,
			     hdcp_keys->devicekey[i]);

	regmap_update_bits(csi->grf, GRF_SYSTEM_CON0,
			SW_ADAPTER_I2CSLADR_MASK |
			SW_EFUSE_HDCP_EN_MASK,
			SW_ADAPTER_I2CSLADR(0) |
			SW_EFUSE_HDCP_EN(0));
	regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_HDCP_CTRL,
			HDCP_ENABLE_MASK |
			HDCP_ENC_EN_MASK,
			HDCP_ENABLE(1) |
			HDCP_ENC_EN(1));

	/* Enable decryption logic */
	if (hdcp->seeds) {
		seeds = (hdcp->seeds[0] & 0xff) << 8;
		seeds |= (hdcp->seeds[1] & 0xff);
	}
	if (seeds) {
		regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_HDCP_CTRL,
				   KEY_DECRIPT_ENABLE_MASK,
				   KEY_DECRIPT_ENABLE(1));
		regmap_write(csi->hdmirx_regmap, HDMI_RX_HDCP_SEED, seeds);
	} else {
		regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_HDCP_CTRL,
				   KEY_DECRIPT_ENABLE_MASK,
				   KEY_DECRIPT_ENABLE(0));
	}

	return 0;
}

static void rk628_csi_set_hdmi_hdcp(struct v4l2_subdev *sd, bool en)
{
	struct rk628_csi *csi = to_csi(sd);

	v4l2_dbg(1, debug, sd, "%s: %sable\n", __func__, en ? "en" : "dis");

	if (en) {
		rk628_hdmi_hdcp_load_key(csi);
	} else {
		regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_HDCP_CTRL,
				HDCP_ENABLE_MASK |
				HDCP_ENC_EN_MASK,
				HDCP_ENABLE(0) |
				HDCP_ENC_EN(0));
	}
}

static void rk628_hdmirx_hpd_ctrl(struct v4l2_subdev *sd, bool en)
{
	u8 en_level, set_level;
	struct rk628_csi *csi = to_csi(sd);

	v4l2_dbg(1, debug, sd, "%s: %sable, hpd invert:%d\n", __func__,
			en ? "en" : "dis", csi->hpd_output_inverted);
	en_level = csi->hpd_output_inverted ? 0 : 1;
	set_level = en ? en_level : !en_level;
	regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_HDMI_SETUP_CTRL,
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

	reset_control_assert(csi->rst_csi0);
	udelay(10);
	reset_control_deassert(csi->rst_csi0);

	regmap_write(csi->csi_regmap, CSITX_SYS_CTRL0_IMD, 0x1);
	usleep_range(1000, 1000);
	regmap_write(csi->csi_regmap, CSITX_SYS_CTRL0_IMD, 0x0);
}

static void enable_csitx(struct v4l2_subdev *sd)
{
	u32 i, ret, val;
	struct rk628_csi *csi = to_csi(sd);

	for (i = 0; i < CSITX_ERR_RETRY_TIMES; i++) {
		rk628_csi_set_csi(sd);
		regmap_update_bits(csi->csi_regmap, CSITX_CSITX_EN,
					DPHY_EN_MASK |
					CSITX_EN_MASK,
					DPHY_EN(1) |
					CSITX_EN(1));
		regmap_write(csi->csi_regmap, CSITX_CONFIG_DONE, CONFIG_DONE_IMD);
		msleep(40);
		regmap_write(csi->csi_regmap, CSITX_ERR_INTR_CLR_IMD, 0xffffffff);
		regmap_update_bits(csi->csi_regmap, CSITX_SYS_CTRL1,
				BYPASS_SELECT_MASK, BYPASS_SELECT(0));
		regmap_write(csi->csi_regmap, CSITX_CONFIG_DONE, CONFIG_DONE_IMD);
		msleep(40);
		ret = regmap_read(csi->csi_regmap,
				CSITX_ERR_INTR_RAW_STATUS_IMD, &val);
		if (!ret && !val)
			break;

		v4l2_err(sd, "%s csitx err, retry:%d, err status:%#x, ret:%d\n",
				__func__, i, val, ret);
	}

}

static void enable_stream(struct v4l2_subdev *sd, bool en)
{
	struct rk628_csi *csi = to_csi(sd);

	v4l2_dbg(1, debug, sd, "%s: %sable\n", __func__, en ? "en" : "dis");
	if (en) {
		rk628_hdmirx_vid_enable(sd, true);
		enable_csitx(sd);
	} else {
		rk628_hdmirx_vid_enable(sd, false);
		regmap_update_bits(csi->csi_regmap, CSITX_CSITX_EN,
					DPHY_EN_MASK |
					CSITX_EN_MASK,
					DPHY_EN(0) |
					CSITX_EN(0));
		regmap_write(csi->csi_regmap, CSITX_CONFIG_DONE, CONFIG_DONE_IMD);
	}
}

static void calc_dsp_frm_hst_vst(const struct videomode *src,
				 const struct videomode *dst,
				 u32 *dsp_frame_hst, u32 *dsp_frame_vst)
{
	u32 bp_in, bp_out;
	u32 v_scale_ratio;
	u64 t_frm_st;
	u64 t_bp_in, t_bp_out, t_delta, tin;
	u32 src_pixclock, dst_pixclock;
	u32 dsp_htotal, src_htotal, src_vtotal;

	src_pixclock = div_u64(1000000000000llu, src->pixelclock);
	dst_pixclock = div_u64(1000000000000llu, dst->pixelclock);

	src_htotal = src->hsync_len + src->hback_porch + src->hactive +
		     src->hfront_porch;
	src_vtotal = src->vsync_len + src->vback_porch + src->vactive +
		     src->vfront_porch;
	dsp_htotal = dst->hsync_len + dst->hback_porch + dst->hactive +
		     dst->hfront_porch;

	bp_in = (src->vback_porch + src->vsync_len) * src_htotal +
		src->hsync_len + src->hback_porch;
	bp_out = (dst->vback_porch + dst->vsync_len) * dsp_htotal +
		 dst->hsync_len + dst->hback_porch;

	t_bp_in = bp_in * src_pixclock;
	t_bp_out = bp_out * dst_pixclock;
	tin = src_vtotal * src_htotal * src_pixclock;

	v_scale_ratio = src->vactive / dst->vactive;
	if (v_scale_ratio <= 2)
		t_delta = 5 * src_htotal * src_pixclock;
	else
		t_delta = 12 * src_htotal * src_pixclock;

	if (t_bp_in + t_delta > t_bp_out)
		t_frm_st = (t_bp_in + t_delta - t_bp_out);
	else
		t_frm_st = tin - (t_bp_out - (t_bp_in + t_delta));

	do_div(t_frm_st, src_pixclock);
	*dsp_frame_hst = do_div(t_frm_st, src_htotal);
	*dsp_frame_vst = t_frm_st;
}

static void rk628_post_process_scaler_init(struct v4l2_subdev *sd,
					   const struct videomode *src,
					   const struct videomode *dst)
{
	struct rk628_csi *csi = to_csi(sd);
	u32 dsp_frame_hst, dsp_frame_vst;
	u32 scl_hor_mode, scl_ver_mode;
	u32 scl_v_factor, scl_h_factor;
	u32 dsp_htotal, dsp_hs_end, dsp_hact_st, dsp_hact_end;
	u32 dsp_vtotal, dsp_vs_end, dsp_vact_st, dsp_vact_end;
	u32 dsp_hbor_end, dsp_hbor_st, dsp_vbor_end, dsp_vbor_st;
	u16 bor_right = 0, bor_left = 0, bor_up = 0, bor_down = 0;
	u8 hor_down_mode = 0, ver_down_mode = 0;

	dsp_htotal = dst->hsync_len + dst->hback_porch + dst->hactive +
		     dst->hfront_porch;
	dsp_vtotal = dst->vsync_len + dst->vback_porch + dst->vactive +
		     dst->vfront_porch;
	dsp_hs_end = dst->hsync_len;
	dsp_vs_end = dst->vsync_len;
	dsp_hbor_end = dst->hsync_len + dst->hback_porch + dst->hactive;
	dsp_hbor_st = dst->hsync_len + dst->hback_porch;
	dsp_vbor_end = dst->vsync_len + dst->vback_porch + dst->vactive;
	dsp_vbor_st = dst->vsync_len + dst->vback_porch;
	dsp_hact_st = dsp_hbor_st + bor_left;
	dsp_hact_end = dsp_hbor_end - bor_right;
	dsp_vact_st = dsp_vbor_st + bor_up;
	dsp_vact_end = dsp_vbor_end - bor_down;

	calc_dsp_frm_hst_vst(src, dst, &dsp_frame_hst, &dsp_frame_vst);
	v4l2_dbg(1, debug, sd, "dsp_frame_vst=%d, dsp_frame_hst=%d\n",
		dsp_frame_vst, dsp_frame_hst);

	if (src->hactive > dst->hactive) {
		scl_hor_mode = 2;

		if (hor_down_mode == 0) {
			if ((src->hactive - 1) / (dst->hactive - 1) > 2)
				scl_h_factor = ((src->hactive - 1) << 14) /
					       (dst->hactive - 1);
			else
				scl_h_factor = ((src->hactive - 2) << 14) /
					       (dst->hactive - 1);
		} else {
			scl_h_factor = (dst->hactive << 16) /
				       (src->hactive - 1);
		}

		v4l2_dbg(1, debug, sd, "horizontal scale down\n");
	} else if (src->hactive == dst->hactive) {
		scl_hor_mode = 0;
		scl_h_factor = 0;

		v4l2_dbg(1, debug, sd, "horizontal no scale\n");
	} else {
		scl_hor_mode = 1;
		scl_h_factor = ((src->hactive - 1) << 16) / (dst->hactive - 1);

		v4l2_dbg(1, debug, sd, "horizontal scale up\n");
	}

	if (src->vactive > dst->vactive) {
		scl_ver_mode = 2;

		if (ver_down_mode == 0) {
			if ((src->vactive - 1) / (dst->vactive - 1) > 2)
				scl_v_factor = ((src->vactive - 1) << 14) /
					       (dst->vactive - 1);
			else
				scl_v_factor = ((src->vactive - 2) << 14) /
					       (dst->vactive - 1);
		} else {
			scl_v_factor = (dst->vactive << 16) /
				       (src->vactive - 1);
		}

		v4l2_dbg(1, debug, sd, "vertical scale down\n");
	} else if (src->vactive == dst->vactive) {
		scl_ver_mode = 0;
		scl_v_factor = 0;

		v4l2_dbg(1, debug, sd, "vertical no scale\n");
	} else {
		scl_ver_mode = 1;
		scl_v_factor = ((src->vactive - 1) << 16) / (dst->vactive - 1);

		v4l2_dbg(1, debug, sd, "vertical scale up\n");
	}

	regmap_update_bits(csi->grf, GRF_RGB_DEC_CON0,
			   SW_HRES_MASK, SW_HRES(src->hactive));
	regmap_write(csi->grf, GRF_SCALER_CON0,
		     SCL_VER_DOWN_MODE(ver_down_mode) |
		     SCL_HOR_DOWN_MODE(hor_down_mode) |
		     SCL_VER_MODE(scl_ver_mode) | SCL_HOR_MODE(scl_hor_mode) |
		     SCL_EN(1));
	regmap_write(csi->grf, GRF_SCALER_CON1,
		     SCL_V_FACTOR(scl_v_factor) | SCL_H_FACTOR(scl_h_factor));
	regmap_write(csi->grf, GRF_SCALER_CON2,
		     DSP_FRAME_VST(dsp_frame_vst) |
		     DSP_FRAME_HST(dsp_frame_hst));
	regmap_write(csi->grf, GRF_SCALER_CON3,
		     DSP_HS_END(dsp_hs_end) | DSP_HTOTAL(dsp_htotal));
	regmap_write(csi->grf, GRF_SCALER_CON4,
		     DSP_HACT_END(dsp_hact_end) | DSP_HACT_ST(dsp_hact_st));
	regmap_write(csi->grf, GRF_SCALER_CON5,
		     DSP_VS_END(dsp_vs_end) | DSP_VTOTAL(dsp_vtotal));
	regmap_write(csi->grf, GRF_SCALER_CON6,
		     DSP_VACT_END(dsp_vact_end) | DSP_VACT_ST(dsp_vact_st));
	regmap_write(csi->grf, GRF_SCALER_CON7,
		     DSP_HBOR_END(dsp_hbor_end) | DSP_HBOR_ST(dsp_hbor_st));
	regmap_write(csi->grf, GRF_SCALER_CON8,
		     DSP_VBOR_END(dsp_vbor_end) | DSP_VBOR_ST(dsp_vbor_st));
}

static void rk628_post_process_setup(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);
	struct v4l2_bt_timings *bt = &csi->timings.bt;
	struct videomode src, dst;

	src.hactive = bt->width;
	src.hfront_porch = bt->hfrontporch;
	src.hsync_len = bt->hsync;
	src.hback_porch = bt->hbackporch;
	src.vactive = bt->height;
	src.vfront_porch = bt->vfrontporch;
	src.vsync_len = bt->vsync;
	src.vback_porch = bt->vbackporch;
	src.pixelclock = bt->pixelclock;
	if (!src.pixelclock) {
		enable_stream(sd, false);
		csi->nosignal = true;
		schedule_delayed_work(&csi->delayed_work_enable_hotplug, HZ / 20);
		return;
	}

	src.flags = 0;
	if (bt->interlaced == V4L2_DV_INTERLACED)
		src.flags |= DISPLAY_FLAGS_INTERLACED;

	/* do not scale now */
	dst = src;

	reset_control_assert(csi->rst_decoder);
	udelay(10);
	reset_control_deassert(csi->rst_decoder);
	udelay(10);

	clk_set_rate(csi->clk_rx_read, src.pixelclock);
	clk_prepare_enable(csi->clk_rx_read);
	reset_control_assert(csi->rst_clk_rx);
	udelay(10);
	reset_control_deassert(csi->rst_clk_rx);
	udelay(10);

	clk_set_rate(csi->clk_vop, dst.pixelclock);
	clk_prepare_enable(csi->clk_vop);
	reset_control_assert(csi->rst_vop);
	udelay(10);
	reset_control_deassert(csi->rst_vop);
	udelay(10);

	rk628_post_process_scaler_init(sd, &src, &dst);
}

static void rk628_csi_set_csi(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);
	u8 i, video_fmt;
	u8 lanes = csi->csi_lanes_in_use;
	u8 lane_num;
	u8 dphy_lane_en;
	u32 wc_usrdef, val, avi_pb = 0;
	u8 cnt = 0, max_cnt = 2;
	u32 hdcp_ctrl_val = 0;

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

	regmap_update_bits(csi->csi_regmap, CSITX_CSITX_EN,
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
	regmap_update_bits(csi->csi_regmap, CSITX_SYS_CTRL1,
			BYPASS_SELECT_MASK,
			BYPASS_SELECT(1));
	regmap_write(csi->csi_regmap, CSITX_CONFIG_DONE, CONFIG_DONE_IMD);
	regmap_write(csi->csi_regmap, CSITX_SYS_CTRL2,
			VOP_WHOLE_FRM_EN | VSYNC_ENABLE);
	regmap_update_bits(csi->csi_regmap, CSITX_SYS_CTRL3_IMD,
			CONT_MODE_CLK_CLR_MASK |
			CONT_MODE_CLK_SET_MASK |
			NON_CONTINOUS_MODE_MASK,
			CONT_MODE_CLK_CLR(0) |
			CONT_MODE_CLK_SET(0) |
			NON_CONTINOUS_MODE(1));

	regmap_write(csi->csi_regmap, CSITX_VOP_PATH_CTRL,
			VOP_WC_USERDEFINE(wc_usrdef) |
			VOP_DT_USERDEFINE(YUV422_8BIT) |
			VOP_PIXEL_FORMAT(0) |
			VOP_WC_USERDEFINE_EN(1) |
			VOP_DT_USERDEFINE_EN(1) |
			VOP_PATH_EN(1));
	regmap_update_bits(csi->csi_regmap, CSITX_DPHY_CTRL,
				CSI_DPHY_EN_MASK,
				CSI_DPHY_EN(dphy_lane_en));
	regmap_write(csi->csi_regmap, CSITX_CONFIG_DONE, CONFIG_DONE_IMD);
	v4l2_dbg(1, debug, sd, "%s csi cofig done\n", __func__);

	mutex_lock(&csi->confctl_mutex);
	regmap_read(csi->hdmirx_regmap, HDMI_RX_HDCP_CTRL, &val);
	if ((val & HDCP_ENABLE_MASK))
		max_cnt = 5;

	for (i = 0; i < 100; i++) {
		regmap_read(csi->hdmirx_regmap, HDMI_RX_PDEC_AVI_PB, &val);

		v4l2_dbg(2, debug, sd, "%s PDEC_AVI_PB:%#x, avi_rcv_rdy:%d\n",
			 __func__, val, csi->avi_rcv_rdy);
		if (i > 30 && !(hdcp_ctrl_val & 0x400)) {
			regmap_read(csi->hdmirx_regmap, HDMI_RX_HDCP_CTRL,
				    &hdcp_ctrl_val);
			/* force hdcp avmute */
			hdcp_ctrl_val |= 0x400;
			regmap_write(csi->csi_regmap, HDMI_RX_HDCP_CTRL,
				     hdcp_ctrl_val);
		}

		if (val && val == avi_pb && csi->avi_rcv_rdy) {
			if (++cnt >= max_cnt)
				break;
		} else {
			cnt = 0;
			avi_pb = val;
		}
		msleep(30);
	}
	mutex_unlock(&csi->confctl_mutex);

	video_fmt = (val & VIDEO_FORMAT_MASK) >> 5;
	v4l2_dbg(1, debug, sd, "%s PDEC_AVI_PB:%#x, video format:%d\n",
			__func__, val, video_fmt);
	if (video_fmt) {
		/* yuv data: cfg SW_YUV2VYU_SWP */
		regmap_write(csi->grf, GRF_CSC_CTRL_CON,
				SW_YUV2VYU_SWP(1) |
				SW_R2Y_EN(0));
	} else {
		/* rgb data: cfg SW_R2Y_EN */
		regmap_write(csi->grf, GRF_CSC_CTRL_CON,
				SW_YUV2VYU_SWP(0) |
				SW_R2Y_EN(1));
	}

	/* if avi packet is not stable, reset ctrl*/
	if (cnt < max_cnt)
		schedule_delayed_work(&csi->delayed_work_enable_hotplug, HZ / 20);
}

static int rk628_hdmirx_phy_power_on(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);
	int ret;

	/* Bit31 is used to distinguish HDMI cable mode and direct connection
	 * mode in the rk628_combrxphy driver.
	 * Bit31: 0 -direct connection mode;
	 *        1 -cable mode;
	 * The cable mode is to know the input clock frequency through cdr_mode
	 * in the rk628_combrxphy driver, and the cable mode supports up to
	 * 297M, so 297M is passed uniformly here.
	 */
	phy_set_bus_width(csi->rxphy, 297000 | BIT(31));

	if (csi->rxphy_pwron) {
		v4l2_dbg(1, debug, sd, "rxphy already power on, power off!\n");
		ret = phy_power_off(csi->rxphy);
		if (ret)
			v4l2_err(sd, "hdmi rxphy power off failed!\n");
		else
			csi->rxphy_pwron = false;
		usleep_range(100, 100);
	}

	if (csi->rxphy_pwron == false) {
		rk628_hdmirx_ctrl_enable(sd, 0);
		ret = phy_power_on(csi->rxphy);
		if (ret) {
			csi->rxphy_pwron = false;
			v4l2_err(sd, "hdmi rxphy power on failed\n");
		} else {
			csi->rxphy_pwron = true;
		}
		rk628_hdmirx_ctrl_enable(sd, 1);
		msleep(60);
	}

	return ret;
}

static int rk628_hdmirx_phy_power_off(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);
	int ret = 0;

	if (csi->rxphy_pwron) {
		v4l2_dbg(1, debug, sd, "rxphy power off!\n");
		ret = phy_power_off(csi->rxphy);
		if (ret)
			v4l2_err(sd, "hdmi rxphy power off failed!\n");
		else
			csi->rxphy_pwron = false;
	}
	usleep_range(100, 100);
	return ret;
}

static void rk628_hdmirx_vid_enable(struct v4l2_subdev *sd, bool en)
{
	struct rk628_csi *csi = to_csi(sd);

	v4l2_dbg(1, debug, sd, "%s: %sable\n", __func__, en ? "en" : "dis");
	if (en) {
		regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_DMI_DISABLE_IF,
				VID_ENABLE_MASK, VID_ENABLE(1));
	} else {
		regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_DMI_DISABLE_IF,
				VID_ENABLE_MASK, VID_ENABLE(0));
	}
}

static void rk628_hdmirx_controller_reset(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);

	v4l2_dbg(1, debug, sd, "%s reset hdmirx_controller\n", __func__);
	reset_control_assert(csi->rst_hdmirx_pon);
	reset_control_deassert(csi->rst_hdmirx_pon);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_DMI_SW_RST, 0x000101ff);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_DMI_DISABLE_IF, 0x00000000);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_DMI_DISABLE_IF, 0x0000017f);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_DMI_DISABLE_IF, 0x0001017f);
}

static void rk628_hdmirx_audio_fifo_init(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);

	v4l2_dbg(1, debug, sd, "%s initial fifo\n", __func__);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_ICLR, 0x1f);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_CTRL, 0x10001);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_CTRL, 0x10000);
	csi->audio_state.pre_state = csi->audio_state.init_state = INIT_FIFO_STATE*4;
}

static void rk628_hdmirx_audio_fifo_initd(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);

	v4l2_dbg(1, debug, sd, "%s double initial fifo\n", __func__);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_ICLR, 0x1f);
	regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_TH,
			   AFIF_TH_START_MASK,
			   AFIF_TH_START(192));
	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_CTRL, 0x10001);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_CTRL, 0x10000);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_CTRL, 0x10001);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_CTRL, 0x10000);
	regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_TH,
			   AFIF_TH_START_MASK,
			   AFIF_TH_START(INIT_FIFO_STATE));
	csi->audio_state.pre_state = csi->audio_state.init_state = INIT_FIFO_STATE*4;
}

static uint32_t rk628_hdmirx_audio_fs(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);
	u64 tmdsclk = 0;
	u32 clkrate = 0, cts_decoded = 0, n_decoded = 0, fs_audio = 0;

	/* fout=128*fs=ftmds*N/CTS */
	regmap_read(csi->hdmirx_regmap, HDMI_RX_HDMI_CKM_RESULT, &clkrate);
	clkrate = clkrate & 0xffff;
	/* tmdsclk = (clkrate/1000) * 49500000 */
	tmdsclk = clkrate * (49500000 / 1000);
	regmap_read(csi->hdmirx_regmap, HDMI_RX_PDEC_ACR_CTS, &cts_decoded);
	regmap_read(csi->hdmirx_regmap, HDMI_RX_PDEC_ACR_N, &n_decoded);
	if (cts_decoded != 0) {
		fs_audio = div_u64((tmdsclk * n_decoded), cts_decoded);
		fs_audio /= 128;
		fs_audio = div_u64(fs_audio + 50, 100);
		fs_audio *= 100;
	}
	v4l2_dbg(1, debug, sd,
		 "%s: clkrate:%u tmdsclk:%llu, n_decoded:%u, cts_decoded:%u, fs_audio:%u\n",
		 __func__, clkrate, tmdsclk, n_decoded, cts_decoded, fs_audio);
	if (!is_validfs(fs_audio))
		fs_audio = 0;
	return fs_audio;
}

static void rk628_hdmirx_audio_clk_set_rate(struct v4l2_subdev *sd, u32 rate)
{
	struct rk628_csi *csi = to_csi(sd);

	v4l2_dbg(2, debug, sd, "%s: %u to %u\n",
		 __func__, csi->audio_state.hdmirx_aud_clkrate, rate);
	clk_set_rate(csi->clk_hdmirx_aud, rate);
	csi->audio_state.hdmirx_aud_clkrate = rate;
}

static void rk628_hdmirx_audio_set_fs(struct v4l2_subdev *sd, u32 fs_audio)
{
	struct rk628_csi *csi = to_csi(sd);
	u32 hdmirx_aud_clkrate_t = fs_audio*128;

	v4l2_dbg(2, debug, sd, "%s: %u to %u with fs %u\n", __func__,
		 csi->audio_state.hdmirx_aud_clkrate, hdmirx_aud_clkrate_t,
		 fs_audio);
	clk_set_rate(csi->clk_hdmirx_aud, hdmirx_aud_clkrate_t);
	csi->audio_state.hdmirx_aud_clkrate = hdmirx_aud_clkrate_t;
	csi->audio_state.fs_audio = fs_audio;
}

static void rk628_hdmirx_audio_clk_ppm_inc(struct v4l2_subdev *sd, int ppm)
{
	struct rk628_csi *csi = to_csi(sd);
	int delta, rate, inc;

	rate = csi->audio_state.hdmirx_aud_clkrate;
	if (ppm < 0) {
		ppm = -ppm;
		inc = -1;
	} else
		inc = 1;
	delta = (int)div64_u64((uint64_t)rate * ppm + 500000, 1000000);
	delta *= inc;
	rate = csi->audio_state.hdmirx_aud_clkrate + delta;
	v4l2_dbg(2, debug, sd, "%s: %u to %u(delta:%d)\n",
		 __func__, csi->audio_state.hdmirx_aud_clkrate, rate, delta);
	clk_set_rate(csi->clk_hdmirx_aud, rate);
	csi->audio_state.hdmirx_aud_clkrate = rate;
}

static void rk628_hdmirx_audio_setup(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);
	u32 audio_pll_n, audio_pll_cts;

	audio_pll_n = 5644;
	audio_pll_cts = 148500;
	csi->audio_state.ctsn_flag = 0;
	csi->audio_state.fs_audio = 0;
	csi->audio_state.pre_state = 0;
	csi->audio_state.init_state = INIT_FIFO_STATE*4;
	csi->audio_state.fifo_int = false;
	csi->fifo_ints_en = false;
	csi->ctsn_ints_en = false;

	rk628_hdmirx_audio_clk_set_rate(sd, 5644800);
	/* manual aud CTS */
	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUDPLL_GEN_CTS, audio_pll_cts);
	/* manual aud N */
	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUDPLL_GEN_N, audio_pll_n);

	/* aud CTS N en manual */
	regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_AUD_CLK_CTRL,
			CTS_N_REF_MASK, CTS_N_REF(1));
	/* aud pll ctrl */
	regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_AUD_PLL_CTRL,
			PLL_LOCK_TOGGLE_DIV_MASK, PLL_LOCK_TOGGLE_DIV(0));
	regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_TH,
		AFIF_TH_START_MASK |
		AFIF_TH_MAX_MASK |
		AFIF_TH_MIN_MASK,
		AFIF_TH_START(64) |
		AFIF_TH_MAX(8) |
		AFIF_TH_MIN(8));

	/* AUTO_VMUTE */
	regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_CTRL,
			AFIF_SUBPACKET_DESEL_MASK |
			AFIF_SUBPACKETS_MASK,
			AFIF_SUBPACKET_DESEL(0) |
			AFIF_SUBPACKETS(1));
	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_SAO_CTRL,
			I2S_LPCM_BPCUV(0) |
			I2S_32_16(1));
	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_MUTE_CTRL,
			APPLY_INT_MUTE(0)	|
			APORT_SHDW_CTRL(3)	|
			AUTO_ACLK_MUTE(2)	|
			AUD_MUTE_SPEED(1)	|
			AUD_AVMUTE_EN(1)	|
			AUD_MUTE_SEL(0)		|
			AUD_MUTE_MODE(1));

	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_PAO_CTRL,
			PAO_RATE(0));
	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_CHEXTR_CTRL,
			AUD_LAYOUT_CTRL(1));
	csi->ctsn_ints_en = true;
	regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_IEN_SET, ACR_N_CHG_ICLR | ACR_CTS_CHG_ICLR);
	/* audio detect */
	regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_AUDIODET_CTRL,
			AUDIODET_THRESHOLD(0));
}

static void rk628_csi_delayed_work_audio(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct rk628_csi *csi = container_of(dwork, struct rk628_csi,
			delayed_work_audio);
	struct v4l2_subdev *sd = &csi->sd;
	struct rk628_audiostate *audio_state = &csi->audio_state;
	u32 fs_audio;
	int cur_state, init_state, pre_state;

	init_state = audio_state->init_state;
	pre_state = audio_state->pre_state;
	fs_audio = rk628_hdmirx_audio_fs(sd);
	if (!is_validfs(fs_audio)) {
		v4l2_dbg(2, debug, sd, "%s: no supported fs(%u)\n", __func__, fs_audio);
		goto exit;
	}
	if (abs(fs_audio - audio_state->fs_audio) > 1000)
		rk628_hdmirx_audio_set_fs(sd, fs_audio);
	regmap_read(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_FILLSTS1, &cur_state);
	v4l2_dbg(2, debug, sd, "%s: HDMI_RX_AUD_FIFO_FILLSTS1:%#x, single offset:%d, total offset:%d\n",
		 __func__, cur_state, cur_state - pre_state, cur_state - init_state);
	if (cur_state != 0)
		csi->audio_present = true;
	else
		csi->audio_present = false;

	if ((cur_state - init_state) > 16 && (cur_state - pre_state) > 0)
		rk628_hdmirx_audio_clk_ppm_inc(sd, 10);
	else if ((cur_state != 0) && (cur_state - init_state) < -16 && (cur_state - pre_state) < 0)
		rk628_hdmirx_audio_clk_ppm_inc(sd, -10);
	audio_state->pre_state = cur_state;
exit:
	schedule_delayed_work(&csi->delayed_work_audio, msecs_to_jiffies(1000));
}

static void rk628_csi_delayed_work_audio_rate_change(struct work_struct *work)
{
	u32 fifo_ints, fifo_fillsts;
	u32 fs_audio;
	struct delayed_work *dwork = to_delayed_work(work);
	struct rk628_csi *csi = container_of(dwork, struct rk628_csi,
					     delayed_work_audio_rate_change);
	struct v4l2_subdev *sd = &csi->sd;

	mutex_lock(&csi->confctl_mutex);
	fs_audio = rk628_hdmirx_audio_fs(sd);
	v4l2_dbg(1, debug, sd, "%s get audio fs %u\n", __func__, fs_audio);
	if (csi->audio_state.ctsn_flag == (ACR_N_CHG_ICLR | ACR_CTS_CHG_ICLR)) {
		csi->audio_state.ctsn_flag = 0;
		if (is_validfs(fs_audio)) {
			rk628_hdmirx_audio_set_fs(sd, fs_audio);
			regmap_read(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_ISTS, &fifo_ints);
			v4l2_dbg(1, debug, sd, "%s fifo ints %#x\n", __func__, fifo_ints);
			if ((fifo_ints & 0x18) == 0x18)
				rk628_hdmirx_audio_fifo_initd(sd);
			else if (fifo_ints & 0x18)
				rk628_hdmirx_audio_fifo_init(sd);
			regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_DMI_DISABLE_IF,
					   AUD_ENABLE_MASK, AUD_ENABLE(1));
			/* We start audio work after recieveing cts n interrupt */
			csi->fifo_ints_en = true;
			regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_IEN_SET,
				     AFIF_OVERFL_ISTS | AFIF_UNDERFL_ISTS);
			schedule_delayed_work(&csi->delayed_work_audio, msecs_to_jiffies(1000));
		} else {
			v4l2_warn(sd, "%s invalid fs when ctsn updating\n", __func__);
		}
	}
	if (csi->audio_state.fifo_int) {
		csi->audio_state.fifo_int = false;
		if (is_validfs(fs_audio))
			rk628_hdmirx_audio_set_fs(sd, fs_audio);
		regmap_read(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_FILLSTS1, &fifo_fillsts);
		if (fifo_fillsts) {
			v4l2_dbg(1, debug, sd, "%s overflow after underflow\n", __func__);
			rk628_hdmirx_audio_fifo_initd(sd);
		} else {
			v4l2_dbg(1, debug, sd, "%s underflow after overflow\n", __func__);
			rk628_hdmirx_audio_fifo_init(sd);
		}
	}
	mutex_unlock(&csi->confctl_mutex);
}

static void rk628_hdmirx_controller_setup(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);

	regmap_write(csi->hdmirx_regmap, HDMI_RX_HDMI20_CONTROL, 0x10001f10);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_HDMI_MODE_RECOVER, 0x000000e1);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_CTRL, 0xbfff8011);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_ASP_CTRL, 0x00000040);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_HDMI_RESMPL_CTRL, 0x00000001);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_HDMI_SYNC_CTRL, 0x00000014);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_ERR_FILTER, 0x00000008);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_SCDC_I2CCONFIG, 0x01000000);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_SCDC_CONFIG, 0x00000001);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_SCDC_WRDATA0, 0xabcdef01);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_CHLOCK_CONFIG, 0x0030c15c);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_HDMI_ERROR_PROTECT,
			0x000d0c98);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_MD_HCTRL1, 0x00000010);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_MD_HCTRL2, 0x00001738);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_MD_VCTRL, 0x00000002);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_MD_VTH, 0x0000073a);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_MD_IL_POL, 0x00000004);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_ACRM_CTRL, 0x00000000);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_HDMI_DCM_CTRL, 0x00040414);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_HDMI_CKM_EVLTM, 0x00103e70);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_HDMI_CKM_F, 0x0c1c0b54);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_HDMI_RESMPL_CTRL, 0x00000001);

	regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_HDCP_SETTINGS,
			   HDMI_RESERVED_MASK |
			   FAST_I2C_MASK |
			   ONE_DOT_ONE_MASK |
			   FAST_REAUTH_MASK,
			   HDMI_RESERVED(1) |
			   FAST_I2C(0) |
			   ONE_DOT_ONE(0) |
			   FAST_REAUTH(0));
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

	for ( i = 0; i < RXPHY_CFG_MAX_TIMES; i++) {
		rk628_hdmirx_phy_power_on(sd);
		cnt = 0;

		do {
			cnt++;
			regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_HACT_PX,
					&val);
			width = val & 0xffff;
			regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_VAL, &val);
			height = val & 0xffff;
			regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_HT1, &val);
			frame_width = (val >> 16) & 0xffff;
			regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_VTL, &val);
			frame_height = val & 0xffff;
			regmap_read(csi->hdmirx_regmap, HDMI_RX_SCDC_REGS1,
					&val);
			status = val;
			v4l2_dbg(1, debug, sd, "%s read wxh:%dx%d, total:%dx%d,"
					" SCDC_REGS1:%#x, cnt:%d\n", __func__,
					width, height, frame_width,
					frame_height, status, cnt);

			if (!tx_5v_power_present(sd)) {
				v4l2_info(sd, "HDMI pull out, return!\n");
				return -1;
			}

			if (cnt >= 15)
				break;
		} while(((status & 0xfff) != 0xf00) ||
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

	if (i == RXPHY_CFG_MAX_TIMES) {
		return -1;
	}
	return 0;
}

static void rk628_csi_initial_setup(struct v4l2_subdev *sd)
{
	struct rk628_csi *csi = to_csi(sd);
	struct v4l2_subdev_edid def_edid;

	clk_prepare_enable(csi->clk_hdmirx);
	clk_prepare_enable(csi->clk_imodet);
	clk_prepare_enable(csi->clk_hdmirx_aud);
	clk_prepare_enable(csi->clk_hdmirx_cec);
	clk_prepare_enable(csi->clk_vop);
	clk_prepare_enable(csi->clk_csi0);
	clk_prepare_enable(csi->clk_i2s_mclk);
	udelay(10);
	reset_control_assert(csi->rst_hdmirx);
	reset_control_assert(csi->rst_hdmirx_pon);
	reset_control_assert(csi->rst_csi0);
	udelay(10);
	reset_control_deassert(csi->rst_hdmirx);
	reset_control_deassert(csi->rst_hdmirx_pon);
	reset_control_deassert(csi->rst_csi0);
	udelay(10);

	regmap_update_bits(csi->grf, GRF_SYSTEM_CON0,
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
	rk628_csi_set_hdmi_hdcp(sd, false);

	mipi_dphy_reset(csi);
	mipi_dphy_power_on(csi);
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
		/* automaticly set timing rather than set by userspace */
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
	regmap_write(csi->hdmirx_regmap, HDMI_RX_MD_ICLR, md_mask);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_ICLR, pdec_mask);
	if (en) {
		regmap_write(csi->hdmirx_regmap, HDMI_RX_MD_IEN_SET, md_mask);
		regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_IEN_SET, pdec_mask);
		csi->vid_ints_en = true;
	} else {
		regmap_write(csi->hdmirx_regmap, HDMI_RX_MD_IEN_CLR, md_mask);
		regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_IEN_CLR, pdec_mask);
		regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_IEN_CLR, 0x1f);
		csi->vid_ints_en = false;
	}
	usleep_range(5000, 5000);
	regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_IEN, &md_ien);
	regmap_read(csi->hdmirx_regmap, HDMI_RX_PDEC_IEN, &pdec_ien);
	v4l2_dbg(1, debug, sd, "%s MD_IEN:%#x, PDEC_IEN:%#x\n", __func__, md_ien, pdec_ien);
}

static void rk628_csi_isr_ctsn(struct v4l2_subdev *sd, u32 pdec_ints)
{
	struct rk628_csi *csi = to_csi(sd);
	u32 ctsn_mask = ACR_N_CHG_ICLR | ACR_CTS_CHG_ICLR;

	v4l2_dbg(1, debug, sd, "%s: pdec_ints:%#x\n", __func__, pdec_ints);
	/* cts & n both need update but maybe come diff int */
	if (pdec_ints & ACR_N_CHG_ICLR)
		csi->audio_state.ctsn_flag |= ACR_N_CHG_ICLR;
	if (pdec_ints & ACR_CTS_CHG_ICLR)
		csi->audio_state.ctsn_flag |= ACR_CTS_CHG_ICLR;
	if (csi->audio_state.ctsn_flag == ctsn_mask) {
		v4l2_dbg(1, debug, sd, "%s: ctsn updated, disable ctsn int\n", __func__);
		regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_IEN_CLR, ctsn_mask);
		csi->ctsn_ints_en = false;
		schedule_delayed_work(&csi->delayed_work_audio_rate_change, 0);
	}
	regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_ICLR, pdec_ints & ctsn_mask);
}

static void rk628_csi_isr_fifoints(struct v4l2_subdev *sd, u32 fifo_ints)
{
	struct rk628_csi *csi = to_csi(sd);
	u32 fifo_mask = AFIF_OVERFL_ISTS | AFIF_UNDERFL_ISTS;

	v4l2_dbg(1, debug, sd, "%s: fifo_ints:%#x\n", __func__, fifo_ints);
	/* cts & n both need update but maybe come diff int */
	if (fifo_ints & AFIF_OVERFL_ISTS) {
		v4l2_dbg(1, debug, sd, "%s: Audio FIFO overflow\n", __func__);
		csi->audio_state.fifo_flag |= AFIF_OVERFL_ISTS;
	}
	if (fifo_ints & AFIF_UNDERFL_ISTS) {
		v4l2_dbg(1, debug, sd, "%s: Audio FIFO underflow\n", __func__);
		csi->audio_state.fifo_flag |= AFIF_UNDERFL_ISTS;
	}
	if (csi->audio_state.fifo_flag == fifo_mask) {
		csi->audio_state.fifo_int = true;
		csi->audio_state.fifo_flag = 0;
		schedule_delayed_work(&csi->delayed_work_audio_rate_change, 0);
	}
	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_ICLR, fifo_ints & fifo_mask);
}

static int rk628_csi_isr(struct v4l2_subdev *sd, u32 status, bool *handled)
{
	u32 md_ints, pdec_ints, fifo_ints, hact, vact;
	bool plugin;
	struct rk628_csi *csi = to_csi(sd);

	if (handled == NULL) {
		v4l2_err(sd, "handled NULL, err return!\n");
		return -EINVAL;
	}
	regmap_read(csi->hdmirx_regmap, HDMI_RX_PDEC_ISTS, &pdec_ints);
	if (csi->ctsn_ints_en) {
		if (pdec_ints & (ACR_N_CHG_ICLR | ACR_CTS_CHG_ICLR)) {
			rk628_csi_isr_ctsn(sd, pdec_ints);
			pdec_ints &= ~(ACR_CTS_CHG_ICLR | ACR_CTS_CHG_ICLR);
			*handled = true;
		}
	}
	if (csi->fifo_ints_en) {
		regmap_read(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_ISTS, &fifo_ints);
		if (fifo_ints & 0x18) {
			rk628_csi_isr_fifoints(sd, fifo_ints);
			*handled = true;
		}
	}
	if (csi->vid_ints_en) {
		regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_ISTS, &md_ints);
		plugin = tx_5v_power_present(sd);
		v4l2_dbg(1, debug, sd, "%s: md_ints: %#x, pdec_ints:%#x, plugin: %d\n",
			 __func__, md_ints, pdec_ints, plugin);

		if ((md_ints & (VACT_LIN_ISTS | HACT_PIX_ISTS |
				HS_CLK_ISTS | DE_ACTIVITY_ISTS |
				VS_ACT_ISTS | HS_ACT_ISTS))
				&& plugin) {

			regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_HACT_PX, &hact);
			regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_VAL, &vact);
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

		if ((pdec_ints & AVI_RCV_ISTS) && plugin) {
			v4l2_dbg(1, debug, sd, "%s: AVI RCV INT!\n", __func__);
			csi->avi_rcv_rdy = true;
			/* After get the AVI_RCV interrupt state, disable interrupt. */
			regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_IEN_CLR, AVI_RCV_ISTS);

			*handled = true;
		}
	}
	if (*handled != true)
		v4l2_dbg(1, debug, sd, "%s: unhandled interrupt!\n", __func__);

	/* clear interrupts */
	regmap_write(csi->hdmirx_regmap, HDMI_RX_MD_ICLR, 0xffffffff);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_ICLR, 0xffffffff);

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

static int rk628_csi_g_mbus_config(struct v4l2_subdev *sd,
			     struct v4l2_mbus_config *cfg)
{
	struct rk628_csi *csi = to_csi(sd);

	cfg->type = V4L2_MBUS_CSI2;
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
	enable_stream(sd, enable);

	return 0;
}

static int rk628_csi_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	switch (code->index) {
	case 0:
		code->code = MEDIA_BUS_FMT_UYVY8_2X8;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int rk628_csi_get_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = -1;
	struct rk628_csi *csi = container_of(ctrl->handler, struct rk628_csi,
			hdl);
	struct v4l2_subdev *sd = &(csi->sd);

	if ( ctrl->id == V4L2_CID_DV_RX_POWER_PRESENT) {
		ret = tx_5v_power_present(sd);
		*ctrl->p_new.p_s32 = ret;
	}

	return ret;
}

static int rk628_csi_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	v4l2_dbg(1, debug, sd, "%s:\n", __func__);

	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_UYVY8_2X8)
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
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fie->code != MEDIA_BUS_FMT_UYVY8_2X8)
		return -EINVAL;

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
		break;
	default:
		return -EINVAL;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	csi->mbus_fmt_code = format->format.code;
	mode = rk628_csi_find_best_fit(format);
	csi->cur_mode = mode;

	if ((mode->width == 3840) && (mode->height == 2160)) {
		v4l2_dbg(1, debug, sd,
			"%s res wxh:%dx%d, link freq:%llu, pixrate:%u\n",
			__func__, mode->width, mode->height,
			link_freq_menu_items[1], RK628_CSI_PIXEL_RATE_HIGH);
		__v4l2_ctrl_s_ctrl(csi->link_freq, 1);
		__v4l2_ctrl_s_ctrl_int64(csi->pixel_rate,
			RK628_CSI_PIXEL_RATE_HIGH);
	} else {
		v4l2_dbg(1, debug, sd,
			"%s res wxh:%dx%d, link freq:%llu, pixrate:%u\n",
			__func__, mode->width, mode->height,
			link_freq_menu_items[0], RK628_CSI_PIXEL_RATE_LOW);
		__v4l2_ctrl_s_ctrl(csi->link_freq, 0);
		__v4l2_ctrl_s_ctrl_int64(csi->pixel_rate,
			RK628_CSI_PIXEL_RATE_LOW);
	}

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
	regmap_update_bits(csi->grf, GRF_SYSTEM_CON0,
			SW_ADAPTER_I2CSLADR_MASK |
			SW_EDID_MODE_MASK,
			SW_ADAPTER_I2CSLADR(0) |
			SW_EDID_MODE(1));

	for (i = 0; i < (edid->blocks * EDID_BLOCK_SIZE); i ++) {
		regmap_read(csi->key_regmap, EDID_BASE + ((edid->start_block *
				EDID_BLOCK_SIZE) + i) * 4, &val);
		edid->edid[i] = val;
	}

	regmap_update_bits(csi->grf, GRF_SYSTEM_CON0,
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
	regmap_update_bits(csi->grf, GRF_SYSTEM_CON0,
			SW_ADAPTER_I2CSLADR_MASK |
			SW_EDID_MODE_MASK,
			SW_ADAPTER_I2CSLADR(0) |
			SW_EDID_MODE(1));

	for (i = 0; i < edid_len; i++) {
		regmap_write(csi->key_regmap, EDID_BASE + i * 4, edid->edid[i]);
	}

	/* read out for debug */
	if (debug >= 3) {
		printk("%s: Read EDID: ======\n", __func__);
		for (i = 0; i < edid_len; i++) {
			regmap_read(csi->key_regmap, EDID_BASE + i * 4, &val);
			printk("0x%02x ", val);
			if ((i + 1) % 8 == 0)
				printk("\n");
		}
		printk("%s: ======\n", __func__);
	}

	/* edid access by RX's i2c, i2c slave addr: 0x0 */
	regmap_update_bits(csi->grf, GRF_SYSTEM_CON0,
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
	strlcpy(inf->base.sensor, RK628_CSI_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, rk628_csi->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, rk628_csi->len_name, sizeof(inf->base.lens));
}

static long rk628_csi_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct rk628_csi *csi = to_csi(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		rk628_csi_get_module_inf(csi, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static inline void testif_testclk_assert(struct rk628_csi *csi)
{
	regmap_update_bits(csi->grf, GRF_MIPI_TX0_CON,
			   PHY_TESTCLK, PHY_TESTCLK);
	udelay(1);
}

static inline void testif_testclk_deassert(struct rk628_csi *csi)
{
	regmap_update_bits(csi->grf, GRF_MIPI_TX0_CON,
			   PHY_TESTCLK, 0);
	udelay(1);
}

static inline void testif_testclr_assert(struct rk628_csi *csi)
{
	regmap_update_bits(csi->grf, GRF_MIPI_TX0_CON,
			   PHY_TESTCLR, PHY_TESTCLR);
	udelay(1);
}

static inline void testif_testclr_deassert(struct rk628_csi *csi)
{
	regmap_update_bits(csi->grf, GRF_MIPI_TX0_CON,
			   PHY_TESTCLR, 0);
	udelay(1);
}

static inline void testif_testen_assert(struct rk628_csi *csi)
{
	regmap_update_bits(csi->grf, GRF_MIPI_TX0_CON,
			   PHY_TESTEN, PHY_TESTEN);
	udelay(1);
}

static inline void testif_testen_deassert(struct rk628_csi *csi)
{
	regmap_update_bits(csi->grf, GRF_MIPI_TX0_CON,
			   PHY_TESTEN, 0);
	udelay(1);
}

static inline void testif_set_data(struct rk628_csi *csi, u8 data)
{
	regmap_update_bits(csi->grf, GRF_MIPI_TX0_CON,
			   PHY_TESTDIN_MASK, PHY_TESTDIN(data));
	udelay(1);
}

static inline u8 testif_get_data(struct rk628_csi *csi)
{
	u32 data = 0;

	regmap_read(csi->grf, GRF_DPHY0_STATUS, &data);

	return data >> PHY_TESTDOUT_SHIFT;
}

static void testif_test_code_write(struct rk628_csi *csi, u8 test_code)
{
	testif_testclk_assert(csi);
	testif_set_data(csi, test_code);
	testif_testen_assert(csi);
	testif_testclk_deassert(csi);
	testif_testen_deassert(csi);
}

static void testif_test_data_write(struct rk628_csi *csi, u8 test_data)
{
	testif_testclk_deassert(csi);
	testif_set_data(csi, test_data);
	testif_testclk_assert(csi);
}

static u8 testif_write(struct rk628_csi *csi, u8 test_code, u8 test_data)
{
	u8 monitor_data;
	struct v4l2_subdev *sd = &csi->sd;

	testif_test_code_write(csi, test_code);
	testif_test_data_write(csi, test_data);
	monitor_data = testif_get_data(csi);

	v4l2_dbg(1, debug, sd, "test_code=0x%02x, ", test_code);
	v4l2_dbg(1, debug, sd, "test_data=0x%02x, ", test_data);
	v4l2_dbg(1, debug, sd, "monitor_data=0x%02x\n", monitor_data);

	return monitor_data;
}

static inline u8 testif_read(struct rk628_csi *csi, u8 test_code)
{
	u8 test_data;

	testif_test_code_write(csi, test_code);
	test_data = testif_get_data(csi);
	testif_test_data_write(csi, test_data);

	return test_data;
}

static inline void mipi_dphy_enableclk_assert(struct rk628_csi *csi)
{
	regmap_update_bits(csi->csi_regmap, CSITX_DPHY_CTRL, DPHY_ENABLECLK,
			DPHY_ENABLECLK);
	udelay(1);
}

static inline void mipi_dphy_enableclk_deassert(struct rk628_csi *csi)
{
	regmap_update_bits(csi->csi_regmap, CSITX_DPHY_CTRL, DPHY_ENABLECLK, 0);
	udelay(1);
}

static inline void mipi_dphy_shutdownz_assert(struct rk628_csi *csi)
{
	regmap_update_bits(csi->grf, GRF_MIPI_TX0_CON, CSI_PHYSHUTDOWNZ, 0);
	udelay(1);
}

static inline void mipi_dphy_shutdownz_deassert(struct rk628_csi *csi)
{
	regmap_update_bits(csi->grf, GRF_MIPI_TX0_CON, CSI_PHYSHUTDOWNZ,
			CSI_PHYSHUTDOWNZ);
	udelay(1);
}

static inline void mipi_dphy_rstz_assert(struct rk628_csi *csi)
{
	regmap_update_bits(csi->grf, GRF_MIPI_TX0_CON, CSI_PHYRSTZ, 0);
	udelay(1);
}

static inline void mipi_dphy_rstz_deassert(struct rk628_csi *csi)
{
	regmap_update_bits(csi->grf, GRF_MIPI_TX0_CON, CSI_PHYRSTZ,
			CSI_PHYRSTZ);
	udelay(1);
}

static void mipi_dphy_init_hsfreqrange(struct rk628_csi *csi)
{
	const struct {
		unsigned long max_lane_mbps;
		u8 hsfreqrange;
	} hsfreqrange_table[] = {
		{  90, 0x00}, { 100, 0x10}, { 110, 0x20}, { 130, 0x01},
		{ 140, 0x11}, { 150, 0x21}, { 170, 0x02}, { 180, 0x12},
		{ 200, 0x22}, { 220, 0x03}, { 240, 0x13}, { 250, 0x23},
		{ 270, 0x04}, { 300, 0x14}, { 330, 0x05}, { 360, 0x15},
		{ 400, 0x25}, { 450, 0x06}, { 500, 0x16}, { 550, 0x07},
		{ 600, 0x17}, { 650, 0x08}, { 700, 0x18}, { 750, 0x09},
		{ 800, 0x19}, { 850, 0x29}, { 900, 0x39}, { 950, 0x0a},
		{1000, 0x1a}, {1050, 0x2a}, {1100, 0x3a}, {1150, 0x0b},
		{1200, 0x1b}, {1250, 0x2b}, {1300, 0x3b}, {1350, 0x0c},
		{1400, 0x1c}, {1450, 0x2c}, {1500, 0x3c}
	};
	u8 hsfreqrange;
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(hsfreqrange_table); index++)
		if (csi->lane_mbps <= hsfreqrange_table[index].max_lane_mbps)
			break;

	if (index == ARRAY_SIZE(hsfreqrange_table))
		--index;

	hsfreqrange = hsfreqrange_table[index].hsfreqrange;
	testif_write(csi, 0x44, HSFREQRANGE(hsfreqrange));
}

static int mipi_dphy_reset(struct rk628_csi *csi)
{
	u32 val, mask;
	int ret;

	mipi_dphy_enableclk_deassert(csi);
	mipi_dphy_shutdownz_assert(csi);
	mipi_dphy_rstz_assert(csi);
	testif_testclr_assert(csi);

	/* Set all REQUEST inputs to zero */
	regmap_update_bits(csi->grf, GRF_MIPI_TX0_CON,
		     FORCETXSTOPMODE_MASK | FORCERXMODE_MASK,
		     FORCETXSTOPMODE(0) | FORCERXMODE(0));
	udelay(1);
	testif_testclr_deassert(csi);
	mipi_dphy_enableclk_assert(csi);
	mipi_dphy_shutdownz_deassert(csi);
	mipi_dphy_rstz_deassert(csi);
	usleep_range(1500, 2000);

	mask = STOPSTATE_CLK | STOPSTATE_LANE0;
	ret = regmap_read_poll_timeout(csi->csi_regmap, CSITX_CSITX_STATUS1,
				       val, (val & mask) == mask,
				       0, 1000);
	if (ret < 0) {
		dev_err(csi->dev, "lane module is not in stop state\n");
		return ret;
	}

	return 0;
}

static int mipi_dphy_power_on(struct rk628_csi *csi)
{
	unsigned int val;
	u32 bus_width;
	int ret;
	struct v4l2_subdev *sd = &csi->sd;

	if ((csi->timings.bt.width == 3840) &&
			(csi->timings.bt.height == 2160)) {
		csi->lane_mbps = MIPI_DATARATE_MBPS_HIGH;
	} else {
		csi->lane_mbps = MIPI_DATARATE_MBPS_LOW;
	}

	bus_width =  csi->lane_mbps << 8;
	bus_width |= COMBTXPHY_MODULEA_EN;
	v4l2_dbg(1, debug, sd, "%s mipi bitrate:%llu mbps\n", __func__,
			csi->lane_mbps);
	phy_set_bus_width(csi->txphy, bus_width);
	phy_set_mode(csi->txphy, PHY_MODE_VIDEO_MIPI);

	mipi_dphy_init_hsfreqrange(csi);
	usleep_range(1500, 2000);
	phy_power_on(csi->txphy);

	ret = regmap_read_poll_timeout(csi->csi_regmap, CSITX_CSITX_STATUS1,
				       val, val & DPHY_PLL_LOCK, 0, 1000);
	if (ret < 0) {
		dev_err(csi->dev, "PHY is not locked\n");
		return ret;
	}

	udelay(10);

	return 0;
}

static void mipi_dphy_power_off(struct rk628_csi *csi)
{
	phy_power_off(csi->txphy);
}

#ifdef CONFIG_COMPAT
static long rk628_csi_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	long ret;

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

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static const struct v4l2_ctrl_ops rk628_csi_ctrl_ops = {
	.g_volatile_ctrl = rk628_csi_get_ctrl,
};

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
	.g_mbus_config = rk628_csi_g_mbus_config,
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
	struct v4l2_fwnode_endpoint *endpoint;
	struct device_node *ep;
	int ret = -EINVAL;
	bool hdcp1x_enable = false;

	csi->clk_hdmirx = devm_clk_get(dev, "hdmirx");
	if (IS_ERR(csi->clk_hdmirx)) {
		ret = PTR_ERR(csi->clk_hdmirx);
		dev_err(dev, "failed to get clk_hdmirx: %d\n", ret);
		return ret;
	}

	csi->clk_imodet = devm_clk_get(dev, "imodet");
	if (IS_ERR(csi->clk_imodet)) {
		ret = PTR_ERR(csi->clk_imodet);
		dev_err(dev, "failed to get clk_imodet: %d\n", ret);
		return ret;
	}

	csi->clk_hdmirx_aud = devm_clk_get(dev, "hdmirx_aud");
	if (IS_ERR(csi->clk_hdmirx_aud)) {
		ret = PTR_ERR(csi->clk_hdmirx_aud);
		dev_err(dev, "failed to get clk_hdmirx_aud: %d\n", ret);
		return ret;
	}

	csi->clk_hdmirx_cec = devm_clk_get(dev, "hdmirx_cec");
	if (IS_ERR(csi->clk_hdmirx_cec)) {
		ret = PTR_ERR(csi->clk_hdmirx_cec);
		dev_err(dev, "failed to get clk_hdmirx_cec: %d\n", ret);
		return ret;
	}

	csi->clk_vop = devm_clk_get(dev, "vop");
	if (IS_ERR(csi->clk_vop)) {
		ret = PTR_ERR(csi->clk_vop);
		dev_err(dev, "failed to get clk_vop: %d\n", ret);
		return ret;
	}

	csi->clk_rx_read = devm_clk_get(dev, "rx_read");
	if (IS_ERR(csi->clk_rx_read)) {
		ret = PTR_ERR(csi->clk_rx_read);
		dev_err(dev, "failed to get clk_rx_read: %d\n", ret);
		return ret;
	}

	csi->clk_csi0 = devm_clk_get(dev, "csi0");
	if (IS_ERR(csi->clk_csi0)) {
		ret = PTR_ERR(csi->clk_csi0);
		dev_err(dev, "failed to get clk_csi0: %d\n", ret);
		return ret;
	}

	csi->clk_i2s_mclk = devm_clk_get_optional(dev, "i2s_mclk");
	if (IS_ERR(csi->clk_i2s_mclk)) {
		ret = PTR_ERR(csi->clk_i2s_mclk);
		dev_err(dev, "failed to get i2s_mclk: %d\n", ret);
		return ret;
	}
	if (csi->clk_i2s_mclk == NULL)
		dev_warn(dev, "i2s_mclk is not configured\n");

	csi->rst_hdmirx = of_reset_control_get(dev->of_node, "hdmirx");
	if (IS_ERR(csi->rst_hdmirx)) {
		ret = PTR_ERR(csi->rst_hdmirx);
		dev_err(dev, "failed to get rst_hdmirx control: %d\n", ret);
		return ret;
	}

	csi->rst_hdmirx_pon = of_reset_control_get(dev->of_node, "hdmirx_pon");
	if (IS_ERR(csi->rst_hdmirx_pon)) {
		ret = PTR_ERR(csi->rst_hdmirx_pon);
		dev_err(dev, "failed to get rst_hdmirx_pon control: %d\n", ret);
		return ret;
	}

	csi->rst_decoder = of_reset_control_get(dev->of_node, "decoder");
	if (IS_ERR(csi->rst_decoder)) {
		ret = PTR_ERR(csi->rst_decoder);
		dev_err(dev, "failed to get rst_decoder control: %d\n", ret);
		return ret;
	}

	csi->rst_clk_rx = of_reset_control_get(dev->of_node, "clk_rx");
	if (IS_ERR(csi->rst_clk_rx)) {
		ret = PTR_ERR(csi->rst_clk_rx);
		dev_err(dev, "failed to get rst_clk_rx control: %d\n", ret);
		return ret;
	}

	csi->rst_vop = of_reset_control_get(dev->of_node, "vop");
	if (IS_ERR(csi->rst_vop)) {
		ret = PTR_ERR(csi->rst_vop);
		dev_err(dev, "failed to get rst_vop control: %d\n", ret);
		return ret;
	}

	csi->rst_csi0 = of_reset_control_get(dev->of_node, "csi0");
	if (IS_ERR(csi->rst_csi0)) {
		ret = PTR_ERR(csi->rst_csi0);
		dev_err(dev, "failed to get rst_csi0 control: %d\n", ret);
		return ret;
	}

	csi->power_gpio = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_HIGH);
	if (IS_ERR(csi->power_gpio)) {
		dev_err(dev, "failed to get power gpio\n");
		ret = PTR_ERR(csi->power_gpio);
		return ret;
	}

	if (csi->power_gpio) {
		gpiod_set_value(csi->power_gpio, 1);
		usleep_range(500, 510);
	}

	csi->plugin_det_gpio = devm_gpiod_get_optional(dev, "plugin-det",
						    GPIOD_IN);
	if (IS_ERR(csi->plugin_det_gpio)) {
		dev_err(dev, "failed to get hdmirx det gpio\n");
		ret = PTR_ERR(csi->plugin_det_gpio);
		return ret;
	}

	csi->rxphy = devm_phy_get(dev, "combrxphy");
	if (IS_ERR(csi->rxphy)) {
		ret = PTR_ERR(csi->rxphy);
		dev_err(dev, "failed to get rxphy: %d\n", ret);
		return ret;
	}

	csi->txphy = devm_phy_get(dev, "combtxphy");
	if (IS_ERR(csi->txphy)) {
		ret = PTR_ERR(csi->txphy);
		dev_err(dev, "failed to get txphy: %d\n", ret);
		return ret;
	}

	if (of_property_read_bool(dev->of_node, "hdcp-enable"))
		hdcp1x_enable = true;

	ep = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!ep) {
		dev_err(dev, "missing endpoint node\n");
		return -EINVAL;
	}

	endpoint = v4l2_fwnode_endpoint_alloc_parse(of_fwnode_handle(ep));
	if (IS_ERR(endpoint)) {
		dev_err(dev, "failed to parse endpoint\n");
		return PTR_ERR(endpoint);
	}

	if (endpoint->bus_type != V4L2_MBUS_CSI2 ||
	    endpoint->bus.mipi_csi2.num_data_lanes == 0) {
		dev_err(dev, "missing CSI-2 properties in endpoint\n");
		goto free_endpoint;
	}

	csi->csi_lanes_in_use = endpoint->bus.mipi_csi2.num_data_lanes;
	csi->enable_hdcp = hdcp1x_enable;
	csi->rxphy_pwron = false;
	csi->txphy_pwron = false;
	csi->nosignal = true;
	csi->avi_rcv_rdy = false;

	ret = 0;

free_endpoint:
	v4l2_fwnode_endpoint_free(endpoint);
	return ret;
}

static const struct regmap_range rk628_csi_readable_ranges[] = {
	regmap_reg_range(CSITX_CONFIG_DONE, CSITX_CSITX_VERSION),
	regmap_reg_range(CSITX_SYS_CTRL0_IMD, CSITX_TIMING_HPW_PADDING_NUM),
	regmap_reg_range(CSITX_VOP_PATH_CTRL, CSITX_VOP_PATH_CTRL),
	regmap_reg_range(CSITX_VOP_PATH_PKT_CTRL, CSITX_VOP_PATH_PKT_CTRL),
	regmap_reg_range(CSITX_CSITX_STATUS0, CSITX_LPDT_DATA_IMD),
	regmap_reg_range(CSITX_DPHY_CTRL, CSITX_DPHY_CTRL),
};

static const struct regmap_access_table rk628_csi_readable_table = {
	.yes_ranges     = rk628_csi_readable_ranges,
	.n_yes_ranges   = ARRAY_SIZE(rk628_csi_readable_ranges),
};

static const struct regmap_config rk628_csi_regmap_cfg = {
	.name = "csi",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = CSI_MAX_REGISTER,
	.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.rd_table = &rk628_csi_readable_table,
};

static const struct regmap_range rk628_hdmirx_readable_ranges[] = {
	regmap_reg_range(HDMI_RX_HDMI_SETUP_CTRL, HDMI_RX_HDMI_SETUP_CTRL),
	regmap_reg_range(HDMI_RX_HDMI_PCB_CTRL, HDMI_RX_HDMI_PCB_CTRL),
	regmap_reg_range(HDMI_RX_HDMI_MODE_RECOVER, HDMI_RX_HDMI_ERROR_PROTECT),
	regmap_reg_range(HDMI_RX_HDMI_SYNC_CTRL, HDMI_RX_HDMI_CKM_RESULT),
	regmap_reg_range(HDMI_RX_HDMI_RESMPL_CTRL, HDMI_RX_HDMI_RESMPL_CTRL),
	regmap_reg_range(HDMI_VM_CFG_CH2, HDMI_VM_CFG_CH2),
	regmap_reg_range(HDMI_RX_HDCP_CTRL, HDMI_RX_HDCP_SETTINGS),
	regmap_reg_range(HDMI_RX_HDCP_KIDX, HDMI_RX_HDCP_KIDX),
	regmap_reg_range(HDMI_RX_HDCP_DBG, HDMI_RX_HDCP_AN0),
	regmap_reg_range(HDMI_RX_HDCP_STS, HDMI_RX_HDCP_STS),
	regmap_reg_range(HDMI_RX_MD_HCTRL1, HDMI_RX_MD_HACT_PX),
	regmap_reg_range(HDMI_RX_MD_VCTRL, HDMI_RX_MD_VSC),
	regmap_reg_range(HDMI_RX_MD_VOL, HDMI_RX_MD_VTL),
	regmap_reg_range(HDMI_RX_MD_IL_POL, HDMI_RX_MD_STS),
	regmap_reg_range(HDMI_RX_AUD_CTRL, HDMI_RX_AUD_CTRL),
	regmap_reg_range(HDMI_RX_AUD_PLL_CTRL, HDMI_RX_AUD_PLL_CTRL),
	regmap_reg_range(HDMI_RX_AUD_CLK_CTRL, HDMI_RX_AUD_CLK_CTRL),
	regmap_reg_range(HDMI_RX_AUD_FIFO_CTRL, HDMI_RX_AUD_FIFO_TH),
	regmap_reg_range(HDMI_RX_AUD_CHEXTR_CTRL, HDMI_RX_AUD_PAO_CTRL),
	regmap_reg_range(HDMI_RX_AUD_FIFO_STS, HDMI_RX_AUD_FIFO_STS),
	regmap_reg_range(HDMI_RX_AUDPLL_GEN_CTS, HDMI_RX_AUDPLL_GEN_N),
	regmap_reg_range(HDMI_RX_PDEC_CTRL, HDMI_RX_PDEC_CTRL),
	regmap_reg_range(HDMI_RX_PDEC_AUDIODET_CTRL, HDMI_RX_PDEC_AUDIODET_CTRL),
	regmap_reg_range(HDMI_RX_PDEC_ERR_FILTER, HDMI_RX_PDEC_ASP_CTRL),
	regmap_reg_range(HDMI_RX_PDEC_GCP_AVMUTE, HDMI_RX_PDEC_GCP_AVMUTE),
	regmap_reg_range(HDMI_RX_PDEC_ACR_CTS, HDMI_RX_PDEC_ACR_N),
	regmap_reg_range(HDMI_RX_PDEC_AIF_CTRL, HDMI_RX_PDEC_AIF_PB0),
	regmap_reg_range(HDMI_RX_PDEC_AVI_PB, HDMI_RX_PDEC_AVI_PB),
	regmap_reg_range(HDMI_RX_HDMI20_CONTROL, HDMI_RX_CHLOCK_CONFIG),
	regmap_reg_range(HDMI_RX_SCDC_REGS1, HDMI_RX_SCDC_REGS2),
	regmap_reg_range(HDMI_RX_SCDC_WRDATA0, HDMI_RX_SCDC_WRDATA0),
	regmap_reg_range(HDMI_RX_PDEC_ISTS, HDMI_RX_PDEC_IEN),
	regmap_reg_range(HDMI_RX_AUD_FIFO_ISTS, HDMI_RX_AUD_FIFO_IEN),
	regmap_reg_range(HDMI_RX_MD_ISTS, HDMI_RX_MD_IEN),
	regmap_reg_range(HDMI_RX_HDMI_ISTS, HDMI_RX_HDMI_IEN),
	regmap_reg_range(HDMI_RX_DMI_DISABLE_IF, HDMI_RX_DMI_DISABLE_IF),
};

static const struct regmap_access_table rk628_hdmirx_readable_table = {
	.yes_ranges     = rk628_hdmirx_readable_ranges,
	.n_yes_ranges   = ARRAY_SIZE(rk628_hdmirx_readable_ranges),
};

static const struct regmap_config rk628_hdmirx_regmap_cfg = {
	.name = "hdmirx",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = HDMIRX_MAX_REGISTER,
	.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.rd_table = &rk628_hdmirx_readable_table,
};

static const struct regmap_range rk628_key_readable_ranges[] = {
	regmap_reg_range(EDID_BASE, EDID_BASE + 0x400),
	regmap_reg_range(HDCP_KEY_BASE, HDCP_KEY_BASE + 0x490),
};

static const struct regmap_access_table rk628_key_readable_table = {
	.yes_ranges     = rk628_key_readable_ranges,
	.n_yes_ranges   = ARRAY_SIZE(rk628_key_readable_ranges),
};

static const struct regmap_config rk628_key_regmap_cfg = {
	.name = "key_map",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = KEY_MAX_REGISTER,
	.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.rd_table = &rk628_key_readable_table,
};

static int rk628_csi_probe(struct platform_device *pdev)
{
	struct rk628_csi *csi;
	struct v4l2_subdev *sd;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	char facing[2];
	int err;
	u32 val;

	struct rk628 *rk628 = dev_get_drvdata(pdev->dev.parent);
	struct i2c_client *client = rk628->client;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	if (!of_device_is_available(dev->of_node))
		return -ENODEV;

	csi = devm_kzalloc(dev, sizeof(*csi), GFP_KERNEL);
	if (!csi)
		return -ENOMEM;

	csi->dev = dev;
	csi->parent = rk628;
	platform_set_drvdata(pdev, csi);

	csi->cur_mode = &supported_modes[0];
	csi->grf = rk628->grf;
	if (!csi->grf)
		return -ENODEV;

	csi->hdmirx_irq = platform_get_irq(pdev, 1);
	if (csi->hdmirx_irq < 0)
		return csi->hdmirx_irq;

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

	csi->i2c_client = client;
	sd = &csi->sd;
	sd->dev = dev;

	err = rk628_csi_probe_of(csi);
	if (err) {
		v4l2_err(sd, "rk628_csi_probe_of failed! err:%d\n", err);
		return err;
	}

	v4l2_subdev_init(sd, &rk628_csi_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	csi->hdmirx_regmap = devm_regmap_init_i2c(rk628->client,
						 &rk628_hdmirx_regmap_cfg);
	csi->csi_regmap = devm_regmap_init_i2c(rk628->client,
						 &rk628_csi_regmap_cfg);
	csi->key_regmap = devm_regmap_init_i2c(rk628->client,
						 &rk628_key_regmap_cfg);

	/* i2c access, read chip id*/
	err = regmap_read(csi->csi_regmap, CSITX_CSITX_VERSION, &val);
	if (err) {
		v4l2_err(sd, "i2c access failed! err:%d\n", err);
		return -ENODEV;
	}
	v4l2_dbg(1, debug, sd, "CSITX VERSION: %#x\n", val);

	mutex_init(&csi->confctl_mutex);

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
			&rk628_csi_ctrl_ops, V4L2_CID_DV_RX_POWER_PRESENT,
			0, 1, 0, 0);
	if (csi->detect_tx_5v_ctrl)
		csi->detect_tx_5v_ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	/* custom controls */
	csi->audio_sampling_rate_ctrl = v4l2_ctrl_new_custom(&csi->hdl,
			&rk628_csi_ctrl_audio_sampling_rate, NULL);
	csi->audio_present_ctrl = v4l2_ctrl_new_custom(&csi->hdl,
			&rk628_csi_ctrl_audio_present, NULL);
	if (csi->audio_sampling_rate_ctrl)
		csi->audio_sampling_rate_ctrl->flags |=
			V4L2_CTRL_FLAG_VOLATILE;

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

	INIT_DELAYED_WORK(&csi->delayed_work_enable_hotplug,
			rk628_csi_delayed_work_enable_hotplug);
	INIT_DELAYED_WORK(&csi->delayed_work_res_change,
			rk628_delayed_work_res_change);
	INIT_DELAYED_WORK(&csi->delayed_work_audio_rate_change,
			rk628_csi_delayed_work_audio_rate_change);
	INIT_DELAYED_WORK(&csi->delayed_work_audio,
			rk628_csi_delayed_work_audio);
	rk628_csi_initial_setup(sd);

	if (csi->hdmirx_irq) {
		v4l2_dbg(1, debug, sd, "cfg hdmirx irq!\n");
		err = devm_request_threaded_irq(dev, csi->hdmirx_irq, NULL,
				rk628_csi_irq_handler, IRQF_TRIGGER_HIGH |
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

	csi->plugin_irq = gpiod_to_irq(csi->plugin_det_gpio);
	if (csi->plugin_irq < 0) {
		dev_err(dev, "failed to get plugin det irq\n");
		err = csi->plugin_irq;
		goto err_work_queues;
	}

	err = devm_request_threaded_irq(dev, csi->plugin_irq, NULL,
			plugin_detect_irq, IRQF_TRIGGER_FALLING |
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "rk628_csi", csi);
	if (err) {
		dev_err(dev, "failed to register plugin det irq (%d)\n", err);
		goto err_work_queues;
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
	cancel_delayed_work(&csi->delayed_work_audio);
	cancel_delayed_work(&csi->delayed_work_audio_rate_change);
err_hdl:
	mutex_destroy(&csi->confctl_mutex);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&csi->hdl);
	return err;
}

static int rk628_csi_remove(struct platform_device *pdev)
{
	struct rk628_csi *csi = platform_get_drvdata(pdev);
	struct v4l2_subdev *sd = &csi->sd;

	if (!csi->hdmirx_irq) {
		del_timer_sync(&csi->timer);
		flush_work(&csi->work_i2c_poll);
	}
	cancel_delayed_work_sync(&csi->delayed_work_enable_hotplug);
	cancel_delayed_work_sync(&csi->delayed_work_res_change);
	cancel_delayed_work_sync(&csi->delayed_work_audio);
	cancel_delayed_work_sync(&csi->delayed_work_audio_rate_change);

	if (csi->rxphy_pwron)
		phy_power_off(csi->rxphy);
	if (csi->txphy_pwron)
		mipi_dphy_power_off(csi);

	v4l2_async_unregister_subdev(sd);
	v4l2_device_unregister_subdev(sd);
	mutex_destroy(&csi->confctl_mutex);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&csi->hdl);

	reset_control_assert(csi->rst_hdmirx);
	reset_control_assert(csi->rst_hdmirx_pon);
	reset_control_assert(csi->rst_decoder);
	reset_control_assert(csi->rst_clk_rx);
	reset_control_assert(csi->rst_vop);
	reset_control_assert(csi->rst_csi0);

	clk_disable_unprepare(csi->clk_hdmirx);
	clk_disable_unprepare(csi->clk_imodet);
	clk_disable_unprepare(csi->clk_hdmirx_aud);
	clk_disable_unprepare(csi->clk_hdmirx_cec);
	clk_disable_unprepare(csi->clk_vop);
	clk_disable_unprepare(csi->clk_rx_read);
	clk_disable_unprepare(csi->clk_csi0);
	clk_disable_unprepare(csi->clk_i2s_mclk);

	return 0;
}

static const struct of_device_id rk628_csi_of_match[] = {
	{ .compatible = "rockchip,rk628-csi" },
	{}
};
MODULE_DEVICE_TABLE(of, rk628_csi_of_match);

static struct platform_driver rk628_csi_driver = {
	.driver = {
		.name = "rk628-csi",
		.of_match_table = of_match_ptr(rk628_csi_of_match),
	},
	.probe	= rk628_csi_probe,
	.remove = rk628_csi_remove,
};
module_platform_driver(rk628_csi_driver);

MODULE_DESCRIPTION("Rockchip RK628 HDMI to MIPI CSI-2 bridge driver");
MODULE_AUTHOR("Dingxian Wen <shawn.wen@rock-chips.com>");
MODULE_LICENSE("GPL v2");
