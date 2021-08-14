// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Rockchip Electronics Co. Ltd.
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
#include <linux/soc/rockchip/rk_vendor_storage.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <video/videomode.h>
#include "rk628_csi.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-1)");

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x0, 0x6)
#define RKMODULE_CAMERA_MODULE_INDEX    "rockchip,camera-module-index"

#define EDID_NUM_BLOCKS_MAX		2
#define EDID_BLOCK_SIZE			128

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

#define USE_4_LANES			4
#define YUV422_8BIT			0x1e
/* Test Code: 0x44 (HS RX Control of Lane 0) */
#define HSFREQRANGE(x)			UPDATE(x, 6, 1)

struct hdcp_keys {
	u8 KSV[HDCP_KEY_KSV_SIZE];
	u8 devicekey[HDCP_PRIVATE_KEY_SIZE];
	u8 sha[HDCP_KEY_SHA_SIZE];
};

struct rk628_hdcp {
	char *seeds;
	struct hdcp_keys *keys;
};

struct rk628_csi {
	struct device *dev;
	struct rk628 *parent;
	struct class *rk628_class;
	struct i2c_client *i2c_client;
	struct v4l2_dv_timings timings;
	struct gpio_desc *power_gpio;
	struct gpio_desc *plugin_det_gpio;
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
	struct delayed_work delayed_work_audio;
	struct timer_list timer;
	struct work_struct work_i2c_poll;
	struct phy *rxphy;
	struct phy *txphy;
	struct mutex confctl_mutex;
	u8 edid_blocks_written;
	u64 lane_mbps;
	u8 csi_lanes_in_use;
	u64 fs_audio;
	u8 fps;
	u32 stream_state;
	int hdmirx_irq;
	int plugin_irq;
	bool nosignal;
	bool rxphy_pwron;
	bool txphy_pwron;
	bool enable_hdcp;
	bool audio_present;
	bool hpd_output_inverted;
	bool avi_rcv_rdy;
	struct rk628_hdcp hdcp;
};

struct rk628_csi_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
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

static void rk628_post_process_setup(struct rk628_csi *csi);
static void rk628_csi_enable_interrupts(struct rk628_csi *csi, bool en);
static int rk628_csi_s_dv_timings(struct rk628_csi *csi,
				 struct v4l2_dv_timings *timings);
static int rk628_csi_s_edid(struct rk628_csi *csi,
				struct v4l2_subdev_edid *edid);
static int mipi_dphy_power_on(struct rk628_csi *csi);
static int mipi_dphy_reset(struct rk628_csi *csi);
static void mipi_dphy_power_off(struct rk628_csi *csi);
static void mipi_dphy_init_hsfreqrange(struct rk628_csi *csi);
static int rk628_hdmirx_phy_power_on(struct rk628_csi *csi);
static int rk628_hdmirx_phy_setup(struct rk628_csi *csi);
static void rk628_hdmirx_controller_setup(struct rk628_csi *csi);
static void rk628_csi_format_change(struct rk628_csi *csi);
static void enable_stream(struct rk628_csi *csi, bool enable);
static void rk628_hdmirx_audio_setup(struct rk628_csi *csi);
static void rk628_csi_set_csi(struct rk628_csi *csi);
static void rk628_hdmirx_vid_enable(struct rk628_csi *csi, bool en);
static void rk628_hdmirx_hpd_ctrl(struct rk628_csi *csi, bool en);
static inline bool tx_5v_power_present(struct rk628_csi *csi);
static void rk628_csi_set_hdmi_hdcp(struct rk628_csi *csi, bool en);

static struct rk628_csi *g_csi;

static ssize_t rk628_resolution_read(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct rk628_csi *csi = g_csi;
	u32 width, height, fps, fs_audio;
	static u8 cnt;

	if (csi->nosignal && tx_5v_power_present(csi)) {
		if (cnt++ >= 6) {
			cnt = 0;
			dev_info(csi->dev, "no signal but 5v_det, recfg hdmirx!\n");
			schedule_delayed_work(&csi->delayed_work_enable_hotplug,
					HZ / 20);
		}
	} else {
		cnt = 0;
	}

	width = csi->timings.bt.width;
	height = csi->timings.bt.height;
	fps = csi->fps;
	fs_audio = csi->fs_audio;

	/* update resolution for userspace only if signal is stable */
	if (csi->nosignal) {
		dev_dbg(csi->dev, "%s: nosignal\n", __func__);
		return sprintf(buf, "unsupported\n");
	}

	dev_dbg(csi->dev, "%s %dx%dP%d@%d\n", __func__, width, height, fps,
			fs_audio);
	return sprintf(buf, "%dx%dP%d@%d\n", width, height, fps, fs_audio);
}

static ssize_t rk628_stream_state_read(struct class *class,
		struct class_attribute *attr, char *buf)
{
	struct rk628_csi *csi = g_csi;

	dev_dbg(csi->dev, "%s: state: %d\n", __func__, csi->stream_state);
	return sprintf(buf, "%d\n", csi->stream_state);
}

static ssize_t rk628_stream_state_write(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	struct rk628_csi *csi = g_csi;
	u32 state = 0;
	int ret;

	if (csi->nosignal) {
		dev_dbg(csi->dev, "%s: nosignal, no need to set stream!\n", __func__);
		return count;
	}

	ret = kstrtouint(buf, 2, &state);
	if (!ret) {
		dev_dbg(csi->dev, "%s: state: %d\n", __func__, state);
		csi->stream_state = state;
		if (csi->stream_state == 0)
			enable_stream(csi, false);
		else
			enable_stream(csi, true);
	} else {
		dev_err(csi->dev, "%s: write stream state failed!!!\n", __func__);
	}

	return count;
}

static CLASS_ATTR(resolution, 0444, rk628_resolution_read, NULL);
static CLASS_ATTR(streamen, 0664, rk628_stream_state_read, rk628_stream_state_write);

static inline bool tx_5v_power_present(struct rk628_csi *csi)
{
	bool ret;
	int val, i, cnt;

	cnt = 0;
	for (i = 0; i < 10; i++) {
		val = gpiod_get_value(csi->plugin_det_gpio);
		if (val > 0)
			cnt++;
		usleep_range(2000, 2100);
	}

	ret = (cnt >= 7) ? true : false;
	dev_dbg(csi->dev, "%s: %d\n", __func__, ret);

	return ret;
}

static int rk628_csi_get_detected_timings(struct rk628_csi *csi,
				     struct v4l2_dv_timings *timings)
{
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
	regmap_read(csi->hdmirx_regmap, 0x3014c, &val);
	hofs_pix = val & 0xffff;
	regmap_read(csi->hdmirx_regmap, 0x30164, &val);
	vbp = (val & 0xffff) + 1;

	regmap_read(csi->hdmirx_regmap, 0x3009c, &val);
	tmdsclk_cnt = val & 0xffff;
	tmp_data = tmdsclk_cnt;
	tmp_data = ((tmp_data * MODETCLK_HZ) + MODETCLK_CNT_NUM / 2);
	do_div(tmp_data, MODETCLK_CNT_NUM);
	tmds_clk = tmp_data;
	if (!(htotal * vtotal)) {
		dev_err(csi->dev, "timing err, htotal:%d, vtotal:%d\n",
				htotal, vtotal);
		goto TIMING_ERR;
	}
	fps = (tmds_clk + (htotal * vtotal) / 2) / (htotal * vtotal);

	regmap_read(csi->hdmirx_regmap, 0x30148, &val);
	modetclk_cnt_hs = val & 0xffff;
	hs = (tmdsclk_cnt * modetclk_cnt_hs + MODETCLK_CNT_NUM / 2) /
		MODETCLK_CNT_NUM;

	regmap_read(csi->hdmirx_regmap, 0x3015c, &val);
	modetclk_cnt_vs = val & 0xffff;
	vs = (tmdsclk_cnt * modetclk_cnt_vs + MODETCLK_CNT_NUM / 2) /
		MODETCLK_CNT_NUM;
	vs = (vs + htotal / 2) / htotal;

	if ((hofs_pix < hs) || (htotal < (hact + hofs_pix)) ||
			(vtotal < (vact + vs + vbp))) {
		dev_err(csi->dev,
			"timing err, total:%dx%d, act:%dx%d, hofs:%d, hs:%d, vs:%d, vbp:%d\n",
			htotal, vtotal, hact, vact, hofs_pix, hs, vs, vbp);
		goto TIMING_ERR;
	}
	hbp = hofs_pix - hs;
	hfp = htotal - hact - hofs_pix;
	vfp = vtotal - vact - vs - vbp;

	dev_dbg(csi->dev, "cnt_num:%d, tmds_cnt:%d, hs_cnt:%d, vs_cnt:%d, hofs:%d\n",
			MODETCLK_CNT_NUM, tmdsclk_cnt, modetclk_cnt_hs,
			modetclk_cnt_vs, hofs_pix);

	bt->width = hact;
	bt->height = vact;
	bt->hfrontporch = hfp;
	bt->hsync = hs;
	bt->hbackporch = hbp;
	bt->vfrontporch = vfp;
	bt->vsync = vs;
	bt->vbackporch = vbp;
	bt->pixelclock = htotal * vtotal * fps;
	csi->fps = fps;

	if (bt->interlaced == V4L2_DV_INTERLACED) {
		bt->height *= 2;
		bt->il_vsync = bt->vsync + 1;
		bt->pixelclock /= 2;
	}

	dev_info(csi->dev,
		"SCDC_REGS1:%#x, act:%dx%d, total:%dx%d, fps:%d, pixclk:%llu\n",
		status, hact, vact, htotal, vtotal, fps, bt->pixelclock);
	dev_info(csi->dev,
		"hfp:%d, hs:%d, hbp:%d, vfp:%d, vs:%d, vbp:%d, interlace:%d\n",
		bt->hfrontporch, bt->hsync, bt->hbackporch, bt->vfrontporch,
		bt->vsync, bt->vbackporch, bt->interlaced);

	return 0;

TIMING_ERR:
	return -ENOLCK;
}

static void rk628_hdmirx_config_all(struct rk628_csi *csi)
{
	int ret;

	rk628_hdmirx_controller_setup(csi);
	ret = rk628_hdmirx_phy_setup(csi);
	if (ret >= 0) {
		rk628_csi_format_change(csi);
		csi->nosignal = false;
	}
}

static void rk628_csi_delayed_work_enable_hotplug(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct rk628_csi *csi = container_of(dwork, struct rk628_csi,
			delayed_work_enable_hotplug);
	bool plugin;

	mutex_lock(&csi->confctl_mutex);
	csi->avi_rcv_rdy = false;
	plugin = tx_5v_power_present(csi);
	dev_dbg(csi->dev, "%s: 5v_det:%d\n", __func__, plugin);
	if (plugin) {
		rk628_csi_enable_interrupts(csi, false);
		rk628_csi_set_hdmi_hdcp(csi, csi->enable_hdcp);
		rk628_hdmirx_hpd_ctrl(csi, true);
		rk628_hdmirx_config_all(csi);
		rk628_csi_enable_interrupts(csi, true);
		regmap_update_bits(csi->grf, GRF_SYSTEM_CON0,
				SW_I2S_DATA_OEN_MASK, SW_I2S_DATA_OEN(0));
		schedule_delayed_work(&csi->delayed_work_audio, 0);
	} else {
		rk628_csi_enable_interrupts(csi, false);
		enable_stream(csi, false);
		cancel_delayed_work(&csi->delayed_work_res_change);
		rk628_hdmirx_hpd_ctrl(csi, false);
		csi->nosignal = true;
		regmap_update_bits(csi->grf, GRF_SYSTEM_CON0,
				SW_I2S_DATA_OEN_MASK, SW_I2S_DATA_OEN(1));
		cancel_delayed_work(&csi->delayed_work_audio);
	}
	mutex_unlock(&csi->confctl_mutex);
}

static void rk628_delayed_work_res_change(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct rk628_csi *csi = container_of(dwork, struct rk628_csi,
			delayed_work_res_change);
	bool plugin;

	mutex_lock(&csi->confctl_mutex);
	csi->avi_rcv_rdy = false;
	plugin = tx_5v_power_present(csi);
	dev_dbg(csi->dev, "%s: 5v_det:%d\n", __func__, plugin);
	if (plugin) {
		dev_dbg(csi->dev, "res change, recfg ctrler and phy!\n");
		rk628_hdmirx_config_all(csi);
		rk628_csi_enable_interrupts(csi, true);
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

static void rk628_csi_set_hdmi_hdcp(struct rk628_csi *csi, bool en)
{
	u32 val;

	dev_dbg(csi->dev, "%s: %sable\n", __func__, en ? "en" : "dis");

	if (en) {
		regmap_read(csi->hdmirx_regmap, HDMI_RX_HDCP_CTRL, &val);
		if (!(val & HDCP_ENABLE_MASK))
			rk628_hdmi_hdcp_load_key(csi);
	} else {
		regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_HDCP_CTRL,
				HDCP_ENABLE_MASK |
				HDCP_ENC_EN_MASK,
				HDCP_ENABLE(0) |
				HDCP_ENC_EN(0));
	}
}

static void rk628_hdmirx_hpd_ctrl(struct rk628_csi *csi, bool en)
{
	u8 en_level, set_level;

	dev_dbg(csi->dev, "%s: %sable, hpd invert:%d\n", __func__,
			en ? "en" : "dis", csi->hpd_output_inverted);
	en_level = csi->hpd_output_inverted ? 0 : 1;
	set_level = en ? en_level : !en_level;
	regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_HDMI_SETUP_CTRL,
			HOT_PLUG_DETECT_MASK, HOT_PLUG_DETECT(set_level));
}

static void rk62_csi_reset(struct rk628_csi *csi)
{
	reset_control_assert(csi->rst_csi0);
	udelay(10);
	reset_control_deassert(csi->rst_csi0);

	regmap_write(csi->csi_regmap, CSITX_SYS_CTRL0_IMD, 0x1);
	usleep_range(1000, 1100);
	regmap_write(csi->csi_regmap, CSITX_SYS_CTRL0_IMD, 0x0);
}

static void enable_csitx(struct rk628_csi *csi)
{
	u32 i, ret, val;

	for (i = 0; i < CSITX_ERR_RETRY_TIMES; i++) {
		rk628_csi_set_csi(csi);
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

		dev_dbg(csi->dev,
			"%s csitx err, retry:%d, err status:%#x, ret:%d\n",
			__func__, i, val, ret);
	}

}

static void enable_stream(struct rk628_csi *csi, bool en)
{
	dev_dbg(csi->dev, "%s: %sable\n", __func__, en ? "en" : "dis");
	if (en) {
		rk628_hdmirx_vid_enable(csi, true);
		enable_csitx(csi);
	} else {
		rk628_hdmirx_vid_enable(csi, false);
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
	long long t_frm_st;
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

static void rk628_post_process_scaler_init(struct rk628_csi *csi,
					   const struct videomode *src,
					   const struct videomode *dst)
{
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
	dev_dbg(csi->dev, "dsp_frame_vst=%d, dsp_frame_hst=%d\n",
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

		dev_dbg(csi->dev, "horizontal scale down\n");
	} else if (src->hactive == dst->hactive) {
		scl_hor_mode = 0;
		scl_h_factor = 0;

		dev_dbg(csi->dev, "horizontal no scale\n");
	} else {
		scl_hor_mode = 1;
		scl_h_factor = ((src->hactive - 1) << 16) / (dst->hactive - 1);

		dev_dbg(csi->dev, "horizontal scale up\n");
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

		dev_dbg(csi->dev, "vertical scale down\n");
	} else if (src->vactive == dst->vactive) {
		scl_ver_mode = 0;
		scl_v_factor = 0;

		dev_dbg(csi->dev, "vertical no scale\n");
	} else {
		scl_ver_mode = 1;
		scl_v_factor = ((src->vactive - 1) << 16) / (dst->vactive - 1);

		dev_dbg(csi->dev, "vertical scale up\n");
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

static void rk628_post_process_setup(struct rk628_csi *csi)
{
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

	rk628_post_process_scaler_init(csi, &src, &dst);
}

static void rk628_csi_set_csi(struct rk628_csi *csi)
{
	u8 i, video_fmt;
	u8 lanes = csi->csi_lanes_in_use;
	u8 lane_num;
	u8 dphy_lane_en;
	u32 wc_usrdef, val, avi_pb = 0;
	u8 cnt = 0;

	lane_num = lanes - 1;
	dphy_lane_en = (1 << (lanes + 1)) - 1;
	wc_usrdef = csi->timings.bt.width * 2;

	rk62_csi_reset(csi);
	rk628_post_process_setup(csi);

	if (csi->txphy_pwron) {
		dev_dbg(csi->dev, "%s: txphy already power on, power off\n",
			__func__);
		mipi_dphy_power_off(csi);
		csi->txphy_pwron = false;
	}

	mipi_dphy_power_on(csi);
	csi->txphy_pwron = true;
	dev_dbg(csi->dev, "%s: txphy power on!\n", __func__);
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
	dev_dbg(csi->dev, "%s csi cofig done\n", __func__);

	for (i = 0; i < 100; i++) {
		regmap_read(csi->hdmirx_regmap, HDMI_RX_PDEC_AVI_PB, &val);
		dev_dbg(csi->dev, "%s PDEC_AVI_PB:%#x, avi_rcv_rdy:%d\n",
			__func__, val, csi->avi_rcv_rdy);
		if (val == avi_pb && csi->avi_rcv_rdy) {
			if (++cnt >= 2)
				break;
		} else {
			cnt = 0;
			avi_pb = val;
		}
		msleep(30);
	}
	video_fmt = (val & VIDEO_FORMAT_MASK) >> 5;
	dev_dbg(csi->dev, "%s PDEC_AVI_PB:%#x, video format:%d\n",
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
}

static int rk628_hdmirx_phy_power_on(struct rk628_csi *csi)
{
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
		dev_dbg(csi->dev, "rxphy already power on, power off!\n");
		ret = phy_power_off(csi->rxphy);
		if (ret)
			dev_err(csi->dev, "hdmi rxphy power off failed!\n");
		else
			csi->rxphy_pwron = false;
	}

	usleep_range(100, 100);
	if (csi->rxphy_pwron == false) {
		ret = phy_power_on(csi->rxphy);
		if (ret) {
			csi->rxphy_pwron = false;
			dev_err(csi->dev, "hdmi rxphy power on failed\n");
		} else {
			csi->rxphy_pwron = true;
		}
	}

	return ret;
}

static void rk628_hdmirx_vid_enable(struct rk628_csi *csi, bool en)
{
	dev_dbg(csi->dev, "%s: %sable\n", __func__, en ? "en" : "dis");
	if (en) {
		regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_DMI_DISABLE_IF,
				VID_ENABLE_MASK, VID_ENABLE(1));
	} else {
		regmap_update_bits(csi->hdmirx_regmap, HDMI_RX_DMI_DISABLE_IF,
				VID_ENABLE_MASK, VID_ENABLE(0));
	}
}

static void rk628_hdmirx_controller_reset(struct rk628_csi *csi)
{
	regmap_write(csi->hdmirx_regmap, HDMI_RX_DMI_SW_RST, 0x000101ff);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_DMI_DISABLE_IF, 0x00000000);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_DMI_DISABLE_IF, 0x0000017f);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_DMI_DISABLE_IF, 0x0001017f);
}

static void rk628_hdmirx_audio_setup(struct rk628_csi *csi)
{
	u32 audio_pll_n, audio_pll_cts;

	audio_pll_n = 5644;
	audio_pll_cts = 148500;

	clk_set_rate(csi->clk_hdmirx_aud, 5644800);

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
			AUD_MUTE_SEL(1)		|
			AUD_MUTE_MODE(1));

	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_PAO_CTRL,
			PAO_RATE(0));
	regmap_write(csi->hdmirx_regmap, HDMI_RX_AUD_CHEXTR_CTRL,
			AUD_LAYOUT_CTRL(1));

	/* audio detect */
	regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_AUDIODET_CTRL,
			AUDIODET_THRESHOLD(0));
}

static void rk628_csi_delayed_work_audio(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct rk628_csi *csi = container_of(dwork, struct rk628_csi,
			delayed_work_audio);
	u32 val;
	static int hdmirx_aud_clkrate = 5644800,
		   init_state = 256, pre_state, cur_state;
	u32 clkrate = 0, cts_decoded = 0, n_decoded = 0;
	u64 tmdsclk = 0, fs_audio = 0;
	static u64 pre_fs_audio;

	/* fout=128*fs=ftmds*N/CTS */
	regmap_read(csi->hdmirx_regmap, HDMI_RX_HDMI_CKM_RESULT, &clkrate);
	clkrate = clkrate & 0xffff;
	/* tmdsclk = (clkrate/1000) * 49500000 */
	tmdsclk = clkrate * (49500000 / 1000);
	regmap_read(csi->hdmirx_regmap, HDMI_RX_PDEC_ACR_CTS, &cts_decoded);
	regmap_read(csi->hdmirx_regmap, HDMI_RX_PDEC_ACR_N, &n_decoded);
	/* fs_audio = ((tmdsclk * n_decoded) / cts_decoded ) / 128 */
	if (cts_decoded != 0) {
		fs_audio = div_u64((tmdsclk * n_decoded), cts_decoded);
		fs_audio = div_u64(fs_audio, 128);
		fs_audio = div_u64(fs_audio + 50, 100);
		fs_audio *= 100;
	}
	dev_dbg(csi->dev,
		"%s: clkrate:%d tmdsclk:%llu, n_decoded:%d, cts_decoded:%d, fs_audio:%llu\n",
		__func__, clkrate, tmdsclk, n_decoded, cts_decoded, fs_audio);
	if ((fs_audio != 0) && (abs(fs_audio - pre_fs_audio) > 1000)) {
		hdmirx_aud_clkrate = 128 * fs_audio;
		switch (fs_audio) {
		case 88200:
			hdmirx_aud_clkrate = 11111000;
			break;
		case 96000:
			hdmirx_aud_clkrate = 12121000;
			break;
		case 176400:
			hdmirx_aud_clkrate = 22222000;
			break;
		case 192000:
			hdmirx_aud_clkrate = 23529000;
			break;
		default:
			break;
		}
		clk_set_rate(csi->clk_hdmirx_aud, hdmirx_aud_clkrate);
		dev_dbg(csi->dev,
			"%s: audo switch clk_hdmirx_aud to %d  fs_audio:%llu pre_fs_audio:%llu\n",
			__func__, hdmirx_aud_clkrate, fs_audio, pre_fs_audio);
		if (pre_fs_audio != 0) {
			regmap_write(csi->hdmirx_regmap,
					HDMI_RX_AUD_FIFO_ICLR, 0x1f);
			regmap_write(csi->hdmirx_regmap,
					HDMI_RX_AUD_FIFO_CTRL, 0x10001);
			regmap_write(csi->hdmirx_regmap,
					HDMI_RX_AUD_FIFO_CTRL, 0x10000);
		}
		pre_fs_audio = fs_audio;
		csi->fs_audio = fs_audio;
	}

	regmap_read(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_FILLSTS1, &cur_state);
	dev_dbg(csi->dev,
		"%s: HDMI_RX_AUD_FIFO_FILLSTS1:%#x, single offset:%d, total offset:%d\n",
		__func__, cur_state, cur_state-pre_state, cur_state-init_state);
	if (cur_state != 0)
		csi->audio_present = true;
	else
		csi->audio_present = false;
	if ((cur_state-init_state) > 16 && (cur_state-pre_state) > 0) {
		hdmirx_aud_clkrate += 10;
		clk_set_rate(csi->clk_hdmirx_aud, hdmirx_aud_clkrate);
		dev_dbg(csi->dev, "%s: (cur_state-init_state) > 16 hdmirx_aud_clkrate:%d\n",
				__func__, hdmirx_aud_clkrate);
	} else if ((cur_state != 0) && (cur_state-init_state) < -16 && (cur_state-pre_state) < 0) {
		hdmirx_aud_clkrate -= 10;
		clk_set_rate(csi->clk_hdmirx_aud, hdmirx_aud_clkrate);
		dev_dbg(csi->dev, "%s: (cur_state-init_state) < -16 hdmirx_aud_clkrate:%d\n",
				__func__, hdmirx_aud_clkrate);
	}
	pre_state = cur_state;

	regmap_read(csi->hdmirx_regmap, HDMI_RX_AUD_FIFO_ISTS, &val);
	dev_dbg(csi->dev, "%s: HDMI_RX_AUD_FIFO_ISTS:%#x\n", __func__, val);
	if ((val != 0x9) && ((val & 0x10) || (val & 0x8))) {
		regmap_write(csi->hdmirx_regmap,
				HDMI_RX_AUD_FIFO_ICLR, 0x1f);
		regmap_write(csi->hdmirx_regmap,
				HDMI_RX_AUD_FIFO_CTRL, 0x10001);
		/*msleep(1);*/
		regmap_write(csi->hdmirx_regmap,
				HDMI_RX_AUD_FIFO_CTRL, 0x10000);
		pre_state = cur_state = 0;

		dev_err(csi->dev, "%s: HDMI_RX_AUD_FIFO_ISTS:%#x, underflow or overflow\n",
				__func__, val);
	}

	schedule_delayed_work(&csi->delayed_work_audio, msecs_to_jiffies(1000));
}

static void rk628_hdmirx_controller_setup(struct rk628_csi *csi)
{
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

static bool rk628_rcv_supported_res(struct rk628_csi *csi, u32 width,
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
		dev_err(csi->dev, "%s do not support res wxh: %dx%d\n",
				__func__, width, height);
		return false;
	} else {
		return true;
	}
}

static int rk628_hdmirx_phy_setup(struct rk628_csi *csi)
{
	u32 i, cnt, val;
	u32 width, height, frame_width, frame_height, status;

	for (i = 0; i < RXPHY_CFG_MAX_TIMES; i++) {
		rk628_hdmirx_phy_power_on(csi);
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
			dev_dbg(csi->dev,
				"%s read wxh:%dx%d, total:%dx%d, SCDC_REGS1:%#x, cnt:%d\n",
				__func__, width, height, frame_width,
				frame_height, status, cnt);

			if (!tx_5v_power_present(csi)) {
				dev_info(csi->dev, "HDMI pull out, return!\n");
				return -1;
			}

			if (cnt >= 15)
				break;
		} while (((status & 0xfff) != 0xf00) ||
				(!rk628_rcv_supported_res(csi, width, height)));

		if (((status & 0xfff) != 0xf00) ||
				(!rk628_rcv_supported_res(csi, width, height))) {
			dev_err(csi->dev, "%s hdmi rxphy lock failed, retry:%d\n",
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

static void rk628_csi_initial_setup(struct rk628_csi *csi)
{
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
	rk628_hdmirx_controller_reset(csi);

	def_edid.pad = 0;
	def_edid.start_block = 0;
	def_edid.blocks = 2;
	def_edid.edid = edid_init_data;
	rk628_csi_s_edid(csi, &def_edid);
	rk628_csi_set_hdmi_hdcp(csi, false);
	rk628_hdmirx_audio_setup(csi);

	mipi_dphy_reset(csi);
	mipi_dphy_power_on(csi);
	csi->txphy_pwron = true;
	if (tx_5v_power_present(csi))
		schedule_delayed_work(&csi->delayed_work_enable_hotplug, msecs_to_jiffies(1000));
}

static void rk628_csi_format_change(struct rk628_csi *csi)
{
	struct v4l2_dv_timings timings;

	if (rk628_csi_get_detected_timings(csi, &timings) == 0)
		rk628_csi_s_dv_timings(csi, &timings);
}

static void rk628_csi_enable_interrupts(struct rk628_csi *csi, bool en)
{
	u32 val_a, val_b;

	dev_dbg(csi->dev, "%s: %sable\n", __func__, en ? "en" : "dis");
	/* clr irq */
	regmap_write(csi->hdmirx_regmap, HDMI_RX_MD_ICLR, 0xffffffff);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_ICLR, 0xffffffff);

	if (en) {
		regmap_write(csi->hdmirx_regmap, HDMI_RX_MD_IEN_SET,
			VACT_LIN_ENSET | HACT_PIX_ENSET);
		regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_IEN_SET,
				AVI_RCV_ENSET);
	} else {
		regmap_write(csi->hdmirx_regmap, HDMI_RX_MD_IEN_CLR,
				0xffffffff);
		regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_IEN_CLR,
				0xffffffff);
	}
	usleep_range(5000, 5000);

	regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_IEN, &val_a);
	regmap_read(csi->hdmirx_regmap, HDMI_RX_PDEC_IEN, &val_b);
	dev_dbg(csi->dev, "%s MD_IEN:%#x, PDEC_IEN:%#x\n",
			__func__, val_a, val_b);
}

static int rk628_csi_isr(struct rk628_csi *csi, u32 status, bool *handled)
{
	u32 md_ints, pdec_ints, hact, vact;
	bool plugin;

	if (handled == NULL) {
		dev_err(csi->dev, "handled NULL, err return!\n");
		return -1;
	}

	regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_ISTS, &md_ints);
	regmap_read(csi->hdmirx_regmap, HDMI_RX_PDEC_ISTS, &pdec_ints);
	plugin = tx_5v_power_present(csi);
	dev_dbg(csi->dev, "%s: md_ints: %#x, pdec_ints:%#x, plugin: %d\n",
			__func__, md_ints, pdec_ints, plugin);

	if ((md_ints & (VACT_LIN_ISTS | HACT_PIX_ISTS)) && plugin) {
		regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_HACT_PX, &hact);
		regmap_read(csi->hdmirx_regmap, HDMI_RX_MD_VAL, &vact);
		dev_dbg(csi->dev, "HACT:%#x, VACT:%#x\n", hact, vact);

		rk628_csi_enable_interrupts(csi, false);
		enable_stream(csi, false);
		csi->nosignal = true;
		schedule_delayed_work(&csi->delayed_work_res_change, HZ / 2);

		dev_dbg(csi->dev, "%s: hact/vact change, md_ints: %#x\n",
				__func__, (u32)(md_ints & (VACT_LIN_ISTS |
					HACT_PIX_ISTS)));
		*handled = true;
	}

	if ((pdec_ints & AVI_RCV_ISTS) && plugin) {
		dev_dbg(csi->dev, "%s: AVI RCV INT!\n", __func__);
		csi->avi_rcv_rdy = true;
		/* After get the AVI_RCV interrupt state, disable interrupt. */
		regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_IEN_CLR,
				0xffffffff);

		*handled = true;
	}

	if (*handled != true)
		dev_dbg(csi->dev, "%s: unhandled interrupt!\n", __func__);

	/* clear interrupts */
	regmap_write(csi->hdmirx_regmap, HDMI_RX_MD_ICLR, 0xffffffff);
	regmap_write(csi->hdmirx_regmap, HDMI_RX_PDEC_ICLR, 0xffffffff);

	return 0;
}

static irqreturn_t rk628_csi_irq_handler(int irq, void *dev_id)
{
	struct rk628_csi *csi = dev_id;
	bool handled = false;

	rk628_csi_isr(csi, 0, &handled);

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static void rk628_csi_irq_poll_timer(unsigned long arg)
{
	struct rk628_csi *csi = (struct rk628_csi *)arg;

	schedule_work(&csi->work_i2c_poll);
	mod_timer(&csi->timer, jiffies + msecs_to_jiffies(POLL_INTERVAL_MS));
}

static void rk628_csi_work_i2c_poll(struct work_struct *work)
{
	struct rk628_csi *csi = container_of(work, struct rk628_csi,
			work_i2c_poll);

	rk628_csi_format_change(csi);
}

static int rk628_csi_s_dv_timings(struct rk628_csi *csi,
		struct v4l2_dv_timings *timings)
{
	csi->timings = *timings;
	enable_stream(csi, false);

	return 0;
}

static int rk628_csi_s_edid(struct rk628_csi *csi,
				struct v4l2_subdev_edid *edid)
{
	u16 edid_len = edid->blocks * EDID_BLOCK_SIZE;
	u32 i, val;

	dev_dbg(csi->dev, "%s, pad %d, start block %d, blocks %d\n",
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

	rk628_hdmirx_hpd_ctrl(csi, false);

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

	for (i = 0; i < edid_len; i++)
		regmap_write(csi->key_regmap, EDID_BASE + i * 4, edid->edid[i]);

	/* read out for debug */
	if (debug) {
		pr_info("%s: Read EDID: ======\n", __func__);
		for (i = 0; i < edid_len; i++) {
			regmap_read(csi->key_regmap, EDID_BASE + i * 4, &val);
			pr_info("0x%02x ", val);
			if ((i + 1) % 8 == 0)
				pr_info("\n");
		}
		pr_info("%s: ======\n", __func__);
	}

	/* edid access by RX's i2c, i2c slave addr: 0x0 */
	regmap_update_bits(csi->grf, GRF_SYSTEM_CON0,
			SW_ADAPTER_I2CSLADR_MASK |
			SW_EDID_MODE_MASK,
			SW_ADAPTER_I2CSLADR(0) |
			SW_EDID_MODE(0));
	csi->edid_blocks_written = edid->blocks;
	udelay(100);

	if (tx_5v_power_present(csi))
		rk628_hdmirx_hpd_ctrl(csi, true);

	return 0;
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

	testif_test_code_write(csi, test_code);
	testif_test_data_write(csi, test_data);
	monitor_data = testif_get_data(csi);

	dev_dbg(csi->dev, "test_code=0x%02x, ", test_code);
	dev_dbg(csi->dev, "test_data=0x%02x, ", test_data);
	dev_dbg(csi->dev, "monitor_data=0x%02x\n", monitor_data);

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

	if ((csi->timings.bt.width == 3840) &&
			(csi->timings.bt.height == 2160)) {
		csi->lane_mbps = MIPI_DATARATE_MBPS_HIGH;
	} else {
		csi->lane_mbps = MIPI_DATARATE_MBPS_LOW;
	}

	bus_width =  csi->lane_mbps << 8;
	bus_width |= COMBTXPHY_MODULEA_EN;
	dev_dbg(csi->dev, "%s mipi bitrate:%llu mbps\n", __func__,
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

static irqreturn_t plugin_detect_irq(int irq, void *dev_id)
{
	struct rk628_csi *csi = dev_id;

	/* control hpd after 50ms */
	schedule_delayed_work(&csi->delayed_work_enable_hotplug, HZ / 20);
	tx_5v_power_present(csi);

	return IRQ_HANDLED;
}

static int rk628_csi_probe_of(struct rk628_csi *csi)
{
	struct device *dev = csi->dev;
	int ret = -EINVAL;
	bool hdcp1x_enable = false;

	csi->clk_hdmirx = devm_clk_get(dev, "hdmirx");
	if (IS_ERR(csi->clk_hdmirx)) {
		ret = PTR_ERR(csi->clk_hdmirx);
		dev_err(csi->dev, "failed to get clk_hdmirx: %d\n", ret);
		return ret;
	}

	csi->clk_imodet = devm_clk_get(dev, "imodet");
	if (IS_ERR(csi->clk_imodet)) {
		ret = PTR_ERR(csi->clk_imodet);
		dev_err(csi->dev, "failed to get clk_imodet: %d\n", ret);
		return ret;
	}

	csi->clk_hdmirx_aud = devm_clk_get(dev, "hdmirx_aud");
	if (IS_ERR(csi->clk_hdmirx_aud)) {
		ret = PTR_ERR(csi->clk_hdmirx_aud);
		dev_err(csi->dev, "failed to get clk_hdmirx_aud: %d\n", ret);
		return ret;
	}

	csi->clk_hdmirx_cec = devm_clk_get(dev, "hdmirx_cec");
	if (IS_ERR(csi->clk_hdmirx_cec)) {
		ret = PTR_ERR(csi->clk_hdmirx_cec);
		dev_err(csi->dev, "failed to get clk_hdmirx_cec: %d\n", ret);
		return ret;
	}

	csi->clk_vop = devm_clk_get(dev, "vop");
	if (IS_ERR(csi->clk_vop)) {
		ret = PTR_ERR(csi->clk_vop);
		dev_err(csi->dev, "failed to get clk_vop: %d\n", ret);
		return ret;
	}

	csi->clk_rx_read = devm_clk_get(dev, "rx_read");
	if (IS_ERR(csi->clk_rx_read)) {
		ret = PTR_ERR(csi->clk_rx_read);
		dev_err(csi->dev, "failed to get clk_rx_read: %d\n", ret);
		return ret;
	}

	csi->clk_csi0 = devm_clk_get(dev, "csi0");
	if (IS_ERR(csi->clk_csi0)) {
		ret = PTR_ERR(csi->clk_csi0);
		dev_err(csi->dev, "failed to get clk_csi0: %d\n", ret);
		return ret;
	}

	csi->clk_i2s_mclk = devm_clk_get(dev, "i2s_mclk");
	if (IS_ERR(csi->clk_i2s_mclk)) {
		ret = PTR_ERR(csi->clk_i2s_mclk);
		csi->clk_i2s_mclk = NULL;
		dev_warn(csi->dev, "i2s_mclk is not configured: %d\n", ret);
	}

	csi->rst_hdmirx = of_reset_control_get(dev->of_node, "hdmirx");
	if (IS_ERR(csi->rst_hdmirx)) {
		ret = PTR_ERR(csi->rst_hdmirx);
		dev_err(csi->dev, "failed to get rst_hdmirx control: %d\n", ret);
		return ret;
	}

	csi->rst_hdmirx_pon = of_reset_control_get(dev->of_node, "hdmirx_pon");
	if (IS_ERR(csi->rst_hdmirx_pon)) {
		ret = PTR_ERR(csi->rst_hdmirx_pon);
		dev_err(csi->dev, "failed to get rst_hdmirx_pon control: %d\n", ret);
		return ret;
	}

	csi->rst_decoder = of_reset_control_get(dev->of_node, "decoder");
	if (IS_ERR(csi->rst_decoder)) {
		ret = PTR_ERR(csi->rst_decoder);
		dev_err(csi->dev, "failed to get rst_decoder control: %d\n", ret);
		return ret;
	}

	csi->rst_clk_rx = of_reset_control_get(dev->of_node, "clk_rx");
	if (IS_ERR(csi->rst_clk_rx)) {
		ret = PTR_ERR(csi->rst_clk_rx);
		dev_err(csi->dev, "failed to get rst_clk_rx control: %d\n", ret);
		return ret;
	}

	csi->rst_vop = of_reset_control_get(dev->of_node, "vop");
	if (IS_ERR(csi->rst_vop)) {
		ret = PTR_ERR(csi->rst_vop);
		dev_err(csi->dev, "failed to get rst_vop control: %d\n", ret);
		return ret;
	}

	csi->rst_csi0 = of_reset_control_get(dev->of_node, "csi0");
	if (IS_ERR(csi->rst_csi0)) {
		ret = PTR_ERR(csi->rst_csi0);
		dev_err(csi->dev, "failed to get rst_csi0 control: %d\n", ret);
		return ret;
	}

	csi->power_gpio = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_HIGH);
	if (IS_ERR(csi->power_gpio)) {
		dev_err(csi->dev, "failed to get power gpio\n");
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
		dev_err(csi->dev, "failed to get hdmirx det gpio\n");
		ret = PTR_ERR(csi->plugin_det_gpio);
		return ret;
	}

	csi->rxphy = devm_phy_get(dev, "combrxphy");
	if (IS_ERR(csi->rxphy)) {
		ret = PTR_ERR(csi->rxphy);
		dev_err(csi->dev, "failed to get rxphy: %d\n", ret);
		return ret;
	}

	csi->txphy = devm_phy_get(dev, "combtxphy");
	if (IS_ERR(csi->txphy)) {
		ret = PTR_ERR(csi->txphy);
		dev_err(csi->dev, "failed to get txphy: %d\n", ret);
		return ret;
	}

	if (of_property_read_bool(dev->of_node, "hdcp-enable"))
		hdcp1x_enable = true;

	csi->csi_lanes_in_use = USE_4_LANES;
	csi->enable_hdcp = hdcp1x_enable;
	csi->rxphy_pwron = false;
	csi->txphy_pwron = false;
	csi->nosignal = true;
	csi->avi_rcv_rdy = false;
	csi->stream_state = 0;
	ret = 0;

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
	regmap_reg_range(HDMI_RX_PDEC_AUDIODET_CTRL, HDMI_RX_PDEC_AUDIODET_CTRL),
	regmap_reg_range(HDMI_RX_PDEC_ERR_FILTER, HDMI_RX_PDEC_ASP_CTRL),
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

static int rk628_create_class_attr(struct rk628_csi *csi)
{
	int ret = -1;

	csi->rk628_class = class_create(THIS_MODULE, "rk628csi");
	if (IS_ERR(csi->rk628_class)) {
		ret = -ENOMEM;
		dev_err(csi->dev, "failed to create rk628csi class!\n");
		return ret;
	}

	ret = class_create_file(csi->rk628_class, &class_attr_resolution);
	if (ret) {
		dev_err(csi->dev, "failed to create attr resolution\n");
		goto err_res;
	}

	ret = class_create_file(csi->rk628_class, &class_attr_streamen);
	if (ret) {
		dev_err(csi->dev, "failed to create attr streamen\n");
		goto err_stream;
	}

	return ret;

err_stream:
	class_remove_file(csi->rk628_class, &class_attr_resolution);
err_res:
	class_destroy(csi->rk628_class);
	return ret;
}

static int rk628_csi_probe(struct platform_device *pdev)
{
	struct rk628_csi *csi;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	int err;
	u32 val, module_index;

	struct rk628 *rk628 = dev_get_drvdata(pdev->dev.parent);
	struct i2c_client *client = rk628->client;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	if (!of_device_is_available(dev->of_node))
		return -ENODEV;

	err = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &module_index);
	if (err == 0) {
		dev_err(dev, "This driver is adapted to camerahal1, exit!\n");
		return -ENODEV;
	}

	csi = devm_kzalloc(dev, sizeof(*csi), GFP_KERNEL);
	if (!csi)
		return -ENOMEM;

	csi->dev = dev;
	csi->parent = rk628;
	platform_set_drvdata(pdev, csi);

	csi->grf = rk628->grf;
	if (!csi->grf)
		return -ENODEV;

	csi->hdmirx_irq = platform_get_irq(pdev, 1);
	if (csi->hdmirx_irq < 0)
		return csi->hdmirx_irq;

	csi->hpd_output_inverted = of_property_read_bool(node,
			"hpd-output-inverted");

	csi->i2c_client = client;
	err = rk628_csi_probe_of(csi);
	if (err) {
		dev_err(csi->dev, "rk628_csi_probe_of failed! err:%d\n", err);
		return err;
	}

	csi->hdmirx_regmap = devm_regmap_init_i2c(rk628->client,
						 &rk628_hdmirx_regmap_cfg);
	csi->csi_regmap = devm_regmap_init_i2c(rk628->client,
						 &rk628_csi_regmap_cfg);
	csi->key_regmap = devm_regmap_init_i2c(rk628->client,
						 &rk628_key_regmap_cfg);

	/* i2c access, read chip id*/
	err = regmap_read(csi->csi_regmap, CSITX_CSITX_VERSION, &val);
	if (err) {
		dev_err(csi->dev, "i2c access failed! err:%d\n", err);
		return -ENODEV;
	}
	dev_dbg(csi->dev, "CSITX VERSION: %#x\n", val);

	mutex_init(&csi->confctl_mutex);
	err = rk628_create_class_attr(csi);
	if (err) {
		dev_err(csi->dev, "create class attr failed! err:%d\n", err);
		return -ENODEV;
	}

	INIT_DELAYED_WORK(&csi->delayed_work_enable_hotplug,
			rk628_csi_delayed_work_enable_hotplug);
	INIT_DELAYED_WORK(&csi->delayed_work_res_change,
			rk628_delayed_work_res_change);
	INIT_DELAYED_WORK(&csi->delayed_work_audio,
			rk628_csi_delayed_work_audio);
	rk628_csi_initial_setup(csi);

	if (csi->hdmirx_irq) {
		dev_dbg(csi->dev, "cfg hdmirx irq!\n");
		err = devm_request_threaded_irq(dev, csi->hdmirx_irq, NULL,
				rk628_csi_irq_handler, IRQF_TRIGGER_HIGH |
				IRQF_ONESHOT, "rk628_csi", csi);
		if (err) {
			dev_err(csi->dev, "request rk628-csi irq failed! err:%d\n",
					err);
			goto err_work_queues;
		}
	} else {
		dev_dbg(csi->dev, "no irq, cfg poll!\n");
		INIT_WORK(&csi->work_i2c_poll,
			  rk628_csi_work_i2c_poll);
		csi->timer.data = (unsigned long)csi;
		csi->timer.function = rk628_csi_irq_poll_timer;
		csi->timer.expires = jiffies +
			msecs_to_jiffies(POLL_INTERVAL_MS);
		add_timer(&csi->timer);
	}

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

	rk628_csi_enable_interrupts(csi, tx_5v_power_present(csi));
	dev_info(csi->dev, "%s found @ 0x%x (%s)\n", client->name,
		  client->addr << 1, client->adapter->name);
	g_csi = csi;

	return 0;

err_work_queues:
	if (!csi->hdmirx_irq)
		flush_work(&csi->work_i2c_poll);
	cancel_delayed_work(&csi->delayed_work_enable_hotplug);
	cancel_delayed_work(&csi->delayed_work_res_change);
	cancel_delayed_work(&csi->delayed_work_audio);
	mutex_destroy(&csi->confctl_mutex);
	return err;
}

static int rk628_csi_remove(struct platform_device *pdev)
{
	struct rk628_csi *csi = platform_get_drvdata(pdev);

	if (!csi->hdmirx_irq) {
		del_timer_sync(&csi->timer);
		flush_work(&csi->work_i2c_poll);
	}
	cancel_delayed_work_sync(&csi->delayed_work_enable_hotplug);
	cancel_delayed_work_sync(&csi->delayed_work_res_change);
	cancel_delayed_work_sync(&csi->delayed_work_audio);

	if (csi->rxphy_pwron)
		phy_power_off(csi->rxphy);
	if (csi->txphy_pwron)
		mipi_dphy_power_off(csi);

	mutex_destroy(&csi->confctl_mutex);

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
		.name = "rk628-csi-hal1",
		.of_match_table = of_match_ptr(rk628_csi_of_match),
	},
	.probe	= rk628_csi_probe,
	.remove = rk628_csi_remove,
};
module_platform_driver(rk628_csi_driver);

MODULE_DESCRIPTION("Rockchip RK628 HDMI to MIPI CSI-2 bridge driver for camerahal1");
MODULE_AUTHOR("Dingxian Wen <shawn.wen@rock-chips.com>");
MODULE_LICENSE("GPL v2");
