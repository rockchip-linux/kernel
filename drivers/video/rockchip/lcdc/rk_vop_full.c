/*
 * drivers/video/rockchip/lcdc/rk_vop_full.c
 *
 * Copyright (C) 2016 ROCKCHIP, Inc.
 * Author: hjc <hjc@rock-chips.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/rockchip-iovmm.h>
#include <asm/div64.h>
#include <linux/uaccess.h>
#include <linux/rockchip/cpu.h>
#include <linux/rockchip/iomap.h>
#include <linux/rockchip/grf.h>
#include <linux/rockchip/common.h>
#include <dt-bindings/clock/rk_system_status.h>

#include "rk_vop_full.h"

/*#define CONFIG_RK_FPGA 1*/
#define VOP_CHIP(dev)	(dev->data->chip_type)

static int dbg_thresd;
module_param(dbg_thresd, int, S_IRUGO | S_IWUSR);

#define DBG(level, x...) do {			\
	if (unlikely(dbg_thresd >= level))	\
		pr_info(x);\
	} while (0)

static struct rk_lcdc_win rk322xh_vop_win[] = {
	{ .name = "win0",
	  .id = 0,
	  .property.feature = SUPPORT_WIN_IDENTIFY | SUPPORT_HW_EXIST |
				SUPPORT_SCALE | SUPPORT_YUV |
				SUPPORT_YUV10BIT,
	  .property.max_input_x = 4096,
	  .property.max_input_y = 2304},
	{ .name = "win1",
	  .id = 1,
	  .property.feature = SUPPORT_WIN_IDENTIFY | SUPPORT_HW_EXIST |
				SUPPORT_SCALE | SUPPORT_YUV |
				SUPPORT_YUV10BIT,
	  .property.max_input_x = 4096,
	  .property.max_input_y = 2304},
	{ .name = "win2",
	  .id = 2,
	  .property.feature = SUPPORT_WIN_IDENTIFY | SUPPORT_HW_EXIST |
				SUPPORT_SCALE | SUPPORT_YUV |
				SUPPORT_YUV10BIT,
	  .property.max_input_x = 4096,
	  .property.max_input_y = 2304},
	{
	  .name = "hwc",
	  .id = 3,
	  .property.feature = SUPPORT_WIN_IDENTIFY | SUPPORT_HW_EXIST |
				SUPPORT_HWC_LAYER,
	  .property.max_input_x = 128,
	  .property.max_input_y = 128
	}
};

static const struct vop_data rk322xh_data = {
	.chip_type = VOP_RK322XH,
	.win = rk322xh_vop_win,
	.n_wins = ARRAY_SIZE(rk322xh_vop_win),
};

#if defined(CONFIG_OF)
static const struct of_device_id vop_dt_ids[] = {
	{.compatible = "rockchip,rk322xh-lcdc",
	 .data = &rk322xh_data, },
	{}
};
#endif

static const u32 sdr2hdr_bt1886eotf_yn_for_bt2020[65] = {
	0,
	1820,   3640,   5498,   7674,
	10256,  13253,  16678,  20539,
	24847,  29609,  34833,  40527,
	46699,  53354,  60499,  68141,
	76285,  84937,  94103,  103787,
	108825, 113995, 119296, 124731,
	130299, 136001, 141837, 147808,
	153915, 160158, 166538, 173055,
	176365, 179709, 183089, 186502,
	189951, 193434, 196952, 200505,
	204093, 207715, 211373, 215066,
	218795, 222558, 226357, 230191,
	232121, 234060, 236008, 237965,
	239931, 241906, 243889, 245882,
	247883, 249894, 251913, 253941,
	255978, 258024, 260079, 262143,
};

static const u32 sdr2hdr_bt1886eotf_yn_for_hdr[65] = {
	0,
	5,     21,    49,    91,
	150,   225,   320,   434,
	569,   726,   905,   1108,
	1336,  1588,  1866,  2171,
	2502,  2862,  3250,  3667,
	3887,  4114,  4349,  4591,
	4841,  5099,  5364,  5638,
	5920,  6209,  6507,  6812,
	6968,  7126,  7287,  7449,
	7613,  7779,  7948,  8118,
	8291,  8466,  8643,  8822,
	9003,  9187,  9372,  9560,
	9655,  9750,  9846,  9942,
	10039, 10136, 10234, 10333,
	10432, 10531, 10631, 10732,
	10833, 10935, 11038, 11141,
};

static const u32 sdr2hdr_st2084oetf_yn_for_bt2020[65] = {
	0,
	0,     0,     1,     2,
	4,     6,     9,     18,
	27,    36,    72,    108,
	144,   180,   216,   252,
	288,   360,   432,   504,
	576,   648,   720,   792,
	864,   1008,  1152,  1296,
	1444,  1706,  1945,  2166,
	2372,  2566,  2750,  2924,
	3251,  3553,  3834,  4099,
	4350,  4588,  4816,  5035,
	5245,  5447,  5832,  6194,
	6536,  6862,  7173,  7471,
	7758,  8035,  8560,  9055,
	9523,  9968,  10800, 11569,
	12963, 14210, 15347, 16383,
};

static const u32 sdr2hdr_st2084oetf_yn_for_hdr[65] = {
	0,
	668,   910,   1217,  1600,
	2068,  2384,  2627,  3282,
	3710,  4033,  4879,  5416,
	5815,  6135,  6401,  6631,
	6833,  7176,  7462,  7707,
	7921,  8113,  8285,  8442,
	8586,  8843,  9068,  9268,
	9447,  9760,  10027, 10259,
	10465, 10650, 10817, 10971,
	11243, 11480, 11689, 11877,
	12047, 12202, 12345, 12477,
	12601, 12716, 12926, 13115,
	13285, 13441, 13583, 13716,
	13839, 13953, 14163, 14350,
	14519, 14673, 14945, 15180,
	15570, 15887, 16153, 16383,
};

static const u32 sdr2hdr_st2084oetf_dxn_pow2[64] = {
	0,  0,  1,  2,
	3,  3,  3,  5,
	5,  5,  7,  7,
	7,  7,  7,  7,
	7,  8,  8,  8,
	8,  8,  8,  8,
	8,  9,  9,  9,
	9,  10, 10, 10,
	10, 10, 10, 10,
	11, 11, 11, 11,
	11, 11, 11, 11,
	11, 11, 12, 12,
	12, 12, 12, 12,
	12, 12, 13, 13,
	13, 13, 14, 14,
	15, 15, 15, 15,
};

static const u32 sdr2hdr_st2084oetf_dxn[64] = {
	1,     1,     2,     4,
	8,     8,     8,     32,
	32,    32,    128,   128,
	128,   128,   128,   128,
	128,   256,   256,   256,
	256,   256,   256,   256,
	256,   512,   512,   512,
	512,   1024,  1024,  1024,
	1024,  1024,  1024,  1024,
	2048,  2048,  2048,  2048,
	2048,  2048,  2048,  2048,
	2048,  2048,  4096,  4096,
	4096,  4096,  4096,  4096,
	4096,  4096,  8192,  8192,
	8192,  8192,  16384, 16384,
	32768, 32768, 32768, 32768,
};

static const u32 sdr2hdr_st2084oetf_xn[63] = {
	1,      2,      4,      8,
	16,     24,     32,     64,
	96,     128,    256,    384,
	512,    640,    768,    896,
	1024,   1280,   1536,   1792,
	2048,   2304,   2560,   2816,
	3072,   3584,   4096,   4608,
	5120,   6144,   7168,   8192,
	9216,   10240,  11264,  12288,
	14336,  16384,  18432,  20480,
	22528,  24576,  26624,  28672,
	30720,  32768,  36864,  40960,
	45056,  49152,  53248,  57344,
	61440,  65536,  73728,  81920,
	90112,  98304,  114688, 131072,
	163840, 196608, 229376,
};

static const u32 hdr2sdr_eetf_yn[33] = {
	647,
	945,  1254, 1573, 1903,
	2241, 2586, 2940, 3299,
	3664, 4035, 4410, 4789,
	5171, 5556, 5943, 6333,
	6723, 7100, 7458, 7795,
	8110, 8402, 8670, 8913,
	9131, 9323, 9487, 9624,
	9731, 9809, 9856, 9872,
};

static const u32 hdr2sdr_bt1886oetf_yn[33] = {
	0,
	0,     0,     0,     0,
	0,     57,    295,   612,
	1036,  1602,  2358,  2912,
	3366,  3758,  4106,  4422,
	4712,  5235,  5700,  6122,
	6509,  7207,  7828,  8390,
	8908,  9839,  10668, 11419,
	12109, 13353, 14459, 16383,
};

static const u32 hdr2sdr_sat_yn[9] = {
	0,
	1792, 3584, 3472, 2778,
	2083, 1389, 694,  0,
};

static void vop_load_hdr2sdr_table(struct vop_device *vop_dev)
{
	int i = 0;
	u32 hdr2sdr_eetf_oetf_yn[33];

	for (i = 0; i < 33; i++)
		hdr2sdr_eetf_oetf_yn[i] =
			hdr2sdr_eetf_yn[i] + (hdr2sdr_bt1886oetf_yn[i] << 16);
	vop_writel(vop_dev, EETF_OETF_Y0, hdr2sdr_eetf_oetf_yn[0]);
	for (i = 0; i < 32; i++)
		vop_writel(vop_dev, EETF_OETF_Y1 + (i << 2),
			   hdr2sdr_eetf_oetf_yn[i + 1]);

	vop_writel(vop_dev, SAT_Y0, hdr2sdr_sat_yn[0]);

	for (i = 0; i < 8; i++)
		vop_writel(vop_dev, SAT_Y1 + (i << 2), hdr2sdr_sat_yn[i + 1]);
}

static void vop_load_sdr2hdr_table(struct vop_device *vop_dev, int cmd)
{
	int i = 0;
	u32 sdr2hdr_eotf_oetf_yn[65];
	u32 sdr2hdr_oetf_dx_dxpow[64];

	for (i = 0; i < 65; i++) {
		if (cmd == SDR2HDR_FOR_BT2020) {
			sdr2hdr_eotf_oetf_yn[i] =
				sdr2hdr_bt1886eotf_yn_for_bt2020[i] +
				(sdr2hdr_st2084oetf_yn_for_bt2020[i] << 18);
		} else {
			sdr2hdr_eotf_oetf_yn[i] =
				sdr2hdr_bt1886eotf_yn_for_hdr[i] +
				(sdr2hdr_st2084oetf_yn_for_hdr[i] << 18);
		}
	}

	for (i = 0; i < 64; i++)
		sdr2hdr_oetf_dx_dxpow[i] = sdr2hdr_st2084oetf_dxn[i] +
				(sdr2hdr_st2084oetf_dxn_pow2[i] << 16);
	vop_writel(vop_dev, EOTF_OETF_Y0, sdr2hdr_eotf_oetf_yn[0]);
	for (i = 0; i < 64; i++)
		vop_writel(vop_dev, EOTF_OETF_Y1 + (i << 2),
			   sdr2hdr_eotf_oetf_yn[i + 1]);

	for (i = 0; i < 64; i++)
		vop_writel(vop_dev, OETF_DX_DXPOW1 + (i << 2),
			   sdr2hdr_oetf_dx_dxpow[i]);

	for (i = 0; i < 63; i++)
		vop_writel(vop_dev, OETF_XN1 + (i << 2),
			   sdr2hdr_st2084oetf_xn[i]);
}

static int vop_set_bcsh(struct rk_lcdc_driver *dev_drv, bool enable);

static int vop_clk_enable(struct vop_device *vop_dev)
{
	if (!vop_dev->clk_on) {
		if (vop_dev->hclk)
			clk_prepare_enable(vop_dev->hclk);
		if (vop_dev->dclk)
			clk_prepare_enable(vop_dev->dclk);
		if (vop_dev->aclk)
			clk_prepare_enable(vop_dev->aclk);
		if (vop_dev->hclk_noc)
			clk_prepare_enable(vop_dev->hclk_noc);
		if (vop_dev->aclk_noc)
			clk_prepare_enable(vop_dev->aclk_noc);
		spin_lock(&vop_dev->reg_lock);
		vop_dev->clk_on = 1;
		spin_unlock(&vop_dev->reg_lock);
	}

	return 0;
}

static int vop_clk_disable(struct vop_device *vop_dev)
{
	if (vop_dev->clk_on) {
		spin_lock(&vop_dev->reg_lock);
		vop_dev->clk_on = 0;
		spin_unlock(&vop_dev->reg_lock);
		msleep(25);
		if (vop_dev->dclk)
			clk_disable_unprepare(vop_dev->dclk);
		if (vop_dev->hclk)
			clk_disable_unprepare(vop_dev->hclk);
		if (vop_dev->aclk)
			clk_disable_unprepare(vop_dev->aclk);
		if (vop_dev->hclk_noc)
			clk_disable_unprepare(vop_dev->hclk_noc);
		if (vop_dev->aclk_noc)
			clk_disable_unprepare(vop_dev->aclk_noc);
	}

	return 0;
}

static int vop_disable_irq(struct vop_device *vop_dev)
{
	if (likely(vop_dev->clk_on)) {
		spin_lock(&vop_dev->reg_lock);
		vop_writel(vop_dev, INTR_EN0, 0xffff0000);
		vop_writel(vop_dev, INTR_EN1, 0xffff0000);
		vop_writel(vop_dev, INTR_CLEAR0, 0xffffffff);
		vop_writel(vop_dev, INTR_CLEAR1, 0xffffffff);
		vop_cfg_done(vop_dev);
		spin_unlock(&vop_dev->reg_lock);
	};

	return 0;
}

static int vop_reg_dump(struct rk_lcdc_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	int i;

	pr_info("lcdc reg:\n");
	for (i = 0; i <= 0x200 >> 2; i++) {
		pr_info("0x%04x: ", i * 16 + 0x20020000);
		pr_info("%08x  %08x  %08x  %08x\n",
			vop_readl(vop_dev, i * 16 + 0x0),
			vop_readl(vop_dev, i * 16 + 0x4),
			vop_readl(vop_dev, i * 16 + 0x8),
			vop_readl(vop_dev, i * 16 + 0xc));
	}

	for (i = 0x600; i <= 0x650 >> 2; i++) {
		pr_info("0x%04x: ", i * 16 + 0x20020000);
		pr_info("%08x  %08x  %08x  %08x\n",
			vop_readl(vop_dev, i * 16 + 0x0),
			vop_readl(vop_dev, i * 16 + 0x4),
			vop_readl(vop_dev, i * 16 + 0x8),
			vop_readl(vop_dev, i * 16 + 0xc));
	}

	for (i = SDR2HDR_CTRL; i <= 0x730 >> 2; i++) {
		pr_info("0x%04x: ", i * 16 + 0x20020000);
		pr_info("%08x  %08x  %08x  %08x\n",
			vop_readl(vop_dev, i * 16 + 0x0),
			vop_readl(vop_dev, i * 16 + 0x4),
			vop_readl(vop_dev, i * 16 + 0x8),
			vop_readl(vop_dev, i * 16 + 0xc));
	}

	for (i = HDR2SDR_CTRL; i <= 0xa30 >> 2; i++) {
		pr_info("0x%04x: ", i * 16 + 0x20020000);
		pr_info("%08x  %08x  %08x  %08x\n",
			vop_readl(vop_dev, i * 16 + 0x0),
			vop_readl(vop_dev, i * 16 + 0x4),
			vop_readl(vop_dev, i * 16 + 0x8),
			vop_readl(vop_dev, i * 16 + 0xc));
	}
	return 0;
}

#define WIN_EN(id)		\
static int win##id##_enable(struct vop_device *vop_dev, int en)	\
{ \
	spin_lock(&vop_dev->reg_lock);					\
	vop_msk_reg(vop_dev, WIN##id##_CTRL0, V_WIN##id##_EN((u64)en));	\
	vop_cfg_done(vop_dev);						\
	spin_unlock(&vop_dev->reg_lock);				\
	return 0;							\
}

WIN_EN(0);
WIN_EN(1);
WIN_EN(2);

/*enable/disable win directly*/
static int vop_win_direct_en(struct rk_lcdc_driver *drv,
			     int win_id, int en)
{
	struct vop_device *vop_dev =
	    container_of(drv, struct vop_device, driver);
	if (win_id == 0)
		win0_enable(vop_dev, en);
	else if (win_id == 1)
		win1_enable(vop_dev, en);
	else if (win_id == 2)
		win2_enable(vop_dev, en);
	else
		dev_err(vop_dev->dev, "invalid win number:%d\n", win_id);
	return 0;
}

#define SET_WIN_ADDR(id) \
static int set_win##id##_addr(struct vop_device *vop_dev, u32 addr) \
{							\
	spin_lock(&vop_dev->reg_lock);			\
	vop_writel(vop_dev, WIN##id##_YRGB_MST, addr);	\
	vop_msk_reg(vop_dev, WIN##id##_CTRL0, V_WIN##id##_EN(1));	\
	vop_cfg_done(vop_dev);			\
	spin_unlock(&vop_dev->reg_lock);		\
	return 0;					\
}

SET_WIN_ADDR(0);
SET_WIN_ADDR(1);
SET_WIN_ADDR(2);
int vop_direct_set_win_addr(struct rk_lcdc_driver *dev_drv,
			    int win_id, u32 addr)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	if (win_id == 0)
		set_win0_addr(vop_dev, addr);
	else if (win_id == 1)
		set_win1_addr(vop_dev, addr);
	else if (win_id == 2)
		set_win2_addr(vop_dev, addr);

	return 0;
}

static void lcdc_read_reg_defalut_cfg(struct vop_device *vop_dev)
{
	int reg = 0;
	u32 val = 0;
	struct rk_screen *screen = vop_dev->driver.cur_screen;
	u32 h_pw_bp = screen->mode.hsync_len + screen->mode.left_margin;
	u32 V_pw_bp = screen->mode.vsync_len + screen->mode.upper_margin;
	u32 st_x, st_y;
	struct rk_lcdc_win *win0 = vop_dev->driver.win[0];

	spin_lock(&vop_dev->reg_lock);
	for (reg = 0; reg < vop_dev->len; reg += 4) {
		val = vop_readl_backup(vop_dev, reg);
		switch (reg) {
		case WIN0_ACT_INFO:
			win0->area[0].xact = (val & MASK(WIN0_ACT_WIDTH)) + 1;
			win0->area[0].yact =
				((val & MASK(WIN0_ACT_HEIGHT)) >> 16) + 1;
			break;
		case WIN0_DSP_INFO:
			win0->area[0].xsize = (val & MASK(WIN0_DSP_WIDTH)) + 1;
			win0->area[0].ysize =
			    ((val & MASK(WIN0_DSP_HEIGHT)) >> 16) + 1;
			break;
		case WIN0_DSP_ST:
			st_x = val & MASK(WIN0_DSP_XST);
			st_y = (val & MASK(WIN0_DSP_YST)) >> 16;
			win0->area[0].xpos = st_x - h_pw_bp;
			win0->area[0].ypos = st_y - V_pw_bp;
			break;
		case WIN0_CTRL0:
			win0->state = val & MASK(WIN0_EN);
			win0->area[0].fmt_cfg =
					(val & MASK(WIN0_DATA_FMT)) >> 1;
			win0->fmt_10 = (val & MASK(WIN0_FMT_10)) >> 4;
			win0->area[0].format = win0->area[0].fmt_cfg;
			break;
		case WIN0_VIR:
			win0->area[0].y_vir_stride =
					val & MASK(WIN0_VIR_STRIDE);
			win0->area[0].uv_vir_stride =
			    (val & MASK(WIN0_VIR_STRIDE_UV)) >> 16;
			if (win0->area[0].format == ARGB888)
				win0->area[0].xvir = win0->area[0].y_vir_stride;
			else if (win0->area[0].format == RGB888)
				win0->area[0].xvir =
				    win0->area[0].y_vir_stride * 4 / 3;
			else if (win0->area[0].format == RGB565)
				win0->area[0].xvir =
				    2 * win0->area[0].y_vir_stride;
			else
				win0->area[0].xvir =
				    4 * win0->area[0].y_vir_stride;
			break;
		case WIN0_YRGB_MST:
			win0->area[0].smem_start = val;
			break;
		case WIN0_CBR_MST:
			win0->area[0].cbr_start = val;
			break;
		default:
			break;
		}
	}
	spin_unlock(&vop_dev->reg_lock);
}

/********do basic init*********/
static int vop_pre_init(struct rk_lcdc_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	u64 val = 0;

	if (vop_dev->pre_init)
		return 0;

	vop_dev->hclk = devm_clk_get(vop_dev->dev, "hclk_vop");
	if (IS_ERR(vop_dev->hclk)) {
		dev_err(vop_dev->dev, "failed to get vop hclk source\n");
		vop_dev->hclk = NULL;
	}
	vop_dev->aclk = devm_clk_get(vop_dev->dev, "aclk_vop");
	if (IS_ERR(vop_dev->aclk)) {
		dev_err(vop_dev->dev, "failed to get vop aclk source\n");
		vop_dev->aclk = NULL;
	}
	vop_dev->dclk = devm_clk_get(vop_dev->dev, "dclk_vop");
	if (IS_ERR(vop_dev->dclk)) {
		dev_err(vop_dev->dev, "failed to get vop dclk source\n");
		vop_dev->dclk = NULL;
	}
	vop_dev->hclk_noc = devm_clk_get(vop_dev->dev, "hclk_vop_noc");
	if (IS_ERR(vop_dev->hclk_noc)) {
		dev_err(vop_dev->dev, "failed to get vop hclk_noc source\n");
		vop_dev->hclk_noc = NULL;
	}
	vop_dev->aclk_noc = devm_clk_get(vop_dev->dev, "aclk_vop_noc");
	if (IS_ERR(vop_dev->aclk_noc)) {
		dev_err(vop_dev->dev, "failed to get vop aclk_noc source\n");
		vop_dev->aclk_noc = NULL;
	}

	if (!support_uboot_display())
		rk_disp_pwr_enable(dev_drv);
	vop_clk_enable(vop_dev);

	memcpy(vop_dev->regsbak, vop_dev->regs, vop_dev->len);
	/*backup reg config at uboot */
	lcdc_read_reg_defalut_cfg(vop_dev);

	vop_writel(vop_dev, FRC_LOWER01_0, 0x12844821);
	vop_writel(vop_dev, FRC_LOWER01_1, 0x21488412);
	vop_writel(vop_dev, FRC_LOWER10_0, 0xa55a9696);
	vop_writel(vop_dev, FRC_LOWER10_1, 0x5aa56969);
	vop_writel(vop_dev, FRC_LOWER11_0, 0xdeb77deb);
	vop_writel(vop_dev, FRC_LOWER11_1, 0xed7bb7de);

	vop_msk_reg(vop_dev, SYS_CTRL, V_AUTO_GATING_EN(0));
	vop_msk_reg(vop_dev, DSP_CTRL1, V_DITHER_UP_EN(1));

	vop_msk_reg(vop_dev, SDR2HDR_CTRL, V_WIN_CSC_MODE_SEL(1));
	val = V_SRC_MAX(12642) | V_SRC_MIN(494);
	vop_msk_reg(vop_dev, HDR2SDR_SRC_RANGE, val);
	val = V_NORMFACEETF(1327);
	vop_msk_reg(vop_dev, HDR2SDR_NORMFACEETF, val);
	val = V_SRC_MAX(4636) | V_SRC_MIN(0);
	vop_msk_reg(vop_dev, HDR2SDR_DST_RANGE, val);
	val = V_NORMFACCGAMMA(10240);
	vop_msk_reg(vop_dev, HDR2SDR_NORMFACCGAMMA, val);

	vop_cfg_done(vop_dev);
	vop_dev->pre_init = true;

	return 0;
}

static void vop_deint(struct vop_device *vop_dev)
{
	if (vop_dev->clk_on) {
		vop_disable_irq(vop_dev);
		spin_lock(&vop_dev->reg_lock);
		vop_msk_reg(vop_dev, WIN0_CTRL0, V_WIN0_EN(0));
		vop_msk_reg(vop_dev, WIN1_CTRL0, V_WIN0_EN(0));
		vop_msk_reg(vop_dev, WIN2_CTRL0, V_WIN0_EN(0));
		vop_cfg_done(vop_dev);
		spin_unlock(&vop_dev->reg_lock);
		mdelay(50);
	}
}

static int rk322xh_vop_get_win_csc(struct rk_lcdc_win *win)
{
	int win_csc = 0;

	if (IS_YUV(win->area[0].fmt_cfg)) {
		if (win->colorspace == CSC_BT2020)
			win_csc = COLOR_YCBCR_BT2020;
		else if (win->colorspace == CSC_BT709)
			win_csc = COLOR_YCBCR_BT709;
		else if (win->colorspace == CSC_BT601F)
			win_csc = COLOR_YCBCR_BT601F;
		else
			win_csc = COLOR_YCBCR;
	} else {
		win_csc = COLOR_RGB; /* default RGB 601*/
	}

	win->csc_mode = win_csc;
	return win_csc;
}

static void rk322xh_vop_win_csc_mode(struct vop_device *vop_dev,
				     struct rk_lcdc_win *win,
				     int overlay_mode,
				     int output_color)
{
	u64 val;
	u32 shift;
	int win_csc_mode = 0;
	int r2y_en = 0, y2r_en = 0;
	int win_csc = win->csc_mode;

	if (IS_YUV_COLOR(win_csc) && (overlay_mode == VOP_RGB_DOMAIN)) {
		r2y_en = 0;
		y2r_en = 1;
		/* y2r csc mode depend on data color space */
		win_csc_mode = win->colorspace;
	} else if (!IS_YUV_COLOR(win_csc) && (overlay_mode == VOP_YUV_DOMAIN)) {
		r2y_en = 1;
		y2r_en = 0;
		/* r2y csc mode depend on output color mode */
		if (output_color == COLOR_YCBCR_BT2020)
			win_csc_mode = VOP_CSC_BT2020;
		else if (output_color == COLOR_YCBCR_BT709)
			win_csc_mode = VOP_CSC_BT709L;
		else
			win_csc_mode = VOP_CSC_BT601L;
	} else {
		r2y_en = 0;
		y2r_en = 0;
	}

	shift = win->id << 2;
	val = (V_WIN0_R2Y_EN(r2y_en) | V_WIN0_Y2R_EN(y2r_en)) << shift;
	vop_msk_reg(vop_dev, SDR2HDR_CTRL, val);
	if (r2y_en | y2r_en) {
		shift = win->id * 0x100;
		val = V_WIN0_CSC_MODE(win_csc_mode);
		vop_msk_reg(vop_dev, WIN0_CTRL0 + shift, val);
	}
}

/*
 * color space & sdr/hdr path
 * win0 support hdr <-> sdr
 * win1/win2 support sdr -> hdr
 *
 * win0   hdr2sdr: post convert mode = 1
 *        sdr2hdr: post convert mode = 0
 * win1/2 sdr2hdr: pre convert mode  = 0
 */
static void rk322xh_vop_hdr_csc_cfg(struct rk_lcdc_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rk_lcdc_win *win = NULL;
	int i = 0, overlay_mode;
	int post_hdr2sdr_en = 0, pre_sdr2hdr_en = 0, post_sdr2hdr_en = 0;
	int output_data_space;
	u64 val = 0;
	u32 win_state = 0;
	int output_color = dev_drv->output_color;

	output_data_space = dev_drv->cur_screen->data_space;

	vop_dev->pre_sdr2hdr = 0;
	vop_dev->post_sdr2hdr = 0;
	vop_dev->post_hdr2sdr = 0;
	dev_drv->pre_overlay = 1;

	for (i = 0; i < dev_drv->lcdc_win_num; i++) {
		win = dev_drv->win[i];
		win_state |= win->state << win->id;
		if (!win->state)
			continue;
		if (win->id == 0) { /* win0 should be hdr mode */
			if (output_data_space && win->area[0].data_space) {
				post_hdr2sdr_en = 0;
				post_sdr2hdr_en = 0;
			} else if (!output_data_space &&
				   win->area[0].data_space) {
				post_hdr2sdr_en = 1;
				post_sdr2hdr_en = 0;
			} else {
				pr_warn("sdr input and hdr output!\n");
				post_hdr2sdr_en = 0;
				post_sdr2hdr_en = 1;
			}
			vop_load_hdr2sdr_table(vop_dev);
			val = V_HDR2SDR_EN(post_hdr2sdr_en);
			vop_msk_reg(vop_dev, HDR2SDR_CTRL, val);
		} else {
			/* hdr2sdr or sdr2hdr */
			if (output_data_space) {/* hdr output */
				if (win->area[0].data_space)
					pre_sdr2hdr_en &= ~(1 << win->id);
				else
					pre_sdr2hdr_en |= 1 << win->id;
			} else { /* sdr output */
				if (win->area[0].data_space)
					pr_err("pre sdr2hdr mode not support hdr2sdr\n");
				else
					pre_sdr2hdr_en &= ~(1 << win->id);
			}
		}
	}
	win_state &= 0xfe;
	/* sdr to hdr */
	if ((pre_sdr2hdr_en == win_state) && (pre_sdr2hdr_en != 0)) {
		/* enable pre sdr2hdr */
		vop_dev->pre_sdr2hdr = 1;
		val = V_BT1886EOTF_PRE_CONV_EN(pre_sdr2hdr_en) |
			V_RGB2RGB_PRE_CONV_EN(pre_sdr2hdr_en) |
			V_RGB2RGB_PRE_CONV_MODE(0) |
			V_ST2084OETF_PRE_CONV_EN(pre_sdr2hdr_en);
		vop_load_sdr2hdr_table(vop_dev, SDR2HDR_FOR_HDR);
	} else if (pre_sdr2hdr_en == 0) {
		/* disable pre sdr2hdr */
		val = V_BT1886EOTF_PRE_CONV_EN(pre_sdr2hdr_en) |
			V_RGB2RGB_PRE_CONV_EN(pre_sdr2hdr_en) |
			V_RGB2RGB_PRE_CONV_MODE(0) |
			V_ST2084OETF_PRE_CONV_EN(pre_sdr2hdr_en);
	} else {
		pr_err("unsupport: sdr2hdr_en:0x%x, win_state: 0x%x\n",
		       pre_sdr2hdr_en, win_state);
	}
	val |= V_BT1886EOTF_POST_CONV_EN(post_sdr2hdr_en) |
		V_RGB2RGB_POST_CONV_EN(post_sdr2hdr_en) |
		V_RGB2RGB_POST_CONV_MODE(0) |
		V_ST2084OETF_POST_CONV_EN(post_sdr2hdr_en);
	vop_msk_reg(vop_dev, SDR2HDR_CTRL, val);
	if (pre_sdr2hdr_en || post_sdr2hdr_en)
		vop_load_sdr2hdr_table(vop_dev, SDR2HDR_FOR_HDR);
	vop_dev->post_sdr2hdr = post_sdr2hdr_en;
	vop_dev->post_hdr2sdr = post_hdr2sdr_en;
	if (post_hdr2sdr_en || pre_sdr2hdr_en || post_sdr2hdr_en) {
		/* hdr must in rgb domain transform */
		overlay_mode = VOP_RGB_DOMAIN;
	} else {
		if (output_color == COLOR_RGB)
			overlay_mode = VOP_RGB_DOMAIN;
		else
			overlay_mode = VOP_YUV_DOMAIN;
	}
	dev_drv->overlay_mode = overlay_mode;
	for (i = 0; i < dev_drv->lcdc_win_num; i++) {
		win = dev_drv->win[i];
		if (!win->state)
			continue;
		rk322xh_vop_get_win_csc(win);
		rk322xh_vop_win_csc_mode(vop_dev, win,
					 overlay_mode, output_color);
		/* if win0 enable hdr2sdr, close win y2r */
		if ((win->id == 0) && (post_hdr2sdr_en))
			vop_msk_reg(vop_dev, SDR2HDR_CTRL, V_WIN0_Y2R_EN(0));
		/* if hdr2sdr enable, close win0 y2r */
		if ((win->id == 0) && (post_hdr2sdr_en))
			vop_msk_reg(vop_dev, WIN0_CTRL0, V_WIN0_Y2R_EN(0));
	}
}

/*
 * win0   bt2020->bt709: post convert mode = 1
 *        bt709->bt2020: post convert mode = 0
 * win1/2 bt2020->bt709: pre convert mode  = 1
 *        bt709->bt2020: pre convert mode  = 0
 */
static void rk322xh_vop_sdr_csc_cfg(struct rk_lcdc_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rk_lcdc_win *win;
	int i = 0, win_csc = 0;
	int post_conv = 0, post_conv_mode = 0, pre_conv = 0, pre_conv_mode = 0;
	int overlay_mode, force_rgb_overlay = 0;
	u64 val = 0;
	u32 win_state = 0;
	int output_color = dev_drv->output_color;

	/* revert hdr2sdr config */
	val = V_HDR2SDR_EN(0);
	vop_msk_reg(vop_dev, HDR2SDR_CTRL, val);
	if (dev_drv->cur_screen->data_space)
		pr_warn("win data space is sdr, but output data space is %d!\n",
			dev_drv->cur_screen->data_space);
	/* get win csc and overlay mode, sdr2hdr mode must be rgb domain */
	for (i = 0; i < dev_drv->lcdc_win_num; i++) {
		win = dev_drv->win[i];
		win_state |= win->state << win->id;
		if (!win->state)
			continue;
		win_csc = rk322xh_vop_get_win_csc(win);

		if (((win_csc == COLOR_RGB_BT2020) ||
		     (win_csc == COLOR_YCBCR_BT2020)) &&
		    (output_color != COLOR_YCBCR_BT2020 &&
		     output_color != COLOR_RGB_BT2020)) {
			force_rgb_overlay = 1;
			vop_load_sdr2hdr_table(vop_dev, SDR2HDR_FOR_BT2020);
			/* enable bt2020->bt709 */
			if (win->id == 0) {
				post_conv = 1;
				post_conv_mode = 1;
			} else {
				pre_conv = 1;
				pre_conv_mode |= 1 << win->id;
			}
		} else if (((win_csc != COLOR_RGB_BT2020) &&
		     (win_csc != COLOR_YCBCR_BT2020)) &&
		    (output_color == COLOR_YCBCR_BT2020 ||
		     output_color == COLOR_RGB_BT2020)) {
			force_rgb_overlay = 1;
			vop_load_sdr2hdr_table(vop_dev, SDR2HDR_FOR_BT2020);
			/* enable bt709->bt2020 */
			if (win->id == 0) {
				post_conv = 1;
				post_conv_mode = 0;
			} else {
				pre_conv = 1;
				pre_conv_mode &= ~(1 << win->id);
			}
		} else {
			force_rgb_overlay = 0;
			if (win->id == 0) {
				post_conv = 0;
				post_conv_mode = 0;
			} else {
				pre_conv_mode &= ~(1 << win->id);
			}
		}
	}

	if (force_rgb_overlay) {
		overlay_mode = VOP_RGB_DOMAIN;
	} else {
		if (output_color == COLOR_RGB)
			overlay_mode = VOP_RGB_DOMAIN;
		else
			overlay_mode = VOP_YUV_DOMAIN;
	}

	dev_drv->overlay_mode = overlay_mode;
	for (i = 0; i < dev_drv->lcdc_win_num; i++) {
		win = dev_drv->win[i];
		if (!win->state)
			continue;
		rk322xh_vop_win_csc_mode(vop_dev, win,
					 overlay_mode, output_color);
	}

	/* pre conv for win1,2,3,hwc, mode:0: bt709->bt2020, 1: bt2020->bt709 */
	win_state &= 0xfe;
	if (pre_conv == 1) {
		dev_drv->pre_overlay = 1;
		if (pre_conv_mode == win_state) {
			val = V_BT1886EOTF_PRE_CONV_EN(1) |
				V_RGB2RGB_PRE_CONV_EN(1) |
				V_RGB2RGB_PRE_CONV_MODE(!!pre_conv_mode) |
				V_ST2084OETF_PRE_CONV_EN(1);
		} else if (pre_conv_mode == 0) {
			val = V_BT1886EOTF_PRE_CONV_EN(1) |
				V_RGB2RGB_PRE_CONV_EN(1) |
				V_RGB2RGB_PRE_CONV_MODE(pre_conv_mode) |
				V_ST2084OETF_PRE_CONV_EN(1);
		} else {
			pr_err("unsupport, pre_conv: 0x%x, win_state: 0x%x\n",
			       pre_conv, win_state);
		}
	} else {
		val = V_BT1886EOTF_PRE_CONV_EN(0) |
			V_RGB2RGB_PRE_CONV_EN(0) |
			V_RGB2RGB_PRE_CONV_MODE(0) |
			V_ST2084OETF_PRE_CONV_EN(0);
	}

	/* post conv for win0: mode:0: bt709->bt2020, 1: bt2020->bt709 */
	val |= V_BT1886EOTF_POST_CONV_EN(post_conv) |
		V_RGB2RGB_POST_CONV_EN(post_conv) |
		V_RGB2RGB_POST_CONV_MODE(post_conv_mode) |
		V_ST2084OETF_POST_CONV_EN(post_conv);
	vop_msk_reg(vop_dev, SDR2HDR_CTRL, val);
}

static void rk322xh_vop_bcsh_path_sel(struct rk_lcdc_driver *dev_drv);
static void rk322xh_vop_csc_cfg(struct rk_lcdc_driver *dev_drv)
{
	dev_drv->pre_overlay = 0;
	if ((dev_drv->win[0]->area[0].data_space ||
	     dev_drv->cur_screen->data_space) &&
	    dev_drv->win[0]->area[0].state)
		rk322xh_vop_hdr_csc_cfg(dev_drv);
	else
		rk322xh_vop_sdr_csc_cfg(dev_drv);

	rk322xh_vop_bcsh_path_sel(dev_drv);
}

static int vop_post_cfg(struct rk_lcdc_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rk_screen *screen = dev_drv->cur_screen;
	u16 x_res = screen->mode.xres;
	u16 y_res = screen->mode.yres;
	u64 val;
	u16 h_total, v_total;
	u16 post_hsd_en, post_vsd_en;
	u16 post_dsp_hact_st, post_dsp_hact_end;
	u16 post_dsp_vact_st, post_dsp_vact_end;
	u16 post_dsp_vact_st_f1, post_dsp_vact_end_f1;
	u16 post_h_fac, post_v_fac;

	screen->post_dsp_stx = x_res * (100 - dev_drv->overscan.left) / 200;
	screen->post_dsp_sty = y_res * (100 - dev_drv->overscan.top) / 200;
	screen->post_xsize = x_res *
	    (dev_drv->overscan.left + dev_drv->overscan.right) / 200;
	screen->post_ysize = y_res *
	    (dev_drv->overscan.top + dev_drv->overscan.bottom) / 200;

	h_total = screen->mode.hsync_len + screen->mode.left_margin +
	    x_res + screen->mode.right_margin;
	v_total = screen->mode.vsync_len + screen->mode.upper_margin +
	    y_res + screen->mode.lower_margin;

	if (screen->post_dsp_stx + screen->post_xsize > x_res) {
		dev_warn(vop_dev->dev, "post:stx[%d]+xsize[%d]>x_res[%d]\n",
			 screen->post_dsp_stx, screen->post_xsize, x_res);
		screen->post_dsp_stx = x_res - screen->post_xsize;
	}
	if (screen->x_mirror == 0) {
		post_dsp_hact_st = screen->post_dsp_stx +
		    screen->mode.hsync_len + screen->mode.left_margin;
		post_dsp_hact_end = post_dsp_hact_st + screen->post_xsize;
	} else {
		post_dsp_hact_end = h_total - screen->mode.right_margin -
		    screen->post_dsp_stx;
		post_dsp_hact_st = post_dsp_hact_end - screen->post_xsize;
	}
	if ((screen->post_xsize < x_res) && (screen->post_xsize != 0)) {
		post_hsd_en = 1;
		post_h_fac =
		    GET_SCALE_FACTOR_BILI_DN(x_res, screen->post_xsize);
	} else {
		post_hsd_en = 0;
		post_h_fac = 0x1000;
	}

	if (screen->post_dsp_sty + screen->post_ysize > y_res) {
		dev_warn(vop_dev->dev, "post:sty[%d]+ysize[%d]> y_res[%d]\n",
			 screen->post_dsp_sty, screen->post_ysize, y_res);
		screen->post_dsp_sty = y_res - screen->post_ysize;
	}

	if ((screen->post_ysize < y_res) && (screen->post_ysize != 0)) {
		post_vsd_en = 1;
		post_v_fac = GET_SCALE_FACTOR_BILI_DN(y_res,
						      screen->post_ysize);
	} else {
		post_vsd_en = 0;
		post_v_fac = 0x1000;
	}

	if (screen->mode.vmode & FB_VMODE_INTERLACED) {
		post_dsp_vact_st = screen->post_dsp_sty / 2 +
					screen->mode.vsync_len +
					screen->mode.upper_margin;
		post_dsp_vact_end = post_dsp_vact_st +
					screen->post_ysize / 2;

		post_dsp_vact_st_f1 = screen->mode.vsync_len +
					screen->mode.upper_margin +
					y_res / 2 +
					screen->mode.lower_margin +
					screen->mode.vsync_len +
					screen->mode.upper_margin +
					screen->post_dsp_sty / 2 +
					1;
		post_dsp_vact_end_f1 = post_dsp_vact_st_f1 +
					screen->post_ysize / 2;
	} else {
		if (screen->y_mirror == 0) {
			post_dsp_vact_st = screen->post_dsp_sty +
			    screen->mode.vsync_len +
			    screen->mode.upper_margin;
			post_dsp_vact_end = post_dsp_vact_st +
				screen->post_ysize;
		} else {
			post_dsp_vact_end = v_total -
				screen->mode.lower_margin -
			    screen->post_dsp_sty;
			post_dsp_vact_st = post_dsp_vact_end -
				screen->post_ysize;
		}
		post_dsp_vact_st_f1 = 0;
		post_dsp_vact_end_f1 = 0;
	}
	DBG(1, "post:xsize=%d,ysize=%d,xpos=%d",
	    screen->post_xsize, screen->post_ysize, screen->xpos);
	DBG(1, ",ypos=%d,hsd_en=%d,h_fac=%d,vsd_en=%d,v_fac=%d\n",
	    screen->ypos, post_hsd_en, post_h_fac, post_vsd_en, post_v_fac);
	val = V_DSP_HACT_END_POST(post_dsp_hact_end) |
	    V_DSP_HACT_ST_POST(post_dsp_hact_st);
	vop_msk_reg(vop_dev, POST_DSP_HACT_INFO, val);

	val = V_DSP_VACT_END_POST(post_dsp_vact_end) |
	    V_DSP_VACT_ST_POST(post_dsp_vact_st);
	vop_msk_reg(vop_dev, POST_DSP_VACT_INFO, val);

	val = V_POST_HS_FACTOR_YRGB(post_h_fac) |
	    V_POST_VS_FACTOR_YRGB(post_v_fac);
	vop_msk_reg(vop_dev, POST_SCL_FACTOR_YRGB, val);
	val = V_DSP_VACT_END_POST(post_dsp_vact_end_f1) |
	    V_DSP_VACT_ST_POST(post_dsp_vact_st_f1);
	vop_msk_reg(vop_dev, POST_DSP_VACT_INFO_F1, val);
	val = V_POST_HOR_SD_EN(post_hsd_en) | V_POST_VER_SD_EN(post_vsd_en);
	vop_msk_reg(vop_dev, POST_SCL_CTRL, val);

	return 0;
}

static int vop_clr_key_cfg(struct rk_lcdc_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rk_lcdc_win *win;
	u32 colorkey_r, colorkey_g, colorkey_b;
	int i, key_val;

	for (i = 0; i < dev_drv->lcdc_win_num; i++) {
		win = dev_drv->win[i];
		key_val = win->color_key_val;
		colorkey_r = (key_val & 0xff) << 2;
		colorkey_g = ((key_val >> 8) & 0xff) << 12;
		colorkey_b = ((key_val >> 16) & 0xff) << 22;
		/* color key dither 565/888->aaa */
		key_val = colorkey_r | colorkey_g | colorkey_b;
		switch (i) {
		case 0:
			vop_writel(vop_dev, WIN0_COLOR_KEY, key_val);
			break;
		case 1:
			vop_writel(vop_dev, WIN1_COLOR_KEY, key_val);
			break;
		case 2:
			vop_writel(vop_dev, WIN2_COLOR_KEY, key_val);
			break;
		default:
			pr_info("%s:un support win num:%d\n",
				__func__, i);
			break;
		}
	}
	return 0;
}

static int vop_alpha_cfg(struct rk_lcdc_driver *dev_drv, int win_id)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rk_lcdc_win *win = dev_drv->win[win_id];
	struct alpha_config alpha_config;
	u64 val;
	int ppixel_alpha = 0, global_alpha = 0, i;
	u32 src_alpha_ctl = 0, dst_alpha_ctl = 0;
	int alpha_en = 1;

	memset(&alpha_config, 0, sizeof(struct alpha_config));
	for (i = 0; i < win->area_num; i++) {
		ppixel_alpha |= ((win->area[i].format == ARGB888) ||
				 (win->area[i].format == FBDC_ARGB_888) ||
				 (win->area[i].format == FBDC_ABGR_888) ||
				 (win->area[i].format == ABGR888)) ? 1 : 0;
	}

	global_alpha = (win->g_alpha_val == 0) ? 0 : 1;

	for (i = 0; i < dev_drv->lcdc_win_num; i++) {
		if (!dev_drv->win[i]->state)
			continue;
		if (win->z_order > dev_drv->win[i]->z_order)
			break;
	}

	/*
	 * The bottom layer not support ppixel_alpha mode.
	 */
	if (i == dev_drv->lcdc_win_num)
		ppixel_alpha = 0;
	alpha_config.src_global_alpha_val = win->g_alpha_val;
	win->alpha_mode = AB_SRC_OVER;

	switch (win->alpha_mode) {
	case AB_USER_DEFINE:
		break;
	case AB_CLEAR:
		alpha_config.src_factor_mode = AA_ZERO;
		alpha_config.dst_factor_mode = AA_ZERO;
		break;
	case AB_SRC:
		alpha_config.src_factor_mode = AA_ONE;
		alpha_config.dst_factor_mode = AA_ZERO;
		break;
	case AB_DST:
		alpha_config.src_factor_mode = AA_ZERO;
		alpha_config.dst_factor_mode = AA_ONE;
		break;
	case AB_SRC_OVER:
		alpha_config.src_color_mode = AA_SRC_PRE_MUL;
		if (global_alpha)
			alpha_config.src_factor_mode = AA_SRC_GLOBAL;
		else
			alpha_config.src_factor_mode = AA_ONE;
		alpha_config.dst_factor_mode = AA_SRC_INVERSE;
		break;
	case AB_DST_OVER:
		alpha_config.src_color_mode = AA_SRC_PRE_MUL;
		alpha_config.src_factor_mode = AA_SRC_INVERSE;
		alpha_config.dst_factor_mode = AA_ONE;
		break;
	case AB_SRC_IN:
		alpha_config.src_color_mode = AA_SRC_PRE_MUL;
		alpha_config.src_factor_mode = AA_SRC;
		alpha_config.dst_factor_mode = AA_ZERO;
		break;
	case AB_DST_IN:
		alpha_config.src_factor_mode = AA_ZERO;
		alpha_config.dst_factor_mode = AA_SRC;
		break;
	case AB_SRC_OUT:
		alpha_config.src_color_mode = AA_SRC_PRE_MUL;
		alpha_config.src_factor_mode = AA_SRC_INVERSE;
		alpha_config.dst_factor_mode = AA_ZERO;
		break;
	case AB_DST_OUT:
		alpha_config.src_factor_mode = AA_ZERO;
		alpha_config.dst_factor_mode = AA_SRC_INVERSE;
		break;
	case AB_SRC_ATOP:
		alpha_config.src_color_mode = AA_SRC_PRE_MUL;
		alpha_config.src_factor_mode = AA_SRC;
		alpha_config.dst_factor_mode = AA_SRC_INVERSE;
		break;
	case AB_DST_ATOP:
		alpha_config.src_color_mode = AA_SRC_PRE_MUL;
		alpha_config.src_factor_mode = AA_SRC_INVERSE;
		alpha_config.dst_factor_mode = AA_SRC;
		break;
	case XOR:
		alpha_config.src_color_mode = AA_SRC_PRE_MUL;
		alpha_config.src_factor_mode = AA_SRC_INVERSE;
		alpha_config.dst_factor_mode = AA_SRC_INVERSE;
		break;
	case AB_SRC_OVER_GLOBAL:
		alpha_config.src_global_alpha_mode = AA_PER_PIX_GLOBAL;
		alpha_config.src_color_mode = AA_SRC_NO_PRE_MUL;
		alpha_config.src_factor_mode = AA_SRC_GLOBAL;
		alpha_config.dst_factor_mode = AA_SRC_INVERSE;
		break;
	default:
		pr_err("alpha mode error\n");
		break;
	}
	if ((ppixel_alpha == 1) && (global_alpha == 1))
		alpha_config.src_global_alpha_mode = AA_PER_PIX_GLOBAL;
	else if (ppixel_alpha == 1)
		alpha_config.src_global_alpha_mode = AA_PER_PIX;
	else if (global_alpha == 1)
		alpha_config.src_global_alpha_mode = AA_GLOBAL;
	else
		alpha_en = 0;
	alpha_config.src_alpha_mode = AA_STRAIGHT;
	alpha_config.src_alpha_cal_m0 = AA_NO_SAT;

	switch (win_id) {
	case 0:
		src_alpha_ctl = WIN0_SRC_ALPHA_CTRL;
		dst_alpha_ctl = WIN0_DST_ALPHA_CTRL;
		break;
	case 1:
		src_alpha_ctl = WIN1_SRC_ALPHA_CTRL;
		dst_alpha_ctl = WIN1_DST_ALPHA_CTRL;
		break;
	case 2:
		src_alpha_ctl = WIN2_SRC_ALPHA_CTRL;
		dst_alpha_ctl = WIN2_DST_ALPHA_CTRL;
		break;
	case 3:
		src_alpha_ctl = HWC_SRC_ALPHA_CTRL;
		dst_alpha_ctl = HWC_DST_ALPHA_CTRL;
		break;
	}
	val = V_WIN0_DST_FACTOR_MODE(alpha_config.dst_factor_mode);
	vop_msk_reg(vop_dev, dst_alpha_ctl, val);
	val = V_WIN0_SRC_ALPHA_EN(alpha_en) |
	    V_WIN0_SRC_COLOR_MODE(alpha_config.src_color_mode) |
	    V_WIN0_SRC_ALPHA_MODE(alpha_config.src_alpha_mode) |
	    V_WIN0_SRC_BLEND_MODE(alpha_config.src_global_alpha_mode) |
	    V_WIN0_SRC_ALPHA_CAL_MODE(alpha_config.src_alpha_cal_m0) |
	    V_WIN0_SRC_FACTOR_MODE(alpha_config.src_factor_mode) |
	    V_WIN0_SRC_GLOBAL_ALPHA(alpha_config.src_global_alpha_val);

	vop_msk_reg(vop_dev, src_alpha_ctl, val);

	return 0;
}

static int vop_axi_gather_cfg(struct vop_device *vop_dev,
			      struct rk_lcdc_win *win)
{
	u64 val;
	u16 yrgb_gather_num = 3;
	u16 cbcr_gather_num = 1;

	switch (win->area[0].format) {
	case ARGB888:
	case XBGR888:
	case ABGR888:
		yrgb_gather_num = 3;
		break;
	case RGB888:
	case RGB565:
		yrgb_gather_num = 2;
		break;
	case YUV444:
	case YUV422:
	case YUV420:
	case YUV420_A:
	case YUV422_A:
	case YUV444_A:
	case YUV420_NV21:
		yrgb_gather_num = 1;
		cbcr_gather_num = 2;
		break;
	default:
		dev_err(vop_dev->driver.dev, "%s:un supported format[%d]\n",
			__func__, win->area[0].format);
		return -EINVAL;
	}

	if ((win->id == 0) || (win->id == 1) || (win->id == 2)) {
		val = V_WIN0_YRGB_AXI_GATHER_EN(1) |
			V_WIN0_CBR_AXI_GATHER_EN(1) |
			V_WIN0_YRGB_AXI_GATHER_NUM(yrgb_gather_num) |
			V_WIN0_CBR_AXI_GATHER_NUM(cbcr_gather_num);
		vop_msk_reg(vop_dev, WIN0_CTRL1 + (win->id * 0x100), val);
	} else if (win->id == 3) {
		val = V_HWC_AXI_GATHER_EN(1) |
			V_HWC_AXI_GATHER_NUM(yrgb_gather_num);
		vop_msk_reg(vop_dev, HWC_CTRL1, val);
	}
	return 0;
}

static int vop_win_full_reg_update(struct rk_lcdc_driver *dev_drv, int win_id)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rk_lcdc_win *win = dev_drv->win[win_id];
	u64 val;
	u32 off;
	int format;

	off = win_id * 0x100;

	if (win->state == 1) {
		vop_axi_gather_cfg(vop_dev, win);

		/*
		 * rk322x have a bug on windows 0 and 1:
		 *
		 * When switch win format from RGB to YUV, would flash
		 * some green lines on the top of the windows.
		 *
		 * Use bg_en show one blank frame to skip the error frame.
		 */
		if (IS_YUV(win->area[0].fmt_cfg)) {
			val = vop_readl(vop_dev, WIN0_CTRL0);
			format = (val & MASK(WIN0_DATA_FMT)) >> 1;

			if (!IS_YUV(format)) {
				if (dev_drv->overlay_mode == VOP_YUV_DOMAIN) {
					val = V_WIN0_DSP_BG_RED(0x200) |
						V_WIN0_DSP_BG_GREEN(0x40) |
						V_WIN0_DSP_BG_BLUE(0x200) |
						V_WIN0_BG_EN(1);
					vop_msk_reg(vop_dev, WIN0_DSP_BG + off,
						    val);
				} else {
					val = V_WIN0_DSP_BG_RED(0) |
						V_WIN0_DSP_BG_GREEN(0) |
						V_WIN0_DSP_BG_BLUE(0) |
						V_WIN0_BG_EN(1);
					vop_msk_reg(vop_dev, WIN0_DSP_BG + off,
						    val);
				}
			} else {
				val = V_WIN0_BG_EN(0);
				vop_msk_reg(vop_dev, WIN0_DSP_BG + off, val);
			}
		} else {
			val = V_WIN0_BG_EN(0);
			vop_msk_reg(vop_dev, WIN0_DSP_BG + off, val);
		}

		val = V_WIN0_EN(win->state) |
			V_WIN0_DATA_FMT(win->area[0].fmt_cfg) |
			V_WIN0_FMT_10(win->fmt_10) |
			V_WIN0_LB_MODE(win->win_lb_mode) |
			V_WIN0_RB_SWAP(win->area[0].swap_rb) |
			V_WIN0_X_MIR_EN(win->xmirror) |
			V_WIN0_Y_MIR_EN(win->ymirror) |
			V_WIN0_UV_SWAP(win->area[0].swap_uv);
		vop_msk_reg(vop_dev, WIN0_CTRL0 + off, val);
		val = V_WIN0_BIC_COE_SEL(win->bic_coe_el) |
		    V_WIN0_VSD_YRGB_GT4(win->vsd_yrgb_gt4) |
		    V_WIN0_VSD_YRGB_GT2(win->vsd_yrgb_gt2) |
		    V_WIN0_VSD_CBR_GT4(win->vsd_cbr_gt4) |
		    V_WIN0_VSD_CBR_GT2(win->vsd_cbr_gt2) |
		    V_WIN0_YRGB_HOR_SCL_MODE(win->yrgb_hor_scl_mode) |
		    V_WIN0_YRGB_VER_SCL_MODE(win->yrgb_ver_scl_mode) |
		    V_WIN0_YRGB_HSD_MODE(win->yrgb_hsd_mode) |
		    V_WIN0_YRGB_VSU_MODE(win->yrgb_vsu_mode) |
		    V_WIN0_YRGB_VSD_MODE(win->yrgb_vsd_mode) |
		    V_WIN0_CBR_HOR_SCL_MODE(win->cbr_hor_scl_mode) |
		    V_WIN0_CBR_VER_SCL_MODE(win->cbr_ver_scl_mode) |
		    V_WIN0_CBR_HSD_MODE(win->cbr_hsd_mode) |
		    V_WIN0_CBR_VSU_MODE(win->cbr_vsu_mode) |
		    V_WIN0_CBR_VSD_MODE(win->cbr_vsd_mode);
		vop_msk_reg(vop_dev, WIN0_CTRL1 + off, val);
		val = V_WIN0_VIR_STRIDE(win->area[0].y_vir_stride) |
		    V_WIN0_VIR_STRIDE_UV(win->area[0].uv_vir_stride);
		vop_writel(vop_dev, WIN0_VIR + off, val);
		val = V_WIN0_ACT_WIDTH(win->area[0].xact - 1) |
		    V_WIN0_ACT_HEIGHT(win->area[0].yact - 1);
		vop_writel(vop_dev, WIN0_ACT_INFO + off, val);

		val = V_WIN0_DSP_WIDTH(win->area[0].xsize - 1) |
		    V_WIN0_DSP_HEIGHT(win->area[0].ysize - 1);
		vop_writel(vop_dev, WIN0_DSP_INFO + off, val);

		val = V_WIN0_DSP_XST(win->area[0].dsp_stx) |
		    V_WIN0_DSP_YST(win->area[0].dsp_sty);
		vop_writel(vop_dev, WIN0_DSP_ST + off, val);

		val = V_WIN0_HS_FACTOR_YRGB(win->scale_yrgb_x) |
		    V_WIN0_VS_FACTOR_YRGB(win->scale_yrgb_y);
		vop_writel(vop_dev, WIN0_SCL_FACTOR_YRGB + off, val);

		val = V_WIN0_HS_FACTOR_CBR(win->scale_cbcr_x) |
		    V_WIN0_VS_FACTOR_CBR(win->scale_cbcr_y);
		vop_writel(vop_dev, WIN0_SCL_FACTOR_CBR + off, val);
	} else {
		val = V_WIN0_EN(win->state);
		vop_msk_reg(vop_dev, WIN0_CTRL0 + off, val);
	}

	return 0;
}

static int vop_hwc_reg_update(struct rk_lcdc_driver *dev_drv, int win_id)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rk_lcdc_win *win = dev_drv->win[win_id];
	unsigned int hwc_size = 0;
	u64 val;

	if ((win->area[0].xsize == 32) && (win->area[0].ysize == 32)) {
		hwc_size = 0;
	} else if ((win->area[0].xsize == 64) && (win->area[0].ysize == 64)) {
		hwc_size = 1;
	} else if ((win->area[0].xsize == 96) && (win->area[0].ysize == 96)) {
		hwc_size = 2;
	} else if ((win->area[0].xsize == 128) &&
		   (win->area[0].ysize == 128)) {
		hwc_size = 3;
	} else {
		dev_err(vop_dev->dev, "un supported hwc size[%dx%d]!\n",
			win->area[0].xsize, win->area[0].ysize);
		return -EINVAL;
	}

	if (win->state == 1) {
		vop_axi_gather_cfg(vop_dev, win);
		val = V_HWC_EN(1) | V_HWC_DATA_FMT(win->area[0].fmt_cfg) |
		    V_HWC_RB_SWAP(win->area[0].swap_rb);
		vop_msk_reg(vop_dev, HWC_CTRL0, val);

		val = V_HWC_SIZE(hwc_size);
		vop_msk_reg(vop_dev, HWC_CTRL0, val);

		val = V_HWC_DSP_XST(win->area[0].dsp_stx) |
		    V_HWC_DSP_YST(win->area[0].dsp_sty);
		vop_msk_reg(vop_dev, HWC_DSP_ST, val);
	} else {
		val = V_HWC_EN(win->state);
		vop_msk_reg(vop_dev, HWC_CTRL0, val);
	}

	return 0;
}

static int vop_layer_update_regs(struct vop_device *vop_dev,
				 struct rk_lcdc_win *win)
{
	struct rk_lcdc_driver *dev_drv = &vop_dev->driver;

	if (likely(vop_dev->clk_on)) {
		vop_msk_reg(vop_dev, SYS_CTRL,
			    V_VOP_STANDBY_EN(vop_dev->standby));
		switch (win->id) {
		case 0:
		case 1:
		case 2:
			vop_win_full_reg_update(dev_drv, win->id);
			break;
		case 3:
			vop_hwc_reg_update(dev_drv, win->id);
			break;
		default:
			pr_err("invalid win number:%d!\n", win->id);
			break;
		}
		vop_cfg_done(vop_dev);
	}

	DBG(2, "%s for lcdc%d\n", __func__, vop_dev->id);
	return 0;
}

static int vop_mmu_en(struct rk_lcdc_driver *dev_drv)
{
	u64 val;
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);

	if (unlikely(!vop_dev->clk_on)) {
		pr_info("%s,clk_on = %d\n", __func__, vop_dev->clk_on);
		return 0;
	}
	if (dev_drv->iommu_enabled) {
		if (!vop_dev->iommu_status && dev_drv->mmu_dev) {
			if (likely(vop_dev->clk_on)) {
				val = V_VOP_MMU_EN(1);
				vop_msk_reg(vop_dev, SYS_CTRL, val);
				val = V_AXI_OUTSTANDING_MAX_NUM(31) |
					V_AXI_MAX_OUTSTANDING_EN(1);
				vop_msk_reg(vop_dev, SYS_CTRL1, val);
			}
			vop_dev->iommu_status = 1;
			rockchip_iovmm_activate(dev_drv->dev);
		}
	}
	return 0;
}

static int vop_set_dclk(struct rk_lcdc_driver *dev_drv, int reset_rate)
{
	int ret = 0, fps = 0;
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rk_screen *screen = dev_drv->cur_screen;
#ifdef CONFIG_RK_FPGA
	return 0;
#endif
	if (reset_rate)
		ret = clk_set_rate(vop_dev->dclk, screen->mode.pixclock);
	if (ret)
		dev_err(dev_drv->dev, "set lcdc%d dclk[%d] failed\n",
			vop_dev->id, screen->mode.pixclock);
	vop_dev->pixclock =
	    div_u64(1000000000000llu, clk_get_rate(vop_dev->dclk));
	vop_dev->driver.pixclock = vop_dev->pixclock;

	fps = rk_fb_calc_fps(screen, vop_dev->pixclock);
	screen->ft = 1000 / fps;
	dev_info(vop_dev->dev, "%s: dclk:%lu>>fps:%d ",
		 vop_dev->driver.name, clk_get_rate(vop_dev->dclk), fps);
	return 0;
}

static int vop_config_timing(struct rk_lcdc_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rk_screen *screen = dev_drv->cur_screen;
	u16 hsync_len = screen->mode.hsync_len;
	u16 left_margin = screen->mode.left_margin;
	u16 right_margin = screen->mode.right_margin;
	u16 vsync_len = screen->mode.vsync_len;
	u16 upper_margin = screen->mode.upper_margin;
	u16 lower_margin = screen->mode.lower_margin;
	u16 x_res = screen->mode.xres;
	u16 y_res = screen->mode.yres;
	u64 val;
	u16 h_total, v_total;
	u16 vact_end_f1, vact_st_f1, vs_end_f1, vs_st_f1;

	h_total = hsync_len + left_margin + x_res + right_margin;
	v_total = vsync_len + upper_margin + y_res + lower_margin;

	val = V_DSP_HS_END(hsync_len) | V_DSP_HTOTAL(h_total);
	vop_msk_reg(vop_dev, DSP_HTOTAL_HS_END, val);

	val = V_DSP_HACT_END(hsync_len + left_margin + x_res) |
	    V_DSP_HACT_ST(hsync_len + left_margin);
	vop_msk_reg(vop_dev, DSP_HACT_ST_END, val);

	if (screen->mode.vmode & FB_VMODE_INTERLACED) {
		/* First Field Timing */
		val = V_DSP_VS_END(vsync_len) |
		    V_DSP_VTOTAL(2 * (vsync_len + upper_margin +
				      lower_margin) + y_res + 1);
		vop_msk_reg(vop_dev, DSP_VTOTAL_VS_END, val);

		val = V_DSP_VACT_END(vsync_len + upper_margin + y_res / 2) |
		    V_DSP_VACT_ST(vsync_len + upper_margin);
		vop_msk_reg(vop_dev, DSP_VACT_ST_END, val);

		/* Second Field Timing */
		vs_st_f1 = vsync_len + upper_margin + y_res / 2 + lower_margin;
		vs_end_f1 = 2 * vsync_len + upper_margin + y_res / 2 +
		    lower_margin;
		val = V_DSP_VS_ST_F1(vs_st_f1) | V_DSP_VS_END_F1(vs_end_f1);
		vop_msk_reg(vop_dev, DSP_VS_ST_END_F1, val);

		vact_end_f1 = 2 * (vsync_len + upper_margin) + y_res +
		    lower_margin + 1;
		vact_st_f1 = 2 * (vsync_len + upper_margin) + y_res / 2 +
		    lower_margin + 1;
		val = V_DSP_VACT_END_F1(vact_end_f1) |
			V_DSP_VACT_ST_F1(vact_st_f1);
		vop_msk_reg(vop_dev, DSP_VACT_ST_END_F1, val);
		vop_msk_reg(vop_dev, DSP_CTRL0,
			    V_DSP_INTERLACE(1) | V_DSP_FIELD_POL(0));

		val = V_DSP_LINE_FLAG_NUM_0(lower_margin ?
					    vact_end_f1 : vact_end_f1 - 1);

		val |= V_DSP_LINE_FLAG_NUM_1(lower_margin ?
					     vact_end_f1 : vact_end_f1 - 1);
		vop_msk_reg(vop_dev, LINE_FLAG, val);
	} else {
		val = V_DSP_VS_END(vsync_len) | V_DSP_VTOTAL(v_total);
		vop_msk_reg(vop_dev, DSP_VTOTAL_VS_END, val);

		val = V_DSP_VACT_END(vsync_len + upper_margin + y_res) |
		    V_DSP_VACT_ST(vsync_len + upper_margin);
		vop_msk_reg(vop_dev, DSP_VACT_ST_END, val);

		vop_msk_reg(vop_dev, DSP_CTRL0, V_DSP_INTERLACE(0) |
			    V_DSP_FIELD_POL(0));

		val = V_DSP_LINE_FLAG_NUM_0(vsync_len + upper_margin + y_res) |
			V_DSP_LINE_FLAG_NUM_1(vsync_len + upper_margin + y_res);
		vop_msk_reg(vop_dev, LINE_FLAG, val);
	}
	vop_post_cfg(dev_drv);

	return 0;
}

static void rk322xh_vop_bcsh_path_sel(struct rk_lcdc_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	u32 bcsh_ctrl;
	u32 r2y_mode = 0, y2r_mode = 0;

	vop_msk_reg(vop_dev, SYS_CTRL, V_OVERLAY_MODE(dev_drv->overlay_mode));
	vop_msk_reg(vop_dev, SYS_CTRL1,
		    V_LEVEL2_OVERLAY_EN(dev_drv->pre_overlay));
	if (dev_drv->overlay_mode == VOP_YUV_DOMAIN) {
		if (IS_YUV_COLOR(dev_drv->output_color)) {	/* bypass */
			vop_msk_reg(vop_dev, BCSH_CTRL,
				    V_BCSH_Y2R_EN(0) | V_BCSH_R2Y_EN(0));
		} else {		/* YUV2RGB */
			if (dev_drv->output_color == COLOR_RGB_BT2020)
				y2r_mode = VOP_CSC_BT2020;
			else
				y2r_mode = VOP_CSC_BT709L;

			vop_msk_reg(vop_dev, BCSH_CTRL, V_BCSH_Y2R_EN(1) |
				    V_BCSH_Y2R_CSC_MODE(y2r_mode) |
				    V_BCSH_R2Y_EN(0));
		}
	} else {
		/* overlay_mode=VOP_RGB_DOMAIN */
		/* bypass  --need check,if bcsh close */
		if (!IS_YUV_COLOR(dev_drv->output_color)) {
			bcsh_ctrl = vop_readl(vop_dev, BCSH_CTRL);
			if (((bcsh_ctrl & MASK(BCSH_EN)) == 1) ||
			    (dev_drv->bcsh.enable == 1))/*bcsh enabled */
				vop_msk_reg(vop_dev, BCSH_CTRL,
					    V_BCSH_R2Y_EN(1) |
					    V_BCSH_Y2R_EN(1));
			else
				vop_msk_reg(vop_dev, BCSH_CTRL,
					    V_BCSH_R2Y_EN(0) |
					    V_BCSH_Y2R_EN(0));
		} else {
			/* RGB2YUV */
			if (dev_drv->output_color == COLOR_YCBCR_BT2020)
				r2y_mode = VOP_CSC_BT2020;
			else if (dev_drv->output_color == COLOR_YCBCR_BT709)
				r2y_mode = VOP_CSC_BT709L;
			else
				r2y_mode = VOP_CSC_BT601L;
			vop_msk_reg(vop_dev, BCSH_CTRL,
				    V_BCSH_R2Y_EN(1) |
				    V_BCSH_R2Y_CSC_MODE(r2y_mode) |
				    V_BCSH_Y2R_EN(0));
		}
	}
}

static int vop_get_dspbuf_info(struct rk_lcdc_driver *dev_drv, u16 *xact,
			       u16 *yact, int *format, u32 *dsp_addr,
			       int *ymirror)
{
	struct vop_device *vop_dev =
			container_of(dev_drv, struct vop_device, driver);
	u32 val;

	spin_lock(&vop_dev->reg_lock);

	val = vop_readl(vop_dev, WIN0_ACT_INFO);
	*xact = (val & MASK(WIN0_ACT_WIDTH)) + 1;
	*yact = ((val & MASK(WIN0_ACT_HEIGHT)) >> 16) + 1;

	val = vop_readl(vop_dev, WIN0_CTRL0);
	*format = (val & MASK(WIN0_DATA_FMT)) >> 1;
	*ymirror = (val & MASK(WIN0_Y_MIR_EN)) >> 22;
	*dsp_addr = vop_readl(vop_dev, WIN0_YRGB_MST);

	spin_unlock(&vop_dev->reg_lock);

	return 0;
}

static int vop_post_dspbuf(struct rk_lcdc_driver *dev_drv, u32 rgb_mst,
			   int format, u16 xact, u16 yact, u16 xvir,
			   int ymirror)
{
	struct vop_device *vop_dev =
			container_of(dev_drv, struct vop_device, driver);
	int swap = (format == RGB888) ? 1 : 0;
	struct rk_lcdc_win *win = dev_drv->win[0];
	u64 val;

	val = V_WIN0_DATA_FMT(format) | V_WIN0_RB_SWAP(swap) |
		V_WIN0_Y_MIR_EN(ymirror);
	vop_msk_reg(vop_dev, WIN0_CTRL0, val);

	vop_msk_reg(vop_dev, WIN0_VIR,	V_WIN0_VIR_STRIDE(xvir));
	vop_writel(vop_dev, WIN0_ACT_INFO, V_WIN0_ACT_WIDTH(xact - 1) |
		   V_WIN0_ACT_HEIGHT(yact - 1));

	vop_writel(vop_dev, WIN0_YRGB_MST, rgb_mst);

	vop_cfg_done(vop_dev);

	if (format == RGB888)
		win->area[0].format = BGR888;
	else
		win->area[0].format = format;

	win->ymirror = ymirror;
	win->state = 1;
	win->last_state = 1;

	return 0;
}

static int vop_load_screen(struct rk_lcdc_driver *dev_drv, bool initscreen)
{
	u16 face = 0;
	u16 dclk_ddr = 0;
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rk_screen *screen = dev_drv->cur_screen;
	u64 val = 0;

	if (unlikely(!vop_dev->clk_on)) {
		pr_info("%s,clk_on = %d\n", __func__, vop_dev->clk_on);
		return 0;
	}

	if (!vop_dev->standby && initscreen && (dev_drv->first_frame != 1))
		flush_kthread_worker(&dev_drv->update_regs_worker);

	spin_lock(&vop_dev->reg_lock);
	if (likely(vop_dev->clk_on)) {
		switch (screen->face) {
		case OUT_P565:
			face = OUT_P565;
			val = V_DITHER_DOWN_EN(1) | V_DITHER_UP_EN(1) |
				V_PRE_DITHER_DOWN_EN(1) |
				V_DITHER_DOWN_SEL(1) | V_DITHER_DOWN_MODE(0);
			break;
		case OUT_P666:
			face = OUT_P666;
			val = V_DITHER_DOWN_EN(1) | V_DITHER_UP_EN(1) |
				V_PRE_DITHER_DOWN_EN(1) |
				V_DITHER_DOWN_SEL(1) | V_DITHER_DOWN_MODE(1);
			break;
		case OUT_D888_P565:
			face = OUT_P888;
			val = V_DITHER_DOWN_EN(1) | V_DITHER_UP_EN(1) |
				V_PRE_DITHER_DOWN_EN(1) |
				V_DITHER_DOWN_SEL(1) | V_DITHER_DOWN_MODE(0);
			break;
		case OUT_D888_P666:
			face = OUT_P888;
			val = V_DITHER_DOWN_EN(1) | V_DITHER_UP_EN(1) |
				V_PRE_DITHER_DOWN_EN(1) |
				V_DITHER_DOWN_SEL(1) | V_DITHER_DOWN_MODE(1);
			break;
		case OUT_P888:
			face = OUT_P888;
			val = V_DITHER_DOWN_EN(0) | V_DITHER_UP_EN(1)
				| V_PRE_DITHER_DOWN_EN(1);
			break;
		case OUT_YUV_420:
			face = OUT_YUV_420;
			dclk_ddr = 1;
			val = V_DITHER_DOWN_EN(0) | V_DITHER_UP_EN(1)
				| V_PRE_DITHER_DOWN_EN(1);
			break;
		case OUT_YUV_420_10BIT:
			face = OUT_YUV_420;
			dclk_ddr = 1;
			val = V_DITHER_DOWN_EN(0) | V_DITHER_UP_EN(1)
				| V_PRE_DITHER_DOWN_EN(0);
				break;
			break;
		case OUT_P101010:
			face = OUT_P101010;
			val = V_DITHER_DOWN_EN(0) | V_DITHER_UP_EN(1)
				| V_PRE_DITHER_DOWN_EN(0);
			break;
		default:
			dev_err(vop_dev->dev, "un supported screen face[%d]!\n",
				screen->face);
			break;
		}

		vop_msk_reg(vop_dev, DSP_CTRL1, val);
		switch (screen->type) {
		case SCREEN_TVOUT:
			val = V_SW_UV_OFFSET_EN(1) | V_SW_IMD_TVE_DCLK_EN(1) |
				V_SW_IMD_TVE_DCLK_EN(1) |
				V_SW_IMD_TVE_DCLK_POL(1) |
				V_SW_GENLOCK(1) | V_SW_DAC_SEL(1);
			if (screen->mode.xres == 720 &&
			    screen->mode.yres == 576)
				val |= V_SW_TVE_MODE(1);
			else
				val |= V_SW_TVE_MODE(0);
			vop_msk_reg(vop_dev, SYS_CTRL, val);
			break;
		case SCREEN_HDMI:
			val = V_HDMI_OUT_EN(1) | V_SW_UV_OFFSET_EN(0);
			vop_msk_reg(vop_dev, SYS_CTRL, val);
			if ((screen->face == OUT_P888) ||
			    (screen->face == OUT_P101010)) {
				face = OUT_P101010;
				val = V_PRE_DITHER_DOWN_EN(0);
				vop_msk_reg(vop_dev, DSP_CTRL1, val);
			}
			break;
		case SCREEN_RGB:
		case SCREEN_LVDS:
			val = V_RGB_OUT_EN(1) | V_HDMI_OUT_EN(1);
			vop_msk_reg(vop_dev, SYS_CTRL, val);
			break;
		default:
			dev_err(vop_dev->dev, "un supported interface[%d]!\n",
				screen->type);
			break;
		}
		val = V_HDMI_HSYNC_POL(screen->pin_hsync) |
			V_HDMI_VSYNC_POL(screen->pin_vsync) |
			V_HDMI_DEN_POL(screen->pin_den) |
			V_HDMI_DCLK_POL(screen->pin_dclk);
		/*hsync vsync den dclk polo,dither */
		vop_msk_reg(vop_dev, DSP_CTRL1, val);

		val = V_DSP_OUT_MODE(face) | V_DSP_DCLK_DDR(dclk_ddr) |
		    V_DSP_BG_SWAP(screen->swap_gb) |
		    V_DSP_RB_SWAP(screen->swap_rb) |
		    V_DSP_RG_SWAP(screen->swap_rg) |
		    V_DSP_DELTA_SWAP(screen->swap_delta) |
		    V_DSP_DUMMY_SWAP(screen->swap_dumy) | V_DSP_OUT_ZERO(0) |
		    V_DSP_BLANK_EN(0) | V_DSP_BLACK_EN(0) |
		    V_DSP_X_MIR_EN(screen->x_mirror) |
		    V_DSP_Y_MIR_EN(screen->y_mirror);
		val |= V_SW_CORE_DCLK_SEL(!!screen->pixelrepeat);
		if (screen->mode.vmode & FB_VMODE_INTERLACED)
			val |= V_SW_P2I_EN(1);
		else
			val |= V_SW_P2I_EN(0);
		vop_msk_reg(vop_dev, DSP_CTRL0, val);

		if (screen->mode.vmode & FB_VMODE_INTERLACED)
			vop_msk_reg(vop_dev, SYS_CTRL1, V_REG_DONE_FRM(1));
		else
			vop_msk_reg(vop_dev, SYS_CTRL1, V_REG_DONE_FRM(0));
		dev_drv->output_color = screen->color_mode;
		rk322xh_vop_csc_cfg(dev_drv);
		/* BG color */
		if (dev_drv->overlay_mode == VOP_YUV_DOMAIN) {
			val = V_DSP_OUT_RGB_YUV(1);
			vop_msk_reg(vop_dev, POST_SCL_CTRL, val);
			val = V_DSP_BG_BLUE(0x200) | V_DSP_BG_GREEN(0x40) |
				V_DSP_BG_RED(0x200);
			vop_msk_reg(vop_dev, DSP_BG, val);
		} else {
			val = V_DSP_OUT_RGB_YUV(0);
			vop_msk_reg(vop_dev, POST_SCL_CTRL, val);
			val = V_DSP_BG_BLUE(0) | V_DSP_BG_GREEN(0) |
				V_DSP_BG_RED(0x3ff);
			vop_msk_reg(vop_dev, DSP_BG, val);
		}
		vop_config_timing(dev_drv);
		vop_cfg_done(vop_dev);
	}
	spin_unlock(&vop_dev->reg_lock);
	vop_set_dclk(dev_drv, 1);
	if (screen->type != SCREEN_HDMI && screen->type != SCREEN_TVOUT &&
	    dev_drv->trsm_ops && dev_drv->trsm_ops->enable)
		dev_drv->trsm_ops->enable();
	if (screen->init)
		screen->init();

	return 0;
}

/*enable layer,open:1,enable;0 disable*/
static void vop_layer_enable(struct vop_device *vop_dev,
			     unsigned int win_id, bool open)
{
	spin_lock(&vop_dev->reg_lock);
	if (likely(vop_dev->clk_on) &&
	    vop_dev->driver.win[win_id]->state != open) {
		if (open) {
			if (!vop_dev->atv_layer_cnt) {
				dev_info(vop_dev->dev,
					 "wakeup from standby!\n");
				vop_dev->standby = 0;
			}
			vop_dev->atv_layer_cnt |= (1 << win_id);
		} else {
			if (vop_dev->atv_layer_cnt & (1 << win_id))
				vop_dev->atv_layer_cnt &= ~(1 << win_id);
		}
		vop_dev->driver.win[win_id]->state = open;
		if (!open) {
			vop_layer_update_regs(vop_dev,
					      vop_dev->driver.win[win_id]);
			vop_cfg_done(vop_dev);
		}
		/* if no layer used,disable lcdc */
		if (!vop_dev->atv_layer_cnt) {
			dev_info(vop_dev->dev,
				 "no layer is used,go to standby!\n");
			vop_dev->standby = 1;
		}
	}
	spin_unlock(&vop_dev->reg_lock);
}

static int vop_enable_irq(struct rk_lcdc_driver *dev_drv)
{
	struct vop_device *vop_dev = container_of(dev_drv,
						    struct vop_device, driver);
	u64 val;
	/* struct rk_screen *screen = dev_drv->cur_screen; */

	vop_mask_writel(vop_dev, INTR_CLEAR0, INTR_MASK, INTR_MASK);

	val = INTR_FS | INTR_LINE_FLAG0 | INTR_BUS_ERROR | INTR_LINE_FLAG1 |
		INTR_WIN0_EMPTY | INTR_WIN1_EMPTY | INTR_HWC_EMPTY |
		INTR_POST_BUF_EMPTY;

	vop_mask_writel(vop_dev, INTR_EN0, INTR_MASK, val);

	return 0;
}

static int vop_open(struct rk_lcdc_driver *dev_drv, int win_id, bool open)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);

	/* enable clk,when first layer open */
	if ((open) && (!vop_dev->atv_layer_cnt)) {
		vop_pre_init(dev_drv);
		vop_clk_enable(vop_dev);
		vop_enable_irq(dev_drv);
		if (dev_drv->iommu_enabled) {
			if (!dev_drv->mmu_dev) {
				dev_drv->mmu_dev =
				    rk_fb_get_sysmmu_device_by_compatible
				    (dev_drv->mmu_dts_name);
				if (dev_drv->mmu_dev) {
					rk_fb_platform_set_sysmmu
					    (dev_drv->mmu_dev, dev_drv->dev);
				} else {
					dev_err(dev_drv->dev,
						"fail get rk iommu device\n");
					return -1;
				}
			}
		}
		if ((support_uboot_display() && (vop_dev->prop == PRMRY)))
			vop_set_dclk(dev_drv, 0);
		else
			vop_load_screen(dev_drv, 1);
		if (dev_drv->bcsh.enable)
			vop_set_bcsh(dev_drv, 1);
		spin_lock(&vop_dev->reg_lock);
		spin_unlock(&vop_dev->reg_lock);
	}

	if (win_id < dev_drv->lcdc_win_num)
		vop_layer_enable(vop_dev, win_id, open);
	else
		dev_err(vop_dev->dev, "invalid win id:%d\n", win_id);

	dev_drv->first_frame = 0;
	return 0;
}

static int win_full_display(struct vop_device *vop_dev, struct rk_lcdc_win *win)
{
	u32 y_addr;
	u32 uv_addr;
	unsigned int off;

	off = win->id * 0x100;
	y_addr = win->area[0].smem_start + win->area[0].y_offset;
	uv_addr = win->area[0].cbr_start + win->area[0].c_offset;
	DBG(2, "lcdc[%d]:win[%d]>>:y_addr:0x%x>>uv_addr:0x%x",
	    vop_dev->id, win->id, y_addr, uv_addr);
	DBG(2, ">>y_offset:0x%x>>c_offset=0x%x\n",
	    win->area[0].y_offset, win->area[0].c_offset);
	spin_lock(&vop_dev->reg_lock);
	if (likely(vop_dev->clk_on)) {
		win->area[0].y_addr = y_addr;
		win->area[0].uv_addr = uv_addr;
		vop_writel(vop_dev, WIN0_YRGB_MST + off, win->area[0].y_addr);
		vop_writel(vop_dev, WIN0_CBR_MST + off, win->area[0].uv_addr);
	}
	spin_unlock(&vop_dev->reg_lock);

	return 0;
}

static int hwc_display(struct vop_device *vop_dev, struct rk_lcdc_win *win)
{
	u32 y_addr;

	y_addr = win->area[0].smem_start + win->area[0].y_offset;
	DBG(2, "lcdc[%d]:hwc>>%s>>y_addr:0x%x>>\n",
	    vop_dev->id, __func__, y_addr);
	spin_lock(&vop_dev->reg_lock);
	if (likely(vop_dev->clk_on)) {
		win->area[0].y_addr = y_addr;
		vop_writel(vop_dev, HWC_MST, win->area[0].y_addr);
	}
	spin_unlock(&vop_dev->reg_lock);

	return 0;
}

static int vop_pan_display(struct rk_lcdc_driver *dev_drv, int win_id)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rk_lcdc_win *win = NULL;
	struct rk_screen *screen = dev_drv->cur_screen;

	win = dev_drv->win[win_id];
	if (!screen) {
		dev_err(dev_drv->dev, "screen is null!\n");
		return -ENOENT;
	}
	if (unlikely(!vop_dev->clk_on)) {
		pr_info("%s,clk_on = %d\n", __func__, vop_dev->clk_on);
		return 0;
	}

	switch (win_id) {
	case 0:
	case 1:
	case 2:
		win_full_display(vop_dev, win);
		break;
	case 3:
		hwc_display(vop_dev, win);
		break;
	default:
		dev_err(dev_drv->dev, "invalid win number:%d!\n", win_id);
		break;
	}

	return 0;
}

static int vop_cal_scl_fac(struct rk_lcdc_win *win, struct rk_screen *screen)
{
	u16 srcW = 0;
	u16 srcH = 0;
	u16 dstW = 0;
	u16 dstH = 0;
	u16 yrgb_srcW = 0;
	u16 yrgb_srcH = 0;
	u16 yrgb_dstW = 0;
	u16 yrgb_dstH = 0;
	u32 yrgb_vscalednmult = 0;
	u32 yrgb_xscl_factor = 0;
	u32 yrgb_yscl_factor = 0;
	u8 yrgb_vsd_bil_gt2 = 0;
	u8 yrgb_vsd_bil_gt4 = 0;

	u16 cbcr_srcW = 0;
	u16 cbcr_srcH = 0;
	u16 cbcr_dstW = 0;
	u16 cbcr_dstH = 0;
	u32 cbcr_vscalednmult = 0;
	u32 cbcr_xscl_factor = 0;
	u32 cbcr_yscl_factor = 0;
	u8 cbcr_vsd_bil_gt2 = 0;
	u8 cbcr_vsd_bil_gt4 = 0;
	u8 yuv_fmt = 0;

	srcW = win->area[0].xact;
	if ((screen->mode.vmode & FB_VMODE_INTERLACED) &&
	    (win->area[0].yact == 2 * win->area[0].ysize)) {
		srcH = win->area[0].yact / 2;
		yrgb_vsd_bil_gt2 = 1;
		cbcr_vsd_bil_gt2 = 1;
	} else {
		srcH = win->area[0].yact;
	}
	dstW = win->area[0].xsize;
	dstH = win->area[0].ysize;

	/*yrgb scl mode */
	yrgb_srcW = srcW;
	yrgb_srcH = srcH;
	yrgb_dstW = dstW;
	yrgb_dstH = dstH;
	if ((yrgb_dstW * 8 <= yrgb_srcW) || (yrgb_dstH * 8 <= yrgb_srcH)) {
		pr_err("ERROR: yrgb scale exceed 8,");
		pr_err("srcW=%d,srcH=%d,dstW=%d,dstH=%d\n",
		       yrgb_srcW, yrgb_srcH, yrgb_dstW, yrgb_dstH);
	}
	if (yrgb_srcW < yrgb_dstW)
		win->yrgb_hor_scl_mode = SCALE_UP;
	else if (yrgb_srcW > yrgb_dstW)
		win->yrgb_hor_scl_mode = SCALE_DOWN;
	else
		win->yrgb_hor_scl_mode = SCALE_NONE;

	if (yrgb_srcH < yrgb_dstH)
		win->yrgb_ver_scl_mode = SCALE_UP;
	else if (yrgb_srcH > yrgb_dstH)
		win->yrgb_ver_scl_mode = SCALE_DOWN;
	else
		win->yrgb_ver_scl_mode = SCALE_NONE;

	/*cbcr scl mode */
	switch (win->area[0].format) {
	case YUV422:
	case YUV422_A:
		cbcr_srcW = srcW / 2;
		cbcr_dstW = dstW;
		cbcr_srcH = srcH;
		cbcr_dstH = dstH;
		yuv_fmt = 1;
		break;
	case YUV420:
	case YUV420_A:
	case YUV420_NV21:
		cbcr_srcW = srcW / 2;
		cbcr_dstW = dstW;
		cbcr_srcH = srcH / 2;
		cbcr_dstH = dstH;
		yuv_fmt = 1;
		break;
	case YUV444:
	case YUV444_A:
		cbcr_srcW = srcW;
		cbcr_dstW = dstW;
		cbcr_srcH = srcH;
		cbcr_dstH = dstH;
		yuv_fmt = 1;
		break;
	default:
		cbcr_srcW = 0;
		cbcr_dstW = 0;
		cbcr_srcH = 0;
		cbcr_dstH = 0;
		yuv_fmt = 0;
		break;
	}
	if (yuv_fmt) {
		if ((cbcr_dstW * 8 <= cbcr_srcW) ||
		    (cbcr_dstH * 8 <= cbcr_srcH)) {
			pr_err("ERROR: cbcr scale exceed 8,");
			pr_err("srcW=%d,srcH=%d,dstW=%d,dstH=%d\n", cbcr_srcW,
			       cbcr_srcH, cbcr_dstW, cbcr_dstH);
		}
	}

	if (cbcr_srcW < cbcr_dstW)
		win->cbr_hor_scl_mode = SCALE_UP;
	else if (cbcr_srcW > cbcr_dstW)
		win->cbr_hor_scl_mode = SCALE_DOWN;
	else
		win->cbr_hor_scl_mode = SCALE_NONE;

	if (cbcr_srcH < cbcr_dstH)
		win->cbr_ver_scl_mode = SCALE_UP;
	else if (cbcr_srcH > cbcr_dstH)
		win->cbr_ver_scl_mode = SCALE_DOWN;
	else
		win->cbr_ver_scl_mode = SCALE_NONE;

	/* line buffer mode */
	if ((win->area[0].format == YUV422) ||
	    (win->area[0].format == YUV420) ||
	    (win->area[0].format == YUV420_NV21) ||
	    (win->area[0].format == YUV422_A) ||
	    (win->area[0].format == YUV420_A)) {
		if (win->cbr_hor_scl_mode == SCALE_DOWN) {
			if ((cbcr_dstW > VOP_INPUT_MAX_WIDTH / 2) ||
			    (cbcr_dstW == 0))
				pr_err("ERROR cbcr_dstW = %d,exceeds 2048\n",
				       cbcr_dstW);
			else if (cbcr_dstW > 1280)
				win->win_lb_mode = LB_YUV_3840X5;
			else
				win->win_lb_mode = LB_YUV_2560X8;
		} else {	/* SCALE_UP or SCALE_NONE */
			if ((cbcr_srcW > VOP_INPUT_MAX_WIDTH / 2) ||
			    (cbcr_srcW == 0))
				pr_err("ERROR cbcr_srcW = %d,exceeds 2048\n",
				       cbcr_srcW);
			else if (cbcr_srcW > 1280)
				win->win_lb_mode = LB_YUV_3840X5;
			else
				win->win_lb_mode = LB_YUV_2560X8;
		}
	} else {
		if (win->yrgb_hor_scl_mode == SCALE_DOWN) {
			if ((yrgb_dstW > VOP_INPUT_MAX_WIDTH) ||
			    (yrgb_dstW == 0))
				pr_err("ERROR yrgb_dstW = %d\n", yrgb_dstW);
			else if (yrgb_dstW > 2560)
				win->win_lb_mode = LB_RGB_3840X2;
			else if (yrgb_dstW > 1920)
				win->win_lb_mode = LB_RGB_2560X4;
			else if (yrgb_dstW > 1280)
				win->win_lb_mode = LB_RGB_1920X5;
			else
				win->win_lb_mode = LB_RGB_1280X8;
		} else {	/* SCALE_UP or SCALE_NONE */
			if ((yrgb_srcW > VOP_INPUT_MAX_WIDTH) ||
			    (yrgb_srcW == 0))
				pr_err("ERROR yrgb_srcW = %d\n", yrgb_srcW);
			else if (yrgb_srcW > 2560)
				win->win_lb_mode = LB_RGB_3840X2;
			else if (yrgb_srcW > 1920)
				win->win_lb_mode = LB_RGB_2560X4;
			else if (yrgb_srcW > 1280)
				win->win_lb_mode = LB_RGB_1920X5;
			else
				win->win_lb_mode = LB_RGB_1280X8;
		}
	}
	DBG(1, "win->win_lb_mode = %d;\n", win->win_lb_mode);

	/* vsd/vsu scale ALGORITHM */
	win->yrgb_hsd_mode = SCALE_DOWN_BIL;	/*not to specify */
	win->cbr_hsd_mode = SCALE_DOWN_BIL;	/*not to specify */
	win->yrgb_vsd_mode = SCALE_DOWN_BIL;	/*not to specify */
	win->cbr_vsd_mode = SCALE_DOWN_BIL;	/*not to specify */
	switch (win->win_lb_mode) {
	case LB_YUV_3840X5:
	case LB_YUV_2560X8:
	case LB_RGB_1920X5:
	case LB_RGB_1280X8:
		win->yrgb_vsu_mode = SCALE_UP_BIC;
		win->cbr_vsu_mode = SCALE_UP_BIC;
		break;
	case LB_RGB_3840X2:
		if (win->yrgb_ver_scl_mode != SCALE_NONE)
			pr_err("ERROR : not allow yrgb ver scale\n");
		if (win->cbr_ver_scl_mode != SCALE_NONE)
			pr_err("ERROR : not allow cbcr ver scale\n");
		break;
	case LB_RGB_2560X4:
		win->yrgb_vsu_mode = SCALE_UP_BIL;
		win->cbr_vsu_mode = SCALE_UP_BIL;
		break;
	default:
		pr_info("%s:un supported win_lb_mode:%d\n",
			__func__, win->win_lb_mode);
		break;
	}

	if (win->ymirror == 1)
		win->yrgb_vsd_mode = SCALE_DOWN_BIL;
	if (screen->mode.vmode & FB_VMODE_INTERLACED) {
		/* interlace mode must bill */
		win->yrgb_vsd_mode = SCALE_DOWN_BIL;
		win->cbr_vsd_mode = SCALE_DOWN_BIL;
	}
	if ((win->yrgb_ver_scl_mode == SCALE_DOWN) &&
	    (win->area[0].fbdc_en == 1)) {
		/*
		 * in this pattern,use bil mode,not support souble scd,
		 * use avg mode, support double scd, but aclk should be
		 * bigger than dclk,aclk>>dclk
		 */
		if (yrgb_srcH >= 2 * yrgb_dstH) {
			pr_err("ERROR : fbdc mode,not support y scale down:");
			pr_err("srcH[%d] > 2 *dstH[%d]\n",
			       yrgb_srcH, yrgb_dstH);
		}
	}
	DBG(1, "yrgb:hsd=%d,vsd=%d,vsu=%d;cbcr:hsd=%d,vsd=%d,vsu=%d\n",
	    win->yrgb_hsd_mode, win->yrgb_vsd_mode, win->yrgb_vsu_mode,
	    win->cbr_hsd_mode, win->cbr_vsd_mode, win->cbr_vsu_mode);

	/* SCALE FACTOR */

	/* (1.1)YRGB HOR SCALE FACTOR */
	switch (win->yrgb_hor_scl_mode) {
	case SCALE_NONE:
		yrgb_xscl_factor = (1 << SCALE_FACTOR_DEFAULT_FIXPOINT_SHIFT);
		break;
	case SCALE_UP:
		yrgb_xscl_factor = GET_SCALE_FACTOR_BIC(yrgb_srcW, yrgb_dstW);
		break;
	case SCALE_DOWN:
		switch (win->yrgb_hsd_mode) {
		case SCALE_DOWN_BIL:
			yrgb_xscl_factor =
			    GET_SCALE_FACTOR_BILI_DN(yrgb_srcW, yrgb_dstW);
			break;
		case SCALE_DOWN_AVG:
			yrgb_xscl_factor =
			    GET_SCALE_FACTOR_AVRG(yrgb_srcW, yrgb_dstW);
			break;
		default:
			pr_info("%s:un supported yrgb_hsd_mode:%d\n", __func__,
				win->yrgb_hsd_mode);
			break;
		}
		break;
	default:
		pr_info("%s:un supported yrgb_hor_scl_mode:%d\n",
			__func__, win->yrgb_hor_scl_mode);
		break;
	}

	/* (1.2)YRGB VER SCALE FACTOR */
	switch (win->yrgb_ver_scl_mode) {
	case SCALE_NONE:
		yrgb_yscl_factor = (1 << SCALE_FACTOR_DEFAULT_FIXPOINT_SHIFT);
		break;
	case SCALE_UP:
		switch (win->yrgb_vsu_mode) {
		case SCALE_UP_BIL:
			yrgb_yscl_factor =
			    GET_SCALE_FACTOR_BILI_UP(yrgb_srcH, yrgb_dstH);
			break;
		case SCALE_UP_BIC:
			if (yrgb_srcH < 3) {
				pr_err("yrgb_srcH should be");
				pr_err(" greater than 3 !!!\n");
			}
			yrgb_yscl_factor = GET_SCALE_FACTOR_BIC(yrgb_srcH,
								yrgb_dstH);
			break;
		default:
			pr_info("%s:un support yrgb_vsu_mode:%d\n",
				__func__, win->yrgb_vsu_mode);
			break;
		}
		break;
	case SCALE_DOWN:
		switch (win->yrgb_vsd_mode) {
		case SCALE_DOWN_BIL:
			yrgb_vscalednmult =
			    vop_get_hard_ware_vskiplines(yrgb_srcH, yrgb_dstH);
			yrgb_yscl_factor =
			    GET_SCALE_FACTOR_BILI_DN_VSKIP(yrgb_srcH, yrgb_dstH,
							   yrgb_vscalednmult);
			if (yrgb_yscl_factor >= 0x2000) {
				pr_err("yrgb_yscl_factor should less 0x2000");
				pr_err("yrgb_yscl_factor=%4x;\n",
				       yrgb_yscl_factor);
			}
			if (yrgb_vscalednmult == 4) {
				yrgb_vsd_bil_gt4 = 1;
				yrgb_vsd_bil_gt2 = 0;
			} else if (yrgb_vscalednmult == 2) {
				yrgb_vsd_bil_gt4 = 0;
				yrgb_vsd_bil_gt2 = 1;
			} else {
				yrgb_vsd_bil_gt4 = 0;
				yrgb_vsd_bil_gt2 = 0;
			}
			break;
		case SCALE_DOWN_AVG:
			yrgb_yscl_factor = GET_SCALE_FACTOR_AVRG(yrgb_srcH,
								 yrgb_dstH);
			break;
		default:
			pr_info("%s:un support yrgb_vsd_mode:%d\n",
				__func__, win->yrgb_vsd_mode);
			break;
		}		/*win->yrgb_vsd_mode */
		break;
	default:
		pr_info("%s:un supported yrgb_ver_scl_mode:%d\n",
			__func__, win->yrgb_ver_scl_mode);
		break;
	}
	win->scale_yrgb_x = yrgb_xscl_factor;
	win->scale_yrgb_y = yrgb_yscl_factor;
	win->vsd_yrgb_gt4 = yrgb_vsd_bil_gt4;
	win->vsd_yrgb_gt2 = yrgb_vsd_bil_gt2;
	DBG(1, "yrgb:h_fac=%d, V_fac=%d,gt4=%d, gt2=%d\n", yrgb_xscl_factor,
	    yrgb_yscl_factor, yrgb_vsd_bil_gt4, yrgb_vsd_bil_gt2);

	/*(2.1)CBCR HOR SCALE FACTOR */
	switch (win->cbr_hor_scl_mode) {
	case SCALE_NONE:
		cbcr_xscl_factor = (1 << SCALE_FACTOR_DEFAULT_FIXPOINT_SHIFT);
		break;
	case SCALE_UP:
		cbcr_xscl_factor = GET_SCALE_FACTOR_BIC(cbcr_srcW, cbcr_dstW);
		break;
	case SCALE_DOWN:
		switch (win->cbr_hsd_mode) {
		case SCALE_DOWN_BIL:
			cbcr_xscl_factor =
			    GET_SCALE_FACTOR_BILI_DN(cbcr_srcW, cbcr_dstW);
			break;
		case SCALE_DOWN_AVG:
			cbcr_xscl_factor =
			    GET_SCALE_FACTOR_AVRG(cbcr_srcW, cbcr_dstW);
			break;
		default:
			pr_info("%s:un support cbr_hsd_mode:%d\n",
				__func__, win->cbr_hsd_mode);
			break;
		}
		break;
	default:
		pr_info("%s:un supported cbr_hor_scl_mode:%d\n",
			__func__, win->cbr_hor_scl_mode);
		break;
	}			/*win->cbr_hor_scl_mode */

	/* (2.2)CBCR VER SCALE FACTOR */
	switch (win->cbr_ver_scl_mode) {
	case SCALE_NONE:
		cbcr_yscl_factor = (1 << SCALE_FACTOR_DEFAULT_FIXPOINT_SHIFT);
		break;
	case SCALE_UP:
		switch (win->cbr_vsu_mode) {
		case SCALE_UP_BIL:
			cbcr_yscl_factor =
			    GET_SCALE_FACTOR_BILI_UP(cbcr_srcH, cbcr_dstH);
			break;
		case SCALE_UP_BIC:
			if (cbcr_srcH < 3) {
				pr_err("cbcr_srcH should be ");
				pr_err("greater than 3 !!!\n");
			}
			cbcr_yscl_factor = GET_SCALE_FACTOR_BIC(cbcr_srcH,
								cbcr_dstH);
			break;
		default:
			pr_info("%s:un support cbr_vsu_mode:%d\n",
				__func__, win->cbr_vsu_mode);
			break;
		}
		break;
	case SCALE_DOWN:
		switch (win->cbr_vsd_mode) {
		case SCALE_DOWN_BIL:
			cbcr_vscalednmult =
			    vop_get_hard_ware_vskiplines(cbcr_srcH, cbcr_dstH);
			cbcr_yscl_factor =
			    GET_SCALE_FACTOR_BILI_DN_VSKIP(cbcr_srcH, cbcr_dstH,
							   cbcr_vscalednmult);
			if (cbcr_yscl_factor >= 0x2000) {
				pr_err("cbcr_yscl_factor should be less ");
				pr_err("than 0x2000,cbcr_yscl_factor=%4x;\n",
				       cbcr_yscl_factor);
			}

			if (cbcr_vscalednmult == 4) {
				cbcr_vsd_bil_gt4 = 1;
				cbcr_vsd_bil_gt2 = 0;
			} else if (cbcr_vscalednmult == 2) {
				cbcr_vsd_bil_gt4 = 0;
				cbcr_vsd_bil_gt2 = 1;
			} else {
				cbcr_vsd_bil_gt4 = 0;
				cbcr_vsd_bil_gt2 = 0;
			}
			break;
		case SCALE_DOWN_AVG:
			cbcr_yscl_factor = GET_SCALE_FACTOR_AVRG(cbcr_srcH,
								 cbcr_dstH);
			break;
		default:
			pr_info("%s:un support cbr_vsd_mode:%d\n",
				__func__, win->cbr_vsd_mode);
			break;
		}
		break;
	default:
		pr_info("%s:un supported cbr_ver_scl_mode:%d\n",
			__func__, win->cbr_ver_scl_mode);
		break;
	}
	win->scale_cbcr_x = cbcr_xscl_factor;
	win->scale_cbcr_y = cbcr_yscl_factor;
	win->vsd_cbr_gt4 = cbcr_vsd_bil_gt4;
	win->vsd_cbr_gt2 = cbcr_vsd_bil_gt2;

	DBG(1, "cbcr:h_fac=%d,v_fac=%d,gt4=%d,gt2=%d\n", cbcr_xscl_factor,
	    cbcr_yscl_factor, cbcr_vsd_bil_gt4, cbcr_vsd_bil_gt2);
	return 0;
}

static int dsp_x_pos(int mirror_en, struct rk_screen *screen,
		     struct rk_lcdc_win_area *area)
{
	int pos;

	if (screen->x_mirror && mirror_en)
		pr_err("not support both win and global mirror\n");

	if ((!mirror_en) && (!screen->x_mirror))
		pos = area->xpos + screen->mode.left_margin +
			screen->mode.hsync_len;
	else
		pos = screen->mode.xres - area->xpos -
			area->xsize + screen->mode.left_margin +
			screen->mode.hsync_len;

	return pos;
}

static int dsp_y_pos(int mirror_en, struct rk_screen *screen,
		     struct rk_lcdc_win_area *area)
{
	int pos;

	if (screen->y_mirror && mirror_en)
		pr_err("not support both win and global mirror\n");

	if ((!mirror_en) && (!screen->y_mirror))
		pos = area->ypos + screen->mode.upper_margin +
			screen->mode.vsync_len;
	else
		pos = screen->mode.yres - area->ypos -
			area->ysize + screen->mode.upper_margin +
			screen->mode.vsync_len;

	return pos;
}

static int win_full_set_par(struct vop_device *vop_dev,
			    struct rk_screen *screen, struct rk_lcdc_win *win)
{
	u32 xact = 0, yact = 0, xvir = 0, yvir = 0, xpos = 0, ypos = 0;
	u8 fmt_cfg = 0, swap_rb = 0, swap_uv = 0;
	char fmt[9] = "NULL";

	xpos = dsp_x_pos(win->xmirror, screen, &win->area[0]);
	ypos = dsp_y_pos(win->ymirror, screen, &win->area[0]);

	spin_lock(&vop_dev->reg_lock);
	if (likely(vop_dev->clk_on)) {
		vop_cal_scl_fac(win, screen);
		switch (win->area[0].format) {
		case FBDC_RGB_565:
			fmt_cfg = 2;
			swap_rb = 0;
			win->fmt_10 = 0;
			win->area[0].fbdc_fmt_cfg = 0x05;
			break;
		case FBDC_ARGB_888:
			fmt_cfg = 0;
			swap_rb = 0;
			win->fmt_10 = 0;
			win->area[0].fbdc_fmt_cfg = 0x0c;
			break;
		case FBDC_ABGR_888:
			fmt_cfg = 0;
			swap_rb = 1;
			win->fmt_10 = 0;
			win->area[0].fbdc_fmt_cfg = 0x0c;
			break;
		case FBDC_RGBX_888:
			fmt_cfg = 0;
			swap_rb = 0;
			win->fmt_10 = 0;
			win->area[0].fbdc_fmt_cfg = 0x3a;
			break;
		case ARGB888:
			fmt_cfg = 0;
			swap_rb = 0;
			win->fmt_10 = 0;
			break;
		case XBGR888:
		case ABGR888:
			fmt_cfg = 0;
			swap_rb = 1;
			win->fmt_10 = 0;
			break;
		case BGR888:
			fmt_cfg = 1;
			swap_rb = 1;
			win->fmt_10 = 0;
			break;
		case RGB888:
			fmt_cfg = 1;
			swap_rb = 0;
			win->fmt_10 = 0;
			break;
		case RGB565:
			fmt_cfg = 2;
			swap_rb = 0;
			win->fmt_10 = 0;
			break;
		case YUV422:
			fmt_cfg = 5;
			swap_rb = 0;
			win->fmt_10 = 0;
			break;
		case YUV420:
			fmt_cfg = 4;
			swap_rb = 0;
			win->fmt_10 = 0;
			break;
		case YUV420_NV21:
			fmt_cfg = 4;
			swap_rb = 0;
			swap_uv = 1;
			win->fmt_10 = 0;
			break;
		case YUV444:
			fmt_cfg = 6;
			swap_rb = 0;
			win->fmt_10 = 0;
			break;
		case YUV422_A:
			fmt_cfg = 5;
			swap_rb = 0;
			win->fmt_10 = 1;
			break;
		case YUV420_A:
			fmt_cfg = 4;
			swap_rb = 0;
			win->fmt_10 = 1;
			break;
		case YUV444_A:
			fmt_cfg = 6;
			swap_rb = 0;
			win->fmt_10 = 1;
			break;
		default:
			dev_err(vop_dev->dev, "%s:unsupport format[%d]!\n",
				__func__, win->area[0].format);
			break;
		}
		win->area[0].fmt_cfg = fmt_cfg;
		win->area[0].swap_rb = swap_rb;
		win->area[0].swap_uv = swap_uv;
		win->area[0].dsp_stx = xpos;
		win->area[0].dsp_sty = ypos;
		xact = win->area[0].xact;
		yact = win->area[0].yact;
		xvir = win->area[0].xvir;
		yvir = win->area[0].yvir;
	}
	spin_unlock(&vop_dev->reg_lock);

	DBG(1, "lcdc[%d]:win[%d]\n>>format:%s>>>xact:%d>>yact:%d>>xsize:%d",
	    vop_dev->id, win->id, get_format_string(win->area[0].format, fmt),
	    xact, yact, win->area[0].xsize);
	DBG(1, ">>ysize:%d>>xvir:%d>>yvir:%d>>xpos:%d>>ypos:%d>>\n",
	    win->area[0].ysize, xvir, yvir, xpos, ypos);

	return 0;
}

static int hwc_set_par(struct vop_device *vop_dev,
		       struct rk_screen *screen, struct rk_lcdc_win *win)
{
	u32 xact = 0, yact = 0, xvir = 0, yvir = 0, xpos = 0, ypos = 0;
	u8 fmt_cfg = 0, swap_rb = 0;
	char fmt[9] = "NULL";

	xpos = win->area[0].xpos + screen->mode.left_margin +
	    screen->mode.hsync_len;
	ypos = win->area[0].ypos + screen->mode.upper_margin +
	    screen->mode.vsync_len;

	spin_lock(&vop_dev->reg_lock);
	if (likely(vop_dev->clk_on)) {
		switch (win->area[0].format) {
		case ARGB888:
			fmt_cfg = 0;
			swap_rb = 0;
			break;
		case XBGR888:
		case ABGR888:
			fmt_cfg = 0;
			swap_rb = 1;
			break;
		case RGB888:
			fmt_cfg = 1;
			swap_rb = 0;
			break;
		case RGB565:
			fmt_cfg = 2;
			swap_rb = 0;
			break;
		default:
			dev_err(vop_dev->dev, "%s:un supported format[%d]!\n",
				__func__, win->area[0].format);
			break;
		}
		win->area[0].fmt_cfg = fmt_cfg;
		win->area[0].swap_rb = swap_rb;
		win->area[0].dsp_stx = xpos;
		win->area[0].dsp_sty = ypos;
		xact = win->area[0].xact;
		yact = win->area[0].yact;
		xvir = win->area[0].xvir;
		yvir = win->area[0].yvir;
	}
	spin_unlock(&vop_dev->reg_lock);

	DBG(1, "lcdc[%d]:hwc>>%s\n>>format:%s>>>xact:%d>>yact:%d>>xsize:%d",
	    vop_dev->id, __func__, get_format_string(win->area[0].format, fmt),
	    xact, yact, win->area[0].xsize);
	DBG(1, ">>ysize:%d>>xvir:%d>>yvir:%d>>xpos:%d>>ypos:%d>>\n",
	    win->area[0].ysize, xvir, yvir, xpos, ypos);
	return 0;
}

static int vop_set_par(struct rk_lcdc_driver *dev_drv, int win_id)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rk_lcdc_win *win = NULL;
	struct rk_screen *screen = dev_drv->cur_screen;

	if (unlikely(!vop_dev->clk_on)) {
		pr_info("%s,clk_on = %d\n", __func__, vop_dev->clk_on);
		return 0;
	}
	win = dev_drv->win[win_id];
	switch (win_id) {
	case 0:
	case 1:
	case 2:
		win_full_set_par(vop_dev, screen, win);
		break;
	case 3:
		hwc_set_par(vop_dev, screen, win);
		break;
	default:
		dev_err(dev_drv->dev, "unsupported win number:%d\n", win_id);
		break;
	}
	return 0;
}

static int vop_ioctl(struct rk_lcdc_driver *dev_drv, unsigned int cmd,
		     unsigned long arg, int win_id)
{
	struct vop_device *vop_dev =
			container_of(dev_drv, struct vop_device, driver);
	u32 panel_size[2];
	void __user *argp = (void __user *)arg;
	struct color_key_cfg clr_key_cfg;

	switch (cmd) {
	case RK_FBIOGET_PANEL_SIZE:
		panel_size[0] = vop_dev->screen->mode.xres;
		panel_size[1] = vop_dev->screen->mode.yres;
		if (copy_to_user(argp, panel_size, 8))
			return -EFAULT;
		break;
	case RK_FBIOPUT_COLOR_KEY_CFG:
		if (copy_from_user(&clr_key_cfg, argp,
				   sizeof(struct color_key_cfg)))
			return -EFAULT;
		vop_clr_key_cfg(dev_drv);
		vop_writel(vop_dev, WIN0_COLOR_KEY,
			   clr_key_cfg.win0_color_key_cfg);
		vop_writel(vop_dev, WIN1_COLOR_KEY,
			   clr_key_cfg.win1_color_key_cfg);
		break;

	default:
		break;
	}
	return 0;
}

static int vop_early_suspend(struct rk_lcdc_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);

	if (dev_drv->suspend_flag)
		return 0;

	dev_drv->suspend_flag = 1;
	smp_wmb();
	flush_kthread_worker(&dev_drv->update_regs_worker);

	if (dev_drv->trsm_ops && dev_drv->trsm_ops->disable)
		dev_drv->trsm_ops->disable();

	if (likely(vop_dev->clk_on)) {
		spin_lock(&vop_dev->reg_lock);
		vop_msk_reg(vop_dev, DSP_CTRL0, V_DSP_BLANK_EN(1));
		vop_mask_writel(vop_dev, INTR_CLEAR0, INTR_MASK, INTR_MASK);
		vop_msk_reg(vop_dev, DSP_CTRL0, V_DSP_OUT_ZERO(1));
		vop_msk_reg(vop_dev, SYS_CTRL, V_VOP_STANDBY_EN(1));
		vop_cfg_done(vop_dev);

		if (dev_drv->iommu_enabled && dev_drv->mmu_dev) {
			mdelay(50);
			rockchip_iovmm_deactivate(dev_drv->dev);
		}

		spin_unlock(&vop_dev->reg_lock);
	}

	vop_clk_disable(vop_dev);
	rk_disp_pwr_disable(dev_drv);

	return 0;
}

static int vop_early_resume(struct rk_lcdc_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);

	if (!dev_drv->suspend_flag)
		return 0;
	rk_disp_pwr_enable(dev_drv);

	vop_clk_enable(vop_dev);
	spin_lock(&vop_dev->reg_lock);
	memcpy(vop_dev->regs, vop_dev->regsbak, vop_dev->len);
	spin_unlock(&vop_dev->reg_lock);

	spin_lock(&vop_dev->reg_lock);

	vop_msk_reg(vop_dev, DSP_CTRL0, V_DSP_OUT_ZERO(0));
	vop_msk_reg(vop_dev, SYS_CTRL, V_VOP_STANDBY_EN(0));
	vop_msk_reg(vop_dev, DSP_CTRL0, V_DSP_BLANK_EN(0));
	vop_cfg_done(vop_dev);
	spin_unlock(&vop_dev->reg_lock);

	if (dev_drv->iommu_enabled && dev_drv->mmu_dev) {
		/* win address maybe effect after next frame start,
		 * but mmu maybe effect right now, so we delay 50ms
		 */
		mdelay(50);
		rockchip_iovmm_activate(dev_drv->dev);
	}

	dev_drv->suspend_flag = 0;

	if (dev_drv->trsm_ops && dev_drv->trsm_ops->enable)
		dev_drv->trsm_ops->enable();

	return 0;
}

static int vop_blank(struct rk_lcdc_driver *dev_drv, int win_id, int blank_mode)
{
	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		vop_early_resume(dev_drv);
		break;
	case FB_BLANK_NORMAL:
		vop_early_suspend(dev_drv);
		break;
	default:
		vop_early_suspend(dev_drv);
		break;
	}

	dev_info(dev_drv->dev, "blank mode:%d\n", blank_mode);

	return 0;
}

static int vop_get_win_state(struct rk_lcdc_driver *dev_drv,
			     int win_id, int area_id)
{
	struct vop_device *vop_dev =
			container_of(dev_drv, struct vop_device, driver);
	u32 area_status = 0, state = 0;

	switch (win_id) {
	case 0:
		area_status = vop_read_bit(vop_dev, WIN0_CTRL0, V_WIN0_EN(0));
		break;
	case 1:
		area_status = vop_read_bit(vop_dev, WIN1_CTRL0, V_WIN1_EN(0));
		break;
	case 2:
		area_status = vop_read_bit(vop_dev, WIN1_CTRL0, V_WIN2_EN(0));
		break;
	case 3:
		area_status = vop_read_bit(vop_dev, HWC_CTRL0, V_HWC_EN(0));
		break;
	default:
		pr_err("!!!%s,win[%d]area[%d],unsupport!!!\n",
		       __func__, win_id, area_id);
		break;
	}

	state = (area_status > 0) ? 1 : 0;
	return state;
}

static int vop_get_area_num(struct rk_lcdc_driver *dev_drv,
			    unsigned int *area_support)
{
	area_support[0] = 1;
	area_support[1] = 1;
	area_support[2] = 1;

	return 0;
}

/*overlay will be do at regupdate*/
static int vop_ovl_mgr(struct rk_lcdc_driver *dev_drv, int swap, bool set)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rk_lcdc_win *win = NULL;
	int i, ovl = 0;
	u64 val;
	int z_order_num = 0;
	int layer0_sel = 0, layer1_sel = 1, layer2_sel = 2, layer3_sel = 3;

	spin_lock(&vop_dev->reg_lock);
	if ((swap == 0) && (vop_dev->clk_on)) {
		rk322xh_vop_csc_cfg(dev_drv);
		for (i = 0; i < dev_drv->lcdc_win_num; i++) {
			win = dev_drv->win[i];
			if (win->state == 1)
				z_order_num++;
		}
		for (i = 0; i < dev_drv->lcdc_win_num; i++) {
			win = dev_drv->win[i];
			if ((win->id == 0) && (win->state) &&
			    (dev_drv->pre_overlay) && (win->z_order != 0))
				pr_err("error: two overlay win0 must be layer0\n");
			if (win->state == 0)
				win->z_order = z_order_num++;
			switch (win->z_order) {
			case 0:
				layer0_sel = win->id;
				break;
			case 1:
				layer1_sel = win->id;
				break;
			case 2:
				layer2_sel = win->id;
				break;
			case 3:
				layer3_sel = win->id;
				break;
			default:
				break;
			}
		}
	} else {
		layer0_sel = swap % 10;
		layer1_sel = swap / 10 % 10;
		layer2_sel = swap / 100 % 10;
		layer3_sel = swap / 1000;
	}

	if (vop_dev->clk_on) {
		if (set) {
			val = V_DSP_LAYER0_SEL(layer0_sel) |
			    V_DSP_LAYER1_SEL(layer1_sel) |
			    V_DSP_LAYER2_SEL(layer2_sel) |
			    V_DSP_LAYER3_SEL(layer3_sel);
			vop_msk_reg(vop_dev, DSP_CTRL1, val);
		} else {
			layer0_sel = vop_read_bit(vop_dev, DSP_CTRL1,
						  V_DSP_LAYER0_SEL(0));
			layer1_sel = vop_read_bit(vop_dev, DSP_CTRL1,
						  V_DSP_LAYER1_SEL(0));
			layer2_sel = vop_read_bit(vop_dev, DSP_CTRL1,
						  V_DSP_LAYER2_SEL(0));
			layer3_sel = vop_read_bit(vop_dev, DSP_CTRL1,
						  V_DSP_LAYER3_SEL(0));
			ovl = layer3_sel * 1000 + layer2_sel * 100 +
			    layer1_sel * 10 + layer0_sel;
		}
	} else {
		ovl = -EPERM;
	}
	spin_unlock(&vop_dev->reg_lock);

	return ovl;
}

static int vop_color_space_to_string(u8 color_space, char *color)
{
	if (!color)
		return 0;

	switch (color_space) {
	case CSC_BT601:
		strcpy(color, "BT601L");
		break;
	case CSC_BT709:
		strcpy(color, "BT709L");
		break;
	case CSC_BT2020:
		strcpy(color, "BT2020");
		break;
	case CSC_BT601F:
		strcpy(color, "BT601F");
		break;
	default:
		strcpy(color, "invalid\n");
		break;
	}

	return 0;
}

static char *vop_format_to_string(int format, char *fmt)
{
	if (!fmt)
		return NULL;

	switch (format) {
	case 0:
		strcpy(fmt, "ARGB888");
		break;
	case 1:
		strcpy(fmt, "RGB888");
		break;
	case 2:
		strcpy(fmt, "RGB565");
		break;
	case 4:
		strcpy(fmt, "YCbCr420");
		break;
	case 5:
		strcpy(fmt, "YCbCr422");
		break;
	case 6:
		strcpy(fmt, "YCbCr444");
		break;
	default:
		strcpy(fmt, "invalid\n");
		break;
	}
	return fmt;
}

static ssize_t vop_get_disp_info(struct rk_lcdc_driver *dev_drv,
				 char *buf, int win_id)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rk_screen *screen = dev_drv->cur_screen;
	u16 hsync_len = screen->mode.hsync_len;
	u16 left_margin = screen->mode.left_margin;
	u16 vsync_len = screen->mode.vsync_len;
	u16 upper_margin = screen->mode.upper_margin;
	u32 h_pw_bp = hsync_len + left_margin;
	u32 v_pw_bp = vsync_len + upper_margin;
	u32 fmt_id;
	char format_w0[9] = "NULL";
	char format_w1[9] = "NULL";
	char format_w2[9] = "NULL";
	char color_space_w0[9] = "NULL";
	char color_space_w1[9] = "NULL";
	char color_space_w2[9] = "NULL";
	char dsp_buf[100];
	u32 win_ctrl, zorder, vir_info, act_info, dsp_info, dsp_st;
	u32 y_factor, uv_factor;
	u8 layer0_sel, layer1_sel, layer2_sel;
	u8 w0_state, w1_state, w2_state;

	u32 w0_vir_y, w0_vir_uv, w0_act_x, w0_act_y, w0_dsp_x, w0_dsp_y;
	u32 w0_st_x = h_pw_bp, w0_st_y = v_pw_bp;
	u32 w1_vir_y, w1_vir_uv, w1_act_x, w1_act_y, w1_dsp_x, w1_dsp_y;
	u32 w1_st_x = h_pw_bp, w1_st_y = v_pw_bp;
	u32 w2_vir_y, w2_vir_uv, w2_act_x, w2_act_y, w2_dsp_x, w2_dsp_y;
	u32 w2_st_x = h_pw_bp, w2_st_y = v_pw_bp;
	u32 w0_y_h_fac, w0_y_v_fac, w0_uv_h_fac, w0_uv_v_fac;
	u32 w1_y_h_fac, w1_y_v_fac, w1_uv_h_fac, w1_uv_v_fac;
	u32 w2_y_h_fac, w2_y_v_fac, w2_uv_h_fac, w2_uv_v_fac;

	u32 dclk_freq, overlay_mode;
	int size = 0;

	dclk_freq = screen->mode.pixclock;
	/*vop_reg_dump(dev_drv); */

	spin_lock(&vop_dev->reg_lock);
	if (vop_dev->clk_on) {
		zorder = vop_readl(vop_dev, DSP_CTRL1);
		layer0_sel = (zorder & MASK(DSP_LAYER0_SEL)) >> 8;
		layer1_sel = (zorder & MASK(DSP_LAYER1_SEL)) >> 10;
		layer2_sel = (zorder & MASK(DSP_LAYER2_SEL)) >> 12;
		overlay_mode = vop_read_bit(vop_dev, SYS_CTRL,
					    V_OVERLAY_MODE(0));
		/* WIN0 */
		win_ctrl = vop_readl(vop_dev, WIN0_CTRL0);
		w0_state = win_ctrl & MASK(WIN0_EN);
		fmt_id = (win_ctrl & MASK(WIN0_DATA_FMT)) >> 1;
		vop_format_to_string(fmt_id, format_w0);
		vop_color_space_to_string(dev_drv->win[0]->colorspace,
					  color_space_w0);
		vir_info = vop_readl(vop_dev, WIN0_VIR);
		act_info = vop_readl(vop_dev, WIN0_ACT_INFO);
		dsp_info = vop_readl(vop_dev, WIN0_DSP_INFO);
		dsp_st = vop_readl(vop_dev, WIN0_DSP_ST);
		y_factor = vop_readl(vop_dev, WIN0_SCL_FACTOR_YRGB);
		uv_factor = vop_readl(vop_dev, WIN0_SCL_FACTOR_CBR);
		w0_vir_y = vir_info & MASK(WIN0_VIR_STRIDE);
		w0_vir_uv = (vir_info & MASK(WIN0_VIR_STRIDE_UV)) >> 16;
		w0_act_x = (act_info & MASK(WIN0_ACT_WIDTH)) + 1;
		w0_act_y = ((act_info & MASK(WIN0_ACT_HEIGHT)) >> 16) + 1;
		w0_dsp_x = (dsp_info & MASK(WIN0_DSP_WIDTH)) + 1;
		w0_dsp_y = ((dsp_info & MASK(WIN0_DSP_HEIGHT)) >> 16) + 1;
		if (w0_state) {
			w0_st_x = dsp_st & MASK(WIN0_DSP_XST);
			w0_st_y = (dsp_st & MASK(WIN0_DSP_YST)) >> 16;
		}
		w0_y_h_fac = y_factor & MASK(WIN0_HS_FACTOR_YRGB);
		w0_y_v_fac = (y_factor & MASK(WIN0_VS_FACTOR_YRGB)) >> 16;
		w0_uv_h_fac = uv_factor & MASK(WIN0_HS_FACTOR_CBR);
		w0_uv_v_fac = (uv_factor & MASK(WIN0_VS_FACTOR_CBR)) >> 16;

		/* WIN1 */
		win_ctrl = vop_readl(vop_dev, WIN1_CTRL0);
		w1_state = win_ctrl & MASK(WIN1_EN);
		fmt_id = (win_ctrl & MASK(WIN1_DATA_FMT)) >> 1;
		vop_format_to_string(fmt_id, format_w1);
		vop_color_space_to_string(dev_drv->win[1]->colorspace,
					  color_space_w1);
		vir_info = vop_readl(vop_dev, WIN1_VIR);
		act_info = vop_readl(vop_dev, WIN1_ACT_INFO);
		dsp_info = vop_readl(vop_dev, WIN1_DSP_INFO);
		dsp_st = vop_readl(vop_dev, WIN1_DSP_ST);
		y_factor = vop_readl(vop_dev, WIN1_SCL_FACTOR_YRGB);
		uv_factor = vop_readl(vop_dev, WIN1_SCL_FACTOR_CBR);
		w1_vir_y = vir_info & MASK(WIN1_VIR_STRIDE);
		w1_vir_uv = (vir_info & MASK(WIN1_VIR_STRIDE_UV)) >> 16;
		w1_act_x = (act_info & MASK(WIN1_ACT_WIDTH)) + 1;
		w1_act_y = ((act_info & MASK(WIN1_ACT_HEIGHT)) >> 16) + 1;
		w1_dsp_x = (dsp_info & MASK(WIN1_DSP_WIDTH)) + 1;
		w1_dsp_y = ((dsp_info & MASK(WIN1_DSP_HEIGHT)) >> 16) + 1;
		if (w1_state) {
			w1_st_x = dsp_st & MASK(WIN1_DSP_XST);
			w1_st_y = (dsp_st & MASK(WIN1_DSP_YST)) >> 16;
		}
		w1_y_h_fac = y_factor & MASK(WIN1_HS_FACTOR_YRGB);
		w1_y_v_fac = (y_factor & MASK(WIN1_VS_FACTOR_YRGB)) >> 16;
		w1_uv_h_fac = uv_factor & MASK(WIN1_HS_FACTOR_CBR);
		w1_uv_v_fac = (uv_factor & MASK(WIN1_VS_FACTOR_CBR)) >> 16;

		/* WIN2 */
		win_ctrl = vop_readl(vop_dev, WIN2_CTRL0);
		w2_state = win_ctrl & MASK(WIN2_EN);
		fmt_id = (win_ctrl & MASK(WIN2_DATA_FMT)) >> 1;
		vop_format_to_string(fmt_id, format_w2);
		vop_color_space_to_string(dev_drv->win[2]->colorspace,
					  color_space_w2);
		vir_info = vop_readl(vop_dev, WIN2_VIR);
		act_info = vop_readl(vop_dev, WIN2_ACT_INFO);
		dsp_info = vop_readl(vop_dev, WIN2_DSP_INFO);
		dsp_st = vop_readl(vop_dev, WIN2_DSP_ST);
		y_factor = vop_readl(vop_dev, WIN2_SCL_FACTOR_YRGB);
		uv_factor = vop_readl(vop_dev, WIN2_SCL_FACTOR_CBR);
		w2_vir_y = vir_info & MASK(WIN2_VIR_STRIDE);
		w2_vir_uv = (vir_info & MASK(WIN2_VIR_STRIDE_UV)) >> 16;
		w2_act_x = (act_info & MASK(WIN2_ACT_WIDTH)) + 1;
		w2_act_y = ((act_info & MASK(WIN2_ACT_HEIGHT)) >> 16) + 1;
		w2_dsp_x = (dsp_info & MASK(WIN2_DSP_WIDTH)) + 1;
		w2_dsp_y = ((dsp_info & MASK(WIN2_DSP_HEIGHT)) >> 16) + 1;
		if (w2_state) {
			w2_st_x = dsp_st & MASK(WIN2_DSP_XST);
			w2_st_y = (dsp_st & MASK(WIN2_DSP_YST)) >> 16;
		}
		w2_y_h_fac = y_factor & MASK(WIN2_HS_FACTOR_YRGB);
		w2_y_v_fac = (y_factor & MASK(WIN2_VS_FACTOR_YRGB)) >> 16;
		w2_uv_h_fac = uv_factor & MASK(WIN2_HS_FACTOR_CBR);
		w2_uv_v_fac = (uv_factor & MASK(WIN2_VS_FACTOR_CBR)) >> 16;

	} else {
		spin_unlock(&vop_dev->reg_lock);
		return -EPERM;
	}
	spin_unlock(&vop_dev->reg_lock);
	size += snprintf(dsp_buf, 80,
		"z-order:\n  win[%d]\n  win[%d]\n  win[%d]\n",
		layer2_sel, layer1_sel, layer0_sel);
	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));
	size += snprintf(dsp_buf, 80,
		"overlay mode: %s\n", overlay_mode ? "YUV" : "RGB");
	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));
	/* win0 */
	size += snprintf(dsp_buf, 80,
		"win0:\n  state:%d, fmt:%7s_%dbit_%s_%s\n  y_vir:%4d, uv_vir:%4d,",
		w0_state, format_w0,
		vop_read_bit(vop_dev, WIN0_CTRL0, V_WIN0_FMT_10(0)) ? 10 : 8,
		dev_drv->win[0]->area[0].data_space ? "HDR" : "SDR",
		color_space_w0, w0_vir_y, w0_vir_uv);
	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));

	size += snprintf(dsp_buf, 80,
		 " x_act  :%5d, y_act  :%5d, dsp_x   :%5d, dsp_y   :%5d\n",
		 w0_act_x, w0_act_y, w0_dsp_x, w0_dsp_y);
	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));

	size += snprintf(dsp_buf, 80,
		 "  x_st :%4d, y_st  :%4d, y_h_fac:%5d, y_v_fac:%5d, ",
		 w0_st_x - h_pw_bp, w0_st_y - v_pw_bp, w0_y_h_fac, w0_y_v_fac);
	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));

	size += snprintf(dsp_buf, 80,
		 "uv_h_fac:%5d, uv_v_fac:%5d\n  y_addr:0x%08x,    uv_addr:0x%08x\n",
		 w0_uv_h_fac, w0_uv_v_fac, vop_readl(vop_dev, WIN0_YRGB_MST),
		 vop_readl(vop_dev, WIN0_CBR_MST));
	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));

	/* win1 */
	size += snprintf(dsp_buf, 80,
		"win1:\n  state:%d, fmt:%7s_%dbit_%s_%s\n  y_vir:%4d, uv_vir:%4d,",
		w1_state, format_w1,
		vop_read_bit(vop_dev, WIN1_CTRL0, V_WIN1_FMT_10(0)) ? 10 : 8,
		dev_drv->win[1]->area[0].data_space ? "HDR" : "SDR",
		color_space_w1, w1_vir_y, w1_vir_uv);
	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));

	size += snprintf(dsp_buf, 80,
		 " x_act  :%5d, y_act  :%5d, dsp_x   :%5d, dsp_y   :%5d\n",
		 w1_act_x, w1_act_y, w1_dsp_x, w1_dsp_y);
	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));

	size += snprintf(dsp_buf, 80,
		 "  x_st :%4d, y_st  :%4d, y_h_fac:%5d, y_v_fac:%5d, ",
		 w1_st_x - h_pw_bp, w1_st_y - v_pw_bp, w1_y_h_fac, w1_y_v_fac);
	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));

	size += snprintf(dsp_buf, 80,
		 "uv_h_fac:%5d, uv_v_fac:%5d\n  y_addr:0x%08x,    uv_addr:0x%08x\n",
		 w1_uv_h_fac, w1_uv_v_fac, vop_readl(vop_dev, WIN1_YRGB_MST),
		 vop_readl(vop_dev, WIN1_CBR_MST));
	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));

	/* win2 */
	size += snprintf(dsp_buf, 80,
		"win2:\n  state:%d, fmt:%7s_%dbit_%s_%s\n  y_vir:%4d, uv_vir:%4d,",
		w2_state, format_w2,
		vop_read_bit(vop_dev, WIN2_CTRL0, V_WIN2_FMT_10(0)) ? 10 : 8,
		dev_drv->win[2]->area[0].data_space ? "HDR" : "SDR",
		color_space_w2, w2_vir_y, w2_vir_uv);
	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));

	size += snprintf(dsp_buf, 80,
		 " x_act  :%5d, y_act  :%5d, dsp_x   :%5d, dsp_y   :%5d\n",
		 w2_act_x, w2_act_y, w2_dsp_x, w2_dsp_y);
	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));

	size += snprintf(dsp_buf, 80,
		 "  x_st :%4d, y_st  :%4d, y_h_fac:%5d, y_v_fac:%5d, ",
		 w2_st_x - h_pw_bp, w2_st_y - v_pw_bp, w2_y_h_fac, w2_y_v_fac);
	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));

	size += snprintf(dsp_buf, 80,
		 "uv_h_fac:%5d, uv_v_fac:%5d\n  y_addr:0x%08x,    uv_addr:0x%08x\n",
		 w2_uv_h_fac, w2_uv_v_fac, vop_readl(vop_dev, WIN2_YRGB_MST),
		 vop_readl(vop_dev, WIN2_CBR_MST));
	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));

	size += snprintf(dsp_buf, 80,
		"HDR:\n  pre sdr2hdr : %d\n  post_hdr2sdr: %d\n  post_sdr2hdr: %d\n",
		vop_dev->pre_sdr2hdr, vop_dev->post_hdr2sdr,
		vop_dev->post_sdr2hdr);

	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));
	size += snprintf(dsp_buf, 80,
		"BCSH:\n  y2r: %d\n  r2y: %d\n",
		vop_read_bit(vop_dev, BCSH_CTRL, V_BCSH_Y2R_EN(0)),
		vop_read_bit(vop_dev, BCSH_CTRL, V_BCSH_R2Y_EN(0)));
	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));

	size += snprintf(dsp_buf, 80,
		"vop output:\n  data space: %s\n  color mode: %s\n",
		screen->data_space ? "HDR" : "SDR",
		(screen->color_mode > COLOR_RGB_BT2020) ? "Ycbcr" : "RGB");

	strcat(buf, dsp_buf);
	memset(dsp_buf, 0, sizeof(dsp_buf));

	return size;
}

static int vop_fps_mgr(struct rk_lcdc_driver *dev_drv, int fps, bool set)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	struct rk_screen *screen = dev_drv->cur_screen;
	u64 ft = 0;
	u32 dotclk;
	int ret;
	u32 pixclock;
	u32 x_total, y_total;

	if (set) {
		if (fps == 0) {
			dev_info(dev_drv->dev, "unsupport set fps=0\n");
			return 0;
		}
		ft = div_u64(1000000000000llu, fps);
		x_total =
		    screen->mode.upper_margin + screen->mode.lower_margin +
		    screen->mode.yres + screen->mode.vsync_len;
		y_total =
		    screen->mode.left_margin + screen->mode.right_margin +
		    screen->mode.xres + screen->mode.hsync_len;
		dev_drv->pixclock = div_u64(ft, x_total * y_total);
		dotclk = div_u64(1000000000000llu, dev_drv->pixclock);
		ret = clk_set_rate(vop_dev->dclk, dotclk);
	}

	pixclock = div_u64(1000000000000llu, clk_get_rate(vop_dev->dclk));
	vop_dev->pixclock = pixclock;
	dev_drv->pixclock = vop_dev->pixclock;
	fps = rk_fb_calc_fps(screen, pixclock);
	screen->ft = 1000 / fps;	/*one frame time in ms */

	if (set)
		dev_info(dev_drv->dev, "%s:dclk:%lu,fps:%d\n", __func__,
			 clk_get_rate(vop_dev->dclk), fps);

	return fps;
}

static int vop_fb_win_remap(struct rk_lcdc_driver *dev_drv, u16 order)
{
	mutex_lock(&dev_drv->fb_win_id_mutex);
	if (order == FB_DEFAULT_ORDER)
		order = FB0_WIN0_FB1_WIN1_FB2_WIN2_FB3_WIN3_FB4_HWC;
	dev_drv->fb4_win_id = order / 10000;
	dev_drv->fb3_win_id = (order / 1000) % 10;
	dev_drv->fb2_win_id = (order / 100) % 10;
	dev_drv->fb1_win_id = (order / 10) % 10;
	dev_drv->fb0_win_id = order % 10;
	mutex_unlock(&dev_drv->fb_win_id_mutex);

	return 0;
}

static int vop_get_win_id(struct rk_lcdc_driver *dev_drv, const char *id)
{
	int win_id = 0;

	mutex_lock(&dev_drv->fb_win_id_mutex);
	if (!strcmp(id, "fb0") || !strcmp(id, "fb5"))
		win_id = dev_drv->fb0_win_id;
	else if (!strcmp(id, "fb1") || !strcmp(id, "fb6"))
		win_id = dev_drv->fb1_win_id;
	else if (!strcmp(id, "fb2") || !strcmp(id, "fb7"))
		win_id = dev_drv->fb2_win_id;
	else if (!strcmp(id, "fb3") || !strcmp(id, "fb8"))
		win_id = dev_drv->fb3_win_id;
	else if (!strcmp(id, "fb4") || !strcmp(id, "fb9"))
		win_id = dev_drv->fb4_win_id;
	mutex_unlock(&dev_drv->fb_win_id_mutex);

	return win_id;
}

static int vop_config_done(struct rk_lcdc_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	int i;
	u64 val;
	struct rk_lcdc_win *win = NULL;

	spin_lock(&vop_dev->reg_lock);
	vop_post_cfg(dev_drv);

	vop_msk_reg(vop_dev, SYS_CTRL, V_VOP_STANDBY_EN(vop_dev->standby));
	for (i = 0; i < dev_drv->lcdc_win_num; i++) {
		win = dev_drv->win[i];
		vop_alpha_cfg(dev_drv, i);
		if ((win->state == 0) && (win->last_state == 1)) {
			switch (win->id) {
			case 0:
				val = V_WIN0_EN(0);
				vop_msk_reg(vop_dev, WIN0_CTRL0, val);
				break;
			case 1:
				val = V_WIN1_EN(0);
				vop_msk_reg(vop_dev, WIN1_CTRL0, val);
				break;
			case 2:
				val = V_WIN2_EN(0);
				vop_msk_reg(vop_dev, WIN2_CTRL0, val);
				break;
			case 3:
				val = V_HWC_EN(0);
				vop_msk_reg(vop_dev, HWC_CTRL0, val);
				break;
			default:
				break;
			}
		} else if (win->state == 1) {
			vop_layer_update_regs(vop_dev, win);
		}
		win->last_state = win->state;
	}
	vop_cfg_done(vop_dev);
	spin_unlock(&vop_dev->reg_lock);
	return 0;
}

static int vop_set_irq_to_cpu(struct rk_lcdc_driver *dev_drv, int enable)
{
	struct vop_device *vop_dev =
			container_of(dev_drv, struct vop_device, driver);
	if (enable)
		enable_irq(vop_dev->irq);
	else
		disable_irq(vop_dev->irq);
	return 0;
}

int vop_poll_vblank(struct rk_lcdc_driver *dev_drv)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	u32 int_reg;
	int ret;

	if (vop_dev->clk_on && (!dev_drv->suspend_flag)) {
		int_reg = vop_readl(vop_dev, INTR_STATUS0);
		if (int_reg & INTR_LINE_FLAG0) {
			vop_dev->driver.frame_time.last_framedone_t =
			    vop_dev->driver.frame_time.framedone_t;
			vop_dev->driver.frame_time.framedone_t = cpu_clock(0);
			vop_mask_writel(vop_dev, INTR_CLEAR0, INTR_LINE_FLAG0,
					INTR_LINE_FLAG0);
			ret = RK_LF_STATUS_FC;
		} else {
			ret = RK_LF_STATUS_FR;
		}
	} else {
		ret = RK_LF_STATUS_NC;
	}

	return ret;
}

static int vop_get_dsp_addr(struct rk_lcdc_driver *dev_drv,
			    unsigned int dsp_addr[][4])
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	spin_lock(&vop_dev->reg_lock);
	if (vop_dev->clk_on) {
		dsp_addr[0][0] = vop_readl(vop_dev, WIN0_YRGB_MST);
		dsp_addr[1][0] = vop_readl(vop_dev, WIN1_YRGB_MST);
		dsp_addr[2][0] = vop_readl(vop_dev, WIN2_YRGB_MST);
		dsp_addr[3][0] = vop_readl(vop_dev, HWC_MST);
	}
	spin_unlock(&vop_dev->reg_lock);
	return 0;
}

int vop_update_pwm(int bl_pwm_period, int bl_pwm_duty)
{
	/*
	 * TODO:
	 * pwm_period_hpr = bl_pwm_period;
	 * pwm_duty_lpr = bl_pwm_duty;
	 * pr_info("bl_pwm_period_hpr = 0x%x, bl_pwm_duty_lpr = 0x%x\n",
	 * bl_pwm_period, bl_pwm_duty);
	 */

	return 0;
}

/*
 *  a:[-30~0]:
 *    sin_hue = sin(a)*256 +0x100;
 *    cos_hue = cos(a)*256;
 *  a:(0~30]
 *    sin_hue = sin(a)*256;
 *    cos_hue = cos(a)*256;
 */
static int vop_get_bcsh_hue(struct rk_lcdc_driver *dev_drv, bcsh_hue_mode mode)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	u32 val = 0;

	spin_lock(&vop_dev->reg_lock);
	if (vop_dev->clk_on) {
		val = vop_readl(vop_dev, BCSH_H);
		switch (mode) {
		case H_SIN:
			val &= MASK(SIN_HUE);
			break;
		case H_COS:
			val &= MASK(COS_HUE);
			val >>= 16;
			break;
		default:
			break;
		}
	}
	spin_unlock(&vop_dev->reg_lock);

	return val;
	return 0;
}

static int vop_set_bcsh_hue(struct rk_lcdc_driver *dev_drv,
			    int sin_hue, int cos_hue)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	u64 val;

	spin_lock(&vop_dev->reg_lock);
	if (vop_dev->clk_on) {
		val = V_SIN_HUE(sin_hue) | V_COS_HUE(cos_hue);
		vop_msk_reg(vop_dev, BCSH_H, val);
		vop_cfg_done(vop_dev);
	}
	spin_unlock(&vop_dev->reg_lock);

	return 0;
}

static int vop_set_bcsh_bcs(struct rk_lcdc_driver *dev_drv,
			    bcsh_bcs_mode mode, int value)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	u64 val = 0;

	spin_lock(&vop_dev->reg_lock);
	if (vop_dev->clk_on) {
		switch (mode) {
		case BRIGHTNESS:
			/*from 0 to 255,typical is 128 */
			if (value < 0x80)
				value += 0x80;
			else if (value >= 0x80)
				value = value - 0x80;
			val = V_BRIGHTNESS(value);
			break;
		case CONTRAST:
			/*from 0 to 510,typical is 256 */
			val = V_CONTRAST(value);
			break;
		case SAT_CON:
			/*from 0 to 1015,typical is 256 */
			val = V_SAT_CON(value);
			break;
		default:
			break;
		}
		vop_msk_reg(vop_dev, BCSH_BCS, val);
		vop_cfg_done(vop_dev);
	}
	spin_unlock(&vop_dev->reg_lock);

	return val;
}

static int vop_get_bcsh_bcs(struct rk_lcdc_driver *dev_drv, bcsh_bcs_mode mode)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);
	u64 val = 0;

	spin_lock(&vop_dev->reg_lock);
	if (vop_dev->clk_on) {
		val = vop_readl(vop_dev, BCSH_BCS);
		switch (mode) {
		case BRIGHTNESS:
			val &= MASK(BRIGHTNESS);
			if (val > 0x80)
				val -= 0x80;
			else
				val += 0x80;
			break;
		case CONTRAST:
			val &= MASK(CONTRAST);
			val >>= 8;
			break;
		case SAT_CON:
			val &= MASK(SAT_CON);
			val >>= 20;
			break;
		default:
			break;
		}
	}
	spin_unlock(&vop_dev->reg_lock);
	return val;
}

static int vop_open_bcsh(struct rk_lcdc_driver *dev_drv, bool open)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);

	spin_lock(&vop_dev->reg_lock);
	if (vop_dev->clk_on) {
		if (open) {
			vop_writel(vop_dev, BCSH_COLOR_BAR, 0x1);
			vop_writel(vop_dev, BCSH_BCS, 0xd0010000);
			vop_writel(vop_dev, BCSH_H, 0x01000000);
			dev_drv->bcsh.enable = 1;
		} else {
			vop_msk_reg(vop_dev, BCSH_COLOR_BAR, V_BCSH_EN(0));
			dev_drv->bcsh.enable = 0;
		}
		rk322xh_vop_bcsh_path_sel(dev_drv);
		vop_cfg_done(vop_dev);
	}
	spin_unlock(&vop_dev->reg_lock);

	return 0;
}

static int vop_set_bcsh(struct rk_lcdc_driver *dev_drv, bool enable)
{
	if (!enable || !dev_drv->bcsh.enable) {
		vop_open_bcsh(dev_drv, false);
		return 0;
	}

	if (dev_drv->bcsh.brightness <= 255 ||
	    dev_drv->bcsh.contrast <= 510 ||
	    dev_drv->bcsh.sat_con <= 1015 ||
	    (dev_drv->bcsh.sin_hue <= 511 && dev_drv->bcsh.cos_hue <= 511)) {
		vop_open_bcsh(dev_drv, true);
		if (dev_drv->bcsh.brightness <= 255)
			vop_set_bcsh_bcs(dev_drv, BRIGHTNESS,
					 dev_drv->bcsh.brightness);
		if (dev_drv->bcsh.contrast <= 510)
			vop_set_bcsh_bcs(dev_drv, CONTRAST,
					 dev_drv->bcsh.contrast);
		if (dev_drv->bcsh.sat_con <= 1015)
			vop_set_bcsh_bcs(dev_drv, SAT_CON,
					 dev_drv->bcsh.sat_con);
		if (dev_drv->bcsh.sin_hue <= 511 &&
		    dev_drv->bcsh.cos_hue <= 511)
			vop_set_bcsh_hue(dev_drv, dev_drv->bcsh.sin_hue,
					 dev_drv->bcsh.cos_hue);
	}

	return 0;
}

static int __maybe_unused
vop_dsp_black(struct rk_lcdc_driver *dev_drv, int enable)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);

	if (enable) {
		spin_lock(&vop_dev->reg_lock);
		if (likely(vop_dev->clk_on)) {
			vop_msk_reg(vop_dev, DSP_CTRL0, V_DSP_BLACK_EN(1));
			vop_cfg_done(vop_dev);
		}
		spin_unlock(&vop_dev->reg_lock);
	} else {
		spin_lock(&vop_dev->reg_lock);
		if (likely(vop_dev->clk_on)) {
			vop_msk_reg(vop_dev, DSP_CTRL0, V_DSP_BLACK_EN(0));

			vop_cfg_done(vop_dev);
		}
		spin_unlock(&vop_dev->reg_lock);
	}

	return 0;
}

static int vop_set_overscan(struct rk_lcdc_driver *dev_drv,
			    struct overscan *overscan)
{
	struct vop_device *vop_dev =
	    container_of(dev_drv, struct vop_device, driver);

	if (unlikely(!vop_dev->clk_on)) {
		pr_info("%s,clk_on = %d\n", __func__, vop_dev->clk_on);
		return 0;
	}
	/*vop_post_cfg(dev_drv);*/

	return 0;
}

static struct rk_lcdc_drv_ops lcdc_drv_ops = {
	.open = vop_open,
	.win_direct_en = vop_win_direct_en,
	.load_screen = vop_load_screen,
	.get_dspbuf_info = vop_get_dspbuf_info,
	.post_dspbuf = vop_post_dspbuf,
	.set_par = vop_set_par,
	.pan_display = vop_pan_display,
	.direct_set_addr = vop_direct_set_win_addr,
	/*.lcdc_reg_update = vop_reg_update,*/
	.blank = vop_blank,
	.ioctl = vop_ioctl,
	.suspend = vop_early_suspend,
	.resume = vop_early_resume,
	.get_win_state = vop_get_win_state,
	.area_support_num = vop_get_area_num,
	.ovl_mgr = vop_ovl_mgr,
	.get_disp_info = vop_get_disp_info,
	.fps_mgr = vop_fps_mgr,
	.fb_get_win_id = vop_get_win_id,
	.fb_win_remap = vop_fb_win_remap,
	.poll_vblank = vop_poll_vblank,
	.get_dsp_addr = vop_get_dsp_addr,
	.set_dsp_bcsh_hue = vop_set_bcsh_hue,
	.set_dsp_bcsh_bcs = vop_set_bcsh_bcs,
	.get_dsp_bcsh_hue = vop_get_bcsh_hue,
	.get_dsp_bcsh_bcs = vop_get_bcsh_bcs,
	.open_bcsh = vop_open_bcsh,
	.dump_reg = vop_reg_dump,
	.cfg_done = vop_config_done,
	.set_irq_to_cpu = vop_set_irq_to_cpu,
	/*.dsp_black = vop_dsp_black,*/
	.mmu_en    = vop_mmu_en,
	.set_overscan   = vop_set_overscan,
};

static irqreturn_t vop_isr(int irq, void *dev_id)
{
	struct vop_device *vop_dev = (struct vop_device *)dev_id;
	ktime_t timestamp = ktime_get();
	u32 intr_status;
	unsigned long flags;

	spin_lock_irqsave(&vop_dev->irq_lock, flags);

	intr_status = vop_readl(vop_dev, INTR_STATUS0);
	vop_mask_writel(vop_dev, INTR_CLEAR0, INTR_MASK, intr_status);

	spin_unlock_irqrestore(&vop_dev->irq_lock, flags);
	/* This is expected for vop iommu irqs, since the irq is shared */
	if (!intr_status)
		return IRQ_NONE;

	if (intr_status & INTR_FS) {
		timestamp = ktime_get();
		vop_dev->driver.vsync_info.timestamp = timestamp;
		wake_up_interruptible_all(&vop_dev->driver.vsync_info.wait);
		intr_status &= ~INTR_FS;
	}

	if (intr_status & INTR_LINE_FLAG0)
		intr_status &= ~INTR_LINE_FLAG0;

	if (intr_status & INTR_LINE_FLAG1)
		intr_status &= ~INTR_LINE_FLAG1;

	if (intr_status & INTR_FS_NEW)
		intr_status &= ~INTR_FS_NEW;

	if (intr_status & INTR_BUS_ERROR) {
		intr_status &= ~INTR_BUS_ERROR;
		dev_warn_ratelimited(vop_dev->dev, "bus error!");
	}

	if (intr_status & INTR_WIN0_EMPTY) {
		intr_status &= ~INTR_WIN0_EMPTY;
		dev_warn_ratelimited(vop_dev->dev, "intr win0 empty!");
	}

	if (intr_status & INTR_WIN1_EMPTY) {
		intr_status &= ~INTR_WIN1_EMPTY;
		dev_warn_ratelimited(vop_dev->dev, "intr win1 empty!");
	}

	if (intr_status & INTR_HWC_EMPTY) {
		intr_status &= ~INTR_HWC_EMPTY;
		dev_warn_ratelimited(vop_dev->dev, "intr hwc empty!");
	}

	if (intr_status & INTR_POST_BUF_EMPTY) {
		intr_status &= ~INTR_POST_BUF_EMPTY;
		dev_warn_ratelimited(vop_dev->dev, "intr post buf empty!");
	}

	if (intr_status)
		dev_err(vop_dev->dev, "Unknown VOP IRQs: %#02x\n", intr_status);

	return IRQ_HANDLED;
}

#if defined(CONFIG_PM)
static int vop_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int vop_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define vop_suspend NULL
#define vop_resume  NULL
#endif

static int vop_parse_dt(struct vop_device *vop_dev)
{
	struct device_node *np = vop_dev->dev->of_node;
	struct rk_lcdc_driver *dev_drv = &vop_dev->driver;
	int val;

	if (of_property_read_u32(np, "rockchip,prop", &val))
		vop_dev->prop = PRMRY;	/*default set it as primary */
	else
		vop_dev->prop = val;

	if (of_property_read_u32(np, "rockchip,mirror", &val))
		dev_drv->rotate_mode = NO_MIRROR;
	else
		dev_drv->rotate_mode = val;

	if (of_property_read_u32(np, "rockchip,pwr18", &val))
		/*default set it as 3.xv power supply */
		vop_dev->pwr18 = false;
	else
		vop_dev->pwr18 = (val ? true : false);

	if (of_property_read_u32(np, "rockchip,fb-win-map", &val))
		dev_drv->fb_win_map = FB_DEFAULT_ORDER;
	else
		dev_drv->fb_win_map = val;

	if (of_property_read_u32(np, "rockchip,bcsh-en", &val))
		dev_drv->bcsh.enable = false;
	else
		dev_drv->bcsh.enable = (val ? true : false);

	if (of_property_read_u32(np, "rockchip,brightness", &val))
		dev_drv->bcsh.brightness = 0xffff;
	else
		dev_drv->bcsh.brightness = val;

	if (of_property_read_u32(np, "rockchip,contrast", &val))
		dev_drv->bcsh.contrast = 0xffff;
	else
		dev_drv->bcsh.contrast = val;

	if (of_property_read_u32(np, "rockchip,sat-con", &val))
		dev_drv->bcsh.sat_con = 0xffff;
	else
		dev_drv->bcsh.sat_con = val;

	if (of_property_read_u32(np, "rockchip,hue", &val)) {
		dev_drv->bcsh.sin_hue = 0xffff;
		dev_drv->bcsh.cos_hue = 0xffff;
	} else {
		dev_drv->bcsh.sin_hue = val & 0xff;
		dev_drv->bcsh.cos_hue = (val >> 8) & 0xff;
	}

	if (of_property_read_u32(np, "rockchip,iommu-enabled", &val))
		dev_drv->iommu_enabled = 0;
	else
		dev_drv->iommu_enabled = val;
	return 0;
}

static int vop_probe(struct platform_device *pdev)
{
	struct vop_device *vop_dev = NULL;
	struct rk_lcdc_driver *dev_drv;
	const struct of_device_id *of_id;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;
	int prop;
	int ret = 0;

	/* if the primary lcdc has not registered ,the extend
	 * lcdc register later
	 */
	of_property_read_u32(np, "rockchip,prop", &prop);
	if (prop == EXTEND) {
		if (!is_prmry_rk_lcdc_registered())
			return -EPROBE_DEFER;
	}
	vop_dev = devm_kzalloc(dev, sizeof(struct vop_device), GFP_KERNEL);
	if (!vop_dev)
		return -ENOMEM;
	of_id = of_match_device(vop_dt_ids, dev);
	vop_dev->data = of_id->data;
	if (VOP_CHIP(vop_dev) != VOP_RK322XH) {
		dev_info(dev, "unsupport chip: %d\n", VOP_CHIP(vop_dev));
		return -ENODEV;
	}
	platform_set_drvdata(pdev, vop_dev);
	vop_dev->dev = dev;
	vop_parse_dt(vop_dev);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	vop_dev->reg_phy_base = res->start;
	vop_dev->len = resource_size(res);
	vop_dev->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(vop_dev->regs))
		return PTR_ERR(vop_dev->regs);
	dev_info(dev, "vop_dev->regs=0x%lx\n", (long)vop_dev->regs);

	vop_dev->regsbak = devm_kzalloc(dev, vop_dev->len, GFP_KERNEL);
	if (IS_ERR(vop_dev->regsbak))
		return PTR_ERR(vop_dev->regsbak);

	vop_dev->id = 0;
	dev_set_name(vop_dev->dev, "vop%d", vop_dev->id);
	dev_drv = &vop_dev->driver;
	dev_drv->dev = dev;
	dev_drv->prop = prop;
	dev_drv->id = vop_dev->id;
	dev_drv->ops = &lcdc_drv_ops;
	dev_drv->lcdc_win_num = vop_dev->data->n_wins;
	dev_drv->reserved_fb = 0;
	spin_lock_init(&vop_dev->reg_lock);
	spin_lock_init(&vop_dev->irq_lock);

	vop_dev->irq = platform_get_irq(pdev, 0);
	if (vop_dev->irq < 0) {
		dev_err(&pdev->dev, "cannot find IRQ for lcdc%d\n",
			vop_dev->id);
		return -ENXIO;
	}

	ret = devm_request_irq(dev, vop_dev->irq, vop_isr,
			       IRQF_DISABLED | IRQF_SHARED,
			       dev_name(dev), vop_dev);
	if (ret) {
		dev_err(&pdev->dev, "cannot requeset irq %d - err %d\n",
			vop_dev->irq, ret);
		return ret;
	}

	if (dev_drv->iommu_enabled)
		strcpy(dev_drv->mmu_dts_name, VOPB_IOMMU_COMPATIBLE_NAME);

	ret = rk_fb_register(dev_drv, vop_dev->data->win, vop_dev->id);
	if (ret < 0) {
		dev_err(dev, "register fb for failed!\n");
		return ret;
	}
	vop_dev->screen = dev_drv->screen0;
	dev_info(dev, "lcdc%d probe ok, iommu %s\n",
		 vop_dev->id, dev_drv->iommu_enabled ? "enabled" : "disabled");

	return 0;
}

static int vop_remove(struct platform_device *pdev)
{
	return 0;
}

static void vop_shutdown(struct platform_device *pdev)
{
	struct vop_device *vop_dev = platform_get_drvdata(pdev);
	struct rk_lcdc_driver *dev_drv = &vop_dev->driver;

	dev_drv->suspend_flag = 1;
	smp_wmb();
	flush_kthread_worker(&dev_drv->update_regs_worker);
	kthread_stop(dev_drv->update_regs_thread);
	vop_deint(vop_dev);
	/*
	 * if (dev_drv->trsm_ops && dev_drv->trsm_ops->disable)
	 * dev_drv->trsm_ops->disable();
	 */

	vop_clk_disable(vop_dev);
	rk_disp_pwr_disable(dev_drv);
}

static struct platform_driver vop_driver = {
	.probe = vop_probe,
	.remove = vop_remove,
	.driver = {
		   .name = "rk322xh-lcdc",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(vop_dt_ids),
		   },
	.suspend = vop_suspend,
	.resume = vop_resume,
	.shutdown = vop_shutdown,
};

static int __init vop_module_init(void)
{
	return platform_driver_register(&vop_driver);
}

static void __exit vop_module_exit(void)
{
	platform_driver_unregister(&vop_driver);
}

fs_initcall(vop_module_init);
module_exit(vop_module_exit);
