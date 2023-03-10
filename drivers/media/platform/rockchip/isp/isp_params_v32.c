// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2021 Rockchip Electronics Co., Ltd. */

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>	/* for ISP params */
#include "dev.h"
#include "regs.h"
#include "isp_params_v32.h"

#define ISP32_MODULE_EN			BIT(0)
#define ISP32_SELF_FORCE_UPD		BIT(31)
#define ISP32_REG_WR_MASK		BIT(31) //disable write protect

#define ISP32_NOBIG_OVERFLOW_SIZE	(1536 * 896)
#define ISP32_AUTO_BIGMODE_WIDTH	1536
#define ISP32_VIR2_NOBIG_OVERFLOW_SIZE	(960 * 540)
#define ISP32_VIR2_AUTO_BIGMODE_WIDTH	960
#define ISP32_VIR4_NOBIG_OVERFLOW_SIZE	(640 * 400)
#define ISP32_VIR4_AUTO_BIGMODE_WIDTH	640

#define ISP32_VIR2_MAX_WIDTH		1920
#define ISP32_VIR2_MAX_SIZE		(1920 * 1080)
#define ISP32_VIR4_MAX_WIDTH		1280
#define ISP32_VIR4_MAX_SIZE		(1280 * 800)

static inline void
isp3_param_write_direct(struct rkisp_isp_params_vdev *params_vdev,
			u32 value, u32 addr)
{
	rkisp_write(params_vdev->dev, addr, value, true);
}

static inline void
isp3_param_write(struct rkisp_isp_params_vdev *params_vdev,
		 u32 value, u32 addr)
{
	rkisp_write(params_vdev->dev, addr, value, false);
}

static inline u32
isp3_param_read_direct(struct rkisp_isp_params_vdev *params_vdev, u32 addr)
{
	return rkisp_read(params_vdev->dev, addr, true);
}

static inline u32
isp3_param_read(struct rkisp_isp_params_vdev *params_vdev, u32 addr)
{
	return rkisp_read(params_vdev->dev, addr, false);
}

static inline u32
isp3_param_read_cache(struct rkisp_isp_params_vdev *params_vdev, u32 addr)
{
	return rkisp_read_reg_cache(params_vdev->dev, addr);
}

static inline void
isp3_param_set_bits(struct rkisp_isp_params_vdev *params_vdev,
		    u32 reg, u32 bit_mask)
{
	rkisp_set_bits(params_vdev->dev, reg, 0, bit_mask, false);
}

static inline void
isp3_param_clear_bits(struct rkisp_isp_params_vdev *params_vdev,
		      u32 reg, u32 bit_mask)
{
	rkisp_clear_bits(params_vdev->dev, reg, bit_mask, false);
}

static void
isp_dpcc_config(struct rkisp_isp_params_vdev *params_vdev,
		const struct isp2x_dpcc_cfg *arg)
{
	u32 value;
	int i;

	value = isp3_param_read(params_vdev, ISP3X_DPCC0_MODE);
	value &= ISP_DPCC_EN;

	value |= !!arg->stage1_enable << 2 |
		 !!arg->grayscale_mode << 1;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_MODE);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_MODE);

	value = (arg->sw_rk_out_sel & 0x03) << 5 |
		!!arg->sw_dpcc_output_sel << 4 |
		!!arg->stage1_rb_3x3 << 3 |
		!!arg->stage1_g_3x3 << 2 |
		!!arg->stage1_incl_rb_center << 1 |
		!!arg->stage1_incl_green_center;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_OUTPUT_MODE);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_OUTPUT_MODE);

	value = !!arg->stage1_use_fix_set << 3 |
		!!arg->stage1_use_set_3 << 2 |
		!!arg->stage1_use_set_2 << 1 |
		!!arg->stage1_use_set_1;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_SET_USE);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_SET_USE);

	value = !!arg->sw_rk_red_blue1_en << 13 |
		!!arg->rg_red_blue1_enable << 12 |
		!!arg->rnd_red_blue1_enable << 11 |
		!!arg->ro_red_blue1_enable << 10 |
		!!arg->lc_red_blue1_enable << 9 |
		!!arg->pg_red_blue1_enable << 8 |
		!!arg->sw_rk_green1_en << 5 |
		!!arg->rg_green1_enable << 4 |
		!!arg->rnd_green1_enable << 3 |
		!!arg->ro_green1_enable << 2 |
		!!arg->lc_green1_enable << 1 |
		!!arg->pg_green1_enable;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_METHODS_SET_1);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_METHODS_SET_1);

	value = !!arg->sw_rk_red_blue2_en << 13 |
		!!arg->rg_red_blue2_enable << 12 |
		!!arg->rnd_red_blue2_enable << 11 |
		!!arg->ro_red_blue2_enable << 10 |
		!!arg->lc_red_blue2_enable << 9 |
		!!arg->pg_red_blue2_enable << 8 |
		!!arg->sw_rk_green2_en << 5 |
		!!arg->rg_green2_enable << 4 |
		!!arg->rnd_green2_enable << 3 |
		!!arg->ro_green2_enable << 2 |
		!!arg->lc_green2_enable << 1 |
		!!arg->pg_green2_enable;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_METHODS_SET_2);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_METHODS_SET_2);

	value = !!arg->sw_rk_red_blue3_en << 13 |
		!!arg->rg_red_blue3_enable << 12 |
		!!arg->rnd_red_blue3_enable << 11 |
		!!arg->ro_red_blue3_enable << 10 |
		!!arg->lc_red_blue3_enable << 9 |
		!!arg->pg_red_blue3_enable << 8 |
		!!arg->sw_rk_green3_en << 5 |
		!!arg->rg_green3_enable << 4 |
		!!arg->rnd_green3_enable << 3 |
		!!arg->ro_green3_enable << 2 |
		!!arg->lc_green3_enable << 1 |
		!!arg->pg_green3_enable;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_METHODS_SET_3);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_METHODS_SET_3);

	value = ISP_PACK_4BYTE(arg->line_thr_1_g, arg->line_thr_1_rb,
				arg->sw_mindis1_g, arg->sw_mindis1_rb);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_LINE_THRESH_1);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_LINE_THRESH_1);

	value = ISP_PACK_4BYTE(arg->line_mad_fac_1_g, arg->line_mad_fac_1_rb,
				arg->sw_dis_scale_max1, arg->sw_dis_scale_min1);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_LINE_MAD_FAC_1);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_LINE_MAD_FAC_1);

	value = ISP_PACK_4BYTE(arg->pg_fac_1_g, arg->pg_fac_1_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_PG_FAC_1);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_PG_FAC_1);

	value = ISP_PACK_4BYTE(arg->rnd_thr_1_g, arg->rnd_thr_1_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_RND_THRESH_1);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_RND_THRESH_1);

	value = ISP_PACK_4BYTE(arg->rg_fac_1_g, arg->rg_fac_1_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_RG_FAC_1);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_RG_FAC_1);

	value = ISP_PACK_4BYTE(arg->line_thr_2_g, arg->line_thr_2_rb,
				arg->sw_mindis2_g, arg->sw_mindis2_rb);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_LINE_THRESH_2);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_LINE_THRESH_2);

	value = ISP_PACK_4BYTE(arg->line_mad_fac_2_g, arg->line_mad_fac_2_rb,
				arg->sw_dis_scale_max2, arg->sw_dis_scale_min2);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_LINE_MAD_FAC_2);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_LINE_MAD_FAC_2);

	value = ISP_PACK_4BYTE(arg->pg_fac_2_g, arg->pg_fac_2_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_PG_FAC_2);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_PG_FAC_2);

	value = ISP_PACK_4BYTE(arg->rnd_thr_2_g, arg->rnd_thr_2_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_RND_THRESH_2);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_RND_THRESH_2);

	value = ISP_PACK_4BYTE(arg->rg_fac_2_g, arg->rg_fac_2_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_RG_FAC_2);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_RG_FAC_2);

	value = ISP_PACK_4BYTE(arg->line_thr_3_g, arg->line_thr_3_rb,
				 arg->sw_mindis3_g, arg->sw_mindis3_rb);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_LINE_THRESH_3);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_LINE_THRESH_3);

	value = ISP_PACK_4BYTE(arg->line_mad_fac_3_g, arg->line_mad_fac_3_rb,
				arg->sw_dis_scale_max3, arg->sw_dis_scale_min3);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_LINE_MAD_FAC_3);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_LINE_MAD_FAC_3);

	value = ISP_PACK_4BYTE(arg->pg_fac_3_g, arg->pg_fac_3_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_PG_FAC_3);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_PG_FAC_3);

	value = ISP_PACK_4BYTE(arg->rnd_thr_3_g, arg->rnd_thr_3_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_RND_THRESH_3);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_RND_THRESH_3);

	value = ISP_PACK_4BYTE(arg->rg_fac_3_g, arg->rg_fac_3_rb, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_RG_FAC_3);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_RG_FAC_3);

	value = (arg->ro_lim_3_rb & 0x03) << 10 |
		(arg->ro_lim_3_g & 0x03) << 8 |
		(arg->ro_lim_2_rb & 0x03) << 6 |
		(arg->ro_lim_2_g & 0x03) << 4 |
		(arg->ro_lim_1_rb & 0x03) << 2 |
		(arg->ro_lim_1_g & 0x03);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_RO_LIMITS);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_RO_LIMITS);

	value = (arg->rnd_offs_3_rb & 0x03) << 10 |
		(arg->rnd_offs_3_g & 0x03) << 8 |
		(arg->rnd_offs_2_rb & 0x03) << 6 |
		(arg->rnd_offs_2_g & 0x03) << 4 |
		(arg->rnd_offs_1_rb & 0x03) << 2 |
		(arg->rnd_offs_1_g & 0x03);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_RND_OFFS);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_RND_OFFS);

	value = !!arg->bpt_rb_3x3 << 11 |
		!!arg->bpt_g_3x3 << 10 |
		!!arg->bpt_incl_rb_center << 9 |
		!!arg->bpt_incl_green_center << 8 |
		!!arg->bpt_use_fix_set << 7 |
		!!arg->bpt_use_set_3 << 6 |
		!!arg->bpt_use_set_2 << 5 |
		!!arg->bpt_use_set_1 << 4 |
		!!arg->bpt_cor_en << 1 |
		!!arg->bpt_det_en;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_BPT_CTRL);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_BPT_CTRL);

	isp3_param_write(params_vdev, arg->bp_number, ISP3X_DPCC0_BPT_NUMBER);
	isp3_param_write(params_vdev, arg->bp_number, ISP3X_DPCC1_BPT_NUMBER);
	isp3_param_write(params_vdev, arg->bp_table_addr, ISP3X_DPCC0_BPT_ADDR);
	isp3_param_write(params_vdev, arg->bp_table_addr, ISP3X_DPCC1_BPT_ADDR);

	value = ISP_PACK_2SHORT(arg->bpt_h_addr, arg->bpt_v_addr);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_BPT_DATA);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_BPT_DATA);

	isp3_param_write(params_vdev, arg->bp_cnt, ISP3X_DPCC0_BP_CNT);
	isp3_param_write(params_vdev, arg->bp_cnt, ISP3X_DPCC1_BP_CNT);

	isp3_param_write(params_vdev, arg->sw_pdaf_en, ISP3X_DPCC0_PDAF_EN);
	isp3_param_write(params_vdev, arg->sw_pdaf_en, ISP3X_DPCC1_PDAF_EN);

	value = 0;
	for (i = 0; i < ISP32_DPCC_PDAF_POINT_NUM; i++)
		value |= !!arg->pdaf_point_en[i] << i;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_PDAF_POINT_EN);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_PDAF_POINT_EN);

	value = ISP_PACK_2SHORT(arg->pdaf_offsetx, arg->pdaf_offsety);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_PDAF_OFFSET);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_PDAF_OFFSET);

	value = ISP_PACK_2SHORT(arg->pdaf_wrapx, arg->pdaf_wrapy);
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_PDAF_WRAP);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_PDAF_WRAP);

	value = ISP_PACK_2SHORT(arg->pdaf_wrapx_num, arg->pdaf_wrapy_num);
	isp3_param_write(params_vdev, value, ISP_DPCC0_PDAF_SCOPE);
	isp3_param_write(params_vdev, value, ISP_DPCC1_PDAF_SCOPE);

	for (i = 0; i < ISP32_DPCC_PDAF_POINT_NUM / 2; i++) {
		value = ISP_PACK_4BYTE(arg->point[2 * i].x, arg->point[2 * i].y,
					arg->point[2 * i + 1].x, arg->point[2 * i + 1].y);
		isp3_param_write(params_vdev, value, ISP3X_DPCC0_PDAF_POINT_0 + 4 * i);
		isp3_param_write(params_vdev, value, ISP3X_DPCC1_PDAF_POINT_0 + 4 * i);
	}

	isp3_param_write(params_vdev, arg->pdaf_forward_med, ISP3X_DPCC0_PDAF_FORWARD_MED);
	isp3_param_write(params_vdev, arg->pdaf_forward_med, ISP3X_DPCC1_PDAF_FORWARD_MED);
}

static void
isp_dpcc_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 value;

	value = isp3_param_read(params_vdev, ISP3X_DPCC0_MODE);
	value &= ~ISP_DPCC_EN;

	if (en)
		value |= ISP_DPCC_EN;
	isp3_param_write(params_vdev, value, ISP3X_DPCC0_MODE);
	isp3_param_write(params_vdev, value, ISP3X_DPCC1_MODE);
}

static void
isp_bls_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp32_bls_cfg *arg)
{
	const struct isp2x_bls_fixed_val *pval;
	u32 new_control, value;

	new_control = isp3_param_read(params_vdev, ISP3X_BLS_CTRL);
	new_control &= (ISP_BLS_ENA | ISP32_BLS_BLS2_EN);

	pval = &arg->bls1_val;
	if (arg->bls1_en) {
		new_control |= ISP_BLS_BLS1_EN;

		switch (params_vdev->raw_type) {
		case RAW_BGGR:
			isp3_param_write(params_vdev, pval->r, ISP3X_BLS1_D_FIXED);
			isp3_param_write(params_vdev, pval->gr, ISP3X_BLS1_C_FIXED);
			isp3_param_write(params_vdev, pval->gb, ISP3X_BLS1_B_FIXED);
			isp3_param_write(params_vdev, pval->b, ISP3X_BLS1_A_FIXED);
			break;
		case RAW_GBRG:
			isp3_param_write(params_vdev, pval->r, ISP3X_BLS1_C_FIXED);
			isp3_param_write(params_vdev, pval->gr, ISP3X_BLS1_D_FIXED);
			isp3_param_write(params_vdev, pval->gb, ISP3X_BLS1_A_FIXED);
			isp3_param_write(params_vdev, pval->b, ISP3X_BLS1_B_FIXED);
			break;
		case RAW_GRBG:
			isp3_param_write(params_vdev, pval->r, ISP3X_BLS1_B_FIXED);
			isp3_param_write(params_vdev, pval->gr, ISP3X_BLS1_A_FIXED);
			isp3_param_write(params_vdev, pval->gb, ISP3X_BLS1_D_FIXED);
			isp3_param_write(params_vdev, pval->b, ISP3X_BLS1_C_FIXED);
			break;
		case RAW_RGGB:
		default:
			isp3_param_write(params_vdev, pval->r, ISP3X_BLS1_A_FIXED);
			isp3_param_write(params_vdev, pval->gr, ISP3X_BLS1_B_FIXED);
			isp3_param_write(params_vdev, pval->gb, ISP3X_BLS1_C_FIXED);
			isp3_param_write(params_vdev, pval->b, ISP3X_BLS1_D_FIXED);
			break;
		}
	}

	/* fixed subtraction values */
	pval = &arg->fixed_val;
	if (!arg->enable_auto) {
		switch (params_vdev->raw_type) {
		case RAW_BGGR:
			isp3_param_write(params_vdev, pval->r, ISP3X_BLS_D_FIXED);
			isp3_param_write(params_vdev, pval->gr, ISP3X_BLS_C_FIXED);
			isp3_param_write(params_vdev, pval->gb, ISP3X_BLS_B_FIXED);
			isp3_param_write(params_vdev, pval->b, ISP3X_BLS_A_FIXED);
			break;
		case RAW_GBRG:
			isp3_param_write(params_vdev, pval->r, ISP3X_BLS_C_FIXED);
			isp3_param_write(params_vdev, pval->gr, ISP3X_BLS_D_FIXED);
			isp3_param_write(params_vdev, pval->gb, ISP3X_BLS_A_FIXED);
			isp3_param_write(params_vdev, pval->b, ISP3X_BLS_B_FIXED);
			break;
		case RAW_GRBG:
			isp3_param_write(params_vdev, pval->r, ISP3X_BLS_B_FIXED);
			isp3_param_write(params_vdev, pval->gr, ISP3X_BLS_A_FIXED);
			isp3_param_write(params_vdev, pval->gb, ISP3X_BLS_D_FIXED);
			isp3_param_write(params_vdev, pval->b, ISP3X_BLS_C_FIXED);
			break;
		case RAW_RGGB:
		default:
			isp3_param_write(params_vdev, pval->r, ISP3X_BLS_A_FIXED);
			isp3_param_write(params_vdev, pval->gr, ISP3X_BLS_B_FIXED);
			isp3_param_write(params_vdev, pval->gb, ISP3X_BLS_C_FIXED);
			isp3_param_write(params_vdev, pval->b, ISP3X_BLS_D_FIXED);
			break;
		}
	} else {
		if (arg->en_windows & BIT(1)) {
			isp3_param_write(params_vdev, arg->bls_window2.h_offs, ISP3X_BLS_H2_START);
			value = arg->bls_window2.h_offs + arg->bls_window2.h_size;
			isp3_param_write(params_vdev, value, ISP3X_BLS_H2_STOP);
			isp3_param_write(params_vdev, arg->bls_window2.v_offs, ISP3X_BLS_V2_START);
			value = arg->bls_window2.v_offs + arg->bls_window2.v_size;
			isp3_param_write(params_vdev, value, ISP3X_BLS_V2_STOP);
			new_control |= ISP_BLS_WINDOW_2;
		}

		if (arg->en_windows & BIT(0)) {
			isp3_param_write(params_vdev, arg->bls_window1.h_offs, ISP3X_BLS_H1_START);
			value = arg->bls_window1.h_offs + arg->bls_window1.h_size;
			isp3_param_write(params_vdev, value, ISP3X_BLS_H1_STOP);
			isp3_param_write(params_vdev, arg->bls_window1.v_offs, ISP3X_BLS_V1_START);
			value = arg->bls_window1.v_offs + arg->bls_window1.v_size;
			isp3_param_write(params_vdev, value, ISP3X_BLS_V1_STOP);
			new_control |= ISP_BLS_WINDOW_1;
		}

		isp3_param_write(params_vdev, arg->bls_samples, ISP3X_BLS_SAMPLES);

		new_control |= ISP_BLS_MODE_MEASURED;
	}
	isp3_param_write(params_vdev, new_control, ISP3X_BLS_CTRL);

	isp3_param_write(params_vdev, arg->isp_ob_offset, ISP32_BLS_ISP_OB_OFFSET);
	isp3_param_write(params_vdev, arg->isp_ob_predgain, ISP32_BLS_ISP_OB_PREDGAIN);
	isp3_param_write(params_vdev, arg->isp_ob_max, ISP32_BLS_ISP_OB_MAX);
}

static void
isp_bls_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 new_control;

	new_control = isp3_param_read(params_vdev, ISP3X_BLS_CTRL);
	if (en)
		new_control |= ISP_BLS_ENA;
	else
		new_control &= ~ISP_BLS_ENA;
	isp3_param_write(params_vdev, new_control, ISP3X_BLS_CTRL);
}

static void
isp_sdg_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp2x_sdg_cfg *arg)
{
	int i;

	isp3_param_write(params_vdev, arg->xa_pnts.gamma_dx0, ISP3X_ISP_GAMMA_DX_LO);
	isp3_param_write(params_vdev, arg->xa_pnts.gamma_dx1, ISP3X_ISP_GAMMA_DX_HI);

	for (i = 0; i < ISP32_DEGAMMA_CURVE_SIZE; i++) {
		isp3_param_write(params_vdev, arg->curve_r.gamma_y[i],
				 ISP3X_ISP_GAMMA_R_Y_0 + i * 4);
		isp3_param_write(params_vdev, arg->curve_g.gamma_y[i],
				 ISP3X_ISP_GAMMA_G_Y_0 + i * 4);
		isp3_param_write(params_vdev, arg->curve_b.gamma_y[i],
				 ISP3X_ISP_GAMMA_B_Y_0 + i * 4);
	}
}

static void
isp_sdg_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 val;

	val = isp3_param_read_cache(params_vdev, ISP3X_ISP_CTRL0);
	if (en)
		isp3_param_write(params_vdev, val | CIF_ISP_CTRL_ISP_GAMMA_IN_ENA,
				 ISP3X_ISP_CTRL0);
	else
		isp3_param_write(params_vdev, val & ~CIF_ISP_CTRL_ISP_GAMMA_IN_ENA,
				      ISP3X_ISP_CTRL0);
}

static void
isp_lsc_matrix_cfg_sram(struct rkisp_isp_params_vdev *params_vdev,
			const struct isp3x_lsc_cfg *pconfig,
			bool is_check)
{
	struct rkisp_device *dev = params_vdev->dev;
	u32 sram_addr, data, table;
	int i, j;

	if (is_check &&
	    !(isp3_param_read(params_vdev, ISP3X_LSC_CTRL) & ISP_LSC_EN))
		return;

	table = isp3_param_read_direct(params_vdev, ISP3X_LSC_STATUS);
	table &= ISP3X_LSC_ACTIVE_TABLE;
	/* default table 0 for multi device */
	if (!dev->hw_dev->is_single)
		table = ISP3X_LSC_ACTIVE_TABLE;

	/* CIF_ISP_LSC_TABLE_ADDRESS_153 = ( 17 * 18 ) >> 1 */
	sram_addr = table ? ISP3X_LSC_TABLE_ADDRESS_0 : CIF_ISP_LSC_TABLE_ADDRESS_153;
	isp3_param_write_direct(params_vdev, sram_addr, ISP3X_LSC_R_TABLE_ADDR);
	isp3_param_write_direct(params_vdev, sram_addr, ISP3X_LSC_GR_TABLE_ADDR);
	isp3_param_write_direct(params_vdev, sram_addr, ISP3X_LSC_GB_TABLE_ADDR);
	isp3_param_write_direct(params_vdev, sram_addr, ISP3X_LSC_B_TABLE_ADDR);

	/* program data tables (table size is 9 * 17 = 153) */
	for (i = 0; i < CIF_ISP_LSC_SECTORS_MAX * CIF_ISP_LSC_SECTORS_MAX;
	     i += CIF_ISP_LSC_SECTORS_MAX) {
		/*
		 * 17 sectors with 2 values in one DWORD = 9
		 * DWORDs (2nd value of last DWORD unused)
		 */
		for (j = 0; j < CIF_ISP_LSC_SECTORS_MAX - 1; j += 2) {
			data = ISP_ISP_LSC_TABLE_DATA(pconfig->r_data_tbl[i + j],
						      pconfig->r_data_tbl[i + j + 1]);
			isp3_param_write_direct(params_vdev, data, ISP3X_LSC_R_TABLE_DATA);

			data = ISP_ISP_LSC_TABLE_DATA(pconfig->gr_data_tbl[i + j],
						      pconfig->gr_data_tbl[i + j + 1]);
			isp3_param_write_direct(params_vdev, data, ISP3X_LSC_GR_TABLE_DATA);

			data = ISP_ISP_LSC_TABLE_DATA(pconfig->gb_data_tbl[i + j],
						      pconfig->gb_data_tbl[i + j + 1]);
			isp3_param_write_direct(params_vdev, data, ISP3X_LSC_GB_TABLE_DATA);

			data = ISP_ISP_LSC_TABLE_DATA(pconfig->b_data_tbl[i + j],
						      pconfig->b_data_tbl[i + j + 1]);
			isp3_param_write_direct(params_vdev, data, ISP3X_LSC_B_TABLE_DATA);
		}

		data = ISP_ISP_LSC_TABLE_DATA(pconfig->r_data_tbl[i + j], 0);
		isp3_param_write_direct(params_vdev, data, ISP3X_LSC_R_TABLE_DATA);

		data = ISP_ISP_LSC_TABLE_DATA(pconfig->gr_data_tbl[i + j], 0);
		isp3_param_write_direct(params_vdev, data, ISP3X_LSC_GR_TABLE_DATA);

		data = ISP_ISP_LSC_TABLE_DATA(pconfig->gb_data_tbl[i + j], 0);
		isp3_param_write_direct(params_vdev, data, ISP3X_LSC_GB_TABLE_DATA);

		data = ISP_ISP_LSC_TABLE_DATA(pconfig->b_data_tbl[i + j], 0);
		isp3_param_write_direct(params_vdev, data, ISP3X_LSC_B_TABLE_DATA);
	}
	isp3_param_write_direct(params_vdev, !table, ISP3X_LSC_TABLE_SEL);
}

static void
isp_lsc_cfg_sram_task(unsigned long data)
{
	struct rkisp_isp_params_vdev *params_vdev =
		(struct rkisp_isp_params_vdev *)data;
	struct isp32_isp_params_cfg *params = params_vdev->isp32_params;

	isp_lsc_matrix_cfg_sram(params_vdev, &params->others.lsc_cfg, true);
}

static void
isp_lsc_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp3x_lsc_cfg *arg)
{
	struct rkisp_isp_params_val_v32 *priv_val =
		(struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;
	struct isp32_isp_params_cfg *params_rec = params_vdev->isp32_params;
	struct rkisp_device *dev = params_vdev->dev;
	u32 data, lsc_ctrl;
	int i;

	lsc_ctrl = isp3_param_read(params_vdev, ISP3X_LSC_CTRL);
	params_rec->others.lsc_cfg = *arg;
	if (dev->hw_dev->is_single && (lsc_ctrl & ISP_LSC_EN))
		tasklet_schedule(&priv_val->lsc_tasklet);

	for (i = 0; i < ISP32_LSC_SIZE_TBL_SIZE / 4; i++) {
		/* program x size tables */
		data = CIF_ISP_LSC_SECT_SIZE(arg->x_size_tbl[i * 2], arg->x_size_tbl[i * 2 + 1]);
		isp3_param_write(params_vdev, data, ISP3X_LSC_XSIZE_01 + i * 4);
		data = CIF_ISP_LSC_SECT_SIZE(arg->x_size_tbl[i * 2 + 8], arg->x_size_tbl[i * 2 + 9]);
		isp3_param_write(params_vdev, data, ISP3X_LSC_XSIZE_89 + i * 4);

		/* program x grad tables */
		data = CIF_ISP_LSC_SECT_SIZE(arg->x_grad_tbl[i * 2], arg->x_grad_tbl[i * 2 + 1]);
		isp3_param_write(params_vdev, data, ISP3X_LSC_XGRAD_01 + i * 4);
		data = CIF_ISP_LSC_SECT_SIZE(arg->x_grad_tbl[i * 2 + 8], arg->x_grad_tbl[i * 2 + 9]);
		isp3_param_write(params_vdev, data, ISP3X_LSC_XGRAD_89 + i * 4);

		/* program y size tables */
		data = CIF_ISP_LSC_SECT_SIZE(arg->y_size_tbl[i * 2], arg->y_size_tbl[i * 2 + 1]);
		isp3_param_write(params_vdev, data, ISP3X_LSC_YSIZE_01 + i * 4);
		data = CIF_ISP_LSC_SECT_SIZE(arg->y_size_tbl[i * 2 + 8], arg->y_size_tbl[i * 2 + 9]);
		isp3_param_write(params_vdev, data, ISP3X_LSC_YSIZE_89 + i * 4);

		/* program y grad tables */
		data = CIF_ISP_LSC_SECT_SIZE(arg->y_grad_tbl[i * 2], arg->y_grad_tbl[i * 2 + 1]);
		isp3_param_write(params_vdev, data, ISP3X_LSC_YGRAD_01 + i * 4);
		data = CIF_ISP_LSC_SECT_SIZE(arg->y_grad_tbl[i * 2 + 8], arg->y_grad_tbl[i * 2 + 9]);
		isp3_param_write(params_vdev, data, ISP3X_LSC_YGRAD_89 + i * 4);
	}

	if (arg->sector_16x16)
		lsc_ctrl |= ISP3X_LSC_SECTOR_16X16;
	else
		lsc_ctrl &= ~ISP3X_LSC_SECTOR_16X16;
	isp3_param_write(params_vdev, lsc_ctrl, ISP3X_LSC_CTRL);
}

static void
isp_lsc_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	struct isp32_isp_params_cfg *params_rec = params_vdev->isp32_params;
	u32 val = isp3_param_read(params_vdev, ISP3X_LSC_CTRL);

	if (en == !!(val & ISP_LSC_EN))
		return;

	if (en) {
		val = ISP_LSC_EN | ISP32_SELF_FORCE_UPD;
		isp3_param_set_bits(params_vdev, ISP3X_LSC_CTRL, val);
		if (params_vdev->dev->hw_dev->is_single)
			isp_lsc_matrix_cfg_sram(params_vdev,
						&params_rec->others.lsc_cfg, false);
	} else {
		isp3_param_clear_bits(params_vdev, ISP3X_LSC_CTRL, ISP_LSC_EN);
		isp3_param_clear_bits(params_vdev, ISP3X_GAIN_CTRL, BIT(8));
	}
}

static void
isp_debayer_config(struct rkisp_isp_params_vdev *params_vdev,
		   const struct isp32_debayer_cfg *arg)
{
	u32 value;

	value = isp3_param_read(params_vdev, ISP3X_DEBAYER_CONTROL);
	value &= ISP_DEBAYER_EN;

	value |= !!arg->filter_c_en << 8 |
		 !!arg->filter_g_en << 4;
	isp3_param_write(params_vdev, value, ISP3X_DEBAYER_CONTROL);

	value = (arg->max_ratio & 0x3F) << 24 | arg->select_thed << 16 |
		(arg->thed1 & 0x0F) << 12 | (arg->thed0 & 0x0F) << 8 |
		(arg->dist_scale & 0x0F) << 4 | !!arg->clip_en;
	isp3_param_write(params_vdev, value, ISP3X_DEBAYER_G_INTERP);

	value = (arg->filter1_coe4 & 0x1F) << 24 | (arg->filter1_coe3 & 0x1F) << 16 |
		(arg->filter1_coe2 & 0x1F) << 8 | (arg->filter1_coe1 & 0x1F);
	isp3_param_write(params_vdev, value, ISP3X_DEBAYER_G_INTERP_FILTER1);

	value = (arg->filter2_coe4 & 0x1F) << 24 | (arg->filter2_coe3 & 0x1F) << 16 |
		(arg->filter2_coe2 & 0x1F) << 8 | (arg->filter2_coe1 & 0x1F);
	isp3_param_write(params_vdev, value, ISP3X_DEBAYER_G_INTERP_FILTER2);

	value = arg->hf_offset << 16 | (arg->gain_offset & 0xFFF);
	isp3_param_write(params_vdev, value, ISP32_DEBAYER_G_INTERP_OFFSET);

	value = (arg->offset & 0x7FF);
	isp3_param_write(params_vdev, value, ISP32_DEBAYER_G_FILTER_OFFSET);

	value = arg->guid_gaus_coe2 << 16 |
		arg->guid_gaus_coe1 << 8 | arg->guid_gaus_coe0;
	isp3_param_write(params_vdev, value, ISP32_DEBAYER_C_FILTER_GUIDE_GAUS);

	value = arg->ce_gaus_coe2 << 16 |
		arg->ce_gaus_coe1 << 8 | arg->ce_gaus_coe0;
	isp3_param_write(params_vdev, value, ISP32_DEBAYER_C_FILTER_CE_GAUS);

	value = arg->alpha_gaus_coe2 << 16 |
		arg->alpha_gaus_coe1 << 8 | arg->alpha_gaus_coe0;
	isp3_param_write(params_vdev, value, ISP32_DEBAYER_C_FILTER_ALPHA_GAUS);

	value = (arg->loggd_offset & 0xfff) << 16 | (arg->loghf_offset & 0x1fff);
	isp3_param_write(params_vdev, value, ISP32_DEBAYER_C_FILTER_LOG_OFFSET);

	value = (arg->alpha_scale & 0xfffff) << 12 | (arg->alpha_offset & 0xfff);
	isp3_param_write(params_vdev, value, ISP32_DEBAYER_C_FILTER_ALPHA);

	value = (arg->edge_scale & 0xfffff) << 12 | (arg->edge_offset & 0xfff);
	isp3_param_write(params_vdev, value, ISP32_DEBAYER_C_FILTER_EDGE);

	value = (arg->wgtslope & 0xfff) << 16 |
		(arg->exp_shift & 0x3f) << 8 | arg->ce_sgm;
	isp3_param_write(params_vdev, value, ISP32_DEBAYER_C_FILTER_IIR_0);

	value = (arg->wet_ghost & 0x3f) << 8 | (arg->wet_clip & 0x7f);
	isp3_param_write(params_vdev, value, ISP32_DEBAYER_C_FILTER_IIR_1);

	value = (arg->bf_curwgt & 0x7f) << 24 |
		(arg->bf_clip & 0x7f) << 16 | arg->bf_sgm;
	isp3_param_write(params_vdev, value, ISP32_DEBAYER_C_FILTER_BF);
}

static void
isp_debayer_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	if (en)
		isp3_param_set_bits(params_vdev,
				    ISP3X_DEBAYER_CONTROL, ISP32_MODULE_EN);
	else
		isp3_param_clear_bits(params_vdev,
				      ISP3X_DEBAYER_CONTROL, ISP32_MODULE_EN);
}

static void
isp_awbgain_config(struct rkisp_isp_params_vdev *params_vdev,
		   const struct isp32_awb_gain_cfg *arg)
{
	struct rkisp_device *dev = params_vdev->dev;

	if (!arg->gain0_red || !arg->gain0_blue ||
	    !arg->gain1_red || !arg->gain1_blue ||
	    !arg->gain2_red || !arg->gain2_blue ||
	    !arg->gain0_green_r || !arg->gain0_green_b ||
	    !arg->gain1_green_r || !arg->gain1_green_b ||
	    !arg->gain2_green_r || !arg->gain2_green_b) {
		dev_err(dev->dev, "awb gain is zero!\n");
		return;
	}

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->gain0_green_b, arg->gain0_green_r),
			 ISP3X_ISP_AWB_GAIN0_G);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->gain0_blue, arg->gain0_red),
			 ISP3X_ISP_AWB_GAIN0_RB);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->gain1_green_b, arg->gain1_green_r),
			 ISP3X_ISP_AWB_GAIN1_G);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->gain1_blue, arg->gain1_red),
			 ISP3X_ISP_AWB_GAIN1_RB);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->gain2_green_b, arg->gain2_green_r),
			 ISP3X_ISP_AWB_GAIN2_G);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->gain2_blue, arg->gain2_red),
			 ISP3X_ISP_AWB_GAIN2_RB);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->awb1_gain_gb, arg->awb1_gain_gr),
			 ISP32_ISP_AWB1_GAIN_G);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->awb1_gain_b, arg->awb1_gain_r),
			 ISP32_ISP_AWB1_GAIN_RB);
}

static void
isp_awbgain_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 val;

	val = isp3_param_read_cache(params_vdev, ISP3X_ISP_CTRL0);
	if (en)
		isp3_param_write(params_vdev, val | CIF_ISP_CTRL_ISP_AWB_ENA,
				 ISP3X_ISP_CTRL0);
	else
		isp3_param_write(params_vdev, val & ~CIF_ISP_CTRL_ISP_AWB_ENA,
				 ISP3X_ISP_CTRL0);
}

static void
isp_ccm_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp32_ccm_cfg *arg)
{
	u32 value;
	u32 i;

	value = isp3_param_read(params_vdev, ISP3X_CCM_CTRL);
	value &= ISP_CCM_EN;

	value |= !!arg->asym_adj_en << 3 |
		 !!arg->enh_adj_en << 2 |
		 !!arg->highy_adjust_dis << 1;
	isp3_param_write(params_vdev, value, ISP3X_CCM_CTRL);

	value = ISP_PACK_2SHORT(arg->coeff0_r, arg->coeff1_r);
	isp3_param_write(params_vdev, value, ISP3X_CCM_COEFF0_R);

	value = ISP_PACK_2SHORT(arg->coeff2_r, arg->offset_r);
	isp3_param_write(params_vdev, value, ISP3X_CCM_COEFF1_R);

	value = ISP_PACK_2SHORT(arg->coeff0_g, arg->coeff1_g);
	isp3_param_write(params_vdev, value, ISP3X_CCM_COEFF0_G);

	value = ISP_PACK_2SHORT(arg->coeff2_g, arg->offset_g);
	isp3_param_write(params_vdev, value, ISP3X_CCM_COEFF1_G);

	value = ISP_PACK_2SHORT(arg->coeff0_b, arg->coeff1_b);
	isp3_param_write(params_vdev, value, ISP3X_CCM_COEFF0_B);

	value = ISP_PACK_2SHORT(arg->coeff2_b, arg->offset_b);
	isp3_param_write(params_vdev, value, ISP3X_CCM_COEFF1_B);

	value = ISP_PACK_2SHORT(arg->coeff0_y, arg->coeff1_y);
	isp3_param_write(params_vdev, value, ISP3X_CCM_COEFF0_Y);

	value = ISP_PACK_2SHORT(arg->coeff2_y, 0);
	isp3_param_write(params_vdev, value, ISP3X_CCM_COEFF1_Y);

	for (i = 0; i < ISP32_CCM_CURVE_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->alp_y[2 * i], arg->alp_y[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_CCM_ALP_Y0 + 4 * i);
	}

	value = (arg->right_bit & 0xf) << 4 | (arg->bound_bit & 0xf);
	isp3_param_write(params_vdev, value, ISP3X_CCM_BOUND_BIT);

	value = (arg->color_coef1_g2y & 0x7ff) << 16 |
		(arg->color_coef0_r2y & 0x7ff);
	isp3_param_write(params_vdev, value, ISP32_CCM_ENHANCE0);

	value = (arg->color_enh_rat_max & 0x3fff) << 16 |
		(arg->color_coef2_b2y & 0x7ff);
	isp3_param_write(params_vdev, value, ISP32_CCM_ENHANCE1);
}

static void
isp_ccm_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	if (en)
		isp3_param_set_bits(params_vdev, ISP3X_CCM_CTRL, ISP_CCM_EN);
	else
		isp3_param_clear_bits(params_vdev, ISP3X_CCM_CTRL, ISP_CCM_EN);
}

static void
isp_goc_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp3x_gammaout_cfg *arg)
{
	int i;
	u32 value;

	value = isp3_param_read(params_vdev, ISP3X_GAMMA_OUT_CTRL);
	value &= ISP3X_GAMMA_OUT_EN;
	value |= !!arg->equ_segm << 1 | !!arg->finalx4_dense_en << 2;
	isp3_param_write(params_vdev, value, ISP3X_GAMMA_OUT_CTRL);

	isp3_param_write(params_vdev, arg->offset, ISP3X_GAMMA_OUT_OFFSET);
	for (i = 0; i < ISP32_GAMMA_OUT_MAX_SAMPLES / 2; i++) {
		value = ISP_PACK_2SHORT(arg->gamma_y[2 * i],
					arg->gamma_y[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_GAMMA_OUT_Y0 + i * 4);
	}
	isp3_param_write(params_vdev, arg->gamma_y[2 * i], ISP3X_GAMMA_OUT_Y0 + i * 4);
}

static void
isp_goc_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	if (en)
		isp3_param_set_bits(params_vdev, ISP3X_GAMMA_OUT_CTRL, ISP3X_GAMMA_OUT_EN);
	else
		isp3_param_clear_bits(params_vdev, ISP3X_GAMMA_OUT_CTRL, ISP3X_GAMMA_OUT_EN);
}

static void
isp_cproc_config(struct rkisp_isp_params_vdev *params_vdev,
		 const struct isp2x_cproc_cfg *arg)
{
	u32 quantization = params_vdev->quantization;

	isp3_param_write(params_vdev, arg->contrast, ISP3X_CPROC_CONTRAST);
	isp3_param_write(params_vdev, arg->hue, ISP3X_CPROC_HUE);
	isp3_param_write(params_vdev, arg->sat, ISP3X_CPROC_SATURATION);
	isp3_param_write(params_vdev, arg->brightness, ISP3X_CPROC_BRIGHTNESS);

	if (quantization != V4L2_QUANTIZATION_FULL_RANGE) {
		isp3_param_clear_bits(params_vdev, ISP3X_CPROC_CTRL,
				      CIF_C_PROC_YOUT_FULL |
				      CIF_C_PROC_YIN_FULL |
				      CIF_C_PROC_COUT_FULL);
	} else {
		isp3_param_set_bits(params_vdev, ISP3X_CPROC_CTRL,
				    CIF_C_PROC_YOUT_FULL |
				    CIF_C_PROC_YIN_FULL |
				    CIF_C_PROC_COUT_FULL);
	}
}

static void
isp_cproc_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	if (en)
		isp3_param_set_bits(params_vdev, ISP3X_CPROC_CTRL,
				    CIF_C_PROC_CTR_ENABLE);
	else
		isp3_param_clear_bits(params_vdev, ISP3X_CPROC_CTRL,
				      CIF_C_PROC_CTR_ENABLE);
}

static void
isp_ie_config(struct rkisp_isp_params_vdev *params_vdev,
	      const struct isp2x_ie_cfg *arg)
{
	u32 eff_ctrl;

	eff_ctrl = isp3_param_read(params_vdev, ISP3X_IMG_EFF_CTRL);
	eff_ctrl &= ~CIF_IMG_EFF_CTRL_MODE_MASK;

	if (params_vdev->quantization == V4L2_QUANTIZATION_FULL_RANGE)
		eff_ctrl |= CIF_IMG_EFF_CTRL_YCBCR_FULL;

	switch (arg->effect) {
	case V4L2_COLORFX_SEPIA:
		eff_ctrl |= CIF_IMG_EFF_CTRL_MODE_SEPIA;
		break;
	case V4L2_COLORFX_SET_CBCR:
		isp3_param_write(params_vdev, arg->eff_tint, ISP3X_IMG_EFF_TINT);
		eff_ctrl |= CIF_IMG_EFF_CTRL_MODE_SEPIA;
		break;
		/*
		 * Color selection is similar to water color(AQUA):
		 * grayscale + selected color w threshold
		 */
	case V4L2_COLORFX_AQUA:
		eff_ctrl |= CIF_IMG_EFF_CTRL_MODE_COLOR_SEL;
		isp3_param_write(params_vdev, arg->color_sel,
				 ISP3X_IMG_EFF_COLOR_SEL);
		break;
	case V4L2_COLORFX_EMBOSS:
		eff_ctrl |= CIF_IMG_EFF_CTRL_MODE_EMBOSS;
		isp3_param_write(params_vdev, arg->eff_mat_1,
				 CIF_IMG_EFF_MAT_1);
		isp3_param_write(params_vdev, arg->eff_mat_2,
				 CIF_IMG_EFF_MAT_2);
		isp3_param_write(params_vdev, arg->eff_mat_3,
				 CIF_IMG_EFF_MAT_3);
		break;
	case V4L2_COLORFX_SKETCH:
		eff_ctrl |= CIF_IMG_EFF_CTRL_MODE_SKETCH;
		isp3_param_write(params_vdev, arg->eff_mat_3,
				 CIF_IMG_EFF_MAT_3);
		isp3_param_write(params_vdev, arg->eff_mat_4,
				 CIF_IMG_EFF_MAT_4);
		isp3_param_write(params_vdev, arg->eff_mat_5,
				 CIF_IMG_EFF_MAT_5);
		break;
	case V4L2_COLORFX_BW:
		eff_ctrl |= CIF_IMG_EFF_CTRL_MODE_BLACKWHITE;
		break;
	case V4L2_COLORFX_NEGATIVE:
		eff_ctrl |= CIF_IMG_EFF_CTRL_MODE_NEGATIVE;
		break;
	default:
		break;
	}

	isp3_param_write(params_vdev, eff_ctrl, ISP3X_IMG_EFF_CTRL);
}

static void
isp_ie_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	if (en) {
		isp3_param_set_bits(params_vdev, ISP3X_IMG_EFF_CTRL,
				    CIF_IMG_EFF_CTRL_CFG_UPD |
				    CIF_IMG_EFF_CTRL_ENABLE);
	} else {
		isp3_param_clear_bits(params_vdev, ISP3X_IMG_EFF_CTRL,
				      CIF_IMG_EFF_CTRL_ENABLE);
	}
}

static void
isp_rawaebig_config_foraf(struct rkisp_isp_params_vdev *params_vdev,
			  const struct isp32_rawaf_meas_cfg *arg)
{
	u32 block_hsize, block_vsize;
	u32 addr, value;
	u32 wnd_num_idx = 2;
	const u32 ae_wnd_num[] = {
		1, 5, 15, 15
	};

	addr = ISP3X_RAWAE_BIG1_BASE;
	value = isp3_param_read(params_vdev, addr + ISP3X_RAWAE_BIG_CTRL);
	value &= ISP3X_RAWAE_BIG_EN;

	value |= ISP3X_RAWAE_BIG_WND0_NUM(wnd_num_idx);
	isp3_param_write(params_vdev, value, addr + ISP3X_RAWAE_BIG_CTRL);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->win[0].h_offs, arg->win[0].v_offs),
			 addr + ISP3X_RAWAE_BIG_OFFSET);

	block_hsize = arg->win[0].h_size / ae_wnd_num[wnd_num_idx];
	block_vsize = arg->win[0].v_size / ae_wnd_num[wnd_num_idx];
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(block_hsize, block_vsize),
			 addr + ISP3X_RAWAE_BIG_BLK_SIZE);
}

static void
isp_rawaf_config(struct rkisp_isp_params_vdev *params_vdev,
		 const struct isp32_rawaf_meas_cfg *arg)
{
	u32 i, var, ctrl;
	u16 h_size, v_size;
	u16 h_offs, v_offs;
	u8 gaus_en, viir_en, v1_fir_sel;
	size_t num_of_win = min_t(size_t, ARRAY_SIZE(arg->win),
				  arg->num_afm_win);

	for (i = 0; i < num_of_win; i++) {
		h_size = arg->win[i].h_size;
		v_size = arg->win[i].v_size;
		h_offs = arg->win[i].h_offs < 2 ? 2 : arg->win[i].h_offs;
		v_offs = arg->win[i].v_offs < 1 ? 1 : arg->win[i].v_offs;

		if (i == 0) {
			h_size = h_size / 15 * 15;
			v_size = v_size / 15 * 15;
		}

		/*
		 * (horizontal left row), value must be greater or equal 2
		 * (vertical top line), value must be greater or equal 1
		 */
		isp3_param_write(params_vdev,
				 ISP_PACK_2SHORT(v_offs, h_offs),
				 ISP3X_RAWAF_OFFSET_WINA + i * 8);

		/*
		 * value must be smaller than [width of picture -2]
		 * value must be lower than (number of lines -2)
		 */
		isp3_param_write(params_vdev,
				 ISP_PACK_2SHORT(v_size, h_size),
				 ISP3X_RAWAF_SIZE_WINA + i * 8);
	}

	var = 0;
	for (i = 0; i < ISP32_RAWAF_LINE_NUM; i++) {
		if (arg->line_en[i])
			var |= ISP3X_RAWAF_INTLINE0_EN << i;
		var |= ISP3X_RAWAF_INELINE0(arg->line_num[i]) << 4 * i;
	}
	isp3_param_write(params_vdev, var, ISP3X_RAWAF_INT_LINE);

	var = isp3_param_read(params_vdev, ISP3X_RAWAF_THRES);
	var &= ~0xFFFF;
	var |= arg->afm_thres;
	isp3_param_write(params_vdev, var, ISP3X_RAWAF_THRES);

	var = (arg->lum_var_shift[1] & 0x7) << 20 | (arg->lum_var_shift[0] & 0x7) << 16 |
		(arg->afm_var_shift[1] & 0x7) << 4 | (arg->afm_var_shift[0] & 0x7);
	isp3_param_write(params_vdev, var, ISP3X_RAWAF_VAR_SHIFT);

	for (i = 0; i < ISP32_RAWAF_GAMMA_NUM / 2; i++) {
		var = ISP_PACK_2SHORT(arg->gamma_y[2 * i], arg->gamma_y[2 * i + 1]);
		isp3_param_write(params_vdev, var, ISP3X_RAWAF_GAMMA_Y0 + i * 4);
	}
	var = ISP_PACK_2SHORT(arg->gamma_y[16], 0);
	isp3_param_write(params_vdev, var, ISP3X_RAWAF_GAMMA_Y8);

	var = (arg->v2iir_var_shift & 0x7) << 12 | (arg->v1iir_var_shift & 0x7) << 8 |
		(arg->h2iir_var_shift & 0x7) << 4 | (arg->h1iir_var_shift & 0x7);
	isp3_param_write(params_vdev, var, ISP3X_RAWAF_HVIIR_VAR_SHIFT);

	var = ISP_PACK_2SHORT(arg->h_fv_thresh, arg->v_fv_thresh);
	isp3_param_write(params_vdev, var, ISP3X_RAWAF_HIIR_THRESH);

	for (i = 0; i < ISP32_RAWAF_VFIR_COE_NUM; i++) {
		var = ISP_PACK_2SHORT(arg->v1fir_coe[i], arg->v2fir_coe[i]);
		isp3_param_write(params_vdev, var, ISP32_RAWAF_V_FIR_COE0 + i * 4);
	}

	for (i = 0; i < ISP32_RAWAF_GAUS_COE_NUM / 4; i++) {
		var = ISP_PACK_4BYTE(arg->gaus_coe[i * 4], arg->gaus_coe[i * 4 + 1],
				     arg->gaus_coe[i * 4 + 2], arg->gaus_coe[i * 4 + 3]);
		isp3_param_write(params_vdev, var, ISP32_RAWAF_GAUS_COE03 + i * 4);
	}
	var = ISP_PACK_4BYTE(arg->gaus_coe[ISP32_RAWAF_GAUS_COE_NUM - 1], 0, 0, 0);
	isp3_param_write(params_vdev, var, ISP32_RAWAF_GAUS_COE8);

	isp3_param_write(params_vdev, arg->highlit_thresh, ISP3X_RAWAF_HIGHLIT_THRESH);

	viir_en = arg->viir_en;
	gaus_en = arg->gaus_en;
	v1_fir_sel = arg->v1_fir_sel;
	if (viir_en == 0)
		v1_fir_sel = 0;

	ctrl = isp3_param_read(params_vdev, ISP3X_RAWAF_CTRL);
	ctrl &= ISP3X_RAWAF_EN;
	if (arg->hiir_en) {
		ctrl |= ISP3X_RAWAF_HIIR_EN;
		for (i = 0; i < ISP32_RAWAF_HIIR_COE_NUM / 2; i++) {
			var = ISP_PACK_2SHORT(arg->h1iir1_coe[i * 2], arg->h1iir1_coe[i * 2 + 1]);
			isp3_param_write(params_vdev, var, ISP3X_RAWAF_H1_IIR1_COE01 + i * 4);
			var = ISP_PACK_2SHORT(arg->h1iir2_coe[i * 2], arg->h1iir2_coe[i * 2 + 1]);
			isp3_param_write(params_vdev, var, ISP3X_RAWAF_H1_IIR2_COE01 + i * 4);
			var = ISP_PACK_2SHORT(arg->h2iir1_coe[i * 2], arg->h2iir1_coe[i * 2 + 1]);
			isp3_param_write(params_vdev, var, ISP3X_RAWAF_H2_IIR1_COE01 + i * 4);
			var = ISP_PACK_2SHORT(arg->h2iir2_coe[i * 2], arg->h2iir2_coe[i * 2 + 1]);
			isp3_param_write(params_vdev, var, ISP3X_RAWAF_H2_IIR2_COE01 + i * 4);
		}
	}
	if (viir_en) {
		ctrl |= ISP3X_RAWAF_VIIR_EN;
		for (i = 0; i < ISP32_RAWAF_VIIR_COE_NUM; i++) {
			var = ISP_PACK_2SHORT(arg->v1iir_coe[i], arg->v2iir_coe[i]);
			isp3_param_write(params_vdev, var, ISP3X_RAWAF_V_IIR_COE0 + i * 4);
		}
	}
	if (arg->ldg_en) {
		ctrl |= ISP3X_RAWAF_LDG_EN;
		for (i = 0; i < ISP32_RAWAF_CURVE_NUM; i++) {
			isp3_param_write(params_vdev,
					 arg->curve_h[i].ldg_lumth |
					 arg->curve_h[i].ldg_gain << 8 |
					 arg->curve_h[i].ldg_gslp << 16,
					 ISP3X_RAWAF_H_CURVEL + i * 16);
			isp3_param_write(params_vdev,
					 arg->curve_v[i].ldg_lumth |
					 arg->curve_v[i].ldg_gain << 8 |
					 arg->curve_v[i].ldg_gslp << 16,
					 ISP3X_RAWAF_V_CURVEL + i * 16);
		}
	}

	ctrl |= !!arg->ae_config_use << 20 | !!arg->from_ynr << 19 |
		!!arg->from_awb << 18 | (arg->v_dnscl_mode & 0x3) << 16 |
		!!arg->sobel_sel << 15 | !!arg->vldg_sel << 14 |
		!!arg->y_mode << 13 | !!arg->ae_mode << 12 |
		!!arg->v2_fv_mode << 11 | !!arg->v1_fv_mode << 10 |
		!!arg->h2_fv_mode << 9 | !!arg->h1_fv_mode << 8 |
		!!arg->accu_8bit_mode << 6 | !!v1_fir_sel  << 3 |
		!!gaus_en << 2 | !!arg->gamma_en << 1;
	isp3_param_write(params_vdev, ctrl, ISP3X_RAWAF_CTRL);

	ctrl = isp3_param_read(params_vdev, ISP3X_VI_ISP_PATH);
	if ((ctrl & ISP3X_RAWAF_SEL(3)) != ISP3X_RAWAF_SEL(arg->rawaf_sel)) {
		ctrl &= ~(ISP3X_RAWAF_SEL(3));
		ctrl |= ISP3X_RAWAF_SEL(arg->rawaf_sel);
		isp3_param_write(params_vdev, ctrl, ISP3X_VI_ISP_PATH);
	}

	params_vdev->afaemode_en = arg->ae_mode;
	if (params_vdev->afaemode_en)
		isp_rawaebig_config_foraf(params_vdev, arg);
}

static void
isp_rawaebig_enable_foraf(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 exp_ctrl;
	u32 addr = ISP3X_RAWAE_BIG1_BASE;

	exp_ctrl = isp3_param_read(params_vdev, addr + ISP3X_RAWAE_BIG_CTRL);
	exp_ctrl &= ~ISP32_REG_WR_MASK;
	if (en)
		exp_ctrl |= ISP32_MODULE_EN;
	else
		exp_ctrl &= ~ISP32_MODULE_EN;

	isp3_param_write(params_vdev, exp_ctrl, addr + ISP3X_RAWAE_BIG_CTRL);
}

static void
isp_rawaf_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 afm_ctrl = isp3_param_read(params_vdev, ISP3X_RAWAF_CTRL);

	afm_ctrl &= ~ISP32_REG_WR_MASK;
	if (en)
		afm_ctrl |= ISP3X_RAWAF_EN;
	else
		afm_ctrl &= ~ISP3X_RAWAF_EN;

	isp3_param_write(params_vdev, afm_ctrl, ISP3X_RAWAF_CTRL);
	if (params_vdev->afaemode_en) {
		isp_rawaebig_enable_foraf(params_vdev, en);
		if (!en)
			params_vdev->afaemode_en = false;
	}
}

static void
isp_rawaelite_config(struct rkisp_isp_params_vdev *params_vdev,
		     const struct isp2x_rawaelite_meas_cfg *arg)
{
	struct rkisp_device *ispdev = params_vdev->dev;
	struct v4l2_rect *out_crop = &ispdev->isp_sdev.out_crop;
	u32 width = out_crop->width;
	u32 block_hsize, block_vsize, value;
	u32 wnd_num_idx = 0;
	const u32 ae_wnd_num[] = {1, 5};

	value = isp3_param_read(params_vdev, ISP3X_RAWAE_LITE_CTRL);
	value &= ~(ISP3X_RAWAE_LITE_WNDNUM);
	if (arg->wnd_num) {
		value |= ISP3X_RAWAE_LITE_WNDNUM;
		wnd_num_idx = 1;
	}
	value &= ~ISP32_REG_WR_MASK;
	isp3_param_write(params_vdev, value, ISP3X_RAWAE_LITE_CTRL);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->win.h_offs, arg->win.v_offs),
			 ISP3X_RAWAE_LITE_OFFSET);

	block_hsize = arg->win.h_size / ae_wnd_num[wnd_num_idx];
	value = block_hsize * ae_wnd_num[wnd_num_idx] + arg->win.h_offs;
	if (ispdev->hw_dev->is_unite)
		width = width / 2 + RKMOUDLE_UNITE_EXTEND_PIXEL;
	if (value + 1 > width)
		block_hsize -= 1;
	block_vsize = arg->win.v_size / ae_wnd_num[wnd_num_idx];
	value = block_vsize * ae_wnd_num[wnd_num_idx] + arg->win.v_offs;
	if (value + 2 > out_crop->height)
		block_vsize -= 1;
	if (block_vsize % 2)
		block_vsize -= 1;
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(block_hsize, block_vsize),
			 ISP3X_RAWAE_LITE_BLK_SIZ);

	value = isp3_param_read(params_vdev, ISP3X_VI_ISP_PATH);
	if ((value & ISP3X_RAWAE012_SEL(3)) != ISP3X_RAWAE012_SEL(arg->rawae_sel)) {
		value &= ~(ISP3X_RAWAE012_SEL(3));
		value |= ISP3X_RAWAE012_SEL(arg->rawae_sel);
		isp3_param_write(params_vdev, value, ISP3X_VI_ISP_PATH);
	}
}

static void
isp_rawaelite_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 exp_ctrl;

	exp_ctrl = isp3_param_read(params_vdev, ISP3X_RAWAE_LITE_CTRL);
	exp_ctrl &= ~ISP32_REG_WR_MASK;
	if (en)
		exp_ctrl |= ISP3X_RAWAE_LITE_EN;
	else
		exp_ctrl &= ~ISP3X_RAWAE_LITE_EN;

	isp3_param_write(params_vdev, exp_ctrl, ISP3X_RAWAE_LITE_CTRL);
}

static void
isp_rawaebig_config(struct rkisp_isp_params_vdev *params_vdev,
		    const struct isp2x_rawaebig_meas_cfg *arg,
		    u32 blk_no)
{
	struct rkisp_device *ispdev = params_vdev->dev;
	struct v4l2_rect *out_crop = &ispdev->isp_sdev.out_crop;
	u32 width = out_crop->width;
	u32 block_hsize, block_vsize;
	u32 addr, i, value, h_size, v_size;
	u32 wnd_num_idx = 0;
	const u32 ae_wnd_num[] = {
		1, 5, 15, 15
	};

	switch (blk_no) {
	case 1:
		addr = ISP3X_RAWAE_BIG2_BASE;
		break;
	case 2:
		addr = ISP3X_RAWAE_BIG3_BASE;
		break;
	case 0:
	default:
		addr = ISP3X_RAWAE_BIG1_BASE;
		break;
	}

	/* avoid to override the old enable value */
	value = isp3_param_read(params_vdev, addr + ISP3X_RAWAE_BIG_CTRL);
	value &= ISP3X_RAWAE_BIG_EN;

	wnd_num_idx = arg->wnd_num;
	value |= ISP3X_RAWAE_BIG_WND0_NUM(wnd_num_idx);

	if (arg->subwin_en[0])
		value |= ISP3X_RAWAE_BIG_WND1_EN;
	if (arg->subwin_en[1])
		value |= ISP3X_RAWAE_BIG_WND2_EN;
	if (arg->subwin_en[2])
		value |= ISP3X_RAWAE_BIG_WND3_EN;
	if (arg->subwin_en[3])
		value |= ISP3X_RAWAE_BIG_WND4_EN;

	isp3_param_write(params_vdev, value, addr + ISP3X_RAWAE_BIG_CTRL);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->win.h_offs, arg->win.v_offs),
			 addr + ISP3X_RAWAE_BIG_OFFSET);

	block_hsize = arg->win.h_size / ae_wnd_num[wnd_num_idx];
	value = block_hsize * ae_wnd_num[wnd_num_idx] + arg->win.h_offs;
	if (ispdev->hw_dev->is_unite)
		width = width / 2 + RKMOUDLE_UNITE_EXTEND_PIXEL;
	if (value + 1 > width)
		block_hsize -= 1;
	block_vsize = arg->win.v_size / ae_wnd_num[wnd_num_idx];
	value = block_vsize * ae_wnd_num[wnd_num_idx] + arg->win.v_offs;
	if (value + 2 > out_crop->height)
		block_vsize -= 1;
	if (block_vsize % 2)
		block_vsize -= 1;
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(block_hsize, block_vsize),
			 addr + ISP3X_RAWAE_BIG_BLK_SIZE);

	for (i = 0; i < ISP32_RAWAEBIG_SUBWIN_NUM; i++) {
		isp3_param_write(params_vdev,
			ISP_PACK_2SHORT(arg->subwin[i].h_offs, arg->subwin[i].v_offs),
			addr + ISP3X_RAWAE_BIG_WND1_OFFSET + 8 * i);

		v_size = arg->subwin[i].v_size + arg->subwin[i].v_offs;
		h_size = arg->subwin[i].h_size + arg->subwin[i].h_offs;
		isp3_param_write(params_vdev,
			ISP_PACK_2SHORT(h_size, v_size),
			addr + ISP3X_RAWAE_BIG_WND1_SIZE + 8 * i);
	}

	value = isp3_param_read(params_vdev, ISP3X_VI_ISP_PATH);
	if (blk_no == 0) {
		if ((value & ISP3X_RAWAE3_SEL(3)) != ISP3X_RAWAE3_SEL(arg->rawae_sel)) {
			value &= ~(ISP3X_RAWAE3_SEL(3));
			value |= ISP3X_RAWAE3_SEL(arg->rawae_sel);
			isp3_param_write(params_vdev, value, ISP3X_VI_ISP_PATH);
		}
	} else {
		if ((value & ISP3X_RAWAE012_SEL(3)) != ISP3X_RAWAE012_SEL(arg->rawae_sel)) {
			value &= ~(ISP3X_RAWAE012_SEL(3));
			value |= ISP3X_RAWAE012_SEL(arg->rawae_sel);
			isp3_param_write(params_vdev, value, ISP3X_VI_ISP_PATH);
		}
	}
}

static void
isp_rawaebig_enable(struct rkisp_isp_params_vdev *params_vdev,
		    bool en, u32 blk_no)
{
	u32 exp_ctrl;
	u32 addr;

	switch (blk_no) {
	case 1:
		addr = ISP3X_RAWAE_BIG2_BASE;
		break;
	case 2:
		addr = ISP3X_RAWAE_BIG3_BASE;
		break;
	case 0:
	default:
		addr = ISP3X_RAWAE_BIG1_BASE;
		break;
	}

	exp_ctrl = isp3_param_read(params_vdev, addr + ISP3X_RAWAE_BIG_CTRL);
	exp_ctrl &= ~ISP32_REG_WR_MASK;
	if (en)
		exp_ctrl |= ISP32_MODULE_EN;
	else
		exp_ctrl &= ~ISP32_MODULE_EN;

	isp3_param_write(params_vdev, exp_ctrl, addr + ISP3X_RAWAE_BIG_CTRL);
}

static void
isp_rawae1_config(struct rkisp_isp_params_vdev *params_vdev,
		  const struct isp2x_rawaebig_meas_cfg *arg)
{
	isp_rawaebig_config(params_vdev, arg, 1);
}

static void
isp_rawae1_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	isp_rawaebig_enable(params_vdev, en, 1);
}

static void
isp_rawae2_config(struct rkisp_isp_params_vdev *params_vdev,
		  const struct isp2x_rawaebig_meas_cfg *arg)
{
	isp_rawaebig_config(params_vdev, arg, 2);
}

static void
isp_rawae2_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	isp_rawaebig_enable(params_vdev, en, 2);
}

static void
isp_rawae3_config(struct rkisp_isp_params_vdev *params_vdev,
		  const struct isp2x_rawaebig_meas_cfg *arg)
{
	isp_rawaebig_config(params_vdev, arg, 0);
}

static void
isp_rawae3_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	isp_rawaebig_enable(params_vdev, en, 0);
}

static void
isp_rawawb_cfg_sram(struct rkisp_isp_params_vdev *params_vdev,
		    const struct isp32_rawawb_meas_cfg *arg, bool is_check)
{
	u32 i, val = ISP32_MODULE_EN;

	if (is_check &&
	    !(isp3_param_read(params_vdev, ISP3X_RAWAWB_CTRL) & val))
		return;

	for (i = 0; i < ISP32_RAWAWB_WEIGHT_NUM / 5; i++) {
		isp3_param_write(params_vdev,
				 (arg->wp_blk_wei_w[5 * i] & 0x3f) |
				 (arg->wp_blk_wei_w[5 * i + 1] & 0x3f) << 6 |
				 (arg->wp_blk_wei_w[5 * i + 2] & 0x3f) << 12 |
				 (arg->wp_blk_wei_w[5 * i + 3] & 0x3f) << 18 |
				 (arg->wp_blk_wei_w[5 * i + 4] & 0x3f) << 24,
				 ISP3X_RAWAWB_WRAM_DATA_BASE);
	}
}

static void
isp_rawawb_config(struct rkisp_isp_params_vdev *params_vdev,
		  const struct isp32_rawawb_meas_cfg *arg)
{
	struct isp32_isp_params_cfg *params_rec = params_vdev->isp32_params;
	struct isp32_rawawb_meas_cfg *arg_rec = &params_rec->meas.rawawb;
	const struct isp2x_bls_fixed_val *pval = &arg->bls2_val;
	u32 value, val, mask;

	value = isp3_param_read(params_vdev, ISP3X_BLS_CTRL);
	value &= ~ISP32_BLS_BLS2_EN;
	if (arg->bls2_en) {
		switch (params_vdev->raw_type) {
		case RAW_BGGR:
			isp3_param_write(params_vdev, pval->r, ISP32_BLS2_D_FIXED);
			isp3_param_write(params_vdev, pval->gr, ISP32_BLS2_C_FIXED);
			isp3_param_write(params_vdev, pval->gb, ISP32_BLS2_B_FIXED);
			isp3_param_write(params_vdev, pval->b, ISP32_BLS2_A_FIXED);
			break;
		case RAW_GBRG:
			isp3_param_write(params_vdev, pval->r, ISP32_BLS2_C_FIXED);
			isp3_param_write(params_vdev, pval->gr, ISP32_BLS2_D_FIXED);
			isp3_param_write(params_vdev, pval->gb, ISP32_BLS2_A_FIXED);
			isp3_param_write(params_vdev, pval->b, ISP32_BLS2_B_FIXED);
			break;
		case RAW_GRBG:
			isp3_param_write(params_vdev, pval->r, ISP32_BLS2_B_FIXED);
			isp3_param_write(params_vdev, pval->gr, ISP32_BLS2_A_FIXED);
			isp3_param_write(params_vdev, pval->gb, ISP32_BLS2_D_FIXED);
			isp3_param_write(params_vdev, pval->b, ISP32_BLS2_C_FIXED);
			break;
		case RAW_RGGB:
		default:
			isp3_param_write(params_vdev, pval->r, ISP32_BLS2_A_FIXED);
			isp3_param_write(params_vdev, pval->gr, ISP32_BLS2_B_FIXED);
			isp3_param_write(params_vdev, pval->gb, ISP32_BLS2_C_FIXED);
			isp3_param_write(params_vdev, pval->b, ISP32_BLS2_D_FIXED);
		}
		value |= ISP32_BLS_BLS2_EN;
	}
	isp3_param_write(params_vdev, value, ISP3X_BLS_CTRL);

	value = arg->in_overexposure_threshold << 16 |
		!!arg->blk_with_luma_wei_en << 8 |
		(arg->blk_measure_illu_idx & 0x7) << 4 |
		!!arg->blk_rtdw_measure_en << 3 |
		!!arg->blk_measure_xytype << 2 |
		!!arg->blk_measure_mode << 1 |
		!!arg->blk_measure_enable;
	isp3_param_write(params_vdev, value, ISP3X_RAWAWB_BLK_CTRL);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->h_offs, arg->v_offs),
			 ISP3X_RAWAWB_WIN_OFFS);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->h_size, arg->v_size),
			 ISP3X_RAWAWB_WIN_SIZE);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->r_max, arg->g_max),
			 ISP3X_RAWAWB_LIMIT_RG_MAX);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->b_max, arg->y_max),
			 ISP3X_RAWAWB_LIMIT_BY_MAX);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->r_min, arg->g_min),
			 ISP3X_RAWAWB_LIMIT_RG_MIN);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->b_min, arg->y_min),
			 ISP3X_RAWAWB_LIMIT_BY_MIN);

	value = !!arg->wp_hist_xytype << 4 |
		!!arg->wp_blk_wei_en1 << 3 |
		!!arg->wp_blk_wei_en0 << 2 |
		!!arg->wp_luma_wei_en1 << 1 |
		!!arg->wp_luma_wei_en0;
	isp3_param_write(params_vdev, value, ISP3X_RAWAWB_WEIGHT_CURVE_CTRL);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->wp_luma_weicurve_y0,
					arg->wp_luma_weicurve_y1,
					arg->wp_luma_weicurve_y2,
					arg->wp_luma_weicurve_y3),
			 ISP3X_RAWAWB_YWEIGHT_CURVE_XCOOR03);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->wp_luma_weicurve_y4,
					arg->wp_luma_weicurve_y5,
					arg->wp_luma_weicurve_y6,
					arg->wp_luma_weicurve_y7),
			 ISP3X_RAWAWB_YWEIGHT_CURVE_XCOOR47);

	isp3_param_write(params_vdev,
			 arg->wp_luma_weicurve_y8,
			 ISP3X_RAWAWB_YWEIGHT_CURVE_XCOOR8);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->wp_luma_weicurve_w0,
					arg->wp_luma_weicurve_w1,
					arg->wp_luma_weicurve_w2,
					arg->wp_luma_weicurve_w3),
			 ISP3X_RAWAWB_YWEIGHT_CURVE_YCOOR03);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->wp_luma_weicurve_w4,
					arg->wp_luma_weicurve_w5,
					arg->wp_luma_weicurve_w6,
					arg->wp_luma_weicurve_w7),
			 ISP3X_RAWAWB_YWEIGHT_CURVE_YCOOR47);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->wp_luma_weicurve_w8,
					 arg->pre_wbgain_inv_r),
			 ISP3X_RAWAWB_YWEIGHT_CURVE_YCOOR8);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->pre_wbgain_inv_g,
					 arg->pre_wbgain_inv_b),
			 ISP3X_RAWAWB_PRE_WBGAIN_INV);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex0_u_0, arg->vertex0_v_0),
			 ISP3X_RAWAWB_UV_DETC_VERTEX0_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex1_u_0, arg->vertex1_v_0),
			 ISP3X_RAWAWB_UV_DETC_VERTEX1_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex2_u_0, arg->vertex2_v_0),
			 ISP3X_RAWAWB_UV_DETC_VERTEX2_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex3_u_0, arg->vertex3_v_0),
			 ISP3X_RAWAWB_UV_DETC_VERTEX3_0);

	isp3_param_write(params_vdev, arg->islope01_0,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE01_0);

	isp3_param_write(params_vdev, arg->islope12_0,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE12_0);

	isp3_param_write(params_vdev, arg->islope23_0,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE23_0);

	isp3_param_write(params_vdev, arg->islope30_0,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE30_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex0_u_1,
					 arg->vertex0_v_1),
			 ISP3X_RAWAWB_UV_DETC_VERTEX0_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex1_u_1,
					 arg->vertex1_v_1),
			 ISP3X_RAWAWB_UV_DETC_VERTEX1_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex2_u_1,
					 arg->vertex2_v_1),
			 ISP3X_RAWAWB_UV_DETC_VERTEX2_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex3_u_1,
					 arg->vertex3_v_1),
			 ISP3X_RAWAWB_UV_DETC_VERTEX3_1);

	isp3_param_write(params_vdev, arg->islope01_1,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE01_1);

	isp3_param_write(params_vdev, arg->islope12_1,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE12_1);

	isp3_param_write(params_vdev, arg->islope23_1,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE23_1);

	isp3_param_write(params_vdev, arg->islope30_1,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE30_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex0_u_2,
					 arg->vertex0_v_2),
			 ISP3X_RAWAWB_UV_DETC_VERTEX0_2);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex1_u_2,
					 arg->vertex1_v_2),
			 ISP3X_RAWAWB_UV_DETC_VERTEX1_2);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex2_u_2,
					 arg->vertex2_v_2),
			 ISP3X_RAWAWB_UV_DETC_VERTEX2_2);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex3_u_2,
					 arg->vertex3_v_2),
			 ISP3X_RAWAWB_UV_DETC_VERTEX3_2);

	isp3_param_write(params_vdev, arg->islope01_2,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE01_2);

	isp3_param_write(params_vdev, arg->islope12_2,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE12_2);

	isp3_param_write(params_vdev, arg->islope23_2,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE23_2);

	isp3_param_write(params_vdev, arg->islope30_2,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE30_2);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex0_u_3,
					 arg->vertex0_v_3),
			 ISP3X_RAWAWB_UV_DETC_VERTEX0_3);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex1_u_3,
					 arg->vertex1_v_3),
			 ISP3X_RAWAWB_UV_DETC_VERTEX1_3);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex2_u_3,
					 arg->vertex2_v_3),
			 ISP3X_RAWAWB_UV_DETC_VERTEX2_3);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->vertex3_u_3,
					 arg->vertex3_v_3),
			 ISP3X_RAWAWB_UV_DETC_VERTEX3_3);

	isp3_param_write(params_vdev, arg->islope01_3,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE01_3);

	isp3_param_write(params_vdev, arg->islope12_3,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE12_3);

	isp3_param_write(params_vdev, arg->islope23_3,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE23_3);

	isp3_param_write(params_vdev, arg->islope30_3,
			 ISP3X_RAWAWB_UV_DETC_ISLOPE30_3);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->rgb2ryuvmat0_y,
					 arg->rgb2ryuvmat1_y),
			 ISP3X_RAWAWB_YUV_RGB2ROTY_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->rgb2ryuvmat2_y,
					 arg->rgb2ryuvofs_y),
			 ISP3X_RAWAWB_YUV_RGB2ROTY_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->rgb2ryuvmat0_u,
					 arg->rgb2ryuvmat1_u),
			 ISP3X_RAWAWB_YUV_RGB2ROTU_0);


	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->rgb2ryuvmat2_u,
					 arg->rgb2ryuvofs_u),
			 ISP3X_RAWAWB_YUV_RGB2ROTU_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->rgb2ryuvmat0_v,
					 arg->rgb2ryuvmat1_v),
			 ISP3X_RAWAWB_YUV_RGB2ROTV_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->rgb2ryuvmat2_v,
					 arg->rgb2ryuvofs_v),
			 ISP3X_RAWAWB_YUV_RGB2ROTV_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->coor_x1_ls0_y,
					 arg->vec_x21_ls0_y),
			 ISP3X_RAWAWB_YUV_X_COOR_Y_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->coor_x1_ls0_u,
					 arg->vec_x21_ls0_u),
			 ISP3X_RAWAWB_YUV_X_COOR_U_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->coor_x1_ls0_v,
					 arg->vec_x21_ls0_v),
			 ISP3X_RAWAWB_YUV_X_COOR_V_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->dis_x1x2_ls0, 0,
					arg->rotu0_ls0, arg->rotu1_ls0),
			 ISP3X_RAWAWB_YUV_X1X2_DIS_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->rotu2_ls0, arg->rotu3_ls0,
					arg->rotu4_ls0, arg->rotu5_ls0),
			 ISP3X_RAWAWB_YUV_INTERP_CURVE_UCOOR_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->th0_ls0, arg->th1_ls0),
			 ISP3X_RAWAWB_YUV_INTERP_CURVE_TH0_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->th2_ls0, arg->th3_ls0),
			 ISP3X_RAWAWB_YUV_INTERP_CURVE_TH1_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->th4_ls0, arg->th5_ls0),
			 ISP3X_RAWAWB_YUV_INTERP_CURVE_TH2_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->coor_x1_ls1_y,
					 arg->vec_x21_ls1_y),
			 ISP3X_RAWAWB_YUV_X_COOR_Y_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->coor_x1_ls1_u,
					 arg->vec_x21_ls1_u),
			 ISP3X_RAWAWB_YUV_X_COOR_U_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->coor_x1_ls1_v,
					 arg->vec_x21_ls1_v),
			 ISP3X_RAWAWB_YUV_X_COOR_V_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->dis_x1x2_ls1, 0,
					arg->rotu0_ls1, arg->rotu1_ls1),
			 ISP3X_RAWAWB_YUV_X1X2_DIS_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->rotu2_ls1, arg->rotu3_ls1,
					arg->rotu4_ls1, arg->rotu5_ls1),
			 ISP3X_RAWAWB_YUV_INTERP_CURVE_UCOOR_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->th0_ls1, arg->th1_ls1),
			 ISP3X_RAWAWB_YUV_INTERP_CURVE_TH0_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->th2_ls1, arg->th3_ls1),
			 ISP3X_RAWAWB_YUV_INTERP_CURVE_TH1_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->th4_ls1, arg->th5_ls1),
			 ISP3X_RAWAWB_YUV_INTERP_CURVE_TH2_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->coor_x1_ls2_y, arg->vec_x21_ls2_y),
			 ISP3X_RAWAWB_YUV_X_COOR_Y_2);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->coor_x1_ls2_u, arg->vec_x21_ls2_u),
			 ISP3X_RAWAWB_YUV_X_COOR_U_2);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->coor_x1_ls2_v, arg->vec_x21_ls2_v),
			 ISP3X_RAWAWB_YUV_X_COOR_V_2);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->dis_x1x2_ls2, 0,
					arg->rotu0_ls2, arg->rotu1_ls2),
			 ISP3X_RAWAWB_YUV_X1X2_DIS_2);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->rotu2_ls2, arg->rotu3_ls2,
					arg->rotu4_ls2, arg->rotu5_ls2),
			 ISP3X_RAWAWB_YUV_INTERP_CURVE_UCOOR_2);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->th0_ls2, arg->th1_ls2),
			 ISP3X_RAWAWB_YUV_INTERP_CURVE_TH0_2);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->th2_ls2, arg->th3_ls2),
			 ISP3X_RAWAWB_YUV_INTERP_CURVE_TH1_2);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->th4_ls2, arg->th5_ls2),
			 ISP3X_RAWAWB_YUV_INTERP_CURVE_TH2_2);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->coor_x1_ls3_y,
					 arg->vec_x21_ls3_y),
			 ISP3X_RAWAWB_YUV_X_COOR_Y_3);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->coor_x1_ls3_u,
					 arg->vec_x21_ls3_u),
			 ISP3X_RAWAWB_YUV_X_COOR_U_3);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->coor_x1_ls3_v,
					 arg->vec_x21_ls3_v),
			 ISP3X_RAWAWB_YUV_X_COOR_V_3);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->dis_x1x2_ls3, 0,
					arg->rotu0_ls3, arg->rotu1_ls3),
			 ISP3X_RAWAWB_YUV_X1X2_DIS_3);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->rotu2_ls3, arg->rotu3_ls3,
					arg->rotu4_ls3, arg->rotu5_ls3),
			 ISP3X_RAWAWB_YUV_INTERP_CURVE_UCOOR_3);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->th0_ls3, arg->th1_ls3),
			 ISP3X_RAWAWB_YUV_INTERP_CURVE_TH0_3);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->th2_ls3, arg->th3_ls3),
			 ISP3X_RAWAWB_YUV_INTERP_CURVE_TH1_3);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->th4_ls3, arg->th5_ls3),
			 ISP3X_RAWAWB_YUV_INTERP_CURVE_TH2_3);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->wt0, arg->wt1),
			 ISP3X_RAWAWB_RGB2XY_WT01);

	isp3_param_write(params_vdev, arg->wt2,
			 ISP3X_RAWAWB_RGB2XY_WT2);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->mat0_x, arg->mat0_y),
			 ISP3X_RAWAWB_RGB2XY_MAT0_XY);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->mat1_x, arg->mat1_y),
			 ISP3X_RAWAWB_RGB2XY_MAT1_XY);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->mat2_x, arg->mat2_y),
			 ISP3X_RAWAWB_RGB2XY_MAT2_XY);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->nor_x0_0, arg->nor_x1_0),
			 ISP3X_RAWAWB_XY_DETC_NOR_X_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->nor_y0_0, arg->nor_y1_0),
			 ISP3X_RAWAWB_XY_DETC_NOR_Y_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->big_x0_0, arg->big_x1_0),
			 ISP3X_RAWAWB_XY_DETC_BIG_X_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->big_y0_0, arg->big_y1_0),
			 ISP3X_RAWAWB_XY_DETC_BIG_Y_0);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->nor_x0_1, arg->nor_x1_1),
			 ISP3X_RAWAWB_XY_DETC_NOR_X_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->nor_y0_1, arg->nor_y1_1),
			 ISP3X_RAWAWB_XY_DETC_NOR_Y_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->big_x0_1, arg->big_x1_1),
			 ISP3X_RAWAWB_XY_DETC_BIG_X_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->big_y0_1, arg->big_y1_1),
			 ISP3X_RAWAWB_XY_DETC_BIG_Y_1);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->nor_x0_2, arg->nor_x1_2),
			 ISP3X_RAWAWB_XY_DETC_NOR_X_2);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->nor_y0_2, arg->nor_y1_2),
			 ISP3X_RAWAWB_XY_DETC_NOR_Y_2);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->big_x0_2, arg->big_x1_2),
			 ISP3X_RAWAWB_XY_DETC_BIG_X_2);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->big_y0_2, arg->big_y1_2),
			 ISP3X_RAWAWB_XY_DETC_BIG_Y_2);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->nor_x0_3, arg->nor_x1_3),
			 ISP3X_RAWAWB_XY_DETC_NOR_X_3);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->nor_y0_3, arg->nor_y1_3),
			 ISP3X_RAWAWB_XY_DETC_NOR_Y_3);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->big_x0_3, arg->big_x1_3),
			 ISP3X_RAWAWB_XY_DETC_BIG_X_3);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->big_y0_3, arg->big_y1_3),
			 ISP3X_RAWAWB_XY_DETC_BIG_Y_3);

	value = (arg->exc_wp_region0_excen & 0x3) |
		!!arg->exc_wp_region0_measen << 2 |
		!!arg->exc_wp_region0_domain << 3 |
		(arg->exc_wp_region1_excen & 0x3) << 4 |
		!!arg->exc_wp_region1_measen << 6 |
		!!arg->exc_wp_region1_domain << 7 |
		(arg->exc_wp_region2_excen & 0x3) << 8 |
		!!arg->exc_wp_region2_measen << 10 |
		!!arg->exc_wp_region2_domain << 11 |
		(arg->exc_wp_region3_excen & 0x3) << 12 |
		!!arg->exc_wp_region3_measen << 14 |
		!!arg->exc_wp_region3_domain << 15 |
		(arg->exc_wp_region4_excen & 0x3) << 16 |
		!!arg->exc_wp_region4_domain << 19 |
		(arg->exc_wp_region5_excen & 0x3) << 20 |
		!!arg->exc_wp_region5_domain << 23 |
		(arg->exc_wp_region6_excen & 0x3) << 24 |
		!!arg->exc_wp_region6_domain << 27 |
		!!arg->multiwindow_en << 31;
	isp3_param_write(params_vdev, value, ISP3X_RAWAWB_MULTIWINDOW_EXC_CTRL);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->multiwindow0_h_offs,
					 arg->multiwindow0_v_offs),
			 ISP3X_RAWAWB_MULTIWINDOW0_OFFS);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->multiwindow0_h_size,
					 arg->multiwindow0_v_size),
			 ISP3X_RAWAWB_MULTIWINDOW0_SIZE);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->multiwindow1_h_offs,
					 arg->multiwindow1_v_offs),
			 ISP3X_RAWAWB_MULTIWINDOW1_OFFS);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->multiwindow1_h_size,
					 arg->multiwindow1_v_size),
			 ISP3X_RAWAWB_MULTIWINDOW1_SIZE);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->multiwindow2_h_offs,
					 arg->multiwindow2_v_offs),
			 ISP3X_RAWAWB_MULTIWINDOW2_OFFS);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->multiwindow2_h_size,
					 arg->multiwindow2_v_size),
			 ISP3X_RAWAWB_MULTIWINDOW2_SIZE);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->multiwindow3_h_offs,
					 arg->multiwindow3_v_offs),
			 ISP3X_RAWAWB_MULTIWINDOW3_OFFS);
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->multiwindow3_h_size,
					 arg->multiwindow3_v_size),
			 ISP3X_RAWAWB_MULTIWINDOW3_SIZE);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region0_xu0,
					 arg->exc_wp_region0_xu1),
			 ISP3X_RAWAWB_EXC_WP_REGION0_XU);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region0_yv0,
					 arg->exc_wp_region0_yv1),
			 ISP3X_RAWAWB_EXC_WP_REGION0_YV);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region1_xu0,
					 arg->exc_wp_region1_xu1),
			 ISP3X_RAWAWB_EXC_WP_REGION1_XU);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region1_yv0,
					 arg->exc_wp_region1_yv1),
			 ISP3X_RAWAWB_EXC_WP_REGION1_YV);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region2_xu0,
					 arg->exc_wp_region2_xu1),
			 ISP3X_RAWAWB_EXC_WP_REGION2_XU);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region2_yv0,
					 arg->exc_wp_region2_yv1),
			 ISP3X_RAWAWB_EXC_WP_REGION2_YV);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region3_xu0,
					 arg->exc_wp_region3_xu1),
			 ISP3X_RAWAWB_EXC_WP_REGION3_XU);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region3_yv0,
					 arg->exc_wp_region3_yv1),
			 ISP3X_RAWAWB_EXC_WP_REGION3_YV);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region4_xu0,
					 arg->exc_wp_region4_xu1),
			 ISP3X_RAWAWB_EXC_WP_REGION4_XU);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region4_yv0,
					 arg->exc_wp_region4_yv1),
			 ISP3X_RAWAWB_EXC_WP_REGION4_YV);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region5_xu0,
					 arg->exc_wp_region5_xu1),
			 ISP3X_RAWAWB_EXC_WP_REGION5_XU);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region5_yv0,
					 arg->exc_wp_region5_yv1),
			 ISP3X_RAWAWB_EXC_WP_REGION5_YV);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region6_xu0,
					 arg->exc_wp_region6_xu1),
			 ISP3X_RAWAWB_EXC_WP_REGION6_XU);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->exc_wp_region6_yv0,
					 arg->exc_wp_region6_yv1),
			 ISP3X_RAWAWB_EXC_WP_REGION6_YV);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->exc_wp_region0_weight,
					arg->exc_wp_region1_weight,
					arg->exc_wp_region2_weight,
					arg->exc_wp_region3_weight),
			 ISP32_RAWAWB_EXC_WP_WEIGHT0_3);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->exc_wp_region4_weight,
					arg->exc_wp_region5_weight,
					arg->exc_wp_region6_weight, 0),
			 ISP32_RAWAWB_EXC_WP_WEIGHT4_6);

	if (params_vdev->dev->hw_dev->is_single)
		isp_rawawb_cfg_sram(params_vdev, arg, false);
	else
		memcpy(arg_rec->wp_blk_wei_w, arg->wp_blk_wei_w,
		       ISP32_RAWAWB_WEIGHT_NUM);

	/* avoid to override the old enable value */
	value = isp3_param_read_cache(params_vdev, ISP3X_RAWAWB_CTRL);
	value &= (ISP32_MODULE_EN |
		  ISP32_RAWAWB_2DDR_PATH_EN |
		  ISP32_RAWAWB_2DDR_PATH_DS);
	value |= !!arg->low12bit_val << 28 |
		 !!arg->yuv3d_en1 << 26 |
		 !!arg->xy_en1 << 25 |
		 !!arg->uv_en1 << 24 |
		 (arg->light_num & 0x7) << 20 |
		 !!arg->rawlsc_bypass_en << 19 |
		 !!arg->wind_size << 18 |
		 !!arg->in_overexposure_check_en << 17 |
		 !!arg->in_rshift_to_12bit_en << 16 |
		 (arg->yuv3d_ls_idx3 & 0x7) << 13 |
		 (arg->yuv3d_ls_idx2 & 0x7) << 10 |
		 (arg->yuv3d_ls_idx1 & 0x7) << 7 |
		 (arg->yuv3d_ls_idx0 & 0x7) << 4 |
		 !!arg->yuv3d_en0 << 3 |
		 !!arg->xy_en0 << 2 |
		 !!arg->uv_en0 << 1;
	isp3_param_write(params_vdev, value, ISP3X_RAWAWB_CTRL);

	mask = ISP32_DRC2AWB_SEL | ISP32_BNR2AWB_SEL | ISP3X_RAWAWB_SEL(3);
	val = ISP3X_RAWAWB_SEL(arg->rawawb_sel) |
	      (arg->bnr2awb_sel & 0x1) << 26 | (arg->drc2awb_sel & 0x1) << 27;
	value = isp3_param_read(params_vdev, ISP3X_VI_ISP_PATH);
	if ((value & mask) != val) {
		value &= ~mask;
		value |= val;
		isp3_param_write(params_vdev, value, ISP3X_VI_ISP_PATH);
	}
}

static void
isp_rawawb_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 awb_ctrl;

	awb_ctrl = isp3_param_read_cache(params_vdev, ISP3X_RAWAWB_CTRL);
	awb_ctrl &= ~ISP32_REG_WR_MASK;
	if (en)
		awb_ctrl |= ISP32_MODULE_EN;
	else
		awb_ctrl &= ~ISP32_MODULE_EN;

	isp3_param_write(params_vdev, awb_ctrl, ISP3X_RAWAWB_CTRL);
}

static void
isp_rawhstlite_config(struct rkisp_isp_params_vdev *params_vdev,
		      const struct isp2x_rawhistlite_cfg *arg)
{
	u32 i;
	u32 value;
	u32 hist_ctrl;
	u32 block_hsize, block_vsize;

	/* avoid to override the old enable value */
	hist_ctrl = isp3_param_read(params_vdev, ISP3X_RAWHIST_LITE_CTRL);
	hist_ctrl &= ISP3X_RAWHIST_EN;
	hist_ctrl = hist_ctrl |
		    ISP3X_RAWHIST_MODE(arg->mode) |
		    ISP3X_RAWHIST_DATASEL(arg->data_sel) |
		    ISP3X_RAWHIST_WATERLINE(arg->waterline) |
		    ISP3X_RAWHIST_STEPSIZE(arg->stepsize);
	isp3_param_write(params_vdev, hist_ctrl, ISP3X_RAWHIST_LITE_CTRL);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->win.h_offs, arg->win.v_offs),
			 ISP3X_RAWHIST_LITE_OFFS);

	block_hsize = arg->win.h_size / ISP32_RAWHISTLITE_ROW_NUM - 1;
	block_vsize = arg->win.v_size / ISP32_RAWHISTLITE_COLUMN_NUM - 1;
	block_hsize &= 0xFFFE;
	block_vsize &= 0xFFFE;
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(block_hsize, block_vsize),
			 ISP3X_RAWHIST_LITE_SIZE);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->rcc, arg->gcc, arg->bcc, arg->off),
			 ISP3X_RAWHIST_LITE_RAW2Y_CC);

	for (i = 0; i < (ISP32_RAWHISTLITE_WEIGHT_REG_SIZE / 4); i++) {
		value = ISP_PACK_4BYTE(arg->weight[4 * i + 0],
				       arg->weight[4 * i + 1],
				       arg->weight[4 * i + 2],
				       arg->weight[4 * i + 3]);
		isp3_param_write(params_vdev, value,
				 ISP3X_RAWHIST_LITE_WEIGHT + 4 * i);
	}

	value = ISP_PACK_4BYTE(arg->weight[4 * i + 0], 0, 0, 0);
	isp3_param_write(params_vdev, value,
			 ISP3X_RAWHIST_LITE_WEIGHT + 4 * i);
}

static void
isp_rawhstlite_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 hist_ctrl;

	hist_ctrl = isp3_param_read(params_vdev, ISP3X_RAWHIST_LITE_CTRL);
	hist_ctrl &= ~(ISP32_MODULE_EN | ISP32_REG_WR_MASK);

	if (en)
		hist_ctrl |= ISP32_MODULE_EN;

	isp3_param_write(params_vdev, hist_ctrl, ISP3X_RAWHIST_LITE_CTRL);
}

static void
isp_rawhstbig_cfg_sram(struct rkisp_isp_params_vdev *params_vdev,
		       const struct isp2x_rawhistbig_cfg *arg,
		       u32 blk_no, bool is_check)
{
	u32 i, j, wnd_num_idx, value;
	u8 weight15x15[ISP32_RAWHISTBIG_WEIGHT_REG_SIZE];
	const u32 hist_wnd_num[] = {5, 5, 15, 15};
	u32 addr;

	switch (blk_no) {
	case 1:
		addr = ISP3X_RAWHIST_BIG2_BASE;
		break;
	case 2:
		addr = ISP3X_RAWHIST_BIG3_BASE;
		break;
	case 0:
	default:
		addr = ISP3X_RAWHIST_BIG1_BASE;
		break;
	}

	value = ISP3X_RAWHIST_EN;
	if (is_check &&
	    !(isp3_param_read(params_vdev, addr + ISP3X_RAWHIST_BIG_CTRL) & value))
		return;

	wnd_num_idx = arg->wnd_num;
	memset(weight15x15, 0, sizeof(weight15x15));
	for (i = 0; i < hist_wnd_num[wnd_num_idx]; i++) {
		for (j = 0; j < hist_wnd_num[wnd_num_idx]; j++) {
			weight15x15[i * ISP32_RAWHISTBIG_ROW_NUM + j] =
				arg->weight[i * hist_wnd_num[wnd_num_idx] + j];
		}
	}

	for (i = 0; i < (ISP32_RAWHISTBIG_WEIGHT_REG_SIZE / 5); i++) {
		value = (weight15x15[5 * i + 0] & 0x3f) |
			(weight15x15[5 * i + 1] & 0x3f) << 6 |
			(weight15x15[5 * i + 2] & 0x3f) << 12 |
			(weight15x15[5 * i + 3] & 0x3f) << 18 |
			(weight15x15[5 * i + 4] & 0x3f) << 24;
		isp3_param_write_direct(params_vdev, value,
					addr + ISP3X_RAWHIST_BIG_WEIGHT_BASE);
	}
}

static void
isp_rawhstbig_config(struct rkisp_isp_params_vdev *params_vdev,
		     const struct isp2x_rawhistbig_cfg *arg, u32 blk_no)
{
	struct isp32_isp_params_cfg *params_rec = params_vdev->isp32_params;
	struct rkisp_device *dev = params_vdev->dev;
	struct isp2x_rawhistbig_cfg *arg_rec;
	u32 hist_ctrl, block_hsize, block_vsize, wnd_num_idx;
	const u32 hist_wnd_num[] = {5, 5, 15, 15};
	u32 addr;

	switch (blk_no) {
	case 1:
		addr = ISP3X_RAWHIST_BIG2_BASE;
		arg_rec = &params_rec->meas.rawhist1;
		break;
	case 2:
		addr = ISP3X_RAWHIST_BIG3_BASE;
		arg_rec = &params_rec->meas.rawhist2;
		break;
	case 0:
	default:
		addr = ISP3X_RAWHIST_BIG1_BASE;
		arg_rec = &params_rec->meas.rawhist3;
		break;
	}

	wnd_num_idx = arg->wnd_num;
	/* avoid to override the old enable value */
	hist_ctrl = isp3_param_read(params_vdev, addr + ISP3X_RAWHIST_BIG_CTRL);
	hist_ctrl &= ISP3X_RAWHIST_EN;
	hist_ctrl = hist_ctrl |
		    ISP3X_RAWHIST_MODE(arg->mode) |
		    ISP3X_RAWHIST_DATASEL(arg->data_sel) |
		    ISP3X_RAWHIST_WATERLINE(arg->waterline) |
		    ISP3X_RAWHIST_WND_NUM(arg->wnd_num) |
		    ISP3X_RAWHIST_STEPSIZE(arg->stepsize);
	isp3_param_write(params_vdev, hist_ctrl, addr + ISP3X_RAWHIST_BIG_CTRL);

	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(arg->win.h_offs, arg->win.v_offs),
			 addr + ISP3X_RAWHIST_BIG_OFFS);

	block_hsize = arg->win.h_size / hist_wnd_num[wnd_num_idx] - 1;
	block_vsize = arg->win.v_size / hist_wnd_num[wnd_num_idx] - 1;
	block_hsize &= 0xFFFE;
	block_vsize &= 0xFFFE;
	isp3_param_write(params_vdev,
			 ISP_PACK_2SHORT(block_hsize, block_vsize),
			 addr + ISP3X_RAWHIST_BIG_SIZE);

	isp3_param_write(params_vdev,
			 ISP_PACK_4BYTE(arg->rcc, arg->gcc, arg->bcc, arg->off),
			 addr + ISP3X_RAWHIST_BIG_RAW2Y_CC);

	if (dev->hw_dev->is_single)
		isp_rawhstbig_cfg_sram(params_vdev, arg, blk_no, false);
	else
		*arg_rec = *arg;
}

static void
isp_rawhstbig_enable(struct rkisp_isp_params_vdev *params_vdev,
		     bool en, u32 blk_no)
{
	u32 hist_ctrl;
	u32 addr;

	switch (blk_no) {
	case 1:
		addr = ISP3X_RAWHIST_BIG2_BASE;
		break;
	case 2:
		addr = ISP3X_RAWHIST_BIG3_BASE;
		break;
	case 0:
	default:
		addr = ISP3X_RAWHIST_BIG1_BASE;
		break;
	}

	hist_ctrl = isp3_param_read(params_vdev, addr + ISP3X_RAWHIST_BIG_CTRL);
	hist_ctrl &= ~(ISP3X_RAWHIST_EN | ISP32_REG_WR_MASK);
	if (en)
		hist_ctrl |= ISP3X_RAWHIST_EN;

	isp3_param_write(params_vdev, hist_ctrl, addr + ISP3X_RAWHIST_BIG_CTRL);
}

static void
isp_rawhst1_config(struct rkisp_isp_params_vdev *params_vdev,
		   const struct isp2x_rawhistbig_cfg *arg)
{
	isp_rawhstbig_config(params_vdev, arg, 1);
}

static void
isp_rawhst1_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	isp_rawhstbig_enable(params_vdev, en, 1);
}

static void
isp_rawhst2_config(struct rkisp_isp_params_vdev *params_vdev,
		   const struct isp2x_rawhistbig_cfg *arg)
{
	isp_rawhstbig_config(params_vdev, arg, 2);
}

static void
isp_rawhst2_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	isp_rawhstbig_enable(params_vdev, en, 2);
}

static void
isp_rawhst3_config(struct rkisp_isp_params_vdev *params_vdev,
		   const struct isp2x_rawhistbig_cfg *arg)
{
	isp_rawhstbig_config(params_vdev, arg, 0);
}

static void
isp_rawhst3_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	isp_rawhstbig_enable(params_vdev, en, 0);
}

static void
isp_hdrmge_config(struct rkisp_isp_params_vdev *params_vdev,
		  const struct isp32_hdrmge_cfg *arg,
		  enum rkisp_params_type type)
{
	u32 value;
	int i;

	if (type == RKISP_PARAMS_SHD || type == RKISP_PARAMS_ALL) {
		value = ISP_PACK_2SHORT(arg->gain0, arg->gain0_inv);
		isp3_param_write(params_vdev, value, ISP3X_HDRMGE_GAIN0);

		value = ISP_PACK_2SHORT(arg->gain1, arg->gain1_inv);
		isp3_param_write(params_vdev, value, ISP3X_HDRMGE_GAIN1);

		value = arg->gain2;
		isp3_param_write(params_vdev, value, ISP3X_HDRMGE_GAIN2);

		value = isp3_param_read_cache(params_vdev, ISP3X_HDRMGE_CTRL);
		if (arg->s_base)
			value |= BIT(1);
		else
			value &= ~BIT(1);
		if (arg->each_raw_en)
			value |= BIT(6);
		else
			value &= ~BIT(6);
		isp3_param_write(params_vdev, value, ISP3X_HDRMGE_CTRL);
	}

	if (type == RKISP_PARAMS_IMD || type == RKISP_PARAMS_ALL) {
		value = ISP_PACK_4BYTE(arg->ms_dif_0p8, arg->ms_diff_0p15,
				       arg->lm_dif_0p9, arg->lm_dif_0p15);
		isp3_param_write(params_vdev, value, ISP3X_HDRMGE_LIGHTZ);
		value = (arg->ms_scl & 0x7ff) |
			(arg->ms_thd0 & 0x3ff) << 12 |
			(arg->ms_thd1 & 0x3ff) << 22;
		isp3_param_write(params_vdev, value, ISP3X_HDRMGE_MS_DIFF);
		value = (arg->lm_scl & 0x7ff) |
			(arg->lm_thd0 & 0x3ff) << 12 |
			(arg->lm_thd1 & 0x3ff) << 22;
		isp3_param_write(params_vdev, value, ISP3X_HDRMGE_LM_DIFF);

		for (i = 0; i < ISP32_HDRMGE_L_CURVE_NUM; i++) {
			value = ISP_PACK_2SHORT(arg->curve.curve_0[i], arg->curve.curve_1[i]);
			isp3_param_write(params_vdev, value, ISP3X_HDRMGE_DIFF_Y0 + 4 * i);
		}

		for (i = 0; i < ISP32_HDRMGE_E_CURVE_NUM; i++) {
			value = (arg->l_raw1[i] & 0x3ff) << 20 |
				(arg->l_raw0[i] & 0x3ff) << 10 |
				(arg->e_y[i] & 0x3ff);
			isp3_param_write(params_vdev, value, ISP3X_HDRMGE_OVER_Y0 + 4 * i);
		}

		value = ISP_PACK_2SHORT(arg->each_raw_gain0, arg->each_raw_gain1);
		isp3_param_write(params_vdev, value, ISP32_HDRMGE_EACH_GAIN);
	}
}

static void
isp_hdrmge_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
}

static void
isp_hdrdrc_config(struct rkisp_isp_params_vdev *params_vdev,
		  const struct isp32_drc_cfg *arg,
		  enum rkisp_params_type type)
{
	u32 i, value;

	if (type == RKISP_PARAMS_IMD)
		return;

	value = (arg->offset_pow2 & 0x0F) << 28 |
		(arg->compres_scl & 0x1FFF) << 14 |
		(arg->position & 0x03FFF);
	isp3_param_write(params_vdev, value, ISP3X_DRC_CTRL1);

	value = arg->delta_scalein << 24 |
		(arg->hpdetail_ratio & 0xFFF) << 12 |
		(arg->lpdetail_ratio & 0xFFF);
	isp3_param_write(params_vdev, value, ISP3X_DRC_LPRATIO);

	value = ISP_PACK_4BYTE(arg->bilat_wt_off, 0, arg->weipre_frame, arg->weicur_pix);
	isp3_param_write(params_vdev, value, ISP3X_DRC_EXPLRATIO);

	value = (arg->force_sgm_inv0 & 0xFFFF) << 16 |
		arg->motion_scl << 8 | arg->edge_scl;
	isp3_param_write(params_vdev, value, ISP3X_DRC_SIGMA);

	value = ISP_PACK_2SHORT(arg->space_sgm_inv0, arg->space_sgm_inv1);
	isp3_param_write(params_vdev, value, ISP3X_DRC_SPACESGM);

	value = ISP_PACK_2SHORT(arg->range_sgm_inv0, arg->range_sgm_inv1);
	isp3_param_write(params_vdev, value, ISP3X_DRC_RANESGM);

	value = (arg->weig_bilat & 0x1f) | (arg->weig_maxl & 0x1f) << 8 |
		(arg->bilat_soft_thd & 0x3fff) << 16;
	if (arg->enable_soft_thd)
		value |= BIT(31);
	isp3_param_write(params_vdev, value, ISP3X_DRC_BILAT);

	for (i = 0; i < ISP32_DRC_Y_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->gain_y[2 * i],
					arg->gain_y[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_DRC_GAIN_Y0 + 4 * i);
	}
	value = ISP_PACK_2SHORT(arg->gain_y[2 * i], 0);
	isp3_param_write(params_vdev, value, ISP3X_DRC_GAIN_Y0 + 4 * i);

	for (i = 0; i < ISP32_DRC_Y_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->compres_y[2 * i],
					arg->compres_y[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_DRC_COMPRES_Y0 + 4 * i);
	}
	value = ISP_PACK_2SHORT(arg->compres_y[2 * i], 0);
	isp3_param_write(params_vdev, value, ISP3X_DRC_COMPRES_Y0 + 4 * i);

	for (i = 0; i < ISP32_DRC_Y_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->scale_y[2 * i],
					arg->scale_y[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_DRC_SCALE_Y0 + 4 * i);
	}
	value = ISP_PACK_2SHORT(arg->scale_y[2 * i], 0);
	isp3_param_write(params_vdev, value, ISP3X_DRC_SCALE_Y0 + 4 * i);

	value = ISP_PACK_2SHORT(arg->min_ogain, arg->iir_weight);
	isp3_param_write(params_vdev, value, ISP3X_DRC_IIRWG_GAIN);

	value = arg->gas_t & 0x1fff;
	isp3_param_write(params_vdev, value, ISP32_DRC_LUM3X2_CTRL);

	value = ISP_PACK_4BYTE(arg->gas_l0, arg->gas_l1, arg->gas_l2, arg->gas_l3);
	isp3_param_write(params_vdev, value, ISP32_DRC_LUM3X2_GAS);
}

static void
isp_hdrdrc_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 value;
	bool real_en;

	value = isp3_param_read(params_vdev, ISP3X_DRC_CTRL0);
	real_en = !!(value & ISP32_MODULE_EN);
	if ((en && real_en) || (!en && !real_en))
		return;

	if (en) {
		value |= ISP32_MODULE_EN;
		isp3_param_set_bits(params_vdev, ISP3X_ISP_CTRL1,
				    ISP3X_ADRC_FST_FRAME);
	} else {
		value = 0;
		isp3_param_clear_bits(params_vdev, ISP3X_GAIN_CTRL, BIT(12));
	}
	isp3_param_write(params_vdev, value, ISP3X_DRC_CTRL0);
}

static void
isp_gic_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp21_gic_cfg *arg)
{
	u32 value;
	s32 i;

	value = (arg->regmingradthrdark2 & 0x03FF) << 20 |
		(arg->regmingradthrdark1 & 0x03FF) << 10 |
		(arg->regminbusythre & 0x03FF);
	isp3_param_write(params_vdev, value, ISP3X_GIC_DIFF_PARA1);

	value = (arg->regdarkthre & 0x07FF) << 21 |
		(arg->regmaxcorvboth & 0x03FF) << 11 |
		(arg->regdarktthrehi & 0x07FF);
	isp3_param_write(params_vdev, value, ISP3X_GIC_DIFF_PARA2);

	value = (arg->regkgrad2dark & 0x0F) << 28 |
		(arg->regkgrad1dark & 0x0F) << 24 |
		(arg->regstrengthglobal_fix & 0xFF) << 16 |
		(arg->regdarkthrestep & 0x0F) << 12 |
		(arg->regkgrad2 & 0x0F) << 8 |
		(arg->regkgrad1 & 0x0F) << 4 |
		(arg->reggbthre & 0x0F);
	isp3_param_write(params_vdev, value, ISP3X_GIC_DIFF_PARA3);

	value = (arg->regmaxcorv & 0x03FF) << 20 |
		(arg->regmingradthr2 & 0x03FF) << 10 |
		(arg->regmingradthr1 & 0x03FF);
	isp3_param_write(params_vdev, value, ISP3X_GIC_DIFF_PARA4);

	value = (arg->gr_ratio & 0x03) << 28 |
		(arg->noise_scale & 0x7F) << 12 |
		(arg->noise_base & 0xFFF);
	isp3_param_write(params_vdev, value, ISP3X_GIC_NOISE_PARA1);

	value = arg->diff_clip & 0x7fff;
	isp3_param_write(params_vdev, value, ISP3X_GIC_NOISE_PARA2);

	for (i = 0; i < ISP32_GIC_SIGMA_Y_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->sigma_y[2 * i], arg->sigma_y[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_GIC_SIGMA_VALUE0 + 4 * i);
	}
	value = ISP_PACK_2SHORT(arg->sigma_y[2 * i], 0);
	isp3_param_write(params_vdev, value, ISP3X_GIC_SIGMA_VALUE0 + 4 * i);
}

static void
isp_gic_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 value = 0;

	if (en)
		value |= ISP32_MODULE_EN;
	isp3_param_write(params_vdev, value, ISP3X_GIC_CONTROL);
}

static void
isp_dhaz_config(struct rkisp_isp_params_vdev *params_vdev,
		const struct isp32_dhaz_cfg *arg)
{
	u32 i, value, ctrl;

	ctrl = isp3_param_read(params_vdev, ISP3X_DHAZ_CTRL);
	ctrl &= ISP3X_DHAZ_ENMUX;

	ctrl |= !!arg->enh_luma_en << 28 | !!arg->color_deviate_en << 27 |
		!!arg->round_en << 26 | !!arg->soft_wr_en << 25 |
		!!arg->enhance_en << 20 | !!arg->air_lc_en << 16 |
		!!arg->hpara_en << 12 | !!arg->hist_en << 8 |
		!!arg->dc_en << 4;
	/* merge dual unite isp params at frame end */
	if (arg->soft_wr_en) {
		for (i = 0; i < ISP32_DHAZ_HIST_WR_NUM / 3; i++) {
			value = (arg->hist_wr[i * 3] & 0x3ff) |
				(arg->hist_wr[i * 3 + 1] & 0x3ff) << 10 |
				(arg->hist_wr[i * 3 + 2] & 0x3ff) << 20;
			isp3_param_write(params_vdev, value, ISP3X_DHAZ_HIST_WR0 + i * 4);
		}
		value = arg->hist_wr[i * 3] & 0x3ff;
		isp3_param_write(params_vdev, value, ISP3X_DHAZ_HIST_WR0 + i * 4);
	}
	isp3_param_write(params_vdev, ctrl, ISP3X_DHAZ_CTRL);

	value = ISP_PACK_4BYTE(arg->dc_min_th, arg->dc_max_th,
			       arg->yhist_th, arg->yblk_th);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_ADP0);

	value = ISP_PACK_4BYTE(arg->bright_min, arg->bright_max,
			       arg->wt_max, 0);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_ADP1);

	value = ISP_PACK_4BYTE(arg->air_min, arg->air_max,
			       arg->dark_th, arg->tmax_base);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_ADP2);

	value = ISP_PACK_2SHORT(arg->tmax_off, arg->tmax_max);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_ADP_TMAX);

	value = (arg->hist_min & 0xFFFF) << 16 |
		(arg->hist_th_off & 0xFF) << 8 |
		(arg->hist_k & 0x1F);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_ADP_HIST0);

	value = ISP_PACK_2SHORT(arg->hist_scale, arg->hist_gratio);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_ADP_HIST1);

	value = ISP_PACK_2SHORT(arg->enhance_chroma, arg->enhance_value);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_ENHANCE);

	value = (arg->iir_wt_sigma & 0x07FF) << 16 |
		(arg->iir_sigma & 0xFF) << 8 |
		(arg->stab_fnum & 0x1F);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_IIR0);

	value = (arg->iir_pre_wet & 0x0F) << 24 |
		(arg->iir_tmax_sigma & 0x7FF) << 8 |
		(arg->iir_air_sigma & 0xFF);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_IIR1);

	value = (arg->cfg_wt & 0x01FF) << 16 |
		(arg->cfg_air & 0xFF) << 8 |
		(arg->cfg_alpha & 0xFF);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_SOFT_CFG0);

	value = ISP_PACK_2SHORT(arg->cfg_tmax, arg->cfg_gratio);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_SOFT_CFG1);

	value = (arg->range_sima & 0x01FF) << 16 |
		(arg->space_sigma_pre & 0xFF) << 8 |
		(arg->space_sigma_cur & 0xFF);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_BF_SIGMA);

	value = ISP_PACK_2SHORT(arg->bf_weight, arg->dc_weitcur);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_BF_WET);

	for (i = 0; i < ISP32_DHAZ_ENH_CURVE_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->enh_curve[2 * i], arg->enh_curve[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_DHAZ_ENH_CURVE0 + 4 * i);
	}
	value = ISP_PACK_2SHORT(arg->enh_curve[2 * i], 0);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_ENH_CURVE0 + 4 * i);

	value = ISP_PACK_4BYTE(arg->gaus_h0, arg->gaus_h1, arg->gaus_h2, 0);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_GAUS);

	for (i = 0; i < ISP32_DHAZ_SIGMA_IDX_NUM / 4; i++) {
		value = ISP_PACK_4BYTE(arg->sigma_idx[i * 4], arg->sigma_idx[i * 4 + 1],
					arg->sigma_idx[i * 4 + 2], arg->sigma_idx[i * 4 + 3]);
		isp3_param_write(params_vdev, value, ISP3X_DHAZ_GAIN_IDX0 + i * 4);
	}
	value = ISP_PACK_4BYTE(arg->sigma_idx[i * 4], arg->sigma_idx[i * 4 + 1],
				arg->sigma_idx[i * 4 + 2], 0);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_GAIN_IDX0 + i * 4);

	for (i = 0; i < ISP32_DHAZ_SIGMA_LUT_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->sigma_lut[i * 2], arg->sigma_lut[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP3X_DHAZ_GAIN_LUT0 + i * 4);
	}
	value = ISP_PACK_2SHORT(arg->sigma_lut[i * 2], 0);
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_GAIN_LUT0 + i * 4);

	for (i = 0; i < ISP32_DHAZ_ENH_LUMA_NUM / 3; i++) {
		value = (arg->enh_luma[i * 3 + 2] & 0x3ff) << 20 |
			(arg->enh_luma[i * 3 + 1] & 0x3ff) << 10 |
			(arg->enh_luma[i * 3] & 0x3ff);
		isp3_param_write(params_vdev, value, ISP32_DHAZ_ENH_LUMA0 + i * 4);
	}
	value = (arg->enh_luma[i * 3 + 1] & 0x3ff) << 10 |
		(arg->enh_luma[i * 3] & 0x3ff);
	isp3_param_write(params_vdev, value, ISP32_DHAZ_ENH_LUMA0 + i * 4);
}

static void
isp_dhaz_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 value;
	bool real_en;

	value = isp3_param_read(params_vdev, ISP3X_DHAZ_CTRL);
	real_en = !!(value & ISP3X_DHAZ_ENMUX);
	if ((en && real_en) || (!en && !real_en))
		return;

	if (en) {
		value |= ISP32_SELF_FORCE_UPD | ISP3X_DHAZ_ENMUX;
		isp3_param_set_bits(params_vdev, ISP3X_ISP_CTRL1,
				    ISP3X_DHAZ_FST_FRAME);
	} else {
		value &= ~ISP3X_DHAZ_ENMUX;
		isp3_param_clear_bits(params_vdev, ISP3X_GAIN_CTRL, BIT(16));
	}
	isp3_param_write(params_vdev, value, ISP3X_DHAZ_CTRL);
}

static void
isp_3dlut_config(struct rkisp_isp_params_vdev *params_vdev,
		 const struct isp2x_3dlut_cfg *arg)
{
	struct rkisp_device *dev = params_vdev->dev;
	struct rkisp_isp_params_val_v32 *priv_val;
	u32 value, buf_idx, i;
	u32 *data;

	priv_val = (struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;
	buf_idx = (priv_val->buf_3dlut_idx++) % ISP32_3DLUT_BUF_NUM;

	if (!priv_val->buf_3dlut[buf_idx].vaddr) {
		dev_err(dev->dev, "no find 3dlut buf\n");
		return;
	}
	data = (u32 *)priv_val->buf_3dlut[buf_idx].vaddr;
	for (i = 0; i < arg->actual_size; i++)
		data[i] = (arg->lut_b[i] & 0x3FF) |
			  (arg->lut_g[i] & 0xFFF) << 10 |
			  (arg->lut_r[i] & 0x3FF) << 22;
	rkisp_prepare_buffer(params_vdev->dev, &priv_val->buf_3dlut[buf_idx]);
	value = priv_val->buf_3dlut[buf_idx].dma_addr;
	isp3_param_write(params_vdev, value, ISP3X_MI_LUT_3D_RD_BASE);
	isp3_param_write(params_vdev, arg->actual_size, ISP3X_MI_LUT_3D_RD_WSIZE);

	value = isp3_param_read(params_vdev, ISP3X_3DLUT_CTRL);
	value &= ISP3X_3DLUT_EN;

	if (value)
		isp3_param_set_bits(params_vdev, ISP3X_3DLUT_UPDATE, 0x01);

	isp3_param_write(params_vdev, value, ISP3X_3DLUT_CTRL);
}

static void
isp_3dlut_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 value;
	bool en_state;
	struct rkisp_isp_params_val_v32 *priv_val;

	value = isp3_param_read(params_vdev, ISP3X_3DLUT_CTRL);
	en_state = (value & ISP3X_3DLUT_EN) ? true : false;

	if (en == en_state)
		return;

	priv_val = (struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;
	if (en && priv_val->buf_3dlut[0].vaddr) {
		isp3_param_set_bits(params_vdev, ISP3X_3DLUT_CTRL, 0x01);
		isp3_param_set_bits(params_vdev, ISP3X_3DLUT_UPDATE, 0x01);
	} else {
		isp3_param_clear_bits(params_vdev, ISP3X_3DLUT_CTRL, 0x01);
		isp3_param_clear_bits(params_vdev, ISP3X_3DLUT_UPDATE, 0x01);
		isp3_param_clear_bits(params_vdev, ISP3X_GAIN_CTRL, BIT(20));
	}
}

static void
isp_ldch_config(struct rkisp_isp_params_vdev *params_vdev,
		const struct isp32_ldch_cfg *arg)
{
	struct rkisp_device *dev = params_vdev->dev;
	struct rkisp_isp_params_val_v32 *priv_val;
	struct isp2x_mesh_head *head;
	int buf_idx, i;
	u32 value;

	value = isp3_param_read(params_vdev, ISP3X_LDCH_STS);
	value &= ISP32_MODULE_EN;
	value |= !!arg->map13p3_en << 7 |
		 !!arg->force_map_en << 6 |
		 !!arg->bic_mode_en << 4 |
		 !!arg->sample_avr_en << 3 |
		 !!arg->zero_interp_en << 2 |
		 !!arg->frm_end_dis << 1;
	isp3_param_write(params_vdev, value, ISP3X_LDCH_STS);
	if (arg->bic_mode_en) {
		for (i = 0; i < ISP32_LDCH_BIC_NUM / 4; i++) {
			value = ISP_PACK_4BYTE(arg->bicubic[i * 4], arg->bicubic[i * 4 + 1],
					arg->bicubic[i * 4 + 2], arg->bicubic[i * 4 + 3]);
			isp3_param_write(params_vdev, value, ISP32_LDCH_BIC_TABLE0 + i * 4);
		}
	}

	priv_val = (struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;
	for (i = 0; i < ISP32_MESH_BUF_NUM; i++) {
		if (!priv_val->buf_ldch[i].mem_priv)
			continue;
		if (arg->buf_fd == priv_val->buf_ldch[i].dma_fd)
			break;
	}
	if (i == ISP32_MESH_BUF_NUM) {
		dev_err(dev->dev, "cannot find ldch buf fd(%d)\n", arg->buf_fd);
		return;
	}

	if (!priv_val->buf_ldch[i].vaddr) {
		dev_err(dev->dev, "no ldch buffer allocated\n");
		return;
	}

	buf_idx = priv_val->buf_ldch_idx;
	head = (struct isp2x_mesh_head *)priv_val->buf_ldch[buf_idx].vaddr;
	head->stat = MESH_BUF_INIT;

	buf_idx = i;
	head = (struct isp2x_mesh_head *)priv_val->buf_ldch[buf_idx].vaddr;
	head->stat = MESH_BUF_CHIPINUSE;
	priv_val->buf_ldch_idx = buf_idx;
	rkisp_prepare_buffer(dev, &priv_val->buf_ldch[buf_idx]);
	value = priv_val->buf_ldch[buf_idx].dma_addr + head->data_oft;
	isp3_param_write(params_vdev, value, ISP3X_MI_LUT_LDCH_RD_BASE);
	isp3_param_write(params_vdev, arg->hsize, ISP3X_MI_LUT_LDCH_RD_H_WSIZE);
	isp3_param_write(params_vdev, arg->vsize, ISP3X_MI_LUT_LDCH_RD_V_SIZE);
}

static void
isp_ldch_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	struct rkisp_device *dev = params_vdev->dev;
	struct rkisp_isp_params_val_v32 *priv_val;
	u32 buf_idx;

	priv_val = (struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;
	if (en) {
		buf_idx = priv_val->buf_ldch_idx;
		if (!priv_val->buf_ldch[buf_idx].vaddr) {
			dev_err(dev->dev, "no ldch buffer allocated\n");
			return;
		}
		isp3_param_set_bits(params_vdev, ISP3X_LDCH_STS, 0x01);
	} else {
		isp3_param_clear_bits(params_vdev, ISP3X_LDCH_STS, 0x01);
	}
}

static void
isp_ynr_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp32_ynr_cfg *arg)
{
	u32 i, value;

	value = isp3_param_read(params_vdev, ISP3X_YNR_GLOBAL_CTRL);
	value &= ISP32_MODULE_EN;

	value |= !!arg->rnr_en << 26 |
		 !!arg->thumb_mix_cur_en << 24 |
		 (arg->global_gain_alpha & 0xF) << 20 |
		 (arg->global_gain & 0x3FF) << 8 |
		 (arg->flt1x1_bypass_sel & 0x3) << 6 |
		 !!arg->nlm11x11_bypass << 5 |
		 !!arg->flt1x1_bypass << 4 |
		 !!arg->lgft3x3_bypass << 3 |
		 !!arg->lbft5x5_bypass << 2 |
		 !!arg->bft3x3_bypass << 1;
	isp3_param_write(params_vdev, value, ISP3X_YNR_GLOBAL_CTRL);

	value = ISP_PACK_2SHORT(arg->rnr_max_r, arg->local_gainscale);
	isp3_param_write(params_vdev, value, ISP3X_YNR_RNR_MAX_R);

	value = ISP_PACK_2SHORT(arg->rnr_center_coorh, arg->rnr_center_coorv);
	isp3_param_write(params_vdev, value, ISP3X_YNR_RNR_CENTER_COOR);

	value = ISP_PACK_2SHORT(arg->loclagain_adj_thresh, arg->localgain_adj);
	isp3_param_write(params_vdev, value, ISP3X_YNR_LOCAL_GAIN_CTRL);

	value = ISP_PACK_2SHORT(arg->low_bf_inv0, arg->low_bf_inv1);
	isp3_param_write(params_vdev, value, ISP3X_YNR_LOWNR_CTRL0);

	value = ISP_PACK_2SHORT(arg->low_thred_adj, arg->low_peak_supress);
	isp3_param_write(params_vdev, value, ISP3X_YNR_LOWNR_CTRL1);

	value = ISP_PACK_2SHORT(arg->low_edge_adj_thresh, arg->low_dist_adj);
	isp3_param_write(params_vdev, value, ISP3X_YNR_LOWNR_CTRL2);

	value = (arg->low_bi_weight & 0xFF) << 24 |
		(arg->low_weight & 0xFF) << 16 |
		(arg->low_center_weight & 0xFFFF);
	isp3_param_write(params_vdev, value, ISP3X_YNR_LOWNR_CTRL3);

	value = ISP_PACK_2SHORT(arg->lbf_weight_thres, arg->frame_full_size);
	isp3_param_write(params_vdev, value, ISP3X_YNR_LOWNR_CTRL4);

	value = (arg->low_gauss1_coeff2 & 0xFFFF) << 16 |
		(arg->low_gauss1_coeff1 & 0xFF) << 8 |
		(arg->low_gauss1_coeff0 & 0xFF);
	isp3_param_write(params_vdev, value, ISP3X_YNR_GAUSS1_COEFF);

	value = (arg->low_gauss2_coeff2 & 0xFFFF) << 16 |
		(arg->low_gauss2_coeff1 & 0xFF) << 8 |
		(arg->low_gauss2_coeff0 & 0xFF);
	isp3_param_write(params_vdev, value, ISP3X_YNR_GAUSS2_COEFF);

	for (i = 0; i < ISP32_YNR_XY_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->luma_points_x[2 * i],
					arg->luma_points_x[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_YNR_SGM_DX_0_1 + 4 * i);
	}
	value = ISP_PACK_2SHORT(arg->luma_points_x[2 * i], 0);
	isp3_param_write(params_vdev, value, ISP3X_YNR_SGM_DX_0_1 + 4 * i);

	for (i = 0; i < ISP32_YNR_XY_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->lsgm_y[2 * i],
					arg->lsgm_y[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_YNR_LSGM_Y_0_1 + 4 * i);
	}
	value = ISP_PACK_2SHORT(arg->lsgm_y[2 * i], 0);
	isp3_param_write(params_vdev, value, ISP3X_YNR_LSGM_Y_0_1 + 4 * i);

	for (i = 0; i < ISP32_YNR_XY_NUM / 4; i++) {
		value = ISP_PACK_4BYTE(arg->rnr_strength3[4 * i],
					arg->rnr_strength3[4 * i + 1],
					arg->rnr_strength3[4 * i + 2],
					arg->rnr_strength3[4 * i + 3]);
		isp3_param_write(params_vdev, value, ISP3X_YNR_RNR_STRENGTH03 + 4 * i);
	}
	value = ISP_PACK_4BYTE(arg->rnr_strength3[4 * i], 0, 0, 0);
	isp3_param_write(params_vdev, value, ISP3X_YNR_RNR_STRENGTH03 + 4 * i);

	value = (arg->nlm_hi_bf_scale & 0x3ff) << 16 |
		(arg->nlm_hi_gain_alpha & 0x1f) << 11 |
		(arg->nlm_min_sigma & 0x7ff);
	isp3_param_write(params_vdev, value, ISP32_YNR_NLM_SIGMA_GAIN);

	value = (arg->nlm_coe[5] & 0xf) << 20 | (arg->nlm_coe[4] & 0xf) << 16 |
		(arg->nlm_coe[3] & 0xf) << 12 | (arg->nlm_coe[2] & 0xf) << 8 |
		(arg->nlm_coe[1] & 0xf) << 4 | (arg->nlm_coe[0] & 0xf);
	isp3_param_write(params_vdev, value, ISP32_YNR_NLM_COE);

	value = (arg->nlm_center_weight & 0x3ffff) << 10 | (arg->nlm_weight_offset & 0x3ff);
	isp3_param_write(params_vdev, value, ISP32_YNR_NLM_WEIGHT);

	value = arg->nlm_nr_weight & 0x7ff;
	isp3_param_write(params_vdev, value, ISP32_YNR_NLM_NR_WEIGHT);
}

static void
isp_ynr_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 ynr_ctrl;
	bool real_en;

	ynr_ctrl = isp3_param_read_cache(params_vdev, ISP3X_YNR_GLOBAL_CTRL);
	real_en = !!(ynr_ctrl & ISP32_MODULE_EN);
	if ((en && real_en) || (!en && !real_en))
		return;

	if (en) {
		ynr_ctrl |= ISP32_MODULE_EN;
		isp3_param_set_bits(params_vdev, ISP3X_ISP_CTRL1,
				    ISP3X_YNR_FST_FRAME);
	} else {
		ynr_ctrl &= ~ISP32_MODULE_EN;
	}

	isp3_param_write(params_vdev, ynr_ctrl, ISP3X_YNR_GLOBAL_CTRL);
}

static void
isp_cnr_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp32_cnr_cfg *arg)
{
	u32 i, value, ctrl, gain_ctrl;

	gain_ctrl = isp3_param_read(params_vdev, ISP3X_GAIN_CTRL);
	ctrl = isp3_param_read(params_vdev, ISP3X_CNR_CTRL);
	ctrl &= ISP32_MODULE_EN;

	ctrl |= !!arg->bf3x3_wgt0_sel << 8 |
		(arg->thumb_mode & 0x3) << 4 |
		!!arg->yuv422_mode << 2 |
		!!arg->exgain_bypass << 1;
	value = (arg->global_gain & 0x3ff) |
		(arg->global_gain_alpha & 0xf) << 12 |
		arg->gain_iso << 16;
	/* gain disable, using global gain for cnr */
	if (ctrl & ISP32_MODULE_EN && !(gain_ctrl & ISP32_MODULE_EN)) {
		ctrl |= BIT(1);
		value &= ~ISP3X_CNR_GLOBAL_GAIN_ALPHA_MAX;
		value |= BIT(15);
	}
	isp3_param_write(params_vdev, ctrl, ISP3X_CNR_CTRL);
	isp3_param_write(params_vdev, value, ISP3X_CNR_EXGAIN);

	value = ISP_PACK_2SHORT(arg->thumb_sigma_c, arg->thumb_sigma_y);
	isp3_param_write(params_vdev, value, ISP32_CNR_THUMB1);

	value = arg->thumb_bf_ratio & 0x7ff;
	isp3_param_write(params_vdev, value, ISP32_CNR_THUMB_BF_RATIO);

	value = ISP_PACK_4BYTE(arg->lbf1x7_weit_d0, arg->lbf1x7_weit_d1,
			       arg->lbf1x7_weit_d2, arg->lbf1x7_weit_d3);
	isp3_param_write(params_vdev, value, ISP32_CNR_LBF_WEITD);

	value = (arg->wgt_slope & 0x3ff) << 20 | (arg->exp_shift & 0x3f) << 12 |
		arg->iir_strength << 4 | (arg->iir_uvgain & 0xf);
	isp3_param_write(params_vdev, value, ISP32_CNR_IIR_PARA1);

	value = ISP_PACK_4BYTE(arg->chroma_ghost, arg->iir_uv_clip, 0, 0);
	isp3_param_write(params_vdev, value, ISP32_CNR_IIR_PARA2);

	value = ISP_PACK_4BYTE(arg->gaus_coe[0], arg->gaus_coe[1],
			       arg->gaus_coe[2], arg->gaus_coe[3]);
	isp3_param_write(params_vdev, value, ISP32_CNR_GAUS_COE1);

	value = ISP_PACK_4BYTE(arg->gaus_coe[4], arg->gaus_coe[5], 0, 0);
	isp3_param_write(params_vdev, value, ISP32_CNR_GAUS_COE2);

	value = (arg->global_alpha & 0x7ff) << 20 | arg->bf_wgt_clip << 12 |
		(arg->gaus_ratio & 0x7ff);
	isp3_param_write(params_vdev, value, ISP32_CNR_GAUS_RATIO);

	value = arg->bf_ratio << 24 | (arg->sigma_r & 0x3fff) << 8 |
		(arg->uv_gain & 0x7f);
	isp3_param_write(params_vdev, value, ISP32_CNR_BF_PARA1);

	value = (arg->adj_ratio & 0x7fff) << 16 | (arg->adj_offset & 0x1ff);
	isp3_param_write(params_vdev, value, ISP32_CNR_BF_PARA2);

	for (i = 0; i < ISP32_CNR_SIGMA_Y_NUM / 4; i++) {
		value = ISP_PACK_4BYTE(arg->sigma_y[i * 4], arg->sigma_y[i * 4 + 1],
				arg->sigma_y[i * 4 + 2], arg->sigma_y[i * 4 + 3]);
		isp3_param_write(params_vdev, value, ISP32_CNR_SIGMA0 + i * 4);
	}
	value = arg->sigma_y[i * 4];
	isp3_param_write(params_vdev, value, ISP32_CNR_SIGMA0 + i * 4);

	value = (arg->iir_gain_alpha & 0xf) << 8 | arg->iir_global_gain;
	isp3_param_write(params_vdev, value, ISP32_CNR_IIR_GLOBAL_GAIN);
}

static void
isp_cnr_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 cnr_ctrl;
	bool real_en;

	cnr_ctrl = isp3_param_read_cache(params_vdev, ISP3X_CNR_CTRL);
	real_en = !!(cnr_ctrl & ISP32_MODULE_EN);
	if ((en && real_en) || (!en && !real_en))
		return;

	if (en) {
		cnr_ctrl |= ISP32_MODULE_EN;
		isp3_param_set_bits(params_vdev, ISP3X_ISP_CTRL1,
				    ISP3X_CNR_FST_FRAME);
	} else {
		cnr_ctrl &= ~ISP32_MODULE_EN;
	}

	isp3_param_write(params_vdev, cnr_ctrl, ISP3X_CNR_CTRL);
}

static void
isp_sharp_config(struct rkisp_isp_params_vdev *params_vdev,
		 const struct isp32_sharp_cfg *arg)
{
	u32 i, value;

	value = isp3_param_read(params_vdev, ISP3X_SHARP_EN);
	value &= ISP32_MODULE_EN;

	value |= !!arg->bypass << 1 |
		 !!arg->center_mode << 2 |
		 !!arg->exgain_bypass << 3 |
		 !!arg->radius_ds_mode << 4 |
		 !!arg->noiseclip_mode << 5;
	isp3_param_write(params_vdev, value, ISP3X_SHARP_EN);

	value = ISP_PACK_4BYTE(arg->pbf_ratio, arg->gaus_ratio,
				arg->bf_ratio, arg->sharp_ratio);
	isp3_param_write(params_vdev, value, ISP3X_SHARP_RATIO);

	value = (arg->luma_dx[6] & 0x0F) << 24 |
		(arg->luma_dx[5] & 0x0F) << 20 |
		(arg->luma_dx[4] & 0x0F) << 16 |
		(arg->luma_dx[3] & 0x0F) << 12 |
		(arg->luma_dx[2] & 0x0F) << 8 |
		(arg->luma_dx[1] & 0x0F) << 4 |
		(arg->luma_dx[0] & 0x0F);
	isp3_param_write(params_vdev, value, ISP3X_SHARP_LUMA_DX);

	value = (arg->pbf_sigma_inv[2] & 0x3FF) << 20 |
		(arg->pbf_sigma_inv[1] & 0x3FF) << 10 |
		(arg->pbf_sigma_inv[0] & 0x3FF);
	isp3_param_write(params_vdev, value, ISP3X_SHARP_PBF_SIGMA_INV_0);

	value = (arg->pbf_sigma_inv[5] & 0x3FF) << 20 |
		(arg->pbf_sigma_inv[4] & 0x3FF) << 10 |
		(arg->pbf_sigma_inv[3] & 0x3FF);
	isp3_param_write(params_vdev, value, ISP3X_SHARP_PBF_SIGMA_INV_1);

	value = (arg->pbf_sigma_inv[7] & 0x3FF) << 10 |
		(arg->pbf_sigma_inv[6] & 0x3FF);
	isp3_param_write(params_vdev, value, ISP3X_SHARP_PBF_SIGMA_INV_2);

	value = (arg->bf_sigma_inv[2] & 0x3FF) << 20 |
		(arg->bf_sigma_inv[1] & 0x3FF) << 10 |
		(arg->bf_sigma_inv[0] & 0x3FF);
	isp3_param_write(params_vdev, value, ISP3X_SHARP_BF_SIGMA_INV_0);

	value = (arg->bf_sigma_inv[5] & 0x3FF) << 20 |
		(arg->bf_sigma_inv[4] & 0x3FF) << 10 |
		(arg->bf_sigma_inv[3] & 0x3FF);
	isp3_param_write(params_vdev, value, ISP3X_SHARP_BF_SIGMA_INV_1);

	value = (arg->bf_sigma_inv[7] & 0x3FF) << 10 |
		(arg->bf_sigma_inv[6] & 0x3FF);
	isp3_param_write(params_vdev, value, ISP3X_SHARP_BF_SIGMA_INV_2);

	value = (arg->bf_sigma_shift & 0x0F) << 4 |
		(arg->pbf_sigma_shift & 0x0F);
	isp3_param_write(params_vdev, value, ISP3X_SHARP_SIGMA_SHIFT);

	value = (arg->clip_hf[2] & 0x3FF) << 20 |
		(arg->clip_hf[1] & 0x3FF) << 10 |
		(arg->clip_hf[0] & 0x3FF);
	isp3_param_write(params_vdev, value, ISP3X_SHARP_CLIP_HF_0);

	value = (arg->clip_hf[5] & 0x3FF) << 20 |
		(arg->clip_hf[4] & 0x3FF) << 10 |
		(arg->clip_hf[3] & 0x3FF);
	isp3_param_write(params_vdev, value, ISP3X_SHARP_CLIP_HF_1);

	value = (arg->clip_hf[7] & 0x3FF) << 10 |
		(arg->clip_hf[6] & 0x3FF);
	isp3_param_write(params_vdev, value, ISP3X_SHARP_CLIP_HF_2);

	value = ISP_PACK_4BYTE(arg->pbf_coef0, arg->pbf_coef1, arg->pbf_coef2, 0);
	isp3_param_write(params_vdev, value, ISP3X_SHARP_PBF_COEF);

	value = ISP_PACK_4BYTE(arg->bf_coef0, arg->bf_coef1, arg->bf_coef2, 0);
	isp3_param_write(params_vdev, value, ISP3X_SHARP_BF_COEF);

	value = ISP_PACK_4BYTE(arg->gaus_coef[0], arg->gaus_coef[1], arg->gaus_coef[2], 0);
	isp3_param_write(params_vdev, value, ISP3X_SHARP_GAUS_COEF0);

	value = ISP_PACK_4BYTE(arg->gaus_coef[3], arg->gaus_coef[4], arg->gaus_coef[5], 0);
	isp3_param_write(params_vdev, value, ISP3X_SHARP_GAUS_COEF1);

	value = arg->local_gainscale << 24 | (arg->global_gain_alpha & 0xf) << 16 |
		(arg->global_gain & 0x3ff);
	isp3_param_write(params_vdev, value, ISP32_SHARP_GAIN);

	for (i = 0; i < ISP32_SHARP_GAIN_ADJ_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->gain_adj[i * 2], arg->gain_adj[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP32_SHARP_GAIN_ADJUST0 + i * 4);
	}

	value = ISP_PACK_2SHORT(arg->center_wid, arg->center_het);
	isp3_param_write(params_vdev, value, ISP32_SHARP_CENTER);

	for (i = 0; i < ISP32_SHARP_STRENGTH_NUM / 4; i++) {
		value = ISP_PACK_4BYTE(arg->strength[i * 4], arg->strength[i * 4 + 1],
				       arg->strength[i * 4 + 2], arg->strength[i * 4 + 3]);
		isp3_param_write(params_vdev, value, ISP32_SHARP_GAIN_DIS_STRENGTH0 + i * 4);
	}
	value = ISP_PACK_4BYTE(arg->strength[i * 4], arg->strength[i * 4 + 1], 0, 0);
	isp3_param_write(params_vdev, value, ISP32_SHARP_GAIN_DIS_STRENGTH0 + i * 4);

	value = (arg->noise_strength & 0x3fff) << 16 | (arg->enhance_bit & 0xf) << 12 |
		(arg->noise_sigma & 0x3ff);
	isp3_param_write(params_vdev, value, ISP32_SHARP_TEXTURE);
}

static void
isp_sharp_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 value;

	value = isp3_param_read_cache(params_vdev, ISP3X_SHARP_EN);
	value &= ~ISP32_MODULE_EN;

	if (en)
		value |= ISP32_MODULE_EN;

	isp3_param_write(params_vdev, value, ISP3X_SHARP_EN);
}

static void
isp_baynr_config(struct rkisp_isp_params_vdev *params_vdev,
		 const struct isp32_baynr_cfg *arg)
{
	u32 i, value;

	value = isp3_param_read(params_vdev, ISP3X_BAYNR_CTRL);
	value &= ISP32_MODULE_EN;

	value |= !!arg->bay3d_gain_en << 16 |
		 (arg->lg2_mode & 0x3) << 12 |
		 !!arg->gauss_en << 8 |
		 !!arg->log_bypass << 4;
	isp3_param_write(params_vdev, value, ISP3X_BAYNR_CTRL);

	value = ISP_PACK_2SHORT(arg->dgain0, arg->dgain1);
	isp3_param_write(params_vdev, value, ISP3X_BAYNR_DGAIN0);

	isp3_param_write(params_vdev, arg->dgain2, ISP3X_BAYNR_DGAIN1);
	isp3_param_write(params_vdev, arg->pix_diff, ISP3X_BAYNR_PIXDIFF);

	value = ISP_PACK_2SHORT(arg->softthld, arg->diff_thld);
	isp3_param_write(params_vdev, value, ISP3X_BAYNR_THLD);

	value = ISP_PACK_2SHORT(arg->reg_w1, arg->bltflt_streng);
	isp3_param_write(params_vdev, value, ISP3X_BAYNR_W1_STRENG);

	for (i = 0; i < ISP32_BAYNR_XY_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->sigma_x[2 * i], arg->sigma_x[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_BAYNR_SIGMAX01 + 4 * i);
	}

	for (i = 0; i < ISP32_BAYNR_XY_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->sigma_y[2 * i], arg->sigma_y[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_BAYNR_SIGMAY01 + 4 * i);
	}

	value = (arg->weit_d2 & 0x3FF) << 20 |
		(arg->weit_d1 & 0x3FF) << 10 |
		(arg->weit_d0 & 0x3FF);
	isp3_param_write(params_vdev, value, ISP3X_BAYNR_WRIT_D);

	value = ISP_PACK_2SHORT(arg->lg2_off, arg->lg2_lgoff);
	isp3_param_write(params_vdev, value, ISP3X_BAYNR_LG_OFF);

	value = arg->dat_max & 0xfffff;
	isp3_param_write(params_vdev, value, ISP3X_BAYNR_DAT_MAX);

	value = ISP_PACK_2SHORT(arg->rgain_off, arg->bgain_off);
	isp3_param_write(params_vdev, value, ISP32_BAYNR_SIGOFF);

	for (i = 0; i < ISP32_BAYNR_GAIN_NUM / 4; i++) {
		value = ISP_PACK_4BYTE(arg->gain_x[i * 4], arg->gain_x[i * 4 + 1],
				       arg->gain_x[i * 4 + 2], arg->gain_x[i * 4 + 3]);
		isp3_param_write(params_vdev, value, ISP32_BAYNR_GAINX03 + i * 4);
	}

	for (i = 0; i < ISP32_BAYNR_GAIN_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->gain_y[i * 2], arg->gain_y[i * 2 + 1]);
		isp3_param_write(params_vdev, value, ISP32_BAYNR_GAINY01 + i * 4);
	}
}

static void
isp_baynr_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 value;

	value = isp3_param_read_cache(params_vdev, ISP3X_BAYNR_CTRL);
	value &= ~ISP32_MODULE_EN;

	if (en)
		value |= ISP32_MODULE_EN;

	isp3_param_write(params_vdev, value, ISP3X_BAYNR_CTRL);
}

static void
isp_bay3d_config(struct rkisp_isp_params_vdev *params_vdev,
		 const struct isp32_bay3d_cfg *arg)
{
	struct rkisp_isp_params_val_v32 *priv_val;
	u32 i, value;

	priv_val = (struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;
	value = isp3_param_read(params_vdev, ISP3X_BAY3D_CTRL);
	value &= (ISP32_MODULE_EN | ISP32_BAY3D_BWSAVING(1));

	value |= !!arg->loswitch_protect << 12 |
		 !!arg->glbpk_en << 11 |
		 !!arg->logaus3_bypass_en << 10 |
		 !!arg->logaus5_bypass_en << 9 |
		 !!arg->lomed_bypass_en << 8 |
		 !!arg->hichnsplit_en << 7 |
		 !!arg->hiabs_possel << 6 |
		 !!arg->higaus_bypass_en << 5 |
		 !!arg->himed_bypass_en << 4 |
		 //!!arg->lobypass_en << 3 |
		 !!arg->hibypass_en << 2 |
		 !!arg->bypass_en << 1;
	if (!(value & ISP32_MODULE_EN)) {
		value &= ~ISP32_BAY3D_BWSAVING(1);
		if (arg->bwsaving_en)
			value |= ISP32_BAY3D_BWSAVING(1);
	} else if ((value & ISP32_BAY3D_BWSAVING(1)) !=
		   ISP32_BAY3D_BWSAVING(!!arg->bwsaving_en)) {
		v4l2_warn(&params_vdev->dev->v4l2_dev,
			  "bwsaving to %d no support change for bay3d en\n",
			  arg->bwsaving_en);
	}
	isp3_param_write(params_vdev, value, ISP3X_BAY3D_CTRL);

	value = !!arg->wgtmix_opt_en << 12 |
		!!arg->higaus5x5_en << 11 |
		(arg->higaus3_mode & 0x3) << 9 |
		!!arg->curds_high_en << 8 |
		!!arg->iirwr_rnd_en << 7 |
		!!arg->pksig_ind_sel << 6 |
		!!arg->hisig_ind_sel << 5 |
		!!arg->lo4x4_en << 4 |
		!!arg->lo4x8_en << 3 |
		!!arg->bwopt_gain_dis << 2 |
		!!arg->hichncor_en << 1 |
		!!arg->hiwgt_opt_en;
	if (priv_val->is_lo8x8)
		value &= ~(BIT(3) | BIT(4));
	else if (!(value & (BIT(3) | BIT(4))))
		value |= BIT(3);
	isp3_param_write(params_vdev, value, ISP32_BAY3D_CTRL1);

	value = ISP_PACK_2SHORT(arg->softwgt, arg->hidif_th);
	isp3_param_write(params_vdev, value, ISP3X_BAY3D_KALRATIO);

	isp3_param_write(params_vdev, arg->glbpk2, ISP3X_BAY3D_GLBPK2);

	value = ISP_PACK_2SHORT(arg->wgtlmt, arg->wgtratio);
	isp3_param_write(params_vdev, value, ISP3X_BAY3D_WGTLMT);

	for (i = 0; i < ISP32_BAY3D_XY_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->sig0_x[2 * i],
					arg->sig0_x[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_BAY3D_SIG0_X0 + 4 * i);

		value = ISP_PACK_2SHORT(arg->sig1_x[2 * i],
					arg->sig1_x[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_BAY3D_SIG1_X0 + 4 * i);
	}

	for (i = 0; i < ISP32_BAY3D_XY_NUM / 2; i++) {
		value = ISP_PACK_2SHORT(arg->sig0_y[2 * i],
					arg->sig0_y[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_BAY3D_SIG0_Y0 + 4 * i);

		value = ISP_PACK_2SHORT(arg->sig1_y[2 * i],
					arg->sig1_y[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_BAY3D_SIG1_Y0 + 4 * i);

		value = ISP_PACK_2SHORT(arg->sig2_y[2 * i],
					arg->sig2_y[2 * i + 1]);
		isp3_param_write(params_vdev, value, ISP3X_BAY3D_SIG2_Y0 + 4 * i);
	}

	value = ISP_PACK_2SHORT(arg->hisigrat0, arg->hisigrat1);
	isp3_param_write(params_vdev, value, ISP32_BAY3D_HISIGRAT);

	value = ISP_PACK_2SHORT(arg->hisigoff0, arg->hisigoff1);
	isp3_param_write(params_vdev, value, ISP32_BAY3D_HISIGOFF);

	value = ISP_PACK_2SHORT(arg->losigoff, arg->losigrat);
	isp3_param_write(params_vdev, value, ISP32_BAY3D_LOSIG);

	value = ISP_PACK_2SHORT(arg->rgain_off, arg->bgain_off);
	isp3_param_write(params_vdev, value, ISP32_BAY3D_SIGPK);

	value = ISP_PACK_4BYTE(arg->siggaus0, arg->siggaus1,
			       arg->siggaus2, arg->siggaus3);
	isp3_param_write(params_vdev, value, ISP32_BAY3D_SIGGAUS);
}

static void
isp_bay3d_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	struct rkisp_device *ispdev = params_vdev->dev;
	struct rkisp_isp_params_val_v32 *priv_val;
	u32 value, bay3d_ctrl;

	priv_val = (struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;
	bay3d_ctrl = isp3_param_read_cache(params_vdev, ISP3X_BAY3D_CTRL);
	if ((en && (bay3d_ctrl & ISP32_MODULE_EN)) ||
	    (!en && !(bay3d_ctrl & ISP32_MODULE_EN)))
		return;

	if (en) {
		if (!priv_val->buf_3dnr_iir.mem_priv) {
			dev_err(ispdev->dev, "no bay3d buffer available\n");
			return;
		}

		/* mibuf_size for fifo_cur_full, set to max: (3072 - 2) / 2, 2 align */
		value = 0x5fe << 16;
		isp3_param_set_bits(params_vdev, ISP3X_BAY3D_IN_IRQ_LINECNT, value);

		value = isp3_param_read_cache(params_vdev, ISP32_BAY3D_CTRL1);
		if (priv_val->is_lo8x8) {
			if (value & (BIT(3) | BIT(4))) {
				value &= ~(BIT(3) | BIT(4));
				isp3_param_write(params_vdev, value, ISP32_BAY3D_CTRL1);
			}
		} else if (!(value & (BIT(3) | BIT(4)))) {
			value |= BIT(3);
			isp3_param_write(params_vdev, value, ISP32_BAY3D_CTRL1);
		}
		bay3d_ctrl |= ISP32_MODULE_EN;
		isp3_param_write(params_vdev, bay3d_ctrl, ISP3X_BAY3D_CTRL);

		value = ISP3X_BAY3D_IIR_WR_AUTO_UPD | ISP3X_BAY3D_CUR_WR_AUTO_UPD |
			ISP3X_BAY3D_DS_WR_AUTO_UPD | ISP3X_BAY3D_IIRSELF_UPD |
			ISP3X_BAY3D_CURSELF_UPD | ISP3X_BAY3D_DSSELF_UPD |
			ISP3X_BAY3D_RDSELF_UPD;
		isp3_param_set_bits(params_vdev, MI_WR_CTRL2, value);

		isp3_param_set_bits(params_vdev, ISP3X_ISP_CTRL1, ISP3X_RAW3D_FST_FRAME);
	} else {
		bay3d_ctrl &= ~ISP32_MODULE_EN;
		isp3_param_write(params_vdev, bay3d_ctrl, ISP3X_BAY3D_CTRL);
		isp3_param_clear_bits(params_vdev, ISP3X_GAIN_CTRL, BIT(4));
	}
}

static void
isp_gain_config(struct rkisp_isp_params_vdev *params_vdev,
		const struct isp3x_gain_cfg *arg)
{
	u32 val;

	val = arg->g0 & 0x3ffff;
	isp3_param_write(params_vdev, val, ISP3X_GAIN_G0);
	val = ISP_PACK_2SHORT(arg->g1, arg->g2);
	isp3_param_write(params_vdev, val, ISP3X_GAIN_G1_G2);
}

static void
isp_gain_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	struct rkisp_isp_params_val_v32 *priv_val =
		(struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;
	u32 val = 0;

	val = isp3_param_read_cache(params_vdev, ISP3X_GAIN_CTRL);
	if (en) {
		val |= priv_val->lut3d_en << 20 |
			priv_val->dhaz_en << 16 |
			priv_val->drc_en << 12 |
			priv_val->lsc_en << 8 |
			priv_val->bay3d_en << 4;
		if (isp3_param_read(params_vdev, ISP3X_HDRMGE_CTRL) & BIT(0))
			val |= BIT(1);
		if (val)
			val |= ISP32_MODULE_EN;
	}
	isp3_param_write(params_vdev, val, ISP3X_GAIN_CTRL);
}

static void
isp_cac_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp32_cac_cfg *arg)
{
	struct rkisp_device *dev = params_vdev->dev;
	struct rkisp_isp_params_val_v32 *priv_val;
	struct isp2x_mesh_head *head;
	u32 i, val, ctrl;

	priv_val = (struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;

	ctrl = isp3_param_read(params_vdev, ISP3X_CAC_CTRL);
	ctrl &= ISP3X_CAC_EN;
	ctrl |= !!arg->bypass_en << 1 | !!arg->center_en << 3 |
		(arg->clip_g_mode & 0x3) << 5 | !!arg->edge_detect_en << 7 |
		!!arg->neg_clip0_en << 9;

	val = (arg->psf_sft_bit & 0xff) |
		(arg->cfg_num & 0x7ff) << 8;
	isp3_param_write(params_vdev, val, ISP3X_CAC_PSF_PARA);

	val = ISP_PACK_2SHORT(arg->center_width, arg->center_height);
	isp3_param_write(params_vdev, val, ISP3X_CAC_STRENGTH_CENTER);

	for (i = 0; i < ISP32_CAC_STRENGTH_NUM / 2; i++) {
		val = ISP_PACK_2SHORT(arg->strength[2 * i], arg->strength[2 * i + 1]);
		isp3_param_write(params_vdev, val, ISP3X_CAC_STRENGTH0 + i * 4);
	}

	val = (arg->flat_thed_r & 0x1f) << 8 | (arg->flat_thed_b & 0x1f);
	isp3_param_write(params_vdev, val, ISP32_CAC_FLAT_THED);

	val = ISP_PACK_2SHORT(arg->offset_b, arg->offset_r);
	isp3_param_write(params_vdev, val, ISP32_CAC_OFFSET);

	val = arg->expo_thed_b & 0x1fffff;
	isp3_param_write(params_vdev, val, ISP32_CAC_EXPO_THED_B);

	val = arg->expo_thed_r & 0x1fffff;
	isp3_param_write(params_vdev, val, ISP32_CAC_EXPO_THED_R);

	val = arg->expo_adj_b & 0xfffff;
	isp3_param_write(params_vdev, val, ISP32_CAC_EXPO_ADJ_B);

	val = arg->expo_adj_r & 0xfffff;
	isp3_param_write(params_vdev, val, ISP32_CAC_EXPO_ADJ_R);

	for (i = 0; i < ISP32_MESH_BUF_NUM; i++) {
		if (!priv_val->buf_cac[i].mem_priv)
			continue;
		if (arg->buf_fd == priv_val->buf_cac[i].dma_fd)
			break;
	}

	if (i == ISP32_MESH_BUF_NUM) {
		dev_err(dev->dev, "cannot find cac buf fd(%d)\n", arg->buf_fd);
		return;
	}

	if (!priv_val->buf_cac[i].vaddr) {
		dev_err(dev->dev, "no cac buffer allocated\n");
		return;
	}

	val = priv_val->buf_cac_idx;
	head = (struct isp2x_mesh_head *)priv_val->buf_cac[val].vaddr;
	head->stat = MESH_BUF_INIT;

	head = (struct isp2x_mesh_head *)priv_val->buf_cac[i].vaddr;
	head->stat = MESH_BUF_CHIPINUSE;
	priv_val->buf_cac_idx = i;
	rkisp_prepare_buffer(dev, &priv_val->buf_cac[i]);
	val = priv_val->buf_cac[i].dma_addr + head->data_oft;
	isp3_param_write(params_vdev, val, ISP3X_MI_LUT_CAC_RD_BASE);
	isp3_param_write(params_vdev, arg->hsize, ISP3X_MI_LUT_CAC_RD_H_WSIZE);
	isp3_param_write(params_vdev, arg->vsize, ISP3X_MI_LUT_CAC_RD_V_SIZE);
	if (ctrl & ISP3X_CAC_EN)
		ctrl |= ISP3X_CAC_LUT_EN | ISP32_SELF_FORCE_UPD | ISP3X_CAC_LUT_MODE(3);
	isp3_param_write(params_vdev, ctrl, ISP3X_CAC_CTRL);
}

static void
isp_cac_enable(struct rkisp_isp_params_vdev *params_vdev, bool en)
{
	u32 val;

	val = isp3_param_read(params_vdev, ISP3X_CAC_CTRL);
	val &= ~(ISP3X_CAC_EN | ISP3X_CAC_LUT_EN | ISP32_SELF_FORCE_UPD);
	if (en)
		val |= ISP3X_CAC_EN | ISP3X_CAC_LUT_EN |
		       ISP32_SELF_FORCE_UPD | ISP3X_CAC_LUT_MODE(3);
	isp3_param_write(params_vdev, val, ISP3X_CAC_CTRL);
}

static void
isp_csm_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp21_csm_cfg *arg)
{
	u32 i, val;

	for (i = 0; i < ISP32_CSM_COEFF_NUM; i++) {
		if (i == 0)
			val = (arg->csm_y_offset & 0x3f) << 24 |
			      (arg->csm_c_offset & 0xff) << 16 |
			      (arg->csm_coeff[i] & 0x1ff);
		else
			val = arg->csm_coeff[i] & 0x1ff;
		isp3_param_write(params_vdev, val, ISP3X_ISP_CC_COEFF_0 + i * 4);
	}

	val = isp3_param_read_cache(params_vdev, ISP3X_ISP_CTRL0);
	val |= CIF_ISP_CTRL_ISP_CSM_Y_FULL_ENA | CIF_ISP_CTRL_ISP_CSM_C_FULL_ENA;
	isp3_param_write(params_vdev, val, ISP3X_ISP_CTRL0);
}

static void
isp_cgc_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp21_cgc_cfg *arg)
{
	u32 val = isp3_param_read_cache(params_vdev, ISP3X_ISP_CTRL0);
	u32 eff_ctrl, cproc_ctrl;

	params_vdev->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	val &= ~(ISP3X_SW_CGC_YUV_LIMIT | ISP3X_SW_CGC_RATIO_EN);
	if (arg->yuv_limit) {
		val |= ISP3X_SW_CGC_YUV_LIMIT;
		params_vdev->quantization = V4L2_QUANTIZATION_LIM_RANGE;
	}
	if (arg->ratio_en)
		val |= ISP3X_SW_CGC_RATIO_EN;
	isp3_param_write(params_vdev, val, ISP3X_ISP_CTRL0);

	cproc_ctrl = isp3_param_read(params_vdev, ISP3X_CPROC_CTRL);
	if (cproc_ctrl & CIF_C_PROC_CTR_ENABLE) {
		val = CIF_C_PROC_YOUT_FULL | CIF_C_PROC_YIN_FULL | CIF_C_PROC_COUT_FULL;
		if (arg->yuv_limit)
			cproc_ctrl &= ~val;
		else
			cproc_ctrl |= val;
		isp3_param_write(params_vdev, cproc_ctrl, ISP3X_CPROC_CTRL);
	}

	eff_ctrl = isp3_param_read(params_vdev, ISP3X_IMG_EFF_CTRL);
	if (eff_ctrl & CIF_IMG_EFF_CTRL_ENABLE) {
		if (arg->yuv_limit)
			eff_ctrl &= ~CIF_IMG_EFF_CTRL_YCBCR_FULL;
		else
			eff_ctrl |= CIF_IMG_EFF_CTRL_YCBCR_FULL;
		isp3_param_write(params_vdev, eff_ctrl, ISP3X_IMG_EFF_CTRL);
	}
}

static void
isp_vsm_config(struct rkisp_isp_params_vdev *params_vdev,
	       const struct isp32_vsm_cfg *arg)
{
	struct rkisp_device *ispdev = params_vdev->dev;
	struct v4l2_rect *out_crop = &ispdev->isp_sdev.out_crop;
	u32 width = out_crop->width;
	u32 height = out_crop->height;
	u32 val, h, v;

	val = arg->h_offs;
	isp3_param_write(params_vdev, val, ISP32_VSM_H_OFFS);
	val = arg->v_offs;
	isp3_param_write(params_vdev, val, ISP32_VSM_V_OFFS);

	h = arg->h_size;
	if (h > width - arg->h_offs)
		h = width - arg->h_offs;
	h &= ~1;
	isp3_param_write(params_vdev, h, ISP32_VSM_H_SIZE);

	v = arg->v_size;
	if (v > height - arg->v_offs)
		v = height - arg->v_offs;
	v &= ~1;
	isp3_param_write(params_vdev, v, ISP32_VSM_V_SIZE);

	val = arg->h_segments;
	if (val > (h - 48) / 16)
		val = (h - 48) / 16;
	isp3_param_write(params_vdev, val, ISP32_VSM_H_SEGMENTS);

	val = arg->v_segments;
	if (val > (v - 48) / 16)
		val = (v - 48) / 16;
	isp3_param_write(params_vdev, val, ISP32_VSM_V_SEGMENTS);
}

static void
isp_vsm_enable(struct rkisp_isp_params_vdev *params_vdev,
	       bool en)
{
	isp3_param_write(params_vdev, en, ISP32_VSM_MODE);
}

struct rkisp_isp_params_ops_v32 isp_params_ops_v32 = {
	.dpcc_config = isp_dpcc_config,
	.dpcc_enable = isp_dpcc_enable,
	.bls_config = isp_bls_config,
	.bls_enable = isp_bls_enable,
	.sdg_config = isp_sdg_config,
	.sdg_enable = isp_sdg_enable,
	.lsc_config = isp_lsc_config,
	.lsc_enable = isp_lsc_enable,
	.awbgain_config = isp_awbgain_config,
	.awbgain_enable = isp_awbgain_enable,
	.debayer_config = isp_debayer_config,
	.debayer_enable = isp_debayer_enable,
	.ccm_config = isp_ccm_config,
	.ccm_enable = isp_ccm_enable,
	.goc_config = isp_goc_config,
	.goc_enable = isp_goc_enable,
	.csm_config = isp_csm_config,
	.cproc_config = isp_cproc_config,
	.cproc_enable = isp_cproc_enable,
	.ie_config = isp_ie_config,
	.ie_enable = isp_ie_enable,
	.rawaf_config = isp_rawaf_config,
	.rawaf_enable = isp_rawaf_enable,
	.rawae0_config = isp_rawaelite_config,
	.rawae0_enable = isp_rawaelite_enable,
	.rawae1_config = isp_rawae1_config,
	.rawae1_enable = isp_rawae1_enable,
	.rawae2_config = isp_rawae2_config,
	.rawae2_enable = isp_rawae2_enable,
	.rawae3_config = isp_rawae3_config,
	.rawae3_enable = isp_rawae3_enable,
	.rawawb_config = isp_rawawb_config,
	.rawawb_enable = isp_rawawb_enable,
	.rawhst0_config = isp_rawhstlite_config,
	.rawhst0_enable = isp_rawhstlite_enable,
	.rawhst1_config = isp_rawhst1_config,
	.rawhst1_enable = isp_rawhst1_enable,
	.rawhst2_config = isp_rawhst2_config,
	.rawhst2_enable = isp_rawhst2_enable,
	.rawhst3_config = isp_rawhst3_config,
	.rawhst3_enable = isp_rawhst3_enable,
	.hdrmge_config = isp_hdrmge_config,
	.hdrmge_enable = isp_hdrmge_enable,
	.hdrdrc_config = isp_hdrdrc_config,
	.hdrdrc_enable = isp_hdrdrc_enable,
	.gic_config = isp_gic_config,
	.gic_enable = isp_gic_enable,
	.dhaz_config = isp_dhaz_config,
	.dhaz_enable = isp_dhaz_enable,
	.isp3dlut_config = isp_3dlut_config,
	.isp3dlut_enable = isp_3dlut_enable,
	.ldch_config = isp_ldch_config,
	.ldch_enable = isp_ldch_enable,
	.ynr_config = isp_ynr_config,
	.ynr_enable = isp_ynr_enable,
	.cnr_config = isp_cnr_config,
	.cnr_enable = isp_cnr_enable,
	.sharp_config = isp_sharp_config,
	.sharp_enable = isp_sharp_enable,
	.baynr_config = isp_baynr_config,
	.baynr_enable = isp_baynr_enable,
	.bay3d_config = isp_bay3d_config,
	.bay3d_enable = isp_bay3d_enable,
	.gain_config = isp_gain_config,
	.gain_enable = isp_gain_enable,
	.cac_config = isp_cac_config,
	.cac_enable = isp_cac_enable,
	.cgc_config = isp_cgc_config,
	.vsm_config = isp_vsm_config,
	.vsm_enable = isp_vsm_enable,
};

static __maybe_unused
void __isp_isr_other_config(struct rkisp_isp_params_vdev *params_vdev,
			    const struct isp32_isp_params_cfg *new_params,
			    enum rkisp_params_type type)
{
	struct rkisp_isp_params_ops_v32 *ops =
		(struct rkisp_isp_params_ops_v32 *)params_vdev->priv_ops;
	u64 module_cfg_update = new_params->module_cfg_update;

	if (type == RKISP_PARAMS_SHD) {
		if ((module_cfg_update & ISP32_MODULE_HDRMGE))
			ops->hdrmge_config(params_vdev, &new_params->others.hdrmge_cfg, type);

		if ((module_cfg_update & ISP32_MODULE_DRC))
			ops->hdrdrc_config(params_vdev, &new_params->others.drc_cfg, type);
		return;
	}

	v4l2_dbg(4, rkisp_debug, &params_vdev->dev->v4l2_dev,
		 "%s seq:%d module_cfg_update:0x%llx\n",
		 __func__, new_params->frame_id, module_cfg_update);

	if (module_cfg_update & ISP32_MODULE_LSC)
		ops->lsc_config(params_vdev, &new_params->others.lsc_cfg);

	if (module_cfg_update & ISP32_MODULE_DPCC)
		ops->dpcc_config(params_vdev, &new_params->others.dpcc_cfg);

	if (module_cfg_update & ISP32_MODULE_BLS)
		ops->bls_config(params_vdev, &new_params->others.bls_cfg);

	if (module_cfg_update & ISP32_MODULE_SDG)
		ops->sdg_config(params_vdev, &new_params->others.sdg_cfg);

	if (module_cfg_update & ISP32_MODULE_AWB_GAIN)
		ops->awbgain_config(params_vdev, &new_params->others.awb_gain_cfg);

	if (module_cfg_update & ISP32_MODULE_DEBAYER)
		ops->debayer_config(params_vdev, &new_params->others.debayer_cfg);

	if (module_cfg_update & ISP32_MODULE_CCM)
		ops->ccm_config(params_vdev, &new_params->others.ccm_cfg);

	if (module_cfg_update & ISP32_MODULE_GOC)
		ops->goc_config(params_vdev, &new_params->others.gammaout_cfg);

	/* range csm->cgc->cproc->ie */
	if (module_cfg_update & ISP3X_MODULE_CSM)
		ops->csm_config(params_vdev, &new_params->others.csm_cfg);

	if (module_cfg_update & ISP3X_MODULE_CGC)
		ops->cgc_config(params_vdev, &new_params->others.cgc_cfg);

	if (module_cfg_update & ISP32_MODULE_CPROC)
		ops->cproc_config(params_vdev, &new_params->others.cproc_cfg);

	if (module_cfg_update & ISP32_MODULE_IE)
		ops->ie_config(params_vdev, &new_params->others.ie_cfg);

	if (module_cfg_update & ISP32_MODULE_HDRMGE)
		ops->hdrmge_config(params_vdev, &new_params->others.hdrmge_cfg, type);

	if (module_cfg_update & ISP32_MODULE_DRC)
		ops->hdrdrc_config(params_vdev, &new_params->others.drc_cfg, type);

	if (module_cfg_update & ISP32_MODULE_GIC)
		ops->gic_config(params_vdev, &new_params->others.gic_cfg);

	if (module_cfg_update & ISP32_MODULE_DHAZ)
		ops->dhaz_config(params_vdev, &new_params->others.dhaz_cfg);

	if (module_cfg_update & ISP32_MODULE_3DLUT)
		ops->isp3dlut_config(params_vdev, &new_params->others.isp3dlut_cfg);

	if (module_cfg_update & ISP32_MODULE_LDCH)
		ops->ldch_config(params_vdev, &new_params->others.ldch_cfg);

	if (module_cfg_update & ISP32_MODULE_YNR)
		ops->ynr_config(params_vdev, &new_params->others.ynr_cfg);

	if (module_cfg_update & ISP32_MODULE_CNR)
		ops->cnr_config(params_vdev, &new_params->others.cnr_cfg);

	if (module_cfg_update & ISP32_MODULE_SHARP)
		ops->sharp_config(params_vdev, &new_params->others.sharp_cfg);

	if (module_cfg_update & ISP32_MODULE_BAYNR)
		ops->baynr_config(params_vdev, &new_params->others.baynr_cfg);

	if (module_cfg_update & ISP32_MODULE_BAY3D)
		ops->bay3d_config(params_vdev, &new_params->others.bay3d_cfg);

	if (module_cfg_update & ISP32_MODULE_CAC)
		ops->cac_config(params_vdev, &new_params->others.cac_cfg);

	if (module_cfg_update & ISP32_MODULE_GAIN)
		ops->gain_config(params_vdev, &new_params->others.gain_cfg);

	if (module_cfg_update & ISP32_MODULE_VSM)
		ops->vsm_config(params_vdev, &new_params->others.vsm_cfg);
}

static __maybe_unused
void __isp_isr_other_en(struct rkisp_isp_params_vdev *params_vdev,
			const struct isp32_isp_params_cfg *new_params,
			enum rkisp_params_type type)
{
	struct rkisp_isp_params_ops_v32 *ops =
		(struct rkisp_isp_params_ops_v32 *)params_vdev->priv_ops;
	struct rkisp_isp_params_val_v32 *priv_val =
		(struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;
	u64 module_en_update = new_params->module_en_update;
	u64 module_ens = new_params->module_ens;
	u32 gain_ctrl, cnr_ctrl, val;

	if (type == RKISP_PARAMS_SHD)
		return;

	v4l2_dbg(4, rkisp_debug, &params_vdev->dev->v4l2_dev,
		 "%s seq:%d module_en_update:0x%llx module_ens:0x%llx\n",
		 __func__, new_params->frame_id, module_en_update, module_ens);

	if (module_en_update & ISP32_MODULE_DPCC)
		ops->dpcc_enable(params_vdev, !!(module_ens & ISP32_MODULE_DPCC));

	if (module_en_update & ISP32_MODULE_BLS)
		ops->bls_enable(params_vdev, !!(module_ens & ISP32_MODULE_BLS));

	if (module_en_update & ISP32_MODULE_SDG)
		ops->sdg_enable(params_vdev, !!(module_ens & ISP32_MODULE_SDG));

	if (module_en_update & ISP32_MODULE_LSC) {
		ops->lsc_enable(params_vdev, !!(module_ens & ISP32_MODULE_LSC));
		priv_val->lsc_en = !!(module_ens & ISP32_MODULE_LSC);
	}

	if (module_en_update & ISP32_MODULE_AWB_GAIN)
		ops->awbgain_enable(params_vdev, !!(module_ens & ISP32_MODULE_AWB_GAIN));

	if (module_en_update & ISP32_MODULE_DEBAYER)
		ops->debayer_enable(params_vdev, !!(module_ens & ISP32_MODULE_DEBAYER));

	if (module_en_update & ISP32_MODULE_CCM)
		ops->ccm_enable(params_vdev, !!(module_ens & ISP32_MODULE_CCM));

	if (module_en_update & ISP32_MODULE_GOC)
		ops->goc_enable(params_vdev, !!(module_ens & ISP32_MODULE_GOC));

	if (module_en_update & ISP32_MODULE_CPROC)
		ops->cproc_enable(params_vdev, !!(module_ens & ISP32_MODULE_CPROC));

	if (module_en_update & ISP32_MODULE_IE)
		ops->ie_enable(params_vdev, !!(module_ens & ISP32_MODULE_IE));

	if (module_en_update & ISP32_MODULE_HDRMGE) {
		ops->hdrmge_enable(params_vdev, !!(module_ens & ISP32_MODULE_HDRMGE));
		priv_val->mge_en = !!(module_ens & ISP32_MODULE_HDRMGE);
	}

	if (module_en_update & ISP32_MODULE_DRC) {
		ops->hdrdrc_enable(params_vdev, !!(module_ens & ISP32_MODULE_DRC));
		priv_val->drc_en = !!(module_ens & ISP32_MODULE_DRC);
	}

	if (module_en_update & ISP32_MODULE_GIC)
		ops->gic_enable(params_vdev, !!(module_ens & ISP32_MODULE_GIC));

	if (module_en_update & ISP32_MODULE_DHAZ) {
		ops->dhaz_enable(params_vdev, !!(module_ens & ISP32_MODULE_DHAZ));
		priv_val->dhaz_en = !!(module_ens & ISP32_MODULE_DHAZ);
	}

	if (module_en_update & ISP32_MODULE_3DLUT) {
		ops->isp3dlut_enable(params_vdev, !!(module_ens & ISP32_MODULE_3DLUT));
		priv_val->lut3d_en = !!(module_ens & ISP32_MODULE_3DLUT);
	}

	if (module_en_update & ISP32_MODULE_LDCH)
		ops->ldch_enable(params_vdev, !!(module_ens & ISP32_MODULE_LDCH));

	if (module_en_update & ISP32_MODULE_YNR)
		ops->ynr_enable(params_vdev, !!(module_ens & ISP32_MODULE_YNR));

	if (module_en_update & ISP32_MODULE_CNR)
		ops->cnr_enable(params_vdev, !!(module_ens & ISP32_MODULE_CNR));

	if (module_en_update & ISP32_MODULE_SHARP)
		ops->sharp_enable(params_vdev, !!(module_ens & ISP32_MODULE_SHARP));

	if (module_en_update & ISP32_MODULE_BAYNR)
		ops->baynr_enable(params_vdev, !!(module_ens & ISP32_MODULE_BAYNR));

	if (module_en_update & ISP32_MODULE_BAY3D) {
		ops->bay3d_enable(params_vdev, !!(module_ens & ISP32_MODULE_BAY3D));
		priv_val->bay3d_en = !!(module_ens & ISP32_MODULE_BAY3D);
	}

	if (module_en_update & ISP32_MODULE_CAC)
		ops->cac_enable(params_vdev, !!(module_ens & ISP32_MODULE_CAC));

	if (module_en_update & ISP32_MODULE_GAIN ||
	    ((priv_val->buf_info_owner == RKISP_INFO2DRR_OWNER_GAIN) &&
	     !(isp3_param_read(params_vdev, ISP3X_GAIN_CTRL) & ISP3X_GAIN_2DDR_EN)))
		ops->gain_enable(params_vdev, !!(module_ens & ISP32_MODULE_GAIN));

	if (module_en_update & ISP32_MODULE_VSM)
		ops->vsm_enable(params_vdev, !!(module_ens & ISP32_MODULE_VSM));

	/* gain disable, using global gain for cnr */
	gain_ctrl = isp3_param_read_cache(params_vdev, ISP3X_GAIN_CTRL);
	cnr_ctrl = isp3_param_read_cache(params_vdev, ISP3X_CNR_CTRL);
	if (!(gain_ctrl & ISP32_MODULE_EN) && cnr_ctrl & ISP32_MODULE_EN) {
		cnr_ctrl |= BIT(1);
		isp3_param_write(params_vdev, cnr_ctrl, ISP3X_CNR_CTRL);
		val = isp3_param_read(params_vdev, ISP3X_CNR_EXGAIN) & 0x3ff;
		isp3_param_write(params_vdev, val | 0x8000, ISP3X_CNR_EXGAIN);
	}
}

static __maybe_unused
void __isp_isr_meas_config(struct rkisp_isp_params_vdev *params_vdev,
			   struct isp32_isp_params_cfg *new_params,
			   enum rkisp_params_type type)
{
	struct rkisp_isp_params_ops_v32 *ops =
		(struct rkisp_isp_params_ops_v32 *)params_vdev->priv_ops;
	struct rkisp_isp_params_val_v32 *priv_val =
		(struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;
	u64 module_cfg_update = new_params->module_cfg_update;

	if (type == RKISP_PARAMS_SHD)
		return;

	v4l2_dbg(4, rkisp_debug, &params_vdev->dev->v4l2_dev,
		 "%s seq:%d module_cfg_update:0x%llx\n",
		 __func__, new_params->frame_id, module_cfg_update);

	if ((module_cfg_update & ISP32_MODULE_RAWAF))
		ops->rawaf_config(params_vdev, &new_params->meas.rawaf);

	if ((module_cfg_update & ISP32_MODULE_RAWAE0))
		ops->rawae0_config(params_vdev, &new_params->meas.rawae0);

	if ((module_cfg_update & ISP32_MODULE_RAWAE1))
		ops->rawae1_config(params_vdev, &new_params->meas.rawae1);

	if ((module_cfg_update & ISP32_MODULE_RAWAE2))
		ops->rawae2_config(params_vdev, &new_params->meas.rawae2);

	if ((module_cfg_update & ISP32_MODULE_RAWAE3) && !params_vdev->afaemode_en)
		ops->rawae3_config(params_vdev, &new_params->meas.rawae3);

	if ((module_cfg_update & ISP32_MODULE_RAWHIST0))
		ops->rawhst0_config(params_vdev, &new_params->meas.rawhist0);

	if ((module_cfg_update & ISP32_MODULE_RAWHIST1))
		ops->rawhst1_config(params_vdev, &new_params->meas.rawhist1);

	if ((module_cfg_update & ISP32_MODULE_RAWHIST2))
		ops->rawhst2_config(params_vdev, &new_params->meas.rawhist2);

	if ((module_cfg_update & ISP32_MODULE_RAWHIST3))
		ops->rawhst3_config(params_vdev, &new_params->meas.rawhist3);

	if ((module_cfg_update & ISP32_MODULE_RAWAWB) ||
	    ((priv_val->buf_info_owner == RKISP_INFO2DRR_OWNER_AWB) &&
	     !(isp3_param_read(params_vdev, ISP3X_RAWAWB_CTRL) & ISP32_RAWAWB_2DDR_PATH_EN)))
		ops->rawawb_config(params_vdev, &new_params->meas.rawawb);
}

static __maybe_unused
void __isp_isr_meas_en(struct rkisp_isp_params_vdev *params_vdev,
		       struct isp32_isp_params_cfg *new_params,
		       enum rkisp_params_type type)
{
	struct rkisp_isp_params_ops_v32 *ops =
		(struct rkisp_isp_params_ops_v32 *)params_vdev->priv_ops;
	u64 module_en_update = new_params->module_en_update;
	u64 module_ens = new_params->module_ens;

	if (type == RKISP_PARAMS_SHD)
		return;

	v4l2_dbg(4, rkisp_debug, &params_vdev->dev->v4l2_dev,
		 "%s seq:%d module_en_update:0x%llx module_ens:0x%llx\n",
		 __func__, new_params->frame_id, module_en_update, module_ens);

	if (module_en_update & ISP32_MODULE_RAWAF)
		ops->rawaf_enable(params_vdev, !!(module_ens & ISP32_MODULE_RAWAF));

	if (module_en_update & ISP32_MODULE_RAWAE0)
		ops->rawae0_enable(params_vdev, !!(module_ens & ISP32_MODULE_RAWAE0));

	if (module_en_update & ISP32_MODULE_RAWAE1)
		ops->rawae1_enable(params_vdev, !!(module_ens & ISP32_MODULE_RAWAE1));

	if (module_en_update & ISP32_MODULE_RAWAE2)
		ops->rawae2_enable(params_vdev, !!(module_ens & ISP32_MODULE_RAWAE2));

	if ((module_en_update & ISP32_MODULE_RAWAE3) && !params_vdev->afaemode_en)
		ops->rawae3_enable(params_vdev, !!(module_ens & ISP32_MODULE_RAWAE3));

	if (module_en_update & ISP32_MODULE_RAWHIST0)
		ops->rawhst0_enable(params_vdev, !!(module_ens & ISP32_MODULE_RAWHIST0));

	if (module_en_update & ISP32_MODULE_RAWHIST1)
		ops->rawhst1_enable(params_vdev, !!(module_ens & ISP32_MODULE_RAWHIST1));

	if (module_en_update & ISP32_MODULE_RAWHIST2)
		ops->rawhst2_enable(params_vdev, !!(module_ens & ISP32_MODULE_RAWHIST2));

	if (module_en_update & ISP32_MODULE_RAWHIST3)
		ops->rawhst3_enable(params_vdev, !!(module_ens & ISP32_MODULE_RAWHIST3));

	if (module_en_update & ISP32_MODULE_RAWAWB)
		ops->rawawb_enable(params_vdev, !!(module_ens & ISP32_MODULE_RAWAWB));
}

static __maybe_unused
void __isp_config_hdrshd(struct rkisp_isp_params_vdev *params_vdev)
{
	struct rkisp_isp_params_ops_v32 *ops =
		(struct rkisp_isp_params_ops_v32 *)params_vdev->priv_ops;
	struct rkisp_isp_params_val_v32 *priv_val =
		(struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;

	ops->hdrmge_config(params_vdev, &priv_val->last_hdrmge, RKISP_PARAMS_SHD);
	ops->hdrdrc_config(params_vdev, &priv_val->last_hdrdrc, RKISP_PARAMS_SHD);
}

static
void rkisp_params_cfgsram_v32(struct rkisp_isp_params_vdev *params_vdev)
{
	struct isp32_isp_params_cfg *params = params_vdev->isp32_params;

	isp_lsc_matrix_cfg_sram(params_vdev, &params->others.lsc_cfg, true);
	isp_rawhstbig_cfg_sram(params_vdev, &params->meas.rawhist1, 1, true);
	isp_rawhstbig_cfg_sram(params_vdev, &params->meas.rawhist2, 2, true);
	isp_rawhstbig_cfg_sram(params_vdev, &params->meas.rawhist3, 0, true);
	isp_rawawb_cfg_sram(params_vdev, &params->meas.rawawb, true);
}

static int
rkisp_alloc_internal_buf(struct rkisp_isp_params_vdev *params_vdev,
			 const struct isp32_isp_params_cfg *new_params)
{
	struct rkisp_device *dev = params_vdev->dev;
	struct rkisp_isp_subdev *isp_sdev = &dev->isp_sdev;
	struct rkisp_isp_params_val_v32 *priv_val;
	u64 module_en_update, module_ens;
	int ret, i;

	priv_val = (struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;
	module_en_update = new_params->module_en_update;
	module_ens = new_params->module_ens;

	priv_val->buf_3dlut_idx = 0;
	for (i = 0; i < ISP32_3DLUT_BUF_NUM; i++) {
		priv_val->buf_3dlut[i].is_need_vaddr = true;
		priv_val->buf_3dlut[i].size = ISP32_3DLUT_BUF_SIZE;
		ret = rkisp_alloc_buffer(dev, &priv_val->buf_3dlut[i]);
		if (ret) {
			dev_err(dev->dev, "alloc 3dlut buf fail:%d\n", ret);
			goto err_3dlut;
		}
	}

	if ((module_en_update & ISP32_MODULE_BAY3D) &&
	    (module_ens & ISP32_MODULE_BAY3D)) {
		bool is_hdr = !(dev->rd_mode == HDR_NORMAL || dev->rd_mode == HDR_RDBK_FRAME1);
		bool is_bwsaving = !!new_params->others.bay3d_cfg.bwsaving_en;
		bool is_glbpk = !!new_params->others.bay3d_cfg.glbpk_en;
		bool is_bwopt_dis = !!new_params->others.bay3d_cfg.bwopt_gain_dis;
		bool is_predgain = !!new_params->others.bls_cfg.isp_ob_predgain;
		u32 w = ALIGN(isp_sdev->in_crop.width, 16);
		u32 h = ALIGN(isp_sdev->in_crop.height, 16);
		u32 val, wrap_line, wsize, div;
		dma_addr_t dma_addr;

		priv_val->is_lo8x8 = (!new_params->others.bay3d_cfg.lo4x8_en &&
				      !new_params->others.bay3d_cfg.lo4x4_en);

		/*
		 * bwopt_dis one line image with one line pk gain
		 * other two line image with one line pk gain
		 */
		wsize = is_bwopt_dis ? w : w * 2;
		if (is_bwsaving)
			wsize = wsize * 3 / 4;
		/* pk gain to ddr */
		if (!is_glbpk)
			wsize += w / 8;
		/* pixel to Byte */
		wsize *= 2;
		div = is_bwopt_dis ? 1 : 2;
		val = ALIGN(wsize * h / div, 16);
		priv_val->buf_3dnr_iir.size = val;
		ret = rkisp_alloc_buffer(dev, &priv_val->buf_3dnr_iir);
		if (ret) {
			dev_err(dev->dev, "alloc bay3d iir buf fail:%d\n", ret);
			goto err_3dnr;
		}
		isp3_param_write(params_vdev, val, ISP3X_MI_BAY3D_IIR_WR_SIZE);
		val = priv_val->buf_3dnr_iir.dma_addr;
		isp3_param_write(params_vdev, val, ISP3X_MI_BAY3D_IIR_WR_BASE);
		isp3_param_write(params_vdev, val, ISP3X_MI_BAY3D_IIR_RD_BASE);

		div = priv_val->is_lo8x8 ? 64 : 16;
		val = w * h / div;
		/* pixel to Byte and align */
		val = ALIGN(val * 2, 16);
		priv_val->buf_3dnr_ds.size = val;
		ret = rkisp_alloc_buffer(dev, &priv_val->buf_3dnr_ds);
		if (ret) {
			rkisp_free_buffer(dev, &priv_val->buf_3dnr_iir);
			dev_err(dev->dev, "alloc bay3d ds buf fail:%d\n", ret);
			goto err_3dnr;
		}
		isp3_param_write(params_vdev, val, ISP3X_MI_BAY3D_DS_WR_SIZE);
		val = priv_val->buf_3dnr_ds.dma_addr;
		isp3_param_write(params_vdev, val, ISP3X_MI_BAY3D_DS_WR_BASE);
		isp3_param_write(params_vdev, val, ISP3X_MI_BAY3D_DS_RD_BASE);

		wrap_line = priv_val->is_lo8x8 ? 76 : 36;
		wsize = is_bwopt_dis ? w : w * 2;
		if (is_bwsaving)
			wsize = wsize * 3 / 4;
		if (is_hdr || is_predgain)
			wsize += w / 8;
		/* pixel to Byte and align */
		wsize = ALIGN(wsize * 2, 16);
		div = is_bwopt_dis ? 1 : 2;
		val = ALIGN(wsize * wrap_line / div, 16);
		priv_val->buf_3dnr_cur.size = val;
		if (val > dev->hw_dev->sram.size) {
			ret = rkisp_alloc_buffer(dev, &priv_val->buf_3dnr_cur);
			if (ret) {
				rkisp_free_buffer(dev, &priv_val->buf_3dnr_iir);
				rkisp_free_buffer(dev, &priv_val->buf_3dnr_ds);
				dev_err(dev->dev, "alloc bay3d cur buf fail:%d\n", ret);
				goto err_3dnr;
			}
			dma_addr = priv_val->buf_3dnr_cur.dma_addr;
			priv_val->is_sram = false;
		} else {
			dma_addr = dev->hw_dev->sram.dma_addr;
			priv_val->is_sram = true;
		}
		isp3_param_write(params_vdev, val, ISP3X_MI_BAY3D_CUR_WR_SIZE);
		isp3_param_write(params_vdev, val, ISP32_MI_BAY3D_CUR_RD_SIZE);
		isp3_param_write(params_vdev, wsize, ISP3X_MI_BAY3D_CUR_WR_LENGTH);
		isp3_param_write(params_vdev, wsize, ISP3X_MI_BAY3D_CUR_RD_LENGTH);
		isp3_param_write(params_vdev, dma_addr, ISP3X_MI_BAY3D_CUR_WR_BASE);
		isp3_param_write(params_vdev, dma_addr, ISP3X_MI_BAY3D_CUR_RD_BASE);
		val = wrap_line << 16 | 28;
		isp3_param_write(params_vdev, val, ISP3X_BAY3D_MI_ST);
	}
	return 0;
err_3dnr:
	i = ISP32_3DLUT_BUF_NUM;
err_3dlut:
	for (i -= 1; i >= 0; i--)
		rkisp_free_buffer(dev, &priv_val->buf_3dlut[i]);
	return ret;
}

static bool
rkisp_params_check_bigmode_v32(struct rkisp_isp_params_vdev *params_vdev)
{
	struct rkisp_device *ispdev = params_vdev->dev;
	struct device *dev = params_vdev->dev->dev;
	struct rkisp_hw_dev *hw = params_vdev->dev->hw_dev;
	struct v4l2_rect *crop = &params_vdev->dev->isp_sdev.in_crop;
	u32 width = hw->max_in.w, height = hw->max_in.h, size = width * height;
	u32 bigmode_max_w, bigmode_max_size;
	int k = 0, idx1[DEV_MAX] = { 0 };
	int n = 0, idx2[DEV_MAX] = { 0 };
	int i = 0, j = 0;
	bool is_bigmode = false;

multi_overflow:
	if (hw->is_multi_overflow) {
		ispdev->multi_index = 0;
		ispdev->multi_mode = 0;
		bigmode_max_w = ISP32_AUTO_BIGMODE_WIDTH;
		bigmode_max_size = ISP32_NOBIG_OVERFLOW_SIZE;
		dev_warn(dev, "over virtual isp max resolution, force to 2 readback\n");
		goto end;
	}

	switch (hw->dev_link_num) {
	case 4:
		bigmode_max_w = ISP32_VIR4_AUTO_BIGMODE_WIDTH;
		bigmode_max_size = ISP32_VIR4_NOBIG_OVERFLOW_SIZE;
		ispdev->multi_index = ispdev->dev_id;
		ispdev->multi_mode = 2;
		/* internal buf of hw divided to four parts
		 *             bigmode             nobigmode
		 *  _________  max width:1280      max width:640
		 * |_sensor0_| max size:1280*800  max size:640*400
		 * |_sensor1_| max size:1280*800  max size:640*400
		 * |_sensor2_| max size:1280*800  max size:640*400
		 * |_sensor3_| max size:1280*800  max size:640*400
		 */
		for (i = 0; i < hw->dev_num; i++) {
			if (hw->isp_size[i].w <= ISP32_VIR4_MAX_WIDTH &&
			    hw->isp_size[i].size <= ISP32_VIR4_MAX_SIZE)
				continue;
			dev_warn(dev, "isp%d %dx%d over four vir isp max:1280x800\n",
				 i, hw->isp_size[i].w, hw->isp_size[i].h);
			hw->is_multi_overflow = true;
			goto multi_overflow;
		}
		break;
	case 3:
		bigmode_max_w = ISP32_VIR4_AUTO_BIGMODE_WIDTH;
		bigmode_max_size = ISP32_VIR4_NOBIG_OVERFLOW_SIZE;
		ispdev->multi_index = ispdev->dev_id;
		ispdev->multi_mode = 2;
		/* case0:      bigmode             nobigmode
		 *  _________  max width:1280      max width:640
		 * |_sensor0_| max size:1280*800  max size:640*400
		 * |_sensor1_| max size:1280*800  max size:640*400
		 * |_sensor2_| max size:1280*800  max size:640*400
		 * |_________|
		 *
		 * case1:      bigmode               special reg cfg
		 *  _________  max width:3072
		 * | sensor0 | max size:1920*1080   mode=0 index=0
		 * |_________|
		 * |_sensor1_| max size:1280*800    mode=2 index=2
		 * |_sensor2_| max size:1280*800    mode=2 index=3
		 *             max width:1280
		 */
		for (i = 0; i < hw->dev_num; i++) {
			if (!hw->isp_size[i].size) {
				if (i < hw->dev_link_num)
					idx2[n++] = i;
				continue;
			}
			if (hw->isp_size[i].w <= ISP32_VIR4_MAX_WIDTH &&
			    hw->isp_size[i].size <= ISP32_VIR4_MAX_SIZE)
				continue;
			idx1[k++] = i;
		}
		if (k) {
			is_bigmode = true;
			if (k != 1 ||
			    (hw->isp_size[idx1[0]].size > ISP32_VIR2_MAX_SIZE)) {
				dev_warn(dev, "isp%d %dx%d over three vir isp max:1280x800\n",
					 idx1[0], hw->isp_size[idx1[0]].w, hw->isp_size[idx1[0]].h);
				hw->is_multi_overflow = true;
				goto multi_overflow;
			} else {
				if (idx1[0] == ispdev->dev_id) {
					ispdev->multi_mode = 0;
					ispdev->multi_index = 0;
				} else {
					ispdev->multi_mode = 2;
					if (ispdev->multi_index == 0 ||
					    ispdev->multi_index == 1)
						ispdev->multi_index = 3;
				}
			}
		} else if (ispdev->multi_index >= hw->dev_link_num) {
			ispdev->multi_index = idx2[ispdev->multi_index - hw->dev_link_num];
		}
		break;
	case 2:
		bigmode_max_w = ISP32_VIR2_AUTO_BIGMODE_WIDTH;
		bigmode_max_size = ISP32_VIR2_NOBIG_OVERFLOW_SIZE;
		ispdev->multi_index = ispdev->dev_id;
		ispdev->multi_mode = 1;
		/* case0:      bigmode            nobigmode
		 *  _________  max width:1920     max width:960
		 * | sensor0 | max size:1920*1080 max size:960*540
		 * |_________|
		 * | sensor1 | max size:1920*1080 max size:960*540
		 * |_________|
		 *
		 * case1:      bigmode              special reg cfg
		 *  _________  max width:3072
		 * | sensor0 | max size:           mode=0 index=0
		 * |         | 1920*1080+1280*800
		 * |_________|
		 * |_sensor1_| max size:1280*800   mode=2 index=3
		 *             max width:1280
		 */
		for (i = 0; i < hw->dev_num; i++) {
			if (!hw->isp_size[i].size) {
				if (i < hw->dev_link_num)
					idx2[n++] = i;
				continue;
			}
			if (hw->isp_size[i].w <= ISP32_VIR2_MAX_WIDTH &&
			    hw->isp_size[i].size <= ISP32_VIR2_MAX_SIZE) {
				if (hw->isp_size[i].w > ISP32_VIR4_MAX_WIDTH ||
				    hw->isp_size[i].size > ISP32_VIR4_MAX_SIZE)
					j++;
				continue;
			}
			idx1[k++] = i;
		}
		if (k) {
			is_bigmode = true;
			if (k == 2 || j ||
			    hw->isp_size[idx1[k - 1]].size > (ISP32_VIR4_MAX_SIZE + ISP32_VIR2_MAX_SIZE)) {
				dev_warn(dev, "isp%d %dx%d over two vir isp max:1920x1080\n",
					 idx1[k - 1], hw->isp_size[idx1[k - 1]].w, hw->isp_size[idx1[k - 1]].h);
				hw->is_multi_overflow = true;
				goto multi_overflow;
			} else {
				if (idx1[0] == ispdev->dev_id) {
					ispdev->multi_mode = 0;
					ispdev->multi_index = 0;
				} else {
					ispdev->multi_mode = 2;
					ispdev->multi_index = 3;
				}
			}
		} else if (ispdev->multi_index >= hw->dev_link_num) {
			ispdev->multi_index = idx2[ispdev->multi_index - hw->dev_link_num];
		}
		break;
	default:
		bigmode_max_w = ISP32_AUTO_BIGMODE_WIDTH;
		bigmode_max_size = ISP32_NOBIG_OVERFLOW_SIZE;
		ispdev->multi_mode = 0;
		ispdev->multi_index = 0;
		width = crop->width;
		height = crop->height;
		size = width * height;
		break;
	}

end:
	if (!is_bigmode &&
	    (width > bigmode_max_w || size > bigmode_max_size))
		is_bigmode = true;
	return ispdev->is_bigmode = is_bigmode;
}

/* Not called when the camera active, thus not isr protection. */
static void
rkisp_params_first_cfg_v32(struct rkisp_isp_params_vdev *params_vdev)
{
	struct rkisp_device *dev = params_vdev->dev;
	struct rkisp_isp_params_val_v32 *priv_val =
		(struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;

	dev->is_bigmode = rkisp_params_check_bigmode_v32(params_vdev);
	spin_lock(&params_vdev->config_lock);
	/* override the default things */
	if (!params_vdev->isp32_params->module_cfg_update &&
	    !params_vdev->isp32_params->module_en_update)
		dev_warn(dev->dev, "can not get first iq setting in stream on\n");

	priv_val->bay3d_en = 0;
	priv_val->dhaz_en = 0;
	priv_val->drc_en = 0;
	priv_val->lsc_en = 0;
	priv_val->mge_en = 0;
	priv_val->lut3d_en = 0;
	if (dev->is_bigmode)
		rkisp_set_bits(params_vdev->dev, ISP3X_ISP_CTRL1, 0,
			       ISP3X_BIGMODE_MANUAL | ISP3X_BIGMODE_FORCE_EN, false);

	__isp_isr_meas_config(params_vdev, params_vdev->isp32_params, RKISP_PARAMS_ALL);
	__isp_isr_other_config(params_vdev, params_vdev->isp32_params, RKISP_PARAMS_ALL);
	__isp_isr_other_en(params_vdev, params_vdev->isp32_params, RKISP_PARAMS_ALL);
	__isp_isr_meas_en(params_vdev, params_vdev->isp32_params, RKISP_PARAMS_ALL);

	priv_val->cur_hdrmge = params_vdev->isp32_params->others.hdrmge_cfg;
	priv_val->cur_hdrdrc = params_vdev->isp32_params->others.drc_cfg;
	priv_val->last_hdrmge = priv_val->cur_hdrmge;
	priv_val->last_hdrdrc = priv_val->cur_hdrdrc;
	spin_unlock(&params_vdev->config_lock);

	if (dev->hw_dev->is_single && (dev->isp_state & ISP_START))
		rkisp_set_bits(dev, ISP3X_ISP_CTRL0, 0, CIF_ISP_CTRL_ISP_CFG_UPD, true);
}

static void rkisp_save_first_param_v32(struct rkisp_isp_params_vdev *params_vdev, void *param)
{
	struct rkisp_isp_params_val_v32 *priv_val =
		(struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;

	memcpy(params_vdev->isp32_params, param, params_vdev->vdev_fmt.fmt.meta.buffersize);

	if (!params_vdev->first_params)
		return;
	tasklet_enable(&priv_val->lsc_tasklet);
	rkisp_alloc_internal_buf(params_vdev, params_vdev->isp32_params);
}

static void rkisp_clear_first_param_v32(struct rkisp_isp_params_vdev *params_vdev)
{
	memset(params_vdev->isp32_params, 0, sizeof(struct isp32_isp_params_cfg));
}

static void rkisp_deinit_mesh_buf(struct rkisp_isp_params_vdev *params_vdev,
				  u64 module_id)
{
	struct rkisp_isp_params_val_v32 *priv_val;
	struct rkisp_dummy_buffer *buf;
	int i;

	priv_val = params_vdev->priv_val;
	if (!priv_val)
		return;

	switch (module_id) {
	case ISP32_MODULE_CAC:
		buf = priv_val->buf_cac;
		break;
	case ISP32_MODULE_LDCH:
	default:
		buf = priv_val->buf_ldch;
		break;
	}

	for (i = 0; i < ISP32_MESH_BUF_NUM; i++)
		rkisp_free_buffer(params_vdev->dev, buf + i);
}

static int rkisp_init_mesh_buf(struct rkisp_isp_params_vdev *params_vdev,
			       struct rkisp_meshbuf_size *meshsize)
{
	struct rkisp_device *ispdev = params_vdev->dev;
	struct device *dev = ispdev->dev;
	struct rkisp_isp_params_val_v32 *priv_val;
	struct isp2x_mesh_head *mesh_head;
	struct rkisp_dummy_buffer *buf;
	u32 mesh_w = meshsize->meas_width;
	u32 mesh_h = meshsize->meas_height;
	u32 mesh_size, buf_size;
	int i, ret, buf_cnt = meshsize->buf_cnt;

	priv_val = params_vdev->priv_val;
	if (!priv_val) {
		dev_err(dev, "priv_val is NULL\n");
		return -EINVAL;
	}

	switch (meshsize->module_id) {
	case ISP32_MODULE_CAC:
		priv_val->buf_cac_idx = 0;
		buf = priv_val->buf_cac;
		mesh_w = (mesh_w + 62) / 64 * 9;
		mesh_h = (mesh_h + 62) / 64 * 2;
		mesh_size = mesh_w * 4 * mesh_h;
		break;
	case ISP32_MODULE_LDCH:
	default:
		priv_val->buf_ldch_idx = 0;
		buf = priv_val->buf_ldch;
		mesh_w = ((mesh_w + 15) / 16 + 2) / 2;
		mesh_h = (mesh_h + 7) / 8 + 1;
		mesh_size = mesh_w * 4 * mesh_h;
		break;
	}

	if (buf_cnt <= 0 || buf_cnt > ISP32_MESH_BUF_NUM)
		buf_cnt = ISP32_MESH_BUF_NUM;
	buf_size = PAGE_ALIGN(mesh_size + ALIGN(sizeof(struct isp2x_mesh_head), 16));
	for (i = 0; i < buf_cnt; i++) {
		buf->is_need_vaddr = true;
		buf->is_need_dbuf = true;
		buf->is_need_dmafd = true;
		buf->size = buf_size;
		ret = rkisp_alloc_buffer(params_vdev->dev, buf);
		if (ret) {
			dev_err(dev, "%s failed\n", __func__);
			goto err;
		}

		mesh_head = (struct isp2x_mesh_head *)buf->vaddr;
		mesh_head->stat = MESH_BUF_INIT;
		mesh_head->data_oft = ALIGN(sizeof(struct isp2x_mesh_head), 16);
		buf++;
	}

	return 0;
err:
	rkisp_deinit_mesh_buf(params_vdev, meshsize->module_id);
	return -ENOMEM;
}

static void
rkisp_get_param_size_v32(struct rkisp_isp_params_vdev *params_vdev,
			 unsigned int sizes[])
{
	sizes[0] = sizeof(struct isp32_isp_params_cfg);
}

static void
rkisp_params_get_meshbuf_inf_v32(struct rkisp_isp_params_vdev *params_vdev,
				 void *meshbuf_inf)
{
	struct rkisp_isp_params_val_v32 *priv_val;
	struct rkisp_meshbuf_info *meshbuf = meshbuf_inf;
	struct rkisp_dummy_buffer *buf;
	int i;

	priv_val = params_vdev->priv_val;
	switch (meshbuf->module_id) {
	case ISP32_MODULE_CAC:
		priv_val->buf_cac_idx = 0;
		buf = priv_val->buf_cac;
		break;
	case ISP32_MODULE_LDCH:
	default:
		priv_val->buf_ldch_idx = 0;
		buf = priv_val->buf_ldch;
		break;
	}

	for (i = 0; i < ISP32_MESH_BUF_NUM; i++) {
		if (!buf->mem_priv) {
			meshbuf->buf_fd[i] = -1;
			meshbuf->buf_size[i] = 0;
		} else {
			meshbuf->buf_fd[i] = buf->dma_fd;
			meshbuf->buf_size[i] = buf->size;
		}
		buf++;
	}
}

static void
rkisp_params_set_meshbuf_size_v32(struct rkisp_isp_params_vdev *params_vdev,
				  void *size)
{
	struct rkisp_meshbuf_size *meshsize = size;

	rkisp_deinit_mesh_buf(params_vdev, meshsize->module_id);
	rkisp_init_mesh_buf(params_vdev, meshsize);
}

static void
rkisp_params_free_meshbuf_v32(struct rkisp_isp_params_vdev *params_vdev,
			      u64 module_id)
{
	rkisp_deinit_mesh_buf(params_vdev, module_id);
}

static int
rkisp_params_info2ddr_cfg_v32(struct rkisp_isp_params_vdev *params_vdev, void *arg)
{
	struct rkisp_device *dev = params_vdev->dev;
	struct rkisp_isp_params_val_v32 *priv_val;
	struct rkisp_info2ddr *cfg = arg;
	struct rkisp_dummy_buffer *buf;
	u32 reg, ctrl, mask, size, val, wsize = 0, vsize = 0;
	int i, ret;

	priv_val = params_vdev->priv_val;

	if (cfg->buf_cnt > RKISP_INFO2DDR_BUF_MAX)
		cfg->buf_cnt = RKISP_INFO2DDR_BUF_MAX;
	else if (cfg->buf_cnt == 0)
		cfg->buf_cnt = 1;
	for (val = 0; val < cfg->buf_cnt; val++)
		cfg->buf_fd[val] = -1;

	switch (cfg->owner) {
	case RKISP_INFO2DRR_OWNER_NULL:
		rkisp_clear_reg_cache_bits(dev, ISP3X_RAWAWB_CTRL,
					   ISP32_RAWAWB_2DDR_PATH_EN);
		rkisp_clear_reg_cache_bits(dev, ISP3X_GAIN_CTRL,
					   ISP3X_GAIN_2DDR_EN);
		priv_val->buf_info_owner = cfg->owner;
		return 0;
	case RKISP_INFO2DRR_OWNER_GAIN:
		ctrl = ISP3X_GAIN_2DDR_mode(cfg->u.gain.gain2ddr_mode);
		ctrl |= ISP3X_GAIN_2DDR_EN;
		mask = ISP3X_GAIN_2DDR_mode(3);
		reg = ISP3X_GAIN_CTRL;

		if (cfg->wsize)
			wsize = (cfg->wsize + 7) / 8;
		else
			wsize = (dev->isp_sdev.in_crop.width + 7) / 8;
		/* 0 or 3: 4x8mode, 1: 2x8 mode, 2: 1x8mode */
		val = cfg->u.gain.gain2ddr_mode;
		val = (val == 1) ? 2 : ((val == 2) ? 1 : 4);
		if (cfg->vsize)
			vsize = cfg->vsize;
		else
			vsize = dev->isp_sdev.in_crop.height / val;
		break;
	case RKISP_INFO2DRR_OWNER_AWB:
		ctrl = cfg->u.awb.awb2ddr_sel ? ISP32_RAWAWB_2DDR_PATH_DS : 0;
		ctrl |= ISP32_RAWAWB_2DDR_PATH_EN;
		mask = ISP32_RAWAWB_2DDR_PATH_DS;
		reg = ISP3X_RAWAWB_CTRL;

		val = cfg->u.awb.awb2ddr_sel ? 8 : 1;
		if (cfg->wsize)
			wsize = cfg->wsize;
		else
			wsize = dev->isp_sdev.in_crop.width * 4 / val;
		if (cfg->vsize)
			vsize = cfg->vsize;
		else
			vsize = dev->isp_sdev.in_crop.height / val;
		break;
	default:
		dev_err(dev->dev, "%s no support owner:%d\n", __func__, cfg->owner);
		return -EINVAL;
	}

	if (!wsize || !vsize) {
		dev_err(dev->dev, "%s inval wsize:%d vsize:%d\n", __func__, wsize, vsize);
		return -EINVAL;
	}

	wsize = ALIGN(wsize, 16);
	size = wsize * vsize;
	for (i = 0; i < cfg->buf_cnt; i++) {
		buf = &priv_val->buf_info[i];
		if (buf->mem_priv)
			rkisp_free_buffer(dev, buf);
		buf->size = size;
		buf->is_need_dbuf = true;
		buf->is_need_dmafd = true;
		buf->is_need_vaddr = true;
		ret = rkisp_alloc_buffer(dev, buf);
		if (ret) {
			dev_err(dev->dev, "%s alloc buf failed\n", __func__);
			goto err;
		}
		*(u32 *)buf->vaddr = RKISP_INFO2DDR_BUF_INIT;
		cfg->buf_fd[i] = buf->dma_fd;
	}
	buf = &priv_val->buf_info[0];
	isp3_param_write(params_vdev, buf->dma_addr, ISP3X_MI_GAIN_WR_BASE);
	isp3_param_write(params_vdev, buf->size, ISP3X_MI_GAIN_WR_SIZE);
	isp3_param_write(params_vdev, wsize, ISP3X_MI_GAIN_WR_LENGTH);
	if (dev->hw_dev->is_single)
		rkisp_write(dev, ISP3X_MI_WR_CTRL2, ISP3X_GAINSELF_UPD, true);
	rkisp_set_reg_cache_bits(dev, reg, mask, ctrl);

	priv_val->buf_info_idx = 0;
	priv_val->buf_info_cnt = cfg->buf_cnt;
	priv_val->buf_info_owner = cfg->owner;

	cfg->wsize = wsize;
	cfg->vsize = vsize;
	return 0;
err:
	for (i -= 1; i >= 0; i--) {
		buf = &priv_val->buf_info[i];
		rkisp_free_buffer(dev, buf);
		cfg->buf_fd[i] = -1;
	}
	cfg->owner = RKISP_INFO2DRR_OWNER_NULL;
	cfg->buf_cnt = 0;
	return -ENOMEM;
}

static void
rkisp_params_stream_stop_v32(struct rkisp_isp_params_vdev *params_vdev)
{
	struct rkisp_device *ispdev = params_vdev->dev;
	struct rkisp_isp_params_val_v32 *priv_val;
	int i;

	priv_val = (struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;
	tasklet_disable(&priv_val->lsc_tasklet);
	rkisp_free_buffer(ispdev, &priv_val->buf_3dnr_iir);
	rkisp_free_buffer(ispdev, &priv_val->buf_3dnr_cur);
	rkisp_free_buffer(ispdev, &priv_val->buf_3dnr_ds);
	for (i = 0; i < ISP32_3DLUT_BUF_NUM; i++)
		rkisp_free_buffer(ispdev, &priv_val->buf_3dlut[i]);
	for (i = 0; i < RKISP_STATS_DDR_BUF_NUM; i++)
		rkisp_free_buffer(ispdev, &ispdev->stats_vdev.stats_buf[i]);
	priv_val->buf_info_owner = 0;
	priv_val->buf_info_cnt = 0;
	priv_val->buf_info_idx = -1;
	for (i = 0; i < RKISP_INFO2DDR_BUF_MAX; i++)
		rkisp_free_buffer(ispdev, &priv_val->buf_info[i]);
}

static void
rkisp_params_fop_release_v32(struct rkisp_isp_params_vdev *params_vdev)
{
	rkisp_deinit_mesh_buf(params_vdev, ISP32_MODULE_LDCH);
	rkisp_deinit_mesh_buf(params_vdev, ISP32_MODULE_CAC);
}

/* Not called when the camera active, thus not isr protection. */
static void
rkisp_params_disable_isp_v32(struct rkisp_isp_params_vdev *params_vdev)
{
	params_vdev->isp32_params->module_ens = 0;
	params_vdev->isp32_params->module_en_update = 0x7ffffffffff;

	__isp_isr_other_en(params_vdev, params_vdev->isp32_params, RKISP_PARAMS_ALL);
	__isp_isr_meas_en(params_vdev, params_vdev->isp32_params, RKISP_PARAMS_ALL);
}

static void
module_data_abandon(struct rkisp_isp_params_vdev *params_vdev,
		    struct isp32_isp_params_cfg *params)
{
	struct rkisp_isp_params_val_v32 *priv_val;
	struct isp2x_mesh_head *mesh_head;
	int i;

	priv_val = (struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;
	if (params->module_cfg_update & ISP32_MODULE_LDCH) {
		const struct isp32_ldch_cfg *arg = &params->others.ldch_cfg;

		for (i = 0; i < ISP32_MESH_BUF_NUM; i++) {
			if (priv_val->buf_ldch[i].vaddr &&
			    arg->buf_fd == priv_val->buf_ldch[i].dma_fd) {
				mesh_head = (struct isp2x_mesh_head *)priv_val->buf_ldch[i].vaddr;
				mesh_head->stat = MESH_BUF_CHIPINUSE;
				break;
			}
		}
	}

	if (params->module_cfg_update & ISP32_MODULE_CAC) {
		const struct isp32_cac_cfg *arg = &params->others.cac_cfg;

		for (i = 0; i < ISP32_MESH_BUF_NUM; i++) {
			if (priv_val->buf_cac[i].vaddr &&
			    arg->buf_fd == priv_val->buf_cac[i].dma_fd) {
				mesh_head = (struct isp2x_mesh_head *)priv_val->buf_cac[i].vaddr;
				mesh_head->stat = MESH_BUF_CHIPINUSE;
				break;
			}
		}
	}
}

static void
rkisp_params_cfg_v32(struct rkisp_isp_params_vdev *params_vdev,
		     u32 frame_id, enum rkisp_params_type type)
{
	struct isp32_isp_params_cfg *new_params = NULL;
	struct rkisp_buffer *cur_buf = params_vdev->cur_buf;
	struct rkisp_device *dev = params_vdev->dev;
	struct rkisp_hw_dev *hw_dev = dev->hw_dev;

	spin_lock(&params_vdev->config_lock);
	if (!params_vdev->streamon)
		goto unlock;

	/* get buffer by frame_id */
	while (!list_empty(&params_vdev->params) && !cur_buf) {
		cur_buf = list_first_entry(&params_vdev->params,
				struct rkisp_buffer, queue);

		new_params = (struct isp32_isp_params_cfg *)(cur_buf->vaddr[0]);
		if (new_params->frame_id < frame_id) {
			list_del(&cur_buf->queue);
			if (list_empty(&params_vdev->params))
				break;
			else if (new_params->module_en_update ||
				 (new_params->module_cfg_update & ISP32_MODULE_FORCE)) {
				/* update en immediately */
				__isp_isr_meas_config(params_vdev, new_params, type);
				__isp_isr_other_config(params_vdev, new_params, type);
				__isp_isr_other_en(params_vdev, new_params, type);
				__isp_isr_meas_en(params_vdev, new_params, type);
				new_params->module_cfg_update = 0;
			}
			if (new_params->module_cfg_update &
			    (ISP32_MODULE_LDCH | ISP32_MODULE_CAC)) {
				module_data_abandon(params_vdev, new_params);
			}
			vb2_buffer_done(&cur_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
			cur_buf = NULL;
			continue;
		} else if (new_params->frame_id == frame_id) {
			list_del(&cur_buf->queue);
		} else {
			cur_buf = NULL;
		}
		break;
	}

	if (!cur_buf)
		goto unlock;

	new_params = (struct isp32_isp_params_cfg *)(cur_buf->vaddr[0]);
	__isp_isr_meas_config(params_vdev, new_params, type);
	__isp_isr_other_config(params_vdev, new_params, type);
	__isp_isr_other_en(params_vdev, new_params, type);
	__isp_isr_meas_en(params_vdev, new_params, type);
	if (!hw_dev->is_single && type != RKISP_PARAMS_SHD)
		__isp_config_hdrshd(params_vdev);

	if (type != RKISP_PARAMS_IMD) {
		struct rkisp_isp_params_val_v32 *priv_val =
			(struct rkisp_isp_params_val_v32 *)params_vdev->priv_val;

		priv_val->last_hdrmge = priv_val->cur_hdrmge;
		priv_val->last_hdrdrc = priv_val->cur_hdrdrc;
		priv_val->cur_hdrmge = new_params->others.hdrmge_cfg;
		priv_val->cur_hdrdrc = new_params->others.drc_cfg;
		new_params->module_cfg_update = 0;
		vb2_buffer_done(&cur_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
		cur_buf = NULL;
	}

unlock:
	params_vdev->cur_buf = cur_buf;
	spin_unlock(&params_vdev->config_lock);
}

static void
rkisp_params_clear_fstflg(struct rkisp_isp_params_vdev *params_vdev)
{
	u32 value = isp3_param_read(params_vdev, ISP3X_ISP_CTRL1);

	value &= (ISP3X_YNR_FST_FRAME | ISP3X_ADRC_FST_FRAME |
		  ISP3X_DHAZ_FST_FRAME | ISP3X_CNR_FST_FRAME |
		  ISP3X_RAW3D_FST_FRAME);
	if (value) {
		isp3_param_clear_bits(params_vdev, ISP3X_ISP_CTRL1, value);
	}
}

static void
rkisp_params_isr_v32(struct rkisp_isp_params_vdev *params_vdev,
		     u32 isp_mis)
{
	struct rkisp_device *dev = params_vdev->dev;
	u32 cur_frame_id;

	rkisp_dmarx_get_frame(dev, &cur_frame_id, NULL, NULL, true);
	if (isp_mis & CIF_ISP_V_START) {
		if (params_vdev->rdbk_times)
			params_vdev->rdbk_times--;
		if (!params_vdev->cur_buf)
			return;

		if (IS_HDR_RDBK(dev->rd_mode) && !params_vdev->rdbk_times) {
			rkisp_params_cfg_v32(params_vdev, cur_frame_id, RKISP_PARAMS_SHD);
			return;
		}
	}

	if ((isp_mis & CIF_ISP_FRAME) && !params_vdev->rdbk_times)
		rkisp_params_clear_fstflg(params_vdev);

	if ((isp_mis & CIF_ISP_FRAME) && !IS_HDR_RDBK(dev->rd_mode))
		rkisp_params_cfg_v32(params_vdev, cur_frame_id + 1, RKISP_PARAMS_ALL);
}

static struct rkisp_isp_params_ops rkisp_isp_params_ops_tbl = {
	.save_first_param = rkisp_save_first_param_v32,
	.clear_first_param = rkisp_clear_first_param_v32,
	.get_param_size = rkisp_get_param_size_v32,
	.first_cfg = rkisp_params_first_cfg_v32,
	.disable_isp = rkisp_params_disable_isp_v32,
	.isr_hdl = rkisp_params_isr_v32,
	.param_cfg = rkisp_params_cfg_v32,
	.param_cfgsram = rkisp_params_cfgsram_v32,
	.get_meshbuf_inf = rkisp_params_get_meshbuf_inf_v32,
	.set_meshbuf_size = rkisp_params_set_meshbuf_size_v32,
	.free_meshbuf = rkisp_params_free_meshbuf_v32,
	.stream_stop = rkisp_params_stream_stop_v32,
	.fop_release = rkisp_params_fop_release_v32,
	.check_bigmode = rkisp_params_check_bigmode_v32,
	.info2ddr_cfg = rkisp_params_info2ddr_cfg_v32,
};

int rkisp_init_params_vdev_v32(struct rkisp_isp_params_vdev *params_vdev)
{
	struct rkisp_isp_params_val_v32 *priv_val;
	int size;

	priv_val = kzalloc(sizeof(*priv_val), GFP_KERNEL);
	if (!priv_val)
		return -ENOMEM;

	size = sizeof(struct isp32_isp_params_cfg);
	params_vdev->isp32_params = vmalloc(size);
	if (!params_vdev->isp32_params) {
		kfree(priv_val);
		return -ENOMEM;
	}

	params_vdev->priv_val = (void *)priv_val;
	params_vdev->ops = &rkisp_isp_params_ops_tbl;
	params_vdev->priv_ops = &isp_params_ops_v32;
	rkisp_clear_first_param_v32(params_vdev);
	tasklet_init(&priv_val->lsc_tasklet,
		     isp_lsc_cfg_sram_task,
		     (unsigned long)params_vdev);
	tasklet_disable(&priv_val->lsc_tasklet);
	priv_val->buf_info_owner = 0;
	priv_val->buf_info_cnt = 0;
	priv_val->buf_info_idx = -1;
	return 0;
}

void rkisp_uninit_params_vdev_v32(struct rkisp_isp_params_vdev *params_vdev)
{
	struct rkisp_isp_params_val_v32 *priv_val = params_vdev->priv_val;

	if (params_vdev->isp32_params)
		vfree(params_vdev->isp32_params);
	if (priv_val) {
		tasklet_kill(&priv_val->lsc_tasklet);
		kfree(priv_val);
		params_vdev->priv_val = NULL;
	}
}
