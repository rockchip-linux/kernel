// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2019 Fuzhou Rockchip Electronics Co., Ltd. */

#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-common.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include <linux/dma-iommu.h>
#include <linux/rk-camera-module.h>
#include "dev.h"
#include "regs.h"

static inline
struct rkisp_bridge_buf *to_bridge_buf(struct rkisp_ispp_buf *dbufs)
{
	return container_of(dbufs, struct rkisp_bridge_buf, dbufs);
}

/* compatible with MI frame end are triggered before ISP frame end */
static void reg_buf_wait_for_stats(struct rkisp_bridge_device *dev,
				   struct rkisp_ispp_reg *reg_buf,
				   struct rkisp_isp2x_stat_buffer *tmp_statsbuf)
{
	s32 retry = 10;

	do {
		if (reg_buf->frame_id > tmp_statsbuf->frame_id)
			usleep_range(1000, 1200);
		else
			break;
	} while (retry-- > 0);

	if (retry < 0)
		v4l2_err(&dev->sd, "reg id(%d) don't match stats id(%d)\n",
			 reg_buf->frame_id, tmp_statsbuf->frame_id);
}

static void dump_dbg_reg(struct rkisp_bridge_device *dev, struct rkisp_ispp_reg *reg_buf)
{
	struct rkisp_isp2x_stat_buffer *tmp_statsbuf;
	struct rkisp_hw_dev *hw = dev->ispdev->hw_dev;
	u32 offset = 0, size;

	tmp_statsbuf = (struct rkisp_isp2x_stat_buffer *)dev->ispdev->stats_vdev.tmp_statsbuf.vaddr;
	reg_buf_wait_for_stats(dev, reg_buf, tmp_statsbuf);
	memset(reg_buf->isp_offset, -1, sizeof(reg_buf->isp_offset));
	memset(reg_buf->ispp_offset, -1, sizeof(reg_buf->ispp_offset));
	memset(reg_buf->isp_size, 0, sizeof(reg_buf->isp_offset));
	memset(reg_buf->isp_stats_size, 0, sizeof(reg_buf->isp_offset));
	memset(reg_buf->ispp_size, 0, sizeof(reg_buf->ispp_offset));
	if (rkisp_debug_reg & ISP2X_MODULE_DPCC) {
		size = 4 + ISP_DPCC0_PDAF_FORWARD_MED - ISP_DPCC0_MODE;
		reg_buf->isp_size[ISP2X_ID_DPCC] = size;
		reg_buf->isp_offset[ISP2X_ID_DPCC] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_DPCC0_MODE, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_BLS) {
		size = 4 + ISP_BLS_D_MEASURED - ISP_BLS_CTRL;
		reg_buf->isp_size[ISP2X_ID_BLS] = size;
		reg_buf->isp_offset[ISP2X_ID_BLS] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_BLS_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_SDG) {
		size = 4 + ISP_GAMMA_B_Y_16 - ISP_GAMMA_DX_LO;
		reg_buf->isp_size[ISP2X_ID_SDG] = size;
		reg_buf->isp_offset[ISP2X_ID_SDG] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_GAMMA_DX_LO, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_SIHST) {
		size = 4 + ISP_HIST_HIST3_DBG2 - ISP_HIST_HIST_CTRL;
		reg_buf->isp_size[ISP2X_ID_SIHST] = size;
		reg_buf->isp_offset[ISP2X_ID_SIHST] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_HIST_HIST_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_LSC) {
		size = 4 + ISP_LSC_STATUS - ISP_LSC_CTRL;
		reg_buf->isp_size[ISP2X_ID_LSC] = size;
		reg_buf->isp_offset[ISP2X_ID_LSC] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_LSC_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_AWB_GAIN) {
		size = 4 + CIF_ISP_AWB_GAIN_RB_V12 - CIF_ISP_AWB_GAIN_G_V12;
		reg_buf->isp_size[ISP2X_ID_AWB_GAIN] = size;
		reg_buf->isp_offset[ISP2X_ID_AWB_GAIN] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + CIF_ISP_AWB_GAIN_G_V12, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_CCM) {
		size = 4 + ISP_CCM_BOUND_BIT - ISP_CCM_CTRL;
		reg_buf->isp_size[ISP2X_ID_CCM] = size;
		reg_buf->isp_offset[ISP2X_ID_CCM] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_CCM_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_GOC) {
		size = 4 + ISP_GAMMA_OUT_Y40 - ISP_GAMMA_OUT_CTRL;
		reg_buf->isp_size[ISP2X_ID_GOC] = size;
		reg_buf->isp_offset[ISP2X_ID_GOC] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_GAMMA_OUT_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_CPROC) {
		size = 4 + CPROC_HUE - CPROC_CTRL;
		reg_buf->isp_size[ISP2X_ID_CPROC] = size;
		reg_buf->isp_offset[ISP2X_ID_CPROC] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + CPROC_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_SIAF) {
		size = 4 + ISP_AFM_LUM_C - ISP_AFM_CTRL;
		reg_buf->isp_size[ISP2X_ID_SIAF] = size;
		reg_buf->isp_offset[ISP2X_ID_SIAF] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_AFM_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_SIAWB) {
		size = 4 + CIF_ISP_AWB_MEAN_V10 - CIF_ISP_AWB_PROP_V10;
		reg_buf->isp_size[ISP2X_ID_SIAWB] = size;
		reg_buf->isp_offset[ISP2X_ID_SIAWB] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + CIF_ISP_AWB_PROP_V10, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_IE) {
		size = 4 + CIF_IMG_EFF_SHARPEN - CIF_IMG_EFF_CTRL;
		reg_buf->isp_size[ISP2X_ID_IE] = size;
		reg_buf->isp_offset[ISP2X_ID_IE] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + CIF_IMG_EFF_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_YUVAE) {
		size = 4 + ISP_YUVAE_RO_DBG3 - ISP_YUVAE_CTRL;
		reg_buf->isp_size[ISP2X_ID_YUVAE] = size;
		reg_buf->isp_offset[ISP2X_ID_YUVAE] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_YUVAE_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_WDR) {
		size = 4 + ISP_WDR_BLKMEAN8_ROW9_4TO7 - ISP_WDR_CTRL;
		reg_buf->isp_size[ISP2X_ID_WDR] = size;
		reg_buf->isp_offset[ISP2X_ID_WDR] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_WDR_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_RK_IESHARP) {
		size = 4 + CIF_RKSHARP_UV_GAUSS_OTHER_COE33_COE35 - CIF_RKSHARP_CTRL;
		reg_buf->isp_size[ISP2X_ID_RK_IESHARP] = size;
		reg_buf->isp_offset[ISP2X_ID_RK_IESHARP] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + CIF_RKSHARP_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_RAWAF) {
		size = 4 + ISP_RAWAF_INT_STATE - ISP_RAWAF_CTRL;
		reg_buf->isp_size[ISP2X_ID_RAWAF] = size;
		reg_buf->isp_offset[ISP2X_ID_RAWAF] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_RAWAF_CTRL, size);
		offset += size;

		size = ISP2X_RAWAF_SUMDATA_NUM * sizeof(tmp_statsbuf->params.rawaf.ramdata[0]);
		reg_buf->isp_size[ISP2X_ID_RAWAF] += size;
		reg_buf->isp_stats_size[ISP2X_ID_RAWAF] = size;
		if (tmp_statsbuf->frame_id == reg_buf->frame_id)
			memcpy(&reg_buf->reg[offset], &tmp_statsbuf->params.rawaf.ramdata[0], size);
		else
			memset(&reg_buf->reg[offset], 0, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_RAWAE0) {
		size = 4 + ISP_RAWAE_LITE_RO_DBG2 - ISP_RAWAE_LITE_CTRL;
		reg_buf->isp_size[ISP2X_ID_RAWAE0] = size;
		reg_buf->isp_offset[ISP2X_ID_RAWAE0] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_RAWAE_LITE_CTRL, size);
		offset += size;

		size = ISP2X_RAWAELITE_MEAN_NUM * sizeof(tmp_statsbuf->params.rawae0.data[0]);
		reg_buf->isp_size[ISP2X_ID_RAWAE0] += size;
		reg_buf->isp_stats_size[ISP2X_ID_RAWAE0] = size;
		if (tmp_statsbuf->frame_id == reg_buf->frame_id)
			memcpy(&reg_buf->reg[offset], &tmp_statsbuf->params.rawae0.data[0], size);
		else
			memset(&reg_buf->reg[offset], 0, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_RAWAE1) {
		size = 4 + RAWAE_BIG_RO_DBG3 - RAWAE_BIG_CTRL;
		reg_buf->isp_size[ISP2X_ID_RAWAE1] = size;
		reg_buf->isp_offset[ISP2X_ID_RAWAE1] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + RAWAE_BIG1_BASE, size);
		offset += size;

		size = ISP2X_RAWAEBIG_MEAN_NUM * sizeof(tmp_statsbuf->params.rawae1.data[0]);
		reg_buf->isp_size[ISP2X_ID_RAWAE1] += size;
		reg_buf->isp_stats_size[ISP2X_ID_RAWAE1] = size;
		if (tmp_statsbuf->frame_id == reg_buf->frame_id)
			memcpy(&reg_buf->reg[offset], &tmp_statsbuf->params.rawae1.data[0], size);
		else
			memset(&reg_buf->reg[offset], 0, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_RAWAE2) {
		size = 4 + RAWAE_BIG_RO_DBG3 - RAWAE_BIG_CTRL;
		reg_buf->isp_size[ISP2X_ID_RAWAE2] = size;
		reg_buf->isp_offset[ISP2X_ID_RAWAE2] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + RAWAE_BIG2_BASE, size);
		offset += size;

		size = ISP2X_RAWAEBIG_MEAN_NUM * sizeof(tmp_statsbuf->params.rawae2.data[0]);
		reg_buf->isp_size[ISP2X_ID_RAWAE2] += size;
		reg_buf->isp_stats_size[ISP2X_ID_RAWAE2] = size;
		if (tmp_statsbuf->frame_id == reg_buf->frame_id)
			memcpy(&reg_buf->reg[offset], &tmp_statsbuf->params.rawae2.data[0], size);
		else
			memset(&reg_buf->reg[offset], 0, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_RAWAE3) {
		size = 4 + RAWAE_BIG_RO_DBG3 - RAWAE_BIG_CTRL;
		reg_buf->isp_size[ISP2X_ID_RAWAE3] = size;
		reg_buf->isp_offset[ISP2X_ID_RAWAE3] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + RAWAE_BIG3_BASE, size);
		offset += size;

		size = ISP2X_RAWAEBIG_MEAN_NUM * sizeof(tmp_statsbuf->params.rawae3.data[0]);
		reg_buf->isp_size[ISP2X_ID_RAWAE3] += size;
		reg_buf->isp_stats_size[ISP2X_ID_RAWAE3] = size;
		if (tmp_statsbuf->frame_id == reg_buf->frame_id)
			memcpy(&reg_buf->reg[offset], &tmp_statsbuf->params.rawae3.data[0], size);
		else
			memset(&reg_buf->reg[offset], 0, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_RAWAWB) {
		size = 4 + ISP_RAWAWB_RAM_CTRL - ISP_RAWAWB_CTRL;
		reg_buf->isp_size[ISP2X_ID_RAWAWB] = size;
		reg_buf->isp_offset[ISP2X_ID_RAWAWB] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_RAWAWB_CTRL, size);
		offset += size;

		size = ISP2X_RAWAWB_RAMDATA_NUM * sizeof(tmp_statsbuf->params.rawawb.ramdata[0]);
		reg_buf->isp_size[ISP2X_ID_RAWAWB] += size;
		reg_buf->isp_stats_size[ISP2X_ID_RAWAWB] = size;
		if (tmp_statsbuf->frame_id == reg_buf->frame_id)
			memcpy(&reg_buf->reg[offset], &tmp_statsbuf->params.rawawb.ramdata[0], size);
		else
			memset(&reg_buf->reg[offset], 0, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_RAWHIST0) {
		size = 4 + ISP_RAWHIST_LITE_WEIGHT - ISP_RAWHIST_LITE_CTRL;
		reg_buf->isp_size[ISP2X_ID_RAWHIST0] = size;
		reg_buf->isp_offset[ISP2X_ID_RAWHIST0] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_RAWHIST_LITE_CTRL, size);
		offset += size;

		size = ISP2X_HIST_BIN_N_MAX * sizeof(tmp_statsbuf->params.rawhist0.hist_bin[0]);
		reg_buf->isp_size[ISP2X_ID_RAWHIST0] += size;
		reg_buf->isp_stats_size[ISP2X_ID_RAWHIST0] = size;
		if (tmp_statsbuf->frame_id == reg_buf->frame_id)
			memcpy(&reg_buf->reg[offset], &tmp_statsbuf->params.rawhist0.hist_bin[0], size);
		else
			memset(&reg_buf->reg[offset], 0, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_RAWHIST1) {
		size = 4 + ISP_RAWHIST_BIG_WEIGHT_BASE - ISP_RAWHIST_BIG_CTRL;
		reg_buf->isp_size[ISP2X_ID_RAWHIST1] = size;
		reg_buf->isp_offset[ISP2X_ID_RAWHIST1] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_RAWHIST_BIG1_BASE, size);
		offset += size;

		size = ISP2X_HIST_BIN_N_MAX * sizeof(tmp_statsbuf->params.rawhist1.hist_bin[0]);
		reg_buf->isp_size[ISP2X_ID_RAWHIST1] += size;
		reg_buf->isp_stats_size[ISP2X_ID_RAWHIST1] = size;
		if (tmp_statsbuf->frame_id == reg_buf->frame_id)
			memcpy(&reg_buf->reg[offset], &tmp_statsbuf->params.rawhist1.hist_bin[0], size);
		else
			memset(&reg_buf->reg[offset], 0, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_RAWHIST2) {
		size = 4 + ISP_RAWHIST_BIG_WEIGHT_BASE - ISP_RAWHIST_BIG_CTRL;
		reg_buf->isp_size[ISP2X_ID_RAWHIST2] = size;
		reg_buf->isp_offset[ISP2X_ID_RAWHIST2] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_RAWHIST_BIG2_BASE, size);
		offset += size;

		size = ISP2X_HIST_BIN_N_MAX * sizeof(tmp_statsbuf->params.rawhist2.hist_bin[0]);
		reg_buf->isp_size[ISP2X_ID_RAWHIST2] += size;
		reg_buf->isp_stats_size[ISP2X_ID_RAWHIST2] = size;
		if (tmp_statsbuf->frame_id == reg_buf->frame_id)
			memcpy(&reg_buf->reg[offset], &tmp_statsbuf->params.rawhist2.hist_bin[0], size);
		else
			memset(&reg_buf->reg[offset], 0, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_RAWHIST3) {
		size = 4 + ISP_RAWHIST_BIG_WEIGHT_BASE - ISP_RAWHIST_BIG_CTRL;
		reg_buf->isp_size[ISP2X_ID_RAWHIST3] = size;
		reg_buf->isp_offset[ISP2X_ID_RAWHIST3] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_RAWHIST_BIG3_BASE, size);
		offset += size;

		size = ISP2X_HIST_BIN_N_MAX * sizeof(tmp_statsbuf->params.rawhist3.hist_bin[0]);
		reg_buf->isp_size[ISP2X_ID_RAWHIST3] += size;
		reg_buf->isp_stats_size[ISP2X_ID_RAWHIST3] = size;
		if (tmp_statsbuf->frame_id == reg_buf->frame_id)
			memcpy(&reg_buf->reg[offset], &tmp_statsbuf->params.rawhist3.hist_bin[0], size);
		else
			memset(&reg_buf->reg[offset], 0, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_HDRMGE) {
		size = 4 + ISP_HDRMGE_OVER_Y16 - ISP_HDRMGE_CTRL;
		reg_buf->isp_size[ISP2X_ID_HDRMGE] = size;
		reg_buf->isp_offset[ISP2X_ID_HDRMGE] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_HDRMGE_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_RAWNR) {
		size = 4 + ISP_RAWNR_RGBAIN_FLIP - ISP_RAWNR_CTRL;
		reg_buf->isp_size[ISP2X_ID_RAWNR] = size;
		reg_buf->isp_offset[ISP2X_ID_RAWNR] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_RAWNR_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_HDRTMO) {
		size = 4 + ISP_HDRTMO_HIST_RO31 - ISP_HDRTMO_CTRL;
		reg_buf->isp_size[ISP2X_ID_HDRTMO] = size;
		reg_buf->isp_offset[ISP2X_ID_HDRTMO] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_HDRTMO_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_GIC) {
		size = 4 + ISP_GIC_NOISE_CTRL1 - ISP_GIC_CONTROL;
		reg_buf->isp_size[ISP2X_ID_GIC] = size;
		reg_buf->isp_offset[ISP2X_ID_GIC] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_GIC_CONTROL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_DHAZ) {
		size = 4 + ISP_DHAZ_HIST_REG95 - ISP_DHAZ_CTRL;
		reg_buf->isp_size[ISP2X_ID_DHAZ] = size;
		reg_buf->isp_offset[ISP2X_ID_DHAZ] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_DHAZ_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_3DLUT) {
		size = 4 + ISP_3DLUT_UPDATE - ISP_3DLUT_CTRL;
		reg_buf->isp_size[ISP2X_ID_3DLUT] = size;
		reg_buf->isp_offset[ISP2X_ID_3DLUT] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_3DLUT_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_LDCH) {
		size = 4 + ISP_LDCH_STS - ISP_LDCH_STS;
		reg_buf->isp_size[ISP2X_ID_LDCH] = size;
		reg_buf->isp_offset[ISP2X_ID_LDCH] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_LDCH_STS, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_GAIN) {
		size = 4 + ISP_GAIN_LUT8 - ISP_GAIN_CTRL;
		reg_buf->isp_size[ISP2X_ID_GAIN] = size;
		reg_buf->isp_offset[ISP2X_ID_GAIN] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_GAIN_CTRL, size);
		offset += size;
	}

	if (rkisp_debug_reg & ISP2X_MODULE_DEBAYER) {
		size = 4 + ISP_DEBAYER_C_FILTER - ISP_DEBAYER_CONTROL;
		reg_buf->isp_size[ISP2X_ID_DEBAYER] = size;
		reg_buf->isp_offset[ISP2X_ID_DEBAYER] = offset;
		memcpy_fromio(&reg_buf->reg[offset], hw->base_addr + ISP_DEBAYER_CONTROL, size);
		offset += size;
	}

	reg_buf->reg_size = offset;
}

static void rkisp_bridge_try_sendtohal(struct rkisp_device *dev)
{
	struct rkisp_hw_dev *hw = dev->hw_dev;
	struct rkisp_bridge_device *br_dev = &dev->br_dev;
	struct rkisp_ispp_buf *cur_fbcgain = dev->cur_fbcgain;
	struct rkisp_buffer *cur_spbuf = dev->cur_spbuf;
	struct isp2x_ispgain_buf *buf;
	struct rkisp_bridge_buf *bdgbuf;
	struct vb2_buffer *vb2_buf;
	u32 *vaddr, size;
	u64 pic_ts, gain_ts, sp_ts;

	if (cur_fbcgain && cur_spbuf) {
		if ((cur_fbcgain->frame_id == cur_spbuf->vb.sequence) &&
		    (cur_fbcgain->index == cur_spbuf->dev_id)) {
			vb2_buf = &cur_spbuf->vb.vb2_buf;
			cur_fbcgain->buf_idx = vb2_buf->index;
			buf = vb2_plane_vaddr(vb2_buf, 1);
			buf->gain_dmaidx = cur_fbcgain->gain_dmaidx;
			buf->mfbc_dmaidx = cur_fbcgain->mfbc_dmaidx;
			buf->gain_size = cur_fbcgain->gain_size;
			buf->mfbc_size = cur_fbcgain->mfbc_size;
			buf->frame_id = cur_fbcgain->frame_id;
			bdgbuf = to_bridge_buf(cur_fbcgain);
			rkisp_finish_buffer(dev, &bdgbuf->dummy[GROUP_BUF_GAIN]);
			vb2_buffer_done(&cur_spbuf->vb.vb2_buf, VB2_BUF_STATE_DONE);
			list_add_tail(&cur_fbcgain->list, &hw->rpt_list);
			dev->cur_fbcgain = NULL;
			dev->cur_spbuf = NULL;
			v4l2_dbg(3, rkisp_debug, &br_dev->sd,
				 "%s send mfbcgain buf to hal, frame_id %d\n",
				 __func__, cur_fbcgain->frame_id);

			bdgbuf = to_bridge_buf(cur_fbcgain);
			vaddr = bdgbuf->dummy[GROUP_BUF_PIC].vaddr;
			size = bdgbuf->dummy[GROUP_BUF_PIC].size;
			pic_ts = *(u64 *)(vaddr + size / 4 - 2);

			vaddr = bdgbuf->dummy[GROUP_BUF_GAIN].vaddr;
			size = bdgbuf->dummy[GROUP_BUF_GAIN].size;
			gain_ts = *(u64 *)(vaddr + size / 4 - 2);

			size = vb2_plane_size(&cur_spbuf->vb.vb2_buf, 0);
			vaddr = (u32 *)vb2_plane_vaddr(&cur_spbuf->vb.vb2_buf, 0);
			sp_ts = *(u64 *)(vaddr + size / 4 - 2);
			if (abs(pic_ts - gain_ts) > 5000000LL || abs(pic_ts - sp_ts) > 5000000LL ||
			    abs(gain_ts - sp_ts) > 5000000LL) {
				v4l2_info(&br_dev->sd,
					"%s: frame %d, timestamp is not match (pic_ts %lld, gain_ts %lld, sp_ts %lld)\n",
					__func__, cur_fbcgain->frame_id, pic_ts, gain_ts, sp_ts);
			}
		} else {
			v4l2_info(&br_dev->sd,
				  "%s frame_id(%d, %d) or dev_id(%d, %d) is not match\n",
				  __func__,
				  cur_fbcgain->frame_id, cur_spbuf->vb.sequence,
				  cur_fbcgain->index, cur_spbuf->dev_id);
		}
	}
}

static void rkisp_bridge_save_fbcgain(struct rkisp_device *dev, struct rkisp_ispp_buf *fbcgain)
{
	struct rkisp_hw_dev *hw = dev->hw_dev;
	struct rkisp_bridge_device *br_dev = &dev->br_dev;
	unsigned long lock_flags = 0;

	spin_lock_irqsave(&hw->buf_lock, lock_flags);
	if (dev->cur_fbcgain) {
		v4l2_dbg(1, rkisp_debug, &br_dev->sd,
			 "%s old mfbcgain buf is exit, frame_id %d\n",
			 __func__, dev->cur_fbcgain->frame_id);
		list_add_tail(&dev->cur_fbcgain->list, &hw->list);
		dev->cur_fbcgain = NULL;
		br_dev->dbg.frameloss++;
	}
	dev->cur_fbcgain = fbcgain;
	rkisp_bridge_try_sendtohal(dev);
	spin_unlock_irqrestore(&hw->buf_lock, lock_flags);
}

static void rkisp_bridge_work(struct work_struct *work)
{
	struct rkisp_bridge_work *br_wk =
		container_of(work, struct rkisp_bridge_work, work);
	struct rkisp_bridge_device *dev = br_wk->dev;

	struct rkisp_ispp_reg *reg_buf = (struct rkisp_ispp_reg *)br_wk->param;

	dump_dbg_reg(dev, reg_buf);

	kfree(br_wk);
}

static int config_gain(struct rkisp_bridge_device *dev)
{
	u32 w = dev->crop.width;
	u32 h = dev->crop.height;
	u32 val;

	val = ALIGN(w, 64) * ALIGN(h, 128) >> 4;
	rkisp_write(dev->ispdev, MI_GAIN_WR_SIZE, val, false);
	val = ALIGN((w + 3) >> 2, 16);
	rkisp_write(dev->ispdev, MI_GAIN_WR_LENGTH, val, false);
	rkisp_set_bits(dev->ispdev, MI_WR_CTRL2,
		       0, SW_GAIN_WR_AUTOUPD, true);
	return 0;
}

static void crop_on(struct rkisp_bridge_device *dev)
{
	struct rkisp_device *ispdev = dev->ispdev;
	u32 src_w = ispdev->isp_sdev.out_crop.width;
	u32 src_h = ispdev->isp_sdev.out_crop.height;
	u32 dest_w = dev->crop.width;
	u32 dest_h = dev->crop.height;
	u32 left = dev->crop.left;
	u32 top = dev->crop.top;
	u32 ctrl = CIF_DUAL_CROP_CFG_UPD;

	if (ispdev->isp_ver == ISP_V20 &&
	    ispdev->rd_mode == HDR_RDBK_FRAME1 &&
	    ispdev->isp_sdev.in_fmt.fmt_type == FMT_BAYER)
		src_h += RKMODULE_EXTEND_LINE;

	rkisp_write(ispdev, CIF_DUAL_CROP_M_H_OFFS, left, false);
	rkisp_write(ispdev, CIF_DUAL_CROP_M_V_OFFS, top, false);
	rkisp_write(ispdev, CIF_DUAL_CROP_M_H_SIZE, dest_w, false);
	rkisp_write(ispdev, CIF_DUAL_CROP_M_V_SIZE, dest_h, false);
	ctrl |= rkisp_read(ispdev, CIF_DUAL_CROP_CTRL, true);
	if (src_w == dest_w && src_h == dest_h)
		ctrl &= ~(CIF_DUAL_CROP_MP_MODE_YUV | CIF_DUAL_CROP_MP_MODE_RAW);
	else
		ctrl |= CIF_DUAL_CROP_MP_MODE_YUV;
	rkisp_write(ispdev, CIF_DUAL_CROP_CTRL, ctrl, false);
}

static void crop_off(struct rkisp_bridge_device *dev)
{
	struct rkisp_device *ispdev = dev->ispdev;
	u32 ctrl = CIF_DUAL_CROP_GEN_CFG_UPD;

	ctrl = rkisp_read(ispdev, CIF_DUAL_CROP_CTRL, true);
	ctrl &= ~(CIF_DUAL_CROP_MP_MODE_YUV | CIF_DUAL_CROP_MP_MODE_RAW);
	rkisp_write(ispdev, CIF_DUAL_CROP_CTRL, ctrl, false);
}

static int bridge_start(struct rkisp_bridge_device *dev)
{
	struct rkisp_device *ispdev = dev->ispdev;
	struct rkisp_stream *sp_stream;

	hdr_config_dmatx(ispdev);

	sp_stream = &ispdev->cap_dev.stream[RKISP_STREAM_SP];
	crop_on(dev);
	config_gain(dev);
	dev->ops->config(dev);
	rkisp_start_spstream(sp_stream);

	if (!dev->ispdev->hw_dev->is_mi_update) {
		rkisp_config_dmatx_valid_buf(dev->ispdev);
		force_cfg_update(dev->ispdev);
		rkisp_update_spstream_buf(sp_stream);
		hdr_update_dmatx_buf(dev->ispdev);
	}
	rkisp_stats_first_ddr_config(&dev->ispdev->stats_vdev);
	ispdev->skip_frame = 0;
	dev->en = true;

	ispdev->cap_dev.is_done_early = false;
	if (ispdev->send_fbcgain)
		ispdev->cap_dev.wait_line = 0;
	if (ispdev->cap_dev.wait_line) {
		if (ispdev->cap_dev.wait_line < dev->crop.height / 4)
			ispdev->cap_dev.wait_line = dev->crop.height / 4;
		ispdev->cap_dev.is_done_early = true;
	}
	return 0;
}

static int bridge_stop(struct rkisp_bridge_device *dev)
{
	struct rkisp_stream *sp_stream;
	int ret;
	u32 irq;

	sp_stream = &dev->ispdev->cap_dev.stream[RKISP_STREAM_SP];
	dev->stopping = true;
	dev->ops->disable(dev);
	rkisp_stop_spstream(sp_stream);
	hdr_stop_dmatx(dev->ispdev);
	if (!dev->ispdev->hw_dev->is_shutdown) {
		ret = wait_event_timeout(dev->done, !dev->en,
					 msecs_to_jiffies(1000));
		if (!ret)
			v4l2_warn(&dev->sd,
				  "%s timeout ret:%d\n", __func__, ret);
	}
	crop_off(dev);
	dev->stopping = false;
	dev->en = false;
	irq = dev->cfg->frame_end_id;
	irq = (irq == MI_MPFBC_FRAME) ? ISP_FRAME_MPFBC : ISP_FRAME_MP;
	dev->ispdev->irq_ends_mask &= ~irq;
	drain_workqueue(dev->wq);

	/* make sure ispp last frame done */
	if (dev->work_mode & ISP_ISPP_QUICK) {
		rkisp_clear_bits(dev->ispdev, MI_IMSC,
				 dev->cfg->frame_end_id, true);
		usleep_range(20000, 25000);
	}
	return 0;
}

static void bridge_update_mi(struct rkisp_bridge_device *br)
{
	struct rkisp_device *dev = br->ispdev;
	struct rkisp_hw_dev *hw = dev->hw_dev;
	struct rkisp_bridge_buf *buf;
	u32 val;

	if (hw->nxt_buf) {
		buf = to_bridge_buf(hw->nxt_buf);
		val = buf->dummy[GROUP_BUF_PIC].dma_addr;
		rkisp_write(dev, br->cfg->reg.y0_base, val, true);
		val += br->cfg->offset;
		rkisp_write(dev, br->cfg->reg.uv0_base, val, true);
		val = buf->dummy[GROUP_BUF_GAIN].dma_addr;
		rkisp_write(dev, br->cfg->reg.g0_base, val, true);
	}

	if (dev->cap_dev.is_done_early && !br->frame_early) {
		br->frame_early = true;
		hrtimer_start(&br->frame_qst, ns_to_ktime(1000000), HRTIMER_MODE_REL);
	}

	v4l2_dbg(2, rkisp_debug, &br->sd,
		 "update pic(shd:0x%x base:0x%x) gain(shd:0x%x base:0x%x)\n",
		 rkisp_read(dev, br->cfg->reg.y0_base_shd, true),
		 rkisp_read(dev, br->cfg->reg.y0_base, true),
		 rkisp_read(dev, br->cfg->reg.g0_base_shd, true),
		 rkisp_read(dev, br->cfg->reg.g0_base, true));
}

static int bridge_frame_end(struct rkisp_bridge_device *dev, u32 state)
{
	struct rkisp_device *ispdev = dev->ispdev;
	struct rkisp_hw_dev *hw = ispdev->hw_dev;
	struct v4l2_subdev *sd = v4l2_get_subdev_hostdata(&dev->sd);
	unsigned long lock_flags = 0;
	u64 ns = rkisp_time_get_ns(ispdev);
	struct rkisp_bridge_buf *buf;
	u32 val;

	if (dev->stopping) {
		if (dev->ops->is_stopped(dev)) {
			dev->en = false;
			dev->stopping = false;
			wake_up(&dev->done);
		}
	}

	if (!dev->en) {
		val = dev->cfg->frame_end_id;
		val = (val == MI_MPFBC_FRAME) ? ISP_FRAME_MPFBC : ISP_FRAME_MP;
		ispdev->irq_ends_mask &= ~val;
		return 0;
	}

	if (dev->work_mode & ISP_ISPP_QUICK ||
	    (state == FRAME_IRQ && ispdev->cap_dev.is_done_early))
		return 0;
	dev->frame_early = false;
	rkisp_dmarx_get_frame(dev->ispdev, &dev->dbg.id, NULL, NULL, true);
	dev->dbg.interval = ns - dev->dbg.timestamp;
	dev->dbg.timestamp = ns;
	if (hw->cur_buf && hw->nxt_buf) {
		if (ispdev->skip_frame > 0) {
			ispdev->skip_frame--;
			spin_lock_irqsave(&hw->buf_lock, lock_flags);
			list_add_tail(&hw->cur_buf->list, &hw->list);
			spin_unlock_irqrestore(&hw->buf_lock, lock_flags);
		} else {
			u64 sof_ns = 0;
			struct rkisp_ispp_reg *reg_buf = NULL;

			ns = 0;
			rkisp_dmarx_get_frame(ispdev, &hw->cur_buf->frame_id,
					      &sof_ns, &ns, true);
			if (!sof_ns)
				sof_ns = 0;
			if (!ns)
				ns = rkisp_time_get_ns(ispdev);
			hw->cur_buf->frame_timestamp = ns;
			hw->cur_buf->index = ispdev->dev_id;
			v4l2_subdev_call(sd, core, ioctl, RKISP_ISPP_CMD_REQUEST_REGBUF,
					 &reg_buf);
			if (reg_buf) {
				struct rkisp_bridge_work *br_wk;

				br_wk = kzalloc(sizeof(struct rkisp_bridge_work), GFP_ATOMIC);
				if (br_wk) {
					reg_buf->stat = ISP_ISPP_INUSE;
					reg_buf->dev_id = hw->cur_buf->index;
					reg_buf->frame_id = hw->cur_buf->frame_id;
					reg_buf->sof_timestamp = sof_ns;
					reg_buf->frame_timestamp = hw->cur_buf->frame_timestamp;
					reg_buf->exposure = ispdev->params_vdev.exposure;

					br_wk->dev = dev;
					br_wk->param = (void *)reg_buf;
					INIT_WORK((struct work_struct *)&br_wk->work, rkisp_bridge_work);
					if (!queue_work(dev->wq, (struct work_struct *)&br_wk->work)) {
						v4l2_err(&dev->sd, "queue work failed\n");
						kfree(br_wk);
					}
				}
			}

			if (ispdev->send_fbcgain) {
				u32 *vaddr, size;

				buf = to_bridge_buf(hw->cur_buf);
				vaddr = buf->dummy[GROUP_BUF_PIC].vaddr;
				size = buf->dummy[GROUP_BUF_PIC].size;
				*(u64 *)(vaddr + size / 4 - 2) = rkisp_time_get_ns(ispdev);

				vaddr = buf->dummy[GROUP_BUF_GAIN].vaddr;
				size = buf->dummy[GROUP_BUF_GAIN].size;
				*(u64 *)(vaddr + size / 4 - 2) = rkisp_time_get_ns(ispdev);
				hw->cur_buf->mfbc_dmaidx = hw->cur_buf->didx[GROUP_BUF_PIC];
				hw->cur_buf->gain_dmaidx = hw->cur_buf->didx[GROUP_BUF_GAIN];
				hw->cur_buf->is_move_judge = true;
				rkisp_bridge_save_fbcgain(ispdev, hw->cur_buf);
			} else {
				hw->cur_buf->is_move_judge = false;
				v4l2_subdev_call(sd, video, s_rx_buffer, hw->cur_buf, NULL);
			}
		}
		hw->cur_buf = NULL;
	} else {
		v4l2_dbg(1, rkisp_debug, &dev->sd, "no buf, lost frame:%d\n", dev->dbg.id);
		dev->dbg.frameloss++;
	}

	if (hw->nxt_buf) {
		hw->cur_buf = hw->nxt_buf;
		hw->nxt_buf = NULL;
	}

	return 0;
}

static enum hrtimer_restart rkisp_bridge_frame_done_early(struct hrtimer *timer)
{
	struct rkisp_bridge_device *br =
		container_of(timer, struct rkisp_bridge_device, frame_qst);
	struct rkisp_device *dev = br->ispdev;
	enum hrtimer_restart ret = HRTIMER_NORESTART;
	u32 ycnt, line = dev->cap_dev.wait_line;
	u32 seq, time, max_time = 1000000;
	u64 ns = ktime_get_ns();

	time = (u32)(ns - br->fs_ns);
	ycnt = rkisp_read(dev, ISP_MPFBC_ENC_POS, true) & 0x3ff;
	ycnt *= 8;
	rkisp_dmarx_get_frame(dev, &seq, NULL, NULL, true);
	if (!br->en || dev->isp_state == ISP_STOP) {
		goto end;
	} else if (ycnt < line) {
		if (!ycnt)
			ns = max_time;
		else
			ns = time * (line - ycnt) / ycnt;
		if (ns > max_time)
			ns = max_time;
		hrtimer_forward(timer, timer->base->get_time(), ns_to_ktime(ns));
		ret = HRTIMER_RESTART;
	} else {
		v4l2_dbg(3, rkisp_debug, &dev->v4l2_dev,
			 "%s seq:%d line:%d ycnt:%d time:%dus\n",
			 __func__, seq, line, ycnt, time / 1000);
		bridge_frame_end(br, FRAME_WORK);
	}
end:
	return ret;
}

static int config_mpfbc(struct rkisp_bridge_device *dev)
{
	struct rkisp_hw_dev *hw = dev->ispdev->hw_dev;
	u32 h = hw->max_in.h ? hw->max_in.h : dev->crop.height;
	u32 ctrl = 0;

	if (dev->work_mode & ISP_ISPP_QUICK) {
		rkisp_set_bits(dev->ispdev, CTRL_SWS_CFG,
			       0, SW_ISP2PP_PIPE_EN, true);
		ctrl = SW_MPFBC_MAINISP_MODE;
		if (dev->ispdev->hw_dev->nxt_buf)
			ctrl |= SW_MPFBC_PINGPONG_EN;
	}

	rkisp_write(dev->ispdev, ISP_MPFBC_VIR_WIDTH, 0, true);
	rkisp_write(dev->ispdev, ISP_MPFBC_VIR_HEIGHT, ALIGN(h, 16), true);

	ctrl |= (dev->work_mode & ISP_ISPP_422) | SW_MPFBC_EN;
	rkisp_write(dev->ispdev, ISP_MPFBC_BASE, ctrl, true);
	rkisp_set_bits(dev->ispdev, MI_WR_CTRL, MI_LUM_BURST_MASK,
		       MI_MIPI_LUM_BURST16, true);
	dev->ispdev->irq_ends_mask |= ISP_FRAME_MPFBC;
	return 0;
}

static void disable_mpfbc(struct rkisp_bridge_device *dev)
{
	if (dev->ispdev->hw_dev->is_single)
		rkisp_clear_bits(dev->ispdev, ISP_MPFBC_BASE,
				 SW_MPFBC_EN, true);
}

static bool is_stopped_mpfbc(struct rkisp_bridge_device *dev)
{
	bool en = true;

	if (dev->ispdev->hw_dev->is_single)
		en = is_mpfbc_stopped(dev->ispdev->base_addr);
	return en;
}

static struct rkisp_bridge_ops mpfbc_ops = {
	.config = config_mpfbc,
	.disable = disable_mpfbc,
	.is_stopped = is_stopped_mpfbc,
	.frame_end = bridge_frame_end,
	.update_mi = bridge_update_mi,
	.start = bridge_start,
	.stop = bridge_stop,
};

static struct rkisp_bridge_config mpfbc_cfg = {
	.frame_end_id = MI_MPFBC_FRAME,
	.reg = {
		.y0_base = ISP_MPFBC_HEAD_PTR,
		.uv0_base = ISP_MPFBC_PAYL_PTR,
		.y1_base = ISP_MPFBC_HEAD_PTR2,
		.uv1_base = ISP_MPFBC_PAYL_PTR2,
		.g0_base = MI_GAIN_WR_BASE,
		.g1_base = MI_GAIN_WR_BASE2,

		.y0_base_shd = ISP_MPFBC_HEAD_PTR,
		.uv0_base_shd = ISP_MPFBC_PAYL_PTR,
		.g0_base_shd = MI_GAIN_WR_BASE_SHD,
	},
};

static int config_mp(struct rkisp_bridge_device *dev)
{
	u32 w = dev->crop.width;
	u32 h = dev->crop.height;
	u32 val;

	if (dev->work_mode & ISP_ISPP_QUICK) {
		rkisp_set_bits(dev->ispdev, CTRL_SWS_CFG, 0,
			       SW_ISP2PP_PIPE_EN, true);
		if (dev->ispdev->hw_dev->nxt_buf)
			rkisp_set_bits(dev->ispdev, CIF_MI_CTRL, 0,
				       CIF_MI_MP_PINGPONG_ENABLE, true);
	}

	val = w * h;
	rkisp_write(dev->ispdev, CIF_MI_MP_Y_SIZE_INIT, val, false);
	val = (dev->work_mode & ISP_ISPP_422) ? val : val / 2;
	rkisp_write(dev->ispdev, CIF_MI_MP_CB_SIZE_INIT, val, false);
	rkisp_write(dev->ispdev, CIF_MI_MP_CR_SIZE_INIT, 0, false);
	rkisp_write(dev->ispdev, CIF_MI_MP_Y_OFFS_CNT_INIT, 0, false);
	rkisp_write(dev->ispdev, CIF_MI_MP_CB_OFFS_CNT_INIT, 0, false);
	rkisp_write(dev->ispdev, CIF_MI_MP_CR_OFFS_CNT_INIT, 0, false);

	rkisp_write(dev->ispdev, ISP_MPFBC_BASE,
		    dev->work_mode & ISP_ISPP_422, true);
	rkisp_set_bits(dev->ispdev, CIF_MI_CTRL, MI_CTRL_MP_FMT_MASK,
		       MI_CTRL_MP_WRITE_YUV_SPLA | CIF_MI_CTRL_MP_ENABLE |
		       CIF_MI_MP_AUTOUPDATE_ENABLE, true);
	dev->ispdev->irq_ends_mask |= ISP_FRAME_MP;
	return 0;
}

static void disable_mp(struct rkisp_bridge_device *dev)
{
	if (dev->ispdev->hw_dev->is_single)
		rkisp_clear_bits(dev->ispdev, CIF_MI_CTRL,
				 CIF_MI_CTRL_MP_ENABLE |
				 CIF_MI_CTRL_RAW_ENABLE, true);
}

static bool is_stopped_mp(struct rkisp_bridge_device *dev)
{
	bool en = true;

	if (dev->ispdev->hw_dev->is_single)
		en = mp_is_stream_stopped(dev->ispdev->base_addr);
	return en;
}

static struct rkisp_bridge_ops mp_ops = {
	.config = config_mp,
	.disable = disable_mp,
	.is_stopped = is_stopped_mp,
	.frame_end = bridge_frame_end,
	.update_mi = bridge_update_mi,
	.start = bridge_start,
	.stop = bridge_stop,
};

static struct rkisp_bridge_config mp_cfg = {
	.frame_end_id = MI_MP_FRAME,
	.reg = {
		.y0_base = MI_MP_WR_Y_BASE,
		.uv0_base = MI_MP_WR_CB_BASE,
		.y1_base = MI_MP_WR_Y_BASE2,
		.uv1_base = MI_MP_WR_CB_BASE2,
		.g0_base = MI_GAIN_WR_BASE,
		.g1_base = MI_GAIN_WR_BASE2,

		.y0_base_shd = MI_MP_WR_Y_BASE_SHD,
		.uv0_base_shd = MI_MP_WR_CB_BASE_SHD,
		.g0_base_shd = MI_GAIN_WR_BASE_SHD,
	},
};

int rkisp_bridge_get_fbcbuf_fd(struct rkisp_device *dev, struct isp2x_buf_idxfd *idxfd)
{
	struct rkisp_hw_dev *hw = dev->hw_dev;
	struct rkisp_bridge_device *br_dev = &dev->br_dev;
	struct rkisp_bridge_buf *buf;
	struct rkisp_dummy_buffer *dummy;
	unsigned long lock_flags = 0;
	int i, j, buf_idx;

	spin_lock_irqsave(&hw->buf_lock, lock_flags);
	if (!hw->is_buf_init) {
		spin_unlock_irqrestore(&hw->buf_lock, lock_flags);
		return -EAGAIN;
	}
	spin_unlock_irqrestore(&hw->buf_lock, lock_flags);

	buf_idx = 0;
	for (i = 0; i < br_dev->buf_num; i++) {
		buf = &hw->bufs[i];
		for (j = 0; j < GROUP_BUF_MAX; j++) {
			dummy = &buf->dummy[j];
			buf->dbufs.dfd[j] = dma_buf_fd(dummy->dbuf, O_CLOEXEC);
			get_dma_buf(buf->dbufs.dbuf[j]);
			idxfd->index[buf_idx] = buf->dbufs.didx[j];
			idxfd->dmafd[buf_idx] = buf->dbufs.dfd[j];
			buf_idx++;
		}
	}

	idxfd->buf_num = buf_idx;

	return 0;
}

void rkisp_bridge_sendtopp_buffer(struct rkisp_device *dev, u32 dev_id, u32 buf_idx)
{
	struct rkisp_hw_dev *hw = dev->hw_dev;
	struct rkisp_bridge_device *br_dev = &dev->br_dev;
	struct v4l2_subdev *sd = v4l2_get_subdev_hostdata(&br_dev->sd);
	struct rkisp_ispp_buf *cur_buf, *cur_buf_tmp, *find_buf;
	struct rkisp_bridge_buf *buf;
	unsigned long lock_flags = 0;
	bool find_flg = false;
	u32 *vaddr, size;
	u64 pic_ts, gain_ts;

	spin_lock_irqsave(&hw->buf_lock, lock_flags);
	list_for_each_entry(cur_buf, &hw->rpt_list, list) {
		if (cur_buf->index == dev_id && cur_buf->buf_idx == buf_idx) {
			find_flg = true;
			break;
		}
	}

	if (find_flg) {
		list_del(&cur_buf->list);
		find_buf = cur_buf;
		list_for_each_entry_safe(cur_buf, cur_buf_tmp, &hw->rpt_list, list) {
			if ((cur_buf->frame_id < find_buf->frame_id) &&
				(cur_buf->index == find_buf->index)) {
				list_del_init(&cur_buf->list);
				v4l2_dbg(3, rkisp_debug, &br_dev->sd,
					 "%s send buffer to pp, frame_id %d\n",
					 __func__, cur_buf->frame_id);

				buf = to_bridge_buf(cur_buf);
				rkisp_prepare_buffer(dev, &buf->dummy[GROUP_BUF_GAIN]);
				vaddr = buf->dummy[GROUP_BUF_PIC].vaddr;
				size = buf->dummy[GROUP_BUF_PIC].size;
				pic_ts = *(u64 *)(vaddr + size / 4 - 2);

				vaddr = buf->dummy[GROUP_BUF_GAIN].vaddr;
				size = buf->dummy[GROUP_BUF_GAIN].size;
				gain_ts = *(u64 *)(vaddr + size / 4 - 2);
				if (abs(pic_ts - gain_ts) > 5000000LL) {
					v4l2_info(&br_dev->sd,
						"%s: frame %d, timestamp is not match (pic_ts %lld, gain_ts %lld)",
						__func__, cur_buf->frame_id, pic_ts, gain_ts);
				}
				cur_buf->is_move_judge = true;
				v4l2_subdev_call(sd, video, s_rx_buffer, cur_buf, NULL);
			}
		}

		v4l2_dbg(3, rkisp_debug, &br_dev->sd,
			 "%s send buffer to pp, frame_id %d\n",
			 __func__, find_buf->frame_id);

		buf = to_bridge_buf(find_buf);
		rkisp_prepare_buffer(dev, &buf->dummy[GROUP_BUF_GAIN]);
		vaddr = buf->dummy[GROUP_BUF_PIC].vaddr;
		size = buf->dummy[GROUP_BUF_PIC].size;
		pic_ts = *(u64 *)(vaddr + size / 4 - 2);

		vaddr = buf->dummy[GROUP_BUF_GAIN].vaddr;
		size = buf->dummy[GROUP_BUF_GAIN].size;
		gain_ts = *(u64 *)(vaddr + size / 4 - 2);
		if (abs(pic_ts - gain_ts) > 5000000LL) {
			v4l2_info(&br_dev->sd,
				"%s: frame %d, timestamp is not match (pic_ts %lld, gain_ts %lld)",
				__func__, find_buf->frame_id, pic_ts, gain_ts);
		}
		find_buf->is_move_judge = true;
		v4l2_subdev_call(sd, video, s_rx_buffer, find_buf, NULL);
	}
	spin_unlock_irqrestore(&hw->buf_lock, lock_flags);
}

void rkisp_bridge_save_spbuf(struct rkisp_device *dev, struct rkisp_buffer *sp_buf)
{
	struct rkisp_hw_dev *hw = dev->hw_dev;
	struct rkisp_bridge_device *br_dev = &dev->br_dev;
	unsigned long lock_flags = 0;

	spin_lock_irqsave(&hw->buf_lock, lock_flags);
	if (dev->cur_spbuf) {
		v4l2_dbg(1, rkisp_debug, &br_dev->sd,
			 "%s old sp buf is exit, frame_id %d\n",
			 __func__, dev->cur_spbuf->vb.sequence);
		rkisp_spbuf_queue(&dev->cap_dev.stream[RKISP_STREAM_SP], dev->cur_spbuf);
		dev->cur_spbuf = NULL;
		dev->cap_dev.stream[RKISP_STREAM_SP].dbg.frameloss++;
	}
	dev->cur_spbuf = sp_buf;
	rkisp_bridge_try_sendtohal(dev);
	spin_unlock_irqrestore(&hw->buf_lock, lock_flags);
}

void rkisp_bridge_stop_spstream(struct rkisp_device *dev)
{
	struct rkisp_hw_dev *hw = dev->hw_dev;
	unsigned long lock_flags = 0;

	spin_lock_irqsave(&hw->buf_lock, lock_flags);
	if (dev->cur_spbuf) {
		rkisp_spbuf_queue(&dev->cap_dev.stream[RKISP_STREAM_SP], dev->cur_spbuf);
		dev->cur_spbuf = NULL;
	}
	spin_unlock_irqrestore(&hw->buf_lock, lock_flags);
}

void rkisp_bridge_init_ops_v20(struct rkisp_bridge_device *dev)
{
	if (dev->work_mode & ISP_ISPP_FBC) {
		dev->ops = &mpfbc_ops;
		dev->cfg = &mpfbc_cfg;
	} else {
		dev->ops = &mp_ops;
		dev->cfg = &mp_cfg;
	}
	dev->frame_qst.function = rkisp_bridge_frame_done_early;
}
