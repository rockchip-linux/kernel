/*
**************************************************************************
 * Rockchip driver for CIF ISP 1.1
 * (Based on Intel driver for sofiaxxx)
 *
 * Copyright (C) 2015 Intel Mobile Communications GmbH
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
**************************************************************************
 */

#ifndef _CIF_ISP11_ISP_H
#define _CIF_ISP11_ISP_H

#include <media/v4l2-common.h>
#include <media/videobuf-core.h>
#include <media/rk-isp11-ioctl.h>
#include <media/v4l2-controls_rockchip.h>

/****************************************************************************
*                                                     ISP device struct
****************************************************************************/
enum cif_isp11_pix_fmt;

enum cif_isp11_pix_fmt_quantization {
	CIF_ISP11_QUANTIZATION_DEFAULT = 0,
	CIF_ISP11_QUANTIZATION_FULL_RANGE = 1,
	CIF_ISP11_QUANTIZATION_LIM_RANGE = 2
};

struct cif_isp11_isp_dev {
	bool dpcc_en;
	bool bls_en;
	bool sdg_en;
	bool lsc_en;
	bool awb_meas_en;
	bool awb_gain_en;
	bool flt_en;
	bool bdm_en;
	bool ctk_en;
	bool goc_en;
	bool hst_en;
	bool aec_en;
	bool cproc_en;
	bool afc_en;
	bool ie_en;
	bool dpf_en;
	bool wdr_en;
	bool y12_en;

	/* Purpose of mutex is to protect and serialize use
		of isp data structure and CIF API calls. */
	struct mutex mutex;
	/* Current ISP parameters */
	spinlock_t config_lock;
	struct cifisp_dpcc_config dpcc_config;
	struct cifisp_bls_config bls_config;
	struct cifisp_sdg_config sdg_config;
	struct cifisp_lsc_config lsc_config;
	struct cifisp_awb_meas_config awb_meas_config;
	struct cifisp_awb_gain_config awb_gain_config;
	struct cifisp_flt_config flt_config;
	struct cifisp_bdm_config bdm_config;
	struct cifisp_ctk_config ctk_config;
	struct cifisp_goc_config goc_config;
	struct cifisp_hst_config hst_config;
	struct cifisp_aec_config aec_config;
	struct cifisp_cproc_config cproc_config;
	struct cifisp_afc_config afc_config;
	struct cifisp_ie_config ie_config;
	struct cifisp_dpf_config dpf_config;
	struct cifisp_dpf_strength_config dpf_strength_config;
	struct cifisp_wdr_config wdr_config;

	bool isp_param_dpcc_update_needed;
	bool isp_param_bls_update_needed;
	bool isp_param_sdg_update_needed;
	bool isp_param_lsc_update_needed;
	bool isp_param_awb_meas_update_needed;
	bool isp_param_awb_gain_update_needed;
	bool isp_param_flt_update_needed;
	bool isp_param_bdm_update_needed;
	bool isp_param_ctk_update_needed;
	bool isp_param_goc_update_needed;
	bool isp_param_hst_update_needed;
	bool isp_param_aec_update_needed;
	bool isp_param_cproc_update_needed;
	bool isp_param_afc_update_needed;
	bool isp_param_ie_update_needed;
	bool isp_param_range_update_needed;
	bool isp_param_wdr_update_needed;
	bool isp_param_dpf_update_needed;
	bool isp_param_dpf_strength_update_needed;
	bool cif_ism_cropping;

	enum cif_isp11_pix_fmt_quantization quantization;

	/* input resolution needed for LSC param check */
	unsigned int input_width;
	unsigned int input_height;
	unsigned int active_lsc_width;
	unsigned int active_lsc_height;

	/* ISP statistics related */
	spinlock_t irq_lock;
	spinlock_t req_lock;
	struct videobuf_queue vbq_stat;
	struct list_head stat;
	void __iomem *base_addr;    /* registers base address */

	bool streamon;
	unsigned int v_blanking_us;
	bool ignore_measurement_check;

	unsigned int frame_id;
	unsigned int frame_id_setexp;
	unsigned int active_meas;
	struct timeval frame_start_tv;

	unsigned int *dev_id;
};

int register_cifisp_device(
	struct cif_isp11_isp_dev *isp_dev,
	struct video_device *vdev_cifisp,
	struct v4l2_device *v4l2_dev,
	void __iomem *cif_reg_baseaddress);
void unregister_cifisp_device(struct video_device *vdev_cifisp);
void cifisp_configure_isp(
	struct cif_isp11_isp_dev *isp_dev,
	enum cif_isp11_pix_fmt in_pix_fmt,
	enum cif_isp11_pix_fmt_quantization quantization);
void cifisp_disable_isp(struct cif_isp11_isp_dev *isp_dev);
int cifisp_isp_isr(struct cif_isp11_isp_dev *isp_dev, u32 isp_mis);
void cifisp_v_start(struct cif_isp11_isp_dev *isp_dev,
	const struct timeval *timestamp);


#endif
