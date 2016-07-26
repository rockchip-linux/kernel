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
#include <linux/v4l2-controls.h>

#ifndef _RK_ISP11_IOCTL_H
#define _RK_ISP11_IOCTL_H

#define CIFISP_CTK_COEFF_MAX   0x100
#define CIFISP_CTK_OFFSET_MAX 0x800

#define CIFISP_AE_MEAN_MAX 25
#define CIFISP_HIST_BIN_N_MAX 16
#define CIFISP_AFM_MAX_WINDOWS 3
#define CIFISP_DEGAMMA_CURVE_SIZE 17

#define CIFISP_BDM_MAX_TH 0xFF

/* maximum value for horizontal start address*/
#define CIFISP_BLS_START_H_MAX             (0x00000FFF)
/* maximum value for horizontal stop address */
#define CIFISP_BLS_STOP_H_MAX              (0x00000FFF)
/* maximum value for vertical start address*/
#define CIFISP_BLS_START_V_MAX             (0x00000FFF)
/* maximum value for vertical stop address*/
#define CIFISP_BLS_STOP_V_MAX              (0x00000FFF)
/* maximum is 2^18 = 262144*/
#define CIFISP_BLS_SAMPLES_MAX             (0x00000012)
/* maximum value for fixed black level*/
#define CIFISP_BLS_FIX_SUB_MAX             (0x00000FFF)
/* minimum value for fixed black level*/
#define CIFISP_BLS_FIX_SUB_MIN             (0xFFFFF000)
/* 13 bit range (signed)*/
#define CIFISP_BLS_FIX_MASK                (0x00001FFF)
/* AWB */
#define CIFISP_AWB_MAX_GRID                1
#define CIFISP_AWB_MAX_FRAMES              7

/* Gamma out*/
/* Maximum number of color samples supported*/
#define CIFISP_GAMMA_OUT_MAX_SAMPLES       34

/* WDR */
#define CIFISP_WDR_SIZE						48

/* LSC */
#define CIFISP_LSC_GRAD_TBL_SIZE           8
#define CIFISP_LSC_SIZE_TBL_SIZE           8
/* The following matches the tuning process,
 * not the max capabilites of the chip. */
#define	CIFISP_LSC_DATA_TBL_SIZE           289
/* HIST */
#define CIFISP_HISTOGRAM_WEIGHT_GRIDS_SIZE       25

/* DPCC */
#define CIFISP_DPCC_METHODS_MAX                    (3)

/* DPF */
#define CIFISP_DPF_MAX_NLF_COEFFS      17
#define CIFISP_DPF_MAX_SPATIAL_COEFFS  6

#define CIFISP_STAT_AWB		(1 << 0)
#define CIFISP_STAT_AUTOEXP	(1 << 1)
#define CIFISP_STAT_AFM_FIN		(1 << 2)
#define CIFISP_STAT_HIST		(1 << 3)

enum cifisp_histogram_mode {
	CIFISP_HISTOGRAM_MODE_DISABLE         = 0,
	CIFISP_HISTOGRAM_MODE_RGB_COMBINED    = 1,
	CIFISP_HISTOGRAM_MODE_R_HISTOGRAM     = 2,
	CIFISP_HISTOGRAM_MODE_G_HISTOGRAM     = 3,
	CIFISP_HISTOGRAM_MODE_B_HISTOGRAM     = 4,
	CIFISP_HISTOGRAM_MODE_Y_HISTOGRAM     = 5
};

enum cifisp_exp_ctrl_autostop {
	CIFISP_EXP_CTRL_AUTOSTOP_0 = 0,
	CIFISP_EXP_CTRL_AUTOSTOP_1 = 1
};

enum cifisp_exp_meas_mode {
	CIFISP_EXP_MEASURING_MODE_0 = 0,    /**< Y = 16 + 0.25R + 0.5G + 0.1094B */
	CIFISP_EXP_MEASURING_MODE_1 = 1,    /**< Y = (R + G + B) x (85/256) */
};

struct cifisp_window {
	unsigned short h_offs;
	unsigned short v_offs;
	unsigned short h_size;
	unsigned short v_size;
};

enum cifisp_awb_mode_type {
	CIFISP_AWB_MODE_MANUAL  = 0,
	CIFISP_AWB_MODE_RGB     = 1,
	CIFISP_AWB_MODE_YCBCR   = 2
};

enum cifisp_bls_win_enable {
	ISP_BLS_CTRL_WINDOW_ENABLE_0 = 0,
	ISP_BLS_CTRL_WINDOW_ENABLE_1 = 1,
	ISP_BLS_CTRL_WINDOW_ENABLE_2 = 2,
	ISP_BLS_CTRL_WINDOW_ENABLE_3 = 3
};

enum cifisp_flt_mode {
	CIFISP_FLT_STATIC_MODE,
	CIFISP_FLT_DYNAMIC_MODE
};

struct cifisp_awb_meas {
	unsigned int cnt;
	unsigned char mean_y;
	unsigned char mean_cb;
	unsigned char mean_cr;
	unsigned short mean_r;
	unsigned short mean_b;
	unsigned short mean_g;
};

struct cifisp_awb_stat {
	struct cifisp_awb_meas awb_mean[CIFISP_AWB_MAX_GRID];
};

struct cifisp_hist_stat {
	unsigned short hist_bins[CIFISP_HIST_BIN_N_MAX];
};

/*! BLS mean measured values*/
struct cifisp_bls_meas_val {
	/*! Mean measured value for Bayer pattern position A.*/
	unsigned short meas_a;
	/*! Mean measured value for Bayer pattern position B.*/
	unsigned short meas_b;
	/*! Mean measured value for Bayer pattern position C.*/
	unsigned short meas_c;
	/*! Mean measured value for Bayer pattern position D.*/
	unsigned short meas_d;
};

/*! BLS fixed subtraction values. The values will be subtracted from the sensor
 *  values. Therefore a negative value means addition instead of subtraction!*/
struct cifisp_bls_fixed_val {
	/*! Fixed (signed!) subtraction value for Bayer pattern position A.*/
	signed short fixed_a;
	/*! Fixed (signed!) subtraction value for Bayer pattern position B.*/
	signed short fixed_b;
	/*! Fixed (signed!) subtraction value for Bayer pattern position C.*/
	signed short fixed_c;
	/*! Fixed (signed!) subtraction value for Bayer pattern position D.*/
	signed short fixed_d;
};

/* Configuration used by black level subtraction */
struct cifisp_bls_config {
	/*! Automatic mode activated means that the measured values
	 * are subtracted.Otherwise the fixed subtraction
	 * values will be subtracted.*/
	bool enable_auto;
	unsigned char en_windows;
	struct cifisp_window bls_window1;      /*!< Measurement window 1.*/
	struct cifisp_window bls_window2;      /*!< Measurement window 2*/
	/*! Set amount of measured pixels for each Bayer position
	 * (A, B,C and D) to 2^bls_samples.*/
	unsigned char bls_samples;
	struct cifisp_bls_fixed_val fixed_val; /*!< Fixed subtraction values.*/
};


struct cifisp_ae_stat {
	unsigned char exp_mean[CIFISP_AE_MEAN_MAX];
	struct cifisp_bls_meas_val bls_val; /*available wit exposure results*/
};

struct cifisp_af_meas_val {
	unsigned int sum;
	unsigned int lum;
};

struct cifisp_af_stat {
	struct cifisp_af_meas_val window[CIFISP_AFM_MAX_WINDOWS];
};

struct cifisp_stat {
	struct cifisp_awb_stat awb;
	struct cifisp_ae_stat ae;
	struct cifisp_af_stat af;
	struct cifisp_hist_stat hist;
};

struct cifisp_stat_buffer {
	unsigned int meas_type;
	struct cifisp_stat params;
};

struct cifisp_dpcc_methods_config {
	unsigned int method;
	unsigned int  line_thresh;
	unsigned int  line_mad_fac;
	unsigned int  pg_fac;
	unsigned int  rnd_thresh;
	unsigned int  rg_fac;
};

struct cifisp_dpcc_config {
	unsigned int  mode;
	unsigned int  output_mode;
	unsigned int  set_use;
	struct cifisp_dpcc_methods_config methods[CIFISP_DPCC_METHODS_MAX];
	unsigned int  ro_limits;
	unsigned int  rnd_offs;
};

struct cifisp_gamma_corr_curve {
	unsigned short gamma_y[CIFISP_DEGAMMA_CURVE_SIZE];
};

struct cifisp_gamma_curve_x_axis_pnts {
	unsigned int  gamma_dx0;
	unsigned int  gamma_dx1;
};

/* Configuration used by sensor degamma */
struct cifisp_sdg_config {
	struct cifisp_gamma_corr_curve curve_r;
	struct cifisp_gamma_corr_curve curve_g;
	struct cifisp_gamma_corr_curve curve_b;
	struct cifisp_gamma_curve_x_axis_pnts xa_pnts;
};


/* Configuration used by Lens shading correction */
struct cifisp_lsc_config {
	unsigned int r_data_tbl[CIFISP_LSC_DATA_TBL_SIZE];
	unsigned int gr_data_tbl[CIFISP_LSC_DATA_TBL_SIZE];
	unsigned int gb_data_tbl[CIFISP_LSC_DATA_TBL_SIZE];
	unsigned int b_data_tbl[CIFISP_LSC_DATA_TBL_SIZE];

	unsigned int x_grad_tbl[CIFISP_LSC_GRAD_TBL_SIZE];
	unsigned int y_grad_tbl[CIFISP_LSC_GRAD_TBL_SIZE];

	unsigned int x_size_tbl[CIFISP_LSC_SIZE_TBL_SIZE];
	unsigned int y_size_tbl[CIFISP_LSC_SIZE_TBL_SIZE];
	unsigned short config_width;
	unsigned short config_height;
};

struct cifisp_ie_config {
	enum v4l2_colorfx effect;
	unsigned short color_sel;
	/*3x3 Matrix Coefficients for Emboss Effect 1*/
	unsigned short eff_mat_1;
	/*3x3 Matrix Coefficients for Emboss Effect 2*/
	unsigned short eff_mat_2;
	/*3x3 Matrix Coefficients for Emboss 3/Sketch 1*/
	unsigned short eff_mat_3;
	/*3x3 Matrix Coefficients for Sketch Effect 2*/
	unsigned short eff_mat_4;
	/*3x3 Matrix Coefficients for Sketch Effect 3*/
	unsigned short eff_mat_5;
	/*Chrominance increment values of tint (used for sepia effect)*/
	unsigned short eff_tint;
};

/* Configuration used by auto white balance */
struct cifisp_awb_meas_config {
	/*! white balance measurement window (in pixels)
	 * Note: currently the h and v offsets are mapped to grid offsets*/
	struct cifisp_window awb_wnd;
	enum cifisp_awb_mode_type awb_mode;
	/*! only pixels values < max_y contribute to awb measurement
	 * (set to 0 to disable this feature)*/
	unsigned char    max_y;
	/*! only pixels values > min_y contribute to awb measurement*/
	unsigned char    min_y;
	/*! Chrominance sum maximum value, only consider pixels with Cb+Cr
	 *  smaller than threshold for awb measurements*/
	unsigned char    max_csum;
	/*! Chrominance minimum value, only consider pixels with Cb/Cr
	 *  each greater than threshold value for awb measurements*/
	unsigned char    min_c;
	/*! number of frames - 1 used for mean value calculation
	 *  (ucFrames=0 means 1 Frame)*/
	unsigned char    frames;
	/*! reference Cr value for AWB regulation, target for AWB*/
	unsigned char    awb_ref_cr;
	/*! reference Cb value for AWB regulation, target for AWB*/
	unsigned char    awb_ref_cb;
	bool enable_ymax_cmp;
};

struct cifisp_awb_gain_config {
	unsigned short  gain_red;
	unsigned short  gain_green_r;
	unsigned short  gain_blue;
	unsigned short  gain_green_b;
};

/* Configuration used by ISP filtering */
struct cifisp_flt_config {
	enum cifisp_flt_mode  mode;    /* ISP_FILT_MODE register fields*/
	unsigned char grn_stage1;    /* ISP_FILT_MODE register fields*/
	unsigned char chr_h_mode;    /* ISP_FILT_MODE register fields*/
	unsigned char chr_v_mode;    /* ISP_FILT_MODE register fields*/
	unsigned int  thresh_bl0;
	unsigned int  thresh_bl1;
	unsigned int  thresh_sh0;
	unsigned int  thresh_sh1;
	unsigned int  lum_weight;
	unsigned int  fac_sh1;
	unsigned int  fac_sh0;
	unsigned int  fac_mid;
	unsigned int  fac_bl0;
	unsigned int  fac_bl1;
};

/* Configuration used by Bayer DeMosaic */
struct cifisp_bdm_config {
	unsigned char demosaic_th;
};

/* Configuration used by Cross Talk correction */
struct cifisp_ctk_config {
	unsigned short coeff0;
	unsigned short coeff1;
	unsigned short coeff2;
	unsigned short coeff3;
	unsigned short coeff4;
	unsigned short coeff5;
	unsigned short coeff6;
	unsigned short coeff7;
	unsigned short coeff8;
	/* offset for the crosstalk correction matrix */
	unsigned short ct_offset_r;
	unsigned short ct_offset_g;
	unsigned short ct_offset_b;
};

enum cifisp_goc_mode {
	CIFISP_GOC_MODE_LOGARITHMIC,
	CIFISP_GOC_MODE_EQUIDISTANT
};

/* Configuration used by Gamma Out correction */
struct cifisp_goc_config {
	enum cifisp_goc_mode mode;
	unsigned short gamma_y[CIFISP_GAMMA_OUT_MAX_SAMPLES];
};

/* CCM (Color Correction) */
struct cifisp_cproc_config {
	unsigned char c_out_range;
	unsigned char y_in_range;
	unsigned char y_out_range;
	unsigned char contrast;
	unsigned char brightness;
	unsigned char sat;
	unsigned char hue;
};

/* Configuration used by Histogram */
struct cifisp_hst_config {
	enum cifisp_histogram_mode mode;
	unsigned char histogram_predivider;
	struct cifisp_window meas_window;
	unsigned char hist_weight[CIFISP_HISTOGRAM_WEIGHT_GRIDS_SIZE];
};

/* Configuration used by Auto Exposure Control */
struct cifisp_aec_config {
	enum cifisp_exp_meas_mode mode;
	enum cifisp_exp_ctrl_autostop autostop;
	struct cifisp_window meas_window;
};

struct cifisp_afc_config {
	unsigned char num_afm_win;	/* max CIFISP_AFM_MAX_WINDOWS */
	struct cifisp_window afm_win[CIFISP_AFM_MAX_WINDOWS];
	unsigned int thres;
	unsigned int var_shift;
};

enum cifisp_dpf_gain_usage {
	CIFISP_DPF_GAIN_USAGE_DISABLED      = 1,   /**< don't use any gains in preprocessing stage */
	CIFISP_DPF_GAIN_USAGE_NF_GAINS      = 2,   /**< use only the noise function gains  from registers DPF_NF_GAIN_R, ... */
	CIFISP_DPF_GAIN_USAGE_LSC_GAINS     = 3,   /**< use only the gains from LSC module */
	CIFISP_DPF_GAIN_USAGE_NF_LSC_GAINS  = 4,   /**< use the moise function gains and the gains from LSC module */
	CIFISP_DPF_GAIN_USAGE_AWB_GAINS     = 5,   /**< use only the gains from AWB module */
	CIFISP_DPF_GAIN_USAGE_AWB_LSC_GAINS = 6,   /**< use the gains from AWB and LSC module */
	CIFISP_DPF_GAIN_USAGE_MAX                  /**< upper border (only for an internal evaluation) */
};

enum cifisp_dpf_rb_filtersize {
	CIFISP_DPF_RB_FILTERSIZE_13x9      = 0,    /**< red and blue filter kernel size 13x9 (means 7x5 active pixel) */
	CIFISP_DPF_RB_FILTERSIZE_9x9       = 1,    /**< red and blue filter kernel size 9x9 (means 5x5 active pixel) */	
};

enum cifisp_dpf_nll_scale_mode {
	CIFISP_NLL_SCALE_LINEAR        = 0,        /**< use a linear scaling */
	CIFISP_NLL_SCALE_LOGARITHMIC   = 1,        /**< use a logarithmic scaling */
} ;

struct cifisp_dpf_nll {
	unsigned short coeff[CIFISP_DPF_MAX_NLF_COEFFS];
	enum cifisp_dpf_nll_scale_mode scale_mode;
};

struct cifisp_dpf_rb_flt {
	enum cifisp_dpf_rb_filtersize fltsize;
	unsigned char spatial_coeff[CIFISP_DPF_MAX_SPATIAL_COEFFS];
	bool r_enable;
	bool b_enable;
};

struct cifisp_dpf_g_flt {
	unsigned char spatial_coeff[CIFISP_DPF_MAX_SPATIAL_COEFFS];
	bool gr_enable;
	bool gb_enable;
};

struct cifisp_dpf_gain {
	enum cifisp_dpf_gain_usage mode;
	unsigned short nf_r_gain;
	unsigned short nf_b_gain;
	unsigned short nf_gr_gain;
	unsigned short nf_gb_gain;
};

struct cifisp_dpf_config {
	struct cifisp_dpf_gain gain;
	struct cifisp_dpf_g_flt g_flt;
	struct cifisp_dpf_rb_flt rb_flt;
	struct cifisp_dpf_nll nll;
};

struct cifisp_dpf_strength_config {
	unsigned char r;
	unsigned char g;
	unsigned char b;
};

struct cifisp_last_capture_config {
	struct cifisp_cproc_config cproc;
	struct cifisp_goc_config   goc;
	struct cifisp_ctk_config   ctk;
	struct cifisp_bdm_config   bdm;
	struct cifisp_flt_config   flt;
	struct cifisp_awb_gain_config awb_gain;
	struct cifisp_awb_meas_config awb_meas;
	struct cifisp_lsc_config lsc;
	struct cifisp_sdg_config sdg;
	struct cifisp_bls_config bls;
};

enum cifisp_wdr_mode {
	CIFISP_WDR_MODE_BLACK = 0x140d3,
	CIFISP_WDR_MODE_GLOBAL = 0x140d2
};

/* Configuration used by Gamma Out correction */
struct cifisp_wdr_config {
	enum cifisp_wdr_mode mode;
	unsigned short c_wdr[CIFISP_WDR_SIZE];
};

/*Private IOCTLs */
/* DPCC */
#define CIFISP_IOC_G_DPCC \
	_IOR('v', BASE_VIDIOC_PRIVATE + 0, struct cifisp_dpcc_config)
#define CIFISP_IOC_S_DPCC \
	_IOW('v', BASE_VIDIOC_PRIVATE + 1, struct cifisp_dpcc_config)
/* Black Level Subtraction */
#define CIFISP_IOC_G_BLS \
	_IOR('v', BASE_VIDIOC_PRIVATE + 2, struct cifisp_bls_config)
#define CIFISP_IOC_S_BLS \
	_IOW('v', BASE_VIDIOC_PRIVATE + 3, struct cifisp_bls_config)
/* Sensor DeGamma */
#define CIFISP_IOC_G_SDG \
	_IOR('v', BASE_VIDIOC_PRIVATE + 4, struct cifisp_sdg_config)
#define CIFISP_IOC_S_SDG \
	_IOW('v', BASE_VIDIOC_PRIVATE + 5, struct cifisp_sdg_config)
/* Lens Shading Correction */
#define CIFISP_IOC_G_LSC \
	_IOR('v', BASE_VIDIOC_PRIVATE + 6, struct cifisp_lsc_config)
#define CIFISP_IOC_S_LSC \
	_IOW('v', BASE_VIDIOC_PRIVATE + 7, struct cifisp_lsc_config)
/* Auto White Balance */
#define CIFISP_IOC_G_AWB_MEAS \
	_IOR('v', BASE_VIDIOC_PRIVATE + 8, struct cifisp_awb_meas_config)
#define CIFISP_IOC_S_AWB_MEAS \
	_IOW('v', BASE_VIDIOC_PRIVATE + 9, struct cifisp_awb_meas_config)
/* ISP Filtering( Sharpening & Noise reduction */
#define CIFISP_IOC_G_FLT \
	_IOR('v', BASE_VIDIOC_PRIVATE + 10, struct cifisp_flt_config)
#define CIFISP_IOC_S_FLT \
	_IOW('v', BASE_VIDIOC_PRIVATE + 11, struct cifisp_flt_config)
/* Bayer Demosaic */
#define CIFISP_IOC_G_BDM \
	_IOR('v', BASE_VIDIOC_PRIVATE + 12, struct cifisp_bdm_config)
#define CIFISP_IOC_S_BDM \
	_IOW('v', BASE_VIDIOC_PRIVATE + 13, struct cifisp_bdm_config)
/* Cross Talk correction */
#define CIFISP_IOC_G_CTK \
	_IOR('v', BASE_VIDIOC_PRIVATE + 14, struct cifisp_ctk_config)
#define CIFISP_IOC_S_CTK \
	_IOW('v', BASE_VIDIOC_PRIVATE + 15, struct cifisp_ctk_config)
/* Gamma Out Correction */
#define CIFISP_IOC_G_GOC \
	_IOR('v', BASE_VIDIOC_PRIVATE + 16, struct cifisp_goc_config)
#define CIFISP_IOC_S_GOC \
	_IOW('v', BASE_VIDIOC_PRIVATE + 17, struct cifisp_goc_config)
/* Histogram Measurement */
#define CIFISP_IOC_G_HST \
	_IOR('v', BASE_VIDIOC_PRIVATE + 18, struct cifisp_hst_config)
#define CIFISP_IOC_S_HST \
	_IOW('v', BASE_VIDIOC_PRIVATE + 19, struct cifisp_hst_config)
/* Auto Exposure Measurements */
#define CIFISP_IOC_G_AEC \
	_IOR('v', BASE_VIDIOC_PRIVATE + 20, struct cifisp_aec_config)
#define CIFISP_IOC_S_AEC \
	_IOW('v', BASE_VIDIOC_PRIVATE + 21, struct cifisp_aec_config)
#define CIFISP_IOC_G_BPL \
	_IOR('v', BASE_VIDIOC_PRIVATE + 22, struct cifisp_aec_config)
#define CIFISP_IOC_G_AWB_GAIN \
	_IOR('v', BASE_VIDIOC_PRIVATE + 23, struct cifisp_awb_gain_config)
#define CIFISP_IOC_S_AWB_GAIN \
	_IOW('v', BASE_VIDIOC_PRIVATE + 24, struct cifisp_awb_gain_config)
#define CIFISP_IOC_G_CPROC \
	_IOR('v', BASE_VIDIOC_PRIVATE + 25, struct cifisp_cproc_config)
#define CIFISP_IOC_S_CPROC \
	_IOW('v', BASE_VIDIOC_PRIVATE + 26, struct cifisp_cproc_config)
#define CIFISP_IOC_G_AFC \
	_IOR('v', BASE_VIDIOC_PRIVATE + 27, struct cifisp_afc_config)
#define CIFISP_IOC_S_AFC \
	_IOW('v', BASE_VIDIOC_PRIVATE + 28, struct cifisp_afc_config)
#define CIFISP_IOC_G_IE \
	_IOR('v', BASE_VIDIOC_PRIVATE + 29, struct cifisp_ie_config)
#define CIFISP_IOC_S_IE \
	_IOW('v', BASE_VIDIOC_PRIVATE + 30, struct cifisp_ie_config)
#define CIFISP_IOC_G_DPF \
	_IOR('v', BASE_VIDIOC_PRIVATE + 31, struct cifisp_dpf_config)
#define CIFISP_IOC_S_DPF \
	_IOW('v', BASE_VIDIOC_PRIVATE + 32, struct cifisp_dpf_config)
#define CIFISP_IOC_G_DPF_STRENGTH \
	_IOR('v', BASE_VIDIOC_PRIVATE + 33, struct cifisp_dpf_strength_config)
#define CIFISP_IOC_S_DPF_STRENGTH \
	_IOW('v', BASE_VIDIOC_PRIVATE + 34, struct cifisp_dpf_strength_config)
#define CIFISP_IOC_G_LAST_CONFIG \
	_IOR('v', BASE_VIDIOC_PRIVATE + 35, struct cifisp_last_capture_config)
#define CIFISP_IOC_S_WDR \
	_IOR('v', BASE_VIDIOC_PRIVATE + 36, struct cifisp_wdr_config)
#define CIFISP_IOC_G_WDR \
	_IOR('v', BASE_VIDIOC_PRIVATE + 37, struct cifisp_wdr_config)

/*  CIF-ISP Private control IDs */
#define V4L2_CID_CIFISP_DPCC    (V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_CIFISP_BLS    (V4L2_CID_PRIVATE_BASE + 1)
#define V4L2_CID_CIFISP_SDG    (V4L2_CID_PRIVATE_BASE + 2)
#define V4L2_CID_CIFISP_LSC    (V4L2_CID_PRIVATE_BASE + 3)
#define V4L2_CID_CIFISP_AWB_MEAS    (V4L2_CID_PRIVATE_BASE + 4)
#define V4L2_CID_CIFISP_FLT    (V4L2_CID_PRIVATE_BASE + 5)
#define V4L2_CID_CIFISP_BDM    (V4L2_CID_PRIVATE_BASE + 6)
#define V4L2_CID_CIFISP_CTK    (V4L2_CID_PRIVATE_BASE + 7)
#define V4L2_CID_CIFISP_GOC    (V4L2_CID_PRIVATE_BASE + 8)
#define V4L2_CID_CIFISP_HST    (V4L2_CID_PRIVATE_BASE + 9)
#define V4L2_CID_CIFISP_AEC    (V4L2_CID_PRIVATE_BASE + 10)
#define V4L2_CID_CIFISP_AWB_GAIN    (V4L2_CID_PRIVATE_BASE + 11)
#define V4L2_CID_CIFISP_CPROC    (V4L2_CID_PRIVATE_BASE + 12)
#define V4L2_CID_CIFISP_AFC    (V4L2_CID_PRIVATE_BASE + 13)
#define V4L2_CID_CIFISP_IE    (V4L2_CID_PRIVATE_BASE + 14)
#define V4L2_CID_CIFISP_DPF    (V4L2_CID_PRIVATE_BASE + 15)
#define V4L2_CID_CIFISP_WDR   (V4L2_CID_PRIVATE_BASE + 16)

/* Camera Sensors' running modes */
#define CI_MODE_PREVIEW	0x8000
#define CI_MODE_VIDEO	0x4000
#define CI_MODE_STILL_CAPTURE	0x2000
#define CI_MODE_CONTINUOUS	0x1000
#define CI_MODE_NONE	0x0000

#endif
