// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Rockchip Electronics Co. Ltd.
 *
 * Author: Zorro Liu <zorro.liu@rock-chips.com>
 */

#ifndef EPD_LUT_H
#define EPD_LUT_H

// same to pvi_wf_mode
enum epd_lut_type {
	WF_TYPE_RESET = 0,
	WF_TYPE_GRAY2,	// like DU
	WF_TYPE_GRAY4,  // like DU4
	WF_TYPE_GC16,
	WF_TYPE_GL16,
	WF_TYPE_GLR16,
	WF_TYPE_GLD16,
	WF_TYPE_A2,
	WF_TYPE_GCC16,
	PVI_WF_MAX,

	WF_TYPE_AUTO,	// like GC16, rk define
	WF_TYPE_MAX,
	WF_TYPE_GRAY16,
};

struct epd_lut_data {
	unsigned int frame_num;
	unsigned int *data;
	u8 *wf_table[2];
};

/*
 * EPD LUT module export symbols
 */
int epd_lut_from_mem_init(void *waveform);
int epd_lut_from_file_init(struct device *dev, void *waveform, int size);
const char *epd_lut_get_wf_version(void);
int epd_lut_get(struct epd_lut_data *output, enum epd_lut_type lut_type, int temperture);

//you can change overlay lut mode here
int epd_overlay_lut(void);

/*
 * PVI Waveform Interfaces
 */
int pvi_wf_input(void *waveform_file);
const char *pvi_wf_get_version(void);
int pvi_wf_get_lut(struct epd_lut_data *output, enum epd_lut_type lut_type, int temperture);

/*
 * RKF Waveform Interfaces
 */
int rkf_wf_input(void *waveform_file);
const char *rkf_wf_get_version(void);
int rkf_wf_get_lut(struct epd_lut_data *output, enum epd_lut_type lut_type, int temperture);
#endif
