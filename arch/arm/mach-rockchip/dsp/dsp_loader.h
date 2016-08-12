/**
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd
 * author: ZhiChao Yu zhichao.yu@rock-chips.com
 *
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
#ifndef _ARCH_ARM_MACH_RK_DSP_LOADER_H_
#define _ARCH_ARM_MACH_RK_DSP_LOADER_H_

#include <linux/firmware.h>
#include "dsp_dma.h"

/*
 * DSP image ids
 */
#define DSP_IMAGE_MAIN    1
#define DSP_IMAGE_3DNR    2

#define DSP_IMAGE_NAME_SIZE            32
#define DSP_IMAGE_MAX_SECTION          4

/*
 * These macros should be same as render types in dsp_work.h
 */
#define DSP_IMAGE_CAP_OS      0x00000001
#define DSP_IMAGE_CAP_ADAS    0x00000002
#define DSP_IMAGE_CAP_2DNR    0x00000004
#define DSP_IMAGE_CAP_3DNR    0x00000008
#define DSP_IMAGE_CAP_HDR     0x00000010
#define DSP_IMAGE_CAP_DEFOG   0x00000020
#define DSP_IMAGE_CAP_LLE     0x00000040

#define DSP_IMAGE_CODE_INTERNAL 0x0
#define DSP_IMAGE_CODE_EXTERNAL 0x1
#define DSP_IMAGE_DATA_INTERNAL 0x2
#define DSP_IMAGE_DATA_EXTERNAL 0x3

struct dsp_section {
	u32 valid;
	u32 type;
	u32 size;
	void *src;
	void *dst;
};

struct dsp_image {
	u32 id;
	char name[DSP_IMAGE_NAME_SIZE];
	struct dsp_section sections[DSP_IMAGE_MAX_SECTION];
	struct list_head list_node;
};

struct dsp_loader {
	struct dsp_dma *dma;
	struct list_head images;

	/* Reserved dsp external text memory */
	u8 *external_text;

	int (*load_image)(struct dsp_loader *, u32 id);
};

void dsp_loader_request_firmware(const struct firmware *fw, void *context);

int dsp_loader_create(struct dsp_dma *dma, struct dsp_loader **loader);

int dsp_loader_destroy(struct dsp_loader *loader);

#endif
