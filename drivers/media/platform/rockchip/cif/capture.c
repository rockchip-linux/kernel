// SPDX-License-Identifier: GPL-2.0
/*
 * Rockchip CIF Driver
 *
 * Copyright (C) 2018 Rockchip Electronics Co., Ltd.
 */

#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/iommu.h>
#include <media/v4l2-common.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-dma-sg.h>
#include <soc/rockchip/rockchip-system-status.h>
#include <soc/rockchip/rockchip_iommu.h>
#include <linux/rk-isp32-config.h>
#include <linux/dma-fence.h>
#include <linux/sync_file.h>
#include <linux/fdtable.h>
#include <linux/mm.h>
#include <clocksource/arm_arch_timer.h>
#include <linux/kfifo.h>
#include <linux/gpio/consumer.h>

#include "dev.h"
#include "mipi-csi2.h"
#include "common.h"
#include "rkcif-externel.h"
#include "../../../i2c/cam-tb-setup.h"

#define CIF_REQ_BUFS_MIN	1
#define CIF_MIN_WIDTH		64
#define CIF_MIN_HEIGHT		64
#define CIF_MAX_WIDTH		8192
#define CIF_MAX_HEIGHT		8192

#define OUTPUT_STEP_WISE	8

#define RKCIF_PLANE_Y		0
#define RKCIF_PLANE_CBCR	1
#define RKCIF_MAX_PLANE		3

#define STREAM_PAD_SINK		0
#define STREAM_PAD_SOURCE	1

#define CIF_TIMEOUT_FRAME_NUM	(2)

#define CIF_DVP_PCLK_DUAL_EDGE	(V4L2_MBUS_PCLK_SAMPLE_RISING |\
				 V4L2_MBUS_PCLK_SAMPLE_FALLING)

/*
 * Round up height when allocate memory so that Rockchip encoder can
 * use DMA buffer directly, though this may waste a bit of memory.
 */
#define MEMORY_ALIGN_ROUND_UP_HEIGHT		16

/* Get xsubs and ysubs for fourcc formats
 *
 * @xsubs: horizontal color samples in a 4*4 matrix, for yuv
 * @ysubs: vertical color samples in a 4*4 matrix, for yuv
 */
static int fcc_xysubs(u32 fcc, u32 *xsubs, u32 *ysubs)
{
	switch (fcc) {
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
		*xsubs = 2;
		*ysubs = 1;
		break;
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV12:
		*xsubs = 2;
		*ysubs = 2;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct cif_output_fmt out_fmts[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV16,
		.cplanes = 2,
		.mplanes = 1,
		.fmt_val = YUV_OUTPUT_422 | UV_STORAGE_ORDER_UVUV,
		.bpp = { 8, 16 },
		.csi_fmt_val = CSI_WRDDR_TYPE_YUV422,
		.fmt_type = CIF_FMT_TYPE_YUV,
	}, {
		.fourcc = V4L2_PIX_FMT_NV61,
		.fmt_val = YUV_OUTPUT_422 | UV_STORAGE_ORDER_VUVU,
		.cplanes = 2,
		.mplanes = 1,
		.bpp = { 8, 16 },
		.csi_fmt_val = CSI_WRDDR_TYPE_YUV422,
		.fmt_type = CIF_FMT_TYPE_YUV,
	}, {
		.fourcc = V4L2_PIX_FMT_NV12,
		.fmt_val = YUV_OUTPUT_420 | UV_STORAGE_ORDER_UVUV,
		.cplanes = 2,
		.mplanes = 1,
		.bpp = { 8, 16 },
		.csi_fmt_val = CSI_WRDDR_TYPE_YUV420SP,
		.fmt_type = CIF_FMT_TYPE_YUV,
	}, {
		.fourcc = V4L2_PIX_FMT_NV21,
		.fmt_val = YUV_OUTPUT_420 | UV_STORAGE_ORDER_VUVU,
		.cplanes = 2,
		.mplanes = 1,
		.bpp = { 8, 16 },
		.csi_fmt_val = CSI_WRDDR_TYPE_YUV420SP,
		.fmt_type = CIF_FMT_TYPE_YUV,
	}, {
		.fourcc = V4L2_PIX_FMT_YUYV,
		.cplanes = 2,
		.mplanes = 1,
		.bpp = { 8, 16 },
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW8,
		.fmt_type = CIF_FMT_TYPE_YUV,
	}, {
		.fourcc = V4L2_PIX_FMT_YVYU,
		.cplanes = 2,
		.mplanes = 1,
		.bpp = { 8, 16 },
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW8,
		.fmt_type = CIF_FMT_TYPE_YUV,
	}, {
		.fourcc = V4L2_PIX_FMT_UYVY,
		.cplanes = 2,
		.mplanes = 1,
		.bpp = { 8, 16 },
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW8,
		.fmt_type = CIF_FMT_TYPE_YUV,
	}, {
		.fourcc = V4L2_PIX_FMT_VYUY,
		.cplanes = 2,
		.mplanes = 1,
		.bpp = { 8, 16 },
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW8,
		.fmt_type = CIF_FMT_TYPE_YUV,
	}, {
		.fourcc = V4L2_PIX_FMT_RGB24,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 24 },
		.csi_fmt_val = CSI_WRDDR_TYPE_RGB888,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_BGR24,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 24 },
		.csi_fmt_val = CSI_WRDDR_TYPE_RGB888,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_RGB565,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.csi_fmt_val = CSI_WRDDR_TYPE_RGB565,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_BGR666,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 18 },
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB8,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 8 },
		.raw_bpp = 8,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW8,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG8,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 8 },
		.raw_bpp = 8,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW8,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG8,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 8 },
		.raw_bpp = 8,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW8,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR8,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 8 },
		.raw_bpp = 8,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW8,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB10,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 10,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW10,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG10,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 10,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW10,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG10,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 10,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW10,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR10,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 10,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW10,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB12,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 12,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW12,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG12,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 12,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW12,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG12,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 12,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW12,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR12,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 12,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW12,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR16,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 16,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW8,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG16,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 16,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW8,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG16,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 16,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW8,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB16,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 16,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW8,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_Y16,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 16,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW8,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_GREY,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = {8},
		.raw_bpp = 8,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW8,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4l2_PIX_FMT_EBD8,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = {8},
		.raw_bpp = 8,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW8,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4l2_PIX_FMT_SPD16,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = {16},
		.raw_bpp = 16,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW8,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_Y12,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 12,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW12,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_Y10,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 10,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW10,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB16,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 16,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW16,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG16,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 16,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW16,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG16,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 16,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW16,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR16,
		.cplanes = 1,
		.mplanes = 1,
		.bpp = { 16 },
		.raw_bpp = 16,
		.csi_fmt_val = CSI_WRDDR_TYPE_RAW16,
		.fmt_type = CIF_FMT_TYPE_RAW,
	}
	/* TODO: We can support NV12M/NV21M/NV16M/NV61M too */
};

static const struct cif_input_fmt in_fmts[] = {
	{
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_2X8,
		.dvp_fmt_val	= YUV_INPUT_422 | YUV_INPUT_ORDER_YUYV,
		.csi_fmt_val	= CSI_WRDDR_TYPE_YUV422,
		.csi_yuv_order	= CSI_YUV_INPUT_ORDER_YUYV,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_2X8,
		.dvp_fmt_val	= YUV_INPUT_422 | YUV_INPUT_ORDER_YUYV,
		.csi_fmt_val	= CSI_WRDDR_TYPE_YUV422,
		.csi_yuv_order	= CSI_YUV_INPUT_ORDER_YUYV,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YVYU8_2X8,
		.dvp_fmt_val	= YUV_INPUT_422 | YUV_INPUT_ORDER_YVYU,
		.csi_fmt_val	= CSI_WRDDR_TYPE_YUV422,
		.csi_yuv_order	= CSI_YUV_INPUT_ORDER_YVYU,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YVYU8_2X8,
		.dvp_fmt_val	= YUV_INPUT_422 | YUV_INPUT_ORDER_YVYU,
		.csi_fmt_val	= CSI_WRDDR_TYPE_YUV422,
		.csi_yuv_order	= CSI_YUV_INPUT_ORDER_YVYU,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_2X8,
		.dvp_fmt_val	= YUV_INPUT_422 | YUV_INPUT_ORDER_UYVY,
		.csi_fmt_val	= CSI_WRDDR_TYPE_YUV422,
		.csi_yuv_order	= CSI_YUV_INPUT_ORDER_UYVY,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_UYVY8_2X8,
		.dvp_fmt_val	= YUV_INPUT_422 | YUV_INPUT_ORDER_UYVY,
		.csi_fmt_val	= CSI_WRDDR_TYPE_YUV422,
		.csi_yuv_order	= CSI_YUV_INPUT_ORDER_UYVY,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_VYUY8_2X8,
		.dvp_fmt_val	= YUV_INPUT_422 | YUV_INPUT_ORDER_VYUY,
		.csi_fmt_val	= CSI_WRDDR_TYPE_YUV422,
		.csi_yuv_order	= CSI_YUV_INPUT_ORDER_VYUY,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_VYUY8_2X8,
		.dvp_fmt_val	= YUV_INPUT_422 | YUV_INPUT_ORDER_VYUY,
		.csi_fmt_val	= CSI_WRDDR_TYPE_YUV422,
		.csi_yuv_order	= CSI_YUV_INPUT_ORDER_VYUY,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_INTERLACED,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR8_1X8,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_8,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW8,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG8_1X8,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_8,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW8,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG8_1X8,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_8,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW8,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB8_1X8,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_8,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW8,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR10_1X10,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_10,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW10,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG10_1X10,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_10,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW10,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG10_1X10,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_10,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW10,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB10_1X10,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_10,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW10,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR12_1X12,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_12,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW12,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG12_1X12,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_12,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW12,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG12_1X12,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_12,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW12,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB12_1X12,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_12,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW12,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR14_1X14,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW14_RK3588,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG14_1X14,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW14_RK3588,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG14_1X14,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW14_RK3588,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB14_1X14,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW14_RK3588,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_RGB888_1X24,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RGB888,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_BGR888_1X24,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RGB888,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_GBR888_1X24,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RGB888,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_RGB565_1X16,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RGB565,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y8_1X8,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_8,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW8,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y10_1X10,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_10,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW10,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_Y12_1X12,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_12,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW12,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_EBD_1X8,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_8,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW8,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SPD_2X8,
		.dvp_fmt_val	= INPUT_MODE_RAW | RAW_DATA_WIDTH_12,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW12,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_YUV8_1X24,//use for yuv420_8bit input
		.csi_fmt_val	= CSI_WRDDR_TYPE_YUV420SP,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_UV8_1X8,//use for yuv420_8bit legacy input
		.csi_fmt_val	= CSI_WRDDR_TYPE_YUV420LEGACY,
		.fmt_type	= CIF_FMT_TYPE_YUV,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SBGGR16_1X16,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW16,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGBRG16_1X16,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW16,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SGRBG16_1X16,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW16,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	}, {
		.mbus_code	= MEDIA_BUS_FMT_SRGGB16_1X16,
		.csi_fmt_val	= CSI_WRDDR_TYPE_RAW16,
		.fmt_type	= CIF_FMT_TYPE_RAW,
		.field		= V4L2_FIELD_NONE,
	},
};

static inline int rkcif_get_interlace_mode(struct rkcif_stream *stream)
{
	if (stream->cif_fmt_in->field == V4L2_FIELD_INTERLACED) {
		if (stream->cifdev->use_hw_interlace)
			return RKCIF_INTERLACE_HW;
		else if (stream->cifdev->chip_id >= CHIP_RK3568_CIF)
			return RKCIF_INTERLACE_SOFT_AUTO;
		else
			return RKCIF_INTERLACE_SOFT;
	}
	return RKCIF_INTERLACE_NONE;
}

static int rkcif_output_fmt_check(struct rkcif_stream *stream,
				  const struct cif_output_fmt *output_fmt)
{
	const struct cif_input_fmt *input_fmt = stream->cif_fmt_in;
	struct csi_channel_info *channel = &stream->cifdev->channels[stream->id];
	int ret = -EINVAL;

	stream->rounding_bit = 0;
	switch (input_fmt->mbus_code) {
	case MEDIA_BUS_FMT_YUYV8_2X8:
	case MEDIA_BUS_FMT_YVYU8_2X8:
	case MEDIA_BUS_FMT_UYVY8_2X8:
	case MEDIA_BUS_FMT_VYUY8_2X8:
		if (output_fmt->fourcc == V4L2_PIX_FMT_NV16 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_NV61 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_NV12 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_NV21 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_YUYV ||
		    output_fmt->fourcc == V4L2_PIX_FMT_YVYU ||
		    output_fmt->fourcc == V4L2_PIX_FMT_UYVY ||
		    output_fmt->fourcc == V4L2_PIX_FMT_VYUY)
			ret = 0;
		break;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_Y8_1X8:
		if (output_fmt->fourcc == V4L2_PIX_FMT_SRGGB8 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_SGRBG8 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_SGBRG8 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_SBGGR8 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_GREY)
			ret = 0;
		break;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_Y10_1X10:
		if (output_fmt->fourcc == V4L2_PIX_FMT_SRGGB10 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_SGRBG10 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_SGBRG10 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_SBGGR10 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_Y10) {
			ret = 0;
		} else if (stream->cifdev->chip_id >= CHIP_RV1103B_CIF &&
			   (output_fmt->fourcc == V4L2_PIX_FMT_SRGGB8 ||
			    output_fmt->fourcc == V4L2_PIX_FMT_SGRBG8 ||
			    output_fmt->fourcc == V4L2_PIX_FMT_SGBRG8 ||
			    output_fmt->fourcc == V4L2_PIX_FMT_SBGGR8 ||
			    output_fmt->fourcc == V4L2_PIX_FMT_GREY)) {
			ret = 0;
			stream->rounding_bit = ROUNDING_2BIT_RV1103B;
		}
		break;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_Y12_1X12:
		if (output_fmt->fourcc == V4L2_PIX_FMT_SRGGB12 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_SGRBG12 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_SGBRG12 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_SBGGR12 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_Y12) {
			ret = 0;
		} else if (stream->cifdev->chip_id >= CHIP_RV1103B_CIF &&
			   (output_fmt->fourcc == V4L2_PIX_FMT_SRGGB10 ||
			    output_fmt->fourcc == V4L2_PIX_FMT_SGRBG10 ||
			    output_fmt->fourcc == V4L2_PIX_FMT_SGBRG10 ||
			    output_fmt->fourcc == V4L2_PIX_FMT_SBGGR10 ||
			    output_fmt->fourcc == V4L2_PIX_FMT_Y10)) {
			ret = 0;
			stream->rounding_bit = ROUNDING_2BIT_RV1103B;
		} else if (stream->cifdev->chip_id >= CHIP_RV1103B_CIF &&
			   (output_fmt->fourcc == V4L2_PIX_FMT_SRGGB8 ||
			    output_fmt->fourcc == V4L2_PIX_FMT_SGRBG8 ||
			    output_fmt->fourcc == V4L2_PIX_FMT_SGBRG8 ||
			    output_fmt->fourcc == V4L2_PIX_FMT_SBGGR8 ||
			    output_fmt->fourcc == V4L2_PIX_FMT_GREY)) {
			ret = 0;
			stream->rounding_bit = ROUNDING_4BIT_RV1103B;
		}
		break;
	case MEDIA_BUS_FMT_RGB888_1X24:
	case MEDIA_BUS_FMT_BGR888_1X24:
	case MEDIA_BUS_FMT_GBR888_1X24:
		if (output_fmt->fourcc == V4L2_PIX_FMT_RGB24 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_BGR24)
			ret = 0;
		break;
	case MEDIA_BUS_FMT_RGB565_1X16:
		if (output_fmt->fourcc == V4L2_PIX_FMT_RGB565)
			ret = 0;
		break;
	case MEDIA_BUS_FMT_EBD_1X8:
		if (output_fmt->fourcc == V4l2_PIX_FMT_EBD8 ||
		    (channel->data_bit == 8 &&
		     (output_fmt->fourcc == V4L2_PIX_FMT_SRGGB8 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SGRBG8 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SGBRG8 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SBGGR8)) ||
		    (channel->data_bit == 10 &&
		     (output_fmt->fourcc == V4L2_PIX_FMT_SRGGB10 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SGRBG10 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SGBRG10 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SBGGR10)) ||
		    (channel->data_bit == 12 &&
		     (output_fmt->fourcc == V4L2_PIX_FMT_SRGGB12 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SGRBG12 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SGBRG12 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SBGGR12)) ||
		    (channel->data_bit == 16 &&
		     (output_fmt->fourcc == V4L2_PIX_FMT_SRGGB16 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SGRBG16 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SGBRG16 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SBGGR16)))
			ret = 0;
		break;
	case MEDIA_BUS_FMT_SPD_2X8:
		if (output_fmt->fourcc == V4l2_PIX_FMT_SPD16 ||
		    (channel->data_bit == 8 &&
		     (output_fmt->fourcc == V4L2_PIX_FMT_SRGGB8 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SGRBG8 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SGBRG8 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SBGGR8)) ||
		    (channel->data_bit == 10 &&
		     (output_fmt->fourcc == V4L2_PIX_FMT_SRGGB10 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SGRBG10 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SGBRG10 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SBGGR10)) ||
		    (channel->data_bit == 12 &&
		     (output_fmt->fourcc == V4L2_PIX_FMT_SRGGB12 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SGRBG12 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SGBRG12 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SBGGR12)) ||
		    (channel->data_bit == 16 &&
		     (output_fmt->fourcc == V4L2_PIX_FMT_SRGGB16 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SGRBG16 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SGBRG16 ||
		      output_fmt->fourcc == V4L2_PIX_FMT_SBGGR16)))
			ret = 0;
		break;
	case MEDIA_BUS_FMT_SBGGR16_1X16:
	case MEDIA_BUS_FMT_SGBRG16_1X16:
	case MEDIA_BUS_FMT_SGRBG16_1X16:
	case MEDIA_BUS_FMT_SRGGB16_1X16:
		if (output_fmt->fourcc == V4L2_PIX_FMT_SRGGB16 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_SGRBG16 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_SGBRG16 ||
		    output_fmt->fourcc == V4L2_PIX_FMT_SBGGR16)
			ret = 0;
		break;
	case MEDIA_BUS_FMT_UV8_1X8:
	case MEDIA_BUS_FMT_YUV8_1X24:
		if (output_fmt->fourcc == V4L2_PIX_FMT_NV12)
			ret = 0;
		break;
	default:
		break;
	}
	if (ret)
		v4l2_dbg(4, rkcif_debug, &stream->cifdev->v4l2_dev,
			 "input mbus_code 0x%x, can't transform to %c%c%c%c\n",
			 input_fmt->mbus_code,
			 output_fmt->fourcc & 0xff,
			 (output_fmt->fourcc >> 8) & 0xff,
			 (output_fmt->fourcc >> 16) & 0xff,
			 (output_fmt->fourcc >> 24) & 0xff);
	return ret;
}

static int rkcif_stop_dma_capture(struct rkcif_stream *stream);

struct rkcif_rx_buffer *to_cif_rx_buf(struct rkisp_rx_buf *dbufs)
{
	return container_of(dbufs, struct rkcif_rx_buffer, dbufs);
}

static struct v4l2_subdev *get_remote_sensor(struct rkcif_stream *stream, u16 *index)
{
	struct media_pad *local, *remote;
	struct media_entity *sensor_me;
	struct v4l2_subdev *sub = NULL;

	local = &stream->vnode.vdev.entity.pads[0];
	if (!local) {
		v4l2_err(&stream->cifdev->v4l2_dev,
			 "%s: video pad[0] is null\n", __func__);
		return NULL;
	}

	remote = media_pad_remote_pad_first(local);
	if (!remote) {
		v4l2_err(&stream->cifdev->v4l2_dev,
			 "%s: remote pad is null\n", __func__);
		return NULL;
	}

	if (index)
		*index = remote->index;

	sensor_me = remote->entity;

	sub = media_entity_to_v4l2_subdev(sensor_me);

	return sub;

}

static void get_remote_terminal_sensor(struct rkcif_stream *stream,
				       struct v4l2_subdev **sensor_sd)
{
	struct media_graph graph;
	struct media_entity *entity = &stream->vnode.vdev.entity;
	struct media_device *mdev = entity->graph_obj.mdev;
	int ret;

	/* Walk the graph to locate sensor nodes. */
	mutex_lock(&mdev->graph_mutex);
	ret = media_graph_walk_init(&graph, mdev);
	if (ret) {
		mutex_unlock(&mdev->graph_mutex);
		*sensor_sd = NULL;
		return;
	}

	media_graph_walk_start(&graph, entity);
	while ((entity = media_graph_walk_next(&graph))) {
		if (entity->function == MEDIA_ENT_F_CAM_SENSOR)
			break;
	}
	mutex_unlock(&mdev->graph_mutex);
	media_graph_walk_cleanup(&graph);

	if (entity)
		*sensor_sd = media_entity_to_v4l2_subdev(entity);
	else
		*sensor_sd = NULL;
}

static struct rkcif_sensor_info *sd_to_sensor(struct rkcif_device *dev,
					      struct v4l2_subdev *sd)
{
	u32 i;

	for (i = 0; i < dev->num_sensors; ++i)
		if (dev->sensors[i].sd == sd)
			return &dev->sensors[i];

	if (i == dev->num_sensors) {
		for (i = 0; i < dev->num_sensors; ++i) {
			if (dev->sensors[i].mbus.type == V4L2_MBUS_CCP2)
				return &dev->lvds_subdev.sensor_self;
		}
	}

	return NULL;
}

static unsigned char get_data_type(u32 pixelformat, u8 cmd_mode_en, u8 dsi_input)
{
	switch (pixelformat) {
	/* csi raw8 */
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_Y8_1X8:
		return 0x2a;
	/* csi raw10 */
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_Y10_1X10:
		return 0x2b;
	/* csi raw12 */
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_Y12_1X12:
		return 0x2c;
	/* csi raw16 */
	case MEDIA_BUS_FMT_SBGGR16_1X16:
	case MEDIA_BUS_FMT_SGBRG16_1X16:
	case MEDIA_BUS_FMT_SGRBG16_1X16:
	case MEDIA_BUS_FMT_SRGGB16_1X16:
		return 0x2e;
	/* csi uyvy 422 */
	case MEDIA_BUS_FMT_UYVY8_2X8:
	case MEDIA_BUS_FMT_VYUY8_2X8:
	case MEDIA_BUS_FMT_YUYV8_2X8:
	case MEDIA_BUS_FMT_YVYU8_2X8:
		return 0x1e;
	case MEDIA_BUS_FMT_RGB888_1X24:
	case MEDIA_BUS_FMT_BGR888_1X24:
	case MEDIA_BUS_FMT_GBR888_1X24:
		if (dsi_input) {
			if (cmd_mode_en) /* dsi command mode*/
				return 0x39;
			else /* dsi video mode */
				return 0x3e;
		} else {
			return 0x24;
		}
	case MEDIA_BUS_FMT_RGB565_1X16:
		if (dsi_input) {
			if (cmd_mode_en) /* dsi command mode*/
				return 0x39;
			else /* dsi video mode */
				return 0x0e;
		} else {
			return 0x22;
		}
	case MEDIA_BUS_FMT_EBD_1X8:
		return 0x12;
	case MEDIA_BUS_FMT_SPD_2X8:
		return 0x2f;
	case MEDIA_BUS_FMT_UV8_1X8:
		return 0x1a;//use for yuv420_8bit legacy input
	case MEDIA_BUS_FMT_YUV8_1X24:
		return 0x18;//use for yuv420_8bit input
	default:
		return 0x2b;
	}
}

static int get_csi_crop_align(const struct cif_input_fmt *fmt_in)
{
	switch (fmt_in->csi_fmt_val) {
	case CSI_WRDDR_TYPE_RGB888:
		return 24;
	case CSI_WRDDR_TYPE_RGB565:
		return 16;
	case CSI_WRDDR_TYPE_RAW10:
	case CSI_WRDDR_TYPE_RAW12:
		return 4;
	case CSI_WRDDR_TYPE_RAW8:
	case CSI_WRDDR_TYPE_YUV422:
		return 8;
	default:
		return -1;
	}
}

const struct
cif_input_fmt *rkcif_get_input_fmt(struct rkcif_device *dev, struct v4l2_rect *rect,
			     u32 pad_id, struct csi_channel_info *csi_info)
{
	struct v4l2_subdev_format fmt = {0};
	struct v4l2_subdev *sd = dev->terminal_sensor.sd;
	struct rkmodule_channel_info ch_info = {0};
	struct rkmodule_capture_info capture_info;
	int ret;
	u32 i;

	fmt.pad = 0;
	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.reserved[0] = 0;
	fmt.format.field = V4L2_FIELD_NONE;
	ret = v4l2_subdev_call(sd, pad, get_fmt, NULL, &fmt);
	if (ret < 0) {
		v4l2_warn(sd->v4l2_dev,
			  "sensor fmt invalid, set to default size\n");
		goto set_default;
	}
	ch_info.index = pad_id;
	ret = v4l2_subdev_call(sd,
			       core, ioctl,
			       RKMODULE_GET_CHANNEL_INFO,
			       &ch_info);
	if (!ret) {
		fmt.format.width = ch_info.width;
		fmt.format.height = ch_info.height;
		fmt.format.code = ch_info.bus_fmt;
		if (ch_info.vc >= 0 && ch_info.vc <= 3)
			csi_info->vc = ch_info.vc;
		else
			csi_info->vc = pad_id;
		if (ch_info.data_type > 0)
			csi_info->data_type = ch_info.data_type;
		else
			csi_info->data_type = 0;
		if (ch_info.data_bit > 0)
			csi_info->data_bit = ch_info.data_bit;
		if (ch_info.field == 0)
			fmt.format.field = V4L2_FIELD_NONE;
		else
			fmt.format.field = ch_info.field;
	} else {
		csi_info->vc = pad_id;
	}

	v4l2_dbg(1, rkcif_debug, sd->v4l2_dev,
		 "remote fmt: mbus code:0x%x, size:%dx%d, field: %d\n",
		 fmt.format.code, fmt.format.width,
		 fmt.format.height, fmt.format.field);
	rect->left = 0;
	rect->top = 0;
	rect->width = fmt.format.width;
	rect->height = fmt.format.height;
	ret = v4l2_subdev_call(sd,
			       core, ioctl,
			       RKMODULE_GET_CAPTURE_MODE,
			       &capture_info);
	if (!ret) {
		if (capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE &&
		    dev->hw_dev->is_rk3588s2) {
			for (i = 0; i < capture_info.multi_dev.dev_num; i++) {
				if (capture_info.multi_dev.dev_idx[i] == 0)
					capture_info.multi_dev.dev_idx[i] = 2;
				else if (capture_info.multi_dev.dev_idx[i] == 2)
					capture_info.multi_dev.dev_idx[i] = 4;
				else if (capture_info.multi_dev.dev_idx[i] == 3)
					capture_info.multi_dev.dev_idx[i] = 5;
			}
		}
		csi_info->capture_info = capture_info;
	}

	for (i = 0; i < ARRAY_SIZE(in_fmts); i++)
		if (fmt.format.code == in_fmts[i].mbus_code &&
		    fmt.format.field == in_fmts[i].field)
			return &in_fmts[i];

	v4l2_err(sd->v4l2_dev, "remote sensor mbus code not supported\n");

set_default:
	rect->left = 0;
	rect->top = 0;
	rect->width = RKCIF_DEFAULT_WIDTH;
	rect->height = RKCIF_DEFAULT_HEIGHT;

	return NULL;
}

const struct
cif_output_fmt *rkcif_find_output_fmt(struct rkcif_stream *stream, u32 pixelfmt)
{
	const struct cif_output_fmt *fmt;
	u32 i;

	for (i = 0; i < ARRAY_SIZE(out_fmts); i++) {
		fmt = &out_fmts[i];
		if (fmt->fourcc == pixelfmt)
			return fmt;
	}

	return NULL;
}

static enum cif_reg_index get_dvp_reg_index_of_id_ctrl0(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_DVP_ID0_CTRL0;
		break;
	case 1:
		index = CIF_REG_DVP_ID1_CTRL0;
		break;
	case 2:
		index = CIF_REG_DVP_ID2_CTRL0;
		break;
	case 3:
		index = CIF_REG_DVP_ID3_CTRL0;
		break;
	default:
		index = CIF_REG_DVP_ID0_CTRL0;
		break;
	}

	return index;
}

static enum cif_reg_index get_dvp_reg_index_of_id_ctrl1(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_DVP_ID0_CTRL1;
		break;
	case 1:
		index = CIF_REG_DVP_ID1_CTRL1;
		break;
	case 2:
		index = CIF_REG_DVP_ID2_CTRL1;
		break;
	case 3:
		index = CIF_REG_DVP_ID3_CTRL1;
		break;
	default:
		index = CIF_REG_DVP_ID0_CTRL1;
		break;
	}

	return index;
}

static enum cif_reg_index get_dvp_reg_index_of_vlw(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_DVP_VLW_ID0;
		break;
	case 1:
		index = CIF_REG_DVP_VLW_ID1;
		break;
	case 2:
		index = CIF_REG_DVP_VLW_ID2;
		break;
	case 3:
		index = CIF_REG_DVP_VLW_ID3;
		break;
	default:
		index = CIF_REG_DVP_VLW_ID0;
		break;
	}

	return index;
}

static enum cif_reg_index get_dvp_reg_index_of_id_crop_start(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_DVP_ID0_CROP_START;
		break;
	case 1:
		index = CIF_REG_DVP_ID1_CROP_START;
		break;
	case 2:
		index = CIF_REG_DVP_ID2_CROP_START;
		break;
	case 3:
		index = CIF_REG_DVP_ID3_CROP_START;
		break;
	default:
		index = CIF_REG_DVP_ID0_CROP_START;
		break;
	}

	return index;
}

static enum cif_reg_index get_reg_index_of_id_ctrl0(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_MIPI_LVDS_ID0_CTRL0;
		break;
	case 1:
		index = CIF_REG_MIPI_LVDS_ID1_CTRL0;
		break;
	case 2:
		index = CIF_REG_MIPI_LVDS_ID2_CTRL0;
		break;
	case 3:
		index = CIF_REG_MIPI_LVDS_ID3_CTRL0;
		break;
	default:
		index = CIF_REG_MIPI_LVDS_ID0_CTRL0;
		break;
	}

	return index;
}

static enum cif_reg_index get_reg_index_of_lvds_id_ctrl0(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_LVDS_ID0_CTRL0;
		break;
	case 1:
		index = CIF_REG_LVDS_ID1_CTRL0;
		break;
	case 2:
		index = CIF_REG_LVDS_ID2_CTRL0;
		break;
	case 3:
		index = CIF_REG_LVDS_ID3_CTRL0;
		break;
	default:
		index = CIF_REG_LVDS_ID0_CTRL0;
		break;
	}

	return index;
}

static enum cif_reg_index get_reg_index_of_id_ctrl1(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_MIPI_LVDS_ID0_CTRL1;
		break;
	case 1:
		index = CIF_REG_MIPI_LVDS_ID1_CTRL1;
		break;
	case 2:
		index = CIF_REG_MIPI_LVDS_ID2_CTRL1;
		break;
	case 3:
		index = CIF_REG_MIPI_LVDS_ID3_CTRL1;
		break;
	default:
		index = CIF_REG_MIPI_LVDS_ID0_CTRL1;
		break;
	}

	return index;
}

static enum cif_reg_index get_reg_index_of_frm0_y_addr(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_MIPI_LVDS_FRAME0_ADDR_Y_ID0;
		break;
	case 1:
		index = CIF_REG_MIPI_LVDS_FRAME0_ADDR_Y_ID1;
		break;
	case 2:
		index = CIF_REG_MIPI_LVDS_FRAME0_ADDR_Y_ID2;
		break;
	case 3:
		index = CIF_REG_MIPI_LVDS_FRAME0_ADDR_Y_ID3;
		break;
	default:
		index = CIF_REG_MIPI_LVDS_FRAME0_ADDR_Y_ID0;
		break;
	}

	return index;
}

static enum cif_reg_index get_reg_index_of_frm1_y_addr(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_MIPI_LVDS_FRAME1_ADDR_Y_ID0;
		break;
	case 1:
		index = CIF_REG_MIPI_LVDS_FRAME1_ADDR_Y_ID1;
		break;
	case 2:
		index = CIF_REG_MIPI_LVDS_FRAME1_ADDR_Y_ID2;
		break;
	case 3:
		index = CIF_REG_MIPI_LVDS_FRAME1_ADDR_Y_ID3;
		break;
	default:
		index = CIF_REG_MIPI_LVDS_FRAME1_ADDR_Y_ID0;
		break;
	}

	return index;
}

static enum cif_reg_index get_reg_index_of_frm0_uv_addr(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_MIPI_LVDS_FRAME0_ADDR_UV_ID0;
		break;
	case 1:
		index = CIF_REG_MIPI_LVDS_FRAME0_ADDR_UV_ID1;
		break;
	case 2:
		index = CIF_REG_MIPI_LVDS_FRAME0_ADDR_UV_ID2;
		break;
	case 3:
		index = CIF_REG_MIPI_LVDS_FRAME0_ADDR_UV_ID3;
		break;
	default:
		index = CIF_REG_MIPI_LVDS_FRAME0_ADDR_UV_ID0;
		break;
	}

	return index;
}

static enum cif_reg_index get_reg_index_of_frm1_uv_addr(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_MIPI_LVDS_FRAME1_ADDR_UV_ID0;
		break;
	case 1:
		index = CIF_REG_MIPI_LVDS_FRAME1_ADDR_UV_ID1;
		break;
	case 2:
		index = CIF_REG_MIPI_LVDS_FRAME1_ADDR_UV_ID2;
		break;
	case 3:
		index = CIF_REG_MIPI_LVDS_FRAME1_ADDR_UV_ID3;
		break;
	default:
		index = CIF_REG_MIPI_LVDS_FRAME1_ADDR_UV_ID0;
		break;
	}

	return index;
}

static enum cif_reg_index get_reg_index_of_frm0_y_vlw(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_MIPI_LVDS_FRAME0_VLW_Y_ID0;
		break;
	case 1:
		index = CIF_REG_MIPI_LVDS_FRAME0_VLW_Y_ID1;
		break;
	case 2:
		index = CIF_REG_MIPI_LVDS_FRAME0_VLW_Y_ID2;
		break;
	case 3:
		index = CIF_REG_MIPI_LVDS_FRAME0_VLW_Y_ID3;
		break;
	default:
		index = CIF_REG_MIPI_LVDS_FRAME0_VLW_Y_ID0;
		break;
	}

	return index;
}

static enum cif_reg_index get_reg_index_of_frm1_y_vlw(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_MIPI_LVDS_FRAME1_VLW_Y_ID0;
		break;
	case 1:
		index = CIF_REG_MIPI_LVDS_FRAME1_VLW_Y_ID1;
		break;
	case 2:
		index = CIF_REG_MIPI_LVDS_FRAME1_VLW_Y_ID2;
		break;
	case 3:
		index = CIF_REG_MIPI_LVDS_FRAME1_VLW_Y_ID3;
		break;
	default:
		index = CIF_REG_MIPI_LVDS_FRAME1_VLW_Y_ID0;
		break;
	}

	return index;
}

static enum cif_reg_index get_reg_index_of_frm0_uv_vlw(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_MIPI_LVDS_FRAME0_VLW_UV_ID0;
		break;
	case 1:
		index = CIF_REG_MIPI_LVDS_FRAME0_VLW_UV_ID1;
		break;
	case 2:
		index = CIF_REG_MIPI_LVDS_FRAME0_VLW_UV_ID2;
		break;
	case 3:
		index = CIF_REG_MIPI_LVDS_FRAME0_VLW_UV_ID3;
		break;
	default:
		index = CIF_REG_MIPI_LVDS_FRAME0_VLW_UV_ID0;
		break;
	}

	return index;
}

static enum cif_reg_index get_reg_index_of_frm1_uv_vlw(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_MIPI_LVDS_FRAME1_VLW_UV_ID0;
		break;
	case 1:
		index = CIF_REG_MIPI_LVDS_FRAME1_VLW_UV_ID1;
		break;
	case 2:
		index = CIF_REG_MIPI_LVDS_FRAME1_VLW_UV_ID2;
		break;
	case 3:
		index = CIF_REG_MIPI_LVDS_FRAME1_VLW_UV_ID3;
		break;
	default:
		index = CIF_REG_MIPI_LVDS_FRAME1_VLW_UV_ID0;
		break;
	}

	return index;
}

static enum cif_reg_index get_reg_index_of_id_crop_start(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_MIPI_LVDS_ID0_CROP_START;
		break;
	case 1:
		index = CIF_REG_MIPI_LVDS_ID1_CROP_START;
		break;
	case 2:
		index = CIF_REG_MIPI_LVDS_ID2_CROP_START;
		break;
	case 3:
		index = CIF_REG_MIPI_LVDS_ID3_CROP_START;
		break;
	default:
		index = CIF_REG_MIPI_LVDS_ID0_CROP_START;
		break;
	}

	return index;
}

static enum cif_reg_index get_reg_index_of_lvds_sav_eav_act0(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_LVDS_SAV_EAV_ACT0_ID0;
		break;
	case 1:
		index = CIF_REG_LVDS_SAV_EAV_ACT0_ID1;
		break;
	case 2:
		index = CIF_REG_LVDS_SAV_EAV_ACT0_ID2;
		break;
	case 3:
		index = CIF_REG_LVDS_SAV_EAV_ACT0_ID3;
		break;
	default:
		index = CIF_REG_LVDS_SAV_EAV_ACT0_ID0;
		break;
	}

	return index;
}

static enum cif_reg_index get_reg_index_of_lvds_sav_eav_act1(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_LVDS_SAV_EAV_ACT1_ID0;
		break;
	case 1:
		index = CIF_REG_LVDS_SAV_EAV_ACT1_ID1;
		break;
	case 2:
		index = CIF_REG_LVDS_SAV_EAV_ACT1_ID2;
		break;
	case 3:
		index = CIF_REG_LVDS_SAV_EAV_ACT1_ID3;
		break;
	default:
		index = CIF_REG_LVDS_SAV_EAV_ACT1_ID0;
		break;
	}

	return index;
}

static enum cif_reg_index get_reg_index_of_lvds_sav_eav_blk0(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_LVDS_SAV_EAV_BLK0_ID0;
		break;
	case 1:
		index = CIF_REG_LVDS_SAV_EAV_BLK0_ID1;
		break;
	case 2:
		index = CIF_REG_LVDS_SAV_EAV_BLK0_ID2;
		break;
	case 3:
		index = CIF_REG_LVDS_SAV_EAV_BLK0_ID3;
		break;
	default:
		index = CIF_REG_LVDS_SAV_EAV_BLK0_ID0;
		break;
	}

	return index;
}

static enum cif_reg_index get_reg_index_of_lvds_sav_eav_blk1(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_LVDS_SAV_EAV_BLK1_ID0;
		break;
	case 1:
		index = CIF_REG_LVDS_SAV_EAV_BLK1_ID1;
		break;
	case 2:
		index = CIF_REG_LVDS_SAV_EAV_BLK1_ID2;
		break;
	case 3:
		index = CIF_REG_LVDS_SAV_EAV_BLK1_ID3;
		break;
	default:
		index = CIF_REG_LVDS_SAV_EAV_BLK1_ID0;
		break;
	}

	return index;
}

static enum cif_reg_index get_dvp_reg_index_of_frm0_y_addr(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_DVP_FRM0_ADDR_Y;
		break;
	case 1:
		index = CIF_REG_DVP_FRM0_ADDR_Y_ID1;
		break;
	case 2:
		index = CIF_REG_DVP_FRM0_ADDR_Y_ID2;
		break;
	case 3:
		index = CIF_REG_DVP_FRM0_ADDR_Y_ID3;
		break;
	default:
		index = CIF_REG_DVP_FRM0_ADDR_Y;
		break;
	}

	return index;
}

static enum cif_reg_index get_dvp_reg_index_of_frm1_y_addr(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_DVP_FRM1_ADDR_Y;
		break;
	case 1:
		index = CIF_REG_DVP_FRM1_ADDR_Y_ID1;
		break;
	case 2:
		index = CIF_REG_DVP_FRM1_ADDR_Y_ID2;
		break;
	case 3:
		index = CIF_REG_DVP_FRM1_ADDR_Y_ID3;
		break;
	default:
		index = CIF_REG_DVP_FRM0_ADDR_Y;
		break;
	}

	return index;
}

static enum cif_reg_index get_dvp_reg_index_of_frm0_uv_addr(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_DVP_FRM0_ADDR_UV;
		break;
	case 1:
		index = CIF_REG_DVP_FRM0_ADDR_UV_ID1;
		break;
	case 2:
		index = CIF_REG_DVP_FRM0_ADDR_UV_ID2;
		break;
	case 3:
		index = CIF_REG_DVP_FRM0_ADDR_UV_ID3;
		break;
	default:
		index = CIF_REG_DVP_FRM0_ADDR_UV;
		break;
	}

	return index;
}

static enum cif_reg_index get_dvp_reg_index_of_frm1_uv_addr(int channel_id)
{
	enum cif_reg_index index;

	switch (channel_id) {
	case 0:
		index = CIF_REG_DVP_FRM1_ADDR_UV;
		break;
	case 1:
		index = CIF_REG_DVP_FRM1_ADDR_UV_ID1;
		break;
	case 2:
		index = CIF_REG_DVP_FRM1_ADDR_UV_ID2;
		break;
	case 3:
		index = CIF_REG_DVP_FRM1_ADDR_UV_ID3;
		break;
	default:
		index = CIF_REG_DVP_FRM1_ADDR_UV;
		break;
	}

	return index;
}

static void rkcif_enable_capture(struct rkcif_stream *stream)
{
	rkcif_write_register_or(stream->cifdev,
				get_reg_index_of_id_ctrl0(stream->id),
				ENABLE_CAPTURE);
}

static void rkcif_disable_capture(struct rkcif_stream *stream)
{
	rkcif_write_register_and(stream->cifdev,
				 get_reg_index_of_id_ctrl0(stream->id),
				 ~ENABLE_CAPTURE);
}

int rkcif_get_linetime(struct rkcif_stream *stream)
{
	struct rkcif_device *cif_dev = stream->cifdev;
	struct rkcif_sensor_info *sensor = &cif_dev->terminal_sensor;
	u32 numerator, denominator;
	u32 def_fps = 0;
	int line_time = 0;
	int vblank_def = 0;
	int vblank_curr = 0;

	numerator = sensor->src_fi.interval.numerator;
	denominator = sensor->src_fi.interval.denominator;
	if (!numerator || !denominator) {
		v4l2_err(&cif_dev->v4l2_dev,
			 "get frame interval fail, numerator %d, denominator %d\n",
			 numerator, denominator);
		return -EINVAL;
	}
	def_fps = denominator / numerator;
	if (!def_fps) {
		v4l2_err(&cif_dev->v4l2_dev,
			 "get fps fail, numerator %d, denominator %d\n",
			 numerator, denominator);
		return -EINVAL;
	}
	vblank_def = rkcif_get_sensor_vblank_def(cif_dev);
	vblank_curr = rkcif_get_sensor_vblank(cif_dev);
	if (!vblank_def || !vblank_curr) {
		v4l2_err(&cif_dev->v4l2_dev,
			 "get vblank fail, vblank_def %d, vblank_curr %d\n",
			 vblank_def, vblank_curr);
		return -EINVAL;
	}
	line_time = div_u64(1000000000, def_fps);
	line_time = div_u64(line_time, vblank_def + sensor->raw_rect.height);
	v4l2_dbg(3, rkcif_debug, &cif_dev->v4l2_dev,
		 "line_time %d, numerator %d, denominator %d, vblank_def %d\n",
		 line_time, numerator, denominator, vblank_def);
	return line_time;
}

/***************************** stream operations ******************************/
static int rkcif_assign_new_buffer_oneframe(struct rkcif_stream *stream,
					    enum rkcif_yuvaddr_state stat)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_dummy_buffer *dummy_buf = &dev->hw_dev->dummy_buf;
	struct rkcif_buffer *buffer = NULL;
	u32 frm_addr_y = CIF_REG_DVP_FRM0_ADDR_Y;
	u32 frm_addr_uv = CIF_REG_DVP_FRM0_ADDR_UV;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&stream->vbq_lock, flags);
	if (stat == RKCIF_YUV_ADDR_STATE_INIT) {
		if (!stream->curr_buf) {
			if (!list_empty(&stream->buf_head)) {
				stream->curr_buf = list_first_entry(&stream->buf_head,
								    struct rkcif_buffer,
								    queue);
				list_del(&stream->curr_buf->queue);
			}
		}

		if (stream->curr_buf) {
			rkcif_write_register(dev, CIF_REG_DVP_FRM0_ADDR_Y,
					     stream->curr_buf->buff_addr[RKCIF_PLANE_Y]);
			rkcif_write_register(dev, CIF_REG_DVP_FRM0_ADDR_UV,
					     stream->curr_buf->buff_addr[RKCIF_PLANE_CBCR]);
		} else {
			if (dummy_buf->vaddr) {
				rkcif_write_register(dev, CIF_REG_DVP_FRM0_ADDR_Y,
						     dummy_buf->dma_addr);
				rkcif_write_register(dev, CIF_REG_DVP_FRM0_ADDR_UV,
						     dummy_buf->dma_addr);
			}
		}

		if (!stream->next_buf) {
			if (!list_empty(&stream->buf_head)) {
				stream->next_buf = list_first_entry(&stream->buf_head,
								    struct rkcif_buffer, queue);
				list_del(&stream->next_buf->queue);
			}
		}

		if (stream->next_buf) {
			rkcif_write_register(dev, CIF_REG_DVP_FRM1_ADDR_Y,
					     stream->next_buf->buff_addr[RKCIF_PLANE_Y]);
			rkcif_write_register(dev, CIF_REG_DVP_FRM1_ADDR_UV,
					     stream->next_buf->buff_addr[RKCIF_PLANE_CBCR]);
		} else {
			if (dummy_buf->vaddr) {
				rkcif_write_register(dev, CIF_REG_DVP_FRM1_ADDR_Y,
						     dummy_buf->dma_addr);
				rkcif_write_register(dev, CIF_REG_DVP_FRM1_ADDR_UV,
						     dummy_buf->dma_addr);
			}
		}
	} else if (stat == RKCIF_YUV_ADDR_STATE_UPDATE) {
		if (!list_empty(&stream->buf_head)) {
			if (stream->frame_phase == CIF_CSI_FRAME0_READY) {
				stream->curr_buf = list_first_entry(&stream->buf_head,
								    struct rkcif_buffer, queue);
				list_del(&stream->curr_buf->queue);
				buffer = stream->curr_buf;
			} else if (stream->frame_phase == CIF_CSI_FRAME1_READY) {
				stream->next_buf = list_first_entry(&stream->buf_head,
								    struct rkcif_buffer, queue);
				list_del(&stream->next_buf->queue);
				buffer = stream->next_buf;
			}
		} else {
			if (dummy_buf->vaddr && stream->frame_phase == CIF_CSI_FRAME0_READY)
				stream->curr_buf = NULL;
			if (dummy_buf->vaddr && stream->frame_phase == CIF_CSI_FRAME1_READY)
				stream->next_buf = NULL;
			buffer = NULL;
		}
		if (stream->frame_phase == CIF_CSI_FRAME0_READY) {
			frm_addr_y = CIF_REG_DVP_FRM0_ADDR_Y;
			frm_addr_uv = CIF_REG_DVP_FRM0_ADDR_UV;
		} else if (stream->frame_phase == CIF_CSI_FRAME1_READY) {
			frm_addr_y = CIF_REG_DVP_FRM1_ADDR_Y;
			frm_addr_uv = CIF_REG_DVP_FRM1_ADDR_UV;
		}

		if (buffer) {
			rkcif_write_register(dev, frm_addr_y,
					     buffer->buff_addr[RKCIF_PLANE_Y]);
			rkcif_write_register(dev, frm_addr_uv,
					     buffer->buff_addr[RKCIF_PLANE_CBCR]);
		} else {
			if (dummy_buf->vaddr) {
				rkcif_write_register(dev, frm_addr_y,
					     dummy_buf->dma_addr);
				rkcif_write_register(dev, frm_addr_uv,
					     dummy_buf->dma_addr);
			} else {
				ret = -EINVAL;
			}
			dev->err_state |= (RKCIF_ERR_ID0_NOT_BUF << stream->id);
			dev->irq_stats.not_active_buf_cnt[stream->id]++;
		}
	}
	spin_unlock_irqrestore(&stream->vbq_lock, flags);
	return ret;
}

static struct v4l2_subdev *get_rkisp_sd(struct sditf_priv *priv)
{
	struct media_pad *pad = NULL;

	if (priv && priv->pads[0].entity->num_links) {
		if (priv->is_combine_mode || priv->cif_dev->is_camera_over_bridge)
			pad = media_pad_remote_pad_first(&priv->pads[1]);
		else
			pad = media_pad_remote_pad_first(&priv->pads[0]);
		if (pad)
			return media_entity_to_v4l2_subdev(pad->entity);
	}
	return NULL;
}

static void rkcif_rx_buffer_free(struct rkcif_stream *stream)
{
	struct v4l2_subdev *sd;
	struct rkisp_rx_buf *dbufs;
	struct rkcif_device *dev = stream->cifdev;

	sd = get_rkisp_sd(dev->sditf[0]);
	if (!sd)
		return;

	while (!list_empty(&stream->rx_buf_head_vicap)) {
		dbufs = list_first_entry(&stream->rx_buf_head_vicap, struct rkisp_rx_buf, list);
		if (dbufs->is_init)
			v4l2_subdev_call(sd, core, ioctl,
					 RKISP_VICAP_CMD_RX_BUFFER_FREE, dbufs);
		dma_buf_put(dbufs->dbuf);
		list_del(&dbufs->list);
		kfree(dbufs);
	}
}

static void rkcif_s_rx_buffer(struct rkcif_stream *stream, struct rkisp_rx_buf *dbufs)
{
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_subdev *sd;
	struct rkcif_rx_buffer *rx_buf = NULL;

	sd = get_rkisp_sd(dev->sditf[0]);
	if (!sd)
		return;
	rx_buf = to_cif_rx_buf(dbufs);
	stream->last_buf_toisp = rx_buf;
	if (dev->switch_info.is_use_switch) {
		if (!rx_buf->is_init[dev->csi_host_idx_def]) {
			dbufs->is_init = false;
			rx_buf->is_init[dev->csi_host_idx_def] = true;
		}
	}
	if ((dev->rdbk_debug &&
	     dbufs->sequence < 15) ||
	    rkcif_debug == 3)
		v4l2_info(&dev->v4l2_dev,
			  "s_buf seq %d type %d, dma addr %x, %lld\n",
			  dbufs->sequence, dbufs->type, (u32)rx_buf->dummy.dma_addr,
			  rkcif_time_get_ns(dev));
	v4l2_subdev_call(sd, video, s_rx_buffer, dbufs, NULL);
}

static void rkcif_enable_skip_frame_rv1126b(struct rkcif_stream *stream, int cap_m, int skip_n)
{
	struct rkcif_device *dev = stream->cifdev;
	u32 val = 0;

	val = rkcif_read_register(dev, get_reg_index_of_id_ctrl0(stream->id));
	val &= (~SKIP_FRM_MASK_RV1126B);
	val |= (skip_n << SKIP_FRM_OFFSET_RV1126B);
	rkcif_write_register(dev, get_reg_index_of_id_ctrl0(stream->id), val);
	stream->skip_info.skip_en = true;
}

static void rkcif_enable_skip_frame(struct rkcif_stream *stream, int cap_m, int skip_n)
{
	struct rkcif_device *dev = stream->cifdev;
	u32 val = 0;
	u32 skip_mask = 0;

	val = rkcif_read_register(dev, CIF_REG_MIPI_LVDS_CTRL);
	if (dev->chip_id > CHIP_RK3562_CIF) {
		val &= 0xffffc0ff;
		skip_mask = (cap_m << RKCIF_CAP_SHIFT_RK3576) | (skip_n << RKCIF_SKIP_SHIFT_RK3576) |
			RKCIF_SKIP_EN_TOTAL_RK3576;
		val |= skip_mask;
	} else {
		val &= 0xffff00ff;
		val |= cap_m << RKCIF_CAP_SHIFT | skip_n << RKCIF_SKIP_SHIFT | RKCIF_SKIP_EN(stream->id);
	}
	rkcif_write_register(dev, CIF_REG_MIPI_LVDS_CTRL, val);
	if (dev->chip_id > CHIP_RK3562_CIF)
		rkcif_write_register_or(dev, get_reg_index_of_id_ctrl0(stream->id), RKCIF_SKIP_EN_RK3576);
	stream->skip_info.skip_en = true;
}

static void rkcif_disable_skip_frame(struct rkcif_stream *stream)
{	struct rkcif_device *dev = stream->cifdev;
	u32 val = 0;

	if (dev->chip_id >= CHIP_RV1126B_CIF) {
		rkcif_write_register_and(dev, get_reg_index_of_id_ctrl0(stream->id),
					 ~SKIP_FRM_MASK_RV1126B);
	} else if (dev->chip_id > CHIP_RK3562_CIF) {
		rkcif_write_register_and(dev, get_reg_index_of_id_ctrl0(stream->id),
					 ~RKCIF_SKIP_EN_RK3576);
	} else {
		val = rkcif_read_register(dev, CIF_REG_MIPI_LVDS_CTRL);
		val &= ~(RKCIF_SKIP_EN(stream->id));
		rkcif_write_register(dev, CIF_REG_MIPI_LVDS_CTRL, val);
	}
	stream->skip_info.skip_en = false;
}

static void rkcif_rdbk_with_tools(struct rkcif_stream *stream,
				      struct rkcif_rx_buffer *active_buf)
{
	unsigned long flags;

	spin_lock_irqsave(&stream->tools_vdev->vbq_lock, flags);
	if (stream->tools_vdev->state == RKCIF_STATE_STREAMING && active_buf) {
		list_add_tail(&active_buf->list_tool, &stream->tools_vdev->buf_done_head);
		active_buf->use_cnt = 2;
		if (!work_busy(&stream->tools_vdev->work))
			schedule_work(&stream->tools_vdev->work);
	}
	spin_unlock_irqrestore(&stream->tools_vdev->vbq_lock, flags);
}

static void rkcif_rdbk_frame_end_toisp(struct rkcif_stream *stream,
				       struct rkcif_rx_buffer *buffer)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_sensor_info *sensor = &stream->cifdev->terminal_sensor;
	u32 denominator, numerator;
	u64 l_ts, m_ts, s_ts, time = 30000000LL;
	int ret, fps = -1;
	unsigned long flags;

	spin_lock_irqsave(&dev->hdr_lock, flags);
	if (dev->rdbk_rx_buf[stream->id]) {
		list_add_tail(&dev->rdbk_rx_buf[stream->id]->list, &stream->rx_buf_head);
		dev->rdbk_rx_buf[stream->id] = buffer;
	} else {
		dev->rdbk_rx_buf[stream->id] = buffer;
	}

	numerator = sensor->fi.interval.numerator;
	denominator = sensor->fi.interval.denominator;
	if (denominator && numerator)
		time = numerator * 1000 / denominator * 1000 * 1000;

	if (dev->hdr.hdr_mode == HDR_X3 &&
	    dev->rdbk_rx_buf[RDBK_L] &&
	    dev->rdbk_rx_buf[RDBK_M] &&
	    dev->rdbk_rx_buf[RDBK_S]) {
		l_ts = dev->rdbk_rx_buf[RDBK_L]->fe_timestamp;
		m_ts = dev->rdbk_rx_buf[RDBK_M]->fe_timestamp;
		s_ts = dev->rdbk_rx_buf[RDBK_S]->fe_timestamp;

		if (m_ts < l_ts || s_ts < m_ts) {
			v4l2_err(&dev->v4l2_dev,
				 "s/m/l frame err, timestamp s:%lld m:%lld l:%lld\n",
				 s_ts, m_ts, l_ts);
			goto RDBK_TOISP_UNMATCH;
		}

		if ((m_ts - l_ts) > time || (s_ts - m_ts) > time) {
			ret = v4l2_subdev_call(sensor->sd,
					       video,
					       g_frame_interval,
					       &sensor->fi);
			if (!ret) {
				denominator = sensor->fi.interval.denominator;
				numerator = sensor->fi.interval.numerator;
				if (denominator && numerator) {
					time = numerator * 1000 / denominator * 1000 * 1000;
					fps = denominator / numerator;
				}
			}

			if ((m_ts - l_ts) > time || (s_ts - m_ts) > time) {
				v4l2_err(&dev->v4l2_dev,
					 "timestamp no match, s:%lld m:%lld l:%lld, fps:%d\n",
					 s_ts, m_ts, l_ts, fps);
				goto RDBK_TOISP_UNMATCH;
			}
		}
		dev->rdbk_rx_buf[RDBK_M]->dbufs.sequence = dev->rdbk_rx_buf[RDBK_L]->dbufs.sequence;
		dev->rdbk_rx_buf[RDBK_S]->dbufs.sequence = dev->rdbk_rx_buf[RDBK_L]->dbufs.sequence;
		rkcif_s_rx_buffer(&dev->stream[RDBK_L], &dev->rdbk_rx_buf[RDBK_L]->dbufs);
		rkcif_s_rx_buffer(&dev->stream[RDBK_M], &dev->rdbk_rx_buf[RDBK_M]->dbufs);
		rkcif_s_rx_buffer(&dev->stream[RDBK_S], &dev->rdbk_rx_buf[RDBK_S]->dbufs);
		rkcif_rdbk_with_tools(&dev->stream[RDBK_L], dev->rdbk_rx_buf[RDBK_L]);
		rkcif_rdbk_with_tools(&dev->stream[RDBK_M], dev->rdbk_rx_buf[RDBK_M]);
		rkcif_rdbk_with_tools(&dev->stream[RDBK_S], dev->rdbk_rx_buf[RDBK_S]);
		atomic_dec(&dev->stream[RDBK_L].buf_cnt);
		atomic_dec(&dev->stream[RDBK_M].buf_cnt);
		atomic_dec(&dev->stream[RDBK_S].buf_cnt);
		dev->rdbk_rx_buf[RDBK_L] = NULL;
		dev->rdbk_rx_buf[RDBK_M] = NULL;
		dev->rdbk_rx_buf[RDBK_S] = NULL;
	} else if (dev->hdr.hdr_mode == HDR_X2 &&
		dev->rdbk_rx_buf[RDBK_L] && dev->rdbk_rx_buf[RDBK_M]) {
		l_ts = dev->rdbk_rx_buf[RDBK_L]->fe_timestamp;
		s_ts = dev->rdbk_rx_buf[RDBK_M]->fe_timestamp;

		if (s_ts < l_ts) {
			v4l2_err(&dev->v4l2_dev,
				 "s/l frame err, timestamp s:%lld l:%lld\n",
				 s_ts, l_ts);
			goto RDBK_TOISP_UNMATCH;
		}

		if ((s_ts - l_ts) > time) {
			ret = v4l2_subdev_call(sensor->sd,
					       video,
					       g_frame_interval,
					       &sensor->fi);
			if (!ret) {
				denominator = sensor->fi.interval.denominator;
				numerator = sensor->fi.interval.numerator;
				if (denominator && numerator) {
					time = numerator * 1000 / denominator * 1000 * 1000;
					fps = denominator / numerator;
				}
			}
			if ((s_ts - l_ts) > time) {
				v4l2_err(&dev->v4l2_dev,
					 "timestamp no match, s:%lld l:%lld, fps:%d\n",
					 s_ts, l_ts, fps);
				goto RDBK_TOISP_UNMATCH;
			}
		}
		dev->rdbk_rx_buf[RDBK_M]->dbufs.sequence = dev->rdbk_rx_buf[RDBK_L]->dbufs.sequence;
		rkcif_s_rx_buffer(&dev->stream[RDBK_L], &dev->rdbk_rx_buf[RDBK_L]->dbufs);
		rkcif_s_rx_buffer(&dev->stream[RDBK_M], &dev->rdbk_rx_buf[RDBK_M]->dbufs);
		rkcif_rdbk_with_tools(&dev->stream[RDBK_L], dev->rdbk_rx_buf[RDBK_L]);
		rkcif_rdbk_with_tools(&dev->stream[RDBK_M], dev->rdbk_rx_buf[RDBK_M]);
		atomic_dec(&dev->stream[RDBK_L].buf_cnt);
		atomic_dec(&dev->stream[RDBK_M].buf_cnt);
		dev->rdbk_rx_buf[RDBK_L] = NULL;
		dev->rdbk_rx_buf[RDBK_M] = NULL;
	}

	spin_unlock_irqrestore(&dev->hdr_lock, flags);
	return;

RDBK_TOISP_UNMATCH:
	spin_unlock_irqrestore(&dev->hdr_lock, flags);
}

static void rkcif_get_other_dev_update_frame_addr(struct rkcif_stream *stream,
						  int dev_idx,
						  u32 *frm_addr_y,
						  u32 *frm_addr_uv)
{
	struct v4l2_mbus_config *mbus_cfg = &stream->cifdev->active_sensor->mbus;
	u32 intstat = stream->cifdev->other_intstat[dev_idx];
	int frame_phase = 0;

	switch (stream->id) {
	case RKCIF_STREAM_MIPI_ID0:
		frame_phase = SW_FRM_END_ID0(intstat);
		break;
	case RKCIF_STREAM_MIPI_ID1:
		frame_phase = SW_FRM_END_ID1(intstat);
		break;
	case RKCIF_STREAM_MIPI_ID2:
		frame_phase = SW_FRM_END_ID2(intstat);
		break;
	case RKCIF_STREAM_MIPI_ID3:
		frame_phase = SW_FRM_END_ID3(intstat);
		break;
	}

	if (frame_phase == 0)
		return;

	if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY ||
	    mbus_cfg->type == V4L2_MBUS_CCP2) {
		*frm_addr_y = frame_phase & CIF_CSI_FRAME0_READY ?
			      get_reg_index_of_frm0_y_addr(stream->id) :
			      get_reg_index_of_frm1_y_addr(stream->id);
		*frm_addr_uv = frame_phase & CIF_CSI_FRAME0_READY ?
			       get_reg_index_of_frm0_uv_addr(stream->id) :
			       get_reg_index_of_frm1_uv_addr(stream->id);
	} else {
		*frm_addr_y = frame_phase & CIF_CSI_FRAME0_READY ?
			      get_dvp_reg_index_of_frm0_y_addr(stream->id) :
			      get_dvp_reg_index_of_frm1_y_addr(stream->id);
		*frm_addr_uv = frame_phase & CIF_CSI_FRAME0_READY ?
			       get_dvp_reg_index_of_frm0_uv_addr(stream->id) :
			       get_dvp_reg_index_of_frm1_uv_addr(stream->id);
	}
}

static void rkcif_write_buff_addr_multi_dev_combine(struct rkcif_stream *stream,
						    u32 frm_addr_y, u32 frm_addr_uv,
						    u32 buff_addr_y, u32 buff_addr_cbcr,
						    bool is_dummy_buf)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkmodule_capture_info *capture_info = &dev->channels[stream->id].capture_info;
	u32 addr_y, addr_cbcr;
	int addr_offset = 0;
	int i = 0;
	int tmp_host_index = dev->csi_host_idx;
	u32 tmp_frm_addr_y, tmp_frm_addr_uv;

	for (i = 0; i < capture_info->multi_dev.dev_num; i++) {
		tmp_frm_addr_y = frm_addr_y;
		tmp_frm_addr_uv = frm_addr_uv;
		if (is_dummy_buf) {
			addr_y = buff_addr_y;
		} else {
			addr_offset = dev->channels[stream->id].left_virtual_width;
			addr_y = buff_addr_y + addr_offset * i;
		}
		dev->csi_host_idx = capture_info->multi_dev.dev_idx[i];
		if (i < capture_info->multi_dev.dev_num - 1)
			rkcif_get_other_dev_update_frame_addr(stream, i,
							      &tmp_frm_addr_y,
							      &tmp_frm_addr_uv);
		rkcif_write_register(dev, tmp_frm_addr_y, addr_y);
		if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW &&
		    frm_addr_uv && buff_addr_cbcr) {
			if (is_dummy_buf) {
				addr_cbcr = buff_addr_cbcr;
			} else {
				addr_offset = dev->channels[stream->id].left_virtual_width;
				addr_cbcr = buff_addr_cbcr + addr_offset * i;
			}
			rkcif_write_register(dev, tmp_frm_addr_uv, addr_cbcr);
		}
	}
	dev->csi_host_idx = tmp_host_index;
}

static void rkcif_assign_new_buffer_init_toisp(struct rkcif_stream *stream,
					       int channel_id)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_rx_buffer *rx_buf;
	struct v4l2_mbus_config *mbus_cfg = &dev->active_sensor->mbus;
	struct rkmodule_capture_info *capture_info = &dev->channels[channel_id].capture_info;
	struct rkcif_stream *buf_stream = stream;
	u32 frm0_addr_y;
	u32 frm1_addr_y;
	u32 buff_addr_y;
	unsigned long flags;

	if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY ||
	    mbus_cfg->type == V4L2_MBUS_CCP2) {
		frm0_addr_y = get_reg_index_of_frm0_y_addr(channel_id);
		frm1_addr_y = get_reg_index_of_frm1_y_addr(channel_id);
	} else {
		frm0_addr_y = get_dvp_reg_index_of_frm0_y_addr(channel_id);
		frm1_addr_y = get_dvp_reg_index_of_frm1_y_addr(channel_id);
	}

	spin_lock_irqsave(&buf_stream->vbq_lock, flags);
	if (dev->switch_info.is_use_switch &&
	    dev->switch_info.switch_dev->switch_info.is_init_buf)
		buf_stream = &dev->switch_info.switch_dev->stream[stream->id];

	if (!buf_stream->curr_buf_toisp) {
		if (!list_empty(&buf_stream->rx_buf_head)) {
			rx_buf = list_first_entry(&buf_stream->rx_buf_head,
						 struct rkcif_rx_buffer,
						 list);
			if (rx_buf) {
				list_del(&rx_buf->list);
				buf_stream->curr_buf_toisp = rx_buf;
			}
		}
	}

	if (buf_stream->curr_buf_toisp) {
		buff_addr_y = buf_stream->curr_buf_toisp->dummy.dma_addr;
		if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
			rkcif_write_buff_addr_multi_dev_combine(buf_stream, frm0_addr_y, 0,
								buff_addr_y, 0, false);
		} else {
			rkcif_write_register(dev, frm0_addr_y, buff_addr_y);
		}
	} else {
		if (buf_stream->lack_buf_cnt < 2)
			buf_stream->lack_buf_cnt++;
		 buf_stream->toisp_buf_state.state = RKCIF_TOISP_BUF_LOSS;
	}

	if (!buf_stream->next_buf_toisp) {
		if (!list_empty(&buf_stream->rx_buf_head)) {
			rx_buf = list_first_entry(&buf_stream->rx_buf_head,
						 struct rkcif_rx_buffer, list);
			if (rx_buf) {
				list_del(&rx_buf->list);
				buf_stream->next_buf_toisp = rx_buf;
			} else {
				buf_stream->next_buf_toisp = buf_stream->curr_buf_toisp;
			}
		} else if (buf_stream->curr_buf_toisp) {
			buf_stream->next_buf_toisp = buf_stream->curr_buf_toisp;
			buf_stream->toisp_buf_state.state = RKCIF_TOISP_BUF_THESAME;
			if (buf_stream->lack_buf_cnt < 2)
				buf_stream->lack_buf_cnt++;
		}
	}

	if (buf_stream->next_buf_toisp) {
		buff_addr_y = buf_stream->next_buf_toisp->dummy.dma_addr;
		if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
			rkcif_write_buff_addr_multi_dev_combine(buf_stream, frm1_addr_y, 0,
								buff_addr_y, 0, false);
		} else {
			rkcif_write_register(dev, frm1_addr_y, buff_addr_y);
		}
	} else {
		if (buf_stream->lack_buf_cnt < 2)
			buf_stream->lack_buf_cnt++;
	}

	spin_unlock_irqrestore(&buf_stream->vbq_lock, flags);
	buf_stream->buf_owner = RKCIF_DMAEN_BY_ISP;
}

void rkcif_dphy_quick_stream(struct rkcif_device *dev, int on)
{
	struct rkcif_pipeline *p = NULL;
	int j = 0;

	if (dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_DPHY ||
	    dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_CPHY ||
	    dev->active_sensor->mbus.type == V4L2_MBUS_CCP2) {
		p = &dev->pipe;
		for (j = 0; j < p->num_subdevs; j++) {
			if (p->subdevs[j] != dev->terminal_sensor.sd &&
			    p->subdevs[j] != dev->active_sensor->sd) {
				v4l2_subdev_call(p->subdevs[j], core, ioctl,
						 RKMODULE_SET_QUICK_STREAM, &on);
				break;
			}
		}
		v4l2_subdev_call(dev->active_sensor->sd, core, ioctl,
				 RKMODULE_SET_QUICK_STREAM, &on);
	}
}

static int rkcif_assign_new_buffer_update_toisp(struct rkcif_stream *stream,
						int channel_id)
{
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_mbus_config *mbus_cfg = &dev->active_sensor->mbus;
	struct rkmodule_capture_info *capture_info = &dev->channels[channel_id].capture_info;
	struct rkcif_rx_buffer *buffer = NULL;
	struct rkcif_rx_buffer *active_buf = NULL;
	struct sditf_priv *priv = dev->sditf[0];
	struct rkcif_stream *buf_stream = stream;
	u32 frm_addr_y, buff_addr_y;
	unsigned long flags;

	if (dev->switch_info.is_use_switch &&
	    dev->switch_info.switch_dev->switch_info.is_init_buf)
		buf_stream = &dev->switch_info.switch_dev->stream[stream->id];
	spin_lock_irqsave(&buf_stream->vbq_lock, flags);
	if (dev->is_stop_skip) {
		dev->is_stop_skip = false;
		if (((stream->frame_idx - 1) % stream->thunderboot_skip_interval) != 0) {
			stream->thunderboot_skip_interval = 0;
			stream->frame_idx = stream->sequence + 1;
			spin_unlock_irqrestore(&buf_stream->vbq_lock, flags);
			return 0;
		} else {
			stream->thunderboot_skip_interval = 0;
			stream->frame_idx = stream->sequence + 2;
		}
	}
	if (dev->is_thunderboot &&
	    stream->thunderboot_skip_interval &&
	    ((stream->frame_idx - 1) % stream->thunderboot_skip_interval) != 0) {
		spin_unlock_irqrestore(&buf_stream->vbq_lock, flags);
		return 0;
	}
	if (stream->thunderboot_skip_interval) {
		if (stream->frame_idx < stream->thunderboot_skip_interval)
			stream->sequence = stream->frame_idx - 1;
		else
			stream->sequence = stream->frame_idx / stream->thunderboot_skip_interval;
	} else {
		stream->sequence = stream->frame_idx - 1;
	}
	if (dev->rdbk_debug &&
	    stream->frame_idx < 15)
		v4l2_info(&dev->v4l2_dev,
			  "stream[%d] done seq %d, real seq %d\n",
			  stream->id,
			  stream->sequence,
			  stream->frame_idx - 1);
	spin_unlock_irqrestore(&buf_stream->vbq_lock, flags);

	if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY ||
	    mbus_cfg->type == V4L2_MBUS_CCP2) {
		frm_addr_y = stream->frame_phase & CIF_CSI_FRAME0_READY ?
			     get_reg_index_of_frm0_y_addr(channel_id) :
			     get_reg_index_of_frm1_y_addr(channel_id);
	} else {
		frm_addr_y = stream->frame_phase & CIF_CSI_FRAME0_READY ?
			     get_dvp_reg_index_of_frm0_y_addr(channel_id) :
			     get_dvp_reg_index_of_frm1_y_addr(channel_id);
	}

	spin_lock_irqsave(&stream->vbq_lock, flags);
	if (stream->cur_skip_frame)
		goto out_get_buf;
	memset(&buf_stream->toisp_buf_state, 0, sizeof(stream->toisp_buf_state));
	if (!list_empty(&buf_stream->rx_buf_head)) {
		if (buf_stream->curr_buf_toisp && buf_stream->next_buf_toisp &&
		    buf_stream->curr_buf_toisp != buf_stream->next_buf_toisp)
			buf_stream->toisp_buf_state.state = RKCIF_TOISP_BUF_ROTATE;
		else
			buf_stream->toisp_buf_state.state = RKCIF_TOISP_BUF_LOSS;
		if (stream->frame_phase == CIF_CSI_FRAME0_READY) {
			active_buf = buf_stream->curr_buf_toisp;

			buffer = list_first_entry(&buf_stream->rx_buf_head,
						 struct rkcif_rx_buffer, list);
			if (buffer) {
				list_del(&buffer->list);
				buf_stream->curr_buf_toisp = buffer;
			}
			if (priv && (priv->mode.rdbk_mode == RKISP_VICAP_RDBK_AUTO ||
			    priv->mode.rdbk_mode == RKISP_VICAP_RDBK_AUTO_ONE_FRAME)) {
				if (!active_buf)
					goto out_get_buf;
				if (stream->is_fb_first_frame) {
					stream->sequence = 0;
					active_buf->dbufs.is_first = true;
					stream->is_fb_first_frame = false;
					if (stream->frame_idx != 1)
						v4l2_info(&dev->v4l2_dev,
							  "stream[%d],the first frame may be incomplete, fs cnt %d\n",
							  stream->id, stream->frame_idx);
				}
				active_buf->dbufs.sequence = stream->sequence;
				active_buf->dbufs.timestamp = stream->readout.fs_timestamp;
				active_buf->fe_timestamp = rkcif_time_get_ns(dev);
				stream->last_frame_idx = stream->frame_idx;
				if (dev->hdr.hdr_mode == NO_HDR) {
					rkcif_s_rx_buffer(stream, &active_buf->dbufs);
					if (dev->is_support_tools && stream->tools_vdev)
						rkcif_rdbk_with_tools(stream, active_buf);
					atomic_dec(&stream->buf_cnt);
				} else {
					rkcif_rdbk_frame_end_toisp(stream, active_buf);
				}
			} else {
				if (active_buf)
					rkcif_s_rx_buffer(stream, &active_buf->dbufs);
				if (dev->is_support_tools && stream->tools_vdev)
					rkcif_rdbk_with_tools(stream, active_buf);
			}
		} else if (stream->frame_phase == CIF_CSI_FRAME1_READY) {
			active_buf = buf_stream->next_buf_toisp;
			buffer = list_first_entry(&buf_stream->rx_buf_head,
						 struct rkcif_rx_buffer, list);
			if (buffer) {
				list_del(&buffer->list);
				buf_stream->next_buf_toisp = buffer;
			}
			if (priv && (priv->mode.rdbk_mode == RKISP_VICAP_RDBK_AUTO ||
			    priv->mode.rdbk_mode == RKISP_VICAP_RDBK_AUTO_ONE_FRAME)) {
				if (!active_buf)
					goto out_get_buf;
				if (stream->is_fb_first_frame) {
					stream->sequence = 0;
					active_buf->dbufs.is_first = true;
					stream->is_fb_first_frame = false;
					if (stream->frame_idx != 1)
						v4l2_info(&dev->v4l2_dev,
							  "stream[%d],the first frame may be incomplete, fs cnt %d\n",
							  stream->id, stream->frame_idx);
				}
				active_buf->dbufs.sequence = stream->sequence;
				active_buf->dbufs.timestamp = stream->readout.fs_timestamp;
				active_buf->fe_timestamp = rkcif_time_get_ns(dev);
				stream->last_frame_idx = stream->frame_idx;
				if (dev->hdr.hdr_mode == NO_HDR) {
					rkcif_s_rx_buffer(stream, &active_buf->dbufs);
					if (dev->is_support_tools && stream->tools_vdev)
						rkcif_rdbk_with_tools(stream, active_buf);
					atomic_dec(&stream->buf_cnt);
				} else {
					rkcif_rdbk_frame_end_toisp(stream, active_buf);
				}
			} else {
				if (active_buf)
					rkcif_s_rx_buffer(stream, &active_buf->dbufs);
				if (dev->is_support_tools && stream->tools_vdev)
					rkcif_rdbk_with_tools(stream, active_buf);
			}
		}
		if (buf_stream->lack_buf_cnt)
			buf_stream->lack_buf_cnt--;
	} else {
		if (priv->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ)
			goto out_get_buf;
		if (buf_stream->lack_buf_cnt < 2)
			buf_stream->lack_buf_cnt++;
		if (dev->hw_dev->dummy_buf.vaddr) {
			if (stream->frame_phase == CIF_CSI_FRAME0_READY) {
				active_buf = buf_stream->curr_buf_toisp;
			} else {
				active_buf = buf_stream->next_buf_toisp;
			}
		} else if (buf_stream->curr_buf_toisp && buf_stream->next_buf_toisp &&
			   buf_stream->curr_buf_toisp != buf_stream->next_buf_toisp) {
			if (stream->frame_phase == CIF_CSI_FRAME0_READY) {
				active_buf = buf_stream->curr_buf_toisp;
				buf_stream->curr_buf_toisp = buf_stream->next_buf_toisp;
				buffer = buf_stream->next_buf_toisp;
			} else if (stream->frame_phase == CIF_CSI_FRAME1_READY) {
				active_buf = buf_stream->next_buf_toisp;
				buf_stream->next_buf_toisp = buf_stream->curr_buf_toisp;
				buffer = buf_stream->curr_buf_toisp;
			}
			buf_stream->toisp_buf_state.state = RKCIF_TOISP_BUF_THESAME;
			if (stream->cifdev->rdbk_debug)
				v4l2_info(&stream->cifdev->v4l2_dev,
					  "stream[%d] hold buf %x\n",
					  stream->id,
					  (u32)buf_stream->next_buf_toisp->dummy.dma_addr);
		} else {
			buf_stream->toisp_buf_state.state = RKCIF_TOISP_BUF_LOSS;

			active_buf = buf_stream->curr_buf_toisp;
			buf_stream->curr_buf_toisp = NULL;
			buf_stream->next_buf_toisp = NULL;

		}

		if (active_buf) {
			if (stream->is_fb_first_frame) {
				stream->sequence = 0;
				active_buf->dbufs.is_first = true;
				stream->is_fb_first_frame = false;
				if (stream->frame_idx != 1)
					v4l2_info(&dev->v4l2_dev,
						  "stream[%d],the first frame may be incomplete, fs cnt %d\n",
						  stream->id, stream->frame_idx);
			}
			active_buf->dbufs.sequence = stream->sequence;
			active_buf->dbufs.timestamp = stream->readout.fs_timestamp;
			active_buf->fe_timestamp = rkcif_time_get_ns(dev);
			stream->last_frame_idx = stream->frame_idx;
			if (dev->hdr.hdr_mode == NO_HDR) {
				rkcif_s_rx_buffer(stream, &active_buf->dbufs);
				atomic_dec(&stream->buf_cnt);
			} else {
				rkcif_rdbk_frame_end_toisp(stream, active_buf);
			}
		} else {
			if (stream->cifdev->rdbk_debug && dev->hw_dev->dummy_buf.vaddr)
				v4l2_info(&stream->cifdev->v4l2_dev,
					  "stream[%d] loss frame %d\n",
					  stream->id,
					  stream->frame_idx - 1);
		}
		if (dev->is_support_tools && stream->tools_vdev && active_buf)
			rkcif_rdbk_with_tools(stream, active_buf);
	}
out_get_buf:
	buf_stream->frame_phase_cache = stream->frame_phase;
	if (buffer) {
		buff_addr_y = buffer->dummy.dma_addr;
		if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
			rkcif_write_buff_addr_multi_dev_combine(stream, frm_addr_y, 0,
								buff_addr_y, 0, false);
		} else {
			rkcif_write_register(dev, frm_addr_y, buff_addr_y);
		}
		if (dev->rdbk_debug > 1 &&
		    stream->frame_idx < 15)
			v4l2_info(&dev->v4l2_dev,
				  "stream[%d] update, seq %d, reg %x, buf %x\n",
				  stream->id,
				  stream->frame_idx - 1,
				  frm_addr_y, (u32)buffer->dummy.dma_addr);
	} else if (dev->hw_dev->dummy_buf.vaddr && priv &&
		   priv->mode.rdbk_mode == RKISP_VICAP_RDBK_AUTO) {
		buff_addr_y = dev->hw_dev->dummy_buf.dma_addr;
		if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
			rkcif_write_buff_addr_multi_dev_combine(stream, frm_addr_y, 0,
								buff_addr_y, 0, true);
		} else {
			rkcif_write_register(dev, frm_addr_y, buff_addr_y);
		}
	}
	spin_unlock_irqrestore(&buf_stream->vbq_lock, flags);
	return 0;
}

static int rkcif_assign_new_buffer_pingpong_toisp(struct rkcif_stream *stream,
						  int init, int channel_id)
{
	int ret = 0;

	if (init)
		rkcif_assign_new_buffer_init_toisp(stream, channel_id);
	else
		ret = rkcif_assign_new_buffer_update_toisp(stream, channel_id);
	return ret;
}

void rkcif_assign_check_buffer_update_toisp(struct rkcif_stream *stream)
{
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_mbus_config *mbus_cfg = &dev->active_sensor->mbus;
	struct rkcif_rx_buffer *buffer = NULL;
	struct rkmodule_capture_info *capture_info = &dev->channels[stream->id].capture_info;
	struct rkcif_stream *buf_stream = stream;
	u32 frm_addr_y, buff_addr_y;
	int frame_phase = 0;
	int frame_phase_next = 0;
	bool is_dual_update  = false;
	uint32_t cur_dma_addr = 0, next_dma_addr = 0;

	if (dev->switch_info.is_use_switch &&
	    dev->switch_info.switch_dev->switch_info.is_init_buf)
		buf_stream = &dev->switch_info.switch_dev->stream[stream->id];

	if (buf_stream->toisp_buf_state.state == RKCIF_TOISP_BUF_ROTATE ||
	    (buf_stream->toisp_buf_state.state == RKCIF_TOISP_BUF_THESAME &&
	     buf_stream->toisp_buf_state.check_cnt >= 1) ||
	    (buf_stream->toisp_buf_state.state == RKCIF_TOISP_BUF_LOSS &&
	     buf_stream->toisp_buf_state.check_cnt >= 2)) {
		if ((dev->rdbk_debug > 2 &&
		    stream->frame_idx < 15) ||
		    rkcif_debug == 3) {
			if (buf_stream->curr_buf_toisp)
				cur_dma_addr = buf_stream->curr_buf_toisp->dummy.dma_addr;
			if (buf_stream->next_buf_toisp)
				next_dma_addr = buf_stream->next_buf_toisp->dummy.dma_addr;
			v4l2_info(&dev->v4l2_dev,
				  "stream[%d] addr check not equal 0x%x 0x%x state %d chech_cnt %d\n",
				  stream->id,
				  cur_dma_addr,
				  next_dma_addr, buf_stream->toisp_buf_state.state, buf_stream->toisp_buf_state.check_cnt);
		}
		return;
	}
	v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev,
		 "stream[%d] addr check  0x%x 0x%x state %d chech_cnt %d\n",
		 stream->id,
		 cur_dma_addr,
		 next_dma_addr, buf_stream->toisp_buf_state.state, buf_stream->toisp_buf_state.check_cnt);
	if (stream->frame_phase == 0)
		stream->frame_phase = CIF_CSI_FRAME0_READY;
	frame_phase = stream->frame_phase;

	if (buf_stream->toisp_buf_state.state == RKCIF_TOISP_BUF_LOSS &&
	    buf_stream->toisp_buf_state.check_cnt == 0)
		is_dual_update = true;

	if (dev->rdbk_debug > 2 &&
	    stream->frame_idx < 15)
		v4l2_info(&dev->v4l2_dev,
			  "stream[%d] check update, lack_buf %d\n",
			  stream->id, buf_stream->lack_buf_cnt);
	if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY ||
	    mbus_cfg->type == V4L2_MBUS_CCP2) {
		frm_addr_y = frame_phase & CIF_CSI_FRAME0_READY ?
			     get_reg_index_of_frm0_y_addr(stream->id) :
			     get_reg_index_of_frm1_y_addr(stream->id);
	} else {
		frm_addr_y = frame_phase & CIF_CSI_FRAME0_READY ?
			     get_dvp_reg_index_of_frm0_y_addr(stream->id) :
			     get_dvp_reg_index_of_frm1_y_addr(stream->id);
	}
	if (!list_empty(&buf_stream->rx_buf_head)) {
		if (frame_phase == CIF_CSI_FRAME0_READY) {
			buffer = list_first_entry(&buf_stream->rx_buf_head,
						 struct rkcif_rx_buffer, list);
			if (buffer) {
				list_del(&buffer->list);
				buf_stream->curr_buf_toisp = buffer;
				buff_addr_y = buf_stream->curr_buf_toisp->dummy.dma_addr;
				if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
					rkcif_write_buff_addr_multi_dev_combine(stream,
										frm_addr_y, 0,
										buff_addr_y, 0,
										false);
				} else {
					rkcif_write_register(dev, frm_addr_y, buff_addr_y);
				}
				if (dev->rdbk_debug > 1 &&
				    stream->frame_idx < 15)
					v4l2_info(&dev->v4l2_dev,
						  "stream[%d] check update, seq %d, addr 0x%x, buf 0x%x\n",
						  stream->id,
						  stream->frame_idx - 1, frm_addr_y,
						  (u32)buf_stream->curr_buf_toisp->dummy.dma_addr);
			}
		} else if (frame_phase == CIF_CSI_FRAME1_READY) {
			buffer = list_first_entry(&buf_stream->rx_buf_head,
						 struct rkcif_rx_buffer, list);
			if (buffer) {
				list_del(&buffer->list);
				buf_stream->next_buf_toisp = buffer;
				buff_addr_y = buf_stream->next_buf_toisp->dummy.dma_addr;
				if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
					rkcif_write_buff_addr_multi_dev_combine(stream,
										frm_addr_y, 0,
										buff_addr_y, 0,
										false);
				} else {
					rkcif_write_register(dev, frm_addr_y, buff_addr_y);
				}
				if (dev->rdbk_debug > 1 &&
				    stream->frame_idx < 15)
					v4l2_info(&dev->v4l2_dev,
						  "stream[%d] check update, seq %d, addr 0x%x, buf 0x%x\n",
						  stream->id,
						  stream->frame_idx - 1, frm_addr_y,
						  (u32)buf_stream->next_buf_toisp->dummy.dma_addr);
			}
		}
		if (buf_stream->lack_buf_cnt)
			buf_stream->lack_buf_cnt--;
	}
	if (is_dual_update) {
		frame_phase_next = frame_phase & CIF_CSI_FRAME0_READY ?
				CIF_CSI_FRAME1_READY : CIF_CSI_FRAME0_READY;
		if (dev->rdbk_debug > 1 &&
		    stream->frame_idx < 15)
			v4l2_info(&dev->v4l2_dev,
				  "stream[%d] dual update, seq %d, phase %d\n",
				  stream->id,
				  stream->frame_idx - 1,
				  frame_phase);
		if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
		    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY ||
		    mbus_cfg->type == V4L2_MBUS_CCP2) {
			frm_addr_y = frame_phase_next & CIF_CSI_FRAME0_READY ?
			     get_reg_index_of_frm0_y_addr(stream->id) :
			     get_reg_index_of_frm1_y_addr(stream->id);
		} else {
			frm_addr_y = frame_phase_next & CIF_CSI_FRAME0_READY ?
				     get_dvp_reg_index_of_frm0_y_addr(stream->id) :
				     get_dvp_reg_index_of_frm1_y_addr(stream->id);
		}
		if (frame_phase == CIF_CSI_FRAME0_READY)
			buf_stream->next_buf_toisp = buf_stream->curr_buf_toisp;
		else
			buf_stream->curr_buf_toisp = buf_stream->next_buf_toisp;
		buff_addr_y = buf_stream->curr_buf_toisp->dummy.dma_addr;
		if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
			rkcif_write_buff_addr_multi_dev_combine(stream, frm_addr_y, 0,
								buff_addr_y, 0, false);
		} else {
			rkcif_write_register(dev, frm_addr_y, buff_addr_y);
		}
	}
	buf_stream->toisp_buf_state.check_cnt++;
}

static void rkcif_assign_new_buffer_init(struct rkcif_stream *stream,
					 int channel_id)
{
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_mbus_config *mbus_cfg = &dev->active_sensor->mbus;
	u32 frm0_addr_y, frm0_addr_uv;
	u32 frm1_addr_y, frm1_addr_uv;
	u32 buff_addr_y, buff_addr_cbcr;
	unsigned long flags;
	struct rkcif_dummy_buffer *dummy_buf = &dev->hw_dev->dummy_buf;
	struct csi_channel_info *channel = &dev->channels[channel_id];
	struct rkcif_stream *buf_stream = stream;

	stream->lack_buf_cnt = 0;
	if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY ||
	    mbus_cfg->type == V4L2_MBUS_CCP2) {
		frm0_addr_y = get_reg_index_of_frm0_y_addr(channel_id);
		frm0_addr_uv = get_reg_index_of_frm0_uv_addr(channel_id);
		frm1_addr_y = get_reg_index_of_frm1_y_addr(channel_id);
		frm1_addr_uv = get_reg_index_of_frm1_uv_addr(channel_id);
	} else {
		frm0_addr_y = get_dvp_reg_index_of_frm0_y_addr(channel_id);
		frm0_addr_uv = get_dvp_reg_index_of_frm0_uv_addr(channel_id);
		frm1_addr_y = get_dvp_reg_index_of_frm1_y_addr(channel_id);
		frm1_addr_uv = get_dvp_reg_index_of_frm1_uv_addr(channel_id);
	}

	spin_lock_irqsave(&stream->vbq_lock, flags);

	if (!stream->curr_buf) {
		if (!list_empty(&buf_stream->buf_head)) {
			stream->curr_buf = list_first_entry(&buf_stream->buf_head,
							    struct rkcif_buffer,
							    queue);
			v4l2_dbg(4, rkcif_debug, &dev->v4l2_dev, "%s %d, stream[%d] buf idx %d\n",
				 __func__, __LINE__, stream->id, stream->curr_buf->vb.vb2_buf.index);
			list_del(&stream->curr_buf->queue);
			atomic_inc(&buf_stream->sub_stream_buf_cnt);
		}
	}

	if (stream->curr_buf) {
		buff_addr_y = stream->curr_buf->buff_addr[RKCIF_PLANE_Y];
		buff_addr_cbcr = stream->curr_buf->buff_addr[RKCIF_PLANE_CBCR];
		if (channel->capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
			rkcif_write_buff_addr_multi_dev_combine(stream,
								frm0_addr_y,
								frm0_addr_uv,
								buff_addr_y,
								buff_addr_cbcr,
								false);
		} else {
			rkcif_write_register(dev, frm0_addr_y,
					     stream->curr_buf->buff_addr[RKCIF_PLANE_Y]);
			if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
				rkcif_write_register(dev, frm0_addr_uv,
						     stream->curr_buf->buff_addr[RKCIF_PLANE_CBCR]);
		}
	} else {
		if (dummy_buf->vaddr) {
			buff_addr_y = dummy_buf->dma_addr;
			buff_addr_cbcr = dummy_buf->dma_addr;
			if (channel->capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
				rkcif_write_buff_addr_multi_dev_combine(stream,
									frm0_addr_y,
									frm0_addr_uv,
									buff_addr_y,
									buff_addr_cbcr,
									true);
			} else {
				rkcif_write_register(dev, frm0_addr_y, buff_addr_y);
				if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
					rkcif_write_register(dev, frm0_addr_uv, buff_addr_cbcr);
			}
		} else {
			if (stream->lack_buf_cnt < 2)
				stream->lack_buf_cnt++;
		}
	}

	if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT) {
		stream->next_buf = stream->curr_buf;
		if (stream->next_buf) {
			buff_addr_y = stream->next_buf->buff_addr[RKCIF_PLANE_Y];
			buff_addr_cbcr = stream->next_buf->buff_addr[RKCIF_PLANE_CBCR];
			if (channel->capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
				rkcif_write_buff_addr_multi_dev_combine(stream,
									frm1_addr_y,
									frm1_addr_uv,
									buff_addr_y,
									buff_addr_cbcr,
									false);
			} else {
				rkcif_write_register(dev, frm1_addr_y,
						     buff_addr_y + (channel->virtual_width / 2));
				if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
					rkcif_write_register(dev, frm1_addr_uv,
							     buff_addr_cbcr + (channel->virtual_width / 2));
			}
		}
	} else {
		if (channel->capture_info.mode == RKMODULE_ONE_CH_TO_MULTI_ISP &&
		    channel->capture_info.one_to_multi.frame_pattern[0] == 1)
			buf_stream = &dev->stream[dev->sditf[1]->connect_id];
		if (!stream->next_buf) {
			if (!list_empty(&buf_stream->buf_head)) {
				stream->next_buf = list_first_entry(&buf_stream->buf_head,
								    struct rkcif_buffer, queue);
				v4l2_dbg(4, rkcif_debug, &dev->v4l2_dev, "%s %d, stream[%d] buf idx %d\n",
					 __func__, __LINE__, stream->id, stream->next_buf->vb.vb2_buf.index);
				list_del(&stream->next_buf->queue);
				atomic_inc(&buf_stream->sub_stream_buf_cnt);
			}
		}

		if (!stream->next_buf && dummy_buf->vaddr) {
			buff_addr_y = dummy_buf->dma_addr;
			buff_addr_cbcr = dummy_buf->dma_addr;
			if (channel->capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
				rkcif_write_buff_addr_multi_dev_combine(stream,
									frm1_addr_y,
									frm1_addr_uv,
									buff_addr_y,
									buff_addr_cbcr,
									true);
			} else {
				rkcif_write_register(dev, frm1_addr_y, dummy_buf->dma_addr);
				if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
					rkcif_write_register(dev, frm1_addr_uv, dummy_buf->dma_addr);
			}

		} else if (!stream->next_buf && stream->curr_buf) {
			stream->next_buf = stream->curr_buf;
			if (stream->lack_buf_cnt < 2)
				stream->lack_buf_cnt++;
		}
		if (stream->next_buf) {
			buff_addr_y = stream->next_buf->buff_addr[RKCIF_PLANE_Y];
			buff_addr_cbcr = stream->next_buf->buff_addr[RKCIF_PLANE_CBCR];
			if (channel->capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
				rkcif_write_buff_addr_multi_dev_combine(stream,
									frm1_addr_y,
									frm1_addr_uv,
									buff_addr_y,
									buff_addr_cbcr,
									false);
			} else {
				rkcif_write_register(dev, frm1_addr_y, buff_addr_y);
				if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
					rkcif_write_register(dev, frm1_addr_uv, buff_addr_cbcr);
			}
		} else {
			if (stream->lack_buf_cnt < 2)
				stream->lack_buf_cnt++;
		}
	}
	spin_unlock_irqrestore(&stream->vbq_lock, flags);

	stream->is_dvp_yuv_addr_init = true;

	/* for BT.656/BT.1120 multi channels function,
	 * yuv addr of unused channel must be set
	 */
	if (mbus_cfg->type == V4L2_MBUS_BT656) {
		int ch_id;

		for (ch_id = 0; ch_id < RKCIF_MAX_STREAM_DVP; ch_id++) {
			if (dev->stream[ch_id].is_dvp_yuv_addr_init)
				continue;
			if (dummy_buf->dma_addr) {
				rkcif_write_register(dev,
						     get_dvp_reg_index_of_frm0_y_addr(ch_id),
						     dummy_buf->dma_addr);
				rkcif_write_register(dev,
						     get_dvp_reg_index_of_frm0_uv_addr(ch_id),
						     dummy_buf->dma_addr);
				rkcif_write_register(dev,
						     get_dvp_reg_index_of_frm1_y_addr(ch_id),
						     dummy_buf->dma_addr);
				rkcif_write_register(dev,
						     get_dvp_reg_index_of_frm1_uv_addr(ch_id),
						     dummy_buf->dma_addr);
			}
		}
	}
	stream->buf_owner = RKCIF_DMAEN_BY_VICAP;
}

static struct rkcif_stream *rkcif_get_update_buffer_stream(struct rkcif_stream *stream)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_stream *buf_stream = NULL;
	u32 i = 0, pattern_cnt = 0, tmp = 0;

	for (i = 0; i < dev->channels[0].capture_info.one_to_multi.isp_num; i++)
		pattern_cnt += dev->channels[0].capture_info.one_to_multi.frame_pattern[i];

	if (pattern_cnt == 0) {
		v4l2_info(&dev->v4l2_dev,
			  "stream[%d] pattern_cnt is %d, pls check it\n",
			  stream->id, pattern_cnt);
		return NULL;
	}
	tmp = (stream->frame_idx + 1) % pattern_cnt;
	pattern_cnt = 0;
	for (i = 0; i < dev->channels[0].capture_info.one_to_multi.isp_num; i++) {
		pattern_cnt += dev->channels[0].capture_info.one_to_multi.frame_pattern[i];
		if (tmp < pattern_cnt) {
			buf_stream = &dev->stream[i];
			break;
		}
	}
	if (buf_stream == NULL)
		buf_stream = stream;
	if (dev->exp_dbg)
		v4l2_info(&dev->v4l2_dev,
			  "stream[%d] update buffer\n", buf_stream->id);
	return buf_stream;
}

static int rkcif_assign_new_buffer_update(struct rkcif_stream *stream,
					  int channel_id)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_dummy_buffer *dummy_buf = &dev->hw_dev->dummy_buf;
	struct v4l2_mbus_config *mbus_cfg = &dev->active_sensor->mbus;
	struct rkcif_buffer *buffer = NULL;
	u32 frm_addr_y, frm_addr_uv;
	struct csi_channel_info *channel = &dev->channels[channel_id];
	struct rkisp_rx_buf *dbufs = NULL;
	struct dma_buf *dbuf = NULL;
	struct rkcif_stream *buf_stream = stream;
	int ret = 0;
	u32 buff_addr_y = 0, buff_addr_cbcr = 0;
	unsigned long flags;

	if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY ||
	    mbus_cfg->type == V4L2_MBUS_CCP2) {
		frm_addr_y = stream->frame_phase & CIF_CSI_FRAME0_READY ?
			     get_reg_index_of_frm0_y_addr(channel_id) :
			     get_reg_index_of_frm1_y_addr(channel_id);
		frm_addr_uv = stream->frame_phase & CIF_CSI_FRAME0_READY ?
			      get_reg_index_of_frm0_uv_addr(channel_id) :
			      get_reg_index_of_frm1_uv_addr(channel_id);
	} else {
		frm_addr_y = stream->frame_phase & CIF_CSI_FRAME0_READY ?
			     get_dvp_reg_index_of_frm0_y_addr(channel_id) :
			     get_dvp_reg_index_of_frm1_y_addr(channel_id);
		frm_addr_uv = stream->frame_phase & CIF_CSI_FRAME0_READY ?
			      get_dvp_reg_index_of_frm0_uv_addr(channel_id) :
			      get_dvp_reg_index_of_frm1_uv_addr(channel_id);
	}

	if (stream->to_stop_dma && (stream->dma_en & RKCIF_DMAEN_BY_ISP)) {
		v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev, "%s %d\n", __func__, __LINE__);
		goto stop_dma;
	}

	if (dev->channels[0].capture_info.mode == RKMODULE_ONE_CH_TO_MULTI_ISP)
		buf_stream = rkcif_get_update_buffer_stream(stream);

	spin_lock_irqsave(&stream->vbq_lock, flags);
	if (dev->channels[0].capture_info.mode == RKMODULE_ONE_CH_TO_MULTI_ISP &&
	    (buf_stream->state != RKCIF_STATE_STREAMING || buf_stream->stopping)) {
		buffer = NULL;
		if (stream->frame_phase == CIF_CSI_FRAME0_READY)
			stream->curr_buf = NULL;
		else
			stream->next_buf = NULL;
	} else if (!list_empty(&buf_stream->buf_head)) {

		if (!dummy_buf->vaddr &&
		    stream->curr_buf == stream->next_buf &&
		    rkcif_get_interlace_mode(stream) != RKCIF_INTERLACE_SOFT)
			ret = -EINVAL;

		if (stream->frame_phase == CIF_CSI_FRAME0_READY) {
			if (!stream->curr_buf)
				ret = -EINVAL;
			stream->curr_buf = list_first_entry(&buf_stream->buf_head,
							    struct rkcif_buffer, queue);
			if (stream->curr_buf) {
				list_del(&stream->curr_buf->queue);
				buffer = stream->curr_buf;
				v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev,
					 "stream[%d] update curr_buf 0x%x, buf idx %d\n",
					 stream->id, buffer->buff_addr[0], stream->curr_buf->vb.vb2_buf.index);
			}
		} else if (stream->frame_phase == CIF_CSI_FRAME1_READY) {
			if (!stream->next_buf)
				ret = -EINVAL;
			if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT) {
				if (stream->next_buf != stream->curr_buf) {
					stream->next_buf = stream->curr_buf;
					buffer = stream->next_buf;
				} else {
					buffer = NULL;
				}

			} else {
				stream->next_buf = list_first_entry(&buf_stream->buf_head,
								    struct rkcif_buffer, queue);
				if (stream->next_buf) {
					list_del(&stream->next_buf->queue);
					buffer = stream->next_buf;
					v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev,
						 "stream[%d] update next_buf 0x%x, buf idx %d\n",
						 stream->id, buffer->buff_addr[0], stream->next_buf->vb.vb2_buf.index);
				}
			}
		}
	} else {
		buffer = NULL;
		if (!(stream->cur_stream_mode & RKCIF_STREAM_MODE_TOISP) && dummy_buf->vaddr) {
			if (stream->frame_phase == CIF_CSI_FRAME0_READY) {
				stream->curr_buf  = NULL;
			} else if (stream->frame_phase == CIF_CSI_FRAME1_READY) {
				if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT) {
					stream->next_buf = stream->curr_buf;
					buffer = stream->next_buf;
				} else {
					stream->next_buf = NULL;
				}
			}
		} else if (stream->curr_buf != stream->next_buf) {
			if (stream->frame_phase == CIF_CSI_FRAME0_READY && stream->next_buf) {
				stream->curr_buf = stream->next_buf;
				buffer = stream->next_buf;
			} else if (stream->frame_phase == CIF_CSI_FRAME1_READY && stream->curr_buf) {
				stream->next_buf = stream->curr_buf;
				buffer = stream->curr_buf;
			}
			if (stream->lack_buf_cnt < 2)
				stream->lack_buf_cnt++;
		} else {
			stream->curr_buf = NULL;
			stream->next_buf = NULL;
			if (stream->lack_buf_cnt < 2)
				stream->lack_buf_cnt++;
		}
	}
	stream->frame_phase_cache = stream->frame_phase;

	if (buffer) {
		buff_addr_y = buffer->buff_addr[RKCIF_PLANE_Y];
		buff_addr_cbcr = buffer->buff_addr[RKCIF_PLANE_CBCR];
		if (stream->dma_en & RKCIF_DMAEN_BY_ISP) {
			if (stream->buf_replace_cnt < 2)
				stream->buf_replace_cnt++;
			if (stream->frame_phase == CIF_CSI_FRAME0_READY &&
			    stream->next_buf)
				dbuf = stream->next_buf->dbuf;
			else if (stream->frame_phase == CIF_CSI_FRAME1_READY &&
				 stream->curr_buf)
				dbuf = stream->curr_buf->dbuf;

			if (dbuf) {
				list_for_each_entry(dbufs, &stream->rx_buf_head_vicap, list) {
					if (dbufs->dbuf == dbuf)
						break;
				}
			}
			if (dbufs)
				rkcif_s_rx_buffer(stream, dbufs);
		}
	} else if (dummy_buf) {
		buff_addr_y = dummy_buf->dma_addr;
		buff_addr_cbcr = dummy_buf->dma_addr;
	}
	if (buffer || dummy_buf) {
		if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT &&
		    stream->frame_phase == CIF_CSI_FRAME1_READY) {
			if (channel->capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
				rkcif_write_buff_addr_multi_dev_combine(stream,
									frm_addr_y,
									frm_addr_uv,
									buff_addr_y,
									buff_addr_cbcr,
									false);
			} else {
				rkcif_write_register(dev, frm_addr_y,
						     buff_addr_y + (channel->virtual_width / 2));
				if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
					rkcif_write_register(dev, frm_addr_uv,
							     buff_addr_cbcr + (channel->virtual_width / 2));
			}
		} else {
			if (channel->capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
				rkcif_write_buff_addr_multi_dev_combine(stream,
									frm_addr_y,
									frm_addr_uv,
									buff_addr_y,
									buff_addr_cbcr,
									false);
			} else {
				rkcif_write_register(dev, frm_addr_y, buff_addr_y);
				if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
					rkcif_write_register(dev, frm_addr_uv, buff_addr_cbcr);
			}
		}
	}
	spin_unlock_irqrestore(&stream->vbq_lock, flags);
	return ret;
stop_dma:
	if (stream->buf_replace_cnt) {
		spin_lock_irqsave(&stream->vbq_lock, flags);
		buff_addr_y = stream->curr_buf_toisp->dummy.dma_addr;
		if (channel->capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE)
			rkcif_write_buff_addr_multi_dev_combine(stream,
								frm_addr_y, 0,
								buff_addr_y, 0, false);
		else
			rkcif_write_register(dev, frm_addr_y, buff_addr_y);
		if (stream->frame_phase == CIF_CSI_FRAME0_READY &&
		    stream->next_buf)
			dbuf = stream->next_buf->dbuf;
		else if (stream->frame_phase == CIF_CSI_FRAME1_READY &&
			 stream->curr_buf)
			dbuf = stream->curr_buf->dbuf;

		if (dbuf) {
			list_for_each_entry(dbufs, &stream->rx_buf_head_vicap, list)
				if (dbufs->dbuf == dbuf)
					break;
		} else {
			dbufs = &stream->curr_buf_toisp->dbufs;
		}
		if (dbufs)
			rkcif_s_rx_buffer(stream, dbufs);

		if (stream->frame_phase == CIF_CSI_FRAME0_READY &&
		    stream->curr_buf) {
			stream->curr_buf = NULL;
		} else if (stream->frame_phase == CIF_CSI_FRAME1_READY &&
			   stream->next_buf) {
			stream->next_buf = NULL;
		}
		stream->buf_replace_cnt--;
		spin_unlock_irqrestore(&stream->vbq_lock, flags);
	}
	return -EINVAL;
}

static int rkcif_get_new_buffer_wake_up_mode(struct rkcif_stream *stream)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_dummy_buffer *dummy_buf = &dev->hw_dev->dummy_buf;
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&stream->vbq_lock, flags);
	if (!list_empty(&stream->buf_head)) {
		if (!dummy_buf->vaddr &&
		    stream->curr_buf == stream->next_buf)
			ret = -EINVAL;
		if (stream->line_int_cnt % 2) {
			stream->curr_buf = list_first_entry(&stream->buf_head,
							    struct rkcif_buffer, queue);
			if (stream->curr_buf)
				list_del(&stream->curr_buf->queue);
		} else {
			stream->next_buf = list_first_entry(&stream->buf_head,
							    struct rkcif_buffer, queue);
			if (stream->next_buf)
				list_del(&stream->next_buf->queue);
		}
		stream->is_buf_active = true;
		if (stream->lack_buf_cnt)
			stream->lack_buf_cnt--;
	} else {
		stream->is_buf_active = false;
		if (dummy_buf->vaddr) {
			if (stream->line_int_cnt % 2)
				stream->curr_buf = NULL;
			else
				stream->next_buf = NULL;
		} else if (stream->curr_buf != stream->next_buf) {
			if (stream->line_int_cnt % 2) {
				stream->curr_buf = stream->next_buf;
				stream->frame_phase_cache = CIF_CSI_FRAME0_READY;
			} else {
				stream->next_buf = stream->curr_buf;
				stream->frame_phase_cache = CIF_CSI_FRAME1_READY;
			}
			stream->is_buf_active = true;
			if (stream->lack_buf_cnt < 2)
				stream->lack_buf_cnt++;
		} else {
			if (dev->chip_id < CHIP_RK3588_CIF)
				ret = -EINVAL;
			else
				ret = 0;
			if (stream->lack_buf_cnt < 2)
				stream->lack_buf_cnt++;
		}
	}
	spin_unlock_irqrestore(&stream->vbq_lock, flags);

	return ret;
}

static int rkcif_update_new_buffer_wake_up_mode(struct rkcif_stream *stream)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_dummy_buffer *dummy_buf = &dev->hw_dev->dummy_buf;
	struct v4l2_mbus_config *mbus_cfg = &dev->active_sensor->mbus;
	struct rkmodule_capture_info *capture_info = &dev->channels[stream->id].capture_info;
	struct rkcif_buffer *buffer = NULL;
	u32 frm_addr_y, frm_addr_uv;
	u32 buff_addr_y, buff_addr_cbcr;
	int channel_id = stream->id;
	int ret = 0;
	unsigned long flags;

	if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY ||
	    mbus_cfg->type == V4L2_MBUS_CCP2) {
		frm_addr_y = stream->frame_phase & CIF_CSI_FRAME0_READY ?
			     get_reg_index_of_frm0_y_addr(channel_id) :
			     get_reg_index_of_frm1_y_addr(channel_id);
		frm_addr_uv = stream->frame_phase & CIF_CSI_FRAME0_READY ?
			      get_reg_index_of_frm0_uv_addr(channel_id) :
			      get_reg_index_of_frm1_uv_addr(channel_id);
	} else {
		frm_addr_y = stream->frame_phase & CIF_CSI_FRAME0_READY ?
			     get_dvp_reg_index_of_frm0_y_addr(channel_id) :
			     get_dvp_reg_index_of_frm1_y_addr(channel_id);
		frm_addr_uv = stream->frame_phase & CIF_CSI_FRAME0_READY ?
			      get_dvp_reg_index_of_frm0_uv_addr(channel_id) :
			      get_dvp_reg_index_of_frm1_uv_addr(channel_id);
	}
	spin_lock_irqsave(&stream->vbq_lock, flags);
	if (stream->is_buf_active) {
		if (stream->frame_phase == CIF_CSI_FRAME0_READY)
			buffer = stream->curr_buf;
		else if (stream->frame_phase == CIF_CSI_FRAME1_READY)
			buffer = stream->next_buf;
	}
	spin_unlock_irqrestore(&stream->vbq_lock, flags);
	if (buffer) {
		buff_addr_y = buffer->buff_addr[RKCIF_PLANE_Y];
		buff_addr_cbcr = buffer->buff_addr[RKCIF_PLANE_CBCR];
		if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
			rkcif_write_buff_addr_multi_dev_combine(stream, frm_addr_y,
								frm_addr_uv,
								buff_addr_y,
								buff_addr_cbcr,
								false);
		} else {
			rkcif_write_register(dev, frm_addr_y, buff_addr_y);
			if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
				rkcif_write_register(dev, frm_addr_uv, buff_addr_cbcr);
		}
	} else {
		if (dummy_buf->vaddr) {
			buff_addr_y = dummy_buf->dma_addr;
			buff_addr_cbcr = dummy_buf->dma_addr;
			if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
				rkcif_write_buff_addr_multi_dev_combine(stream,
									frm_addr_y,
									frm_addr_uv,
									buff_addr_y,
									buff_addr_cbcr,
									true);
			} else {
				rkcif_write_register(dev, frm_addr_y, buff_addr_y);
				if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
					rkcif_write_register(dev, frm_addr_uv, buff_addr_cbcr);
			}
		} else {
			if (dev->chip_id < CHIP_RK3588_CIF)
				ret = -EINVAL;
			else
				ret = 0;
		}
		dev->err_state |= (RKCIF_ERR_ID0_NOT_BUF << stream->id);
		dev->irq_stats.not_active_buf_cnt[stream->id]++;
	}

	return ret;
}

static int rkcif_get_new_buffer_wake_up_mode_rdbk(struct rkcif_stream *stream)
{
	struct rkcif_rx_buffer *buffer = NULL;
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_mbus_config *mbus_cfg = &dev->active_sensor->mbus;
	int ret = 0;
	unsigned long flags;
	u32 frm_addr_y;
	int frame_phase = 0;

	spin_lock_irqsave(&stream->vbq_lock, flags);
	memset(&stream->toisp_buf_state, 0, sizeof(stream->toisp_buf_state));
	if (!list_empty(&stream->rx_buf_head)) {
		if (stream->curr_buf_toisp && stream->next_buf_toisp &&
		    stream->curr_buf_toisp != stream->next_buf_toisp)
			stream->toisp_buf_state.state = RKCIF_TOISP_BUF_ROTATE;
		else
			stream->toisp_buf_state.state = RKCIF_TOISP_BUF_LOSS;
		if (stream->line_int_cnt % 2) {
			buffer = list_first_entry(&stream->rx_buf_head,
						 struct rkcif_rx_buffer, list);
			if (buffer) {
				list_del(&buffer->list);
				stream->curr_buf_toisp = buffer;
			}
			frame_phase = CIF_CSI_FRAME0_READY;
		} else {
			buffer = list_first_entry(&stream->rx_buf_head,
						 struct rkcif_rx_buffer, list);
			if (buffer) {
				list_del(&buffer->list);
				stream->next_buf_toisp = buffer;
			}
			frame_phase = CIF_CSI_FRAME1_READY;
		}
		if (stream->lack_buf_cnt)
			stream->lack_buf_cnt--;
	} else {
		if (stream->lack_buf_cnt < 2)
			stream->lack_buf_cnt++;
		if (stream->curr_buf_toisp && stream->next_buf_toisp &&
		    stream->curr_buf_toisp != stream->next_buf_toisp) {
			if (stream->line_int_cnt % 2) {
				stream->curr_buf_toisp = stream->next_buf_toisp;
				frame_phase = CIF_CSI_FRAME0_READY;
			} else {
				stream->next_buf_toisp = stream->curr_buf_toisp;
				frame_phase = CIF_CSI_FRAME1_READY;
			}
			buffer = stream->curr_buf_toisp;
			ret = 0;
			if (stream->cifdev->rdbk_debug)
				v4l2_info(&stream->cifdev->v4l2_dev,
					  "stream[%d] hold buf %x\n",
					  stream->id,
					  (u32)stream->next_buf_toisp->dummy.dma_addr);
			stream->toisp_buf_state.state = RKCIF_TOISP_BUF_THESAME;
		} else {
			ret = -EINVAL;
			stream->toisp_buf_state.state = RKCIF_TOISP_BUF_LOSS;
		}
	}
	if (buffer) {
		if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
		    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY ||
		    mbus_cfg->type == V4L2_MBUS_CCP2) {
			frm_addr_y = frame_phase & CIF_CSI_FRAME0_READY ?
				     get_reg_index_of_frm0_y_addr(stream->id) :
				     get_reg_index_of_frm1_y_addr(stream->id);
		} else {
			frm_addr_y = frame_phase & CIF_CSI_FRAME0_READY ?
				     get_dvp_reg_index_of_frm0_y_addr(stream->id) :
				     get_dvp_reg_index_of_frm1_y_addr(stream->id);
		}
		rkcif_write_register(dev, frm_addr_y,
				     buffer->dummy.dma_addr);
		if (dev->rdbk_debug > 1 &&
		    stream->frame_idx < 15)
			v4l2_info(&dev->v4l2_dev,
				  "stream[%d] rdbk update, seq %d, reg %x, buf %x\n",
				  stream->id,
				  stream->frame_idx - 1,
				  frm_addr_y, (u32)buffer->dummy.dma_addr);
	}
	spin_unlock_irqrestore(&stream->vbq_lock, flags);

	return ret;
}

static void rkcif_assign_dummy_buffer(struct rkcif_stream *stream)
{
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_mbus_config *mbus_cfg = &dev->active_sensor->mbus;
	struct rkcif_dummy_buffer *dummy_buf = &dev->hw_dev->dummy_buf;
	unsigned long flags;

	spin_lock_irqsave(&stream->vbq_lock, flags);

	/* for BT.656/BT.1120 multi channels function,
	 * yuv addr of unused channel must be set
	 */
	if (mbus_cfg->type == V4L2_MBUS_BT656 && dummy_buf->vaddr) {
		rkcif_write_register(dev,
				     get_dvp_reg_index_of_frm0_y_addr(stream->id),
				     dummy_buf->dma_addr);
		rkcif_write_register(dev,
				     get_dvp_reg_index_of_frm0_uv_addr(stream->id),
				     dummy_buf->dma_addr);
		rkcif_write_register(dev,
				     get_dvp_reg_index_of_frm1_y_addr(stream->id),
				     dummy_buf->dma_addr);
		rkcif_write_register(dev,
				     get_dvp_reg_index_of_frm1_uv_addr(stream->id),
				     dummy_buf->dma_addr);
	}

	spin_unlock_irqrestore(&stream->vbq_lock, flags);
}

static int rkcif_assign_new_buffer_pingpong(struct rkcif_stream *stream,
					     int init, int channel_id)
{
	int ret = 0;

	if (init)
		rkcif_assign_new_buffer_init(stream, channel_id);
	else
		ret = rkcif_assign_new_buffer_update(stream, channel_id);
	return ret;
}

static void rkcif_assign_new_buffer_init_rockit(struct rkcif_stream *stream,
							  int channel_id)
{
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_mbus_config *mbus_cfg = &dev->active_sensor->mbus;
	u32 frm0_addr_y, frm0_addr_uv;
	u32 frm1_addr_y, frm1_addr_uv;
	unsigned long flags;
	struct rkcif_dummy_buffer *dummy_buf = &dev->hw_dev->dummy_buf;
	struct csi_channel_info *channel = &dev->channels[channel_id];

	if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY ||
	    mbus_cfg->type == V4L2_MBUS_CCP2) {
		frm0_addr_y = get_reg_index_of_frm0_y_addr(channel_id);
		frm0_addr_uv = get_reg_index_of_frm0_uv_addr(channel_id);
		frm1_addr_y = get_reg_index_of_frm1_y_addr(channel_id);
		frm1_addr_uv = get_reg_index_of_frm1_uv_addr(channel_id);
	} else {
		frm0_addr_y = get_dvp_reg_index_of_frm0_y_addr(channel_id);
		frm0_addr_uv = get_dvp_reg_index_of_frm0_uv_addr(channel_id);
		frm1_addr_y = get_dvp_reg_index_of_frm1_y_addr(channel_id);
		frm1_addr_uv = get_dvp_reg_index_of_frm1_uv_addr(channel_id);
	}

	spin_lock_irqsave(&stream->vbq_lock, flags);

	if (!stream->curr_buf_rockit) {
		if (!list_empty(&stream->rockit_buf_head)) {
			stream->curr_buf_rockit = list_first_entry(&stream->rockit_buf_head,
							    struct rkcif_buffer,
							    queue);
			list_del(&stream->curr_buf_rockit->queue);
		}
	}

	if (stream->curr_buf_rockit) {
		rkcif_write_register(dev, frm0_addr_y,
				     stream->curr_buf_rockit->buff_addr[RKCIF_PLANE_Y]);
		if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
			rkcif_write_register(dev, frm0_addr_uv,
					     stream->curr_buf_rockit->buff_addr[RKCIF_PLANE_CBCR]);
	} else {
		if (dummy_buf->vaddr) {
			rkcif_write_register(dev, frm0_addr_y, dummy_buf->dma_addr);
			if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
				rkcif_write_register(dev, frm0_addr_uv, dummy_buf->dma_addr);
		} else {
			if (stream->lack_buf_cnt < 2)
				stream->lack_buf_cnt++;
		}
	}

	if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT ||
	    rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO) {
		stream->next_buf_rockit = stream->curr_buf_rockit;
		if (stream->next_buf_rockit) {
			rkcif_write_register(dev, frm1_addr_y,
					     stream->next_buf_rockit->buff_addr[RKCIF_PLANE_Y] + (channel->virtual_width / 2));
			if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
				rkcif_write_register(dev, frm1_addr_uv,
						     stream->next_buf_rockit->buff_addr[RKCIF_PLANE_CBCR] + (channel->virtual_width / 2));
		}
	} else {
		if (!stream->next_buf_rockit) {
			if (!list_empty(&stream->rockit_buf_head)) {
				stream->next_buf_rockit = list_first_entry(&stream->rockit_buf_head,
								    struct rkcif_buffer, queue);
				list_del(&stream->next_buf_rockit->queue);
			}
		}

		if (stream->next_buf_rockit) {
			rkcif_write_register(dev, frm1_addr_y,
					     stream->next_buf_rockit->buff_addr[RKCIF_PLANE_Y]);
			if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
				rkcif_write_register(dev, frm1_addr_uv,
						     stream->next_buf_rockit->buff_addr[RKCIF_PLANE_CBCR]);
		} else {
			if (dummy_buf->vaddr) {
				rkcif_write_register(dev, frm1_addr_y, dummy_buf->dma_addr);
				if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
					rkcif_write_register(dev, frm1_addr_uv, dummy_buf->dma_addr);
			} else {
				if (stream->curr_buf_rockit) {
					stream->next_buf_rockit = stream->curr_buf_rockit;
					rkcif_write_register(dev, frm1_addr_y,
							     stream->next_buf_rockit->buff_addr[RKCIF_PLANE_Y]);
					if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
						rkcif_write_register(dev, frm1_addr_uv,
								     stream->next_buf_rockit->buff_addr[RKCIF_PLANE_CBCR]);
				}
				if (stream->lack_buf_cnt < 2)
					stream->lack_buf_cnt++;
			}
		}
	}
	spin_unlock_irqrestore(&stream->vbq_lock, flags);

	stream->is_dvp_yuv_addr_init = true;

	/* for BT.656/BT.1120 multi channels function,
	 * yuv addr of unused channel must be set
	 */
	if (mbus_cfg->type == V4L2_MBUS_BT656) {
		int ch_id;

		for (ch_id = 0; ch_id < RKCIF_MAX_STREAM_DVP; ch_id++) {
			if (dev->stream[ch_id].is_dvp_yuv_addr_init)
				continue;
			if (dummy_buf->dma_addr) {
				rkcif_write_register(dev,
						     get_dvp_reg_index_of_frm0_y_addr(ch_id),
						     dummy_buf->dma_addr);
				rkcif_write_register(dev,
						     get_dvp_reg_index_of_frm0_uv_addr(ch_id),
						     dummy_buf->dma_addr);
				rkcif_write_register(dev,
						     get_dvp_reg_index_of_frm1_y_addr(ch_id),
						     dummy_buf->dma_addr);
				rkcif_write_register(dev,
						     get_dvp_reg_index_of_frm1_uv_addr(ch_id),
						     dummy_buf->dma_addr);
			}
		}
	}
	stream->buf_owner = RKCIF_DMAEN_BY_ROCKIT;
}

static int rkcif_assign_new_buffer_update_rockit(struct rkcif_stream *stream,
							    int channel_id)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_dummy_buffer *dummy_buf = &dev->hw_dev->dummy_buf;
	struct v4l2_mbus_config *mbus_cfg = &dev->active_sensor->mbus;
	struct rkcif_buffer *buffer = NULL;
	u32 frm_addr_y, frm_addr_uv;
	struct csi_channel_info *channel = &dev->channels[channel_id];
	int ret = 0;
	unsigned long flags;

	if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY ||
	    mbus_cfg->type == V4L2_MBUS_CCP2) {
		frm_addr_y = stream->frame_phase & CIF_CSI_FRAME0_READY ?
			     get_reg_index_of_frm0_y_addr(channel_id) :
			     get_reg_index_of_frm1_y_addr(channel_id);
		frm_addr_uv = stream->frame_phase & CIF_CSI_FRAME0_READY ?
			      get_reg_index_of_frm0_uv_addr(channel_id) :
			      get_reg_index_of_frm1_uv_addr(channel_id);
	} else {
		frm_addr_y = stream->frame_phase & CIF_CSI_FRAME0_READY ?
			     get_dvp_reg_index_of_frm0_y_addr(channel_id) :
			     get_dvp_reg_index_of_frm1_y_addr(channel_id);
		frm_addr_uv = stream->frame_phase & CIF_CSI_FRAME0_READY ?
			      get_dvp_reg_index_of_frm0_uv_addr(channel_id) :
			      get_dvp_reg_index_of_frm1_uv_addr(channel_id);
	}

	spin_lock_irqsave(&stream->vbq_lock, flags);
	if (!list_empty(&stream->rockit_buf_head)) {

		if (!dummy_buf->vaddr &&
		    stream->curr_buf_rockit == stream->next_buf_rockit &&
		    (rkcif_get_interlace_mode(stream) != RKCIF_INTERLACE_SOFT ||
		     rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO))
			ret = -EINVAL;

		if (stream->frame_phase == CIF_CSI_FRAME0_READY) {
			if (!stream->curr_buf_rockit)
				ret = -EINVAL;
			stream->curr_buf_rockit = list_first_entry(&stream->rockit_buf_head,
							    struct rkcif_buffer, queue);
			if (stream->curr_buf_rockit) {
				list_del(&stream->curr_buf_rockit->queue);
				buffer = stream->curr_buf_rockit;
			}
		} else if (stream->frame_phase == CIF_CSI_FRAME1_READY) {
			if (!stream->next_buf_rockit)
				ret = -EINVAL;
			if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT ||
			    rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO) {
				if (stream->next_buf_rockit != stream->curr_buf_rockit) {
					stream->next_buf_rockit = stream->curr_buf_rockit;
					buffer = stream->next_buf_rockit;
				} else {
					buffer = NULL;
				}

			} else {
				stream->next_buf_rockit = list_first_entry(&stream->rockit_buf_head,
								    struct rkcif_buffer, queue);
				if (stream->next_buf_rockit) {
					list_del(&stream->next_buf_rockit->queue);
					buffer = stream->next_buf_rockit;
				}
			}
		}
	} else {
		buffer = NULL;
		if (dummy_buf->vaddr) {
			if (stream->frame_phase == CIF_CSI_FRAME0_READY) {
				stream->curr_buf_rockit  = NULL;
			} else if (stream->frame_phase == CIF_CSI_FRAME1_READY) {
				if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT ||
				    rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO) {
					stream->next_buf_rockit = stream->curr_buf_rockit;
					buffer = stream->next_buf_rockit;
				} else {
					stream->next_buf_rockit = NULL;
				}
			}
		} else if (stream->curr_buf_rockit && stream->next_buf_rockit &&
			   stream->curr_buf_rockit != stream->next_buf_rockit) {
			if (stream->frame_phase == CIF_CSI_FRAME0_READY) {
				stream->curr_buf_rockit = stream->next_buf_rockit;
				buffer = stream->next_buf_rockit;
			} else if (stream->frame_phase == CIF_CSI_FRAME1_READY) {
				stream->next_buf_rockit = stream->curr_buf_rockit;
				buffer = stream->curr_buf_rockit;
			}
			if (stream->lack_buf_cnt < 2)
				stream->lack_buf_cnt++;
		} else {
			if (stream->lack_buf_cnt < 2)
				stream->lack_buf_cnt++;
		}
	}
	stream->frame_phase_cache = stream->frame_phase;

	if (buffer) {
		if ((rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT ||
		     rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO) &&
		    stream->frame_phase == CIF_CSI_FRAME1_READY) {
			rkcif_write_register(dev, frm_addr_y,
					     buffer->buff_addr[RKCIF_PLANE_Y] + (channel->virtual_width / 2));
			if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
				rkcif_write_register(dev, frm_addr_uv,
						     buffer->buff_addr[RKCIF_PLANE_CBCR] + (channel->virtual_width / 2));
		} else {
			rkcif_write_register(dev, frm_addr_y,
					     buffer->buff_addr[RKCIF_PLANE_Y]);
			if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
				rkcif_write_register(dev, frm_addr_uv,
						     buffer->buff_addr[RKCIF_PLANE_CBCR]);
		}
	} else {
		if (dummy_buf->vaddr) {
			rkcif_write_register(dev, frm_addr_y, dummy_buf->dma_addr);
			if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
				rkcif_write_register(dev, frm_addr_uv, dummy_buf->dma_addr);
			dev->err_state |= (RKCIF_ERR_ID0_NOT_BUF << stream->id);
			dev->irq_stats.not_active_buf_cnt[stream->id]++;
		} else {
			ret = -EINVAL;
			dev->err_state |= (RKCIF_ERR_ID0_NOT_BUF << stream->id);
			dev->irq_stats.not_active_buf_cnt[stream->id]++;
		}
	}
	spin_unlock_irqrestore(&stream->vbq_lock, flags);
	return ret;
}

static int rkcif_assign_new_buffer_pingpong_rockit(struct rkcif_stream *stream,
					     int init, int channel_id)
{
	int ret = 0;

	if (init)
		rkcif_assign_new_buffer_init_rockit(stream, channel_id);
	else
		ret = rkcif_assign_new_buffer_update_rockit(stream, channel_id);
	return ret;
}

void rkcif_check_buffer_update_pingpong_rockit(struct rkcif_stream *stream,
					       int channel_id)
{
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_mbus_config *mbus_cfg = &dev->active_sensor->mbus;
	struct rkcif_buffer *buffer = NULL;
	u32 frm_addr_y = 0, frm_addr_uv = 0;
	u32 frm0_addr_y = 0, frm0_addr_uv = 0;
	u32 frm1_addr_y = 0, frm1_addr_uv = 0;
	u32 buff_addr_y = 0, buff_addr_cbcr = 0;
	unsigned long flags;
	int frame_phase = 0;
	bool is_dual_update_buf = false;
	int on = 1;

	if (stream->state != RKCIF_STATE_STREAMING ||
	    rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT ||
	    rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO)
		return;

	spin_lock_irqsave(&stream->vbq_lock, flags);
	if (stream->curr_buf_rockit == stream->next_buf_rockit ||
	    stream->curr_buf_rockit == NULL ||
	    stream->next_buf_rockit == NULL) {
		frame_phase = stream->frame_phase_cache;
		if (!stream->is_line_wake_up ||
		    (stream->is_line_wake_up && stream->frame_idx < 2)) {
			if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
			    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY ||
			    mbus_cfg->type == V4L2_MBUS_CCP2) {
				frm0_addr_y = get_reg_index_of_frm0_y_addr(channel_id);
				frm1_addr_y = get_reg_index_of_frm1_y_addr(channel_id);
				frm0_addr_uv = get_reg_index_of_frm0_uv_addr(channel_id);
				frm1_addr_uv = get_reg_index_of_frm1_uv_addr(channel_id);
			} else {
				frm0_addr_y = get_dvp_reg_index_of_frm0_y_addr(channel_id);
				frm1_addr_y = get_dvp_reg_index_of_frm1_y_addr(channel_id);
				frm0_addr_uv = get_dvp_reg_index_of_frm0_uv_addr(channel_id);
				frm1_addr_uv = get_dvp_reg_index_of_frm1_uv_addr(channel_id);
			}
			if (frame_phase & CIF_CSI_FRAME0_READY) {
				frm_addr_y = frm0_addr_y;
				frm_addr_uv = frm0_addr_uv;
			} else {
				frm_addr_y = frm1_addr_y;
				frm_addr_uv = frm1_addr_uv;
			}
			if (!stream->dma_en && stream->curr_buf_rockit == NULL && stream->next_buf_rockit == NULL)
				is_dual_update_buf = true;
			if (!list_empty(&stream->rockit_buf_head)) {
				if (frame_phase == CIF_CSI_FRAME0_READY) {
					stream->curr_buf_rockit = list_first_entry(&stream->rockit_buf_head,
									    struct rkcif_buffer, queue);
					if (stream->curr_buf_rockit) {
						list_del(&stream->curr_buf_rockit->queue);
						buffer = stream->curr_buf_rockit;
					}
					if (buffer && is_dual_update_buf)
						stream->next_buf_rockit = buffer;
				} else if (frame_phase == CIF_CSI_FRAME1_READY) {
					stream->next_buf_rockit = list_first_entry(&stream->rockit_buf_head,
								    struct rkcif_buffer, queue);
					if (stream->next_buf_rockit) {
						list_del(&stream->next_buf_rockit->queue);
						buffer = stream->next_buf_rockit;
					}
					if (buffer && is_dual_update_buf)
						stream->curr_buf_rockit = buffer;
				}
			} else {
				v4l2_info(&dev->v4l2_dev, "%s %d\n", __func__, __LINE__);
			}
			if (buffer) {
				if (is_dual_update_buf) {
					buff_addr_y = buffer->buff_addr[RKCIF_PLANE_Y];
					buff_addr_cbcr = buffer->buff_addr[RKCIF_PLANE_CBCR];

					rkcif_write_register(dev, frm0_addr_y, buff_addr_y);
					if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
						rkcif_write_register(dev,
								     frm0_addr_uv,
								     buff_addr_cbcr);
					rkcif_write_register(dev, frm1_addr_y, buff_addr_y);
					if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
						rkcif_write_register(dev,
								     frm1_addr_uv,
								     buff_addr_cbcr);
				} else {

					buff_addr_y = buffer->buff_addr[RKCIF_PLANE_Y];
					buff_addr_cbcr = buffer->buff_addr[RKCIF_PLANE_CBCR];

					rkcif_write_register(dev, frm_addr_y, buff_addr_y);
					if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
						rkcif_write_register(dev,
								     frm_addr_uv,
								     buff_addr_cbcr);
				}
			}
		} else {
			v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
				 "%s %d, is_wake_up %d, frame_idx %d\n",
				 __func__, __LINE__, stream->is_line_wake_up, stream->frame_idx);
			if (stream->curr_buf_rockit == stream->next_buf_rockit) {
				if (stream->frame_phase_cache == CIF_CSI_FRAME0_READY) {
					stream->curr_buf_rockit = list_first_entry(&stream->rockit_buf_head,
									    struct rkcif_buffer, queue);
					v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev,
						 "%s %d, stream[%d] buf idx %d\n",
						 __func__, __LINE__, stream->id, stream->curr_buf_rockit->vb.vb2_buf.index);
					if (stream->curr_buf_rockit)
						list_del(&stream->curr_buf_rockit->queue);
				} else if (stream->frame_phase_cache == CIF_CSI_FRAME1_READY) {
					stream->next_buf_rockit = list_first_entry(&stream->rockit_buf_head,
									    struct rkcif_buffer, queue);
					v4l2_dbg(4, rkcif_debug, &dev->v4l2_dev,
						 "%s %d, stream[%d] buf idx %d\n",
						 __func__, __LINE__, stream->id, stream->next_buf_rockit->vb.vb2_buf.index);
					if (stream->next_buf_rockit)
						list_del(&stream->next_buf_rockit->queue);
				}
				stream->is_buf_active = true;
			}
		}
		v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
			 "%s, stream[%d] update buffer, frame_phase %d, is_stop %s, lack_buf_cnt %d\n",
			 __func__, stream->id, frame_phase,
			 (stream->dma_en ? "false" : "true"),
			 stream->lack_buf_cnt);
		if (!stream->dma_en) {
			if (stream->to_stop_dma) {
				stream->to_stop_dma = 0;
				wake_up(&stream->wq_stopped);
			} else {
				if (stream->cifdev->resume_mode != RKISP_RTT_MODE_ONE_FRAME)
					stream->to_en_dma = RKCIF_DMAEN_BY_ROCKIT;
				v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
					 "%s stream[%d] start dma capture, frame cnt %d\n",
					 __func__, stream->id, stream->frame_idx);
			}
		} else {
			v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
				 "%s %d, dma_en 0x%x, frame cnt %d\n",
				 __func__, __LINE__, stream->dma_en, stream->frame_idx);
		}
		if (stream->lack_buf_cnt)
			stream->lack_buf_cnt--;

	} else {
		v4l2_info(&dev->v4l2_dev, "%s %d, state %d, curr_buf_rockit %p, next_buf_rockit %p\n",
			  __func__, __LINE__, stream->state, stream->curr_buf_rockit, stream->next_buf_rockit);
	}
	spin_unlock_irqrestore(&stream->vbq_lock, flags);
	if (stream->to_en_dma) {
		rkcif_enable_dma_capture(stream, true);
		mutex_lock(&dev->stream_lock);
		if (atomic_read(&dev->sensor_off)) {
			atomic_set(&dev->sensor_off, 0);
			rkcif_dphy_quick_stream(dev, on);
			v4l2_subdev_call(dev->terminal_sensor.sd, core, ioctl,
					 RKMODULE_SET_QUICK_STREAM, &on);
		}
		mutex_unlock(&dev->stream_lock);
	}
}

static void rkcif_csi_set_lvds_sav_eav(struct rkcif_stream *stream,
				       struct csi_channel_info *channel)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkmodule_lvds_cfg *lvds_cfg = &channel->lvds_cfg;
	struct rkmodule_lvds_frame_sync_code *frm_sync_code = NULL;
	struct rkmodule_lvds_frm_sync_code *odd_sync_code = NULL;
	struct rkmodule_lvds_frm_sync_code *even_sync_code = NULL;

	if (dev->hdr.hdr_mode == NO_HDR || dev->hdr.hdr_mode == HDR_COMPR) {
		frm_sync_code = &lvds_cfg->frm_sync_code[LVDS_CODE_GRP_LINEAR];
		odd_sync_code = &frm_sync_code->odd_sync_code;
		even_sync_code = odd_sync_code;
	} else {
		if (channel->id == RKCIF_STREAM_MIPI_ID0)
			frm_sync_code = &lvds_cfg->frm_sync_code[LVDS_CODE_GRP_LONG];

		if (dev->hdr.hdr_mode == HDR_X2) {
			if (channel->id == RKCIF_STREAM_MIPI_ID1)
				frm_sync_code = &lvds_cfg->frm_sync_code[LVDS_CODE_GRP_SHORT];
			else
				frm_sync_code = &lvds_cfg->frm_sync_code[LVDS_CODE_GRP_LONG];
		} else if (dev->hdr.hdr_mode == HDR_X3) {
			if (channel->id == RKCIF_STREAM_MIPI_ID1)
				frm_sync_code = &lvds_cfg->frm_sync_code[LVDS_CODE_GRP_MEDIUM];
			else if (channel->id == RKCIF_STREAM_MIPI_ID2)
				frm_sync_code = &lvds_cfg->frm_sync_code[LVDS_CODE_GRP_SHORT];
			else
				frm_sync_code = &lvds_cfg->frm_sync_code[LVDS_CODE_GRP_LONG];
		}

		odd_sync_code = &frm_sync_code->odd_sync_code;
		even_sync_code = &frm_sync_code->even_sync_code;
	}

	if (odd_sync_code &&  even_sync_code) {
		rkcif_write_register(stream->cifdev,
				     get_reg_index_of_lvds_sav_eav_act0(channel->id),
				     SW_LVDS_EAV_ACT(odd_sync_code->act.eav) |
				     SW_LVDS_SAV_ACT(odd_sync_code->act.sav));

		rkcif_write_register(stream->cifdev,
				     get_reg_index_of_lvds_sav_eav_blk0(channel->id),
				     SW_LVDS_EAV_BLK(odd_sync_code->blk.eav) |
				     SW_LVDS_SAV_BLK(odd_sync_code->blk.sav));

		rkcif_write_register(stream->cifdev,
				     get_reg_index_of_lvds_sav_eav_act1(channel->id),
				     SW_LVDS_EAV_ACT(even_sync_code->act.eav) |
				     SW_LVDS_SAV_ACT(even_sync_code->act.sav));

		rkcif_write_register(stream->cifdev,
				     get_reg_index_of_lvds_sav_eav_blk1(channel->id),
				     SW_LVDS_EAV_BLK(even_sync_code->blk.eav) |
				     SW_LVDS_SAV_BLK(even_sync_code->blk.sav));
	}
}

static unsigned char get_csi_fmt_val(struct rkcif_stream *stream,
				     struct csi_channel_info *csi_info)
{
	const struct cif_input_fmt *cif_fmt_in = stream->cif_fmt_in;
	unsigned char csi_fmt_val = 0;

	if (cif_fmt_in->mbus_code == MEDIA_BUS_FMT_SPD_2X8 ||
	    cif_fmt_in->mbus_code == MEDIA_BUS_FMT_EBD_1X8) {
		switch (csi_info->data_bit) {
		case 8:
			csi_fmt_val = CSI_WRDDR_TYPE_RAW8;
			break;
		case 10:
			csi_fmt_val = CSI_WRDDR_TYPE_RAW10;
			break;
		case 12:
			csi_fmt_val = CSI_WRDDR_TYPE_RAW12;
			break;
		default:
			csi_fmt_val = CSI_WRDDR_TYPE_RAW12;
			break;
		}
	} else if (stream->cifdev->chip_id < CHIP_RK3576_CIF &&
		   cif_fmt_in->csi_fmt_val == CSI_WRDDR_TYPE_RGB888) {
		csi_fmt_val = CSI_WRDDR_TYPE_YUV422;
	} else if (cif_fmt_in->csi_fmt_val == CSI_WRDDR_TYPE_RGB565) {
		csi_fmt_val = CSI_WRDDR_TYPE_RAW8;
	} else {
		csi_fmt_val = cif_fmt_in->csi_fmt_val;
	}
	return csi_fmt_val;
}

static int rkcif_csi_channel_init(struct rkcif_stream *stream,
				  struct csi_channel_info *channel)
{
	struct rkcif_device *dev = stream->cifdev;
	struct sditf_priv *priv = dev->sditf[0];
	const struct cif_output_fmt *fmt;
	u32 fourcc;
	int vc = dev->channels[stream->id].vc;

	channel->enable = 1;
	channel->width = stream->pixm.width;
	channel->height = stream->pixm.height;

	channel->fmt_val = stream->cif_fmt_out->csi_fmt_val;

	channel->cmd_mode_en = dev->terminal_sensor.dsi_mode;; /* default use DSI Video Mode */
	channel->dsi_input = dev->terminal_sensor.dsi_input_en;

	if (stream->crop_enable) {
		channel->crop_en = 1;

		if (channel->fmt_val == CSI_WRDDR_TYPE_RGB888 && dev->chip_id < CHIP_RK3576_CIF)
			channel->crop_st_x = 3 * stream->crop[CROP_SRC_ACT].left / 2;
		else if (channel->fmt_val == CSI_WRDDR_TYPE_RGB565)
			channel->crop_st_x = 2 * stream->crop[CROP_SRC_ACT].left;
		else
			channel->crop_st_x = stream->crop[CROP_SRC_ACT].left;

		channel->crop_st_y = stream->crop[CROP_SRC_ACT].top;
		if (priv && priv->is_combine_mode && dev->sditf_cnt <= RKCIF_MAX_SDITF)
			channel->crop_st_y *= dev->sditf_cnt;
		channel->width = stream->crop[CROP_SRC_ACT].width;
		channel->height = stream->crop[CROP_SRC_ACT].height;
	} else {
		channel->crop_st_x = 0;
		channel->crop_st_y = 0;
		channel->width = stream->pixm.width;
		channel->height = stream->pixm.height;
		channel->crop_en = 0;
	}

	if (priv && priv->is_combine_mode && dev->sditf_cnt <= RKCIF_MAX_SDITF)
		channel->height *= dev->sditf_cnt;

	fmt = rkcif_find_output_fmt(stream, stream->pixm.pixelformat);
	if (!fmt) {
		v4l2_err(&dev->v4l2_dev, "can not find output format: 0x%x",
			 stream->pixm.pixelformat);
		return -EINVAL;
	}

	if (channel->capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE)
		channel->width /=  channel->capture_info.multi_dev.dev_num;

	if (dev->sditf[0] && dev->sditf[0]->mode.rdbk_mode == RKISP_VICAP_ONLINE_UNITE &&
	    (dev->hdr.hdr_mode == NO_HDR ||
	     dev->hdr.hdr_mode == HDR_COMPR ||
	     (dev->hdr.hdr_mode == HDR_X2 && stream->id == 1) ||
	     (dev->hdr.hdr_mode == HDR_X3 && stream->id == 2))) {
		channel->crop_st_x += channel->width / 2;
		channel->crop_st_x -= dev->unite_extend_pixel;
		channel->width /= 2;
		channel->width += dev->unite_extend_pixel;
	}
	/*
	 * for mipi or lvds, when enable compact, the virtual width of raw10/raw12
	 * needs aligned with :ALIGN(bits_per_pixel * width / 8, 8), if enable 16bit mode
	 * needs aligned with :ALIGN(bits_per_pixel * width * 2, 8), to optimize reading and
	 * writing of ddr, aliged with 256
	 */
	if (fmt->fmt_type == CIF_FMT_TYPE_RAW && stream->is_compact &&
	    fmt->csi_fmt_val != CSI_WRDDR_TYPE_RGB888 &&
	    fmt->csi_fmt_val != CSI_WRDDR_TYPE_RGB565) {
		if (channel->capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
			channel->virtual_width = ALIGN(channel->width * 2 * fmt->raw_bpp / 8, 256);
			channel->left_virtual_width = channel->width * fmt->raw_bpp / 8;
		} else {
			channel->virtual_width = ALIGN(channel->width * fmt->raw_bpp / 8, 256);
		}
	} else {
		if (channel->capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
			channel->virtual_width = ALIGN(channel->width * 2 * fmt->bpp[0] / 8, 8);
			channel->left_virtual_width = ALIGN(channel->width * fmt->bpp[0] / 8, 8);
		} else {
			channel->virtual_width = ALIGN(channel->width * fmt->bpp[0] / 8, 8);
		}
	}
	if (dev->chip_id > CHIP_RK3562_CIF && stream->sw_dbg_en)
		channel->virtual_width = (channel->virtual_width + 23) / 24 * 24;

	if ((channel->fmt_val == CSI_WRDDR_TYPE_RGB888 && dev->chip_id < CHIP_RK3576_CIF) ||
	    channel->fmt_val == CSI_WRDDR_TYPE_RGB565)
		channel->width = channel->width * fmt->bpp[0] / 8;

	if (channel->fmt_val == CSI_WRDDR_TYPE_RGB888 && dev->chip_id < CHIP_RK3576_CIF)
		channel->width /= 2;
	/*
	 * rk cif don't support output yuyv fmt data
	 * if user request yuyv fmt, the input mode must be RAW8
	 * and the width is double Because the real input fmt is
	 * yuyv
	 */
	fourcc = stream->cif_fmt_out->fourcc;
	if (fourcc == V4L2_PIX_FMT_YUYV || fourcc == V4L2_PIX_FMT_YVYU ||
	    fourcc == V4L2_PIX_FMT_UYVY || fourcc == V4L2_PIX_FMT_VYUY) {
		if (dev->chip_id < CHIP_RK3588_CIF) {
			channel->fmt_val = CSI_WRDDR_TYPE_RAW8;
			channel->width *= 2;
		}
		channel->virtual_width *= 2;
		if (channel->capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE)
			channel->left_virtual_width *= 2;
	}
	if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT ||
	    rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO) {
		channel->virtual_width *= 2;
		channel->height /= 2;
	}
	if (dev->channels[stream->id].data_type)
		channel->data_type = dev->channels[stream->id].data_type;
	else
		channel->data_type = get_data_type(stream->cif_fmt_in->mbus_code,
						   channel->cmd_mode_en,
						   channel->dsi_input);

	channel->csi_fmt_val = get_csi_fmt_val(stream,
					       &dev->channels[stream->id]);

	if (dev->hdr.hdr_mode == NO_HDR ||
	    dev->hdr.hdr_mode == HDR_COMPR ||
	    (dev->hdr.hdr_mode == HDR_X2 && stream->id > 1) ||
	    (dev->hdr.hdr_mode == HDR_X3 && stream->id > 2))
		channel->vc = vc < 4 ? vc : channel->id;
	else
		channel->vc = channel->id;
	v4l2_dbg(1, rkcif_debug, &dev->v4l2_dev,
		 "%s: channel width %d, height %d, virtual_width %d, vc %d\n", __func__,
		 channel->width, channel->height, channel->virtual_width, channel->vc);
	return 0;
}

static void rkcif_write_buffer(struct rkcif_stream *stream, struct rkcif_buffer *buffer,
			       int frame_phase, int even_offset)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_dummy_buffer *dummy_buf = &dev->hw_dev->dummy_buf;
	u32 frm_addr_y, frm_addr_uv;
	struct csi_channel_info *channel = &dev->channels[stream->id];

	if (frame_phase == CIF_CSI_FRAME0_READY) {
		frm_addr_y = get_reg_index_of_frm0_y_addr(stream->id);
		frm_addr_uv = get_reg_index_of_frm0_uv_addr(stream->id);
	} else {
		frm_addr_y = get_reg_index_of_frm1_y_addr(stream->id);
		frm_addr_uv = get_reg_index_of_frm1_uv_addr(stream->id);
	}
	if (buffer) {
		rkcif_write_register(dev, frm_addr_y,
			buffer->buff_addr[RKCIF_PLANE_Y] + even_offset *
			(channel->virtual_width / 2));
		if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
			rkcif_write_register(dev, frm_addr_uv,
				buffer->buff_addr[RKCIF_PLANE_CBCR] +
				even_offset * (channel->virtual_width / 2));
	} else if (dummy_buf->vaddr) {
		rkcif_write_register(dev, frm_addr_y, dummy_buf->dma_addr);
		if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
			rkcif_write_register(dev, frm_addr_uv, dummy_buf->dma_addr);
	}
}

static void rkcif_buf_init_interlace(struct rkcif_stream *stream, int channel_id)
{
	struct rkcif_device *dev = stream->cifdev;
	unsigned long flags;
	struct rkcif_dummy_buffer *dummy_buf = &dev->hw_dev->dummy_buf;
	int buf_offset = 0;

	spin_lock_irqsave(&stream->vbq_lock, flags);
	if (!stream->curr_buf) {
		if (!list_empty(&stream->buf_head)) {
			stream->curr_buf = list_first_entry(&stream->buf_head,
							    struct rkcif_buffer,
							    queue);
			list_del(&stream->curr_buf->queue);
		}
	}
	spin_unlock_irqrestore(&stream->vbq_lock, flags);

	if (!stream->odd_frame_first)
		buf_offset = 1;

	if (stream->curr_buf) {
		rkcif_write_buffer(stream, stream->curr_buf, CIF_CSI_FRAME0_READY, buf_offset);
		rkcif_write_buffer(stream, stream->curr_buf, CIF_CSI_FRAME1_READY, buf_offset);
	} else {
		if (dummy_buf->vaddr) {
			rkcif_write_buffer(stream, NULL, CIF_CSI_FRAME0_READY, 0);
			rkcif_write_buffer(stream, NULL, CIF_CSI_FRAME1_READY, 0);
		} else {
			v4l2_err(&dev->v4l2_dev,
				 "stream[%d] mipi interlace not buf, if cap_en, may cause system death\n",
				 stream->id);
		}
	}

}

static int rkcif_csi_channel_set(struct rkcif_stream *stream,
				 struct csi_channel_info *channel,
				 enum v4l2_mbus_type mbus_type)
{
	unsigned int val = 0x0;
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_stream *detect_stream = &dev->stream[0];
	unsigned int wait_line = 0x3fff;

	if (channel->id >= 4)
		return -EINVAL;

	if (!channel->enable) {
		rkcif_write_register(dev, get_reg_index_of_id_ctrl0(channel->id),
				     CSI_DISABLE_CAPTURE);
		return 0;
	}

	rkcif_write_register_and(dev, CIF_REG_MIPI_LVDS_INTSTAT,
				 ~(CSI_START_INTSTAT(channel->id) |
				 CSI_DMA_END_INTSTAT(channel->id) |
				 CSI_LINE_INTSTAT(channel->id)));

	rkcif_write_register_or(dev, CIF_REG_MIPI_LVDS_INTEN,
				CSI_START_INTEN(channel->id));

	if (detect_stream->is_line_wake_up) {
		rkcif_write_register_or(dev, CIF_REG_MIPI_LVDS_INTEN,
					CSI_LINE_INTEN(channel->id));
		wait_line = dev->wait_line;
	}
	rkcif_write_register(dev, CIF_REG_MIPI_LVDS_LINE_INT_NUM_ID0_1,
			     wait_line << 16 | wait_line);
	rkcif_write_register(dev, CIF_REG_MIPI_LVDS_LINE_INT_NUM_ID2_3,
			     wait_line << 16 | wait_line);

	rkcif_write_register_or(dev, CIF_REG_MIPI_LVDS_INTEN,
				CSI_DMA_END_INTEN(channel->id));
	rkcif_write_register(dev, CIF_REG_MIPI_WATER_LINE,
			     CIF_MIPI_LVDS_SW_WATER_LINE_25_RK1808 |
			     CIF_MIPI_LVDS_SW_WATER_LINE_ENABLE_RK1808 |
			     CIF_MIPI_LVDS_SW_HURRY_VALUE_RK1808(0x3) |
			     CIF_MIPI_LVDS_SW_HURRY_ENABLE_RK1808);

	val = CIF_MIPI_LVDS_SW_PRESS_VALUE(0x3) |
		CIF_MIPI_LVDS_SW_PRESS_ENABLE |
		CIF_MIPI_LVDS_SW_HURRY_VALUE(0x3) |
		CIF_MIPI_LVDS_SW_HURRY_ENABLE |
		CIF_MIPI_LVDS_SW_WATER_LINE_25 |
		CIF_MIPI_LVDS_SW_WATER_LINE_ENABLE;
	if (mbus_type == V4L2_MBUS_CSI2_DPHY) {
		val &= ~CIF_MIPI_LVDS_SW_SEL_LVDS;
	} else if (mbus_type == V4L2_MBUS_CCP2) {
		if (channel->fmt_val == CSI_WRDDR_TYPE_RAW12)
			val |= CIF_MIPI_LVDS_SW_LVDS_WIDTH_12BITS;
		else if (channel->fmt_val == CSI_WRDDR_TYPE_RAW10)
			val |= CIF_MIPI_LVDS_SW_LVDS_WIDTH_10BITS;
		else
			val |= CIF_MIPI_LVDS_SW_LVDS_WIDTH_8BITS;
		val |= CIF_MIPI_LVDS_SW_SEL_LVDS;
	}
	rkcif_write_register(dev, CIF_REG_MIPI_LVDS_CTRL, val);

	rkcif_write_register_or(dev, CIF_REG_MIPI_LVDS_INTEN,
				CSI_ALL_ERROR_INTEN);

	rkcif_write_register(dev, get_reg_index_of_id_ctrl1(channel->id),
			     channel->width | (channel->height << 16));

	rkcif_write_register(dev, get_reg_index_of_frm0_y_vlw(channel->id),
			     channel->virtual_width);
	rkcif_write_register(dev, get_reg_index_of_frm1_y_vlw(channel->id),
			     channel->virtual_width);
	rkcif_write_register(dev, get_reg_index_of_frm0_uv_vlw(channel->id),
			     channel->virtual_width);
	rkcif_write_register(dev, get_reg_index_of_frm1_uv_vlw(channel->id),
			     channel->virtual_width);

	rkcif_write_register(dev, get_reg_index_of_id_crop_start(channel->id),
			     channel->crop_st_y << 16 | channel->crop_st_x);

	/* Set up an buffer for the next frame */
	if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO)
		rkcif_buf_init_interlace(stream, channel->id);
	else
		rkcif_assign_new_buffer_pingpong(stream,
					 RKCIF_YUV_ADDR_STATE_INIT,
					 channel->id);

	if (mbus_type == V4L2_MBUS_CSI2_DPHY) {
		//need always enable crop
		val = CSI_ENABLE_CAPTURE | channel->fmt_val |
		      channel->cmd_mode_en << 4 | CSI_ENABLE_CROP |
		      channel->vc << 8 | channel->data_type << 10;
		if (stream->is_compact)
			val |= CSI_ENABLE_MIPI_COMPACT;
		else
			val &= ~CSI_ENABLE_MIPI_COMPACT;

		if (stream->cifdev->chip_id >= CHIP_RK3568_CIF)
			val |= stream->cif_fmt_in->csi_yuv_order;
	} else if (mbus_type == V4L2_MBUS_CCP2) {
		rkcif_csi_set_lvds_sav_eav(stream, channel);
		val = LVDS_ENABLE_CAPTURE | LVDS_MODE(channel->lvds_cfg.mode) |
		      LVDS_MAIN_LANE(0) | LVDS_FID(0) |
		      LVDS_LANES_ENABLED(dev->active_sensor->lanes);

		if (stream->is_compact)
			val |= LVDS_COMPACT;
		else
			val &= ~LVDS_COMPACT;
	}
	if (stream->is_high_align)
		val |= CSI_HIGH_ALIGN;
	else
		val &= ~CSI_HIGH_ALIGN;
	rkcif_write_register(dev, get_reg_index_of_id_ctrl0(channel->id), val);

	dev->intr_mask = rkcif_read_register(dev, CIF_REG_MIPI_LVDS_INTEN);
	return 0;
}

static int rkcif_dvp_get_input_yuv_order(struct rkcif_stream *stream)
{
	unsigned int mask;
	const struct cif_input_fmt *fmt = stream->cif_fmt_in;

	switch (fmt->mbus_code) {
	case MEDIA_BUS_FMT_UYVY8_2X8:
		mask = CSI_YUV_INPUT_ORDER_UYVY >> 11;
		break;
	case MEDIA_BUS_FMT_VYUY8_2X8:
		mask = CSI_YUV_INPUT_ORDER_VYUY >> 11;
		break;
	case MEDIA_BUS_FMT_YUYV8_2X8:
		mask = CSI_YUV_INPUT_ORDER_YUYV >> 11;
		break;
	case MEDIA_BUS_FMT_YVYU8_2X8:
		mask = CSI_YUV_INPUT_ORDER_YVYU >> 11;
		break;
	default:
		mask = CSI_YUV_INPUT_ORDER_UYVY >> 11;
		break;
	}
	return mask;
}

static int rkcif_dvp_get_input_yuv_order_rk3576(struct rkcif_stream *stream)
{
	unsigned int mask;
	const struct cif_input_fmt *fmt = stream->cif_fmt_in;

	switch (fmt->mbus_code) {
	case MEDIA_BUS_FMT_UYVY8_2X8:
		mask = CSI_YUV_INPUT_ORDER_UYVY;
		break;
	case MEDIA_BUS_FMT_VYUY8_2X8:
		mask = CSI_YUV_INPUT_ORDER_VYUY;
		break;
	case MEDIA_BUS_FMT_YUYV8_2X8:
		mask = CSI_YUV_INPUT_ORDER_YUYV;
		break;
	case MEDIA_BUS_FMT_YVYU8_2X8:
		mask = CSI_YUV_INPUT_ORDER_YVYU;
		break;
	default:
		mask = CSI_YUV_INPUT_ORDER_UYVY;
		break;
	}
	return mask;
}

static int rkcif_csi_get_output_type_mask(struct rkcif_stream *stream)
{
	unsigned int mask;
	const struct cif_output_fmt *fmt = stream->cif_fmt_out;

	switch (fmt->fourcc) {
	case V4L2_PIX_FMT_NV16:
		mask = CSI_WRDDR_TYPE_YUV422SP_RK3588 | CSI_YUV_OUTPUT_ORDER_UYVY;
		break;
	case V4L2_PIX_FMT_NV61:
		mask = CSI_WRDDR_TYPE_YUV422SP_RK3588 | CSI_YUV_OUTPUT_ORDER_VYUY;
		break;
	case V4L2_PIX_FMT_NV12:
		mask = CSI_WRDDR_TYPE_YUV420SP_RK3588 | CSI_YUV_OUTPUT_ORDER_UYVY;
		break;
	case V4L2_PIX_FMT_NV21:
		mask = CSI_WRDDR_TYPE_YUV420SP_RK3588 | CSI_YUV_OUTPUT_ORDER_VYUY;
		break;
	case V4L2_PIX_FMT_YUYV:
		mask = CSI_WRDDR_TYPE_YUV_PACKET | CSI_YUV_OUTPUT_ORDER_YUYV;
		break;
	case V4L2_PIX_FMT_YVYU:
		mask = CSI_WRDDR_TYPE_YUV_PACKET | CSI_YUV_OUTPUT_ORDER_YVYU;
		break;
	case V4L2_PIX_FMT_UYVY:
		mask = CSI_WRDDR_TYPE_YUV_PACKET | CSI_YUV_OUTPUT_ORDER_UYVY;
		break;
	case V4L2_PIX_FMT_VYUY:
		mask = CSI_WRDDR_TYPE_YUV_PACKET | CSI_YUV_OUTPUT_ORDER_VYUY;
		break;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
		mask = CSI_WRDDR_TYPE_YUV_PACKET | CSI_YUV_OUTPUT_ORDER_UYVY;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_BGR666:
		mask = CSI_WRDDR_TYPE_RAW_COMPACT;
		break;
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_Y10:
	case V4L2_PIX_FMT_Y12:
		if (stream->is_compact)
			mask = CSI_WRDDR_TYPE_RAW_COMPACT;
		else
			mask = CSI_WRDDR_TYPE_RAW_UNCOMPACT;
		break;
	case V4L2_PIX_FMT_SBGGR16:
	case V4L2_PIX_FMT_SGBRG16:
	case V4L2_PIX_FMT_SGRBG16:
	case V4L2_PIX_FMT_SRGGB16:
	case V4L2_PIX_FMT_Y16:
		mask = CSI_WRDDR_TYPE_RAW_UNCOMPACT;
		break;
	default:
		mask = CSI_WRDDR_TYPE_RAW_COMPACT;
		break;
	}
	return mask;
}

static int rkcif_csi_get_output_type_mask_rk3576(struct rkcif_stream *stream)
{
	unsigned int mask;
	const struct cif_output_fmt *fmt = stream->cif_fmt_out;

	switch (fmt->fourcc) {
	case V4L2_PIX_FMT_NV16:
		mask = CSI_WRDDR_TYPE_YUV422SP_RK3588 << 3 | CSI_YUV_OUTPUT_ORDER_UYVY >> 4;
		break;
	case V4L2_PIX_FMT_NV61:
		mask = CSI_WRDDR_TYPE_YUV422SP_RK3588 << 3 | CSI_YUV_OUTPUT_ORDER_VYUY >> 4;
		break;
	case V4L2_PIX_FMT_NV12:
		mask = CSI_WRDDR_TYPE_YUV420SP_RK3588 << 3 | CSI_YUV_OUTPUT_ORDER_UYVY >> 4;
		break;
	case V4L2_PIX_FMT_NV21:
		mask = CSI_WRDDR_TYPE_YUV420SP_RK3588 << 3 | CSI_YUV_OUTPUT_ORDER_VYUY >> 4;
		break;
	case V4L2_PIX_FMT_YUYV:
		mask = CSI_WRDDR_TYPE_YUV_PACKET << 3 | CSI_YUV_OUTPUT_ORDER_YUYV >> 4;
		break;
	case V4L2_PIX_FMT_YVYU:
		mask = CSI_WRDDR_TYPE_YUV_PACKET << 3 | CSI_YUV_OUTPUT_ORDER_YVYU >> 4;
		break;
	case V4L2_PIX_FMT_UYVY:
		mask = CSI_WRDDR_TYPE_YUV_PACKET << 3 | CSI_YUV_OUTPUT_ORDER_UYVY >> 4;
		break;
	case V4L2_PIX_FMT_VYUY:
		mask = CSI_WRDDR_TYPE_YUV_PACKET << 3 | CSI_YUV_OUTPUT_ORDER_VYUY >> 4;
		break;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_BGR666:
		mask = CSI_WRDDR_TYPE_RAW_COMPACT << 3;
		break;
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_Y10:
	case V4L2_PIX_FMT_Y12:
		if (stream->is_compact)
			mask = CSI_WRDDR_TYPE_RAW_COMPACT << 3;
		else
			mask = CSI_WRDDR_TYPE_RAW_UNCOMPACT << 3;
		break;
	case V4L2_PIX_FMT_SBGGR16:
	case V4L2_PIX_FMT_SGBRG16:
	case V4L2_PIX_FMT_SGRBG16:
	case V4L2_PIX_FMT_SRGGB16:
	case V4L2_PIX_FMT_Y16:
		mask = CSI_WRDDR_TYPE_RAW_UNCOMPACT << 3;
		break;
	default:
		mask = CSI_WRDDR_TYPE_RAW_COMPACT << 3;
		break;
	}
	return mask;
}

static int rkcif_lvds_get_output_type_mask(struct rkcif_stream *stream)
{
	unsigned int mask;
	const struct cif_output_fmt *fmt = stream->cif_fmt_out;
	int wr_type_offset = 0;
	int yuvout_offset = 0;

	if (stream->cifdev->chip_id == CHIP_RV1106_CIF) {
		wr_type_offset = 17;
		yuvout_offset = 9;
	}

	switch (fmt->fourcc) {
	case V4L2_PIX_FMT_NV16:
		mask = (CSI_WRDDR_TYPE_YUV422SP_RK3588 << wr_type_offset) |
			(CSI_YUV_OUTPUT_ORDER_UYVY << yuvout_offset);
		break;
	case V4L2_PIX_FMT_NV61:
		mask = (CSI_WRDDR_TYPE_YUV422SP_RK3588 << wr_type_offset) |
			(CSI_YUV_OUTPUT_ORDER_VYUY << yuvout_offset);
		break;
	case V4L2_PIX_FMT_NV12:
		mask = (CSI_WRDDR_TYPE_YUV420SP_RK3588 << wr_type_offset) |
			(CSI_YUV_OUTPUT_ORDER_UYVY << yuvout_offset);
		break;
	case V4L2_PIX_FMT_NV21:
		mask = (CSI_WRDDR_TYPE_YUV420SP_RK3588 << wr_type_offset) |
			(CSI_YUV_OUTPUT_ORDER_VYUY << yuvout_offset);
		break;
	case V4L2_PIX_FMT_YUYV:
		mask = (CSI_WRDDR_TYPE_YUV_PACKET << wr_type_offset) |
			(CSI_YUV_OUTPUT_ORDER_YUYV << yuvout_offset);
		break;
	case V4L2_PIX_FMT_YVYU:
		mask = (CSI_WRDDR_TYPE_YUV_PACKET << wr_type_offset) |
			(CSI_YUV_OUTPUT_ORDER_YVYU << yuvout_offset);
		break;
	case V4L2_PIX_FMT_UYVY:
		mask = (CSI_WRDDR_TYPE_YUV_PACKET << wr_type_offset) |
			(CSI_YUV_OUTPUT_ORDER_UYVY << yuvout_offset);
		break;
	case V4L2_PIX_FMT_VYUY:
		mask = (CSI_WRDDR_TYPE_YUV_PACKET << wr_type_offset) |
			(CSI_YUV_OUTPUT_ORDER_VYUY << yuvout_offset);
		break;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_BGR666:
		mask = CSI_WRDDR_TYPE_RAW_COMPACT << wr_type_offset;
		break;
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_Y10:
	case V4L2_PIX_FMT_Y12:
		if (stream->is_compact)
			mask = CSI_WRDDR_TYPE_RAW_COMPACT << wr_type_offset;
		else
			mask = CSI_WRDDR_TYPE_RAW_UNCOMPACT << wr_type_offset;
		break;
	case V4L2_PIX_FMT_SBGGR16:
	case V4L2_PIX_FMT_SGBRG16:
	case V4L2_PIX_FMT_SGRBG16:
	case V4L2_PIX_FMT_SRGGB16:
	case V4L2_PIX_FMT_Y16:
		mask = CSI_WRDDR_TYPE_RAW_UNCOMPACT << wr_type_offset;
		break;
	default:
		mask = CSI_WRDDR_TYPE_RAW_COMPACT << wr_type_offset;
		break;
	}
	return mask;
}

static void rkcif_modify_frame_skip_config(struct rkcif_stream *stream)
{
	if (stream->skip_info.skip_to_en) {
		if (stream->cifdev->chip_id >= CHIP_RV1126B_CIF) {
			rkcif_enable_skip_frame_rv1126b(stream,
							stream->skip_info.cap_m,
							stream->skip_info.skip_n);
		} else {
			rkcif_disable_skip_frame(stream);
			rkcif_enable_skip_frame(stream,
						stream->skip_info.cap_m,
						stream->skip_info.skip_n);
		}
		stream->skip_info.skip_to_en = false;
	} else if (stream->skip_info.skip_to_dis) {
		rkcif_disable_skip_frame(stream);
	}
}

static u32 rkcif_get_parse_type_rk3576(const struct cif_input_fmt *cif_fmt_in,
				       struct csi_channel_info *channel)
{
	u32 parse_type = 0;

	switch (channel->csi_fmt_val) {
	case CSI_WRDDR_TYPE_RAW8:
		parse_type = CSI_WRDDR_TYPE_RAW8 << 3;
		break;
	case CSI_WRDDR_TYPE_RAW10:
		parse_type = CSI_WRDDR_TYPE_RAW10 << 3;
		break;
	case CSI_WRDDR_TYPE_RAW12:
		parse_type = CSI_WRDDR_TYPE_RAW12 << 3;
		break;
	case CSI_WRDDR_TYPE_RAW14_RK3588:
		parse_type = CSI_WRDDR_TYPE_RAW14_RK3588 << 4;
		break;
	case CSI_WRDDR_TYPE_RGB888:
		parse_type = CSI_WRDDR_TYPE_RGB888_RK3576;
		break;
	case CSI_WRDDR_TYPE_YUV422:
		if (cif_fmt_in->field == V4L2_FIELD_NONE)
			parse_type = CSI_WRDDR_TYPE_YUV422 << 4;
		else
			parse_type = (CSI_WRDDR_TYPE_YUV422 + 1) << 4;
		break;
	case CSI_WRDDR_TYPE_YUV420SP:
		parse_type = CSI_WRDDR_TYPE_YUV420SP << 4;
		break;
	case CSI_WRDDR_TYPE_YUV420LEGACY:
		parse_type = CSI_WRDDR_TYPE_YUV420LEGACY;
		break;
	case CSI_WRDDR_TYPE_RAW16:
		parse_type = CSI_WRDDR_TYPE_RAW16;
		break;
	default:
		break;
	}
	return parse_type;
}

static u32 rkcif_get_split_dphy_mask_rk3576(struct rkcif_device *dev)
{
	u32 val = 0;
	int i = 0;
	bool is_split = false;

	switch (dev->csi_host_idx) {
	case 0:
		break;
	case 1:
	case 2:
		for (i = 0; i < dev->hw_dev->dev_num; i++) {
			if (dev->hw_dev->cif_dev[i]->csi_host_idx == 2) {
				is_split = true;
				break;
			}
		}
		if (is_split)
			val = SW_DPHY1_SPLIT_EN_RK3576;
		break;
	case 3:
	case 4:
		for (i = 0; i < dev->hw_dev->dev_num; i++) {
			if (dev->hw_dev->cif_dev[i]->csi_host_idx == 4) {
				is_split = true;
				break;
			}
		}
		if (is_split)
			val = SW_DPHY2_SPLIT_EN_RK3576;
		break;
	default:
		break;
	}
	return val;
}

static u32 rkcif_get_split_dphy_mask_rv1103b(struct rkcif_device *dev)
{
	u32 val = 0;

	if (dev->hw_dev->dev_num >= 2)
		val = SW_DPHY_SPLIT_EN_RV1103B;
	return val;
}

static u32 rkcif_get_split_mask_rv1126b(struct rkcif_device *dev)
{
	u32 val = 0;
	int i = 0;
	bool is_split = false;

	switch (dev->csi_host_idx) {
	case 0:
		for (i = 0; i < dev->hw_dev->dev_num; i++) {
			if (dev->hw_dev->cif_dev[i]->csi_host_idx == 1) {
				is_split = true;
				break;
			}
		}
		if (is_split)
			val = MIPI0_WORK_RV1126B | MIPI1_WORK_RV1126B;
		else
			val = MIPI0_WORK_RV1126B;
		break;
	case 1:
		for (i = 0; i < dev->hw_dev->dev_num; i++) {
			if (dev->hw_dev->cif_dev[i]->csi_host_idx == 0) {
				is_split = true;
				break;
			}
		}
		if (is_split)
			val = MIPI0_WORK_RV1126B | MIPI1_WORK_RV1126B;
		else
			val = MIPI1_WORK_RV1126B;
		break;
	case 2:
		for (i = 0; i < dev->hw_dev->dev_num; i++) {
			if (dev->hw_dev->cif_dev[i]->csi_host_idx == 3) {
				is_split = true;
				break;
			}
		}
		if (is_split)
			val = MIPI2_WORK_RV1126B | MIPI3_WORK_RV1126B;
		else
			val = MIPI2_WORK_RV1126B;
		break;
	case 3:
		for (i = 0; i < dev->hw_dev->dev_num; i++) {
			if (dev->hw_dev->cif_dev[i]->csi_host_idx == 2) {
				is_split = true;
				break;
			}
		}
		if (is_split)
			val = MIPI2_WORK_RV1126B | MIPI3_WORK_RV1126B;
		else
			val = MIPI3_WORK_RV1126B;
		break;
	default:
		break;
	}
	return val;
}

/*config reg for rk3588*/
static int rkcif_csi_channel_set_v1(struct rkcif_stream *stream,
				       struct csi_channel_info *channel,
				       enum v4l2_mbus_type mbus_type, unsigned int mode,
				       int index)
{
	unsigned int val = 0x0;
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_stream *detect_stream = &dev->stream[0];
	struct sditf_priv *priv = dev->sditf[0];
	struct rkmodule_capture_info *capture_info = &channel->capture_info;
	unsigned int wait_line = 0x3fff;
	unsigned int dma_en = 0;
	int offset = 0;

	if (channel->id >= 4)
		return -EINVAL;

	if (!channel->enable) {
		rkcif_write_register(dev, get_reg_index_of_id_ctrl0(channel->id),
				     CSI_DISABLE_CAPTURE);
		return 0;
	}

	val = rkcif_read_register(dev, CIF_REG_GLB_CTRL);
	if (stream->sw_dbg_en) {
		val &= ~BIT(16);
		v4l2_subdev_call(dev->active_sensor->sd,
				 core, ioctl,
				 RKCIF_CMD_SET_PPI_DATA_DEBUG,
				 &stream->sw_dbg_en);
	} else {
		v4l2_subdev_call(dev->active_sensor->sd,
				 core, ioctl,
				 RKCIF_CMD_SET_PPI_DATA_DEBUG,
				 &stream->sw_dbg_en);
	}
	if (dev->chip_id == CHIP_RK3588_CIF ||
	    dev->chip_id == CHIP_RV1106_CIF ||
	    dev->chip_id == CHIP_RK3562_CIF) {
		val |= GLB_RESET_IDI_EN_RK3588;
	} else if (dev->chip_id == CHIP_RK3576_CIF) {
		val |= GLB_RESET_IDI_EN_RK3576;
		val |= rkcif_get_split_dphy_mask_rk3576(dev);
	} else if (dev->chip_id == CHIP_RV1103B_CIF) {
		val |= rkcif_get_split_dphy_mask_rv1103b(dev);
	}
	if (dev->chip_id == CHIP_RV1103B_CIF) {
		if (rkcif_frm_toisp_protect)
			val &= ~BIT(28);
		else
			val |= BIT(28);
	}
	rkcif_write_register(dev, CIF_REG_GLB_CTRL, val);

	if (dev->terminal_sensor.hdmi_input_en) {
		if (dev->chip_id == CHIP_RK3562_CIF ||
		    dev->chip_id == CHIP_RK3576_CIF)
			rkcif_write_register_and(dev, CIF_REG_GLB_CTRL, ~(u32)BIT(16));
	}

	rkcif_write_register_and(dev, CIF_REG_MIPI_LVDS_INTSTAT,
				 ~(CSI_START_INTSTAT(channel->id) |
				 CSI_DMA_END_INTSTAT(channel->id) |
				 CSI_LINE_INTSTAT_V1(channel->id)));

	if (!(capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE &&
	      index < capture_info->multi_dev.dev_num - 1)) {

		rkcif_write_register_or(dev, CIF_REG_MIPI_LVDS_INTEN,
				dev->chip_id < CHIP_RK3576_CIF ?
				CSI_START_INTEN(channel->id) :
				CSI_START_INTEN_RK3576(channel->id));
		if (dev->chip_id >= CHIP_RK3576_CIF)
			rkcif_write_register_or(dev, CIF_REG_MIPI_LVDS_INTEN,
				CSI_INF_END_INTEN_RK3576(channel->id));

		if ((!priv || (priv && priv->mode.rdbk_mode >= RKISP_VICAP_RDBK_AIQ)) &&
		    detect_stream->is_line_wake_up) {
			rkcif_write_register_or(dev, CIF_REG_MIPI_LVDS_INTEN,
						CSI_LINE_INTEN_RK3588(channel->id));
			wait_line = dev->wait_line;
		}
		rkcif_write_register(dev, CIF_REG_MIPI_LVDS_LINE_INT_NUM_ID0_1,
				     wait_line << 16 | wait_line);
		rkcif_write_register(dev, CIF_REG_MIPI_LVDS_LINE_INT_NUM_ID2_3,
				     wait_line << 16 | wait_line);

		rkcif_write_register_or(dev, CIF_REG_MIPI_LVDS_INTEN,
					CSI_DMA_END_INTEN(channel->id));
	}
	if (atomic_read(&stream->cifdev->id_use_cnt) == 0) {
		if (dev->chip_id > CHIP_RK3562_CIF) {
			val = CIF_MIPI_LVDS_SW_WATER_LINE_ENABLE |
			      (CIF_MIPI_LVDS_SW_WATER_LINE_25 << 19);
			if (stream->cifdev->hdr.hdr_mode != NO_HDR &&
			    stream->cifdev->hdr.esp.mode == HDR_NORMAL_VC)
				val |= CSI_HDR_VC_MODE_PROTECT >> 25;
			val |= (!!stream->sw_dbg_en) << 31;
			rkcif_write_register(dev, CIF_REG_MIPI_LVDS_CTRL, val);
		} else {
			val = CIF_MIPI_LVDS_SW_PRESS_VALUE_RK3588(0x3) |
				CIF_MIPI_LVDS_SW_PRESS_ENABLE |
				CIF_MIPI_LVDS_SW_HURRY_VALUE_RK3588(0x3) |
				CIF_MIPI_LVDS_SW_HURRY_ENABLE |
				CIF_MIPI_LVDS_SW_WATER_LINE_25 |
				CIF_MIPI_LVDS_SW_WATER_LINE_ENABLE;
			if (mbus_type == V4L2_MBUS_CSI2_DPHY ||
			    mbus_type == V4L2_MBUS_CSI2_CPHY)
				val &= ~CIF_MIPI_LVDS_SW_SEL_LVDS_RV1106;
			else
				val |= CIF_MIPI_LVDS_SW_SEL_LVDS_RV1106;
			rkcif_write_register(dev, CIF_REG_MIPI_LVDS_CTRL, val);
		}

		rkcif_write_register_or(dev, CIF_REG_MIPI_LVDS_INTEN,
					CSI_ALL_ERROR_INTEN_V1);
		rkcif_do_soft_reset(dev);
	}
#if IS_ENABLED(CONFIG_CPU_RV1106)
	if (channel->id == 1)
		rv1106_sdmmc_get_lock();
#endif

	if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE &&
	    priv && priv->mode.rdbk_mode == RKISP_VICAP_ONLINE &&
	    (dev->hdr.hdr_mode == NO_HDR ||
	     (dev->hdr.hdr_mode == HDR_X2 && stream->id == 1) ||
	     (dev->hdr.hdr_mode == HDR_X3 && stream->id == 2)))
		offset = channel->capture_info.multi_dev.pixel_offset;

	if (dev->chip_id < CHIP_RK3576_CIF)
		rkcif_write_register(dev, get_reg_index_of_id_ctrl1(channel->id),
				     (channel->width + offset) | (channel->height << 16));
	else
		rkcif_write_register(dev, CIF_REG_MIPI_SET_SIZE_ID0 + channel->id,
				     (channel->width + offset) | (channel->height << 16));

#if IS_ENABLED(CONFIG_CPU_RV1106)
	if (channel->id == 1)
		rv1106_sdmmc_put_lock();
#endif

	rkcif_write_register(dev, get_reg_index_of_id_crop_start(channel->id),
			     channel->crop_st_y << 16 | channel->crop_st_x);

	if (mode == RKCIF_STREAM_MODE_CAPTURE) {
		if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO)
			rkcif_buf_init_interlace(stream, channel->id);
		else
			rkcif_assign_new_buffer_pingpong(stream,
							 RKCIF_YUV_ADDR_STATE_INIT,
							 channel->id);
	} else if (mode == RKCIF_STREAM_MODE_TOISP ||
		 mode == RKCIF_STREAM_MODE_TOISP_RDBK) {
		rkcif_assign_new_buffer_pingpong_toisp(stream,
						       RKCIF_YUV_ADDR_STATE_INIT,
						       channel->id);
	} else if (mode == RKCIF_STREAM_MODE_ROCKIT) {
		rkcif_assign_new_buffer_pingpong_rockit(stream,
							RKCIF_YUV_ADDR_STATE_INIT,
							channel->id);
	}

	if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE &&
	    index == (capture_info->multi_dev.dev_num - 1) &&
	    priv && priv->mode.rdbk_mode != RKISP_VICAP_ONLINE)
		rkcif_write_register(dev, get_reg_index_of_id_crop_start(channel->id),
				     channel->crop_st_y << 16 |
				     (channel->crop_st_x + capture_info->multi_dev.pixel_offset));

	val = channel->virtual_width;
	if (dev->chip_id >= CHIP_RV1103B_CIF && dev->sditf[0] &&
	    dev->sditf[0]->hdr_wrap_line)
		val |= dev->sditf[0]->hdr_wrap_line << 20;
	rkcif_write_register(dev, get_reg_index_of_frm0_y_vlw(channel->id), val);

	if (stream->lack_buf_cnt == 2)
		stream->dma_en = 0;

	if (stream->dma_en) {
		if (mbus_type == V4L2_MBUS_CSI2_DPHY ||
		    mbus_type == V4L2_MBUS_CSI2_CPHY) {
			if (dev->chip_id > CHIP_RK3562_CIF)
				dma_en = CSI_DMA_ENABLE_RK3576;
			else
				dma_en = CSI_DMA_ENABLE;
		} else {
			dma_en = LVDS_DMAEN_RV1106;
		}
	}
	if (mbus_type == V4L2_MBUS_CSI2_DPHY ||
	    mbus_type == V4L2_MBUS_CSI2_CPHY) {

		if (stream->cifdev->hdr.esp.mode == HDR_LINE_CNT ||
		    stream->cifdev->hdr.esp.mode == HDR_ID_CODE)
			channel->vc = 0;

		if (dev->chip_id > CHIP_RK3562_CIF) {
			val = CSI_ENABLE_CAPTURE | dma_en |
			      CSI_ENABLE_CROP_RK3576;
			val |= rkcif_get_parse_type_rk3576(stream->cif_fmt_in, channel);
			val |= rkcif_csi_get_output_type_mask_rk3576(stream);
			val |= stream->cif_fmt_in->csi_yuv_order >> 4;
			if (stream->is_high_align)
				val |= CSI_HIGH_ALIGN_RK3576;
			else
				val &= ~CSI_HIGH_ALIGN_RK3576;
			if (stream->id == 0 && (stream->cif_fmt_out->fourcc == V4L2_PIX_FMT_NV12 ||
			   stream->cif_fmt_out->fourcc == V4L2_PIX_FMT_NV21))
				val |= CSI_UVDS_EN;

			if (dev->chip_id >= CHIP_RV1103B_CIF && dev->sditf[0] &&
			    dev->sditf[0]->hdr_wrap_line)
				val |= (0x2 << 20);
			if (dev->chip_id >= CHIP_RV1103B_CIF && stream->rounding_bit)
				val |= stream->rounding_bit;

			rkcif_write_register(dev, get_reg_index_of_id_ctrl0(channel->id), val);

			val = channel->vc | channel->data_type << 2;

			if (stream->cifdev->hdr.hdr_mode == NO_HDR ||
			    stream->cifdev->hdr.hdr_mode == HDR_COMPR)
				val |= CSI_NO_HDR >> 12;
			else if (stream->cifdev->hdr.hdr_mode == HDR_X2)
				val |= CSI_HDR2 >> 12;
			else if (stream->cifdev->hdr.hdr_mode == HDR_X3)
				val |= CSI_HDR3 >> 12;
			if (stream->cifdev->hdr.esp.mode == HDR_NORMAL_VC) {
				val |= CSI_HDR_MODE_VC >> 12;
			} else if (stream->cifdev->hdr.esp.mode == HDR_LINE_CNT) {
				val |= CSI_HDR_MODE_LINE_CNT >> 12;
				rkcif_write_register(dev, CIF_REG_MIPI_ON_PAD, 0x4);
				val |= stream->id << 12;
			} else if (stream->cifdev->hdr.esp.mode == HDR_ID_CODE) {
				val |= CSI_HDR_MODE_LINE_INFO >> 12;
			}
			val |= channel->cmd_mode_en << 14;
			rkcif_write_register(dev, get_reg_index_of_id_ctrl1(channel->id), val);
		} else {
			val = CSI_ENABLE_CAPTURE | dma_en |
			      channel->cmd_mode_en << 26 | CSI_ENABLE_CROP_V1 |
			      channel->vc << 8 | channel->data_type << 10;
			if (dev->chip_id >= CHIP_RK3588_CIF) {
				if (channel->csi_fmt_val == CSI_WRDDR_TYPE_RGB888)
					val |= CSI_WRDDR_TYPE_YUV422;
				else if (channel->csi_fmt_val == CSI_WRDDR_TYPE_RAW14_RK3588)
					val |= channel->csi_fmt_val << 1;
				else
					val |= channel->csi_fmt_val;
			} else {
				val |= channel->csi_fmt_val;
			}

			val |= stream->cif_fmt_in->csi_yuv_order;
			val |= rkcif_csi_get_output_type_mask(stream);
			if (stream->cifdev->hdr.hdr_mode == NO_HDR ||
			    stream->cifdev->hdr.hdr_mode == HDR_COMPR)
				val |= CSI_NO_HDR;
			else if (stream->cifdev->hdr.hdr_mode == HDR_X2)
				val |= CSI_HDR2;
			else if (stream->cifdev->hdr.hdr_mode == HDR_X3)
				val |= CSI_HDR3;
			if (stream->cifdev->hdr.esp.mode == HDR_NORMAL_VC)
				val |= CSI_HDR_MODE_VC;
			else if (stream->cifdev->hdr.esp.mode == HDR_LINE_CNT)
				val |= CSI_HDR_MODE_LINE_CNT;
			else if (stream->cifdev->hdr.esp.mode == HDR_ID_CODE)
				val |= CSI_HDR_MODE_LINE_INFO;
			if (stream->cifdev->hdr.hdr_mode != NO_HDR &&
			    stream->cifdev->hdr.esp.mode == HDR_NORMAL_VC)
				val |= CSI_HDR_VC_MODE_PROTECT;
			if (stream->is_high_align)
				val |= CSI_HIGH_ALIGN_RK3588;
			else
				val &= ~CSI_HIGH_ALIGN_RK3588;
			rkcif_write_register(dev, get_reg_index_of_id_ctrl0(channel->id), val);
		}
		rkcif_write_register(dev, CIF_REG_MIPI_EFFECT_CODE_ID0, 0x02410251);
		rkcif_write_register(dev, CIF_REG_MIPI_EFFECT_CODE_ID1, 0x02420252);
	} else if (mbus_type == V4L2_MBUS_CCP2) {
		rkcif_csi_set_lvds_sav_eav(stream, channel);
		val = LVDS_ENABLE_CAPTURE_RV1106 | LVDS_MODE_RV1106(channel->lvds_cfg.mode) |
		      LVDS_MAIN_LANE_RV1106(0) | LVDS_FID_RV1106(0) |
		      LVDS_LANES_ENABLED_RV1106(dev->active_sensor->lanes) |
		      (channel->csi_fmt_val << 18) |
		      rkcif_lvds_get_output_type_mask(stream) |
		      (stream->cif_fmt_in->csi_yuv_order << 9) |
		      dma_en;
		if (stream->cifdev->hdr.hdr_mode == HDR_X3)
			val |= BIT(12);
		rkcif_write_register(dev, get_reg_index_of_lvds_id_ctrl0(channel->id), val);
	}
	if (dev->chip_id >= CHIP_RV1106_CIF)
		rkcif_modify_frame_skip_config(stream);
	if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
		if (index == (capture_info->multi_dev.dev_num - 1))
			atomic_inc(&stream->cifdev->id_use_cnt);
	} else {
		atomic_inc(&stream->cifdev->id_use_cnt);
	}
	if (!(capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE &&
	      index < capture_info->multi_dev.dev_num - 1)) {
		if (mode == RKCIF_STREAM_MODE_CAPTURE)
			rkcif_assign_new_buffer_pingpong(stream,
						 RKCIF_YUV_ADDR_STATE_INIT,
						 channel->id);
		else if (mode == RKCIF_STREAM_MODE_TOISP ||
			 mode == RKCIF_STREAM_MODE_TOISP_RDBK)
			rkcif_assign_new_buffer_pingpong_toisp(stream,
						       RKCIF_YUV_ADDR_STATE_INIT,
						       channel->id);
	}
	dev->intr_mask = rkcif_read_register(dev, CIF_REG_MIPI_LVDS_INTEN);
	return 0;
}

/*config reg for rv1126b*/
static int rkcif_csi_channel_set_rv1126b(struct rkcif_stream *stream,
				       struct csi_channel_info *channel,
				       enum v4l2_mbus_type mbus_type, unsigned int mode,
				       int index)
{
	unsigned int val = 0x0;
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_stream *detect_stream = &dev->stream[0];
	struct sditf_priv *priv = dev->sditf[0];
	struct rkmodule_capture_info *capture_info = &channel->capture_info;
	unsigned int wait_line = 0x3fff;
	unsigned int dma_en = 0;
	int offset = 0;

	if (channel->id >= 4)
		return -EINVAL;

	if (!channel->enable) {
		rkcif_write_register(dev, get_reg_index_of_id_ctrl0(channel->id),
				     CSI_DISABLE_CAPTURE);
		return 0;
	}
	val = rkcif_read_register(dev, CIF_REG_GLB_CTRL);
	val |= rkcif_get_split_mask_rv1126b(dev);
	if (rkcif_frm_toisp_protect)
		val &= ~BIT(28);
	else
		val |= BIT(28);
	rkcif_write_register(dev, CIF_REG_GLB_CTRL, val);
	rkcif_write_register_and(dev, CIF_REG_MIPI_LVDS_INTSTAT,
				 ~(CSI_START_INTSTAT(channel->id) |
				 CSI_DMA_END_INTSTAT(channel->id) |
				 CSI_LINE_INTSTAT_V1(channel->id)));

	if (!(capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE &&
	      index < capture_info->multi_dev.dev_num - 1)) {

		rkcif_write_register_or(dev, CIF_REG_MIPI_LVDS_INTEN,
					CSI_START_INTEN_RK3576(channel->id));

		if (priv && priv->mode.rdbk_mode >= RKISP_VICAP_RDBK_AIQ && detect_stream->is_line_wake_up) {
			rkcif_write_register_or(dev, CIF_REG_MIPI_LVDS_INTEN,
						CSI_LINE_INTEN_RK3588(channel->id));
			wait_line = dev->wait_line;
		}
		rkcif_write_register(dev, CIF_REG_MIPI_LVDS_LINE_INT_NUM_ID0_1,
				     wait_line << 16 | wait_line);
		rkcif_write_register(dev, CIF_REG_MIPI_LVDS_LINE_INT_NUM_ID2_3,
				     wait_line << 16 | wait_line);

		rkcif_write_register_or(dev, CIF_REG_MIPI_LVDS_INTEN,
					CSI_DMA_END_INTEN(channel->id));
	}
	if (atomic_read(&stream->cifdev->id_use_cnt) == 0) {
		val = (CIF_MIPI_LVDS_SW_WATER_LINE_25 << 19) |
		      (dev->csi_host_idx << 1) | CSI_ENABLE_CAPTURE;
		if (stream->sw_dbg_en) {
			val |= (!!stream->sw_dbg_en) << 31;
			v4l2_subdev_call(dev->active_sensor->sd,
					 core, ioctl,
					 RKCIF_CMD_SET_PPI_DATA_DEBUG,
					 &stream->sw_dbg_en);
		} else {
			v4l2_subdev_call(dev->active_sensor->sd,
					 core, ioctl,
					 RKCIF_CMD_SET_PPI_DATA_DEBUG,
					 &stream->sw_dbg_en);
		}
		if (mbus_type == V4L2_MBUS_CCP2) {
			val |= LVDS_PATH_EN_RV1126B;
			val |= (dev->active_sensor->lanes - 1) << 5;
		}
		rkcif_write_register(dev, CIF_REG_MIPI_LVDS_CTRL, val);

		rkcif_write_register_or(dev, CIF_REG_MIPI_LVDS_INTEN,
					CSI_ALL_ERROR_INTEN_V1);
		rkcif_do_soft_reset(dev);
	}

	if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE &&
	    priv && priv->mode.rdbk_mode == RKISP_VICAP_ONLINE &&
	    (dev->hdr.hdr_mode == NO_HDR ||
	     (dev->hdr.hdr_mode == HDR_X2 && stream->id == 1) ||
	     (dev->hdr.hdr_mode == HDR_X3 && stream->id == 2)))
		offset = channel->capture_info.multi_dev.pixel_offset;

	rkcif_write_register(dev, CIF_REG_MIPI_SET_SIZE_ID0 + channel->id,
			     (channel->width + offset) | (channel->height << 16));

	rkcif_write_register(dev, get_reg_index_of_id_crop_start(channel->id),
			     channel->crop_st_y << 16 | channel->crop_st_x);

	if (mode == RKCIF_STREAM_MODE_CAPTURE) {
		if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO)
			rkcif_buf_init_interlace(stream, channel->id);
		else
			rkcif_assign_new_buffer_pingpong(stream,
							 RKCIF_YUV_ADDR_STATE_INIT,
							 channel->id);
	} else if (mode == RKCIF_STREAM_MODE_TOISP ||
		 mode == RKCIF_STREAM_MODE_TOISP_RDBK) {
		rkcif_assign_new_buffer_pingpong_toisp(stream,
						       RKCIF_YUV_ADDR_STATE_INIT,
						       channel->id);
	} else if (mode == RKCIF_STREAM_MODE_ROCKIT) {
		rkcif_assign_new_buffer_pingpong_rockit(stream,
							RKCIF_YUV_ADDR_STATE_INIT,
							channel->id);
	}

	if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE &&
	    index == (capture_info->multi_dev.dev_num - 1) &&
	    priv && priv->mode.rdbk_mode != RKISP_VICAP_ONLINE)
		rkcif_write_register(dev, get_reg_index_of_id_crop_start(channel->id),
				     channel->crop_st_y << 16 |
				     (channel->crop_st_x + capture_info->multi_dev.pixel_offset));

	val = channel->virtual_width;
	if (dev->sditf[0] &&
	    dev->sditf[0]->hdr_wrap_line)
		val |= dev->sditf[0]->hdr_wrap_line << 20;
	rkcif_write_register(dev, get_reg_index_of_frm0_y_vlw(channel->id), val);

	if (stream->lack_buf_cnt == 2)
		stream->dma_en = 0;

	if (stream->dma_en) {
		if (mbus_type == V4L2_MBUS_CSI2_DPHY ||
		    mbus_type == V4L2_MBUS_CSI2_CPHY ||
		    dev->chip_id >= CHIP_RV1126B_CIF) {
			dma_en = CSI_DMA_ENABLE_RK3576;
		} else {
			dma_en = LVDS_DMAEN_RV1106;
		}
	}

	if (stream->cifdev->hdr.esp.mode == HDR_LINE_CNT ||
	    stream->cifdev->hdr.esp.mode == HDR_ID_CODE)
		channel->vc = 0;

	val = CSI_ENABLE_CAPTURE | dma_en |
	      CSI_ENABLE_CROP_RK3576;
	val |= rkcif_get_parse_type_rk3576(stream->cif_fmt_in, channel);
	val |= rkcif_csi_get_output_type_mask_rk3576(stream);
	val |= stream->cif_fmt_in->csi_yuv_order >> 4;
	if (stream->is_high_align)
		val |= CSI_HIGH_ALIGN_RK3576;
	else
		val &= ~CSI_HIGH_ALIGN_RK3576;
	if (stream->id == 0 && (stream->cif_fmt_out->fourcc == V4L2_PIX_FMT_NV12 ||
	   stream->cif_fmt_out->fourcc == V4L2_PIX_FMT_NV21))
		val |= CSI_UVDS_EN;

	if (dev->sditf[0] &&
	    dev->sditf[0]->hdr_wrap_line)
		val |= (0x2 << 20);
	if (stream->rounding_bit)
		val |= stream->rounding_bit;
	if (!dev->terminal_sensor.hdmi_input_en)
		val |= DMA_ADAPT_EN_RV1126B;

	rkcif_write_register(dev, get_reg_index_of_id_ctrl0(channel->id), val);

	val = channel->vc | channel->data_type << 4;

	if (stream->cifdev->hdr.hdr_mode == NO_HDR ||
	    stream->cifdev->hdr.hdr_mode == HDR_COMPR)
		val |= CSI_NO_HDR >> 10;
	else if (stream->cifdev->hdr.hdr_mode == HDR_X2)
		val |= CSI_HDR2 >> 10;
	else if (stream->cifdev->hdr.hdr_mode == HDR_X3)
		val |= CSI_HDR3 >> 10;
	if (stream->cifdev->hdr.esp.mode == HDR_NORMAL_VC) {
		val |= CSI_HDR_MODE_VC >> 10;
	} else if (stream->cifdev->hdr.esp.mode == HDR_LINE_CNT) {
		val |= CSI_HDR_MODE_LINE_CNT >> 10;
		rkcif_write_register(dev, CIF_REG_MIPI_ON_PAD, 0x4);
		val |= stream->id << 10;
	} else if (stream->cifdev->hdr.esp.mode == HDR_ID_CODE) {
		val |= CSI_HDR_MODE_LINE_INFO >> 10;
	}
	val |= channel->cmd_mode_en << 16;
	if (mbus_type == V4L2_MBUS_CCP2) {
		rkcif_csi_set_lvds_sav_eav(stream, channel);
		val |= LVDS_MODE_RV1126B(channel->lvds_cfg.mode);
		val |= LVDS_FID_RV1126B(0);
	}
	rkcif_write_register(dev, get_reg_index_of_id_ctrl1(channel->id), val);
	rkcif_write_register(dev, CIF_REG_MIPI_EFFECT_CODE_ID0, 0x02410251);
	rkcif_write_register(dev, CIF_REG_MIPI_EFFECT_CODE_ID1, 0x02420252);

	rkcif_modify_frame_skip_config(stream);
	if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
		if (index == (capture_info->multi_dev.dev_num - 1))
			atomic_inc(&stream->cifdev->id_use_cnt);
	} else {
		atomic_inc(&stream->cifdev->id_use_cnt);
	}
	if (!(capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE &&
	      index < capture_info->multi_dev.dev_num - 1)) {
		if (mode == RKCIF_STREAM_MODE_CAPTURE)
			rkcif_assign_new_buffer_pingpong(stream,
						 RKCIF_YUV_ADDR_STATE_INIT,
						 channel->id);
		else if (mode == RKCIF_STREAM_MODE_TOISP ||
			 mode == RKCIF_STREAM_MODE_TOISP_RDBK)
			rkcif_assign_new_buffer_pingpong_toisp(stream,
						       RKCIF_YUV_ADDR_STATE_INIT,
						       channel->id);
	}
	dev->intr_mask = rkcif_read_register(dev, CIF_REG_MIPI_LVDS_INTEN);
	return 0;
}

void rkcif_reinit_right_half_config(struct rkcif_stream *stream)
{
	struct rkcif_device *cif_dev = stream->cifdev;
	struct csi_channel_info *channel = &cif_dev->channels[0];
	const struct cif_output_fmt *fmt;

	fmt = rkcif_find_output_fmt(&cif_dev->stream[0], cif_dev->stream[0].pixm.pixelformat);
	if (!fmt) {
		v4l2_err(&cif_dev->v4l2_dev, "can not find output format: 0x%x",
			 cif_dev->stream[0].pixm.pixelformat);
		return;
	}
	channel->crop_st_x += channel->width / 2;
	channel->crop_st_x -= cif_dev->unite_extend_pixel;
	channel->width /= 2;
	channel->width += cif_dev->unite_extend_pixel;
	channel->virtual_width = ALIGN(channel->width * fmt->raw_bpp / 8, 256);
	if (cif_dev->chip_id < CHIP_RK3576_CIF)
		rkcif_write_register(cif_dev, get_reg_index_of_id_ctrl1(channel->id),
				     channel->width | (channel->height << 16));
	else
		rkcif_write_register(cif_dev, CIF_REG_MIPI_SET_SIZE_ID0 + channel->id,
				     channel->width | (channel->height << 16));
	rkcif_write_register(cif_dev, get_reg_index_of_id_crop_start(channel->id),
			     channel->crop_st_y << 16 | channel->crop_st_x);
	rkcif_write_register_and(cif_dev, CIF_REG_MIPI_LVDS_INTEN,
				 ~(CSI_DMA_END_INTEN(stream->id)));
	stream->to_en_dma = RKCIF_DMAEN_BY_ISP;
	rkcif_enable_dma_capture(stream, false);
}

static int rkcif_csi_stream_start(struct rkcif_stream *stream, unsigned int mode)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_sensor_info *active_sensor = dev->active_sensor;
	enum v4l2_mbus_type mbus_type = active_sensor->mbus.type;
	struct csi_channel_info *channel;
	int isp_num = dev->channels[0].capture_info.one_to_multi.isp_num;
	struct rkcif_pipeline *p = &dev->pipe;
	u32 ret = 0;
	int i;

	if (stream->state < RKCIF_STATE_STREAMING) {
		if (!dev->is_thunderboot)
			stream->frame_idx = 0;
		stream->buf_wake_up_cnt = 0;
		stream->frame_phase = 0;
		stream->lack_buf_cnt = 0;
		stream->is_in_vblank = false;
		stream->is_change_toisp = false;
		stream->is_finish_single_cap = true;
		stream->is_wait_single_cap = false;
		stream->last_frame_idx = 0;
		stream->frame_phase_cache = CIF_CSI_FRAME1_READY;
	}
	stream->interlaced_bad_frame = false;
	stream->last_fs_interlaced_phase = 0;
	stream->last_fe_interlaced_phase = 0;
	stream->to_stop_dma = 0;

	channel = &dev->channels[stream->id];
	channel->id = stream->id;
	if (mbus_type == V4L2_MBUS_CCP2) {
		ret = v4l2_subdev_call(dev->terminal_sensor.sd, core,
				       ioctl, RKMODULE_GET_LVDS_CFG,
				       &channel->lvds_cfg);
		if (ret) {
			v4l2_err(&dev->v4l2_dev, "Err: get lvds config failed!!\n");
			return ret;
		}
	}
	rkcif_csi_channel_init(stream, channel);

	if (dev->channels[0].capture_info.mode == RKMODULE_ONE_CH_TO_MULTI_ISP) {
		if (atomic_read(&p->stream_cnt) < isp_num - 1)
			goto one_to_multi_skip;
		else
			*channel = dev->channels[0];
	}

	if (stream->state != RKCIF_STATE_STREAMING) {
		if (mode  == RKCIF_STREAM_MODE_CAPTURE) {
			stream->dma_en |= RKCIF_DMAEN_BY_VICAP;
		} else if (mode == RKCIF_STREAM_MODE_TOISP_RDBK) {
			stream->dma_en |= RKCIF_DMAEN_BY_ISP;
		} else if (mode == RKCIF_STREAM_MODE_TOISP) {
			if (dev->hdr.hdr_mode == HDR_X2 &&
			    stream->id == 0)
				stream->dma_en |= RKCIF_DMAEN_BY_ISP;
			else if (dev->hdr.hdr_mode == HDR_X3 && (stream->id == 0 || stream->id == 1))
				stream->dma_en |= RKCIF_DMAEN_BY_ISP;
			else if (dev->sditf[0]->mode.rdbk_mode == RKISP_VICAP_ONLINE_UNITE)
				stream->dma_en |= RKCIF_DMAEN_BY_ISP;
		} else if (mode == RKCIF_STREAM_MODE_ROCKIT) {
			stream->dma_en |= RKCIF_DMAEN_BY_ROCKIT;
		}
		if (stream->cifdev->chip_id < CHIP_RK3588_CIF) {
			rkcif_csi_channel_set(stream, channel, mbus_type);
		} else if (stream->cifdev->chip_id < CHIP_RV1126B_CIF) {
			if (channel->capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
				for (i = 0; i < channel->capture_info.multi_dev.dev_num; i++) {
					dev->csi_host_idx = channel->capture_info.multi_dev.dev_idx[i];
					rkcif_csi_channel_set_v1(stream, channel, mbus_type, mode, i);
				}
			} else {
				if (!dev->switch_info.is_use_switch ||
				    atomic_inc_return(&dev->hw_dev->switch_stream_cnt[dev->switch_info.host_idx]) == 1)
					rkcif_csi_channel_set_v1(stream, channel, mbus_type, mode, 0);
			}
		} else {
			rkcif_csi_channel_set_rv1126b(stream, channel, mbus_type, mode, 0);
		}
	} else {
		if (stream->cifdev->chip_id >= CHIP_RK3588_CIF) {
			if (mode == RKCIF_STREAM_MODE_CAPTURE) {
				stream->to_en_dma = RKCIF_DMAEN_BY_VICAP;
			} else if (mode == RKCIF_STREAM_MODE_TOISP_RDBK) {
				stream->to_en_dma = RKCIF_DMAEN_BY_ISP;
			} else if (mode == RKCIF_STREAM_MODE_TOISP) {
				if (dev->hdr.hdr_mode == HDR_X2 &&
				    stream->id == 0 &&
				    (!stream->dma_en))
					stream->to_en_dma = RKCIF_DMAEN_BY_ISP;
				else if (dev->hdr.hdr_mode == HDR_X3 &&
					 (stream->id == 0 || stream->id == 1) &&
					 (!stream->dma_en))
					stream->to_en_dma = RKCIF_DMAEN_BY_ISP;
			} else if (mode == RKCIF_STREAM_MODE_ROCKIT) {
				stream->to_en_dma = RKCIF_DMAEN_BY_ROCKIT;
			}
		}
	}
one_to_multi_skip:
	if (stream->state != RKCIF_STATE_STREAMING) {
		stream->line_int_cnt = 0;
		if (stream->is_line_wake_up)
			stream->is_can_stop = false;
		else
			stream->is_can_stop = true;
		stream->state = RKCIF_STATE_STREAMING;
		dev->workmode = RKCIF_WORKMODE_PINGPONG;
	}

	return 0;
}

static void rkcif_stream_stop(struct rkcif_stream *stream)
{
	struct rkcif_device *cif_dev = stream->cifdev;
	struct v4l2_mbus_config *mbus_cfg = &cif_dev->active_sensor->mbus;
	u32 val;
	int id;
	int i = 0;
	int ret = 0;

	atomic_dec_if_positive(&stream->cifdev->id_use_cnt);
	if (cif_dev->switch_info.is_use_switch) {
		ret = atomic_dec_if_positive(&cif_dev->hw_dev->switch_stream_cnt[cif_dev->switch_info.host_idx]);
		if (ret) {
			stream->state = RKCIF_STATE_READY;
			stream->dma_en = 0;
			return;
		}
	}

	if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY ||
	    mbus_cfg->type == V4L2_MBUS_CCP2) {
		id = stream->id;
		val = rkcif_read_register(cif_dev, get_reg_index_of_id_ctrl0(id));
		if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
		    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY) {
			if (cif_dev->chip_id >= CHIP_RK3576_CIF)
				val &= ~(CSI_ENABLE_CAPTURE | CSI_DMA_ENABLE_RK3576);
			else
				val &= ~(CSI_ENABLE_CAPTURE | CSI_DMA_ENABLE);
		} else {
			val &= ~LVDS_ENABLE_CAPTURE;
		}

		if (cif_dev->channels[id].capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
			for (i = 0; i < cif_dev->channels[id].capture_info.multi_dev.dev_num; i++) {
				cif_dev->csi_host_idx = cif_dev->channels[id].capture_info.multi_dev.dev_idx[i];
				rkcif_write_register(cif_dev, get_reg_index_of_id_ctrl0(id), val);
			}
		} else {
			rkcif_write_register(cif_dev, get_reg_index_of_id_ctrl0(id), val);
		}

		val = CSI_DMA_END_INTSTAT(id);
		if (cif_dev->chip_id >= CHIP_RK3576_CIF)
			val |= CSI_START_INTSTAT_RK3576(id);
		else
			val |= CSI_START_INTSTAT(id);
		if (cif_dev->chip_id >= CHIP_RK3588_CIF)
			val |= CSI_LINE_INTSTAT_V1(id);
		else
			val |= CSI_LINE_INTSTAT(id);
		rkcif_write_register_or(cif_dev, CIF_REG_MIPI_LVDS_INTSTAT, val);

		val = CSI_DMA_END_INTEN(id);
		if (cif_dev->chip_id >= CHIP_RK3576_CIF)
			val |= CSI_START_INTEN_RK3576(id);
		else
			val |= CSI_START_INTEN(id);
		if (cif_dev->chip_id >= CHIP_RK3588_CIF)
			val |= CSI_LINE_INTEN_RK3588(id);
		else
			val |= CSI_LINE_INTEN(id);
		rkcif_write_register_and(cif_dev, CIF_REG_MIPI_LVDS_INTEN, ~val);

		if (stream->cifdev->chip_id < CHIP_RK3588_CIF) {
			rkcif_write_register_and(cif_dev, CIF_REG_MIPI_LVDS_INTEN,
						 ~CSI_ALL_ERROR_INTEN);
		} else {
			if (atomic_read(&stream->cifdev->id_use_cnt) == 0) {
				rkcif_write_register_and(cif_dev, CIF_REG_MIPI_LVDS_INTEN,
						~CSI_ALL_ERROR_INTEN_V1);
				if (cif_dev->channels[id].capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
					for (i = 0; i < cif_dev->channels[id].capture_info.multi_dev.dev_num; i++) {
						cif_dev->csi_host_idx = cif_dev->channels[id].capture_info.multi_dev.dev_idx[i];
						rkcif_write_register_and(cif_dev, CIF_REG_MIPI_LVDS_CTRL,
									 ~CSI_ENABLE_CAPTURE);
					}
				} else {
					rkcif_write_register_and(cif_dev, CIF_REG_MIPI_LVDS_CTRL,
								 ~CSI_ENABLE_CAPTURE);
				}
			}
		}

	} else {
		if (atomic_read(&cif_dev->pipe.stream_cnt) == 1) {
			val = rkcif_read_register(cif_dev, CIF_REG_DVP_CTRL);
			rkcif_write_register(cif_dev, CIF_REG_DVP_CTRL,
					     val & (~ENABLE_CAPTURE));
			rkcif_write_register(cif_dev, CIF_REG_DVP_INTEN, 0x0);
			rkcif_write_register(cif_dev, CIF_REG_DVP_INTSTAT, 0x3ff);
			rkcif_write_register(cif_dev, CIF_REG_DVP_FRAME_STATUS, 0x0);
			if (IS_ENABLED(CONFIG_CPU_RV1106))
				rkcif_config_dvp_pin(cif_dev, false);
		}
	}
	stream->state = RKCIF_STATE_READY;
	stream->dma_en = 0;
}

static bool rkcif_is_extending_line_for_height(struct rkcif_device *dev,
					       struct rkcif_stream *stream,
					       const struct cif_input_fmt *fmt)
{
	bool is_extended = false;
	struct rkmodule_hdr_cfg hdr_cfg;
	int ret;

	if (dev->chip_id == CHIP_RV1126_CIF ||
	    dev->chip_id == CHIP_RV1126_CIF_LITE) {
		if (dev->terminal_sensor.sd) {
			ret = v4l2_subdev_call(dev->terminal_sensor.sd,
					       core, ioctl,
					       RKMODULE_GET_HDR_CFG,
					       &hdr_cfg);
			if (!ret)
				dev->hdr = hdr_cfg;
			else
				dev->hdr.hdr_mode = NO_HDR;
		}

		if (fmt && fmt->fmt_type == CIF_FMT_TYPE_RAW) {
			if ((dev->hdr.hdr_mode == HDR_X2 &&
			    stream->id == RKCIF_STREAM_MIPI_ID1) ||
			    (dev->hdr.hdr_mode == HDR_X3 &&
			     stream->id == RKCIF_STREAM_MIPI_ID2) ||
			     (dev->hdr.hdr_mode == NO_HDR)) {
				is_extended = true;
			}
		}
	}

	return is_extended;
}

static int rkcif_queue_setup(struct vb2_queue *queue,
			     unsigned int *num_buffers,
			     unsigned int *num_planes,
			     unsigned int sizes[],
			     struct device *alloc_ctxs[])
{
	struct rkcif_stream *stream = queue->drv_priv;
	struct rkcif_extend_info *extend_line = &stream->extend_line;
	struct rkcif_device *dev = stream->cifdev;
	const struct v4l2_pix_format_mplane *pixm = NULL;
	const struct cif_output_fmt *cif_fmt;
	const struct cif_input_fmt *in_fmt;
	bool is_extended = false;
	u32 i, height;

	pixm = &stream->pixm;
	cif_fmt = stream->cif_fmt_out;
	in_fmt = stream->cif_fmt_in;

	*num_planes = cif_fmt->mplanes;

	if (stream->crop_enable)
		height = stream->crop[CROP_SRC_ACT].height;
	else
		height = pixm->height;

	is_extended = rkcif_is_extending_line_for_height(dev, stream, in_fmt);
	if (is_extended && extend_line->is_extended) {
		height = extend_line->pixm.height;
		pixm = &extend_line->pixm;
	}

	for (i = 0; i < cif_fmt->mplanes; i++) {
		const struct v4l2_plane_pix_format *plane_fmt;
		int h;

		if (dev->chip_id >= CHIP_RK3588_CIF && dev->terminal_sensor.hdmi_input_en)
			h = height;
		else
			h = round_up(height, MEMORY_ALIGN_ROUND_UP_HEIGHT);

		plane_fmt = &pixm->plane_fmt[i];
		sizes[i] = plane_fmt->sizeimage / height * h;
	}
	stream->total_buf_num = *num_buffers;
	v4l2_dbg(1, rkcif_debug, &dev->v4l2_dev, "%s count %d, size %d, extended(%d, %d)\n",
		 v4l2_type_names[queue->type], *num_buffers, sizes[0],
		 is_extended, extend_line->is_extended);

	return 0;
}

static void rkcif_check_buffer_update_pingpong(struct rkcif_stream *stream,
					       int channel_id)
{
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_mbus_config *mbus_cfg = &dev->active_sensor->mbus;
	struct rkcif_buffer *buffer = NULL;
	u32 frm_addr_y = 0, frm_addr_uv = 0;
	u32 frm0_addr_y = 0, frm0_addr_uv = 0;
	u32 frm1_addr_y = 0, frm1_addr_uv = 0;
	u32 buff_addr_y = 0, buff_addr_cbcr = 0;
	struct rkmodule_capture_info *capture_info = &dev->channels[channel_id].capture_info;
	unsigned long flags;
	int frame_phase = 0;
	bool is_dual_update_buf = false;
	int on = 1;

	if (stream->state != RKCIF_STATE_STREAMING ||
	    rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT ||
	    rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO)
		return;

	spin_lock_irqsave(&stream->vbq_lock, flags);
	if (stream->curr_buf == stream->next_buf ||
	    stream->curr_buf == NULL ||
	    stream->next_buf == NULL) {
		frame_phase = stream->frame_phase_cache;
		if (!stream->is_line_wake_up ||
		    (stream->is_line_wake_up && stream->frame_idx < 2)) {
			if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
			    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY ||
			    mbus_cfg->type == V4L2_MBUS_CCP2) {
				frm0_addr_y = get_reg_index_of_frm0_y_addr(channel_id);
				frm1_addr_y = get_reg_index_of_frm1_y_addr(channel_id);
				frm0_addr_uv = get_reg_index_of_frm0_uv_addr(channel_id);
				frm1_addr_uv = get_reg_index_of_frm1_uv_addr(channel_id);
			} else {
				frm0_addr_y = get_dvp_reg_index_of_frm0_y_addr(channel_id);
				frm1_addr_y = get_dvp_reg_index_of_frm1_y_addr(channel_id);
				frm0_addr_uv = get_dvp_reg_index_of_frm0_uv_addr(channel_id);
				frm1_addr_uv = get_dvp_reg_index_of_frm1_uv_addr(channel_id);
			}
			if (frame_phase & CIF_CSI_FRAME0_READY) {
				frm_addr_y = frm0_addr_y;
				frm_addr_uv = frm0_addr_uv;
			} else {
				frm_addr_y = frm1_addr_y;
				frm_addr_uv = frm1_addr_uv;
			}
			if (!stream->dma_en && stream->curr_buf == NULL && stream->next_buf == NULL)
				is_dual_update_buf = true;
			if (!list_empty(&stream->buf_head)) {
				if (frame_phase == CIF_CSI_FRAME0_READY) {
					stream->curr_buf = list_first_entry(&stream->buf_head,
									    struct rkcif_buffer, queue);
					if (stream->curr_buf) {
						list_del(&stream->curr_buf->queue);
						buffer = stream->curr_buf;
					}
					if (buffer && is_dual_update_buf)
						stream->next_buf = buffer;
				} else if (frame_phase == CIF_CSI_FRAME1_READY) {
					stream->next_buf = list_first_entry(&stream->buf_head,
								    struct rkcif_buffer, queue);
					if (stream->next_buf) {
						list_del(&stream->next_buf->queue);
						buffer = stream->next_buf;
					}
					if (buffer && is_dual_update_buf)
						stream->curr_buf = buffer;
				}
			} else {
				v4l2_info(&dev->v4l2_dev, "%s %d\n", __func__, __LINE__);
			}
			if (buffer) {
				if (is_dual_update_buf) {
					buff_addr_y = buffer->buff_addr[RKCIF_PLANE_Y];
					buff_addr_cbcr = buffer->buff_addr[RKCIF_PLANE_CBCR];
					if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
						rkcif_write_buff_addr_multi_dev_combine(stream,
											frm0_addr_y,
											frm0_addr_uv,
											buff_addr_y,
											buff_addr_cbcr,
											false);
						rkcif_write_buff_addr_multi_dev_combine(stream,
											frm1_addr_y,
											frm1_addr_uv,
											buff_addr_y,
											buff_addr_cbcr,
											false);
					} else {
						rkcif_write_register(dev, frm0_addr_y, buff_addr_y);
						if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
							rkcif_write_register(dev,
									     frm0_addr_uv,
									     buff_addr_cbcr);
						rkcif_write_register(dev, frm1_addr_y, buff_addr_y);
						if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
							rkcif_write_register(dev,
									     frm1_addr_uv,
									     buff_addr_cbcr);
					}
				} else {

					buff_addr_y = buffer->buff_addr[RKCIF_PLANE_Y];
					buff_addr_cbcr = buffer->buff_addr[RKCIF_PLANE_CBCR];
					if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
						rkcif_write_buff_addr_multi_dev_combine(stream,
											frm_addr_y,
											frm_addr_uv,
											buff_addr_y,
											buff_addr_cbcr,
											false);
					} else {
						rkcif_write_register(dev, frm_addr_y, buff_addr_y);
						if (stream->cif_fmt_out->fmt_type != CIF_FMT_TYPE_RAW)
							rkcif_write_register(dev,
									     frm_addr_uv,
									     buff_addr_cbcr);
					}
				}
			}
		} else {
			v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
				 "%s %d, is_wake_up %d, frame_idx %d\n",
				 __func__, __LINE__, stream->is_line_wake_up, stream->frame_idx);
			if (stream->curr_buf == stream->next_buf) {
				if (stream->frame_phase_cache == CIF_CSI_FRAME0_READY) {
					stream->curr_buf = list_first_entry(&stream->buf_head,
									    struct rkcif_buffer, queue);
					v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev,
						 "%s %d, stream[%d] buf idx %d\n",
						 __func__, __LINE__, stream->id, stream->curr_buf->vb.vb2_buf.index);
					if (stream->curr_buf)
						list_del(&stream->curr_buf->queue);
				} else if (stream->frame_phase_cache == CIF_CSI_FRAME1_READY) {
					stream->next_buf = list_first_entry(&stream->buf_head,
									    struct rkcif_buffer, queue);
					v4l2_dbg(4, rkcif_debug, &dev->v4l2_dev,
						 "%s %d, stream[%d] buf idx %d\n",
						 __func__, __LINE__, stream->id, stream->next_buf->vb.vb2_buf.index);
					if (stream->next_buf)
						list_del(&stream->next_buf->queue);
				}
				stream->is_buf_active = true;
			}
		}
		v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
			 "%s, stream[%d] update buffer, frame_phase %d, is_stop %s, lack_buf_cnt %d\n",
			 __func__, stream->id, frame_phase,
			 (stream->dma_en ? "false" : "true"),
			 stream->lack_buf_cnt);
		if (!stream->dma_en) {
			if (stream->to_stop_dma) {
				stream->to_stop_dma = 0;
				wake_up(&stream->wq_stopped);
			} else {
				if (stream->cifdev->resume_mode != RKISP_RTT_MODE_ONE_FRAME ||
				    stream->is_single_cap)
					stream->to_en_dma = RKCIF_DMAEN_BY_VICAP;
				v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
					 "%s stream[%d] start dma capture, frame cnt %d\n",
					 __func__, stream->id, stream->frame_idx);
			}
		} else {
			v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
				 "%s %d, dma_en 0x%x, frame cnt %d\n",
				 __func__, __LINE__, stream->dma_en, stream->frame_idx);
		}
		if (stream->lack_buf_cnt)
			stream->lack_buf_cnt--;

	} else {
		v4l2_info(&dev->v4l2_dev, "%s %d, state %d, curr_buf %p, next_buf %p\n",
			  __func__, __LINE__, stream->state, stream->curr_buf, stream->next_buf);
	}
	spin_unlock_irqrestore(&stream->vbq_lock, flags);

	if (stream->to_en_dma) {
		rkcif_enable_dma_capture(stream, true);
		mutex_lock(&dev->stream_lock);
		if (atomic_read(&dev->sensor_off)) {
			atomic_set(&dev->sensor_off, 0);
			rkcif_dphy_quick_stream(dev, on);
			v4l2_subdev_call(dev->terminal_sensor.sd, core, ioctl,
					 RKMODULE_SET_QUICK_STREAM, &on);
		}
		mutex_unlock(&dev->stream_lock);
	}
}

static void rkcif_add_fence_to_vb_done(struct rkcif_stream *stream,
					      struct vb2_v4l2_buffer *vb_done)
{
	unsigned long lock_flags = 0;
	struct rkcif_fence *vb_fence;
	struct rkcif_device *cif_dev = stream->cifdev;
	struct v4l2_device *v4l2_dev = &cif_dev->v4l2_dev;

	spin_lock_irqsave(&stream->fence_lock, lock_flags);
	if (!list_empty(&stream->qbuf_fence_list_head)) {
		vb_fence = list_first_entry(&stream->qbuf_fence_list_head,
					    struct rkcif_fence, fence_list);
		list_del(&vb_fence->fence_list);
	} else {
		vb_fence = NULL;
	}

	if (vb_fence)
		list_add_tail(&vb_fence->fence_list, &stream->done_fence_list_head);
	spin_unlock_irqrestore(&stream->fence_lock, lock_flags);

	if (vb_fence) {
		/*  pass the fence_fd to userspace through timecode.userbits */
		vb_done->timecode.userbits[0] = vb_fence->fence_fd & 0xff;
		vb_done->timecode.userbits[1] = (vb_fence->fence_fd & 0xff00) >> 8;
		vb_done->timecode.userbits[2] = (vb_fence->fence_fd & 0xff0000) >> 16;
		vb_done->timecode.userbits[3] = (vb_fence->fence_fd & 0xff000000) >> 24;
		v4l2_dbg(3, rkcif_debug, v4l2_dev, "%s: fence:%p, fence_fd:%d\n",
			 __func__, vb_fence->fence, vb_fence->fence_fd);
	} else {
		/* config userbits 0 or 0xffffffff as invalid fence_fd*/
		memset(vb_done->timecode.userbits, 0xff, sizeof(vb_done->timecode.userbits));
		v4l2_err(v4l2_dev, "%s: failed to get fence fd!\n", __func__);
	}
}

static void rkcif_dqbuf_get_done_fence(struct rkcif_stream *stream)
{
	unsigned long lock_flags = 0;
	struct rkcif_fence *done_fence;
	struct v4l2_device *v4l2_dev = &stream->cifdev->v4l2_dev;

	spin_lock_irqsave(&stream->fence_lock, lock_flags);
	if (!list_empty(&stream->done_fence_list_head)) {
		done_fence = list_first_entry(&stream->done_fence_list_head,
				struct rkcif_fence, fence_list);
		list_del(&done_fence->fence_list);
	} else {
		done_fence = NULL;
	}
	spin_unlock_irqrestore(&stream->fence_lock, lock_flags);

	if (done_fence) {
		spin_lock_irqsave(&stream->fence_lock, lock_flags);
		if (stream->rkcif_fence) {
			dma_fence_signal(stream->rkcif_fence->fence);
			dma_fence_put(stream->rkcif_fence->fence);
			v4l2_dbg(2, rkcif_debug, v4l2_dev, "%s: signal fence:%p, old_fd:%d\n",
				 __func__,
				 stream->rkcif_fence->fence,
				 stream->rkcif_fence->fence_fd);
			kfree(stream->rkcif_fence);
			stream->rkcif_fence = NULL;
		}
		stream->rkcif_fence = done_fence;
		spin_unlock_irqrestore(&stream->fence_lock, lock_flags);
		v4l2_dbg(3, rkcif_debug, v4l2_dev, "%s: fence:%p, fence_fd:%d\n",
			 __func__, done_fence->fence, done_fence->fence_fd);
	}
}

static void rkcif_fence_signal(struct rkcif_stream *stream)
{
	unsigned long lock_flags = 0;
	struct v4l2_device *v4l2_dev = &stream->cifdev->v4l2_dev;

	spin_lock_irqsave(&stream->fence_lock, lock_flags);
	if (stream->rkcif_fence) {
		dma_fence_signal(stream->rkcif_fence->fence);
		dma_fence_put(stream->rkcif_fence->fence);
		v4l2_dbg(2, rkcif_debug, v4l2_dev, "%s: signal fence:%p, old_fd:%d\n",
			 __func__,
			 stream->rkcif_fence->fence,
			 stream->rkcif_fence->fence_fd);
		kfree(stream->rkcif_fence);
		stream->rkcif_fence = NULL;
	}
	spin_unlock_irqrestore(&stream->fence_lock, lock_flags);
}

static void rkcif_free_fence(struct rkcif_stream *stream)
{
	unsigned long lock_flags = 0;
	struct rkcif_fence *vb_fence, *done_fence;
	struct v4l2_device *v4l2_dev = &stream->cifdev->v4l2_dev;
	struct files_struct *files = current->files;
	LIST_HEAD(local_list);

	spin_lock_irqsave(&stream->fence_lock, lock_flags);
	if (stream->rkcif_fence) {
		v4l2_dbg(2, rkcif_debug, v4l2_dev, "%s: signal rkcif_fence fd:%d\n",
			 __func__, stream->rkcif_fence->fence_fd);
		dma_fence_signal(stream->rkcif_fence->fence);
		dma_fence_put(stream->rkcif_fence->fence);
		kfree(stream->rkcif_fence);
		stream->rkcif_fence = NULL;
	}
	list_replace_init(&stream->qbuf_fence_list_head, &local_list);
	spin_unlock_irqrestore(&stream->fence_lock, lock_flags);

	while (!list_empty(&local_list)) {
		vb_fence = list_first_entry(&local_list, struct rkcif_fence, fence_list);
		list_del(&vb_fence->fence_list);
		v4l2_dbg(2, rkcif_debug, v4l2_dev, "%s: free qbuf_fence fd:%d\n",
			 __func__, vb_fence->fence_fd);
		dma_fence_put(vb_fence->fence);
		if (files) {
			put_unused_fd(vb_fence->fence_fd);
			close_fd(vb_fence->fence_fd);
		}
		kfree(vb_fence);
	}

	spin_lock_irqsave(&stream->fence_lock, lock_flags);
	list_replace_init(&stream->done_fence_list_head, &local_list);
	spin_unlock_irqrestore(&stream->fence_lock, lock_flags);

	while (!list_empty(&local_list)) {
		done_fence = list_first_entry(&local_list, struct rkcif_fence, fence_list);
		list_del(&done_fence->fence_list);
		v4l2_dbg(2, rkcif_debug, v4l2_dev, "%s: free done_fence fd:%d\n",
			 __func__, done_fence->fence_fd);
		dma_fence_put(done_fence->fence);
		if (files) {
			put_unused_fd(done_fence->fence_fd);
			close_fd(done_fence->fence_fd);
		}
		kfree(done_fence);
	}
}

static const char *rkcif_fence_get_name(struct dma_fence *fence)
{
	return CIF_DRIVER_NAME;
}

static const struct dma_fence_ops rkcif_fence_ops = {
	.get_driver_name = rkcif_fence_get_name,
	.get_timeline_name = rkcif_fence_get_name,
};

static void rkcif_fence_context_init(struct rkcif_fence_context *fence_ctx)
{
	fence_ctx->context = dma_fence_context_alloc(1);
	spin_lock_init(&fence_ctx->spinlock);
}

static struct dma_fence *rkcif_dma_fence_alloc(struct rkcif_fence_context *fence_ctx)
{
	struct dma_fence *fence = NULL;

	if (fence_ctx == NULL) {
		pr_err("fence_context is NULL!\n");
		return ERR_PTR(-EINVAL);
	}

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	if (!fence)
		return ERR_PTR(-ENOMEM);

	dma_fence_init(fence, &rkcif_fence_ops, &fence_ctx->spinlock,
		       fence_ctx->context, ++fence_ctx->seqno);

	return fence;
}

static int rkcif_dma_fence_get_fd(struct dma_fence *fence)
{
	struct sync_file *sync_file = NULL;
	int fence_fd = -1;

	if (!fence)
		return -EINVAL;

	fence_fd = get_unused_fd_flags(O_CLOEXEC);
	if (fence_fd < 0)
		return fence_fd;

	sync_file = sync_file_create(fence);
	if (!sync_file) {
		put_unused_fd(fence_fd);
		return -ENOMEM;
	}

	fd_install(fence_fd, sync_file->file);

	return fence_fd;
}

static void rkcif_qbuf_alloc_fence(struct rkcif_stream *stream)
{
	struct dma_fence *fence;
	int fence_fd;
	struct rkcif_fence *rkcif_fence;
	unsigned long lock_flags = 0;
	struct v4l2_device *v4l2_dev = &stream->cifdev->v4l2_dev;

	fence = rkcif_dma_fence_alloc(&stream->fence_ctx);
	if (!IS_ERR(fence)) {
		rkcif_fence = kzalloc(sizeof(struct rkcif_fence), GFP_KERNEL);
		if (!rkcif_fence) {
			kfree(fence);
			v4l2_err(v4l2_dev, "%s: failed to alloc rkcif_fence!\n", __func__);
			return;
		}
		fence_fd = rkcif_dma_fence_get_fd(fence);
		if (fence_fd >= 0) {
			rkcif_fence->fence = fence;
			rkcif_fence->fence_fd = fence_fd;
			spin_lock_irqsave(&stream->fence_lock, lock_flags);
			list_add_tail(&rkcif_fence->fence_list, &stream->qbuf_fence_list_head);
			spin_unlock_irqrestore(&stream->fence_lock, lock_flags);
			v4l2_dbg(3, rkcif_debug, v4l2_dev, "%s: fence:%p, fence_fd:%d\n",
				 __func__, fence, fence_fd);
		} else {
			dma_fence_put(fence);
			kfree(rkcif_fence);
			v4l2_err(v4l2_dev, "%s: failed to get fence fd!\n", __func__);
		}
	} else {
		v4l2_err(v4l2_dev, "%s: alloc fence failed!\n", __func__);
	}
}

/*
 * The vb2_buffer are stored in rkcif_buffer, in order to unify
 * mplane buffer and none-mplane buffer.
 */
void rkcif_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct rkcif_buffer *cifbuf = to_rkcif_buffer(vbuf);
	struct vb2_queue *queue = vb->vb2_queue;
	struct rkcif_stream *stream = queue->drv_priv;
	struct v4l2_pix_format_mplane *pixm = &stream->pixm;
	const struct cif_output_fmt *fmt = stream->cif_fmt_out;
	struct rkcif_hw *hw_dev = stream->cifdev->hw_dev;
	struct rkcif_tools_buffer *tools_buf;
	struct rkcif_tools_vdev *tools_vdev = stream->tools_vdev;
	unsigned long flags;
	int i;
	bool is_find_tools_buf = false;

	if (tools_vdev) {
		spin_lock_irqsave(&stream->tools_vdev->vbq_lock, flags);
		if (!list_empty(&tools_vdev->src_buf_head)) {
			list_for_each_entry(tools_buf, &tools_vdev->src_buf_head, list) {
				if (tools_buf->vb == vbuf) {
					is_find_tools_buf = true;
					break;
				}
			}
			if (is_find_tools_buf) {
				if (tools_buf->use_cnt)
					tools_buf->use_cnt--;
				if (tools_buf->use_cnt) {
					spin_unlock_irqrestore(&stream->tools_vdev->vbq_lock, flags);
					return;
				}
			}
		}
		spin_unlock_irqrestore(&stream->tools_vdev->vbq_lock, flags);
	}

	memset(cifbuf->buff_addr, 0, sizeof(cifbuf->buff_addr));
	/* If mplanes > 1, every c-plane has its own m-plane,
	 * otherwise, multiple c-planes are in the same m-plane
	 */
	for (i = 0; i < fmt->mplanes; i++) {
		void *addr = vb2_plane_vaddr(vb, i);

		if (hw_dev->is_dma_sg_ops) {
			struct sg_table *sgt = vb2_dma_sg_plane_desc(vb, i);

			cifbuf->buff_addr[i] = sg_dma_address(sgt->sgl);
		} else {
			cifbuf->buff_addr[i] = vb2_dma_contig_plane_dma_addr(vb, i);
		}
		if (rkcif_debug && addr && !hw_dev->iommu_en) {
			memset(addr, 0, pixm->plane_fmt[i].sizeimage);
			if (cifbuf->vb.vb2_buf.vb2_queue->mem_ops->finish)
				cifbuf->vb.vb2_buf.vb2_queue->mem_ops->finish(cifbuf->vb.vb2_buf.planes[i].mem_priv);
			v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
				 "Clear buffer, size: 0x%08x\n",
				 pixm->plane_fmt[i].sizeimage);
		}
	}

	if (fmt->mplanes == 1) {
		for (i = 0; i < fmt->cplanes - 1; i++)
			cifbuf->buff_addr[i + 1] = cifbuf->buff_addr[i] +
				pixm->plane_fmt[i].bytesperline * pixm->height;
	}
	spin_lock_irqsave(&stream->vbq_lock, flags);
	list_add_tail(&cifbuf->queue, &stream->buf_head);
	cifbuf->id = stream->id;
	spin_unlock_irqrestore(&stream->vbq_lock, flags);

	if (stream->dma_en & RKCIF_DMAEN_BY_ISP && (!cifbuf->dbuf)) {
		struct rkisp_rx_buf *dbufs = NULL;

		dbufs = kzalloc(sizeof(struct rkisp_rx_buf), GFP_ATOMIC);

		if (dbufs) {
			memset(dbufs, 0, sizeof(struct rkisp_rx_buf));
			if (stream->cifdev->hdr.hdr_mode == HDR_X2 && stream->id == 0)
				dbufs->type = BUF_MIDDLE;
			else if (stream->cifdev->hdr.hdr_mode == HDR_X3 && stream->id == 0)
				dbufs->type = BUF_LONG;
			else if (stream->cifdev->hdr.hdr_mode == HDR_X3 && stream->id == 1)
				dbufs->type = BUF_MIDDLE;
			cifbuf->dbuf = hw_dev->mem_ops->get_dmabuf(vb, vb->planes[0].mem_priv, O_RDWR);
			if (cifbuf->dbuf)
				dbufs->dbuf = cifbuf->dbuf;
			list_add_tail(&dbufs->list, &stream->rx_buf_head_vicap);
		} else {
			v4l2_err(&stream->cifdev->v4l2_dev,
				 "stream[%d] buf queue, index: %d, dma_addr 0x%x, malloc dbufs fail\n",
				 stream->id, vb->index, cifbuf->buff_addr[0]);
		}
	}
	if (stream->cifdev->workmode == RKCIF_WORKMODE_PINGPONG &&
	    stream->lack_buf_cnt &&
	    stream->cur_stream_mode & RKCIF_STREAM_MODE_CAPTURE &&
	    stream->cifdev->channels[0].capture_info.mode != RKMODULE_ONE_CH_TO_MULTI_ISP)
		rkcif_check_buffer_update_pingpong(stream, stream->id);
	atomic_inc(&stream->buf_cnt);

	if (stream->low_latency)
		rkcif_qbuf_alloc_fence(stream);

	spin_lock_irqsave(&stream->cifdev->stream_spinlock, flags);
	stream->is_finish_single_cap = true;
	if (stream->is_wait_single_cap &&
	    (stream->cifdev->hdr.hdr_mode == NO_HDR ||
	     (stream->cifdev->hdr.hdr_mode == HDR_X2 && stream->id == 1) ||
	     (stream->cifdev->hdr.hdr_mode == HDR_X3 && stream->id == 2))) {
		stream->is_wait_single_cap = false;
		spin_unlock_irqrestore(&stream->cifdev->stream_spinlock, flags);
		rkcif_quick_stream_on(stream->cifdev, false);
		complete(&stream->start_complete);
	} else {
		spin_unlock_irqrestore(&stream->cifdev->stream_spinlock, flags);
	}
	v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
		 "stream[%d] buf queue, index: %d, dma_addr 0x%x, dbuf %p, mem_priv %p\n",
		 stream->id, vb->index, cifbuf->buff_addr[0], cifbuf->vb.vb2_buf.planes[0].dbuf,
		 cifbuf->vb.vb2_buf.planes[0].mem_priv);
}

void rkcif_free_rx_buf(struct rkcif_stream *stream, int buf_num)
{
	struct rkcif_rx_buffer *buf;
	struct rkcif_device *dev = stream->cifdev;
	struct sditf_priv *priv = dev->sditf[0];
	struct v4l2_subdev *sd;
	int i = 0;
	unsigned long flags;
	phys_addr_t resmem_free_start;
	phys_addr_t resmem_free_end;
	u32 share_head_size = 0;
	u32 rtt_min_size = 0;

	if (!priv)
		return;

	sd = get_rkisp_sd(dev->sditf[0]);
	if (!sd)
		return;

	if ((dev->is_rtt_suspend || dev->is_aov_reserved) && dev->is_thunderboot) {
		stream->curr_buf_toisp = NULL;
		stream->next_buf_toisp = NULL;
		INIT_LIST_HEAD(&stream->rx_buf_head);

		for (i = 0; i < buf_num; i++) {
			buf = &stream->rx_buf[i];
			if (buf->dbufs.is_init)
				v4l2_subdev_call(sd, core, ioctl,
						 RKISP_VICAP_CMD_RX_BUFFER_FREE, &buf->dbufs);
			rkcif_free_reserved_mem_buf(dev, buf);
			memset(buf, 0, sizeof(*buf));
			buf->dummy.is_free = true;
		}

		if (IS_ENABLED(CONFIG_VIDEO_ROCKCHIP_THUNDER_BOOT_ISP)) {
			share_head_size = dev->thunderboot_sensor_num * sizeof(struct rkisp32_thunderboot_resmem_head);
			if (share_head_size != dev->share_mem_size)
				v4l2_info(&stream->cifdev->v4l2_dev,
					  "share mem head error, rtt head size %d, arm head size %d\n",
					  dev->share_mem_size, share_head_size);
			if (share_head_size + dev->nr_buf_size > stream->pixm.plane_fmt[0].sizeimage)
				rtt_min_size = share_head_size + dev->nr_buf_size;
			else
				rtt_min_size = stream->pixm.plane_fmt[0].sizeimage;
			if (dev->is_rtt_suspend)
				resmem_free_start = dev->resmem_pa + rtt_min_size;
			else
				resmem_free_start = dev->resmem_pa + stream->pixm.plane_fmt[0].sizeimage;
			resmem_free_end = dev->resmem_pa + dev->resmem_size;
			v4l2_info(&stream->cifdev->v4l2_dev,
				  "free reserved mem start 0x%x, end 0x%x, share_head_size 0x%x, nr_buf_size 0x%x\n",
				  (u32)resmem_free_start, (u32)resmem_free_end, share_head_size, dev->nr_buf_size);
			free_reserved_area(phys_to_virt(resmem_free_start),
					   phys_to_virt(resmem_free_end),
					   -1, "rkisp_thunderboot");
			if (dev->is_rtt_suspend)
				dev->resmem_size = rtt_min_size;
			else
				dev->resmem_size = stream->pixm.plane_fmt[0].sizeimage;
		}
		atomic_set(&stream->buf_cnt, 0);
		stream->total_buf_num = 0;
		stream->rx_buf_num = 0;

		return;
	}

	spin_lock_irqsave(&stream->vbq_lock, flags);
	stream->curr_buf_toisp = NULL;
	stream->next_buf_toisp = NULL;
	INIT_LIST_HEAD(&stream->rx_buf_head);
	spin_unlock_irqrestore(&stream->vbq_lock, flags);

	spin_lock_irqsave(&dev->buffree_lock, flags);
	for (i = 0; i < buf_num; i++) {
		buf = &stream->rx_buf[i];
		if (buf->dummy.is_free)
			continue;
		if (stream->is_m_online_fb_res && buf == stream->last_buf_toisp) {
			/* todo, release left buf*/
			stream->is_m_online_fb_res = false;
			list_add_tail(&buf->list, &stream->rx_buf_head);
			continue;
		}
		if (buf->dbufs.is_init)
			v4l2_subdev_call(sd, core, ioctl,
					 RKISP_VICAP_CMD_RX_BUFFER_FREE, &buf->dbufs);
		if (!dev->is_thunderboot) {
			rkcif_free_buffer(dev, &buf->dummy);
			memset(buf, 0, sizeof(*buf));
			buf->dummy.is_free = true;
		} else {
			list_add_tail(&buf->list_free, &priv->buf_free_list);
		}
		atomic_dec(&stream->buf_cnt);
		stream->total_buf_num--;
	}
	stream->rx_buf_num = 0;

	spin_unlock_irqrestore(&dev->buffree_lock, flags);

	if (buf_num)
		schedule_work(&priv->buffree_work.work);

	stream->dma_en &= ~RKCIF_DMAEN_BY_ISP;
	v4l2_dbg(1, rkcif_debug, &stream->cifdev->v4l2_dev,
		 "free rx_buf, buf_num %d\n", buf_num);
}

static void rkcif_sync_crop_info(struct rkcif_stream *stream);
static u32 rkcif_get_right_half_buf_size(struct rkcif_stream *stream)
{
	u32 width, height, virtual_width;

	rkcif_sync_crop_info(stream);
	if (stream->crop_enable) {
		width = stream->crop[CROP_SRC_ACT].width;
		height = stream->crop[CROP_SRC_ACT].height;
	} else {
		width = stream->pixm.width;
		height = stream->pixm.height;
	}
	width /= 2;
	width += stream->cifdev->unite_extend_pixel;
	if (stream->is_compact)
		virtual_width = ALIGN(width * stream->cif_fmt_out->raw_bpp / 8, 256);
	else
		virtual_width = ALIGN(width * stream->cif_fmt_out->bpp[0] / 8, 8);
	if (stream->cifdev->chip_id > CHIP_RK3562_CIF && stream->sw_dbg_en)
		virtual_width = (virtual_width + 23) / 24 * 24;
	return virtual_width * height;
}

static void rkcif_get_resmem_head(struct rkcif_device *cif_dev);
int rkcif_init_rx_buf(struct rkcif_stream *stream, int buf_num)
{
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_pix_format_mplane *pixm = &stream->pixm;
	struct rkcif_dummy_buffer *dummy;
	struct rkcif_rx_buffer *buf;
	struct sditf_priv *priv = dev->sditf[0];
	int frm_type = 0;
	int i = 0;
	int j = 0;
	int ret = 0;
	bool is_match_pre = false;

	if (!priv)
		return -EINVAL;

	if (buf_num > RKISP_VICAP_BUF_CNT_MAX)
		return -EINVAL;

	if (dev->hdr.hdr_mode == NO_HDR) {
		if (stream->id == 0)
			frm_type = BUF_SHORT;
		else
			return -EINVAL;
	} else if (dev->hdr.hdr_mode == HDR_X2) {
		if (stream->id == 0)
			frm_type = BUF_MIDDLE;
		else if (stream->id == 1)
			frm_type = BUF_SHORT;
		else
			return -EINVAL;
	} else if (dev->hdr.hdr_mode == HDR_X3) {
		if (stream->id == 0)
			frm_type = BUF_LONG;
		else if (stream->id == 1)
			frm_type = BUF_MIDDLE;
		else if (stream->id == 2)
			frm_type = BUF_SHORT;
		else
			return -EINVAL;
	}

	INIT_LIST_HEAD(&stream->rx_buf_head);
	while (true) {
		buf = &stream->rx_buf[i];
		memset(buf, 0, sizeof(*buf));
		dummy = &buf->dummy;
		if (priv->mode.rdbk_mode == RKISP_VICAP_ONLINE_UNITE &&
		    (priv->hdr_cfg.hdr_mode == NO_HDR || priv->hdr_cfg.hdr_mode == HDR_COMPR ||
		     (priv->hdr_cfg.hdr_mode == HDR_X2 && stream->id == 1) ||
		     (priv->hdr_cfg.hdr_mode == HDR_X3 && stream->id == 2)))
			dummy->size = rkcif_get_right_half_buf_size(stream);
		else if (dev->chip_id >= CHIP_RV1103B_CIF && priv->hdr_wrap_line)
			dummy->size = pixm->plane_fmt[0].bytesperline * priv->hdr_wrap_line;
		else
			dummy->size = pixm->plane_fmt[0].sizeimage;
		dummy->is_need_vaddr = true;
		dummy->is_need_dbuf = true;
		if (dev->is_thunderboot || dev->is_rtt_suspend || dev->is_aov_reserved) {
			if (i == 0)
				rkcif_get_resmem_head(dev);
			buf->buf_idx = i;
			ret = rkcif_alloc_reserved_mem_buf(dev, buf);
			if (ret) {
				stream->rx_buf_num = i;
				v4l2_info(&dev->v4l2_dev,
					 "reserved mem support alloc buf num %d, require buf num %d\n",
					 i, buf_num);
				break;
			}
			if (dev->rdbk_debug)
				v4l2_info(&dev->v4l2_dev,
					  "stream[%d] buf addr 0x%llx\n",
					  stream->id, (u64)dummy->dma_addr);
		} else {
			ret = rkcif_alloc_buffer(dev, dummy);
			if (ret) {
				stream->rx_buf_num = i;
				v4l2_info(&dev->v4l2_dev,
					 "alloc buf num %d, require buf num %d\n",
					 i, buf_num);
				break;
			}
			buf->dbufs.dbuf = dummy->dbuf;
		}
		buf->dbufs.is_init = false;
		buf->dbufs.type = frm_type;
		is_match_pre = false;
		if (dev->pre_buf_num) {
			for (j = 0; j < dev->pre_buf_num; j++) {
				if (dev->pre_buf_addr[i] == buf->dbufs.dma) {
					if (i == 0)
						buf->dbufs.is_first = true;
					stream->is_fb_first_frame = false;
					buf->dbufs.sequence = stream->frame_idx;
					buf->dbufs.timestamp = dev->pre_buf_timestamp[i];
					rkcif_s_rx_buffer(stream, &buf->dbufs);
					stream->frame_idx++;
					is_match_pre = true;
					break;
				}
			}
		}
		if (!is_match_pre)
			list_add_tail(&buf->list, &stream->rx_buf_head);
		dummy->is_free = false;
		if (stream->is_compact)
			buf->dbufs.is_uncompact = false;
		else
			buf->dbufs.is_uncompact = true;
		if (priv && i == 0 && dev->pre_buf_num == 0) {
			buf->dbufs.is_first = true;
			if (priv->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ)
				rkcif_s_rx_buffer(stream, &buf->dbufs);
		}
		v4l2_dbg(1, rkcif_debug, &dev->v4l2_dev,
			"init rx_buf,dma_addr 0x%llx size: 0x%x\n",
			(u64)dummy->dma_addr, dummy->size);
		i++;
		if (!dev->is_thunderboot && i >= buf_num) {
			stream->rx_buf_num = buf_num;
			break;
		} else if (i >= RKISP_VICAP_BUF_CNT_MAX) {
			stream->rx_buf_num = i;
			v4l2_info(&dev->v4l2_dev,
				  "reserved mem alloc buf num %d\n", i);
			break;
		}
	}
	if (stream->rx_buf_num) {
		stream->total_buf_num = stream->rx_buf_num;
		atomic_set(&stream->buf_cnt, stream->rx_buf_num);
		return 0;
	} else {
		return -EINVAL;
	}
}

static int rkcif_create_dummy_buf(struct rkcif_stream *stream)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_hw *hw = dev->hw_dev;
	struct rkcif_dummy_buffer *dummy_buf = &hw->dummy_buf;
	struct rkcif_device *tmp_dev = NULL;
	struct v4l2_subdev_frame_interval_enum fie;
	struct v4l2_subdev_format fmt;
	u32 max_size = 0;
	u32 size = 0;
	int ret = 0;
	int i, j;

	for (i = 0; i < hw->dev_num; i++) {
		tmp_dev = hw->cif_dev[i];
		if (tmp_dev->terminal_sensor.sd) {
			for (j = 0; j < 32; j++) {
				memset(&fie, 0, sizeof(fie));
				fie.index = j;
				fie.pad = 0;
				fie.which = V4L2_SUBDEV_FORMAT_ACTIVE;
				ret = v4l2_subdev_call(tmp_dev->terminal_sensor.sd,
						       pad, enum_frame_interval,
						       NULL, &fie);
				if (!ret) {
					if (fie.code == MEDIA_BUS_FMT_RGB888_1X24 ||
					    fie.code == MEDIA_BUS_FMT_BGR888_1X24 ||
					    fie.code == MEDIA_BUS_FMT_GBR888_1X24)
						size = fie.width * fie.height * 3;
					else
						size = fie.width * fie.height * 2;
					v4l2_dbg(1, rkcif_debug, &dev->v4l2_dev,
						 "%s enum fmt, width %d, height %d\n",
						 __func__, fie.width, fie.height);
				} else {
					break;
				}
				if (size > max_size)
					max_size = size;
			}
		} else {
			continue;
		}
	}

	if (max_size == 0 && dev->terminal_sensor.sd) {
		fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		ret = v4l2_subdev_call(dev->terminal_sensor.sd,
				       pad, get_fmt, NULL, &fmt);
		if (!ret) {
			if (fmt.format.code == MEDIA_BUS_FMT_RGB888_1X24 ||
			    fmt.format.code == MEDIA_BUS_FMT_BGR888_1X24 ||
			    fmt.format.code == MEDIA_BUS_FMT_GBR888_1X24)
				size = fmt.format.width  * fmt.format.height * 3;
			else
				size = fmt.format.width * fmt.format.height * 2;
			if (size > max_size)
				max_size = size;
		}
	}

	dummy_buf->size = max_size;

	dummy_buf->is_need_vaddr = true;
	dummy_buf->is_need_dbuf = true;
	ret = rkcif_alloc_buffer(dev, dummy_buf);
	if (ret) {
		v4l2_err(&dev->v4l2_dev,
			 "Failed to allocate the memory for dummy buffer, size %d\n", max_size);
		return -ENOMEM;
	}

	v4l2_info(&dev->v4l2_dev, "Allocate dummy buffer, size: 0x%08x\n",
		  dummy_buf->size);

	return ret;
}

static void rkcif_destroy_dummy_buf(struct rkcif_stream *stream)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_dummy_buffer *dummy_buf = &dev->hw_dev->dummy_buf;

	if (dummy_buf->vaddr)
		rkcif_free_buffer(dev, dummy_buf);
	dummy_buf->dma_addr = 0;
	dummy_buf->vaddr = NULL;
}

static void rkcif_do_cru_reset(struct rkcif_device *dev)
{
	struct rkcif_hw *cif_hw = dev->hw_dev;

	unsigned int val, i;

	if (dev->luma_vdev.enable)
		rkcif_stop_luma(&dev->luma_vdev);

	if (dev->chip_id == CHIP_RK1808_CIF) {
		val = rkcif_read_register(dev, CIF_REG_MIPI_WATER_LINE);
		val |= CIF_MIPI_LVDS_SW_DMA_IDLE_RK1808;
		rkcif_write_register(dev, CIF_REG_MIPI_WATER_LINE, val);
	} else {
		val = rkcif_read_register(dev, CIF_REG_MIPI_LVDS_CTRL);
		val |= CIF_MIPI_LVDS_SW_DMA_IDLE;
		rkcif_write_register(dev, CIF_REG_MIPI_LVDS_CTRL, val);
		val = rkcif_read_register(dev, CIF_REG_DVP_CTRL);
		val |= DVP_SW_DMA_IDLE;
		rkcif_write_register(dev, CIF_REG_DVP_CTRL, val);
	}
	udelay(5);

	for (i = 0; i < ARRAY_SIZE(cif_hw->cif_rst); i++)
		if (cif_hw->cif_rst[i])
			reset_control_assert(cif_hw->cif_rst[i]);

	udelay(10);

	for (i = 0; i < ARRAY_SIZE(cif_hw->cif_rst); i++)
		if (cif_hw->cif_rst[i])
			reset_control_deassert(cif_hw->cif_rst[i]);

	if (cif_hw->iommu_en) {
		rockchip_iommu_disable(cif_hw->dev);
		rockchip_iommu_enable(cif_hw->dev);
	}
}

void rkcif_do_soft_reset(struct rkcif_device *dev)
{
	struct csi_channel_info *channel = &dev->channels[0];
	int tmp_csi_host_idx = 0;
	int i = 0;

	if (dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_DPHY ||
	    dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_CPHY ||
	    dev->active_sensor->mbus.type == V4L2_MBUS_CCP2) {
		if (channel->capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
			tmp_csi_host_idx = dev->csi_host_idx;
			for (i = 0; i < channel->capture_info.multi_dev.dev_num; i++) {
				dev->csi_host_idx = channel->capture_info.multi_dev.dev_idx[i];
				rkcif_write_register_or(dev, CIF_REG_MIPI_LVDS_CTRL, 0x000A0000);
			}
			if (dev->chip_id >= CHIP_RV1103B_CIF) {
				for (i = 0; i < channel->capture_info.multi_dev.dev_num; i++) {
					dev->csi_host_idx = channel->capture_info.multi_dev.dev_idx[i];
					rkcif_write_register_and(dev, CIF_REG_MIPI_LVDS_CTRL, ~0x000f0000);
				}
			}
			dev->csi_host_idx = tmp_csi_host_idx;
		} else {
			rkcif_write_register_or(dev, CIF_REG_MIPI_LVDS_CTRL, 0x000A0000);
			if (dev->chip_id >= CHIP_RV1103B_CIF)
				rkcif_write_register_and(dev, CIF_REG_MIPI_LVDS_CTRL, ~0x000f0000);
		}
	} else {
		rkcif_write_register_or(dev, CIF_REG_DVP_CTRL, 0x000A0000);
		if (dev->chip_id >= CHIP_RV1103B_CIF)
			rkcif_write_register_and(dev, CIF_REG_DVP_CTRL, ~0x000f0000);

	}
	usleep_range(10, 20);
	v4l2_dbg(1, rkcif_debug, &dev->v4l2_dev,
		"vicap do soft reset 0x%x\n", 0x000A0000);
}

static void rkcif_release_rdbk_buf(struct rkcif_stream *stream)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_buffer *rdbk_buf;
	struct rkcif_buffer *tmp_buf;
	unsigned long flags;
	bool has_added;
	int index = 0;

	if (stream->id == RKCIF_STREAM_MIPI_ID0)
		index = RDBK_L;
	else if (stream->id == RKCIF_STREAM_MIPI_ID1)
		index = RDBK_M;
	else if (stream->id == RKCIF_STREAM_MIPI_ID2)
		index = RDBK_S;
	else
		return;

	spin_lock_irqsave(&dev->hdr_lock, flags);
	rdbk_buf = dev->rdbk_buf[index];
	if (rdbk_buf) {
		if (rdbk_buf != stream->curr_buf &&
		    rdbk_buf != stream->next_buf) {

			has_added = false;

			list_for_each_entry(tmp_buf, &stream->buf_head, queue) {
				if (tmp_buf == rdbk_buf) {
					has_added = true;
					break;
				}
			}

			if (!has_added)
				list_add_tail(&rdbk_buf->queue, &stream->buf_head);
		}
		dev->rdbk_buf[index] = NULL;
	}
	spin_unlock_irqrestore(&dev->hdr_lock, flags);

}

static void rkcif_detach_sync_mode(struct rkcif_device *cif_dev)
{
	int i = 0;
	struct rkcif_hw *hw = cif_dev->hw_dev;
	struct rkcif_device *tmp_dev;
	struct rkcif_multi_sync_config *sync_config;

	if ((!cif_dev->sync_cfg.type) ||
	    (atomic_read(&cif_dev->pipe.stream_cnt) != 0))
		return;
	mutex_lock(&hw->dev_lock);
	memset(&cif_dev->sync_cfg, 0, sizeof(cif_dev->sync_cfg));
	sync_config = &hw->sync_config[cif_dev->sync_cfg.group];
	sync_config->streaming_cnt--;
	if (cif_dev->sync_cfg.type == EXTERNAL_MASTER_MODE) {
		for (i = 0; i < sync_config->ext_master.count; i++) {
			tmp_dev = sync_config->ext_master.cif_dev[i];
			if (tmp_dev == cif_dev) {
				sync_config->ext_master.is_streaming[i] = false;
				break;
			}
		}
	}
	if (cif_dev->sync_cfg.type == INTERNAL_MASTER_MODE)
		sync_config->int_master.is_streaming[0] = false;
	if (cif_dev->sync_cfg.type == SLAVE_MODE) {
		for (i = 0; i < sync_config->slave.count; i++) {
			tmp_dev = sync_config->slave.cif_dev[i];
			if (tmp_dev == cif_dev) {
				sync_config->slave.is_streaming[i] = false;
				break;
			}
		}
	}

	if (!sync_config->streaming_cnt && sync_config->is_attach) {
		sync_config->is_attach = false;
		sync_config->mode = RKCIF_NOSYNC_MODE;
		sync_config->dev_cnt = 0;
	}
	mutex_unlock(&hw->dev_lock);
}

static void rkcif_clean_state_one_to_multi_mode(struct rkcif_device *dev)
{
	struct sditf_priv *priv = NULL;
	struct sditf_effect_exp *effect_exp;
	struct sditf_effect_time *effect_time;
	struct sditf_effect_gain *effect_gain;
	struct sditf_time *time;
	struct sditf_gain *gain;
	int i = 0;

	while (work_busy(&dev->exp_work)) {
		usleep_range(2000, 3000);
		i++;
		if (i > 5)
			break;
	}

	for (i = 0; i < dev->sditf_cnt; i++) {
		priv = dev->sditf[i];
		if (priv) {
			atomic_set(&priv->frm_sync_seq, 0);
			priv->frame_idx.cur_frame_idx = 0;
			priv->frame_idx.total_frame_idx = 0;
			while (!list_empty(&priv->effect_exp_head)) {
				effect_exp = list_first_entry(&priv->effect_exp_head,
							      struct sditf_effect_exp, list);
				if (effect_exp) {
					list_del(&effect_exp->list);
					kfree(effect_exp);
					effect_exp = NULL;
				}
			}
			while (!list_empty(&priv->time_head)) {
				time = list_first_entry(&priv->time_head,
							struct sditf_time, list);
				if (time) {
					list_del(&time->list);
					kfree(time);
					time = NULL;
				}
			}
			while (!list_empty(&priv->time_head)) {
				gain = list_first_entry(&priv->gain_head,
							struct sditf_gain, list);
				if (gain) {
					list_del(&gain->list);
					kfree(gain);
					gain = NULL;
				}
			}
		}
	}
	while (!list_empty(&dev->effect_time_head)) {
		effect_time = list_first_entry(&dev->effect_time_head,
					       struct sditf_effect_time, list);
		if (effect_time) {
			list_del(&effect_time->list);
			kfree(effect_time);
			effect_time = NULL;
		}
	}
	while (!list_empty(&dev->effect_gain_head)) {
		effect_gain = list_first_entry(&dev->effect_gain_head,
					       struct sditf_effect_gain, list);
		if (effect_gain) {
			list_del(&effect_gain->list);
			kfree(effect_gain);
			effect_gain = NULL;
		}
	}
}

void rkcif_do_stop_stream(struct rkcif_stream *stream,
			  enum rkcif_stream_mode mode)
{
	struct rkcif_vdev_node *node = &stream->vnode;
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	struct rkcif_buffer *buf = NULL;
	int ret;
	struct rkcif_hw *hw_dev = dev->hw_dev;
	struct rkmodule_capture_info *capture_info = &dev->channels[0].capture_info;
	bool can_reset = true;
	int i;
	unsigned long flags;
	u32 vblank = 0;
	u32 frame_time_ns = 0;
	u64 cur_time = 0;
	u64 fs_time = 0;
	int on = 0;

	mutex_lock(&dev->stream_lock);

	v4l2_info(&dev->v4l2_dev, "stream[%d] start stopping, total mode 0x%x, cur 0x%x\n",
		stream->id, stream->cur_stream_mode, mode);

	if (mode == stream->cur_stream_mode) {
		if (stream->dma_en) {
			if (!dev->sensor_linetime)
				dev->sensor_linetime = rkcif_get_linetime(stream);
			vblank = rkcif_get_sensor_vblank(dev);
			if (vblank) {
				frame_time_ns = (vblank + dev->terminal_sensor.raw_rect.height) *
						dev->sensor_linetime;
				spin_lock_irqsave(&stream->fps_lock, flags);
				fs_time = stream->readout.fs_timestamp;
				spin_unlock_irqrestore(&stream->fps_lock, flags);
				cur_time = rkcif_time_get_ns(dev);
				if (cur_time > fs_time &&
				    cur_time - fs_time < (frame_time_ns - 10000000) &&
				    stream->is_in_vblank)
					rkcif_stream_stop(stream);
				else
					stream->stopping = true;
			} else {
				stream->stopping = true;
			}
		} else if (dev->sditf[0] && (!dev->sditf[0]->is_toisp_off)) {
			stream->stopping = true;
		} else {
			rkcif_stream_stop(stream);
		}
		if (stream->stopping == true) {
			if (mode == RKCIF_STREAM_MODE_TOISP && dev->sditf[0]->is_toisp_off)
				ret = 0;
			else
				ret = wait_event_timeout(stream->wq_stopped,
							 stream->state != RKCIF_STATE_STREAMING,
							 msecs_to_jiffies(500));
			if (!ret) {
				rkcif_stream_stop(stream);
				stream->stopping = false;
			}
		}

		video_device_pipeline_stop(&node->vdev);
		if (dev->is_camera_over_bridge) {
			ret = v4l2_subdev_call(dev->sditf[stream->id]->sensor_sd,
					       video,
					       s_stream,
					       on);
			if (ret < 0)
				v4l2_err(v4l2_dev, "camera over bridge stream-off failed error:%d\n",
					 ret);
		}
		ret = dev->pipe.set_stream(&dev->pipe, false);
		if (ret < 0)
			v4l2_err(v4l2_dev, "pipeline stream-off failed error:%d\n",
				 ret);

		dev->is_start_hdr = false;
		stream->is_dvp_yuv_addr_init = false;
		if (stream->skip_info.skip_en) {
			stream->skip_info.skip_en = false;
			stream->skip_info.skip_to_en = true;
		}
	} else if (mode == RKCIF_STREAM_MODE_CAPTURE && stream->dma_en & RKCIF_DMAEN_BY_VICAP) {
		//only stop dma
		stream->to_stop_dma = RKCIF_DMAEN_BY_VICAP;
		stream->is_wait_dma_stop = true;
		wait_event_timeout(stream->wq_stopped,
				   !stream->is_wait_dma_stop,
				   msecs_to_jiffies(1000));
	} else if (mode == RKCIF_STREAM_MODE_TOISP && stream->dma_en & RKCIF_DMAEN_BY_VICAP) {
		//only stop dma
		stream->to_stop_dma = RKCIF_DMAEN_BY_ISP;
		stream->is_wait_dma_stop = true;
		wait_event_timeout(stream->wq_stopped,
				   !stream->is_wait_dma_stop,
				   msecs_to_jiffies(1000));
	}

	if (!atomic_read(&dev->pipe.stream_cnt) &&
	    dev->channels[0].capture_info.mode == RKMODULE_ONE_CH_TO_MULTI_ISP)
		rkcif_clean_state_one_to_multi_mode(dev);
	if (dev->channels[0].capture_info.mode == RKMODULE_MULTI_CH_TO_MULTI_ISP &&
	    dev->sditf[stream->id])
		atomic_set(&dev->sditf[stream->id]->frm_sync_seq, 0);

	if ((mode & RKCIF_STREAM_MODE_CAPTURE) == RKCIF_STREAM_MODE_CAPTURE) {
		/* release buffers */
		spin_lock_irqsave(&stream->vbq_lock, flags);
		if (stream->curr_buf)
			list_add_tail(&stream->curr_buf->queue, &stream->buf_head);
		if (stream->next_buf &&
		    stream->next_buf != stream->curr_buf)
			list_add_tail(&stream->next_buf->queue, &stream->buf_head);
		spin_unlock_irqrestore(&stream->vbq_lock, flags);

		if (dev->hdr.hdr_mode == HDR_X2 ||
		    dev->hdr.hdr_mode == HDR_X3)
			rkcif_release_rdbk_buf(stream);

		stream->curr_buf = NULL;
		stream->next_buf = NULL;

		rkcif_rx_buffer_free(stream);
		list_for_each_entry(buf, &stream->buf_head, queue) {
			v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev,
				 "stream[%d] buf return addr 0x%x\n",
				 stream->id, buf->buff_addr[0]);
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		}
		while (!list_empty(&stream->buf_head_multi_cache)) {
			buf = list_first_entry(&stream->buf_head_multi_cache,
					       struct rkcif_buffer, queue);
			if (buf) {
				list_del(&buf->queue);
				vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
			}
		}
		INIT_LIST_HEAD(&stream->buf_head);
		while (!list_empty(&stream->vb_done_list)) {
			buf = list_first_entry(&stream->vb_done_list,
					       struct rkcif_buffer, queue);
			if (buf) {
				list_del(&buf->queue);
				vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
			}
		}
		stream->total_buf_num = 0;
		atomic_set(&stream->buf_cnt, 0);
		stream->lack_buf_cnt = 0;
		stream->dma_en &= ~RKCIF_DMAEN_BY_VICAP;
	}

	if (mode == stream->cur_stream_mode) {
		ret = dev->pipe.close(&dev->pipe);
		if (ret < 0)
			v4l2_err(v4l2_dev, "pipeline close failed error:%d\n", ret);
		if (atomic_read(&dev->pipe.stream_cnt) == 0)
			dev->can_be_reset = true;
		mutex_lock(&hw_dev->dev_lock);
		for (i = 0; i < hw_dev->dev_num; i++) {
			if (atomic_read(&hw_dev->cif_dev[i]->pipe.stream_cnt) != 0) {
				can_reset = false;
				break;
			}
		}
		if (can_reset && hw_dev->dummy_buf.vaddr)
			rkcif_destroy_dummy_buf(stream);
		mutex_unlock(&hw_dev->dev_lock);
		if (dev->can_be_reset && dev->chip_id >= CHIP_RK3588_CIF) {
			rkcif_do_soft_reset(dev);
			atomic_set(&dev->streamoff_cnt, 0);
			if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
				for (i = 0; i < capture_info->multi_dev.dev_num - 1; i++)
					dev->other_intstat[i] = 0;
			}
		}
		if (dev->can_be_reset && can_reset) {
			dev->can_be_reset = false;
			dev->reset_work_cancel = true;
			dev->early_line = 0;
			dev->sensor_linetime = 0;
			dev->wait_line = 0;
			stream->is_line_wake_up = false;
			dev->is_in_flip = false;
			dev->pre_buf_num = 0;
			dev->hw_dev->is_in_reset = false;
		}
		if (atomic_read(&dev->pipe.stream_cnt) == 0)
			atomic_set(&stream->sub_stream_buf_cnt, 0);
		stream->rounding_bit = 0;
		if (stream->id == RKCIF_STREAM_MIPI_ID0 && dev->is_support_get_exp) {
			kfifo_free(&stream->exp_kfifo);

			kfifo_free(&stream->gain_kfifo);

			kfifo_free(&stream->vts_kfifo);

			kfifo_free(&stream->dcg_kfifo);
		}
		stream->crop_enable = false;
		stream->crop_mask = 0;
		stream->frame_loss = 0;
		stream->is_fb_first_frame = true;
	}
	if (mode == RKCIF_STREAM_MODE_CAPTURE) {
		tasklet_disable(&stream->vb_done_tasklet);
		INIT_LIST_HEAD(&stream->vb_done_list);
	}

	if (mode == stream->cur_stream_mode)
		rkcif_detach_sync_mode(dev);
	if (!atomic_read(&dev->pipe.stream_cnt) && dev->is_alloc_buf_user)
		rkcif_free_buf_by_user_require(dev);
	rkcif_free_fence(stream);
	stream->cur_stream_mode &= ~mode;
	v4l2_info(&dev->v4l2_dev, "stream[%d] stopping finished, dma_en 0x%x\n", stream->id, stream->dma_en);
	mutex_unlock(&dev->stream_lock);
}

static void rkcif_stop_streaming(struct vb2_queue *queue)
{
	struct rkcif_stream *stream = queue->drv_priv;

	rkcif_do_stop_stream(stream, RKCIF_STREAM_MODE_CAPTURE);
}

/*
 * CIF supports the following input modes,
 *    YUV, the default mode
 *    PAL,
 *    NTSC,
 *    RAW, if the input format is raw bayer
 *    JPEG, TODO
 *    MIPI, TODO
 */
static u32 rkcif_determine_input_mode(struct rkcif_stream *stream)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_sensor_info *sensor_info = dev->active_sensor;
	struct rkcif_sensor_info *terminal_sensor = &dev->terminal_sensor;
	__u32 intf = BT656_STD_RAW;
	u32 mode = INPUT_MODE_YUV;
	v4l2_std_id std;
	int ret;

	ret = v4l2_subdev_call(sensor_info->sd, video, querystd, &std);
	if (ret == 0) {
		/* retrieve std from sensor if exist */
		switch (std) {
		case V4L2_STD_NTSC:
			mode = INPUT_MODE_NTSC;
			break;
		case V4L2_STD_PAL:
			mode = INPUT_MODE_PAL;
			break;
		case V4L2_STD_ATSC:
			mode = INPUT_MODE_BT1120;
			break;
		default:
			v4l2_err(&dev->v4l2_dev,
				 "std: %lld is not supported", std);
		}
	} else {
		/* determine input mode by mbus_code (fmt_type) */
		switch (stream->cif_fmt_in->fmt_type) {
		case CIF_FMT_TYPE_YUV:
			if (dev->chip_id >= CHIP_RK3568_CIF) {
				if (sensor_info->mbus.type == V4L2_MBUS_BT656)
					mode = INPUT_MODE_BT656_YUV422;
				else
					mode = INPUT_MODE_YUV;
			} else {
				mode = INPUT_MODE_YUV;
			}
			break;
		case CIF_FMT_TYPE_RAW:
			if (dev->chip_id >= CHIP_RK3568_CIF) {
				ret = v4l2_subdev_call(terminal_sensor->sd,
						       core, ioctl,
						       RKMODULE_GET_BT656_INTF_TYPE,
						       &intf);
				if (!ret) {
					if (intf == BT656_SONY_RAW)
						mode = INPUT_MODE_SONY_RAW;
					else
						mode = INPUT_MODE_RAW;
				} else {
					mode = INPUT_MODE_RAW;
				}
			} else {
				mode = INPUT_MODE_RAW;
			}
			break;
		}
	}

	return mode;
}

static u32 rkcif_determine_input_mode_rk3588(struct rkcif_stream *stream)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_sensor_info *sensor_info = dev->active_sensor;
	struct rkcif_sensor_info *terminal_sensor = &dev->terminal_sensor;
	__u32 intf = BT656_STD_RAW;
	u32 mode = INPUT_MODE_YUV;
	v4l2_std_id std;
	int ret;

	ret = v4l2_subdev_call(sensor_info->sd, video, querystd, &std);
	if (ret == 0) {
		/* retrieve std from sensor if exist */
		switch (std) {
		case V4L2_STD_NTSC:
		case V4L2_STD_PAL:
			mode = INPUT_BT656_YUV422;
			break;
		case V4L2_STD_ATSC:
			mode = INPUT_BT1120_YUV422;
			break;
		default:
			v4l2_err(&dev->v4l2_dev,
				 "std: %lld is not supported", std);
		}
		mode |= CSI_WRDDR_TYPE_RAW8 << 6;
	} else {
		/* determine input mode by mbus_code (fmt_type) */
		switch (stream->cif_fmt_in->fmt_type) {
		case CIF_FMT_TYPE_YUV:
			if (sensor_info->mbus.type == V4L2_MBUS_BT656) {
				if ((sensor_info->mbus.bus.parallel.flags & CIF_DVP_PCLK_DUAL_EDGE) == CIF_DVP_PCLK_DUAL_EDGE)
					mode = INPUT_BT1120_YUV422;
				else
					mode = INPUT_BT656_YUV422;
			} else {
				mode = INPUT_BT601_YUV422;
			}
			mode |= CSI_WRDDR_TYPE_RAW8 << 6;
			break;
		case CIF_FMT_TYPE_RAW:
			ret = v4l2_subdev_call(terminal_sensor->sd,
					       core, ioctl,
					       RKMODULE_GET_BT656_INTF_TYPE,
					       &intf);
			if (!ret) {
				if (intf == BT656_SONY_RAW)
					mode = INPUT_SONY_RAW;
				else
					mode = INPUT_BT601_RAW;
			} else {
				mode = INPUT_BT601_RAW;
			}
			mode |= stream->cif_fmt_in->csi_fmt_val << 6;
			break;
		}
	}
	return mode;
}

static u32 rkcif_determine_input_mode_rk3576(struct rkcif_stream *stream)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_sensor_info *sensor_info = dev->active_sensor;
	struct rkcif_sensor_info *terminal_sensor = &dev->terminal_sensor;
	__u32 intf = BT656_STD_RAW;
	u32 mode = INPUT_MODE_YUV;
	v4l2_std_id std;
	int ret;

	ret = v4l2_subdev_call(sensor_info->sd, video, querystd, &std);
	if (ret == 0) {
		/* retrieve std from sensor if exist */
		switch (std) {
		case V4L2_STD_NTSC:
		case V4L2_STD_PAL:
			mode = INPUT_BT656_YUV422 << 2;
			break;
		case V4L2_STD_ATSC:
			mode = INPUT_BT1120_YUV422 << 2;
			break;
		default:
			v4l2_err(&dev->v4l2_dev,
				 "std: %lld is not supported", std);
		}
		mode |= CSI_WRDDR_TYPE_RAW8 << 11;
	} else {
		/* determine input mode by mbus_code (fmt_type) */
		switch (stream->cif_fmt_in->fmt_type) {
		case CIF_FMT_TYPE_YUV:
			if (sensor_info->mbus.type == V4L2_MBUS_BT656) {
				if ((sensor_info->mbus.bus.parallel.flags & CIF_DVP_PCLK_DUAL_EDGE) == CIF_DVP_PCLK_DUAL_EDGE)
					mode = INPUT_BT1120_YUV422 << 2;
				else
					mode = INPUT_BT656_YUV422 << 2;
			} else {
				mode = INPUT_BT601_YUV422 << 2;
			}
			mode |= CSI_WRDDR_TYPE_RAW8 << 11;
			break;
		case CIF_FMT_TYPE_RAW:
			ret = v4l2_subdev_call(terminal_sensor->sd,
					       core, ioctl,
					       RKMODULE_GET_BT656_INTF_TYPE,
					       &intf);
			if (!ret) {
				if (intf == BT656_SONY_RAW)
					mode = INPUT_SONY_RAW << 2;
				else
					mode = INPUT_BT601_RAW << 2;
			} else {
				mode = INPUT_BT601_RAW << 2;
			}
			mode |= stream->cif_fmt_in->csi_fmt_val << 11;
			break;
		}
	}
	return mode;
}

static u32 rkcif_determine_input_mode_rv1126b(struct rkcif_stream *stream)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_sensor_info *sensor_info = dev->active_sensor;
	struct rkcif_sensor_info *terminal_sensor = &dev->terminal_sensor;
	__u32 intf = BT656_STD_RAW;
	u32 mode = INPUT_MODE_YUV;
	v4l2_std_id std;
	int ret;

	ret = v4l2_subdev_call(sensor_info->sd, video, querystd, &std);
	if (ret == 0) {
		/* retrieve std from sensor if exist */
		switch (std) {
		case V4L2_STD_NTSC:
		case V4L2_STD_PAL:
			mode = INPUT_BT656_RV1126B;
			break;
		case V4L2_STD_ATSC:
			mode = INPUT_BT1120_RV1126B;
			break;
		default:
			v4l2_err(&dev->v4l2_dev,
				 "std: %lld is not supported", std);
		}
	} else {
		/* determine input mode by mbus_code (fmt_type) */
		switch (stream->cif_fmt_in->fmt_type) {
		case CIF_FMT_TYPE_YUV:
			if (sensor_info->mbus.type == V4L2_MBUS_BT656) {
				if ((sensor_info->mbus.bus.parallel.flags & CIF_DVP_PCLK_DUAL_EDGE) == CIF_DVP_PCLK_DUAL_EDGE)
					mode = INPUT_BT1120_RV1126B;
				else
					mode = INPUT_BT656_RV1126B;
			} else {
				mode = INPUT_BT601_RV1126B;
			}
			break;
		case CIF_FMT_TYPE_RAW:
			ret = v4l2_subdev_call(terminal_sensor->sd,
					       core, ioctl,
					       RKMODULE_GET_BT656_INTF_TYPE,
					       &intf);
			if (!ret) {
				if (intf == BT656_SONY_RAW)
					mode = INPUT_SONY_RAW_RV1126B;
				else
					mode = INPUT_BT601_RV1126B;
			} else {
				mode = INPUT_BT601_RV1126B;
			}
			break;
		}
	}
	return mode;
}

static inline u32 rkcif_scl_ctl(struct rkcif_stream *stream)
{
	u32 fmt_type = stream->cif_fmt_in->fmt_type;

	return (fmt_type == CIF_FMT_TYPE_YUV) ?
		ENABLE_YUV_16BIT_BYPASS : ENABLE_RAW_16BIT_BYPASS;
}

/**
 * rkcif_align_bits_per_pixel() - return the bit width of per pixel for stored
 * In raw or jpeg mode, data is stored by 16-bits,so need to align it.
 */
static u32 rkcif_align_bits_per_pixel(struct rkcif_stream *stream,
				      const struct cif_output_fmt *fmt,
				      int plane_index)
{
	u32 bpp = 0, i, cal = 0;

	if (fmt) {
		switch (fmt->fourcc) {
		case V4L2_PIX_FMT_NV16:
		case V4L2_PIX_FMT_NV61:
		case V4L2_PIX_FMT_NV12:
		case V4L2_PIX_FMT_NV21:
		case V4L2_PIX_FMT_GREY:
		case V4L2_PIX_FMT_Y16:
			bpp = fmt->bpp[plane_index];
			break;
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_YVYU:
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_VYUY:
			if (stream->cifdev->chip_id < CHIP_RK3588_CIF)
				bpp = fmt->bpp[plane_index];
			else
				bpp = fmt->bpp[plane_index + 1];
			break;
		case V4L2_PIX_FMT_RGB24:
		case V4L2_PIX_FMT_BGR24:
		case V4L2_PIX_FMT_RGB565:
		case V4L2_PIX_FMT_BGR666:
		case V4L2_PIX_FMT_SRGGB8:
		case V4L2_PIX_FMT_SGRBG8:
		case V4L2_PIX_FMT_SGBRG8:
		case V4L2_PIX_FMT_SBGGR8:
		case V4L2_PIX_FMT_SRGGB10:
		case V4L2_PIX_FMT_SGRBG10:
		case V4L2_PIX_FMT_SGBRG10:
		case V4L2_PIX_FMT_SBGGR10:
		case V4L2_PIX_FMT_SRGGB12:
		case V4L2_PIX_FMT_SGRBG12:
		case V4L2_PIX_FMT_SGBRG12:
		case V4L2_PIX_FMT_SBGGR12:
		case V4L2_PIX_FMT_SBGGR16:
		case V4L2_PIX_FMT_SGBRG16:
		case V4L2_PIX_FMT_SGRBG16:
		case V4L2_PIX_FMT_SRGGB16:
		case V4l2_PIX_FMT_SPD16:
		case V4l2_PIX_FMT_EBD8:
		case V4L2_PIX_FMT_Y10:
		case V4L2_PIX_FMT_Y12:
			if (stream->cifdev->chip_id < CHIP_RV1126_CIF) {
				bpp = max(fmt->bpp[plane_index], (u8)CIF_RAW_STORED_BIT_WIDTH);
				cal = CIF_RAW_STORED_BIT_WIDTH;
			} else {
				bpp = max(fmt->bpp[plane_index], (u8)CIF_RAW_STORED_BIT_WIDTH_RV1126);
				cal = CIF_RAW_STORED_BIT_WIDTH_RV1126;
			}
			for (i = 1; i < 5; i++) {
				if (i * cal >= bpp) {
					bpp = i * cal;
					break;
				}
			}
			break;
		default:
			v4l2_err(&stream->cifdev->v4l2_dev, "fourcc: %d is not supported!\n",
				 fmt->fourcc);
			break;
		}
	}

	return bpp;
}

/**
 * rkcif_cal_raw_vir_line_ratio() - return ratio for virtual line width setting
 * In raw or jpeg mode, data is stored by 16-bits,
 * so need to align virtual line width.
 */
static u32 rkcif_cal_raw_vir_line_ratio(struct rkcif_stream *stream,
					const struct cif_output_fmt *fmt)
{
	u32 ratio = 0, bpp = 0;

	if (fmt) {
		bpp = rkcif_align_bits_per_pixel(stream, fmt, 0);
		ratio = bpp / CIF_YUV_STORED_BIT_WIDTH;
	}

	return ratio;
}

static void rkcif_sync_crop_info(struct rkcif_stream *stream)
{
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_subdev_selection input_sel;
	int ret;

	if (dev->terminal_sensor.sd) {
		input_sel.target = V4L2_SEL_TGT_CROP_BOUNDS;
		input_sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		input_sel.pad = 0;
		ret = v4l2_subdev_call(dev->terminal_sensor.sd,
				       pad, get_selection, NULL,
				       &input_sel);
		if (!ret) {
			stream->crop[CROP_SRC_SENSOR] = input_sel.r;
			stream->crop_enable = true;
			stream->crop_mask |= CROP_SRC_SENSOR_MASK;
			dev->terminal_sensor.selection = input_sel;
		} else {
			stream->crop_mask &= ~CROP_SRC_SENSOR_MASK;
			dev->terminal_sensor.selection.r = dev->terminal_sensor.raw_rect;
		}
	}

	if ((stream->crop_mask & 0x3) == (CROP_SRC_USR_MASK | CROP_SRC_SENSOR_MASK)) {
		if (stream->crop[CROP_SRC_USR].left + stream->crop[CROP_SRC_USR].width >
		    stream->crop[CROP_SRC_SENSOR].width ||
		    stream->crop[CROP_SRC_USR].top + stream->crop[CROP_SRC_USR].height >
		    stream->crop[CROP_SRC_SENSOR].height)
			stream->crop[CROP_SRC_USR] = stream->crop[CROP_SRC_SENSOR];
	}

	if (stream->crop_mask & CROP_SRC_USR_MASK) {
		stream->crop[CROP_SRC_ACT] = stream->crop[CROP_SRC_USR];
		if (stream->crop_mask & CROP_SRC_SENSOR_MASK) {
			stream->crop[CROP_SRC_ACT].left = stream->crop[CROP_SRC_USR].left +
							  stream->crop[CROP_SRC_SENSOR].left;
			stream->crop[CROP_SRC_ACT].top = stream->crop[CROP_SRC_USR].top +
							  stream->crop[CROP_SRC_SENSOR].top;
		}
	} else if (stream->crop_mask & CROP_SRC_SENSOR_MASK) {
		stream->crop[CROP_SRC_ACT] = stream->crop[CROP_SRC_SENSOR];
	} else {
		stream->crop[CROP_SRC_ACT] = dev->terminal_sensor.raw_rect;
	}
}

/**rkcif_sanity_check_fmt - check fmt for setting
 * @stream - the stream for setting
 * @s_crop - the crop information
 */
static int rkcif_sanity_check_fmt(struct rkcif_stream *stream,
				  const struct v4l2_rect *s_crop)
{
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	struct v4l2_rect input, *crop;

	if (dev->terminal_sensor.sd) {
		stream->cif_fmt_in = rkcif_get_input_fmt(dev,
							 &input, stream->id,
							 &dev->channels[stream->id]);
		if (!stream->cif_fmt_in) {
			v4l2_err(v4l2_dev, "Input fmt is invalid\n");
			return -EINVAL;
		}
	} else {
		v4l2_err(v4l2_dev, "terminal_sensor is invalid\n");
		return -EINVAL;
	}

	if (stream->cif_fmt_in->mbus_code == MEDIA_BUS_FMT_EBD_1X8 ||
		stream->cif_fmt_in->mbus_code == MEDIA_BUS_FMT_SPD_2X8) {
		stream->crop_enable = false;
		return 0;
	}

	if (s_crop)
		crop = (struct v4l2_rect *)s_crop;
	else
		crop = &stream->crop[CROP_SRC_ACT];

	if (crop->width + crop->left > input.width ||
	    crop->height + crop->top > input.height) {
		v4l2_err(v4l2_dev, "crop size is bigger than input\n");
		return -EINVAL;
	}

	if (dev->active_sensor &&
	    (dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_DPHY ||
	    dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_CPHY)) {
		if (crop->left > 0) {
			int align_x = get_csi_crop_align(stream->cif_fmt_in);

			if (align_x > 0 && crop->left % align_x != 0) {
				v4l2_err(v4l2_dev,
					 "ERROR: crop left must align %d\n",
					 align_x);
				return -EINVAL;
			}
		}
	} else if (dev->active_sensor && dev->active_sensor->mbus.type == V4L2_MBUS_CCP2) {
		if (crop->left % 4 != 0 && crop->width % 4 != 0) {
			v4l2_err(v4l2_dev,
				 "ERROR: lvds crop left and width must align %d\n", 4);
			return -EINVAL;
		}
	}
	if (atomic_read(&dev->pipe.stream_cnt) == 0)
		v4l2_subdev_call(dev->terminal_sensor.sd, video,
				 g_frame_interval, &dev->terminal_sensor.src_fi);

	return 0;
}

int rkcif_update_sensor_info(struct rkcif_stream *stream)
{
	struct rkcif_sensor_info *sensor, *terminal_sensor;
	struct v4l2_subdev *sensor_sd;
	int ret = 0;

	sensor_sd = get_remote_sensor(stream, NULL);
	if (!sensor_sd) {
		v4l2_err(&stream->cifdev->v4l2_dev,
			 "%s: stream[%d] get remote sensor_sd failed!\n",
			 __func__, stream->id);
		return -ENODEV;
	}

	sensor = sd_to_sensor(stream->cifdev, sensor_sd);
	if (!sensor) {
		v4l2_err(&stream->cifdev->v4l2_dev,
			 "%s: stream[%d] get remote sensor failed!\n",
			 __func__, stream->id);
		return -ENODEV;
	}
	ret = v4l2_subdev_call(sensor->sd, pad, get_mbus_config,
			       0, &sensor->mbus);
	if (ret && ret != -ENOIOCTLCMD) {
		v4l2_err(&stream->cifdev->v4l2_dev,
			 "%s: get remote %s mbus failed!\n", __func__, sensor->sd->name);
		return ret;
	}

	stream->cifdev->active_sensor = sensor;

	terminal_sensor = &stream->cifdev->terminal_sensor;
	get_remote_terminal_sensor(stream, &terminal_sensor->sd);
	if (terminal_sensor->sd) {
		ret = v4l2_subdev_call(terminal_sensor->sd, pad, get_mbus_config,
				       0, &terminal_sensor->mbus);
		if (ret && ret != -ENOIOCTLCMD) {
			v4l2_err(&stream->cifdev->v4l2_dev,
				 "%s: get terminal %s mbus failed!\n",
				 __func__, terminal_sensor->sd->name);
			return ret;
		}
		ret = v4l2_subdev_call(terminal_sensor->sd, video,
				       g_frame_interval, &terminal_sensor->fi);
		if (ret) {
			v4l2_err(&stream->cifdev->v4l2_dev,
				 "%s: get terminal %s g_frame_interval failed!\n",
				 __func__, terminal_sensor->sd->name);
			return ret;
		}
		if (v4l2_subdev_call(terminal_sensor->sd, core, ioctl, RKMODULE_GET_CSI_DSI_INFO,
					&terminal_sensor->dsi_input_en)) {
			v4l2_dbg(1, rkcif_debug, &stream->cifdev->v4l2_dev,
				"%s: get terminal %s CSI/DSI sel failed, default csi input!\n",
				__func__, terminal_sensor->sd->name);
			terminal_sensor->dsi_input_en = 0;
		}
		if (terminal_sensor->dsi_input_en) {
			if (v4l2_subdev_call(terminal_sensor->sd, core, ioctl,
					RKMODULE_GET_DSI_MODE, &terminal_sensor->dsi_mode)) {
				v4l2_dbg(1, rkcif_debug, &stream->cifdev->v4l2_dev,
					"%s: get terminal %s DSI mode failed, set video mode!\n",
					__func__, terminal_sensor->sd->name);
				terminal_sensor->dsi_mode = 0;
			}
		} else {
			terminal_sensor->dsi_mode = 0;
		}
		if (v4l2_subdev_call(terminal_sensor->sd, core, ioctl, RKMODULE_GET_HDMI_MODE,
					&terminal_sensor->hdmi_input_en)) {
			v4l2_dbg(1, rkcif_debug, &stream->cifdev->v4l2_dev,
				"%s: get terminal %s hdmiin, default!\n",
				__func__, terminal_sensor->sd->name);
			terminal_sensor->hdmi_input_en = 0;
		}
	} else {
		v4l2_err(&stream->cifdev->v4l2_dev,
			 "%s: stream[%d] get remote terminal sensor failed!\n",
			 __func__, stream->id);
		return -ENODEV;
	}


	if (terminal_sensor->mbus.type == V4L2_MBUS_CSI2_DPHY ||
	    terminal_sensor->mbus.type == V4L2_MBUS_CSI2_CPHY)
		terminal_sensor->lanes = terminal_sensor->mbus.bus.mipi_csi2.num_data_lanes;
	else if (terminal_sensor->mbus.type == V4L2_MBUS_CCP2)
		terminal_sensor->lanes = terminal_sensor->mbus.bus.mipi_csi1.data_lane;
	return ret;
}

static int rkcif_dvp_get_output_type_mask(struct rkcif_stream *stream)
{
	unsigned int mask;
	const struct cif_output_fmt *fmt = stream->cif_fmt_out;

	switch (fmt->fourcc) {
	case V4L2_PIX_FMT_NV16:
		mask = (CSI_WRDDR_TYPE_YUV422SP_RK3588 << 11) |
		       (CSI_YUV_OUTPUT_ORDER_UYVY << 1);
		break;
	case V4L2_PIX_FMT_NV61:
		mask = (CSI_WRDDR_TYPE_YUV422SP_RK3588 << 11) |
		       (CSI_YUV_OUTPUT_ORDER_VYUY << 1);
		break;
	case V4L2_PIX_FMT_NV12:
		mask = (CSI_WRDDR_TYPE_YUV420SP_RK3588 << 11) |
		       (CSI_YUV_OUTPUT_ORDER_UYVY << 1);
		break;
	case V4L2_PIX_FMT_NV21:
		mask = (CSI_WRDDR_TYPE_YUV420SP_RK3588 << 11) |
		       (CSI_YUV_OUTPUT_ORDER_VYUY << 1);
		break;
	case V4L2_PIX_FMT_YUYV:
		mask = (CSI_WRDDR_TYPE_YUV_PACKET << 11) |
		       (CSI_YUV_OUTPUT_ORDER_YUYV << 1);
		break;
	case V4L2_PIX_FMT_YVYU:
		mask = (CSI_WRDDR_TYPE_YUV_PACKET << 11) |
		       (CSI_YUV_OUTPUT_ORDER_YVYU << 1);
		break;
	case V4L2_PIX_FMT_UYVY:
		mask = (CSI_WRDDR_TYPE_YUV_PACKET << 11) |
		       (CSI_YUV_OUTPUT_ORDER_UYVY << 1);
		break;
	case V4L2_PIX_FMT_VYUY:
		mask = (CSI_WRDDR_TYPE_YUV_PACKET << 11) |
		       (CSI_YUV_OUTPUT_ORDER_VYUY << 1);
		break;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_BGR666:
		mask = CSI_WRDDR_TYPE_RAW_COMPACT << 11;
		break;
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_Y10:
	case V4L2_PIX_FMT_Y12:
		if (stream->is_compact)
			mask = CSI_WRDDR_TYPE_RAW_COMPACT << 11;
		else
			mask = CSI_WRDDR_TYPE_RAW_UNCOMPACT << 11;
		break;
	case V4L2_PIX_FMT_SBGGR16:
	case V4L2_PIX_FMT_SGBRG16:
	case V4L2_PIX_FMT_SGRBG16:
	case V4L2_PIX_FMT_SRGGB16:
	case V4L2_PIX_FMT_Y16:
		mask = CSI_WRDDR_TYPE_RAW_UNCOMPACT << 11;
		break;
	default:
		mask = CSI_WRDDR_TYPE_RAW_COMPACT << 11;
		break;
	}
	return mask;
}

static int rkcif_dvp_get_output_type_mask_rk3576(struct rkcif_stream *stream)
{
	unsigned int mask;
	const struct cif_output_fmt *fmt = stream->cif_fmt_out;

	switch (fmt->fourcc) {
	case V4L2_PIX_FMT_NV16:
		mask = (CSI_WRDDR_TYPE_YUV422SP_RK3588 << 15) |
		       CSI_YUV_OUTPUT_ORDER_UYVY;
		break;
	case V4L2_PIX_FMT_NV61:
		mask = (CSI_WRDDR_TYPE_YUV422SP_RK3588 << 15) |
		       CSI_YUV_OUTPUT_ORDER_VYUY;
		break;
	case V4L2_PIX_FMT_NV12:
		mask = (CSI_WRDDR_TYPE_YUV420SP_RK3588 << 15) |
		       CSI_YUV_OUTPUT_ORDER_UYVY;
		break;
	case V4L2_PIX_FMT_NV21:
		mask = (CSI_WRDDR_TYPE_YUV420SP_RK3588 << 15) |
		       CSI_YUV_OUTPUT_ORDER_VYUY;
		break;
	case V4L2_PIX_FMT_YUYV:
		mask = (CSI_WRDDR_TYPE_YUV_PACKET << 15) |
		       CSI_YUV_OUTPUT_ORDER_YUYV;
		break;
	case V4L2_PIX_FMT_YVYU:
		mask = (CSI_WRDDR_TYPE_YUV_PACKET << 15) |
		       CSI_YUV_OUTPUT_ORDER_YVYU;
		break;
	case V4L2_PIX_FMT_UYVY:
		mask = (CSI_WRDDR_TYPE_YUV_PACKET << 15) |
		       CSI_YUV_OUTPUT_ORDER_UYVY;
		break;
	case V4L2_PIX_FMT_VYUY:
		mask = (CSI_WRDDR_TYPE_YUV_PACKET << 15) |
		       CSI_YUV_OUTPUT_ORDER_VYUY;
		break;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_BGR666:
		mask = CSI_WRDDR_TYPE_RAW_COMPACT << 15;
		break;
	case V4L2_PIX_FMT_SRGGB8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SRGGB12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_Y10:
	case V4L2_PIX_FMT_Y12:
		if (stream->is_compact)
			mask = CSI_WRDDR_TYPE_RAW_COMPACT << 15;
		else
			mask = CSI_WRDDR_TYPE_RAW_UNCOMPACT << 15;
		break;
	case V4L2_PIX_FMT_SBGGR16:
	case V4L2_PIX_FMT_SGBRG16:
	case V4L2_PIX_FMT_SGRBG16:
	case V4L2_PIX_FMT_SRGGB16:
	case V4L2_PIX_FMT_Y16:
		mask = CSI_WRDDR_TYPE_RAW_UNCOMPACT << 15;
		break;
	default:
		mask = CSI_WRDDR_TYPE_RAW_COMPACT << 15;
		break;
	}
	return mask;
}

static int rkcif_stream_start(struct rkcif_stream *stream, unsigned int mode)
{
	u32 val, mbus_flags, href_pol, vsync_pol,
	    xfer_mode = 0, yc_swap = 0, inputmode = 0,
	    mipimode = 0, workmode = 0, multi_id = 0,
	    multi_id_en = BT656_1120_MULTI_ID_DISABLE,
	    multi_id_mode = BT656_1120_MULTI_ID_MODE_1,
	    multi_id_sel = BT656_1120_MULTI_ID_SEL_LSB,
	    bt1120_edge_mode = BT1120_CLOCK_SINGLE_EDGES,
	    bt1120_flags = 0,
	    out_fmt_mask = 0,
	    in_fmt_yuv_order = 0;
	struct rkmodule_bt656_mbus_info bt1120_info;
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_sensor_info *sensor_info;
	struct v4l2_mbus_config *mbus;
	struct rkcif_dvp_sof_subdev *sof_sd = &dev->dvp_sof_subdev;
	const struct cif_output_fmt *fmt;
	unsigned int dma_en = 0;
	unsigned int dma_state = 0;
	int i = 0;
	u32 sav_detect = BT656_DETECT_SAV;
	u32 reserved = 0;

	if (stream->state < RKCIF_STATE_STREAMING) {
		stream->frame_idx = 0;
		stream->buf_wake_up_cnt = 0;
		stream->lack_buf_cnt = 0;
		stream->frame_phase = 0;
		stream->is_in_vblank = false;
		stream->is_change_toisp = false;
		stream->last_frame_idx = 0;
		stream->frame_phase_cache = CIF_CSI_FRAME1_READY;
	}

	sensor_info = dev->active_sensor;
	mbus = &sensor_info->mbus;

	dma_state = stream->dma_en;
	if ((mode & RKCIF_STREAM_MODE_CAPTURE) == RKCIF_STREAM_MODE_CAPTURE)
		stream->dma_en |= RKCIF_DMAEN_BY_VICAP;
	else if ((mode & RKCIF_STREAM_MODE_TOISP_RDBK) == RKCIF_STREAM_MODE_TOISP_RDBK)
		stream->dma_en |= RKCIF_DMAEN_BY_ISP;
	else if ((mode & RKCIF_STREAM_MODE_ROCKIT) == RKCIF_STREAM_MODE_ROCKIT)
		stream->dma_en |= RKCIF_DMAEN_BY_ROCKIT;

	if (dma_state)
		return 0;

	mbus_flags = mbus->bus.parallel.flags;
	if ((mbus_flags & CIF_DVP_PCLK_DUAL_EDGE) == CIF_DVP_PCLK_DUAL_EDGE) {
		bt1120_edge_mode = (dev->chip_id < CHIP_RK3588_CIF ?
			BT1120_CLOCK_DOUBLE_EDGES : BT1120_CLOCK_DOUBLE_EDGES_RK3588);
		rkcif_enable_dvp_clk_dual_edge(dev, true);
	} else {
		bt1120_edge_mode = dev->chip_id < CHIP_RK3588_CIF ?
			BT1120_CLOCK_SINGLE_EDGES : BT1120_CLOCK_SINGLE_EDGES_RK3588;
		rkcif_enable_dvp_clk_dual_edge(dev, false);
	}

	if (IS_ENABLED(CONFIG_CPU_RV1106))
		rkcif_config_dvp_pin(dev, true);

	if (mbus_flags & V4L2_MBUS_PCLK_SAMPLE_RISING)
		rkcif_config_dvp_clk_sampling_edge(dev, RKCIF_CLK_RISING);
	else
		rkcif_config_dvp_clk_sampling_edge(dev, RKCIF_CLK_FALLING);

#if IS_ENABLED(CONFIG_CPU_RV1106)
	rv1106_sdmmc_get_lock();
#endif
	if (sensor_info->sd && mbus->type == V4L2_MBUS_BT656) {
		int ret;

		multi_id_en = BT656_1120_MULTI_ID_ENABLE;

		ret = v4l2_subdev_call(sensor_info->sd,
				       core, ioctl,
				       RKMODULE_GET_BT656_MBUS_INFO,
				       &bt1120_info);
		if (ret) {
			v4l2_warn(&dev->v4l2_dev,
				  "warning: no multi channel info for BT.656\n");
		} else {
			bt1120_flags = bt1120_info.flags;
			if (bt1120_flags & RKMODULE_CAMERA_BT656_PARSE_ID_LSB)
				multi_id_sel = BT656_1120_MULTI_ID_SEL_LSB;
			else
				multi_id_sel = BT656_1120_MULTI_ID_SEL_MSB;

			if (((bt1120_flags & RKMODULE_CAMERA_BT656_CHANNELS) >> 2) > 3)
				multi_id_mode = BT656_1120_MULTI_ID_MODE_4;
			else if (((bt1120_flags & RKMODULE_CAMERA_BT656_CHANNELS) >> 2) > 1)
				multi_id_mode = BT656_1120_MULTI_ID_MODE_2;
			for (i = 0; i < 4; i++)
				multi_id |= DVP_SW_MULTI_ID(i, i, bt1120_info.id_en_bits);
			rkcif_write_register_or(dev, CIF_REG_DVP_MULTI_ID, multi_id);
		}
	}

	href_pol = (mbus_flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH) ?
		    HSY_HIGH_ACTIVE : HSY_LOW_ACTIVE;
	vsync_pol = (mbus_flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH) ?
		     VSY_HIGH_ACTIVE : VSY_LOW_ACTIVE;

	if (dev->chip_id < CHIP_RK3588_CIF)
		inputmode = rkcif_determine_input_mode(stream);
	else if (dev->chip_id < CHIP_RK3576_CIF)
		inputmode = rkcif_determine_input_mode_rk3588(stream);
	else
		inputmode = rkcif_determine_input_mode_rk3576(stream);
	if (dev->chip_id <= CHIP_RK1808_CIF) {
		if (inputmode == INPUT_MODE_BT1120) {
			if (stream->cif_fmt_in->field == V4L2_FIELD_NONE)
				xfer_mode = BT1120_TRANSMIT_PROGRESS;
			else
				xfer_mode = BT1120_TRANSMIT_INTERFACE;
			if (CIF_FETCH_IS_Y_FIRST(stream->cif_fmt_in->dvp_fmt_val))
				yc_swap = BT1120_YC_SWAP;
		}
	} else if (dev->chip_id < CHIP_RK3588_CIF) {
		if (sensor_info->mbus.type == V4L2_MBUS_BT656) {
			if (stream->cif_fmt_in->field == V4L2_FIELD_NONE)
				xfer_mode = BT1120_TRANSMIT_PROGRESS;
			else
				xfer_mode = BT1120_TRANSMIT_INTERFACE;
		}

		if (inputmode == INPUT_MODE_BT1120) {
			if (CIF_FETCH_IS_Y_FIRST(stream->cif_fmt_in->dvp_fmt_val))
				yc_swap = BT1120_YC_SWAP;
		}
	} else if (dev->chip_id < CHIP_RK3562_CIF) {
		if (sensor_info->mbus.type == V4L2_MBUS_BT656) {
			if (stream->cif_fmt_in->field == V4L2_FIELD_NONE)
				xfer_mode = BT1120_TRANSMIT_PROGRESS_RK3588;
			else
				xfer_mode = BT1120_TRANSMIT_INTERFACE_RK3588;
		}
		if ((inputmode & INPUT_BT1120_YUV422) == INPUT_BT1120_YUV422)
			if (CIF_FETCH_IS_Y_FIRST(stream->cif_fmt_in->dvp_fmt_val))
				yc_swap = BT1120_YC_SWAP_RK3588;
	} else {
		if (sensor_info->mbus.type == V4L2_MBUS_BT656) {
			if (stream->cif_fmt_in->field == V4L2_FIELD_NONE)
				xfer_mode = BT1120_TRANSMIT_PROGRESS_RK3576;
			else
				xfer_mode = BT1120_TRANSMIT_INTERFACE_RK3576;
		}
		if ((inputmode & INPUT_BT1120_YUV422) == INPUT_BT1120_YUV422)
			if (CIF_FETCH_IS_Y_FIRST(stream->cif_fmt_in->dvp_fmt_val))
				yc_swap = BT1120_YC_SWAP_RK3576;
	}

	if (dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_DPHY ||
	    dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_CPHY) {
		inputmode = INPUT_MODE_MIPI;

		/* if cif is linked with mipi,
		 * href pol must be set as high active,
		 * vsyn pol must be set as low active.
		 */
		href_pol = HSY_HIGH_ACTIVE;
		vsync_pol = VSY_LOW_ACTIVE;

		if (stream->cif_fmt_in->fmt_type == CIF_FMT_TYPE_YUV)
			mipimode = MIPI_MODE_YUV;
		else if (stream->cif_fmt_in->fmt_type == CIF_FMT_TYPE_RAW)
			mipimode = MIPI_MODE_RGB;
		else
			mipimode = MIPI_MODE_32BITS_BYPASS;
	}

	if (dev->chip_id < CHIP_RK3588_CIF) {
		val = vsync_pol | href_pol | inputmode | mipimode
			| stream->cif_fmt_out->fmt_val
			| stream->cif_fmt_in->dvp_fmt_val
			| xfer_mode | yc_swap | multi_id_en
			| multi_id_sel | multi_id_mode | bt1120_edge_mode;
		if (stream->is_high_align)
			val |= CIF_HIGH_ALIGN;
		else
			val &= ~CIF_HIGH_ALIGN;
	} else if (dev->chip_id < CHIP_RK3576_CIF) {
		out_fmt_mask = rkcif_dvp_get_output_type_mask(stream);
		in_fmt_yuv_order = rkcif_dvp_get_input_yuv_order(stream);
		val = vsync_pol | href_pol | inputmode
			| yc_swap
			| out_fmt_mask
			| in_fmt_yuv_order
			| multi_id_en
			| xfer_mode
			| sav_detect
			| multi_id_sel | multi_id_mode | bt1120_edge_mode;
		if (stream->is_high_align)
			val |= CIF_HIGH_ALIGN_RK3588;
		else
			val &= ~CIF_HIGH_ALIGN_RK3588;
	} else {
		out_fmt_mask = rkcif_dvp_get_output_type_mask_rk3576(stream);
		in_fmt_yuv_order = rkcif_dvp_get_input_yuv_order_rk3576(stream);
		val = vsync_pol | href_pol
			| inputmode
			| yc_swap
			| out_fmt_mask
			| in_fmt_yuv_order
			| multi_id_en
			| xfer_mode
			| multi_id_sel | multi_id_mode
			| (bt1120_edge_mode >> 8);
		if (stream->is_high_align)
			val |= (CIF_HIGH_ALIGN_RK3588 << 2);
		else
			val &= ~(CIF_HIGH_ALIGN_RK3588 << 2);
	}
	if (dev->chip_id >= CHIP_RK3576_CIF)
		val |= DVP_UVDS_EN;
	rkcif_write_register(dev, CIF_REG_DVP_FOR, val);

	if (dev->chip_id >= CHIP_RK3588_CIF) {
		val = stream->pixm.plane_fmt[0].bytesperline;
	} else {
		fmt = rkcif_find_output_fmt(stream, stream->pixm.pixelformat);
		if (fmt->fmt_type == CIF_FMT_TYPE_RAW &&
		    fmt->csi_fmt_val == CSI_WRDDR_TYPE_RAW8)
			val = ALIGN(stream->pixm.width * fmt->raw_bpp / 8, 256);
		else
			val = stream->pixm.width * rkcif_cal_raw_vir_line_ratio(stream, fmt);
	}

	if (stream->crop_enable) {
		dev->channels[stream->id].crop_en = 1;
		dev->channels[stream->id].crop_st_x = stream->crop[CROP_SRC_ACT].left;
		dev->channels[stream->id].crop_st_y = stream->crop[CROP_SRC_ACT].top;
		dev->channels[stream->id].width = stream->crop[CROP_SRC_ACT].width;
		dev->channels[stream->id].height = stream->crop[CROP_SRC_ACT].height;
	} else {
		dev->channels[stream->id].crop_st_y = 0;
		dev->channels[stream->id].crop_st_x = 0;
		dev->channels[stream->id].width = stream->pixm.width;
		dev->channels[stream->id].height = stream->pixm.height;
		dev->channels[stream->id].crop_en = 0;
	}

	if (dev->chip_id > CHIP_RK3562_CIF && stream->sw_dbg_en)
		val = (val + 23) / 24 * 24;

	rkcif_write_register(dev, CIF_REG_DVP_VIR_LINE_WIDTH, val);
	rkcif_write_register(dev, CIF_REG_DVP_SET_SIZE,
			     dev->channels[stream->id].width |
			     (dev->channels[stream->id].height << 16));
	rkcif_write_register(dev, CIF_REG_DVP_CROP,
			     dev->channels[stream->id].crop_st_y << CIF_CROP_Y_SHIFT |
			     dev->channels[stream->id].crop_st_x);

	if (atomic_read(&dev->pipe.stream_cnt) <= 1)
		rkcif_write_register(dev, CIF_REG_DVP_FRAME_STATUS, FRAME_STAT_CLS);

	if (dev->chip_id < CHIP_RK3588_CIF) {
		rkcif_write_register(dev, CIF_REG_DVP_INTSTAT, INTSTAT_CLS);
		rkcif_write_register(dev, CIF_REG_DVP_SCL_CTRL, rkcif_scl_ctl(stream));
		rkcif_write_register_or(dev, CIF_REG_DVP_INTEN,
				DVP_DMA_END_INTSTAT(stream->id) |
				INTSTAT_ERR | PST_INF_FRAME_END);
		/* enable line int for sof */
		rkcif_write_register(dev, CIF_REG_DVP_LINE_INT_NUM, 0x1);
		rkcif_write_register_or(dev, CIF_REG_DVP_INTEN, LINE_INT_EN);
	} else if (dev->chip_id < CHIP_RK3576_CIF) {
		if (dev->chip_id == CHIP_RV1106_CIF)
			reserved = 0xfc3c0000;
		else
			reserved = 0;
		rkcif_write_register(dev, CIF_REG_DVP_INTSTAT, 0x3c3ffff | reserved);
		rkcif_write_register_or(dev, CIF_REG_DVP_INTEN, 0x033ffff);//0x3c3ffff
	} else {
		rkcif_write_register(dev, CIF_REG_DVP_INTSTAT, 0x3c3ffff);
		rkcif_write_register_or(dev, CIF_REG_DVP_INTEN, 0x3c3ff0f);
	}

	if (stream->dma_en) {
		if (dev->chip_id < CHIP_RK1808_CIF) {
			rkcif_assign_new_buffer_oneframe(stream,
							 RKCIF_YUV_ADDR_STATE_INIT);
		} else {
			if (mode == RKCIF_STREAM_MODE_CAPTURE)
				rkcif_assign_new_buffer_pingpong(stream,
						 RKCIF_YUV_ADDR_STATE_INIT,
						 stream->id);
			else if (mode == RKCIF_STREAM_MODE_TOISP ||
				 mode == RKCIF_STREAM_MODE_TOISP_RDBK)
				rkcif_assign_new_buffer_pingpong_toisp(stream,
						       RKCIF_YUV_ADDR_STATE_INIT,
						       stream->id);
			else if (mode == RKCIF_STREAM_MODE_ROCKIT)
				rkcif_assign_new_buffer_pingpong_rockit(stream,
							RKCIF_YUV_ADDR_STATE_INIT,
							stream->id);
		}
	}

	if (dev->chip_id < CHIP_RK3588_CIF) {
		/* enable line int for sof */
		rkcif_write_register(dev, CIF_REG_DVP_LINE_INT_NUM, 0x1);
		rkcif_write_register_or(dev, CIF_REG_DVP_INTEN, LINE_INT_EN);
	}

	if (dev->workmode == RKCIF_WORKMODE_ONEFRAME)
		workmode = MODE_ONEFRAME;
	else if (dev->workmode == RKCIF_WORKMODE_PINGPONG)
		workmode = MODE_PINGPONG;
	else
		workmode = MODE_LINELOOP;

	if ((inputmode & INPUT_MODE_BT1120) == INPUT_MODE_BT1120) {
		workmode = MODE_PINGPONG;
		dev->workmode = RKCIF_WORKMODE_PINGPONG;
	}

	if (dev->chip_id == CHIP_RK3588_CIF) {
		if (stream->dma_en)
			dma_en = DVP_DMA_EN;
		if (stream->lack_buf_cnt == 2)
			dma_en = 0;
		rkcif_write_register(dev, CIF_REG_DVP_CTRL,
			     DVP_SW_WATER_LINE_25
			     | dma_en
			     | DVP_PRESS_EN
			     | DVP_HURRY_EN
			     | DVP_SW_WATER_LINE_25
			     | DVP_SW_PRESS_VALUE(3)
			     | DVP_SW_HURRY_VALUE(3)
			     | ENABLE_CAPTURE);
	} else if (dev->chip_id == CHIP_RV1106_CIF) {
		if (stream->dma_en)
			dma_en = DVP_SW_DMA_EN(stream->id);
		if (stream->lack_buf_cnt == 2)
			dma_en = 0;
		rkcif_write_register(dev, CIF_REG_DVP_CTRL,
			     DVP_SW_WATER_LINE_25
			     | DVP_PRESS_EN
			     | DVP_HURRY_EN
			     | DVP_SW_WATER_LINE_25
			     | DVP_SW_PRESS_VALUE(3)
			     | DVP_SW_HURRY_VALUE(3)
			     | DVP_SW_CAP_EN(stream->id)
			     | dma_en
			     | ENABLE_CAPTURE);
	} else if (dev->chip_id == CHIP_RK3576_CIF) {
		if (stream->dma_en)
			dma_en = DVP_SW_DMA_EN_RK3676(stream->id);
		if (stream->lack_buf_cnt == 2)
			dma_en = 0;
		rkcif_write_register(dev, CIF_REG_DVP_CTRL,
			     DVP_SW_WATER_LINE_25_RK3576
			     | DVP_SW_CAP_EN_RK3576(stream->id)
			     | dma_en
			     | ENABLE_CAPTURE);
	} else {
		rkcif_write_register(dev, CIF_REG_DVP_CTRL,
				     AXI_BURST_16 | workmode | ENABLE_CAPTURE);
	}
	dev->intr_mask = rkcif_read_register(dev, CIF_REG_DVP_INTSTAT);
#if IS_ENABLED(CONFIG_CPU_RV1106)
	rv1106_sdmmc_put_lock();
#endif
	atomic_set(&sof_sd->frm_sync_seq, 0);
	stream->state = RKCIF_STATE_STREAMING;
	stream->cifdev->dvp_sof_in_oneframe = 0;

	return 0;
}

static int rkcif_stream_start_rv1126b(struct rkcif_stream *stream, unsigned int mode)
{
	u32 val, mbus_flags, href_pol, vsync_pol,
	    yc_swap = 0, inputmode = 0,
	    multi_id = 0,
	    multi_id_en = BT656_1120_MULTI_ID_DISABLE_RV1126B,
	    multi_id_mode = BT656_1120_MULTI_ID_MODE_1_RV1126B,
	    multi_id_sel = BT656_1120_MULTI_ID_SEL_LSB_RV1126B,
	    bt1120_edge_mode = BT1120_CLOCK_SINGLE_EDGES,
	    bt1120_flags = 0;
	struct rkmodule_bt656_mbus_info bt1120_info;
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_sensor_info *sensor_info;
	struct v4l2_mbus_config *mbus;
	struct rkcif_dvp_sof_subdev *sof_sd = &dev->dvp_sof_subdev;
	unsigned int dma_en = 0;
	unsigned int dma_state = 0;
	u32 parse_type = 0;
	u32 output_type = 0;
	struct csi_channel_info *channel = &dev->channels[stream->id];

	if (stream->state < RKCIF_STATE_STREAMING) {
		stream->frame_idx = 0;
		stream->buf_wake_up_cnt = 0;
		stream->lack_buf_cnt = 0;
		stream->frame_phase = 0;
		stream->is_in_vblank = false;
		stream->is_change_toisp = false;
	}

	sensor_info = dev->active_sensor;
	mbus = &sensor_info->mbus;

	dma_state = stream->dma_en;
	if ((mode & RKCIF_STREAM_MODE_CAPTURE) == RKCIF_STREAM_MODE_CAPTURE)
		stream->dma_en |= RKCIF_DMAEN_BY_VICAP;
	else if ((mode & RKCIF_STREAM_MODE_TOISP_RDBK) == RKCIF_STREAM_MODE_TOISP_RDBK)
		stream->dma_en |= RKCIF_DMAEN_BY_ISP;
	else if ((mode & RKCIF_STREAM_MODE_ROCKIT) == RKCIF_STREAM_MODE_ROCKIT)
		stream->dma_en |= RKCIF_DMAEN_BY_ROCKIT;

	if (dma_state)
		return 0;

	mbus_flags = mbus->bus.parallel.flags;
	if ((mbus_flags & CIF_DVP_PCLK_DUAL_EDGE) == CIF_DVP_PCLK_DUAL_EDGE) {
		bt1120_edge_mode = BT1120_CLOCK_DOUBLE_EDGES_RV1126B;
		rkcif_enable_dvp_clk_dual_edge(dev, true);
	} else {
		bt1120_edge_mode = BT1120_CLOCK_SINGLE_EDGES_RV1126B;
		rkcif_enable_dvp_clk_dual_edge(dev, false);
	}

	if (mbus_flags & V4L2_MBUS_PCLK_SAMPLE_RISING)
		rkcif_config_dvp_clk_sampling_edge(dev, RKCIF_CLK_RISING);
	else
		rkcif_config_dvp_clk_sampling_edge(dev, RKCIF_CLK_FALLING);

	if (sensor_info->sd && mbus->type == V4L2_MBUS_BT656) {
		int ret;

		multi_id_en = BT656_1120_MULTI_ID_ENABLE_RV1126B;

		ret = v4l2_subdev_call(sensor_info->sd,
				       core, ioctl,
				       RKMODULE_GET_BT656_MBUS_INFO,
				       &bt1120_info);
		if (ret) {
			v4l2_warn(&dev->v4l2_dev,
				  "warning: no multi channel info for BT.656\n");
		} else {
			bt1120_flags = bt1120_info.flags;
			if (bt1120_flags & RKMODULE_CAMERA_BT656_PARSE_ID_LSB)
				multi_id_sel = BT656_1120_MULTI_ID_SEL_LSB_RV1126B;
			else
				multi_id_sel = BT656_1120_MULTI_ID_SEL_MSB_RV1126B;

			if (((bt1120_flags & RKMODULE_CAMERA_BT656_CHANNELS) >> 2) > 3)
				multi_id_mode = BT656_1120_MULTI_ID_MODE_4_RV1126B;
			else if (((bt1120_flags & RKMODULE_CAMERA_BT656_CHANNELS) >> 2) > 1)
				multi_id_mode = BT656_1120_MULTI_ID_MODE_2_RV1126B;
			multi_id = stream->id | (bt1120_info.id_en_bits << 4);
			rkcif_write_register(dev, get_dvp_reg_index_of_id_ctrl1(stream->id), multi_id);
		}
	}

	val = stream->pixm.plane_fmt[0].bytesperline;
	if (stream->crop_enable) {
		dev->channels[stream->id].crop_en = 1;
		dev->channels[stream->id].crop_st_x = stream->crop[CROP_SRC_ACT].left;
		dev->channels[stream->id].crop_st_y = stream->crop[CROP_SRC_ACT].top;
		dev->channels[stream->id].width = stream->crop[CROP_SRC_ACT].width;
		dev->channels[stream->id].height = stream->crop[CROP_SRC_ACT].height;
	} else {
		dev->channels[stream->id].crop_st_y = 0;
		dev->channels[stream->id].crop_st_x = 0;
		dev->channels[stream->id].width = stream->pixm.width;
		dev->channels[stream->id].height = stream->pixm.height;
		dev->channels[stream->id].crop_en = 0;
	}

	if (dev->chip_id > CHIP_RK3562_CIF && stream->sw_dbg_en)
		val = (val + 23) / 24 * 24;

	rkcif_write_register(dev, get_dvp_reg_index_of_vlw(stream->id), val);
	rkcif_write_register(dev, CIF_REG_DVP_SET_SIZE_ID0 + stream->id,
			     dev->channels[stream->id].width |
			     (dev->channels[stream->id].height << 16));
	rkcif_write_register(dev, get_dvp_reg_index_of_id_crop_start(stream->id),
			     dev->channels[stream->id].crop_st_y << CIF_CROP_Y_SHIFT |
			     dev->channels[stream->id].crop_st_x);

	rkcif_write_register(dev, CIF_REG_DVP_INTSTAT, 0x3c3ffff);
	rkcif_write_register_or(dev, CIF_REG_DVP_INTEN, 0x3c3ff0f);

	if (stream->dma_en) {
		if (mode == RKCIF_STREAM_MODE_CAPTURE)
			rkcif_assign_new_buffer_pingpong(stream,
					 RKCIF_YUV_ADDR_STATE_INIT,
					 stream->id);
		else if (mode == RKCIF_STREAM_MODE_TOISP ||
			 mode == RKCIF_STREAM_MODE_TOISP_RDBK)
			rkcif_assign_new_buffer_pingpong_toisp(stream,
					       RKCIF_YUV_ADDR_STATE_INIT,
					       stream->id);
		else if (mode == RKCIF_STREAM_MODE_ROCKIT)
			rkcif_assign_new_buffer_pingpong_rockit(stream,
						RKCIF_YUV_ADDR_STATE_INIT,
						stream->id);
	}

	dev->workmode = RKCIF_WORKMODE_PINGPONG;
	href_pol = (mbus_flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH) ?
		HSY_HIGH_ACTIVE : HSY_LOW_ACTIVE;
	vsync_pol = (mbus_flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH) ?
		VSY_HIGH_ACTIVE : VSY_LOW_ACTIVE;
	if ((inputmode & INPUT_BT1120_YUV422) == INPUT_BT1120_YUV422)
		if (CIF_FETCH_IS_Y_FIRST(stream->cif_fmt_in->dvp_fmt_val))
			yc_swap = BT1120_YC_SWAP_RV1126B;
	inputmode = rkcif_determine_input_mode_rv1126b(stream);
	val = ENABLE_CAPTURE
		| yc_swap
		| inputmode
		| bt1120_edge_mode
		| (href_pol << 6)
		| (vsync_pol << 6)
		| multi_id_en
		| multi_id_sel
		| multi_id_mode
		| DVP_SW_WATER_LINE_25_RV1126B;
	if (stream->sw_dbg_en)
		val |= BIT(31);
	rkcif_write_register(dev, CIF_REG_DVP_CTRL, val);

	channel->csi_fmt_val = stream->cif_fmt_in->csi_fmt_val;
	parse_type = rkcif_get_parse_type_rk3576(stream->cif_fmt_in, channel);
	output_type = rkcif_csi_get_output_type_mask_rk3576(stream);
	if (stream->dma_en)
		dma_en = CSI_DMA_ENABLE_RK3576;
	if (stream->lack_buf_cnt == 2)
		dma_en = 0;
	val = ENABLE_CAPTURE
		| CSI_ENABLE_CROP_RK3576
		| dma_en
		| parse_type
		| output_type
		| (stream->cif_fmt_in->csi_yuv_order >> 4)
		| BIT(22);
	if (stream->is_high_align)
		val |= CSI_HIGH_ALIGN_RK3576;
	else
		val &= ~CSI_HIGH_ALIGN_RK3576;
	if (stream->id == 0 && (stream->cif_fmt_out->fourcc == V4L2_PIX_FMT_NV12 ||
	   stream->cif_fmt_out->fourcc == V4L2_PIX_FMT_NV21))
		val |= CSI_UVDS_EN;
	if (stream->rounding_bit)
		val |= stream->rounding_bit;
	rkcif_write_register(dev, get_dvp_reg_index_of_id_ctrl0(stream->id), val);
	dev->intr_mask = rkcif_read_register(dev, CIF_REG_DVP_INTSTAT);
	atomic_set(&sof_sd->frm_sync_seq, 0);
	stream->state = RKCIF_STATE_STREAMING;
	stream->cifdev->dvp_sof_in_oneframe = 0;

	return 0;
}

static void rkcif_attach_sync_mode(struct rkcif_device *cifdev)
{
	struct rkcif_hw *hw = cifdev->hw_dev;
	struct rkcif_device *dev;
	struct sditf_priv *priv;
	int i = 0, j = 0;
	int ret = 0;
	int count = 0;
	int sync_type = NO_SYNC_MODE;
	int sync_group = 0;
	struct rkcif_sync_cfg sync_cfg;
	struct rkcif_multi_sync_config *sync_config;

	mutex_lock(&hw->dev_lock);
	if (cifdev->sditf_cnt <= 1) {
		ret = v4l2_subdev_call(cifdev->terminal_sensor.sd,
				       core, ioctl,
				       RKMODULE_GET_SYNC_MODE,
				       &sync_type);
		if (!ret)
			sync_cfg.type = sync_type;
		else
			sync_cfg.type = NO_SYNC_MODE;
		ret = v4l2_subdev_call(cifdev->terminal_sensor.sd,
				       core, ioctl,
				       RKMODULE_GET_GROUP_ID,
				       &sync_group);
		if (!ret && sync_group < RKCIF_MAX_GROUP)
			sync_cfg.group = sync_group;
		else
			sync_cfg.group = 0;
	} else {
		for (j = 0; j < cifdev->sditf_cnt; j++) {
			if (cifdev->sditf[j]->sensor_sd)
				ret |= v4l2_subdev_call(cifdev->sditf[j]->sensor_sd,
						core, ioctl,
						RKMODULE_GET_SYNC_MODE,
						&sync_type);
			else
				ret = -EINVAL;
			if (!ret && sync_type)
				break;
		}
		if (!ret)
			sync_cfg.type = sync_type;
		else
			sync_cfg.type = NO_SYNC_MODE;
		if (cifdev->sditf[0]->sensor_sd)
			ret = v4l2_subdev_call(cifdev->sditf[0]->sensor_sd,
					       core, ioctl,
					       RKMODULE_GET_GROUP_ID,
					       &sync_group);
		else
			ret = -EINVAL;
		if (!ret && sync_group < RKCIF_MAX_GROUP)
			sync_cfg.group = sync_group;
		else
			sync_cfg.group = 0;
	}
	cifdev->sync_cfg = sync_cfg;
	if (sync_cfg.type == NO_SYNC_MODE ||
	    hw->sync_config[sync_cfg.group].is_attach) {
		mutex_unlock(&hw->dev_lock);
		return;
	}

	sync_config = &hw->sync_config[sync_cfg.group];
	memset(sync_config, 0, sizeof(struct rkcif_multi_sync_config));
	for (i = 0; i < hw->dev_num; i++) {
		dev = hw->cif_dev[i];
		if (dev->sditf_cnt <= 1) {
			ret = v4l2_subdev_call(dev->terminal_sensor.sd,
					       core, ioctl,
					       RKMODULE_GET_SYNC_MODE,
					       &sync_type);
			if (!ret)
				sync_cfg.type = sync_type;
			else
				sync_cfg.type = NO_SYNC_MODE;
			ret = v4l2_subdev_call(dev->terminal_sensor.sd,
					       core, ioctl,
					       RKMODULE_GET_GROUP_ID,
					       &sync_group);
			if (!ret && sync_group < RKCIF_MAX_GROUP)
				sync_cfg.group = sync_group;
			else
				sync_cfg.group = 0;
		} else {
			priv = dev->sditf[0];
			if (priv && priv->is_combine_mode && dev->sditf_cnt <= RKCIF_MAX_SDITF) {
				for (j = 0; j < dev->sditf_cnt; j++) {
					if (dev->sditf[j]->sensor_sd)
						ret |= v4l2_subdev_call(dev->sditf[j]->sensor_sd,
								core, ioctl,
								RKMODULE_GET_SYNC_MODE,
								&sync_type);
					else
						ret = -EINVAL;
					if (!ret && sync_type) {
						priv = dev->sditf[j];
						break;
					}
				}
				if (!ret)
					sync_cfg.type = sync_type;
				else
					sync_cfg.type = NO_SYNC_MODE;
				if (priv->sensor_sd)
					ret = v4l2_subdev_call(priv->sensor_sd,
							       core, ioctl,
							       RKMODULE_GET_GROUP_ID,
							       &sync_group);
				else
					ret = -EINVAL;
				if (!ret && sync_group < RKCIF_MAX_GROUP)
					sync_cfg.group = sync_group;
				else
					sync_cfg.group = 0;
			}
		}
		if (sync_cfg.group == cifdev->sync_cfg.group) {
			if (sync_cfg.type == EXTERNAL_MASTER_MODE) {
				count = sync_config->ext_master.count;
				sync_config->ext_master.cif_dev[count] = dev;
				sync_config->ext_master.count++;
				sync_config->dev_cnt++;
				sync_config->sync_mask |= BIT(dev->csi_host_idx);
			} else if (sync_cfg.type == INTERNAL_MASTER_MODE) {
				count = sync_config->int_master.count;
				sync_config->int_master.cif_dev[count] = dev;
				sync_config->int_master.count++;
				sync_config->dev_cnt++;
				sync_config->sync_mask |= BIT(dev->csi_host_idx);
			} else if (sync_cfg.type == SLAVE_MODE) {
				count = sync_config->slave.count;
				sync_config->slave.cif_dev[count] = dev;
				sync_config->slave.count++;
				sync_config->dev_cnt++;
				sync_config->sync_mask |= BIT(dev->csi_host_idx);
			} else if (sync_cfg.type == SOFT_SYNC_MODE) {
				count = sync_config->soft_sync.count;
				sync_config->soft_sync.cif_dev[count] = dev;
				sync_config->soft_sync.count++;
				sync_config->dev_cnt++;
				sync_config->sync_mask |= BIT(dev->csi_host_idx);
			}
			dev->sync_cfg = sync_cfg;
		} else {
			ret = v4l2_subdev_call(dev->terminal_sensor.sd,
					       core, ioctl,
					       RKMODULE_GET_SYNC_MODE,
					       &sync_type);
		}
	}
	if (sync_config->int_master.count == 1) {
		if (sync_config->ext_master.count) {
			sync_config->mode = RKCIF_MASTER_MASTER;
			sync_config->is_attach = true;
		} else if (sync_config->slave.count) {
			sync_config->mode = RKCIF_MASTER_SLAVE;
			sync_config->is_attach = true;
		} else {
			dev_info(hw->dev,
				 "Missing slave device, do not use sync mode\n");
		}
		if (sync_config->is_attach)
			dev_info(hw->dev,
				 "group %d, int_master %d, ext_master %d, slave %d\n",
				 i,
				 sync_config->int_master.count,
				 sync_config->ext_master.count,
				 sync_config->slave.count);
	} else if (sync_config->soft_sync.count > 1) {
		sync_config->mode = RKCIF_SOFT_SYNC;
		sync_config->is_attach = true;
		dev_info(hw->dev, "group used soft sync mode\n");
	}
	mutex_unlock(&hw->dev_lock);
}

static void rkcif_monitor_reset_event(struct rkcif_device *dev);

int rkcif_do_start_stream(struct rkcif_stream *stream, enum rkcif_stream_mode mode)
{
	struct rkcif_vdev_node *node = &stream->vnode;
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_hw *hw_dev = dev->hw_dev;
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	struct rkcif_sensor_info *sensor_info = dev->active_sensor;
	struct rkcif_sensor_info *terminal_sensor = NULL;
	struct rkmodule_hdr_cfg hdr_cfg;
	struct rkcif_csi_info csi_info = {0};
	int rkmodule_stream_seq = RKMODULE_START_STREAM_DEFAULT;
	int ret;
	int i = 0;
	u32 skip_frame = 0;
	int on = 1;

	v4l2_info(&dev->v4l2_dev, "stream[%d] start streaming\n", stream->id);

	rkcif_attach_sync_mode(dev);
	mutex_lock(&dev->stream_lock);

	if ((stream->cur_stream_mode & RKCIF_STREAM_MODE_CAPTURE) == mode) {
		ret = -EBUSY;
		v4l2_err(v4l2_dev, "stream in busy state\n");
		goto destroy_buf;
	}
	if (stream->dma_en == 0)
		stream->fs_cnt_in_single_frame = 0;

	if (dev->wait_line_cache != 0) {
		dev->wait_line = dev->wait_line_cache;
		stream->is_line_wake_up = true;
	}

	if (stream->is_line_wake_up)
		stream->is_line_inten = true;
	else
		stream->is_line_inten = false;

	if (!dev->active_sensor) {
		ret = rkcif_update_sensor_info(stream);
		if (ret < 0) {
			v4l2_err(v4l2_dev,
				 "update sensor info failed %d\n",
				 ret);
			goto out;
		}
	}
	terminal_sensor = &dev->terminal_sensor;
	if (terminal_sensor->sd) {
		ret = v4l2_subdev_call(terminal_sensor->sd,
				       core, ioctl,
				       RKMODULE_GET_HDR_CFG,
				       &hdr_cfg);
		if (!ret)
			dev->hdr = hdr_cfg;
		else
			dev->hdr.hdr_mode = NO_HDR;

		ret = v4l2_subdev_call(terminal_sensor->sd,
				       video, g_frame_interval, &terminal_sensor->fi);
		if (ret)
			terminal_sensor->fi.interval = (struct v4l2_fract) {1, 30};

		ret = v4l2_subdev_call(terminal_sensor->sd,
				       core, ioctl,
				       RKMODULE_GET_START_STREAM_SEQ,
				       &rkmodule_stream_seq);
		if (ret)
			rkmodule_stream_seq = RKMODULE_START_STREAM_DEFAULT;

		rkcif_sync_crop_info(stream);
	}

	ret = rkcif_sanity_check_fmt(stream, NULL);
	if (ret < 0)
		goto destroy_buf;

	mutex_lock(&hw_dev->dev_lock);
	if (atomic_read(&dev->pipe.stream_cnt) == 0 &&
	    dev->active_sensor &&
	    (dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_DPHY ||
	     dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_CPHY ||
	     dev->active_sensor->mbus.type == V4L2_MBUS_CCP2)) {
		if (dev->channels[0].capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
			csi_info.csi_num = dev->channels[0].capture_info.multi_dev.dev_num;
			if (csi_info.csi_num > RKCIF_MAX_CSI_NUM) {
				v4l2_err(v4l2_dev,
					 "csi num %d, max %d\n",
					 csi_info.csi_num, RKCIF_MAX_CSI_NUM);
				goto out;
			}
			for (i = 0; i < csi_info.csi_num; i++) {
				csi_info.csi_idx[i] = dev->channels[0].capture_info.multi_dev.dev_idx[i];
				if (dev->hw_dev->is_rk3588s2)
					v4l2_info(v4l2_dev, "rk3588s2 combine mode attach to mipi%d\n",
						  csi_info.csi_idx[i]);
			}
		} else {
			csi_info.csi_num = 1;
			if (!dev->switch_info.is_use_switch)
				dev->csi_host_idx = dev->csi_host_idx_def;
			csi_info.csi_idx[0] = dev->csi_host_idx;
		}
		dev->csi_info = csi_info;
		ret = v4l2_subdev_call(dev->active_sensor->sd,
				       core, ioctl,
				       RKCIF_CMD_SET_CSI_IDX,
				       &csi_info);
		if (ret)
			v4l2_err(&dev->v4l2_dev, "set csi idx %d fail\n", dev->csi_host_idx);

	}

	if (((dev->active_sensor && dev->active_sensor->mbus.type == V4L2_MBUS_BT656) ||
	     dev->is_use_dummybuf) &&
	    (!dev->hw_dev->dummy_buf.vaddr) &&
	    mode == RKCIF_STREAM_MODE_CAPTURE) {
		ret = rkcif_create_dummy_buf(stream);
		if (ret < 0) {
			mutex_unlock(&hw_dev->dev_lock);
			v4l2_err(v4l2_dev, "Failed to create dummy_buf, %d\n", ret);
			goto destroy_buf;
		}
	}
	mutex_unlock(&hw_dev->dev_lock);

	if (mode == RKCIF_STREAM_MODE_CAPTURE)
		tasklet_enable(&stream->vb_done_tasklet);

	if (stream->cur_stream_mode == RKCIF_STREAM_MODE_NONE) {
		ret = dev->pipe.open(&dev->pipe, &node->vdev.entity, true);
		if (ret < 0) {
			v4l2_err(v4l2_dev, "open cif pipeline failed %d\n",
				 ret);
			goto disable_tasklet;
		}

		/*
		 * start sub-devices
		 * When use bt601, the sampling edge of cif is random,
		 * can be rising or fallling after powering on cif.
		 * To keep the coherence of edge, open sensor in advance.
		 */
		if (sensor_info->mbus.type == V4L2_MBUS_PARALLEL ||
		    rkmodule_stream_seq == RKMODULE_START_STREAM_FRONT) {
			ret = dev->pipe.set_stream(&dev->pipe, true);
			if (ret < 0)
				goto disable_tasklet;
		}
		ret = v4l2_subdev_call(terminal_sensor->sd,
				       core, ioctl,
				       RKMODULE_GET_SKIP_FRAME,
				       &skip_frame);
		if (!ret && skip_frame < RKCIF_SKIP_FRAME_MAX)
			stream->skip_frame = skip_frame;
		else
			stream->skip_frame = 0;
		stream->cur_skip_frame = stream->skip_frame;
		if (stream->id == RKCIF_STREAM_MIPI_ID0 && dev->is_support_get_exp) {
			ret = kfifo_alloc(&stream->exp_kfifo,
					  sizeof(struct rkcif_sensor_exp) * RKCIF_EXP_NUM_MAX, GFP_KERNEL);
			if (ret < 0)
				goto disable_tasklet;
			ret = kfifo_alloc(&stream->gain_kfifo,
					  sizeof(struct rkcif_sensor_gain) * RKCIF_EXP_NUM_MAX, GFP_KERNEL);
			if (ret < 0)
				goto disable_tasklet;
			ret = kfifo_alloc(&stream->vts_kfifo,
					  sizeof(struct rkcif_sensor_vts) * RKCIF_EXP_NUM_MAX, GFP_KERNEL);
			if (ret < 0)
				goto disable_tasklet;
			ret = kfifo_alloc(&stream->dcg_kfifo,
					  sizeof(struct rkcif_sensor_dcg) * RKCIF_EXP_NUM_MAX, GFP_KERNEL);
			if (ret < 0)
				goto disable_tasklet;
			ret = v4l2_subdev_call(terminal_sensor->sd,
				       core, ioctl,
				       RKMODULE_GET_EXP_DELAY,
				       &stream->exp_delay);
			if (ret) {
				stream->exp_delay.exp_delay = 2;
				stream->exp_delay.gain_delay = 2;
				stream->exp_delay.vts_delay = 2;
				stream->exp_delay.dcg_delay = 1;
			}
		}
	}
	if (dev->chip_id >= CHIP_RK1808_CIF) {
		if (dev->active_sensor &&
		    (dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_DPHY ||
		    dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_CPHY ||
		    dev->active_sensor->mbus.type == V4L2_MBUS_CCP2)) {
			ret = rkcif_csi_stream_start(stream, mode);
		} else {
			if (dev->chip_id >= CHIP_RV1126B_CIF)
				ret = rkcif_stream_start_rv1126b(stream, mode);
			else
				ret = rkcif_stream_start(stream, mode);
		}
	} else {
		ret = rkcif_stream_start(stream, mode);
	}
	if (ret < 0)
		goto disable_tasklet;
	if (stream->cur_stream_mode == RKCIF_STREAM_MODE_NONE &&
	    dev->channels[0].capture_info.mode == RKMODULE_ONE_CH_TO_MULTI_ISP &&
	    stream->id == 0) {
		for (i = 0; i < dev->sditf_cnt; i++)
			sditf_get_default_exp(dev->sditf[i]);
		schedule_work(&dev->exp_work);
	}

	if (stream->cur_stream_mode == RKCIF_STREAM_MODE_NONE) {
		ret = video_device_pipeline_start(&node->vdev, &dev->pipe.pipe);
		if (ret < 0) {
			v4l2_err(&dev->v4l2_dev, "start pipeline failed %d\n",
				 ret);
			goto pipe_stream_off;
		}

		if (sensor_info->mbus.type != V4L2_MBUS_PARALLEL &&
		    rkmodule_stream_seq != RKMODULE_START_STREAM_FRONT) {
			ret = dev->pipe.set_stream(&dev->pipe, true);
			if (ret < 0)
				goto stop_stream;
		}
		if (dev->is_camera_over_bridge) {
			ret = v4l2_subdev_call(dev->sditf[stream->id]->sensor_sd,
					       video,
					       s_stream,
					       on);
			if (ret < 0)
				goto stop_stream;
		}
	}
	if (dev->chip_id == CHIP_RV1126_CIF ||
	    dev->chip_id == CHIP_RV1126_CIF_LITE ||
	    dev->chip_id == CHIP_RK3568_CIF) {
		if (dev->hdr.hdr_mode == NO_HDR) {
			if (dev->stream[RKCIF_STREAM_MIPI_ID0].state == RKCIF_STATE_STREAMING)
				rkcif_start_luma(&dev->luma_vdev,
						dev->stream[RKCIF_STREAM_MIPI_ID0].cif_fmt_in);
		} else if (dev->hdr.hdr_mode == HDR_X2) {
			if (dev->stream[RKCIF_STREAM_MIPI_ID0].state == RKCIF_STATE_STREAMING &&
			    dev->stream[RKCIF_STREAM_MIPI_ID1].state == RKCIF_STATE_STREAMING)
				rkcif_start_luma(&dev->luma_vdev,
					dev->stream[RKCIF_STREAM_MIPI_ID0].cif_fmt_in);
		} else if (dev->hdr.hdr_mode == HDR_X3) {
			if (dev->stream[RKCIF_STREAM_MIPI_ID0].state == RKCIF_STATE_STREAMING &&
			    dev->stream[RKCIF_STREAM_MIPI_ID1].state == RKCIF_STATE_STREAMING &&
			    dev->stream[RKCIF_STREAM_MIPI_ID2].state == RKCIF_STATE_STREAMING)
				rkcif_start_luma(&dev->luma_vdev,
					dev->stream[RKCIF_STREAM_MIPI_ID0].cif_fmt_in);
		}
	}
	dev->reset_work_cancel = false;
	stream->cur_stream_mode |= mode;
	rkcif_monitor_reset_event(dev);
	goto out;

stop_stream:
	rkcif_stream_stop(stream);
pipe_stream_off:
	dev->pipe.set_stream(&dev->pipe, false);

disable_tasklet:
	if (mode == RKCIF_STREAM_MODE_CAPTURE)
		tasklet_disable(&stream->vb_done_tasklet);
destroy_buf:
	if (stream->curr_buf)
		list_add_tail(&stream->curr_buf->queue, &stream->buf_head);
	if (stream->next_buf &&
	    stream->next_buf != stream->curr_buf)
		list_add_tail(&stream->next_buf->queue, &stream->buf_head);

	stream->curr_buf = NULL;
	stream->next_buf = NULL;
	atomic_set(&stream->buf_cnt, 0);
	while (!list_empty(&stream->buf_head)) {
		struct rkcif_buffer *buf;

		buf = list_first_entry(&stream->buf_head,
				       struct rkcif_buffer, queue);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
		list_del(&buf->queue);
	}

out:
	mutex_unlock(&dev->stream_lock);
	return ret;
}

static int rkcif_start_streaming(struct vb2_queue *queue, unsigned int count)
{
	struct rkcif_stream *stream = queue->drv_priv;
	int ret = 0;

	ret = rkcif_do_start_stream(stream, RKCIF_STREAM_MODE_CAPTURE);
	return ret;
}

static struct vb2_ops rkcif_vb2_ops = {
	.queue_setup = rkcif_queue_setup,
	.buf_queue = rkcif_buf_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.stop_streaming = rkcif_stop_streaming,
	.start_streaming = rkcif_start_streaming,
};

static int rkcif_init_vb2_queue(struct vb2_queue *q,
				struct rkcif_stream *stream,
				enum v4l2_buf_type buf_type)
{
	struct rkcif_hw *hw_dev = stream->cifdev->hw_dev;

	q->type = buf_type;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_USERPTR;
	q->drv_priv = stream;
	q->ops = &rkcif_vb2_ops;
	q->mem_ops = hw_dev->mem_ops;
	q->buf_struct_size = sizeof(struct rkcif_buffer);
	if (stream->cifdev->is_use_dummybuf)
		q->min_buffers_needed = 1;
	else
		q->min_buffers_needed = CIF_REQ_BUFS_MIN;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &stream->vnode.vlock;
	q->dev = hw_dev->dev;
	q->allow_cache_hints = 1;
	q->bidirectional = 1;
	q->gfp_flags = GFP_DMA32;
	if (hw_dev->is_dma_contig)
		q->dma_attrs = DMA_ATTR_FORCE_CONTIGUOUS;
	return vb2_queue_init(q);
}

int rkcif_set_fmt(struct rkcif_stream *stream,
		  struct v4l2_pix_format_mplane *pixm,
		  bool try)
{
	struct rkcif_device *dev = stream->cifdev;
	struct sditf_priv *priv = dev->sditf[0];
	const struct cif_output_fmt *fmt;
	const struct cif_input_fmt *cif_fmt_in = NULL;
	struct v4l2_rect input_rect;
	unsigned int imagesize = 0, ex_imagesize = 0, planes;
	u32 xsubs = 1, ysubs = 1, i;
	struct rkmodule_hdr_cfg hdr_cfg;
	struct rkcif_extend_info *extend_line = &stream->extend_line;
	struct csi_channel_info *channel_info = &dev->channels[stream->id];
	int ret;

	for (i = 0; i < RKCIF_MAX_PLANE; i++)
		memset(&pixm->plane_fmt[i], 0, sizeof(struct v4l2_plane_pix_format));

	fmt = rkcif_find_output_fmt(stream, pixm->pixelformat);
	if (!fmt)
		fmt = &out_fmts[0];

	input_rect.width = RKCIF_DEFAULT_WIDTH;
	input_rect.height = RKCIF_DEFAULT_HEIGHT;

	if (dev->terminal_sensor.sd) {
		cif_fmt_in = rkcif_get_input_fmt(dev,
						 &input_rect, stream->id,
						 channel_info);
		stream->cif_fmt_in = cif_fmt_in;
	} else {
		v4l2_err(&stream->cifdev->v4l2_dev,
			 "terminal subdev does not exist\n");
		return -EINVAL;
	}

	ret = rkcif_output_fmt_check(stream, fmt);
	if (ret)
		return -EINVAL;

	if (dev->terminal_sensor.sd) {
		ret = v4l2_subdev_call(dev->terminal_sensor.sd,
				       core, ioctl,
				       RKMODULE_GET_HDR_CFG,
				       &hdr_cfg);
		if (!ret)
			dev->hdr = hdr_cfg;
		else
			dev->hdr.hdr_mode = NO_HDR;

		dev->terminal_sensor.raw_rect = input_rect;
		if (atomic_read(&dev->pipe.stream_cnt) == 0) {
			ret = v4l2_subdev_call(dev->terminal_sensor.sd, video,
					       g_frame_interval, &dev->terminal_sensor.src_fi);
			if (ret) {
				v4l2_err(&stream->cifdev->v4l2_dev,
					 "%s: get terminal %s g_frame_interval failed!\n",
					 __func__, dev->terminal_sensor.sd->name);
				return ret;
			}
		}
	}

	/* CIF has not scale function,
	 * the size should not be larger than input
	 */
	pixm->width = clamp_t(u32, pixm->width,
			      CIF_MIN_WIDTH, input_rect.width);
	pixm->height = clamp_t(u32, pixm->height,
			       CIF_MIN_HEIGHT, input_rect.height);
	pixm->num_planes = fmt->mplanes;
	pixm->field = V4L2_FIELD_NONE;
	pixm->quantization = V4L2_QUANTIZATION_DEFAULT;

	rkcif_sync_crop_info(stream);
	/* calculate plane size and image size */
	fcc_xysubs(fmt->fourcc, &xsubs, &ysubs);

	planes = fmt->cplanes ? fmt->cplanes : fmt->mplanes;

	if (cif_fmt_in &&
	    (cif_fmt_in->mbus_code == MEDIA_BUS_FMT_SPD_2X8 ||
	     cif_fmt_in->mbus_code == MEDIA_BUS_FMT_EBD_1X8))
		stream->crop_enable = false;

	for (i = 0; i < planes; i++) {
		struct v4l2_plane_pix_format *plane_fmt;
		int width, height, bpl, size, bpp, ex_size;

		if (i == 0) {
			if (stream->crop_enable) {
				width = stream->crop[CROP_SRC_ACT].width;
				height = stream->crop[CROP_SRC_ACT].height;
			} else {
				width = pixm->width;
				height = pixm->height;
			}
		} else {
			if (stream->crop_enable) {
				width = stream->crop[CROP_SRC_ACT].width / xsubs;
				height = stream->crop[CROP_SRC_ACT].height / ysubs;
			} else {
				width = pixm->width / xsubs;
				height = pixm->height / ysubs;
			}
		}

		if (priv && priv->is_combine_mode && dev->sditf_cnt <= RKCIF_MAX_SDITF)
			height *= dev->sditf_cnt;

		extend_line->pixm.height = height + RKMODULE_EXTEND_LINE;

		/* compact mode need bytesperline 4bytes align,
		 * align 8 to bring into correspondence with virtual width.
		 * to optimize reading and writing of ddr, aliged with 256.
		 */
		if (fmt->fmt_type == CIF_FMT_TYPE_RAW &&
		    cif_fmt_in &&
		    (cif_fmt_in->mbus_code == MEDIA_BUS_FMT_EBD_1X8 ||
		     cif_fmt_in->mbus_code == MEDIA_BUS_FMT_SPD_2X8)) {
			stream->is_compact = false;
		}

		if (fmt->fmt_type == CIF_FMT_TYPE_RAW && stream->is_compact &&
		    (dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_DPHY ||
		     dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_CPHY ||
		     dev->active_sensor->mbus.type == V4L2_MBUS_CCP2) &&
		     fmt->csi_fmt_val != CSI_WRDDR_TYPE_RGB888 &&
		     fmt->csi_fmt_val != CSI_WRDDR_TYPE_RGB565) {
			bpl = ALIGN(width * fmt->raw_bpp / 8, 256);
		} else {
			if (fmt->fmt_type == CIF_FMT_TYPE_RAW && stream->is_compact &&
			    fmt->csi_fmt_val != CSI_WRDDR_TYPE_RGB888 &&
			    fmt->csi_fmt_val != CSI_WRDDR_TYPE_RGB565 &&
			    dev->chip_id >= CHIP_RK3588_CIF) {
				bpl = ALIGN(width * fmt->raw_bpp / 8, 256);
			} else {
				bpp = rkcif_align_bits_per_pixel(stream, fmt, i);
				bpl = width * bpp / CIF_YUV_STORED_BIT_WIDTH;
			}
		}
		if (dev->chip_id > CHIP_RK3562_CIF && stream->sw_dbg_en)
			bpl = (bpl + 23) / 24 * 24;
		size = bpl * height;
		imagesize += size;
		ex_size = bpl * extend_line->pixm.height;
		ex_imagesize += ex_size;

		if (fmt->mplanes > i) {
			/* Set bpl and size for each mplane */
			plane_fmt = pixm->plane_fmt + i;
			plane_fmt->bytesperline = bpl;
			plane_fmt->sizeimage = size;

			plane_fmt = extend_line->pixm.plane_fmt + i;
			plane_fmt->bytesperline = bpl;
			plane_fmt->sizeimage = ex_size;
		}
		v4l2_dbg(1, rkcif_debug, &stream->cifdev->v4l2_dev,
			 "C-Plane %i size: %d, Total imagesize: %d\n",
			 i, size, imagesize);
	}

	/* convert to non-MPLANE format.
	 * It's important since we want to unify non-MPLANE
	 * and MPLANE.
	 */
	if (fmt->mplanes == 1) {
		pixm->plane_fmt[0].sizeimage = imagesize;
		extend_line->pixm.plane_fmt[0].sizeimage = ex_imagesize;
	}

	if (!try) {
		stream->cif_fmt_out = fmt;
		stream->pixm = *pixm;

		v4l2_dbg(1, rkcif_debug, &stream->cifdev->v4l2_dev,
			 "%s: req(%d, %d) out(%d, %d)\n", __func__,
			 pixm->width, pixm->height,
			 stream->pixm.width, stream->pixm.height);
	}
	return 0;
}

void rkcif_stream_init(struct rkcif_device *dev, u32 id)
{
	struct rkcif_stream *stream = &dev->stream[id];
	struct v4l2_pix_format_mplane pixm;
	int i;

	memset(stream, 0, sizeof(*stream));
	memset(&pixm, 0, sizeof(pixm));
	stream->id = id;
	stream->cifdev = dev;

	INIT_LIST_HEAD(&stream->buf_head);
	INIT_LIST_HEAD(&stream->buf_head_multi_cache);
	INIT_LIST_HEAD(&stream->rx_buf_head);
	INIT_LIST_HEAD(&stream->rx_buf_head_vicap);
	INIT_LIST_HEAD(&stream->rockit_buf_head);
	spin_lock_init(&stream->vbq_lock);
	spin_lock_init(&stream->fps_lock);
	stream->state = RKCIF_STATE_READY;
	init_waitqueue_head(&stream->wq_stopped);

	/* Set default format */
	pixm.pixelformat = V4L2_PIX_FMT_NV12;
	pixm.width = RKCIF_DEFAULT_WIDTH;
	pixm.height = RKCIF_DEFAULT_HEIGHT;
	rkcif_set_fmt(stream, &pixm, false);

	for (i = 0; i < CROP_SRC_MAX; i++) {
		stream->crop[i].left = 0;
		stream->crop[i].top = 0;
		stream->crop[i].width = RKCIF_DEFAULT_WIDTH;
		stream->crop[i].height = RKCIF_DEFAULT_HEIGHT;
	}

	stream->crop_enable = false;
	stream->crop_dyn_en = false;
	stream->crop_mask = 0x0;

	if (dev->inf_id == RKCIF_DVP) {
		if (dev->chip_id <= CHIP_RK3568_CIF)
			stream->is_compact = false;
		else
			stream->is_compact = true;
	} else {
		if (dev->chip_id >= CHIP_RV1126_CIF)
			stream->is_compact = true;
		else
			stream->is_compact = false;
	}

	stream->is_high_align = false;
	stream->is_finish_stop_dma = false;
	stream->is_wait_dma_stop = false;

	if (dev->chip_id == CHIP_RV1126_CIF ||
	    dev->chip_id == CHIP_RV1126_CIF_LITE)
		stream->extend_line.is_extended = true;
	else
		stream->extend_line.is_extended = false;

	stream->is_dvp_yuv_addr_init = false;
	stream->is_fs_fe_not_paired = false;
	stream->fs_cnt_in_single_frame = 0;
	if (dev->wait_line) {
		dev->wait_line_cache = dev->wait_line;
		dev->wait_line_bak = dev->wait_line;
		stream->is_line_wake_up = true;
	} else {
		stream->is_line_wake_up = false;
		dev->wait_line_cache = 0;
		dev->wait_line_bak = 0;
	}
	stream->cur_stream_mode = 0;
	stream->dma_en = 0;
	stream->to_en_dma = 0;
	stream->to_stop_dma = 0;
	stream->to_en_scale = false;
	stream->buf_owner = 0;
	stream->buf_replace_cnt = 0;
	stream->is_stop_capture = false;
	stream->is_single_cap = false;
	atomic_set(&stream->buf_cnt, 0);
	stream->rx_buf_num = 0;
	init_completion(&stream->stop_complete);
	init_completion(&stream->start_complete);
	stream->is_wait_stop_complete = false;
	stream->thunderboot_skip_interval = get_rk_cam_skip();
	atomic_set(&stream->sub_stream_buf_cnt, 0);
	spin_lock_init(&stream->fence_lock);
	rkcif_fence_context_init(&stream->fence_ctx);
	INIT_LIST_HEAD(&stream->qbuf_fence_list_head);
	INIT_LIST_HEAD(&stream->done_fence_list_head);
	stream->low_latency = false;
	stream->rkcif_fence = NULL;
	stream->rounding_bit = 0;
	stream->is_m_online_fb_res = false;
	stream->is_fb_first_frame = true;
	stream->frame_idx = 0;
	memset(&stream->sensor_exp_info, 0, sizeof(stream->sensor_exp_info));
	stream->frame_loss = 0;
	stream->is_pause_stream = false;
}

int rkcif_sensor_set_power(struct rkcif_stream *stream, int on)
{
	struct rkcif_device *cif_dev = stream->cifdev;
	struct sditf_priv *priv = cif_dev->sditf[0];
	int i = 0;

	if (!on && atomic_dec_if_positive(&cif_dev->sd_power_cnt))
		return 0;

	if (on && atomic_inc_return(&cif_dev->sd_power_cnt) > 1)
		return 0;

	if (cif_dev->terminal_sensor.sd)
		v4l2_subdev_call(cif_dev->terminal_sensor.sd,
				 core, s_power, on);
	if (priv && cif_dev->sditf_cnt > 1) {
		if (priv->is_combine_mode) {
			for (i = 0; i < cif_dev->sditf_cnt; i++) {
				if (cif_dev->sditf[i] && cif_dev->sditf[i]->sensor_sd)
					v4l2_subdev_call(cif_dev->sditf[i]->sensor_sd, core,
							 s_power, on);
			}
		} else if (cif_dev->is_camera_over_bridge) {
			if (stream->id < cif_dev->sditf_cnt)
				v4l2_subdev_call(cif_dev->sditf[stream->id]->sensor_sd, core,
						 s_power, on);
		}
	}
	return 0;
}

static int rkcif_fh_open(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct rkcif_vdev_node *vnode = vdev_to_node(vdev);
	struct rkcif_stream *stream = to_rkcif_stream(vnode);
	struct rkcif_device *cifdev = stream->cifdev;
	int ret;
	int on = 1;

	ret = rkcif_attach_hw(cifdev);
	if (ret)
		return ret;

	if (cifdev->is_thunderboot) {
		ret = rkisp_cond_poll_timeout(cifdev->is_thunderboot_start,
					      2000, 5000 * USEC_PER_MSEC);
		if (ret) {
			mutex_lock(&cifdev->stream_lock);
			cifdev->is_thunderboot = false;
			mutex_unlock(&cifdev->stream_lock);
			return -EINVAL;
		}
	}
	/* Make sure active sensor is valid before .set_fmt() */
	ret = rkcif_update_sensor_info(stream);
	if (ret < 0) {
		v4l2_err(vdev,
			 "update sensor info failed %d\n",
			 ret);

		return ret;
	}

	ret = pm_runtime_resume_and_get(cifdev->dev);
	if (ret < 0) {
		v4l2_err(vdev, "Failed to get runtime pm, %d\n",
			 ret);
		return ret;
	}

	ret = v4l2_fh_open(filp);
	if (!ret) {
		mutex_lock(&cifdev->stream_lock);
		ret = v4l2_pipeline_pm_get(&vnode->vdev.entity);
		v4l2_dbg(1, rkcif_debug, vdev, "open video, entity use_count %d\n",
			 vnode->vdev.entity.use_count);
		ret = rkcif_sensor_set_power(stream, on);
		mutex_unlock(&cifdev->stream_lock);
		if (ret < 0)
			vb2_fop_release(filp);
	}
	return ret;
}

static int rkcif_fh_release(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct rkcif_vdev_node *vnode = vdev_to_node(vdev);
	struct rkcif_stream *stream = to_rkcif_stream(vnode);
	struct rkcif_device *cifdev = stream->cifdev;
	int ret = 0;
	int on = 0;

	ret = vb2_fop_release(filp);
	if (!ret) {
		mutex_lock(&cifdev->stream_lock);
		ret = rkcif_sensor_set_power(stream, on);
		v4l2_pipeline_pm_put(&vnode->vdev.entity);
		v4l2_dbg(1, rkcif_debug, vdev, "close video, entity use_count %d\n",
			 vnode->vdev.entity.use_count);
		mutex_unlock(&cifdev->stream_lock);
	}

	pm_runtime_put_sync(cifdev->dev);
	return ret;
}

static const struct v4l2_file_operations rkcif_fops = {
	.open = rkcif_fh_open,
	.release = rkcif_fh_release,
	.unlocked_ioctl = video_ioctl2,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = video_ioctl2,
#endif
};

static int rkcif_enum_input(struct file *file, void *priv,
			    struct v4l2_input *input)
{
	if (input->index > 0)
		return -EINVAL;

	input->type = V4L2_INPUT_TYPE_CAMERA;
	strscpy(input->name, "Camera", sizeof(input->name));

	return 0;
}

static int rkcif_try_fmt_vid_cap_mplane(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct rkcif_stream *stream = video_drvdata(file);
	int ret = 0;

	ret = rkcif_set_fmt(stream, &f->fmt.pix_mp, true);

	return ret;
}

static int rkcif_enum_framesizes(struct file *file, void *prov,
				 struct v4l2_frmsizeenum *fsize)
{
	struct v4l2_frmsize_discrete *d = &fsize->discrete;
	struct v4l2_frmsize_stepwise *s = &fsize->stepwise;
	struct rkcif_stream *stream = video_drvdata(file);
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_rect input_rect;
	struct csi_channel_info csi_info;

	if (fsize->index != 0)
		return -EINVAL;

	if (!rkcif_find_output_fmt(stream, fsize->pixel_format))
		return -EINVAL;

	input_rect.width = RKCIF_DEFAULT_WIDTH;
	input_rect.height = RKCIF_DEFAULT_HEIGHT;

	if (dev->terminal_sensor.sd)
		rkcif_get_input_fmt(dev,
				    &input_rect, stream->id,
				    &csi_info);

	if (dev->hw_dev->adapt_to_usbcamerahal) {
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		d->width = input_rect.width;
		d->height = input_rect.height;
	} else {
		fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		s->min_width = CIF_MIN_WIDTH;
		s->min_height = CIF_MIN_HEIGHT;
		s->max_width = input_rect.width;
		s->max_height = input_rect.height;
		s->step_width = OUTPUT_STEP_WISE;
		s->step_height = OUTPUT_STEP_WISE;
	}

	return 0;
}

static int rkcif_enum_frameintervals(struct file *file, void *fh,
				     struct v4l2_frmivalenum *fival)
{
	struct rkcif_stream *stream = video_drvdata(file);
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_sensor_info *sensor = dev->active_sensor;
	struct v4l2_subdev_frame_interval fi;
	int ret;

	if (fival->index != 0)
		return -EINVAL;

	if (!sensor || !sensor->sd) {
		/* TODO: active_sensor is NULL if using DMARX path */
		v4l2_err(&dev->v4l2_dev, "%s Not active sensor\n", __func__);
		return -ENODEV;
	}

	ret = v4l2_subdev_call(sensor->sd, video, g_frame_interval, &fi);
	if (ret && ret != -ENOIOCTLCMD) {
		return ret;
	} else if (ret == -ENOIOCTLCMD) {
		/* Set a default value for sensors not implements ioctl */
		fi.interval.numerator = 1;
		fi.interval.denominator = 30;
	}

	if (dev->hw_dev->adapt_to_usbcamerahal) {
		fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
		fival->discrete.numerator = fi.interval.numerator;
		fival->discrete.denominator = fi.interval.denominator;
	} else {
		fival->type = V4L2_FRMIVAL_TYPE_CONTINUOUS;
		fival->stepwise.step.numerator = 1;
		fival->stepwise.step.denominator = 1;
		fival->stepwise.max.numerator = 1;
		fival->stepwise.max.denominator = 1;
		fival->stepwise.min.numerator = fi.interval.numerator;
		fival->stepwise.min.denominator = fi.interval.denominator;
	}

	return 0;
}

static int rkcif_enum_fmt_vid_cap_mplane(struct file *file, void *priv,
					 struct v4l2_fmtdesc *f)
{
	const struct cif_output_fmt *fmt = NULL;
	struct rkcif_stream *stream = video_drvdata(file);
	struct rkcif_device *dev = stream->cifdev;
	const struct cif_input_fmt *cif_fmt_in = NULL;
	struct v4l2_rect input_rect;
	int i = 0;
	int ret = 0;
	int fource_idx = 0;

	if (f->index >= ARRAY_SIZE(out_fmts))
		return -EINVAL;

	if (dev->terminal_sensor.sd) {
		cif_fmt_in = rkcif_get_input_fmt(dev,
					   &input_rect, stream->id,
					   &dev->channels[stream->id]);
		stream->cif_fmt_in = cif_fmt_in;
	} else {
		v4l2_err(&stream->cifdev->v4l2_dev,
			 "terminal subdev does not exist\n");
		return -EINVAL;
	}

	if (f->index != 0)
		fource_idx = stream->new_fource_idx;

	for (i = fource_idx; i < ARRAY_SIZE(out_fmts); i++) {
		fmt = &out_fmts[i];
		ret = rkcif_output_fmt_check(stream, fmt);
		if (!ret) {
			f->pixelformat = fmt->fourcc;
			stream->new_fource_idx = i + 1;
			break;
		}
	}
	if (i == ARRAY_SIZE(out_fmts))
		return -EINVAL;

	switch (f->pixelformat) {
	case V4l2_PIX_FMT_EBD8:
		strscpy(f->description,
			"Embedded data 8-bit",
			sizeof(f->description));
		break;
	case V4l2_PIX_FMT_SPD16:
		strscpy(f->description,
			"Shield pix data 16-bit",
			sizeof(f->description));
		break;
	default:
		break;
	}
	return 0;
}

static int rkcif_s_fmt_vid_cap_mplane(struct file *file,
				      void *priv, struct v4l2_format *f)
{
	struct rkcif_stream *stream = video_drvdata(file);
	struct rkcif_device *dev = stream->cifdev;
	int ret = 0;

	if (vb2_is_busy(&stream->vnode.buf_queue)) {
		v4l2_err(&dev->v4l2_dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	ret = rkcif_set_fmt(stream, &f->fmt.pix_mp, false);

	return ret;
}

static int rkcif_g_fmt_vid_cap_mplane(struct file *file, void *fh,
				      struct v4l2_format *f)
{
	struct rkcif_stream *stream = video_drvdata(file);

	f->fmt.pix_mp = stream->pixm;

	return 0;
}

static int rkcif_querycap(struct file *file, void *priv,
			  struct v4l2_capability *cap)
{
	struct rkcif_stream *stream = video_drvdata(file);
	struct device *dev = stream->cifdev->dev;

	strscpy(cap->driver, dev->driver->name, sizeof(cap->driver));
	strscpy(cap->card, dev->driver->name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		 "platform:%s", dev_name(dev));

	return 0;
}

static __maybe_unused int rkcif_cropcap(struct file *file, void *fh,
					struct v4l2_cropcap *cap)
{
	struct rkcif_stream *stream = video_drvdata(file);
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_rect *raw_rect = &dev->terminal_sensor.raw_rect;
	int ret = 0;

	if (stream->crop_mask & CROP_SRC_SENSOR) {
		cap->bounds.left = stream->crop[CROP_SRC_SENSOR].left;
		cap->bounds.top = stream->crop[CROP_SRC_SENSOR].top;
		cap->bounds.width = stream->crop[CROP_SRC_SENSOR].width;
		cap->bounds.height = stream->crop[CROP_SRC_SENSOR].height;
	} else {
		cap->bounds.left = raw_rect->left;
		cap->bounds.top = raw_rect->top;
		cap->bounds.width = raw_rect->width;
		cap->bounds.height = raw_rect->height;
	}

	cap->defrect = cap->bounds;
	cap->pixelaspect.numerator = 1;
	cap->pixelaspect.denominator = 1;

	return ret;
}

static int rkcif_s_selection(struct file *file, void *fh,
			     struct v4l2_selection *s)
{
	struct rkcif_stream *stream = video_drvdata(file);
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_subdev *sensor_sd;
	struct v4l2_subdev_selection sd_sel;
	const struct v4l2_rect *rect = &s->r;
	struct v4l2_rect sensor_crop;
	struct v4l2_rect *raw_rect = &dev->terminal_sensor.raw_rect;
	u16 pad = 0;
	int ret = 0;

	if (!s) {
		v4l2_dbg(1, rkcif_debug, &dev->v4l2_dev, "sel is null\n");
		goto err;
	}

	if (s->target == V4L2_SEL_TGT_CROP_BOUNDS) {
		sensor_sd = get_remote_sensor(stream, &pad);

		sd_sel.r = s->r;
		sd_sel.pad = pad;
		sd_sel.target = s->target;
		sd_sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;

		ret = v4l2_subdev_call(sensor_sd, pad, set_selection, NULL, &sd_sel);
		if (!ret) {
			s->r = sd_sel.r;
			v4l2_dbg(1, rkcif_debug, &dev->v4l2_dev, "%s: pad:%d, which:%d, target:%d\n",
				 __func__, pad, sd_sel.which, sd_sel.target);
		}
	} else if (s->target == V4L2_SEL_TGT_CROP) {
		ret = rkcif_sanity_check_fmt(stream, rect);
		if (ret) {
			v4l2_err(&dev->v4l2_dev, "set crop failed\n");
			return ret;
		}

		if (stream->crop_mask & CROP_SRC_SENSOR) {
			sensor_crop = stream->crop[CROP_SRC_SENSOR];
			if (rect->left + rect->width > sensor_crop.width ||
			    rect->top + rect->height > sensor_crop.height) {
				v4l2_err(&dev->v4l2_dev,
					 "crop size is bigger than sensor input:left:%d, top:%d, width:%d, height:%d\n",
					 sensor_crop.left, sensor_crop.top,
					 sensor_crop.width, sensor_crop.height);
				return -EINVAL;
			}
		} else {
			if (rect->left + rect->width > raw_rect->width ||
			    rect->top + rect->height > raw_rect->height) {
				v4l2_err(&dev->v4l2_dev,
					 "crop size is bigger than sensor raw input:left:%d, top:%d, width:%d, height:%d\n",
					 raw_rect->left, raw_rect->top,
					 raw_rect->width, raw_rect->height);
				return -EINVAL;
			}
		}

		stream->crop[CROP_SRC_USR] = *rect;
		stream->crop_enable = true;
		stream->crop_mask |= CROP_SRC_USR_MASK;
		stream->crop[CROP_SRC_ACT] = stream->crop[CROP_SRC_USR];
		if (stream->crop_mask & CROP_SRC_SENSOR) {
			sensor_crop = stream->crop[CROP_SRC_SENSOR];
			stream->crop[CROP_SRC_ACT].left = sensor_crop.left + stream->crop[CROP_SRC_USR].left;
			stream->crop[CROP_SRC_ACT].top = sensor_crop.top + stream->crop[CROP_SRC_USR].top;
		}

		if (stream->state == RKCIF_STATE_STREAMING) {
			stream->crop_dyn_en = true;

			v4l2_info(&dev->v4l2_dev, "enable dynamic crop, S_SELECTION(%ux%u@%u:%u) target: %d\n",
				  rect->width, rect->height, rect->left, rect->top, s->target);
		} else {
			v4l2_info(&dev->v4l2_dev, "static crop, S_SELECTION(%ux%u@%u:%u) target: %d\n",
				  rect->width, rect->height, rect->left, rect->top, s->target);
		}
	} else {
		goto err;
	}

	return ret;

err:
	return -EINVAL;
}

static int rkcif_g_selection(struct file *file, void *fh,
			     struct v4l2_selection *s)
{
	struct rkcif_stream *stream = video_drvdata(file);
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_subdev *sensor_sd;
	struct v4l2_subdev_selection sd_sel;
	u16 pad = 0;
	int ret = 0;

	if (!s) {
		v4l2_dbg(1, rkcif_debug, &dev->v4l2_dev, "sel is null\n");
		goto err;
	}

	if (s->target == V4L2_SEL_TGT_CROP_BOUNDS) {
		sensor_sd = get_remote_sensor(stream, &pad);

		sd_sel.pad = pad;
		sd_sel.target = s->target;
		sd_sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;

		v4l2_dbg(1, rkcif_debug, &dev->v4l2_dev, "%s(line:%d): sd:%s pad:%d, which:%d, target:%d\n",
			 __func__, __LINE__, sensor_sd->name, pad, sd_sel.which, sd_sel.target);

		ret = v4l2_subdev_call(sensor_sd, pad, get_selection, NULL, &sd_sel);
		if (!ret) {
			s->r = sd_sel.r;
		} else {
			s->r.left = 0;
			s->r.top = 0;
			s->r.width = stream->pixm.width;
			s->r.height = stream->pixm.height;
		}
	} else if (s->target == V4L2_SEL_TGT_CROP) {
		if (stream->crop_mask & (CROP_SRC_USR_MASK | CROP_SRC_SENSOR_MASK)) {
			s->r = stream->crop[CROP_SRC_ACT];
		} else {
			s->r.left = 0;
			s->r.top = 0;
			s->r.width = stream->pixm.width;
			s->r.height = stream->pixm.height;
		}
	} else {
		goto err;
	}

	return ret;
err:
	return -EINVAL;
}

static int rkcif_get_max_common_div(int a, int b)
{
	int remainder = a % b;

	while (remainder != 0) {
		a = b;
		b = remainder;
		remainder = a % b;
	}
	return b;
}

void rkcif_set_fps(struct rkcif_stream *stream, struct rkcif_fps *fps)
{
	struct rkcif_sensor_info *sensor = &stream->cifdev->terminal_sensor;
	struct rkcif_device *cif_dev = stream->cifdev;
	struct rkcif_stream *tmp_stream = NULL;
	u32 numerator, denominator;
	u32 def_fps = 0;
	u32 cur_fps = 0;
	int cap_m, skip_n;
	int i = 0;
	int max_common_div;
	bool skip_en = false;
	s32 vblank_def = 0;
	s32 vblank_curr = 0;
	int ret = 0;

	if (!stream->cifdev->terminal_sensor.sd) {
		ret = rkcif_update_sensor_info(stream);
		if (ret) {
			v4l2_err(&stream->cifdev->v4l2_dev,
				 "%s update sensor info fail\n",
				 __func__);
			return;
		}

	}
	if (!stream->cifdev->terminal_sensor.sd)
		return;
	if (atomic_read(&cif_dev->pipe.stream_cnt) == 0) {
		numerator = sensor->fi.interval.numerator;
		denominator = sensor->fi.interval.denominator;
	} else {
		numerator = sensor->src_fi.interval.numerator;
		denominator = sensor->src_fi.interval.denominator;
	}
	def_fps = denominator / numerator;

	vblank_def = rkcif_get_sensor_vblank_def(cif_dev);
	vblank_curr = rkcif_get_sensor_vblank(cif_dev);
	if (vblank_def)
		cur_fps = def_fps * (u32)(vblank_def + sensor->raw_rect.height) /
			  (u32)(vblank_curr + sensor->raw_rect.height);
	else
		cur_fps = def_fps;

	if (fps->fps == 0 || fps->fps > cur_fps) {
		v4l2_err(&stream->cifdev->v4l2_dev,
			"set fps %d fps failed, current fps %d fps\n",
			fps->fps, cur_fps);
		return;
	}
	if (cif_dev->chip_id >= CHIP_RV1126B_CIF) {
		cap_m = 1;
		skip_n = cur_fps / fps->fps - 1;
	} else {
		cap_m = fps->fps;
		skip_n = cur_fps - fps->fps;
		max_common_div = rkcif_get_max_common_div(cap_m, skip_n);
		cap_m /= max_common_div;
		skip_n /= max_common_div;
		if (cif_dev->chip_id > CHIP_RK3562_CIF) {
			if (cap_m > 3) {
				skip_n = skip_n / (cap_m / 3);
				if (skip_n == 0)
					skip_n = 1;
				cap_m = 3;
			}
		} else {
			if (cap_m > 64) {
				skip_n = skip_n / (cap_m / 64);
				if (skip_n == 0)
					skip_n = 1;
				cap_m = 64;
			}
		}
		if (skip_n > 7) {
			cap_m = cap_m / (skip_n / 7);
			if (cap_m == 0)
				cap_m = 1;
			skip_n = 7;
		}
	}
	if (fps->fps == cur_fps)
		skip_en = false;
	else
		skip_en = true;

	if (fps->ch_num > 1 && fps->ch_num < 4) {
		for (i = 0; i < fps->ch_num; i++) {
			tmp_stream = &cif_dev->stream[i];
			if (skip_en) {
				tmp_stream->skip_info.skip_to_en = true;
				tmp_stream->skip_info.cap_m = cap_m;
				tmp_stream->skip_info.skip_n = skip_n;
			} else {
				tmp_stream->skip_info.skip_to_dis = true;
			}
		}
	} else {
		if (skip_en) {
			stream->skip_info.skip_to_en = true;
			stream->skip_info.cap_m = cap_m;
			stream->skip_info.skip_n = skip_n;
		} else {
			stream->skip_info.skip_to_dis = true;
		}
	}
	v4l2_dbg(1, rkcif_debug, &stream->cifdev->v4l2_dev,
		    "skip_to_en %d, cap_m %d, skip_n %d\n",
			stream->skip_info.skip_to_en,
			cap_m,
			skip_n);
}

static void rkcif_alloc_buf_by_user_require(struct rkcif_device *dev,
					     struct rkcif_buffer_info *buf_info)
{
	struct rkcif_stream *detect_stream = &dev->stream[0];
	struct v4l2_pix_format_mplane *pixm = &detect_stream->pixm;
	struct rkcif_extend_info *extend_line = &detect_stream->extend_line;
	const struct v4l2_plane_pix_format *plane_fmt;
	int ret = 0;
	int i = 0;
	int height = 0;
	int h = 0;
	u32 size = 0;
	bool is_extended = false;

	is_extended = rkcif_is_extending_line_for_height(dev,
							 detect_stream,
							 detect_stream->cif_fmt_in);
	if (detect_stream->crop_enable)
		height = detect_stream->crop[CROP_SRC_ACT].height;
	else
		height = pixm->height;
	if (is_extended && extend_line->is_extended) {
		height = extend_line->pixm.height;
		pixm = &extend_line->pixm;
	}
	h = round_up(height, MEMORY_ALIGN_ROUND_UP_HEIGHT);
	plane_fmt = &pixm->plane_fmt[0];
	size = plane_fmt->sizeimage / height * h;

	for (i = 0; i < buf_info->buf_num; i++) {
		if (!dev->buf_user[i]) {
			dev->buf_user[i] = kzalloc(sizeof(*dev->buf_user[i]), GFP_KERNEL);
			if (!dev->buf_user[i]) {
				buf_info->buf_num = i;
				break;
			}
		}
		dev->buf_user[i]->is_need_dbuf = true;
		dev->buf_user[i]->is_need_dmafd = true;
		dev->buf_user[i]->is_need_vaddr = true;
		dev->buf_user[i]->size = size;
		ret = rkcif_alloc_buffer(dev, dev->buf_user[i]);
		if (ret) {
			buf_info->buf_num = i;
			kfree(dev->buf_user[i]);
			dev->buf_user[i] = NULL;
			break;
		}
		buf_info->dma_fd[i] = dev->buf_user[i]->dma_fd;
	}
	if (buf_info->buf_num > 0)
		dev->is_alloc_buf_user = true;
}

void rkcif_free_buf_by_user_require(struct rkcif_device *dev)
{
	int i = 0;

	while (i < VB2_MAX_FRAME) {
		if (dev->buf_user[i]) {
			rkcif_free_buffer(dev, dev->buf_user[i]);
			kfree(dev->buf_user[i]);
			dev->buf_user[i] = NULL;
		} else {
			break;
		}
		i++;
	}
	dev->is_alloc_buf_user = false;
}

int rkcif_quick_stream_on(struct rkcif_device *dev, bool is_intr)
{
	struct v4l2_subdev *sd;
	struct rkcif_stream *stream = NULL;
	int i = 0;
	int stream_num = 0;
	int ret = 0;
	int on = 1;

	if (dev->hdr.hdr_mode == HDR_X2)
		stream_num = 2;
	else if (dev->hdr.hdr_mode == HDR_X3)
		stream_num = 3;
	else
		stream_num = 1;

	for (i = 0; i < stream_num; i++)
		dev->stream[i].cur_skip_frame = dev->stream[i].skip_frame;
	dev->sditf[0]->mode.rdbk_mode = dev->sditf[0]->mode_src.rdbk_mode;
	if (dev->switch_info.is_use_switch)
		atomic_inc(&dev->hw_dev->switch_stream_cnt[dev->switch_info.host_idx]);
	if (dev->sditf[0]->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ) {
		for (i = 0; i < stream_num; i++) {
			stream = &dev->stream[i];
			stream->is_pause_stream = false;
			if (stream->cifdev->hdr.hdr_mode == NO_HDR ||
			    (stream->cifdev->hdr.hdr_mode == HDR_X2 && stream->id == 1) ||
			    (stream->cifdev->hdr.hdr_mode == HDR_X3 && stream->id == 2)) {
				rkcif_enable_capture(stream);
			} else {
				dev->stream[i].to_en_dma = RKCIF_DMAEN_BY_ISP;
				rkcif_enable_dma_capture(&dev->stream[i], true);
			}
		}
		if (dev->chip_id == CHIP_RV1106_CIF) {
			sditf_change_to_online(dev->sditf[0]);
			sd = get_rkisp_sd(dev->sditf[0]);
			if (sd)
				ret = v4l2_subdev_call(sd, core, ioctl,
						       RKISP_VICAP_CMD_MODE, &dev->sditf[0]->mode);
			if (ret) {
				v4l2_err(&dev->v4l2_dev, "set isp work mode online fail\n");
				return -EINVAL;
			}
		}
	} else {
		if (dev->chip_id == CHIP_RV1106_CIF) {
			sditf_disable_immediately(dev->sditf[0]);
			sd = get_rkisp_sd(dev->sditf[0]);
			if (sd)
				ret = v4l2_subdev_call(sd, core, ioctl,
						       RKISP_VICAP_CMD_MODE, &dev->sditf[0]->mode);
		}
		for (i = 0; i < stream_num; i++) {
			dev->stream[i].is_pause_stream = false;
			if (dev->sditf[0]->mode.rdbk_mode != RKISP_VICAP_RDBK_AIQ)
				dev->stream[i].to_en_dma = RKCIF_DMAEN_BY_ISP;
			else
				dev->stream[i].to_en_dma = RKCIF_DMAEN_BY_VICAP;
			rkcif_enable_dma_capture(&dev->stream[i], true);
		}
	}
	if (is_intr) {
		if (atomic_read(&dev->sensor_off)) {
			atomic_set(&dev->sensor_off, 0);
			rkcif_dphy_quick_stream(dev, on);
			dev->sensor_work.on = 1;
			schedule_work(&dev->sensor_work.work);
		}

	} else {
		mutex_lock(&dev->stream_lock);
		rkcif_dphy_quick_stream(dev, on);
		if (atomic_read(&dev->sensor_off)) {
			v4l2_subdev_call(dev->terminal_sensor.sd, core, ioctl,
					 RKMODULE_SET_QUICK_STREAM, &on);
			atomic_set(&dev->sensor_off, 0);
		}
		mutex_unlock(&dev->stream_lock);
	}
	dev->resume_mode = RKISP_RTT_MODE_MULTI_FRAME;
	return ret;
}

void rkcif_flip_end_wait_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct rkcif_device *dev = container_of(dwork,
						struct rkcif_device,
						work_flip);
	int stream_num = 0;
	int i = 0;
	struct rkcif_stream *cur_stream = NULL;
	unsigned long flags;

	mutex_lock(&dev->stream_lock);
	if (dev->hdr.hdr_mode == HDR_X2)
		stream_num = 2;
	else if (dev->hdr.hdr_mode == HDR_X3)
		stream_num = 3;
	else
		stream_num = 1;
	if (dev->sditf[0]->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ) {
		for (i = 0; i < stream_num; i++) {
			cur_stream = &dev->stream[i];
			if (dev->hdr.hdr_mode == NO_HDR ||
			    (dev->hdr.hdr_mode == HDR_X2 && cur_stream->id == 1) ||
			    (dev->hdr.hdr_mode == HDR_X3 && cur_stream->id == 2)) {
				rkcif_enable_capture(cur_stream);
			} else {
				cur_stream->to_en_dma = RKCIF_DMAEN_BY_ISP;
				rkcif_enable_dma_capture(cur_stream, true);
			}
			if (i == 0) {
				spin_lock_irqsave(&cur_stream->vbq_lock, flags);
				dev->is_in_flip = false;
				spin_unlock_irqrestore(&cur_stream->vbq_lock, flags);
			}
		}
	} else {
		for (i = 0; i < stream_num; i++) {
			cur_stream = &dev->stream[i];
			if (dev->sditf[0]->mode.rdbk_mode != RKISP_VICAP_RDBK_AIQ)
				cur_stream->to_en_dma = RKCIF_DMAEN_BY_ISP;
			else
				cur_stream->to_en_dma = RKCIF_DMAEN_BY_VICAP;
			rkcif_enable_dma_capture(cur_stream, true);
			if (i == 0) {
				spin_lock_irqsave(&cur_stream->vbq_lock, flags);
				dev->is_in_flip = false;
				spin_unlock_irqrestore(&cur_stream->vbq_lock, flags);
			}
		}
	}
	mutex_unlock(&dev->stream_lock);
}

static int rkcif_get_error_info(struct rkcif_device *cif_dev,
				struct rkmodule_error_info *err_info)
{
	int count = 0;
	int is_dvp = (cif_dev->inf_id == RKCIF_DVP);

	memset(err_info, 0, sizeof(*err_info));

	if (is_dvp)
		count = snprintf(err_info->detail, sizeof(err_info->detail), "rkcif_dvp:");
	else
		count = snprintf(err_info->detail, sizeof(err_info->detail), "rkcif_mipi%d:", cif_dev->csi_host_idx);

#define APPEND_STAT(field) \
	do {\
		ssize_t remaining = sizeof(err_info->detail) - count;\
		if (remaining > 0) { \
			int written = snprintf(err_info->detail + count, remaining, "%lld,", cif_dev->irq_stats.field); \
			if (written >= 0) { \
				count += written; \
			} \
		} \
	} while (0)

	APPEND_STAT(bus0_err);
	APPEND_STAT(bus1_err);

	if (is_dvp) {
		APPEND_STAT(dvp_overflow_cnt);
		APPEND_STAT(dvp_bwidth_lack_cnt);
		APPEND_STAT(dvp_size_err_cnt);
	} else {
		APPEND_STAT(csi_overflow_cnt);
		APPEND_STAT(csi_bwidth_lack_cnt);
		APPEND_STAT(csi_size_err_cnt);
	}

	count += snprintf(err_info->detail + count, sizeof(err_info->detail) - count,
			  "%lld,%lld,%lld,%lld,",
			  cif_dev->irq_stats.not_active_buf_cnt[0],
			  cif_dev->irq_stats.not_active_buf_cnt[1],
			  cif_dev->irq_stats.not_active_buf_cnt[2],
			  cif_dev->irq_stats.not_active_buf_cnt[3]);

	err_info->err_code = cif_dev->irq_stats.all_err_cnt ? BIT(0) : 0;

	return 0;
}

static int rkcif_do_reset_work(struct rkcif_device *cif_dev,
			       enum rkmodule_reset_src reset_src);

static long rkcif_ioctl_default(struct file *file, void *fh,
				bool valid_prio, unsigned int cmd, void *arg)
{
	struct rkcif_stream *stream = video_drvdata(file);
	struct rkcif_device *dev = stream->cifdev;
	struct sditf_priv *priv = dev->sditf[0];
	struct rkcif_stream *cur_stream = NULL;
	const struct cif_input_fmt *in_fmt;
	struct v4l2_rect rect;
	struct csi_channel_info csi_info;
	struct rkcif_fps fps;
	struct rkcif_buffer_info *buf_info = NULL;
	struct rkmodule_capture_info *capture_info;
	int reset_src;
	struct rkcif_quick_stream_param *stream_param;
	int ret = 0;
	int i = 0;
	int stream_num = 0;
	int on = 0;
	struct rkcif_stream *last_stream = NULL;
	unsigned long flags;
	int flip_skip = 0;
	u32 tline = 0;
	u32 delay_ms = 0;
	u32 vblank = 0;
	struct rkisp_vicap_mode vicap_mode;
	struct v4l2_subdev *sd = NULL;
	struct rkmodule_error_info *err_info;

	switch (cmd) {
	case RKCIF_CMD_GET_CSI_MEMORY_MODE:
		if (stream->is_compact) {
			*(int *)arg = CSI_LVDS_MEM_COMPACT;
		} else {
			if (stream->is_high_align)
				*(int *)arg = CSI_LVDS_MEM_WORD_HIGH_ALIGN;
			else
				*(int *)arg = CSI_LVDS_MEM_WORD_LOW_ALIGN;
		}
		break;
	case RKCIF_CMD_SET_CSI_MEMORY_MODE:
		if (dev->terminal_sensor.sd) {
			in_fmt = rkcif_get_input_fmt(dev,
						     &rect, 0, &csi_info);
			if (in_fmt == NULL) {
				v4l2_err(&dev->v4l2_dev, "can't get sensor input format\n");
				return -EINVAL;
			}
		} else {
			v4l2_err(&dev->v4l2_dev, "can't get sensor device\n");
			return -EINVAL;
		}
		if (*(int *)arg == CSI_LVDS_MEM_COMPACT) {
			if (((dev->inf_id == RKCIF_DVP && dev->chip_id <= CHIP_RK3568_CIF) ||
			    (dev->inf_id == RKCIF_MIPI_LVDS && dev->chip_id < CHIP_RV1126_CIF)) &&
			    in_fmt->csi_fmt_val != CSI_WRDDR_TYPE_RAW8) {
				v4l2_err(&dev->v4l2_dev, "device not support compact\n");
				return -EINVAL;
			}
			stream->is_compact = true;
			stream->is_high_align = false;
		} else if (*(int *)arg == CSI_LVDS_MEM_WORD_HIGH_ALIGN) {
			stream->is_compact = false;
			stream->is_high_align = true;
		} else {
			stream->is_compact = false;
			stream->is_high_align = false;
		}
		break;
	case RKCIF_CMD_SET_FPS:
		fps = *(struct rkcif_fps *)arg;
		rkcif_set_fps(stream, &fps);
		break;
	case RKCIF_CMD_SET_RESET:
		reset_src = *(int *)arg;
		if (dev->hw_dev->is_in_reset)
			return 0;
		return rkcif_do_reset_work(dev, reset_src);
	case RKCIF_CMD_SET_QUICK_STREAM:
		stream_param = (struct rkcif_quick_stream_param *)arg;
		if (!dev->sditf[0])
			return -EINVAL;
		if (dev->hdr.hdr_mode == HDR_X2) {
			stream_num = 2;
			last_stream = &dev->stream[1];
		} else if (dev->hdr.hdr_mode == HDR_X3) {
			stream_num = 3;
			last_stream = &dev->stream[2];
		} else {
			stream_num = 1;
			last_stream = &dev->stream[0];
		}
		if (stream_param->on) {
			spin_lock_irqsave(&dev->stream_spinlock, flags);
			if (last_stream->is_finish_single_cap) {
				spin_unlock_irqrestore(&dev->stream_spinlock, flags);
				ret = rkcif_quick_stream_on(dev, false);
				v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev,
					 "%s %d, finish single capture, restart now\n", __func__, __LINE__);
			} else {
				last_stream->is_wait_single_cap = true;
				spin_unlock_irqrestore(&dev->stream_spinlock, flags);
				v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev,
					 "%s %d, wait for single capture finish, and than to restart\n", __func__, __LINE__);
				reinit_completion(&last_stream->start_complete);
				wait_for_completion_timeout(&last_stream->start_complete,
							    msecs_to_jiffies(RKCIF_STOP_MAX_WAIT_TIME_MS));
			}
		} else {
			if (dev->sditf[0]->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ) {
				stream->cifdev->sensor_state = stream_param->on;
				stream->cifdev->sensor_state_change = true;
				for (i = 0; i < stream_num; i++) {
					cur_stream = &dev->stream[i];
					reinit_completion(&cur_stream->stop_complete);
					cur_stream->is_wait_stop_complete = true;
					wait_for_completion_timeout(&cur_stream->stop_complete,
								    msecs_to_jiffies(RKCIF_STOP_MAX_WAIT_TIME_MS));
				}
			} else {
				for (i = 0; i < stream_num; i++) {
					cur_stream = &dev->stream[i];
					cur_stream->is_wait_stop_complete = true;
					reinit_completion(&cur_stream->stop_complete);
					if (dev->sditf[0]->mode.rdbk_mode == RKISP_VICAP_RDBK_AUTO)
						cur_stream->to_stop_dma = RKCIF_DMAEN_BY_ISP;
					else
						cur_stream->to_stop_dma = RKCIF_DMAEN_BY_VICAP;
					wait_for_completion_timeout(&cur_stream->stop_complete,
								    msecs_to_jiffies(RKCIF_STOP_MAX_WAIT_TIME_MS));
				}
				mutex_lock(&stream->cifdev->stream_lock);
				rkcif_dphy_quick_stream(dev, stream_param->on);
				v4l2_subdev_call(dev->terminal_sensor.sd, core, ioctl,
						 RKMODULE_SET_QUICK_STREAM, &stream_param->on);
				atomic_inc(&stream->cifdev->sensor_off);
				mutex_unlock(&stream->cifdev->stream_lock);
			}
			stream_param->frame_num = dev->stream[0].frame_idx - 1;
			if (!dev->is_rtt_suspend) {
				dev->resume_mode = stream_param->resume_mode;
				v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev,
					 "set resume mode %d\n", dev->resume_mode);
			}
		}
		break;
	case RKCIF_CMD_START_CAPTURE_ONE_FRAME_AOV:
		if (!dev->sditf[0])
			return -EINVAL;
		if (dev->hdr.hdr_mode == HDR_X2)
			stream_num = 2;
		else if (dev->hdr.hdr_mode == HDR_X3)
			stream_num = 3;
		else
			stream_num = 1;
		for (i = 0; i < stream_num; i++) {
			dev->stream[i].cur_skip_frame = dev->stream[i].skip_frame;
			dev->stream[i].is_single_cap = true;
			stream->is_finish_single_cap = false;
			stream->is_wait_single_cap = false;
		}
		vicap_mode = priv->mode_src;
		if (vicap_mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ) {
			if (dev->chip_id == CHIP_RV1106_CIF)
				vicap_mode.rdbk_mode = RKISP_VICAP_RDBK_AUTO_ONE_FRAME;
			else
				vicap_mode.rdbk_mode = RKISP_VICAP_ONLINE_ONE_FRAME;
		} else if (vicap_mode.rdbk_mode == RKISP_VICAP_RDBK_AIQ) {
			vicap_mode.rdbk_mode = RKISP_VICAP_RDBK_AIQ;
		} else {
			vicap_mode.rdbk_mode = RKISP_VICAP_RDBK_AUTO_ONE_FRAME;
		}

		sd = get_rkisp_sd(priv);
		if (sd) {
			ret = v4l2_subdev_call(sd, core, ioctl,
					       RKISP_VICAP_CMD_MODE, &vicap_mode);
			if (ret)
				v4l2_err(&dev->v4l2_dev,
					 "set isp work mode %d failed\n", vicap_mode.rdbk_mode);
			else
				v4l2_dbg(1, rkcif_debug, &dev->v4l2_dev,
					 "set isp work mode %d", vicap_mode.rdbk_mode);
		}

		if (dev->sditf[0]->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ) {
			for (i = 0; i < stream_num; i++) {
				cur_stream = &dev->stream[i];
				if (dev->hdr.hdr_mode == NO_HDR ||
				    (dev->hdr.hdr_mode == HDR_X2 && cur_stream->id == 1) ||
				    (dev->hdr.hdr_mode == HDR_X3 && cur_stream->id == 2)) {
					rkcif_enable_capture(cur_stream);
				} else {
					cur_stream->to_en_dma = RKCIF_DMAEN_BY_ISP;
					rkcif_enable_dma_capture(cur_stream, true);
				}
			}
		} else {
			for (i = 0; i < stream_num; i++) {
				cur_stream = &dev->stream[i];
				if (dev->sditf[0]->mode.rdbk_mode != RKISP_VICAP_RDBK_AIQ)
					cur_stream->to_en_dma = RKCIF_DMAEN_BY_ISP;
				else
					cur_stream->to_en_dma = RKCIF_DMAEN_BY_VICAP;
				rkcif_enable_dma_capture(cur_stream, true);
			}
			if (dev->switch_info.is_use_switch)
				atomic_inc(&dev->hw_dev->switch_stream_cnt[dev->switch_info.host_idx]);
		}
		mutex_lock(&stream->cifdev->stream_lock);
		on = 1;
		rkcif_dphy_quick_stream(dev, on);
		v4l2_subdev_call(dev->terminal_sensor.sd, core, ioctl,
				 RKMODULE_SET_QUICK_STREAM, &on);
		atomic_set(&stream->cifdev->sensor_off, 0);
		mutex_unlock(&stream->cifdev->stream_lock);
		break;
	case RKCIF_CMD_ALLOC_BUF:
		buf_info = (struct rkcif_buffer_info *)arg;
		if (buf_info == NULL)
			return -ENOMEM;
		if (buf_info->buf_num > VIDEO_MAX_FRAME) {
			v4l2_err(&dev->v4l2_dev, "alloc buf fail, max buf num %d\n",
				 VIDEO_MAX_FRAME);
			return -EINVAL;
		}
		mutex_lock(&dev->stream_lock);
		if (dev->is_alloc_buf_user) {
			v4l2_err(&dev->v4l2_dev, "it's already alloc buf, release buf before alloc new buf\n");
			mutex_unlock(&dev->stream_lock);
			return -EINVAL;
		}
		rkcif_alloc_buf_by_user_require(dev, buf_info);
		mutex_unlock(&dev->stream_lock);
		break;
	case RKCIF_CMD_FREE_BUF:
		mutex_lock(&dev->stream_lock);
		if (dev->is_alloc_buf_user)
			rkcif_free_buf_by_user_require(dev);
		mutex_unlock(&dev->stream_lock);
		break;
	case RKMODULE_SET_CAPTURE_MODE:
		capture_info = (struct rkmodule_capture_info *)arg;
		if (capture_info->mode == RKMODULE_ONE_CH_TO_MULTI_ISP) {
			for (i = 0; i < capture_info->one_to_multi.isp_num; i++) {
				if (capture_info->one_to_multi.frame_pattern[i] == 0 ||
				    capture_info->one_to_multi.frame_pattern[i] > 32)
					return -EINVAL;
			}
		}
		dev->channels[0].capture_info = *capture_info;
		v4l2_info(&dev->v4l2_dev,
			  "set capture mode %d\n", dev->channels[0].capture_info.mode);
		break;
	case RKCIF_CMD_SET_SENSOR_FLIP_START:
		mutex_lock(&dev->stream_lock);
		if (atomic_read(&dev->sensor_off)) {
			mutex_unlock(&dev->stream_lock);
			return 0;
		}
		if (dev->is_in_flip) {
			v4l2_err(&dev->v4l2_dev, "last flip ops not finish\n");
			mutex_unlock(&dev->stream_lock);
			return -EBUSY;
		}
		if (dev->hdr.hdr_mode == HDR_X2)
			stream_num = 2;
		else if (dev->hdr.hdr_mode == HDR_X3)
			stream_num = 3;
		else
			stream_num = 1;
		if (dev->sditf[0]->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ) {
			for (i = 0; i < stream_num; i++) {
				cur_stream = &dev->stream[i];
				reinit_completion(&cur_stream->stop_complete);
				cur_stream->is_wait_stop_complete = true;
				wait_for_completion_timeout(&cur_stream->stop_complete,
							    msecs_to_jiffies(RKCIF_STOP_MAX_WAIT_TIME_MS));
				if (i == 0) {
					spin_lock_irqsave(&cur_stream->vbq_lock, flags);
					dev->is_in_flip = true;
					spin_unlock_irqrestore(&cur_stream->vbq_lock, flags);
				}
			}
		} else {
			for (i = 0; i < stream_num; i++) {
				cur_stream = &dev->stream[i];
				cur_stream->is_wait_stop_complete = true;
				reinit_completion(&cur_stream->stop_complete);
				if (dev->sditf[0]->mode.rdbk_mode != RKISP_VICAP_RDBK_AIQ)
					cur_stream->to_stop_dma = RKCIF_DMAEN_BY_ISP;
				else
					cur_stream->to_stop_dma = RKCIF_DMAEN_BY_VICAP;
				wait_for_completion_timeout(&cur_stream->stop_complete,
							    msecs_to_jiffies(RKCIF_STOP_MAX_WAIT_TIME_MS));
				if (i == 0) {
					spin_lock_irqsave(&cur_stream->vbq_lock, flags);
					dev->is_in_flip = true;
					spin_unlock_irqrestore(&cur_stream->vbq_lock, flags);
				}
			}
		}
		mutex_unlock(&dev->stream_lock);
		break;
	case RKCIF_CMD_SET_SENSOR_FLIP_END:
		mutex_lock(&dev->stream_lock);
		if (atomic_read(&dev->sensor_off)) {
			mutex_unlock(&dev->stream_lock);
			return 0;
		}
		flip_skip = *(int *)arg;
		if (flip_skip) {
			tline = rkcif_get_linetime(stream);
			tline = div_u64(tline, 1000);
			vblank = rkcif_get_sensor_vblank(dev);
			delay_ms = tline * ((flip_skip - 1) * (vblank + stream->pixm.height) + vblank);
			delay_ms = div_u64(delay_ms, 1000);
		} else {
			delay_ms = 1;
		}
		schedule_delayed_work(&dev->work_flip, msecs_to_jiffies(delay_ms));
		v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev, "flip delay_ms %d\n", delay_ms);
		mutex_unlock(&dev->stream_lock);
		break;
	case RKCIF_CMD_SUPPORT_GET_EXP:
		if (*(int *)arg)
			dev->is_support_get_exp = true;
		else
			dev->is_support_get_exp = false;
		break;
	case RKMODULE_GET_ERROR_INFO:
		err_info = (struct rkmodule_error_info *)arg;
		ret = rkcif_get_error_info(dev, err_info);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int rkcif_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	int ret;
	struct rkcif_stream *stream = video_drvdata(file);

	ret = vb2_ioctl_dqbuf(file, priv, p);
	if (stream->low_latency)
		rkcif_dqbuf_get_done_fence(stream);

	return ret;
}

static const struct v4l2_ioctl_ops rkcif_v4l2_ioctl_ops = {
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_dqbuf = rkcif_dqbuf,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_enum_input = rkcif_enum_input,
	.vidioc_try_fmt_vid_cap_mplane = rkcif_try_fmt_vid_cap_mplane,
	.vidioc_enum_fmt_vid_cap = rkcif_enum_fmt_vid_cap_mplane,
	.vidioc_s_fmt_vid_cap_mplane = rkcif_s_fmt_vid_cap_mplane,
	.vidioc_g_fmt_vid_cap_mplane = rkcif_g_fmt_vid_cap_mplane,
	.vidioc_querycap = rkcif_querycap,
	.vidioc_s_selection = rkcif_s_selection,
	.vidioc_g_selection = rkcif_g_selection,
	.vidioc_enum_frameintervals = rkcif_enum_frameintervals,
	.vidioc_enum_framesizes = rkcif_enum_framesizes,
	.vidioc_default = rkcif_ioctl_default,
};

void rkcif_vb_done_oneframe(struct rkcif_stream *stream,
			    struct vb2_v4l2_buffer *vb_done)
{
	const struct cif_output_fmt *fmt = stream->cif_fmt_out;
	u32 i;

	/* Dequeue a filled buffer */
	for (i = 0; i < fmt->mplanes; i++) {
		vb2_set_plane_payload(&vb_done->vb2_buf, i,
				      stream->pixm.plane_fmt[i].sizeimage);
	}

	vb2_buffer_done(&vb_done->vb2_buf, VB2_BUF_STATE_DONE);
	v4l2_dbg(2, rkcif_debug, &stream->cifdev->v4l2_dev,
		 "stream[%d] vb done, index: %d, sequence %d\n", stream->id,
		 vb_done->vb2_buf.index, vb_done->sequence);
	atomic_dec(&stream->buf_cnt);
}

static void rkcif_vb_done_one_to_multi(struct rkcif_device *dev,
					      struct rkcif_buffer *active_buf)
{
	struct rkcif_stream *stream = &dev->stream[active_buf->id];
	struct sditf_priv *priv = dev->sditf[stream->id];

	active_buf->vb.sequence = priv->frame_idx.cur_frame_idx - 1;
	rkcif_vb_done_oneframe(stream, &active_buf->vb);
}

static void rkcif_tasklet_handle(unsigned long data)
{
	struct rkcif_stream *stream = (struct rkcif_stream *)data;
	struct rkcif_buffer *buf = NULL;
	unsigned long flags = 0;
	LIST_HEAD(local_list);

	spin_lock_irqsave(&stream->vbq_lock, flags);
	list_replace_init(&stream->vb_done_list, &local_list);
	spin_unlock_irqrestore(&stream->vbq_lock, flags);

	while (!list_empty(&local_list)) {
		buf = list_first_entry(&local_list,
				       struct rkcif_buffer, queue);
		list_del(&buf->queue);
		atomic_dec(&stream->cifdev->stream[buf->id].sub_stream_buf_cnt);
		if (stream->cifdev->channels[0].capture_info.mode == RKMODULE_ONE_CH_TO_MULTI_ISP)
			rkcif_vb_done_one_to_multi(stream->cifdev, buf);
		else
			rkcif_vb_done_oneframe(stream, &buf->vb);
	}
}

void rkcif_vb_done_tasklet(struct rkcif_stream *stream, struct rkcif_buffer *buf)
{
	unsigned long flags = 0;

	if (!stream || !buf)
		return;
	spin_lock_irqsave(&stream->vbq_lock, flags);
	list_add_tail(&buf->queue, &stream->vb_done_list);
	spin_unlock_irqrestore(&stream->vbq_lock, flags);
	tasklet_schedule(&stream->vb_done_tasklet);
}

static void rkcif_unregister_stream_vdev(struct rkcif_stream *stream)
{
	tasklet_kill(&stream->vb_done_tasklet);
	media_entity_cleanup(&stream->vnode.vdev.entity);
	video_unregister_device(&stream->vnode.vdev);
}

static int rkcif_register_stream_vdev(struct rkcif_stream *stream,
				      bool is_multi_input)
{
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	struct video_device *vdev = &stream->vnode.vdev;
	struct rkcif_vdev_node *node;
	int ret = 0;
	char *vdev_name;

	if (dev->chip_id < CHIP_RV1126_CIF) {
		if (is_multi_input) {
			switch (stream->id) {
			case RKCIF_STREAM_MIPI_ID0:
				vdev_name = CIF_MIPI_ID0_VDEV_NAME;
				break;
			case RKCIF_STREAM_MIPI_ID1:
				vdev_name = CIF_MIPI_ID1_VDEV_NAME;
				break;
			case RKCIF_STREAM_MIPI_ID2:
				vdev_name = CIF_MIPI_ID2_VDEV_NAME;
				break;
			case RKCIF_STREAM_MIPI_ID3:
				vdev_name = CIF_MIPI_ID3_VDEV_NAME;
				break;
			case RKCIF_STREAM_DVP:
				vdev_name = CIF_DVP_VDEV_NAME;
				break;
			default:
				ret = -EINVAL;
				v4l2_err(v4l2_dev, "Invalid stream\n");
				goto unreg;
			}
		} else {
			vdev_name = CIF_VIDEODEVICE_NAME;
		}
	} else {
		if (dev->inf_id == RKCIF_MIPI_LVDS) {
			switch (stream->id) {
			case RKCIF_STREAM_MIPI_ID0:
				vdev_name = CIF_MIPI_ID0_VDEV_NAME;
				break;
			case RKCIF_STREAM_MIPI_ID1:
				vdev_name = CIF_MIPI_ID1_VDEV_NAME;
				break;
			case RKCIF_STREAM_MIPI_ID2:
				vdev_name = CIF_MIPI_ID2_VDEV_NAME;
				break;
			case RKCIF_STREAM_MIPI_ID3:
				vdev_name = CIF_MIPI_ID3_VDEV_NAME;
				break;
			default:
				ret = -EINVAL;
				v4l2_err(v4l2_dev, "Invalid stream\n");
				goto unreg;
			}
		} else {
			switch (stream->id) {
			case RKCIF_STREAM_MIPI_ID0:
				vdev_name = CIF_DVP_ID0_VDEV_NAME;
				break;
			case RKCIF_STREAM_MIPI_ID1:
				vdev_name = CIF_DVP_ID1_VDEV_NAME;
				break;
			case RKCIF_STREAM_MIPI_ID2:
				vdev_name = CIF_DVP_ID2_VDEV_NAME;
				break;
			case RKCIF_STREAM_MIPI_ID3:
				vdev_name = CIF_DVP_ID3_VDEV_NAME;
				break;
			default:
				ret = -EINVAL;
				v4l2_err(v4l2_dev, "Invalid stream\n");
				goto unreg;
			}
		}
	}

	strscpy(vdev->name, vdev_name, sizeof(vdev->name));
	node = vdev_to_node(vdev);
	mutex_init(&node->vlock);

	vdev->ioctl_ops = &rkcif_v4l2_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->fops = &rkcif_fops;
	vdev->minor = -1;
	vdev->v4l2_dev = v4l2_dev;
	vdev->lock = &node->vlock;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE |
			    V4L2_CAP_STREAMING;
	video_set_drvdata(vdev, stream);
	vdev->vfl_dir = VFL_DIR_RX;
	node->pad.flags = MEDIA_PAD_FL_SINK;

	rkcif_init_vb2_queue(&node->buf_queue, stream,
			     V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	vdev->queue = &node->buf_queue;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret < 0) {
		v4l2_err(v4l2_dev,
			 "video_register_device failed with error %d\n", ret);
		return ret;
	}

	ret = media_entity_pads_init(&vdev->entity, 1, &node->pad);
	if (ret < 0)
		goto unreg;

	INIT_LIST_HEAD(&stream->vb_done_list);
	tasklet_init(&stream->vb_done_tasklet,
		     rkcif_tasklet_handle,
		     (unsigned long)stream);
	tasklet_disable(&stream->vb_done_tasklet);
	return 0;
unreg:
	video_unregister_device(vdev);
	return ret;
}

void rkcif_unregister_stream_vdevs(struct rkcif_device *dev,
				   int stream_num)
{
	struct rkcif_stream *stream;
	int i;

	for (i = 0; i < stream_num; i++) {
		stream = &dev->stream[i];
		rkcif_unregister_stream_vdev(stream);
	}
}

int rkcif_register_stream_vdevs(struct rkcif_device *dev,
				int stream_num,
				bool is_multi_input)
{
	struct rkcif_stream *stream;
	int i, j, ret;

	for (i = 0; i < stream_num; i++) {
		stream = &dev->stream[i];
		stream->cifdev = dev;
		ret = rkcif_register_stream_vdev(stream, is_multi_input);
		if (ret < 0)
			goto err;
	}
	dev->num_channels = stream_num;
	return 0;
err:
	for (j = 0; j < i; j++) {
		stream = &dev->stream[j];
		rkcif_unregister_stream_vdev(stream);
	}

	return ret;
}

static struct v4l2_subdev *get_lvds_remote_sensor(struct v4l2_subdev *sd)
{
	struct media_pad *local, *remote;
	struct media_entity *sensor_me;

	local = &sd->entity.pads[RKCIF_LVDS_PAD_SINK];
	remote = media_pad_remote_pad_first(local);
	if (!remote) {
		v4l2_warn(sd, "No link between dphy and sensor with lvds\n");
		return NULL;
	}

	sensor_me = media_pad_remote_pad_first(local)->entity;
	return media_entity_to_v4l2_subdev(sensor_me);
}

static int rkcif_lvds_subdev_link_setup(struct media_entity *entity,
					const struct media_pad *local,
					const struct media_pad *remote,
					u32 flags)
{
	return 0;
}

static int rkcif_lvds_sd_set_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_subdev *sensor = get_lvds_remote_sensor(sd);

	/*
	 * Do not allow format changes and just relay whatever
	 * set currently in the sensor.
	 */
	return v4l2_subdev_call(sensor, pad, get_fmt, NULL, fmt);
}

static int rkcif_lvds_sd_get_fmt(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct rkcif_lvds_subdev *subdev = container_of(sd, struct rkcif_lvds_subdev, sd);
	struct v4l2_subdev *sensor = get_lvds_remote_sensor(sd);
	int ret;

	/*
	 * Do not allow format changes and just relay whatever
	 * set currently in the sensor.
	 */
	ret = v4l2_subdev_call(sensor, pad, get_fmt, NULL, fmt);
	if (!ret)
		subdev->in_fmt = fmt->format;

	return ret;
}

static struct v4l2_rect *rkcif_lvds_sd_get_crop(struct rkcif_lvds_subdev *subdev,
						struct v4l2_subdev_state *sd_state,
						enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_crop(&subdev->sd, sd_state, RKCIF_LVDS_PAD_SINK);
	else
		return &subdev->crop;
}

static int rkcif_lvds_sd_set_selection(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *sd_state,
				       struct v4l2_subdev_selection *sel)
{
	struct rkcif_lvds_subdev *subdev = container_of(sd, struct rkcif_lvds_subdev, sd);
	struct v4l2_subdev *sensor = get_lvds_remote_sensor(sd);
	int ret = 0;

	ret = v4l2_subdev_call(sensor, pad, set_selection,
			       sd_state, sel);
	if (!ret)
		subdev->crop = sel->r;

	return ret;
}

static int rkcif_lvds_sd_get_selection(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *sd_state,
				       struct v4l2_subdev_selection *sel)
{
	struct rkcif_lvds_subdev *subdev = container_of(sd, struct rkcif_lvds_subdev, sd);
	struct v4l2_subdev *sensor = get_lvds_remote_sensor(sd);
	struct v4l2_subdev_format fmt;
	int ret = 0;

	if (!sel) {
		v4l2_dbg(1, rkcif_debug, sd, "sel is null\n");
		goto err;
	}

	if (sel->pad > RKCIF_LVDS_PAD_SRC_ID3) {
		v4l2_dbg(1, rkcif_debug, sd, "pad[%d] isn't matched\n", sel->pad);
		goto err;
	}

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		if (sel->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
			ret = v4l2_subdev_call(sensor, pad, get_selection,
					       sd_state, sel);
			if (ret) {
				fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
				ret = v4l2_subdev_call(sensor, pad, get_fmt, NULL, &fmt);
				if (!ret) {
					subdev->in_fmt = fmt.format;
					sel->r.top = 0;
					sel->r.left = 0;
					sel->r.width = subdev->in_fmt.width;
					sel->r.height = subdev->in_fmt.height;
					subdev->crop = sel->r;
				} else {
					sel->r = subdev->crop;
				}
			} else {
				subdev->crop = sel->r;
			}
		} else {
			sel->r = *v4l2_subdev_get_try_crop(sd, sd_state, sel->pad);
		}
		break;

	case V4L2_SEL_TGT_CROP:
		sel->r = *rkcif_lvds_sd_get_crop(subdev, sd_state, sel->which);
		break;

	default:
		return -EINVAL;
	}

	return 0;
err:
	return -EINVAL;
}

static int rkcif_lvds_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				    struct v4l2_mbus_config *mbus)
{
	struct v4l2_subdev *sensor_sd = get_lvds_remote_sensor(sd);
	int ret;

	ret = v4l2_subdev_call(sensor_sd, pad, get_mbus_config, 0, mbus);
	if (ret)
		return ret;

	return 0;
}

static int rkcif_lvds_sd_s_stream(struct v4l2_subdev *sd, int on)
{
	struct rkcif_lvds_subdev *subdev = container_of(sd, struct rkcif_lvds_subdev, sd);

	if (on)
		atomic_set(&subdev->frm_sync_seq, 0);

	return 0;
}

static int rkcif_lvds_sd_s_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static int rkcif_sof_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				     struct v4l2_event_subscription *sub)
{
	if (sub->type == V4L2_EVENT_FRAME_SYNC ||
	    sub->type == V4L2_EVENT_RESET_DEV)
		return v4l2_event_subscribe(fh, sub, RKCIF_V4L2_EVENT_ELEMS, NULL);
	else
		return -EINVAL;
}

static const struct media_entity_operations rkcif_lvds_sd_media_ops = {
	.link_setup = rkcif_lvds_subdev_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_pad_ops rkcif_lvds_sd_pad_ops = {
	.set_fmt = rkcif_lvds_sd_set_fmt,
	.get_fmt = rkcif_lvds_sd_get_fmt,
	.set_selection = rkcif_lvds_sd_set_selection,
	.get_selection = rkcif_lvds_sd_get_selection,
	.get_mbus_config = rkcif_lvds_g_mbus_config,
};

static const struct v4l2_subdev_video_ops rkcif_lvds_sd_video_ops = {
	.s_stream = rkcif_lvds_sd_s_stream,
};

static const struct v4l2_subdev_core_ops rkcif_lvds_sd_core_ops = {
	.s_power = rkcif_lvds_sd_s_power,
	.subscribe_event = rkcif_sof_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static struct v4l2_subdev_ops rkcif_lvds_sd_ops = {
	.core = &rkcif_lvds_sd_core_ops,
	.video = &rkcif_lvds_sd_video_ops,
	.pad = &rkcif_lvds_sd_pad_ops,
};

static void rkcif_lvds_event_inc_sof(struct rkcif_device *dev)
{
	struct rkcif_lvds_subdev *subdev = &dev->lvds_subdev;

	if (subdev) {
		struct v4l2_event event = {
			.type = V4L2_EVENT_FRAME_SYNC,
			.u.frame_sync.frame_sequence =
				atomic_inc_return(&subdev->frm_sync_seq) - 1,
		};
		v4l2_event_queue(subdev->sd.devnode, &event);
	}
}

static u32 rkcif_lvds_get_sof(struct rkcif_device *dev)
{
	if (dev)
		return atomic_read(&dev->lvds_subdev.frm_sync_seq) - 1;

	return 0;
}

static u32 rkcif_lvds_set_sof(struct rkcif_device *dev, u32 seq)
{
	if (dev)
		atomic_set(&dev->lvds_subdev.frm_sync_seq, seq);

	return 0;
}

int rkcif_register_lvds_subdev(struct rkcif_device *dev)
{
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	struct rkcif_lvds_subdev *lvds_subdev = &dev->lvds_subdev;
	struct v4l2_subdev *sd;
	int ret;
	int pad_num = 4;

	memset(lvds_subdev, 0, sizeof(*lvds_subdev));
	lvds_subdev->cifdev = dev;
	sd = &lvds_subdev->sd;
	lvds_subdev->state = RKCIF_LVDS_STOP;
	v4l2_subdev_init(sd, &rkcif_lvds_sd_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->entity.ops = &rkcif_lvds_sd_media_ops;
	if (dev->chip_id != CHIP_RV1126_CIF_LITE)
		snprintf(sd->name, sizeof(sd->name), "rkcif-lvds-subdev");
	else
		snprintf(sd->name, sizeof(sd->name), "rkcif-lite-lvds-subdev");

	lvds_subdev->pads[RKCIF_LVDS_PAD_SINK].flags =
		MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	lvds_subdev->pads[RKCIF_LVDS_PAD_SRC_ID0].flags = MEDIA_PAD_FL_SOURCE;
	lvds_subdev->pads[RKCIF_LVDS_PAD_SRC_ID1].flags = MEDIA_PAD_FL_SOURCE;
	lvds_subdev->pads[RKCIF_LVDS_PAD_SRC_ID2].flags = MEDIA_PAD_FL_SOURCE;
	lvds_subdev->pads[RKCIF_LVDS_PAD_SRC_ID3].flags = MEDIA_PAD_FL_SOURCE;
	if (dev->chip_id == CHIP_RV1106_CIF) {
		lvds_subdev->pads[RKCIF_LVDS_PAD_SCL_ID0].flags = MEDIA_PAD_FL_SOURCE;
		lvds_subdev->pads[RKCIF_LVDS_PAD_SCL_ID1].flags = MEDIA_PAD_FL_SOURCE;
		lvds_subdev->pads[RKCIF_LVDS_PAD_SCL_ID2].flags = MEDIA_PAD_FL_SOURCE;
		lvds_subdev->pads[RKCIF_LVDS_PAD_SCL_ID3].flags = MEDIA_PAD_FL_SOURCE;
		pad_num = RKCIF_LVDS_PAD_MAX;
	}

	lvds_subdev->in_fmt.width = RKCIF_DEFAULT_WIDTH;
	lvds_subdev->in_fmt.height = RKCIF_DEFAULT_HEIGHT;
	lvds_subdev->crop.left = 0;
	lvds_subdev->crop.top = 0;
	lvds_subdev->crop.width = RKCIF_DEFAULT_WIDTH;
	lvds_subdev->crop.height = RKCIF_DEFAULT_HEIGHT;

	ret = media_entity_pads_init(&sd->entity, pad_num,
				     lvds_subdev->pads);
	if (ret < 0)
		return ret;
	sd->owner = THIS_MODULE;
	v4l2_set_subdevdata(sd, lvds_subdev);
	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0)
		goto free_media;

	ret = v4l2_device_register_subdev_nodes(v4l2_dev);
	if (ret < 0)
		goto free_subdev;
	return ret;
free_subdev:
	v4l2_device_unregister_subdev(sd);
free_media:
	media_entity_cleanup(&sd->entity);
	v4l2_err(sd, "Failed to register subdev, ret:%d\n", ret);
	return ret;
}

void rkcif_unregister_lvds_subdev(struct rkcif_device *dev)
{
	struct v4l2_subdev *sd = &dev->lvds_subdev.sd;

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
}

static void rkcif_dvp_event_inc_sof(struct rkcif_device *dev)
{
	struct rkcif_dvp_sof_subdev *subdev = &dev->dvp_sof_subdev;

	if (subdev) {
		struct v4l2_event event = {
			.type = V4L2_EVENT_FRAME_SYNC,
			.u.frame_sync.frame_sequence =
				atomic_inc_return(&subdev->frm_sync_seq) - 1,
		};
		v4l2_event_queue(subdev->sd.devnode, &event);
	}
}

static u32 rkcif_dvp_get_sof(struct rkcif_device *dev)
{
	if (dev)
		return atomic_read(&dev->dvp_sof_subdev.frm_sync_seq) - 1;

	return 0;
}

static u32 rkcif_dvp_set_sof(struct rkcif_device *dev, u32 seq)
{
	if (dev)
		atomic_set(&dev->dvp_sof_subdev.frm_sync_seq, seq);

	return 0;
}

static const struct v4l2_subdev_core_ops rkcif_dvp_sof_sd_core_ops = {
	.subscribe_event = rkcif_sof_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static struct v4l2_subdev_ops rkcif_dvp_sof_sd_ops = {
	.core = &rkcif_dvp_sof_sd_core_ops,
};

int rkcif_register_dvp_sof_subdev(struct rkcif_device *dev)
{
	struct v4l2_device *v4l2_dev = &dev->v4l2_dev;
	struct rkcif_dvp_sof_subdev *subdev = &dev->dvp_sof_subdev;
	struct v4l2_subdev *sd;
	int ret;

	memset(subdev, 0, sizeof(*subdev));
	subdev->cifdev = dev;
	sd = &subdev->sd;
	v4l2_subdev_init(sd, &rkcif_dvp_sof_sd_ops);
	sd->owner = THIS_MODULE;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	snprintf(sd->name, sizeof(sd->name), "rkcif-dvp-sof");

	v4l2_set_subdevdata(sd, subdev);
	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0)
		goto end;

	ret = v4l2_device_register_subdev_nodes(v4l2_dev);
	if (ret < 0)
		goto free_subdev;

	return ret;

free_subdev:
	v4l2_device_unregister_subdev(sd);

end:
	v4l2_err(sd, "Failed to register subdev, ret:%d\n", ret);
	return ret;
}

void rkcif_unregister_dvp_sof_subdev(struct rkcif_device *dev)
{
	struct v4l2_subdev *sd = &dev->dvp_sof_subdev.sd;

	v4l2_device_unregister_subdev(sd);
}

void rkcif_irq_oneframe(struct rkcif_device *cif_dev)
{
	/* TODO: xuhf-debug: add stream type */
	struct rkcif_stream *stream;
	u32 lastline, lastpix, ctl, cif_frmst, intstat, frmid;
	int ret = 0;

	intstat = rkcif_read_register(cif_dev, CIF_REG_DVP_INTSTAT);
	cif_frmst = rkcif_read_register(cif_dev, CIF_REG_DVP_FRAME_STATUS);
	lastline = rkcif_read_register(cif_dev, CIF_REG_DVP_LAST_LINE);
	lastpix = rkcif_read_register(cif_dev, CIF_REG_DVP_LAST_PIX);
	ctl = rkcif_read_register(cif_dev, CIF_REG_DVP_CTRL);
	frmid = CIF_GET_FRAME_ID(cif_frmst);

	/* There are two irqs enabled:
	 *  - PST_INF_FRAME_END: cif FIFO is ready, this is prior to FRAME_END
	 *  -	      FRAME_END: cif has saved frame to memory, a frame ready
	 */
	stream = &cif_dev->stream[RKCIF_STREAM_CIF];

	if ((intstat & PST_INF_FRAME_END)) {
		rkcif_write_register(cif_dev, CIF_REG_DVP_INTSTAT,
				     PST_INF_FRAME_END_CLR);

		if (stream->stopping)
			/* To stop CIF ASAP, before FRAME_END irq */
			rkcif_write_register(cif_dev, CIF_REG_DVP_CTRL,
					     ctl & (~ENABLE_CAPTURE));
	}

	if ((intstat & FRAME_END)) {
		struct rkcif_buffer *active_buf = NULL;

		rkcif_write_register(cif_dev, CIF_REG_DVP_INTSTAT,
				     FRAME_END_CLR);

		if (stream->stopping) {
			rkcif_stream_stop(stream);
			stream->stopping = false;
			wake_up(&stream->wq_stopped);
			return;
		}

		if (lastline != stream->pixm.height ||
		    !(cif_frmst & CIF_F0_READY)) {
			/* Clearing status must be complete before fe packet
			 * arrives while cif is connected with mipi,
			 * so it should be placed before printing log here,
			 * otherwise it would be delayed.
			 * At the same time, don't clear the frame id
			 * for switching address.
			 */
			rkcif_write_register(cif_dev, CIF_REG_DVP_FRAME_STATUS,
					     FRM0_STAT_CLS);
			v4l2_err(&cif_dev->v4l2_dev,
				 "Bad frame, irq:0x%x frmst:0x%x size:%dx%d\n",
				 intstat, cif_frmst, lastline, lastpix);

			return;
		}

		if (frmid % 2 != 0) {
			stream->frame_phase = CIF_CSI_FRAME0_READY;
			if (stream->curr_buf)
				active_buf = stream->curr_buf;
		} else {
			stream->frame_phase = CIF_CSI_FRAME1_READY;
			if (stream->next_buf)
				active_buf = stream->next_buf;
		}

		/* In one-frame mode:
		 * 1,must clear status manually by writing 0 to enable
		 * the next frame end irq;
		 * 2,do not clear the frame id for switching address.
		 */
		rkcif_write_register(cif_dev, CIF_REG_DVP_FRAME_STATUS,
				     cif_frmst & FRM0_STAT_CLS);
		ret = rkcif_assign_new_buffer_oneframe(stream,
						 RKCIF_YUV_ADDR_STATE_UPDATE);

		if (active_buf && (!ret)) {
			active_buf->vb.sequence = stream->frame_idx - 1;
			rkcif_vb_done_tasklet(stream, active_buf);
		}

		cif_dev->irq_stats.frm_end_cnt[stream->id]++;
	}
}

static int rkcif_csi_g_mipi_id(struct v4l2_device *v4l2_dev,
			       unsigned int intstat)
{
	if (intstat & CSI_FRAME_END_ID0)
		return RKCIF_STREAM_MIPI_ID0;

	if (intstat & CSI_FRAME_END_ID1)
		return RKCIF_STREAM_MIPI_ID1;

	if (intstat & CSI_FRAME_END_ID2)
		return RKCIF_STREAM_MIPI_ID2;
	if (intstat & CSI_FRAME_END_ID3)
		return RKCIF_STREAM_MIPI_ID3;

	return -EINVAL;
}

static int rkcif_dvp_g_ch_id(struct v4l2_device *v4l2_dev,
			     u32 *intstat, u32 frm_stat)
{
	if (*intstat & DVP_FRAME_END_ID0) {
		*intstat &= ~DVP_FRAME_END_ID0;
		return RKCIF_STREAM_MIPI_ID0;
	}

	if (*intstat & DVP_FRAME_END_ID1) {
		*intstat &= ~DVP_FRAME_END_ID1;
		return RKCIF_STREAM_MIPI_ID1;
	}

	if (*intstat & DVP_FRAME_END_ID2) {
		*intstat &= ~DVP_FRAME_END_ID2;
		return RKCIF_STREAM_MIPI_ID2;
	}

	if (*intstat & DVP_FRAME_END_ID3) {
		*intstat &= ~DVP_FRAME_END_ID3;
		return RKCIF_STREAM_MIPI_ID3;
	}

	return -EINVAL;
}

static int rkcif_dvp_g_ch_id_by_fe(struct v4l2_device *v4l2_dev,
				   u32 intstat)
{
	if (intstat & DVP_ALL_END_ID0)
		return RKCIF_STREAM_MIPI_ID0;

	if (intstat & DVP_ALL_END_ID1)
		return RKCIF_STREAM_MIPI_ID1;

	if (intstat & DVP_ALL_END_ID2)
		return RKCIF_STREAM_MIPI_ID2;

	if (intstat & DVP_ALL_END_ID3)
		return RKCIF_STREAM_MIPI_ID3;

	return -EINVAL;
}

static bool rkcif_is_csi2_err_trigger_reset(struct rkcif_timer *timer)
{
	struct rkcif_device *dev = container_of(timer,
						struct rkcif_device,
						reset_watchdog_timer);
	struct rkcif_stream *stream = &dev->stream[RKCIF_STREAM_MIPI_ID0];
	bool is_triggered = false, is_assign_triggered = false, is_first_err = false;
	unsigned long flags;
	u64 cur_time, diff_time;

	spin_lock_irqsave(&timer->csi2_err_lock, flags);

	if (timer->csi2_err_cnt_even != 0 &&
	    timer->csi2_err_cnt_odd != 0) {
		timer->csi2_err_cnt_odd = 0;
		timer->csi2_err_cnt_even = 0;
		timer->reset_src = RKCIF_RESET_SRC_ERR_CSI2;
		timer->csi2_err_triggered_cnt++;
		if (timer->csi2_err_triggered_cnt == 1) {
			is_first_err = true;
			timer->csi2_first_err_timestamp = rkcif_time_get_ns(dev);
		}

		is_assign_triggered = true;

		v4l2_info(&dev->v4l2_dev,
			  "find csi2 err cnt is:%d\n",
			  timer->csi2_err_triggered_cnt);
	}

	if (!is_first_err) {
		if (timer->csi2_err_triggered_cnt >= 1) {
			cur_time = rkcif_time_get_ns(dev);
			diff_time = cur_time - timer->csi2_first_err_timestamp;
			diff_time = div_u64(diff_time, 1000000);
			if (diff_time >= timer->err_time_interval) {
				is_triggered = true;
				v4l2_info(&dev->v4l2_dev, "trigger reset for time out of csi err\n");
				goto end_judge;
			}

			if (!is_assign_triggered &&
			   (timer->csi2_err_cnt_odd == 0 ||
			    timer->csi2_err_cnt_even == 0)) {
				is_triggered = true;
				v4l2_info(&dev->v4l2_dev, "trigger reset for csi err\n");
				goto end_judge;
			}
		}
	}

	/*
	 * when fs cnt is beyond 2, it indicates that frame end is not coming,
	 * or fs and fe had been not paired.
	 */
	if (stream->is_fs_fe_not_paired ||
	    (stream->fs_cnt_in_single_frame > RKCIF_FS_DETECTED_NUM &&
	     dev->chip_id < CHIP_RK3588_CIF)) {
		is_triggered = true;
		v4l2_info(&dev->v4l2_dev, "reset for fs & fe not paired\n");
	}
end_judge:
	spin_unlock_irqrestore(&timer->csi2_err_lock, flags);

	return is_triggered;
}

static bool rkcif_is_triggered_monitoring(struct rkcif_device *dev)
{
	struct rkcif_timer *timer = &dev->reset_watchdog_timer;
	struct rkcif_stream *stream = &dev->stream[RKCIF_STREAM_MIPI_ID0];
	bool ret = false;

	if (timer->monitor_mode == RKCIF_MONITOR_MODE_IDLE)
		ret = false;

	if (timer->monitor_mode == RKCIF_MONITOR_MODE_CONTINUE ||
	    timer->monitor_mode == RKCIF_MONITOR_MODE_HOTPLUG) {
		if (stream->frame_idx >= timer->triggered_frame_num)
			ret = true;
	}

	if (timer->monitor_mode == RKCIF_MONITOR_MODE_TRIGGER) {
		timer->is_csi2_err_occurred = rkcif_is_csi2_err_trigger_reset(timer);
		ret = timer->is_csi2_err_occurred;
	}

	return ret;
}

s32 rkcif_get_sensor_vblank(struct rkcif_device *dev)
{
	struct rkcif_sensor_info *terminal_sensor = &dev->terminal_sensor;
	struct v4l2_subdev *sd = terminal_sensor->sd;
	struct v4l2_ctrl_handler *hdl = sd->ctrl_handler;
	struct v4l2_ctrl *ctrl = NULL;

	if (!list_empty(&hdl->ctrls)) {
		list_for_each_entry(ctrl, &hdl->ctrls, node) {
			if (ctrl->id == V4L2_CID_VBLANK)
				return ctrl->val;
		}
	}

	return 0;
}

s32 rkcif_get_sensor_vblank_def(struct rkcif_device *dev)
{
	struct rkcif_sensor_info *terminal_sensor = &dev->terminal_sensor;
	struct v4l2_subdev *sd = terminal_sensor->sd;
	struct v4l2_ctrl_handler *hdl = sd->ctrl_handler;
	struct v4l2_ctrl *ctrl = NULL;

	if (!list_empty(&hdl->ctrls)) {
		list_for_each_entry(ctrl, &hdl->ctrls, node) {
			if (ctrl->id == V4L2_CID_VBLANK)
				return ctrl->default_value;
		}
	}

	return 0;
}

static void rkcif_cal_csi_crop_width_vwidth(struct rkcif_stream *stream,
					    u32 raw_width, u32 *crop_width,
					    u32 *crop_vwidth)
{
	struct rkcif_device *dev = stream->cifdev;
	struct csi_channel_info *channel = &dev->channels[stream->id];
	const struct cif_output_fmt *fmt;
	u32 fourcc;

	fmt = rkcif_find_output_fmt(stream, stream->pixm.pixelformat);
	if (!fmt) {
		v4l2_err(&dev->v4l2_dev, "can not find output format: 0x%x",
			 stream->pixm.pixelformat);
		return;
	}

	*crop_width = raw_width;

	/*
	 * for mipi or lvds, when enable compact, the virtual width of raw10/raw12
	 * needs aligned with :ALIGN(bits_per_pixel * width / 8, 8), if enable 16bit mode
	 * needs aligned with :ALIGN(bits_per_pixel * width * 2, 8), to optimize reading and
	 * writing of ddr, aliged with 256
	 */
	if (fmt->fmt_type == CIF_FMT_TYPE_RAW && stream->is_compact &&
	    fmt->csi_fmt_val != CSI_WRDDR_TYPE_RGB888 &&
	    fmt->csi_fmt_val != CSI_WRDDR_TYPE_RGB565) {
		*crop_vwidth = ALIGN(raw_width * fmt->raw_bpp / 8, 256);
	} else {
		*crop_vwidth = ALIGN(raw_width * fmt->bpp[0] / 8, 8);
	}

	if (channel->fmt_val == CSI_WRDDR_TYPE_RGB888 || channel->fmt_val == CSI_WRDDR_TYPE_RGB565)
		*crop_width = raw_width * fmt->bpp[0] / 8;
	/*
	 * rk cif don't support output yuyv fmt data
	 * if user request yuyv fmt, the input mode must be RAW8
	 * and the width is double Because the real input fmt is
	 * yuyv
	 */
	fourcc = stream->cif_fmt_out->fourcc;
	if (fourcc == V4L2_PIX_FMT_YUYV || fourcc == V4L2_PIX_FMT_YVYU ||
	    fourcc == V4L2_PIX_FMT_UYVY || fourcc == V4L2_PIX_FMT_VYUY) {
		*crop_width = 2 * raw_width;
		*crop_vwidth *= 2;
	}
}

static void rkcif_dynamic_crop(struct rkcif_stream *stream)
{
	struct rkcif_device *cif_dev = stream->cifdev;
	struct v4l2_mbus_config *mbus;
	const struct cif_output_fmt *fmt;
	u32 raw_width, crop_width = 64, crop_vwidth = 64,
	    crop_height = 64, crop_x = 0, crop_y = 0;

	if (!cif_dev->active_sensor)
		return;

	mbus = &cif_dev->active_sensor->mbus;
	if (mbus->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus->type == V4L2_MBUS_CSI2_CPHY ||
	    mbus->type == V4L2_MBUS_CCP2) {
		struct csi_channel_info *channel = &cif_dev->channels[stream->id];

		if (channel->fmt_val == CSI_WRDDR_TYPE_RGB888 && cif_dev->chip_id < CHIP_RK3576_CIF)
			crop_x = 3 * stream->crop[CROP_SRC_ACT].left / 2;
		else if (channel->fmt_val == CSI_WRDDR_TYPE_RGB565)
			crop_x = 2 * stream->crop[CROP_SRC_ACT].left;
		else
			crop_x = stream->crop[CROP_SRC_ACT].left;

		crop_y = stream->crop[CROP_SRC_ACT].top;
		raw_width = stream->crop[CROP_SRC_ACT].width;
		crop_height = stream->crop[CROP_SRC_ACT].height;

		rkcif_cal_csi_crop_width_vwidth(stream,
						raw_width,
						&crop_width, &crop_vwidth);
		rkcif_write_register(cif_dev,
				     get_reg_index_of_id_crop_start(channel->id),
				     crop_y << 16 | crop_x);
		rkcif_write_register(cif_dev, get_reg_index_of_id_ctrl1(channel->id),
				     crop_height << 16 | crop_width);

		rkcif_write_register(cif_dev,
				     get_reg_index_of_frm0_y_vlw(channel->id),
				     crop_vwidth);
		rkcif_write_register(cif_dev,
				     get_reg_index_of_frm1_y_vlw(channel->id),
				     crop_vwidth);
		rkcif_write_register(cif_dev,
				     get_reg_index_of_frm0_uv_vlw(channel->id),
				     crop_vwidth);
		rkcif_write_register(cif_dev,
				     get_reg_index_of_frm1_uv_vlw(channel->id),
				     crop_vwidth);
	} else {

		raw_width = stream->crop[CROP_SRC_ACT].width;
		crop_width = raw_width;
		crop_vwidth = raw_width;
		crop_height = stream->crop[CROP_SRC_ACT].height;
		crop_x = stream->crop[CROP_SRC_ACT].left;
		crop_y = stream->crop[CROP_SRC_ACT].top;

		rkcif_write_register(cif_dev, CIF_REG_DVP_CROP,
				     crop_y << CIF_CROP_Y_SHIFT | crop_x);

		if (stream->cif_fmt_in->fmt_type == CIF_FMT_TYPE_RAW) {
			fmt = rkcif_find_output_fmt(stream, stream->pixm.pixelformat);
			crop_vwidth = raw_width * rkcif_cal_raw_vir_line_ratio(stream, fmt);
		}
		rkcif_write_register(cif_dev, CIF_REG_DVP_VIR_LINE_WIDTH, crop_vwidth);

		rkcif_write_register(cif_dev, CIF_REG_DVP_SET_SIZE,
				     crop_height << 16 | crop_width);
	}

	stream->crop_dyn_en = false;
}

static void rkcif_monitor_reset_event(struct rkcif_device *dev)
{
	struct rkcif_stream *stream = NULL;
	struct rkcif_timer *timer = &dev->reset_watchdog_timer;
	unsigned int cycle = 0;
	u64 fps, timestamp0, timestamp1;
	unsigned long flags, fps_flags;
	int i = 0;

	if (timer->is_running)
		return;

	if (timer->monitor_mode == RKCIF_MONITOR_MODE_IDLE)
		return;

	for (i = 0; i < RKCIF_MAX_STREAM_MIPI; i++) {
		stream = &dev->stream[i];
		if (stream->state == RKCIF_STATE_STREAMING)
			break;
	}

	if (i >= RKCIF_MAX_STREAM_MIPI)
		return;

	timer->is_triggered = rkcif_is_triggered_monitoring(dev);

	if (timer->is_triggered) {
		struct v4l2_rect *raw_rect = &dev->terminal_sensor.raw_rect;
		enum rkcif_monitor_mode mode;
		s32 vblank = 0;
		u32 vts = 0;
		u64 numerator = 0;
		u64 denominator = 0;

		if (stream->frame_idx  > 2) {
			spin_lock_irqsave(&stream->fps_lock, fps_flags);
			timestamp0 = stream->fps_stats.frm0_timestamp;
			timestamp1 = stream->fps_stats.frm1_timestamp;
			spin_unlock_irqrestore(&stream->fps_lock, fps_flags);

			fps = timestamp0 > timestamp1 ?
			      timestamp0 - timestamp1 : timestamp1 - timestamp0;
			fps = div_u64(fps, 1000);
		} else {
			numerator = dev->terminal_sensor.src_fi.interval.numerator;
			denominator = dev->terminal_sensor.src_fi.interval.denominator;
			fps = div_u64(1000000 * numerator, denominator);
		}
		spin_lock_irqsave(&timer->timer_lock, flags);

		timer->frame_end_cycle_us = fps;

		vblank = rkcif_get_sensor_vblank(dev);
		timer->raw_height = raw_rect->height;
		vts = timer->raw_height + vblank;
		timer->vts = vts;

		timer->line_end_cycle = div_u64(timer->frame_end_cycle_us, timer->vts);
		fps = div_u64(timer->frame_end_cycle_us, 1000);
		cycle = fps * timer->frm_num_of_monitor_cycle;
		timer->cycle = msecs_to_jiffies(cycle);

		timer->run_cnt = 0;
		timer->is_running = true;
		timer->is_buf_stop_update = false;
		for (i = 0; i < dev->num_channels; i++) {
			stream = &dev->stream[i];
			if (stream->state == RKCIF_STATE_STREAMING)
				timer->last_buf_wakeup_cnt[i] = stream->buf_wake_up_cnt;
		}
		/* in trigger mode, monitoring count is fps */
		mode = timer->monitor_mode;
		if (mode == RKCIF_MONITOR_MODE_CONTINUE ||
		    mode == RKCIF_MONITOR_MODE_HOTPLUG)
			timer->max_run_cnt = 0xffffffff - CIF_TIMEOUT_FRAME_NUM;
		else
			timer->max_run_cnt = div_u64(1000, fps) * 1;

		timer->timer.expires = jiffies + timer->cycle;
		mod_timer(&timer->timer, timer->timer.expires);

		spin_unlock_irqrestore(&timer->timer_lock, flags);

		v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev,
			 "%s:mode:%d, raw height:%d,vblank:%d, cycle:%ld, fps:%llu\n",
			  __func__, timer->monitor_mode, raw_rect->height,
			  vblank, timer->cycle, div_u64(1000, fps));
	}
}

static void rkcif_rdbk_frame_end(struct rkcif_stream *stream)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_sensor_info *sensor = &stream->cifdev->terminal_sensor;
	u32 denominator, numerator;
	u64 l_ts, m_ts, s_ts, time = 30000000LL;
	int ret, fps = -1;
	int i = 0;
	unsigned long flags;

	if (dev->hdr.hdr_mode == HDR_X2) {
		if (stream->id != RKCIF_STREAM_MIPI_ID1 ||
		    dev->stream[RKCIF_STREAM_MIPI_ID0].state != RKCIF_STATE_STREAMING ||
		    dev->stream[RKCIF_STREAM_MIPI_ID1].state != RKCIF_STATE_STREAMING)
			return;
	} else if (dev->hdr.hdr_mode == HDR_X3) {
		if (stream->id != RKCIF_STREAM_MIPI_ID2 ||
		    dev->stream[RKCIF_STREAM_MIPI_ID0].state != RKCIF_STATE_STREAMING ||
		    dev->stream[RKCIF_STREAM_MIPI_ID1].state != RKCIF_STATE_STREAMING ||
		    dev->stream[RKCIF_STREAM_MIPI_ID2].state != RKCIF_STATE_STREAMING)
			return;
	}

	numerator = sensor->fi.interval.numerator;
	denominator = sensor->fi.interval.denominator;
	if (denominator && numerator)
		time = numerator * 1000 / denominator * 1000 * 1000;

	if (dev->hdr.hdr_mode == HDR_X3) {
		if (dev->rdbk_buf[RDBK_L] &&
		    dev->rdbk_buf[RDBK_M] &&
		    dev->rdbk_buf[RDBK_S]) {
			l_ts = dev->rdbk_buf[RDBK_L]->fe_timestamp;
			m_ts = dev->rdbk_buf[RDBK_M]->fe_timestamp;
			s_ts = dev->rdbk_buf[RDBK_S]->fe_timestamp;

			if (m_ts < l_ts || s_ts < m_ts) {
				v4l2_err(&dev->v4l2_dev,
					 "s/m/l frame err, timestamp s:%lld m:%lld l:%lld\n",
					 s_ts, m_ts, l_ts);
				return;
			}

			if ((m_ts - l_ts) > time || (s_ts - m_ts) > time) {
				ret = v4l2_subdev_call(sensor->sd,
						       video,
						       g_frame_interval,
						       &sensor->fi);
				if (!ret) {
					denominator = sensor->fi.interval.denominator;
					numerator = sensor->fi.interval.numerator;
					if (denominator && numerator) {
						time = numerator * 1000 / denominator * 1000 * 1000;
						fps = denominator / numerator;
					}
				}

				if ((m_ts - l_ts) > time || (s_ts - m_ts) > time) {
					v4l2_err(&dev->v4l2_dev,
						 "timestamp no match, s:%lld m:%lld l:%lld, fps:%d\n",
						 s_ts, m_ts, l_ts, fps);
					return;
				}
			}
			dev->rdbk_buf[RDBK_M]->vb.sequence = dev->rdbk_buf[RDBK_L]->vb.sequence;
			dev->rdbk_buf[RDBK_S]->vb.sequence = dev->rdbk_buf[RDBK_L]->vb.sequence;
			if (dev->is_support_tools &&
			    dev->stream[RKCIF_STREAM_MIPI_ID0].tools_vdev->state == RKCIF_STATE_STREAMING &&
			    dev->stream[RKCIF_STREAM_MIPI_ID1].tools_vdev->state == RKCIF_STATE_STREAMING &&
			    dev->stream[RKCIF_STREAM_MIPI_ID2].tools_vdev->state == RKCIF_STATE_STREAMING) {
				for (i = 0; i < 3; i++) {
					spin_lock_irqsave(&dev->stream[i].tools_vdev->vbq_lock, flags);
					list_add_tail(&dev->rdbk_buf[i]->queue,
						      &dev->stream[i].tools_vdev->buf_done_head);
					if (!work_busy(&dev->stream[i].tools_vdev->work))
						schedule_work(&dev->stream[i].tools_vdev->work);
					spin_unlock_irqrestore(&dev->stream[i].tools_vdev->vbq_lock, flags);
				}
			} else {
				rkcif_vb_done_tasklet(&dev->stream[RKCIF_STREAM_MIPI_ID0], dev->rdbk_buf[RDBK_L]);
				rkcif_vb_done_tasklet(&dev->stream[RKCIF_STREAM_MIPI_ID1], dev->rdbk_buf[RDBK_M]);
				rkcif_vb_done_tasklet(&dev->stream[RKCIF_STREAM_MIPI_ID2], dev->rdbk_buf[RDBK_S]);
			}
		} else {
			if (!dev->rdbk_buf[RDBK_L])
				v4l2_err(&dev->v4l2_dev, "lost long frames\n");
			if (!dev->rdbk_buf[RDBK_M])
				v4l2_err(&dev->v4l2_dev, "lost medium frames\n");
			if (!dev->rdbk_buf[RDBK_S])
				v4l2_err(&dev->v4l2_dev, "lost short frames\n");
			return;
		}
	} else if (dev->hdr.hdr_mode == HDR_X2) {
		if (dev->rdbk_buf[RDBK_L] && dev->rdbk_buf[RDBK_M]) {
			l_ts = dev->rdbk_buf[RDBK_L]->fe_timestamp;
			s_ts = dev->rdbk_buf[RDBK_M]->fe_timestamp;

			if (s_ts < l_ts) {
				v4l2_err(&dev->v4l2_dev,
					 "s/l frame err, timestamp s:%lld l:%lld\n",
					 s_ts, l_ts);
				return;
			}

			if ((s_ts - l_ts) > time) {
				ret = v4l2_subdev_call(sensor->sd,
						       video,
						       g_frame_interval,
						       &sensor->fi);
				if (!ret) {
					denominator = sensor->fi.interval.denominator;
					numerator = sensor->fi.interval.numerator;
					if (denominator && numerator) {
						time = numerator * 1000 / denominator * 1000 * 1000;
						fps = denominator / numerator;
					}
				}
				if ((s_ts - l_ts) > time) {
					v4l2_err(&dev->v4l2_dev,
						 "timestamp no match, s:%lld l:%lld, fps:%d\n",
						 s_ts, l_ts, fps);
					return;
				}
			}
			dev->rdbk_buf[RDBK_M]->vb.sequence = dev->rdbk_buf[RDBK_L]->vb.sequence;
			if (dev->is_support_tools &&
			    dev->stream[RKCIF_STREAM_MIPI_ID0].tools_vdev->state == RKCIF_STATE_STREAMING &&
			    dev->stream[RKCIF_STREAM_MIPI_ID1].tools_vdev->state == RKCIF_STATE_STREAMING) {
				for (i = 0; i < 2; i++) {
					spin_lock_irqsave(&dev->stream[i].tools_vdev->vbq_lock, flags);
					list_add_tail(&dev->rdbk_buf[i]->queue,
						      &dev->stream[i].tools_vdev->buf_done_head);
					if (!work_busy(&dev->stream[i].tools_vdev->work))
						schedule_work(&dev->stream[i].tools_vdev->work);
					spin_unlock_irqrestore(&dev->stream[i].tools_vdev->vbq_lock, flags);
				}
			} else {
				rkcif_vb_done_tasklet(&dev->stream[RKCIF_STREAM_MIPI_ID0], dev->rdbk_buf[RDBK_L]);
				rkcif_vb_done_tasklet(&dev->stream[RKCIF_STREAM_MIPI_ID1], dev->rdbk_buf[RDBK_M]);
			}
		} else {
			if (!dev->rdbk_buf[RDBK_L])
				v4l2_err(&dev->v4l2_dev, "lost long frames\n");
			if (!dev->rdbk_buf[RDBK_M])
				v4l2_err(&dev->v4l2_dev, "lost short frames\n");
			return;
		}
	}

	dev->rdbk_buf[RDBK_L] = NULL;
	dev->rdbk_buf[RDBK_M] = NULL;
	dev->rdbk_buf[RDBK_S] = NULL;
}

static void rkcif_buf_done_with_tools(struct rkcif_stream *stream,
				      struct rkcif_buffer *active_buf)
{
	unsigned long flags;

	spin_lock_irqsave(&stream->tools_vdev->vbq_lock, flags);
	if (stream->tools_vdev->state == RKCIF_STATE_STREAMING) {
		list_add_tail(&active_buf->queue, &stream->tools_vdev->buf_done_head);
		if (!work_busy(&stream->tools_vdev->work))
			schedule_work(&stream->tools_vdev->work);
	} else {
		rkcif_vb_done_tasklet(stream, active_buf);
	}
	spin_unlock_irqrestore(&stream->tools_vdev->vbq_lock, flags);
}

static void rkcif_buf_done_prepare(struct rkcif_stream *stream,
				   struct rkcif_buffer *active_buf,
				   int mipi_id,
				   u32 mode)
{
	unsigned long flags;
	struct vb2_v4l2_buffer *vb_done = NULL;
	struct rkcif_device *cif_dev = stream->cifdev;

	if (active_buf) {
		vb_done = &active_buf->vb;
		if (cif_dev->chip_id < CHIP_RK3588_CIF &&
		    cif_dev->active_sensor->mbus.type == V4L2_MBUS_BT656)
			vb_done->vb2_buf.timestamp = stream->readout.fe_timestamp;
		else
			vb_done->vb2_buf.timestamp = stream->readout.fs_timestamp;
		vb_done->sequence = stream->frame_idx - 1;
		active_buf->fe_timestamp = rkcif_time_get_ns(cif_dev);
		if (stream->is_line_wake_up) {
			spin_lock_irqsave(&stream->fps_lock, flags);
			if (mode)
				stream->fps_stats.frm0_timestamp = vb_done->vb2_buf.timestamp;
			else
				stream->fps_stats.frm1_timestamp = vb_done->vb2_buf.timestamp;
			stream->readout.wk_timestamp = vb_done->vb2_buf.timestamp;
			spin_unlock_irqrestore(&stream->fps_lock, flags);
		}
		if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT ||
		    rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO)
			vb_done->sequence /= 2;
		if (stream->cur_skip_frame) {
			rkcif_buf_queue(&active_buf->vb.vb2_buf);
			return;
		}
		if (cif_dev->channels[0].capture_info.mode == RKMODULE_ONE_CH_TO_MULTI_ISP)
			vb_done->sequence /= cif_dev->channels[0].capture_info.one_to_multi.isp_num;
	} else if (((cif_dev->hdr.hdr_mode == HDR_X2 && stream->id < 2) ||
		    (cif_dev->hdr.hdr_mode == HDR_X3 && stream->id < 3))
		   && cif_dev->rdbk_buf[stream->id]) {
		vb_done = &cif_dev->rdbk_buf[stream->id]->vb;
		if (cif_dev->chip_id < CHIP_RK3588_CIF &&
		    cif_dev->active_sensor->mbus.type == V4L2_MBUS_BT656)
			vb_done->vb2_buf.timestamp = stream->readout.fe_timestamp;
		else
			vb_done->vb2_buf.timestamp = stream->readout.fs_timestamp;
		vb_done->sequence = stream->frame_idx - 1;
		cif_dev->rdbk_buf[stream->id]->fe_timestamp = rkcif_time_get_ns(cif_dev);
	}

	if (cif_dev->hdr.hdr_mode == NO_HDR || cif_dev->hdr.hdr_mode == HDR_COMPR ||
	    (cif_dev->hdr.hdr_mode == HDR_X2 && stream->id > 1) ||
	    (cif_dev->hdr.hdr_mode == HDR_X3 && stream->id > 2)) {
		if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT ||
		    rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO) {
			if (stream->frame_phase == CIF_CSI_FRAME1_READY && active_buf) {
				if (cif_dev->is_support_tools && stream->tools_vdev)
					rkcif_buf_done_with_tools(stream, active_buf);
				else
					rkcif_vb_done_tasklet(stream, active_buf);
			}
		} else {
			if (active_buf) {
				if (cif_dev->is_support_tools && stream->tools_vdev)
					rkcif_buf_done_with_tools(stream, active_buf);
				else
					rkcif_vb_done_tasklet(stream, active_buf);
			}
		}
	} else {
		if (cif_dev->is_start_hdr) {
			spin_lock_irqsave(&cif_dev->hdr_lock, flags);
			if (mipi_id == RKCIF_STREAM_MIPI_ID0) {
				if (cif_dev->rdbk_buf[RDBK_L] && active_buf) {
					v4l2_err(&cif_dev->v4l2_dev,
						 "multiple long data in %s frame,frm_idx:%d,state:0x%x\n",
						 cif_dev->hdr.hdr_mode == HDR_X2 ? "hdr_x2" : "hdr_x3",
						 stream->frame_idx,
						 cif_dev->rdbk_buf[RDBK_L]->vb.vb2_buf.state);
					cif_dev->rdbk_buf[RDBK_L]->vb.vb2_buf.state = VB2_BUF_STATE_ACTIVE;
					rkcif_buf_queue(&cif_dev->rdbk_buf[RDBK_L]->vb.vb2_buf);
					cif_dev->rdbk_buf[RDBK_L] = NULL;
				}
				if (active_buf)
					cif_dev->rdbk_buf[RDBK_L] = active_buf;
			} else if (mipi_id == RKCIF_STREAM_MIPI_ID1) {
				if (cif_dev->rdbk_buf[RDBK_M] && active_buf) {
					v4l2_err(&cif_dev->v4l2_dev,
						 "multiple %s frame,frm_idx:%d,state:0x%x\n",
						 cif_dev->hdr.hdr_mode == HDR_X2 ? "short data in hdr_x2" : "medium data in hdr_x3",
						 stream->frame_idx,
						 cif_dev->rdbk_buf[RDBK_M]->vb.vb2_buf.state);
					cif_dev->rdbk_buf[RDBK_M]->vb.vb2_buf.state = VB2_BUF_STATE_ACTIVE;
					rkcif_buf_queue(&cif_dev->rdbk_buf[RDBK_M]->vb.vb2_buf);
					cif_dev->rdbk_buf[RDBK_M] = NULL;
				}
				if (active_buf)
					cif_dev->rdbk_buf[RDBK_M] = active_buf;
				if (cif_dev->hdr.hdr_mode == HDR_X2)
					rkcif_rdbk_frame_end(stream);
			} else if (mipi_id == RKCIF_STREAM_MIPI_ID2) {
				if (cif_dev->rdbk_buf[RDBK_S] && active_buf) {
					v4l2_err(&cif_dev->v4l2_dev,
						 "multiple %s frame, frm_idx:%d,state:0x%x\n",
						 cif_dev->hdr.hdr_mode == HDR_X2 ? "err short data in hdr_x3" : "short data in hdr_x3",
						 stream->frame_idx,
						 cif_dev->rdbk_buf[RDBK_S]->vb.vb2_buf.state);
					cif_dev->rdbk_buf[RDBK_S]->vb.vb2_buf.state = VB2_BUF_STATE_ACTIVE;
					rkcif_buf_queue(&cif_dev->rdbk_buf[RDBK_S]->vb.vb2_buf);
					cif_dev->rdbk_buf[RDBK_S] = NULL;
				}
				if (active_buf)
					cif_dev->rdbk_buf[RDBK_S] = active_buf;
				if (cif_dev->hdr.hdr_mode == HDR_X3)
					rkcif_rdbk_frame_end(stream);
			}
			spin_unlock_irqrestore(&cif_dev->hdr_lock, flags);
		} else {
			if (active_buf) {
				vb_done->vb2_buf.state = VB2_BUF_STATE_ACTIVE;
				rkcif_buf_queue(&vb_done->vb2_buf);
			}

			v4l2_info(&cif_dev->v4l2_dev,
				  "warning:hdr runs stream[%d], stream[0]:%s stream[1]:%s stream[2]:%s stream[3]:%s\n",
				  stream->id,
				  cif_dev->stream[0].state != RKCIF_STATE_STREAMING ? "stopped" : "running",
				  cif_dev->stream[1].state != RKCIF_STATE_STREAMING ? "stopped" : "running",
				  cif_dev->stream[2].state != RKCIF_STATE_STREAMING ? "stopped" : "running",
				  cif_dev->stream[3].state != RKCIF_STATE_STREAMING ? "stopped" : "running");
		}
	}

}

static void rkcif_line_wake_up_interlace(struct rkcif_stream *stream,
						 int mipi_id)
{
	struct rkcif_device *cif_dev = stream->cifdev;
	struct rkcif_buffer *active_buf = NULL;
	struct rkcif_dummy_buffer *dummy_buf = &cif_dev->hw_dev->dummy_buf;
	int frame_id = 0;
	int fe_interlaced_phase = 0;

	if (stream->id == 0)
		frame_id = rkcif_read_register(cif_dev, CIF_REG_MIPI_FRAME_NUM_VC0);
	else if (stream->id == 1)
		frame_id = rkcif_read_register(cif_dev, CIF_REG_MIPI_FRAME_NUM_VC1);
	else if (stream->id == 2)
		frame_id = rkcif_read_register(cif_dev, CIF_REG_MIPI_FRAME_NUM_VC2);
	else if (stream->id == 3)
		frame_id = rkcif_read_register(cif_dev, CIF_REG_MIPI_FRAME_NUM_VC3);

	if ((frame_id & 0xffff) % 2 == stream->odd_frame_id) {
		//odd frame
		if (stream->odd_frame_first)
			fe_interlaced_phase = CIF_CSI_FRAME0_READY;
		else
			fe_interlaced_phase = CIF_CSI_FRAME1_READY;
	} else {
		//even frame
		if (stream->odd_frame_first)
			fe_interlaced_phase = CIF_CSI_FRAME1_READY;
		else
			fe_interlaced_phase = CIF_CSI_FRAME0_READY;
	}

	v4l2_dbg(rkcif_debug, 3, &cif_dev->v4l2_dev,
		 "stream[%d] mipi fe interlace phase %d, frame num 0x%x\n",
		 stream->id, fe_interlaced_phase, frame_id);

	if (fe_interlaced_phase == CIF_CSI_FRAME1_READY &&
	    stream->last_fe_interlaced_phase == CIF_CSI_FRAME0_READY) {
		if (dummy_buf->vaddr ||  stream->next_buf) {
			active_buf = stream->curr_buf;
			if (active_buf) {
				active_buf->vb.vb2_buf.timestamp = stream->readout.fs_timestamp;
				active_buf->vb.sequence = (stream->frame_idx - 1) / 2;
				if (stream->interlaced_bad_frame) {
					stream->interlaced_bad_frame = false;
					rkcif_buf_queue(&active_buf->vb.vb2_buf);
					v4l2_dbg(rkcif_debug, 3, &cif_dev->v4l2_dev,
						 "stream[%d] mipi fe interlace bad frame queue, phase %d, line %d\n",
						 stream->id, fe_interlaced_phase, __LINE__);
				} else {
					rkcif_vb_done_tasklet(stream, active_buf);
				}
				stream->curr_buf = stream->next_buf;
				stream->next_buf = NULL;
			} else {
				stream->curr_buf = stream->next_buf;
				stream->next_buf = NULL;
			}
		}
	}
	stream->last_fe_interlaced_phase = fe_interlaced_phase;
}

static void rkcif_line_wake_up(struct rkcif_stream *stream, int mipi_id)
{
	u32 mode;
	struct rkcif_buffer *active_buf = NULL;
	int ret = 0;

	mode = stream->line_int_cnt % 2;
	if (mode) {
		if (stream->curr_buf)
			active_buf = stream->curr_buf;
	} else {
		if (stream->next_buf)
			active_buf = stream->next_buf;
	}

	if (stream->stopping) {
		stream->is_can_stop = true;
		return;
	}
	ret = rkcif_get_new_buffer_wake_up_mode(stream);
	if (ret)
		return;

	if (active_buf && stream->low_latency)
		rkcif_add_fence_to_vb_done(stream, &active_buf->vb);

	rkcif_buf_done_prepare(stream, active_buf, mipi_id, mode);
}

static void rkcif_store_last_buf_for_online(struct rkcif_stream *stream,
					    struct rkcif_rx_buffer *buf)
{
	struct rkcif_device *dev = stream->cifdev;
	struct v4l2_mbus_config *mbus_cfg = &dev->active_sensor->mbus;
	u32 frm0_addr_y, frm1_addr_y;

	INIT_LIST_HEAD(&stream->rx_buf_head);
	stream->curr_buf_toisp = buf;
	stream->next_buf_toisp = buf;
	if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY ||
	    mbus_cfg->type == V4L2_MBUS_CCP2) {
		frm0_addr_y = get_reg_index_of_frm0_y_addr(stream->id);
		frm1_addr_y = get_reg_index_of_frm1_y_addr(stream->id);
	} else {
		frm0_addr_y = get_dvp_reg_index_of_frm0_y_addr(stream->id);
		frm1_addr_y = get_dvp_reg_index_of_frm1_y_addr(stream->id);
	}
	rkcif_write_register(dev, frm0_addr_y,
			     buf->dummy.dma_addr);
	rkcif_write_register(dev, frm1_addr_y,
			     buf->dummy.dma_addr);
}

static void rkcif_line_wake_up_rdbk(struct rkcif_stream *stream, int mipi_id)
{
	u32 mode;
	struct rkcif_rx_buffer *active_buf = NULL;
	struct sditf_priv *priv = NULL;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&stream->vbq_lock, flags);
	if (stream->cifdev->is_thunderboot &&
	    stream->thunderboot_skip_interval &&
	    ((stream->frame_idx - 1) % stream->thunderboot_skip_interval != 0)) {
		spin_unlock_irqrestore(&stream->vbq_lock, flags);
		return;
	}
	if (stream->thunderboot_skip_interval) {
		if (stream->frame_idx < stream->thunderboot_skip_interval)
			stream->sequence = stream->frame_idx - 1;
		else
			stream->sequence = stream->frame_idx / stream->thunderboot_skip_interval;
	} else {
		stream->sequence = stream->frame_idx - 1;
	}
	spin_unlock_irqrestore(&stream->vbq_lock, flags);

	mode = stream->line_int_cnt % 2;
	if (mode) {
		if (stream->curr_buf_toisp)
			active_buf = stream->curr_buf_toisp;
		stream->frame_phase = CIF_CSI_FRAME0_READY;
	} else {
		if (stream->next_buf_toisp)
			active_buf = stream->next_buf_toisp;
		stream->frame_phase = CIF_CSI_FRAME1_READY;
	}

	if (!active_buf) {
		v4l2_err(&stream->cifdev->v4l2_dev,
			 "err buffer state in %s\n",
			 __func__);
		return;
	}

	if (stream->stopping) {
		stream->is_can_stop = true;
		return;
	}
	ret = rkcif_get_new_buffer_wake_up_mode_rdbk(stream);
	v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
		 "%d frame_idx %d, last_rx_buf_idx %d cur dma buf %x, ret %d\n",
		 __LINE__, stream->frame_idx, stream->last_rx_buf_idx,
		 (u32)active_buf->dummy.dma_addr, ret);

	priv = stream->cifdev->sditf[0];
	if (stream->cur_stream_mode & RKCIF_STREAM_MODE_TOISP_RDBK) {
		spin_lock_irqsave(&stream->vbq_lock, flags);
		if (stream->cifdev->is_thunderboot &&
		    (stream->frame_idx - 1) == stream->last_rx_buf_idx &&
		     stream->cifdev->is_rdbk_to_online) {
			stream->cur_stream_mode &= ~RKCIF_STREAM_MODE_TOISP_RDBK;
			stream->cur_stream_mode |= RKCIF_STREAM_MODE_TOISP;
			stream->cifdev->wait_line = 0;
			v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
				 "stream[%d] frame_idx %d, last_rx_buf_idx %d cur dma buf %x,  change to online\n",
				 stream->id, stream->frame_idx, stream->last_rx_buf_idx,
				 (u32)active_buf->dummy.dma_addr);
			if (stream->cifdev->hdr.hdr_mode == NO_HDR ||
			    (priv->hdr_cfg.hdr_mode == HDR_X2 && stream->id == 1) ||
			    (priv->hdr_cfg.hdr_mode == HDR_X3 && stream->id == 2)) {
				stream->to_stop_dma = RKCIF_DMAEN_BY_ISP;
				rkcif_stop_dma_capture(stream);
			}
			active_buf->dbufs.is_switch = true;
			if ((priv->hdr_cfg.hdr_mode == HDR_X2 && stream->id != 1) ||
			    (priv->hdr_cfg.hdr_mode == HDR_X3 && stream->id != 2)) {
				rkcif_store_last_buf_for_online(stream, active_buf);
				stream->is_change_toisp = true;
			}
		}
		spin_unlock_irqrestore(&stream->vbq_lock, flags);
		active_buf->dbufs.sequence = stream->sequence;
		active_buf->dbufs.timestamp = stream->readout.fs_timestamp;
		active_buf->fe_timestamp = rkcif_time_get_ns(stream->cifdev);
		stream->last_frame_idx = stream->frame_idx;
		if (stream->cifdev->hdr.hdr_mode == NO_HDR) {
			rkcif_s_rx_buffer(stream, &active_buf->dbufs);
			if (stream->cifdev->is_support_tools && stream->tools_vdev)
				rkcif_rdbk_with_tools(stream, active_buf);
		} else {
			rkcif_rdbk_frame_end_toisp(stream, active_buf);
		}
	}

}

static void rkcif_deal_readout_time(struct rkcif_stream *stream)
{
	struct rkcif_device *cif_dev = stream->cifdev;
	struct rkcif_stream *detect_stream = &cif_dev->stream[0];
	unsigned long flags;

	spin_lock_irqsave(&stream->fps_lock, flags);
	stream->readout.fe_timestamp = rkcif_time_get_ns(cif_dev);

	if (cif_dev->inf_id == RKCIF_DVP) {
		spin_unlock_irqrestore(&stream->fps_lock, flags);
		return;
	}

	if (stream->id == RKCIF_STREAM_MIPI_ID0)
		detect_stream->readout.readout_time = stream->readout.fe_timestamp - stream->readout.fs_timestamp;

	if ((cif_dev->hdr.hdr_mode == NO_HDR || cif_dev->hdr.hdr_mode == HDR_COMPR) &&
	    (stream->id == RKCIF_STREAM_MIPI_ID0)) {
		detect_stream->readout.early_time = stream->readout.fe_timestamp - stream->readout.wk_timestamp;

	} else if ((cif_dev->hdr.hdr_mode == HDR_X2) && (stream->id == RKCIF_STREAM_MIPI_ID1)) {
		detect_stream->readout.early_time = stream->readout.fe_timestamp - stream->readout.wk_timestamp;
		detect_stream->readout.total_time = stream->readout.fe_timestamp - detect_stream->readout.fe_timestamp;
		detect_stream->readout.total_time += detect_stream->readout.readout_time;
	} else if ((cif_dev->hdr.hdr_mode == HDR_X3) && (stream->id == RKCIF_STREAM_MIPI_ID2)) {
		detect_stream->readout.early_time = stream->readout.fe_timestamp - stream->readout.wk_timestamp;
		detect_stream->readout.total_time = stream->readout.fe_timestamp - detect_stream->readout.fe_timestamp;
		detect_stream->readout.total_time += detect_stream->readout.readout_time;
	}
	if (!stream->is_line_wake_up)
		detect_stream->readout.early_time = 0;
	spin_unlock_irqrestore(&stream->fps_lock, flags);
}

static void rkcif_update_stream_interlace(struct rkcif_device *cif_dev,
					  struct rkcif_stream *stream,
					  int mipi_id)
{
	struct rkcif_buffer *active_buf = NULL;
	struct rkcif_dummy_buffer *dummy_buf = &cif_dev->hw_dev->dummy_buf;
	unsigned long flags;
	int frame_id = 0;
	int fe_interlaced_phase = 0;

	if (stream->id == 0)
		frame_id = rkcif_read_register(cif_dev, CIF_REG_MIPI_FRAME_NUM_VC0);
	else if (stream->id == 1)
		frame_id = rkcif_read_register(cif_dev, CIF_REG_MIPI_FRAME_NUM_VC1);
	else if (stream->id == 2)
		frame_id = rkcif_read_register(cif_dev, CIF_REG_MIPI_FRAME_NUM_VC2);
	else if (stream->id == 3)
		frame_id = rkcif_read_register(cif_dev, CIF_REG_MIPI_FRAME_NUM_VC3);

	if ((frame_id & 0xffff) % 2 == stream->odd_frame_id) {
		//odd frame
		if (stream->odd_frame_first)
			fe_interlaced_phase = CIF_CSI_FRAME0_READY;
		else
			fe_interlaced_phase = CIF_CSI_FRAME1_READY;
	} else {
		//even frame
		if (stream->odd_frame_first)
			fe_interlaced_phase = CIF_CSI_FRAME1_READY;
		else
			fe_interlaced_phase = CIF_CSI_FRAME0_READY;
	}

	v4l2_dbg(rkcif_debug, 3, &cif_dev->v4l2_dev,
		 "stream[%d] mipi fe interlace phase %d, frame num 0x%x\n",
		 stream->id, fe_interlaced_phase, frame_id);

	if (!stream->is_line_wake_up) {
		spin_lock_irqsave(&stream->fps_lock, flags);
		if (stream->frame_phase & CIF_CSI_FRAME0_READY)
			stream->fps_stats.frm0_timestamp = ktime_get_ns();
		else if (stream->frame_phase & CIF_CSI_FRAME1_READY)
			stream->fps_stats.frm1_timestamp = ktime_get_ns();
		spin_unlock_irqrestore(&stream->fps_lock, flags);
	}

	if (!stream->is_line_wake_up) {
		if (fe_interlaced_phase == CIF_CSI_FRAME1_READY &&
		    stream->last_fe_interlaced_phase == CIF_CSI_FRAME0_READY) {
			if (dummy_buf->vaddr ||  stream->next_buf) {
				active_buf = stream->curr_buf;
				if (active_buf) {
					active_buf->vb.vb2_buf.timestamp = stream->readout.fs_timestamp;
					active_buf->vb.sequence = (stream->frame_idx - 1) / 2;
					if (stream->interlaced_bad_frame) {
						stream->interlaced_bad_frame = false;
						rkcif_buf_queue(&active_buf->vb.vb2_buf);
						v4l2_dbg(rkcif_debug, 3, &cif_dev->v4l2_dev,
							 "stream[%d] mipi fe interlace bad frame queue, phase %d, line %d\n",
							 stream->id, fe_interlaced_phase, __LINE__);
					} else {
						rkcif_vb_done_tasklet(stream, active_buf);
					}
					stream->curr_buf = stream->next_buf;
					stream->next_buf = NULL;
				} else {
					stream->curr_buf = stream->next_buf;
					stream->next_buf = NULL;
				}
			}
		}
	}
	rkcif_deal_readout_time(stream);
	stream->last_fe_interlaced_phase = fe_interlaced_phase;
}

static void rkcif_update_stream(struct rkcif_device *cif_dev,
				struct rkcif_stream *stream,
				int mipi_id)
{
	struct rkcif_buffer *active_buf = NULL;
	unsigned long flags;
	int ret = 0;

	if (stream->frame_phase == (CIF_CSI_FRAME0_READY | CIF_CSI_FRAME1_READY)) {
		cif_dev->err_state |= (RKCIF_ERR_ID0_TRIG_SIMULT << stream->id);
		cif_dev->irq_stats.trig_simult_cnt[stream->id]++;
		return;
	}
	if (!stream->is_line_wake_up) {

		spin_lock_irqsave(&stream->fps_lock, flags);
		if (stream->frame_phase & CIF_CSI_FRAME0_READY) {
			if (stream->curr_buf)
				active_buf = stream->curr_buf;
			stream->fps_stats.frm0_timestamp = rkcif_time_get_ns(cif_dev);
		} else if (stream->frame_phase & CIF_CSI_FRAME1_READY) {
			if (stream->next_buf)
				active_buf = stream->next_buf;
			stream->fps_stats.frm1_timestamp = rkcif_time_get_ns(cif_dev);
		}
		spin_unlock_irqrestore(&stream->fps_lock, flags);
	}

	rkcif_deal_readout_time(stream);

	if (!stream->is_line_wake_up) {
		ret = rkcif_assign_new_buffer_pingpong(stream,
						       RKCIF_YUV_ADDR_STATE_UPDATE,
						       mipi_id);
		if (ret && cif_dev->chip_id < CHIP_RK3588_CIF)
			return;
		stream->last_frame_idx = stream->frame_idx;
	} else {
		ret = rkcif_update_new_buffer_wake_up_mode(stream);
		if (ret && cif_dev->chip_id < CHIP_RK3588_CIF)
			return;
	}
	if (cif_dev->chip_id < CHIP_RK3588_CIF &&
	    cif_dev->active_sensor->mbus.type == V4L2_MBUS_BT656 &&
	    stream->id != 0)
		stream->frame_idx++;
	if (!stream->is_line_wake_up)
		rkcif_buf_done_prepare(stream, active_buf, mipi_id, 0);

	if (cif_dev->chip_id == CHIP_RV1126_CIF ||
	    cif_dev->chip_id == CHIP_RV1126_CIF_LITE ||
	    cif_dev->chip_id == CHIP_RK3568_CIF)
		rkcif_luma_isr(&cif_dev->luma_vdev, mipi_id, cif_dev->stream[0].frame_idx - 1);
}

static void rkcif_update_stream_toisp(struct rkcif_device *cif_dev,
				      struct rkcif_stream *stream,
				      int mipi_id)
{
	if (stream->frame_phase == (CIF_CSI_FRAME0_READY | CIF_CSI_FRAME1_READY)) {

		v4l2_err(&cif_dev->v4l2_dev, "stream[%d], frm0/frm1 end simultaneously,frm id:%d\n",
			 stream->id, stream->frame_idx);
		return;
	}

	spin_lock(&stream->fps_lock);
	if (stream->frame_phase & CIF_CSI_FRAME0_READY)
		stream->fps_stats.frm0_timestamp = rkcif_time_get_ns(cif_dev);
	else if (stream->frame_phase & CIF_CSI_FRAME1_READY)
		stream->fps_stats.frm1_timestamp = rkcif_time_get_ns(cif_dev);
	spin_unlock(&stream->fps_lock);

	if (cif_dev->inf_id == RKCIF_MIPI_LVDS)
		rkcif_deal_readout_time(stream);

	if (!stream->is_line_wake_up)
		rkcif_assign_new_buffer_pingpong_toisp(stream,
						       RKCIF_YUV_ADDR_STATE_UPDATE,
						       mipi_id);
}

static void rkcif_update_stream_rockit(struct rkcif_device *cif_dev,
				struct rkcif_stream *stream,
				int mipi_id)
{
	struct rkcif_buffer *active_buf = NULL;
	unsigned long flags;

	if (stream->frame_phase == (CIF_CSI_FRAME0_READY | CIF_CSI_FRAME1_READY)) {

		v4l2_err(&cif_dev->v4l2_dev, "stream[%d], frm0/frm1 end simultaneously,frm id:%d\n",
			 stream->id, stream->frame_idx);
		return;
	}
	if (!stream->is_line_wake_up) {

		spin_lock_irqsave(&stream->fps_lock, flags);
		if (stream->frame_phase & CIF_CSI_FRAME0_READY) {
			if (stream->curr_buf_rockit)
				active_buf = stream->curr_buf_rockit;
			stream->fps_stats.frm0_timestamp = rkcif_time_get_ns(cif_dev);
		} else if (stream->frame_phase & CIF_CSI_FRAME1_READY) {
			if (stream->next_buf_rockit)
				active_buf = stream->next_buf_rockit;
			stream->fps_stats.frm1_timestamp = rkcif_time_get_ns(cif_dev);
		}
		spin_unlock_irqrestore(&stream->fps_lock, flags);
	}

	if (cif_dev->inf_id == RKCIF_MIPI_LVDS)
		rkcif_deal_readout_time(stream);

	rkcif_assign_new_buffer_pingpong_rockit(stream,
						RKCIF_YUV_ADDR_STATE_UPDATE,
						mipi_id);

	if (active_buf) {
		active_buf->vb.vb2_buf.timestamp = stream->readout.fs_timestamp;
		active_buf->vb.sequence = stream->frame_idx - 1;
		rkcif_rockit_buf_done(stream, active_buf);
	}
}

static u32 rkcif_get_sof(struct rkcif_device *cif_dev)
{
	u32 val = 0x0;
	struct rkcif_sensor_info *sensor = cif_dev->active_sensor;
	struct csi2_dev *csi;

	if (sensor->mbus.type == V4L2_MBUS_CSI2_DPHY ||
	    sensor->mbus.type == V4L2_MBUS_CSI2_CPHY ||
	    (sensor->mbus.type == V4L2_MBUS_CCP2 &&
	     cif_dev->chip_id >= CHIP_RV1106_CIF)) {
		csi = container_of(sensor->sd, struct csi2_dev, sd);
		val = rkcif_csi2_get_sof(csi);
	} else if (sensor->mbus.type == V4L2_MBUS_CCP2) {
		val = rkcif_lvds_get_sof(cif_dev);
	} else if (sensor->mbus.type == V4L2_MBUS_PARALLEL ||
		 sensor->mbus.type == V4L2_MBUS_BT656) {
		val = rkcif_dvp_get_sof(cif_dev);
	}
	return val;
}

void rkcif_set_sof(struct rkcif_device *cif_dev, u32 seq)
{
	struct rkcif_sensor_info *sensor = cif_dev->active_sensor;
	struct csi2_dev *csi;

	if (sensor->mbus.type == V4L2_MBUS_CSI2_DPHY ||
	    sensor->mbus.type == V4L2_MBUS_CSI2_CPHY ||
	    (sensor->mbus.type == V4L2_MBUS_CCP2 &&
	     cif_dev->chip_id >= CHIP_RV1106_CIF)) {
		csi = container_of(sensor->sd, struct csi2_dev, sd);
		rkcif_csi2_set_sof(csi, seq);
	} else if (sensor->mbus.type == V4L2_MBUS_CCP2) {
		rkcif_lvds_set_sof(cif_dev, seq);
	} else if (sensor->mbus.type == V4L2_MBUS_PARALLEL ||
		   sensor->mbus.type == V4L2_MBUS_BT656) {
		rkcif_dvp_set_sof(cif_dev, seq);
	}
}

static void rkcif_toisp_set_stream(struct rkcif_device *dev, int on)
{
	struct v4l2_subdev *sd = get_rkisp_sd(dev->sditf[0]);

	if (sd)
		v4l2_subdev_call(sd, core, ioctl,
				 RKISP_VICAP_CMD_SET_STREAM, &on);
}

static int rkcif_streamoff_in_reset(struct rkcif_device *cif_dev,
				 struct rkcif_stream *resume_stream[],
				 struct rkcif_resume_info *resume_info,
				 enum rkmodule_reset_src reset_src)
{
	struct rkcif_stream *stream = NULL;
	struct rkcif_pipeline *p = &cif_dev->pipe;
	struct rkcif_sensor_info *terminal_sensor = &cif_dev->terminal_sensor;
	struct sditf_priv *priv = cif_dev->sditf[0];
	u32 on, sof_cnt;
	int i, j, ret = 0;

	for (i = 0, j = 0; i < RKCIF_MAX_STREAM_MIPI; i++) {
		stream = &cif_dev->stream[i];

		if (stream->state == RKCIF_STATE_STREAMING) {

			v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev,
				 "stream[%d] stopping\n", stream->id);

			rkcif_stream_stop(stream);

			if (stream->id == RKCIF_STREAM_MIPI_ID0) {
				sof_cnt = rkcif_get_sof(cif_dev);
				v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev,
					 "%s: stream[%d] sync frmid & csi_sof, frm_id:%d, csi_sof:%d\n",
					 __func__,
					 stream->id,
					 stream->frame_idx,
					 sof_cnt);

				resume_info->frm_sync_seq = stream->frame_idx;
			}

			stream->state = RKCIF_STATE_RESET_IN_STREAMING;
			resume_stream[j] = stream;
			j += 1;

			v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev,
				 "%s stop stream[%d] in streaming, frm_id:%d, csi_sof:%d\n",
				 __func__, stream->id, stream->frame_idx, rkcif_get_sof(cif_dev));

		}
	}

	on = 0;
	for (i = 0; i < p->num_subdevs; i++) {

		if (p->subdevs[i] == terminal_sensor->sd) {

			if (reset_src != RKCIF_RESET_SRC_ERR_APP) {
				ret = v4l2_subdev_call(p->subdevs[i], core, ioctl,
						       RKMODULE_SET_QUICK_STREAM, &on);
				if (ret)
					v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev,
						 "quick stream off subdev:%s failed\n",
						 p->subdevs[i]->name);
			}
		} else {
			ret = v4l2_subdev_call(p->subdevs[i], video, s_stream, on);
		}
		if (ret)
			v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev,
				 "%s:stream %s subdev:%s failed\n",
				 __func__, on ? "on" : "off", p->subdevs[i]->name);
	}

	if (priv && cif_dev->sditf_cnt > 1) {
		if (priv->is_combine_mode) {
			for (i = 0; i < cif_dev->sditf_cnt; i++) {
				if (cif_dev->sditf[i] && cif_dev->sditf[i]->sensor_sd)
					ret = v4l2_subdev_call(cif_dev->sditf[i]->sensor_sd, core, ioctl,
							       RKMODULE_SET_QUICK_STREAM, &on);
			}
		} else if (cif_dev->is_camera_over_bridge) {
			for (i = 0; i < cif_dev->sditf_cnt; i++) {
				if (cif_dev->sditf[i] && cif_dev->sditf[i]->sensor_sd &&
				    (cif_dev->stream[i].state == RKCIF_STATE_STREAMING ||
				     cif_dev->stream[i].state == RKCIF_STATE_RESET_IN_STREAMING))
					ret = v4l2_subdev_call(cif_dev->sditf[i]->sensor_sd, core, ioctl,
							       RKMODULE_SET_QUICK_STREAM, &on);
			}
		}
	}
	return ret;
}

static int rkcif_streamon_in_reset(struct rkcif_device *cif_dev,
				 struct rkcif_stream *resume_stream[],
				 struct rkcif_resume_info *resume_info,
				 enum rkmodule_reset_src reset_src)
{
	struct rkcif_stream *stream = NULL;
	struct rkcif_pipeline *p = &cif_dev->pipe;
	struct rkcif_sensor_info *terminal_sensor = &cif_dev->terminal_sensor;
	struct sditf_priv *priv = cif_dev->sditf[0];
	int i = 0;
	int ret = 0;
	int on = 0;
	int capture_mode = 0;

	for (i = 0; i < RKCIF_MAX_STREAM_MIPI; i++) {
		stream = resume_stream[i];
		if (stream == NULL || stream->state != RKCIF_STATE_RESET_IN_STREAMING)
			break;
		stream->fs_cnt_in_single_frame = 0;
		if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT ||
		    rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO) {
			if (stream->curr_buf == stream->next_buf) {
				if (stream->curr_buf)
					list_add_tail(&stream->curr_buf->queue, &stream->buf_head);
			} else {
				if (stream->curr_buf)
					list_add_tail(&stream->curr_buf->queue, &stream->buf_head);
				if (stream->next_buf)
					list_add_tail(&stream->next_buf->queue, &stream->buf_head);
			}
			stream->curr_buf = NULL;
			stream->next_buf = NULL;
		}
		if (!cif_dev->sditf[0] || cif_dev->sditf[0]->mode.rdbk_mode == RKISP_VICAP_RDBK_AIQ)
			capture_mode = RKCIF_STREAM_MODE_CAPTURE;
		else {
			if (cif_dev->sditf[0]->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ)
				capture_mode = RKCIF_STREAM_MODE_TOISP;
			else
				capture_mode = RKCIF_STREAM_MODE_TOISP_RDBK;
		}
		stream->is_fs_fe_not_paired = false;
		stream->fs_cnt_in_single_frame = 0;
		if (cif_dev->active_sensor  &&
		    (cif_dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_DPHY ||
		    cif_dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_CPHY ||
		    cif_dev->active_sensor->mbus.type == V4L2_MBUS_CCP2))
			ret = rkcif_csi_stream_start(stream, capture_mode);
		else
			ret = rkcif_stream_start(stream, capture_mode);
		if (ret) {
			v4l2_err(&cif_dev->v4l2_dev, "%s:resume stream[%d] failed\n",
				 __func__, stream->id);
			return ret;
		}

		v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev,
			 "resume stream[%d], frm_idx:%d, csi_sof:%d\n",
			 stream->id, stream->frame_idx,
			 rkcif_get_sof(cif_dev));
	}

	on = 1;
	for (i = 0; i < p->num_subdevs; i++) {

		if (p->subdevs[i] == terminal_sensor->sd) {

			rkcif_set_sof(cif_dev, resume_info->frm_sync_seq);

			if (reset_src != RKCIF_RESET_SRC_ERR_APP) {
				ret = v4l2_subdev_call(p->subdevs[i], core, ioctl,
						       RKMODULE_SET_QUICK_STREAM, &on);
				if (ret)
					v4l2_err(&cif_dev->v4l2_dev,
						 "quick stream on subdev:%s failed\n",
						 p->subdevs[i]->name);
			}
		} else {
			if (p->subdevs[i] == terminal_sensor->sd)
				rkcif_set_sof(cif_dev, resume_info->frm_sync_seq);

			ret = v4l2_subdev_call(p->subdevs[i], video, s_stream, on);
		}

		if (ret)
			v4l2_err(&cif_dev->v4l2_dev, "reset subdev:%s failed\n",
				 p->subdevs[i]->name);
	}

	if (priv && cif_dev->sditf_cnt > 1) {
		if (priv->is_combine_mode) {
			for (i = 0; i < cif_dev->sditf_cnt; i++) {
				if (cif_dev->sditf[i] && cif_dev->sditf[i]->sensor_sd)
					ret = v4l2_subdev_call(cif_dev->sditf[i]->sensor_sd, core, ioctl,
							       RKMODULE_SET_QUICK_STREAM, &on);
			}
		} else if (cif_dev->is_camera_over_bridge) {
			for (i = 0; i < cif_dev->sditf_cnt; i++) {
				if (cif_dev->sditf[i] && cif_dev->sditf[i]->sensor_sd &&
				    (cif_dev->stream[i].state == RKCIF_STATE_STREAMING ||
				     cif_dev->stream[i].state == RKCIF_STATE_RESET_IN_STREAMING))
					ret = v4l2_subdev_call(cif_dev->sditf[i]->sensor_sd, core, ioctl,
							       RKMODULE_SET_QUICK_STREAM, &on);
			}
		}
	}
	return ret;
}

static int rkcif_do_reset_work_below_rk3588(struct rkcif_device *cif_dev,
			       enum rkmodule_reset_src reset_src)
{
	struct rkcif_stream *resume_stream[2][RKCIF_MAX_STREAM_MIPI] = {0};
	struct rkcif_resume_info *resume_info[2];
	struct rkcif_timer *timer = &cif_dev->reset_watchdog_timer;
	struct rkcif_hw *hw = cif_dev->hw_dev;
	struct rkcif_device *cifdev = NULL;
	int i, ret = 0;

	mutex_lock(&hw->dev_lock);
	if (cif_dev->reset_work_cancel) {
		ret = 0;
		goto unlock_stream;
	}

	hw->is_in_reset = true;
	v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev, "do rkcif reset\n");

	for (i = 0; i < hw->dev_num; i++) {
		cifdev = hw->cif_dev[i];
		resume_info[i] = &cifdev->reset_work.resume_info;
		ret |= rkcif_streamoff_in_reset(cifdev,
						resume_stream[i],
						resume_info[i],
						reset_src);
	}

	rkcif_do_cru_reset(cif_dev);

	for (i = 0; i < hw->dev_num; i++) {
		cifdev = hw->cif_dev[i];
		resume_info[i] = &cifdev->reset_work.resume_info;
		ret |= rkcif_streamon_in_reset(cifdev,
					       resume_stream[i],
					       resume_info[i],
					       reset_src);
	}
	rkcif_start_luma(&cif_dev->luma_vdev,
			 cif_dev->stream[RKCIF_STREAM_MIPI_ID0].cif_fmt_in);

	timer->csi2_err_triggered_cnt = 0;
	for (i = 0; i < hw->dev_num; i++)
		rkcif_monitor_reset_event(cif_dev);

	v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev, "do rkcif reset successfully!\n");
	hw->is_in_reset = false;
	mutex_unlock(&hw->dev_lock);
	return 0;

unlock_stream:
	mutex_unlock(&hw->dev_lock);
	return ret;
}

static int rkcif_do_reset_work(struct rkcif_device *cif_dev,
			       enum rkmodule_reset_src reset_src)
{
	struct rkcif_stream *resume_stream[RKCIF_MAX_STREAM_MIPI] = { NULL };
	struct rkcif_resume_info *resume_info = &cif_dev->reset_work.resume_info;
	struct rkcif_timer *timer = &cif_dev->reset_watchdog_timer;
	struct sditf_priv *priv = cif_dev->sditf[0];
	int ret = 0;

	if (cif_dev->chip_id < CHIP_RK3588_CIF)
		return rkcif_do_reset_work_below_rk3588(cif_dev, reset_src);

	mutex_lock(&cif_dev->stream_lock);
	if (cif_dev->reset_work_cancel) {
		ret = 0;
		goto unlock_stream;
	}
	v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev, "do rkcif reset\n");

	rkcif_streamoff_in_reset(cif_dev,
				 resume_stream,
				 resume_info,
				 reset_src);

	rkcif_do_soft_reset(cif_dev);

	if (priv && priv->mode.rdbk_mode == RKISP_VICAP_ONLINE)
		rkcif_toisp_set_stream(cif_dev, 1);

	rkcif_streamon_in_reset(cif_dev,
				resume_stream,
				resume_info,
				reset_src);

	timer->csi2_err_triggered_cnt = 0;
	rkcif_monitor_reset_event(cif_dev);

	v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev, "do rkcif reset successfully!\n");
	mutex_unlock(&cif_dev->stream_lock);
	return 0;

unlock_stream:
	mutex_unlock(&cif_dev->stream_lock);
	return ret;
}

void rkcif_reset_work(struct work_struct *work)
{
	struct rkcif_work_struct *reset_work = container_of(work,
							    struct rkcif_work_struct,
							    work);
	struct rkcif_device *dev = container_of(reset_work,
						struct rkcif_device,
						reset_work);
	int ret;

	ret = rkcif_do_reset_work(dev, reset_work->reset_src);
	if (ret)
		v4l2_info(&dev->v4l2_dev, "do reset work failed!\n");
}

static bool rkcif_is_reduced_frame_rate(struct rkcif_device *dev)
{
	struct rkcif_timer *timer = &dev->reset_watchdog_timer;
	struct rkcif_stream *stream = &dev->stream[RKCIF_STREAM_MIPI_ID0];
	struct v4l2_rect *raw_rect = &dev->terminal_sensor.raw_rect;
	u64 fps, timestamp0, timestamp1, diff_time;
	unsigned long fps_flags;
	unsigned int deviation = 1;
	bool is_reduced = false;
	u64 cur_time = 0;
	u64 time_distance = 0;

	spin_lock_irqsave(&stream->fps_lock, fps_flags);
	timestamp0 = stream->fps_stats.frm0_timestamp;
	timestamp1 = stream->fps_stats.frm1_timestamp;
	spin_unlock_irqrestore(&stream->fps_lock, fps_flags);

	fps = timestamp0 > timestamp1 ?
	      timestamp0 - timestamp1 : timestamp1 - timestamp0;
	fps =  div_u64(fps, 1000);
	diff_time = fps > timer->frame_end_cycle_us ?
		    fps - timer->frame_end_cycle_us : 0;
	deviation = DIV_ROUND_UP(timer->vts, 100);

	v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev, "diff_time:%lld,devi_t:%ld,devi_h:%d\n",
		  diff_time, timer->line_end_cycle * deviation, deviation);

	cur_time = rkcif_time_get_ns(dev);
	time_distance = timestamp0 > timestamp1 ?
			cur_time - timestamp0 : cur_time - timestamp1;
	time_distance = div_u64(time_distance, 1000);
	if (time_distance > fps * 2)
		return false;

	if (diff_time > timer->line_end_cycle * deviation) {
		s32 vblank = 0;
		unsigned int vts;

		is_reduced = true;
		vblank = rkcif_get_sensor_vblank(dev);
		vts = vblank + timer->raw_height;

		v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev, "old vts:%d,new vts:%d\n", timer->vts, vts);

		v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev,
			  "reduce frame rate,vblank:%d, height(raw output):%d, fps:%lld, frm_end_t:%ld, line_t:%ld, diff:%lld\n",
			  rkcif_get_sensor_vblank(dev),
			  raw_rect->height,
			  fps,
			  timer->frame_end_cycle_us,
			  timer->line_end_cycle,
			  diff_time);

		timer->vts = vts;
		timer->frame_end_cycle_us = fps;
		timer->line_end_cycle = div_u64(timer->frame_end_cycle_us, timer->vts);
	} else {
		is_reduced = false;
	}

	timer->frame_end_cycle_us = fps;

	fps = div_u64(fps, 1000);
	fps = fps * timer->frm_num_of_monitor_cycle;
	timer->cycle = msecs_to_jiffies(fps);
	timer->timer.expires = jiffies + timer->cycle;

	return is_reduced;

}

static void rkcif_dvp_event_reset_pipe(struct rkcif_device *dev, int reset_src)
{
	struct rkcif_dvp_sof_subdev *subdev = &dev->dvp_sof_subdev;

	if (subdev) {
		struct v4l2_event event = {
			.type = V4L2_EVENT_RESET_DEV,
			.reserved[0] = reset_src,
		};
		v4l2_event_queue(subdev->sd.devnode, &event);
	}
}

static void rkcif_lvds_event_reset_pipe(struct rkcif_device *dev, int reset_src)
{
	struct rkcif_lvds_subdev *subdev = &dev->lvds_subdev;

	if (subdev) {
		struct v4l2_event event = {
			.type = V4L2_EVENT_RESET_DEV,
			.reserved[0] = reset_src,
		};
		v4l2_event_queue(subdev->sd.devnode, &event);
	}
}

static void rkcif_send_reset_event(struct rkcif_device *cif_dev, int reset_src)
{
	struct v4l2_mbus_config *mbus = &cif_dev->active_sensor->mbus;
	struct csi2_dev *csi;

	if (mbus->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus->type == V4L2_MBUS_CSI2_CPHY) {
		csi = container_of(cif_dev->active_sensor->sd, struct csi2_dev, sd);
		rkcif_csi2_event_reset_pipe(csi, reset_src);
	} else if (mbus->type == V4L2_MBUS_CCP2) {
		rkcif_lvds_event_reset_pipe(cif_dev, reset_src);
	} else {
		rkcif_dvp_event_reset_pipe(cif_dev, reset_src);
	}
	v4l2_dbg(3, rkcif_debug, &cif_dev->v4l2_dev,
		 "send reset event,bus type 0x%x\n",
		 mbus->type);
}

static void rkcif_init_reset_work(struct rkcif_timer *timer)
{
	struct rkcif_device *dev = container_of(timer,
						struct rkcif_device,
						reset_watchdog_timer);
	struct rkcif_stream *stream = NULL;
	unsigned long flags;
	int i = 0;

	v4l2_info(&dev->v4l2_dev,
		  "do reset work schedule, run_cnt:%d, reset source:%d\n",
		  timer->run_cnt, timer->reset_src);

	spin_lock_irqsave(&timer->timer_lock, flags);
	timer->is_running = false;
	timer->is_triggered = false;
	timer->csi2_err_cnt_odd = 0;
	timer->csi2_err_cnt_even = 0;
	timer->csi2_err_fs_fe_cnt = 0;
	timer->notifer_called_cnt = 0;
	dev->is_toisp_reset = false;
	for (i = 0; i < dev->num_channels; i++) {
		stream = &dev->stream[i];
		if (stream->state == RKCIF_STATE_STREAMING)
			timer->last_buf_wakeup_cnt[stream->id] = stream->buf_wake_up_cnt;
	}
	spin_unlock_irqrestore(&timer->timer_lock, flags);
	if (dev->chip_id >= CHIP_RK3588_CIF) {
		if (timer->is_ctrl_by_user) {
			rkcif_send_reset_event(dev, timer->reset_src);
		} else {
			dev->reset_work.reset_src = timer->reset_src;
			if (!schedule_work(&dev->reset_work.work))
				v4l2_info(&dev->v4l2_dev,
					  "schedule reset work failed\n");
		}
	} else {
		if (!dev->hw_dev->is_in_reset) {
			if (timer->is_ctrl_by_user) {
				rkcif_send_reset_event(dev, timer->reset_src);
			} else {
				dev->reset_work.reset_src = timer->reset_src;
				if (!schedule_work(&dev->reset_work.work))
					v4l2_info(&dev->v4l2_dev,
						  "schedule reset work failed\n");
			}
		}
	}
}

static int rkcif_detect_reset_event(struct rkcif_stream *stream,
				    struct rkcif_timer *timer,
				    int check_cnt,
				    bool *is_mod_timer)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_sensor_info *terminal_sensor = &dev->terminal_sensor;
	unsigned long flags;
	int ret, is_reset = 0;
	struct rkmodule_vicap_reset_info rst_info;

	if (dev->is_toisp_reset) {
		is_reset = 1;
		timer->reset_src = RKCIF_RESET_SRC_ERR_ISP;
	}
	if (is_reset) {
		rkcif_init_reset_work(timer);
		return is_reset;
	}

	if (timer->last_buf_wakeup_cnt[stream->id] < stream->buf_wake_up_cnt &&
	    check_cnt == 0) {

		v4l2_dbg(1, rkcif_debug, &dev->v4l2_dev,
			 "info: frame end still update(%d, %d) in detecting cnt:%d, mode:%d\n",
			  timer->last_buf_wakeup_cnt[stream->id], stream->frame_idx,
			  timer->run_cnt, timer->monitor_mode);

		timer->last_buf_wakeup_cnt[stream->id] = stream->buf_wake_up_cnt;

		if (stream->frame_idx > 2)
			rkcif_is_reduced_frame_rate(dev);

		if (timer->monitor_mode == RKCIF_MONITOR_MODE_HOTPLUG) {
			ret = v4l2_subdev_call(terminal_sensor->sd,
					       core, ioctl,
					       RKMODULE_GET_VICAP_RST_INFO,
					       &rst_info);
			if (ret)
				is_reset = 0;
			else
				is_reset = rst_info.is_reset;
			rst_info.is_reset = 0;
			if (is_reset)
				timer->reset_src = RKCIF_RESET_SRC_ERR_HOTPLUG;
			v4l2_subdev_call(terminal_sensor->sd, core, ioctl,
					 RKMODULE_SET_VICAP_RST_INFO, &rst_info);
			if (!is_reset) {
				is_reset = rkcif_is_csi2_err_trigger_reset(timer);
				if (is_reset)
					timer->reset_src = RKCIF_RESET_SRC_ERR_CSI2;
			}
		} else if (timer->monitor_mode == RKCIF_MONITOR_MODE_CONTINUE) {
			is_reset = rkcif_is_csi2_err_trigger_reset(timer);
		} else if (timer->monitor_mode == RKCIF_MONITOR_MODE_TRIGGER) {
			is_reset = timer->is_csi2_err_occurred;
			if (is_reset)
				timer->reset_src = RKCIF_RESET_SRC_ERR_CSI2;
			timer->is_csi2_err_occurred = false;
		}

		if (is_reset) {
			rkcif_init_reset_work(timer);
			return is_reset;
		}

		if (timer->monitor_mode == RKCIF_MONITOR_MODE_CONTINUE ||
		    timer->monitor_mode == RKCIF_MONITOR_MODE_HOTPLUG) {
			if (timer->run_cnt == timer->max_run_cnt)
				timer->run_cnt = 0x0;
			*is_mod_timer = true;
		} else {
			if (timer->run_cnt <= timer->max_run_cnt) {
				*is_mod_timer = true;
			} else {
				spin_lock_irqsave(&timer->timer_lock, flags);
				timer->is_triggered = false;
				timer->is_running = false;
				spin_unlock_irqrestore(&timer->timer_lock, flags);
				v4l2_info(&dev->v4l2_dev, "stop reset detecting!\n");
			}
		}
	} else if (timer->last_buf_wakeup_cnt[stream->id] == stream->buf_wake_up_cnt) {

		bool is_reduced = false;

		if (stream->frame_idx > 2)
			is_reduced = rkcif_is_reduced_frame_rate(dev);
		else if (timer->run_cnt < 20)
			is_reduced = true;

		if (is_reduced) {
			*is_mod_timer = true;
			v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev, "%s fps is reduced\n", __func__);
		} else {

			v4l2_info(&dev->v4l2_dev,
				  "do reset work due to frame end is stopped, run_cnt:%d\n",
				  timer->run_cnt);

			timer->reset_src = RKICF_RESET_SRC_ERR_CUTOFF;
			rkcif_init_reset_work(timer);
			is_reset = true;
		}
	}

	return is_reset;

}

void rkcif_reset_watchdog_timer_handler(struct timer_list *t)
{
	struct rkcif_timer *timer = container_of(t, struct rkcif_timer, timer);
	struct rkcif_device *dev = container_of(timer,
						struct rkcif_device,
						reset_watchdog_timer);
	struct rkcif_stream *stream = NULL;
	unsigned long flags;
	unsigned int i;
	int is_reset = 0;
	int check_cnt = 0;
	bool is_mod_timer = false;

	for (i = 0; i < dev->num_channels; i++) {
		stream = &dev->stream[i];
		if (stream->state == RKCIF_STATE_STREAMING) {
			is_reset = rkcif_detect_reset_event(stream,
							    timer,
							    check_cnt,
							    &is_mod_timer);
			check_cnt++;
			if (is_reset)
				break;
		}
	}
	if (!is_reset && is_mod_timer)
		mod_timer(&timer->timer, jiffies + timer->cycle);

	timer->run_cnt += 1;

	if (!check_cnt) {
		spin_lock_irqsave(&timer->timer_lock, flags);
		timer->is_triggered = false;
		timer->is_running = false;
		spin_unlock_irqrestore(&timer->timer_lock, flags);

		v4l2_info(&dev->v4l2_dev,
			  "all stream is stopped, stop reset detect!\n");
	}
}

int rkcif_reset_notifier(struct notifier_block *nb,
			 unsigned long action, void *data)
{
	struct rkcif_hw *hw = container_of(nb, struct rkcif_hw, reset_notifier);
	struct rkcif_device *dev = NULL;
	struct rkcif_timer *timer = NULL;
	unsigned long flags, val;
	u32 *csi_idx = data;
	int i = 0;
	bool is_match_dev = false;

	for (i = 0; i < hw->dev_num; i++) {
		dev = hw->cif_dev[i];
		if (*csi_idx == dev->csi_host_idx) {
			is_match_dev = true;
			break;
		}
	}
	if (!is_match_dev)
		return -EINVAL;
	timer = &dev->reset_watchdog_timer;
	if (timer->is_running) {
		val = action & CSI2_ERR_COUNT_ALL_MASK;
		spin_lock_irqsave(&timer->csi2_err_lock, flags);
		if ((val % timer->csi2_err_ref_cnt) == 0) {
			timer->notifer_called_cnt++;
			if ((timer->notifer_called_cnt % 2) == 0)
				timer->csi2_err_cnt_even = val;
			else
				timer->csi2_err_cnt_odd = val;
		}

		timer->csi2_err_fs_fe_cnt = (action & CSI2_ERR_FSFE_MASK) >> 8;
		spin_unlock_irqrestore(&timer->csi2_err_lock, flags);
	}

	return 0;
}

void rkcif_modify_line_int(struct rkcif_stream *stream, bool en)
{
	struct rkcif_device *cif_dev = stream->cifdev;
	u32 line_intr_en = 0;

	if (cif_dev->chip_id >= CHIP_RK3588_CIF)
		line_intr_en = CSI_LINE_INTEN_RK3588(stream->id);
	else
		line_intr_en = CSI_LINE_INTEN(stream->id);
	if (en) {
		if (cif_dev->wait_line_bak != cif_dev->wait_line) {
			cif_dev->wait_line_bak = cif_dev->wait_line;
			rkcif_write_register(cif_dev, CIF_REG_MIPI_LVDS_LINE_INT_NUM_ID0_1,
					     cif_dev->wait_line << 16 | cif_dev->wait_line);
			rkcif_write_register(cif_dev, CIF_REG_MIPI_LVDS_LINE_INT_NUM_ID2_3,
					     cif_dev->wait_line << 16 | cif_dev->wait_line);
		}
		rkcif_write_register_or(cif_dev, CIF_REG_MIPI_LVDS_INTEN,
					line_intr_en);
		stream->is_line_inten = true;
	} else {
		rkcif_write_register_and(cif_dev, CIF_REG_MIPI_LVDS_INTEN,
					 ~line_intr_en);
	}
}

static void rkcif_detect_wake_up_mode_change(struct rkcif_stream *stream)
{
	struct rkcif_device *cif_dev = stream->cifdev;
	struct sditf_priv *priv = cif_dev->sditf[0];
	bool is_change = false;
	int ch = 0;
	int i = 0;

	if (!priv || priv->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ)
		return;

	if ((cif_dev->hdr.hdr_mode == NO_HDR || cif_dev->hdr.hdr_mode == HDR_COMPR) &&
	    stream->id == RKCIF_STREAM_MIPI_ID0) {
		if (cif_dev->wait_line != cif_dev->wait_line_cache)
			cif_dev->wait_line = cif_dev->wait_line_cache;
	} else if (cif_dev->hdr.hdr_mode == HDR_X2 && stream->id == RKCIF_STREAM_MIPI_ID1) {
		if (cif_dev->wait_line != cif_dev->wait_line_cache)
			cif_dev->wait_line = cif_dev->wait_line_cache;
	} else if (cif_dev->hdr.hdr_mode == HDR_X3 && stream->id == RKCIF_STREAM_MIPI_ID2) {
		if (cif_dev->wait_line != cif_dev->wait_line_cache)
			cif_dev->wait_line = cif_dev->wait_line_cache;
	}

	if (cif_dev->wait_line && (!stream->is_line_wake_up)) {
		is_change = true;
		stream->is_line_wake_up = true;
		if (stream->frame_phase == CIF_CSI_FRAME0_READY)
			stream->line_int_cnt = 1;
		else if (stream->frame_phase == CIF_CSI_FRAME1_READY)
			stream->line_int_cnt = 0;
		if (cif_dev->hdr.hdr_mode == HDR_X2) {
			cif_dev->stream[0].is_line_wake_up = true;
			cif_dev->stream[0].line_int_cnt = stream->line_int_cnt;
		} else if (cif_dev->hdr.hdr_mode == HDR_X3) {
			cif_dev->stream[0].is_line_wake_up = true;
			cif_dev->stream[1].is_line_wake_up = true;
			cif_dev->stream[0].line_int_cnt = stream->line_int_cnt;
			cif_dev->stream[1].line_int_cnt = stream->line_int_cnt;
		}
	} else if ((cif_dev->wait_line == 0) && stream->is_line_wake_up) {
		stream->is_line_wake_up = false;
	}
	if (stream->is_line_wake_up) {
		if (is_change) {
			if (cif_dev->hdr.hdr_mode == HDR_X2)
				ch = 2;
			else if (cif_dev->hdr.hdr_mode == HDR_X3)
				ch = 3;
			else
				ch = 1;
			for (i = 0; i < ch; i++)
				rkcif_modify_line_int(&cif_dev->stream[i], true);
		} else {
			rkcif_modify_line_int(stream, true);
		}
	}
}

u32 rkcif_mbus_pixelcode_to_v4l2(u32 pixelcode)
{
	u32 pixelformat;

	switch (pixelcode) {
	case MEDIA_BUS_FMT_Y8_1X8:
		pixelformat = V4L2_PIX_FMT_GREY;
		break;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
		pixelformat = V4L2_PIX_FMT_SBGGR8;
		break;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
		pixelformat = V4L2_PIX_FMT_SGBRG8;
		break;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
		pixelformat = V4L2_PIX_FMT_SGRBG8;
		break;
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		pixelformat = V4L2_PIX_FMT_SRGGB8;
		break;
	case MEDIA_BUS_FMT_Y10_1X10:
		pixelformat = V4L2_PIX_FMT_Y10;
		break;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		pixelformat = V4L2_PIX_FMT_SBGGR10;
		break;
	case MEDIA_BUS_FMT_SGBRG10_1X10:
		pixelformat = V4L2_PIX_FMT_SGBRG10;
		break;
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		pixelformat = V4L2_PIX_FMT_SGRBG10;
		break;
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		pixelformat = V4L2_PIX_FMT_SRGGB10;
		break;
	case MEDIA_BUS_FMT_Y12_1X12:
		pixelformat = V4L2_PIX_FMT_Y12;
		break;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		pixelformat = V4L2_PIX_FMT_SBGGR12;
		break;
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		pixelformat = V4L2_PIX_FMT_SGBRG12;
		break;
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		pixelformat = V4L2_PIX_FMT_SGRBG12;
		break;
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		pixelformat = V4L2_PIX_FMT_SRGGB12;
		break;
	case MEDIA_BUS_FMT_SPD_2X8:
		pixelformat = V4l2_PIX_FMT_SPD16;
		break;
	case MEDIA_BUS_FMT_EBD_1X8:
		pixelformat = V4l2_PIX_FMT_EBD8;
		break;
	case MEDIA_BUS_FMT_SBGGR16_1X16:
		pixelformat = V4L2_PIX_FMT_SBGGR16;
		break;
	case MEDIA_BUS_FMT_SGBRG16_1X16:
		pixelformat = V4L2_PIX_FMT_SGBRG16;
		break;
	case MEDIA_BUS_FMT_SGRBG16_1X16:
		pixelformat = V4L2_PIX_FMT_SGRBG16;
		break;
	case MEDIA_BUS_FMT_SRGGB16_1X16:
		pixelformat = V4L2_PIX_FMT_SRGGB16;
		break;
	default:
		pixelformat = V4L2_PIX_FMT_SRGGB10;
	}

	return pixelformat;
}

void rkcif_set_default_fmt(struct rkcif_device *cif_dev)
{
	struct v4l2_subdev_selection input_sel;
	struct v4l2_pix_format_mplane pixm;
	struct v4l2_subdev_format fmt;
	int stream_num = 0;
	int ret, i;

	if (cif_dev->chip_id < CHIP_RV1126_CIF)
		return;

	stream_num = RKCIF_MAX_STREAM_MIPI;

	if (!cif_dev->terminal_sensor.sd)
		rkcif_update_sensor_info(&cif_dev->stream[0]);

	if (cif_dev->terminal_sensor.sd) {
		for (i = 0; i < stream_num; i++) {
			memset(&fmt, 0, sizeof(fmt));
			fmt.pad = i;
			fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
			v4l2_subdev_call(cif_dev->terminal_sensor.sd, pad, get_fmt, NULL, &fmt);

			memset(&pixm, 0, sizeof(pixm));
			pixm.pixelformat = rkcif_mbus_pixelcode_to_v4l2(fmt.format.code);
			pixm.width = fmt.format.width;
			pixm.height = fmt.format.height;

			memset(&input_sel, 0, sizeof(input_sel));
			input_sel.pad = i;
			input_sel.target = V4L2_SEL_TGT_CROP_BOUNDS;
			input_sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
			ret = v4l2_subdev_call(cif_dev->terminal_sensor.sd,
					       pad, get_selection, NULL,
					       &input_sel);
			if (!ret) {
				pixm.width = input_sel.r.width;
				pixm.height = input_sel.r.height;
			}
			rkcif_set_fmt(&cif_dev->stream[i], &pixm, false);
		}
	}
}

void rkcif_enable_dma_capture(struct rkcif_stream *stream, bool is_only_enable)
{
	struct rkcif_device *cif_dev = stream->cifdev;
	struct v4l2_mbus_config *mbus_cfg = &cif_dev->active_sensor->mbus;
	struct csi_channel_info *channel = &cif_dev->channels[stream->id];
	u32 val = 0;
	u32 uncompact = 0;

	if (stream->buf_owner == RKCIF_DMAEN_BY_ISP)
		stream->buf_owner = RKCIF_DMAEN_BY_ISP_TO_VICAP;

	atomic_dec(&cif_dev->streamoff_cnt);
	if (stream->dma_en) {
		stream->dma_en |= stream->to_en_dma;
		stream->to_en_dma = 0;
		return;
	}

	stream->dma_en |= stream->to_en_dma;
	if (!is_only_enable) {
		if (stream->to_en_dma == RKCIF_DMAEN_BY_VICAP)
			rkcif_assign_new_buffer_pingpong(stream,
							 RKCIF_YUV_ADDR_STATE_INIT,
							 stream->id);
		else if (stream->to_en_dma == RKCIF_DMAEN_BY_ISP)
			rkcif_assign_new_buffer_pingpong_toisp(stream,
							       RKCIF_YUV_ADDR_STATE_INIT,
							       stream->id);
		rkcif_write_register(cif_dev, get_reg_index_of_frm0_y_vlw(stream->id),
				     channel->virtual_width);
	}
	if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY) {
		if (cif_dev->chip_id < CHIP_RK3562_CIF)
			rkcif_write_register_or(cif_dev, CIF_REG_MIPI_LVDS_CTRL, 0x00010000);
		else
			rkcif_write_register_or(cif_dev,  get_reg_index_of_frm0_y_vlw(stream->id), BIT(31));
	} else {
		if (cif_dev->chip_id < CHIP_RK3562_CIF)
			rkcif_write_register_or(cif_dev, CIF_REG_DVP_CTRL, 0x00010000);
		else
			rkcif_write_register_or(cif_dev, CIF_REG_DVP_VIR_LINE_WIDTH, BIT(28) << stream->id);
	}
	if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY) {
		val = rkcif_read_register(cif_dev, get_reg_index_of_id_ctrl0(stream->id));
		if (cif_dev->chip_id < CHIP_RK3576_CIF) {
			val |= CSI_DMA_ENABLE;
			uncompact = CSI_WRDDR_TYPE_RAW_UNCOMPACT;
		} else {
			val |= CSI_DMA_ENABLE_RK3576;
			uncompact = CSI_WRDDR_TYPE_RAW_UNCOMPACT << 3;
			if (!(rkcif_read_register(cif_dev, CIF_REG_MIPI_LVDS_INTEN) &
			      CSI_START_INTEN_RK3576(stream->id))) {
				rkcif_write_register(cif_dev, CIF_REG_MIPI_LVDS_INTSTAT,
						     CSI_START_INTSTAT_RK3576(stream->id));
				rkcif_write_register_or(cif_dev, CIF_REG_MIPI_LVDS_INTEN,
							CSI_START_INTEN_RK3576(stream->id));
			}
		}
		if (!stream->is_compact)
			val |= uncompact;
		else
			val &= ~uncompact;
		rkcif_write_register(cif_dev, get_reg_index_of_id_ctrl0(stream->id), val);
	} else if (mbus_cfg->type == V4L2_MBUS_CCP2) {
		val = rkcif_read_register(cif_dev, get_reg_index_of_lvds_id_ctrl0(stream->id));
		if (!stream->is_compact)
			val |= CSI_WRDDR_TYPE_RAW_UNCOMPACT << 17;
		else
			val &= ~(CSI_WRDDR_TYPE_RAW_UNCOMPACT << 17);
		val |= LVDS_DMAEN_RV1106;
		rkcif_write_register(cif_dev, get_reg_index_of_lvds_id_ctrl0(stream->id), val);
	} else {
		val = rkcif_read_register(cif_dev, CIF_REG_DVP_FOR);
		if (!stream->is_compact)
			val |= CSI_WRDDR_TYPE_RAW_UNCOMPACT << 11;
		else
			val &= ~(CSI_WRDDR_TYPE_RAW_UNCOMPACT << 11);
		rkcif_write_register(cif_dev, CIF_REG_DVP_FOR, val);
		val = rkcif_read_register(cif_dev, CIF_REG_DVP_CTRL);
		if (cif_dev->chip_id == CHIP_RK3588_CIF)
			val |= DVP_DMA_EN;
		else if (cif_dev->chip_id == CHIP_RV1106_CIF)
			val |= DVP_SW_DMA_EN(stream->id);
		rkcif_write_register(cif_dev, CIF_REG_DVP_CTRL, val);
	}

	stream->to_en_dma = 0;
}

static int rkcif_stop_dma_capture(struct rkcif_stream *stream)
{
	struct rkcif_device *cif_dev = stream->cifdev;
	struct v4l2_mbus_config *mbus_cfg = &cif_dev->active_sensor->mbus;
	u32 val = 0;

	if (stream->buf_replace_cnt)
		return -EINVAL;

	stream->dma_en &= ~stream->to_stop_dma;
	atomic_inc(&cif_dev->streamoff_cnt);
	if (stream->dma_en != 0) {
		if (stream->dma_en & RKCIF_DMAEN_BY_ISP)
			stream->buf_owner = RKCIF_DMAEN_BY_ISP;
		stream->to_stop_dma = 0;
		return 0;
	}

	if (mbus_cfg->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus_cfg->type == V4L2_MBUS_CSI2_CPHY) {
		val = rkcif_read_register(cif_dev, get_reg_index_of_id_ctrl0(stream->id));
		if (cif_dev->chip_id < CHIP_RK3576_CIF)
			val &= ~CSI_DMA_ENABLE;
		else
			val &= ~CSI_DMA_ENABLE_RK3576;
		if (stream->is_stop_capture) {
			val &= ~CSI_ENABLE_CAPTURE;
			stream->is_stop_capture = false;
		}
		rkcif_write_register(cif_dev, get_reg_index_of_id_ctrl0(stream->id), val);
	} else if (mbus_cfg->type == V4L2_MBUS_CCP2) {
		val = rkcif_read_register(cif_dev, get_reg_index_of_lvds_id_ctrl0(stream->id));
		val &= ~LVDS_DMAEN_RV1106;
		if (stream->is_stop_capture) {
			val &= ~ENABLE_CAPTURE;
			stream->is_stop_capture = false;
		}
		rkcif_write_register(cif_dev, get_reg_index_of_lvds_id_ctrl0(stream->id), val);
	}  else {
		val = rkcif_read_register(cif_dev, CIF_REG_DVP_CTRL);
		if (cif_dev->chip_id == CHIP_RK3588_CIF)
			val &= ~DVP_DMA_EN;
		else if (cif_dev->chip_id == CHIP_RV1106_CIF)
			val &= ~(DVP_SW_DMA_EN(stream->id));
		if (stream->is_stop_capture) {
			val &= ~ENABLE_CAPTURE;
			stream->is_stop_capture = false;
		}
		rkcif_write_register(cif_dev, CIF_REG_DVP_CTRL, val);
	}
	stream->to_stop_dma = 0;
	v4l2_dbg(4, rkcif_debug, &cif_dev->v4l2_dev,
		 "stream[%d] replace_cnt %d, y_addr 0x%x, 0x%x\n",
		 stream->id, stream->buf_replace_cnt,
		 rkcif_read_register(cif_dev, get_reg_index_of_frm0_y_addr(stream->id)),
		 rkcif_read_register(cif_dev, get_reg_index_of_frm1_y_addr(stream->id)));
	return 0;
}

static bool rkcif_check_frame_active(struct rkcif_device *cif_dev)
{
	if (cif_dev->sditf[0] &&
	    cif_dev->sditf[0]->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ &&
	    cif_dev->sditf[0]->is_toisp_off &&
	    cif_dev->sditf[0]->is_multi_online)
		return false;

	return true;
}

static void rkcif_send_sof(struct rkcif_device *cif_dev)
{
	struct v4l2_mbus_config *mbus = &cif_dev->active_sensor->mbus;
	struct csi2_dev *csi;

	if (!rkcif_check_frame_active(cif_dev))
		return;

	if (mbus->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus->type == V4L2_MBUS_CSI2_CPHY ||
	    (mbus->type == V4L2_MBUS_CCP2 &&
	     cif_dev->chip_id >= CHIP_RV1106_CIF)) {
		csi = container_of(cif_dev->active_sensor->sd, struct csi2_dev, sd);
		rkcif_csi2_event_inc_sof(csi);
	} else if (mbus->type == V4L2_MBUS_CCP2) {
		rkcif_lvds_event_inc_sof(cif_dev);
	} else {
		rkcif_dvp_event_inc_sof(cif_dev);
	}
	v4l2_dbg(3, rkcif_debug, &cif_dev->v4l2_dev, "send sof %d\n\n", rkcif_get_sof(cif_dev));
}

static int rkcif_g_toisp_ch(unsigned int intstat_glb, int index)
{
	if (intstat_glb & TOISP_END_CH0(index))
		return RKCIF_TOISP_CH0;
	if (intstat_glb & TOISP_END_CH1(index))
		return RKCIF_TOISP_CH1;
	if (intstat_glb & TOISP_END_CH2(index))
		return RKCIF_TOISP_CH2;

	return -EINVAL;
}

static int rkcif_g_toisp_fs(unsigned int intstat_glb, int index)
{
	if (intstat_glb & TOISP_FS_CH0(index))
		return RKCIF_TOISP_CH0;
	if (intstat_glb & TOISP_FS_CH1(index))
		return RKCIF_TOISP_CH1;
	if (intstat_glb & TOISP_FS_CH2(index))
		return RKCIF_TOISP_CH2;

	return -EINVAL;
}

static int rkcif_g_toisp_fs_rk3576(unsigned int intstat_glb, int index)
{
	if (intstat_glb & TOISP_FS_CH0_RK3576(index))
		return RKCIF_TOISP_CH0;
	if (intstat_glb & TOISP_FS_CH1_RK3576(index))
		return RKCIF_TOISP_CH1;
	if (intstat_glb & TOISP_FS_CH2_RK3576(index))
		return RKCIF_TOISP_CH2;

	return -EINVAL;
}

static int rkcif_g_toisp_ch_rk3576(unsigned int intstat_glb, int index)
{
	if (intstat_glb & TOISP_END_CH0_RK3576(index))
		return RKCIF_TOISP_CH0;
	if (intstat_glb & TOISP_END_CH1_RK3576(index))
		return RKCIF_TOISP_CH1;
	if (intstat_glb & TOISP_END_CH2_RK3576(index))
		return RKCIF_TOISP_CH2;

	return -EINVAL;
}

static u32 rkcif_toisp_get_src_id(struct sditf_priv *priv, int index, int ch)
{
	u32 reg_index = 0;
	u32 val = 0;
	u32 id = 0;

	if (priv->cif_dev->chip_id < CHIP_RV1103B_CIF) {
		if (index == 0)
			reg_index = CIF_REG_TOISP0_CTRL;
		else
			reg_index = CIF_REG_TOISP1_CTRL;
		val = rkcif_read_register(priv->cif_dev, reg_index);
		id = (val >> (ch * 8 + 3)) & 0x1f;
	} else {
		switch (ch) {
		case 0:
			reg_index = CIF_REG_TOISP0_CTRL;
			break;
		case 1:
			reg_index = CIF_REG_TOISP0_CH1_CTRL;
			break;
		case 2:
			reg_index = CIF_REG_TOISP0_CH2_CTRL;
			break;
		default:
			v4l2_err(&priv->cif_dev->v4l2_dev,
				 "get error toisp ch %d\n",
				 ch);
			break;
		}
		val = rkcif_read_register(priv->cif_dev, reg_index);
		id = (val >> 3) & 0x1f;
	}
	return id;
}

static void rkcif_check_vblank_value(struct rkcif_device *dev)
{
	u32 tline = 0;
	u32 vblank_us = 0;
	u32 vblank = 0;

	tline = rkcif_get_linetime(&dev->stream[0]);
	vblank = rkcif_get_sensor_vblank(dev);
	vblank_us = tline * vblank;
	vblank_us = div_u64(vblank_us, 1000);
	if (vblank_us < 1000)
		v4l2_warn(&dev->v4l2_dev,
			  "Warning: vblank need >= 1000us if isp work in online, cur %u us\n",
			  vblank_us);
	else
		v4l2_dbg(3, rkcif_debug, &dev->v4l2_dev, "vblank time %u us\n", vblank_us);
}

static void rkcif_add_sensor_exp_to_kfifo(struct rkcif_stream *stream)
{
	struct rkcif_sensor_exp sensor_exp;
	struct rkcif_sensor_gain sensor_gain;
	struct rkcif_sensor_vts sensor_vts;
	struct rkcif_sensor_dcg sensor_dcg;
	struct rkmodule_exp_info sensor_exp_info = {0};
	int i = 0;
	int ret = 0;
	int j = 0;
	int idx_max = 0;

	if (!stream->cifdev->is_support_get_exp)
		return;

	if (stream->cifdev->hdr.hdr_mode == HDR_X2)
		idx_max = 2;
	else if (stream->cifdev->hdr.hdr_mode == HDR_X3)
		idx_max = 3;
	else
		idx_max = 1;
	ret = v4l2_subdev_call(stream->cifdev->terminal_sensor.sd,
			       core, ioctl,
			       RKMODULE_GET_EXP_INFO,
			       &sensor_exp_info);
	if (!ret)
		stream->sensor_exp_info = sensor_exp_info;

	if (stream->frame_idx == 0) {
		for (i = 0; i < stream->exp_delay.exp_delay; i++) {
			sensor_exp.sequence = i;
			for (j = 0; j < idx_max; j++)
				sensor_exp.exp[j] = stream->sensor_exp_info.exp[j];
			kfifo_in(&stream->exp_kfifo, &sensor_exp, sizeof(sensor_exp));
		}
		for (i = 0; i < stream->exp_delay.gain_delay; i++) {
			sensor_gain.sequence = i;
			for (j = 0; j < idx_max; j++)
				sensor_gain.gain[j] = stream->sensor_exp_info.gain[j];
			kfifo_in(&stream->gain_kfifo, &sensor_gain, sizeof(sensor_gain));
		}
		for (i = 0; i < stream->exp_delay.vts_delay; i++) {
			sensor_vts.sequence = i;
			sensor_vts.vts = stream->sensor_exp_info.vts;
			kfifo_in(&stream->vts_kfifo, &sensor_vts, sizeof(sensor_vts));
		}
		if (sensor_exp_info.dcg_used) {
			for (i = 0; i < stream->exp_delay.dcg_delay; i++) {
				sensor_dcg.sequence = i;
				for (j = 0; j < idx_max; j++)
					sensor_dcg.dcg[j] = stream->sensor_exp_info.dcg_val[j];
				kfifo_in(&stream->dcg_kfifo, &sensor_dcg, sizeof(sensor_dcg));
			}
		}
	} else {
		if (kfifo_is_full(&stream->exp_kfifo)) {
			ret = kfifo_out(&stream->exp_kfifo, &sensor_exp, sizeof(sensor_exp));
			if (!ret)
				v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
					 "stream[%d] seq %d, exp %u %u %u\n",
					 stream->id, sensor_exp.sequence,
					 sensor_exp.exp[0],
					 sensor_exp.exp[1],
					 sensor_exp.exp[2]);
		}
		sensor_exp.sequence = stream->frame_idx + stream->exp_delay.exp_delay - 1;
		for (j = 0; j < idx_max; j++)
			sensor_exp.exp[j] = stream->sensor_exp_info.exp[j];
		kfifo_in(&stream->exp_kfifo, &sensor_exp, sizeof(sensor_exp));

		if (kfifo_is_full(&stream->gain_kfifo)) {
			ret = kfifo_out(&stream->gain_kfifo, &sensor_gain, sizeof(sensor_gain));
			if (!ret)
				v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
					 "stream[%d] seq %d, gain %u %u %u\n",
					 stream->id, sensor_gain.sequence,
					 sensor_gain.gain[0],
					 sensor_gain.gain[1],
					 sensor_gain.gain[2]);
		}
		sensor_gain.sequence = stream->frame_idx + stream->exp_delay.gain_delay - 1;
		for (j = 0; j < idx_max; j++)
			sensor_gain.gain[j] = stream->sensor_exp_info.gain[j];
		kfifo_in(&stream->gain_kfifo, &sensor_gain, sizeof(sensor_gain));

		if (kfifo_is_full(&stream->vts_kfifo)) {
			ret = kfifo_out(&stream->vts_kfifo, &sensor_vts, sizeof(sensor_vts));
			if (!ret)
				v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
					 "stream[%d] seq %d, vts %u\n",
					 stream->id, sensor_vts.sequence, sensor_vts.vts);
		}
		sensor_vts.sequence = stream->frame_idx + stream->exp_delay.vts_delay - 1;
		sensor_vts.vts = stream->sensor_exp_info.vts;
		kfifo_in(&stream->vts_kfifo, &sensor_vts, sizeof(sensor_vts));

		if (sensor_exp_info.dcg_used) {
			if (kfifo_is_full(&stream->dcg_kfifo)) {
				ret = kfifo_out(&stream->dcg_kfifo, &sensor_dcg, sizeof(sensor_dcg));
				if (!ret)
					v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
						 "stream[%d] seq %d, dcg %u %u %u\n",
						 stream->id, sensor_dcg.sequence,
						 sensor_dcg.dcg[0],
						 sensor_dcg.dcg[1],
						 sensor_dcg.dcg[2]);
			}
			sensor_dcg.sequence = stream->frame_idx + stream->exp_delay.dcg_delay - 1;
			for (j = 0; j < idx_max; j++)
				sensor_dcg.dcg[j] = stream->sensor_exp_info.dcg_val[j];
			kfifo_in(&stream->dcg_kfifo, &sensor_dcg, sizeof(sensor_dcg));
		}
	}
}

static int rkcif_get_sof_and_exp_info(struct rkcif_device *cif_dev,
				      struct rkisp_vicap_sof *sof_info)
{
	struct rkcif_stream *stream = &cif_dev->stream[0];
	struct rkcif_sensor_exp sensor_exp = {0};
	struct rkcif_sensor_gain sensor_gain = {0};
	struct rkcif_sensor_vts sensor_vts = {0};
	struct rkcif_sensor_dcg sensor_dcg = {0};
	int ret = 0;
	int i = 0;
	int j = 0;
	int idx_max = 0;

	sof_info->timestamp = stream->readout.fs_timestamp;
	sof_info->sequence = stream->frame_idx;

	if (!cif_dev->is_support_get_exp)
		return 0;

	if (cif_dev->hdr.hdr_mode == HDR_X2)
		idx_max = 2;
	else if (cif_dev->hdr.hdr_mode == HDR_X3)
		idx_max = 3;
	else
		idx_max = 1;

	if (!kfifo_is_empty(&stream->exp_kfifo)) {
		ret = kfifo_out(&stream->exp_kfifo, &sensor_exp, sizeof(sensor_exp));
		if (ret == sizeof(sensor_exp)) {
			v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
				 "stream[%d] seq %d, exp %u %u %u\n",
				 stream->id, sensor_exp.sequence,
				 sensor_exp.exp[0],
				 sensor_exp.exp[1],
				 sensor_exp.exp[2]);
		}
	}
	if (!kfifo_is_empty(&stream->gain_kfifo)) {
		ret = kfifo_out(&stream->gain_kfifo, &sensor_gain, sizeof(sensor_gain));
		if (ret == sizeof(sensor_gain))
			v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
				 "stream[%d] seq %d, gain %u %u %u\n",
				 stream->id, sensor_gain.sequence,
				 sensor_gain.gain[0],
				 sensor_gain.gain[1],
				 sensor_gain.gain[2]);
		else
			return ret;
	}
	if (sensor_exp.sequence > sensor_gain.sequence) {
		for (i = 0; i < RKCIF_EXP_NUM_MAX; i++) {
			if (!kfifo_is_empty(&stream->gain_kfifo)) {
				ret = kfifo_out(&stream->gain_kfifo, &sensor_gain, sizeof(sensor_gain));
				if (ret == sizeof(sensor_gain))
					v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
						 "stream[%d] seq %d, gain %u %u %u\n",
						 stream->id, sensor_gain.sequence,
						 sensor_gain.gain[0],
						 sensor_gain.gain[1],
						 sensor_gain.gain[2]);
				else
					return ret;
				if (sensor_gain.sequence == sensor_exp.sequence)
					break;
				else if (sensor_gain.sequence > sensor_exp.sequence)
					return -EINVAL;
			}
		}
		if (i == RKCIF_EXP_NUM_MAX)
			return -EINVAL;
	} else if (sensor_exp.sequence < sensor_gain.sequence) {
		for (i = 0; i < RKCIF_EXP_NUM_MAX; i++) {
			if (!kfifo_is_empty(&stream->exp_kfifo)) {
				ret = kfifo_out(&stream->exp_kfifo, &sensor_exp, sizeof(sensor_exp));
				if (ret == sizeof(sensor_exp))
					v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
						 "stream[%d] seq %d, exp %u %u %u\n",
						 stream->id, sensor_exp.sequence,
						 sensor_exp.exp[0],
						 sensor_exp.exp[1],
						 sensor_exp.exp[2]);
				else
					return ret;
				if (sensor_exp.sequence == sensor_gain.sequence)
					break;
				else if (sensor_exp.sequence > sensor_gain.sequence)
					return -EINVAL;
			}
		}
		if (i == RKCIF_EXP_NUM_MAX)
			return -EINVAL;
	}
	if (!kfifo_is_empty(&stream->vts_kfifo)) {
		ret = kfifo_out(&stream->vts_kfifo, &sensor_vts, sizeof(sensor_vts));
		if (ret == sizeof(sensor_vts))
			v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
				 "stream[%d] seq %d, vts %u\n",
				 stream->id, sensor_vts.sequence, sensor_vts.vts);
		else
			return ret;
	}
	if (sensor_exp.sequence > sensor_vts.sequence) {
		for (i = 0; i < RKCIF_EXP_NUM_MAX; i++) {
			if (!kfifo_is_empty(&stream->vts_kfifo)) {
				ret = kfifo_out(&stream->vts_kfifo, &sensor_vts, sizeof(sensor_vts));
				if (ret == sizeof(sensor_vts))
					v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
						 "stream[%d] seq %d, vts %u\n",
						 stream->id, sensor_vts.sequence, sensor_vts.vts);
				else
					return ret;
				if (sensor_vts.sequence == sensor_exp.sequence)
					break;
				else if (sensor_vts.sequence > sensor_exp.sequence)
					return -EINVAL;
			}
		}
		if (i == RKCIF_EXP_NUM_MAX)
			return -EINVAL;
	} else if (sensor_exp.sequence < sensor_vts.sequence) {
		return -EINVAL;
	}
	if (stream->sensor_exp_info.dcg_used) {
		if (!kfifo_is_empty(&stream->dcg_kfifo)) {
			ret = kfifo_out(&stream->dcg_kfifo, &sensor_dcg, sizeof(sensor_dcg));
			if (ret == sizeof(sensor_dcg))
				v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
					 "stream[%d] seq %d, dcg %u %u %u\n",
					 stream->id, sensor_dcg.sequence,
					 sensor_dcg.dcg[0],
					 sensor_dcg.dcg[1],
					 sensor_dcg.dcg[2]);
			else
				return ret;
		}
		if (sensor_exp.sequence > sensor_dcg.sequence) {
			for (i = 0; i < RKCIF_EXP_NUM_MAX; i++) {
				if (!kfifo_is_empty(&stream->dcg_kfifo)) {
					ret = kfifo_out(&stream->dcg_kfifo, &sensor_dcg, sizeof(sensor_dcg));
					if (ret == sizeof(sensor_dcg))
						v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
							 "stream[%d] seq %d, dcg %u %u %u\n",
							 stream->id, sensor_dcg.sequence,
							 sensor_dcg.dcg[0],
							 sensor_dcg.dcg[1],
							 sensor_dcg.dcg[2]);
					else
						return ret;
					if (sensor_dcg.sequence == sensor_exp.sequence)
						break;
					else if (sensor_dcg.sequence > sensor_exp.sequence)
						return -EINVAL;
				}
			}
			if (i == RKCIF_EXP_NUM_MAX)
				return -EINVAL;
		} else if (sensor_exp.sequence < sensor_dcg.sequence) {
			return -EINVAL;
		}
		sof_info->dcg_used = 1;
		sof_info->dcg_ratio = stream->sensor_exp_info.dcg_ratio;
		for (j = 0; j < idx_max; j++)
			sof_info->dcg_val[j] = sensor_dcg.dcg[j];
	} else {
		sof_info->dcg_used = 0;
	}
	sof_info->sequence = sensor_exp.sequence;
	for (j = 0; j < idx_max; j++) {
		sof_info->exp[j] = sensor_exp.exp[j];
		sof_info->gain[j] = sensor_gain.gain[j];
	}
	sof_info->vts = sensor_vts.vts;
	sof_info->hts = stream->sensor_exp_info.hts;
	sof_info->pclk = stream->sensor_exp_info.pclk;
	sof_info->gain_mode = stream->sensor_exp_info.gain_mode;
	sof_info->is_exp_active = true;
	v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
		 "seq %d, exp %u %u %u, gain %u %u %u, dcg %u %u %u, vts %u\n",
		 sof_info->sequence,
		 sof_info->exp[0],
		 sof_info->exp[1],
		 sof_info->exp[2],
		 sof_info->gain[0],
		 sof_info->gain[1],
		 sof_info->gain[2],
		 sof_info->dcg_val[0],
		 sof_info->dcg_val[1],
		 sof_info->dcg_val[2],
		 sof_info->vts);
	return 0;
}

static void rkcif_toisp_check_stop_status(struct sditf_priv *priv,
					  unsigned int intstat_glb,
					  int index)
{
	int ch = 0;
	struct rkcif_stream *stream;
	int src_id = 0;
	int i = 0;
	u32 val = 0;
	u64 cur_time = 0;
	int on = 0;
	unsigned long flags;
	struct v4l2_subdev *sd = NULL;
	struct rkisp_vicap_sof sof = {0};

	for (i = 0; i < TOISP_CH_MAX; i++) {
		if (priv->cif_dev->chip_id < CHIP_RK3576_CIF)
			ch = rkcif_g_toisp_ch(intstat_glb, index);
		else
			ch = rkcif_g_toisp_ch_rk3576(intstat_glb, index);
		if (ch >= 0) {
			src_id = priv->toisp_inf.ch_info[ch].id;
			if (src_id != rkcif_toisp_get_src_id(priv, index, ch))
				continue;
			if (src_id == 24)
				stream = &priv->cif_dev->stream[0];
			else
				stream = &priv->cif_dev->stream[src_id % 4];
			if (stream->stopping) {
				v4l2_dbg(3, rkcif_debug, &priv->cif_dev->v4l2_dev,
					 "stream[%d] stop\n",
					 stream->id);
				rkcif_stream_stop(stream);
				stream->stopping = false;
				wake_up(&stream->wq_stopped);
			}
			if (!(stream->cur_stream_mode & RKCIF_STREAM_MODE_CAPTURE)) {
				cur_time = rkcif_time_get_ns(stream->cifdev);
				stream->readout.total_time = cur_time - stream->readout.fe_timestamp;
				stream->readout.readout_time = cur_time - stream->readout.fs_timestamp;
				stream->readout.fe_timestamp = cur_time;
			}

			spin_lock_irqsave(&stream->cifdev->stream_spinlock, flags);
			if (stream->is_wait_stop_complete) {
				if (stream->cifdev->hdr.hdr_mode == NO_HDR ||
				    (stream->cifdev->hdr.hdr_mode == HDR_X2 && stream->id == 1) ||
				    (stream->cifdev->hdr.hdr_mode == HDR_X3 && stream->id == 2)) {
					rkcif_disable_capture(stream);
				} else {
					stream->to_stop_dma = RKCIF_DMAEN_BY_ISP;
					rkcif_stop_dma_capture(stream);
				}
				stream->is_wait_stop_complete = false;
				stream->is_pause_stream = true;
				complete(&stream->stop_complete);
			}
			if (stream->cifdev->sensor_state_change &&
			    (stream->cifdev->hdr.hdr_mode == NO_HDR ||
			    (stream->cifdev->hdr.hdr_mode == HDR_X2 && stream->id == 1) ||
			    (stream->cifdev->hdr.hdr_mode == HDR_X3 && stream->id == 2))) {
				stream->cifdev->sensor_state_change = false;
				spin_unlock_irqrestore(&stream->cifdev->stream_spinlock, flags);
				rkcif_dphy_quick_stream(stream->cifdev, on);
				atomic_inc(&stream->cifdev->sensor_off);
				stream->cifdev->sensor_work.on = stream->cifdev->sensor_state;
				schedule_work(&stream->cifdev->sensor_work.work);
			} else {
				spin_unlock_irqrestore(&stream->cifdev->stream_spinlock, flags);
			}
			spin_lock_irqsave(&stream->cifdev->stream_spinlock, flags);
			if (stream->is_single_cap && (!stream->cur_skip_frame) &&
			    (stream->cifdev->hdr.hdr_mode == NO_HDR ||
			    (stream->cifdev->hdr.hdr_mode == HDR_X2 && stream->id == 1) ||
			    (stream->cifdev->hdr.hdr_mode == HDR_X3 && stream->id == 2))) {
				stream->is_finish_single_cap = true;
				if (!stream->is_wait_single_cap) {
					stream->is_single_cap = false;
					spin_unlock_irqrestore(&stream->cifdev->stream_spinlock, flags);
					rkcif_dphy_quick_stream(stream->cifdev, on);
					stream->cifdev->sensor_work.on = 0;
					atomic_inc(&stream->cifdev->sensor_off);
					schedule_work(&stream->cifdev->sensor_work.work);
				} else {
					stream->is_single_cap = false;
					stream->is_wait_single_cap = false;
					stream->cifdev->resume_mode = RKISP_RTT_MODE_MULTI_FRAME;
					complete(&stream->start_complete);
					spin_unlock_irqrestore(&stream->cifdev->stream_spinlock, flags);
				}
			} else {
				spin_unlock_irqrestore(&stream->cifdev->stream_spinlock, flags);
			}
			if (stream->cur_skip_frame &&
			    (stream->cifdev->hdr.hdr_mode == NO_HDR ||
			     (stream->cifdev->hdr.hdr_mode == HDR_X2 && stream->id == 1) ||
			     (stream->cifdev->hdr.hdr_mode == HDR_X3 && stream->id == 2)))
				stream->cur_skip_frame--;
			if (stream->cifdev->chip_id >= CHIP_RV1106_CIF)
				rkcif_modify_frame_skip_config(stream);
			if (stream->cifdev->rdbk_debug &&
			    stream->frame_idx < 15)
				v4l2_info(&priv->cif_dev->v4l2_dev,
					  "stream[%d] toisp fe %d\n",
					  stream->id,
					  stream->frame_idx - 1);

			switch (ch) {
			case RKCIF_TOISP_CH0:
				if (priv->cif_dev->chip_id < CHIP_RK3576_CIF)
					val = TOISP_END_CH0(index);
				else
					val = TOISP_END_CH0_RK3576(index);
				intstat_glb = intstat_glb & (~val);
				break;
			case RKCIF_TOISP_CH1:
				if (priv->cif_dev->chip_id < CHIP_RK3576_CIF)
					val = TOISP_END_CH1(index);
				else
					val = TOISP_END_CH1_RK3576(index);
				intstat_glb = intstat_glb & (~val);
				break;
			case RKCIF_TOISP_CH2:
				if (priv->cif_dev->chip_id < CHIP_RK3576_CIF)
					val = TOISP_END_CH2(index);
				else
					val = TOISP_END_CH2_RK3576(index);
				intstat_glb = intstat_glb & (~val);
				break;
			default:
				break;
			}
		}
		if (priv->is_toisp_off)
			continue;
		if (priv->cif_dev->chip_id < CHIP_RK3576_CIF)
			ch = rkcif_g_toisp_fs(intstat_glb, index);
		else
			ch = rkcif_g_toisp_fs_rk3576(intstat_glb, index);
		if (ch >= 0 && (!priv->is_toisp_off)) {
			src_id = priv->toisp_inf.ch_info[ch].id;
			if (src_id != rkcif_toisp_get_src_id(priv, index, ch))
				continue;
			if (src_id == 24)
				stream = &priv->cif_dev->stream[0];
			else
				stream = &priv->cif_dev->stream[src_id % 4];
			if (priv->cif_dev->chip_id < CHIP_RK3576_CIF) {
				if (stream->id == 0) {
					spin_lock_irqsave(&stream->fps_lock, flags);
					stream->readout.fs_timestamp = rkcif_time_get_ns(priv->cif_dev);
					spin_unlock_irqrestore(&stream->fps_lock, flags);
					rkcif_add_sensor_exp_to_kfifo(&priv->cif_dev->stream[0]);
					sd = get_rkisp_sd(priv->cif_dev->sditf[0]);
					if (sd) {
						rkcif_get_sof_and_exp_info(priv->cif_dev, &sof);
						v4l2_subdev_call(sd, core, ioctl,
								 RKISP_VICAP_CMD_SOF, &sof);
					}
					spin_lock_irqsave(&stream->vbq_lock, flags);
					if ((!stream->thunderboot_skip_interval ||
					     (stream->thunderboot_skip_interval &&
					      (stream->frame_idx % stream->thunderboot_skip_interval) == 0)) &&
					    (!stream->cifdev->is_in_flip)) {
						rkcif_send_sof(stream->cifdev);
					}
					stream->frame_idx++;
					v4l2_dbg(3, rkcif_debug, &priv->cif_dev->v4l2_dev,
						  "stream[%d] toisp sof seq %d\n",
						  stream->id,
						  stream->frame_idx - 1);
					spin_unlock_irqrestore(&stream->vbq_lock, flags);
					if (stream->frame_idx < 3)
						rkcif_check_vblank_value(stream->cifdev);
				} else {
					stream->frame_idx++;
				}
				if (stream->to_en_dma)
					rkcif_enable_dma_capture(stream, false);
				if (stream->to_en_scale) {
					stream->to_en_scale = false;
					rkcif_scale_start(stream->scale_vdev);
				}
			}
			if ((priv->mode_src.rdbk_mode == RKISP_VICAP_ONLINE_MULTI ||
			     priv->mode_src.rdbk_mode == RKISP_VICAP_ONLINE_UNITE) &&
			    ((priv->hdr_cfg.hdr_mode == NO_HDR && stream->id == 0) ||
			      (priv->hdr_cfg.hdr_mode == HDR_X2 && stream->id == 1) ||
			      (priv->hdr_cfg.hdr_mode == HDR_X3 && stream->id == 2)))
				sditf_disable_immediately(priv);
			stream->buf_wake_up_cnt++;
			if (stream->frame_idx % 2)
				stream->fps_stats.frm0_timestamp = rkcif_time_get_ns(stream->cifdev);
			else
				stream->fps_stats.frm1_timestamp = rkcif_time_get_ns(stream->cifdev);
			if (stream->cifdev->rdbk_debug &&
			    stream->frame_idx < 15)
				v4l2_info(&priv->cif_dev->v4l2_dev,
					  "stream[%d] toisp sof seq %d\n",
					  stream->id,
					  stream->frame_idx - 1);
			switch (ch) {
			case RKCIF_TOISP_CH0:
				if (priv->cif_dev->chip_id < CHIP_RK3576_CIF)
					val = TOISP_FS_CH0(index);
				else
					val = TOISP_FS_CH0_RK3576(index);
				intstat_glb = intstat_glb & (~val);
				break;
			case RKCIF_TOISP_CH1:
				if (priv->cif_dev->chip_id < CHIP_RK3576_CIF)
					val = TOISP_FS_CH1(index);
				else
					val = TOISP_FS_CH1_RK3576(index);
				intstat_glb = intstat_glb & (~val);
				break;
			case RKCIF_TOISP_CH2:
				if (priv->cif_dev->chip_id < CHIP_RK3576_CIF)
					val = TOISP_FS_CH2(index);
				else
					val = TOISP_FS_CH2_RK3576(index);
				intstat_glb = intstat_glb & (~val);
				break;
			default:
				break;
			}
		}
	}
}

void rkcif_irq_handle_toisp(struct rkcif_device *cif_dev, unsigned int intstat_glb)
{
	int i = 0;
	bool to_check = false;
	struct sditf_priv *priv = cif_dev->sditf[0];

	if (!priv || priv->mode.rdbk_mode >= RKISP_VICAP_RDBK_AIQ)
		return;

	for (i = 0; i < 2; i++) {
		to_check = false;
		if (priv->toisp_inf.link_mode == TOISP0 &&
		    i == 0) {
			to_check = true;
		} else if (priv->toisp_inf.link_mode == TOISP1 &&
			   i == 1) {
			to_check = true;
		} else if (priv->toisp_inf.link_mode == TOISP_UNITE) {
			if ((cif_dev->chip_id == CHIP_RK3588_CIF && i == 1) ||
			    (cif_dev->chip_id != CHIP_RK3588_CIF && i == 0))
				to_check = true;
		}
		if (to_check)
			rkcif_toisp_check_stop_status(priv, intstat_glb, i);
	}
}

static int rkcif_check_group_sync_state(struct rkcif_device *cif_dev)
{
	struct rkcif_stream *detect_stream = &cif_dev->stream[0];
	struct rkcif_stream *next_stream = NULL;
	struct rkcif_hw *hw = cif_dev->hw_dev;
	u64 fs_interval = 0;
	int i = 0;
	int ret = 0;
	struct rkcif_multi_sync_config *sync_config;

	sync_config = &hw->sync_config[cif_dev->sync_cfg.group];
	sync_config->sync_code |= BIT(cif_dev->csi_host_idx);
	v4l2_dbg(3, rkcif_debug, &cif_dev->v4l2_dev,
		 "sync code 0x%x, mask 0x%x, update 0x%x, cache 0x%x, timestamp %llu\n",
		 sync_config->sync_code,
		 sync_config->sync_mask,
		 sync_config->update_code,
		 sync_config->update_cache,
		 detect_stream->readout.fs_timestamp);

	if (sync_config->sync_code != sync_config->sync_mask)
		return -EINVAL;

	for (i = 0; i < sync_config->dev_cnt; i++) {
		if (sync_config->mode == RKCIF_MASTER_MASTER) {
			if (i < sync_config->ext_master.count)
				next_stream = &sync_config->ext_master.cif_dev[i]->stream[0];
			else
				next_stream = &sync_config->int_master.cif_dev[0]->stream[0];
		} else if (sync_config->mode == RKCIF_MASTER_SLAVE) {
			if (i < sync_config->slave.count)
				next_stream = &sync_config->slave.cif_dev[i]->stream[0];
			else
				next_stream = &sync_config->int_master.cif_dev[0]->stream[0];
		} else if (sync_config->mode == RKCIF_SOFT_SYNC) {
			next_stream = &sync_config->soft_sync.cif_dev[i]->stream[0];
		} else {
			v4l2_err(&cif_dev->v4l2_dev,
				 "ERROR: invalid group sync mode\n");
			ret = -EINVAL;
			break;
		}
		if (detect_stream == next_stream)
			continue;
		fs_interval = abs(detect_stream->readout.fs_timestamp - next_stream->readout.fs_timestamp);
		if (fs_interval > RKCIF_MAX_INTERVAL_NS) {
			ret = -EINVAL;
			break;
		}
	}
	return ret;
}

static void rkcif_deal_sof(struct rkcif_device *cif_dev)
{
	struct rkcif_stream *detect_stream = &cif_dev->stream[0];
	struct rkcif_hw *hw = cif_dev->hw_dev;
	struct rkcif_device *tmp_dev = NULL;
	unsigned long flags;
	int i = 0;
	int ret = 0;
	struct v4l2_subdev *sd = NULL;
	struct rkisp_vicap_sof sof = {0};

	detect_stream->fs_cnt_in_single_frame++;
	if ((!cif_dev->sditf[0] ||
	     cif_dev->sditf[0]->mode.rdbk_mode >= RKISP_VICAP_RDBK_AIQ) &&
	    detect_stream->fs_cnt_in_single_frame > 1 &&
	    cif_dev->chip_id < CHIP_RK3588_CIF)
		return;

	spin_lock_irqsave(&detect_stream->fps_lock, flags);
	detect_stream->readout.fs_timestamp = rkcif_time_get_ns(cif_dev);
	spin_unlock_irqrestore(&detect_stream->fps_lock, flags);

	rkcif_add_sensor_exp_to_kfifo(&cif_dev->stream[0]);
	sd = get_rkisp_sd(cif_dev->sditf[0]);
	if (sd) {
		rkcif_get_sof_and_exp_info(cif_dev, &sof);
		v4l2_subdev_call(sd, core, ioctl,
				 RKISP_VICAP_CMD_SOF, &sof);
	}

	if (cif_dev->sditf[0] &&
	    cif_dev->sditf[0]->mode.rdbk_mode >= RKISP_VICAP_RDBK_AIQ &&
	    (!detect_stream->dma_en) && cif_dev->chip_id < CHIP_RK3576_CIF)
		return;
	if (cif_dev->sync_cfg.type != NO_SYNC_MODE &&
	    cif_dev->sync_cfg.type != SOFT_SYNC_MODE &&
	    cif_dev->is_detect_group_sync) {
		struct rkcif_multi_sync_config *sync_config;

		sync_config = &hw->sync_config[cif_dev->sync_cfg.group];
		ret = rkcif_check_group_sync_state(cif_dev);
		if (!ret) {
			sync_config->sync_code = 0;
			sync_config->frame_idx++;
			spin_lock_irqsave(&hw->group_lock, flags);
			sync_config->update_code = sync_config->sync_mask;
			spin_unlock_irqrestore(&hw->group_lock, flags);
			for (i = 0; i < sync_config->dev_cnt; i++) {
				if (sync_config->mode == RKCIF_MASTER_MASTER) {
					if (i < sync_config->ext_master.count)
						tmp_dev = sync_config->ext_master.cif_dev[i];
					else
						tmp_dev = sync_config->int_master.cif_dev[0];
				} else if (sync_config->mode == RKCIF_MASTER_SLAVE) {
					if (i < sync_config->slave.count)
						tmp_dev = sync_config->slave.cif_dev[i];
					else
						tmp_dev = sync_config->int_master.cif_dev[0];
				} else {
					v4l2_err(&cif_dev->v4l2_dev,
						 "ERROR: invalid group sync mode\n");
				}
				if (tmp_dev) {
					spin_lock_irqsave(&tmp_dev->stream[0].vbq_lock, flags);
					if ((!tmp_dev->stream[0].thunderboot_skip_interval ||
					    (tmp_dev->stream[0].thunderboot_skip_interval &&
					    (tmp_dev->stream[0].frame_idx % tmp_dev->stream[0].thunderboot_skip_interval) == 0)) &&
					    (!tmp_dev->is_in_flip)) {
						if (tmp_dev->channels[0].capture_info.mode == RKMODULE_MULTI_CH_TO_MULTI_ISP &&
						    tmp_dev->sditf[tmp_dev->stream[0].id])
							sditf_event_inc_sof(tmp_dev->sditf[tmp_dev->stream[0].id]);
						else if (tmp_dev->channels[0].capture_info.mode == RKMODULE_ONE_CH_TO_MULTI_ISP)
							schedule_work(&tmp_dev->exp_work);
						else
							rkcif_send_sof(tmp_dev);
						if (tmp_dev->stream[0].cifdev->rdbk_debug &&
						    tmp_dev->stream[0].frame_idx < 15)
							v4l2_info(&tmp_dev->v4l2_dev,
								  "stream[%d] send sof %d, real sof %d\n",
								  tmp_dev->stream[0].id,
								  rkcif_get_sof(tmp_dev),
								  tmp_dev->stream[0].frame_idx);
					}
					if (tmp_dev->sditf[0] &&
					    tmp_dev->sditf[0]->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ &&
					    tmp_dev->sditf[0]->is_multi_online) {
						if (!tmp_dev->sditf[0]->is_toisp_off)
							tmp_dev->stream[0].frame_idx++;
					} else {
						tmp_dev->stream[0].frame_idx = sync_config->frame_idx;
					}
					spin_unlock_irqrestore(&tmp_dev->stream[0].vbq_lock, flags);
				}
			}
		}
	} else {
		spin_lock_irqsave(&detect_stream->vbq_lock, flags);
		if ((!detect_stream->thunderboot_skip_interval ||
		    (detect_stream->thunderboot_skip_interval &&
		    (detect_stream->frame_idx % detect_stream->thunderboot_skip_interval) == 0)) &&
		    (!detect_stream->cifdev->is_in_flip)) {
			if (cif_dev->channels[0].capture_info.mode == RKMODULE_MULTI_CH_TO_MULTI_ISP &&
			    cif_dev->sditf[detect_stream->id])
				sditf_event_inc_sof(cif_dev->sditf[detect_stream->id]);
			else if (cif_dev->channels[0].capture_info.mode == RKMODULE_ONE_CH_TO_MULTI_ISP)
				schedule_work(&cif_dev->exp_work);
			else
				rkcif_send_sof(cif_dev);
			if ((detect_stream->cifdev->rdbk_debug &&
			    detect_stream->frame_idx < 15) ||
			    rkcif_debug > 3)
				v4l2_info(&cif_dev->v4l2_dev,
					  "stream[%d] send sof %d, real sof %d\n",
					  detect_stream->id,
					  rkcif_get_sof(cif_dev),
					  detect_stream->frame_idx);
			if (rkcif_check_frame_active(cif_dev))
				detect_stream->frame_idx++;
		}
		spin_unlock_irqrestore(&detect_stream->vbq_lock, flags);
		if (detect_stream->cifdev->rdbk_debug &&
		    detect_stream->frame_idx < 15 &&
		    (!cif_dev->sditf[0] || cif_dev->sditf[0]->mode.rdbk_mode >= RKISP_VICAP_RDBK_AIQ))
			v4l2_info(&cif_dev->v4l2_dev,
				  "stream[%d] real sof %d %lld\n",
				  detect_stream->id,
				  detect_stream->frame_idx - 1,
				  rkcif_time_get_ns(cif_dev));
	}
	if (detect_stream->frame_idx < 3 && cif_dev->sditf[0])
		rkcif_check_vblank_value(cif_dev);
}

unsigned int rkcif_irq_global(struct rkcif_device *cif_dev)
{
	unsigned int intstat_glb = 0;

	intstat_glb = rkcif_read_register(cif_dev, CIF_REG_GLB_INTST);
	if (intstat_glb)
		v4l2_dbg(2, rkcif_debug, &cif_dev->v4l2_dev,
			 "intstat_glb 0x%x\n",
			 intstat_glb);
	else
		return intstat_glb;

	if (intstat_glb & SCALE_TOISP_AXI0_ERR) {
		v4l2_err(&cif_dev->v4l2_dev,
			"ERROR: AXI0 bus err intstat_glb:0x%x !!\n",
			intstat_glb);
		cif_dev->irq_stats.bus0_err++;
		cif_dev->irq_stats.all_err_cnt++;
		return 0;
	}
	if (cif_dev->chip_id == CHIP_RK3588_CIF && intstat_glb & SCALE_TOISP_AXI1_ERR) {
		v4l2_err(&cif_dev->v4l2_dev,
			"ERROR: AXI1 bus err intstat_glb:0x%x !!\n",
			intstat_glb);
		cif_dev->irq_stats.bus1_err++;
		cif_dev->irq_stats.all_err_cnt++;
		return 0;
	}
	return intstat_glb;
}

static bool rkcif_check_buffer_prepare(struct rkcif_stream *stream)
{
	struct rkcif_device *cif_dev = stream->cifdev;
	unsigned long flags;
	bool is_update = false;
	struct rkcif_multi_sync_config *sync_config;

	spin_lock_irqsave(&cif_dev->hw_dev->group_lock, flags);
	sync_config = &cif_dev->hw_dev->sync_config[cif_dev->sync_cfg.group];
	if (stream->id == 0 &&
	    sync_config->update_code & BIT(cif_dev->csi_host_idx)) {
		is_update = true;
		sync_config->update_code &= ~(BIT(cif_dev->csi_host_idx));
	} else if (stream->id != 0) {
		if ((stream->dma_en & RKCIF_DMAEN_BY_ISP && cif_dev->rdbk_rx_buf[RDBK_L]) ||
		    (stream->dma_en & RKCIF_DMAEN_BY_VICAP && cif_dev->rdbk_buf[RDBK_L]))
			is_update = true;
		else
			v4l2_err(&cif_dev->v4l2_dev,
					 "ERR: loss long frame in readback mode\n");
	}
	spin_unlock_irqrestore(&cif_dev->hw_dev->group_lock, flags);
	return is_update;
}

bool rkcif_check_single_dev_stream_on(struct rkcif_hw *hw)
{
	struct rkcif_device *cif_dev = NULL;
	struct v4l2_subdev *sd = NULL;
	int i = 0;
	int stream_cnt = 0;

	if (hw->dev_num == 1)
		return true;
	for (i = 0; i < hw->dev_num; i++) {
		cif_dev = hw->cif_dev[i];
		sd = get_rkisp_sd(cif_dev->sditf[0]);
		if (sd)
			stream_cnt++;
	}
	if (stream_cnt > 1)
		return false;
	return true;
}

static u64 rkcif_get_boot_time_ns_from_arch_timer(void)
{
	u64 ns;

	ns = arch_timer_read_counter() * 1000;
	do_div(ns, 24);

	return ns;
}

static u64 rkcif_get_rtt_time_offset(struct rkcif_device *cif_dev)
{
	u64 offset = 0;
	u64 arch_time = 0;

	arch_time = rkcif_get_boot_time_ns_from_arch_timer();
	offset = arch_time - rkcif_time_get_ns(cif_dev);
	return offset;
}

static void rkcif_get_resmem_head(struct rkcif_device *cif_dev)
{
	void *resmem_va = phys_to_virt(cif_dev->resmem_pa);
	struct rkisp_thunderboot_resmem_head *head = NULL;
	struct rkisp32_thunderboot_resmem_head *tmp = NULL;
	int size = 0;
	int offset = 0;
	int i = 0;
	int dev_id = 0;
	u64 rtt_offset_time = 0;

	if (cif_dev->resmem_pa == 0 || cif_dev->resmem_size == 0)
		return;

	size = sizeof(struct rkisp32_thunderboot_resmem_head);
	if (cif_dev->sditf[0])
		dev_id = cif_dev->sditf[0]->mode.dev_id;
	offset = size * dev_id;

	tmp = resmem_va + offset;

	dma_sync_single_for_cpu(cif_dev->dev, cif_dev->resmem_addr + offset,
				size, DMA_FROM_DEVICE);
	head = &tmp->head;
	if (cif_dev->is_rtt_suspend) {
		cif_dev->resume_mode = head->rtt_mode;
		cif_dev->nr_buf_size = head->nr_buf_size;
	}
	cif_dev->share_mem_size = head->share_mem_size;
	cif_dev->thunderboot_sensor_num = head->camera_num;
	if (head->pre_buf_num && head->pre_buf_num < MAX_PRE_BUF_NUM) {
		cif_dev->pre_buf_num = head->pre_buf_num;
		rtt_offset_time = rkcif_get_rtt_time_offset(cif_dev);
		for (i = 0; i < head->pre_buf_num; i++) {
			cif_dev->pre_buf_addr[i] = head->pre_buf_addr[i];
			cif_dev->pre_buf_timestamp[i] = head->pre_buf_timestamp[i] * 1000000;
			if (cif_dev->pre_buf_timestamp[i] < rtt_offset_time)
				cif_dev->pre_buf_timestamp[i] = 0;
			else
				cif_dev->pre_buf_timestamp[i] -= rtt_offset_time;
		}
	}

	v4l2_err(&cif_dev->v4l2_dev,
		 "get isp_dev %02x, resume_mode 0x%x, nr_buf_size %d\n",
		 dev_id, cif_dev->resume_mode, cif_dev->nr_buf_size);
}

static int rkcif_subdevs_set_power(struct rkcif_device *cif_dev, int on)
{
	struct sditf_priv *priv = cif_dev->sditf[0];
	int ret = 0;
	int i = 0;

	if (cif_dev->terminal_sensor.sd)
		ret = v4l2_subdev_call(cif_dev->terminal_sensor.sd,
				       core, s_power, on);
	if (priv && cif_dev->sditf_cnt > 1) {
		if (priv->is_combine_mode) {
			for (i = 0; i < cif_dev->sditf_cnt; i++) {
				if (cif_dev->sditf[i] && cif_dev->sditf[i]->sensor_sd)
					v4l2_subdev_call(cif_dev->sditf[i]->sensor_sd, core,
							 s_power, on);
			}
		} else if (cif_dev->is_camera_over_bridge) {
			for (i = 0; i < cif_dev->sditf_cnt; i++) {
				if (cif_dev->sditf[i] && cif_dev->sditf[i]->sensor_sd &&
				    (cif_dev->stream[i].state == RKCIF_STATE_STREAMING ||
				     cif_dev->stream[i].state == RKCIF_STATE_RESET_IN_STREAMING))
					v4l2_subdev_call(cif_dev->sditf[i]->sensor_sd, core,
							 s_power, on);
			}
		}
	}
	return ret;
}

static void rkcif_sensor_quick_streaming_cb(void *data)
{
	struct v4l2_subdev *subdevs = (struct v4l2_subdev *)data;
	int on = 1;

	v4l2_subdev_call(subdevs, core, ioctl,
			 RKMODULE_SET_QUICK_STREAM, &on);
}

static int rkcif_terminal_sensor_set_stream(struct rkcif_device *cif_dev, int on)
{
	struct rkcif_pipeline *p = &cif_dev->pipe;
	struct rkcif_sensor_info *terminal_sensor = &cif_dev->terminal_sensor;
	int i = 0;
	int ret = 0;

	for (i = 0; i < p->num_subdevs; i++) {
		if (p->subdevs[i] == terminal_sensor->sd && on)
			rkcif_set_sof(cif_dev, cif_dev->stream[0].frame_idx);
		if (p->subdevs[i] == terminal_sensor->sd &&
		    (cif_dev->chip_id == CHIP_RV1106_CIF ||
		     cif_dev->chip_id == CHIP_RV1103B_CIF ||
		     cif_dev->chip_id == CHIP_RV1126B_CIF)) {
			if (!rk_tb_mcu_is_done() && on) {
				cif_dev->tb_client.data = p->subdevs[i];
				cif_dev->tb_client.cb = rkcif_sensor_quick_streaming_cb;
				rk_tb_client_register_cb(&cif_dev->tb_client);
			} else {
				if (on && cif_dev->sync_cfg.type != NO_SYNC_MODE) {
					rkcif_set_sensor_streamon_in_sync_mode(cif_dev);
					continue;
				}
				ret = v4l2_subdev_call(p->subdevs[i], core, ioctl,
						       RKMODULE_SET_QUICK_STREAM, &on);
				if (ret)
					v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev,
						 "%s:quick stream %s subdev:%s failed\n",
						 __func__, on ? "on" : "off",
						 p->subdevs[i]->name);
			}
		} else {
			ret = v4l2_subdev_call(p->subdevs[i], video, s_stream, on);
			if (ret)
				v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev,
					 "%s:stream %s subdev:%s failed\n",
					 __func__, on ? "on" : "off", p->subdevs[i]->name);
		}
	}

	return ret;
}

static int rkcif_sditf_sensor_set_stream(struct rkcif_device *cif_dev, int on)
{
	struct sditf_priv *priv = cif_dev->sditf[0];
	int i = 0;
	int ret = 0;

	if (priv && cif_dev->sditf_cnt > 1) {
		if (cif_dev->is_camera_over_bridge) {
			for (i = 0; i < cif_dev->sditf_cnt; i++) {
				if (cif_dev->sditf[i] && cif_dev->sditf[i]->sensor_sd &&
				    (cif_dev->stream[i].state == RKCIF_STATE_STREAMING ||
				     cif_dev->stream[i].state == RKCIF_STATE_RESET_IN_STREAMING)) {
					ret = v4l2_subdev_call(cif_dev->sditf[i]->sensor_sd, video, s_stream, on);
					if (ret)
						v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev,
							 "%s:stream %s subdev:%s failed\n",
							 __func__, on ? "on" : "off",
							 cif_dev->sditf[i]->sensor_sd->name);
				}
			}
		} else if (priv->is_combine_mode) {
			for (i = 0; i < cif_dev->sditf_cnt; i++) {
				if (cif_dev->sditf[i] && cif_dev->sditf[i]->sensor_sd) {
					ret = v4l2_subdev_call(cif_dev->sditf[i]->sensor_sd, video, s_stream, on);
					if (ret)
						v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev,
							 "%s:stream %s subdev:%s failed\n",
							 __func__, on ? "on" : "off",
							 cif_dev->sditf[i]->sensor_sd->name);
				}
			}
		}
	}

	return ret;
}

static int rkcif_subdevs_set_stream(struct rkcif_device *cif_dev, int on)
{
	int ret = 0;

	if (on) {
		ret |= rkcif_terminal_sensor_set_stream(cif_dev, on);
		ret |= rkcif_sditf_sensor_set_stream(cif_dev, on);
	} else {
		ret |= rkcif_sditf_sensor_set_stream(cif_dev, on);
		ret |= rkcif_terminal_sensor_set_stream(cif_dev, on);
	}

	return ret;
}

int rkcif_stream_suspend(struct rkcif_device *cif_dev, int mode)
{
	struct rkcif_stream *stream = NULL;
	struct rkcif_resume_info *resume_info = &cif_dev->reset_work.resume_info;
	struct sditf_priv *priv = cif_dev->sditf[0];
	int ret = 0;
	int i = 0;
	int sof_cnt = 0;
	int on = 0;
	int suspend_cnt = 0;

	mutex_lock(&cif_dev->stream_lock);

	if (priv && priv->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ && mode == RKCIF_RESUME_CIF)
		goto out_suspend;

	for (i = 0; i < RKCIF_MAX_STREAM_MIPI; i++) {
		stream = &cif_dev->stream[i];

		if (stream->state == RKCIF_STATE_STREAMING) {
			suspend_cnt++;
			v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev,
				 "stream[%d] stopping\n", stream->id);
			if (atomic_read(&cif_dev->sensor_off) == 0) {
				stream->stopping = true;
				ret = wait_event_timeout(stream->wq_stopped,
							 stream->state != RKCIF_STATE_STREAMING,
							 msecs_to_jiffies(500));
				if (!ret) {
					rkcif_stream_stop(stream);
					stream->stopping = false;
				}
			} else {
				rkcif_stream_stop(stream);
			}

			if (stream->id == RKCIF_STREAM_MIPI_ID0) {
				sof_cnt = rkcif_get_sof(cif_dev);
				v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev,
					 "%s: stream[%d] sync frmid & csi_sof, frm_id:%d, csi_sof:%d\n",
					 __func__,
					 stream->id,
					 stream->frame_idx,
					 sof_cnt);

				resume_info->frm_sync_seq = stream->frame_idx;
			}

			stream->state = RKCIF_STATE_RESET_IN_STREAMING;
			stream->is_fs_fe_not_paired = false;
			stream->fs_cnt_in_single_frame = 0;

			v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev,
				 "%s stop stream[%d] in streaming, frm_id:%d, csi_sof:%d\n",
				 __func__, stream->id, stream->frame_idx, rkcif_get_sof(cif_dev));

		}
	}

	if (suspend_cnt == 0)
		goto out_suspend;

	if (!cif_dev->resume_mode)
		rkcif_subdevs_set_power(cif_dev, on);

	rkcif_subdevs_set_stream(cif_dev, on);

out_suspend:
	mutex_unlock(&cif_dev->stream_lock);
	return 0;
}

static void rkcif_clean_buffer_state(struct rkcif_stream *stream)
{
	struct sditf_priv *priv = stream->cifdev->sditf[0];
	unsigned long flags;

	spin_lock_irqsave(&stream->vbq_lock, flags);
	if (!priv || priv->mode.rdbk_mode == RKISP_VICAP_RDBK_AIQ) {
		if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT ||
		    rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO) {
			if (stream->curr_buf == stream->next_buf) {
				if (stream->curr_buf)
					list_add_tail(&stream->curr_buf->queue, &stream->buf_head);
			} else {
				if (stream->curr_buf)
					list_add_tail(&stream->curr_buf->queue, &stream->buf_head);
				if (stream->next_buf)
					list_add_tail(&stream->next_buf->queue, &stream->buf_head);
			}
			stream->curr_buf = NULL;
			stream->next_buf = NULL;
		}
	} else {
		if (priv->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ) {
			if (stream->curr_buf_toisp == stream->next_buf_toisp) {
				if (stream->curr_buf_toisp)
					list_add_tail(&stream->curr_buf_toisp->list, &stream->rx_buf_head);
			} else {
				if (stream->curr_buf_toisp)
					list_add_tail(&stream->curr_buf_toisp->list, &stream->rx_buf_head);
				if (stream->next_buf_toisp)
					list_add_tail(&stream->next_buf_toisp->list, &stream->rx_buf_head);
			}
			stream->curr_buf_toisp = NULL;
			stream->next_buf_toisp = NULL;
		} else {
			if (stream->curr_buf_toisp == stream->next_buf_toisp) {
				if (stream->curr_buf_toisp)
					list_add_tail(&stream->curr_buf_toisp->list, &stream->rx_buf_head);
			} else {
				if (stream->curr_buf_toisp)
					list_add_tail(&stream->curr_buf_toisp->list, &stream->rx_buf_head);
				if (stream->next_buf_toisp)
					list_add_tail(&stream->next_buf_toisp->list, &stream->rx_buf_head);
			}
			stream->curr_buf_toisp = NULL;
			stream->next_buf_toisp = NULL;
		}
	}

	spin_unlock_irqrestore(&stream->vbq_lock, flags);
}

int rkcif_stream_resume(struct rkcif_device *cif_dev, int mode)
{
	struct rkcif_stream *stream = NULL;
	struct rkcif_stream *tmp_stream = NULL;
	struct sditf_priv *priv = cif_dev->sditf[0];
	struct v4l2_subdev *sd = NULL;
	int ret = 0;
	int i = 0;
	u32 capture_mode = 0;
	int on = 1;
	int resume_cnt = 0;
	unsigned long flags;
	struct rkisp_vicap_mode vicap_mode;
	bool is_single_dev = false;

	mutex_lock(&cif_dev->stream_lock);

	rkcif_get_resmem_head(cif_dev);
	if (cif_dev->resume_mode == RKISP_RTT_MODE_ONE_FRAME) {
		if (cif_dev->is_rtt_suspend) {
			capture_mode = RKCIF_STREAM_MODE_TOISP_RDBK;
			if (priv)
				priv->mode.rdbk_mode = RKISP_VICAP_RDBK_AUTO;
		} else {
			if (priv) {
				priv->mode.rdbk_mode = priv->mode_src.rdbk_mode;
				if (priv->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ) {
					if (cif_dev->chip_id == CHIP_RV1106_CIF) {
						capture_mode = RKCIF_STREAM_MODE_TOISP_RDBK;
						priv->mode.rdbk_mode = RKISP_VICAP_RDBK_AUTO_ONE_FRAME;
					} else {
						capture_mode = RKCIF_STREAM_MODE_TOISP;
						priv->mode.rdbk_mode = RKISP_VICAP_ONLINE_ONE_FRAME;
					}
				} else {
					priv->mode.rdbk_mode = RKISP_VICAP_RDBK_AUTO_ONE_FRAME;
					capture_mode = RKCIF_STREAM_MODE_TOISP_RDBK;
				}
			} else {
				capture_mode = RKCIF_STREAM_MODE_CAPTURE;
			}
		}
	} else if (cif_dev->resume_mode == RKISP_RTT_MODE_MULTI_FRAME) {
		if (priv) {
			priv->mode.rdbk_mode = priv->mode_src.rdbk_mode;
			if (priv->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ)
				capture_mode = RKCIF_STREAM_MODE_TOISP;
			else if (priv->mode.rdbk_mode == RKISP_VICAP_RDBK_AIQ)
				capture_mode = RKCIF_STREAM_MODE_CAPTURE;
			else
				capture_mode = RKCIF_STREAM_MODE_TOISP_RDBK;
		} else {
			capture_mode = RKCIF_STREAM_MODE_CAPTURE;
		}
	} else {
		if (priv) {
			if (priv->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ)
				capture_mode = RKCIF_STREAM_MODE_TOISP;
			else if (priv->mode.rdbk_mode == RKISP_VICAP_RDBK_AUTO ||
				 priv->mode.rdbk_mode == RKISP_VICAP_RDBK_AUTO_ONE_FRAME)
				capture_mode = RKCIF_STREAM_MODE_TOISP_RDBK;
			else
				capture_mode = RKCIF_STREAM_MODE_CAPTURE;
		} else {
			capture_mode = RKCIF_STREAM_MODE_CAPTURE;
		}
	}
	if (priv && priv->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ && mode == RKCIF_RESUME_CIF)
		goto out_resume;

	for (i = 0; i < RKCIF_MAX_STREAM_MIPI; i++) {
		stream = &cif_dev->stream[i];
		if (stream->state != RKCIF_STATE_RESET_IN_STREAMING)
			continue;

		stream->fs_cnt_in_single_frame = 0;
		if (cif_dev->resume_mode == RKISP_RTT_MODE_ONE_FRAME) {
			spin_lock_irqsave(&stream->cifdev->stream_spinlock, flags);
			stream->is_single_cap = true;
			stream->is_finish_single_cap = false;
			stream->is_wait_single_cap = false;
			spin_unlock_irqrestore(&stream->cifdev->stream_spinlock, flags);
		}
		if (cif_dev->switch_info.is_use_switch) {
			if (atomic_read(&cif_dev->hw_dev->switch_stream_cnt[cif_dev->switch_info.host_idx]) == 0) {
				tmp_stream = stream;
				if (cif_dev->switch_info.switch_dev->switch_info.is_init_buf)
					tmp_stream = &cif_dev->switch_info.switch_dev->stream[stream->id];
				rkcif_clean_buffer_state(tmp_stream);
			}
		} else {
			rkcif_clean_buffer_state(stream);
		}

		is_single_dev = rkcif_check_single_dev_stream_on(cif_dev->hw_dev);
		if (priv) {
			if (stream->is_single_cap && stream->id == 0) {
				vicap_mode = priv->mode;
				sd = get_rkisp_sd(priv);
				if (sd) {
					ret = v4l2_subdev_call(sd, core, ioctl,
							       RKISP_VICAP_CMD_MODE, &vicap_mode);
					if (ret)
						v4l2_err(&cif_dev->v4l2_dev,
							 "set isp work mode %d failed\n", vicap_mode.rdbk_mode);
					else
						v4l2_dbg(1, rkcif_debug, &stream->cifdev->v4l2_dev,
							 "set isp work mode %d", vicap_mode.rdbk_mode);
				}
			}

			if (priv->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ) {
				if (cif_dev->chip_id == CHIP_RV1106_CIF || is_single_dev)
					sditf_change_to_online(priv);
				if (cif_dev->resume_mode == RKISP_RTT_MODE_MULTI_FRAME &&
				    stream->rx_buf_num &&
				    (priv->hdr_cfg.hdr_mode == NO_HDR ||
				     (priv->hdr_cfg.hdr_mode == HDR_X2 && stream->id == 1) ||
				     (priv->hdr_cfg.hdr_mode == HDR_X3 && stream->id == 2)))
					rkcif_free_rx_buf(stream, priv->buf_num);
				else if (!stream->rx_buf_num &&
					 ((priv->hdr_cfg.hdr_mode == HDR_X2 && stream->id == 0) ||
					 (priv->hdr_cfg.hdr_mode == HDR_X3 && (stream->id == 0 || stream->id == 1))))
					rkcif_init_rx_buf(stream, 1);
			} else {
				if (cif_dev->chip_id == CHIP_RV1106_CIF || is_single_dev)
					sditf_disable_immediately(priv);
				if (!stream->rx_buf_num &&
				    capture_mode == RKCIF_STREAM_MODE_TOISP_RDBK &&
				    (!cif_dev->switch_info.is_use_switch)) {
					if (cif_dev->resume_mode == RKISP_RTT_MODE_ONE_FRAME)
						rkcif_init_rx_buf(stream, 1);
					else
						rkcif_init_rx_buf(stream, priv->buf_num);
				}
			}
			v4l2_dbg(3, rkcif_debug, &stream->cifdev->v4l2_dev,
				 "toisp work mode %d, capture mode %d\n", priv->mode.rdbk_mode, capture_mode);
		}

		stream->lack_buf_cnt = 0;
		if (cif_dev->active_sensor  &&
		    (cif_dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_DPHY ||
		    cif_dev->active_sensor->mbus.type == V4L2_MBUS_CSI2_CPHY ||
		    cif_dev->active_sensor->mbus.type == V4L2_MBUS_CCP2))
			ret = rkcif_csi_stream_start(stream, capture_mode);
		else
			ret = rkcif_stream_start(stream, capture_mode);
		if (ret)
			v4l2_err(&cif_dev->v4l2_dev, "%s:resume stream[%d] failed\n",
				 __func__, stream->id);

		resume_cnt++;
		stream->cur_skip_frame = stream->skip_frame;
		v4l2_dbg(1, rkcif_debug, &cif_dev->v4l2_dev,
			 "resume stream[%d], frm_idx:%d, csi_sof:%d\n",
			 stream->id, stream->frame_idx,
			 rkcif_get_sof(cif_dev));
	}

	if (resume_cnt == 0)
		goto out_resume;

	if (!cif_dev->resume_mode)
		rkcif_subdevs_set_power(cif_dev, on);

	atomic_set(&cif_dev->streamoff_cnt, 0);
	rkcif_subdevs_set_stream(cif_dev, on);

	if (cif_dev->chip_id < CHIP_RK3588_CIF)
		rkcif_start_luma(&cif_dev->luma_vdev,
				 cif_dev->stream[RKCIF_STREAM_MIPI_ID0].cif_fmt_in);
out_resume:
	mutex_unlock(&cif_dev->stream_lock);
	return 0;
}

void rkcif_err_print_work(struct work_struct *work)
{
	struct rkcif_err_state_work *err_state_work = container_of(work,
							      struct rkcif_err_state_work,
							      work);
	struct rkcif_device *dev = container_of(err_state_work,
						struct rkcif_device,
						err_state_work);
	u32 err_state = 0;
	int intstat = 0;
	int lastline = 0;
	int lastpixel = 0;
	u64 cur_time = 0;
	bool is_print = false;

	cur_time = rkcif_time_get_ns(dev);
	if (err_state_work->last_timestamp == 0) {
		is_print = true;
	} else {
		if (cur_time - err_state_work->last_timestamp > 500000000)
			is_print = true;
	}
	err_state_work->last_timestamp = cur_time;
	err_state = err_state_work->err_state;
	intstat = err_state_work->intstat;
	lastline = err_state_work->lastline;
	lastpixel = err_state_work->lastpixel;
	if (err_state & RKCIF_ERR_ID0_NOT_BUF && is_print)
		v4l2_err(&dev->v4l2_dev,
			 "stream[0] not active buffer, frame num %d, cnt %llu\n",
			 dev->stream[0].frame_idx, dev->irq_stats.not_active_buf_cnt[0]);
	if (err_state & RKCIF_ERR_ID1_NOT_BUF && is_print)
		v4l2_err(&dev->v4l2_dev,
			 "stream[1] not active buffer, frame num %d, cnt %llu\n",
			 dev->stream[1].frame_idx, dev->irq_stats.not_active_buf_cnt[1]);
	if (err_state & RKCIF_ERR_ID2_NOT_BUF && is_print)
		v4l2_err(&dev->v4l2_dev,
			 "stream[2] not active buffer, frame num %d, cnt %llu\n",
			 dev->stream[2].frame_idx, dev->irq_stats.not_active_buf_cnt[2]);
	if (err_state & RKCIF_ERR_ID3_NOT_BUF && is_print)
		v4l2_err(&dev->v4l2_dev,
			 "stream[3] not active buffer, frame num %d, cnt %llu\n",
			 dev->stream[3].frame_idx, dev->irq_stats.not_active_buf_cnt[3]);
	if (err_state & RKCIF_ERR_ID0_TRIG_SIMULT && is_print)
		v4l2_err(&dev->v4l2_dev,
			 "stream[0], frm0/frm1 end simultaneously,frm id:%d, cnt %llu\n",
			 dev->stream[0].frame_idx, dev->irq_stats.trig_simult_cnt[0]);
	if (err_state & RKCIF_ERR_ID1_TRIG_SIMULT && is_print)
		v4l2_err(&dev->v4l2_dev,
			 "stream[1], frm0/frm1 end simultaneously,frm id:%d, cnt %llu\n",
			 dev->stream[1].frame_idx, dev->irq_stats.trig_simult_cnt[1]);
	if (err_state & RKCIF_ERR_ID2_TRIG_SIMULT && is_print)
		v4l2_err(&dev->v4l2_dev,
			 "stream[2], frm0/frm1 end simultaneously,frm id:%d, cnt %llu\n",
			 dev->stream[2].frame_idx, dev->irq_stats.trig_simult_cnt[2]);
	if (err_state & RKCIF_ERR_ID3_TRIG_SIMULT && is_print)
		v4l2_err(&dev->v4l2_dev,
			 "stream[3], frm0/frm1 end simultaneously,frm id:%d, cnt %llu\n",
			 dev->stream[3].frame_idx, dev->irq_stats.trig_simult_cnt[3]);
	if (err_state & RKCIF_ERR_SIZE) {
		if (dev->chip_id >= CHIP_RK3588_CIF)
			v4l2_err(&dev->v4l2_dev,
				 "ERROR: size err, intstat:0x%x, size:0x%x,0x%x,0x%x,0x%x, cnt %llu\n",
				 intstat, err_state_work->size_id0, err_state_work->size_id1,
				 err_state_work->size_id2, err_state_work->size_id3,
				 dev->irq_stats.csi_size_err_cnt);
		else
			v4l2_err(&dev->v4l2_dev,
				 "ERROR: size err, intstat:0x%x, lastline:0x%x, cnt %llu\n",
				 intstat, lastline, dev->irq_stats.csi_size_err_cnt);
	}
	if (err_state & RKCIF_ERR_OVERFLOW)
		v4l2_err(&dev->v4l2_dev,
			 "ERROR: fifo overflow, intstat:0x%x, lastline:0x%x, cnt %llu\n",
			 intstat, lastline, dev->irq_stats.csi_overflow_cnt);
	if (err_state & RKCIF_ERR_BANDWIDTH_LACK)
		v4l2_err(&dev->v4l2_dev,
			 "ERROR: bandwidth lack, intstat:0x%x, lastline:0x%x, cnt %llu\n",
			 intstat, lastline, dev->irq_stats.csi_bwidth_lack_cnt);
	if (err_state & RKCIF_ERR_ID0_MULTI_FS)
		v4l2_err(&dev->v4l2_dev,
			 "ERR: multi fs in oneframe in id0, fs_num:%u\n",
			 dev->stream[0].fs_cnt_in_single_frame);
	if (err_state & RKCIF_ERR_BUS)
		v4l2_err(&dev->v4l2_dev, "dvp bus err, intstat 0x%x, last line 0x%x\n",
			 intstat, lastline);
	if (err_state & RKCIF_ERR_PIXEL)
		v4l2_err(&dev->v4l2_dev, "dvp pix err, intstat 0x%x, last pixel 0x%x\n",
			 intstat, lastpixel);
	if (err_state & RKCIF_ERR_LINE)
		v4l2_err(&dev->v4l2_dev, "dvp line err, intstat 0x%x, last line 0x%x\n",
			 intstat, lastline);
}

static void rkcif_check_mipi_interlaced_frame_id(struct rkcif_stream *stream)
{
	struct rkcif_device *dev = stream->cifdev;
	struct rkcif_buffer *buffer = NULL;
	struct rkcif_dummy_buffer *dummy_buf = &dev->hw_dev->dummy_buf;
	int frame_id = 0;
	int fs_interlaced_phase = 0;
	unsigned long flags;
	int buf_offset = 0;

	if (stream->id == 0)
		frame_id = rkcif_read_register(dev, CIF_REG_MIPI_FRAME_NUM_VC0);
	else if (stream->id == 1)
		frame_id = rkcif_read_register(dev, CIF_REG_MIPI_FRAME_NUM_VC1);
	else if (stream->id == 2)
		frame_id = rkcif_read_register(dev, CIF_REG_MIPI_FRAME_NUM_VC2);
	else if (stream->id == 3)
		frame_id = rkcif_read_register(dev, CIF_REG_MIPI_FRAME_NUM_VC3);

	if ((frame_id & 0xffff) % 2 == stream->odd_frame_id) {
		//odd frame
		if (stream->odd_frame_first) {
			fs_interlaced_phase = CIF_CSI_FRAME0_READY;
			buf_offset = 1;
		} else {
			fs_interlaced_phase = CIF_CSI_FRAME1_READY;
			buf_offset = 1;
		}
	} else {
		//even frame
		if (stream->odd_frame_first) {
			fs_interlaced_phase = CIF_CSI_FRAME1_READY;
			buf_offset = 0;
		} else {
			fs_interlaced_phase = CIF_CSI_FRAME0_READY;
			buf_offset = 0;
		}
	}

	v4l2_dbg(rkcif_debug, 3, &dev->v4l2_dev,
		 "stream[%d] mipi fs interlace phase %d, frame num 0x%x\n",
		 stream->id, fs_interlaced_phase, frame_id);
	if (stream->last_fs_interlaced_phase == 0) {
		buffer = stream->curr_buf;
		rkcif_write_buffer(stream, buffer, CIF_CSI_FRAME0_READY, buf_offset);
		rkcif_write_buffer(stream, buffer, CIF_CSI_FRAME1_READY, buf_offset);
	} else {
		if (fs_interlaced_phase == CIF_CSI_FRAME0_READY) {
			buffer = stream->curr_buf;
			if (stream->last_fs_interlaced_phase == CIF_CSI_FRAME1_READY) {
				if (stream->last_fe_interlaced_phase == CIF_CSI_FRAME0_READY) {
					if (buffer)
						rkcif_buf_queue(&buffer->vb.vb2_buf);
					stream->curr_buf = stream->next_buf;
					stream->next_buf = NULL;
					buffer = stream->curr_buf;
					stream->last_fe_interlaced_phase = CIF_CSI_FRAME1_READY;
					v4l2_dbg(rkcif_debug, 3, &dev->v4l2_dev,
						 "stream[%d] mipi fs interlace bad frame queue, phase %d, line %d\n",
						 stream->id, fs_interlaced_phase, __LINE__);
				}
				rkcif_write_buffer(stream, buffer, CIF_CSI_FRAME0_READY, buf_offset);
				rkcif_write_buffer(stream, buffer, CIF_CSI_FRAME1_READY, buf_offset);
			} else if (stream->last_fs_interlaced_phase == CIF_CSI_FRAME0_READY) {
				stream->interlaced_bad_frame = true;
				rkcif_write_buffer(stream, buffer, CIF_CSI_FRAME0_READY, buf_offset);
				rkcif_write_buffer(stream, buffer, CIF_CSI_FRAME1_READY, buf_offset);
				v4l2_dbg(rkcif_debug, 3, &dev->v4l2_dev,
					 "stream[%d] mipi fs interlace bad frame, phase %d, line %d\n",
					 stream->id, fs_interlaced_phase, __LINE__);
			}
		} else {
			if (stream->last_fs_interlaced_phase == CIF_CSI_FRAME0_READY) {
				if (stream->next_buf) {
					if (stream->curr_buf) {
						rkcif_buf_queue(&stream->curr_buf->vb.vb2_buf);
						stream->curr_buf = stream->next_buf;
						stream->next_buf = NULL;
					}
					stream->interlaced_bad_frame = true;
					v4l2_dbg(rkcif_debug, 3, &dev->v4l2_dev,
						 "stream[%d] mipi fs interlace bad frame, phase %d\n",
						 stream->id, fs_interlaced_phase);
				}
				spin_lock_irqsave(&stream->vbq_lock, flags);
				if (!list_empty(&stream->buf_head)) {
					buffer = list_first_entry(&stream->buf_head,
								  struct rkcif_buffer, queue);
					list_del(&buffer->queue);
					stream->next_buf = buffer;
				}
				spin_unlock_irqrestore(&stream->vbq_lock, flags);
				if (buffer) {
					rkcif_write_buffer(stream, buffer, CIF_CSI_FRAME0_READY, buf_offset);
					rkcif_write_buffer(stream, buffer, CIF_CSI_FRAME1_READY, buf_offset);
				} else if (dummy_buf->vaddr) {
					stream->interlaced_bad_frame = true;
					rkcif_write_buffer(stream, NULL, CIF_CSI_FRAME0_READY, buf_offset);
					rkcif_write_buffer(stream, NULL, CIF_CSI_FRAME1_READY, buf_offset);
					v4l2_dbg(rkcif_debug, 3, &dev->v4l2_dev,
						 "stream[%d] mipi interlace not buf, use dummy buf, line %d\n",
						 stream->id, __LINE__);
				} else {
					v4l2_dbg(rkcif_debug, 3, &dev->v4l2_dev,
						 "stream[%d] mipi interlace not buf, may overwrite buf, line %d\n",
						 stream->id, __LINE__);
				}
			} else {
				if (stream->last_fe_interlaced_phase != CIF_CSI_FRAME1_READY) {
					buffer = stream->curr_buf;
					if (buffer)
						rkcif_buf_queue(&buffer->vb.vb2_buf);
					stream->curr_buf = stream->next_buf;
					stream->next_buf = NULL;
					stream->last_fe_interlaced_phase  = CIF_CSI_FRAME1_READY;
					v4l2_dbg(rkcif_debug, 3, &dev->v4l2_dev,
						 "stream[%d] mipi fs interlace bad frame queue, phase %d, line %d\n",
						 stream->id, fs_interlaced_phase, __LINE__);
				}
				if (stream->next_buf) {
					buffer = stream->next_buf;
					rkcif_write_buffer(stream, buffer, CIF_CSI_FRAME0_READY, buf_offset);
					rkcif_write_buffer(stream, buffer, CIF_CSI_FRAME1_READY, buf_offset);
				} else if (stream->curr_buf) {
					buffer = stream->curr_buf;
					rkcif_write_buffer(stream, buffer, CIF_CSI_FRAME0_READY, buf_offset);
					rkcif_write_buffer(stream, buffer, CIF_CSI_FRAME1_READY, buf_offset);
				} else if (dummy_buf->vaddr) {
					rkcif_write_buffer(stream, NULL, CIF_CSI_FRAME0_READY, buf_offset);
					rkcif_write_buffer(stream, NULL, CIF_CSI_FRAME1_READY, buf_offset);
					stream->interlaced_bad_frame = true;
					v4l2_dbg(rkcif_debug, 3, &dev->v4l2_dev,
						 "stream[%d] mipi interlace not buf, use dummy buf, line %d\n",
						 stream->id, __LINE__);
				} else {
					v4l2_dbg(rkcif_debug, 3, &dev->v4l2_dev,
						 "stream[%d] mipi interlace not buf, may overwrite buf, line %d\n",
						 stream->id, __LINE__);
				}
			}
		}
	}
	stream->last_fs_interlaced_phase = fs_interlaced_phase;
}

static void rkcif_save_other_intstat_with_combine_mode(struct rkcif_device *cif_dev)
{
	struct rkmodule_capture_info *capture_info = &cif_dev->channels[0].capture_info;
	int i = 0;
	int tmp_csi_host = 0;

	if (capture_info->mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
		tmp_csi_host = cif_dev->csi_host_idx;
		for (i = 0; i < capture_info->multi_dev.dev_num - 1; i++) {
			cif_dev->csi_host_idx = capture_info->multi_dev.dev_idx[i];
			cif_dev->other_intstat[i] = rkcif_read_register(cif_dev, CIF_REG_MIPI_LVDS_INTSTAT);
			if (cif_dev->other_intstat[i])
				rkcif_write_register(cif_dev,
						     CIF_REG_MIPI_LVDS_INTSTAT,
						     cif_dev->other_intstat[i]);
		}
		cif_dev->csi_host_idx = tmp_csi_host;
	}
}

static void rkcif_check_one_to_multi_sub_stream_stop_state(struct rkcif_device *cif_dev)
{
	struct rkcif_stream *stream = NULL;
	struct csi_channel_info *channel = &cif_dev->channels[0];
	int i = 0;

	if (channel->capture_info.mode != RKMODULE_ONE_CH_TO_MULTI_ISP)
		return;

	for (i = 1; i < channel->capture_info.one_to_multi.isp_num; i++) {
		stream = &cif_dev->stream[i];
		if (stream->stopping && atomic_read(&stream->sub_stream_buf_cnt) == 0) {
			stream->stopping = false;
			wake_up(&stream->wq_stopped);
		}
	}
}

void rkcif_switch_change(struct rkcif_device *cif_dev, bool is_switch)
{
	if (is_switch) {
		gpiod_direction_output_raw(cif_dev->switch_info.gpio_pin, 1);
	} else {
		gpiod_direction_output_raw(cif_dev->switch_info.gpio_pin, 0);
	}
}

void rkcif_update_unite_extend_pixel(struct rkcif_device *cif_dev)
{
	struct v4l2_subdev *sd = get_rkisp_sd(cif_dev->sditf[0]);

	if (!sd)
		return;
	v4l2_subdev_call(sd, core, ioctl,
			 RKISP_VICAP_CMD_GET_UNITE_EXTEND_PIXEL,
			 &cif_dev->unite_extend_pixel);
}

/* pingpong irq for rk3588 and next */
void rkcif_irq_pingpong_v1(struct rkcif_device *cif_dev)
{
	struct rkcif_stream *stream;
	struct v4l2_mbus_config *mbus;
	struct csi_channel_info *channel = &cif_dev->channels[0];
	unsigned int intstat, i = 0xff;
	unsigned long flags;
	bool is_update = false;
	int ret = 0;
	int on = 0;
	int tmp_csi_host_idx = 0;
	struct rkcif_stream *last_stream = NULL;

	if (!cif_dev->active_sensor)
		return;

	mbus = &cif_dev->active_sensor->mbus;
	if (mbus->type == V4L2_MBUS_CSI2_DPHY ||
	    mbus->type == V4L2_MBUS_CSI2_CPHY ||
	    mbus->type == V4L2_MBUS_CCP2) {
		int mipi_id;
		u32 lastline = 0;

		intstat = rkcif_read_register(cif_dev, CIF_REG_MIPI_LVDS_INTSTAT);
		lastline = rkcif_read_register(cif_dev, CIF_REG_MIPI_LVDS_LINE_LINE_CNT_ID0_1);
		cif_dev->err_state_work.lastline = lastline;
		cif_dev->err_state_work.intstat = intstat;

		rkcif_save_other_intstat_with_combine_mode(cif_dev);
		/* clear all interrupts that has been triggered */
		if (intstat) {
			rkcif_write_register(cif_dev, CIF_REG_MIPI_LVDS_INTSTAT, intstat);
			v4l2_dbg(2, rkcif_debug, &cif_dev->v4l2_dev,
				 "intstat 0x%x\n", intstat);
		} else {
			return;
		}

		if (intstat & CSI_SIZE_ERR) {
			if (cif_dev->chip_id >= CHIP_RK3588_CIF) {
				cif_dev->err_state_work.size_id0 = rkcif_read_register(cif_dev,
					CIF_REG_MIPI_FRAME_SIZE_ID0);
				cif_dev->err_state_work.size_id1 = rkcif_read_register(cif_dev,
					CIF_REG_MIPI_FRAME_SIZE_ID1);
				cif_dev->err_state_work.size_id2 = rkcif_read_register(cif_dev,
					CIF_REG_MIPI_FRAME_SIZE_ID2);
				cif_dev->err_state_work.size_id3 = rkcif_read_register(cif_dev,
					CIF_REG_MIPI_FRAME_SIZE_ID3);
			}
			cif_dev->irq_stats.csi_size_err_cnt++;
			cif_dev->err_state |= RKCIF_ERR_SIZE;
			if (cif_dev->sditf[0] && cif_dev->sditf[0]->mode.rdbk_mode < RKISP_VICAP_RDBK_AIQ)
				return;
			if (cif_dev->channels[0].capture_info.mode == RKMODULE_MULTI_DEV_COMBINE_ONE) {
				tmp_csi_host_idx = cif_dev->csi_host_idx;
				for (i = 0; i < channel->capture_info.multi_dev.dev_num; i++) {
					cif_dev->csi_host_idx = channel->capture_info.multi_dev.dev_idx[i];
					rkcif_write_register_or(cif_dev, CIF_REG_MIPI_LVDS_CTRL, 0x000A0000);
				}
				if (cif_dev->chip_id >= CHIP_RV1103B_CIF) {
					for (i = 0; i < channel->capture_info.multi_dev.dev_num; i++) {
						cif_dev->csi_host_idx = channel->capture_info.multi_dev.dev_idx[i];
						rkcif_write_register_and(cif_dev, CIF_REG_MIPI_LVDS_CTRL, ~0x000f0000);
					}
				}
				cif_dev->csi_host_idx = tmp_csi_host_idx;
			} else {
				rkcif_write_register_or(cif_dev, CIF_REG_MIPI_LVDS_CTRL, 0x000A0000);
				if (cif_dev->chip_id >= CHIP_RV1103B_CIF)
					rkcif_write_register_and(cif_dev, CIF_REG_MIPI_LVDS_CTRL, ~0x000f0000);
			}
			cif_dev->irq_stats.all_err_cnt++;
			return;
		}

		if (intstat & CSI_FIFO_OVERFLOW_V1) {
			cif_dev->irq_stats.csi_overflow_cnt++;
			cif_dev->err_state |= RKCIF_ERR_OVERFLOW;
			cif_dev->irq_stats.all_err_cnt++;
			return;
		}

		if (intstat & CSI_BANDWIDTH_LACK_V1 &&
		    cif_dev->intr_mask & CSI_BANDWIDTH_LACK_V1) {
			cif_dev->irq_stats.csi_bwidth_lack_cnt++;
			cif_dev->err_state |= RKCIF_ERR_BANDWIDTH_LACK;
			if (cif_dev->irq_stats.csi_bwidth_lack_cnt > 10) {
				rkcif_write_register_and(cif_dev, CIF_REG_MIPI_LVDS_INTEN, ~(CSI_BANDWIDTH_LACK_V1));
				cif_dev->intr_mask &= ~(CSI_BANDWIDTH_LACK_V1);
				schedule_delayed_work(&cif_dev->work_deal_err, msecs_to_jiffies(1000));
			}
		}

		if (intstat & CSI_ALL_ERROR_INTEN_V1) {
			cif_dev->irq_stats.all_err_cnt++;
			return;
		}

		for (i = 0; i < RKCIF_MAX_STREAM_MIPI; i++) {
			mipi_id = rkcif_csi_g_mipi_id(&cif_dev->v4l2_dev,
						      intstat);
			if (mipi_id < 0)
				continue;

			if (cif_dev->switch_info.is_use_switch) {
				if (cif_dev->switch_info.is_active) {
					cif_dev->switch_info.is_active = false;
					cif_dev->switch_info.switch_dev->switch_info.is_active = true;
					stream = &cif_dev->stream[mipi_id];
					if (cif_dev->switch_info.switch_dev->stream[0].state == RKCIF_STATE_STREAMING)
						rkcif_switch_change(cif_dev, !!cif_dev->switch_info.switch_dev->switch_info.gpio_val);
				} else {
					cif_dev->switch_info.is_active = true;
					cif_dev->switch_info.switch_dev->switch_info.is_active = false;
					stream = &cif_dev->switch_info.switch_dev->stream[mipi_id];
					if (cif_dev->stream[0].state == RKCIF_STATE_STREAMING)
						rkcif_switch_change(cif_dev, !!cif_dev->switch_info.gpio_val);
				}

			} else {
				stream = &cif_dev->stream[mipi_id];
			}
			if (!cif_dev->sditf[0] ||
			    cif_dev->sditf[0]->mode.rdbk_mode >= RKISP_VICAP_RDBK_AIQ)
				stream->buf_wake_up_cnt++;

			if (stream->stopping && (!stream->dma_en)) {
				rkcif_stream_stop(stream);
				stream->stopping = false;
				wake_up(&stream->wq_stopped);
				continue;
			}

			if (stream->state != RKCIF_STATE_STREAMING)
				continue;
			stream->is_in_vblank = true;
			switch (mipi_id) {
			case RKCIF_STREAM_MIPI_ID0:
				stream->frame_phase = SW_FRM_END_ID0(intstat);
				intstat &= ~CSI_FRAME_END_ID0;
				break;
			case RKCIF_STREAM_MIPI_ID1:
				stream->frame_phase = SW_FRM_END_ID1(intstat);
				intstat &= ~CSI_FRAME_END_ID1;
				break;
			case RKCIF_STREAM_MIPI_ID2:
				stream->frame_phase = SW_FRM_END_ID2(intstat);
				intstat &= ~CSI_FRAME_END_ID2;
				break;
			case RKCIF_STREAM_MIPI_ID3:
				stream->frame_phase = SW_FRM_END_ID3(intstat);
				intstat &= ~CSI_FRAME_END_ID3;
				break;
			}
			if (stream->low_latency)
				rkcif_fence_signal(stream);
			if (stream->cifdev->rdbk_debug &&
			    stream->frame_idx < 15 &&
			    (!cif_dev->sditf[0] || cif_dev->sditf[0]->mode.rdbk_mode >= RKISP_VICAP_RDBK_AIQ))
				v4l2_info(&cif_dev->v4l2_dev,
					  "stream[%d] fe %d, phase %d, %lld\n",
					  stream->id,
					  stream->frame_idx - 1,
					  stream->frame_phase,
					  rkcif_time_get_ns(cif_dev));
			if (stream->is_finish_stop_dma && stream->is_wait_dma_stop) {
				stream->is_wait_dma_stop = false;
				wake_up(&stream->wq_stopped);
				stream->is_finish_stop_dma = false;
				continue;
			}

			if (stream->is_finish_stop_dma && stream->is_wait_stop_complete) {
				stream->is_wait_stop_complete = false;
				stream->is_pause_stream = true;
				complete(&stream->stop_complete);
			}

			if (stream->is_finish_stop_dma)
				stream->is_finish_stop_dma = false;

			if (stream->crop_dyn_en)
				rkcif_dynamic_crop(stream);

			if ((stream->frame_idx - stream->last_frame_idx - 1) != 0) {
				stream->frame_loss += (stream->frame_idx - stream->last_frame_idx - 1);
				v4l2_dbg(3, rkcif_debug, &cif_dev->v4l2_dev,
					 "cur_frame_idx %d, last_frame_idx %d, frame loss %d\n",
					 stream->frame_idx, stream->last_frame_idx, stream->frame_loss);
			}
			if (stream->dma_en & RKCIF_DMAEN_BY_VICAP) {
				if (cif_dev->sync_cfg.type == NO_SYNC_MODE ||
				    cif_dev->sync_cfg.type == SOFT_SYNC_MODE ||
				    !cif_dev->is_detect_group_sync)
					is_update = true;
				else
					is_update = rkcif_check_buffer_prepare(stream);
				v4l2_dbg(4, rkcif_debug, &cif_dev->v4l2_dev,
					 "dma capture by vicap, is_updata %d, group mode %d, dma_en %d\n",
					 is_update, cif_dev->sync_cfg.type, stream->dma_en);
				if (is_update) {
					if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO)
						rkcif_update_stream_interlace(cif_dev, stream, mipi_id);
					else
						rkcif_update_stream(cif_dev, stream, mipi_id);
				}
			} else if (stream->dma_en & RKCIF_DMAEN_BY_ISP) {
				v4l2_dbg(4, rkcif_debug, &cif_dev->v4l2_dev,
					 "dma capture by isp, dma_en 0x%x\n",
					 stream->dma_en);
				if (cif_dev->sync_cfg.type == NO_SYNC_MODE ||
				    cif_dev->sync_cfg.type == SOFT_SYNC_MODE ||
				    !cif_dev->is_detect_group_sync)
					is_update = true;
				else
					is_update = rkcif_check_buffer_prepare(stream);
				if (is_update)
					rkcif_update_stream_toisp(cif_dev, stream, mipi_id);
			} else if (stream->dma_en & RKCIF_DMAEN_BY_ROCKIT) {
				v4l2_dbg(4, rkcif_debug, &cif_dev->v4l2_dev,
					 "dma capture by rockit, dma_en 0x%x\n",
					 stream->dma_en);
				rkcif_update_stream_rockit(cif_dev, stream, mipi_id);
			}
			spin_lock_irqsave(&stream->cifdev->stream_spinlock, flags);
			if (stream->is_single_cap && !stream->cur_skip_frame) {
				stream->is_single_cap = false;
				spin_unlock_irqrestore(&stream->cifdev->stream_spinlock, flags);
				if (cif_dev->switch_info.is_use_switch &&
				    atomic_dec_if_positive(&cif_dev->hw_dev->switch_stream_cnt[cif_dev->switch_info.host_idx])) {
					if (stream->dma_en & RKCIF_DMAEN_BY_ISP)
						stream->dma_en &= ~RKCIF_DMAEN_BY_ISP;
					else if (stream->dma_en & RKCIF_DMAEN_BY_VICAP)
						stream->dma_en &= ~RKCIF_DMAEN_BY_VICAP;
					else if (stream->dma_en & RKCIF_DMAEN_BY_ROCKIT)
						stream->dma_en &= ~RKCIF_DMAEN_BY_ROCKIT;
					atomic_inc(&stream->cifdev->streamoff_cnt);
					v4l2_dbg(4, rkcif_debug, &cif_dev->v4l2_dev,
						 "%s %d, switch stream %d\n", __func__, __LINE__,
						 atomic_read(&cif_dev->hw_dev->switch_stream_cnt[cif_dev->switch_info.host_idx]));
				} else {
					if (stream->dma_en & RKCIF_DMAEN_BY_ISP)
						stream->to_stop_dma = RKCIF_DMAEN_BY_ISP;
					else if (stream->dma_en & RKCIF_DMAEN_BY_VICAP)
						stream->to_stop_dma = RKCIF_DMAEN_BY_VICAP;
					else if (stream->dma_en & RKCIF_DMAEN_BY_ROCKIT)
						stream->to_stop_dma = RKCIF_DMAEN_BY_ROCKIT;
					rkcif_stop_dma_capture(stream);
				}
				if (cif_dev->hdr.hdr_mode == HDR_X2)
					last_stream = &stream->cifdev->stream[1];
				else if (cif_dev->hdr.hdr_mode == HDR_X3)
					last_stream = &stream->cifdev->stream[2];
				else
					last_stream = stream;
				spin_lock_irqsave(&stream->cifdev->stream_spinlock, flags);
				if (!last_stream->is_wait_single_cap) {
					spin_unlock_irqrestore(&stream->cifdev->stream_spinlock, flags);
					if ((last_stream->cifdev->hdr.hdr_mode == NO_HDR && atomic_read(&last_stream->cifdev->streamoff_cnt) == 1) ||
					    (last_stream->cifdev->hdr.hdr_mode == HDR_X2 && atomic_read(&last_stream->cifdev->streamoff_cnt) == 2) ||
					    (last_stream->cifdev->hdr.hdr_mode == HDR_X3 && atomic_read(&last_stream->cifdev->streamoff_cnt) == 3)) {
						if (!cif_dev->switch_info.is_use_switch ||
						    atomic_read(&cif_dev->hw_dev->switch_stream_cnt[cif_dev->switch_info.host_idx]) == 0)
							rkcif_dphy_quick_stream(stream->cifdev, on);
						stream->cifdev->sensor_work.on = 0;
						atomic_inc(&stream->cifdev->sensor_off);
						schedule_work(&stream->cifdev->sensor_work.work);
					}
				} else {
					last_stream->is_wait_single_cap = false;
					cif_dev->resume_mode = RKISP_RTT_MODE_MULTI_FRAME;
					complete(&stream->start_complete);
					spin_unlock_irqrestore(&stream->cifdev->stream_spinlock, flags);
					if (cif_dev->switch_info.is_use_switch)
						atomic_inc(&cif_dev->hw_dev->switch_stream_cnt[cif_dev->switch_info.host_idx]);
				}
			} else if (stream->lack_buf_cnt == 2 && !stream->cur_skip_frame) {
				spin_unlock_irqrestore(&stream->cifdev->stream_spinlock, flags);
				if (stream->dma_en & RKCIF_DMAEN_BY_ISP)
					stream->to_stop_dma = RKCIF_DMAEN_BY_ISP;
				else if (stream->dma_en & RKCIF_DMAEN_BY_VICAP)
					stream->to_stop_dma = RKCIF_DMAEN_BY_VICAP;
				else if (stream->dma_en & RKCIF_DMAEN_BY_ROCKIT)
					stream->to_stop_dma = RKCIF_DMAEN_BY_ROCKIT;
				rkcif_stop_dma_capture(stream);
			} else {
				spin_unlock_irqrestore(&stream->cifdev->stream_spinlock, flags);
			}

			if (cif_dev->chip_id >= CHIP_RV1106_CIF)
				rkcif_modify_frame_skip_config(stream);
			if (stream->is_change_toisp) {
				stream->is_change_toisp = false;
				if (cif_dev->hdr.hdr_mode == NO_HDR ||
				    (cif_dev->hdr.hdr_mode == HDR_X2 && stream->id == 1) ||
				    (cif_dev->hdr.hdr_mode == HDR_X3 && stream->id == 2))
					sditf_change_to_online(cif_dev->sditf[0]);
				rkcif_modify_line_int(stream, false);
				stream->is_line_inten = false;
			}

			if (stream->cur_skip_frame)
				stream->cur_skip_frame--;
			if (stream->to_en_scale) {
				stream->to_en_scale = false;
				rkcif_scale_start(stream->scale_vdev);
			}
			rkcif_detect_wake_up_mode_change(stream);
			if (mipi_id == RKCIF_STREAM_MIPI_ID0) {
				if (stream->fs_cnt_in_single_frame > 1) {
					if (cif_dev->chip_id < CHIP_RK3588_CIF) {
						cif_dev->err_state |= RKCIF_ERR_ID0_MULTI_FS;
						stream->is_fs_fe_not_paired = true;
					}
					stream->fs_cnt_in_single_frame = 0;
				} else {
					stream->fs_cnt_in_single_frame--;
				}
			}
			rkcif_monitor_reset_event(cif_dev);
			cif_dev->irq_stats.frm_end_cnt[stream->id]++;
			rkcif_check_one_to_multi_sub_stream_stop_state(cif_dev);
		}
		for (i = 0; i < RKCIF_MAX_STREAM_MIPI; i++) {
			if (intstat & (cif_dev->chip_id < CHIP_RK3576_CIF ?
			    CSI_START_INTSTAT(i) : CSI_START_INTSTAT_RK3576(i))) {
				if (cif_dev->switch_info.is_use_switch) {
					if (cif_dev->switch_info.is_active)
						stream = &cif_dev->stream[i];
					else
						stream = &cif_dev->switch_info.switch_dev->stream[i];
				} else {
					stream = &cif_dev->stream[i];
				}
				if (stream->state != RKCIF_STATE_STREAMING)
					continue;
				spin_lock_irqsave(&stream->vbq_lock, flags);
				if (stream->stopping && stream->dma_en) {
					if (stream->dma_en & RKCIF_DMAEN_BY_VICAP)
						stream->to_stop_dma = RKCIF_DMAEN_BY_VICAP;
					else if (stream->dma_en & RKCIF_DMAEN_BY_ISP)
						stream->to_stop_dma = RKCIF_DMAEN_BY_ISP;
					stream->is_stop_capture = true;
				}
				if (stream->to_stop_dma) {
					if (cif_dev->switch_info.is_use_switch &&
					    atomic_dec_if_positive(&cif_dev->hw_dev->switch_stream_cnt[cif_dev->switch_info.host_idx])) {
						stream->to_stop_dma = 0;
						stream->is_finish_stop_dma = true;
					} else {
						ret = rkcif_stop_dma_capture(stream);
						if (!ret) {
							stream->is_finish_stop_dma = true;
							if (stream->is_wait_stop_complete)
								stream->is_pause_stream = true;
						}
					}
				}
				spin_unlock_irqrestore(&stream->vbq_lock, flags);

				if (stream->dma_en || cif_dev->chip_id >= CHIP_RK3576_CIF) {
					if (i == 0) {
						spin_lock_irqsave(&stream->vbq_lock, flags);
						if (!stream->cur_skip_frame && (!cif_dev->is_in_flip)) {
							spin_unlock_irqrestore(&stream->vbq_lock, flags);
							rkcif_deal_sof(stream->cifdev);
						} else {
							spin_unlock_irqrestore(&stream->vbq_lock, flags);
						}
					} else {
						spin_lock_irqsave(&stream->fps_lock, flags);
						stream->readout.fs_timestamp = rkcif_time_get_ns(cif_dev);
						if (cif_dev->hdr.hdr_mode == HDR_X2 || cif_dev->hdr.hdr_mode == HDR_X3)
							stream->frame_idx = cif_dev->stream[0].frame_idx;
						else
							stream->frame_idx++;
						if (cif_dev->channels[0].capture_info.mode == RKMODULE_MULTI_CH_TO_MULTI_ISP &&
						    cif_dev->sditf[stream->id])
							sditf_event_inc_sof(cif_dev->sditf[stream->id]);
						spin_unlock_irqrestore(&stream->fps_lock, flags);
					}
				}
				stream->is_in_vblank = false;
				if (stream->to_en_dma)
					rkcif_enable_dma_capture(stream, false);
				if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO)
					rkcif_check_mipi_interlaced_frame_id(stream);
			}
			if (intstat & CSI_LINE_INTSTAT_V1(i)) {
				if (cif_dev->switch_info.is_use_switch) {
					if (cif_dev->switch_info.is_active)
						stream = &cif_dev->stream[i];
					else
						stream = &cif_dev->switch_info.switch_dev->stream[i];
				} else {
					stream = &cif_dev->stream[i];
				}
				if (stream->state != RKCIF_STATE_STREAMING)
					continue;
				if (stream->is_line_inten) {
					stream->line_int_cnt++;
					if (cif_dev->rdbk_debug > 1 &&
					    stream->frame_idx < 15)
						v4l2_info(&cif_dev->v4l2_dev,
							  "line int %lld\n",
							  stream->line_int_cnt);
					if (cif_dev->sditf[0] && (cif_dev->sditf[0]->mode.rdbk_mode == RKISP_VICAP_RDBK_AUTO ||
					    cif_dev->sditf[0]->mode.rdbk_mode == RKISP_VICAP_RDBK_AUTO_ONE_FRAME))
						rkcif_line_wake_up_rdbk(stream, stream->id);
					else if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO)
						rkcif_line_wake_up_interlace(stream, stream->id);
					else
						rkcif_line_wake_up(stream, stream->id);
				}
				v4l2_dbg(3, rkcif_debug, &cif_dev->v4l2_dev,
					 "%s: id0 cur line:%d\n", __func__, lastline & 0x3fff);
			}
		}
	} else {
		struct rkcif_stream *stream;
		int ch_id;
		int lastline;

		intstat = rkcif_read_register(cif_dev, CIF_REG_DVP_INTSTAT);
		if (intstat)
			rkcif_write_register(cif_dev, CIF_REG_DVP_INTSTAT, intstat);
		else
			return;
		if (cif_dev->chip_id < CHIP_RV1126B_CIF) {
			lastline = rkcif_read_register(cif_dev, CIF_REG_DVP_LINE_CNT);
			cif_dev->err_state_work.lastline = lastline;
		}
		cif_dev->err_state_work.intstat = intstat;
		stream = &cif_dev->stream[RKCIF_STREAM_CIF];

		if (intstat & DVP_SIZE_ERR) {
			cif_dev->irq_stats.dvp_size_err_cnt++;
			rkcif_write_register_or(cif_dev, CIF_REG_DVP_CTRL, 0x000A0000);
			if (cif_dev->chip_id >= CHIP_RV1103B_CIF)
				rkcif_write_register_and(cif_dev, CIF_REG_DVP_CTRL, ~0x000f0000);
			cif_dev->err_state |= RKCIF_ERR_SIZE;
			if (cif_dev->chip_id >= CHIP_RV1126B_CIF) {
				cif_dev->err_state_work.size_id0 = rkcif_read_register(cif_dev,
					CIF_REG_DVP_FRAME_NUM_ID0);
				cif_dev->err_state_work.size_id1 = rkcif_read_register(cif_dev,
					CIF_REG_DVP_FRAME_NUM_ID1);
				cif_dev->err_state_work.size_id2 = rkcif_read_register(cif_dev,
					CIF_REG_DVP_FRAME_NUM_ID2);
				cif_dev->err_state_work.size_id3 = rkcif_read_register(cif_dev,
					CIF_REG_DVP_FRAME_NUM_ID3);
			}
		}

		if (intstat & DVP_FIFO_OVERFLOW) {
			cif_dev->irq_stats.dvp_overflow_cnt++;
			cif_dev->err_state |= RKCIF_ERR_OVERFLOW;
		}

		if (intstat & DVP_BANDWIDTH_LACK) {
			cif_dev->irq_stats.dvp_bwidth_lack_cnt++;
			cif_dev->err_state |= RKCIF_ERR_BANDWIDTH_LACK;
		}

		if (intstat & INTSTAT_ERR_RK3588) {
			cif_dev->irq_stats.all_err_cnt++;
			return;
		}

		for (i = 0; i < RKCIF_MAX_STREAM_DVP; i++) {
			ch_id = rkcif_dvp_g_ch_id_by_fe(&cif_dev->v4l2_dev, intstat);

			if (ch_id < 0)
				continue;

			stream = &cif_dev->stream[ch_id];
			if (!cif_dev->sditf[0] ||
			    cif_dev->sditf[0]->mode.rdbk_mode >= RKISP_VICAP_RDBK_AIQ)
				stream->buf_wake_up_cnt++;

			if (stream->stopping) {
				rkcif_stream_stop(stream);
				stream->stopping = false;
				wake_up(&stream->wq_stopped);
				continue;
			}

			if (stream->state != RKCIF_STATE_STREAMING)
				continue;
			stream->is_in_vblank = true;
			switch (ch_id) {
			case RKCIF_STREAM_MIPI_ID0:
				stream->frame_phase = SW_FRM_END_ID0(intstat);
				intstat &= ~DVP_ALL_END_ID0;
				break;
			case RKCIF_STREAM_MIPI_ID1:
				stream->frame_phase = SW_FRM_END_ID1(intstat);
				intstat &= ~DVP_ALL_END_ID1;
				break;
			case RKCIF_STREAM_MIPI_ID2:
				stream->frame_phase = SW_FRM_END_ID2(intstat);
				intstat &= ~DVP_ALL_END_ID2;
				break;
			case RKCIF_STREAM_MIPI_ID3:
				stream->frame_phase = SW_FRM_END_ID3(intstat);
				intstat &= ~DVP_ALL_END_ID3;
				break;
			}
			if (stream->dma_en & RKCIF_DMAEN_BY_VICAP) {
				if (cif_dev->sync_cfg.type == NO_SYNC_MODE ||
				    cif_dev->sync_cfg.type == SOFT_SYNC_MODE)
					is_update = true;
				else
					is_update = rkcif_check_buffer_prepare(stream);
				if (is_update)
					rkcif_update_stream(cif_dev, stream, ch_id);
			} else if (stream->dma_en & RKCIF_DMAEN_BY_ISP) {
				rkcif_update_stream_toisp(cif_dev, stream, ch_id);
			} else if (stream->dma_en & RKCIF_DMAEN_BY_ROCKIT) {
				rkcif_update_stream_rockit(cif_dev, stream, ch_id);
			}

			if (stream->to_en_dma)
				rkcif_enable_dma_capture(stream, false);
			if (stream->to_stop_dma) {
				rkcif_stop_dma_capture(stream);
				wake_up(&stream->wq_stopped);
			}
			cif_dev->irq_stats.frm_end_cnt[stream->id]++;
		}

		for (i = 0; i < RKCIF_STREAM_DVP; i++) {
			if (intstat & (cif_dev->chip_id < CHIP_RK3576_CIF ?
			    DVP_START_INTSTAT(i) : DVP_START_INTSTAT_RK3576(i))) {
				stream = &cif_dev->stream[i];
				if (i == 0) {
					if (!stream->cur_skip_frame)
						rkcif_deal_sof(cif_dev);
				} else {
					spin_lock_irqsave(&stream->fps_lock, flags);
					stream->readout.fs_timestamp = rkcif_time_get_ns(cif_dev);
					stream->frame_idx++;
					spin_unlock_irqrestore(&stream->fps_lock, flags);
				}
				stream->is_in_vblank = false;
			}
		}

		if (stream->crop_dyn_en)
			rkcif_dynamic_crop(stream);
	}
}

void rkcif_irq_pingpong(struct rkcif_device *cif_dev)
{
	struct rkcif_stream *stream;
	struct rkcif_stream *detect_stream = &cif_dev->stream[0];
	struct v4l2_mbus_config *mbus;
	unsigned int intstat = 0x0, i = 0xff;
	unsigned long flags;
	int ret = 0;

	if (!cif_dev->active_sensor)
		return;

	mbus = &cif_dev->active_sensor->mbus;
	if ((mbus->type == V4L2_MBUS_CSI2_DPHY ||
	     mbus->type == V4L2_MBUS_CSI2_CPHY ||
	     mbus->type == V4L2_MBUS_CCP2) &&
	    (cif_dev->chip_id == CHIP_RK1808_CIF ||
	     cif_dev->chip_id == CHIP_RV1126_CIF ||
	     cif_dev->chip_id == CHIP_RK3568_CIF)) {
		int mipi_id;
		u32 lastline = 0;

		intstat = rkcif_read_register(cif_dev, CIF_REG_MIPI_LVDS_INTSTAT);
		lastline = rkcif_read_register(cif_dev, CIF_REG_MIPI_LVDS_LINE_LINE_CNT_ID0_1);
		cif_dev->err_state_work.lastline = lastline;
		cif_dev->err_state_work.intstat = intstat;

		/* clear all interrupts that has been triggered */
		rkcif_write_register(cif_dev, CIF_REG_MIPI_LVDS_INTSTAT, intstat);

		if (intstat & CSI_FIFO_OVERFLOW) {
			cif_dev->irq_stats.csi_overflow_cnt++;
			cif_dev->err_state |= RKCIF_ERR_OVERFLOW;
		}

		if (intstat & CSI_BANDWIDTH_LACK) {
			cif_dev->irq_stats.csi_bwidth_lack_cnt++;
			cif_dev->err_state |= RKCIF_ERR_BANDWIDTH_LACK;
			return;
		}

		if (intstat & CSI_ALL_ERROR_INTEN) {
			cif_dev->irq_stats.all_err_cnt++;
			return;
		}

		for (i = 0; i < RKCIF_MAX_STREAM_MIPI; i++) {
			mipi_id = rkcif_csi_g_mipi_id(&cif_dev->v4l2_dev,
						      intstat);
			if (mipi_id < 0)
				continue;

			stream = &cif_dev->stream[mipi_id];
			if (stream->stopping && stream->is_can_stop) {
				rkcif_stream_stop(stream);
				stream->stopping = false;
				wake_up(&stream->wq_stopped);
				continue;
			}
			stream->buf_wake_up_cnt++;

			if (stream->state != RKCIF_STATE_STREAMING)
				continue;

			switch (mipi_id) {
			case RKCIF_STREAM_MIPI_ID0:
				stream->frame_phase = SW_FRM_END_ID0(intstat);
				intstat &= ~CSI_FRAME_END_ID0;
				break;
			case RKCIF_STREAM_MIPI_ID1:
				stream->frame_phase = SW_FRM_END_ID1(intstat);
				intstat &= ~CSI_FRAME_END_ID1;
				break;
			case RKCIF_STREAM_MIPI_ID2:
				stream->frame_phase = SW_FRM_END_ID2(intstat);
				intstat &= ~CSI_FRAME_END_ID2;
				break;
			case RKCIF_STREAM_MIPI_ID3:
				stream->frame_phase = SW_FRM_END_ID3(intstat);
				intstat &= ~CSI_FRAME_END_ID3;
				break;
			}

			if (stream->low_latency)
				rkcif_fence_signal(stream);

			if (stream->crop_dyn_en)
				rkcif_dynamic_crop(stream);

			if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO)
				rkcif_update_stream_interlace(cif_dev, stream, mipi_id);
			else
				rkcif_update_stream(cif_dev, stream, mipi_id);
			rkcif_detect_wake_up_mode_change(stream);
			rkcif_monitor_reset_event(cif_dev);
			if (mipi_id == RKCIF_STREAM_MIPI_ID0) {
				if ((intstat & (CSI_FRAME1_START_ID0 | CSI_FRAME0_START_ID0)) == 0 &&
				    detect_stream->fs_cnt_in_single_frame > 1) {
					cif_dev->err_state |= RKCIF_ERR_ID0_MULTI_FS;
					detect_stream->is_fs_fe_not_paired = true;
					detect_stream->fs_cnt_in_single_frame = 0;
				} else {
					detect_stream->fs_cnt_in_single_frame--;
				}
			}
			cif_dev->irq_stats.frm_end_cnt[stream->id]++;
			rkcif_check_one_to_multi_sub_stream_stop_state(cif_dev);
		}
		for (i = 0; i < RKCIF_MAX_STREAM_MIPI; i++) {
			if (intstat & CSI_START_INTSTAT(i)) {
				stream = &cif_dev->stream[i];
				if (i == 0) {
					if (!stream->cur_skip_frame)
						rkcif_deal_sof(cif_dev);
				} else {
					spin_lock_irqsave(&stream->fps_lock, flags);
					stream->readout.fs_timestamp = rkcif_time_get_ns(cif_dev);
					stream->frame_idx++;
					if (cif_dev->channels[0].capture_info.mode == RKMODULE_MULTI_CH_TO_MULTI_ISP &&
					    cif_dev->sditf[stream->id])
						sditf_event_inc_sof(cif_dev->sditf[stream->id]);
					spin_unlock_irqrestore(&stream->fps_lock, flags);
				}
				stream->is_in_vblank = false;
				if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO)
					rkcif_check_mipi_interlaced_frame_id(stream);
			}
			if (intstat & CSI_LINE_INTSTAT(i)) {
				stream = &cif_dev->stream[i];
				if (stream->is_line_inten) {
					stream->line_int_cnt++;
					if (rkcif_get_interlace_mode(stream) == RKCIF_INTERLACE_SOFT_AUTO)
						rkcif_line_wake_up_interlace(stream, stream->id);
					else
						rkcif_line_wake_up(stream, stream->id);
					rkcif_modify_line_int(stream, false);
					stream->is_line_inten = false;
				}
				v4l2_dbg(3, rkcif_debug, &cif_dev->v4l2_dev,
					 "%s: id0 cur line:%d\n", __func__, lastline & 0x3fff);
			}
		}
	} else {
		u32 lastline, lastpix, ctl;
		u32 cif_frmst, frmid, int_en;
		struct rkcif_stream *stream;
		int ch_id;

		intstat = rkcif_read_register(cif_dev, CIF_REG_DVP_INTSTAT);
		cif_frmst = rkcif_read_register(cif_dev, CIF_REG_DVP_FRAME_STATUS);
		lastline = rkcif_read_register(cif_dev, CIF_REG_DVP_LAST_LINE);
		lastline = CIF_FETCH_Y_LAST_LINE(lastline);
		lastpix = rkcif_read_register(cif_dev, CIF_REG_DVP_LAST_PIX);
		lastpix =  CIF_FETCH_Y_LAST_LINE(lastpix);
		ctl = rkcif_read_register(cif_dev, CIF_REG_DVP_CTRL);
		cif_dev->err_state_work.lastline = lastline;
		cif_dev->err_state_work.lastpixel = lastpix;
		cif_dev->err_state_work.intstat = intstat;

		rkcif_write_register(cif_dev, CIF_REG_DVP_INTSTAT, intstat);

		stream = &cif_dev->stream[RKCIF_STREAM_CIF];

		if (intstat & BUS_ERR) {
			cif_dev->irq_stats.dvp_bus_err_cnt++;
			cif_dev->err_state |= RKCIF_ERR_BUS;
		}

		if (intstat & DVP_ALL_OVERFLOW) {
			cif_dev->irq_stats.dvp_overflow_cnt++;
			cif_dev->err_state |= RKCIF_ERR_OVERFLOW;
		}

		if (intstat & LINE_ERR) {
			cif_dev->irq_stats.dvp_line_err_cnt++;
			cif_dev->err_state |= RKCIF_ERR_LINE;
		}

		if (intstat & PIX_ERR) {
			cif_dev->irq_stats.dvp_pix_err_cnt++;
			cif_dev->err_state |= RKCIF_ERR_PIXEL;
		}

		if (intstat & INTSTAT_ERR)
			cif_dev->irq_stats.all_err_cnt++;

		/* There are two irqs enabled:
		 *  - PST_INF_FRAME_END: cif FIFO is ready,
		 *    this is prior to FRAME_END
		 *  - FRAME_END: cif has saved frame to memory,
		 *    a frame ready
		 */
		if ((intstat & PST_INF_FRAME_END)) {
			stream = &cif_dev->stream[RKCIF_STREAM_CIF];
			if (stream->stopping)
				/* To stop CIF ASAP, before FRAME_END irq */
				rkcif_write_register(cif_dev, CIF_REG_DVP_CTRL,
						     ctl & (~ENABLE_CAPTURE));
		}

		if (cif_dev->chip_id <= CHIP_RK1808_CIF) {

			stream = &cif_dev->stream[RKCIF_STREAM_CIF];

			if ((intstat & FRAME_END)) {
				struct rkcif_buffer *active_buf = NULL;

				int_en = rkcif_read_register(cif_dev, CIF_REG_DVP_INTEN);
				int_en |= LINE_INT_EN;
				rkcif_write_register(cif_dev, CIF_REG_DVP_INTEN, int_en);
				cif_dev->dvp_sof_in_oneframe = 0;

				if (stream->stopping) {
					rkcif_stream_stop(stream);
					stream->stopping = false;
					rkcif_assign_dummy_buffer(stream);
					wake_up(&stream->wq_stopped);
					return;
				}
				stream->buf_wake_up_cnt++;

				frmid = CIF_GET_FRAME_ID(cif_frmst);
				if ((cif_frmst == 0xfffd0002) || (cif_frmst == 0xfffe0002)) {
					v4l2_info(&cif_dev->v4l2_dev, "frmid:%d, frmstat:0x%x\n",
						  frmid, cif_frmst);
					rkcif_write_register(cif_dev, CIF_REG_DVP_FRAME_STATUS,
							     FRAME_STAT_CLS);
				}

				if (lastline != stream->pixm.height ||
				    (!(cif_frmst & CIF_F0_READY) &&
				     !(cif_frmst & CIF_F1_READY))) {

					cif_dev->dvp_sof_in_oneframe = 1;
					v4l2_err(&cif_dev->v4l2_dev,
						 "Bad frame, pp irq:0x%x frmst:0x%x size:%dx%d\n",
						 intstat, cif_frmst, lastpix, lastline);
					return;
				}

				if (cif_frmst & CIF_F0_READY) {
					if (stream->curr_buf)
						active_buf = stream->curr_buf;
					stream->frame_phase = CIF_CSI_FRAME0_READY;
				} else if (cif_frmst & CIF_F1_READY) {
					if (stream->next_buf)
						active_buf = stream->next_buf;
					stream->frame_phase = CIF_CSI_FRAME1_READY;
				}

				spin_lock_irqsave(&stream->fps_lock, flags);
				if (stream->frame_phase & CIF_CSI_FRAME0_READY)
					stream->fps_stats.frm0_timestamp = rkcif_time_get_ns(cif_dev);
				else if (stream->frame_phase & CIF_CSI_FRAME1_READY)
					stream->fps_stats.frm1_timestamp = rkcif_time_get_ns(cif_dev);
				spin_unlock_irqrestore(&stream->fps_lock, flags);

				ret = rkcif_assign_new_buffer_oneframe(stream,
								 RKCIF_YUV_ADDR_STATE_UPDATE);

				if (active_buf && (!ret)) {
					active_buf->vb.sequence = stream->frame_idx;
					rkcif_vb_done_tasklet(stream, active_buf);
				}
				stream->frame_idx++;
				cif_dev->irq_stats.frm_end_cnt[stream->id]++;
			}
		} else {
			for (i = 0; i < RKCIF_MAX_STREAM_DVP; i++) {
				ch_id = rkcif_dvp_g_ch_id(&cif_dev->v4l2_dev, &intstat, cif_frmst);

				if (ch_id < 0)
					continue;

				if (ch_id == RKCIF_STREAM_MIPI_ID0) {
					int_en = rkcif_read_register(cif_dev, CIF_REG_DVP_INTEN);
					int_en |= LINE_INT_EN;
					rkcif_write_register(cif_dev, CIF_REG_DVP_INTEN, int_en);
					cif_dev->dvp_sof_in_oneframe = 0;
				}

				stream = &cif_dev->stream[ch_id];

				if (stream->stopping) {
					rkcif_stream_stop(stream);
					stream->stopping = false;
					wake_up(&stream->wq_stopped);
					continue;
				}
				stream->buf_wake_up_cnt++;

				if (stream->state != RKCIF_STATE_STREAMING)
					continue;

				switch (ch_id) {
				case RKCIF_STREAM_MIPI_ID0:
					stream->frame_phase = DVP_FRM_STS_ID0(cif_frmst);
					break;
				case RKCIF_STREAM_MIPI_ID1:
					stream->frame_phase = DVP_FRM_STS_ID1(cif_frmst);
					break;
				case RKCIF_STREAM_MIPI_ID2:
					stream->frame_phase = DVP_FRM_STS_ID2(cif_frmst);
					break;
				case RKCIF_STREAM_MIPI_ID3:
					stream->frame_phase = DVP_FRM_STS_ID3(cif_frmst);
					break;
				}


				frmid = CIF_GET_FRAME_ID(cif_frmst);
				if ((frmid == 0xfffd) || (frmid == 0xfffe)) {
					v4l2_info(&cif_dev->v4l2_dev, "frmid:%d, frmstat:0x%x\n",
						  frmid, cif_frmst);
					rkcif_write_register(cif_dev, CIF_REG_DVP_FRAME_STATUS,
							     FRAME_STAT_CLS);
				}
				rkcif_update_stream(cif_dev, stream, ch_id);
				cif_dev->irq_stats.frm_end_cnt[stream->id]++;
			}
		}

		if ((intstat & LINE_INT_END) && !(intstat & FRAME_END) &&
		    (cif_dev->dvp_sof_in_oneframe == 0)) {
			if ((intstat & (PRE_INF_FRAME_END | PST_INF_FRAME_END)) == 0x0) {
				if ((intstat & INTSTAT_ERR) == 0x0) {
					if (!stream->cur_skip_frame)
						rkcif_deal_sof(cif_dev);
					int_en = rkcif_read_register(cif_dev, CIF_REG_DVP_INTEN);
					int_en &= ~LINE_INT_EN;
					rkcif_write_register(cif_dev, CIF_REG_DVP_INTEN, int_en);
					cif_dev->dvp_sof_in_oneframe = 1;
				}
			}
		}

		if (stream->crop_dyn_en)
			rkcif_dynamic_crop(stream);
	}
}

void rkcif_irq_lite_lvds(struct rkcif_device *cif_dev)
{
	struct rkcif_stream *stream;
	struct v4l2_mbus_config *mbus = &cif_dev->active_sensor->mbus;
	unsigned int intstat, i = 0xff;

	if (mbus->type == V4L2_MBUS_CCP2 &&
	    cif_dev->chip_id == CHIP_RV1126_CIF_LITE) {
		int mipi_id;
		u32 lastline = 0;

		intstat = rkcif_read_register(cif_dev, CIF_REG_MIPI_LVDS_INTSTAT);
		lastline = rkcif_read_register(cif_dev, CIF_REG_MIPI_LVDS_LINE_INT_NUM_ID0_1);
		cif_dev->err_state_work.lastline = lastline;
		cif_dev->err_state_work.intstat = intstat;

		/* clear all interrupts that has been triggered */
		rkcif_write_register(cif_dev, CIF_REG_MIPI_LVDS_INTSTAT, intstat);

		if (intstat & CSI_FIFO_OVERFLOW) {
			v4l2_err(&cif_dev->v4l2_dev,
				 "ERROR: cif lite lvds fifo overflow, intstat:0x%x, lastline:%d!!\n",
				  intstat, lastline);
			return;
		}

		if (intstat & CSI_BANDWIDTH_LACK) {
			v4l2_err(&cif_dev->v4l2_dev,
				 "ERROR: cif lite lvds bandwidth lack, intstat:0x%x!!\n",
				 intstat);
			return;
		}

		if (intstat & CSI_ALL_ERROR_INTEN) {
			v4l2_err(&cif_dev->v4l2_dev,
				 "ERROR: cif lite lvds all err:0x%x!!\n", intstat);
			return;
		}

		for (i = 0; i < RKCIF_MAX_STREAM_MIPI; i++) {
			mipi_id = rkcif_csi_g_mipi_id(&cif_dev->v4l2_dev,
						      intstat);
			if (mipi_id < 0)
				continue;

			stream = &cif_dev->stream[mipi_id];

			if (stream->stopping) {
				rkcif_stream_stop(stream);
				stream->stopping = false;
				wake_up(&stream->wq_stopped);
				continue;
			}

			if (stream->state != RKCIF_STATE_STREAMING)
				continue;

			stream->buf_wake_up_cnt++;
			switch (mipi_id) {
			case RKCIF_STREAM_MIPI_ID0:
				stream->frame_phase = SW_FRM_END_ID0(intstat);
				intstat &= ~CSI_FRAME_END_ID0;
				break;
			case RKCIF_STREAM_MIPI_ID1:
				stream->frame_phase = SW_FRM_END_ID1(intstat);
				intstat &= ~CSI_FRAME_END_ID1;
				break;
			case RKCIF_STREAM_MIPI_ID2:
				stream->frame_phase = SW_FRM_END_ID2(intstat);
				intstat &= ~CSI_FRAME_END_ID2;
				break;
			case RKCIF_STREAM_MIPI_ID3:
				stream->frame_phase = SW_FRM_END_ID3(intstat);
				intstat &= ~CSI_FRAME_END_ID3;
				break;
			}

			rkcif_update_stream(cif_dev, stream, mipi_id);
			rkcif_monitor_reset_event(cif_dev);
			cif_dev->irq_stats.frm_end_cnt[stream->id]++;
		}

		if (intstat & CSI_FRAME0_START_ID0)
			rkcif_lvds_event_inc_sof(cif_dev);

		if (intstat & CSI_FRAME1_START_ID0)
			rkcif_lvds_event_inc_sof(cif_dev);
	}
}

int rkcif_sditf_disconnect(struct video_device *vdev)
{
	struct rkcif_vdev_node *vnode = vdev_to_node(vdev);
	struct rkcif_stream *stream = to_rkcif_stream(vnode);
	struct rkcif_device *cifdev = stream->cifdev;
	struct media_link *link;
	int ret;

	link = list_first_entry(&cifdev->sditf[0]->sd.entity.links, struct media_link, list);
	ret = media_entity_setup_link(link, 0);
	if (ret)
		dev_err(cifdev->dev, "failed to disable link of sditf with isp");

	return ret;
}
EXPORT_SYMBOL(rkcif_sditf_disconnect);

void rkcif_external_soft_reset(struct video_device *vdev)
{
	struct rkcif_vdev_node *vnode = NULL;
	struct rkcif_stream *stream = NULL;
	struct rkcif_device *cifdev = NULL;

	if (!vdev)
		return;

	vnode = vdev_to_node(vdev);
	stream = to_rkcif_stream(vnode);
	cifdev = stream->cifdev;
	if (cifdev && cifdev->chip_id >= CHIP_RK3588_CIF)
		rkcif_do_soft_reset(cifdev);
}
EXPORT_SYMBOL(rkcif_external_soft_reset);
