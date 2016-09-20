/*
**************************************************************************
 * Rockchip driver for NVP6124b
 * (Based on NEXTCHIP driver for nvp)
 *
 * Copyright 2011 by NEXTCHIP Co., Ltd.
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
**************************************************************************
 */
#ifndef __COAX_PROTOCOL_NVP6124_H__
#define __COAX_PROTOCOL_NVP6124_H__
#define PACKET_MODE	0x0B

#define AHD2_PEL_D0	0x20
#define AHD2_FHD_D0	0x10
#define AHD2_PEL_OUT	0x0C
#define AHD2_PEL_BAUD	0x02
#define AHD2_PEL_LINE	0x07
#define AHD2_PEL_SYNC	0x0D
#define AHD2_PEL_EVEN	0x2F
#define AHD2_FHD_BAUD	0x00
#define AHD2_FHD_LINE	0x03
#define AHD2_FHD_LINES	0x05
#define AHD2_FHD_BYTE	0x0A
#define AHD2_FHD_MODE	0x0B
#define AHD2_FHD_OUT	0x09
#define ACP_CLR		0x3A

#define ACP_CAM_STAT	0x55
#define ACP_REG_WR	0x60
#define ACP_REG_RD	0x61
#define ACP_MODE_ID	0x60

#endif
