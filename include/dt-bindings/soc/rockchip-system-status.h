/*
 *
 * Copyright (C) 2017 ROCKCHIP, Inc.
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

#ifndef _DT_BINDINGS_SOC_ROCKCHIP_SYSTEM_STATUS_H
#define _DT_BINDINGS_SOC_ROCKCHIP_SYSTEM_STATUS_H

#define SYS_STATUS_NORMAL	(1 << 0)
#define SYS_STATUS_SUSPEND	(1 << 1)
#define SYS_STATUS_IDLE		(1 << 2)
#define SYS_STATUS_REBOOT	(1 << 3)
#define SYS_STATUS_VIDEO_4K	(1 << 4)
#define SYS_STATUS_VIDEO_1080P	(1 << 5)
#define SYS_STATUS_GPU		(1 << 6)
#define SYS_STATUS_RGA		(1 << 7)
#define SYS_STATUS_CIF0		(1 << 8)
#define SYS_STATUS_CIF1		(1 << 9)
#define SYS_STATUS_LCDC0	(1 << 10)
#define SYS_STATUS_LCDC1	(1 << 11)
#define SYS_STATUS_BOOST	(1 << 12)
#define SYS_STATUS_PERFORMANCE	(1 << 13)
#define SYS_STATUS_ISP		(1 << 14)
#define SYS_STATUS_HDMI		(1 << 15)
#define SYS_STATUS_VIDEO_4K_10B	(1 << 16)
#define SYS_STATUS_LOW_POWER	(1 << 17)
#define SYS_STATUS_HDMIRX	(1 << 18)
#define SYS_STATUS_VIDEO_SVEP	(1 << 19)
#define SYS_STATUS_VIDEO_4K_60P	(1 << 20)
#define SYS_STATUS_DEEP_SUSPEND	(1 << 21)

#define SYS_STATUS_VIDEO	(SYS_STATUS_VIDEO_4K | \
				 SYS_STATUS_VIDEO_1080P | \
				 SYS_STATUS_VIDEO_4K_10B | \
				 SYS_STATUS_VIDEO_4K_60P)
#define SYS_STATUS_DUALVIEW	(SYS_STATUS_LCDC0 | SYS_STATUS_LCDC1)

#define DMC_FREQ_LEVEL_LOW	(0x1 << 0)
#define DMC_FREQ_LEVEL_MID_LOW	(0x1 << 1)
#define DMC_FREQ_LEVEL_MID_HIGH	(0x1 << 2)
#define DMC_FREQ_LEVEL_HIGH	(0x1 << 3)

#define DMC_WAIT_MODE_NORMAL	(0x1 << 0)
#define DMC_WAIT_MODE_VOP_VBANK	(0x1 << 1)
#define DMC_WAIT_MODE_VOP_LINE	(0x1 << 2)
#define DMC_WAIT_MODE_VOP_AUTO	(0x1 << 3)
#define DMC_WAIT_MODE_ISP_VBANK	(0x1 << 4)

#endif
