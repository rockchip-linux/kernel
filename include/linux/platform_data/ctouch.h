/*
 * include/linux/platform_data/ctouch.h
 *
 * Copyright (C) Guangzhou FriendlyARM Computer Tech. Co., Ltd.
 * (http://www.friendlyarm.com)
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

#ifndef __PLAT_CTOUCH_H__
#define __PLAT_CTOUCH_H__

enum {
	CTP_NONE = 0,
	CTP_AUTO,
	CTP_FT5X06,
	CTP_FT5526_KR,
	CTP_ITE7260,
	CTP_GOODIX,
	CTP_HIMAX,
	CTP_MAX
};

#if defined(CONFIG_DRM_PANEL_FRIENDLYELEC)
extern unsigned int panel_get_touch_id(void);
extern void panel_set_touch_id(int type);

#else

/* Stubs */
#define panel_get_touch_id()		(CTP_AUTO)
#define panel_set_touch_id(type)	\
	do { } while (0)

#endif

#endif	// __PLAT_CTOUCH_H__

