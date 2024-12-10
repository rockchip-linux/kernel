/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#ifndef __PLAT_RK_VENDOR_STORAGE_H
#define __PLAT_RK_VENDOR_STORAGE_H

#define RSV_ID				0
#define SN_ID				1
#define WIFI_MAC_ID			2
#define LAN_MAC_ID			3
#define BT_MAC_ID			4
#define HDCP_14_HDMI_ID			5
#define HDCP_14_DP_ID			6
#define HDCP_2X_ID			7
#define DRM_KEY_ID			8
#define PLAYREADY_CERT_ID		9
#define ATTENTION_KEY_ID		10
#define PLAYREADY_ROOT_KEY_0_ID		11
#define PLAYREADY_ROOT_KEY_1_ID		12
#define HDCP_14_HDMIRX_ID		13
#define SENSOR_CALIBRATION_ID		14
#define IMEI_ID				15
#define LAN_RGMII_DL_ID			16
#define EINK_VCOM_ID			17

#define VENDOR_HEAD_TAG			0x524B5644
#define FLASH_VENDOR_PART_SIZE		8
#define VENDOR_PART_SIZE		128

struct vendor_item {
	u16  id;
	u16  offset;
	u16  size;
	u16  flag;
};

struct vendor_info {
	u32	tag;
	u32	version;
	u16	next_index;
	u16	item_num;
	u16	free_offset;
	u16	free_size;
	struct	vendor_item item[126]; /* 126 * 8 */
	u8	data[VENDOR_PART_SIZE * 512 - 1024 - 8];
	u32	hash;
	u32	version2;
};

struct flash_vendor_info {
	u32	tag;
	u32	version;
	u16	next_index;
	u16	item_num;
	u16	free_offset;
	u16	free_size;
	struct	vendor_item item[62]; /* 62 * 8 */
	u8	data[FLASH_VENDOR_PART_SIZE * 512 - 512 - 8];
	u32	hash;
	u32	version2;
};

#if IS_REACHABLE(CONFIG_ROCKCHIP_VENDOR_STORAGE)
int rk_vendor_read(u32 id, void *pbuf, u32 size);
int rk_vendor_write(u32 id, void *pbuf, u32 size);
int rk_vendor_register(void *read, void *write);
bool is_rk_vendor_ready(void);
#else
static inline int rk_vendor_read(u32 id, void *pbuf, u32 size)
{
	return -1;
}

static inline int rk_vendor_write(u32 id, void *pbuf, u32 size)
{
	return -1;
}

static inline int rk_vendor_register(void *read, void *write)
{
	return -1;
}

static inline bool is_rk_vendor_ready(void)
{
	return false;
}
#endif

#endif
