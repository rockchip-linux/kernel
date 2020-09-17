/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Tusson <dusong@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __SPI_RK_PREISP_H__
#define __SPI_RK_PREISP_H__

#include <linux/types.h>

#define RKPREISP_VERSION "1.0.15"

#define PREISP_FW_NAME_LEN 128

struct preisp_apb_pkt {
	int32_t  data_len;
	int32_t  addr;
	union {
		int32_t *data;
		char reserved[8];
	} un;
};

enum {
	PREISP_SLEEP_MODE_BYPASS = 0,
	PREISP_SLEEP_MODE_STANDBY,
	PREISP_SLEEP_MODE_MAX
};

#define PREISP_POWER_ON	 _IO('p',   1)
#define PREISP_POWER_OFF	_IO('p',   2)
#define PREISP_REQUEST_SLEEP _IOW('p',  3, int32_t)
#define PREISP_WAKEUP	   _IO('p',   4)
#define PREISP_DOWNLOAD_FW  _IOW('p',  5, char[PREISP_FW_NAME_LEN])
#define PREISP_APB_WRITE	_IOW('p',  6, struct preisp_apb_pkt)
#define PREISP_APB_READ	 _IOR('p',  7, struct preisp_apb_pkt)
#define PREISP_ST_QUERY	 _IOR('p',  8, int32_t)
#define PREISP_IRQ_REQUEST  _IOW('p',  9, int32_t)
#define PREISP_SEND_MSG	 _IOW('p', 11, int32_t)
#define PREISP_QUERY_MSG	_IOR('p', 12, int32_t)
#define PREISP_RECV_MSG	 _IOR('p', 13, int32_t)
#define PREISP_CLIENT_CONNECT	_IOW('p', 15, int32_t)
#define PREISP_CLIENT_DISCONNECT  _IO('p', 16)
#define PREISP_RESET_FW	 _IOW('p',  17, char[PREISP_FW_NAME_LEN])
#define PREISP_CORE_VDD_SET _IOW('p',  18, int32_t)
#define PREISP_CORE_VDD_GET _IOW('p',  19, int32_t)
#define PREISP_AP_DSI_STREAM _IOW('p',  20, int32_t)
#endif
