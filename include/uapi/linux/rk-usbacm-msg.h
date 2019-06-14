/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2019 Fuzhou Rockchip Electronics Co., Ltd. */

#ifndef _RKCOPROC_MSG_H
#define _RKCOPROC_MSG_H

#include <linux/videodev2.h>

struct msg_init_s {
	__u32 msg_size; /* unit 4 bytes */
	__u32 mipi_clk; /* bps */
	__u8 mipi_lane;
	__u8 cam_num;
} __attribute__((packed));

/* user definition msg type*/
enum {
	/* private */
	ID_MSG_INIT = _IOWR('N', BASE_VIDIOC_PRIVATE + 0, struct msg_init_s),
};

struct cam_msg_s {
	__u32 type;         /* msg identification */
	__u8 cam_id;        /* camera identification */
	__s8 ret_value;     /* valid when get feedback */
	char msg_entity[0]; /* msg entity, size = _IOC_SIZE(type) */
} __attribute__((packed));
#endif
