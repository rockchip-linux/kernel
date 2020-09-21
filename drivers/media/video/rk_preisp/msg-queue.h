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
#ifndef __RK_PREISP_MSG_H__
#define __RK_PREISP_MSG_H__

#include <linux/types.h>
#include <linux/string.h>
#include "spi2apb.h"

#define DSP_R_MSG_QUEUE_ADDR 0x60050000
#define DSP_S_MSG_QUEUE_ADDR 0x60050010

#define MSG_QUEUE_DEFAULT_SIZE (8 * 1024)

typedef struct msg_queue {
	uint32_t *buf_head; /* msg buffer head */
	uint32_t *buf_tail; /* msg buffer tail */
	uint32_t *cur_send; /* current msg send position */
	uint32_t *cur_recv; /* current msg receive position */
} msg_queue_t;

#define DSP_PMU_SYS_REG0			0x120000f0
#define DSP_MSG_QUEUE_OK_MASK	   0xffff0001
#define DSP_MSG_QUEUE_OK_TAG		0x16080001

typedef struct dsp_msg_queue {
	uint32_t buf_head; //msg buffer head
	uint32_t buf_tail; //msg buffer tail
	uint32_t cur_send; //current msg send position
	uint32_t cur_recv; //current msg receive position
} dsp_msg_queue_t;

typedef struct msg {
	uint32_t size; // unit 4 bytes
	uint16_t type; // msg identification
	int8_t  camera_id;
	int8_t  sync;
} msg_t;

enum {
	id_msg_set_sys_mode_bypass_t = 0x0200,
	id_msg_set_sys_mode_standby_t,

	id_msg_set_log_level_t = 0x0250,

	//dsp -> ap
	id_msg_do_i2c_t = 0x0390,
	//ap -> dsp
	id_msg_do_i2c_ret_t,

	id_msg_dsp_log_t = 0x0400,
};

typedef msg_t msg_set_sys_mode_standby_t;

enum {
	LOG_ERROR,
	LOG_WARN,
	LOG_INFO,
	LOG_DEBUG,
};

typedef struct {
	uint32_t size;
	uint16_t type;
	int8_t  core_id;
	int8_t  log_level;
} msg_dsp_log_t;

typedef msg_dsp_log_t msg_set_log_level_t;

//dsp -> ap
typedef struct {
	uint16_t addr;
	uint16_t flags;
	uint16_t len;
	uint8_t  buf[6];
} do_i2c_msg_t;

typedef struct {
	uint32_t size;
	uint16_t type;
	int16_t  nr;
	uint16_t scl_rate;
	uint16_t num_msg;
} msg_do_i2c_head_t;

#define AP_I2C_ONCE_MAX_NUM 20
typedef struct {
	msg_do_i2c_head_t head;
	do_i2c_msg_t msg[AP_I2C_ONCE_MAX_NUM];
} msg_do_i2c_t;

//ap -> dsp
typedef struct {
	uint32_t size;
	uint16_t type;
	int16_t  nr;
	uint16_t addr;
	uint16_t len;
	uint8_t  buf[8];
} msg_do_i2c_ret_t;

#define MSG(TYPE, var) TYPE var; \
	var.size = sizeof(TYPE) / 4;\
	var.type = id_ ## TYPE;

#define PREISP_IRQ_TYPE_MSG 0x12345678

/**
 * msq_init - Initialize msg queue
 *
 * @q: the msg queue to initialize
 * @size: size of msg queue buf
 *
 * It returns zero on success, else a negative error code.
 */
int msq_init(struct msg_queue *q, int size);

/**
 * msq_release - release msg queue buf
 *
 * @q: the msg queue to release
 */
void msq_release(struct msg_queue *q);

/**
 * msq_is_empty - tests whether a msg queue is empty
 *
 * @q: the msg queue to test
 *
 * It returns true on msg queue is empty, else false.
 */
int msq_is_empty(const struct msg_queue *q);

/**
 * msq_send_msg - send a msg to msg queue
 *
 * @q: msg queue
 * @m: a msg to queue
 *
 * It returns zero on success, else a negative error code.
 */
int msq_send_msg(struct msg_queue *q, const struct msg *m);

/**
 * msq_recv_msg - receive a msg from msg queue
 *
 * @q: msg queue
 * @m: a msg pointer buf [out]
 *
 * need call msq_free_received_msg to free msg after msg use done
 *
 * It returns zero on success, else a negative error code.
 */
int msq_recv_msg(struct msg_queue *q, struct msg **m);

/**
 * msq_free_received_msg - free a received msg
 *
 * @q: msg queue
 * @m: a msg
 *
 * It returns zero on success, else a negative error code.
 */
int msq_free_received_msg(struct msg_queue *q, const struct msg *m);

/**
 * dsp_msq_init - init AP <-> DSP msg queue
 *
 * @spi: spi device
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_init(struct spi_device *spi);

/**
 * dsp_msq_read_head - read dsp msg queue head
 *
 * @spi: spi device
 * @addr: msg queue head addr
 * @m: msg queue pointer
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_read_head(struct spi_device *spi,
		uint32_t addr, struct dsp_msg_queue *q);

/**
 * dsp_msq_send_msg - send a msg to AP -> DSP msg queue
 *
 * @spi: spi device
 * @m: a msg to send
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_send_msg(struct spi_device *spi, const struct msg *m);

/**
 * dsp_msq_recv_query - query next msg size from DSP -> AP msg queue
 *
 * @q: msg queue
 * @size: msg size buf [out]
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_recv_query(struct spi_device *spi, int32_t *size);

/**
 * dsp_msq_recv_msg - receive a msg from DSP -> AP msg queue
 *
 * @q: msg queue
 * @m: a msg pointer buf [out]
 *
 * need call dsp_msq_free_received_msg to free msg after msg use done
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_recv_msg(struct spi_device *spi, struct msg **m);

/**
 * dsp_msq_free_received_msg - free a received msg
 *
 * @q: msg queue
 * @m: a msg
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_free_received_msg(const struct msg *m);

#endif
