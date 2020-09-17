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

#include "msg-queue.h"

/**
 * msq_init - Initialize msg queue
 *
 * @q: the msg queue to initialize
 * @size: size of msg queue buf
 *
 * It returns zero on success, else a negative error code.
 */
int msq_init(struct msg_queue *q, int size)
{
	uint32_t *buf = kmalloc(size, GFP_KERNEL);

	q->buf_head = buf;
	q->buf_tail = buf + size / sizeof(uint32_t);
	q->cur_send = buf;
	q->cur_recv = buf;

	return 0;
}

/**
 * msq_release - release msg queue buf
 *
 * @q: the msg queue to release
 */
void msq_release(struct msg_queue *q)
{
	kfree(q->buf_head);
	q->buf_head = NULL;
	q->buf_tail = NULL;
	q->cur_send = NULL;
	q->cur_recv = NULL;
}

/**
 * msq_is_empty - tests whether a msg queue is empty
 *
 * @q: the msg queue to test
 *
 * It returns true on msg queue is empty, else false.
 */
int msq_is_empty(const struct msg_queue *q)
{
	return q->cur_send == q->cur_recv;
}

/**
 * msq_tail_free_size - get msg queue tail unused buf size
 *
 * @q: msg queue
 *
 * It returns size of msg queue tail unused buf size, unit 4 bytes
 */
static uint32_t msq_tail_free_size(const struct msg_queue *q)
{
	if (q->cur_send >= q->cur_recv)
		return (q->buf_tail - q->cur_send);

	return q->cur_recv - q->cur_send;
}

/**
 * msq_head_free_size - get msg queue head unused buf size
 *
 * @q: msg queue
 *
 * It returns size of msg queue head unused buf size, unit 4 bytes
 */
static uint32_t msq_head_free_size(const struct msg_queue *q)
{
	if (q->cur_send >= q->cur_recv)
		return (q->cur_recv - q->buf_head);

	return 0;
}

/**
 * msq_send_msg - send a msg to msg queue
 *
 * @q: msg queue
 * @m: a msg to queue
 *
 * It returns zero on success, else a negative error code.
 */
int msq_send_msg(struct msg_queue *q, const struct msg *m)
{
	int ret = 0;

	if (msq_tail_free_size(q) > m->size) {
		uint32_t *next_send;

		memcpy(q->cur_send, m, m->size * sizeof(uint32_t));
		next_send = q->cur_send + m->size;
		if (next_send == q->buf_tail)
			next_send = q->buf_head;
		q->cur_send = next_send;
	} else if (msq_head_free_size(q) > m->size) {
		*q->cur_send = 0; /* set size to 0 for skip to head mark */
		memcpy(q->buf_head, m, m->size * sizeof(uint32_t));
		q->cur_send = q->buf_head + m->size;
	} else {
		ret = -1;
	}

	return ret;
}

/**
 * msq_recv_msg - receive a msg from msg queue
 *
 * @q: msg queue
 * @m: a msg pointer buf [out]
 *
 * need call msq_recv_msg_free to free msg after msg use done
 *
 * It returns zero on success, else a negative error code.
 */
int msq_recv_msg(struct msg_queue *q, struct msg **m)
{
	*m = NULL;
	if (msq_is_empty(q))
		return -1;

	/* skip to head when size is 0 */
	if (*q->cur_recv == 0)
		*m = (struct msg *)q->buf_head;
	else
		*m = (struct msg *)q->cur_recv;

	return 0;
}

/**
 * msq_free_received_msg - free a received msg to msg queue
 *
 * @q: msg queue
 * @m: a msg
 *
 * It returns zero on success, else a negative error code.
 */
int msq_free_received_msg(struct msg_queue *q, const struct msg *m)
{
	/* skip to head when size is 0 */
	if (*q->cur_recv == 0) {
		q->cur_recv = q->buf_head + m->size;
	} else {
		uint32_t *next_recv;

		next_recv = q->cur_recv + m->size;
		if (next_recv == q->buf_tail)
			next_recv = q->buf_head;

		q->cur_recv = next_recv;
	}

	return 0;
}

/**
 * dsp_msq_init - init AP <-> DSP msg queue
 *
 * @spi: spi device
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_init(struct spi_device *spi)
{
	int err = 0;
	struct dsp_msg_queue queue;

	queue.buf_head = (DSP_R_MSG_QUEUE_ADDR + sizeof(queue));
	queue.buf_tail = (DSP_R_MSG_QUEUE_ADDR + sizeof(queue) + 16 * 1024);
	queue.cur_recv = queue.buf_head;
	queue.cur_send = queue.buf_head;

	err = spi2apb_safe_write(spi, DSP_R_MSG_QUEUE_ADDR,
			(int32_t *)&queue, sizeof(queue));

	queue.buf_head = DSP_S_MSG_QUEUE_ADDR + sizeof(queue);
	queue.buf_tail = DSP_S_MSG_QUEUE_ADDR + sizeof(queue) + 16 * 1024;
	queue.cur_recv = queue.buf_head;
	queue.cur_send = queue.buf_head;

	err = spi2apb_safe_write(spi, DSP_S_MSG_QUEUE_ADDR,
			(int32_t *)&queue, sizeof(queue));
	return err;
}

/**
 * dsp_msq_is_empty - tests whether a msg queue is empty
 *
 * @q: the msg queue to test
 *
 * It returns true on msg queue is empty, else false.
 */
static int dsp_msq_is_empty(const struct dsp_msg_queue *q)
{
	return q->cur_send == q->cur_recv;
}

/**
 * dsp_msq_total_size - get msg queue buf total size
 *
 * @q: msg queue
 *
 * It returns size of msg queue buf, unit byte.
 */
static uint32_t dsp_msq_total_size(const struct dsp_msg_queue *q)
{
	return q->buf_tail - q->buf_head;
}

/**
 * dsp_msq_tail_free_size - get msg queue tail unused buf size
 *
 * @q: msg queue
 *
 * It returns size of msg queue tail unused buf size, unit byte
 */
static uint32_t dsp_msq_tail_free_size(const struct dsp_msg_queue *q)
{
	if (q->cur_send >= q->cur_recv)
		return (q->buf_tail - q->cur_send);

	return q->cur_recv - q->cur_send;
}

/**
 * dsp_msq_head_free_size - get msg queue head unused buf size
 *
 * @q: msg queue
 *
 * It returns size of msg queue head unused buf size, unit byte
 */
static uint32_t dsp_msq_head_free_size(const struct dsp_msg_queue *q)
{
	if (q->cur_send >= q->cur_recv)
		return (q->cur_recv - q->buf_head);

	return 0;
}

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
		uint32_t addr, struct dsp_msg_queue *q)
{
	int err = 0;
	int32_t reg;

	err = spi2apb_safe_r32(spi, DSP_PMU_SYS_REG0, &reg);

	if (err || ((reg & DSP_MSG_QUEUE_OK_MASK) != DSP_MSG_QUEUE_OK_TAG)) {
		/* dev_warn(&spi->dev, "dsp msg queue head not init!\n"); */
		return -1;
	}

	err = spi2apb_safe_read(spi, addr, (int32_t *)q, sizeof(*q));
	return err;
}

/**
 * dsp_msq_send_msg - send a msg to AP -> DSP msg queue
 *
 * @spi: spi device
 * @m: a msg to send
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_send_msg(struct spi_device *spi, const struct msg *m)
{
	int err = 0;
	struct dsp_msg_queue queue;
	struct dsp_msg_queue *q = &queue;
	uint32_t msg_size = m->size * sizeof(uint32_t);

	err = dsp_msq_read_head(spi, DSP_R_MSG_QUEUE_ADDR, q);
	if (err)
		return err;

	if (dsp_msq_tail_free_size(q) > msg_size) {
		uint32_t next_send;

		err = spi2apb_safe_write(spi, q->cur_send, (int32_t *)m, msg_size);
		next_send = q->cur_send + msg_size;
		if (next_send == q->buf_tail)
			next_send = q->buf_head;

		q->cur_send = next_send;
	} else if (dsp_msq_head_free_size(q) > msg_size) {
		//set size to 0 for skip to head mark
		err = spi2apb_safe_w32(spi, q->cur_send, 0);
		if (err)
			return err;

		err = spi2apb_safe_write(spi, q->buf_head, (int32_t *)m, msg_size);

		q->cur_send = q->buf_head + msg_size;
	} else {
		return -1;
	}

	if (err)
		return err;

	err = spi2apb_safe_w32(spi, DSP_R_MSG_QUEUE_ADDR +
			(uint8_t *)&q->cur_send - (uint8_t *)q, q->cur_send);

	spi2apb_interrupt_request(spi, PREISP_IRQ_TYPE_MSG);

	return err;
}

/**
 * dsp_msq_recv_query - query next msg size from DSP -> AP msg queue
 *
 * @q: msg queue
 * @size: msg size buf [out]
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_recv_query(struct spi_device *spi, int32_t *size)
{
	struct dsp_msg_queue queue;
	struct dsp_msg_queue *q = &queue;
	int err = 0;

	err = dsp_msq_read_head(spi, DSP_S_MSG_QUEUE_ADDR, q);
	if (err)
		return err;

	*size = 0;
	if (dsp_msq_is_empty(q))
		return 0;

	//skip to head when size is 0
	err = spi2apb_safe_r32(spi, q->cur_recv, size);
	if (err)
		return err;

	if (*size == 0)
		err = spi2apb_safe_r32(spi, (int32_t)q->buf_head, size);

	return err;
}

/**
 * dsp_msq_recv_msg - receive a msg from DSP -> AP msg queue
 *
 * @q: msg queue
 * @m: a msg pointer buf [out]
 *
 * need call dsp_msq_recv_msg_free to free msg after msg use done
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_recv_msg(struct spi_device *spi, struct msg **m)
{
	struct dsp_msg_queue queue;
	struct dsp_msg_queue *q = &queue;
	uint32_t size = 0, msg_size = 0;
	uint32_t recv_addr = 0;
	uint32_t next_recv_addr = 0;
	int err = 0;

	*m = NULL;

	err = dsp_msq_read_head(spi, DSP_S_MSG_QUEUE_ADDR, q);
	if (err)
		return err;

	if (dsp_msq_is_empty(q))
		return -1;

	/* skip to head when size is 0 */
	err = spi2apb_safe_r32(spi, (int32_t)q->cur_recv, (int32_t *)&size);
	if (err)
		return err;
	if (size == 0) {
		err = spi2apb_safe_r32(spi, (int32_t)q->buf_head,
				(int32_t *)&size);
		if (err)
			return err;

		msg_size = size * sizeof(uint32_t);
		recv_addr = q->buf_head;
		next_recv_addr = q->buf_head + msg_size;
	} else {
		msg_size = size * sizeof(uint32_t);
		recv_addr = q->cur_recv;
		next_recv_addr = q->cur_recv + msg_size;
		if (next_recv_addr == q->buf_tail)
			next_recv_addr = q->buf_head;

	}

	if (msg_size > dsp_msq_total_size(q))
		return -2;

	*m = (struct msg *)kmalloc(msg_size, GFP_KERNEL);
	err = spi2apb_safe_read(spi, recv_addr, (int32_t *)*m, msg_size);
	if (err == 0) {
		err = spi2apb_safe_w32(spi, DSP_S_MSG_QUEUE_ADDR +
			(uint8_t *)&q->cur_recv - (uint8_t *)q, next_recv_addr);
	}

	if (err)
		kfree(*m);

	return err;
}

/**
 * dsp_msq_free_received_msg - free a received msg
 *
 * @q: msg queue
 * @m: a msg
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_free_received_msg(const struct msg *m)
{
	if (m)
		kfree(m);

	return 0;
}
