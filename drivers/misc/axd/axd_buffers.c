/*
 * Copyright (C) 2011-2014 Imagination Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * AXD generic buffer management API.
 */
#include <linux/err.h>
#include <linux/slab.h>

#include "axd_buffers.h"

/**
 * axd_buffer_init - sets up axd buffer as a pool of fixed sized buffers.
 * @address: starting address of the buffer as set up in the system
 * @total_size: total size of available buffer
 * @element_size: size of each buffer element
 *
 * axd_buffer_t *buffer is a memory pool of size @element_size and starting at
 * address @address and of @total_size size.
 */
static int bufferq_init(struct axd_bufferq *bufferq, const char *name,
			char *address, unsigned int num_elements,
			unsigned int element_size, unsigned int nonblock)
{
	int i;
	char **queue;
	unsigned int *size;

	strncpy(bufferq->name, name, 16);
	bufferq->stride = element_size;
	bufferq->max = num_elements;
	bufferq->rd_idx = 0;
	bufferq->wr_idx = 0;
	bufferq->nonblock = nonblock;
	queue = kcalloc(num_elements, sizeof(char *), GFP_KERNEL);
	if (!queue)
		return -ENOMEM;
	bufferq->queue = queue;
	size = kcalloc(num_elements, sizeof(unsigned int), GFP_KERNEL);
	if (!size) {
		kfree(queue);
		bufferq->queue = NULL;
		return -ENOMEM;
	}
	bufferq->size = size;
	/*
	 * setup the queue with all available buffer addresses if the base
	 * address is passed. Set it up as emptry if base address is NULL.
	 */
	if (address) {
		for (i = 0; i < num_elements; i++) {
			queue[i] = address + (element_size * i);
			size[i] = element_size;
		}
		sema_init(&bufferq->rd_sem, num_elements);
		sema_init(&bufferq->wr_sem, 0);
	} else {
		for (i = 0; i < num_elements; i++) {
			queue[i] = NULL;
			size[i] = element_size;
		}
		sema_init(&bufferq->rd_sem, 0);
		sema_init(&bufferq->wr_sem, num_elements);
	}
	spin_lock_init(&bufferq->q_rdlock);
	spin_lock_init(&bufferq->q_wrlock);
	pr_debug("Initialized %s of %d elements of size %d bytes\n",
					name, num_elements, element_size);
	pr_debug("Address of %s: 0x%08X\n", name, (unsigned int)bufferq);
	return 0;
}

int axd_bufferq_init(struct axd_bufferq *bufferq, const char *name,
			char *address, unsigned int num_elements,
			unsigned int element_size, unsigned int nonblock)
{
	return bufferq_init(bufferq,
			name, address, num_elements, element_size, nonblock);
}

int axd_bufferq_init_empty(struct axd_bufferq *bufferq, const char *name,
			unsigned int num_elements, unsigned int element_size,
			unsigned int nonblock)
{
	return bufferq_init(bufferq,
			name, NULL, num_elements, element_size, nonblock);
}

void axd_bufferq_clear(struct axd_bufferq *bufferq)
{
	kfree(bufferq->queue);
	kfree(bufferq->size);
	bufferq->queue = NULL;
	bufferq->size = NULL;
}

/**
 * axd_buffer_take - returns a valid buffer pointer
 * @buffer: the buffers pool to be accessed
 *
 * This function will go into interruptible sleep if the pool is empty.
 */
char *axd_bufferq_take(struct axd_bufferq *bufferq, int *buf_size)
{
	char *buf;
	int ret;

	if (!bufferq->queue)
		return NULL;

	pr_debug("--(%s)-- taking new buffer\n", bufferq->name);
	if (bufferq->nonblock) {
		ret = down_trylock(&bufferq->rd_sem);
		if (ret)
			return ERR_PTR(-EAGAIN);

	} else {
		ret = down_interruptible(&bufferq->rd_sem);
		if (ret)
			return ERR_PTR(-ERESTARTSYS);
		if (bufferq->abort_take) {
			bufferq->abort_take = 0;
			return ERR_PTR(-ERESTARTSYS);
		}
	}
	/*
	 * must ensure we have one access at a time to the queue and rd_idx
	 * to be preemption and SMP safe
	 * Sempahores will ensure that we will only read after a complete write
	 * has finished, so we will never read and write from the same location.
	 */
	spin_lock(&bufferq->q_rdlock);
	buf = bufferq->queue[bufferq->rd_idx];
	if (buf_size)
		*buf_size = bufferq->size[bufferq->rd_idx];
	bufferq->rd_idx++;
	if (bufferq->rd_idx >= bufferq->max)
		bufferq->rd_idx = 0;
	spin_unlock(&bufferq->q_rdlock);
	up(&bufferq->wr_sem);
	pr_debug("--(%s)-- took buffer <0x%08X>\n", bufferq->name,
							(unsigned int)buf);
	return buf;
}

/**
 * axd_buffer_put - returns a buffer to the pool.
 * @buffer: the buffers pool to be accessed
 * @buf: the buffer to be returned.
 *
 * This function will go into interruptible sleep if the pool is full.
 */
int axd_bufferq_put(struct axd_bufferq *bufferq, char *buf, int buf_size)
{
	int ret;

	if (!bufferq->queue)
		return 0;

	if (buf_size < 0)
		buf_size = bufferq->stride;

	pr_debug("++(%s)++ returning buffer\n", bufferq->name);
	if (bufferq->nonblock) {
		ret = down_trylock(&bufferq->wr_sem);
		if (ret)
			return -EAGAIN;

	} else {
		ret = down_interruptible(&bufferq->wr_sem);
		if (ret)
			return -ERESTARTSYS;
		if (bufferq->abort_put) {
			bufferq->abort_put = 0;
			return -ERESTARTSYS;
		}
	}
	/*
	 * must ensure we have one access at a time to the queue and wr_idx
	 * to be preemption and SMP safe.
	 * Semaphores will ensure that we only write after a complete read has
	 * finished, so we will never write and read from the same location.
	 */
	spin_lock(&bufferq->q_wrlock);
	bufferq->queue[bufferq->wr_idx] = buf;
	bufferq->size[bufferq->wr_idx] = buf_size;
	bufferq->wr_idx++;
	if (bufferq->wr_idx >= bufferq->max)
		bufferq->wr_idx = 0;
	spin_unlock(&bufferq->q_wrlock);
	up(&bufferq->rd_sem);
	pr_debug("++(%s)++ returned buffer <0x%08X>\n", bufferq->name,
							(unsigned int)buf);
	return 0;
}

int axd_bufferq_is_full(struct axd_bufferq *bufferq)
{
	int ret;
	/*
	 * if we can't put a buffer, then we're full.
	 */
	ret = down_trylock(&bufferq->wr_sem);
	if (!ret)
		up(&bufferq->wr_sem);
	return ret;
}

int axd_bufferq_is_empty(struct axd_bufferq *bufferq)
{
	int ret;
	/*
	 * if we can't take more buffers, then its empty.
	 */
	ret = down_trylock(&bufferq->rd_sem);
	if (!ret)
		up(&bufferq->rd_sem);
	return ret;
}

void axd_bufferq_abort_take(struct axd_bufferq *bufferq)
{
	if (axd_bufferq_is_empty(bufferq)) {
		bufferq->abort_take = 1;
		up(&bufferq->rd_sem);
	}
}

void axd_bufferq_abort_put(struct axd_bufferq *bufferq)
{
	if (axd_bufferq_is_full(bufferq)) {
		bufferq->abort_put = 1;
		up(&bufferq->wr_sem);
	}
}
