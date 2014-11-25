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
#ifndef AXD_BUFFERS_H_
#define AXD_BUFFERS_H_

#include <linux/semaphore.h>
#include <linux/spinlock.h>

/**
 * struct axd_bufferq - axd buffer management structure
 * @name:	name of the buffer queue
 * @stride:	the space between buffers in memory
 * @max:	total number of buffers this queue can handle
 * @rd_idx:	read index of the circular buffer
 * @wr_idx:	write index of the circular buffer
 * @rd_sem:	semaphore to block when full
 * @wr_sem:	semaphore to block when empty
 * @q_rdlock:	smp critical section protection for reads
 * @q_wrlock:	smp critical section protection for writes
 * @queue:	array of pointers to buffer addresses
 * @size:	array of buffer's actual amount of data it has inside or it can
 *		store.
 * @nonblock:	return an error instead of block when empty/full
 * @abort_take:	abort any pending blocked take operation
 * @abort_put:	abort any pending blocked put operation
 *
 * axd_bufferq takes a contiguous memory region and divides it into smaller
 * buffers regions of equal size and represents it as a queue. To avoid
 * excessive locking it's done as a circular buffer queue.
 */
struct axd_bufferq {
	char name[16];
	unsigned int stride;
	unsigned int max;
	unsigned int rd_idx;
	unsigned int wr_idx;
	struct semaphore rd_sem;
	struct semaphore wr_sem;
	spinlock_t q_rdlock;
	spinlock_t q_wrlock;
	char **queue;
	unsigned int *size;
	unsigned int nonblock;
	unsigned int abort_take;
	unsigned int abort_put;
};

int axd_bufferq_init(struct axd_bufferq *bufferq, const char *name,
			char *address, unsigned int num_elements,
			unsigned int element_size, unsigned int nonblock);
int axd_bufferq_init_empty(struct axd_bufferq *bufferq, const char *name,
			unsigned int num_elements, unsigned int element_size,
			unsigned int nonblock);
void axd_bufferq_clear(struct axd_bufferq *bufferq);
char *axd_bufferq_take(struct axd_bufferq *bufferq, int *buf_size);
int axd_bufferq_put(struct axd_bufferq *bufferq, char *buf, int buf_size);
int axd_bufferq_is_full(struct axd_bufferq *bufferq);
int axd_bufferq_is_empty(struct axd_bufferq *bufferq);
void axd_bufferq_abort_take(struct axd_bufferq *bufferq);
void axd_bufferq_abort_put(struct axd_bufferq *bufferq);

#endif /* AXD_BUFFERS_H_ */
