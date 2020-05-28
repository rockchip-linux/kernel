/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Rockchip NPU USB ACM driver
 *
 * Copyright (C) 2019 Fuzhou Rockchip Electronics Co., Ltd.
 */

#ifndef NPU_ACM_H
#define NPU_ACM_H

int npu_acm_read(unsigned char *buf, int size);
int npu_acm_write(unsigned char *buf, int write_length);
int npu_acm_transfer(unsigned char *write_buf, int write_length,
		     unsigned char *read_buf, int read_size,
		     int *read_count);

#endif /* NPU_ACM_H */

