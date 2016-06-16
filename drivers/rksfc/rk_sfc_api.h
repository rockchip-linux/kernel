/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __RK_FTL_API_H
#define __RK_FTL_API_H

int spi_flash_init(void __iomem *reg_addr);
int snor_read(unsigned int sec, unsigned int n_sec, void *p_data);
int snor_write(unsigned int sec, unsigned int n_sec, void *p_data);
unsigned int snor_get_capacity(void);
void rknand_print_hex(char *s, void *buf, int width, int len);
void sfc_clean_irq(void);
#endif

