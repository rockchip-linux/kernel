/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __RK_FLASH_API_H
#define __RK_FLASH_API_H

void rknand_print_hex(char *s, void *buf, int width, int len);

#ifdef CONFIG_RK_NANDC_NAND
int sftl_flash_init(void __iomem *reg_addr);
int sftl_flash_read(unsigned int sec, unsigned int n_sec, void *p_data);
int sftl_flash_write(unsigned int sec, unsigned int n_sec, void *p_data);
unsigned int sftl_flash_get_capacity(void);
void sftl_flash_deinit(void);
int sftl_flash_resume(void __iomem *reg_addr);
void nandc_clean_irq(void);
#endif

#ifdef CONFIG_RK_SFC_NOR
int spi_flash_init(void __iomem *reg_addr);
int snor_read(unsigned int sec, unsigned int n_sec, void *p_data);
int snor_write(unsigned int sec, unsigned int n_sec, void *p_data);
unsigned int snor_get_capacity(void);
void snor_deinit(void);
int snor_resume(void __iomem *reg_addr);
void sfc_clean_irq(void);
#endif

#ifdef CONFIG_RK_SFC_NAND
int snand_init(void __iomem *reg_addr);
int snand_read(unsigned int sec, unsigned int n_sec, void *p_data);
int snand_write(unsigned int sec, unsigned int n_sec, void *p_data);
unsigned int snand_get_capacity(void);
void snand_deinit(void);
int snand_resume(void __iomem *reg_addr);
void sfc_clean_irq(void);
#endif

#endif
