/*
 *
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _SPI_BOOT_H
#define _SPI_BOOT_H

int spi_flash_init(unsigned long reg_addr);
void spi_flash_read_id(u8 chip_sel, void *buf);
void spi_read_flash_info(void *buf);
u32 spi_boot_erase(u8 chip_sel, u32 blk_index, u32 nblk, u8 mod);
u32 spi_boot_read_pba(u8 chip_sel, u32 PBA, void *pbuf, u32 n_sec);
u32 spi_boot_write_pba(u8 chip_sel, u32 PBA, void *pbuf, u32 n_sec);
u32 spi_boot_read_lba(u8 chip_sel, u32 LBA, void *pbuf, u32 n_sec);
u32 spi_boot_write_lba(u8 chip_sel, u32 LBA, void *pbuf, u32 n_sec, u16 mode);

#endif

