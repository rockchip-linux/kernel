/* driver/rksfc/sfc_nor.h
 *
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _SFNOR_H
#define _SFNOR_H

/*Manufactory ID*/
#define MID_WINBOND             0xEF
#define MID_GIGADEV             0xC8
#define MID_MICRON              0x2C
#define MID_MACRONIX            0xC2
#define MID_SPANSION            0x01
#define MID_EON                 0x1C
#define MID_ST                  0x20

/*Command Set*/
#define CMD_READ_JEDECID        (0x9F)
#define CMD_READ_DATA           (0x03)
#define CMD_READ_STATUS         (0x05)
#define CMD_WRITE_STATUS        (0x01)
#define CMD_PAGE_PROG           (0x02)
#define CMD_SECTOR_ERASE        (0x20)
#define CMD_BLK64K_ERASE        (0xD8)
#define CMD_BLK32K_ERASE        (0x52)
#define CMD_CHIP_ERASE          (0xC7)
#define CMD_WRITE_EN            (0x06)
#define CMD_WRITE_DIS           (0x04)
#define CMD_PAGE_READ           (0x13)
#define CMD_GET_FEATURE         (0x0F)
#define CMD_SET_FEATURE         (0x1F)
#define CMD_PROG_LOAD           (0x02)
#define CMD_PROG_EXEC           (0x10)
#define CMD_BLOCK_ERASE         (0xD8)
#define CMD_READ_DATA_X2        (0x3B)
#define CMD_READ_DATA_X4        (0x6B)
#define CMD_PROG_LOAD_X4        (0x32)
#define CMD_READ_STATUS2        (0x35)
#define CMD_READ_STATUS3        (0x15)
#define CMD_WRITE_STATUS2       (0x31)
#define CMD_WRITE_STATUS3       (0x11)
/* X1 cmd, X1 addr, X1 data */
#define CMD_FAST_READ_X1        (0x0B)
/* X1 cmd, X1 addr, X2 data */
#define CMD_FAST_READ_X2        (0x3B)
/* X1 cmd, X1 addr, X4 data SUPPORT GD MARCONIX WINBOND */
#define CMD_FAST_READ_X4        (0x6B)
/* X1 cmd, X1 addr, X4 data SUPPORT GD MARCONIX WINBOND */
#define CMD_FAST_4READ_X4       (0x6C)
/* X1 cmd, X4 addr, X4 data SUPPORT EON GD MARCONIX WINBOND */
#define CMD_FAST_READ_A4        (0xEB)
/* X1 cmd, X1 addr, X4 data, SUPPORT GD WINBOND */
#define CMD_PAGE_PROG_X4        (0x32)
/* X1 cmd, X4 addr, X4 data, SUPPORT MARCONIX */
#define CMD_PAGE_PROG_A4        (0x38)
#define CMD_RESET_NAND          (0xFF)
#define CMD_ENTER_4BYTE_MODE    (0xB7)
#define CMD_EXIT_4BYTE_MODE     (0xE9)
#define CMD_ENABLE_RESER	(0x66)
#define CMD_RESET_DEVICE	(0x99)

enum NOR_ERASE_TYPE {
	ERASE_SECTOR = 0,
	ERASE_BLOCK64K,
	ERASE_CHIP
};

enum SNOR_IO_MODE {
	IO_MODE_SPI = 0,
	IO_MODE_QPI
};

enum SNOR_READ_MODE {
	READ_MODE_NOMAL = 0,
	READ_MODE_FAST
};

enum SNOR_ADDR_MODE {
	ADDR_MODE_3BYTE = 0,
	ADDR_MODE_4BYTE
};


int snor_init(void);
u32 snor_get_capacity(void);
int snor_read_id(u8 *data);
int snor_read(u32 sec, u32 n_sec, void *p_data);
int snor_write(u32 sec, u32 n_sec, void *p_data);
int snor_prog(u32 addr, void *p_data, u32 size);
int snor_erase(u32 addr, enum NOR_ERASE_TYPE erase_type);
int snor_erase_blk(u32 addr);

#endif
