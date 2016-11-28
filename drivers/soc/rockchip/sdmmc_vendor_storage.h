/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#ifndef _SDMMC_VENDOR_STORAGE
#define _SDMMC_VENDOR_STORAGE

int rk_emmc_transfer(u8 *buffer, unsigned int addr, unsigned int blksz,
		     int write);

#endif

