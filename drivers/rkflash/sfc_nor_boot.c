/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/mutex.h>

#include "sfc.h"
#include "sfc_nor.h"
#include "typedef.h"

int spi_flash_init(void __iomem	*reg_addr)
{
	int ret;

	sfc_init(reg_addr);
	ret = snor_init();

	return ret;
}
EXPORT_SYMBOL_GPL(spi_flash_init);

void spi_flash_read_id(u8 chip_sel, void *buf)
{
	snor_read_id(buf);
}
EXPORT_SYMBOL_GPL(spi_flash_read_id);
