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

#include "flash.h"
#include "rk_sftl.h"
#include "rkflash_api.h"
#include "typedef.h"

static DEFINE_MUTEX(g_flash_ops_mutex);

int sftl_flash_init(void __iomem *reg_addr)
{
	int ret;

	ret = flash_init(reg_addr);
	if (ret == 0)
		ret = sftl_init();

	return ret;
}
EXPORT_SYMBOL_GPL(sftl_flash_init);

void sftl_flash_read_id(u8 chip_sel, void *buf)
{
	mutex_lock(&g_flash_ops_mutex);
	flash_get_id(chip_sel, buf);
	mutex_unlock(&g_flash_ops_mutex);
}
EXPORT_SYMBOL_GPL(sftl_flash_read_id);

unsigned int sftl_flash_get_capacity(void)
{
	return sftl_get_density();
}

int sftl_flash_write(u32 sec, u32 n_sec, void *p_data)
{
	mutex_lock(&g_flash_ops_mutex);
	sftl_write(sec, n_sec, p_data);
	mutex_unlock(&g_flash_ops_mutex);

	return 0;
}

int sftl_flash_read(u32 sec, u32 n_sec, void *p_data)
{
	mutex_lock(&g_flash_ops_mutex);
	sftl_read(sec, n_sec, p_data);
	mutex_unlock(&g_flash_ops_mutex);

	return 0;
}

void sftl_flash_deinit(void)
{
	u8 chip_sel = 0;

	sftl_deinit();
	flash_reset(chip_sel);
}

int sftl_flash_resume(void __iomem *reg_addr)
{
	return flash_init(reg_addr);
}

void sftl_flash_clean_irq(void)
{
}
