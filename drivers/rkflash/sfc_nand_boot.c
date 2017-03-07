/*
 * Copyright (c) 2017, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/mutex.h>

#include "rk_sftl.h"
#include "rkflash_api.h"
#include "sfc_nand.h"
#include "typedef.h"

static DEFINE_MUTEX(g_flash_ops_mutex);

int snand_init(void __iomem *reg_addr)
{
	int ret;

	ret = sfc_nand_init(reg_addr);
	if (ret == 0)
		ret = sftl_init();

	return ret;
}
EXPORT_SYMBOL_GPL(snand_init);

void snand_read_id(u8 chip_sel, void *buf)
{
	mutex_lock(&g_flash_ops_mutex);
	sfc_nand_read_id(buf);
	mutex_unlock(&g_flash_ops_mutex);

}
EXPORT_SYMBOL_GPL(snand_read_id);

unsigned int snand_get_capacity(void)
{
	return sftl_get_density();
}

int snand_write(u32 sec, u32 n_sec, void *p_data)
{
	mutex_lock(&g_flash_ops_mutex);
	sftl_write(sec, n_sec, p_data);
	mutex_unlock(&g_flash_ops_mutex);

	return 0;
}

int snand_read(u32 sec, u32 n_sec, void *p_data)
{
	mutex_lock(&g_flash_ops_mutex);
	sftl_read(sec, n_sec, p_data);
	mutex_unlock(&g_flash_ops_mutex);

	return 0;
}

void snand_deinit(void)
{
	sftl_deinit();
	sfc_nand_deinit();
}

int snand_resume(void __iomem *reg_addr)
{
	return sfc_nand_init(reg_addr);
}
