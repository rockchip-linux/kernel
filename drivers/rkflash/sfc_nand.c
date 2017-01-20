/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include "flash.h"
#include "flash_com.h"
#include "sfc.h"
#include "sfc_nand.h"

#define SFC_NAND_STRESS_TEST_EN		0

#define SFC_NAND_PROG_ERASE_ERROR	-2
#define SFC_NAND_HW_ERROR		-1
#define SFC_NAND_ECC_ERROR		NAND_ERROR
#define SFC_NAND_ECC_REFRESH		NAND_STS_REFRESH
#define SFC_NAND_ECC_OK			NAND_STS_OK

#define SFC_NAND_PAGE_MAX_SIZE		2112

struct SFC_NAND_DEV_T {
	u32 capacity;
	u8 manufacturer;
	u8 mem_type;
	u16 page_size;
	u32 block_size;
};

static struct NAND_PARA_INFO_T  nand_para = {
	2,
	{0x98, 0xC2, 0x98, 0, 0, 0},
	TOSHIBA,
	1,
	4,
	64,
	1,
	1,
	1024,
	0x100,
	LSB_0,
	RR_NONE,
	16,
	40,
	1,
	0,
	BBF_1,
	MPM_0,
	{0}
}; /* TC58CVG0S3HQA2E */

static u32 gp_page_buf[SFC_NAND_PAGE_MAX_SIZE / 4];
struct SFC_NAND_DEV_T sfc_nand_dev;

static int sfc_nand_write_en(void)
{
	int ret;
	union SFCCMD_DATA sfcmd;

	sfcmd.d32 = 0;
	sfcmd.b.cmd = CMD_WRITE_EN;
	ret = sfc_request(sfcmd.d32, 0, 0, NULL);
	return ret;
}

static int sfc_nand_read_feature(u8 addr, u8 *data)
{
	int ret;
	union SFCCMD_DATA sfcmd;

	sfcmd.d32 = 0;
	sfcmd.b.cmd = 0x0F;
	sfcmd.b.datasize = 1;
	sfcmd.b.addrbits = SFC_ADDR_XBITS;
	*data = 0;

	ret = sfc_request(sfcmd.d32, 0x8 << 16, addr, data);
	if (ret != SFC_OK)
		return ret;
	return SFC_OK;
}

static int sfc_nand_write_feature(u32 addr, u8 status)
{
	int ret;
	union SFCCMD_DATA sfcmd;

	sfc_nand_write_en();

	sfcmd.d32 = 0;
	sfcmd.b.cmd = 0x1F;
	sfcmd.b.datasize = 1;
	sfcmd.b.addrbits = SFC_ADDR_XBITS;
	sfcmd.b.rw = SFC_WRITE;

	ret = sfc_request(sfcmd.d32, 0x8 << 16, addr, &status);
	if (ret != SFC_OK)
		return ret;
	return ret;
}

static int sfc_nand_wait_busy(u8 *data, int timeout)
{
	int ret;
	u32 i;
	u8 status;

	*data = 0;
	for (i = 0; i < timeout; i++) {
		ret = sfc_nand_read_feature(0xC0, &status);
		if (ret != SFC_OK)
			return ret;
		*data = status;
		if (!(status & (1 << 0)))
			return SFC_OK;
		sfc_delay(1);
	}
	return -1;
}

u32 sfc_nand_erase_block(u8 cs, u32 addr)
{
	int ret;
	union SFCCMD_DATA sfcmd;
	u8 status;

	sfcmd.d32 = 0;
	sfcmd.b.cmd = 0xD8;
	sfcmd.b.addrbits = SFC_ADDR_24BITS;
	sfc_nand_write_en();
	ret = sfc_request(sfcmd.d32, 0, addr, NULL);
	if (ret != SFC_OK)
		return ret;
	ret = sfc_nand_wait_busy(&status, 1000 * 1000);
	if (status & (1 << 2))
		return SFC_NAND_PROG_ERASE_ERROR;
	return ret;
}

u32 sfc_nand_prog_page(u8 cs, u32 addr, u32 *p_data, u32 *p_spare)
{
	int ret;
	union SFCCMD_DATA sfcmd;
	union SFCCTRL_DATA sfctrl;
	u8 status;
	u32 data_sz = 2048;
	u32 spare_sz = 8;
	u32 spare_offs = 4;

	memset(gp_page_buf, 0, SFC_NAND_PAGE_MAX_SIZE);
	memcpy(gp_page_buf, p_data, data_sz);
	memcpy(gp_page_buf + (data_sz + spare_offs) / 4, p_spare, spare_sz);

	sfcmd.d32 = 0;
	sfcmd.b.cmd = 0x02;
	sfcmd.b.addrbits = SFC_ADDR_XBITS;
	sfcmd.b.datasize = SFC_NAND_PAGE_MAX_SIZE;
	sfcmd.b.rw = SFC_WRITE;

	sfctrl.d32 = 0;
	sfctrl.b.datalines = 0;
	sfctrl.b.enbledma = 1;
	sfctrl.b.addrbits = 16;
	ret = sfc_request(sfcmd.d32, sfctrl.d32, 0, gp_page_buf);

	sfcmd.d32 = 0;
	sfcmd.b.cmd = 0x10;
	sfcmd.b.addrbits = SFC_ADDR_24BITS;
	sfcmd.b.datasize = 0;
	sfcmd.b.rw = SFC_WRITE;
	sfc_nand_write_en();
	ret = sfc_request(sfcmd.d32, 0, addr, p_data);
	if (ret != SFC_OK)
		return ret;
	ret = sfc_nand_wait_busy(&status, 1000 * 1000);
	if (status & (1 << 3))
		return SFC_NAND_PROG_ERASE_ERROR;
	return ret;
}

u32 sfc_nand_read_page(u8 cs, u32 addr, u32 *p_data, u32 *p_spare)
{
	int ret;
	union SFCCMD_DATA sfcmd;
	union SFCCTRL_DATA sfctrl;
	u8 status;
	u8 ecc;
	u32 data_sz = 2048;
	u32 spare_sz = 8;
	u32 spare_offs = 4;

	sfcmd.d32 = 0;
	sfcmd.b.cmd = 0x13;
	sfcmd.b.datasize = 0;
	sfcmd.b.addrbits = SFC_ADDR_24BITS;
	ret = sfc_request(sfcmd.d32, 0, addr, p_data);

	ret = sfc_nand_wait_busy(&status, 1000 * 1000);
	ecc = (status >> 4) & 0x03;
	sfcmd.d32 = 0;
	sfcmd.b.cmd = 0x03;
	sfcmd.b.datasize = SFC_NAND_PAGE_MAX_SIZE;
	sfcmd.b.addrbits = SFC_ADDR_24BITS;
	sfctrl.d32 = 0;
	sfctrl.b.datalines = 0;
	sfctrl.b.enbledma = 1;

	memset(gp_page_buf, 0, SFC_NAND_PAGE_MAX_SIZE);
	ret = sfc_request(sfcmd.d32, sfctrl.d32, 0, gp_page_buf);
	memcpy(p_data, gp_page_buf, data_sz);
	memcpy(p_spare, gp_page_buf + (data_sz + spare_offs) / 4, spare_sz);
	if (ret != SFC_OK)
		return SFC_NAND_ECC_ERROR;

	/*
	 * ecc status:
	 * 0, No bit errors were detected
	 * 1, Bit errors were detected and corrected. Bit error count did not
	 *	exceed the bit flip detection threshold
	 * 2, Multiple bit errors were detected and not corrected.
	 * 3, Bit errors were detected and corrected. Bit error count exceed
	 *	the bit flip detection threshold
	 */
	if ((ecc == 0) || (ecc == 1))
		ret = SFC_NAND_ECC_OK;
	else if (ecc == 3)
		ret = SFC_NAND_ECC_REFRESH;
	else
		ret = SFC_NAND_ECC_ERROR;

	if (ret != SFC_NAND_ECC_OK) {
		PRINT_E("sfc_nand_read_page[0x%x], ret=0x%x\n", addr, ret);
		if (p_data)
			rknand_print_hex("data:", p_data, 4, 8);
		if (p_spare)
			rknand_print_hex("spare:", p_spare, 4, 2);
	}
	return ret;
}

int sfc_nand_read_id(u8 *data)
{
	int ret;
	union SFCCMD_DATA sfcmd;

	sfcmd.d32 = 0;
	sfcmd.b.cmd = CMD_READ_JEDECID;
	sfcmd.b.datasize = 3;
	sfcmd.b.addrbits = SFC_ADDR_XBITS;

	ret = sfc_request(sfcmd.d32, 0x8 << 16, 0, data);

	return ret;
}

/*
 * Read the 1st page's 1st byte of a phy_blk
 * If not FF, it's bad blk
 */
static int sfc_nand_get_bad_block_list(u16 *table, u32 die)
{
	u16 blk;
	u32 bad_cnt, page;
	u32 blk_per_die;
	u32 *pread;
	u32 *pspare_read;

	PRINT_E("sfc_nand_get_bad_block_list\n");
	pread = ftl_malloc(2048);
	pspare_read = ftl_malloc(8);
	bad_cnt = 0;
	blk_per_die = nand_para.plane_per_die * nand_para.blk_per_plane;
	for (blk = 0; blk < blk_per_die; blk++) {
		page = (blk + blk_per_die * die) * nand_para.page_per_blk + 0;
		sfc_nand_read_page(0, page, pread, pspare_read);

		if (pread[0] != 0xFFFFFFFF) {
			table[bad_cnt++] = blk;
			PRINT_E("die[%d], bad_blk[%d]\n", die, blk);
		}
	}
	return (int)bad_cnt;
}

#if SFC_NAND_STRESS_TEST_EN

#define SFC_NAND_PAGE_SIZE	2048
#define SFC_NAND_SPARE_SIZE	8

static u16 bad_blk_list[1024];
static u32 pwrite[SFC_NAND_PAGE_SIZE / 4];
static u32 pread[SFC_NAND_PAGE_SIZE / 4];
static u32 pspare_write[SFC_NAND_SPARE_SIZE / 4];
static u32 pspare_read[SFC_NAND_SPARE_SIZE / 4];
static u32 bad_blk_num;
static u32 bad_page_num;

static void sfc_nand_test(void)
{
	u32 i, blk, page, bad_cnt, page_addr;
	int ret;
	u32 pages_num = 64;
	u32 blk_addr = 64;
	u32 is_bad_blk = 0;

	PRINT_E("sfc_nand_test\n");

	bad_blk_num = 0;
	bad_page_num = 0;
	bad_cnt	= sfc_nand_get_bad_block_list(bad_blk_list, 0);

	for (blk = 0; blk < 1024; blk++) {
		for (i = 0; i < bad_cnt; i++) {
			if (bad_blk_list[i] == blk)
				break;
		}
		if (i < bad_cnt)
			continue;
		is_bad_blk = 0;
		PRINT_E("Flash prog block: %x\n", blk);
		sfc_nand_erase_block(0, blk * blk_addr);
		for (page = 0; page < pages_num; page++) {
			page_addr = blk * blk_addr + page;
			for (i = 0; i < 512; i++)
				pwrite[i] = (page_addr << 16) + i;
			pspare_write[0] = pwrite[0] + 0x5AF0;
			pspare_write[1] = pspare_write[0] + 1;
			sfc_nand_prog_page(0, page_addr, pwrite, pspare_write);
			memset(pread, 0, 2048);
			memset(pspare_read, 0, 8);
			ret = sfc_nand_read_page(0, page_addr, pread,
						 pspare_read);
			if (ret != SFC_NAND_ECC_OK)
				is_bad_blk = 1;
			for (i = 0; i < 512; i++) {
				if (pwrite[i] != pread[i]) {
					is_bad_blk = 1;
					break;
				}
			}
			for (i = 0; i < 2; i++) {
				if (pspare_write[i] != pspare_read[i]) {
					is_bad_blk = 1;
					break;
				}
			}
			if (is_bad_blk) {
				bad_page_num++;
				PRINT_E("ERR:page%x, ret=%x\n", page_addr, ret);
				rknand_print_hex("data:", pread, 4, 8);
				rknand_print_hex("spare:", pspare_read, 4, 2);
			}
		}
		sfc_nand_erase_block(0, blk * blk_addr);
		if (is_bad_blk)
			bad_blk_num++;
	}
	PRINT_E("bad_blk_num = %d, bad_page_num = %d\n",
		bad_blk_num, bad_page_num);

	PRINT_E("Flash Test Finish!!!\n");
	while (1)
		;
}
#endif

static void ftl_flash_init(void)
{
	/* para init */
	g_nand_phy_info.nand_type	= nand_para.cell;
	g_nand_phy_info.die_num		= nand_para.die_per_chip;
	g_nand_phy_info.plane_per_die	= nand_para.plane_per_die;
	g_nand_phy_info.blk_per_plane	= nand_para.blk_per_plane;
	g_nand_phy_info.page_per_blk	= nand_para.page_per_blk;
	g_nand_phy_info.page_per_slc_blk	= nand_para.page_per_blk /
						  nand_para.cell;
	g_nand_phy_info.byte_per_sec	= 512;
	g_nand_phy_info.sec_per_page	= nand_para.sec_per_page;
	g_nand_phy_info.sec_per_blk	= nand_para.sec_per_page *
					  nand_para.page_per_blk;
	g_nand_phy_info.reserved_blk	= 8;
	g_nand_phy_info.blk_per_die	= nand_para.plane_per_die *
					  nand_para.blk_per_plane;
	g_nand_phy_info.ecc_bits	= nand_para.ecc_bits;

	/* driver register */
	g_nand_ops.get_bad_blk_list	= sfc_nand_get_bad_block_list;
	g_nand_ops.erase_blk		= sfc_nand_erase_block;
	g_nand_ops.prog_page		= sfc_nand_prog_page;
	g_nand_ops.read_page		= sfc_nand_read_page;
}

u32 sfc_nand_init(void __iomem *sfc_addr)
{
	u8 id_byte[5];
	u8 ecc_bits = 0;

	PRINT_E("sfc_nand_init\n");
	sfc_init(sfc_addr);
	sfc_nand_read_id(id_byte);
	PRINT_E("sfc_nand id: %x %x %x\n", id_byte[0], id_byte[1], id_byte[2]);
	if ((0xFF == id_byte[0] && 0xFF == id_byte[1]) ||
	    (0x00 == id_byte[0] && 0x00 == id_byte[1]))
		return SFC_ERROR;

	sfc_nand_dev.manufacturer = id_byte[0];
	sfc_nand_dev.mem_type = id_byte[1];

	/* disable block lock */
	sfc_nand_write_feature(0xA0, 0);
	sfc_nand_read_feature(0x10, &ecc_bits);
	PRINT_E("sfc_nand ecc bits = %d\n", ecc_bits >> 4);

	ftl_flash_init();

	#if SFC_NAND_STRESS_TEST_EN
	sfc_nand_test();
	#endif

	return SFC_OK;
}

void sfc_nand_deinit(void)
{
}
