/*
 * Copyright (C) 2014 Imagination Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#ifndef __LINUX_MTD_SPI_NAND_H
#define __LINUX_MTD_SPI_NAND_H

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>

struct spi_nand {
	struct mtd_info		mtd;
	struct nand_chip	nand_chip;
	struct device		*dev;
	const char		*name;

	u8			*buf, *data_buf;
	size_t			buf_size;
	off_t			buf_start;
	unsigned int		page_addr;
	unsigned int		bitflips;
	bool			ecc;

	int (*reset)(struct spi_nand *snand);
	int (*read_id)(struct spi_nand *snand, u8 *buf);

	int (*write_disable)(struct spi_nand *snand);
	int (*write_enable)(struct spi_nand *snand);

	int (*read_reg)(struct spi_nand *snand, u8 opcode, u8 *buf);
	int (*write_reg)(struct spi_nand *snand, u8 opcode, u8 *buf);
	void (*get_ecc_status)(unsigned int status,
			       unsigned int *corrected,
			       unsigned int *ecc_errors);

	int (*store_cache)(struct spi_nand *snand, unsigned int page_offset,
			   size_t length, u8 *write_buf);
	int (*write_page)(struct spi_nand *snand, unsigned int page_addr);
	int (*load_page)(struct spi_nand *snand, unsigned int page_addr);
	int (*read_cache)(struct spi_nand *snand, unsigned int page_offset,
			  size_t length, u8 *read_buf);
	int (*block_erase)(struct spi_nand *snand, unsigned int page_addr);

	void *priv;
};

int spi_nand_register(struct spi_nand *snand, struct nand_flash_dev *flash_ids);
void spi_nand_unregister(struct spi_nand *snand);

#endif
