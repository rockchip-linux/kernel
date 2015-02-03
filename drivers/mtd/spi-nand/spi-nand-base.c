/*
 * Copyright (C) 2014 Imagination Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * Notes:
 * 1. Erase and program operations need to call write_enable() first,
 *    to clear the enable bit. This bit is cleared automatically after
 *    the erase or program operation.
 *
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nand.h>
#include <linux/of_platform.h>
#include <linux/of_mtd.h>
#include <linux/slab.h>

/* Registers common to all devices */
#define SPI_NAND_LOCK_REG		0xa0
#define SPI_NAND_PROT_UNLOCK_ALL	0x0

#define SPI_NAND_FEATURE_REG		0xb0
#define SPI_NAND_ECC_EN			BIT(4)

#define SPI_NAND_STATUS_REG		0xc0
#define SPI_NAND_STATUS_REG_ECC_MASK	0x3
#define SPI_NAND_STATUS_REG_ECC_SHIFT	4
#define SPI_NAND_STATUS_REG_PROG_FAIL	BIT(3)
#define SPI_NAND_STATUS_REG_ERASE_FAIL	BIT(2)
#define SPI_NAND_STATUS_REG_WREN	BIT(1)
#define SPI_NAND_STATUS_REG_BUSY	BIT(0)

#define SPI_NAND_CMD_BUF_LEN		8

/* Rewind and fill the buffer with 0xff */
static void spi_nand_clear_buffer(struct spi_nand *snand)
{
	snand->buf_start = 0;
	memset(snand->data_buf, 0xff, snand->buf_size);
}

static int spi_nand_enable_ecc(struct spi_nand *snand)
{
	int ret;

	ret = snand->read_reg(snand, SPI_NAND_FEATURE_REG, snand->buf);
	if (ret)
		return ret;

	snand->buf[0] |= SPI_NAND_ECC_EN;
	ret = snand->write_reg(snand, SPI_NAND_FEATURE_REG, snand->buf);
	if (ret)
		return ret;
	snand->ecc = true;

	return 0;
}

static int spi_nand_disable_ecc(struct spi_nand *snand)
{
	int ret;

	ret = snand->read_reg(snand, SPI_NAND_FEATURE_REG, snand->buf);
	if (ret)
		return ret;

	snand->buf[0] &= ~SPI_NAND_ECC_EN;
	ret = snand->write_reg(snand, SPI_NAND_FEATURE_REG, snand->buf);
	if (ret)
		return ret;
	snand->ecc = false;

	return 0;
}

/*
 * Wait until the status register busy bit is cleared.
 * Returns a negatie errno on error or time out, and a non-negative status
 * value if the device is ready.
 */
static int spi_nand_wait_till_ready(struct spi_nand *snand)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(100);
	bool timeout = false;
	int ret;

	/*
	 * Perhaps we should set a different timeout for each
	 * operation (reset, read, write, erase).
	 */
	while (!timeout) {
		if (time_after_eq(jiffies, deadline))
			timeout = true;

		ret = snand->read_reg(snand, SPI_NAND_STATUS_REG, snand->buf);
		if (ret < 0) {
			dev_err(snand->dev, "error reading status register\n");
			return ret;
		} else if (!(snand->buf[0] & SPI_NAND_STATUS_REG_BUSY)) {
			return snand->buf[0];
		}

		cond_resched();
	}

	dev_err(snand->dev, "operation timed out\n");

	return -ETIMEDOUT;
}

static int spi_nand_reset(struct spi_nand *snand)
{
	int ret;

	ret = snand->reset(snand);
	if (ret < 0) {
		dev_err(snand->dev, "reset command failed\n");
		return ret;
	}

	/*
	 * The NAND core won't wait after a device reset, so we need
	 * to do that here.
	 */
	ret = spi_nand_wait_till_ready(snand);
	if (ret < 0)
		return ret;
	return 0;
}

static int spi_nand_status(struct spi_nand *snand)
{
	int ret, status;

	ret = snand->read_reg(snand, SPI_NAND_STATUS_REG, snand->buf);
	if (ret < 0) {
		dev_err(snand->dev, "error reading status register\n");
		return ret;
	}
	status = snand->buf[0];

	/* Convert this into standard NAND_STATUS values */
	if (status & SPI_NAND_STATUS_REG_BUSY)
		snand->buf[0] = 0;
	else
		snand->buf[0] = NAND_STATUS_READY;

	if (status & SPI_NAND_STATUS_REG_PROG_FAIL ||
	    status & SPI_NAND_STATUS_REG_ERASE_FAIL)
		snand->buf[0] |= NAND_STATUS_FAIL;

	/*
	 * Since we unlock the entire device at initialization, unconditionally
	 * set the WP bit to indicate it's not protected.
	 */
	snand->buf[0] |= NAND_STATUS_WP;
	return 0;
}

static int spi_nand_erase(struct spi_nand *snand, int page_addr)
{
	int ret;

	ret = snand->write_enable(snand);
	if (ret < 0) {
		dev_err(snand->dev, "write enable command failed\n");
		return ret;
	}

	ret = snand->block_erase(snand, page_addr);
	if (ret < 0) {
		dev_err(snand->dev, "block erase command failed\n");
		return ret;
	}

	return 0;
}

static int spi_nand_write(struct spi_nand *snand)
{
	int ret;

	/* Store the page to cache */
	ret = snand->store_cache(snand, 0, snand->buf_size, snand->data_buf);
	if (ret < 0) {
		dev_err(snand->dev, "error %d storing page 0x%x to cache\n",
			ret, snand->page_addr);
		return ret;
	}

	ret = snand->write_enable(snand);
	if (ret < 0) {
		dev_err(snand->dev, "write enable command failed\n");
		return ret;
	}

	/* Get page from the device cache into our internal buffer */
	ret = snand->write_page(snand, snand->page_addr);
	if (ret < 0) {
		dev_err(snand->dev, "error %d reading page 0x%x from cache\n",
			ret, snand->page_addr);
		return ret;
	}

	return 0;
}

static int spi_nand_read_id(struct spi_nand *snand)
{
	int ret;

	ret = snand->read_id(snand, snand->data_buf);
	if (ret < 0) {
		dev_err(snand->dev, "error %d reading ID\n", ret);
		return ret;
	}
	return 0;
}

static int spi_nand_read_page(struct spi_nand *snand, unsigned int page_addr,
			      unsigned int page_offset, size_t length)
{
	unsigned int corrected = 0, ecc_error = 0;
	int ret;

	/* Load a page into the cache register */
	ret = snand->load_page(snand, page_addr);
	if (ret < 0) {
		dev_err(snand->dev, "error %d loading page 0x%x to cache\n",
			ret, page_addr);
		return ret;
	}

	ret = spi_nand_wait_till_ready(snand);
	if (ret < 0)
		return ret;

	if (snand->ecc) {
		snand->get_ecc_status(ret, &corrected, &ecc_error);
		snand->bitflips = corrected;

		/*
		 * If there's an ECC error, print a message and notify MTD
		 * about it. Then complete the read, to load actual data on
		 * the buffer (instead of the status result).
		 */
		if (ecc_error) {
			dev_err(snand->dev,
				"internal ECC error reading page 0x%x\n",
				page_addr);
			snand->mtd.ecc_stats.failed++;
		} else {
			snand->mtd.ecc_stats.corrected += corrected;
		}
	}

	/* Get page from the device cache into our internal buffer */
	ret = snand->read_cache(snand, page_offset, length, snand->data_buf);
	if (ret < 0) {
		dev_err(snand->dev, "error %d reading page 0x%x from cache\n",
			ret, page_addr);
		return ret;
	}
	return 0;
}

static u8 spi_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct spi_nand *snand = chip->priv;
	char val = 0xff;

	if (snand->buf_start < snand->buf_size)
		val = snand->data_buf[snand->buf_start++];
	return val;
}

static void spi_nand_write_buf(struct mtd_info *mtd, const u8 *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct spi_nand *snand = chip->priv;
	size_t n = min_t(size_t, len, snand->buf_size - snand->buf_start);

	memcpy(snand->data_buf + snand->buf_start, buf, n);
	snand->buf_start += n;
}

static void spi_nand_read_buf(struct mtd_info *mtd, u8 *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct spi_nand *snand = chip->priv;
	size_t n = min_t(size_t, len, snand->buf_size - snand->buf_start);

	memcpy(buf, snand->data_buf + snand->buf_start, n);
	snand->buf_start += n;
}

static int spi_nand_write_page_hwecc(struct mtd_info *mtd,
		struct nand_chip *chip, const uint8_t *buf, int oob_required)
{
	chip->write_buf(mtd, buf, mtd->writesize);
	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);

	return 0;
}

static int spi_nand_read_page_hwecc(struct mtd_info *mtd,
		struct nand_chip *chip, uint8_t *buf, int oob_required,
		int page)
{
	struct spi_nand *snand = chip->priv;

	chip->read_buf(mtd, buf, mtd->writesize);
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);

	return snand->bitflips;
}

static int spi_nand_waitfunc(struct mtd_info *mtd, struct nand_chip *chip)
{
	struct spi_nand *snand = chip->priv;
	int ret;

	ret = spi_nand_wait_till_ready(snand);

	if (ret < 0) {
		return NAND_STATUS_FAIL;
	} else if (ret & SPI_NAND_STATUS_REG_PROG_FAIL) {
		dev_err(snand->dev, "page program failed\n");
		return NAND_STATUS_FAIL;
	} else if (ret & SPI_NAND_STATUS_REG_ERASE_FAIL) {
		dev_err(snand->dev, "block erase failed\n");
		return NAND_STATUS_FAIL;
	}

	return NAND_STATUS_READY;
}

static void spi_nand_cmdfunc(struct mtd_info *mtd, unsigned int command,
			     int column, int page_addr)
{
	struct nand_chip *chip = mtd->priv;
	struct spi_nand *snand = chip->priv;

	/*
	 * In case there's any unsupported command, let's make sure
	 * we don't keep garbage around in the buffer.
	 */
	if (command != NAND_CMD_PAGEPROG) {
		spi_nand_clear_buffer(snand);
		snand->page_addr = 0;
	}

	switch (command) {
	case NAND_CMD_READ0:
		spi_nand_read_page(snand, page_addr, 0, mtd->writesize);
		break;
	case NAND_CMD_READOOB:
		spi_nand_disable_ecc(snand);
		spi_nand_read_page(snand, page_addr, mtd->writesize,
				   mtd->oobsize);
		spi_nand_enable_ecc(snand);
		break;
	case NAND_CMD_READID:
		spi_nand_read_id(snand);
		break;
	case NAND_CMD_ERASE1:
		spi_nand_erase(snand, page_addr);
		break;
	case NAND_CMD_ERASE2:
		/* There's nothing to do here, as the erase is one-step */
		break;
	case NAND_CMD_SEQIN:
		snand->buf_start = column;
		snand->page_addr = page_addr;
		break;
	case NAND_CMD_PAGEPROG:
		spi_nand_write(snand);
		break;
	case NAND_CMD_STATUS:
		spi_nand_status(snand);
		break;
	case NAND_CMD_RESET:
		spi_nand_reset(snand);
		break;
	default:
		dev_err(&mtd->dev, "unknown command 0x%x\n", command);
	}
}

static void spi_nand_select_chip(struct mtd_info *mtd, int chip)
{
	/* We need this to override the default */
}

int spi_nand_check(struct spi_nand *snand)
{
	if (!snand->dev)
		return -ENODEV;
	if (!snand->read_cache)
		return -ENODEV;
	if (!snand->load_page)
		return -ENODEV;
	if (!snand->store_cache)
		return -ENODEV;
	if (!snand->write_page)
		return -ENODEV;
	if (!snand->write_reg)
		return -ENODEV;
	if (!snand->read_reg)
		return -ENODEV;
	if (!snand->block_erase)
		return -ENODEV;
	if (!snand->reset)
		return -ENODEV;
	if (!snand->write_enable)
		return -ENODEV;
	if (!snand->write_disable)
		return -ENODEV;
	if (!snand->get_ecc_status)
		return -ENODEV;
	return 0;
}

int spi_nand_register(struct spi_nand *snand, struct nand_flash_dev *flash_ids)
{
	struct nand_chip *chip = &snand->nand_chip;
	struct mtd_part_parser_data ppdata = {};
	struct mtd_info *mtd = &snand->mtd;
	struct device_node *np = snand->dev->of_node;
	int ret;

	/* Let's check all the hooks are in-place so we don't panic later */
	ret = spi_nand_check(snand);
	if (ret)
		return ret;

	chip->priv	= snand;
	chip->read_buf	= spi_nand_read_buf;
	chip->write_buf	= spi_nand_write_buf;
	chip->read_byte	= spi_nand_read_byte;
	chip->cmdfunc	= spi_nand_cmdfunc;
	chip->waitfunc	= spi_nand_waitfunc;
	chip->select_chip = spi_nand_select_chip;
	chip->options |= NAND_NO_SUBPAGE_WRITE;
	chip->bits_per_cell = 1;

	chip->ecc.read_page	= spi_nand_read_page_hwecc;
	chip->ecc.write_page	= spi_nand_write_page_hwecc;
	chip->ecc.mode		= NAND_ECC_HW;

	if (of_get_nand_on_flash_bbt(np))
		chip->bbt_options |= NAND_BBT_USE_FLASH | NAND_BBT_NO_OOB;

	mtd->name = snand->name;
	mtd->owner = THIS_MODULE;
	mtd->priv = chip;

	/* Allocate buffer to be used to read/write the internal registers */
	snand->buf = kmalloc(SPI_NAND_CMD_BUF_LEN, GFP_KERNEL);
	if (!snand->buf)
		return -ENOMEM;

	/* Preallocate buffer for flash identification (NAND_CMD_READID) */
	snand->buf_size = SPI_NAND_CMD_BUF_LEN;
	snand->data_buf = kmalloc(snand->buf_size, GFP_KERNEL);

	ret = nand_scan_ident(mtd, 1, flash_ids);
	if (ret)
		return ret;

	/*
	 * SPI NAND has on-die ECC, which means we can correct as much as
	 * we are required to. This must be done after identification of
	 * the device.
	 */
	chip->ecc.strength = chip->ecc_strength_ds;
	chip->ecc.size = chip->ecc_step_ds;

	/*
	 * Unlock all the device before calling nand_scan_tail. This is needed
	 * in case the in-flash bad block table needs to be created.
	 * We could override __nand_unlock(), but since it's not currently used
	 * by the NAND core we call this explicitly.
	 */
	snand->buf[0] = SPI_NAND_PROT_UNLOCK_ALL;
	ret = snand->write_reg(snand, SPI_NAND_LOCK_REG, snand->buf);
	if (ret)
		return ret;

	/* Free the buffer and allocate a good one, to fit a page plus OOB */
	kfree(snand->data_buf);

	snand->buf_size = mtd->writesize + mtd->oobsize;
	snand->data_buf = kmalloc(snand->buf_size, GFP_KERNEL);
	if (!snand->data_buf)
		return -ENOMEM;

	ret = nand_scan_tail(mtd);
	if (ret)
		return ret;

	ppdata.of_node = np;
	return mtd_device_parse_register(mtd, NULL, &ppdata, NULL, 0);
}
EXPORT_SYMBOL_GPL(spi_nand_register);

void spi_nand_unregister(struct spi_nand *snand)
{
	kfree(snand->buf);
	kfree(snand->data_buf);
	nand_release(&snand->mtd);
}
EXPORT_SYMBOL_GPL(spi_nand_unregister);

MODULE_AUTHOR("Ezequiel Garcia <ezequiel.garcia@imgtec.com>");
MODULE_DESCRIPTION("Framework for SPI NAND");
MODULE_LICENSE("GPL v2");
