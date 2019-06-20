// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016-2019 RockChip, Inc.
 * Author: yifeng.zhao@rock-chips.com
 */
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>

#define	NANDC_V6_NUM_BANKS	4
#define	NANDC_V6_DEF_TIMEOUT	20000
#define	NANDC_V6_READ		0
#define	NANDC_V6_WRITE		1

#define	NANDC_REG_V6_FMCTL	0x00
#define	NANDC_REG_V6_FMWAIT	0x04
#define	NANDC_REG_V6_FLCTL	0x08
#define	NANDC_REG_V6_BCHCTL	0x0c
#define	NANDC_REG_V6_DMA_CFG	0x10
#define	NANDC_REG_V6_DMA_BUF0	0x14
#define	NANDC_REG_V6_DMA_BUF1	0x18
#define	NANDC_REG_V6_DMA_ST	0x1C
#define	NANDC_REG_V6_BCHST	0x20
#define	NANDC_REG_V6_RANDMZ	0x150
#define	NANDC_REG_V6_VER	0x160
#define	NANDC_REG_V6_INTEN	0x16C
#define	NANDC_REG_V6_INTCLR	0x170
#define	NANDC_REG_V6_INTST	0x174
#define	NANDC_REG_V6_SPARE0	0x200
#define	NANDC_REG_V6_SPARE1	0x230
#define	NANDC_REG_V6_BANK0	0x800
#define	NANDC_REG_V6_BANK1	0x900
#define	NANDC_REG_V6_BANK2	0xa00
#define	NANDC_REG_V6_BANK3	0xb00
#define	NANDC_REG_V6_SRAM0	0x1000
#define	NANDC_REG_V6_SRAM1	0x1400

#define	NANDC_REG_V6_DATA	0x00
#define	NANDC_REG_V6_ADDR	0x04
#define	NANDC_REG_V6_CMD	0x08

/* FMCTL */
#define NANDC_V6_WP		BIT(8)
#define NANDC_V6_CE_SEL_MSK	0xFF
#define NANDC_V6_CE_SEL(x)	(1 << (x))
#define NANDC_V6_RDY		BIT(9)

/* FLCTL */
#define NANDC_V6_FL_RST		BIT(0)
#define NANDC_V6_FL_DIR_S	0x1
#define NANDC_V6_FL_XFER_START	BIT(2)
#define NANDC_V6_FL_XFER_EN	BIT(3)
#define NANDC_V6_FL_ST_BUF_S	0x4
#define NANDC_V6_FL_XFER_COUNT	BIT(5)
#define NANDC_V6_FL_ACORRECT	BIT(10)
#define NANDC_V6_FL_XFER_READY	BIT(20)

/* BCHCTL */
#define NAND_V6_BCH_REGION_S	0x5
#define NAND_V6_BCH_REGION_M	0x7

/* BCHST */
#define NANDC_V6_BCH0_ST_ERR	BIT(2)
#define NANDC_V6_BCH1_ST_ERR	BIT(15)
#define NANDC_V6_ECC_ERR_CNT0(x) (((((x) & (0x1F << 3)) >> 3) \
				| (((x) & (1 << 27)) >> 22)) & 0x3F)
#define NANDC_V6_ECC_ERR_CNT1(x) (((((x) & (0x1F << 16)) >> 16) \
				| (((x) & (1 << 29)) >> 24)) & 0x3F)

#define NANDC_V6_INT_DMA	BIT(0)

/*
 * NAND Controller structure: stores rk nand controller information
 *
 * @chip:		nand chip info
 * @mtd			mtd info
 * @dev:		parent device (used to print error messages)
 * @regs:		NAND controller registers
 * @hclk:		NAND Controller ahb clock
 * @clk:		NAND Controller interface clock
 * @gclk:		NAND Controller clock gate
 * @ecc_mode:		NAND Controller current ecc mode
 * @selected_bank:	NAND Controller current selected bank
 * @clk_rate:		NAND controller current clock rate
 * @oob_buf:		temp buffer for oob read and write
 * @page_buf:		temp buffer for page read and write
 * @complete:		a completion object used to wait for NAND
 *			controller events
 */
struct rk_nandc {
	struct nand_chip chip;
	struct mtd_info mtd;
	struct device *dev;
	void __iomem *regs;
	struct clk *hclk;
	struct clk *clk;
	struct clk *gclk;
	int ecc_mode;
	int max_ecc_strength;
	unsigned char banks[NANDC_V6_NUM_BANKS];
	int selected_bank;
	unsigned long clk_rate;
	u32 *oob_buf;
	u32 *page_buf;
	struct completion complete;
};

static inline struct rk_nandc *mtd_to_rk_nandc(struct mtd_info *mtd)
{
	return container_of(mtd, struct rk_nandc, mtd);
}

static void rk_nandc_init(struct rk_nandc *nandc)
{
	writel(0, nandc->regs + NANDC_REG_V6_RANDMZ);
	writel(0, nandc->regs + NANDC_REG_V6_DMA_CFG);
	writel(NANDC_V6_WP, nandc->regs + NANDC_REG_V6_FMCTL);
	writel(NANDC_V6_FL_RST, nandc->regs + NANDC_REG_V6_FLCTL);
	writel(0x1081, nandc->regs + NANDC_REG_V6_FMWAIT);
}

static irqreturn_t rk_nandc_interrupt(int irq, void *dev_id)
{
	struct rk_nandc *nandc = dev_id;
	u32 st = readl(nandc->regs + NANDC_REG_V6_INTST);
	u32 ien = readl(nandc->regs + NANDC_REG_V6_INTEN);

	if (!(ien & st))
		return IRQ_NONE;

	if ((ien & st) == ien)
		complete(&nandc->complete);

	writel(st, nandc->regs + NANDC_REG_V6_INTCLR);
	writel(~st & ien, nandc->regs + NANDC_REG_V6_INTEN);

	return IRQ_HANDLED;
}

static void rk_nandc_select_chip(struct mtd_info *mtd, int chipnr)
{
	struct rk_nandc *nandc = mtd_to_rk_nandc(mtd);
	struct nand_chip *chip = mtd->priv;
	u32 reg;
	int banknr;
	void __iomem *bank_base;

	reg = readl(nandc->regs + NANDC_REG_V6_FMCTL);
	reg &= ~NANDC_V6_CE_SEL_MSK;

	if (chipnr == -1) {
		banknr = -1;
	} else {
		banknr = nandc->banks[chipnr];
		bank_base = nandc->regs + NANDC_REG_V6_BANK0 + banknr * 0x100;

		chip->IO_ADDR_R = bank_base;
		chip->IO_ADDR_W = bank_base;

		reg |= NANDC_V6_CE_SEL(banknr);
	}
	writel(reg, nandc->regs + NANDC_REG_V6_FMCTL);

	nandc->selected_bank = banknr;
}

static void rk_nandc_cmd_ctrl(struct mtd_info *mtd, int dat, unsigned int ctrl)
{
	struct rk_nandc *nandc = mtd_to_rk_nandc(mtd);
	struct nand_chip *chip = mtd->priv;
	u32 reg;
	void __iomem *bank_base = nandc->regs + NANDC_REG_V6_BANK0
				+ nandc->selected_bank * 0x100;

	WARN_ON(nandc->selected_bank < 0);

	if (ctrl & NAND_CTRL_CHANGE) {
		WARN_ON((ctrl & NAND_ALE) && (ctrl & NAND_CLE));
		if (ctrl & NAND_ALE)
			bank_base += NANDC_REG_V6_ADDR;
		else if (ctrl & NAND_CLE)
			bank_base += NANDC_REG_V6_CMD;
		chip->IO_ADDR_W = bank_base;

		reg = readl(nandc->regs + NANDC_REG_V6_FMCTL);
		reg &= ~NANDC_V6_CE_SEL_MSK;
		if (ctrl & NAND_NCE)
			reg |= NANDC_V6_CE_SEL(nandc->selected_bank);
		writel(reg, nandc->regs + NANDC_REG_V6_FMCTL);
	}

	if (dat != NAND_CMD_NONE)
		writeb(dat & 0xFF, chip->IO_ADDR_W);
}

static void rk_nandc_xfer_start(struct rk_nandc *nandc, u8 dir, u8 n_KB,
				dma_addr_t dma_data, dma_addr_t dma_oob)
{
	u32 reg;

	reg = readl(nandc->regs + NANDC_REG_V6_BCHCTL);
	reg = (reg & (~(0x7 << 5))) | (nandc->selected_bank << 5);
	writel(reg, nandc->regs + NANDC_REG_V6_BCHCTL);

	reg = (1 << 0) | ((!dir) << 1) | (1 << 2) | (2 << 3) | (7 << 6) |
	      (16 << 9);
	writel(reg, nandc->regs + NANDC_REG_V6_DMA_CFG);
	writel(dma_data, nandc->regs + NANDC_REG_V6_DMA_BUF0);
	writel(dma_oob, nandc->regs + NANDC_REG_V6_DMA_BUF1);

	reg = (dir << 1) | (1 << 3) | (1 << 5) | (1 << 10) | (n_KB << 22) |
	      (1 << 29);
	writel(reg, nandc->regs + NANDC_REG_V6_FLCTL);
	reg |= (1 << 2);
	writel(reg, nandc->regs + NANDC_REG_V6_FLCTL);
}

static int rk_nand_wait_for_xfer_done(struct rk_nandc *nandc)
{
	u32 reg;
	int ret;
	void __iomem *ptr = nandc->regs + NANDC_REG_V6_FLCTL;

	ret = readl_poll_timeout_atomic(ptr, reg,
					reg & NANDC_V6_FL_XFER_READY,
					1, 10000);
	if (ret)
		pr_err("timeout reg=%x\n", reg);
	return ret;
}

static unsigned long rk_nand_dma_map_single(void *ptr, int size, int dir)
{
#ifdef CONFIG_ARM64
	__dma_map_area((void *)ptr, size, dir);
	return ((unsigned long)virt_to_phys((void *)ptr));
#else
	return dma_map_single(NULL, (void *)ptr, size, dir);
#endif
}

static void rk_nand_dma_unmap_single(unsigned long ptr, int size, int dir)
{
#ifdef CONFIG_ARM64
	__dma_unmap_area(phys_to_virt(ptr), size, dir);
#else
	dma_unmap_single(NULL, (dma_addr_t)ptr, size, dir);
#endif
}

static int rk_nandc_hw_syndrome_ecc_read_page(struct mtd_info *mtd,
					      struct nand_chip *chip,
					      u8 *buf,
					      int oob_required, int page)
{
	struct rk_nandc *nandc = mtd_to_rk_nandc(mtd);
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	int max_bitflips = 0;
	dma_addr_t dma_data, dma_oob;
	int ret, i;
	int bch_st;
	int dma_oob_size = ecc->steps * 128;

	dma_data = rk_nand_dma_map_single(nandc->page_buf, mtd->writesize,
					  DMA_FROM_DEVICE);
	dma_oob = rk_nand_dma_map_single(nandc->oob_buf, dma_oob_size,
					 DMA_FROM_DEVICE);

	init_completion(&nandc->complete);
	writel(NANDC_V6_INT_DMA, nandc->regs + NANDC_REG_V6_INTEN);
	rk_nandc_xfer_start(nandc, 0, ecc->steps, dma_data, dma_oob);
	wait_for_completion_timeout(&nandc->complete, msecs_to_jiffies(5));
	rk_nand_wait_for_xfer_done(nandc);
	rk_nand_dma_unmap_single(dma_data, mtd->writesize, DMA_FROM_DEVICE);
	rk_nand_dma_unmap_single(dma_oob, dma_oob_size, DMA_FROM_DEVICE);

	if (oob_required) {
		u8 oob_step = (nandc->ecc_mode <= 24) ? 64 : 128;
		u8 *oob;
		u32 tmp;

		for (i = 0; i < ecc->steps; i++) {
			oob = chip->oob_poi + i * (ecc->bytes + 4);
			tmp = nandc->oob_buf[i * oob_step / 4];
			*oob++ = (u8)tmp;
			*oob++ = (u8)(tmp >> 8);
			*oob++ = (u8)(tmp >> 16);
			*oob++ = (u8)(tmp >> 24);
		}
	}

	for (i = 0; i < ecc->steps / 2; i++) {
		bch_st = readl(nandc->regs + NANDC_REG_V6_BCHST + i * 4);
		if (bch_st & NANDC_V6_BCH0_ST_ERR ||
		    bch_st & NANDC_V6_BCH1_ST_ERR) {
			mtd->ecc_stats.failed++;
			max_bitflips = -1;
		} else {
			ret = NANDC_V6_ECC_ERR_CNT0(bch_st);
			mtd->ecc_stats.corrected += ret;
			max_bitflips = max_t(unsigned int, max_bitflips, ret);

			ret = NANDC_V6_ECC_ERR_CNT1(bch_st);
			mtd->ecc_stats.corrected += ret;
			max_bitflips = max_t(unsigned int, max_bitflips, ret);
		}
	}
	memcpy(buf, nandc->page_buf, mtd->writesize);

	if (max_bitflips == -1) {
		dev_err(nandc->dev, "read_page %x %x %x %x %x %p %x\n",
			page, max_bitflips, bch_st, ((u32 *)buf)[0],
			((u32 *)buf)[1], buf, (u32)dma_data);
	}
	return max_bitflips;
}

static int rk_nandc_hw_syndrome_ecc_write_page(struct mtd_info *mtd,
					       struct nand_chip *chip,
					       const u8 *buf,
					       int oob_required, int page)
{
	struct rk_nandc *nandc = mtd_to_rk_nandc(mtd);
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	dma_addr_t dma_data, dma_oob;
	int i;
	int dma_oob_size = ecc->steps * 64;

	for (i = 0; i < ecc->steps; i++) {
		u32 tmp;

		if (oob_required) {
			u8 *oob;

			oob = chip->oob_poi + i * (ecc->bytes + 4);
			tmp = oob[0] | (oob[1] << 8) | (oob[1] << 16) |
				(oob[1] << 24);
		} else {
			tmp = 0xFFFFFFFF;
		}
		nandc->oob_buf[i] = tmp;
	}

	memcpy(nandc->page_buf, buf, mtd->writesize);
	dma_data = rk_nand_dma_map_single((void *)nandc->page_buf,
					  mtd->writesize, DMA_TO_DEVICE);
	dma_oob = rk_nand_dma_map_single(nandc->oob_buf, dma_oob_size,
					 DMA_TO_DEVICE);
	init_completion(&nandc->complete);
	writel(NANDC_V6_INT_DMA, nandc->regs + NANDC_REG_V6_INTEN);
	rk_nandc_xfer_start(nandc, 1, ecc->steps, dma_data, dma_oob);
	wait_for_completion_timeout(&nandc->complete, msecs_to_jiffies(10));
	rk_nand_wait_for_xfer_done(nandc);
	rk_nand_dma_unmap_single(dma_data, mtd->writesize, DMA_TO_DEVICE);
	rk_nand_dma_unmap_single(dma_oob, dma_oob_size, DMA_TO_DEVICE);
	return 0;
}

static int rk_nandc_hw_ecc_read_oob(struct mtd_info *mtd,
				    struct nand_chip *chip,
				    int page)
{
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);

	chip->pagebuf = -1;

	return chip->ecc.read_page(mtd, chip, chip->buffers->databuf, 1, page);
}

static int rk_nandc_hw_ecc_write_oob(struct mtd_info *mtd,
				     struct nand_chip *chip,
				     int page)
{
	int ret, status;

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0, page);

	chip->pagebuf = -1;

	memset(chip->buffers->databuf, 0xff, mtd->writesize);
	ret = chip->ecc.write_page(mtd, chip, chip->buffers->databuf, 1, page);
	if (ret)
		return ret;

	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

static void rk_nandc_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct rk_nandc *nandc = mtd_to_rk_nandc(mtd);
	int offs = 0;
	void __iomem *bank_base = nandc->regs + NANDC_REG_V6_BANK0
				+ nandc->selected_bank * 0x100;

	for (offs = 0; offs < len; offs++)
		buf[offs] = readl(bank_base);
}

static void rk_nandc_write_buf(struct mtd_info *mtd, const uint8_t *buf,
			       int len)
{
	struct rk_nandc *nandc = mtd_to_rk_nandc(mtd);
	int offs = 0;
	void __iomem *bank_base = nandc->regs + NANDC_REG_V6_BANK0
				+ nandc->selected_bank * 0x100;

	for (offs = 0; offs < len; offs++)
		writeb(buf[offs], bank_base);
}

static u8 rk_nandc_read_byte(struct mtd_info *mtd)
{
	u8 ret;

	rk_nandc_read_buf(mtd, &ret, 1);

	return ret;
}

static struct nand_ecclayout nand_oob_fix = {
	.eccbytes = 28,
	.eccpos = {
		   4, 5, 6, 7, 8, 9, 10},
	.oobfree = {
		{.offset = 0,
		 .length = 4} }
};

static int rk_nandc_hw_ecc_setup(struct mtd_info *mtd,
				 struct nand_ecc_ctrl *ecc,
				 uint32_t strength)
{
	struct rk_nandc *nandc = mtd_to_rk_nandc(mtd);
	u32 reg;

	ecc->strength = strength;
	ecc->bytes = DIV_ROUND_UP(ecc->strength * 14, 8);
	/* HW ECC always work with even numbers of ECC bytes */
	ecc->bytes = ALIGN(ecc->bytes, 2);

	switch (ecc->strength) {
	case 60:
		reg = 0x00040010;
		break;
	case 40:
		reg = 0x00040000;
		break;
	case 24:
		reg = 0x00000010;
		break;
	case 16:
		reg = 0x00000000;
		break;
	default:
		return -EINVAL;
	}
	writel(reg, nandc->regs + NANDC_REG_V6_BCHCTL);

	return 0;
}

static int rk_nandc_hw_ecc_ctrl_init(struct mtd_info *mtd,
				     struct nand_ecc_ctrl *ecc)
{
	static const u8 strengths[] = {60, 40, 24, 16};
	struct rk_nandc *nandc = mtd_to_rk_nandc(mtd);
	int max_strength;
	u32 i, ver;

	ecc->size = 1024;
	ecc->prepad = 4;
	ecc->steps = mtd->writesize / ecc->size;
	max_strength = ((mtd->oobsize / ecc->steps) - 4) * 8 / 14;
	nandc->max_ecc_strength = 60;

	ver = readl(nandc->regs + NANDC_REG_V6_VER);
	if (ver == 0x801)
		nandc->max_ecc_strength = 16;
	else if (ver == 0x56363232 || ver == 0x56383030 || ver == 0x56363030)
		nandc->max_ecc_strength = 60;
	else
		dev_err(nandc->dev, "unsupported nandc version %x\n", ver);

	if (max_strength > nandc->max_ecc_strength)
		max_strength = nandc->max_ecc_strength;

	nandc->page_buf = kmalloc(mtd->writesize, GFP_KERNEL | GFP_DMA);
	if (!nandc->page_buf)
		return -ENOMEM;
	nandc->oob_buf = kmalloc(ecc->steps * 128, GFP_KERNEL | GFP_DMA);
	if (!nandc->oob_buf) {
		kfree(nandc->page_buf);
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(strengths); i++) {
		if (max_strength >= strengths[i])
			break;
	}

	if (i >= ARRAY_SIZE(strengths)) {
		dev_err(nandc->dev, "unsupported strength\n");
		return -ENOTSUPP;
	}

	nandc->ecc_mode = strengths[i];
	rk_nandc_hw_ecc_setup(mtd, ecc, nandc->ecc_mode);

	nand_oob_fix.eccbytes = ecc->bytes * ecc->steps;
	for (i = 0; i < (u32)ecc->bytes; i++)
		nand_oob_fix.eccpos[i] = i + 4;
	ecc->layout = &nand_oob_fix;

	if (mtd->oobsize < (u32)(ecc->bytes + 4) * ecc->steps)
		return -EINVAL;

	return 0;
}

static int rk_nandc_ecc_init(struct mtd_info *mtd, struct nand_ecc_ctrl *ecc)
{
	int ret;

	switch (ecc->mode) {
	case NAND_ECC_HW_SYNDROME:
		ret = rk_nandc_hw_ecc_ctrl_init(mtd, ecc);
		if (ret)
			return ret;
		ecc->read_page = rk_nandc_hw_syndrome_ecc_read_page;
		ecc->write_page = rk_nandc_hw_syndrome_ecc_write_page;
		ecc->read_oob = rk_nandc_hw_ecc_read_oob;
		ecc->write_oob = rk_nandc_hw_ecc_write_oob;
		break;
	case NAND_ECC_HW:
	case NAND_ECC_NONE:
	case NAND_ECC_SOFT:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int rk_nandc_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	int page, res = 0, i;
	struct nand_chip *chip = (struct nand_chip *)mtd->priv;
	u16 bad = 0xff;
	u8 *data_buf = chip->buffers->databuf;
	int chipnr = (int)(ofs >> chip->chip_shift);

	page = (int)(ofs >> chip->page_shift) & chip->pagemask;
	chip->select_chip(mtd, chipnr);
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0x00, page);
	if (rk_nandc_hw_syndrome_ecc_read_page(mtd, chip, data_buf, 0,
					       page) == -1) {
		chip->cmdfunc(mtd, NAND_CMD_READOOB, chip->badblockpos, page);
		for (i = 0; i < 8; i++) {
			bad = chip->read_byte(mtd);
			if (bad)
				break;
		}
		if (i >= 8)
			res = 1;
	}
	chip->select_chip(mtd, -1);
	return res;
}

static int rk_nandc_dev_ready(struct mtd_info *mtd)
{
	struct rk_nandc *nandc = mtd_to_rk_nandc(mtd);
	u32 reg;

	reg = readl(nandc->regs + NANDC_REG_V6_FMCTL);

	return (reg & NANDC_V6_RDY);
}

static int rk_nandc_chips_init(struct device *dev, struct rk_nandc *nandc)
{
	struct nand_chip *chip;
	struct mtd_info *mtd;
	size_t chipnr, bank_idx;
	int nand_maf_id, nand_dev_id;
	int ret;

	mtd		= &nandc->mtd;
	chip		= &nandc->chip;
	mtd->priv	= chip;
	mtd->owner	= THIS_MODULE;
	mtd->name	= "rk-nand";

	chip->ecc.mode = NAND_ECC_HW_SYNDROME;

	chip->chip_delay = 25;
	chip->select_chip = rk_nandc_select_chip;
	chip->cmd_ctrl = rk_nandc_cmd_ctrl;
	chip->read_buf = rk_nandc_read_buf;
	chip->write_buf = rk_nandc_write_buf;
	chip->read_byte = rk_nandc_read_byte;
	chip->block_bad = rk_nandc_block_bad;
	chip->dev_ready = rk_nandc_dev_ready;
	chip->bbt_options = NAND_BBT_USE_FLASH | NAND_BBT_NO_OOB;
	chip->options = NAND_NO_SUBPAGE_WRITE;

	chipnr = 0;
	for (bank_idx = 0; bank_idx < NANDC_V6_NUM_BANKS; bank_idx++) {
		nandc->banks[chipnr] = bank_idx;
		if (chipnr == 0) {
			ret = nand_scan_ident(mtd, 1, NULL);
			if (ret)
				continue;
			chip->select_chip(mtd, 0);
			chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);
			chip->cmdfunc(mtd, NAND_CMD_READID, 0x00, -1);
			nand_maf_id = chip->read_byte(mtd);
			nand_dev_id = chip->read_byte(mtd);
			chip->select_chip(mtd, -1);
		} else {
			chip->select_chip(mtd, chipnr);
			chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);
			chip->cmdfunc(mtd, NAND_CMD_READID, 0x00, -1);
			if (nand_maf_id != chip->read_byte(mtd) ||
			    nand_dev_id != chip->read_byte(mtd)) {
				chip->select_chip(mtd, -1);
				continue;
			}
			chip->select_chip(mtd, -1);
			chip->numchips++;
			mtd->size += chip->chipsize;
		}
		chipnr++;
	}

	if (chipnr == 0) {
		dev_err(dev, "No NAND chips found\n");
		return -ENODEV;
	}

	rk_nandc_ecc_init(mtd, &chip->ecc);
	ret = nand_scan_tail(mtd);
	if (ret) {
		dev_err(dev,  "Failed to scan NAND: %d\n", ret);
		return ret;
	}
	ret = mtd_device_parse_register(mtd, NULL, NULL, NULL, 0);
	if (ret) {
		dev_err(dev, "failed to register mtd device: %d\n", ret);
		nand_release(mtd);
		return ret;
	}

	return 0;
}

static int rk_nandc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *r;
	struct rk_nandc *nandc;
	int irq;
	int ret;
	int clock_frequency;

	nandc = devm_kzalloc(dev, sizeof(*nandc), GFP_KERNEL);
	if (!nandc)
		return -ENOMEM;

	nandc->dev = dev;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	nandc->regs = devm_ioremap_resource(dev, r);
	if (IS_ERR(nandc->regs))
		return PTR_ERR(nandc->regs);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "failed to retrieve irq\n");
		return irq;
	}

	nandc->hclk = devm_clk_get(dev, "hclk_nandc");
	if (IS_ERR(nandc->hclk)) {
		dev_err(dev, "failed to retrieve hclk_nandc %p\n", nandc->hclk);
		return PTR_ERR(nandc->hclk);
	}

	ret = clk_prepare_enable(nandc->hclk);
	if (ret)
		return ret;

	nandc->clk = devm_clk_get(dev, "clk_nandc");
	if (IS_ERR(nandc->clk)) {
		dev_err(dev, "failed to retrieve nandc clk\n");
		ret = PTR_ERR(nandc->clk);
		goto out_ahb_clk_unprepare;
	}

	if (of_property_read_u32(dev->of_node, "clock-frequency",
				 &clock_frequency))
		clock_frequency = 150 * 1000 * 1000;
	clk_set_rate(nandc->clk, clock_frequency);

	ret = clk_prepare_enable(nandc->clk);
	if (ret)
		goto out_ahb_clk_unprepare;
	nandc->clk_rate = clk_get_rate(nandc->clk);

	nandc->gclk = devm_clk_get(&pdev->dev, "g_clk_nandc");
	if (!(IS_ERR(nandc->gclk)))
		clk_prepare_enable(nandc->gclk);

	rk_nandc_init(nandc);

	writel(0, nandc->regs + NANDC_REG_V6_INTEN);
	ret = devm_request_irq(dev, irq, rk_nandc_interrupt,
			       0, "rk-nand", nandc);
	if (ret)
		goto out_nandc_clk_unprepare;

	platform_set_drvdata(pdev, nandc);

	ret = rk_nandc_chips_init(dev, nandc);
	if (ret) {
		dev_err(dev, "failed to init nand chips\n");
		goto out_nandc_clk_unprepare;
	}
	return 0;

out_nandc_clk_unprepare:
	clk_disable_unprepare(nandc->clk);
out_ahb_clk_unprepare:
	clk_disable_unprepare(nandc->hclk);

	return ret;
}

static int rk_nandc_remove(struct platform_device *pdev)
{
	struct rk_nandc *nandc = platform_get_drvdata(pdev);

	kfree(nandc->page_buf);
	kfree(nandc->oob_buf);
	clk_disable_unprepare(nandc->clk);
	clk_disable_unprepare(nandc->hclk);
	return 0;
}

static const struct of_device_id rk_nandc_ids[] = {
	{.compatible = "rockchip,rk-nandc"},
	{.compatible = "rockchip,rk-nandc-v6"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rk_nandc_ids);

static struct platform_driver rk_nandc_driver = {
	.driver = {
		.name = "rk-nand",
		.of_match_table = rk_nandc_ids,
	},
	.probe = rk_nandc_probe,
	.remove = rk_nandc_remove,
};
module_platform_driver(rk_nandc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yifeng Zhao <zyf@rock-chips.com>");
MODULE_DESCRIPTION("MTD NAND driver for Rockchip SoC");
