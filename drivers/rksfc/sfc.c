/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include "typedef.h"
#include "sfc.h"

#define DMA_INT		BIT(7)      /* dma interrupt */
#define NSPIERR_INT	BIT(6)      /* Nspi error interrupt */
#define AHBERR_INT	BIT(5)      /* Ahb bus error interrupt */
#define FINISH_INT	BIT(4)      /* Transfer finish interrupt */
#define TXEMPTY_INT	BIT(3)      /* Tx fifo empty interrupt */
#define TXOF_INT	BIT(2)      /* Tx fifo overflow interrupt */
#define RXUF_INT	BIT(1)      /* Rx fifo underflow interrupt */
#define RXFULL_INT	BIT(0)      /* Rx fifo full interrupt */

/* SFC_FSR Register*/
#define SFC_RXFULL	BIT(3)      /* rx fifo full */
#define SFC_RXEMPTY	BIT(2)      /* rx fifo empty */
#define SFC_TXEMPTY	BIT(1)      /* tx fifo empty */
#define SFC_TXFULL	BIT(0)      /* tx fifo full */

/* SFC_RCVR Register */
#define SFC_RESET	BIT(0)     /* controller reset */

/* SFC_SR Register */
/* sfc busy flag. When busy, don't try to set the control register */
#define SFC_BUSY	BIT(0)

/* SFC_DMA_TRIGGER Register */
/* Dma start trigger signal. Auto cleared after write */
#define SFC_DMA_START	BIT(0)

#define	SFC_CTRL	0x00
#define	SFC_IMR		0x04
#define	SFC_ICLR	0x08
#define	SFC_FTLR	0x0C
#define	SFC_RCVR	0x10
#define	SFC_AX		0x14
#define	SFC_ABIT	0x18
#define	SFC_MASKISR	0x1C
#define SFC_FSR		0x20
#define SFC_SR		0x24
#define SFC_RAWISR	0x28
#define SFC_DMA_TRIGGER	0x80
#define SFC_DMA_ADDR	0x84
#define SFC_CMD		0x100
#define SFC_ADDR	0x104
#define SFC_DATA	0x108

union SFCFSR_DATA {
	u32 d32;
	struct {
		unsigned txempty : 1;
		unsigned txfull :  1;
		unsigned rxempty : 1;
		unsigned rxfull :  1;
		unsigned reserved7_4 : 4;
		unsigned txlevel : 5;
		unsigned reserved15_13 : 3;
		unsigned rxlevel : 5;
		unsigned reserved31_21 : 11;
	} b;
};

void __iomem *g_sfc_reg;

static void sfc_reset(void)
{
	int timeout = 10000;

	writel(SFC_RESET, g_sfc_reg + SFC_RCVR);
	while ((readl(g_sfc_reg + SFC_RCVR) == SFC_RESET) && (timeout > 0)) {
		sfc_delay(1);
		timeout--;
	}
	writel(0xFFFFFFFF, g_sfc_reg + SFC_ICLR);
}

int sfc_init(void __iomem *reg_addr)
{
	g_sfc_reg = reg_addr;
	sfc_reset();
	writel(0, g_sfc_reg + SFC_CTRL);
	return OK;
}

void sfc_clean_irq(void)
{
	writel(0xFFFFFFFF, g_sfc_reg + SFC_ICLR);
	writel(0xFFFFFFFF, g_sfc_reg + SFC_IMR);
}

int sfc_request(u32 sfcmd, u32 sfctrl, u32 addr, void *data)
{
	int ret = SFC_OK;
	union SFCCMD_DATA cmd;
	int reg;
	int timeout = 0;

	reg = readl(g_sfc_reg + SFC_FSR);
	if (!(reg & SFC_TXEMPTY) || !(reg & SFC_RXEMPTY) ||
	    (readl(g_sfc_reg + SFC_SR) & SFC_BUSY))
		sfc_reset();

	cmd.d32 = sfcmd;
	if (SFC_ADDR_XBITS == cmd.b.addrbits) {
		union SFCCTRL_DATA ctrl;

		ctrl.d32 = sfctrl;
		if (!ctrl.b.addrbits)
			return SFC_PARAM_ERR;
		/* Controller plus 1 automatically */
		writel(ctrl.b.addrbits - 1, g_sfc_reg + SFC_ABIT);
	}

	writel(sfctrl, g_sfc_reg + SFC_CTRL);
	writel(sfcmd, g_sfc_reg + SFC_CMD);
	if (cmd.b.addrbits)
		writel(addr, g_sfc_reg + SFC_ADDR);
	if (!cmd.b.datasize)
		goto exit_wait;
	if (SFC_ENABLE_DMA & sfctrl) {
		unsigned long dma_addr;
		u8 direction = (SFC_WRITE == cmd.b.rw) ? 1 : 0;

		dma_addr = rksfc_dma_map_single((unsigned long)data,
						cmd.b.datasize,
						direction);
		rk_sfc_irq_flag_init();
		writel(0xFFFFFFFF, g_sfc_reg + SFC_ICLR);
		writel(~(FINISH_INT), g_sfc_reg + SFC_IMR);
		writel((u32)dma_addr, g_sfc_reg + SFC_DMA_ADDR);
		writel(SFC_DMA_START, g_sfc_reg + SFC_DMA_TRIGGER);

		timeout = cmd.b.datasize * 10;
		wait_for_sfc_irq_completed();
		while ((readl(g_sfc_reg + SFC_SR) & SFC_BUSY) &&
		       (timeout-- > 0))
			sfc_delay(1);
		writel(0xFFFFFFFF, g_sfc_reg + SFC_ICLR);
		if (timeout <= 0)
			ret = SFC_WAIT_TIMEOUT;
		direction = (SFC_WRITE == cmd.b.rw) ? 1 : 0;
		rksfc_dma_unmap_single(dma_addr,
				       cmd.b.datasize,
				       direction);
	} else {
		u32 i, words, count, bytes;
		union SFCFSR_DATA    fifostat;
		u32 *p_data = (u32 *)data;

		if (SFC_WRITE == cmd.b.rw) {
			words  = (cmd.b.datasize + 3) >> 2;
			while (words) {
				fifostat.d32 = readl(g_sfc_reg + SFC_FSR);
				if (fifostat.b.txlevel > 0) {
					count = MIN(words, fifostat.b.txlevel);
					for (i = 0; i < count; i++) {
						writel(*p_data++,
						       g_sfc_reg + SFC_DATA);
						words--;
					}
					if (0 == words)
						break;
					timeout = 0;
				} else {
					sfc_delay(1);
					if (timeout++ > 10000) {
						ret = SFC_TX_TIMEOUT;
						break;
					}
				}
			}
		} else {
			/* SFC_READ == cmd.b.rw */
			bytes = cmd.b.datasize & 0x3;
			words = cmd.b.datasize >> 2;
			while (words) {
				fifostat.d32 = readl(g_sfc_reg + SFC_FSR);
				if (fifostat.b.rxlevel > 0) {
					u32 count = MIN(words,
							fifostat.b.rxlevel);

					for (i = 0; i < count; i++) {
						*p_data++ = readl(g_sfc_reg +
								  SFC_DATA);
						words--;
					}
					if (0 == words)
						break;
					timeout = 0;
				} else {
					sfc_delay(1);
					if (timeout++ > 10000) {
						ret = SFC_RX_TIMEOUT;
						break;
					}
				}
			}

			timeout = 0;
			while (bytes) {
				fifostat.d32 = readl(g_sfc_reg + SFC_FSR);
				if (fifostat.b.rxlevel > 0) {
					u8 *p_data1 = (u8 *)p_data;

					words = readl(g_sfc_reg + SFC_DATA);
					for (i = 0; i < bytes; i++)
						p_data1[i] =
						(u8)((words >> (i * 8)) & 0xFF);
					break;
				}

				sfc_delay(1);
				if (timeout++ > 10000) {
					ret = SFC_RX_TIMEOUT;
					break;
				}
			}
		}
	}

exit_wait:
	timeout = 0;    /* wait cmd or data send complete */
	while (!(readl(g_sfc_reg + SFC_FSR) & SFC_TXEMPTY)) {
		sfc_delay(1);
		if (timeout++ > 100000) {         /* wait 100ms */
			ret = SFC_TX_TIMEOUT;
			break;
		}
	}
	sfc_delay(1); /* CS# High Time (read/write) >100ns */
	return ret;
}

