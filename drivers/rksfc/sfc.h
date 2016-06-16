/* driver/rksfc/sfc.h
 *
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _SFC_H
#define _SFC_H

#define SFC_MAX_IOSIZE		(1024*8)    /* 8K byte */
#define SFC_EN_INT		(0)         /* enable interrupt */
#define SFC_EN_DMA		(1)         /* enable dma */
#define SFC_FIFO_DEPTH		(0x10)      /* 16 words */

/* FIFO watermark */
#define SFC_RX_WMARK		(SFC_FIFO_DEPTH)	/* RX watermark level */
#define SFC_TX_WMARK		(SFC_FIFO_DEPTH)	/* TX watermark level */
#define SFC_RX_WMARK_SHIFT	(8)
#define SFC_TX_WMARK_SHIFT	(0)

/*return value*/
#define SFC_OK                      (0)
#define SFC_ERROR                   (-1)
#define SFC_PARAM_ERR               (-2)
#define SFC_TX_TIMEOUT              (-3)
#define SFC_RX_TIMEOUT              (-4)
#define SFC_WAIT_TIMEOUT            (-5)
#define SFC_BUSY_TIMEOUT            (-6)
#define SFC_ECC_FAIL                (-7)
#define SFC_PROG_FAIL               (-8)
#define SFC_ERASE_FAIL              (-9)

/* SFC_CMD Register */
#define SFC_ADDR_0BITS              (0)
#define SFC_ADDR_24BITS             (1)
#define SFC_ADDR_32BITS             (2)
#define SFC_ADDR_XBITS              (3)

#define SFC_WRITE                   (1)
#define SFC_READ                    (0)

/* SFC_CTRL Register */
#define SFC_1BITS_LINE              (0)
#define SFC_2BITS_LINE              (1)
#define SFC_4BITS_LINE              (2)

#define SFC_ENABLE_DMA              (1<<14)

/*------------------------------ Global Typedefs -----------------------------*/
enum SFC_DATA_LINES {
	DATA_LINES_X1 = 0,
	DATA_LINES_X2,
	DATA_LINES_X4
};

union SFCCTRL_DATA {
	/* raw register data */
	u32 d32;
	/* register bits */
	struct {
		/* spi mode select */
		unsigned mode : 1;
		/*
		 * Shift in phase selection
		 * 0: shift in the flash data at posedge sclk_out
		 * 1: shift in the flash data at negedge sclk_out
		 */
		unsigned sps : 1;
		unsigned reserved3_2 : 2;
		/* sclk_idle_level_cycles */
		unsigned scic : 4;
		/* Cmd bits number */
		unsigned cmdlines : 2;
		/* Address bits number */
		unsigned addrlines : 2;
		/* Data bits number */
		unsigned datalines : 2;
		/* this bit is not exit in regiseter, just use for code param */
		unsigned enbledma : 1;
		unsigned reserved15 : 1;
		unsigned addrbits : 5;
		unsigned reserved31_21 : 11;
	} b;
};

union SFCCMD_DATA {
	/* raw register data */
	u32 d32;
	/* register bits */
	struct {
		/* Command that will send to Serial Flash */
		unsigned cmd : 8;
		/* Dummy bits number */
		unsigned dummybits : 4;
		/* 0: read, 1: write */
		unsigned rw : 1;
		/* Continuous read mode */
		unsigned readmode : 1;
		/* Address bits number */
		unsigned addrbits : 2;
		/* Transferred bytes number */
		unsigned datasize : 14;
		/* Chip select */
		unsigned cs : 2;
	} b;
};

#define sfc_delay(us)	udelay(us)
int sfc_init(void __iomem *reg_addr);
int sfc_request(u32 sfcmd, u32 sfctrl, u32 addr, void *data);
void sfc_handle_irq(void);
int rksfc_get_reg_addr(unsigned long *p_sfc_addr);
unsigned long rksfc_dma_map_single(unsigned long ptr, int size, int dir);
void rksfc_dma_unmap_single(unsigned long ptr, int size, int dir);
void sfc_clean_irq(void);
void rk_sfc_irq_flag_init(void);
void wait_for_sfc_irq_completed(void);
#endif
