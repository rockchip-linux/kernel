// SPDX-License-Identifier: GPL-2.0

/* Copyright (c) 2018 Rockchip Electronics Co., Ltd. */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>

#ifdef CONFIG_RK_SFC_NOR_MTD_PANIC_WRITE
#include <clocksource/arm_arch_timer.h>
#include <linux/errno.h>
#include <linux/math64.h>
#include <linux/string.h>
#endif

#include "sfc.h"

#define SFC_MAX_IOSIZE_VER3		(1024 * 8)
#define SFC_MAX_IOSIZE_VER4		(0xFFFFFFFF)

static void __iomem *g_sfc_reg;
static u32 sfc_version;

#ifdef CONFIG_RK_SFC_NOR_MTD_PANIC_WRITE
static u32 sfc_panic_timer_rate;
#endif

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

u16 sfc_get_version(void)
{
	return  (u32)(readl(g_sfc_reg + SFC_VER) & 0xffff);
}

u32 sfc_get_max_iosize(void)
{
	if (sfc_get_version() >= SFC_VER_4)
		return SFC_MAX_IOSIZE_VER4;
	else
		return SFC_MAX_IOSIZE_VER3;
}

u32 sfc_get_max_dll_cells(void)
{
	switch (sfc_get_version()) {
	case SFC_VER_8:
	case SFC_VER_6:
	case SFC_VER_5:
		return SCLK_SMP_SEL_MAX_V5;
	case SFC_VER_4:
		return SCLK_SMP_SEL_MAX_V4;
	default:
		return 0;
	}
}

void sfc_set_delay_lines(u16 cells)
{
	u16 cell_max = (u16)sfc_get_max_dll_cells();
	u32 val = 0;

	if (cells > cell_max)
		cells = cell_max;

	if (cells)
		val = SCLK_SMP_SEL_EN | cells;

	writel(val, g_sfc_reg + SFC_DLL_CTRL0);
}

int sfc_init(void __iomem *reg_addr)
{
	g_sfc_reg = reg_addr;
	writel(0, g_sfc_reg + SFC_CTRL);

	if (sfc_get_version() >= SFC_VER_4)
		writel(1, g_sfc_reg + SFC_LEN_CTRL);
	sfc_version = sfc_get_version();

#ifdef CONFIG_RK_SFC_NOR_MTD_PANIC_WRITE
	sfc_panic_timer_rate = arch_timer_get_rate();
#endif

	return SFC_OK;
}

void sfc_clean_irq(void)
{
	writel(0xFFFFFFFF, g_sfc_reg + SFC_ICLR);
	writel(0xFFFFFFFF, g_sfc_reg + SFC_IMR);
}

int sfc_request(struct rk_sfc_op *op, u32 addr, void *data, u32 size)
{
	int ret = SFC_OK;
	union SFCCMD_DATA cmd;
	int reg;
	int timeout = 0;
	u32 *p_data = (u32 *)data;
	u32 temp = 0;

	reg = readl(g_sfc_reg + SFC_FSR);

	if (!(reg & SFC_TXEMPTY) || !(reg & SFC_RXEMPTY) ||
	    (readl(g_sfc_reg + SFC_SR) & SFC_BUSY))
		sfc_reset();

	cmd.d32 = op->sfcmd.d32;

	if (size && size < 4 && cmd.b.rw == SFC_WRITE) {
		if (size == 1)
			temp = *((u8 *)data);
		else if (size == 2)
			temp = *((u16 *)data);
		else
			temp = ((u8 *)data)[0] | ((u8 *)data)[1] << 8 | ((u8 *)data)[2] << 16;
		p_data = &temp;
	} else if (size >= 4 && ((uintptr_t)data & 0x3)) {
		pr_err("%s data addr unaligned access\n", __func__);
	} else if (size & 0x3 && cmd.b.rw == SFC_WRITE) {
		pr_err("%s data size unaligned access\n", __func__);
	}

	if (cmd.b.addrbits == SFC_ADDR_XBITS) {
		union SFCCTRL_DATA ctrl;

		ctrl.d32 = op->sfctrl.d32;

		if (!ctrl.b.addrbits)
			return SFC_PARAM_ERR;

		/* Controller plus 1 automatically */
		writel(ctrl.b.addrbits - 1, g_sfc_reg + SFC_ABIT);
	}

	/* shift in the data at negedge sclk_out */
	op->sfctrl.d32 |= 0x2;
	cmd.b.datasize = size;

	if (sfc_version >= SFC_VER_4)
		writel(size, g_sfc_reg + SFC_LEN_EXT);
	else
		cmd.b.datasize = size;

	writel(op->sfctrl.d32, g_sfc_reg + SFC_CTRL);
	writel(cmd.d32, g_sfc_reg + SFC_CMD);

	if (cmd.b.addrbits)
		writel(addr, g_sfc_reg + SFC_ADDR);

	if (!size)
		goto exit_wait;
	if (op->sfctrl.b.enbledma) {
		unsigned long dma_addr;
		u8 direction = (cmd.b.rw == SFC_WRITE) ? 1 : 0;

		dma_addr = rksfc_dma_map_single((unsigned long)data,
						size,
						direction);
		rksfc_irq_flag_init();
		writel(0xFFFFFFFF, g_sfc_reg + SFC_ICLR);
		writel(~((u32)DMA_INT), g_sfc_reg + SFC_IMR);
		writel((u32)dma_addr, g_sfc_reg + SFC_DMA_ADDR);
		writel(SFC_DMA_START, g_sfc_reg + SFC_DMA_TRIGGER);

		rksfc_wait_for_irq_completed();
		timeout = size * 10;
		while ((readl(g_sfc_reg + SFC_SR) & SFC_BUSY) &&
		       (timeout-- > 0))
			sfc_delay(1);
		if (timeout <= 0)
			ret = SFC_WAIT_TIMEOUT;
		direction = (cmd.b.rw == SFC_WRITE) ? 1 : 0;
		rksfc_dma_unmap_single(dma_addr,
				       size,
				       direction);
	} else {
		u32 i, words, count, bytes;
		union SFCFSR_DATA    fifostat;

		if (cmd.b.rw == SFC_WRITE) {
			words  = (size + 3) >> 2;

			while (words) {
				fifostat.d32 = readl(g_sfc_reg + SFC_FSR);

				if (fifostat.b.txlevel > 0) {
					count = words < fifostat.b.txlevel ?
						words : fifostat.b.txlevel;

					for (i = 0; i < count; i++) {
						writel(*p_data++,
						       g_sfc_reg + SFC_DATA);
						words--;
					}

					if (words == 0)
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
			bytes = size & 0x3;
			words = size >> 2;

			while (words) {
				fifostat.d32 = readl(g_sfc_reg + SFC_FSR);

				if (fifostat.b.rxlevel > 0) {
					u32 count;

					count = words < fifostat.b.rxlevel ?
						words : fifostat.b.rxlevel;

					for (i = 0; i < count; i++) {
						*p_data++ = readl(g_sfc_reg +
								  SFC_DATA);
						words--;
					}

					if (words == 0)
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

				if (!bytes)
					break;
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

	while (readl(g_sfc_reg + SFC_SR) & SFC_BUSY) {
		sfc_delay(1);

		if (timeout++ > 100000) {         /* wait 100ms */
			ret = SFC_TX_TIMEOUT;
			break;
		}
	}

	sfc_delay(1); /* CS# High Time (read/write) >100ns */
	return ret;
}

#ifdef CONFIG_RK_SFC_NOR_MTD_PANIC_WRITE
bool sfc_panic_expired(const struct sfc_panic_ctx *ctx)
{
	return arch_timer_read_counter() - ctx->start >= ctx->ticks;
}

static bool sfc_panic_idle(void)
{
	u32 fsr = readl(g_sfc_reg + SFC_FSR);

	return !(readl(g_sfc_reg + SFC_SR) & SFC_BUSY) &&
	       (fsr & SFC_TXEMPTY) && (fsr & SFC_RXEMPTY);
}

int sfc_panic_begin(struct sfc_panic_ctx *ctx, unsigned int timeout_ms)
{
	if (!g_sfc_reg || !sfc_panic_timer_rate || !timeout_ms)
		return -ENODEV;
	if (sfc_version != SFC_VER_3 && sfc_version != SFC_VER_4 &&
	    sfc_version != SFC_VER_5)
		return -EOPNOTSUPP;

	ctx->start = arch_timer_read_counter();
	ctx->ticks = div_u64((u64)sfc_panic_timer_rate * timeout_ms, 1000);
	if (!ctx->ticks)
		return -EINVAL;
	if (!sfc_panic_idle())
		return -EBUSY;

	/* This terminal panic path must not wait for an IRQ or reset the SFC. */
	writel(0xffffffff, g_sfc_reg + SFC_IMR);
	writel(0xffffffff, g_sfc_reg + SFC_ICLR);
	return 0;
}

int sfc_request_panic(const struct rk_sfc_op *op, u32 addr, void *data,
		      u32 size, const struct sfc_panic_ctx *ctx)
{
	union SFCCMD_DATA cmd = op->sfcmd;
	union SFCFSR_DATA fsr;
	u32 ctrl = op->sfctrl.d32;
	u32 done = 0, n, word;

	if (op->sfctrl.b.enbledma || cmd.b.addrbits == SFC_ADDR_XBITS ||
	    size > 256 || (size && !data))
		return -EINVAL;
	if (sfc_panic_expired(ctx))
		return -ETIMEDOUT;
	if (!sfc_panic_idle())
		return -EBUSY;

	writel(0xffffffff, g_sfc_reg + SFC_ICLR);
	ctrl &= ~SFC_ENABLE_DMA;
	ctrl |= 0x2; /* Keep the sampling phase used by sfc_request(). */
	cmd.b.datasize = size;
	if (sfc_version >= SFC_VER_4)
		writel(size, g_sfc_reg + SFC_LEN_EXT);
	writel(ctrl, g_sfc_reg + SFC_CTRL);
	writel(cmd.d32, g_sfc_reg + SFC_CMD);
	if (cmd.b.addrbits)
		writel(addr, g_sfc_reg + SFC_ADDR);

	while (done < size) {
		if (sfc_panic_expired(ctx))
			return -ETIMEDOUT;
		if (readl(g_sfc_reg + SFC_RAWISR) &
		    (NSPIERR_INT | AHBERR_INT | TXOF_INT | RXUF_INT))
			return -EIO;

		fsr.d32 = readl(g_sfc_reg + SFC_FSR);
		n = min_t(u32, sizeof(word), size - done);
		if (cmd.b.rw == SFC_WRITE && fsr.b.txlevel) {
			word = 0xffffffff;
			memcpy(&word, (u8 *)data + done, n);
			writel(word, g_sfc_reg + SFC_DATA);
			done += n;
		} else if (cmd.b.rw == SFC_READ && fsr.b.rxlevel) {
			word = readl(g_sfc_reg + SFC_DATA);
			memcpy((u8 *)data + done, &word, n);
			done += n;
		} else {
			udelay(1);
		}
	}

	while (readl(g_sfc_reg + SFC_SR) & SFC_BUSY) {
		if (sfc_panic_expired(ctx))
			return -ETIMEDOUT;
		udelay(1);
	}
	if (readl(g_sfc_reg + SFC_RAWISR) &
	    (NSPIERR_INT | AHBERR_INT | TXOF_INT | RXUF_INT))
		return -EIO;

	udelay(1); /* Match the normal path's minimum CS high time. */
	return sfc_panic_expired(ctx) ? -ETIMEDOUT : 0;
}
#endif
