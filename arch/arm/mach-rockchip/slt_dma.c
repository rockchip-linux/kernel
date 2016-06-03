/*
 * slt_test/soc_core/slt_dma.c
 *
 * SLT DMA Test for Rockchip
 *
 * Copyright (C) 2016 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl330.h>
#include <linux/scatterlist.h>

#include <linux/err.h>
#include <asm/unaligned.h>

/* Register and Bit field Definitions */
#define DS			0x0
#define DS_ST_STOP		0x0
#define DS_ST_EXEC		0x1
#define DS_ST_CMISS		0x2
#define DS_ST_UPDTPC		0x3
#define DS_ST_WFE		0x4
#define DS_ST_ATBRR		0x5
#define DS_ST_QBUSY		0x6
#define DS_ST_WFP		0x7
#define DS_ST_KILL		0x8
#define DS_ST_CMPLT		0x9
#define DS_ST_FLTCMP		0xe
#define DS_ST_FAULT		0xf

#define DPC			0x4
#define INTEN			0x20
#define ES			0x24
#define INTSTATUS		0x28
#define INTCLR			0x2c
#define FSM			0x30
#define FSC			0x34
#define FTM			0x38

#define _FTC			0x40
#define FTC(n)			(_FTC + (n)*0x4)

#define _CS			0x100
#define CS(n)			(_CS + (n)*0x8)
#define CS_CNS			(1 << 21)

#define _CPC			0x104
#define CPC(n)			(_CPC + (n)*0x8)

#define _SA			0x400
#define SA(n)			(_SA + (n)*0x20)

#define _DA			0x404
#define DA(n)			(_DA + (n)*0x20)

#define _CC			0x408
#define CC(n)			(_CC + (n)*0x20)

#define CC_SRCINC		(1 << 0)
#define CC_DSTINC		(1 << 14)
#define CC_SRCPRI		(1 << 8)
#define CC_DSTPRI		(1 << 22)
#define CC_SRCNS		(1 << 9)
#define CC_DSTNS		(1 << 23)
#define CC_SRCIA		(1 << 10)
#define CC_DSTIA		(1 << 24)
#define CC_SRCBRSTLEN_SHFT	4
#define CC_DSTBRSTLEN_SHFT	18
#define CC_SRCBRSTSIZE_SHFT	1
#define CC_DSTBRSTSIZE_SHFT	15
#define CC_SRCCCTRL_SHFT	11
#define CC_SRCCCTRL_MASK	0x7
#define CC_DSTCCTRL_SHFT	25
#define CC_DRCCCTRL_MASK	0x7
#define CC_SWAP_SHFT		28

#define _LC0			0x40c
#define LC0(n)			(_LC0 + (n)*0x20)

#define _LC1			0x410
#define LC1(n)			(_LC1 + (n)*0x20)

#define DBGSTATUS		0xd00
#define DBG_BUSY		(1 << 0)

#define DBGCMD			0xd04
#define DBGINST0		0xd08
#define DBGINST1		0xd0c

#define CR0			0xe00
#define CR1			0xe04
#define CR2			0xe08
#define CR3			0xe0c
#define CR4			0xe10
#define CRD			0xe14

#define PERIPH_ID		0xfe0
#define PERIPH_REV_SHIFT	20
#define PERIPH_REV_MASK		0xf
#define PERIPH_REV_R0P0		0
#define PERIPH_REV_R1P0		1
#define PERIPH_REV_R1P1		2
#define PCELL_ID		0xff0

#define CR0_PERIPH_REQ_SET	(1 << 0)
#define CR0_BOOT_EN_SET		(1 << 1)
#define CR0_BOOT_MAN_NS		(1 << 2)
#define CR0_NUM_CHANS_SHIFT	4
#define CR0_NUM_CHANS_MASK	0x7
#define CR0_NUM_PERIPH_SHIFT	12
#define CR0_NUM_PERIPH_MASK	0x1f
#define CR0_NUM_EVENTS_SHIFT	17
#define CR0_NUM_EVENTS_MASK	0x1f

#define CR1_ICACHE_LEN_SHIFT	0
#define CR1_ICACHE_LEN_MASK	0x7
#define CR1_NUM_ICACHELINES_SHIFT	4
#define CR1_NUM_ICACHELINES_MASK	0xf

#define CRD_DATA_WIDTH_SHIFT	0
#define CRD_DATA_WIDTH_MASK	0x7
#define CRD_WR_CAP_SHIFT	4
#define CRD_WR_CAP_MASK		0x7
#define CRD_WR_Q_DEP_SHIFT	8
#define CRD_WR_Q_DEP_MASK	0xf
#define CRD_RD_CAP_SHIFT	12
#define CRD_RD_CAP_MASK		0x7
#define CRD_RD_Q_DEP_SHIFT	16
#define CRD_RD_Q_DEP_MASK	0xf
#define CRD_DATA_BUFF_SHIFT	20
#define CRD_DATA_BUFF_MASK	0x3ff

#define PART			0x330
#define DESIGNER		0x41
#define REVISION		0x0
#define INTEG_CFG		0x0
#define PERIPH_ID_VAL		((PART << 0) | (DESIGNER << 12))

#define PCELL_ID_VAL		0xb105f00d

#define PL330_STATE_STOPPED		(1 << 0)
#define PL330_STATE_EXECUTING		(1 << 1)
#define PL330_STATE_WFE			(1 << 2)
#define PL330_STATE_FAULTING		(1 << 3)
#define PL330_STATE_COMPLETING		(1 << 4)
#define PL330_STATE_WFP			(1 << 5)
#define PL330_STATE_KILLING		(1 << 6)
#define PL330_STATE_FAULT_COMPLETING	(1 << 7)
#define PL330_STATE_CACHEMISS		(1 << 8)
#define PL330_STATE_UPDTPC		(1 << 9)
#define PL330_STATE_ATBARRIER		(1 << 10)
#define PL330_STATE_QUEUEBUSY		(1 << 11)
#define PL330_STATE_INVALID		(1 << 15)

#define PL330_STABLE_STATES (PL330_STATE_STOPPED | PL330_STATE_EXECUTING \
				| PL330_STATE_WFE | PL330_STATE_FAULTING)

#define CMD_DMAADDH		0x54
#define CMD_DMAEND		0x00
#define CMD_DMAFLUSHP		0x35
#define CMD_DMAGO		0xa0
#define CMD_DMALD		0x04
#define CMD_DMALDP		0x25
#define CMD_DMALP		0x20
#define CMD_DMALPEND		0x28
#define CMD_DMAKILL		0x01
#define CMD_DMAMOV		0xbc
#define CMD_DMANOP		0x18
#define CMD_DMARMB		0x12
#define CMD_DMASEV		0x34
#define CMD_DMAST		0x08
#define CMD_DMASTP		0x29
#define CMD_DMASTZ		0x0c
#define CMD_DMAWFE		0x36
#define CMD_DMAWFP		0x30
#define CMD_DMAWMB		0x13

#define SZ_DMAADDH		3
#define SZ_DMAEND		1
#define SZ_DMAFLUSHP		2
#define SZ_DMALD		1
#define SZ_DMALDP		2
#define SZ_DMALP		2
#define SZ_DMALPEND		2
#define SZ_DMAKILL		1
#define SZ_DMAMOV		6
#define SZ_DMANOP		1
#define SZ_DMARMB		1
#define SZ_DMASEV		2
#define SZ_DMAST		1
#define SZ_DMASTP		2
#define SZ_DMASTZ		1
#define SZ_DMAWFE		2
#define SZ_DMAWFP		2
#define SZ_DMAWMB		1
#define SZ_DMAGO		6

#define BRST_LEN(ccr)		((((ccr) >> CC_SRCBRSTLEN_SHFT) & 0xf) + 1)
#define BRST_SIZE(ccr)		(1 << (((ccr) >> CC_SRCBRSTSIZE_SHFT) & 0x7))

#define BYTE_TO_BURST(b, ccr)	((b) / BRST_SIZE(ccr) / BRST_LEN(ccr))
#define BURST_TO_BYTE(c, ccr)	((c) * BRST_SIZE(ccr) * BRST_LEN(ccr))

/*
 * With 256 bytes, we can do more than 2.5MB and 5MB xfers per req
 * at 1byte/burst for P<->M and M<->M respectively.
 * For typical scenario, at 1word/burst, 10MB and 20MB xfers per req
 * should be enough for P<->M and M<->M respectively.
 */
#define MCODE_BUFF_PER_REQ	256

/* If the _pl330_req is available to the client */
#define IS_FREE(req)	(*((u8 *)((req)->mc_cpu)) == CMD_DMAEND)

/* Use this _only_ to wait on transient states */
#define UNTIL(t, s)	while (!(_state(t) & (s))) cpu_relax();

#ifndef PL330_DEBUG_MCGEN
static unsigned cmd_line;
#define PL330_DBGCMD_DUMP(off, x...)	do { \
						printk("%x:", cmd_line); \
						printk(x); \
						cmd_line += off; \
					} while (0)
#define PL330_DBGMC_START(addr)		(cmd_line = addr)
#else
#define PL330_DBGCMD_DUMP(off, x...)	do {} while (0)
#define PL330_DBGMC_START(addr)		do {} while (0)
#endif

#define msecs_to_loops(t) (loops_per_jiffy / 1000 * HZ * t)

#define DMA_BUFFER_SIZE (6*256)
#define DMA_CHANNEL 7

struct slt_dma {
	size_t size;
	char *src_buffer;
	char *dst_buffer;
	dma_addr_t src_phy_addr;
	dma_addr_t dst_phy_addr;
	char *vcode;
	dma_addr_t pcode;
	void __iomem *base;
	struct kobject *kobj;
	struct kobj_attribute pl330_attr;
};

enum pl330_dst {
	SRC = 0,
	DST,
};

enum pl330_cond {
	SINGLE,
	BURST,
	ALWAYS,
};

enum dmamov_dst {
	SAR = 0,
	CCR,
	DAR,
};

static inline u32 _emit_ADDH(unsigned dry_run, u8 buf[],
		enum pl330_dst da, u16 val)
{
	if (dry_run)
		return SZ_DMAADDH;

	buf[0] = CMD_DMAADDH;
	buf[0] |= (da << 1);
	put_unaligned(val, (u16 *)&buf[1]);

	PL330_DBGCMD_DUMP(SZ_DMAADDH, "\tDMAADDH %s %u\n",
		da == 1 ? "DA" : "SA", val);

	return SZ_DMAADDH;
}

static inline u32 _emit_END(unsigned dry_run, u8 buf[])
{
	if (dry_run)
		return SZ_DMAEND;

	buf[0] = CMD_DMAEND;

	PL330_DBGCMD_DUMP(SZ_DMAEND, "\tDMAEND\n");

	return SZ_DMAEND;
}

static inline u32 _emit_FLUSHP(unsigned dry_run, u8 buf[], u8 peri)
{
	if (dry_run)
		return SZ_DMAFLUSHP;

	buf[0] = CMD_DMAFLUSHP;

	peri &= 0x1f;
	peri <<= 3;
	buf[1] = peri;

	PL330_DBGCMD_DUMP(SZ_DMAFLUSHP, "\tDMAFLUSHP %u\n", peri >> 3);

	return SZ_DMAFLUSHP;
}

static inline u32 _emit_LD(unsigned dry_run, u8 buf[],	enum pl330_cond cond)
{
	if (dry_run)
		return SZ_DMALD;

	buf[0] = CMD_DMALD;

	if (cond == SINGLE)
		buf[0] |= (0 << 1) | (1 << 0);
	else if (cond == BURST)
		buf[0] |= (1 << 1) | (1 << 0);

	PL330_DBGCMD_DUMP(SZ_DMALD, "\tDMALD%c\n",
		cond == SINGLE ? 'S' : (cond == BURST ? 'B' : 'A'));

	return SZ_DMALD;
}

static inline u32 _emit_LDP(unsigned dry_run, u8 buf[],
		enum pl330_cond cond, u8 peri)
{
	if (dry_run)
		return SZ_DMALDP;

	buf[0] = CMD_DMALDP;

	if (cond == BURST)
		buf[0] |= (1 << 1);

	peri &= 0x1f;
	peri <<= 3;
	buf[1] = peri;

	PL330_DBGCMD_DUMP(SZ_DMALDP, "\tDMALDP%c %u\n",
		cond == SINGLE ? 'S' : 'B', peri >> 3);

	return SZ_DMALDP;
}

static inline u32 _emit_LP(unsigned dry_run, u8 buf[],
		unsigned loop, u8 cnt)
{
	if (dry_run)
		return SZ_DMALP;

	buf[0] = CMD_DMALP;

	if (loop)
		buf[0] |= (1 << 1);

	cnt--; /* DMAC increments by 1 internally */
	buf[1] = cnt;

	PL330_DBGCMD_DUMP(SZ_DMALP, "\tDMALP_%c %u\n", loop ? '1' : '0', cnt);

	return SZ_DMALP;
}

struct _arg_LPEND {
	enum pl330_cond cond;
	bool forever;
	unsigned loop;
	u8 bjump;
};

static inline u32 _emit_LPEND(unsigned dry_run, u8 buf[],
		const struct _arg_LPEND *arg)
{
	enum pl330_cond cond = arg->cond;
	bool forever = arg->forever;
	unsigned loop = arg->loop;
	u8 bjump = arg->bjump;

	if (dry_run)
		return SZ_DMALPEND;

	buf[0] = CMD_DMALPEND;

	if (loop)
		buf[0] |= (1 << 2);

	if (!forever)
		buf[0] |= (1 << 4);

	if (cond == SINGLE)
		buf[0] |= (0 << 1) | (1 << 0);
	else if (cond == BURST)
		buf[0] |= (1 << 1) | (1 << 0);

	buf[1] = bjump;

	PL330_DBGCMD_DUMP(SZ_DMALPEND, "\tDMALP%s%c_%c bjmpto_%x\n",
			forever ? "FE" : "END",
			cond == SINGLE ? 'S' : (cond == BURST ? 'B' : 'A'),
			loop ? '1' : '0',
			bjump);

	return SZ_DMALPEND;
}

static inline u32 _emit_KILL(unsigned dry_run, u8 buf[])
{
	if (dry_run)
		return SZ_DMAKILL;

	buf[0] = CMD_DMAKILL;

	return SZ_DMAKILL;
}

static inline u32 _emit_MOV(unsigned dry_run, u8 buf[],
		enum dmamov_dst dst, u32 val)
{
	if (dry_run)
		return SZ_DMAMOV;

	buf[0] = CMD_DMAMOV;
	buf[1] = dst;
	put_unaligned(val, (u32 *)&buf[2]);

	PL330_DBGCMD_DUMP(SZ_DMAMOV, "\tDMAMOV %s 0x%x\n",
		dst == SAR ? "SAR" : (dst == DAR ? "DAR" : "CCR"), val);

	return SZ_DMAMOV;
}

static inline u32 _emit_NOP(unsigned dry_run, u8 buf[])
{
	if (dry_run)
		return SZ_DMANOP;

	buf[0] = CMD_DMANOP;

	PL330_DBGCMD_DUMP(SZ_DMANOP, "\tDMANOP\n");

	return SZ_DMANOP;
}

static inline u32 _emit_RMB(unsigned dry_run, u8 buf[])
{
	if (dry_run)
		return SZ_DMARMB;

	buf[0] = CMD_DMARMB;

	PL330_DBGCMD_DUMP(SZ_DMARMB, "\tDMARMB\n");

	return SZ_DMARMB;
}

static inline u32 _emit_SEV(unsigned dry_run, u8 buf[], u8 ev)
{
	if (dry_run)
		return SZ_DMASEV;

	buf[0] = CMD_DMASEV;

	ev &= 0x1f;
	ev <<= 3;
	buf[1] = ev;

	PL330_DBGCMD_DUMP(SZ_DMASEV, "\tDMASEV %u\n", ev >> 3);

	return SZ_DMASEV;
}

static inline u32 _emit_ST(unsigned dry_run, u8 buf[], enum pl330_cond cond)
{
	if (dry_run)
		return SZ_DMAST;

	buf[0] = CMD_DMAST;

	if (cond == SINGLE)
		buf[0] |= (0 << 1) | (1 << 0);
	else if (cond == BURST)
		buf[0] |= (1 << 1) | (1 << 0);

	PL330_DBGCMD_DUMP(SZ_DMAST, "\tDMAST%c\n",
		cond == SINGLE ? 'S' : (cond == BURST ? 'B' : 'A'));

	return SZ_DMAST;
}

static inline u32 _emit_STP(unsigned dry_run, u8 buf[],
		enum pl330_cond cond, u8 peri)
{
	if (dry_run)
		return SZ_DMASTP;

	buf[0] = CMD_DMASTP;

	if (cond == BURST)
		buf[0] |= (1 << 1);

	peri &= 0x1f;
	peri <<= 3;
	buf[1] = peri;

	PL330_DBGCMD_DUMP(SZ_DMASTP, "\tDMASTP%c %u\n",
		cond == SINGLE ? 'S' : 'B', peri >> 3);

	return SZ_DMASTP;
}

static inline u32 _emit_STZ(unsigned dry_run, u8 buf[])
{
	if (dry_run)
		return SZ_DMASTZ;

	buf[0] = CMD_DMASTZ;

	PL330_DBGCMD_DUMP(SZ_DMASTZ, "\tDMASTZ\n");

	return SZ_DMASTZ;
}

static inline u32 _emit_WFE(unsigned dry_run, u8 buf[], u8 ev,
		unsigned invalidate)
{
	if (dry_run)
		return SZ_DMAWFE;

	buf[0] = CMD_DMAWFE;

	ev &= 0x1f;
	ev <<= 3;
	buf[1] = ev;

	if (invalidate)
		buf[1] |= (1 << 1);

	PL330_DBGCMD_DUMP(SZ_DMAWFE, "\tDMAWFE %u%s\n",
		ev >> 3, invalidate ? ", I" : "");

	return SZ_DMAWFE;
}

static inline u32 _emit_WFP(unsigned dry_run, u8 buf[],
		enum pl330_cond cond, u8 peri)
{
	if (dry_run)
		return SZ_DMAWFP;

	buf[0] = CMD_DMAWFP;

	if (cond == SINGLE)
		buf[0] |= (0 << 1) | (0 << 0);
	else if (cond == BURST)
		buf[0] |= (1 << 1) | (0 << 0);
	else
		buf[0] |= (0 << 1) | (1 << 0);

	peri &= 0x1f;
	peri <<= 3;
	buf[1] = peri;

	PL330_DBGCMD_DUMP(SZ_DMAWFP, "\tDMAWFP%c %u\n",
		cond == SINGLE ? 'S' : (cond == BURST ? 'B' : 'P'), peri >> 3);

	return SZ_DMAWFP;
}

static inline u32 _emit_WMB(unsigned dry_run, u8 buf[])
{
	if (dry_run)
		return SZ_DMAWMB;

	buf[0] = CMD_DMAWMB;

	PL330_DBGCMD_DUMP(SZ_DMAWMB, "\tDMAWMB\n");

	return SZ_DMAWMB;
}

struct _arg_GO {
	u8 chan;
	u32 addr;
	unsigned ns;
};

static inline u32 _emit_GO(unsigned dry_run, u8 buf[],
		const struct _arg_GO *arg)
{
	u8 chan = arg->chan;
	u32 addr = arg->addr;
	unsigned ns = arg->ns;

	if (dry_run)
		return SZ_DMAGO;

	buf[0] = CMD_DMAGO;
	buf[0] |= (ns << 1);

	buf[1] = chan & 0x7;

	put_unaligned(addr, (u32 *)&buf[2]);

	return SZ_DMAGO;
}

/* Returns Time-Out */
static bool until_dmac_idle(struct slt_dma *dma)
{
	unsigned long loops = msecs_to_loops(5);

	do {
		/* Until Manager is Idle */
		if (!(readl(dma->base + DBGSTATUS) & DBG_BUSY))
			break;

		cpu_relax();
	} while (--loops);

	if (!loops)
		return true;

	return false;
}

static int dma_test_start(struct slt_dma *dma)
{
	int cyc, off, t;
	/*unsigned lcnt0, ljmp0;*/
	unsigned lcnt1, ljmp1, ljmpfe;
	unsigned int ccr = 0xbdc2f7, size = DMA_BUFFER_SIZE;
	unsigned dry_run = 0, chan = DMA_CHANNEL;
	/*unsigned ev = 2;*/
	struct _arg_LPEND lpend;
	u8 *buf = dma->vcode;
	u8 insn[32];
	dma_addr_t src_addr, dst_addr;

	u32 val;

	off = 0;
	ljmpfe = off;

	memset(dma->src_buffer, 0x55, size);
	PL330_DBGMC_START((unsigned)buf);

	t = 8;
	src_addr = dma->src_phy_addr;
	dst_addr = dma->dst_phy_addr;

	lcnt1 = size / 128;
	cyc = 1;

	/* DMAMOV CCR, ccr */
	off += _emit_MOV(dry_run, &buf[off], CCR, ccr);
	/* forever loop */
	off += _emit_MOV(dry_run, &buf[off], SAR, src_addr);
	off += _emit_MOV(dry_run, &buf[off], DAR, dst_addr);

	/* loop1 */
	off += _emit_LP(dry_run, &buf[off], 1, lcnt1);
	ljmp1 = off;
	while (cyc--) {
		off += _emit_LD(dry_run, &buf[off], ALWAYS);
		off += _emit_ST(dry_run, &buf[off], ALWAYS);
	}
	lpend.cond = ALWAYS;
	lpend.forever = false;
	lpend.loop = 1;
	lpend.bjump = off - ljmp1;
	off += _emit_LPEND(dry_run, &buf[off], &lpend);

#if 0
	/*lcnt0 = 256;*/
	cyc = 128;   /* cyc for nop */
	lcnt1 = 1;
#if 0
	/* loop0 */
	off += _emit_LP(dry_run, &buf[off], 0,  lcnt0);
	ljmp0 = off;
#endif
	/* loop1 */
	off += _emit_LP(dry_run, &buf[off], 1, lcnt1);
	ljmp1 = off;

	while (cyc--) {
		off += _emit_NOP(dry_run, &buf[off]);
	}

	lpend.cond = ALWAYS;
	lpend.forever = false;
	lpend.loop = 1;
	lpend.bjump = off - ljmp1;
	off += _emit_LPEND(dry_run, &buf[off], &lpend);
#if 0
	lpend.cond = ALWAYS;
	lpend.forever = false;
	lpend.loop = 0;
	lpend.bjump = off - ljmp0;
	off += _emit_LPEND(dry_run, &buf[off], &lpend);
#endif
#endif
	/* mark forever loop */
	lpend.cond = ALWAYS;
	lpend.forever = true;
	lpend.loop = 1;
	lpend.bjump = off - ljmpfe;
	off +=  _emit_LPEND(dry_run, &buf[off], &lpend);

	insn[0] = CMD_DMAGO;
	insn[0] |= (1 << 1);
	insn[1] = chan & 0x7;
	put_unaligned(dma->pcode, (u32 *)&insn[2]);
	val = (insn[0] << 16) | (insn[1] << 24);
	writel(val, dma->base + DBGINST0);
	val = *((u32 *)&insn[2]);
	writel(val, dma->base + DBGINST1);

	/* If timed out due to halted state-machine */
	if (until_dmac_idle(dma)) {
		pr_err("DMAC halted!\n");
		return -1;
	}

	/* Get going */
	writel(0, dma->base + DBGCMD);
	return off;
}

static void dma_test_stop(struct slt_dma *dma)
{
	u8 insn[8];
	unsigned chan = DMA_CHANNEL, val = 0;

	memset(insn, 0, 8);
	_emit_KILL(0, insn);

	val = (insn[0] << 16) | (insn[1] << 24);
	val |= (1 << 0);
	val |= (chan << 8); /* Channel Number */

	writel(val, dma->base + DBGINST0);

	val = *((u32 *)&insn[2]);
	writel(val, dma->base + DBGINST1);

	/* If timed out due to halted state-machine */
	if (until_dmac_idle(dma)) {
		pr_err("DMAC halted!\n");
		return;
	}

	/* Get going */
	writel(0, dma->base + DBGCMD);
}

static ssize_t pl330_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	return 0;
}

static ssize_t pl330_store(struct kobject *kobj, struct kobj_attribute *attr,
			    const char *buf, size_t n)
{
	int on;
	struct slt_dma *dma = container_of(attr, struct slt_dma, pl330_attr);

	sscanf(buf, "%d", &on);

	if (on == 1)
		dma_test_start(dma);
	else
		dma_test_stop(dma);

	return n;
}

static void init_pl330_node(struct slt_dma *dma)
{
	int ret = 0;

	dma->kobj = kobject_create_and_add("pl330", NULL);
	dma->pl330_attr.attr.name = "memcpy";
	dma->pl330_attr.attr.mode = S_IRUGO | S_IWUSR | S_IWUGO;
	dma->pl330_attr.show = pl330_show;
	dma->pl330_attr.store = pl330_store;
	ret = sysfs_create_file(dma->kobj, &dma->pl330_attr.attr);
}

static int __init slt_dma_test_init(void)
{
	int ret = -1;
	u32 buf[2];
	struct device_node *np = NULL;
	struct slt_dma *dma0 = NULL;

	np = of_find_node_by_name(np, "pdma");

	if (!np)
		return ret;

	ret = of_property_read_u32_array(np, "reg", buf, 2);

	if (ret) {
		pr_err("%s get reg fail\n", __func__);
		return ret;
	}

	dma0 = kzalloc(sizeof(*dma0), GFP_KERNEL);
	if (!dma0) {
		pr_err("%s alloc slt_dma0 fail\n", __func__);
		return -ENOMEM;
	}

	dma0->base = ioremap(buf[0], buf[1]);

	if (!dma0->base) {
		pr_err("%s ioremap slt_dma0 fail\n", __func__);
		goto err2;
	}

	pr_info("dma phy:%08x, size:%08x map:%p\n", buf[0], buf[1], dma0->base);
	dma0->size = DMA_BUFFER_SIZE;
	dma0->src_buffer = dma_alloc_coherent(NULL, dma0->size * 2 + 1024,
					&dma0->src_phy_addr, DMA_MEMORY_MAP);

	if (!dma0->src_buffer)
		goto err1;

	dma0->dst_buffer = dma0->src_buffer + dma0->size;
	dma0->dst_phy_addr = dma0->src_phy_addr + dma0->size;
	dma0->vcode = dma0->dst_buffer + dma0->size;
	dma0->pcode = dma0->dst_phy_addr + dma0->size;

	init_pl330_node(dma0);
	pr_info("dma alloc v:%p, p:%08x\n", dma0->src_buffer,
		dma0->src_phy_addr);
	return 0;

err1:
	iounmap(dma0->base);
err2:
	kfree(dma0);

	return ret;
}
device_initcall(slt_dma_test_init);

static void __exit slt_dma_test_exit(void)
{
}
module_exit(slt_dma_test_exit);

MODULE_AUTHOR("Huibin Hong <huibin.hong@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip slt dma driver");
MODULE_LICENSE("GPL v2");
