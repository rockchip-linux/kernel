/*
 * Copyright (C) 2011-2014 Imagination Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This file implements running AXD as a single VPE along side linux on the same
 * core.
 */
#include <linux/device.h>
#include <linux/io.h>
#include <linux/irqchip/mips-gic.h>

#include <asm/cpu-features.h>
#include <asm/hazards.h>
#include <asm/mipsregs.h>
#include <asm/mipsmtregs.h>
#include <asm/tlbmisc.h>

#include "axd_module.h"
#include "axd_platform.h"

static unsigned int axd_irq;
static unsigned int axd_vpe;

void axd_platform_init(struct axd_dev *axd)
{
	unsigned int val;
	unsigned long irqflags;
	unsigned long mtflags;

	axd_irq = axd->axd_irq;
	axd_vpe = axd->vpe;

	/*
	 * make sure nothing else on this vpe or another vpe can try to modify
	 * any of the shared registers below
	 */
	local_irq_save(irqflags);
	mtflags = dvpe();

	/* EVP = 0, VPC = 1 */
	val = read_c0_mvpcontrol();
	val &= ~MVPCONTROL_EVP;
	val |= MVPCONTROL_VPC;
	write_c0_mvpcontrol(val);
	instruction_hazard();

	/* prepare TC for setting up */
	settc(axd_vpe);
	write_tc_c0_tchalt(1);

	/* make sure no interrupts are pending and exceptions bits are clear */
	write_vpe_c0_cause(0);
	write_vpe_c0_status(0);

	/* bind TC to VPE */
	val = read_tc_c0_tcbind();
	val |= (axd_vpe << TCBIND_CURTC_SHIFT) | (axd_vpe << TCBIND_CURVPE_SHIFT);
	write_tc_c0_tcbind(val);

	/* VPA = 1, MVP = 1 */
	val = read_vpe_c0_vpeconf0();
	val |= VPECONF0_MVP;
	val |= VPECONF0_VPA;
	write_vpe_c0_vpeconf0(val);

	/* A = 1, IXMT = 0 */
	val = read_tc_c0_tcstatus();
	val &= ~TCSTATUS_IXMT;
	val |= TCSTATUS_A;
	write_tc_c0_tcstatus(val);

	/* TE = 1 */
	val = read_vpe_c0_vpecontrol();
	val |= VPECONTROL_TE;
	write_vpe_c0_vpecontrol(val);

	/* EVP = 1, VPC = 0 */
	val = read_c0_mvpcontrol();
	val |= MVPCONTROL_EVP;
	val &= ~MVPCONTROL_VPC;
	write_c0_mvpcontrol(val);
	instruction_hazard();

	evpe(mtflags);
	local_irq_restore(irqflags);
}

static void reset(unsigned int thread)
{
	unsigned int val;
	unsigned long irqflags;
	unsigned long mtflags;

	local_irq_save(irqflags);
	mtflags = dvpe();

	settc(axd_vpe);
	/* first stop TC1 */
	write_tc_c0_tchalt(1);

	/* clear EXL and ERL from TCSTATUS */
	val = read_c0_tcstatus();
	val &= ~(ST0_EXL | ST0_ERL);
	write_c0_tcstatus(val);

	evpe(mtflags);
	local_irq_restore(irqflags);
}

void axd_platform_set_pc(unsigned long pc)
{
	unsigned long irqflags;
	unsigned long mtflags;

	local_irq_save(irqflags);
	mtflags = dvpe();

	settc(axd_vpe);
	write_tc_c0_tcrestart(pc);

	evpe(mtflags);
	local_irq_restore(irqflags);
}

static int thread_control(unsigned int thread, int start)
{
	unsigned long irqflags;
	unsigned long mtflags;

	local_irq_save(irqflags);
	mtflags = dvpe();

	settc(axd_vpe);
	/* start/stop the thread */
	write_tc_c0_tchalt(!start);

	evpe(mtflags);
	local_irq_restore(irqflags);

	return 1;
}

int axd_platform_start(void)
{
	int thread = 0;

	reset(thread);
	if (thread_control(thread, 1))
		return 0;
	return -1;
}

void axd_platform_stop(void)
{
	int thread = 0;

	thread_control(thread, 0);
}

unsigned int axd_platform_num_threads(void)
{
	return 1;
}

void axd_platform_kick(void)
{
	unsigned int val;
	unsigned long irqflags;
	unsigned long mtflags;

	/*
	 * ensure all writes to shared uncached memory are visible to AXD
	 * before sending interrupt
	 */
	wmb();

	if (axd_irq) {
		gic_send_ipi(axd_irq);
		return;
	}

	/* fallback to sending interrupt at SWT1 */

	local_irq_save(irqflags);
	mtflags = dvpe();

	settc(axd_vpe);
	val = read_vpe_c0_cause();
	val |= CAUSEF_IP1;
	write_vpe_c0_cause(val);

	evpe(mtflags);
	local_irq_restore(irqflags);
}

inline unsigned long axd_platform_lock(void)
{
	return dvpe();
}

inline void axd_platform_unlock(unsigned long flags)
{
	evpe(flags);
}

inline void axd_platform_irq_ack(void)
{
}

static void print_regs(unsigned int thread)
{
	unsigned long irqflags;
	unsigned long mtflags;

	local_irq_save(irqflags);
	mtflags = dvpe();

	settc(thread);
	pr_err("PC:\t\t0x%08lX\n", read_tc_c0_tcrestart());
	pr_err("STATUS:\t\t0x%08lX\n", read_vpe_c0_status());
	pr_err("CAUSE:\t\t0x%08lX\n", read_vpe_c0_cause());
	pr_err("EPC:\t\t0x%08lX\n", read_vpe_c0_epc());
	pr_err("EBASE:\t\t0x%08lX\n", read_vpe_c0_ebase());
	pr_err("BADVADDR:\t0x%08lX\n", read_vpe_c0_badvaddr());
	pr_err("CONFIG:\t\t0x%08lX\n", read_vpe_c0_config());
	pr_err("MVPCONTROL:\t0x%08X\n", read_c0_mvpcontrol());
	pr_err("VPECONTROL:\t0x%08lX\n", read_vpe_c0_vpecontrol());
	pr_err("VPECONF0:\t0x%08lX\n", read_vpe_c0_vpeconf0());
	pr_err("TCBIND:\t\t0x%08lX\n", read_tc_c0_tcbind());
	pr_err("TCSTATUS:\t0x%08lX\n", read_tc_c0_tcstatus());
	pr_err("TCHALT:\t\t0x%08lX\n", read_tc_c0_tchalt());
	pr_err("\n");
	pr_err("$0: 0x%08lX\tat: 0x%08lX\tv0: 0x%08lX\tv1: 0x%08lX\n",
				mftgpr(0), mftgpr(1), mftgpr(2), mftgpr(3));
	pr_err("a0: 0x%08lX\ta1: 0x%08lX\ta2: 0x%08lX\ta3: 0x%08lX\n",
				mftgpr(4), mftgpr(5), mftgpr(6), mftgpr(7));
	pr_err("t0: 0x%08lX\tt1: 0x%08lX\tt2: 0x%08lX\tt3: 0x%08lX\n",
				mftgpr(8), mftgpr(9), mftgpr(10), mftgpr(11));
	pr_err("t4: 0x%08lX\tt5: 0x%08lX\tt6: 0x%08lX\tt7: 0x%08lX\n",
				mftgpr(12), mftgpr(13), mftgpr(14), mftgpr(15));
	pr_err("s0: 0x%08lX\ts1: 0x%08lX\ts2: 0x%08lX\ts3: 0x%08lX\n",
				mftgpr(16), mftgpr(17), mftgpr(18), mftgpr(19));
	pr_err("s4: 0x%08lX\ts5: 0x%08lX\ts6: 0x%08lX\ts7: 0x%08lX\n",
				mftgpr(20), mftgpr(21), mftgpr(22), mftgpr(23));
	pr_err("t8: 0x%08lX\tt9: 0x%08lX\tk0: 0x%08lX\tk1: 0x%08lX\n",
				mftgpr(24), mftgpr(25), mftgpr(26), mftgpr(27));
	pr_err("gp: 0x%08lX\tsp: 0x%08lX\ts8: 0x%08lX\tra: 0x%08lX\n",
				mftgpr(28), mftgpr(29), mftgpr(30), mftgpr(31));

	evpe(mtflags);
	local_irq_restore(irqflags);
}

void axd_platform_print_regs(void)
{
	int i;

	for (i = 1; i < 2; i++) {
		pr_err("Thread %d regs dump\n", i);
		print_regs(i);
	}
}
