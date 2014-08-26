/*
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/init.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/linkage.h>

#include <soc/tegra/flowctrl.h>
#include <soc/tegra/iomap.h>
#include <soc/tegra/sleep.h>

#define CNTFRQ	12000000
#define GIC_DIST_BASE	0x50041000
#define GIC_CPU_BASE	0x50042000

#ifdef CONFIG_PM_SLEEP
/*
 * void gic_init_secure(DistributorBase);
 * Initialize secure copy of GIC at EL3.
 * Derive from u-boot: arch/arm/lib/gic_64.S
 */
ENTRY(gic_init_secure)
	/*
	 * Initialize Distributor
	 * x0: Distributor Base
	 */
	mov	w9, #0x3		// EnableGrp0 | EnableGrp1
	str	w9, [x0, GIC_DIST_CTRL]	// Secure GICD_CTLR
	ldr	w9, [x0, GIC_DIST_CTR]
	and	w10, w9, #0x1f		// ITLinesNumber
	cbz	w10, 1f			// No SPIs
	add	x11, x0, (GIC_DIST_IGROUP + 4)
	mov	w9, #~0			// Config SPIs as Grp1
0:	str	w9, [x11], #0x4
	sub	w10, w10, #0x1
	cbnz	w10, 0b
1:
	ret
ENDPROC(gic_init_secure)

/*
 * void gic_init_secure_percpu(DistributorBase, CpuInterfaceBase);
 * Initialize secure copy of GIC at EL3.
 * Derive from u-boot: arch/arm/lib/gic_64.S
 */
ENTRY(gic_init_secure_percpu)
	/*
	 * Initialize SGIs and PPIs
	 * x0: Distributor Base
	 * x1: Cpu Interface Base
	 */
	mov	w9, #~0				// Config SGIs and PPIs as Grp1
	str	w9, [x0, GIC_DIST_IGROUP]
	mov	w9, #0x1			// Enable SGI 0
	str	w9, [x0, GIC_DIST_ENABLE_SET]

	/*
	 * Initialize Cpu Interface
	 */
	mov	w9, #0x1e7		// Disable IRQ/FIQ Bypass &
					// Enable Ack Group1 Interrupt &
					// EnableGrp0 & EnableGrp1
	str	w9, [x1, GIC_CPU_CTRL]	// Secure GICC_CTLR

	mov	w9, #0x1 << 7		// Non-Secure access to GICC_PMR
	str	w9, [x1, GIC_CPU_PRIMASK]
	ret
ENDPROC(gic_init_secure_percpu)

	__HEAD
/*
 *	tegra_resume
 *
 *	  CPU reset vector when restarting the CPU following
 *	  an LP0 transition.
 */
ENTRY(tegra_resume)
	/*
	 * EL3 initialisation
	 */
	mrs	x0, CurrentEL
	cmp	x0, #0xC		// EL3?
	b.ne	start_ns		// skip EL3 initialisation

	mov	x0, #0x30		// AW | FW
	msr	scr_el3, x0
	msr	cptr_el3, xzr		// Disable copro. traps to EL3

	ldr	x0, =CNTFRQ
	msr	cntfrq_el0, x0

	/*
	 * Re-init GIC
	 */
	mrs	x0, mpidr_el1
	tst	x0, #15
	b.ne	1f			// secondary CPU

	ldr	x0, =GIC_DIST_BASE
	bl	gic_init_secure
1:	ldr	x0, =GIC_DIST_BASE
	ldr	x1, =GIC_CPU_BASE
	bl	gic_init_secure_percpu

	/*
	 * Prepare the switch to the secure EL2_SP2 mode from EL3
	 */
	mov	x0, #0x5b1		// NS EL0/EL1 | HVC | 64 bit EL2
	msr	scr_el3, x0

	mov	x0, #0x33ff
	msr	cptr_el2, x0		// Disable copro. traps to EL2
	msr	sctlr_el2, xzr

	adr	x0, start_ns		// Return after mode switch
	mov	x1, #0x3c9		// EL2_SP2 | D | A | I | F
	msr	elr_el3, x0
	msr	spsr_el3, x1
	eret

start_ns:
	mrs	x0, mpidr_el1
	ands 	x0, x0, #15
	b.ne	secondary_holding_pen	// for slave CPU start up

	/* Clear the flow controller flags for this CPU. */
	cpu_to_csr_reg	x1, x0
	ldr	x2, =TEGRA_FLOW_CTRL_BASE
	ldr	x3, [x2, x1]

	/* Clear event & intr flag */
	orr	x3, x3, #FLOW_CTRL_CSR_INTR_FLAG | FLOW_CTRL_CSR_EVENT_FLAG
	movz	x0, #0x3ffd	// enable, cluster_switch, immed, bitmaps
				// & ext flags for CPU power mgnt
	bic	x3, x3, x0
	str	x3, [x2, x1]

done:
	b	cpu_resume
ENDPROC(tegra_resume)
#endif
