#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/memory.h>

#define _RKPM_SEELP_S_INCLUDE_
#include "pm.h"

#if 0
/********************* console used for sleep.S ******************************/
#define UART_REG_DLL	(0x00)
#define UART_REG_DLH	(0x04)
#define UART_REG_IER	(0x04)
#define UART_REG_FCR	(0x08)
#define UART_REG_LCR	(0x0c)
#define UART_REG_MCR	(0x10)

#define UARTLCR_DLAB	(1 << 7)
#define UARTFCR_DMAEN	(1 << 3)
#define UARTFCR_FIFOEN	(1 << 0)

#define CONSOLE_UART_BASE	(0x10230000)
#define CONSOLE_CLKRATE		(24000000)
#define CONSOLE_BAUDRATE	(1500000)

#define GRF_BASE		(0x10300000)
#define GRF_GPIO1D_IOMUX	(0x000001C)
#define GRF_GPIO1D_VAL		(0x00500050)

.macro early_console_init
#if 1
	ldr r0, =GRF_BASE
	ldr r1, =GRF_GPIO1D_VAL
	str r1, [r0, #GRF_GPIO1D_IOMUX]

	ldr	r0, =CONSOLE_UART_BASE
	ldr	r1, =CONSOLE_CLKRATE
	ldr	r2, =CONSOLE_BAUDRATE
	/* Program the baudrate */
	/* Divisor =  Uart clock / (16 * baudrate) */
	lsl	r2, r2, #4
	mov	r2, #1
	and	r1, r2, #0xff		/* r1 = DLL */
	lsr	r2, r2, #8
	and	r2, r2, #0xff		/* r2 = DLH */
	ldr	r3, [r0, #UART_REG_LCR]
	orr	r3, r3, #UARTLCR_DLAB
	str	r3, [r0, #UART_REG_LCR]	/* enable DLL, DLH programming */
	str	r1, [r0, #UART_REG_DLL]	/* program DLL */
	str	r2, [r0, #UART_REG_DLH]	/* program DLH */
	mov	r2, #~UARTLCR_DLAB
	and	r3, r3, r2
	str	r3, [r0, #UART_REG_LCR]	/* disable DLL, DLH programming */

	/* 8n1 */
	mov	r3, #3
	str	r3, [r0, #UART_REG_LCR]
	/* no interrupt */
	mov	r3, #0
	str	r3, [r0, #UART_REG_IER]
	/* enable fifo, DMA */
	mov	r3, #(UARTFCR_FIFOEN | UARTFCR_DMAEN)
	str	r3, [r0, #UART_REG_FCR]
	/* DTR + RTS */
	mov	r3, #3
	str	r3, [r0, #UART_REG_MCR]
	mov	r0, #1
	dsb	sy
#else
	/* resume console */
	ldr	r0,	=CONSOLE_UART_BASE
	mov	r1,	#0x0
	str	r1,	[r0, #0x4]
	mov	r1,	#0x83
	str	r1,	[r0, #0xc]
	mov	r1,	#0x0d
	str	r1,	[r0, #0x0]
	mov	r1,	#0x00
	str	r1,	[r0, #0x4]
	mov	r1,	#0x3
	str	r1,	[r0, #0xc]
	mov	r1,	#0xc1
	str	r1,	[r0, #0x8]
	dsb	sy
#endif
.endm

.macro early_console_putc ch
	ldr	r0, =CONSOLE_UART_BASE
	mov	r1, #\ch
	str	r1, [r0]
.endm
/********************* console used for sleep.S ******************************/
#endif

#define RV1108_PMU_PWRMODE_COMMON_CON 		(0x20010018)
#define RV1108_DDR_DERETION			(0x00002000)
#define RV1108_CRU_DPLL_CON2 			(0x20200028)
#define RV1108_CRU_DPLL_LOCK			(0x80000000)

.text
ENTRY(rv1108_pm_slp_cpu_while_tst)
	stmfd sp!, { r3 - r12, lr }
1:	mov r3, r3
	b 1b
	ldmfd sp!, { r3 - r12, pc }
ENDPROC(rv1108_pm_slp_cpu_while_tst)

.data
.align
ENTRY(rv1108_pm_slp_cpu_resume)
9:	mov r1, r1

	setmode	PSR_I_BIT | PSR_F_BIT | SVC_MODE, r1  @ set SVC, irqs off

	mrc p15, 0, r1, c0, c0, 5
	/* b . */
	and r1, r1, #0xf
	cmp r1, #0
	beq cpu0Run

cpu1loop:
	WFENE                   /*; wait if it.s locked */
	B cpu1loop           /* ; if any failure, loop */

cpu0Run:
	/* ddr deretention */
	ldr r1, =RV1108_PMU_PWRMODE_COMMON_CON
	ldr r2, =RV1108_DDR_DERETION
	str r2, [r1]

	/* dpll lock */
dpll_unlock:
	ldr r1, =RV1108_CRU_DPLL_CON2
	ldr r2, [r1]
	and r2, r2, #RV1108_CRU_DPLL_LOCK
	cmp r2, #0
	beq dpll_unlock

	adr r1, 9b	/* boot ram base */
	ldr r5, 8f	/* resume data offset, from ram base */
	add r5, r5, r1	/* resume data addr */
	ldr pc, [r5, #(RKPM_BOOTDATA_CPUCODE*4)]
8:      .long (0x00 + 0x700)	/* RKPM_BOOT_CODE_OFFSET+RKPM_BOOT_CODE_SIZE */
ENDPROC(rv1108_pm_slp_cpu_resume)
