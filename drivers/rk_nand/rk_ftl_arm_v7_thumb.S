/*
 * Copyright (c) 2016-2017, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * date: 2019-06-03
 */
	.syntax unified
	.arch armv7-a
	.fpu softvfp
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 2
	.eabi_attribute 30, 4
	.eabi_attribute 34, 1
	.eabi_attribute 18, 4
	.thumb
	.file	"rk_ftl_arm_v7.S"
	.text
	.align	1
	.global	FlashMemCmp8
	.thumb
	.thumb_func
	.type	FlashMemCmp8, %function
FlashMemCmp8:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L10
	push	{r4, r5, lr}
	.save {r4, r5, lr}
	ldrb	r3, [r3]	@ zero_extendqisi2
	cbz	r3, .L4
	ldrb	r4, [r0, #1]	@ zero_extendqisi2
	ldrb	r3, [r1, #1]	@ zero_extendqisi2
	cmp	r4, r3
	beq	.L8
	movs	r3, #0
.L4:
	cmp	r3, r2
	beq	.L8
	ldrb	r5, [r0, r3]	@ zero_extendqisi2
	ldrb	r4, [r1, r3]	@ zero_extendqisi2
	adds	r3, r3, #1
	cmp	r5, r4
	beq	.L4
	mov	r0, r3
	pop	{r4, r5, pc}
.L8:
	movs	r0, #0
	pop	{r4, r5, pc}
.L11:
	.align	2
.L10:
	.word	.LANCHOR0
	.fnend
	.size	FlashMemCmp8, .-FlashMemCmp8
	.align	1
	.global	FlashRsvdBlkChk
	.thumb
	.thumb_func
	.type	FlashRsvdBlkChk, %function
FlashRsvdBlkChk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L15
	ldrb	r2, [r3, #1]	@ zero_extendqisi2
	ldr	r3, [r3, #4]
	muls	r3, r2, r3
	cmp	r1, r3
	bcs	.L14
	adds	r0, r0, #0
	it	ne
	movne	r0, #1
	bx	lr
.L14:
	movs	r0, #1
	bx	lr
.L16:
	.align	2
.L15:
	.word	.LANCHOR0
	.fnend
	.size	FlashRsvdBlkChk, .-FlashRsvdBlkChk
	.align	1
	.global	FlashGetRandomizer
	.thumb
	.thumb_func
	.type	FlashGetRandomizer, %function
FlashGetRandomizer:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L25
	and	r2, r1, #127
	push	{r4, lr}
	.save {r4, lr}
	ldrh	r4, [r3, r2, lsl #1]
	ldr	r3, .L25+4
	ldrb	r3, [r3, #8]	@ zero_extendqisi2
	cbz	r3, .L18
	bl	FlashRsvdBlkChk
	cbz	r0, .L18
	orr	r4, r4, #-1073741824
.L18:
	mov	r0, r4
	pop	{r4, pc}
.L26:
	.align	2
.L25:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.fnend
	.size	FlashGetRandomizer, .-FlashGetRandomizer
	.align	1
	.global	FlashSetRandomizer
	.thumb
	.thumb_func
	.type	FlashSetRandomizer, %function
FlashSetRandomizer:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L35
	and	r2, r1, #127
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0
	ldrh	r5, [r3, r2, lsl #1]
	ldr	r3, .L35+4
	ldrb	r2, [r3, #8]	@ zero_extendqisi2
	mov	r4, r3
	cbz	r2, .L28
	bl	FlashRsvdBlkChk
	cbz	r0, .L28
	orr	r5, r5, #-1073741824
.L28:
	add	r4, r4, r6, lsl #3
	ldr	r3, [r4, #12]
	str	r5, [r3, #336]
	pop	{r4, r5, r6, pc}
.L36:
	.align	2
.L35:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.fnend
	.size	FlashSetRandomizer, .-FlashSetRandomizer
	.align	1
	.global	FlashBlockAlignInit
	.thumb
	.thumb_func
	.type	FlashBlockAlignInit, %function
FlashBlockAlignInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	cmp	r0, #256
	ldr	r3, .L42
	bls	.L38
	mov	r2, #512
	b	.L41
.L38:
	cmp	r0, #128
	bhi	.L40
	str	r0, [r3, #4]
	bx	lr
.L40:
	mov	r2, #256
.L41:
	str	r2, [r3, #4]
	bx	lr
.L43:
	.align	2
.L42:
	.word	.LANCHOR0
	.fnend
	.size	FlashBlockAlignInit, .-FlashBlockAlignInit
	.align	1
	.global	FlashReadCmd
	.thumb
	.thumb_func
	.type	FlashReadCmd, %function
FlashReadCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	lsrs	r2, r1, #16
	ldr	r4, .L45
	add	r4, r4, r0, lsl #3
	ldr	r3, [r4, #12]
	ldrb	r4, [r4, #16]	@ zero_extendqisi2
	add	r3, r3, r4, lsl #8
	movs	r4, #0
	str	r4, [r3, #2056]
	str	r4, [r3, #2052]
	str	r4, [r3, #2052]
	uxtb	r4, r1
	str	r4, [r3, #2052]
	lsrs	r4, r1, #8
	str	r4, [r3, #2052]
	str	r2, [r3, #2052]
	movs	r2, #48
	str	r2, [r3, #2056]
	pop	{r4, lr}
	b	FlashSetRandomizer
.L46:
	.align	2
.L45:
	.word	.LANCHOR0
	.fnend
	.size	FlashReadCmd, .-FlashReadCmd
	.align	1
	.global	FlashReadDpDataOutCmd
	.thumb
	.thumb_func
	.type	FlashReadDpDataOutCmd, %function
FlashReadDpDataOutCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	uxtb	r6, r1
	ldr	r4, .L51
	lsrs	r5, r1, #8
	add	r3, r4, r0, lsl #3
	ldrb	r4, [r4, #60]	@ zero_extendqisi2
	ldr	r2, [r3, #12]
	ldrb	r3, [r3, #16]	@ zero_extendqisi2
	cmp	r4, #1
	lsr	r4, r1, #16
	lsl	r3, r3, #8
	add	r3, r3, r2
	bne	.L48
	movs	r2, #6
	str	r2, [r3, #2056]
	movs	r2, #0
	str	r2, [r3, #2052]
	str	r2, [r3, #2052]
	str	r6, [r3, #2052]
	str	r5, [r3, #2052]
	str	r4, [r3, #2052]
	b	.L50
.L48:
	movs	r2, #0
	str	r2, [r3, #2056]
	str	r2, [r3, #2052]
	str	r2, [r3, #2052]
	str	r6, [r3, #2052]
	str	r5, [r3, #2052]
	str	r4, [r3, #2052]
	movs	r4, #5
	str	r4, [r3, #2056]
	str	r2, [r3, #2052]
	str	r2, [r3, #2052]
.L50:
	movs	r2, #224
	str	r2, [r3, #2056]
	pop	{r4, r5, r6, lr}
	b	FlashSetRandomizer
.L52:
	.align	2
.L51:
	.word	.LANCHOR0
	.fnend
	.size	FlashReadDpDataOutCmd, .-FlashReadDpDataOutCmd
	.align	1
	.global	FlashProgFirstCmd
	.thumb
	.thumb_func
	.type	FlashProgFirstCmd, %function
FlashProgFirstCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	lsrs	r2, r1, #16
	ldr	r4, .L54
	add	r4, r4, r0, lsl #3
	ldr	r3, [r4, #12]
	ldrb	r4, [r4, #16]	@ zero_extendqisi2
	add	r3, r3, r4, lsl #8
	movs	r4, #128
	str	r4, [r3, #2056]
	movs	r4, #0
	str	r4, [r3, #2052]
	str	r4, [r3, #2052]
	uxtb	r4, r1
	str	r4, [r3, #2052]
	lsrs	r4, r1, #8
	str	r4, [r3, #2052]
	str	r2, [r3, #2052]
	pop	{r4, lr}
	b	FlashSetRandomizer
.L55:
	.align	2
.L54:
	.word	.LANCHOR0
	.fnend
	.size	FlashProgFirstCmd, .-FlashProgFirstCmd
	.align	1
	.global	FlashEraseCmd
	.thumb
	.thumb_func
	.type	FlashEraseCmd, %function
FlashEraseCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, lr}
	.save {r4, r5, lr}
	ldr	r5, .L61
	add	r0, r5, r0, lsl #3
	ldrb	r3, [r0, #16]	@ zero_extendqisi2
	ldr	r4, [r0, #12]
	lsls	r3, r3, #8
	cbz	r2, .L57
	adds	r2, r4, r3
	movs	r0, #96
	str	r0, [r2, #2056]
	uxtb	r0, r1
	str	r0, [r2, #2052]
	lsrs	r0, r1, #8
	str	r0, [r2, #2052]
	lsrs	r0, r1, #16
	str	r0, [r2, #2052]
	ldr	r2, [r5, #4]
	add	r1, r1, r2
.L57:
	add	r3, r3, r4
	movs	r2, #96
	str	r2, [r3, #2056]
	uxtb	r2, r1
	str	r2, [r3, #2052]
	lsrs	r2, r1, #8
	lsrs	r1, r1, #16
	str	r2, [r3, #2052]
	str	r1, [r3, #2052]
	movs	r2, #208
	str	r2, [r3, #2056]
	pop	{r4, r5, pc}
.L62:
	.align	2
.L61:
	.word	.LANCHOR0
	.fnend
	.size	FlashEraseCmd, .-FlashEraseCmd
	.align	1
	.global	FlashProgDpSecondCmd
	.thumb
	.thumb_func
	.type	FlashProgDpSecondCmd, %function
FlashProgDpSecondCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, lr}
	.save {r4, r5, lr}
	lsrs	r2, r1, #16
	ldr	r4, .L64
	add	r5, r4, r0, lsl #3
	ldrb	r4, [r4, #55]	@ zero_extendqisi2
	ldr	r3, [r5, #12]
	ldrb	r5, [r5, #16]	@ zero_extendqisi2
	add	r3, r3, r5, lsl #8
	str	r4, [r3, #2056]
	movs	r4, #0
	str	r4, [r3, #2052]
	str	r4, [r3, #2052]
	uxtb	r4, r1
	str	r4, [r3, #2052]
	lsrs	r4, r1, #8
	str	r4, [r3, #2052]
	str	r2, [r3, #2052]
	pop	{r4, r5, lr}
	b	FlashSetRandomizer
.L65:
	.align	2
.L64:
	.word	.LANCHOR0
	.fnend
	.size	FlashProgDpSecondCmd, .-FlashProgDpSecondCmd
	.align	1
	.global	FlashProgSecondCmd
	.thumb
	.thumb_func
	.type	FlashProgSecondCmd, %function
FlashProgSecondCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L67
	add	r0, r3, r0, lsl #3
	ldrb	r2, [r0, #16]	@ zero_extendqisi2
	ldr	r3, [r0, #12]
	add	r3, r3, r2, lsl #8
	movs	r2, #16
	str	r2, [r3, #2056]
	bx	lr
.L68:
	.align	2
.L67:
	.word	.LANCHOR0
	.fnend
	.size	FlashProgSecondCmd, .-FlashProgSecondCmd
	.align	1
	.global	FlashProgDpFirstCmd
	.thumb
	.thumb_func
	.type	FlashProgDpFirstCmd, %function
FlashProgDpFirstCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L70
	add	r0, r2, r0, lsl #3
	ldrb	r2, [r2, #54]	@ zero_extendqisi2
	ldrb	r1, [r0, #16]	@ zero_extendqisi2
	ldr	r3, [r0, #12]
	add	r3, r3, r1, lsl #8
	str	r2, [r3, #2056]
	bx	lr
.L71:
	.align	2
.L70:
	.word	.LANCHOR0
	.fnend
	.size	FlashProgDpFirstCmd, .-FlashProgDpFirstCmd
	.align	1
	.global	js_hash
	.thumb
	.thumb_func
	.type	js_hash, %function
js_hash:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L76
	add	r1, r1, r0
	push	{r4, lr}
	.save {r4, lr}
.L73:
	cmp	r0, r1
	beq	.L75
	lsls	r2, r3, #5
	ldrb	r4, [r0], #1	@ zero_extendqisi2
	add	r2, r2, r3, lsr #2
	add	r2, r2, r4
	eors	r3, r3, r2
	b	.L73
.L75:
	mov	r0, r3
	pop	{r4, pc}
.L77:
	.align	2
.L76:
	.word	1204201446
	.fnend
	.size	js_hash, .-js_hash
	.align	1
	.global	FlashLoadIdbInfo
	.thumb
	.thumb_func
	.type	FlashLoadIdbInfo, %function
FlashLoadIdbInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	movs	r0, #0
	bx	lr
	.fnend
	.size	FlashLoadIdbInfo, .-FlashLoadIdbInfo
	.align	1
	.global	BuildFlashLsbPageTable
	.thumb
	.thumb_func
	.type	BuildFlashLsbPageTable, %function
BuildFlashLsbPageTable:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r1
	cbnz	r0, .L80
	ldr	r3, .L109
.L81:
	strh	r0, [r3, r0, lsl #1]	@ movhi
	adds	r0, r0, #1
	cmp	r0, #256
	bne	.L81
.L85:
	movs	r1, #255
	ldr	r0, .L109+4
	mov	r2, #1024
	uxth	r4, r4
	bl	memset
	movs	r3, #0
	ldr	r1, .L109+8
	b	.L82
.L80:
	cmp	r0, #1
	bne	.L83
	ldr	r5, .L109
	movs	r3, #0
.L84:
	uxth	r2, r3
	cmp	r2, #3
	ite	ls
	movls	r0, #0
	movhi	r0, #1
	bics	r1, r0, r3
	ite	ne
	movne	r1, #2
	moveq	r1, #3
	rsb	r1, r1, r2, lsl #1
	cmp	r0, #0
	it	ne
	movne	r2, r1
	strh	r2, [r5, r3, lsl #1]	@ movhi
	adds	r3, r3, #1
	cmp	r3, #256
	bne	.L84
	b	.L85
.L83:
	cmp	r0, #2
	bne	.L86
	ldr	r0, .L109
	movs	r3, #0
.L87:
	uxth	r2, r3
	lsls	r1, r2, #1
	subs	r1, r1, #1
	cmp	r2, #1
	it	hi
	movhi	r2, r1
	strh	r2, [r0, r3, lsl #1]	@ movhi
	adds	r3, r3, #1
	cmp	r3, #256
	bne	.L87
	b	.L85
.L86:
	cmp	r0, #3
	bne	.L88
	ldr	r5, .L109
	movs	r3, #0
.L89:
	uxth	r2, r3
	cmp	r2, #5
	ite	ls
	movls	r0, #0
	movhi	r0, #1
	bics	r1, r0, r3
	ite	ne
	movne	r1, #4
	moveq	r1, #5
	rsb	r1, r1, r2, lsl #1
	cmp	r0, #0
	it	ne
	movne	r2, r1
	strh	r2, [r5, r3, lsl #1]	@ movhi
	adds	r3, r3, #1
	cmp	r3, #256
	bne	.L89
	b	.L85
.L88:
	cmp	r0, #4
	mov	r3, #0
	bne	.L90
	ldr	r2, .L109+8
	strh	r3, [r2, #76]	@ movhi
	movs	r3, #1
	strh	r0, [r2, #84]	@ movhi
	strh	r3, [r2, #78]	@ movhi
	movs	r3, #2
	strh	r3, [r2, #80]	@ movhi
	movs	r3, #3
	strh	r3, [r2, #82]	@ movhi
	movs	r3, #5
	strh	r3, [r2, #86]	@ movhi
	movs	r3, #7
	strh	r3, [r2, #88]	@ movhi
	movs	r3, #8
	strh	r3, [r2, #90]!	@ movhi
.L91:
	tst	r3, #1
	ite	ne
	movne	r1, #7
	moveq	r1, #6
	rsb	r1, r1, r3, lsl #1
	adds	r3, r3, #1
	strh	r1, [r2, #2]!	@ movhi
	uxth	r3, r3
	cmp	r3, #256
	bne	.L91
	b	.L85
.L90:
	cmp	r0, #5
	bne	.L92
	ldr	r2, .L109
.L93:
	strh	r3, [r2, r3, lsl #1]	@ movhi
	adds	r3, r3, #1
	cmp	r3, #16
	bne	.L93
	ldr	r2, .L109+12
.L94:
	strh	r3, [r2, #2]!	@ movhi
	adds	r3, r3, #2
	uxth	r3, r3
	cmp	r3, #496
	bne	.L94
	b	.L85
.L92:
	cmp	r0, #6
	bne	.L95
	ldr	r5, .L109
.L96:
	uxth	r2, r3
	cmp	r2, #5
	ite	ls
	movls	r0, #0
	movhi	r0, #1
	add	r1, r2, r2, lsl #1
	bics	r6, r0, r3
	ite	ne
	movne	r6, #10
	moveq	r6, #12
	subs	r1, r1, r6
	cmp	r0, #0
	it	ne
	movne	r2, r1
	strh	r2, [r5, r3, lsl #1]	@ movhi
	adds	r3, r3, #1
	cmp	r3, #256
	bne	.L96
	b	.L85
.L95:
	cmp	r0, #9
	bne	.L85
	ldr	r2, .L109+8
	movw	r1, #509
	strh	r3, [r2, #76]	@ movhi
	movs	r3, #1
	strh	r3, [r2, #78]	@ movhi
	movs	r3, #2
	strh	r3, [r2, #80]!	@ movhi
	movs	r3, #3
.L97:
	strh	r3, [r2, #2]!	@ movhi
	adds	r3, r3, #2
	uxth	r3, r3
	cmp	r3, r1
	bne	.L97
	b	.L85
.L82:
	uxth	r2, r3
	cmp	r2, r4
	bcs	.L108
	ldr	r2, .L109
	ldrh	r2, [r2, r3, lsl #1]
	adds	r3, r3, #1
	add	r0, r1, r2, lsl #1
	strh	r2, [r0, #588]	@ movhi
	b	.L82
.L108:
	pop	{r4, r5, r6, pc}
.L110:
	.align	2
.L109:
	.word	.LANCHOR0+76
	.word	.LANCHOR0+588
	.word	.LANCHOR0
	.word	.LANCHOR0+106
	.fnend
	.size	BuildFlashLsbPageTable, .-BuildFlashLsbPageTable
	.align	1
	.global	FlashPrintInfo
	.thumb
	.thumb_func
	.type	FlashPrintInfo, %function
FlashPrintInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	bx	lr
	.fnend
	.size	FlashPrintInfo, .-FlashPrintInfo
	.align	1
	.global	FlashDieInfoInit
	.thumb
	.thumb_func
	.type	FlashDieInfoInit, %function
FlashDieInfoInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L126
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	movs	r6, #0
	ldr	r4, .L126+4
	ldrh	r0, [r3, #266]
	addw	r10, r4, #1788
	strb	r6, [r4, #1612]
	mov	r9, r4
	strb	r6, [r4, #1613]
	bl	FlashBlockAlignInit
	add	r0, r4, #1616
	movs	r1, #8
	bl	__memzero
	add	r0, r4, #1624
	movs	r1, #32
	bl	__memzero
	add	r0, r4, #1656
	movs	r1, #128
	bl	__memzero
	ldr	r5, [r4, #1784]
	adds	r7, r5, #1
.L114:
	mov	r0, r7
	add	r1, r10, r6, lsl #3
	ldrb	r2, [r5]	@ zero_extendqisi2
	bl	FlashMemCmp8
	ldr	r8, .L126+8
	cbnz	r0, .L113
	ldrb	r3, [r9, #1612]	@ zero_extendqisi2
	add	r2, r9, r3, lsl #2
	str	r0, [r2, #1624]
	adds	r2, r3, #1
	add	r3, r3, r9
	strb	r2, [r9, #1612]
	strb	r6, [r3, #1616]
.L113:
	adds	r6, r6, #1
	cmp	r6, #4
	bne	.L114
	ldrb	r3, [r4, #1612]	@ zero_extendqisi2
	ldr	r2, .L126+4
	strb	r3, [r4, #1613]
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	cmp	r3, #2
	beq	.L115
.L119:
	ldrb	r3, [r5, #13]	@ zero_extendqisi2
	ldrb	r2, [r4, #1612]	@ zero_extendqisi2
	smulbb	r2, r2, r3
	ldrh	r3, [r5, #14]
	smulbb	r3, r2, r3
	strh	r3, [r4, #1820]	@ movhi
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L115:
	ldr	r9, [r2, #4]
	movs	r6, #0
	mov	r10, r2
.L118:
	mov	r0, r7
	add	r1, r8, r6, lsl #3
	ldrb	r2, [r5]	@ zero_extendqisi2
	bl	FlashMemCmp8
	cbnz	r0, .L116
	ldrb	r1, [r5, #13]	@ zero_extendqisi2
	ldrh	r3, [r5, #14]
	ldrb	r2, [r10, #1612]	@ zero_extendqisi2
	mul	r1, r9, r1
	and	r3, r3, #65280
	add	r0, r10, r2, lsl #2
	muls	r3, r1, r3
	str	r3, [r0, #1624]
	ldrb	r1, [r5, #23]	@ zero_extendqisi2
	cbz	r1, .L117
	lsls	r3, r3, #1
	str	r3, [r0, #1624]
.L117:
	adds	r3, r2, #1
	add	r2, r2, r4
	strb	r3, [r4, #1612]
	strb	r6, [r2, #1616]
.L116:
	adds	r6, r6, #1
	cmp	r6, #4
	bne	.L118
	b	.L119
.L127:
	.align	2
.L126:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR0+1788
	.fnend
	.size	FlashDieInfoInit, .-FlashDieInfoInit
	.align	1
	.global	ftl_flash_suspend
	.thumb
	.thumb_func
	.type	ftl_flash_suspend, %function
ftl_flash_suspend:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L129
	ldr	r2, [r3, #1824]
	ldr	r1, [r2]
	str	r1, [r3, #1828]
	ldr	r1, [r2, #4]
	str	r1, [r3, #1832]
	ldr	r1, [r2, #8]
	str	r1, [r3, #1836]
	ldr	r1, [r2, #12]
	str	r1, [r3, #1840]
	ldr	r1, [r2, #304]
	str	r1, [r3, #1844]
	ldr	r1, [r2, #308]
	str	r1, [r3, #1848]
	ldr	r1, [r2, #336]
	ldr	r2, [r2, #344]
	str	r1, [r3, #1852]
	str	r2, [r3, #1856]
	bx	lr
.L130:
	.align	2
.L129:
	.word	.LANCHOR0
	.fnend
	.size	ftl_flash_suspend, .-ftl_flash_suspend
	.global	__aeabi_uidiv
	.align	1
	.global	LogAddr2PhyAddr
	.thumb
	.thumb_func
	.type	LogAddr2PhyAddr, %function
LogAddr2PhyAddr:
	.fnstart
	@ args = 4, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r7, r0
	ldr	r0, .L136
	mov	r8, r3
	ldr	r4, [r7, #4]
	mov	r10, r1
	mov	r9, r2
	ldrh	r3, [r0, #1874]
	bic	r4, r4, #-2147483648
	ldrh	r5, [r0, #1872]
	mov	r6, r0
	ldrh	fp, [r0, #4]
	smulbb	r5, r5, r3
	ldrb	r3, [r0]	@ zero_extendqisi2
	cmp	r3, #1
	ubfx	r3, r4, #10, #16
	it	eq
	lsleq	fp, fp, #1
	str	r3, [sp, #4]
	mov	r0, r3
	uxth	r5, r5
	it	eq
	uxtheq	fp, fp
	mov	r1, r5
	bl	__aeabi_uidiv
	cmp	r10, #1
	ubfx	r1, r4, #0, #10
	uxth	r0, r0
	ldr	r3, [sp, #4]
	smulbb	r5, r0, r5
	sub	r3, r3, r5
	uxth	r3, r3
	bne	.L133
	ldrb	r2, [r6, #1888]	@ zero_extendqisi2
	cbnz	r2, .L133
	ldr	r2, .L136
	add	r1, r2, r1, lsl #1
	ldrh	r1, [r1, #76]
.L133:
	add	r6, r6, r0, lsl #2
	ldr	r2, [r6, #1624]
	mla	fp, fp, r3, r2
	ldrb	r3, [sp, #48]	@ zero_extendqisi2
	cmp	r3, #1
	add	r1, r1, fp
	str	r1, [r9]
	str	r0, [r8]
	bls	.L135
	ldr	r0, [r7, #4]
	ldr	r3, [r7, #40]
	add	r0, r0, #1024
	subs	r3, r0, r3
	rsbs	r0, r3, #0
	adcs	r0, r0, r3
	b	.L134
.L135:
	movs	r0, #0
.L134:
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L137:
	.align	2
.L136:
	.word	.LANCHOR0
	.fnend
	.size	LogAddr2PhyAddr, .-LogAddr2PhyAddr
	.align	1
	.global	ftl_read_flash_info
	.thumb
	.thumb_func
	.type	ftl_read_flash_info, %function
ftl_read_flash_info:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movs	r1, #11
	mov	r4, r0
	bl	__memzero
	ldr	r3, .L142
	movs	r5, #1
	ldr	r2, [r3, #1784]
	ldrb	r1, [r2, #9]	@ zero_extendqisi2
	ldr	r2, [r3, #4]
	add	r3, r3, #1616
	smulbb	r2, r1, r2
	strh	r2, [r4, #4]	@ unaligned
	ldrb	r2, [r3, #273]	@ zero_extendqisi2
	strb	r2, [r4, #7]
	ldr	r2, [r3, #276]
	str	r2, [r4]	@ unaligned
	ldr	r2, [r3, #168]
	ldrb	r0, [r3, #-4]	@ zero_extendqisi2
	ldrb	r1, [r2, #9]	@ zero_extendqisi2
	strb	r1, [r4, #6]
	movs	r1, #32
	strb	r1, [r4, #8]
	ldrb	r2, [r2, #7]	@ zero_extendqisi2
	strb	r2, [r4, #9]
	movs	r2, #0
	strb	r2, [r4, #10]
.L139:
	uxtb	r1, r2
	cmp	r1, r0
	bcs	.L141
	ldrb	r1, [r2, r3]	@ zero_extendqisi2
	adds	r2, r2, #1
	lsl	r6, r5, r1
	ldrb	r1, [r4, #10]	@ zero_extendqisi2
	orrs	r1, r1, r6
	strb	r1, [r4, #10]
	b	.L139
.L141:
	pop	{r4, r5, r6, pc}
.L143:
	.align	2
.L142:
	.word	.LANCHOR0
	.fnend
	.size	ftl_read_flash_info, .-ftl_read_flash_info
	.align	1
	.global	FlashScheduleEnSet
	.thumb
	.thumb_func
	.type	FlashScheduleEnSet, %function
FlashScheduleEnSet:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L145
	ldr	r2, [r3, #1896]
	str	r0, [r3, #1896]
	mov	r0, r2
	bx	lr
.L146:
	.align	2
.L145:
	.word	.LANCHOR0
	.fnend
	.size	FlashScheduleEnSet, .-FlashScheduleEnSet
	.align	1
	.global	FlashGetPageSize
	.thumb
	.thumb_func
	.type	FlashGetPageSize, %function
FlashGetPageSize:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L148
	ldr	r3, [r3, #1784]
	ldrb	r0, [r3, #9]	@ zero_extendqisi2
	bx	lr
.L149:
	.align	2
.L148:
	.word	.LANCHOR0
	.fnend
	.size	FlashGetPageSize, .-FlashGetPageSize
	.align	1
	.global	NandcReadDontCaseBusyEn
	.thumb
	.thumb_func
	.type	NandcReadDontCaseBusyEn, %function
NandcReadDontCaseBusyEn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	bx	lr
	.fnend
	.size	NandcReadDontCaseBusyEn, .-NandcReadDontCaseBusyEn
	.align	1
	.global	NandcGetChipIf
	.thumb
	.thumb_func
	.type	NandcGetChipIf, %function
NandcGetChipIf:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L152
	add	r0, r3, r0, lsl #3
	ldrb	r2, [r0, #16]	@ zero_extendqisi2
	ldr	r0, [r0, #12]
	adds	r2, r2, #8
	add	r0, r0, r2, lsl #8
	bx	lr
.L153:
	.align	2
.L152:
	.word	.LANCHOR0
	.fnend
	.size	NandcGetChipIf, .-NandcGetChipIf
	.align	1
	.global	NandcSetDdrPara
	.thumb
	.thumb_func
	.type	NandcSetDdrPara, %function
NandcSetDdrPara:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L155
	ldr	r2, [r3, #1824]
	lsls	r3, r0, #8
	orr	r0, r3, r0, lsl #16
	orr	r3, r0, #1
	str	r3, [r2, #304]
	bx	lr
.L156:
	.align	2
.L155:
	.word	.LANCHOR0
	.fnend
	.size	NandcSetDdrPara, .-NandcSetDdrPara
	.align	1
	.global	NandcSetDdrDiv
	.thumb
	.thumb_func
	.type	NandcSetDdrDiv, %function
NandcSetDdrDiv:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L158
	orr	r0, r0, #16640
	ldr	r3, [r3, #1824]
	str	r0, [r3, #344]
	bx	lr
.L159:
	.align	2
.L158:
	.word	.LANCHOR0
	.fnend
	.size	NandcSetDdrDiv, .-NandcSetDdrDiv
	.align	1
	.global	NandcSetDdrMode
	.thumb
	.thumb_func
	.type	NandcSetDdrMode, %function
NandcSetDdrMode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L163
	ldr	r2, [r3, #1824]
	ldr	r3, [r2]
	cbnz	r0, .L161
	bfi	r3, r0, #13, #1
	b	.L162
.L161:
	orr	r3, r3, #253952
.L162:
	str	r3, [r2]
	bx	lr
.L164:
	.align	2
.L163:
	.word	.LANCHOR0
	.fnend
	.size	NandcSetDdrMode, .-NandcSetDdrMode
	.align	1
	.global	NandcSetMode
	.thumb
	.thumb_func
	.type	NandcSetMode, %function
NandcSetMode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L172
	ands	r1, r0, #6
	ldr	r2, [r3, #1824]
	ldr	r3, [r2]
	beq	.L166
	orr	r3, r3, #24576
	lsls	r1, r0, #29
	bfc	r3, #15, #1
	movw	r1, #8322
	orr	r3, r3, #196608
	str	r1, [r2, #344]
	add	r1, r1, #1040384
	it	mi
	orrmi	r3, r3, #32768
	addw	r1, r1, #3969
	str	r1, [r2, #304]
	movs	r1, #38
	str	r1, [r2, #308]
	movs	r1, #39
	str	r1, [r2, #308]
	b	.L168
.L166:
	bfi	r3, r1, #13, #1
.L168:
	str	r3, [r2]
	movs	r0, #0
	bx	lr
.L173:
	.align	2
.L172:
	.word	.LANCHOR0
	.fnend
	.size	NandcSetMode, .-NandcSetMode
	.align	1
	.global	NandcFlashCs
	.thumb
	.thumb_func
	.type	NandcFlashCs, %function
NandcFlashCs:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L175
	add	r0, r2, r0, lsl #3
	movs	r2, #1
	ldr	r1, [r0, #12]
	ldrb	r0, [r0, #16]	@ zero_extendqisi2
	ldr	r3, [r1]
	lsls	r2, r2, r0
	bfi	r3, r2, #0, #8
	str	r3, [r1]
	bx	lr
.L176:
	.align	2
.L175:
	.word	.LANCHOR0
	.fnend
	.size	NandcFlashCs, .-NandcFlashCs
	.align	1
	.global	NandcFlashDeCs
	.thumb
	.thumb_func
	.type	NandcFlashDeCs, %function
NandcFlashDeCs:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L178
	add	r0, r3, r0, lsl #3
	ldr	r2, [r0, #12]
	ldr	r3, [r2]
	bfc	r3, #0, #8
	bfc	r3, #17, #1
	str	r3, [r2]
	bx	lr
.L179:
	.align	2
.L178:
	.word	.LANCHOR0
	.fnend
	.size	NandcFlashDeCs, .-NandcFlashDeCs
	.align	1
	.global	NandcDelayns
	.thumb
	.thumb_func
	.type	NandcDelayns, %function
NandcDelayns:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	.save {r3, lr}
	addw	r0, r0, #999
	ldr	r3, .L181
	umull	r0, r1, r0, r3
	ldr	r3, .L181+4
	ldr	r3, [r3, #8]
	lsrs	r0, r1, #6
	blx	r3
	movs	r0, #0
	pop	{r3, pc}
.L182:
	.align	2
.L181:
	.word	274877907
	.word	arm_delay_ops
	.fnend
	.size	NandcDelayns, .-NandcDelayns
	.align	1
	.global	FlashReadStatus
	.thumb
	.thumb_func
	.type	FlashReadStatus, %function
FlashReadStatus:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movs	r2, #112
	ldr	r3, .L184
	add	r0, r3, r0, lsl #3
	ldrb	r4, [r0, #16]	@ zero_extendqisi2
	ldr	r5, [r0, #12]
	movs	r0, #80
	add	r3, r5, r4, lsl #8
	adds	r4, r4, #8
	str	r2, [r3, #2056]
	lsls	r4, r4, #8
	bl	NandcDelayns
	ldr	r0, [r5, r4]
	pop	{r3, r4, r5, pc}
.L185:
	.align	2
.L184:
	.word	.LANCHOR0
	.fnend
	.size	FlashReadStatus, .-FlashReadStatus
	.align	1
	.global	ToshibaSetRRPara
	.thumb
	.thumb_func
	.type	ToshibaSetRRPara, %function
ToshibaSetRRPara:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	add	r8, r1, r1, lsl #2
	ldr	r9, .L195+8
	movs	r4, #0
	ldr	r7, .L195
	mov	r5, r0
	add	r10, r9, #288
	mov	r6, r1
.L187:
	ldrb	r3, [r7, #1901]	@ zero_extendqisi2
	cmp	r4, r3
	bcs	.L194
	movs	r3, #85
	str	r3, [r5, #8]
	ldrsb	r3, [r4, r10]
	movs	r0, #200
	str	r3, [r5, #4]
	bl	NandcDelayns
	ldrb	r3, [r7, #1900]	@ zero_extendqisi2
	cmp	r3, #34
	bne	.L188
	add	r3, r4, r8
	add	r3, r3, r10
	b	.L193
.L188:
	cmp	r3, #35
	bne	.L190
	ldr	r2, .L195+4
	add	r3, r4, r8
	add	r3, r3, r2
.L193:
	ldrsb	r3, [r3, #5]
	b	.L192
.L190:
	add	r3, r9, r6
	ldrsb	r3, [r3, #432]
.L192:
	str	r3, [r5]
	adds	r4, r4, #1
	b	.L187
.L194:
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L196:
	.align	2
.L195:
	.word	.LANCHOR0
	.word	.LANCHOR1+336
	.word	.LANCHOR1
	.fnend
	.size	ToshibaSetRRPara, .-ToshibaSetRRPara
	.align	1
	.global	SamsungSetRRPara
	.thumb
	.thumb_func
	.type	SamsungSetRRPara, %function
SamsungSetRRPara:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L201
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	add	r1, r3, r1, lsl #2
	movs	r4, #0
	ldr	r8, .L201+4
	adds	r5, r1, #3
	mov	r9, #161
	mov	r6, r0
	mov	r7, r3
	mov	r10, r4
.L198:
	ldrb	r3, [r8, #1901]	@ zero_extendqisi2
	cmp	r4, r3
	bcs	.L200
	str	r9, [r6, #8]
	mov	r0, #300
	str	r10, [r6]
	ldrsb	r3, [r7, r4]
	adds	r4, r4, #1
	str	r3, [r6]
	ldrsb	r3, [r5, #1]!
	str	r3, [r6]
	bl	NandcDelayns
	b	.L198
.L200:
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L202:
	.align	2
.L201:
	.word	.LANCHOR1+440
	.word	.LANCHOR0
	.fnend
	.size	SamsungSetRRPara, .-SamsungSetRRPara
	.align	1
	.global	HynixSetRRPara
	.thumb
	.thumb_func
	.type	HynixSetRRPara, %function
HynixSetRRPara:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r7, r3
	ldr	r5, .L211
	mov	r6, r0
	mov	r9, r1
	mov	fp, r2
	ldr	r3, [r5, #1784]
	ldrb	r4, [r3, #19]	@ zero_extendqisi2
	lsls	r3, r0, #3
	cmp	r4, #6
	bne	.L204
	add	r4, r5, r0, lsl #6
	addw	r4, r4, #1922
	add	r4, r4, r7, lsl #2
	b	.L205
.L204:
	cmp	r4, #7
	bne	.L206
	mov	r10, #160
	add	r4, r7, r7, lsl #2
	mla	r10, r10, r0, r5
	addw	r10, r10, #1930
	add	r4, r10, r4, lsl #1
	b	.L205
.L206:
	cmp	r4, #8
	iteet	ne
	addne	r4, r7, r3
	addweq	r4, r5, #1930
	addeq	r2, r7, r7, lsl #2
	addne	r4, r5, r4, lsl #3
	ite	eq
	addeq	r4, r4, r2
	addwne	r4, r4, #1922
.L205:
	add	r3, r3, r5
	mov	r0, r6
	add	r9, r9, #-1
	subs	r4, r4, #1
	ldrb	r8, [r3, #16]	@ zero_extendqisi2
	add	r9, r9, fp
	ldr	r10, [r3, #12]
	bl	NandcFlashCs
	add	r1, fp, #-1
	movs	r2, #54
	lsl	r8, r8, #8
	add	r3, r10, r8
	mov	fp, r3
	str	r2, [r3, #2056]
.L208:
	cmp	r1, r9
	beq	.L210
	ldrb	r3, [r1, #1]!	@ zero_extendqisi2
	movs	r0, #200
	str	r3, [fp, #2052]
	str	r1, [sp, #4]
	bl	NandcDelayns
	ldrsb	r3, [r4, #1]!
	str	r3, [fp, #2048]
	ldr	r1, [sp, #4]
	b	.L208
.L210:
	add	r8, r8, r10
	movs	r3, #22
	mov	r0, r6
	add	r5, r5, r6
	str	r3, [r8, #2056]
	bl	NandcFlashDeCs
	strb	r7, [r5, #2756]
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L212:
	.align	2
.L211:
	.word	.LANCHOR0
	.fnend
	.size	HynixSetRRPara, .-HynixSetRRPara
	.align	1
	.global	FlashSetReadRetryDefault
	.thumb
	.thumb_func
	.type	FlashSetReadRetryDefault, %function
FlashSetReadRetryDefault:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r3, .L219
	ldr	r2, [r3, #1784]
	ldrb	r2, [r2, #19]	@ zero_extendqisi2
	subs	r2, r2, #1
	cmp	r2, #7
	bhi	.L213
	movs	r4, #0
	mov	r5, r3
.L214:
	ldr	r2, .L219+4
	uxtb	r0, r4
	ldrb	r3, [r2, r4, lsl #3]	@ zero_extendqisi2
	cmp	r3, #173
	bne	.L215
	ldrb	r1, [r5, #1903]	@ zero_extendqisi2
	adds	r2, r2, #118
	movs	r3, #0
	bl	HynixSetRRPara
.L215:
	adds	r4, r4, #1
	cmp	r4, #4
	bne	.L214
.L213:
	pop	{r3, r4, r5, pc}
.L220:
	.align	2
.L219:
	.word	.LANCHOR0
	.word	.LANCHOR0+1788
	.fnend
	.size	FlashSetReadRetryDefault, .-FlashSetReadRetryDefault
	.align	1
	.global	FlashReadStatusEN
	.thumb
	.thumb_func
	.type	FlashReadStatusEN, %function
FlashReadStatusEN:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldr	r5, .L231
	add	r0, r5, r0, lsl #3
	ldr	r3, [r5, #1784]
	ldrb	r4, [r0, #16]	@ zero_extendqisi2
	ldr	r6, [r0, #12]
	ldrb	r3, [r3, #8]	@ zero_extendqisi2
	cmp	r3, #2
	lsl	r3, r4, #8
	add	r4, r4, #8
	bne	.L222
	cbnz	r2, .L223
	ldrb	r2, [r5, #57]	@ zero_extendqisi2
	b	.L230
.L223:
	ldrb	r2, [r5, #58]	@ zero_extendqisi2
.L230:
	add	r3, r3, r6
	str	r2, [r3, #2056]
	ldrb	r0, [r5, #59]	@ zero_extendqisi2
	cbz	r0, .L226
	movs	r3, #0
	add	r5, r6, r4, lsl #8
.L225:
	cmp	r3, r0
	bcs	.L226
	lsls	r2, r3, #3
	adds	r3, r3, #1
	lsr	r2, r1, r2
	uxtb	r2, r2
	str	r2, [r5, #4]
	b	.L225
.L222:
	add	r3, r3, r6
	movs	r2, #112
	str	r2, [r3, #2056]
.L226:
	movs	r0, #80
	lsls	r4, r4, #8
	bl	NandcDelayns
	ldr	r0, [r6, r4]
	uxtb	r0, r0
	pop	{r4, r5, r6, pc}
.L232:
	.align	2
.L231:
	.word	.LANCHOR0
	.fnend
	.size	FlashReadStatusEN, .-FlashReadStatusEN
	.align	1
	.global	FlashWaitReadyEN
	.thumb
	.thumb_func
	.type	FlashWaitReadyEN, %function
FlashWaitReadyEN:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r0
	mov	r5, r1
	mov	r6, r2
.L237:
	mov	r0, r4
	mov	r1, r5
	mov	r2, r6
	bl	FlashReadStatusEN
	cmp	r0, #255
	mov	r3, r0
	beq	.L237
	lsls	r3, r3, #25
	bpl	.L237
	pop	{r4, r5, r6, pc}
	.fnend
	.size	FlashWaitReadyEN, .-FlashWaitReadyEN
	.align	1
	.global	FlashWaitCmdDone
	.thumb
	.thumb_func
	.type	FlashWaitCmdDone, %function
FlashWaitCmdDone:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r8, r0
	ldr	r5, .L247
	add	r4, r5, r0, lsl #4
	ldr	r3, [r4, #1664]
	ldrb	r6, [r4, #1656]	@ zero_extendqisi2
	cbz	r3, .L242
	add	r5, r5, r8, lsl #2
	mov	r0, r6
	bl	NandcFlashCs
	ldr	r1, [r4, #1660]
	ldr	r2, [r5, #1624]
	mov	r0, r6
	adds	r2, r2, #0
	it	ne
	movne	r2, #1
	bl	FlashWaitReadyEN
	mov	r5, r0
	mov	r0, r6
	bl	NandcFlashDeCs
	ldr	r2, [r4, #1664]
	sbfx	r3, r5, #0, #1
	str	r3, [r2]
	movs	r2, #0
	ldr	r1, [r4, #1668]
	str	r2, [r4, #1664]
	cbz	r1, .L242
	str	r3, [r1]
	str	r2, [r4, #1668]
.L242:
	movs	r0, #0
	pop	{r3, r4, r5, r6, r7, r8, r9, pc}
.L248:
	.align	2
.L247:
	.word	.LANCHOR0
	.fnend
	.size	FlashWaitCmdDone, .-FlashWaitCmdDone
	.align	1
	.thumb
	.thumb_func
	.type	flash_read_ecc, %function
flash_read_ecc:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L250
	push	{r4, lr}
	.save {r4, lr}
	add	r0, r2, r0, lsl #3
	ldrb	r4, [r0, #16]	@ zero_extendqisi2
	ldr	r3, [r0, #12]
	movs	r0, #80
	add	r4, r3, r4, lsl #8
	movs	r3, #122
	str	r3, [r4, #2056]
	bl	NandcDelayns
	ldr	r3, [r4, #2048]
	ldr	r0, [r4, #2048]
	and	r3, r3, #15
	and	r0, r0, #15
	cmp	r0, r3
	it	cc
	movcc	r0, r3
	ldr	r3, [r4, #2048]
	and	r3, r3, #15
	cmp	r0, r3
	it	cc
	movcc	r0, r3
	ldr	r3, [r4, #2048]
	and	r3, r3, #15
	cmp	r0, r3
	it	cc
	movcc	r0, r3
	pop	{r4, pc}
.L251:
	.align	2
.L250:
	.word	.LANCHOR0
	.fnend
	.size	flash_read_ecc, .-flash_read_ecc
	.align	1
	.global	NandcWaitFlashReadyNoDelay
	.thumb
	.thumb_func
	.type	NandcWaitFlashReadyNoDelay, %function
NandcWaitFlashReadyNoDelay:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L257
	push	{r0, r1, r2, r4, r5, lr}
	.save {r4, r5, lr}
	.pad #12
	add	r0, r3, r0, lsl #3
	ldr	r4, .L257+4
	ldr	r5, [r0, #12]
.L254:
	ldr	r3, [r5]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	lsls	r3, r3, #22
	bmi	.L255
	movs	r0, #10
	bl	NandcDelayns
	subs	r4, r4, #1
	bne	.L254
	mov	r0, #-1
	b	.L253
.L255:
	movs	r0, #0
.L253:
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, pc}
.L258:
	.align	2
.L257:
	.word	.LANCHOR0
	.word	100000
	.fnend
	.size	NandcWaitFlashReadyNoDelay, .-NandcWaitFlashReadyNoDelay
	.align	1
	.global	NandcWaitFlashReady
	.thumb
	.thumb_func
	.type	NandcWaitFlashReady, %function
NandcWaitFlashReady:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L264
	push	{r0, r1, r2, r4, r5, lr}
	.save {r4, r5, lr}
	.pad #12
	add	r0, r3, r0, lsl #3
	ldr	r4, .L264+4
	ldr	r5, [r0, #12]
	movs	r0, #150
	bl	NandcDelayns
.L261:
	ldr	r3, [r5]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	lsls	r3, r3, #22
	bmi	.L262
	movs	r0, #10
	bl	NandcDelayns
	subs	r4, r4, #1
	bne	.L261
	mov	r0, #-1
	b	.L260
.L262:
	movs	r0, #0
.L260:
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, pc}
.L265:
	.align	2
.L264:
	.word	.LANCHOR0
	.word	100000
	.fnend
	.size	NandcWaitFlashReady, .-NandcWaitFlashReady
	.align	1
	.global	FlashReset
	.thumb
	.thumb_func
	.type	FlashReset, %function
FlashReset:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L267
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	add	r3, r3, r0, lsl #3
	mov	r4, r0
	ldrb	r6, [r3, #16]	@ zero_extendqisi2
	ldr	r5, [r3, #12]
	bl	NandcFlashCs
	movs	r3, #255
	mov	r0, r4
	add	r5, r5, r6, lsl #8
	str	r3, [r5, #2056]
	bl	NandcWaitFlashReady
	mov	r0, r4
	pop	{r4, r5, r6, lr}
	b	NandcFlashDeCs
.L268:
	.align	2
.L267:
	.word	.LANCHOR0
	.fnend
	.size	FlashReset, .-FlashReset
	.align	1
	.global	flash_enter_slc_mode
	.thumb
	.thumb_func
	.type	flash_enter_slc_mode, %function
flash_enter_slc_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r5, r0
	ldr	r4, .L271
	ldrb	r3, [r4, #1888]	@ zero_extendqisi2
	cbz	r3, .L269
	add	r4, r4, r5, lsl #3
	bl	NandcFlashCs
	movs	r0, #100
	ldr	r3, [r4, #12]
	ldrb	r4, [r4, #16]	@ zero_extendqisi2
	add	r4, r3, r4, lsl #8
	movs	r3, #239
	str	r3, [r4, #2056]
	movs	r3, #145
	str	r3, [r4, #2052]
	bl	NandcDelayns
	movs	r3, #0
	movs	r2, #1
	str	r3, [r4, #2048]
	movs	r0, #100
	str	r2, [r4, #2048]
	str	r3, [r4, #2048]
	str	r3, [r4, #2048]
	bl	NandcDelayns
	mov	r0, r5
	bl	NandcWaitFlashReadyNoDelay
	movs	r3, #218
	mov	r0, r5
	str	r3, [r4, #2056]
	pop	{r3, r4, r5, lr}
	b	NandcWaitFlashReady
.L269:
	pop	{r3, r4, r5, pc}
.L272:
	.align	2
.L271:
	.word	.LANCHOR0
	.fnend
	.size	flash_enter_slc_mode, .-flash_enter_slc_mode
	.align	1
	.global	flash_exit_slc_mode
	.thumb
	.thumb_func
	.type	flash_exit_slc_mode, %function
flash_exit_slc_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r5, r0
	ldr	r4, .L275
	ldrb	r3, [r4, #1888]	@ zero_extendqisi2
	cbz	r3, .L273
	add	r4, r4, r5, lsl #3
	bl	NandcFlashCs
	movs	r0, #100
	ldr	r3, [r4, #12]
	ldrb	r4, [r4, #16]	@ zero_extendqisi2
	add	r4, r3, r4, lsl #8
	movs	r3, #239
	str	r3, [r4, #2056]
	movs	r3, #145
	str	r3, [r4, #2052]
	bl	NandcDelayns
	movs	r3, #2
	movs	r0, #100
	str	r3, [r4, #2048]
	movs	r3, #1
	str	r3, [r4, #2048]
	movs	r3, #0
	str	r3, [r4, #2048]
	str	r3, [r4, #2048]
	bl	NandcDelayns
	mov	r0, r5
	bl	NandcWaitFlashReadyNoDelay
	movs	r3, #223
	mov	r0, r5
	str	r3, [r4, #2056]
	pop	{r3, r4, r5, lr}
	b	NandcWaitFlashReady
.L273:
	pop	{r3, r4, r5, pc}
.L276:
	.align	2
.L275:
	.word	.LANCHOR0
	.fnend
	.size	flash_exit_slc_mode, .-flash_exit_slc_mode
	.align	1
	.global	FlashEraseBlock
	.thumb
	.thumb_func
	.type	FlashEraseBlock, %function
FlashEraseBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r0
	mov	r5, r1
	mov	r6, r2
	bl	NandcWaitFlashReady
	mov	r0, r4
	bl	NandcFlashCs
	mov	r2, r6
	mov	r1, r5
	mov	r0, r4
	bl	FlashEraseCmd
	mov	r0, r4
	bl	NandcWaitFlashReady
	mov	r1, r5
	mov	r0, r4
	bl	FlashReadStatus
	mov	r5, r0
	mov	r0, r4
	bl	NandcFlashDeCs
	and	r0, r5, #1
	pop	{r4, r5, r6, pc}
	.fnend
	.size	FlashEraseBlock, .-FlashEraseBlock
	.align	1
	.global	FlashSetInterfaceMode
	.thumb
	.thumb_func
	.type	FlashSetInterfaceMode, %function
FlashSetInterfaceMode:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r5, .L306
	movs	r4, #0
	movs	r7, #239
	mov	lr, #128
	mov	ip, #1
	mov	r8, #35
	ldrb	r3, [r5, #2760]	@ zero_extendqisi2
	mov	r9, #32
	mov	r10, #5
	and	r2, r3, #1
	str	r2, [sp, #12]
	and	r2, r3, #4
	uxtb	r3, r2
	mov	r2, r4
	str	r3, [sp, #8]
	add	r3, r5, #12
	str	r3, [sp, #4]
.L288:
	ldr	r1, .L306+4
	ldr	r3, [sp, #4]
	ldrb	r1, [r4, r1]	@ zero_extendqisi2
	ldr	r6, [r4, r3]
	add	r3, r3, r4
	cmp	r1, #152
	ldr	r3, [r3, #4]
	beq	.L279
	cmp	r1, #69
	beq	.L279
	cmp	r1, #173
	beq	.L279
	cmp	r1, #44
	bne	.L280
.L279:
	cmp	r0, #1
	uxtb	r3, r3
	bne	.L281
	ldr	r5, [sp, #12]
	cbz	r5, .L280
	lsls	r3, r3, #8
	cmp	r1, #173
	add	fp, r6, r3
	str	r7, [fp, #2056]
	bne	.L282
	str	r0, [fp, #2052]
	b	.L305
.L282:
	cmp	r1, #44
	itete	eq
	streq	r0, [fp, #2052]
	strne	lr, [fp, #2052]
	streq	r10, [fp, #2048]
	strne	r0, [fp, #2048]
	b	.L286
.L281:
	ldr	r5, [sp, #8]
	cbz	r5, .L280
	lsls	r3, r3, #8
	cmp	r1, #173
	add	fp, r6, r3
	str	r7, [fp, #2056]
	bne	.L285
	str	ip, [fp, #2052]
	str	r9, [fp, #2048]
	b	.L286
.L285:
	cmp	r1, #44
	bne	.L287
	str	ip, [fp, #2052]
	str	r8, [fp, #2048]
	b	.L286
.L287:
	str	lr, [fp, #2052]
.L305:
	str	r2, [fp, #2048]
.L286:
	add	r3, r3, r6
	str	r2, [r3, #2048]
	str	r2, [r3, #2048]
	str	r2, [r3, #2048]
.L280:
	adds	r4, r4, #8
	cmp	r4, #32
	bne	.L288
	movs	r0, #0
	bl	NandcWaitFlashReady
	movs	r0, #0
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L307:
	.align	2
.L306:
	.word	.LANCHOR0
	.word	.LANCHOR0+1788
	.fnend
	.size	FlashSetInterfaceMode, .-FlashSetInterfaceMode
	.align	1
	.global	FlashReadSpare
	.thumb
	.thumb_func
	.type	FlashReadSpare, %function
FlashReadSpare:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r2
	ldr	r5, .L309
	ldr	r3, .L309+4
	add	r5, r5, r0, lsl #3
	ldrb	r3, [r3, #265]	@ zero_extendqisi2
	ldrb	r2, [r5, #16]	@ zero_extendqisi2
	ldr	r4, [r5, #12]
	lsls	r3, r3, #9
	add	r4, r4, r2, lsl #8
	movs	r2, #0
	str	r2, [r4, #2056]
	str	r3, [r4, #2052]
	lsrs	r3, r3, #8
	str	r3, [r4, #2052]
	uxtb	r3, r1
	str	r3, [r4, #2052]
	lsrs	r3, r1, #8
	lsrs	r1, r1, #16
	str	r3, [r4, #2052]
	movs	r3, #48
	str	r1, [r4, #2052]
	str	r3, [r4, #2056]
	bl	NandcWaitFlashReady
	ldr	r3, [r4, #2048]
	strb	r3, [r6]
	pop	{r4, r5, r6, pc}
.L310:
	.align	2
.L309:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.fnend
	.size	FlashReadSpare, .-FlashReadSpare
	.align	1
	.global	SandiskProgTestBadBlock
	.thumb
	.thumb_func
	.type	SandiskProgTestBadBlock, %function
SandiskProgTestBadBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L312
	push	{r4, lr}
	.save {r4, lr}
	add	r2, r2, r0, lsl #3
	ldrb	r4, [r2, #16]	@ zero_extendqisi2
	ldr	r3, [r2, #12]
	add	r4, r3, r4, lsl #8
	movs	r3, #162
	str	r3, [r4, #2056]
	movs	r3, #128
	str	r3, [r4, #2056]
	movs	r3, #0
	str	r3, [r4, #2052]
	str	r3, [r4, #2052]
	uxtb	r3, r1
	str	r3, [r4, #2052]
	lsrs	r3, r1, #8
	lsrs	r1, r1, #16
	str	r3, [r4, #2052]
	str	r1, [r4, #2052]
	movs	r3, #16
	str	r3, [r4, #2056]
	bl	NandcWaitFlashReady
	movs	r3, #112
	movs	r0, #80
	str	r3, [r4, #2056]
	bl	NandcDelayns
	ldr	r0, [r4, #2048]
	and	r0, r0, #1
	pop	{r4, pc}
.L313:
	.align	2
.L312:
	.word	.LANCHOR0
	.fnend
	.size	SandiskProgTestBadBlock, .-SandiskProgTestBadBlock
	.align	1
	.global	SandiskSetRRPara
	.thumb
	.thumb_func
	.type	SandiskSetRRPara, %function
SandiskSetRRPara:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movs	r3, #239
	mov	r5, r0
	str	r3, [r0, #8]
	movs	r3, #17
	str	r3, [r0, #4]
	movs	r0, #200
	mov	r4, r1
	bl	NandcDelayns
	ldr	r0, .L321
	ldr	r1, .L321+4
	add	r4, r4, r4, lsl #2
	movs	r2, #0
	sub	r6, r0, #48
.L315:
	ldrb	r3, [r1, #1901]	@ zero_extendqisi2
	cmp	r2, r3
	bcs	.L320
	ldrb	r3, [r1, #1900]	@ zero_extendqisi2
	cmp	r3, #67
	add	r3, r2, r4
	ite	eq
	addeq	r3, r3, r6
	addne	r3, r3, r0
	adds	r2, r2, #1
	ldrsb	r3, [r3, #5]
	str	r3, [r5]
	b	.L315
.L320:
	movs	r0, #0
	pop	{r4, r5, r6, lr}
	b	NandcWaitFlashReady
.L322:
	.align	2
.L321:
	.word	.LANCHOR1+336
	.word	.LANCHOR0
	.fnend
	.size	SandiskSetRRPara, .-SandiskSetRRPara
	.align	1
	.global	micron_auto_read_calibration_config
	.thumb
	.thumb_func
	.type	micron_auto_read_calibration_config, %function
micron_auto_read_calibration_config:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, r0
	mov	r5, r1
	bl	NandcWaitFlashReady
	ldr	r3, .L324
	movs	r0, #200
	add	r2, r3, r4, lsl #3
	ldrb	r4, [r2, #16]	@ zero_extendqisi2
	ldr	r3, [r2, #12]
	add	r4, r3, r4, lsl #8
	movs	r3, #239
	str	r3, [r4, #2056]
	movs	r3, #150
	str	r3, [r4, #2052]
	bl	NandcDelayns
	str	r5, [r4, #2048]
	movs	r3, #0
	str	r3, [r4, #2048]
	str	r3, [r4, #2048]
	str	r3, [r4, #2048]
	pop	{r3, r4, r5, pc}
.L325:
	.align	2
.L324:
	.word	.LANCHOR0
	.fnend
	.size	micron_auto_read_calibration_config, .-micron_auto_read_calibration_config
	.align	1
	.global	FlashEraseSLc2KBlocks
	.thumb
	.thumb_func
	.type	FlashEraseSLc2KBlocks, %function
FlashEraseSLc2KBlocks:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	movs	r5, #0
	ldr	r8, .L337+4
	.pad #20
	sub	sp, sp, #20
	mov	r6, r0
	mov	r9, r1
	mov	r7, r5
.L327:
	cmp	r7, r9
	beq	.L336
	rsb	r3, r7, r9
	add	r2, sp, #8
	adds	r0, r6, r5
	movs	r1, #0
	uxtb	r3, r3
	str	r3, [sp]
	add	r3, sp, #12
	bl	LogAddr2PhyAddr
	ldrb	r2, [r8, #1612]	@ zero_extendqisi2
	ldr	r3, [sp, #12]
	cmp	r3, r2
	bcc	.L328
	mov	r3, #-1
	str	r3, [r6, r5]
	b	.L329
.L328:
	add	r2, r8, r3
	add	r3, r8, r3, lsl #4
	ldrb	r4, [r2, #1616]	@ zero_extendqisi2
	strb	r4, [r3, #1656]
	mov	r0, r4
	bl	NandcWaitFlashReady
	mov	r0, r4
	bl	NandcFlashCs
	movs	r2, #0
	mov	r0, r4
	ldr	r1, [sp, #8]
	bl	FlashEraseCmd
	mov	r0, r4
	bl	NandcWaitFlashReady
	mov	r0, r4
	ldr	r1, [sp, #8]
	bl	FlashReadStatus
	movs	r2, #0
	ldr	r3, [sp, #8]
	sbfx	r0, r0, #0, #1
	str	r0, [r6, r5]
	mov	r0, r4
	ldr	r1, [r8, #4]
	add	r1, r1, r3
	bl	FlashEraseCmd
	mov	r0, r4
	bl	NandcWaitFlashReady
	mov	r0, r4
	ldr	r1, [sp, #8]
	bl	FlashReadStatus
	lsls	r3, r0, #31
	itt	mi
	movmi	r3, #-1
	strmi	r3, [r6, r5]
	ldr	r3, [r6, r5]
	adds	r3, r3, #1
	bne	.L331
	ldr	r0, .L337
	ldr	r1, [sp, #8]
	bl	printk
.L331:
	mov	r0, r4
	bl	NandcFlashDeCs
.L329:
	adds	r7, r7, #1
	adds	r5, r5, #36
	b	.L327
.L336:
	movs	r0, #0
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, pc}
.L338:
	.align	2
.L337:
	.word	.LC1
	.word	.LANCHOR0
	.fnend
	.size	FlashEraseSLc2KBlocks, .-FlashEraseSLc2KBlocks
	.align	1
	.global	FlashReadDpCmd
	.thumb
	.thumb_func
	.type	FlashReadDpCmd, %function
FlashReadDpCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L343
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	r5, r2
	add	r2, r3, r0, lsl #3
	ldrb	r6, [r3, #52]	@ zero_extendqisi2
	mov	r7, r1
	uxtb	lr, r1
	ldrb	r4, [r2, #16]	@ zero_extendqisi2
	uxtb	r10, r5
	ldr	ip, [r2, #12]
	lsr	r9, r5, #8
	ldrb	r2, [r3, #60]	@ zero_extendqisi2
	mov	r8, r0
	lsls	r4, r4, #8
	ldrb	r3, [r3, #53]	@ zero_extendqisi2
	cmp	r2, #1
	add	r4, r4, ip
	lsr	r2, r1, #8
	lsr	r5, r5, #16
	lsr	r1, r1, #16
	str	r6, [r4, #2056]
	bne	.L340
	movs	r6, #0
	str	r6, [r4, #2052]
	str	r6, [r4, #2052]
	str	lr, [r4, #2052]
	str	r2, [r4, #2052]
	str	r1, [r4, #2052]
	str	r3, [r4, #2056]
	bl	NandcWaitFlashReady
	str	r6, [r4, #2056]
	str	r6, [r4, #2052]
	str	r6, [r4, #2052]
	b	.L342
.L340:
	str	lr, [r4, #2052]
	str	r2, [r4, #2052]
	str	r1, [r4, #2052]
	str	r3, [r4, #2056]
.L342:
	str	r10, [r4, #2052]
	movs	r3, #48
	str	r9, [r4, #2052]
	mov	r0, r8
	str	r5, [r4, #2052]
	mov	r1, r7
	str	r3, [r4, #2056]
	pop	{r4, r5, r6, r7, r8, r9, r10, lr}
	b	FlashSetRandomizer
.L344:
	.align	2
.L343:
	.word	.LANCHOR0
	.fnend
	.size	FlashReadDpCmd, .-FlashReadDpCmd
	.align	1
	.global	ftl_flash_de_init
	.thumb
	.thumb_func
	.type	ftl_flash_de_init, %function
ftl_flash_de_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movs	r0, #0
	ldr	r4, .L355
	bl	NandcWaitFlashReady
	bl	FlashSetReadRetryDefault
	ldr	r0, [r4, #2764]
	cbz	r0, .L346
	movs	r0, #0
	bl	flash_enter_slc_mode
	b	.L347
.L346:
	bl	flash_exit_slc_mode
.L347:
	ldrb	r3, [r4, #2768]	@ zero_extendqisi2
	ldr	r5, .L355
	cbz	r3, .L348
	ldrb	r3, [r5, #2760]	@ zero_extendqisi2
	lsls	r3, r3, #31
	bpl	.L348
	movs	r0, #1
	bl	FlashSetInterfaceMode
	movs	r0, #1
	bl	NandcSetMode
	movs	r3, #0
	strb	r3, [r5, #2768]
.L348:
	ldr	r3, [r4, #12]
	movs	r0, #0
	str	r0, [r3, #336]
	pop	{r3, r4, r5, pc}
.L356:
	.align	2
.L355:
	.word	.LANCHOR0
	.fnend
	.size	ftl_flash_de_init, .-ftl_flash_de_init
	.align	1
	.global	NandcRandmzSel
	.thumb
	.thumb_func
	.type	NandcRandmzSel, %function
NandcRandmzSel:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L358
	add	r0, r3, r0, lsl #3
	ldr	r3, [r0, #12]
	str	r1, [r3, #336]
	bx	lr
.L359:
	.align	2
.L358:
	.word	.LANCHOR0
	.fnend
	.size	NandcRandmzSel, .-NandcRandmzSel
	.global	__aeabi_idiv
	.align	1
	.global	NandcTimeCfg
	.thumb
	.thumb_func
	.type	NandcTimeCfg, %function
NandcTimeCfg:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	mov	r4, r0
	movs	r0, #0
	bl	rknand_get_clk_rate
	ldr	r1, .L370
	bl	__aeabi_idiv
	ldr	r3, .L370+4
	ldr	r3, [r3, #1824]
	cmp	r0, #250
	ble	.L361
	movw	r2, #8354
	b	.L368
.L361:
	cmp	r0, #220
	bgt	.L369
	cmp	r0, #185
	ble	.L364
	movw	r2, #4226
	b	.L368
.L364:
	cmp	r0, #160
	ble	.L365
	movw	r2, #4194
	b	.L368
.L365:
	cmp	r4, #35
	bhi	.L366
	movw	r2, #4193
	b	.L368
.L366:
	cmp	r4, #99
	bls	.L367
.L369:
	movw	r2, #8322
	b	.L368
.L367:
	movw	r2, #4225
.L368:
	str	r2, [r3, #4]
	pop	{r4, pc}
.L371:
	.align	2
.L370:
	.word	1000000
	.word	.LANCHOR0
	.fnend
	.size	NandcTimeCfg, .-NandcTimeCfg
	.align	1
	.global	FlashTimingCfg
	.thumb
	.thumb_func
	.type	FlashTimingCfg, %function
FlashTimingCfg:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	sub	r3, r0, #4192
	subs	r3, r3, #1
	cmp	r3, #1
	bls	.L373
	sub	r3, r0, #4224
	subs	r3, r3, #1
	cmp	r3, #1
	bls	.L373
	movw	r3, #8322
	cmp	r0, r3
	bne	.L374
.L373:
	ldr	r3, .L375
	ldr	r3, [r3, #1824]
	str	r0, [r3, #4]
.L374:
	ldr	r3, .L375+4
	ldrb	r0, [r3, #277]	@ zero_extendqisi2
	b	NandcTimeCfg
.L376:
	.align	2
.L375:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.fnend
	.size	FlashTimingCfg, .-FlashTimingCfg
	.align	1
	.global	NandcInit
	.thumb
	.thumb_func
	.type	NandcInit, %function
NandcInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movs	r1, #0
	ldr	r3, .L379
	movs	r2, #1
	movs	r5, #0
	str	r1, [r3, #16]
	str	r0, [r3, #12]
	str	r2, [r3, #24]
	movs	r2, #2
	str	r0, [r3, #20]
	str	r2, [r3, #32]
	movs	r2, #3
	str	r0, [r3, #28]
	str	r0, [r3, #36]
	str	r0, [r3, #1824]
	str	r2, [r3, #40]
	ldr	r2, [r0]
	and	r2, r2, #253952
	ubfx	r4, r2, #13, #1
	bfi	r2, r1, #13, #1
	ldr	r1, [r0, #352]
	orr	r2, r2, #256
	str	r4, [r3, #2772]
	movw	r4, #2049
	ubfx	r1, r1, #16, #4
	str	r1, [r3, #2776]
	ldr	r1, [r0, #352]
	cmp	r1, r4
	mov	r4, r3
	str	r1, [r3, #2780]
	itt	eq
	moveq	r3, #8
	streq	r3, [r4, #2776]
	str	r2, [r0]
	movs	r0, #40
	ldr	r3, [r4, #1824]
	str	r5, [r3, #336]
	bl	NandcTimeCfg
	ldr	r3, [r4, #1824]
	movw	r2, #8322
	mov	r0, #36864
	str	r2, [r3, #344]
	ldr	r2, .L379+4
	str	r2, [r3, #304]
	bl	ftl_malloc
	str	r5, [r4, #2812]
	str	r5, [r4, #2820]
	str	r0, [r4, #2784]
	str	r0, [r4, #2788]
	add	r0, r0, #32768
	str	r0, [r4, #2792]
	pop	{r3, r4, r5, pc}
.L380:
	.align	2
.L379:
	.word	.LANCHOR0
	.word	1579009
	.fnend
	.size	NandcInit, .-NandcInit
	.align	1
	.global	NandcGetTimeCfg
	.thumb
	.thumb_func
	.type	NandcGetTimeCfg, %function
NandcGetTimeCfg:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, lr}
	.save {r4, r5, lr}
	ldr	r4, .L382
	ldr	r5, [r4, #1824]
	ldr	r5, [r5, #4]
	str	r5, [r0]
	ldr	r0, [r4, #1824]
	ldr	r0, [r0]
	str	r0, [r1]
	ldr	r1, [r4, #1824]
	ldr	r1, [r1, #304]
	str	r1, [r2]
	ldr	r1, [r4, #1824]
	ldr	r2, [r1, #308]
	ldr	r1, [r1, #344]
	uxtb	r2, r2
	orr	r2, r2, r1, lsl #16
	str	r2, [r3]
	pop	{r4, r5, pc}
.L383:
	.align	2
.L382:
	.word	.LANCHOR0
	.fnend
	.size	NandcGetTimeCfg, .-NandcGetTimeCfg
	.align	1
	.global	NandcBchSel
	.thumb
	.thumb_func
	.type	NandcBchSel, %function
NandcBchSel:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L392
	movs	r1, #1
	ldr	r2, [r3, #1824]
	str	r0, [r3, #2824]
	movs	r3, #0
	str	r1, [r2, #8]
	movs	r1, #16
	cmp	r0, r1
	bfi	r3, r1, #8, #8
	bfc	r3, #18, #1
	bne	.L385
.L388:
	bfc	r3, #4, #1
	b	.L386
.L385:
	cmp	r0, #24
	bne	.L387
	orr	r3, r3, #16
	b	.L386
.L387:
	cmp	r0, #40
	orr	r3, r3, #262144
	orr	r3, r3, #16
	beq	.L388
.L386:
	orr	r3, r3, #1
	str	r3, [r2, #12]
	bx	lr
.L393:
	.align	2
.L392:
	.word	.LANCHOR0
	.fnend
	.size	NandcBchSel, .-NandcBchSel
	.align	1
	.global	FlashBchSel
	.thumb
	.thumb_func
	.type	FlashBchSel, %function
FlashBchSel:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L395
	strb	r0, [r3, #1889]
	b	NandcBchSel
.L396:
	.align	2
.L395:
	.word	.LANCHOR0
	.fnend
	.size	FlashBchSel, .-FlashBchSel
	.align	1
	.global	ftl_flash_resume
	.thumb
	.thumb_func
	.type	ftl_flash_resume, %function
ftl_flash_resume:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L405
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movs	r5, #0
	ldr	r2, [r3, #1824]
	addw	r6, r3, #1788
	ldr	r1, [r3, #1828]
	mov	r4, r3
	str	r1, [r2]
	ldr	r1, [r3, #1832]
	ldr	r2, [r3, #1824]
	str	r1, [r2, #4]
	ldr	r1, [r3, #1836]
	str	r1, [r2, #8]
	ldr	r1, [r3, #1840]
	str	r1, [r2, #12]
	ldr	r1, [r3, #1844]
	str	r1, [r2, #304]
	ldr	r1, [r3, #1848]
	str	r1, [r2, #308]
	ldr	r1, [r3, #1852]
	str	r1, [r2, #336]
	ldr	r1, [r3, #1856]
	str	r1, [r2, #344]
.L399:
	ldrb	r3, [r6, r5, lsl #3]	@ zero_extendqisi2
	subs	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L398
	uxtb	r0, r5
	bl	FlashReset
.L398:
	adds	r5, r5, #1
	cmp	r5, #4
	bne	.L399
	ldrb	r3, [r4, #2768]	@ zero_extendqisi2
	ldr	r5, .L405
	cbz	r3, .L400
	movs	r0, #1
	bl	NandcSetMode
	ldrb	r0, [r5, #2760]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r5, #2760]	@ zero_extendqisi2
	bl	NandcSetMode
	ldrb	r0, [r5, #1845]	@ zero_extendqisi2
	bl	NandcSetDdrPara
.L400:
	ldr	r3, [r4, #1784]
	pop	{r4, r5, r6, lr}
	ldrb	r0, [r3, #20]	@ zero_extendqisi2
	b	FlashBchSel
.L406:
	.align	2
.L405:
	.word	.LANCHOR0
	.fnend
	.size	ftl_flash_resume, .-ftl_flash_resume
	.align	1
	.global	ftl_nandc_get_irq_status
	.thumb
	.thumb_func
	.type	ftl_nandc_get_irq_status, %function
ftl_nandc_get_irq_status:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r0, [r0, #372]
	bx	lr
	.fnend
	.size	ftl_nandc_get_irq_status, .-ftl_nandc_get_irq_status
	.align	1
	.global	rk_nandc_flash_ready
	.thumb
	.thumb_func
	.type	rk_nandc_flash_ready, %function
rk_nandc_flash_ready:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #368]
	orr	r3, r3, #2
	str	r3, [r0, #368]
	ldr	r3, [r0, #364]
	bic	r3, r3, #2
	str	r3, [r0, #364]
	bx	lr
	.fnend
	.size	rk_nandc_flash_ready, .-rk_nandc_flash_ready
	.align	1
	.global	NandcIqrWaitFlashReady
	.thumb
	.thumb_func
	.type	NandcIqrWaitFlashReady, %function
NandcIqrWaitFlashReady:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	mov	r4, r0
	bl	rk_nandc_rb_irq_flag_init
	ldr	r3, [r4, #368]
	orr	r3, r3, #2
	str	r3, [r4, #368]
	ldr	r3, [r4, #364]
	orr	r3, r3, #2
	str	r3, [r4, #364]
	ldr	r3, [r4]
	lsls	r3, r3, #22
	bmi	.L410
	mov	r0, r4
	pop	{r4, lr}
	b	wait_for_nand_flash_ready
.L410:
	ldr	r3, [r4, #368]
	orr	r3, r3, #2
	str	r3, [r4, #368]
	ldr	r3, [r4, #364]
	bic	r3, r3, #2
	str	r3, [r4, #364]
	pop	{r4, pc}
	.fnend
	.size	NandcIqrWaitFlashReady, .-NandcIqrWaitFlashReady
	.align	1
	.global	FlashEraseBlocks
	.thumb
	.thumb_func
	.type	FlashEraseBlocks, %function
FlashEraseBlocks:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r9, r0
	ldr	r4, .L445
	.pad #28
	sub	sp, sp, #28
	mov	r10, r1
	mov	r8, r2
	ldrb	r5, [r4]	@ zero_extendqisi2
	cbz	r5, .L429
	mov	r1, r2
	bl	FlashEraseSLc2KBlocks
	b	.L413
.L429:
	mov	fp, r4
.L412:
	cmp	r5, r8
	bcs	.L443
	movs	r3, #36
	movs	r1, #0
	add	r2, sp, #16
	mul	r6, r3, r5
	add	r3, r9, r6
	str	r3, [sp, #12]
	rsb	r3, r5, r8
	ldr	r0, [sp, #12]
	uxtb	r3, r3
	str	r3, [sp]
	add	r3, sp, #20
	bl	LogAddr2PhyAddr
	ldrb	r3, [r4, #1612]	@ zero_extendqisi2
	mov	r7, r0
	ldr	r0, [sp, #20]
	cmp	r0, r3
	bcc	.L414
	mov	r3, #-1
	str	r3, [r9, r6]
	b	.L415
.L414:
	ldrb	r3, [fp, #2828]	@ zero_extendqisi2
	cmp	r3, #0
	add	r3, r4, r0, lsl #4
	it	eq
	moveq	r7, #0
	ldr	r3, [r3, #1664]
	cbz	r3, .L417
	uxtb	r0, r0
	bl	FlashWaitCmdDone
.L417:
	ldr	r1, [sp, #20]
	ldr	r3, [sp, #12]
	lsls	r2, r1, #4
	add	lr, r4, r2
	str	r3, [lr, #1664]
	movs	r3, #0
	str	r3, [lr, #1668]
	ldr	r3, [sp, #16]
	str	r3, [lr, #1660]
	cbz	r7, .L418
	add	r3, r6, #36
	add	r3, r3, r9
	str	r3, [lr, #1668]
.L418:
	add	r1, r1, r4
	add	r2, r2, r4
	ldrb	r6, [r1, #1616]	@ zero_extendqisi2
	mov	r0, r6
	strb	r6, [r2, #1656]
	bl	NandcFlashCs
	cmp	r10, #1
	mov	r0, r6
	bne	.L419
	ldrb	r3, [fp, #1888]	@ zero_extendqisi2
	cbz	r3, .L419
	bl	flash_enter_slc_mode
	b	.L420
.L419:
	bl	flash_exit_slc_mode
.L420:
	ldr	r3, [sp, #20]
	mov	r0, r6
	ldr	r1, [sp, #16]
	add	r5, r5, r7
	add	r3, r4, r3, lsl #2
	ldr	r2, [r3, #1624]
	adds	r2, r2, #0
	it	ne
	movne	r2, #1
	bl	FlashWaitReadyEN
	mov	r0, r6
	mov	r2, r7
	ldr	r1, [sp, #16]
	bl	FlashEraseCmd
	mov	r0, r6
	bl	NandcFlashDeCs
.L415:
	adds	r5, r5, #1
	b	.L412
.L443:
	ldr	r0, [r4, #1824]
	movs	r5, #0
	bl	NandcIqrWaitFlashReady
	ldr	r6, .L445
	add	r7, r6, #1656
.L422:
	ldrb	r2, [r4, #1612]	@ zero_extendqisi2
	ldr	r3, .L445
	cmp	r5, r2
	bcs	.L444
	uxtb	r0, r5
	bl	FlashWaitCmdDone
	cmp	r10, #1
	bne	.L423
	ldrb	r3, [r6, #1888]	@ zero_extendqisi2
	cbz	r3, .L423
	lsls	r3, r5, #4
	ldrb	r0, [r7, r3]	@ zero_extendqisi2
	bl	flash_exit_slc_mode
.L423:
	adds	r5, r5, #1
	b	.L422
.L444:
	ldr	r2, [r3, #2832]
	cbnz	r2, .L425
.L427:
	movs	r0, #0
	b	.L413
.L425:
	ldrb	r3, [r3, #1788]	@ zero_extendqisi2
	cmp	r3, #69
	bne	.L427
	movs	r3, #0
	movs	r2, #36
	mov	r1, r3
.L426:
	cmp	r3, r8
	beq	.L427
	mul	r0, r2, r3
	adds	r3, r3, #1
	str	r1, [r9, r0]
	b	.L426
.L413:
	add	sp, sp, #28
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L446:
	.align	2
.L445:
	.word	.LANCHOR0
	.fnend
	.size	FlashEraseBlocks, .-FlashEraseBlocks
	.align	1
	.global	rk_nandc_flash_xfer_completed
	.thumb
	.thumb_func
	.type	rk_nandc_flash_xfer_completed, %function
rk_nandc_flash_xfer_completed:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #368]
	orr	r3, r3, #1
	str	r3, [r0, #368]
	ldr	r3, [r0, #364]
	bic	r3, r3, #1
	str	r3, [r0, #364]
	bx	lr
	.fnend
	.size	rk_nandc_flash_xfer_completed, .-rk_nandc_flash_xfer_completed
	.align	1
	.global	NandcSendDumpDataStart
	.thumb
	.thumb_func
	.type	NandcSendDumpDataStart, %function
NandcSendDumpDataStart:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, [r0, #16]
	.pad #8
	sub	sp, sp, #8
	ldr	r3, .L449
	str	r2, [sp, #4]
	ldr	r2, [sp, #4]
	bic	r2, r2, #4
	str	r2, [sp, #4]
	ldr	r2, [sp, #4]
	str	r2, [r0, #16]
	str	r3, [r0, #8]
	orr	r3, r3, #4
	str	r3, [r0, #8]
	add	sp, sp, #8
	@ sp needed
	bx	lr
.L450:
	.align	2
.L449:
	.word	538969130
	.fnend
	.size	NandcSendDumpDataStart, .-NandcSendDumpDataStart
	.align	1
	.global	NandcSendDumpDataDone
	.thumb
	.thumb_func
	.type	NandcSendDumpDataDone, %function
NandcSendDumpDataDone:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	.pad #8
	sub	sp, sp, #8
.L452:
	ldr	r3, [r0, #8]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	lsls	r3, r3, #11
	bpl	.L452
	add	sp, sp, #8
	@ sp needed
	bx	lr
	.fnend
	.size	NandcSendDumpDataDone, .-NandcSendDumpDataDone
	.align	1
	.global	NandcXferStart
	.thumb
	.thumb_func
	.type	NandcXferStart, %function
NandcXferStart:
	.fnstart
	@ args = 8, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	mov	r8, r2
	mov	r7, r1
	ldr	r2, [sp, #60]
	ldr	fp, [sp, #56]
	cbnz	r2, .L469
	adds	r2, fp, #0
	it	ne
	movne	r2, #1
	b	.L456
.L469:
	movs	r2, #1
.L456:
	ldr	r5, .L474
	movs	r4, #0
	add	r0, r5, r0, lsl #3
	ldr	r6, [r0, #12]
	ldrb	r1, [r0, #16]	@ zero_extendqisi2
	movs	r0, #16
	ldr	r9, [r6, #12]
	bfi	r9, r0, #8, #8
	bfi	r9, r4, #3, #1
	bfi	r4, r7, #1, #1
	bfi	r9, r1, #5, #3
	orr	r4, r4, #8
	movs	r1, #1
	bfi	r4, r1, #5, #2
	lsrs	r3, r3, r1
	orr	r4, r4, #536870912
	orr	r4, r4, #1024
	bfi	r4, r3, #4, #1
	ldr	r3, [r5, #2776]
	cmp	r3, #3
	bls	.L457
	ldr	r3, [r6, #16]
	str	r3, [sp, #12]
	ldr	r3, [sp, #12]
	bic	r3, r3, #4
	str	r3, [sp, #12]
	cmp	r2, #0
	beq	.L458
	cbnz	r7, .L459
.L467:
	mov	r0, r6
	add	r8, r8, #1
	bl	rk_nandc_xfer_irq_flag_init
	ldr	r3, [r6, #368]
	asr	r8, r8, #1
	orr	r3, r3, #1
	str	r3, [r6, #368]
	ldr	r3, [r6, #364]
	bfi	r4, r8, #22, #6
	orr	r3, r3, #1
	str	r3, [r6, #364]
	cmp	fp, #0
	beq	.L460
	mov	r0, fp
	b	.L461
.L459:
	ldr	r3, [r5, #2824]
	mov	ip, r5
	ldr	r0, [sp, #60]
	cmp	r3, #25
	ite	cc
	movcc	r3, #64
	movcs	r3, #128
	str	r3, [sp]
	lsr	r3, r8, #1
	str	r3, [sp, #4]
	movs	r3, #0
	mov	r1, r3
.L463:
	ldr	r2, [sp, #4]
	cmp	r1, r2
	bcs	.L467
	ldr	r2, [sp, #60]
	lsr	r10, r3, #2
	cbz	r2, .L464
	ldrh	r2, [r0, #2]
	ldrh	lr, [r0], #4
	orr	lr, lr, r2, lsl #16
	ldr	r2, [ip, #2792]
	str	lr, [r2, r10, lsl #2]
	b	.L465
.L464:
	ldr	r2, [ip, #2792]
	mov	lr, r2
	mov	r2, #-1
	str	r2, [lr, r10, lsl #2]
.L465:
	ldr	r2, [sp]
	adds	r1, r1, #1
	add	r3, r3, r2
	b	.L463
.L460:
	ldr	r0, [r5, #2788]
.L461:
	ldr	r3, [r5, #2792]
	ubfx	r8, r4, #22, #5
	mov	r2, r7
	str	r0, [r5, #2796]
	lsl	r1, r8, #10
	str	r3, [r5, #2800]
	bl	rknand_dma_map_single
	mov	r2, r7
	lsl	r1, r8, #7
	clz	r7, r7
	lsrs	r7, r7, #5
	str	r0, [r5, #2804]
	ldr	r0, [r5, #2800]
	bl	rknand_dma_map_single
	movs	r3, #1
	str	r3, [r5, #2812]
	tst	fp, #3
	ldr	r3, [r5, #2804]
	str	r0, [r5, #2808]
	str	r3, [r6, #20]
	ldr	r3, [r5, #2808]
	str	r3, [r6, #24]
	mov	r3, #0
	str	r3, [sp, #12]
	ldr	r3, [sp, #12]
	bic	r3, r3, #15872
	orr	r3, r3, #8192
	str	r3, [sp, #12]
	ldr	r3, [sp, #12]
	orr	r3, r3, #448
	str	r3, [sp, #12]
	itttt	eq
	ldreq	r3, [sp, #12]
	biceq	r3, r3, #56
	orreq	r3, r3, #16
	streq	r3, [sp, #12]
	ldr	r3, [sp, #12]
	orr	r3, r3, #4
	str	r3, [sp, #12]
	ldr	r3, [sp, #12]
	bic	r3, r3, #2
	orr	r7, r3, r7, lsl #1
	str	r7, [sp, #12]
	ldr	r3, [sp, #12]
	orr	r3, r3, #1
	str	r3, [sp, #12]
.L458:
	ldr	r3, [sp, #12]
	str	r3, [r6, #16]
.L457:
	str	r9, [r6, #12]
	str	r4, [r6, #8]
	orr	r4, r4, #4
	str	r4, [r6, #8]
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L475:
	.align	2
.L474:
	.word	.LANCHOR0
	.fnend
	.size	NandcXferStart, .-NandcXferStart
	.align	1
	.global	Ftl_log2
	.thumb
	.thumb_func
	.type	Ftl_log2, %function
Ftl_log2:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	movs	r1, #0
	movs	r2, #1
.L477:
	cmp	r2, r0
	uxth	r3, r1
	add	r1, r1, #1
	bhi	.L479
	lsls	r2, r2, #1
	b	.L477
.L479:
	subs	r0, r3, #1
	uxth	r0, r0
	bx	lr
	.fnend
	.size	Ftl_log2, .-Ftl_log2
	.align	1
	.global	FtlPrintInfo
	.thumb
	.thumb_func
	.type	FtlPrintInfo, %function
FtlPrintInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	bx	lr
	.fnend
	.size	FtlPrintInfo, .-FtlPrintInfo
	.align	1
	.global	FtlSysBlkNumInit
	.thumb
	.thumb_func
	.type	FtlSysBlkNumInit, %function
FtlSysBlkNumInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L483
	cmp	r0, #23
	it	ls
	movls	r0, #24
	ldrh	r2, [r3, #2840]
	ldrh	r1, [r3, #2850]
	str	r0, [r3, #2836]
	muls	r2, r0, r2
	subs	r0, r1, r0
	ldr	r1, [r3, #2856]
	strh	r0, [r3, #2848]	@ movhi
	movs	r0, #0
	str	r2, [r3, #2844]
	subs	r2, r1, r2
	str	r2, [r3, #2852]
	bx	lr
.L484:
	.align	2
.L483:
	.word	.LANCHOR0
	.fnend
	.size	FtlSysBlkNumInit, .-FtlSysBlkNumInit
	.align	1
	.global	FtlConstantsInit
	.thumb
	.thumb_func
	.type	FtlConstantsInit, %function
FtlConstantsInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldrh	r1, [r0, #14]
	mov	r5, r0
	ldr	r4, .L513
	ldrh	lr, [r0, #8]
	str	r1, [sp]
	ldrh	r1, [sp]
	ldrh	r2, [r0, #10]
	ldrh	r3, [r0, #12]
	addw	r0, r4, #2868
	strh	r1, [r4, #2850]	@ movhi
	movs	r1, #0
	strh	lr, [r4, #2860]	@ movhi
	strh	r2, [r4, #2862]	@ movhi
	strh	r3, [r4, #2864]	@ movhi
.L486:
	strb	r1, [r1, r0]
	adds	r1, r1, #1
	cmp	r1, #32
	bne	.L486
	ldrh	r0, [r5, #14]
	ldrh	r1, [r5, #20]
	cmp	r1, r0, lsr #8
	bcs	.L487
	uxtb	r10, r3
	ldr	r9, .L513+4
	lsl	r1, r10, #1
	uxtb	r1, r1
	str	r1, [sp, #8]
	subs	r1, r2, #1
	muls	r1, r3, r1
	str	r1, [sp, #4]
	movs	r1, #0
.L488:
	cmp	r1, r3
	bcs	.L490
	ldr	r6, [sp, #4]
	rsb	ip, r3, r1
	uxtb	r0, r1
	add	ip, ip, r9
	adds	r6, r1, r6
	add	r6, r9, r6
	str	r6, [sp, #12]
	movs	r6, #0
	mov	r8, r6
.L491:
	cmp	r8, r2
	add	r6, r6, r3
	bcs	.L512
	ldr	r7, [sp, #12]
	add	fp, r0, r10
	strb	r0, [ip, r6]
	add	r8, r8, #1
	strb	fp, [r7, r6]
	ldr	r7, [sp, #8]
	add	r0, r0, r7
	uxtb	r0, r0
	b	.L491
.L512:
	adds	r1, r1, #1
	b	.L488
.L490:
	lsls	r2, r2, #1
	strh	r2, [r4, #2862]	@ movhi
	ldr	r2, [sp]
	lsrs	r7, r2, #1
	strh	r7, [r4, #2850]	@ movhi
.L487:
	cmp	lr, #1
	mov	r2, #5
	strh	r2, [r4, #2900]	@ movhi
	mov	r2, #0
	strh	r2, [r4, #2902]	@ movhi
	it	eq
	ldreq	r2, .L513
	ldrb	r9, [r4]	@ zero_extendqisi2
	it	eq
	strheq	lr, [r2, #2900]	@ movhi
	mov	r2, #4352
	strh	r2, [r4, #2904]	@ movhi
	cmp	r9, #0
	beq	.L493
	ldr	r2, .L513
	mov	r1, #384
	strh	r1, [r2, #2904]	@ movhi
.L493:
	ldrh	r7, [r4, #2862]
	ldrh	r6, [r4, #2850]
	ldrh	r8, [r5, #16]
	smulbb	r7, r7, r3
	ldrh	fp, [r5, #20]
	smulbb	r3, r6, r3
	ldrh	r1, [r5, #18]
	mov	r0, fp
	strh	r8, [r4, #2908]	@ movhi
	strh	fp, [r4, #2914]	@ movhi
	uxth	r7, r7
	strh	r1, [r4, #2910]	@ movhi
	str	r1, [sp]
	strh	r3, [r4, #2906]	@ movhi
	smulbb	r3, r7, r8
	strh	r7, [r4, #2840]	@ movhi
	strh	r3, [r4, #2912]	@ movhi
	bl	Ftl_log2
	lsl	r3, fp, #9
	cmp	r6, #1024
	uxth	r3, r3
	strh	r3, [r4, #2918]	@ movhi
	it	hi
	ldrhi	r2, .L513
	lsr	r3, r3, #8
	strh	r3, [r4, #2920]	@ movhi
	ldrh	r3, [r5, #26]
	strh	r3, [r4, #2922]	@ movhi
	mul	r3, r6, r7
	str	r3, [r4, #2856]
	itt	hi
	uxtbhi	r3, r6
	strhhi	r3, [r2, #2902]	@ movhi
	mov	r10, r0
	ldrh	r5, [r4, #2902]
	ldr	r1, [sp]
	subs	r5, r6, r5
	strh	r0, [r4, #2916]	@ movhi
	ldrh	r0, [r4, #2904]
	muls	r5, r7, r5
	ldr	r3, .L513
	lsls	r0, r0, #3
	mul	r1, r1, fp
	str	r3, [sp]
	mul	r5, fp, r5
	mul	r5, r8, r5
	asrs	r5, r5, #11
	str	r5, [r4, #2924]
	bl	__aeabi_idiv
	ldr	r3, [sp]
	uxth	r0, r0
	cmp	r0, #4
	itet	ls
	movls	r2, #4
	strhhi	r0, [r3, #2928]	@ movhi
	strhls	r2, [r3, #2928]	@ movhi
	cmp	r9, #0
	beq	.L497
	mov	r3, #640
	strh	r3, [r4, #2904]	@ movhi
.L497:
	ldrh	r3, [r4, #2904]
	lsls	r6, r6, #6
	mov	r1, r7
	ldr	r5, .L513
	asr	r3, r3, r10
	add	r10, r10, #9
	asr	r6, r6, r10
	adds	r3, r3, #2
	strh	r3, [r4, #2930]	@ movhi
	uxth	r0, r6
	strh	r6, [r4, #2932]	@ movhi
	add	r6, r0, #8
	mul	r3, r7, r0
	ldrh	r0, [r4, #2928]
	str	r3, [r4, #2936]
	bl	__aeabi_uidiv
	cmp	r7, #1
	uxtah	r0, r6, r0
	ldr	r6, .L513
	it	eq
	addeq	r0, r0, #4
	str	r0, [r5, #2836]
	ldrh	r0, [r4, #2836]
	bl	FtlSysBlkNumInit
	ldr	r3, [r4, #2836]
	ldr	r5, [r4, #2852]
	mov	r0, #2048
	ldrh	r7, [r4, #2914]
	str	r3, [r4, #2940]
	ldrh	r3, [r4, #2908]
	lsls	r5, r5, #2
	mov	r1, r7
	muls	r5, r3, r5
	ldrh	r3, [r4, #2916]
	adds	r3, r3, #9
	lsrs	r5, r5, r3
	adds	r5, r5, #2
	uxth	r5, r5
	strh	r5, [r4, #2944]	@ movhi
	bl	__aeabi_idiv
	ldrh	r2, [r4, #2928]
	movs	r3, #0
	str	r3, [r4, #1892]
	adds	r3, r2, #3
	strh	r3, [r4, #2928]	@ movhi
	ldr	r3, [r4, #2936]
	adds	r1, r3, #3
	str	r1, [r4, #2936]
	strh	r0, [r4, #2946]	@ movhi
	ldrb	r0, [r4, #1888]	@ zero_extendqisi2
	cbz	r0, .L500
	adds	r2, r2, #4
	adds	r3, r3, #5
	strh	r2, [r6, #2928]	@ movhi
	b	.L511
.L500:
	cmp	r1, #7
	bhi	.L501
	movs	r3, #8
.L511:
	str	r3, [r6, #2936]
.L501:
	ldrh	r2, [r4, #2848]
	movs	r3, #0
	strh	r3, [r4, #2948]	@ movhi
	movs	r0, #0
	lsrs	r3, r2, #3
	add	r3, r3, r2, lsl #1
	adds	r3, r3, #52
	add	r5, r3, r5, lsl #2
	cmp	r5, r7, lsl #9
	ittt	cc
	movcc	r2, #1
	ldrcc	r3, .L513
	strhcc	r2, [r3, #2948]	@ movhi
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L514:
	.align	2
.L513:
	.word	.LANCHOR0
	.word	.LANCHOR0+2868
	.fnend
	.size	FtlConstantsInit, .-FtlConstantsInit
	.align	1
	.global	IsBlkInVendorPart
	.thumb
	.thumb_func
	.type	IsBlkInVendorPart, %function
IsBlkInVendorPart:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L522
	ldrh	r3, [r2, #2950]
	cbz	r3, .L521
	ldr	r3, [r2, #2952]
	ldrh	r2, [r2, #2928]
	add	r2, r3, r2, lsl #1
.L517:
	cmp	r3, r2
	beq	.L521
	ldrh	r1, [r3], #2
	cmp	r1, r0
	bne	.L517
	movs	r0, #1
	bx	lr
.L521:
	movs	r0, #0
	bx	lr
.L523:
	.align	2
.L522:
	.word	.LANCHOR0
	.fnend
	.size	IsBlkInVendorPart, .-IsBlkInVendorPart
	.align	1
	.global	FtlCacheMetchLpa
	.thumb
	.thumb_func
	.type	FtlCacheMetchLpa, %function
FtlCacheMetchLpa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L532
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldr	r3, [r2, #2956]
	cbz	r3, .L531
	ldr	r5, [r2, #2960]
	movs	r6, #36
	movs	r2, #0
.L527:
	mla	r4, r6, r2, r5
	ldr	r4, [r4, #16]
	cmp	r4, r0
	bcc	.L526
	cmp	r4, r1
	bls	.L529
.L526:
	adds	r2, r2, #1
	cmp	r2, r3
	bne	.L527
.L531:
	movs	r0, #0
	pop	{r4, r5, r6, pc}
.L529:
	movs	r0, #1
	pop	{r4, r5, r6, pc}
.L533:
	.align	2
.L532:
	.word	.LANCHOR0
	.fnend
	.size	FtlCacheMetchLpa, .-FtlCacheMetchLpa
	.align	1
	.global	FtlGetCap
	.thumb
	.thumb_func
	.type	FtlGetCap, %function
FtlGetCap:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L535
	ldr	r0, [r3, #1892]
	bx	lr
.L536:
	.align	2
.L535:
	.word	.LANCHOR0
	.fnend
	.size	FtlGetCap, .-FtlGetCap
	.align	1
	.global	ftl_get_density
	.thumb
	.thumb_func
	.type	ftl_get_density, %function
ftl_get_density:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L538
	ldr	r0, [r3, #1892]
	bx	lr
.L539:
	.align	2
.L538:
	.word	.LANCHOR0
	.fnend
	.size	ftl_get_density, .-ftl_get_density
	.align	1
	.global	FtlGetLpn
	.thumb
	.thumb_func
	.type	FtlGetLpn, %function
FtlGetLpn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L541
	ldr	r0, [r3, #2964]
	bx	lr
.L542:
	.align	2
.L541:
	.word	.LANCHOR0
	.fnend
	.size	FtlGetLpn, .-FtlGetLpn
	.align	1
	.global	FtlBbmMapBadBlock
	.thumb
	.thumb_func
	.type	FtlBbmMapBadBlock, %function
FtlBbmMapBadBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	.pad #12
	mov	r6, r0
	ldr	r5, .L544
	ldrh	r4, [r5, #2906]
	mov	r1, r4
	bl	__aeabi_uidiv
	uxth	r2, r0
	smulbb	r3, r2, r4
	add	r1, r5, r2, lsl #2
	ldr	r4, [r1, #2996]
	movs	r1, #1
	subs	r3, r6, r3
	uxth	r3, r3
	and	r0, r3, #31
	lsrs	r7, r3, #5
	lsls	r1, r1, r0
	ldr	r0, [r4, r7, lsl #2]
	orrs	r1, r1, r0
	ldr	r0, .L544+4
	str	r1, [r4, r7, lsl #2]
	str	r1, [sp]
	mov	r1, r6
	bl	printk
	ldrh	r3, [r5, #2974]
	movs	r0, #0
	adds	r3, r3, #1
	strh	r3, [r5, #2974]	@ movhi
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, pc}
.L545:
	.align	2
.L544:
	.word	.LANCHOR0
	.word	.LC2
	.fnend
	.size	FtlBbmMapBadBlock, .-FtlBbmMapBadBlock
	.global	__aeabi_uidivmod
	.align	1
	.global	FtlBbmIsBadBlock
	.thumb
	.thumb_func
	.type	FtlBbmIsBadBlock, %function
FtlBbmIsBadBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r7, r0
	ldr	r5, .L547
	ldrh	r6, [r5, #2906]
	mov	r1, r6
	bl	__aeabi_uidivmod
	mov	r0, r7
	uxth	r4, r1
	mov	r1, r6
	bl	__aeabi_uidiv
	lsrs	r2, r4, #5
	and	r4, r4, #31
	uxth	r0, r0
	add	r5, r5, r0, lsl #2
	ldr	r3, [r5, #2996]
	ldr	r0, [r3, r2, lsl #2]
	lsrs	r0, r0, r4
	and	r0, r0, #1
	pop	{r3, r4, r5, r6, r7, pc}
.L548:
	.align	2
.L547:
	.word	.LANCHOR0
	.fnend
	.size	FtlBbmIsBadBlock, .-FtlBbmIsBadBlock
	.align	1
	.global	FtlBbtInfoPrint
	.thumb
	.thumb_func
	.type	FtlBbtInfoPrint, %function
FtlBbtInfoPrint:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	bx	lr
	.fnend
	.size	FtlBbtInfoPrint, .-FtlBbtInfoPrint
	.align	1
	.global	FtlBbtMemInit
	.thumb
	.thumb_func
	.type	FtlBbtMemInit, %function
FtlBbtMemInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r0, .L551
	movw	r3, #65535
	movs	r1, #255
	movs	r2, #16
	addw	r0, r0, #2980
	strh	r3, [r0, #-12]	@ movhi
	movs	r3, #0
	strh	r3, [r0, #-6]	@ movhi
	b	memset
.L552:
	.align	2
.L551:
	.word	.LANCHOR0
	.fnend
	.size	FtlBbtMemInit, .-FtlBbtMemInit
	.align	1
	.global	FtlBbtCalcTotleCnt
	.thumb
	.thumb_func
	.type	FtlBbtCalcTotleCnt, %function
FtlBbtCalcTotleCnt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movs	r5, #0
	ldr	r4, .L561
	ldrh	r6, [r4, #2906]
	ldrh	r3, [r4, #2862]
	mov	r4, r5
	muls	r6, r3, r6
.L554:
	uxth	r0, r5
	cmp	r0, r6
	bge	.L560
	bl	FtlBbmIsBadBlock
	cbz	r0, .L555
	adds	r4, r4, #1
	uxth	r4, r4
.L555:
	adds	r5, r5, #1
	b	.L554
.L560:
	mov	r0, r4
	pop	{r4, r5, r6, pc}
.L562:
	.align	2
.L561:
	.word	.LANCHOR0
	.fnend
	.size	FtlBbtCalcTotleCnt, .-FtlBbtCalcTotleCnt
	.align	1
	.global	V2P_block
	.thumb
	.thumb_func
	.type	V2P_block, %function
V2P_block:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r4, r1
	ldr	r6, .L564
	mov	r7, r0
	ldrh	r5, [r6, #2864]
	mov	r1, r5
	bl	__aeabi_uidivmod
	mov	r0, r7
	smlabb	r4, r4, r5, r1
	mov	r1, r5
	bl	__aeabi_uidiv
	ldrh	r3, [r6, #2906]
	smlabb	r0, r3, r0, r4
	uxth	r0, r0
	pop	{r3, r4, r5, r6, r7, pc}
.L565:
	.align	2
.L564:
	.word	.LANCHOR0
	.fnend
	.size	V2P_block, .-V2P_block
	.align	1
	.global	P2V_plane
	.thumb
	.thumb_func
	.type	P2V_plane, %function
P2V_plane:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L567
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0
	ldrh	r1, [r3, #2906]
	ldrh	r5, [r3, #2864]
	bl	__aeabi_uidiv
	mov	r1, r5
	smulbb	r4, r0, r5
	mov	r0, r6
	bl	__aeabi_uidivmod
	add	r1, r1, r4
	uxth	r0, r1
	pop	{r4, r5, r6, pc}
.L568:
	.align	2
.L567:
	.word	.LANCHOR0
	.fnend
	.size	P2V_plane, .-P2V_plane
	.align	1
	.global	P2V_block_in_plane
	.thumb
	.thumb_func
	.type	P2V_block_in_plane, %function
P2V_block_in_plane:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	ldr	r4, .L570
	ldrh	r1, [r4, #2906]
	bl	__aeabi_uidivmod
	uxth	r0, r1
	ldrh	r1, [r4, #2864]
	bl	__aeabi_uidiv
	uxth	r0, r0
	pop	{r4, pc}
.L571:
	.align	2
.L570:
	.word	.LANCHOR0
	.fnend
	.size	P2V_block_in_plane, .-P2V_block_in_plane
	.align	1
	.global	ftl_cmp_data_ver
	.thumb
	.thumb_func
	.type	ftl_cmp_data_ver, %function
ftl_cmp_data_ver:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	cmp	r0, r1
	bls	.L573
	subs	r0, r0, r1
	cmp	r0, #-2147483648
	ite	hi
	movhi	r0, #0
	movls	r0, #1
	bx	lr
.L573:
	subs	r0, r1, r0
	cmp	r0, #-2147483648
	ite	ls
	movls	r0, #0
	movhi	r0, #1
	bx	lr
	.fnend
	.size	ftl_cmp_data_ver, .-ftl_cmp_data_ver
	.align	1
	.global	FtlFreeSysBlkQueueInit
	.thumb
	.thumb_func
	.type	FtlFreeSysBlkQueueInit, %function
FtlFreeSysBlkQueueInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L576
	mov	r1, #2048
	push	{r4, lr}
	.save {r4, lr}
	movs	r4, #0
	strh	r0, [r3, #3028]	@ movhi
	addw	r0, r3, #3036
	strh	r4, [r3, #3030]	@ movhi
	strh	r4, [r3, #3032]	@ movhi
	strh	r4, [r3, #3034]	@ movhi
	bl	__memzero
	mov	r0, r4
	pop	{r4, pc}
.L577:
	.align	2
.L576:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueInit, .-FtlFreeSysBlkQueueInit
	.align	1
	.global	FtlFreeSysBlkQueueEmpty
	.thumb
	.thumb_func
	.type	FtlFreeSysBlkQueueEmpty, %function
FtlFreeSysBlkQueueEmpty:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L579
	ldrh	r0, [r3, #3034]
	clz	r0, r0
	lsrs	r0, r0, #5
	bx	lr
.L580:
	.align	2
.L579:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueEmpty, .-FtlFreeSysBlkQueueEmpty
	.align	1
	.global	FtlFreeSysBlkQueueFull
	.thumb
	.thumb_func
	.type	FtlFreeSysBlkQueueFull, %function
FtlFreeSysBlkQueueFull:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L582
	ldrh	r0, [r3, #3034]
	sub	r3, r0, #1024
	rsbs	r0, r3, #0
	adcs	r0, r0, r3
	bx	lr
.L583:
	.align	2
.L582:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueFull, .-FtlFreeSysBlkQueueFull
	.align	1
	.global	FtlFreeSysBlkQueueIn
	.thumb
	.thumb_func
	.type	FtlFreeSysBlkQueueIn, %function
FtlFreeSysBlkQueueIn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	subs	r3, r0, #1
	movw	r2, #65533
	mov	r6, r0
	uxth	r3, r3
	cmp	r3, r2
	bhi	.L584
	ldr	r4, .L593
	ldrh	r3, [r4, #3034]
	cmp	r3, #1024
	beq	.L584
	cbz	r1, .L586
	ldr	r5, .L593+4
	ldr	r3, [r5, #740]
	cbnz	r3, .L586
	bl	P2V_block_in_plane
	movs	r1, #1
	lsls	r3, r6, #10
	mov	r2, r1
	mov	r7, r0
	ldr	r0, [r5, #744]
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
	ldr	r2, [r5, #748]
	ldrh	r3, [r2, r7, lsl #1]
	adds	r3, r3, #1
	strh	r3, [r2, r7, lsl #1]	@ movhi
	ldr	r3, [r5, #752]
	adds	r3, r3, #1
	str	r3, [r5, #752]
.L586:
	ldrh	r3, [r4, #3034]
	adds	r3, r3, #1
	strh	r3, [r4, #3034]	@ movhi
	ldrh	r3, [r4, #3032]
	add	r2, r4, r3, lsl #1
	adds	r3, r3, #1
	ubfx	r3, r3, #0, #10
	strh	r3, [r4, #3032]	@ movhi
	strh	r6, [r2, #3036]	@ movhi
.L584:
	pop	{r3, r4, r5, r6, r7, pc}
.L594:
	.align	2
.L593:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlFreeSysBlkQueueIn, .-FtlFreeSysBlkQueueIn
	.align	1
	.global	FtlFreeSysBLkSort
	.thumb
	.thumb_func
	.type	FtlFreeSysBLkSort, %function
FtlFreeSysBLkSort:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L606
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldrh	r4, [r3, #784]
	ldr	r3, .L606+4
	ldrh	r2, [r3, #3034]
	cbz	r2, .L595
	movs	r0, #0
	ldrh	r1, [r3, #3030]
	ldrh	r2, [r3, #3032]
	and	r4, r4, #31
	mov	r6, r0
.L597:
	uxth	r5, r0
	adds	r0, r0, #1
	cmp	r5, r4
	bge	.L605
	add	r5, r3, r1, lsl #1
	adds	r1, r1, #1
	ubfx	r1, r1, #0, #10
	ldrh	r6, [r5, #3036]
	add	r5, r3, r2, lsl #1
	strh	r6, [r5, #3036]	@ movhi
	movs	r6, #1
	add	r2, r2, r6
	ubfx	r2, r2, #0, #10
	b	.L597
.L605:
	cbz	r6, .L595
	strh	r1, [r3, #3030]	@ movhi
	strh	r2, [r3, #3032]	@ movhi
.L595:
	pop	{r4, r5, r6, pc}
.L607:
	.align	2
.L606:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBLkSort, .-FtlFreeSysBLkSort
	.align	1
	.global	FtlFreeSysBlkQueueOut
	.thumb
	.thumb_func
	.type	FtlFreeSysBlkQueueOut, %function
FtlFreeSysBlkQueueOut:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	ldr	r4, .L618
	ldr	r6, .L618+4
	mov	r7, r4
.L609:
	ldrh	r1, [r4, #3034]
	cbz	r1, .L610
	ldrh	r3, [r4, #3030]
	subs	r1, r1, #1
	ldr	r10, [r6, #740]
	strh	r1, [r4, #3034]	@ movhi
	add	r2, r4, r3, lsl #1
	adds	r3, r3, #1
	ubfx	r3, r3, #0, #10
	ldr	r8, .L618+4
	ldrh	r5, [r2, #3036]
	strh	r3, [r4, #3030]	@ movhi
	cmp	r10, #0
	bne	.L611
	mov	r0, r5
	bl	P2V_block_in_plane
	lsls	r3, r5, #10
	mov	r9, r0
	ldr	r0, [r6, #744]
	str	r3, [r0, #4]
	ldrb	r3, [r4, #1888]	@ zero_extendqisi2
	cbz	r3, .L612
	mov	r1, r10
	movs	r2, #1
	bl	FlashEraseBlocks
.L612:
	movs	r1, #1
	ldr	r0, [r8, #744]
	mov	r2, r1
	bl	FlashEraseBlocks
	ldr	r2, [r8, #748]
	ldrh	r3, [r2, r9, lsl #1]
	adds	r3, r3, #1
	strh	r3, [r2, r9, lsl #1]	@ movhi
	ldr	r3, [r8, #752]
	adds	r3, r3, #1
	str	r3, [r8, #752]
	b	.L611
.L610:
	ldr	r0, .L618+8
	bl	printk
.L613:
	b	.L613
.L611:
	subs	r3, r5, #1
	movw	r2, #65533
	uxth	r3, r3
	cmp	r3, r2
	bls	.L614
	mov	r1, r5
	ldrh	r2, [r7, #3034]
	ldr	r0, .L618+12
	bl	printk
	b	.L609
.L614:
	mov	r0, r5
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L619:
	.align	2
.L618:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC3
	.word	.LC4
	.fnend
	.size	FtlFreeSysBlkQueueOut, .-FtlFreeSysBlkQueueOut
	.align	1
	.global	test_node_in_list
	.thumb
	.thumb_func
	.type	test_node_in_list, %function
test_node_in_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L625
	push	{r4, r5, lr}
	.save {r4, r5, lr}
	movw	r5, #65535
	ldr	r4, [r3, #804]
	ldr	r2, [r0]
	ldr	r3, .L625+4
	subs	r0, r2, r4
	asrs	r0, r0, #1
	muls	r3, r0, r3
	movs	r0, #6
	uxth	r3, r3
.L622:
	cmp	r1, r3
	beq	.L623
	ldrh	r3, [r2]
	cmp	r3, r5
	beq	.L624
	mla	r2, r0, r3, r4
	b	.L622
.L623:
	movs	r0, #1
	pop	{r4, r5, pc}
.L624:
	movs	r0, #0
	pop	{r4, r5, pc}
.L626:
	.align	2
.L625:
	.word	.LANCHOR2
	.word	-1431655765
	.fnend
	.size	test_node_in_list, .-test_node_in_list
	.align	1
	.global	insert_data_list
	.thumb
	.thumb_func
	.type	insert_data_list, %function
insert_data_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r5, .L643
	ldrh	r3, [r5, #2848]
	cmp	r3, r0
	bls	.L629
	movs	r4, #6
	ldr	r2, .L643+4
	movw	r3, #65535
	muls	r4, r0, r4
	ldr	lr, [r2, #804]
	mov	r7, r2
	add	r1, lr, r4
	strh	r3, [r1, #2]	@ movhi
	strh	r3, [lr, r4]	@ movhi
	ldr	r3, [r2, #808]
	cmp	r3, #0
	beq	.L642
	ldr	r8, [r2, #812]
	lsl	fp, r0, #1
	ldrh	r2, [r1, #4]
	ldrh	r6, [r8, r0, lsl #1]
	cbz	r2, .L640
	muls	r6, r2, r6
	b	.L631
.L640:
	mov	r6, #-1
.L631:
	ldr	ip, [r7, #804]
	ldr	r2, .L643+8
	rsb	r9, ip, r3
	ldrh	r5, [r5, #2848]
	asr	r9, r9, #1
	str	r5, [sp]
	mul	r2, r2, r9
	ldr	r9, [r7, #748]
	add	r5, r9, fp
	str	r5, [sp, #4]
	movs	r5, #0
	uxth	r2, r2
.L638:
	adds	r5, r5, #1
	ldr	r7, [sp]
	uxth	r5, r5
	cmp	r5, r7
	bhi	.L629
	cmp	r0, r2
	beq	.L629
	ldrh	r7, [r3, #4]
	lsl	r10, r2, #1
	ldrh	fp, [r8, r2, lsl #1]
	cbz	r7, .L641
	mul	r7, r7, fp
	b	.L633
.L641:
	mov	r7, #-1
.L633:
	cmp	r7, r6
	bne	.L634
	ldr	r7, [sp, #4]
	ldrh	r10, [r9, r10]
	ldrh	r7, [r7]
	cmp	r10, r7
	bcc	.L636
	b	.L635
.L634:
	bhi	.L635
.L636:
	ldrh	r7, [r3]
	movw	r10, #65535
	cmp	r7, r10
	bne	.L637
	strh	r2, [r1, #2]	@ movhi
	strh	r0, [r3]	@ movhi
	ldr	r3, .L643+4
	str	r1, [r3, #816]
	b	.L629
.L637:
	movs	r3, #6
	mov	r2, r7
	mla	r3, r3, r7, ip
	b	.L638
.L635:
	strh	r2, [lr, r4]	@ movhi
	ldrh	r2, [r3, #2]
	strh	r2, [r1, #2]	@ movhi
	ldr	r2, .L643+4
	ldr	r4, [r2, #808]
	cmp	r3, r4
	bne	.L639
	strh	r0, [r3, #2]	@ movhi
.L642:
	str	r1, [r2, #808]
	b	.L629
.L639:
	ldrh	r4, [r3, #2]
	ldr	r1, [r2, #804]
	movs	r2, #6
	muls	r2, r4, r2
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L629:
	movs	r0, #0
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L644:
	.align	2
.L643:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1431655765
	.fnend
	.size	insert_data_list, .-insert_data_list
	.align	1
	.global	INSERT_DATA_LIST
	.thumb
	.thumb_func
	.type	INSERT_DATA_LIST, %function
INSERT_DATA_LIST:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	.save {r3, lr}
	bl	insert_data_list
	ldr	r2, .L646
	ldrh	r3, [r2, #820]
	adds	r3, r3, #1
	strh	r3, [r2, #820]	@ movhi
	pop	{r3, pc}
.L647:
	.align	2
.L646:
	.word	.LANCHOR2
	.fnend
	.size	INSERT_DATA_LIST, .-INSERT_DATA_LIST
	.align	1
	.global	insert_free_list
	.thumb
	.thumb_func
	.type	insert_free_list, %function
insert_free_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	movw	r5, #65535
	cmp	r0, r5
	beq	.L649
	movs	r1, #6
	ldr	r2, .L656
	mul	lr, r1, r0
	ldr	ip, [r2, #804]
	mov	r7, r2
	add	r6, ip, lr
	strh	r5, [r6, #2]	@ movhi
	strh	r5, [ip, lr]	@ movhi
	ldr	r3, [r2, #824]
	cbz	r3, .L655
	ldr	r8, [r2, #804]
	ldr	r9, [r2, #748]
	rsb	r4, r8, r3
	ldr	r2, .L656+4
	asrs	r4, r4, #1
	ldrh	r10, [r9, r0, lsl #1]
	muls	r4, r2, r4
	uxth	r2, r4
.L653:
	ldrh	r4, [r9, r2, lsl #1]
	cmp	r4, r10
	bcs	.L651
	ldrh	r4, [r3]
	cmp	r4, r5
	bne	.L652
	strh	r2, [r6, #2]	@ movhi
	strh	r0, [r3]	@ movhi
	b	.L649
.L652:
	mla	r3, r1, r4, r8
	mov	r2, r4
	b	.L653
.L651:
	ldrh	r1, [r3, #2]
	strh	r1, [r6, #2]	@ movhi
	strh	r2, [ip, lr]	@ movhi
	ldr	r1, [r7, #824]
	ldr	r2, .L656
	cmp	r3, r1
	bne	.L654
	strh	r0, [r3, #2]	@ movhi
.L655:
	str	r6, [r2, #824]
	b	.L649
.L654:
	ldrh	r4, [r3, #2]
	ldr	r1, [r2, #804]
	movs	r2, #6
	muls	r2, r4, r2
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L649:
	movs	r0, #0
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L657:
	.align	2
.L656:
	.word	.LANCHOR2
	.word	-1431655765
	.fnend
	.size	insert_free_list, .-insert_free_list
	.align	1
	.global	INSERT_FREE_LIST
	.thumb
	.thumb_func
	.type	INSERT_FREE_LIST, %function
INSERT_FREE_LIST:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	.save {r3, lr}
	bl	insert_free_list
	ldr	r2, .L659
	ldrh	r3, [r2, #828]
	adds	r3, r3, #1
	strh	r3, [r2, #828]	@ movhi
	pop	{r3, pc}
.L660:
	.align	2
.L659:
	.word	.LANCHOR2
	.fnend
	.size	INSERT_FREE_LIST, .-INSERT_FREE_LIST
	.align	1
	.global	List_remove_node
	.thumb
	.thumb_func
	.type	List_remove_node, %function
List_remove_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	movs	r4, #6
	ldr	r7, .L666
	movw	r6, #65535
	muls	r1, r4, r1
	ldr	r3, [r0]
	ldr	r2, [r7, #804]
	adds	r5, r2, r1
	cmp	r5, r3
	ldrh	r3, [r2, r1]
	bne	.L662
	cmp	r3, r6
	ittee	eq
	moveq	r3, #0
	streq	r3, [r0]
	mlane	r3, r4, r3, r2
	strne	r3, [r0]
	it	ne
	strhne	r6, [r3, #2]	@ movhi
	b	.L664
.L662:
	cmp	r3, r6
	ldrh	r0, [r5, #2]
	bne	.L665
	cmp	r0, r3
	beq	.L664
	muls	r0, r4, r0
	strh	r3, [r2, r0]	@ movhi
	b	.L664
.L665:
	mla	r3, r4, r3, r2
	strh	r0, [r3, #2]	@ movhi
	ldrh	r6, [r5, #2]
	ldrh	r0, [r2, r1]
	ldr	r3, [r7, #804]
	muls	r4, r6, r4
	strh	r0, [r3, r4]	@ movhi
.L664:
	movw	r3, #65535
	movs	r0, #0
	strh	r3, [r2, r1]	@ movhi
	strh	r3, [r5, #2]	@ movhi
	pop	{r4, r5, r6, r7, pc}
.L667:
	.align	2
.L666:
	.word	.LANCHOR2
	.fnend
	.size	List_remove_node, .-List_remove_node
	.align	1
	.global	List_pop_index_node
	.thumb
	.thumb_func
	.type	List_pop_index_node, %function
List_pop_index_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldr	r3, [r0]
	cbz	r3, .L674
	ldr	r2, .L675
	movs	r6, #6
	movw	r5, #65535
	ldr	r4, [r2, #804]
.L670:
	cbnz	r1, .L671
.L673:
	subs	r4, r3, r4
	ldr	r3, .L675+4
	asrs	r4, r4, #1
	muls	r4, r3, r4
	uxth	r4, r4
	mov	r1, r4
	bl	List_remove_node
	mov	r0, r4
	pop	{r4, r5, r6, pc}
.L671:
	ldrh	r2, [r3]
	cmp	r2, r5
	beq	.L673
	subs	r1, r1, #1
	mla	r3, r6, r2, r4
	uxth	r1, r1
	b	.L670
.L674:
	movw	r0, #65535
	pop	{r4, r5, r6, pc}
.L676:
	.align	2
.L675:
	.word	.LANCHOR2
	.word	-1431655765
	.fnend
	.size	List_pop_index_node, .-List_pop_index_node
	.align	1
	.global	List_get_gc_head_node
	.thumb
	.thumb_func
	.type	List_get_gc_head_node, %function
List_get_gc_head_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L684
	push	{r4, lr}
	.save {r4, lr}
	ldr	r3, [r2, #808]
	cbz	r3, .L682
	movs	r4, #6
	ldr	r1, [r2, #804]
	movw	r2, #65535
.L679:
	cbz	r0, .L680
	ldrh	r3, [r3]
	cmp	r3, r2
	beq	.L682
	subs	r0, r0, #1
	mla	r3, r4, r3, r1
	uxth	r0, r0
	b	.L679
.L682:
	movw	r0, #65535
	pop	{r4, pc}
.L680:
	subs	r3, r3, r1
	ldr	r0, .L684+4
	asrs	r3, r3, #1
	muls	r0, r3, r0
	uxth	r0, r0
	pop	{r4, pc}
.L685:
	.align	2
.L684:
	.word	.LANCHOR2
	.word	-1431655765
	.fnend
	.size	List_get_gc_head_node, .-List_get_gc_head_node
	.align	1
	.global	List_update_data_list
	.thumb
	.thumb_func
	.type	List_update_data_list, %function
List_update_data_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r5, r0
	ldr	r4, .L693
	ldrh	r3, [r4, #832]
	cmp	r3, r0
	beq	.L687
	ldrh	r3, [r4, #880]
	cmp	r3, r0
	beq	.L687
	ldrh	r3, [r4, #928]
	cmp	r3, r0
	beq	.L687
	movs	r3, #6
	ldr	r1, [r4, #804]
	ldr	r2, [r4, #808]
	muls	r3, r0, r3
	adds	r0, r1, r3
	cmp	r0, r2
	beq	.L687
	ldr	r6, [r4, #812]
	ldrh	r2, [r0, #4]
	ldrh	r7, [r6, r5, lsl #1]
	cbz	r2, .L691
	muls	r2, r7, r2
	b	.L688
.L691:
	mov	r2, #-1
.L688:
	ldrh	r0, [r0, #2]
	movw	r7, #65535
	cmp	r0, r7
	bne	.L689
	ldrh	r3, [r1, r3]
	cmp	r3, r0
	beq	.L687
.L689:
	movs	r3, #6
	muls	r3, r0, r3
	ldr	r0, .L693+4
	asrs	r7, r3, #1
	add	r3, r3, r1
	muls	r0, r7, r0
	ldrh	r3, [r3, #4]
	ldrh	r0, [r6, r0, lsl #1]
	cbz	r3, .L692
	muls	r3, r0, r3
	b	.L690
.L692:
	mov	r3, #-1
.L690:
	cmp	r2, r3
	bcs	.L687
	ldr	r0, .L693+8
	mov	r1, r5
	bl	List_remove_node
	ldrh	r3, [r4, #820]
	mov	r0, r5
	subs	r3, r3, #1
	strh	r3, [r4, #820]	@ movhi
	bl	INSERT_DATA_LIST
.L687:
	movs	r0, #0
	pop	{r3, r4, r5, r6, r7, pc}
.L694:
	.align	2
.L693:
	.word	.LANCHOR2
	.word	-1431655765
	.word	.LANCHOR2+808
	.fnend
	.size	List_update_data_list, .-List_update_data_list
	.align	1
	.global	ftl_map_blk_alloc_new_blk
	.thumb
	.thumb_func
	.type	ftl_map_blk_alloc_new_blk, %function
ftl_map_blk_alloc_new_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	movs	r3, #0
	ldrh	r1, [r0, #10]
	mov	r4, r0
	ldr	r2, [r0, #12]
.L696:
	uxth	r5, r3
	cmp	r5, r1
	bcs	.L699
	mov	r7, r2
	adds	r3, r3, #1
	ldrh	r6, [r7]
	adds	r2, r2, #2
	cmp	r6, #0
	bne	.L696
	bl	FtlFreeSysBlkQueueOut
	movw	r2, #65533
	subs	r3, r0, #1
	mov	r1, r0
	strh	r0, [r7]	@ movhi
	uxth	r3, r3
	cmp	r3, r2
	bls	.L697
	ldr	r3, .L702
	ldr	r0, .L702+4
	ldrh	r2, [r3, #3034]
	bl	printk
.L698:
	b	.L698
.L697:
	ldr	r3, [r4, #28]
	strh	r6, [r4, #2]	@ movhi
	adds	r3, r3, #1
	str	r3, [r4, #28]
	ldrh	r3, [r4, #8]
	strh	r5, [r4]	@ movhi
	adds	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
.L699:
	movs	r0, #0
	pop	{r3, r4, r5, r6, r7, pc}
.L703:
	.align	2
.L702:
	.word	.LANCHOR0
	.word	.LC5
	.fnend
	.size	ftl_map_blk_alloc_new_blk, .-ftl_map_blk_alloc_new_blk
	.align	1
	.global	select_l2p_ram_region
	.thumb
	.thumb_func
	.type	select_l2p_ram_region, %function
select_l2p_ram_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	movs	r1, #0
	ldr	r3, .L718
	movs	r4, #12
	ldr	r7, .L718+4
	movw	r5, #65535
	ldrh	r2, [r3, #2946]
	ldr	r3, [r7, #976]
.L705:
	uxth	r0, r1
	cmp	r0, r2
	bcs	.L716
	adds	r1, r1, #1
	mla	r6, r4, r1, r3
	ldrh	r6, [r6, #-12]
	cmp	r6, r5
	bne	.L705
	b	.L706
.L716:
	movs	r1, #0
	mov	r6, #-2147483648
	mov	lr, #12
	mov	r0, r2
.L708:
	uxth	r5, r1
	cmp	r5, r2
	bcs	.L717
	mla	r4, lr, r1, r3
	ldr	r4, [r4, #4]
	cmp	r4, #0
	blt	.L709
	cmp	r4, r6
	itt	cc
	movcc	r6, r4
	movcc	r0, r5
.L709:
	adds	r1, r1, #1
	b	.L708
.L717:
	cmp	r0, r2
	bcc	.L706
	ldrh	r7, [r7, #980]
	movs	r1, #0
	mov	r4, #-1
	mov	r0, r2
.L711:
	uxth	r5, r1
	cmp	r5, r2
	bcs	.L706
	ldr	r6, [r3, #4]
	cmp	r6, r4
	bcs	.L712
	ldrh	lr, [r3]
	cmp	lr, r7
	itt	ne
	movne	r4, r6
	movne	r0, r5
.L712:
	adds	r1, r1, #1
	adds	r3, r3, #12
	b	.L711
.L706:
	pop	{r4, r5, r6, r7, pc}
.L719:
	.align	2
.L718:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	select_l2p_ram_region, .-select_l2p_ram_region
	.align	1
	.global	FtlUpdateVaildLpn
	.thumb
	.thumb_func
	.type	FtlUpdateVaildLpn, %function
FtlUpdateVaildLpn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r1, .L727
	push	{r4, r5, lr}
	.save {r4, r5, lr}
	ldrh	r2, [r1, #982]
	mov	r3, r1
	cmp	r2, #4
	bhi	.L721
	cbnz	r0, .L721
	adds	r2, r2, #1
	strh	r2, [r1, #982]	@ movhi
	pop	{r4, r5, pc}
.L721:
	ldr	r1, .L727+4
	movs	r2, #0
	strh	r2, [r3, #982]	@ movhi
	movw	r5, #65535
	str	r2, [r3, #984]
	ldrh	r1, [r1, #2848]
	ldr	r2, [r3, #812]
	add	r1, r2, r1, lsl #1
.L722:
	cmp	r2, r1
	beq	.L726
	ldrh	r4, [r2], #2
	cmp	r4, r5
	ittt	ne
	ldrne	r0, [r3, #984]
	addne	r0, r0, r4
	strne	r0, [r3, #984]
	b	.L722
.L726:
	pop	{r4, r5, pc}
.L728:
	.align	2
.L727:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlUpdateVaildLpn, .-FtlUpdateVaildLpn
	.align	1
	.global	ftl_set_blk_mode
	.thumb
	.thumb_func
	.type	ftl_set_blk_mode, %function
ftl_set_blk_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	lsrs	r3, r0, #5
	ldr	r2, .L732
	and	r0, r0, #31
	uxth	r3, r3
	cbz	r1, .L730
	ldr	r1, [r2, #988]
	movs	r2, #1
	lsls	r2, r2, r0
	ldr	r0, [r1, r3, lsl #2]
	orrs	r2, r2, r0
	str	r2, [r1, r3, lsl #2]
	bx	lr
.L730:
	ldr	r1, [r2, #988]
	movs	r2, #1
	lsls	r2, r2, r0
	ldr	r0, [r1, r3, lsl #2]
	bic	r0, r0, r2
	str	r0, [r1, r3, lsl #2]
	bx	lr
.L733:
	.align	2
.L732:
	.word	.LANCHOR2
	.fnend
	.size	ftl_set_blk_mode, .-ftl_set_blk_mode
	.align	1
	.global	ftl_get_blk_mode
	.thumb
	.thumb_func
	.type	ftl_get_blk_mode, %function
ftl_get_blk_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L735
	lsrs	r1, r0, #5
	ldr	r2, [r3, #988]
	and	r3, r0, #31
	ldr	r0, [r2, r1, lsl #2]
	lsrs	r0, r0, r3
	and	r0, r0, #1
	bx	lr
.L736:
	.align	2
.L735:
	.word	.LANCHOR2
	.fnend
	.size	ftl_get_blk_mode, .-ftl_get_blk_mode
	.align	1
	.global	ftl_sb_update_avl_pages
	.thumb
	.thumb_func
	.type	ftl_sb_update_avl_pages, %function
ftl_sb_update_avl_pages:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	movs	r3, #0
	strh	r3, [r0, #4]	@ movhi
	ldr	r3, .L746
	push	{r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	adds	r4, r2, #7
	ldrh	r5, [r3, #2840]
	movw	r7, #65535
	add	r4, r0, r4, lsl #1
.L738:
	cmp	r2, r5
	bcs	.L744
	ldrh	r6, [r4, #2]!
	adds	r2, r2, #1
	cmp	r6, r7
	uxth	r2, r2
	ittt	ne
	ldrhne	r6, [r0, #4]
	addne	r6, r6, #1
	strhne	r6, [r0, #4]	@ movhi
	b	.L738
.L744:
	ldrh	r6, [r3, #2908]
	add	r4, r0, #14
	movs	r2, #0
	movw	r7, #65535
.L741:
	uxth	r3, r2
	cmp	r3, r5
	bcs	.L745
	ldrh	r3, [r4, #2]!
	adds	r2, r2, #1
	cmp	r3, r7
	itttt	ne
	ldrhne	r3, [r0, #4]
	addne	r3, r3, r6
	addne	r3, r3, #-1
	subne	r3, r3, r1
	it	ne
	strhne	r3, [r0, #4]	@ movhi
	b	.L741
.L745:
	pop	{r4, r5, r6, r7, pc}
.L747:
	.align	2
.L746:
	.word	.LANCHOR0
	.fnend
	.size	ftl_sb_update_avl_pages, .-ftl_sb_update_avl_pages
	.align	1
	.global	make_superblock
	.thumb
	.thumb_func
	.type	make_superblock, %function
make_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movs	r3, #0
	ldr	r6, .L761
	add	r7, r0, #16
	mov	r4, r0
	mov	r5, r3
	addw	r10, r6, #2868
	movw	r9, #65535
	ldrh	r8, [r6, #2840]
	strh	r3, [r0, #4]	@ movhi
	strb	r3, [r0, #7]
.L749:
	uxth	r3, r5
	cmp	r3, r8
	bcs	.L760
	ldrb	r0, [r10, r5]	@ zero_extendqisi2
	ldrh	r1, [r4]
	bl	V2P_block
	strh	r9, [r7]	@ movhi
	mov	fp, r0
	bl	FtlBbmIsBadBlock
	cbnz	r0, .L750
	strh	fp, [r7]	@ movhi
	ldrb	r3, [r4, #7]	@ zero_extendqisi2
	adds	r3, r3, #1
	strb	r3, [r4, #7]
.L750:
	adds	r5, r5, #1
	adds	r7, r7, #2
	b	.L749
.L760:
	ldrb	r2, [r4, #7]	@ zero_extendqisi2
	ldrh	r3, [r6, #2908]
	smulbb	r3, r2, r3
	strh	r3, [r4, #4]	@ movhi
	movs	r3, #0
	strb	r3, [r4, #9]
	ldr	r3, [r6, #2832]
	cbz	r3, .L752
	ldr	r3, .L761+4
	ldrh	r2, [r4]
	ldr	r3, [r3, #748]
	ldrh	r3, [r3, r2, lsl #1]
	cmp	r3, #79
	itt	ls
	movls	r3, #1
	strbls	r3, [r4, #9]
.L752:
	ldrb	r3, [r6]	@ zero_extendqisi2
	cbz	r3, .L753
	movs	r3, #1
	strb	r3, [r4, #9]
.L753:
	movs	r0, #0
	pop	{r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L762:
	.align	2
.L761:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	make_superblock, .-make_superblock
	.align	1
	.global	update_multiplier_value
	.thumb
	.thumb_func
	.type	update_multiplier_value, %function
update_multiplier_value:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	movs	r5, #0
	ldr	r3, .L772
	mov	r7, r0
	mov	r4, r5
	addw	r6, r3, #2868
	ldrh	r8, [r3, #2840]
	ldrh	r9, [r3, #2908]
.L764:
	uxth	r3, r5
	cmp	r3, r8
	bcs	.L771
	mov	r1, r7
	ldrb	r0, [r6, r5]	@ zero_extendqisi2
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	cbnz	r0, .L765
	add	r4, r4, r9
	uxth	r4, r4
.L765:
	adds	r5, r5, #1
	b	.L764
.L771:
	cbz	r4, .L767
	mov	r1, r4
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r4, r0
.L767:
	ldr	r3, .L772+4
	movs	r2, #6
	movs	r0, #0
	ldr	r3, [r3, #804]
	mla	r7, r2, r7, r3
	strh	r4, [r7, #4]	@ movhi
	pop	{r3, r4, r5, r6, r7, r8, r9, pc}
.L773:
	.align	2
.L772:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	update_multiplier_value, .-update_multiplier_value
	.align	1
	.global	GetFreeBlockMinEraseCount
	.thumb
	.thumb_func
	.type	GetFreeBlockMinEraseCount, %function
GetFreeBlockMinEraseCount:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L777
	ldr	r0, [r2, #824]
	cbz	r0, .L775
	ldr	r3, [r2, #804]
	subs	r0, r0, r3
	ldr	r3, .L777+4
	asrs	r0, r0, #1
	muls	r0, r3, r0
	ldr	r3, [r2, #748]
	uxth	r0, r0
	ldrh	r0, [r3, r0, lsl #1]
.L775:
	bx	lr
.L778:
	.align	2
.L777:
	.word	.LANCHOR2
	.word	-1431655765
	.fnend
	.size	GetFreeBlockMinEraseCount, .-GetFreeBlockMinEraseCount
	.align	1
	.global	GetFreeBlockMaxEraseCount
	.thumb
	.thumb_func
	.type	GetFreeBlockMaxEraseCount, %function
GetFreeBlockMaxEraseCount:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r1, .L787
	push	{r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	ldr	r3, [r1, #824]
	cbz	r3, .L785
	ldrh	r2, [r1, #828]
	movs	r6, #6
	ldr	r4, [r1, #804]
	movw	r7, #65535
	rsb	r2, r2, r2, lsl #3
	subs	r3, r3, r4
	asrs	r2, r2, #3
	asrs	r3, r3, #1
	cmp	r0, r2
	it	gt
	uxthgt	r0, r2
	ldr	r2, .L787+4
	muls	r3, r2, r3
	movs	r2, #0
	uxth	r3, r3
.L782:
	uxth	r5, r2
	cmp	r5, r0
	bcs	.L784
	mul	r5, r6, r3
	adds	r2, r2, #1
	ldrh	r5, [r4, r5]
	cmp	r5, r7
	bne	.L786
.L784:
	ldr	r2, [r1, #748]
	ldrh	r0, [r2, r3, lsl #1]
	pop	{r4, r5, r6, r7, pc}
.L786:
	mov	r3, r5
	b	.L782
.L785:
	mov	r0, r3
	pop	{r4, r5, r6, r7, pc}
.L788:
	.align	2
.L787:
	.word	.LANCHOR2
	.word	-1431655765
	.fnend
	.size	GetFreeBlockMaxEraseCount, .-GetFreeBlockMaxEraseCount
	.align	1
	.global	FtlPrintInfo2buf
	.thumb
	.thumb_func
	.type	FtlPrintInfo2buf, %function
FtlPrintInfo2buf:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	r6, r0
	ldr	r8, .L799+240
	add	r4, r6, #12
	ldr	r1, .L799
	.pad #32
	sub	sp, sp, #32
	bl	strcpy
	mov	r0, r4
	ldr	r1, .L799+4
	ldr	r2, [r8, #1860]
	bl	sprintf
	ldr	r1, .L799+8
	ldr	r2, [r8, #2924]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L799+12
	ldr	r3, [r3, #504]
	cmp	r3, #1
	add	r4, r4, r0
	bne	.L795
	add	r0, sp, #16
	add	r1, sp, #20
	add	r2, sp, #24
	add	r3, sp, #28
	bl	NandcGetTimeCfg
	mov	r0, r4
	ldr	r1, .L799+16
	ldr	r7, .L799+20
	ldr	r3, [sp, #24]
	ldr	r2, [sp, #16]
	str	r3, [sp]
	ldr	r3, [sp, #28]
	str	r3, [sp, #4]
	ldr	r3, [sp, #20]
	bl	sprintf
	ldr	r1, .L799+24
	add	r4, r4, r0
	add	r5, r4, #10
	mov	r0, r4
	bl	strcpy
	mov	r0, r5
	ldr	r1, .L799+28
	ldr	r2, [r8, #2964]
	bl	sprintf
	ldr	r1, .L799+32
	ldr	r2, [r7, #984]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+36
	ldr	r2, [r7, #992]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+40
	ldr	r2, [r7, #996]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+44
	ldr	r2, [r7, #1000]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+48
	ldr	r2, [r7, #1004]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+52
	ldr	r2, [r7, #1008]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+56
	ldr	r2, [r7, #1012]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r7, #1016]
	ldr	r1, .L799+60
	lsrs	r2, r2, #11
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r7, #1020]
	ldr	r1, .L799+64
	lsrs	r2, r2, #11
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+68
	ldr	r2, [r7, #1024]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+72
	ldr	r2, [r7, #1028]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	add	r5, r5, r0
	bl	FtlBbtCalcTotleCnt
	ldr	r1, .L799+76
	ldrh	r2, [r8, #2974]
	mov	r3, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+80
	ldrh	r2, [r7, #828]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+84
	ldr	r2, [r7, #1032]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+88
	ldr	r2, [r7, #1036]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+92
	ldr	r2, [r7, #1040]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+96
	ldr	r2, [r7, #752]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+100
	ldr	r2, [r7, #1044]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+104
	ldr	r2, [r7, #1048]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+108
	ldrh	r2, [r7, #786]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+112
	ldrh	r2, [r7, #784]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+116
	ldr	r2, [r8, #1892]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+120
	ldr	r2, [r8, #2940]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+124
	ldr	r2, [r8, #2836]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+128
	ldrh	r2, [r8, #3034]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+132
	ldrh	r2, [r8, #2848]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+136
	ldrh	r2, [r7, #1052]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+140
	ldr	r2, [r8, #2852]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+144
	ldrh	r2, [r7, #1056]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+148
	ldrh	r2, [r8, #2968]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+152
	ldrh	r2, [r7, #834]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+156
	ldrb	r2, [r7, #838]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+160
	ldrh	r2, [r7, #832]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+164
	ldrb	r2, [r7, #840]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+168
	ldrh	r2, [r7, #836]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, [r7, #812]
	ldrh	r2, [r7, #832]
	ldr	r1, .L799+172
	ldrh	r2, [r3, r2, lsl #1]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+176
	ldrh	r2, [r7, #882]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+180
	ldrb	r2, [r7, #886]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+184
	ldrh	r2, [r7, #880]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+188
	ldrb	r2, [r7, #888]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+192
	ldrh	r2, [r7, #884]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, [r7, #812]
	ldrh	r2, [r7, #880]
	ldr	r1, .L799+196
	ldrh	r2, [r3, r2, lsl #1]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+200
	ldrh	r2, [r7, #930]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+204
	ldrb	r2, [r7, #934]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+208
	ldrh	r2, [r7, #928]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+212
	ldrb	r2, [r7, #936]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+216
	ldrh	r2, [r7, #932]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+220
	ldrh	r2, [r7, #1074]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+224
	ldrb	r2, [r7, #1078]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+228
	ldrh	r2, [r7, #1072]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+232
	ldrb	r2, [r7, #1080]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L799+236
	ldrh	r2, [r7, #1076]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, [r7, #1208]
	ldr	r3, [r8, #2832]
	ldr	r2, [r7, #1120]
	b	.L800
.L801:
	.align	2
.L799:
	.word	.LC6
	.word	.LC7
	.word	.LC8
	.word	.LANCHOR1
	.word	.LC9
	.word	.LANCHOR2
	.word	.LC10
	.word	.LC11
	.word	.LC12
	.word	.LC13
	.word	.LC14
	.word	.LC15
	.word	.LC16
	.word	.LC17
	.word	.LC18
	.word	.LC19
	.word	.LC20
	.word	.LC21
	.word	.LC22
	.word	.LC23
	.word	.LC24
	.word	.LC25
	.word	.LC26
	.word	.LC27
	.word	.LC28
	.word	.LC29
	.word	.LC30
	.word	.LC31
	.word	.LC32
	.word	.LC33
	.word	.LC34
	.word	.LC35
	.word	.LC36
	.word	.LC37
	.word	.LC38
	.word	.LC39
	.word	.LC40
	.word	.LC41
	.word	.LC42
	.word	.LC43
	.word	.LC44
	.word	.LC45
	.word	.LC46
	.word	.LC47
	.word	.LC48
	.word	.LC49
	.word	.LC50
	.word	.LC51
	.word	.LC52
	.word	.LC53
	.word	.LC54
	.word	.LC55
	.word	.LC56
	.word	.LC57
	.word	.LC58
	.word	.LC59
	.word	.LC60
	.word	.LC61
	.word	.LC62
	.word	.LC63
	.word	.LANCHOR0
.L800:
	orr	r2, r3, r2, lsl #8
	str	r1, [sp]
	add	r5, r5, r0
	ldr	r1, [r7, #1200]
	mov	r0, r5
	str	r1, [sp, #4]
	ldr	r3, [r7, #1204]
	ldr	r1, .L802
	bl	sprintf
	ldr	r1, .L802+4
	ldr	r2, [r7, #1196]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L802+8
	ldr	r2, [r7, #1220]
	adds	r4, r5, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L802+12
	ldrh	r2, [r7, #1636]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L802+16
	ldrh	r2, [r7, #1638]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L802+20
	ldr	r2, [r7, #1640]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L802+24
	ldrh	r2, [r7, #1644]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	add	r4, r4, r0
	bl	GetFreeBlockMinEraseCount
	ldr	r1, .L802+28
	mov	r2, r0
	mov	r0, r4
	bl	sprintf
	add	r4, r4, r0
	ldrh	r0, [r7, #828]
	bl	GetFreeBlockMaxEraseCount
	ldr	r1, .L802+32
	mov	r2, r0
	mov	r0, r4
	bl	sprintf
	ldrh	r3, [r7, #1072]
	movw	r2, #65535
	cmp	r3, r2
	add	r4, r4, r0
	beq	.L792
	ldr	r2, [r7, #812]
	mov	r0, r4
	ldr	r1, .L802+36
	ldrh	r2, [r2, r3, lsl #1]
	bl	sprintf
	add	r4, r4, r0
.L792:
	movs	r0, #0
	movs	r5, #0
	bl	List_get_gc_head_node
	mov	r9, #6
	movw	r10, #65535
	uxth	r3, r0
.L794:
	cmp	r3, r10
	beq	.L793
	ldr	r2, [r7, #812]
	mov	r0, r4
	mul	r8, r9, r3
	ldr	r1, .L802+40
	ldrh	r2, [r2, r3, lsl #1]
	str	r2, [sp]
	ldr	r2, [r7, #804]
	add	r2, r2, r8
	ldrh	r2, [r2, #4]
	str	r2, [sp, #4]
	ldr	r2, [r7, #748]
	ldrh	r2, [r2, r3, lsl #1]
	str	r2, [sp, #8]
	mov	r2, r5
	bl	sprintf
	adds	r5, r5, #1
	ldr	r3, [r7, #804]
	cmp	r5, #16
	ldrh	r3, [r3, r8]
	add	r4, r4, r0
	bne	.L794
.L793:
	ldr	r2, [r7, #804]
	movs	r5, #0
	ldr	r3, [r7, #824]
	mov	r9, #6
	movw	r10, #65535
	subs	r3, r3, r2
	ldr	r2, .L802+44
	asrs	r3, r3, #1
	muls	r3, r2, r3
	uxth	r3, r3
.L796:
	cmp	r3, r10
	beq	.L795
	mul	r8, r9, r3
	ldr	r2, [r7, #804]
	mov	r0, r4
	ldr	r1, .L802+48
	add	r2, r2, r8
	ldrh	r2, [r2, #4]
	str	r2, [sp]
	ldr	r2, [r7, #748]
	ldrh	r2, [r2, r3, lsl #1]
	str	r2, [sp, #4]
	mov	r2, r5
	bl	sprintf
	adds	r5, r5, #1
	ldr	r3, [r7, #804]
	cmp	r5, #4
	ldrh	r3, [r3, r8]
	add	r4, r4, r0
	bne	.L796
.L795:
	subs	r0, r4, r6
	add	sp, sp, #32
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L803:
	.align	2
.L802:
	.word	.LC64
	.word	.LC65
	.word	.LC66
	.word	.LC67
	.word	.LC68
	.word	.LC69
	.word	.LC70
	.word	.LC71
	.word	.LC72
	.word	.LC73
	.word	.LC74
	.word	-1431655765
	.word	.LC75
	.fnend
	.size	FtlPrintInfo2buf, .-FtlPrintInfo2buf
	.align	1
	.global	ftl_proc_ftl_read
	.thumb
	.thumb_func
	.type	ftl_proc_ftl_read, %function
ftl_proc_ftl_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r5, r0
	ldr	r1, .L805
	ldr	r2, .L805+4
	bl	sprintf
	adds	r4, r5, r0
	mov	r0, r4
	bl	FtlPrintInfo2buf
	add	r0, r0, r4
	subs	r0, r0, r5
	pop	{r3, r4, r5, pc}
.L806:
	.align	2
.L805:
	.word	.LC76
	.word	.LC77
	.fnend
	.size	ftl_proc_ftl_read, .-ftl_proc_ftl_read
	.align	1
	.global	GetSwlReplaceBlock
	.thumb
	.thumb_func
	.type	GetSwlReplaceBlock, %function
GetSwlReplaceBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r4, .L835
	ldr	r2, [r4, #1040]
	ldr	r3, [r4, #1048]
	cmp	r2, r3
	bcs	.L808
	ldr	r2, .L835+4
	movs	r3, #0
	ldr	r0, [r4, #748]
	str	r3, [r4, #1032]
	ldrh	r1, [r2, #2848]
	mov	r5, r2
.L809:
	cmp	r3, r1
	bcs	.L834
	ldrh	r6, [r0, r3, lsl #1]
	adds	r3, r3, #1
	ldr	r2, [r4, #1032]
	add	r2, r2, r6
	str	r2, [r4, #1032]
	b	.L809
.L834:
	ldr	r6, [r4, #1032]
	mov	r0, r6
	bl	__aeabi_uidiv
	ldrh	r1, [r5, #2900]
	str	r0, [r4, #1040]
	ldr	r0, [r4, #1036]
	subs	r0, r6, r0
	bl	__aeabi_uidiv
	str	r0, [r4, #1032]
	b	.L811
.L808:
	ldr	r3, [r4, #1044]
	cmp	r2, r3
	bls	.L811
	ldr	r0, .L835+4
	adds	r3, r3, #1
	str	r3, [r4, #1044]
	movs	r3, #0
.L813:
	ldrh	r2, [r0, #2848]
	cmp	r3, r2
	bcs	.L811
	ldr	r1, [r4, #748]
	ldrh	r2, [r1, r3, lsl #1]
	adds	r2, r2, #1
	strh	r2, [r1, r3, lsl #1]	@ movhi
	adds	r3, r3, #1
	b	.L813
.L811:
	ldr	r6, [r4, #1048]
	ldr	r8, [r4, #1040]
	add	r3, r6, #256
	cmp	r3, r8
	bls	.L816
	ldr	r2, .L835
	add	r3, r6, #768
	ldr	r2, [r2, #1044]
	cmp	r3, r2
	bls	.L816
	ldr	r3, .L835+4
	ldr	r3, [r3, #2832]
	cbnz	r3, .L817
.L819:
	movw	r0, #65535
	b	.L818
.L817:
	cmp	r6, #40
	bhi	.L819
.L816:
	ldrh	r0, [r4, #828]
	add	r0, r0, r0, lsl #1
	ubfx	r0, r0, #2, #16
	bl	GetFreeBlockMaxEraseCount
	add	r3, r6, #64
	cmp	r0, r3
	mov	r9, r0
	bcs	.L820
	cmp	r6, #40
	bhi	.L819
.L820:
	ldr	r3, [r4, #808]
	ldr	r2, .L835
	cmp	r3, #0
	beq	.L819
	ldr	r1, .L835+4
	movw	r7, #65535
	ldr	r0, [r2, #804]
	mov	ip, #6
	ldr	r10, [r2, #748]
	mov	r5, r7
	ldrh	r1, [r1, #2848]
	mov	lr, r7
	str	r1, [sp, #20]
	movs	r1, #0
.L821:
	ldrh	r2, [r3]
	cmp	r2, lr
	str	r2, [sp, #16]
	beq	.L823
	adds	r1, r1, #1
	ldr	r2, [sp, #20]
	uxth	r1, r1
	cmp	r1, r2
	bhi	.L819
	ldrh	fp, [r3, #4]
	cmp	fp, #0
	beq	.L822
	subs	r3, r3, r0
	ldr	r2, .L835+8
	asrs	r3, r3, #1
	muls	r3, r2, r3
	uxth	r3, r3
	ldrh	fp, [r10, r3, lsl #1]
	cmp	fp, r6
	bls	.L827
	cmp	fp, r7
	itt	cc
	movcc	r7, fp
	movcc	r5, r3
.L822:
	ldr	r3, [sp, #16]
	mla	r3, ip, r3, r0
	b	.L821
.L827:
	mov	r5, r3
.L823:
	movw	r3, #65535
	cmp	r5, r3
	beq	.L819
	ldrh	fp, [r10, r5, lsl #1]
	lsls	r3, r5, #1
	cmp	fp, r6
	bls	.L825
	str	r3, [sp, #16]
	bl	GetFreeBlockMinEraseCount
	ldr	r3, [sp, #16]
	cmp	r0, r6
	it	hi
	strhi	r7, [r4, #1048]
.L825:
	cmp	fp, r8
	bcs	.L819
	add	r2, fp, #128
	cmp	r9, r2
	ble	.L819
	add	r2, fp, #256
	cmp	r2, r8
	bcc	.L826
	ldr	r2, [r4, #1044]
	add	fp, fp, #768
	cmp	fp, r2
	bcs	.L819
.L826:
	ldr	r2, [r4, #812]
	mov	r1, r5
	ldr	r0, .L835+12
	ldrh	r2, [r2, r3]
	str	r2, [sp]
	mov	r2, r8
	ldrh	r3, [r10, r3]
	str	r9, [sp, #8]
	str	r3, [sp, #4]
	ldr	r3, [r4, #1044]
	bl	printk
	mov	r0, r5
	movs	r3, #1
	str	r3, [r4, #1648]
.L818:
	add	sp, sp, #28
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L836:
	.align	2
.L835:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-1431655765
	.word	.LC78
	.fnend
	.size	GetSwlReplaceBlock, .-GetSwlReplaceBlock
	.align	1
	.global	free_data_superblock
	.thumb
	.thumb_func
	.type	free_data_superblock, %function
free_data_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	movw	r2, #65535
	cmp	r0, r2
	push	{r3, lr}
	.save {r3, lr}
	beq	.L838
	ldr	r2, .L839
	movs	r1, #0
	ldr	r2, [r2, #812]
	strh	r1, [r2, r0, lsl #1]	@ movhi
	bl	INSERT_FREE_LIST
.L838:
	movs	r0, #0
	pop	{r3, pc}
.L840:
	.align	2
.L839:
	.word	.LANCHOR2
	.fnend
	.size	free_data_superblock, .-free_data_superblock
	.align	1
	.global	FtlGcBufInit
	.thumb
	.thumb_func
	.type	FtlGcBufInit, %function
FtlGcBufInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L852
	mov	ip, #36
	ldr	r0, .L852+4
	movs	r3, #0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movs	r7, #12
	mov	lr, #1
	mov	r6, r0
	mov	r4, r2
	str	r3, [r2, #1652]
.L842:
	ldrh	r1, [r0, #2840]
	adds	r5, r3, #1
	uxth	r3, r3
	ldr	r8, .L852+4
	cmp	r3, r1
	bcs	.L850
	mul	r8, r7, r3
	ldr	r9, [r2, #1656]
	add	r1, r9, r8
	str	lr, [r1, #8]
	ldrh	r1, [r0, #2918]
	muls	r1, r3, r1
	it	mi
	addmi	r1, r1, #3
	bic	r10, r1, #3
	ldr	r1, [r4, #1660]
	add	r1, r1, r10
	str	r1, [r9, r8]
	ldr	r10, [r2, #1656]
	ldrh	r1, [r6, #2920]
	muls	r1, r3, r1
	it	mi
	addmi	r1, r1, #3
	add	r9, r10, r8
	bic	fp, r1, #3
	ldr	r1, [r4, #1664]
	add	r1, r1, fp
	str	r1, [r9, #4]
	ldr	r1, [r2, #1668]
	mla	r3, ip, r3, r1
	ldr	r1, [r10, r8]
	str	r1, [r3, #8]
	ldr	r1, [r9, #4]
	str	r1, [r3, #12]
	mov	r3, r5
	b	.L842
.L850:
	ldr	r0, .L852
	mov	lr, #12
	movs	r5, #0
.L846:
	ldr	r3, [r2, #1672]
	cmp	r1, r3
	bcs	.L851
	mul	r7, lr, r1
	ldr	r6, [r0, #1656]
	ldr	r4, [r0, #1660]
	adds	r3, r6, r7
	str	r5, [r3, #8]
	ldrh	r3, [r8, #2918]
	muls	r3, r1, r3
	it	mi
	addmi	r3, r3, #3
	bic	r3, r3, #3
	add	r3, r3, r4
	str	r3, [r6, r7]
	ldr	r3, [r2, #1656]
	ldr	r6, [r0, #1664]
	add	r7, r7, r3
	ldrh	r3, [r8, #2920]
	muls	r3, r1, r3
	add	r1, r1, #1
	it	mi
	addmi	r3, r3, #3
	bic	r3, r3, #3
	uxth	r1, r1
	add	r3, r3, r6
	str	r3, [r7, #4]
	b	.L846
.L851:
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L853:
	.align	2
.L852:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlGcBufInit, .-FtlGcBufInit
	.align	1
	.global	FtlGcBufFree
	.thumb
	.thumb_func
	.type	FtlGcBufFree, %function
FtlGcBufFree:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L861
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	movs	r5, #0
	ldr	lr, [r3, #1672]
	mov	r10, #36
	ldr	r6, [r3, #1656]
	mov	r8, #12
	mov	ip, r5
.L855:
	uxth	r4, r5
	cmp	r4, r1
	bcs	.L854
	mla	r4, r10, r4, r0
	movs	r2, #0
.L856:
	uxth	r3, r2
	cmp	r3, lr
	bcs	.L857
	mul	r3, r8, r3
	ldr	r7, [r4, #8]
	adds	r2, r2, #1
	add	r9, r6, r3
	ldr	r3, [r6, r3]
	cmp	r3, r7
	bne	.L856
	str	ip, [r9, #8]
.L857:
	adds	r5, r5, #1
	b	.L855
.L854:
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L862:
	.align	2
.L861:
	.word	.LANCHOR2
	.fnend
	.size	FtlGcBufFree, .-FtlGcBufFree
	.align	1
	.global	FtlGcBufAlloc
	.thumb
	.thumb_func
	.type	FtlGcBufAlloc, %function
FtlGcBufAlloc:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L871
	mov	ip, #1
	push	{r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	movs	r4, #0
	ldr	r5, [r3, #1672]
	mov	lr, #12
	ldr	r6, [r3, #1656]
	mov	r8, #36
.L864:
	uxth	r2, r4
	cmp	r2, r1
	bcs	.L870
	mov	r9, #0
.L865:
	uxth	r3, r9
	cmp	r3, r5
	bcs	.L866
	mla	r3, lr, r3, r6
	add	r9, r9, #1
	ldr	r7, [r3, #8]
	cmp	r7, #0
	bne	.L865
	mla	r2, r8, r2, r0
	ldr	r7, [r3]
	str	ip, [r3, #8]
	str	r7, [r2, #8]
	ldr	r3, [r3, #4]
	str	r3, [r2, #12]
.L866:
	adds	r4, r4, #1
	b	.L864
.L870:
	pop	{r4, r5, r6, r7, r8, r9, pc}
.L872:
	.align	2
.L871:
	.word	.LANCHOR2
	.fnend
	.size	FtlGcBufAlloc, .-FtlGcBufAlloc
	.align	1
	.global	IsBlkInGcList
	.thumb
	.thumb_func
	.type	IsBlkInGcList, %function
IsBlkInGcList:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L879
	ldr	r3, [r2, #1676]
	ldrh	r2, [r2, #1680]
	add	r2, r3, r2, lsl #1
.L874:
	cmp	r3, r2
	beq	.L878
	ldrh	r1, [r3], #2
	cmp	r1, r0
	bne	.L874
	movs	r0, #1
	bx	lr
.L878:
	movs	r0, #0
	bx	lr
.L880:
	.align	2
.L879:
	.word	.LANCHOR2
	.fnend
	.size	IsBlkInGcList, .-IsBlkInGcList
	.align	1
	.global	FtlGcUpdatePage
	.thumb
	.thumb_func
	.type	FtlGcUpdatePage, %function
FtlGcUpdatePage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r6, r0
	ubfx	r0, r0, #10, #16
	mov	r7, r1
	mov	r8, r2
	bl	P2V_block_in_plane
	ldr	r3, .L888
	movs	r4, #0
	ldrh	r1, [r3, #1680]
	ldr	r2, [r3, #1676]
.L882:
	uxth	r5, r4
	cmp	r5, r1
	bcs	.L886
	adds	r4, r4, #1
	add	lr, r2, r4, lsl #1
	ldrh	lr, [lr, #-2]
	cmp	lr, r0
	bne	.L882
.L886:
	cmp	r5, r1
	bne	.L884
	strh	r0, [r2, r5, lsl #1]	@ movhi
	ldrh	r0, [r3, #1680]
	adds	r0, r0, #1
	strh	r0, [r3, #1680]	@ movhi
.L884:
	ldrh	r2, [r3, #1688]
	movs	r0, #12
	muls	r0, r2, r0
	ldr	r2, [r3, #1684]
	adds	r1, r2, r0
	str	r7, [r1, #4]
	str	r8, [r1, #8]
	str	r6, [r2, r0]
	ldrh	r2, [r3, #1688]
	adds	r2, r2, #1
	strh	r2, [r3, #1688]	@ movhi
	pop	{r4, r5, r6, r7, r8, pc}
.L889:
	.align	2
.L888:
	.word	.LANCHOR2
	.fnend
	.size	FtlGcUpdatePage, .-FtlGcUpdatePage
	.align	1
	.global	FtlGcRefreshOpenBlock
	.thumb
	.thumb_func
	.type	FtlGcRefreshOpenBlock, %function
FtlGcRefreshOpenBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r5, r0
	ldr	r4, .L897
	ldrh	r3, [r4, #1690]
	cmp	r3, r0
	beq	.L892
	ldrh	r3, [r4, #1692]
	cmp	r3, r0
	beq	.L892
	ldrh	r3, [r4, #1694]
	cmp	r3, r0
	beq	.L892
	ldrh	r3, [r4, #1696]
	cmp	r3, r0
	beq	.L892
	ldr	r0, .L897+4
	mov	r1, r5
	bl	printk
	ldrh	r2, [r4, #1690]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L894
	strh	r5, [r4, #1690]	@ movhi
	b	.L892
.L894:
	ldrh	r2, [r4, #1692]
	cmp	r2, r3
	bne	.L895
	strh	r5, [r4, #1692]	@ movhi
	b	.L892
.L895:
	ldrh	r2, [r4, #1694]
	cmp	r2, r3
	bne	.L896
	strh	r5, [r4, #1694]	@ movhi
	b	.L892
.L896:
	ldrh	r2, [r4, #1696]
	cmp	r2, r3
	it	eq
	strheq	r5, [r4, #1696]	@ movhi
.L892:
	movs	r0, #0
	pop	{r3, r4, r5, pc}
.L898:
	.align	2
.L897:
	.word	.LANCHOR2
	.word	.LC79
	.fnend
	.size	FtlGcRefreshOpenBlock, .-FtlGcRefreshOpenBlock
	.align	1
	.global	FtlGcRefreshBlock
	.thumb
	.thumb_func
	.type	FtlGcRefreshBlock, %function
FtlGcRefreshBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r5, r0
	ldr	r4, .L909
	ldrh	r3, [r4, #1690]
	cmp	r3, r0
	beq	.L907
	ldrh	r3, [r4, #1692]
	cmp	r3, r0
	beq	.L907
	ldrh	r3, [r4, #1694]
	cmp	r3, r0
	beq	.L907
	ldrh	r3, [r4, #1696]
	cmp	r3, r0
	beq	.L907
	ldr	r0, .L909+4
	mov	r1, r5
	bl	printk
	ldrh	r2, [r4, #1690]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L901
	strh	r5, [r4, #1690]	@ movhi
	b	.L907
.L901:
	ldrh	r2, [r4, #1692]
	cmp	r2, r3
	bne	.L902
	strh	r5, [r4, #1692]	@ movhi
	b	.L907
.L902:
	ldrh	r2, [r4, #1694]
	cmp	r2, r3
	bne	.L903
	strh	r5, [r4, #1694]	@ movhi
	b	.L907
.L903:
	ldrh	r2, [r4, #1696]
	cmp	r2, r3
	bne	.L908
	strh	r5, [r4, #1696]	@ movhi
.L907:
	movs	r0, #0
	pop	{r3, r4, r5, pc}
.L908:
	mov	r0, #-1
	pop	{r3, r4, r5, pc}
.L910:
	.align	2
.L909:
	.word	.LANCHOR2
	.word	.LC79
	.fnend
	.size	FtlGcRefreshBlock, .-FtlGcRefreshBlock
	.align	1
	.global	FtlGcMarkBadPhyBlk
	.thumb
	.thumb_func
	.type	FtlGcMarkBadPhyBlk, %function
FtlGcMarkBadPhyBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r0
	bl	P2V_block_in_plane
	ldr	r4, .L920
	mov	r2, r5
	ldrh	r1, [r4, #1698]
	mov	r6, r0
	ldr	r0, .L920+4
	bl	printk
	mov	r0, r6
	bl	FtlGcRefreshBlock
	ldr	r3, .L920+8
	ldr	r2, [r3, #2832]
	mov	r3, r4
	cbz	r2, .L912
	ldr	r1, [r4, #748]
	ldrh	r2, [r1, r6, lsl #1]
	cmp	r2, #39
	itt	hi
	subhi	r2, r2, #40
	strhhi	r2, [r1, r6, lsl #1]	@ movhi
.L912:
	ldrh	r2, [r3, #1698]
	movs	r1, #0
	ldr	r4, .L920+12
.L913:
	uxth	r0, r1
	cmp	r0, r2
	bcs	.L919
	adds	r1, r1, #1
	add	r0, r4, r1, lsl #1
	ldrh	r0, [r0, #-2]
	cmp	r0, r5
	bne	.L913
	b	.L914
.L919:
	cmp	r2, #15
	itttt	ls
	addls	r1, r2, #1
	strhls	r1, [r3, #1698]	@ movhi
	addls	r3, r3, r2, lsl #1
	strhls	r5, [r3, #1700]	@ movhi
.L914:
	movs	r0, #0
	pop	{r4, r5, r6, pc}
.L921:
	.align	2
.L920:
	.word	.LANCHOR2
	.word	.LC80
	.word	.LANCHOR0
	.word	.LANCHOR2+1700
	.fnend
	.size	FtlGcMarkBadPhyBlk, .-FtlGcMarkBadPhyBlk
	.align	1
	.global	FtlGcReFreshBadBlk
	.thumb
	.thumb_func
	.type	FtlGcReFreshBadBlk, %function
FtlGcReFreshBadBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	ldr	r4, .L928
	ldrh	r3, [r4, #1698]
	cbz	r3, .L923
	ldrh	r1, [r4, #1690]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L923
	ldrh	r2, [r4, #1734]
	cmp	r2, r3
	itt	cs
	movcs	r3, #0
	strhcs	r3, [r4, #1734]	@ movhi
	ldrh	r3, [r4, #1734]
	add	r3, r4, r3, lsl #1
	ldrh	r0, [r3, #1700]
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	ldrh	r3, [r4, #1734]
	adds	r3, r3, #1
	strh	r3, [r4, #1734]	@ movhi
.L923:
	movs	r0, #0
	pop	{r4, pc}
.L929:
	.align	2
.L928:
	.word	.LANCHOR2
	.fnend
	.size	FtlGcReFreshBadBlk, .-FtlGcReFreshBadBlk
	.align	1
	.global	ftl_memset
	.thumb
	.thumb_func
	.type	ftl_memset, %function
ftl_memset:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	mov	r4, r0
	cbz	r2, .L931
	bl	memset
.L931:
	mov	r0, r4
	pop	{r4, pc}
	.fnend
	.size	ftl_memset, .-ftl_memset
	.align	1
	.global	FtlMemInit
	.thumb
	.thumb_func
	.type	FtlMemInit, %function
FtlMemInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r0, #1024
	ldr	r4, .L1039
	movs	r6, #0
	ldr	r5, .L1039+4
	movw	r3, #65535
	movs	r7, #12
	mov	r8, #36
	strh	r6, [r4, #1736]	@ movhi
	str	r6, [r4, #1740]
	str	r6, [r4, #1120]
	str	r6, [r4, #1024]
	str	r6, [r4, #1028]
	str	r6, [r4, #1012]
	str	r6, [r4, #1000]
	str	r6, [r4, #996]
	str	r6, [r4, #1004]
	str	r6, [r4, #1008]
	str	r6, [r4, #992]
	str	r6, [r4, #1032]
	str	r6, [r4, #1036]
	str	r6, [r4, #752]
	str	r6, [r4, #1044]
	str	r6, [r4, #1048]
	str	r6, [r4, #1744]
	str	r6, [r4, #1648]
	str	r3, [r4, #1748]
	str	r6, [r4, #1752]
	str	r6, [r4, #1640]
	str	r6, [r4, #1756]
	strh	r3, [r4, #1690]	@ movhi
	strh	r3, [r4, #1692]	@ movhi
	strh	r3, [r4, #1694]	@ movhi
	strh	r3, [r4, #1696]	@ movhi
	movs	r3, #32
	strh	r6, [r4, #1644]	@ movhi
	strh	r3, [r4, #1636]	@ movhi
	movs	r3, #128
	strh	r6, [r4, #1698]	@ movhi
	strh	r3, [r4, #1638]	@ movhi
	strh	r6, [r4, #1760]	@ movhi
	strh	r6, [r4, #1734]	@ movhi
	ldrh	r1, [r5, #2914]
	bl	__aeabi_idiv
	ldrh	r3, [r5, #2840]
	str	r6, [r5, #2956]
	lsls	r3, r3, #2
	cmp	r0, r3
	str	r0, [r4, #1764]
	ldrh	r0, [r5, #2912]
	it	hi
	strhi	r3, [r4, #1764]
	lsls	r0, r0, #1
	bl	ftl_malloc
	str	r0, [r4, #1676]
	ldrh	r0, [r5, #2912]
	muls	r0, r7, r0
	bl	ftl_malloc
	ldrh	r6, [r5, #2840]
	mul	r6, r8, r6
	lsl	r9, r6, #3
	str	r0, [r4, #1684]
	mov	r0, r9
	bl	ftl_malloc
	str	r0, [r4, #1768]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #1772]
	mov	r0, r9
	bl	ftl_malloc
	str	r0, [r4, #1776]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #744]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #1668]
	ldr	r0, [r4, #1764]
	mul	r0, r8, r0
	bl	ftl_malloc
	ldrh	r6, [r5, #2918]
	ldrh	r3, [r5, #2840]
	lsls	r3, r3, #1
	adds	r3, r3, #1
	str	r3, [r4, #1672]
	str	r0, [r5, #2960]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #1780]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #1784]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #1788]
	ldr	r0, [r4, #1672]
	muls	r0, r6, r0
	bl	ftl_malloc
	str	r0, [r4, #1660]
	ldr	r0, [r4, #1764]
	muls	r0, r6, r0
	bl	ftl_malloc
	str	r0, [r4, #1792]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #1796]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #1800]
	ldr	r0, [r4, #1672]
	muls	r0, r7, r0
	bl	ftl_malloc
	ldrh	r3, [r5, #2840]
	ldrh	r6, [r5, #2920]
	muls	r6, r3, r6
	str	r0, [r4, #1656]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #1804]
	lsls	r0, r6, #3
	bl	ftl_malloc
	ldrh	r3, [r5, #2920]
	str	r0, [r4, #1808]
	ldr	r0, [r4, #1672]
	muls	r0, r3, r0
	bl	ftl_malloc
	ldrh	r3, [r5, #2920]
	str	r0, [r4, #1664]
	ldr	r0, [r4, #1764]
	muls	r0, r3, r0
	bl	ftl_malloc
	str	r0, [r4, #1812]
	ldrh	r0, [r5, #2850]
	lsls	r0, r0, #1
	uxth	r0, r0
	strh	r0, [r4, #1816]	@ movhi
	bl	ftl_malloc
	str	r0, [r4, #1820]
	ldrh	r0, [r4, #1816]
	addw	r0, r0, #547
	lsrs	r0, r0, #9
	strh	r0, [r4, #1816]	@ movhi
	lsls	r0, r0, #9
	bl	ftl_malloc
	ldrh	r6, [r5, #2850]
	lsls	r6, r6, #1
	str	r0, [r4, #1824]
	adds	r0, r0, #32
	str	r0, [r4, #748]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #1828]
	mov	r0, r6
	bl	ftl_malloc
	ldr	r6, [r5, #2936]
	lsls	r6, r6, #1
	str	r0, [r4, #812]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #1832]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #1836]
	ldrh	r0, [r5, #2850]
	lsrs	r0, r0, #3
	adds	r0, r0, #4
	bl	ftl_malloc
	str	r0, [r4, #988]
	ldrh	r0, [r5, #2928]
	lsls	r0, r0, #1
	bl	ftl_malloc
	str	r0, [r5, #2952]
	ldrh	r0, [r5, #2928]
	lsls	r0, r0, #1
	bl	ftl_malloc
	str	r0, [r4, #1840]
	ldrh	r0, [r5, #2928]
	lsls	r0, r0, #2
	bl	ftl_malloc
	str	r0, [r4, #1844]
	ldrh	r0, [r5, #2930]
	lsls	r0, r0, #2
	bl	ftl_malloc
	ldrh	r2, [r5, #2930]
	movs	r1, #0
	lsls	r2, r2, #2
	str	r0, [r4, #1848]
	bl	ftl_memset
	ldrh	r6, [r5, #2944]
	lsls	r6, r6, #2
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #1852]
	mov	r0, r6
	bl	ftl_malloc
	ldr	r6, .L1039+8
	str	r0, [r4, #1856]
	ldr	r0, [r5, #2936]
	lsls	r0, r0, #2
	bl	ftl_malloc
	str	r0, [r4, #1860]
	ldrh	r0, [r5, #2946]
	muls	r0, r7, r0
	bl	ftl_malloc
	ldrh	r3, [r5, #2946]
	str	r0, [r4, #976]
	ldrh	r0, [r5, #2918]
	muls	r0, r3, r0
	bl	ftl_malloc
	ldrh	r3, [r5, #2850]
	str	r0, [r4, #1864]
	movs	r0, #6
	muls	r0, r3, r0
	bl	ftl_malloc
	ldrh	r3, [r5, #2906]
	adds	r3, r3, #31
	asrs	r3, r3, #5
	strh	r3, [r4, #1868]	@ movhi
	str	r0, [r4, #804]
	ldrh	r0, [r5, #2862]
	muls	r0, r3, r0
	lsls	r0, r0, #2
	bl	ftl_malloc
	ldrh	r2, [r4, #1868]
	ldrh	r7, [r5, #2862]
	movs	r3, #1
	lsls	r2, r2, #2
	mov	r1, r2
	str	r0, [r5, #2996]
.L937:
	cmp	r3, r7
	bcs	.L1037
	ldr	r0, [r5, #2996]
	adds	r3, r3, #1
	add	r0, r0, r1
	add	r1, r1, r2
	str	r0, [r6, #4]!
	b	.L937
.L1037:
	ldr	r2, .L1039+12
	movs	r1, #0
.L939:
	cmp	r3, #8
	beq	.L1038
	add	r0, r2, r3, lsl #2
	adds	r3, r3, #1
	str	r1, [r0, #28]
	b	.L939
.L1038:
	ldr	r2, [r4, #1832]
	ldr	r3, .L1039
	cbnz	r2, .L941
.L943:
	ldr	r1, .L1039+16
	ldr	r0, .L1039+20
	bl	printk
	mov	r0, #-1
	pop	{r3, r4, r5, r6, r7, r8, r9, pc}
.L1040:
	.align	2
.L1039:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR0+2996
	.word	.LANCHOR0+2968
	.word	.LANCHOR3
	.word	.LC81
.L941:
	ldr	r2, [r3, #1836]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1852]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1860]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #976]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1864]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #804]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r5, #2996]
	cmp	r2, #0
	beq	.L943
	ldr	r3, [r3, #812]
	cmp	r3, #0
	beq	.L943
	ldr	r2, [r4, #1676]
	ldr	r3, .L1041
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1684]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1768]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1776]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #744]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1668]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1772]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1780]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1784]
	cmp	r2, #0
	beq	.L943
	ldr	r3, [r3, #1788]
	cmp	r3, #0
	beq	.L943
	ldr	r2, [r4, #1660]
	ldr	r3, .L1041
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1796]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1800]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1656]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1804]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1808]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1664]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #748]
	cmp	r2, #0
	beq	.L943
	ldr	r3, [r3, #1820]
	cmp	r3, #0
	beq	.L943
	ldr	r3, .L1041+4
	ldr	r3, [r3, #2952]
	cmp	r3, #0
	beq	.L943
	ldr	r3, .L1041
	ldr	r2, [r3, #1840]
	cmp	r2, #0
	beq	.L943
	ldr	r2, [r3, #1844]
	cmp	r2, #0
	beq	.L943
	ldr	r3, [r3, #1848]
	cmp	r3, #0
	beq	.L943
	movs	r0, #0
	pop	{r3, r4, r5, r6, r7, r8, r9, pc}
.L1042:
	.align	2
.L1041:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlMemInit, .-FtlMemInit
	.align	1
	.global	FtlBbt2Bitmap
	.thumb
	.thumb_func
	.type	FtlBbt2Bitmap, %function
FtlBbt2Bitmap:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r4, r0
	ldr	r3, .L1048
	mov	r0, r1
	mov	r5, r1
	movs	r1, #0
	movs	r6, #1
	movw	r7, #65535
	ldrh	r2, [r3, #1868]
	lsls	r2, r2, #2
	bl	ftl_memset
	subs	r2, r4, #2
	addw	r4, r4, #1022
.L1045:
	ldrh	r3, [r2, #2]!
	cmp	r3, r7
	beq	.L1043
	lsrs	r1, r3, #5
	cmp	r2, r4
	and	r3, r3, #31
	ldr	r0, [r5, r1, lsl #2]
	lsl	r3, r6, r3
	orr	r3, r3, r0
	str	r3, [r5, r1, lsl #2]
	bne	.L1045
.L1043:
	pop	{r3, r4, r5, r6, r7, pc}
.L1049:
	.align	2
.L1048:
	.word	.LANCHOR2
	.fnend
	.size	FtlBbt2Bitmap, .-FtlBbt2Bitmap
	.align	1
	.global	ftl_free_no_use_map_blk
	.thumb
	.thumb_func
	.type	ftl_free_no_use_map_blk, %function
ftl_free_no_use_map_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movs	r1, #0
	ldrh	r2, [r0, #10]
	mov	r4, r0
	ldr	r6, [r0, #20]
	ldr	r7, [r0, #12]
	ldr	r5, [r0, #24]
	lsls	r2, r2, #1
	mov	r0, r6
	bl	ftl_memset
	movs	r1, #0
.L1051:
	ldrh	r2, [r4, #6]
	uxth	r3, r1
	cmp	r2, r3
	bls	.L1070
	ldr	r2, [r5, r3, lsl #2]
	movs	r0, #0
	ubfx	r2, r2, #10, #16
.L1052:
	ldrh	lr, [r4, #10]
	uxth	r3, r0
	cmp	lr, r3
	bls	.L1071
	ldrh	lr, [r7, r3, lsl #1]
	cmp	lr, r2
	bne	.L1053
	cbz	r2, .L1053
	ldrh	lr, [r6, r3, lsl #1]
	add	lr, lr, #1
	strh	lr, [r6, r3, lsl #1]	@ movhi
.L1053:
	adds	r0, r0, #1
	b	.L1052
.L1071:
	adds	r1, r1, #1
	b	.L1051
.L1070:
	ldr	r3, .L1074
	mov	r8, #0
	mov	r10, r8
	ldrh	r2, [r3, #2910]
	ldrh	r3, [r4]
	strh	r2, [r6, r3, lsl #1]	@ movhi
	mov	r2, r8
	ldrh	r9, [r6]
.L1056:
	ldrh	r3, [r4, #10]
	uxth	r5, r8
	cmp	r3, r5
	bls	.L1072
	ldrh	r3, [r6, r5, lsl #1]
	lsls	r1, r5, #1
	cmp	r9, r3
	bls	.L1057
	ldrh	r0, [r7, r5, lsl #1]
	add	fp, r7, r1
	cbnz	r0, .L1058
	b	.L1059
.L1057:
	cbnz	r3, .L1059
	ldrh	r0, [r7, r1]
	add	fp, r7, r1
	cbz	r0, .L1059
	mov	r5, r2
	b	.L1061
.L1058:
	cbnz	r3, .L1073
	mov	r9, r3
.L1061:
	movs	r1, #1
	bl	FtlFreeSysBlkQueueIn
	strh	r10, [fp]	@ movhi
	ldrh	r3, [r4, #8]
	mov	r2, r5
	subs	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
.L1059:
	add	r8, r8, #1
	b	.L1056
.L1073:
	mov	r2, r5
	mov	r9, r3
	b	.L1059
.L1072:
	mov	r0, r2
	pop	{r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1075:
	.align	2
.L1074:
	.word	.LANCHOR0
	.fnend
	.size	ftl_free_no_use_map_blk, .-ftl_free_no_use_map_blk
	.align	1
	.global	FtlL2PDataInit
	.thumb
	.thumb_func
	.type	FtlL2PDataInit, %function
FtlL2PDataInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	movs	r1, #0
	ldr	r4, .L1080
	movs	r7, #0
	ldr	r5, .L1080+4
	mov	r6, r7
	ldr	r2, [r4, #2936]
	mov	r9, r4
	ldr	r0, [r5, #1836]
	lsls	r2, r2, #1
	bl	ftl_memset
	ldrh	r3, [r4, #2918]
	ldrh	r2, [r4, #2946]
	movs	r1, #255
	ldr	r0, [r5, #1864]
	muls	r2, r3, r2
	bl	ftl_memset
	mov	r3, r5
	movs	r5, #12
	mov	r2, r4
	movw	lr, #65535
.L1077:
	ldrh	ip, [r2, #2946]
	adds	r0, r7, #1
	uxth	r7, r7
	ldr	r1, .L1080
	cmp	ip, r7
	bls	.L1079
	mul	r1, r5, r7
	ldr	ip, [r3, #976]
	add	r8, ip, r1
	str	r6, [r8, #4]
	strh	lr, [ip, r1]	@ movhi
	ldr	r4, [r3, #976]
	add	ip, r4, r1
	ldrh	r1, [r9, #2918]
	muls	r1, r7, r1
	ldr	r7, [r3, #1864]
	bic	r1, r1, #3
	add	r1, r1, r7
	mov	r7, r0
	str	r1, [ip, #8]
	b	.L1077
.L1079:
	movw	r2, #65535
	strh	r2, [r3, #1874]	@ movhi
	strh	r2, [r3, #1872]	@ movhi
	ldr	r2, [r1, #2936]
	strh	r2, [r3, #1882]	@ movhi
	movw	r2, #61634
	strh	r2, [r3, #1876]	@ movhi
	ldrh	r2, [r3, #1916]
	strh	r2, [r3, #1880]	@ movhi
	ldrh	r2, [r1, #2944]
	strh	r2, [r3, #1878]	@ movhi
	ldr	r2, [r3, #1832]
	str	r2, [r3, #1884]
	ldr	r2, [r3, #1860]
	str	r2, [r3, #1888]
	ldr	r2, [r3, #1836]
	str	r2, [r3, #1892]
	ldr	r2, [r3, #1852]
	str	r2, [r3, #1896]
	pop	{r3, r4, r5, r6, r7, r8, r9, pc}
.L1081:
	.align	2
.L1080:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlL2PDataInit, .-FtlL2PDataInit
	.align	1
	.global	FtlVariablesInit
	.thumb
	.thumb_func
	.type	FtlVariablesInit, %function
FtlVariablesInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movs	r4, #0
	ldr	r6, .L1083
	movw	r3, #65535
	ldr	r5, .L1083+4
	mov	r1, r4
	ldrh	r2, [r6, #2928]
	ldr	r0, [r6, #2952]
	strh	r3, [r5, #1924]	@ movhi
	mov	r3, #-1
	lsls	r2, r2, #1
	str	r3, [r5, #1936]
	str	r4, [r5, #1920]
	str	r4, [r5, #1928]
	str	r4, [r5, #1932]
	str	r4, [r6, #2832]
	strh	r4, [r6, #2950]	@ movhi
	bl	ftl_memset
	ldrh	r2, [r6, #2850]
	mov	r1, r4
	ldr	r0, [r5, #748]
	lsls	r2, r2, #1
	bl	ftl_memset
	ldrh	r2, [r6, #2850]
	mov	r1, r4
	ldr	r0, [r5, #1820]
	lsls	r2, r2, #1
	bl	ftl_memset
	add	r0, r5, #756
	movs	r1, #48
	bl	__memzero
	addw	r0, r5, #1124
	mov	r1, #512
	bl	__memzero
	bl	FtlGcBufInit
	bl	FtlL2PDataInit
	mov	r0, r4
	pop	{r4, r5, r6, pc}
.L1084:
	.align	2
.L1083:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlVariablesInit, .-FtlVariablesInit
	.align	1
	.global	SupperBlkListInit
	.thumb
	.thumb_func
	.type	SupperBlkListInit, %function
SupperBlkListInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movs	r2, #6
	ldr	r6, .L1096
	movs	r5, #0
	ldr	r4, .L1096+4
	.pad #20
	sub	sp, sp, #20
	movs	r1, #0
	mov	r9, r5
	ldrh	r3, [r6, #2850]
	mov	r10, r5
	ldr	r0, [r4, #804]
	mov	r8, r6
	muls	r2, r3, r2
	bl	ftl_memset
	str	r5, [r4, #824]
	str	r5, [r4, #808]
	str	r5, [r4, #816]
	strh	r5, [r4, #820]	@ movhi
	strh	r5, [r4, #828]	@ movhi
	strh	r5, [r4, #1736]	@ movhi
.L1086:
	uxth	fp, r5
	ldrh	r2, [r8, #2848]
	sxth	r7, fp
	cmp	r7, r2
	bge	.L1093
	ldr	r3, .L1096
	movs	r2, #0
	mov	r6, r2
	ldrh	ip, [r3, #2840]
	ldrh	r3, [r3, #2908]
	str	r3, [sp, #4]
.L1094:
	sxth	r1, r2
	cmp	r1, ip
	bge	.L1095
	add	r1, r1, r8
	str	r2, [sp, #12]
	str	ip, [sp, #8]
	ldrb	r0, [r1, #2868]	@ zero_extendqisi2
	mov	r1, fp
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	ldr	ip, [sp, #8]
	ldr	r2, [sp, #12]
	cbnz	r0, .L1087
	ldr	r3, [sp, #4]
	add	r6, r6, r3
	uxth	r6, r6
.L1087:
	adds	r2, r2, #1
	b	.L1094
.L1095:
	cbz	r6, .L1089
	sxth	r1, r6
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r6, r0
	b	.L1090
.L1089:
	ldr	r1, [r4, #812]
	sxth	r2, fp
	movw	r0, #65535
	strh	r0, [r1, r2, lsl #1]	@ movhi
.L1090:
	add	r0, r7, r7, lsl #1
	ldr	r1, [r4, #804]
	ldr	r2, .L1096+4
	add	r1, r1, r0, lsl #1
	strh	r6, [r1, #4]	@ movhi
	ldrh	r1, [r4, #832]
	cmp	r7, r1
	beq	.L1091
	ldrh	r1, [r2, #880]
	cmp	r7, r1
	beq	.L1091
	ldrh	r1, [r2, #928]
	cmp	r7, r1
	beq	.L1091
	ldr	r2, [r2, #812]
	ldrh	r2, [r2, r7, lsl #1]
	cbnz	r2, .L1092
	add	r9, r9, #1
	mov	r0, fp
	uxth	r9, r9
	bl	INSERT_FREE_LIST
	b	.L1091
.L1092:
	add	r10, r10, #1
	mov	r0, fp
	uxth	r10, r10
	bl	INSERT_DATA_LIST
.L1091:
	adds	r5, r5, #1
	b	.L1086
.L1093:
	movs	r0, #0
	strh	r10, [r4, #820]	@ movhi
	strh	r9, [r4, #828]	@ movhi
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1097:
	.align	2
.L1096:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	SupperBlkListInit, .-SupperBlkListInit
	.align	1
	.global	FtlGcPageVarInit
	.thumb
	.thumb_func
	.type	FtlGcPageVarInit, %function
FtlGcPageVarInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movs	r1, #255
	ldr	r5, .L1099
	movs	r3, #0
	ldr	r4, .L1099+4
	ldrh	r2, [r5, #2912]
	ldr	r0, [r4, #1676]
	strh	r3, [r4, #1680]	@ movhi
	lsls	r2, r2, #1
	strh	r3, [r4, #1688]	@ movhi
	bl	ftl_memset
	ldrh	r3, [r5, #2912]
	movs	r2, #12
	ldr	r0, [r4, #1684]
	movs	r1, #255
	muls	r2, r3, r2
	bl	ftl_memset
	pop	{r3, r4, r5, lr}
	b	FtlGcBufInit
.L1100:
	.align	2
.L1099:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlGcPageVarInit, .-FtlGcPageVarInit
	.align	1
	.global	ftl_memcpy
	.thumb
	.thumb_func
	.type	ftl_memcpy, %function
ftl_memcpy:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	memcpy
	.fnend
	.size	ftl_memcpy, .-ftl_memcpy
	.align	1
	.global	FlashReadIdbData
	.thumb
	.thumb_func
	.type	FlashReadIdbData, %function
FlashReadIdbData:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	.save {r3, lr}
	mov	r2, #2048
	ldr	r1, .L1103
	bl	ftl_memcpy
	movs	r0, #0
	pop	{r3, pc}
.L1104:
	.align	2
.L1103:
	.word	.LANCHOR2+1940
	.fnend
	.size	FlashReadIdbData, .-FlashReadIdbData
	.align	1
	.global	FlashLoadPhyInfoInRam
	.thumb
	.thumb_func
	.type	FlashLoadPhyInfoInRam, %function
FlashLoadPhyInfoInRam:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	movs	r4, #0
	ldr	r5, .L1117
.L1109:
	mov	r0, r5
	ldr	r1, .L1117+4
	ldrb	r2, [r5, #-1]	@ zero_extendqisi2
	bl	FlashMemCmp8
	mov	r6, r0
	cbnz	r0, .L1106
	ldr	r5, .L1117+8
	lsls	r4, r4, #5
	add	r2, r5, #508
	adds	r7, r2, r4
	beq	.L1112
	add	r4, r4, r5
	mov	r3, r0
	addw	r1, r5, #3100
	ldrb	r2, [r4, #530]	@ zero_extendqisi2
	b	.L1111
.L1106:
	adds	r4, r4, #1
	adds	r5, r5, #32
	cmp	r4, #81
	bne	.L1109
	b	.L1112
.L1116:
	adds	r3, r3, #1
	cmp	r3, #4
	beq	.L1110
.L1111:
	lsls	r0, r3, #5
	ldrb	r0, [r0, r1]	@ zero_extendqisi2
	cmp	r0, r2
	bne	.L1116
.L1110:
	ldr	r1, .L1117+12
	movs	r2, #32
	ldr	r0, .L1117+16
	add	r1, r1, r3, lsl #5
	bl	ftl_memcpy
	mov	r1, r7
	movs	r2, #32
	ldr	r0, .L1117+20
	bl	ftl_memcpy
	ldrh	r0, [r5, #266]
	bl	FlashBlockAlignInit
	b	.L1107
.L1112:
	mov	r6, #-1
.L1107:
	mov	r0, r6
	pop	{r3, r4, r5, r6, r7, pc}
.L1118:
	.align	2
.L1117:
	.word	.LANCHOR1+509
	.word	.LANCHOR0+1788
	.word	.LANCHOR1
	.word	.LANCHOR1+3100
	.word	.LANCHOR0+44
	.word	.LANCHOR1+256
	.fnend
	.size	FlashLoadPhyInfoInRam, .-FlashLoadPhyInfoInRam
	.align	1
	.global	NandcCopy1KB
	.thumb
	.thumb_func
	.type	NandcCopy1KB, %function
NandcCopy1KB:
	.fnstart
	@ args = 4, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r1, #1
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r2
	add	r2, r0, #4096
	ldr	r6, [sp, #16]
	add	r5, r0, #512
	add	r2, r2, r4, lsl #9
	bne	.L1120
	cbz	r3, .L1121
	mov	r0, r2
	mov	r1, r3
	mov	r2, #1024
	bl	ftl_memcpy
.L1121:
	cbz	r6, .L1119
	ldrb	r3, [r6, #2]	@ zero_extendqisi2
	lsrs	r4, r4, #1
	ldrb	r2, [r6, #1]	@ zero_extendqisi2
	add	r4, r4, r4, lsl #1
	lsls	r3, r3, #16
	orr	r2, r3, r2, lsl #8
	ldrb	r3, [r6]	@ zero_extendqisi2
	lsls	r4, r4, #2
	orrs	r3, r3, r2
	ldrb	r2, [r6, #3]	@ zero_extendqisi2
	orr	r3, r3, r2, lsl #24
	str	r3, [r5, r4, lsl #2]
	pop	{r4, r5, r6, pc}
.L1120:
	cbz	r3, .L1124
	mov	r1, r2
	mov	r0, r3
	mov	r2, #1024
	bl	ftl_memcpy
.L1124:
	cbz	r6, .L1119
	lsrs	r4, r4, #1
	add	r4, r4, r4, lsl #1
	lsls	r4, r4, #2
	ldr	r3, [r5, r4, lsl #2]
	lsrs	r2, r3, #8
	strb	r3, [r6]
	strb	r2, [r6, #1]
	lsrs	r2, r3, #16
	lsrs	r3, r3, #24
	strb	r2, [r6, #2]
	strb	r3, [r6, #3]
.L1119:
	pop	{r4, r5, r6, pc}
	.fnend
	.size	NandcCopy1KB, .-NandcCopy1KB
	.align	1
	.global	ftl_memcpy32
	.thumb
	.thumb_func
	.type	ftl_memcpy32, %function
ftl_memcpy32:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	movs	r3, #0
	push	{r4, lr}
	.save {r4, lr}
.L1136:
	cmp	r3, r2
	beq	.L1138
	ldr	r4, [r1, r3, lsl #2]
	str	r4, [r0, r3, lsl #2]
	adds	r3, r3, #1
	b	.L1136
.L1138:
	pop	{r4, pc}
	.fnend
	.size	ftl_memcpy32, .-ftl_memcpy32
	.align	1
	.global	ftl_memcmp
	.thumb
	.thumb_func
	.type	ftl_memcmp, %function
ftl_memcmp:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	memcmp
	.fnend
	.size	ftl_memcmp, .-ftl_memcmp
	.align	1
	.global	timer_get_time
	.thumb
	.thumb_func
	.type	timer_get_time, %function
timer_get_time:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1141
	ldr	r0, [r3]
	b	jiffies_to_msecs
.L1142:
	.align	2
.L1141:
	.word	jiffies
	.fnend
	.size	timer_get_time, .-timer_get_time
	.align	1
	.global	FlashSramLoadStore
	.thumb
	.thumb_func
	.type	FlashSramLoadStore, %function
FlashSramLoadStore:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, lr}
	.save {r4, r5, lr}
	mov	r5, r0
	ldr	r4, .L1146
	ldr	r4, [r4, #3988]
	add	r4, r4, #4096
	add	r1, r1, r4
	cbz	r2, .L1145
	mov	r0, r1
	mov	r1, r5
.L1145:
	mov	r2, r3
	pop	{r4, r5, lr}
	b	ftl_memcpy
.L1147:
	.align	2
.L1146:
	.word	.LANCHOR2
	.fnend
	.size	FlashSramLoadStore, .-FlashSramLoadStore
	.align	1
	.global	FlashCs123Init
	.thumb
	.thumb_func
	.type	FlashCs123Init, %function
FlashCs123Init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	bx	lr
	.fnend
	.size	FlashCs123Init, .-FlashCs123Init
	.align	1
	.global	rk_nand_suspend
	.thumb
	.thumb_func
	.type	rk_nand_suspend, %function
rk_nand_suspend:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	ftl_flash_suspend
	.fnend
	.size	rk_nand_suspend, .-rk_nand_suspend
	.align	1
	.global	rk_nand_resume
	.thumb
	.thumb_func
	.type	rk_nand_resume, %function
rk_nand_resume:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	ftl_flash_resume
	.fnend
	.size	rk_nand_resume, .-rk_nand_resume
	.align	1
	.global	rk_ftl_get_capacity
	.thumb
	.thumb_func
	.type	rk_ftl_get_capacity, %function
rk_ftl_get_capacity:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1152
	ldr	r0, [r3, #1892]
	bx	lr
.L1153:
	.align	2
.L1152:
	.word	.LANCHOR0
	.fnend
	.size	rk_ftl_get_capacity, .-rk_ftl_get_capacity
	.align	1
	.global	rk_nandc_get_irq_status
	.thumb
	.thumb_func
	.type	rk_nandc_get_irq_status, %function
rk_nandc_get_irq_status:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r0, [r0, #372]
	bx	lr
	.fnend
	.size	rk_nandc_get_irq_status, .-rk_nandc_get_irq_status
	.align	1
	.global	rknand_proc_ftlread
	.thumb
	.thumb_func
	.type	rknand_proc_ftlread, %function
rknand_proc_ftlread:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	ftl_proc_ftl_read
	.fnend
	.size	rknand_proc_ftlread, .-rknand_proc_ftlread
	.align	1
	.global	ReadFlashInfo
	.thumb
	.thumb_func
	.type	ReadFlashInfo, %function
ReadFlashInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	ftl_read_flash_info
	.fnend
	.size	ReadFlashInfo, .-ReadFlashInfo
	.align	1
	.global	rknand_print_hex
	.thumb
	.thumb_func
	.type	rknand_print_hex, %function
rknand_print_hex:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	movs	r5, #0
	mov	r9, r0
	mov	r6, r1
	mov	r7, r2
	mov	r8, r3
	mov	r4, r5
.L1158:
	cmp	r4, r8
	beq	.L1166
	cbnz	r5, .L1159
	ldr	r0, .L1167
	mov	r1, r9
	mov	r2, r6
	mov	r3, r4
	bl	printk
.L1159:
	cmp	r7, #4
	ldr	r0, .L1167+4
	bne	.L1160
	ldr	r1, [r6, r4, lsl #2]
	b	.L1165
.L1160:
	cmp	r7, #2
	ite	eq
	ldrsheq	r1, [r6, r4, lsl #1]
	ldrbne	r1, [r6, r4]	@ zero_extendqisi2
.L1165:
	adds	r5, r5, #1
	bl	printk
	cmp	r5, #15
	bls	.L1163
	ldr	r0, .L1167+8
	movs	r5, #0
	ldr	r1, .L1167+12
	bl	printk
.L1163:
	adds	r4, r4, #1
	b	.L1158
.L1166:
	ldr	r0, .L1167+8
	ldr	r1, .L1167+12
	pop	{r3, r4, r5, r6, r7, r8, r9, lr}
	b	printk
.L1168:
	.align	2
.L1167:
	.word	.LC82
	.word	.LC83
	.word	.LC76
	.word	.LC84
	.fnend
	.size	rknand_print_hex, .-rknand_print_hex
	.align	1
	.global	HynixGetReadRetryDefault
	.thumb
	.thumb_func
	.type	HynixGetReadRetryDefault, %function
HynixGetReadRetryDefault:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	cmp	r0, #2
	ldr	r7, .L1278
	mov	r3, #172
	mov	r1, #173
	mov	r2, #174
	.pad #44
	sub	sp, sp, #44
	mov	r4, r0
	strb	r3, [r7, #1906]
	mov	r3, #175
	strb	r0, [r7, #1902]
	strb	r1, [r7, #1907]
	strb	r2, [r7, #1908]
	strb	r3, [r7, #1909]
	bne	.L1170
	movs	r3, #167
	strb	r3, [r7, #1906]
	ldr	r3, .L1278+4
	movs	r2, #247
	movs	r5, #7
	strb	r2, [r3, #3245]
	b	.L1231
.L1170:
	cmp	r0, #3
	bne	.L1172
	movs	r3, #176
	strb	r3, [r7, #1906]
	movs	r3, #177
	strb	r3, [r7, #1907]
	movs	r3, #178
	strb	r3, [r7, #1908]
	movs	r3, #179
	strb	r3, [r7, #1909]
	movs	r3, #180
	strb	r3, [r7, #1910]
	movs	r3, #181
	strb	r3, [r7, #1911]
	movs	r3, #182
	strb	r3, [r7, #1912]
	movs	r3, #183
	b	.L1272
.L1172:
	cmp	r0, #4
	bne	.L1173
	movs	r0, #204
	strb	r1, [r7, #1911]
	strb	r0, [r7, #1906]
	movs	r0, #191
	strb	r2, [r7, #1912]
	strb	r0, [r7, #1907]
	movs	r0, #170
	strb	r0, [r7, #1908]
	movs	r0, #171
	strb	r0, [r7, #1909]
	movs	r0, #205
	strb	r0, [r7, #1910]
.L1272:
	movs	r5, #8
	strb	r3, [r7, #1913]
	mov	r6, r5
	b	.L1171
.L1173:
	cmp	r0, #5
	bne	.L1174
	movs	r3, #56
	movs	r5, #8
	strb	r3, [r7, #1906]
	movs	r3, #57
	strb	r3, [r7, #1907]
	movs	r3, #58
	strb	r3, [r7, #1908]
	movs	r3, #59
	strb	r3, [r7, #1909]
	b	.L1231
.L1174:
	cmp	r0, #6
	bne	.L1175
	movs	r3, #14
	movs	r5, #12
	strb	r3, [r7, #1906]
	movs	r3, #15
	strb	r3, [r7, #1907]
	movs	r3, #16
	strb	r3, [r7, #1908]
	movs	r3, #17
	strb	r3, [r7, #1909]
	b	.L1231
.L1175:
	cmp	r0, #7
	bne	.L1176
	movs	r3, #176
	movs	r5, #12
	strb	r3, [r7, #1906]
	movs	r6, #10
	movs	r3, #177
	strb	r3, [r7, #1907]
	movs	r3, #178
	strb	r3, [r7, #1908]
	movs	r3, #179
	strb	r3, [r7, #1909]
	movs	r3, #180
	strb	r3, [r7, #1910]
	movs	r3, #181
	strb	r3, [r7, #1911]
	movs	r3, #182
	strb	r3, [r7, #1912]
	movs	r3, #183
	strb	r3, [r7, #1913]
	movs	r3, #212
	strb	r3, [r7, #1914]
	movs	r3, #213
	strb	r3, [r7, #1915]
	b	.L1171
.L1176:
	cmp	r0, #8
	mov	r5, #7
	bne	.L1231
	movs	r3, #6
	strb	r5, [r7, #1907]
	strb	r3, [r7, #1906]
	movs	r5, #50
	movs	r3, #9
	strb	r0, [r7, #1908]
	strb	r3, [r7, #1909]
	movs	r6, #5
	movs	r3, #10
	strb	r3, [r7, #1910]
	b	.L1171
.L1231:
	movs	r6, #4
.L1171:
	subs	r3, r4, #1
	cmp	r3, #1
	bhi	.L1269
	ldr	r1, .L1278+8
	mov	r10, #0
.L1177:
	ldrb	r3, [r7, #1612]	@ zero_extendqisi2
	uxtb	r8, r10
	ldr	r2, .L1278
	cmp	r3, r8
	bls	.L1184
	add	r8, r8, r2
	addw	fp, r2, #1905
	ldrb	r3, [r8, #1616]	@ zero_extendqisi2
	add	r4, r2, r3, lsl #6
	add	r3, r2, r3, lsl #3
	addw	r4, r4, #1922
	movs	r2, #55
	ldrb	r8, [r3, #16]	@ zero_extendqisi2
	ldr	r0, [r3, #12]
	mov	r9, r4
	mov	r3, r4
	add	r8, r0, r8, lsl #8
	addw	ip, r8, #2056
.L1179:
	str	r2, [ip]
	ldrb	r0, [fp, #1]!	@ zero_extendqisi2
	str	r1, [sp, #16]
	str	r3, [sp, #12]
	str	r0, [r8, #2052]
	movs	r0, #80
	str	r2, [sp, #8]
	str	ip, [sp, #4]
	bl	NandcDelayns
	ldr	r0, [r8, #2048]
	ldr	r3, [sp, #12]
	ldr	ip, [sp, #4]
	ldr	r2, [sp, #8]
	strb	r0, [r3], #1
	subs	r0, r3, r4
	ldr	r1, [sp, #16]
	uxtb	r0, r0
	cmp	r0, r6
	bcc	.L1179
	movs	r2, #0
.L1180:
	movs	r3, #1
	add	lr, r1, r2
.L1181:
	ldrb	ip, [lr, r3, lsl #2]	@ zero_extendqisi2
	ldrb	r0, [r9]	@ zero_extendqisi2
	add	r0, r0, ip
	strb	r0, [r9, r3, lsl #3]
	adds	r3, r3, #1
	cmp	r3, #7
	bne	.L1181
	adds	r2, r2, #1
	add	r9, r9, #1
	cmp	r2, #4
	bne	.L1180
	movs	r3, #0
	add	r10, r10, #1
	strb	r3, [r4, #16]
	strb	r3, [r4, #24]
	strb	r3, [r4, #32]
	strb	r3, [r4, #40]
	strb	r3, [r4, #48]
	strb	r3, [r4, #41]
	strb	r3, [r4, #49]
	b	.L1177
.L1269:
	subs	r3, r4, #3
	cmp	r3, #5
	bhi	.L1184
	smulbb	r2, r6, r5
	lsls	r3, r2, #4
	asrs	r2, r2, #1
	str	r3, [sp, #36]
	lsls	r3, r2, #1
	str	r3, [sp, #8]
	movs	r3, #0
.L1276:
	str	r3, [sp, #20]
	ldrb	r3, [sp, #20]	@ zero_extendqisi2
	str	r3, [sp, #16]
	ldr	r3, .L1278
	ldr	r2, [sp, #16]
	ldrb	r3, [r3, #1612]	@ zero_extendqisi2
	cmp	r3, r2
	bhi	.L1229
.L1184:
	ldr	r3, .L1278
	strb	r6, [r3, #1903]
	strb	r5, [r3, #1904]
	add	sp, sp, #44
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1229:
	ldr	r2, [sp, #16]
	ldr	r3, .L1278
	add	r3, r3, r2
	ldrb	fp, [r3, #1616]	@ zero_extendqisi2
	ldr	r3, .L1278
	add	r3, r3, fp, lsl #3
	mov	r0, fp
	ldr	r10, [r3, #12]
	ldrb	r3, [r3, #16]	@ zero_extendqisi2
	lsls	r7, r3, #8
	str	r3, [sp, #4]
	add	r8, r10, r7
	movs	r3, #255
	str	r3, [r8, #2056]
	bl	NandcWaitFlashReady
	cmp	r4, #7
	bne	.L1186
	ldr	r2, .L1278+12
	movs	r3, #160
	mla	ip, r3, fp, r2
	add	r3, ip, #28
	b	.L1273
.L1186:
	cmp	r4, #8
	beq	.L1188
	ldr	r2, .L1278+12
	add	ip, r2, fp, lsl #6
	add	r3, ip, #20
.L1273:
	add	r7, r7, r10
	cmp	r4, #4
	str	r3, [sp, #24]
	mov	r3, #54
	str	r3, [r7, #2056]
	bne	.L1189
	movs	r3, #255
	str	r3, [r7, #2052]
	movs	r3, #64
	str	r3, [r7, #2048]
	movs	r3, #204
	b	.L1274
.L1189:
	subs	r3, r4, #5
	cmp	r3, #1
	bhi	.L1191
	ldr	r3, .L1278
	ldrb	r2, [r3, #1906]	@ zero_extendqisi2
	ldr	r3, [sp, #4]
	add	r3, r10, r3, lsl #8
	str	r2, [r3, #2052]
	movs	r2, #82
	str	r2, [r3, #2048]
	b	.L1190
.L1191:
	cmp	r4, #7
	bne	.L1190
	movs	r3, #174
	str	r3, [r7, #2052]
	movs	r3, #0
	str	r3, [r7, #2048]
	movs	r3, #176
.L1274:
	str	r3, [r7, #2052]
	movs	r3, #77
	str	r3, [r7, #2048]
.L1190:
	ldr	r3, [sp, #4]
	cmp	r4, #6
	mov	r2, #22
	add	r3, r10, r3, lsl #8
	str	r2, [r3, #2056]
	mov	r2, #23
	str	r2, [r3, #2056]
	mov	r2, #4
	str	r2, [r3, #2056]
	mov	r2, #25
	str	r2, [r3, #2056]
	mov	r2, #0
	str	r2, [r3, #2056]
	str	r2, [r3, #2052]
	str	r2, [r3, #2052]
	it	eq
	moveq	r2, #31
	str	r2, [r3, #2052]
	movs	r2, #2
	str	r2, [r3, #2052]
	movs	r2, #0
	str	r2, [r3, #2052]
.L1230:
	ldr	r3, [sp, #4]
	movs	r2, #48
	mov	r0, fp
	add	r3, r10, r3, lsl #8
	str	r2, [r3, #2056]
	bl	NandcWaitFlashReady
	subs	r3, r4, #5
	cmp	r3, #1
	str	r3, [sp, #28]
	bls	.L1234
	cmp	r4, #8
	beq	.L1234
	cmp	r4, #7
	ite	ne
	movne	r2, #2
	moveq	r2, #32
	b	.L1194
.L1279:
	.align	2
.L1278:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR1+3228
	.word	.LANCHOR0+1902
.L1234:
	movs	r2, #16
.L1194:
	ldr	r3, .L1280
	ldr	r1, [sp, #4]
	ldr	r3, [r3, #3992]
	add	r0, r10, r1, lsl #8
	mov	r7, r3
.L1195:
	ldr	r1, [r0, #2048]
	strb	r1, [r7], #1
	subs	r1, r7, r3
	uxtb	r1, r1
	cmp	r1, r2
	bcc	.L1195
	cmp	r4, #8
	bne	.L1196
	movs	r2, #0
.L1198:
	ldrb	r7, [r3, r2, lsl #2]	@ zero_extendqisi2
	uxtb	r1, r2
	cmp	r7, #50
	beq	.L1197
	add	r7, r3, r2, lsl #2
	ldrb	r7, [r7, #1]	@ zero_extendqisi2
	cmp	r7, #5
	beq	.L1197
	adds	r2, r2, #1
	cmp	r2, #8
	bne	.L1198
	b	.L1199
.L1197:
	cmp	r1, #7
	bne	.L1200
.L1199:
	ldr	r0, .L1280+4
	movs	r1, #0
	bl	printk
.L1201:
	b	.L1201
.L1196:
	cmp	r4, #7
	bne	.L1202
	movs	r2, #0
.L1204:
	ldrb	r7, [r3, r2, lsl #2]	@ zero_extendqisi2
	uxtb	r1, r2
	cmp	r7, #12
	beq	.L1203
	add	r7, r3, r2, lsl #2
	ldrb	r7, [r7, #1]	@ zero_extendqisi2
	cmp	r7, #10
	beq	.L1203
	adds	r2, r2, #1
	cmp	r2, #8
	bne	.L1204
	b	.L1205
.L1203:
	cmp	r1, #7
	bne	.L1200
.L1205:
	ldr	r0, .L1280+4
	movs	r1, #0
	bl	printk
.L1206:
	b	.L1206
.L1202:
	cmp	r4, #6
	bne	.L1200
	subs	r2, r3, #1
	adds	r3, r3, #7
.L1207:
	ldrb	r1, [r2, #1]!	@ zero_extendqisi2
	cmp	r1, #12
	beq	.L1200
	ldrb	r1, [r2, #8]	@ zero_extendqisi2
	cmp	r1, #4
	beq	.L1200
	cmp	r2, r3
	bne	.L1207
	ldr	r0, .L1280+4
	movs	r1, #0
	bl	printk
.L1209:
	b	.L1209
.L1200:
	ldr	r1, .L1280
	ldr	r7, [sp, #36]
	ldr	r2, [r1, #3992]
	adds	r7, r2, r7
	mov	r3, r2
.L1210:
	cmp	r3, r7
	beq	.L1277
	ldr	lr, [r0, #2048]
	strb	lr, [r3], #1
	b	.L1210
.L1277:
	ldr	r3, [r1, #3992]
	movs	r0, #8
	ldr	r1, [sp, #8]
	adds	r1, r3, r1
	str	r1, [sp, #32]
.L1213:
	ldr	r7, [sp, #8]
	add	lr, r1, r7
.L1212:
	ldrh	r7, [r1]
	mvns	r7, r7
	strh	r7, [r1], #2	@ movhi
	cmp	r1, lr
	bne	.L1212
	ldr	r7, [sp, #8]
	subs	r0, r0, #1
	add	r1, r1, r7
	bne	.L1213
	str	r3, [sp, #12]
.L1214:
	movs	r7, #0
	mov	r0, r7
.L1217:
	movs	r3, #1
	mov	lr, #0
	mov	r8, #16
	lsl	r1, r3, r0
	mov	ip, lr
.L1215:
	ldr	r3, [sp, #12]
	ldrh	r9, [r3, lr]
	ldr	r3, [sp, #8]
	and	r9, r9, r1
	cmp	r9, r1
	it	eq
	addeq	ip, ip, #1
	subs	r8, r8, #1
	add	lr, lr, r3
	bne	.L1215
	cmp	ip, #8
	add	r0, r0, #1
	itt	hi
	orrhi	r7, r7, r1
	uxthhi	r7, r7
	cmp	r0, #16
	bne	.L1217
	ldr	r3, [sp, #12]
	strh	r7, [r3], #2	@ movhi
	str	r3, [sp, #12]
	ldr	r1, [sp, #12]
	ldr	r3, [sp, #32]
	cmp	r1, r3
	bne	.L1214
	ldr	r3, .L1280
	ldr	r1, [r3, #3992]
	movs	r3, #0
	subs	r0, r1, #4
	add	r7, r1, #28
.L1219:
	ldr	lr, [r0, #4]!
	cmp	lr, #0
	it	eq
	addeq	r3, r3, #1
	cmp	r0, r7
	bne	.L1219
	cmp	r3, #7
	ble	.L1220
	ldr	r0, .L1280+8
	movs	r2, #1
	mov	r3, #1024
	bl	rknand_print_hex
	ldr	r0, .L1280+4
	movs	r1, #0
	bl	printk
.L1221:
	b	.L1221
.L1220:
	cmp	r4, #6
	beq	.L1236
	cmp	r4, #7
	beq	.L1237
	cmp	r4, #8
	ite	ne
	movne	r0, #8
	moveq	r0, #5
	b	.L1222
.L1236:
	movs	r0, #4
	b	.L1222
.L1237:
	movs	r0, #10
.L1222:
	add	r9, r6, #-1
	ldr	r1, [sp, #24]
	movs	r7, #0
	uxtb	r9, r9
	add	r9, r9, #1
.L1223:
	mov	ip, r1
	mov	r3, r2
.L1224:
	ldrb	lr, [r3], #1	@ zero_extendqisi2
	strb	lr, [ip], #1
	rsb	lr, r2, r3
	uxtb	lr, lr
	cmp	lr, r6
	bcc	.L1224
	adds	r7, r7, #1
	add	r2, r2, r9
	cmp	r7, r5
	add	r1, r1, r0
	blt	.L1223
	ldr	r3, [sp, #4]
	mov	r0, fp
	add	r7, r10, r3, lsl #8
	movs	r3, #255
	str	r3, [r7, #2056]
	bl	NandcWaitFlashReady
	ldr	r3, [sp, #28]
	cmp	r3, #1
	bhi	.L1226
	movs	r3, #54
	str	r3, [r7, #2056]
	ldr	r3, .L1280+12
	mov	r1, #-1
	ldr	r0, [sp, #16]
	ldrb	r3, [r3, #1906]	@ zero_extendqisi2
	str	r3, [r7, #2052]
	movs	r3, #0
	str	r3, [r7, #2048]
	movs	r3, #22
	str	r3, [r7, #2056]
	bl	FlashReadCmd
	b	.L1227
.L1226:
	cmp	r4, #8
	ite	eq
	moveq	r3, #190
	movne	r3, #56
	str	r3, [r7, #2056]
.L1227:
	mov	r0, fp
	bl	NandcWaitFlashReady
	ldr	r3, [sp, #20]
	adds	r3, r3, #1
	b	.L1276
.L1188:
	movs	r3, #120
	movs	r2, #23
	str	r3, [r8, #2056]
	movs	r1, #25
	movs	r3, #0
	str	r3, [r8, #2052]
	str	r3, [r8, #2052]
	str	r3, [r8, #2052]
	str	r2, [r8, #2056]
	movs	r2, #4
	str	r2, [r8, #2056]
	str	r1, [r8, #2056]
	movs	r1, #218
	str	r1, [r8, #2056]
	movs	r1, #21
	str	r3, [r8, #2056]
	str	r3, [r8, #2052]
	str	r3, [r8, #2052]
	str	r1, [r8, #2052]
	str	r2, [r8, #2052]
	str	r3, [r8, #2052]
	ldr	r3, .L1280+16
	str	r3, [sp, #24]
	b	.L1230
.L1281:
	.align	2
.L1280:
	.word	.LANCHOR2
	.word	.LC85
	.word	.LC86
	.word	.LANCHOR0
	.word	.LANCHOR0+1930
	.fnend
	.size	HynixGetReadRetryDefault, .-HynixGetReadRetryDefault
	.align	1
	.global	FlashGetReadRetryDefault
	.thumb
	.thumb_func
	.type	FlashGetReadRetryDefault, %function
FlashGetReadRetryDefault:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r3, r0
	cmp	r0, #0
	beq	.L1282
	subs	r2, r0, #1
	cmp	r2, #7
	bhi	.L1284
	b	HynixGetReadRetryDefault
.L1284:
	cmp	r0, #49
	bne	.L1285
	ldr	r0, .L1305
	movs	r2, #64
	ldr	r1, .L1305+4
	addw	r0, r0, #1906
	strb	r3, [r0, #-4]
	movs	r3, #4
	strb	r3, [r0, #-3]
	movs	r3, #15
	strb	r3, [r0, #-2]
	b	.L1303
.L1285:
	cmp	r0, #33
	beq	.L1286
	sub	r2, r0, #65
	cmp	r2, #1
	bhi	.L1287
.L1286:
	ldr	r0, .L1305
	strb	r3, [r0, #1902]
	movs	r3, #4
	b	.L1304
.L1287:
	cmp	r0, #34
	beq	.L1288
	cmp	r0, #67
	bne	.L1289
.L1288:
	ldr	r0, .L1305
	strb	r3, [r0, #1902]
	movs	r3, #5
.L1304:
	strb	r3, [r0, #1903]
	movs	r2, #45
	movs	r3, #7
	addw	r0, r0, #1906
	strb	r3, [r0, #-2]
	ldr	r1, .L1305+8
	b	.L1303
.L1289:
	cmp	r0, #35
	beq	.L1290
	cmp	r0, #68
	bne	.L1282
.L1290:
	ldr	r0, .L1305
	movs	r2, #95
	ldr	r1, .L1305+12
	addw	r0, r0, #1906
	strb	r3, [r0, #-4]
	movs	r3, #5
	strb	r3, [r0, #-3]
	movs	r3, #17
	strb	r3, [r0, #-2]
.L1303:
	b	ftl_memcpy
.L1282:
	bx	lr
.L1306:
	.align	2
.L1305:
	.word	.LANCHOR0
	.word	.LANCHOR1+440
	.word	.LANCHOR1+288
	.word	.LANCHOR1+336
	.fnend
	.size	FlashGetReadRetryDefault, .-FlashGetReadRetryDefault
	.align	1
	.global	NandcXferComp
	.thumb
	.thumb_func
	.type	NandcXferComp, %function
NandcXferComp:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	.pad #8
	ldr	r5, .L1346
	add	r0, r5, r0, lsl #3
	ldr	r3, [r5, #2776]
	ldr	r4, [r0, #12]
	cmp	r3, #3
	bls	.L1338
	ldr	r3, [r4, #16]
	lsls	r6, r3, #29
	bpl	.L1338
	mov	r0, r4
	bl	wait_for_nandc_xfer_completed
	ldr	r6, [r4, #16]
	ldr	r3, [r4, #8]
	ubfx	r6, r6, #1, #1
	str	r3, [sp]
	cmp	r6, #0
	beq	.L1320
	movs	r6, #0
.L1310:
	ldr	r2, [r4, #28]
	ldr	r3, [sp]
	ubfx	r2, r2, #16, #5
	ubfx	r3, r3, #22, #6
	cmp	r2, r3
	bge	.L1318
	ldr	r3, [r5, #2776]
	cmp	r3, #5
	bhi	.L1311
.L1314:
	adds	r6, r6, #1
	bic	r3, r6, #-16777216
	cmp	r3, #0
	bne	.L1310
	ldr	r2, [r4, #28]
	mov	r1, r6
	ldr	r3, [sp]
	ubfx	r2, r2, #16, #5
	ldr	r0, .L1346+4
	ubfx	r3, r3, #22, #6
	bl	printk
	ldr	r0, .L1346+8
	mov	r1, r4
	movs	r2, #4
	mov	r3, #512
	bl	rknand_print_hex
	b	.L1310
.L1311:
	ldr	r3, [r4]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	lsls	r0, r3, #18
	bpl	.L1314
	ldr	r3, [sp, #4]
	lsls	r1, r3, #14
	bpl	.L1314
.L1318:
	ldr	r3, [r5, #2812]
	ldr	r4, .L1346
	cmp	r3, #0
	beq	.L1319
	ldr	r1, [sp]
	movs	r2, #0
	ldr	r0, [r4, #2804]
	ubfx	r1, r1, #22, #5
	lsls	r1, r1, #10
	bl	rknand_dma_unmap_single
	ldr	r0, [r4, #2808]
	movs	r2, #0
	ldr	r1, [sp]
	ubfx	r1, r1, #22, #5
	lsls	r1, r1, #7
	bl	rknand_dma_unmap_single
	b	.L1319
.L1322:
	ldr	r3, [r4, #8]
	adds	r6, r6, #1
	str	r3, [sp]
	bic	r3, r6, #-16777216
	cbnz	r3, .L1320
	ldr	r2, [sp]
	mov	r1, r6
	ldr	r3, [r4, #28]
	ldr	r0, .L1346+12
	ubfx	r3, r3, #16, #5
	bl	printk
	ldr	r0, .L1346+8
	mov	r1, r4
	movs	r2, #4
	mov	r3, #512
	bl	rknand_print_hex
.L1320:
	ldr	r3, [sp]
	lsls	r2, r3, #11
	bpl	.L1322
	ldr	r3, [r5, #2820]
	cbz	r3, .L1323
	mov	r0, r4
	bl	NandcSendDumpDataStart
.L1323:
	ldr	r3, [r5, #2812]
	ldr	r6, .L1346
	cbz	r3, .L1324
	ldr	r1, [sp]
	movs	r2, #1
	ldr	r0, [r6, #2804]
	ubfx	r1, r1, #22, #5
	lsls	r1, r1, #10
	bl	rknand_dma_unmap_single
	ldr	r0, [r6, #2808]
	movs	r2, #1
	ldr	r1, [sp]
	ubfx	r1, r1, #22, #5
	lsls	r1, r1, #7
	bl	rknand_dma_unmap_single
.L1324:
	ldr	r3, [r5, #2820]
	cbz	r3, .L1319
	mov	r0, r4
	bl	NandcSendDumpDataDone
.L1319:
	movs	r3, #0
	str	r3, [r5, #2812]
	b	.L1307
.L1338:
	ldr	r3, [r4, #8]
	str	r3, [sp]
	ldr	r3, [sp]
	lsls	r3, r3, #11
	bpl	.L1338
.L1307:
	add	sp, sp, #8
	@ sp needed
	pop	{r4, r5, r6, pc}
.L1347:
	.align	2
.L1346:
	.word	.LANCHOR0
	.word	.LC87
	.word	.LC88
	.word	.LC89
	.fnend
	.size	NandcXferComp, .-NandcXferComp
	.align	1
	.global	NandcXferData
	.thumb
	.thumb_func
	.type	NandcXferData, %function
NandcXferData:
	.fnstart
	@ args = 4, pretend = 0, frame = 80
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r9, r3
	ldr	r5, .L1392
	tst	r9, #63
	.pad #92
	sub	sp, sp, #92
	mov	r8, r0
	add	r3, r5, r0, lsl #3
	mov	r10, r1
	mov	r6, r2
	ldr	r7, [r3, #12]
	bne	.L1349
	ldr	r3, [sp, #128]
	cbnz	r3, .L1350
	add	r0, sp, #24
	movs	r1, #255
	movs	r2, #64
	bl	memset
	add	r3, sp, #24
	str	r3, [sp, #128]
.L1350:
	ldr	r3, [sp, #128]
	mov	r0, r8
	str	r9, [sp]
	mov	r1, r10
	mov	r2, r6
	str	r3, [sp, #4]
	movs	r3, #0
	bl	NandcXferStart
	mov	r0, r8
	bl	NandcXferComp
	cmp	r10, #0
	bne	.L1376
	ldr	r3, [r5, #2824]
	lsrs	r4, r6, #1
	mov	r2, r10
	mov	r1, r10
	cmp	r3, #25
	ldr	r3, [sp, #128]
	ite	cc
	movcc	lr, #64
	movcs	lr, #128
.L1353:
	cmp	r1, r4
	add	r3, r3, #4
	add	ip, r2, lr
	bcs	.L1390
	ldr	r0, [r5, #2792]
	lsrs	r2, r2, #2
	adds	r1, r1, #1
	ldr	r2, [r0, r2, lsl #2]
	lsrs	r0, r2, #8
	strb	r2, [r3, #-4]
	strb	r0, [r3, #-3]
	lsrs	r0, r2, #16
	lsrs	r2, r2, #24
	strb	r0, [r3, #-2]
	strb	r2, [r3, #-1]
	mov	r2, ip
	b	.L1353
.L1390:
	movs	r2, #0
	ldr	r0, [r5, #2824]
	ldr	r1, [r5, #2776]
	lsrs	r6, r6, #2
	mov	fp, r2
.L1355:
	cmp	r2, r6
	bcs	.L1351
	cmp	r0, #0
	beq	.L1351
	add	r3, r2, #8
	ldr	r3, [r7, r3, lsl #2]
	str	r3, [sp, #20]
	ldr	r3, [sp, #20]
	lsls	r4, r3, #29
	bmi	.L1379
	ldr	r4, [sp, #20]
	ubfx	r4, r4, #15, #1
	cmp	r4, #0
	bne	.L1379
	cmp	r1, #5
	bls	.L1357
	ldr	r4, [sp, #20]
	ldr	r3, [sp, #20]
	ubfx	r4, r4, #3, #5
	ubfx	ip, r3, #27, #1
	ldr	r3, [sp, #20]
	ldr	lr, [sp, #20]
	ubfx	r3, r3, #16, #5
	orr	r4, r4, ip, lsl #5
	ubfx	lr, lr, #29, #1
	orr	r3, r3, lr, lsl #5
	cmp	r4, r3
	ldr	r3, [sp, #20]
	ittee	hi
	ldrhi	r4, [sp, #20]
	ubfxhi	r3, r3, #3, #5
	ldrls	r4, [sp, #20]
	ubfxls	r3, r3, #16, #5
	ite	hi
	ubfxhi	r4, r4, #27, #1
	ubfxls	r4, r4, #29, #1
	b	.L1389
.L1357:
	cmp	r1, #3
	bls	.L1359
	ldr	r4, [sp, #20]
	ldr	r3, [sp, #20]
	ubfx	r4, r4, #3, #5
	ubfx	ip, r3, #28, #1
	ldr	r3, [sp, #20]
	ldr	lr, [sp, #20]
	ubfx	r3, r3, #16, #5
	orr	r4, r4, ip, lsl #5
	ubfx	lr, lr, #30, #1
	orr	r3, r3, lr, lsl #5
	cmp	r4, r3
	ldr	r3, [sp, #20]
	ittee	hi
	ldrhi	r4, [sp, #20]
	ubfxhi	r3, r3, #3, #5
	ldrls	r4, [sp, #20]
	ubfxls	r3, r3, #16, #5
	ite	hi
	ubfxhi	r4, r4, #28, #1
	ubfxls	r4, r4, #30, #1
.L1389:
	orr	r4, r3, r4, lsl #5
.L1359:
	cmp	fp, r4
	it	cc
	movcc	fp, r4
	b	.L1356
.L1379:
	mov	fp, #-1
.L1356:
	adds	r2, r2, #1
	b	.L1355
.L1376:
	mov	fp, #0
.L1351:
	movs	r3, #0
	str	r3, [r7, #16]
	b	.L1362
.L1349:
	cmp	r1, #1
	mov	r4, #0
	bne	.L1388
	mov	fp, r4
.L1363:
	cmp	r4, r6
	bcs	.L1391
	and	ip, r4, #3
	cmp	r9, #0
	beq	.L1382
	add	r3, r9, r4, lsl #9
	b	.L1365
.L1382:
	mov	r3, r9
.L1365:
	ldr	r2, [sp, #128]
	mov	r0, r7
	ldr	r1, [sp, #128]
	cmp	r2, #0
	str	ip, [sp, #8]
	ite	ne
	movne	r2, #2
	moveq	r2, #0
	mla	r2, r4, r2, r1
	movs	r1, #1
	adds	r4, r4, #2
	str	r2, [sp]
	mov	r2, ip
	bl	NandcCopy1KB
	mov	r0, r8
	movs	r1, #1
	movs	r2, #2
	ldr	ip, [sp, #8]
	str	fp, [sp]
	str	fp, [sp, #4]
	mov	r3, ip
	bl	NandcXferStart
	mov	r0, r8
	bl	NandcXferComp
	b	.L1363
.L1391:
	mov	fp, #0
	b	.L1362
.L1388:
	str	r4, [sp]
	mov	r1, r4
	str	r4, [sp, #4]
	movs	r2, #2
	mov	r3, r4
	mov	fp, r4
	bl	NandcXferStart
	str	r9, [sp, #8]
.L1368:
	cmp	r4, r6
	bcs	.L1362
	mov	r0, r8
	bl	NandcXferComp
	ldr	r3, [r7, #32]
	add	ip, r4, #2
	cmp	ip, r6
	str	r3, [sp, #20]
	bcs	.L1369
	movs	r3, #0
	mov	r0, r8
	str	r3, [sp]
	movs	r2, #2
	str	r3, [sp, #4]
	mov	r1, r3
	and	r3, ip, #3
	str	ip, [sp, #12]
	bl	NandcXferStart
	ldr	ip, [sp, #12]
.L1369:
	ldr	r3, [sp, #20]
	lsls	r3, r3, #29
	bmi	.L1384
	ldr	r2, [sp, #20]
	ldr	r3, [sp, #20]
	ubfx	r2, r2, #3, #5
	ubfx	r3, r3, #27, #1
	orr	r3, r2, r3, lsl #5
	cmp	fp, r3
	it	cc
	movcc	fp, r3
	b	.L1370
.L1384:
	mov	fp, #-1
.L1370:
	ldr	r1, [sp, #128]
	cmp	r9, #0
	ldr	r3, [sp, #8]
	it	eq
	moveq	r3, #0
	ldr	r0, [sp, #128]
	cmp	r1, #0
	ite	ne
	movne	r1, #2
	moveq	r1, #0
	sub	r2, ip, #2
	str	ip, [sp, #12]
	mla	r4, r4, r1, r0
	and	r2, r2, #3
	mov	r0, r7
	movs	r1, #0
	str	r4, [sp]
	bl	NandcCopy1KB
	ldr	ip, [sp, #12]
	ldr	r3, [sp, #8]
	mov	r4, ip
	add	r3, r3, #1024
	str	r3, [sp, #8]
	b	.L1368
.L1362:
	ldr	r3, [r5, #2776]
	cmp	r3, #5
	bls	.L1374
	cmp	r10, #0
	bne	.L1374
	ldr	r3, [r7]
	and	r2, r3, #139264
	cmp	r2, #139264
	ittt	eq
	moveq	fp, #-1
	orreq	r3, r3, #131072
	streq	r3, [r7]
.L1374:
	mov	r0, fp
	add	sp, sp, #92
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1393:
	.align	2
.L1392:
	.word	.LANCHOR0
	.fnend
	.size	NandcXferData, .-NandcXferData
	.align	1
	.global	FlashReadRawPage
	.thumb
	.thumb_func
	.type	FlashReadRawPage, %function
FlashReadRawPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #8
	mov	r8, r3
	ldr	r3, .L1396
	mov	r6, r1
	mov	r7, r2
	mov	r4, r0
	ldrb	r5, [r3, #265]	@ zero_extendqisi2
	cbnz	r0, .L1395
	ldr	r2, .L1396+4
	ldrb	r3, [r2, #1]	@ zero_extendqisi2
	ldr	r2, [r2, #4]
	muls	r2, r3, r2
	cmp	r1, r2
	it	cc
	movcc	r5, #4
.L1395:
	mov	r0, r4
	bl	NandcWaitFlashReady
	mov	r0, r4
	bl	NandcFlashCs
	mov	r1, r6
	mov	r0, r4
	bl	FlashReadCmd
	mov	r0, r4
	bl	NandcWaitFlashReady
	mov	r2, r5
	movs	r1, #0
	mov	r3, r7
	mov	r0, r4
	str	r8, [sp]
	bl	NandcXferData
	mov	r5, r0
	mov	r0, r4
	bl	NandcFlashDeCs
	mov	r0, r5
	add	sp, sp, #8
	@ sp needed
	pop	{r4, r5, r6, r7, r8, pc}
.L1397:
	.align	2
.L1396:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.fnend
	.size	FlashReadRawPage, .-FlashReadRawPage
	.align	1
	.global	FlashDdrTunningRead
	.thumb
	.thumb_func
	.type	FlashDdrTunningRead, %function
FlashDdrTunningRead:
	.fnstart
	@ args = 4, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r7, r3
	ldr	r4, .L1424
	.pad #20
	sub	sp, sp, #20
	mov	r10, r0
	mov	fp, r2
	str	r1, [sp]
	ldr	r3, [r4, #1824]
	ldr	r3, [r3, #304]
	str	r3, [sp, #8]
	ldr	r3, [r4, #2776]
	cmp	r3, #8
	ldr	r3, [sp, #56]
	ite	cc
	movcc	r9, #6
	movcs	r9, #12
	cmp	r3, #0
	beq	.L1412
	movs	r0, #1
	bl	FlashSetInterfaceMode
	movs	r0, #1
	bl	NandcSetMode
	mov	r0, r10
	bl	FlashReset
	mov	r3, r7
	mov	r2, fp
	mov	r0, r10
	ldr	r1, [sp]
	bl	FlashReadRawPage
	mov	r6, r0
	ldrb	r0, [r4, #2760]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #2760]	@ zero_extendqisi2
	bl	NandcSetMode
	adds	r3, r6, #1
	bne	.L1401
.L1410:
	mov	r6, #-1
	b	.L1402
.L1401:
	mov	r2, r6
	ldr	r0, .L1424+4
	ldr	r1, [sp]
	bl	printk
	cmp	r6, #9
	itt	ls
	addls	r4, r4, r10, lsl #3
	ldrls	r3, [r4, #12]
	ldr	r4, .L1424+8
	itttt	ls
	ldrls	r2, [r3, #3840]
	ldrls	r2, [r3]
	orrls	r2, r2, #131072
	strls	r2, [r3]
	ldr	r3, [r4, #3996]
	adds	r3, r3, #1
	str	r3, [r4, #3996]
	cmp	r3, #2048
	bcc	.L1402
	movs	r7, #0
	str	r7, [r4, #3996]
	mov	fp, r7
	b	.L1400
.L1412:
	mov	r6, #1024
.L1400:
	movs	r4, #0
	mov	r8, #-1
	str	r4, [sp, #4]
	mov	ip, r4
	mov	r5, r4
.L1408:
	uxtb	r0, r9
	str	ip, [sp, #12]
	bl	NandcSetDdrPara
	mov	r3, r7
	mov	r0, r10
	mov	r2, fp
	ldr	r1, [sp]
	bl	FlashReadRawPage
	adds	r3, r6, #1
	cmp	r0, r3
	ldr	ip, [sp, #12]
	bhi	.L1404
	cmp	r0, #2
	bhi	.L1414
	adds	r5, r5, #1
	cmp	r5, #9
	bls	.L1414
	rsb	r4, r5, r9
	mov	r6, r0
	mov	r8, #0
	b	.L1406
.L1404:
	ldr	r3, [sp, #4]
	cmp	r3, r5
	bcs	.L1415
	cmp	r5, #7
	rsb	ip, r5, r4
	bhi	.L1416
	str	r5, [sp, #4]
	b	.L1415
.L1414:
	mov	r8, #0
	mov	r4, r9
	mov	r6, r0
	mov	r7, r8
	mov	fp, r8
	b	.L1405
.L1415:
	movs	r5, #0
.L1405:
	add	r9, r9, #2
	cmp	r9, #69
	bls	.L1408
.L1406:
	ldr	r3, [sp, #4]
	cmp	r3, r5
	it	cs
	movcs	r4, ip
	b	.L1407
.L1416:
	mov	r4, ip
.L1407:
	cbz	r4, .L1409
	ldr	r0, .L1424+12
	mov	r1, r4
	bl	printk
	uxtb	r0, r4
	bl	NandcSetDdrPara
.L1409:
	cmp	r8, #0
	beq	.L1402
	ldr	r0, .L1424+16
	mov	r1, r10
	ldr	r2, [sp]
	bl	printk
	ldr	r3, [sp, #56]
	cmp	r3, #0
	beq	.L1410
	ldr	r3, [sp, #8]
	ubfx	r0, r3, #8, #8
	bl	NandcSetDdrPara
.L1402:
	mov	r0, r6
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1425:
	.align	2
.L1424:
	.word	.LANCHOR0
	.word	.LC90
	.word	.LANCHOR2
	.word	.LC91
	.word	.LC92
	.fnend
	.size	FlashDdrTunningRead, .-FlashDdrTunningRead
	.align	1
	.global	FlashReadPage
	.thumb
	.thumb_func
	.type	FlashReadPage, %function
FlashReadPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r8, r2
	mov	r5, r0
	mov	r6, r1
	mov	r7, r3
	bl	FlashReadRawPage
	adds	r2, r0, #1
	mov	r4, r0
	bne	.L1427
	ldr	r9, .L1446+4
	ldrb	fp, [r9, #8]	@ zero_extendqisi2
	mov	r10, r9
	cmp	fp, #0
	bne	.L1428
.L1430:
	ldrb	r3, [r10, #2768]	@ zero_extendqisi2
	ldr	r9, .L1446+4
	cbz	r3, .L1427
	b	.L1445
.L1428:
	movs	r3, #0
	mov	r0, r5
	strb	r3, [r9, #8]
	mov	r1, r6
	mov	r3, r7
	mov	r2, r8
	bl	FlashReadRawPage
	strb	fp, [r9, #8]
	adds	r3, r0, #1
	beq	.L1430
	mov	r4, r0
	b	.L1427
.L1445:
	ldr	r3, [r9, #1824]
	mov	r1, r6
	mov	r0, r5
	mov	r2, r8
	ldr	r10, [r3, #304]
	movs	r3, #1
	str	r3, [sp]
	mov	r3, r7
	bl	FlashDdrTunningRead
	adds	r1, r0, #1
	mov	r4, r0
	beq	.L1431
	ldrb	r3, [r9, #1889]	@ zero_extendqisi2
	cmp	r0, r3, lsr #1
	bls	.L1427
.L1431:
	ubfx	r0, r10, #8, #8
	bl	NandcSetDdrPara
.L1427:
	ldr	r9, .L1446+8
	ldr	ip, [r9, #4000]
	cmp	ip, #0
	beq	.L1432
	adds	r2, r4, #1
	bne	.L1432
	mov	r1, r6
	mov	r2, r8
	mov	r3, r7
	mov	r0, r5
	blx	ip
	mov	r3, r6
	mov	r2, r5
	mov	r4, r0
	ldr	r0, .L1446
	mov	r1, r4
	bl	printk
	adds	r3, r4, #1
	bne	.L1432
	ldr	r3, .L1446+4
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cbz	r3, .L1432
	mov	r0, r5
	bl	flash_enter_slc_mode
	ldr	r4, [r9, #4000]
	mov	r0, r5
	mov	r1, r6
	mov	r2, r8
	mov	r3, r7
	blx	r4
	mov	r4, r0
	mov	r0, r5
	bl	flash_exit_slc_mode
.L1432:
	mov	r0, r4
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1447:
	.align	2
.L1446:
	.word	.LC93
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FlashReadPage, .-FlashReadPage
	.align	1
	.global	FlashDdrParaScan
	.thumb
	.thumb_func
	.type	FlashDdrParaScan, %function
FlashDdrParaScan:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #8
	mov	r7, r0
	ldr	r4, .L1458
	mov	r6, r1
	movs	r5, #0
	ldrb	r0, [r4, #2760]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #2760]	@ zero_extendqisi2
	bl	NandcSetMode
	mov	r1, r6
	mov	r2, r5
	mov	r3, r5
	mov	r0, r7
	str	r5, [sp]
	bl	FlashDdrTunningRead
	mov	r1, r6
	mov	r2, r5
	mov	r3, r5
	mov	r8, r0
	mov	r0, r7
	bl	FlashReadRawPage
	adds	r0, r0, #1
	beq	.L1449
	cmp	r8, #-1
	bne	.L1450
.L1449:
	ldrb	r3, [r4, #2760]	@ zero_extendqisi2
	lsls	r3, r3, #31
	bpl	.L1450
	movs	r0, #1
	bl	FlashSetInterfaceMode
	movs	r0, #1
	bl	NandcSetMode
	ldr	r3, .L1458
	movs	r2, #0
	strb	r2, [r3, #2768]
	b	.L1451
.L1450:
	movs	r3, #1
	strb	r3, [r4, #2768]
.L1451:
	movs	r0, #0
	add	sp, sp, #8
	@ sp needed
	pop	{r4, r5, r6, r7, r8, pc}
.L1459:
	.align	2
.L1458:
	.word	.LANCHOR0
	.fnend
	.size	FlashDdrParaScan, .-FlashDdrParaScan
	.align	1
	.global	FlashLoadPhyInfo
	.thumb
	.thumb_func
	.type	FlashLoadPhyInfo, %function
FlashLoadPhyInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r3, .L1475
	movs	r4, #0
	ldr	r7, .L1475+4
	mov	r9, #4
	ldr	r5, .L1475+8
	mov	r8, #-1
	ldr	r0, [r3]	@ unaligned
	ldr	r3, [r7, #4]
	str	r4, [r5, #4008]
	mov	r6, r5
	str	r0, [sp, #12]	@ unaligned
	mov	r0, r4
	str	r3, [sp, #4]
	ldr	r3, [r5, #3992]
	str	r3, [r5, #4004]
	bl	flash_enter_slc_mode
.L1461:
	add	fp, r4, #1
	mov	r10, #0
.L1463:
	add	r3, sp, #12
	ldrb	r0, [r3, r10]	@ zero_extendqisi2
	bl	FlashBchSel
	movs	r0, #0
	mov	r1, r4
	ldr	r2, [r5, #3992]
	mov	r3, r0
	bl	FlashReadRawPage
	adds	r0, r0, #1
	bne	.L1462
	movs	r0, #0
	mov	r1, fp
	ldr	r2, [r6, #3992]
	mov	r3, r0
	bl	FlashReadRawPage
	adds	r0, r0, #1
	bne	.L1462
	add	r10, r10, #1
	cmp	r10, #4
	beq	.L1464
	b	.L1463
.L1465:
	add	r0, fp, #12
	movw	r1, #2036
	bl	js_hash
	ldr	r3, [fp, #8]
	cmp	r3, r0
	bne	.L1471
	ldr	r8, .L1475+24
	add	r1, fp, #160
	movs	r2, #32
	add	r0, r8, #256
	bl	ftl_memcpy
	ldr	r1, [r6, #4004]
	movs	r2, #32
	ldr	r0, .L1475+12
	adds	r1, r1, #192
	bl	ftl_memcpy
	ldr	r1, [r6, #4004]
	mov	r2, #852
	ldr	r0, .L1475+16
	adds	r1, r1, #224
	bl	ftl_memcpy
	ldrh	r0, [r8, #266]
	bl	FlashBlockAlignInit
	ldr	r8, [r6, #4004]
	str	r4, [r6, #4008]
	mov	r0, r4
	ldr	r1, [r7, #4]
	ldr	r3, [r8, #1076]
	strb	r3, [r7, #2768]
	bl	__aeabi_uidiv
	adds	r0, r0, #1
	cmp	r0, #1
	itee	hi
	strhi	r0, [r6, #4012]
	movls	r3, #2
	strls	r3, [r6, #4012]
	ldrh	r3, [r8, #14]
	mov	r8, #0
	strb	r3, [r5, #4016]
.L1464:
	ldr	r3, [sp, #4]
	subs	r9, r9, #1
	add	r4, r4, r3
	bne	.L1461
	mov	r0, r9
.L1474:
	bl	flash_exit_slc_mode
	mov	r0, r8
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1462:
	ldr	fp, [r5, #4004]
	ldr	r2, .L1475+20
	ldr	r3, [fp]
	cmp	r3, r2
	bne	.L1464
	cmp	r8, #0
	bne	.L1465
	mov	r0, r4
	ldr	r1, [r7, #4]
	bl	__aeabi_uidiv
	ldr	r3, .L1475+8
	adds	r0, r0, #1
	str	r0, [r3, #4012]
	mov	r0, r8
	b	.L1474
.L1471:
	mov	r8, #-1
	b	.L1464
.L1476:
	.align	2
.L1475:
	.word	.LANCHOR3+11
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+44
	.word	.LANCHOR0+1902
	.word	1312902724
	.word	.LANCHOR1
	.fnend
	.size	FlashLoadPhyInfo, .-FlashLoadPhyInfo
	.align	1
	.global	ToshibaReadRetrial
	.thumb
	.thumb_func
	.type	ToshibaReadRetrial, %function
ToshibaReadRetrial:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #28
	sub	sp, sp, #28
	mov	r9, r0
	mov	fp, r3
	str	r2, [sp, #8]
	str	r1, [sp, #16]
	bl	NandcWaitFlashReady
	ldr	r4, .L1505
	add	r3, r4, r9, lsl #3
	ldrb	r2, [r3, #16]	@ zero_extendqisi2
	ldr	r7, [r3, #12]
	ldrb	r3, [r4, #1900]	@ zero_extendqisi2
	add	r6, r2, #8
	subs	r3, r3, #67
	add	r6, r7, r6, lsl #8
	cmp	r3, #1
	lsl	r3, r2, #8
	str	r3, [sp, #12]
	bls	.L1494
	ldrb	r5, [r4, #2768]	@ zero_extendqisi2
	cbz	r5, .L1495
	movs	r0, #0
	str	r2, [sp, #20]
	bl	NandcSetDdrMode
	movs	r3, #1
	ldr	r2, [sp, #20]
	str	r3, [sp, #4]
	b	.L1479
.L1495:
	str	r5, [sp, #4]
.L1479:
	ldr	r3, [sp, #12]
	movs	r1, #92
	adds	r3, r7, r3
	str	r1, [r3, #2056]
	movs	r1, #197
	str	r1, [r3, #2056]
	b	.L1478
.L1494:
	movs	r3, #0
	str	r3, [sp, #4]
.L1478:
	mov	r8, #1
	mov	r10, #-1
	lsls	r3, r2, #8
	str	r3, [sp, #20]
.L1480:
	ldr	r3, .L1505+4
	ldrb	r3, [r3, #4017]	@ zero_extendqisi2
	adds	r3, r3, #1
	cmp	r8, r3
	bcs	.L1504
	ldrb	r3, [r4, #1900]	@ zero_extendqisi2
	mov	r0, r6
	uxtb	r1, r8
	subs	r3, r3, #67
	cmp	r3, #1
	bhi	.L1481
	bl	SandiskSetRRPara
	b	.L1482
.L1481:
	bl	ToshibaSetRRPara
.L1482:
	ldrb	r3, [r4, #1900]	@ zero_extendqisi2
	cmp	r3, #34
	bne	.L1483
	ldr	r3, .L1505+4
	ldrb	r3, [r3, #4017]	@ zero_extendqisi2
	subs	r3, r3, #3
	cmp	r8, r3
	itttt	eq
	moveq	r2, #179
	ldreq	r3, [sp, #20]
	addeq	r3, r7, r3
	streq	r2, [r3, #2056]
.L1483:
	ldr	r3, [sp, #12]
	movs	r2, #38
	adds	r3, r7, r3
	str	r2, [r3, #2056]
	movs	r2, #93
	str	r2, [r3, #2056]
	ldr	r3, [sp, #4]
	cbz	r3, .L1484
	movs	r0, #4
	bl	NandcSetDdrMode
	mov	r0, r9
	mov	r3, fp
	ldr	r1, [sp, #16]
	ldr	r2, [sp, #8]
	bl	FlashReadRawPage
	mov	r5, r0
	movs	r0, #0
	bl	NandcSetDdrMode
	b	.L1485
.L1484:
	mov	r0, r9
	ldr	r1, [sp, #16]
	ldr	r2, [sp, #8]
	mov	r3, fp
	bl	FlashReadRawPage
	mov	r5, r0
.L1485:
	adds	r2, r5, #1
	beq	.L1486
	ldrb	r2, [r4, #1889]	@ zero_extendqisi2
	cmp	r10, #-1
	it	eq
	moveq	r10, r5
	add	r2, r2, r2, lsl #1
	cmp	r5, r2, asr #2
	bcc	.L1488
	mov	fp, #0
	str	fp, [sp, #8]
.L1486:
	add	r8, r8, #1
	b	.L1480
.L1504:
	mov	r5, r10
.L1488:
	ldrb	r2, [r4, #1900]	@ zero_extendqisi2
	mov	r0, r6
	movs	r1, #0
	subs	r2, r2, #67
	cmp	r2, #1
	bhi	.L1490
	bl	SandiskSetRRPara
	b	.L1491
.L1490:
	bl	ToshibaSetRRPara
.L1491:
	ldr	r3, [sp, #12]
	movs	r2, #255
	add	r7, r7, r3
	str	r2, [r7, #2056]
	ldrb	r2, [r4, #1889]	@ zero_extendqisi2
	add	r2, r2, r2, lsl #1
	cmp	r5, r2, asr #2
	bcc	.L1492
	adds	r3, r5, #1
	it	ne
	movne	r5, #256
.L1492:
	mov	r0, r9
	bl	NandcWaitFlashReady
	ldr	r3, [sp, #4]
	cbz	r3, .L1493
	movs	r0, #4
	bl	NandcSetDdrMode
.L1493:
	mov	r0, r5
	add	sp, sp, #28
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1506:
	.align	2
.L1505:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	ToshibaReadRetrial, .-ToshibaReadRetrial
	.align	1
	.global	SamsungReadRetrial
	.thumb
	.thumb_func
	.type	SamsungReadRetrial, %function
SamsungReadRetrial:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r8, r0
	ldr	r5, .L1520
	mov	r9, r2
	mov	r7, r3
	mov	fp, r1
	bl	NandcWaitFlashReady
	add	r2, r5, r8, lsl #3
	mov	r10, #1
	mov	r4, #-1
	ldrb	r3, [r2, #16]	@ zero_extendqisi2
	ldr	r6, [r2, #12]
	adds	r3, r3, #8
	add	r6, r6, r3, lsl #8
.L1508:
	ldr	r3, .L1520+4
	ldrb	r3, [r3, #4017]	@ zero_extendqisi2
	adds	r3, r3, #1
	cmp	r10, r3
	bcs	.L1511
	mov	r0, r6
	uxtb	r1, r10
	bl	SamsungSetRRPara
	mov	r2, r9
	mov	r0, r8
	mov	r1, fp
	mov	r3, r7
	bl	FlashReadRawPage
	adds	r2, r0, #1
	beq	.L1509
	ldrb	r3, [r5, #1889]	@ zero_extendqisi2
	cmp	r4, #-1
	it	eq
	moveq	r4, r0
	add	r3, r3, r3, lsl #1
	cmp	r0, r3, asr #2
	bcc	.L1514
	movs	r7, #0
	mov	r9, r7
.L1509:
	add	r10, r10, #1
	b	.L1508
.L1514:
	mov	r4, r0
.L1511:
	mov	r0, r6
	movs	r1, #0
	bl	SamsungSetRRPara
	ldrb	r3, [r5, #1889]	@ zero_extendqisi2
	add	r3, r3, r3, lsl #1
	cmp	r4, r3, asr #2
	bcc	.L1513
	adds	r3, r4, #1
	it	ne
	movne	r4, #256
.L1513:
	mov	r0, r4
	pop	{r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1521:
	.align	2
.L1520:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	SamsungReadRetrial, .-SamsungReadRetrial
	.align	1
	.global	MicronReadRetrial
	.thumb
	.thumb_func
	.type	MicronReadRetrial, %function
MicronReadRetrial:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r8, r3
	ldr	r3, .L1546
	.pad #36
	sub	sp, sp, #36
	mov	r6, r0
	mov	fp, r2
	str	r1, [sp, #20]
	ldrb	r5, [r3, #1889]	@ zero_extendqisi2
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cbnz	r3, .L1523
	add	r5, r5, r5, lsl #1
	ubfx	r5, r5, #2, #8
	b	.L1524
.L1523:
	ldr	r2, .L1546+4
	smull	r2, r3, r5, r2
	uxtb	r5, r3
.L1524:
	ldr	r3, .L1546
	mov	r10, #0
	add	r3, r3, r6, lsl #3
	str	r3, [sp, #24]
.L1534:
	mov	r0, r6
	mov	r9, #0
	bl	NandcWaitFlashReady
	mov	r4, #-1
	ldr	r3, [sp, #24]
	ldr	r3, [r3, #12]
	str	r3, [sp, #12]
	ldr	r3, [sp, #24]
	ldrb	r3, [r3, #16]	@ zero_extendqisi2
	str	r3, [sp, #16]
	ldr	r2, [sp, #16]
	ldr	r3, [sp, #12]
	add	r7, r3, r2, lsl #8
.L1525:
	ldr	r3, .L1546+8
	ldrb	r3, [r3, #4017]	@ zero_extendqisi2
	cmp	r9, r3
	bcs	.L1528
	movs	r3, #239
	movs	r0, #200
	str	r3, [r7, #2056]
	movs	r3, #137
	str	r3, [r7, #2052]
	bl	NandcDelayns
	mov	ip, #0
	add	r3, r9, #1
	mov	r0, r6
	str	r3, [r7, #2048]
	mov	r2, fp
	str	ip, [r7, #2048]
	str	ip, [r7, #2048]
	str	ip, [r7, #2048]
	str	r3, [sp, #8]
	mov	r3, r8
	ldr	r1, [sp, #20]
	str	ip, [sp, #28]
	bl	FlashReadRawPage
	adds	r3, r0, #1
	beq	.L1526
	cmp	r4, #-1
	it	eq
	moveq	r4, r0
	cmp	r0, r5
	ldr	ip, [sp, #28]
	bcc	.L1536
	mov	r8, ip
	mov	fp, ip
.L1526:
	ldr	r9, [sp, #8]
	b	.L1525
.L1536:
	mov	r4, r0
	mov	r8, ip
	mov	fp, ip
.L1528:
	ldr	r2, [sp, #16]
	movs	r0, #200
	ldr	r3, [sp, #12]
	movs	r7, #0
	add	r3, r3, r2, lsl #8
	movs	r2, #239
	str	r3, [sp, #8]
	str	r2, [r3, #2056]
	movs	r2, #137
	str	r2, [r3, #2052]
	bl	NandcDelayns
	cmp	r4, r5
	ldr	r3, [sp, #8]
	str	r7, [r3, #2048]
	str	r7, [r3, #2048]
	str	r7, [r3, #2048]
	str	r7, [r3, #2048]
	bcc	.L1530
	adds	r1, r4, #1
	ldr	r0, .L1546+12
	it	ne
	movne	r4, #256
	mov	r1, r9
	str	r4, [sp]
	mov	r3, r9
	ldr	r2, [sp, #20]
	bl	printk
	cmp	r10, #0
	bne	.L1532
	ldr	r3, .L1546
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cbz	r3, .L1541
	adds	r2, r4, #1
	bne	.L1541
	mov	r0, r6
	movs	r1, #3
	bl	micron_auto_read_calibration_config
	mov	r10, #1
	b	.L1534
.L1532:
	mov	r0, r6
	mov	r1, r7
	bl	micron_auto_read_calibration_config
	adds	r3, r4, #1
	it	ne
	movne	r4, #256
	b	.L1541
.L1530:
	cmp	r10, #0
	beq	.L1541
	mov	r0, r6
	mov	r1, r7
	bl	micron_auto_read_calibration_config
	mov	r4, #256
.L1541:
	mov	r0, r4
	add	sp, sp, #36
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1547:
	.align	2
.L1546:
	.word	.LANCHOR0
	.word	1431655766
	.word	.LANCHOR2
	.word	.LC94
	.fnend
	.size	MicronReadRetrial, .-MicronReadRetrial
	.align	1
	.global	HynixReadRetrial
	.thumb
	.thumb_func
	.type	HynixReadRetrial, %function
HynixReadRetrial:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r8, r3
	ldr	r5, .L1565
	mov	r10, r2
	mov	r7, r0
	mov	fp, r1
	adds	r2, r5, r0
	mov	r6, #-1
	ldr	r3, [r5, #1784]
	ldrb	r4, [r2, #1914]	@ zero_extendqisi2
	ldrb	r9, [r5, #1904]	@ zero_extendqisi2
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	subs	r3, r3, #7
	cmp	r3, #1
	it	ls
	ldrbls	r4, [r2, #1922]	@ zero_extendqisi2
	bl	NandcWaitFlashReady
	mov	ip, #0
.L1550:
	cmp	ip, r9
	bcs	.L1554
	adds	r4, r4, #1
	mov	r0, r7
	ldrb	r1, [r5, #1903]	@ zero_extendqisi2
	uxtb	r4, r4
	ldr	r2, .L1565+4
	cmp	r4, r9
	it	cs
	movcs	r4, #0
	str	ip, [sp, #4]
	mov	r3, r4
	bl	HynixSetRRPara
	mov	r2, r10
	mov	r0, r7
	mov	r1, fp
	mov	r3, r8
	bl	FlashReadRawPage
	adds	r2, r0, #1
	ldr	ip, [sp, #4]
	beq	.L1552
	ldrb	r3, [r5, #1889]	@ zero_extendqisi2
	cmp	r6, #-1
	it	eq
	moveq	r6, r0
	add	r3, r3, r3, lsl #1
	cmp	r0, r3, asr #2
	bcc	.L1559
	mov	r8, #0
	mov	r10, r8
.L1552:
	add	ip, ip, #1
	b	.L1550
.L1559:
	mov	r6, r0
.L1554:
	ldr	r3, [r5, #1784]
	add	r7, r7, r5
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	subs	r3, r3, #7
	cmp	r3, #1
	ldrb	r3, [r5, #1889]	@ zero_extendqisi2
	ite	ls
	strbls	r4, [r7, #1922]
	strbhi	r4, [r7, #1914]
	add	r3, r3, r3, lsl #1
	cmp	r6, r3, asr #2
	bcc	.L1558
	adds	r3, r6, #1
	it	ne
	movne	r6, #256
.L1558:
	mov	r0, r6
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1566:
	.align	2
.L1565:
	.word	.LANCHOR0
	.word	.LANCHOR0+1906
	.fnend
	.size	HynixReadRetrial, .-HynixReadRetrial
	.align	1
	.global	FlashProgPage
	.thumb
	.thumb_func
	.type	FlashProgPage, %function
FlashProgPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #8
	mov	r8, r3
	ldr	r3, .L1570
	mov	r5, r1
	mov	r7, r2
	mov	r4, r0
	ldrb	r6, [r3, #265]	@ zero_extendqisi2
	cbnz	r0, .L1568
	ldr	r2, .L1570+4
	ldrb	r3, [r2, #1]	@ zero_extendqisi2
	ldr	r1, [r2, #4]
	muls	r1, r3, r1
	cmp	r5, r1
	bcs	.L1568
	ldrb	r3, [r2]	@ zero_extendqisi2
	cbnz	r3, .L1569
	subs	r6, r6, #2
	b	.L1568
.L1569:
	movs	r6, #4
.L1568:
	mov	r0, r4
	bl	NandcWaitFlashReady
	mov	r0, r4
	bl	NandcFlashCs
	mov	r0, r4
	mov	r1, r5
	bl	FlashProgFirstCmd
	uxtb	r2, r6
	mov	r3, r7
	mov	r0, r4
	movs	r1, #1
	str	r8, [sp]
	bl	NandcXferData
	mov	r1, r5
	mov	r0, r4
	bl	FlashProgSecondCmd
	mov	r0, r4
	bl	NandcWaitFlashReady
	mov	r1, r5
	mov	r0, r4
	bl	FlashReadStatus
	mov	r5, r0
	mov	r0, r4
	bl	NandcFlashDeCs
	and	r0, r5, #1
	add	sp, sp, #8
	@ sp needed
	pop	{r4, r5, r6, r7, r8, pc}
.L1571:
	.align	2
.L1570:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.fnend
	.size	FlashProgPage, .-FlashProgPage
	.align	1
	.global	FlashSavePhyInfo
	.thumb
	.thumb_func
	.type	FlashSavePhyInfo, %function
FlashSavePhyInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r4, .L1581
	ldr	r5, .L1581+4
	ldr	r8, .L1581+12
	ldr	r3, [r4, #3992]
	mov	r9, r4
	ldrb	r0, [r4, #4018]	@ zero_extendqisi2
	mov	r10, r5
	str	r3, [r4, #4004]
	bl	FlashBchSel
	ldr	r0, [r4, #3992]
	mov	r1, #2048
	bl	__memzero
	ldr	r3, [r4, #4004]
	addw	r1, r5, #1788
	movs	r2, #32
	str	r8, [r3]
	ldr	r0, [r4, #4004]
	ldrb	r3, [r5, #1612]	@ zero_extendqisi2
	adds	r0, r0, #16
	strh	r3, [r0, #-4]	@ movhi
	ldrb	r3, [r5, #1]	@ zero_extendqisi2
	strh	r3, [r0, #-2]	@ movhi
	ldrb	r3, [r5, #2768]	@ zero_extendqisi2
	str	r3, [r0, #1060]
	bl	ftl_memcpy
	ldr	r0, [r4, #4004]
	add	r1, r5, #1616
	movs	r2, #8
	adds	r0, r0, #80
	bl	ftl_memcpy
	ldr	r0, [r4, #4004]
	add	r1, r5, #1624
	movs	r2, #32
	adds	r0, r0, #96
	bl	ftl_memcpy
	ldr	r0, [r4, #4004]
	ldr	r1, .L1581+8
	movs	r2, #32
	adds	r0, r0, #160
	bl	ftl_memcpy
	ldr	r0, [r4, #4004]
	add	r1, r5, #44
	movs	r2, #32
	adds	r0, r0, #192
	bl	ftl_memcpy
	ldr	r0, [r4, #4004]
	mov	r2, #852
	addw	r1, r5, #1902
	adds	r0, r0, #224
	bl	ftl_memcpy
	ldr	r6, [r4, #4004]
	movw	r1, #2036
	add	r0, r6, #12
	bl	js_hash
	mov	r3, #1592
	str	r3, [r6, #4]
	ldr	r3, [r4, #4020]
	str	r3, [r4, #4004]
	str	r0, [r6, #8]
	movs	r0, #0
	bl	flash_enter_slc_mode
	movs	r6, #0
	mov	r7, r6
.L1575:
	ldr	r1, [r5, #4]
	movs	r0, #0
	mov	r2, r0
	muls	r1, r7, r1
	bl	FlashEraseBlock
	ldr	r1, [r5, #4]
	movs	r0, #0
	ldr	r2, [r4, #3992]
	muls	r1, r7, r1
	mov	r3, r0
	bl	FlashProgPage
	ldr	r1, [r5, #4]
	movs	r0, #0
	ldr	r2, [r4, #3992]
	muls	r1, r7, r1
	mov	r3, r0
	adds	r1, r1, #1
	bl	FlashProgPage
	ldr	r1, [r5, #4]
	movs	r0, #0
	ldr	r2, [r4, #4020]
	muls	r1, r7, r1
	mov	r3, r0
	bl	FlashReadRawPage
	adds	r2, r7, #1
	adds	r0, r0, #1
	beq	.L1573
	ldr	fp, [r9, #4004]
	ldr	r3, [fp]
	cmp	r3, r8
	bne	.L1573
	add	r0, fp, #12
	movw	r1, #2036
	str	r2, [sp, #4]
	bl	js_hash
	ldr	r3, [fp, #8]
	cmp	r3, r0
	ldr	r2, [sp, #4]
	bne	.L1573
	ldr	r3, [r10, #4]
	cmp	r6, #1
	str	r2, [r9, #4012]
	mul	r3, r3, r7
	str	r3, [r9, #4008]
	beq	.L1576
	movs	r6, #1
.L1573:
	cmp	r2, #4
	mov	r7, r2
	bne	.L1575
	b	.L1574
.L1576:
	movs	r6, #2
.L1574:
	movs	r0, #0
	bl	flash_exit_slc_mode
	clz	r0, r6
	lsrs	r0, r0, #5
	negs	r0, r0
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1582:
	.align	2
.L1581:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR1+256
	.word	1312902724
	.fnend
	.size	FlashSavePhyInfo, .-FlashSavePhyInfo
	.align	1
	.global	FlashReadIdbDataRaw
	.thumb
	.thumb_func
	.type	FlashReadIdbDataRaw, %function
FlashReadIdbDataRaw:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r9, r0
	ldr	r3, .L1602
	ldr	r4, .L1602+4
	ldr	r0, [r3]	@ unaligned
	ldr	r3, [r4, #2764]
	ldrb	r10, [r4, #1889]	@ zero_extendqisi2
	str	r0, [sp, #4]	@ unaligned
	cbz	r3, .L1584
	movs	r0, #0
	bl	flash_enter_slc_mode
.L1584:
	mov	r0, r9
	mov	r1, #2048
	bl	__memzero
	mov	r8, #-1
	movs	r5, #2
.L1585:
	ldrb	r3, [r4, #1]	@ zero_extendqisi2
	cmp	r5, r3
	bcs	.L1589
	movs	r7, #0
.L1587:
	add	r3, sp, #4
	ldr	r6, .L1602+8
	ldrb	fp, [r7, r3]	@ zero_extendqisi2
	mov	r0, fp
	bl	FlashBchSel
	ldr	r1, [r4, #4]
	movs	r0, #0
	ldr	r2, [r6, #3992]
	muls	r1, r5, r1
	mov	r3, r0
	bl	FlashReadRawPage
	adds	r0, r0, #1
	bne	.L1586
	adds	r7, r7, #1
	cmp	r7, #4
	bne	.L1587
	b	.L1588
.L1586:
	ldr	r3, [r6, #3992]
	ldr	r2, .L1602+12
	ldr	r3, [r3]
	cmp	r3, r2
	bne	.L1588
	mov	r1, fp
	ldr	r0, .L1602+16
	bl	printk
	mov	r2, #2048
	mov	r0, r9
	ldr	r1, [r6, #3992]
	bl	ftl_memcpy
	ldr	r3, [r6, #3992]
	ldr	r2, .L1602+4
	ldr	r3, [r3, #512]
	strb	r3, [r2, #1]
	ldr	r3, [r6, #4012]
	cmp	r3, r5
	bls	.L1592
	str	r5, [r6, #4012]
	bl	FlashSavePhyInfo
	mov	r8, #0
.L1588:
	adds	r5, r5, #1
	b	.L1585
.L1592:
	mov	r8, #0
.L1589:
	mov	r0, r10
	bl	FlashBchSel
	ldr	r3, [r4, #2764]
	cbz	r3, .L1596
	movs	r0, #0
	bl	flash_exit_slc_mode
.L1596:
	mov	r0, r8
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1603:
	.align	2
.L1602:
	.word	.LANCHOR3+11
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-52655045
	.word	.LC95
	.fnend
	.size	FlashReadIdbDataRaw, .-FlashReadIdbDataRaw
	.align	1
	.global	FlashInit
	.thumb
	.thumb_func
	.type	FlashInit, %function
FlashInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r5, r0
	.pad #20
	sub	sp, sp, #20
	mov	r0, #32768
	bl	ftl_malloc
	ldr	r6, .L1723
	ldr	r4, .L1723+4
	movs	r7, #0
	add	r9, r4, #12
	mov	r8, r6
	str	r0, [r6, #3992]
	mov	r0, #32768
	bl	ftl_malloc
	str	r0, [r6, #4020]
	mov	r0, #4096
	bl	ftl_malloc
	str	r0, [r6, #4024]
	mov	r0, #32768
	bl	ftl_malloc
	str	r0, [r6, #4028]
	mov	r0, #4096
	bl	ftl_malloc
	movs	r3, #50
	str	r7, [r6, #4012]
	strb	r3, [r4, #1]
	strb	r3, [r6, #4016]
	movs	r3, #128
	strb	r7, [r4, #2768]
	str	r3, [r4, #4]
	movs	r3, #60
	str	r7, [r6, #3996]
	strb	r7, [r4]
	strb	r7, [r6, #4036]
	strb	r3, [r6, #4018]
	str	r0, [r6, #4032]
	mov	r0, r5
	bl	NandcInit
	addw	r5, r4, #1788
.L1610:
	add	r3, r9, r7, lsl #3
	uxtb	fp, r7
	ldr	r6, [r9, r7, lsl #3]
	ldrb	r10, [r3, #4]	@ zero_extendqisi2
	mov	r0, fp
	bl	FlashReset
	mov	r0, fp
	bl	NandcFlashCs
	movs	r3, #144
	add	r6, r6, r10, lsl #8
	movs	r0, #200
	mov	r10, #0
	str	r3, [r6, #2056]
	str	r10, [r6, #2052]
	bl	NandcDelayns
	ldr	r2, [r6, #2048]
	uxtb	r2, r2
	strb	r2, [r5]
	ldr	r1, [r6, #2048]
	cmp	r2, #44
	strb	r1, [r5, #1]
	ldr	r1, [r6, #2048]
	strb	r1, [r5, #2]
	ldr	r1, [r6, #2048]
	strb	r1, [r5, #3]
	ldr	r1, [r6, #2048]
	strb	r1, [r5, #4]
	ldr	r1, [r6, #2048]
	strb	r1, [r5, #5]
	bne	.L1605
	movs	r2, #239
	movs	r0, #200
	str	r2, [r6, #2056]
	movs	r2, #1
	str	r2, [r6, #2052]
	bl	NandcDelayns
	movs	r2, #4
	str	r2, [r6, #2048]
	str	r10, [r6, #2048]
	str	r10, [r6, #2048]
	str	r10, [r6, #2048]
.L1605:
	mov	r0, fp
	bl	NandcFlashDeCs
	ldrb	r2, [r5]	@ zero_extendqisi2
	subs	r3, r2, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L1606
	ldrb	r1, [r5, #2]	@ zero_extendqisi2
	ldrb	r3, [r5, #1]	@ zero_extendqisi2
	ldr	r0, .L1723+8
	str	r1, [sp]
	ldrb	r1, [r5, #3]	@ zero_extendqisi2
	str	r1, [sp, #4]
	ldrb	r1, [r5, #4]	@ zero_extendqisi2
	str	r1, [sp, #8]
	ldrb	r1, [r5, #5]	@ zero_extendqisi2
	str	r1, [sp, #12]
	adds	r1, r7, #1
	bl	printk
.L1606:
	cbnz	r7, .L1607
	ldrb	r3, [r4, #1788]	@ zero_extendqisi2
	subs	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L1662
	ldr	r3, .L1723+4
	ldrb	r3, [r3, #1789]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L1662
.L1607:
	ldrb	r3, [r5]	@ zero_extendqisi2
	adds	r7, r7, #1
	adds	r5, r5, #8
	cmp	r3, #181
	itt	eq
	moveq	r3, #44
	strbeq	r3, [r5, #-8]
	cmp	r7, #4
	bne	.L1610
	ldrb	r3, [r4, #1788]	@ zero_extendqisi2
	cmp	r3, #173
	beq	.L1611
	ldr	r3, .L1723+4
	ldr	r0, [r3, #2772]
	bl	NandcSetDdrMode
.L1611:
	ldr	r6, .L1723+4
	mov	r1, #852
	ldr	r5, .L1723+12
	addw	r0, r6, #1902
	bl	__memzero
	ldr	r3, .L1723+16
	ldr	r2, [r4, #2780]
	ldrb	r0, [r4, #1789]	@ zero_extendqisi2
	cmp	r2, r5
	str	r3, [r4, #1784]
	mov	r3, #0
	strb	r3, [r4, #8]
	itt	eq
	moveq	r3, #1
	streq	r3, [r6, #2764]
	cmp	r0, #161
	beq	.L1613
	cmp	r0, #241
	beq	.L1613
	cmp	r0, #218
	beq	.L1613
	and	r3, r0, #253
	cmp	r3, #209
	beq	.L1613
	cmp	r0, #220
	bne	.L1614
	ldr	r3, .L1723+4
	ldrb	r3, [r3, #1791]	@ zero_extendqisi2
	cmp	r3, #149
	bne	.L1614
.L1613:
	ldrb	r7, [r4, #1788]	@ zero_extendqisi2
	movs	r3, #16
	strb	r3, [r4, #1]
	movs	r6, #1
	strb	r3, [r8, #4018]
	cmp	r7, #152
	ldr	r3, .L1723+20
	strb	r6, [r4]
	ldr	r1, .L1723
	strb	r7, [r3, #3257]
	strb	r0, [r3, #3258]
	bne	.L1616
	ldr	r7, .L1723+4
	ldrsb	r7, [r7, #1792]
	cmp	r7, #0
	blt	.L1617
	movs	r6, #24
	strb	r6, [r1, #4018]
.L1616:
	cmp	r2, r5
	beq	.L1619
	movw	r1, #2049
	cmp	r2, r1
	bne	.L1620
.L1619:
	movs	r2, #16
	strb	r2, [r8, #4018]
.L1620:
	cmp	r0, #218
	bne	.L1621
	mov	r2, #2048
	b	.L1718
.L1621:
	cmp	r0, #220
	bne	.L1623
	mov	r2, #4096
.L1718:
	strh	r2, [r3, #3270]	@ movhi
	strb	r0, [r3, #3258]
	b	.L1622
.L1623:
	cmp	r0, #211
	itttt	eq
	moveq	r2, #4096
	strheq	r2, [r3, #3270]	@ movhi
	moveq	r2, #2
	strbeq	r2, [r3, #3269]
.L1622:
	ldr	r1, .L1723+24
	movs	r2, #32
	ldr	r0, .L1723+28
	bl	ftl_memcpy
	ldr	r0, .L1723+16
	movs	r2, #32
	addw	r1, r0, #3000
	bl	ftl_memcpy
.L1614:
	ldrb	r3, [r4]	@ zero_extendqisi2
	ldr	r6, .L1723+4
	cmp	r3, #0
	bne	.L1624
	bl	FlashLoadPhyInfoInRam
	cbnz	r0, .L1626
	ldr	r3, [r6, #1784]
	ldrh	r3, [r3, #16]
	lsrs	r3, r3, #8
	lsls	r7, r3, #31
	and	r0, r3, #7
	strb	r0, [r6, #2760]
	bmi	.L1626
	movs	r3, #1
	strb	r3, [r6, #2768]
	bl	FlashSetInterfaceMode
	ldrb	r0, [r6, #2760]	@ zero_extendqisi2
	bl	NandcSetMode
.L1626:
	ldr	r3, [r4, #1784]
	ldr	r6, .L1723+4
	ldrb	r3, [r3, #26]	@ zero_extendqisi2
	strb	r3, [r4, #1888]
	bl	FlashLoadPhyInfo
	cmp	r0, #0
	beq	.L1624
	ldr	r3, [r6, #2772]
	cbz	r3, .L1629
	movs	r0, #1
	bl	FlashSetInterfaceMode
	movs	r0, #1
	b	.L1719
.L1724:
	.align	2
.L1723:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC96
	.word	1446522928
	.word	.LANCHOR1+256
	.word	.LANCHOR1
	.word	.LANCHOR1+3132
	.word	.LANCHOR0+44
.L1629:
	ldrb	r0, [r6, #2760]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r6, #2760]	@ zero_extendqisi2
.L1719:
	bl	NandcSetMode
	bl	FlashLoadPhyInfo
	cmp	r0, #0
	beq	.L1624
	movs	r0, #1
	ldr	r6, .L1725
	bl	FlashSetInterfaceMode
	movs	r0, #1
	bl	NandcSetMode
	ldr	r3, [r4, #1784]
	ldr	r0, .L1725+4
	ldrh	r1, [r3, #14]
	bl	printk
	bl	FlashLoadPhyInfoInRam
	adds	r1, r0, #1
	beq	.L1681
	bl	FlashDieInfoInit
	ldr	r3, [r6, #1784]
	ldrb	r0, [r3, #19]	@ zero_extendqisi2
	bl	FlashGetReadRetryDefault
	ldr	r3, [r6, #1784]
	ldrh	r2, [r6, #1820]
	ldrb	r1, [r3, #9]	@ zero_extendqisi2
	addw	r2, r2, #4095
	cmp	r1, r2, asr #12
	blt	.L1631
	ldrh	r2, [r3, #14]
	adds	r2, r2, #255
	cmp	r1, r2, asr #8
	bge	.L1632
.L1631:
	ldrh	r2, [r3, #14]
	bic	r2, r2, #255
	strh	r2, [r3, #14]	@ movhi
.L1632:
	ldrb	r3, [r4, #2760]	@ zero_extendqisi2
	tst	r3, #6
	beq	.L1633
	bl	FlashSavePhyInfo
	movs	r0, #0
	ldr	r1, [r8, #4008]
	bl	FlashDdrParaScan
.L1633:
	bl	FlashSavePhyInfo
.L1624:
	ldr	r2, [r4, #1784]
	ldr	r7, .L1725
	ldr	r6, .L1725+8
	ldrb	r3, [r2, #26]	@ zero_extendqisi2
	ldrh	r0, [r2, #10]
	ldrb	r9, [r2, #18]	@ zero_extendqisi2
	strb	r3, [r4, #1888]
	ldrh	r3, [r2, #16]
	ubfx	r1, r3, #7, #1
	strb	r1, [r4, #8]
	ubfx	r1, r3, #3, #1
	strb	r1, [r8, #4037]
	ubfx	r1, r3, #4, #1
	ubfx	r3, r3, #8, #3
	strb	r1, [r4, #2828]
	strb	r3, [r4, #2760]
	movs	r3, #0
	ldrb	r1, [r2, #12]	@ zero_extendqisi2
	str	r3, [r8, #4000]
	bl	__aeabi_idiv
	mov	r1, r0
	mov	r0, r9
	bl	BuildFlashLsbPageTable
	bl	FlashDieInfoInit
	ldr	r3, [r4, #1784]
	ldrh	r2, [r3, #16]
	lsls	r2, r2, #25
	bpl	.L1635
	ldrb	r0, [r3, #19]	@ zero_extendqisi2
	ldrb	r3, [r7, #1903]	@ zero_extendqisi2
	strb	r0, [r7, #1900]
	strb	r3, [r7, #1901]
	ldrb	r3, [r7, #1904]	@ zero_extendqisi2
	strb	r3, [r6, #4017]
	subs	r3, r0, #1
	cmp	r3, #7
	bhi	.L1636
	ldr	r3, .L1725+12
	str	r3, [r6, #4000]
	subs	r3, r0, #5
	cmp	r3, #1
	bls	.L1637
	cmp	r0, #8
	bne	.L1638
.L1637:
	cmp	r0, #7
	mov	r3, #1
	str	r3, [r4, #2820]
	beq	.L1663
	cmp	r0, #8
	b	.L1722
.L1663:
	ldr	r3, .L1725+16
.L1640:
	subs	r1, r3, #1
	movs	r2, #0
	adds	r3, r3, #31
.L1641:
	ldrsb	r6, [r1, #1]!
	cmp	r6, #0
	it	eq
	addeq	r2, r2, #1
	cmp	r1, r3
	bne	.L1641
	cmp	r2, #27
	bls	.L1635
	bl	FlashGetReadRetryDefault
	bl	FlashSavePhyInfo
	b	.L1635
.L1636:
	sub	r3, r0, #17
	cmp	r3, #2
	bhi	.L1643
	ldr	r3, .L1725+20
	cmp	r0, #19
	str	r3, [r6, #4000]
	ite	ne
	movne	r3, #7
	moveq	r3, #15
	strb	r3, [r6, #4017]
	b	.L1635
.L1643:
	cmp	r0, #33
	beq	.L1645
	sub	r3, r0, #65
	cmp	r3, #1
	bhi	.L1646
.L1645:
	ldr	r3, .L1725+24
	str	r3, [r8, #4000]
	movs	r3, #4
	strb	r3, [r4, #1901]
	movs	r3, #7
	strb	r3, [r8, #4017]
	b	.L1635
.L1646:
	sub	r3, r0, #67
	uxtb	r3, r3
	cmp	r3, #1
	bls	.L1647
	sub	r2, r0, #34
	cmp	r2, #1
	bhi	.L1648
.L1647:
	ldr	r2, .L1725+24
	cmp	r0, #35
	str	r2, [r8, #4000]
	beq	.L1649
	cmp	r0, #68
	beq	.L1649
	ldr	r2, .L1725+8
	movs	r1, #7
	strb	r1, [r2, #4017]
	b	.L1650
.L1649:
	movs	r2, #17
	strb	r2, [r8, #4017]
.L1650:
	cmp	r3, #1
	ite	ls
	movls	r3, #4
	movhi	r3, #5
	strb	r3, [r4, #1901]
	b	.L1635
.L1648:
	cmp	r0, #49
	itt	eq
	ldreq	r3, .L1725+28
	streq	r3, [r6, #4000]
.L1635:
	ldr	r2, [r4, #2780]
	ldr	r3, .L1725
	cmp	r2, r5
	bne	.L1652
	ldrb	r1, [r3, #1888]	@ zero_extendqisi2
	cbz	r1, .L1652
	ldr	r3, [r3, #1784]
	movs	r1, #0
	strb	r1, [r3, #18]
.L1652:
	ldrb	r1, [r4, #1788]	@ zero_extendqisi2
	ldr	r3, .L1725
	cmp	r1, #44
	bne	.L1653
	ldrb	r1, [r3, #2768]	@ zero_extendqisi2
	cbz	r1, .L1653
	cmp	r2, r5
	bne	.L1654
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cbnz	r3, .L1653
.L1654:
	movs	r0, #1
	movs	r3, #0
	strb	r3, [r4, #2768]
	bl	FlashSetInterfaceMode
	movs	r0, #1
	bl	NandcSetMode
.L1653:
	ldrb	r3, [r4, #2760]	@ zero_extendqisi2
	tst	r3, #6
	beq	.L1655
	ldr	r2, .L1725
	ldrb	r2, [r2, #2768]	@ zero_extendqisi2
	cbnz	r2, .L1656
	lsls	r3, r3, #31
	bmi	.L1655
.L1656:
	movs	r0, #0
	bl	flash_enter_slc_mode
	movs	r0, #0
	ldr	r1, [r8, #4008]
	bl	FlashDdrParaScan
	movs	r0, #0
	bl	flash_exit_slc_mode
.L1655:
	ldr	r3, [r4, #1784]
	mov	r8, #16
	ldr	r6, .L1725
	ldrb	r0, [r3, #20]	@ zero_extendqisi2
	bl	FlashBchSel
	ldr	r0, .L1725+32
	bl	FlashReadIdbDataRaw
	ldr	r0, .L1725+36
	strb	r8, [r4, #1]
	bl	FlashTimingCfg
	ldr	r5, [r4, #1784]
	ldrb	r3, [r5, #12]	@ zero_extendqisi2
	ldrh	r7, [r5, #14]
	strh	r3, [r4, #1868]	@ movhi
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	str	r3, [r4, #1864]
	ldr	r3, [r4, #1788]
	str	r3, [r4, #1860]
	ldrb	r3, [r4, #1612]	@ zero_extendqisi2
	strh	r3, [r4, #1870]	@ movhi
	ldrb	r3, [r5, #13]	@ zero_extendqisi2
	strh	r7, [r4, #1874]	@ movhi
	strh	r3, [r4, #1872]	@ movhi
	ldrh	r3, [r5, #10]
	strh	r3, [r4, #1876]	@ movhi
	ldrb	r1, [r5, #12]	@ zero_extendqisi2
	ldrh	r0, [r5, #10]
	bl	__aeabi_idiv
	strh	r0, [r4, #1878]	@ movhi
	ldrb	r2, [r5, #9]	@ zero_extendqisi2
	strh	r2, [r4, #1880]	@ movhi
	ldrb	r1, [r5, #9]	@ zero_extendqisi2
	ldrh	r3, [r5, #10]
	smulbb	r3, r1, r3
	mov	r1, #512
	strh	r1, [r4, #1884]	@ movhi
	ldrb	r1, [r4, #1]	@ zero_extendqisi2
	strh	r1, [r4, #1886]	@ movhi
	uxth	r3, r3
	ldrb	r1, [r4]	@ zero_extendqisi2
	strh	r3, [r4, #1882]	@ movhi
	cmp	r1, #1
	bne	.L1658
	lsls	r3, r3, #1
	lsrs	r1, r7, #1
	lsls	r2, r2, #1
	strh	r3, [r6, #1882]	@ movhi
	strb	r8, [r6, #1]
	movs	r3, #8
	strh	r1, [r6, #1874]	@ movhi
	strh	r2, [r6, #1880]	@ movhi
	strh	r3, [r6, #1886]	@ movhi
.L1658:
	ldrb	r0, [r5, #20]	@ zero_extendqisi2
	bl	FlashBchSel
	bl	ftl_flash_suspend
	movs	r0, #0
	b	.L1681
.L1662:
	mvn	r0, #1
	b	.L1681
.L1638:
	cmp	r0, #7
.L1722:
	beq	.L1663
	ldr	r3, .L1725+40
	b	.L1640
.L1617:
	strb	r6, [r1, #4036]
	b	.L1616
.L1681:
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1726:
	.align	2
.L1725:
	.word	.LANCHOR0
	.word	.LC97
	.word	.LANCHOR2
	.word	HynixReadRetrial
	.word	.LANCHOR0+1930
	.word	MicronReadRetrial
	.word	ToshibaReadRetrial
	.word	SamsungReadRetrial
	.word	.LANCHOR2+1940
	.word	150000
	.word	.LANCHOR0+1922
	.fnend
	.size	FlashInit, .-FlashInit
	.align	1
	.global	FlashPageProgMsbFFData
	.thumb
	.thumb_func
	.type	FlashPageProgMsbFFData, %function
FlashPageProgMsbFFData:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r4, r2
	ldr	r3, .L1747
	mov	r8, r0
	mov	r9, r1
	ldr	r2, [r3, #1784]
	mov	r6, r3
	ldrb	r7, [r2, #19]	@ zero_extendqisi2
	ldrb	r2, [r3, #1888]	@ zero_extendqisi2
	cbz	r2, .L1728
	ldr	r3, [r3, #2764]
	cbnz	r3, .L1727
.L1728:
	subs	r3, r7, #5
	cmp	r3, #3
	bls	.L1729
	cmp	r7, #68
	beq	.L1729
	cmp	r7, #35
	beq	.L1729
	cmp	r7, #19
	bne	.L1727
.L1729:
	ldr	r10, .L1747
	movw	fp, #65535
	ldr	r5, .L1747+4
.L1731:
	ldr	r3, [r6, #1784]
	ldrh	r3, [r3, #10]
	cmp	r3, r4
	bls	.L1727
	add	r3, r10, r4, lsl #1
	ldrh	r3, [r3, #588]
	cmp	r3, fp
	bne	.L1727
	cmp	r7, #8
	ldr	r0, [r5, #4020]
	bne	.L1732
	mov	r1, #32768
	bl	__memzero
	b	.L1733
.L1732:
	movs	r1, #255
	mov	r2, #32768
	bl	memset
.L1733:
	ldr	r2, [r5, #4020]
	add	r1, r4, r9
	adds	r4, r4, #1
	mov	r0, r8
	mov	r3, r2
	uxth	r4, r4
	bl	FlashProgPage
	b	.L1731
.L1727:
	pop	{r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1748:
	.align	2
.L1747:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FlashPageProgMsbFFData, .-FlashPageProgMsbFFData
	.align	1
	.global	FlashReadSlc2KPages
	.thumb
	.thumb_func
	.type	FlashReadSlc2KPages, %function
FlashReadSlc2KPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1799
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movs	r7, #0
	ldrb	r3, [r3, #265]	@ zero_extendqisi2
	.pad #36
	sub	sp, sp, #36
	mov	r4, r0
	str	r1, [sp, #16]
	str	r2, [sp, #20]
	str	r3, [sp, #12]
.L1750:
	ldr	r3, [sp, #16]
	cmp	r7, r3
	beq	.L1798
	ldr	r3, [sp, #16]
	add	r2, sp, #28
	mov	r0, r4
	ldr	r1, [sp, #20]
	subs	r3, r3, r7
	ldr	r8, .L1799+20
	uxtb	r3, r3
	str	r3, [sp]
	add	r3, sp, #24
	mov	fp, r8
	bl	LogAddr2PhyAddr
	ldrb	r2, [r8, #1612]	@ zero_extendqisi2
	ldr	r3, [sp, #24]
	cmp	r3, r2
	bcc	.L1751
	mov	r3, #-1
	str	r3, [r4]
	b	.L1752
.L1751:
	add	r3, r3, r8
	mov	r9, #0
	ldrb	r6, [r3, #1616]	@ zero_extendqisi2
	mov	r0, r6
	bl	NandcWaitFlashReady
	mov	r0, r6
	bl	NandcFlashCs
.L1753:
	ldr	r1, [sp, #28]
	mov	r0, r6
	bl	FlashReadCmd
	mov	r0, r6
	bl	NandcWaitFlashReady
	ldr	r3, [r4, #12]
	mov	r0, r6
	movs	r1, #0
	str	r3, [sp]
	ldr	r2, [sp, #12]
	ldr	r3, [r4, #8]
	bl	NandcXferData
	ldr	r3, .L1799+4
	ldrb	r3, [r3, #4036]	@ zero_extendqisi2
	mov	r5, r0
	cbnz	r3, .L1754
.L1757:
	adds	r3, r5, #1
	beq	.L1755
	b	.L1756
.L1754:
	mov	r0, r6
	bl	flash_read_ecc
	cmp	r0, #5
	bls	.L1757
	mov	r5, #256
	b	.L1756
.L1755:
	cmp	r9, #10
	beq	.L1776
	add	r9, r9, #1
	b	.L1753
.L1756:
	cmp	r9, #0
	it	ne
	movne	r5, #256
	b	.L1758
.L1776:
	mov	r5, #256
.L1758:
	mov	r9, #0
.L1759:
	ldr	r3, [sp, #28]
	mov	r0, r6
	ldr	r1, [fp, #4]
	add	r1, r1, r3
	bl	FlashReadCmd
	mov	r0, r6
	bl	NandcWaitFlashReady
	ldr	r3, [r4, #8]
	cbz	r3, .L1760
	add	r3, r3, #2048
.L1760:
	ldr	r2, [r4, #12]
	cbz	r2, .L1761
	adds	r2, r2, #8
.L1761:
	str	r2, [sp]
	mov	r0, r6
	ldr	r2, [sp, #12]
	movs	r1, #0
	bl	NandcXferData
	ldr	r3, .L1799+4
	ldrb	r2, [r3, #4036]	@ zero_extendqisi2
	mov	r10, r0
	cbnz	r2, .L1762
.L1765:
	cmp	r10, #-1
	beq	.L1763
	b	.L1764
.L1762:
	mov	r0, r6
	bl	flash_read_ecc
	cmp	r0, #5
	bls	.L1765
	mov	r10, #256
	b	.L1764
.L1763:
	cmp	r9, #10
	beq	.L1780
	add	r9, r9, #1
	b	.L1759
.L1764:
	cmp	r9, #0
	it	ne
	movne	r10, #256
	b	.L1766
.L1780:
	mov	r10, #256
.L1766:
	mov	r0, r6
	bl	NandcFlashDeCs
	ldrb	r3, [r8, #1889]	@ zero_extendqisi2
	cmp	r5, r10
	add	r3, r3, r3, lsl #1
	it	cc
	movcc	r5, r10
	cmp	r5, r3, asr #2
	bls	.L1767
	adds	r0, r5, #1
	it	ne
	movne	r5, #256
	str	r5, [r4]
	b	.L1769
.L1767:
	movs	r3, #0
	str	r3, [r4]
.L1769:
	ldr	r3, [r4, #12]
	cbz	r3, .L1770
	ldr	r2, [r3, #12]
	adds	r2, r2, #1
	bne	.L1770
	ldr	r2, [r3, #8]
	adds	r1, r2, #1
	bne	.L1770
	ldr	r3, [r3]
	adds	r3, r3, #1
	it	ne
	strne	r2, [r4]
.L1770:
	ldr	r3, [r4]
	adds	r2, r3, #1
	bne	.L1752
	ldr	r1, [r4, #4]
	ldr	r0, .L1799+8
	ldrb	r2, [r8, #1889]	@ zero_extendqisi2
	bl	printk
	ldr	r1, [r4, #8]
	cbz	r1, .L1772
	ldr	r0, .L1799+12
	movs	r2, #4
	movs	r3, #8
	bl	rknand_print_hex
.L1772:
	ldr	r1, [r4, #12]
	cbz	r1, .L1752
	movs	r2, #4
	ldr	r0, .L1799+16
	mov	r3, r2
	bl	rknand_print_hex
.L1752:
	adds	r7, r7, #1
	adds	r4, r4, #36
	b	.L1750
.L1798:
	movs	r0, #0
	add	sp, sp, #36
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1800:
	.align	2
.L1799:
	.word	.LANCHOR1
	.word	.LANCHOR2
	.word	.LC98
	.word	.LC99
	.word	.LC100
	.word	.LANCHOR0
	.fnend
	.size	FlashReadSlc2KPages, .-FlashReadSlc2KPages
	.align	1
	.global	FlashReadPages
	.thumb
	.thumb_func
	.type	FlashReadPages, %function
FlashReadPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1900
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #52
	sub	sp, sp, #52
	ldrb	r3, [r3, #265]	@ zero_extendqisi2
	mov	r10, r0
	ldr	r5, .L1900+4
	str	r1, [sp, #28]
	str	r3, [sp, #24]
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	str	r2, [sp, #16]
	str	r3, [sp, #32]
	ldrb	r3, [r5]	@ zero_extendqisi2
	cbz	r3, .L1844
	bl	FlashReadSlc2KPages
	b	.L1895
.L1844:
	mov	r8, r3
	str	r3, [sp, #12]
.L1802:
	ldr	r3, [sp, #12]
	ldr	r2, [sp, #28]
	cmp	r3, r2
	bcs	.L1897
	ldr	r3, [sp, #12]
	mov	r9, #36
	ldr	r2, [sp, #12]
	ldr	r1, [sp, #16]
	mul	r9, r9, r3
	ldr	r3, [sp, #28]
	subs	r3, r3, r2
	add	r2, sp, #44
	add	r7, r10, r9
	uxtb	r3, r3
	ldr	fp, [r7, #4]
	mov	r0, r7
	str	r3, [sp]
	add	r3, sp, #40
	bl	LogAddr2PhyAddr
	ldrb	r2, [r5, #1612]	@ zero_extendqisi2
	ldr	r3, [sp, #40]
	mov	r6, r0
	cmp	r3, r2
	bcc	.L1804
	mov	r3, #-1
	str	r3, [r10, r9]
	b	.L1805
.L1804:
	add	r3, r3, r5
	ldrb	r4, [r3, #1616]	@ zero_extendqisi2
	ldr	r3, .L1900+8
	mov	r0, r4
	ldrb	r3, [r3, #4037]	@ zero_extendqisi2
	cmp	r3, #0
	it	eq
	moveq	r6, #0
	bl	NandcWaitFlashReady
	ldr	r3, .L1900+4
	ldr	r3, [r3, #1784]
	ldrb	r2, [r3, #19]	@ zero_extendqisi2
	subs	r3, r2, #1
	cmp	r3, #7
	bhi	.L1807
	subs	r2, r2, #7
	adds	r1, r5, r4
	cmp	r2, #1
	ldr	r2, .L1900+4
	ldrb	r3, [r1, #1914]	@ zero_extendqisi2
	add	r2, r2, r4
	it	ls
	ldrbls	r3, [r1, #1922]	@ zero_extendqisi2
	ldrb	r2, [r2, #2756]	@ zero_extendqisi2
	cmp	r2, r3
	beq	.L1807
	mov	r0, r4
	ldrb	r1, [r5, #1903]	@ zero_extendqisi2
	ldr	r2, .L1900+12
	bl	HynixSetRRPara
.L1807:
	mov	r0, r4
	and	r3, fp, #-2147483648
	str	r3, [sp, #20]
	bl	NandcFlashCs
	ldr	r3, [sp, #16]
	cmp	r3, #1
	beq	.L1809
	ldr	r3, [sp, #20]
	cbz	r3, .L1810
.L1809:
	ldr	r3, .L1900+4
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cbz	r3, .L1810
	mov	r0, r4
	bl	flash_enter_slc_mode
.L1812:
	mov	fp, #0
	b	.L1811
.L1810:
	mov	r0, r4
	bl	flash_exit_slc_mode
	b	.L1812
.L1898:
	cmp	r4, #255
	beq	.L1845
.L1813:
	cbz	r6, .L1815
	ldr	r2, [r5, #4]
	mov	r0, r4
	add	r2, r2, r1
	bl	FlashReadDpCmd
.L1816:
	mov	r0, r4
	bl	NandcWaitFlashReady
	cbz	r6, .L1814
	mov	r0, r4
	ldr	r1, [sp, #44]
	bl	FlashReadDpDataOutCmd
.L1814:
	ldr	r3, [r7, #12]
	mov	r0, r4
	movs	r1, #0
	ldr	r2, [sp, #24]
	str	r3, [sp]
	ldr	r3, [r7, #8]
	bl	NandcXferData
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	mov	ip, r0
	cbz	r3, .L1847
	cmp	r0, #-1
	bne	.L1847
	movs	r6, #0
	strb	fp, [r5, #8]
.L1811:
	ldr	r1, [sp, #44]
	adds	r0, r1, #1
	bne	.L1813
	b	.L1898
.L1815:
	mov	r0, r4
	bl	FlashReadCmd
	b	.L1816
.L1845:
	movs	r6, #0
	b	.L1814
.L1847:
	mov	fp, ip
	cbz	r6, .L1818
	ldr	r3, .L1900+4
	mov	r0, r4
	str	ip, [sp, #36]
	ldr	r1, [r3, #4]
	ldr	r3, [sp, #44]
	add	r1, r1, r3
	bl	FlashReadDpDataOutCmd
	add	r3, r9, #36
	add	r3, r3, r10
	mov	r0, r4
	movs	r1, #0
	ldr	r2, [r3, #12]
	str	r2, [sp]
	ldr	r2, [sp, #24]
	ldr	r3, [r3, #8]
	bl	NandcXferData
	cmp	r0, #-1
	ldr	ip, [sp, #36]
	it	eq
	moveq	r6, #0
	mov	r8, r0
.L1818:
	mov	r0, r4
	str	ip, [sp, #36]
	bl	NandcFlashDeCs
	ldr	ip, [sp, #36]
	ldrb	r3, [sp, #32]	@ zero_extendqisi2
	cmp	ip, #-1
	strb	r3, [r5, #8]
	bne	.L1825
	ldrb	r3, [r5, #2768]	@ zero_extendqisi2
	cbnz	r3, .L1820
.L1824:
	ldr	r3, .L1900+8
	ldr	r6, [r3, #4000]
	cbnz	r6, .L1821
	b	.L1899
.L1820:
	ldr	r3, [r5, #1824]
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r6, [r3, #304]
	movs	r3, #1
	str	r3, [sp]
	ldr	r2, [r7, #8]
	ldr	r3, [r7, #12]
	bl	FlashDdrTunningRead
	cmp	r0, #-1
	mov	fp, r0
	beq	.L1823
	ldrb	r3, [r5, #1889]	@ zero_extendqisi2
	cmp	r0, r3, lsr #1
	bls	.L1849
.L1823:
	ubfx	r0, r6, #8, #8
	bl	NandcSetDdrPara
	cmp	fp, #-1
	beq	.L1824
	b	.L1849
.L1821:
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r2, [r7, #8]
	ldr	r3, [r7, #12]
	blx	r6
	cmp	r0, #-1
	mov	fp, r0
	bne	.L1826
	ldr	r3, [r5, #1784]
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	subs	r3, r3, #1
	cmp	r3, #7
	bhi	.L1827
	mov	r0, r4
	ldrb	r1, [r5, #1903]	@ zero_extendqisi2
	ldr	r2, .L1900+12
	movs	r3, #0
	bl	HynixSetRRPara
.L1827:
	ldr	r1, [sp, #44]
	mov	r0, r4
	ldr	r2, [r7, #8]
	ldr	r3, [r7, #12]
	bl	FlashReadRawPage
	ldr	r1, [r7, #4]
	ldrb	r2, [r5, #1889]	@ zero_extendqisi2
	mov	fp, r0
	ldr	r0, .L1900+16
	mov	r3, fp
	bl	printk
	cmp	fp, #-1
	bne	.L1826
	ldrb	r6, [r5, #1888]	@ zero_extendqisi2
	cbz	r6, .L1828
	ldr	r3, [sp, #16]
	mov	r0, r4
	cmp	r3, #1
	beq	.L1829
	ldr	r3, [sp, #20]
	cbz	r3, .L1830
.L1829:
	bl	flash_enter_slc_mode
	b	.L1831
.L1830:
	bl	flash_exit_slc_mode
.L1831:
	ldr	r3, .L1900+8
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r2, [r7, #8]
	ldr	r6, [r3, #4000]
	ldr	r3, [r7, #12]
	blx	r6
	movs	r6, #0
	mov	fp, r0
	b	.L1832
.L1899:
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r2, [r7, #8]
	ldr	r3, [r7, #12]
	bl	FlashReadRawPage
	mov	fp, r0
	b	.L1832
.L1849:
	movs	r6, #0
.L1825:
	ldrb	r3, [r5, #1889]	@ zero_extendqisi2
	add	r3, r3, r3, lsl #1
	cmp	fp, r3, asr #2
	bls	.L1834
	ldr	r3, .L1900+8
	ldr	r3, [r3, #4000]
	cbz	r3, .L1851
.L1832:
	cmp	fp, #-1
	beq	.L1828
	b	.L1834
.L1851:
	mov	fp, #256
.L1828:
	str	fp, [r10, r9]
	b	.L1835
.L1901:
	.align	2
.L1900:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+1906
	.word	.LC101
.L1826:
	movs	r6, #0
.L1834:
	cmp	fp, #256
	beq	.L1828
	movs	r3, #0
	str	r3, [r10, r9]
.L1835:
	ldr	r3, [r10, r9]
	adds	r2, r3, #1
	bne	.L1837
	ldr	r1, [r7, #4]
	ldr	r0, .L1902
	ldrb	r2, [r5, #1889]	@ zero_extendqisi2
	bl	printk
	ldr	r1, [r7, #12]
	cbz	r1, .L1837
	movs	r2, #4
	ldr	r0, .L1902+4
	mov	r3, r2
	bl	rknand_print_hex
.L1837:
	cbz	r6, .L1839
	ldrb	r3, [r5, #1889]	@ zero_extendqisi2
	add	r3, r3, r3, lsl #1
	cmp	r8, r3, asr #2
	bls	.L1840
	ldr	r3, .L1902+8
	ldr	r3, [r3, #4000]
	cmp	r3, #0
	it	eq
	moveq	r8, #256
.L1840:
	add	r9, r9, #36
	cmp	r8, #-1
	str	r8, [r10, r9]
	beq	.L1839
	cmp	r8, #256
	itt	ne
	movne	r3, #0
	strne	r3, [r10, r9]
.L1839:
	ldr	r3, [sp, #12]
	add	r3, r3, r6
	str	r3, [sp, #12]
	ldr	r3, [sp, #16]
	cmp	r3, #1
	beq	.L1841
	ldr	r3, [sp, #20]
	cbz	r3, .L1805
.L1841:
	ldrb	r3, [r5, #1888]	@ zero_extendqisi2
	cbz	r3, .L1805
	mov	r0, r4
	bl	flash_exit_slc_mode
.L1805:
	ldr	r3, [sp, #12]
	adds	r3, r3, #1
	str	r3, [sp, #12]
	b	.L1802
.L1897:
	movs	r0, #0
.L1895:
	add	sp, sp, #52
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1903:
	.align	2
.L1902:
	.word	.LC98
	.word	.LC100
	.word	.LANCHOR2
	.fnend
	.size	FlashReadPages, .-FlashReadPages
	.align	1
	.global	FlashLoadFactorBbt
	.thumb
	.thumb_func
	.type	FlashLoadFactorBbt, %function
FlashLoadFactorBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movs	r1, #16
	ldr	r7, .L1916
	.pad #52
	sub	sp, sp, #52
	ldr	r4, .L1916+4
	movs	r5, #0
	mov	fp, #-1
	ldrh	r3, [r7, #1874]
	addw	r0, r4, #4040
	ldrh	r6, [r7, #1872]
	mov	r10, r5
	mov	r8, r7
	mov	r9, r4
	smulbb	r6, r6, r3
	bl	__memzero
	ldr	r3, [r4, #4024]
	uxth	r6, r6
	str	r5, [sp, #20]
	str	r3, [sp, #24]
.L1905:
	ldrb	r3, [r8, #1612]	@ zero_extendqisi2
	uxtb	r7, r5
	cmp	r3, r7
	bls	.L1915
	mul	ip, r6, r7
	subs	r4, r6, #1
	sub	r3, r6, #12
	uxth	r4, r4
.L1906:
	cmp	r4, r3
	ble	.L1908
	add	r2, ip, r4
	movs	r1, #1
	add	r0, sp, #12
	str	r3, [sp, #4]
	lsls	r2, r2, #10
	str	r2, [sp, #16]
	mov	r2, r1
	str	ip, [sp]
	bl	FlashReadPages
	ldr	r2, [sp, #12]
	ldr	ip, [sp]
	adds	r2, r2, #1
	ldr	r3, [sp, #4]
	beq	.L1907
	ldr	r2, [r9, #4024]
	ldrh	r1, [r2]
	movw	r2, #61664
	cmp	r1, r2
	bne	.L1907
	mov	r1, r7
	ldr	r0, .L1916+8
	mov	r2, r4
	add	r10, r10, #1
	bl	printk
	ldr	r3, .L1916+4
	uxth	r10, r10
	add	r7, r3, r7, lsl #1
	strh	r4, [r7, #4040]	@ movhi
	b	.L1908
.L1907:
	subs	r4, r4, #1
	uxth	r4, r4
	b	.L1906
.L1908:
	ldrb	r3, [r8, #1612]	@ zero_extendqisi2
	adds	r5, r5, #1
	cmp	r3, r10
	it	eq
	moveq	fp, #0
	b	.L1905
.L1915:
	mov	r0, fp
	add	sp, sp, #52
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1917:
	.align	2
.L1916:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC102
	.fnend
	.size	FlashLoadFactorBbt, .-FlashLoadFactorBbt
	.align	1
	.global	FlashProgSlc2KPages
	.thumb
	.thumb_func
	.type	FlashProgSlc2KPages, %function
FlashProgSlc2KPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #68
	sub	sp, sp, #68
	ldr	r8, .L1948+20
	movs	r7, #0
	str	r3, [sp, #12]
	mov	r6, r1
	ldr	r3, .L1948
	mov	r9, r2
	mov	r4, r0
	mov	r10, r0
	mov	fp, r8
	ldrb	r3, [r3, #265]	@ zero_extendqisi2
	str	r3, [sp, #8]
.L1919:
	cmp	r7, r6
	beq	.L1947
	subs	r3, r6, r7
	add	r2, sp, #20
	mov	r0, r10
	mov	r1, r9
	uxtb	r3, r3
	str	r3, [sp]
	add	r3, sp, #24
	bl	LogAddr2PhyAddr
	ldrb	r2, [r8, #1612]	@ zero_extendqisi2
	ldr	r3, [sp, #24]
	cmp	r3, r2
	bcc	.L1920
	mov	r3, #-1
	str	r3, [r10]
	b	.L1921
.L1920:
	add	r3, r3, r8
	ldrb	r5, [r3, #1616]	@ zero_extendqisi2
	mov	r0, r5
	bl	NandcWaitFlashReady
	mov	r0, r5
	bl	NandcFlashCs
	mov	r0, r5
	ldr	r1, [sp, #20]
	bl	FlashProgFirstCmd
	ldr	r3, [r10, #12]
	movs	r1, #1
	mov	r0, r5
	ldr	r2, [sp, #8]
	str	r3, [sp]
	ldr	r3, [r10, #8]
	bl	NandcXferData
	mov	r0, r5
	ldr	r1, [sp, #20]
	bl	FlashProgSecondCmd
	mov	r0, r5
	bl	NandcWaitFlashReady
	mov	r0, r5
	ldr	r1, [sp, #20]
	bl	FlashReadStatus
	ldr	r3, [sp, #20]
	sbfx	r0, r0, #0, #1
	str	r0, [r10]
	mov	r0, r5
	ldr	r1, [r8, #4]
	add	r1, r1, r3
	bl	FlashProgFirstCmd
	ldr	r3, [r10, #8]
	cbz	r3, .L1922
	add	r3, r3, #2048
.L1922:
	ldr	r2, [r10, #12]
	cbz	r2, .L1923
	adds	r2, r2, #8
.L1923:
	str	r2, [sp]
	movs	r1, #1
	ldr	r2, [sp, #8]
	mov	r0, r5
	bl	NandcXferData
	ldr	r1, [fp, #4]
	mov	r0, r5
	ldr	r3, [sp, #20]
	add	r1, r1, r3
	bl	FlashProgSecondCmd
	mov	r0, r5
	bl	NandcWaitFlashReady
	mov	r0, r5
	ldr	r1, [sp, #20]
	bl	FlashReadStatus
	lsls	r2, r0, #31
	mov	r0, r5
	itt	mi
	movmi	r3, #-1
	strmi	r3, [r10]
	bl	NandcFlashDeCs
.L1921:
	adds	r7, r7, #1
	add	r10, r10, #36
	b	.L1919
.L1947:
	ldr	r3, [sp, #12]
	cmp	r3, #0
	beq	.L1946
	ldr	r8, .L1948+24
	mov	r10, #0
	mov	fp, r8
.L1926:
	cmp	r10, r6
	beq	.L1946
	ldr	r3, [r4]
	adds	r3, r3, #1
	bne	.L1927
	ldr	r1, [r4, #4]
	ldr	r0, .L1948+4
	bl	printk
	b	.L1928
.L1927:
	rsb	r3, r10, r6
	mov	r1, r9
	add	r2, sp, #20
	mov	r0, r4
	uxtb	r3, r3
	str	r3, [sp]
	add	r3, sp, #24
	mov	r7, r4
	bl	LogAddr2PhyAddr
	ldr	r2, [r8, #4028]
	movs	r3, #0
	str	r3, [r2]
	ldr	r2, [r8, #4032]
	str	r3, [r2]
	ldmia	r7!, {r0, r1, r2, r3}
	add	r5, sp, #28
	stmia	r5!, {r0, r1, r2, r3}
	ldmia	r7!, {r0, r1, r2, r3}
	stmia	r5!, {r0, r1, r2, r3}
	add	r0, sp, #28
	ldr	r3, [r7]
	movs	r1, #1
	mov	r2, r9
	str	r3, [r5]
	ldr	r3, [r8, #4028]
	str	r3, [sp, #36]
	ldr	r3, [r8, #4032]
	str	r3, [sp, #40]
	bl	FlashReadPages
	ldr	r5, [sp, #28]
	adds	r3, r5, #1
	bne	.L1929
	ldr	r0, .L1948+8
	ldr	r1, [r4, #4]
	bl	printk
	str	r5, [r4]
.L1929:
	ldr	r3, [r4, #12]
	cbz	r3, .L1930
	ldr	r2, [r3]
	ldr	r3, [fp, #4032]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L1930
	ldr	r0, .L1948+12
	ldr	r1, [r4, #4]
	bl	printk
	mov	r3, #-1
	str	r3, [r4]
.L1930:
	ldr	r3, [r4, #8]
	cbz	r3, .L1928
	ldr	r2, [r3]
	ldr	r3, [fp, #4028]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L1928
	ldr	r0, .L1948+16
	ldr	r1, [r4, #4]
	bl	printk
	mov	r3, #-1
	str	r3, [r4]
.L1928:
	add	r10, r10, #1
	adds	r4, r4, #36
	b	.L1926
.L1946:
	movs	r0, #0
	add	sp, sp, #68
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1949:
	.align	2
.L1948:
	.word	.LANCHOR1
	.word	.LC103
	.word	.LC104
	.word	.LC105
	.word	.LC106
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FlashProgSlc2KPages, .-FlashProgSlc2KPages
	.align	1
	.global	FtlLoadFactoryBbt
	.thumb
	.thumb_func
	.type	FtlLoadFactoryBbt, %function
FtlLoadFactoryBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movs	r6, #0
	ldr	r5, .L1961
	ldr	r7, .L1961+4
	ldr	r3, [r5, #1780]
	subw	r8, r7, #2978
	ldr	r9, [r5, #1804]
	mov	r10, r8
	str	r3, [r5, #4064]
	str	r9, [r5, #4068]
.L1951:
	ldrh	r3, [r8, #2862]
	cmp	r6, r3
	bcs	.L1960
	ldrh	r4, [r8, #2906]
	movw	fp, #61664
	movw	r3, #65535
	strh	r3, [r7, #2]!	@ movhi
	subs	r4, r4, #1
	uxth	r4, r4
.L1952:
	ldrh	r3, [r10, #2906]
	sub	r2, r3, #16
	cmp	r4, r2
	ble	.L1954
	mla	r3, r3, r6, r4
	movs	r1, #1
	ldr	r0, .L1961+8
	mov	r2, r1
	lsls	r3, r3, #10
	str	r3, [r5, #4060]
	bl	FlashReadPages
	ldr	r3, [r5, #4056]
	adds	r3, r3, #1
	beq	.L1953
	ldrh	r3, [r9]
	cmp	r3, fp
	bne	.L1953
	strh	r4, [r7]	@ movhi
	b	.L1954
.L1953:
	subs	r4, r4, #1
	uxth	r4, r4
	b	.L1952
.L1954:
	adds	r6, r6, #1
	b	.L1951
.L1960:
	movs	r0, #0
	pop	{r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1962:
	.align	2
.L1961:
	.word	.LANCHOR2
	.word	.LANCHOR0+2978
	.word	.LANCHOR2+4056
	.fnend
	.size	FtlLoadFactoryBbt, .-FtlLoadFactoryBbt
	.align	1
	.global	FtlGetLastWrittenPage
	.thumb
	.thumb_func
	.type	FtlGetLastWrittenPage, %function
FtlGetLastWrittenPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 104
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1974
	cmp	r1, #1
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #104
	sub	sp, sp, #104
	it	eq
	ldrheq	r4, [r3, #2910]
	mov	r8, r1
	it	ne
	ldrhne	r4, [r3, #2908]
	lsls	r6, r0, #10
	add	r3, sp, #40
	add	r0, sp, #4
	subs	r5, r4, #1
	str	r3, [sp, #16]
	movs	r1, #1
	mov	r2, r8
	uxth	r5, r5
	movs	r7, #0
	str	r7, [sp, #12]
	sxth	r3, r5
	orrs	r3, r3, r6
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #40]
	adds	r3, r3, #1
	bne	.L1966
.L1967:
	sxth	r3, r7
	sxth	r4, r5
	cmp	r3, r4
	bgt	.L1966
	add	r4, r4, r3
	add	r0, sp, #4
	movs	r1, #1
	mov	r2, r8
	add	r4, r4, r4, lsr #31
	asrs	r4, r4, #1
	sxth	r3, r4
	orrs	r3, r3, r6
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #40]
	adds	r3, r3, #1
	bne	.L1968
	ldr	r3, [sp, #44]
	adds	r3, r3, #1
	bne	.L1968
	ldr	r3, [sp, #4]
	adds	r3, r3, #1
	beq	.L1968
	subs	r4, r4, #1
	uxth	r5, r4
	b	.L1967
.L1968:
	adds	r3, r4, #1
	uxth	r7, r3
	b	.L1967
.L1966:
	sxth	r0, r5
	add	sp, sp, #104
	@ sp needed
	pop	{r4, r5, r6, r7, r8, pc}
.L1975:
	.align	2
.L1974:
	.word	.LANCHOR0
	.fnend
	.size	FtlGetLastWrittenPage, .-FtlGetLastWrittenPage
	.align	1
	.global	FtlLoadBbt
	.thumb
	.thumb_func
	.type	FtlLoadBbt, %function
FtlLoadBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	ldr	r4, .L2005
	ldr	r7, .L2005+4
	ldr	r3, [r4, #1780]
	mov	r8, r4
	ldr	r6, [r4, #1804]
	str	r3, [r4, #4064]
	str	r6, [r4, #4068]
	bl	FtlBbtMemInit
	ldrh	r5, [r7, #2906]
	subs	r5, r5, #1
	uxth	r5, r5
.L1977:
	ldrh	r3, [r7, #2906]
	subs	r3, r3, #48
	cmp	r5, r3
	ble	.L1980
	movs	r1, #1
	ldr	r0, .L2005+8
	lsls	r3, r5, #10
	str	r3, [r4, #4060]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r3, [r4, #4056]
	adds	r3, r3, #1
	bne	.L1978
	movs	r1, #1
	ldr	r3, [r8, #4060]
	ldr	r0, .L2005+8
	mov	r2, r1
	adds	r3, r3, #1
	str	r3, [r8, #4060]
	bl	FlashReadPages
.L1978:
	ldr	r3, [r4, #4056]
	adds	r3, r3, #1
	beq	.L1979
	ldrh	r2, [r6]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L1979
	ldr	r3, [r6, #4]
	strh	r5, [r7, #2968]	@ movhi
	str	r3, [r7, #2976]
	ldrh	r3, [r6, #8]
	strh	r3, [r7, #2972]	@ movhi
	b	.L1980
.L1979:
	subs	r5, r5, #1
	uxth	r5, r5
	b	.L1977
.L1980:
	ldrh	r3, [r7, #2968]
	movw	r2, #65535
	ldr	r5, .L2005+4
	cmp	r3, r2
	beq	.L1994
	ldrh	r3, [r5, #2972]
	cmp	r3, r2
	beq	.L1984
	movs	r1, #1
	ldr	r0, .L2005+8
	lsls	r3, r3, #10
	str	r3, [r4, #4060]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r3, [r4, #4056]
	adds	r3, r3, #1
	beq	.L1984
	ldrh	r2, [r6]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L1984
	ldr	r3, [r6, #4]
	ldr	r2, [r5, #2976]
	cmp	r3, r2
	itttt	hi
	strhi	r3, [r5, #2976]
	ldrhhi	r2, [r5, #2972]
	ldrhhi	r3, [r6, #8]
	strhhi	r2, [r5, #2968]	@ movhi
	it	hi
	strhhi	r3, [r5, #2972]	@ movhi
.L1984:
	ldrh	r0, [r7, #2968]
	movs	r1, #1
	bl	FtlGetLastWrittenPage
	movw	r8, #61649
	uxth	r5, r0
	adds	r0, r0, #1
	strh	r0, [r7, #2970]	@ movhi
.L1986:
	sxth	r3, r5
	cmp	r3, #0
	blt	.L1991
	ldrh	r2, [r7, #2968]
	movs	r1, #1
	ldr	r0, .L2005+8
	orr	r3, r3, r2, lsl #10
	str	r3, [r4, #4060]
	ldr	r3, [r4, #1780]
	mov	r2, r1
	str	r3, [r4, #4064]
	bl	FlashReadPages
	ldr	r3, [r4, #4056]
	adds	r3, r3, #1
	beq	.L1987
	ldrh	r3, [r6]
	cmp	r3, r8
	bne	.L1987
.L1991:
	ldrh	r2, [r6, #10]
	ldrh	r0, [r6, #12]
	ldr	r3, .L2005+4
	strh	r2, [r7, #2974]	@ movhi
	movw	r2, #65535
	cmp	r0, r2
	bne	.L1988
	b	.L1989
.L1987:
	subs	r5, r5, #1
	uxth	r5, r5
	b	.L1986
.L1988:
	ldr	r2, [r3, #2836]
	cmp	r0, r2
	beq	.L1989
	ldrh	r3, [r3, #2850]
	lsrs	r3, r3, #2
	cmp	r2, r3
	bcs	.L1989
	cmp	r0, r3
	bcs	.L1989
	bl	FtlSysBlkNumInit
.L1989:
	ldr	r6, .L2005+12
	movs	r5, #0
.L1992:
	ldrh	r3, [r7, #2862]
	cmp	r5, r3
	bcs	.L2004
	ldrh	r2, [r4, #1868]
	ldr	r1, [r4, #4064]
	ldr	r0, [r6, #4]!
	lsls	r2, r2, #2
	mla	r1, r5, r2, r1
	bl	ftl_memcpy
	adds	r5, r5, #1
	b	.L1992
.L2004:
	movs	r0, #0
	pop	{r4, r5, r6, r7, r8, pc}
.L1994:
	mov	r0, #-1
	pop	{r4, r5, r6, r7, r8, pc}
.L2006:
	.align	2
.L2005:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2+4056
	.word	.LANCHOR0+2992
	.fnend
	.size	FtlLoadBbt, .-FtlLoadBbt
	.align	1
	.global	FtlScanSysBlk
	.thumb
	.thumb_func
	.type	FtlScanSysBlk, %function
FtlScanSysBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movs	r6, #0
	ldr	r5, .L2092
	.pad #36
	sub	sp, sp, #36
	ldr	r4, .L2092+4
	mov	r1, r6
	ldr	r2, [r5, #2936]
	ldr	r0, [r4, #1860]
	mov	r9, r4
	strh	r6, [r5, #2950]	@ movhi
	lsls	r2, r2, #2
	strh	r6, [r4, #1916]	@ movhi
	bl	ftl_memset
	ldr	r2, [r5, #2936]
	mov	r1, r6
	ldr	r0, [r4, #1832]
	lsls	r2, r2, #1
	bl	ftl_memset
	ldrh	r2, [r5, #2928]
	mov	r1, r6
	ldr	r0, [r4, #1844]
	lsls	r2, r2, #2
	bl	ftl_memset
	ldrh	r2, [r5, #2928]
	ldr	r0, [r5, #2952]
	mov	r1, r6
	lsls	r2, r2, #1
	bl	ftl_memset
	add	r0, r4, #1056
	movs	r1, #255
	movs	r2, #16
	bl	memset
	ldrh	r3, [r5, #2848]
	str	r5, [sp, #12]
	mov	r5, r4
	str	r3, [sp, #4]
.L2008:
	ldr	r6, .L2092
	ldr	r2, [sp, #4]
	ldrh	r3, [r6, #2850]
	mov	r0, r6
	cmp	r3, r2
	bls	.L2050
	ldr	r1, [r4, #1664]
	mov	r10, #36
	ldrh	r8, [r6, #2840]
	movs	r7, #0
	ldr	r2, [r4, #1768]
	str	r1, [sp, #8]
	ldr	r3, [r4, #1660]
	ldrh	r1, [r6, #2920]
	str	r7, [sp]
	str	r1, [sp, #16]
	addw	r1, r6, #2868
	str	r1, [sp, #20]
.L2051:
	uxth	r1, r7
	cmp	r1, r8
	bcs	.L2087
	str	r3, [sp, #28]
	ldr	r3, [sp, #20]
	ldr	r1, [sp, #4]
	str	r2, [sp, #24]
	ldrb	r0, [r3, r7]	@ zero_extendqisi2
	bl	V2P_block
	mov	fp, r0
	bl	FtlBbmIsBadBlock
	ldr	r2, [sp, #24]
	ldr	r3, [sp, #28]
	cbnz	r0, .L2009
	ldr	r1, [sp]
	ldr	r6, [sp]
	mla	r0, r10, r1, r2
	lsl	r1, fp, #10
	str	r1, [r0, #4]
	ldr	r1, [sp, #16]
	muls	r1, r6, r1
	ldr	r6, [sp, #8]
	it	mi
	addmi	r1, r1, #3
	bic	r1, r1, #3
	str	r3, [r0, #8]
	add	r1, r1, r6
	str	r1, [r0, #12]
	ldr	r1, [sp]
	adds	r1, r1, #1
	uxth	r1, r1
	str	r1, [sp]
.L2009:
	adds	r7, r7, #1
	b	.L2051
.L2087:
	ldr	r3, [sp]
	cmp	r3, #0
	beq	.L2013
	mov	r1, r3
	ldr	r0, [r4, #1768]
	movs	r2, #1
	ldr	r8, .L2092
	bl	FlashReadPages
	movs	r3, #0
	str	r3, [sp, #8]
.L2014:
	ldrh	r3, [sp, #8]
	ldr	r2, [sp]
	cmp	r3, r2
	bcs	.L2013
	ldr	r3, [sp, #8]
	mov	r10, #36
	mul	r10, r10, r3
	ldr	r3, [r4, #1768]
	add	r2, r3, r10
	ldr	r3, [r3, r10]
	ldr	r6, [r2, #4]
	adds	r3, r3, #1
	ldr	r7, [r2, #12]
	ubfx	r6, r6, #10, #16
	bne	.L2016
	mov	fp, #16
	movw	r3, #65535
.L2015:
	ldr	r0, [r4, #1768]
	movs	r1, #1
	str	r3, [sp, #16]
	add	r0, r0, r10
	ldr	r2, [r0, #4]
	adds	r2, r2, #1
	str	r2, [r0, #4]
	mov	r2, r1
	bl	FlashReadPages
	ldrh	r2, [r7]
	ldr	r3, [sp, #16]
	cmp	r2, r3
	bne	.L2017
	ldr	r3, [r9, #1768]
	mov	r2, #-1
	str	r2, [r3, r10]
	b	.L2016
.L2017:
	ldr	r2, [r5, #1768]
	ldr	r2, [r2, r10]
	adds	r2, r2, #1
	bne	.L2016
	add	fp, fp, #-1
	uxth	fp, fp
	cmp	fp, #0
	bne	.L2015
.L2016:
	ldr	r3, [r5, #1768]
	ldr	r3, [r3, r10]
	adds	r3, r3, #1
	beq	.L2019
	ldr	r2, [r5, #1024]
	ldr	r3, [r7, #4]
	adds	r1, r2, #1
	beq	.L2020
	cmp	r2, r3
	bhi	.L2021
.L2020:
	adds	r2, r3, #1
	itt	ne
	addne	r2, r3, #1
	strne	r2, [r9, #1024]
.L2021:
	ldrh	r2, [r7]
	movw	r1, #61604
	cmp	r2, r1
	beq	.L2023
	bhi	.L2024
	movw	r3, #61574
	cmp	r2, r3
	bne	.L2022
	ldr	r3, [sp, #12]
	ldr	r2, [sp, #12]
	ldrh	r10, [r3, #2928]
	ldrh	r0, [r2, #2950]
	ldr	r2, [r9, #1844]
	add	lr, r10, #-1
	uxth	r3, lr
	rsb	lr, r0, lr
	str	r2, [sp, #16]
	b	.L2037
.L2024:
	movw	r3, #61634
	cmp	r2, r3
	beq	.L2026
	movw	r3, #65535
	cmp	r2, r3
	bne	.L2022
	mov	r0, r6
	b	.L2086
.L2026:
	ldr	r3, [sp, #12]
	ldrh	r0, [r4, #1916]
	ldr	r2, [r4, #1860]
	ldr	r1, [r3, #2936]
	str	r2, [sp, #16]
	uxth	lr, r1
	add	r3, lr, #-1
	rsb	lr, r0, lr
	add	lr, lr, #-1
	uxth	r3, r3
	sxth	lr, lr
.L2028:
	sxth	ip, r3
	cmp	ip, lr
	ble	.L2088
	ldr	r2, [r7, #4]
	lsl	fp, ip, #2
	mov	r10, r2
	ldr	r2, [sp, #16]
	ldr	r2, [r2, ip, lsl #2]
	cmp	r10, r2
	bls	.L2029
	ldr	r2, [sp, #16]
	ldr	r2, [r2]
	cbnz	r2, .L2030
	cmp	r0, r1
	itt	ne
	addne	r0, r0, #1
	strhne	r0, [r9, #1916]	@ movhi
.L2030:
	movs	r1, #0
	uxth	lr, r3
.L2031:
	uxth	r2, r1
	cmp	r2, lr
	bcs	.L2089
	ldr	r0, [r5, #1860]
	sxth	r2, r2
	adds	r1, r1, #1
	add	r10, r0, r2, lsl #2
	ldr	r10, [r10, #4]
	str	r10, [r0, r2, lsl #2]
	ldr	r0, [r5, #1832]
	add	r10, r0, r2, lsl #1
	ldrh	r10, [r10, #2]
	strh	r10, [r0, r2, lsl #1]	@ movhi
	b	.L2031
.L2089:
	ldr	r1, [r7, #4]
	ldr	r2, [r4, #1860]
	str	r1, [r2, fp]
	sxth	r1, r3
	ldr	r2, [r4, #1832]
	cmp	r1, #0
	strh	r6, [r2, ip, lsl #1]	@ movhi
	bge	.L2033
	b	.L2022
.L2029:
	subs	r3, r3, #1
	uxth	r3, r3
	b	.L2028
.L2088:
	cmp	ip, #0
	bge	.L2064
	b	.L2022
.L2033:
	ldrh	r0, [r5, #1916]
	ldr	r2, [r8, #2936]
	subs	r2, r2, r0
	subs	r2, r2, #1
	sxth	r2, r2
	cmp	r1, r2
	bgt	.L2022
.L2064:
	ldr	r2, [r4, #1860]
	sxth	r3, r3
	ldr	r1, [r7, #4]
	adds	r0, r0, #1
	strh	r0, [r4, #1916]	@ movhi
	str	r1, [r2, r3, lsl #2]
	ldr	r2, [r4, #1832]
	b	.L2084
.L2043:
	ldr	r1, [sp, #16]
	lsl	fp, ip, #2
	ldr	r2, [r7, #4]
	ldr	r1, [r1, ip, lsl #2]
	cmp	r2, r1
	bhi	.L2090
	subs	r3, r3, #1
	uxth	r3, r3
.L2037:
	sxth	ip, r3
	cmp	ip, lr
	bgt	.L2043
	b	.L2042
.L2093:
	.align	2
.L2092:
	.word	.LANCHOR0
	.word	.LANCHOR2
.L2090:
	ldr	r2, [sp, #16]
	ldr	r2, [r2]
	cbnz	r2, .L2039
	cmp	r0, r10
	itt	ne
	addne	r0, r0, #1
	strhne	r0, [r8, #2950]	@ movhi
.L2039:
	movs	r1, #0
	uxth	lr, r3
.L2040:
	uxth	r2, r1
	cmp	r2, lr
	bcs	.L2091
	ldr	r0, [r4, #1844]
	sxth	r2, r2
	adds	r1, r1, #1
	add	r10, r0, r2, lsl #2
	ldr	r10, [r10, #4]
	str	r10, [r0, r2, lsl #2]
	ldr	r0, [r8, #2952]
	add	r10, r0, r2, lsl #1
	ldrh	r10, [r10, #2]
	strh	r10, [r0, r2, lsl #1]	@ movhi
	b	.L2040
.L2091:
	ldr	r2, [r5, #1844]
	ldr	r1, [r7, #4]
	str	r1, [r2, fp]
	ldr	r2, .L2094
	ldr	r2, [r2, #2952]
	strh	r6, [r2, ip, lsl #1]	@ movhi
.L2042:
	sxth	r3, r3
	cmp	r3, #0
	blt	.L2022
	ldrh	r2, [r8, #2928]
	ldrh	r1, [r8, #2950]
	subs	r2, r2, #1
	subs	r2, r2, r1
	sxth	r2, r2
	cmp	r3, r2
	bgt	.L2022
	adds	r1, r1, #1
	ldr	r2, [r9, #1844]
	strh	r1, [r8, #2950]	@ movhi
	ldr	r1, [r7, #4]
	str	r1, [r2, r3, lsl #2]
	ldr	r2, [r8, #2952]
.L2084:
	strh	r6, [r2, r3, lsl #1]	@ movhi
	b	.L2022
.L2023:
	ldrh	r1, [r9, #1056]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L2044
	strh	r6, [r5, #1056]	@ movhi
	b	.L2085
.L2044:
	ldrh	r0, [r5, #1060]
	cmp	r0, r2
	beq	.L2045
	movs	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L2045:
	ldr	r2, [r9, #1064]
	ldr	r3, [r7, #4]
	cmp	r2, r3
	bcs	.L2046
	ldrh	r3, [r5, #1056]
	strh	r6, [r5, #1056]	@ movhi
	strh	r3, [r5, #1060]	@ movhi
	ldr	r3, [r7, #4]
.L2085:
	str	r3, [r5, #1064]
	b	.L2022
.L2046:
	strh	r6, [r5, #1060]	@ movhi
	b	.L2022
.L2019:
	ldrb	r1, [r8, #1888]	@ zero_extendqisi2
	mov	r0, r6
	cbz	r1, .L2047
.L2086:
	movs	r1, #0
.L2047:
	bl	FtlFreeSysBlkQueueIn
.L2022:
	ldr	r3, [sp, #8]
	adds	r3, r3, #1
	str	r3, [sp, #8]
	b	.L2014
.L2013:
	ldr	r6, [sp, #4]
	adds	r6, r6, #1
	uxth	r3, r6
	str	r3, [sp, #4]
	b	.L2008
.L2050:
	ldr	r3, .L2094+4
	ldr	r5, [r3, #1832]
	ldrh	r2, [r5]
	cbz	r2, .L2052
.L2055:
	ldr	r5, [r0, #2952]
	ldr	r3, .L2094
	ldrh	r2, [r5]
	cbz	r2, .L2053
	b	.L2076
.L2052:
	ldrh	r3, [r3, #1916]
	cmp	r3, #0
	beq	.L2055
	ldr	r6, [r6, #2936]
.L2056:
	uxth	r3, r2
	sxth	r1, r3
	cmp	r1, r6
	bcs	.L2055
	ldrh	r7, [r5, r1, lsl #1]
	adds	r2, r2, #1
	cmp	r7, #0
	beq	.L2056
	mov	lr, #0
.L2057:
	ldr	r5, [r0, #2936]
	sxth	r2, r3
	cmp	r2, r5
	bcs	.L2055
	ldr	r5, [r4, #1832]
	subs	r6, r2, r1
	adds	r3, r3, #1
	ldrh	r7, [r5, r2, lsl #1]
	uxth	r3, r3
	strh	r7, [r5, r6, lsl #1]	@ movhi
	ldr	r5, [r4, #1860]
	ldr	r7, [r5, r2, lsl #2]
	str	r7, [r5, r6, lsl #2]
	ldr	r5, [r4, #1832]
	strh	lr, [r5, r2, lsl #1]	@ movhi
	b	.L2057
.L2053:
	ldrh	r1, [r3, #2950]
	cbz	r1, .L2076
	ldrh	r6, [r3, #2928]
.L2060:
	uxth	r3, r2
	sxth	r1, r3
	cmp	r1, r6
	bge	.L2076
	ldrh	r7, [r5, r1, lsl #1]
	adds	r2, r2, #1
	cmp	r7, #0
	beq	.L2060
	ldr	r5, .L2094
	mov	ip, #0
.L2061:
	ldrh	r6, [r0, #2928]
	sxth	r2, r3
	cmp	r2, r6
	bge	.L2076
	ldr	r6, [r5, #2952]
	rsb	lr, r1, r2
	adds	r3, r3, #1
	ldrh	r7, [r6, r2, lsl #1]
	uxth	r3, r3
	strh	r7, [r6, lr, lsl #1]	@ movhi
	ldr	r6, [r4, #1844]
	ldr	r7, [r6, r2, lsl #2]
	str	r7, [r6, lr, lsl #2]
	ldr	r6, [r5, #2952]
	strh	ip, [r6, r2, lsl #1]	@ movhi
	b	.L2061
.L2076:
	movs	r0, #0
	add	sp, sp, #36
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2095:
	.align	2
.L2094:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlScanSysBlk, .-FtlScanSysBlk
	.align	1
	.global	FtlLoadSysInfo
	.thumb
	.thumb_func
	.type	FtlLoadSysInfo, %function
FtlLoadSysInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	movs	r1, #0
	ldr	r4, .L2124
	ldr	r6, .L2124+4
	ldr	r3, [r4, #1780]
	mov	r5, r4
	ldrh	r2, [r6, #2848]
	ldr	r0, [r4, #812]
	str	r3, [r4, #4064]
	ldr	r3, [r4, #1804]
	lsls	r2, r2, #1
	str	r3, [r4, #4068]
	bl	ftl_memset
	ldrh	r0, [r4, #1056]
	movw	r3, #65535
	cmp	r0, r3
	bne	.L2097
.L2108:
	mov	r0, #-1
	b	.L2098
.L2097:
	movs	r1, #1
	ldr	r10, .L2124+12
	bl	FtlGetLastWrittenPage
	ldrh	r8, [r5, #1056]
	mov	r7, r5
	uxth	r4, r0
	adds	r0, r0, #1
	strh	r0, [r5, #1058]	@ movhi
.L2099:
	sxth	r3, r4
	cmp	r3, #0
	blt	.L2107
	sxth	r9, r8
	movs	r1, #1
	ldr	r0, .L2124+8
	orr	r3, r3, r9, lsl #10
	str	r3, [r5, #4060]
	ldr	r3, [r5, #1780]
	mov	r2, r1
	ldr	fp, .L2124+4
	str	r3, [r5, #4064]
	bl	FlashReadPages
	ldrb	r3, [r6]	@ zero_extendqisi2
	cbz	r3, .L2100
	ldr	r3, [r7, #4068]
	ldr	r3, [r3, #12]
	cbz	r3, .L2100
	ldr	r0, [r7, #4064]
	ldrh	r1, [fp, #2918]
	str	r3, [sp, #4]
	bl	js_hash
	ldr	r3, [sp, #4]
	cmp	r3, r0
	beq	.L2100
	cbnz	r4, .L2101
	ldrh	r3, [r7, #1060]
	cmp	r9, r3
	beq	.L2101
	uxth	r8, r3
	ldrh	r4, [fp, #2910]
	b	.L2103
.L2101:
	mov	r3, #-1
	str	r3, [r5, #4056]
.L2100:
	ldr	r3, [r5, #4056]
	adds	r3, r3, #1
	beq	.L2103
	ldr	r3, [r7, #1780]
	ldr	r3, [r3]
	cmp	r3, r10
	bne	.L2103
	ldr	r3, [r7, #1804]
	ldrh	r2, [r3]
	movw	r3, #61604
	cmp	r2, r3
	bne	.L2103
.L2107:
	ldr	r4, .L2124
	movs	r2, #48
	ldr	r1, [r5, #4064]
	add	r0, r4, #756
	ldr	r7, .L2124+4
	bl	ftl_memcpy
	ldrh	r2, [r6, #2848]
	ldr	r1, [r5, #4064]
	ldr	r0, [r5, #812]
	adds	r1, r1, #48
	lsls	r2, r2, #1
	bl	ftl_memcpy
	ldrh	r2, [r6, #2848]
	ldr	r3, [r5, #4064]
	add	r1, r2, #24
	ldr	r0, [r5, #988]
	ubfx	r1, r1, #1, #30
	lsrs	r2, r2, #3
	adds	r2, r2, #4
	add	r1, r3, r1, lsl #2
	bl	ftl_memcpy
	ldrh	r3, [r6, #2948]
	cbz	r3, .L2105
	ldrh	r2, [r7, #2848]
	ldr	r1, [r4, #4064]
	ldr	r0, [r4, #1856]
	lsrs	r3, r2, #3
	add	r3, r3, r2, lsl #1
	ldrh	r2, [r7, #2944]
	adds	r3, r3, #52
	ubfx	r3, r3, #2, #14
	lsls	r2, r2, #2
	add	r1, r1, r3, lsl #2
	bl	ftl_memcpy
	b	.L2105
.L2103:
	subs	r4, r4, #1
	uxth	r4, r4
	b	.L2099
.L2105:
	ldr	r2, [r5, #756]
	ldr	r3, .L2124+12
	ldr	r4, .L2124
	cmp	r2, r3
	bne	.L2108
	ldrb	r0, [r4, #766]	@ zero_extendqisi2
	ldrh	r1, [r6, #2862]
	ldrh	r2, [r4, #764]
	cmp	r0, r1
	ldr	r3, .L2124+4
	strh	r2, [r4, #1062]	@ movhi
	bne	.L2108
	ldrh	r1, [r3, #2908]
	ldrh	r0, [r3, #2914]
	str	r2, [r4, #4092]
	muls	r1, r2, r1
	str	r1, [r3, #2964]
	muls	r1, r0, r1
	ldrh	r0, [r3, #2974]
	str	r1, [r3, #1892]
	ldr	r1, [r3, #2852]
	subs	r0, r1, r0
	ldrh	r1, [r3, #2840]
	subs	r0, r0, r2
	bl	__aeabi_uidiv
	ldrh	r3, [r4, #772]
	ldrh	r1, [r4, #770]
	lsrs	r2, r3, #6
	strh	r2, [r4, #834]	@ movhi
	ldrh	r2, [r4, #774]
	and	r3, r3, #63
	strb	r3, [r4, #838]
	ldrb	r3, [r4, #767]	@ zero_extendqisi2
	strh	r2, [r4, #880]	@ movhi
	ldrh	r2, [r4, #776]
	strb	r3, [r4, #840]
	movw	r3, #65535
	strh	r1, [r4, #832]	@ movhi
	strh	r3, [r4, #1072]	@ movhi
	movs	r3, #0
	strh	r3, [r4, #1074]	@ movhi
	strb	r3, [r4, #1078]
	strb	r3, [r4, #1080]
	str	r3, [r4, #1012]
	strh	r0, [r4, #1052]	@ movhi
	lsrs	r0, r2, #6
	and	r2, r2, #63
	strb	r2, [r4, #886]
	ldrb	r2, [r4, #768]	@ zero_extendqisi2
	strh	r0, [r4, #882]	@ movhi
	strb	r2, [r4, #888]
	ldrh	r2, [r4, #778]
	strh	r2, [r4, #928]	@ movhi
	ldrh	r2, [r4, #780]
	lsrs	r0, r2, #6
	and	r2, r2, #63
	strb	r2, [r4, #934]
	ldrb	r2, [r4, #769]	@ zero_extendqisi2
	strh	r0, [r4, #930]	@ movhi
	strb	r2, [r4, #936]
	str	r3, [r4, #1000]
	ldr	r2, [r4, #788]
	str	r3, [r4, #992]
	str	r3, [r4, #1008]
	str	r3, [r4, #1036]
	str	r3, [r4, #1044]
	str	r3, [r4, #1004]
	ldr	r3, [r4, #796]
	str	r2, [r4, #1032]
	ldr	r2, [r4, #1024]
	cmp	r3, r2
	ldr	r2, [r5, #1028]
	it	hi
	strhi	r3, [r4, #1024]
	ldr	r3, [r5, #792]
	cmp	r3, r2
	itt	hi
	ldrhi	r2, .L2124
	strhi	r3, [r2, #1028]
	movw	r3, #65535
	cmp	r1, r3
	beq	.L2111
	ldr	r0, .L2124+16
	bl	make_superblock
.L2111:
	ldrh	r2, [r5, #880]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L2112
	ldr	r0, .L2124+20
	bl	make_superblock
.L2112:
	ldrh	r2, [r5, #928]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L2113
	ldr	r0, .L2124+24
	bl	make_superblock
.L2113:
	ldrh	r2, [r5, #1072]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L2114
	ldr	r0, .L2124+28
	bl	make_superblock
.L2114:
	movs	r0, #0
.L2098:
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2125:
	.align	2
.L2124:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2+4056
	.word	1179929683
	.word	.LANCHOR2+832
	.word	.LANCHOR2+880
	.word	.LANCHOR2+928
	.word	.LANCHOR2+1072
	.fnend
	.size	FtlLoadSysInfo, .-FtlLoadSysInfo
	.align	1
	.global	FtlDumpBlockInfo
	.thumb
	.thumb_func
	.type	FtlDumpBlockInfo, %function
FtlDumpBlockInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 72
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	ubfx	r0, r0, #10, #16
	.pad #100
	sub	sp, sp, #100
	mov	r8, r1
	bl	P2V_block_in_plane
	ldr	r4, .L2142
	ldr	r7, .L2142+4
	ldr	r1, .L2142+8
	ldrh	r9, [r4, #2908]
	mov	r6, r0
	ldr	r0, .L2142+12
	bl	printk
	ldr	r3, [r7, #812]
	mov	r1, r6
	ldr	r0, .L2142+16
	ldrh	r2, [r3, r6, lsl #1]
	bl	printk
	add	r0, sp, #96
	strh	r6, [r0, #-48]!	@ movhi
	bl	make_superblock
	ldrb	r5, [r4, #1888]	@ zero_extendqisi2
	str	r7, [sp, #44]
	cbz	r5, .L2127
	cmp	r8, #0
	bne	.L2138
	mov	r0, r6
	bl	ftl_get_blk_mode
	cmp	r0, #1
	mov	r5, r0
	bne	.L2127
	ldrh	r9, [r4, #2910]
	b	.L2127
.L2138:
	movs	r5, #0
.L2127:
	ldr	r0, .L2142+20
	mov	r1, r5
	mov	r2, r9
	ldrh	r3, [r4, #2908]
	bl	printk
	ldr	r10, .L2142
	movs	r6, #0
.L2128:
	ldr	r3, .L2142+4
	movs	r2, #0
	ldrh	fp, [r10, #2840]
	add	lr, sp, #62
	ldrh	r0, [r10, #2920]
	mov	r8, #36
	ldr	r3, [r3, #1768]
	mov	r4, r2
	movw	ip, #65535
	str	r3, [sp, #28]
	ldr	r3, .L2142+4
	ldr	r3, [r3, #1660]
	str	r3, [sp, #32]
	ldrh	r3, [r10, #2918]
	str	r3, [sp, #36]
	ldr	r3, .L2142+4
	ldr	r3, [r3, #1664]
	str	r3, [sp, #40]
.L2129:
	uxth	r3, r2
	cmp	r3, fp
	bcs	.L2140
	ldrh	r3, [lr, #2]!
	cmp	r3, ip
	beq	.L2130
	ldr	r1, [sp, #28]
	orr	r3, r6, r3, lsl #10
	ldr	r7, [sp, #32]
	mla	r1, r8, r4, r1
	str	r3, [r1, #4]
	ldr	r3, [sp, #36]
	muls	r3, r4, r3
	it	mi
	addmi	r3, r3, #3
	bic	r3, r3, #3
	add	r3, r3, r7
	str	r3, [r1, #8]
	mov	r3, r0
	muls	r3, r4, r3
	add	r4, r4, #1
	it	mi
	addmi	r3, r3, #3
	ldr	r7, [sp, #40]
	bic	r3, r3, #3
	uxth	r4, r4
	add	r3, r3, r7
	str	r3, [r1, #12]
.L2130:
	adds	r2, r2, #1
	b	.L2129
.L2140:
	ldr	r3, .L2142+4
	mov	r1, r4
	mov	r2, r5
	mov	r8, #0
	mov	fp, #36
	ldr	r0, [r3, #1768]
	bl	FlashReadPages
.L2134:
	uxth	r3, r8
	cmp	r3, r4
	bcs	.L2141
	ldr	r3, [sp, #44]
	mul	r2, fp, r8
	ldrh	r1, [sp, #48]
	ldr	ip, [r3, #1768]
	add	r8, r8, #1
	add	lr, ip, r2
	ldr	r3, [lr, #12]
	ldr	r0, [lr, #8]
	ldr	r7, [r3]
	str	r7, [sp]
	ldr	r7, [r3, #4]
	str	r7, [sp, #4]
	ldr	r7, [r3, #8]
	str	r7, [sp, #8]
	ldr	r3, [r3, #12]
	str	r3, [sp, #12]
	ldr	r3, [r0]
	str	r3, [sp, #16]
	ldr	r3, [r0, #4]
	ldr	r0, .L2142+24
	str	r3, [sp, #20]
	ldr	r2, [ip, r2]
	ldr	r3, [lr, #4]
	bl	printk
	b	.L2134
.L2141:
	adds	r6, r6, #1
	uxth	r6, r6
	cmp	r6, r9
	bne	.L2128
.L2136:
	movs	r0, #0
	add	sp, sp, #100
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2143:
	.align	2
.L2142:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR3+16
	.word	.LC107
	.word	.LC108
	.word	.LC109
	.word	.LC110
	.fnend
	.size	FtlDumpBlockInfo, .-FtlDumpBlockInfo
	.align	1
	.global	FtlScanAllBlock
	.thumb
	.thumb_func
	.type	FtlScanAllBlock, %function
FtlScanAllBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #92
	sub	sp, sp, #92
	ldr	r0, .L2160
	mov	r8, #0
	ldr	r1, .L2160+4
	bl	printk
	ldr	r5, .L2160+8
	mov	r7, r5
.L2145:
	ldr	r3, .L2160+12
	uxth	r4, r8
	ldrh	r3, [r3, #2850]
	cmp	r3, r4
	bls	.L2156
	add	r9, sp, #88
	mov	r0, r4
	mov	fp, #36
	movw	r10, #65535
	strh	r4, [r9, #-48]!	@ movhi
	bl	ftl_get_blk_mode
	ldr	r2, [r5, #812]
	mov	r1, r4
	ldrh	r2, [r2, r4, lsl #1]
	mov	r3, r0
	ldr	r0, .L2160+16
	bl	printk
	mov	r0, r9
	bl	make_superblock
	ldr	r3, .L2160+12
	movs	r2, #0
	ldr	r9, [r5, #1664]
	ldrh	lr, [r3, #2840]
	mov	r4, r2
	ldr	r3, [r5, #1768]
	str	r3, [sp, #28]
	add	ip, sp, #54
	ldr	r3, [r5, #1660]
	str	r3, [sp, #32]
	ldr	r3, .L2160+12
	ldrh	r0, [r3, #2918]
	ldrh	r3, [r3, #2920]
	str	r3, [sp, #36]
.L2146:
	uxth	r3, r2
	cmp	r3, lr
	bcs	.L2157
	ldrh	r3, [ip, #2]!
	cmp	r3, r10
	beq	.L2147
	ldr	r1, [sp, #28]
	lsls	r3, r3, #10
	ldr	r6, [sp, #32]
	mla	r1, fp, r4, r1
	str	r3, [r1, #4]
	mov	r3, r0
	muls	r3, r4, r3
	it	mi
	addmi	r3, r3, #3
	bic	r3, r3, #3
	add	r3, r3, r6
	str	r3, [r1, #8]
	ldr	r3, [sp, #36]
	muls	r3, r4, r3
	add	r4, r4, #1
	it	mi
	addmi	r3, r3, #3
	bic	r3, r3, #3
	uxth	r4, r4
	add	r3, r3, r9
	str	r3, [r1, #12]
.L2147:
	adds	r2, r2, #1
	b	.L2146
.L2157:
	ldr	r0, [r7, #1768]
	mov	r1, r4
	movs	r2, #0
	mov	r9, #0
	bl	FlashReadPages
	mov	r10, #36
.L2151:
	uxth	r3, r9
	cmp	r3, r4
	bcs	.L2158
	mul	r2, r10, r9
	ldr	ip, [r7, #1768]
	ldrh	r1, [sp, #40]
	add	r9, r9, #1
	add	lr, ip, r2
	ldr	r3, [lr, #12]
	ldr	r0, [lr, #8]
	ldr	r6, [r3]
	str	r6, [sp]
	ldr	r6, [r3, #4]
	str	r6, [sp, #4]
	ldr	r6, [r3, #8]
	str	r6, [sp, #8]
	ldr	r3, [r3, #12]
	str	r3, [sp, #12]
	ldr	r3, [r0]
	str	r3, [sp, #16]
	ldr	r3, [r0, #4]
	ldr	r0, .L2160+20
	str	r3, [sp, #20]
	ldr	r2, [ip, r2]
	ldr	r3, [lr, #4]
	bl	printk
	b	.L2151
.L2158:
	ldr	r0, [r7, #1768]
	mov	r1, r4
	movs	r2, #1
	mov	r9, #0
	bl	FlashReadPages
	mov	r10, #36
.L2153:
	uxth	r3, r9
	cmp	r3, r4
	bcs	.L2159
	mul	r2, r10, r9
	ldr	ip, [r7, #1768]
	ldrh	r1, [sp, #40]
	add	r9, r9, #1
	add	lr, ip, r2
	ldr	r3, [lr, #12]
	ldr	r0, [lr, #8]
	ldr	r6, [r3]
	str	r6, [sp]
	ldr	r6, [r3, #4]
	str	r6, [sp, #4]
	ldr	r6, [r3, #8]
	str	r6, [sp, #8]
	ldr	r3, [r3, #12]
	str	r3, [sp, #12]
	ldr	r3, [r0]
	str	r3, [sp, #16]
	ldr	r3, [r0, #4]
	ldr	r0, .L2160+24
	str	r3, [sp, #20]
	ldr	r2, [ip, r2]
	ldr	r3, [lr, #4]
	bl	printk
	b	.L2153
.L2159:
	add	r8, r8, #1
	b	.L2145
.L2156:
	movs	r0, #0
	add	sp, sp, #92
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2161:
	.align	2
.L2160:
	.word	.LC107
	.word	.LANCHOR3+36
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC111
	.word	.LC112
	.word	.LC113
	.fnend
	.size	FtlScanAllBlock, .-FtlScanAllBlock
	.align	1
	.global	ftl_scan_all_ppa
	.thumb
	.thumb_func
	.type	ftl_scan_all_ppa, %function
ftl_scan_all_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #24
	sub	sp, sp, #24
	ldr	r6, .L2168
	movs	r7, #0
	ldr	r0, .L2168+4
	ldr	r4, .L2168+8
	ldrh	r1, [r6, #2906]
	ldrh	r2, [r6, #2908]
	bl	printk
.L2163:
	ldrh	r3, [r6, #2906]
	cmp	r7, r3
	bcs	.L2165
	lsl	r8, r7, #10
	movs	r5, #0
.L2166:
	ldrh	r3, [r6, #2908]
	cmp	r5, r3
	bcs	.L2167
	add	r3, r5, r8
	str	r3, [r4, #4060]
	ldr	r3, [r4, #1780]
	movs	r2, #0
	movs	r1, #1
	ldr	r0, .L2168+12
	str	r2, [r4, #4056]
	adds	r5, r5, #1
	str	r3, [r4, #4064]
	ldr	r3, [r4, #1804]
	str	r3, [r4, #4068]
	bl	FlashReadPages
	ldr	r3, [r4, #4068]
	ldr	r2, [r4, #4064]
	ldr	r0, .L2168+16
	ldr	r1, [r3, #4]
	str	r1, [sp]
	ldr	r1, [r3, #8]
	str	r1, [sp, #4]
	ldr	r1, [r3, #12]
	str	r1, [sp, #8]
	ldr	r1, [r2]
	str	r1, [sp, #12]
	ldr	r2, [r2, #4]
	ldr	r1, [r4, #4060]
	str	r2, [sp, #16]
	ldr	r2, [r4, #4056]
	ldr	r3, [r3]
	bl	printk
	b	.L2166
.L2167:
	adds	r7, r7, #1
	b	.L2163
.L2165:
	ldr	r0, .L2168+20
	ldr	r1, .L2168+24
	add	sp, sp, #24
	@ sp needed
	pop	{r4, r5, r6, r7, r8, lr}
	b	printk
.L2169:
	.align	2
.L2168:
	.word	.LANCHOR0
	.word	.LC114
	.word	.LANCHOR2
	.word	.LANCHOR2+4056
	.word	.LC115
	.word	.LC116
	.word	.LANCHOR3+52
	.fnend
	.size	ftl_scan_all_ppa, .-ftl_scan_all_ppa
	.align	1
	.global	FlashProgPages
	.thumb
	.thumb_func
	.type	FlashProgPages, %function
FlashProgPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #76
	sub	sp, sp, #76
	ldr	r5, .L2225
	mov	r4, r0
	str	r1, [sp, #8]
	mov	r9, r2
	str	r3, [sp, #20]
	ldr	r6, [r5, #1784]
	ldrb	r7, [r5]	@ zero_extendqisi2
	ldrb	r6, [r6, #19]	@ zero_extendqisi2
	str	r6, [sp, #16]
	ldr	r6, .L2225+4
	ldrb	r6, [r6, #265]	@ zero_extendqisi2
	str	r6, [sp, #12]
	cbz	r7, .L2171
	bl	FlashProgSlc2KPages
	b	.L2172
.L2185:
	movs	r3, #36
	add	r2, sp, #28
	mov	r1, r9
	mul	r8, r3, r7
	ldr	r3, [sp, #8]
	subs	r3, r3, r7
	add	fp, r4, r8
	uxtb	r3, r3
	str	r3, [sp]
	mov	r0, fp
	add	r3, sp, #32
	bl	LogAddr2PhyAddr
	ldrb	r2, [r5, #1612]	@ zero_extendqisi2
	ldr	r3, [sp, #32]
	mov	r10, r0
	cmp	r3, r2
	bcc	.L2173
	mov	r3, #-1
	str	r3, [r4, r8]
.L2174:
	adds	r7, r7, #1
.L2171:
	ldr	r3, [sp, #8]
	ldr	r6, .L2225
	cmp	r7, r3
	bcc	.L2185
	b	.L2223
.L2173:
	add	r3, r5, r3, lsl #4
	ldrb	r1, [r5, #2828]	@ zero_extendqisi2
	ldr	r3, [r3, #1664]
	cmp	r1, #0
	it	eq
	moveq	r10, #0
	cbz	r3, .L2176
	cmp	r2, #1
	bne	.L2177
	ldr	r0, [r5, #1824]
	bl	NandcIqrWaitFlashReady
.L2177:
	ldrb	r0, [sp, #32]	@ zero_extendqisi2
	bl	FlashWaitCmdDone
.L2176:
	ldr	r3, [sp, #32]
	movs	r0, #0
	add	r2, r5, r3, lsl #4
	add	r1, r2, #1664
	str	r0, [r2, #1668]
	ldr	r0, [sp, #28]
	str	fp, [r2, #1664]
	str	r0, [r2, #1660]
	cmp	r10, #0
	beq	.L2178
	add	r2, r8, #36
	add	r2, r2, r4
	str	r2, [r1, #4]
.L2178:
	adds	r2, r5, r3
	add	r3, r5, r3, lsl #4
	ldrb	r6, [r2, #1616]	@ zero_extendqisi2
	strb	r6, [r3, #1656]
	mov	r0, r6
	ldrb	r3, [r5, #1612]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L2179
	bl	NandcWaitFlashReady
	b	.L2180
.L2179:
	bl	NandcFlashCs
	mov	r0, r6
	ldr	r3, [sp, #32]
	ldr	r1, [sp, #28]
	add	r3, r5, r3, lsl #2
	ldr	r2, [r3, #1624]
	adds	r2, r2, #0
	it	ne
	movne	r2, #1
	bl	FlashWaitReadyEN
	mov	r0, r6
	bl	NandcFlashDeCs
.L2180:
	ldr	r3, [sp, #16]
	subs	r3, r3, #1
	cmp	r3, #7
	bhi	.L2181
	adds	r3, r5, r6
	ldrb	r3, [r3, #2756]	@ zero_extendqisi2
	cbz	r3, .L2181
	mov	r0, r6
	ldrb	r1, [r5, #1903]	@ zero_extendqisi2
	ldr	r2, .L2225+8
	movs	r3, #0
	bl	HynixSetRRPara
.L2181:
	mov	r0, r6
	bl	NandcFlashCs
	cmp	r9, #1
	mov	r0, r6
	bne	.L2182
	ldrb	r3, [r5, #1888]	@ zero_extendqisi2
	cbz	r3, .L2182
	bl	flash_enter_slc_mode
	b	.L2183
.L2182:
	bl	flash_exit_slc_mode
.L2183:
	mov	r0, r6
	ldr	r1, [sp, #28]
	bl	FlashProgFirstCmd
	ldr	r3, [fp, #12]
	mov	r0, r6
	movs	r1, #1
	str	r3, [sp]
	ldr	r2, [sp, #12]
	ldr	r3, [fp, #8]
	bl	NandcXferData
	cmp	r10, #0
	beq	.L2184
	mov	r0, r6
	ldr	r1, [sp, #28]
	bl	FlashProgDpFirstCmd
	mov	r0, r6
	add	r8, r8, #36
	add	r8, r8, r4
	ldr	r3, [sp, #32]
	ldr	r1, [sp, #28]
	add	r3, r5, r3, lsl #2
	ldr	r2, [r3, #1624]
	adds	r2, r2, #0
	it	ne
	movne	r2, #1
	bl	FlashWaitReadyEN
	ldr	r1, [r5, #4]
	mov	r0, r6
	ldr	r3, [sp, #28]
	add	r1, r1, r3
	bl	FlashProgDpSecondCmd
	ldr	r3, [r8, #12]
	mov	r0, r6
	movs	r1, #1
	str	r3, [sp]
	ldr	r2, [sp, #12]
	ldr	r3, [r8, #8]
	bl	NandcXferData
.L2184:
	mov	r0, r6
	ldr	r1, [sp, #28]
	bl	FlashProgSecondCmd
	mov	r0, r6
	bl	NandcFlashDeCs
	add	r7, r7, r10
	b	.L2174
.L2223:
	ldr	r0, [r6, #1824]
	movs	r5, #0
	bl	NandcIqrWaitFlashReady
	add	r8, r6, #1656
	mov	r7, r6
.L2186:
	ldrb	r3, [r6, #1612]	@ zero_extendqisi2
	cmp	r5, r3
	bcs	.L2224
	uxtb	r0, r5
	bl	FlashWaitCmdDone
	cmp	r9, #1
	bne	.L2187
	ldrb	r3, [r7, #1888]	@ zero_extendqisi2
	cbz	r3, .L2187
	lsls	r3, r5, #4
	ldrb	r0, [r8, r3]	@ zero_extendqisi2
	bl	flash_exit_slc_mode
.L2187:
	adds	r5, r5, #1
	b	.L2186
.L2224:
	ldr	r3, [sp, #20]
	cbnz	r3, .L2189
.L2197:
	movs	r0, #0
	b	.L2172
.L2189:
	ldr	r7, .L2225+12
	mov	r8, #0
	mov	r10, r7
.L2190:
	ldr	r3, [sp, #8]
	cmp	r8, r3
	beq	.L2197
	ldr	r3, [r4]
	adds	r3, r3, #1
	bne	.L2191
	ldr	r1, [r4, #4]
	ldr	r0, .L2225+16
	bl	printk
	b	.L2192
.L2191:
	ldr	r3, [sp, #8]
	mov	r1, r9
	add	r2, sp, #28
	mov	r0, r4
	rsb	r3, r8, r3
	mov	r6, r4
	uxtb	r3, r3
	str	r3, [sp]
	add	r3, sp, #32
	bl	LogAddr2PhyAddr
	ldr	r2, [r7, #4028]
	movs	r3, #0
	str	r3, [r2]
	ldr	r2, [r7, #4032]
	str	r3, [r2]
	ldmia	r6!, {r0, r1, r2, r3}
	add	r5, sp, #36
	stmia	r5!, {r0, r1, r2, r3}
	ldmia	r6!, {r0, r1, r2, r3}
	stmia	r5!, {r0, r1, r2, r3}
	add	r0, sp, #36
	ldr	r3, [r6]
	movs	r1, #1
	mov	r2, r9
	str	r3, [r5]
	ldr	r3, [r7, #4028]
	str	r3, [sp, #44]
	ldr	r3, [r7, #4032]
	str	r3, [sp, #48]
	bl	FlashReadPages
	ldr	r5, [sp, #36]
	adds	r3, r5, #1
	bne	.L2193
	ldr	r0, .L2225+20
	ldr	r1, [r4, #4]
	bl	printk
	str	r5, [r4]
.L2193:
	ldr	r3, [r4, #12]
	cbz	r3, .L2194
	ldr	r2, [r3]
	ldr	r3, [r10, #4032]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L2194
	ldr	r0, .L2225+24
	ldr	r1, [r4, #4]
	bl	printk
	mov	r3, #-1
	str	r3, [r4]
.L2194:
	ldr	r3, [r4, #8]
	cbz	r3, .L2192
	ldr	r2, [r3]
	ldr	r3, [r10, #4028]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L2192
	ldr	r0, .L2225+28
	ldr	r1, [r4, #4]
	bl	printk
	mov	r3, #-1
	str	r3, [r4]
.L2192:
	add	r8, r8, #1
	adds	r4, r4, #36
	b	.L2190
.L2172:
	add	sp, sp, #76
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2226:
	.align	2
.L2225:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR0+1906
	.word	.LANCHOR2
	.word	.LC103
	.word	.LC104
	.word	.LC105
	.word	.LC106
	.fnend
	.size	FlashProgPages, .-FlashProgPages
	.align	1
	.global	FlashTestBlk
	.thumb
	.thumb_func
	.type	FlashTestBlk, %function
FlashTestBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 104
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2230
	push	{r4, r5, lr}
	.save {r4, r5, lr}
	mov	r4, r0
	ldr	r2, [r3, #4012]
	.pad #108
	sub	sp, sp, #108
	cmp	r0, r2
	bcc	.L2229
	ldr	r5, [r3, #4020]
	add	r0, sp, #40
	movs	r1, #165
	movs	r2, #32
	str	r0, [sp, #16]
	lsls	r4, r4, #10
	str	r5, [sp, #12]
	bl	memset
	mov	r0, r5
	movs	r1, #90
	movs	r2, #8
	bl	memset
	movs	r1, #1
	mov	r2, r1
	add	r0, sp, #4
	str	r4, [sp, #8]
	bl	FlashEraseBlocks
	movs	r1, #1
	mov	r2, r1
	mov	r3, r1
	add	r0, sp, #4
	bl	FlashProgPages
	movs	r1, #0
	movs	r2, #1
	ldr	r4, [sp, #4]
	add	r0, sp, #4
	adds	r4, r4, #0
	it	ne
	movne	r4, #1
	negs	r4, r4
	bl	FlashEraseBlocks
	b	.L2228
.L2229:
	movs	r4, #0
.L2228:
	mov	r0, r4
	add	sp, sp, #108
	@ sp needed
	pop	{r4, r5, pc}
.L2231:
	.align	2
.L2230:
	.word	.LANCHOR2
	.fnend
	.size	FlashTestBlk, .-FlashTestBlk
	.align	1
	.global	FlashMakeFactorBbt
	.thumb
	.thumb_func
	.type	FlashMakeFactorBbt, %function
FlashMakeFactorBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 80
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #84
	sub	sp, sp, #84
	ldr	r5, .L2290
	movs	r1, #1
	ldr	r4, .L2290+4
	ldr	r0, .L2290+8
	ldr	r3, [r5, #4024]
	ldrh	r8, [r4, #1872]
	str	r3, [sp, #20]
	ldrh	r3, [r4, #1874]
	smulbb	r8, r8, r3
	ldr	r3, [r4, #1784]
	ldrb	r3, [r3, #24]	@ zero_extendqisi2
	uxth	r8, r8
	str	r3, [sp, #24]
	ldrh	r3, [r4, #4]
	str	r3, [sp, #16]
	ldrb	r3, [r4]	@ zero_extendqisi2
	ldr	r4, .L2290+4
	cmp	r3, #1
	itttt	eq
	ldreq	r3, [sp, #16]
	lsleq	r3, r3, #1
	uxtheq	r3, r3
	streq	r3, [sp, #16]
	bl	printk
	ldr	r0, [r5, #4024]
	mov	r1, #4096
	ldr	r5, .L2290
	bl	__memzero
	lsr	r3, r8, #4
	str	r3, [sp, #28]
	movs	r3, #0
	str	r3, [sp, #8]
.L2234:
	ldrb	r7, [sp, #8]	@ zero_extendqisi2
	ldrb	r3, [r4, #1612]	@ zero_extendqisi2
	cmp	r3, r7
	bls	.L2286
	add	r3, r5, r7, lsl #1
	ldrh	r6, [r3, #4040]
	cmp	r6, #0
	bne	.L2264
	ldrh	r2, [r4, #1880]
	mov	r1, r6
	ldr	r0, [r5, #3992]
	mov	r9, r6
	lsls	r2, r2, #9
	bl	ftl_memset
	adds	r3, r4, r7
	ldrb	r10, [r3, #1616]	@ zero_extendqisi2
	lsls	r3, r7, #2
	add	fp, r4, r3
	str	r6, [sp, #4]
	str	r3, [sp, #32]
.L2236:
	ldrh	r3, [sp, #4]
	cmp	r3, r8
	str	r3, [sp, #12]
	bcs	.L2246
	movs	r3, #255
	strb	r3, [sp, #42]
	strb	r3, [sp, #43]
	ldr	r3, [sp, #24]
	lsls	r1, r3, #31
	bpl	.L2238
	ldr	r3, [fp, #1624]
	add	r2, sp, #42
	mov	r0, r10
	add	r3, r3, r6
	str	r3, [sp, #36]
	mov	r1, r3
	bl	FlashReadSpare
	ldrb	r2, [r4]	@ zero_extendqisi2
	cmp	r2, #1
	ldr	r3, [sp, #36]
	bne	.L2238
	ldr	r1, [r4, #4]
	add	r2, sp, #43
	mov	r0, r10
	add	r1, r1, r3
	bl	FlashReadSpare
	ldrb	r3, [sp, #42]	@ zero_extendqisi2
	ldrb	r2, [sp, #43]	@ zero_extendqisi2
	ands	r3, r3, r2
	strb	r3, [sp, #42]
.L2238:
	ldr	r3, [sp, #24]
	lsls	r2, r3, #30
	bpl	.L2240
	ldr	r3, [r4, #1784]
	mov	r0, r10
	add	r2, sp, #43
	ldrh	r1, [r3, #10]
	ldr	r3, [fp, #1624]
	subs	r1, r1, #1
	add	r1, r1, r3
	add	r1, r1, r6
	bl	FlashReadSpare
.L2240:
	ldr	r3, [r4, #1784]
	ldrb	r2, [r3, #7]	@ zero_extendqisi2
	ldrb	r3, [sp, #42]	@ zero_extendqisi2
	cmp	r2, #1
	beq	.L2241
	cmp	r2, #8
	bne	.L2242
.L2241:
	cbz	r3, .L2263
	ldrb	r0, [sp, #43]	@ zero_extendqisi2
	clz	r0, r0
	lsrs	r0, r0, #5
	b	.L2243
.L2242:
	cmp	r3, #255
	bne	.L2263
	ldrb	r0, [sp, #43]	@ zero_extendqisi2
	subs	r0, r0, #255
	it	ne
	movne	r0, #1
	b	.L2243
.L2263:
	movs	r0, #1
.L2243:
	ldr	r3, [sp, #24]
	lsls	r3, r3, #29
	bpl	.L2244
	ldr	r3, .L2290+4
	mov	r0, r10
	ldr	r2, [sp, #32]
	add	r3, r3, r2
	ldr	r1, [r3, #1624]
	add	r1, r1, r6
	bl	SandiskProgTestBadBlock
.L2244:
	cbz	r0, .L2245
	mov	r1, r7
	ldr	r2, [sp, #4]
	ldr	r0, .L2290+12
	add	r9, r9, #1
	bl	printk
	ldr	r2, [r5, #3992]
	uxth	r9, r9
	ldr	r3, [sp, #12]
	and	r0, r3, #31
	lsrs	r1, r3, #5
	movs	r3, #1
	lsls	r3, r3, r0
	ldr	r0, [r2, r1, lsl #2]
	orrs	r3, r3, r0
	str	r3, [r2, r1, lsl #2]
	ldr	r2, [sp, #28]
	ldrb	r3, [r4, #1612]	@ zero_extendqisi2
	muls	r3, r2, r3
	cmp	r9, r3
	bgt	.L2246
.L2245:
	ldr	r3, [sp, #4]
	adds	r3, r3, #1
	str	r3, [sp, #4]
	ldr	r3, [sp, #16]
	add	r6, r6, r3
	b	.L2236
.L2246:
	mov	r2, r9
	ldr	r0, .L2290+16
	mov	r1, r7
	bl	printk
	ldrb	r3, [r4, #1612]	@ zero_extendqisi2
	ldr	r2, [sp, #28]
	muls	r3, r2, r3
	cmp	r9, r3
	blt	.L2248
	ldrh	r2, [r4, #1880]
	movs	r1, #0
	ldr	r0, [r5, #3992]
	lsls	r2, r2, #9
	bl	ftl_memset
.L2248:
	cmp	r7, #0
	bne	.L2250
	ldrh	r10, [r5, #4012]
	mov	r9, #1
	mov	r6, r7
.L2251:
	ldr	r3, .L2290+4
	ldrb	r3, [r3, #1]	@ zero_extendqisi2
	cmp	r3, r10
	bls	.L2287
	mov	r0, r10
	bl	FlashTestBlk
	cbz	r0, .L2252
	mov	r1, r10
	ldr	r0, .L2290+20
	bl	printk
	ldr	r2, [r5, #3992]
	lsr	r0, r10, #5
	adds	r6, r6, #1
	and	r3, r10, #31
	ldr	r1, [r2, r0, lsl #2]
	uxth	r6, r6
	lsl	r3, r9, r3
	orrs	r3, r3, r1
	str	r3, [r2, r0, lsl #2]
.L2252:
	add	r10, r10, #1
	uxth	r10, r10
	b	.L2251
.L2287:
	add	fp, r8, #-1
	sub	r9, r8, #50
	mov	r10, #1
	uxth	fp, fp
.L2254:
	cmp	fp, r9
	ble	.L2288
	mov	r0, fp
	bl	FlashTestBlk
	cbz	r0, .L2255
	mov	r1, fp
	ldr	r0, .L2290+20
	bl	printk
	ldr	r2, [r5, #3992]
	lsr	r0, fp, #5
	and	r3, fp, #31
	lsl	r3, r10, r3
	ldr	r1, [r2, r0, lsl #2]
	orrs	r3, r3, r1
	str	r3, [r2, r0, lsl #2]
.L2255:
	add	fp, fp, #-1
	uxth	fp, fp
	b	.L2254
.L2288:
	ldr	r3, .L2290+4
	ldr	r2, [r5, #4012]
	ldrb	r3, [r3, #1]	@ zero_extendqisi2
	subs	r3, r3, r2
	cmp	r6, r3
	bcc	.L2250
	ldrh	r2, [r4, #1880]
	movs	r1, #0
	ldr	r0, [r5, #3992]
	lsls	r2, r2, #9
	bl	ftl_memset
.L2250:
	ldrb	r6, [sp, #8]	@ zero_extendqisi2
	add	r10, r8, #-1
	add	r9, r5, r7, lsl #1
	uxth	r10, r10
	mul	r6, r8, r6
.L2258:
	mov	r1, r7
	ldr	r0, .L2290+24
	mov	r2, r10
	bl	printk
	ldr	r1, [r5, #3992]
.L2259:
	lsr	r3, r10, #5
	and	r2, r10, #31
	ldr	r3, [r1, r3, lsl #2]
	lsrs	r3, r3, r2
	ands	r3, r3, #1
	beq	.L2289
	add	r10, r10, #-1
	uxth	r10, r10
	b	.L2259
.L2289:
	ldr	r1, [sp, #20]
	movw	r2, #61664
	strh	r10, [r9, #4040]	@ movhi
	add	r0, sp, #44
	strh	r2, [r1]	@ movhi
	strh	r10, [r1, #2]	@ movhi
	strh	r3, [r1, #8]	@ movhi
	movs	r1, #1
	ldr	r3, [r5, #3992]
	mov	r2, r1
	str	r3, [sp, #52]
	ldr	r3, [r5, #4024]
	str	r3, [sp, #56]
	add	r3, r10, r6
	lsls	r3, r3, #10
	str	r3, [sp, #48]
	bl	FlashEraseBlocks
	movs	r1, #1
	mov	r3, r1
	mov	r2, r1
	add	r0, sp, #44
	bl	FlashProgPages
	ldr	r3, [sp, #44]
	cbz	r3, .L2264
	add	r10, r10, #-1
	uxth	r10, r10
	b	.L2258
.L2264:
	ldr	r3, [sp, #8]
	adds	r3, r3, #1
	str	r3, [sp, #8]
	b	.L2234
.L2286:
	add	sp, sp, #84
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2291:
	.align	2
.L2290:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC117
	.word	.LC118
	.word	.LC119
	.word	.LC120
	.word	.LC121
	.fnend
	.size	FlashMakeFactorBbt, .-FlashMakeFactorBbt
	.align	1
	.global	FtlLowFormatEraseBlock
	.thumb
	.thumb_func
	.type	FtlLowFormatEraseBlock, %function
FtlLowFormatEraseBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r7, .L2344
	str	r0, [sp, #4]
	str	r1, [sp]
	ldrb	r3, [r7, #2828]	@ zero_extendqisi2
	str	r3, [sp, #16]
	ldr	r3, .L2344+4
	ldr	r2, [r3, #740]
	mov	fp, r3
	cmp	r2, #0
	bne	.L2321
	ldrb	r3, [r7, #1888]	@ zero_extendqisi2
	mov	r8, #36
	mov	r9, r2
	mov	r5, r2
	mov	r4, r2
	mov	r10, fp
	str	r3, [sp, #8]
	str	r0, [fp, #1756]
.L2294:
	ldrh	r0, [r7, #2840]
	uxth	r1, r9
	ldr	r3, .L2344
	cmp	r0, r1
	bls	.L2339
	mul	r6, r8, r1
	ldr	r0, [fp, #744]
	movs	r3, #0
	str	r3, [r0, r6]
	ldr	r3, .L2344
	add	r1, r1, r3
	ldrb	r0, [r1, #2868]	@ zero_extendqisi2
	ldr	r1, [sp, #4]
	bl	V2P_block
	ldr	r3, [sp]
	mov	r6, r0
	cbz	r3, .L2295
	bl	IsBlkInVendorPart
	cbnz	r0, .L2296
.L2295:
	mov	r0, r6
	bl	FtlBbmIsBadBlock
	cbnz	r0, .L2297
	ldr	r3, [fp, #744]
	lsls	r6, r6, #10
	ldrh	r1, [r7, #2920]
	muls	r1, r5, r1
	it	mi
	addmi	r1, r1, #3
	mla	lr, r8, r5, r3
	adds	r5, r5, #1
	bic	r1, r1, #3
	uxth	r5, r5
	str	r0, [lr, #8]
	ldr	r0, [r10, #1808]
	str	r6, [lr, #4]
	add	r1, r1, r0
	str	r1, [lr, #12]
	b	.L2296
.L2297:
	adds	r4, r4, #1
	uxth	r4, r4
.L2296:
	add	r9, r9, #1
	b	.L2294
.L2339:
	cmp	r5, #0
	beq	.L2320
	ldr	r2, [sp, #8]
	mov	r8, #0
	ldr	r0, [fp, #744]
	mov	r9, #36
	adds	r6, r2, #0
	mov	r2, r5
	it	ne
	movne	r6, #1
	strb	r8, [r3, #2828]
	str	r3, [sp, #8]
	mov	r1, r6
	bl	FlashEraseBlocks
	ldrb	r2, [sp, #16]	@ zero_extendqisi2
	ldr	r3, [sp, #8]
	strb	r2, [r3, #2828]
.L2301:
	uxth	r2, r8
	cmp	r2, r5
	bcs	.L2340
	mul	r2, r9, r8
	ldr	r1, [fp, #744]
	adds	r0, r1, r2
	ldr	r2, [r1, r2]
	adds	r2, r2, #1
	bne	.L2302
	ldr	r0, [r0, #4]
	adds	r4, r4, #1
	ubfx	r0, r0, #10, #16
	uxth	r4, r4
	bl	FtlBbmMapBadBlock
.L2302:
	add	r8, r8, #1
	b	.L2301
.L2340:
	ldr	r3, [sp]
	cbnz	r3, .L2304
	movs	r3, #6
	uxth	r6, r6
	str	r3, [sp, #12]
	movs	r3, #1
	str	r3, [sp, #8]
	b	.L2305
.L2304:
	ldrh	r3, [r7, #2910]
	str	r3, [sp, #8]
	ldrb	r3, [r7, #1888]	@ zero_extendqisi2
	cbnz	r3, .L2322
	ldr	r3, [sp, #8]
	movs	r6, #1
	lsrs	r3, r3, #2
	str	r3, [sp, #12]
	b	.L2305
.L2322:
	movs	r6, #1
	str	r6, [sp, #12]
.L2305:
	ldr	r8, .L2344+4
	movs	r7, #0
	mov	r10, r8
.L2315:
	mov	fp, #0
	mov	r5, fp
.L2306:
	ldr	r9, .L2344
	uxth	r3, fp
	ldrh	r1, [r9, #2840]
	cmp	r1, r3
	bls	.L2341
	movs	r2, #36
	ldr	r0, [r8, #744]
	mul	r1, r2, r3
	movs	r2, #0
	str	r2, [r0, r1]
	ldr	r2, .L2344
	ldr	r1, [sp, #4]
	add	r3, r3, r2
	ldrb	r0, [r3, #2868]	@ zero_extendqisi2
	bl	V2P_block
	ldr	r3, [sp]
	mov	r9, r0
	cbz	r3, .L2307
	bl	IsBlkInVendorPart
	cbnz	r0, .L2308
.L2307:
	mov	r0, r9
	bl	FtlBbmIsBadBlock
	cbnz	r0, .L2308
	ldr	r1, [r10, #744]
	movs	r3, #36
	ldr	r0, [r8, #1800]
	mla	r1, r3, r5, r1
	add	r3, r7, r9, lsl #10
	str	r3, [r1, #4]
	ldr	r3, [r10, #1796]
	str	r3, [r1, #8]
	ldr	r3, .L2344
	ldrh	r3, [r3, #2920]
	muls	r3, r5, r3
	add	r5, r5, #1
	it	mi
	addmi	r3, r3, #3
	bic	r3, r3, #3
	uxth	r5, r5
	add	r3, r3, r0
	str	r3, [r1, #12]
.L2308:
	add	fp, fp, #1
	b	.L2306
.L2341:
	cmp	r5, #0
	beq	.L2320
	mov	r2, r6
	movs	r3, #1
	ldr	r0, [r8, #744]
	mov	r1, r5
	mov	fp, #0
	strb	fp, [r9, #2828]
	bl	FlashProgPages
	movs	r2, #36
	ldrb	r3, [sp, #16]	@ zero_extendqisi2
	strb	r3, [r9, #2828]
	ldr	r9, .L2344+4
.L2312:
	uxth	r3, fp
	cmp	r3, r5
	bcs	.L2342
	mul	r3, r2, fp
	ldr	r1, [r10, #744]
	adds	r0, r1, r3
	ldr	r3, [r1, r3]
	cbz	r3, .L2313
	ldr	r0, [r0, #4]
	adds	r4, r4, #1
	str	r2, [sp, #20]
	ubfx	r0, r0, #10, #16
	uxth	r4, r4
	bl	FtlBbmMapBadBlock
	ldr	r2, [sp, #20]
.L2313:
	add	fp, fp, #1
	b	.L2312
.L2342:
	ldr	r3, [sp, #12]
	add	r7, r7, r3
	ldr	r3, [sp, #8]
	uxth	r7, r7
	cmp	r7, r3
	bcc	.L2315
	movs	r7, #0
	mov	r8, #36
.L2316:
	uxth	r3, r7
	cmp	r3, r5
	bcs	.L2343
	ldr	r3, [sp]
	cbz	r3, .L2317
	mul	r3, r8, r7
	ldr	r2, [r9, #744]
	adds	r1, r2, r3
	ldr	r3, [r2, r3]
	cbnz	r3, .L2317
	ldr	r0, [r1, #4]
	movs	r1, #1
	ubfx	r0, r0, #10, #16
	bl	FtlFreeSysBlkQueueIn
.L2317:
	adds	r7, r7, #1
	b	.L2316
.L2343:
	ldr	r3, [sp, #4]
	cmp	r3, #63
	bls	.L2319
	ldr	r3, [sp]
	cbz	r3, .L2320
.L2319:
	ldr	r0, [r9, #744]
	mov	r1, r6
	mov	r2, r5
	bl	FlashEraseBlocks
.L2320:
	mov	r0, r4
	b	.L2293
.L2321:
	movs	r0, #0
.L2293:
	add	sp, sp, #28
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2345:
	.align	2
.L2344:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlLowFormatEraseBlock, .-FtlLowFormatEraseBlock
	.align	1
	.global	FtlBbmTblFlush
	.thumb
	.thumb_func
	.type	FtlBbmTblFlush, %function
FtlBbmTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r4, .L2363
	ldr	r5, [r4, #740]
	cmp	r5, #0
	bne	.L2348
	ldr	r7, .L2363+4
	mov	r1, r5
	ldr	r0, [r4, #1780]
	ldr	r3, [r4, #1804]
	add	r8, r7, #2992
	ldrh	r2, [r7, #2918]
	str	r0, [r4, #4064]
	str	r3, [r4, #4068]
	bl	ftl_memset
.L2349:
	ldrh	r3, [r7, #2862]
	ldr	r6, .L2363+4
	cmp	r5, r3
	bge	.L2362
	ldrh	r2, [r4, #1868]
	ldr	r3, [r4, #4064]
	ldr	r1, [r8, #4]!
	mul	r0, r2, r5
	lsls	r2, r2, #2
	adds	r5, r5, #1
	add	r0, r3, r0, lsl #2
	bl	ftl_memcpy
	b	.L2349
.L2362:
	ldr	r8, [r4, #4068]
	movs	r1, #255
	movs	r2, #16
	mov	r9, #0
	ldr	fp, .L2363
	mov	r5, r6
	mov	r0, r8
	mov	r10, r9
	bl	memset
	movw	r3, #61649
	strh	r3, [r8]	@ movhi
	ldr	r3, [r6, #2976]
	str	r3, [r8, #4]
	ldrh	r3, [r6, #2968]
	strh	r3, [r8, #2]	@ movhi
	ldrh	r3, [r6, #2972]
	strh	r3, [r8, #8]	@ movhi
	ldrh	r3, [r6, #2974]
	strh	r3, [r8, #10]	@ movhi
	ldr	r3, [r6, #2836]
	strh	r3, [r8, #12]	@ movhi
.L2351:
	ldr	r3, [r4, #1780]
	movs	r6, #0
	ldrh	r1, [r7, #2968]
	ldrh	r2, [r7, #2970]
	str	r3, [r4, #4064]
	ldr	r3, [r4, #1804]
	str	r6, [r4, #4056]
	str	r3, [r4, #4068]
	orr	r3, r2, r1, lsl #10
	ldrh	r0, [r8, #10]
	str	r3, [r4, #4060]
	ldrh	r3, [r7, #2972]
	str	r0, [sp]
	ldr	r0, .L2363+8
	bl	printk
	ldrh	r3, [r7, #2910]
	ldrh	r2, [r7, #2970]
	subs	r3, r3, #1
	cmp	r2, r3
	blt	.L2352
	ldr	r3, [r5, #2976]
	movs	r1, #1
	ldrh	r2, [r5, #2968]
	adds	r3, r3, #1
	ldr	r0, [fp, #744]
	str	r3, [r5, #2976]
	str	r3, [r8, #4]
	ldrh	r3, [r5, #2972]
	strh	r2, [r8, #8]	@ movhi
	strh	r2, [r5, #2972]	@ movhi
	mov	r2, r1
	strh	r3, [r5, #2968]	@ movhi
	lsls	r3, r3, #10
	str	r3, [fp, #4060]
	str	r3, [r0, #4]
	strh	r6, [r5, #2970]	@ movhi
	bl	FlashEraseBlocks
.L2352:
	movs	r1, #1
	ldr	r6, .L2363
	mov	r2, r1
	mov	r3, r1
	addw	r0, r6, #4056
	bl	FlashProgPages
	ldrh	r3, [r7, #2970]
	adds	r3, r3, #1
	strh	r3, [r7, #2970]	@ movhi
	ldr	r3, [r4, #4056]
	adds	r2, r3, #1
	bne	.L2353
	add	r9, r9, #1
	ldr	r0, .L2363+12
	ldr	r1, [fp, #4060]
	uxth	r9, r9
	bl	printk
	cmp	r9, #3
	bls	.L2351
	ldr	r0, .L2363+16
	mov	r2, r9
	ldr	r1, [r6, #4060]
	bl	printk
	movs	r3, #1
	str	r3, [r6, #740]
	b	.L2348
.L2353:
	add	r10, r10, #1
	cmp	r10, #1
	beq	.L2351
	cmp	r3, #256
	beq	.L2351
.L2348:
	movs	r0, #0
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2364:
	.align	2
.L2363:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC122
	.word	.LC123
	.word	.LC124
	.fnend
	.size	FtlBbmTblFlush, .-FtlBbmTblFlush
	.align	1
	.global	allocate_data_superblock
	.thumb
	.thumb_func
	.type	allocate_data_superblock, %function
allocate_data_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r5, r0
	ldr	r4, .L2421
	.pad #20
	sub	sp, sp, #20
	ldr	r3, [r4, #740]
	cmp	r3, #0
	bne	.L2366
	ldr	r9, .L2421+16
	mov	r8, r4
.L2367:
	ldr	r3, .L2421+4
	ldrb	r2, [r5, #8]	@ zero_extendqisi2
	cmp	r5, r3
	bne	.L2368
	ldrh	r7, [r8, #828]
	ldr	r6, [r8, #1648]
	lsrs	r1, r7, #1
	mul	r0, r6, r7
	adds	r3, r1, #1
	add	r3, r3, r0, lsr #2
	ldr	r0, [r9, #2832]
	uxth	r3, r3
	cbz	r0, .L2391
	ldr	r0, [r8, #1048]
	cmp	r0, #39
	bhi	.L2391
	cmp	r0, #2
	bls	.L2396
	lsls	r0, r7, #31
	bpl	.L2369
	cbz	r6, .L2393
	b	.L2369
.L2368:
	cmp	r2, #1
	bne	.L2396
	ldrh	r3, [r9, #2860]
	cmp	r3, #1
	beq	.L2396
	ldrb	r3, [r9, #1888]	@ zero_extendqisi2
	cbnz	r3, .L2396
	ldrh	r0, [r8, #828]
	ldr	r3, [r9, #2832]
	lsrs	r1, r0, #3
	cbz	r3, .L2369
	ldr	r3, [r8, #1048]
	cmp	r3, #1
	bhi	.L2369
	rsb	r0, r0, r0, lsl #3
	ubfx	r1, r0, #3, #16
	b	.L2369
.L2391:
	mov	r1, r3
.L2369:
	cbz	r1, .L2370
	subs	r1, r1, #1
	uxth	r1, r1
	b	.L2370
.L2393:
	mov	r1, r6
	b	.L2370
.L2396:
	movs	r1, #0
.L2370:
	ldr	r0, .L2421+8
	bl	List_pop_index_node
	ldrh	r3, [r4, #828]
	subs	r3, r3, #1
	strh	r3, [r4, #828]	@ movhi
	ldrh	r3, [r9, #2848]
	uxth	r6, r0
	cmp	r3, r6
	bls	.L2367
	ldr	r3, [r8, #812]
	ldrh	r7, [r3, r6, lsl #1]
	cmp	r7, #0
	bne	.L2367
	strh	r6, [r5]	@ movhi
	mov	r0, r5
	bl	make_superblock
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cbz	r3, .L2372
	ldr	r0, [r8, #744]
	movs	r2, #36
	ldrh	ip, [r9, #2840]
	add	lr, r5, #14
	movw	r10, #65535
	mov	r3, r0
	mla	r1, r2, ip, r0
	mov	ip, r7
	str	r1, [sp, #4]
	b	.L2373
.L2372:
	ldr	r3, [r8, #812]
	b	.L2416
.L2373:
	ldr	r1, [sp, #4]
	cmp	r3, r1
	beq	.L2418
	str	ip, [r3, #8]
	adds	r3, r3, #36
	str	ip, [r3, #-24]
	ldrh	r1, [lr, #2]!
	cmp	r1, r10
	itttt	ne
	lslne	r1, r1, #10
	mlane	fp, r2, r7, r0
	addne	r7, r7, #1
	uxthne	r7, r7
	it	ne
	strne	r1, [fp, #4]
	b	.L2373
.L2418:
	ldr	r3, [r9, #2832]
	cbz	r3, .L2376
	ldr	r3, .L2421+12
	cmp	r5, r3
	bne	.L2376
	ldr	r3, [r8, #748]
	ldrh	r3, [r3, r6, lsl #1]
	cmp	r3, #40
	itt	hi
	movhi	r3, #0
	strbhi	r3, [r8, #840]
.L2376:
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	ldr	r2, [r4, #748]
	cbnz	r3, .L2377
	ldrh	r1, [r2, r6, lsl #1]
	cbz	r1, .L2397
	ldrh	r3, [r9, #2900]
	add	r3, r3, r1
	uxth	r3, r3
	b	.L2378
.L2397:
	movs	r3, #2
.L2378:
	strh	r3, [r2, r6, lsl #1]	@ movhi
	mov	r0, r6
	ldr	r3, [r4, #1032]
	movs	r1, #0
	adds	r3, r3, #1
	str	r3, [r4, #1032]
	bl	ftl_set_blk_mode
	b	.L2379
.L2377:
	ldrh	r3, [r2, r6, lsl #1]
	lsrs	r0, r6, #5
	and	r1, r6, #31
	adds	r3, r3, #1
	strh	r3, [r2, r6, lsl #1]	@ movhi
	ldr	r3, [r4, #1036]
	ldr	r2, [r4, #988]
	adds	r3, r3, #1
	str	r3, [r4, #1036]
	movs	r3, #1
	lsls	r3, r3, r1
	ldr	r1, [r2, r0, lsl #2]
	orrs	r3, r3, r1
	str	r3, [r2, r0, lsl #2]
.L2379:
	ldr	r3, [r4, #748]
	ldr	r2, [r4, #1044]
	ldr	r0, [r4, #1032]
	ldrh	r3, [r3, r6, lsl #1]
	ldrh	r1, [r9, #2848]
	cmp	r3, r2
	ldrh	r2, [r9, #2900]
	it	hi
	strhi	r3, [r8, #1044]
	ldr	r3, [r4, #1036]
	mla	r0, r0, r2, r3
	bl	__aeabi_uidiv
	ldr	r2, [r4, #1824]
	ldr	r1, [r4, #744]
	ldr	r3, [r2, #16]
	adds	r3, r3, #1
	str	r3, [r2, #16]
	movs	r3, #36
	adds	r2, r1, #4
	mla	r3, r3, r7, r1
	adds	r3, r3, #40
	str	r0, [r4, #1040]
.L2381:
	adds	r2, r2, #36
	cmp	r2, r3
	beq	.L2419
	ldr	r1, [r2, #-36]
	bic	r1, r1, #1020
	bic	r1, r1, #3
	str	r1, [r2, #-36]
	b	.L2381
.L2419:
	ldrb	r3, [r9, #1888]	@ zero_extendqisi2
	cbz	r3, .L2383
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	mov	r2, r7
	ldr	r0, [r4, #744]
	cmp	r3, #1
	ite	eq
	moveq	r1, #0
	movne	r1, #1
	bl	FlashEraseBlocks
.L2383:
	mov	r2, r7
	ldr	r0, [r4, #744]
	ldrb	r1, [r5, #8]	@ zero_extendqisi2
	mov	r10, #0
	bl	FlashEraseBlocks
	add	r2, r5, #16
	mov	ip, #36
	mov	fp, r10
.L2385:
	uxth	r3, r10
	cmp	r3, r7
	bcs	.L2420
	mul	r3, ip, r10
	ldr	r1, [r4, #744]
	adds	r0, r1, r3
	ldr	r3, [r1, r3]
	adds	r1, r3, #1
	bne	.L2386
	ldr	r0, [r0, #4]
	add	fp, fp, #1
	str	ip, [sp, #12]
	ubfx	r0, r0, #10, #16
	str	r3, [sp, #8]
	str	r2, [sp, #4]
	bl	FtlBbmMapBadBlock
	ldr	r3, [sp, #8]
	ldr	r2, [sp, #4]
	ldr	ip, [sp, #12]
	strh	r3, [r2]	@ movhi
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	subs	r3, r3, #1
	strb	r3, [r5, #7]
.L2386:
	add	r10, r10, #1
	adds	r2, r2, #2
	b	.L2385
.L2420:
	cmp	fp, #0
	beq	.L2388
	mov	r0, r6
	bl	update_multiplier_value
	bl	FtlBbmTblFlush
.L2388:
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cbnz	r3, .L2389
	ldr	r3, [r4, #812]
.L2416:
	movw	r2, #65535
	strh	r2, [r3, r6, lsl #1]	@ movhi
	b	.L2367
.L2389:
	ldr	r2, .L2421+16
	ldrh	r2, [r2, #2908]
	strh	r6, [r5]	@ movhi
	smulbb	r3, r2, r3
	movs	r2, #0
	strh	r2, [r5, #2]	@ movhi
	strb	r2, [r5, #6]
	ldr	r2, [r4, #1024]
	uxth	r3, r3
	strh	r3, [r5, #4]	@ movhi
	str	r2, [r5, #12]
	adds	r2, r2, #1
	str	r2, [r4, #1024]
	ldr	r2, [r4, #812]
	ldrh	r1, [r5]
	strh	r3, [r2, r1, lsl #1]	@ movhi
.L2366:
	movs	r0, #0
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2422:
	.align	2
.L2421:
	.word	.LANCHOR2
	.word	.LANCHOR2+928
	.word	.LANCHOR2+824
	.word	.LANCHOR2+832
	.word	.LANCHOR0
	.fnend
	.size	allocate_data_superblock, .-allocate_data_superblock
	.align	1
	.global	FtlGcFreeBadSuperBlk
	.thumb
	.thumb_func
	.type	FtlGcFreeBadSuperBlk, %function
FtlGcFreeBadSuperBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r5, .L2436
	str	r0, [sp]
	ldrh	r3, [r5, #1698]
	cmp	r3, #0
	beq	.L2425
	ldr	r7, .L2436+4
	mov	r10, #0
	mov	r8, r5
.L2424:
	ldrh	r2, [r7, #2840]
	uxth	r3, r10
	cmp	r2, r3
	bls	.L2433
	add	r3, r3, r7
	ldr	r1, [sp]
	mov	fp, #0
	ldrb	r0, [r3, #2868]	@ zero_extendqisi2
	bl	V2P_block
	ldr	r2, .L2436+8
	mov	r9, r0
.L2426:
	ldrh	r3, [r5, #1698]
	uxth	r4, fp
	cmp	r3, r4
	bls	.L2434
	lsls	r6, r4, #1
	add	r3, r8, r6
	ldrh	r3, [r3, #1700]
	cmp	r3, r9
	bne	.L2427
	mov	r1, r9
	ldr	r0, .L2436+12
	str	r2, [sp, #4]
	bl	printk
	mov	r0, r9
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldrh	r1, [r8, #1698]
	ldr	r2, [sp, #4]
	adds	r3, r2, r6
.L2428:
	cmp	r4, r1
	bcs	.L2435
	ldrh	r0, [r3, #2]
	adds	r4, r4, #1
	uxth	r4, r4
	strh	r0, [r3], #2	@ movhi
	b	.L2428
.L2435:
	subs	r1, r1, #1
	strh	r1, [r5, #1698]	@ movhi
.L2427:
	add	fp, fp, #1
	b	.L2426
.L2434:
	add	r10, r10, #1
	b	.L2424
.L2433:
	bl	FtlGcReFreshBadBlk
.L2425:
	movs	r0, #0
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2437:
	.align	2
.L2436:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2+1700
	.word	.LC125
	.fnend
	.size	FtlGcFreeBadSuperBlk, .-FtlGcFreeBadSuperBlk
	.align	1
	.global	update_vpc_list
	.thumb
	.thumb_func
	.type	update_vpc_list, %function
update_vpc_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L2447
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, r0
	ldr	r3, [r2, #812]
	mov	r5, r2
	ldrh	r3, [r3, r0, lsl #1]
	cbnz	r3, .L2439
	ldrh	r1, [r2, #1072]
	cmp	r1, r0
	bne	.L2440
	movw	r3, #65535
	strh	r3, [r2, #1072]	@ movhi
	b	.L2441
.L2440:
	ldrh	r1, [r2, #832]
	cmp	r1, r0
	beq	.L2446
	ldrh	r1, [r2, #880]
	cmp	r1, r0
	beq	.L2446
	ldrh	r2, [r2, #928]
	cmp	r2, r0
	beq	.L2446
.L2441:
	mov	r1, r4
	ldr	r0, .L2447+4
	bl	List_remove_node
	ldrh	r3, [r5, #820]
	mov	r0, r4
	subs	r3, r3, #1
	strh	r3, [r5, #820]	@ movhi
	bl	free_data_superblock
	mov	r0, r4
	bl	FtlGcFreeBadSuperBlk
	movs	r0, #1
	pop	{r3, r4, r5, pc}
.L2439:
	bl	List_update_data_list
.L2446:
	movs	r0, #0
	pop	{r3, r4, r5, pc}
.L2448:
	.align	2
.L2447:
	.word	.LANCHOR2
	.word	.LANCHOR2+808
	.fnend
	.size	update_vpc_list, .-update_vpc_list
	.align	1
	.global	decrement_vpc_count
	.thumb
	.thumb_func
	.type	decrement_vpc_count, %function
decrement_vpc_count:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	movw	r3, #65535
	cmp	r0, r3
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r0
	ldr	r5, .L2459
	beq	.L2450
	ldr	r3, [r5, #812]
	ldrh	r2, [r3, r0, lsl #1]
	cbnz	r2, .L2451
	mov	r1, r4
	ldr	r0, .L2459+4
	bl	printk
	ldr	r3, [r5, #812]
	movs	r2, #32
	add	r0, r5, #824
	mov	r1, r4
	strh	r2, [r3, r4, lsl #1]	@ movhi
	bl	test_node_in_list
	cbz	r0, .L2452
	mov	r1, r4
	add	r0, r5, #824
	bl	List_remove_node
	ldrh	r3, [r5, #828]
	mov	r0, r4
	subs	r3, r3, #1
	strh	r3, [r5, #828]	@ movhi
	bl	INSERT_DATA_LIST
	ldr	r3, [r5, #812]
	ldr	r0, .L2459+8
	mov	r1, r4
	ldrh	r2, [r3, r4, lsl #1]
	bl	printk
.L2452:
	mov	r0, r4
	bl	FtlGcRefreshBlock
	b	.L2455
.L2451:
	subs	r2, r2, #1
	strh	r2, [r3, r0, lsl #1]	@ movhi
.L2450:
	ldrh	r0, [r5, #1924]
	movw	r3, #65535
	ldr	r6, .L2459
	cmp	r0, r3
	bne	.L2454
	strh	r4, [r6, #1924]	@ movhi
	b	.L2455
.L2454:
	cmp	r0, r4
	beq	.L2455
	bl	update_vpc_list
	strh	r4, [r6, #1924]	@ movhi
	adds	r0, r0, #0
	it	ne
	movne	r0, #1
	pop	{r4, r5, r6, pc}
.L2455:
	movs	r0, #0
	pop	{r4, r5, r6, pc}
.L2460:
	.align	2
.L2459:
	.word	.LANCHOR2
	.word	.LC126
	.word	.LC127
	.fnend
	.size	decrement_vpc_count, .-decrement_vpc_count
	.align	1
	.global	FtlSlcSuperblockCheck
	.thumb
	.thumb_func
	.type	FtlSlcSuperblockCheck, %function
FtlSlcSuperblockCheck:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r4, r0
	ldrh	r3, [r0, #4]
	cmp	r3, #0
	beq	.L2461
	ldrh	r3, [r0]
	movw	r5, #65535
	cmp	r3, r5
	beq	.L2461
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	movs	r7, #0
	ldr	r6, .L2473
	adds	r3, r3, #8
	ldrh	r3, [r0, r3, lsl #1]
.L2465:
	cmp	r3, r5
	bne	.L2472
.L2467:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	ldrh	r2, [r6, #2840]
	adds	r3, r3, #1
	uxtb	r3, r3
	strb	r3, [r4, #6]
	cmp	r2, r3
	itttt	eq
	strbeq	r7, [r4, #6]
	ldrheq	r3, [r4, #2]
	addeq	r3, r3, #1
	strheq	r3, [r4, #2]	@ movhi
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	adds	r3, r3, #8
	ldrh	r3, [r4, r3, lsl #1]
	b	.L2465
.L2472:
	ldrb	r1, [r4, #8]	@ zero_extendqisi2
	cmp	r1, #1
	bne	.L2468
	ldrb	r3, [r6, #1888]	@ zero_extendqisi2
	cbnz	r3, .L2468
	ldrh	r3, [r4, #2]
	add	r3, r6, r3, lsl #1
	ldrh	r3, [r3, #588]
	cmp	r3, r5
	bne	.L2468
	ldrh	r3, [r4, #4]
	ldrh	r0, [r4]
	subs	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	ldrh	r2, [r4, #4]
	cmp	r2, #0
	bne	.L2467
	ldrh	r3, [r4, #2]
	strb	r2, [r4, #6]
	adds	r3, r3, #1
	strh	r3, [r4, #2]	@ movhi
	pop	{r3, r4, r5, r6, r7, pc}
.L2468:
	ldr	r2, .L2473
	ldrb	r3, [r2, #1888]	@ zero_extendqisi2
	cbz	r3, .L2461
	cmp	r1, #1
	bne	.L2461
	ldrh	r1, [r4, #2]
	ldrh	r3, [r2, #2910]
	cmp	r1, r3
	bcc	.L2461
	ldr	r3, .L2473+4
	ldrh	r0, [r4]
	ldrh	r5, [r4, #4]
	ldr	r1, [r3, #812]
	ldrh	r3, [r1, r0, lsl #1]
	subs	r3, r3, r5
	strh	r3, [r1, r0, lsl #1]	@ movhi
	ldrh	r2, [r2, #2908]
	movs	r3, #0
	strh	r3, [r4, #4]	@ movhi
	strb	r3, [r4, #6]
	strh	r2, [r4, #2]	@ movhi
.L2461:
	pop	{r3, r4, r5, r6, r7, pc}
.L2474:
	.align	2
.L2473:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlSlcSuperblockCheck, .-FtlSlcSuperblockCheck
	.align	1
	.global	get_new_active_ppa
	.thumb
	.thumb_func
	.type	get_new_active_ppa, %function
get_new_active_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	movs	r6, #0
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	mov	r4, r0
	strb	r6, [r0, #10]
	movw	r8, #65535
	adds	r3, r3, #8
	ldr	r5, .L2492
	ldrh	r2, [r0, r3, lsl #1]
.L2476:
	cmp	r2, r8
	ldr	r7, .L2492
	bne	.L2491
.L2477:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	ldrh	r2, [r5, #2840]
	adds	r3, r3, #1
	uxtb	r3, r3
	strb	r3, [r4, #6]
	cmp	r2, r3
	itttt	eq
	strbeq	r6, [r4, #6]
	ldrheq	r3, [r4, #2]
	addeq	r3, r3, #1
	strheq	r3, [r4, #2]	@ movhi
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	adds	r3, r3, #8
	ldrh	r2, [r4, r3, lsl #1]
	b	.L2476
.L2491:
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	ldrh	r1, [r4, #2]
	cmp	r3, #1
	ldrh	r3, [r4, #4]
	bne	.L2479
	ldrb	r0, [r5, #1888]	@ zero_extendqisi2
	cbnz	r0, .L2479
	add	r0, r5, r1, lsl #1
	ldrh	r0, [r0, #588]
	cmp	r0, r8
	bne	.L2479
	subs	r3, r3, #1
	ldrh	r0, [r4]
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	b	.L2477
.L2479:
	movw	r6, #65535
	orr	r5, r1, r2, lsl #10
	mov	r8, r6
	subs	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
.L2480:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	ldrh	r1, [r7, #2840]
.L2482:
	adds	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, r1
	itttt	eq
	ldrheq	r3, [r4, #2]
	addeq	r3, r3, #1
	strheq	r3, [r4, #2]	@ movhi
	moveq	r3, #0
	add	r2, r3, #8
	ldrh	r2, [r4, r2, lsl #1]
	cmp	r2, r6
	beq	.L2482
	strb	r3, [r4, #6]
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L2487
	ldrb	r3, [r7, #1888]	@ zero_extendqisi2
	cbnz	r3, .L2484
	ldrh	r2, [r4, #2]
	ldr	r3, .L2492
	add	r3, r3, r2, lsl #1
	ldrh	r3, [r3, #588]
	cmp	r3, r8
	bne	.L2484
	ldrh	r3, [r4, #4]
	cbz	r3, .L2484
	subs	r3, r3, #1
	ldrh	r0, [r4]
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	b	.L2480
.L2484:
	ldrb	r3, [r7, #1888]	@ zero_extendqisi2
	ldr	r2, .L2492
	cbz	r3, .L2487
	ldrh	r1, [r4, #2]
	ldrh	r3, [r2, #2910]
	cmp	r1, r3
	bcc	.L2487
	ldr	r3, .L2492+4
	ldrh	r0, [r4]
	ldrh	r6, [r4, #4]
	ldr	r1, [r3, #812]
	ldrh	r3, [r1, r0, lsl #1]
	subs	r3, r3, r6
	strh	r3, [r1, r0, lsl #1]	@ movhi
	ldrh	r2, [r2, #2908]
	movs	r3, #0
	strh	r3, [r4, #4]	@ movhi
	strb	r3, [r4, #6]
	strh	r2, [r4, #2]	@ movhi
.L2487:
	mov	r0, r5
	pop	{r4, r5, r6, r7, r8, pc}
.L2493:
	.align	2
.L2492:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	get_new_active_ppa, .-get_new_active_ppa
	.align	1
	.global	FtlVpcTblFlush
	.thumb
	.thumb_func
	.type	FtlVpcTblFlush, %function
FtlVpcTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	ldr	r4, .L2517
	ldr	r3, [r4, #740]
	cmp	r3, #0
	bne	.L2496
	ldr	r2, [r4, #1780]
	movs	r1, #255
	ldr	r6, [r4, #1804]
	ldr	r5, .L2517+4
	str	r2, [r4, #4064]
	ldrh	r2, [r4, #1056]
	str	r6, [r4, #4068]
	str	r3, [r6, #12]
	strh	r2, [r6, #2]	@ movhi
	movw	r2, #61604
	strh	r2, [r6]	@ movhi
	ldr	r2, [r4, #1064]
	str	r3, [r6, #8]
	ldr	r3, .L2517+8
	str	r2, [r6, #4]
	ldrh	r2, [r4, #834]
	str	r3, [r4, #756]
	ldr	r3, .L2517+12
	str	r3, [r4, #760]
	ldrh	r3, [r4, #1062]
	strh	r3, [r4, #764]	@ movhi
	ldrh	r3, [r5, #2862]
	strb	r3, [r4, #766]
	ldrh	r3, [r4, #832]
	strh	r3, [r4, #770]	@ movhi
	ldrb	r3, [r4, #838]	@ zero_extendqisi2
	orr	r3, r3, r2, lsl #6
	strh	r3, [r4, #772]	@ movhi
	ldrb	r3, [r4, #840]	@ zero_extendqisi2
	ldrh	r2, [r4, #882]
	strb	r3, [r4, #767]
	ldrh	r3, [r4, #880]
	strh	r3, [r4, #774]	@ movhi
	ldrb	r3, [r4, #886]	@ zero_extendqisi2
	orr	r3, r3, r2, lsl #6
	strh	r3, [r4, #776]	@ movhi
	ldrb	r3, [r4, #888]	@ zero_extendqisi2
	strb	r3, [r4, #768]
	ldrh	r3, [r4, #928]
	strh	r3, [r4, #778]	@ movhi
	ldrh	r2, [r4, #930]
	ldrb	r3, [r4, #934]	@ zero_extendqisi2
	ldr	r0, [r4, #4064]
	orr	r3, r3, r2, lsl #6
	strh	r3, [r4, #780]	@ movhi
	ldrb	r3, [r4, #936]	@ zero_extendqisi2
	ldrh	r2, [r5, #2918]
	strb	r3, [r4, #769]
	ldr	r3, [r4, #1032]
	str	r3, [r4, #788]
	ldr	r3, [r4, #1024]
	str	r3, [r4, #796]
	ldr	r3, [r4, #1028]
	str	r3, [r4, #792]
	ldrh	r3, [r4, #1690]
	strh	r3, [r4, #800]	@ movhi
	ldrh	r3, [r4, #1692]
	strh	r3, [r4, #802]	@ movhi
	bl	ftl_memset
	add	r1, r4, #756
	movs	r2, #48
	ldr	r0, [r4, #4064]
	bl	ftl_memcpy
	ldrh	r2, [r5, #2848]
	ldr	r0, [r4, #4064]
	ldr	r1, [r4, #812]
	lsls	r2, r2, #1
	adds	r0, r0, #48
	bl	ftl_memcpy
	ldrh	r2, [r5, #2848]
	ldr	r0, [r4, #4064]
	add	r3, r2, #24
	ldr	r1, [r4, #988]
	ubfx	r3, r3, #1, #30
	lsrs	r2, r2, #3
	adds	r2, r2, #4
	add	r0, r0, r3, lsl #2
	bl	ftl_memcpy
	ldrh	r3, [r5, #2948]
	cbz	r3, .L2497
	ldrh	r2, [r5, #2848]
	ldr	r0, [r4, #4064]
	ldr	r1, [r4, #1852]
	lsrs	r3, r2, #3
	add	r3, r3, r2, lsl #1
	ldrh	r2, [r5, #2944]
	adds	r3, r3, #52
	ubfx	r3, r3, #2, #14
	lsls	r2, r2, #2
	add	r0, r0, r3, lsl #2
	bl	ftl_memcpy
.L2497:
	movs	r0, #0
	ldr	r7, .L2517
	bl	FtlUpdateVaildLpn
	ldr	r10, .L2517+4
	mov	r8, #0
	movw	r9, #65535
.L2498:
	ldr	r3, [r4, #1780]
	ldrh	r2, [r4, #1056]
	ldrh	r1, [r4, #1058]
	str	r3, [r4, #4064]
	ldr	r3, [r4, #1804]
	str	r3, [r4, #4068]
	orr	r3, r1, r2, lsl #10
	str	r3, [r4, #4060]
	ldrh	r3, [r5, #2910]
	subs	r3, r3, #1
	cmp	r1, r3
	blt	.L2499
	movs	r3, #0
	ldrh	r9, [r7, #1060]
	strh	r3, [r7, #1058]	@ movhi
	strh	r2, [r7, #1060]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	ldr	r3, [r7, #1024]
	adds	r2, r3, #1
	str	r2, [r7, #1024]
	str	r3, [r7, #1064]
	lsls	r2, r0, #10
	strh	r0, [r7, #1056]	@ movhi
	str	r2, [r7, #4060]
	str	r3, [r6, #4]
	strh	r0, [r6, #2]	@ movhi
.L2499:
	ldrb	r3, [r5]	@ zero_extendqisi2
	cbz	r3, .L2500
	ldr	r0, [r4, #1780]
	ldrh	r1, [r10, #2918]
	bl	js_hash
	str	r0, [r6, #12]
.L2500:
	movs	r1, #1
	ldr	r0, .L2517+16
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #1058]
	ldr	r2, [r4, #4056]
	adds	r3, r3, #1
	adds	r1, r2, #1
	uxth	r3, r3
	strh	r3, [r4, #1058]	@ movhi
	bne	.L2501
	cmp	r3, #1
	add	r8, r8, #1
	it	eq
	ldrheq	r3, [r5, #2910]
	uxth	r8, r8
	itt	eq
	addeq	r3, r3, #-1
	strheq	r3, [r7, #1058]	@ movhi
	cmp	r8, #3
	bls	.L2498
	ldr	r0, .L2517+20
	mov	r2, r8
	ldr	r1, [r4, #4060]
	bl	printk
	movs	r3, #1
	str	r3, [r4, #740]
	b	.L2496
.L2501:
	cmp	r3, #1
	beq	.L2498
	cmp	r2, #256
	beq	.L2498
	movw	r3, #65535
	cmp	r9, r3
	beq	.L2496
	mov	r0, r9
	movs	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L2496:
	movs	r0, #0
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L2518:
	.align	2
.L2517:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	1179929683
	.word	1342177366
	.word	.LANCHOR2+4056
	.word	.LC128
	.fnend
	.size	FtlVpcTblFlush, .-FtlVpcTblFlush
	.align	1
	.global	FtlSuperblockPowerLostFix
	.thumb
	.thumb_func
	.type	FtlSuperblockPowerLostFix, %function
FtlSuperblockPowerLostFix:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r4, r0
	ldr	r5, .L2535
	.pad #44
	sub	sp, sp, #44
	ldr	r3, [r5, #740]
	cmp	r3, #0
	bne	.L2519
	ldr	r2, .L2535+4
	ldrb	r6, [r2, #1888]	@ zero_extendqisi2
	mov	r8, r2
	cbz	r6, .L2534
	ldrb	r6, [r0, #8]	@ zero_extendqisi2
	cmp	r6, #1
	bne	.L2530
	ldrh	fp, [r0, #4]
	b	.L2521
.L2530:
	mov	r6, r3
.L2534:
	mov	fp, #12
.L2521:
	ldr	r7, [r5, #1804]
	mov	r3, #-1
	str	r3, [sp, #20]
	mov	r9, #0
	ldr	r3, [r5, #1780]
	movw	r2, #61589
	str	r7, [sp, #16]
	ldr	r10, .L2535
	str	r3, [sp, #12]
	mvn	r3, #2
	str	r3, [r7, #8]
	mvn	r3, #1
	str	r3, [r7, #12]
	ldrh	r3, [r4]
	strh	r9, [r7]	@ movhi
	strh	r3, [r7, #2]	@ movhi
	ldr	r3, [r5, #1780]
	str	r2, [r3]
	add	r2, r2, #304087040
	ldr	r3, [r5, #1780]
	add	r2, r2, #1269760
	addw	r2, r2, #1507
	str	r2, [r3, #4]
.L2522:
	adds	fp, fp, #-1
	bcc	.L2525
	ldrh	r3, [r4, #4]
	cbnz	r3, .L2523
.L2525:
	ldr	r2, [r5, #812]
	ldrh	r1, [r4]
	ldrh	r0, [r4, #4]
	ldrh	r3, [r2, r1, lsl #1]
	subs	r3, r3, r0
	strh	r3, [r2, r1, lsl #1]	@ movhi
	ldrh	r3, [r8, #2908]
	strh	r3, [r4, #2]	@ movhi
	movs	r3, #0
	strb	r3, [r4, #6]
	strh	r3, [r4, #4]	@ movhi
	b	.L2519
.L2523:
	mov	r0, r4
	bl	get_new_active_ppa
	str	r0, [sp, #8]
	adds	r0, r0, #1
	beq	.L2525
	ldr	r3, [r5, #1028]
	movs	r1, #1
	add	r0, sp, #4
	str	r3, [r7, #4]
	adds	r3, r3, #1
	adds	r2, r3, #1
	mov	r2, r6
	it	eq
	moveq	r3, r9
	str	r3, [r10, #1028]
	movs	r3, #0
	bl	FlashProgPages
	ldrh	r0, [r4]
	bl	decrement_vpc_count
	b	.L2522
.L2519:
	add	sp, sp, #44
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2536:
	.align	2
.L2535:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlSuperblockPowerLostFix, .-FtlSuperblockPowerLostFix
	.align	1
	.global	ftl_map_blk_gc
	.thumb
	.thumb_func
	.type	ftl_map_blk_gc, %function
ftl_map_blk_gc:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r4, r0
	ldr	r5, [r0, #12]
	ldr	r8, [r0, #24]
	bl	ftl_free_no_use_map_blk
	ldrh	r3, [r4, #10]
	ldrh	r2, [r4, #8]
	subs	r3, r3, #4
	ldr	r9, .L2550+12
	cmp	r2, r3
	blt	.L2538
	uxth	r0, r0
	ldrh	r10, [r5, r0, lsl #1]
	cmp	r10, #0
	beq	.L2538
	ldr	r3, [r4, #32]
	cmp	r3, #0
	bne	.L2538
	movs	r2, #1
	str	r2, [r4, #32]
	strh	r3, [r5, r0, lsl #1]	@ movhi
	ldrh	r3, [r4, #8]
	ldrh	r2, [r4, #2]
	subs	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	ldrh	r3, [r9, #2910]
	cmp	r2, r3
	bcc	.L2539
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L2539:
	ldr	r5, .L2550
	mov	fp, #0
.L2540:
	ldrh	r3, [r4, #6]
	uxth	r6, fp
	cmp	r3, r6
	bls	.L2549
	ldr	r3, [r8, r6, lsl #2]
	add	ip, r8, r6, lsl #2
	cmp	r10, r3, lsr #10
	bne	.L2541
	ldr	r3, [r5, #1784]
	movs	r1, #1
	ldr	r7, [r5, #1804]
	ldr	r0, .L2550+4
	mov	r2, r1
	str	r3, [r5, #4064]
	str	r7, [r5, #4068]
	ldr	r3, [r8, r6, lsl #2]
	str	ip, [sp, #4]
	str	r3, [r5, #4060]
	bl	FlashReadPages
	ldr	r3, [r5, #4056]
	adds	r3, r3, #1
	ldr	r3, .L2550
	ldr	ip, [sp, #4]
	bne	.L2542
.L2544:
	movs	r2, #0
	ldr	r0, .L2550+8
	str	r2, [ip]
	ldr	r1, [r3, #4060]
	ldrh	r2, [r7, #8]
	str	r3, [sp, #4]
	bl	printk
	movs	r2, #1
	ldr	r3, [sp, #4]
	str	r2, [r3, #740]
	b	.L2543
.L2542:
	ldrh	r1, [r7, #8]
	cmp	r1, r6
	bne	.L2544
	ldrh	r0, [r7]
	ldrh	r2, [r4, #4]
	cmp	r0, r2
	bne	.L2544
	mov	r0, r4
	ldr	r2, [r5, #4064]
	bl	FtlMapWritePage
.L2541:
	add	fp, fp, #1
	b	.L2540
.L2549:
	mov	r0, r10
	movs	r1, #1
	bl	FtlFreeSysBlkQueueIn
	movs	r3, #0
	str	r3, [r4, #32]
.L2538:
	ldrh	r2, [r4, #2]
	ldrh	r3, [r9, #2910]
	cmp	r2, r3
	bcc	.L2543
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L2543:
	movs	r0, #0
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2551:
	.align	2
.L2550:
	.word	.LANCHOR2
	.word	.LANCHOR2+4056
	.word	.LC129
	.word	.LANCHOR0
	.fnend
	.size	ftl_map_blk_gc, .-ftl_map_blk_gc
	.align	1
	.global	Ftl_write_map_blk_to_last_page
	.thumb
	.thumb_func
	.type	Ftl_write_map_blk_to_last_page, %function
Ftl_write_map_blk_to_last_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	r4, r0
	ldr	r5, .L2563
	ldr	r7, [r0, #12]
	ldr	r9, [r0, #24]
	ldr	r6, [r5, #740]
	cmp	r6, #0
	bne	.L2553
	ldrh	r3, [r0]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L2554
	ldrh	r3, [r0, #8]
	adds	r3, r3, #1
	strh	r3, [r0, #8]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	strh	r0, [r7]	@ movhi
	ldr	r3, [r4, #28]
	strh	r6, [r4, #2]	@ movhi
	adds	r3, r3, #1
	strh	r6, [r4]	@ movhi
	str	r3, [r4, #28]
	b	.L2553
.L2554:
	ldrh	r8, [r7, r3, lsl #1]
	movs	r1, #255
	ldrh	r3, [r0, #2]
	ldr	r7, [r5, #1804]
	orr	r3, r3, r8, lsl #10
	str	r3, [r5, #4060]
	ldr	r3, [r5, #1780]
	str	r7, [r5, #4068]
	ldr	r10, .L2563+4
	str	r3, [r5, #4064]
	ldr	r3, [r0, #28]
	str	r3, [r7, #4]
	movw	r3, #64245
	strh	r3, [r7, #8]	@ movhi
	ldrh	r3, [r0, #4]
	strh	r8, [r7, #2]	@ movhi
	strh	r3, [r7]	@ movhi
	ldrh	r2, [r10, #2910]
	ldr	r0, [r5, #1780]
	lsls	r2, r2, #3
	bl	ftl_memset
	mov	r1, r6
.L2555:
	ldrh	r2, [r4, #6]
	uxth	r3, r1
	cmp	r2, r3
	bls	.L2562
	ldr	r2, [r9, r3, lsl #2]
	cmp	r8, r2, lsr #10
	bne	.L2556
	adds	r6, r6, #1
	ldr	r2, [r5, #1780]
	uxth	r6, r6
	str	r3, [r2, r6, lsl #3]
	ldr	r2, [r9, r3, lsl #2]
	ldr	r3, [r5, #1780]
	add	r3, r3, r6, lsl #3
	str	r2, [r3, #4]
.L2556:
	adds	r1, r1, #1
	b	.L2555
.L2562:
	ldrb	r3, [r10]	@ zero_extendqisi2
	cbz	r3, .L2558
	ldr	r3, .L2563+4
	ldr	r0, [r5, #4064]
	ldrh	r1, [r3, #2918]
	bl	js_hash
	str	r0, [r7, #12]
.L2558:
	movs	r1, #1
	movs	r3, #0
	ldr	r0, .L2563+8
	mov	r2, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	mov	r0, r4
	adds	r3, r3, #1
	strh	r3, [r4, #2]	@ movhi
	bl	ftl_map_blk_gc
.L2553:
	movs	r0, #0
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L2564:
	.align	2
.L2563:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2+4056
	.fnend
	.size	Ftl_write_map_blk_to_last_page, .-Ftl_write_map_blk_to_last_page
	.align	1
	.global	FtlMapWritePage
	.thumb
	.thumb_func
	.type	FtlMapWritePage, %function
FtlMapWritePage:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	movs	r5, #0
	ldr	r9, .L2589
	mov	r4, r0
	mov	r8, r1
	str	r2, [sp]
	mov	r10, r9
.L2566:
	ldr	r3, [r9, #1008]
	ldr	r6, .L2589
	adds	r3, r3, #1
	str	r3, [r9, #1008]
	ldr	r3, .L2589+4
	ldrh	r2, [r4, #2]
	ldr	fp, .L2589+4
	ldrh	r3, [r3, #2910]
	subs	r3, r3, #1
	cmp	r2, r3
	bge	.L2567
	ldrh	r2, [r4]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L2568
.L2567:
	mov	r0, r4
	bl	Ftl_write_map_blk_to_last_page
.L2568:
	ldr	r3, [r10, #740]
	cmp	r3, #0
	bne	.L2584
	ldrh	r2, [r4]
	movs	r1, #16
	ldr	r3, [r4, #12]
	ldr	r0, [r9, #1804]
	ldrh	r7, [r3, r2, lsl #1]
	ldrh	r3, [r4, #2]
	str	r0, [r9, #4068]
	orr	r3, r3, r7, lsl #10
	str	r3, [r9, #4060]
	ldr	r3, [sp]
	str	r3, [r9, #4064]
	bl	__memzero
	ldr	r2, [r4, #28]
	ldr	r3, [r9, #4068]
	str	r2, [r3, #4]
	strh	r8, [r3, #8]	@ movhi
	ldrh	r2, [r4, #4]
	strh	r7, [r3, #2]	@ movhi
	strh	r2, [r3]	@ movhi
	ldrb	r2, [fp]	@ zero_extendqisi2
	cbz	r2, .L2570
	str	r3, [sp, #4]
	ldr	r3, .L2589+4
	ldr	r0, [r9, #4064]
	ldrh	r1, [r3, #2918]
	bl	js_hash
	ldr	r3, [sp, #4]
	str	r0, [r3, #12]
.L2570:
	movs	r1, #1
	ldr	r0, .L2589+8
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	adds	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r4, #2]	@ movhi
	ldr	r2, [r10, #4056]
	adds	r1, r2, #1
	bne	.L2571
	ldr	r0, .L2589+12
	adds	r5, r5, #1
	ldr	r1, [r9, #4060]
	bl	printk
	ldrh	r3, [r4, #2]
	uxth	r5, r5
	cmp	r3, #2
	ittt	ls
	ldrhls	r3, [fp, #2910]
	addls	r3, r3, #-1
	strhls	r3, [r4, #2]	@ movhi
	cmp	r5, #3
	bls	.L2566
	ldr	r0, .L2589+16
	mov	r2, r5
	ldr	r1, [r6, #4060]
	bl	printk
	movs	r3, #1
	str	r3, [r6, #740]
	b	.L2584
.L2571:
	cbz	r2, .L2574
	cmp	r3, #1
	strh	r7, [r4, #40]	@ movhi
	bne	.L2575
.L2576:
	movs	r3, #0
	str	r3, [r4, #36]
	b	.L2566
.L2575:
	cmp	r2, #256
	b	.L2588
.L2574:
	cmp	r3, #1
.L2588:
	beq	.L2576
	ldr	r3, [r4, #36]
	cmp	r3, #0
	bne	.L2576
	ldr	r2, [r6, #4060]
	ldr	r3, [r4, #24]
	str	r2, [r3, r8, lsl #2]
.L2584:
	movs	r0, #0
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2590:
	.align	2
.L2589:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2+4056
	.word	.LC130
	.word	.LC131
	.fnend
	.size	FtlMapWritePage, .-FtlMapWritePage
	.align	1
	.global	flush_l2p_region
	.thumb
	.thumb_func
	.type	flush_l2p_region, %function
flush_l2p_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movs	r4, #12
	ldr	r5, .L2592
	muls	r4, r0, r4
	add	r0, r5, #1872
	ldr	r3, [r5, #976]
	adds	r2, r3, r4
	ldrh	r1, [r3, r4]
	ldr	r2, [r2, #8]
	bl	FtlMapWritePage
	ldr	r3, [r5, #976]
	movs	r0, #0
	add	r4, r4, r3
	ldr	r3, [r4, #4]
	bic	r3, r3, #-2147483648
	str	r3, [r4, #4]
	pop	{r3, r4, r5, pc}
.L2593:
	.align	2
.L2592:
	.word	.LANCHOR2
	.fnend
	.size	flush_l2p_region, .-flush_l2p_region
	.align	1
	.global	FtlMapTblRecovery
	.thumb
	.thumb_func
	.type	FtlMapTblRecovery, %function
FtlMapTblRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r3, [r0, #24]
	mov	r4, r0
	ldrh	r10, [r0, #6]
	movs	r1, #0
	ldr	r7, .L2636
	str	r3, [sp]
	ldr	r3, [r0, #12]
	lsl	r2, r10, #2
	ldr	r8, .L2636+8
	mov	r5, r7
	mov	r9, r7
	str	r3, [sp, #4]
	ldr	r3, [r0, #16]
	str	r3, [sp, #16]
	ldrh	r3, [r0, #8]
	ldr	r0, [sp]
	str	r3, [sp, #12]
	bl	ftl_memset
	ldr	r3, [r7, #1780]
	movs	r2, #1
	ldr	r6, [r7, #1804]
	str	r2, [r4, #36]
	str	r3, [r7, #4064]
	movw	r3, #65535
	str	r6, [r7, #4068]
	strh	r3, [r4]	@ movhi
	strh	r3, [r4, #2]	@ movhi
	movs	r3, #0
	str	r3, [r4, #32]
	str	r3, [r4, #28]
	str	r3, [sp, #8]
.L2595:
	ldrh	r2, [sp, #8]
	ldr	r3, [sp, #12]
	ldr	fp, .L2636+8
	sxth	r7, r2
	cmp	r7, r3
	bge	.L2613
	ldr	r3, [sp, #12]
	subs	r1, r3, #1
	cmp	r7, r1
	lsl	r1, r7, #1
	bne	.L2596
	ldr	r3, [sp, #4]
	mov	r8, #0
	str	r2, [sp, #12]
	add	r3, r3, r1
	str	r3, [sp, #8]
	ldr	r3, [sp, #4]
	movs	r1, #1
	ldrh	r0, [r3, r7, lsl #1]
	bl	FtlGetLastWrittenPage
	ldr	r2, [sp, #12]
	adds	r3, r0, #1
	strh	r3, [r4, #2]	@ movhi
	sxth	r0, r0
	ldr	r3, [sp, #16]
	add	r9, r0, #1
	strh	r2, [r4]	@ movhi
	ldr	r3, [r3, r7, lsl #2]
	ldr	r7, .L2636
	str	r3, [r4, #28]
.L2597:
	sxth	r2, r8
	cmp	r2, r9
	bge	.L2613
	ldr	r3, [sp, #8]
	ldr	r0, .L2636+4
	ldrh	r1, [r3]
	orr	r2, r2, r1, lsl #10
	movs	r1, #1
	str	r2, [r5, #4060]
	mov	r2, r1
	bl	FlashReadPages
	ldrb	r2, [fp]	@ zero_extendqisi2
	cbz	r2, .L2598
	ldr	r2, [r7, #4068]
	ldr	r2, [r2, #12]
	cbz	r2, .L2598
	ldr	r1, .L2636+8
	ldr	r0, [r7, #4064]
	str	r2, [sp, #4]
	ldrh	r1, [r1, #2918]
	bl	js_hash
	ldr	r2, [sp, #4]
	cmp	r2, r0
	itt	ne
	movne	r2, #-1
	strne	r2, [r7, #4056]
.L2598:
	ldr	r2, [r7, #4056]
	adds	r2, r2, #1
	beq	.L2599
	ldrh	r2, [r6, #8]
	cmp	r2, r10
	bcs	.L2599
	ldrh	r1, [r4, #4]
	ldrh	r0, [r6]
	cmp	r0, r1
	ittt	eq
	ldreq	r1, [r7, #4060]
	ldreq	r3, [sp]
	streq	r1, [r3, r2, lsl #2]
.L2599:
	add	r8, r8, #1
	b	.L2597
.L2613:
	mov	r0, r4
	bl	ftl_free_no_use_map_blk
	ldrh	r2, [r4, #2]
	ldrh	r3, [fp, #2910]
	cmp	r2, r3
	bne	.L2602
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
	b	.L2602
.L2596:
	ldr	r2, [r5, #1780]
	ldr	r3, [sp, #4]
	ldr	r0, .L2636+4
	str	r2, [r5, #4064]
	add	fp, r3, r1
	ldrh	r2, [r8, #2910]
	ldrh	r1, [r3, r7, lsl #1]
	subs	r2, r2, #1
	orr	r2, r2, r1, lsl #10
	movs	r1, #1
	str	r2, [r5, #4060]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r2, [r5, #4056]
	adds	r2, r2, #1
	beq	.L2615
	ldrh	r1, [r6]
	ldrh	r2, [r4, #4]
	cmp	r1, r2
	bne	.L2615
	ldrh	r1, [r6, #8]
	movw	r2, #64245
	cmp	r1, r2
	bne	.L2615
	movs	r0, #0
	mov	lr, #8
	mov	ip, #4
.L2604:
	uxth	r2, r0
	ldrh	r1, [r8, #2910]
	sxth	r7, r2
	subs	r1, r1, #1
	cmp	r7, r1
	bge	.L2607
	ldr	r1, [r5, #1780]
	adds	r0, r0, #1
	ldrh	r7, [r1, r7, lsl #3]
	cmp	r7, r10
	itttt	cc
	smlabbcc	r2, r2, lr, ip
	ldrcc	r3, [sp]
	ldrcc	r2, [r1, r2]
	strcc	r2, [r3, r7, lsl #2]
	b	.L2604
.L2615:
	movs	r7, #0
.L2635:
	ldrh	r1, [r8, #2910]
	sxth	r2, r7
	cmp	r2, r1
	bge	.L2607
	ldrh	r1, [fp]
	ldr	r0, .L2636+4
	orr	r2, r2, r1, lsl #10
	movs	r1, #1
	str	r2, [r9, #4060]
	mov	r2, r1
	bl	FlashReadPages
	ldrb	r2, [r8]	@ zero_extendqisi2
	cbz	r2, .L2608
	ldr	r2, [r9, #4068]
	ldr	r2, [r2, #12]
	cbz	r2, .L2608
	ldr	r0, [r9, #4064]
	ldrh	r1, [r8, #2918]
	str	r2, [sp, #20]
	bl	js_hash
	ldr	r2, [sp, #20]
	cmp	r2, r0
	itt	ne
	movne	r2, #-1
	strne	r2, [r9, #4056]
.L2608:
	ldr	r2, .L2636
	ldr	r2, [r2, #4056]
	adds	r2, r2, #1
	beq	.L2609
	ldrh	r2, [r6, #8]
	cmp	r2, r10
	bcs	.L2609
	ldrh	r1, [r4, #4]
	ldrh	r0, [r6]
	cmp	r0, r1
	ittt	eq
	ldreq	r1, [r9, #4060]
	ldreq	r3, [sp]
	streq	r1, [r3, r2, lsl #2]
.L2609:
	adds	r7, r7, #1
	b	.L2635
.L2607:
	ldr	r3, [sp, #8]
	adds	r3, r3, #1
	str	r3, [sp, #8]
	b	.L2595
.L2602:
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, r4
	bl	ftl_map_blk_gc
	movs	r0, #0
	add	sp, sp, #28
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2637:
	.align	2
.L2636:
	.word	.LANCHOR2
	.word	.LANCHOR2+4056
	.word	.LANCHOR0
	.fnend
	.size	FtlMapTblRecovery, .-FtlMapTblRecovery
	.align	1
	.global	FtlLoadVonderInfo
	.thumb
	.thumb_func
	.type	FtlLoadVonderInfo, %function
FtlLoadVonderInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	.save {r3, lr}
	ldr	r3, .L2639
	ldr	r0, .L2639+4
	ldrh	r2, [r3, #2928]
	subs	r0, r0, #248
	strh	r2, [r0, #10]	@ movhi
	movw	r2, #61574
	strh	r2, [r0, #4]	@ movhi
	ldrh	r2, [r3, #2950]
	strh	r2, [r0, #8]	@ movhi
	ldrh	r2, [r3, #2930]
	ldr	r3, [r3, #2952]
	strh	r2, [r0, #6]	@ movhi
	str	r3, [r0, #12]
	ldr	r3, .L2639+8
	ldr	r2, [r3, #1844]
	str	r2, [r0, #16]
	ldr	r2, [r3, #1840]
	ldr	r3, [r3, #1848]
	str	r2, [r0, #20]
	str	r3, [r0, #24]
	bl	FtlMapTblRecovery
	movs	r0, #0
	pop	{r3, pc}
.L2640:
	.align	2
.L2639:
	.word	.LANCHOR0
	.word	.LANCHOR4
	.word	.LANCHOR2
	.fnend
	.size	FtlLoadVonderInfo, .-FtlLoadVonderInfo
	.align	1
	.global	FtlLoadMapInfo
	.thumb
	.thumb_func
	.type	FtlLoadMapInfo, %function
FtlLoadMapInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	.save {r3, lr}
	bl	FtlL2PDataInit
	ldr	r0, .L2642
	bl	FtlMapTblRecovery
	movs	r0, #0
	pop	{r3, pc}
.L2643:
	.align	2
.L2642:
	.word	.LANCHOR2+1872
	.fnend
	.size	FtlLoadMapInfo, .-FtlLoadMapInfo
	.align	1
	.global	FtlVendorPartWrite
	.thumb
	.thumb_func
	.type	FtlVendorPartWrite, %function
FtlVendorPartWrite:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2657
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r5, r0
	mov	r4, r1
	add	r0, r0, r1
	ldrh	r1, [r3, #2904]
	.pad #60
	sub	sp, sp, #60
	mov	r10, r2
	ldrh	r2, [r3, #2916]
	cmp	r0, r1
	str	r3, [sp, #4]
	bhi	.L2652
	ldr	r7, .L2657+4
	lsr	r6, r5, r2
	mov	fp, #0
	lsls	r3, r6, #2
	str	r3, [sp]
.L2646:
	cmp	r4, #0
	beq	.L2656
	ldr	r3, .L2657+4
	mov	r0, r5
	ldr	r2, [sp]
	ldr	r3, [r3, #1848]
	ldr	ip, [r3, r2]
	ldr	r3, [sp, #4]
	str	ip, [sp, #12]
	ldrh	r2, [r3, #2914]
	mov	r1, r2
	str	r2, [sp, #8]
	bl	__aeabi_uidivmod
	ldr	r2, [sp, #8]
	mov	r8, r1
	ldr	ip, [sp, #12]
	subs	r3, r2, r1
	uxth	r9, r3
	cmp	r9, r4
	it	hi
	uxthhi	r9, r4
	cmp	ip, #0
	beq	.L2648
	cmp	r9, r2
	beq	.L2648
	ldr	r2, [r7, #1788]
	movs	r1, #1
	add	r0, sp, #20
	str	ip, [sp, #24]
	str	r2, [sp, #28]
	movs	r2, #0
	str	r2, [sp, #32]
	mov	r2, r1
	bl	FlashReadPages
	b	.L2649
.L2648:
	ldr	r3, [sp, #4]
	movs	r1, #0
	ldr	r0, [r7, #1788]
	ldrh	r2, [r3, #2918]
	bl	ftl_memset
.L2649:
	lsl	ip, r9, #9
	ldr	r0, [r7, #1788]
	uxth	r8, r8
	mov	r1, r10
	mov	r2, ip
	str	ip, [sp, #8]
	add	r0, r0, r8, lsl #9
	rsb	r4, r9, r4
	bl	ftl_memcpy
	mov	r1, r6
	ldr	r0, .L2657+8
	adds	r6, r6, #1
	ldr	r2, [r7, #1788]
	add	r5, r5, r9
	bl	FtlMapWritePage
	ldr	r3, [sp]
	adds	r0, r0, #1
	ldr	ip, [sp, #8]
	it	eq
	moveq	fp, #-1
	adds	r3, r3, #4
	str	r3, [sp]
	add	r10, r10, ip
	b	.L2646
.L2656:
	mov	r0, fp
	b	.L2645
.L2652:
	mov	r0, #-1
.L2645:
	add	sp, sp, #60
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2658:
	.align	2
.L2657:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR4-248
	.fnend
	.size	FtlVendorPartWrite, .-FtlVendorPartWrite
	.align	1
	.global	Ftl_save_ext_data
	.thumb
	.thumb_func
	.type	Ftl_save_ext_data, %function
Ftl_save_ext_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L2661
	ldr	r3, .L2661+4
	ldr	r1, [r2, #1124]
	cmp	r1, r3
	bne	.L2659
	ldr	r3, .L2661+8
	movs	r0, #0
	movs	r1, #1
	str	r3, [r2, #1128]
	ldr	r3, [r2, #1016]
	str	r3, [r2, #1212]
	ldr	r3, [r2, #1020]
	str	r3, [r2, #1216]
	ldr	r3, [r2, #1012]
	str	r3, [r2, #1132]
	ldr	r3, [r2, #1000]
	str	r3, [r2, #1136]
	ldr	r3, [r2, #992]
	str	r3, [r2, #1140]
	ldr	r3, [r2, #1008]
	str	r3, [r2, #1144]
	ldr	r3, [r2, #1036]
	str	r3, [r2, #1152]
	ldr	r3, [r2, #752]
	addw	r2, r2, #1124
	str	r3, [r2, #32]
	ldr	r3, [r2, #-128]
	str	r3, [r2, #36]
	ldr	r3, [r2, #-120]
	str	r3, [r2, #40]
	ldr	r3, [r2, #-80]
	str	r3, [r2, #44]
	ldr	r3, [r2, #-76]
	str	r3, [r2, #48]
	ldr	r3, [r2, #-4]
	str	r3, [r2, #60]
	ldr	r3, [r2, #616]
	str	r3, [r2, #64]
	b	FtlVendorPartWrite
.L2659:
	bx	lr
.L2662:
	.align	2
.L2661:
	.word	.LANCHOR2
	.word	1179929683
	.word	1342177366
	.fnend
	.size	Ftl_save_ext_data, .-Ftl_save_ext_data
	.align	1
	.global	FtlEctTblFlush
	.thumb
	.thumb_func
	.type	FtlEctTblFlush, %function
FtlEctTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2670
	push	{r4, lr}
	.save {r4, lr}
	ldr	r2, [r3, #2832]
	ldr	r3, .L2670+4
	cbz	r2, .L2668
	ldr	r2, [r3, #1048]
	cmp	r2, #39
	ite	hi
	movhi	r1, #32
	movls	r1, #4
	b	.L2664
.L2668:
	movs	r1, #32
.L2664:
	ldr	r4, .L2670+8
	ldrh	r2, [r4, #-204]
	cmp	r2, #31
	ittt	ls
	addls	r2, r2, #1
	movls	r1, #1
	strhls	r2, [r4, #-204]	@ movhi
	cbnz	r0, .L2666
	ldr	r0, [r3, #1824]
	ldr	r2, [r0, #20]
	add	r2, r2, r1
	ldr	r1, [r0, #16]
	cmp	r1, r2
	bcc	.L2667
.L2666:
	ldr	r2, [r3, #1824]
	movs	r0, #64
	ldr	r1, [r2, #16]
	str	r1, [r2, #20]
	ldr	r1, .L2670+12
	str	r1, [r2]
	ldr	r2, [r3, #1824]
	ldrh	r1, [r3, #1816]
	lsls	r3, r1, #9
	str	r3, [r2, #12]
	ldr	r3, [r2, #8]
	adds	r3, r3, #1
	str	r3, [r2, #8]
	movs	r3, #0
	str	r3, [r2, #4]
	bl	FtlVendorPartWrite
	bl	Ftl_save_ext_data
.L2667:
	movs	r0, #0
	pop	{r4, pc}
.L2671:
	.align	2
.L2670:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR4
	.word	1112818501
	.fnend
	.size	FtlEctTblFlush, .-FtlEctTblFlush
	.align	1
	.global	FtlVendorPartRead
	.thumb
	.thumb_func
	.type	FtlVendorPartRead, %function
FtlVendorPartRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2683
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r7, r0
	mov	r5, r1
	add	r0, r0, r1
	ldrh	r1, [r3, #2904]
	.pad #60
	sub	sp, sp, #60
	mov	r8, r2
	ldrh	r2, [r3, #2916]
	cmp	r0, r1
	str	r3, [sp, #8]
	bhi	.L2681
	lsr	r6, r7, r2
	ldr	r10, .L2683+12
	movs	r3, #0
	str	r3, [sp]
	lsl	r9, r6, #2
	mov	fp, r10
.L2674:
	cmp	r5, #0
	beq	.L2682
	ldr	r3, [r10, #1848]
	mov	r0, r7
	ldr	ip, [r3, r9]
	ldr	r3, [sp, #8]
	str	ip, [sp, #12]
	ldrh	r4, [r3, #2914]
	mov	r1, r4
	bl	__aeabi_uidivmod
	subs	r3, r4, r1
	ldr	ip, [sp, #12]
	str	r1, [sp, #4]
	uxth	r4, r3
	cmp	r4, r5
	it	hi
	uxthhi	r4, r5
	cmp	ip, #0
	beq	.L2676
	ldr	r2, [fp, #1788]
	movs	r1, #1
	add	r0, sp, #20
	str	ip, [sp, #24]
	str	ip, [sp, #12]
	str	r2, [sp, #28]
	movs	r2, #0
	str	r2, [sp, #32]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r2, [sp, #20]
	ldr	r3, [sp]
	adds	r2, r2, #1
	ldr	r2, [fp, #4056]
	it	eq
	moveq	r3, #-1
	ldr	ip, [sp, #12]
	cmp	r2, #256
	str	r3, [sp]
	bne	.L2678
	mov	r1, r6
	mov	r2, ip
	ldr	r0, .L2683+4
	bl	printk
	ldr	r0, .L2683+8
	mov	r1, r6
	ldr	r2, [r10, #1788]
	bl	FtlMapWritePage
.L2678:
	ldrh	r2, [sp, #4]
	mov	r0, r8
	ldr	r1, [fp, #1788]
	add	r1, r1, r2, lsl #9
	lsls	r2, r4, #9
	bl	ftl_memcpy
	b	.L2679
.L2676:
	mov	r0, r8
	mov	r1, ip
	lsls	r2, r4, #9
	bl	ftl_memset
.L2679:
	adds	r6, r6, #1
	subs	r5, r5, r4
	add	r7, r7, r4
	add	r8, r8, r4, lsl #9
	add	r9, r9, #4
	b	.L2674
.L2682:
	ldr	r0, [sp]
	b	.L2673
.L2681:
	mov	r0, #-1
.L2673:
	add	sp, sp, #60
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2684:
	.align	2
.L2683:
	.word	.LANCHOR0
	.word	.LC132
	.word	.LANCHOR4-248
	.word	.LANCHOR2
	.fnend
	.size	FtlVendorPartRead, .-FtlVendorPartRead
	.align	1
	.global	FtlLoadEctTbl
	.thumb
	.thumb_func
	.type	FtlLoadEctTbl, %function
FtlLoadEctTbl:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	movs	r0, #64
	ldr	r4, .L2687
	ldr	r2, [r4, #1824]
	ldrh	r1, [r4, #1816]
	bl	FtlVendorPartRead
	ldr	r3, [r4, #1824]
	ldr	r2, [r3]
	ldr	r3, .L2687+4
	cmp	r2, r3
	beq	.L2686
	ldr	r1, .L2687+8
	ldr	r0, .L2687+12
	bl	printk
	ldrh	r2, [r4, #1816]
	ldr	r0, [r4, #1824]
	movs	r1, #0
	lsls	r2, r2, #9
	bl	ftl_memset
.L2686:
	movs	r0, #0
	pop	{r4, pc}
.L2688:
	.align	2
.L2687:
	.word	.LANCHOR2
	.word	1112818501
	.word	.LC133
	.word	.LC76
	.fnend
	.size	FtlLoadEctTbl, .-FtlLoadEctTbl
	.align	1
	.global	Ftl_load_ext_data
	.thumb
	.thumb_func
	.type	Ftl_load_ext_data, %function
Ftl_load_ext_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movs	r0, #0
	ldr	r4, .L2693
	movs	r1, #1
	ldr	r5, .L2693+4
	addw	r2, r4, #1124
	bl	FtlVendorPartRead
	ldr	r3, [r4, #1124]
	cmp	r3, r5
	beq	.L2690
	addw	r0, r4, #1124
	mov	r1, #512
	bl	__memzero
	str	r5, [r4, #1124]
.L2690:
	ldr	r2, [r4, #1124]
	ldr	r3, .L2693
	cmp	r2, r5
	bne	.L2691
	ldr	r2, [r3, #1212]
	str	r2, [r3, #1016]
	ldr	r2, [r3, #1216]
	str	r2, [r3, #1020]
	ldr	r2, [r3, #1132]
	str	r2, [r3, #1012]
	ldr	r2, [r3, #1136]
	str	r2, [r3, #1000]
	ldr	r2, [r3, #1140]
	str	r2, [r3, #992]
	ldr	r2, [r3, #1144]
	str	r2, [r3, #1008]
	ldr	r2, [r3, #1152]
	str	r2, [r3, #1036]
	ldr	r2, [r3, #1156]
	str	r2, [r3, #752]
	ldr	r2, [r3, #1160]
	str	r2, [r3, #996]
	ldr	r2, [r3, #1164]
	str	r2, [r3, #1004]
	ldr	r2, [r3, #1168]
	str	r2, [r3, #1044]
	ldr	r2, [r3, #1172]
	str	r2, [r3, #1048]
	ldr	r2, [r3, #1184]
	str	r2, [r3, #1120]
.L2691:
	movs	r3, #0
	ldr	r2, [r4, #1192]
	str	r3, [r4, #1740]
	ldr	r3, .L2693+8
	ldr	r5, .L2693+12
	cmp	r2, r3
	bne	.L2692
	ldr	r0, .L2693+16
	movs	r3, #1
	ldr	r1, .L2693+20
	str	r3, [r5, #2832]
	bl	printk
.L2692:
	ldrh	r2, [r5, #2900]
	ldr	r3, [r4, #1036]
	ldr	r0, [r4, #1032]
	ldrh	r1, [r5, #2848]
	mla	r0, r0, r2, r3
	bl	__aeabi_uidiv
	str	r0, [r4, #1040]
	pop	{r3, r4, r5, pc}
.L2694:
	.align	2
.L2693:
	.word	.LANCHOR2
	.word	1179929683
	.word	305432421
	.word	.LANCHOR0
	.word	.LC76
	.word	.LC134
	.fnend
	.size	Ftl_load_ext_data, .-Ftl_load_ext_data
	.align	1
	.global	FtlMapBlkWriteDumpData
	.thumb
	.thumb_func
	.type	FtlMapBlkWriteDumpData, %function
FtlMapBlkWriteDumpData:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0
	ldr	r2, [r0, #36]
	ldrh	r5, [r0, #6]
	ldr	r3, [r0, #24]
	cbz	r2, .L2695
	ldr	r4, .L2701
	movs	r2, #0
	str	r2, [r0, #36]
	ldr	r2, [r4, #740]
	cbnz	r2, .L2695
	subs	r5, r5, #1
	ldr	r0, [r4, #1784]
	ldr	r2, [r4, #1804]
	uxth	r5, r5
	str	r0, [r4, #4064]
	str	r2, [r4, #4068]
	ldr	r3, [r3, r5, lsl #2]
	str	r3, [r4, #4060]
	cbz	r3, .L2699
	movs	r1, #1
	addw	r0, r4, #4056
	mov	r2, r1
	bl	FlashReadPages
	b	.L2700
.L2699:
	ldr	r3, .L2701+4
	movs	r1, #255
	ldrh	r2, [r3, #2918]
	bl	ftl_memset
.L2700:
	mov	r0, r6
	mov	r1, r5
	ldr	r2, [r4, #4064]
	pop	{r4, r5, r6, lr}
	b	FtlMapWritePage
.L2695:
	pop	{r4, r5, r6, pc}
.L2702:
	.align	2
.L2701:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlMapBlkWriteDumpData, .-FtlMapBlkWriteDumpData
	.align	1
	.global	FlashReadFacBbtData
	.thumb
	.thumb_func
	.type	FlashReadFacBbtData, %function
FlashReadFacBbtData:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2717
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	r8, r2
	ldrh	r2, [r3, #1872]
	mov	r7, r1
	ldrh	r3, [r3, #1874]
	.pad #40
	sub	sp, sp, #40
	mov	r5, r0
	smulbb	r3, r2, r3
	ldr	r2, .L2717+4
	ldr	r1, [r2, #3992]
	mov	r9, r2
	uxth	r3, r3
	subs	r6, r3, #1
	str	r1, [sp, #12]
	mul	r10, r3, r7
	ldr	r1, [r2, #4024]
	uxth	r6, r6
	sub	r4, r3, #16
	str	r1, [sp, #16]
.L2704:
	cmp	r6, r4
	ble	.L2716
	movs	r1, #1
	add	r3, r6, r10
	add	r0, sp, #4
	mov	r2, r1
	lsls	r3, r3, #10
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #4]
	adds	r3, r3, #1
	beq	.L2705
	ldr	r3, [r9, #4024]
	ldrh	r2, [r3]
	movw	r3, #61664
	cmp	r2, r3
	bne	.L2705
	cbz	r5, .L2711
	cbz	r7, .L2712
.L2709:
	ldr	r1, [r9, #3992]
	mov	r2, r8
	mov	r0, r5
	bl	ftl_memcpy
	movs	r2, #4
	ldr	r0, .L2717+8
	mov	r1, r5
	mov	r3, r2
	bl	rknand_print_hex
	movs	r0, #0
	b	.L2706
.L2712:
	ldr	r6, .L2717+4
	mov	lr, #1
.L2707:
	ldr	r2, [r9, #4012]
	uxth	r3, r7
	adds	r7, r7, #1
	cmp	r3, r2
	bcs	.L2709
	ldr	r2, [r6, #3992]
	lsrs	r1, r3, #5
	and	r4, r3, #31
	lsl	r3, lr, r4
	ldr	r0, [r2, r1, lsl #2]
	orrs	r3, r3, r0
	str	r3, [r2, r1, lsl #2]
	b	.L2707
.L2705:
	subs	r6, r6, #1
	uxth	r6, r6
	b	.L2704
.L2716:
	mov	r0, #-1
	b	.L2706
.L2711:
	mov	r0, r5
.L2706:
	add	sp, sp, #40
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L2718:
	.align	2
.L2717:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC135
	.fnend
	.size	FlashReadFacBbtData, .-FlashReadFacBbtData
	.align	1
	.global	FlashGetBadBlockList
	.thumb
	.thumb_func
	.type	FlashGetBadBlockList, %function
FlashGetBadBlockList:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2729
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r5, r0
	ldr	r3, [r3, #1784]
	ldr	r6, .L2729+4
	ldrb	r4, [r3, #13]	@ zero_extendqisi2
	ldrh	r3, [r3, #14]
	ldr	r0, [r6, #4020]
	smulbb	r4, r4, r3
	uxth	r4, r4
	adds	r2, r4, #7
	asrs	r2, r2, #3
	bl	FlashReadFacBbtData
	adds	r0, r0, #1
	bne	.L2720
.L2724:
	movs	r3, #0
	b	.L2721
.L2720:
	movs	r1, #0
	lsr	lr, r4, #4
	mov	ip, #1
	subs	r4, r4, #1
	mov	r3, r1
.L2722:
	uxth	r2, r1
	cmp	r2, r4
	bge	.L2721
	ldr	r7, [r6, #4020]
	lsrs	r0, r2, #5
	and	r8, r2, #31
	adds	r1, r1, #1
	lsl	r8, ip, r8
	ldr	r0, [r7, r0, lsl #2]
	tst	r8, r0
	ittt	ne
	strhne	r2, [r5, r3, lsl #1]	@ movhi
	addne	r0, r3, #1
	uxthne	r3, r0
	cmp	r3, lr
	bcc	.L2722
	b	.L2724
.L2721:
	movw	r2, #65535
	movs	r0, #0
	strh	r2, [r5, r3, lsl #1]	@ movhi
	pop	{r4, r5, r6, r7, r8, pc}
.L2730:
	.align	2
.L2729:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FlashGetBadBlockList, .-FlashGetBadBlockList
	.align	1
	.global	FtlMakeBbt
	.thumb
	.thumb_func
	.type	FtlMakeBbt, %function
FtlMakeBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r4, .L2753
	ldr	r5, [r4, #740]
	cmp	r5, #0
	bne	.L2732
	bl	FtlBbtMemInit
	ldr	r7, .L2753+4
	bl	FtlLoadFactoryBbt
	mov	fp, r4
	sub	r9, r7, #18
	subw	r8, r7, #2996
.L2733:
	ldrh	r3, [r8, #2862]
	ldr	r6, .L2753+8
	cmp	r5, r3
	bcs	.L2751
	ldrh	r3, [r9, #2]!
	movw	r2, #65535
	ldr	r0, [r4, #1780]
	ldr	r10, [r4, #1804]
	cmp	r3, r2
	str	r0, [r4, #4064]
	str	r10, [r4, #4068]
	beq	.L2734
	ldrh	ip, [r8, #2906]
	movs	r1, #1
	ldr	r0, .L2753+12
	mov	r2, r1
	mla	ip, ip, r5, r3
	lsl	r3, ip, #10
	str	ip, [sp, #4]
	str	r3, [fp, #4060]
	bl	FlashReadPages
	ldrh	r2, [r8, #2906]
	ldr	r0, [r7]
	adds	r2, r2, #7
	ldr	r1, [fp, #4064]
	asrs	r2, r2, #3
	bl	ftl_memcpy
	ldr	ip, [sp, #4]
	b	.L2735
.L2734:
	mov	r1, r5
	bl	FlashGetBadBlockList
	ldr	r0, [fp, #4064]
	ldr	r1, [r7]
	bl	FtlBbt2Bitmap
	ldrh	r6, [r8, #2906]
.L2737:
	subs	r6, r6, #1
	uxth	r6, r6
.L2736:
	ldr	r3, .L2753+8
	ldrh	r0, [r3, #2906]
	smlabb	r0, r0, r5, r6
	uxth	r0, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L2737
	ldr	r0, [r4, #1804]
	movs	r1, #16
	strh	r6, [r9]	@ movhi
	bl	__memzero
	movw	r3, #61664
	strh	r3, [r10]	@ movhi
	movs	r3, #0
	str	r3, [r10, #4]
	ldrh	r3, [r9]
	ldrh	ip, [r8, #2906]
	ldrh	r2, [r4, #1868]
	strh	r3, [r10, #2]	@ movhi
	ldrh	r3, [r9]
	ldr	r1, [r7]
	lsls	r2, r2, #2
	ldr	r0, [r4, #4064]
	mla	ip, ip, r5, r3
	lsl	r3, ip, #10
	str	r3, [r4, #4060]
	str	ip, [sp, #4]
	bl	ftl_memcpy
	movs	r1, #1
	ldr	r0, .L2753+12
	mov	r2, r1
	bl	FlashEraseBlocks
	movs	r1, #1
	ldr	r0, .L2753+12
	mov	r3, r1
	mov	r2, r1
	bl	FlashProgPages
	ldr	r3, [r4, #4056]
	adds	r3, r3, #1
	ldr	ip, [sp, #4]
	bne	.L2735
	uxth	r0, ip
	bl	FtlBbmMapBadBlock
	b	.L2736
.L2735:
	uxth	r0, ip
	adds	r5, r5, #1
	bl	FtlBbmMapBadBlock
	adds	r7, r7, #4
	b	.L2733
.L2751:
	movs	r5, #0
.L2740:
	ldrh	r3, [r6, #2922]
	uxth	r0, r5
	ldr	r7, .L2753+8
	adds	r5, r5, #1
	cmp	r3, r0
	bls	.L2752
	bl	FtlBbmMapBadBlock
	b	.L2740
.L2752:
	ldrh	r5, [r7, #2980]
	movw	r8, #65535
	subs	r5, r5, #1
	uxth	r5, r5
.L2742:
	ldrh	r3, [r6, #2980]
	subs	r3, r3, #48
	cmp	r5, r3
	ble	.L2746
	mov	r0, r5
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L2743
	mov	r0, r5
	bl	FlashTestBlk
	cbz	r0, .L2744
	mov	r0, r5
	bl	FtlBbmMapBadBlock
	b	.L2743
.L2744:
	ldrh	r3, [r7, #2968]
	cmp	r3, r8
	bne	.L2745
	strh	r5, [r7, #2968]	@ movhi
	b	.L2743
.L2745:
	ldr	r3, .L2753+8
	strh	r5, [r3, #2972]	@ movhi
	b	.L2746
.L2743:
	subs	r5, r5, #1
	uxth	r5, r5
	b	.L2742
.L2746:
	ldr	r0, [r4, #744]
	movs	r5, #0
	ldrh	r3, [r6, #2968]
	movs	r2, #2
	str	r5, [r6, #2976]
	movs	r1, #1
	strh	r5, [r6, #2970]	@ movhi
	lsls	r3, r3, #10
	str	r3, [r0, #4]
	ldrh	r3, [r6, #2972]
	lsls	r3, r3, #10
	str	r3, [r0, #40]
	bl	FlashEraseBlocks
	ldrh	r0, [r6, #2968]
	bl	FtlBbmMapBadBlock
	ldrh	r0, [r6, #2972]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldr	r3, [r6, #2976]
	ldrh	r2, [r6, #2972]
	adds	r3, r3, #1
	str	r3, [r6, #2976]
	ldrh	r3, [r6, #2968]
	strh	r5, [r6, #2970]	@ movhi
	strh	r2, [r6, #2968]	@ movhi
	strh	r3, [r6, #2972]	@ movhi
	bl	FtlBbmTblFlush
.L2732:
	movs	r0, #0
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2754:
	.align	2
.L2753:
	.word	.LANCHOR2
	.word	.LANCHOR0+2996
	.word	.LANCHOR0
	.word	.LANCHOR2+4056
	.fnend
	.size	FtlMakeBbt, .-FtlMakeBbt
	.align	1
	.global	log2phys
	.thumb
	.thumb_func
	.type	log2phys, %function
log2phys:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r9, r0
	ldr	r6, .L2771
	mov	r10, r1
	str	r2, [sp, #4]
	ldr	r3, [r6, #2964]
	ldrh	r8, [r6, #2916]
	cmp	r0, r3
	bcs	.L2756
	add	r3, r8, #7
	ldr	r2, .L2771+4
	lsr	r5, r0, r3
	str	r3, [sp]
	ldrh	r1, [r6, #2946]
	movs	r4, #0
	ldr	fp, [r2, #976]
	mov	r8, r2
	uxth	r5, r5
	movs	r2, #12
	b	.L2757
.L2756:
	ldr	r3, [sp, #4]
	mov	r0, #-1
	cmp	r3, #0
	bne	.L2758
	str	r0, [r1]
	b	.L2758
.L2762:
	adds	r4, r4, #1
	mla	r0, r2, r4, fp
	ldrh	r0, [r0, #-12]
	cmp	r0, r5
	beq	.L2759
.L2757:
	uxth	r7, r4
	cmp	r7, r1
	bcc	.L2762
	bl	select_l2p_ram_region
	movs	r4, #12
	movw	r3, #65535
	muls	r4, r0, r4
	mov	r7, r0
	add	r2, fp, r4
	ldrh	r1, [fp, r4]
	cmp	r1, r3
	bne	.L2770
.L2763:
	ldr	r3, [r8, #1852]
	ldr	fp, .L2771+4
	ldr	r8, [r3, r5, lsl #2]
	cmp	r8, #0
	bne	.L2764
	ldr	r2, [fp, #976]
	movs	r1, #255
	add	r2, r2, r4
	ldr	r0, [r2, #8]
	ldrh	r2, [r6, #2918]
	bl	ftl_memset
	ldr	r2, [fp, #976]
	strh	r5, [r2, r4]	@ movhi
	ldr	r2, [fp, #976]
	add	r4, r4, r2
	str	r8, [r4, #4]
.L2759:
	ldr	r3, [sp]
	movs	r2, #1
	ldr	r1, [sp, #4]
	lsls	r2, r2, r3
	movs	r3, #12
	subs	r2, r2, #1
	and	r9, r9, r2
	ldr	r2, .L2771+4
	uxth	r9, r9
	cbnz	r1, .L2760
	ldr	r1, [r2, #976]
	mla	r3, r3, r7, r1
	ldr	r3, [r3, #8]
	ldr	r3, [r3, r9, lsl #2]
	str	r3, [r10]
	b	.L2761
.L2760:
	muls	r3, r7, r3
	ldr	r1, [r2, #976]
	ldr	r0, [r10]
	add	r1, r1, r3
	ldr	r1, [r1, #8]
	str	r0, [r1, r9, lsl #2]
	ldr	r1, [r2, #976]
	strh	r5, [r2, #980]	@ movhi
	add	r3, r3, r1
	ldr	r1, [r3, #4]
	orr	r1, r1, #-2147483648
	str	r1, [r3, #4]
.L2761:
	ldr	r2, [r2, #976]
	movs	r3, #12
	mla	r7, r3, r7, r2
	ldr	r3, [r7, #4]
	adds	r2, r3, #1
	beq	.L2768
	adds	r3, r3, #1
	str	r3, [r7, #4]
	b	.L2768
.L2770:
	ldr	r3, [r2, #4]
	cmp	r3, #0
	bge	.L2763
	bl	flush_l2p_region
	b	.L2763
.L2764:
	ldr	r2, [fp, #976]
	movs	r1, #1
	addw	r0, fp, #4056
	str	r8, [fp, #4060]
	add	r2, r2, r4
	ldr	r2, [r2, #8]
	str	r2, [fp, #4064]
	ldr	r2, [fp, #1804]
	str	r2, [fp, #4068]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r2, [fp, #4068]
	ldrh	r2, [r2, #8]
	cmp	r2, r5
	beq	.L2765
	mov	r1, r5
	mov	r2, r8
	ldr	r0, .L2771+8
	bl	printk
	movs	r2, #4
	ldr	r0, .L2771+12
	mov	r3, r2
	ldr	r1, [fp, #4068]
	bl	rknand_print_hex
	ldrh	r3, [r6, #2944]
	ldr	r0, .L2771+16
	movs	r2, #4
	ldr	r1, [fp, #1852]
	bl	rknand_print_hex
	movs	r3, #1
	str	r3, [fp, #740]
	b	.L2766
.L2765:
	ldr	r2, [fp, #4056]
	cmp	r2, #256
	bne	.L2766
	mov	r1, r5
	mov	r2, r8
	ldr	r0, .L2771+20
	bl	printk
	ldr	r3, [fp, #976]
	add	r0, fp, #1872
	mov	r1, r5
	add	r3, r3, r4
	ldr	r2, [r3, #8]
	bl	FtlMapWritePage
.L2766:
	ldr	r3, .L2771+4
	movs	r1, #0
	ldr	r3, [r3, #976]
	adds	r2, r3, r4
	str	r1, [r2, #4]
	strh	r5, [r3, r4]	@ movhi
	b	.L2759
.L2768:
	movs	r0, #0
.L2758:
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2772:
	.align	2
.L2771:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC136
	.word	.LC100
	.word	.LC137
	.word	.LC138
	.fnend
	.size	log2phys, .-log2phys
	.align	1
	.global	FtlWriteDumpData
	.thumb
	.thumb_func
	.type	FtlWriteDumpData, %function
FtlWriteDumpData:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	.pad #40
	sub	sp, sp, #40
	ldr	r4, .L2791
	ldr	r3, [r4, #740]
	cmp	r3, #0
	bne	.L2773
	ldrh	r2, [r4, #836]
	cmp	r2, #0
	beq	.L2775
	ldrb	r3, [r4, #840]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2775
	ldr	r7, .L2791+4
	ldrb	r1, [r4, #839]	@ zero_extendqisi2
	ldrh	r3, [r7, #2908]
	muls	r3, r1, r3
	cmp	r2, r3
	beq	.L2775
	ldrb	r9, [r4, #842]	@ zero_extendqisi2
	ldr	r6, [r7, #2964]
	ldrh	r8, [r7, #2840]
	cmp	r9, #0
	bne	.L2773
	subs	r6, r6, #1
	mov	r1, sp
	mov	r2, r9
	mov	r0, r6
	bl	log2phys
	ldr	r5, [r4, #1804]
	ldr	r0, [r4, #1780]
	ldr	r3, [sp]
	str	r6, [sp, #20]
	str	r0, [sp, #12]
	str	r3, [sp, #8]
	adds	r3, r3, #1
	str	r5, [sp, #16]
	str	r9, [r5, #4]
	beq	.L2777
	movs	r1, #1
	mov	r2, r9
	add	r0, sp, #4
	bl	FlashReadPages
	b	.L2778
.L2777:
	movs	r1, #255
	ldrh	r2, [r7, #2918]
	bl	ftl_memset
.L2778:
	movs	r7, #0
	ldr	r10, .L2791
	lsl	r8, r8, #2
	movw	r3, #61589
	mov	r9, r7
	strh	r3, [r5]	@ movhi
.L2779:
	cmp	r7, r8
	beq	.L2780
	ldrh	r3, [r4, #836]
	cbz	r3, .L2780
	ldr	r3, [sp, #8]
	adds	r7, r7, #1
	str	r6, [r5, #8]
	ldr	r0, .L2791+8
	str	r3, [r5, #12]
	ldrh	r3, [r10, #832]
	strh	r3, [r5, #2]	@ movhi
	bl	get_new_active_ppa
	ldr	r3, [r10, #1028]
	movs	r1, #1
	str	r0, [sp, #8]
	add	r0, sp, #4
	str	r3, [r5, #4]
	adds	r3, r3, #1
	adds	r2, r3, #1
	mov	r2, #0
	it	eq
	moveq	r3, r9
	str	r3, [r10, #1028]
	mov	r3, r2
	bl	FlashProgPages
	ldrh	r0, [r4, #832]
	bl	decrement_vpc_count
	b	.L2779
.L2780:
	movs	r3, #1
	b	.L2790
.L2775:
	movs	r3, #0
.L2790:
	strb	r3, [r4, #842]
.L2773:
	add	sp, sp, #40
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L2792:
	.align	2
.L2791:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2+832
	.fnend
	.size	FtlWriteDumpData, .-FtlWriteDumpData
	.align	1
	.global	l2p_flush
	.thumb
	.thumb_func
	.type	l2p_flush, %function
l2p_flush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	bl	FtlWriteDumpData
	movs	r4, #0
	ldr	r5, .L2798
	movs	r7, #12
	ldr	r6, .L2798+4
.L2794:
	ldrh	r3, [r5, #2946]
	uxth	r0, r4
	cmp	r3, r0
	bls	.L2797
	ldr	r3, [r6, #976]
	mla	r3, r7, r0, r3
	ldr	r3, [r3, #4]
	cmp	r3, #0
	bge	.L2795
	bl	flush_l2p_region
.L2795:
	adds	r4, r4, #1
	b	.L2794
.L2797:
	movs	r0, #0
	pop	{r3, r4, r5, r6, r7, pc}
.L2799:
	.align	2
.L2798:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	l2p_flush, .-l2p_flush
	.align	1
	.global	allocate_new_data_superblock
	.thumb
	.thumb_func
	.type	allocate_new_data_superblock, %function
allocate_new_data_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0
	ldr	r4, .L2826
	ldrh	r5, [r0]
	ldr	r3, [r4, #740]
	cmp	r3, #0
	bne	.L2801
	movw	r3, #65535
	cmp	r5, r3
	beq	.L2802
	ldr	r3, [r4, #812]
	mov	r0, r5
	ldrh	r3, [r3, r5, lsl #1]
	cbz	r3, .L2803
	bl	INSERT_DATA_LIST
	b	.L2802
.L2803:
	bl	INSERT_FREE_LIST
.L2802:
	ldr	r2, .L2826
	movs	r3, #0
	strb	r3, [r6, #8]
	add	r3, r2, #880
	cmp	r6, r3
	beq	.L2804
	ldr	r3, .L2826+4
	ldrh	r1, [r3, #2860]
	cmp	r1, #1
	beq	.L2804
	ldrb	r0, [r3, #1888]	@ zero_extendqisi2
	cbz	r0, .L2805
.L2804:
	movs	r3, #1
	strb	r3, [r6, #8]
	b	.L2806
.L2805:
	add	r0, r2, #832
	cmp	r6, r0
	bne	.L2806
	cmp	r1, #3
	beq	.L2808
	ldr	r2, [r2, #1120]
	cmp	r2, #1
	bne	.L2809
.L2808:
	movs	r2, #1
	strb	r2, [r4, #840]
.L2809:
	ldr	r3, [r3, #2832]
	cbz	r3, .L2806
	ldr	r3, [r4, #1048]
	cmp	r3, #39
	ittt	ls
	movls	r2, #1
	ldrls	r3, .L2826
	strbls	r2, [r3, #840]
.L2806:
	ldrh	r0, [r4, #1924]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L2811
	cmp	r5, r0
	bne	.L2812
	ldr	r3, .L2826
	ldr	r3, [r3, #812]
	ldrh	r3, [r3, r0, lsl #1]
	cbz	r3, .L2813
.L2812:
	bl	update_vpc_list
.L2813:
	movw	r3, #65535
	strh	r3, [r4, #1924]	@ movhi
.L2811:
	mov	r0, r6
	bl	allocate_data_superblock
	bl	l2p_flush
	movs	r0, #0
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L2801:
	movs	r0, #0
	pop	{r4, r5, r6, pc}
.L2827:
	.align	2
.L2826:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	allocate_new_data_superblock, .-allocate_new_data_superblock
	.align	1
	.global	FtlCheckVpc
	.thumb
	.thumb_func
	.type	FtlCheckVpc, %function
FtlCheckVpc:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #12
	movs	r4, #0
	ldr	r1, .L2848
	ldr	r0, .L2848+4
	bl	printk
	ldr	r0, .L2848+8
	mov	r1, #8192
	ldr	r5, .L2848+12
	bl	__memzero
	ldr	r7, .L2848+16
.L2829:
	ldr	r3, [r5, #2964]
	ldr	r6, .L2848+12
	cmp	r4, r3
	bcs	.L2846
	mov	r0, r4
	add	r1, sp, #4
	movs	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	adds	r3, r0, #1
	beq	.L2830
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	add	r0, r7, r0, lsl #1
	ldrh	r3, [r0, #-200]
	adds	r3, r3, #1
	strh	r3, [r0, #-200]	@ movhi
.L2830:
	adds	r4, r4, #1
	b	.L2829
.L2846:
	ldr	r8, .L2848+20
	movs	r4, #0
	ldr	r7, .L2848+16
.L2832:
	ldrh	r3, [r6, #2848]
	uxth	r1, r4
	ldr	r5, .L2848+20
	cmp	r3, r1
	bls	.L2847
	ldr	r3, [r8, #812]
	ldrh	r2, [r3, r1, lsl #1]
	add	r3, r7, r1, lsl #1
	ldrh	r3, [r3, #-200]
	cmp	r2, r3
	beq	.L2833
	ldr	r0, .L2848+24
	bl	printk
.L2833:
	adds	r4, r4, #1
	b	.L2832
.L2847:
	ldr	r3, [r5, #824]
	cbz	r3, .L2828
	ldr	r2, [r5, #804]
	movs	r6, #0
	ldrh	r7, [r5, #828]
	mov	r9, #6
	subs	r3, r3, r2
	ldr	r2, .L2848+28
	ldr	r8, .L2848+16
	asrs	r3, r3, #1
	muls	r3, r2, r3
	uxth	r4, r3
.L2836:
	uxth	r3, r6
	cmp	r3, r7
	bcs	.L2828
	ldr	r3, [r5, #812]
	ldrh	r2, [r3, r4, lsl #1]
	cbz	r2, .L2837
	add	r3, r8, r4, lsl #1
	ldr	r0, .L2848+32
	mov	r1, r4
	ldrh	r3, [r3, #-200]
	bl	printk
.L2837:
	mul	r4, r9, r4
	ldr	r3, [r5, #804]
	adds	r6, r6, #1
	ldrh	r4, [r3, r4]
	movw	r3, #65535
	cmp	r4, r3
	bne	.L2836
.L2828:
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, pc}
.L2849:
	.align	2
.L2848:
	.word	.LANCHOR3+72
	.word	.LC107
	.word	.LANCHOR4-200
	.word	.LANCHOR0
	.word	.LANCHOR4
	.word	.LANCHOR2
	.word	.LC139
	.word	-1431655765
	.word	.LC140
	.fnend
	.size	FtlCheckVpc, .-FtlCheckVpc
	.align	1
	.global	Ftlscanalldata
	.thumb
	.thumb_func
	.type	Ftlscanalldata, %function
Ftlscanalldata:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	movs	r1, #0
	.pad #36
	sub	sp, sp, #36
	ldr	r0, .L2865
	bl	printk
	ldr	r7, .L2865+4
	ldr	r5, .L2865+8
	movs	r4, #0
.L2851:
	ldr	r3, [r7, #2964]
	cmp	r4, r3
	bcs	.L2864
	mov	r0, r4
	add	r1, sp, #28
	movs	r2, #0
	bl	log2phys
	ubfx	r3, r4, #0, #11
	cbnz	r3, .L2852
	ldr	r0, .L2865+12
	mov	r1, r4
	ldr	r2, [sp, #28]
	bl	printk
.L2852:
	ldr	r3, [sp, #28]
	adds	r2, r3, #1
	beq	.L2854
	str	r3, [r5, #4060]
	movs	r2, #0
	ldr	r3, [r5, #1780]
	movs	r1, #1
	ldr	r6, [r5, #1804]
	ldr	r0, .L2865+16
	str	r3, [r5, #4064]
	str	r2, [r5, #4056]
	str	r4, [r5, #4072]
	str	r6, [r5, #4068]
	bl	FlashReadPages
	ldr	r3, [r5, #4056]
	ldr	r2, .L2865+8
	cmp	r3, #256
	beq	.L2855
	adds	r3, r3, #1
	beq	.L2855
	ldr	r3, [r6, #8]
	cmp	r3, r4
	beq	.L2854
.L2855:
	ldr	r3, [r2, #4068]
	ldr	r1, [r2, #4064]
	ldr	r0, [r3, #4]
	str	r0, [sp]
	ldr	r0, [r3, #8]
	str	r0, [sp, #4]
	ldr	r0, [r3, #12]
	str	r0, [sp, #8]
	ldr	r0, [r1]
	str	r0, [sp, #12]
	ldr	r1, [r1, #4]
	ldr	r0, .L2865+20
	str	r1, [sp, #16]
	mov	r1, r4
	ldr	r2, [r2, #4060]
	ldr	r3, [r3]
	bl	printk
.L2854:
	adds	r4, r4, #1
	b	.L2851
.L2864:
	add	sp, sp, #36
	@ sp needed
	pop	{r4, r5, r6, r7, pc}
.L2866:
	.align	2
.L2865:
	.word	.LC141
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC142
	.word	.LANCHOR2+4056
	.word	.LC143
	.fnend
	.size	Ftlscanalldata, .-Ftlscanalldata
	.align	1
	.global	FtlReUsePrevPpa
	.thumb
	.thumb_func
	.type	FtlReUsePrevPpa, %function
FtlReUsePrevPpa:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	.pad #12
	mov	r6, r0
	ubfx	r0, r1, #10, #16
	str	r1, [sp, #4]
	bl	P2V_block_in_plane
	ldr	r5, .L2876
	ldr	r4, [r5, #812]
	ldrh	r3, [r4, r0, lsl #1]
	cbnz	r3, .L2868
	ldr	r4, [r5, #824]
	cbz	r4, .L2869
	ldr	r1, [r5, #804]
	mov	lr, #6
	ldr	r2, .L2876+4
	movw	ip, #65535
	subs	r4, r4, r1
	ldrh	r7, [r5, #828]
	asrs	r4, r4, #1
	muls	r4, r2, r4
	uxth	r4, r4
.L2870:
	uxth	r2, r3
	cmp	r2, r7
	bcs	.L2869
	cmp	r4, r0
	bne	.L2871
	mov	r1, r4
	ldr	r0, .L2876+8
	bl	List_remove_node
	ldrh	r3, [r5, #828]
	mov	r0, r4
	subs	r3, r3, #1
	strh	r3, [r5, #828]	@ movhi
	bl	INSERT_DATA_LIST
	ldr	r2, [r5, #812]
	ldrh	r3, [r2, r4, lsl #1]
	adds	r3, r3, #1
	strh	r3, [r2, r4, lsl #1]	@ movhi
	b	.L2869
.L2871:
	mul	r4, lr, r4
	adds	r3, r3, #1
	ldrh	r4, [r1, r4]
	cmp	r4, ip
	bne	.L2870
	b	.L2869
.L2868:
	adds	r3, r3, #1
	strh	r3, [r4, r0, lsl #1]	@ movhi
.L2869:
	mov	r0, r6
	add	r1, sp, #4
	movs	r2, #1
	bl	log2phys
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, pc}
.L2877:
	.align	2
.L2876:
	.word	.LANCHOR2
	.word	-1431655765
	.word	.LANCHOR2+824
	.fnend
	.size	FtlReUsePrevPpa, .-FtlReUsePrevPpa
	.align	1
	.global	FtlRecoverySuperblock
	.thumb
	.thumb_func
	.type	FtlRecoverySuperblock, %function
FtlRecoverySuperblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movw	r7, #65535
	ldrh	r3, [r0]
	.pad #68
	sub	sp, sp, #68
	mov	r4, r0
	cmp	r3, r7
	beq	.L3022
	ldrh	r3, [r0, #2]
	movs	r5, #0
	ldr	r6, .L3038
	str	r3, [sp, #12]
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	ldr	r2, [sp, #12]
	str	r3, [sp, #28]
	ldrh	r3, [r6, #2908]
	cmp	r3, r2
	bne	.L2881
	strh	r5, [r0, #4]	@ movhi
	strb	r5, [r0, #6]
	b	.L3022
.L2881:
	ldrh	r0, [r0, #16]
.L2882:
	cmp	r0, r7
	add	r5, r5, #1
	bne	.L3033
	uxth	r3, r5
	adds	r3, r3, #8
	ldrh	r0, [r4, r3, lsl #1]
	b	.L2882
.L3033:
	ldrb	r1, [r4, #8]	@ zero_extendqisi2
	cmp	r1, #1
	bne	.L2884
	bl	FtlGetLastWrittenPage
	adds	r2, r0, #1
	mov	r5, r0
	beq	.L2885
	ldrb	r3, [r6, #1888]	@ zero_extendqisi2
	cbnz	r3, .L3024
	ldr	r3, .L3038
	add	r3, r3, r0, lsl #1
	ldrh	r7, [r3, #76]
	b	.L2959
.L2884:
	movs	r1, #0
	bl	FtlGetLastWrittenPage
	adds	r3, r0, #1
	mov	r5, r0
	beq	.L2885
.L3024:
	mov	r7, r5
.L2959:
	ldr	r3, .L3038+4
	movw	r9, #65535
	ldrh	ip, [r6, #2840]
	mov	r10, #36
	ldrh	r0, [r6, #2920]
	ldr	r2, [r3, #1768]
	ldr	r8, [r3, #1664]
	add	r3, r4, #14
	str	r3, [sp, #20]
	str	r2, [sp, #4]
	movs	r2, #0
	mov	lr, r3
	mov	r6, r2
	mov	fp, r2
	b	.L2887
.L2885:
	movs	r3, #0
	strh	r3, [r4, #2]	@ movhi
	strb	r3, [r4, #6]
	b	.L3022
.L2890:
	ldrh	r3, [lr, #2]!
	cmp	r3, r9
	beq	.L2888
	ldr	r1, [sp, #4]
	orr	r3, r7, r3, lsl #10
	mla	r1, r10, r6, r1
	str	r3, [r1, #4]
	mov	r3, r0
	muls	r3, r6, r3
	add	r6, r6, #1
	it	mi
	addmi	r3, r3, #3
	str	fp, [r1, #8]
	uxth	r6, r6
	bic	r3, r3, #3
	add	r3, r3, r8
	str	r3, [r1, #12]
.L2888:
	adds	r2, r2, #1
.L2887:
	uxth	r3, r2
	cmp	r3, ip
	bcc	.L2890
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L2961
	ldr	r3, .L3038
	ldrb	r1, [r3, #1888]	@ zero_extendqisi2
	adds	r3, r1, #0
	it	ne
	movne	r3, #1
	b	.L3025
.L2961:
	movs	r3, #0
.L3025:
	ldr	r8, .L3038+4
	mov	r1, r6
	str	r3, [sp, #24]
	mov	fp, #0
	ldr	r2, [sp, #24]
	ldr	r0, [r8, #1768]
	bl	FlashReadPages
	ldr	r3, [r8, #1028]
	subs	r3, r3, #1
	str	r8, [sp, #8]
	str	r3, [sp, #4]
	movw	r3, #65535
	str	r3, [sp, #16]
.L2892:
	uxth	r9, fp
	cmp	r9, r6
	bcs	.L2899
	movs	r1, #36
	ldr	r0, [r8, #1768]
	mul	r1, r1, fp
	add	lr, r0, r1
	ldr	r1, [r0, r1]
	cbnz	r1, .L2893
	ldr	r3, [lr, #12]
	ldr	r10, [r3, #4]
	cmp	r10, #-1
	beq	.L2894
	mov	r0, r10
	ldr	r1, [r8, #1028]
	str	r3, [sp, #32]
	bl	ftl_cmp_data_ver
	ldr	r3, [sp, #32]
	cbz	r0, .L2894
	add	r1, r10, #1
	str	r1, [r8, #1028]
.L2894:
	ldr	r1, [r3]
	adds	r1, r1, #1
	bne	.L2895
.L2899:
	cmp	r9, r6
	ldr	r6, .L3038+4
	bne	.L3023
	ldr	r3, [r6, #1768]
	adds	r5, r5, #1
	uxth	r10, r5
	ldr	r0, [r3, #4]
	b	.L3026
.L2893:
	ldr	r1, [lr, #4]
	ldr	r0, .L3038+8
	bl	printk
	uxth	r3, r7
	ldrh	r1, [r4]
	str	r3, [sp, #16]
	ldr	r3, .L3038+12
	strh	r1, [r3, #3648]	@ movhi
.L2895:
	add	fp, fp, #1
	b	.L2892
.L3023:
	ldr	r3, [sp, #8]
	movs	r2, #36
	uxth	r10, r5
	ldr	r3, [r3, #1768]
	mla	r9, r2, r9, r3
	ldr	r0, [r9, #4]
.L3026:
	ubfx	r0, r0, #10, #16
	bl	P2V_plane
	ldrb	r5, [r4, #8]	@ zero_extendqisi2
	ldr	r2, .L3038
	cmp	r5, #1
	str	r0, [sp, #8]
	bne	.L2901
	ldrb	r3, [r2, #1888]	@ zero_extendqisi2
	cbnz	r3, .L2901
	add	r3, r2, r10, lsl #1
	ldrh	r10, [r3, #76]
.L2901:
	ldrh	r3, [r2, #2908]
	cmp	r3, r10
	itttt	eq
	strheq	r10, [r4, #2]	@ movhi
	moveq	r3, #0
	strbeq	r3, [r4, #6]
	strheq	r3, [r4, #4]	@ movhi
	ldrh	r3, [sp, #28]
	str	r3, [sp, #32]
	ldr	r3, [sp, #12]
	cmp	r10, r3
	bne	.L2903
	ldr	r3, [sp, #8]
	ldr	r2, [sp, #32]
	cmp	r3, r2
	bne	.L2903
	mov	r0, r4
	mov	r1, r10
	mov	r2, r3
	b	.L3031
.L2903:
	ldr	r2, [sp, #16]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L2904
	cmp	r5, #0
	bne	.L2905
.L2904:
	ldr	r3, [r6, #1936]
	uxth	fp, r7
	mov	r9, #-1
	ldr	r8, .L3038+4
	adds	r3, r3, #1
	mov	r7, r9
	ittt	eq
	ldreq	r3, .L3038+4
	ldreq	r2, [sp, #4]
	streq	r2, [r3, #1936]
	ldr	r3, [r6, #1936]
	str	r3, [sp, #16]
	ldr	r3, [sp, #12]
	adds	r3, r3, #7
	cmp	fp, r3
	itet	gt
	subgt	r5, fp, #7
	ldrle	r5, [sp, #12]
	uxthgt	r5, r5
.L2908:
	cmp	r5, fp
	bhi	.L2921
	ldr	r3, .L3038
	ldr	r1, [sp, #20]
	ldrh	ip, [r3, #2840]
	ldr	r3, [r8, #1768]
	str	r3, [sp, #28]
	movs	r3, #0
	mov	r6, r3
.L2922:
	uxth	r2, r3
	cmp	r2, ip
	bcs	.L3034
	ldrh	r2, [r1, #2]!
	movw	r0, #65535
	cmp	r2, r0
	beq	.L2909
	ldr	r0, [sp, #28]
	mov	lr, #36
	orr	r2, r5, r2, lsl #10
	mla	r0, lr, r6, r0
	adds	r6, r6, #1
	uxth	r6, r6
	str	r2, [r0, #4]
.L2909:
	adds	r3, r3, #1
	b	.L2922
.L3034:
	mov	r1, r6
	ldr	r2, [sp, #24]
	ldr	r0, [r8, #1768]
	bl	FlashReadPages
	ldr	r3, .L3038
	movs	r2, #36
	movw	r1, #65535
	ldrb	r0, [r3, #1888]	@ zero_extendqisi2
	ldr	r3, [r8, #1768]
	mla	r6, r2, r6, r3
	ldr	r2, .L3038
	add	lr, r2, r5, lsl #1
.L2911:
	cmp	r3, r6
	beq	.L3035
	ldr	r2, [r3]
	cbnz	r2, .L2912
	ldr	r2, [r3, #12]
	ldrh	ip, [r2]
	cmp	ip, r1
	beq	.L2913
	ldr	r2, [r2, #4]
	cmp	r2, #-1
	beq	.L2913
	cmp	r9, #-1
	ldr	r7, [r8, #1936]
	str	r2, [r8, #1936]
	bne	.L2913
	ldrh	r2, [lr, #588]
	cmp	r2, r1
	bne	.L2914
	cmp	r0, #0
	beq	.L2913
.L2914:
	ldr	r2, [sp, #4]
	cmp	r7, r2
	ite	ne
	movne	r9, r7
	moveq	r9, #-1
	b	.L2913
.L3039:
	.align	2
.L3038:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC144
	.word	.LANCHOR5
.L2912:
	ldr	r3, .L3040
	ldrh	r2, [r4]
	strh	r2, [r3, #3648]	@ movhi
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cbnz	r3, .L2905
	ldr	r3, .L3040+4
	add	r5, r3, r5, lsl #1
	movw	r3, #65535
	ldrh	r2, [r5, #588]
	cmp	r2, r3
	ldr	r3, .L3040+8
	bne	.L2916
	cmp	r9, #-1
	beq	.L2917
	str	r9, [r3, #1936]
	b	.L2905
.L2917:
	ldr	r2, [sp, #16]
	ldr	r1, [sp, #4]
	cmp	r2, r1
	bne	.L3027
	ldr	r2, [r3, #1936]
	b	.L3032
.L2916:
	ldr	r2, [sp, #4]
	cmp	r7, r2
	beq	.L2919
	adds	r2, r7, #1
	beq	.L2905
	str	r7, [r3, #1936]
	b	.L2905
.L2919:
	ldr	r2, [r3, #1936]
	ldr	r1, [sp, #4]
	cmp	r2, r1
	beq	.L2905
.L3032:
	subs	r2, r2, #1
	b	.L3027
.L2913:
	adds	r3, r3, #36
	b	.L2911
.L3035:
	adds	r5, r5, #1
	uxth	r5, r5
	b	.L2908
.L2921:
	ldr	r3, .L3040+8
	mov	r2, #-1
.L3027:
	str	r2, [r3, #1936]
.L2905:
	ldr	r3, .L3040
	movs	r2, #1
	ldr	fp, [sp, #12]
	ldr	r5, .L3040+8
	strh	r2, [r3, #3650]	@ movhi
.L2923:
	ldr	r3, .L3040+4
	movw	r9, #65535
	ldr	r1, [r5, #1768]
	movs	r6, #0
	ldr	r7, [sp, #20]
	ldrh	r8, [r3, #2840]
	ldrb	r0, [r3, #1888]	@ zero_extendqisi2
	str	r6, [sp, #16]
.L2924:
	uxth	r3, r6
	cmp	r3, r8
	bcs	.L3036
	ldrh	r3, [r7, #2]!
	cmp	r3, r9
	beq	.L2925
	ldr	r2, [sp, #16]
	mov	lr, #36
	orr	r3, fp, r3, lsl #10
	mla	r2, lr, r2, r1
	str	r3, [r2, #4]
	ldrb	lr, [r4, #8]	@ zero_extendqisi2
	cmp	lr, #1
	bne	.L2926
	cbz	r0, .L2926
	orr	r3, r3, #-2147483648
	str	r3, [r2, #4]
.L2926:
	ldr	r3, [sp, #16]
	add	ip, r3, #1
	uxth	r3, ip
	str	r3, [sp, #16]
.L2925:
	adds	r6, r6, #1
	b	.L2924
.L3036:
	ldr	r0, [r5, #1768]
	ldr	r1, [sp, #16]
	ldr	r2, [sp, #24]
	bl	FlashReadPages
	movs	r3, #0
.L3030:
	str	r3, [sp, #28]
	ldr	r2, [sp, #16]
	ldrh	r3, [sp, #28]
	cmp	r3, r2
	bcs	.L3037
	ldr	r3, [sp, #28]
	movs	r6, #36
	ldr	r9, [r5, #1768]
	muls	r6, r3, r6
	add	r8, r9, r6
	ldr	r7, [r8, #4]
	ubfx	r0, r7, #10, #16
	str	r7, [sp, #60]
	bl	P2V_plane
	ldr	r3, [sp, #12]
	cmp	fp, r3
	bcc	.L2929
	bne	.L2930
	ldr	r3, [sp, #32]
	cmp	r0, r3
	bcc	.L2929
.L2930:
	cmp	fp, r10
	bne	.L2931
	ldr	r3, [sp, #8]
	cmp	r0, r3
	beq	.L2932
.L2931:
	ldr	r3, [r9, r6]
	adds	r3, r3, #1
	beq	.L2933
	ldr	r8, [r8, #12]
	movw	r3, #61589
	ldrh	r2, [r8]
	cmp	r2, r3
	beq	.L2934
	ldrh	r0, [r4]
	b	.L3028
.L2934:
	ldr	r3, [r8, #4]
	adds	r7, r3, #1
	str	r3, [sp, #4]
	beq	.L2935
	mov	r0, r3
	ldr	r1, [r5, #1028]
	bl	ftl_cmp_data_ver
	cbz	r0, .L2935
	ldr	r3, [sp, #4]
	adds	r3, r3, #1
	str	r3, [r5, #1028]
.L2935:
	ldr	r7, [r8, #8]
	add	r1, sp, #56
	ldr	r3, [r8, #12]
	movs	r2, #0
	mov	r0, r7
	str	r3, [sp, #52]
	bl	log2phys
	ldr	r1, [r5, #1936]
	adds	r0, r1, #1
	beq	.L2936
	ldr	r0, [sp, #4]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L2936
	ldr	r3, [sp, #52]
	adds	r2, r3, #1
	beq	.L2937
	ldr	r0, [r5, #1768]
	movs	r2, #0
	movs	r1, #1
	add	r0, r0, r6
	str	r3, [r0, #4]
	ldr	r9, [r0, #12]
	bl	FlashReadPages
	ldr	r2, [r5, #1768]
	ldr	r3, [r9, #4]
	add	ip, r2, r6
	str	r3, [sp, #36]
	ldr	r3, [r2, r6]
	adds	r3, r3, #1
	bne	.L2938
	b	.L2939
.L2937:
	ldr	r3, [sp, #60]
	ldr	r2, [sp, #56]
	cmp	r2, r3
	bne	.L2929
	mov	r0, r7
	add	r1, sp, #52
	movs	r2, #1
	bl	log2phys
	b	.L2929
.L2938:
	ldr	r8, [r9, #8]
	cmp	r8, r7
	bne	.L2939
	ldr	r0, [r5, #1936]
	ldr	r1, [sp, #36]
	str	r2, [sp, #44]
	str	ip, [sp, #40]
	bl	ftl_cmp_data_ver
	ldr	ip, [sp, #40]
	ldr	r2, [sp, #44]
	cbz	r0, .L2939
	ldr	r3, [sp, #56]
	ldr	r1, [sp, #60]
	cmp	r3, r1
	beq	.L2944
	ldr	r1, [sp, #52]
	cmp	r3, r1
	beq	.L2939
	adds	r0, r3, #1
	beq	.L2942
	str	r3, [ip, #4]
	mov	r0, ip
	movs	r1, #1
	movs	r2, #0
	ldr	r9, [ip, #12]
	bl	FlashReadPages
	b	.L2943
.L2942:
	str	r3, [r2, r6]
.L2943:
	ldr	r3, [r5, #1768]
	ldr	r3, [r3, r6]
	adds	r3, r3, #1
	beq	.L2944
	ldr	r6, [r9, #4]
	ldr	r0, [r5, #1936]
	mov	r1, r6
	bl	ftl_cmp_data_ver
	cbz	r0, .L2944
	ldr	r0, [sp, #36]
	mov	r1, r6
	bl	ftl_cmp_data_ver
	cbz	r0, .L2939
.L2944:
	mov	r0, r8
	ldr	r1, [sp, #52]
	bl	FtlReUsePrevPpa
.L2939:
	mov	r3, #-1
	str	r3, [sp, #52]
	b	.L2946
.L3041:
	.align	2
.L3040:
	.word	.LANCHOR5
	.word	.LANCHOR0
	.word	.LANCHOR2
.L2936:
	ldr	r3, [sp, #60]
	ldr	r2, [sp, #56]
	cmp	r2, r3
	beq	.L2946
	ldr	r2, .L3042
	ldr	r3, [sp, #52]
	ldr	r2, [r2, #2856]
	cmp	r2, r3, lsr #10
	bhi	.L2948
	adds	r3, r3, #1
	bne	.L2929
.L2948:
	mov	r0, r7
	add	r1, sp, #60
	movs	r2, #1
	bl	log2phys
	ldr	r8, [sp, #56]
	cmp	r8, #-1
	beq	.L2946
	ldr	r3, [sp, #52]
	cmp	r8, r3
	beq	.L2946
	ubfx	r0, r8, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r5, #832]
	cmp	r3, r0
	beq	.L2950
	ldrh	r3, [r5, #880]
	cmp	r3, r0
	beq	.L2950
	ldrh	r3, [r5, #928]
	cmp	r3, r0
	bne	.L2946
.L2950:
	ldr	r3, .L3042+4
	movs	r1, #1
	movs	r2, #0
	ldr	r0, [r3, #1768]
	str	r8, [r0, #4]
	ldr	r6, [r0, #12]
	bl	FlashReadPages
	ldr	r3, .L3042+4
	ldr	r1, [r6, #4]
	ldr	r3, [r3, #1768]
	ldr	r3, [r3]
	adds	r3, r3, #1
	beq	.L2946
	ldr	r0, [sp, #4]
	bl	ftl_cmp_data_ver
	cbnz	r0, .L2946
	mov	r0, r7
	add	r1, sp, #56
	movs	r2, #1
	bl	log2phys
.L2946:
	ldr	r0, [sp, #52]
	adds	r1, r0, #1
	beq	.L2929
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, [r5, #812]
	ldrh	r3, [r3, r0, lsl #1]
	mov	r1, r0
	cbz	r3, .L2951
.L3028:
	bl	decrement_vpc_count
	b	.L2929
.L2951:
	ldr	r0, .L3042+8
	bl	printk
	b	.L2929
.L2933:
	ldrh	r3, [r4]
	mov	r1, r7
	ldr	r2, .L3042+12
	ldr	r0, .L3042+16
	strh	r3, [r2, #3648]	@ movhi
	ldr	r2, [sp, #4]
	bl	printk
	ldr	r3, .L3042+12
	ldr	r3, [r3, #3652]
	cmp	r3, #31
	bhi	.L2952
	ldr	r2, .L3042+12
	ldr	r1, [sp, #60]
	add	r2, r2, r3, lsl #2
	adds	r3, r3, #1
	str	r1, [r2, #3656]
	ldr	r2, .L3042+12
	str	r3, [r2, #3652]
.L2952:
	ldrh	r0, [r4]
	bl	decrement_vpc_count
	ldr	r3, [r5, #1936]
	adds	r2, r3, #1
	bne	.L2953
	ldr	r3, [sp, #4]
	b	.L3029
.L2953:
	ldr	r2, [sp, #4]
	cmp	r3, r2
	bls	.L2929
	mov	r3, r2
.L3029:
	str	r3, [r5, #1936]
.L2929:
	ldr	r3, [sp, #28]
	adds	r3, r3, #1
	b	.L3030
.L3037:
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	add	fp, fp, #1
	cmp	r3, #1
	uxth	fp, fp
	ldr	r3, .L3042
	bne	.L2955
	ldrb	r2, [r3, #1888]	@ zero_extendqisi2
	cbz	r2, .L2955
	ldrh	r2, [r3, #2910]
	cmp	r2, fp
	bne	.L2955
	cmp	r10, fp
	beq	.L2932
.L2955:
	ldrh	r3, [r3, #2908]
	cmp	fp, r3
	bne	.L2923
	ldr	r2, .L3042
	movw	r0, #65535
	movs	r3, #0
	strh	fp, [r4, #2]	@ movhi
	strh	r3, [r4, #4]	@ movhi
	ldrh	r1, [r2, #2840]
.L2956:
	uxth	r2, r3
	cmp	r2, r1
	bcs	.L3022
	ldr	r6, [sp, #20]
	adds	r3, r3, #1
	ldrh	r5, [r6, #2]!
	cmp	r5, r0
	str	r6, [sp, #20]
	beq	.L2956
	strb	r2, [r4, #6]
	b	.L3022
.L2932:
	ldrb	r3, [sp, #8]	@ zero_extendqisi2
	mov	r0, r4
	ldr	r2, [sp, #8]
	mov	r1, r10
	strh	r10, [r4, #2]	@ movhi
	strb	r3, [r4, #6]
.L3031:
	bl	ftl_sb_update_avl_pages
.L3022:
	movs	r0, #0
	add	sp, sp, #68
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3043:
	.align	2
.L3042:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC145
	.word	.LANCHOR5
	.word	.LC146
	.fnend
	.size	FtlRecoverySuperblock, .-FtlRecoverySuperblock
	.align	1
	.global	FtlVpcCheckAndModify
	.thumb
	.thumb_func
	.type	FtlVpcCheckAndModify, %function
FtlVpcCheckAndModify:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #12
	movs	r4, #0
	ldr	r7, .L3059
	ldr	r1, .L3059+4
	ldr	r0, .L3059+8
	bl	printk
	ldr	r5, .L3059+12
	ldrh	r2, [r7, #2850]
	movs	r1, #0
	ldr	r0, [r5, #1828]
	lsls	r2, r2, #1
	bl	ftl_memset
.L3045:
	ldr	r3, [r7, #2964]
	cmp	r4, r3
	bcs	.L3057
	mov	r0, r4
	add	r1, sp, #4
	movs	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	adds	r3, r0, #1
	beq	.L3046
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r5, #1828]
	ldrh	r3, [r2, r0, lsl #1]
	adds	r3, r3, #1
	strh	r3, [r2, r0, lsl #1]	@ movhi
.L3046:
	adds	r4, r4, #1
	b	.L3045
.L3057:
	ldr	r6, .L3059+12
	mov	r8, #0
	movw	r9, #65535
.L3048:
	ldrh	r3, [r7, #2848]
	uxth	r4, r8
	cmp	r3, r4
	bls	.L3058
	ldr	r3, [r5, #812]
	ldrh	r2, [r3, r4, lsl #1]
	ldr	r3, [r5, #1828]
	ldrh	r3, [r3, r4, lsl #1]
	cmp	r2, r3
	beq	.L3050
	cmp	r2, r9
	beq	.L3050
	ldr	r0, .L3059+16
	mov	r1, r4
	bl	printk
	ldrh	r3, [r6, #832]
	cmp	r3, r4
	beq	.L3050
	ldrh	r3, [r6, #928]
	cmp	r3, r4
	beq	.L3050
	ldrh	r3, [r6, #880]
	cmp	r3, r4
	beq	.L3050
	ldr	r3, [r6, #812]
	ldrh	r2, [r3, r4, lsl #1]
	cbnz	r2, .L3052
	ldr	r2, [r6, #1828]
	ldrh	r2, [r2, r4, lsl #1]
	strh	r2, [r3, r4, lsl #1]	@ movhi
	b	.L3050
.L3052:
	ldr	r2, [r6, #1828]
	mov	r0, r4
	ldrh	r2, [r2, r4, lsl #1]
	strh	r2, [r3, r4, lsl #1]	@ movhi
	bl	update_vpc_list
.L3050:
	add	r8, r8, #1
	b	.L3048
.L3058:
	bl	l2p_flush
	bl	FtlVpcTblFlush
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, pc}
.L3060:
	.align	2
.L3059:
	.word	.LANCHOR0
	.word	.LANCHOR3+84
	.word	.LC107
	.word	.LANCHOR2
	.word	.LC147
	.fnend
	.size	FtlVpcCheckAndModify, .-FtlVpcCheckAndModify
	.align	1
	.global	FtlGcScanTempBlk
	.thumb
	.thumb_func
	.type	FtlGcScanTempBlk, %function
FtlGcScanTempBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 72
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L3108
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #76
	sub	sp, sp, #76
	ldrh	r5, [r3, #3288]
	movw	r3, #65535
	mov	r4, r0
	str	r1, [sp, #16]
	cmp	r5, r3
	beq	.L3094
	cbnz	r5, .L3062
	b	.L3063
.L3094:
	movs	r5, #0
.L3062:
	ldr	r3, .L3108+4
	ldr	r2, [sp, #16]
	ldrh	r3, [r3, #2908]
	cmp	r2, r3
	bne	.L3064
.L3063:
	bl	FtlGcPageVarInit
.L3064:
	ldr	r6, .L3108+8
	mov	r3, #-1
	str	r3, [sp, #12]
	movs	r3, #0
	str	r3, [sp, #4]
.L3065:
	ldrh	r2, [r4]
	movs	r3, #0
	strb	r3, [r4, #8]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L3066
.L3067:
	ldr	r10, .L3108+4
.L3091:
	ldr	r3, [r6, #1768]
	movs	r2, #0
	ldrh	r9, [r10, #2840]
	add	lr, r4, #14
	ldrh	fp, [r10, #2918]
	mov	r8, #36
	str	r3, [sp, #8]
	mov	r7, r2
	ldr	r3, [r6, #1660]
	movw	ip, #65535
	str	r3, [sp, #20]
	ldr	r3, [r6, #1664]
	str	r3, [sp, #24]
	ldrh	r3, [r10, #2920]
	str	r3, [sp, #28]
.L3068:
	uxth	r3, r2
	cmp	r3, r9
	bcs	.L3106
	ldrh	r3, [lr, #2]!
	cmp	r3, ip
	beq	.L3069
	ldr	r1, [sp, #8]
	orr	r3, r5, r3, lsl #10
	ldr	r0, [sp, #20]
	mla	r1, r8, r7, r1
	str	r3, [r1, #4]
	mov	r3, fp
	muls	r3, r7, r3
	it	mi
	addmi	r3, r3, #3
	bic	r3, r3, #3
	add	r3, r3, r0
	str	r3, [r1, #8]
	ldr	r3, [sp, #28]
	muls	r3, r7, r3
	add	r7, r7, #1
	it	mi
	addmi	r3, r3, #3
	ldr	r0, [sp, #24]
	bic	r3, r3, #3
	uxth	r7, r7
	add	r3, r3, r0
	str	r3, [r1, #12]
.L3069:
	adds	r2, r2, #1
	b	.L3068
.L3106:
	ldr	r0, [r6, #1768]
	mov	r1, r7
	movs	r2, #0
	mov	fp, #0
	bl	FlashReadPages
.L3073:
	uxth	r3, fp
	cmp	r3, r7
	bcs	.L3107
	mov	r9, #36
	ldr	r2, [r6, #1768]
	ldr	ip, .L3108+8
	mul	r9, r9, fp
	add	r3, r2, r9
	ldr	r1, [r3, #4]
	ldr	r8, [r2, r9]
	ldr	r3, [r3, #12]
	str	r1, [sp, #8]
	cmp	r8, #0
	bne	.L3074
	ldrh	r1, [r3]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L3075
.L3077:
	ldr	r3, .L3108+8
	movs	r1, #0
	ldrh	r2, [r4]
	ldr	r3, [r3, #812]
	strh	r1, [r3, r2, lsl #1]	@ movhi
	ldrh	r0, [r4]
	bl	INSERT_FREE_LIST
	ldr	r2, .L3108+8
	movw	r3, #65535
	strh	r3, [r4]	@ movhi
	strh	r3, [r2, #1072]	@ movhi
	b	.L3105
.L3075:
	ldr	r0, [r3, #8]
	ldr	r2, [r10, #2964]
	cmp	r0, r2
	bhi	.L3077
	ldrb	r2, [r10]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L3079
	add	r1, sp, #32
	mov	r2, r8
	str	r3, [sp, #20]
	bl	log2phys
	ldr	r3, [sp, #20]
	ldr	r2, [sp, #32]
	ldr	r1, [r3, #12]
	cmp	r2, r1
	bne	.L3079
	adds	r1, r2, #1
	beq	.L3079
	str	r2, [sp, #40]
	movs	r1, #1
	ldr	r2, [r6, #1800]
	add	r0, sp, #36
	str	r2, [sp, #44]
	ldr	r2, [r6, #1808]
	str	r2, [sp, #48]
	mov	r2, r8
	bl	FlashReadPages
	ldrh	r2, [r10, #2914]
	ldr	r1, [r6, #1768]
	lsl	lr, r2, #7
	add	r9, r9, r1
	ldr	r0, [sp, #44]
	ldr	r3, [sp, #20]
.L3081:
	cmp	r8, lr
	beq	.L3079
	ldr	r1, [r9, #8]
	ldr	r2, [r0, r8, lsl #2]
	ldr	r1, [r1, r8, lsl #2]
	cmp	r1, r2
	beq	.L3082
	ldrh	r1, [r4]
	ldr	r2, [sp, #40]
	ldr	r0, .L3108+12
	bl	printk
	ldr	r3, [r6, #812]
	ldrh	r2, [r4]
	movs	r1, #0
	strh	r1, [r3, r2, lsl #1]	@ movhi
	ldrh	r0, [r4]
	bl	INSERT_FREE_LIST
	movw	r3, #65535
	strh	r3, [r4]	@ movhi
	strh	r3, [r6, #1072]	@ movhi
	b	.L3105
.L3082:
	add	r8, r8, #1
	b	.L3081
.L3079:
	ldr	r0, [r3, #12]
	add	fp, fp, #1
	ldr	r1, [sp, #8]
	ldr	r2, [r3, #8]
	bl	FtlGcUpdatePage
	b	.L3073
.L3074:
	ldrh	r1, [r4]
	ldr	r2, [sp, #8]
	ldr	r0, .L3108+16
	str	ip, [sp, #20]
	bl	printk
	ldr	r2, .L3108+4
	ldrh	r3, [r4]
	ldr	r1, [r2, #2832]
	ldr	ip, [sp, #20]
	cbnz	r1, .L3085
	ldrb	r2, [r2, #1888]	@ zero_extendqisi2
	cbz	r2, .L3086
.L3085:
	ldr	r2, [ip, #748]
	ldrh	r2, [r2, r3, lsl #1]
	cmp	r2, #159
	bls	.L3087
.L3086:
	ldr	r2, [ip, #1768]
	ldr	r2, [r2, r9]
	adds	r2, r2, #1
	bne	.L3088
.L3087:
	ldr	r2, [ip, #1768]
	add	r9, r9, r2
	ldr	r2, [r9, #4]
	str	r2, [sp, #12]
.L3088:
	ldr	r2, .L3108+8
	movs	r1, #0
	ldr	r2, [r2, #812]
	strh	r1, [r2, r3, lsl #1]	@ movhi
	ldrh	r0, [r4]
	bl	INSERT_FREE_LIST
	movw	r3, #65535
	strh	r3, [r4]	@ movhi
.L3105:
	bl	FtlGcPageVarInit
	movs	r5, #0
	b	.L3065
.L3107:
	ldr	r3, [sp, #4]
	adds	r5, r5, #1
	ldr	r2, [sp, #16]
	adds	r3, r3, #1
	uxth	r5, r5
	cmp	r3, r2
	str	r3, [sp, #4]
	bcs	.L3090
.L3092:
	ldrh	r3, [r10, #2908]
	cmp	r3, r5
	bhi	.L3091
	b	.L3066
.L3090:
	ldr	r2, .L3108
	movw	r1, #65535
	ldrh	r3, [r2, #3288]
	cmp	r3, r1
	beq	.L3092
	ldr	r1, [sp, #4]
	add	r3, r3, r1
	strh	r3, [r2, #3288]	@ movhi
	ldrh	r3, [r10, #2908]
	cmp	r3, r5
	bls	.L3092
	b	.L3093
.L3066:
	ldr	r3, .L3108
	movw	r2, #65535
	strh	r5, [r4, #2]	@ movhi
	mov	r0, r4
	mov	r1, r5
	strh	r2, [r3, #3288]	@ movhi
	movs	r2, #0
	strb	r2, [r4, #6]
	bl	ftl_sb_update_avl_pages
.L3093:
	ldr	r0, [sp, #12]
	add	sp, sp, #76
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3109:
	.align	2
.L3108:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC148
	.word	.LC149
	.fnend
	.size	FtlGcScanTempBlk, .-FtlGcScanTempBlk
	.align	1
	.global	FtlReadRefresh
	.thumb
	.thumb_func
	.type	FtlReadRefresh, %function
FtlReadRefresh:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #44
	sub	sp, sp, #44
	ldr	r5, .L3126
	ldr	r4, .L3126+4
	ldr	r9, [r5, #1204]
	mov	r6, r5
	cmp	r9, #0
	beq	.L3111
	ldr	r2, [r5, #1208]
	ldr	r3, [r4, #2964]
	cmp	r2, r3
	bcs	.L3112
	mov	r5, #2048
	mov	r7, r6
.L3117:
	ldr	r0, [r6, #1208]
	ldr	r3, [r4, #2964]
	cmp	r0, r3
	bcs	.L3116
	mov	r1, sp
	movs	r2, #0
	bl	log2phys
	ldr	r3, [r7, #1208]
	adds	r3, r3, #1
	str	r3, [r7, #1208]
	ldr	r2, [sp]
	adds	r1, r2, #1
	beq	.L3115
	add	r0, sp, #40
	str	r2, [sp, #8]
	movs	r1, #1
	movs	r2, #0
	str	r2, [r0, #-36]!
	str	r3, [sp, #20]
	str	r2, [sp, #12]
	str	r2, [sp, #16]
	bl	FlashReadPages
	ldr	r3, [sp, #4]
	cmp	r3, #256
	bne	.L3116
	ldr	r0, [sp]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L3116:
	mov	r0, #-1
	b	.L3119
.L3115:
	subs	r5, r5, #1
	bne	.L3117
	b	.L3116
.L3112:
	ldr	r3, [r5, #992]
	movs	r0, #0
	str	r0, [r5, #1204]
	str	r0, [r5, #1208]
	str	r3, [r5, #1200]
	b	.L3119
.L3111:
	ldr	r8, [r5, #992]
	ldr	r1, [r5, #1044]
	ldr	r7, [r5, #1200]
	add	r2, r8, #1048576
	ldr	r3, [r4, #2964]
	movw	r4, #10000
	cmp	r1, r4
	ite	hi
	movhi	r4, #31
	movls	r4, #63
	cmp	r7, r2
	bhi	.L3121
	mov	r0, #1000
	lsrs	r1, r1, #10
	adds	r1, r1, #1
	muls	r0, r3, r0
	bl	__aeabi_uidiv
	add	r0, r0, r7
	cmp	r0, r8
	bcc	.L3121
	ldrh	r3, [r5, #784]
	ands	r0, r4, r3
	bne	.L3123
	ldr	r2, [r5, #1224]
	cmp	r2, r3
	beq	.L3119
.L3121:
	ldrh	r3, [r6, #784]
	movs	r0, #0
	str	r8, [r6, #1200]
	str	r0, [r6, #1208]
	str	r3, [r6, #1224]
	movs	r3, #1
	str	r3, [r6, #1204]
	b	.L3119
.L3123:
	mov	r0, r9
.L3119:
	add	sp, sp, #44
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, pc}
.L3127:
	.align	2
.L3126:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlReadRefresh, .-FtlReadRefresh
	.align	1
	.global	FtlGcFreeTempBlock
	.thumb
	.thumb_func
	.type	FtlGcFreeTempBlock, %function
FtlGcFreeTempBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r5, .L3166
	ldr	r6, .L3166+4
	ldr	r2, [r5, #740]
	mov	r4, r5
	ldrh	r1, [r6, #2908]
	cmp	r2, #0
	bne	.L3164
	ldrh	r5, [r5, #928]
	movw	r7, #65535
	cmp	r5, r7
	bne	.L3131
.L3140:
	ldrh	r2, [r4, #928]
	movw	r3, #65535
	movs	r7, #0
	ldr	r5, .L3166
	cmp	r2, r3
	str	r7, [r4, #1932]
	beq	.L3164
	bl	FtlCacheWriteBack
	ldrb	r0, [r5, #935]	@ zero_extendqisi2
	ldrh	r3, [r6, #2908]
	mov	r10, #12
	ldr	r2, [r5, #812]
	ldrh	r1, [r5, #928]
	smulbb	r3, r0, r3
	strh	r3, [r2, r1, lsl #1]	@ movhi
	ldr	r3, [r5, #1012]
	ldrh	r2, [r5, #1688]
	add	r3, r3, r2
	str	r3, [r5, #1012]
	b	.L3141
.L3131:
	cbz	r0, .L3134
	ldr	r3, .L3166+8
	ldrh	r0, [r3, #3288]
	cmp	r0, r7
	beq	.L3135
.L3136:
	movs	r1, #2
	b	.L3134
.L3135:
	strh	r2, [r3, #3288]	@ movhi
	ldrh	r3, [r4, #828]
	cmp	r3, #17
	bhi	.L3136
.L3134:
	ldr	r7, .L3166
	add	r0, r7, #928
	bl	FtlGcScanTempBlk
	str	r0, [sp, #4]
	adds	r0, r0, #1
	beq	.L3137
	ldr	r2, [r7, #748]
	ldrh	r3, [r2, r5, lsl #1]
	cmp	r3, #4
	bls	.L3138
	subs	r3, r3, #5
	movs	r0, #1
	strh	r3, [r2, r5, lsl #1]	@ movhi
	bl	FtlEctTblFlush
.L3138:
	ldr	r3, [r4, #1932]
	ldr	r2, .L3166
	cbnz	r3, .L3139
	ldr	r0, [sp, #4]
	ldr	r3, [r2, #1220]
	ubfx	r0, r0, #10, #16
	adds	r3, r3, #1
	str	r3, [r2, #1220]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
.L3139:
	movs	r3, #0
	str	r3, [r4, #1932]
	b	.L3151
.L3137:
	ldr	r3, .L3166+8
	ldrh	r2, [r3, #3288]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L3151
	b	.L3140
.L3144:
	ldr	r3, [r9, #4]
	cmp	r0, r3
	bne	.L3162
.L3143:
	adds	r7, r7, #1
.L3141:
	ldrh	r3, [r4, #1688]
	uxth	r8, r7
	cmp	r3, r8
	bls	.L3165
	mul	r8, r10, r8
	ldr	fp, [r5, #1684]
	ldr	r3, [r6, #2964]
	add	r9, fp, r8
	ldr	r0, [r9, #8]
	cmp	r0, r3
	bcs	.L3162
	add	r1, sp, #4
	movs	r2, #0
	bl	log2phys
	ldr	r3, [fp, r8]
	ldr	r0, [sp, #4]
	cmp	r0, r3
	bne	.L3144
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	add	r1, r9, #4
	movs	r2, #1
	mov	r8, r0
	ldr	r0, [r9, #8]
	bl	log2phys
	mov	r0, r8
	b	.L3163
.L3162:
	ldrh	r0, [r5, #928]
.L3163:
	bl	decrement_vpc_count
	b	.L3143
.L3165:
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrb	r3, [r6, #1888]	@ zero_extendqisi2
	cbz	r3, .L3146
	ldr	r3, .L3166
	ldr	r0, .L3166+12
	ldrh	r1, [r3, #928]
	bl	printk
.L3146:
	ldrh	r0, [r4, #928]
	ldr	r3, [r4, #812]
	ldrh	r3, [r3, r0, lsl #1]
	cbz	r3, .L3147
	bl	INSERT_DATA_LIST
	b	.L3148
.L3147:
	bl	INSERT_FREE_LIST
.L3148:
	movw	r8, #65535
	movs	r5, #0
	strh	r8, [r4, #928]	@ movhi
	strh	r5, [r4, #1688]	@ movhi
	strh	r5, [r4, #1680]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	ldr	r3, [r6, #2832]
	ldr	r7, .L3166
	strh	r8, [r4, #1072]	@ movhi
	cbz	r3, .L3149
	ldr	r3, [r7, #1048]
	cmp	r3, #39
	bhi	.L3149
	ldrh	r3, [r7, #1052]
	ldrh	r2, [r7, #828]
	cmp	r2, r3
	bcs	.L3164
	lsls	r3, r3, #1
	strh	r3, [r7, #1636]	@ movhi
	b	.L3164
.L3149:
	ldrh	r3, [r4, #1052]
	ldrh	r1, [r4, #828]
	ldr	r2, .L3166
	add	r0, r3, r3, lsl #1
	cmp	r1, r0, asr #2
	ble	.L3164
	ldrb	r0, [r6, #1888]	@ zero_extendqisi2
	cbz	r0, .L3150
	subs	r3, r3, #2
	strh	r3, [r2, #1636]	@ movhi
.L3164:
	movs	r0, #0
	b	.L3130
.L3150:
	movs	r3, #20
	strh	r3, [r2, #1636]	@ movhi
	b	.L3130
.L3151:
	movs	r0, #1
.L3130:
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3167:
	.align	2
.L3166:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LC150
	.fnend
	.size	FtlGcFreeTempBlock, .-FtlGcFreeTempBlock
	.align	1
	.global	FtlGcPageRecovery
	.thumb
	.thumb_func
	.type	FtlGcPageRecovery, %function
FtlGcPageRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r5, .L3170
	ldr	r4, .L3170+4
	ldrh	r1, [r5, #2908]
	add	r0, r4, #928
	bl	FtlGcScanTempBlk
	ldrh	r2, [r4, #930]
	ldrh	r3, [r5, #2908]
	cmp	r2, r3
	bcc	.L3168
	add	r0, r4, #1872
	bl	FtlMapBlkWriteDumpData
	movs	r0, #0
	bl	FtlGcFreeTempBlock
	movs	r3, #0
	str	r3, [r4, #1932]
.L3168:
	pop	{r3, r4, r5, pc}
.L3171:
	.align	2
.L3170:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlGcPageRecovery, .-FtlGcPageRecovery
	.align	1
	.global	FtlPowerLostRecovery
	.thumb
	.thumb_func
	.type	FtlPowerLostRecovery, %function
FtlPowerLostRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movs	r5, #0
	ldr	r4, .L3173
	ldr	r3, .L3173+4
	add	r6, r4, #832
	add	r4, r4, #880
	mov	r0, r6
	str	r5, [r3, #3652]
	bl	FtlRecoverySuperblock
	mov	r0, r6
	bl	FtlSlcSuperblockCheck
	mov	r0, r4
	bl	FtlRecoverySuperblock
	mov	r0, r4
	bl	FtlSlcSuperblockCheck
	bl	FtlGcPageRecovery
	movw	r0, #65535
	bl	decrement_vpc_count
	mov	r0, r5
	pop	{r4, r5, r6, pc}
.L3174:
	.align	2
.L3173:
	.word	.LANCHOR2
	.word	.LANCHOR5
	.fnend
	.size	FtlPowerLostRecovery, .-FtlPowerLostRecovery
	.align	1
	.global	FtlSysBlkInit
	.thumb
	.thumb_func
	.type	FtlSysBlkInit, %function
FtlSysBlkInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	movs	r3, #0
	ldr	r5, .L3192
	movw	r7, #65535
	ldr	r6, .L3192+4
	ldr	r4, .L3192+8
	ldrh	r0, [r5, #2844]
	strh	r3, [r6, #3650]	@ movhi
	strh	r7, [r6, #3648]	@ movhi
	bl	FtlFreeSysBlkQueueInit
	bl	FtlScanSysBlk
	ldrh	r3, [r4, #1056]
	cmp	r3, r7
	bne	.L3176
.L3178:
	mov	r7, #-1
	b	.L3177
.L3176:
	bl	FtlLoadSysInfo
	mov	r7, r0
	cmp	r0, #0
	bne	.L3178
	bl	FtlLoadMapInfo
	bl	FtlLoadVonderInfo
	bl	Ftl_load_ext_data
	bl	FtlLoadEctTbl
	bl	FtlFreeSysBLkSort
	bl	SupperBlkListInit
	bl	FtlPowerLostRecovery
	movs	r0, #1
	bl	FtlUpdateVaildLpn
	ldrh	r1, [r5, #2946]
	ldr	r2, [r4, #976]
	movs	r0, #12
	mov	r3, r7
.L3179:
	cmp	r3, r1
	bge	.L3184
	mla	lr, r0, r3, r2
	ldr	lr, [lr, #4]
	cmp	lr, #0
	bge	.L3180
.L3184:
	ldrh	r2, [r4, #784]
	cmp	r3, r1
	add	r2, r2, #1
	strh	r2, [r4, #784]	@ movhi
	bge	.L3191
	b	.L3181
.L3180:
	adds	r3, r3, #1
	b	.L3179
.L3191:
	ldrh	r3, [r6, #3650]
	cbz	r3, .L3185
.L3181:
	ldrh	r1, [r4, #832]
	ldr	r2, [r4, #812]
	ldrh	r0, [r4, #836]
	ldrh	r3, [r2, r1, lsl #1]
	subs	r3, r3, r0
	strh	r3, [r2, r1, lsl #1]	@ movhi
	ldr	r1, [r4, #812]
	ldrh	r3, [r5, #2908]
	ldrh	r0, [r4, #880]
	ldrh	r6, [r4, #884]
	strh	r3, [r4, #834]	@ movhi
	movs	r3, #0
	strb	r3, [r4, #838]
	strh	r3, [r4, #836]	@ movhi
	ldrh	r2, [r1, r0, lsl #1]
	subs	r2, r2, r6
	strh	r2, [r1, r0, lsl #1]	@ movhi
	ldrh	r2, [r5, #2908]
	strb	r3, [r4, #886]
	strh	r3, [r4, #884]	@ movhi
	ldrh	r3, [r4, #786]
	strh	r2, [r4, #882]	@ movhi
	adds	r3, r3, #1
	strh	r3, [r4, #786]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	bl	FtlVpcTblFlush
.L3185:
	ldrh	r0, [r4, #832]
	movw	r3, #65535
	ldr	r6, .L3192+8
	cmp	r0, r3
	beq	.L3186
	ldrh	r3, [r6, #836]
	cbnz	r3, .L3186
	ldrh	r3, [r6, #884]
	cbnz	r3, .L3186
	bl	FtlGcRefreshOpenBlock
	ldrh	r0, [r6, #880]
	bl	FtlGcRefreshOpenBlock
	bl	FtlVpcTblFlush
	add	r0, r6, #832
	bl	allocate_new_data_superblock
	add	r0, r6, #880
	bl	allocate_new_data_superblock
.L3186:
	ldrb	r3, [r5]	@ zero_extendqisi2
	cbnz	r3, .L3187
	ldrh	r3, [r4, #784]
	lsls	r3, r3, #27
	bne	.L3177
.L3187:
	bl	FtlVpcCheckAndModify
.L3177:
	mov	r0, r7
	pop	{r3, r4, r5, r6, r7, pc}
.L3193:
	.align	2
.L3192:
	.word	.LANCHOR0
	.word	.LANCHOR5
	.word	.LANCHOR2
	.fnend
	.size	FtlSysBlkInit, .-FtlSysBlkInit
	.align	1
	.global	FtlLowFormat
	.thumb
	.thumb_func
	.type	FtlLowFormat, %function
FtlLowFormat:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	ldr	r4, .L3228
	ldr	r6, [r4, #740]
	cmp	r6, #0
	bne	.L3196
	ldr	r5, .L3228+4
	mov	r1, r6
	ldr	r0, [r4, #1856]
	ldrh	r2, [r5, #2944]
	lsls	r2, r2, #2
	bl	ftl_memset
	ldrh	r2, [r5, #2944]
	mov	r1, r6
	ldr	r0, [r4, #1852]
	lsls	r2, r2, #2
	bl	ftl_memset
	ldrh	r0, [r5, #2844]
	str	r6, [r4, #1024]
	str	r6, [r4, #1028]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cbz	r0, .L3197
	bl	FtlMakeBbt
.L3197:
	ldr	r0, .L3228+8
	movs	r2, #0
.L3198:
	ldrh	r1, [r5, #2914]
	uxth	r3, r2
	adds	r2, r2, #1
	cmp	r3, r1, lsl #7
	bge	.L3225
	ldr	r6, [r4, #1796]
	mvns	r1, r3
	orr	r1, r3, r1, lsl #16
	str	r1, [r6, r3, lsl #2]
	ldr	r1, [r4, #1800]
	str	r0, [r1, r3, lsl #2]
	b	.L3198
.L3225:
	ldr	r3, .L3228+4
	movs	r6, #0
	ldrh	r7, [r3, #2848]
.L3200:
	ldrh	r3, [r5, #2850]
	ldr	r8, .L3228+4
	cmp	r3, r7
	bls	.L3226
	mov	r0, r7
	movs	r1, #1
	bl	FtlLowFormatEraseBlock
	adds	r7, r7, #1
	uxth	r7, r7
	add	r0, r0, r6
	uxth	r6, r0
	b	.L3200
.L3226:
	ldrh	r1, [r8, #2840]
	subs	r3, r6, #3
	cmp	r3, r1, lsl #1
	bge	.L3202
.L3206:
	movs	r6, #0
	mov	r7, r6
	b	.L3203
.L3202:
	mov	r0, r6
	bl	__aeabi_uidiv
	ldr	r3, [r8, #2940]
	add	r0, r0, r3
	uxth	r0, r0
	bl	FtlSysBlkNumInit
	ldrh	r0, [r8, #2844]
	bl	FtlFreeSysBlkQueueInit
	ldrh	r6, [r8, #2848]
.L3204:
	ldrh	r3, [r5, #2850]
	cmp	r3, r6
	bls	.L3206
	mov	r0, r6
	adds	r6, r6, #1
	movs	r1, #1
	bl	FtlLowFormatEraseBlock
	uxth	r6, r6
	b	.L3204
.L3203:
	ldrh	r3, [r5, #2848]
	uxth	r0, r6
	ldr	r10, .L3228+4
	adds	r6, r6, #1
	cmp	r3, r0
	bls	.L3227
	movs	r1, #0
	bl	FtlLowFormatEraseBlock
	add	r0, r0, r7
	uxth	r7, r0
	b	.L3203
.L3227:
	ldrh	r3, [r10, #2850]
	ldrh	r6, [r10, #2840]
	ldr	fp, [r10, #2852]
	str	r3, [r4, #1756]
	mov	r1, r6
	mov	r0, fp
	bl	__aeabi_uidiv
	ubfx	r8, r0, #5, #16
	mov	r9, r0
	add	r3, r8, #36
	strh	r3, [r4, #1052]	@ movhi
	movs	r3, #24
	str	r0, [r10, #2964]
	muls	r3, r6, r3
	cmp	r7, r3
	ble	.L3208
	rsb	r0, r7, fp
	mov	r1, r6
	bl	__aeabi_uidiv
	ldr	r3, .L3228
	str	r0, [r10, #2964]
	lsrs	r0, r0, #5
	adds	r0, r0, #24
	strh	r0, [r3, #1052]	@ movhi
.L3208:
	ldr	r3, [r5, #2832]
	cmp	r3, #1
	bne	.L3209
	mov	r0, r7
	mov	r1, r6
	bl	__aeabi_uidiv
	ldrh	r10, [r4, #1052]
	uxtah	r0, r10, r0
	add	r10, r10, r0, asr #2
	strh	r10, [r4, #1052]	@ movhi
.L3209:
	ldrb	r3, [r5, #1888]	@ zero_extendqisi2
	cbz	r3, .L3210
	mov	r0, r7
	mov	r1, r6
	bl	__aeabi_uidiv
	ldrh	r10, [r4, #1052]
	uxtah	r0, r10, r0
	add	r10, r10, r0, asr #2
	strh	r10, [r4, #1052]	@ movhi
.L3210:
	ldrh	r3, [r5, #2902]
	cbz	r3, .L3212
	ldrh	r2, [r4, #1052]
	add	r2, r2, r3, lsr #1
	strh	r2, [r4, #1052]	@ movhi
	mul	r2, r6, r3
	cmp	r2, r7
	itttt	gt
	addgt	r3, r3, #32
	ldrgt	r2, .L3228+4
	addgt	r3, r3, r8
	strgt	r9, [r2, #2964]
	itt	gt
	ldrgt	r2, .L3228
	strhgt	r3, [r2, #1052]	@ movhi
.L3212:
	ldrh	r2, [r4, #1052]
	ldr	r3, [r5, #2964]
	ldr	r7, .L3228
	subs	r3, r3, r2
	muls	r6, r3, r6
	ldrh	r3, [r5, #2908]
	str	r6, [r4, #4092]
	muls	r6, r3, r6
	ldrh	r3, [r5, #2914]
	str	r6, [r5, #2964]
	muls	r6, r3, r6
	str	r6, [r5, #1892]
	bl	FtlBbmTblFlush
	ldrh	r2, [r5, #2850]
	movs	r1, #0
	ldr	r0, [r4, #812]
	movw	r6, #65535
	lsls	r2, r2, #1
	bl	ftl_memset
	ldrh	r2, [r5, #2848]
	ldr	r0, [r4, #988]
	movs	r1, #255
	movs	r3, #0
	strh	r6, [r4, #1072]	@ movhi
	lsrs	r2, r2, #3
	str	r3, [r4, #984]
	strh	r3, [r4, #1074]	@ movhi
	strb	r3, [r4, #1078]
	strb	r3, [r4, #1080]
	strh	r3, [r4, #834]	@ movhi
	strb	r3, [r4, #838]
	strh	r3, [r4, #832]	@ movhi
	movs	r3, #1
	strb	r3, [r4, #840]
	bl	ftl_memset
.L3214:
	ldr	r5, .L3228
	add	r0, r5, #832
	bl	make_superblock
	ldrb	r3, [r4, #839]	@ zero_extendqisi2
	ldrh	r2, [r4, #832]
	cbnz	r3, .L3215
	ldr	r3, [r7, #812]
	strh	r6, [r3, r2, lsl #1]	@ movhi
	ldrh	r3, [r7, #832]
	adds	r3, r3, #1
	strh	r3, [r7, #832]	@ movhi
	b	.L3214
.L3215:
	ldr	r3, [r5, #1024]
	movw	r7, #65535
	ldrh	r1, [r5, #836]
	str	r3, [r5, #844]
	adds	r3, r3, #1
	str	r3, [r5, #1024]
	ldr	r3, [r5, #812]
	strh	r1, [r3, r2, lsl #1]	@ movhi
	movs	r3, #0
	strh	r3, [r5, #882]	@ movhi
	strb	r3, [r5, #886]
	ldrh	r3, [r5, #832]
	adds	r3, r3, #1
	strh	r3, [r5, #880]	@ movhi
	movs	r3, #1
	strb	r3, [r5, #888]
.L3216:
	ldr	r6, .L3228
	add	r0, r6, #880
	bl	make_superblock
	ldrb	r3, [r4, #887]	@ zero_extendqisi2
	ldrh	r2, [r4, #880]
	cbnz	r3, .L3217
	ldr	r3, [r5, #812]
	strh	r7, [r3, r2, lsl #1]	@ movhi
	ldrh	r3, [r5, #880]
	adds	r3, r3, #1
	strh	r3, [r5, #880]	@ movhi
	b	.L3216
.L3217:
	ldr	r3, [r6, #1024]
	movw	r4, #65535
	ldrh	r1, [r6, #884]
	str	r3, [r6, #892]
	adds	r3, r3, #1
	str	r3, [r6, #1024]
	ldr	r3, [r6, #812]
	strh	r1, [r3, r2, lsl #1]	@ movhi
	strh	r4, [r6, #928]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	movs	r3, #0
	strh	r3, [r6, #1058]	@ movhi
	ldr	r3, [r6, #4092]
	strh	r4, [r6, #1060]	@ movhi
	strh	r3, [r6, #1062]	@ movhi
	ldr	r3, [r6, #1024]
	str	r3, [r6, #1064]
	adds	r3, r3, #1
	str	r3, [r6, #1024]
	strh	r0, [r6, #1056]	@ movhi
	bl	FtlVpcTblFlush
	bl	FtlSysBlkInit
	cbnz	r0, .L3196
	ldr	r3, .L3228+12
	movs	r2, #1
	str	r2, [r3, #504]
.L3196:
	movs	r0, #0
	pop	{r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3229:
	.align	2
.L3228:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	168778952
	.word	.LANCHOR1
	.fnend
	.size	FtlLowFormat, .-FtlLowFormat
	.align	1
	.global	FtlReInitForSDUpdata
	.thumb
	.thumb_func
	.type	FtlReInitForSDUpdata, %function
FtlReInitForSDUpdata:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r3, r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	.pad #16
	ldr	r4, .L3258
	ldrb	r3, [r4, #1888]	@ zero_extendqisi2
	cbz	r3, .L3231
.L3233:
	movs	r0, #0
	b	.L3232
.L3231:
	ldr	r5, .L3258+4
	ldr	r0, [r5, #3988]
	bl	FlashInit
	cmp	r0, #0
	bne	.L3233
	bl	FlashLoadFactorBbt
	cbz	r0, .L3234
	bl	FlashMakeFactorBbt
.L3234:
	ldr	r0, [r5, #4020]
	bl	FlashReadIdbDataRaw
	cbz	r0, .L3235
	movs	r1, #0
	movs	r2, #16
	mov	r0, sp
	movs	r6, #1
	bl	FlashReadFacBbtData
	movs	r3, #0
	mov	r2, r3
	ldr	r1, [sp]
.L3236:
	lsl	r5, r6, r2
	adds	r0, r3, #1
	tst	r5, r1
	add	r2, r2, #1
	it	ne
	movne	r3, r0
	cmp	r2, #16
	bne	.L3236
	cmp	r3, #6
	bls	.L3254
	movs	r2, #0
	movs	r6, #1
.L3239:
	lsl	r5, r6, r2
	adds	r0, r3, #1
	tst	r5, r1
	add	r2, r2, #1
	it	ne
	movne	r3, r0
	cmp	r2, #24
	bne	.L3239
	cmp	r3, #17
	bhi	.L3240
.L3254:
	strb	r2, [r4, #1]
	b	.L3238
.L3240:
	movs	r3, #36
	strb	r3, [r4, #1]
.L3238:
	ldrb	r3, [r4, #1]	@ zero_extendqisi2
	strh	r3, [r4, #1886]	@ movhi
.L3235:
	ldr	r1, .L3258+8
	ldr	r0, .L3258+12
	bl	printk
	ldr	r0, .L3258+16
	bl	FtlConstantsInit
	bl	FtlVariablesInit
	ldrh	r0, [r4, #2844]
	bl	FtlFreeSysBlkQueueInit
	movs	r4, #1
.L3241:
	bl	FtlLoadBbt
	cbz	r0, .L3242
.L3256:
	bl	FtlLowFormat
	cmp	r4, #3
	bhi	.L3257
	adds	r4, r4, #1
	b	.L3241
.L3257:
	mov	r0, #-1
	b	.L3232
.L3242:
	bl	FtlSysBlkInit
	cmp	r0, #0
	bne	.L3256
	ldr	r3, .L3258+20
	movs	r2, #1
	str	r2, [r3, #504]
.L3232:
	add	sp, sp, #16
	@ sp needed
	pop	{r4, r5, r6, pc}
.L3259:
	.align	2
.L3258:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC77
	.word	.LC76
	.word	.LANCHOR0+1860
	.word	.LANCHOR1
	.fnend
	.size	FtlReInitForSDUpdata, .-FtlReInitForSDUpdata
	.align	1
	.global	Ftl_gc_temp_data_write_back
	.thumb
	.thumb_func
	.type	Ftl_gc_temp_data_write_back, %function
Ftl_gc_temp_data_write_back:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	ldr	r5, .L3276
	ldr	r3, [r5, #740]
	cbz	r3, .L3261
.L3264:
	movs	r0, #0
	pop	{r4, r5, r6, r7, r8, pc}
.L3261:
	ldr	r3, .L3276+4
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cbz	r3, .L3263
	ldr	r3, [r5, #1652]
	lsls	r3, r3, #31
	bpl	.L3263
	ldrh	r3, [r5, #932]
	cmp	r3, #0
	bne	.L3264
.L3263:
	movs	r2, #0
	ldr	r0, [r5, #1772]
	ldr	r1, [r5, #1652]
	movs	r6, #0
	mov	r3, r2
	ldr	r7, .L3276
	bl	FlashProgPages
	mov	r8, #36
.L3265:
	ldr	r1, [r5, #1652]
	uxth	r3, r6
	ldr	r4, .L3276
	cmp	r3, r1
	bcs	.L3275
	mul	r3, r8, r3
	ldr	r0, [r7, #1772]
	adds	r6, r6, #1
	adds	r1, r0, r3
	ldr	r0, [r0, r3]
	ldr	r2, [r1, #12]
	cmp	r0, #-1
	bne	.L3266
	ldrh	r1, [r4, #928]
	movs	r5, #0
	ldr	r2, [r4, #812]
	strh	r5, [r2, r1, lsl #1]	@ movhi
	ldr	r2, [r4, #1220]
	strh	r0, [r4, #928]	@ movhi
	adds	r2, r2, #1
	str	r2, [r4, #1220]
	ldr	r2, [r4, #1772]
	add	r3, r3, r2
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	bl	FtlGcPageVarInit
	b	.L3274
.L3266:
	ldr	r0, [r2, #12]
	ldr	r1, [r1, #4]
	ldr	r2, [r2, #8]
	bl	FtlGcUpdatePage
	b	.L3265
.L3275:
	ldr	r0, [r4, #1772]
	bl	FtlGcBufFree
	movs	r3, #0
	str	r3, [r4, #1652]
	ldrh	r3, [r4, #932]
	cmp	r3, #0
	bne	.L3264
	movs	r0, #1
	bl	FtlGcFreeTempBlock
.L3274:
	movs	r0, #1
	pop	{r4, r5, r6, r7, r8, pc}
.L3277:
	.align	2
.L3276:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	Ftl_gc_temp_data_write_back, .-Ftl_gc_temp_data_write_back
	.align	1
	.global	Ftl_get_new_temp_ppa
	.thumb
	.thumb_func
	.type	Ftl_get_new_temp_ppa, %function
Ftl_get_new_temp_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movw	r3, #65535
	ldr	r4, .L3281
	ldrh	r2, [r4, #928]
	cmp	r2, r3
	beq	.L3279
	ldrh	r3, [r4, #932]
	cbnz	r3, .L3280
.L3279:
	bl	FtlCacheWriteBack
	movs	r0, #0
	movs	r5, #0
	bl	FtlGcFreeTempBlock
	ldr	r0, .L3281+4
	strb	r5, [r4, #936]
	bl	allocate_data_superblock
	strh	r5, [r4, #1680]	@ movhi
	strh	r5, [r4, #1688]	@ movhi
	bl	l2p_flush
	mov	r0, r5
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L3280:
	ldr	r0, .L3281+4
	pop	{r3, r4, r5, lr}
	b	get_new_active_ppa
.L3282:
	.align	2
.L3281:
	.word	.LANCHOR2
	.word	.LANCHOR2+928
	.fnend
	.size	Ftl_get_new_temp_ppa, .-Ftl_get_new_temp_ppa
	.align	1
	.global	ftl_do_gc
	.thumb
	.thumb_func
	.type	ftl_do_gc, %function
ftl_do_gc:
	.fnstart
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L3456
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r8, r0
	ldr	r0, [r3, #740]
	.pad #44
	sub	sp, sp, #44
	mov	r6, r1
	mov	r7, r3
	cmp	r0, #0
	bne	.L3383
	ldr	r2, .L3456+4
	ldr	r1, [r2, #504]
	adds	r1, r1, #1
	beq	.L3443
	ldr	r4, [r3, #1744]
	cmp	r4, #0
	bne	.L3443
	ldrh	r1, [r3, #820]
	cmp	r1, #47
	bls	.L3386
	ldrh	r1, [r2, #3288]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L3285
.L3288:
	ldrh	r4, [r7, #1692]
	movw	r1, #65535
	ldr	r3, .L3456
	cmp	r4, r1
	bne	.L3286
	b	.L3287
.L3285:
	ldrh	r3, [r3, #928]
	cmp	r3, r2
	beq	.L3288
	movs	r0, #1
	bl	FtlGcFreeTempBlock
	cmp	r0, #0
	beq	.L3288
	movs	r0, #1
	b	.L3443
.L3286:
	ldrh	r2, [r3, #1690]
	cmp	r2, r1
	bne	.L3287
	ldrh	r0, [r3, #1694]
	cmp	r0, r2
	beq	.L3287
	ldrh	r1, [r3, #1696]
	cmp	r1, r2
	itttt	ne
	strhne	r4, [r3, #1690]	@ movhi
	strhne	r0, [r3, #1692]	@ movhi
	strhne	r1, [r3, #1694]	@ movhi
	strhne	r2, [r3, #1696]	@ movhi
.L3287:
	ldr	r4, [r7, #1640]
	cmp	r8, #1
	add	r4, r4, #1
	add	r4, r4, r8, lsl #7
	str	r4, [r7, #1640]
	bne	.L3299
	ldr	r3, .L3456+8
	ldr	r2, [r3, #2832]
	mov	r10, r3
	cbnz	r2, .L3290
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3299
.L3290:
	ldr	r3, [r7, #1048]
	ldr	r5, .L3456
	cmp	r3, #39
	bhi	.L3299
	ldr	r9, .L3456+12
	ldrh	r3, [r9, #3784]
	add	r4, r4, r3
	str	r4, [r5, #1640]
	bl	FtlGcReFreshBadBlk
	ldrh	r3, [r5, #1072]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L3299
	ldrh	r2, [r5, #1690]
	cmp	r2, r3
	bne	.L3299
	ldr	r3, [r5, #1640]
	cmp	r3, #1024
	bhi	.L3291
	ldrh	r3, [r5, #828]
	cmp	r3, #63
	bhi	.L3299
.L3291:
	ldrh	r3, [r7, #1638]
	movs	r0, #0
	ldrh	r4, [r7, #828]
	adds	r3, r3, #64
	ldr	r2, .L3456+12
	cmp	r4, r3
	strh	r0, [r9, #3784]	@ movhi
	ldr	r1, .L3456
	bgt	.L3299
	ldr	r3, [r1, #1048]
	str	r0, [r1, #1640]
	cbnz	r3, .L3292
	movs	r3, #6
	b	.L3444
.L3292:
	cmp	r3, #5
	bhi	.L3293
	movs	r3, #18
.L3444:
	strh	r3, [r2, #3784]	@ movhi
.L3293:
	movs	r0, #32
	bl	List_get_gc_head_node
	movw	ip, #65535
	uxth	r2, r0
	cmp	r2, ip
	beq	.L3298
	ldrh	r0, [r7, #1644]
	ldr	r5, .L3456
	cmp	r0, #0
	beq	.L3295
	ldrh	r4, [r10, #2910]
	ldrh	r3, [r10, #2840]
	ldr	fp, [r5, #812]
	muls	r3, r4, r3
	ldrh	r1, [fp, r2, lsl #1]
	adds	r3, r3, #1
	cmp	r1, r3
	bgt	.L3298
	adds	r1, r0, #1
	str	ip, [sp, #20]
	str	r2, [sp, #16]
	mov	r10, #0
	uxth	r1, r1
	str	r10, [r5, #1648]
	strh	r1, [r5, #1644]	@ movhi
	str	r1, [sp, #12]
	bl	List_get_gc_head_node
	ldr	ip, [sp, #20]
	uxth	r4, r0
	ldr	r1, [sp, #12]
	cmp	r4, ip
	ldr	r2, [sp, #16]
	beq	.L3298
	ldrh	r2, [fp, r2, lsl #1]
	ldrh	r3, [fp, r4, lsl #1]
	ldr	r0, .L3456+16
	str	r2, [sp]
	mov	r2, r4
	bl	printk
	ldrh	r3, [r5, #1644]
	cmp	r3, #40
	bls	.L3296
	ldr	r3, [r5, #812]
	ldrh	r3, [r3, r4, lsl #1]
	cmp	r3, #32
	it	hi
	strhhi	r10, [r5, #1644]	@ movhi
.L3296:
	movs	r3, #6
	strh	r3, [r9, #3784]	@ movhi
	b	.L3315
.L3295:
	movs	r3, #1
	strh	r3, [r5, #1644]	@ movhi
.L3298:
	bl	GetSwlReplaceBlock
	movw	r3, #65535
	cmp	r0, r3
	mov	r4, r0
	bne	.L3315
	movs	r3, #0
	strh	r3, [r9, #3784]	@ movhi
.L3299:
	ldrh	r3, [r7, #1072]
	movw	r4, #65535
	ldr	r5, .L3456
	cmp	r3, r4
	bne	.L3449
	ldrh	r4, [r5, #928]
	cmp	r4, r3
	bne	.L3300
	ldrh	r10, [r5, #1690]
	cmp	r10, r4
	bne	.L3301
	ldrh	r3, [r5, #828]
	ldr	r2, [r5, #1640]
	cmp	r3, #24
	ite	cc
	movcc	r3, #5120
	movcs	r3, #1024
	cmp	r2, r3
	bls	.L3303
	ldr	r2, .L3456+12
	movs	r3, #0
	str	r3, [r7, #1640]
	mov	r9, r5
	strh	r3, [r2, #3784]	@ movhi
	bl	GetSwlReplaceBlock
	cmp	r0, r10
	mov	r4, r0
	bne	.L3304
	ldrh	r2, [r5, #828]
	ldrh	r3, [r5, #1638]
	cmp	r2, r3
	bcs	.L3305
	movs	r0, #64
	bl	List_get_gc_head_node
	uxth	r3, r0
	cmp	r3, r4
	bne	.L3450
.L3314:
	bl	FtlGcReFreshBadBlk
	cmp	r8, #0
	beq	.L3451
.L3315:
	ldrh	r2, [r7, #928]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L3326
.L3381:
	ldrh	r3, [r7, #1690]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L3326
	cmp	r4, r3
	bne	.L3326
.L3382:
	ldrh	r1, [r7, #1072]
	movw	r3, #65535
	ldr	r2, .L3456
	cmp	r1, r3
	beq	.L3327
.L3331:
	movw	r4, #65535
	b	.L3326
.L3327:
	movs	r3, #0
	ldrh	r1, [r2, #828]
	str	r3, [r2, #1648]
	ldrh	r3, [r2, #1636]
	ldr	r4, .L3456+12
	cmp	r1, r3
	bls	.L3328
	ldrh	r3, [r4, #3784]
	cbnz	r3, .L3329
	ldr	r3, .L3456+8
	ldr	r2, [r2, #984]
	ldr	r3, [r3, #2964]
	add	r3, r3, r3, lsl #1
	cmp	r2, r3, lsr #2
	bcs	.L3394
.L3329:
	ldrh	r3, [r7, #1052]
	add	r3, r3, r3, lsl #1
	ubfx	r3, r3, #2, #16
	b	.L3330
.L3394:
	movs	r3, #18
.L3330:
	strh	r3, [r7, #1636]	@ movhi
	bl	FtlReadRefresh
	movs	r0, #0
	bl	List_get_gc_head_node
	ldr	r3, [r7, #812]
	uxth	r0, r0
	ldrh	r3, [r3, r0, lsl #1]
	cmp	r3, #4
	bls	.L3328
	ldrh	r0, [r4, #3784]
	b	.L3443
.L3328:
	ldrh	r0, [r4, #3784]
	cmp	r0, #0
	bne	.L3331
	ldrh	r4, [r7, #1052]
	add	r3, r4, r4, lsl #1
	asrs	r3, r3, #2
	strh	r3, [r7, #1636]	@ movhi
	bl	List_get_gc_head_node
	ldr	r3, [r7, #812]
	uxth	r0, r0
	ldrh	r2, [r3, r0, lsl #1]
	ldr	r3, .L3456+8
	ldrh	r1, [r3, #2910]
	ldrh	r3, [r3, #2840]
	muls	r3, r1, r3
	add	r3, r3, r3, lsr #31
	cmp	r2, r3, asr #1
	ble	.L3332
	ldr	r3, .L3456
	subs	r4, r4, #1
	ldrh	r3, [r3, #828]
	cmp	r3, r4
	blt	.L3332
	bl	FtlReadRefresh
	b	.L3448
.L3332:
	cmp	r2, #0
	bne	.L3331
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrh	r0, [r7, #828]
	adds	r0, r0, #1
	b	.L3443
.L3457:
	.align	2
.L3456:
	.word	.LANCHOR2
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR5
	.word	.LC151
.L3326:
	ldr	r3, .L3458
	ldr	r3, [r3, #2832]
	cmp	r3, #0
	ite	eq
	moveq	r5, #1
	movne	r5, #2
	b	.L3325
.L3451:
	movw	r3, #65535
	cmp	r4, r3
	bne	.L3315
.L3380:
	ldrh	r4, [r7, #828]
	cmp	r4, #24
	bhi	.L3390
	ldr	r3, .L3458
	cmp	r4, #16
	ldrh	r5, [r3, #2908]
	bls	.L3317
	lsrs	r5, r5, #5
.L3316:
	ldrh	r3, [r7, #1636]
	ldr	r2, .L3458+4
	cmp	r3, r4
	bcc	.L3452
.L3320:
	ldr	r3, .L3458
	ldr	r3, [r3, #2832]
	cmp	r3, #0
	bne	.L3453
.L3393:
	movw	r4, #65535
.L3325:
	ldrh	r2, [r7, #1072]
	movw	r1, #65535
	ldr	r3, .L3458+4
	cmp	r2, r1
	bne	.L3334
	cmp	r4, r2
	beq	.L3335
	strh	r4, [r3, #1072]	@ movhi
	b	.L3336
.L3335:
	ldrh	r2, [r3, #1690]
	cmp	r2, r4
	beq	.L3336
	ldr	r1, [r3, #812]
	ldrh	r2, [r1, r2, lsl #1]
	cbnz	r2, .L3337
	strh	r4, [r3, #1690]	@ movhi
.L3337:
	ldrh	r3, [r7, #1690]
	strh	r3, [r7, #1072]	@ movhi
	movw	r3, #65535
	strh	r3, [r7, #1690]	@ movhi
.L3336:
	ldrh	r0, [r7, #1072]
	movw	r6, #65535
	movs	r3, #0
	strb	r3, [r7, #1080]
	cmp	r0, r6
	beq	.L3334
	bl	IsBlkInGcList
	cbz	r0, .L3339
	ldr	r3, .L3458+4
	strh	r6, [r3, #1072]	@ movhi
.L3339:
	ldr	r3, .L3458
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cbz	r3, .L3340
	ldrh	r0, [r7, #1072]
	bl	ftl_get_blk_mode
	strb	r0, [r7, #1080]
.L3340:
	ldrh	r2, [r7, #1072]
	movw	r3, #65535
	ldr	r6, .L3458+4
	cmp	r2, r3
	beq	.L3334
	add	r0, r6, #1072
	bl	make_superblock
	ldr	r2, .L3458+8
	movs	r3, #0
	ldrh	r1, [r6, #1072]
	strh	r3, [r6, #1074]	@ movhi
	strb	r3, [r6, #1078]
	strh	r3, [r2, #3786]	@ movhi
	ldr	r3, [r6, #812]
	ldrh	r3, [r3, r1, lsl #1]
	strh	r3, [r2, #3788]	@ movhi
.L3334:
	ldrh	r3, [r7, #1072]
	ldrh	r1, [r7, #832]
	ldr	r2, .L3458+4
	cmp	r1, r3
	beq	.L3341
	ldrh	r1, [r2, #880]
	cmp	r1, r3
	beq	.L3341
	ldrh	r2, [r2, #928]
	cmp	r2, r3
	bne	.L3342
.L3341:
	movw	r3, #65535
	strh	r3, [r7, #1072]	@ movhi
.L3342:
	ldr	r7, .L3458
.L3375:
	ldr	r6, .L3458+4
	movw	r3, #65535
	ldrh	r2, [r6, #1072]
	mov	fp, r6
	cmp	r2, r3
	bne	.L3343
	mov	r10, r6
	movs	r3, #0
	str	r3, [r6, #1648]
.L3344:
	ldrh	r9, [fp, #1644]
	mov	r0, r9
	bl	List_get_gc_head_node
	movw	r2, #65535
	uxth	r3, r0
	strh	r3, [fp, #1072]	@ movhi
	cmp	r3, r2
	bne	.L3345
	ldr	r3, .L3458+4
	movs	r2, #0
	movs	r0, #8
	strh	r2, [r3, #1644]	@ movhi
	b	.L3443
.L3345:
	mov	r0, r3
	str	r3, [sp, #12]
	bl	IsBlkInGcList
	add	r9, r9, #1
	ldr	r3, [sp, #12]
	cbz	r0, .L3346
	strh	r9, [r10, #1644]	@ movhi
	b	.L3344
.L3346:
	ldrh	lr, [r7, #2840]
	uxth	r9, r9
	ldrh	r2, [r7, #2908]
	ldr	r1, [r10, #812]
	strh	r9, [r10, #1644]	@ movhi
	mul	r2, lr, r2
	ldrh	r0, [r1, r3, lsl #1]
	add	lr, r2, r2, lsr #31
	cmp	r0, lr, asr #1
	bgt	.L3348
	cmp	r9, #48
	bls	.L3349
	cmp	r0, #8
	bls	.L3349
	ldrh	r0, [r10, #1680]
	cmp	r0, #35
	bhi	.L3349
.L3348:
	movs	r0, #0
	strh	r0, [fp, #1644]	@ movhi
.L3349:
	ldrh	r1, [r1, r3, lsl #1]
	cmp	r1, r2
	blt	.L3350
	movw	r2, #65535
	cmp	r4, r2
	bne	.L3350
	ldrh	r0, [fp, #1644]
	ldr	r2, .L3458+4
	cmp	r0, #3
	bhi	.L3350
	movs	r3, #0
	strh	r4, [r2, #1072]	@ movhi
	strh	r3, [r2, #1644]	@ movhi
.L3448:
	ldr	r3, .L3458+8
	ldrh	r0, [r3, #3784]
	b	.L3443
.L3350:
	cbnz	r1, .L3351
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrh	r3, [fp, #1644]
	adds	r3, r3, #1
	strh	r3, [fp, #1644]	@ movhi
	b	.L3344
.L3351:
	movs	r2, #0
	strb	r2, [r6, #1080]
	ldr	r2, .L3458
	ldrb	r2, [r2, #1888]	@ zero_extendqisi2
	cbz	r2, .L3352
	mov	r0, r3
	bl	ftl_get_blk_mode
	ldr	r3, .L3458+4
	strb	r0, [r3, #1080]
.L3352:
	ldr	r0, .L3458+12
	bl	make_superblock
	ldr	r2, .L3458+8
	ldrh	r0, [r6, #1072]
	movs	r3, #0
	ldr	r1, [r6, #812]
	strh	r3, [r2, #3786]	@ movhi
	ldrh	r1, [r1, r0, lsl #1]
	strh	r3, [r6, #1074]	@ movhi
	strb	r3, [r6, #1078]
	strh	r1, [r2, #3788]	@ movhi
.L3343:
	cmp	r8, #1
	bne	.L3353
	bl	FtlReadRefresh
.L3353:
	movs	r3, #1
	str	r3, [r6, #1744]
	ldrh	r3, [r7, #2908]
	str	r3, [sp, #12]
	ldrb	r3, [r7, #1888]	@ zero_extendqisi2
	cbz	r3, .L3354
	ldr	r3, .L3458+4
	ldrb	r3, [r3, #1080]	@ zero_extendqisi2
	cmp	r3, #1
	itt	eq
	ldrheq	r3, [r7, #2910]
	streq	r3, [sp, #12]
.L3354:
	ldrh	r3, [r6, #1074]
	ldr	r1, [sp, #12]
	adds	r2, r3, r5
	cmp	r2, r1
	ittt	gt
	movgt	r2, r1
	subgt	r3, r2, r3
	uxthgt	r5, r3
	movs	r3, #0
.L3447:
	str	r3, [sp, #16]
	ldrh	r3, [sp, #16]
	ldr	r6, .L3458+4
	cmp	r3, r5
	bcs	.L3363
	ldr	r3, [r6, #1668]
	addw	r1, r6, #1086
	ldr	r2, [sp, #16]
	movw	lr, #65535
	ldrh	r10, [r7, #2840]
	str	r3, [sp, #20]
	ldrh	r3, [r6, #1074]
	add	ip, r3, r2
	movs	r3, #0
	mov	r9, r3
.L3364:
	uxth	r2, r3
	cmp	r2, r10
	bcs	.L3454
	ldrh	r2, [r1, #2]!
	cmp	r2, lr
	beq	.L3357
	ldr	r0, [sp, #20]
	mov	fp, #36
	orr	r2, ip, r2, lsl #10
	mla	fp, fp, r9, r0
	add	r9, r9, #1
	uxth	r9, r9
	str	r2, [fp, #4]
.L3357:
	adds	r3, r3, #1
	b	.L3364
.L3459:
	.align	2
.L3458:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR5
	.word	.LANCHOR2+1072
.L3454:
	ldr	r0, [r6, #1668]
	mov	r1, r9
	ldrb	r2, [r6, #1080]	@ zero_extendqisi2
	mov	fp, #0
	bl	FlashReadPages
	ldr	r6, .L3460
.L3359:
	uxth	r3, fp
	cmp	r3, r9
	bcs	.L3455
	mov	ip, #36
	ldr	r2, [r6, #1668]
	mul	r10, ip, fp
	add	r3, r2, r10
	ldr	r2, [r2, r10]
	ldr	r3, [r3, #12]
	adds	r2, r2, #1
	beq	.L3397
	ldrh	r1, [r3]
	movw	r2, #61589
	cmp	r1, r2
	bne	.L3397
	ldr	r0, [r3, #8]
	add	r1, sp, #32
	movs	r2, #0
	str	ip, [sp, #24]
	str	r3, [sp, #20]
	bl	log2phys
	ldr	r1, [r6, #1668]
	add	r1, r1, r10
	ldr	r0, [r1, #4]
	ldr	r2, [sp, #32]
	ldr	r3, [sp, #20]
	bic	r2, r2, #-2147483648
	ldr	ip, [sp, #24]
	cmp	r2, r0
	bne	.L3397
	str	r3, [sp, #28]
	ldr	r3, .L3460+4
	ldr	r0, [r6, #1652]
	ldr	r1, [r1, #16]
	ldrh	r2, [r3, #3786]
	adds	r2, r2, #1
	strh	r2, [r3, #3786]	@ movhi
	ldr	r2, [r6, #1772]
	mla	r2, ip, r0, r2
	str	r1, [r2, #16]
	str	r2, [sp, #20]
	bl	Ftl_get_new_temp_ppa
	ldr	r1, [r6, #1652]
	ldr	r2, [sp, #20]
	ldr	ip, [sp, #24]
	ldr	r3, [sp, #28]
	str	r0, [r2, #4]
	ldr	r2, [r6, #1772]
	mla	ip, ip, r1, r2
	ldr	r2, [r6, #1668]
	add	r2, r2, r10
	ldr	r1, [r2, #8]
	str	r1, [ip, #8]
	movs	r1, #1
	ldr	r2, [r2, #12]
	str	r2, [ip, #12]
	ldr	r2, [sp, #32]
	str	r2, [r3, #12]
	ldrh	r2, [r6, #928]
	strh	r2, [r3, #2]	@ movhi
	ldr	r2, [r6, #1028]
	ldr	r0, [r6, #1668]
	str	r2, [r3, #4]
	add	r0, r0, r10
	ldr	r3, [r6, #1652]
	adds	r3, r3, #1
	str	r3, [r6, #1652]
	bl	FtlGcBufAlloc
	ldrb	r3, [r7, #1888]	@ zero_extendqisi2
	cbnz	r3, .L3361
	ldrb	r3, [r6, #935]	@ zero_extendqisi2
	ldr	r2, [r6, #1652]
	cmp	r2, r3
	beq	.L3361
	ldrh	r3, [r6, #932]
	cbnz	r3, .L3397
.L3361:
	bl	Ftl_gc_temp_data_write_back
	cbz	r0, .L3397
	ldr	r3, .L3460
	movs	r2, #0
	movw	r1, #65535
	str	r2, [r3, #1744]
	strh	r1, [r3, #1072]	@ movhi
	strh	r2, [r3, #1074]	@ movhi
	b	.L3448
.L3397:
	add	fp, fp, #1
	b	.L3359
.L3455:
	ldr	r3, [sp, #16]
	adds	r3, r3, #1
	b	.L3447
.L3363:
	ldrh	r3, [r6, #1074]
	add	r5, r5, r3
	ldr	r3, [sp, #12]
	uxth	r5, r5
	strh	r5, [r6, #1074]	@ movhi
	cmp	r5, r3
	bcc	.L3365
	ldr	r3, [r6, #1652]
	ldr	r5, .L3460+4
	cbz	r3, .L3366
	bl	Ftl_gc_temp_data_write_back
	cbz	r0, .L3366
	movs	r3, #0
	ldrh	r0, [r5, #3784]
	str	r3, [r6, #1744]
	b	.L3443
.L3366:
	ldrh	r5, [r5, #3786]
	cbnz	r5, .L3367
	ldrh	r2, [r6, #1072]
	ldr	r3, [r6, #812]
	ldrh	r3, [r3, r2, lsl #1]
	cbz	r3, .L3367
.L3368:
	ldr	r3, [r7, #2964]
	cmp	r5, r3
	bcs	.L3373
	mov	r0, r5
	add	r1, sp, #36
	movs	r2, #0
	bl	log2phys
	ldr	r0, [sp, #36]
	adds	r3, r0, #1
	beq	.L3369
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r6, #1072]
	cmp	r3, r0
	bne	.L3369
.L3373:
	ldr	r3, .L3460+8
	ldr	r3, [r3, #2964]
	cmp	r5, r3
	bcc	.L3367
	ldrh	r2, [r6, #1072]
	movs	r1, #0
	ldr	r3, [r6, #812]
	strh	r1, [r3, r2, lsl #1]	@ movhi
	ldrh	r0, [r6, #1072]
	bl	update_vpc_list
	bl	FtlCacheWriteBack
	bl	l2p_flush
	bl	FtlVpcTblFlush
	b	.L3367
.L3369:
	adds	r5, r5, #1
	b	.L3368
.L3367:
	movw	r3, #65535
	strh	r3, [r6, #1072]	@ movhi
.L3365:
	ldrh	r3, [r6, #828]
	cmp	r3, #2
	bhi	.L3374
	ldrh	r5, [r7, #2908]
	b	.L3375
.L3374:
	ldr	r2, .L3460
	movs	r1, #0
	str	r1, [r2, #1744]
	ldr	r2, .L3460+4
	ldrh	r0, [r2, #3784]
	cmp	r0, #0
	bne	.L3443
	adds	r0, r3, #1
	b	.L3443
.L3383:
	movs	r0, #0
	b	.L3443
.L3386:
	mov	r0, r4
	b	.L3443
.L3450:
	ldr	r2, [r5, #1740]
	ldr	r5, .L3460+8
	cbnz	r2, .L3307
	ldrh	r2, [r5, #2860]
	cmp	r2, #3
	beq	.L3307
	ldr	r2, [r9, #1120]
	cbnz	r2, .L3307
	ldr	r2, [r5, #2832]
	cbnz	r2, .L3307
	ldrb	r0, [r5, #1888]	@ zero_extendqisi2
	cbz	r0, .L3308
.L3307:
	ldr	r2, [r7, #812]
	ldrh	r0, [r5, #2860]
	ldrh	r1, [r2, r3, lsl #1]
	cmp	r0, #3
	ldrh	r3, [r5, #2910]
	ldrh	r2, [r5, #2840]
	mul	r2, r2, r3
	ite	eq
	lsreq	r3, r3, #1
	movne	r3, #0
	add	r3, r3, r2
	cmp	r1, r3
	bgt	.L3310
	movs	r0, #0
	bl	List_get_gc_head_node
	ldr	r3, [r5, #2964]
	ldr	r2, .L3460
	ldr	r1, [r7, #984]
	add	r3, r3, r3, lsl #1
	cmp	r1, r3, lsr #2
	ite	hi
	movhi	r3, #128
	movls	r3, #160
	strh	r3, [r2, #1638]	@ movhi
	uxth	r4, r0
	b	.L3312
.L3310:
	movs	r3, #128
	b	.L3446
.L3308:
	ldr	r2, [r9, #812]
	ldrh	r3, [r2, r3, lsl #1]
	cmp	r3, #7
	bhi	.L3313
	bl	List_get_gc_head_node
	movs	r3, #128
	strh	r3, [r7, #1638]	@ movhi
	uxth	r4, r0
	b	.L3312
.L3313:
	movs	r3, #64
.L3446:
	strh	r3, [r7, #1638]	@ movhi
	b	.L3314
.L3305:
	movs	r3, #80
	strh	r3, [r5, #1638]	@ movhi
	b	.L3314
.L3312:
	movw	r3, #65535
	cmp	r4, r3
	beq	.L3314
.L3304:
	ldr	r1, [r7, #748]
	ldr	r3, [r7, #812]
	ldrh	r2, [r7, #828]
	ldrh	r1, [r1, r4, lsl #1]
	ldrh	r3, [r3, r4, lsl #1]
	ldr	r0, .L3460+12
	str	r1, [sp]
	ldrh	r1, [r7, #1636]
	str	r1, [sp, #4]
	mov	r1, r4
	bl	printk
	b	.L3314
.L3317:
	cmp	r4, #12
	bls	.L3318
	lsrs	r5, r5, #4
	b	.L3316
.L3318:
	cmp	r4, #8
	bls	.L3316
	lsrs	r5, r5, #2
	b	.L3316
.L3390:
	movs	r5, #1
	b	.L3316
.L3452:
	ldrh	r3, [r2, #928]
	movw	r1, #65535
	cmp	r3, r1
	bne	.L3321
	ldrh	r1, [r2, #1690]
	cmp	r1, r3
	bne	.L3321
	ldr	r3, .L3460+4
	ldrh	r0, [r3, #3784]
	cbnz	r0, .L3322
	ldr	r3, .L3460+8
	ldr	r2, [r2, #984]
	ldr	r3, [r3, #2964]
	add	r3, r3, r3, lsl #1
	cmp	r2, r3, lsr #2
	bcs	.L3391
.L3322:
	ldrh	r3, [r7, #1052]
	add	r3, r3, r3, lsl #1
	ubfx	r3, r3, #2, #16
	b	.L3323
.L3461:
	.align	2
.L3460:
	.word	.LANCHOR2
	.word	.LANCHOR5
	.word	.LANCHOR0
	.word	.LC152
.L3391:
	movs	r3, #18
.L3323:
	strh	r3, [r7, #1636]	@ movhi
	movs	r3, #0
	str	r3, [r7, #1648]
	b	.L3443
.L3321:
	ldrh	r3, [r7, #1052]
	add	r3, r3, r3, lsl #1
	asrs	r3, r3, #2
	strh	r3, [r7, #1636]	@ movhi
	b	.L3320
.L3453:
	cmp	r6, #2
	bhi	.L3393
	adds	r5, r5, #1
	uxth	r5, r5
	b	.L3393
.L3449:
	cmp	r8, #0
	beq	.L3380
	b	.L3315
.L3301:
	cmp	r8, #0
	beq	.L3380
	b	.L3381
.L3300:
	cmp	r8, #0
	beq	.L3380
	b	.L3331
.L3303:
	cmp	r8, #0
	beq	.L3380
	b	.L3382
.L3443:
	add	sp, sp, #44
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
	.fnend
	.size	ftl_do_gc, .-ftl_do_gc
	.align	1
	.global	FtlCacheWriteBack
	.thumb
	.thumb_func
	.type	FtlCacheWriteBack, %function
FtlCacheWriteBack:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r3, .L3505
	ldr	r7, .L3505+4
	ldr	r5, [r3, #3792]
	ldr	r3, [r7, #740]
	cmp	r3, #0
	bne	.L3464
	ldr	r4, .L3505+8
	ldr	r1, [r4, #2956]
	cmp	r1, #0
	beq	.L3464
	ldrb	r3, [r4, #1888]	@ zero_extendqisi2
	cbz	r3, .L3487
	ldrb	r8, [r5, #8]	@ zero_extendqisi2
	add	r0, r8, #-1
	rsbs	r8, r0, #0
	adc	r8, r8, r0
	b	.L3466
.L3487:
	mov	r8, r3
.L3466:
	ldr	r0, [r4, #2960]
	mov	r2, r8
	ldrb	r3, [r5, #9]	@ zero_extendqisi2
	mov	r9, #0
	bl	FlashProgPages
	ldr	r10, .L3505+8
	mov	fp, #36
.L3467:
	ldr	r3, [r4, #2956]
	cmp	r9, r3
	bcs	.L3485
	mul	r6, fp, r9
	ldr	r3, [r10, #2960]
	adds	r2, r3, r6
	ldr	r3, [r3, r6]
	adds	r3, r3, #1
	beq	.L3488
	ldr	r3, [r2, #4]
	cmp	r8, #0
	beq	.L3501
	orr	r3, r3, #-2147483648
.L3501:
	ldr	r0, [r2, #16]
	add	r1, sp, #4
	movs	r2, #1
	str	r3, [sp, #4]
	bl	log2phys
	ldr	r3, [r4, #2960]
	add	r6, r6, r3
	ldr	r3, [r6, #12]
	ldr	r0, [r3, #12]
	adds	r2, r0, #1
	beq	.L3471
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, [r7, #812]
	ldrh	r2, [r3, r0, lsl #1]
	mov	r6, r0
	cbnz	r2, .L3472
	ldr	r0, .L3505+12
	mov	r1, r6
	bl	printk
.L3472:
	mov	r0, r6
	bl	decrement_vpc_count
.L3471:
	add	r9, r9, #1
	b	.L3467
.L3503:
	movw	r5, #16386
.L3484:
	ldrh	r3, [r7, #1698]
	cbz	r3, .L3485
	movs	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	subs	r5, r5, #1
	bne	.L3484
.L3485:
	movs	r3, #0
	str	r3, [r4, #2956]
	b	.L3464
.L3488:
	ldr	r10, .L3505+8
	mov	r9, #0
.L3468:
	ldr	r3, [r4, #2956]
	cmp	r9, r3
	bcs	.L3503
	movs	r6, #36
	ldr	r3, [r10, #2960]
	mov	fp, #0
	mov	r2, #-1
	mul	r6, r6, r9
	str	r2, [r3, r6]
.L3474:
	ldr	r3, [r4, #2960]
	adds	r2, r3, r6
	ldr	r3, [r3, r6]
	adds	r3, r3, #1
	bne	.L3504
	ldr	r0, [r2, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r5]
	cmp	r3, r0
	bne	.L3475
	ldr	r1, [r7, #812]
	ldrh	r0, [r5, #4]
	ldrh	r2, [r1, r3, lsl #1]
	subs	r2, r2, r0
	strh	r2, [r1, r3, lsl #1]	@ movhi
	ldrh	r3, [r10, #2908]
	strb	fp, [r5, #6]
	strh	fp, [r5, #4]	@ movhi
	strh	r3, [r5, #2]	@ movhi
.L3475:
	ldrh	r3, [r5, #4]
	cbnz	r3, .L3476
	mov	r0, r5
	bl	allocate_new_data_superblock
.L3476:
	ldr	r3, [r7, #1220]
	adds	r3, r3, #1
	str	r3, [r7, #1220]
	ldr	r3, [r4, #2960]
	add	r3, r3, r6
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	FtlGcMarkBadPhyBlk
	mov	r0, r5
	bl	get_new_active_ppa
	ldr	r3, [r4, #2960]
	mov	r2, r8
	movs	r1, #1
	add	r3, r3, r6
	str	r0, [sp, #4]
	str	r0, [r3, #4]
	mov	r0, r3
	ldrb	r3, [r5, #9]	@ zero_extendqisi2
	bl	FlashProgPages
	ldr	r3, [r4, #2960]
	ldr	r3, [r3, r6]
	adds	r3, r3, #1
	ittt	eq
	moveq	r3, #1
	ldreq	r2, .L3505+4
	streq	r3, [r2, #740]
	ldr	r3, [r7, #740]
	cmp	r3, #0
	beq	.L3474
	b	.L3464
.L3504:
	ldr	r3, [r2, #4]
	cmp	r8, #0
	beq	.L3502
	orr	r3, r3, #-2147483648
.L3502:
	ldr	r0, [r2, #16]
	add	r1, sp, #4
	movs	r2, #1
	str	r3, [sp, #4]
	bl	log2phys
	ldr	r3, [r4, #2960]
	add	r6, r6, r3
	ldr	r3, [r6, #12]
	ldr	r0, [r3, #12]
	adds	r3, r0, #1
	beq	.L3481
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, [r7, #812]
	ldrh	r2, [r3, r0, lsl #1]
	mov	r6, r0
	cbnz	r2, .L3482
	ldr	r0, .L3505+12
	mov	r1, r6
	bl	printk
.L3482:
	mov	r0, r6
	bl	decrement_vpc_count
.L3481:
	add	r9, r9, #1
	b	.L3468
.L3464:
	movs	r0, #0
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3506:
	.align	2
.L3505:
	.word	.LANCHOR5
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC153
	.fnend
	.size	FtlCacheWriteBack, .-FtlCacheWriteBack
	.align	1
	.global	FtlSysFlush
	.thumb
	.thumb_func
	.type	FtlSysFlush, %function
FtlSysFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	.save {r3, lr}
	ldr	r3, .L3512
	ldr	r3, [r3, #740]
	cbnz	r3, .L3508
	ldr	r3, .L3512+4
	ldr	r3, [r3, #504]
	adds	r3, r3, #1
	beq	.L3508
	bl	FtlCacheWriteBack
	bl	l2p_flush
	movs	r0, #1
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L3508:
	movs	r0, #0
	pop	{r3, pc}
.L3513:
	.align	2
.L3512:
	.word	.LANCHOR2
	.word	.LANCHOR1
	.fnend
	.size	FtlSysFlush, .-FtlSysFlush
	.align	1
	.global	FtlDeInit
	.thumb
	.thumb_func
	.type	FtlDeInit, %function
FtlDeInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	.save {r3, lr}
	ldr	r3, .L3516
	ldr	r3, [r3, #504]
	cmp	r3, #1
	bne	.L3515
	bl	FtlSysFlush
.L3515:
	movs	r0, #0
	pop	{r3, pc}
.L3517:
	.align	2
.L3516:
	.word	.LANCHOR1
	.fnend
	.size	FtlDeInit, .-FtlDeInit
	.align	1
	.global	ftl_deinit
	.thumb
	.thumb_func
	.type	ftl_deinit, %function
ftl_deinit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	.save {r3, lr}
	bl	ftl_flash_de_init
	bl	FtlDeInit
	pop	{r3, lr}
	b	ftl_flash_de_init
	.fnend
	.size	ftl_deinit, .-ftl_deinit
	.align	1
	.global	rk_ftl_de_init
	.thumb
	.thumb_func
	.type	rk_ftl_de_init, %function
rk_ftl_de_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	.save {r3, lr}
	movs	r1, #0
	ldr	r0, .L3520
	bl	printk
	pop	{r3, lr}
	b	ftl_deinit
.L3521:
	.align	2
.L3520:
	.word	.LC154
	.fnend
	.size	rk_ftl_de_init, .-rk_ftl_de_init
	.align	1
	.global	ftl_cache_flush
	.thumb
	.thumb_func
	.type	ftl_cache_flush, %function
ftl_cache_flush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	FtlCacheWriteBack
	.fnend
	.size	ftl_cache_flush, .-ftl_cache_flush
	.align	1
	.global	rk_ftl_cache_write_back
	.thumb
	.thumb_func
	.type	rk_ftl_cache_write_back, %function
rk_ftl_cache_write_back:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	FtlCacheWriteBack
	.fnend
	.size	rk_ftl_cache_write_back, .-rk_ftl_cache_write_back
	.align	1
	.global	ftl_discard
	.thumb
	.thumb_func
	.type	ftl_discard, %function
ftl_discard:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #12
	adds	r2, r0, r1
	ldr	r7, .L3542
	mov	r6, r0
	mov	r5, r1
	ldr	r3, [r7, #1892]
	cmp	r2, r3
	bhi	.L3531
	ldr	r3, .L3542+4
	ldr	r0, [r3, #504]
	adds	r3, r0, #1
	beq	.L3525
	cmp	r1, #31
	bls	.L3534
	ldr	r3, .L3542+8
	ldr	r2, [r3, #740]
	mov	r8, r3
	cmp	r2, #0
	bne	.L3534
	bl	FtlCacheWriteBack
	ldrh	r4, [r7, #2914]
	mov	r0, r6
	mov	r1, r4
	bl	__aeabi_uidiv
	smulbb	r3, r0, r4
	mov	r9, r0
	subs	r6, r6, r3
	uxth	r6, r6
	cbz	r6, .L3526
	subs	r4, r4, r6
	add	r9, r0, #1
	cmp	r4, r5
	it	cs
	movcs	r4, r5
	uxth	r4, r4
	subs	r5, r5, r4
.L3526:
	ldr	r4, .L3542+12
	mov	r3, #-1
	str	r3, [sp, #4]
.L3527:
	ldrh	r3, [r7, #2914]
	cmp	r5, r3
	bcc	.L3541
	mov	r0, r9
	mov	r1, sp
	movs	r2, #0
	bl	log2phys
	ldr	r3, [sp]
	adds	r3, r3, #1
	beq	.L3528
	ldr	r3, [r4, #3796]
	add	r1, sp, #4
	movs	r2, #1
	mov	r0, r9
	adds	r3, r3, #1
	str	r3, [r4, #3796]
	ldr	r3, [r8, #996]
	adds	r3, r3, #1
	str	r3, [r8, #996]
	bl	log2phys
	ldr	r0, [sp]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	decrement_vpc_count
.L3528:
	ldrh	r3, [r7, #2914]
	add	r9, r9, #1
	subs	r5, r5, r3
	b	.L3527
.L3541:
	ldr	r3, .L3542+12
	ldr	r2, [r3, #3796]
	cmp	r2, #32
	bls	.L3534
	movs	r2, #0
	str	r2, [r3, #3796]
	bl	l2p_flush
	bl	FtlVpcTblFlush
	b	.L3534
.L3531:
	mov	r0, #-1
	b	.L3525
.L3534:
	movs	r0, #0
.L3525:
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, pc}
.L3543:
	.align	2
.L3542:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR2
	.word	.LANCHOR5
	.fnend
	.size	ftl_discard, .-ftl_discard
	.align	1
	.global	FtlDiscard
	.thumb
	.thumb_func
	.type	FtlDiscard, %function
FtlDiscard:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	ftl_discard
	.fnend
	.size	FtlDiscard, .-FtlDiscard
	.align	1
	.global	ftl_read
	.thumb
	.thumb_func
	.type	ftl_read, %function
ftl_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r8, r3
	ldr	r3, .L3588
	.pad #84
	sub	sp, sp, #84
	mov	r7, r1
	str	r2, [sp, #44]
	ldr	r3, [r3, #504]
	adds	r4, r3, #1
	beq	.L3568
	cmp	r0, #16
	bne	.L3547
	add	r0, r1, #256
	mov	r1, r2
	mov	r2, r8
	bl	FtlVendorPartRead
	b	.L3546
.L3547:
	ldr	r3, [sp, #44]
	adds	r3, r1, r3
	str	r3, [sp, #36]
	ldr	r3, .L3588+4
	ldr	r1, [sp, #36]
	ldr	r2, [r3, #1892]
	cmp	r1, r2
	bhi	.L3569
	ldrh	r4, [r3, #2914]
	mov	r0, r7
	mov	r1, r4
	bl	__aeabi_uidiv
	mov	r1, r4
	ldr	r3, [sp, #36]
	str	r0, [sp, #28]
	subs	r0, r3, #1
	bl	__aeabi_uidiv
	ldr	r3, [sp, #28]
	ldr	r1, [sp, #44]
	rsb	r3, r3, #1
	str	r0, [sp, #32]
	add	r3, r3, r0
	str	r3, [sp, #24]
	ldr	r3, .L3588+8
	ldr	r0, [sp, #28]
	ldr	r2, [r3, #1020]
	add	r2, r2, r1
	ldr	r1, [sp, #24]
	str	r2, [r3, #1020]
	ldr	r2, [r3, #992]
	add	r2, r2, r1
	ldr	r1, [sp, #32]
	str	r2, [r3, #992]
	bl	FtlCacheMetchLpa
	cbz	r0, .L3548
	bl	FtlCacheWriteBack
.L3548:
	mov	r10, #0
	ldr	r5, [sp, #28]
	ldr	r4, .L3588+8
	ldr	r9, .L3588+4
	mov	r6, r10
	str	r10, [sp, #48]
	str	r10, [sp, #52]
.L3549:
	ldr	r3, [sp, #24]
	cmp	r3, #0
	beq	.L3587
	add	r1, sp, #76
	mov	r0, r5
	movs	r2, #0
	bl	log2phys
	ldr	r3, [sp, #76]
	adds	r1, r3, #1
	bne	.L3584
	mov	fp, #0
.L3550:
	ldrh	r0, [r9, #2914]
	cmp	fp, r0
	bcs	.L3554
	mla	r0, r0, r5, fp
	cmp	r0, r7
	bcc	.L3552
	ldr	r3, [sp, #36]
	cmp	r0, r3
	bcs	.L3552
	subs	r0, r0, r7
	mov	r1, #512
	add	r0, r8, r0, lsl #9
	bl	__memzero
.L3552:
	add	fp, fp, #1
	b	.L3550
.L3584:
	ldr	r2, [r4, #1768]
	mov	fp, #36
	mla	fp, fp, r6, r2
	str	r3, [fp, #4]
	ldr	r3, [sp, #28]
	cmp	r5, r3
	bne	.L3555
	ldrh	ip, [r9, #2914]
	mov	r0, r7
	ldr	r3, [r4, #1796]
	mov	r1, ip
	str	ip, [sp, #40]
	str	r3, [fp, #8]
	bl	__aeabi_uidivmod
	ldr	ip, [sp, #40]
	ldr	r2, [sp, #44]
	rsb	r3, r1, ip
	str	r1, [sp, #56]
	cmp	r3, r2
	it	cs
	movcs	r3, r2
	cmp	r3, ip
	str	r3, [sp, #48]
	bne	.L3556
	str	r8, [fp, #8]
	b	.L3556
.L3555:
	ldr	r3, [sp, #32]
	cmp	r5, r3
	bne	.L3557
	ldr	r3, [r4, #1800]
	ldrh	r2, [r9, #2914]
	ldr	r1, [sp, #36]
	str	r3, [fp, #8]
	mul	r3, r2, r5
	rsb	r10, r3, r1
	cmp	r10, r2
	bne	.L3556
	b	.L3585
.L3557:
	ldrh	r3, [r9, #2914]
	muls	r3, r5, r3
.L3585:
	subs	r3, r3, r7
	add	r3, r8, r3, lsl #9
	str	r3, [fp, #8]
.L3556:
	ldrh	r3, [r9, #2920]
	ldr	r2, [r4, #1808]
	str	r5, [fp, #16]
	muls	r3, r6, r3
	adds	r6, r6, #1
	bic	r3, r3, #3
	add	r3, r3, r2
	str	r3, [fp, #12]
.L3554:
	ldr	r3, [sp, #24]
	adds	r5, r5, #1
	subs	r3, r3, #1
	str	r3, [sp, #24]
	beq	.L3558
	ldrh	r3, [r9, #2840]
	cmp	r6, r3, lsl #3
	bne	.L3549
.L3558:
	cmp	r6, #0
	beq	.L3549
	ldr	r0, [r4, #1768]
	mov	r1, r6
	movs	r2, #0
	bl	FlashReadPages
	ldr	r3, [sp, #56]
	lsls	r3, r3, #9
	str	r3, [sp, #60]
	ldr	r3, [sp, #48]
	lsls	r3, r3, #9
	str	r3, [sp, #64]
	lsl	r3, r10, #9
	str	r3, [sp, #68]
	movs	r3, #0
	str	r3, [sp, #40]
.L3565:
	ldr	r3, [sp, #40]
	mov	ip, #36
	ldr	r1, [sp, #28]
	mul	fp, ip, r3
	ldr	r3, [r4, #1768]
	add	r3, r3, fp
	ldr	r2, [r3, #16]
	cmp	r2, r1
	bne	.L3560
	ldr	r1, [r3, #8]
	ldr	r3, [r4, #1796]
	cmp	r1, r3
	bne	.L3561
	ldr	r3, [sp, #60]
	mov	r0, r8
	ldr	r2, [sp, #64]
	add	r1, r1, r3
	b	.L3586
.L3560:
	ldr	r1, [sp, #32]
	cmp	r2, r1
	bne	.L3561
	ldr	r1, [r3, #8]
	ldr	r3, [r4, #1800]
	cmp	r1, r3
	bne	.L3561
	ldrh	r0, [r9, #2914]
	ldr	r3, [sp, #32]
	ldr	r2, [sp, #68]
	muls	r0, r3, r0
	subs	r0, r0, r7
	add	r0, r8, r0, lsl #9
.L3586:
	bl	ftl_memcpy
.L3561:
	ldr	r2, [r4, #1768]
	add	r3, r2, fp
	ldr	r1, [r2, fp]
	adds	r2, r1, #1
	itttt	eq
	streq	r1, [sp, #52]
	ldreq	r2, [r4, #1196]
	addeq	r2, r2, #1
	streq	r2, [r4, #1196]
	ldr	r2, [r3, #12]
	ldr	r1, [r3, #16]
	ldr	r2, [r2, #8]
	cmp	r1, r2
	beq	.L3563
	ldr	r2, [r4, #1196]
	ldr	r0, .L3588+12
	adds	r2, r2, #1
	str	r2, [r4, #1196]
	ldr	lr, [r3, #12]
	ldr	r2, [r3, #8]
	ldr	r1, [lr, #4]
	str	r1, [sp]
	ldr	r1, [lr, #8]
	str	r1, [sp, #4]
	ldr	r1, [lr, #12]
	str	r1, [sp, #8]
	ldr	r1, [r2]
	str	r1, [sp, #12]
	ldr	r2, [r2, #4]
	str	r2, [sp, #16]
	ldr	r1, [r3, #16]
	ldr	r2, [r3, #4]
	ldr	r3, [lr]
	bl	printk
.L3563:
	ldr	r3, [r4, #1768]
	add	r2, r3, fp
	ldr	r3, [r3, fp]
	cmp	r3, #256
	bne	.L3564
	ldr	r0, [r2, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L3564:
	ldr	r3, [sp, #40]
	adds	r3, r3, #1
	str	r3, [sp, #40]
	cmp	r3, r6
	bne	.L3565
	movs	r6, #0
	b	.L3549
.L3587:
	ldr	r3, .L3588+8
	ldrh	r3, [r3, #1698]
	cbz	r3, .L3567
	ldr	r0, [sp, #24]
	movs	r1, #1
	bl	ftl_do_gc
.L3567:
	ldr	r0, [sp, #52]
	b	.L3546
.L3568:
	mov	r0, r3
	b	.L3546
.L3569:
	mov	r0, #-1
.L3546:
	add	sp, sp, #84
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3589:
	.align	2
.L3588:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC143
	.fnend
	.size	ftl_read, .-ftl_read
	.align	1
	.global	ftl_vendor_read
	.thumb
	.thumb_func
	.type	ftl_vendor_read, %function
ftl_vendor_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, lr}
	.save {r4, r5, lr}
	mov	r5, r0
	mov	r4, r1
	mov	r3, r2
	mov	r1, r5
	mov	r2, r4
	movs	r0, #16
	pop	{r4, r5, lr}
	b	ftl_read
	.fnend
	.size	ftl_vendor_read, .-ftl_vendor_read
	.align	1
	.global	FlashBootVendorRead
	.thumb
	.thumb_func
	.type	FlashBootVendorRead, %function
FlashBootVendorRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0
	mov	r5, r1
	mov	r4, r2
	bl	rknand_device_lock
	ldr	r3, .L3594
	ldr	r3, [r3, #504]
	cmp	r3, #1
	bne	.L3593
	mov	r2, r4
	mov	r0, r6
	mov	r1, r5
	bl	ftl_vendor_read
	mov	r4, r0
	b	.L3592
.L3593:
	mov	r4, #-1
.L3592:
	bl	rknand_device_unlock
	mov	r0, r4
	pop	{r4, r5, r6, pc}
.L3595:
	.align	2
.L3594:
	.word	.LANCHOR1
	.fnend
	.size	FlashBootVendorRead, .-FlashBootVendorRead
	.align	1
	.global	ftl_sys_read
	.thumb
	.thumb_func
	.type	ftl_sys_read, %function
ftl_sys_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	mov	r4, r1
	mov	r3, r2
	add	r1, r0, #256
	mov	r2, r4
	movs	r0, #16
	pop	{r4, lr}
	b	ftl_read
	.fnend
	.size	ftl_sys_read, .-ftl_sys_read
	.align	1
	.global	StorageSysDataLoad
	.thumb
	.thumb_func
	.type	StorageSysDataLoad, %function
StorageSysDataLoad:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r5, r0
	mov	r4, r1
	mov	r0, r1
	mov	r1, #512
	bl	__memzero
	bl	rknand_device_lock
	mov	r2, r4
	movs	r1, #1
	mov	r0, r5
	bl	ftl_sys_read
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	pop	{r3, r4, r5, pc}
	.fnend
	.size	StorageSysDataLoad, .-StorageSysDataLoad
	.align	1
	.global	FtlRead
	.thumb
	.thumb_func
	.type	FtlRead, %function
FtlRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	ftl_read
	.fnend
	.size	FtlRead, .-FtlRead
	.align	1
	.global	FtlInit
	.thumb
	.thumb_func
	.type	FtlInit, %function
FtlInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r3, #-1
	ldr	r2, .L3615
	ldr	r7, .L3615+4
	ldr	r4, .L3615+8
	ldr	r6, .L3615+12
	ldr	r1, .L3615+16
	ldr	r0, .L3615+20
	str	r3, [r7, #504]
	movs	r3, #0
	str	r3, [r2, #3800]
	str	r3, [r4, #740]
	bl	printk
	addw	r0, r6, #1860
	bl	FtlConstantsInit
	bl	FtlMemInit
	bl	FtlVariablesInit
	ldrh	r0, [r6, #2844]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cbz	r0, .L3600
	ldr	r0, .L3615+24
	b	.L3614
.L3600:
	bl	FtlSysBlkInit
	mov	r5, r0
	cbz	r0, .L3602
	ldr	r0, .L3615+28
.L3614:
	ldr	r1, .L3615+32
	bl	printk
	b	.L3601
.L3602:
	movs	r1, #1
	str	r1, [r7, #504]
	bl	ftl_do_gc
	ldrh	r7, [r4, #828]
	cmp	r7, #15
	bhi	.L3603
	mov	r8, r4
	movw	r6, #65535
.L3606:
	ldrh	r3, [r4, #1072]
	cmp	r3, r6
	bne	.L3604
	ldrh	r3, [r8, #1690]
	cmp	r3, r6
	bne	.L3604
	and	r0, r5, #7
	bl	List_get_gc_head_node
	uxth	r0, r0
	bl	FtlGcRefreshBlock
.L3604:
	movs	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	movs	r0, #0
	movs	r1, #1
	bl	ftl_do_gc
	ldrh	r2, [r4, #828]
	adds	r3, r7, #2
	cmp	r2, r3
	bhi	.L3601
	adds	r5, r5, #1
	cmp	r5, #1024
	bne	.L3606
	b	.L3601
.L3603:
	ldrb	r3, [r6, #1888]	@ zero_extendqisi2
	cbz	r3, .L3601
	movs	r4, #128
.L3608:
	movs	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	subs	r4, r4, #1
	bne	.L3608
.L3601:
	movs	r0, #0
	pop	{r4, r5, r6, r7, r8, pc}
.L3616:
	.align	2
.L3615:
	.word	.LANCHOR5
	.word	.LANCHOR1
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC77
	.word	.LC76
	.word	.LC155
	.word	.LC156
	.word	.LANCHOR3+108
	.fnend
	.size	FtlInit, .-FtlInit
	.align	1
	.global	rk_ftl_init
	.thumb
	.thumb_func
	.type	rk_ftl_init, %function
rk_ftl_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r0, #2048
	bl	ftl_malloc
	ldr	r5, .L3621
	ldr	r4, .L3621+4
	movs	r6, #0
	add	r1, r5, #3808
	str	r6, [r5, #3808]
	str	r6, [r4, #3988]
	str	r0, [r5, #3804]
	addw	r0, r4, #3988
	bl	rknand_get_reg_addr
	ldr	r3, [r4, #3988]
	cbz	r3, .L3620
	bl	rk_nandc_irq_init
	mov	r1, r6
	mov	r2, r6
	mov	r3, #2048
	ldr	r0, [r5, #3804]
	bl	FlashSramLoadStore
	bl	rknand_flash_cs_init
	ldr	r0, [r4, #3988]
	bl	FlashInit
	mov	r4, r0
	cbnz	r0, .L3619
	bl	FtlInit
.L3619:
	mov	r1, r4
	ldr	r0, .L3621+8
	bl	printk
	mov	r0, r4
	pop	{r4, r5, r6, pc}
.L3620:
	mov	r0, #-1
	pop	{r4, r5, r6, pc}
.L3622:
	.align	2
.L3621:
	.word	.LANCHOR5
	.word	.LANCHOR2
	.word	.LC157
	.fnend
	.size	rk_ftl_init, .-rk_ftl_init
	.align	1
	.global	ftl_fix_nand_power_lost_error
	.thumb
	.thumb_func
	.type	ftl_fix_nand_power_lost_error, %function
ftl_fix_nand_power_lost_error:
	.fnstart
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L3638
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	.pad #48
	sub	sp, sp, #48
	ldrb	r2, [r3, #1888]	@ zero_extendqisi2
	mov	r8, r3
	cmp	r2, #0
	beq	.L3623
	ldr	r4, .L3638+4
	movw	r7, #4097
	ldr	r6, .L3638+8
	ldr	r0, .L3638+12
	ldr	r3, [r4, #812]
	ldrh	r5, [r6, #3648]
	mov	r1, r5
	ldrh	r2, [r3, r5, lsl #1]
	bl	printk
	ldrh	r0, [r4, #832]
	bl	FtlGcRefreshOpenBlock
	ldrh	r0, [r4, #880]
	bl	FtlGcRefreshOpenBlock
	add	r0, r4, #832
	bl	allocate_new_data_superblock
	add	r0, r4, #880
	bl	allocate_new_data_superblock
	lsl	r9, r5, #1
.L3625:
	subs	r7, r7, #1
	beq	.L3629
	movs	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	ldr	r3, [r4, #812]
	ldrh	r3, [r3, r9]
	cmp	r3, #0
	bne	.L3625
.L3629:
	ldr	r3, [r4, #812]
	mov	r1, r5
	ldr	r0, .L3638+12
	ldrh	r2, [r3, r5, lsl #1]
	bl	printk
	ldr	r3, [r4, #812]
	ldrh	r7, [r3, r5, lsl #1]
	cbnz	r7, .L3627
	add	r0, sp, #48
	mov	r10, #36
	movw	r9, #65535
	strh	r5, [r0, #-48]!	@ movhi
	bl	make_superblock
	ldr	r3, .L3638+4
	ldrh	ip, [r8, #2840]
	ldr	r8, [r3, #744]
	mov	r3, r7
	mov	lr, r3
	add	r0, sp, #14
.L3630:
	uxth	r2, r3
	cmp	r2, ip
	bcs	.L3637
	ldrh	r2, [r0, #2]!
	cmp	r2, r9
	beq	.L3631
	mla	r1, r10, r7, r8
	adds	r7, r7, #1
	lsls	r2, r2, #10
	uxth	r7, r7
	str	r2, [r1, #4]
	str	lr, [r1, #8]
	str	lr, [r1, #12]
.L3631:
	adds	r3, r3, #1
	b	.L3630
.L3637:
	ldr	r3, [r4, #812]
	mov	r1, r5
	ldr	r0, .L3638+16
	ldrh	r2, [r3, r5, lsl #1]
	bl	printk
	movs	r1, #0
	mov	r2, r7
	ldr	r0, [r4, #744]
	bl	FlashEraseBlocks
	ldr	r0, [r4, #744]
	movs	r1, #1
	mov	r2, r7
	bl	FlashEraseBlocks
.L3627:
	movw	r3, #65535
	strh	r3, [r6, #3648]	@ movhi
.L3623:
	add	sp, sp, #48
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L3639:
	.align	2
.L3638:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR5
	.word	.LC158
	.word	.LC159
	.fnend
	.size	ftl_fix_nand_power_lost_error, .-ftl_fix_nand_power_lost_error
	.align	1
	.global	rk_ftl_garbage_collect
	.thumb
	.thumb_func
	.type	rk_ftl_garbage_collect, %function
rk_ftl_garbage_collect:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	ftl_do_gc
	.fnend
	.size	rk_ftl_garbage_collect, .-rk_ftl_garbage_collect
	.align	1
	.global	ftl_write
	.thumb
	.thumb_func
	.type	ftl_write, %function
ftl_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 88
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #92
	sub	sp, sp, #92
	ldr	r9, .L3714+16
	mov	r8, r1
	str	r3, [sp, #12]
	str	r2, [sp, #8]
	ldr	r3, [r9, #740]
	cmp	r3, #0
	bne	.L3683
	ldr	r2, .L3714
	ldr	r2, [r2, #504]
	adds	r2, r2, #1
	beq	.L3684
	cmp	r0, #16
	bne	.L3643
	add	r0, r1, #256
	ldr	r2, [sp, #12]
	ldr	r1, [sp, #8]
	bl	FtlVendorPartWrite
	b	.L3642
.L3643:
	ldr	r7, .L3714+4
	ldr	r3, [sp, #8]
	adds	r6, r1, r3
	ldr	r3, [r7, #1892]
	cmp	r6, r3
	bhi	.L3685
	ldrh	r4, [r7, #2914]
	mov	r3, #2048
	ldr	r5, .L3714+8
	mov	r0, r1
	mov	r1, r4
	str	r3, [r5, #3812]
	bl	__aeabi_uidiv
	mov	r1, r4
	mov	fp, r0
	subs	r0, r6, #1
	bl	__aeabi_uidiv
	rsb	r6, fp, r0
	str	r0, [sp, #20]
	adds	r3, r6, #1
	str	r3, [sp, #4]
	ldr	r2, [sp, #4]
	ldr	r3, [r9, #1000]
	add	r3, r3, r2
	ldr	r2, [sp, #8]
	str	r3, [r9, #1000]
	ldr	r3, [r9, #1016]
	cmp	r2, r4, lsl #1
	add	r3, r3, r2
	str	r3, [r9, #1016]
	mov	r3, r2
	ldr	r2, [r7, #2956]
	ite	cs
	movcs	r3, #1
	movcc	r3, #0
	str	r3, [sp, #28]
	cmp	r2, #0
	beq	.L3645
	movs	r3, #36
	ldr	r7, [r7, #2960]
	muls	r3, r2, r3
	subs	r3, r3, #36
	add	r7, r7, r3
	ldr	r3, [r7, #16]
	cmp	fp, r3
	bne	.L3646
	ldr	r3, [r9, #1004]
	mov	r1, r4
	mov	r0, r8
	adds	r3, r3, #1
	str	r3, [r9, #1004]
	ldr	r3, [r5, #3816]
	adds	r3, r3, #1
	str	r3, [r5, #3816]
	bl	__aeabi_uidivmod
	ldr	r0, [r7, #8]
	ldr	r3, [sp, #8]
	subs	r4, r4, r1
	add	r0, r0, r1, lsl #9
	ldr	r1, [sp, #12]
	cmp	r4, r3
	it	cs
	movcs	r4, r3
	lsl	r9, r4, #9
	mov	r2, r9
	bl	ftl_memcpy
	cbnz	r6, .L3647
	ldr	r3, [r5, #3816]
	cmp	r3, #2
	ble	.L3683
.L3647:
	ldr	r3, [sp, #8]
	add	r8, r8, r4
	add	fp, fp, #1
	str	r6, [sp, #4]
	subs	r3, r3, r4
	str	r3, [sp, #8]
	ldr	r3, [sp, #12]
	add	r3, r3, r9
	str	r3, [sp, #12]
.L3646:
	movs	r3, #0
	str	r3, [r5, #3816]
.L3645:
	mov	r0, fp
	ldr	r1, [sp, #20]
	bl	FtlCacheMetchLpa
	cbz	r0, .L3648
	bl	FtlCacheWriteBack
.L3648:
	ldr	r4, .L3714+4
	mov	r6, fp
	ldr	r7, .L3714+12
	mov	r10, r4
	str	r7, [r5, #3792]
.L3649:
	ldr	r3, [sp, #4]
	ldr	r5, .L3714+16
	cmp	r3, #0
	beq	.L3713
	ldrh	r3, [r7, #4]
	cbnz	r3, .L3650
	add	r2, r5, #832
	ldr	r9, .L3714
	cmp	r7, r2
	bne	.L3651
	ldrh	r7, [r5, #884]
	cbnz	r7, .L3652
	add	r0, r5, #880
	bl	allocate_new_data_superblock
	str	r7, [r9, #3292]
.L3652:
	ldr	r0, .L3714+12
	bl	allocate_new_data_superblock
	ldr	r3, [r9, #3292]
	cbnz	r3, .L3686
.L3654:
	ldr	r7, .L3714+12
	b	.L3653
.L3651:
	str	r3, [r9, #3292]
	ldrh	r3, [r5, #836]
	cmp	r3, #0
	bne	.L3654
	mov	r0, r7
	bl	allocate_new_data_superblock
	b	.L3653
.L3686:
	ldr	r7, .L3714+20
.L3653:
	ldrh	r3, [r7, #4]
	cbnz	r3, .L3655
	mov	r0, r7
	bl	allocate_new_data_superblock
.L3655:
	ldr	r3, .L3714+8
	str	r7, [r3, #3792]
.L3650:
	ldr	r2, [r5, #1764]
	ldr	r3, [r4, #2956]
	ldrh	ip, [r7, #4]
	subs	r3, r2, r3
	ldr	r2, [sp, #4]
	cmp	ip, r3
	ldr	r9, .L3714+16
	it	cs
	movcs	ip, r3
	cmp	ip, r2
	mov	r3, ip
	it	cs
	movcs	r3, r2
	str	r3, [sp, #44]
	movs	r3, #0
	str	r3, [sp, #24]
.L3656:
	ldr	r3, [sp, #24]
	ldr	r2, [sp, #44]
	cmp	r3, r2
	beq	.L3657
	ldrh	r3, [r7, #4]
	cmp	r3, #0
	beq	.L3657
	ldr	r3, [sp, #28]
	cbz	r3, .L3658
	ldr	r3, [sp, #20]
	cmp	r6, r3
	bne	.L3658
	ldr	r3, [sp, #24]
	cbz	r3, .L3658
	ldr	r3, [sp, #8]
	ldrh	r2, [r4, #2914]
	add	r3, r8, r3
	mls	r3, r2, r6, r3
	cmp	r3, r2
	bne	.L3657
.L3658:
	add	r1, sp, #48
	movs	r2, #0
	mov	r0, r6
	movs	r5, #36
	bl	log2phys
	mov	r0, r7
	bl	get_new_active_ppa
	ldr	r3, [r4, #2956]
	ldr	r1, [r4, #2960]
	ldrh	r2, [r4, #2920]
	mla	r1, r5, r3, r1
	mul	ip, r3, r2
	str	r6, [r1, #16]
	str	r0, [r1, #4]
	lsr	r0, ip, #2
	str	r0, [sp, #36]
	ldr	r0, [r9, #1812]
	mov	lr, r0
	str	r0, [sp, #40]
	ldr	r0, [sp, #36]
	add	r0, lr, r0, lsl #2
	ldrh	lr, [r4, #2918]
	str	r0, [sp, #16]
	str	r0, [r1, #12]
	mul	r3, r3, lr
	ldr	r0, [r9, #1792]
	bic	r3, r3, #3
	add	r3, r3, r0
	ldr	r0, [sp, #16]
	str	r3, [r1, #8]
	movs	r1, #0
	bl	ftl_memset
	cmp	r6, fp
	beq	.L3659
	ldr	r3, [sp, #20]
	cmp	r6, r3
	bne	.L3710
	ldr	r3, [sp, #8]
	add	r5, r8, r3
	ldrh	r3, [r4, #2914]
	smulbb	r3, r3, r6
	subs	r5, r5, r3
	movs	r3, #0
	str	r3, [sp, #32]
	uxth	r5, r5
	b	.L3662
.L3659:
	ldrh	r5, [r4, #2914]
	mov	r0, r8
	mov	r1, r5
	bl	__aeabi_uidivmod
	ldr	r3, [sp, #8]
	subs	r5, r5, r1
	str	r1, [sp, #32]
	cmp	r5, r3
	it	cs
	movcs	r5, r3
.L3662:
	ldrh	r3, [r4, #2914]
	cmp	r5, r3
	bne	.L3663
	cmp	r6, fp
	mov	lr, #36
	ittet	ne
	mulne	r1, r6, r5
	ldrne	r3, [sp, #12]
	ldreq	r1, [sp, #12]
	rsbne	r1, r8, r1
	it	ne
	addne	r1, r3, r1, lsl #9
	ldr	r3, [sp, #28]
	cbz	r3, .L3665
	ldr	r2, [r10, #2956]
	ldr	r5, [r10, #2960]
	mla	r2, lr, r2, r5
	str	r1, [r2, #8]
	b	.L3666
.L3665:
	ldr	r0, [r10, #2960]
	ldr	r3, [r10, #2956]
	ldrh	r2, [r10, #2918]
	mla	r3, lr, r3, r0
	ldr	r0, [r3, #8]
	b	.L3711
.L3663:
	ldr	r2, [sp, #48]
	movs	r3, #36
	adds	r1, r2, #1
	beq	.L3667
	ldr	r1, [r4, #2960]
	add	r0, sp, #52
	str	r2, [sp, #56]
	ldr	r2, [r4, #2956]
	str	r6, [sp, #68]
	mla	r3, r3, r2, r1
	movs	r1, #1
	ldr	r2, [r3, #8]
	ldr	r3, [r3, #12]
	str	r2, [sp, #60]
	movs	r2, #0
	str	r3, [sp, #64]
	bl	FlashReadPages
	ldr	r3, [sp, #52]
	adds	r3, r3, #1
	bne	.L3668
	ldr	r3, [r9, #1196]
	adds	r3, r3, #1
	str	r3, [r9, #1196]
	b	.L3670
.L3715:
	.align	2
.L3714:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR5
	.word	.LANCHOR2+832
	.word	.LANCHOR2
	.word	.LANCHOR2+880
.L3668:
	ldr	r3, [sp, #16]
	ldr	r3, [r3, #8]
	cmp	r3, r6
	beq	.L3670
	ldr	r3, [r9, #1196]
	mov	r2, r6
	ldr	r0, .L3716
	adds	r3, r3, #1
	str	r3, [r9, #1196]
	ldr	r3, [sp, #16]
	ldr	r1, [r3, #8]
	bl	printk
	b	.L3670
.L3667:
	ldr	r1, [r4, #2960]
	ldr	r2, [r4, #2956]
	mla	r2, r3, r2, r1
	movs	r1, #0
	ldr	r0, [r2, #8]
	ldrh	r2, [r4, #2918]
	bl	ftl_memset
.L3670:
	cmp	r6, fp
	mov	lr, #36
	lsl	r2, r5, #9
	bne	.L3671
	ldr	r1, [r4, #2956]
	ldr	r5, [r4, #2960]
	ldr	r3, [sp, #32]
	mla	r1, lr, r1, r5
	ldr	r0, [r1, #8]
	ldr	r1, [sp, #12]
	add	r0, r0, r3, lsl #9
	b	.L3711
.L3671:
	ldr	r0, [r4, #2960]
	ldr	r3, [r4, #2956]
	ldrh	r1, [r4, #2914]
	mla	r3, lr, r3, r0
	muls	r1, r6, r1
	ldr	r0, [r3, #8]
	rsb	r1, r8, r1
	ldr	r3, [sp, #12]
	add	r1, r3, r1, lsl #9
	b	.L3711
.L3710:
	ldr	r3, [sp, #28]
	cbz	r3, .L3672
	ldr	r3, [r4, #2956]
	ldr	r2, [r4, #2960]
	mla	r5, r5, r3, r2
	ldrh	r3, [r4, #2914]
	ldr	r2, [sp, #12]
	muls	r3, r6, r3
	rsb	r3, r8, r3
	add	r3, r2, r3, lsl #9
	str	r3, [r5, #8]
	b	.L3666
.L3672:
	ldr	r2, [r4, #2960]
	ldr	r3, [r4, #2956]
	ldrh	r1, [r4, #2914]
	mla	r3, r5, r3, r2
	ldrh	r2, [r4, #2918]
	muls	r1, r6, r1
	ldr	r0, [r3, #8]
	rsb	r1, r8, r1
	ldr	r3, [sp, #12]
	add	r1, r3, r1, lsl #9
.L3711:
	bl	ftl_memcpy
.L3666:
	ldr	r2, [sp, #40]
	movw	r3, #61589
	ldr	r1, [sp, #36]
	strh	r3, [r2, r1, lsl #2]	@ movhi
	ldr	r2, [sp, #16]
	ldr	r3, [r9, #1028]
	str	r3, [r2, #4]
	adds	r3, r3, #1
	adds	r2, r3, #1
	ldr	r2, [sp, #16]
	it	eq
	moveq	r3, #0
	str	r3, [r9, #1028]
	ldr	r3, [sp, #16]
	str	r6, [r3, #8]
	adds	r6, r6, #1
	ldr	r3, [sp, #48]
	str	r3, [r2, #12]
	ldrh	r3, [r7]
	strh	r3, [r2, #2]	@ movhi
	ldr	r3, [r4, #2956]
	adds	r3, r3, #1
	str	r3, [r4, #2956]
	ldr	r3, [sp, #24]
	adds	r3, r3, #1
	str	r3, [sp, #24]
	b	.L3656
.L3657:
	ldr	r3, [sp, #4]
	ldr	r2, [sp, #24]
	subs	r3, r3, r2
	str	r3, [sp, #4]
	ldr	r3, .L3716+4
	ldr	r2, [r10, #2956]
	ldr	r3, [r3, #1764]
	cmp	r2, r3
	bcs	.L3676
	ldr	r3, [sp, #28]
	cbnz	r3, .L3676
	ldrh	r3, [r7, #4]
	cbz	r3, .L3676
.L3678:
	movs	r3, #0
	str	r3, [sp, #28]
	b	.L3649
.L3676:
	bl	FtlCacheWriteBack
	movs	r3, #0
	str	r3, [r10, #2956]
	ldr	r3, [sp, #4]
	cmp	r3, #1
	bhi	.L3649
	b	.L3678
.L3713:
	mov	r0, r3
	ldr	r3, [sp, #20]
	rsb	r1, fp, r3
	bl	ftl_do_gc
	ldrh	r3, [r5, #828]
	cmp	r3, #5
	bls	.L3689
	cmp	r3, #31
	bhi	.L3683
	ldr	r3, .L3716+8
	ldrb	r3, [r3]	@ zero_extendqisi2
	cbnz	r3, .L3683
.L3689:
	ldr	r4, [sp, #4]
	mov	r8, #128
	ldr	r6, .L3716+4
	movw	r7, #65535
.L3702:
	ldrh	r3, [r5, #1072]
	cmp	r3, r7
	bne	.L3682
	ldrh	r3, [r6, #1690]
	cmp	r3, r7
	bne	.L3682
	ldrh	r3, [r6, #1692]
	cmp	r3, r7
	bne	.L3682
	and	r0, r4, #7
	bl	List_get_gc_head_node
	uxth	r0, r0
	bl	FtlGcRefreshBlock
.L3682:
	movs	r0, #1
	strh	r8, [r5, #1638]	@ movhi
	strh	r8, [r5, #1636]	@ movhi
	mov	r1, r0
	bl	ftl_do_gc
	movs	r0, #0
	movs	r1, #1
	bl	ftl_do_gc
	ldr	r3, [r5, #740]
	cbnz	r3, .L3683
	ldrh	r3, [r6, #828]
	cmp	r3, #2
	bhi	.L3683
	adds	r4, r4, #1
	cmp	r4, #256
	bne	.L3702
	b	.L3683
.L3685:
	mov	r0, #-1
	b	.L3642
.L3683:
	movs	r0, #0
	b	.L3642
.L3684:
	mov	r0, r3
.L3642:
	add	sp, sp, #92
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3717:
	.align	2
.L3716:
	.word	.LC160
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	ftl_write, .-ftl_write
	.align	1
	.global	ftl_vendor_write
	.thumb
	.thumb_func
	.type	ftl_vendor_write, %function
ftl_vendor_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, lr}
	.save {r4, r5, lr}
	mov	r5, r0
	mov	r4, r1
	mov	r3, r2
	mov	r1, r5
	mov	r2, r4
	movs	r0, #16
	pop	{r4, r5, lr}
	b	ftl_write
	.fnend
	.size	ftl_vendor_write, .-ftl_vendor_write
	.align	1
	.global	FlashBootVendorWrite
	.thumb
	.thumb_func
	.type	FlashBootVendorWrite, %function
FlashBootVendorWrite:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0
	mov	r5, r1
	mov	r4, r2
	bl	rknand_device_lock
	ldr	r3, .L3722
	ldr	r3, [r3, #504]
	cmp	r3, #1
	bne	.L3721
	mov	r2, r4
	mov	r0, r6
	mov	r1, r5
	bl	ftl_vendor_write
	mov	r4, r0
	b	.L3720
.L3721:
	mov	r4, #-1
.L3720:
	bl	rknand_device_unlock
	mov	r0, r4
	pop	{r4, r5, r6, pc}
.L3723:
	.align	2
.L3722:
	.word	.LANCHOR1
	.fnend
	.size	FlashBootVendorWrite, .-FlashBootVendorWrite
	.align	1
	.global	ftl_sys_write
	.thumb
	.thumb_func
	.type	ftl_sys_write, %function
ftl_sys_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	mov	r4, r1
	mov	r3, r2
	add	r1, r0, #256
	mov	r2, r4
	movs	r0, #16
	pop	{r4, lr}
	b	ftl_write
	.fnend
	.size	ftl_sys_write, .-ftl_sys_write
	.align	1
	.global	StorageSysDataStore
	.thumb
	.thumb_func
	.type	StorageSysDataStore, %function
StorageSysDataStore:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, r1
	mov	r5, r0
	bl	rknand_device_lock
	mov	r2, r4
	movs	r1, #1
	mov	r0, r5
	bl	ftl_sys_write
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	pop	{r3, r4, r5, pc}
	.fnend
	.size	StorageSysDataStore, .-StorageSysDataStore
	.align	1
	.global	FtlDumpSysBlock
	.thumb
	.thumb_func
	.type	FtlDumpSysBlock, %function
FtlDumpSysBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	lsl	r8, r0, #10
	ldr	r4, .L3734
	.pad #28
	sub	sp, sp, #28
	ldr	r7, .L3734+4
	movs	r5, #0
	mov	r6, r0
	ldr	r3, [r4, #1780]
	mov	r9, r4
	str	r3, [r4, #4064]
	ldr	r3, [r4, #1804]
	str	r3, [r4, #4068]
.L3727:
	ldrh	r2, [r7, #2910]
	sxth	r3, r5
	cmp	r3, r2
	bge	.L3733
	movs	r1, #1
	ldr	r0, .L3734+8
	orr	r3, r3, r8
	str	r3, [r4, #4060]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r3, [r4, #4068]
	ldr	r0, .L3734+12
	mov	r1, r6
	ldr	r2, [r3]
	str	r2, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	ldr	r2, [r3, #8]
	str	r2, [sp, #8]
	ldr	r3, [r3, #12]
	ldr	r2, [r4, #4056]
	str	r3, [sp, #12]
	ldr	r3, [r4, #4064]
	ldr	r3, [r3]
	str	r3, [sp, #16]
	ldr	r3, [r4, #4060]
	bl	printk
	ldr	r3, [r4, #4068]
	ldr	r3, [r3]
	adds	r3, r3, #1
	beq	.L3728
	ldr	r0, .L3734+16
	movs	r2, #4
	ldr	r1, [r9, #1780]
	mov	r3, #768
	bl	rknand_print_hex
.L3728:
	adds	r5, r5, #1
	b	.L3727
.L3733:
	add	sp, sp, #28
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, pc}
.L3735:
	.align	2
.L3734:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2+4056
	.word	.LC161
	.word	.LC162
	.fnend
	.size	FtlDumpSysBlock, .-FtlDumpSysBlock
	.align	1
	.global	dump_map_info
	.thumb
	.thumb_func
	.type	dump_map_info, %function
dump_map_info:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #52
	sub	sp, sp, #52
	ldr	r5, .L3754
	addw	fp, r5, #2868
	ldrh	r7, [r5, #2848]
	mov	r9, r5
.L3737:
	ldrh	r3, [r5, #2850]
	ldr	r4, .L3754+4
	cmp	r3, r7
	bls	.L3751
	ldr	r1, [r4, #1664]
	mov	r8, #0
	ldrh	ip, [r9, #2840]
	mov	r10, #36
	ldr	r2, [r4, #1768]
	mov	r6, r8
	str	r1, [sp, #24]
	ldr	r3, [r4, #1660]
	ldrh	r1, [r9, #2920]
	str	r1, [sp, #28]
.L3747:
	uxth	r1, r8
	cmp	r1, ip
	bcs	.L3752
	mov	r1, r7
	ldrb	r0, [fp, r8]	@ zero_extendqisi2
	str	r3, [sp, #44]
	str	r2, [sp, #40]
	str	ip, [sp, #36]
	bl	V2P_block
	str	r0, [sp, #32]
	bl	FtlBbmIsBadBlock
	ldr	r1, [sp, #32]
	ldr	ip, [sp, #36]
	ldr	r2, [sp, #40]
	ldr	r3, [sp, #44]
	cbnz	r0, .L3738
	mla	r0, r10, r6, r2
	lsls	r1, r1, #10
	str	r1, [r0, #4]
	ldr	r1, [sp, #28]
	muls	r1, r6, r1
	add	r6, r6, #1
	it	mi
	addmi	r1, r1, #3
	bic	lr, r1, #3
	ldr	r1, [sp, #24]
	uxth	r6, r6
	str	r3, [r0, #8]
	add	r1, r1, lr
	str	r1, [r0, #12]
.L3738:
	add	r8, r8, #1
	b	.L3747
.L3752:
	cbz	r6, .L3742
	ldr	r0, [r4, #1768]
	mov	r1, r6
	movs	r2, #1
	mov	r8, #0
	bl	FlashReadPages
	mov	r10, #36
.L3743:
	uxth	r3, r8
	cmp	r3, r6
	bcs	.L3742
	ldr	r3, [r4, #1768]
	ldr	r0, .L3754+8
	mla	r3, r10, r8, r3
	add	r8, r8, #1
	adds	r2, r3, #4
	ldmia	r2, {r2, r3, lr}
	ldr	r1, [lr, #4]
	str	r1, [sp]
	ldr	r1, [lr, #8]
	str	r1, [sp, #4]
	ldr	r1, [lr, #12]
	str	r1, [sp, #8]
	ldr	r1, [r3]
	str	r1, [sp, #12]
	ubfx	r1, r2, #10, #16
	ldr	r3, [r3, #4]
	str	r3, [sp, #16]
	ldr	r3, [lr]
	bl	printk
	b	.L3743
.L3742:
	adds	r7, r7, #1
	uxth	r7, r7
	b	.L3737
.L3751:
	mov	r8, #0
.L3746:
	ldrh	r3, [r4, #1916]
	sxth	r6, r8
	ldr	r7, .L3754+4
	cmp	r6, r3
	bge	.L3749
	lsls	r6, r6, #1
	movs	r7, #0
.L3750:
	ldrh	r2, [r5, #2910]
	sxth	r3, r7
	adds	r7, r7, #1
	cmp	r3, r2
	bge	.L3753
	ldr	r2, [r4, #1832]
	movs	r1, #1
	ldr	r0, .L3754+12
	ldrh	r2, [r2, r6]
	orr	r3, r3, r2, lsl #10
	mov	r2, r1
	str	r3, [r4, #4060]
	bl	FlashReadPages
	ldr	r3, [r4, #4068]
	ldr	r1, [r4, #1832]
	ldr	r2, [r4, #4064]
	ldr	r0, [r3]
	ldrh	r1, [r1, r6]
	str	r0, [sp]
	ldr	r0, [r3, #4]
	str	r0, [sp, #4]
	ldr	r0, [r3, #8]
	str	r0, [sp, #8]
	ldr	r3, [r3, #12]
	ldr	r0, .L3754+16
	str	r3, [sp, #12]
	ldr	r3, [r2]
	str	r3, [sp, #16]
	ldr	r3, [r2, #4]
	ldr	r2, [r4, #4056]
	str	r3, [sp, #20]
	ldr	r3, [r4, #4060]
	bl	printk
	b	.L3750
.L3753:
	add	r8, r8, #1
	b	.L3746
.L3749:
	ldr	r1, [r7, #1832]
	movs	r2, #2
	ldr	r3, [r5, #2936]
	ldr	r0, .L3754+20
	bl	rknand_print_hex
	ldr	r1, [r7, #1852]
	ldrh	r3, [r5, #2944]
	movs	r2, #4
	ldr	r0, .L3754+24
	bl	rknand_print_hex
	ldr	r0, .L3754+28
	ldr	r1, [r7, #1856]
	movs	r2, #4
	ldrh	r3, [r5, #2944]
	add	sp, sp, #52
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	rknand_print_hex
.L3755:
	.align	2
.L3754:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC163
	.word	.LANCHOR2+4056
	.word	.LC110
	.word	.LC164
	.word	.LC165
	.word	.LC166
	.fnend
	.size	dump_map_info, .-dump_map_info
	.align	1
	.global	flash_boot_enter_slc_mode
	.thumb
	.thumb_func
	.type	flash_boot_enter_slc_mode, %function
flash_boot_enter_slc_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L3758
	ldr	r2, [r3, #2780]
	ldr	r3, .L3758+4
	cmp	r2, r3
	bne	.L3756
	b	flash_enter_slc_mode
.L3756:
	bx	lr
.L3759:
	.align	2
.L3758:
	.word	.LANCHOR0
	.word	1446522928
	.fnend
	.size	flash_boot_enter_slc_mode, .-flash_boot_enter_slc_mode
	.align	1
	.global	flash_boot_exit_slc_mode
	.thumb
	.thumb_func
	.type	flash_boot_exit_slc_mode, %function
flash_boot_exit_slc_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L3762
	ldr	r2, [r3, #2780]
	ldr	r3, .L3762+4
	cmp	r2, r3
	bne	.L3760
	b	flash_exit_slc_mode
.L3760:
	bx	lr
.L3763:
	.align	2
.L3762:
	.word	.LANCHOR0
	.word	1446522928
	.fnend
	.size	flash_boot_exit_slc_mode, .-flash_boot_exit_slc_mode
	.align	1
	.thumb
	.thumb_func
	.type	IdBlockReadData, %function
IdBlockReadData:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r6, r0
	ldr	r4, .L3772
	mov	r8, r1
	.pad #20
	sub	sp, sp, #20
	mov	r1, r6
	mov	fp, r2
	ldr	r0, .L3772+4
	ldr	r3, [r4, #1784]
	mov	r2, r8
	movs	r7, #0
	ldrb	r9, [r3, #9]	@ zero_extendqisi2
	ldr	r3, [r4, #4]
	smulbb	r5, r3, r9
	bl	printk
	mov	r0, r6
	uxth	r5, r5
	mov	r1, r5
	bl	__aeabi_uidivmod
	mul	ip, r9, r1
	mov	r10, r1
	subs	r3, r6, r1
	str	r3, [sp]
	ubfx	ip, ip, #2, #2
.L3765:
	cmp	r7, r8
	bcs	.L3771
	add	r2, r7, r10
	ldrb	r1, [r4, #1888]	@ zero_extendqisi2
	ubfx	r2, r2, #2, #16
	rsb	r5, ip, #4
	add	r3, r4, r2, lsl #1
	uxth	r5, r5
	ldrh	r3, [r3, #76]
	cbz	r1, .L3766
	ldr	r1, .L3772
	ldr	r0, .L3772+8
	ldr	r1, [r1, #2780]
	cmp	r1, r0
	it	eq
	moveq	r3, r2
.L3766:
	ldr	r2, [sp]
	add	r7, r7, r5
	add	r2, ip, r2
	ldrb	ip, [r4, #1889]	@ zero_extendqisi2
	uxth	r7, r7
	mla	r3, r9, r3, r2
	ldr	r2, [r4, #1784]
	str	ip, [sp, #4]
	ldrb	r1, [r2, #9]	@ zero_extendqisi2
	str	r3, [sp, #12]
	ldr	r3, .L3772+12
	str	r1, [sp, #8]
	ldrb	r0, [r3, #4018]	@ zero_extendqisi2
	bl	FlashBchSel
	movs	r0, #0
	bl	flash_boot_enter_slc_mode
	ldr	r3, [sp, #12]
	ldr	r1, [sp, #8]
	mov	r0, r3
	bl	__aeabi_uidiv
	mov	r2, fp
	add	fp, fp, r5, lsl #9
	mov	r1, r0
	movs	r0, #0
	mov	r3, r0
	bl	FlashReadPage
	movs	r0, #0
	bl	flash_boot_exit_slc_mode
	ldr	ip, [sp, #4]
	mov	r0, ip
	bl	FlashBchSel
	mov	ip, #0
	b	.L3765
.L3771:
	mov	r1, r6
	mov	r2, r8
	movs	r3, #0
	ldr	r0, .L3772+16
	bl	printk
	movs	r0, #0
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3773:
	.align	2
.L3772:
	.word	.LANCHOR0
	.word	.LC167
	.word	1446522928
	.word	.LANCHOR2
	.word	.LC168
	.fnend
	.size	IdBlockReadData, .-IdBlockReadData
	.align	1
	.global	write_idblock
	.thumb
	.thumb_func
	.type	write_idblock, %function
write_idblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 112
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r4, r0
	ldr	r7, .L3816
	.pad #124
	sub	sp, sp, #124
	mov	r0, #256000
	mov	r6, r1
	mov	r5, r2
	ldr	r3, [r7, #1784]
	ldr	r8, [r7, #4]
	ldrb	r10, [r3, #9]	@ zero_extendqisi2
	bl	ftl_malloc
	str	r0, [sp, #8]
	cmp	r0, #0
	beq	.L3798
	addw	r4, r4, #511
	lsrs	r4, r4, #9
	cmp	r4, #8
	bls	.L3796
	cmp	r4, #500
	bhi	.L3798
	b	.L3776
.L3796:
	movs	r4, #8
.L3776:
	ldr	r2, [r6]
	ldr	r3, .L3816+4
	cmp	r2, r3
	bne	.L3798
	smulbb	r3, r10, r8
	uxth	r9, r3
	add	r0, r9, #-1
	add	r0, r0, r4
	mov	r1, r9
	bl	__aeabi_uidiv
	movs	r3, #0
	movw	r2, #63871
	str	r0, [sp, #28]
	add	r0, r6, #254976
	add	r0, r0, #512
.L3780:
	ldr	r1, [r0, #-4]!
	cbnz	r1, .L3777
	ldr	r1, [r6, r3, lsl #2]
	adds	r3, r3, #1
	cmp	r3, #4096
	add	r2, r2, #-1
	it	hi
	movhi	r3, #0
	cmp	r2, #4096
	str	r1, [r0, #512]
	bne	.L3780
	b	.L3779
.L3777:
	ldr	r0, .L3816+8
	bl	printk
.L3779:
	mov	r1, r5
	movs	r3, #5
	ldr	r0, .L3816+12
	movs	r2, #4
	bl	rknand_print_hex
	ldrb	r2, [r7, #1]	@ zero_extendqisi2
	ldr	r1, [r6, #512]
	subs	r5, r5, #4
	ldr	r0, .L3816+16
	bl	printk
	ldr	r2, .L3816+20
	ldrh	r3, [r7, #1886]
	mov	r1, r4
	ldr	r0, .L3816+24
	ldr	r2, [r2, #4012]
	str	r2, [sp]
	mov	r2, r4
	bl	printk
	ldrb	r3, [r7, #1]	@ zero_extendqisi2
	ldr	r7, .L3816
	ldr	r2, [r6, #512]
	cmp	r2, r3
	it	hi
	strhi	r3, [r6, #512]
	lsls	r3, r4, #7
	str	r3, [sp, #32]
	movs	r3, #0
	str	r3, [sp, #16]
	str	r3, [sp, #12]
.L3794:
	ldr	r2, [r5, #4]
	ldrb	r3, [r7, #1]	@ zero_extendqisi2
	ldr	r10, .L3816
	cmp	r2, r3
	bcs	.L3782
	ldr	r3, .L3816+20
	ldr	r3, [r3, #4012]
	cmp	r2, r3
	bcc	.L3782
	ldr	r3, [sp, #28]
	cmp	r3, #1
	bls	.L3783
	ldr	r3, [sp, #12]
	cbz	r3, .L3783
	ldr	r3, [r5]
	adds	r3, r3, #1
	cmp	r2, r3
	beq	.L3782
.L3783:
	ldr	r0, [sp, #8]
	mov	r1, #512
	bl	__memzero
	ldr	r3, [r5, #4]
	ldr	r0, .L3816+28
	mul	r8, r3, r9
	ldr	r3, [r10, #1784]
	ldrb	r3, [r3, #9]	@ zero_extendqisi2
	mov	r1, r8
	str	r3, [sp, #20]
	ldr	r3, [r10, #4]
	ldrh	r2, [sp, #20]
	smulbb	r10, r3, r2
	mov	r2, r4
	bl	printk
	movs	r0, #0
	bl	flash_boot_enter_slc_mode
	mov	r0, r8
	uxth	r10, r10
	ldr	r1, [sp, #20]
	bl	__aeabi_uidiv
	mov	r1, r0
	movs	r0, #0
	mov	r2, r0
	bl	FlashEraseBlock
	cmp	r4, r10
	bls	.L3799
	movs	r0, #0
	add	r1, r8, r10
	mov	fp, #2
	mov	r2, r0
	bl	FlashEraseBlock
	b	.L3784
.L3799:
	mov	fp, #1
.L3784:
	movs	r0, #0
	bl	flash_boot_exit_slc_mode
	ldr	r3, [r7, #1784]
	ldrh	r0, [r3, #10]
	ldrb	r1, [r3, #12]	@ zero_extendqisi2
	lsls	r0, r0, #2
	mul	r0, fp, r0
	bl	__aeabi_idiv
	mov	r1, r10
	mov	r10, #0
	str	r0, [sp, #44]
	mov	r0, r8
	bl	__aeabi_uidivmod
	mov	ip, r1
	rsb	r3, r1, r8
	str	r6, [sp, #24]
	str	r3, [sp, #36]
.L3785:
	ldr	r3, [sp, #44]
	cmp	r10, r3
	bcs	.L3815
	add	r2, r10, ip
	ubfx	r2, r2, #2, #16
	cbz	r2, .L3786
	adds	r3, r2, #1
	add	r1, r7, r3, lsl #1
	ldrh	fp, [r1, #76]
	ldrb	r1, [r7, #1888]	@ zero_extendqisi2
	cbz	r1, .L3787
	ldr	r1, [r7, #2780]
	ldr	r0, .L3816+32
	cmp	r1, r0
	it	eq
	moveq	fp, r3
.L3787:
	add	fp, fp, #1073741824
	add	fp, fp, #-1
	lsl	r3, fp, #2
	str	r3, [sp, #56]
.L3786:
	movw	r3, #61424
	str	r3, [sp, #60]
	add	r3, r7, r2, lsl #1
	ldrh	fp, [r3, #76]
	ldrb	r3, [r7, #1888]	@ zero_extendqisi2
	cbz	r3, .L3788
	ldr	r3, [r7, #2780]
	ldr	r1, .L3816+32
	cmp	r3, r1
	it	eq
	moveq	fp, r2
.L3788:
	ldr	r3, [sp, #20]
	add	r10, r10, #4
	ldr	r2, [sp, #36]
	str	ip, [sp, #52]
	uxth	r10, r10
	mla	r2, r3, fp, r2
	ldr	r3, .L3816
	ldrb	r3, [r3, #1889]	@ zero_extendqisi2
	str	r2, [sp, #48]
	str	r3, [sp, #40]
	ldr	r3, .L3816+20
	ldrb	r0, [r3, #4018]	@ zero_extendqisi2
	bl	FlashBchSel
	movs	r0, #0
	bl	flash_boot_enter_slc_mode
	ldr	r3, .L3816
	ldr	r3, [r3, #1784]
	ldrb	r1, [r3, #9]	@ zero_extendqisi2
	ldr	r2, [sp, #48]
	mov	r0, r2
	bl	__aeabi_uidiv
	add	r3, sp, #56
	mov	r1, r0
	ldr	r2, [sp, #24]
	movs	r0, #0
	bl	FlashProgPage
	movs	r0, #0
	bl	flash_boot_exit_slc_mode
	ldr	r0, [sp, #40]
	bl	FlashBchSel
	ldr	r1, [sp, #20]
	ldr	r0, [sp, #36]
	bl	__aeabi_uidiv
	add	r2, fp, #1
	uxth	r2, r2
	mov	r1, r0
	movs	r0, #0
	bl	FlashPageProgMsbFFData
	ldr	r3, [sp, #24]
	ldr	ip, [sp, #52]
	add	r3, r3, #2048
	str	r3, [sp, #24]
	b	.L3785
.L3817:
	.align	2
.L3816:
	.word	.LANCHOR0
	.word	-52655045
	.word	.LC169
	.word	.LC170
	.word	.LC171
	.word	.LANCHOR2
	.word	.LC172
	.word	.LC173
	.word	1446522928
.L3815:
	mov	r1, r8
	mov	r2, r4
	movs	r3, #0
	ldr	r0, .L3818
	bl	printk
	ldr	r0, [r5, #4]
	mov	r1, r4
	mov	r8, #0
	mul	r0, r0, r9
	ldr	r2, [sp, #8]
	bl	IdBlockReadData
	mov	r1, r6
	ldr	r0, [sp, #8]
.L3793:
	mov	fp, r0
	mov	r10, r1
	ldr	r2, [fp]
	adds	r0, r0, #4
	ldr	r3, [r10]
	adds	r1, r1, #4
	cmp	r2, r3
	beq	.L3790
	ldr	r0, [sp, #8]
	mov	r1, #512
	bl	__memzero
	ldr	r3, [fp]
	ldr	r0, .L3818+4
	str	r3, [sp]
	ldr	r3, [r10]
	bic	r10, r8, #255
	ldr	r1, [sp, #12]
	lsl	r10, r10, #2
	str	r3, [sp, #4]
	mov	r3, r8
	ldr	r2, [r5, #4]
	bl	printk
	ldr	r0, .L3818+8
	add	r1, r6, r10
	movs	r2, #4
	mov	r3, #256
	bl	rknand_print_hex
	movs	r2, #4
	ldr	r0, .L3818+12
	ldr	r3, [sp, #8]
	add	r1, r3, r10
	mov	r3, #256
	bl	rknand_print_hex
	movs	r0, #0
	bl	flash_boot_enter_slc_mode
	ldr	r1, [r5, #4]
	movs	r0, #0
	mul	r1, r1, r9
	mov	r2, r0
	bl	FlashEraseBlock
	ldr	r3, [sp, #28]
	cmp	r3, #1
	bls	.L3791
	ldr	r1, [r5, #4]
	movs	r0, #0
	mov	r2, r0
	mla	r1, r1, r9, r9
	bl	FlashEraseBlock
.L3791:
	movs	r0, #0
	bl	flash_boot_exit_slc_mode
	ldr	r0, .L3818+16
	ldr	r1, [r5, #4]
	bl	printk
	ldr	r3, [sp, #32]
	cmp	r8, r3
	bcc	.L3782
	b	.L3792
.L3790:
	ldr	r3, [sp, #32]
	add	r8, r8, #1
	cmp	r8, r3
	bne	.L3793
.L3792:
	ldr	r3, [sp, #16]
	adds	r3, r3, #1
	str	r3, [sp, #16]
.L3782:
	ldr	r3, [sp, #12]
	adds	r5, r5, #4
	adds	r3, r3, #1
	str	r3, [sp, #12]
	cmp	r3, #5
	bne	.L3794
	ldr	r0, [sp, #8]
	bl	ftl_free
	ldr	r3, [sp, #16]
	clz	r0, r3
	lsrs	r0, r0, #5
	negs	r0, r0
	b	.L3775
.L3798:
	mov	r0, #-1
.L3775:
	add	sp, sp, #124
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3819:
	.align	2
.L3818:
	.word	.LC174
	.word	.LC175
	.word	.LC176
	.word	.LC177
	.word	.LC178
	.fnend
	.size	write_idblock, .-write_idblock
	.align	1
	.global	CRC_32
	.thumb
	.thumb_func
	.type	CRC_32, %function
CRC_32:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	add	r1, r1, r0
	ldr	r4, .L3824
	movs	r2, #0
.L3821:
	cmp	r0, r1
	beq	.L3823
	ldrb	r3, [r0], #1	@ zero_extendqisi2
	eor	r3, r3, r2, lsr #24
	add	r3, r4, r3, lsl #2
	ldr	r3, [r3, #3296]
	eor	r2, r3, r2, lsl #8
	b	.L3821
.L3823:
	mov	r0, r2
	pop	{r4, pc}
.L3825:
	.align	2
.L3824:
	.word	.LANCHOR1
	.fnend
	.size	CRC_32, .-CRC_32
	.align	1
	.global	write_loader_lba
	.thumb
	.thumb_func
	.type	write_loader_lba, %function
write_loader_lba:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r0, #64
	push	{r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	mov	r5, r0
	.pad #52
	sub	sp, sp, #52
	mov	r6, r1
	mov	r8, r2
	ldr	r4, .L3848
	bne	.L3827
	ldr	r2, [r2]
	ldr	r3, .L3848+4
	cmp	r2, r3
	bne	.L3827
	mov	r0, #256000
	movs	r3, #1
	strb	r3, [r4, #3820]
	bl	ftl_malloc
	mov	r1, #256000
	str	r0, [r4, #3824]
	bl	__memzero
	str	r5, [r4, #3828]
.L3827:
	str	r6, [sp]
	mov	r3, r5
	ldr	r0, .L3848+8
	ldr	r1, [r4, #3824]
	ldr	r2, [r8]
	bl	printk
	ldrb	r3, [r4, #3820]	@ zero_extendqisi2
	ldr	r9, .L3848
	cmp	r3, #0
	beq	.L3826
	sub	r0, r5, #64
	ldr	r7, [r9, #3824]
	cmp	r0, #500
	bcs	.L3829
	rsb	r2, r5, #564
	add	r0, r7, r0, lsl #9
	cmp	r2, r6
	mov	r1, r8
	it	cs
	movcs	r2, r6
	lsls	r2, r2, #9
	bl	ftl_memcpy
	b	.L3830
.L3829:
	cmp	r5, #564
	bcs	.L3838
.L3830:
	ldr	r3, [r4, #3828]
	cmp	r3, r5
	beq	.L3836
	ldr	r2, .L3848
	movs	r3, #0
	mov	r8, r3
	strb	r3, [r2, #3820]
	cbz	r7, .L3837
	mov	r0, r7
	bl	ftl_free
.L3837:
	str	r8, [r4, #3824]
.L3836:
	add	r5, r5, r6
	str	r5, [r4, #3828]
	b	.L3826
.L3838:
	ldr	r3, .L3848+12
	ldr	r0, [r9, #3828]
	ldr	r3, [r3, #1784]
	subs	r0, r0, #64
	cmp	r0, #500
	it	cs
	movcs	r0, #500
	ldrb	r3, [r3, #9]	@ zero_extendqisi2
	cmp	r3, #4
	beq	.L3839
	movs	r3, #2
	str	r3, [sp, #8]
	movs	r3, #3
	str	r3, [sp, #12]
	movs	r3, #4
	str	r3, [sp, #16]
	movs	r3, #5
	str	r3, [sp, #20]
	movs	r3, #6
	str	r3, [sp, #24]
	b	.L3832
.L3839:
	movs	r3, #0
.L3831:
	add	r1, sp, #8
	lsls	r2, r3, #1
	cmp	r0, #256
	it	ls
	movls	r2, r3
	str	r2, [r1, r3, lsl #2]
	adds	r3, r3, #1
	cmp	r3, #5
	bne	.L3831
.L3832:
	movw	r3, #63872
.L3835:
	ldr	r2, [r7, r3, lsl #2]
	cbz	r2, .L3833
	adds	r3, r3, #128
	lsls	r0, r3, #2
	b	.L3834
.L3833:
	subs	r3, r3, #1
	cmp	r3, #4096
	bne	.L3835
	lsls	r0, r0, #9
.L3834:
	mov	r1, r7
	add	r2, sp, #8
	bl	write_idblock
	ldr	r0, [r4, #3824]
	movs	r7, #0
	strb	r7, [r4, #3820]
	bl	ftl_free
	str	r7, [r4, #3824]
	b	.L3836
.L3826:
	add	sp, sp, #52
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, pc}
.L3849:
	.align	2
.L3848:
	.word	.LANCHOR5
	.word	-52655045
	.word	.LC179
	.word	.LANCHOR0
	.fnend
	.size	write_loader_lba, .-write_loader_lba
	.align	1
	.global	FtlWrite
	.thumb
	.thumb_func
	.type	FtlWrite, %function
FtlWrite:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r5, r3
	sub	r3, r1, #64
	mov	r7, r0
	cmp	r3, #1984
	mov	r4, r1
	mov	r6, r2
	bcs	.L3851
	cbnz	r0, .L3851
	mov	r0, r1
	mov	r1, r2
	mov	r2, r5
	bl	write_loader_lba
.L3851:
	mov	r0, r7
	mov	r1, r4
	mov	r2, r6
	mov	r3, r5
	pop	{r4, r5, r6, r7, r8, lr}
	b	ftl_write
	.fnend
	.size	FtlWrite, .-FtlWrite
	.align	1
	.global	rknand_sys_storage_ioctl
	.thumb
	.thumb_func
	.type	rknand_sys_storage_ioctl, %function
rknand_sys_storage_ioctl:
	.fnstart
	@ args = 0, pretend = 0, frame = 528
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L3960
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	cmp	r1, r3
	.pad #528
	sub	sp, sp, #528
	mov	r4, r1
	mov	r5, r2
	beq	.L3854
	bhi	.L3855
	subw	r3, r3, #3461
	cmp	r1, r3
	beq	.L3856
	bhi	.L3857
	subs	r3, r3, #125
	cmp	r1, r3
	beq	.L3858
	bhi	.L3859
	subs	r3, r3, #237
	cmp	r1, r3
	bne	.L3915
	bl	rknand_dev_flush
	b	.L3948
.L3859:
	ldr	r3, .L3960+4
	cmp	r1, r3
	beq	.L3861
	adds	r3, r3, #1
	cmp	r1, r3
	beq	.L3862
	b	.L3915
.L3857:
	ldr	r3, .L3960+8
	cmp	r1, r3
	beq	.L3863
	bhi	.L3864
	mvn	r1, #1960
	add	r3, r3, r1
	cmp	r4, r3
	beq	.L3865
	b	.L3915
.L3864:
	ldr	r3, .L3960+12
	cmp	r1, r3
	beq	.L3863
	adds	r3, r3, #10
	cmp	r1, r3
	beq	.L3863
	b	.L3915
.L3855:
	ldr	r3, .L3960+16
	cmp	r1, r3
	beq	.L3866
	bhi	.L3867
	subs	r3, r3, #78
	cmp	r1, r3
	beq	.L3868
	bcc	.L3869
	adds	r3, r3, #21
	cmp	r1, r3
	beq	.L3870
	adds	r3, r3, #56
	cmp	r1, r3
	beq	.L3871
	b	.L3915
.L3867:
	ldr	r3, .L3960+20
	cmp	r1, r3
	mov	r6, r3
	beq	.L3872
	bhi	.L3873
	mvn	r2, #956
	add	r3, r3, r2
	cmp	r1, r3
	beq	.L3874
	add	r3, r3, #956
	cmp	r1, r3
	beq	.L3875
	b	.L3915
.L3873:
	ldr	r3, .L3960+24
	cmp	r1, r3
	beq	.L3875
	adds	r3, r3, #1
	cmp	r1, r3
	beq	.L3872
	b	.L3915
.L3869:
	ldr	r0, .L3960+28
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	mov	r4, r0
	cbnz	r0, .L3876
.L3881:
	mvn	r0, #11
	b	.L3853
.L3876:
	mov	r1, r5
	mov	r2, #512
	bl	rk_copy_from_user
	cbnz	r0, .L3954
	ldr	r0, .L3960+32
	ldmia	r4, {r1, r2}
	bl	printk
	ldr	r3, [r4, #4]
	cmp	r3, #8
	str	r3, [sp, #4]
	bhi	.L3952
	bl	rknand_device_lock
	mov	r2, r4
	ldr	r0, [r4]
	ldr	r1, [sp, #4]
	bl	IdBlockReadData
	bl	rknand_device_unlock
	mov	r0, r5
	mov	r1, r4
	ldr	r2, [sp, #4]
	lsls	r2, r2, #9
	bl	rk_copy_to_user
	cmp	r0, #0
	beq	.L3947
	ldr	r0, .L3960+36
.L3950:
	bl	printk
.L3952:
	mov	r0, r4
.L3953:
	bl	ftl_free
.L3951:
	mvn	r0, #13
	b	.L3853
.L3868:
	ldr	r0, .L3960+40
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	mov	r4, r0
	cmp	r0, #0
	beq	.L3881
	mov	r1, r5
	mov	r2, #4096
	bl	rk_copy_from_user
	cbz	r0, .L3882
.L3954:
	ldr	r0, .L3960+44
	b	.L3950
.L3882:
	ldr	r5, .L3960+48
	ldr	r0, .L3960+52
	ldmia	r4, {r1, r2}
	bl	printk
	ldr	r3, [r5, #3832]
	mov	r6, r5
	cbnz	r3, .L3883
	mov	r0, #260096
	bl	ftl_malloc
	str	r0, [r5, #3832]
	cmp	r0, #0
	beq	.L3952
	mov	r1, #260096
	bl	__memzero
.L3883:
	ldr	r2, [r4, #4]
	movw	r3, #4088
	cmp	r2, r3
	bhi	.L3952
	ldr	r3, [r4]
	cmp	r3, #251904
	bhi	.L3952
	ldr	r0, [r6, #3832]
	add	r1, r4, #8
	add	r0, r0, r3
	bl	memcpy
.L3947:
	mov	r0, r4
	bl	ftl_free
.L3948:
	movs	r4, #0
	b	.L3880
.L3871:
	ldr	r0, .L3960+56
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	mov	r6, r0
	cmp	r0, #0
	beq	.L3881
	mov	r1, r5
	movs	r2, #28
	bl	rk_copy_from_user
	cbz	r0, .L3886
	ldr	r0, .L3960+44
	bl	printk
	b	.L3887
.L3886:
	ldmia	r6, {r1, r2}
	ldr	r0, .L3960+60
	bl	printk
	ldr	r1, [r6]
	cmp	r1, #256000
	bhi	.L3887
	ldr	r5, .L3960+48
	ldr	r0, [r5, #3832]
	cbz	r0, .L3887
	bl	CRC_32
	ldr	r3, [r6, #4]
	cmp	r3, r0
	beq	.L3889
	mov	r0, r6
	bl	ftl_free
	b	.L3916
.L3889:
	bl	rknand_device_lock
	ldr	r1, [r5, #3832]
	add	r2, r6, #8
	ldr	r0, [r6]
	bl	write_idblock
	movs	r4, #0
	bl	rknand_device_unlock
	ldr	r0, [r5, #3832]
	bl	ftl_free
	str	r4, [r5, #3832]
	mov	r0, r6
	b	.L3893
.L3887:
	mov	r0, r6
	b	.L3953
.L3870:
	ldr	r0, .L3960+64
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	mov	r6, r0
	cmp	r0, #0
	beq	.L3881
	bl	ftl_read_flash_info
	mov	r0, r5
	mov	r1, r6
	movs	r2, #11
	b	.L3958
.L3854:
	ldr	r0, .L3960+68
	bl	printk
	bl	nand_blk_add_whole_disk
	mov	r0, #4096
	bl	ftl_malloc
	mov	r6, r0
	cmp	r0, #0
	beq	.L3881
	movs	r1, #64
	bl	__memzero
	mov	r0, r5
	mov	r1, r6
	movs	r2, #64
	b	.L3958
.L3866:
	ldr	r0, .L3960+72
	bl	printk
	ldr	r3, .L3960+48
	movs	r2, #4
	mov	r0, r5
	ldr	r3, [r3, #3836]
	ldr	r3, [r3, #20]
	add	r1, sp, #4
	str	r3, [sp, #4]
.L3942:
	bl	rk_copy_to_user
	cmp	r0, #0
	bne	.L3951
	b	.L3948
.L3874:
	ldr	r0, .L3960+76
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	mov	r6, r0
	cmp	r0, #0
	beq	.L3881
	bl	rknand_device_lock
	mov	r1, #264
	movs	r2, #2
	mov	r3, r6
	movs	r0, #16
	bl	ftl_read
	bl	rknand_device_unlock
	mov	r2, #1024
	mov	r0, r5
	mov	r1, r6
.L3958:
	bl	rk_copy_to_user
	mov	r4, r0
	mov	r0, r6
	cmp	r4, #0
	bne	.L3953
.L3893:
	bl	ftl_free
	b	.L3880
.L3858:
	ldr	r0, .L3960+80
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	mov	r4, r0
	cbz	r0, .L3894
.L3899:
	ldr	r0, .L3960+44
	bl	printk
	b	.L3951
.L3894:
	ldr	r2, [sp, #8]
	ldr	r3, .L3960+84
	cmp	r2, r3
	beq	.L3895
.L3896:
	mov	r4, #-1
	b	.L3880
.L3961:
	.align	2
.L3960:
	.word	1074033155
	.word	1074029570
	.word	1074031656
	.word	1074031666
	.word	1074033235
	.word	1074034193
	.word	1074034194
	.word	.LC180
	.word	.LC182
	.word	.LC183
	.word	.LC184
	.word	.LC181
	.word	.LANCHOR5
	.word	.LC185
	.word	.LC186
	.word	.LC187
	.word	.LC188
	.word	.LC189
	.word	.LC190
	.word	.LC191
	.word	.LC192
	.word	1263358532
.L3895:
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L3896
	ldr	r6, .L3962
	mov	r2, #512
	add	r0, sp, #8
	ldr	r1, [r6, #3836]
	bl	memcpy
	ldr	r2, [r6, #3840]
	ldr	r3, .L3962+4
	cmp	r2, r3
	beq	.L3897
	add	r0, sp, #72
	movs	r1, #128
	str	r4, [sp, #16]
	str	r4, [sp, #20]
	bl	__memzero
.L3897:
	add	r0, sp, #264
	mov	r1, #256
	movs	r3, #0
	str	r3, [sp, #24]
	bl	__memzero
	b	.L3956
.L3861:
	ldr	r0, .L3962+8
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L3899
	ldr	r2, [sp, #8]
	ldr	r3, .L3962+12
	cmp	r2, r3
	bne	.L3896
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L3896
	ldr	r2, .L3962
	ldr	r3, .L3962+4
	ldr	r1, [r2, #3840]
	cmp	r1, r3
	bne	.L3916
	ldr	r3, [sp, #20]
	subs	r1, r3, #1
	cmp	r1, #127
	bhi	.L3917
	ldr	r4, [r2, #3836]
	add	r1, sp, #72
	add	r0, r4, #64
	str	r3, [r4, #12]
	ldr	r2, [sp, #20]
	bl	memcpy
	movs	r0, #1
	mov	r1, r4
	b	.L3949
.L3865:
	ldr	r0, .L3962+16
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L3899
	ldr	r2, [sp, #8]
	ldr	r3, .L3962+20
	cmp	r2, r3
	bne	.L3896
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L3896
	ldr	r5, .L3962
	ldr	r3, [r5, #3844]
	cbnz	r3, .L3900
.L3903:
	movs	r0, #0
	b	.L3853
.L3900:
	ldr	r3, [r5, #3848]
	ldr	r2, .L3962+24
	ldr	r1, [r3]
	cmp	r1, r2
	beq	.L3901
	str	r2, [r3]
	mov	r2, #504
	ldr	r3, [r5, #3848]
	str	r2, [r3, #4]
	movs	r2, #0
	str	r2, [r3, #8]
	str	r2, [r3, #12]
.L3901:
	ldr	r1, [r5, #3848]
	movs	r4, #0
	mov	r0, r4
	str	r4, [r1, #16]
	bl	StorageSysDataStore
	ldr	r3, [r5, #3836]
	ldr	r2, .L3962+12
	ldr	r1, [r3]
	cmp	r1, r2
	it	ne
	strne	r2, [r3]
	ldr	r0, [r5, #3836]
	itt	ne
	movne	r2, #504
	ldrne	r3, .L3962
	mov	r1, #128
	add	r0, r0, #64
	itt	ne
	ldrne	r3, [r3, #3836]
	strne	r4, [r3, #8]
	mov	r4, #0
	it	ne
	strne	r2, [r3, #4]
	str	r4, [r0, #-52]
	bl	__memzero
	movs	r0, #1
	ldr	r1, [r5, #3836]
	bl	StorageSysDataStore
	str	r4, [r5, #3844]
	str	r4, [r5, #3840]
	b	.L3880
.L3856:
	ldr	r0, .L3962+28
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L3899
	ldr	r2, [sp, #8]
	ldr	r3, .L3962+32
	cmp	r2, r3
	bne	.L3896
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L3896
	ldr	r5, .L3962
	ldr	r3, [r5, #3844]
	cmp	r3, #1
	beq	.L3903
	ldr	r3, [r5, #3848]
	ldr	r2, .L3962+24
	ldr	r1, [r3]
	cmp	r1, r2
	it	ne
	strne	r2, [r3]
	ldr	r1, [r5, #3848]
	itttt	ne
	movne	r2, #504
	ldrne	r3, [r5, #3848]
	strne	r2, [r3, #4]
	strne	r0, [r3, #8]
	it	ne
	strne	r0, [r3, #12]
	movs	r3, #1
	movs	r0, #0
	str	r3, [r1, #16]
	bl	StorageSysDataStore
	ldr	r3, [r5, #3836]
	ldr	r2, .L3962+12
	ldr	r1, [r3]
	cmp	r1, r2
	beq	.L3905
	str	r2, [r3]
	mov	r2, #504
	ldr	r3, .L3962
	ldr	r3, [r3, #3836]
	str	r2, [r3, #4]
	movs	r2, #0
	str	r2, [r3, #8]
.L3905:
	ldr	r0, [r5, #3836]
	movs	r1, #128
	movs	r4, #0
	adds	r0, r0, #64
	str	r4, [r0, #-52]
	bl	__memzero
	movs	r0, #1
	ldr	r1, [r5, #3836]
	bl	StorageSysDataStore
	movs	r3, #1
	str	r3, [r5, #3844]
	b	.L3880
.L3862:
	ldr	r0, .L3962+36
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L3899
	ldr	r2, [sp, #8]
	ldr	r3, .L3962+40
	cmp	r2, r3
	bne	.L3896
	ldr	r2, [sp, #12]
	cmp	r2, #512
	bhi	.L3896
	add	r0, sp, #16
	ldr	r1, .L3962+44
	b	.L3955
.L3863:
	ldr	r0, .L3962+48
	cmp	r4, r0
	mov	r6, r0
	bne	.L3906
	ldr	r0, .L3962+52
	b	.L3945
.L3906:
	ldr	r3, .L3962+56
	cmp	r4, r3
	ite	eq
	ldreq	r0, .L3962+60
	ldrne	r0, .L3962+64
.L3945:
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L3899
	ldr	r2, [sp, #8]
	ldr	r3, .L3962+68
	cmp	r2, r3
	bne	.L3951
	ldr	r3, .L3962+56
	cmp	r4, r3
	bne	.L3909
	ldr	r3, .L3962
	mov	r0, r5
	add	r1, sp, #8
	movs	r2, #16
	ldr	r3, [r3, #3836]
	ldr	r3, [r3, #20]
	str	r3, [sp, #12]
	strb	r3, [sp, #16]
	bl	rk_copy_to_user
	cmp	r0, #0
	beq	.L3853
	b	.L3951
.L3909:
	ldr	r5, .L3962+72
	ldr	r3, [r5, #20]
	mov	r0, r5
	cmp	r3, #10
	bhi	.L3951
	ldr	r3, .L3962
	ldr	r1, [r3, #3836]
	ldr	r3, [sp, #12]
	ldr	r2, [r1, #24]
	cmp	r2, r3
	beq	.L3910
	cbz	r2, .L3910
	mov	r1, r3
	ldr	r0, .L3962+76
	bl	printk
	ldr	r3, [r5, #20]
	adds	r3, r3, #1
	str	r3, [r5, #20]
	b	.L3951
.L3910:
	movs	r2, #0
	cmp	r4, r6
	str	r2, [r0, #20]
	ite	ne
	movne	r2, #1
	moveq	r3, r2
	mov	r0, #1
	it	eq
	moveq	r2, r3
	str	r3, [r1, #24]
	str	r2, [r1, #20]
	bl	StorageSysDataStore
	adds	r0, r0, #1
	bne	.L3948
	mvn	r4, #1
	b	.L3880
.L3963:
	.align	2
.L3962:
	.word	.LANCHOR5
	.word	-1067903959
	.word	.LC193
	.word	1263358532
	.word	.LC194
	.word	1112753220
	.word	1146313043
	.word	.LC195
	.word	1112755781
	.word	.LC196
	.word	1094995539
	.word	.LANCHOR5+3852
	.word	1074031666
	.word	.LC197
	.word	1074031676
	.word	.LC198
	.word	.LC199
	.word	1280262987
	.word	.LANCHOR6
	.word	.LC200
.L3875:
	ldr	r0, .L3964
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L3899
	ldr	r2, [sp, #8]
	ldr	r3, .L3964+4
	cmp	r2, r3
	bne	.L3896
	ldr	r2, [sp, #12]
	cmp	r2, #504
	bhi	.L3896
	ldr	r3, .L3964+8
	add	r0, sp, #16
	cmp	r4, r3
	ldr	r3, .L3964+12
	ite	eq
	ldreq	r1, [r3, #24]
	ldrne	r1, [r3, #28]
	adds	r1, r1, #8
.L3955:
	bl	memcpy
.L3956:
	mov	r0, r5
	add	r1, sp, #8
	mov	r2, #520
	b	.L3942
.L3872:
	ldr	r0, .L3964+16
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L3899
	ldr	r2, [sp, #8]
	ldr	r3, .L3964+4
	cmp	r2, r3
	bne	.L3896
	ldr	r2, [sp, #12]
	cmp	r2, #504
	bhi	.L3896
	cmp	r4, r6
	add	r2, r2, #8
	ldr	r4, .L3964+12
	bne	.L3914
	add	r1, sp, #8
	ldr	r0, [r4, #24]
	bl	memcpy
	movs	r0, #2
	ldr	r1, [r4, #24]
	b	.L3949
.L3914:
	add	r1, sp, #8
	ldr	r0, [r4, #28]
	bl	memcpy
	ldr	r1, [r4, #28]
	movs	r0, #3
.L3949:
	bl	StorageSysDataStore
	mov	r4, r0
.L3880:
	ldr	r0, .L3964+20
	mov	r1, r4
	bl	printk
	mov	r0, r4
	b	.L3853
.L3915:
	mvn	r0, #21
	b	.L3853
.L3916:
	mvn	r0, #1
	b	.L3853
.L3917:
	mvn	r0, #2
.L3853:
	add	sp, sp, #528
	@ sp needed
	pop	{r4, r5, r6, pc}
.L3965:
	.align	2
.L3964:
	.word	.LC201
	.word	1145980246
	.word	1074034192
	.word	.LANCHOR6
	.word	.LC202
	.word	.LC203
	.fnend
	.size	rknand_sys_storage_ioctl, .-rknand_sys_storage_ioctl
	.align	1
	.global	rk_ftl_storage_sys_init
	.thumb
	.thumb_func
	.type	rk_ftl_storage_sys_init, %function
rk_ftl_storage_sys_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r3, #-1
	ldr	r4, .L3975
	mov	r2, #512
	ldr	r5, .L3975+4
	movs	r6, #0
	addw	r0, r4, #3852
	ldr	r1, [r4, #3804]
	str	r3, [r4, #3828]
	add	r3, r1, #512
	str	r3, [r4, #3836]
	str	r1, [r4, #3848]
	add	r3, r1, #1024
	add	r1, r1, #1536
	str	r3, [r5, #24]
	strb	r6, [r4, #3820]
	str	r6, [r4, #3824]
	str	r6, [r4, #3832]
	str	r1, [r5, #28]
	bl	ftl_memcpy
	ldr	r7, [r4, #3848]
	str	r6, [r4, #3840]
	str	r6, [r5, #20]
	ldr	r3, [r7, #16]
	ldr	r8, [r7, #508]
	str	r3, [r4, #3844]
	cmp	r8, #0
	beq	.L3967
	mov	r0, r7
	mov	r1, #508
	bl	js_hash
	cmp	r8, r0
	beq	.L3967
	str	r6, [r7, #16]
	ldr	r0, .L3975+8
	str	r6, [r4, #3844]
	bl	printk
.L3967:
	ldr	r3, [r4, #3844]
	cbz	r3, .L3968
	ldr	r2, .L3975+12
	ldr	r3, .L3975
	str	r2, [r3, #3840]
.L3968:
	ldr	r1, [r5, #24]
	movs	r0, #2
	bl	StorageSysDataLoad
	ldr	r1, [r5, #28]
	movs	r0, #3
	bl	StorageSysDataLoad
	pop	{r4, r5, r6, r7, r8, lr}
	b	rknand_sys_storage_init
.L3976:
	.align	2
.L3975:
	.word	.LANCHOR5
	.word	.LANCHOR6
	.word	.LC204
	.word	-1067903959
	.fnend
	.size	rk_ftl_storage_sys_init, .-rk_ftl_storage_sys_init
	.align	1
	.global	StorageSysDataDeInit
	.thumb
	.thumb_func
	.type	StorageSysDataDeInit, %function
StorageSysDataDeInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	movs	r0, #0
	bx	lr
	.fnend
	.size	StorageSysDataDeInit, .-StorageSysDataDeInit
	.align	1
	.global	rk_ftl_vendor_storage_init
	.thumb
	.thumb_func
	.type	rk_ftl_vendor_storage_init, %function
rk_ftl_vendor_storage_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r0, #65536
	bl	ftl_malloc
	ldr	r5, .L3987
	str	r0, [r5, #32]
	cmp	r0, #0
	beq	.L3984
	movs	r7, #0
	ldr	r8, .L3987+8
	mov	r9, r5
	mov	r4, r7
	mov	r6, r7
.L3982:
	lsls	r0, r6, #7
	movs	r1, #128
	ldr	r2, [r5, #32]
	bl	FlashBootVendorRead
	cmp	r0, #0
	bne	.L3980
	ldr	r3, [r9, #32]
	ldr	r0, .L3987+4
	add	r2, r3, #65280
	adds	r2, r2, #252
	ldr	r1, [r3]
	ldr	r3, [r3, #4]
	ldr	r2, [r2]
	bl	printk
	ldr	r2, [r9, #32]
	ldr	r3, [r2]
	cmp	r3, r8
	bne	.L3981
	add	r3, r2, #65280
	ldr	r1, [r2, #4]
	adds	r3, r3, #252
	ldr	r3, [r3]
	cmp	r3, r1
	bne	.L3981
	cmp	r4, r3
	itt	cc
	movcc	r7, r6
	movcc	r4, r3
.L3981:
	cbnz	r6, .L3986
	movs	r6, #1
	b	.L3982
.L3986:
	cbz	r4, .L3983
	lsls	r0, r7, #7
	movs	r1, #128
	bl	FlashBootVendorRead
	cbnz	r0, .L3980
	pop	{r3, r4, r5, r6, r7, r8, r9, pc}
.L3983:
	mov	r0, r2
	mov	r1, #65536
	bl	__memzero
	ldr	r3, [r5, #32]
	ldr	r2, .L3987+8
	mov	r0, r4
	str	r6, [r3, #4]
	str	r2, [r3]
	movw	r2, #65532
	str	r6, [r3, r2]
	movw	r2, #64504
	strh	r4, [r3, #12]	@ movhi
	strh	r2, [r3, #14]	@ movhi
	pop	{r3, r4, r5, r6, r7, r8, r9, pc}
.L3980:
	ldr	r0, [r5, #32]
	bl	kfree
	movs	r3, #0
	mov	r0, #-1
	str	r3, [r5, #32]
	pop	{r3, r4, r5, r6, r7, r8, r9, pc}
.L3984:
	mvn	r0, #11
	pop	{r3, r4, r5, r6, r7, r8, r9, pc}
.L3988:
	.align	2
.L3987:
	.word	.LANCHOR6
	.word	.LC205
	.word	1380668996
	.fnend
	.size	rk_ftl_vendor_storage_init, .-rk_ftl_vendor_storage_init
	.align	1
	.global	rk_ftl_vendor_read
	.thumb
	.thumb_func
	.type	rk_ftl_vendor_read, %function
rk_ftl_vendor_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	ldr	r3, .L3995
	ldr	r5, [r3, #32]
	cbz	r5, .L3994
	ldrh	r7, [r5, #10]
	movs	r3, #0
.L3991:
	cmp	r3, r7
	bcs	.L3994
	add	r6, r5, r3, lsl #3
	ldrh	r4, [r6, #16]
	cmp	r4, r0
	bne	.L3992
	ldrh	r3, [r6, #20]
	mov	r0, r1
	cmp	r2, r3
	it	cs
	movcs	r2, r3
	ldrh	r3, [r6, #18]
	mov	r4, r2
	add	r3, r3, #1024
	adds	r1, r5, r3
	bl	memcpy
	mov	r0, r4
	pop	{r3, r4, r5, r6, r7, pc}
.L3992:
	adds	r3, r3, #1
	b	.L3991
.L3994:
	mov	r0, #-1
	pop	{r3, r4, r5, r6, r7, pc}
.L3996:
	.align	2
.L3995:
	.word	.LANCHOR6
	.fnend
	.size	rk_ftl_vendor_read, .-rk_ftl_vendor_read
	.align	1
	.global	rk_ftl_vendor_write
	.thumb
	.thumb_func
	.type	rk_ftl_vendor_write, %function
rk_ftl_vendor_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r9, r2
	ldr	r2, .L4019
	.pad #28
	sub	sp, sp, #28
	mov	ip, r0
	mov	r3, r1
	ldr	r4, [r2, #32]
	cmp	r4, #0
	beq	.L4012
	add	r7, r9, #63
	ldrh	r1, [r4, #8]
	ldrh	r2, [r4, #10]
	bic	r7, r7, #63
	movs	r6, #0
	str	r1, [sp, #4]
.L3999:
	cmp	r6, r2
	bcs	.L4017
	add	r5, r4, r6, lsl #3
	ldrh	r1, [r5, #16]
	cmp	r1, ip
	bne	.L4000
	ldrh	r1, [r5, #20]
	add	fp, r4, #1024
	adds	r1, r1, #63
	bic	r1, r1, #63
	str	r1, [sp, #8]
	cmp	r9, r1
	bls	.L4001
	ldrh	r1, [r4, #14]
	cmp	r1, r7
	bcc	.L4012
	ldrh	r8, [r5, #18]
	subs	r2, r2, #1
	str	r2, [sp, #12]
.L4002:
	ldr	r2, [sp, #12]
	adds	r5, r5, #8
	cmp	r6, r2
	bcs	.L4018
	ldrh	r2, [r5, #16]
	add	r0, fp, r8
	ldrh	r1, [r5, #18]
	adds	r6, r6, #1
	strh	r8, [r5, #10]	@ movhi
	strh	r2, [r5, #8]	@ movhi
	add	r1, r1, fp
	ldrh	r2, [r5, #20]
	str	r3, [sp, #20]
	add	r10, r2, #63
	str	ip, [sp, #16]
	bic	r10, r10, #63
	strh	r2, [r5, #12]	@ movhi
	add	r8, r8, r10
	mov	r2, r10
	bl	memcpy
	ldr	r3, [sp, #20]
	ldr	ip, [sp, #16]
	b	.L4002
.L4018:
	add	r6, r4, r6, lsl #3
	uxth	r8, r8
	add	r0, fp, r8
	mov	r1, r3
	strh	r8, [r6, #18]	@ movhi
	mov	r2, r9
	strh	ip, [r6, #16]	@ movhi
	uxth	r7, r7
	strh	r9, [r6, #20]	@ movhi
	add	r8, r8, r7
	bl	memcpy
	ldrh	r5, [r4, #14]
	strh	r8, [r4, #12]	@ movhi
	ldr	r3, [sp, #8]
	add	r3, r3, r5
	subs	r7, r3, r7
	strh	r7, [r4, #14]	@ movhi
	b	.L4016
.L4001:
	ldrh	r0, [r5, #18]
	mov	r1, r3
	mov	r2, r9
	add	r0, r0, fp
	bl	memcpy
	strh	r9, [r5, #20]	@ movhi
	b	.L4016
.L4000:
	adds	r6, r6, #1
	b	.L3999
.L4017:
	ldrh	r1, [r4, #14]
	cmp	r1, r7
	bcc	.L4012
	add	r2, r4, r2, lsl #3
	uxth	r7, r7
	subs	r1, r1, r7
	strh	ip, [r2, #16]	@ movhi
	ldrh	r0, [r4, #12]
	strh	r9, [r2, #20]	@ movhi
	strh	r0, [r2, #18]	@ movhi
	add	r0, r0, r7
	strh	r1, [r4, #14]	@ movhi
	mov	r1, r3
	strh	r0, [r4, #12]	@ movhi
	ldrh	r0, [r2, #18]
	mov	r2, r9
	add	r0, r0, #1024
	add	r0, r0, r4
	bl	memcpy
	ldrh	r3, [r4, #10]
	adds	r3, r3, #1
	strh	r3, [r4, #10]	@ movhi
.L4016:
	ldr	r3, [r4, #4]
	movw	r2, #65532
	movs	r1, #128
	adds	r3, r3, #1
	str	r3, [r4, #4]
	str	r3, [r4, r2]
	mov	r2, r4
	ldrh	r3, [r4, #8]
	adds	r3, r3, #1
	uxth	r3, r3
	cmp	r3, #1
	it	hi
	movhi	r3, #0
	strh	r3, [r4, #8]	@ movhi
	ldr	r3, [sp, #4]
	lsls	r0, r3, #7
	bl	FlashBootVendorWrite
	movs	r0, #0
	b	.L3998
.L4012:
	mov	r0, #-1
.L3998:
	add	sp, sp, #28
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L4020:
	.align	2
.L4019:
	.word	.LANCHOR6
	.fnend
	.size	rk_ftl_vendor_write, .-rk_ftl_vendor_write
	.align	1
	.global	rk_ftl_vendor_storage_ioctl
	.thumb
	.thumb_func
	.type	rk_ftl_vendor_storage_ioctl, %function
rk_ftl_vendor_storage_ioctl:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r0, #4096
	mov	r6, r1
	mov	r5, r2
	bl	ftl_malloc
	mov	r4, r0
	cmp	r0, #0
	beq	.L4030
	ldr	r3, .L4037
	cmp	r6, r3
	beq	.L4024
	adds	r3, r3, #1
	cmp	r6, r3
	beq	.L4025
	b	.L4036
.L4024:
	mov	r1, r5
	movs	r2, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4036
	ldr	r2, [r4]
	ldr	r3, .L4037+4
	cmp	r2, r3
	beq	.L4027
.L4028:
	mov	r5, #-1
	b	.L4023
.L4027:
	ldrh	r0, [r4, #4]
	add	r1, r4, #8
	ldrh	r2, [r4, #6]
	bl	rk_ftl_vendor_read
	adds	r3, r0, #1
	beq	.L4028
	uxth	r2, r0
	strh	r0, [r4, #6]	@ movhi
	mov	r1, r4
	mov	r0, r5
	adds	r2, r2, #8
	bl	rk_copy_to_user
	cmp	r0, #0
	ite	eq
	moveq	r5, #0
	mvnne	r5, #13
	b	.L4023
.L4025:
	mov	r1, r5
	movs	r2, #8
	bl	rk_copy_from_user
	cbnz	r0, .L4036
	ldr	r2, [r4]
	ldr	r3, .L4037+4
	cmp	r2, r3
	bne	.L4028
	ldrh	r2, [r4, #6]
	movw	r3, #4087
	cmp	r2, r3
	bhi	.L4028
	mov	r0, r4
	mov	r1, r5
	adds	r2, r2, #8
	bl	rk_copy_from_user
	cbnz	r0, .L4036
	ldrh	r0, [r4, #4]
	add	r1, r4, #8
	ldrh	r2, [r4, #6]
	bl	rk_ftl_vendor_write
	mov	r5, r0
	b	.L4023
.L4036:
	mvn	r5, #13
.L4023:
	mov	r0, r4
	bl	kfree
	b	.L4022
.L4030:
	mov	r5, #-1
.L4022:
	mov	r0, r5
	pop	{r4, r5, r6, pc}
.L4038:
	.align	2
.L4037:
	.word	1074034177
	.word	1448232273
	.fnend
	.size	rk_ftl_vendor_storage_ioctl, .-rk_ftl_vendor_storage_ioctl
	.global	gTable_Crc32
	.global	SecureBootUnlockTryCount
	.global	SecureBootCheckOK
	.global	SecureBootEn
	.global	gpVendor1Info
	.global	gpVendor0Info
	.global	g_idb_buffer
	.global	gSnSectorData
	.global	gpDrmKeyInfo
	.global	gpBootConfig
	.global	gLoaderBootInfo
	.global	RK29_NANDC1_REG_BASE
	.global	RK29_NANDC_REG_BASE
	.global	gc_ink_free_return_value
	.global	check_valid_page_count_table
	.global	FtlUpdateVaildLpnCount
	.global	g_ect_tbl_power_up_flush
	.global	last_cache_match_count
	.global	power_up_flag
	.global	g_LowFormat
	.global	gFtlInitStatus
	.global	DeviceCapacity
	.global	ToshibaRefValue
	.global	Toshiba15RefValue
	.global	ToshibaA19RefValue
	.global	SamsungRefValue
	.global	refValueDefault
	.global	FbbtBlk
	.global	random_seed
	.global	gSlcNandParaInfo
	.global	gNandParaInfo
	.global	g_page_map_check_enable
	.global	g_power_lost_ecc_error_blk
	.global	g_power_lost_recovery_flag
	.global	c_mlc_erase_count_value
	.global	g_recovery_ppa_tbl
	.global	g_recovery_page_min_ver
	.global	g_recovery_page_num
	.global	g_cur_erase_blk
	.global	g_gc_skip_write_count
	.global	g_gc_head_data_block_count
	.global	g_gc_head_data_block
	.global	g_ftl_nand_free_count
	.global	g_in_swl_replace
	.global	g_in_gc_progress
	.global	g_all_blk_used_slc_mode
	.global	g_max_erase_count
	.global	g_totle_sys_slc_erase_count
	.global	g_totle_slc_erase_count
	.global	g_min_erase_count
	.global	g_totle_avg_erase_count
	.global	g_totle_mlc_erase_count
	.global	g_totle_l2p_write_count
	.global	g_totle_cache_write_count
	.global	g_tmp_data_superblock_id
	.global	g_totle_read_page_count
	.global	g_totle_discard_page_count
	.global	g_totle_read_sector
	.global	g_totle_write_sector
	.global	g_totle_write_page_count
	.global	g_totle_gc_page_count
	.global	g_gc_blk_index
	.global	g_gc_merge_free_blk_threshold
	.global	g_gc_free_blk_threshold
	.global	g_gc_refresh_block_temp_tbl
	.global	g_free_slc_blk_num
	.global	g_gc_refresh_block_temp_num
	.global	g_gc_bad_block_temp_tbl
	.global	g_gc_bad_block_gc_index
	.global	g_gc_bad_block_temp_num
	.global	g_gc_next_blk_3
	.global	g_gc_next_blk_2
	.global	g_gc_next_blk_1
	.global	g_gc_next_blk
	.global	g_gc_cur_blk_max_valid_pages
	.global	g_gc_cur_blk_valid_pages
	.global	g_gc_page_offset
	.global	g_gc_blk_num
	.global	p_gc_blk_tbl
	.global	p_gc_page_info
	.global	g_sys_ext_data
	.global	g_sys_save_data
	.global	gp_last_act_superblock
	.global	g_gc_superblock
	.global	g_gc_temp_superblock
	.global	g_buffer_superblock
	.global	g_active_superblock
	.global	g_num_data_superblocks
	.global	g_num_free_superblocks
	.global	p_data_block_list_tail
	.global	p_data_block_list_head
	.global	p_free_data_block_list_head
	.global	p_data_block_list_table
	.global	g_l2p_last_update_region_id
	.global	p_l2p_map_buf
	.global	p_l2p_ram_map
	.global	g_totle_vendor_block
	.global	p_vendor_region_ppn_table
	.global	p_vendor_block_ver_table
	.global	p_vendor_block_valid_page_count
	.global	p_vendor_block_table
	.global	g_totle_map_block
	.global	p_map_region_ppn_check_table
	.global	p_map_region_ppn_table
	.global	p_map_block_ver_table
	.global	p_map_block_valid_page_count
	.global	p_map_block_table
	.global	p_blk_mode_table
	.global	p_valid_page_count_check_table
	.global	p_valid_page_count_table
	.global	g_totle_swl_count
	.global	p_swl_mul_table
	.global	p_erase_count_table
	.global	g_ect_tbl_info_size
	.global	gp_ect_tbl_info
	.global	g_gc_num_req
	.global	c_gc_page_buf_num
	.global	gp_gc_page_buf_info
	.global	p_gc_data_buf
	.global	p_gc_spare_buf
	.global	p_io_spare_buf
	.global	p_io_data_buf_1
	.global	p_io_data_buf_0
	.global	p_sys_spare_buf
	.global	p_vendor_data_buf
	.global	p_sys_data_buf_1
	.global	p_sys_data_buf
	.global	g_wr_page_num
	.global	req_wr_io
	.global	c_wr_page_buf_num
	.global	p_wr_io_data_buf
	.global	p_wr_io_spare_buf
	.global	p_plane_order_table
	.global	g_req_cache
	.global	req_gc_dst
	.global	req_gc
	.global	req_erase
	.global	req_prgm
	.global	req_read
	.global	req_sys
	.global	gVendorBlkInfo
	.global	gL2pMapInfo
	.global	gSysFreeQueue
	.global	gSysInfo
	.global	gBbtInfo
	.global	g_flash_read_only_en
	.global	g_inkDie_check_enable
	.global	g_SlcPartLbaEndSector
	.global	g_MaxLbn
	.global	g_VaildLpn
	.global	g_MaxLpn
	.global	g_MaxLbaSector
	.global	g_GlobalDataVersion
	.global	g_GlobalSysVersion
	.global	ftl_gc_temp_power_lost_recovery_flag
	.global	c_ftl_nand_max_data_blks
	.global	c_ftl_nand_data_op_blks_per_plane
	.global	c_ftl_nand_data_blks_per_plane
	.global	c_ftl_nand_max_sys_blks
	.global	c_ftl_nand_init_sys_blks_per_plane
	.global	c_ftl_nand_sys_blks_per_plane
	.global	c_ftl_vendor_part_size
	.global	c_ftl_nand_max_vendor_blks
	.global	c_ftl_nand_max_map_blks
	.global	c_ftl_nand_map_blks_per_plane
	.global	c_ftl_nand_vendor_region_num
	.global	c_ftl_nand_l2pmap_ram_region_num
	.global	c_ftl_nand_map_region_num
	.global	c_ftl_nand_totle_phy_blks
	.global	c_ftl_nand_reserved_blks
	.global	c_ftl_nand_byte_pre_oob
	.global	c_ftl_nand_byte_pre_page
	.global	c_ftl_nand_sec_pre_page_shift
	.global	c_ftl_nand_sec_pre_page
	.global	c_ftl_nand_page_pre_super_blk
	.global	c_ftl_nand_page_pre_slc_blk
	.global	c_ftl_nand_page_pre_blk
	.global	c_ftl_nand_bbm_buf_size
	.global	c_ftl_nand_ext_blk_pre_plane
	.global	c_ftl_nand_blk_pre_plane
	.global	c_ftl_nand_planes_num
	.global	c_ftl_nand_blks_per_die
	.global	c_ftl_nand_planes_per_die
	.global	c_ftl_nand_die_num
	.global	c_ftl_nand_type
	.global	gMasterTempBuf
	.global	gMasterInfo
	.global	gNandcDumpWriteEn
	.global	gToggleModeClkDiv
	.global	gBootDdrMode
	.global	gNandcEccBits
	.global	gpNandc1
	.global	gpNandc
	.global	g_nandc_version_data
	.global	gNandcVer
	.global	gNandChipMap
	.global	gNandIDataBuf
	.global	idb_flash_slc_mode
	.global	FlashDdrTunningReadCount
	.global	FlashWaitBusyScheduleEn
	.global	gNandPhyInfo
	.global	gFlashProgCheckSpareBuffer
	.global	gFlashProgCheckBuffer
	.global	gFlashSpareBuffer
	.global	gFlashPageBuffer1
	.global	gFlashPageBuffer0
	.global	gpFlashSaveInfo
	.global	gReadRetryInfo
	.global	gpNandParaInfo
	.global	gNandOptPara
	.global	g_nand_ecc_en
	.global	g_slc2KBNand
	.global	g_maxRetryCount
	.global	g_maxRegNum
	.global	g_retryMode
	.global	gNandIDBResBlkNumSaveInFlash
	.global	gNandIDBResBlkNum
	.global	gNandFlashResEndPageAddr
	.global	gNandFlashInfoBlockAddr
	.global	gNandFlashIdbBlockAddr
	.global	gNandFlashInfoBlockEcc
	.global	gNandFlashIDBEccBits
	.global	gNandFlashEccBits
	.global	gNandRandomizer
	.global	gBlockPageAlignSize
	.global	gTotleBlock
	.global	gNandMaxChip
	.global	gNandMaxDie
	.global	gFlashInterfaceMode
	.global	gFlashSlcMode
	.global	gFlashOnfiModeEn
	.global	gFlashToggleModeEn
	.global	gFlashSdrModeEn
	.global	gMultiPageProgEn
	.global	gMultiPageReadEn
	.global	gpReadRetrial
	.global	mlcPageToSlcPageTbl
	.global	slcPageToMlcPageTbl
	.global	DieAddrs
	.global	gDieOp
	.global	DieCsIndex
	.global	IDByte
	.global	read_retry_cur_offset
	.section	.rodata
	.align	2
.LANCHOR3 = . + 0
	.type	__func__.21456, %object
	.size	__func__.21456, 11
__func__.21456:
	.ascii	"FtlMemInit\000"
.LC0:
	.byte	60
	.byte	40
	.byte	24
	.byte	16
	.space	1
	.type	__func__.22234, %object
	.size	__func__.22234, 17
__func__.22234:
	.ascii	"FtlDumpBlockInfo\000"
	.space	3
	.type	__func__.22253, %object
	.size	__func__.22253, 16
__func__.22253:
	.ascii	"FtlScanAllBlock\000"
	.type	__func__.22519, %object
	.size	__func__.22519, 17
__func__.22519:
	.ascii	"ftl_scan_all_ppa\000"
	.space	3
	.type	__func__.22199, %object
	.size	__func__.22199, 12
__func__.22199:
	.ascii	"FtlCheckVpc\000"
	.type	__func__.22501, %object
	.size	__func__.22501, 21
__func__.22501:
	.ascii	"FtlVpcCheckAndModify\000"
	.space	3
	.type	__func__.21529, %object
	.size	__func__.21529, 8
__func__.21529:
	.ascii	"FtlInit\000"
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC1:
	.ascii	"FlashEraseBlocks pageAddr error %x\012\000"
.LC2:
	.ascii	"phyBlk = 0x%x die = %d block_in_die = 0x%x 0x%8x\012"
	.ascii	"\000"
.LC3:
	.ascii	"FtlFreeSysBlkQueueOut free count = %d\012\000"
.LC4:
	.ascii	"FtlFreeSysBlkQueueOut = %x, free count = %d, error\012"
	.ascii	"\000"
.LC5:
	.ascii	"FtlFreeSysBlkQueueOut = %x, free count = %d\012\000"
.LC6:
	.ascii	"FLASH INFO:\012\000"
.LC7:
	.ascii	"FLASH ID: %x\012\000"
.LC8:
	.ascii	"Device Capacity: %d MB\012\000"
.LC9:
	.ascii	"FMWAIT: %x %x %x %x\012\000"
.LC10:
	.ascii	"FTL INFO:\012\000"
.LC11:
	.ascii	"g_MaxLpn = 0x%x\012\000"
.LC12:
	.ascii	"g_VaildLpn = 0x%x\012\000"
.LC13:
	.ascii	"read_page_count = 0x%x\012\000"
.LC14:
	.ascii	"discard_page_count = 0x%x\012\000"
.LC15:
	.ascii	"write_page_count = 0x%x\012\000"
.LC16:
	.ascii	"cache_write_count = 0x%x\012\000"
.LC17:
	.ascii	"l2p_write_count = 0x%x\012\000"
.LC18:
	.ascii	"gc_page_count = 0x%x\012\000"
.LC19:
	.ascii	"totle_write = %d MB\012\000"
.LC20:
	.ascii	"totle_read = %d MB\012\000"
.LC21:
	.ascii	"GSV = 0x%x\012\000"
.LC22:
	.ascii	"GDV = 0x%x\012\000"
.LC23:
	.ascii	"bad blk num = %d %d\012\000"
.LC24:
	.ascii	"free_superblocks = 0x%x\012\000"
.LC25:
	.ascii	"mlc_EC = 0x%x\012\000"
.LC26:
	.ascii	"slc_EC = 0x%x\012\000"
.LC27:
	.ascii	"avg_EC = 0x%x\012\000"
.LC28:
	.ascii	"sys_EC = 0x%x\012\000"
.LC29:
	.ascii	"max_EC = 0x%x\012\000"
.LC30:
	.ascii	"min_EC = 0x%x\012\000"
.LC31:
	.ascii	"PLT = 0x%x\012\000"
.LC32:
	.ascii	"POT = 0x%x\012\000"
.LC33:
	.ascii	"MaxSector = 0x%x\012\000"
.LC34:
	.ascii	"init_sys_blks_pp = 0x%x\012\000"
.LC35:
	.ascii	"sys_blks_pp = 0x%x\012\000"
.LC36:
	.ascii	"free sysblock = 0x%x\012\000"
.LC37:
	.ascii	"data_blks_pp = 0x%x\012\000"
.LC38:
	.ascii	"data_op_blks_pp = 0x%x\012\000"
.LC39:
	.ascii	"max_data_blks = 0x%x\012\000"
.LC40:
	.ascii	"Sys.id = 0x%x\012\000"
.LC41:
	.ascii	"Bbt.id = 0x%x\012\000"
.LC42:
	.ascii	"ACT.page = 0x%x\012\000"
.LC43:
	.ascii	"ACT.plane = 0x%x\012\000"
.LC44:
	.ascii	"ACT.id = 0x%x\012\000"
.LC45:
	.ascii	"ACT.mode = 0x%x\012\000"
.LC46:
	.ascii	"ACT.a_pages = 0x%x\012\000"
.LC47:
	.ascii	"ACT VPC = 0x%x\012\000"
.LC48:
	.ascii	"BUF.page = 0x%x\012\000"
.LC49:
	.ascii	"BUF.plane = 0x%x\012\000"
.LC50:
	.ascii	"BUF.id = 0x%x\012\000"
.LC51:
	.ascii	"BUF.mode = 0x%x\012\000"
.LC52:
	.ascii	"BUF.a_pages = 0x%x\012\000"
.LC53:
	.ascii	"BUF VPC = 0x%x\012\000"
.LC54:
	.ascii	"TMP.page = 0x%x\012\000"
.LC55:
	.ascii	"TMP.plane = 0x%x\012\000"
.LC56:
	.ascii	"TMP.id = 0x%x\012\000"
.LC57:
	.ascii	"TMP.mode = 0x%x\012\000"
.LC58:
	.ascii	"TMP.a_pages = 0x%x\012\000"
.LC59:
	.ascii	"GC.page = 0x%x\012\000"
.LC60:
	.ascii	"GC.plane = 0x%x\012\000"
.LC61:
	.ascii	"GC.id = 0x%x\012\000"
.LC62:
	.ascii	"GC.mode = 0x%x\012\000"
.LC63:
	.ascii	"GC.a_pages = 0x%x\012\000"
.LC64:
	.ascii	"WR_CHK = 0x%x %x %x %x\012\000"
.LC65:
	.ascii	"Read Err = 0x%x\012\000"
.LC66:
	.ascii	"Prog Err = 0x%x\012\000"
.LC67:
	.ascii	"gc_free_blk_th= 0x%x\012\000"
.LC68:
	.ascii	"gc_merge_free_blk_th= 0x%x\012\000"
.LC69:
	.ascii	"gc_skip_write_count= 0x%x\012\000"
.LC70:
	.ascii	"gc_blk_index= 0x%x\012\000"
.LC71:
	.ascii	"free min EC= 0x%x\012\000"
.LC72:
	.ascii	"free max EC= 0x%x\012\000"
.LC73:
	.ascii	"GC__SB VPC = 0x%x\012\000"
.LC74:
	.ascii	"%d. [0x%x]=0x%x 0x%x  0x%x\012\000"
.LC75:
	.ascii	"free %d. [0x%x] 0x%x  0x%x\012\000"
.LC76:
	.ascii	"%s\012\000"
.LC77:
	.ascii	"FTL version: 5.0.56 20190603\000"
.LC78:
	.ascii	"swblk %x ,avg = %x max= %x vpc= %x,ec=%x ,max ec=%x"
	.ascii	"\012\000"
.LC79:
	.ascii	"FtlGcRefreshBlock  0x%x\012\000"
.LC80:
	.ascii	"FtlGcMarkBadPhyBlk %d 0x%x\012\000"
.LC81:
	.ascii	"%s error allocating memory. return -1\012\000"
.LC82:
	.ascii	"%s %p:0x%x:\000"
.LC83:
	.ascii	"%x \000"
.LC84:
	.ascii	"\000"
.LC85:
	.ascii	"otp error! %d\000"
.LC86:
	.ascii	"rr\000"
.LC87:
	.ascii	"%d statReg->V6.mtrans_cnt=%d flReg.V6.page_num=%d\012"
	.ascii	"\000"
.LC88:
	.ascii	"nandc:\000"
.LC89:
	.ascii	"%d flReg.d32=%x %x\012\000"
.LC90:
	.ascii	"sdr read ok %x ecc=%d\012\000"
.LC91:
	.ascii	"sync para %d\012\000"
.LC92:
	.ascii	"TOG mode Read error %x %x\012\000"
.LC93:
	.ascii	"read retry status %x %x %x\012\000"
.LC94:
	.ascii	"micron RR %d row=%x,count %d,status=%d\012\000"
.LC95:
	.ascii	"ECC:%d\012\000"
.LC96:
	.ascii	"No.%d FLASH ID:%x %x %x %x %x %x\012\000"
.LC97:
	.ascii	"FlashLoadPhyInfo fail %x!!\012\000"
.LC98:
	.ascii	"Read pageadd=%x  ecc=%x err=%x\012\000"
.LC99:
	.ascii	"data:\000"
.LC100:
	.ascii	"spare:\000"
.LC101:
	.ascii	"ReadRetry pageadd=%x ecc=%x err=%x\012\000"
.LC102:
	.ascii	"FLFB:%d %d\012\000"
.LC103:
	.ascii	"prog error: = %x\012\000"
.LC104:
	.ascii	"prog read error: = %x\012\000"
.LC105:
	.ascii	"prog read s error: = %x %x %x\012\000"
.LC106:
	.ascii	"prog read d error: = %x %x %x\012\000"
.LC107:
	.ascii	"...%s enter...\012\000"
.LC108:
	.ascii	"superBlkID = %x vpc=%x\012\000"
.LC109:
	.ascii	"flashmode = %x pagenum = %x %x\012\000"
.LC110:
	.ascii	"id = %x,%x addr= %x,spare= %x %x %x %x data=%x %x\012"
	.ascii	"\000"
.LC111:
	.ascii	"blk = %x vpc=%x mode = %x\012\000"
.LC112:
	.ascii	"mlc id = %x,%x addr= %x,spare= %x %x %x %x data=%x "
	.ascii	"%x\012\000"
.LC113:
	.ascii	"slc id = %x,%x addr= %x,spare= %x %x %x %x data=%x "
	.ascii	"%x\012\000"
.LC114:
	.ascii	"ftl_scan_all_ppa blk %x page %x\012\000"
.LC115:
	.ascii	"addr= %x, status= %d,spare= %x %x %x %x data=%x %x\012"
	.ascii	"\000"
.LC116:
	.ascii	"%s finished\012\000"
.LC117:
	.ascii	"FlashMakeFactorBbt %d\012\000"
.LC118:
	.ascii	"bad block:%d %d\012\000"
.LC119:
	.ascii	"FMFB:%d %d\012\000"
.LC120:
	.ascii	"E:bad block:%d\012\000"
.LC121:
	.ascii	"FMFB:Save %d %d\012\000"
.LC122:
	.ascii	"FtlBbmTblFlush id=%x,page=%x,previd=%x cnt=%d\012\000"
.LC123:
	.ascii	"FtlBbmTblFlush error:%x\012\000"
.LC124:
	.ascii	"FtlBbmTblFlush error = %x error count = %d\012\000"
.LC125:
	.ascii	"FtlGcFreeBadSuperBlk 0x%x\012\000"
.LC126:
	.ascii	"decrement_vpc_count %x = %d\012\000"
.LC127:
	.ascii	"decrement_vpc_count %x = %d in free list\012\000"
.LC128:
	.ascii	"FtlVpcTblFlush error = %x error count = %d\012\000"
.LC129:
	.ascii	"page map lost: %x %x\012\000"
.LC130:
	.ascii	"FtlMapWritePage error = %x\012\000"
.LC131:
	.ascii	"FtlMapWritePage error = %x error count = %d\012\000"
.LC132:
	.ascii	"FtlVendorPartRead refresh = %x phyAddr = %x\012\000"
.LC133:
	.ascii	"no ect\000"
.LC134:
	.ascii	"slc mode\000"
.LC135:
	.ascii	"BBT:\000"
.LC136:
	.ascii	"region_id = %x phyAddr = %x\012\000"
.LC137:
	.ascii	"map_ppn:\000"
.LC138:
	.ascii	"load_l2p_region refresh = %x phyAddr = %x\012\000"
.LC139:
	.ascii	"FtlCheckVpc2 %x = %x  %x\012\000"
.LC140:
	.ascii	"free blk vpc error %x = %x  %x\012\000"
.LC141:
	.ascii	"Ftlscanalldata = %x\012\000"
.LC142:
	.ascii	"scan lpa = %x ppa= %x\012\000"
.LC143:
	.ascii	"lba = %x,addr= %x,spare= %x %x %x %x data=%x %x\012"
	.ascii	"\000"
.LC144:
	.ascii	"RSB refresh addr %x\012\000"
.LC145:
	.ascii	"spuer block %x vpn is 0\012 \000"
.LC146:
	.ascii	"g_recovery_ppa %x ver %x\012 \000"
.LC147:
	.ascii	"FtlCheckVpc %x = %x  %x\012\000"
.LC148:
	.ascii	"FtlGcScanTempBlk Error ID %x %x!!!!!!! \012\000"
.LC149:
	.ascii	"FtlGcScanTempBlkError ID %x %x!!!!!!!\012\000"
.LC150:
	.ascii	"GC des block %x done\012\000"
.LC151:
	.ascii	"%d GC datablk  = %x vpc %x %x\012\000"
.LC152:
	.ascii	"SWL %x, FSB = %x vpc= %x,ec=%x th=%x\012\000"
.LC153:
	.ascii	"Ftlwrite decrement_vpc_count %x = %d\012\000"
.LC154:
	.ascii	"rk_ftl_de_init %x\012\000"
.LC155:
	.ascii	"...%s: no bad block mapping table, format device\012"
	.ascii	"\000"
.LC156:
	.ascii	"...%s FtlSysBlkInit error ,format device!\012\000"
.LC157:
	.ascii	"FtlInit %x\012\000"
.LC158:
	.ascii	"fix power lost blk = %x vpc=%x\012\000"
.LC159:
	.ascii	"erase power lost blk = %x vpc=%x\012\000"
.LC160:
	.ascii	"FtlWrite: lpa error:%x %x\012\000"
.LC161:
	.ascii	"id = %x,%x addr= %x,spare= %x %x %x %x data = %x\012"
	.ascii	"\000"
.LC162:
	.ascii	":\000"
.LC163:
	.ascii	"phyBlk = %x,addr= %x,spare= %x %x %x %x data=%x %x\012"
	.ascii	"\000"
.LC164:
	.ascii	"Mblk:\000"
.LC165:
	.ascii	"L2P:\000"
.LC166:
	.ascii	"L2PC:\000"
.LC167:
	.ascii	"IdBlockReadData %x %x\012\000"
.LC168:
	.ascii	"IdBlockReadData %x %x ret= %x\012\000"
.LC169:
	.ascii	"write_idblock fix data %x %x\012\000"
.LC170:
	.ascii	"idblk:\000"
.LC171:
	.ascii	"idb reverse %x %x\012\000"
.LC172:
	.ascii	"write_idblock totle_sec %x %x %x %x\012\000"
.LC173:
	.ascii	"IDBlockWriteData %x %x\012\000"
.LC174:
	.ascii	"IDBlockWriteData %x %x ret= %x\012\000"
.LC175:
	.ascii	"write and check error:%d idb=%x,offset=%x,r=%x,w=%x"
	.ascii	"\012\000"
.LC176:
	.ascii	"write\000"
.LC177:
	.ascii	"read\000"
.LC178:
	.ascii	"write_idblock error %d\012\000"
.LC179:
	.ascii	"wl_lba %p %x %x %x\012\000"
.LC180:
	.ascii	"READ_SECTOR_IO\012\000"
.LC181:
	.ascii	"rk_copy_from_user error\012\000"
.LC182:
	.ascii	"READ_SECTOR_IO %x %x\012\000"
.LC183:
	.ascii	"rk_copy_to_user error\012\000"
.LC184:
	.ascii	"WRITE_SECTOR_IO\012\000"
.LC185:
	.ascii	"WRITE_SECTOR_IO %x %x\012\000"
.LC186:
	.ascii	"END_WRITE_SECTOR_IO\012\000"
.LC187:
	.ascii	"END_WRITE_SECTOR_IO %x %x\012\000"
.LC188:
	.ascii	"GET_FLASH_INFO_IO\012\000"
.LC189:
	.ascii	"GET_BAD_BLOCK_IO\012\000"
.LC190:
	.ascii	"GET_LOCK_FLAG_IO\012\000"
.LC191:
	.ascii	"GET_PUBLIC_KEY_IO\012\000"
.LC192:
	.ascii	"RKNAND_GET_DRM_KEY\012\000"
.LC193:
	.ascii	"RKNAND_STORE_DRM_KEY\012\000"
.LC194:
	.ascii	"RKNAND_DIASBLE_SECURE_BOOT\012\000"
.LC195:
	.ascii	"RKNAND_ENASBLE_SECURE_BOOT\012\000"
.LC196:
	.ascii	"RKNAND_GET_SN_SECTOR\012\000"
.LC197:
	.ascii	"RKNAND_LOADER_UNLOCK\012\000"
.LC198:
	.ascii	"RKNAND_LOADER_STATUS\012\000"
.LC199:
	.ascii	"RKNAND_LOADER_LOCK\012\000"
.LC200:
	.ascii	"LockKey not match %d\012\000"
.LC201:
	.ascii	"RKNAND_GET_VENDOR_SECTOR\012\000"
.LC202:
	.ascii	"RKNAND_STORE_VENDOR_SECTOR\012\000"
.LC203:
	.ascii	"return ret = %lx\012\000"
.LC204:
	.ascii	"secureBootEn check error\012\000"
.LC205:
	.ascii	"\0013vendor storage %x,%x,%x\012\000"
	.data
	.align	2
.LANCHOR1 = . + 0
	.type	random_seed, %object
	.size	random_seed, 256
random_seed:
	.short	22378
	.short	1512
	.short	25245
	.short	17827
	.short	25756
	.short	19440
	.short	9026
	.short	10030
	.short	29528
	.short	20467
	.short	29676
	.short	24432
	.short	31328
	.short	6872
	.short	13426
	.short	13842
	.short	8783
	.short	1108
	.short	782
	.short	28837
	.short	30729
	.short	9505
	.short	18676
	.short	23085
	.short	18730
	.short	1085
	.short	32609
	.short	14697
	.short	20858
	.short	15170
	.short	30365
	.short	1607
	.short	32298
	.short	4995
	.short	18905
	.short	1976
	.short	9592
	.short	20204
	.short	17443
	.short	13615
	.short	23330
	.short	29369
	.short	13947
	.short	9398
	.short	32398
	.short	8984
	.short	27600
	.short	21785
	.short	6019
	.short	6311
	.short	31598
	.short	30210
	.short	19327
	.short	13896
	.short	11347
	.short	27545
	.short	3107
	.short	26575
	.short	32270
	.short	19852
	.short	20601
	.short	8349
	.short	9290
	.short	29819
	.short	13579
	.short	3661
	.short	28676
	.short	27331
	.short	32574
	.short	8693
	.short	31253
	.short	9081
	.short	5399
	.short	6842
	.short	20087
	.short	5537
	.short	1274
	.short	11617
	.short	9530
	.short	4866
	.short	8035
	.short	23219
	.short	1178
	.short	23272
	.short	7383
	.short	18944
	.short	12488
	.short	12871
	.short	29340
	.short	20532
	.short	11022
	.short	22514
	.short	228
	.short	22363
	.short	24978
	.short	14584
	.short	12138
	.short	3092
	.short	17916
	.short	16863
	.short	14554
	.short	31457
	.short	29474
	.short	25311
	.short	24121
	.short	3684
	.short	28037
	.short	22865
	.short	22839
	.short	25217
	.short	13217
	.short	27186
	.short	14938
	.short	11180
	.short	29754
	.short	24180
	.short	15150
	.short	32455
	.short	20434
	.short	23848
	.short	29983
	.short	16120
	.short	14769
	.short	20041
	.short	29803
	.short	28406
	.short	17598
	.short	28087
	.type	gNandParaInfo, %object
	.size	gNandParaInfo, 32
gNandParaInfo:
	.byte	0
	.byte	0
	.space	5
	.byte	0
	.byte	1
	.byte	8
	.short	128
	.byte	2
	.byte	1
	.short	2048
	.short	0
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.type	ToshibaA19RefValue, %object
	.size	ToshibaA19RefValue, 45
ToshibaA19RefValue:
	.byte	4
	.byte	5
	.byte	6
	.byte	7
	.byte	13
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	4
	.byte	4
	.byte	124
	.byte	126
	.byte	0
	.byte	0
	.byte	124
	.byte	120
	.byte	120
	.byte	0
	.byte	124
	.byte	118
	.byte	116
	.byte	114
	.byte	0
	.byte	8
	.byte	8
	.byte	0
	.byte	0
	.byte	0
	.byte	11
	.byte	126
	.byte	118
	.byte	116
	.byte	0
	.byte	16
	.byte	118
	.byte	114
	.byte	112
	.byte	0
	.byte	2
	.byte	0
	.byte	126
	.byte	124
	.byte	0
	.space	3
	.type	Toshiba15RefValue, %object
	.size	Toshiba15RefValue, 95
Toshiba15RefValue:
	.byte	4
	.byte	5
	.byte	6
	.byte	7
	.byte	13
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	2
	.byte	4
	.byte	2
	.byte	0
	.byte	0
	.byte	8
	.byte	8
	.byte	0
	.byte	0
	.byte	0
	.byte	124
	.byte	0
	.byte	124
	.byte	124
	.byte	0
	.byte	122
	.byte	0
	.byte	122
	.byte	122
	.byte	0
	.byte	11
	.byte	126
	.byte	118
	.byte	116
	.byte	0
	.byte	120
	.byte	2
	.byte	120
	.byte	122
	.byte	0
	.byte	126
	.byte	4
	.byte	126
	.byte	122
	.byte	0
	.byte	16
	.byte	118
	.byte	114
	.byte	112
	.byte	0
	.byte	118
	.byte	4
	.byte	118
	.byte	120
	.byte	0
	.byte	4
	.byte	4
	.byte	4
	.byte	118
	.byte	0
	.byte	2
	.byte	0
	.byte	126
	.byte	124
	.byte	0
	.byte	6
	.byte	10
	.byte	6
	.byte	2
	.byte	0
	.byte	116
	.byte	124
	.byte	116
	.byte	118
	.byte	0
	.byte	4
	.byte	4
	.byte	124
	.byte	126
	.byte	0
	.byte	0
	.byte	124
	.byte	120
	.byte	120
	.byte	0
	.byte	124
	.byte	118
	.byte	116
	.byte	114
	.byte	0
	.space	1
	.type	ToshibaRefValue, %object
	.size	ToshibaRefValue, 8
ToshibaRefValue:
	.byte	0
	.byte	4
	.byte	124
	.byte	120
	.byte	116
	.byte	8
	.byte	12
	.byte	112
	.type	SamsungRefValue, %object
	.size	SamsungRefValue, 64
SamsungRefValue:
	.byte	-89
	.byte	-92
	.byte	-91
	.byte	-90
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	5
	.byte	10
	.byte	0
	.byte	0
	.byte	40
	.byte	0
	.byte	-20
	.byte	-40
	.byte	-19
	.byte	-11
	.byte	-19
	.byte	-26
	.byte	10
	.byte	15
	.byte	5
	.byte	0
	.byte	15
	.byte	10
	.byte	-5
	.byte	-20
	.byte	-24
	.byte	-17
	.byte	-24
	.byte	-36
	.byte	-15
	.byte	-5
	.byte	-2
	.byte	-16
	.byte	10
	.byte	0
	.byte	-5
	.byte	-20
	.byte	-48
	.byte	-30
	.byte	-48
	.byte	-62
	.byte	20
	.byte	15
	.byte	-5
	.byte	-20
	.byte	-24
	.byte	-5
	.byte	-24
	.byte	-36
	.byte	30
	.byte	20
	.byte	-5
	.byte	-20
	.byte	-5
	.byte	-1
	.byte	-5
	.byte	-8
	.byte	7
	.byte	12
	.byte	2
	.byte	0
	.type	gFtlInitStatus, %object
	.size	gFtlInitStatus, 4
gFtlInitStatus:
	.word	-1
	.type	NandFlashParaTbl, %object
	.size	NandFlashParaTbl, 2592
NandFlashParaTbl:
	.byte	6
	.byte	44
	.byte	100
	.byte	68
	.byte	75
	.byte	-87
	.byte	0
	.byte	4
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	2048
	.short	479
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	44
	.byte	68
	.byte	68
	.byte	75
	.byte	-87
	.byte	0
	.byte	4
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	1064
	.short	479
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	44
	.byte	104
	.byte	4
	.byte	74
	.byte	-87
	.byte	0
	.byte	4
	.byte	1
	.byte	8
	.short	256
	.byte	2
	.byte	2
	.short	2048
	.short	287
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	5
	.byte	44
	.byte	-120
	.byte	4
	.byte	75
	.byte	-87
	.byte	0
	.byte	4
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	2048
	.short	287
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	44
	.byte	-88
	.byte	5
	.byte	-53
	.byte	-87
	.byte	0
	.byte	4
	.byte	2
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	2048
	.short	287
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	44
	.byte	104
	.byte	4
	.byte	70
	.byte	-119
	.byte	0
	.byte	4
	.byte	1
	.byte	8
	.short	256
	.byte	2
	.byte	2
	.short	2048
	.short	287
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	44
	.byte	72
	.byte	4
	.byte	74
	.byte	-91
	.byte	0
	.byte	4
	.byte	1
	.byte	8
	.short	256
	.byte	2
	.byte	2
	.short	1024
	.short	287
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	44
	.byte	-124
	.byte	100
	.byte	60
	.byte	-91
	.byte	0
	.byte	4
	.byte	1
	.byte	32
	.short	512
	.byte	2
	.byte	2
	.short	1024
	.short	479
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	5
	.byte	44
	.byte	-124
	.byte	100
	.byte	84
	.byte	-87
	.byte	0
	.byte	4
	.byte	1
	.byte	32
	.short	512
	.byte	2
	.byte	2
	.short	1024
	.short	479
	.byte	4
	.byte	18
	.byte	60
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	44
	.byte	-41
	.byte	-108
	.byte	62
	.byte	-124
	.byte	0
	.byte	4
	.byte	1
	.byte	8
	.short	128
	.byte	2
	.byte	2
	.short	4096
	.short	279
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	44
	.byte	72
	.byte	4
	.byte	70
	.byte	-123
	.byte	0
	.byte	4
	.byte	1
	.byte	8
	.short	256
	.byte	2
	.byte	2
	.short	1024
	.short	287
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	44
	.byte	-120
	.byte	5
	.byte	-58
	.byte	-119
	.byte	0
	.byte	4
	.byte	2
	.byte	8
	.short	256
	.byte	2
	.byte	2
	.short	2048
	.short	287
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	5
	.byte	44
	.byte	-120
	.byte	36
	.byte	75
	.byte	-87
	.byte	0
	.byte	4
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	2048
	.short	287
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	44
	.byte	104
	.byte	0
	.byte	39
	.byte	-87
	.byte	0
	.byte	4
	.byte	1
	.byte	16
	.short	128
	.byte	1
	.byte	2
	.short	2048
	.short	287
	.byte	0
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	5
	.byte	44
	.byte	100
	.byte	100
	.byte	86
	.byte	-91
	.byte	0
	.byte	4
	.byte	1
	.byte	24
	.short	512
	.byte	2
	.byte	2
	.short	700
	.short	479
	.byte	4
	.byte	18
	.byte	60
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	44
	.byte	-124
	.byte	-59
	.byte	75
	.byte	-87
	.byte	0
	.byte	4
	.byte	2
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	2048
	.short	479
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	44
	.byte	-43
	.byte	-47
	.byte	-90
	.byte	104
	.byte	0
	.byte	4
	.byte	2
	.byte	8
	.short	64
	.byte	1
	.byte	2
	.short	2048
	.short	279
	.byte	0
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	44
	.byte	-36
	.byte	-112
	.byte	-90
	.byte	84
	.byte	0
	.byte	4
	.byte	1
	.byte	8
	.short	64
	.byte	1
	.byte	2
	.short	1024
	.short	279
	.byte	0
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	5
	.byte	44
	.byte	100
	.byte	100
	.byte	84
	.byte	-92
	.byte	0
	.byte	4
	.byte	1
	.byte	32
	.short	512
	.byte	2
	.byte	1
	.short	1024
	.short	479
	.byte	4
	.byte	18
	.byte	60
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	5
	.byte	44
	.byte	-124
	.byte	68
	.byte	50
	.byte	-86
	.byte	0
	.byte	4
	.byte	1
	.byte	32
	.short	512
	.byte	2
	.byte	1
	.short	2184
	.short	1479
	.byte	5
	.byte	19
	.byte	60
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.space	4
	.byte	5
	.byte	44
	.byte	100
	.byte	68
	.byte	50
	.byte	-91
	.byte	0
	.byte	4
	.byte	1
	.byte	32
	.short	512
	.byte	2
	.byte	1
	.short	1048
	.short	1479
	.byte	5
	.byte	19
	.byte	60
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.space	4
	.byte	5
	.byte	44
	.byte	100
	.byte	100
	.byte	60
	.byte	-91
	.byte	0
	.byte	4
	.byte	1
	.byte	32
	.short	512
	.byte	2
	.byte	1
	.short	1044
	.short	479
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	5
	.byte	44
	.byte	-124
	.byte	68
	.byte	50
	.byte	-86
	.byte	0
	.byte	4
	.byte	1
	.byte	32
	.short	512
	.byte	2
	.byte	1
	.short	2184
	.short	1479
	.byte	5
	.byte	19
	.byte	60
	.byte	32
	.byte	1
	.byte	0
	.byte	4
	.byte	0
	.byte	1
	.byte	0
	.space	4
	.byte	5
	.byte	44
	.byte	-124
	.byte	68
	.byte	52
	.byte	-86
	.byte	0
	.byte	4
	.byte	1
	.byte	32
	.short	512
	.byte	2
	.byte	1
	.short	2184
	.short	1479
	.byte	5
	.byte	19
	.byte	60
	.byte	32
	.byte	1
	.byte	0
	.byte	4
	.byte	0
	.byte	1
	.byte	0
	.space	4
	.byte	5
	.byte	44
	.byte	-124
	.byte	-60
	.byte	52
	.byte	-86
	.byte	0
	.byte	4
	.byte	1
	.byte	32
	.short	512
	.byte	2
	.byte	1
	.short	2184
	.short	1479
	.byte	5
	.byte	19
	.byte	60
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.space	4
	.byte	5
	.byte	44
	.byte	-124
	.byte	68
	.byte	52
	.byte	-92
	.byte	0
	.byte	4
	.byte	1
	.byte	32
	.short	512
	.byte	2
	.byte	1
	.short	2184
	.short	1479
	.byte	5
	.byte	19
	.byte	60
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.space	4
	.byte	5
	.byte	44
	.byte	-124
	.byte	100
	.byte	60
	.byte	-87
	.byte	0
	.byte	4
	.byte	1
	.byte	32
	.short	512
	.byte	2
	.byte	2
	.short	1024
	.short	479
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-83
	.byte	-34
	.byte	-108
	.byte	-46
	.byte	4
	.byte	67
	.byte	2
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	2048
	.short	473
	.byte	1
	.byte	1
	.byte	24
	.byte	32
	.byte	4
	.byte	0
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-83
	.byte	-41
	.byte	-108
	.byte	-38
	.byte	116
	.byte	-61
	.byte	2
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	1024
	.short	473
	.byte	1
	.byte	2
	.byte	40
	.byte	32
	.byte	4
	.byte	0
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-83
	.byte	-41
	.byte	-108
	.byte	-111
	.byte	96
	.byte	68
	.byte	2
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	1046
	.short	473
	.byte	1
	.byte	3
	.byte	40
	.byte	32
	.byte	4
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-83
	.byte	-34
	.byte	-108
	.byte	-38
	.byte	116
	.byte	-60
	.byte	2
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	2090
	.short	473
	.byte	1
	.byte	4
	.byte	40
	.byte	32
	.byte	4
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-83
	.byte	-34
	.byte	-108
	.byte	-21
	.byte	116
	.byte	68
	.byte	2
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	2
	.short	1066
	.short	473
	.byte	1
	.byte	7
	.byte	40
	.byte	32
	.byte	4
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-83
	.byte	-43
	.byte	-108
	.byte	-38
	.byte	116
	.byte	-60
	.byte	2
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	530
	.short	473
	.byte	1
	.byte	3
	.byte	40
	.byte	32
	.byte	4
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-83
	.byte	-41
	.byte	-108
	.byte	-102
	.byte	116
	.byte	66
	.byte	2
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	1024
	.short	281
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	4
	.byte	0
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-83
	.byte	-34
	.byte	20
	.byte	-89
	.byte	66
	.byte	74
	.byte	2
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	2
	.short	1060
	.short	473
	.byte	2
	.byte	5
	.byte	40
	.byte	32
	.byte	4
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-83
	.byte	-41
	.byte	20
	.byte	-98
	.byte	52
	.byte	74
	.byte	2
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	1056
	.short	473
	.byte	2
	.byte	5
	.byte	40
	.byte	32
	.byte	4
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-83
	.byte	-34
	.byte	-108
	.byte	-89
	.byte	66
	.byte	72
	.byte	2
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	2
	.short	1060
	.short	473
	.byte	2
	.byte	5
	.byte	40
	.byte	32
	.byte	4
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-83
	.byte	-34
	.byte	20
	.byte	-85
	.byte	66
	.byte	74
	.byte	2
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	2
	.short	1056
	.short	473
	.byte	2
	.byte	6
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-83
	.byte	58
	.byte	20
	.byte	-85
	.byte	66
	.byte	74
	.byte	2
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	2
	.short	2092
	.short	473
	.byte	2
	.byte	5
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-83
	.byte	-43
	.byte	-108
	.byte	-102
	.byte	116
	.byte	66
	.byte	2
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	1
	.short	1024
	.short	273
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	4
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-83
	.byte	58
	.byte	20
	.byte	3
	.byte	8
	.byte	80
	.byte	2
	.byte	1
	.byte	32
	.short	388
	.byte	2
	.byte	2
	.short	1362
	.short	473
	.byte	9
	.byte	8
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-119
	.byte	100
	.byte	68
	.byte	75
	.byte	-87
	.byte	0
	.byte	7
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	2048
	.short	479
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-119
	.byte	-120
	.byte	36
	.byte	75
	.byte	-87
	.byte	-124
	.byte	7
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	2048
	.short	479
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-119
	.byte	-120
	.byte	36
	.byte	75
	.byte	-87
	.byte	0
	.byte	7
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	2048
	.short	279
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-119
	.byte	104
	.byte	36
	.byte	74
	.byte	-87
	.byte	0
	.byte	7
	.byte	1
	.byte	8
	.short	256
	.byte	2
	.byte	2
	.short	2048
	.short	279
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-119
	.byte	104
	.byte	4
	.byte	74
	.byte	-87
	.byte	0
	.byte	7
	.byte	1
	.byte	8
	.short	256
	.byte	2
	.byte	2
	.short	2048
	.short	279
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-119
	.byte	-41
	.byte	-108
	.byte	62
	.byte	-124
	.byte	0
	.byte	7
	.byte	1
	.byte	8
	.short	256
	.byte	2
	.byte	2
	.short	2048
	.short	279
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-119
	.byte	104
	.byte	4
	.byte	70
	.byte	-87
	.byte	0
	.byte	7
	.byte	1
	.byte	8
	.short	256
	.byte	2
	.byte	2
	.short	2048
	.short	279
	.byte	1
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	5
	.byte	-119
	.byte	100
	.byte	100
	.byte	60
	.byte	-95
	.byte	0
	.byte	7
	.byte	1
	.byte	32
	.short	512
	.byte	2
	.byte	1
	.short	1024
	.short	455
	.byte	4
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-104
	.byte	-41
	.byte	-124
	.byte	-109
	.byte	114
	.byte	87
	.byte	1
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	1
	.short	1060
	.short	1473
	.byte	2
	.byte	33
	.byte	40
	.byte	32
	.byte	2
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-104
	.byte	-34
	.byte	-124
	.byte	-109
	.byte	114
	.byte	87
	.byte	1
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	1
	.short	2092
	.short	1473
	.byte	2
	.byte	33
	.byte	40
	.byte	32
	.byte	2
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-104
	.byte	58
	.byte	-123
	.byte	-109
	.byte	118
	.byte	87
	.byte	1
	.byte	2
	.byte	32
	.short	256
	.byte	2
	.byte	1
	.short	2092
	.short	1505
	.byte	2
	.byte	33
	.byte	40
	.byte	32
	.byte	2
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-104
	.byte	-43
	.byte	-124
	.byte	50
	.byte	114
	.byte	86
	.byte	1
	.byte	1
	.byte	16
	.short	128
	.byte	2
	.byte	1
	.short	2056
	.short	1473
	.byte	2
	.byte	33
	.byte	40
	.byte	32
	.byte	2
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-104
	.byte	-41
	.byte	-108
	.byte	50
	.byte	118
	.byte	86
	.byte	1
	.byte	1
	.byte	16
	.short	128
	.byte	2
	.byte	2
	.short	2058
	.short	1489
	.byte	2
	.byte	33
	.byte	40
	.byte	32
	.byte	2
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-104
	.byte	-34
	.byte	-108
	.byte	-126
	.byte	118
	.byte	86
	.byte	1
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	2062
	.short	1489
	.byte	1
	.byte	33
	.byte	40
	.byte	32
	.byte	2
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-104
	.byte	-34
	.byte	-108
	.byte	-109
	.byte	118
	.byte	80
	.byte	1
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	2
	.short	1066
	.short	1497
	.byte	2
	.byte	34
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-104
	.byte	58
	.byte	-107
	.byte	-109
	.byte	122
	.byte	80
	.byte	1
	.byte	2
	.byte	32
	.short	256
	.byte	2
	.byte	2
	.short	1066
	.short	1497
	.byte	2
	.byte	34
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-104
	.byte	-41
	.byte	-108
	.byte	50
	.byte	118
	.byte	85
	.byte	1
	.byte	1
	.byte	16
	.short	128
	.byte	2
	.byte	2
	.short	2050
	.short	401
	.byte	2
	.byte	0
	.byte	24
	.byte	32
	.byte	1
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-104
	.byte	-34
	.byte	-108
	.byte	-109
	.byte	118
	.byte	87
	.byte	1
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	2
	.short	1058
	.short	1497
	.byte	2
	.byte	33
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-104
	.byte	-41
	.byte	-124
	.byte	-109
	.byte	114
	.byte	80
	.byte	1
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	1
	.short	1060
	.short	1473
	.byte	2
	.byte	34
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-104
	.byte	-34
	.byte	-108
	.byte	-109
	.byte	118
	.byte	81
	.byte	1
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	2
	.short	1074
	.short	1497
	.byte	2
	.byte	35
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-104
	.byte	58
	.byte	-108
	.byte	-109
	.byte	118
	.byte	81
	.byte	1
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	2
	.short	2106
	.short	1497
	.byte	2
	.byte	35
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-104
	.byte	-41
	.byte	-124
	.byte	-109
	.byte	114
	.byte	81
	.byte	1
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	1
	.short	1056
	.short	1497
	.byte	2
	.byte	35
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	69
	.byte	-34
	.byte	-108
	.byte	-109
	.byte	118
	.byte	87
	.byte	8
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	2
	.short	1058
	.short	1497
	.byte	2
	.byte	66
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	69
	.byte	-41
	.byte	-124
	.byte	-109
	.byte	114
	.byte	87
	.byte	8
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	1
	.short	1060
	.short	1473
	.byte	2
	.byte	66
	.byte	40
	.byte	32
	.byte	2
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	69
	.byte	-34
	.byte	-92
	.byte	-126
	.byte	118
	.byte	86
	.byte	8
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	2082
	.short	473
	.byte	1
	.byte	65
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	69
	.byte	-34
	.byte	-108
	.byte	-109
	.byte	118
	.byte	80
	.byte	8
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	2
	.short	1066
	.short	1497
	.byte	2
	.byte	67
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	69
	.byte	-41
	.byte	-124
	.byte	-109
	.byte	114
	.byte	80
	.byte	8
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	1
	.short	1060
	.short	1473
	.byte	2
	.byte	67
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	69
	.byte	-34
	.byte	-92
	.byte	-126
	.byte	118
	.byte	-41
	.byte	8
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	2090
	.short	1241
	.byte	1
	.byte	66
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	69
	.byte	-34
	.byte	-124
	.byte	-109
	.byte	114
	.byte	87
	.byte	8
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	1
	.short	2092
	.short	1473
	.byte	2
	.byte	66
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	69
	.byte	58
	.byte	-108
	.byte	-109
	.byte	118
	.byte	81
	.byte	8
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	2
	.short	2106
	.short	473
	.byte	2
	.byte	68
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	69
	.byte	-34
	.byte	-108
	.byte	-109
	.byte	118
	.byte	81
	.byte	8
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	2
	.short	1074
	.short	473
	.byte	2
	.byte	68
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	69
	.byte	58
	.byte	-92
	.byte	-109
	.byte	122
	.byte	80
	.byte	8
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	2
	.short	2138
	.short	1497
	.byte	2
	.byte	0
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	69
	.byte	-34
	.byte	-108
	.byte	-126
	.byte	118
	.byte	86
	.byte	8
	.byte	1
	.byte	16
	.short	256
	.byte	2
	.byte	2
	.short	2062
	.short	473
	.byte	1
	.byte	0
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	69
	.byte	-34
	.byte	-108
	.byte	-109
	.byte	118
	.byte	-41
	.byte	8
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	2
	.short	1058
	.short	1497
	.byte	2
	.byte	66
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-20
	.byte	-41
	.byte	-108
	.byte	126
	.byte	100
	.byte	68
	.byte	0
	.byte	1
	.byte	16
	.short	128
	.byte	2
	.byte	2
	.short	2048
	.short	473
	.byte	2
	.byte	49
	.byte	60
	.byte	36
	.byte	3
	.byte	0
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-20
	.byte	-34
	.byte	-43
	.byte	126
	.byte	104
	.byte	68
	.byte	0
	.byte	2
	.byte	16
	.short	128
	.byte	2
	.byte	2
	.short	2048
	.short	505
	.byte	2
	.byte	49
	.byte	60
	.byte	36
	.byte	3
	.byte	0
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-20
	.byte	-41
	.byte	-108
	.byte	122
	.byte	84
	.byte	67
	.byte	0
	.byte	1
	.byte	16
	.short	128
	.byte	2
	.byte	2
	.short	2076
	.short	409
	.byte	2
	.byte	0
	.byte	40
	.byte	36
	.byte	3
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-20
	.byte	-34
	.byte	-43
	.byte	122
	.byte	88
	.byte	67
	.byte	0
	.byte	2
	.byte	16
	.short	128
	.byte	2
	.byte	2
	.short	2076
	.short	441
	.byte	2
	.byte	0
	.byte	40
	.byte	36
	.byte	3
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-20
	.byte	-43
	.byte	-108
	.byte	118
	.byte	84
	.byte	67
	.byte	0
	.byte	1
	.byte	16
	.short	128
	.byte	2
	.byte	2
	.short	1038
	.short	281
	.byte	2
	.byte	0
	.byte	24
	.byte	36
	.byte	3
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.byte	6
	.byte	-20
	.byte	-41
	.byte	20
	.byte	118
	.byte	84
	.byte	-62
	.byte	0
	.byte	1
	.byte	16
	.short	128
	.byte	2
	.byte	2
	.short	2076
	.short	1169
	.byte	2
	.byte	0
	.byte	24
	.byte	40
	.byte	3
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.type	NandOptPara, %object
	.size	NandOptPara, 128
NandOptPara:
	.byte	1
	.byte	0
	.byte	49
	.byte	63
	.byte	0
	.byte	49
	.byte	-128
	.byte	21
	.byte	0
	.byte	50
	.byte	17
	.byte	-128
	.byte	112
	.byte	120
	.byte	120
	.byte	3
	.byte	1
	.byte	0
	.space	14
	.byte	2
	.byte	0
	.byte	49
	.byte	63
	.byte	0
	.byte	49
	.byte	-128
	.byte	21
	.byte	0
	.byte	0
	.byte	17
	.byte	-127
	.byte	112
	.byte	-15
	.byte	-14
	.byte	0
	.byte	0
	.byte	0
	.space	14
	.byte	3
	.byte	0
	.byte	49
	.byte	63
	.byte	0
	.byte	49
	.byte	-128
	.byte	21
	.byte	96
	.byte	96
	.byte	17
	.byte	-127
	.byte	112
	.byte	-15
	.byte	-14
	.byte	0
	.byte	0
	.byte	0
	.space	14
	.byte	4
	.byte	0
	.byte	49
	.byte	63
	.byte	0
	.byte	49
	.byte	-128
	.byte	21
	.byte	96
	.byte	96
	.byte	17
	.byte	-127
	.byte	112
	.byte	112
	.byte	112
	.byte	0
	.byte	0
	.byte	0
	.space	14
	.type	refValueDefault, %object
	.size	refValueDefault, 28
refValueDefault:
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	6
	.byte	10
	.byte	6
	.byte	0
	.byte	-3
	.byte	-7
	.byte	-8
	.byte	0
	.byte	-6
	.byte	-13
	.byte	-15
	.byte	0
	.byte	-11
	.byte	-20
	.byte	-23
	.byte	0
	.byte	0
	.byte	-26
	.byte	-30
	.byte	0
	.byte	0
	.byte	-32
	.byte	-37
	.type	gSlcNandParaInfo, %object
	.size	gSlcNandParaInfo, 32
gSlcNandParaInfo:
	.byte	2
	.byte	-104
	.byte	-15
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	1
	.byte	1
	.byte	4
	.short	64
	.byte	1
	.byte	1
	.short	1024
	.short	256
	.byte	0
	.byte	0
	.byte	16
	.byte	40
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.byte	0
	.byte	0
	.space	4
	.type	ftl_gc_temp_block_bops_scan_page_addr, %object
	.size	ftl_gc_temp_block_bops_scan_page_addr, 2
ftl_gc_temp_block_bops_scan_page_addr:
	.short	-1
	.space	2
	.type	power_up_flag, %object
	.size	power_up_flag, 4
power_up_flag:
	.word	1
	.type	gTable_Crc32, %object
	.size	gTable_Crc32, 1024
gTable_Crc32:
	.word	0
	.word	79760823
	.word	159521646
	.word	222500569
	.word	319043292
	.word	398801771
	.word	445001138
	.word	507977733
	.word	638086584
	.word	583622671
	.word	797603542
	.word	726367073
	.word	890002276
	.word	835540691
	.word	1015955466
	.word	944721341
	.word	1276173168
	.word	1221711559
	.word	1167245342
	.word	1096011177
	.word	1595207084
	.word	1540743195
	.word	1452734146
	.word	1381497717
	.word	1780004552
	.word	1859763071
	.word	1671081382
	.word	1734058001
	.word	2031910932
	.word	2111671715
	.word	1889442682
	.word	1952421581
	.word	-1742620960
	.word	-1662993577
	.word	-1851544178
	.word	-1788698567
	.word	-1960476612
	.word	-1880846965
	.word	-2102944942
	.word	-2040097051
	.word	-1104553128
	.word	-1159145745
	.word	-1213480906
	.word	-1284846207
	.word	-1389499004
	.word	-1444093901
	.word	-1531971862
	.word	-1603339427
	.word	-734958192
	.word	-789553113
	.word	-575441154
	.word	-646808759
	.word	-952804532
	.word	-1007397125
	.word	-826851294
	.word	-898216555
	.word	-231145432
	.word	-151515745
	.word	-71623866
	.word	-8775951
	.word	-516081932
	.word	-436454589
	.word	-390124134
	.word	-327278547
	.word	880959607
	.word	809725376
	.word	1023441689
	.word	968980142
	.word	663115435
	.word	591878940
	.word	772033989
	.word	717570162
	.word	311037391
	.word	374014072
	.word	453514913
	.word	533273366
	.word	26098451
	.word	89077412
	.word	135012477
	.word	214773194
	.word	2022882055
	.word	2085861040
	.word	1896915049
	.word	1976675806
	.word	1805028827
	.word	1868005484
	.word	1645516469
	.word	1725274882
	.word	1587205823
	.word	1515969288
	.word	1461243345
	.word	1406779494
	.word	1302257763
	.word	1231023572
	.word	1142749965
	.word	1088288442
	.word	-1398548841
	.word	-1469916384
	.word	-1524511239
	.word	-1579106226
	.word	-1079517109
	.word	-1150882308
	.word	-1239024859
	.word	-1293617518
	.word	-1968456913
	.word	-1905609064
	.word	-2094423999
	.word	-2014794250
	.word	-1716548109
	.word	-1653702588
	.word	-1876060515
	.word	-1796433110
	.word	-525136409
	.word	-462290864
	.word	-382658935
	.word	-303031490
	.word	-206095557
	.word	-143247732
	.word	-97181611
	.word	-17551902
	.word	-960798625
	.word	-1032163864
	.word	-818316495
	.word	-872909178
	.word	-708880765
	.word	-780248268
	.word	-599962131
	.word	-654557094
	.word	1761919214
	.word	1841679705
	.word	1619450752
	.word	1682429495
	.word	2046883378
	.word	2126642053
	.word	1937960284
	.word	2000937195
	.word	1326230870
	.word	1271767265
	.word	1183757880
	.word	1112521615
	.word	1544067978
	.word	1489606205
	.word	1435140324
	.word	1363905875
	.word	622074782
	.word	567612969
	.word	748028144
	.word	676793671
	.word	907029826
	.word	852566261
	.word	1066546732
	.word	995310491
	.word	52196902
	.word	131955601
	.word	178154824
	.word	241131775
	.word	270024954
	.word	349785421
	.word	429546388
	.word	492525091
	.word	-249203186
	.word	-169575495
	.word	-123245216
	.word	-60399401
	.word	-501137198
	.word	-421507739
	.word	-341615684
	.word	-278768117
	.word	-684909642
	.word	-739502591
	.word	-558956328
	.word	-630321809
	.word	-1003934358
	.word	-1058529059
	.word	-844417532
	.word	-915784781
	.word	-1120555650
	.word	-1175150391
	.word	-1263028720
	.word	-1334395993
	.word	-1372480606
	.word	-1427073515
	.word	-1481408308
	.word	-1552773765
	.word	-1690451770
	.word	-1610822287
	.word	-1832920152
	.word	-1770072545
	.word	-2009467366
	.word	-1929839699
	.word	-2118390412
	.word	-2055544637
	.word	1569104025
	.word	1497869614
	.word	1409596407
	.word	1355134528
	.word	1317180997
	.word	1245944818
	.word	1191218475
	.word	1136754844
	.word	2072956193
	.word	2135933078
	.word	1913443919
	.word	1993202680
	.word	1753938941
	.word	1816917578
	.word	1627971731
	.word	1707732260
	.word	295074793
	.word	358053470
	.word	403988615
	.word	483749168
	.word	43142453
	.word	106119298
	.word	185620059
	.word	265378796
	.word	933107281
	.word	861871078
	.word	1042025791
	.word	987562120
	.word	614080653
	.word	542846266
	.word	756562915
	.word	702101076
	.word	-978905479
	.word	-1050272818
	.word	-869987049
	.word	-924581728
	.word	-693952347
	.word	-765317870
	.word	-551470133
	.word	-606062980
	.word	-475038783
	.word	-412191114
	.word	-366124881
	.word	-286495464
	.word	-257209059
	.word	-194363222
	.word	-114731405
	.word	-35103804
	.word	-1984443127
	.word	-1921597250
	.word	-2143955353
	.word	-2064327728
	.word	-1699480619
	.word	-1636632990
	.word	-1825447749
	.word	-1745818356
	.word	-1346395983
	.word	-1417761530
	.word	-1505903649
	.word	-1560496536
	.word	-1128556947
	.word	-1199924262
	.word	-1254519549
	.word	-1309114188
	.bss
	.align	2
.LANCHOR0 = . + 0
.LANCHOR2 = . + 4344
.LANCHOR4 = . + 8688
.LANCHOR5 = . + 13032
.LANCHOR6 = . + 17376
	.type	g_slc2KBNand, %object
	.size	g_slc2KBNand, 1
g_slc2KBNand:
	.space	1
	.type	gNandIDBResBlkNum, %object
	.size	gNandIDBResBlkNum, 1
gNandIDBResBlkNum:
	.space	1
	.space	2
	.type	gBlockPageAlignSize, %object
	.size	gBlockPageAlignSize, 4
gBlockPageAlignSize:
	.space	4
	.type	gNandRandomizer, %object
	.size	gNandRandomizer, 1
gNandRandomizer:
	.space	1
	.space	3
	.type	gNandChipMap, %object
	.size	gNandChipMap, 32
gNandChipMap:
	.space	32
	.type	gNandOptPara, %object
	.size	gNandOptPara, 32
gNandOptPara:
	.space	32
	.type	slcPageToMlcPageTbl, %object
	.size	slcPageToMlcPageTbl, 512
slcPageToMlcPageTbl:
	.space	512
	.type	mlcPageToSlcPageTbl, %object
	.size	mlcPageToSlcPageTbl, 1024
mlcPageToSlcPageTbl:
	.space	1024
	.type	gNandMaxDie, %object
	.size	gNandMaxDie, 1
gNandMaxDie:
	.space	1
	.type	gNandMaxChip, %object
	.size	gNandMaxChip, 1
gNandMaxChip:
	.space	1
	.space	2
	.type	DieCsIndex, %object
	.size	DieCsIndex, 8
DieCsIndex:
	.space	8
	.type	DieAddrs, %object
	.size	DieAddrs, 32
DieAddrs:
	.space	32
	.type	gDieOp, %object
	.size	gDieOp, 128
gDieOp:
	.space	128
	.type	gpNandParaInfo, %object
	.size	gpNandParaInfo, 4
gpNandParaInfo:
	.space	4
	.type	IDByte, %object
	.size	IDByte, 32
IDByte:
	.space	32
	.type	gTotleBlock, %object
	.size	gTotleBlock, 2
gTotleBlock:
	.space	2
	.space	2
	.type	gpNandc, %object
	.size	gpNandc, 4
gpNandc:
	.space	4
	.type	NANDC_FMCTL, %object
	.size	NANDC_FMCTL, 4
NANDC_FMCTL:
	.space	4
	.type	NANDC_FMWAIT, %object
	.size	NANDC_FMWAIT, 4
NANDC_FMWAIT:
	.space	4
	.type	NANDC_FLCTL, %object
	.size	NANDC_FLCTL, 4
NANDC_FLCTL:
	.space	4
	.type	NANDC_BCHCTL, %object
	.size	NANDC_BCHCTL, 4
NANDC_BCHCTL:
	.space	4
	.type	NANDC_DLL_CTL_REG0, %object
	.size	NANDC_DLL_CTL_REG0, 4
NANDC_DLL_CTL_REG0:
	.space	4
	.type	NANDC_DLL_CTL_REG1, %object
	.size	NANDC_DLL_CTL_REG1, 4
NANDC_DLL_CTL_REG1:
	.space	4
	.type	NANDC_RANDMZ_CFG, %object
	.size	NANDC_RANDMZ_CFG, 4
NANDC_RANDMZ_CFG:
	.space	4
	.type	NANDC_FMWAIT_SYN, %object
	.size	NANDC_FMWAIT_SYN, 4
NANDC_FMWAIT_SYN:
	.space	4
	.type	gNandPhyInfo, %object
	.size	gNandPhyInfo, 28
gNandPhyInfo:
	.space	28
	.type	gFlashSlcMode, %object
	.size	gFlashSlcMode, 1
gFlashSlcMode:
	.space	1
	.type	gNandFlashEccBits, %object
	.size	gNandFlashEccBits, 1
gNandFlashEccBits:
	.space	1
	.space	2
	.type	g_MaxLbaSector, %object
	.size	g_MaxLbaSector, 4
g_MaxLbaSector:
	.space	4
	.type	FlashWaitBusyScheduleEn, %object
	.size	FlashWaitBusyScheduleEn, 4
FlashWaitBusyScheduleEn:
	.space	4
	.type	g_retryMode, %object
	.size	g_retryMode, 1
g_retryMode:
	.space	1
	.type	g_maxRegNum, %object
	.size	g_maxRegNum, 1
g_maxRegNum:
	.space	1
	.type	gReadRetryInfo, %object
	.size	gReadRetryInfo, 852
gReadRetryInfo:
	.space	852
	.space	2
	.type	read_retry_cur_offset, %object
	.size	read_retry_cur_offset, 4
read_retry_cur_offset:
	.space	4
	.type	gFlashInterfaceMode, %object
	.size	gFlashInterfaceMode, 1
gFlashInterfaceMode:
	.space	1
	.space	3
	.type	idb_flash_slc_mode, %object
	.size	idb_flash_slc_mode, 4
idb_flash_slc_mode:
	.space	4
	.type	gFlashToggleModeEn, %object
	.size	gFlashToggleModeEn, 1
gFlashToggleModeEn:
	.space	1
	.space	3
	.type	gBootDdrMode, %object
	.size	gBootDdrMode, 4
gBootDdrMode:
	.space	4
	.type	gNandcVer, %object
	.size	gNandcVer, 4
gNandcVer:
	.space	4
	.type	g_nandc_version_data, %object
	.size	g_nandc_version_data, 4
g_nandc_version_data:
	.space	4
	.type	gMasterTempBuf, %object
	.size	gMasterTempBuf, 4
gMasterTempBuf:
	.space	4
	.type	gMasterInfo, %object
	.size	gMasterInfo, 32
gMasterInfo:
	.space	32
	.type	gNandcDumpWriteEn, %object
	.size	gNandcDumpWriteEn, 4
gNandcDumpWriteEn:
	.space	4
	.type	gNandcEccBits, %object
	.size	gNandcEccBits, 4
gNandcEccBits:
	.space	4
	.type	gMultiPageProgEn, %object
	.size	gMultiPageProgEn, 1
gMultiPageProgEn:
	.space	1
	.space	3
	.type	g_inkDie_check_enable, %object
	.size	g_inkDie_check_enable, 4
g_inkDie_check_enable:
	.space	4
	.type	c_ftl_nand_sys_blks_per_plane, %object
	.size	c_ftl_nand_sys_blks_per_plane, 4
c_ftl_nand_sys_blks_per_plane:
	.space	4
	.type	c_ftl_nand_planes_num, %object
	.size	c_ftl_nand_planes_num, 2
c_ftl_nand_planes_num:
	.space	2
	.space	2
	.type	c_ftl_nand_max_sys_blks, %object
	.size	c_ftl_nand_max_sys_blks, 4
c_ftl_nand_max_sys_blks:
	.space	4
	.type	c_ftl_nand_data_blks_per_plane, %object
	.size	c_ftl_nand_data_blks_per_plane, 2
c_ftl_nand_data_blks_per_plane:
	.space	2
	.type	c_ftl_nand_blk_pre_plane, %object
	.size	c_ftl_nand_blk_pre_plane, 2
c_ftl_nand_blk_pre_plane:
	.space	2
	.type	c_ftl_nand_max_data_blks, %object
	.size	c_ftl_nand_max_data_blks, 4
c_ftl_nand_max_data_blks:
	.space	4
	.type	c_ftl_nand_totle_phy_blks, %object
	.size	c_ftl_nand_totle_phy_blks, 4
c_ftl_nand_totle_phy_blks:
	.space	4
	.type	c_ftl_nand_type, %object
	.size	c_ftl_nand_type, 2
c_ftl_nand_type:
	.space	2
	.type	c_ftl_nand_die_num, %object
	.size	c_ftl_nand_die_num, 2
c_ftl_nand_die_num:
	.space	2
	.type	c_ftl_nand_planes_per_die, %object
	.size	c_ftl_nand_planes_per_die, 2
c_ftl_nand_planes_per_die:
	.space	2
	.space	2
	.type	p_plane_order_table, %object
	.size	p_plane_order_table, 32
p_plane_order_table:
	.space	32
	.type	c_mlc_erase_count_value, %object
	.size	c_mlc_erase_count_value, 2
c_mlc_erase_count_value:
	.space	2
	.type	c_ftl_nand_ext_blk_pre_plane, %object
	.size	c_ftl_nand_ext_blk_pre_plane, 2
c_ftl_nand_ext_blk_pre_plane:
	.space	2
	.type	c_ftl_vendor_part_size, %object
	.size	c_ftl_vendor_part_size, 2
c_ftl_vendor_part_size:
	.space	2
	.type	c_ftl_nand_blks_per_die, %object
	.size	c_ftl_nand_blks_per_die, 2
c_ftl_nand_blks_per_die:
	.space	2
	.type	c_ftl_nand_page_pre_blk, %object
	.size	c_ftl_nand_page_pre_blk, 2
c_ftl_nand_page_pre_blk:
	.space	2
	.type	c_ftl_nand_page_pre_slc_blk, %object
	.size	c_ftl_nand_page_pre_slc_blk, 2
c_ftl_nand_page_pre_slc_blk:
	.space	2
	.type	c_ftl_nand_page_pre_super_blk, %object
	.size	c_ftl_nand_page_pre_super_blk, 2
c_ftl_nand_page_pre_super_blk:
	.space	2
	.type	c_ftl_nand_sec_pre_page, %object
	.size	c_ftl_nand_sec_pre_page, 2
c_ftl_nand_sec_pre_page:
	.space	2
	.type	c_ftl_nand_sec_pre_page_shift, %object
	.size	c_ftl_nand_sec_pre_page_shift, 2
c_ftl_nand_sec_pre_page_shift:
	.space	2
	.type	c_ftl_nand_byte_pre_page, %object
	.size	c_ftl_nand_byte_pre_page, 2
c_ftl_nand_byte_pre_page:
	.space	2
	.type	c_ftl_nand_byte_pre_oob, %object
	.size	c_ftl_nand_byte_pre_oob, 2
c_ftl_nand_byte_pre_oob:
	.space	2
	.type	c_ftl_nand_reserved_blks, %object
	.size	c_ftl_nand_reserved_blks, 2
c_ftl_nand_reserved_blks:
	.space	2
	.type	DeviceCapacity, %object
	.size	DeviceCapacity, 4
DeviceCapacity:
	.space	4
	.type	c_ftl_nand_max_vendor_blks, %object
	.size	c_ftl_nand_max_vendor_blks, 2
c_ftl_nand_max_vendor_blks:
	.space	2
	.type	c_ftl_nand_vendor_region_num, %object
	.size	c_ftl_nand_vendor_region_num, 2
c_ftl_nand_vendor_region_num:
	.space	2
	.type	c_ftl_nand_map_blks_per_plane, %object
	.size	c_ftl_nand_map_blks_per_plane, 2
c_ftl_nand_map_blks_per_plane:
	.space	2
	.space	2
	.type	c_ftl_nand_max_map_blks, %object
	.size	c_ftl_nand_max_map_blks, 4
c_ftl_nand_max_map_blks:
	.space	4
	.type	c_ftl_nand_init_sys_blks_per_plane, %object
	.size	c_ftl_nand_init_sys_blks_per_plane, 4
c_ftl_nand_init_sys_blks_per_plane:
	.space	4
	.type	c_ftl_nand_map_region_num, %object
	.size	c_ftl_nand_map_region_num, 2
c_ftl_nand_map_region_num:
	.space	2
	.type	c_ftl_nand_l2pmap_ram_region_num, %object
	.size	c_ftl_nand_l2pmap_ram_region_num, 2
c_ftl_nand_l2pmap_ram_region_num:
	.space	2
	.type	g_page_map_check_enable, %object
	.size	g_page_map_check_enable, 2
g_page_map_check_enable:
	.space	2
	.type	g_totle_vendor_block, %object
	.size	g_totle_vendor_block, 2
g_totle_vendor_block:
	.space	2
	.type	p_vendor_block_table, %object
	.size	p_vendor_block_table, 4
p_vendor_block_table:
	.space	4
	.type	g_wr_page_num, %object
	.size	g_wr_page_num, 4
g_wr_page_num:
	.space	4
	.type	req_wr_io, %object
	.size	req_wr_io, 4
req_wr_io:
	.space	4
	.type	g_MaxLpn, %object
	.size	g_MaxLpn, 4
g_MaxLpn:
	.space	4
	.type	gBbtInfo, %object
	.size	gBbtInfo, 60
gBbtInfo:
	.space	60
	.type	gSysFreeQueue, %object
	.size	gSysFreeQueue, 2056
gSysFreeQueue:
	.space	2056
	.type	g_flash_read_only_en, %object
	.size	g_flash_read_only_en, 4
g_flash_read_only_en:
	.space	4
	.type	req_erase, %object
	.size	req_erase, 4
req_erase:
	.space	4
	.type	p_erase_count_table, %object
	.size	p_erase_count_table, 4
p_erase_count_table:
	.space	4
	.type	g_totle_sys_slc_erase_count, %object
	.size	g_totle_sys_slc_erase_count, 4
g_totle_sys_slc_erase_count:
	.space	4
	.type	g_sys_save_data, %object
	.size	g_sys_save_data, 48
g_sys_save_data:
	.space	48
	.type	p_data_block_list_table, %object
	.size	p_data_block_list_table, 4
p_data_block_list_table:
	.space	4
	.type	p_data_block_list_head, %object
	.size	p_data_block_list_head, 4
p_data_block_list_head:
	.space	4
	.type	p_valid_page_count_table, %object
	.size	p_valid_page_count_table, 4
p_valid_page_count_table:
	.space	4
	.type	p_data_block_list_tail, %object
	.size	p_data_block_list_tail, 4
p_data_block_list_tail:
	.space	4
	.type	g_num_data_superblocks, %object
	.size	g_num_data_superblocks, 2
g_num_data_superblocks:
	.space	2
	.space	2
	.type	p_free_data_block_list_head, %object
	.size	p_free_data_block_list_head, 4
p_free_data_block_list_head:
	.space	4
	.type	g_num_free_superblocks, %object
	.size	g_num_free_superblocks, 2
g_num_free_superblocks:
	.space	2
	.space	2
	.type	g_active_superblock, %object
	.size	g_active_superblock, 48
g_active_superblock:
	.space	48
	.type	g_buffer_superblock, %object
	.size	g_buffer_superblock, 48
g_buffer_superblock:
	.space	48
	.type	g_gc_temp_superblock, %object
	.size	g_gc_temp_superblock, 48
g_gc_temp_superblock:
	.space	48
	.type	p_l2p_ram_map, %object
	.size	p_l2p_ram_map, 4
p_l2p_ram_map:
	.space	4
	.type	g_l2p_last_update_region_id, %object
	.size	g_l2p_last_update_region_id, 2
g_l2p_last_update_region_id:
	.space	2
	.type	FtlUpdateVaildLpnCount, %object
	.size	FtlUpdateVaildLpnCount, 2
FtlUpdateVaildLpnCount:
	.space	2
	.type	g_VaildLpn, %object
	.size	g_VaildLpn, 4
g_VaildLpn:
	.space	4
	.type	p_blk_mode_table, %object
	.size	p_blk_mode_table, 4
p_blk_mode_table:
	.space	4
	.type	g_totle_read_page_count, %object
	.size	g_totle_read_page_count, 4
g_totle_read_page_count:
	.space	4
	.type	g_totle_discard_page_count, %object
	.size	g_totle_discard_page_count, 4
g_totle_discard_page_count:
	.space	4
	.type	g_totle_write_page_count, %object
	.size	g_totle_write_page_count, 4
g_totle_write_page_count:
	.space	4
	.type	g_totle_cache_write_count, %object
	.size	g_totle_cache_write_count, 4
g_totle_cache_write_count:
	.space	4
	.type	g_totle_l2p_write_count, %object
	.size	g_totle_l2p_write_count, 4
g_totle_l2p_write_count:
	.space	4
	.type	g_totle_gc_page_count, %object
	.size	g_totle_gc_page_count, 4
g_totle_gc_page_count:
	.space	4
	.type	g_totle_write_sector, %object
	.size	g_totle_write_sector, 4
g_totle_write_sector:
	.space	4
	.type	g_totle_read_sector, %object
	.size	g_totle_read_sector, 4
g_totle_read_sector:
	.space	4
	.type	g_GlobalSysVersion, %object
	.size	g_GlobalSysVersion, 4
g_GlobalSysVersion:
	.space	4
	.type	g_GlobalDataVersion, %object
	.size	g_GlobalDataVersion, 4
g_GlobalDataVersion:
	.space	4
	.type	g_totle_mlc_erase_count, %object
	.size	g_totle_mlc_erase_count, 4
g_totle_mlc_erase_count:
	.space	4
	.type	g_totle_slc_erase_count, %object
	.size	g_totle_slc_erase_count, 4
g_totle_slc_erase_count:
	.space	4
	.type	g_totle_avg_erase_count, %object
	.size	g_totle_avg_erase_count, 4
g_totle_avg_erase_count:
	.space	4
	.type	g_max_erase_count, %object
	.size	g_max_erase_count, 4
g_max_erase_count:
	.space	4
	.type	g_min_erase_count, %object
	.size	g_min_erase_count, 4
g_min_erase_count:
	.space	4
	.type	c_ftl_nand_data_op_blks_per_plane, %object
	.size	c_ftl_nand_data_op_blks_per_plane, 2
c_ftl_nand_data_op_blks_per_plane:
	.space	2
	.space	2
	.type	gSysInfo, %object
	.size	gSysInfo, 16
gSysInfo:
	.space	16
	.type	g_gc_superblock, %object
	.size	g_gc_superblock, 48
g_gc_superblock:
	.space	48
	.type	g_all_blk_used_slc_mode, %object
	.size	g_all_blk_used_slc_mode, 4
g_all_blk_used_slc_mode:
	.space	4
	.type	g_sys_ext_data, %object
	.size	g_sys_ext_data, 512
g_sys_ext_data:
	.space	512
	.type	g_gc_free_blk_threshold, %object
	.size	g_gc_free_blk_threshold, 2
g_gc_free_blk_threshold:
	.space	2
	.type	g_gc_merge_free_blk_threshold, %object
	.size	g_gc_merge_free_blk_threshold, 2
g_gc_merge_free_blk_threshold:
	.space	2
	.type	g_gc_skip_write_count, %object
	.size	g_gc_skip_write_count, 4
g_gc_skip_write_count:
	.space	4
	.type	g_gc_blk_index, %object
	.size	g_gc_blk_index, 2
g_gc_blk_index:
	.space	2
	.space	2
	.type	g_in_swl_replace, %object
	.size	g_in_swl_replace, 4
g_in_swl_replace:
	.space	4
	.type	g_gc_num_req, %object
	.size	g_gc_num_req, 4
g_gc_num_req:
	.space	4
	.type	gp_gc_page_buf_info, %object
	.size	gp_gc_page_buf_info, 4
gp_gc_page_buf_info:
	.space	4
	.type	p_gc_data_buf, %object
	.size	p_gc_data_buf, 4
p_gc_data_buf:
	.space	4
	.type	p_gc_spare_buf, %object
	.size	p_gc_spare_buf, 4
p_gc_spare_buf:
	.space	4
	.type	req_gc, %object
	.size	req_gc, 4
req_gc:
	.space	4
	.type	c_gc_page_buf_num, %object
	.size	c_gc_page_buf_num, 4
c_gc_page_buf_num:
	.space	4
	.type	p_gc_blk_tbl, %object
	.size	p_gc_blk_tbl, 4
p_gc_blk_tbl:
	.space	4
	.type	g_gc_blk_num, %object
	.size	g_gc_blk_num, 2
g_gc_blk_num:
	.space	2
	.space	2
	.type	p_gc_page_info, %object
	.size	p_gc_page_info, 4
p_gc_page_info:
	.space	4
	.type	g_gc_page_offset, %object
	.size	g_gc_page_offset, 2
g_gc_page_offset:
	.space	2
	.type	g_gc_next_blk, %object
	.size	g_gc_next_blk, 2
g_gc_next_blk:
	.space	2
	.type	g_gc_next_blk_1, %object
	.size	g_gc_next_blk_1, 2
g_gc_next_blk_1:
	.space	2
	.type	g_gc_next_blk_2, %object
	.size	g_gc_next_blk_2, 2
g_gc_next_blk_2:
	.space	2
	.type	g_gc_next_blk_3, %object
	.size	g_gc_next_blk_3, 2
g_gc_next_blk_3:
	.space	2
	.type	g_gc_bad_block_temp_num, %object
	.size	g_gc_bad_block_temp_num, 2
g_gc_bad_block_temp_num:
	.space	2
	.type	g_gc_bad_block_temp_tbl, %object
	.size	g_gc_bad_block_temp_tbl, 34
g_gc_bad_block_temp_tbl:
	.space	34
	.type	g_gc_bad_block_gc_index, %object
	.size	g_gc_bad_block_gc_index, 2
g_gc_bad_block_gc_index:
	.space	2
	.type	g_free_slc_blk_num, %object
	.size	g_free_slc_blk_num, 2
g_free_slc_blk_num:
	.space	2
	.space	2
	.type	g_SlcPartLbaEndSector, %object
	.size	g_SlcPartLbaEndSector, 4
g_SlcPartLbaEndSector:
	.space	4
	.type	g_in_gc_progress, %object
	.size	g_in_gc_progress, 4
g_in_gc_progress:
	.space	4
	.type	g_gc_head_data_block, %object
	.size	g_gc_head_data_block, 4
g_gc_head_data_block:
	.space	4
	.type	g_gc_head_data_block_count, %object
	.size	g_gc_head_data_block_count, 4
g_gc_head_data_block_count:
	.space	4
	.type	g_cur_erase_blk, %object
	.size	g_cur_erase_blk, 4
g_cur_erase_blk:
	.space	4
	.type	g_gc_refresh_block_temp_num, %object
	.size	g_gc_refresh_block_temp_num, 2
g_gc_refresh_block_temp_num:
	.space	2
	.space	2
	.type	c_wr_page_buf_num, %object
	.size	c_wr_page_buf_num, 4
c_wr_page_buf_num:
	.space	4
	.type	req_read, %object
	.size	req_read, 4
req_read:
	.space	4
	.type	req_gc_dst, %object
	.size	req_gc_dst, 4
req_gc_dst:
	.space	4
	.type	req_prgm, %object
	.size	req_prgm, 4
req_prgm:
	.space	4
	.type	p_sys_data_buf, %object
	.size	p_sys_data_buf, 4
p_sys_data_buf:
	.space	4
	.type	p_sys_data_buf_1, %object
	.size	p_sys_data_buf_1, 4
p_sys_data_buf_1:
	.space	4
	.type	p_vendor_data_buf, %object
	.size	p_vendor_data_buf, 4
p_vendor_data_buf:
	.space	4
	.type	p_wr_io_data_buf, %object
	.size	p_wr_io_data_buf, 4
p_wr_io_data_buf:
	.space	4
	.type	p_io_data_buf_0, %object
	.size	p_io_data_buf_0, 4
p_io_data_buf_0:
	.space	4
	.type	p_io_data_buf_1, %object
	.size	p_io_data_buf_1, 4
p_io_data_buf_1:
	.space	4
	.type	p_sys_spare_buf, %object
	.size	p_sys_spare_buf, 4
p_sys_spare_buf:
	.space	4
	.type	p_io_spare_buf, %object
	.size	p_io_spare_buf, 4
p_io_spare_buf:
	.space	4
	.type	p_wr_io_spare_buf, %object
	.size	p_wr_io_spare_buf, 4
p_wr_io_spare_buf:
	.space	4
	.type	g_ect_tbl_info_size, %object
	.size	g_ect_tbl_info_size, 2
g_ect_tbl_info_size:
	.space	2
	.space	2
	.type	p_swl_mul_table, %object
	.size	p_swl_mul_table, 4
p_swl_mul_table:
	.space	4
	.type	gp_ect_tbl_info, %object
	.size	gp_ect_tbl_info, 4
gp_ect_tbl_info:
	.space	4
	.type	p_valid_page_count_check_table, %object
	.size	p_valid_page_count_check_table, 4
p_valid_page_count_check_table:
	.space	4
	.type	p_map_block_table, %object
	.size	p_map_block_table, 4
p_map_block_table:
	.space	4
	.type	p_map_block_valid_page_count, %object
	.size	p_map_block_valid_page_count, 4
p_map_block_valid_page_count:
	.space	4
	.type	p_vendor_block_valid_page_count, %object
	.size	p_vendor_block_valid_page_count, 4
p_vendor_block_valid_page_count:
	.space	4
	.type	p_vendor_block_ver_table, %object
	.size	p_vendor_block_ver_table, 4
p_vendor_block_ver_table:
	.space	4
	.type	p_vendor_region_ppn_table, %object
	.size	p_vendor_region_ppn_table, 4
p_vendor_region_ppn_table:
	.space	4
	.type	p_map_region_ppn_table, %object
	.size	p_map_region_ppn_table, 4
p_map_region_ppn_table:
	.space	4
	.type	p_map_region_ppn_check_table, %object
	.size	p_map_region_ppn_check_table, 4
p_map_region_ppn_check_table:
	.space	4
	.type	p_map_block_ver_table, %object
	.size	p_map_block_ver_table, 4
p_map_block_ver_table:
	.space	4
	.type	p_l2p_map_buf, %object
	.size	p_l2p_map_buf, 4
p_l2p_map_buf:
	.space	4
	.type	c_ftl_nand_bbm_buf_size, %object
	.size	c_ftl_nand_bbm_buf_size, 2
c_ftl_nand_bbm_buf_size:
	.space	2
	.space	2
	.type	gL2pMapInfo, %object
	.size	gL2pMapInfo, 44
gL2pMapInfo:
	.space	44
	.type	g_totle_map_block, %object
	.size	g_totle_map_block, 2
g_totle_map_block:
	.space	2
	.space	2
	.type	g_req_cache, %object
	.size	g_req_cache, 4
g_req_cache:
	.space	4
	.type	g_tmp_data_superblock_id, %object
	.size	g_tmp_data_superblock_id, 2
g_tmp_data_superblock_id:
	.space	2
	.space	2
	.type	g_totle_swl_count, %object
	.size	g_totle_swl_count, 4
g_totle_swl_count:
	.space	4
	.type	ftl_gc_temp_power_lost_recovery_flag, %object
	.size	ftl_gc_temp_power_lost_recovery_flag, 4
ftl_gc_temp_power_lost_recovery_flag:
	.space	4
	.type	g_recovery_page_min_ver, %object
	.size	g_recovery_page_min_ver, 4
g_recovery_page_min_ver:
	.space	4
	.type	gNandIDataBuf, %object
	.size	gNandIDataBuf, 2048
gNandIDataBuf:
	.space	2048
	.type	RK29_NANDC_REG_BASE, %object
	.size	RK29_NANDC_REG_BASE, 4
RK29_NANDC_REG_BASE:
	.space	4
	.type	gFlashPageBuffer0, %object
	.size	gFlashPageBuffer0, 4
gFlashPageBuffer0:
	.space	4
	.type	FlashDdrTunningReadCount, %object
	.size	FlashDdrTunningReadCount, 4
FlashDdrTunningReadCount:
	.space	4
	.type	gpReadRetrial, %object
	.size	gpReadRetrial, 4
gpReadRetrial:
	.space	4
	.type	gpFlashSaveInfo, %object
	.size	gpFlashSaveInfo, 4
gpFlashSaveInfo:
	.space	4
	.type	gNandFlashInfoBlockAddr, %object
	.size	gNandFlashInfoBlockAddr, 4
gNandFlashInfoBlockAddr:
	.space	4
	.type	gNandFlashIdbBlockAddr, %object
	.size	gNandFlashIdbBlockAddr, 4
gNandFlashIdbBlockAddr:
	.space	4
	.type	gNandIDBResBlkNumSaveInFlash, %object
	.size	gNandIDBResBlkNumSaveInFlash, 1
gNandIDBResBlkNumSaveInFlash:
	.space	1
	.type	g_maxRetryCount, %object
	.size	g_maxRetryCount, 1
g_maxRetryCount:
	.space	1
	.type	gNandFlashIDBEccBits, %object
	.size	gNandFlashIDBEccBits, 1
gNandFlashIDBEccBits:
	.space	1
	.space	1
	.type	gFlashPageBuffer1, %object
	.size	gFlashPageBuffer1, 4
gFlashPageBuffer1:
	.space	4
	.type	gFlashSpareBuffer, %object
	.size	gFlashSpareBuffer, 4
gFlashSpareBuffer:
	.space	4
	.type	gFlashProgCheckBuffer, %object
	.size	gFlashProgCheckBuffer, 4
gFlashProgCheckBuffer:
	.space	4
	.type	gFlashProgCheckSpareBuffer, %object
	.size	gFlashProgCheckSpareBuffer, 4
gFlashProgCheckSpareBuffer:
	.space	4
	.type	g_nand_ecc_en, %object
	.size	g_nand_ecc_en, 1
g_nand_ecc_en:
	.space	1
	.type	gMultiPageReadEn, %object
	.size	gMultiPageReadEn, 1
gMultiPageReadEn:
	.space	1
	.space	2
	.type	FbbtBlk, %object
	.size	FbbtBlk, 16
FbbtBlk:
	.space	16
	.type	req_sys, %object
	.size	req_sys, 36
req_sys:
	.space	36
	.type	g_MaxLbn, %object
	.size	g_MaxLbn, 4
g_MaxLbn:
	.space	4
	.type	gVendorBlkInfo, %object
	.size	gVendorBlkInfo, 44
gVendorBlkInfo:
	.space	44
	.type	g_ect_tbl_power_up_flush, %object
	.size	g_ect_tbl_power_up_flush, 2
g_ect_tbl_power_up_flush:
	.space	2
	.space	2
	.type	check_valid_page_count_table, %object
	.size	check_valid_page_count_table, 8192
check_valid_page_count_table:
	.space	8192
	.type	g_power_lost_ecc_error_blk, %object
	.size	g_power_lost_ecc_error_blk, 2
g_power_lost_ecc_error_blk:
	.space	2
	.type	g_power_lost_recovery_flag, %object
	.size	g_power_lost_recovery_flag, 2
g_power_lost_recovery_flag:
	.space	2
	.type	g_recovery_page_num, %object
	.size	g_recovery_page_num, 4
g_recovery_page_num:
	.space	4
	.type	g_recovery_ppa_tbl, %object
	.size	g_recovery_ppa_tbl, 128
g_recovery_ppa_tbl:
	.space	128
	.type	gc_ink_free_return_value, %object
	.size	gc_ink_free_return_value, 2
gc_ink_free_return_value:
	.space	2
	.type	g_gc_cur_blk_valid_pages, %object
	.size	g_gc_cur_blk_valid_pages, 2
g_gc_cur_blk_valid_pages:
	.space	2
	.type	g_gc_cur_blk_max_valid_pages, %object
	.size	g_gc_cur_blk_max_valid_pages, 2
g_gc_cur_blk_max_valid_pages:
	.space	2
	.space	2
	.type	gp_last_act_superblock, %object
	.size	gp_last_act_superblock, 4
gp_last_act_superblock:
	.space	4
	.type	gc_discard_updated, %object
	.size	gc_discard_updated, 4
gc_discard_updated:
	.space	4
	.type	g_LowFormat, %object
	.size	g_LowFormat, 4
g_LowFormat:
	.space	4
	.type	gLoaderBootInfo, %object
	.size	gLoaderBootInfo, 4
gLoaderBootInfo:
	.space	4
	.type	RK29_NANDC1_REG_BASE, %object
	.size	RK29_NANDC1_REG_BASE, 4
RK29_NANDC1_REG_BASE:
	.space	4
	.type	g_ftl_nand_free_count, %object
	.size	g_ftl_nand_free_count, 4
g_ftl_nand_free_count:
	.space	4
	.type	last_cache_match_count, %object
	.size	last_cache_match_count, 4
last_cache_match_count:
	.space	4
	.type	idb_write_enable, %object
	.size	idb_write_enable, 1
idb_write_enable:
	.space	1
	.space	3
	.type	idb_buf, %object
	.size	idb_buf, 4
idb_buf:
	.space	4
	.type	idb_last_lba, %object
	.size	idb_last_lba, 4
idb_last_lba:
	.space	4
	.type	g_idb_buffer, %object
	.size	g_idb_buffer, 4
g_idb_buffer:
	.space	4
	.type	gpDrmKeyInfo, %object
	.size	gpDrmKeyInfo, 4
gpDrmKeyInfo:
	.space	4
	.type	SecureBootCheckOK, %object
	.size	SecureBootCheckOK, 4
SecureBootCheckOK:
	.space	4
	.type	SecureBootEn, %object
	.size	SecureBootEn, 4
SecureBootEn:
	.space	4
	.type	gpBootConfig, %object
	.size	gpBootConfig, 4
gpBootConfig:
	.space	4
	.type	gSnSectorData, %object
	.size	gSnSectorData, 512
gSnSectorData:
	.space	512
	.type	SecureBootUnlockTryCount, %object
	.size	SecureBootUnlockTryCount, 4
SecureBootUnlockTryCount:
	.space	4
	.type	gpVendor0Info, %object
	.size	gpVendor0Info, 4
gpVendor0Info:
	.space	4
	.type	gpVendor1Info, %object
	.size	gpVendor1Info, 4
gpVendor1Info:
	.space	4
	.type	g_vendor, %object
	.size	g_vendor, 4
g_vendor:
	.space	4
	.type	g_gc_refresh_block_temp_tbl, %object
	.size	g_gc_refresh_block_temp_tbl, 34
g_gc_refresh_block_temp_tbl:
	.space	34
	.space	2
	.type	gToggleModeClkDiv, %object
	.size	gToggleModeClkDiv, 4
gToggleModeClkDiv:
	.space	4
	.type	gpNandc1, %object
	.size	gpNandc1, 4
gpNandc1:
	.space	4
	.type	gNandFlashResEndPageAddr, %object
	.size	gNandFlashResEndPageAddr, 4
gNandFlashResEndPageAddr:
	.space	4
	.type	gNandFlashInfoBlockEcc, %object
	.size	gNandFlashInfoBlockEcc, 1
gNandFlashInfoBlockEcc:
	.space	1
	.type	gFlashOnfiModeEn, %object
	.size	gFlashOnfiModeEn, 1
gFlashOnfiModeEn:
	.space	1
	.type	gFlashSdrModeEn, %object
	.size	gFlashSdrModeEn, 1
gFlashSdrModeEn:
	.space	1
